#include "fused_epoch_pl.hpp"
#include "scenario_material.hpp"
#include <gtsam/inference/Symbol.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
PredictionConditionsV1 valid(){return {true,true,true,true,true};}
Eigen::Vector3d committedPerturb(const std::string& path,int target,double perturb){
 NoisyMotionScenario scene(path,"U0",20261010,target);IncrementalUwbImuEstimator est(scene.cfg,scene.lever);
 est.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);est.ingestImu(scene.imu[0]);
 for(int e=1;e<=target;++e){for(int i=(e-1)*10+1;i<=e*10;++i)est.ingestImu(scene.imu[i]);auto b=scene.uwb[e-1];if(e==target)b.measurements[0].range_m+=perturb;
 auto tx=est.prepareEpoch(b,EpochPreparationOptions::nominalOnly());auto plan=EpochCommitPlan::nominalPlan(tx);est.commitEpoch(std::move(tx),plan);}
 auto s=est.currentState();return s.position_world_m+s.q_world_body*scene.lever;
}
int main(int argc,char** argv){try{
 NoisyMotionScenario scene(argv[1],"U0",20261010,15);IncrementalUwbImuEstimator est(scene.cfg,scene.lever);
 est.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);est.ingestImu(scene.imu[0]);
 Eigen::MatrixXd response14;
 for(int e=1;e<=15;++e){
 for(int i=(e-1)*10+1;i<=e*10;++i)est.ingestImu(scene.imu[i]);auto b=scene.uwb[e-1];if(e==15)b.measurements[0].range_m+=2.25;
 auto tx=est.prepareEpoch(b,EpochPreparationOptions{false,true,false,false});auto pred=est.researchPreUwbPredictionV1(tx);
 require(est.backendUpdateCount()==pred.backend_updates && pred.excluded_batch_id==b.id,"read only/source lineage");
 auto sameTx=tx;for(auto& m:sameTx.uwb_batch.measurements)m.range_m+=100.;auto same=est.researchPreUwbPredictionV1(sameTx);
 require((same.prior.covariance-pred.prior.covariance).norm()==0 && (same.prior.mean.position_world_m-pred.prior.mean.position_world_m).norm()==0,"current range independent prediction");
 if(e>=14){
 int n=b.measurements.size();auto p=pred.prior.covariance;auto state=pred.prior.mean;auto c=tagMap(state,scene.lever);
 gtsam::Pose3 pose(gtsam::Rot3(state.q_world_body.toRotationMatrix()),state.position_world_m);gtsam::Matrix hp;Eigen::VectorXd innovation=-UwbPoseBatchFactor(1,b,scene.lever).evaluateError(pose,hp);
 Eigen::MatrixXd h=Eigen::MatrixXd::Zero(n,15);h.leftCols(6)=hp;Eigen::MatrixXd s=h*p*h.transpose()+validatedUwbCovariance(b);
 Eigen::MatrixXd k=p*h.transpose()*s.inverse();auto all=fusedCandidate(b,-1,pred,scene.lever,valid(),1e-6,1e-9);
 require(std::abs(all.gaussian.statistic-innovation.dot(s.ldlt().solve(innovation)))<1e-8,"NIS vs joint parity");
 require(all.gaussian.dof==n,"NIS dof");require((all.response-c*k).norm()<1e-9,"gain response vs SVD");
 require((all.gaussian.covariance-(p-k*h*p)).norm()<1e-9,"posterior covariance");
 // Independent uncondensed 30-state QR oracle; same Gaussian subproblem as
 // historical reference, current single-range faults only.
 Eigen::LLT<Eigen::MatrixXd> prev(pred.previous_covariance);Eigen::MatrixXd wp=prev.matrixL().solve(Eigen::MatrixXd::Identity(15,15));
 Eigen::LLT<Eigen::MatrixXd> range(validatedUwbCovariance(b));Eigen::MatrixXd wr=range.matrixL().solve(Eigen::MatrixXd::Identity(n,n));
 Eigen::MatrixXd a=Eigen::MatrixXd::Zero(30+n,30);a.topLeftCorner(15,15)=wp;a.middleRows(15,15)=pred.transition_design;a.bottomRightCorner(n,15)=wr*h;
 Eigen::VectorXd z=Eigen::VectorXd::Zero(30+n);z.tail(n)=wr*innovation;
 auto qr=a.colPivHouseholderQr();Eigen::VectorXd dense=qr.solve(z);
 require((dense.tail(15)-all.gaussian.mean).norm()<1e-8,"Schur vs uncondensed QR mean");
 Eigen::MatrixXd fault=Eigen::MatrixXd::Zero(30+n,n);fault.bottomRows(n)=wr;
 Eigen::MatrixXd fullResponse=qr.solve(fault);require((c*fullResponse.bottomRows(15)-all.response).norm()<1e-8,"Schur vs full QR fault response");
 // Frozen range perturbation (exact); actual nonlinear replay measured below.
 Eigen::VectorXd zp=z;zp(30)+=1e-5*wr(0,0);for(int j=1;j<n;++j)zp(30+j)+=1e-5*wr(j,0);
 Eigen::VectorXd dp=qr.solve(zp);require((c*(dp-dense).tail(15)/1e-5-all.response.col(0)).norm()<1e-7,"frozen finite difference");
 Eigen::MatrixXd cc=Eigen::MatrixXd::Zero(3,30);cc.rightCols(15)=c;
 std::vector<Eigen::MatrixXd> faults;for(int i=0;i<n;++i)faults.push_back(fault.col(i));
 auto full=gaussianReference(a,z,Eigen::MatrixXd::Identity(30+n,30+n),cc,faults,1e-6,1e-9);
 if(e==14)require((full.pl-all.gaussian.pl).norm()<1e-8,"same scope PL equality");
 for(int j=0;j<6;++j){gtsam::Vector6 dx=gtsam::Vector6::Zero();dx(j)=1e-6;Eigen::Vector3d fd=(pose.retract(dx).transformFrom(scene.lever)-pose.retract(-dx).transformFrom(scene.lever))/2e-6;require((fd-c.col(j)).norm()<1e-8,"lever rotation map");}
 for(int j=0;j<5;++j){auto bad=valid();if(j==0)bad.covariance_certified=false;if(j==1)bad.independent_current_noise=false;if(j==2)bad.clean_history=false;if(j==3)bad.clean_imu=false;if(j==4)bad.no_current_reuse=false;require(fusedEpochPl(b,pred,scene.lever,bad).status==predictionRefusal(bad),"negative prediction condition");}
 auto invalid=pred;invalid.prior.covariance(0,0)=-1;require(fusedEpochPl(b,invalid,scene.lever,valid()).status=="PREDICTION_COVARIANCE_UNQUALIFIED","invalid covariance");
 invalid=pred;invalid.excluded_batch_id=BatchId(999);require(fusedEpochPl(b,invalid,scene.lever,valid()).status=="PREDICTION_LINEAGE_FAILED","wrong batch");
 if(e==14)response14=all.response;
 std::cout<<"snapshot "<<e<<" NIS="<<all.gaussian.statistic<<" dof="<<all.gaussian.dof<<" Schur/QR response_error="<<(c*fullResponse.bottomRows(15)-all.response).norm()<<"\n";
 }
 auto r=fusedEpochPl(b,pred,scene.lever,valid());auto plan=EpochCommitPlan::nominalPlan(tx);
 if(e==15){require(r.alarm && r.excluded==0 && r.status=="FROZEN_MODEL_ONLY","B real recovery");for(auto& g:tx.uwb_groups)if(!g.nominal && g.source_measurements.size()==b.measurements.size()-1 && std::find(g.source_measurements.begin(),g.source_measurements.end(),b.measurements[0].id)==g.source_measurements.end())plan.groups_to_add={tx.imu_group.id,g.id};}
 auto receipt=est.commitEpoch(std::move(tx),plan);require(receipt.backend_updates==1,"single commit");
 }
 double eps=1e-4;Eigen::Vector3d actual=(committedPerturb(argv[1],14,eps)-committedPerturb(argv[1],14,-eps))/(2*eps);
 double difference=(actual-response14.col(0)).norm();std::cout<<"actual nonlinear finite difference="<<actual.transpose()<<" frozen="<<response14.col(0).transpose()<<" difference="<<difference<<"\n";
 require(difference<.03,"actual response domain comparison");
 std::cout<<"B source/independence, NIS, Schur QR, frozen FD, lever, refusal, native recovery: PASS\n";
 return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
