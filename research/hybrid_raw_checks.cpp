#include "scenario_material.hpp"
#include "raw_source_law.hpp"
#include "raw_nonlinear_inclusion.hpp"
#include "raw_prefix_history.hpp"
#include "hybrid_coordinator.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void check(bool v,const char* s){if(!v)throw std::runtime_error(s);}
int main(int argc,char** argv){try{
 std::string path=argc>1?argv[1]:"config/hybrid_raw.yaml";
 NoisyMotionScenario scene(path,"U0",20261010,5);
 check(scene.initial_law_valid && scene.physical_source_reference,"matched law");
 auto inventory=rawSourceInventory(50,8,scene.cfg.realtime.prior_sigmas,.02,.001,scene.cfg.imu.accelerometer_bias_rw_sigma,scene.cfg.imu.gyroscope_bias_rw_sigma,.1);
 std::set<std::string> ids;for(const auto& s:inventory)check(ids.insert(s.id).second,"unique primitive ID");check(ids.size()==6421,"full horizon unique dimension");
 // Shared point-sample trapezoid oracle for a parameterized scalar family.
 for(double dt:{.005,.02,.1})for(double variance:{.0004,.01,1.}){
  Eigen::Matrix<double,2,3> map;map<<dt/2,dt/2,0,0,dt/2,dt/2;
  Eigen::Matrix2d cov=variance*map*map.transpose();
  check(std::abs(cov(0,0)-variance*dt*dt/2)<1e-14 && std::abs(cov(0,1)-variance*dt*dt/4)<1e-14,"shared endpoint units");
  // Constant bias marginalization is t^2 Var(b), fresh increments t Var(b).
  check(std::abs(std::pow(10*dt,2)*variance-100*dt*dt*variance)<1e-13,"latent bias analytic oracle");
 }
 IncrementalUwbImuEstimator backend(scene.cfg,scene.lever);backend.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);backend.ingestImu(scene.imu[0]);
 OnlineGaussianHistory online(2,3,.02,.001,scene.lever,true,64u*1024u*1024u,true);RawPrefixHistory prefix;
 RawInclusionParameters param;param.numeric=scene.numeric_certificate;param.initial_sigmas=scene.cfg.realtime.prior_sigmas;param.gravity=scene.cfg.imu.gravity_mps2;
 check(scene.numeric_certificate.valid && scene.numeric_certificate.accel_sample<1e-10,"directed source forward-map enclosure");
 // Parameterized deterministic source-event boundary family, independent of
 // the motion replay. No random draws or corner-as-continuous-proof claim.
 for(double sign:{-1.,1.}){
  NavigationState observation;observation.position_world_m=Eigen::Vector3d::Constant(sign*3.5);observation.velocity_world_mps=Eigen::Vector3d::Constant(sign*3.5);observation.accel_bias_mps2=Eigen::Vector3d::Constant(sign*.7);observation.gyro_bias_radps=Eigen::Vector3d::Constant(sign*.07);
  RawInclusionParameters family;family.initial_sigmas=scene.cfg.realtime.prior_sigmas;family.numeric.valid=true;RawNonlinearInclusion outer(observation,family);
  for(int i=0;i<=500;++i){auto m=sample(scene.cfg,i);double drift=sign*i*7*.001*std::sqrt(.005);m.specific_force_mps2.array()+=drift+sign*7*.02;m.angular_velocity_radps.array()+=sign*i*7*.0001*std::sqrt(.005)+sign*7*.001;outer.append(m);}
  Eigen::Vector3d trueTag=scene.lever;auto outerBound=outer.tag(scene.lever,Eigen::Vector3d::Constant(sign*100));check(outerBound.valid && (trueTag.array()>=outerBound.low.array()).all() && (trueTag.array()<=outerBound.high.array()).all(),"boundary source/RW/shared-endpoint full horizon inclusion");
  auto extra=sample(scene.cfg,501);bool refused=false;try{outer.append(extra);}catch(const std::exception&){refused=true;}check(refused,"source horizon expiry");
 }
 auto smallBudget=param;smallBudget.risk_budget=1e-7;RawNonlinearInclusion unpaid(scene.initial_observation,smallBudget);for(int i=0;i<=10;++i)unpaid.append(scene.imu[i]);check(!unpaid.tag(scene.lever,Eigen::Vector3d::Zero()).valid,"raw event cannot exceed registered budget");
 RawNonlinearInclusion inclusion(scene.initial_observation,param);inclusion.append(scene.imu[0]);
 double maxCov=0,maxCross=0,maxSourceCov=0,maxSourceCross=0;Eigen::MatrixXd previousEndpoint;FactorGroupId previousImu;
 for(int epoch=1;epoch<=5;++epoch){for(int i=(epoch-1)*10+1;i<=epoch*10;++i){backend.ingestImu(scene.imu[i]);inclusion.append(scene.imu[i]);}
  auto tx=backend.prepareEpoch(scene.uwb[epoch-1],EpochPreparationOptions{true,true,false,false});prefix.observe(tx,2);online.prepare(tx,false);
  auto window=prefix.window(tx);auto material=prefix.material(tx);auto raw=rawGaussianWindow(window,material,{},.02,.001,scene.lever,false,true);
  double radius=std::sqrt(boost::math::quantile(boost::math::complement(boost::math::chi_squared(6421),1e-6)));
  auto evidence=online.preUwbEvidence(tx,radius);
  if(epoch==1 || epoch==5){auto a=uwbEpochRaim(scene.uwb[epoch-1]);auto old=hybridRawCurrent(scene.uwb[epoch-1],evidence,scene.lever,a,true,1e-6);auto joint=hybridRawCurrent(scene.uwb[epoch-1],evidence,scene.lever,a,true,1e-6,nullptr,nullptr,true);check(old.excluded==joint.excluded && (joint.gaussian.pl.array()<=old.gaussian.pl.array()+1e-8).all(),"same-law frozen envelope snapshot");std::cout<<"same_law_frozen_snapshot epoch="<<epoch<<" old="<<old.gaussian.pl.head<2>().norm()<<"/"<<old.gaussian.pl.z()<<" joint="<<joint.gaussian.pl.head<2>().norm()<<"/"<<joint.gaussian.pl.z()<<" guarantee=FROZEN_LINEAR_DIAGNOSTIC\n";}

  std::vector<int> kept;for(size_t k=0;k<window.blocks.size();++k){bool current=false;for(const auto& g:tx.uwb_groups)current=current || (g.nominal && g.id==window.blocks[k].group_id);if(!current)for(int r=0;r<window.blocks[k].residual_raw.size();++r)kept.push_back(raw.offsets[k]+r);}
  Eigen::MatrixXd a(kept.size(),raw.a.cols());Eigen::VectorXd z(kept.size());for(size_t i=0;i<kept.size();++i){a.row(i)=raw.a.row(kept[i]);z(i)=raw.z(kept[i]);}
  Eigen::LLT<Eigen::MatrixXd> white(principal(raw.sigma,kept));check(white.info()==Eigen::Success,"physical covariance SPD without jitter");
  Eigen::MatrixXd aw=white.matrixL().solve(a);Eigen::VectorXd zw=white.matrixL().solve(z);
  auto qr=aw.colPivHouseholderQr();Eigen::MatrixXd gain=qr.solve(Eigen::MatrixXd::Identity(aw.rows(),aw.rows()));Eigen::MatrixXd cov=gain*gain.transpose();Eigen::VectorXd mean=gain*zw;
  double rel=(cov.bottomRightCorner(15,15)-evidence.history.covariance).norm()/evidence.history.covariance.norm();maxCov=std::max(maxCov,rel);maxCross=std::max(maxCross,raw.cross_norm);
  check(rel<1e-6 && (mean.tail(15)-evidence.correction).norm()<1e-6,"physical source prefix/retirement full15 oracle");
  // Independent residual oracle mutates literal input samples and terminal
  // bias, then uses the public reintegration entry, not its RW map builder.
  auto sourceTx=prefix.material(tx);auto trueValues=std::make_shared<gtsam::Values>(*sourceTx.frozen_values);
  for(size_t e=0;e<=tx.proposed_epoch;++e){auto trueState=scene.truth(e*.05);trueValues->update(gtsam::Symbol('x',e),gtsam::Pose3(gtsam::Rot3(trueState.q_world_body.toRotationMatrix()),trueState.position_world_m));trueValues->update(gtsam::Symbol('v',e),gtsam::Vector3(trueState.velocity_world_mps));trueValues->update(gtsam::Symbol('b',e),gtsam::imuBias::ConstantBias(trueState.accel_bias_mps2,trueState.gyro_bias_radps));}sourceTx.frozen_values=trueValues;
  auto rw=physicalBiasIncrementMap(sourceTx);
  Eigen::MatrixXd independentRw(15,60);
  for(size_t k=1;k<=10;++k)for(int axis=0;axis<6;++axis){auto plus=sourceTx,minus=sourceTx;double eps=2e-5;
   auto mutate=[&](EpochTransaction& t,double d){for(size_t i=k;i<t.raw_imu_slice.size();++i){if(axis<3)t.raw_imu_slice[i].specific_force_mps2(axis)+=d;else t.raw_imu_slice[i].angular_velocity_radps(axis-3)+=d;}auto values=std::make_shared<gtsam::Values>(*t.frozen_values);auto key=gtsam::Symbol('b',t.proposed_epoch);auto bias=values->at<gtsam::imuBias::ConstantBias>(key).vector();bias(axis)+=d;values->update(key,gtsam::imuBias::ConstantBias(bias.head<3>(),bias.tail<3>()));t.frozen_values=values;};
   mutate(plus,eps);mutate(minus,-eps);RawImuEpisode empty{"oracle",tx.begin,TimestampNs(tx.begin.value()+1),0};
   auto jp=episodeFactorError(plus,integrateEpisode(plus,empty,0));auto jm=episodeFactorError(minus,integrateEpisode(minus,empty,0));double sig=(axis<3?scene.cfg.imu.accelerometer_bias_rw_sigma:scene.cfg.imu.gyroscope_bias_rw_sigma)*std::sqrt(.005);
   independentRw.col(6*(k-1)+axis)=-(jp-jm)/(2*eps)*sig;
   check((independentRw.col(6*(k-1)+axis)-rw.col(6*(k-1)+axis)).norm()<1e-6*rw.col(6*(k-1)+axis).norm()+1e-14,"independent full15 RW source residual");
  }
  // Independent covariance assembler uses literal primitive perturbations at
  // truth, all ADC/RW columns, and does not read the covariance under test.
  Eigen::MatrixXd independentAdc(15,66);
  for(size_t k=0;k<=10;++k)for(int axis=0;axis<6;++axis){auto plus=sourceTx,minus=sourceTx;double eps=2e-5;
   if(axis<3){plus.raw_imu_slice[k].specific_force_mps2(axis)+=eps;minus.raw_imu_slice[k].specific_force_mps2(axis)-=eps;}
   else{plus.raw_imu_slice[k].angular_velocity_radps(axis-3)+=eps;minus.raw_imu_slice[k].angular_velocity_radps(axis-3)-=eps;}
   RawImuEpisode empty{"independent-adc",tx.begin,TimestampNs(tx.begin.value()+1),0};
   independentAdc.col(6*k+axis)=-(episodeFactorError(plus,integrateEpisode(plus,empty,0))-episodeFactorError(minus,integrateEpisode(minus,empty,0)))/(2*eps)*(axis<3?.02:.001);
  }
  auto oneWindow=window;oneWindow.blocks.clear();for(const auto& block:window.blocks)if(block.group_id==tx.imu_group.id)oneWindow.blocks.push_back(block);
  auto tested=rawGaussianWindow(oneWindow,sourceTx,{},.02,.001,scene.lever,false,true);
  Eigen::MatrixXd expected=independentAdc*independentAdc.transpose()+independentRw*independentRw.transpose();
  Eigen::VectorXd scale=expected.diagonal().cwiseSqrt().cwiseInverse();
  double sourceDifference=(scale.asDiagonal()*(tested.sigma-expected)*scale.asDiagonal()).norm();maxSourceCov=std::max(maxSourceCov,sourceDifference);check(sourceDifference<1e-6,"independent units-normalized full15 source covariance");
  if(epoch>1){auto twoWindow=window;twoWindow.blocks.clear();for(auto id:{previousImu,tx.imu_group.id})for(const auto& block:window.blocks)if(block.group_id==id)twoWindow.blocks.push_back(block);
   auto pair=rawGaussianWindow(twoWindow,sourceTx,{},.02,.001,scene.lever,false,true);
   Eigen::MatrixXd cross=previousEndpoint*independentAdc.leftCols(6).transpose();double difference=(pair.sigma.topRightCorner(15,15)-cross).norm()/cross.norm();maxSourceCross=std::max(maxSourceCross,difference);check(difference<1e-6,"independent adjacent full15 source cross covariance");}
  previousEndpoint=independentAdc.rightCols(6);previousImu=tx.imu_group.id;
  auto plan=EpochCommitPlan::nominalPlan(tx);auto receipt=backend.commitEpoch(EpochTransaction(tx),plan);check(receipt.backend_updates==1,"actual source commit");
  auto actual=backend.currentState();auto enclosed=inclusion.issue(tx,receipt,actual,scene.lever);check(enclosed.valid && enclosed.event_risk<=1e-6 && enclosed.hpl>=std::hypot(enclosed.pl.x(),enclosed.pl.y()),"raw continuous inclusion actual receipt and outward horizontal support");
  auto truth=scene.truth(epoch*.05);Eigen::Vector3d trueTag=truth.position_world_m+truth.q_world_body*scene.lever;
  check((trueTag.array()>=enclosed.low.array()).all() && (trueTag.array()<=enclosed.high.array()).all(),"independent raw inclusion realization");
  check(!inclusion.tag(scene.lever,Eigen::Vector3d::Constant(1e308)).valid,"raw horizontal output overflow refuses finite qualification");
  auto forged=receipt;forged.backend_updates=0;check(!inclusion.issue(tx,forged,actual,scene.lever).valid,"inclusion receipt tamper");
  auto altered=tx;altered.raw_imu_slice[0].specific_force_mps2.x()+=1;check(!inclusion.issue(altered,receipt,actual,scene.lever).valid,"inclusion raw source tamper");
  for(double offset:{-100.,0.,100.}){Eigen::Vector3d center(offset,-offset,offset);auto outside=inclusion.tag(scene.lever,center);check(((trueTag-center).cwiseAbs().array()<=outside.pl.array()).all(),"arbitrary adaptive center uniform transfer");}
online.committed(tx,plan);prefix.committed(tx,plan);
 }
 check(maxCross>0,"adjacent PIM shared source cross");
 std::cout<<"raw source inventory="<<ids.size()<<" full15 RW/ADC/cross/history oracle PASS max_cov_relative="<<maxCov<<" cross="<<maxCross<<" source_cov_normalized="<<maxSourceCov<<" source_cross_relative="<<maxSourceCross<<" numerical_source_accel="<<scene.numeric_certificate.accel_sample<<" continuous_inclusion=PASS\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
