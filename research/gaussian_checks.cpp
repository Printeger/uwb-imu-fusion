#include "gaussian_reference.hpp"
#include "gaussian_window.hpp"
#include "statistical_history.hpp"
#include "raw_domain.hpp"
#include "bounded_imu_recovery.hpp"
#include "imu_recovery_scope.hpp"
#include "scenario_material.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <stdexcept>
using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
void check(bool x){if(!x)throw std::runtime_error("Gaussian oracle check failed");}
int main(int argc,char** argv){
 if(argc!=2)throw std::runtime_error("usage: fde_next_checks config/fde_next.yaml");
 auto cfg=YAML::LoadFile(argv[1]);
 const double task=2*cfg["noise_tail_per_candidate"].as<double>()*cfg["max_candidates"].as<int>()*cfg["epochs"].as<int>();
 check(task<cfg["risk_budget"].as<double>());
 std::cout<<"model="<<cfg["model_version"].as<std::string>()<<" task_union_risk="<<task<<"\n";
 Eigen::MatrixXd a(5,2);a<<1,0,0,1,1,1,2,-1,-1,2;
 Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,2);c.topRows(2).setIdentity();
 Eigen::VectorXd z(5);z<<.12,-.08,.01,.03,-.02;
 Eigen::MatrixXd sigma=Eigen::MatrixXd::Identity(5,5);sigma(0,1)=sigma(1,0)=.4;
 Eigen::MatrixXd b=Eigen::MatrixXd::Zero(5,1);b(4,0)=1;
 auto r=gaussianReference(a,z,sigma,c,{b},1e-3,1e-6);
 check(r.status=="FROZEN_MODEL_ONLY" && r.pl.allFinite());
 auto exact=(a.transpose()*sigma.ldlt().solve(a)).ldlt().solve(a.transpose()*sigma.ldlt().solve(z)).eval();
 check((r.mean-exact).norm()<1e-12);
 auto wrong=gaussianReference(a,z,Eigen::MatrixXd::Identity(5,5),c,{b},1e-3,1e-6);
 check((wrong.mean-r.mean).norm()>1e-4);
 // One visited model is already enough to violate a fixed-model parity law:
 // fitting its column to the same noise forces its residual to zero. Candidate
 // count alone cannot charge this adaptive family as one fixed Gaussian root.
 Eigen::Vector3d noise(.4,-.3,.2);Eigen::MatrixXd fixed(3,1),fitted(3,1),point=Eigen::MatrixXd::Zero(3,1);
 fixed<<1,0,0;fitted=noise.normalized();point(0,0)=1.;
 auto fixed_noise=gaussianReference(fixed,noise,Eigen::Matrix3d::Identity(),point,{},1e-3,1e-6);
 auto adaptive_noise=gaussianReference(fitted,noise,Eigen::Matrix3d::Identity(),point,{},1e-3,1e-6);
 check(fixed_noise.statistic>.1 && adaptive_noise.statistic<1e-20);
 Eigen::MatrixXd safe_c=Eigen::MatrixXd::Zero(3,2);safe_c(0,1)=1;
 check(gaussianReference(a,z,sigma,safe_c,{a.col(0)},1e-3,1e-6).pl.allFinite());
 check(gaussianReference(a,z,sigma,c,{a.col(0)},1e-3,1e-6).status=="DANGEROUS_NULLSPACE");
 check(gaussianReference(a,z,sigma,c,{1e-10*b},1e-3,1e-6).status=="INDETERMINATE");
 std::vector<int> rows{1,2,3,4};check(principal(sigma,rows)(0,1)==0);
 Eigen::MatrixXd selected_a(4,2);Eigen::VectorXd selected_z(4);
 for(int i=0;i<4;++i){selected_a.row(i)=a.row(rows[i]);selected_z(i)=z(rows[i]);}
 auto deleted=gaussianReference(selected_a,selected_z,principal(sigma,rows),c,{},1e-3,1e-6);
 auto wa=(r.whitener*a).eval();auto wz=(r.whitener*z).eval();
 for(int i=0;i<4;++i){selected_a.row(i)=wa.row(rows[i]);selected_z(i)=wz(rows[i]);}
 auto mixed=gaussianReference(selected_a,selected_z,Eigen::MatrixXd::Identity(4,4),c,{},1e-3,1e-6);
 check((deleted.mean-mixed.mean).norm()>1e-4);
 Eigen::MatrixXd j(4,1);j<<1,2,3,4;auto joint=(j*j.transpose()).eval();check(joint(0,3)==4);
 check((principal(joint,rows={0,2})-(Eigen::Vector2d(1,3)*Eigen::Vector2d(1,3).transpose())).norm()==0);
 // Full Gaussian dimension survives condensation, including detector-only
 // directions even when [B,z] has low realized rank. Shared source/cross maps
 // must be transformed together; raw deletion reconstructs the new root.
 for(double correlation : {0., .4, -.3}) {
  auto cov=sigma;cov(0,1)=cov(1,0)=correlation;
  Eigen::MatrixXd src=Eigen::LLT<Eigen::MatrixXd>(cov).matrixL();
  Eigen::MatrixXd future=src.col(0);
  auto hist=statisticalHistory(a.leftCols(1),a.rightCols(1),z,cov,b,src,future);
  Eigen::MatrixXd cp=Eigen::MatrixXd::Zero(3,1);cp(0,0)=1;
  auto full=gaussianReference(a,z,cov,safe_c,{b},1e-3,1e-6);
  auto compact=gaussianReference(hist.a,hist.z,hist.sigma,cp,{hist.b},1e-3,1e-6);
  check(full.dof==compact.dof && hist.z.size()==4);
  check(std::abs(full.statistic-compact.statistic)<1e-12);
  check(std::abs(full.mean(1)-compact.mean(0))<1e-12);
  check(std::abs(full.covariance(1,1)-compact.covariance(0,0))<1e-12);
  check((hist.j*hist.j.transpose()-hist.sigma).norm()<1e-12);
  check((hist.cross-hist.transform*future).norm()<1e-12);
  auto rhs=z-a.col(1)*.2-b.col(0)*.3;
  auto white=Eigen::LLT<Eigen::MatrixXd>(cov).matrixL().solve(rhs).eval();
  auto ho=Eigen::LLT<Eigen::MatrixXd>(cov).matrixL().solve(a.leftCols(1)).eval();
  double x=(ho.col(0).dot(white))/ho.squaredNorm();
  check(std::abs((white-ho*x).squaredNorm()-(hist.z-hist.a.col(0)*.2-hist.b.col(0)*.3).squaredNorm())<1e-12);
 }
 // Uniform analytic remainder over a right Pose3 Expmap ball.
 RawNoiseEvent event(3421,1e-5);check(event.q>5. && event.q<7.);
 // Fixed fault semantics, arbitrary bad amplitude; the healthy branch must
 // enclose the tag for every predetermined +/- noise realization tested.
 std::vector<Eigen::Vector3d> anchors{{0,0,0},{6,0,0},{0,6,0},{0,0,6},{6,6,6},{6,0,6},{0,6,6},{6,6,0}};
 Eigen::Vector3d true_tag(2,3,1);double eps=.6;
 for(int fault=0;fault<8;++fault)for(double amplitude:{-20.,0.,20.}){
  std::vector<double> ranges;for(int i=0;i<8;++i)ranges.push_back((true_tag-anchors[i]).norm()+(i%2?eps:-eps)+(i==fault?amplitude:0.));
  auto branches=rangeDomainBranches(anchors,ranges,eps);check(branches.size()==8);
  auto& healthy=branches[fault];check(healthy.valid);
  check(((true_tag-healthy.center).cwiseAbs().array()<=healthy.box.array()+1e-12).all());
 }
 for(auto& anchor:anchors)anchor.z()=0.;
 check(!rangeDomainBranches(anchors,std::vector<double>(8,3.),eps)[0].valid);
 for(double theta:{.01,.2,.8})for(double rho:{.01,.1,.4}){
  auto bound=tagRemainder(5.,theta,rho,.25);check(bound.valid && bound.range>0.);
  gtsam::Pose3 center(gtsam::Rot3::RzRyRx(.1,.2,.3),gtsam::Point3(1,2,3));
  Eigen::Vector3d arm(.2,-.1,.08),anchor(-2,-1,0),dir(1,2,3);dir.normalize();
  Eigen::Matrix<double,6,1> tangent;tangent<<theta*dir,rho*dir;
  auto moved=center.retract(tangent);auto tag=(center.translation()+center.rotation().matrix()*arm).eval();
  auto delta=(center.rotation().matrix()*(tangent.tail<3>()+tangent.head<3>().cross(arm))).eval();
  double actual=std::abs((moved.translation()+moved.rotation().matrix()*arm-anchor).norm()-(tag-anchor).norm()-(tag-anchor).normalized().dot(delta));
  check(actual<=tagRemainder((tag-anchor).norm(),theta,rho,arm.norm()).range);
 }
 auto short_pim=healthyIntegrationRemainder(.05,.12,1.3,11.);
 auto long_pim=healthyIntegrationRemainder(1.55,.12,1.3,11.);
 check(short_pim.rotation>0 && long_pim.velocity>short_pim.velocity);
 check(healthyIntegrationRemainder(.05,0,1.3,11.).velocity==std::nextafter(0.,INFINITY));
 check(fixedReferenceFamilyCharge(5.,.25,50,128,1e-9)>cfg["risk_budget"].as<double>());
 check(!tagRemainder(.1,.2,.4,.25).valid);check(!tagRemainder(5.,3.2,.1,.25).valid);
 // Preserve full-state cross covariance; bounded acceleration is not random.
 Eigen::Matrix<double,15,15> boundary=Eigen::Matrix<double,15,15>::Identity();boundary(3,6)=boundary(6,3)=.2;boundary(0,3)=boundary(3,0)=.1;
 auto bridge=fullStateBridge(boundary,.3,4.,2.);
 check(std::abs(bridge.covariance(3,3)-1.21)<1e-12);
 check(std::abs(bridge.covariance(3,6)-.5)<1e-12 && bridge.covariance(0,3)==.1);
 check(std::abs(bridge.box(3)-.18)<1e-12 && std::abs(bridge.box(6)-1.2)<1e-12);
 // Independent body->tag right-local rotation derivative.
 Eigen::Vector3d lever(.2,-.1,.08);Eigen::Matrix3d rot=Eigen::AngleAxisd(.3,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
 Eigen::Matrix3d skew;skew<<0,-lever.z(),lever.y(),lever.z(),0,-lever.x(),-lever.y(),lever.x(),0;
 for(int j=0;j<3;++j){Eigen::Vector3d axis=Eigen::Vector3d::Unit(j);
 auto fd=(rot*Eigen::AngleAxisd(1e-6,axis).toRotationMatrix()*lever-rot*Eigen::AngleAxisd(-1e-6,axis).toRotationMatrix()*lever)/2e-6;
 check((fd+rot*skew.col(j)).norm()<1e-9);}
 check(riskFloor(3e-5,1e-7,{1e-4},.001,.001,1000)>4e-5);
 check(riskFloor(1e-6,0,{1e-4},.001,.001,9)<4e-5);
 // Native raw support check: a 50 ms sample episode contaminates both
 // factors; removing only the second factor cannot claim complete isolation.
 auto base_cfg=IntegrityConfigLoader::load(cfg["base_config"].as<std::string>());
 IncrementalUwbImuEstimator estimator(base_cfg,Eigen::Vector3d::Zero());
 estimator.initialize(initialState(),base_cfg.realtime.prior_sigmas);estimator.ingestImu(sample(base_cfg,0));
 auto first=prepare(estimator,base_cfg,1);auto keep=EpochCommitPlan::nominalPlan(first);estimator.commitEpoch(std::move(first),keep);
 auto second=prepare(estimator,base_cfg,2);IntegrityWindowRequest req;req.epochs=10;auto w=estimator.buildIntegrityWindow(second,req);
 HypothesisGeneratorConfig gc;gc.include_uwb_faults=false;
 auto templates=HypothesisGenerator(gc).generate(w,second,ImuFaultSubspaceBuilder{}.buildAnalytic(second,estimator.buildPendingFactorBlock(second,second.imu_group.id)),{});
 auto episodes=imuEpisodeModels(templates,w,second,{50,500});
 auto it=std::find_if(episodes.modes.begin(),episodes.modes.end(),[](const auto& m){return m.onset_epoch==0 && m.physical_source_id.find("50ms")!=std::string::npos;});
 check(it!=episodes.modes.end());check(it->affected_groups.size()==2);check(it->affected_measurements.size()==10);
 ExclusionAction incomplete;incomplete.groups_to_remove={second.imu_group.id};check(!episodeCoverageComplete(*it,incomplete));
 incomplete.groups_to_remove=it->affected_groups;check(episodeCoverageComplete(*it,incomplete));
 auto missing=*it;missing.recoverability=HistoryRecoverability::MissingProvenance;check(!episodeCoverageComplete(missing,incomplete));
 check(bridgeMotionDomain(.03,.2,4.,2.));check(!bridgeMotionDomain(5.4,.2,4.,2.));
 auto native_joint=rawGaussianWindow(w,second,episodes.modes,.02,.001,Eigen::Vector3d(.2,-.1,.08));
 auto shared_joint=rawGaussianWindow(w,second,episodes.modes,.02,.001,Eigen::Vector3d(.2,-.1,.08),true);
 check((shared_joint.sigma-native_joint.sigma).norm()<1e-18);
 check(shared_joint.noise_source_ids==native_joint.noise_source_ids);
 check((shared_joint.noise_j-native_joint.noise_j).norm()==0);
 check(native_joint.cross_norm>1e-12);check(Eigen::LLT<Eigen::MatrixXd>(native_joint.sigma).info()==Eigen::Success);
 NoisyMotionScenario s0(argv[1],"U0",20261010,2),s1(argv[1],"U0",20261010,2);
 check(s0.imu[10].specific_force_mps2==s1.imu[10].specific_force_mps2);
 check(s0.uwb[1].measurements[0].range_m==s1.uwb[1].measurements[0].range_m);
 auto batched=gaussianReference(a,z,sigma,c,{b},1e-3,1e-6,true);
 check((batched.mean-r.mean).norm()==0);check((batched.covariance-r.covariance).norm()==0);
 check((batched.pl.array()>=r.pl.array()-1e-12).all());check((batched.pl-r.pl).norm()<1e-9);
 check(gaussianReference(a,z,sigma,c,{a.col(0)},1e-3,1e-6,true).status=="DANGEROUS_NULLSPACE");
 check(gaussianReference(a,z,sigma,c,{1e-10*b},1e-3,1e-6,true).status=="INDETERMINATE");
 auto asymmetric=sigma;asymmetric(0,1)+=.1;
 check(gaussianReference(a,z,asymmetric,c,{},1e-3,1e-6).status=="INDETERMINATE");
 auto malformed=b;malformed(0,0)=NAN;check(gaussianReference(a,z,sigma,c,{malformed},1e-3,1e-6,true).status=="INDETERMINATE");
 ExclusionAction keep_all;keep_all.id=ExclusionActionId(1);
 auto base=gaussianReference(native_joint.a,native_joint.z,native_joint.sigma,native_joint.c,{},1e-6,1e-9);
 GaussianCandidateCache cache(native_joint,w,1e-6,1e-9);size_t decompositions=1;
 auto reference=gaussianCandidate(native_joint,w,keep_all,1e-6,1e-9);
 auto cached=gaussianCandidate(native_joint,w,keep_all,1e-6,1e-9,true,&base,&cache,&decompositions);
 check(reference.status==cached.status);check((reference.mean-cached.mean).norm()<1e-12);
 keep_all.id=ExclusionActionId(2);auto again=gaussianCandidate(native_joint,w,keep_all,1e-6,1e-9,true,&base,&cache,&decompositions);
 check((again.pl.array()==cached.pl.array()).all());check(decompositions==1 && cache.entries.size()==1);
 auto foreign=native_joint;bool rejected=false;try{gaussianCandidate(foreign,w,keep_all,1e-6,1e-9,true,&base,&cache);}catch(const std::runtime_error&){rejected=true;}check(rejected);
 check(gaussianReference(a,z,sigma,c,{1e-11*a.col(0)},1e-3,1e-6).status=="INDETERMINATE");
 check(gaussianReference(a,z,sigma,c,{1e-11*a.col(0)},1e-3,1e-6,true).status=="INDETERMINATE");
 // Fixed semantic exclusion: changing all removed ADC samples must not
 // change retained nonlinear reference, bridge boundary, or its covariance.
 BoundedRecoveryAttempt independent[2];
 for(int sign=0;sign<2;++sign){
  NoisyMotionScenario noisy(argv[1],"I2",20261010,20);
  if(sign)for(auto& sample:noisy.imu)if(sample.timestamp.seconds()>=.705-1e-12 && sample.timestamp.seconds()<1.205-1e-12)sample.specific_force_mps2.x()-=40.;
  IncrementalUwbImuEstimator backend(noisy.cfg,noisy.lever);backend.initialize(noisy.initial_observation,noisy.cfg.realtime.prior_sigmas);backend.ingestImu(noisy.imu[0]);size_t cursor=1;
  for(int e=1;e<=20;++e){while(cursor<noisy.imu.size() && !(noisy.uwb[e-1].timestamp<noisy.imu[cursor].timestamp))backend.ingestImu(noisy.imu[cursor++]);
   auto t=backend.prepareEpoch(noisy.uwb[e-1]);
   if(e==20){ExclusionAction action;action.bridge_mode=BridgeMode::GenericKinematic;
    for(const auto& past:t.recoverable_history)if(past.proposed_epoch>=15)for(const auto& group:past.groups)if(group.kind==FactorKind::CombinedImu)action.groups_to_remove.push_back(group.id);
    action.groups_to_remove.push_back(t.imu_group.id);
    independent[sign]=boundedRecovery(t,action,noisy.initial_observation,4.,2.,&noisy.imu,&noisy.cfg.bridge.generic);
    check(independent[sign].rebuilds==2 && independent[sign].status=="BRIDGE_DOMAIN_EXCEEDS_LOCAL_MODEL");
    check(independent[sign].healthy_rank<independent[sign].healthy_columns);
    check(std::abs(independent[sign].full_state_domain_ratio-2.4)<1e-12);
   }
   auto p=EpochCommitPlan::nominalPlan(t);check(backend.commitEpoch(std::move(t),p).backend_updates==1);
  }
 }
 check(std::abs(independent[0].objective_after-independent[1].objective_after)<1e-10);
 check((independent[0].bridge.covariance-independent[1].bridge.covariance).norm()<1e-10);
 for(int e=0;e<=20;++e){check(independent[0].reference.at<gtsam::Pose3>(gtsam::Symbol('x',e)).localCoordinates(independent[1].reference.at<gtsam::Pose3>(gtsam::Symbol('x',e))).norm()<1e-10);
  check((independent[0].reference.at<gtsam::Vector3>(gtsam::Symbol('v',e))-independent[1].reference.at<gtsam::Vector3>(gtsam::Symbol('v',e))).norm()<1e-10);
  check((independent[0].reference.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',e)).vector()-independent[1].reference.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',e)).vector()).norm()<1e-10);
 }
 std::cout<<"legacy_checks=43 history_parameter_cases=3 finite_pl="<<r.pl.transpose()<<" statistic="<<r.statistic<<" threshold="<<r.threshold<<"\n";
}
