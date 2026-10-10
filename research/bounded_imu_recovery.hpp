#pragma once
#include "imu_recovery_scope.hpp"
#include "uwb_imu_pl/factors/kinematic_bridge_factor.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/navigation/NavState.h>
#include <set>
#include <Eigen/Eigenvalues>
namespace uwb_imu_pl::research {
struct FullStateBridge {
 Eigen::Matrix<double,15,15> transition=Eigen::Matrix<double,15,15>::Constant(NAN),covariance=Eigen::Matrix<double,15,15>::Constant(NAN);
 Eigen::Matrix<double,15,1> box=Eigen::Matrix<double,15,1>::Constant(NAN);
};
// World-frame acceleration box already includes force/gravity/attitude
// conversion. No suspect accelerometer or gyro appears in this propagation.
// Bias drift is zero only under the declared constant-bias generator law.
inline FullStateBridge fullStateBridge(const Eigen::Matrix<double,15,15>& p,
 double t,double accel,double rate){
 if(t<0 || accel<0 || rate<0 || !p.allFinite() || !p.isApprox(p.transpose(),1e-12))
  throw std::runtime_error("invalid bridge boundary");
 Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,15,15>> eig(p);
 if(eig.info()!=Eigen::Success || eig.eigenvalues().minCoeff()<0)throw std::runtime_error("bridge boundary covariance is not PSD");
 FullStateBridge o;o.transition.setIdentity();o.transition.block<3,3>(3,6)=t*Eigen::Matrix3d::Identity();
 o.covariance=o.transition*p*o.transition.transpose();o.box.setZero();
 o.box.head<3>().setConstant(rate*t);o.box.segment<3>(3).setConstant(.5*accel*t*t);o.box.segment<3>(6).setConstant(accel*t);
 return o;
}
struct BoundedRecoveryAttempt {
 std::string status="NOT_RUN";int rebuilds=0;size_t retained=0,removed=0;
 double objective_before=INFINITY,objective_after=INFINITY;
 int healthy_rank=0,healthy_columns=0;
 double full_state_domain_ratio=INFINITY;
 Eigen::Matrix<double,5,1> cumulative=Eigen::Matrix<double,5,1>::Zero(),local=Eigen::Matrix<double,5,1>::Zero();
 FullStateBridge bridge;gtsam::Values reference;
};
// New reference built solely from retained graph factors and one original
// initialization observation. Recenter twice, each optimizer has a fixed
// iteration budget; no post-detector retry loop and no real backend writes.
// Existing bridge weights are optimization metrics, NEVER Gaussian evidence.
inline BoundedRecoveryAttempt boundedRecovery(const EpochTransaction& tx,
 const ExclusionAction& action,const NavigationState& initial,double accel,double rate,
 const std::vector<ImuMeasurement>* raw_prefix=nullptr,const GenericBridgeSpec* bridge_spec=nullptr){
 BoundedRecoveryAttempt o;if(action.bridge_mode==BridgeMode::None)return o;
 std::set<std::uint64_t> removed;for(auto id:action.groups_to_remove)removed.insert(id.value());
 std::map<std::size_t,const PendingFactorGroup*> imu;
 std::map<std::size_t,const HistoricalEpochContext*> history;
 for(const auto& h:tx.recoverable_history){history[h.proposed_epoch]=&h;for(const auto& g:h.groups)
  if(g.kind==FactorKind::CombinedImu)imu[h.proposed_epoch]=&g;}
 imu[tx.proposed_epoch]=&tx.imu_group;
 // Full prefix exists at onset20. Refuse marginal priors: their unrepresented
 // fault/sample dependence cannot be erased by dropping a fault column.
 if(!tx.frozen_values->exists(gtsam::Symbol('x',0))){o.status="MARGINAL_RAW_PREFIX_MISSING";return o;}
 std::size_t first=tx.proposed_epoch;for(const auto& kv:imu)if(removed.count(kv.second->id.value()))first=std::min(first,kv.first);
 gtsam::NonlinearFactorGraph graph;
 for(const auto& f:tx.frozen_slots)if(f.factor){if(f.group_id && removed.count(f.group_id->value())){++o.removed;continue;}graph.push_back(f.factor);++o.retained;}
 if(!removed.count(tx.imu_group.id.value()))graph.push_back(tx.imu_group.factors);
 if(raw_prefix){
  // Rebuild every retained PIM from observed raw input and the ORIGINAL bias
  // observation. Neither polluted online biasHat nor a removed sample enters
  // the new reference. Future samples beyond this transaction are never read.
  gtsam::NonlinearFactorGraph rebuilt;
  for(const auto& f:graph){auto imu_factor=dynamic_cast<const gtsam::CombinedImuFactor*>(f.get());
   if(!imu_factor){rebuilt.push_back(f);continue;}
   size_t prev=gtsam::Symbol(imu_factor->key1()).index(),next=gtsam::Symbol(imu_factor->key3()).index();
   double begin=.05*prev,end=.05*next;
   if(end>tx.end.seconds()+1e-12){o.status="NONCAUSAL_RAW_PREFIX";return o;}
   std::vector<const ImuMeasurement*> slice;
   for(const auto& sample:*raw_prefix)if(sample.timestamp.seconds()>=begin-1e-12 && sample.timestamp.seconds()<=end+1e-12)slice.push_back(&sample);
   if(slice.size()<2 || std::abs(slice.front()->timestamp.seconds()-begin)>1e-12 || std::abs(slice.back()->timestamp.seconds()-end)>1e-12){o.status="RAW_PREFIX_ENDPOINT_MISSING";return o;}
   auto params=boost::dynamic_pointer_cast<gtsam::PreintegrationCombinedParams>(imu_factor->preintegratedMeasurements().params());
   gtsam::PreintegratedCombinedMeasurements pim(params,gtsam::imuBias::ConstantBias(initial.accel_bias_mps2,initial.gyro_bias_radps));
   for(size_t k=1;k<slice.size();++k)pim.integrateMeasurement(.5*(slice[k-1]->specific_force_mps2+slice[k]->specific_force_mps2),.5*(slice[k-1]->angular_velocity_radps+slice[k]->angular_velocity_radps),slice[k]->timestamp.seconds()-slice[k-1]->timestamp.seconds());
   rebuilt.emplace_shared<gtsam::CombinedImuFactor>(imu_factor->key1(),imu_factor->key2(),imu_factor->key3(),imu_factor->key4(),imu_factor->key5(),imu_factor->key6(),pim);
  }
  graph=std::move(rebuilt);
 }else for(const auto& h:history)if(h.first>=first && h.second->preintegration && !removed.count(imu.count(h.first)?imu.at(h.first)->id.value():0)){
  o.status="POST_ONSET_HEALTHY_PIM_REBUILD_REQUIRED";return o;
 }
 for(const auto& g:tx.uwb_groups)if(g.nominal && !removed.count(g.id.value()))graph.push_back(g.factors);
 gtsam::NonlinearFactorGraph healthy_graph;
 for(const auto& f:graph){auto bridge=dynamic_cast<const KinematicPoseVelocityBridgeFactor*>(f.get());
  auto bias_metric=dynamic_cast<const gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>*>(f.get());
  if(bridge){first=std::min(first,(size_t)gtsam::Symbol(bridge->key3()).index());continue;}
  if(bias_metric)continue;
  healthy_graph.push_back(f);
 }
 // Exact original factors before onset, with freshly reconstructed numerical
 // initial Values. Starting from polluted tx.frozen_values is prohibited.
 gtsam::Values values;gtsam::Pose3 pose(gtsam::Rot3(initial.q_world_body.toRotationMatrix()),initial.position_world_m);
 gtsam::Vector3 velocity=initial.velocity_world_mps;gtsam::imuBias::ConstantBias bias(initial.accel_bias_mps2,initial.gyro_bias_radps);
 auto add=[&](size_t e){values.insert(gtsam::Symbol('x',e),pose);values.insert(gtsam::Symbol('v',e),velocity);values.insert(gtsam::Symbol('b',e),bias);};add(0);
 for(size_t e=1;e<=tx.proposed_epoch;++e){double dt=.05;
  bool excluded=imu.count(e) && removed.count(imu.at(e)->id.value());
  if(excluded){pose=gtsam::Pose3(pose.rotation(),pose.translation()+dt*velocity);
   EpochTransaction bt;bt.previous_epoch=e-1;bt.proposed_epoch=e;bt.begin=TimestampNs((e-1)*50000000LL);bt.end=TimestampNs(e*50000000LL);
   GenericBridgeSpec spec=bridge_spec?*bridge_spec:GenericBridgeSpec{};graph.push_back(BridgeFactory().makeGeneric(bt,spec).factors);graph.push_back(BridgeFactory().makeBiasContinuity(bt,spec).factors);
  }else{
   const gtsam::CombinedImuFactor* factor=nullptr;
   for(const auto& f:graph)if(auto c=dynamic_cast<const gtsam::CombinedImuFactor*>(f.get()))if(c->key3()==gtsam::Symbol('x',e)){factor=c;break;}
   if(!factor){bool bridge_present=false;for(const auto& f:graph)if(auto bridge=dynamic_cast<const KinematicPoseVelocityBridgeFactor*>(f.get()))bridge_present=bridge_present || bridge->key3()==gtsam::Symbol('x',e);
    if(!bridge_present){o.status="HEALTHY_PREFIX_MATERIAL_MISSING";return o;}
    pose=gtsam::Pose3(pose.rotation(),pose.translation()+dt*velocity);
   }else{auto next=factor->preintegratedMeasurements().predict(gtsam::NavState(pose,velocity),bias);pose=next.pose();velocity=next.velocity();}
  }add(e);
 }
 auto initial_values=values;
 o.objective_before=graph.error(values);
 try{
  for(int r=0;r<2;++r){auto before=values;gtsam::GaussNewtonParams params;params.maxIterations=8;params.relativeErrorTol=1e-7;params.absoluteErrorTol=1e-7;
   values=gtsam::GaussNewtonOptimizer(graph,values,params).optimize();++o.rebuilds;
   o.local.setZero();for(size_t e=0;e<=tx.proposed_epoch;++e){
    auto dx=before.at<gtsam::Pose3>(gtsam::Symbol('x',e)).localCoordinates(values.at<gtsam::Pose3>(gtsam::Symbol('x',e)));
    o.local(0)=std::max(o.local(0),dx.head<3>().norm());o.local(1)=std::max(o.local(1),dx.tail<3>().norm());
    o.local(2)=std::max(o.local(2),(before.at<gtsam::Vector3>(gtsam::Symbol('v',e))-values.at<gtsam::Vector3>(gtsam::Symbol('v',e))).norm());
    auto db=(values.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',e)).vector()-before.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',e)).vector()).eval();
    o.local(3)=std::max(o.local(3),db.head<3>().norm());o.local(4)=std::max(o.local(4),db.tail<3>().norm());
   }
  }
  for(size_t e=0;e<=tx.proposed_epoch;++e){
   auto dx=initial_values.at<gtsam::Pose3>(gtsam::Symbol('x',e)).localCoordinates(values.at<gtsam::Pose3>(gtsam::Symbol('x',e)));
   o.cumulative(0)=std::max(o.cumulative(0),dx.head<3>().norm());o.cumulative(1)=std::max(o.cumulative(1),dx.tail<3>().norm());
   o.cumulative(2)=std::max(o.cumulative(2),(initial_values.at<gtsam::Vector3>(gtsam::Symbol('v',e))-values.at<gtsam::Vector3>(gtsam::Symbol('v',e))).norm());
   auto db=(values.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',e)).vector()-initial_values.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',e)).vector()).eval();
   o.cumulative(3)=std::max(o.cumulative(3),db.head<3>().norm());o.cumulative(4)=std::max(o.cumulative(4),db.tail<3>().norm());
  }
  o.objective_after=graph.error(values);o.reference=values;
  auto healthy_linear=healthy_graph.linearize(values);
  auto h=healthy_linear->jacobian().first;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> rank(h);rank.setThreshold(1e-10);
  o.healthy_rank=rank.rank();o.healthy_columns=15*(tx.proposed_epoch+1);
  // Healthy prefix boundary covariance, independently solved without any
  // factor at/after onset. Joint p/v/R/bias cross blocks remain intact.
  gtsam::NonlinearFactorGraph prefix;gtsam::Values prefix_values;
  for(const auto& f:graph){bool earlier=true;for(auto key:f->keys())earlier=earlier && gtsam::Symbol(key).index()<first;if(earlier)prefix.push_back(f);}
  for(size_t e=0;e<first;++e)for(char c:{'x','v','b'})prefix_values.insert(gtsam::Symbol(c,e),values.at(gtsam::Symbol(c,e)));
  auto boundary=gtsam::GaussNewtonOptimizer(prefix,prefix_values).optimize();gtsam::Marginals marg(prefix,boundary);
  gtsam::KeyVector keys={gtsam::Symbol('x',first-1),gtsam::Symbol('v',first-1),gtsam::Symbol('b',first-1)};
  auto joint=marg.jointMarginalCovariance(keys);Eigen::Matrix<double,15,15> p;const int widths[3]={6,3,6};int row=0;
  for(int i=0;i<3;++i){int col=0;for(int j=0;j<3;++j){p.block(row,col,widths[i],widths[j])=joint.at(keys[i],keys[j]);col+=widths[j];}row+=widths[i];}
  // Convert Pose right-local translation uncertainty to world, preserving
  // every cross term before constant-velocity propagation.
  Eigen::Matrix<double,15,15> world=Eigen::Matrix<double,15,15>::Identity();world.block<3,3>(3,3)=boundary.at<gtsam::Pose3>(keys[0]).rotation().matrix();
  o.bridge=fullStateBridge(world*p*world.transpose(),.05*(tx.proposed_epoch-first+1),accel,rate);
  o.full_state_domain_ratio=std::max(o.bridge.box.head<3>().maxCoeff()/.25,o.bridge.box.segment<3>(6).maxCoeff()/.5);
  o.status=o.full_state_domain_ratio>1.?"BRIDGE_DOMAIN_EXCEEDS_LOCAL_MODEL":"PIM_HISTORY_UNIFORM_BOUND_MISSING";
 }catch(const std::exception& ex){o.status="RECOVERY_SOLVE_FAILED:"+std::string(ex.what());}
 return o;
}
}
