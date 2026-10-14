#include "rolling_recovery.hpp"
#include "epoch_raim.hpp"
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <Eigen/SVD>
#include <set>
#include <tuple>
namespace uwb_imu_pl::research {
namespace {
void reintegrate(PendingFactorGroup& group,const std::vector<ImuMeasurement>& raw,
 const std::shared_ptr<const gtsam::PreintegratedCombinedMeasurements>& original,
 const Eigen::Matrix<double,6,1>& bias,size_t previous,size_t next){
 auto params=boost::dynamic_pointer_cast<gtsam::PreintegrationCombinedParams>(original->params());
 gtsam::PreintegratedCombinedMeasurements pim(params,gtsam::imuBias::ConstantBias(bias.head<3>(),bias.tail<3>()));
 for(size_t i=1;i<raw.size();++i)pim.integrateMeasurement(.5*(raw[i-1].specific_force_mps2+raw[i].specific_force_mps2),.5*(raw[i-1].angular_velocity_radps+raw[i].angular_velocity_radps),double(raw[i].timestamp.value()-raw[i-1].timestamp.value())/1e9);
 group.factors.resize(0);group.factors.emplace_shared<gtsam::CombinedImuFactor>(gtsam::Symbol('x',previous),gtsam::Symbol('v',previous),gtsam::Symbol('x',next),gtsam::Symbol('v',next),gtsam::Symbol('b',previous),gtsam::Symbol('b',next),pim);group.raw_covariance=pim.preintMeasCov();
}
void omitRange(EpochTransaction& tx,EpochCommitPlan& plan,int mask){
 if(mask<0)return;
 auto anchor=tx.uwb_batch.measurements.at(mask).anchor_id;
 auto contains=[](const PendingFactorGroup& g,MeasurementId id){return std::find(g.source_measurements.begin(),g.source_measurements.end(),id)!=g.source_measurements.end();};
 for(const auto& g:tx.uwb_groups)if(g.nominal){plan.groups_to_add.erase(std::remove(plan.groups_to_add.begin(),plan.groups_to_add.end(),g.id),plan.groups_to_add.end());break;}
 bool found=false;
 for(const auto& g:tx.uwb_groups)if(!g.nominal && g.source_measurements.size()==7 && !contains(g,tx.uwb_batch.measurements.at(mask).id)){plan.groups_to_add.push_back(g.id);found=true;break;}
 if(!found)throw std::runtime_error("CURRENT_UWB_REPLACEMENT_MISSING");
 for(const auto& h:tx.recoverable_history){MeasurementId source;
  for(const auto& m:h.uwb_batch.measurements)if(m.anchor_id==anchor)source=m.id;
  for(const auto& g:h.groups)if(g.kind==FactorKind::UwbBatch && std::find(h.selected_groups.begin(),h.selected_groups.end(),g.id)!=h.selected_groups.end() && contains(g,source)){
   bool matched=false;for(const auto& replacement:h.groups)if(replacement.replaces_group && *replacement.replaces_group==g.id && replacement.source_measurements.size()+1==g.source_measurements.size() && !contains(replacement,source)){plan.groups_to_remove.push_back(g.id);plan.groups_to_add.push_back(replacement.id);matched=true;break;}
   if(!matched)throw std::runtime_error("HISTORICAL_UWB_REPLACEMENT_MISSING");
  }
 }
}
PendingFactorGroup metric(const RollingRawState& raw,const IntegrityConfig& config,
 const EpochTransaction& tx,FactorGroupId replaces,bool stable_rotation){
 // Low namespace bit survives the ledger's <<8 factor-ID encoding.
 PendingFactorGroup g;g.id=FactorGroupId(0xd001000000000000ULL+tx.id.value());
 g.kind=FactorKind::BoundaryPrior;g.sensor=SensorType::Prior;g.nominal=false;
 g.model_id=stable_rotation?"sc6-boundary-numerical-rotation-metric-v1":"sc5-discarded-boundary-recovery-metric-v1";g.noise_model_id="optimization-metric-NOT-independent-source";
 g.source_ids={"raw-capsule-epoch:"+std::to_string(raw.epoch),"discarded-prior-group:"+std::to_string(replaces.value())};
 g.replaces_group=replaces;g.recovery_epoch=tx.proposed_epoch;
 g.keys={gtsam::Symbol('x',raw.epoch),gtsam::Symbol('v',raw.epoch),gtsam::Symbol('b',raw.epoch)};
 Eigen::Matrix3d middle;Eigen::Vector3d p,v;Eigen::Matrix<double,6,1> b;
 for(int j=0;j<3;++j){p(j)=raw.p[j].lower()/2+raw.p[j].upper()/2;v(j)=raw.v[j].lower()/2+raw.v[j].upper()/2;for(int k=0;k<3;++k)middle(j,k)=raw.rotation[j][k].lower()/2+raw.rotation[j][k].upper()/2;}
 for(int j=0;j<6;++j)b(j)=raw.bias[j].lower()/2+raw.bias[j].upper()/2;
 Eigen::JacobiSVD<Eigen::Matrix3d> svd(middle,Eigen::ComputeFullU|Eigen::ComputeFullV);
 Eigen::Matrix3d correction=Eigen::Matrix3d::Identity();correction(2,2)=(svd.matrixU()*svd.matrixV().transpose()).determinant()<0?-1:1;
 Eigen::Matrix3d rotation=svd.matrixU()*correction*svd.matrixV().transpose();
 // Capsule rotation boxes can be symmetric/full: their midpoint has no
 // identifiable orientation, and its polar factor is numerically arbitrary.
 // Retain the already-existing boundary orientation as an OPTIMIZATION metric;
 // this is not a new measurement or a raw integrity constraint.
 if(stable_rotation)rotation=tx.frozen_values->at<gtsam::Pose3>(g.keys[0]).rotation().matrix();
 auto sigmas=config.realtime.prior_sigmas;
 g.factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(g.keys[0],gtsam::Pose3(gtsam::Rot3(rotation),p),gtsam::noiseModel::Diagonal::Sigmas(sigmas.head<6>()));
 g.factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(g.keys[1],v,gtsam::noiseModel::Diagonal::Sigmas(sigmas.segment<3>(6)));
 g.factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(g.keys[2],gtsam::imuBias::ConstantBias(b.head<3>(),b.tail<3>()),gtsam::noiseModel::Diagonal::Sigmas(sigmas.tail<6>()));
 return g;
}
void seed(const gtsam::Values& values,size_t e,NavigationState& s){auto p=values.at<gtsam::Pose3>(gtsam::Symbol('x',e));auto b=values.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',e));s.position_world_m=p.translation();s.q_world_body=Eigen::Quaterniond(p.rotation().matrix());s.velocity_world_mps=values.at<gtsam::Vector3>(gtsam::Symbol('v',e));s.accel_bias_mps2=b.accelerometer();s.gyro_bias_radps=b.gyroscope();}
}
RollingRecovery::RollingRecovery(const NavigationState& s,const ResolvedRollingPolicyV1& p,bool stable_rotation,bool optimized):policy_(p),stable_rotation_(stable_rotation),optimized_(optimized){bias_observation_.head<3>()=s.accel_bias_mps2;bias_observation_.tail<3>()=s.gyro_bias_radps;}
RollingFdeAssessment RollingRecovery::prepare(EpochTransaction& tx,const Sc2ImuMonitor& monitor,
 const std::deque<ImuMeasurement>& replay,const std::deque<RollingRawState>& capsules,
 const IntegrityConfig& config,bool fde){
 return prepare(tx,finiteMonitorView(monitor,tx.end),replay,capsules,config,fde);
}
RollingFdeAssessment RollingRecovery::prepare(EpochTransaction& tx,const RollingMonitorView& monitor,
 const std::deque<ImuMeasurement>& replay,const std::deque<RollingRawState>& capsules,
 const IntegrityConfig& config,bool fde){
 using Clock=std::chrono::steady_clock;const auto start=Clock::now();
 RollingFdeAssessment result;result.plan=EpochCommitPlan::nominalPlan(tx);
 if(monitor.open!=policy_.open)throw std::invalid_argument("RECOVERY_CERTIFICATE_MODE_MIX");
 auto assessment=monitor.state;
 if(policy_.open && fde && monitor.alert_generation && monitor.alert_generation!=alert_generation_){
  // Re-arm only numerical recovery for this new observed support. The
  // source event, automaton histories, raw p/v/R/bias and prior repair records
  // stay on their original global clock; this is not a healthy new task.
  alert_generation_=monitor.alert_generation;isolated_=false;
  quarantine_begin_=TimestampNs(0);quarantine_end_=TimestampNs(0);
  observed_anchor_=-1;observed_uwb_onset_=0;held_anchor_=-1;held_until_=0;
  open_pending_=true;
 }
 if(policy_.open && open_pending_)assessment.alarm=true;
 if(!monitor.matches(tx) || (!assessment.nominal_possible && !assessment.possible))throw std::runtime_error("RECOVERY_MONITOR_SCOPE_REFUSED");
 std::array<std::optional<EpochRaim>,9> cached;
 auto candidate=[&](int mask){if(!optimized_)return uwbCandidate(tx.uwb_batch,mask,1e-6,1e-9);auto& item=cached[mask+1];if(!item)item=uwbCandidate(tx.uwb_batch,mask,1e-6,1e-9);return *item;};
 auto all=candidate(-1);bool uwb_alarm=all.gaussian.statistic>all.gaussian.threshold;result.uwb_alarm=uwb_alarm;if(uwb_alarm)result.uwb_alarm_time=Clock::now();
 int detected=-1;
 if(uwb_alarm){double best=INFINITY;for(int mask=0;mask<8;++mask){auto c=candidate(mask);if(c.gaussian.statistic<=c.gaussian.threshold && c.gaussian.statistic<best){best=c.gaussian.statistic;detected=mask;}}}
 if(detected>=0){observed_anchor_=detected;if(!observed_uwb_onset_)observed_uwb_onset_=tx.proposed_epoch;}
 if(!fde){result.status=assessment.alarm || uwb_alarm?"DETECTION_HELD":"KEEP";result.recovery="FDE_DISABLED";return result;}
 if(policy_.open && history_unavailable_){result.status="RECOVERY_OUT_OF_BUFFER";result.recovery="BOUND_VALID_BUT_RECOVERY_UNAVAILABLE";return result;}
 bool held=held_anchor_>=0 && int(tx.proposed_epoch)<=held_until_;
 // A retired alarm does not make the previous shared PIM endpoint healthy.
 // Keep the inferred support until no physical sample of this PIM overlaps.
 auto touches_quarantine=[&]{return std::any_of(tx.raw_imu_slice.begin(),tx.raw_imu_slice.end(),[&](const auto& m){return !(m.timestamp<quarantine_begin_) && m.timestamp<quarantine_end_;});};
 bool bridge=(assessment.alarm && !isolated_) || (isolated_ && touches_quarantine());
 if(assessment.alarm){
  if(quarantine_begin_.value()==0 || assessment.union_begin<quarantine_begin_)quarantine_begin_=assessment.union_begin;
  if(quarantine_end_<assessment.union_end)quarantine_end_=assessment.union_end;
  bridge=!isolated_ || std::any_of(tx.raw_imu_slice.begin(),tx.raw_imu_slice.end(),[&](const auto& m){return !(m.timestamp<quarantine_begin_) && m.timestamp<quarantine_end_;});
  result.first_dependency=size_t((quarantine_begin_.value()-5000000)/50000000+1);
 }
 if(bridge && !result.first_dependency)result.first_dependency=size_t((quarantine_begin_.value()-5000000)/50000000+1);
 const bool historical_range=observed_anchor_>=0 && !isolated_ && observed_uwb_onset_<tx.proposed_epoch;
 if(bridge && !isolated_){
  auto first=result.first_dependency;if(historical_range)first=std::min(first,observed_uwb_onset_);
  if(replay.empty() || replay.front().timestamp.value()>int64_t(first-1)*50000000LL || replay.back().timestamp!=tx.end){
   if(policy_.open)history_unavailable_=true;
   result.status="RECOVERY_OUT_OF_BUFFER";result.recovery="BOUND_VALID_BUT_RECOVERY_UNAVAILABLE";return result;
  }
 }
 const bool search=uwb_alarm || assessment.alarm || held || historical_range || (policy_.open && isolated_);
 if(!search)return result;
 result.actions=18;
 // Evaluate the complete native mask x IMU-action product. Physical proof
 // keeps all task branches independently of this data-driven action selection.
 size_t best_card=100;double best_stat=INFINITY;int selected=-2;
 for(int mask=-1;mask<8;++mask)for(int imu=0;imu<2;++imu){
  auto post=candidate(mask);
  bool eligible=post.gaussian.statistic<=post.gaussian.threshold && bool(imu)==bridge;
  if(held && mask!=held_anchor_)eligible=false;
  if(historical_range && mask!=observed_anchor_)eligible=false;
  if(!eligible)continue;
  size_t card=(mask>=0)+imu;
  if(std::make_pair(card,post.gaussian.statistic)<std::make_pair(best_card,best_stat)){best_card=card;best_stat=post.gaussian.statistic;selected=mask;}
 }
 if(selected==-2)throw std::runtime_error("NO_COMPLETE_POST_PASSED_ROLLING_ACTION");
 result.excluded=selected;result.current_bridge=bridge;
 if(selected>=0){omitRange(tx,result.plan,selected);held_anchor_=selected;if(uwb_alarm)held_until_=int(tx.proposed_epoch)+2;}
 if(isolated_ && !bridge){reintegrate(tx.imu_group,tx.raw_imu_slice,tx.preintegration,bias_observation_,tx.previous_epoch,tx.proposed_epoch);result.rejoined=true;result.status="IMU_REJOINED_ORIGINAL_BIAS_REINTEGRATION";result.recovery="FDE_SUCCESS_REJOINED";return result;}
 if(bridge){
  result.plan.groups_to_add.erase(std::remove(result.plan.groups_to_add.begin(),result.plan.groups_to_add.end(),tx.imu_group.id),result.plan.groups_to_add.end());
  result.plan.groups_to_add.push_back(tx.generic_bridge_group.id);result.plan.groups_to_add.push_back(tx.generic_bias_continuity_group.id);result.plan.bridge_mode=BridgeMode::GenericKinematic;
  for(const auto& h:tx.recoverable_history)if(h.proposed_epoch>=result.first_dependency){
   for(const auto& g:h.groups)if(g.kind==FactorKind::CombinedImu && std::find(h.selected_groups.begin(),h.selected_groups.end(),g.id)!=h.selected_groups.end()){
    result.plan.groups_to_remove.push_back(g.id);int pair=0;for(const auto& r:h.groups)if(r.replaces_group && *r.replaces_group==g.id && (r.kind==FactorKind::KinematicBridge || r.kind==FactorKind::BiasContinuity)){result.plan.groups_to_add.push_back(r.id);++pair;}
    if(pair!=2)throw std::runtime_error("HISTORY_BRIDGE_PAIR_MISSING");
   }
  }
  if(!isolated_ && !tx.frozen_values->exists(gtsam::Symbol('x',0))){
   size_t boundary=tx.proposed_epoch+1-policy_.fgo;
   const RollingRawState* capsule=nullptr;for(const auto& c:capsules)if(c.epoch==boundary)capsule=&c;
   if(!capsule)throw std::runtime_error("BOUNDED_RAW_RECOVERY_CAPSULE_MISSING");
   FactorGroupId replaces;std::set<uint64_t> ids;
   for(const auto& slot:tx.frozen_slots)if(dynamic_cast<const gtsam::LinearContainerFactor*>(slot.factor.get())){
    if(!slot.group_id)throw std::runtime_error("BOUNDARY_LEDGER_SOURCE_MISSING");
    if(ids.insert(slot.group_id->value()).second)result.plan.groups_to_remove.push_back(*slot.group_id);
    replaces=*slot.group_id;result.removed_boundary.push_back(slot.factor);result.removed_prior_slots.push_back(slot.slot);
   }
   if(result.removed_boundary.empty())throw std::runtime_error("ACTUAL_NUMERICAL_BOUNDARY_MISSING");
   auto regularizer=metric(*capsule,config,tx,replaces,stable_rotation_);result.metric_group=regularizer.id;result.plan.groups_to_add.push_back(regularizer.id);
   for(auto prior:ids)result.plan.replacement_relations[prior]=regularizer.id.value();
   HistoricalEpochContext provenance;provenance.previous_epoch=boundary;provenance.proposed_epoch=boundary;provenance.begin=provenance.end=TimestampNs(boundary*50000000LL);provenance.groups.push_back(std::move(regularizer));tx.recoverable_history.push_back(std::move(provenance));
   result.boundary_repair=true;result.history_marginalized=result.first_dependency<=boundary;
  }
 }
 std::set<uint64_t> remove,add;for(auto id:result.plan.groups_to_remove)remove.insert(id.value());for(auto id:result.plan.groups_to_add)add.insert(id.value());
 gtsam::NonlinearFactorGraph graph;for(const auto& f:tx.frozen_slots)if(!f.group_id || !remove.count(f.group_id->value()))graph.push_back(f.factor);
 for(const auto* g:std::vector<const PendingFactorGroup*>{&tx.imu_group,&tx.generic_bridge_group,&tx.generic_bias_continuity_group})if(add.count(g->id.value()))graph.push_back(g->factors);
 for(const auto& g:tx.uwb_groups)if(add.count(g.id.value()))graph.push_back(g.factors);
 for(const auto& h:tx.recoverable_history)for(const auto& g:h.groups)if(add.count(g.id.value()))graph.push_back(g.factors);
 // All candidate optimizers are temporary; the one existing native backend
 // has not been touched. Reference spans only live keys, never epoch0..k.
 gtsam::Values values=*tx.frozen_values;
 for(int round=0;round<2;++round){gtsam::GaussNewtonParams params;params.maxIterations=8;params.relativeErrorTol=1e-7;params.absoluteErrorTol=1e-7;values=gtsam::GaussNewtonOptimizer(graph,values,params).optimize();++result.rebuilds;}
 if(!std::isfinite(graph.error(values)))throw std::runtime_error("ROLLING_CANDIDATE_NONFINITE");
 seed(values,tx.proposed_epoch,tx.cv_predicted_state);result.numeric_evaluated=1;
 result.plan.action_id=ExclusionActionId(7000+(selected+1)*2+int(bridge));result.plan.fde_status=bridge?FdeStatus::SuccessImuExclusionGenericBridge:selected>=0?FdeStatus::SuccessUwbExclusion:FdeStatus::SuccessKeepAll;
 result.plan.recovery_epoch_begin=result.first_dependency;result.plan.recovery_epoch_end=tx.proposed_epoch;
 result.status=bridge?"FULL_RAW_SUPPORT_BRIDGE":"UWB_EXCLUSION";result.recovery=result.boundary_repair?"FDE_BOUNDARY_REPLACED_METRIC":"FDE_SUCCESS_ACTIVE_GRAPH";
 result.candidate_ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();return result;
}
void RollingRecovery::committed(const RollingFdeAssessment& a,const EpochTransaction& tx,const CommitReceipt& r,const FactorLedger& ledger){
 if(r.transaction_id!=tx.id || r.graph_version!=tx.base_graph_version+1 || r.backend_updates!=1)throw std::runtime_error("ROLLING_RECOVERY_RECEIPT_REFUSED");
 if(a.current_bridge){if(r.group_to_slots.count(tx.imu_group.id.value()) || !r.group_to_slots.count(tx.generic_bridge_group.id.value()) || !r.group_to_slots.count(tx.generic_bias_continuity_group.id.value()))throw std::runtime_error("ACTUAL_BRIDGE_RECEIPT_INCOMPLETE");}
 if(a.rejoined && (!r.group_to_slots.count(tx.imu_group.id.value()) || r.group_to_slots.count(tx.generic_bridge_group.id.value())))throw std::runtime_error("ACTUAL_IMU_REJOIN_RECEIPT_INCOMPLETE");
 if(a.current_bridge && (!ledger.activeSlots(tx.imu_group.id).empty() || ledger.activeSlots(tx.generic_bridge_group.id)!=r.group_to_slots.at(tx.generic_bridge_group.id.value()) || ledger.activeSlots(tx.generic_bias_continuity_group.id)!=r.group_to_slots.at(tx.generic_bias_continuity_group.id.value())))throw std::runtime_error("ACTUAL_BRIDGE_LEDGER_MISMATCH");
 if(a.rejoined && ledger.activeSlots(tx.imu_group.id)!=r.group_to_slots.at(tx.imu_group.id.value()))throw std::runtime_error("ACTUAL_REJOIN_LEDGER_MISMATCH");
 for(auto id:a.plan.groups_to_remove)if(std::find(r.historical_groups_removed.begin(),r.historical_groups_removed.end(),id)==r.historical_groups_removed.end())throw std::runtime_error("HISTORY_REMOVAL_RECEIPT_INCOMPLETE");
 if(a.boundary_repair){
  if(!r.group_to_slots.count(a.metric_group.value()) || r.group_to_slots.at(a.metric_group.value()).size()!=3)throw std::runtime_error("REPLACEMENT_METRIC_RECEIPT_MISSING");
  if(ledger.activeSlots(a.metric_group)!=r.group_to_slots.at(a.metric_group.value()))throw std::runtime_error("REPLACEMENT_METRIC_LEDGER_MISMATCH");
  for(size_t slot:a.removed_prior_slots)if(std::find(r.removed_factor_slots.begin(),r.removed_factor_slots.end(),slot)==r.removed_factor_slots.end())throw std::runtime_error("POLLUTED_PRIOR_SLOT_NOT_REMOVED");
  for(const auto& e:ledger.entries())if(e.lifecycle==FactorLifecycle::Active)for(const auto& f:a.removed_boundary)if(e.factor.get()==f.get())throw std::runtime_error("POLLUTED_PRIOR_STILL_ACTIVE");
 }
 if(a.current_bridge){isolated_=true;open_pending_=false;}
}
}
