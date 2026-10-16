#include "rolling_recovery.hpp"
#include "epoch_raim.hpp"
#include "numerical_candidate.hpp"
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
void omitRange(EpochTransaction& tx,EpochCommitPlan& plan,AnchorId anchor){
 auto retained=[&](const UwbBatch& batch,const std::vector<MeasurementId>& members){
  std::set<MeasurementId> bad;for(const auto& m:batch.measurements)if(m.anchor_id==anchor)bad.insert(m.id);
  std::vector<MeasurementId> expected;for(auto id:members)if(!bad.count(id))expected.push_back(id);
  return expected;
 };
 auto replace=[&](const UwbBatch& batch,const auto& groups,const std::vector<FactorGroupId>& selected,bool current){
  for(const auto& g:groups)if(g.kind==FactorKind::UwbBatch && std::find(selected.begin(),selected.end(),g.id)!=selected.end()){
   const auto expected=retained(batch,g.source_measurements);
   if(expected==g.source_measurements)continue;
   const PendingFactorGroup* replacement=nullptr;
   for(const auto& candidate:groups)if(candidate.replaces_group && *candidate.replaces_group==g.id && candidate.source_measurements==expected){replacement=&candidate;break;}
   if(!replacement)throw std::runtime_error(current?"CURRENT_UWB_REPLACEMENT_MISSING":"HISTORICAL_UWB_REPLACEMENT_MISSING");
   if(current)plan.groups_to_add.erase(std::remove(plan.groups_to_add.begin(),plan.groups_to_add.end(),g.id),plan.groups_to_add.end());
   else plan.groups_to_remove.push_back(g.id);
   plan.groups_to_add.push_back(replacement->id);
  }
 };
 const auto current=plan.groups_to_add;
 replace(tx.uwb_batch,tx.uwb_groups,current,true);
 for(const auto& h:tx.recoverable_history)replace(h.uwb_batch,h.groups,h.selected_groups,false);
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
 if(actual_time_!=bool(monitor.original_time))throw std::invalid_argument("RECOVERY_TIME_MODEL_MIX");
 if(actual_time_){
  EpochSources sources;sources.epoch=tx.proposed_epoch;sources.begin=tx.begin;sources.end=tx.end;
  for(const auto& sample:tx.raw_imu_slice){const auto time=monitor.original_time(sample.id);
   if(sample.timestamp<time)throw std::invalid_argument("RECOVERY_PARENT_FUTURE");
   if(sources.parents.empty() || sources.parents.back().first!=sample.id)sources.parents.emplace_back(sample.id,time);
  }
  if(sources.parents.empty() || (!timeline_.empty() && (timeline_.back().epoch+1!=sources.epoch || timeline_.back().end!=sources.begin)))
   throw std::invalid_argument("RECOVERY_EPOCH_SOURCE_GAP");
  pending_sources_=std::move(sources);
 }
 auto dependency=[&]()->size_t{
  if(!actual_time_)return size_t((quarantine_begin_.value()-5000000)/50000000+1);
  if(!timeline_.empty() && timeline_.front().epoch>1 && quarantine_begin_<timeline_.front().parents.front().second)return 0;
  auto touches=[&](const EpochSources& source){return std::any_of(source.parents.begin(),source.parents.end(),[&](const auto& p){return !(p.second<quarantine_begin_) && p.second<quarantine_end_;});};
  for(const auto& source:timeline_)if(touches(source))return source.epoch;
  return tx.proposed_epoch;
 };
 if(policy_.open && fde && monitor.alert_generation && monitor.alert_generation!=alert_generation_){
  // Re-arm only numerical recovery for this new observed support. The
  // source event, automaton histories, raw p/v/R/bias and prior repair records
  // stay on their original global clock; this is not a healthy new task.
  alert_generation_=monitor.alert_generation;isolated_=false;
  quarantine_begin_=TimestampNs(0);quarantine_end_=TimestampNs(0);
  if(!actual_time_){observed_anchor_.reset();observed_uwb_onset_=0;held_anchor_.reset();held_until_=0;}
  open_pending_=true;
 }
 if(policy_.open && open_pending_)assessment.alarm=true;
 if(!monitor.matches(tx) || (!assessment.nominal_possible && !assessment.possible))throw std::runtime_error("RECOVERY_MONITOR_SCOPE_REFUSED");
 const auto masks=rawRangeFaultMasks(tx.uwb_batch);
 std::vector<std::optional<EpochRaim>> cached(tx.uwb_batch.measurements.size()+1);
 if(tx.uwb_batch.measurements.size()<4){
  if((assessment.alarm || isolated_) && !actual_time_)throw std::runtime_error("INSUFFICIENT_UWB_ROWS_DURING_RECOVERY");
  result.status="INSUFFICIENT_UWB_ROWS_FOR_FUNCTIONAL_POST";result.recovery="FUNCTIONAL_ACTION_UNAVAILABLE";return result;
 }
 Eigen::MatrixXd anchor_design(tx.uwb_batch.measurements.size()-1,3);for(int i=1;i<int(tx.uwb_batch.measurements.size());++i)anchor_design.row(i-1)=(tx.uwb_batch.measurements[i].anchor_position_m-tx.uwb_batch.measurements[0].anchor_position_m).transpose();auto singular=Eigen::JacobiSVD<Eigen::MatrixXd>(anchor_design).singularValues();
 const bool difficult=numerical_ && (tx.uwb_batch.measurements.size()<=5 || singular.minCoeff()<1e-3*singular.maxCoeff());
 auto evaluate=[&](int mask){
  size_t retained=0;for(const auto& m:tx.uwb_batch.measurements)
   if(mask<0 || m.anchor_id!=tx.uwb_batch.measurements[mask].anchor_id)++retained;
  if(monitor.has_range_time_model)return numericalUwbCandidate(tx.uwb_batch,mask,1e-6,&monitor.range_transport).functional;
  return difficult || (numerical_ && retained<5)?numericalUwbCandidate(tx.uwb_batch,mask).functional:uwbCandidate(tx.uwb_batch,mask,1e-6,1e-9);
 };
 auto candidate=[&](int mask){if(!optimized_)return evaluate(mask);auto& item=cached[mask+1];if(!item)item=evaluate(mask);return *item;};
 // Complete numerical mask census is bounded and entirely before native commit.
 if(difficult)for(int mask:masks)candidate(mask);
 auto all=candidate(-1);bool uwb_alarm=all.gaussian.statistic>all.gaussian.threshold;result.uwb_alarm=uwb_alarm;if(uwb_alarm)result.uwb_alarm_time=Clock::now();
 int detected=-1;
 if(uwb_alarm){double best=INFINITY;for(int mask:masks){if(mask<0)continue;auto c=candidate(mask);if(c.gaussian.statistic<=c.gaussian.threshold && c.gaussian.statistic<best){best=c.gaussian.statistic;detected=mask;}}}
 if(detected>=0){if(actual_time_ && observed_uwb_repaired_)observed_uwb_onset_=0;observed_uwb_repaired_=false;observed_anchor_=tx.uwb_batch.measurements.at(detected).anchor_id;if(!observed_uwb_onset_)observed_uwb_onset_=tx.proposed_epoch;}
 if(!fde){result.status=assessment.alarm || uwb_alarm?"DETECTION_HELD":"KEEP";result.recovery="FDE_DISABLED";return result;}
 if(policy_.open && history_unavailable_){result.status="RECOVERY_OUT_OF_BUFFER";result.recovery="BOUND_VALID_BUT_RECOVERY_UNAVAILABLE";return result;}
 auto rowFor=[&](const std::optional<AnchorId>& anchor){if(anchor)for(int mask:masks)if(mask>=0 && tx.uwb_batch.measurements[mask].anchor_id==*anchor)return mask;return -1;};
 const int held_row=rowFor(held_anchor_),observed_row=rowFor(observed_anchor_);
 result.held_source_present=held_row>=0;
 bool held=held_anchor_.has_value() && (actual_time_?held_remaining_>0:tx.proposed_epoch<=held_until_);
 // A retired alarm does not make the previous shared PIM endpoint healthy.
 // Keep the inferred support until no physical sample of this PIM overlaps.
 auto touches_quarantine=[&]{return std::any_of(tx.raw_imu_slice.begin(),tx.raw_imu_slice.end(),[&](const auto& m){const auto time=actual_time_?monitor.original_time(m.id):m.timestamp;return !(time<quarantine_begin_) && time<quarantine_end_;});};
 bool bridge=(assessment.alarm && !isolated_) || (isolated_ && touches_quarantine());
 if(assessment.alarm){
  if(quarantine_begin_.value()==0 || assessment.union_begin<quarantine_begin_)quarantine_begin_=assessment.union_begin;
  if(quarantine_end_<assessment.union_end)quarantine_end_=assessment.union_end;
  bridge=!isolated_ || std::any_of(tx.raw_imu_slice.begin(),tx.raw_imu_slice.end(),[&](const auto& m){const auto time=actual_time_?monitor.original_time(m.id):m.timestamp;return !(time<quarantine_begin_) && time<quarantine_end_;});
  result.first_dependency=dependency();
 }
 if(bridge && !result.first_dependency)result.first_dependency=dependency();
 const bool historical_range=observed_anchor_.has_value() && (!actual_time_ || !observed_uwb_repaired_) && !isolated_ && observed_uwb_onset_<tx.proposed_epoch;
 if(bridge && !isolated_){
  auto first=result.first_dependency;if(historical_range)first=std::min(first,observed_uwb_onset_);
  bool replay_complete=false;
  if(actual_time_ && first){
   const EpochSources* required=&*pending_sources_;for(const auto& source:timeline_)if(source.epoch==first)required=&source;
   auto has=[&](MeasurementId id){return std::any_of(replay.begin(),replay.end(),[&](const auto& raw){return raw.id==id;});};
   replay_complete=has(required->parents.front().first) && has(pending_sources_->parents.back().first);
  }else if(!actual_time_)replay_complete=!replay.empty() && replay.front().timestamp.value()<=int64_t(first-1)*50000000LL && replay.back().timestamp==tx.end;
  if(!replay_complete){
   if(policy_.open)history_unavailable_=true;
   result.status="RECOVERY_OUT_OF_BUFFER";result.recovery="BOUND_VALID_BUT_RECOVERY_UNAVAILABLE";return result;
  }
 }
 const bool search=uwb_alarm || assessment.alarm || held || historical_range || (policy_.open && isolated_);
 if(!search)return result;
 result.actions=2*masks.size();
 // Evaluate the complete native mask x IMU-action product. Physical proof
 // keeps all task branches independently of this data-driven action selection.
 size_t best_card=100;double best_stat=INFINITY;int selected=-2;
 for(int mask:masks)for(int imu=0;imu<2;++imu){
  auto post=candidate(mask);
  bool eligible=post.gaussian.statistic<=post.gaussian.threshold && bool(imu)==bridge;
  if(held && mask!=held_row)eligible=false;
  if(historical_range && mask!=observed_row)eligible=false;
  if(!eligible)continue;
  size_t card=(mask>=0)+imu;
  if(std::make_pair(card,post.gaussian.statistic)<std::make_pair(best_card,best_stat)){best_card=card;best_stat=post.gaussian.statistic;selected=mask;}
 }
 if(selected==-2){
  if(!actual_time_)throw std::runtime_error("NO_COMPLETE_POST_PASSED_ROLLING_ACTION");
  result.status="NO_FUNCTIONAL_POST_PASSED";result.recovery="BOUND_VALID_BUT_RECOVERY_UNAVAILABLE";return result;
 }
 result.excluded=selected;result.current_bridge=bridge;
 if(selected>=0){result.excluded_source=tx.uwb_batch.measurements.at(selected).anchor_id;omitRange(tx,result.plan,*result.excluded_source);held_anchor_=result.excluded_source;if(uwb_alarm){held_until_=tx.proposed_epoch+2;if(actual_time_)held_remaining_=2;}}
 else if(historical_range){result.excluded_source=observed_anchor_;omitRange(tx,result.plan,*observed_anchor_);}
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
   TimestampNs boundary_time(boundary*50000000LL);
   if(actual_time_){bool found=false;for(const auto& source:timeline_)if(source.epoch==boundary){boundary_time=source.end;found=true;break;}
    if(!found)throw std::runtime_error("ACTUAL_BOUNDARY_TIME_MISSING");}
   HistoricalEpochContext provenance;provenance.previous_epoch=boundary;provenance.proposed_epoch=boundary;provenance.begin=provenance.end=boundary_time;provenance.groups.push_back(std::move(regularizer));tx.recoverable_history.push_back(std::move(provenance));
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
 if(actual_time_){
  if(a.excluded_source)observed_uwb_repaired_=true;
  if(a.held_source_present && !a.uwb_alarm && held_remaining_ && a.recovery!="FDE_DISABLED")--held_remaining_;
  if(a.rejoined)isolated_=false;
  if(!pending_sources_ || pending_sources_->epoch!=r.committed_epoch || pending_sources_->end!=r.state_timestamp)throw std::runtime_error("RECOVERY_SOURCE_RECEIPT_MISMATCH");
  timeline_.push_back(std::move(*pending_sources_));pending_sources_.reset();
  while(timeline_.size()>size_t(policy_.replay+1))timeline_.pop_front();
 }
}
}
