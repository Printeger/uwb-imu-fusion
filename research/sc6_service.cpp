#include "unified_rolling_core.hpp"
#include "bounded_range_time.hpp"
#include "native_semantic_audit.hpp"
#include <fstream>
#include <unistd.h>
namespace uwb_imu_pl::research {
namespace {using Clock=std::chrono::steady_clock;double ms(Clock::time_point a){return std::chrono::duration<double,std::milli>(Clock::now()-a).count();}
size_t bindRangeTime(std::vector<PendingFactorGroup>& groups,const UwbBatch& batch,const RangeTimeRadii& radii,const Eigen::Vector3d& lever){
 size_t count=0;
 for(auto& group:groups){if(group.kind!=FactorKind::UwbBatch)continue;
  bool nonzero=false;for(auto id:group.source_measurements)nonzero|=radii.at(id)>0;
  if(!nonzero)continue; // Exact zero-span model uses the original native factor.
  UwbBatch subset;subset.id=batch.id;subset.timestamp=batch.timestamp;subset.covariance_m2=group.raw_covariance;
  subset.covariance_model_id="original-principal-covariance";
  for(auto id:group.source_measurements){auto found=std::find_if(batch.measurements.begin(),batch.measurements.end(),[&](const auto& m){return m.id==id;});
   if(found==batch.measurements.end())throw std::invalid_argument("TIME_FACTOR_ORIGINAL_SOURCE_MISSING");subset.measurements.push_back(*found);}
  if(group.keys.size()!=1)throw std::invalid_argument("TIME_FACTOR_KEY_LAYOUT");
  group.factors.resize(0);group.factors.add(boost::make_shared<BoundedTimeRangeFactor>(group.keys.front(),subset,lever,radii));
  group.model_id="sc9-original-range-bounded-time-profile-v1";group.noise_model_id="original-covariance-with-deterministic-whitened-box-nuisance";++count;
 }
 return count;
}
bool stableRotationMetric(const YAML::Node& law,bool tight){if(!law["closure"] || !law["closure"]["rotation_metric"])return tight;auto mode=law["closure"]["rotation_metric"].as<std::string>();if(mode=="frozen_boundary")return true;if(mode=="polar_prototype")return false;throw std::invalid_argument("ROTATION_METRIC_UNREGISTERED");}
RawInclusionParameters params(const IntegrityConfig& cfg,const YAML::Node& law){RawInclusionParameters p;p.initial_sigmas=cfg.realtime.prior_sigmas;p.numeric.valid=true;p.accel_adc=law["raw_accel_sigma_mps2"].as<double>();p.gyro_adc=law["raw_gyro_sigma_radps"].as<double>();p.accel_rw=cfg.imu.accelerometer_bias_rw_sigma;p.gyro_rw=cfg.imu.gyroscope_bias_rw_sigma;p.gravity=cfg.imu.gravity_mps2;return p;}}
UnifiedRollingIntegrityCore::UnifiedRollingIntegrityCore(const YAML::Node& law,const IntegrityConfig& cfg,const NavigationState& initial,const Eigen::Vector3d& lever,const Sc6MotionDomain& domain,const ImuMeasurement& first,bool tight,bool shadow,bool optimized):config_(cfg),lever_(lever),domain_(domain),policy_(resolveRollingPolicy(law,config_)),optimized_(optimized),event_(policy_.budget),proof_(policy_,initial,cfg.realtime.prior_sigmas,cfg.imu.accelerometer_bias_rw_sigma,cfg.imu.gyroscope_bias_rw_sigma,lever,domain.acceleration,domain.rate,tight,optimized),faults_(std::make_unique<OpenFaultPool>(initial,params(cfg,law),policy_.budget,policy_.max_fault_records,domain.acceleration,domain.rate)),recovery_(initial,policy_,stableRotationMetric(law,tight),optimized),backend_(config_,lever){
 reference_bias_=proof_.certificate().separator.bias;
 if(law["sc7"] && law["sc7"]["joint_raw"] && law["sc7"]["joint_raw"].as<bool>()){auto p=params(cfg,law);JointRawLaw joint;joint.accel_adc=p.accel_adc;joint.gyro_adc=p.gyro_adc;joint.accel_rw=p.accel_rw;joint.gyro_rw=p.gyro_rw;joint.gravity=p.gravity;joint.acceleration=domain.acceleration;joint.rate=domain.rate;joint.jerk=domain.jerk;proof_.enableJoint(first,joint,law["sc7"]["joint_sweeps"]?law["sc7"]["joint_sweeps"].as<unsigned>():2);}
 if(law["sc7"] && law["sc7"]["tag_support"] && law["sc7"]["tag_support"].as<bool>())proof_.enableTagSupport();
 if(law["sc7"] && law["sc7"]["numerical_recovery"])recovery_.enableNumericalRecovery(law["sc7"]["numerical_recovery"].as<bool>());
 if(law["sc7"] && law["sc7"]["attribute"])proof_.enableAttribution(law["sc7"]["attribute"].as<bool>());
 if(shadow)shadow_=std::make_unique<RollingIntegrity>(policy_,initial,cfg.realtime.prior_sigmas,cfg.imu.accelerometer_bias_rw_sigma,cfg.imu.gyroscope_bias_rw_sigma,lever,domain.acceleration,domain.rate);
 if(law["closure"] && law["closure"]["temporal_fd"]){if(law["closure"]["temporal_fd"].as<std::string>()!="sc6-adjacent-necessary-v1")throw std::invalid_argument("TEMPORAL_VERSION_MISMATCH");temporal_=std::make_unique<Sc6TemporalMonitor>(params(cfg,law),policy_.budget,domain,policy_.max_fault_records);temporal_->append(first);}
 if(!policy_.open)throw std::invalid_argument("SC6_REQUIRES_OPEN_EVENT");backend_.initialize(initial,cfg.realtime.prior_sigmas);backend_.ingestImu(first);faults_->append(first);replay_.push_back(first);capsules_.push_back(proof_.certificate().separator);
}
UnifiedRollingIntegrityCore::UnifiedRollingIntegrityCore(const YAML::Node& law,const IntegrityConfig& cfg,const NavigationState& initial,const Eigen::Vector3d& lever,const Sc6MotionDomain& domain,const ImuMeasurement& first,GeneralizedCoreInitialization model):
 config_(cfg),lever_(lever),domain_(domain),policy_(resolveRollingPolicy(law,config_)),optimized_(true),event_(policy_.budget),
 proof_(policy_,model.initial_set,cfg.imu.accelerometer_bias_rw_sigma,cfg.imu.gyroscope_bias_rw_sigma,lever,domain.acceleration,domain.rate,true,true),
 recovery_(initial,policy_,true,true),sources_(std::move(model.sources)),anchors_(std::move(model.anchors)),backend_(config_,lever){
 if(!sources_ || initial.timestamp!=model.initial_set.timestamp || first.timestamp!=initial.timestamp ||
    sources_->status()!=model.initial_set.status || model.raw_law.gravity!=cfg.imu.gravity_mps2 || sources_->event().id()!=model.initial_set.event_id || sources_->event().coordinates()!=model.initial_set.source_prefix ||
    sources_->event().prefixUpper()!=model.initial_set.risk_upper || model.raw_law.acceleration!=domain.acceleration ||
    model.raw_law.rate!=domain.rate || model.raw_law.jerk!=domain.jerk)throw std::invalid_argument("GENERALIZED_INITIALIZATION_SOURCE_MISMATCH");
 const auto& registered=sources_->law();
 if(registered.accel_adc!=model.raw_law.accel_adc || registered.gyro_adc!=model.raw_law.gyro_adc || registered.accel_rw!=model.raw_law.accel_rw || registered.gyro_rw!=model.raw_law.gyro_rw)
  throw std::invalid_argument("GENERALIZED_NOISE_MODEL_MISMATCH");
 proof_.enableGeneralizedJoint(model.raw_law);proof_.enableTagSupport();recovery_.enableNumericalRecovery(true);recovery_.enableActualTime();
 generalized_monitor_=std::make_unique<GeneralizedFaultMonitor>(model.raw_law,domain,model.initial_set,policy_.max_fault_records);
 sources_->append(first);generalized_monitor_->append(first,sources_->event(),sources_->sensor());
 backend_.initialize(initial,cfg.realtime.prior_sigmas);backend_.enableGeneralizedGroupIdentityResearchV2();backend_.enablePhysicalTimeResearchV1();backend_.ingestImu(first);
 reference_bias_=proof_.certificate().separator.bias;replay_.push_back(first);capsules_.push_back(proof_.certificate().separator);
}
Sc6Result UnifiedRollingIntegrityCore::step(const Sc6Envelope& input,bool fde)try{
 if(failed_)throw std::logic_error("CORE_FAILED_CLOSED");
 if(!sources_ && (!input.source_valid || input.imu.size()!=10 || input.epoch!=input.uwb.id.value()))throw std::invalid_argument("SC6_ARRIVED_SOURCE_UNQUALIFIED");
 if(input.epoch!=proof_.certificate().covered_through+1)throw std::invalid_argument("CORE_LOGICAL_EPOCH_GAP");
 RangeCounts counts;
 if(sources_){
  counts=validateRanges(input.range_events,anchors_,input.ready);
  UwbBatch selected;selected.id=input.uwb.id;selected.timestamp=input.uwb.timestamp;
  for(const auto& range:input.range_events){if(anchors_.at(range.measurement.anchor_id,range.calibration_version).frame!="world")throw std::invalid_argument("CORE_ANCHOR_FRAME_MISMATCH");sources_->registerPotentialRange(range);if(range.valid)selected.measurements.push_back(range.measurement);}
  if(!sameRawBatch(selected,input.uwb) || input.ready<input.uwb.timestamp)throw std::invalid_argument("CORE_CANONICAL_BATCH_MISMATCH");
  for(const auto& raw:input.imu)if(input.uwb.timestamp<raw.timestamp)throw std::invalid_argument("CORE_FUTURE_IMU");
 }
 Sc6Result result;result.input_counts=counts;result.epoch=input.epoch;auto allocation=sources_?TimeUniformBlock{}:event_.block(input.epoch);auto start=Clock::now();
 result.monitor_min_margin.fill(INFINITY);result.temporal_margin.fill(INFINITY);result.alarm_time=Clock::time_point{};if(sources_){
  for(const auto& m:input.imu){sources_->append(m);backend_.ingestImu(m);
   if(generalized_monitor_->append(m,sources_->event(),sources_->sensor()))result.alarm_time=Clock::now();
   const auto& d=generalized_monitor_->diagnostic();for(int j=0;j<6;++j){
    result.monitor_min_margin[j]=std::min(result.monitor_min_margin[j],std::min(d.measured[j]-d.healthy_low[j],d.healthy_high[j]-d.measured[j]));
    if(generalized_monitor_->margin[j]<result.temporal_margin[j]){result.temporal_margin[j]=generalized_monitor_->margin[j];result.temporal_difference[j]=generalized_monitor_->difference[j];result.temporal_radius[j]=generalized_monitor_->radius[j];}}
   replay_.push_back(m);
  }
 }else{size_t old_alerts=faults_->records().size();for(const auto& m:input.imu){backend_.ingestImu(m);faults_->append(m);const auto& d=faults_->diagnostic();for(int j=0;j<6;++j)result.monitor_min_margin[j]=std::min(result.monitor_min_margin[j],std::min(d.measured[j]-d.healthy_low[j],d.healthy_high[j]-d.measured[j]));if(faults_->records().size()>old_alerts){result.alarm_time=Clock::now();old_alerts=faults_->records().size();++combined_alert_generation_;}if(temporal_){if(temporal_->append(m,faults_->currentAlarm(),faults_->records().size())){++combined_alert_generation_;result.alarm_time=Clock::now();}for(int j=0;j<6;++j)if(temporal_->margin[j]<result.temporal_margin[j]){result.temporal_margin[j]=temporal_->margin[j];result.temporal_difference[j]=temporal_->difference[j];result.temporal_radius[j]=temporal_->radius[j];}if(temporal_->records().size()+faults_->records().size()>policy_.max_fault_records)throw std::invalid_argument("SC6_COMBINED_FAULT_RECORD_CAPACITY");}replay_.push_back(m);}while(replay_.size()>size_t(10*policy_.replay+1))replay_.pop_front();}
 result.imu_fd_ms=ms(start);auto graph_prepare_start=Clock::now();auto tx=sources_ && input.uwb.measurements.empty()?backend_.prepareEmptyEpochResearchV1(input.uwb):backend_.prepareEpoch(input.uwb);result.graph_prepare_ms=ms(graph_prepare_start);result.prepare_ms=ms(start);start=Clock::now();
 std::shared_ptr<const GeneralizedRawEvidence> evidence;
 RollingMonitorView view;
 if(sources_){evidence=sources_->evidence(tx);view=generalized_monitor_->view(tx,sources_->event());
  view.original_time=[this](MeasurementId id){return sources_->originalTime(id);};
  const auto endpoint=rollingPropagate(capsules_.back(),domain_.acceleration,domain_.rate,evidence->evolution);
  for(const auto& m:tx.uwb_batch.measurements){const double radius=rangeTransportRadius(endpoint,m.timestamp,tx.end,lever_,domain_.acceleration,domain_.rate).upper();
   view.range_transport[m.id]=radius;result.max_range_transport=std::max(result.max_range_transport,radius);}
  view.has_range_time_model=true;range_time_history_[tx.proposed_epoch]=view.range_transport;
  result.time_factor_groups=bindRangeTime(tx.uwb_groups,tx.uwb_batch,view.range_transport,lever_);
  for(auto& history:tx.recoverable_history){bool has_uwb=false;for(const auto& group:history.groups)has_uwb|=group.kind==FactorKind::UwbBatch;
   if(has_uwb)bindRangeTime(history.groups,history.uwb_batch,range_time_history_.at(history.proposed_epoch),lever_);}

  result.monitor=generalized_monitor_->diagnostic();combined_alert_generation_=view.alert_generation;
  replay_parents_.push_back(tx.raw_imu_slice.front().id);while(replay_parents_.size()>size_t(policy_.replay+1))replay_parents_.pop_front();
  while(!replay_.empty() && replay_.front().id!=replay_parents_.front())replay_.pop_front();
 }else view=faults_->view();
 if(faults_)result.monitor=faults_->diagnostic();result.imu_alarm=view.state.alarm;result.alert_generation=view.alert_generation;result.imu_active=view.active;result.imu_ended=view.ended;result.imu_excluded=view.excluded;result.first_imu_alarm_ns=view.state.first_alarm.value();result.uwb_active=faults_?faults_->activeUwb():0;result.fault_theta_low.fill(INFINITY);result.fault_theta_high.fill(-INFINITY);if(faults_)for(const auto& c:faults_->activeCandidates()){int a=c.episode.axis;result.fault_theta_low[a]=std::min(result.fault_theta_low[a],c.theta_low);result.fault_theta_high[a]=std::max(result.fault_theta_high[a],c.theta_high);}auto action_view=view;if(temporal_ && temporal_->alarm()){if(!action_view.state.alarm){action_view.state.union_begin=temporal_->begin();action_view.state.union_end=temporal_->end();}else{action_view.state.union_begin=std::min(action_view.state.union_begin,temporal_->begin());action_view.state.union_end=std::max(action_view.state.union_end,temporal_->end());}action_view.state.alarm=true;}
 // An operator-held action can be resumed after the alarm is no longer live.
 // Preserve the newest observed support with its original identity; otherwise
 // open_pending would rearm a temporal generation with the unrelated pool's
 // empty/stale interval. This does not rearm the physical source event.
 if(temporal_ && !temporal_->alarm() && !temporal_->records().empty() && temporal_->firstAlarm().value()>view.state.first_alarm.value()){const auto& observed=temporal_->records().back();action_view.state.union_begin=observed.begin;action_view.state.union_end=observed.end;}
 action_view.alert_generation=combined_alert_generation_;result.imu_alarm=action_view.state.alarm;result.alert_generation=combined_alert_generation_;if(temporal_)result.first_imu_alarm_ns=std::max(result.first_imu_alarm_ns,temporal_->firstAlarm().value());result.imu_alarm_time=result.alarm_time;result.action=recovery_.prepare(tx,action_view,replay_,capsules_,config_,fde);if(result.action.uwb_alarm_time!=Clock::time_point{} && (result.alarm_time==Clock::time_point{} || result.action.uwb_alarm_time<result.alarm_time))result.alarm_time=result.action.uwb_alarm_time;result.candidate_ms=ms(start);result.selected_time=Clock::now();start=Clock::now();
 RollingBoundaryProposalV1 old_proposal;if(shadow_)old_proposal=shadow_->propose(tx,result.action.plan,input.range,view);auto proposal=proof_.propose(tx,result.action.plan,input.range,view,evidence);auto capsule=proposal.current;if(faults_)faults_->appendRanges(tx.uwb_batch,capsule,lever_);result.proof_ms=ms(start);start=Clock::now();
 EpochTransaction saved;if(optimized_){saved.id=tx.id;saved.base_graph_version=tx.base_graph_version;saved.imu_group.id=tx.imu_group.id;saved.generic_bridge_group.id=tx.generic_bridge_group.id;saved.generic_bias_continuity_group.id=tx.generic_bias_continuity_group.id;}else saved=tx;auto count=backend_.backendUpdateCount();auto receipt=result.action.current_bridge?backend_.commitEpochRollingResearchV1(std::move(tx),result.action.plan):backend_.commitEpoch(std::move(tx),result.action.plan);result.commit_ms=ms(start);result.commit_time=Clock::now();start=Clock::now();
 result.backend_updates=receipt.backend_updates;result.state=backend_.currentState();result.tag=proof_.committedTag(result.state);auto keys=backend_.retainedKeysV1();if(receipt.backend_updates!=1 || backend_.backendUpdateCount()!=count+1)throw std::runtime_error("SC6_SINGLE_COMMIT");
 if(sources_){std::set<FactorId> ids;for(const auto& entry:backend_.factorLedger().entries())if(entry.lifecycle==FactorLifecycle::Active){
  if(!ids.insert(entry.factor_id).second)throw std::runtime_error("ACTIVE_NATIVE_FACTOR_ID_COLLISION");
  if(dynamic_cast<const BoundedTimeRangeFactor*>(entry.factor.get()))++result.time_committed_factors;
 }}
 result.state_receipt_ms=ms(start);auto finalize_start=Clock::now();result.bound=proof_.finalize(std::move(proposal),receipt,keys,result.state);if(shadow_){auto old=shadow_->finalize(std::move(old_proposal),receipt,keys,result.state);result.old_hpl=old.hpl;result.old_vpl=old.pl.z();}result.finalize_ms=ms(finalize_start);auto maintenance_start=Clock::now();recovery_.committed(result.action,saved,receipt,backend_.factorLedger());saved=EpochTransaction{};proposal=RollingBoundaryProposalV1{};
 if(sources_)while(range_time_history_.size()>size_t(policy_.replay+1))range_time_history_.erase(range_time_history_.begin());
 capsules_.push_back(capsule);while(capsules_.size()>size_t(policy_.replay+1))capsules_.pop_front();
 auto audit=backend_.onlineMaterialAuditV1();const auto& cert=proof_.certificate();result.version=receipt.graph_version;result.generation=cert.generation;result.parent=cert.parent;result.boundary=cert.separator.epoch;result.source_prefix=cert.source_prefix;result.temporal_alerts=temporal_?temporal_->records().size():0;result.alerts=(faults_?faults_->records().size():generalized_monitor_->alerts())+result.temporal_alerts;result.active_states=audit.values/3;result.proof_explicit=proof_.explicitSize();result.slots=audit.factor_slots;result.live=audit.active_factors;result.ledger=audit.ledger_entries;result.replay=replay_.size();result.capsules=capsules_.size();result.x0=std::find(keys.begin(),keys.end(),gtsam::Symbol('x',0))!=keys.end();result.bias_width=capsule.bias[0].upper()-capsule.bias[0].lower();result.q=evidence?evidence->max_coordinate_radius:allocation.q;result.shared_q=evidence?sources_->event().birth({SourceKind::Adc,sources_->sensor(),replay_parents_.back().value(),0}).q:event_.adcRadius((input.epoch-1)*10);
 for(int j=0;j<6;++j){using I=RollingInterval;I radius=evidence?evidence->evolution.bias_radius[j]:10.*I(allocation.q)*I(j<3?config_.imu.accelerometer_bias_rw_sigma:config_.imu.gyroscope_bias_rw_sigma)*sqrt(I(1)/200.)+I(2e-10);reference_bias_[j]+=I(-radius.upper(),radius.upper());result.reference_bias_width[j]=(I(reference_bias_[j].upper())-I(reference_bias_[j].lower())).upper();}
 for(int j=0;j<3;++j){result.tag_width[j]=result.bound.high(j)-result.bound.low(j);result.velocity_width[j]=capsule.v[j].upper()-capsule.v[j].lower();for(int h=0;h<3;++h)result.rotation_width[j]=std::max(result.rotation_width[j],capsule.rotation[j][h].upper()-capsule.rotation[j][h].lower());}for(int j=0;j<6;++j)result.bias_widths[j]=capsule.bias[j].upper()-capsule.bias[j].lower();result.proof_time=Clock::now();
 if(semantic_audit_){if(evidence)result.source_digest=sourceDigest(input,*evidence);result.receipt_digest=receiptDigest(receipt);result.ledger_digest=ledgerDigest(backend_.factorLedger());result.boundary_digest=boundaryDigest(cert);}
 size_t virt=0,res=0;std::ifstream("/proc/self/statm")>>virt>>res;result.rss=res*size_t(sysconf(_SC_PAGESIZE))/1024;result.action.removed_boundary.clear();result.maintenance_ms=ms(maintenance_start);result.cleanup_ms=ms(start);return result;
}catch(...){failed_=true;throw;}
}
