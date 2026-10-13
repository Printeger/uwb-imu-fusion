#include "sc3_joint_coordinator.hpp"
#include "scenario_material.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <tuple>
#include <optional>
namespace uwb_imu_pl {
FdeDecision FdeManager::decideRawJointResearch(std::vector<CandidateEvaluation>* candidates,const research::Sc3DecisionContext& context)const{
 FdeDecision decision;decision.status=FdeStatus::ModelInvalid;
 if(!candidates || !context.bound || !context.monitor || !context.range || !context.transaction || !context.centers || !context.search || !context.trusted || context.centers->size()!=candidates->size())return decision;
 const auto& search=*context.search;
 auto census=validateActionSearchCensusV1(search.census,*context.trusted,search.max_evaluated_actions,search.lifecycle,search.actions);
 if(!census.valid || !census.exhaustive || (context.expected_roots!=1 && context.expected_roots!=18) || search.census.generated!=context.expected_roots || search.actions.size()!=candidates->size()){decision.status=FdeStatus::SearchIncomplete;decision.reason="joint action census incomplete";return decision;}
 std::vector<size_t> eligible;
 for(size_t i=0;i<candidates->size();++i){auto& c=candidates->at(i);
  if(exactActionSemanticIdentityV1(c.action)!=exactActionSemanticIdentityV1(search.actions[i]))return decision;
  if(!c.valid)continue;
  auto proof=context.bound->project(*context.monitor,*context.range,*context.transaction,context.centers->at(i),context.lever);
  if(!proof.valid || proof.evaluated!=481401 || proof.geometry.pl!=c.pl_xyz_m || proof.geometry.hpl!=c.hpl_m || proof.geometry.pl.z()!=c.vpl_m){decision.reason="joint candidate source/numeric proof mismatch";return decision;}
  c.covers_plausible_set=true;
  if(c.post_detector_passed)eligible.push_back(i);
 }
 if(eligible.empty()){decision.status=FdeStatus::NoValidCandidate;decision.reason="no finite complete post-passed joint candidate";return decision;}
 std::sort(eligible.begin(),eligible.end(),[&](size_t a,size_t b){const auto& x=candidates->at(a);const auto& y=candidates->at(b);return std::make_tuple(x.action.exclusion_cardinality,x.statistic,x.action.id.value())<std::make_tuple(y.action.exclusion_cardinality,y.statistic,y.action.id.value());});
 auto& chosen=candidates->at(eligible[0]);chosen.selected=true;decision.selected_action=chosen.action;decision.commit_allowed=true;
 decision.status=chosen.action.exclusion_cardinality>1?FdeStatus::AmbiguousUnionExclusion:chosen.action.bridge_mode!=BridgeMode::None?FdeStatus::SuccessImuExclusionGenericBridge:chosen.action.exclusion_cardinality?FdeStatus::SuccessUwbExclusion:FdeStatus::NotTriggered;
 // Separate global common-source sup-fault theorem. No production allocation,
 // HMI ledger, formal eligibility or AL qualification is inherited.
 decision.integrity_available=false;decision.reason="SC3 raw common event covers full physical product; AL/deployment unqualified";
 return decision;
}
}
namespace uwb_imu_pl::research {
Sc3JointCoordinator::Sc3JointCoordinator(const NavigationState& initial,const RawInclusionParameters& law,const UwbBatch& identity,size_t capacity,bool diagnostics,bool skip):initial_(initial),law_(law),imu_(initial,law),bound_(initial,law,identity),action_capacity_(capacity),recovery_diagnostics_(diagnostics),skip_post_rejected_numeric_(skip){if(capacity<18)throw std::runtime_error("SC3 complete action capacity refused");}
namespace {
ExclusionAction rangeAction(const EpochTransaction& tx,int excluded){
 ExclusionAction a;if(excluded<0)return a;
 a.exclusion_cardinality=1;const auto anchor=tx.uwb_batch.measurements.at(excluded).anchor_id;
 a.physical_source_ids={"uwb:"+std::to_string(anchor.value())};a.groups_to_remove.push_back(tx.uwb_groups.front().id);
 auto contains=[&](const PendingFactorGroup& g,MeasurementId id){return std::find(g.source_measurements.begin(),g.source_measurements.end(),id)!=g.source_measurements.end();};
 bool current=false;for(const auto& group:tx.uwb_groups)if(group.kind==FactorKind::UwbBatch && group.source_measurements.size()==7 && !contains(group,tx.uwb_batch.measurements[excluded].id)){a.groups_to_add.push_back(group.id);current=true;break;}
 if(!current)throw std::runtime_error("SC3 native current UWB replacement missing");
 // Include all active original/history rows of this source. Previous FD PASS
 // is never proof that a historical row could not carry the same episode.
 for(const auto& history:tx.recoverable_history){MeasurementId row;
  for(const auto& m:history.uwb_batch.measurements)if(m.anchor_id==anchor)row=m.id;
  for(const auto& group:history.groups)if(group.kind==FactorKind::UwbBatch && std::find(history.selected_groups.begin(),history.selected_groups.end(),group.id)!=history.selected_groups.end() && contains(group,row)){
   bool replacement=false;for(const auto& candidate:history.groups)if(candidate.replaces_group && *candidate.replaces_group==group.id && candidate.kind==FactorKind::UwbBatch && candidate.source_measurements.size()+1==group.source_measurements.size() && !contains(candidate,row)){a.groups_to_remove.push_back(group.id);a.groups_to_add.push_back(candidate.id);replacement=true;break;}
   if(!replacement)throw std::runtime_error("SC3 native historical UWB replacement missing");
  }
 }
 return a;
}
void seedFrom(const BoundedRecoveryAttempt& candidate,size_t epoch,NavigationState& state){const auto pose=candidate.reference.at<gtsam::Pose3>(gtsam::Symbol('x',epoch));const auto bias=candidate.reference.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',epoch));state.position_world_m=pose.translation();state.velocity_world_mps=candidate.reference.at<gtsam::Vector3>(gtsam::Symbol('v',epoch));state.q_world_body=Eigen::Quaterniond(pose.rotation().matrix());state.accel_bias_mps2=bias.accelerometer();state.gyro_bias_radps=bias.gyroscope();}
}
Sc3NativeAssessment Sc3JointCoordinator::prepare(EpochTransaction& tx,const RawRangeLawEvidence& source,const Eigen::Vector3d& lever,const GenericBridgeSpec& spec,bool fde){
 Sc3NativeAssessment out;auto start=std::chrono::steady_clock::now();auto all=uwbCandidate(tx.uwb_batch,-1,1e-6,1e-9);out.uwb_alarm=all.gaussian.dof>0 && all.gaussian.statistic>all.gaussian.threshold;out.detector_statistic=all.gaussian.statistic;out.detector_threshold=all.gaussian.threshold;
 bool held=anchor_>=0 && int(tx.proposed_epoch)<=uwb_end_;bool search=fde && (out.uwb_alarm || imu_.monitor().alarm || held);
 std::vector<ExclusionAction> actions;std::vector<EpochTransaction> transactions;std::vector<Sc2RecoveryCoordinator> coordinators;std::vector<Sc2NativeAssessment> assessments;std::vector<NavigationState> centers;std::vector<CandidateEvaluation> candidates;
 std::optional<std::tuple<size_t,double,uint64_t>> knownEligible;
 for(int mask=-1;mask<(search?8:0);++mask){auto post=mask<0?all:uwbCandidate(tx.uwb_batch,mask,1e-6,1e-9);
  for(int removeImu=0;removeImu<(search?2:1);++removeImu){auto candidateTx=tx;auto coordinator=imu_;auto action=rangeAction(candidateTx,mask);action.id=ExclusionActionId(5000+(mask+1)*2+removeImu);action.action_model_id="sc3-raw-full-product-root-mask-"+std::to_string(mask)+"-imu-"+std::to_string(removeImu);
   auto assessment=coordinator.prepare(candidateTx,removeImu!=0,spec,&action,removeImu!=0,true);CandidateEvaluation candidate;candidate.action=assessment.action;candidate.statistic=post.gaussian.statistic;
   candidate.valid=assessment.legal;NavigationState center=candidateTx.cv_predicted_state;center.timestamp=tx.end;
   // This predicate has no graph-reference dependency. A false predicate can
   // NEVER enter the manager's eligible set, regardless of numeric validity.
   // Keep its action/physical identity; omit only its unconsumed GN/PL work.
   bool rangePassed=post.status=="FROZEN_MODEL_ONLY" && post.gaussian.statistic<=post.gaussian.threshold;
   bool imuPassed=!imu_.monitor().alarm || removeImu;
   bool heldPassed=!held || mask==anchor_;
   candidate.post_detector_passed=!fde || ((!search || rangePassed) && imuPassed && heldPassed);
   auto priority=std::make_tuple(size_t(candidate.action.exclusion_cardinality),candidate.statistic,candidate.action.id.value());
   bool postRejected=skip_post_rejected_numeric_ && !candidate.post_detector_passed;
   bool dominated=skip_post_rejected_numeric_ && candidate.post_detector_passed && knownEligible && priority>*knownEligible;
   bool skipNumeric=postRejected || dominated;
   if(skipNumeric){candidate.valid=false;assessment.candidate.status=postRejected?"POST_REJECTED_NO_NUMERIC_CONSUMER":"ORDER_DOMINATED_BY_VALID_ELIGIBLE_ACTION";if(postRejected)++out.numeric_skipped_post_rejected;else ++out.numeric_skipped_dominated;}
   else ++out.numeric_evaluated;
   if(candidate.valid && !assessment.action.groups_to_remove.empty()){
    if(!candidateTx.frozen_values->exists(gtsam::Symbol('x',0)) || candidateTx.history_recoverability!=HistoryRecoverability::Recoverable)candidate.valid=false;
    else{auto rebuildAction=assessment.action;if(rebuildAction.bridge_mode==BridgeMode::None)rebuildAction.bridge_mode=BridgeMode::GenericKinematic;
     assessment.candidate=boundedRecovery(candidateTx,rebuildAction,initial_,.04,.2,&coordinator.observedPrefix(),&spec,recovery_diagnostics_);
     out.total_rebuilds+=assessment.candidate.rebuilds;out.total_reintegrations+=assessment.candidate.raw_reintegrations;out.diagnostic_calls+=assessment.candidate.diagnostic_calls;
     out.optimizer_ms+=assessment.candidate.optimizer_ms;out.reintegrations_ms+=assessment.candidate.raw_reintegration_ms;out.diagnostics_ms+=assessment.candidate.diagnostic_ms;
     candidate.valid=assessment.candidate.rebuilds==2 && std::isfinite(assessment.candidate.objective_after);
     if(candidate.valid){seedFrom(assessment.candidate,tx.proposed_epoch,center);candidateTx.cv_predicted_state=center;}
    }
   }
   Sc3JointBound proof;if(!skipNumeric)proof=bound_.project(coordinator.rawMonitor(),source,candidateTx,center,lever);
   candidate.valid=candidate.valid && proof.valid;candidate.pl_xyz_m=proof.geometry.pl;candidate.hpl_m=proof.geometry.hpl;candidate.vpl_m=proof.geometry.pl.z();
   if(candidate.valid && candidate.post_detector_passed && (!knownEligible || priority<*knownEligible))knownEligible=priority;
   // Gaussian range profile is functional evidence only; no source/fault branch
   // is removed. The raw final proof protects even a missed/incorrect action.
   actions.push_back(candidate.action);transactions.push_back(std::move(candidateTx));coordinators.push_back(std::move(coordinator));assessments.push_back(std::move(assessment));centers.push_back(center);candidates.push_back(std::move(candidate));
  }
 }
 GeneratedActionSnapshotV1 trusted;auto census=censusAndCapActionsV1(actions,action_capacity_,&trusted);
 Sc3DecisionContext context{&bound_,&imu_.rawMonitor(),&source,&tx,&centers,&census,&trusted,lever,size_t(search?18:1)};out.decision=FdeManager{}.decideRawJointResearch(&candidates,context);
 out.candidates=candidates.size();out.expected_actions=census.census.generated;out.evaluated_actions=census.census.evaluated;
 for(const auto& c:candidates){out.finite_candidates+=c.valid;out.post_candidates+=c.valid && c.post_detector_passed;}
 if(!out.decision.commit_allowed)return out;
 size_t winner=0;while(winner<candidates.size() && !candidates[winner].selected)++winner;if(winner==candidates.size())return out;
 out.excluded=int((out.decision.selected_action->id.value()-5000)/2)-1;tx=std::move(transactions[winner]);imu_=std::move(coordinators[winner]);out.imu=std::move(assessments[winner]);out.legal=true;
 if(out.excluded>=0){for(auto id:out.imu.action.groups_to_remove)for(const auto& h:tx.recoverable_history)for(const auto& g:h.groups)if(g.id==id && g.kind==FactorKind::UwbBatch)++out.uwb_history_removed;
  if(anchor_!=out.excluded || int(tx.proposed_epoch)>uwb_end_){anchor_=out.excluded;uwb_end_=int(tx.proposed_epoch)+2;}}
 out.imu_rejoined=out.imu.status=="IMU_REJOINED_ORIGINAL_BIAS_REINTEGRATION";
 out.status=out.imu.current_bridge?(out.excluded>=0?"JOINT_UWB_IMU_BRIDGE":"IMU_SUPPORT_BRIDGE"):out.excluded>=0?"UWB_EXCLUDED":out.imu_rejoined?"IMU_REJOINED":"KEEP";
 out.candidate_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return out;
}
void Sc3JointCoordinator::committed(const Sc3NativeAssessment& assessment,const EpochTransaction& tx,const CommitReceipt& receipt){
 imu_.committed(assessment.imu,tx,receipt);
 if(assessment.excluded>=0){auto plan=sc2CommitPlan(tx,assessment.imu);for(auto id:plan.groups_to_add)for(const auto& g:tx.uwb_groups)if(g.id==id && g.kind==FactorKind::UwbBatch){if(!receipt.group_to_slots.count(g.id.value()) || receipt.group_to_slots.count(tx.uwb_groups.front().id.value()) || g.source_measurements.size()!=7 || std::find(g.source_measurements.begin(),g.source_measurements.end(),tx.uwb_batch.measurements[assessment.excluded].id)!=g.source_measurements.end())throw std::runtime_error("SC3 actual retained current UWB support incomplete");}}
}
}
namespace uwb_imu_pl::research {
int runSc3Joint(int argc,char** argv){
 using Clock=std::chrono::steady_clock;auto ms=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
 if(argc<5 || std::string(argv[4])!="hybrid")throw std::runtime_error("SC3 requires opt-in native hybrid runner");
 auto sourceStart=Clock::now();std::string kind=argv[3];NoisyMotionScenario scene(argv[1],kind,20261010,argc>6?std::stoi(argv[6]):30);
 if(!scene.physical_source_reference || !scene.initial_law_valid || !scene.imu_fault_numeric_certificate.valid || scene.law["sc3_scope_id"].as<std::string>()!="sc3-one-uwb-one-imu-episode-rw-v1" || scene.law["sc3_requested_categories"].as<std::vector<std::string>>()!=std::vector<std::string>({"H0","UWB","ACCEL","GYRO","UWB_ACCEL","UWB_GYRO"}) || scene.law["sc3_uwb_durations_epochs"].as<std::vector<int>>()!=std::vector<int>({1,3}) || scene.law["sc3_uwb_theta_domain"].as<std::string>()!="unbounded-constant-single-anchor" || scene.law["sc3_imu_scope"].as<std::string>()!="sc2-single-imu-episode-rw-v1" || scene.law["sc3_initial_carry_in"].as<bool>() || scene.law["sc3_mode_capacity"].as<size_t>()!=1400 || scene.law["sc3_range_box_capacity"].as<size_t>()!=256)throw std::runtime_error("SC3 registered source/scope admission refused; not FD");
 RawInclusionParameters law;law.numeric=scene.imu_fault_numeric_certificate;law.initial_sigmas=scene.cfg.realtime.prior_sigmas;law.accel_adc=scene.law["raw_accel_sigma_mps2"].as<double>();law.gyro_adc=scene.law["raw_gyro_sigma_radps"].as<double>();law.accel_rw=scene.cfg.imu.accelerometer_bias_rw_sigma;law.gyro_rw=scene.cfg.imu.gyroscope_bias_rw_sigma;law.gravity=scene.cfg.imu.gravity_mps2;law.motion_amplitude=scene.amp;law.motion_frequency=scene.freq;law.motion_angular_rate=scene.rate;law.horizon=scene.law["risk_horizon_epochs"].as<int>();law.risk_budget=scene.law["raw_event_risk"].as<double>();
 Sc3JointCoordinator coordinator(scene.initial_observation,law,scene.uwb[0],scene.law["sc3_action_capacity"].as<size_t>());IncrementalUwbImuEstimator backend(scene.cfg,scene.lever);backend.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);backend.ingestImu(scene.imu[0]);coordinator.append(scene.imu[0]);
 bool fde=argc<6 || std::string(argv[5])=="on";int enableEpoch=std::getenv("SC3_FDE_ENABLE_EPOCH")?std::stoi(std::getenv("SC3_FDE_ENABLE_EPOCH")):1;
 std::ofstream out(argv[2]);if(!out)throw std::runtime_error("SC3 trace unavailable");out<<std::setprecision(17);
 out<<"epoch,scene,fde,status,imu_alarm,first_imu_alarm_ns,uwb_alarm,excluded,imu_bridge,imu_rejoined,history_imu_purge,history_uwb_removed,rebuilds,candidate_status,actions_expected,actions_evaluated,finite_candidates,post_candidates,updates,transaction_id,graph_version,timestamp_ns,qualified,bound_method,scope,physical_expected,physical_represented,physical_evaluated,range_masks,hpl,vpl,risk_upper,tag_x,tag_y,tag_z,velocity_x,velocity_y,velocity_z,qw,qx,qy,qz,horizontal_pe,vertical_pe,velocity_error,rotation_error,raw_bound_status,candidate_ms,bound_ms,full_ms,on_time,formal_eligible,publication_protected,application_available,retained_ids,input_ids,source_init_ms,detector_statistic,detector_threshold\n";
 double sourceMs=ms(sourceStart,Clock::now());
 for(int e=1;e<=scene.epochs;++e){auto start=Clock::now();for(int i=(e-1)*10+1;i<=e*10;++i){backend.ingestImu(scene.imu[i]);coordinator.append(scene.imu[i]);}
  auto tx=backend.prepareEpoch(scene.uwb[e-1],EpochPreparationOptions{true,true,true,true});auto assessment=coordinator.prepare(tx,scene.range_certificates[e-1],scene.lever,scene.cfg.bridge.generic,fde && e>=enableEpoch);
  if(!assessment.legal){backend.discardEpoch(std::move(tx),DiscardReason{assessment.decision.status,assessment.decision.reason,false});throw std::runtime_error("SC3 aborted epoch="+std::to_string(e)+":"+assessment.decision.reason);}
  auto plan=sc2CommitPlan(tx,assessment.imu);plan.fde_status=assessment.decision.status;auto retained=tx;auto before=backend.backendUpdateCount();auto receipt=backend.commitEpoch(std::move(tx),plan);coordinator.committed(assessment,retained,receipt);auto state=backend.currentState();if(receipt.backend_updates!=1 || backend.backendUpdateCount()!=before+1)throw std::runtime_error("SC3 repeated native update");
  auto boundStart=Clock::now();auto proof=coordinator.issue(scene.range_certificates[e-1],retained,receipt,state,scene.lever);double boundMs=ms(boundStart,Clock::now());auto monitor=coordinator.monitor();Eigen::Vector3d tag=state.position_world_m+state.q_world_body*scene.lever;
  // All online calculations above precede any truth/injection scoring.
  double h=NAN,v=NAN,velocity=NAN,rotation=NAN;if(!std::getenv("SC3_REDACT_SCORING")){auto truth=scene.truth(e*.05);Eigen::Vector3d trueTag=truth.position_world_m+truth.q_world_body*scene.lever;h=(tag-trueTag).head<2>().norm();v=std::abs((tag-trueTag).z());velocity=(state.velocity_world_mps-truth.velocity_world_mps).norm();rotation=Eigen::AngleAxisd(state.q_world_body.conjugate()*truth.q_world_body).angle();}
  std::ostringstream ids,input;bool first=true;for(const auto& row:retained.uwb_batch.measurements){if(!first)input<<'|';input<<row.id.value();first=false;}first=true;for(auto id:plan.groups_to_add)for(const auto& g:retained.uwb_groups)if(g.id==id){for(auto row:g.source_measurements){if(!first)ids<<'|';ids<<row.value();first=false;}}
  double fullMs=ms(start,Clock::now());out<<e<<','<<(std::getenv("SC3_REDACT_SCORING")?"REDACTED":kind)<<','<<fde<<','<<assessment.status<<','<<monitor.alarm<<','<<monitor.first_alarm.value()<<','<<assessment.uwb_alarm<<','<<assessment.excluded<<','<<assessment.imu.current_bridge<<','<<assessment.imu_rejoined<<','<<assessment.imu.history_purge<<','<<assessment.uwb_history_removed<<','<<assessment.imu.candidate.rebuilds<<','<<assessment.imu.candidate.status<<','<<assessment.expected_actions<<','<<assessment.evaluated_actions<<','<<assessment.finite_candidates<<','<<assessment.post_candidates<<','<<receipt.backend_updates<<','<<receipt.transaction_id.value()<<','<<receipt.graph_version<<','<<state.timestamp.value()<<','<<proof.valid<<','<<proof.geometry.source<<','<<proof.scope<<','<<proof.expected<<','<<proof.represented<<','<<proof.evaluated<<','<<proof.geometry.considered<<','<<proof.geometry.hpl<<','<<proof.geometry.pl.z()<<','<<proof.geometry.event_risk;
  for(auto vector:{tag,state.velocity_world_mps})for(int j=0;j<3;++j)out<<','<<vector(j);
  out<<','<<state.q_world_body.w()<<','<<state.q_world_body.x()<<','<<state.q_world_body.y()<<','<<state.q_world_body.z()<<','<<h<<','<<v<<','<<velocity<<','<<rotation<<','<<proof.geometry.status<<','<<assessment.candidate_ms<<','<<boundMs<<','<<fullMs<<','<<(fullMs<=50)<<",0,0,0,"<<ids.str()<<','<<input.str()<<','<<sourceMs<<','<<assessment.detector_statistic<<','<<assessment.detector_threshold<<'\n';
  if(e==15 || e==17 || e==25 || e==26 || e==30)std::cout<<"SC3 "<<kind<<" epoch="<<e<<" "<<assessment.status<<" excluded="<<assessment.excluded<<" actions="<<assessment.evaluated_actions<<" H/V="<<proof.geometry.hpl<<'/'<<proof.geometry.pl.z()<<" qualified="<<proof.valid<<" service_ms="<<fullMs<<'\n';
 }
 return 0;
}
}
