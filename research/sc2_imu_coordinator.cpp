#include "sc2_imu_coordinator.hpp"
#include "scenario_material.hpp"
#include "raw_prefix_history.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <chrono>
namespace uwb_imu_pl::research {
Sc2RecoveryCoordinator::Sc2RecoveryCoordinator(const NavigationState& initial,const RawInclusionParameters& law,Sc2ImuScope scope):initial_(initial),law_(law),scope_(scope),monitor_(initial,law,scope),bridge_(initial,law,scope){}
void Sc2RecoveryCoordinator::append(const ImuMeasurement& sample){monitor_.append(sample);if(monitor_.assessment().valid)prefix_.push_back(sample);}
namespace {
void refreshPendingBiasHat(EpochTransaction& tx,const NavigationState& initial){
 auto params=boost::dynamic_pointer_cast<gtsam::PreintegrationCombinedParams>(tx.preintegration->params());
 auto pim=std::make_shared<gtsam::PreintegratedCombinedMeasurements>(params,gtsam::imuBias::ConstantBias(initial.accel_bias_mps2,initial.gyro_bias_radps));
 for(size_t i=1;i<tx.raw_imu_slice.size();++i){const auto& a=tx.raw_imu_slice[i-1];const auto& b=tx.raw_imu_slice[i];pim->integrateMeasurement(.5*(a.specific_force_mps2+b.specific_force_mps2),.5*(a.angular_velocity_radps+b.angular_velocity_radps),(b.timestamp.value()-a.timestamp.value())/1e9);}
 tx.preintegration=pim;tx.imu_group.factors.resize(0);tx.imu_group.factors.emplace_shared<gtsam::CombinedImuFactor>(gtsam::Symbol('x',tx.previous_epoch),gtsam::Symbol('v',tx.previous_epoch),gtsam::Symbol('x',tx.proposed_epoch),gtsam::Symbol('v',tx.proposed_epoch),gtsam::Symbol('b',tx.previous_epoch),gtsam::Symbol('b',tx.proposed_epoch),*pim);tx.imu_group.raw_covariance=pim->preintMeasCov();
}
}
EpochCommitPlan sc2CommitPlan(const EpochTransaction& tx,const Sc2NativeAssessment& assessment){
 auto plan=EpochCommitPlan::nominalPlan(tx);plan.action_id=assessment.action.id;plan.bridge_mode=assessment.action.bridge_mode;
 for(auto id:assessment.action.groups_to_remove){plan.groups_to_add.erase(std::remove(plan.groups_to_add.begin(),plan.groups_to_add.end(),id),plan.groups_to_add.end());if(id!=tx.imu_group.id)plan.groups_to_remove.push_back(id);}
 for(auto id:assessment.action.groups_to_add)plan.groups_to_add.push_back(id);
 if(assessment.current_bridge || assessment.history_purge){plan.fde_status=FdeStatus::SuccessImuExclusionGenericBridge;plan.recovery_epoch_begin=assessment.first_dependency;plan.recovery_epoch_end=tx.proposed_epoch;}
 return plan;
}
Sc2NativeAssessment Sc2RecoveryCoordinator::prepare(EpochTransaction& tx,bool fde,const GenericBridgeSpec& spec){
 Sc2NativeAssessment out;out.action.id=ExclusionActionId(2001);out.action.action_model_id="sc2-complete-raw-support-union";
 auto monitor=monitor_.assessment();
 if(!monitor.valid || (!monitor.nominal_possible && !monitor.possible) || !monitor_.matches(tx)){out.legal=false;out.status="SCOPE_OR_PREFIX_REFUSED";return out;}
 if(!fde){out.status="FDE_OFF";return out;}
 if(!monitor.alarm)return out;
 // A contracting hypothesis bank cannot move the already quarantined prefix
 // forward or release its last shared sample early. No injection-end input.
 if(quarantine_begin_.value()==0 || monitor.union_begin<quarantine_begin_)quarantine_begin_=monitor.union_begin;
 if(quarantine_end_<monitor.union_end)quarantine_end_=monitor.union_end;
 auto closure=sc2SupportClosure({"retained-physical-union",quarantine_begin_,quarantine_end_,0},tx.proposed_epoch);
 out.first_dependency=closure.first_epoch;
 bool direct=false;for(auto sample:tx.raw_imu_slice)direct=direct || (!(sample.timestamp<quarantine_begin_) && sample.timestamp<quarantine_end_);
 out.current_bridge=direct || !isolated_;
 if(!out.current_bridge){refreshPendingBiasHat(tx,initial_);out.status="IMU_REJOINED_ORIGINAL_BIAS_REINTEGRATION";return out;}
 if(!closure.valid || !closure.first_epoch || !tx.frozen_values || !tx.frozen_values->exists(gtsam::Symbol('x',0)) || tx.history_recoverability!=HistoryRecoverability::Recoverable){out.legal=false;out.status="BOUNDARY_OR_ACTIVE_PREFIX_MISSING";return out;}
 out.action.bridge_mode=BridgeMode::GenericKinematic;out.action.groups_to_remove={tx.imu_group.id};out.action.groups_to_add={tx.generic_bridge_group.id,tx.generic_bias_continuity_group.id};out.action.exclusion_cardinality=1;
 for(const auto& history:tx.recoverable_history)if(history.proposed_epoch>=closure.first_epoch){
  for(const auto& group:history.groups)if(group.kind==FactorKind::CombinedImu && std::find(history.selected_groups.begin(),history.selected_groups.end(),group.id)!=history.selected_groups.end()){
   out.history_purge=true;out.action.groups_to_remove.push_back(group.id);int pose=0,bias=0;
   for(const auto& replacement:history.groups)if(replacement.replaces_group && *replacement.replaces_group==group.id){if(replacement.kind==FactorKind::KinematicBridge){out.action.groups_to_add.push_back(replacement.id);++pose;}if(replacement.kind==FactorKind::BiasContinuity){out.action.groups_to_add.push_back(replacement.id);++bias;}}
   if(pose!=1 || bias!=1){out.legal=false;out.status="COMPLETE_HISTORICAL_BRIDGE_PAIR_MISSING";return out;}
  }
 }
 // Existing finite two-rebuild optimizer is a candidate calculation, never a
 // Gaussian/raw boundary certificate. New physical proof is independent of
 // its inverse Hessian, CV/bias weights and old constant-bias bridge.box.
 out.candidate=boundedRecovery(tx,out.action,initial_,scope_.acceleration_bound,scope_.angular_rate_bound,&prefix_,&spec);
 if(out.candidate.rebuilds!=2 || !std::isfinite(out.candidate.objective_after)){out.legal=false;out.status="FINITE_CANDIDATE_REBUILD_REFUSED";out.reason=out.candidate.status;return out;}
 auto pose=out.candidate.reference.at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));auto bias=out.candidate.reference.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',tx.proposed_epoch));
 tx.cv_predicted_state.position_world_m=pose.translation();tx.cv_predicted_state.q_world_body=Eigen::Quaterniond(pose.rotation().matrix());tx.cv_predicted_state.velocity_world_mps=out.candidate.reference.at<gtsam::Vector3>(gtsam::Symbol('v',tx.proposed_epoch));tx.cv_predicted_state.accel_bias_mps2=bias.accelerometer();tx.cv_predicted_state.gyro_bias_radps=bias.gyroscope();
 out.status="FULL_RAW_SUPPORT_BRIDGE";return out;
}
void Sc2RecoveryCoordinator::committed(const Sc2NativeAssessment& assessment,const EpochTransaction& tx,const CommitReceipt& receipt){
 if(!assessment.legal || receipt.transaction_id!=tx.id || receipt.graph_version!=tx.base_graph_version+1 || receipt.backend_updates!=1 || receipt.committed_epoch!=tx.proposed_epoch || receipt.state_timestamp!=tx.end)throw std::runtime_error("SC2 native recovery receipt refused");
 if(assessment.current_bridge){if(!receipt.group_to_slots.count(tx.generic_bridge_group.id.value()) || !receipt.group_to_slots.count(tx.generic_bias_continuity_group.id.value()) || receipt.group_to_slots.count(tx.imu_group.id.value()))throw std::runtime_error("SC2 current raw support not excluded");isolated_=true;}
 for(auto id:assessment.action.groups_to_remove)if(id!=tx.imu_group.id && std::find(receipt.historical_groups_removed.begin(),receipt.historical_groups_removed.end(),id)==receipt.historical_groups_removed.end())throw std::runtime_error("SC2 historical support receipt incomplete");
}
int runSc2Imu(int argc,char** argv){
 using Clock=std::chrono::steady_clock;auto ms=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
 if(argc<5 || std::string(argv[4])!="hybrid")throw std::runtime_error("SC2 opt-in requires native hybrid runner");
 std::string kind=argv[3];auto sourceStart=Clock::now();NoisyMotionScenario scene(argv[1],kind,20261010,argc>6?std::stoi(argv[6]):30);
 if(!scene.physical_source_reference || !scene.initial_law_valid || !scene.imu_fault_numeric_certificate.valid)throw std::runtime_error("SC2 nominal source admission refused; not a detector alarm");
 RawInclusionParameters law;law.numeric=scene.imu_fault_numeric_certificate;law.risk_budget=scene.law["raw_event_risk"].as<double>();law.horizon=scene.law["risk_horizon_epochs"].as<int>();law.initial_sigmas=scene.cfg.realtime.prior_sigmas;law.accel_adc=scene.law["raw_accel_sigma_mps2"].as<double>();law.gyro_adc=scene.law["raw_gyro_sigma_radps"].as<double>();law.accel_rw=scene.cfg.imu.accelerometer_bias_rw_sigma;law.gyro_rw=scene.cfg.imu.gyroscope_bias_rw_sigma;law.gravity=scene.cfg.imu.gravity_mps2;law.motion_amplitude=scene.amp;law.motion_frequency=scene.freq;law.motion_angular_rate=scene.rate;
 if(scene.law["sc2_fault_axes"].as<std::vector<int>>()!=std::vector<int>({0,1,2,3,4,5}) || scene.law["sc2_theta_domain"].as<std::string>()!="unbounded-constant-single-axis" || scene.law["sc2_uwb_domain"].as<std::string>()!="all-healthy-raw-source-rw-v1" || scene.law["sc2_boundary"].as<std::string>()!="original-initial-observation-only" || scene.law["sc2_max_bridge_s"].as<double>()!=2.5)throw std::runtime_error("SC2 registered scope fields refused");
 Sc2ImuScope scope;scope.id=scene.law["sc2_scope_id"].as<std::string>();scope.durations_ms=scene.law["sc2_durations_ms"].as<std::vector<int>>();scope.acceleration_bound=scene.law["sc2_acceleration_bound_mps2"].as<double>();scope.angular_rate_bound=scene.law["sc2_angular_rate_bound_radps"].as<double>();scope.candidate_capacity=scene.law["sc2_candidate_capacity"].as<size_t>();
 Sc2RecoveryCoordinator recovery(scene.initial_observation,law,scope);IncrementalUwbImuEstimator estimator(scene.cfg,scene.lever);estimator.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);estimator.ingestImu(scene.imu[0]);recovery.append(scene.imu[0]);RawPrefixHistory prefix;
 bool fde=argc<6 || std::string(argv[5])=="on";int enableEpoch=std::getenv("SC2_FDE_ENABLE_EPOCH")?std::stoi(std::getenv("SC2_FDE_ENABLE_EPOCH")):1;
 std::ofstream out(argv[2]);if(!out)throw std::runtime_error("SC2 output unavailable");out<<std::setprecision(17);
 out<<"epoch,scene,fde,alarm,current_violation,first_alarm_ns,possible,nominal_possible,status,candidate_status,rebuilds,candidate_ms,history_purge,first_dependency,current_bridge,imu_rejoined,removed_groups,updates,transaction_id,graph_version,timestamp_ns,qualified,bound_method,bound_scope,hpl,vpl,risk_upper,tag_x,tag_y,tag_z,velocity_x,velocity_y,velocity_z,qw,qx,qy,qz,horizontal_pe,vertical_pe,velocity_error,rotation_error,raw_bound_status,range_contraction_ms,full_ms,on_time,formal_eligible,publication_protected,application_available,retained_current_ids,bias_box_x,bias_box_gx,prefix_groups,initial_source_valid,healthy_imu_certificate_valid,source_init_ms,old_local_rot,old_local_pos,old_local_vel,old_local_ba,old_local_bg\n";
 double sourceMs=ms(sourceStart,Clock::now());
 for(int epoch=1;epoch<=scene.epochs;++epoch){auto start=Clock::now();for(int i=(epoch-1)*10+1;i<=epoch*10;++i){estimator.ingestImu(scene.imu[i]);recovery.append(scene.imu[i]);}
  auto tx=estimator.prepareEpoch(scene.uwb[epoch-1],EpochPreparationOptions{true,true,true,true});auto candidateStart=Clock::now();auto assessment=recovery.prepare(tx,fde && epoch>=enableEpoch,scene.cfg.bridge.generic);double candidateMs=ms(candidateStart,Clock::now());
  if(!assessment.legal){estimator.discardEpoch(std::move(tx),DiscardReason{FdeStatus::ModelInvalid,assessment.status+":"+assessment.reason,false});throw std::runtime_error("SC2 attempt aborted at"+std::to_string(epoch)+":"+assessment.status+":"+assessment.reason);}
  prefix.observe(tx,scene.cfg.integrity_window.epochs);auto plan=sc2CommitPlan(tx,assessment);auto retained=tx;auto before=estimator.backendUpdateCount();auto receipt=estimator.commitEpoch(std::move(tx),plan);auto state=estimator.currentState();if(receipt.backend_updates!=1 || estimator.backendUpdateCount()!=before+1)throw std::runtime_error("SC2 unique backend commit failed");
  recovery.committed(assessment,retained,receipt);prefix.committed(retained,plan);auto rangeStart=Clock::now();auto bound=recovery.issue(scene.range_certificates[epoch-1],retained,receipt,state,scene.lever);double rangeMs=ms(rangeStart,Clock::now());auto envelope=recovery.envelope(epoch,scene.lever);auto monitor=recovery.monitor();Eigen::Vector3d tag=state.position_world_m+state.q_world_body*scene.lever;
  // Truth/injection metadata used ONLY here, after all online actions/output.
  double horizontal=NAN,vertical=NAN,velocity=NAN,rotationError=NAN;
  if(!std::getenv("SC2_REDACT_SCORING")){auto truth=scene.truth(epoch*.05);Eigen::Vector3d trueTag=truth.position_world_m+truth.q_world_body*scene.lever;horizontal=(tag-trueTag).head<2>().norm();vertical=std::abs((tag-trueTag).z());velocity=(state.velocity_world_mps-truth.velocity_world_mps).norm();rotationError=Eigen::AngleAxisd(state.q_world_body.conjugate()*truth.q_world_body).angle();}
  std::ostringstream ids;for(size_t i=0;i<retained.uwb_batch.measurements.size();++i){if(i)ids<<'|';ids<<retained.uwb_batch.measurements[i].id.value();}
  double total=ms(start,Clock::now());
  out<<epoch<<','<<(std::getenv("SC2_REDACT_SCORING")?"REDACTED":kind)<<','<<fde<<','<<monitor.alarm<<','<<monitor.current_violation<<','<<monitor.first_alarm.value()<<','<<monitor.possible<<','<<monitor.nominal_possible<<','<<assessment.status<<','<<assessment.candidate.status<<','<<assessment.candidate.rebuilds<<','<<candidateMs<<','<<assessment.history_purge<<','<<assessment.first_dependency<<','<<assessment.current_bridge<<','<<(assessment.status=="IMU_REJOINED_ORIGINAL_BIAS_REINTEGRATION")<<','<<plan.groups_to_remove.size()<<','<<receipt.backend_updates<<','<<receipt.transaction_id.value()<<','<<receipt.graph_version<<','<<state.timestamp.value()<<','<<bound.valid<<','<<bound.source<<','<<scope.id<<','<<bound.hpl<<','<<bound.pl.z()<<','<<bound.event_risk;
  for(auto vector:{tag,state.velocity_world_mps})for(int j=0;j<3;++j)out<<','<<vector(j);
  out<<','<<state.q_world_body.w()<<','<<state.q_world_body.x()<<','<<state.q_world_body.y()<<','<<state.q_world_body.z()<<','<<horizontal<<','<<vertical<<','<<velocity<<','<<rotationError<<','<<bound.status<<','<<rangeMs<<','<<total<<','<<(total<=50)<<",0,0,0,"<<ids.str()<<','<<(envelope.bias_high(0)-envelope.bias_low(0))<<','<<(envelope.bias_high(3)-envelope.bias_low(3))<<','<<prefix.groups.size()<<",1,"<<scene.numeric_certificate.valid<<','<<sourceMs;
  for(int j=0;j<5;++j)out<<','<<assessment.candidate.cumulative(j);out<<'\n';
  if(epoch==15 || epoch==25 || epoch==26 || epoch==30)std::cout<<"SC2 "<<kind<<" epoch="<<epoch<<" status="<<assessment.status<<" alarm="<<monitor.alarm<<" possible="<<monitor.possible<<" qualified="<<bound.valid<<" H/V="<<bound.hpl<<"/"<<bound.pl.z()<<" error="<<horizontal<<"/"<<vertical<<" candidate="<<assessment.candidate.status<<" full_ms="<<total<<'\n';
 }
 return 0;
}
}
