#include "scenario_material.hpp"
#include "sc2_imu_scope.hpp"
#include "sc2_imu_bridge.hpp"
#include "sc2_imu_coordinator.hpp"
#include <iostream>
#include <gtsam/linear/NoiseModel.h>
#include <boost/multiprecision/cpp_dec_float.hpp>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
RawInclusionParameters parameters(const NoisyMotionScenario& s){RawInclusionParameters p;p.numeric=s.numeric_certificate;p.initial_sigmas=s.cfg.realtime.prior_sigmas;p.gravity=s.cfg.imu.gravity_mps2;return p;}

void responseOracle(const NoisyMotionScenario& scenario){
 IncrementalUwbImuEstimator backend(scenario.cfg,scenario.lever);backend.initialize(scenario.initial_observation,scenario.cfg.realtime.prior_sigmas);backend.ingestImu(scenario.imu[0]);
 for(int e=1;e<=2;++e){for(int i=(e-1)*10+1;i<=e*10;++i)backend.ingestImu(scenario.imu[i]);auto tx=backend.prepareEpoch(scenario.uwb[e-1]);
  for(int axis=0;axis<6;++axis){RawImuEpisode episode{"response-oracle",TimestampNs(5000000),TimestampNs(55000000),axis};LinearizedFactorBlock block;block.whitener=gtsam::noiseModel::Gaussian::Covariance(tx.preintegration->preintMeasCov())->R();auto response=episodeResponse(tx,block,episode);
   auto independent=[&](double amplitude){auto samples=tx.raw_imu_slice;for(auto& sample:samples)if(!(sample.timestamp<episode.begin) && sample.timestamp<episode.end){if(axis<3)sample.specific_force_mps2(axis)+=amplitude;else sample.angular_velocity_radps(axis-3)+=amplitude;}auto pim=*tx.preintegration;pim.resetIntegration();for(size_t i=1;i<samples.size();++i)pim.integrateMeasurement(.5*(samples[i-1].specific_force_mps2+samples[i].specific_force_mps2),.5*(samples[i-1].angular_velocity_radps+samples[i].angular_velocity_radps),.005);return episodeFactorError(tx,pim);};
   Eigen::VectorXd derivative=-(independent(2e-5)-independent(-2e-5))/(4e-5);require((derivative-response.raw).norm()<2e-6*std::max(1e-12,derivative.norm()),"matched-RW six-axis independent perturbation oracle");require(response.affected_samples==size_t(e==1?10:1) && response.affected_trapezoids==size_t(e==1?10:1),"raw response shared endpoint");
  }
  auto plan=EpochCommitPlan::nominalPlan(tx);backend.commitEpoch(std::move(tx),plan);
 }
}

void bridgeChecks(){
 for(const auto* kind:{"SC2_A","SC2_G","SC2_S"}){
  NoisyMotionScenario scenario("config/sc2_imu.yaml",kind,20261010,30);require(!scenario.numeric_certificate.valid && scenario.imu_fault_numeric_certificate.valid,"fault addition numerical certificate is not healthy-IMU attestation");
  auto law=parameters(scenario);law.numeric=scenario.imu_fault_numeric_certificate;Sc2IndependentBridge bridge(scenario.initial_observation,law);Sc2ImuMonitor monitor(scenario.initial_observation,law);
  for(const auto& sample:scenario.imu)monitor.append(sample);
  require(monitor.assessment().valid && (std::string(kind)=="SC2_S" || monitor.assessment().alarm),"unlabelled matched-law monitor");
  for(int epoch:{15,25,26,30}){auto envelope=bridge.envelope(epoch,scenario.lever);auto truth=scenario.truth(epoch*.05);auto tag=truth.position_world_m+truth.q_world_body*scenario.lever;
   require(envelope.valid && (truth.position_world_m.array()>=envelope.position_low.array()).all() && (truth.position_world_m.array()<=envelope.position_high.array()).all(),"fault-independent body position bridge");
   require((truth.velocity_world_mps.array()>=envelope.velocity_low.array()).all() && (truth.velocity_world_mps.array()<=envelope.velocity_high.array()).all(),"fault-independent velocity bridge");
   Eigen::Matrix<double,6,1> bias;bias<<truth.accel_bias_mps2,truth.gyro_bias_radps;require((bias.array()>=envelope.bias_low.array()).all() && (bias.array()<=envelope.bias_high.array()).all() && (envelope.bias_high-envelope.bias_low).minCoeff()>0,"nonzero initial/RW bias enclosure");
   auto rotation=truth.q_world_body.toRotationMatrix();require((rotation.array()>=envelope.rotation_low.array()).all() && (rotation.array()<=envelope.rotation_high.array()).all(),"rotation bridge");require((tag.array()>=envelope.tag_low.array()).all() && (tag.array()<=envelope.tag_high.array()).all(),"lever tag bridge");
  }
  require(!bridge.envelope(51,scenario.lever).valid,"bridge horizon expires");
  auto invalidScope=Sc2ImuScope{};invalidScope.acceleration_bound=.001;bool rejected=false;try{Sc2IndependentBridge invalid(scenario.initial_observation,law,invalidScope);}catch(const std::exception&){rejected=true;}require(rejected,"physical motion domain cannot silently shrink");
 }
 // Analytic extremal double-integral and sample-grid RW at t=2.5, with no
 // measured IMU values or truth trajectory used by the bridge kernel.
 NavigationState observation;observation.position_world_m.setZero();observation.velocity_world_mps.setZero();RawInclusionParameters law;law.numeric.valid=true;law.initial_sigmas.setConstant(.001);law.motion_amplitude.setZero();law.motion_frequency.setZero();law.motion_angular_rate=0;Sc2IndependentBridge bridge(observation,law);
 auto envelope=bridge.envelope(50,Eigen::Vector3d::Zero());double t=2.5;
 using MP=boost::multiprecision::cpp_dec_float_100;MP positionMax=7*MP(.001)*sqrt(MP(3))+MP(t)*7*MP(.001)+MP(.04)*MP(t)*MP(t)/2;MP biasMax=7*(MP(.001)+500*MP(.001)*sqrt(MP(1)/200));
 require(MP(envelope.position_high(0))>=positionMax && MP(envelope.position_low(0))<=-positionMax && MP(envelope.bias_high(0))>=biasMax && MP(envelope.bias_low(0))<=-biasMax,"100-digit integral/RW endpoint oracle");
 for(double sign:{-1.,1.}){Eigen::Vector3d p=Eigen::Vector3d::Constant(sign*(7*.001+t*7*.001+.5*.04*t*t));require((p.array()>=envelope.position_low.array()).all() && (p.array()<=envelope.position_high.array()).all(),"analytic signed acceleration integral");double bias=sign*(7*.001+7*500*.001*std::sqrt(.005));require(bias>=envelope.bias_low(0) && bias<=envelope.bias_high(0),"RW signed boundary");}
 std::cout<<"SC2 independent full-state/RW/tag bridge and fault-source numeric checks PASS\n";
}

void issueChecks(){
 NoisyMotionScenario scenario("config/sc2_imu.yaml","SC2_A",20261010,16);auto law=parameters(scenario);law.numeric=scenario.imu_fault_numeric_certificate;Sc2IndependentBridge bridge(scenario.initial_observation,law);Sc2ImuMonitor monitor(scenario.initial_observation,law);
 IncrementalUwbImuEstimator backend(scenario.cfg,scenario.lever);backend.initialize(scenario.initial_observation,scenario.cfg.realtime.prior_sigmas);backend.ingestImu(scenario.imu[0]);monitor.append(scenario.imu[0]);
 for(int epoch=1;epoch<=16;++epoch){for(int i=(epoch-1)*10+1;i<=epoch*10;++i){backend.ingestImu(scenario.imu[i]);monitor.append(scenario.imu[i]);}auto tx=backend.prepareEpoch(scenario.uwb[epoch-1]);auto retained=tx;auto receipt=backend.commitEpoch(std::move(tx),EpochCommitPlan::nominalPlan(retained));auto state=backend.currentState();
  if(epoch!=15 && epoch!=16)continue;
  auto out=bridge.issue(monitor,scenario.range_certificates[epoch-1],retained,receipt,state,scenario.lever);require(out.valid && out.considered==1 && out.event_risk==std::ldexp(2.*6421,-35),"new IMU-fault proof, healthy UWB, same event and actual FDE-off center");auto truth=scenario.truth(epoch*.05);Eigen::Vector3d trueTag=truth.position_world_m+truth.q_world_body*scenario.lever;require((trueTag.array()>=out.low.array()).all() && (trueTag.array()<=out.high.array()).all(),"polluted center does not discard truth");
  auto changed=retained;changed.raw_imu_slice.back().specific_force_mps2.x()+=1;require(!bridge.issue(monitor,scenario.range_certificates[epoch-1],changed,receipt,state,scenario.lever).valid,"raw prefix receipt mutation refused");
  auto wrongReceipt=receipt;wrongReceipt.backend_updates=2;require(!bridge.issue(monitor,scenario.range_certificates[epoch-1],retained,wrongReceipt,state,scenario.lever).valid,"multiple update receipt refused");
  auto wrongRange=scenario.range_certificates[epoch-1];wrongRange.frame="body";require(!bridge.issue(monitor,wrongRange,retained,receipt,state,scenario.lever).valid,"wrong frame evidence refused");
  bool rejected=false;try{Sc2IndependentBridge pollutedBoundary(state,law);}catch(const std::exception&){rejected=true;}require(rejected,"polluted posterior is not an initial boundary");
 }
 Sc2ImuMonitor inconsistent(scenario.initial_observation,law);for(int i=0;i<=141;++i){auto sample=scenario.imu[i];if(i==141)sample.angular_velocity_radps.y()+=1;inconsistent.append(sample);}require(inconsistent.assessment().alarm && inconsistent.assessment().possible==0 && inconsistent.assessment().status=="SCOPE_INCONSISTENT","two-axis explanation cannot silently qualify single-axis scope");
 std::cout<<"SC2 polluted-center receipt/prefix/frame/boundary and unrepresented two-axis checks PASS\n";
}

void nativeRecoveryChecks(){
 NoisyMotionScenario scenario("config/sc2_imu.yaml","SC2_A",20261010,26);auto law=parameters(scenario);law.numeric=scenario.imu_fault_numeric_certificate;
 Sc2RecoveryCoordinator recovery(scenario.initial_observation,law),shadow(scenario.initial_observation,law);
 IncrementalUwbImuEstimator backend(scenario.cfg,scenario.lever);backend.initialize(scenario.initial_observation,scenario.cfg.realtime.prior_sigmas);backend.ingestImu(scenario.imu[0]);recovery.append(scenario.imu[0]);shadow.append(scenario.imu[0]);
 for(int epoch=1;epoch<=26;++epoch){for(int i=(epoch-1)*10+1;i<=epoch*10;++i){backend.ingestImu(scenario.imu[i]);recovery.append(scenario.imu[i]);shadow.append(scenario.imu[i]);}
  auto tx=backend.prepareEpoch(scenario.uwb[epoch-1]);
  if(epoch==15){
   auto missing=tx;missing.history_recoverability=HistoryRecoverability::MissingProvenance;
   auto refusal=shadow.prepare(missing,true,scenario.cfg.bridge.generic);require(!refusal.legal && refusal.status=="BOUNDARY_OR_ACTIVE_PREFIX_MISSING","missing raw history cannot become a healthy marginal");
   auto noInitial=tx;auto values=std::make_shared<gtsam::Values>(*tx.frozen_values);values->erase(gtsam::Symbol('x',0));noInitial.frozen_values=values;
   refusal=shadow.prepare(noInitial,true,scenario.cfg.bridge.generic);require(!refusal.legal && refusal.status=="BOUNDARY_OR_ACTIVE_PREFIX_MISSING","missing original boundary cannot be replaced by posterior");
  }
  auto shadowTx=tx;auto assessment=recovery.prepare(tx,true,scenario.cfg.bridge.generic);auto other=shadow.prepare(shadowTx,true,scenario.cfg.bridge.generic);
  require(assessment.legal && other.legal && assessment.status==other.status && assessment.action.groups_to_remove==other.action.groups_to_remove && assessment.action.groups_to_add==other.action.groups_to_add && tx.cv_predicted_state.position_world_m==shadowTx.cv_predicted_state.position_world_m,"same-input candidate identity without any truth view");
  if(epoch>=15 && epoch<=25)require(assessment.current_bridge && assessment.candidate.rebuilds==2,"sustained complete bridge through last shared sample");if(epoch==26){require(!assessment.current_bridge && assessment.status=="IMU_REJOINED_ORIGINAL_BIAS_REINTEGRATION","support-contract IMU restoration");require(tx.preintegration->biasHat().accelerometer()==scenario.initial_observation.accel_bias_mps2 && tx.preintegration->biasHat().gyroscope()==scenario.initial_observation.gyro_bias_radps,"healthy rejoin cannot inherit polluted posterior biasHat");}
  auto plan=sc2CommitPlan(tx,assessment);auto retained=tx;auto receipt=backend.commitEpoch(std::move(tx),plan);recovery.committed(assessment,retained,receipt);shadow.committed(other,shadowTx,receipt);auto state=backend.currentState();auto bound=recovery.issue(scenario.range_certificates[epoch-1],retained,receipt,state,scenario.lever);require(bound.valid,"native recovery raw issuance");
  // Complete raw supports are removed from ACTUAL active slots, not merely
  // the candidate graph. Each old row has a native pose/bias replacement.
  for(auto group:assessment.action.groups_to_remove)if(group!=retained.imu_group.id)require(backend.factorLedger().activeSlots(group).empty(),"old contaminated/dependent slots remain active");
  if(assessment.current_bridge)require(receipt.group_to_slots.count(retained.imu_group.id.value())==0,"current suspect PIM really absent");else require(receipt.group_to_slots.count(retained.imu_group.id.value())==1,"restored IMU really committed");
 }
 std::cout<<"SC2 native full-support deletion/two rebuilds/shared-tail25/rejoin26 and same-data shadow PASS\n";
}
int main(){try{
 NoisyMotionScenario nominal("config/sc2_imu.yaml","U0",20261010,30);
 responseOracle(nominal);bridgeChecks();issueChecks();nativeRecoveryChecks();
 auto law=parameters(nominal);Sc2ImuMonitor healthy(nominal.initial_observation,law);
 for(auto sample:nominal.imu)healthy.append(sample);require(healthy.assessment().valid && !healthy.assessment().alarm,"nominal shared source prefix");
 for(int axis=0;axis<6;++axis){Sc2ImuMonitor monitor(nominal.initial_observation,law);double amplitude=axis<3?20.:1.;
  RawImuEpisode trueEpisode{"offline-only",TimestampNs(705000000),TimestampNs(1205000000),axis};
  auto closure=sc2SupportClosure(trueEpisode,30);require(closure.valid && closure.raw_samples.size()==100 && closure.direct_pim_epochs.front()==15 && closure.direct_pim_epochs.back()==25 && closure.reference_dependent_epochs.back()==30,"six-axis shared-tail/reference closure");
  for(auto sample:nominal.imu){if(!(sample.timestamp<trueEpisode.begin) && sample.timestamp<trueEpisode.end){if(axis<3)sample.specific_force_mps2(axis)+=amplitude;else sample.angular_velocity_radps(axis-3)+=amplitude;}monitor.append(sample);}
  auto result=monitor.assessment();require(result.valid && result.alarm && result.first_alarm==trueEpisode.begin,"actual raw six-axis alarm");
  bool retained=false;for(const auto& candidate:monitor.candidates())if(candidate.possible && candidate.episode.axis==axis && candidate.episode.begin==trueEpisode.begin && candidate.episode.end==trueEpisode.end){require(candidate.theta_low<=amplitude && candidate.theta_high>=amplitude,"single shared amplitude retained");retained=true;}
  require(retained,"true physical explanation never discarded");
 }
 // Zero additive fault is part of the unbounded family: absence of a raw
 // violation cannot erase its physical explanation or force an alarm.
 Sc2ImuMonitor cancelled(nominal.initial_observation,law);for(auto sample:nominal.imu)cancelled.append(sample);
 bool zeroRetained=false;for(const auto& c:cancelled.candidates())if(c.possible && c.episode.axis==0 && c.episode.begin==TimestampNs(705000000) && c.episode.end==TimestampNs(1205000000))zeroRetained=c.theta_low<=0 && c.theta_high>=0;
 require(zeroRetained && !cancelled.assessment().alarm,"cancelled/zero fault explanation retained without manufactured detection");
 RawImuEpisode shortEpisode{"short",TimestampNs(705000000),TimestampNs(755000000),0};auto shortClosure=sc2SupportClosure(shortEpisode,30);require(shortClosure.raw_samples.size()==10 && shortClosure.direct_pim_epochs==std::vector<size_t>({15,16}),"50ms endpoint closure");
 auto capacity=Sc2ImuScope{};capacity.candidate_capacity=600;bool rejected=false;try{Sc2ImuMonitor incomplete(nominal.initial_observation,law,capacity);}catch(const std::exception&){rejected=true;}require(rejected,"capacity cannot truncate fault scope");
 auto bad=nominal.imu[0];bad.id=MeasurementId(999);healthy.append(bad);require(!healthy.assessment().valid,"duplicate/stale prefix refused");
 std::cout<<"SC2 six-axis scope, shared-tail/dependent closure, raw alarm and shared-theta retention PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
