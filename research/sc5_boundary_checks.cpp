#include "rolling_boundary.hpp"
#include "rolling_recovery.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
namespace {
void check(bool x,const char* s){if(!x)throw std::runtime_error(s);}
template<class F>void rejected(F f){bool bad=false;try{f();}catch(const std::exception&){bad=true;}check(bad,"expected safe rejection");}
RawInclusionParameters lawFor(const IntegrityConfig& cfg,const RollingSourceEvent& event){
 RawInclusionParameters l;l.rolling_source=true;l.coordinate_radius=event.q;l.horizon=event.mission;l.coordinates=event.coordinates;l.initial_sigmas=cfg.realtime.prior_sigmas;
 l.numeric.valid=true;l.numeric.position=l.numeric.rotation_matrix=l.numeric.accel_sample=l.numeric.gyro_sample=1e-10;l.numeric.bias.setConstant(1e-10);l.numeric.velocity.setConstant(1e-10);l.numeric.bias_process.setConstant(1e-10);
 l.accel_rw=cfg.imu.accelerometer_bias_rw_sigma;l.gyro_rw=cfg.imu.gyroscope_bias_rw_sigma;return l;
}
}
int rollingBoundaryChecks(){
 Eigen::Vector3d outputs[2];RollingBoundaryCertificateV1 certs[2];
 for(int variant=0;variant<2;++variant){
  NoisyMotionScenario input("config/sc5_rolling.yaml","SC3_U0",20261010,2);
  if(variant){input.imu.back().specific_force_mps2.setConstant(12345);input.uwb.back().measurements[0].range_m=999;input.causal_imu_certificates.back().valid=false;}
  auto p=resolveRollingPolicy(input.law,input.cfg);RollingSourceEvent event(p.mission,p.budget);auto law=lawFor(input.cfg,event);
  Sc2ImuScope scope;scope.id="sc5-single-imu-episode-rw-v1";scope.horizon=p.mission;scope.candidate_capacity=12ULL*p.mission+1;
  Sc2ImuMonitor monitor(input.initial_observation,law,scope);
  IncrementalUwbImuEstimator backend(input.cfg,input.lever);backend.initialize(input.initial_observation,input.cfg.realtime.prior_sigmas);
  RollingIntegrity proof(p,input.initial_observation,law.initial_sigmas,law.accel_rw,law.gyro_rw,input.lever);
  for(int i=0;i<=10;++i){backend.ingestImu(input.imu[i]);monitor.append(input.imu[i]);}
  auto tx=backend.prepareEpoch(input.uwb[0]);auto plan=EpochCommitPlan::nominalPlan(tx);
  Sc2ImuScope insufficient=scope;insufficient.candidate_capacity=1;
  rejected([&]{Sc2ImuMonitor missing(input.initial_observation,law,insufficient);});
  auto bad=input.range_certificates[0];bad.source_law_id="unregistered";rejected([&]{proof.propose(tx,plan,bad,monitor);});
  auto subset=input.range_certificates[0];subset.batch.measurements.pop_back();rejected([&]{proof.propose(tx,plan,subset,monitor);});
  auto future=tx;future.proposed_epoch=2;rejected([&]{proof.propose(future,plan,input.range_certificates[1],monitor);});
  check(backend.backendUpdateCount()==1,"proposal mutated backend");
  auto proposal=proof.propose(tx,plan,input.range_certificates[0],monitor);auto bad_proposal=proposal;
  RollingIntegrity tampered(p,input.initial_observation,law.initial_sigmas,law.accel_rw,law.gyro_rw,input.lever);
  auto changed=tampered.propose(tx,plan,input.range_certificates[0],monitor);
  changed.current.p[0]=RollingInterval(0);
  auto frozen=tx;
  auto r=backend.commitEpoch(std::move(tx),plan);auto state=backend.currentState();
  auto bound=proof.finalize(std::move(proposal),r,backend.retainedKeysV1(),state);check(bound.valid,"valid step refused");
  outputs[variant]=bound.pl;certs[variant]=proof.certificate();
  rejected([&]{tampered.finalize(std::move(changed),r,backend.retainedKeysV1(),state);});
  RollingRecovery recovery(input.initial_observation,p);RollingFdeAssessment fake;
  fake.current_bridge=true;fake.plan=plan;
  auto forged=r;forged.group_to_slots.erase(frozen.imu_group.id.value());
  forged.group_to_slots[frozen.generic_bridge_group.id.value()]={0};forged.group_to_slots[frozen.generic_bias_continuity_group.id.value()]={1};
  rejected([&]{recovery.committed(fake,frozen,forged,backend.factorLedger());});
  rejected([&]{proof.finalize(std::move(bad_proposal),r,backend.retainedKeysV1(),state);});
  check(backend.backendUpdateCount()==2,"post-commit proof failure caused second update");
 }
 check(outputs[0]==outputs[1] && certs[0].source_prefix==certs[1].source_prefix,"future leaked into current proof");
 for(int j=0;j<3;++j)check(certs[0].separator.p[j].lower()==certs[1].separator.p[j].lower() && certs[0].separator.p[j].upper()==certs[1].separator.p[j].upper(),"future changed separator");
 for(auto point:{CommitFaultPoint::BeforeBackendUpdate,CommitFaultPoint::AfterBackendUpdate}){
  NoisyMotionScenario input("config/sc5_rolling.yaml","SC3_U0",20261010,1);
  auto p=resolveRollingPolicy(input.law,input.cfg);RollingSourceEvent event(p.mission,p.budget);auto law=lawFor(input.cfg,event);
  Sc2ImuScope scope;scope.id="sc5-single-imu-episode-rw-v1";scope.horizon=p.mission;scope.candidate_capacity=12ULL*p.mission+1;
  Sc2ImuMonitor monitor(input.initial_observation,law,scope);
  IncrementalUwbImuEstimator backend(input.cfg,input.lever);backend.initialize(input.initial_observation,input.cfg.realtime.prior_sigmas);
  RollingIntegrity proof(p,input.initial_observation,law.initial_sigmas,law.accel_rw,law.gyro_rw,input.lever);
  for(int i=0;i<=10;++i){backend.ingestImu(input.imu[i]);monitor.append(input.imu[i]);}
  auto tx=backend.prepareEpoch(input.uwb[0]);auto plan=EpochCommitPlan::nominalPlan(tx);
  auto proposed=proof.propose(tx,plan,input.range_certificates[0],monitor);
  backend.setCommitFaultPointForTesting(point);
  if(point==CommitFaultPoint::BeforeBackendUpdate){
   rejected([&]{backend.commitEpoch(std::move(tx),plan);});
   check(backend.backendUpdateCount()==1 && proof.certificate().generation==0,"pre-update failure mutated state/proof");
   backend.discardEpoch(std::move(tx),DiscardReason{});
   auto retry=backend.prepareEpoch(input.uwb[0]);auto retry_plan=EpochCommitPlan::nominalPlan(retry);
   auto proposal=proof.propose(retry,retry_plan,input.range_certificates[0],monitor);
   auto receipt=backend.commitEpoch(std::move(retry),retry_plan);
   check(proof.finalize(std::move(proposal),receipt,backend.retainedKeysV1(),backend.currentState()).valid,"pre-update retry refused");
   check(backend.backendUpdateCount()==2 && proof.certificate().source_prefix==149 && proof.event().risk_upper==event.risk_upper,"retry double-charged sources/risk");
  }else{
   bool terminal=false;try{backend.commitEpoch(std::move(tx),plan);}catch(const CommitTerminalError& e){terminal=e.receipt().backend_updates==1 && e.receipt().backend_poisoned;}
   check(terminal && backend.backendUpdateCount()==2 && proof.certificate().generation==0,"post-update failure fabricated proof/rollback");
   rejected([&]{backend.commitEpoch(std::move(tx),plan);});check(backend.backendUpdateCount()==2,"poisoned backend updated twice");
  }
 }
 std::cout<<"SC5 boundary PASS3 successful native commits +1 injected terminal mutation; full batch/scope/future/old-proposal/set-tamper/wrong-FDE/resource rejection; pre-update retry unique source event; no second post-update commit; future noninterference\n";return 0;
}
