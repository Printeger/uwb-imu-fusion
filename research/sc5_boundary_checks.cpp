#include "rolling_boundary.hpp"
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
  auto bad=input.range_certificates[0];bad.source_law_id="unregistered";rejected([&]{proof.propose(tx,plan,bad,monitor);});
  auto subset=input.range_certificates[0];subset.batch.measurements.pop_back();rejected([&]{proof.propose(tx,plan,subset,monitor);});
  auto future=tx;future.proposed_epoch=2;rejected([&]{proof.propose(future,plan,input.range_certificates[1],monitor);});
  check(backend.backendUpdateCount()==1,"proposal mutated backend");
  auto proposal=proof.propose(tx,plan,input.range_certificates[0],monitor);auto bad_proposal=proposal;
  auto r=backend.commitEpoch(std::move(tx),plan);auto state=backend.currentState();
  auto bound=proof.finalize(std::move(proposal),r,backend.retainedKeysV1(),state);check(bound.valid,"valid step refused");
  outputs[variant]=bound.pl;certs[variant]=proof.certificate();
  rejected([&]{proof.finalize(std::move(bad_proposal),r,backend.retainedKeysV1(),state);});
  check(backend.backendUpdateCount()==2,"post-commit proof failure caused second update");
 }
 check(outputs[0]==outputs[1] && certs[0].source_prefix==certs[1].source_prefix,"future leaked into current proof");
 for(int j=0;j<3;++j)check(certs[0].separator.p[j].lower()==certs[1].separator.p[j].lower() && certs[0].separator.p[j].upper()==certs[1].separator.p[j].upper(),"future changed separator");
 std::cout<<"W3 boundary PASS2 native commits; sealed full batch/scope/future/old-proposal rejection; no second update; future numeric admission noninterference\n";return 0;
}
