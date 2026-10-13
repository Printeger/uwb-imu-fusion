#include "rolling_boundary.hpp"
#include "scenario_material.hpp"
#include "rolling_recovery.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <sys/resource.h>
#include <tbb/global_control.h>
namespace uwb_imu_pl::research {
int runSc5Rolling(int argc,char** argv){
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 if(argc<7)throw std::invalid_argument("rolling requires CONFIG CSV SCENE rolling on|off PLAYED");
 using Clock=std::chrono::steady_clock;auto ms=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
 const auto cold=Clock::now();NoisyMotionScenario input(argv[1],argv[3],20261010,std::stoi(argv[6]),true);
 auto policy=resolveRollingPolicy(input.law,input.cfg);RollingSourceEvent event(policy.mission,policy.budget);
 if(!input.initial_law_valid || !input.physical_source_reference || !input.imu_fault_numeric_certificate.valid || input.causal_imu_certificates.size()!=size_t(input.epochs+1))throw std::invalid_argument("SC5_INITIAL_SOURCE_UNQUALIFIED");
 RawInclusionParameters law;law.rolling_source=true;law.coordinate_radius=event.q;law.horizon=policy.mission;law.coordinates=int(event.coordinates);law.risk_budget=policy.budget;
 // Fixed numeric-domain caps; computed future residues never choose today's radius.
 law.numeric.position=law.numeric.rotation_matrix=law.numeric.accel_sample=law.numeric.gyro_sample=1e-10;
 law.numeric.velocity.setConstant(1e-10);law.numeric.bias.setConstant(1e-10);law.numeric.bias_process.setConstant(1e-10);law.numeric.valid=true;
 law.initial_sigmas=input.cfg.realtime.prior_sigmas;law.accel_adc=input.law["raw_accel_sigma_mps2"].as<double>();law.gyro_adc=input.law["raw_gyro_sigma_radps"].as<double>();law.accel_rw=input.cfg.imu.accelerometer_bias_rw_sigma;law.gyro_rw=input.cfg.imu.gyroscope_bias_rw_sigma;law.gravity=input.cfg.imu.gravity_mps2;
 // Analytically verify the declared simulation domain, no actual truth draws.
 RollingInterval acceleration2(0);for(int j=0;j<3;++j)acceleration2+=square(abs(RollingInterval(input.amp(j)))*square(RollingInterval(input.freq(j))));
 if(sqrt(acceleration2).upper()>.04 || std::abs(input.rate)>.2)throw std::invalid_argument("SC5_PHYSICAL_DOMAIN_UNQUALIFIED");
 Sc2ImuScope scope;scope.id="sc5-single-imu-episode-rw-v1";scope.horizon=policy.mission;scope.candidate_capacity=12ULL*policy.mission+1;
 Sc2ImuMonitor monitor(input.initial_observation,law,scope);
 RollingIntegrity integrity(policy,input.initial_observation,law.initial_sigmas,law.accel_rw,law.gyro_rw,input.lever);
 IncrementalUwbImuEstimator backend(input.cfg,input.lever);backend.initialize(input.initial_observation,input.cfg.realtime.prior_sigmas);
 backend.ingestImu(input.imu.front());monitor.append(input.imu.front());
 std::deque<ImuMeasurement> replay{input.imu.front()};
 std::deque<RollingRawState> capsules{integrity.certificate().separator};
 RollingRecovery recovery(input.initial_observation,policy);
 const bool fde=std::string(argv[5])=="on";
 const int enable_epoch=argc>7?std::stoi(argv[7]):1;
 std::ofstream out(argv[2]);if(!out)throw std::runtime_error("SC5_TRACE_UNAVAILABLE");out<<std::setprecision(17);
 out<<"epoch,qualified,bound_status,functional_status,recovery_status,updates,version,x0_present,fgo_nodes,integrity_material,boundary_epoch,generation,parent,marginal_keys,numerical_priors,raw_read_first,raw_read_last,replay_first,replay_last,replay_samples,source_prefix,task_horizon,risk_upper,q,logical_faults,stored_imu_modes,not_started,active,ended_residual,excluded,shared_adc_index,hpl,vpl,horizontal_pe,vertical_pe,prepare_ms,proof_ms,commit_ms,core_ms,full_ms,on_time,rss_kib,application_status,formal_eligible,publication_protected,tag_x,tag_y,tag_z,vx,vy,vz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz,imu_alarm,first_alarm_ns,imu_bridge,imu_rejoined,excluded_anchor,actions,numeric_evaluated,rebuilds,boundary_repair,history_marginalized,removed_priors,removed_groups,first_dependency,candidate_ms,capsules\n";
 const double cold_ms=ms(cold,Clock::now());size_t valid=0,commits=0,x0_exit=0,marginal=0;
 std::cout<<"SC5 startup requested/resolved "<<policy.fgo<<'/'<<policy.integrity<<" logical-lag="<<policy.fgo-1<<" mission="<<policy.mission<<" played="<<input.epochs<<" replay="<<policy.replay<<" q="<<event.q<<" sources="<<event.coordinates<<" event="<<event.id<<" cold_ms="<<cold_ms<<'\n';
 for(int k=1;k<=input.epochs;++k){auto start=Clock::now();
  if(k>policy.mission){out<<k<<",0,TASK_HORIZON_EXPIRED,NOT_ATTEMPTED,NOT_ATTEMPTED,0\n";continue;}
  if(!input.causal_imu_certificates[k].valid)throw std::invalid_argument("SC5_ARRIVED_SOURCE_NUMERIC_UNQUALIFIED:"+std::to_string(k));
  for(int i=(k-1)*10+1;i<=k*10;++i){backend.ingestImu(input.imu[i]);monitor.append(input.imu[i]);replay.push_back(input.imu[i]);}
  while(replay.size()>size_t(10*policy.replay+1))replay.pop_front();
  auto prep=Clock::now();auto tx=backend.prepareEpoch(input.uwb[k-1],{true,true,true,true});auto prepared=Clock::now();
  auto action=recovery.prepare(tx,monitor,replay,capsules,input.cfg,fde && k>=enable_epoch);
  auto plan=action.plan;
  auto proposal=integrity.propose(tx,plan,input.range_certificates[k-1],monitor);
  auto capsule=proposal.current;
  const size_t read_first=proposal.raw_read_first,read_last=proposal.raw_read_last;
  auto proof_end=Clock::now();const auto before=backend.backendUpdateCount();
  auto retained=tx;
  auto receipt=action.current_bridge?backend.commitEpochRollingResearchV1(std::move(tx),plan):backend.commitEpoch(std::move(tx),plan);auto state=backend.currentState();auto commit_end=Clock::now();
  if(receipt.backend_updates!=1 || backend.backendUpdateCount()!=before+1)throw std::runtime_error("SC5_SINGLE_COMMIT_FAILED");
  auto keys=backend.retainedKeysV1();auto bound=integrity.finalize(std::move(proposal),receipt,keys,state);
  recovery.committed(action,retained,receipt,backend.factorLedger());
  capsules.push_back(std::move(capsule));while(capsules.size()>size_t(policy.replay+1))capsules.pop_front();
  const auto& cert=integrity.certificate();while(replay.size()>size_t(10*policy.replay+1))replay.pop_front();
  const bool x0=std::find(keys.begin(),keys.end(),gtsam::Symbol('x',0))!=keys.end();if(!x0 && !x0_exit)x0_exit=k;
  marginal+=receipt.marginalized_keys.size();++commits;valid+=bound.valid;
  Eigen::Vector3d tag=state.position_world_m+state.q_world_body*input.lever;
  // Scoring only after irreversible native selection, proof and commit.
  auto truth=input.truth(k*.05);auto error=tag-(truth.position_world_m+truth.q_world_body*input.lever);
  rusage usage{};getrusage(RUSAGE_SELF,&usage);
  auto done=Clock::now();double core=ms(start,done);
  std::ostringstream packet;packet<<std::setprecision(17);
  packet<<k<<','<<bound.valid<<','<<bound.status<<','<<action.status<<','<<action.recovery<<','<<receipt.backend_updates<<','<<receipt.graph_version<<','<<x0<<','<<keys.size()/3<<','<<integrity.explicitSize()<<','<<cert.separator.epoch<<','<<cert.generation<<','<<cert.parent<<','<<receipt.marginalized_keys.size()<<','<<receipt.boundary_factor_slots.size()<<','<<read_first<<','<<read_last<<','<<replay.front().id.value()<<','<<replay.back().id.value()<<','<<replay.size()<<','<<cert.source_prefix<<','<<policy.mission<<','<<bound.event_risk<<','<<event.q<<','<<event.logical_faults<<','<<monitor.candidates().size()<<','<<cert.fault_not_started<<','<<cert.fault_active<<','<<cert.fault_ended_residual<<','<<cert.fault_excluded<<','<<cert.shared_adc_index<<','<<bound.hpl<<','<<bound.pl.z()<<','<<error.head<2>().norm()<<','<<std::abs(error.z())<<','<<ms(prep,prepared)<<','<<ms(prepared,proof_end)+ms(commit_end,done)<<','<<ms(proof_end,commit_end)<<','<<core;
  for(const auto& v:std::vector<Eigen::Vector3d>{tag,state.velocity_world_mps})for(int j=0;j<3;++j)packet<<','<<v(j);
  packet<<','<<state.q_world_body.w()<<','<<state.q_world_body.x()<<','<<state.q_world_body.y()<<','<<state.q_world_body.z();
  for(const auto& v:std::vector<Eigen::Vector3d>{state.accel_bias_mps2,state.gyro_bias_radps})for(int j=0;j<3;++j)packet<<','<<v(j);
  packet<<','<<monitor.assessment().alarm<<','<<monitor.assessment().first_alarm.value()<<','<<action.current_bridge<<','<<action.rejoined<<','<<action.excluded<<','<<action.actions<<','<<action.numeric_evaluated<<','<<action.rebuilds<<','<<action.boundary_repair<<','<<action.history_marginalized<<','<<action.removed_prior_slots.size()<<','<<plan.groups_to_remove.size()<<','<<action.first_dependency<<','<<action.candidate_ms<<','<<capsules.size();
  // Delivery includes packet serialization; diagnostics file I/O is separately timed by runner.
  const double full=ms(start,Clock::now());auto text=packet.str();
  // Core packet has states after core_ms: splice full fields before states.
  const size_t prefix_commas=39;size_t at=0;for(size_t j=0;j<prefix_commas;++j)at=text.find(',',at)+1;
  text.insert(at,std::to_string(full)+","+std::to_string(full<=50)+","+std::to_string(usage.ru_maxrss)+",APPLICATION_AL_UNQUALIFIED,0,0,");out<<text<<'\n';
  if(k==30 || k==51 || k==90 || k==150 || k==200)std::cout<<"SC5 epoch="<<k<<" raw="<<bound.valid<<" x0="<<x0<<" separator="<<cert.separator.epoch<<" parent="<<cert.parent<<" raw_first="<<read_first<<" H/V="<<bound.hpl<<'/'<<bound.pl.z()<<" full_ms="<<full<<'\n';
 }
 std::cout<<"SC5 denominator scheduled="<<input.epochs<<" commits="<<commits<<" qualified="<<valid<<" x0_exit="<<x0_exit<<" actual_marginal_keys="<<marginal<<" expired="<<std::max(0,input.epochs-policy.mission)<<'\n';return 0;
}
}
