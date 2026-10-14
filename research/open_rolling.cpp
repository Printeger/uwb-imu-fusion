#include "rolling_boundary.hpp"
#include "rolling_recovery.hpp"
#include "open_fault_pool.hpp"
#include "open_delivery_gate.hpp"
#include <tbb/global_control.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <unistd.h>
namespace uwb_imu_pl::research {
int runOpenRolling(int argc,char** argv){
 if(argc<7)throw std::invalid_argument("open-rolling CONFIG CSV SCENE on|off PLAYED");
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 using Clock=std::chrono::steady_clock;auto ms=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
 auto cold=Clock::now();NoisyMotionScenario input(argv[1],argv[3],20261010,std::stoi(argv[6]));
 auto policy=resolveRollingPolicy(input.law,input.cfg);if(!policy.open)throw std::invalid_argument("FINITE_OPEN_MODE_MIX");
 TimeUniformEvent event(policy.budget);
 OpenDeliveryGate delivery(input.law["rolling_integrity"]["queue_capacity"].as<size_t>());
 const double deadline=input.law["rolling_integrity"]["full_deadline_ms"].as<double>();
 if(!std::isfinite(deadline) || deadline<=0)throw std::invalid_argument("DEADLINE_INVALID");
 if(!input.initial_law_valid || !input.physical_source_reference || !input.imu_fault_numeric_certificate.valid)throw std::invalid_argument("INITIAL_SOURCE_UNQUALIFIED");
 RollingInterval a2(0);for(int j=0;j<3;++j)a2+=square(abs(RollingInterval(input.amp(j)))*square(RollingInterval(input.freq(j))));
 if(sqrt(a2).upper()>.04 || std::abs(input.rate)>.2)throw std::invalid_argument("OPEN_PHYSICAL_DOMAIN_UNQUALIFIED");
 RawInclusionParameters law;law.initial_sigmas=input.cfg.realtime.prior_sigmas;law.numeric.valid=true;
 law.accel_adc=input.law["raw_accel_sigma_mps2"].as<double>();law.gyro_adc=input.law["raw_gyro_sigma_radps"].as<double>();law.accel_rw=input.cfg.imu.accelerometer_bias_rw_sigma;law.gyro_rw=input.cfg.imu.gyroscope_bias_rw_sigma;law.gravity=input.cfg.imu.gravity_mps2;
 RollingIntegrity proof(policy,input.initial_observation,law.initial_sigmas,law.accel_rw,law.gyro_rw,input.lever);
 OpenFaultPool faults(input.initial_observation,law,policy.budget,policy.max_fault_records);
 RollingRecovery recovery(input.initial_observation,policy);
 IncrementalUwbImuEstimator backend(input.cfg,input.lever);backend.initialize(input.initial_observation,law.initial_sigmas);
 backend.ingestImu(input.imu.front());faults.append(input.imu.front());
 std::deque<ImuMeasurement> replay{input.imu.front()};std::deque<RollingRawState> capsules{proof.certificate().separator};
 std::ofstream out(argv[2]);if(!out)throw std::runtime_error("OUTPUT_UNAVAILABLE");out<<std::setprecision(17);
 out<<"epoch,qualified,current_protected,status,recovery,updates,version,generation,parent,boundary,x0,source_prefix,event,risk_upper,q,q_shared_first,alerts,active_imu,ended_imu,excluded_imu,births,bridge,rejoined,excluded_anchor,prior_repaired,removed_priors,removed_groups,first_dependency,replay_samples,capsules,explicit_batches,proof_uwb_first,imu_first,hpl,vpl,width_x,width_y,width_z,horizontal_pe,vertical_pe,bias_width,core_ms,full_ms,core_on_time,on_time,resident_kib,slots,live_factors,ledger_entries,tag_x,tag_y,tag_z,vx,vy,vz,qw,qx,qy,qz,formal_eligible,publication_protected,application_status\n";
 double init_ms=ms(cold,Clock::now());size_t committed=0,qualified=0,repairs=0,rejoins=0;double max_full=0,csv_ms=0;
 std::cout<<"OPEN startup mission=NONE windows="<<policy.fgo<<'/'<<policy.integrity<<" replay="<<policy.replay<<" event="<<event.id()<<" init_ms="<<init_ms<<'\n';
 for(int k=1;k<=input.epochs;++k){auto start=Clock::now();
  // Allocation is evaluated before the new observed samples enter a consumer.
  auto allocation=event.block(k);
  delivery.enqueue(k);
  if(argc>7 && std::string(argv[7])=="cancel:"+std::to_string(k))delivery.cancel(k);
  if(argc>7 && std::string(argv[7])=="overload:"+std::to_string(k)){for(size_t n=0;n<100;++n)if(!delivery.enqueue(k+n+1))break;}
  if(!delivery.start(k)){out<<k<<",0,0,SOURCE_GAP_CANCEL_OR_OVERLOAD,NOT_ATTEMPTED,0\n";continue;}
  if(!input.causal_imu_certificates.at(k).valid)throw std::runtime_error("ARRIVED_NUMERIC_DOMAIN_UNQUALIFIED epoch="+std::to_string(k));
  for(int i=(k-1)*10+1;i<=k*10;++i){backend.ingestImu(input.imu[i]);faults.append(input.imu[i]);replay.push_back(input.imu[i]);}
  while(replay.size()>size_t(10*policy.replay+1))replay.pop_front();
  auto tx=backend.prepareEpoch(input.uwb[k-1]);auto view=faults.view();
  auto action=recovery.prepare(tx,view,replay,capsules,input.cfg,std::string(argv[5])=="on");
  auto proposal=proof.propose(tx,action.plan,input.range_certificates[k-1],view);auto capsule=proposal.current;
  faults.appendRanges(tx.uwb_batch,capsule,input.lever);
  size_t first=(proposal.raw_read_first-1)/10+1;auto retained=tx;auto count=backend.backendUpdateCount();
  auto receipt=action.current_bridge?backend.commitEpochRollingResearchV1(std::move(tx),action.plan):backend.commitEpoch(std::move(tx),action.plan);
  auto state=backend.currentState();auto keys=backend.retainedKeysV1();
  if(receipt.backend_updates!=1 || backend.backendUpdateCount()!=count+1)throw std::runtime_error("OPEN_SINGLE_COMMIT_FAILED");
  auto bound=proof.finalize(std::move(proposal),receipt,keys,state);
  try{recovery.committed(action,retained,receipt,backend.factorLedger());}catch(...){proof.failAfterCommit();throw;}
  retained=EpochTransaction{};proposal=RollingBoundaryProposalV1{};action.removed_boundary.clear();
  capsules.push_back(capsule);while(capsules.size()>size_t(policy.replay+1))capsules.pop_front();
  auto& cert=proof.certificate();auto audit=backend.onlineMaterialAuditV1();size_t virt=0,res=0;std::ifstream("/proc/self/statm")>>virt>>res;
  Eigen::Vector3d tag=state.position_world_m+state.q_world_body*input.lever;
  // Immutable online decisions completed before scoring; no labels/GT feed back.
  auto truth=input.truth(k*.05);Eigen::Vector3d pe=tag-(truth.position_world_m+truth.q_world_body*input.lever);
  bool x0=std::find(keys.begin(),keys.end(),gtsam::Symbol('x',0))!=keys.end();double core=ms(start,Clock::now());
  std::ostringstream row;row<<std::setprecision(17);
  row<<k<<','<<bound.valid<<','<<"CURRENT_GATE"<<','<<bound.status<<','<<action.recovery<<','<<receipt.backend_updates<<','<<receipt.graph_version<<','<<cert.generation<<','<<cert.parent<<','<<cert.separator.epoch<<','<<x0<<','<<cert.source_prefix<<','<<event.id()<<','<<bound.event_risk<<','<<allocation.q<<','<<event.adcRadius((k-1)*10)<<','<<faults.records().size()<<','<<faults.activeSize()<<','<<faults.ended()<<','<<view.excluded<<','<<faults.births()<<','<<action.current_bridge<<','<<action.rejoined<<','<<action.excluded<<','<<action.boundary_repair<<','<<action.removed_prior_slots.size()<<','<<action.plan.groups_to_remove.size()<<','<<action.first_dependency<<','<<replay.size()<<','<<capsules.size()<<','<<proof.explicitSize()<<','<<first<<','<<(k-1)*10+1<<','<<bound.hpl<<','<<bound.pl.z();
  for(int j=0;j<3;++j)row<<','<<bound.high(j)-bound.low(j);
  row<<','<<pe.head<2>().norm()<<','<<std::abs(pe.z())<<','<<capsule.bias[0].upper()-capsule.bias[0].lower()<<','<<core<<",FULL_GATE,"<<(core<=40)<<",TIME_GATE,"<<res*size_t(sysconf(_SC_PAGESIZE))/1024<<','<<audit.factor_slots<<','<<audit.active_factors<<','<<audit.ledger_entries;
  for(const auto& v:std::vector<Eigen::Vector3d>{tag,state.velocity_world_mps})for(int j=0;j<3;++j)row<<','<<v(j);
  row<<','<<state.q_world_body.w()<<','<<state.q_world_body.x()<<','<<state.q_world_body.y()<<','<<state.q_world_body.z()<<",0,0,APPLICATION_AL_UNQUALIFIED";
  auto packet=row.str();double full=ms(start,Clock::now());max_full=std::max(max_full,full);
  auto replace=[&](const std::string& marker,const std::string& value){auto at=packet.find(marker);if(at==std::string::npos)throw std::logic_error("OUTPUT_SCHEMA");packet.replace(at,marker.size(),value);};
  replace("FULL_GATE",std::to_string(full));replace("TIME_GATE",std::to_string(full<=deadline));replace("CURRENT_GATE",std::to_string(delivery.deliver(k,state.timestamp.value(),receipt.graph_version,bound.valid,full,deadline)));
  auto io=Clock::now();out<<packet<<'\n';csv_ms+=ms(io,Clock::now());
  ++committed;qualified+=bound.valid;repairs+=action.boundary_repair;rejoins+=action.rejoined;
  if(k==51 || k==200 || k==500 || k==1000 || action.boundary_repair)std::cout<<"OPEN epoch="<<k<<" qualified="<<bound.valid<<" x0="<<x0<<" alerts="<<faults.records().size()<<" repair="<<action.boundary_repair<<" H/V="<<bound.hpl<<'/'<<bound.pl.z()<<" risk="<<bound.event_risk<<" q="<<allocation.q<<'\n';
 }
 std::cout<<"OPEN denominator scheduled="<<input.epochs<<" committed="<<committed<<" qualified="<<qualified<<" repairs="<<repairs<<" rejoin_epochs="<<rejoins<<" max_full_ms="<<max_full<<" csv_ms="<<csv_ms<<'\n';return 0;
}
}
