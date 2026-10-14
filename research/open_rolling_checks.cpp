#include "open_fault_pool.hpp"
#include "rolling_boundary.hpp"
#include "open_delivery_gate.hpp"
#include "rolling_recovery.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
template<class F>void refuse(F f){bool threw=false;try{f();}catch(const std::exception&){threw=true;}require(threw,"expected refusal");}
int main(){try{
 auto root=researchLaw("config/sc5_open.yaml");auto cfg=IntegrityConfigLoader::load(root["imu_base_config"].as<std::string>());auto policy=resolveRollingPolicy(root,cfg);
 require(policy.open && policy.mission==0 && policy.integrity==policy.fgo,"open parsed as finite");
 RawInclusionParameters law;law.numeric.valid=true;law.initial_sigmas=cfg.realtime.prior_sigmas;law.accel_rw=cfg.imu.accelerometer_bias_rw_sigma;law.gyro_rw=cfg.imu.gyroscope_bias_rw_sigma;
 OpenFaultPool monitor(initialState(),law,policy.budget);
 for(int i=0;i<=900;++i){auto m=sample(cfg,i);if((i>=101 && i<201) || (i>=501 && i<601))m.angular_velocity_radps.x()+=1;monitor.append(m);}
 require(monitor.records().size()==2 && monitor.records()[0].id!=monitor.records()[1].id && monitor.samples()==901,"repeat identities/history forgotten");
 require(monitor.ended()>0 && monitor.births()==12*90 && monitor.activeSize()<=120,"unbounded active bank or no residual summary");
 OpenFaultPool outside(initialState(),law,policy.budget);for(int i=0;i<11;++i)outside.append(sample(cfg,i));auto bad=sample(cfg,11);bad.specific_force_mps2.x()+=20;bad.angular_velocity_radps.x()+=1;refuse([&]{outside.append(bad);});
 require(!outside.view().state.valid && outside.view().state.status=="CONCURRENCY_OR_MODEL_OUTSIDE_SCOPE","two active axes assigned coverage");
 OpenFaultPool scarce(initialState(),law,policy.budget,1);refuse([&]{for(int i=0;i<=601;++i){auto m=sample(cfg,i);if((i>=101 && i<201)||(i>=501 && i<601))m.angular_velocity_radps.x()+=1;scarce.append(m);}});
 require(scarce.records().size()==1,"resource pruning silently discarded identity");
 OpenDeliveryGate queue(2);require(queue.enqueue(1) && queue.enqueue(2) && !queue.enqueue(3),"overload not latched");require(!queue.deliver(1,50000000,2,true,1,50),"overload emitted old protection");
 OpenDeliveryGate cancel(2);cancel.enqueue(1);cancel.cancel(1);require(!cancel.start(1) && !cancel.deliver(1,50000000,2,true,1,50),"cancel emitted old protection");
 OpenDeliveryGate stale(2);stale.enqueue(1);require(stale.start(1) && !stale.deliver(1,0,2,true,1,50) && !stale.deliver(1,50000000,2,true,60,50) && stale.deliver(1,50000000,2,true,1,50) && !stale.deliver(1,50000000,2,true,1,50),"time/version/repeated protection gate failed");
 NoisyMotionScenario input("config/sc5_open.yaml","OE_HEALTHY",20261010,2);auto p=resolveRollingPolicy(input.law,input.cfg);
 RollingIntegrity proof(p,input.initial_observation,input.cfg.realtime.prior_sigmas,law.accel_rw,law.gyro_rw,input.lever);
 OpenFaultPool arrived(input.initial_observation,law,p.budget);IncrementalUwbImuEstimator backend(input.cfg,input.lever);backend.initialize(input.initial_observation,law.initial_sigmas);
 for(int i=0;i<=10;++i){backend.ingestImu(input.imu[i]);arrived.append(input.imu[i]);}
 auto tx=backend.prepareEpoch(input.uwb[0]);auto plan=EpochCommitPlan::nominalPlan(tx);auto view=arrived.view();
 auto mixed=view;mixed.open=false;refuse([&]{proof.propose(tx,plan,input.range_certificates[0],mixed);});
 auto proposal=proof.propose(tx,plan,input.range_certificates[0],view);
 // Alter only future simulation material; current proposal is unaffected.
 input.imu.back().specific_force_mps2.setConstant(999);input.range_certificates.back().valid=false;
 auto unchanged=proof.propose(tx,plan,input.range_certificates[0],view);
 require(proposal.current.p[0].lower()==unchanged.current.p[0].lower(),"future affected current boundary");
 auto receipt=backend.commitEpoch(std::move(tx),plan);require(proof.finalize(std::move(proposal),receipt,backend.retainedKeysV1(),backend.currentState()).valid,"actual open boundary refused");
 refuse([&]{proof.finalize(std::move(unchanged),receipt,backend.retainedKeysV1(),backend.currentState());});require(backend.backendUpdateCount()==2,"proof failure updated twice");
 // Real late recovery outside declared replay. Detection continues while
 // actions are operator-held; neither event nor fault identity is renewed.
 NoisyMotionScenario late("config/sc5_open.yaml","OE_GYRO",20261010,104);
 auto lp=resolveRollingPolicy(late.law,late.cfg);lp.replay=30;
 OpenFaultPool lm(late.initial_observation,law,lp.budget);
 RollingIntegrity lb(lp,late.initial_observation,law.initial_sigmas,law.accel_rw,law.gyro_rw,late.lever);
 RollingRecovery lr(late.initial_observation,lp);IncrementalUwbImuEstimator native(late.cfg,late.lever);
 native.initialize(late.initial_observation,law.initial_sigmas);native.ingestImu(late.imu[0]);lm.append(late.imu[0]);
 std::deque<ImuMeasurement> raw{late.imu[0]};std::deque<RollingRawState> caps{lb.certificate().separator};
 for(int k=1;k<=104;++k){for(int i=(k-1)*10+1;i<=k*10;++i){native.ingestImu(late.imu[i]);lm.append(late.imu[i]);raw.push_back(late.imu[i]);}while(raw.size()>301)raw.pop_front();
  auto t=native.prepareEpoch(late.uwb[k-1]);auto mv=lm.view();auto a=lr.prepare(t,mv,raw,caps,late.cfg,k>=100);
  if(k>=100 && (a.recovery!="BOUND_VALID_BUT_RECOVERY_UNAVAILABLE" || a.current_bridge || a.boundary_repair))throw std::runtime_error("outside buffer k="+std::to_string(k)+" status="+a.status+" recovery="+a.recovery+" alert_generation="+std::to_string(mv.alert_generation)+" alarm="+std::to_string(mv.state.alarm)+" first="+std::to_string(a.first_dependency));
  auto prop=lb.propose(t,a.plan,late.range_certificates[k-1],mv);auto c=prop.current;auto saved=t;auto rr=native.commitEpoch(std::move(t),a.plan);
  require(lb.finalize(std::move(prop),rr,native.retainedKeysV1(),native.currentState()).valid,"outside recovery lost valid raw protection");lr.committed(a,saved,rr,native.factorLedger());
  caps.push_back(c);while(caps.size()>31)caps.pop_front();
 }
 require(lm.records().size()==1 && lm.records()[0].begin.value()>=0,"late identity erased");
 std::cout<<"OPEN CHECKS PASS repeated shared-theta paths, residual/birth IDs, concurrent axes outside, identity capacity refusal, finite/open mix, future independence, actual single commit\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
