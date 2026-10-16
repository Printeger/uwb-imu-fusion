#include "measurement_bootstrap.hpp"
#include "unified_rolling_core.hpp"
#include "numerical_candidate.hpp"
#include "sc6_provider.hpp"
#include <tbb/global_control.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;using I=RollingInterval;
void require(bool b,const char* m){if(!b)throw std::runtime_error(m);}
int main(){try{
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 for(size_t n:{5,12}){
  Sc6Provider legacy("config/sc7.yaml","S0",20261010,1);auto cfg=legacy.cfg;cfg.anchors.clear();cfg.imu.max_gap_s=.025;
  cfg.realtime.prior_sigmas.setConstant(.1);auto policy=resolveRollingPolicy(legacy.law,cfg);
  Sc6MotionDomain motion;motion.acceleration=.1;motion.rate=.2;motion.jerk=.1;motion.alpha=.2;
  JointRawLaw law;law.acceleration=motion.acceleration;law.rate=motion.rate;law.jerk=motion.jerk;law.gravity=cfg.imu.gravity_mps2;
  const Eigen::Vector3d true_tag(.3,-.2,1.2);const auto origin=TimestampNs(550000000123LL);
  std::vector<Eigen::Vector3d> anchors{{4,0,0},{0,4,0},{0,0,4},{-4,-4,-4},{4,4,-4}};
  for(size_t i=5;i<n;++i)anchors.push_back({4*std::cos(double(i)),4*std::sin(double(i)),double(i%3)*2});
  auto batchAt=[&](size_t epoch,TimestampNs time){UwbBatch batch;batch.id=BatchId(10000+epoch);batch.timestamp=time;
   for(size_t i=0;i<n;++i){UwbMeasurement m;m.id=MeasurementId(100000+epoch*100+i);m.factor_id=FactorId(m.id.value());m.anchor_id=AnchorId((uint64_t(1)<<52)+i*37);m.timestamp=time;m.anchor_position_m=anchors[i];m.range_m=(true_tag-anchors[i]).norm();m.sigma_m=.01;m.lever_arm_body_m=legacy.lever;
    if(epoch==0 && i==0)m.range_m+=2.25;batch.measurements.push_back(m);}return batch;};
  GeneralizedCoreInitialization model;model.raw_law=law;
  model.sources=std::make_unique<GeneralizedSourceLedger>(policy.budget,law,ModelEvidenceStatus::VerifiedForDeclaredModel);
  RollingRawState physical;for(int j=0;j<3;++j){physical.p[j]=I(0);physical.v[j]=I(-1,1);for(int k=0;k<3;++k)physical.rotation[j][k]=I(-1,1);}
  for(auto& b:physical.bias)b=I(-.02,.02);
  const auto startup=batchAt(0,origin);
  model.initial_set=measurementInitial(*model.sources,startup,physical,legacy.lever,motion.acceleration,motion.rate,ModelEvidenceStatus::VerifiedForDeclaredModel,"measured-range-bootstrap-one-source-fault-v1");
  require(model.initial_set.measurement_derived && model.initial_set.source_prefix==n,"bootstrap minted independent initial prior coordinates");
  const auto prefix=model.sources->event().coordinates();for(const auto& m:startup.measurements)model.sources->append(m);
  require(prefix==model.sources->event().coordinates(),"startup reuse spent risk twice");
  auto feasible=rollingTagSphere(model.initial_set.domain,legacy.lever);
  for(int j=0;j<3;++j)require(feasible[j].lower()<=true_tag[j] && true_tag[j]<=feasible[j].upper(),"startup excluded actual single-source fault");
  // Center is computed only from the arrived (faulty) startup batch. Its
  // posterior/covariance is not used as a raw feasible domain or extra draw.
  NavigationState numerical;numerical.timestamp=origin;
  const auto seed=numericalUwbCandidate(startup,-1);require(seed.functional.tag.allFinite(),"numerical startup seed failed");
  numerical.position_world_m=seed.functional.tag-legacy.lever;
  for(size_t i=0;i<n;++i){AnchorRecord a;a.id=startup.measurements[i].anchor_id;a.position_world_m=anchors[i];a.covariance_m2.setZero();a.frame="world";a.map_version="bootstrap-v1";model.anchors.declare(a,1);}
  ImuMeasurement raw;raw.id=MeasurementId(1234567);raw.timestamp=origin;raw.specific_force_mps2={20,0,law.gravity};raw.angular_velocity_radps.setZero();
  UnifiedRollingIntegrityCore core(legacy.law,cfg,numerical,legacy.lever,motion,raw,std::move(model));
  size_t bridges=0,rejoins=0,post=0;auto previous=origin;
  for(size_t epoch=1;epoch<=40;++epoch){const auto end=TimestampNs(previous.value()+53000003LL);Sc6Envelope input;input.epoch=epoch;input.uwb=batchAt(epoch,end);input.ready=end;
   while(raw.timestamp.value()+12000000LL<end.value()){raw.id=MeasurementId(raw.id.value()+11);raw.timestamp=TimestampNs(raw.timestamp.value()+12000000LL);raw.specific_force_mps2={0,0,law.gravity};input.imu.push_back(raw);}
   for(const auto& m:input.uwb.measurements)input.range_events.push_back({m,end,1,true});
   const auto result=core.step(input,true);bridges+=result.action.current_bridge;rejoins+=result.action.rejoined;
   if(rejoins && !result.action.current_bridge)++post;
   for(int j=0;j<3;++j)require(result.bound.low[j]<=true_tag[j] && true_tag[j]<=result.bound.high[j],"bootstrap/native proof lost truth");
   require(result.bound.valid && result.generation==epoch,"bootstrap common-core commit failed");previous=end;
  }
  require(bridges>0 && rejoins>0 && post>=4,"faulty sample0 was assumed healthy or failed to recover");
  std::cout<<"SC9 measurement bootstrap N="<<n<<" PASS40 native commits; no Initial births, startup range fault, sample0 accel fault, bridge="<<bridges<<" rejoin="<<rejoins<<" post="<<post<<'\n'<<std::flush;
 }
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
