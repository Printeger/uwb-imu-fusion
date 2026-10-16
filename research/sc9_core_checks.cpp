#include "unified_rolling_core.hpp"
#include "sc6_provider.hpp"
#include <tbb/global_control.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool b,const char* m){if(!b)throw std::runtime_error(m);}
int main(){try{
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 for(const std::string scenario:{"U","A","G","J-held"}){
  const size_t n=scenario=="U"?5:12;
  Sc6Provider src("config/sc7.yaml","S0",20261010,1);auto cfg=src.cfg;cfg.anchors.clear();cfg.imu.max_gap_s=.025;
  cfg.realtime.prior_sigmas.setConstant(.001);
  auto policy=resolveRollingPolicy(src.law,cfg);Sc6MotionDomain domain;domain.acceleration=.1;domain.rate=.2;domain.jerk=.1;domain.alpha=.2;
  JointRawLaw law;law.acceleration=domain.acceleration;law.rate=domain.rate;law.jerk=domain.jerk;law.gravity=cfg.imu.gravity_mps2;
  NavigationState initial;initial.timestamp=TimestampNs(917654321000LL);initial.position_world_m={.2,-.1,1.};
  std::vector<Eigen::Vector3d> positions{{4,0,0},{0,4,0},{0,0,4},{-4,-4,-4},{4,4,-4}};
  for(size_t i=5;i<n;++i)positions.push_back({4*std::cos(double(i)),4*std::sin(double(i)),double(i%3)*2});
  GeneralizedCoreInitialization model;model.raw_law=law;
  model.sources=std::make_unique<GeneralizedSourceLedger>(policy.budget,law,ModelEvidenceStatus::VerifiedForDeclaredModel);
  model.initial_set=model.sources->independentInitial(initial,cfg.realtime.prior_sigmas,"stationary-source-law-v1");
  for(size_t i=0;i<n;++i){AnchorRecord a;a.id=AnchorId((uint64_t(1)<<49)+113*i);a.position_world_m=positions[i];a.covariance_m2.setZero();a.frame="world";a.map_version="fixture-v1";model.anchors.declare(a,1);}
  ImuMeasurement raw;raw.id=MeasurementId(uint64_t(1)<<42);raw.timestamp=initial.timestamp;
  raw.specific_force_mps2={0,0,law.gravity};raw.angular_velocity_radps.setZero();
  UnifiedRollingIntegrityCore core(src.law,cfg,initial,src.lever,domain,raw,std::move(model));
  TimestampNs begin=initial.timestamp;int64_t onset=0;size_t alarms=0,exclusions=0,bridges=0,repairs=0,polluted=0,rejoins=0,post=0;
  for(size_t k=1;k<=80;++k){
   const auto end=TimestampNs(begin.value()+(k%2?47000003LL:63000007LL));if(k==40)onset=begin.value()+13000000;
   Sc6Envelope input;input.epoch=k;input.uwb.id=BatchId(90000+k);input.uwb.timestamp=end;input.ready=end;
   while(raw.timestamp.value()+12000000LL<end.value()){
    raw.timestamp=TimestampNs(raw.timestamp.value()+12000000LL);raw.id=MeasurementId(raw.id.value()+7);
    raw.specific_force_mps2={0,0,law.gravity};raw.angular_velocity_radps.setZero();
    if(onset && onset<=raw.timestamp.value() && raw.timestamp.value()<onset+500000000LL){
     if(scenario=="A" || scenario=="J-held")raw.specific_force_mps2.x()+=20;
     if(scenario=="G")raw.angular_velocity_radps.x()+=1;
    }
    input.imu.push_back(raw);
   }
   const Eigen::Vector3d truth=initial.position_world_m+src.lever;
   for(size_t i=0;i<n;++i){UwbMeasurement m;m.id=MeasurementId(300000+k*100+i);m.factor_id=FactorId(m.id.value());m.anchor_id=AnchorId((uint64_t(1)<<49)+113*i);m.anchor_position_m=positions[i];
    m.timestamp=TimestampNs(end.value()-int64_t(i%3)*1000000LL);m.range_m=(truth-positions[i]).norm();m.sigma_m=.01;m.lever_arm_body_m=src.lever;
    if((scenario=="U" || scenario=="J-held") && k>=40 && k<=42 && i==0)m.range_m+=2.25;
    input.uwb.measurements.push_back(m);input.range_events.push_back({m,end,1,true});
   }
   const bool fde=scenario!="J-held" || k>=73;
   Sc6Result r;try{r=core.step(input,fde);}catch(const std::exception& e){std::cerr<<scenario<<" epoch="<<k<<" ";throw;}if(!(r.max_range_transport>0 && r.time_factor_groups>0 && r.time_committed_factors>0)){std::cerr<<scenario<<" k="<<k<<" radius="<<r.max_range_transport<<" groups="<<r.time_factor_groups<<" committed="<<r.time_committed_factors<<"\n";throw std::runtime_error("actual time factor missing from native ledger");}require(r.bound.valid && r.source_prefix>0,"missing common-core bound");
   require(r.state.timestamp==end && r.generation==k && core.updateCount()==k+1,"common-core commit/source mismatch");
   for(int j=0;j<3;++j)require(r.bound.low[j]<=truth[j] && truth[j]<=r.bound.high[j],"fault union lost truth");
   alarms+=r.imu_alarm || r.action.uwb_alarm;exclusions+=r.action.excluded>=0;bridges+=r.action.current_bridge;repairs+=r.action.boundary_repair;polluted+=r.action.history_marginalized;rejoins+=r.action.rejoined;
   if(rejoins && !r.action.current_bridge)++post;
   begin=end;
  }
  require(alarms>0,"fault did not trigger real monitor");
  if(scenario=="U")require(exclusions==5,"UWB source hold did not end after two further observations");
  else require(bridges>0 && repairs>0 && rejoins>0 && post>=4,"missing native bridge/prior/rejoin post epochs");
  if(scenario=="J-held")require(polluted>0 && exclusions>0,"no actual delayed marginal repair/joint action");
  std::cout<<"SC9 common core "<<scenario<<" N="<<n<<" commits=80 alarms="<<alarms<<" exclusions="<<exclusions<<" bridge="<<bridges<<" repair="<<repairs<<" polluted="<<polluted<<" rejoin="<<rejoins<<" post="<<post<<'\n'<<std::flush;
 }
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
