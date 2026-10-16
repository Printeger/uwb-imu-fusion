#include "unified_rolling_core.hpp"
#include "sc6_provider.hpp"
#include <tbb/global_control.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool b,const char* m){if(!b)throw std::runtime_error(m);}
int main(){try{
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 Sc6Provider legacy("config/sc7.yaml","S0",20261010,1);auto cfg=legacy.cfg;cfg.anchors.clear();cfg.imu.max_gap_s=.025;cfg.realtime.prior_sigmas.setConstant(.001);
 auto policy=resolveRollingPolicy(legacy.law,cfg);Sc6MotionDomain motion;motion.acceleration=.1;motion.rate=.2;motion.jerk=.1;motion.alpha=.2;
 JointRawLaw law;law.acceleration=motion.acceleration;law.rate=motion.rate;law.jerk=motion.jerk;law.gravity=cfg.imu.gravity_mps2;
 NavigationState initial;initial.timestamp=TimestampNs(770000000123LL);initial.position_world_m={.3,-.2,1.2};
 GeneralizedCoreInitialization model;model.raw_law=law;model.sources=std::make_unique<GeneralizedSourceLedger>(policy.budget,law,ModelEvidenceStatus::VerifiedForDeclaredModel);
 model.initial_set=model.sources->independentInitial(initial,cfg.realtime.prior_sigmas,"topology-source-law-v1");
 std::vector<AnchorRecord> anchors;
 for(size_t i=0;i<72;++i){AnchorRecord a;a.id=AnchorId((uint64_t(1)<<50)+37*i);a.position_world_m={4*std::cos(double(i)),4*std::sin(double(i)),2.*double(i%3)};a.covariance_m2.setZero();a.frame="world";a.map_version="v1";model.anchors.declare(a,1);anchors.push_back(a);}
 ImuMeasurement raw;raw.id=MeasurementId(9876543);raw.timestamp=initial.timestamp;raw.specific_force_mps2={0,0,law.gravity};raw.angular_velocity_radps.setZero();
 UnifiedRollingIntegrityCore core(legacy.law,cfg,initial,legacy.lever,motion,raw,std::move(model));
 const std::vector<size_t> visible{0,1,2,3,4,5,6,8,12,32,65,72,5};auto begin=initial.timestamp;size_t old_prefix=0;
 for(size_t k=1;k<=visible.size();++k){
  if(k==6)core.anchorOnline(anchors.back().id,false);if(k==11)core.anchorOnline(anchors.back().id,true);
  if(k==8){anchors[3].position_world_m.x()+=.1;anchors[3].map_version="v2";core.declareAnchor(anchors[3],2);}
  if(k==13){AnchorRecord a;a.id=AnchorId(UINT64_MAX);a.position_world_m={1,2,3};a.covariance_m2.setZero();a.frame="world";a.map_version="v1";core.declareAnchor(a,1);}
  const auto end=TimestampNs(begin.value()+53000003LL);Sc6Envelope input;input.epoch=k;input.uwb.id=BatchId(9000+k);input.uwb.timestamp=end;input.ready=end;
  while(raw.timestamp.value()+12000000LL<end.value()){raw.id=MeasurementId(raw.id.value()+11);raw.timestamp=TimestampNs(raw.timestamp.value()+12000000LL);input.imu.push_back(raw);}
  const Eigen::Vector3d truth=initial.position_world_m+legacy.lever;
  for(size_t i=0;i<72;++i){if(i==71 && k>=6 && k<11)continue;
   UwbMeasurement m;m.id=MeasurementId(100000+k*100+i);m.factor_id=FactorId(m.id.value());m.anchor_id=anchors[i].id;m.anchor_position_m=anchors[i].position_world_m;
   m.timestamp=TimestampNs(end.value()-int64_t(i%3)*1000000LL);m.sigma_m=.01;m.lever_arm_body_m=legacy.lever;const bool valid=i<visible[k-1];
   m.range_m=valid?(truth-m.anchor_position_m).norm():-1.;input.range_events.push_back({m,end,i==3 && k>=8?2u:1u,valid});if(valid)input.uwb.measurements.push_back(m);
  }
  if(k==13){auto duplicate=input.range_events[4];duplicate.measurement.id=MeasurementId(999999);duplicate.measurement.factor_id=FactorId(999999);input.range_events.push_back(duplicate);input.uwb.measurements.push_back(duplicate.measurement);}
  const auto result=core.step(input,true);
  require(result.input_counts.registered==(k==13?73u:72u) && result.input_counts.visible==visible[k-1] && result.input_counts.measurements==visible[k-1]+(k==13),"registered/visible/M conflated");
  require(result.source_prefix>old_prefix && result.bound.valid && core.updateCount()==k+1,"topology reset source/bound/backend");old_prefix=result.source_prefix;
  for(int j=0;j<3;++j)require(result.bound.low[j]<=truth[j] && truth[j]<=result.bound.high[j],"topology lost true tag");
  if(visible[k-1]<4)require(result.action.recovery=="FUNCTIONAL_ACTION_UNAVAILABLE","unobservable geometry falsely recovered");
  begin=end;
 }
 std::cout<<"SC9 topology PASS native13 commits N_visible=0/1/2/3/4/5/6/8/12/32/65/72/5, registered72->73, M distinct, invalid births, sparse64/UINT64_MAX, disconnect/return, calibration revision, collision-free active factor IDs\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
