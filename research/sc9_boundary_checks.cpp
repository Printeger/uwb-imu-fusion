#include "generalized_source_ledger.hpp"
#include "rolling_recovery.hpp"
#include "sc6_provider.hpp"
#include <tbb/global_control.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool b,const char* m){if(!b)throw std::runtime_error(m);}
template<class F>void refuses(F f){bool b=false;try{f();}catch(const std::exception&){b=true;}require(b,"expected source/chronology refusal");}
int main(){try{
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 size_t commits=0,retirements=0,holds=0;
 for(size_t n:{5,12}){
  Sc6Provider src("config/sc7.yaml","S0",20261010,1);auto cfg=src.cfg;cfg.anchors.clear();cfg.imu.max_gap_s=.025;
  const auto policy=resolveRollingPolicy(src.law,cfg);
  NavigationState initial;initial.timestamp=TimestampNs(917654321000LL);initial.position_world_m={.2,-.1,1.};
  Eigen::Matrix<double,15,1> sigmas;sigmas.setConstant(.001);
  JointRawLaw law;law.acceleration=.1;law.rate=.2;law.jerk=.1;law.gravity=cfg.imu.gravity_mps2;
  GeneralizedSourceLedger ledger(policy.budget,law,ModelEvidenceStatus::VerifiedForDeclaredModel);
  const auto initial_set=ledger.independentInitial(initial,sigmas,"independent-synthetic-stationary-gaussian-law-v1");
  RollingIntegrity proof(policy,initial_set,law.accel_rw,law.gyro_rw,src.lever,law.acceleration,law.rate);
  proof.enableGeneralizedJoint(law);proof.enableTagSupport(32);
  IncrementalUwbImuEstimator backend(cfg,src.lever);backend.initialize(initial,sigmas);
  ImuMeasurement raw;raw.id=MeasurementId(uint64_t(1)<<42);raw.timestamp=initial.timestamp;
  raw.specific_force_mps2={0,0,law.gravity};raw.angular_velocity_radps.setZero();
  ledger.append(raw);backend.ingestImu(raw);
  const auto births=ledger.event().coordinates();ledger.append(raw);require(births==ledger.event().coordinates(),"retry spent new risk");
  auto changed=raw;changed.timestamp=TimestampNs(raw.timestamp.value()+1);refuses([&]{ledger.append(changed);});
  std::vector<Eigen::Vector3d> anchors{{4,0,0},{0,4,0},{0,0,4},{-4,-4,-4},{4,4,-4}};
  for(size_t i=5;i<n;++i)anchors.push_back({4*std::cos(double(i)),4*std::sin(double(i)),double(i%3)*2});
  TimestampNs begin=initial.timestamp;
  for(size_t k=1;k<=40;++k){
   const auto end=TimestampNs(begin.value()+(k%2?47000003LL:63000007LL));
   while(raw.timestamp.value()+(n==5?12000000LL:7000000LL)<end.value()){
    raw.timestamp=TimestampNs(raw.timestamp.value()+(n==5?12000000LL:7000000LL));raw.id=MeasurementId(raw.id.value()+7);
    ledger.append(raw);backend.ingestImu(raw);
   }
   UwbBatch batch;batch.id=BatchId(90000+k);batch.timestamp=end;
   const Eigen::Vector3d truth=initial.position_world_m+src.lever;
   for(size_t i=0;i<n;++i){
    if(i==0 && k%7==0)continue;
    UwbMeasurement m;m.id=MeasurementId(300000+k*100+i);m.factor_id=FactorId(m.id.value());
    m.anchor_id=AnchorId((uint64_t(1)<<49)+113*i);m.anchor_position_m=anchors[i];
    m.timestamp=TimestampNs(end.value()-int64_t(i%3)*1000000LL);m.range_m=(truth-anchors[i]).norm();m.sigma_m=.01;m.lever_arm_body_m=src.lever;
    ledger.append(m);batch.measurements.push_back(m);
   }
   if(k%4==0){auto m=batch.measurements.back();m.id=MeasurementId(m.id.value()+50);m.factor_id=FactorId(m.id.value());ledger.append(m);batch.measurements.push_back(m);}
   if(k%3==0)std::reverse(batch.measurements.begin(),batch.measurements.end());
   auto tx=backend.prepareEpoch(batch);const auto evidence=ledger.evidence(tx);
   holds+=evidence->joint_samples.back().raw.timestamp<evidence->end;
   RollingMonitorView view;view.open=true;view.event_id=ledger.event().id();view.state.nominal_possible=true;
   view.matches=[&](const EpochTransaction& t){return t.id==tx.id;};
   const auto plan=EpochCommitPlan::nominalPlan(tx);
   if(k==2){auto wrong=std::make_shared<GeneralizedRawEvidence>(*evidence);wrong->risk_upper=0;
    refuses([&]{proof.propose(tx,plan,{},view,wrong);});
    wrong=std::make_shared<GeneralizedRawEvidence>(*evidence);wrong->joint_samples.back().raw.id=MeasurementId(999);
    refuses([&]{proof.propose(tx,plan,{},view,wrong);});}
   auto proposal=proof.propose(tx,plan,{},view,evidence);
   const auto parent_raw=proposal.explicit_material.front().raw_first;
   if(k>=30)require(proposal.certificate.shared_adc_index==parent_raw,"separator raw parent rebased to latest epoch");
   const auto count=backend.backendUpdateCount();auto receipt=backend.commitEpoch(std::move(tx),plan);
   const auto bound=proof.finalize(std::move(proposal),receipt,backend.retainedKeysV1(),backend.currentState());
   require(bound.valid && bound.status=="GENERALIZED_CONDITIONAL_RAW_BOUND_VERIFIED" && bound.tag_support,"generalized bound not finalized");
   for(int j=0;j<3;++j)require(bound.low[j]<=truth[j] && truth[j]<=bound.high[j],"true stationary tag removed");
   require(!proof.certificate().fault_counts_complete && proof.explicitSize()<size_t(policy.integrity),"fake legacy counts or unbounded explicit window");
   require(receipt.backend_updates==1 && backend.backendUpdateCount()==count+1,"repeated native commit");
   if(!receipt.marginalized_keys.empty())++retirements;
   ++commits;begin=end;
  }
 }
 require(retirements==22 && holds==80,"missing held or retired coverage");
 std::cout<<"SC9 boundary PASS native="<<commits<<" retired="<<retirements<<" held="<<holds<<"; N5/12, dynamic M, actual-time/skew, sparse64, source reuse/tamper, separator identity, conditional truth containment; synthetic fixture only\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
