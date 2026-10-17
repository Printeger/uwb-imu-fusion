#include "sc6_provider.hpp"
#include "rolling_policy.hpp"
#include "uwb_imu_pl/common/timestamp_delta.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include <tbb/global_control.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
namespace uwb_imu_pl {struct P106ReadOnlyGraphSnapshotPeer {static const gtsam::ISAM2& isam(const IncrementalUwbImuEstimator& b){return b.backendIsam();}};}
namespace {
void require(bool b,const char* m){if(!b)throw std::runtime_error(m);}
struct Sample{NavigationState state,cv;double dt;Eigen::Matrix<double,15,15> covariance;};
std::vector<Sample> run(int64_t origin){
 Sc6Provider src("config/sc7.yaml","S0",20261010,1);auto cfg=src.cfg;cfg.anchors.clear();cfg.imu.max_gap_s=.025;resolveRollingPolicy(src.law,cfg);cfg.incremental.max_time_skew_s=1e-8;
 NavigationState initial;initial.timestamp=TimestampNs(origin);initial.position_world_m={.2,-.1,1.};initial.velocity_world_mps={.1,-.02,.03};
 IncrementalUwbImuEstimator backend(cfg,src.lever);backend.initialize(initial,cfg.realtime.prior_sigmas);backend.enableGeneralizedGroupIdentityResearchV2();backend.enablePhysicalTimeResearchV1();
 ImuMeasurement raw;raw.id=MeasurementId(1);raw.timestamp=initial.timestamp;raw.specific_force_mps2={.02,-.01,cfg.imu.gravity_mps2};raw.angular_velocity_radps={.001,.002,.003};backend.ingestImu(raw);
 int64_t elapsed=0,raw_elapsed=0;std::vector<Sample> out;size_t retired=0;
 const std::vector<Eigen::Vector3d> anchors{{4,0,0},{0,4,0},{0,0,4},{-4,-4,-4},{4,4,-4}};
 for(size_t k=1;k<=80;++k){const int64_t interval=k%2?47000003:63000007;elapsed+=interval;
  while(raw_elapsed+12000000<elapsed){raw_elapsed+=12000000;raw.timestamp=TimestampNs(origin+raw_elapsed);raw.id=MeasurementId(raw.id.value()+1);backend.ingestImu(raw);}
  UwbBatch batch;batch.id=BatchId(k);batch.timestamp=TimestampNs(origin+elapsed);
  for(size_t j=0;j<anchors.size();++j){UwbMeasurement m;m.id=MeasurementId(10000+k*10+j);m.factor_id=FactorId(m.id.value());m.anchor_id=AnchorId(j+1);m.timestamp=batch.timestamp;m.anchor_position_m=anchors[j];m.lever_arm_body_m=src.lever;m.sigma_m=.01;m.range_m=(initial.position_world_m+initial.velocity_world_mps*(double(elapsed)/1e9)+src.lever-anchors[j]).norm();batch.measurements.push_back(m);}
  if(k==1){auto bad=batch;bad.measurements.front().timestamp=TimestampNs(batch.timestamp.value()+11);bool rejected=false;try{backend.validateUwbBatch(bad);}catch(const std::exception&){rejected=true;}require(rejected,"Unix nanosecond skew was admitted");}
  auto tx=backend.prepareEpoch(batch);const double expected=double(interval)/1e9;
  require(std::abs(tx.preintegration->deltaTij()-expected)<1e-15,"native PIM lost nanosecond interval");
  const auto* bridge=dynamic_cast<const KinematicPoseVelocityBridgeFactor*>(tx.generic_bridge_group.factors.at(0).get());
  require(bridge && bridge->dtSeconds()==expected,"native bridge lost nanosecond interval");
  Sample sample;sample.cv=tx.cv_predicted_state;sample.dt=tx.preintegration->deltaTij();sample.covariance=tx.preintegration->preintMeasCov();
  auto plan=EpochCommitPlan::nominalPlan(tx);if(k==12 || k==31){plan.groups_to_add.erase(plan.groups_to_add.begin());plan.groups_to_add.push_back(tx.generic_bridge_group.id);plan.groups_to_add.push_back(tx.generic_bias_continuity_group.id);plan.bridge_mode=BridgeMode::GenericKinematic;}CommitReceipt receipt;try{receipt=backend.commitEpoch(std::move(tx),plan);}catch(const std::exception& e){throw std::runtime_error("origin="+std::to_string(origin)+" epoch="+std::to_string(k)+" "+e.what());}retired+=!receipt.marginalized_keys.empty();const auto& native=P106ReadOnlyGraphSnapshotPeer::isam(backend);for(const auto& factor:gtsam::GaussianFactorGraph(native))if(factor)for(auto key:factor->keys())if(!native.getLinearizationPoint().exists(key)){factor->print("DANGLING TREE FACTOR ");throw std::runtime_error("marginalized key still in native tree at epoch="+std::to_string(k)+" key="+gtsam::DefaultKeyFormatter(key));}sample.state=backend.currentState();out.push_back(sample);
 }
 require(retired>0,"absolute-time test did not marginalize");
 bool rejected=false;try{backend.enablePhysicalTimeResearchV1();}catch(const std::exception&){rejected=true;}require(rejected,"late time-mode mutation admitted");
 return out;
}
void equalState(const NavigationState& a,const NavigationState& b){require(a.position_world_m==b.position_world_m && a.velocity_world_mps==b.velocity_world_mps,"time origin changed position/velocity");require(a.q_world_body.coeffs()==b.q_world_body.coeffs(),"time origin changed orientation");require(a.accel_bias_mps2==b.accel_bias_mps2 && a.gyro_bias_radps==b.gyro_bias_radps,"time origin changed bias");}
}
int main(){try{tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 auto reference=run(0);for(int64_t origin:{1664959757027770002LL,-1664959757027770002LL}){auto other=run(origin);for(size_t i=0;i<reference.size();++i){equalState(reference[i].state,other[i].state);equalState(reference[i].cv,other[i].cv);require(reference[i].dt==other[i].dt && reference[i].covariance==other[i].covariance,"time origin changed native preintegration");}}
 require(std::isfinite(timestampDifferenceSecondsV1(TimestampNs(INT64_MAX),TimestampNs(INT64_MIN))),"full timestamp range overflow");
 require(timestampDifferenceSecondsV1(TimestampNs(1664959757027770013LL),TimestampNs(1664959757027770002LL))==11e-9,"nanosecond difference collapsed");
 std::cout<<"SC9 native physical time PASS240 commits at zero/Unix/negative origins; exact states, CV, PIM covariance, bridge dt, marginalization, late-mode refusal\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
