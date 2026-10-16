#include "empirical_sensor_adapter.hpp"
#include <tbb/global_control.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
namespace {
void require(bool value,const char* error){if(!value)throw std::runtime_error(error);}
template<class F>void refuses(F run){bool rejected=false;try{run();}catch(const std::exception&){rejected=true;}require(rejected,"malformed stream admitted");}
}
int main(){try{
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 const auto model=YAML::Load(R"(
status: DECLARED_EMPIRICAL
model_id: deterministic-stream-fixture
gravity_mps2: 9.81
accel_adc_sigma: 0.01
gyro_adc_sigma: 0.001
accel_rw_sigma: 0.0001
gyro_rw_sigma: 0.0001
motion: {acceleration: 0.1, rate: 0.2, jerk: 0.1, alpha: 0.2}
R_model_from_anchor_world: [[1,0,0],[0,1,0],[0,0,1]]
range_to_imu_time_s: 0
initial_velocity_radius: 0.1
initial_bias_radius: [0.02,0.02,0.02,0.02,0.02,0.02]
accel_bias_mps2: [0,0,0]
gyro_bias_radps: [0,0,0]
)");
 const auto setup=empiricalSetup(model);
 const std::vector<Eigen::Vector3d> positions{{4,0,0},{0,4,0},{0,0,4},{-4,-4,-4},{4,4,-4}};
 AnchorRegistry registry;std::vector<AnchorId> ids;
 for(size_t j=0;j<positions.size();++j){AnchorRecord a;a.id=AnchorId(j?((uint64_t(1)<<49)+j*7):UINT64_MAX);a.position_world_m=positions[j];a.frame="world";a.map_version="stream-fixture";registry.declare(a,1);ids.push_back(a.id);}
 const Eigen::Vector3d body(.2,-.1,1.),truth=body+setup.lever;
 // Negative arbitrary start time also exercises arrival max without a hidden
 // timestamp-zero assumption. Range arrives before the closing IMU packet.
 const int64_t origin=-917000000000LL;
 std::vector<ImuMeasurement> imu;
 for(size_t k=0;k<=302;++k){ImuMeasurement m;m.id=MeasurementId((uint64_t(1)<<42)+k*7);m.timestamp=TimestampNs(origin+int64_t(k)*10000000);m.specific_force_mps2={0,0,9.81};imu.push_back(m);}
 std::vector<EmpiricalBatch> batches;
 for(size_t k=0;k<=40;++k){EmpiricalBatch b;b.values.id=BatchId(9000+k);b.values.timestamp=TimestampNs(origin+555000000+int64_t(k)*60000000);b.latest_original_valid_range=b.values.timestamp;
  for(size_t j=0;j<positions.size();++j){UwbMeasurement m;m.id=MeasurementId(1000000+k*100+j);m.factor_id=FactorId(m.id.value());m.anchor_id=ids[j];m.timestamp=b.values.timestamp;m.anchor_position_m=positions[j];m.lever_arm_body_m=setup.lever;m.sigma_m=.01;m.range_m=(truth-positions[j]).norm();
   if(k>=15 && k<=17 && j==0)m.range_m+=2.25;
   b.values.measurements.push_back(m);b.events.push_back({m,TimestampNs(m.timestamp.value()+2000000),1,true});
  }batches.push_back(b);
 }
 struct Arrival{int64_t time;bool imu;size_t index;};std::vector<Arrival> arrivals;
 for(size_t k=0;k<imu.size();++k)arrivals.push_back({imu[k].timestamp.value()+12000000,true,k});
 for(size_t k=0;k<batches.size();++k)arrivals.push_back({batches[k].values.timestamp.value()+2000000,false,k});
 std::sort(arrivals.begin(),arrivals.end(),[](const auto& a,const auto& b){return a.time<b.time;});
 CausalEmpiricalStream stream(setup,registry);std::vector<Sc6Result> streamed;
 for(const auto& arrival:arrivals){
  if(arrival.imu)stream.appendImu(imu[arrival.index],TimestampNs(arrival.time));else stream.appendBatch(batches[arrival.index]);
  while(auto out=stream.next(true))streamed.push_back(std::move(out->result));
 }
 require(streamed.size()==40 && stream.startupBatches()==1 && stream.pendingBatches()==0 && stream.updates()==41,"stream denominator/watermark accounting");
 auto direct=empiricalBootstrap(setup,registry,std::vector<ImuMeasurement>(imu.begin(),imu.begin()+57),batches.front().events,batches.front().values);
 require(direct.source_digest==stream.startupDigest(),"startup arrived-source mismatch");
 size_t cursor=57,exclusions=0;
 for(size_t k=1;k<batches.size();++k){Sc6Envelope input;input.epoch=k;input.uwb=batches[k].values;input.range_events=batches[k].events;input.ready=batches[k].events.front().received;
  while(cursor<imu.size() && !(input.uwb.timestamp<imu[cursor].timestamp))input.imu.push_back(imu[cursor++]);
  const auto r=direct.core->step(input,true);const auto& s=streamed[k-1];
  require(r.source_digest==s.source_digest && r.receipt_digest==s.receipt_digest && r.ledger_digest==s.ledger_digest && r.boundary_digest==s.boundary_digest,"stream/direct native semantic mismatch");
  require(r.tag==s.tag && r.bound.hpl==s.bound.hpl && r.bound.pl==s.bound.pl,"stream/direct numeric mismatch");
  exclusions+=s.action.excluded_source.has_value();
  for(int j=0;j<3;++j)require(s.bound.low[j]<=truth[j] && truth[j]<=s.bound.high[j],"stream diagnostic set lost fixture truth");
 }
 require(exclusions>0,"stream never committed physical-source isolation");
 {CausalEmpiricalStream s(setup,registry);refuses([&]{s.appendImu(imu[0],TimestampNs(imu[0].timestamp.value()-1));});refuses([&]{s.next(true);});}
 {CausalEmpiricalStream s(setup,registry);s.appendImu(imu[0],imu[0].timestamp);refuses([&]{s.appendImu(imu[0],imu[0].timestamp);});}
 {CausalEmpiricalStream s(setup,registry);s.appendImu(imu[0],imu[0].timestamp);auto gap=imu[1];gap.timestamp=TimestampNs(imu[0].timestamp.value()+50000001);refuses([&]{s.appendImu(gap,gap.timestamp);});}
 {CausalEmpiricalStream s(setup,registry);auto b=batches[0];b.events[0].received=TimestampNs(b.events[0].measurement.timestamp.value()-1);refuses([&]{s.appendBatch(b);});}
 {CausalEmpiricalStream s(setup,registry);s.appendBatch(batches[1]);refuses([&]{s.appendBatch(batches[0]);});}
 {CausalEmpiricalStream s(setup,registry);for(size_t k=0;k<4096;++k){auto m=imu[0];m.id=MeasurementId(k+1);m.timestamp=TimestampNs(origin+int64_t(k)*10000000);s.appendImu(m,m.timestamp);}auto m=imu[0];m.timestamp=TimestampNs(origin+4096LL*10000000);refuses([&]{s.appendImu(m,m.timestamp);});}
 std::cout<<"SC9 causal stream PASS40 native epochs/direct digest equality, measured startup, early range/late IMU, sparse64/U/recovery, negative start, future/order/gap/capacity and failed-closed cases\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
