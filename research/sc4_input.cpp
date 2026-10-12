#include "sc4_system.hpp"
#include "scenario_material.hpp"
#include "uwb_imu_pl/estimation/causal_initializer.hpp"
#include <fstream>
#include <sstream>
#include <map>
#include <set>

namespace uwb_imu_pl::research {
Sc4Input sc4MatchedInput(const std::string& path,const std::string& scene,int count){
 auto cfg=researchLaw(path);
 if(count<1 || count>cfg["sc4_max_input_epochs"].as<int>() || cfg["sc4_scope_id"].as<std::string>()!="sc3-one-uwb-one-imu-episode-rw-v1" || cfg["risk_horizon_epochs"].as<int>()!=50 || cfg["sc3_initial_carry_in"].as<bool>() || cfg["sc3_mode_capacity"].as<int>()!=1400 || cfg["sc3_action_capacity"].as<int>()!=18 || cfg["sc3_range_box_capacity"].as<int>()!=256)throw std::runtime_error("SC4 registered scope/resource admission refused");
 if(scene!="SC3_U0" && scene!="SC3_U1" && scene!="SC3_U2" && scene!="SC3_IA" && scene!="SC3_IG" && scene!="SC3_A" && scene!="SC3_G" && scene!="SC3_STAGGERED")throw std::runtime_error("SC4 unknown matched input recipe");
 NoisyMotionScenario s(path,scene,20261010,count,true);
 if(!s.physical_source_reference || !s.initial_law_valid || !s.imu_fault_numeric_certificate.valid)throw std::runtime_error("SC4 matched source admission refused, not a detector alarm");
 Sc4Input input;input.config=s.cfg;input.initial=s.initial_observation;input.lever=s.lever;
 input.imu=std::move(s.imu);input.batches=std::move(s.uwb);input.ranges=std::move(s.range_certificates);
 auto& law=input.law;law.numeric=s.imu_fault_numeric_certificate;law.initial_sigmas=input.config.realtime.prior_sigmas;
 law.accel_adc=s.law["raw_accel_sigma_mps2"].as<double>();law.gyro_adc=s.law["raw_gyro_sigma_radps"].as<double>();law.accel_rw=input.config.imu.accelerometer_bias_rw_sigma;law.gyro_rw=input.config.imu.gyroscope_bias_rw_sigma;law.gravity=input.config.imu.gravity_mps2;
 law.motion_amplitude=s.amp;law.motion_frequency=s.freq;law.motion_angular_rate=s.rate;law.horizon=50;law.risk_budget=s.law["raw_event_risk"].as<double>();
 input.matched_source=true;input.source="raw-source-rw-v1";input.scope="sc3-one-uwb-one-imu-episode-rw-v1";
 if(count>50 && cfg["sc4_stress_fixed_lag_epochs"])input.config.incremental.fixed_lag_epochs=cfg["sc4_stress_fixed_lag_epochs"].as<int>();
 return input;
}
namespace {
using Row=std::map<std::string,std::string>;
std::vector<std::string> split(const std::string& line){std::vector<std::string> out;std::stringstream s(line);std::string x;while(std::getline(s,x,',')){if(!x.empty() && x.back()=='\r')x.pop_back();out.push_back(x);}return out;}
std::vector<Row> csv(const std::string& path){std::ifstream stream(path);if(!stream)throw std::runtime_error("missing canonical input "+path);std::string line;std::getline(stream,line);auto header=split(line);std::vector<Row> rows;while(std::getline(stream,line)){if(line.empty())continue;auto fields=split(line);if(fields.size()!=header.size())throw std::runtime_error("malformed canonical input row");Row row;for(size_t i=0;i<fields.size();++i)row[header[i]]=fields[i];rows.push_back(std::move(row));}return rows;}
double number(const Row& row,const std::string& key){size_t end=0;double x=std::stod(row.at(key),&end);if(end!=row.at(key).size() || !std::isfinite(x))throw std::runtime_error("nonfinite/invalid canonical input");return x;}
}
Sc4Input sc4RecordInput(const std::string& config,const std::string& cache,int count){
 auto law=researchLaw(config);Sc4Input out;out.config=IntegrityConfigLoader::load(law["imu_base_config"].as<std::string>());
 std::vector<ImuMeasurement> imu;std::vector<UwbMeasurement> ranges;
 for(const auto& row:csv(cache+"/imu.csv")){ImuMeasurement m;m.id=MeasurementId(imu.size()+1);m.timestamp=TimestampNs(std::llround(number(row,"t")*1e9));m.specific_force_mps2={number(row,"ax"),number(row,"ay"),number(row,"az")};m.angular_velocity_radps={number(row,"gx"),number(row,"gy"),number(row,"gz")};imu.push_back(m);}
 std::map<int64_t,UwbBatch> batches;uint64_t rid=1;
 for(const auto& row:csv(cache+"/uwb.csv")){if(number(row,"valid")!=1 || number(row,"tag_id")!=0)continue;UwbMeasurement m;m.id=MeasurementId(rid++);m.factor_id=FactorId(m.id.value());m.timestamp=TimestampNs(std::llround(number(row,"t")*1e9));m.anchor_id=AnchorId(uint64_t(number(row,"anchor_id")));m.range_m=number(row,"range");m.sigma_m=number(row,"sigma");m.anchor_position_m={number(row,"anchor_x"),number(row,"anchor_y"),number(row,"anchor_z")};m.lever_arm_body_m={number(row,"lever_x"),number(row,"lever_y"),number(row,"lever_z")};ranges.push_back(m);auto& b=batches[m.timestamp.value()];b.timestamp=m.timestamp;b.measurements.push_back(m);}
 if(imu.size()<2 || ranges.empty() || count<1)throw std::runtime_error("insufficient real input");
 out.lever=ranges.front().lever_arm_body_m.value_or(Eigen::Vector3d::Zero());auto initial=initializeCausallyV1(imu,ranges,out.config,out.lever,true);
 auto origin=initial.state.timestamp;out.initial=initial.state;out.initial.timestamp=TimestampNs(0);out.config.realtime.prior_sigmas=initial.prior_sigmas;
 // Causal bootstrap consumes the prefix. No GT, labels or draw certification.
 auto boundary=initial.boundary;boundary.id=MeasurementId(1);boundary.timestamp=TimestampNs(0);out.imu.push_back(boundary);
 for(auto m:imu)if(origin<m.timestamp){m.id=MeasurementId(out.imu.size()+1);m.timestamp=TimestampNs(m.timestamp.value()-origin.value());out.imu.push_back(m);}
 for(auto& kv:batches)if(origin<kv.second.timestamp && out.batches.size()<size_t(count)){auto b=kv.second;b.id=BatchId(out.batches.size()+1);b.timestamp=TimestampNs(b.timestamp.value()-origin.value());for(auto& m:b.measurements)m.timestamp=b.timestamp;out.batches.push_back(std::move(b));}
 if(out.batches.size()!=size_t(count))throw std::runtime_error("real input has fewer requested epochs");
 return out;
}
std::string sc4InputError(const Sc4Input& input){
 if(input.imu.empty() || input.batches.empty() || input.initial.timestamp.value()!=0 || input.imu.front().timestamp.value()!=0)return "INITIAL_BOUNDARY_MISSING";
 int64_t previous=-1;std::set<uint64_t> ids;
 for(const auto& m:input.imu){if(m.timestamp.value()<=previous || !ids.insert(m.id.value()).second || !m.specific_force_mps2.allFinite() || !m.angular_velocity_radps.allFinite())return "IMU_TIME_OR_SOURCE_INVALID";previous=m.timestamp.value();}
 previous=0;ids.clear();
 for(size_t i=0;i<input.batches.size();++i){const auto& b=input.batches[i];if(b.timestamp.value()<=previous || b.measurements.empty())return "UWB_TIME_OR_MISSING";previous=b.timestamp.value();std::set<AnchorId> anchors;
  for(const auto& m:b.measurements)if(m.timestamp!=b.timestamp || !ids.insert(m.id.value()).second || !anchors.insert(m.anchor_id).second || !std::isfinite(m.range_m) || m.range_m<=0 || !std::isfinite(m.sigma_m) || m.sigma_m<=0 || !m.anchor_position_m.allFinite())return "UWB_SOURCE_INVALID";
  if(input.matched_source && (b.measurements.size()!=8 || b.timestamp.value()!=int64_t(i+1)*50000000LL))return "MATCHED_BATCH_DOMAIN_INVALID";
 }
 if(input.matched_source){if(input.ranges.size()!=input.batches.size() || !input.law.numeric.valid)return "MATCHED_EVIDENCE_MISSING";for(size_t i=0;i<input.imu.size();++i)if(input.imu[i].timestamp.value()!=int64_t(i)*5000000LL || input.imu[i].id.value()!=i+1)return "MATCHED_SAMPLE_GAP";}
 if(input.imu.back().timestamp<input.batches.back().timestamp)return "IMU_TAIL_MISSING";
 return {};
}
}
