#pragma once
#include "raw_imu_episode.hpp"
#include "raw_nonlinear_inclusion.hpp"
#include "raw_range_contraction.hpp"
#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <random>

namespace uwb_imu_pl::research {
inline Eigen::Quaterniond attitude(double time,bool rotating) {
  if(!rotating)return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(1.2*time,Eigen::Vector3d(1.,2.,3.).normalized()));
}
inline ImuMeasurement sample(const IntegrityConfig& cfg,int index,bool rotating=false) {
  const double t=index*.005;
  ImuMeasurement s;s.id=MeasurementId(index+1);s.timestamp=TimestampNs(index*5000000LL);
  s.specific_force_mps2=attitude(t,rotating).conjugate()*Eigen::Vector3d(0,0,cfg.imu.gravity_mps2);
  s.angular_velocity_radps.setZero();
  if(rotating)s.angular_velocity_radps=Eigen::Vector3d(1.,2.,3.).normalized()*1.2;
  return s;
}
inline UwbBatch ranges(const IntegrityConfig& cfg,int epoch,bool rotating=false,double anchor_fault=0.,bool flat=false) {
  UwbBatch b;b.id=BatchId(epoch);b.timestamp=TimestampNs(epoch*50000000LL);
  for(std::size_t i=0;i<cfg.anchors.size();++i) {
    UwbMeasurement m;m.id=MeasurementId(epoch*100+i+1);m.factor_id=FactorId(epoch*100+i+1);
    m.anchor_id=cfg.anchors[i].id;m.timestamp=b.timestamp;m.anchor_position_m=cfg.anchors[i].position_world_m;
    if(flat)m.anchor_position_m.z()=1.1;
    m.range_m=(m.anchor_position_m-Eigen::Vector3d(0,0,1)).norm()+(i==0?anchor_fault:0.);
    m.sigma_m=cfg.realtime.range_sigma_m;b.measurements.push_back(m);
  }
  return b;
}
inline NavigationState initialState() { NavigationState s;s.position_world_m={0,0,1};return s; }
inline EpochTransaction prepare(IncrementalUwbImuEstimator& estimator,const IntegrityConfig& cfg,int epoch,bool rotating=false) {
  for(int i=(epoch-1)*10+1;i<=epoch*10;++i)estimator.ingestImu(sample(cfg,i,rotating));
  return estimator.prepareEpoch(ranges(cfg,epoch,rotating));
}
inline Eigen::Vector3d researchVector(const YAML::Node& n){return {n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};}
inline YAML::Node researchLaw(const std::string& path){
 auto node=YAML::LoadFile(path);
 if(!node["raw_base_law"])return node;
 auto base=researchLaw(node["raw_base_law"].as<std::string>());
 for(auto item:node)base[item.first.as<std::string>()]=item.second;
 return base;
}
struct NoisyMotionScenario {
 YAML::Node law;IntegrityConfig cfg;Eigen::Vector3d lever,amp,freq;
 std::vector<ImuMeasurement> imu;std::vector<UwbBatch> uwb;
 std::vector<RawRangeLawEvidence> range_certificates;
 int epochs;double dt,rate;
 NavigationState initial_observation;
 bool initial_law_valid=false,physical_source_reference=false;
 RawInputRoundoff numeric_certificate,imu_fault_numeric_certificate;
 std::vector<Eigen::Matrix<double,6,1>> true_bias;

 NavigationState truth(double t)const{
  NavigationState s;s.timestamp=TimestampNs(std::llround(t*1e9));
  s.position_world_m=amp.cwiseProduct((freq*t).array().sin().matrix())+Eigen::Vector3d(0,0,1);
  s.velocity_world_mps=amp.cwiseProduct(freq).cwiseProduct((freq*t).array().cos().matrix());
  s.q_world_body=Eigen::Quaterniond(Eigen::AngleAxisd(rate*t,Eigen::Vector3d(1,2,3).normalized()));
  if(!true_bias.empty()){size_t i=size_t(std::llround(t/.005));if(i>=true_bias.size())throw std::runtime_error("bias truth outside registered sample grid");s.accel_bias_mps2=true_bias[i].head<3>();s.gyro_bias_radps=true_bias[i].tail<3>();}return s;
 }
 NoisyMotionScenario(const std::string& path,const std::string& kind,int seed,int count):law(researchLaw(path)),cfg(IntegrityConfigLoader::load(law[(kind=="I1" || kind=="I2" || kind=="screen" || kind.rfind("SC2_",0)==0 || kind.rfind("SC3_",0)==0)?"imu_base_config":"base_config"].as<std::string>())){
  epochs=count?count:law["epochs"].as<int>();dt=law["epoch_s"].as<double>();rate=law["angular_rate_radps"].as<double>();
  if(dt!=.05 || law["imu_sample_s"].as<double>()!=.005 || epochs>law["epochs"].as<int>())throw std::runtime_error("unsupported timestamp/horizon contract");
  lever=researchVector(law["lever"]);amp=researchVector(law["translation_amplitude"]);freq=researchVector(law["translation_frequency"]);
  cfg.incremental.fixed_lag_epochs=law["fixed_lag_epochs"].as<int>();cfg.integrity_window.epochs=law["window_epochs"].as<int>();
  cfg.realtime.range_sigma_m=law["range_sigma_m"].as<double>();
  Eigen::Matrix<double,15,1> initial_normals=Eigen::Matrix<double,15,1>::Zero();
  std::vector<Eigen::Matrix<double,6,1>> bias_normals,adc_normals,additive_imu_fault;
  physical_source_reference=law["source_law_id"] && law["source_law_id"].as<std::string>()=="raw-source-rw-v1";
  if(physical_source_reference){
   if(law["bias_process"].as<std::string>()!="sample-grid-gaussian-rw" || !law["nominal_weights_are_metric"].as<bool>())throw std::runtime_error("unsupported primitive law");
   std::mt19937 bias_rng(seed+23497);std::normal_distribution<double> bias_normal(0,1);
   Eigen::Matrix<double,6,1> bias;bias.head<3>()=researchVector(law["true_initial_accel_bias_mps2"]);bias.tail<3>()=researchVector(law["true_initial_gyro_bias_radps"]);
   true_bias.push_back(bias);
   for(int i=1;i<=epochs*10;++i){Eigen::Matrix<double,6,1> draw;for(int axis=0;axis<6;++axis){draw(axis)=bias_normal(bias_rng);bias(axis)+=(axis<3?cfg.imu.accelerometer_bias_rw_sigma:cfg.imu.gyroscope_bias_rw_sigma)*std::sqrt(.005)*draw(axis);}bias_normals.push_back(draw);true_bias.push_back(bias);}
  }
  initial_observation=truth(0);
  initial_law_valid=law["initial_observation_law"] && law["initial_observation_law"].as<std::string>()=="independent-tangent-gaussian-v1";
  if(initial_law_valid){
   std::mt19937 initial_rng(seed+15401);std::normal_distribution<double> initial_normal(0,1);
   Eigen::Matrix<double,15,1> eta;for(int k=0;k<15;++k){initial_normals(k)=initial_normal(initial_rng);eta(k)=cfg.realtime.prior_sigmas(k)*initial_normals(k);}
   gtsam::Pose3 true_pose(gtsam::Rot3(initial_observation.q_world_body.toRotationMatrix()),initial_observation.position_world_m);
   auto observed=true_pose.retract(eta.head<6>());
   initial_observation.q_world_body=Eigen::Quaterniond(observed.rotation().matrix());initial_observation.position_world_m=observed.translation();
   initial_observation.velocity_world_mps+=eta.segment<3>(6);
   initial_observation.accel_bias_mps2+=eta.segment<3>(9);initial_observation.gyro_bias_radps+=eta.tail<3>();
  }else initial_observation.position_world_m+=researchVector(law["initial_position_error_m"]);
  std::mt19937 imu_rng(seed),uwb_rng(seed+7919);std::normal_distribution<double> normal(0,1);
  for(int i=0;i<=epochs*10;++i){double t=i*.005;auto state=truth(t);auto m=sample(cfg,i);
   Eigen::Vector3d acc=-amp.cwiseProduct(freq.cwiseProduct(freq)).cwiseProduct((freq*t).array().sin().matrix());
   m.specific_force_mps2=state.q_world_body.conjugate()*(acc+Eigen::Vector3d(0,0,cfg.imu.gravity_mps2));
   m.angular_velocity_radps=rate*Eigen::Vector3d(1,2,3).normalized();
   m.specific_force_mps2+=state.accel_bias_mps2;m.angular_velocity_radps+=state.gyro_bias_radps;
   Eigen::Matrix<double,6,1> draw;for(int j=0;j<3;++j){draw(j)=normal(imu_rng);draw(j+3)=normal(imu_rng);m.specific_force_mps2(j)+=law["raw_accel_sigma_mps2"].as<double>()*draw(j);m.angular_velocity_radps(j)+=law["raw_gyro_sigma_radps"].as<double>()*draw(j+3);}adc_normals.push_back(draw);
   Eigen::Matrix<double,6,1> fault=Eigen::Matrix<double,6,1>::Zero();
   const int fault_epoch=law["fault_epoch"].as<int>();
   const double begin=(fault_epoch-1)*dt+.005;
   const double duration=kind=="I1"?.05:.5;
   if((kind=="I1" || kind=="I2") && t>=begin-1e-12 && t<begin+duration-1e-12)
    {fault(0)=law["imu_accel_amplitude_mps2"].as<double>();m.specific_force_mps2.x()+=fault(0);}
   if(kind.rfind("SC2_",0)==0 || kind=="SC3_A" || kind=="SC3_G" || kind=="SC3_IA" || kind=="SC3_IG" || kind=="SC3_STAGGERED"){int onset=law["sc2_witness_onset_epoch"].as<int>();long long begin=(onset-1)*50000000LL+5000000LL;long long end=begin+(kind=="SC2_S"?50:law["sc2_witness_duration_ms"].as<int>())*1000000LL;
    if(m.timestamp.value()>=begin && m.timestamp.value()<end){int axis=(kind=="SC2_G" || kind=="SC3_G" || kind=="SC3_IG")?law["sc2_gyro_witness_axis"].as<int>():law["sc2_accel_witness_axis"].as<int>();double amplitude=(kind=="SC2_G" || kind=="SC3_G" || kind=="SC3_IG")?law["sc2_gyro_witness_amplitude_radps"].as<double>():kind=="SC2_S"?law["sc2_short_witness_amplitude_mps2"].as<double>():law["sc2_accel_witness_amplitude_mps2"].as<double>();fault(axis)=amplitude;if(axis<3)m.specific_force_mps2(axis)+=amplitude;else m.angular_velocity_radps(axis-3)+=amplitude;}
   }
   additive_imu_fault.push_back(fault);
   imu.push_back(m);
  }
  if(physical_source_reference)numeric_certificate=certifyRawGenerator(initial_observation,initial_normals,cfg.realtime.prior_sigmas,amp,freq,rate,cfg.imu.gravity_mps2,true_bias.front(),cfg.imu.accelerometer_bias_rw_sigma,cfg.imu.gyroscope_bias_rw_sigma,law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),bias_normals,adc_normals,imu);
  if(physical_source_reference && (kind.rfind("SC2_",0)==0 || kind.rfind("SC3_",0)==0))imu_fault_numeric_certificate=certifyRawGenerator(initial_observation,initial_normals,cfg.realtime.prior_sigmas,amp,freq,rate,cfg.imu.gravity_mps2,true_bias.front(),cfg.imu.accelerometer_bias_rw_sigma,cfg.imu.gyroscope_bias_rw_sigma,law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),bias_normals,adc_normals,imu,&additive_imu_fault,&true_bias);
  else imu_fault_numeric_certificate=numeric_certificate;
  for(int e=1;e<=epochs;++e){auto b=ranges(cfg,e);auto s=truth(e*dt);Eigen::Vector3d tag=s.position_world_m+s.q_world_body*lever;
   std::array<double,8> range_draws{},faults{};size_t row=0;
   for(auto& m:b.measurements){if(kind=="U3")m.anchor_position_m.z()=1.1;m.range_m=(m.anchor_position_m-tag).norm()+cfg.realtime.range_sigma_m*(range_draws[row++]=normal(uwb_rng));}
   const int f=law["fault_epoch"].as<int>(),end=f+(kind=="U2"?law["persistent_epochs"].as<int>():1);
   if((kind=="U1" || kind=="U2") && e>=f && e<end){size_t index=law["fault_anchor_index"].as<int>();faults[index]=law["uwb_amplitude_m"].as<double>();b.measurements.at(index).range_m+=faults[index];}
   if(kind=="SC3_A" || kind=="SC3_G" || kind=="SC3_U1" || kind=="SC3_U2" || kind=="SC3_STAGGERED"){
    int onset=law[kind=="SC3_STAGGERED"?"sc3_staggered_uwb_onset_epoch":"sc3_uwb_witness_onset_epoch"].as<int>();int duration=kind=="SC3_U1"?1:law["sc3_uwb_witness_duration_epochs"].as<int>();
    if(e>=onset && e<onset+duration){size_t index=law["sc3_uwb_witness_anchor_index"].as<int>();faults[index]=law["sc3_uwb_witness_amplitude_m"].as<double>();b.measurements.at(index).range_m+=faults[index];}
   }
   if(kind=="H1" && e==3){faults[0]=.02;b.measurements[0].range_m+=.02;}
   uwb.push_back(b);
   if(physical_source_reference)range_certificates.push_back(certifyRawRanges(b,amp,freq,rate,lever,range_draws,faults));
  }
 }
};

} // namespace
