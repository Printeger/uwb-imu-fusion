#include "empirical_sensor_adapter.hpp"
#include "measurement_bootstrap.hpp"
#include "numerical_candidate.hpp"
#include "native_semantic_audit.hpp"

namespace uwb_imu_pl::research {
namespace {
Eigen::Vector3d vector(const YAML::Node& n){
 if(n.size()!=3)throw std::runtime_error("MODEL_VECTOR_SIZE");
 return {n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};
}
}
EmpiricalSetup empiricalSetup(const YAML::Node& model){
 if(model["status"].as<std::string>()!="DECLARED_EMPIRICAL")throw std::invalid_argument("DATASET_REQUIRES_DECLARED_EMPIRICAL_MODEL");
 EmpiricalSetup out;out.model=YAML::Clone(model);out.law=researchLaw("config/sc7.yaml");
 out.config=IntegrityConfigLoader::load(out.law["imu_base_config"].as<std::string>());out.config.anchors.clear();
 out.policy=resolveRollingPolicy(out.law,out.config);
 auto& rawlaw=out.raw_law;auto& cfg=out.config;auto& domain=out.domain;
 rawlaw.gravity=model["gravity_mps2"].as<double>();rawlaw.accel_adc=model["accel_adc_sigma"].as<double>();rawlaw.gyro_adc=model["gyro_adc_sigma"].as<double>();rawlaw.accel_rw=model["accel_rw_sigma"].as<double>();rawlaw.gyro_rw=model["gyro_rw_sigma"].as<double>();
 domain.id=model["model_id"].as<std::string>();domain.acceleration=model["motion"]["acceleration"].as<double>();domain.rate=model["motion"]["rate"].as<double>();domain.jerk=model["motion"]["jerk"].as<double>();domain.alpha=model["motion"]["alpha"].as<double>();
 rawlaw.acceleration=domain.acceleration;rawlaw.rate=domain.rate;rawlaw.jerk=domain.jerk;
 cfg.imu.gravity_mps2=rawlaw.gravity;cfg.imu.accelerometer_sigma=rawlaw.accel_adc;cfg.imu.gyroscope_sigma=rawlaw.gyro_adc;
 cfg.imu.accelerometer_bias_rw_sigma=rawlaw.accel_rw;cfg.imu.gyroscope_bias_rw_sigma=rawlaw.gyro_rw;cfg.imu.max_gap_s=.05;
 cfg.bridge.generic.acceleration_bound_mps2=domain.acceleration;cfg.bridge.generic.angular_rate_bound_radps=domain.rate;cfg.bridge.generic.angular_acceleration_bound_radps2=domain.alpha;cfg.bridge.generic.calibration_id=domain.id;
 for(int j=0;j<3;++j)for(int k=0;k<3;++k)out.world(j,k)=model["R_model_from_anchor_world"][j][k].as<double>();
 if((out.world.transpose()*out.world-Eigen::Matrix3d::Identity()).norm()>1e-10 || out.world.determinant()<.999999)throw std::invalid_argument("MODEL_WORLD_ROTATION");
 out.range_clock_offset=model["range_to_imu_time_s"].as<double>();
 if(!std::isfinite(out.range_clock_offset))throw std::invalid_argument("MODEL_CLOCK_INVALID");
 return out;
}
EmpiricalBootstrap empiricalBootstrap(const EmpiricalSetup& setup,AnchorRegistry anchors,
 const std::vector<ImuMeasurement>& prefix,const std::vector<CanonicalRangeEvent>& potentials,UwbBatch ranges){
 if(prefix.empty() || ranges.measurements.size()<4 || prefix.back().timestamp<ranges.timestamp)
  throw std::invalid_argument("CAUSAL_STARTUP_MATERIAL_MISSING");
 const auto& model=setup.model;const auto& cfg=setup.config;const auto& domain=setup.domain;const auto& lever=setup.lever;
 GeneralizedCoreInitialization initialization;initialization.raw_law=setup.raw_law;initialization.anchors=std::move(anchors);
 initialization.sources=std::make_unique<GeneralizedSourceLedger>(setup.policy.budget,setup.raw_law,ModelEvidenceStatus::DeclaredEmpirical);
 SemanticDigest digest;digest.s("sc9-arrived-bootstrap-v1");digest.s(YAML::Dump(model));digest.s(YAML::Dump(setup.law));
 digest.u(prefix.size());Eigen::Vector3d mean_force=Eigen::Vector3d::Zero();
 for(const auto& m:prefix){initialization.sources->append(m);mean_force+=m.specific_force_mps2;digest.imu(m);}mean_force/=prefix.size();
 digest.u(potentials.size());for(const auto& event:potentials){initialization.sources->registerPotentialRange(event);digest.range(event.measurement);digest.u(event.received.value());digest.u(event.calibration_version);digest.u(event.valid);}
 digest.u(ranges.id.value());digest.u(ranges.timestamp.value());digest.u(ranges.measurements.size());for(const auto& m:ranges.measurements)digest.range(m);
 ranges.timestamp=prefix.back().timestamp;
 RollingRawState physical;const double speed=model["initial_velocity_radius"].as<double>();
 EmpiricalBootstrap out;auto& numerical=out.numerical;numerical.timestamp=prefix.back().timestamp;numerical.accel_bias_mps2=vector(model["accel_bias_mps2"]);numerical.gyro_bias_radps=vector(model["gyro_bias_radps"]);
 for(int j=0;j<3;++j){physical.p[j]=RollingInterval(0);physical.v[j]=RollingInterval(-speed,speed);for(int k=0;k<3;++k)physical.rotation[j][k]=RollingInterval(-1,1);}
 for(int j=0;j<6;++j){const double center=j<3?numerical.accel_bias_mps2[j]:numerical.gyro_bias_radps[j-3],radius=model["initial_bias_radius"][j].as<double>();physical.bias[j]=RollingInterval(center)+RollingInterval(-radius,radius);}
 initialization.initial_set=measurementInitial(*initialization.sources,ranges,physical,lever,domain.acceleration,domain.rate,ModelEvidenceStatus::DeclaredEmpirical,domain.id);
 const Eigen::Vector3d specific=mean_force-numerical.accel_bias_mps2;if(specific.norm()>1e-6)numerical.q_world_body=Eigen::Quaterniond::FromTwoVectors(specific,Eigen::Vector3d::UnitZ());
 auto seed=numericalUwbCandidate(ranges,-1).functional.tag;
 if(!seed.allFinite()){seed.setZero();for(const auto& m:ranges.measurements)seed+=m.anchor_position_m;seed/=ranges.measurements.size();}
 numerical.position_world_m=seed-numerical.q_world_body*lever;
 out.core=std::make_unique<UnifiedRollingIntegrityCore>(setup.law,cfg,numerical,lever,domain,prefix.back(),std::move(initialization));
 out.core->enableSemanticAudit();out.source_digest=digest.finish();return out;
}
void CausalEmpiricalStream::appendImu(const ImuMeasurement& m,TimestampNs received)try{
 available();
 if(received<m.timestamp)throw std::invalid_argument("STREAM_FUTURE_IMU");
 if(last_imu_ && !(last_imu_.value()<m.timestamp))throw std::invalid_argument("STREAM_IMU_ORDER");
 if(last_imu_ && (uint64_t(m.timestamp.value())-uint64_t(last_imu_->value()))/1000000000.L>setup_.config.imu.max_gap_s)
  throw std::invalid_argument("STREAM_IMU_GAP");
 if(!m.specific_force_mps2.allFinite() || !m.angular_velocity_radps.allFinite())throw std::invalid_argument("STREAM_IMU_NONFINITE");
 if(imu_.size()>=imu_capacity_)throw std::length_error("STREAM_IMU_CAPACITY");
 imu_.push_back(m);last_imu_=m.timestamp;if(!latest_received_ || *latest_received_<received)latest_received_=received;++imu_received_;
}catch(...){failed_=true;throw;}
void CausalEmpiricalStream::appendBatch(EmpiricalBatch batch)try{
 available();
 if(last_batch_ && !(last_batch_.value()<batch.values.timestamp))throw std::invalid_argument("STREAM_RANGE_ORDER");
 if(batches_.size()>=batch_capacity_)throw std::length_error("STREAM_BATCH_CAPACITY");
 TimestampNs ready=batch.values.timestamp;for(const auto& event:batch.events)ready=std::max(ready,event.received);
 validateRanges(batch.events,anchors_,ready);
 UwbBatch selected;selected.id=batch.values.id;selected.timestamp=batch.values.timestamp;
 for(const auto& event:batch.events)if(event.valid)selected.measurements.push_back(event.measurement);
 if(!sameRawBatch(selected,batch.values))throw std::invalid_argument("STREAM_CANONICAL_BATCH_MISMATCH");
 if(!latest_received_ || *latest_received_<ready)latest_received_=ready;last_batch_=batch.values.timestamp;
 batches_.push_back(std::move(batch));++batches_received_;
}catch(...){failed_=true;throw;}
std::optional<EmpiricalEmission> CausalEmpiricalStream::next(bool actions)try{
 available();
 auto retainStartup=[&](const EmpiricalBatch& batch){
  if(batch.events.size()>startup_range_capacity_-startup_ranges_.size())throw std::length_error("STREAM_STARTUP_RANGE_CAPACITY");
  startup_ranges_.insert(startup_ranges_.end(),batch.events.begin(),batch.events.end());++startup_batches_;
 };
 if(!bootstrap_.core){
  if(imu_.empty())return std::nullopt;
  while(!startup_batch_ && !batches_.empty()){
   auto batch=std::move(batches_.front());batches_.pop_front();retainStartup(batch);
   const bool warm=!(batch.values.timestamp<imu_.front().timestamp) &&
    uint64_t(batch.values.timestamp.value())-uint64_t(imu_.front().timestamp.value())>=500000000ULL;
   if(warm && batch.values.measurements.size()>=4)startup_batch_=std::move(batch);
  }
  if(!startup_batch_)return std::nullopt;
  auto first=std::lower_bound(imu_.begin(),imu_.end(),startup_batch_->values.timestamp,[](const auto& m,TimestampNs time){return m.timestamp<time;});
  if(first==imu_.end())return std::nullopt; // Do not invent the right-hand sample.
  const size_t consumed=std::distance(imu_.begin(),first)+1;
  std::vector<ImuMeasurement> prefix(imu_.begin(),std::next(first));
  // High-rate ranges between the selected startup batch and its first actual
  // IMU endpoint are startup potential sources, not silently discarded inputs.
  while(!batches_.empty() && !(prefix.back().timestamp<batches_.front().values.timestamp)){
   retainStartup(batches_.front());batches_.pop_front();
  }
  bootstrap_=empiricalBootstrap(setup_,anchors_,prefix,startup_ranges_,startup_batch_->values);
  last_used_imu_=prefix.back().timestamp;for(size_t k=0;k<consumed;++k)imu_.pop_front();
  startup_batch_.reset();startup_ranges_.clear();startup_ranges_.shrink_to_fit();
 }
 if(batches_.empty() || !last_imu_ || *last_imu_<batches_.front().values.timestamp)return std::nullopt;
 auto batch=std::move(batches_.front());batches_.pop_front();
 Sc6Envelope input;input.epoch=epochs_+1;input.uwb=std::move(batch.values);input.range_events=std::move(batch.events);input.ready=*latest_received_;
 while(!imu_.empty() && !(input.uwb.timestamp<imu_.front().timestamp)){
  last_used_imu_=imu_.front().timestamp;input.imu.push_back(std::move(imu_.front()));imu_.pop_front();
 }
 EmpiricalEmission out;out.batch=input.uwb.id;out.latest_original_used=last_used_imu_;
 if(batch.latest_original_valid_range)out.latest_original_used=std::max(out.latest_original_used,*batch.latest_original_valid_range);
 out.result=bootstrap_.core->step(input,actions);++epochs_;return out;
}catch(...){failed_=true;throw;}
}
