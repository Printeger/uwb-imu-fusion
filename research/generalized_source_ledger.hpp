#pragma once
#include "rolling_evidence.hpp"
#include "anchor_registry.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"

namespace uwb_imu_pl::research {
// Original sources are immutable. The persistent identity map is explicitly
// O(T), with a capacity refusal, rather than an asserted constant-memory proof.
// Noise/physical law qualification belongs to configuration admission, never
// to residual-driven selection or to a successful numerical estimate.
class GeneralizedSourceLedger {
 SourceLayoutEvent event_;
 JointRawLaw law_;
 ModelEvidenceStatus status_;
 std::map<MeasurementId,ImuMeasurement> raw_;
 std::map<MeasurementId,UwbMeasurement> ranges_;
 std::map<MeasurementId,std::tuple<AnchorId,FactorId,TimestampNs,uint64_t,bool>> potential_ranges_;
 std::optional<TimestampNs> latest_raw_;
 uint64_t sensor_;
 public:
 GeneralizedSourceLedger(double epsilon,const JointRawLaw& law,ModelEvidenceStatus status,
                         uint64_t sensor=0,size_t capacity=1000000)
  :event_(epsilon,capacity),law_(law),status_(status),sensor_(sensor){
  if(status==ModelEvidenceStatus::MissingOrInconsistent ||
     !std::isfinite(law.accel_adc+law.gyro_adc+law.accel_rw+law.gyro_rw) ||
     std::min({law.accel_adc,law.gyro_adc,law.accel_rw,law.gyro_rw})<0)
   throw std::invalid_argument("SOURCE_MODEL_MISSING");
 }
 const SourceLayoutEvent& event()const{return event_;}
 ModelEvidenceStatus status()const{return status_;}
 uint64_t sensor()const{return sensor_;}
 const JointRawLaw& law()const{return law_;}
 TimestampNs originalTime(MeasurementId id)const{return raw_.at(id).timestamp;}
 InitialSetEvidence independentInitial(const NavigationState& center,
    const Eigen::Matrix<double,15,1>& sigmas,const std::string& model){
  if(event_.coordinates() || model.empty())throw std::invalid_argument("INITIAL_BIRTH_ORDER");
  std::vector<SourceCoordinate> layout;
  for(unsigned axis=0;axis<15;++axis)layout.push_back({SourceKind::Initial,sensor_,0,axis});
  const auto b=event_.append(layout);
  InitialSetEvidence initial;initial.domain=rollingInitial(center,sigmas,b.q,1e-10);
  initial.timestamp=center.timestamp;initial.status=status_;initial.event_id=event_.id();initial.model_id=model;
  initial.source_prefix=event_.coordinates();initial.risk_upper=event_.prefixUpper();return initial;
 }
 void append(const ImuMeasurement& m){
  const auto old=raw_.find(m.id);
  if(old!=raw_.end()){
   if(old->second.quality_flags!=m.quality_flags || old->second.timestamp!=m.timestamp || old->second.specific_force_mps2!=m.specific_force_mps2 ||
      old->second.angular_velocity_radps!=m.angular_velocity_radps)throw std::invalid_argument("ORIGINAL_IMU_MUTATED");
   return;
  }
  if((latest_raw_ && !(latest_raw_.value()<m.timestamp)) || !m.specific_force_mps2.allFinite() || !m.angular_velocity_radps.allFinite())
   throw std::invalid_argument("ORIGINAL_IMU_ORDER");
  std::vector<SourceCoordinate> layout;
  for(unsigned axis=0;axis<6;++axis){layout.push_back({SourceKind::Adc,sensor_,m.id.value(),axis});
   if(latest_raw_)layout.push_back({SourceKind::BiasIncrement,sensor_,m.id.value(),axis});}
  event_.append(layout);raw_.emplace(m.id,m);latest_raw_=m.timestamp;
 }
 void registerPotentialRange(const CanonicalRangeEvent& e){
  const auto& m=e.measurement;const auto identity=std::make_tuple(m.anchor_id,m.factor_id,m.timestamp,e.calibration_version,e.valid);
  auto found=potential_ranges_.find(m.id);
  if(found!=potential_ranges_.end() && found->second!=identity)throw std::invalid_argument("POTENTIAL_RANGE_IDENTITY_CHANGED");
  event_.append({{SourceKind::Range,m.anchor_id.value(),m.id.value(),0}});
  potential_ranges_.emplace(m.id,identity);
  if(e.valid)append(m);
 }
 void append(const UwbMeasurement& m){
  const auto old=ranges_.find(m.id);
  if(old!=ranges_.end()){
   const auto& a=old->second;
   if(a.tag_id!=m.tag_id || a.sequence!=m.sequence || a.quality_flags!=m.quality_flags || a.timestamp!=m.timestamp || a.anchor_id!=m.anchor_id || a.range_m!=m.range_m || a.sigma_m!=m.sigma_m ||
      a.anchor_position_m!=m.anchor_position_m || a.factor_id!=m.factor_id || a.lever_arm_body_m!=m.lever_arm_body_m)
    throw std::invalid_argument("ORIGINAL_RANGE_MUTATED");
   return;
  }
  if(!std::isfinite(m.range_m) || !std::isfinite(m.sigma_m) || m.range_m<=0 || m.sigma_m<=0)
   throw std::invalid_argument("ORIGINAL_RANGE_INVALID");
  event_.append({{SourceKind::Range,m.anchor_id.value(),m.id.value(),0}});ranges_.emplace(m.id,m);
 }
 std::shared_ptr<const GeneralizedRawEvidence> evidence(const EpochTransaction& tx)const{
  using I=RollingInterval;
  if(tx.raw_imu_slice.size()<2)throw std::invalid_argument("ORIGINAL_IMU_SLICE_MISSING");
  auto out=std::make_shared<GeneralizedRawEvidence>();
  out->begin=tx.begin;out->end=tx.end;out->event_id=event_.id();out->status=status_;
  out->source_prefix=event_.coordinates();out->risk_upper=event_.prefixUpper();
  const auto& first=raw_.at(tx.raw_imu_slice.front().id);
  if(tx.raw_imu_slice.front().specific_force_mps2!=first.specific_force_mps2 || tx.raw_imu_slice.front().angular_velocity_radps!=first.angular_velocity_radps)throw std::invalid_argument("NATIVE_FIRST_PARENT_MISMATCH");
  if(tx.begin<first.timestamp)throw std::invalid_argument("ORIGINAL_IMU_PARENT_FUTURE");
  std::vector<TimestampNs> times;std::vector<std::array<double,6>> qs;
  MeasurementId previous=first.id;
  for(size_t i=1;i<tx.raw_imu_slice.size();++i){const auto& used=tx.raw_imu_slice[i];const auto& raw=raw_.at(used.id);
   if(raw.id!=previous){std::array<double,6> q;
    for(unsigned a=0;a<6;++a)q[a]=event_.birth({SourceKind::BiasIncrement,sensor_,raw.id.value(),a}).q;
    times.push_back(raw.timestamp);qs.push_back(q);previous=raw.id;
   }
  }
  out->evolution=rawTimeStep(tx.begin,tx.end,first.timestamp,times,qs,law_.accel_rw,law_.gyro_rw);
  for(size_t i=1;i<tx.raw_imu_slice.size();++i){const auto& used=tx.raw_imu_slice[i];const auto& raw=raw_.at(used.id);
   if(used.specific_force_mps2!=raw.specific_force_mps2 || used.angular_velocity_radps!=raw.angular_velocity_radps || used.timestamp<raw.timestamp)
    throw std::invalid_argument("NATIVE_IMU_ORIGINAL_MISMATCH");
   JointTimedSample sample;sample.raw=raw;sample.integration_end=used.timestamp;
   for(unsigned a=0;a<6;++a){const double q=event_.birth({SourceKind::Adc,sensor_,raw.id.value(),a}).q;
    sample.adc_radius[a]=I(q)*I(a<3?law_.accel_adc:law_.gyro_adc);out->max_coordinate_radius=std::max(out->max_coordinate_radius,q);
    // Full-interval support on both sides is a conservative relaxation. It
    // also includes the partial previous-grid increment at a held boundary.
    sample.rw_from_begin[a]=sample.rw_to_end[a]=out->evolution.bias_radius[a];
   }
   out->joint_samples.push_back(std::move(sample));
  }
  for(const auto& m:tx.uwb_batch.measurements){
   const auto& original=ranges_.at(m.id);
   if(original.timestamp!=m.timestamp || original.anchor_id!=m.anchor_id || original.range_m!=m.range_m || original.sigma_m!=m.sigma_m ||
      original.anchor_position_m!=m.anchor_position_m || original.factor_id!=m.factor_id)
    throw std::invalid_argument("NATIVE_RANGE_ORIGINAL_MISMATCH");
   auto potential=potential_ranges_.find(m.id);out->range_calibration_versions.push_back(potential==potential_ranges_.end()?0:std::get<3>(potential->second));
   const double q=event_.birth({SourceKind::Range,m.anchor_id.value(),m.id.value(),0}).q;
   out->range_noise_radius.push_back((I(q)*I(m.sigma_m)).upper());out->max_coordinate_radius=std::max(out->max_coordinate_radius,q);
  }
  return out;
 }
};
}
