#pragma once
#include "open_fault_pool.hpp"
#include "sc6_motion.hpp"
namespace uwb_imu_pl::research {
// A necessary relation between TWO arrived physical samples. It supplements
// functional FD only. The original raw fault pool and boundary union are kept.
// On E, h_a=R^T(a+g) has derivative norm <= J+Omega(A+g), and body omega
// derivative <=alpha. Initial bias cancels; the single intervening RW source
// and both original (possibly shared-PIM) ADC coordinates retain birth q.
class Sc6TemporalMonitor {
 using I=RollingInterval;
 TimeUniformEvent event_;RawInclusionParameters law_;Sc6MotionDomain domain_;
 ImuMeasurement previous_;bool initialized_=false,pending_=false,enabled_=false;
 TimestampNs begin_,end_,first_alarm_;size_t capacity_;
 std::vector<OpenFaultRecord> records_;
 public:
 std::array<double,6> difference{},radius{},margin{};
 Sc6TemporalMonitor(const RawInclusionParameters& law,double budget,
                    const Sc6MotionDomain& domain,size_t capacity):event_(budget),law_(law),domain_(domain),capacity_(capacity){
  if(!std::isfinite(domain.jerk+domain.alpha) || domain.jerk<0 || domain.alpha<0 || !capacity)throw std::invalid_argument("TEMPORAL_DOMAIN_UNQUALIFIED");
 }
 bool append(const ImuMeasurement& m,bool pool_alarm=false,size_t pool_records=0){
  if(!m.specific_force_mps2.allFinite() || !m.angular_velocity_radps.allFinite())throw std::invalid_argument("TEMPORAL_NONFINITE_SOURCE");
  if(!initialized_){if(m.id.value()!=1 || m.timestamp.value()!=0)throw std::invalid_argument("TEMPORAL_INITIAL_SOURCE");previous_=m;initialized_=true;return false;}
  if(m.id.value()!=previous_.id.value()+1 || m.timestamp.value()-previous_.timestamp.value()!=5000000LL)throw std::invalid_argument("TEMPORAL_SOURCE_GAP");
  const uint64_t index=uint64_t(m.timestamp.value()/5000000LL);
  if(pending_ && end_<m.timestamp){pending_=false;enabled_=false;}
  bool violated=false;
  for(int axis=0;axis<6;++axis){bool accel=axis<3;I dt=I(1)/200.;
   I derivative=accel?I(domain_.jerk)+I(domain_.rate)*(I(domain_.acceleration)+I(law_.gravity)):I(domain_.alpha);
   I r=derivative*dt+(I(event_.adcRadius(index))+I(event_.adcRadius(index-1)))*I(accel?law_.accel_adc:law_.gyro_adc)
       +I(event_.adcRadius(index))*I(accel?law_.accel_rw:law_.gyro_rw)*sqrt(dt)+I(4e-10);
   I delta=I(accel?m.specific_force_mps2(axis):m.angular_velocity_radps(axis-3))-
           I(accel?previous_.specific_force_mps2(axis):previous_.angular_velocity_radps(axis-3));
   difference[axis]=accel?m.specific_force_mps2(axis)-previous_.specific_force_mps2(axis):m.angular_velocity_radps(axis-3)-previous_.angular_velocity_radps(axis-3);
   radius[axis]=r.upper();margin[axis]=radius[axis]-abs(delta).lower();
   violated|=delta.lower()>radius[axis] || delta.upper()<-radius[axis];
  }
  previous_=m;
  if(!violated || pending_)return false;
  // Constant-axis episodes can change their indicator only on this grid.
  // An unexplained jump elsewhere violates the original contract; never
  // invent a new fault grid to accommodate it.
  if(index%10!=1)throw std::invalid_argument("TEMPORAL_FAULT_GRID_OUTSIDE_SCOPE");
  // Both a NEW episode and an ENDING episode explain a jump. Preserve the
  // union of 50/500ms supports; no truth-based onset/axis choice is made.
  const int64_t now=m.timestamp.value();int64_t earliest=now;for(int duration:{50,500})if(now-int64_t(duration)*1000000LL>=5000000LL)earliest=std::min<int64_t>(earliest,now-int64_t(duration)*1000000LL);begin_=TimestampNs(earliest);end_=TimestampNs(now+500000000LL);
  pending_=true;enabled_=!pool_alarm;
  if(!enabled_)return false; // already covered by the actual absolute FD
  if(records_.size()+pool_records>=capacity_)throw std::invalid_argument("TEMPORAL_FAULT_IDENTITY_RESOURCE_EXHAUSTED");
  first_alarm_=m.timestamp;records_.push_back({records_.size()+1,begin_,end_,"sc6-temporal-observed:"+std::to_string(m.id.value())});
  return true;
 }
 bool alarm()const{return pending_ && enabled_;}
 TimestampNs begin()const{return begin_;}TimestampNs end()const{return end_;}
 TimestampNs firstAlarm()const{return first_alarm_;}
 const std::vector<OpenFaultRecord>& records()const{return records_;}
};
}
