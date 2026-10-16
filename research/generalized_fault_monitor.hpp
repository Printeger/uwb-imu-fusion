#pragma once
#include "generalized_source_ledger.hpp"
#include "open_fault_pool.hpp"
#include "sc6_motion.hpp"

namespace uwb_imu_pl::research {
// Functional necessary-condition monitor on original, irregularly timed
// samples. It never certifies sample 0 healthy and never prunes the raw union.
// The ambiguous beginning/ending-jump support covers every phase of episodes
// up to the configured 500 ms duration; values cannot select a narrower phase.
class GeneralizedFaultMonitor {
 using I=RollingInterval;
 JointRawLaw law_;Sc6MotionDomain domain_;
 std::array<I,6> bias_;
 std::optional<ImuMeasurement> previous_;
 std::vector<OpenFaultRecord> records_;
 TimestampNs begin_,end_,first_alarm_;
 size_t capacity_;bool active_=false;
 OpenImuDiagnostic diagnostic_;
 static TimestampNs shifted(TimestampNs stamp,int64_t delta){
  if((delta>0 && stamp.value()>INT64_MAX-delta) || (delta<0 && stamp.value()<INT64_MIN-delta))
   throw std::overflow_error("FAULT_SUPPORT_TIME_OVERFLOW");
  return TimestampNs(stamp.value()+delta);
 }
 public:
 std::array<double,6> difference{},radius{},margin{};
 GeneralizedFaultMonitor(const JointRawLaw& law,const Sc6MotionDomain& domain,
                         const InitialSetEvidence& initial,size_t capacity)
  :law_(law),domain_(domain),bias_(initial.domain.bias),capacity_(capacity){
  if(!capacity || !std::isfinite(domain.jerk+domain.alpha+domain.acceleration+domain.rate) ||
     std::min({domain.jerk,domain.alpha,domain.acceleration,domain.rate})<0)
   throw std::invalid_argument("GENERALIZED_MONITOR_DOMAIN");
 }
 bool append(const ImuMeasurement& m,const SourceLayoutEvent& event,uint64_t sensor=0){
  if(previous_ && !(previous_->timestamp<m.timestamp))throw std::invalid_argument("MONITOR_ORIGINAL_TIME_ORDER");
  if(active_ && !(m.timestamp<end_))active_=false;
  const I dt=previous_?durationSeconds(previous_->timestamp,m.timestamp):I(0);
  bool violated=false;
  for(unsigned a=0;a<6;++a){const bool accel=a<3;
   const double q=event.birth({SourceKind::Adc,sensor,m.id.value(),a}).q;
   const I adc=I(q)*I(accel?law_.accel_adc:law_.gyro_adc);
   I rw(0);if(previous_){const double qr=event.birth({SourceKind::BiasIncrement,sensor,m.id.value(),a}).q;
    rw=I(qr)*I(accel?law_.accel_rw:law_.gyro_rw)*sqrt(dt);bias_[a]+=I(-rw.upper(),rw.upper());}
   const I physical=accel?I(domain_.acceleration)+I(law_.gravity):I(domain_.rate);
   const I healthy=bias_[a]+I(-physical.upper(),physical.upper())+I(-adc.upper(),adc.upper());
   const double y=accel?m.specific_force_mps2[a]:m.angular_velocity_radps[a-3];
   diagnostic_.measured[a]=y;diagnostic_.healthy_low[a]=healthy.lower();diagnostic_.healthy_high[a]=healthy.upper();diagnostic_.rw_radius[a]=rw.upper();diagnostic_.q=std::max(diagnostic_.q,q);
   bool axis_violation=y<healthy.lower() || y>healthy.upper();
   margin[a]=INFINITY;difference[a]=radius[a]=0;
   if(previous_){const double oldq=event.birth({SourceKind::Adc,sensor,previous_->id.value(),a}).q;
    const I derivative=accel?I(domain_.jerk)+I(domain_.rate)*(I(domain_.acceleration)+I(law_.gravity)):I(domain_.alpha);
    const I r=derivative*dt+adc+I(oldq)*I(accel?law_.accel_adc:law_.gyro_adc)+rw+I(4e-10);
    const I delta=I(y)-I(accel?previous_->specific_force_mps2[a]:previous_->angular_velocity_radps[a-3]);
    difference[a]=y-(accel?previous_->specific_force_mps2[a]:previous_->angular_velocity_radps[a-3]);radius[a]=r.upper();margin[a]=r.upper()-abs(delta).lower();
    axis_violation|=delta.lower()>r.upper() || delta.upper()<-r.upper();
   }
   violated|=axis_violation;
  }
  const auto support_begin=shifted(previous_?previous_->timestamp:m.timestamp,-500000000LL);
  const auto support_end=shifted(m.timestamp,500000000LL);
  previous_=m;diagnostic_.violation=violated;
  if(!violated){diagnostic_.no_active=!active_;return false;}
  if(active_){begin_=std::min(begin_,support_begin);end_=std::max(end_,support_end);diagnostic_.no_active=false;return false;}
  if(records_.size()>=capacity_)throw std::length_error("GENERALIZED_FAULT_RECORD_CAPACITY");
  active_=true;begin_=support_begin;end_=support_end;first_alarm_=m.timestamp;diagnostic_.no_active=false;
  records_.push_back({records_.size()+1,begin_,end_,"sc9-observed-original:"+std::to_string(m.id.value())});return true;
 }
 RollingMonitorView view(const EpochTransaction& tx,const SourceLayoutEvent& event)const{
  RollingMonitorView out;out.open=true;out.event_id=event.id();out.alert_generation=records_.size();
  out.state.nominal_possible=!active_;out.state.possible=1;out.state.alarm=active_;
  out.state.union_begin=begin_;out.state.union_end=end_;out.state.first_alarm=first_alarm_;
  // An arbitrary-phase continuum has no finite exact branch census. The raw
  // contractor retains its per-sample all-axis outer union separately.
  const auto id=tx.id;const auto begin=tx.begin,end=tx.end;
  out.matches=[id,begin,end](const EpochTransaction& other){return other.id==id && other.begin==begin && other.end==end;};return out;
 }
 const OpenImuDiagnostic& diagnostic()const{return diagnostic_;}
 size_t alerts()const{return records_.size();}
};
}
