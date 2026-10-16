#pragma once
#include "rolling_raw.hpp"
#include <limits>
#include <set>

namespace uwb_imu_pl::research {
// Outward conversion also covers nanosecond differences larger than 2^53;
// neither signed subtraction nor epoch*period arithmetic is used.
inline RollingInterval durationSeconds(TimestampNs begin,TimestampNs end){
 if(!(begin<end))throw std::invalid_argument("NONPOSITIVE_PHYSICAL_INTERVAL");
 const uint64_t ns=uint64_t(end.value())-uint64_t(begin.value());
 const double d=static_cast<double>(ns);
 RollingInterval n=ns<=(uint64_t(1)<<53)?RollingInterval(d):
  RollingInterval(std::nextafter(d,-INFINITY),std::nextafter(d,INFINITY));
 return n/RollingInterval(1000000000.);
}
struct RawTimeStep {
 RollingInterval duration;
 std::array<RollingInterval,6> bias_radius;
};
// q per actual RW coordinate (not per interpolation/hold endpoint). Endpoint
// references do not create a new Brownian increment. The declared law is a
// sample-grid random walk, whose support sums sigma*q*sqrt(actual_dt).
inline RawTimeStep rawTimeStep(TimestampNs begin,TimestampNs end,TimestampNs previous_raw_stamp,
 const std::vector<TimestampNs>& births,const std::vector<std::array<double,6>>& q,
 double accel_rw,double gyro_rw){
 if(births.size()!=q.size() || !std::isfinite(accel_rw) || !std::isfinite(gyro_rw) || accel_rw<0 || gyro_rw<0)
  throw std::invalid_argument("RW_LAYOUT_INVALID");
 RawTimeStep out;out.duration=durationSeconds(begin,end);for(auto& r:out.bias_radius)r=RollingInterval(0);
 if(begin<previous_raw_stamp)throw std::invalid_argument("RW_PARENT_AFTER_BEGIN");
 auto previous=previous_raw_stamp;
 for(size_t i=0;i<births.size();++i){
  if(!(begin<births[i]) || end<births[i])throw std::invalid_argument("RW_SOURCE_OUTSIDE_INTERVAL");
  auto dt=durationSeconds(previous,births[i]);
  for(int axis=0;axis<6;++axis){
   if(!std::isfinite(q[i][axis]) || q[i][axis]<=0)throw std::invalid_argument("RW_BIRTH_RADIUS_INVALID");
   out.bias_radius[axis]+=RollingInterval(q[i][axis])*RollingInterval(axis<3?accel_rw:gyro_rw)*sqrt(dt);
  }
  previous=births[i];
 }
 return out;
}
// Endpoint velocity has an explicit source: the current raw state domain.
// Reverse integration gives |p(t)-p(T)| <= |v(T)|h+A*h²/2. Rotation of the
// fixed lever contributes <= |lever|*Omega*h. Reverse triangle inequality
// transports the range shell without asserting another Gaussian draw.
inline RollingInterval rangeTransportRadius(const RollingRawState& endpoint,
 TimestampNs measured,TimestampNs epoch,const Eigen::Vector3d& lever,double acceleration,double angular_rate){
 if(epoch<measured || !lever.allFinite() || !std::isfinite(acceleration) || acceleration<0 ||
    !std::isfinite(angular_rate) || angular_rate<0)throw std::invalid_argument("RANGE_TRANSPORT_DOMAIN");
 if(measured==epoch)return RollingInterval(0);
 using I=RollingInterval;const I dt=durationSeconds(measured,epoch);
 I v2(0),l2(0);for(int j=0;j<3;++j){
  if(!std::isfinite(endpoint.v[j].lower()) || !std::isfinite(endpoint.v[j].upper()))
   throw std::invalid_argument("RANGE_TRANSPORT_VELOCITY_DOMAIN_MISSING");
  v2+=square(endpoint.v[j]);l2+=square(I(lever[j]));
 }
 I radius=I(sqrt(v2).upper())*dt+I(acceleration)*square(dt)/2.+sqrt(l2)*I(angular_rate)*dt;
 if(!std::isfinite(radius.upper()))throw std::overflow_error("RANGE_TRANSPORT_OVERFLOW");
 return I(0,radius.upper());
}
}
