#pragma once
#include "rolling_raw.hpp"
namespace uwb_imu_pl::research {
// Independent local prototype: scalar known-R, constant acceleration, 3 poses.
// This is NOT the production bias update. Each possible fault branch is kept.
struct Sc6BiasBranch {RollingInterval bias,acceleration,fault;bool feasible=true;};
inline Sc6BiasBranch sc6BiasConstraint(RollingInterval p0,RollingInterval p1,
 RollingInterval p2,double dt,RollingInterval y,RollingInterval noise,
 RollingInterval bias,RollingInterval rw,RollingInterval fault){
 if(!std::isfinite(dt) || dt<=0)throw std::invalid_argument("BIAS_PROTOTYPE_TIME");
 using I=RollingInterval;I acceleration=(p2-2.*p1+p0)/square(I(dt));
 I next=bias+rw;I inferred=y-acceleration-fault-noise;
 double l=std::max(next.lower(),inferred.lower()),h=std::min(next.upper(),inferred.upper());
 if(l>h)return {next,acceleration,fault,false};return {I(l,h),acceleration,fault,true};
}
}
