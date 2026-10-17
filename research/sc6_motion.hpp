#pragma once
#include "rolling_raw.hpp"
namespace uwb_imu_pl::research {
struct Sc6MotionRecipe {int kind=0;Eigen::Vector3d amp{.1,.08,.05},freq{.5,.4,.3};};
struct Sc6MotionValue {RollingVector p,v,a,omega;std::array<RollingVector,3> r;};
// Directed analytic kinematics. Recipe is provider-only; online consumes domain.
Sc6MotionValue sc6MotionAt(int64_t sample_index,const Sc6MotionRecipe&);
struct Sc6MotionDomain {std::string id;double acceleration=.04,rate=.2,jerk=.02,alpha=0,global_speed=INFINITY;};
Sc6MotionDomain sc6Domain(const Sc6MotionRecipe&);
RawRangeLawEvidence certifySc6Ranges(const UwbBatch&,const Sc6MotionValue&,
 const Eigen::Vector3d&,const std::array<double,8>&,const std::array<double,8>&);
}
