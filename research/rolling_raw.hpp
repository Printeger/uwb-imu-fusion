#pragma once
#include "raw_range_contraction.hpp"
#include <boost/numeric/interval.hpp>
#include <array>

namespace uwb_imu_pl::research {
using RollingInterval=boost::numeric::interval<double>;
using RollingVector=std::array<RollingInterval,3>;
struct RollingRawState {
 size_t epoch=0;
 bool has_tag_relation=false;RollingVector tag_relation;Eigen::Vector3d tag_lever=Eigen::Vector3d::Zero();
 RollingVector p,v;
 std::array<RollingVector,3> rotation;
 std::array<RollingInterval,6> bias;
};
RollingRawState rollingInitial(const NavigationState&,const Eigen::Matrix<double,15,1>&,double q,double roundoff);
RollingRawState rollingPropagate(const RollingRawState&,double acceleration,double angular_rate,
 double accel_rw,double gyro_rw,double q);
RollingVector rollingTag(const RollingRawState&,const Eigen::Vector3d& lever);
// Necessary inverse of tag=p+Rl; contraction never assigns tag directly to p.
bool rollingContractTag(RollingRawState&,const RollingVector&,const Eigen::Vector3d&);
// Necessary bidirectional continuous bounded-acceleration relation.
bool rollingContractDynamics(RollingRawState& previous,RollingRawState& next,double acceleration);
bool rollingContractCoupledDynamics(RollingRawState&,RollingRawState&,double acceleration);
RollingVector rollingTagSphere(const RollingRawState&,const Eigen::Vector3d&);
bool rollingContractTagSphere(RollingRawState&,const RollingVector&,const Eigen::Vector3d&);
RollingRawState rollingProject(const RollingRawState&);
}
