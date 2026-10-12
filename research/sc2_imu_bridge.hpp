#pragma once
#include "sc2_imu_scope.hpp"
#include "raw_range_contraction.hpp"
namespace uwb_imu_pl::research {
struct Sc2StateEnvelope {
 Eigen::Vector3d position_low=Eigen::Vector3d::Constant(NAN),position_high=position_low,velocity_low=position_low,velocity_high=position_low,tag_low=position_low,tag_high=position_low;
 Eigen::Matrix<double,6,1> bias_low=Eigen::Matrix<double,6,1>::Constant(NAN),bias_high=bias_low;
 Eigen::Matrix3d rotation_low=Eigen::Matrix3d::Constant(NAN),rotation_high=rotation_low;
 double rotation_diameter=INFINITY,event_risk=INFINITY,time_s=0;
 int epoch=0;bool valid=false;
 std::string method="raw-imu-fault-independent-bridge-uwb-v1",scope="sc2-single-imu-episode-rw-v1";
};
// Original source-aware initial boundary, never a polluted posterior. This
// physical bridge is valid for all fault hypotheses/actions simultaneously.
class Sc2IndependentBridge {
 NavigationState initial_;RawInclusionParameters law_;Sc2ImuScope scope_;
 public:
 Sc2IndependentBridge(const NavigationState&,const RawInclusionParameters&,Sc2ImuScope={});
 Sc2StateEnvelope envelope(int epoch,const Eigen::Vector3d& lever)const;
 RawRangeResult issue(const Sc2ImuMonitor&,const RawRangeLawEvidence&,const EpochTransaction&,
 const CommitReceipt&,const NavigationState&,const Eigen::Vector3d&)const;
};
}
