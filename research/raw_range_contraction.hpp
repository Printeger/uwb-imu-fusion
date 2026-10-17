#pragma once
#include "raw_nonlinear_inclusion.hpp"
#include <array>
namespace uwb_imu_pl::research {
// Trusted simulation adapter evidence, not an online fault detector. Contains
// no draw, truth point or injection identity. Batch is a sealed value snapshot.
struct RawRangeLawEvidence {
 UwbBatch batch;
 std::array<double,8> rho_num{};
 std::string source_law_id="raw-source-rw-v1",frame="world";
 bool valid=false;
 private:
 UwbBatch sealed_batch_;std::array<double,8> sealed_rho_{};
 friend RawRangeLawEvidence certifySc6Ranges(const UwbBatch&,const struct Sc6MotionValue&,const Eigen::Vector3d&,const std::array<double,8>&,const std::array<double,8>&);
 friend RawRangeLawEvidence certifyRawRanges(const UwbBatch&,const Eigen::Vector3d&,const Eigen::Vector3d&,double,const Eigen::Vector3d&,const std::array<double,8>&,const std::array<double,8>&);
 friend bool rawRangeShells(const RawRangeLawEvidence&,const UwbBatch&,int,std::array<struct RawRangeShell,8>&);
 friend bool rawRangeShells(const RawRangeLawEvidence&,const UwbBatch&,int,std::array<struct RawRangeShell,8>&,int,double);
};
RawRangeLawEvidence certifyRawRanges(const UwbBatch&,const Eigen::Vector3d& amplitude,
 const Eigen::Vector3d& frequency,double rate,const Eigen::Vector3d& lever,
 const std::array<double,8>& normals,const std::array<double,8>& additive_fault);
struct RawRangeShell { double low=0,high=0; };
bool sameRawBatch(const UwbBatch&,const UwbBatch&);
bool rawRangeShells(const RawRangeLawEvidence&,const UwbBatch&,int epoch,
 std::array<RawRangeShell,8>&);
bool rawRangeShells(const RawRangeLawEvidence&,const UwbBatch&,int epoch,
 std::array<RawRangeShell,8>&,int mission,double coordinate_radius);
// NONE (-1) plus each physical row: current projection of the episode scope.
std::array<int,9> rawRangeFaultMasks();
// Representative row per distinct physical source, in input order; -1=KEEP.
// Excluding a source removes every observation of that anchor.
std::vector<int> rawRangeFaultMasks(const UwbBatch&);
struct JointConstraintEvidence {
 std::string method="sc7-raw-imu-state-integral-theta-v1";
 std::string fault_relaxation="ALL7_EPOCH_CONSTANT_THETA_REPEAT_OUTER_NO_HEALTHY_GAP";
 size_t first_source=0,last_source=0,branches=0,empty=0,frontier=0;
 std::array<double,6> before{},after{};
 std::array<std::array<double,6>,7> branch_widths{};
 bool valid=false,fallback=false;
};
struct RawSupportBox {
 Eigen::Vector3d low,high;int omitted=-1;bool frontier=false;
};
struct AttributionSnapshot {
 std::string stage;size_t epoch=0;int omitted=-2;
 Eigen::Vector3d low,high;
 std::string extremum_class="OUTER_ENDPOINT_NO_FEASIBILITY_WITNESS";
};
struct TagSupportEvidence;
struct RawRangeBudget { size_t max_boxes=256;unsigned passes=8,split_depth=0;bool all_ranges_healthy=false;bool tight_shell=false;bool optimized=false;bool retain_support=false;bool lower_shell=false;bool scoped_rounding=false; };
struct RawRangeResult {
 std::string bound_method="coordinate-box-v1";
 std::string status="EVIDENCE_REFUSED",reason,source="raw-imu-uwb-contracted-v1";
 std::string relaxation="CURRENT_SINGLE_FAULT_OUTER_RELAXATION_OF_EPISODE_SCOPE";
 Eigen::Vector3d low=Eigen::Vector3d::Constant(NAN),high=low,pl=low,old_pl=low,input_low=low,input_high=low;
 double hpl=INFINITY,old_hpl=INFINITY,event_risk=INFINITY,elapsed_ms=0;
 size_t considered=0,empty=0,unresolved=0,visited=0,retained=0,frontier=0,peak_boxes=0;
 std::shared_ptr<const TagSupportEvidence> tag_support;
 JointConstraintEvidence joint;
 std::vector<RawSupportBox> support;
 std::vector<AttributionSnapshot> attribution;
 bool valid=false,budget_exhausted=false;
};
// Geometry kernel: all nine branches are mandatory; no FDE/truth input.
RawRangeResult contractRawRangeBox(const Eigen::Vector3d&,const Eigen::Vector3d&,
 const UwbBatch&,const std::array<RawRangeShell,8>&,const Eigen::Vector3d&,RawRangeBudget={});
RawRangeResult contractRawRangeBox(const Eigen::Vector3d&,const Eigen::Vector3d&,
 const UwbBatch&,const std::vector<RawRangeShell>&,const Eigen::Vector3d&,RawRangeBudget={});
RawRangeResult issueRawRangeContraction(const RawInclusionEvidence&,const RawRangeLawEvidence&,
 const EpochTransaction&,const CommitReceipt&,const NavigationState&,const Eigen::Vector3d&,
 RawRangeBudget={});
}
