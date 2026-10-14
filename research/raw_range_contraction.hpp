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
struct RawRangeBudget { size_t max_boxes=256;unsigned passes=8,split_depth=0;bool all_ranges_healthy=false;bool tight_shell=false;bool optimized=false; };
struct RawRangeResult {
 std::string status="EVIDENCE_REFUSED",reason,source="raw-imu-uwb-contracted-v1";
 std::string relaxation="CURRENT_SINGLE_FAULT_OUTER_RELAXATION_OF_EPISODE_SCOPE";
 Eigen::Vector3d low=Eigen::Vector3d::Constant(NAN),high=low,pl=low,old_pl=low;
 double hpl=INFINITY,old_hpl=INFINITY,event_risk=INFINITY,elapsed_ms=0;
 size_t considered=0,empty=0,unresolved=0,visited=0,retained=0,frontier=0,peak_boxes=0;
 bool valid=false,budget_exhausted=false;
};
// Geometry kernel: all nine branches are mandatory; no FDE/truth input.
RawRangeResult contractRawRangeBox(const Eigen::Vector3d&,const Eigen::Vector3d&,
 const UwbBatch&,const std::array<RawRangeShell,8>&,const Eigen::Vector3d&,RawRangeBudget={});
RawRangeResult issueRawRangeContraction(const RawInclusionEvidence&,const RawRangeLawEvidence&,
 const EpochTransaction&,const CommitReceipt&,const NavigationState&,const Eigen::Vector3d&,
 RawRangeBudget={});
}
