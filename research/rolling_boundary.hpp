#pragma once
#include "rolling_policy.hpp"
#include "rolling_raw.hpp"
#include "joint_raw.hpp"
#include "rolling_event.hpp"
#include "sc2_imu_scope.hpp"
#include "rolling_monitor.hpp"
#include "rolling_evidence.hpp"
#include <deque>
namespace uwb_imu_pl::research {
struct RollingRawMaterial {
 UwbBatch batch;RawRangeLawEvidence range;
 size_t raw_first=0,raw_last=0;
 std::vector<ImuMeasurement> imu;
 size_t epoch=0;
 std::shared_ptr<const GeneralizedRawEvidence> generalized;
};
struct RollingBoundaryCertificateV1 {
 RollingRawState separator;
 uint64_t generation=0,parent=0,version=1,source_prefix=21;
 size_t covered_through=0,shared_adc_index=0;
 TimestampNs committed_time;
 double source_risk=0;
 bool fault_counts_complete=true;
 std::string event_id,state_method="coordinate-state-box-v1";
 size_t fault_not_started=0,fault_active=0,fault_ended_residual=0,fault_excluded=0;
 std::array<AnchorId,8> anchor_ids;
 std::array<Eigen::Vector3d,8> anchor_positions=[] {std::array<Eigen::Vector3d,8> a;for(auto& p:a)p.setZero();return a;}();
 std::map<AnchorId,Eigen::Vector3d> dynamic_anchors;
 std::map<AnchorId,uint64_t> anchor_versions;
};
struct RollingBoundaryProposalV1 {
 BoundaryMaterialViewV1 material;
 RollingBoundaryCertificateV1 certificate;
 RollingRawState current;
 std::deque<RollingRawMaterial> explicit_material;
 uint64_t parent_generation=0;
 size_t raw_read_first=0,raw_read_last=0,physical_branches=0;
 JointConstraintEvidence joint;
 RawRangeResult geometry;
 std::vector<AttributionSnapshot> attribution;
 private:
 struct Seal {
  const void* owner;
  BoundaryMaterialViewV1 material;
  RollingBoundaryCertificateV1 certificate;
  RollingRawState current;
  std::deque<RollingRawMaterial> explicit_material;
  JointConstraintEvidence joint;
  RawRangeResult geometry;
  uint64_t parent;
 };
 std::shared_ptr<const Seal> seal_;
 friend class RollingIntegrity;
};
class RollingIntegrity {
 ResolvedRollingPolicyV1 policy_;RollingSourceEvent event_;
 RollingBoundaryCertificateV1 certificate_;
 std::deque<RollingRawMaterial> explicit_;
 double accel_rw_,gyro_rw_,acceleration_,rate_;
 Eigen::Vector3d lever_;
 bool failed_=false,tight_=false,optimized_=false,attribute_=false,joint_=false;JointRawLaw joint_law_;unsigned joint_sweeps_=2;bool support_=false;size_t support_budget_=256;
 std::optional<InitialSetEvidence> generalized_;
 public:
 RollingIntegrity(const ResolvedRollingPolicyV1&,const NavigationState&,
  const Eigen::Matrix<double,15,1>&,double ba,double bg,const Eigen::Vector3d&,
  double acceleration=.04,double rate=.2,bool tight=false,bool optimized=false);
 RollingIntegrity(const ResolvedRollingPolicyV1&,const InitialSetEvidence&,
  double ba,double bg,const Eigen::Vector3d&,double acceleration,double rate,bool tight=true,bool optimized=true);
 void enableGeneralizedJoint(const JointRawLaw&,unsigned sweeps=1);
 void enableJoint(const ImuMeasurement&,const JointRawLaw&,unsigned sweeps=2);
 void enableTagSupport(size_t max_boxes=256){support_=true;support_budget_=max_boxes;}
 void enableAttribution(bool enabled){attribute_=enabled;}
 const RollingBoundaryCertificateV1& certificate()const{return certificate_;}
 const RollingSourceEvent& event()const{return event_;}
 size_t explicitSize()const{return explicit_.size();}
 RollingBoundaryProposalV1 propose(const EpochTransaction&,const EpochCommitPlan&,
  const RawRangeLawEvidence&,const Sc2ImuMonitor&)const;
 RollingBoundaryProposalV1 propose(const EpochTransaction&,const EpochCommitPlan&,
  const RawRangeLawEvidence&,const RollingMonitorView&,
  std::shared_ptr<const GeneralizedRawEvidence> generalized={})const;
 RawRangeResult finalize(RollingBoundaryProposalV1&&,const CommitReceipt&,
  const std::vector<gtsam::Key>&,const NavigationState&);
 Eigen::Vector3d committedTag(const NavigationState&)const;
 void failAfterCommit(){failed_=true;}
};
int runSc5Rolling(int,char**);
}
