#pragma once
#include "rolling_policy.hpp"
#include "rolling_raw.hpp"
#include "rolling_event.hpp"
#include "sc2_imu_scope.hpp"
#include <deque>
namespace uwb_imu_pl::research {
struct RollingRawMaterial {
 UwbBatch batch;RawRangeLawEvidence range;
 size_t raw_first=0,raw_last=0;
};
struct RollingBoundaryCertificateV1 {
 RollingRawState separator;
 uint64_t generation=0,parent=0,version=1,source_prefix=21;
 size_t covered_through=0,shared_adc_index=0;
 std::string event_id;
 size_t fault_not_started=0,fault_active=0,fault_ended_residual=0,fault_excluded=0;
};
struct RollingBoundaryProposalV1 {
 BoundaryMaterialViewV1 material;
 RollingBoundaryCertificateV1 certificate;
 RollingRawState current;
 std::deque<RollingRawMaterial> explicit_material;
 uint64_t parent_generation=0;
 size_t raw_read_first=0,raw_read_last=0,physical_branches=0;
 RawRangeResult geometry;
};
class RollingIntegrity {
 ResolvedRollingPolicyV1 policy_;RollingSourceEvent event_;
 RollingBoundaryCertificateV1 certificate_;
 std::deque<RollingRawMaterial> explicit_;
 double accel_rw_,gyro_rw_,acceleration_,rate_;
 Eigen::Vector3d lever_;
 bool failed_=false;
 public:
 RollingIntegrity(const ResolvedRollingPolicyV1&,const NavigationState&,
  const Eigen::Matrix<double,15,1>&,double ba,double bg,const Eigen::Vector3d&,
  double acceleration=.04,double rate=.2);
 const RollingBoundaryCertificateV1& certificate()const{return certificate_;}
 const RollingSourceEvent& event()const{return event_;}
 size_t explicitSize()const{return explicit_.size();}
 RollingBoundaryProposalV1 propose(const EpochTransaction&,const EpochCommitPlan&,
  const RawRangeLawEvidence&,const Sc2ImuMonitor&)const;
 RawRangeResult finalize(RollingBoundaryProposalV1&&,const CommitReceipt&,
  const std::vector<gtsam::Key>&,const NavigationState&);
 void failAfterCommit(){failed_=true;}
};
int runSc5Rolling(int,char**);
}
