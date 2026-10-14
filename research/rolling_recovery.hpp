#pragma once
#include "rolling_boundary.hpp"
#include "bounded_imu_recovery.hpp"
namespace uwb_imu_pl::research {
struct RollingFdeAssessment {
 EpochCommitPlan plan;
 bool current_bridge=false,rejoined=false,boundary_repair=false,history_marginalized=false;
 int excluded=-1,rebuilds=0;
 size_t actions=1,numeric_evaluated=0,reintegrations=0,first_dependency=0;
 double candidate_ms=0;
 std::string status="KEEP",recovery="NOT_NEEDED";
 std::vector<gtsam::NonlinearFactor::shared_ptr> removed_boundary;
 std::vector<size_t> removed_prior_slots;
 FactorGroupId metric_group;
};
class RollingRecovery {
 Eigen::Matrix<double,6,1> bias_observation_;
 ResolvedRollingPolicyV1 policy_;
 bool isolated_=false,stable_rotation_=false;
 TimestampNs quarantine_begin_,quarantine_end_;
 int held_anchor_=-1,held_until_=0,observed_anchor_=-1;
 size_t observed_uwb_onset_=0;
 uint64_t alert_generation_=0;
 bool open_pending_=false,history_unavailable_=false;
 public:
 RollingRecovery(const NavigationState&,const ResolvedRollingPolicyV1&,bool stable_rotation=false);
 RollingFdeAssessment prepare(EpochTransaction&,const Sc2ImuMonitor&,
   const std::deque<ImuMeasurement>&,const std::deque<RollingRawState>&,
   const IntegrityConfig&,bool fde);
 RollingFdeAssessment prepare(EpochTransaction&,const RollingMonitorView&,
   const std::deque<ImuMeasurement>&,const std::deque<RollingRawState>&,
   const IntegrityConfig&,bool fde);
 void committed(const RollingFdeAssessment&,const EpochTransaction&,const CommitReceipt&,
   const FactorLedger&);
};
}
