#pragma once
#include "open_fault_pool.hpp"
#include "rolling_recovery.hpp"
#include "sensor_envelope.hpp"
#include "sc6_temporal_monitor.hpp"
#include "generalized_fault_monitor.hpp"
namespace uwb_imu_pl::research {
struct Sc6Result {
 size_t epoch=0;NavigationState state;Eigen::Vector3d tag;RawRangeResult bound;RollingFdeAssessment action;
 double old_hpl=NAN,old_vpl=NAN;std::array<double,3> tag_width{},velocity_width{},rotation_width{};std::array<double,6> bias_widths{};
 std::chrono::steady_clock::time_point alarm_time,selected_time,commit_time,proof_time;
 std::chrono::steady_clock::time_point imu_alarm_time;
 uint64_t version=0,generation=0,parent=0,source_prefix=0;
 size_t active_states=0,proof_explicit=0;
 RangeCounts input_counts;
 size_t time_factor_groups=0,time_committed_factors=0;double max_range_transport=0;
 size_t boundary=0,alerts=0,slots=0,live=0,ledger=0,replay=0,capsules=0,rss=0;
 std::array<double,6> reference_bias_width{};
 bool x0=false;double bias_width=0,q=0,shared_q=0;
 OpenImuDiagnostic monitor;std::array<double,6> monitor_min_margin{};
 bool imu_alarm=false;uint64_t alert_generation=0;size_t imu_active=0,imu_ended=0,imu_excluded=0,uwb_active=0;int64_t first_imu_alarm_ns=0;
 std::array<double,6> temporal_difference{},temporal_radius{},temporal_margin{};size_t temporal_alerts=0;
 std::array<double,6> fault_theta_low{},fault_theta_high{};
 double prepare_ms=0,candidate_ms=0,proof_ms=0,commit_ms=0,cleanup_ms=0;
 // Observation-only exclusive subdivisions of the existing inclusive spans.
 double imu_fd_ms=0,graph_prepare_ms=0,state_receipt_ms=0,finalize_ms=0,maintenance_ms=0;
};
struct GeneralizedCoreInitialization {
 std::unique_ptr<GeneralizedSourceLedger> sources;
 InitialSetEvidence initial_set;
 AnchorRegistry anchors;
 JointRawLaw raw_law;
};
class UnifiedRollingIntegrityCore {
 IntegrityConfig config_;Eigen::Vector3d lever_;Sc6MotionDomain domain_;ResolvedRollingPolicyV1 policy_;bool optimized_,failed_=false;
 TimeUniformEvent event_;RollingIntegrity proof_;std::unique_ptr<OpenFaultPool> faults_;RollingRecovery recovery_;
 std::unique_ptr<GeneralizedSourceLedger> sources_;
 std::unique_ptr<GeneralizedFaultMonitor> generalized_monitor_;
 AnchorRegistry anchors_;
 std::deque<MeasurementId> replay_parents_;
 std::map<size_t,std::map<MeasurementId,double>> range_time_history_;
 std::unique_ptr<Sc6TemporalMonitor> temporal_;uint64_t combined_alert_generation_=0;
 std::unique_ptr<RollingIntegrity> shadow_;
 IncrementalUwbImuEstimator backend_;
 std::array<RollingInterval,6> reference_bias_;
 std::deque<ImuMeasurement> replay_;std::deque<RollingRawState> capsules_;
 public:
 UnifiedRollingIntegrityCore(const YAML::Node&,const IntegrityConfig&,const NavigationState&,
 const Eigen::Vector3d&,const Sc6MotionDomain&,const ImuMeasurement&,bool tight=false,bool shadow=false,bool optimized=false);
 UnifiedRollingIntegrityCore(const YAML::Node&,const IntegrityConfig&,const NavigationState&,
 const Eigen::Vector3d&,const Sc6MotionDomain&,const ImuMeasurement&,GeneralizedCoreInitialization);
 void declareAnchor(const AnchorRecord& anchor,uint64_t version){anchors_.declare(anchor,version);}
 void anchorOnline(AnchorId anchor,bool online){anchors_.online(anchor,online);}
 static constexpr const char* implementationId(){return "unified-sc6-sc7-rolling-v1";}
 uint64_t updateCount()const{return backend_.backendUpdateCount();}
 Sc6Result step(const Sc6Envelope&,bool fde);
};
using Sc6Service = UnifiedRollingIntegrityCore; // Source-compatible legacy wrapper.
int runSc6System(int,char**);
}
