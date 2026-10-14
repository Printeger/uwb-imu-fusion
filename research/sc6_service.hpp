#pragma once
#include "open_fault_pool.hpp"
#include "rolling_recovery.hpp"
#include "sc6_provider.hpp"
#include "sc6_temporal_monitor.hpp"
namespace uwb_imu_pl::research {
struct Sc6Result {
 size_t epoch=0;NavigationState state;Eigen::Vector3d tag;RawRangeResult bound;RollingFdeAssessment action;
 double old_hpl=NAN,old_vpl=NAN;std::array<double,3> tag_width{},velocity_width{},rotation_width{};std::array<double,6> bias_widths{};
 std::chrono::steady_clock::time_point alarm_time,selected_time,commit_time,proof_time;
 std::chrono::steady_clock::time_point imu_alarm_time;
 uint64_t version=0,generation=0,parent=0,source_prefix=0;
 size_t boundary=0,alerts=0,slots=0,live=0,ledger=0,replay=0,capsules=0,rss=0;
 bool x0=false;double bias_width=0,q=0,shared_q=0;
 OpenImuDiagnostic monitor;std::array<double,6> monitor_min_margin{};
 bool imu_alarm=false;uint64_t alert_generation=0;size_t imu_active=0,imu_ended=0,imu_excluded=0,uwb_active=0;int64_t first_imu_alarm_ns=0;
 std::array<double,6> temporal_difference{},temporal_radius{},temporal_margin{};size_t temporal_alerts=0;
 std::array<double,6> fault_theta_low{},fault_theta_high{};
 double prepare_ms=0,candidate_ms=0,proof_ms=0,commit_ms=0,cleanup_ms=0;
};
class Sc6Service {
 IntegrityConfig config_;Eigen::Vector3d lever_;ResolvedRollingPolicyV1 policy_;bool optimized_;
 TimeUniformEvent event_;RollingIntegrity proof_;OpenFaultPool faults_;RollingRecovery recovery_;
 std::unique_ptr<Sc6TemporalMonitor> temporal_;uint64_t combined_alert_generation_=0;
 std::unique_ptr<RollingIntegrity> shadow_;
 IncrementalUwbImuEstimator backend_;
 std::deque<ImuMeasurement> replay_;std::deque<RollingRawState> capsules_;
 public:
 Sc6Service(const YAML::Node&,const IntegrityConfig&,const NavigationState&,
 const Eigen::Vector3d&,const Sc6MotionDomain&,const ImuMeasurement&,bool tight=false,bool shadow=false,bool optimized=false);
 uint64_t updateCount()const{return backend_.backendUpdateCount();}
 Sc6Result step(const Sc6Envelope&,bool fde);
};
int runSc6System(int,char**);
}
