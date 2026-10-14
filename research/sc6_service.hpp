#pragma once
#include "open_fault_pool.hpp"
#include "rolling_recovery.hpp"
#include "sc6_provider.hpp"
namespace uwb_imu_pl::research {
struct Sc6Result {
 size_t epoch=0;NavigationState state;RawRangeResult bound;RollingFdeAssessment action;
 uint64_t version=0,generation=0,parent=0,source_prefix=0;
 size_t boundary=0,alerts=0,slots=0,live=0,ledger=0,replay=0,capsules=0,rss=0;
 bool x0=false;double bias_width=0,q=0,shared_q=0;
 double prepare_ms=0,candidate_ms=0,proof_ms=0,commit_ms=0,cleanup_ms=0;
};
class Sc6Service {
 IntegrityConfig config_;Eigen::Vector3d lever_;ResolvedRollingPolicyV1 policy_;
 TimeUniformEvent event_;RollingIntegrity proof_;OpenFaultPool faults_;RollingRecovery recovery_;
 IncrementalUwbImuEstimator backend_;
 std::deque<ImuMeasurement> replay_;std::deque<RollingRawState> capsules_;
 public:
 Sc6Service(const YAML::Node&,const IntegrityConfig&,const NavigationState&,
 const Eigen::Vector3d&,const Sc6MotionDomain&,const ImuMeasurement&);
 Sc6Result step(const Sc6Envelope&,bool fde);
};
int runSc6System(int,char**);
}
