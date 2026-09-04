#pragma once

#include "uwb_imu_pl/common/integrity_ids.hpp"
#include "uwb_imu_pl/common/types.hpp"

#include <Eigen/Core>

#include <optional>
#include <string>

namespace uwb_imu_pl {

struct ReinitializationDirective {
  RecoveryRequestId request_id;
  ReinitializationState state = ReinitializationState::Running;
  FdeStatus trigger = FdeStatus::NotTriggered;
  std::string reason;
  TimestampNs requested_at;
  TimestampNs last_committed_timestamp;
};

// ROS-independent state machine. The owner rebuilds the estimator after
// acceptTrustedImu() and calls complete(); source health is intentionally not
// owned here and therefore survives that rebuild.
class ControlledReinitializer {
 public:
  const ReinitializationDirective& directive() const { return directive_; }
  bool blocksUwb() const {
    return directive_.state != ReinitializationState::Running;
  }
  RecoveryRequestId request(FdeStatus trigger, const std::string& reason,
                            const NavigationState& last_committed);
  void beginWaiting();
  std::optional<NavigationState> acceptTrustedImu(
      const ImuMeasurement& measurement);
  void complete();
  Eigen::Matrix<double, 15, 1> inflatedPriorSigmas(
      const Eigen::Matrix<double, 15, 1>& base,
      const Eigen::VectorXd& deterministic_bridge_bound) const;

 private:
  std::uint64_t next_request_id_ = 1;
  ReinitializationDirective directive_;
  NavigationState seed_;
};

}  // namespace uwb_imu_pl
