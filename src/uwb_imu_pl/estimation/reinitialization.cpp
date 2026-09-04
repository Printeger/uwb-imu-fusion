#include "uwb_imu_pl/estimation/reinitialization.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {

RecoveryRequestId ControlledReinitializer::request(
    FdeStatus trigger, const std::string& reason,
    const NavigationState& last_committed) {
  if (directive_.state != ReinitializationState::Running) {
    return directive_.request_id;
  }
  directive_.request_id = RecoveryRequestId(next_request_id_++);
  directive_.state = ReinitializationState::Requested;
  directive_.trigger = trigger;
  directive_.reason = reason;
  directive_.requested_at = last_committed.timestamp;
  directive_.last_committed_timestamp = last_committed.timestamp;
  seed_ = last_committed;
  return directive_.request_id;
}

void ControlledReinitializer::beginWaiting() {
  if (directive_.state != ReinitializationState::Requested) {
    throw std::logic_error("reinitialization is not requested");
  }
  directive_.state = ReinitializationState::WaitingForTrustedImu;
}

std::optional<NavigationState> ControlledReinitializer::acceptTrustedImu(
    const ImuMeasurement& measurement) {
  if (directive_.state != ReinitializationState::WaitingForTrustedImu) {
    return std::nullopt;
  }
  if (!(directive_.last_committed_timestamp < measurement.timestamp) ||
      !measurement.specific_force_mps2.allFinite() ||
      !measurement.angular_velocity_radps.allFinite()) {
    return std::nullopt;
  }
  seed_.timestamp = measurement.timestamp;
  directive_.state = ReinitializationState::Reinitialized;
  return seed_;
}

void ControlledReinitializer::complete() {
  if (directive_.state != ReinitializationState::Reinitialized) {
    throw std::logic_error("trusted IMU boundary has not been accepted");
  }
  directive_.state = ReinitializationState::Running;
}

Eigen::Matrix<double, 15, 1> ControlledReinitializer::inflatedPriorSigmas(
    const Eigen::Matrix<double, 15, 1>& base,
    const Eigen::VectorXd& bound) const {
  if (!base.allFinite() || (base.array() <= 0.0).any() ||
      (bound.size() != 0 && (bound.size() != 9 || !bound.allFinite() ||
                            (bound.array() < 0.0).any()))) {
    throw std::invalid_argument("invalid reinitialization prior inflation input");
  }
  Eigen::Matrix<double, 15, 1> out = base;
  if (bound.size() == 9) {
    out.head<3>() = (out.head<3>().array().square() +
                     bound.head<3>().array().square()).sqrt();
    out.segment<3>(3) = (out.segment<3>(3).array().square() +
                          bound.segment<3>(3).array().square()).sqrt();
    out.segment<3>(6) = (out.segment<3>(6).array().square() +
                          bound.tail<3>().array().square()).sqrt();
  }
  return out;
}

}  // namespace uwb_imu_pl
