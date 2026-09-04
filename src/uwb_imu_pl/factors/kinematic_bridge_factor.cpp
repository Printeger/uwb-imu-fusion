#include "uwb_imu_pl/factors/kinematic_bridge_factor.hpp"

#include "uwb_imu_pl/factors/realtime_factors.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <boost/make_shared.hpp>

#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {
gtsam::Key x(std::size_t epoch) { return gtsam::Symbol('x', epoch); }
gtsam::Key v(std::size_t epoch) { return gtsam::Symbol('v', epoch); }
gtsam::Key b(std::size_t epoch) { return gtsam::Symbol('b', epoch); }

void positive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}
}  // namespace

PendingFactorGroup BridgeFactory::makeGeneric(
    const EpochTransaction& tx, const GenericBridgeSpec& config) const {
  if (!config.enabled) throw std::invalid_argument("generic bridge is disabled");
  const double dt = tx.end.seconds() - tx.begin.seconds();
  positive(dt, "bridge dt");
  positive(config.optimization_sigma_rotation_rad, "rotation sigma");
  positive(config.optimization_sigma_position_m, "position sigma");
  positive(config.optimization_sigma_velocity_mps, "velocity sigma");
  positive(config.optimization_sigma_bias_accel, "accelerometer bias sigma");
  positive(config.optimization_sigma_bias_gyro, "gyroscope bias sigma");
  Eigen::Matrix<double, 9, 1> sigma;
  sigma << Eigen::Vector3d::Constant(config.optimization_sigma_rotation_rad),
      Eigen::Vector3d::Constant(config.optimization_sigma_position_m),
      Eigen::Vector3d::Constant(config.optimization_sigma_velocity_mps);
  const Eigen::Matrix<double, 9, 9> covariance = sigma.array().square().matrix().asDiagonal();
  PendingFactorGroup group;
  group.id = tx.generic_bridge_group.id;
  group.kind = FactorKind::KinematicBridge;
  group.sensor = SensorType::Bridge;
  group.keys = {x(tx.previous_epoch), v(tx.previous_epoch),
                x(tx.proposed_epoch), v(tx.proposed_epoch)};
  group.noise_model_id = "generic_bridge_optimization_covariance_v1";
  group.model_id = config.model;
  group.factors.add(boost::make_shared<KinematicPoseVelocityBridgeFactor>(
      x(tx.previous_epoch), v(tx.previous_epoch), x(tx.proposed_epoch),
      v(tx.proposed_epoch), dt, covariance));
  return group;
}

PendingFactorGroup BridgeFactory::makeBiasContinuity(
    const EpochTransaction& tx, const GenericBridgeSpec& config) const {
  positive(config.optimization_sigma_bias_accel, "accelerometer bias sigma");
  positive(config.optimization_sigma_bias_gyro, "gyroscope bias sigma");
  PendingFactorGroup group;
  group.id = tx.generic_bias_continuity_group.id;
  group.kind = FactorKind::BiasContinuity;
  group.sensor = SensorType::Bridge;
  group.keys = {b(tx.previous_epoch), b(tx.proposed_epoch)};
  group.noise_model_id = "generic_bridge_bias_continuity_covariance_v1";
  group.model_id = "constant_bias_continuity";
  Eigen::Matrix<double, 6, 1> bias_sigmas;
  bias_sigmas << Eigen::Vector3d::Constant(config.optimization_sigma_bias_accel),
      Eigen::Vector3d::Constant(config.optimization_sigma_bias_gyro);
  group.factors.add(boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
      b(tx.previous_epoch), b(tx.proposed_epoch),
      gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas)));
  return group;
}

BridgeUncertainty BridgeFactory::uncertainty(
    const EpochTransaction& tx, const GenericBridgeSpec& config) const {
  const double dt = tx.end.seconds() - tx.begin.seconds();
  positive(dt, "bridge dt");
  BridgeUncertainty out;
  out.integrity_model = BridgeUncertainty::IntegrityModel::DeterministicBox;
  out.calibration_id = config.calibration_id;
  Eigen::Matrix<double, 9, 1> sigma;
  sigma << Eigen::Vector3d::Constant(config.optimization_sigma_rotation_rad),
      Eigen::Vector3d::Constant(config.optimization_sigma_position_m),
      Eigen::Vector3d::Constant(config.optimization_sigma_velocity_mps);
  out.optimization_covariance = sigma.array().square().matrix().asDiagonal();
  out.deterministic_bound.resize(9);
  out.deterministic_bound.head<3>().setConstant(
      config.angular_rate_bound_radps * dt +
      0.5 * config.angular_acceleration_bound_radps2 * dt * dt);
  out.deterministic_bound.segment<3>(3).setConstant(
      0.5 * config.acceleration_bound_mps2 * dt * dt);
  out.deterministic_bound.tail<3>().setConstant(
      config.acceleration_bound_mps2 * dt);
  return out;
}

Eigen::Vector3d BridgeFactory::propagateBoxMargin(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& protected_map,
    const Eigen::MatrixXd& bridge_error_gain,
    const Eigen::VectorXd& box_bound) const {
  if (protected_map.cols() != bridge_error_gain.rows() ||
      bridge_error_gain.cols() != box_bound.size() ||
      !protected_map.allFinite() || !bridge_error_gain.allFinite() ||
      !box_bound.allFinite() || (box_bound.array() < 0.0).any()) {
    throw std::invalid_argument("bridge margin propagation dimensions invalid");
  }
  return (protected_map * bridge_error_gain).cwiseAbs() * box_bound;
}

}  // namespace uwb_imu_pl
