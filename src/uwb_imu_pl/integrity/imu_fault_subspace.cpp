#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

gtsam::Pose3 pose(const NavigationState& state) {
  return gtsam::Pose3(gtsam::Rot3(state.q_world_body.normalized().toRotationMatrix()),
                      state.position_world_m);
}
gtsam::imuBias::ConstantBias bias(const NavigationState& state) {
  return {state.accel_bias_mps2, state.gyro_bias_radps};
}

struct ImuFactorPoint {
  gtsam::Pose3 previous_pose, current_pose;
  gtsam::Vector3 previous_velocity, current_velocity;
  gtsam::imuBias::ConstantBias previous_bias, current_bias;
};

bool factorPoint(const EpochTransaction& tx, ImuFactorPoint* point) {
  if (!point) return false;
  if (tx.frozen_values) {
    // The pending factor is linearized at these Values, not at the
    // measurement-only IMU prediction. A partial/wrong typed frozen snapshot
    // is invalid; never silently switch its sensitivity to another point.
    try {
      const auto& values = *tx.frozen_values;
      point->previous_pose = values.at<gtsam::Pose3>(gtsam::Symbol('x', tx.previous_epoch));
      point->current_pose = values.at<gtsam::Pose3>(gtsam::Symbol('x', tx.proposed_epoch));
      point->previous_velocity = values.at<gtsam::Vector3>(gtsam::Symbol('v', tx.previous_epoch));
      point->current_velocity = values.at<gtsam::Vector3>(gtsam::Symbol('v', tx.proposed_epoch));
      point->previous_bias = values.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', tx.previous_epoch));
      point->current_bias = values.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', tx.proposed_epoch));
    } catch (const std::exception&) {
      return false;
    }
  } else {
    // Historical material transactions carry the actual recorded frozen
    // states in these fields; retain the legacy no-Values contract.
    point->previous_pose = pose(tx.previous_state);
    point->current_pose = pose(tx.nominal_predicted_state);
    point->previous_velocity = tx.previous_state.velocity_world_mps;
    point->current_velocity = tx.nominal_predicted_state.velocity_world_mps;
    point->previous_bias = bias(tx.previous_state);
    point->current_bias = bias(tx.nominal_predicted_state);
  }
  return point->previous_pose.matrix().allFinite() &&
      point->current_pose.matrix().allFinite() &&
      point->previous_velocity.allFinite() && point->current_velocity.allFinite() &&
      point->previous_bias.vector().allFinite() && point->current_bias.vector().allFinite();
}

gtsam::Vector factorError(const ImuFactorPoint& point,
                          const gtsam::PreintegratedCombinedMeasurements& pim) {
  gtsam::CombinedImuFactor factor(1, 2, 3, 4, 5, 6, pim);
  return factor.evaluateError(
      point.previous_pose, point.previous_velocity,
      point.current_pose, point.current_velocity,
      point.previous_bias, point.current_bias);
}

gtsam::PreintegratedCombinedMeasurements reintegrate(
    const EpochTransaction& tx, int axis, double perturbation) {
  gtsam::PreintegratedCombinedMeasurements pim(*tx.preintegration);
  pim.resetIntegration();
  if (tx.raw_imu_slice.size() < 2) return pim;
  for (std::size_t i = 1; i < tx.raw_imu_slice.size(); ++i) {
    const auto& left = tx.raw_imu_slice[i - 1];
    const auto& right = tx.raw_imu_slice[i];
    const double dt = right.timestamp.seconds() - left.timestamp.seconds();
    Eigen::Vector3d acc = 0.5 * (left.specific_force_mps2 +
                                 right.specific_force_mps2);
    Eigen::Vector3d gyro = 0.5 * (left.angular_velocity_radps +
                                  right.angular_velocity_radps);
    if (axis < 3) acc(axis) += perturbation;
    else gyro(axis - 3) += perturbation;
    pim.integrateMeasurement(acc, gyro, dt);
  }
  return pim;
}

}  // namespace

Eigen::MatrixXd ImuFaultSubspaceBuilder::finiteDifference(
    const EpochTransaction& tx, const LinearizedFactorBlock& block,
    double epsilon) const {
  Eigen::MatrixXd map = Eigen::MatrixXd::Zero(block.residual_whitened.size(), 6);
  ImuFactorPoint point;
  if (!factorPoint(tx, &point)) return Eigen::MatrixXd::Constant(
      block.residual_whitened.size(), 6, std::numeric_limits<double>::quiet_NaN());
  for (int axis = 0; axis < 6; ++axis) {
    NumericalWorkCounters::imuOracleReintegration();
    const auto plus = reintegrate(tx, axis, epsilon);
    NumericalWorkCounters::imuOracleReintegration();
    const auto minus = reintegrate(tx, axis, -epsilon);
    const Eigen::VectorXd derivative =
        (factorError(point, plus) - factorError(point, minus)) / (2.0 * epsilon);
    // Integrity uses z=-error.
    map.col(axis) = -block.whitener * derivative;
  }
  return map;
}

ImuFaultSubspaces ImuFaultSubspaceBuilder::build(
    const EpochTransaction& tx, const LinearizedFactorBlock& block) const {
  return buildAnalytic(tx, block);
}

ImuFaultSubspaces ImuFaultSubspaceBuilder::buildAnalytic(
    const EpochTransaction& tx, const LinearizedFactorBlock& block) const {
  ImuFaultSubspaces out;
  if (!tx.preintegration || block.kind != FactorKind::CombinedImu ||
      block.whitener.rows() != 15 || block.whitener.cols() != 15 ||
      block.residual_whitened.size() != 15) {
    out.analytic_reason = "combined IMU block/preintegration contract invalid";
    return out;
  }
  ImuFactorPoint point;
  if (!factorPoint(tx, &point)) {
    out.analytic_reason = "combined IMU frozen factor point invalid";
    return out;
  }
  out.analytic_input_valid = true;
  gtsam::CombinedImuFactor factor(1, 2, 3, 4, 5, 6, *tx.preintegration);
  gtsam::Matrix h_bias_i;
  factor.evaluateError(
      point.previous_pose, point.previous_velocity,
      point.current_pose, point.current_velocity,
      point.previous_bias, point.current_bias,
      boost::none, boost::none, boost::none, boost::none, h_bias_i, boost::none);
  // An additive sample fault is the negative of a bias perturbation for the
  // 9D preintegrated-motion residual. Bias-continuity rows are not corrupted.
  Eigen::Matrix<double, 15, 6> raw = Eigen::Matrix<double, 15, 6>::Zero();
  raw.topRows<9>() = h_bias_i.topRows(9);
  const Eigen::MatrixXd analytic = block.whitener * raw;
  out.accel_xyz = analytic.leftCols(3);
  out.gyro_xyz = analytic.rightCols(3);
  for (int i = 0; i < 3; ++i) {
    out.accel_axis[i] = out.accel_xyz.col(i);
    out.gyro_axis[i] = out.gyro_xyz.col(i);
  }
  out.bias_jump_accel = block.whitener.rightCols(6).leftCols(3);
  out.bias_jump_gyro = block.whitener.rightCols(3);
  out.analytic_computation_valid = analytic.allFinite() &&
      out.bias_jump_accel.allFinite() && out.bias_jump_gyro.allFinite();
  if (!out.analytic_computation_valid) {
    out.analytic_reason = "analytic IMU sensitivity contains non-finite values";
  }
  return out;
}

ImuFaultSubspaces ImuFaultSubspaceBuilder::verifyFiniteDifferenceOracle(
    const EpochTransaction& tx, const LinearizedFactorBlock& block) const {
  ImuFaultSubspaces out = buildAnalytic(tx, block);
  if (!out.analytic_input_valid || !out.analytic_computation_valid) return out;
  const Eigen::MatrixXd oracle = finiteDifference(tx, block, epsilon_);
  Eigen::MatrixXd analytic(out.accel_xyz.rows(), 6);
  if (out.accel_xyz.rows() == out.gyro_xyz.rows()) {
    analytic << out.accel_xyz, out.gyro_xyz;
  }
  out.oracle_executed = true;
  out.oracle_reintegrations = 12;
  out.oracle_relative_error = analytic.rows() == oracle.rows() &&
          analytic.cols() == oracle.cols()
      ? (analytic - oracle).norm() / std::max(1.0, oracle.norm())
      : std::numeric_limits<double>::infinity();
  out.oracle_verified = out.analytic_computation_valid &&
      std::isfinite(out.oracle_relative_error) &&
      out.oracle_relative_error <= tolerance_;
  out.finite_difference_relative_error = out.oracle_relative_error;
  out.analytic_verified = out.oracle_verified;
  return out;
}

ImuFaultSubspaces ImuFaultSubspaceBuilder::verifyFiniteDifferenceSweep(
    const EpochTransaction& tx, const LinearizedFactorBlock& block,
    const std::vector<double>& epsilons) const {
  ImuFaultSubspaces out = buildAnalytic(tx, block);
  if (!out.analytic_input_valid || !out.analytic_computation_valid) return out;
  if (epsilons.empty()) return out;
  Eigen::MatrixXd analytic(out.accel_xyz.rows(), 6);
  if (out.accel_xyz.rows() != out.gyro_xyz.rows()) return out;
  analytic << out.accel_xyz, out.gyro_xyz;
  out.sweep_executed = true;
  bool all_finite = true;
  double worst = 0.0;
  for (const double epsilon : epsilons) {
    if (!std::isfinite(epsilon) || epsilon <= 0.0) {
      out.sweep_verified = false;
      return out;
    }
    const Eigen::MatrixXd oracle = finiteDifference(tx, block, epsilon);
    const double relative = analytic.rows() == oracle.rows() &&
            analytic.cols() == oracle.cols()
        ? (analytic - oracle).norm() / std::max(1.0, oracle.norm())
        : std::numeric_limits<double>::infinity();
    out.sweep_epsilons.push_back(epsilon);
    out.sweep_relative_errors.push_back(relative);
    out.sweep_reintegrations += 12;
    all_finite = all_finite && std::isfinite(relative);
    worst = std::max(worst, relative);
  }
  out.sweep_worst_relative_error = worst;
  out.sweep_verified = all_finite && worst <= tolerance_;
  return out;
}

}  // namespace uwb_imu_pl
