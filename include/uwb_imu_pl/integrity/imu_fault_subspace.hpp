#pragma once

#include "uwb_imu_pl/estimation/epoch_transaction.hpp"
#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"

#include <array>

namespace uwb_imu_pl {

struct ImuFaultSubspaces {
  Eigen::MatrixXd accel_xyz;
  Eigen::MatrixXd gyro_xyz;
  std::array<Eigen::VectorXd, 3> accel_axis;
  std::array<Eigen::VectorXd, 3> gyro_axis;
  Eigen::MatrixXd bias_jump_accel;
  Eigen::MatrixXd bias_jump_gyro;
  double finite_difference_relative_error =
      std::numeric_limits<double>::infinity();
  bool analytic_input_valid = false;
  bool analytic_computation_valid = false;
  bool oracle_executed = false;
  double oracle_relative_error = std::numeric_limits<double>::infinity();
  bool oracle_verified = false;
  std::uint32_t oracle_reintegrations = 0;
  std::string analytic_reason;
  // Compatibility aliases for diagnostic readers predating R1. They mirror
  // oracle_relative_error/oracle_verified and are false/infinite when the
  // production path intentionally does not execute the oracle.
  bool analytic_verified = false;
  std::string residual_ordering = "rotation,position,velocity,bias_accel,bias_gyro";
  std::string frame_contract = "GTSAM Pose3 local tangent; IMU sample body frame";
};

class ImuFaultSubspaceBuilder {
 public:
  explicit ImuFaultSubspaceBuilder(double finite_difference_epsilon = 1e-6,
                                   double verification_tolerance = 2e-4)
      : epsilon_(finite_difference_epsilon), tolerance_(verification_tolerance) {}
  ImuFaultSubspaces build(
      const EpochTransaction& transaction,
      const LinearizedFactorBlock& combined_imu_block) const;
  ImuFaultSubspaces buildAnalytic(
      const EpochTransaction& transaction,
      const LinearizedFactorBlock& combined_imu_block) const;
  ImuFaultSubspaces verifyFiniteDifferenceOracle(
      const EpochTransaction& transaction,
      const LinearizedFactorBlock& combined_imu_block) const;

 private:
  Eigen::MatrixXd finiteDifference(
      const EpochTransaction& transaction,
      const LinearizedFactorBlock& block) const;
  double epsilon_;
  double tolerance_;
};

}  // namespace uwb_imu_pl
