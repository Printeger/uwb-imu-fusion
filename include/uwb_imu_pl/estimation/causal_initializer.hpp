#pragma once
#include "uwb_imu_pl/config/integrity_config.hpp"

namespace uwb_imu_pl {
struct CausalInitializationV1 {
  NavigationState state;
  Eigen::Matrix<double, 15, 1> prior_sigmas;
  ImuMeasurement boundary;
  bool stationary = false;
  double gyro_rms_radps = 0.0;
  double accel_std_mps2 = 0.0;
  double accel_norm_error_mps2 = 0.0;
  double window_cost_before = 0.0;
  double window_cost_after = 0.0;
  std::string motion_status = "LEGACY_IMU_ONLY";
  double uwb_speed_upper_mps = 0.0;
  std::size_t discrete_nodes = 0;
  std::size_t range_factors = 0;
  double maximum_range_time_error_s = 0.0;
  Eigen::Vector3d position_seed_m = Eigen::Vector3d::Zero();
  std::string position_prior_source = "LEGACY_RANGE_DERIVED_SEED";
};

// Consumes sensor data at or before first_imu + 2 s. The returned timestamp
// is that boundary; those measurements must not be inserted in the estimator
// again. Both direct and ROS call this same implementation. No GT input.
CausalInitializationV1 initializeCausallyV1(
    const std::vector<ImuMeasurement>& imu,
    const std::vector<UwbMeasurement>& ranges,
    const IntegrityConfig& config, const Eigen::Vector3d& fallback_lever,
    bool prefer_below_anchors = false);
}  // namespace uwb_imu_pl
