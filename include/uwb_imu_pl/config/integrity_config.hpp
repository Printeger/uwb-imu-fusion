#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace uwb_imu_pl {

struct SnapshotConfig {
  int dimensions = 3;
  int max_iterations = 30;
  double step_tolerance_m = 1e-7;
  double rank_tolerance = 1e-10;
  double max_condition_number = 1e10;
};

struct IncrementalConfig {
  double relinearize_threshold = 0.1;
  int relinearize_skip = 1;
  double smoothness_sigma_m = 0.5;
  double epoch_bin_s = 0.02;
  double max_time_skew_s = 0.01;
  bool enable_method_b = false;
  double method_b_max_condition = 1e10;
  std::uint32_t fixed_lag_epochs = 0;
};

struct ImuNoiseConfig {
  double accelerometer_sigma = 0.0;
  double gyroscope_sigma = 0.0;
  double accelerometer_bias_rw_sigma = 0.0;
  double gyroscope_bias_rw_sigma = 0.0;
  double gravity_mps2 = 9.80665;
  double max_gap_s = 0.02;
};

struct OutputConfig {
  std::string root;
  bool write_residuals = true;
  bool write_timing = true;
  bool write_global_diagnostics = false;
};

struct RealtimeConfig {
  std::string world_frame;
  std::string body_frame;
  std::string imu_topic;
  std::string uwb_topic;
  std::string odometry_topic;
  std::string integrity_topic;
  std::string diagnostics_topic;
  Eigen::Vector3d initial_position_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d initial_velocity_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d lever_arm_body_m = Eigen::Vector3d::Zero();
  double range_sigma_m = 0.10;
  Eigen::Matrix<double, 15, 1> prior_sigmas =
      Eigen::Matrix<double, 15, 1>::Ones();
};

struct IntegrityConfig {
  std::uint64_t seed = 0;
  SnapshotConfig snapshot;
  IncrementalConfig incremental;
  ImuNoiseConfig imu;
  RiskBudget risk;
  OutputConfig output;
  RealtimeConfig realtime;
  std::vector<AnchorRecord> anchors;
  std::string resolved_yaml;
  std::string source_path;
  std::string config_hash;
};

class IntegrityConfigLoader {
 public:
  // This loader is deliberately strict: every safety-relevant field is
  // required, unknown keys are rejected, and no random seed is implicit.
  // A non-empty fixed-lag override is applied before validation,
  // serialization, and hashing. An empty optional uses the YAML value.
  static IntegrityConfig load(
      const std::string& yaml_path,
      const std::optional<std::string>& fixed_lag_epochs_override =
          std::nullopt);
};

}  // namespace uwb_imu_pl
