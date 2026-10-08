#pragma once

#include "uwb_imu_pl/config/integrity_config.hpp"
#include <optional>

namespace uwb_imu_pl {

// Additive, versioned sidecar: none of the frozen config/estimator/FDE
// aggregates are extended. Options are read once from resolved_yaml.
struct EstimationTuningV1 {
  std::optional<Eigen::Matrix<double, 6, 1>> bias_integration_sigmas;
  std::string nominal_initial_guess = "cv";
  int nominal_lm_iterations = 0;
  double nominal_lag_s = 0.0;
  bool nominal_robust_experimental = false;
  bool causal_bootstrap = false;
  bool bootstrap_prefer_below_anchors = false;
  bool bootstrap_exact_uwb_times = false;
  bool bootstrap_uwb_motion_check = false;
  bool bootstrap_seed_only = false;
  bool bootstrap_enforce_below_anchors = false;
};

EstimationTuningV1 readEstimationTuningV1(const IntegrityConfig& config);

struct EstimatorNumericsAuditV1 {
  Eigen::Matrix<double, 6, 1> bias_integration_sigmas = Eigen::Matrix<double,6,1>::Zero();
  std::uint64_t warm_start_attempts = 0;
  std::uint64_t warm_start_accepted = 0;
  double last_warm_start_before = 0.0;
  double last_warm_start_after = 0.0;
  double last_warm_start_ms = 0.0;
};
class IncrementalUwbImuEstimator;
EstimatorNumericsAuditV1 estimatorNumericsAuditV1(
    const IncrementalUwbImuEstimator& estimator);

}  // namespace uwb_imu_pl
