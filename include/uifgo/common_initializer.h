#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "uifgo/config.h"
#include "uifgo/nlos_solver_utils.h"

namespace uifgo {

struct InitializationSeedQualityAudit {
  bool evaluated = false;
  bool accepted = false;
  std::string reason = "NOT_EVALUATED";
  bool graph_keys_valid = false;
  bool values_finite = false;
  bool objective_finite = false;
  bool objective_not_worse = false;
  bool physically_local = false;
  bool uwb_support_present = false;
  bool uwb_quality_not_catastrophic = false;
  double entering_objective = std::numeric_limits<double>::quiet_NaN();
  double terminal_objective = std::numeric_limits<double>::quiet_NaN();
  double objective_increase_allowance = 0.0;
  double entering_median_abs_uwb_residual_m =
      std::numeric_limits<double>::quiet_NaN();
  double terminal_median_abs_uwb_residual_m =
      std::numeric_limits<double>::quiet_NaN();
  double max_position_from_boundary_m = 0.0;
  double position_envelope_m = 0.0;
  double max_velocity_change_from_boundary_mps = 0.0;
  double velocity_change_envelope_mps = 0.0;
};

struct FullInitializationSeedQualityAudit {
  bool evaluated = false;
  bool accepted = false;
  std::string reason = "NOT_EVALUATED";
  bool all_states_finite = false;
  bool motion_finite_and_local = false;
  bool all_uwb_predictions_finite = false;
  size_t state_count = 0;
  size_t uwb_factor_count = 0;
  double max_position_norm_m = std::numeric_limits<double>::quiet_NaN();
  double position_envelope_m = std::numeric_limits<double>::quiet_NaN();
  double max_velocity_norm_mps = std::numeric_limits<double>::quiet_NaN();
  double velocity_envelope_mps = std::numeric_limits<double>::quiet_NaN();
  double max_consecutive_displacement_m =
      std::numeric_limits<double>::quiet_NaN();
  double mean_abs_uwb_residual_m = std::numeric_limits<double>::quiet_NaN();
  double median_abs_uwb_residual_m = std::numeric_limits<double>::quiet_NaN();
  double p95_abs_uwb_residual_m = std::numeric_limits<double>::quiet_NaN();
  double max_abs_uwb_residual_m = std::numeric_limits<double>::quiet_NaN();
  double median_abs_standardized_residual =
      std::numeric_limits<double>::quiet_NaN();
  double p95_abs_standardized_residual =
      std::numeric_limits<double>::quiet_NaN();
  double max_abs_standardized_residual =
      std::numeric_limits<double>::quiet_NaN();
  double cauchy_weight_p05 = std::numeric_limits<double>::quiet_NaN();
  double cauchy_weight_median = std::numeric_limits<double>::quiet_NaN();
  double cauchy_weight_p95 = std::numeric_limits<double>::quiet_NaN();
  double fraction_weight_gt_0_5 = std::numeric_limits<double>::quiet_NaN();
  double fraction_weight_gt_0_1 = std::numeric_limits<double>::quiet_NaN();
  double fraction_weight_lt_0_01 = std::numeric_limits<double>::quiet_NaN();
  double fraction_weight_lt_1e_3 = std::numeric_limits<double>::quiet_NaN();
};

struct CommonInitializationPrefix {
  size_t ordinal = 0;
  // start_state is the fixed causal boundary. It remains as a compatibility
  // field for existing artifact readers.
  size_t start_state = 0;
  // The maximal suffix that may be reoptimized begins here. State zero stays
  // the immutable initialization anchor even when it is inside the lag.
  size_t free_start_state = 0;
  size_t frontier_state = 0;
  double frontier_time_s = std::numeric_limits<double>::quiet_NaN();
  // Total local graph span, including the fixed-boundary IMU bridge.
  double window_duration_s = 0.0;
  // Span constrained by fixed_lag_horizon_s. This may be zero for sparse data.
  double free_window_duration_s = 0.0;
  // Duration of the physical IMU factor from boundary to first free state.
  double boundary_bridge_duration_s = 0.0;
  size_t window_state_count = 0;
  size_t optimized_state_count = 0;
  size_t imu_factor_count = 0;
  size_t uwb_factor_count = 0;
  size_t lm_calls = 0;
  size_t accepted_updates = 0;
  size_t rejected_lambda_trials = 0;
  size_t retry_count = 0;
  std::string lm_reason = "NOT_RUN";
  NavigationStationarityAudit stationarity;
  InitializationSeedQualityAudit seed_quality;
  double initial_objective = std::numeric_limits<double>::quiet_NaN();
  double terminal_objective = std::numeric_limits<double>::quiet_NaN();
  double runtime_s = 0.0;
};

struct CommonInitializationResult {
  bool ok = false;
  std::string status = "INITIALIZATION_FAILED";
  std::string reason = "NOT_RUN";
  std::string identity;
  gtsam::Values values;
  std::vector<CommonInitializationPrefix> prefixes;
  size_t accepted_prefix_count = 0;
  size_t failure_count = 0;
  size_t retry_count = 0;
  size_t failing_frontier_state = std::numeric_limits<size_t>::max();
  double failing_frontier_time_s =
      std::numeric_limits<double>::quiet_NaN();
  double runtime_s = 0.0;
  FullInitializationSeedQualityAudit full_seed_quality;
};

// Initialization-only bounded fixed-lag causal smoother. The earliest state
// in each window is fixed; recent states inside the lag may be revised, while
// states that leave the lag are never reopened. The supplied physical graph is
// const and remains the final estimator graph; this class returns only
// replacement Initial Values. Online calibration variables are deliberately
// unsupported because the common path is fixed-calibration.
class CommonInitializer {
 public:
  explicit CommonInitializer(const Config& config) : config_(config) {}

  CommonInitializationResult Run(
      const gtsam::NonlinearFactorGraph& physical_graph,
      const gtsam::Values& open_loop_values,
      const std::vector<double>& state_times_s,
      const std::vector<size_t>& uwb_factor_indices) const;

  static std::string Identity(const Config& config);

 private:
  Config config_;
};

}  // namespace uifgo
