#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/VectorValues.h>

namespace uifgo {

struct NavigationScales {
  double pose_rotation_rad = 1.0;
  double pose_translation_m = 1.0;
  double velocity_mps = 1.0;
  double accel_bias_mps2 = 1.0;
  double gyro_bias_radps = 1.0;
};

struct NavigationStationarityAudit {
  bool valid = false;
  std::string reason;
  double max_pose_rotation_gradient_objective_per_rad = 0.0;
  double max_pose_translation_gradient_objective_per_m = 0.0;
  double max_velocity_gradient_objective_per_mps = 0.0;
  double max_accel_bias_gradient_objective_per_mps2 = 0.0;
  double max_gyro_bias_gradient_objective_per_radps = 0.0;
  double max_scaled_gradient_objective = 0.0;
  double roundoff_allowance_objective = 0.0;
  std::uint64_t dominant_key = 0;
  std::string dominant_key_name;
  size_t dominant_coordinate = 0;
  std::string dominant_category;
  double dominant_native_gradient_objective = 0.0;
  double dominant_physical_scale = 0.0;
  double dominant_scaled_gradient_objective = 0.0;
  double dominant_absolute_factor_gradient_sum_objective = 0.0;
  double dominant_roundoff_allowance_objective = 0.0;
  bool stationary = false;
};

enum class ConditionalLmPolicy {
  GTSAM_CHECK_ONLY_V1,
  GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1,
  GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2,
};

const char* ConditionalLmPolicyName(ConditionalLmPolicy policy);

struct CheckedLmOptions {
  int max_iterations = 100;
  double relative_tolerance = 1e-6;
  double absolute_tolerance = 1e-8;
  ConditionalLmPolicy policy = ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1;
  NavigationScales navigation_scales;
  double navigation_stationarity_tolerance_objective = 1e-6;
  double gradient_roundoff_safety_factor = 8.0;
  // Development-only Stage2 handoff guard.  NaN disables the amendment and
  // preserves every legacy/Stage1 conditional-LM return.
  double inexact_handoff_scaled_step_tolerance =
      std::numeric_limits<double>::quiet_NaN();
};

struct InexactHandoffTrialAudit {
  size_t trial_index = 0;
  bool certificate_valid = false;
  std::string certificate_status = "NOT_EVALUATED";
  double predicted_lo = std::numeric_limits<double>::quiet_NaN();
  double predicted_hi = std::numeric_limits<double>::quiet_NaN();
  double actual_decrease_lo = std::numeric_limits<double>::quiet_NaN();
  double actual_decrease_hi = std::numeric_limits<double>::quiet_NaN();
  double scaled_navigation_step = std::numeric_limits<double>::quiet_NaN();
};

// Separate from inner convergence: a qualified handoff remains an explicitly
// non-converged fixed-c block and may only let the Stage2 caller execute the
// already-existing exact nonnegative c update.
struct InexactHandoffAudit {
  bool enabled = false;
  bool qualified = false;
  std::string status = "NOT_ENABLED";
  std::string trigger_reason = "NOT_ENABLED";
  size_t handoff_count = 0;
  size_t accepted_update_count = 0;
  bool last_accepted_generic_convergence = false;
  bool last_accepted_stationarity_valid = false;
  double last_accepted_scaled_navigation_step =
      std::numeric_limits<double>::quiet_NaN();
  double last_accepted_scaled_navigation_gradient =
      std::numeric_limits<double>::quiet_NaN();
  double last_accepted_gradient_roundoff_allowance =
      std::numeric_limits<double>::quiet_NaN();
  double scaled_step_tolerance = std::numeric_limits<double>::quiet_NaN();
  double objective_increase_allowance =
      std::numeric_limits<double>::quiet_NaN();
  std::string last_accepted_values_identity;
  std::vector<InexactHandoffTrialAudit> last_lambda_search_trials;
};

// Read-only diagnostics for the last authoritative linked-GTSAM
// checkConvergence call. The trigger flags are independently auditable; they
// never replace or influence checkConvergence as the stopping decision.
struct CheckedLmConvergenceDiagnostics {
  bool initial_error_valid = false;
  double initial_error = std::numeric_limits<double>::quiet_NaN();
  bool check_evaluated = false;
  std::string check_status = "NOT_EVALUATED";
  double previous_error = std::numeric_limits<double>::quiet_NaN();
  double current_error = std::numeric_limits<double>::quiet_NaN();
  double relative_tolerance = std::numeric_limits<double>::quiet_NaN();
  double absolute_tolerance = std::numeric_limits<double>::quiet_NaN();
  double optimizer_internal_relative_tolerance =
      std::numeric_limits<double>::quiet_NaN();
  bool optimizer_internal_small_change_stop_enabled = false;
  double error_tolerance = std::numeric_limits<double>::quiet_NaN();
  double absolute_decrease = std::numeric_limits<double>::quiet_NaN();
  double relative_decrease = std::numeric_limits<double>::quiet_NaN();
  bool relative_decrease_valid = false;
  bool relative_tolerance_enabled = false;
  bool decrease_predicates_reached_by_linked_check = false;
  bool error_tolerance_triggered = false;
  bool absolute_tolerance_triggered = false;
  bool relative_tolerance_triggered = false;
  bool check_result = false;
  bool predicate_union_matches_check_result = false;
  double added_diagnostics_seconds = 0.0;
  std::string policy_version = "GTSAM_CHECK_ONLY_V1";
  bool stationarity_qualification_enabled = false;
  size_t convergence_check_count = 0;
  size_t generic_convergence_count = 0;
  size_t generic_convergence_last_iteration = 0;
  size_t qualification_evaluation_count = 0;
  bool qualification_last_evaluated = false;
  bool qualification_last_passed = false;
  std::string qualification_status = "NOT_ENABLED";
  size_t qualification_last_iteration = 0;
  double qualification_seconds = 0.0;
  size_t iterate_call_count = 0;
  // COMPLETE means both counts below are exact. Any INCOMPLETE_* status means
  // they are only confirmed completed lower bounds: iterate() may have thrown
  // before linked state counters were committed, or linked state deltas were
  // otherwise insufficient to classify a normal return. NOT_EXECUTED means no
  // iterate() call was made.
  std::string lambda_trial_accounting_status = "NOT_EXECUTED";
  size_t lambda_trial_count = 0;
  size_t rejected_lambda_trial_count = 0;
  size_t accepted_update_count = 0;
  size_t no_update_return_count = 0;
};

// Optional, read-only engineering trace of one call to the linked GTSAM
// optimizer. These fields do not participate in convergence or state updates.
struct CheckedLmFirstTryDiagnostics {
  bool valid = false;
  std::string reason;
  bool linear_system_solved = false;
  double delta_norm = std::numeric_limits<double>::quiet_NaN();
  double max_abs_delta = std::numeric_limits<double>::quiet_NaN();
  double old_linearized_error = std::numeric_limits<double>::quiet_NaN();
  double new_linearized_error = std::numeric_limits<double>::quiet_NaN();
  double linearized_cost_change = std::numeric_limits<double>::quiet_NaN();
  double linearized_resolution_threshold =
      std::numeric_limits<double>::quiet_NaN();
  bool linearized_step_valid = false;
  bool linearized_change_resolvable = false;
  double tentative_error = std::numeric_limits<double>::quiet_NaN();
  double tentative_cost_change = std::numeric_limits<double>::quiet_NaN();
  bool model_fidelity_valid = false;
  double model_fidelity = std::numeric_limits<double>::quiet_NaN();
  double minimum_model_fidelity = std::numeric_limits<double>::quiet_NaN();
  bool model_fidelity_passed = false;
  double small_cost_change_threshold =
      std::numeric_limits<double>::quiet_NaN();
  bool small_cost_change = false;
  std::string predicted_first_try_branch;
};

struct CheckedLmDirectionFiniteDifferencePoint {
  double step = std::numeric_limits<double>::quiet_NaN();
  double objective_plus = std::numeric_limits<double>::quiet_NaN();
  double objective_minus = std::numeric_limits<double>::quiet_NaN();
  double central_derivative = std::numeric_limits<double>::quiet_NaN();
  double absolute_difference_from_analytic =
      std::numeric_limits<double>::quiet_NaN();
  double agreement_tolerance = std::numeric_limits<double>::quiet_NaN();
  bool agrees = false;
};

struct CheckedLmDirectionFactorDiagnostics {
  size_t factor_index = 0;
  std::string dynamic_type;
  std::vector<std::uint64_t> keys;
  double error_at_base = std::numeric_limits<double>::quiet_NaN();
  double error_at_tentative = std::numeric_limits<double>::quiet_NaN();
  double decrease = std::numeric_limits<double>::quiet_NaN();
  bool directional_derivative_valid = false;
  std::string directional_derivative_reason = "NOT_EVALUATED";
  double gradient_dot_unit_direction =
      std::numeric_limits<double>::quiet_NaN();
  std::vector<CheckedLmDirectionFiniteDifferencePoint>
      directional_finite_difference;
};

// Parsed only from the actual linked TRYDELTA output produced inside the
// authoritative optimizer.iterate() call. Evaluation fields are computed
// afterwards on the same live graph/base Values and never affect the solver.
struct CheckedLmTrialDirectionDiagnostics {
  size_t call_index = 0;
  size_t trial_index_within_call = 0;
  double lambda = std::numeric_limits<double>::quiet_NaN();
  bool parsed = false;
  std::string parse_reason;
  size_t declared_key_count = 0;
  size_t parsed_key_count = 0;
  size_t parsed_dimension_count = 0;
  double linked_reported_delta_norm =
      std::numeric_limits<double>::quiet_NaN();
  double parsed_delta_norm = std::numeric_limits<double>::quiet_NaN();
  gtsam::VectorValues delta;
  bool accepted_retract_matches = false;
  double accepted_retract_max_difference = std::numeric_limits<double>::quiet_NaN();
  bool selected_for_evaluation = false;
  bool evaluation_valid = false;
  std::string evaluation_reason = "NOT_SELECTED";
  double base_graph_error = std::numeric_limits<double>::quiet_NaN();
  double tentative_graph_error = std::numeric_limits<double>::quiet_NaN();
  double direct_graph_decrease = std::numeric_limits<double>::quiet_NaN();
  double factor_old_sum_double = std::numeric_limits<double>::quiet_NaN();
  double factor_tentative_sum_double =
      std::numeric_limits<double>::quiet_NaN();
  double factor_decrease_sum_double =
      std::numeric_limits<double>::quiet_NaN();
  long double factor_old_sum_long_double = 0.0L;
  long double factor_tentative_sum_long_double = 0.0L;
  long double factor_decrease_sum_long_double = 0.0L;
  double old_linearized_error = std::numeric_limits<double>::quiet_NaN();
  double new_linearized_error = std::numeric_limits<double>::quiet_NaN();
  double predicted_decrease = std::numeric_limits<double>::quiet_NaN();
  double gradient_dot_delta = std::numeric_limits<double>::quiet_NaN();
  double gradient_dot_unit_direction =
      std::numeric_limits<double>::quiet_NaN();
  std::vector<CheckedLmDirectionFiniteDifferencePoint>
      directional_finite_difference;
  std::vector<CheckedLmDirectionFactorDiagnostics> factors;
};

struct CheckedLmCallDiagnostics {
  size_t call_index = 0;
  size_t optimizer_iterations_before = 0;
  size_t optimizer_iterations_after = 0;
  int inner_iterations_before = 0;
  int inner_iterations_after = 0;
  double lambda_before = std::numeric_limits<double>::quiet_NaN();
  double lambda_after = std::numeric_limits<double>::quiet_NaN();
  double error_before = std::numeric_limits<double>::quiet_NaN();
  double error_after = std::numeric_limits<double>::quiet_NaN();
  double accepted_values_delta_norm =
      std::numeric_limits<double>::quiet_NaN();
  double accepted_values_max_abs_delta =
      std::numeric_limits<double>::quiet_NaN();
  bool accepted_state_update = false;
  size_t rejected_lambda_trials_before_acceptance = 0;
  std::string observed_return_class;
  NavigationStationarityAudit stationarity_after;
  CheckedLmFirstTryDiagnostics first_try;
  gtsam::Values values_before;
  std::string linked_trydelta_stdout;
  std::vector<CheckedLmTrialDirectionDiagnostics> actual_trial_directions;
};

struct CheckedLmFiniteDifferencePoint {
  double step = std::numeric_limits<double>::quiet_NaN();
  double objective_plus = std::numeric_limits<double>::quiet_NaN();
  double objective_minus = std::numeric_limits<double>::quiet_NaN();
  double objective_change_plus = std::numeric_limits<double>::quiet_NaN();
  double objective_change_minus = std::numeric_limits<double>::quiet_NaN();
  double central_derivative = std::numeric_limits<double>::quiet_NaN();
  double absolute_difference_from_analytic =
      std::numeric_limits<double>::quiet_NaN();
  double agreement_tolerance = std::numeric_limits<double>::quiet_NaN();
  bool agrees = false;
};

struct CheckedLmFiniteDifferenceDiagnostics {
  bool valid = false;
  std::string reason;
  std::string key;
  std::uint64_t key_value = 0;
  size_t coordinate = 0;
  size_t dimension = 0;
  std::string coordinate_unit;
  double coordinate_scale = std::numeric_limits<double>::quiet_NaN();
  double objective = std::numeric_limits<double>::quiet_NaN();
  double analytic_gradient = std::numeric_limits<double>::quiet_NaN();
  double scaled_analytic_gradient =
      std::numeric_limits<double>::quiet_NaN();
  std::vector<CheckedLmFiniteDifferencePoint> points;
};

struct CheckedLmFactorDiagnostics {
  size_t factor_index = 0;
  std::string dynamic_type;
  std::vector<std::uint64_t> keys;
  double error_at_capture_start = std::numeric_limits<double>::quiet_NaN();
  double error_at_capture_final = std::numeric_limits<double>::quiet_NaN();
};

struct CheckedLmDiagnosticRequest {
  // A15: passive capture only; no extra solve, FD, or budget extension.
  bool passive_terminal_capture = false;
  // A11: default-off, outer1-only caller must stop before any chain.
  bool first_block_budget_diagnostic = false;
  bool emit_linked_gtsam_trylambda = false;
  bool capture_linked_gtsam_trydelta = false;
  std::vector<double> direction_evaluation_lambdas;
  std::vector<double> direction_finite_difference_steps;
  std::vector<double> finite_difference_steps;
  bool run_fixed_checkpoint_lm_recovery = false;
  size_t fixed_checkpoint_max_calls = 50;
  double fixed_checkpoint_max_seconds = 10.0;
};

struct FixedCheckpointLmCallDiagnostics {
  CheckedLmCallDiagnostics linked;
  bool accepted_strict_descent = false;
  bool external_generic_convergence = false;
  NavigationStationarityAudit stationarity;
  bool stationary_qualified = false;
};

struct FixedCheckpointLmArmDiagnostics {
  std::string arm;
  bool valid = false;
  std::string reason = "NOT_RUN";
  bool internal_relative_tolerance_changed = false;
  double internal_relative_tolerance =
      std::numeric_limits<double>::quiet_NaN();
  double external_relative_tolerance =
      std::numeric_limits<double>::quiet_NaN();
  double external_absolute_tolerance =
      std::numeric_limits<double>::quiet_NaN();
  double initial_error = std::numeric_limits<double>::quiet_NaN();
  double final_error = std::numeric_limits<double>::quiet_NaN();
  double initial_lambda = std::numeric_limits<double>::quiet_NaN();
  double final_lambda = std::numeric_limits<double>::quiet_NaN();
  double lambda_upper_bound = std::numeric_limits<double>::quiet_NaN();
  double elapsed_seconds = 0.0;
  size_t call_count = 0;
  size_t lambda_trial_count = 0;
  size_t rejected_lambda_trial_count = 0;
  size_t accepted_update_count = 0;
  size_t accepted_strict_descent_count = 0;
  bool timed_out = false;
  bool stationary_qualified = false;
  NavigationStationarityAudit final_stationarity;
  gtsam::Values values_at_final;
  std::vector<FixedCheckpointLmCallDiagnostics> calls;
};

struct FixedCheckpointLmRecoveryDiagnostics {
  bool requested = false;
  bool valid = false;
  std::string reason = "NOT_REQUESTED";
  double captured_lambda = std::numeric_limits<double>::quiet_NaN();
  double captured_error = std::numeric_limits<double>::quiet_NaN();
  size_t max_calls_per_arm = 0;
  double max_seconds_per_arm = 0.0;
  FixedCheckpointLmArmDiagnostics current_behavior;
  FixedCheckpointLmArmDiagnostics continue_lambda_search;
};

struct CheckedLmDiagnosticCapture {
  bool first_block_budget_diagnostic = false;
  bool derivative_gate_passed = false;
  std::string continuation_status = "NOT_REQUESTED";
  gtsam::Values values_at_call50;
  CheckedLmFiniteDifferenceDiagnostics finite_difference_at_call50;
  bool requested = false;
  bool valid = false;
  std::string reason;
  size_t graph_factor_count = 0;
  size_t values_key_count = 0;
  gtsam::Values values_at_start;
  gtsam::Values values_at_final;
  std::vector<CheckedLmCallDiagnostics> calls;
  std::vector<CheckedLmFactorDiagnostics> factors;
  NavigationStationarityAudit stationarity_at_final;
  CheckedLmFiniteDifferenceDiagnostics finite_difference;
  FixedCheckpointLmRecoveryDiagnostics fixed_checkpoint_recovery;
};

// Fail-closed gate for the frozen A11 full-direction and coordinate checks.
bool FirstBlockDerivativesConsistent(const CheckedLmDiagnosticCapture& capture);

struct CheckedLmResult {
  bool converged = false;
  std::string reason;
  gtsam::Values values;
  size_t iterations = 0;
  int inner_iterations = 0;
  double lambda = 0.0;
  CheckedLmConvergenceDiagnostics convergence;
  NavigationStationarityAudit last_qualification_stationarity;
  CheckedLmDiagnosticCapture diagnostic;
  InexactHandoffAudit inexact_handoff;
  bool fixed_checkpoint_recovery_used = false;
  size_t fixed_checkpoint_restart_count = 0;
};

struct ScaledStepAudit {
  bool valid = false;
  std::string reason;
  double max_navigation_step = 0.0;
  double max_bias_step = 0.0;
  double max_combined_step = 0.0;
};

bool GraphAndValuesKeysMatch(const gtsam::NonlinearFactorGraph& graph,
                             const gtsam::Values& values);

CheckedLmResult RunCheckedConditionalLm(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const CheckedLmOptions& options);

CheckedLmResult RunCheckedConditionalLm(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const CheckedLmOptions& options,
    const CheckedLmDiagnosticRequest* diagnostic_request);

// Reuses the V2 checked LM on an identical fixed-C graph when its configured
// call block ends before navigation stationarity or exhausts lambda. Each
// restart begins at the exact last accepted Values and otherwise uses the same
// options. No checkpoint is returned as a successful/inexact solution.
CheckedLmResult RunCheckedConditionalLmWithFixedCheckpointRecovery(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const CheckedLmOptions& options, size_t max_restarts,
    size_t max_total_calls);

NavigationStationarityAudit AuditNavigationStationarity(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const NavigationScales& scales, double tolerance_objective,
    double roundoff_safety_factor);

// Uses the same local-coordinate convention and analytic gradient as the
// checked-LM diagnostic. This overload exposes a caller-selected navigation
// coordinate so independent audits can sample more than the dominant entry.
CheckedLmFiniteDifferenceDiagnostics
CheckNavigationCoordinateByFiniteDifference(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const NavigationScales& scales, gtsam::Key key, size_t coordinate,
    const std::vector<double>& steps);

// Uses Values::localCoordinates for all navigation manifold coordinates and
// combines that step with an explicitly scaled Euclidean dynamic-bias block.
ScaledStepAudit AuditNavigationAndBiasScaledStep(
    const gtsam::Values& before_navigation,
    const gtsam::Values& after_navigation,
    const std::vector<double>& before_bias_m,
    const std::vector<double>& after_bias_m,
    const NavigationScales& scales, double bias_scale_m);

// Shared T04 refit variant. In addition to X/V/B local coordinates it accepts
// scalar C keys, scaled by amplitude_scale_m.
double MaxScaledValuesStep(const gtsam::Values& before,
                           const gtsam::Values& after,
                           const NavigationScales& scales,
                           double amplitude_scale_m);

double Binary64ObjectiveIncreaseAllowance(double before, double after);

}  // namespace uifgo
