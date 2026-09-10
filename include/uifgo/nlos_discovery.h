#pragma once

#include <cstddef>
#include <functional>
#include <cstdint>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "uifgo/config.h"
#include "uifgo/nlos_solver_utils.h"
#include "uifgo/nlos_support.h"
#include "uifgo/paper_input.h"
#include "uifgo/types.h"

namespace uifgo {

constexpr const char* kT06AutomaticDiscoveryLabel =
    "T06_AUTOMATIC_DISCOVERY_DEVELOPMENT_ONLY";

enum class FusedLassoStatus {
  CONVERGED,
  INVALID_INPUT,
  FACTORIZATION_FAILED,
  NONFINITE_VALUE,
  MAX_ITERATIONS,
};

const char* FusedLassoStatusName(FusedLassoStatus status);

struct FusedLassoOptions {
  double lambda_l1 = 0.05;
  double lambda_tv = 0.10;
  double rho_scale = 1.0;
  double primal_absolute_tolerance_m = 1e-8;
  double primal_relative_tolerance = 1e-6;
  double dual_absolute_tolerance_objective_per_m = 1e-8;
  double dual_relative_tolerance = 1e-6;
  double kkt_tolerance_objective_per_m = 1e-8;
  double tv_subgradient_tolerance_objective_per_m = 1e-8;
  double active_boundary_epsilon_m = 1e-10;
  size_t max_iterations = 10000;
};

struct FusedLassoTrace {
  size_t iteration = 0;
  double objective = 0.0;
  double primal_residual_m = 0.0;
  double primal_threshold_m = 0.0;
  double dual_residual_objective_per_m = 0.0;
  double dual_threshold_objective_per_m = 0.0;
  double max_kkt_violation_objective_per_m = 0.0;
  double max_tv_subgradient_violation_objective_per_m = 0.0;
};

struct FusedLassoResult {
  FusedLassoStatus status = FusedLassoStatus::INVALID_INPUT;
  std::string reason;
  // The declared solution. u is exactly feasible for u >= 0; b/v are ADMM
  // auxiliary coordinates retained only for primal/dual audit.
  std::vector<double> u;
  std::vector<double> b;
  std::vector<double> v;
  // p=rho*q is the unscaled TV dual in objective/m units.
  std::vector<double> p;
  double rho_objective_per_m2 = 0.0;
  size_t iterations = 0;
  double objective = 0.0;
  double primal_residual_m = 0.0;
  double primal_threshold_m = 0.0;
  double dual_residual_objective_per_m = 0.0;
  double dual_threshold_objective_per_m = 0.0;
  double max_kkt_violation_objective_per_m = 0.0;
  double max_tv_subgradient_violation_objective_per_m = 0.0;
  std::vector<FusedLassoTrace> trace;

  bool converged() const { return status == FusedLassoStatus::CONVERGED; }
};

// Solves the frozen fixed-navigation chain problem to declared tolerances:
// 0.5 sum_i w_i (u_i-e_i)^2 + lambda_l1 sum_i u_i
// + lambda_tv ||D u||_1, u>=0.
FusedLassoResult SolveNonnegativeFusedLassoChain(
    const std::vector<double>& e, const std::vector<double>& weights,
    const FusedLassoOptions& options,
    const std::vector<double>* feasible_warm_start = nullptr);

struct DiscoveryObservation {
  std::uint64_t obs_id = 0;
  int tag_id = 0;
  int anchor_id = 0;
  size_t chain_id = 0;
  size_t active_run_id = 0;
  double sensor_time = 0.0;
  double weight = 0.0;
  double bias_m = 0.0;
};

struct DiscoveryOptions {
  FusedLassoOptions fused_lasso;
  double gap_threshold_s = 1.0;
  double active_bias_min_m = 0.02;
  double change_point_min_m = 0.05;
  double merge_max_difference_m = 0.05;
  size_t short_min_count = 2;
  double short_min_duration_s = 0.01;
  double relative_objective_tolerance = 1e-8;
  double scaled_step_tolerance = 1e-6;
  double observation_bias_scale_m = 1.0;
  double navigation_stationarity_tolerance_objective = 1e-6;
  double gradient_roundoff_safety_factor = 8.0;
  NavigationScales navigation_scales;
  size_t max_outer_iterations = 20;
  CheckedLmOptions conditional_lm;
};

struct DiscoveryContext {
  std::string input_plan_hash;
  std::string source_hash;
  std::string config_hash;
  std::string calibration_hash;
  std::string solver_config_hash;
  // Empty on every legacy/development call. The R08 opt-in validation path
  // binds this truth-free role envelope into the producer request.
  std::string validation_context_sha256;
};

// Development-only observation hook. It is deliberately excluded from all
// scientific configuration and identity inputs because it cannot change a
// solver decision or Values update.
struct DiscoveryDiagnosticRequest {
  size_t conditional_lm_outer_iteration = 0;
  CheckedLmDiagnosticRequest conditional_lm;
};

// Sorts by (link,time,obs_id) and starts a new chain only when dt>T_gap.
// Therefore equality connects and any cross-link transition breaks.
void AssignDiscoveryChains(std::vector<DiscoveryObservation>* observations,
                           double gap_threshold_s);

enum class DiscoveryStatus {
  CONVERGED,
  INVALID_INPUT,
  CONDITIONAL_LM_FAILED,
  CHAIN_SOLVE_FAILED,
  NONFINITE_VALUE,
  OBJECTIVE_INCREASE,
  NAVIGATION_NOT_STATIONARY,
  PARTITION_INVALID,
  MAX_OUTER_ITERATIONS,
};

const char* DiscoveryStatusName(DiscoveryStatus status);

struct DiscoveryIteration {
  size_t outer_iteration = 0;
  size_t conditional_lm_iterations = 0;
  int conditional_lm_inner_iterations = 0;
  double conditional_lm_lambda = 0.0;
  CheckedLmConvergenceDiagnostics conditional_lm_convergence;
  NavigationStationarityAudit conditional_lm_qualification_stationarity;
  double objective_before = 0.0;
  double objective_after = 0.0;
  double allowed_objective_increase = 0.0;
  double relative_objective_change = 0.0;
  double max_navigation_scaled_step = 0.0;
  double max_bias_scaled_step = 0.0;
  double combined_scaled_step = 0.0;
  double max_chain_kkt_objective_per_m = 0.0;
  double max_chain_primal_residual_m = 0.0;
  double max_chain_dual_residual_objective_per_m = 0.0;
  double navigation_gradient_objective = 0.0;
  double navigation_roundoff_allowance_objective = 0.0;
  bool objective_ok = false;
  bool step_ok = false;
  bool chain_optimality_ok = false;
  bool navigation_stationarity_ok = false;
  // Both audits use the exact same conditional-LM navigation Values. Only
  // the fixed observation-bias graph changes across the chain update.
  NavigationStationarityAudit pre_chain_navigation_stationarity;
  NavigationStationarityAudit post_chain_navigation_stationarity;
  bool stationarity_audits_share_navigation_values = false;
  double pre_chain_stationarity_seconds = 0.0;
  double post_chain_stationarity_seconds = 0.0;
  double added_diagnostics_seconds = 0.0;
};

struct DiscoveryResult {
  DiscoveryStatus status = DiscoveryStatus::INVALID_INPUT;
  std::string reason;
  gtsam::Values navigation_values;
  std::vector<DiscoveryObservation> snapshot;
  std::vector<FusedLassoResult> chain_solutions;
  std::vector<DiscoveryIteration> iterations;
  SupportPartition partition;
  bool conditional_lm_attempted = false;
  size_t conditional_lm_attempted_outer_iteration = 0;
  CheckedLmResult last_conditional_lm;
  double added_diagnostics_seconds_total = 0.0;

  bool converged() const { return status == DiscoveryStatus::CONVERGED; }
};

// A successful Stage-1-only ablation result. The per-observation b_i values
// remain a regularized snapshot rather than being inserted as Gaussian state
// variables. Consequently no joint covariance is defined for this method.
struct Stage1RegularizedResult {
  bool valid = false;
  std::string status = "INVALID";
  std::string reason;
  std::vector<DiscoveryObservation> observation_bias_snapshot;
  gtsam::Values navigation_values;
  gtsam::NonlinearFactorGraph physical_graph;
  std::vector<FactorMeta> physical_factor_metadata;
  SupportPartition partition;
  std::vector<DiscoveryIteration> iterations;
  double objective_physical = 0.0;
  double objective_l1 = 0.0;
  double objective_tv = 0.0;
  double objective_total = 0.0;
  std::string graph_linearization_sha256;
  std::string values_sha256;
  std::string snapshot_sha256;
  std::string result_id;
  std::string covariance_status =
      "NOT_APPLICABLE_REGULARIZED_STAGE1_NON_GAUSSIAN";
};

// Materializes the physical graph h+beta+b_i-z at the converged Stage-1
// snapshot. L1/TV terms are reported only in the objective and are never added
// as graph factors or Gaussian information.
Stage1RegularizedResult BuildStage1RegularizedResult(
    const DiscoveryResult& discovery,
    const gtsam::NonlinearFactorGraph& base_graph,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const DiscoveryContext& context, const DiscoveryOptions& options);

// Pure partition operation used by U11. Input order is ignored; chain_id and
// active_run_id forbid merge across gaps or inactive runs.
SupportPartition BuildAutomaticSupportPartition(
    std::vector<DiscoveryObservation> immutable_snapshot,
    const DiscoveryOptions& options,
    const DiscoveryContext& context = DiscoveryContext());

// Explicit A18 development numerical-policy injection. NOT a read-only observer.
// Range constants are copied at the exact factory call that constructs the graph.
struct DevelopmentRangeConstant {
  size_t factor_index;
  gtsam::Key pose_key;
  gtsam::Point3 anchor, lever;
  double measurement, sigma, conditional_beta;
  std::uint64_t obs_id;
};

// Shared, exact development identity contract.  R05 fixes a producer/runner
// handshake defect; it deliberately keeps the existing R04 payload schema.
extern const char kA19DevelopmentStage1Schema[];
extern const char kA19DevelopmentStage1Policy[];
extern const char kDevelopmentOnlyRole[];
extern const char kDevelopmentNonconsumableProvider[];
extern const char kA19ValidationStage1Schema[];
extern const char kValidationRole[];
extern const char kValidationNonconsumableProvider[];

using DevelopmentStage1Navigation =
    std::function<CheckedLmResult(
        size_t, const gtsam::NonlinearFactorGraph&, const gtsam::Values&,
        const CheckedLmOptions&,
        const std::vector<DevelopmentRangeConstant>&)>;

struct DevelopmentStage1Request {
  std::string policy, implementation_identity, role;
  std::string output_schema = "A18_STAGE1_DIAGNOSTIC_ONLY";
  std::string output_provider = "development_nonconsumable";
  std::string validation_context_sha256;
  DevelopmentStage1Navigation conditional_navigation;
  std::function<void(const DiscoveryIteration&)> outer_observer;
};

struct DevelopmentStage1ContractCheck {
  bool accepted = false;
  std::string reason;
};

DevelopmentStage1Request MakeA19DevelopmentStage1Request(
    const std::string& implementation_identity,
    DevelopmentStage1Navigation conditional_navigation);

// This is the single validation used by both prepare/preflight and the actual
// producer entry.  It performs no graph construction or optimizer call.
DevelopmentStage1ContractCheck ValidateDevelopmentStage1Request(
    const DevelopmentStage1Request& request,
    const DiscoveryContext& context, const DiscoveryOptions& options,
    bool diagnostic_request_present = false);

class AutomaticSupportProvider {
 public:
  explicit AutomaticSupportProvider(DiscoveryOptions options)
      : options_(std::move(options)) {}

  // Deliberately has no oracle path, GT, labels, or reference trajectory.
  DiscoveryResult Run(
      const gtsam::NonlinearFactorGraph& base_graph,
      const gtsam::Values& base_values,
      const std::vector<FactorMeta>& base_uwb_factor_metadata,
      const PaperInputPlan& plan, const Config& cfg,
      const DiscoveryContext& context) const;

  DiscoveryResult Run(
      const gtsam::NonlinearFactorGraph& base_graph,
      const gtsam::Values& base_values,
      const std::vector<FactorMeta>& base_uwb_factor_metadata,
      const PaperInputPlan& plan, const Config& cfg,
      const DiscoveryContext& context,
      const DiscoveryDiagnosticRequest* diagnostic_request) const;

  // No ordinary runner supplies this request. Outputs are non-consumable diagnostics.
  DiscoveryResult RunDevelopmentStage1(
      const gtsam::NonlinearFactorGraph& base_graph,
      const gtsam::Values& base_values,
      const std::vector<FactorMeta>& base_uwb_factor_metadata,
      const PaperInputPlan& plan, const Config& cfg,
      const DiscoveryContext& context,
      const DiscoveryDiagnosticRequest* diagnostic_request,
      const DevelopmentStage1Request* development_request) const;

 private:
  DiscoveryOptions options_;
};

}  // namespace uifgo
