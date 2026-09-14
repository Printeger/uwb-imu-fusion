#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "uifgo/nlos_inference.h"
#include "uifgo/nlos_solver_utils.h"
#include "uifgo/types.h"

namespace uifgo {

struct RawGaussianReference;

enum class PaperMethod {
  LCB_PARTIAL,
  LCB_FIXED_FULL,
  SUPPRESS_ALL,
  ALL_RANGE,
  ROBUST_HUBER,
  ROBUST_CAUCHY,
  FIXED_REJECTION,
  STRUCTURED_BIAS_ONLY,
  STRUCTURED_DEBIAS,
  FIT_ONLY,
  S_FIT,
  FULL_GATE,
  ETA_ONLY,
  NOMINAL_CURVATURE,
  ORACLE_REFERENCE,
};

enum class PaperExecutionType {
  BASELINE_TRAJECTORY,
  STAGE1_TRAJECTORY,
  AUTOMATIC_STAGE2_TRAJECTORY,
  CACHE_DIAGNOSTIC,
  FINAL_TRAJECTORY,
  EVALUATION_REFERENCE,
};

struct PaperMethodSpec {
  PaperMethod method;
  const char* canonical_name;
  PaperExecutionType default_execution_type;
  bool requires_stage1;
  bool requires_stage2_cache;
  bool permits_final_trajectory;
};

const std::vector<PaperMethodSpec>& CanonicalPaperMethodRegistry();
const PaperMethodSpec& PaperMethodByName(const std::string& name);
const char* PaperExecutionTypeName(PaperExecutionType type);

struct PolicyThresholds {
  double tau_eta = 0.0;
  double tau_s_m = 0.0;
  double tau_gamma = 0.0;
  double tau_nominal_curvature_m2_inv = 0.0;
  std::string parameter_provenance;
};

// Policy-specific decision logic for immutable Stage-3 score records.  It
// deliberately does not run Stage 4; callers select that separately from the
// execution type. Threshold equality is accepted for every comparator.
std::vector<GroupDecisionRecord> FreezePaperPolicyDecisions(
    PaperMethod method,
    const std::vector<GroupRecoverabilityScore>& scores,
    const PolicyThresholds& thresholds);

struct BaselineOptions {
  PaperMethod method = PaperMethod::ALL_RANGE;
  // Required for robust methods; dimensionless standardized-residual scale.
  double robust_scale = 0.0;
  // Required for fixed_rejection_v1; equality is retained.
  double rejection_threshold_sigma = 0.0;
  std::string parameter_provenance;
  // Required by the paper production path. One finite, strictly increasing
  // timestamp per contiguous X(k) state enables temporal/stationarity checks.
  std::vector<double> keyframe_times_s;
  CheckedLmOptions lm;
};

struct BaselineResult {
  bool valid = false;
  std::string status = "ESTIMATION_FAILED";
  std::string reason;
  std::string initialization_path;
  gtsam::NonlinearFactorGraph final_graph;
  gtsam::Values final_values;
  std::vector<FactorMeta> final_factor_metadata;
  std::vector<std::uint64_t> kept_uwb_obs_ids;
  std::vector<std::uint64_t> rejected_uwb_obs_ids;
  CheckedLmResult preliminary;
  CheckedLmResult final;
  SolverCertificate solver_certificate;
};

struct IntermediateSeedQualityOptions {
  // Frozen catastrophe envelopes supplied by the caller from the existing
  // configuration. They are audit limits, not optimizer parameters.
  double position_envelope_m = 0.0;
  double velocity_envelope_mps = 0.0;
};

struct IntermediateSeedQualityAudit {
  std::string policy_version = "INTERMEDIATE_OPTIMIZATION_SEED_QUALITY_V1";
  bool evaluated = false;
  bool accepted = false;
  std::string reason = "NOT_EVALUATED";
  bool graph_keys_valid = false;
  bool all_xvb_finite = false;
  bool objective_finite = false;
  bool objective_not_worse = false;
  bool uwb_residuals_finite = false;
  bool physically_plausible = false;
  size_t state_count = 0;
  size_t uwb_factor_count = 0;
  double input_objective = 0.0;
  double terminal_objective = 0.0;
  double objective_roundoff_allowance = 0.0;
  double max_position_norm_m = 0.0;
  double max_velocity_norm_mps = 0.0;
  double position_envelope_m = 0.0;
  double velocity_envelope_mps = 0.0;
  double max_abs_uwb_residual_m = 0.0;
  double max_abs_standardized_uwb_residual = 0.0;
};

// This role is intentionally non-publishable. Values may only cross the
// in-process boundary into the one authorized subsequent Cauchy solve.
struct IntermediateOptimizationSeed {
  std::string role = "INTERMEDIATE_OPTIMIZATION_SEED";
  bool available = false;
  bool eligible_for_cauchy_initialization = false;
  bool eligible_for_trajectory_export = false;
  bool eligible_for_gt_evaluation = false;
  gtsam::Values values;
  IntermediateSeedQualityAudit quality;
};

struct HuberCauchyWarmStartResult {
  BaselineResult huber_intermediate;
  IntermediateOptimizationSeed intermediate_seed;
  bool cauchy_attempted = false;
  BaselineResult cauchy_final;
};

IntermediateSeedQualityAudit AuditIntermediateOptimizationSeed(
    const gtsam::NonlinearFactorGraph& huber_graph,
    const gtsam::Values& huber_input,
    const gtsam::Values& huber_terminal,
    const std::vector<FactorMeta>& base_uwb_metadata,
    const std::vector<double>& state_times_s,
    const IntermediateSeedQualityOptions& options);

IntermediateOptimizationSeed RetainIntermediateOptimizationSeed(
    const BaselineResult& huber_result,
    const gtsam::Values& huber_input,
    const std::vector<FactorMeta>& base_uwb_metadata,
    const std::vector<double>& state_times_s,
    const IntermediateSeedQualityOptions& options);

// Executes exactly Huber -> audited intermediate seed -> unchanged Cauchy.
// Only cauchy_final.valid represents a CERTIFIED_FINAL_ESTIMATE.
HuberCauchyWarmStartResult RunHuberToCauchyWarmStart(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& qualified_initial_values,
    const std::vector<FactorMeta>& base_uwb_metadata,
    const BaselineOptions& huber_options,
    const BaselineOptions& cauchy_options,
    const IntermediateSeedQualityOptions& seed_quality_options);

bool FixedRejectionKeeps(double standardized_absolute_residual,
                         double rejection_threshold_sigma);

// Runs all_range, robust_huber, robust_cauchy, or fixed_rejection_v1 from the
// common initial Values. Robust methods optimize their robust graph directly;
// all_range/fixed_rejection retain the raw-Gaussian reference semantics. It
// never consumes Stage 1 or Stage 2 state.
BaselineResult RunPaperBaseline(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& common_initial_values,
    const std::vector<FactorMeta>& base_uwb_metadata,
    const BaselineOptions& options,
    const RawGaussianReference* prepared = nullptr);

}  // namespace uifgo
