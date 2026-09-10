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

enum class PaperMethod {
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
  CheckedLmOptions lm;
};

struct BaselineResult {
  bool valid = false;
  std::string status = "ESTIMATION_FAILED";
  std::string reason;
  gtsam::NonlinearFactorGraph final_graph;
  gtsam::Values final_values;
  std::vector<FactorMeta> final_factor_metadata;
  std::vector<std::uint64_t> kept_uwb_obs_ids;
  std::vector<std::uint64_t> rejected_uwb_obs_ids;
  CheckedLmResult preliminary;
  CheckedLmResult final;
};

bool FixedRejectionKeeps(double standardized_absolute_residual,
                         double rejection_threshold_sigma);

// Runs all_range, robust_huber, robust_cauchy, or fixed_rejection_v1 from the
// common initial Values. It never consumes Stage 1 or Stage 2 state.
BaselineResult RunPaperBaseline(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& common_initial_values,
    const std::vector<FactorMeta>& base_uwb_metadata,
    const BaselineOptions& options);

}  // namespace uifgo
