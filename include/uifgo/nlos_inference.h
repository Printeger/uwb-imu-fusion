#pragma once

#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "uifgo/config.h"
#include "uifgo/nlos_refit.h"
#include "uifgo/nlos_scoring.h"
#include "uifgo/nlos_support.h"
#include "uifgo/paper_input.h"
#include "uifgo/types.h"

namespace uifgo {

constexpr const char* kT08DevelopmentGateLabel =
    "T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION";

struct GateThresholds {
  double tau_eta = 0.0;
  double tau_s_m = 0.0;
  double tau_gamma = 0.0;
  std::string parameter_provenance;
};

bool ValidDevelopmentGateThresholds(const GateThresholds& thresholds,
                                    std::string* reason = nullptr);

enum class GroupDecision { USE, SUPPRESS };
const char* GroupDecisionName(GroupDecision decision);

enum class FinalGatePolicy {
  LCB_PARTIAL,
  LCB_FIXED_FULL,
  SUPPRESS_ALL,
  STRUCTURED_DEBIAS,
  FULL_GATE,
  FIT_ONLY,
  S_FIT,
  ETA_ONLY
};
const char* FinalGatePolicyName(FinalGatePolicy policy);

struct GroupDecisionRecord {
  SegmentOverlapGroup group;
  GroupDecision decision = GroupDecision::SUPPRESS;
  std::string reason_code;
  bool eligible = false;
  bool eta_pass = false;
  bool s_pass = false;
  bool gamma_pass = false;
  double max_gamma = 0.0;
  bool max_gamma_available = false;
  std::string decision_linearization_id;
};

std::vector<GroupDecisionRecord> FreezeGroupDecisions(
    const std::vector<GroupRecoverabilityScore>& scores,
    const GateThresholds& thresholds,
    FinalGatePolicy policy = FinalGatePolicy::FULL_GATE);

struct FixedCompensation {
  size_t segment_ordinal = 0;
  std::string segment_id;
  int tag_id = 0;
  int anchor_id = 0;
  size_t candidate_observation_count = 0;
  double c_hat_stage2_m = 0.0;
  double sigma_c_local_m = 0.0;
  bool sigma_available = false;
  double delta_c_fixed_m = 0.0;
  bool use = false;
  std::string reason;
};
std::vector<FixedCompensation> FreezeFixedCompensations(
    const SegmentRefitResult& stage2,
    const std::vector<GroupRecoverabilityScore>& scores, bool full_variant);

struct FrozenObservationMask {
  std::uint64_t obs_id = 0;
  bool candidate = false;
  bool noncandidate_reference = false;
  bool decision_use = false;
  bool final_use = false;
  bool fallback_use = false;
  std::string segment_id;
  std::string group_id;
  std::string reason_code;
};

struct FinalFactorAuditRow {
  std::uint64_t obs_id = 0;
  std::string classification;
  size_t final_factor_count = 0;
  std::string expected_count;
  bool ok = false;
};

struct FinalFactorAudit {
  bool ok = false;
  std::string reason;
  size_t corrected_pseudo_range_count = 0;
  size_t accepted_candidate_count = 0;
  size_t suppressed_candidate_count = 0;
  size_t noncandidate_reference_count = 0;
  std::vector<FinalFactorAuditRow> observations;
};

enum class CovarianceStatus { AVAILABLE, UNAVAILABLE };
const char* CovarianceStatusName(CovarianceStatus status);

struct FinalCovarianceBlock {
  gtsam::Key key = 0;
  std::string role;
  std::string coordinates;
  Eigen::MatrixXd covariance;
};

struct FinalCovarianceResult {
  CovarianceStatus status = CovarianceStatus::UNAVAILABLE;
  std::string reason;
  std::string source = "FULL_FINAL_GRAPH_MARGINAL_COVARIANCE";
  std::string factorization = "GTSAM_CHOLESKY";
  std::vector<FinalCovarianceBlock> blocks;
};

FinalCovarianceResult ComputeFinalGraphCovariance(
    const gtsam::NonlinearFactorGraph& final_graph,
    const gtsam::Values& final_values);

struct FinalGroupScore {
  SegmentOverlapGroup group;
  bool evaluated = false;
  std::string status;
  std::string linearization_id;
  GroupRecoverabilityScore score;
};

// Identity inputs are content identities, not an execution/run directory ID.
// They bind the final inference to the input/config/plan/partition that
// produced it while keeping the engine source-neutral.
struct InferenceIdentityContext {
  std::string input_sha256;
  std::string config_sha256;
  std::string input_plan_sha256;
  std::string support_partition_sha256;
  std::string calibration_sha256;
  std::string solver_config_sha256;
  // Empty preserves the exact legacy v1 identity. Nonempty selects the R08
  // validation-bound v2 identity and is exported with the final graph.
  std::string validation_context_sha256;
};

struct InferenceContentIdentity {
  std::string schema = "t08_final_content_identity_v2";
  std::string graph_linearization_sha256;
  std::string values_sha256;
  std::string context_sha256;
};

InferenceContentIdentity ComputeInferenceContentIdentity(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const InferenceIdentityContext& context);

struct InferenceTiming {
  double decision_seconds = 0.0;
  double recovery_refit_seconds = 0.0;
  double final_score_seconds = 0.0;
  double fallback_seconds = 0.0;
  double covariance_seconds = 0.0;
  double total_seconds = 0.0;
};

struct FallbackAttempt {
  bool attempted = false;
  size_t attempt_count = 0;
  std::string trigger_reason;
  std::string recovery_failure_reason;
  std::string status = "NOT_TRIGGERED";
  std::string fallback_solver_status = "NOT_RUN";
  std::string fallback_failure_reason;
};

struct RefitAttemptDiagnostics {
  bool executed = false;
  std::string execution_status = "NOT_RUN";
  std::string solver_status = "NOT_RUN";
  std::string stop_reason = "NOT_RUN";
  std::string acceptance_audit_status = "NOT_RUN";
  std::string acceptance_failure_reason;
  std::vector<RefitIteration> iterations;
};

enum class InferenceStatus {
  OK,
  NO_CANDIDATES,
  NO_ELIGIBLE_CANDIDATES,
  ZERO_ACCEPTED,
  FALLBACK_OK,
  ESTIMATION_FAILED,
};

const char* InferenceStatusName(InferenceStatus status);

// The only valid Stage-4 result object.  No alternate optimized Values or
// graph is retained.  All final state/residual/covariance exporters must take
// this object and use final_graph/final_values together.
struct InferenceResult {
  InferenceStatus status = InferenceStatus::ESTIMATION_FAILED;
  std::string reason;
  std::string inference_id;
  InferenceIdentityContext identity_context;
  InferenceContentIdentity content_identity;
  GateThresholds gate_thresholds;
  // Empty preserves historical identity bytes.
  std::string requested_fixed_method;
  std::string actual_fixed_method;
  double fixed_kappa = 2.0;
  std::vector<FixedCompensation> fixed_compensations;
  gtsam::NonlinearFactorGraph final_graph;
  gtsam::Values final_values;
  std::vector<double> final_keyframe_times_s;
  std::vector<std::pair<std::string, double>> fixed_static_biases_m;
  std::vector<RefitFactorMeta> final_factor_metadata;
  std::vector<SegmentEstimate> final_segments;
  std::vector<RefitIteration> final_refit_iterations;
  std::vector<FrozenObservationMask> frozen_masks;
  std::vector<GroupDecisionRecord> decisions;
  std::vector<GroupRecoverabilityScore> decision_scores;
  // Scores at the failed/successful recovery attempt linearization are never
  // overwritten by fallback final-score not-applicable rows.
  std::vector<FinalGroupScore> recovery_final_scores;
  std::vector<FinalGroupScore> final_scores;
  RefitAttemptDiagnostics recovery_attempt;
  RefitAttemptDiagnostics fallback_refit_attempt;
  RefitAttemptDiagnostics final_result_refit;
  FinalFactorAudit factor_audit;
  FinalCovarianceResult final_graph_covariance;
  FallbackAttempt fallback;
  InferenceTiming timing;

  bool valid_estimate() const {
    return status != InferenceStatus::ESTIMATION_FAILED &&
           !final_graph.empty() && !final_values.empty() && factor_audit.ok;
  }
};

// Recomputes graph/Values/context fingerprints and the aggregate inference ID.
// Artifact export rejects a result whose content changed after sealing.
bool VerifyInferenceContentIdentity(const InferenceResult& result,
                                    std::string* reason = nullptr);

// Test-only deterministic fault injection.  Production callers use the
// default value and cannot activate these hooks through configuration.
struct InferenceTestHooks {
  bool force_recovery_failure = false;
  bool force_fallback_failure = false;
  bool force_covariance_unavailable = false;
};

class FinalInferenceEngine {
 public:
  FinalInferenceEngine(GateThresholds thresholds, RefitOptions refit_options,
                       RecoverabilityOptions score_options = {},
                       InferenceTestHooks test_hooks = {},
                       FinalGatePolicy policy = FinalGatePolicy::FULL_GATE,
                       const DevelopmentStage2Request*
                           development_refit_request = nullptr,
                       const DevelopmentStage2Request*
                           development_empty_recovery_request = nullptr,
                       const DevelopmentStage2Request*
                           development_empty_fallback_request = nullptr)
      : thresholds_(std::move(thresholds)),
        refit_options_(std::move(refit_options)),
        score_options_(std::move(score_options)),
        test_hooks_(std::move(test_hooks)),
        policy_(policy),
        development_refit_request_(development_refit_request),
        development_empty_recovery_request_(
            development_empty_recovery_request),
        development_empty_fallback_request_(
            development_empty_fallback_request) {}

  InferenceResult Run(
      const gtsam::NonlinearFactorGraph& frozen_raw_graph,
      const std::vector<FactorMeta>& frozen_raw_uwb_metadata,
      const SegmentRefitResult& stage2_refit,
      const SupportPartition& frozen_full_support,
      const std::vector<GroupRecoverabilityScore>& decision_scores,
      const PaperInputPlan& plan, const Config& cfg,
      const InferenceIdentityContext& identity_context) const;

 private:
  GateThresholds thresholds_;
  RefitOptions refit_options_;
  RecoverabilityOptions score_options_;
  InferenceTestHooks test_hooks_;
  FinalGatePolicy policy_ = FinalGatePolicy::FULL_GATE;
  const DevelopmentStage2Request* development_refit_request_ = nullptr;
  const DevelopmentStage2Request* development_empty_recovery_request_ =
      nullptr;
  const DevelopmentStage2Request* development_empty_fallback_request_ =
      nullptr;
};

}  // namespace uifgo
