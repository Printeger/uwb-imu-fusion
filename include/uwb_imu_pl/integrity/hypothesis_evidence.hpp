#pragma once

#include "uwb_imu_pl/integrity/joint_window_detector.hpp"

#include <cstdint>
#include <memory>

namespace uwb_imu_pl {

class CandidateWorkerPool;

// B2 (§5.7 / R2): capacity bounds for the compact mode form.  The compact
// descriptor stores a mode only on the factor groups it actually touches, so
// the padded window-wide matrix is never built for the hot products.  Every
// bound is explicit: when a window would exceed a bound the evaluator falls
// back to the padded form and counts the fallback instead of growing the
// allocation silently.  A hypothesis that exceeds the per-hypothesis
// dimension bound is refused fail-closed and counted.
struct CompactModeCapacity {
  std::size_t max_modes = 512;
  std::size_t max_window_columns = 4096;
  std::size_t max_mode_rows = 1u << 16;
  std::size_t max_total_compact_rows = 1u << 20;
  int max_hypothesis_dimension = 64;
};

struct HypothesisEvaluationConfig {
  double rank_tolerance = 1e-10;
  double max_condition_number = 1e10;
  double min_fault_gram_sigma = 1e-8;
  double max_fault_gram_condition = 1e10;
  double plausible_conditioned_statistic_margin = 0.0;
  bool enable_shared_context = true;
  bool enable_low_dim_batch = true;
  bool retain_detailed_results = true;
  std::size_t hypothesis_workers = 1;
  std::uint64_t fault_model_policy_fingerprint = 0;
  CompactModeCapacity compact_capacity{};
};

// PL-specific semantics are intentionally stored separately from the evidence
// monitorability result.  Both consume the same frozen Gram and LDLT solve,
// but retain their original, different rank/condition gates.
// B1: classify the detection-space response Z_h = Q2^T D_h of a hypothesis.
// `g_h` is the protected-state response C (H^T H)^-1 H^T D_h in the same basis.
// The classification is 1 = full rank, 2 = harmless nullspace (fault invisible
// but projected out of the protected state), 3 = dangerous nullspace, and
// 4 = numerically indistinguishable (smallest singular value inside the rank
// tolerance band).  Exposed so the tri-state logic is unit tested directly.
// B3: additionally reports the per-axis residual of the protected response
// outside the retained detection subspace (direction-level evidence for a
// dangerous nullspace) and, for a structurally harmless nullspace, the finite
// bound slopes computed by the projected path ||g V_r Sigma_r^-1|| (section
// 5.5) so the caller can use a bound instead of discarding the hypothesis.
int classifyDetectionResponse(const Eigen::MatrixXd& z_h,
                              const Eigen::MatrixXd& g_h,
                              double rank_tolerance,
                              double* smallest_singular_value,
                              double* condition, int* rank,
                              Eigen::Vector3d* axis_residual = nullptr,
                              Eigen::Vector3d* harmless_slopes = nullptr);

// Extended overload; the original eight-argument symbol above remains exported
// for ABI compatibility.
int classifyDetectionResponse(const Eigen::MatrixXd& z_h,
                              const Eigen::MatrixXd& g_h,
                              double rank_tolerance,
                              double* smallest_singular_value,
                              double* condition, int* rank,
                              Eigen::Vector3d* axis_residual,
                              Eigen::Vector3d* harmless_slopes,
                              double raw_factor_scale);

struct FrozenHypothesisPlEntry {
  HypothesisId hypothesis;
  MonitorabilityResult monitorability;
  Eigen::Vector3d protected_slopes = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  bool gram_spd = false;
  bool valid = false;
  // B1 audit: rank/condition of the detection-space response Z_h = Q2^T D_h
  // from the window's square-root context, and its structural classification
  // (0 = not evaluated, 1 = full rank, 2 = harmless nullspace, 3 = dangerous
  // nullspace, 4 = numerically indistinguishable).  The numeric Gram and the
  // slopes still come from the shared normal-equation solve, so this batch
  // changes no decision; the classification is the B2 input and is exported.
  int z_rank = 0;
  double z_smallest_singular_value = std::numeric_limits<double>::infinity();
  double z_condition = std::numeric_limits<double>::infinity();
  int z_classification = 0;
  // B3: the bound comes from the projected path ||g V_r Sigma_r^-1|| instead of
  // the fault Gram (structurally harmless detection nullspace).  Gamma stays
  // audit-only in that case, and the PL gate accepts the projected bound.
  bool bound_from_projected_path = false;
};

struct FrozenHypothesisPlProofV1 {
  std::uint64_t schema_version = 1;
  FrozenHypothesisPlEntry served_entry;
  Eigen::MatrixXd certified_gram;
  Eigen::MatrixXd protected_response;
  Eigen::VectorXd gram_eigenvalues;
  Eigen::MatrixXd gram_eigenvectors;
  Eigen::VectorXd gram_eigenvalue_errors;
  Eigen::Vector3d nullspace_axis_residual = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  int nullspace_class = 0;
  Eigen::MatrixXd raw_detection_factor;
  double raw_factor_scale = std::numeric_limits<double>::quiet_NaN();
  double rank_tolerance = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t parent_proof_identity = 0;
  std::uint64_t proof_identity = 0;
  std::uint64_t served_entry_identity = 0;
};

std::uint64_t frozenHypothesisPlEntryIdentity(
    const FrozenHypothesisPlEntry& entry);

bool validateFrozenHypothesisPlEntry(
    const FrozenHypothesisPlProofV1& proof,
    std::string* reason = nullptr);
bool frozenHypothesisPlProof(const FrozenHypothesisPlEntry& entry,
                             FrozenHypothesisPlProofV1* proof);

// Immutable, window-scoped products shared by evidence and KEEP_ALL PL.  The
// context is never reused after a changed observation set, rewhitening,
// marginalization, ordering/version change, or numerical-contract change.
struct FrozenHypothesisNumerics {
  WindowId window_id;
  LinearizationVersion version;
  std::uint64_t window_content_fingerprint = 0;
  std::uint64_t numerical_contract_fingerprint = 0;
  std::uint64_t hypothesis_fingerprint = 0;
  std::uint64_t fault_mode_fingerprint = 0;
  std::uint64_t fault_model_policy_fingerprint = 0;
  Eigen::Matrix3d protected_covariance = Eigen::Matrix3d::Zero();
  std::vector<FrozenHypothesisPlEntry> pl_entries;
  std::size_t mode_count = 0;
  std::size_t hypothesis_count = 0;
  std::size_t dimension_one_count = 0;
  std::size_t dimension_two_count = 0;
  std::size_t dimension_three_count = 0;
  std::size_t dimension_other_count = 0;
  std::size_t low_dimensional_count = 0;
  std::size_t generic_fallback_count = 0;
  std::size_t worker_blocks = 0;
  std::size_t bytes = 0;
  bool valid = false;
  std::string reason;
  // B2 storage discipline: footprint of the compact per-mode blocks (rows sum
  // over the touched factor groups, columns = fault parameter dimension) and
  // whether the configured capacity forced the padded fallback for this
  // window.
  std::size_t compact_rows = 0;
  std::size_t compact_columns = 0;
  bool compact_mode_used = false;
  bool compact_capacity_exceeded = false;
};

std::uint64_t hypothesisSetFingerprint(
    const std::vector<FaultHypothesisV2>& hypotheses);
std::uint64_t faultModeSetFingerprint(
    const std::vector<FaultModeBasis>& modes);

class HypothesisEvidenceEvaluator {
 public:
  explicit HypothesisEvidenceEvaluator(HypothesisEvaluationConfig config = {})
      : config_(config) {}
  std::vector<FaultModeEvidence> evaluateAll(
      const LinearizedIntegrityWindow& window,
      const std::vector<FaultModeBasis>& modes,
      std::vector<FaultHypothesisV2>* hypotheses,
      double squared_detector_threshold,
      std::shared_ptr<const FrozenHypothesisNumerics>* shared = nullptr,
      CandidateWorkerPool* worker_pool = nullptr) const;
  std::vector<FaultModeEvidence> evaluateAll(
      const LinearizedIntegrityWindow& window,
      std::vector<FaultHypothesisV2>* hypotheses,
      double squared_detector_threshold) const;

 private:
  HypothesisEvaluationConfig config_;
};

// The complete frozen plausible set used by action generation, winner
// selection, and coverage audit.  Risk retention alone never discharges an
// action's obligation to cover a plausible strict superset.
std::vector<HypothesisId> completePlausibleHypotheses(
    const std::vector<FaultHypothesisV2>& hypotheses,
    const std::vector<FaultModeEvidence>& evidence);

}  // namespace uwb_imu_pl
