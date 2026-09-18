#pragma once

#include "uwb_imu_pl/integrity/joint_window_detector.hpp"

#include <cstdint>
#include <memory>

namespace uwb_imu_pl {

class CandidateWorkerPool;

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
};

// PL-specific semantics are intentionally stored separately from the evidence
// monitorability result.  Both consume the same frozen Gram and LDLT solve,
// but retain their original, different rank/condition gates.
struct FrozenHypothesisPlEntry {
  HypothesisId hypothesis;
  MonitorabilityResult monitorability;
  Eigen::Vector3d protected_slopes = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  bool gram_spd = false;
  bool valid = false;
};

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
