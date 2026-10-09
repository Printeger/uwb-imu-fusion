#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"

#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "numerical_phase_profile.hpp"
#include "successful_validation_memo.hpp"
#include "scoped_frozen_validation.hpp"
#include "../estimation/classification_numerics.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace uwb_imu_pl {
namespace {

constexpr std::uint32_t kArenaFrozenHypothesisProof = 1;
constexpr std::uint32_t kArenaFrozenHypothesisValidationMemo = 9;
// Net-cost experiment did not justify the new per-hypothesis wrapper/index.
// Keep it opt-in; exhaustive always overrides even when reuse is requested.
bool scopedFrozenValidationReuseEnabled() {
  const char* reuse = std::getenv("UWB_IMU_PL_FROZEN_VALIDATION_REUSE");
  return reuse && std::string(reuse) == "1" &&
      !std::getenv("UWB_IMU_PL_EXHAUSTIVE_FROZEN_VALIDATION");
}
struct ImmutableFrozenHypothesisPayload {
  ImmutableFrozenHypothesisPayload(FrozenHypothesisPlProofV1&& value,
                                  std::uint64_t generation)
      : proof(std::move(value)), arena_generation(generation) {}
  const FrozenHypothesisPlProofV1 proof;
  const std::uint64_t arena_generation;
  detail::SuccessfulValidationMemo validation;
};

void hashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= bytes[i];
    *hash *= 1099511628211ULL;
  }
}

template <class Derived>
void hashMatrix(std::uint64_t* hash, const Eigen::MatrixBase<Derived>& matrix) {
  const Eigen::Index rows = matrix.rows(), columns = matrix.cols();
  hashBytes(hash, &rows, sizeof(rows));
  hashBytes(hash, &columns, sizeof(columns));
  for (Eigen::Index column = 0; column < columns; ++column)
    for (Eigen::Index row = 0; row < rows; ++row) {
      const double value = matrix(row, column);
      hashBytes(hash, &value, sizeof(value));
    }
}

std::uint64_t frozenEntryProofKey(const FrozenHypothesisPlEntry& entry) {
  std::uint64_t key = 1469598103934665603ULL;
  const auto hypothesis = entry.hypothesis.value();
  hashBytes(&key, &hypothesis, sizeof(hypothesis));
  hashBytes(&key, &entry.monitorability.parameter_dimension,
            sizeof(entry.monitorability.parameter_dimension));
  hashBytes(&key, &entry.monitorability.physical_parameter_dimension,
            sizeof(entry.monitorability.physical_parameter_dimension));
  hashBytes(&key, &entry.monitorability.rank,
            sizeof(entry.monitorability.rank));
  hashBytes(&key, &entry.monitorability.sigma_min,
            sizeof(entry.monitorability.sigma_min));
  hashBytes(&key, &entry.monitorability.sigma_max,
            sizeof(entry.monitorability.sigma_max));
  hashBytes(&key, &entry.monitorability.condition_number,
            sizeof(entry.monitorability.condition_number));
  hashBytes(&key, &entry.monitorability.monitorable,
            sizeof(entry.monitorability.monitorable));
  hashMatrix(&key, entry.monitorability.protected_slopes);
  hashBytes(&key, entry.monitorability.reason.data(),
            entry.monitorability.reason.size());
  hashMatrix(&key, entry.protected_slopes);
  hashBytes(&key, &entry.gram_spd, sizeof(entry.gram_spd));
  hashBytes(&key, &entry.valid, sizeof(entry.valid));
  hashBytes(&key, &entry.z_rank, sizeof(entry.z_rank));
  hashBytes(&key, &entry.z_smallest_singular_value,
            sizeof(entry.z_smallest_singular_value));
  hashBytes(&key, &entry.z_condition, sizeof(entry.z_condition));
  hashBytes(&key, &entry.z_classification, sizeof(entry.z_classification));
  hashBytes(&key, &entry.bound_from_projected_path,
            sizeof(entry.bound_from_projected_path));
  return key;
}

std::uint64_t frozenDualProofDigest(
    const FrozenHypothesisNumerics& base,
    const FrozenHypothesisDualNumerics& frozen) {
  std::uint64_t hash = 1469598103934665603ULL;
  const std::uint64_t owner_token = frozen.window_owner
      ? frozen.window_owner->owner_token : 0;
  const std::uintptr_t payload_address =
      reinterpret_cast<std::uintptr_t>(frozen.window_payload);
  hashBytes(&hash, &owner_token, sizeof(owner_token));
  hashBytes(&hash, &payload_address, sizeof(payload_address));
  const auto window_id = base.window_id.value();
  hashBytes(&hash, &window_id, sizeof(window_id));
  hashBytes(&hash, &base.version, sizeof(base.version));
  hashBytes(&hash, &base.window_content_fingerprint,
            sizeof(base.window_content_fingerprint));
  hashBytes(&hash, &base.numerical_contract_fingerprint,
            sizeof(base.numerical_contract_fingerprint));
  hashBytes(&hash, &base.hypothesis_fingerprint,
            sizeof(base.hypothesis_fingerprint));
  hashBytes(&hash, &frozen.candidate_hypothesis_fingerprint,
            sizeof(frozen.candidate_hypothesis_fingerprint));
  hashBytes(&hash, &base.fault_mode_fingerprint,
            sizeof(base.fault_mode_fingerprint));
  hashBytes(&hash, &base.fault_model_policy_fingerprint,
            sizeof(base.fault_model_policy_fingerprint));
  hashMatrix(&hash, base.protected_covariance);
  hashMatrix(&hash, frozen.candidate_detection_modes);
  hashMatrix(&hash, frozen.candidate_protected_modes);
  hashMatrix(&hash, frozen.candidate_protected_factor);
  hashMatrix(&hash, frozen.candidate_protected_covariance);
  const std::size_t activity_size = frozen.candidate_mode_active.size();
  hashBytes(&hash, &activity_size, sizeof(activity_size));
  if (!frozen.candidate_mode_active.empty()) {
    hashBytes(&hash, frozen.candidate_mode_active.data(),
              frozen.candidate_mode_active.size());
  }
  const std::size_t entry_count = base.pl_entries.size();
  hashBytes(&hash, &entry_count, sizeof(entry_count));
  for (const auto& entry : base.pl_entries) {
    const std::uint64_t identity = frozenEntryProofKey(entry);
    hashBytes(&hash, &identity, sizeof(identity));
  }
  const std::size_t block_count = frozen.dual_blocks.size();
  hashBytes(&hash, &block_count, sizeof(block_count));
  for (const auto& block : frozen.dual_blocks) {
    const auto hypothesis = block.hypothesis.value();
    hashBytes(&hash, &hypothesis, sizeof(hypothesis));
    hashBytes(&hash, &block.dimension, sizeof(block.dimension));
    hashMatrix(&hash, block.total_gram);
    hashMatrix(&hash, block.history_gram);
    hashMatrix(&hash, block.score);
    hashMatrix(&hash, block.protected_response);
    hashBytes(&hash, &block.profile_statistic,
              sizeof(block.profile_statistic));
    hashBytes(&hash, &block.valid, sizeof(block.valid));
  }
  hashBytes(&hash, &base.mode_count, sizeof(base.mode_count));
  hashBytes(&hash, &base.hypothesis_count, sizeof(base.hypothesis_count));
  hashBytes(&hash, &base.dimension_one_count, sizeof(base.dimension_one_count));
  hashBytes(&hash, &base.dimension_two_count, sizeof(base.dimension_two_count));
  hashBytes(&hash, &base.dimension_three_count,
            sizeof(base.dimension_three_count));
  hashBytes(&hash, &base.dimension_other_count,
            sizeof(base.dimension_other_count));
  hashBytes(&hash, &base.low_dimensional_count,
            sizeof(base.low_dimensional_count));
  hashBytes(&hash, &base.generic_fallback_count,
            sizeof(base.generic_fallback_count));
  return hash;
}

struct FrozenProofRegistry {
  std::mutex mutex;
  std::map<std::uint64_t, FrozenHypothesisPlProofV1> proofs;
};

FrozenProofRegistry& frozenProofRegistry() {
  static FrozenProofRegistry registry;
  return registry;
}

struct WorkDelta {
  std::uint64_t eigen = 0;
  std::uint64_t svd = 0;
  std::uint64_t ldlt = 0;
  std::uint64_t low_dim = 0;
  std::uint64_t generic = 0;
};

void publish(const WorkDelta& work) {
  NumericalWorkCounters::faultGramEigen(work.eigen);
  NumericalWorkCounters::faultGramSvd(work.svd);
  NumericalWorkCounters::faultGramLdlt(work.ldlt);
  NumericalWorkCounters::lowDimFaultGram(work.low_dim);
  NumericalWorkCounters::genericFaultGramFallback(work.generic);
}

void failDimension(FaultHypothesisV2* hypothesis, FaultModeEvidence* evidence,
                   int physical_dimension) {
  auto& monitor = hypothesis->monitorability;
  monitor.parameter_dimension = 0;
  monitor.physical_parameter_dimension = physical_dimension;
  monitor.reason = "fault map dimension/non-finite gate failed";
  hypothesis->monitored = false;
  evidence->monitorability = monitor;
}

void fillEvidenceMonitor(const SymmetricPsdCertificate& certificate,
                         int dimension, int physical_dimension,
                         const HypothesisEvaluationConfig& config,
                         FaultHypothesisV2* hypothesis,
                         FaultModeEvidence* evidence) {
  auto& monitor = hypothesis->monitorability;
  monitor.parameter_dimension = dimension;
  monitor.physical_parameter_dimension = physical_dimension;
  monitor.rank = certificate.rank;
  monitor.sigma_min = certificate.sigma_min;
  monitor.sigma_max = certificate.sigma_max;
  monitor.condition_number = certificate.condition;
  monitor.monitorable = certificate.valid && monitor.rank == dimension &&
      monitor.sigma_min >= config.min_fault_gram_sigma &&
      monitor.condition_number <= config.max_fault_gram_condition;
  monitor.reason = monitor.monitorable ? "" : "fault subspace is unmonitorable";
  hypothesis->monitored = monitor.monitorable;
  evidence->monitorability = monitor;
}

void fillPlMonitor(const SymmetricPsdCertificate& certificate, int dimension,
                   FrozenHypothesisPlEntry* entry) {
  auto& monitor = entry->monitorability;
  monitor.parameter_dimension = dimension;
  monitor.rank = certificate.rank;
  monitor.sigma_min = certificate.sigma_min;
  monitor.sigma_max = certificate.sigma_max;
  monitor.condition_number = certificate.condition;
  monitor.monitorable = certificate.valid && monitor.rank == dimension;
  if (!monitor.monitorable) {
    monitor.reason = "remaining post-FDE fault is unmonitorable";
  }
}

void storePlCertificate(const SymmetricPsdCertificate& gram,
                        const Eigen::MatrixXd& protected_response,
                        const GramResponseCertificate& response,
                        const Eigen::MatrixXd* raw_detection_factor,
                        double raw_factor_scale,
                        double rank_tolerance,
                        std::uint64_t parent_proof_identity,
                        FrozenHypothesisPlEntry* entry,
                        AttemptProofArena* proof_arena) {
  if (!entry) return;
  FrozenHypothesisPlProofV1 proof;
  proof.served_entry = *entry;
  proof.certified_gram = gram.symmetric_matrix;
  proof.protected_response = protected_response;
  proof.gram_eigenvalues = gram.eigenvalues;
  proof.gram_eigenvectors = gram.eigenvectors;
  proof.gram_eigenvalue_errors = gram.eigenvalue_errors;
  proof.nullspace_axis_residual = response.axis_residual;
  proof.nullspace_class = static_cast<int>(response.nullspace_class);
  proof.proof_identity = response.proof_identity;
  if (raw_detection_factor) proof.raw_detection_factor = *raw_detection_factor;
  proof.raw_factor_scale = raw_factor_scale;
  proof.rank_tolerance = rank_tolerance;
  proof.parent_proof_identity = parent_proof_identity;
  proof.served_entry_identity = frozenEntryProofKey(proof.served_entry);
  const auto key = frozenEntryProofKey(*entry);
  if (proof_arena) {
    if (!scopedFrozenValidationReuseEnabled()) {
      // Reference restores original producer cost, not just full reader work.
      (void)detail::AttemptProofArenaAccess::store(
          proof_arena, kArenaFrozenHypothesisProof, key, nullptr,
          std::make_shared<const FrozenHypothesisPlProofV1>(std::move(proof)));
    } else {
      const auto immutable = std::make_shared<const ImmutableFrozenHypothesisPayload>(
          std::move(proof), proof_arena->generation());
      const std::shared_ptr<const FrozenHypothesisPlProofV1> payload(
          immutable, &immutable->proof);
      (void)detail::AttemptProofArenaAccess::store(
          proof_arena, kArenaFrozenHypothesisProof, key, nullptr, payload);
      (void)detail::AttemptProofArenaAccess::store(
          proof_arena, kArenaFrozenHypothesisValidationMemo, key, nullptr, immutable);
    }
  } else {
    auto& registry = frozenProofRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    registry.proofs[key] = std::move(proof);
  }
}

void analyzeDynamic(const Eigen::MatrixXd& input_gram,
                    const Eigen::MatrixXd* raw_detection_factor,
                    const Eigen::VectorXd& score,
                    const Eigen::MatrixXd* protected_fault,
                    bool allow_harmless_nullspace,
                    std::uint64_t parent_proof_identity,
                    double raw_factor_scale,
                    int physical_dimension, double all_in,
                    double squared_detector_threshold,
                    const HypothesisEvaluationConfig& config,
                    FaultHypothesisV2* hypothesis,
                    FaultModeEvidence* evidence,
                    FrozenHypothesisPlEntry* pl_entry,
                    WorkDelta* work,
                    AttemptProofArena* proof_arena) {
  ++work->generic;
  const int dimension = input_gram.rows();
  if (dimension <= 0 || input_gram.cols() != dimension ||
      score.size() != dimension || !input_gram.allFinite() ||
      !score.allFinite()) {
    failDimension(hypothesis, evidence, physical_dimension);
    return;
  }
  const bool has_raw_factor = raw_detection_factor &&
      raw_detection_factor->cols() == dimension &&
      raw_detection_factor->rows() > 0 && raw_detection_factor->allFinite();
  const bool has_protected_response = protected_fault &&
      protected_fault->rows() == 3 &&
      protected_fault->cols() == dimension && protected_fault->allFinite();
  // The trusted combined constructor already certifies exactly this raw Gram.
  // Reuse its complete certificate within this call only; never accept an
  // externally supplied certificate or change the non-factor reference path.
  const bool reuse_gram = has_raw_factor && has_protected_response &&
      !std::getenv("UWB_IMU_PL_EXHAUSTIVE_GRAM_CERTIFICATES");
  GramResponseCertificate response_certificate;
  if (reuse_gram) {
    response_certificate = certifyFactorGramAndProtectedResponse(
        *raw_detection_factor, input_gram, *protected_fault,
        config.rank_tolerance, parent_proof_identity, raw_factor_scale);
  }
  const SymmetricPsdCertificate gram_certificate = reuse_gram
      ? response_certificate.gram : has_raw_factor
      ? certifyFactorGram(*raw_detection_factor, input_gram,
                          config.rank_tolerance, parent_proof_identity,
                          raw_factor_scale)
      : certifySymmetricPsd(input_gram, config.rank_tolerance,
                            parent_proof_identity, raw_factor_scale);
  if (!gram_certificate.valid) {
    hypothesis->monitorability.reason =
        "fault Gram numerical certificate failed: " + gram_certificate.reason;
    hypothesis->monitored = false;
    evidence->monitorability = hypothesis->monitorability;
    if (pl_entry) pl_entry->monitorability = hypothesis->monitorability;
    return;
  }
  const Eigen::MatrixXd& gram = gram_certificate.symmetric_matrix;
  if (config.retain_detailed_results) evidence->fault_gram = gram;
  ++work->eigen;
  fillEvidenceMonitor(gram_certificate, dimension, physical_dimension,
                      config, hypothesis, evidence);
  if (has_protected_response) {
    if (!reuse_gram) response_certificate = has_raw_factor
        ? certifyFactorGramAndProtectedResponse(
              *raw_detection_factor, input_gram, *protected_fault,
              config.rank_tolerance, parent_proof_identity, raw_factor_scale)
        : certifyGramAndProtectedResponse(
              gram, *protected_fault, config.rank_tolerance,
              parent_proof_identity, raw_factor_scale);
    if (!hypothesis->monitored && allow_harmless_nullspace &&
        response_certificate.valid &&
        response_certificate.nullspace_class ==
            GramNullspaceClass::Harmless) {
      hypothesis->monitorability.monitorable = true;
      hypothesis->monitorability.reason =
          "harmless Gram nullspace certified by protected response";
      hypothesis->monitorability.protected_slopes =
          response_certificate.protected_slopes;
      hypothesis->monitored = true;
      evidence->monitorability = hypothesis->monitorability;
    }
  }
  if (pl_entry) {
    fillPlMonitor(gram_certificate, dimension, pl_entry);
    pl_entry->gram_spd = gram_certificate.psd;
    if (has_protected_response) {
      pl_entry->monitorability.monitorable = response_certificate.valid &&
          response_certificate.nullspace_class !=
              GramNullspaceClass::Dangerous;
      pl_entry->monitorability.reason =
          pl_entry->monitorability.monitorable ? "" : response_certificate.reason;
      if (pl_entry->monitorability.monitorable) {
        pl_entry->protected_slopes = response_certificate.protected_slopes;
        pl_entry->monitorability.protected_slopes =
            pl_entry->protected_slopes;
        pl_entry->valid = pl_entry->protected_slopes.allFinite();
        evidence->monitorability.protected_slopes =
            pl_entry->protected_slopes;
      }
      storePlCertificate(gram_certificate, *protected_fault,
                         response_certificate, raw_detection_factor,
                         raw_factor_scale, config.rank_tolerance,
                         parent_proof_identity, pl_entry, proof_arena);
    } else {
      pl_entry->monitorability.monitorable = false;
      pl_entry->monitorability.reason =
          "protected fault response is missing or invalid";
    }
  }
  if (!hypothesis->monitored) return;
  if (hypothesis->monitored) {
    Eigen::VectorXd inverse = Eigen::VectorXd::Zero(dimension);
    Eigen::MatrixXd positive_basis(dimension, gram_certificate.rank);
    int positive_column = 0;
    for (int index = 0; index < dimension; ++index) {
      if (gram_certificate.eigenvalues(index) >
          gram_certificate.rank_eigenvalue_gate) {
        inverse(index) = 1.0 / gram_certificate.eigenvalues(index);
        positive_basis.col(positive_column++) =
            gram_certificate.eigenvectors.col(index);
      }
    }
    Eigen::VectorXd projected_score = Eigen::VectorXd::Zero(dimension);
    if (positive_basis.cols() > 0) {
      projected_score = positive_basis *
          (positive_basis.transpose() * score);
    }
    const double score_tolerance =
        128.0 * std::numeric_limits<double>::epsilon() *
        static_cast<double>(std::max(1, dimension)) *
        score.norm();
    if ((score - projected_score).norm() > score_tolerance) {
      hypothesis->monitorability.monitorable = false;
      hypothesis->monitorability.reason =
          "fault score has an uncertified Gram-nullspace component";
      hypothesis->monitored = false;
      evidence->monitorability = hypothesis->monitorability;
      return;
    }
    const Eigen::VectorXd estimated = gram_certificate.eigenvectors *
        inverse.asDiagonal() * gram_certificate.eigenvectors.transpose() *
        score;
    if (!estimated.allFinite()) {
      hypothesis->monitorability.monitorable = false;
      hypothesis->monitorability.reason =
          "fault profile solve failed its common Gram certificate";
      hypothesis->monitored = false;
      evidence->monitorability = hypothesis->monitorability;
      return;
    }
    if (config.retain_detailed_results) evidence->estimated_fault = estimated;
    evidence->explained_energy = std::max(0.0, score.dot(estimated));
    evidence->conditioned_statistic = std::max(
        0.0, all_in - evidence->explained_energy);
    // C3 wiring (§8.4): the raw whitened profile value J = ||r_c||^2 + kappa_b
    // - t' Gamma^+ t; no risk-adjusted channel quantity is used here.
    evidence->profile_j = evidence->conditioned_statistic;
    evidence->profile_valid = std::isfinite(evidence->profile_j);
    evidence->log_evidence = 0.5 * evidence->explained_energy;
    evidence->plausible = evidence->conditioned_statistic <=
        squared_detector_threshold +
            config.plausible_conditioned_statistic_margin;
  }
}

template <int Dimension>
void analyzeFixed(const Eigen::Matrix<double, Dimension, Dimension>& input_gram,
                  const Eigen::MatrixXd* raw_detection_factor,
                  const Eigen::Matrix<double, Dimension, 1>& score,
                  const Eigen::Matrix<double, 3, Dimension>& protected_fault,
                  std::uint64_t parent_proof_identity,
                  double raw_factor_scale,
                  int physical_dimension, double all_in,
                  double squared_detector_threshold,
                  const HypothesisEvaluationConfig& config,
                  FaultHypothesisV2* hypothesis,
                  FaultModeEvidence* evidence,
                  FrozenHypothesisPlEntry* pl_entry,
                  WorkDelta* work,
                  AttemptProofArena* proof_arena) {
  ++work->low_dim;
  const auto gram = (0.5 * (input_gram + input_gram.transpose())).eval();
  if (!gram.allFinite() || !score.allFinite() || !protected_fault.allFinite()) {
    failDimension(hypothesis, evidence, physical_dimension);
    return;
  }
  if (config.retain_detailed_results) evidence->fault_gram = gram;
  ++work->eigen;
  const bool has_raw_factor = raw_detection_factor &&
      raw_detection_factor->cols() == Dimension &&
      raw_detection_factor->rows() > 0 && raw_detection_factor->allFinite();
  const bool reuse_gram = has_raw_factor && pl_entry &&
      !std::getenv("UWB_IMU_PL_EXHAUSTIVE_GRAM_CERTIFICATES");
  GramResponseCertificate response_certificate;
  if (reuse_gram) {
    response_certificate = certifyFactorGramAndProtectedResponse(
        *raw_detection_factor, Eigen::MatrixXd(input_gram),
        Eigen::MatrixXd(protected_fault), config.rank_tolerance,
        parent_proof_identity, raw_factor_scale);
  }
  const SymmetricPsdCertificate gram_certificate = reuse_gram
      ? response_certificate.gram : has_raw_factor
      ? certifyFactorGram(*raw_detection_factor, Eigen::MatrixXd(input_gram),
                          config.rank_tolerance, parent_proof_identity,
                          raw_factor_scale)
      : certifySymmetricPsd(Eigen::MatrixXd(input_gram), config.rank_tolerance,
                            parent_proof_identity, raw_factor_scale);
  if (!gram_certificate.valid) {
    hypothesis->monitorability.reason =
        "fault Gram numerical certificate failed: " + gram_certificate.reason;
    hypothesis->monitored = false;
    evidence->monitorability = hypothesis->monitorability;
    return;
  }
  fillEvidenceMonitor(gram_certificate, Dimension, physical_dimension,
                      config, hypothesis, evidence);
  if (pl_entry) {
    fillPlMonitor(gram_certificate, Dimension, pl_entry);
    pl_entry->gram_spd = gram_certificate.psd;
    if (!reuse_gram) response_certificate = has_raw_factor
        ? certifyFactorGramAndProtectedResponse(
              *raw_detection_factor, Eigen::MatrixXd(input_gram),
              Eigen::MatrixXd(protected_fault), config.rank_tolerance,
              parent_proof_identity, raw_factor_scale)
        : certifyGramAndProtectedResponse(
              Eigen::MatrixXd(gram), Eigen::MatrixXd(protected_fault),
              config.rank_tolerance, parent_proof_identity, raw_factor_scale);
    pl_entry->monitorability.monitorable = response_certificate.valid &&
        response_certificate.nullspace_class != GramNullspaceClass::Dangerous;
    pl_entry->monitorability.reason =
        pl_entry->monitorability.monitorable ? "" : response_certificate.reason;
    if (pl_entry->monitorability.monitorable) {
      pl_entry->protected_slopes = response_certificate.protected_slopes;
      pl_entry->monitorability.protected_slopes =
          pl_entry->protected_slopes;
      pl_entry->valid = pl_entry->protected_slopes.allFinite();
      evidence->monitorability.protected_slopes = pl_entry->protected_slopes;
    }
    storePlCertificate(gram_certificate, Eigen::MatrixXd(protected_fault),
                       response_certificate, raw_detection_factor,
                       raw_factor_scale, config.rank_tolerance,
                       parent_proof_identity, pl_entry, proof_arena);
  }
  if (!hypothesis->monitored) return;
  if (hypothesis->monitored) {
    const Eigen::VectorXd inverse =
        gram_certificate.eigenvalues.cwiseInverse();
    const Eigen::VectorXd estimated_dynamic = gram_certificate.eigenvectors *
        inverse.asDiagonal() * gram_certificate.eigenvectors.transpose() *
        Eigen::VectorXd(score);
    if (!estimated_dynamic.allFinite()) {
      hypothesis->monitorability.monitorable = false;
      hypothesis->monitorability.reason =
          "fault profile solve failed its common Gram certificate";
      hypothesis->monitored = false;
      evidence->monitorability = hypothesis->monitorability;
      return;
    }
    const Eigen::Matrix<double, Dimension, 1> estimated = estimated_dynamic;
    if (config.retain_detailed_results) evidence->estimated_fault = estimated;
    evidence->explained_energy = std::max(0.0, score.dot(estimated));
    evidence->conditioned_statistic = std::max(
        0.0, all_in - evidence->explained_energy);
    // C3 wiring (§8.4): raw whitened profile value, see analyzeDynamic.
    evidence->profile_j = evidence->conditioned_statistic;
    evidence->profile_valid = std::isfinite(evidence->profile_j);
    evidence->log_evidence = 0.5 * evidence->explained_energy;
    evidence->plausible = evidence->conditioned_statistic <=
        squared_detector_threshold +
            config.plausible_conditioned_statistic_margin;
  }
}

struct ModeDescriptor {
  FaultModeId id;
  SensorType sensor = SensorType::Unknown;
  // C3 wiring (§8.4): comparability identity of the mode's parameter space.
  FaultUnitKind unit_kind = FaultUnitKind::Unknown;
  int physical_dimension = 0;
  int dimension = 0;
  Eigen::Index offset = 0;
  // B2: index of the source mode, so compact descriptors can be reached from
  // the descriptor without another lookup.
  std::size_t mode_number = 0;
};

// B2 (§5.7/R2): a mode only writes on the factor groups it actually touches.
// The compact descriptor keeps those rows contiguously (no window-wide zero
// padding) together with the products that only depend on the mode itself
// (H^T A and A^T parity).  Cross terms of a hypothesis are then assembled from
// the intersections of these row spans.
struct CompactModeSpan {
  Eigen::Index row_offset = 0;   // row offset of the factor group in the window
  Eigen::Index rows = 0;         // rows of the group
  Eigen::Index block_offset = 0; // row offset inside the compact block
  std::size_t piece = 0;         // index into the whitened pieces of the mode
};

struct CompactModeEntry {
  std::vector<CompactModeSpan> spans;  // ascending row_offset
  Eigen::MatrixXd block;               // Sum(rows) x dimension, whitened
  Eigen::MatrixXd normal_cross;        // H^T A (window columns x dimension)
  Eigen::VectorXd score;               // A^T parity (dimension)
  bool valid = false;
};

bool useEffectiveBasis(const FaultModeBasis& mode) {
  return mode.effective_basis_certified &&
      mode.effective_parameter_dimension > 0 &&
      mode.effective_parameter_basis.rows() == mode.parameter_dimension &&
      mode.effective_parameter_basis.cols() ==
          mode.effective_parameter_dimension;
}

// C3 wiring (§8.4): the physical unit of a mode, from its declared fault kind
// and sensor.  Anything the catalogue does not declare stays Unknown, which is
// never comparable with a declared unit.
FaultUnitKind faultUnitKindOf(FaultKind kind, SensorType sensor) {
  switch (kind) {
    case FaultKind::AnchorBiasEpochIndependent:
    case FaultKind::AnchorBiasPersistentConstant:
    case FaultKind::AnchorBiasRamp:
      return FaultUnitKind::UwbRangeMeters;
    case FaultKind::AccelAxisIntervalConstant:
      return FaultUnitKind::ImuAccelMps2;
    case FaultKind::GyroAxisIntervalConstant:
      return FaultUnitKind::ImuGyroRadps;
    default:
      break;
  }
  switch (sensor) {
    case SensorType::Uwb:
      return FaultUnitKind::UwbRangeMeters;
    case SensorType::ImuAccelerometer:
      return FaultUnitKind::ImuAccelMps2;
    case SensorType::ImuGyroscope:
      return FaultUnitKind::ImuGyroRadps;
    default:
      break;
  }
  return FaultUnitKind::Unknown;
}

std::uint64_t admittedProofIdentity(
    const LinearizedIntegrityWindow& window,
    std::uint64_t frozen_identity) {
  if (frozen_identity != 0) {
    NumericalWorkCounters::frozenIdentityReuse();
    return frozen_identity;
  }
  NumericalWorkCounters::frozenIdentityBuild();
  return frozenWindowNumericalProofIdentity(window, *window.numerics);
}

std::vector<FaultModeEvidence> evaluateContiguous(
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    const HypothesisEvaluationConfig& config,
    std::uint64_t numerical_proof_identity,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared,
    std::shared_ptr<const FrozenHypothesisDualNumerics>* shared_dual,
    CandidateWorkerPool* worker_pool,
    const FrozenWindowAdmission* admission,
    AttemptProofArena* proof_arena) {
  std::vector<FaultModeEvidence> results(hypotheses->size());
  std::map<std::uint64_t, std::size_t> block_index;
  std::vector<Eigen::Index> block_offsets(window.blocks.size());
  Eigen::Index row_offset = 0;
  for (std::size_t index = 0; index < window.blocks.size(); ++index) {
    block_index.emplace(window.blocks[index].group_id.value(), index);
    block_offsets[index] = row_offset;
    row_offset += window.blocks[index].residual_whitened.size();
  }
  std::vector<ModeDescriptor> descriptors;
  descriptors.reserve(modes.size());
  std::unordered_map<std::uint64_t, std::size_t> mode_index;
  Eigen::Index total_columns = 0;
  for (const auto& mode : modes) {
    ModeDescriptor descriptor;
    descriptor.id = mode.id;
    descriptor.sensor = mode.sensor;
    descriptor.unit_kind = faultUnitKindOf(mode.kind, mode.sensor);
    descriptor.physical_dimension = mode.parameter_dimension;
    descriptor.dimension = useEffectiveBasis(mode)
        ? mode.effective_parameter_dimension : mode.parameter_dimension;
    descriptor.offset = total_columns;
    total_columns += descriptor.dimension;
    descriptor.mode_number = descriptors.size();
    mode_index.emplace(mode.id.value(), descriptors.size());
    descriptors.push_back(descriptor);
  }
  // Resolve external mode IDs once.  Every later stage carries stable
  // descriptor indices, so pair collection, numerical assembly and summary
  // accounting cannot drift into repeated map/string lookup work.
  struct HypothesisDescriptor {
    std::vector<std::size_t> parts;
    bool all_ids_known = false;
    int summary_dimension = 0;
  };
  std::vector<HypothesisDescriptor> hypothesis_descriptors(
      hypotheses->size());
  for (std::size_t hypothesis_index = 0;
       hypothesis_index < hypotheses->size(); ++hypothesis_index) {
    const auto& hypothesis = (*hypotheses)[hypothesis_index];
    auto& resolved = hypothesis_descriptors[hypothesis_index];
    resolved.all_ids_known = !hypothesis.modes.empty();
    resolved.parts.reserve(hypothesis.modes.size());
    bool evaluation_prefix_valid = true;
    for (const auto id : hypothesis.modes) {
      NumericalWorkCounters::descriptorIdLookup();
      const auto found = mode_index.find(id.value());
      if (found == mode_index.end()) {
        resolved.all_ids_known = false;
        evaluation_prefix_valid = false;
        continue;
      }
      resolved.summary_dimension += descriptors[found->second].dimension;
      if (evaluation_prefix_valid) resolved.parts.push_back(found->second);
    }
  }
  // ---- B2 (§5.7/R2): compact mode descriptors ---------------------------
  // A mode only writes on the factor groups it touches; the compact entry
  // stores exactly those rows plus the mode-local products (H^T A and
  // A^T parity).  The window-wide zero-padded matrix is materialized only when
  // the audit needs it as the input of the single frozen Q^T application, or
  // when the configured capacity forces the padded fallback.
  const auto& capacity = config.compact_capacity;
  const bool compact_within_capacity =
      modes.size() <= capacity.max_modes &&
      static_cast<std::size_t>(total_columns) <= capacity.max_window_columns;
  std::vector<CompactModeEntry> compact(modes.size());
  std::vector<bool> mode_valid(modes.size(), true);
  std::size_t total_compact_rows = 0;
  bool compact_usable = compact_within_capacity;
  for (std::size_t mode_number = 0;
       compact_usable && mode_number < modes.size(); ++mode_number) {
    const auto& mode = modes[mode_number];
    const auto& descriptor = descriptors[mode_number];
    auto& entry = compact[mode_number];
    mode_valid[mode_number] = descriptor.dimension > 0;
    if (!mode_valid[mode_number]) continue;
    std::vector<Eigen::MatrixXd> pieces;
    pieces.reserve(mode.raw_group_maps.size());
    for (const auto& item : mode.raw_group_maps) {
      const auto found = block_index.find(item.first.value());
      if (found == block_index.end()) {
        mode_valid[mode_number] = false;
        break;
      }
      const auto& block = window.blocks[found->second];
      if (item.second.rows() != block.residual_raw.size() ||
          item.second.cols() != mode.parameter_dimension) {
        mode_valid[mode_number] = false;
        break;
      }
      const Eigen::MatrixXd whitened = block.whitener * item.second;
      pieces.push_back(useEffectiveBasis(mode)
                           ? whitened * mode.effective_parameter_basis
                           : whitened);
      CompactModeSpan span;
      span.row_offset = block_offsets[found->second];
      span.rows = item.second.rows();
      span.piece = pieces.size() - 1;
      entry.spans.push_back(span);
    }
    if (!mode_valid[mode_number]) {
      entry.spans.clear();
      continue;
    }
    std::sort(entry.spans.begin(), entry.spans.end(),
              [](const CompactModeSpan& left, const CompactModeSpan& right) {
                return left.row_offset < right.row_offset;
              });
    Eigen::Index compact_rows = 0;
    for (std::size_t index = 0; index < entry.spans.size(); ++index) {
      auto& span = entry.spans[index];
      // A group mapped twice would double count its rows: refuse the mode.
      if (index > 0 &&
          span.row_offset <
              entry.spans[index - 1].row_offset + entry.spans[index - 1].rows) {
        mode_valid[mode_number] = false;
        break;
      }
      span.block_offset = compact_rows;
      compact_rows += span.rows;
    }
    if (!mode_valid[mode_number]) {
      entry.spans.clear();
      continue;
    }
    if (static_cast<std::size_t>(compact_rows) > capacity.max_mode_rows ||
        total_compact_rows + static_cast<std::size_t>(compact_rows) >
            capacity.max_total_compact_rows) {
      compact_usable = false;
      break;
    }
    entry.block.resize(compact_rows, descriptor.dimension);
    for (const auto& span : entry.spans) {
      entry.block.middleRows(span.block_offset, span.rows) = pieces[span.piece];
    }
    entry.normal_cross =
        Eigen::MatrixXd::Zero(window.H.cols(), descriptor.dimension);
    entry.score = Eigen::VectorXd::Zero(descriptor.dimension);
    for (const auto& span : entry.spans) {
      const auto rows = entry.block.middleRows(span.block_offset, span.rows);
      entry.normal_cross +=
          window.H.middleRows(span.row_offset, span.rows).transpose() * rows;
      entry.score += rows.transpose() *
          window.numerics->parity.segment(span.row_offset, span.rows);
    }
    entry.valid = entry.block.allFinite() && entry.normal_cross.allFinite() &&
        entry.score.allFinite();
    if (!entry.valid) {
      mode_valid[mode_number] = false;
      entry.spans.clear();
      continue;
    }
    total_compact_rows += static_cast<std::size_t>(compact_rows);
    NumericalWorkCounters::compactModeStorage(
        static_cast<std::uint64_t>(compact_rows),
        static_cast<std::uint64_t>(descriptor.dimension));
  }
  const bool compact_fallback = !compact_usable;
  if (compact_fallback) NumericalWorkCounters::compactCapacityFallback();
  // Storage accounting for the report: rows the padded form would have cost.
  NumericalWorkCounters::compactPaddedEquivalent(
      static_cast<std::uint64_t>(window.H.rows()) *
      static_cast<std::uint64_t>(total_columns));

  // P0-03: active, batch-disabled and shared-disabled routes all classify the
  // same direct raw-factor response Z=Q2^T D.  This is numerical proof input,
  // not an optional audit optimization.
  const bool audit_input_needed =
      window.square_root && window.square_root->usable();
  Eigen::MatrixXd dense;  // padded fallback form / audit input only
  if (compact_fallback) {
    dense = Eigen::MatrixXd::Zero(window.H.rows(), total_columns);
    for (std::size_t mode_number = 0; mode_number < modes.size(); ++mode_number) {
      const auto& mode = modes[mode_number];
      const auto& descriptor = descriptors[mode_number];
      // The compact loop may have stopped at the capacity bound, so validity
      // has to be re-derived for every mode in the padded form.
      bool valid = descriptor.dimension > 0;
      for (const auto& item : mode.raw_group_maps) {
        const auto found = block_index.find(item.first.value());
        if (found == block_index.end()) {
          valid = false;
          break;
        }
        const auto& block = window.blocks[found->second];
        if (item.second.rows() != block.residual_raw.size() ||
            item.second.cols() != mode.parameter_dimension) {
          valid = false;
          break;
        }
      }
      mode_valid[mode_number] = valid;
      compact[mode_number] = CompactModeEntry();
      if (!valid) continue;
      for (const auto& item : mode.raw_group_maps) {
        const auto found = block_index.find(item.first.value());
        const auto& block = window.blocks[found->second];
        const Eigen::MatrixXd whitened = block.whitener * item.second;
        dense.block(block_offsets[found->second], descriptor.offset,
                    item.second.rows(), descriptor.dimension) =
            useEffectiveBasis(mode)
                ? whitened * mode.effective_parameter_basis : whitened;
      }
      // The fallback pads every mode across all window rows: count the padded
      // per-mode materialization the compact path avoids.
      NumericalWorkCounters::modeDenseAllocation(
          static_cast<std::uint64_t>(window.H.rows()),
          static_cast<std::uint64_t>(descriptor.dimension));
    }
    if (!dense.allFinite()) return results;
  } else if (audit_input_needed) {
    dense = Eigen::MatrixXd::Zero(window.H.rows(), total_columns);
    for (std::size_t mode_number = 0; mode_number < modes.size(); ++mode_number) {
      if (!mode_valid[mode_number]) continue;
      const auto& descriptor = descriptors[mode_number];
      const auto& entry = compact[mode_number];
      for (const auto& span : entry.spans) {
        dense.block(span.row_offset, descriptor.offset, span.rows,
                    descriptor.dimension) =
            entry.block.middleRows(span.block_offset, span.rows);
      }
    }
    if (!dense.allFinite()) return results;
  }

  Eigen::MatrixXd normal_cross =
      Eigen::MatrixXd::Zero(window.H.cols(), total_columns);
  Eigen::VectorXd scores = Eigen::VectorXd::Zero(total_columns);
  if (compact_fallback) {
    normal_cross = window.H.transpose() * dense;
    scores = dense.transpose() * window.numerics->parity;
  } else {
    for (std::size_t mode_number = 0; mode_number < modes.size(); ++mode_number) {
      if (!mode_valid[mode_number]) continue;
      const auto& descriptor = descriptors[mode_number];
      const auto& entry = compact[mode_number];
      normal_cross.middleCols(descriptor.offset, descriptor.dimension) =
          entry.normal_cross;
      scores.segment(descriptor.offset, descriptor.dimension) = entry.score;
    }
  }
  Eigen::MatrixXd rhs(window.H.cols(), 3 + total_columns);
  rhs.leftCols<3>() = window.protected_state_map.transpose();
  rhs.rightCols(total_columns) = normal_cross;
  const Eigen::MatrixXd solved = solveFrozenInformation(
      window.square_root.get(), *window.numerics, window.base_information, rhs);
  if (solved.rows() != window.H.cols() || solved.cols() != rhs.cols() ||
      !solved.allFinite()) return results;
  const Eigen::MatrixXd covariance_modes = solved.rightCols(total_columns);
  Eigen::MatrixXd protected_modes =
      window.protected_state_map * covariance_modes;
  NumericalWorkCounters::faultModeColumns(
      static_cast<std::uint64_t>(total_columns));
  // B1: one implicit Q^T application gives Z = Q2^T D for every fault mode.
  // Z^T Z equals the fault Gram below (verified by COV-02 and NUM-01), and the
  // per-hypothesis block of Z supplies the rank/condition audit that B2 needs;
  // the numeric Gram itself stays the shared normal-equation form so this
  // batch reproduces the P2 numbers exactly.
  Eigen::MatrixXd all_mode_response;
  Eigen::MatrixXd history_mode_response;
  Eigen::MatrixXd candidate_detection_modes;
  Eigen::MatrixXd candidate_protected_modes;
  Eigen::MatrixXd candidate_protected_factor;
  bool history_response_valid = window.history_summary.nu_perp == 0;
  bool candidate_response_valid = false;
  if (audit_input_needed) {
    Eigen::MatrixXd y_response;
    Eigen::MatrixXd z_response;
    window.square_root->faultResponse(dense, &y_response, &z_response);
    if (z_response.rows() > 0 && z_response.cols() == total_columns &&
        z_response.allFinite()) {
      all_mode_response = std::move(z_response);
    }
    const Eigen::MatrixXd protected_factor =
        window.square_root->protectedResponse();
    if (protected_factor.cols() == y_response.rows() &&
        y_response.cols() == total_columns) {
      protected_modes = protected_factor * y_response;
    }
    // The active dual-channel history response is the raw fault payload on
    // the detector-only tail of the unique frozen history block.  Construct
    // it once for all modes; per-hypothesis Gamma_h is then only a <=3 block
    // product and never rescans candidate row roles.
    std::size_t history_block_count = 0;
    for (std::size_t block_number = 0;
         block_number < window.blocks.size(); ++block_number) {
      const auto& block = window.blocks[block_number];
      if (block.whitening_model_id != "history_summary_sqrt_d1") continue;
      ++history_block_count;
      const Eigen::Index history_rows = window.history_summary.nu_perp;
      if (history_rows > 0 && history_rows <= block.residual_whitened.size() &&
          dense.cols() == total_columns) {
        history_mode_response = dense.middleRows(
            block_offsets[block_number] + block.residual_whitened.size() -
                history_rows,
            history_rows);
      }
    }
    history_response_valid = window.history_summary.nu_perp == 0 ||
        (history_block_count == 1 &&
         history_mode_response.rows() == window.history_summary.nu_perp &&
         history_mode_response.cols() == total_columns &&
         history_mode_response.allFinite());
    // Match RawFactorCertificate's established Jacobi-SVD operation order so
    // the reused dual certificate is bit-identical, not merely close.  The
    // decomposition and unique-mode projection occur once per immutable
    // candidate instead of once again inside PL and once per hypothesis.
    Eigen::JacobiSVD<Eigen::MatrixXd> candidate_svd(
        window.H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singular = candidate_svd.singularValues();
    if (singular.size() == window.H.cols() && singular.allFinite() &&
        singular.size() > 0 && singular(singular.size() - 1) > 0.0) {
      const Eigen::MatrixXd coordinates =
          candidate_svd.matrixU().transpose() * dense;
      candidate_detection_modes =
          dense - candidate_svd.matrixU() * coordinates;
      candidate_protected_factor = window.protected_state_map *
          candidate_svd.matrixV() *
          singular.cwiseInverse().asDiagonal();
      candidate_protected_modes = candidate_protected_factor * coordinates;
      candidate_response_valid = candidate_detection_modes.rows() ==
              window.H.rows() &&
          candidate_detection_modes.cols() == total_columns &&
          candidate_detection_modes.allFinite() &&
          candidate_protected_factor.rows() == 3 &&
          candidate_protected_factor.cols() == window.H.cols() &&
          candidate_protected_factor.allFinite() &&
          candidate_protected_modes.rows() == 3 &&
          candidate_protected_modes.cols() == total_columns &&
          candidate_protected_modes.allFinite();
    }
  }
  // B2 (§5.7): cross blocks are computed on demand for the (mode, mode) pairs the
  // hypotheses actually reference; the previous unconditional all-mode Gram is
  // gone, so an order=1 configuration never pays for pair cross terms.  The
  // requested set is collected and evaluated *before* the worker pool starts:
  // the cache is read-only inside the parallel region (mutating shared state
  // from multiple workers was a real crash in the first B2 iteration).
  // The compact path evaluates a cross block from the intersections of the two
  // modes' group spans, so the zero-padded rows never enter the product.
  auto blockOf = [&](const ModeDescriptor& first,
                     const ModeDescriptor& second) -> Eigen::MatrixXd {
    const Eigen::Index left_offset = first.offset;
    const Eigen::Index right_offset = second.offset;
    if (all_mode_response.cols() == total_columns) {
      return all_mode_response
                 .middleCols(left_offset, first.dimension).transpose() *
          all_mode_response.middleCols(right_offset, second.dimension);
    }
    Eigen::MatrixXd value;
    if (!compact_fallback) {
      const auto& left = compact[first.mode_number];
      const auto& right = compact[second.mode_number];
      value = Eigen::MatrixXd::Zero(first.dimension, second.dimension);
      std::size_t left_index = 0;
      std::size_t right_index = 0;
      while (left_index < left.spans.size() &&
             right_index < right.spans.size()) {
        const auto& left_span = left.spans[left_index];
        const auto& right_span = right.spans[right_index];
        if (left_span.row_offset + left_span.rows <= right_span.row_offset) {
          ++left_index;
          continue;
        }
        if (right_span.row_offset + right_span.rows <= left_span.row_offset) {
          ++right_index;
          continue;
        }
        // Group spans are disjoint blocks, so overlapping spans coincide.
        const Eigen::Index overlap_begin =
            std::max(left_span.row_offset, right_span.row_offset);
        const Eigen::Index overlap_end = std::min(
            left_span.row_offset + left_span.rows,
            right_span.row_offset + right_span.rows);
        const Eigen::Index overlap_rows = overlap_end - overlap_begin;
        value +=
            left.block
                .middleRows(left_span.block_offset + overlap_begin -
                                left_span.row_offset,
                            overlap_rows)
                .transpose() *
            right.block.middleRows(right_span.block_offset + overlap_begin -
                                       right_span.row_offset,
                                   overlap_rows);
        ++left_index;
        ++right_index;
      }
      value -= left.normal_cross.transpose() *
          covariance_modes.middleCols(right_offset, second.dimension);
      return value;
    }
    value = dense.middleCols(left_offset, first.dimension).transpose() *
        dense.middleCols(right_offset, second.dimension);
    value -= normal_cross.middleCols(left_offset, first.dimension).transpose() *
        covariance_modes.middleCols(right_offset, second.dimension);
    return value;
  };
  std::map<std::pair<std::size_t, std::size_t>, Eigen::MatrixXd>
      cross_block_cache;
  {
    std::set<std::pair<std::size_t, std::size_t>> requested;
    for (const auto& hypothesis : hypothesis_descriptors) {
      const auto& parts = hypothesis.parts;
      const bool valid = hypothesis.all_ids_known &&
          std::all_of(parts.begin(), parts.end(),
                      [&](std::size_t index) { return mode_valid[index]; });
      if (!valid) continue;
      for (std::size_t left = 0; left < parts.size(); ++left) {
        for (std::size_t right = 0; right < parts.size(); ++right) {
          requested.emplace(std::min(parts[left], parts[right]),
                            std::max(parts[left], parts[right]));
        }
      }
    }
    for (const auto& key : requested) {
      cross_block_cache.emplace(
          key, blockOf(descriptors[key.first], descriptors[key.second]));
      NumericalWorkCounters::faultCrossBlock();
    }
  }
  const auto& blocks = cross_block_cache;
  auto crossBlock = [&blocks, &blockOf](const ModeDescriptor& left,
                                        const ModeDescriptor& right)
      -> Eigen::MatrixXd {
    const bool ordered = left.mode_number <= right.mode_number;
    const std::pair<std::size_t, std::size_t> key = ordered
        ? std::make_pair(left.mode_number, right.mode_number)
        : std::make_pair(right.mode_number, left.mode_number);
    const auto found = blocks.find(key);
    if (found == blocks.end()) {
      // Unrequested pairing (must not happen: the requested set is collected
      // from the same hypotheses): recompute locally, touching no shared state.
      return ordered ? blockOf(left, right)
                     : Eigen::MatrixXd(blockOf(right, left).transpose());
    }
    NumericalWorkCounters::faultCrossBlockCacheHit();
    if (ordered) return found->second;
    return Eigen::MatrixXd(found->second.transpose());
  };
  auto context = std::make_shared<FrozenHypothesisNumerics>();
  auto dual_context = std::make_shared<FrozenHypothesisDualNumerics>();
  if (admission) {
    dual_context->window_owner = admission->owner;
    dual_context->window_payload = admission->payload;
  }
  context->window_id = window.id;
  context->version = window.version;
  context->window_content_fingerprint = window.numerics->content_fingerprint;
  context->numerical_contract_fingerprint =
      window.numerics->numerical_contract_fingerprint;
  context->hypothesis_fingerprint = hypothesisSetFingerprint(*hypotheses);
  context->fault_mode_fingerprint = faultModeSetFingerprint(modes);
  context->fault_model_policy_fingerprint =
      config.fault_model_policy_fingerprint;
  context->protected_covariance = window.square_root &&
      window.square_root->usable()
      ? window.square_root->protectedCovariance()
      : window.protected_state_map * solved.leftCols<3>();
  dual_context->candidate_detection_modes = candidate_detection_modes;
  dual_context->candidate_protected_modes = candidate_protected_modes;
  dual_context->candidate_protected_factor = candidate_protected_factor;
  dual_context->candidate_mode_active.resize(modes.size(), 0);
  if (dense.cols() == total_columns) {
    for (std::size_t index = 0; index < descriptors.size(); ++index) {
      const auto& descriptor = descriptors[index];
      dual_context->candidate_mode_active[index] =
          !dense.middleCols(descriptor.offset, descriptor.dimension)
               .isZero(0.0);
    }
  }
  std::set<std::uint64_t> inactive_candidate_modes;
  for (std::size_t index = 0; index < modes.size(); ++index) {
    if (!dual_context->candidate_mode_active[index]) {
      inactive_candidate_modes.insert(modes[index].id.value());
    }
  }
  auto candidate_hypotheses = *hypotheses;
  candidate_hypotheses.erase(std::remove_if(
      candidate_hypotheses.begin(), candidate_hypotheses.end(),
      [&](FaultHypothesisV2& hypothesis) {
        hypothesis.modes.erase(std::remove_if(
            hypothesis.modes.begin(), hypothesis.modes.end(),
            [&](FaultModeId mode) {
              return inactive_candidate_modes.count(mode.value()) != 0;
            }), hypothesis.modes.end());
        return hypothesis.modes.empty();
      }), candidate_hypotheses.end());
  dual_context->candidate_hypothesis_fingerprint =
      hypothesisSetFingerprint(candidate_hypotheses);
  if (candidate_protected_factor.rows() == 3) {
    dual_context->candidate_protected_covariance =
        candidate_protected_factor * candidate_protected_factor.transpose();
  }
  // B2 storage discipline: report which storage form served this window.
  context->compact_rows = compact_fallback
      ? static_cast<std::size_t>(window.H.rows()) *
          static_cast<std::size_t>(total_columns)
      : total_compact_rows;
  context->compact_columns = compact_fallback
      ? 0 : static_cast<std::size_t>(total_columns);
  context->compact_mode_used = !compact_fallback;
  context->compact_capacity_exceeded = compact_fallback;
  context->pl_entries.resize(hypotheses->size());
  dual_context->dual_blocks.resize(hypotheses->size());
  context->mode_count = modes.size();
  context->hypothesis_count = hypotheses->size();

  auto evaluate_range = [&](std::size_t begin, std::size_t end) {
    WorkDelta work;
    for (std::size_t hypothesis_index = begin;
         hypothesis_index < end; ++hypothesis_index) {
      auto& hypothesis = (*hypotheses)[hypothesis_index];
      auto& evidence = results[hypothesis_index];
      auto& pl_entry = context->pl_entries[hypothesis_index];
      auto& dual_block = dual_context->dual_blocks[hypothesis_index];
      evidence.hypothesis = hypothesis.id;
      evidence.all_in_statistic = window.numerics->statistic;
      pl_entry.hypothesis = hypothesis.id;
      dual_block.hypothesis = hypothesis.id;
      int dimension = 0;
      int physical_dimension = 0;
      bool found_all = !hypothesis.modes.empty();
      const auto& parts = hypothesis_descriptors[hypothesis_index].parts;
      found_all = hypothesis_descriptors[hypothesis_index].all_ids_known;
      for (const auto part : parts) {
        if (!mode_valid[part]) {
          found_all = false;
          break;
        }
        const auto& descriptor = descriptors[part];
        dimension += descriptor.dimension;
        physical_dimension += descriptor.physical_dimension;
      }
      if (!found_all || dimension <= 0) {
        failDimension(&hypothesis, &evidence, physical_dimension);
        continue;
      }
      // C3 wiring (§8.4): record the comparability identity of this hypothesis.
      // Parts that disagree on the physical unit yield Unknown rather than a
      // guess; the parameter dimension is the one actually analysed.
      evidence.unit_kind = descriptors[parts.front()].unit_kind;
      for (const auto part : parts) {
        if (descriptors[part].unit_kind != evidence.unit_kind) {
          evidence.unit_kind = FaultUnitKind::Unknown;
          break;
        }
      }
      evidence.parameter_dimension = static_cast<std::size_t>(dimension);
      double raw_factor_scale_squared = 0.0;
      if (dense.cols() == total_columns) {
        for (const auto part : parts) {
          const auto& descriptor = descriptors[part];
          raw_factor_scale_squared += dense.middleCols(
              descriptor.offset, descriptor.dimension).squaredNorm();
        }
      }
      const double raw_factor_scale =
          std::sqrt(raw_factor_scale_squared);
      // B2 capacity bound: a hypothesis whose parameter dimension exceeds the
      // configured bound is refused fail-closed and counted, instead of
      // allocating unbounded temporaries.
      if (dimension > config.compact_capacity.max_hypothesis_dimension) {
        NumericalWorkCounters::hypothesisCapacityRefusal();
        auto& monitor = hypothesis.monitorability;
        monitor.parameter_dimension = dimension;
        monitor.physical_parameter_dimension = physical_dimension;
        monitor.reason = "hypothesis dimension exceeds the configured capacity";
        hypothesis.monitored = false;
        evidence.monitorability = monitor;
        evidence.plausible = false;
        pl_entry.monitorability = monitor;
        pl_entry.valid = false;
        pl_entry.z_classification = 3;  // dangerous / unavailable
        continue;
      }
      // B2 (§5.7): refuse only a concatenated hypothesis whose parameter
      // blocks are structurally unsupported or explicitly share physical
      // parameters.  Numerical dependence is classified below from the
      // global stacked response; it must not remove a declared hypothesis.
      if (parts.size() > 1) {
        std::string independence_reason;
        if (!hypothesisParametersIndependent(modes, hypothesis.modes,
                                             &independence_reason)) {
          auto& monitor = hypothesis.monitorability;
          monitor.parameter_dimension = dimension;
          monitor.physical_parameter_dimension = physical_dimension;
          monitor.reason = independence_reason;
          hypothesis.monitored = false;
          evidence.monitorability = monitor;
          evidence.plausible = false;
          pl_entry.monitorability = monitor;
          pl_entry.valid = false;
          pl_entry.z_classification = 3;  // dangerous / unavailable
          continue;
        }
      }
      // B1 audit: detection-space response of this hypothesis, classified from
      // the small SVD of Z_h = Q2^T D_h (never from the squared normal form).
      // Applied to every hypothesis; the added work is one small SVD whose size
      // is (m - rank) x dimension, counted as square_root_qt_columns.
      // B3: direction-level evidence produced by the tri-state classification.
      Eigen::Vector3d classification_axis_residual = Eigen::Vector3d::Zero();
      Eigen::Vector3d classification_harmless_slopes =
          Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      bool classification_evaluated = false;
      Eigen::MatrixXd raw_detection_factor;
      if (all_mode_response.cols() > 0 && config.enable_shared_context) {
        std::vector<Eigen::MatrixXd> z_parts;
        int width = 0;
        for (const auto part : parts) {
          const auto& descriptor = descriptors[part];
          if (descriptor.offset + descriptor.dimension >
              all_mode_response.cols()) continue;
          z_parts.push_back(all_mode_response.middleCols(
              descriptor.offset, descriptor.dimension));
          width += descriptor.dimension;
        }
        if (width == dimension) {
          Eigen::MatrixXd z_h(all_mode_response.rows(), dimension);
          int offset = 0;
          for (const auto& part : z_parts) {
            z_h.middleCols(offset, part.cols()) = part;
            offset += static_cast<int>(part.cols());
          }
          raw_detection_factor = z_h;
          Eigen::MatrixXd g_h(3, dimension);
          offset = 0;
          for (const auto part : parts) {
            const auto& descriptor = descriptors[part];
            g_h.middleCols(offset, descriptor.dimension) =
                protected_modes.middleCols(descriptor.offset,
                                           descriptor.dimension);
            offset += descriptor.dimension;
          }
          pl_entry.z_classification = classifyDetectionResponse(
              z_h, g_h,
              window.numerics
                  ? window.numerics->numerical_contract.rank_tolerance : 1e-10,
              &pl_entry.z_smallest_singular_value, &pl_entry.z_condition,
              &pl_entry.z_rank, &classification_axis_residual,
              &classification_harmless_slopes, raw_factor_scale);
          classification_evaluated = pl_entry.z_classification != 0;
        }
      }
      if (dimension <= 3 && parts.size() <= 2) {
        Eigen::Matrix3d gram = Eigen::Matrix3d::Zero();
        Eigen::Vector3d score = Eigen::Vector3d::Zero();
        Eigen::Matrix3d protected_fault = Eigen::Matrix3d::Zero();
        int left_offset = 0;
        for (std::size_t left = 0; left < parts.size(); ++left) {
          const auto& left_descriptor = descriptors[parts[left]];
          score.segment(left_offset, left_descriptor.dimension) =
              scores.segment(left_descriptor.offset, left_descriptor.dimension);
          protected_fault.middleCols(left_offset, left_descriptor.dimension) =
              protected_modes.middleCols(left_descriptor.offset,
                                          left_descriptor.dimension);
          int right_offset = 0;
          for (std::size_t right = 0; right < parts.size(); ++right) {
            const auto& right_descriptor = descriptors[parts[right]];
            if (left == right) {
              gram.block(left_offset, right_offset,
                         left_descriptor.dimension, right_descriptor.dimension) =
                  crossBlock(left_descriptor, right_descriptor);
            } else {
              gram.block(left_offset, right_offset,
                         left_descriptor.dimension, right_descriptor.dimension) =
                  crossBlock(left_descriptor, right_descriptor);
            }
            right_offset += right_descriptor.dimension;
          }
          left_offset += left_descriptor.dimension;
        }
        dual_block.dimension = dimension;
        dual_block.total_gram = gram;
        dual_block.score = score;
        dual_block.protected_response = protected_fault;
        if (candidate_detection_modes.cols() == total_columns &&
            candidate_protected_modes.cols() == total_columns) {
          Eigen::MatrixXd direct_factor(candidate_detection_modes.rows(),
                                        dimension);
          int direct_offset = 0;
          for (const auto part : parts) {
            const auto& descriptor = descriptors[part];
            direct_factor.middleCols(direct_offset, descriptor.dimension) =
                candidate_detection_modes.middleCols(
                    descriptor.offset, descriptor.dimension);
            dual_block.protected_response.middleCols(
                direct_offset, descriptor.dimension) =
                candidate_protected_modes.middleCols(
                    descriptor.offset, descriptor.dimension);
            direct_offset += descriptor.dimension;
          }
          dual_block.total_gram.topLeftCorner(dimension, dimension) =
              direct_factor.transpose() * direct_factor;
        }
        if (history_mode_response.cols() == total_columns) {
          Eigen::MatrixXd history_factor(history_mode_response.rows(),
                                         dimension);
          int history_offset = 0;
          for (const auto part : parts) {
            const auto& descriptor = descriptors[part];
            history_factor.middleCols(history_offset, descriptor.dimension) =
                history_mode_response.middleCols(descriptor.offset,
                                                  descriptor.dimension);
            history_offset += descriptor.dimension;
          }
          dual_block.history_gram.topLeftCorner(dimension, dimension) =
              history_factor.transpose() * history_factor;
        }
        dual_block.valid = candidate_response_valid &&
            history_response_valid && raw_detection_factor.rows() > 0 &&
            raw_detection_factor.cols() == dimension &&
            dual_block.total_gram.allFinite() &&
            dual_block.history_gram.allFinite() &&
            dual_block.score.allFinite() &&
            dual_block.protected_response.allFinite();
        if (dimension == 1) {
          analyzeFixed<1>(gram.topLeftCorner<1, 1>(), &raw_detection_factor,
                          score.head<1>(),
                          protected_fault.leftCols<1>(),
                          admittedProofIdentity(window,
                                                numerical_proof_identity),
                          raw_factor_scale,
                          physical_dimension,
                          window.numerics->statistic, squared_detector_threshold,
                          config, &hypothesis, &evidence, &pl_entry, &work,
                          proof_arena);
        } else if (dimension == 2) {
          analyzeFixed<2>(gram.topLeftCorner<2, 2>(), &raw_detection_factor,
                          score.head<2>(),
                          protected_fault.leftCols<2>(),
                          admittedProofIdentity(window,
                                                numerical_proof_identity),
                          raw_factor_scale,
                          physical_dimension,
                          window.numerics->statistic, squared_detector_threshold,
                          config, &hypothesis, &evidence, &pl_entry, &work,
                          proof_arena);
        } else {
          analyzeFixed<3>(gram, &raw_detection_factor, score, protected_fault,
                          admittedProofIdentity(window,
                                                numerical_proof_identity),
                          raw_factor_scale,
                          physical_dimension,
                          window.numerics->statistic, squared_detector_threshold,
                          config, &hypothesis, &evidence, &pl_entry, &work,
                          proof_arena);
        }
        dual_block.profile_statistic = evidence.profile_j;
      } else {
        Eigen::MatrixXd gram(dimension, dimension);
        Eigen::VectorXd score(dimension);
        Eigen::MatrixXd protected_fault(3, dimension);
        int left_offset = 0;
        for (std::size_t left = 0; left < parts.size(); ++left) {
          const auto& left_descriptor = descriptors[parts[left]];
          score.segment(left_offset, left_descriptor.dimension) =
              scores.segment(left_descriptor.offset, left_descriptor.dimension);
          protected_fault.middleCols(left_offset, left_descriptor.dimension) =
              protected_modes.middleCols(left_descriptor.offset,
                                          left_descriptor.dimension);
          int right_offset = 0;
          for (std::size_t right = 0; right < parts.size(); ++right) {
            const auto& right_descriptor = descriptors[parts[right]];
            if (left == right && left_descriptor.dimension <= 3) {
              gram.block(left_offset, right_offset, left_descriptor.dimension,
                         right_descriptor.dimension) =
                  crossBlock(left_descriptor, right_descriptor);
            } else {
              gram.block(left_offset, right_offset, left_descriptor.dimension,
                         right_descriptor.dimension) =
                  crossBlock(left_descriptor, right_descriptor);
            }
            right_offset += right_descriptor.dimension;
          }
          left_offset += left_descriptor.dimension;
        }
        analyzeDynamic(gram, &raw_detection_factor, score, &protected_fault, true,
                       admittedProofIdentity(window,
                                             numerical_proof_identity),
                       raw_factor_scale,
                       physical_dimension,
                       window.numerics->statistic, squared_detector_threshold,
                       config, &hypothesis, &evidence, &pl_entry, &work,
                       proof_arena);
      }
      // B3 (section 5.5 / 5.8): the detection-space classification decides
      // availability.  A structurally harmless nullspace keeps a finite bound
      // through the projected path ||g V_r Sigma_r^-1||; a dangerous or
      // numerically indistinguishable response is unavailable with a
      // direction-level reason (never a dictionary-wide rejection).
      if (classification_evaluated) {
        if (pl_entry.z_classification == 2 &&
            classification_harmless_slopes.allFinite()) {
          pl_entry.protected_slopes = classification_harmless_slopes;
          evidence.monitorability.protected_slopes =
              classification_harmless_slopes;
          // Structurally harmless: the fault is invisible in the detection
          // space but fully projected out of the protected state, so the
          // projected path still yields a finite bound (Gamma stays audit
          // only).  The reason records which channel was used.
          pl_entry.valid = true;
          pl_entry.gram_spd = false;
          pl_entry.bound_from_projected_path = true;
          pl_entry.monitorability.monitorable = true;
          hypothesis.monitored = true;
          evidence.monitorability.monitorable = true;
          evidence.monitorability.reason =
              "harmless detection nullspace: finite bound from projected "
              "response (Gamma audit only)";
          evidence.plausible = true;
        } else if (pl_entry.z_classification == 3 ||
                   pl_entry.z_classification == 4) {
          const bool dangerous = pl_entry.z_classification == 3;
          std::string reason = dangerous
              ? "dangerous detection nullspace: protected response not "
                "observable in the retained subspace (axis residuals "
              : "numerically indistinguishable detection response: reference "
                "fallback required (axis residuals ";
          for (int axis = 0; axis < 3; ++axis) {
            if (axis) reason += ",";
            reason += std::string(1, "xyz"[axis]) + "=" +
                std::to_string(classification_axis_residual(axis));
          }
          reason += "); modes";
          for (const auto id : hypothesis.modes) {
            reason += " " + std::to_string(id.value());
          }
          auto& monitor = hypothesis.monitorability;
          monitor.parameter_dimension = dimension;
          monitor.physical_parameter_dimension = physical_dimension;
          monitor.reason = reason;
          hypothesis.monitored = false;
          evidence.monitorability = monitor;
          evidence.plausible = false;
          pl_entry.monitorability = monitor;
          pl_entry.valid = false;
          pl_entry.protected_slopes = Eigen::Vector3d::Constant(
              std::numeric_limits<double>::infinity());
        }
      }
    }
    publish(work);
  };

  const std::size_t active_workers = std::max<std::size_t>(1,
      std::min(config.hypothesis_workers, hypotheses->size()));
  if (worker_pool && active_workers > 1) {
    context->worker_blocks = active_workers;
    NumericalWorkCounters::hypothesisParallelBlocks(active_workers);
    // R11: each immutable hypothesis owns one fixed output slot. Dynamic
    // claiming balances exact/SVD fallback tails without allowing completion
    // order to affect the deterministic index-order reduction below.
    constexpr std::size_t kRetainedWorkerScratchLimitBytes = 64u << 20;
    worker_pool->runFlat(hypotheses->size(), active_workers,
        kRetainedWorkerScratchLimitBytes,
        [&](std::size_t hypothesis_index, std::size_t, RankUpdateScratch&) {
          evaluate_range(hypothesis_index, hypothesis_index + 1);
        });
  } else {
    context->worker_blocks = 1;
    evaluate_range(0, hypotheses->size());
  }
  for (std::size_t hypothesis_index = 0;
       hypothesis_index < hypotheses->size(); ++hypothesis_index) {
    const auto& hypothesis = (*hypotheses)[hypothesis_index];
    const int dimension =
        hypothesis_descriptors[hypothesis_index].summary_dimension;
    if (dimension == 1) ++context->dimension_one_count;
    else if (dimension == 2) ++context->dimension_two_count;
    else if (dimension == 3) ++context->dimension_three_count;
    else ++context->dimension_other_count;
    if (dimension >= 1 && dimension <= 3 && hypothesis.modes.size() <= 2) {
      ++context->low_dimensional_count;
    } else {
      ++context->generic_fallback_count;
    }
  }
  context->bytes = sizeof(*context) +
      context->pl_entries.capacity() * sizeof(FrozenHypothesisPlEntry);
  context->valid = context->protected_covariance.allFinite() &&
      context->pl_entries.size() == hypotheses->size() &&
      std::all_of(context->pl_entries.begin(), context->pl_entries.end(),
          [&](const FrozenHypothesisPlEntry& entry) {
            FrozenHypothesisPlProofV1 proof;
            return !entry.valid || (proof_arena
                ? detail::readAndValidateScopedFrozenHypothesisProof(entry, *proof_arena)
                : (frozenHypothesisPlProof(entry, &proof) &&
                   validateFrozenHypothesisPlEntry(proof)));
          });
  if (!context->valid) context->reason = "shared hypothesis context is incomplete";
  std::shared_ptr<const FrozenHypothesisNumerics> sealed_base = context;
  dual_context->base_owner = sealed_base;
  dual_context->base_payload = sealed_base.get();
  dual_context->valid = context->valid && admission;
  if (dual_context->valid) {
    dual_context->proof_digest = frozenDualProofDigest(
        *sealed_base, *dual_context);
    dual_context->valid = dual_context->proof_digest != 0;
  }
  if (!dual_context->valid) {
    dual_context->reason = "sealed dual hypothesis context is unavailable";
  }
  if (shared) *shared = sealed_base;
  if (shared_dual) *shared_dual = std::move(dual_context);
  return results;
}

std::vector<FaultModeEvidence> evaluateMapped(
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    const HypothesisEvaluationConfig& config,
    std::uint64_t numerical_proof_identity,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared,
    AttemptProofArena* proof_arena) {
  struct Projection {
    Eigen::MatrixXd dense;
    Eigen::MatrixXd detection_response;
    Eigen::MatrixXd normal_cross;
    Eigen::MatrixXd covariance_cross;
    Eigen::MatrixXd protected_fault;
    Eigen::VectorXd score;
    int physical_dimension = 0;
  };
  std::map<std::uint64_t, Projection> projected_modes;
  std::map<std::uint64_t, int> block_offsets;
  int row_offset = 0;
  for (const auto& block : window.blocks) {
    block_offsets[block.group_id.value()] = row_offset;
    row_offset += block.residual_whitened.size();
  }
  for (const auto& mode : modes) {
    const bool effective = useEffectiveBasis(mode);
    const int dimension = effective ? mode.effective_parameter_dimension
                                    : mode.parameter_dimension;
    Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(window.H.rows(), dimension);
    bool valid = dimension > 0;
    for (const auto& item : mode.raw_group_maps) {
      const auto offset = block_offsets.find(item.first.value());
      const auto block = std::find_if(window.blocks.begin(), window.blocks.end(),
          [&](const LinearizedFactorBlock& value) {
            return value.group_id == item.first;
          });
      if (offset == block_offsets.end() || block == window.blocks.end() ||
          item.second.rows() != block->residual_raw.size() ||
          item.second.cols() != mode.parameter_dimension) {
        valid = false;
        break;
      }
      const Eigen::MatrixXd whitened = block->whitener * item.second;
      dense.block(offset->second, 0, item.second.rows(), dimension) =
          effective ? whitened * mode.effective_parameter_basis : whitened;
    }
    if (!valid || !dense.allFinite()) continue;
    // B2: this legacy route materializes one zero-padded dense block per mode
    // (all window rows), which is exactly what the compact path eliminates.
    NumericalWorkCounters::modeDenseAllocation(
        static_cast<std::uint64_t>(dense.rows()),
        static_cast<std::uint64_t>(dense.cols()));
    Projection projection;
    projection.dense = std::move(dense);
    if (window.square_root && window.square_root->usable()) {
      Eigen::MatrixXd y_response;
      window.square_root->faultResponse(
          projection.dense, &y_response, &projection.detection_response);
      const Eigen::MatrixXd protected_factor =
          window.square_root->protectedResponse();
      if (protected_factor.cols() == y_response.rows()) {
        projection.protected_fault = protected_factor * y_response;
      }
    }
    projection.normal_cross = window.H.transpose() * projection.dense;
    projection.physical_dimension = mode.parameter_dimension;
    projected_modes.emplace(mode.id.value(), std::move(projection));
  }
  Eigen::Index solve_columns = config.enable_shared_context ? 3 : 0;
  for (const auto& item : projected_modes)
    solve_columns += item.second.normal_cross.cols();
  Eigen::MatrixXd combined_cross(window.H.cols(), solve_columns);
  Eigen::Index solve_offset = 0;
  if (config.enable_shared_context) {
    combined_cross.leftCols<3>() = window.protected_state_map.transpose();
    solve_offset = 3;
  }
  for (const auto& item : projected_modes) {
    const Eigen::Index columns = item.second.normal_cross.cols();
    combined_cross.middleCols(solve_offset, columns) = item.second.normal_cross;
    solve_offset += columns;
  }
  const Eigen::MatrixXd combined_solutions = solveFrozenInformation(
      window.square_root.get(), *window.numerics,
      window.base_information, combined_cross);
  if (combined_solutions.rows() != window.H.cols() ||
      combined_solutions.cols() != solve_columns ||
      !combined_solutions.allFinite()) return {};

  auto context = std::make_shared<FrozenHypothesisNumerics>();
  if (config.enable_shared_context) {
    context->window_id = window.id;
    context->version = window.version;
    context->window_content_fingerprint = window.numerics->content_fingerprint;
    context->numerical_contract_fingerprint =
        window.numerics->numerical_contract_fingerprint;
    context->hypothesis_fingerprint = hypothesisSetFingerprint(*hypotheses);
    context->fault_mode_fingerprint = faultModeSetFingerprint(modes);
    context->fault_model_policy_fingerprint =
        config.fault_model_policy_fingerprint;
    context->protected_covariance = window.square_root &&
        window.square_root->usable()
        ? window.square_root->protectedCovariance()
        : window.protected_state_map * combined_solutions.leftCols<3>();
    context->pl_entries.resize(hypotheses->size());
    context->mode_count = modes.size();
    context->hypothesis_count = hypotheses->size();
  }
  solve_offset = config.enable_shared_context ? 3 : 0;
  for (auto& item : projected_modes) {
    const Eigen::Index columns = item.second.normal_cross.cols();
    item.second.covariance_cross =
        combined_solutions.middleCols(solve_offset, columns);
    item.second.score = item.second.dense.transpose() * window.numerics->parity;
    // The protected response is part of the common numerical certificate, not
    // an optimization-only artifact.  Both the legacy evidence path and the
    // shared PL path must therefore classify the same Gram nullspace.
    if (item.second.protected_fault.rows() != 3 ||
        item.second.protected_fault.cols() != columns) {
      item.second.protected_fault =
          window.protected_state_map * item.second.covariance_cross;
    }
    solve_offset += columns;
  }

  std::vector<FaultModeEvidence> results;
  results.reserve(hypotheses->size());
  std::map<std::pair<std::uint64_t, std::uint64_t>, Eigen::MatrixXd> pair_grams;
  WorkDelta work;
  for (std::size_t hypothesis_index = 0;
       hypothesis_index < hypotheses->size(); ++hypothesis_index) {
    auto& hypothesis = (*hypotheses)[hypothesis_index];
    FaultModeEvidence evidence;
    evidence.hypothesis = hypothesis.id;
    evidence.all_in_statistic = window.numerics->statistic;
    int columns = 0;
    int physical_dimension = 0;
    bool found_all = !hypothesis.modes.empty();
    std::vector<std::pair<std::uint64_t, const Projection*>> parts;
    for (const auto id : hypothesis.modes) {
      const auto found = projected_modes.find(id.value());
      if (found == projected_modes.end()) {
        found_all = false;
        break;
      }
      columns += found->second.dense.cols();
      physical_dimension += found->second.physical_dimension;
      parts.emplace_back(id.value(), &found->second);
    }
    if (!found_all) {
      failDimension(&hypothesis, &evidence, physical_dimension);
      results.push_back(std::move(evidence));
      continue;
    }
    Eigen::MatrixXd gram = Eigen::MatrixXd::Zero(columns, columns);
    Eigen::VectorXd score = Eigen::VectorXd::Zero(columns);
    Eigen::MatrixXd protected_fault = Eigen::MatrixXd::Zero(3, columns);
    Eigen::MatrixXd raw_detection_factor;
    bool raw_factor_available = !parts.empty();
    Eigen::Index raw_factor_rows = parts.empty()
        ? 0 : parts.front().second->detection_response.rows();
    for (const auto& part : parts) {
      raw_factor_available = raw_factor_available && raw_factor_rows > 0 &&
          part.second->detection_response.rows() == raw_factor_rows &&
          part.second->detection_response.cols() == part.second->dense.cols() &&
          part.second->detection_response.allFinite();
    }
    if (raw_factor_available) raw_detection_factor.resize(raw_factor_rows, columns);
    double raw_factor_scale_squared = 0.0;
    int left_column = 0;
    for (std::size_t left = 0; left < parts.size(); ++left) {
      const int left_size = parts[left].second->dense.cols();
      score.segment(left_column, left_size) = parts[left].second->score;
      raw_factor_scale_squared += parts[left].second->dense.squaredNorm();
      protected_fault.middleCols(left_column, left_size) =
          parts[left].second->protected_fault;
      if (raw_factor_available) {
        raw_detection_factor.middleCols(left_column, left_size) =
            parts[left].second->detection_response;
      }
      int right_column = 0;
      for (std::size_t right = 0; right < parts.size(); ++right) {
        const int right_size = parts[right].second->dense.cols();
        const std::pair<std::uint64_t, std::uint64_t> key{
            std::min(parts[left].first, parts[right].first),
            std::max(parts[left].first, parts[right].first)};
        auto found = pair_grams.find(key);
        if (found == pair_grams.end()) {
          const auto& first = projected_modes.at(key.first);
          const auto& second = projected_modes.at(key.second);
          Eigen::MatrixXd value;
          if (first.detection_response.rows() > 0 &&
              first.detection_response.rows() ==
                  second.detection_response.rows()) {
            value = first.detection_response.transpose() *
                second.detection_response;
          } else {
            value = first.dense.transpose() * second.dense -
                first.normal_cross.transpose() * second.covariance_cross;
          }
          found = pair_grams.emplace(key, std::move(value)).first;
        }
        gram.block(left_column, right_column, left_size, right_size) =
            parts[left].first <= parts[right].first
                ? found->second : found->second.transpose();
        right_column += right_size;
      }
      left_column += left_size;
    }
    FrozenHypothesisPlEntry* entry = nullptr;
    if (config.enable_shared_context) {
      entry = &context->pl_entries[hypothesis_index];
      entry->hypothesis = hypothesis.id;
    }
    std::string independence_reason;
    const bool allow_harmless_nullspace = hypothesis.modes.size() <= 1 ||
        hypothesisParametersIndependent(modes, hypothesis.modes,
                                        &independence_reason);
    analyzeDynamic(gram,
                   raw_factor_available ? &raw_detection_factor : nullptr,
                   score, &protected_fault, allow_harmless_nullspace,
                   admittedProofIdentity(window, numerical_proof_identity),
                   std::sqrt(raw_factor_scale_squared),
                   physical_dimension, window.numerics->statistic,
                   squared_detector_threshold, config, &hypothesis,
                   &evidence, entry, &work, proof_arena);
    results.push_back(std::move(evidence));
  }
  publish(work);
  if (config.enable_shared_context) {
    context->generic_fallback_count = hypotheses->size();
    context->dimension_other_count = hypotheses->size();
    context->worker_blocks = 1;
    context->bytes = sizeof(*context) +
        context->pl_entries.capacity() * sizeof(FrozenHypothesisPlEntry);
    context->valid = context->protected_covariance.allFinite() &&
        std::all_of(context->pl_entries.begin(), context->pl_entries.end(),
            [&](const FrozenHypothesisPlEntry& entry) {
              FrozenHypothesisPlProofV1 proof;
              return !entry.valid || (proof_arena
                  ? detail::readAndValidateScopedFrozenHypothesisProof(entry, *proof_arena)
                  : (frozenHypothesisPlProof(entry, &proof) &&
                     validateFrozenHypothesisPlEntry(proof)));
            });
    if (!context->valid) context->reason = "shared hypothesis context is incomplete";
    if (shared) *shared = std::move(context);
  }
  return results;
}



}  // namespace

bool validateFrozenHypothesisDualProof(
    const FrozenWindowAdmission& admission,
    const FrozenHypothesisNumerics& base,
    const FrozenHypothesisDualNumerics& frozen,
    std::string* reason) {
  auto reject = [&](const std::string& message) {
    if (reason) *reason = message;
    return false;
  };
  if (!admission || !frozen.window_owner || !frozen.window_payload ||
      !frozen.base_owner || !frozen.base_payload) {
    return reject("frozen dual context has no immutable owner binding");
  }
  const bool same_owner =
      frozen.window_owner.get() == admission.owner.get() &&
      !frozen.window_owner.owner_before(admission.owner) &&
      !admission.owner.owner_before(frozen.window_owner);
  if (!same_owner || frozen.window_payload != admission.payload ||
      frozen.window_owner->payload.get() != frozen.window_payload ||
      frozen.base_payload != &base || frozen.base_owner.get() != &base) {
    return reject("frozen dual context owner/payload identity mismatch");
  }
  const auto& window = admission.window();
  if (!frozen.valid || frozen.proof_digest == 0 || !window.numerics ||
      base.window_id != window.id || !(base.version == window.version) ||
      base.window_content_fingerprint != admission.owner->content_hash ||
      base.window_content_fingerprint !=
          window.numerics->content_fingerprint ||
      base.numerical_contract_fingerprint !=
          window.numerics->numerical_contract_fingerprint) {
    return reject("frozen dual context sealed window identity mismatch");
  }
  if (base.mode_count != frozen.candidate_mode_active.size() ||
      base.hypothesis_count != base.pl_entries.size() ||
      base.hypothesis_count != frozen.dual_blocks.size() ||
      base.dimension_one_count + base.dimension_two_count +
              base.dimension_three_count + base.dimension_other_count !=
          base.hypothesis_count ||
      base.low_dimensional_count + base.generic_fallback_count !=
          base.hypothesis_count) {
    return reject("frozen dual context census structure mismatch");
  }
  if (frozen.candidate_detection_modes.rows() != window.H.rows() ||
      frozen.candidate_detection_modes.cols() <= 0 ||
      frozen.candidate_protected_modes.rows() != 3 ||
      frozen.candidate_protected_modes.cols() !=
          frozen.candidate_detection_modes.cols() ||
      frozen.candidate_protected_factor.rows() != 3 ||
      frozen.candidate_protected_factor.cols() != window.H.cols() ||
      !frozen.candidate_detection_modes.allFinite() ||
      !frozen.candidate_protected_modes.allFinite() ||
      !frozen.candidate_protected_factor.allFinite() ||
      !frozen.candidate_protected_covariance.allFinite()) {
    return reject("frozen dual context response structure mismatch");
  }
  if (std::any_of(frozen.candidate_mode_active.begin(),
                  frozen.candidate_mode_active.end(),
                  [](std::uint8_t active) { return active > 1; })) {
    return reject("frozen dual context mode activity is invalid");
  }
  std::set<std::uint64_t> block_ids;
  std::set<std::uint64_t> entry_ids;
  for (std::size_t index = 0; index < frozen.dual_blocks.size(); ++index) {
    const auto& block = frozen.dual_blocks[index];
    const auto& entry = base.pl_entries[index];
    if (!block_ids.insert(block.hypothesis.value()).second ||
        !entry_ids.insert(entry.hypothesis.value()).second ||
        block.hypothesis != entry.hypothesis || block.dimension < 0 ||
        block.dimension > 3 || (block.valid && block.dimension == 0) ||
        !block.total_gram.allFinite() || !block.history_gram.allFinite() ||
        !block.score.allFinite() || !block.protected_response.allFinite() ||
        !std::isfinite(block.profile_statistic)) {
      return reject("frozen dual hypothesis block structure mismatch");
    }
  }
  if (frozenDualProofDigest(base, frozen) != frozen.proof_digest) {
    return reject("frozen dual context proof digest mismatch");
  }
  return true;
}

std::uint64_t frozenHypothesisPlEntryIdentity(
    const FrozenHypothesisPlEntry& entry) {
  return frozenEntryProofKey(entry);
}

bool validateFrozenHypothesisPlEntry(
    const FrozenHypothesisPlProofV1& proof, std::string* reason) {
  return detail::validateFrozenHypothesisWithResponse(proof, reason, nullptr);
}

bool detail::validateFrozenHypothesisWithResponse(
    const FrozenHypothesisPlProofV1& proof, std::string* reason,
    GramResponseCertificate* validated_response) {
  detail::NumericalPhaseScope profile(detail::NumericalProfilePhase::FrozenHypothesisValidation);
  NumericalWorkCounters::frozenHypothesisValidation();
  auto reject = [&](const std::string& message) {
    if (reason) *reason = message;
    return false;
  };
  const auto& entry = proof.served_entry;
  if (proof.certified_gram.rows() == 0 ||
      proof.protected_response.rows() != 3 ||
      proof.protected_response.cols() != proof.certified_gram.cols()) {
    return reject("frozen hypothesis proof payload is incomplete");
  }
  GramResponseCertificate rebuilt = proof.raw_detection_factor.rows() > 0
      ? certifyFactorGramAndProtectedResponse(
            proof.raw_detection_factor, proof.certified_gram,
            proof.protected_response, proof.rank_tolerance,
            proof.parent_proof_identity, proof.raw_factor_scale)
      : certifyGramAndProtectedResponse(
            proof.certified_gram, proof.protected_response,
            proof.rank_tolerance, proof.parent_proof_identity,
            proof.raw_factor_scale);
  auto same = [](const auto& left, const auto& right) {
    return left.rows() == right.rows() && left.cols() == right.cols() &&
        (left.array() == right.array()).all();
  };
  double z_sigma_min = std::numeric_limits<double>::infinity();
  double z_condition = std::numeric_limits<double>::infinity();
  int z_rank = 0;
  Eigen::Vector3d z_axis_residual;
  Eigen::Vector3d z_slopes;
  const int z_classification = proof.raw_detection_factor.rows() > 0
      ? classifyDetectionResponse(
            proof.raw_detection_factor, proof.protected_response,
            proof.rank_tolerance, &z_sigma_min, &z_condition, &z_rank,
            &z_axis_residual, &z_slopes, proof.raw_factor_scale)
      : 0;
  const bool expected_projected =
      z_classification == static_cast<int>(GramNullspaceClass::Harmless);
  const bool expected_monitorable = rebuilt.valid &&
      z_classification != static_cast<int>(GramNullspaceClass::Dangerous) &&
      z_classification != static_cast<int>(GramNullspaceClass::Indeterminate);
  const bool expected_valid = expected_monitorable &&
      rebuilt.protected_slopes.allFinite();
  const bool valid = rebuilt.valid && proof.schema_version == 1 &&
      proof.served_entry_identity != 0 &&
      proof.served_entry_identity == frozenEntryProofKey(entry) &&
      rebuilt.proof_identity == proof.proof_identity &&
      static_cast<int>(rebuilt.nullspace_class) == proof.nullspace_class &&
      same(rebuilt.gram.symmetric_matrix, proof.certified_gram) &&
      same(rebuilt.gram.eigenvalues, proof.gram_eigenvalues) &&
      same(rebuilt.gram.eigenvectors, proof.gram_eigenvectors) &&
      same(rebuilt.gram.eigenvalue_errors, proof.gram_eigenvalue_errors) &&
      same(rebuilt.axis_residual, proof.nullspace_axis_residual) &&
      same(rebuilt.protected_slopes, entry.protected_slopes) &&
      entry.valid == expected_valid &&
      entry.gram_spd == (rebuilt.gram.psd && !expected_projected) &&
      entry.monitorability.parameter_dimension == proof.certified_gram.cols() &&
      entry.monitorability.rank == rebuilt.gram.rank &&
      entry.monitorability.sigma_min == rebuilt.gram.sigma_min &&
      entry.monitorability.sigma_max == rebuilt.gram.sigma_max &&
      entry.monitorability.condition_number == rebuilt.gram.condition &&
      entry.monitorability.monitorable == expected_monitorable &&
      same(entry.monitorability.protected_slopes,
           rebuilt.protected_slopes) &&
      entry.z_rank == z_rank &&
      entry.z_smallest_singular_value == z_sigma_min &&
      entry.z_condition == z_condition &&
      entry.z_classification == z_classification &&
      entry.bound_from_projected_path == expected_projected;
  if (!valid) return reject("frozen hypothesis numerical proof mismatch");
  if (validated_response) *validated_response = std::move(rebuilt);
  return true;
}

bool frozenHypothesisPlProof(const FrozenHypothesisPlEntry& entry,
                             FrozenHypothesisPlProofV1* proof) {
  if (!proof) return false;
  auto& registry = frozenProofRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto found = registry.proofs.find(frozenEntryProofKey(entry));
  if (found == registry.proofs.end()) return false;
  *proof = found->second;
  return proof->served_entry.hypothesis == entry.hypothesis &&
      proof->served_entry.protected_slopes == entry.protected_slopes &&
      proof->served_entry.valid == entry.valid;
}

namespace {
template <typename Owner>
bool scopedFrozenHypothesisPlProof(const FrozenHypothesisPlEntry& entry,
                                   const Owner& owner,
                                   FrozenHypothesisPlProofV1* proof) {
  if (!proof) return false;
  const auto payload = detail::AttemptProofArenaAccess::find(
      owner, kArenaFrozenHypothesisProof, frozenEntryProofKey(entry), nullptr);
  if (!payload) return false;
  *proof = *std::static_pointer_cast<const FrozenHypothesisPlProofV1>(payload);
  return proof->served_entry.hypothesis == entry.hypothesis &&
      proof->served_entry.protected_slopes == entry.protected_slopes &&
      proof->served_entry.valid == entry.valid;
}
}  // namespace

bool frozenHypothesisPlProof(const FrozenHypothesisPlEntry& entry,
                             const AttemptProofArena& arena,
                             FrozenHypothesisPlProofV1* proof) {
  return scopedFrozenHypothesisPlProof(entry, arena, proof);
}

bool frozenHypothesisPlProof(const FrozenHypothesisPlEntry& entry,
                             const AttemptProofLease& lease,
                             FrozenHypothesisPlProofV1* proof) {
  return scopedFrozenHypothesisPlProof(entry, lease, proof);
}

namespace detail {
bool readAndValidateScopedFrozenHypothesisProof(
    const FrozenHypothesisPlEntry& entry, const AttemptProofArena& arena,
    FrozenHypothesisPlProofV1* proof, std::string* reason,
    GramResponseCertificate* validated_response) {
  // An optional numerical output requires actual full validation. The old
  // scalar memo carries no numerical certificate and is never upgraded here.
  if (validated_response || !scopedFrozenValidationReuseEnabled()) {
    FrozenHypothesisPlProofV1 local;
    auto* output = proof ? proof : &local;
    return frozenHypothesisPlProof(entry, arena, output) &&
        validateFrozenHypothesisWithResponse(*output, reason, validated_response);
  }
  const auto key = frozenEntryProofKey(entry);
  const auto payload = AttemptProofArenaAccess::find(
      arena, kArenaFrozenHypothesisProof, key, nullptr);
  if (!payload) return false;
  const auto value = std::static_pointer_cast<const FrozenHypothesisPlProofV1>(payload);
  if (proof) *proof = *value;
  // The original reader's external served-entry bindings run on every call.
  if (!(value->served_entry.hypothesis == entry.hypothesis &&
        value->served_entry.protected_slopes == entry.protected_slopes &&
        value->served_entry.valid == entry.valid)) return false;
  const auto memo_payload = AttemptProofArenaAccess::find(
      arena, kArenaFrozenHypothesisValidationMemo, key, nullptr);
  if (memo_payload) {
    const auto immutable =
        std::static_pointer_cast<const ImmutableFrozenHypothesisPayload>(memo_payload);
    if (immutable->arena_generation == arena.generation() &&
        sameImmutablePayloadOwner(payload, memo_payload, &immutable->proof)) {
      bool executed = false;
      const bool valid = immutable->validation.check([&] {
        executed = true;
        return validateFrozenHypothesisPlEntry(*value, reason);
      });
      if (valid && !executed) NumericalWorkCounters::frozenHypothesisValidationReuse();
      return valid;
    }
  }
  // Missing/replaced private owner: never trust a key or caller-imported alias.
  return validateFrozenHypothesisPlEntry(*value, reason);
}
}  // namespace detail

int classifyDetectionResponse(const Eigen::MatrixXd& z_h,
                              const Eigen::MatrixXd& g_h,
                              double rank_tolerance,
                              double* smallest_singular_value,
                              double* condition, int* rank_out,
                              Eigen::Vector3d* axis_residual,
                              Eigen::Vector3d* harmless_slopes) {
  return classifyDetectionResponse(
      z_h, g_h, rank_tolerance, smallest_singular_value, condition, rank_out,
      axis_residual, harmless_slopes,
      std::numeric_limits<double>::quiet_NaN());
}

int classifyDetectionResponse(const Eigen::MatrixXd& z_h,
                              const Eigen::MatrixXd& g_h,
                              double rank_tolerance,
                              double* smallest_singular_value,
                              double* condition, int* rank_out,
                              Eigen::Vector3d* axis_residual,
                              Eigen::Vector3d* harmless_slopes,
                              double raw_factor_scale) {
  detail::NumericalPhaseScope profile(detail::NumericalProfilePhase::DetectionClassification);
  if (axis_residual) *axis_residual = Eigen::Vector3d::Zero();
  if (harmless_slopes) {
    *harmless_slopes = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
  }
  if (z_h.cols() == 0 || z_h.rows() == 0 || g_h.rows() != 3 ||
      g_h.cols() != z_h.cols() || !z_h.allFinite() || !g_h.allFinite()) {
    return 0;
  }
  // Raw-Z exact fallback for the singular-value transition band.  This is
  // available to the evidence path and prevents a nonzero, unbounded fault map
  // from being re-labelled as a structural zero merely because Z'Z squares a
  // tiny singular value.  Gram-only consumers conservatively use the common
  // PSD certificate below.
  Eigen::JacobiSVD<Eigen::MatrixXd> z_reference(z_h);
  const Eigen::VectorXd z_singular = z_reference.singularValues();
  const double z_largest = z_singular.size() ? z_singular(0) : 0.0;
  const double factor_scale = std::isfinite(raw_factor_scale) &&
      raw_factor_scale >= 0.0 ? raw_factor_scale : z_largest;
  const double z_gate = rank_tolerance * factor_scale;
  int z_rank = 0;
  for (Eigen::Index index = 0; index < z_singular.size(); ++index) {
    if (z_singular(index) > z_gate) ++z_rank;
  }
  const Eigen::MatrixXd gram = z_h.transpose() * z_h;
  const GramResponseCertificate certificate =
      std::getenv("UWB_IMU_PL_EXHAUSTIVE_CLASSIFICATION_HASHES")
          ? certifyFactorGramAndProtectedResponse(z_h, gram, g_h,
                                                rank_tolerance, 0, factor_scale)
          : detail::classificationNumericsWithoutProofIdentity(
                z_h, gram, g_h, rank_tolerance, factor_scale);
  if (rank_out) *rank_out = certificate.gram.rank;
  if (smallest_singular_value) {
    *smallest_singular_value = certificate.gram.sigma_min;
  }
  if (condition) *condition = certificate.gram.condition;
  if (axis_residual) *axis_residual = certificate.axis_residual;
  if (harmless_slopes) *harmless_slopes = certificate.protected_slopes;
  if (!certificate.valid || certificate.gram.rank != z_rank) return 4;
  return static_cast<int>(certificate.nullspace_class);
}

std::uint64_t hypothesisSetFingerprint(
    const std::vector<FaultHypothesisV2>& hypotheses) {
  std::uint64_t hash = 1469598103934665603ULL;
  const std::size_t count = hypotheses.size();
  hashBytes(&hash, &count, sizeof(count));
  for (const auto& hypothesis : hypotheses) {
    const auto id = hypothesis.id.value();
    hashBytes(&hash, &id, sizeof(id));
    hashBytes(&hash, &hypothesis.prior_probability_bound,
              sizeof(hypothesis.prior_probability_bound));
    hashBytes(&hash, &hypothesis.p_md_allocation,
              sizeof(hypothesis.p_md_allocation));
    hashBytes(&hash, &hypothesis.hmi_allocation,
              sizeof(hypothesis.hmi_allocation));
    for (const auto mode : hypothesis.modes) {
      const auto value = mode.value();
      hashBytes(&hash, &value, sizeof(value));
    }
  }
  return hash;
}

std::uint64_t faultModeSetFingerprint(
    const std::vector<FaultModeBasis>& modes) {
  std::uint64_t hash = 1469598103934665603ULL;
  const std::size_t count = modes.size();
  hashBytes(&hash, &count, sizeof(count));
  for (const auto& mode : modes) {
    const auto id = mode.id.value();
    const auto anchor = mode.anchor_id.value();
    const auto onset = mode.onset_time.value();
    hashBytes(&hash, &id, sizeof(id));
    hashBytes(&hash, &mode.kind, sizeof(mode.kind));
    hashBytes(&hash, &mode.sensor, sizeof(mode.sensor));
    hashBytes(&hash, mode.physical_source_id.data(),
              mode.physical_source_id.size());
    hashBytes(&hash, &anchor, sizeof(anchor));
    hashBytes(&hash, &mode.axis, sizeof(mode.axis));
    hashBytes(&hash, &mode.onset_epoch, sizeof(mode.onset_epoch));
    hashBytes(&hash, &onset, sizeof(onset));
    hashBytes(&hash, &mode.parameter_dimension,
              sizeof(mode.parameter_dimension));
    hashBytes(&hash, &mode.effective_parameter_dimension,
              sizeof(mode.effective_parameter_dimension));
    hashBytes(&hash, &mode.effective_basis_certified,
              sizeof(mode.effective_basis_certified));
    hashMatrix(&hash, mode.effective_parameter_basis);
    for (const auto& item : mode.raw_group_maps) {
      const auto group = item.first.value();
      hashBytes(&hash, &group, sizeof(group));
      hashMatrix(&hash, item.second);
    }
    for (const auto group : mode.affected_groups) {
      const auto value = group.value();
      hashBytes(&hash, &value, sizeof(value));
    }
    for (const auto measurement : mode.affected_measurements) {
      const auto value = measurement.value();
      hashBytes(&hash, &value, sizeof(value));
    }
  }
  return hash;
}

std::vector<HypothesisId> completePlausibleHypotheses(
    const std::vector<FaultHypothesisV2>& hypotheses,
    const std::vector<FaultModeEvidence>& evidence) {
  std::set<std::uint64_t> plausible;
  for (const auto& item : evidence) {
    if (item.plausible) plausible.insert(item.hypothesis.value());
  }
  std::vector<HypothesisId> complete;
  for (const auto& hypothesis : hypotheses) {
    if (plausible.count(hypothesis.id.value())) complete.push_back(hypothesis.id);
  }
  return complete;
}

namespace {

std::vector<FaultModeEvidence> evaluateAllImpl(
    const HypothesisEvaluationConfig& config_,
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared,
    std::shared_ptr<const FrozenHypothesisDualNumerics>* shared_dual,
    CandidateWorkerPool* worker_pool,
    const FrozenWindowAdmission* admission,
    AttemptProofArena* proof_arena) {
  if (!hypotheses) throw std::invalid_argument("hypotheses must not be null");
  if (shared) shared->reset();
  if (shared_dual) shared_dual->reset();
  const bool numerical_proof_valid = window.numerics &&
      (admission
           ? validateFrozenWindowNumericalProof(
                 *admission, *window.numerics)
           : window.numerics->content_fingerprint ==
                 integrityWindowFingerprint(window) &&
             validateFrozenWindowNumericalProof(window, *window.numerics));
  if (!window.model_valid || !window.numerics || !window.numerics->valid ||
      !window.numerics->information_factorization ||
      !numerical_proof_valid) return {};
  if (window.numerics->numerical_contract_fingerprint !=
      numericalContractFingerprint(config_.rank_tolerance,
                                   config_.max_condition_number)) {
    NumericalWorkCounters::numericalContractMismatch();
    return {};
  }
  // The admission above is the sole full-content validation for this call.
  // `numerics` and its handle are immutable, so every hypothesis can consume
  // the typed proof identity without rescanning H/z/metadata.  The environment
  // switch is an equivalence/performance ablation only; it restores the old
  // repeated proof construction without changing any result.
  const bool disable_frozen_identity =
      std::getenv("UWB_IMU_PL_DISABLE_FROZEN_IDENTITY_HANDLE") != nullptr;
  const std::uint64_t numerical_proof_identity =
      disable_frozen_identity || !admission
      ? 0 : admission->owner->numerical_identity.proof_identity;
  if (modes.empty()) {
    std::vector<FaultModeEvidence> results;
    results.reserve(hypotheses->size());
    WorkDelta work;
    for (auto& hypothesis : *hypotheses) {
      FaultModeEvidence evidence;
      evidence.hypothesis = hypothesis.id;
      evidence.all_in_statistic = window.numerics->statistic;
      Eigen::MatrixXd gram;
      Eigen::VectorXd score;
      if (hypothesis.A.rows() == window.H.rows() && hypothesis.A.cols() > 0 &&
          hypothesis.A.allFinite()) {
        const Eigen::MatrixXd cross = window.H.transpose() * hypothesis.A;
        const Eigen::MatrixXd covariance_cross = solveFrozenInformation(
            window.square_root.get(), *window.numerics,
            window.base_information, cross);
        gram = hypothesis.A.transpose() * hypothesis.A -
            cross.transpose() * covariance_cross;
        score = hypothesis.A.transpose() * window.numerics->parity;
      }
      Eigen::MatrixXd raw_detection_factor;
      if (window.square_root && window.square_root->usable() &&
          hypothesis.A.rows() == window.H.rows() &&
          hypothesis.A.cols() > 0) {
        Eigen::MatrixXd y;
        window.square_root->faultResponse(hypothesis.A, &y,
                                          &raw_detection_factor);
      }
      analyzeDynamic(gram,
                     raw_detection_factor.rows() > 0
                         ? &raw_detection_factor : nullptr,
                     score, nullptr, false,
                     admittedProofIdentity(window, numerical_proof_identity),
                     hypothesis.A.norm(),
                     hypothesis.A.cols(),
                     window.numerics->statistic, squared_detector_threshold,
                     config_, &hypothesis, &evidence, nullptr, &work,
                     proof_arena);
      results.push_back(std::move(evidence));
    }
    publish(work);
    return results;
  }
  if (config_.enable_shared_context && config_.enable_low_dim_batch) {
    return evaluateContiguous(window, modes, hypotheses,
                              squared_detector_threshold, config_,
                              numerical_proof_identity, shared,
                              shared_dual, worker_pool, admission, proof_arena);
  }
  return evaluateMapped(window, modes, hypotheses, squared_detector_threshold,
                        config_, numerical_proof_identity, shared, proof_arena);
}

}  // namespace

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared,
    CandidateWorkerPool* worker_pool) const {
  // Preserve the checkpoint contract for every legacy value-object caller.
  return evaluateAllImpl(config_, window, modes, hypotheses,
                         squared_detector_threshold, shared, nullptr,
                         worker_pool,
                         nullptr, nullptr);
}

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const FrozenWindowAdmission& admission,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared,
    CandidateWorkerPool* worker_pool) const {
  if (!admission) {
    if (shared) shared->reset();
    return {};
  }
  return evaluateAllImpl(config_, admission.window(), modes, hypotheses,
                         squared_detector_threshold, shared, nullptr,
                         worker_pool,
                         &admission, nullptr);
}

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const FrozenWindowAdmission& admission,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared,
    std::shared_ptr<const FrozenHypothesisDualNumerics>* shared_dual,
    CandidateWorkerPool* worker_pool) const {
  if (!admission) {
    if (shared) shared->reset();
    if (shared_dual) shared_dual->reset();
    return {};
  }
  return evaluateAllImpl(config_, admission.window(), modes, hypotheses,
                         squared_detector_threshold, shared, shared_dual,
                         worker_pool, &admission, nullptr);
}

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const FrozenWindowAdmission& admission,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared,
    std::shared_ptr<const FrozenHypothesisDualNumerics>* shared_dual,
    CandidateWorkerPool* worker_pool,
    AttemptProofArena* proof_arena) const {
  if (!admission || !proof_arena || proof_arena->closed()) {
    if (shared) shared->reset();
    if (shared_dual) shared_dual->reset();
    return {};
  }
  return evaluateAllImpl(config_, admission.window(), modes, hypotheses,
                         squared_detector_threshold, shared, shared_dual,
                         worker_pool, &admission, proof_arena);
}

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const LinearizedIntegrityWindow& window,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold) const {
  return evaluateAll(window, {}, hypotheses, squared_detector_threshold,
                     nullptr, nullptr);
}

}  // namespace uwb_imu_pl
