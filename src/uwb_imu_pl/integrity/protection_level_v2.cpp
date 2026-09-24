#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"
#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace uwb_imu_pl {

namespace {

void hashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= bytes[index];
    *hash *= 1099511628211ULL;
  }
}

template <class T>
void hashScalar(std::uint64_t* hash, const T& value) {
  hashBytes(hash, &value, sizeof(value));
}

template <class Derived>
void hashMatrix(std::uint64_t* hash,
                const Eigen::MatrixBase<Derived>& matrix) {
  const std::int64_t rows = matrix.rows();
  const std::int64_t columns = matrix.cols();
  hashScalar(hash, rows);
  hashScalar(hash, columns);
  for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      const double value = matrix(row, column) == 0.0
          ? 0.0 : static_cast<double>(matrix(row, column));
      hashScalar(hash, value);
    }
  }
}

std::uint64_t candidateProofIdentity(const CandidateEvaluation& candidate) {
  return candidate.window_view
      ? candidateNumericalProofIdentity(*candidate.window_view, candidate)
      : 0;
}

FrozenHypothesisPlProofV1 protectionHypothesisProof(
    HypothesisId hypothesis, const GramResponseCertificate& certificate,
    const Eigen::MatrixXd& raw_factor, double raw_factor_scale,
    double rank_tolerance) {
  FrozenHypothesisPlProofV1 proof;
  proof.served_entry.hypothesis = hypothesis;
  proof.certified_gram = certificate.gram.symmetric_matrix;
  proof.protected_response = Eigen::MatrixXd();
  // G is reconstructed from slopes only in neither producer nor consumer;
  // callers set the actual protected response immediately below.
  proof.gram_eigenvalues = certificate.gram.eigenvalues;
  proof.gram_eigenvectors = certificate.gram.eigenvectors;
  proof.gram_eigenvalue_errors = certificate.gram.eigenvalue_errors;
  proof.nullspace_axis_residual = certificate.axis_residual;
  proof.nullspace_class = static_cast<int>(certificate.nullspace_class);
  proof.proof_identity = certificate.proof_identity;
  proof.raw_detection_factor = raw_factor;
  proof.raw_factor_scale = raw_factor_scale;
  proof.rank_tolerance = rank_tolerance;
  proof.parent_proof_identity = 0;  // caller supplies candidate proof identity
  proof.served_entry.protected_slopes = certificate.protected_slopes;
  proof.served_entry.monitorability.protected_slopes =
      certificate.protected_slopes;
  proof.served_entry.monitorability.parameter_dimension = raw_factor.cols();
  proof.served_entry.monitorability.rank = certificate.gram.rank;
  proof.served_entry.monitorability.sigma_min = certificate.gram.sigma_min;
  proof.served_entry.monitorability.sigma_max = certificate.gram.sigma_max;
  proof.served_entry.monitorability.condition_number =
      certificate.gram.condition;
  proof.served_entry.monitorability.monitorable = certificate.valid;
  proof.served_entry.gram_spd = certificate.gram.psd;
  proof.served_entry.z_rank = certificate.gram.rank;
  proof.served_entry.z_smallest_singular_value = certificate.gram.sigma_min;
  proof.served_entry.z_condition = certificate.gram.condition;
  proof.served_entry.z_classification =
      static_cast<int>(certificate.nullspace_class);
  proof.served_entry.bound_from_projected_path =
      certificate.nullspace_class == GramNullspaceClass::Harmless;
  if (proof.served_entry.bound_from_projected_path) {
    proof.served_entry.gram_spd = false;
  }
  proof.served_entry.valid = certificate.valid &&
      certificate.nullspace_class != GramNullspaceClass::Dangerous &&
      certificate.nullspace_class != GramNullspaceClass::Indeterminate &&
      certificate.protected_slopes.allFinite();
  proof.served_entry_identity =
      frozenHypothesisPlEntryIdentity(proof.served_entry);
  return proof;
}

std::uint64_t resultRegistryKey(const ProtectionLevelV2Result& result) {
  std::uint64_t key = 1469598103934665603ULL;
  hashMatrix(&key, result.pl_xyz_m);
  hashMatrix(&key, result.nominal_component_m);
  hashMatrix(&key, result.fault_component_m);
  hashMatrix(&key, result.bridge_component_m);
  hashScalar(&key, result.hpl_m);
  hashScalar(&key, result.vpl_m);
  hashScalar(&key, result.allocated_outcome_risk);
  hashScalar(&key, result.risk_budget_valid);
  hashScalar(&key, result.model_valid);
  hashScalar(&key, result.formal_eligible);
  hashScalar(&key, result.availability);
  hashBytes(&key, result.detector_certificate_id.data(),
            result.detector_certificate_id.size());
  hashBytes(&key, result.reason.data(), result.reason.size());
  return key;
}

struct ProtectionProofRegistry {
  std::mutex mutex;
  std::map<std::uint64_t, std::vector<ProtectionLevelV2ProofV1>> by_result;
  std::map<std::uint64_t, ProtectionLevelV2ProofV1> by_identity;
  std::map<std::string, ProtectionLevelPublicationPacketV1>
      publication_packets;
};

ProtectionProofRegistry& protectionProofRegistry() {
  static ProtectionProofRegistry registry;
  return registry;
}

std::string hexDigest(std::uint64_t value) {
  std::ostringstream out;
  out << std::hex << value;
  return out.str();
}

std::uint64_t protectionResultProofIdentity(
    const ProtectionLevelV2ProofV1& sidecar) {
  const auto& hypotheses = sidecar.hypotheses;
  const auto& result = sidecar.served_result;
  std::uint64_t proof = 1469598103934665603ULL;
  hashScalar(&proof, sidecar.schema_version);
  hashScalar(&proof, sidecar.candidate_proof_identity);
  hashScalar(&proof, sidecar.detector_candidate_numerical_identity);
  hashScalar(&proof, sidecar.detector_proof_identity);
  hashScalar(&proof, sidecar.pooled_only);
  hashScalar(&proof, sidecar.detector_passed);
  hashScalar(&proof, sidecar.risk.p_hmi_total);
  hashScalar(&proof, sidecar.risk.nominal_axis_tail);
  hashScalar(&proof, sidecar.risk.p_nm);
  hashScalar(&proof, sidecar.risk.p_bridge_escape);
  hashScalar(&proof, sidecar.risk.p_history_contamination);
  hashScalar(&proof, sidecar.risk.p_model_escape);
  hashScalar(&proof, sidecar.risk.horizontal_alert_limit_m);
  hashScalar(&proof, sidecar.risk.vertical_alert_limit_m);
  hashMatrix(&proof, sidecar.protected_covariance_factor);
  hashMatrix(&proof, sidecar.protected_covariance);
  hashScalar(&proof, sidecar.covariance_rank_tolerance);
  hashScalar(&proof, sidecar.nominal_tail);
  hashScalar(&proof, sidecar.nominal_multiplier);
  hashMatrix(&proof, sidecar.nominal_sigma);
  hashMatrix(&proof, sidecar.fixed_bridge_margin);
  hashMatrix(&proof, result.pl_xyz_m);
  hashMatrix(&proof, result.nominal_component_m);
  hashMatrix(&proof, result.fault_component_m);
  hashMatrix(&proof, result.bridge_component_m);
  hashScalar(&proof, result.hpl_m);
  hashScalar(&proof, result.vpl_m);
  hashScalar(&proof, result.allocated_outcome_risk);
  hashScalar(&proof, result.risk_budget_valid);
  hashScalar(&proof, result.model_valid);
  hashScalar(&proof, result.formal_eligible);
  hashScalar(&proof, result.availability);
  hashScalar(&proof, result.hypothesis_tail_used);
  hashScalar(&proof, result.axis_tail_used);
  hashScalar(&proof, result.fault_multiplier_used);
  hashScalar(&proof, result.noncentrality_used);
  hashBytes(&proof, result.detector_certificate_id.data(),
            result.detector_certificate_id.size());
  hashScalar(&proof, hypothesisSetFingerprint(hypotheses));
  for (const auto& hypothesis : hypotheses) {
    hashScalar(&proof, hypothesis.id.value());
    hashMatrix(&proof, hypothesis.A);
    hashScalar(&proof, hypothesis.prior_probability_bound);
    hashScalar(&proof, hypothesis.p_md_allocation);
    hashScalar(&proof, hypothesis.hmi_allocation);
    hashScalar(&proof, hypothesis.monitorability.parameter_dimension);
    hashScalar(&proof, hypothesis.monitorability.physical_parameter_dimension);
    hashScalar(&proof, hypothesis.monitorability.rank);
    hashScalar(&proof, hypothesis.monitorability.monitorable);
    hashScalar(&proof, hypothesis.monitorability.sigma_min);
    hashScalar(&proof, hypothesis.monitorability.sigma_max);
    hashScalar(&proof, hypothesis.monitorability.condition_number);
    hashMatrix(&proof, hypothesis.monitorability.protected_slopes);
  }
  for (const auto& item : sidecar.hypothesis_proofs) {
    hashScalar(&proof, item.served_entry.hypothesis.value());
    hashMatrix(&proof, item.certified_gram);
    hashMatrix(&proof, item.protected_response);
    hashMatrix(&proof, item.gram_eigenvalues);
    hashMatrix(&proof, item.gram_eigenvectors);
    hashMatrix(&proof, item.gram_eigenvalue_errors);
    hashMatrix(&proof, item.nullspace_axis_residual);
    hashMatrix(&proof, item.served_entry.protected_slopes);
    hashMatrix(&proof, item.raw_detection_factor);
    hashScalar(&proof, item.raw_factor_scale);
    hashScalar(&proof, item.rank_tolerance);
    hashScalar(&proof, item.parent_proof_identity);
    hashScalar(&proof, item.schema_version);
    hashScalar(&proof, item.nullspace_class);
    hashScalar(&proof, item.proof_identity);
    hashScalar(&proof, item.served_entry_identity);
  }
  for (const auto& item : sidecar.dual_channel_proofs) {
    hashScalar(&proof, item.proof_identity);
    hashMatrix(&proof, item.request.protected_response);
    hashMatrix(&proof, item.w_matrix);
    hashMatrix(&proof, item.w_eigenvalues);
    hashMatrix(&proof, item.w_eigenvectors);
    hashMatrix(&proof, item.w_eigenvalue_errors);
    hashMatrix(&proof, item.served_result.axis_bound_m);
  }
  for (const auto& item : sidecar.bridge_proofs) {
    hashMatrix(&proof, item.projected_gain);
    hashMatrix(&proof, item.deterministic_bound);
    hashMatrix(&proof, item.served_component);
  }
  for (const auto& item : sidecar.component_proofs) {
    hashScalar(&proof, item.hypothesis.value());
    hashScalar(&proof, item.detector_dof);
    hashScalar(&proof, item.detector_threshold);
    hashScalar(&proof, item.hypothesis_tail);
    hashScalar(&proof, item.axis_tail);
    hashScalar(&proof, item.multiplier);
    hashScalar(&proof, item.noncentrality);
    hashScalar(&proof, item.dual_channel);
    hashMatrix(&proof, item.served_component);
  }
  return proof;
}

void bindProtectionResultProof(
    const CandidateEvaluation& candidate, const DetectorResultV2& detector,
    const std::vector<FaultHypothesisV2>& hypotheses,
    ProtectionLevelV2Result* result, ProtectionLevelV2ProofV1* sidecar) {
  if (!result || !sidecar || !result->model_valid ||
      !result->pl_xyz_m.allFinite()) return;
  sidecar->candidate_proof_identity =
      candidateProofIdentity(candidate);
  sidecar->detector_proof_identity = detector.detector_contract_digest;
  sidecar->detector_candidate_numerical_identity =
      detector.candidate_numerical_identity;
  sidecar->pooled_only =
      detector.contract_mode == DetectorContractMode::LegacyPooledOffline ||
      detector.channel_history_dof == 0;
  sidecar->detector_passed = detector.passed;
  sidecar->hypotheses = hypotheses;
  sidecar->served_result = *result;
  sidecar->proof_identity =
      protectionResultProofIdentity(*sidecar);
  {
    auto& registry = protectionProofRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    registry.by_result[resultRegistryKey(*result)].push_back(*sidecar);
    registry.by_identity[sidecar->proof_identity] = *sidecar;
  }
  std::string proof_reason;
  if (!validateProtectionLevelV2Proof(candidate, detector, hypotheses,
                                      *result, &proof_reason)) {
    result->model_valid = false;
    result->availability = Availability::Unavailable;
    result->reason = proof_reason;
  }
}

// B3 (§5.9): the bound used by every PL path.
//
//   L_{h,d} = s_{h,d} * sqrt(Lambda_h) + k_{h,d} * sigma_d
//   alpha_h   = min(0.5, allocation_h / pi_h)          (charged tail)
//   alpha_{h,d} = alpha_h / axis_count                 (equal split)
//   k_{h,d}   = Phi^-1(1 - alpha_{h,d} / 2)
//   Lambda_h  solves F_{chi2_nu(Lambda_h)}(tau) <= beta_h, conservative side
//
// The per-axis split is what makes the charged risk alpha_h sufficient for the
// union bound over the three axes: using alpha_h on every axis (the pre-B3
// form) understates the tail that the ledger pays for.  Invalid inputs are
// rejected; the caller turns that into an unavailable result instead of a
// clamped bound.
struct FaultAxisBounds {
  double lambda = 0.0;
  double k = 0.0;
  double hypothesis_tail = 0.0;
  double axis_tail = 0.0;
  bool valid = false;
  std::string reason;
};

struct RawFactorCertificate {
  Eigen::MatrixXd u;
  Eigen::MatrixXd v;
  Eigen::VectorXd inverse_singular;
  Eigen::MatrixXd protected_factor;
  Eigen::Matrix3d protected_covariance = Eigen::Matrix3d::Zero();
  bool valid = false;
  std::string reason;

  bool faultProducts(const Eigen::MatrixXd& fault_map,
                     Eigen::MatrixXd* gram,
                     Eigen::MatrixXd* protected_response,
                     Eigen::MatrixXd* residual_factor = nullptr) const {
    if (!valid || !gram || !protected_response ||
        fault_map.rows() != u.rows() || !fault_map.allFinite()) return false;
    const Eigen::MatrixXd coordinates = u.transpose() * fault_map;
    const Eigen::MatrixXd residual = fault_map - u * coordinates;
    *gram = residual.transpose() * residual;
    *protected_response = protected_factor * coordinates;
    if (residual_factor) *residual_factor = residual;
    return gram->allFinite() && protected_response->allFinite();
  }
};

RawFactorCertificate rawFactorCertificate(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate, double rank_tolerance) {
  RawFactorCertificate out;
  Eigen::MatrixXd h;
  const Eigen::MatrixXd& retained = candidate.retainedJacobian();
  if (retained.rows() == candidate.rows &&
      retained.cols() == candidate.state_increment.rows()) {
    h = retained;
  } else {
    std::vector<const LinearizedFactorBlock*> blocks;
    auto removed = [&](FactorGroupId id) {
      return std::find(candidate.action.groups_to_remove.begin(),
                       candidate.action.groups_to_remove.end(), id) !=
          candidate.action.groups_to_remove.end();
    };
    for (const auto& block : window.blocks) {
      if (!removed(block.group_id)) blocks.push_back(&block);
    }
    for (const auto& block : candidate.action.added_blocks) {
      blocks.push_back(&block);
    }
    Eigen::Index rows = 0;
    for (const auto* block : blocks) rows += block->jacobian_whitened.rows();
    if (rows != candidate.rows) {
      out.reason = "raw candidate Jacobian row count mismatch";
      return out;
    }
    h.resize(rows, candidate.state_increment.rows());
    Eigen::Index offset = 0;
    for (const auto* block : blocks) {
      h.middleRows(offset, block->jacobian_whitened.rows()) =
          block->jacobian_whitened;
      offset += block->jacobian_whitened.rows();
    }
  }
  if (h.rows() <= 0 || h.cols() <= 0 || !h.allFinite() ||
      window.protected_state_map.cols() != h.cols()) {
    out.reason = "raw candidate Jacobian dimensions invalid";
    return out;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular = svd.singularValues();
  if (singular.size() != h.cols() || !singular.allFinite()) {
    out.reason = "direct raw-H SVD failed";
    return out;
  }
  const double largest = singular.size() ? singular(0) : 0.0;
  const double gate = rank_tolerance * largest;
  const int rank = static_cast<int>((singular.array() > gate).count());
  if (rank != h.cols() || !(singular(singular.size() - 1) > 0.0)) {
    out.reason = "direct raw-H SVD is rank deficient";
    return out;
  }
  out.u = svd.matrixU();
  out.v = svd.matrixV();
  out.inverse_singular = singular.cwiseInverse();
  out.protected_factor = window.protected_state_map * out.v *
      out.inverse_singular.asDiagonal();
  out.protected_covariance =
      out.protected_factor * out.protected_factor.transpose();
  out.valid = out.protected_covariance.allFinite();
  if (!out.valid) out.reason = "direct raw-H covariance is non-finite";
  return out;
}

bool detectorContractMatchesCandidate(const LinearizedIntegrityWindow& window,
                                      const CandidateEvaluation& candidate,
                                      const DetectorResultV2& detector,
                                      std::string* reason) {
  if (detector.contract_mode == DetectorContractMode::LegacyPooledOffline) {
    return true;
  }
  std::string candidate_reason;
  if (!validateCandidateDetectorCertificate(window, candidate,
                                            &candidate_reason)) {
    if (reason) *reason = candidate_reason;
    return false;
  }
  const auto& certificate = candidate.detector_certificate;
  const bool p_fa_valid = std::isfinite(detector.p_fa_per_test) &&
      detector.p_fa_per_test > 0.0 && detector.p_fa_per_test < 1.0;
  const int expected_channel_count = certificate.history_rows > 0 ? 2 : 1;
  const std::uint64_t expected_horizon =
      std::max<std::uint64_t>(1, detector.continuity_horizon_tests);
  const double expected_current_threshold = p_fa_valid
      ? StatisticalBoundsCache::chiSquaredThreshold(
            certificate.current_dof, detector.p_fa_per_test)
      : std::numeric_limits<double>::infinity();
  const double expected_history_threshold = certificate.history_rows > 0 &&
      p_fa_valid
      ? StatisticalBoundsCache::chiSquaredThreshold(
            certificate.history_dof, detector.p_fa_per_test)
      : 0.0;
  const double expected_pooled_threshold = p_fa_valid
      ? StatisticalBoundsCache::chiSquaredThreshold(
            certificate.pooled_dof, detector.p_fa_per_test)
      : std::numeric_limits<double>::infinity();
  const double expected_operation_bound = p_fa_valid
      ? std::min(1.0, detector.p_fa_per_test *
            static_cast<double>(expected_horizon) *
            static_cast<double>(expected_channel_count))
      : std::numeric_limits<double>::infinity();
  const bool expected_current_accepted = p_fa_valid &&
      certificate.current_statistic <= expected_current_threshold;
  const bool expected_history_accepted = certificate.history_rows == 0 ||
      (p_fa_valid &&
       certificate.history_statistic <= expected_history_threshold);
  const bool valid = certificate.valid &&
      certificate.history_constant_present && detector.channel_split_valid &&
      p_fa_valid && detector.window_id == window.id &&
      detector.accepted_event_id == certificate.accepted_event_id &&
      detector.accepted_event_id == "dual_channel_intersection_v6" &&
      detector.candidate_numerical_identity == certificate.numerical_identity &&
      detector.candidate_numerical_identity != 0 &&
      detector.candidate_certificate_digest == certificate.certificate_digest &&
      detector.detector_contract_digest != 0 &&
      detector.detector_contract_digest == detectorContractDigest(detector) &&
      detector.rows == candidate.rows && detector.rank == candidate.rank &&
      detector.dof == candidate.dof &&
      detector.squared_parity_statistic ==
          certificate.current_statistic + certificate.history_statistic &&
      detector.squared_threshold == expected_pooled_threshold &&
      std::isfinite(detector.history_constant) &&
      detector.history_constant == certificate.history_constant &&
      detector.channel_current_statistic == certificate.current_statistic &&
      detector.channel_history_statistic == certificate.history_statistic &&
      detector.channel_current_dof == certificate.current_dof &&
      detector.channel_history_dof == certificate.history_dof &&
      detector.channel_current_threshold == expected_current_threshold &&
      detector.channel_history_threshold == expected_history_threshold &&
      detector.continuity_horizon_tests == expected_horizon &&
      detector.active_channel_count == expected_channel_count &&
      detector.operation_p_fa_upper_bound == expected_operation_bound &&
      detector.channel_current_accepted == expected_current_accepted &&
      detector.channel_history_accepted == expected_history_accepted &&
      detector.channel_current_accepted && detector.channel_history_accepted &&
      detector.joint_accepted && detector.passed;
  if (!valid && reason) {
    *reason = "active-v6 detector certificate/event/numerical identity mismatch";
  }
  return valid;
}

Eigen::MatrixXd historyGramFromCertificate(
    const CandidateEvaluation& candidate, const Eigen::MatrixXd& fault_map) {
  Eigen::MatrixXd history_gram = Eigen::MatrixXd::Zero(
      fault_map.cols(), fault_map.cols());
  const auto& roles = candidate.detector_certificate.row_roles;
  if (!candidate.detector_certificate.valid ||
      roles.size() != static_cast<std::size_t>(fault_map.rows())) {
    return Eigen::MatrixXd();
  }
  for (Eigen::Index row = 0; row < fault_map.rows(); ++row) {
    if (roles[static_cast<std::size_t>(row)] ==
        CandidateDetectorRowRole::HistoryDetectorOnly) {
      history_gram.noalias() += fault_map.row(row).transpose() *
                                fault_map.row(row);
    }
  }
  return history_gram;
}

FaultAxisBounds dualChannelAxisBounds(
    const DetectorResultV2& detector,
    const FaultHypothesisV2& hypothesis,
    const Eigen::MatrixXd& total_gram,
    const Eigen::MatrixXd& history_gram,
    const Eigen::MatrixXd& protected_response,
    const Eigen::Vector3d& sigma,
    double nominal_tail, double rank_tolerance,
    std::uint64_t numerical_proof_identity,
    Eigen::Vector3d* component,
    std::string* certificate_id,
    DualChannelNumericalProofV1* bound_proof) {
  FaultAxisBounds out;
  double tail = hypothesis.hmi_allocation > 0.0
      ? hypothesis.hmi_allocation /
            std::max(hypothesis.prior_probability_bound, 1e-15)
      : nominal_tail;
  // The production path is required to carry an explicit dual-channel
  // certificate.  The older pooled numerical API remains available only when
  // an offline caller explicitly selects LegacyPooledOffline; an incomplete
  // active-v6 detector always fails closed.
  const bool legacy_single_channel =
      detector.contract_mode == DetectorContractMode::LegacyPooledOffline;
  if (!legacy_single_channel &&
      (detector.accepted_event_id != "dual_channel_intersection_v6" ||
       !detector.channel_split_valid ||
       detector.candidate_numerical_identity == 0 ||
       !std::isfinite(detector.history_constant))) {
    out.reason = "active-v6 dual-channel detector certificate is incomplete";
    return out;
  }
  if (!std::isfinite(tail) || tail <= 0.0) {
    out.reason = "hypothesis tail probability is not positive";
    return out;
  }
  tail = std::min(0.5, tail);
  out.hypothesis_tail = tail;
  out.axis_tail = tail / 3.0;
  bool multiplier_valid = false;
  out.k = StatisticalBoundsCache::normalTwoSidedMultiplierVerified(
      out.axis_tail, &multiplier_valid);
  if (!multiplier_valid) {
    out.reason = "per-axis tail multiplier could not be evaluated";
    return out;
  }
  DualChannelBoundRequest request;
  request.protected_response = protected_response;
  request.p_md = hypothesis.p_md_allocation;
  request.k_axis.setConstant(out.k);
  request.position_rho_m.setZero();
  request.position_rho_validated = true;
  request.position_rho_source = "none";

  ChannelBoundInput current;
  current.detector_id = "joint_window_state_supported";
  current.dof = legacy_single_channel ? detector.dof
                                      : detector.channel_current_dof;
  current.threshold = legacy_single_channel ? detector.squared_threshold
                                            : detector.channel_current_threshold;
  current.actual_threshold = current.threshold;
  current.weight = !legacy_single_channel && detector.channel_history_dof > 0
                       ? 0.5
                       : 1.0;
  const Eigen::MatrixXd current_gram = legacy_single_channel
      ? total_gram : (total_gram - history_gram);
  current.gram = 0.5 * (current_gram + current_gram.transpose());
  current.rho_validated = true;
  current.rho_source = "none";
  request.channels.push_back(std::move(current));
  if (!legacy_single_channel && detector.channel_history_dof > 0 &&
      history_gram.norm() > 0.0) {
    ChannelBoundInput history;
    history.detector_id = "history_eliminated_detector_only";
    history.dof = detector.channel_history_dof;
    history.threshold = detector.channel_history_threshold;
    history.actual_threshold = detector.channel_history_threshold;
    history.weight = 0.5;
    history.gram = 0.5 * (history_gram + history_gram.transpose());
    history.rho_validated = true;
    history.rho_source = "none";
    request.channels.push_back(std::move(history));
  }
  const DualChannelBoundResult bound = computeDualChannelBoundCertified(
      request, rank_tolerance, numerical_proof_identity, bound_proof);
  if (!bound.valid) {
    out.reason = "dual-channel PL bound: " + bound.reason;
    return out;
  }
  *component = bound.axis_bound_m + out.k * sigma +
               bound.position_rho_m;
  *certificate_id = bound.certificate_id;
  for (const double value : bound.channel_lambda) {
    out.lambda = std::max(out.lambda, value);
  }
  out.valid = component->allFinite();
  if (!out.valid) out.reason = "dual-channel PL component is non-finite";
  return out;
}

FaultAxisBounds faultAxisBounds(const DetectorResultV2& detector,
                                const FaultHypothesisV2& hypothesis,
                                double nominal_tail) {
  FaultAxisBounds out;
  double tail = hypothesis.hmi_allocation > 0.0
      ? hypothesis.hmi_allocation /
            std::max(hypothesis.prior_probability_bound, 1e-15)
      : nominal_tail;
  if (!std::isfinite(tail) || tail <= 0.0) {
    out.reason = "hypothesis tail probability is not positive";
    return out;
  }
  tail = std::min(0.5, tail);
  out.hypothesis_tail = tail;
  out.axis_tail = tail / 3.0;
  bool multiplier_valid = false;
  out.k = StatisticalBoundsCache::normalTwoSidedMultiplierVerified(
      out.axis_tail, &multiplier_valid);
  if (!multiplier_valid) {
    out.reason = "per-axis tail multiplier could not be evaluated";
    return out;
  }
  const StatisticalBoundKey key;
  const NoncentralityBoundaryResult boundary =
      StatisticalBoundsCache::noncentralityBoundaryVerified(
          detector.dof, detector.squared_threshold,
          hypothesis.p_md_allocation, key);
  if (!boundary.valid) {
    out.reason = "detection boundary noncentrality: " + boundary.reason;
    return out;
  }
  out.lambda = std::sqrt(boundary.value);
  out.valid = true;
  return out;
}

}  // namespace

bool validateProtectionLevelV2Proof(
    const CandidateEvaluation& candidate, const DetectorResultV2& detector,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const ProtectionLevelV2Result& result, std::string* reason) {
  ProtectionLevelV2ProofV1 sidecar;
  {
    auto& registry = protectionProofRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    const auto found = registry.by_result.find(resultRegistryKey(result));
    if (found != registry.by_result.end()) {
      const auto match = std::find_if(
          found->second.rbegin(), found->second.rend(),
          [&](const ProtectionLevelV2ProofV1& proof) {
            return proof.candidate_proof_identity ==
                candidateProofIdentity(candidate);
          });
      if (match != found->second.rend()) sidecar = *match;
    }
  }
  if (sidecar.proof_identity == 0) {
    if (reason) *reason = "protection-level proof sidecar is unavailable";
    return false;
  }
  std::string payload_reason;
  const bool payload_valid =
      validateProtectionLevelV2ProofPayload(sidecar, &payload_reason);
  const bool pooled_only =
      detector.contract_mode == DetectorContractMode::LegacyPooledOffline ||
      detector.channel_history_dof == 0;
  const bool dual_proofs_valid =
      sidecar.dual_channel_proofs.size() == hypotheses.size() ||
      (pooled_only && sidecar.dual_channel_proofs.empty());
  const bool valid = result.model_valid && sidecar.schema_version == 1 &&
      sidecar.candidate_proof_identity ==
          candidateProofIdentity(candidate) &&
      sidecar.detector_proof_identity == detector.detector_contract_digest &&
      sidecar.detector_candidate_numerical_identity ==
          detector.candidate_numerical_identity &&
      hypothesisSetFingerprint(sidecar.hypotheses) ==
          hypothesisSetFingerprint(hypotheses) &&
      sidecar.hypothesis_proofs.size() == hypotheses.size() &&
      payload_valid &&
      dual_proofs_valid &&
      sidecar.proof_identity != 0 &&
      sidecar.proof_identity ==
          protectionResultProofIdentity(sidecar);
  if (!valid && reason) {
    if (!payload_valid) *reason = payload_reason;
    else if (!dual_proofs_valid) *reason = "protection-level dual proof mismatch";
    else if (sidecar.hypothesis_proofs.size() != hypotheses.size()) {
      *reason = "protection-level hypothesis proof count mismatch";
    } else if (sidecar.candidate_proof_identity !=
               candidateProofIdentity(candidate)) {
      *reason = "protection-level candidate proof mismatch";
    } else if (sidecar.detector_proof_identity !=
               detector.detector_contract_digest) {
      *reason = "protection-level detector proof mismatch";
    } else {
      *reason = "protection-level result proof digest mismatch";
    }
  }
  return valid;
}

bool validateProtectionLevelV2ProofPayload(
    const ProtectionLevelV2ProofV1& proof, std::string* reason) {
  const auto same = [](const auto& left, const auto& right) {
    return left.rows() == right.rows() && left.cols() == right.cols() &&
        (left.array() == right.array()).all();
  };
  const bool hypotheses_valid = proof.hypothesis_proofs.size() ==
          proof.hypotheses.size() &&
      std::all_of(proof.hypothesis_proofs.begin(),
                  proof.hypothesis_proofs.end(),
                  [](const FrozenHypothesisPlProofV1& item) {
                    return std::isfinite(item.rank_tolerance) &&
                        validateFrozenHypothesisPlEntry(item);
                  });
  const bool dual_payloads_valid = std::all_of(
      proof.dual_channel_proofs.begin(), proof.dual_channel_proofs.end(),
      [](const DualChannelNumericalProofV1& item) {
        return validateDualChannelNumericalProof(item);
      });
  const bool dual_valid = dual_payloads_valid &&
      ((proof.pooled_only && proof.dual_channel_proofs.empty()) ||
       proof.dual_channel_proofs.size() == proof.hypotheses.size());
  const RiskBudgetAudit risk_audit = auditRiskBudget(
      proof.risk, proof.hypotheses);
  const SymmetricPsdCertificate covariance_certificate = certifyFactorGram(
      proof.protected_covariance_factor, proof.protected_covariance,
      proof.covariance_rank_tolerance, proof.candidate_proof_identity);
  const double expected_nominal_tail =
      std::max(1e-15, proof.risk.nominal_axis_tail);
  const double expected_nominal_multiplier =
      StatisticalBoundsCache::normalTwoSidedMultiplier(expected_nominal_tail);
  const Eigen::Vector3d expected_sigma =
      proof.protected_covariance.diagonal().cwiseSqrt();
  const Eigen::Vector3d expected_nominal =
      expected_nominal_multiplier * expected_sigma;
  Eigen::Vector3d expected_bridge = proof.fixed_bridge_margin.cwiseAbs();
  bool bridges_valid = proof.fixed_bridge_margin.allFinite();
  for (const auto& bridge : proof.bridge_proofs) {
    const bool dimensions_valid = bridge.projected_gain.rows() == 3 &&
        bridge.projected_gain.cols() == bridge.deterministic_bound.size() &&
        bridge.deterministic_bound.allFinite() &&
        bridge.projected_gain.allFinite() &&
        (bridge.deterministic_bound.array() >= 0.0).all();
    if (!dimensions_valid) {
      bridges_valid = false;
      break;
    }
    const Eigen::Vector3d component =
        bridge.projected_gain.cwiseAbs() * bridge.deterministic_bound;
    bridges_valid = bridges_valid && same(component, bridge.served_component);
    expected_bridge += component;
  }
  bool components_valid = proof.component_proofs.size() ==
      proof.hypotheses.size();
  Eigen::Vector3d expected_fault = Eigen::Vector3d::Zero();
  double max_hypothesis_tail = 0.0;
  double max_axis_tail = 0.0;
  double max_multiplier = 0.0;
  double max_noncentrality = 0.0;
  std::string expected_detector_certificate;
  std::size_t dual_index = 0;
  for (std::size_t index = 0;
       components_valid && index < proof.component_proofs.size(); ++index) {
    const auto& item = proof.component_proofs[index];
    const auto& hypothesis = proof.hypotheses[index];
    double tail = hypothesis.hmi_allocation > 0.0
        ? hypothesis.hmi_allocation /
              std::max(hypothesis.prior_probability_bound, 1e-15)
        : expected_nominal_tail;
    if (!std::isfinite(tail) || tail <= 0.0) {
      components_valid = false;
      break;
    }
    tail = std::min(0.5, tail);
    const double axis_tail = tail / 3.0;
    bool multiplier_valid = false;
    const double multiplier =
        StatisticalBoundsCache::normalTwoSidedMultiplierVerified(
            axis_tail, &multiplier_valid);
    Eigen::Vector3d component = Eigen::Vector3d::Zero();
    double noncentrality = 0.0;
    if (item.dual_channel) {
      if (dual_index >= proof.dual_channel_proofs.size()) {
        components_valid = false;
        break;
      }
      const auto& dual = proof.dual_channel_proofs[dual_index++];
      component = dual.served_result.axis_bound_m +
          dual.request.k_axis.cwiseProduct(expected_sigma) +
          dual.served_result.position_rho_m;
      for (const double value : dual.served_result.channel_lambda) {
        noncentrality = std::max(noncentrality, value);
      }
      expected_detector_certificate = dual.served_result.certificate_id;
    } else {
      const StatisticalBoundKey key;
      const NoncentralityBoundaryResult boundary =
          StatisticalBoundsCache::noncentralityBoundaryVerified(
              item.detector_dof, item.detector_threshold,
              hypothesis.p_md_allocation, key);
      if (!boundary.valid) {
        components_valid = false;
        break;
      }
      noncentrality = std::sqrt(boundary.value);
      component = proof.hypothesis_proofs[index]
                      .served_entry.protected_slopes * noncentrality +
                  multiplier * expected_sigma;
    }
    components_valid = components_valid && multiplier_valid &&
        item.hypothesis == hypothesis.id &&
        item.hypothesis_tail == tail && item.axis_tail == axis_tail &&
        item.multiplier == multiplier &&
        item.noncentrality == noncentrality &&
        same(item.served_component, component);
    expected_fault = expected_fault.cwiseMax(component);
    max_hypothesis_tail = std::max(max_hypothesis_tail, tail);
    max_axis_tail = std::max(max_axis_tail, axis_tail);
    max_multiplier = std::max(max_multiplier, multiplier);
    max_noncentrality = std::max(max_noncentrality, noncentrality);
  }
  components_valid = components_valid &&
      dual_index == proof.dual_channel_proofs.size();
  const Eigen::Vector3d expected_pl =
      expected_nominal.cwiseMax(expected_fault) + expected_bridge;
  const double expected_hpl = std::hypot(expected_pl.x(), expected_pl.y());
  const double expected_vpl = expected_pl.z();
  const bool expected_model_valid = expected_pl.allFinite() &&
      proof.detector_passed;
  const Availability expected_availability = expected_model_valid &&
      expected_hpl <= proof.risk.horizontal_alert_limit_m &&
      expected_vpl <= proof.risk.vertical_alert_limit_m
      ? Availability::Available : Availability::Unavailable;
  std::string expected_reason;
  if (!proof.detector_passed) expected_reason = "post-FDE detector alarm";
  else if (expected_availability != Availability::Available) {
    expected_reason = "post-FDE protection level exceeds alert limit";
  } else {
    expected_reason = "research V2 implemented; Gate J calibration/review pending";
  }
  const auto& served = proof.served_result;
  const bool result_valid = covariance_certificate.valid &&
      (proof.protected_covariance.diagonal().array() > 0.0).all() &&
      proof.nominal_tail == expected_nominal_tail &&
      proof.nominal_multiplier == expected_nominal_multiplier &&
      same(proof.nominal_sigma, expected_sigma) &&
      same(served.nominal_component_m, expected_nominal) &&
      bridges_valid && same(served.bridge_component_m, expected_bridge) &&
      components_valid && same(served.fault_component_m, expected_fault) &&
      same(served.pl_xyz_m, expected_pl) && served.hpl_m == expected_hpl &&
      served.vpl_m == expected_vpl &&
      served.allocated_outcome_risk == risk_audit.total &&
      served.risk_budget_valid == risk_audit.valid &&
      served.model_valid == expected_model_valid &&
      served.formal_eligible == false &&
      served.availability == expected_availability &&
      served.hypothesis_tail_used == max_hypothesis_tail &&
      served.axis_tail_used == max_axis_tail &&
      served.fault_multiplier_used == max_multiplier &&
      served.noncentrality_used == max_noncentrality &&
      served.detector_certificate_id == expected_detector_certificate &&
      served.reason == expected_reason;
  const bool valid = proof.schema_version == 1 && hypotheses_valid &&
      dual_valid && proof.candidate_proof_identity != 0 &&
      proof.detector_proof_identity != 0 &&
      proof.detector_candidate_numerical_identity != 0 &&
      result_valid &&
      proof.proof_identity != 0 &&
      proof.proof_identity == protectionResultProofIdentity(proof);
  if (!valid && reason) {
    if (!hypotheses_valid) *reason = "protection-level hypothesis payload mismatch";
    else if (!dual_valid) *reason = "protection-level dual payload mismatch";
    else if (!result_valid) *reason = "protection-level independent recomputation mismatch";
    else *reason = "protection-level payload digest mismatch";
  }
  return valid;
}

bool protectionLevelV2Proof(const ProtectionLevelV2Result& result,
                            ProtectionLevelV2ProofV1* proof) {
  if (!proof) return false;
  auto& registry = protectionProofRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto found = registry.by_result.find(resultRegistryKey(result));
  if (found == registry.by_result.end() || found->second.empty()) return false;
  *proof = found->second.back();
  return true;
}

std::uint64_t protectionLevelV2ProofIdentity(
    const ProtectionLevelV2Result& result) {
  ProtectionLevelV2ProofV1 proof;
  return protectionLevelV2Proof(result, &proof) ? proof.proof_identity : 0;
}

std::uint64_t protectionLevelV2ProofIdentity(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result) {
  auto& registry = protectionProofRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto found = registry.by_result.find(resultRegistryKey(result));
  if (found == registry.by_result.end()) return 0;
  const auto match = std::find_if(
      found->second.rbegin(), found->second.rend(),
      [&](const ProtectionLevelV2ProofV1& proof) {
        return proof.candidate_proof_identity == candidateProofIdentity(candidate);
      });
  return match == found->second.rend() ? 0 : match->proof_identity;
}

bool bindProtectionLevelPublicationPacket(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    std::uint64_t protection_proof_identity,
    std::string* packet_id) {
  if (!packet_id) return false;
  ProtectionLevelV2ProofV1 proof;
  {
    auto& registry = protectionProofRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    const auto found = registry.by_identity.find(protection_proof_identity);
    if (found == registry.by_identity.end()) return false;
    proof = found->second;
  }
  const std::uint64_t candidate_identity = candidateProofIdentity(candidate);
  if (candidate_identity == 0 ||
      proof.candidate_proof_identity != candidate_identity ||
      resultRegistryKey(proof.served_result) != resultRegistryKey(result) ||
      !validateProtectionLevelV2ProofPayload(proof)) {
    return false;
  }
  ProtectionLevelPublicationPacketV1 packet;
  packet.candidate_proof_identity = candidate_identity;
  packet.protection_proof_identity = protection_proof_identity;
  packet.served_pl = result.pl_xyz_m;
  packet.detector_certificate_id = result.detector_certificate_id;
  std::uint64_t identity = 1469598103934665603ULL;
  hashScalar(&identity, packet.schema_version);
  hashScalar(&identity, packet.candidate_proof_identity);
  hashScalar(&identity, packet.protection_proof_identity);
  hashMatrix(&identity, packet.served_pl);
  hashBytes(&identity, packet.detector_certificate_id.data(),
            packet.detector_certificate_id.size());
  packet.packet_identity = identity;
  *packet_id = "p003-pl-publication-v1-" + hexDigest(identity);
  auto& registry = protectionProofRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  registry.publication_packets[*packet_id] = packet;
  return true;
}

bool protectionLevelPublicationPacket(
    const std::string& packet_id,
    ProtectionLevelPublicationPacketV1* packet) {
  if (!packet) return false;
  auto& registry = protectionProofRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto found = registry.publication_packets.find(packet_id);
  if (found == registry.publication_packets.end()) return false;
  *packet = found->second;
  return true;
}

std::uint64_t protectionLevelPublicationProofIdentity(
    const Eigen::Vector3d& served_pl,
    const std::string& detector_certificate_id) {
  auto& registry = protectionProofRegistry();
  std::lock_guard<std::mutex> lock(registry.mutex);
  const auto found = registry.publication_packets.find(
      detector_certificate_id);
  if (found == registry.publication_packets.end() ||
      !(found->second.served_pl.array() == served_pl.array()).all()) return 0;
  return found->second.protection_proof_identity;
}

bool validateProtectionLevelPublicationProof(
    std::uint64_t proof_identity, const Eigen::Vector3d& served_pl,
    const std::string& detector_certificate_id, std::string* reason) {
  ProtectionLevelPublicationPacketV1 packet;
  ProtectionLevelV2ProofV1 proof;
  {
    auto& registry = protectionProofRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    const auto packet_found = registry.publication_packets.find(
        detector_certificate_id);
    const auto proof_found = registry.by_identity.find(proof_identity);
    if (packet_found == registry.publication_packets.end() ||
        proof_found == registry.by_identity.end()) {
      if (reason) *reason = "publication PL proof packet is unavailable";
      return false;
    }
    packet = packet_found->second;
    proof = proof_found->second;
  }
  if (proof.proof_identity == 0) {
    if (reason) *reason = "publication PL proof sidecar is unavailable";
    return false;
  }
  std::uint64_t expected_packet_identity = 1469598103934665603ULL;
  hashScalar(&expected_packet_identity, packet.schema_version);
  hashScalar(&expected_packet_identity, packet.candidate_proof_identity);
  hashScalar(&expected_packet_identity, packet.protection_proof_identity);
  hashMatrix(&expected_packet_identity, packet.served_pl);
  hashBytes(&expected_packet_identity, packet.detector_certificate_id.data(),
            packet.detector_certificate_id.size());
  const bool valid = validateProtectionLevelV2ProofPayload(proof) &&
      proof.proof_identity == proof_identity &&
      packet.schema_version == 1 &&
      packet.protection_proof_identity == proof_identity &&
      packet.candidate_proof_identity == proof.candidate_proof_identity &&
      packet.detector_certificate_id ==
          proof.served_result.detector_certificate_id &&
      packet.packet_identity == expected_packet_identity &&
      detector_certificate_id ==
          "p003-pl-publication-v1-" + hexDigest(expected_packet_identity) &&
      (packet.served_pl.array() == served_pl.array()).all() &&
      (proof.served_result.pl_xyz_m.array() == served_pl.array()).all();
  if (!valid && reason) *reason = "publication PL proof recomputation failed";
  return valid;
}

double ProtectionLevelV2::detectionBoundaryNoncentralitySquared(
    int dof, double threshold, double p_md) {
  return StatisticalBoundsCache::noncentralityBoundary(
      dof, threshold, p_md);
}

ProtectionLevelV2Result ProtectionLevelV2::compute(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate,
    const DetectorResultV2& detector,
    std::vector<FaultHypothesisV2>* hypotheses,
    const RiskBudgetV2& risk,
    const Eigen::Vector3d& bridge_margin) const {
  return computeStreaming(
      window, candidate, detector, hypotheses,
      [](const FaultHypothesisV2& hypothesis) { return hypothesis.A; },
      risk, bridge_margin);
}

ProtectionLevelV2Result ProtectionLevelV2::computeStreaming(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate,
    const DetectorResultV2& detector,
    std::vector<FaultHypothesisV2>* hypotheses,
    const FaultMapProvider& fault_map_provider,
    const RiskBudgetV2& risk,
    const Eigen::Vector3d& bridge_margin) const {
  ProtectionLevelV2Result result;
  ProtectionLevelV2ProofV1 sidecar;
  result.bridge_component_m = bridge_margin.cwiseAbs();
  sidecar.fixed_bridge_margin = result.bridge_component_m;
  sidecar.risk = risk;
  std::string detector_contract_reason;
  if (!hypotheses || !candidate.valid || !detector.numerically_valid ||
      !detectorContractMatchesCandidate(window, candidate, detector,
                                        &detector_contract_reason) ||
      window.protected_state_map.cols() != candidate.state_increment.rows() ||
      window.protected_state_map.rows() != 3) {
    result.reason = detector_contract_reason.empty()
        ? "invalid post-FDE candidate/protected-state map"
        : detector_contract_reason;
    return result;
  }
  const RiskBudgetAudit risk_audit = auditRiskBudget(risk, *hypotheses);
  result.allocated_outcome_risk = risk_audit.total;
  result.risk_budget_valid = risk_audit.valid;
  if (!result.risk_budget_valid) {
    result.reason = "outcome-conditioned risk budget does not close";
    return result;
  }
  std::vector<Eigen::MatrixXd> fault_maps;
  std::vector<Eigen::MatrixXd> crosses;
  fault_maps.reserve(hypotheses->size());
  crosses.reserve(hypotheses->size());
  int solve_columns = 3;
  for (auto& hypothesis : *hypotheses) {
    Eigen::MatrixXd fault_map = fault_map_provider(hypothesis);
    if (fault_map.rows() != candidate.rows || fault_map.cols() == 0 ||
        !fault_map.allFinite()) {
      hypothesis.monitored = false;
      hypothesis.monitorability.reason = "post-FDE fault map row mismatch";
      result.reason = "remaining hypothesis cannot be mapped post-FDE";
      return result;
    }
    Eigen::MatrixXd cross = candidate.normalCross(fault_map);
    if (cross.rows() != candidate.state_increment.rows()) {
      result.reason = "candidate normal cross-product failed";
      return result;
    }
    solve_columns += cross.cols();
    fault_maps.push_back(std::move(fault_map));
    crosses.push_back(std::move(cross));
  }
  Eigen::MatrixXd solve_rhs(candidate.state_increment.rows(), solve_columns);
  solve_rhs.leftCols<3>() = window.protected_state_map.transpose();
  int solve_column = 3;
  for (const auto& cross : crosses) {
    solve_rhs.middleCols(solve_column, cross.cols()) = cross;
    solve_column += cross.cols();
  }
  const Eigen::MatrixXd covariance_solutions = candidate.covarianceTimes(solve_rhs);
  if (covariance_solutions.rows() != candidate.state_increment.rows() ||
      covariance_solutions.cols() != solve_columns) {
    result.reason = "candidate covariance solve failed";
    return result;
  }
  const double covariance_rank_tolerance = window.numerics
      ? window.numerics->numerical_contract.rank_tolerance : 1e-10;
  const RawFactorCertificate raw_factor = rawFactorCertificate(
      window, candidate, covariance_rank_tolerance);
  if (!raw_factor.valid) {
    result.reason = "direct raw-H certificate failed: " + raw_factor.reason;
    return result;
  }
  const Eigen::Matrix3d protected_covariance =
      raw_factor.protected_covariance;
  const SymmetricPsdCertificate covariance_certificate = certifyFactorGram(
      raw_factor.protected_factor.transpose(), protected_covariance,
      covariance_rank_tolerance,
      candidateProofIdentity(candidate));
  if (!covariance_certificate.valid ||
      (protected_covariance.diagonal().array() <= 0.0).any()) {
    result.reason = "post-FDE protected covariance is invalid: " +
        covariance_certificate.reason;
    return result;
  }
  const double nominal_tail = std::max(1e-15, risk.nominal_axis_tail);
  const double nominal_k = StatisticalBoundsCache::normalTwoSidedMultiplier(
      nominal_tail);
  const Eigen::Vector3d sigma =
      protected_covariance.diagonal().cwiseSqrt();
  sidecar.protected_covariance = protected_covariance;
  sidecar.protected_covariance_factor =
      raw_factor.protected_factor.transpose();
  sidecar.covariance_rank_tolerance = covariance_rank_tolerance;
  sidecar.nominal_tail = nominal_tail;
  sidecar.nominal_multiplier = nominal_k;
  sidecar.nominal_sigma = sigma;
  result.nominal_component_m = nominal_k * sigma;
  result.fault_component_m.setZero();
  solve_column = 3;
  for (std::size_t index = 0; index < hypotheses->size(); ++index) {
    auto& hypothesis = (*hypotheses)[index];
    const Eigen::MatrixXd& fault_map = fault_maps[index];
    const Eigen::MatrixXd& cross = crosses[index];
    const Eigen::MatrixXd covariance_cross = covariance_solutions.middleCols(
        solve_column, cross.cols());
    solve_column += cross.cols();
    Eigen::MatrixXd gram;
    Eigen::MatrixXd protected_fault;
    Eigen::MatrixXd residual_factor;
    if (!raw_factor.faultProducts(fault_map, &gram, &protected_fault,
                                  &residual_factor)) {
      result.reason = "direct raw-H fault certificate failed";
      return result;
    }
    NumericalWorkCounters::faultGramEigen();
    const double rank_tolerance = window.numerics
        ? window.numerics->numerical_contract.rank_tolerance : 1e-10;
    const GramResponseCertificate gram_certificate =
        certifyFactorGramAndProtectedResponse(
            residual_factor, gram, protected_fault,
            rank_tolerance, candidateProofIdentity(candidate),
            fault_map.norm());
    sidecar.hypothesis_proofs.push_back(protectionHypothesisProof(
        hypothesis.id, gram_certificate, residual_factor, fault_map.norm(),
        rank_tolerance));
    sidecar.hypothesis_proofs.back().protected_response = protected_fault;
    sidecar.hypothesis_proofs.back().parent_proof_identity =
        candidateProofIdentity(candidate);
    hypothesis.monitorability.parameter_dimension = fault_map.cols();
    hypothesis.monitorability.rank = gram_certificate.gram.rank;
    hypothesis.monitorability.sigma_min = gram_certificate.gram.sigma_min;
    hypothesis.monitorability.sigma_max = gram_certificate.gram.sigma_max;
    hypothesis.monitorability.condition_number = gram_certificate.gram.condition;
    hypothesis.monitorability.monitorable = gram_certificate.valid &&
        gram_certificate.nullspace_class != GramNullspaceClass::Dangerous;
    hypothesis.monitored = hypothesis.monitorability.monitorable;
    if (!hypothesis.monitored) {
      result.reason = "remaining post-FDE fault has no numerical certificate: " +
          gram_certificate.reason;
      return result;
    }
    hypothesis.monitorability.protected_slopes =
        gram_certificate.protected_slopes;
    Eigen::MatrixXd history_gram = historyGramFromCertificate(candidate,
                                                               fault_map);
    if (history_gram.rows() != fault_map.cols()) {
      result.reason = "candidate detector row-role payload is missing";
      return result;
    }
    Eigen::Vector3d component;
    std::string detector_certificate;
    DualChannelNumericalProofV1 dual_channel_proof;
    const FaultAxisBounds bounds = dualChannelAxisBounds(
        detector, hypothesis, gram, history_gram, protected_fault, sigma,
        nominal_tail, rank_tolerance,
        candidateProofIdentity(candidate),
        &component, &detector_certificate, &dual_channel_proof);
    if (!bounds.valid) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " has no valid section 5.9 bound: " + bounds.reason;
      return result;
    }
    result.hypothesis_tail_used = std::max(result.hypothesis_tail_used,
                                           bounds.hypothesis_tail);
    result.axis_tail_used = std::max(result.axis_tail_used, bounds.axis_tail);
    result.fault_multiplier_used = std::max(result.fault_multiplier_used,
                                            bounds.k);
    result.noncentrality_used = std::max(result.noncentrality_used,
                                         bounds.lambda);
    result.detector_certificate_id = detector_certificate;
    sidecar.dual_channel_proofs.push_back(std::move(dual_channel_proof));
    ProtectionHypothesisComponentProofV1 component_proof;
    component_proof.hypothesis = hypothesis.id;
    component_proof.detector_dof = detector.dof;
    component_proof.detector_threshold = detector.squared_threshold;
    component_proof.hypothesis_tail = bounds.hypothesis_tail;
    component_proof.axis_tail = bounds.axis_tail;
    component_proof.multiplier = bounds.k;
    component_proof.noncentrality = bounds.lambda;
    component_proof.dual_channel = true;
    component_proof.served_component = component;
    sidecar.component_proofs.push_back(std::move(component_proof));
    result.fault_component_m = result.fault_component_m.cwiseMax(component);
  }
  result.pl_xyz_m = result.nominal_component_m
      .cwiseMax(result.fault_component_m) + result.bridge_component_m;
  result.hpl_m = std::hypot(result.pl_xyz_m.x(), result.pl_xyz_m.y());
  result.vpl_m = result.pl_xyz_m.z();
  result.model_valid = result.pl_xyz_m.allFinite() && detector.passed;
  result.availability = result.model_valid &&
      result.hpl_m <= risk.horizontal_alert_limit_m &&
      result.vpl_m <= risk.vertical_alert_limit_m
      ? Availability::Available : Availability::Unavailable;
  result.formal_eligible = false;  // Gate J evidence is intentionally absent.
  if (!detector.passed) result.reason = "post-FDE detector alarm";
  else if (result.availability != Availability::Available) {
    result.reason = "post-FDE protection level exceeds alert limit";
  } else {
    result.reason = "research V2 implemented; Gate J calibration/review pending";
  }
  bindProtectionResultProof(candidate, detector, *hypotheses, &result,
                            &sidecar);
  return result;
}

ProtectionLevelV2Result ProtectionLevelV2::computeShared(
    const LinearizedIntegrityWindow& window,
    CandidateEvaluation* candidate,
    const DetectorResultV2& detector,
    std::vector<FaultHypothesisV2>* hypotheses,
    const ProtectionLevelSharedContext& shared,
    const RiskBudgetV2& risk) const {
  ProtectionLevelV2Result result;
  ProtectionLevelV2ProofV1 sidecar;
  sidecar.risk = risk;
  std::string detector_contract_reason;
  if (!candidate || !hypotheses || !candidate->valid ||
      !detector.numerically_valid ||
      !detectorContractMatchesCandidate(window, *candidate, detector,
                                        &detector_contract_reason) ||
      window.protected_state_map.cols() != candidate->state_increment.rows() ||
      window.protected_state_map.rows() != 3) {
    result.reason = detector_contract_reason.empty()
        ? "invalid post-FDE candidate/protected-state map"
        : detector_contract_reason;
    return result;
  }
  std::map<std::uint64_t, const Eigen::MatrixXd*> modes;
  for (const auto& hypothesis : *hypotheses) {
    for (const auto id : hypothesis.modes) {
      const auto found = shared.mode_maps.find(id.value());
      if (found == shared.mode_maps.end() || found->second.rows() != candidate->rows ||
          found->second.cols() == 0 || !found->second.allFinite()) {
        result.reason = "remaining hypothesis cannot be mapped post-FDE";
        return result;
      }
      modes.emplace(id.value(), &found->second);
    }
  }
  const RiskBudgetAudit risk_audit = auditRiskBudget(risk, *hypotheses);
  result.allocated_outcome_risk = risk_audit.total;
  result.risk_budget_valid = risk_audit.valid;
  if (!result.risk_budget_valid) {
    result.reason = "outcome-conditioned risk budget does not close";
    return result;
  }
  struct SolvedMode {
    const Eigen::MatrixXd* map = nullptr;
    Eigen::MatrixXd cross;
    Eigen::Index solve_offset = 0;
  };
  std::map<std::uint64_t, SolvedMode> solved_modes;
  Eigen::Index solve_columns = 3;
  for (const auto& item : modes) {
    SolvedMode mode;
    mode.map = item.second;
    mode.cross = candidate->normalCross(*item.second);
    if (mode.cross.rows() != candidate->state_increment.rows()) {
      result.reason = "candidate normal cross-product failed";
      return result;
    }
    mode.solve_offset = solve_columns;
    solve_columns += mode.cross.cols();
    solved_modes.emplace(item.first, std::move(mode));
  }
  std::vector<Eigen::Index> bridge_offsets;
  bridge_offsets.reserve(shared.bridges.size());
  for (const auto& bridge : shared.bridges) {
    if (bridge.jacobian_whitened.cols() != candidate->state_increment.rows() ||
        bridge.jacobian_whitened.rows() != bridge.deterministic_bound.size() ||
        !bridge.jacobian_whitened.allFinite() ||
        !bridge.deterministic_bound.allFinite() ||
        (bridge.deterministic_bound.array() < 0.0).any()) {
      result.reason = "bridge projection dimensions invalid";
      return result;
    }
    bridge_offsets.push_back(solve_columns);
    solve_columns += bridge.jacobian_whitened.rows();
  }
  Eigen::MatrixXd rhs(candidate->state_increment.rows(), solve_columns);
  rhs.leftCols<3>() = window.protected_state_map.transpose();
  for (const auto& item : solved_modes) {
    rhs.middleCols(item.second.solve_offset, item.second.cross.cols()) =
        item.second.cross;
  }
  for (std::size_t i = 0; i < shared.bridges.size(); ++i) {
    const auto& bridge = shared.bridges[i];
    rhs.middleCols(bridge_offsets[i], bridge.jacobian_whitened.rows()) =
        bridge.jacobian_whitened.transpose();
  }
  const Eigen::MatrixXd solutions = candidate->covarianceTimes(rhs);
  ++candidate->diagnostics.covariance_solve_count;
  if (solutions.rows() != candidate->state_increment.rows() ||
      solutions.cols() != solve_columns || !solutions.allFinite()) {
    result.reason = "candidate covariance solve failed";
    return result;
  }
  const double covariance_rank_tolerance = window.numerics
      ? window.numerics->numerical_contract.rank_tolerance : 1e-10;
  const RawFactorCertificate raw_factor = rawFactorCertificate(
      window, *candidate, covariance_rank_tolerance);
  if (!raw_factor.valid) {
    result.reason = "direct raw-H certificate failed: " + raw_factor.reason;
    return result;
  }
  const Eigen::Matrix3d protected_covariance =
      raw_factor.protected_covariance;
  const SymmetricPsdCertificate covariance_certificate = certifyFactorGram(
      raw_factor.protected_factor.transpose(), protected_covariance,
      covariance_rank_tolerance,
      candidateProofIdentity(*candidate));
  if (!covariance_certificate.valid ||
      (protected_covariance.diagonal().array() <= 0.0).any()) {
    result.reason = "post-FDE protected covariance is invalid: " +
        covariance_certificate.reason;
    return result;
  }
  result.bridge_component_m.setZero();
  for (std::size_t i = 0; i < shared.bridges.size(); ++i) {
    const auto& bridge = shared.bridges[i];
    const Eigen::MatrixXd gain = solutions.middleCols(
        bridge_offsets[i], bridge.jacobian_whitened.rows());
    ProtectionBridgeProofV1 bridge_proof;
    bridge_proof.projected_gain = window.protected_state_map * gain;
    bridge_proof.deterministic_bound = bridge.deterministic_bound;
    bridge_proof.served_component =
        bridge_proof.projected_gain.cwiseAbs() *
        bridge_proof.deterministic_bound;
    result.bridge_component_m += bridge_proof.served_component;
    sidecar.bridge_proofs.push_back(std::move(bridge_proof));
  }
  const double nominal_tail = std::max(1e-15, risk.nominal_axis_tail);
  const double nominal_k = StatisticalBoundsCache::normalTwoSidedMultiplier(
      nominal_tail);
  const Eigen::Vector3d sigma = protected_covariance.diagonal().cwiseSqrt();
  sidecar.protected_covariance = protected_covariance;
  sidecar.protected_covariance_factor =
      raw_factor.protected_factor.transpose();
  sidecar.covariance_rank_tolerance = covariance_rank_tolerance;
  sidecar.nominal_tail = nominal_tail;
  sidecar.nominal_multiplier = nominal_k;
  sidecar.nominal_sigma = sigma;
  result.nominal_component_m = nominal_k * sigma;
  result.fault_component_m.setZero();
  for (auto& hypothesis : *hypotheses) {
    Eigen::Index columns = 0;
    for (const auto id : hypothesis.modes)
      columns += solved_modes.at(id.value()).map->cols();
    Eigen::MatrixXd protected_fault(3, columns);
    Eigen::MatrixXd fault_map(candidate->rows, columns);
    Eigen::Index offset = 0;
    for (const auto left_id : hypothesis.modes) {
      const auto& left = solved_modes.at(left_id.value());
      const Eigen::Index left_count = left.map->cols();
      fault_map.middleCols(offset, left_count) = *left.map;
      offset += left_count;
    }
    Eigen::MatrixXd gram;
    Eigen::MatrixXd residual_factor;
    if (!raw_factor.faultProducts(fault_map, &gram, &protected_fault,
                                  &residual_factor)) {
      result.reason = "direct raw-H fault certificate failed";
      return result;
    }
    NumericalWorkCounters::faultGramEigen();
    const double rank_tolerance = window.numerics
        ? window.numerics->numerical_contract.rank_tolerance : 1e-10;
    const GramResponseCertificate gram_certificate =
        certifyFactorGramAndProtectedResponse(
            residual_factor, gram, protected_fault, rank_tolerance,
            candidateProofIdentity(*candidate),
            fault_map.norm());
    sidecar.hypothesis_proofs.push_back(protectionHypothesisProof(
        hypothesis.id, gram_certificate, residual_factor, fault_map.norm(),
        rank_tolerance));
    sidecar.hypothesis_proofs.back().protected_response = protected_fault;
    sidecar.hypothesis_proofs.back().parent_proof_identity =
        candidateProofIdentity(*candidate);
    auto& monitor = hypothesis.monitorability;
    monitor.parameter_dimension = columns;
    monitor.rank = gram_certificate.gram.rank;
    monitor.sigma_min = gram_certificate.gram.sigma_min;
    monitor.sigma_max = gram_certificate.gram.sigma_max;
    monitor.condition_number = gram_certificate.gram.condition;
    monitor.monitorable = gram_certificate.valid &&
        gram_certificate.nullspace_class != GramNullspaceClass::Dangerous;
    hypothesis.monitored = monitor.monitorable;
    if (!hypothesis.monitored) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " is unmonitorable: rank=" + std::to_string(monitor.rank) +
          "/" + std::to_string(columns) + ";sigma_min=" +
          std::to_string(monitor.sigma_min) + ";condition=" +
          std::to_string(monitor.condition_number);
      return result;
    }
    monitor.protected_slopes = gram_certificate.protected_slopes;
    Eigen::MatrixXd history_gram = historyGramFromCertificate(*candidate,
                                                               fault_map);
    if (history_gram.rows() != columns) {
      result.reason = "candidate detector row-role payload is missing";
      return result;
    }
    Eigen::Vector3d component;
    std::string detector_certificate;
    DualChannelNumericalProofV1 dual_channel_proof;
    const FaultAxisBounds bounds = dualChannelAxisBounds(
        detector, hypothesis, gram, history_gram, protected_fault, sigma,
        nominal_tail, rank_tolerance,
        candidateProofIdentity(*candidate),
        &component, &detector_certificate, &dual_channel_proof);
    if (!bounds.valid) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " has no valid section 5.9 bound: " + bounds.reason;
      return result;
    }
    result.hypothesis_tail_used = std::max(result.hypothesis_tail_used,
                                           bounds.hypothesis_tail);
    result.axis_tail_used = std::max(result.axis_tail_used, bounds.axis_tail);
    result.fault_multiplier_used = std::max(result.fault_multiplier_used,
                                            bounds.k);
    result.noncentrality_used = std::max(result.noncentrality_used,
                                         bounds.lambda);
    result.detector_certificate_id = detector_certificate;
    sidecar.dual_channel_proofs.push_back(std::move(dual_channel_proof));
    ProtectionHypothesisComponentProofV1 component_proof;
    component_proof.hypothesis = hypothesis.id;
    component_proof.detector_dof = detector.dof;
    component_proof.detector_threshold = detector.squared_threshold;
    component_proof.hypothesis_tail = bounds.hypothesis_tail;
    component_proof.axis_tail = bounds.axis_tail;
    component_proof.multiplier = bounds.k;
    component_proof.noncentrality = bounds.lambda;
    component_proof.dual_channel = true;
    component_proof.served_component = component;
    sidecar.component_proofs.push_back(std::move(component_proof));
    result.fault_component_m = result.fault_component_m.cwiseMax(component);
  }
  result.pl_xyz_m = result.nominal_component_m.cwiseMax(
      result.fault_component_m) + result.bridge_component_m;
  result.hpl_m = std::hypot(result.pl_xyz_m.x(), result.pl_xyz_m.y());
  result.vpl_m = result.pl_xyz_m.z();
  result.model_valid = result.pl_xyz_m.allFinite() && detector.passed;
  result.availability = result.model_valid &&
      result.hpl_m <= risk.horizontal_alert_limit_m &&
      result.vpl_m <= risk.vertical_alert_limit_m
      ? Availability::Available : Availability::Unavailable;
  result.formal_eligible = false;
  if (!detector.passed) result.reason = "post-FDE detector alarm";
  else if (result.availability != Availability::Available)
    result.reason = "post-FDE protection level exceeds alert limit";
  else result.reason = "research V2 implemented; Gate J calibration/review pending";
  bindProtectionResultProof(*candidate, detector, *hypotheses, &result,
                            &sidecar);
  return result;
}

ProtectionLevelV2Result ProtectionLevelV2::computeFrozenAllIn(
    const LinearizedIntegrityWindow& window,
    CandidateEvaluation* candidate,
    const DetectorResultV2& detector,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const std::vector<FaultModeBasis>& modes,
    const FrozenHypothesisNumerics& frozen,
    std::uint64_t fault_model_policy_fingerprint,
    const RiskBudgetV2& risk) const {
  ProtectionLevelV2Result result;
  ProtectionLevelV2ProofV1 sidecar;
  sidecar.risk = risk;
  std::string detector_contract_reason;
  const bool keep_all = candidate && candidate->action.groups_to_remove.empty() &&
      candidate->action.groups_to_add.empty() && candidate->action.added_blocks.empty();
  const bool identity_valid = frozen.valid && window.numerics &&
      frozen.window_id == window.id && frozen.version == window.version &&
      frozen.window_content_fingerprint ==
          window.numerics->content_fingerprint &&
      frozen.window_content_fingerprint == integrityWindowFingerprint(window) &&
      frozen.numerical_contract_fingerprint ==
          window.numerics->numerical_contract_fingerprint &&
      frozen.hypothesis_fingerprint == hypothesisSetFingerprint(hypotheses) &&
      frozen.fault_mode_fingerprint == faultModeSetFingerprint(modes) &&
      frozen.fault_model_policy_fingerprint ==
          fault_model_policy_fingerprint &&
      frozen.hypothesis_count == hypotheses.size() &&
      frozen.pl_entries.size() == hypotheses.size();
  if (!candidate || !candidate->valid || !detector.numerically_valid ||
      !detectorContractMatchesCandidate(window, *candidate, detector,
                                        &detector_contract_reason) ||
      !keep_all || !identity_valid) {
    NumericalWorkCounters::hypothesisSharedMiss();
    result.reason = detector_contract_reason.empty()
        ? "frozen all-in hypothesis context identity mismatch"
        : detector_contract_reason;
    return result;
  }
  if (detector.contract_mode == DetectorContractMode::ActiveDualChannelV6 &&
      detector.channel_history_dof > 0) {
    NumericalWorkCounters::hypothesisSharedMiss();
    result.reason =
        "frozen pooled hypothesis context cannot certify active dual-channel PL";
    return result;
  }
  NumericalWorkCounters::hypothesisSharedHit();
  const RiskBudgetAudit risk_audit = auditRiskBudget(risk, hypotheses);
  result.allocated_outcome_risk = risk_audit.total;
  result.risk_budget_valid = risk_audit.valid;
  if (!result.risk_budget_valid) {
    result.reason = "outcome-conditioned risk budget does not close";
    return result;
  }
  const double covariance_rank_tolerance = window.numerics
      ? window.numerics->numerical_contract.rank_tolerance : 1e-10;
  const RawFactorCertificate raw_factor = rawFactorCertificate(
      window, *candidate, covariance_rank_tolerance);
  // The frozen context is an optimization cache, not an independent numerical
  // authority.  Serve the same direct raw-H covariance used by the ordinary
  // paths so the constructive Gram proof binds exactly the matrix returned to
  // the caller.
  const Eigen::Matrix3d protected_covariance =
      raw_factor.valid ? raw_factor.protected_covariance
                       : Eigen::Matrix3d::Zero();
  const SymmetricPsdCertificate covariance_certificate = raw_factor.valid
      ? certifyFactorGram(raw_factor.protected_factor.transpose(),
                          protected_covariance, covariance_rank_tolerance,
                          candidateProofIdentity(*candidate))
      : SymmetricPsdCertificate{};
  if (!covariance_certificate.valid ||
      (protected_covariance.diagonal().array() <= 0.0).any()) {
    result.reason = "post-FDE protected covariance is invalid: " +
        covariance_certificate.reason;
    return result;
  }
  const double nominal_tail = std::max(1e-15, risk.nominal_axis_tail);
  const double nominal_k = StatisticalBoundsCache::normalTwoSidedMultiplier(
      nominal_tail);
  const Eigen::Vector3d sigma =
      protected_covariance.diagonal().cwiseSqrt();
  sidecar.protected_covariance = protected_covariance;
  sidecar.protected_covariance_factor =
      raw_factor.protected_factor.transpose();
  sidecar.covariance_rank_tolerance = covariance_rank_tolerance;
  sidecar.nominal_tail = nominal_tail;
  sidecar.nominal_multiplier = nominal_k;
  sidecar.nominal_sigma = sigma;
  result.nominal_component_m = nominal_k * sigma;
  result.fault_component_m.setZero();
  result.bridge_component_m.setZero();
  for (std::size_t index = 0; index < hypotheses.size(); ++index) {
    const auto& hypothesis = hypotheses[index];
    const auto& cached = frozen.pl_entries[index];
    if (cached.hypothesis != hypothesis.id || !cached.valid ||
        !(cached.gram_spd || cached.bound_from_projected_path) ||
        !cached.monitorability.monitorable) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " is unmonitorable: rank=" +
          std::to_string(cached.monitorability.rank) + "/" +
          std::to_string(cached.monitorability.parameter_dimension) +
          ";sigma_min=" + std::to_string(cached.monitorability.sigma_min) +
          ";condition=" +
          std::to_string(cached.monitorability.condition_number);
      return result;
    }
    std::string cached_proof_reason;
    FrozenHypothesisPlProofV1 cached_proof;
    if (!frozenHypothesisPlProof(cached, &cached_proof) ||
        !validateFrozenHypothesisPlEntry(cached_proof,
                                         &cached_proof_reason)) {
      result.reason = "frozen hypothesis proof rejected: " +
          cached_proof_reason;
      return result;
    }
    sidecar.hypothesis_proofs.push_back(std::move(cached_proof));
    const FaultAxisBounds bounds =
        faultAxisBounds(detector, hypothesis, nominal_tail);
    if (!bounds.valid) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " has no valid section 5.9 bound: " + bounds.reason;
      return result;
    }
    result.hypothesis_tail_used = std::max(result.hypothesis_tail_used,
                                           bounds.hypothesis_tail);
    result.axis_tail_used = std::max(result.axis_tail_used, bounds.axis_tail);
    result.fault_multiplier_used = std::max(result.fault_multiplier_used,
                                            bounds.k);
    result.noncentrality_used = std::max(result.noncentrality_used,
                                         bounds.lambda);
    const Eigen::Vector3d component =
        cached.protected_slopes * bounds.lambda + bounds.k * sigma;
    ProtectionHypothesisComponentProofV1 component_proof;
    component_proof.hypothesis = hypothesis.id;
    component_proof.detector_dof = detector.dof;
    component_proof.detector_threshold = detector.squared_threshold;
    component_proof.hypothesis_tail = bounds.hypothesis_tail;
    component_proof.axis_tail = bounds.axis_tail;
    component_proof.multiplier = bounds.k;
    component_proof.noncentrality = bounds.lambda;
    component_proof.dual_channel = false;
    component_proof.served_component = component;
    sidecar.component_proofs.push_back(std::move(component_proof));
    result.fault_component_m = result.fault_component_m.cwiseMax(component);
  }
  result.pl_xyz_m = result.nominal_component_m.cwiseMax(
      result.fault_component_m);
  result.hpl_m = std::hypot(result.pl_xyz_m.x(), result.pl_xyz_m.y());
  result.vpl_m = result.pl_xyz_m.z();
  result.model_valid = result.pl_xyz_m.allFinite() && detector.passed;
  result.availability = result.model_valid &&
      result.hpl_m <= risk.horizontal_alert_limit_m &&
      result.vpl_m <= risk.vertical_alert_limit_m
      ? Availability::Available : Availability::Unavailable;
  result.formal_eligible = false;
  if (!detector.passed) result.reason = "post-FDE detector alarm";
  else if (result.availability != Availability::Available)
    result.reason = "post-FDE protection level exceeds alert limit";
  else result.reason = "research V2 implemented; Gate J calibration/review pending";
  bindProtectionResultProof(*candidate, detector, hypotheses, &result,
                            &sidecar);
  return result;
}

}  // namespace uwb_imu_pl
