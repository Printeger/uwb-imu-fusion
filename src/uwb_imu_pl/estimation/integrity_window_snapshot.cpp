#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <atomic>
#include <set>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

class CanonicalWriter {
 public:
  template <typename T>
  void scalar(const T& value) {
    const auto* begin = reinterpret_cast<const unsigned char*>(&value);
    bytes_.insert(bytes_.end(), begin, begin + sizeof(value));
  }
  void string(const std::string& value) {
    scalar(static_cast<std::uint64_t>(value.size()));
    const auto* begin = reinterpret_cast<const unsigned char*>(value.data());
    bytes_.insert(bytes_.end(), begin, begin + value.size());
  }
  template <class Derived>
  void matrix(const Eigen::MatrixBase<Derived>& value) {
    scalar(static_cast<std::int64_t>(value.rows()));
    scalar(static_cast<std::int64_t>(value.cols()));
    for (Eigen::Index column = 0; column < value.cols(); ++column) {
      for (Eigen::Index row = 0; row < value.rows(); ++row) {
        scalar(static_cast<double>(value(row, column)));
      }
    }
  }
  const std::vector<unsigned char>& bytes() const { return bytes_; }

 private:
  std::vector<unsigned char> bytes_;
};

void canonicalVersion(CanonicalWriter* out, const LinearizationVersion& value) {
  out->scalar(value.graph_version);
  out->scalar(value.ordering_version);
  out->scalar(value.noise_model_version);
  out->scalar(value.linpoint_version);
}

std::vector<unsigned char> canonicalWindowPayload(
    const LinearizedIntegrityWindow& window) {
  CanonicalWriter out;
  constexpr std::uint64_t kCanonicalSchema = 1;
  out.scalar(kCanonicalSchema);
  out.scalar(window.id.value());
  canonicalVersion(&out, window.version);
  out.scalar(static_cast<std::uint64_t>(window.state_layout.size()));
  for (const auto& entry : window.state_layout) {
    out.scalar(entry.epoch);
    out.scalar(entry.column_offset);
    out.scalar(entry.dimension);
    out.scalar(entry.protected_current_state);
    out.scalar(static_cast<std::uint64_t>(entry.keys.size()));
    for (const auto key : entry.keys) out.scalar(key);
  }
  out.scalar(static_cast<std::uint64_t>(window.blocks.size()));
  for (const auto& block : window.blocks) {
    out.scalar(block.group_id.value());
    out.scalar(block.kind);
    out.scalar(block.sensor);
    out.scalar(block.role);
    out.matrix(block.jacobian_raw);
    out.matrix(block.residual_raw);
    out.matrix(block.covariance);
    out.matrix(block.whitener);
    out.matrix(block.jacobian_whitened);
    out.matrix(block.residual_whitened);
    out.scalar(static_cast<std::uint64_t>(block.window_column_indices.size()));
    for (const auto value : block.window_column_indices) out.scalar(value);
    out.scalar(static_cast<std::uint64_t>(block.fault_units.size()));
    for (const auto value : block.fault_units) out.scalar(value.value());
    out.scalar(block.effective_weight);
    out.string(block.whitening_model_id);
    canonicalVersion(&out, block.version);
  }
  out.scalar(static_cast<std::uint64_t>(window.slot_accounting.size()));
  for (const auto& slot : window.slot_accounting) {
    out.scalar(slot.slot);
    out.scalar(slot.group_id.has_value());
    if (slot.group_id) out.scalar(slot.group_id->value());
    out.scalar(slot.explicit_window_block);
    out.scalar(slot.boundary_input);
    out.scalar(slot.pointer_identity_valid);
  }
  out.scalar(static_cast<std::uint64_t>(window.factor_inventory.size()));
  for (const auto& entry : window.factor_inventory) {
    out.scalar(entry.group_id.value());
    out.scalar(entry.epoch);
    out.scalar(entry.kind);
    out.scalar(entry.sensor);
    out.scalar(entry.disposition);
    out.scalar(static_cast<std::uint64_t>(entry.keys.size()));
    for (const auto key : entry.keys) out.scalar(key);
    out.scalar(static_cast<std::uint64_t>(entry.slots.size()));
    for (const auto slot : entry.slots) out.scalar(slot);
  }
  out.scalar(window.detector_first_epoch);
  out.scalar(window.recovery_first_epoch);
  out.matrix(window.H);
  out.matrix(window.z);
  out.scalar(window.rank);
  out.scalar(window.dof);
  out.scalar(window.condition_number);
  out.matrix(window.base_information);
  out.matrix(window.base_information_rhs);
  out.matrix(window.protected_state_map);
  const auto& cap = window.capabilities;
  out.scalar(cap.includes_boundary_prior);
  out.scalar(cap.includes_pending_imu);
  out.scalar(cap.includes_pending_uwb);
  out.scalar(cap.complete_factor_provenance);
  out.scalar(cap.history_provenance_valid);
  out.scalar(cap.fixed_lag_maturity_valid);
  out.scalar(cap.no_duplicate_rows);
  out.scalar(cap.every_active_factor_accounted_once);
  out.scalar(cap.frozen_slot_identity_valid);
  out.scalar(cap.history_summary_present);
  out.scalar(cap.history_summary_valid);
  out.scalar(cap.history_summary_capacity_ok);
  const auto& history = window.history_summary;
  out.scalar(history.present);
  out.scalar(history.valid);
  out.string(history.reason);
  out.scalar(history.fault_columns);
  out.scalar(history.boundary_rows);
  out.scalar(history.boundary_columns);
  out.scalar(history.emitted_rows);
  out.scalar(history.rank_boundary);
  out.scalar(history.nu_perp);
  out.scalar(history.kappa_b);
  out.scalar(history.constant_energy);
  out.scalar(history.injected_epochs);
  out.scalar(static_cast<std::uint64_t>(history.column_ids.size()));
  for (const auto& id : history.column_ids) {
    out.scalar(id.kind);
    out.scalar(id.source);
    out.scalar(id.epoch);
  }
  out.scalar(history.constant_columns);
  out.scalar(history.time_linear_columns);
  out.scalar(history.imu_columns);
  out.matrix(history.response);
  out.matrix(history.detector_response);
  out.matrix(history.d_perp);
  out.scalar(history.constant_offset);
  out.scalar(history.information_form_factors);
  out.scalar(history.horizon_first_epoch);
  out.scalar(history.window_first_epoch);
  out.scalar(history.omitted_epoch_count);
  out.scalar(history.material_gap_epoch_count);
  out.scalar(history.claims_full_coverage);
  out.string(history.assumptions);
  out.string(history.omitted_risk_source);
  out.scalar(history.skipped_columns);
  out.string(history.scope_digest);
  out.scalar(history.version.linearization);
  out.scalar(history.version.whitening);
  out.scalar(history.version.mode_set);
  out.scalar(history.version.capacity);
  out.scalar(history.version_digest);
  out.scalar(history.capacity_ok);
  out.string(history.capacity_action);
  out.string(history.state);
  out.scalar(window.model_valid);
  out.string(window.reason);
  // Wall-clock preparation timings are intentionally telemetry rather than
  // semantic payload.  The immutable numerical products are bound by their
  // own typed proof identity below.
  if (window.numerics) {
    out.scalar(true);
    const auto& numerics = *window.numerics;
    out.scalar(numerics.window_id.value());
    canonicalVersion(&out, numerics.version);
    out.scalar(numerics.content_fingerprint);
    out.scalar(numerics.numerical_contract.policy_version);
    out.scalar(numerics.numerical_contract.rank_tolerance);
    out.scalar(numerics.numerical_contract.max_condition_number);
    out.scalar(numerics.numerical_contract.solve_residual_limit);
    out.scalar(numerics.numerical_contract.forward_error_limit);
    out.scalar(numerics.numerical_contract.reference_relative_tolerance);
    out.scalar(numerics.numerical_contract_fingerprint);
    out.scalar(static_cast<bool>(numerics.information_factorization));
    if (numerics.information_factorization) {
      out.matrix(numerics.information_factorization->matrixL().toDenseMatrix());
    }
    out.scalar(static_cast<bool>(numerics.spectral_vectors));
    if (numerics.spectral_vectors) out.matrix(*numerics.spectral_vectors);
    out.matrix(numerics.spectral_inverse_squared);
    out.matrix(numerics.spectral_state_increment);
    out.matrix(numerics.base_state_increment);
    out.matrix(numerics.parity);
    out.scalar(static_cast<std::uint64_t>(numerics.block_row_offsets.size()));
    for (const auto offset : numerics.block_row_offsets) out.scalar(offset);
    out.scalar(numerics.statistic);
    out.scalar(numerics.information_logdet);
    out.scalar(numerics.smallest_singular_value);
    out.scalar(numerics.largest_singular_value);
    out.scalar(numerics.smallest_information_lower_bound);
    out.scalar(numerics.largest_information_upper_bound);
    out.scalar(numerics.solve_relative_residual);
    out.scalar(numerics.normal_equation_forward_error_bound);
    out.scalar(numerics.llt_svd_relative_difference);
    out.scalar(numerics.canonical_spectral_solution);
    out.scalar(numerics.square_root_statistic);
    out.scalar(numerics.square_root_condition_estimate);
    out.scalar(numerics.square_root_detector_only_rows);
    out.scalar(numerics.square_root_certificate_ok);
    out.scalar(numerics.exact_rank);
    out.scalar(numerics.dof);
    out.scalar(numerics.exact_condition);
    out.scalar(numerics.valid);
    out.string(numerics.reason);
  } else {
    out.scalar(false);
  }
  out.scalar(window.square_root ? window.square_root->proofIdentity() : 0);
  return out.bytes();
}

std::uint64_t nextFrozenOwnerToken() {
  static std::atomic<std::uint64_t> next{1};
  return next.fetch_add(1, std::memory_order_relaxed);
}

void hashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= bytes[i];
    *hash *= 1099511628211ULL;
  }
}
template <class Derived>
void hashMatrix(std::uint64_t* hash, const Eigen::MatrixBase<Derived>& matrix) {
  const Eigen::Index rows = matrix.rows(), cols = matrix.cols();
  hashBytes(hash, &rows, sizeof(rows)); hashBytes(hash, &cols, sizeof(cols));
  for (Eigen::Index column = 0; column < cols; ++column)
    for (Eigen::Index row = 0; row < rows; ++row) {
      const double value = matrix(row, column);
      hashBytes(hash, &value, sizeof(value));
    }
}

}  // namespace

std::uint64_t numericalContractFingerprint(double rank_tolerance,
                                           double max_condition_number) {
  FrozenNumericalContract contract;
  contract.rank_tolerance = rank_tolerance;
  contract.max_condition_number = max_condition_number;
  return numericalContractFingerprint(contract);
}

std::uint64_t numericalContractFingerprint(
    const FrozenNumericalContract& contract_values) {
  std::uint64_t contract = 1469598103934665603ULL;
  hashBytes(&contract, &contract_values.policy_version,
            sizeof(contract_values.policy_version));
  hashBytes(&contract, &contract_values.rank_tolerance,
            sizeof(contract_values.rank_tolerance));
  hashBytes(&contract, &contract_values.max_condition_number,
            sizeof(contract_values.max_condition_number));
  hashBytes(&contract, &contract_values.solve_residual_limit,
            sizeof(contract_values.solve_residual_limit));
  hashBytes(&contract, &contract_values.forward_error_limit,
            sizeof(contract_values.forward_error_limit));
  hashBytes(&contract, &contract_values.reference_relative_tolerance,
            sizeof(contract_values.reference_relative_tolerance));
  return contract;
}

std::uint64_t frozenWindowNumericalProofIdentity(
    const LinearizedIntegrityWindow& window,
    const FrozenWindowNumerics& numerics) {
  std::uint64_t proof = 1469598103934665603ULL;
  const std::uint64_t proof_schema_version = 1;
  hashBytes(&proof, &proof_schema_version, sizeof(proof_schema_version));
  hashBytes(&proof, &numerics.content_fingerprint,
            sizeof(numerics.content_fingerprint));
  hashBytes(&proof, &numerics.numerical_contract_fingerprint,
            sizeof(numerics.numerical_contract_fingerprint));
  hashMatrix(&proof, window.H);
  hashMatrix(&proof, window.z);
  hashMatrix(&proof, window.protected_state_map);
  const std::uint64_t square_root_proof = window.square_root
      ? window.square_root->proofIdentity() : 0;
  hashBytes(&proof, &square_root_proof, sizeof(square_root_proof));
  hashBytes(&proof, &numerics.canonical_spectral_solution,
            sizeof(numerics.canonical_spectral_solution));
  hashMatrix(&proof, numerics.spectral_state_increment);
  if (numerics.spectral_vectors) hashMatrix(&proof, *numerics.spectral_vectors);
  else hashMatrix(&proof, Eigen::MatrixXd{});
  hashMatrix(&proof, numerics.spectral_inverse_squared);
  hashMatrix(&proof, numerics.base_state_increment);
  hashMatrix(&proof, numerics.parity);
  hashBytes(&proof, &numerics.statistic, sizeof(numerics.statistic));
  hashBytes(&proof, &numerics.square_root_certificate_ok,
            sizeof(numerics.square_root_certificate_ok));
  return proof;
}

bool validateFrozenWindowNumericalProof(
    const LinearizedIntegrityWindow& window,
    const FrozenWindowNumerics& numerics, std::string* reason) {
  const bool root_served = window.square_root && window.square_root->usable();
  bool served_values_match = false;
  if (root_served &&
      numerics.base_state_increment.size() ==
          window.square_root->baseStateIncrement().size() &&
      numerics.parity.size() == window.square_root->parity().size()) {
    served_values_match = !numerics.canonical_spectral_solution &&
        (numerics.base_state_increment.array() ==
         window.square_root->baseStateIncrement().array()).all() &&
        (numerics.parity.array() == window.square_root->parity().array()).all() &&
        numerics.statistic == window.square_root->statistic();
  } else if (!root_served && window.H.rows() == window.z.size() &&
             window.H.cols() > 0 &&
             numerics.spectral_state_increment.size() == window.H.cols() &&
             numerics.base_state_increment.size() == window.H.cols()) {
    // This is an independent consumer recomputation from the raw frozen H/z.
    // It deliberately does not trust spectral_state_increment or any rehashed
    // stored derivative: changing that state and synchronizing parity/statistic
    // must still fail.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        window.H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd expected_state = svd.solve(window.z);
    const Eigen::VectorXd expected_parity = window.z - window.H * expected_state;
    const double spectral_scale = svd.singularValues().size() > 0
        ? svd.singularValues()(0) : 0.0;
    const double spectral_threshold =
        numerics.numerical_contract.rank_tolerance * spectral_scale;
    Eigen::VectorXd expected_inverse_squared = Eigen::VectorXd::Zero(
        svd.matrixV().cols());
    for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i) {
      if (svd.singularValues()(i) > spectral_threshold) {
        expected_inverse_squared(i) =
            1.0 / (svd.singularValues()(i) * svd.singularValues()(i));
      }
    }
    Eigen::MatrixXd expected_covariance = svd.matrixV() *
        expected_inverse_squared.asDiagonal() * svd.matrixV().transpose();
    Eigen::MatrixXd stored_covariance;
    bool spectral_operator_matches = numerics.spectral_vectors &&
        numerics.spectral_vectors->rows() == window.H.cols() &&
        numerics.spectral_vectors->cols() ==
            numerics.spectral_inverse_squared.size() &&
        numerics.spectral_inverse_squared.allFinite() &&
        numerics.spectral_vectors->allFinite();
    if (spectral_operator_matches) {
      stored_covariance = *numerics.spectral_vectors *
          numerics.spectral_inverse_squared.asDiagonal() *
          numerics.spectral_vectors->transpose();
      const double operator_scale = std::max(expected_covariance.norm(),
                                             stored_covariance.norm());
      spectral_operator_matches = operator_scale > 0.0 &&
          (stored_covariance - expected_covariance).norm() <=
              1024.0 * std::numeric_limits<double>::epsilon() *
              static_cast<double>(std::max(window.H.rows(), window.H.cols())) *
              operator_scale;
    }
    const auto numerically_same = [](const Eigen::VectorXd& actual,
                                     const Eigen::VectorXd& expected) {
      if (actual.size() != expected.size() || !actual.allFinite() ||
          !expected.allFinite()) return false;
      if ((actual.array() == expected.array()).all()) return true;
      const double scale = std::max(actual.norm(), expected.norm());
      return scale > 0.0 && (actual - expected).norm() <=
          512.0 * std::numeric_limits<double>::epsilon() *
          static_cast<double>(std::max<Eigen::Index>(1, actual.size())) * scale;
    };
    served_values_match = numerics.canonical_spectral_solution &&
        spectral_operator_matches &&
        numerically_same(numerics.spectral_state_increment, expected_state) &&
        numerically_same(numerics.base_state_increment, expected_state) &&
        numerically_same(numerics.parity, expected_parity) &&
        std::abs(numerics.statistic - expected_parity.squaredNorm()) <=
          512.0 * std::numeric_limits<double>::epsilon() *
          std::max(std::abs(numerics.statistic),
                   expected_parity.squaredNorm());
  }
  const bool valid = served_values_match &&
      frozenWindowNumericalProofIdentity(window, numerics) != 0;
  if (!valid && reason) {
    *reason = served_values_match
        ? "frozen-window numerical proof identity mismatch"
        : "frozen-window served QR/SVD result does not match its path";
  }
  return valid;
}

Eigen::MatrixXd solveFrozenInformation(const FrozenWindowNumerics& numerics,
                                       const Eigen::MatrixXd& information,
                                       const Eigen::MatrixXd& rhs,
                                       bool* used_spectral_fallback) {
  if (used_spectral_fallback) *used_spectral_fallback = false;
  if (!numerics.information_factorization ||
      information.rows() != information.cols() ||
      rhs.rows() != information.rows()) return Eigen::MatrixXd();
  NumericalWorkCounters::covarianceRhsSolve(
      static_cast<std::uint64_t>(rhs.cols()));
  Eigen::MatrixXd solved = numerics.information_factorization->solve(rhs);
  const double backward = (information * solved - rhs).norm() /
      std::max({1.0, rhs.norm(), information.norm() * solved.norm()});
  const double forward_bound = numerics.exact_condition *
      numerics.exact_condition * backward;
  const double roundoff = 128.0 * std::numeric_limits<double>::epsilon() *
      static_cast<double>(std::max(information.rows(), rhs.cols()));
  if (solved.allFinite() && std::isfinite(forward_bound) &&
      backward <= numerics.numerical_contract.solve_residual_limit + roundoff &&
      forward_bound <= numerics.numerical_contract.forward_error_limit +
                           roundoff) {
    return solved;
  }
  if (!numerics.spectral_vectors ||
      numerics.spectral_inverse_squared.size() != information.rows()) {
    return Eigen::MatrixXd();
  }
  NumericalWorkCounters::spectralRhsSolve(
      static_cast<std::uint64_t>(rhs.cols()));
  if (used_spectral_fallback) *used_spectral_fallback = true;
  const Eigen::MatrixXd fallback = *numerics.spectral_vectors *
      numerics.spectral_inverse_squared.asDiagonal() *
      numerics.spectral_vectors->transpose() * rhs;
  const double fallback_backward = (information * fallback - rhs).norm() /
      std::max({1.0, rhs.norm(), information.norm() * fallback.norm()});
  const double fallback_forward = numerics.exact_condition *
      numerics.exact_condition * fallback_backward;
  if (!fallback.allFinite() || !std::isfinite(fallback_forward) ||
      fallback_backward > numerics.numerical_contract.solve_residual_limit +
                              roundoff ||
      fallback_forward > numerics.numerical_contract.forward_error_limit +
                             roundoff) {
    return Eigen::MatrixXd();
  }
  return fallback;
}

Eigen::MatrixXd solveFrozenInformation(
    const FrozenSquareRootContext* context,
    const FrozenWindowNumerics& numerics,
    const Eigen::MatrixXd& information, const Eigen::MatrixXd& rhs,
    bool* used_spectral_fallback) {
  if (used_spectral_fallback) *used_spectral_fallback = false;
  if (context && context->usable() &&
      rhs.rows() == context->columns()) {
    Eigen::MatrixXd solved = context->informationSolve(rhs);
    const double backward = solved.rows() == rhs.rows()
        ? (information * solved - rhs).norm() /
              std::max({1.0, rhs.norm(), information.norm() * solved.norm()})
        : std::numeric_limits<double>::infinity();
    const double context_condition =
        context->certificate().condition_estimate;
    const double forward = context_condition * context_condition * backward;
    const double roundoff = 128.0 * std::numeric_limits<double>::epsilon() *
        static_cast<double>(std::max(information.rows(), rhs.cols()));
    if (solved.rows() == rhs.rows() && solved.cols() == rhs.cols() &&
        solved.allFinite() && std::isfinite(forward) &&
        backward <= numerics.numerical_contract.solve_residual_limit + roundoff &&
        forward <= numerics.numerical_contract.forward_error_limit + roundoff) {
      return solved;
    }
    NumericalWorkCounters::squareRootFallback();
  } else if (context) {
    NumericalWorkCounters::squareRootFallback();
  }
  return solveFrozenInformation(numerics, information, rhs,
                                used_spectral_fallback);
}

std::uint64_t integrityWindowFingerprint(
    const LinearizedIntegrityWindow& window) {
  NumericalWorkCounters::windowContentHashScan();
  std::uint64_t hash = 1469598103934665603ULL;
  const auto id = window.id.value();
  hashBytes(&hash, &id, sizeof(id));
  hashBytes(&hash, &window.version.graph_version, sizeof(window.version.graph_version));
  hashBytes(&hash, &window.version.ordering_version, sizeof(window.version.ordering_version));
  hashBytes(&hash, &window.version.noise_model_version, sizeof(window.version.noise_model_version));
  hashBytes(&hash, &window.version.linpoint_version, sizeof(window.version.linpoint_version));
  hashMatrix(&hash, window.H); hashMatrix(&hash, window.z);
  hashMatrix(&hash, window.base_information);
  hashMatrix(&hash, window.base_information_rhs);
  hashMatrix(&hash, window.protected_state_map);
  for (const auto& entry : window.state_layout) {
    hashBytes(&hash, &entry.epoch, sizeof(entry.epoch));
    hashBytes(&hash, &entry.column_offset, sizeof(entry.column_offset));
    hashBytes(&hash, &entry.dimension, sizeof(entry.dimension));
    for (const auto key : entry.keys) hashBytes(&hash, &key, sizeof(key));
  }
  for (const auto& slot : window.slot_accounting) {
    hashBytes(&hash, &slot.slot, sizeof(slot.slot));
    const bool has_group = slot.group_id.has_value();
    hashBytes(&hash, &has_group, sizeof(has_group));
    if (slot.group_id) {
      const auto group = slot.group_id->value();
      hashBytes(&hash, &group, sizeof(group));
    }
    hashBytes(&hash, &slot.explicit_window_block,
              sizeof(slot.explicit_window_block));
    hashBytes(&hash, &slot.boundary_input, sizeof(slot.boundary_input));
  }
  for (const auto& entry : window.factor_inventory) {
    const auto group = entry.group_id.value();
    hashBytes(&hash, &group, sizeof(group));
    hashBytes(&hash, &entry.epoch, sizeof(entry.epoch));
    hashBytes(&hash, &entry.kind, sizeof(entry.kind));
    hashBytes(&hash, &entry.sensor, sizeof(entry.sensor));
    hashBytes(&hash, &entry.disposition, sizeof(entry.disposition));
    for (const auto key : entry.keys) hashBytes(&hash, &key, sizeof(key));
    for (const auto slot : entry.slots) hashBytes(&hash, &slot, sizeof(slot));
  }
  for (const auto& block : window.blocks) {
    const auto group = block.group_id.value();
    hashBytes(&hash, &group, sizeof(group));
    hashBytes(&hash, &block.kind, sizeof(block.kind));
    hashBytes(&hash, &block.sensor, sizeof(block.sensor));
    hashBytes(&hash, &block.role, sizeof(block.role));
    hashBytes(&hash, &block.version.graph_version,
              sizeof(block.version.graph_version));
    hashBytes(&hash, &block.version.ordering_version,
              sizeof(block.version.ordering_version));
    hashBytes(&hash, &block.version.noise_model_version,
              sizeof(block.version.noise_model_version));
    hashBytes(&hash, &block.version.linpoint_version,
              sizeof(block.version.linpoint_version));
    hashBytes(&hash, block.whitening_model_id.data(), block.whitening_model_id.size());
    hashMatrix(&hash, block.jacobian_raw);
    hashMatrix(&hash, block.residual_raw);
    hashMatrix(&hash, block.whitener);
    hashMatrix(&hash, block.jacobian_whitened);
    hashMatrix(&hash, block.residual_whitened);
    hashMatrix(&hash, block.covariance);
    hashBytes(&hash, &block.effective_weight, sizeof(block.effective_weight));
    for (const auto column : block.window_column_indices)
      hashBytes(&hash, &column, sizeof(column));
    for (const auto unit : block.fault_units) {
      const auto value = unit.value();
      hashBytes(&hash, &value, sizeof(value));
    }
  }
  // C1-b/C1-c: the condensed boundary IS the summary, so the summary identity
  // is part of the window content.  Without this, a cache entry computed for a
  // different summary version / mode set / horizon would still match the
  // fingerprint of the rows alone (the rows are only the state-supported part
  // of the summary; the response and detector content live in the carrier).
  {
    const auto& history = window.history_summary;
    const bool present = history.present;
    const bool valid = history.valid;
    const bool capacity_ok = history.capacity_ok;
    hashBytes(&hash, &present, sizeof(present));
    hashBytes(&hash, &valid, sizeof(valid));
    hashBytes(&hash, &capacity_ok, sizeof(capacity_ok));
    hashBytes(&hash, &history.version_digest, sizeof(history.version_digest));
    hashBytes(&hash, history.scope_digest.data(), history.scope_digest.size());
    const std::uint64_t components[4] = {history.version.linearization,
                                         history.version.whitening,
                                         history.version.mode_set,
                                         history.version.capacity};
    hashBytes(&hash, components, sizeof(components));
    const std::uint64_t counts[8] = {
        static_cast<std::uint64_t>(history.fault_columns),
        static_cast<std::uint64_t>(history.boundary_rows),
        static_cast<std::uint64_t>(history.emitted_rows),
        static_cast<std::uint64_t>(history.horizon_first_epoch),
        static_cast<std::uint64_t>(history.window_first_epoch),
        static_cast<std::uint64_t>(history.nu_perp),
        static_cast<std::uint64_t>(history.omitted_epoch_count),
        static_cast<std::uint64_t>(history.material_gap_epoch_count)};
    hashBytes(&hash, counts, sizeof(counts));
    hashBytes(&hash, &history.kappa_b, sizeof(history.kappa_b));
    hashBytes(&hash, &history.constant_offset, sizeof(history.constant_offset));
    for (const auto& id : history.column_ids) {
      // HistoryFaultColumnId is a POD triple; hash its fields explicitly so
      // padding cannot make the identity depend on the compiler.
      const std::uint64_t tuple[3] = {static_cast<std::uint64_t>(id.kind),
                                      id.source, id.epoch};
      hashBytes(&hash, tuple, sizeof(tuple));
    }
    hashMatrix(&hash, history.response);
    hashMatrix(&hash, history.detector_response);
    hashMatrix(&hash, history.d_perp);
    hashBytes(&hash, history.assumptions.data(), history.assumptions.size());
    hashBytes(&hash, history.omitted_risk_source.data(),
              history.omitted_risk_source.size());
  }
  return hash;
}

namespace {

FrozenIntegrityWindow sealCompletedWindow(
    LinearizedIntegrityWindow&& builder, std::uint64_t content_hash) {
  const std::uint64_t owner_token = nextFrozenOwnerToken();
  auto payload =
      std::make_shared<LinearizedIntegrityWindow>(std::move(builder));
  auto seal = std::make_shared<FrozenIntegrityWindowSeal>();
  seal->owner_token = owner_token;
  seal->content_hash = content_hash;
  seal->payload = payload;
  if (payload->numerics) {
    auto& identity = seal->numerical_identity;
    identity.window_id = payload->id;
    identity.version = payload->version;
    identity.content_fingerprint = content_hash;
    identity.numerical_contract_fingerprint =
        payload->numerics->numerical_contract_fingerprint;
    identity.proof_identity =
        frozenWindowNumericalProofIdentity(*payload, *payload->numerics);
    identity.valid = identity.proof_identity != 0 &&
        payload->numerics->content_fingerprint == content_hash;
  }
  return FrozenIntegrityWindow{std::move(seal)};
}

}  // namespace

FrozenIntegrityWindow freezeIntegrityWindow(
    LinearizedIntegrityWindow&& builder) {
  const std::uint64_t content_hash = builder.numerics
      ? builder.numerics->content_fingerprint
      : integrityWindowFingerprint(builder);
  return sealCompletedWindow(std::move(builder), content_hash);
}

FrozenIntegrityWindow freezeIntegrityWindowCopy(
    const LinearizedIntegrityWindow& builder) {
  LinearizedIntegrityWindow copy = builder;
  const std::uint64_t content_hash = integrityWindowFingerprint(copy);
  return sealCompletedWindow(std::move(copy), content_hash);
}

FrozenWindowAdmission admitFrozenIntegrityWindow(
    const FrozenIntegrityWindow& handle) {
  NumericalWorkCounters::frozenAdmissionConstantValidation();
  FrozenWindowAdmission out;
  out.owner = handle.seal;
  if (!out.owner) {
    out.reason = "window is not owned by an immutable frozen seal";
    return out;
  }
  const auto& seal = *out.owner;
  if (seal.schema != FrozenIntegrityWindowSeal::kSchema ||
      seal.payload_type != FrozenIntegrityWindowSeal::kPayloadType ||
      seal.owner_token == 0 || !seal.payload) {
    out.owner.reset();
    out.reason = "frozen window seal type/schema/owner is invalid";
    return out;
  }
  const auto& payload = *seal.payload;
  const bool numerics_match = payload.numerics &&
      seal.numerical_identity.valid &&
      seal.numerical_identity.window_id == payload.id &&
      seal.numerical_identity.version == payload.version &&
      seal.numerical_identity.content_fingerprint == seal.content_hash &&
      payload.numerics->content_fingerprint == seal.content_hash;
  if (!numerics_match) {
    out.owner.reset();
    out.reason = "frozen window constant admission identity mismatch";
    return out;
  }
  out.payload = seal.payload.get();
  return out;
}

bool validateFrozenWindowNumericalProof(
    const FrozenWindowAdmission& admission,
    const FrozenWindowNumerics& numerics, std::string* reason) {
  if (!admission) {
    if (reason) *reason = "immutable frozen-window admission is required";
    return false;
  }
  const auto& payload = admission.window();
  const auto& identity = admission.owner->numerical_identity;
  const bool valid = payload.numerics &&
      payload.numerics.get() == &numerics && numerics.valid &&
      identity.valid && identity.window_id == payload.id &&
      identity.version == payload.version &&
      identity.content_fingerprint == admission.owner->content_hash &&
      numerics.content_fingerprint == admission.owner->content_hash &&
      identity.numerical_contract_fingerprint ==
          numerics.numerical_contract_fingerprint &&
      numerics.numerical_contract_fingerprint ==
          numericalContractFingerprint(numerics.numerical_contract) &&
      identity.proof_identity != 0;
  if (!valid && reason) {
    *reason = "frozen-window constant admission identity mismatch";
  }
  return valid;
}

bool frozenWindowHandleMatches(const FrozenWindowAdmission& admission,
                               const FrozenIntegrityWindow& other) {
  if (!admission) return false;
  const FrozenWindowAdmission candidate = admitFrozenIntegrityWindow(other);
  return candidate && admission.ownerToken() == candidate.ownerToken() &&
      &admission.window() == &candidate.window();
}

bool frozenWindowCanonicalEqual(const FrozenIntegrityWindowSeal& left,
                                const FrozenIntegrityWindowSeal& right) {
  if (!left.payload || !right.payload ||
      left.schema != FrozenIntegrityWindowSeal::kSchema ||
      right.schema != FrozenIntegrityWindowSeal::kSchema ||
      left.payload_type != FrozenIntegrityWindowSeal::kPayloadType ||
      right.payload_type != FrozenIntegrityWindowSeal::kPayloadType ||
      left.content_hash != right.content_hash) {
    return false;
  }
  if (left.payload.get() == right.payload.get()) return true;
  return canonicalWindowPayload(*left.payload) ==
      canonicalWindowPayload(*right.payload);
}

void finalizeIntegrityWindow(LinearizedIntegrityWindow* window,
                             double rank_tolerance,
                             double max_condition_number) {
  if (!window) throw std::invalid_argument("window must not be null");
  auto timing_mark = std::chrono::steady_clock::now();
  auto elapsed_ms = [&]() {
    const auto now = std::chrono::steady_clock::now();
    const double value =
        std::chrono::duration<double, std::milli>(now - timing_mark).count();
    timing_mark = now;
    return value;
  };
  window->numerics.reset();
  window->square_root.reset();
  int rows = 0;
  int columns = 0;
  std::set<std::uint64_t> group_ids;
  bool versions_match = true;
  for (const auto& block : window->blocks) {
    if (block.jacobian_whitened.rows() != block.residual_whitened.size() ||
        !block.jacobian_whitened.allFinite() ||
        !block.residual_whitened.allFinite() ||
        !std::isfinite(block.effective_weight) || block.effective_weight <= 0.0) {
      window->model_valid = false;
      window->reason = "invalid or non-finite linearized factor block";
      return;
    }
    rows += static_cast<int>(block.jacobian_whitened.rows());
    columns = std::max(columns, static_cast<int>(block.jacobian_whitened.cols()));
    versions_match = versions_match && block.version == window->version;
    group_ids.insert(block.group_id.value());
  }
  if (!versions_match) {
    window->model_valid = false;
    window->reason = "linearization version mismatch within frozen window";
    return;
  }
  window->H = Eigen::MatrixXd::Zero(rows, columns);
  window->z = Eigen::VectorXd::Zero(rows);
  int offset = 0;
  for (const auto& block : window->blocks) {
    const int n = static_cast<int>(block.jacobian_whitened.rows());
    window->H.block(offset, 0, n, block.jacobian_whitened.cols()) =
        block.jacobian_whitened;
    window->z.segment(offset, n) = block.residual_whitened;
    offset += n;
  }
  window->capabilities.no_duplicate_rows =
      group_ids.size() == window->blocks.size();
  window->preparation_timing.dense_assembly_ms = elapsed_ms();  if (rows == 0 || columns == 0) {
    window->model_valid = false;
    window->reason = "empty integrity window";
    return;
  }
  NumericalWorkCounters::baseSvd();
  Eigen::VectorXd singular;
  Eigen::MatrixXd spectral_vectors;
  Eigen::VectorXd spectral_increment;
  bool svd_success = false;
  if (std::getenv("UWB_IMU_PL_DISABLE_WINDOW_BDCSVD")) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        window->H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    singular = svd.singularValues();
    spectral_vectors = svd.matrixV();
    spectral_increment = svd.solve(window->z);
  } else {
    // BDCSVD is a direct Jacobian-space SVD, not a normal-equation shortcut.
    // It preserves the rank/condition contract while avoiding the much higher
    // cost of Jacobi sweeps on the common ~500x315 frozen window.  Near-gate
    // detector checks remain independently protected by the existing QR path.
    Eigen::BDCSVD<Eigen::MatrixXd> svd(
        window->H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    singular = svd.singularValues();
    spectral_vectors = svd.matrixV();
    spectral_increment = svd.solve(window->z);
  }
  // Eigen 3.3 SVD classes expose no computation-info flag.  Treat incomplete
  // dimensions or any non-finite factor/solution as an explicit failure.
  svd_success = singular.size() == std::min(window->H.rows(), window->H.cols()) &&
      spectral_vectors.rows() == window->H.cols() &&
      spectral_vectors.cols() == singular.size() && singular.allFinite() &&
      spectral_vectors.allFinite() && spectral_increment.allFinite();
  window->preparation_timing.svd_ms = elapsed_ms();
  if (!svd_success) {
    window->model_valid = false;
    window->reason = "window Jacobian SVD failed";
    return;
  }
  const double scale = singular.size() ? singular(0) : 0.0;
  const double threshold = rank_tolerance * scale;
  window->rank = static_cast<int>((singular.array() > threshold).count());
  window->dof = rows - window->rank;
  const double smallest = window->rank > 0 ? singular(window->rank - 1) : 0.0;
  window->condition_number = smallest > 0.0 ? scale / smallest
                                             : std::numeric_limits<double>::infinity();
  window->base_information.noalias() = window->H.transpose() * window->H;
  window->base_information_rhs.noalias() = window->H.transpose() * window->z;
  window->preparation_timing.normal_equations_ms = elapsed_ms();
  auto numerics = std::make_shared<FrozenWindowNumerics>();
  numerics->window_id = window->id;
  numerics->version = window->version;
  numerics->numerical_contract.rank_tolerance = rank_tolerance;
  numerics->numerical_contract.max_condition_number = max_condition_number;
  numerics->smallest_singular_value = smallest;
  numerics->largest_singular_value = scale;
  numerics->exact_rank = window->rank;
  numerics->dof = window->dof;
  numerics->exact_condition = window->condition_number;
  numerics->spectral_vectors =
      std::make_shared<const Eigen::MatrixXd>(std::move(spectral_vectors));
  numerics->spectral_inverse_squared = Eigen::VectorXd::Zero(columns);
  for (int i = 0; i < window->rank; ++i) {
    numerics->spectral_inverse_squared(i) =
        1.0 / (singular(i) * singular(i));
  }
  const double singular_error = 128.0 * std::numeric_limits<double>::epsilon() *
      std::max(window->H.rows(), window->H.cols()) * scale;
  numerics->smallest_information_lower_bound =
      std::pow(std::max(0.0, smallest - singular_error), 2);
  numerics->largest_information_upper_bound = std::pow(scale + singular_error, 2);
  NumericalWorkCounters::baseLlt();
  auto factorization =
      std::make_shared<Eigen::LLT<Eigen::MatrixXd>>(window->base_information);
  numerics->information_factorization = factorization;
  if (factorization->info() == Eigen::Success) {
    NumericalWorkCounters::baseStateSolve();
    NumericalWorkCounters::lltStateSolve();
    const Eigen::VectorXd llt_increment =
        factorization->solve(window->base_information_rhs);
    // The SVD is already required for the rank/condition contract.  Reuse it
    // as the canonical least-squares reference rather than trusting a normal-
    // equation residual whose condition number is squared.
    NumericalWorkCounters::baseStateSolve();
    NumericalWorkCounters::svdStateSolve();
    numerics->spectral_state_increment = spectral_increment;
    numerics->base_state_increment = numerics->spectral_state_increment;
    numerics->parity = window->z - window->H * numerics->base_state_increment;
    numerics->statistic = numerics->parity.squaredNorm();
    numerics->information_logdet = 2.0 * singular.head(window->rank)
        .array().log().sum();
    numerics->canonical_spectral_solution = true;
    const Eigen::VectorXd normal_residual = window->base_information_rhs -
        window->base_information * llt_increment;
    numerics->solve_relative_residual = normal_residual.norm() /
        std::max(1.0, window->base_information_rhs.norm());
    numerics->normal_equation_forward_error_bound =
        window->condition_number * window->condition_number *
        numerics->solve_relative_residual;
    numerics->llt_svd_relative_difference =
        (llt_increment - numerics->base_state_increment).norm() /
        std::max(1.0, numerics->base_state_increment.norm());
  } else {
    numerics->reason = "base information is not SPD";
  }
  Eigen::Index row_offset = 0;
  for (const auto& block : window->blocks) {
    numerics->block_row_offsets.push_back(row_offset);
    row_offset += block.jacobian_whitened.rows();
  }
  numerics->block_row_offsets.push_back(row_offset);
  window->preparation_timing.llt_and_state_solves_ms = elapsed_ms();

  // B1: build the single square-root context of this frozen window.  It owns
  // the canonical nominal increment and parity statistic; the SVD/LLT results
  // above remain the certificate reference and the reference fallback.
  window->square_root = FrozenSquareRootContext::build(
      window->H, window->z, window->protected_state_map,
      ColumnScalePolicy::Unit, ColumnPermutationPolicy::Natural,
      rank_tolerance, numerics->spectral_state_increment,
      numerics->statistic);
  if (window->square_root) {
    numerics->square_root_statistic = window->square_root->statistic();
    numerics->square_root_condition_estimate =
        window->square_root->certificate().condition_estimate;
    numerics->square_root_detector_only_rows =
        window->square_root->detectorOnlyRows();
    numerics->square_root_certificate_ok = window->square_root->usable();
    if (window->square_root->usable()) {
      numerics->base_state_increment = window->square_root->baseStateIncrement();
      numerics->parity = window->square_root->parity();
      numerics->statistic = window->square_root->statistic();
      numerics->canonical_spectral_solution = false;
    }
  }
  window->model_valid = window->dof > 0 && window->rank == columns &&
      window->condition_number <= max_condition_number &&
      window->capabilities.no_duplicate_rows &&
      factorization->info() == Eigen::Success &&
      numerics->base_state_increment.allFinite() &&
      numerics->parity.allFinite() &&
      numerics->solve_relative_residual <=
          numerics->numerical_contract.solve_residual_limit &&
      std::isfinite(numerics->normal_equation_forward_error_bound);
  if (!window->model_valid) {
    window->reason = window->rank != columns ? "window state is rank deficient" :
        (window->dof <= 0 ? "window has no residual degrees of freedom" :
         (window->condition_number > max_condition_number
              ? "window condition number exceeds gate"
              : (!window->capabilities.no_duplicate_rows
                    ? "window contains duplicate factor groups"
                    : (factorization->info() != Eigen::Success
                        ? "window information is not SPD"
                        : "base solve residual exceeds numerical contract"))));
  } else {
    window->reason.clear();
  }
  numerics->valid = window->model_valid;
  numerics->reason = window->reason;
  numerics->content_fingerprint = integrityWindowFingerprint(*window);
  numerics->numerical_contract_fingerprint =
      numericalContractFingerprint(numerics->numerical_contract);
  NumericalWorkCounters::frozenIdentityBuild();
  window->preparation_timing.fingerprint_ms = elapsed_ms();
  window->numerics = std::move(numerics);
}

}  // namespace uwb_imu_pl
