#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <Eigen/SVD>

#include <set>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

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
      std::max(1.0, rhs.norm());
  const double forward_bound = numerics.exact_condition *
      numerics.exact_condition * backward;
  if (solved.allFinite() && std::isfinite(forward_bound) &&
      forward_bound <= numerics.numerical_contract.forward_error_limit) {
    return solved;
  }
  if (!numerics.spectral_vectors ||
      numerics.spectral_inverse_squared.size() != information.rows()) {
    return Eigen::MatrixXd();
  }
  NumericalWorkCounters::spectralRhsSolve(
      static_cast<std::uint64_t>(rhs.cols()));
  if (used_spectral_fallback) *used_spectral_fallback = true;
  return *numerics.spectral_vectors *
      numerics.spectral_inverse_squared.asDiagonal() *
      numerics.spectral_vectors->transpose() * rhs;
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
    if (solved.rows() == rhs.rows() && solved.cols() == rhs.cols() &&
        solved.allFinite()) {
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
  return hash;
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
  const double threshold = rank_tolerance * std::max(1.0, scale);
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
      std::max(window->H.rows(), window->H.cols()) * std::max(1.0, scale);
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
  window->preparation_timing.fingerprint_ms = elapsed_ms();
  window->numerics = std::move(numerics);
}

}  // namespace uwb_imu_pl
