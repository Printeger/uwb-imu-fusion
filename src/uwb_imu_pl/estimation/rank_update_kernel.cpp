#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <cstdlib>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <chrono>
#include <cstring>
#include <set>
#include <type_traits>

namespace uwb_imu_pl {
namespace {

std::atomic<std::uint64_t> g_candidate_route_rank{0};
std::atomic<std::uint64_t> g_candidate_route_dense{0};
std::atomic<std::uint64_t> g_candidate_route_rank_exception{0};

bool groupSelected(FactorGroupId id, const std::vector<FactorGroupId>& ids) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

template<class A, class B> bool sameMatrix(const A& a, const B& b) {
  return a.rows() == b.rows() && a.cols() == b.cols() && (a.array() == b.array()).all();
}
bool sameBlock(const LinearizedFactorBlock& a, const LinearizedFactorBlock& b) {
  return a.group_id == b.group_id && a.version == b.version && a.kind == b.kind &&
      a.sensor == b.sensor && a.role == b.role && a.effective_weight == b.effective_weight &&
      a.whitening_model_id == b.whitening_model_id && a.fault_units == b.fault_units &&
      a.window_column_indices == b.window_column_indices &&
      sameMatrix(a.jacobian_whitened, b.jacobian_whitened) &&
      sameMatrix(a.residual_whitened, b.residual_whitened) &&
      sameMatrix(a.jacobian_raw, b.jacobian_raw) && sameMatrix(a.residual_raw, b.residual_raw) &&
      sameMatrix(a.covariance, b.covariance) && sameMatrix(a.whitener, b.whitener);
}

struct SpectralCertificate {
  bool spd = false;
  bool full_rank = false;
  bool condition_passed = false;
  double lambda_min_lower = 0.0;
  double lambda_max_upper = std::numeric_limits<double>::infinity();
  double condition_lower = 1.0;
  double condition_upper = std::numeric_limits<double>::infinity();
  double margin = -std::numeric_limits<double>::infinity();
};

SpectralCertificate certifyUpdate(const FrozenWindowNumerics& base,
                                  const Eigen::MatrixXd& update_columns,
                                  const Eigen::VectorXd& signs,
                                  double rank_tolerance,
                                  double max_condition,
                                  RankUpdateScratch* scratch) {
  SpectralCertificate out;
  if (update_columns.cols() == 0) {
    out.spd = base.smallest_information_lower_bound > 0.0;
    out.lambda_min_lower = base.smallest_information_lower_bound;
    out.lambda_max_upper = base.largest_information_upper_bound;
  } else {
    if (!base.information_factorization || update_columns.cols() > update_columns.rows() ||
        signs.size() != update_columns.cols()) return out;
    RankUpdateScratch local;
    RankUpdateScratch& work = scratch ? *scratch : local;
    work.whitened_update = base.information_factorization->matrixL().solve(update_columns);
    if (!work.whitened_update.allFinite()) return out;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(work.whitened_update);
    const Eigen::Index count = update_columns.cols();
    const Eigen::MatrixXd r = qr.matrixQR().topRows(count)
        .template triangularView<Eigen::Upper>();
    work.small_symmetric = Eigen::MatrixXd::Identity(count, count);
    work.small_symmetric.noalias() += r * signs.asDiagonal() * r.transpose();
    work.small_symmetric = 0.5 * (work.small_symmetric + work.small_symmetric.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(work.small_symmetric,
                                                         Eigen::EigenvaluesOnly);
    if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite()) return out;
    const double eps = std::numeric_limits<double>::epsilon();
    const double error = 128.0 * eps * std::max<Eigen::Index>(1, count) *
        std::max(1.0, work.small_symmetric.norm());
    const double whitened_min = std::min(1.0, eigen.eigenvalues().minCoeff() - error);
    const double whitened_max = std::max(1.0, eigen.eigenvalues().maxCoeff() + error);
    out.spd = whitened_min > 64.0 * eps;
    if (!out.spd) return out;
    out.lambda_min_lower = base.smallest_information_lower_bound * whitened_min;
    out.lambda_max_upper = base.largest_information_upper_bound * whitened_max;
  }
  if (!(out.lambda_min_lower > 0.0) || !std::isfinite(out.lambda_max_upper)) return out;
  const double singular_min_lower = std::sqrt(out.lambda_min_lower);
  const double singular_max_upper = std::sqrt(out.lambda_max_upper);
  out.condition_upper = singular_max_upper / singular_min_lower;
  // The lower bound is diagnostic only; one is always valid for an SPD matrix.
  out.condition_lower = 1.0;
  const double rank_gate = rank_tolerance * singular_max_upper;
  out.full_rank = singular_min_lower > rank_gate * (1.0 + 1e-7);
  out.condition_passed = out.condition_upper < max_condition * (1.0 - 1e-7);
  out.margin = std::min(singular_min_lower - rank_gate,
                        max_condition - out.condition_upper);
  return out;
}

// A conservative a-posteriori step certificate. QR avoids normal-equation
// conditioning. Bound the inverse of R through its checked residual, then
// bound least-squares solution error through the final normal residual.
// Uncertain bounds (including near gates) always continue to the exact SVD.
bool certifiedLargeStep(const Eigen::MatrixXd& h, const Eigen::VectorXd& z,
                        double gate, Eigen::VectorXd* delta) {
  if (h.rows() < h.cols() || h.cols() == 0) return false;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(h);
  if (qr.rank() != h.cols()) return false;
  *delta = qr.solve(z);
  if (!delta->allFinite() || delta->norm() <= gate * (1.0 + 1e-7)) return false;
  const Eigen::MatrixXd r = qr.matrixR().topRows(h.cols()).template triangularView<Eigen::Upper>();
  // Stream triangular solves to bound ||R^-1||_F and its residual without
  // constructing an explicit inverse (including a triangular full inverse).
  double inverse_squared_norm = 0.0, inverse_residual_squared = 0.0;
  Eigen::VectorXd unit = Eigen::VectorXd::Zero(h.cols());
  for (Eigen::Index column=0; column<h.cols(); ++column) {
    unit(column) = 1.0;
    const Eigen::VectorXd solved = r.triangularView<Eigen::Upper>().solve(unit);
    inverse_squared_norm += solved.squaredNorm();
    inverse_residual_squared += (r.triangularView<Eigen::Upper>() * solved - unit).squaredNorm();
    unit(column) = 0.0;
  }
  const double inverse_norm = std::sqrt(inverse_squared_norm);
  const double eps = std::numeric_limits<double>::epsilon();
  const double gamma = 64.0 * eps * std::max(h.rows(), h.cols());
  const double inverse_residual = std::sqrt(inverse_residual_squared) + gamma * r.norm() * inverse_norm;
  if (!std::isfinite(inverse_residual) || inverse_residual >= 0.5) return false;
  const double sigma_lower = (1.0 - inverse_residual) / inverse_norm - gamma * h.norm();
  if (!(sigma_lower > 0)) return false;
  // Accumulate the a-posteriori normal residual in extended precision. A
  // double-precision rounding bound on H^T(z-Hx) is often inconclusive for
  // the mixed IMU/prior scales, even when the QR solution itself is accurate.
  Eigen::Matrix<long double, Eigen::Dynamic, 1> gradient =
      Eigen::Matrix<long double, Eigen::Dynamic, 1>::Zero(h.cols());
  for (Eigen::Index i=0; i<h.rows(); ++i) {
    long double residual = z(i);
    for (Eigen::Index j=0; j<h.cols(); ++j)
      residual -= static_cast<long double>(h(i,j)) * (*delta)(j);
    for (Eigen::Index j=0; j<h.cols(); ++j)
      gradient(j) += static_cast<long double>(h(i,j)) * residual;
  }
  const double gradient_gamma = 64.0 * std::numeric_limits<long double>::epsilon() *
      std::max(h.rows(), h.cols());
  const double gradient_upper = static_cast<double>(gradient.norm()) +
      gradient_gamma * h.norm() * (z.norm() + h.norm() * delta->norm());
  const double error_bound = gradient_upper / (sigma_lower * sigma_lower);
  return std::isfinite(error_bound) && delta->norm() - error_bound > gate * (1.0 + 1e-7);
}

bool validBlock(const LinearizedFactorBlock& b, const LinearizedIntegrityWindow& w) {
  const auto rows = b.jacobian_whitened.rows();
  const auto square_or_empty = [&](const Eigen::MatrixXd& m) {
    return m.size() == 0 || (m.rows() == rows && m.cols() == rows && m.allFinite());
  };
  return b.version == w.version && b.jacobian_whitened.cols() == w.H.cols() &&
      rows == b.residual_whitened.size() && b.jacobian_whitened.allFinite() &&
      b.residual_whitened.allFinite() && std::isfinite(b.effective_weight) &&
      (b.jacobian_raw.size() == 0 || (b.jacobian_raw.rows() == rows &&
        b.jacobian_raw.cols() == w.H.cols() && b.jacobian_raw.allFinite())) &&
      (b.residual_raw.size() == 0 || (b.residual_raw.size() == rows && b.residual_raw.allFinite())) &&
      square_or_empty(b.covariance) && square_or_empty(b.whitener) &&
      std::all_of(b.window_column_indices.begin(), b.window_column_indices.end(),
                  [&](int i) { return i >= 0 && i < w.H.cols(); });
}

bool validateAction(const LinearizedIntegrityWindow& w, const ExclusionAction& a, std::string* reason) {
  std::set<std::uint64_t> requested, resolved;
  for (auto id : a.groups_to_remove) {
    if (!requested.insert(id.value()).second) { *reason = "duplicate candidate removal"; return false; }
  }
  for (const auto& b : w.blocks) {
    if (!validBlock(b, w)) { *reason = "candidate frozen block/version is invalid"; return false; }
    if (requested.count(b.group_id.value())) resolved.insert(b.group_id.value());
  }
  if (requested != resolved) { *reason = "candidate removal block is absent from frozen window"; return false; }
  std::set<std::uint64_t> additions;
  for (const auto& b : a.added_blocks) {
    if (!validBlock(b, w)) { *reason = "candidate addition block/version is invalid"; return false; }
    // Multiple factor blocks per group are legal; overlapping an active group
    // is only legal when that whole group is removed in this same action.
    for (const auto& old : w.blocks)
      if (old.group_id == b.group_id && !requested.count(b.group_id.value())) {
        *reason = "candidate addition duplicates retained group"; return false;
      }
  }
  return true;
}

bool candidateRows(const LinearizedIntegrityWindow& window,
                   const ExclusionAction& action, Eigen::MatrixXd* h,
                   Eigen::VectorXd* z) {
  std::set<std::uint64_t> requested;
  std::set<std::uint64_t> resolved;
  for (const auto id : action.groups_to_remove) requested.insert(id.value());
  int rows = 0;
  for (const auto& block : window.blocks) {
    if (groupSelected(block.group_id, action.groups_to_remove)) {
      resolved.insert(block.group_id.value());
    } else {
      rows += block.jacobian_whitened.rows();
    }
  }
  if (requested != resolved) return false;
  for (const auto& block : action.added_blocks) rows += block.jacobian_whitened.rows();
  h->setZero(rows, window.H.cols());
  z->setZero(rows);
  int offset = 0;
  auto append = [&](const LinearizedFactorBlock& block) {
    const int n = block.jacobian_whitened.rows();
    h->block(offset, 0, n, block.jacobian_whitened.cols()) = block.jacobian_whitened;
    z->segment(offset, n) = block.residual_whitened;
    offset += n;
  };
  for (const auto& block : window.blocks) {
    if (!groupSelected(block.group_id, action.groups_to_remove)) append(block);
  }
  for (const auto& block : action.added_blocks) append(block);
  return true;
}

void hashCertificateBytes(std::uint64_t* hash, const void* data,
                          std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= bytes[index];
    *hash *= 1099511628211ULL;
  }
}

template <class T>
void hashCertificateScalar(std::uint64_t* hash, const T& value) {
  hashCertificateBytes(hash, &value, sizeof(value));
}

void hashCertificateString(std::uint64_t* hash, const std::string& value) {
  const std::uint64_t size = value.size();
  hashCertificateScalar(hash, size);
  if (!value.empty()) hashCertificateBytes(hash, value.data(), value.size());
}

template <class Derived>
void hashCertificateEigen(std::uint64_t* hash,
                          const Eigen::MatrixBase<Derived>& value) {
  const std::int64_t rows = value.rows();
  const std::int64_t cols = value.cols();
  hashCertificateScalar(hash, rows);
  hashCertificateScalar(hash, cols);
  for (Eigen::Index col = 0; col < value.cols(); ++col) {
    for (Eigen::Index row = 0; row < value.rows(); ++row) {
      const double entry = value(row, col) == 0.0 ? 0.0 : value(row, col);
      hashCertificateScalar(hash, entry);
    }
  }
}

std::uint64_t candidateNumericalProofIdentityImpl(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate) {
  std::uint64_t proof = 1469598103934665603ULL;
  const std::uint64_t parent = window.numerics
      ? frozenWindowNumericalProofIdentity(window, *window.numerics) : 0;
  hashCertificateScalar(&proof, parent);
  hashCertificateScalar(&proof,
                        candidateDetectorActionIdentity(candidate.action));
  hashCertificateString(&proof, candidate.diagnostics.numerical_path);
  hashCertificateScalar(&proof, candidate.exact_slow_path);
  hashCertificateScalar(&proof, candidate.valid);
  hashCertificateScalar(&proof, candidate.rows);
  hashCertificateScalar(&proof, candidate.rank);
  hashCertificateScalar(&proof, candidate.dof);
  hashCertificateScalar(&proof, candidate.statistic);
  hashCertificateScalar(&proof, candidate.condition_number);
  hashCertificateScalar(&proof, candidate.information_logdet);
  hashCertificateEigen(&proof, candidate.state_increment);
  hashCertificateEigen(&proof, candidate.covariance);
  hashCertificateEigen(&proof, candidate.reference_inverse_squared);
  hashCertificateEigen(&proof, candidate.covariance_plus_factor);
  hashCertificateEigen(&proof, candidate.covariance_minus_factor);
  return proof;
}

void hashCertificateVersion(std::uint64_t* hash,
                            const LinearizationVersion& version) {
  hashCertificateScalar(hash, version.graph_version);
  hashCertificateScalar(hash, version.ordering_version);
  hashCertificateScalar(hash, version.noise_model_version);
  hashCertificateScalar(hash, version.linpoint_version);
}

std::uint64_t blockCertificateIdentity(const LinearizedFactorBlock& block) {
  std::uint64_t hash = 1469598103934665603ULL;
  hashCertificateScalar(&hash, block.group_id.value());
  hashCertificateScalar(&hash, static_cast<int>(block.kind));
  hashCertificateScalar(&hash, static_cast<int>(block.sensor));
  hashCertificateScalar(&hash, static_cast<int>(block.role));
  hashCertificateEigen(&hash, block.jacobian_raw);
  hashCertificateEigen(&hash, block.residual_raw);
  hashCertificateEigen(&hash, block.covariance);
  hashCertificateEigen(&hash, block.whitener);
  hashCertificateEigen(&hash, block.jacobian_whitened);
  hashCertificateEigen(&hash, block.residual_whitened);
  hashCertificateScalar(
      &hash, static_cast<std::uint64_t>(block.window_column_indices.size()));
  for (const int index : block.window_column_indices) {
    hashCertificateScalar(&hash, index);
  }
  std::vector<std::uint64_t> fault_units;
  for (const auto id : block.fault_units) fault_units.push_back(id.value());
  std::sort(fault_units.begin(), fault_units.end());
  hashCertificateScalar(&hash,
                        static_cast<std::uint64_t>(fault_units.size()));
  for (const auto value : fault_units) hashCertificateScalar(&hash, value);
  hashCertificateScalar(&hash, block.effective_weight);
  hashCertificateString(&hash, block.whitening_model_id);
  hashCertificateVersion(&hash, block.version);
  return hash;
}

CandidateDetectorCertificate buildCandidateDetectorCertificate(
    const LinearizedIntegrityWindow& window, const ExclusionAction& action,
    const CandidateEvaluation& candidate, double rank_tolerance) {
  CandidateDetectorCertificate out;
  out.accepted_event_id = "dual_channel_intersection_v6";
  out.pooled_rank = candidate.rank;
  out.pooled_dof = candidate.dof;
  if (!candidate.valid || !candidate.state_increment.allFinite()) {
    out.reason = "candidate numerical solution is not valid";
    return out;
  }

  std::vector<const LinearizedFactorBlock*> retained;
  retained.reserve(window.blocks.size() + action.added_blocks.size());
  for (const auto& block : window.blocks) {
    if (!groupSelected(block.group_id, action.groups_to_remove)) {
      retained.push_back(&block);
    }
  }
  for (const auto& block : action.added_blocks) retained.push_back(&block);

  int history_block_count = 0;
  int current_rows = 0;
  int history_rows = 0;
  for (const auto* block : retained) {
    const bool history_block =
        block->whitening_model_id == "history_summary_sqrt_d1";
    if (history_block) ++history_block_count;
    int block_history_rows = 0;
    if (history_block) {
      const bool is_added = std::any_of(
          action.added_blocks.begin(), action.added_blocks.end(),
          [&](const LinearizedFactorBlock& added) { return &added == block; });
      if (is_added) {
        out.reason = "added history summary has no bound constant payload";
        return out;
      }
      if (!window.history_summary.present || !window.history_summary.valid ||
          window.history_summary.nu_perp < 0 ||
          window.history_summary.nu_perp > block->jacobian_whitened.rows() ||
          !std::isfinite(window.history_summary.constant_offset)) {
        out.reason = "history summary constant/row-role payload is missing";
        return out;
      }
      block_history_rows = window.history_summary.nu_perp;
      out.history_constant = window.history_summary.constant_offset;
      out.history_constant_present = true;
    }
    current_rows += block->jacobian_whitened.rows() - block_history_rows;
    history_rows += block_history_rows;
  }
  const bool frozen_history_block = std::any_of(
      window.blocks.begin(), window.blocks.end(),
      [](const LinearizedFactorBlock& block) {
        return block.whitening_model_id == "history_summary_sqrt_d1";
      });
  if (history_block_count > 1) {
    out.reason = "candidate contains multiple history summary blocks";
    return out;
  }
  if ((window.history_summary.present || frozen_history_block) &&
      history_block_count == 0) {
    out.reason = "candidate removed or omitted the frozen history payload";
    return out;
  }
  if (history_block_count == 0) {
    out.history_constant = 0.0;
    out.history_constant_present = true;
  }

  Eigen::MatrixXd current_h(current_rows, window.H.cols());
  Eigen::MatrixXd history_h(history_rows, window.H.cols());
  Eigen::VectorXd current_r(current_rows);
  Eigen::VectorXd history_r(history_rows);
  int current_offset = 0;
  int history_offset = 0;
  out.row_roles.reserve(static_cast<std::size_t>(current_rows + history_rows));
  for (const auto* block : retained) {
    const int rows = block->jacobian_whitened.rows();
    const int block_history_rows =
        block->whitening_model_id == "history_summary_sqrt_d1"
            ? window.history_summary.nu_perp
            : 0;
    const int supported_rows = rows - block_history_rows;
    if (supported_rows > 0) {
      current_h.middleRows(current_offset, supported_rows) =
          block->jacobian_whitened.topRows(supported_rows);
      current_r.segment(current_offset, supported_rows) =
          block->residual_whitened.head(supported_rows) -
          block->jacobian_whitened.topRows(supported_rows) *
              candidate.state_increment;
      current_offset += supported_rows;
      out.row_roles.insert(out.row_roles.end(), supported_rows,
                           CandidateDetectorRowRole::CurrentStateSupported);
    }
    if (block_history_rows > 0) {
      history_h.middleRows(history_offset, block_history_rows) =
          block->jacobian_whitened.bottomRows(block_history_rows);
      history_r.segment(history_offset, block_history_rows) =
          block->residual_whitened.tail(block_history_rows) -
          block->jacobian_whitened.bottomRows(block_history_rows) *
              candidate.state_increment;
      history_offset += block_history_rows;
      out.row_roles.insert(out.row_roles.end(), block_history_rows,
                           CandidateDetectorRowRole::HistoryDetectorOnly);
    }
  }
  auto rankOf = [&](const Eigen::MatrixXd& value) {
    if (value.rows() == 0 || value.cols() == 0) return 0;
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(value);
    qr.setThreshold(rank_tolerance);
    return static_cast<int>(qr.rank());
  };
  out.current_rows = current_rows;
  out.current_rank = rankOf(current_h);
  out.current_dof = current_rows - out.current_rank;
  out.history_rows = history_rows;
  out.history_rank = rankOf(history_h);
  out.history_dof = history_rows - out.history_rank;
  out.current_statistic = current_r.squaredNorm();
  out.history_statistic = history_r.squaredNorm() + out.history_constant;
  const double represented_without_constant =
      out.current_statistic + history_r.squaredNorm();
  const double scale = std::max({1.0, std::abs(candidate.statistic),
                                 std::abs(represented_without_constant)});
  if (!out.history_constant_present || !std::isfinite(out.current_statistic) ||
      !std::isfinite(out.history_statistic) || out.history_statistic < 0.0 ||
      out.current_dof <= 0 || out.current_rank != candidate.rank ||
      out.current_dof + out.history_dof != candidate.dof ||
      out.current_rows + out.history_rows != candidate.rows ||
      out.history_rank != 0 ||
      (out.history_rows > 0 && out.history_dof <= 0) ||
      std::abs(represented_without_constant - candidate.statistic) >
          1e-7 * scale) {
    out.reason = "candidate detector statistics/rank/dof identity is invalid";
    return out;
  }

  out.window_fingerprint = integrityWindowFingerprint(window);
  out.numerical_contract_identity = window.numerics
      ? window.numerics->numerical_contract_fingerprint : 0;
  out.canonical_action_identity = candidateDetectorActionIdentity(action);
  out.numerical_identity = candidateDetectorNumericalIdentity(window, action);
  out.valid = true;
  out.certificate_digest = candidateDetectorCertificateDigest(
      window, candidate, out);
  return out;
}

bool covarianceFromInformation(const Eigen::MatrixXd& information,
                               Eigen::MatrixXd* covariance) {
  NumericalWorkCounters::candidateInnerLlt();
  Eigen::LLT<Eigen::MatrixXd> llt(information);
  if (llt.info() != Eigen::Success) return false;
  covariance->setIdentity(information.rows(), information.cols());
  *covariance = llt.solve(*covariance);
  *covariance = 0.5 * (*covariance + covariance->transpose());
  return covariance->allFinite();
}

void finishCandidate(CandidateEvaluation* result, const Eigen::MatrixXd& h,
                     const Eigen::VectorXd& z, double rank_tolerance,
                     double max_condition, double max_step) {
  NumericalWorkCounters::candidateReferenceSvd();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(h);
  const auto s = svd.singularValues();
  const double largest = s.size() ? s(0) : 0.0;
  const double threshold = rank_tolerance * largest;
  result->rank = static_cast<int>((s.array() > threshold).count());
  result->rows = h.rows();
  result->dof = result->rows - result->rank;
  const double smallest = result->rank > 0 ? s(result->rank - 1) : 0.0;
  result->condition_number = smallest > 0.0 ? largest / smallest
                                             : std::numeric_limits<double>::infinity();
  const Eigen::VectorXd residual = z - h * result->state_increment;
  result->statistic = residual.squaredNorm();
  NumericalWorkCounters::candidateInnerLlt();
  Eigen::LLT<Eigen::MatrixXd> llt(h.transpose() * h);
  if (llt.info() == Eigen::Success) {
    result->information_logdet =
        2.0 * llt.matrixL().toDenseMatrix().diagonal().array().log().sum();
  }
  result->valid = result->covariance.allFinite() &&
      result->state_increment.allFinite() && result->rank == h.cols() &&
      result->dof > 0 && result->condition_number <= max_condition &&
      result->state_increment.norm() <= max_step &&
      std::isfinite(result->statistic);
  if (!result->valid && result->reason.empty()) {
    if (result->rank != h.cols()) result->reason = "candidate rank loss";
    else if (result->dof <= 0) result->reason = "candidate has no residual dof";
    else if (result->condition_number > max_condition) result->reason = "candidate condition gate failed";
    else if (result->state_increment.norm() > max_step) result->reason = "candidate linearization step gate failed";
    else result->reason = "candidate contains non-finite values";
  }
}

}  // namespace

std::uint64_t candidateNumericalProofIdentity(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate) {
  return candidateNumericalProofIdentityImpl(window, candidate);
}

std::uint64_t candidateDetectorActionIdentity(
    const ExclusionAction& action) {
  std::uint64_t hash = 1469598103934665603ULL;
  hashCertificateScalar(&hash, action.id.value());
  auto hash_sorted_ids = [&](const auto& ids) {
    std::vector<std::uint64_t> values;
    values.reserve(ids.size());
    for (const auto id : ids) values.push_back(id.value());
    std::sort(values.begin(), values.end());
    hashCertificateScalar(&hash, static_cast<std::uint64_t>(values.size()));
    for (const auto value : values) hashCertificateScalar(&hash, value);
  };
  hash_sorted_ids(action.covered_units);
  hash_sorted_ids(action.covered_modes);
  std::vector<std::string> sources = action.physical_source_ids;
  std::sort(sources.begin(), sources.end());
  hashCertificateScalar(&hash, static_cast<std::uint64_t>(sources.size()));
  for (const auto& source : sources) hashCertificateString(&hash, source);
  hash_sorted_ids(action.groups_to_remove);
  hash_sorted_ids(action.groups_to_add);
  std::vector<std::uint64_t> added_blocks;
  added_blocks.reserve(action.added_blocks.size());
  for (const auto& block : action.added_blocks) {
    added_blocks.push_back(blockCertificateIdentity(block));
  }
  std::sort(added_blocks.begin(), added_blocks.end());
  hashCertificateScalar(&hash,
                        static_cast<std::uint64_t>(added_blocks.size()));
  for (const auto value : added_blocks) hashCertificateScalar(&hash, value);
  hashCertificateScalar(&hash, static_cast<int>(action.bridge_mode));
  hashCertificateScalar(&hash, action.exclusion_cardinality);
  hashCertificateString(&hash, action.action_model_id);
  hashCertificateScalar(&hash, static_cast<int>(action.recoverability));
  const bool has_begin = action.recovery_epoch_begin.has_value();
  const bool has_end = action.recovery_epoch_end.has_value();
  hashCertificateScalar(&hash, has_begin);
  if (has_begin) hashCertificateScalar(
      &hash, static_cast<std::uint64_t>(*action.recovery_epoch_begin));
  hashCertificateScalar(&hash, has_end);
  if (has_end) hashCertificateScalar(
      &hash, static_cast<std::uint64_t>(*action.recovery_epoch_end));
  hashCertificateString(&hash, action.removal_data_source);
  hashCertificateString(&hash, action.model_error_record);
  hashCertificateScalar(&hash, action.model_error_validated);
  return hash;
}

std::uint64_t candidateDetectorNumericalIdentity(
    const LinearizedIntegrityWindow& window,
    const ExclusionAction& action) {
  std::uint64_t hash = 1469598103934665603ULL;
  hashCertificateScalar(&hash, integrityWindowFingerprint(window));
  const std::uint64_t numerical_contract = window.numerics
      ? window.numerics->numerical_contract_fingerprint : 0;
  hashCertificateScalar(&hash, numerical_contract);
  const bool no_op = action.groups_to_remove.empty() &&
      action.groups_to_add.empty() && action.added_blocks.empty();
  const std::uint64_t numerical_action = no_op
      ? 0 : candidateDetectorActionIdentity(action);
  hashCertificateScalar(&hash, numerical_action);
  return hash;
}

std::uint64_t candidateDetectorCertificateDigest(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate,
    const CandidateDetectorCertificate& certificate) {
  std::uint64_t hash = 1469598103934665603ULL;
  hashCertificateScalar(&hash, integrityWindowFingerprint(window));
  const std::uint64_t numerical_contract = window.numerics
      ? window.numerics->numerical_contract_fingerprint : 0;
  hashCertificateScalar(&hash, numerical_contract);
  hashCertificateVersion(&hash, candidate.base_version);
  hashCertificateScalar(&hash, candidateDetectorActionIdentity(candidate.action));
  hashCertificateScalar(&hash, candidate.rows);
  hashCertificateScalar(&hash, candidate.rank);
  hashCertificateScalar(&hash, candidate.dof);
  hashCertificateScalar(&hash, candidate.statistic);
  hashCertificateScalar(&hash, candidate.condition_number);
  hashCertificateScalar(&hash, candidate.information_logdet);
  hashCertificateScalar(&hash, candidate.valid);
  hashCertificateScalar(&hash, candidate.exact_slow_path);
  hashCertificateScalar(&hash,
                        candidateNumericalProofIdentityImpl(window, candidate));
  const std::uint64_t candidate_window_fingerprint = candidate.window_view
      ? integrityWindowFingerprint(*candidate.window_view) : 0;
  hashCertificateScalar(&hash, candidate_window_fingerprint);
  hashCertificateEigen(&hash, candidate.state_increment);
  hashCertificateEigen(&hash, candidate.covariance);
  hashCertificateEigen(&hash, candidate.reference_inverse_squared);
  hashCertificateEigen(&hash, candidate.covariance_plus_factor);
  hashCertificateEigen(&hash, candidate.covariance_minus_factor);
  if (candidate.reference_vectors) {
    hashCertificateEigen(&hash, *candidate.reference_vectors);
  } else {
    hashCertificateScalar(&hash, std::uint64_t{0});
  }
  if (candidate.shared_base_factorization) {
    const Eigen::MatrixXd factor =
        candidate.shared_base_factorization->matrixL().toDenseMatrix();
    hashCertificateEigen(&hash, factor);
  } else {
    hashCertificateScalar(&hash, std::uint64_t{0});
  }
  hashCertificateEigen(&hash, candidate.retainedJacobian());
  hashCertificateEigen(&hash, candidate.retained_residual);

  hashCertificateScalar(&hash,
                        static_cast<std::uint64_t>(certificate.row_roles.size()));
  for (const auto role : certificate.row_roles) {
    hashCertificateScalar(&hash, static_cast<std::uint8_t>(role));
  }
  hashCertificateScalar(&hash, certificate.current_statistic);
  hashCertificateScalar(&hash, certificate.history_statistic);
  hashCertificateScalar(&hash, certificate.history_constant);
  hashCertificateScalar(&hash, certificate.current_rows);
  hashCertificateScalar(&hash, certificate.current_rank);
  hashCertificateScalar(&hash, certificate.current_dof);
  hashCertificateScalar(&hash, certificate.history_rows);
  hashCertificateScalar(&hash, certificate.history_rank);
  hashCertificateScalar(&hash, certificate.history_dof);
  hashCertificateScalar(&hash, certificate.pooled_rank);
  hashCertificateScalar(&hash, certificate.pooled_dof);
  hashCertificateString(&hash, certificate.accepted_event_id);
  hashCertificateScalar(&hash, certificate.window_fingerprint);
  hashCertificateScalar(&hash, certificate.numerical_contract_identity);
  hashCertificateScalar(&hash, certificate.canonical_action_identity);
  hashCertificateScalar(&hash, certificate.numerical_identity);
  hashCertificateScalar(&hash, certificate.history_constant_present);
  hashCertificateScalar(&hash, certificate.valid);
  return hash;
}

bool validateCandidateDetectorCertificate(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate,
    std::string* reason) {
  auto reject = [&](const std::string& message) {
    if (reason) *reason = message;
    return false;
  };
  if (!window.model_valid || !window.numerics || !window.numerics->valid ||
      window.numerics->window_id != window.id ||
      !(window.numerics->version == window.version) ||
      window.numerics->content_fingerprint != integrityWindowFingerprint(window) ||
      window.numerics->numerical_contract_fingerprint !=
          numericalContractFingerprint(window.numerics->numerical_contract) ||
      !validateFrozenWindowNumericalProof(window, *window.numerics)) {
    return reject("candidate certificate window identity mismatch");
  }
  if (!(candidate.base_version == window.version)) {
    return reject("candidate certificate linearization version mismatch");
  }
  const std::uint64_t expected_proof =
      candidateNumericalProofIdentityImpl(window, candidate);
  if (candidate.window_view &&
      (candidate.window_view->id != window.id ||
       !(candidate.window_view->version == window.version) ||
       integrityWindowFingerprint(*candidate.window_view) !=
           integrityWindowFingerprint(window))) {
    return reject("candidate source window identity mismatch");
  }
  std::string action_reason;
  if (!validateAction(window, candidate.action, &action_reason)) {
    return reject("candidate certificate action invalid: " + action_reason);
  }
  const CandidateDetectorCertificate expected =
      buildCandidateDetectorCertificate(
          window, candidate.action, candidate,
          window.numerics->numerical_contract.rank_tolerance);
  const auto& actual = candidate.detector_certificate;
  if (!actual.valid || !expected.valid) {
    return reject("candidate detector certificate is not valid");
  }
  const bool fields_match =
      actual.row_roles == expected.row_roles &&
      actual.current_statistic == expected.current_statistic &&
      actual.history_statistic == expected.history_statistic &&
      actual.history_constant == expected.history_constant &&
      actual.current_rows == expected.current_rows &&
      actual.current_rank == expected.current_rank &&
      actual.current_dof == expected.current_dof &&
      actual.history_rows == expected.history_rows &&
      actual.history_rank == expected.history_rank &&
      actual.history_dof == expected.history_dof &&
      actual.pooled_rank == expected.pooled_rank &&
      actual.pooled_dof == expected.pooled_dof &&
      actual.accepted_event_id == expected.accepted_event_id &&
      actual.window_fingerprint == expected.window_fingerprint &&
      actual.numerical_contract_identity ==
          expected.numerical_contract_identity &&
      actual.canonical_action_identity == expected.canonical_action_identity &&
      expected_proof != 0 &&
      actual.numerical_identity == expected.numerical_identity &&
      actual.history_constant_present == expected.history_constant_present;
  if (!fields_match) {
    return reject("candidate detector certificate payload mismatch");
  }
  const std::uint64_t recomputed = candidateDetectorCertificateDigest(
      window, candidate, actual);
  if (actual.certificate_digest == 0 ||
      actual.certificate_digest != recomputed ||
      actual.certificate_digest != expected.certificate_digest) {
    return reject("candidate detector certificate digest mismatch");
  }
  return true;
}

Eigen::MatrixXd CandidateEvaluation::covarianceTimes(
    const Eigen::MatrixXd& value) const {
  if (value.rows() <= 0 || !value.allFinite()) return Eigen::MatrixXd();
  Eigen::MatrixXd primary;
  if (covariance.size() != 0) {
    if (covariance.rows() != value.rows() || covariance.cols() != value.rows())
      return Eigen::MatrixXd();
    primary = covariance * value;
  } else if (reference_vectors) {
    if (value.rows() != reference_vectors->rows() ||
        reference_inverse_squared.size() != reference_vectors->cols()) {
      return Eigen::MatrixXd();
    }
    primary = *reference_vectors * reference_inverse_squared.asDiagonal() *
        reference_vectors->transpose() * value;
  } else {
    if (!shared_base_factorization ||
        shared_base_factorization->matrixL().rows() != value.rows()) {
      return Eigen::MatrixXd();
    }
    primary = shared_base_factorization->solve(value);
    if (covariance_plus_factor.cols()) {
      primary.noalias() += covariance_plus_factor *
          (covariance_plus_factor.transpose() * value);
    }
    if (covariance_minus_factor.cols()) {
      primary.noalias() -= covariance_minus_factor *
          (covariance_minus_factor.transpose() * value);
    }
  }
  if (!window_view || !window_view->numerics ||
      detector_certificate.numerical_identity == 0) {
    // Compatibility for standalone/offline containers.  Active candidates are
    // always bound to a frozen window and take the certified path below.
    return primary.allFinite() ? primary : Eigen::MatrixXd();
  }
  const auto& window = *window_view;
  Eigen::MatrixXd information = window.base_information;
  for (const auto& block : window.blocks) {
    if (groupSelected(block.group_id, action.groups_to_remove)) {
      information.noalias() -= block.jacobian_whitened.transpose() *
                               block.jacobian_whitened;
    }
  }
  for (const auto& block : action.added_blocks) {
    information.noalias() += block.jacobian_whitened.transpose() *
                             block.jacobian_whitened;
  }
  information = 0.5 * (information + information.transpose());
  const auto& contract = window.numerics->numerical_contract;
  auto certified = [&](const Eigen::MatrixXd& solution,
                       double jacobian_condition,
                       const Eigen::MatrixXd& certified_information) {
    if (solution.rows() != value.rows() ||
        solution.cols() != value.cols() || !solution.allFinite()) return false;
    const Eigen::MatrixXd residual = certified_information * solution - value;
    const double denominator = std::max(
        {1.0, value.norm(),
         certified_information.norm() * solution.norm()});
    const double backward = residual.norm() / denominator;
    const double condition = std::max(1.0, jacobian_condition);
    const double forward = condition * condition * backward;
    const double roundoff = 128.0 * std::numeric_limits<double>::epsilon() *
        static_cast<double>(std::max(certified_information.rows(),
                                     value.cols()));
    return std::isfinite(backward) && std::isfinite(forward) &&
        backward <= contract.solve_residual_limit + roundoff &&
        forward <= contract.forward_error_limit + roundoff;
  };
  double condition = std::isfinite(condition_number)
      ? condition_number : diagnostics.condition_upper_bound;
  if (!std::isfinite(condition) || condition <= 0.0) {
    condition = window.numerics->exact_condition;
  }
  if (certified(primary, condition, information)) return primary;

  // Exact fallback: rebuild the final whitened Jacobian and use its direct SVD
  // for every RHS type (state, covariance, fault and bridge alike).
  Eigen::MatrixXd final_h;
  Eigen::VectorXd final_z;
  if (!candidateRows(window, action, &final_h, &final_z)) {
    return Eigen::MatrixXd();
  }
  NumericalWorkCounters::candidateReferenceSvd();
  Eigen::JacobiSVD<Eigen::MatrixXd> reference(
      final_h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular = reference.singularValues();
  const double largest = singular.size() ? singular(0) : 0.0;
  const double gate = contract.rank_tolerance * largest;
  const int solved_rank = static_cast<int>((singular.array() > gate).count());
  if (solved_rank != final_h.cols()) return Eigen::MatrixXd();
  Eigen::VectorXd inverse_squared = singular.array().square().inverse();
  const Eigen::MatrixXd fallback = reference.matrixV() *
      inverse_squared.asDiagonal() * reference.matrixV().transpose() * value;
  const double smallest = singular(singular.size() - 1);
  const double exact_condition = smallest > 0.0
      ? largest / smallest : std::numeric_limits<double>::infinity();
  const Eigen::MatrixXd reference_information =
      final_h.transpose() * final_h;
  if (certified(fallback, exact_condition, reference_information)) {
    return fallback;
  }
  // The direct raw-H SVD is the exact fallback.  When the conservative
  // kappa(H)^2 bound is pessimistic, require both a backward-stable SVD result
  // and agreement with the independently formed update/root result.  No full
  // n-by-n normal equation is solved in higher precision.
  const double fallback_backward =
      (reference_information * fallback - value).norm() /
      std::max({value.norm(),
                reference_information.norm() * fallback.norm(),
                std::numeric_limits<double>::min()});
  const double solution_scale = std::max(
      {fallback.norm(), primary.norm(), std::numeric_limits<double>::min()});
  const double independent_agreement =
      (fallback - primary).norm() / solution_scale;
  const double roundoff = 128.0 * std::numeric_limits<double>::epsilon() *
      static_cast<double>(std::max(reference_information.rows(), value.cols()));
  return fallback.allFinite() && std::isfinite(fallback_backward) &&
             fallback_backward <= contract.solve_residual_limit + roundoff &&
             std::isfinite(independent_agreement) &&
             independent_agreement <= contract.reference_relative_tolerance
      ? fallback
      : Eigen::MatrixXd();
}

Eigen::MatrixXd CandidateEvaluation::normalCross(
    const Eigen::MatrixXd& row_map) const {
  if (retainedJacobian().rows() == row_map.rows() &&
      retainedJacobian().cols() > 0) {
    return retainedJacobian().transpose() * row_map;
  }
  if (!window_view || row_map.rows() != rows) return Eigen::MatrixXd();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(
      window_view->H.cols(), row_map.cols());
  Eigen::Index offset = 0;
  for (const auto& block : window_view->blocks) {
    if (groupSelected(block.group_id, action.groups_to_remove)) continue;
    const Eigen::Index count = block.jacobian_whitened.rows();
    result.noalias() += block.jacobian_whitened.transpose() *
        row_map.middleRows(offset, count);
    offset += count;
  }
  for (const auto& block : action.added_blocks) {
    const Eigen::Index count = block.jacobian_whitened.rows();
    result.noalias() += block.jacobian_whitened.transpose() *
        row_map.middleRows(offset, count);
    offset += count;
  }
  return offset == row_map.rows() ? result : Eigen::MatrixXd();
}

BaseCandidateKernel RankUpdateEvaluator::factorizeOnce(
    const LinearizedIntegrityWindow& window,
    const std::vector<ExclusionAction>& actions) const {
  BaseCandidateKernel base;
  base.window_id = window.id;
  base.version = window.version;
  base.window_view = &window;
  if (!window.model_valid) {
    base.reason = "invalid base window: " + window.reason;
    return base;
  }
  for (const auto& block : window.blocks) {
    if (!validBlock(block, window)) { base.reason = "invalid frozen block/version"; return base; }
  }
  if (window.H.rows() != window.z.size() || !window.H.allFinite() || !window.z.allFinite() ||
      window.base_information.rows() != window.H.cols() ||
      window.base_information.cols() != window.H.cols() ||
      window.base_information_rhs.size() != window.H.cols() ||
      !window.base_information.allFinite() || !window.base_information_rhs.allFinite() ||
      (window.protected_state_map.size() != 0 && (window.protected_state_map.cols() != window.H.cols() ||
        !window.protected_state_map.allFinite()))) {
    base.reason = "invalid base matrix dimensions/finiteness"; return base;
  }
  if (!window.numerics || !window.numerics->valid ||
      !(window.numerics->window_id == window.id) ||
      !(window.numerics->version == window.version) ||
      window.numerics->content_fingerprint != integrityWindowFingerprint(window) ||
      window.numerics->numerical_contract_fingerprint !=
          numericalContractFingerprint(config_.rank_tolerance,
                                       config_.max_condition_number)) {
    if (window.numerics &&
        window.numerics->numerical_contract_fingerprint !=
            numericalContractFingerprint(config_.rank_tolerance,
                                         config_.max_condition_number)) {
      NumericalWorkCounters::numericalContractMismatch();
    }
    base.reason = "missing, invalid, or stale frozen-window numerics";
    return base;
  }
  base.numerics = window.numerics;
  base.information_factorization = window.numerics->information_factorization;
  base.state_increment = window.numerics->base_state_increment;
  base.information_logdet = window.numerics->information_logdet;
  base.exact_rank = window.numerics->exact_rank;
  base.smallest_singular_value = window.numerics->smallest_singular_value;
  base.exact_condition = window.numerics->exact_condition;
  // A large all-in step is itself a fault symptom. Keep the one-time base
  // factorization usable so exclusion candidates can recover; apply the
  // linearization-step gate independently to every candidate below.
  base.valid = base.state_increment.allFinite();
  if (!base.valid) base.reason = "base solution contains non-finite values";
  buildSharedCache(&base, actions);
  return base;
}

void RankUpdateEvaluator::buildSharedCache(BaseCandidateKernel* kernel,
    const std::vector<ExclusionAction>& actions) const {
  if (!kernel) return;
  auto& base = *kernel;
  if (!base.window_view) return;
  const auto& window = *base.window_view;
  base.block_cache.reset();
  if (base.valid && config_.enable_shared_cache && !actions.empty() &&
      !std::getenv("UWB_IMU_PL_DISABLE_BLOCK_CACHE")) {
    auto cache = std::make_shared<FrozenBlockSolveCache>();
    cache->window_id = window.id; cache->version = window.version;
    auto add = [&](const LinearizedFactorBlock& block) {
      if (!validBlock(block, window)) return;
      auto& entries = cache->blocks[block.group_id];
      if (std::any_of(entries.begin(), entries.end(), [&](const auto& e) { return sameBlock(e.block, block); })) return;
      entries.push_back({block, solveFrozenInformation(
          window.square_root.get(), *base.numerics, window.base_information,
          block.jacobian_whitened.transpose())});
    };
    std::set<FactorGroupId> removals;
    for (const auto& action : actions) {
      removals.insert(action.groups_to_remove.begin(), action.groups_to_remove.end());
      for (const auto& block : action.added_blocks) add(block);
    }
    for (const auto& block : window.blocks) if (removals.count(block.group_id)) add(block);
    base.block_cache = std::move(cache);
  }
}

CandidateEvaluation RankUpdateEvaluator::evaluate(
    const BaseCandidateKernel& base, const ExclusionAction& action) const {
  return evaluate(base, action, nullptr);
}

CandidateEvaluation RankUpdateEvaluator::evaluate(
    const BaseCandidateKernel& base, const ExclusionAction& action,
    RankUpdateScratch* scratch) const {
  CandidateEvaluation result;
  const auto start = std::chrono::steady_clock::now();
  result.action = action;
  result.base_version = base.version;
  if (!base.window_view) {
    result.reason = "base window view is unavailable";
    return result;
  }
  const auto& window = *base.window_view;
  result.window_view = base.window_view;
  result.diagnostics.kernel_evaluated = true;
  result.diagnostics.numerical_path = "ADD_THEN_REMOVE_SVD";
  auto done = [&]() {
    if (result.valid && !result.detector_certificate.valid) {
      result.detector_certificate = buildCandidateDetectorCertificate(
          window, action, result, config_.rank_tolerance);
    }
    result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    result.diagnostics.kernel_ms = result.wall_ms;
    result.diagnostics.numerical_valid = result.valid;
    result.diagnostics.slow_path = result.exact_slow_path;
    return result;
  };
  if (!base.valid) { result.reason = base.reason; return done(); }
  // Validate the complete change before any Eigen operation or early gate.
  if (!validateAction(window, action, &result.reason)) return done();
  if (!base.numerics ||
      base.numerics->content_fingerprint != integrityWindowFingerprint(window)) {
    result.reason = "missing or stale frozen numerics";
    result.diagnostics.fallback_reason = result.reason;
    return done();
  }
  result.shared_base_factorization = base.information_factorization;
  Eigen::VectorXd rhs = window.base_information_rhs;
  std::vector<const LinearizedFactorBlock*> additions, removals;
  for (const auto& b : action.added_blocks) additions.push_back(&b);
  for (const auto& b : window.blocks)
    if (groupSelected(b.group_id, action.groups_to_remove)) removals.push_back(&b);
  auto order = [](auto& blocks) {
    std::stable_sort(blocks.begin(), blocks.end(), [](auto* a, auto* b) { return a->group_id < b->group_id; });
  };
  order(additions); order(removals);
  double energy = window.z.squaredNorm();
  double energy_scale = energy;
  result.information_logdet = base.information_logdet;
  Eigen::Index update_count = 0;
  for (auto* block : additions) update_count += block->jacobian_whitened.rows();
  for (auto* block : removals) update_count += block->jacobian_whitened.rows();
  Eigen::MatrixXd local_update;
  Eigen::VectorXd local_signs;
  Eigen::MatrixXd& update_columns = scratch ? scratch->update_columns : local_update;
  Eigen::VectorXd& update_signs = scratch ? scratch->update_signs : local_signs;
  if (scratch && update_columns.size() != 0) ++result.diagnostics.scratch_reuse_count;
  update_columns.resize(window.H.cols(), update_count);
  update_signs.resize(update_count);
  Eigen::Index update_offset = 0;
  auto append_update = [&](const std::vector<const LinearizedFactorBlock*>& blocks,
                           double sign) {
    for (auto* block : blocks) {
      const Eigen::Index count = block->jacobian_whitened.rows();
      update_columns.middleCols(update_offset, count) =
          block->jacobian_whitened.transpose();
      update_signs.segment(update_offset, count).setConstant(sign);
      update_offset += count;
    }
  };
  append_update(additions, 1.0);
  append_update(removals, -1.0);
  bool needs_reference = false, legacy_deletion_failed = false;
  auto request_reference = [&](const std::string& reason) {
    needs_reference = true;
    result.exact_slow_path = true;
    if (!result.diagnostics.fallback_reason.empty()) result.diagnostics.fallback_reason += ";";
    result.diagnostics.fallback_reason += reason;
  };
  auto change = [&](const std::vector<const LinearizedFactorBlock*>& blocks, bool add) {
    if (blocks.empty()) return;
    Eigen::Index rows = 0;
    for (auto* b : blocks) rows += b->jacobian_whitened.rows();
    Eigen::MatrixXd j(rows, window.H.cols());
    Eigen::VectorXd z(rows);
    Eigen::Index offset = 0;
    for (auto* b : blocks) {
      const auto n = b->jacobian_whitened.rows();
      j.middleRows(offset, n) = b->jacobian_whitened;
      z.segment(offset, n) = b->residual_whitened;
      offset += n;
    }
    rhs.noalias() += (add ? 1.0 : -1.0) * j.transpose() * z;
    energy += (add ? 1.0 : -1.0) * z.squaredNorm();
    energy_scale += z.squaredNorm();
    if (needs_reference) return;
    Eigen::MatrixXd cj(j.cols(), j.rows());
    Eigen::Index cache_offset = 0;
    for (auto* block : blocks) {
      bool hit = false;
      if (base.block_cache && base.block_cache->window_id == base.window_id &&
          base.block_cache->version == base.version) {
        const auto entries = base.block_cache->blocks.find(block->group_id);
        if (entries != base.block_cache->blocks.end()) for (const auto& entry : entries->second) {
          if (!sameBlock(entry.block, *block)) continue;
          cj.middleCols(cache_offset, block->jacobian_whitened.rows()) = entry.base_solve;
          ++result.diagnostics.cache_hits; hit = true; break;
        }
      }
      if (!hit) cj.middleCols(cache_offset, block->jacobian_whitened.rows()) =
          solveFrozenInformation(base.window_view->square_root.get(),
              *base.numerics, window.base_information,
              block->jacobian_whitened.transpose());
      cache_offset += block->jacobian_whitened.rows();
    }
    if (!add && !additions.empty()) {
      const Eigen::MatrixXd legacy_inner = Eigen::MatrixXd::Identity(rows, rows) - j * cj;
      NumericalWorkCounters::candidateInnerLlt();
      legacy_deletion_failed = Eigen::LLT<Eigen::MatrixXd>(legacy_inner).info() != Eigen::Success;
    }
    // Addition first: the downdate operates on C_add, so an absent/singular
    // deletion-only intermediate can never reject an SPD complete replacement.
    if (!add && result.covariance_minus_factor.cols())
      cj.noalias() -= result.covariance_minus_factor * (result.covariance_minus_factor.transpose() * j.transpose());
    Eigen::MatrixXd inner = Eigen::MatrixXd::Identity(rows, rows) + (add ? 1.0 : -1.0) * j * cj;
    inner = (0.5 * (inner + inner.transpose())).eval();
    NumericalWorkCounters::candidateInnerLlt();
    Eigen::LLT<Eigen::MatrixXd> factor(inner);
    if (factor.info() != Eigen::Success || !cj.allFinite()) {
      request_reference(add ? "addition inner failure" : "final downdate inner failure");
      return;
    }
    Eigen::MatrixXd correction = factor.matrixL().solve(cj.transpose()).transpose();
    if (add) result.covariance_minus_factor = std::move(correction);
    else result.covariance_plus_factor = std::move(correction);
    result.information_logdet += 2.0 * factor.matrixL().toDenseMatrix().diagonal().array().log().sum();
  };
  change(additions, true);
  change(removals, false);
  result.rows = window.H.rows();
  for (auto* b : additions) result.rows += b->jacobian_whitened.rows();
  for (auto* b : removals) result.rows -= b->jacobian_whitened.rows();
  if (!rhs.allFinite() || !std::isfinite(energy)) {
    result.reason = "candidate contains non-finite information or residual energy"; return done();
  }
  const bool keep = additions.empty() && removals.empty();
  // KEEP_ALL reuses the frozen base solve. It still traverses the numerical
  // certificates below so near-step, ill-conditioned, and cancellation cases
  // retain the exact reference behavior required by the numerical contract.
  if (keep) result.state_increment = base.numerics->base_state_increment;
  else {
    result.state_increment = result.covarianceTimes(rhs);
    if (result.state_increment.rows() != rhs.rows() ||
        result.state_increment.cols() != 1 ||
        !result.state_increment.allFinite()) {
      // The update operator could not certify the state RHS.  Continue through
      // the mandatory final-Jacobian reference instead of letting an empty
      // tentative solve enter residual arithmetic.
      request_reference("candidate state RHS certificate failed");
      result.state_increment = Eigen::VectorXd::Zero(rhs.rows());
    }
  }
  Eigen::MatrixXd h;
  Eigen::VectorXd z;
  const double keep_step = result.state_increment.norm();
  const double keep_gate_band =
      1e-7 * config_.max_linearization_step_norm;
  if (keep && std::isfinite(keep_step) &&
      keep_step < config_.max_linearization_step_norm - keep_gate_band) {
    // The unchanged candidate is exactly the finalized frozen window.  Its
    // direct parity statistic avoids the cancellation-prone E-z'A^-1z form,
    // and its rank/condition/logdet already came from the canonical base SVD.
    // Re-running a candidate SVD here adds no independent protection.
    result.rank = base.exact_rank;
    result.dof = base.numerics->dof;
    result.condition_number = base.exact_condition;
    result.information_logdet = base.numerics->information_logdet;
    result.statistic = base.numerics->statistic;
    result.diagnostics.certificate_passed = true;
    result.diagnostics.condition_value_kind = "EXACT_BASE";
    result.diagnostics.numerical_path = "KEEP_BASE_CANONICAL";
    result.valid = result.state_increment.allFinite() &&
        std::isfinite(result.statistic) &&
        std::isfinite(result.information_logdet) &&
        result.rank == window.H.cols() && result.dof > 0 &&
        result.condition_number <= config_.max_condition_number &&
        result.state_increment.norm() <= config_.max_linearization_step_norm;
    if (!result.valid) {
      if (result.rank != window.H.cols()) result.reason = "candidate rank loss";
      else if (result.dof <= 0) result.reason = "candidate has no residual dof";
      else if (result.condition_number > config_.max_condition_number)
        result.reason = "candidate condition gate failed";
      else if (result.state_increment.norm() > config_.max_linearization_step_norm)
        result.reason = "candidate linearization step gate failed";
      else result.reason = "candidate contains non-finite values";
    }
    if (config_.materialize_dense_oracle_fields) {
      if (!candidateRows(window, action, &h, &z)) {
        result.valid = false;
        result.reason = "candidate row materialization failed";
        return done();
      }
      result.covariance = result.covarianceTimes(
          Eigen::MatrixXd::Identity(h.cols(), h.cols()));
      result.retained_jacobian = std::move(h);
      result.retained_residual = std::move(z);
    }
    return done();
  }
  SpectralCertificate certificate;
  if (!needs_reference && config_.enable_numerical_certificate &&
      !std::getenv("UWB_IMU_PL_DISABLE_NUMERICAL_CERTIFICATE")) {
    certificate = certifyUpdate(*base.numerics, update_columns, update_signs,
                                config_.rank_tolerance,
                                config_.max_condition_number, scratch);
    result.diagnostics.condition_lower_bound = certificate.condition_lower;
    result.diagnostics.condition_upper_bound = certificate.condition_upper;
    result.diagnostics.certificate_margin = certificate.margin;
    if (!certificate.spd) request_reference("SPD certificate inconclusive");
    else if (!certificate.full_rank) request_reference("rank certificate inconclusive");
    else if (!certificate.condition_passed) request_reference("condition certificate inconclusive");
    else if (config_.force_exact_condition_number) request_reference("exact condition requested");
    else {
      result.rank = window.H.cols();
      result.dof = result.rows - result.rank;
      result.condition_number = keep ? base.exact_condition
                                     : std::numeric_limits<double>::quiet_NaN();
      result.diagnostics.condition_value_kind = keep ? "EXACT_BASE"
                                                     : "CERTIFIED_BOUNDS";
      result.diagnostics.certificate_passed = true;
      result.diagnostics.numerical_path = keep ? "KEEP_BASE_CERTIFICATE" : "LOW_RANK_CERTIFICATE";
    }
  } else if (!needs_reference) {
    request_reference("numerical certificate disabled");
  }
  if (!needs_reference && result.dof <= 0) {
    result.reason = "candidate has no residual dof"; return done();
  }
  Eigen::VectorXd normal_residual = rhs - window.base_information * result.state_increment;
  for (auto* block : additions) normal_residual.noalias() -=
      block->jacobian_whitened.transpose() *
      (block->jacobian_whitened * result.state_increment);
  for (auto* block : removals) normal_residual.noalias() +=
      block->jacobian_whitened.transpose() *
      (block->jacobian_whitened * result.state_increment);
  double solve_error = std::numeric_limits<double>::infinity();
  if (certificate.lambda_min_lower > 0.0) {
    const double rounding = 128.0 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, energy_scale + rhs.norm() * result.state_increment.norm());
    solve_error = (normal_residual.norm() + rounding) / certificate.lambda_min_lower;
  }
  const double step = result.state_increment.norm();
  result.diagnostics.near_gate = !std::isfinite(solve_error) ||
      std::abs(step - config_.max_linearization_step_norm) <=
          std::max(1e-7 * config_.max_linearization_step_norm, solve_error);
  if (!needs_reference && config_.enable_early_step_gate &&
      !std::getenv("UWB_IMU_PL_DISABLE_EARLY_STEP") &&
      std::isfinite(solve_error) && step - solve_error >
          config_.max_linearization_step_norm * (1.0 + 1e-7)) {
    result.reason = "candidate linearization step gate failed";
    result.diagnostics.matrix_free_step_rejected = true;
    // Preserve the P2 path spelling for downstream readers; the explicit v4
    // flag above distinguishes the new matrix-free certificate.
    result.diagnostics.numerical_path = "QR_CERTIFIED_STEP_REJECTION";
    result.diagnostics.skip_reason = "certified step exceeds gate; final Jacobian not built";
    return done();
  }
  if (!needs_reference && (!std::isfinite(solve_error) ||
      step + solve_error > config_.max_linearization_step_norm ||
      result.diagnostics.near_gate)) request_reference("step certificate inconclusive");
  if (!needs_reference && solve_error >
      1e-9 * std::max(1.0, result.state_increment.norm()))
    request_reference("unstable normal solve");
  const double subtracted = energy - rhs.dot(result.state_increment);
  if (!std::isfinite(subtracted) || (energy_scale > 0.0 && subtracted <= 1e-8 * energy_scale))
    request_reference("residual energy cancellation");
  if (!needs_reference && result.state_increment.norm() <= config_.max_linearization_step_norm &&
      window.protected_state_map.cols() == window.H.cols()) {
    const Eigen::MatrixXd protected_rhs = window.protected_state_map.transpose();
    const Eigen::MatrixXd protected_solve = result.covarianceTimes(protected_rhs);
    Eigen::MatrixXd covariance_residual = protected_rhs -
        window.base_information * protected_solve;
    for (auto* block : additions) covariance_residual.noalias() -=
        block->jacobian_whitened.transpose() *
        (block->jacobian_whitened * protected_solve);
    for (auto* block : removals) covariance_residual.noalias() +=
        block->jacobian_whitened.transpose() *
        (block->jacobian_whitened * protected_solve);
    const double covariance_error = covariance_residual.norm() /
        certificate.lambda_min_lower;
    if (!protected_solve.allFinite() || !std::isfinite(covariance_error) ||
        covariance_error > 1e-7 * std::max(1e-15, protected_solve.norm()))
      request_reference("protected covariance solve residual");
  }
  if (needs_reference) {
    if (!candidateRows(window, action, &h, &z)) {
      result.reason = "candidate removal block is absent from frozen window"; return done();
    }
    // Reference solve uses the final whitened Jacobian, not its squared normal
    // equations. Keep only an operator for covariance queries online.
    NumericalWorkCounters::candidateReferenceSvd();
    Eigen::JacobiSVD<Eigen::MatrixXd> reference(h, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = reference.singularValues();
    const double largest = singular.size() ? singular(0) : 0.0;
    const double gate = config_.rank_tolerance * largest;
    result.rank = static_cast<int>((singular.array() > gate).count());
    const double smallest = result.rank ? singular(result.rank - 1) : 0.0;
    result.condition_number = result.rank == h.cols() && smallest > 0.0
        ? largest / smallest : std::numeric_limits<double>::infinity();
    result.dof = result.rows - result.rank;
    Eigen::VectorXd inverse = Eigen::VectorXd::Zero(singular.size());
    for (Eigen::Index i = 0; i < singular.size(); ++i)
      if (singular(i) > gate) inverse(i) = 1.0 / singular(i);
    result.state_increment = reference.matrixV() * inverse.asDiagonal() *
        reference.matrixU().transpose() * z;
    result.reference_vectors = std::make_shared<const Eigen::MatrixXd>(reference.matrixV());
    result.reference_inverse_squared = inverse.array().square();
    result.information_logdet = result.rank == h.cols()
        ? 2.0 * singular.array().log().sum()
        : -std::numeric_limits<double>::infinity();
    result.diagnostics.numerical_path = "FINAL_JACOBIAN_REFERENCE";
    result.diagnostics.condition_value_kind = "EXACT_SVD";
    result.diagnostics.near_gate = smallest <= 10.0 * gate ||
        result.condition_number >= config_.max_condition_number / 10.0 ||
        std::abs(result.state_increment.norm() - config_.max_linearization_step_norm) <=
            1e-7 * config_.max_linearization_step_norm;
    if (result.rank != h.cols()) {
      result.reason = "candidate rank loss";
      result.statistic = (z - h * result.state_increment).squaredNorm();
      return done();
    }
    const Eigen::MatrixXd state_rhs = h.transpose() * z;
    const Eigen::MatrixXd direct_state = result.state_increment;
    const Eigen::MatrixXd information = h.transpose() * h;
    auto state_certified = [&](const Eigen::MatrixXd& solution) {
      if (solution.rows() != h.cols() || solution.cols() != 1 ||
          !solution.allFinite()) return false;
      const double backward = (information * solution - state_rhs).norm() /
          std::max({1.0, state_rhs.norm(),
                    information.norm() * solution.norm()});
      const double forward = result.condition_number *
          result.condition_number * backward;
      const double roundoff = 128.0 * std::numeric_limits<double>::epsilon() *
          static_cast<double>(information.rows());
      return std::isfinite(backward) && std::isfinite(forward) &&
          backward <= base.numerics->numerical_contract.solve_residual_limit +
                          roundoff &&
          forward <= base.numerics->numerical_contract.forward_error_limit +
                         roundoff;
    };
    if (!state_certified(direct_state)) {
      result.reason = "candidate state RHS reference certificate failed";
      result.valid = false;
      return done();
    }
    if (result.dof <= 0) {
      result.reason = "candidate has no residual dof";
      result.statistic = (z - h * result.state_increment).squaredNorm();
      return done();
    }
  }
  result.statistic = needs_reference ? (z - h * result.state_increment).squaredNorm()
                                     : (keep ? base.numerics->statistic
                                             : std::max(0.0, subtracted));
  result.valid = result.state_increment.allFinite() && std::isfinite(result.statistic) &&
      std::isfinite(result.information_logdet) &&
      (result.diagnostics.certificate_passed ||
       result.condition_number <= config_.max_condition_number) &&
      result.state_increment.norm() <= config_.max_linearization_step_norm;
  if (!result.valid) {
    if (result.condition_number > config_.max_condition_number) result.reason = "candidate condition gate failed";
    else if (result.state_increment.norm() > config_.max_linearization_step_norm) result.reason = "candidate linearization step gate failed";
    else result.reason = "candidate contains non-finite values";
  }
  result.diagnostics.recovered_replacement = legacy_deletion_failed && result.valid;
  if (config_.materialize_dense_oracle_fields) {
    if (h.size() == 0 && !candidateRows(window, action, &h, &z)) {
      result.valid = false; result.reason = "candidate row materialization failed"; return done();
    }
    result.covariance = result.covarianceTimes(Eigen::MatrixXd::Identity(h.cols(), h.cols()));
    result.retained_jacobian = std::move(h); result.retained_residual = std::move(z);
  }
  return done();
}

CandidateEvaluation DenseCandidateOracle::evaluate(
    const LinearizedIntegrityWindow& window, const ExclusionAction& action) const {
  CandidateEvaluation result;
  const auto wall_start = std::chrono::steady_clock::now();
  result.action = action;
  result.base_version = window.version;
  result.window_view = &window;
  if (!validateAction(window, action, &result.reason)) return result;
  for (const auto& block : action.added_blocks) {
    if (!(block.version == window.version) ||
        block.jacobian_whitened.cols() != window.H.cols() ||
        block.jacobian_whitened.rows() != block.residual_whitened.size() ||
        !block.jacobian_whitened.allFinite() ||
        !block.residual_whitened.allFinite()) {
      result.reason = "candidate addition block/version is invalid";
      result.wall_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
      return result;
    }
  }
  Eigen::MatrixXd h;
  Eigen::VectorXd z;
  if (!candidateRows(window, action, &h, &z)) {
    result.reason = "candidate removal block is absent from frozen window";
    result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    return result;
  }
  result.retained_jacobian = h;
  result.retained_residual = z;
  const Eigen::MatrixXd information = h.transpose() * h;
  if (!covarianceFromInformation(information, &result.covariance)) {
    result.reason = "dense candidate information is not SPD";
    result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    return result;
  }
  result.state_increment = result.covariance * h.transpose() * z;
  finishCandidate(&result, h, z, config_.rank_tolerance,
                  config_.max_condition_number,
                  config_.max_linearization_step_norm);
  result.diagnostics.numerical_path = "DENSE_DIRECT_SVD";
  if (result.valid) {
    result.detector_certificate = buildCandidateDetectorCertificate(
        window, action, result, config_.rank_tolerance);
  }
  result.diagnostics.condition_value_kind = "EXACT_SVD";
  result.wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wall_start).count();
  return result;
}

CandidateRouteWorkV1 candidateRouteWorkV1() {
  CandidateRouteWorkV1 out;
  out.routed_rank = g_candidate_route_rank.load();
  out.routed_dense = g_candidate_route_dense.load();
  out.rank_exceptions_recovered = g_candidate_route_rank_exception.load();
  return out;
}

void resetCandidateRouteWorkV1() {
  g_candidate_route_rank.store(0);
  g_candidate_route_dense.store(0);
  g_candidate_route_rank_exception.store(0);
}

CandidateEvaluation CandidateEvaluationRouterV1::evaluate(
    const LinearizedIntegrityWindow& window,
    const BaseCandidateKernel& base,
    const ExclusionAction& action,
    RankUpdateScratch* scratch,
    CandidateRouteSafetyV1* safety_output) const {
  CandidateRouteSafetyV1 safety;
  auto refuse_rank = [&](const std::string& reason) {
    safety.rank_path_safe = false;
    safety.reason = reason;
  };

  // Build the exact final-Jacobian reference first.  Besides being the dense
  // terminal fallback, it supplies a state/rank/condition certificate that is
  // independent of the add/downdate implementation.  Consequently no rank
  // operation is attempted merely because an action happens to have a known
  // type, amplitude, or ID.
  CandidateEvaluation exact = dense_.evaluate(window, action);
  // Keep the golden DenseCandidateOracle API/semantics untouched.  The new
  // P0-06 router strengthens only its own terminal fallback by recomputing the
  // final raw-H SVD state/covariance; this avoids using an H' H inverse as the
  // independent certificate without reopening existing P0-03 callers.
  Eigen::MatrixXd exact_h;
  Eigen::VectorXd exact_z;
  std::string exact_action_reason;
  if (validateAction(window, action, &exact_action_reason) &&
      candidateRows(window, action, &exact_h, &exact_z)) {
    NumericalWorkCounters::candidateReferenceSvd();
    Eigen::JacobiSVD<Eigen::MatrixXd> reference(
        exact_h, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singular = reference.singularValues();
    const double largest = singular.size() ? singular(0) : 0.0;
    const double gate = config_.rank_tolerance * largest;
    const int exact_rank =
        static_cast<int>((singular.array() > gate).count());
    if (exact_rank == exact_h.cols()) {
      const Eigen::VectorXd inverse = singular.cwiseInverse();
      exact.state_increment = reference.matrixV() * inverse.asDiagonal() *
          reference.matrixU().transpose() * exact_z;
      exact.covariance = reference.matrixV() *
          inverse.array().square().matrix().asDiagonal() *
          reference.matrixV().transpose();
      exact.reference_vectors =
          std::make_shared<const Eigen::MatrixXd>(reference.matrixV());
      exact.reference_inverse_squared = inverse.array().square();
      exact.reason.clear();
      finishCandidate(&exact, exact_h, exact_z, config_.rank_tolerance,
                      config_.max_condition_number,
                      config_.max_linearization_step_norm);
      exact.diagnostics.numerical_path = "DENSE_ROUTER_RAW_SVD";
      exact.detector_certificate = {};
      if (exact.valid) {
        exact.detector_certificate = buildCandidateDetectorCertificate(
            window, action, exact, config_.rank_tolerance);
      }
    }
  }
  safety.exact_rank = exact.rank;
  safety.exact_dof = exact.dof;
  safety.exact_condition = exact.condition_number;
  safety.exact_step_norm = exact.state_increment.size() == window.H.cols() &&
          exact.state_increment.allFinite()
      ? exact.state_increment.norm()
      : std::numeric_limits<double>::infinity();
  auto dense_terminal = [&](const std::string& reason) {
    exact.exact_slow_path = true;
    exact.diagnostics.slow_path = true;
    exact.diagnostics.numerical_path = "DENSE_ROUTER_EXACT";
    exact.diagnostics.fallback_reason = reason;
    // The detector certificate binds the complete candidate numerical
    // identity, including diagnostics.  Rebuild it only after the terminal
    // fallback identity is final; otherwise postcondition validation sees the
    // pre-routing digest and (correctly) refuses the candidate.
    exact.detector_certificate = {};
    if (exact.valid) {
      exact.detector_certificate = buildCandidateDetectorCertificate(
          window, action, exact, config_.rank_tolerance);
    }
    return exact;
  };

  bool shape_safe = true;
  if (!base.valid || base.window_view != &window || !base.numerics ||
      base.window_id != window.id || !(base.version == window.version) ||
      base.numerics->content_fingerprint != integrityWindowFingerprint(window) ||
      !validateFrozenWindowNumericalProof(window, *base.numerics)) {
    refuse_rank("RANK_PRECONDITION_BASE_IDENTITY");
    shape_safe = false;
  }

  std::set<FactorGroupId> requested_removals;
  std::set<FactorGroupId> requested_additions;
  for (const auto id : action.groups_to_remove) {
    if (!requested_removals.insert(id).second) {
      refuse_rank("RANK_PRECONDITION_DUPLICATE_REMOVAL");
      shape_safe = false;
    }
  }
  for (const auto id : action.groups_to_add) {
    if (!requested_additions.insert(id).second) {
      refuse_rank("RANK_PRECONDITION_DUPLICATE_ADDITION");
      shape_safe = false;
    }
  }
  if (requested_additions.size() != action.added_blocks.size()) {
    refuse_rank("RANK_PRECONDITION_ADDITION_MAPPING");
    shape_safe = false;
  }
  for (const auto& added : action.added_blocks) {
    if (!requested_additions.count(added.group_id)) {
      refuse_rank("RANK_PRECONDITION_ADDITION_MAPPING");
      shape_safe = false;
    }
    ++safety.added_blocks;
    safety.changed_rows += static_cast<std::size_t>(
        std::max<Eigen::Index>(0, added.jacobian_whitened.rows()));
  }

  std::set<FactorGroupId> observed_removals;
  std::vector<const LinearizedFactorBlock*> changed_blocks;
  for (const auto& block : window.blocks) {
    if (!requested_removals.count(block.group_id)) continue;
    observed_removals.insert(block.group_id);
    changed_blocks.push_back(&block);
    ++safety.removed_blocks;
    safety.changed_rows += static_cast<std::size_t>(
        std::max<Eigen::Index>(0, block.jacobian_whitened.rows()));
  }
  if (observed_removals != requested_removals) {
    refuse_rank("RANK_PRECONDITION_REMOVAL_MAPPING");
    shape_safe = false;
  }
  for (const auto& added : action.added_blocks) changed_blocks.push_back(&added);

  // RankUpdateEvaluator assigns one frozen-information solve per changed
  // block into a fixed row slice.  Certify that exact shape and finiteness
  // before the assignment; an empty/short solve was the concrete NDEBUG crash
  // mechanism reproduced by the fourth review.
  if (shape_safe) {
    for (const auto* block : changed_blocks) {
      const Eigen::MatrixXd solved = solveFrozenInformation(
          window.square_root.get(), *base.numerics, window.base_information,
          block->jacobian_whitened.transpose());
      if (solved.rows() != window.H.cols() ||
          solved.cols() != block->jacobian_whitened.rows() ||
          !solved.allFinite()) {
        refuse_rank("RANK_PRECONDITION_CHANGED_BLOCK_SOLVE");
        shape_safe = false;
        break;
      }
    }
  }

  // The exact candidate must be comfortably inside every numerical/gate
  // boundary before the optimized path is allowed.  This is only a routing
  // margin: the configured acceptance threshold is unchanged and the dense
  // result remains authoritative whenever the proof is unknown.
  const double condition_margin = 0.5 * config_.max_condition_number;
  const double step_margin = 0.5 * config_.max_linearization_step_norm;
  if (shape_safe && (!exact.valid || exact.rank != window.H.cols() ||
                     exact.dof <= 0 || !std::isfinite(exact.condition_number) ||
                     exact.condition_number > condition_margin ||
                     !std::isfinite(safety.exact_step_norm) ||
                     safety.exact_step_norm > step_margin ||
                     safety.changed_rows >
                         static_cast<std::size_t>(window.H.cols()))) {
    refuse_rank("RANK_PRECONDITION_EXACT_MARGIN");
    shape_safe = false;
  }

  if (!shape_safe) {
    safety.rank_path_safe = false;
    if (safety_output) *safety_output = safety;
    ++g_candidate_route_dense;
    return dense_terminal(safety.reason);
  }

  CandidateEvaluation result;
  try {
    result = rank_dependency_
        ? rank_dependency_(base, action, scratch)
        : rank_.evaluate(base, action, scratch);
  } catch (const std::exception& error) {
    safety.rank_path_safe = false;
    safety.reason = std::string("RANK_EXCEPTION_DENSE_EXACT:") + error.what();
    if (safety_output) *safety_output = safety;
    ++g_candidate_route_dense;
    ++g_candidate_route_rank_exception;
    return dense_terminal(safety.reason);
  } catch (...) {
    safety.rank_path_safe = false;
    safety.reason = "RANK_EXCEPTION_DENSE_EXACT:unknown";
    if (safety_output) *safety_output = safety;
    ++g_candidate_route_dense;
    ++g_candidate_route_rank_exception;
    return dense_terminal(safety.reason);
  }
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(
      window.H.cols(), window.H.cols());
  const Eigen::MatrixXd rank_covariance = result.covarianceTimes(identity);
  const Eigen::MatrixXd exact_covariance = exact.covarianceTimes(identity);
  const double state_scale = std::max(
      {1.0, result.state_increment.norm(), exact.state_increment.norm()});
  const double covariance_scale = std::max(
      {1.0, rank_covariance.norm(), exact_covariance.norm()});
  const double statistic_scale = std::max(
      {1.0, std::abs(result.statistic), std::abs(exact.statistic)});
  const bool same_terminal = result.valid == exact.valid &&
      result.rank == exact.rank && result.dof == exact.dof &&
      result.state_increment.size() == exact.state_increment.size() &&
      rank_covariance.rows() == window.H.cols() &&
      rank_covariance.cols() == window.H.cols() &&
      exact_covariance.rows() == window.H.cols() &&
      exact_covariance.cols() == window.H.cols() &&
      result.state_increment.allFinite() && rank_covariance.allFinite() &&
      (result.state_increment - exact.state_increment).norm() <=
          1e-7 * state_scale &&
      (rank_covariance - exact_covariance).norm() <=
          1e-7 * covariance_scale &&
      std::isfinite(result.statistic) && std::isfinite(exact.statistic) &&
      std::abs(result.statistic - exact.statistic) <=
          1e-7 * statistic_scale;
  if (!same_terminal) {
    safety.rank_path_safe = false;
    safety.reason = "RANK_POSTCONDITION_EXACT_MISMATCH";
    if (safety_output) *safety_output = safety;
    ++g_candidate_route_dense;
    return dense_terminal(safety.reason);
  }

  safety.rank_path_safe = true;
  safety.reason = "RANK_SAFETY_CERTIFIED_V1";
  if (safety_output) *safety_output = safety;
  ++g_candidate_route_rank;
  if (result.diagnostics.fallback_reason.empty()) {
    result.diagnostics.fallback_reason = safety.reason;
  } else {
    result.diagnostics.fallback_reason = safety.reason + ";" +
        result.diagnostics.fallback_reason;
  }
  return result;
}

}  // namespace uwb_imu_pl
