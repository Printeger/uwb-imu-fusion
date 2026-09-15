#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <cstdlib>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <set>
#include <cstring>

namespace uwb_imu_pl {
namespace {

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

std::uint64_t windowFingerprint(const LinearizedIntegrityWindow& window) {
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
  for (const auto& block : window.blocks) {
    const auto group = block.group_id.value();
    hashBytes(&hash, &group, sizeof(group));
    hashMatrix(&hash, block.jacobian_whitened);
    hashMatrix(&hash, block.residual_whitened);
  }
  return hash;
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
  const double rank_gate = rank_tolerance * std::max(1.0, singular_max_upper);
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

bool covarianceFromInformation(const Eigen::MatrixXd& information,
                               Eigen::MatrixXd* covariance) {
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
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(h);
  const auto s = svd.singularValues();
  const double largest = s.size() ? s(0) : 0.0;
  const double threshold = rank_tolerance * std::max(1.0, largest);
  result->rank = static_cast<int>((s.array() > threshold).count());
  result->rows = h.rows();
  result->dof = result->rows - result->rank;
  const double smallest = result->rank > 0 ? s(result->rank - 1) : 0.0;
  result->condition_number = smallest > 0.0 ? largest / smallest
                                             : std::numeric_limits<double>::infinity();
  const Eigen::VectorXd residual = z - h * result->state_increment;
  result->statistic = residual.squaredNorm();
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

Eigen::MatrixXd CandidateEvaluation::covarianceTimes(
    const Eigen::MatrixXd& value) const {
  if (covariance.size() != 0) return covariance * value;
  if (reference_vectors) {
    if (value.rows() != reference_vectors->rows()) return Eigen::MatrixXd();
    return *reference_vectors * reference_inverse_squared.asDiagonal() * reference_vectors->transpose() * value;
  }
  if (!shared_base_factorization ||
      shared_base_factorization->matrixL().rows() != value.rows()) {
    return Eigen::MatrixXd();
  }
  Eigen::MatrixXd result = shared_base_factorization->solve(value);
  if (covariance_plus_factor.cols()) {
    result.noalias() += covariance_plus_factor *
        (covariance_plus_factor.transpose() * value);
  }
  if (covariance_minus_factor.cols()) {
    result.noalias() -= covariance_minus_factor *
        (covariance_minus_factor.transpose() * value);
  }
  return result;
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
  base.window = window;
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
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(window.H);
  const auto singular = svd.singularValues();
  const double largest = singular.size() ? singular(0) : 0.0;
  base.exact_rank = (singular.array() > config_.rank_tolerance * std::max(1.0, largest)).count();
  base.smallest_singular_value = base.exact_rank ? singular(base.exact_rank - 1) : 0.0;
  base.exact_condition = base.smallest_singular_value > 0 ? largest / base.smallest_singular_value
      : std::numeric_limits<double>::infinity();
  auto factorization =
      std::make_shared<Eigen::LLT<Eigen::MatrixXd>>(window.base_information);
  if (factorization->info() != Eigen::Success) {
    base.reason = "base information is not SPD";
    return base;
  }
  base.information_factorization = factorization;
  base.state_increment = factorization->solve(window.base_information_rhs);
  base.information_logdet = 2.0 * factorization->matrixL().toDenseMatrix()
      .diagonal().array().log().sum();
  // A large all-in step is itself a fault symptom. Keep the one-time base
  // factorization usable so exclusion candidates can recover; apply the
  // linearization-step gate independently to every candidate below.
  base.valid = base.state_increment.allFinite();
  if (!base.valid) base.reason = "base solution contains non-finite values";
  if (base.valid) {
    auto numerics = std::make_shared<FrozenWindowNumerics>();
    numerics->window_id = window.id;
    numerics->version = window.version;
    numerics->content_fingerprint = windowFingerprint(window);
    numerics->information_factorization = factorization;
    numerics->base_state_increment = base.state_increment;
    numerics->parity = window.z - window.H * base.state_increment;
    numerics->statistic = numerics->parity.squaredNorm();
    numerics->information_logdet = base.information_logdet;
    numerics->smallest_singular_value = base.smallest_singular_value;
    numerics->largest_singular_value = largest;
    numerics->exact_rank = base.exact_rank;
    numerics->exact_condition = base.exact_condition;
    const double singular_error = 128.0 * std::numeric_limits<double>::epsilon() *
        std::max(window.H.rows(), window.H.cols()) * std::max(1.0, largest);
    const double singular_lower = std::max(
        0.0, base.smallest_singular_value - singular_error);
    const double singular_upper = largest + singular_error;
    numerics->smallest_information_lower_bound = singular_lower * singular_lower;
    numerics->largest_information_upper_bound = singular_upper * singular_upper;
    numerics->block_row_offsets.reserve(window.blocks.size() + 1);
    Eigen::Index offset = 0;
    for (const auto& block : window.blocks) {
      numerics->block_row_offsets.push_back(offset);
      offset += block.jacobian_whitened.rows();
    }
    numerics->block_row_offsets.push_back(offset);
    base.numerics = std::move(numerics);
  }
  buildSharedCache(&base, actions);
  return base;
}

void RankUpdateEvaluator::buildSharedCache(BaseCandidateKernel* kernel,
    const std::vector<ExclusionAction>& actions) const {
  if (!kernel) return;
  auto& base = *kernel;
  const auto& window = base.window;
  base.block_cache.reset();
  if (base.valid && config_.enable_shared_cache && !actions.empty() &&
      !std::getenv("UWB_IMU_PL_DISABLE_BLOCK_CACHE")) {
    auto cache = std::make_shared<FrozenBlockSolveCache>();
    cache->window_id = window.id; cache->version = window.version;
    auto add = [&](const LinearizedFactorBlock& block) {
      if (!validBlock(block, window)) return;
      auto& entries = cache->blocks[block.group_id];
      if (std::any_of(entries.begin(), entries.end(), [&](const auto& e) { return sameBlock(e.block, block); })) return;
      entries.push_back({block, base.information_factorization->solve(block.jacobian_whitened.transpose())});
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
  result.window_view = &base.window;
  result.diagnostics.kernel_evaluated = true;
  result.diagnostics.numerical_path = "ADD_THEN_REMOVE_SVD";
  auto done = [&]() {
    result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    result.diagnostics.kernel_ms = result.wall_ms;
    result.diagnostics.numerical_valid = result.valid;
    result.diagnostics.slow_path = result.exact_slow_path;
    return result;
  };
  if (!base.valid) { result.reason = base.reason; return done(); }
  // Validate the complete change before any Eigen operation or early gate.
  if (!validateAction(base.window, action, &result.reason)) return done();
  result.shared_base_factorization = base.information_factorization;
  Eigen::VectorXd rhs = base.window.base_information_rhs;
  std::vector<const LinearizedFactorBlock*> additions, removals;
  for (const auto& b : action.added_blocks) additions.push_back(&b);
  for (const auto& b : base.window.blocks)
    if (groupSelected(b.group_id, action.groups_to_remove)) removals.push_back(&b);
  auto order = [](auto& blocks) {
    std::stable_sort(blocks.begin(), blocks.end(), [](auto* a, auto* b) { return a->group_id < b->group_id; });
  };
  order(additions); order(removals);
  double energy = base.window.z.squaredNorm();
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
  update_columns.resize(base.window.H.cols(), update_count);
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
    Eigen::MatrixXd j(rows, base.window.H.cols());
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
          base.information_factorization->solve(block->jacobian_whitened.transpose());
      cache_offset += block->jacobian_whitened.rows();
    }
    if (!add && !additions.empty()) {
      const Eigen::MatrixXd legacy_inner = Eigen::MatrixXd::Identity(rows, rows) - j * cj;
      legacy_deletion_failed = Eigen::LLT<Eigen::MatrixXd>(legacy_inner).info() != Eigen::Success;
    }
    // Addition first: the downdate operates on C_add, so an absent/singular
    // deletion-only intermediate can never reject an SPD complete replacement.
    if (!add && result.covariance_minus_factor.cols())
      cj.noalias() -= result.covariance_minus_factor * (result.covariance_minus_factor.transpose() * j.transpose());
    Eigen::MatrixXd inner = Eigen::MatrixXd::Identity(rows, rows) + (add ? 1.0 : -1.0) * j * cj;
    inner = (0.5 * (inner + inner.transpose())).eval();
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
  result.rows = base.window.H.rows();
  for (auto* b : additions) result.rows += b->jacobian_whitened.rows();
  for (auto* b : removals) result.rows -= b->jacobian_whitened.rows();
  if (!rhs.allFinite() || !std::isfinite(energy)) {
    result.reason = "candidate contains non-finite information or residual energy"; return done();
  }
  result.state_increment = result.covarianceTimes(rhs);
  Eigen::MatrixXd h;
  Eigen::VectorXd z;
  const bool keep = additions.empty() && removals.empty();
  if (!base.numerics || base.numerics->content_fingerprint != windowFingerprint(base.window)) {
    request_reference("missing or stale frozen numerics");
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
      result.rank = base.window.H.cols();
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
  Eigen::VectorXd normal_residual = rhs - base.window.base_information * result.state_increment;
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
      base.window.protected_state_map.cols() == base.window.H.cols()) {
    const Eigen::MatrixXd protected_rhs = base.window.protected_state_map.transpose();
    const Eigen::MatrixXd protected_solve = result.covarianceTimes(protected_rhs);
    Eigen::MatrixXd covariance_residual = protected_rhs -
        base.window.base_information * protected_solve;
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
    if (!candidateRows(base.window, action, &h, &z)) {
      result.reason = "candidate removal block is absent from frozen window"; return done();
    }
    // Reference solve uses the final whitened Jacobian, not its squared normal
    // equations. Keep only an operator for covariance queries online.
    Eigen::JacobiSVD<Eigen::MatrixXd> reference(h, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = reference.singularValues();
    const double largest = singular.size() ? singular(0) : 0.0;
    const double gate = config_.rank_tolerance * std::max(1.0, largest);
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
    if (result.dof <= 0) {
      result.reason = "candidate has no residual dof";
      result.statistic = (z - h * result.state_increment).squaredNorm();
      return done();
    }
  }
  result.statistic = needs_reference ? (z - h * result.state_increment).squaredNorm()
                                     : std::max(0.0, subtracted);
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
    if (h.size() == 0 && !candidateRows(base.window, action, &h, &z)) {
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
  result.diagnostics.condition_value_kind = "EXACT_SVD";
  result.wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wall_start).count();
  return result;
}

}  // namespace uwb_imu_pl
