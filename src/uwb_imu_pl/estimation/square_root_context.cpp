#include "uwb_imu_pl/estimation/square_root_context.hpp"

#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <Eigen/Householder>
#include <Eigen/QR>

#include <algorithm>
#include <cstring>
#include <map>
#include <mutex>

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

std::uint64_t patternFingerprint(const Eigen::MatrixXd& matrix) {
  std::uint64_t hash = 1469598103934665603ULL;
  const int rows = static_cast<int>(matrix.rows());
  const int columns = static_cast<int>(matrix.cols());
  hashScalar(&hash, rows);
  hashScalar(&hash, columns);
  for (int row = 0; row < rows; ++row) {
    int nnz = 0;
    for (int column = 0; column < columns; ++column) {
      if (matrix(row, column) != 0.0) ++nnz;
    }
    hashScalar(&hash, nnz);
    for (int column = 0; column < columns; ++column) {
      if (matrix(row, column) != 0.0) hashScalar(&hash, column);
    }
  }
  return hash;
}

std::uint64_t buildContentFingerprint(const Eigen::MatrixXd& H,
                                      const Eigen::VectorXd& z,
                                      const Eigen::MatrixXd& protected_map) {
  std::uint64_t hash = 1469598103934665603ULL;
  auto add = [&hash](const auto& matrix) {
    const int rows = static_cast<int>(matrix.rows());
    const int columns = static_cast<int>(matrix.cols());
    hashScalar(&hash, rows);
    hashScalar(&hash, columns);
    for (int column = 0; column < columns; ++column) {
      for (int row = 0; row < rows; ++row) {
        const double value = matrix(row, column);
        hashBytes(&hash, &value, sizeof(value));
      }
    }
  };
  add(H);
  add(z);
  add(protected_map);
  return hash;
}

struct SymbolicCache {
  std::mutex mutex;
  std::map<std::uint64_t, SquareRootSymbolicPlan> plans;
  std::uint64_t hits = 0;
  std::uint64_t misses = 0;
};

SymbolicCache& symbolicCache() {
  static SymbolicCache cache;
  return cache;
}

}  // namespace

std::string toString(ColumnScalePolicy policy) {
  switch (policy) {
    case ColumnScalePolicy::Unit: return "unit";
    case ColumnScalePolicy::ColumnNorm: return "column_norm";
  }
  return "unknown";
}

std::string toString(ColumnPermutationPolicy policy) {
  switch (policy) {
    case ColumnPermutationPolicy::Natural: return "natural";
    case ColumnPermutationPolicy::ColumnPivot: return "column_pivot";
  }
  return "unknown";
}

std::uint64_t squareRootPolicyFingerprint(ColumnScalePolicy scale,
                                          ColumnPermutationPolicy permutation) {
  std::uint64_t hash = 1469598103934665603ULL;
  const int scale_value = static_cast<int>(scale);
  const int permutation_value = static_cast<int>(permutation);
  hashScalar(&hash, scale_value);
  hashScalar(&hash, permutation_value);
  return hash;
}

bool SquareRootSymbolicPlan::sameAs(const SquareRootSymbolicPlan& other) const {
  return pattern_fingerprint == other.pattern_fingerprint &&
      detector_only_rows == other.detector_only_rows &&
      pattern_column_order == other.pattern_column_order;
}

bool SquareRootCertificate::ok() const {
  return full_column_rank && identity_ok && parity_matches_reference &&
      solution_matches_reference && forward_error_ok;
}

struct FrozenSquareRootContext::ImplicitQ {
  std::shared_ptr<const Eigen::HouseholderQR<Eigen::MatrixXd>> natural;
  std::shared_ptr<const Eigen::ColPivHouseholderQR<Eigen::MatrixXd>> pivoted;
  Eigen::Index rows = 0;

  Eigen::MatrixXd applyQt(const Eigen::MatrixXd& rhs) const {
    if (natural) return natural->householderQ().adjoint() * rhs;
    return pivoted->householderQ().adjoint() * rhs;
  }
};

std::shared_ptr<const FrozenSquareRootContext> FrozenSquareRootContext::build(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& protected_state_map,
    ColumnScalePolicy scale_policy,
    ColumnPermutationPolicy permutation_policy,
    double rank_tolerance,
    const Eigen::VectorXd& reference_solution,
    double reference_statistic) {
  if (H.rows() <= 0 || H.cols() <= 0 || H.rows() != z.size() ||
      protected_state_map.cols() != H.cols()) {
    return nullptr;
  }
  auto context = std::shared_ptr<FrozenSquareRootContext>(
      new FrozenSquareRootContext());
  context->rows_ = static_cast<int>(H.rows());
  context->columns_ = static_cast<int>(H.cols());
  context->scale_policy_ = scale_policy;
  context->permutation_policy_ = permutation_policy;
  context->policy_fingerprint_ =
      squareRootPolicyFingerprint(scale_policy, permutation_policy);
  context->content_fingerprint_ =
      buildContentFingerprint(H, z, protected_state_map);

  // ---- symbolic plan (pattern analysis, cacheable) -------------------------
  const std::uint64_t pattern = patternFingerprint(H);
  SquareRootSymbolicPlan plan;
  {
    auto& cache = symbolicCache();
    std::lock_guard<std::mutex> lock(cache.mutex);
    const auto found = cache.plans.find(pattern);
    if (found != cache.plans.end()) {
      plan = found->second;
      ++cache.hits;
      NumericalWorkCounters::squareRootSymbolicHit();
    } else {
      plan.pattern_fingerprint = pattern;
      for (int row = 0; row < context->rows_; ++row) {
        if (H.row(row).cwiseAbs().maxCoeff() == 0.0) {
          plan.detector_only_rows.push_back(row);
        }
      }
      std::vector<std::pair<int, int>> degree;
      degree.reserve(static_cast<std::size_t>(context->columns_));
      for (int column = 0; column < context->columns_; ++column) {
        int nnz = 0;
        for (int row = 0; row < context->rows_; ++row) {
          if (H(row, column) != 0.0) ++nnz;
        }
        degree.emplace_back(nnz, column);
      }
      std::stable_sort(degree.begin(), degree.end());
      for (const auto& item : degree) plan.pattern_column_order.push_back(item.second);
      cache.plans.emplace(pattern, plan);
      ++cache.misses;
      NumericalWorkCounters::squareRootSymbolicMiss();
    }
  }
  context->plan_ = plan;

  // ---- scale matrix S (physical basis, recorded) ---------------------------
  context->scale_ = Eigen::VectorXd::Ones(context->columns_);
  if (scale_policy == ColumnScalePolicy::ColumnNorm) {
    for (int column = 0; column < context->columns_; ++column) {
      const double norm = H.col(column).norm();
      context->scale_(column) = norm > 0.0 ? 1.0 / norm : 1.0;
    }
  }
  {
    std::uint64_t hash = 1469598103934665603ULL;
    for (int column = 0; column < context->columns_; ++column) {
      const double value = context->scale_(column);
      hashScalar(&hash, value);
    }
    context->scale_fingerprint_ = hash;
  }

  // ---- A = (H * S) with the natural order ---------------------------------
  Eigen::MatrixXd scaled(H.rows(), H.cols());
  for (int column = 0; column < context->columns_; ++column) {
    scaled.col(column) = H.col(column) * context->scale_(column);
  }
  context->permutation_.resize(static_cast<std::size_t>(context->columns_));

  NumericalWorkCounters::squareRootFactorization();
  auto q = std::make_shared<ImplicitQ>();
  q->rows = H.rows();
  if (permutation_policy == ColumnPermutationPolicy::ColumnPivot) {
    auto pivoted = std::make_shared<const Eigen::ColPivHouseholderQR<Eigen::MatrixXd>>(
        scaled);
    // Eigen factors scaled * P = Q * R; record the right permutation so that
    // scaled.col(permutation_[j]) is the j-th column of the factored matrix.
    // R and Q come from this single factorization (no second factorization).
    const auto& indices = pivoted->colsPermutation().indices();
    for (int column = 0; column < context->columns_; ++column) {
      context->permutation_[static_cast<std::size_t>(column)] = indices(column);
    }
    Eigen::MatrixXd factored(H.rows(), H.cols());
    for (int column = 0; column < context->columns_; ++column) {
      factored.col(column) =
          scaled.col(context->permutation_[static_cast<std::size_t>(column)]);
    }
    q->pivoted = pivoted;
    context->scaled_ = std::move(factored);
    context->r_ = pivoted->matrixR()
                      .topLeftCorner(context->columns_, context->columns_)
                      .triangularView<Eigen::Upper>();
  } else {
    for (int column = 0; column < context->columns_; ++column) {
      context->permutation_[static_cast<std::size_t>(column)] = column;
    }
    q->natural = std::make_shared<const Eigen::HouseholderQR<Eigen::MatrixXd>>(
        scaled);
    context->scaled_ = std::move(scaled);
    context->r_ = q->natural->matrixQR()
                      .topLeftCorner(context->columns_, context->columns_)
                      .triangularView<Eigen::Upper>();
  }
  context->q_ = q;

  // ---- C~ = C * S * Pmat ---------------------------------------------------
  Eigen::MatrixXd scaled_map = protected_state_map;
  for (int column = 0; column < context->columns_; ++column) {
    scaled_map.col(column) *= context->scale_(column);
  }
  context->protected_map_.resize(3, context->columns_);
  for (int column = 0; column < context->columns_; ++column) {
    // Pmat is the matrix with (Pmat)_{permutation_[j], j} = 1, i.e.
    // (X Pmat).col(j) = X.col(permutation_[j]); C~ must use the same push.
    context->protected_map_.col(column) =
        scaled_map.col(context->permutation_[static_cast<std::size_t>(column)]);
  }

  // ---- rank / condition estimate ------------------------------------------
  double diagonal_min = std::numeric_limits<double>::infinity();
  double diagonal_max = 0.0;
  for (int index = 0; index < context->columns_; ++index) {
    const double magnitude = std::abs(context->r_(index, index));
    diagonal_min = std::min(diagonal_min, magnitude);
    diagonal_max = std::max(diagonal_max, magnitude);
  }
  const double gate = rank_tolerance * std::max(1.0, diagonal_max);
  context->rank_ = 0;
  for (int index = 0; index < context->columns_; ++index) {
    if (std::abs(context->r_(index, index)) > gate) ++context->rank_;
  }
  context->certificate_.r_diagonal_min = diagonal_min;
  context->certificate_.r_diagonal_max = diagonal_max;
  context->certificate_.full_column_rank =
      context->rank_ == context->columns_;
  context->certificate_.condition_estimate =
      diagonal_min > 0.0 ? diagonal_max / diagonal_min
                         : std::numeric_limits<double>::infinity();

  // ---- identities --------------------------------------------------------
  const Eigen::MatrixXd normal = context->scaled_.transpose() * context->scaled_;
  const double normal_scale =
      std::max(1e-300, normal.cwiseAbs().maxCoeff());
  context->certificate_.identity_residual_relative =
      (context->r_.transpose() * context->r_ - normal).cwiseAbs().maxCoeff() /
      normal_scale;
  context->certificate_.identity_ok =
      context->certificate_.identity_residual_relative <= 1e-12 &&
      std::isfinite(context->certificate_.identity_residual_relative);

  // Nominal increment: dx = S * P * (R^-1 * Q1^T z).
  const Eigen::MatrixXd qtz = q->applyQt(z);
  Eigen::VectorXd solution = Eigen::VectorXd::Zero(context->columns_);
  if (context->certificate_.full_column_rank) {
    const Eigen::VectorXd top = qtz.topRows(context->columns_);
    Eigen::VectorXd permuted =
        context->r_.triangularView<Eigen::Upper>().solve(top);
    for (int index = 0; index < context->columns_; ++index) {
      solution(context->permutation_[static_cast<std::size_t>(index)]) =
          permuted(index) * context->scale_(context->permutation_[static_cast<std::size_t>(index)]);
    }
  }
  context->state_increment_ = solution;
  context->parity_ = z - H * context->state_increment_;
  const double q2_statistic = qtz.bottomRows(context->rows_ - context->rank_)
                                  .squaredNorm();
  context->statistic_ = q2_statistic;

  if (reference_solution.size() == context->columns_) {
    const double reference_scale =
        std::max(1.0, reference_solution.norm());
    context->certificate_.solution_relative_difference =
        (context->state_increment_ - reference_solution).norm() / reference_scale;
    context->certificate_.solution_matches_reference =
        context->certificate_.solution_relative_difference <= 1e-8;
  } else {
    context->certificate_.solution_relative_difference = 0.0;
    context->certificate_.solution_matches_reference = true;
  }
  if (std::isfinite(reference_statistic)) {
    context->certificate_.parity_relative_difference =
        std::abs(context->statistic_ - reference_statistic) /
        std::max(1.0, std::abs(reference_statistic));
    context->certificate_.parity_matches_reference =
        context->certificate_.parity_relative_difference <= 1e-9;
  } else {
    context->certificate_.parity_relative_difference = 0.0;
    context->certificate_.parity_matches_reference = true;
  }
  // The fit residual of a least-squares problem does not vanish, so the
  // certificate uses the normal-equation (gradient) residual, scaled by the
  // triangular condition estimate: a solve is only served when A^T (A x - z)
  // is at the level of the already-achieved optimality.
  const Eigen::VectorXd gradient =
      H.transpose() * (H * context->state_increment_ - z);
  const double gradient_residual =
      gradient.norm() / std::max(1.0, (H.transpose() * z).norm());
  context->certificate_.forward_error_bound =
      context->certificate_.condition_estimate * gradient_residual;
  context->certificate_.forward_error_ok =
      std::isfinite(context->certificate_.forward_error_bound) &&
      context->certificate_.forward_error_bound <= 1e-7;

  context->usable_ = context->certificate_.ok() &&
      context->state_increment_.allFinite() && context->parity_.allFinite();
  context->reason_ = context->usable_
      ? std::string()
      : (context->rank_ != context->columns_
             ? "STATE_OUTPUT_UNOBSERVABLE: factored window is rank deficient"
             : "square-root certificate not satisfied");
  if (context->usable_) NumericalWorkCounters::squareRootCertificateHold();
  return context;
}

Eigen::MatrixXd FrozenSquareRootContext::applyQt(
    const Eigen::MatrixXd& rhs) const {
  if (!q_ || rhs.rows() != rows_) return Eigen::MatrixXd();
  NumericalWorkCounters::squareRootQtApplication(
      static_cast<std::uint64_t>(rhs.cols()));
  return q_->applyQt(rhs);
}

Eigen::MatrixXd FrozenSquareRootContext::informationSolve(
    const Eigen::MatrixXd& rhs) const {
  if (!usable_ || rhs.rows() != columns_) return Eigen::MatrixXd();
  NumericalWorkCounters::squareRootInformationSolve(
      static_cast<std::uint64_t>(rhs.cols()));
  // x = S P R^-1 R^-T P^T S rhs
  Eigen::MatrixXd scaled = rhs;
  for (int row = 0; row < columns_; ++row) scaled.row(row) *= scale_(row);
  Eigen::MatrixXd permuted(columns_, rhs.cols());
  for (int row = 0; row < columns_; ++row) {
    permuted.row(row) = scaled.row(permutation_[static_cast<std::size_t>(row)]);
  }
  const Eigen::MatrixXd intermediate =
      r_.transpose().triangularView<Eigen::Lower>().solve(permuted);
  const Eigen::MatrixXd solved =
      r_.triangularView<Eigen::Upper>().solve(intermediate);
  // (H^T H)^-1 = S P R^-1 R^-T P^T S: push through P first, then scale by S.
  // S and P do not commute, so the order of these two steps matters.
  Eigen::MatrixXd pushed(columns_, rhs.cols());
  for (int row = 0; row < columns_; ++row) {
    pushed.row(permutation_[static_cast<std::size_t>(row)]) = solved.row(row);
  }
  Eigen::MatrixXd result = pushed;
  for (int row = 0; row < columns_; ++row) {
    result.row(row) = pushed.row(row) * scale_(row);
  }
  return result;
}

Eigen::MatrixXd FrozenSquareRootContext::leastSquaresSolve(
    const Eigen::MatrixXd& rhs) const {
  if (!usable_ || rhs.rows() != rows_) return Eigen::MatrixXd();
  const Eigen::MatrixXd rotated = applyQt(rhs);
  if (rotated.rows() != rows_) return Eigen::MatrixXd();
  Eigen::MatrixXd top = rotated.topRows(columns_);
  const Eigen::MatrixXd solved =
      r_.triangularView<Eigen::Upper>().solve(top);
  // dx = S P x~: push through P first, then scale by S.
  Eigen::MatrixXd pushed(columns_, rhs.cols());
  for (int row = 0; row < columns_; ++row) {
    pushed.row(permutation_[static_cast<std::size_t>(row)]) = solved.row(row);
  }
  Eigen::MatrixXd result = pushed;
  for (int row = 0; row < columns_; ++row) {
    result.row(row) = pushed.row(row) * scale_(row);
  }
  return result;
}

void FrozenSquareRootContext::faultResponse(const Eigen::MatrixXd& dense,
                                            Eigen::MatrixXd* y,
                                            Eigen::MatrixXd* z) const {
  if (!q_ || dense.rows() != rows_) return;
  const Eigen::MatrixXd rotated = applyQt(dense);
  if (rotated.rows() != rows_) return;
  if (y) *y = rotated.topRows(rank_);
  if (z) *z = rotated.bottomRows(rows_ - rank_);
}

Eigen::MatrixXd FrozenSquareRootContext::protectedResponse() const {
  if (!usable_) return Eigen::MatrixXd();
  NumericalWorkCounters::squareRootInformationSolve(3);
  const Eigen::MatrixXd transposed =
      r_.transpose().triangularView<Eigen::Lower>().solve(
          protected_map_.transpose());
  return transposed.transpose();
}

Eigen::Matrix3d FrozenSquareRootContext::protectedCovariance() const {
  const Eigen::MatrixXd response = protectedResponse();
  if (response.rows() != 3) return Eigen::Matrix3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  return response * response.transpose();
}

FrozenSquareRootContext::SymbolicCacheStats
FrozenSquareRootContext::symbolicCacheStats() {
  auto& cache = symbolicCache();
  std::lock_guard<std::mutex> lock(cache.mutex);
  SymbolicCacheStats stats;
  stats.entries = cache.plans.size();
  stats.hits = cache.hits;
  stats.misses = cache.misses;
  return stats;
}

void FrozenSquareRootContext::clearSymbolicCache() {
  auto& cache = symbolicCache();
  std::lock_guard<std::mutex> lock(cache.mutex);
  cache.plans.clear();
  cache.hits = 0;
  cache.misses = 0;
}

}  // namespace uwb_imu_pl
