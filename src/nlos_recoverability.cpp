#include <Eigen/Cholesky>
#include "uifgo/nlos_recoverability.h"

#include <Eigen/Eigenvalues>
#include <Eigen/SparseQR>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <utility>

namespace uifgo {
namespace {

using Sparse = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;

bool OptionsValid(const RecoverabilityOptions& o) {
  const double values[] = {
      o.rank_absolute_tolerance,
      o.rank_relative_tolerance,
      o.symmetry_absolute_tolerance,
      o.symmetry_relative_tolerance,
      o.psd_absolute_tolerance_m2_inv,
      o.psd_relative_tolerance,
      o.pd_absolute_tolerance_m2_inv,
      o.pd_relative_tolerance,
      o.eta_absolute_tolerance,
      o.orthogonality_absolute_tolerance,
      o.orthogonality_relative_tolerance,
  };
  for (double value : values)
    if (!std::isfinite(value) || value < 0.0) return false;
  return std::isfinite(o.roundoff_safety_factor) &&
         o.roundoff_safety_factor > 0.0 &&
         o.condition_estimator_max_iterations > 0;
}

double StableSparseColumnNorm(const Sparse& matrix, Eigen::Index column) {
  double maximum = 0.0;
  for (Sparse::InnerIterator it(matrix, column); it; ++it) {
    maximum = std::max(maximum, std::abs(it.value()));
  }
  if (maximum == 0.0) return 0.0;
  if (!std::isfinite(maximum))
    return std::numeric_limits<double>::quiet_NaN();
  double scaled_square_sum = 0.0;
  for (Sparse::InnerIterator it(matrix, column); it; ++it) {
    const double ratio = it.value() / maximum;
    scaled_square_sum += ratio * ratio;
  }
  const double scaled_norm = std::sqrt(scaled_square_sum);
  if (!(scaled_norm > 0.0) || !std::isfinite(scaled_norm))
    return std::numeric_limits<double>::quiet_NaN();
  if (maximum > std::numeric_limits<double>::max() / scaled_norm)
    return std::numeric_limits<double>::infinity();
  return maximum * scaled_norm;
}

bool SameSparseColumn(const Sparse& matrix, Eigen::Index lhs,
                      Eigen::Index rhs, double sign) {
  Sparse::InnerIterator a(matrix, lhs);
  Sparse::InnerIterator b(matrix, rhs);
  while (a && b) {
    if (a.row() != b.row() || a.value() != sign * b.value()) return false;
    ++a;
    ++b;
  }
  return !a && !b;
}

double SparseOneNorm(const Sparse& matrix) {
  double maximum = 0.0;
  for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
    double sum = 0.0;
    for (Sparse::InnerIterator it(matrix, column); it; ++it)
      sum += std::abs(it.value());
    maximum = std::max(maximum, sum);
  }
  return maximum;
}

struct ConditionEstimate {
  double condition = std::numeric_limits<double>::infinity();
  size_t iterations = 0;
  bool converged = false;
};

struct InverseFrobeniusBound {
  double inverse_norm_upper = std::numeric_limits<double>::infinity();
  double residual_norm = std::numeric_limits<double>::infinity();
  bool valid = false;
};

// Certify an upper bound on ||R^-1||_2 without retaining an inverse.  If the
// individually solved columns form X and D=I-RX, then
// R^-1=X(I-D)^-1 and ||R^-1||_2 <= ||X||_F/(1-||D||_F) for ||D||_F<1.
InverseFrobeniusBound BoundTriangularInverseFrobenius(const Sparse& upper) {
  InverseFrobeniusBound result;
  const Eigen::Index n = upper.rows();
  if (n == 0 || upper.cols() != n) return result;
  long double inverse_square_sum = 0.0L;
  long double residual_square_sum = 0.0L;
  for (Eigen::Index column = 0; column < n; ++column) {
    Eigen::VectorXd unit = Eigen::VectorXd::Zero(n);
    unit[column] = 1.0;
    const Eigen::VectorXd solution =
        upper.triangularView<Eigen::Upper>().solve(unit);
    if (!solution.allFinite()) return result;
    const Eigen::VectorXd residual = upper * solution - unit;
    if (!residual.allFinite()) return result;
    inverse_square_sum +=
        static_cast<long double>(solution.squaredNorm());
    residual_square_sum +=
        static_cast<long double>(residual.squaredNorm());
  }
  const double inverse_frobenius =
      std::sqrt(static_cast<double>(inverse_square_sum));
  result.residual_norm =
      std::sqrt(static_cast<double>(residual_square_sum));
  if (!(inverse_frobenius > 0.0) || !std::isfinite(inverse_frobenius) ||
      !std::isfinite(result.residual_norm) || result.residual_norm >= 1.0)
    return result;
  result.inverse_norm_upper =
      inverse_frobenius / (1.0 - result.residual_norm);
  result.valid = std::isfinite(result.inverse_norm_upper) &&
                 result.inverse_norm_upper > 0.0;
  return result;
}

ConditionEstimate EstimateTriangularConditionOneNorm(const Sparse& upper,
                                                     size_t max_iterations) {
  ConditionEstimate result;
  const Eigen::Index n = upper.rows();
  if (n == 0) {
    result.condition = 1.0;
    result.converged = true;
    return result;
  }
  if (upper.cols() != n) return result;
  const double matrix_norm = SparseOneNorm(upper);
  if (!(matrix_norm > 0.0) || !std::isfinite(matrix_norm)) return result;

  Sparse lower = upper.transpose();
  Eigen::VectorXd x = Eigen::VectorXd::Constant(n, 1.0 / n);
  double inverse_norm = 0.0;
  Eigen::Index previous_index = -1;
  for (size_t iteration = 1; iteration <= max_iterations; ++iteration) {
    result.iterations = iteration;
    const Eigen::VectorXd y =
        upper.triangularView<Eigen::Upper>().solve(x);
    if (!y.allFinite()) return result;
    inverse_norm = std::max(inverse_norm, y.lpNorm<1>());
    Eigen::VectorXd signs = y.unaryExpr(
        [](double value) { return value >= 0.0 ? 1.0 : -1.0; });
    const Eigen::VectorXd z =
        lower.triangularView<Eigen::Lower>().solve(signs);
    if (!z.allFinite()) return result;
    Eigen::Index index = 0;
    const double z_max = z.cwiseAbs().maxCoeff(&index);
    const double directional = z.dot(x);
    if (z_max <= directional +
                     8.0 * std::numeric_limits<double>::epsilon() * z_max ||
        index == previous_index) {
      result.converged = true;
      break;
    }
    previous_index = index;
    x.setZero();
    x[index] = 1.0;
  }
  result.condition = matrix_norm * inverse_norm;
  if (!std::isfinite(result.condition)) result.converged = false;
  return result;
}

int SymmetricRankAtThreshold(const Eigen::VectorXd& eigenvalues,
                             double threshold) {
  if (eigenvalues.size() == 0) return 0;
  int rank = 0;
  for (Eigen::Index i = 0; i < eigenvalues.size(); ++i)
    if (eigenvalues[i] > threshold) ++rank;
  return rank;
}

double GammaK(size_t k) {
  const double unit_roundoff = std::numeric_limits<double>::epsilon() / 2.0;
  const double product = static_cast<double>(k) * unit_roundoff;
  if (!(product < 1.0)) return std::numeric_limits<double>::infinity();
  return product / (1.0 - product);
}

}  // namespace

const char* RecoverabilityStatusName(RecoverabilityStatus status) {
  switch (status) {
    case RecoverabilityStatus::OK:
      return "OK";
    case RecoverabilityStatus::RANK_DEFICIENT:
      return "RANK_DEFICIENT";
    case RecoverabilityStatus::SPARSE_RANK_UNCERTAIN:
      return "SPARSE_RANK_UNCERTAIN";
    case RecoverabilityStatus::NUMERICAL_RESOLUTION_LOST:
      return "NUMERICAL_RESOLUTION_LOST";
    case RecoverabilityStatus::NUMERICAL_FAILURE:
      return "NUMERICAL_FAILURE";
    case RecoverabilityStatus::INVALID_INPUT:
      return "INVALID_INPUT";
  }
  return "UNKNOWN";
}

RecoverabilityResult ComputeSparseRecoverability(
    const Sparse& input_F, const Eigen::MatrixXd& G,
    const RecoverabilityOptions& options) {
  RecoverabilityResult out;
  auto fail = [&](RecoverabilityStatus status, const std::string& reason) {
    out.status = status;
    out.reason = reason;
    out.s_is_infinite = true;
    out.s_m = std::numeric_limits<double>::infinity();
    out.eta = std::numeric_limits<double>::quiet_NaN();
    return out;
  };

  out.rows = static_cast<size_t>(input_F.rows());
  out.nuisance_columns_total = static_cast<size_t>(input_F.cols());
  out.amplitude_columns = static_cast<size_t>(G.cols());
  if (!OptionsValid(options) || input_F.rows() != G.rows() || G.cols() == 0 ||
      input_F.rows() == 0 || !G.allFinite()) {
    return fail(RecoverabilityStatus::INVALID_INPUT,
                "invalid dimensions, options, or nonfinite G");
  }
  for (Eigen::Index column = 0; column < input_F.outerSize(); ++column)
    for (Sparse::InnerIterator it(input_F, column); it; ++it)
      if (!std::isfinite(it.value()))
        return fail(RecoverabilityStatus::INVALID_INPUT, "nonfinite F");

  // Stable deterministic 2-norm equilibration, with exact zero decided from
  // original coefficients.  F_scaled_full exists only as a sparse matrix.
  std::vector<Triplet> scaled_triplets;
  out.nuisance_columns.resize(input_F.cols());
  for (Eigen::Index column = 0; column < input_F.cols(); ++column) {
    NuisanceColumnAudit audit;
    audit.original_index = static_cast<size_t>(column);
    const double norm = StableSparseColumnNorm(input_F, column);
    if (!std::isfinite(norm))
      return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                  "nuisance column norm is not representable");
    audit.original_norm = norm;
    audit.exact_zero = norm == 0.0;
    audit.scale = audit.exact_zero ? 0.0 : 1.0 / norm;
    if (audit.exact_zero) ++out.exact_zero_columns;
    for (Sparse::InnerIterator it(input_F, column); it; ++it) {
      const double value = it.value() * audit.scale;
      if (!std::isfinite(value))
        return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                    "scaled nuisance coefficient is nonfinite");
      if (value != 0.0) scaled_triplets.emplace_back(it.row(), column, value);
    }
    out.nuisance_columns[column] = audit;
  }
  Sparse F_scaled_full(input_F.rows(), input_F.cols());
  F_scaled_full.setFromTriplets(scaled_triplets.begin(), scaled_triplets.end());
  F_scaled_full.makeCompressed();

  // Record the naive Eigen QR result as a diagnostic only.  It never decides
  // the reportable frozen-SVD rank.
  if (input_F.cols() > 0) {
    Eigen::SparseQR<Sparse, Eigen::COLAMDOrdering<int>> naive;
    naive.setPivotThreshold(
        std::max(options.rank_absolute_tolerance,
                 options.rank_relative_tolerance));
    naive.compute(F_scaled_full);
    if (naive.info() == Eigen::Success) out.naive_qr_pivot_rank = naive.rank();
  }

  // Remove only exact zero and exact duplicate/sign-duplicate columns from the
  // active QR view.  The audit retains their original-column mapping.
  std::vector<int> representatives;
  for (Eigen::Index column = 0; column < F_scaled_full.cols(); ++column) {
    auto& audit = out.nuisance_columns[column];
    if (audit.exact_zero) continue;
    int duplicate_of = -1;
    for (int representative : representatives) {
      if (SameSparseColumn(F_scaled_full, column, representative, 1.0) ||
          SameSparseColumn(F_scaled_full, column, representative, -1.0)) {
        duplicate_of = representative;
        break;
      }
    }
    if (duplicate_of >= 0) {
      audit.exact_duplicate = true;
      audit.representative_original_index = duplicate_of;
      ++out.exact_duplicate_columns;
      continue;
    }
    audit.active_index = static_cast<int>(representatives.size());
    audit.representative_original_index = static_cast<int>(column);
    representatives.push_back(static_cast<int>(column));
  }
  out.nuisance_columns_active = representatives.size();

  std::vector<Triplet> active_triplets;
  for (size_t active = 0; active < representatives.size(); ++active) {
    for (Sparse::InnerIterator it(F_scaled_full, representatives[active]); it;
         ++it) {
      active_triplets.emplace_back(it.row(), static_cast<int>(active),
                                   it.value());
    }
  }
  Sparse F(input_F.rows(), static_cast<Eigen::Index>(representatives.size()));
  F.setFromTriplets(active_triplets.begin(), active_triplets.end());
  F.makeCompressed();
  out.F_scaled = F_scaled_full;
  out.N = G.transpose() * G;
  if (!out.N.allFinite())
    return fail(RecoverabilityStatus::NUMERICAL_FAILURE, "N is nonfinite");

  if (F.cols() == 0) {
    out.E = G;
    out.R = out.N;
    out.sparse_qr_rank = 0;
    out.frozen_F_rank = 0;
    out.frozen_rank_certified = true;
    out.rank_threshold_lower = options.rank_absolute_tolerance;
    out.rank_threshold_upper = options.rank_absolute_tolerance;
  } else {
    if (F.cols() > F.rows()) {
      return fail(RecoverabilityStatus::SPARSE_RANK_UNCERTAIN,
                  "underdetermined active nuisance view has unresolved dependencies");
    }
    Eigen::SparseQR<Sparse, Eigen::COLAMDOrdering<int>> qr;
    // A zero pivot threshold keeps the factorization separate from the frozen
    // SVD rank definition.  The QR rank below is diagnostic; a score is only
    // produced after the independent full-scaled-F certificate succeeds.
    qr.setPivotThreshold(0.0);
    qr.compute(F);
    if (qr.info() != Eigen::Success)
      return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                  "SparseQR factorization failed");
    out.sparse_qr_rank = qr.rank();
    const auto permutation = qr.colsPermutation().indices();
    for (Eigen::Index i = 0; i < permutation.size(); ++i)
      out.active_qr_permutation.push_back(permutation[i]);

    Sparse R_all = qr.matrixR();
    const Eigen::Index diagonal_count =
        std::min(R_all.rows(), R_all.cols());
    std::vector<double> pivots;
    for (Eigen::Index i = 0; i < diagonal_count; ++i)
      pivots.push_back(std::abs(R_all.coeff(i, i)));
    std::sort(pivots.begin(), pivots.end(), std::greater<double>());
    out.qr_pivots_desc = pivots;
    // Every nonzero column of the complete scaled F has unit 2-norm.  Hence
    // 1 <= sigma_max(F_scaled) <= ||F_scaled||_F=sqrt(nonzero columns).  Exact
    // duplicate removal preserves the column space but not sigma_max, so the
    // upper rank threshold deliberately uses the complete column count.
    const size_t nonzero_columns =
        out.nuisance_columns_total - out.exact_zero_columns;
    const double spectral_lower = 1.0;
    const double spectral_upper =
        std::sqrt(static_cast<double>(nonzero_columns));
    out.rank_threshold_lower =
        std::max(options.rank_absolute_tolerance,
                 options.rank_relative_tolerance * spectral_lower);
    out.rank_threshold_upper =
        std::max(options.rank_absolute_tolerance,
                 options.rank_relative_tolerance * spectral_upper);
    const Eigen::Index rank = out.sparse_qr_rank;
    if (rank != F.cols())
      return fail(RecoverabilityStatus::SPARSE_RANK_UNCERTAIN,
                  "active QR view is not structurally full column rank");
    if (rank > 0) {
      std::vector<Triplet> r11_triplets;
      for (Eigen::Index column = 0; column < rank; ++column) {
        for (Sparse::InnerIterator it(R_all, column); it; ++it) {
          if (it.row() < rank)
            r11_triplets.emplace_back(it.row(), column, it.value());
        }
      }
      Sparse R11(rank, rank);
      R11.setFromTriplets(r11_triplets.begin(), r11_triplets.end());
      R11.makeCompressed();
      out.retained_pivot_min = std::numeric_limits<double>::infinity();
      for (Eigen::Index i = 0; i < rank; ++i) {
        const double pivot = std::abs(R11.coeff(i, i));
        out.retained_pivot_min = std::min(out.retained_pivot_min, pivot);
        out.retained_pivot_max = std::max(out.retained_pivot_max, pivot);
      }
      const ConditionEstimate estimate = EstimateTriangularConditionOneNorm(
          R11, options.condition_estimator_max_iterations);
      out.qr_condition_estimate_1 = estimate.condition;
      out.qr_rcond_estimate_1 = 1.0 / estimate.condition;
      out.condition_estimator_iterations = estimate.iterations;
      out.condition_estimator_converged = estimate.converged;
      if (!estimate.converged || !std::isfinite(estimate.condition))
        return fail(RecoverabilityStatus::NUMERICAL_RESOLUTION_LOST,
                    "R11 condition estimator did not converge");

      const InverseFrobeniusBound inverse_bound =
          BoundTriangularInverseFrobenius(R11);
      out.rank_certificate_inverse_residual = inverse_bound.residual_norm;
      out.rank_certificate_sigma_max_upper = spectral_upper;
      if (!inverse_bound.valid)
        return fail(RecoverabilityStatus::SPARSE_RANK_UNCERTAIN,
                    "triangular inverse-norm certificate is unresolved");
      const double qr_backward_error =
          options.roundoff_safety_factor *
          GammaK(out.rows + out.nuisance_columns_active) *
          std::sqrt(static_cast<double>(out.nuisance_columns_active));
      out.rank_certificate_sigma_min_lower =
          std::max(0.0, 1.0 / inverse_bound.inverse_norm_upper -
                            qr_backward_error);
      if (!std::isfinite(out.rank_certificate_sigma_min_lower) ||
          out.rank_certificate_sigma_min_lower <= out.rank_threshold_upper) {
        return fail(RecoverabilityStatus::SPARSE_RANK_UNCERTAIN,
                    "full scaled-F frozen SVD rank cannot be certified");
      }
      out.frozen_F_rank = static_cast<int>(out.nuisance_columns_active);
      out.frozen_rank_certified = true;
    }

    const size_t roundoff_index =
        out.rows + 2 * static_cast<size_t>(out.sparse_qr_rank) +
        out.amplitude_columns;
    const double gamma = GammaK(roundoff_index);
    out.roundoff_multiplier =
        options.roundoff_safety_factor * out.qr_condition_estimate_1 * gamma;
    const double G_norm = G.norm();
    out.projection_residual_floor = out.roundoff_multiplier * G_norm;
    if (!std::isfinite(out.roundoff_multiplier) ||
        out.roundoff_multiplier >= 1.0 ||
        out.projection_residual_floor >= G_norm) {
      return fail(RecoverabilityStatus::NUMERICAL_RESOLUTION_LOST,
                  "sparse projection roundoff audit lost resolution");
    }

    const Eigen::MatrixXd Y = qr.solve(G);
    if (qr.info() != Eigen::Success || !Y.allFinite())
      return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                  "SparseQR least-squares solve failed");
    out.E = G - F * Y;
    if (!out.E.allFinite())
      return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                  "least-squares residual is nonfinite");
    const double normal = (F.transpose() * out.E).norm();
    // Scale the normal-equation backward error by the least-squares input,
    // not by E.  When a completely confounded large G leaves only roundoff E,
    // normalizing by E would make an honest roundoff residual look large.
    const double denominator = F.norm() * G.norm();
    if (denominator > 0.0) {
      out.orthogonality_residual = normal / denominator;
      out.orthogonality_tolerance =
          options.orthogonality_absolute_tolerance / denominator +
          options.orthogonality_relative_tolerance +
          out.roundoff_multiplier;
    } else {
      out.orthogonality_residual = normal;
      out.orthogonality_tolerance =
          options.orthogonality_absolute_tolerance;
    }
    if (!std::isfinite(out.orthogonality_residual) ||
        !std::isfinite(out.orthogonality_tolerance) ||
        out.orthogonality_residual > out.orthogonality_tolerance) {
      return fail(RecoverabilityStatus::NUMERICAL_RESOLUTION_LOST,
                  "least-squares orthogonality audit failed");
    }
    const Eigen::MatrixXd raw_R = out.E.transpose() * out.E;
    out.R = 0.5 * (raw_R + raw_R.transpose());
  }

  if (!out.R.allFinite())
    return fail(RecoverabilityStatus::NUMERICAL_FAILURE, "R is nonfinite");
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> n_solver(out.N);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> r_solver(out.R);
  if (n_solver.info() != Eigen::Success || r_solver.info() != Eigen::Success)
    return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                "small information eigensolver failed");
  const Eigen::VectorXd n_eigen = n_solver.eigenvalues();
  const Eigen::VectorXd r_eigen = r_solver.eigenvalues();
  const double n_scale = n_eigen.cwiseAbs().maxCoeff();
  const double r_scale = r_eigen.cwiseAbs().maxCoeff();
  out.N_rank_pd_threshold =
      std::max(options.pd_absolute_tolerance_m2_inv,
               options.pd_relative_tolerance * n_scale);
  out.R_psd_threshold =
      std::max(options.psd_absolute_tolerance_m2_inv,
               options.psd_relative_tolerance * r_scale);
  out.R_rank_pd_threshold = std::max(
      {options.pd_absolute_tolerance_m2_inv,
       options.pd_relative_tolerance * r_scale,
       out.projection_residual_floor * out.projection_residual_floor});
  out.N_rank = SymmetricRankAtThreshold(n_eigen, out.N_rank_pd_threshold);
  out.R_rank = SymmetricRankAtThreshold(r_eigen, out.R_rank_pd_threshold);
  if (n_eigen.minCoeff() <= out.N_rank_pd_threshold)
    return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                "N is not positive definite at declared tolerance");
  if (r_eigen.minCoeff() < -out.R_psd_threshold)
    return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                "R is not positive semidefinite at declared tolerance");

  const Eigen::MatrixXd n_minus_r =
      0.5 * ((out.N - out.R) + (out.N - out.R).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> n_minus_r_solver(n_minus_r);
  if (n_minus_r_solver.info() != Eigen::Success)
    return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                "N-R eigensolver failed");
  out.n_minus_r_min_eigenvalue =
      n_minus_r_solver.eigenvalues().minCoeff();
  // Operation-error bound in information units.  It scales only with N/R and
  // binary64 gamma terms; in particular it has no fixed absolute floor and
  // does not use the cancellation-prone spectrum of N-R as a scale.
  out.n_minus_r_psd_tolerance = options.roundoff_safety_factor *
      (GammaK(out.rows) + GammaK(out.amplitude_columns)) *
      (n_scale + r_scale) + 2.0 * out.roundoff_multiplier * n_scale;
  if (!std::isfinite(out.n_minus_r_min_eigenvalue) ||
      !std::isfinite(out.n_minus_r_psd_tolerance))
    return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                "N-R audit is nonfinite");
  if (out.n_minus_r_min_eigenvalue < -out.n_minus_r_psd_tolerance)
    return fail(RecoverabilityStatus::NUMERICAL_RESOLUTION_LOST,
                "N-R is not positive semidefinite within sparse roundoff audit");

  const Eigen::MatrixXd n_inverse_sqrt =
      n_solver.eigenvectors() *
      n_eigen.array().rsqrt().matrix().asDiagonal() *
      n_solver.eigenvectors().transpose();
  const Eigen::MatrixXd generalized =
      0.5 * (n_inverse_sqrt * out.R * n_inverse_sqrt +
             (n_inverse_sqrt * out.R * n_inverse_sqrt).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> generalized_solver(generalized);
  if (generalized_solver.info() != Eigen::Success)
    return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                "generalized information eigensolver failed");
  const double eta_raw = generalized_solver.eigenvalues().minCoeff();
  if (eta_raw < -options.eta_absolute_tolerance ||
      eta_raw > 1.0 + options.eta_absolute_tolerance)
    return fail(RecoverabilityStatus::NUMERICAL_FAILURE,
                "eta lies outside its roundoff interval");
  out.eta = std::min(1.0, std::max(0.0, eta_raw));

  if (r_eigen.minCoeff() <= out.R_rank_pd_threshold) {
    out.status = RecoverabilityStatus::RANK_DEFICIENT;
    out.reason = "R is not positive definite at declared sparse tolerance";
    out.s_is_infinite = true;
    out.s_m = std::numeric_limits<double>::infinity();
  } else {
    out.status = RecoverabilityStatus::OK;
    out.reason = "all sparse numerical audits passed";
    out.s_is_infinite = false;
    out.s_m = 1.0 / std::sqrt(r_eigen.minCoeff());
  }
  return out;
}

}  // namespace uifgo

namespace uifgo {
std::vector<double> LocalAmplitudeSigmas(const RecoverabilityResult& r,
                                        std::string* reason) {
  auto fail = [&](const char* why) { if (reason) *reason = why;
    return std::vector<double>{}; };
  const auto n = r.R.rows();
  if (r.status != RecoverabilityStatus::OK || n == 0 || r.R.cols() != n ||
      r.R_rank != n || r.amplitude_columns != static_cast<size_t>(n) ||
      !r.R.allFinite() || !std::isfinite(r.R_rank_pd_threshold) ||
      r.R_rank_pd_threshold < 0)
    return fail("LOCAL_SIGMA_INVALID_OR_RANK_DEFICIENT");
  const RecoverabilityOptions tol;
  if ((r.R-r.R.transpose()).norm() > tol.symmetry_absolute_tolerance +
      tol.symmetry_relative_tolerance*r.R.norm())
    return fail("LOCAL_SIGMA_ASYMMETRIC");
  const Eigen::MatrixXd symmetric = 0.5*(r.R+r.R.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(symmetric);
  if (eig.info()!=Eigen::Success || !eig.eigenvalues().allFinite() ||
      eig.eigenvalues().minCoeff() <= std::max(r.R_rank_pd_threshold,
        std::max(tol.pd_absolute_tolerance_m2_inv,
                 tol.pd_relative_tolerance*eig.eigenvalues().cwiseAbs().maxCoeff())))
    return fail("LOCAL_SIGMA_NOT_POSITIVE_DEFINITE");
  Eigen::LLT<Eigen::MatrixXd> llt(symmetric);
  if (llt.info()!=Eigen::Success) return fail("LOCAL_SIGMA_CHOLESKY_FAILED");
  std::vector<double> result;
  for (Eigen::Index i=0;i<n;++i) {
    const Eigen::VectorXd x=llt.solve(Eigen::VectorXd::Unit(n,i));
    if (llt.info()!=Eigen::Success || !x.allFinite() || !(x[i]>0))
      return fail("LOCAL_SIGMA_SOLVE_FAILED");
    result.push_back(std::sqrt(x[i]));
  }
  if(reason) *reason="LOCAL_SIGMA_AVAILABLE";
  return result;
}
}  // namespace uifgo
