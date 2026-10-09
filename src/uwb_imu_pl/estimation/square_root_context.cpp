#include "uwb_imu_pl/estimation/square_root_context.hpp"

#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "../integrity/numerical_phase_profile.hpp"
#include "classification_numerics.hpp"

#include <Eigen/Householder>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
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

template <class Derived>
void hashMatrix(std::uint64_t* hash,
                const Eigen::MatrixBase<Derived>& matrix) {
  const Eigen::Index rows = matrix.rows();
  const Eigen::Index columns = matrix.cols();
  hashScalar(hash, rows);
  hashScalar(hash, columns);
  for (Eigen::Index column = 0; column < columns; ++column) {
    for (Eigen::Index row = 0; row < rows; ++row) {
      const double value = static_cast<double>(matrix(row, column));
      hashScalar(hash, value);
    }
  }
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

SymmetricPsdCertificate certifySymmetricPsd(
    const Eigen::MatrixXd& matrix, double rank_tolerance,
    std::uint64_t parent_proof_identity, double raw_factor_scale) {
  SymmetricPsdCertificate out;
  std::uint64_t proof = 1469598103934665603ULL;
  hashScalar(&proof, parent_proof_identity);
  hashScalar(&proof, rank_tolerance);
  hashScalar(&proof, raw_factor_scale);
  hashMatrix(&proof, matrix);
  out.proof_identity = proof;
  if (matrix.rows() <= 0 || matrix.rows() != matrix.cols() ||
      !matrix.allFinite() || !(rank_tolerance > 0.0) ||
      !std::isfinite(rank_tolerance)) {
    out.reason = "symmetric PSD input dimensions/values are invalid";
    return out;
  }
  const Eigen::Index dimension = matrix.rows();
  out.symmetric_matrix = 0.5 * (matrix + matrix.transpose());
  out.matrix_scale = out.symmetric_matrix.norm();
  out.symmetry_error = (matrix - matrix.transpose()).norm();
  const double eps = std::numeric_limits<double>::epsilon();
  out.symmetry_tolerance = 64.0 * eps *
      static_cast<double>(std::max<Eigen::Index>(1, dimension)) *
      matrix.norm();
  out.symmetric = out.symmetry_error <= out.symmetry_tolerance;
  if (!out.symmetric) {
    out.reason = "matrix is not symmetric within its rounding certificate";
    return out;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(out.symmetric_matrix);
  if (eigen.info() != Eigen::Success ||
      !eigen.eigenvalues().allFinite() ||
      !eigen.eigenvectors().allFinite()) {
    out.reason = "symmetric PSD eigensolve failed";
    return out;
  }
  out.eigenvalues = eigen.eigenvalues();
  out.eigenvectors = eigen.eigenvectors();
  out.eigenvalue_errors.resize(dimension);
  out.eigenvalue_error = 0.0;
  for (Eigen::Index index = 0; index < dimension; ++index) {
    const Eigen::VectorXd vector = out.eigenvectors.col(index);
    const double value = out.eigenvalues(index);
    const double residual =
        (out.symmetric_matrix * vector - value * vector).norm();
    const double local_roundoff = 64.0 * eps *
        static_cast<double>(std::max<Eigen::Index>(1, dimension)) *
        std::abs(value);
    out.eigenvalue_errors(index) = residual + local_roundoff;
    out.eigenvalue_error = std::max(out.eigenvalue_error,
                                    out.eigenvalue_errors(index));
  }
  const double largest = std::max(0.0, out.eigenvalues.maxCoeff());
  for (Eigen::Index index = 0; index < dimension; ++index) {
    const double value = out.eigenvalues(index);
    const double error = out.eigenvalue_errors(index);
    if (value < 0.0) {
      out.reason = value + error < 0.0
          ? "matrix is not PSD (certified negative eigenvalue)"
          : "PSD interval crosses zero and is numerically indeterminate";
      return out;
    }
  }
  out.psd = true;
  out.sigma_max = std::sqrt(largest);
  // The configured rank tolerance is in raw-factor singular-value units.
  // Gram eigenvalues have squared units, hence the exact corresponding gate.
  // There is deliberately no absolute floor: scale never turns a nonzero
  // negative/asymmetric/hidden response into zero.
  const double factor_scale = std::isfinite(raw_factor_scale) &&
      raw_factor_scale >= 0.0
      ? raw_factor_scale : std::sqrt(largest);
  out.rank_eigenvalue_gate = rank_tolerance * rank_tolerance *
      factor_scale * factor_scale;
  out.rank = 0;
  out.rank_certified = true;
  double smallest_positive = std::numeric_limits<double>::infinity();
  for (Eigen::Index index = 0; index < out.eigenvalues.size(); ++index) {
    const double value = out.eigenvalues(index);
    const double error = out.eigenvalue_errors(index);
    if (value - error > out.rank_eigenvalue_gate) {
      ++out.rank;
      smallest_positive = std::min(smallest_positive, value);
    } else if (value == 0.0 && error == 0.0) {
      continue;
    } else if (!(value + error < out.rank_eigenvalue_gate)) {
      out.rank_certified = false;
    }
  }
  if (!out.rank_certified) {
    out.reason = "rank transition interval is numerically indeterminate";
    return out;
  }
  out.sigma_min = out.rank > 0 ? std::sqrt(smallest_positive) : 0.0;
  out.condition = out.rank > 0 && out.sigma_min > 0.0
      ? out.sigma_max / out.sigma_min
      : std::numeric_limits<double>::infinity();

  hashMatrix(&proof, out.symmetric_matrix);
  hashMatrix(&proof, out.eigenvalues);
  hashMatrix(&proof, out.eigenvectors);
  hashMatrix(&proof, out.eigenvalue_errors);
  hashScalar(&proof, out.rank_eigenvalue_gate);
  hashScalar(&proof, out.rank);
  hashScalar(&proof, out.symmetric);
  hashScalar(&proof, out.psd);
  out.proof_identity = proof;
  out.valid = true;
  return out;
}

GramResponseCertificate certifyGramAndProtectedResponse(
    const Eigen::MatrixXd& gram, const Eigen::MatrixXd& protected_response,
    double rank_tolerance, std::uint64_t parent_proof_identity,
    double raw_factor_scale) {
  GramResponseCertificate out;
  out.gram = certifySymmetricPsd(gram, rank_tolerance,
                                 parent_proof_identity, raw_factor_scale);
  std::uint64_t proof = out.gram.proof_identity;
  hashMatrix(&proof, protected_response);
  out.proof_identity = proof;
  if (!out.gram.valid) {
    out.reason = out.gram.reason;
    return out;
  }
  if (protected_response.rows() != 3 ||
      protected_response.cols() != gram.cols() ||
      !protected_response.allFinite()) {
    out.reason = "protected response dimensions/values are invalid";
    return out;
  }
  const Eigen::Index dimension = gram.cols();
  const int nullity = static_cast<int>(dimension) - out.gram.rank;
  out.response_tolerance = 0.0;
  out.axis_residual.setZero();
  if (nullity == 0) {
    out.nullspace_class = GramNullspaceClass::FullRank;
  } else {
    const Eigen::MatrixXd kernel = out.gram.eigenvectors.leftCols(nullity);
    const Eigen::MatrixXd hidden = protected_response * kernel;
    for (int axis = 0; axis < 3; ++axis) {
      out.axis_residual(axis) = hidden.row(axis).norm();
    }
    const double largest_hidden = out.axis_residual.maxCoeff();
    // Fault amplitude is unbounded, so no floating tolerance can make a
    // nonzero protected remainder harmless.  Without an explicit amplitude
    // charge, only exact structural G*ker(Gram)==0 is harmless.
    out.nullspace_class = largest_hidden == 0.0
        ? GramNullspaceClass::Harmless
        : GramNullspaceClass::Dangerous;
  }

  out.protected_slopes.setZero();
  const int first_positive = static_cast<int>(dimension) - out.gram.rank;
  for (int axis = 0; axis < 3; ++axis) {
    long double quadratic = 0.0L;
    for (int index = first_positive; index < dimension; ++index) {
      const double eigenvalue = out.gram.eigenvalues(index);
      if (!(eigenvalue > out.gram.rank_eigenvalue_gate)) {
        out.nullspace_class = GramNullspaceClass::Indeterminate;
        out.reason = "positive Gram subspace is numerically indeterminate";
        return out;
      }
      const double projection = out.gram.eigenvectors.col(index).dot(
          protected_response.row(axis).transpose());
      quadratic += static_cast<long double>(projection) *
                   static_cast<long double>(projection) /
                   static_cast<long double>(eigenvalue);
    }
    out.protected_slopes(axis) = std::sqrt(
        std::max(0.0L, quadratic));
  }
  if (out.nullspace_class == GramNullspaceClass::Dangerous) {
    out.reason = "Gram nullspace has an unbounded protected response";
  }
  hashScalar(&proof, static_cast<int>(out.nullspace_class));
  hashMatrix(&proof, out.axis_residual);
  hashMatrix(&proof, out.protected_slopes);
  hashScalar(&proof, out.response_tolerance);
  out.proof_identity = proof;
  out.valid = out.nullspace_class != GramNullspaceClass::Indeterminate;
  return out;
}

namespace {
template <bool HashProof>
SymmetricPsdCertificate certifyFactorGramImpl(
    const Eigen::MatrixXd& raw_factor, const Eigen::MatrixXd& actual_gram,
    double rank_tolerance, std::uint64_t parent_proof_identity,
    double raw_factor_scale) {
  detail::NumericalPhaseScope profile(detail::NumericalProfilePhase::FactorGram);
  SymmetricPsdCertificate out;
  std::uint64_t proof = 1469598103934665603ULL;
  const std::uint64_t domain = 0x464143544f524752ULL;  // "FACTORGR"
  if constexpr (HashProof) {
    hashScalar(&proof, domain);
    hashScalar(&proof, parent_proof_identity);
    hashScalar(&proof, rank_tolerance);
    hashScalar(&proof, raw_factor_scale);
    hashMatrix(&proof, raw_factor);
    hashMatrix(&proof, actual_gram);
    out.proof_identity = proof;
  }
  if (raw_factor.rows() <= 0 || raw_factor.cols() <= 0 ||
      actual_gram.rows() != raw_factor.cols() ||
      actual_gram.cols() != raw_factor.cols() ||
      !raw_factor.allFinite() || !actual_gram.allFinite() ||
      !(rank_tolerance > 0.0) || !std::isfinite(rank_tolerance)) {
    out.reason = "factor Gram input dimensions/values are invalid";
    return out;
  }

  const Eigen::Index dimension = raw_factor.cols();
  const double eps = std::numeric_limits<double>::epsilon();
  out.symmetric_matrix = 0.5 * (actual_gram + actual_gram.transpose());
  out.matrix_scale = out.symmetric_matrix.norm();
  out.symmetry_error = (actual_gram - actual_gram.transpose()).norm();
  out.symmetry_tolerance = 64.0 * eps *
      static_cast<double>(std::max<Eigen::Index>(1, dimension)) *
      actual_gram.norm();
  out.symmetric = out.symmetry_error <= out.symmetry_tolerance;
  if (!out.symmetric) {
    out.reason = "factor Gram is not symmetric within its rounding certificate";
    return out;
  }
  const Eigen::MatrixXd reconstructed = raw_factor.transpose() * raw_factor;
  // The constructive certificate is valid only for the Gram that was actually
  // formed from this factor.  A floating tolerance here could turn a genuinely
  // indefinite supplied matrix (for example diag(1, -1e-16)) into a proof for
  // the benign factor diag(1, 0).  Eigen's product is deterministic for these
  // fixed inputs, so require the recomputed entries to match exactly.
  if (!(actual_gram.array() == reconstructed.array()).all()) {
    out.reason = "actual Gram does not match its claimed raw factor";
    return out;
  }

  NumericalWorkCounters::faultGramSvd();
  detail::NumericalPhaseScope svd_profile(detail::NumericalProfilePhase::FactorGramSvd);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(raw_factor, Eigen::ComputeThinV);
  svd_profile.finish();
  const Eigen::VectorXd singular_descending = svd.singularValues();
  if (!singular_descending.allFinite() || !svd.matrixV().allFinite()) {
    out.reason = "raw-factor SVD failed";
    return out;
  }
  out.eigenvalues = Eigen::VectorXd::Zero(dimension);
  out.eigenvectors = Eigen::MatrixXd::Zero(dimension, dimension);
  out.eigenvalue_errors = Eigen::VectorXd::Zero(dimension);
  const Eigen::Index available = singular_descending.size();
  const Eigen::Index structural_nullity = dimension - available;
  for (Eigen::Index index = 0; index < available; ++index) {
    const Eigen::Index destination = dimension - 1 - index;
    const double sigma = singular_descending(index);
    out.eigenvalues(destination) = sigma * sigma;
    out.eigenvectors.col(destination) = svd.matrixV().col(index);
  }
  // Thin V is square whenever rows >= columns.  If rows < columns, obtain the
  // exact structural kernel from a full-V decomposition so all null directions
  // remain bound into the certificate.
  if (structural_nullity > 0) {
    NumericalWorkCounters::faultGramSvd();
    detail::NumericalPhaseScope full_profile(detail::NumericalProfilePhase::FactorGramSvd);
    Eigen::JacobiSVD<Eigen::MatrixXd> full(raw_factor, Eigen::ComputeFullV);
    full_profile.finish();
    if (!full.matrixV().allFinite() || full.matrixV().cols() != dimension) {
      out.reason = "raw-factor full-V SVD failed";
      return out;
    }
    out.eigenvectors.leftCols(structural_nullity) =
        full.matrixV().rightCols(structural_nullity);
  }
  out.eigenvalue_error = 0.0;
  for (Eigen::Index index = 0; index < dimension; ++index) {
    const Eigen::VectorXd vector = out.eigenvectors.col(index);
    const double value = out.eigenvalues(index);
    const double residual =
        (actual_gram * vector - value * vector).norm();
    const double local_roundoff = 64.0 * eps *
        static_cast<double>(std::max<Eigen::Index>(1, dimension)) *
        std::abs(value);
    out.eigenvalue_errors(index) = residual + local_roundoff;
    out.eigenvalue_error = std::max(out.eigenvalue_error,
                                    out.eigenvalue_errors(index));
  }
  out.psd = true;  // constructive factor proof, after actual-Gram agreement
  const double largest = singular_descending.size()
      ? singular_descending(0) : 0.0;
  out.sigma_max = largest;
  const double factor_scale = std::isfinite(raw_factor_scale) &&
      raw_factor_scale >= 0.0 ? raw_factor_scale : largest;
  const double singular_gate = rank_tolerance * factor_scale;
  out.rank_eigenvalue_gate = singular_gate * singular_gate;
  out.rank = 0;
  out.rank_certified = true;
  double smallest_positive = std::numeric_limits<double>::infinity();
  for (Eigen::Index index = 0; index < singular_descending.size(); ++index) {
    const double sigma = singular_descending(index);
    const double error = 64.0 * eps *
        static_cast<double>(std::max(raw_factor.rows(), raw_factor.cols())) *
        std::abs(sigma);
    if (sigma - error > singular_gate) {
      ++out.rank;
      smallest_positive = std::min(smallest_positive, sigma);
    } else if (sigma == 0.0 && error == 0.0) {
      continue;
    } else if (!(sigma + error < singular_gate)) {
      out.rank_certified = false;
    }
  }
  if (!out.rank_certified) {
    out.reason = "rank transition interval is numerically indeterminate";
    return out;
  }
  out.sigma_min = out.rank > 0 ? smallest_positive : 0.0;
  out.condition = out.rank > 0 && out.sigma_min > 0.0
      ? out.sigma_max / out.sigma_min
      : std::numeric_limits<double>::infinity();
  if constexpr (HashProof) {
    hashMatrix(&proof, reconstructed);
    hashMatrix(&proof, out.eigenvalues);
    hashMatrix(&proof, out.eigenvectors);
    hashMatrix(&proof, out.eigenvalue_errors);
    hashScalar(&proof, out.rank_eigenvalue_gate);
    hashScalar(&proof, out.rank);
    hashScalar(&proof, out.rank_certified);
    hashScalar(&proof, out.psd);
    out.proof_identity = proof;
  }
  out.valid = true;
  return out;
}

}  // namespace

SymmetricPsdCertificate certifyFactorGram(
    const Eigen::MatrixXd& raw_factor, const Eigen::MatrixXd& actual_gram,
    double rank_tolerance, std::uint64_t parent_proof_identity,
    double raw_factor_scale) {
  return certifyFactorGramImpl<true>(raw_factor, actual_gram, rank_tolerance,
                                    parent_proof_identity, raw_factor_scale);
}

namespace {
template <bool HashProof>
GramResponseCertificate finishGramResponseCertificate(
    SymmetricPsdCertificate gram, const Eigen::MatrixXd& protected_response) {
  GramResponseCertificate out;
  out.gram = std::move(gram);
  std::uint64_t proof = out.gram.proof_identity;
  if constexpr (HashProof) {
    hashMatrix(&proof, protected_response);
    out.proof_identity = proof;
  }
  if (!out.gram.valid) {
    out.reason = out.gram.reason;
    return out;
  }
  if (protected_response.rows() != 3 ||
      protected_response.cols() != out.gram.symmetric_matrix.cols() ||
      !protected_response.allFinite()) {
    out.reason = "protected response dimensions/values are invalid";
    return out;
  }
  const Eigen::Index dimension = out.gram.symmetric_matrix.cols();
  const int nullity = static_cast<int>(dimension) - out.gram.rank;
  out.response_tolerance = 0.0;
  out.axis_residual.setZero();
  if (nullity == 0) {
    out.nullspace_class = GramNullspaceClass::FullRank;
  } else {
    const Eigen::MatrixXd kernel = out.gram.eigenvectors.leftCols(nullity);
    const Eigen::MatrixXd hidden = protected_response * kernel;
    for (int axis = 0; axis < 3; ++axis) {
      out.axis_residual(axis) = hidden.row(axis).norm();
    }
    out.nullspace_class = out.axis_residual.maxCoeff() == 0.0
        ? GramNullspaceClass::Harmless : GramNullspaceClass::Dangerous;
  }
  out.protected_slopes.setZero();
  const int first_positive = static_cast<int>(dimension) - out.gram.rank;
  for (int axis = 0; axis < 3; ++axis) {
    long double quadratic = 0.0L;
    for (int index = first_positive; index < dimension; ++index) {
      const double eigenvalue = out.gram.eigenvalues(index);
      if (!(eigenvalue > out.gram.rank_eigenvalue_gate)) {
        out.nullspace_class = GramNullspaceClass::Indeterminate;
        out.reason = "positive Gram subspace is numerically indeterminate";
        return out;
      }
      const double projection = out.gram.eigenvectors.col(index).dot(
          protected_response.row(axis).transpose());
      quadratic += static_cast<long double>(projection) * projection /
                   static_cast<long double>(eigenvalue);
    }
    out.protected_slopes(axis) = std::sqrt(std::max(0.0L, quadratic));
  }
  if (out.nullspace_class == GramNullspaceClass::Dangerous) {
    out.reason = "Gram nullspace has an unbounded protected response";
  }
  if constexpr (HashProof) {
    hashScalar(&proof, static_cast<int>(out.nullspace_class));
    hashMatrix(&proof, out.axis_residual);
    hashMatrix(&proof, out.protected_slopes);
    hashScalar(&proof, out.response_tolerance);
    out.proof_identity = proof;
  }
  out.valid = out.nullspace_class != GramNullspaceClass::Indeterminate;
  return out;
}
}  // namespace

GramResponseCertificate certifyFactorGramAndProtectedResponse(
    const Eigen::MatrixXd& raw_factor, const Eigen::MatrixXd& actual_gram,
    const Eigen::MatrixXd& protected_response, double rank_tolerance,
    std::uint64_t parent_proof_identity, double raw_factor_scale) {
  return finishGramResponseCertificate<true>(
      certifyFactorGram(raw_factor, actual_gram, rank_tolerance,
                        parent_proof_identity, raw_factor_scale),
      protected_response);
}

namespace detail {
GramResponseCertificate rebindValidatedFactorResponse(
    const GramResponseCertificate& validated, const Eigen::MatrixXd& raw,
    const Eigen::MatrixXd& actual_gram, const Eigen::MatrixXd& protected_response,
    double rank_tolerance, double raw_factor_scale, std::uint64_t parent) {
  // The internal caller retains the exact validated raw/Gram/G tuple. This
  // reproduces the existing public certifier's identity arithmetic only.
  GramResponseCertificate out = validated;
  std::uint64_t identity = 1469598103934665603ULL;
  hashScalar(&identity, std::uint64_t(0x464143544f524752ULL));
  hashScalar(&identity, parent);
  hashScalar(&identity, rank_tolerance);
  hashScalar(&identity, raw_factor_scale);
  hashMatrix(&identity, raw);
  hashMatrix(&identity, actual_gram);
  hashMatrix(&identity, actual_gram); // reconstructed factor Gram (exact match)
  hashMatrix(&identity, out.gram.eigenvalues);
  hashMatrix(&identity, out.gram.eigenvectors);
  hashMatrix(&identity, out.gram.eigenvalue_errors);
  hashScalar(&identity, out.gram.rank_eigenvalue_gate);
  hashScalar(&identity, out.gram.rank);
  hashScalar(&identity, out.gram.rank_certified);
  hashScalar(&identity, out.gram.psd);
  out.gram.proof_identity = identity;
  hashMatrix(&identity, protected_response);
  hashScalar(&identity, static_cast<int>(out.nullspace_class));
  hashMatrix(&identity, out.axis_residual);
  hashMatrix(&identity, out.protected_slopes);
  hashScalar(&identity, out.response_tolerance);
  out.proof_identity = identity;
  return out;
}
} // namespace detail

namespace detail {
GramResponseCertificate classificationNumericsWithoutProofIdentity(
    const Eigen::MatrixXd& raw_factor, const Eigen::MatrixXd& actual_gram,
    const Eigen::MatrixXd& protected_response, double rank_tolerance,
    double raw_factor_scale) {
  return finishGramResponseCertificate<false>(
      certifyFactorGramImpl<false>(raw_factor, actual_gram, rank_tolerance,
                                  0, raw_factor_scale), protected_response);
}
}  // namespace detail

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
  const double gate = rank_tolerance * diagonal_max;
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

std::uint64_t FrozenSquareRootContext::proofIdentity() const {
  // Recomputed on demand so no ABI-visible or private storage field is needed.
  // content_fingerprint_ binds the exact H/z/protected-map input bytes.
  std::uint64_t proof = 1469598103934665603ULL;
  hashScalar(&proof, content_fingerprint_);
  hashScalar(&proof, policy_fingerprint_);
  hashScalar(&proof, scale_fingerprint_);
  hashScalar(&proof, scale_policy_);
  hashScalar(&proof, permutation_policy_);
  hashMatrix(&proof, scale_);
  for (const int column : permutation_) hashScalar(&proof, column);
  hashMatrix(&proof, scaled_);
  hashMatrix(&proof, r_);
  hashScalar(&proof, rank_);
  hashScalar(&proof, certificate_.full_column_rank);
  hashScalar(&proof, certificate_.identity_ok);
  hashScalar(&proof, certificate_.parity_matches_reference);
  hashScalar(&proof, certificate_.solution_matches_reference);
  hashScalar(&proof, certificate_.forward_error_ok);
  hashScalar(&proof, certificate_.identity_residual_relative);
  hashScalar(&proof, certificate_.parity_relative_difference);
  hashScalar(&proof, certificate_.solution_relative_difference);
  hashScalar(&proof, certificate_.forward_error_bound);
  hashScalar(&proof, certificate_.r_diagonal_min);
  hashScalar(&proof, certificate_.r_diagonal_max);
  hashScalar(&proof, certificate_.condition_estimate);
  hashMatrix(&proof, state_increment_);
  hashMatrix(&proof, parity_);
  hashScalar(&proof, statistic_);
  hashScalar(&proof, usable_);
  hashBytes(&proof, reason_.data(), reason_.size());
  return proof;
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
