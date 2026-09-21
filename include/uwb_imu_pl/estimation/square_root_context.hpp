#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace uwb_imu_pl {

// ---------------------------------------------------------------------------
// B1: the single square-root numerical context of a frozen integrity window.
//
// One column-permuted (optionally scaled) QR factorization of H serves every
// consumer: the nominal state increment, the parity statistic, all information
// solves ((H^T H)^-1 rhs, i.e. Sigma_p / G / block caches), the fault responses
// Y = Q1^T D and Z = Q2^T D, the protected covariance, and the per-hypothesis
// monitorability decomposition.  The full Q, Q2, P or (I - H P H^T) projector
// are never formed; Q^T is applied implicitly through the Householder sequence.
//
// Conventions (frozen in B1, see proof-obligations.md items 3/4/5/7):
//   A   = (H * S) * Pmat                     (m x n, scaled and column permuted)
//   A   = Q * R                              (thin QR, R upper triangular n x n)
//   C~  = C * S * Pmat                       (protected map in the same basis)
//   dx  = S * Pmat * x~                      (x~ solves R x~ = Q1^T z)
//   (H^T H)^-1 = S P R^-1 R^-T P^T S
//   G   = C (H^T H)^-1 H^T D = C~ R^-1 Y,   Y = Q1^T D
//   Z   = Q2^T D,                            Gamma = Z^T Z  (audit quantity)
// ---------------------------------------------------------------------------

enum class ColumnScalePolicy {
  Unit,        // S = I: the frozen columns are already physical and whitened.
  ColumnNorm,  // S_j = 1 / ||H_j|| (guarded): equalizes physical column scale.
};

enum class ColumnPermutationPolicy {
  Natural,      // Pmat = I: keep the state-layout order (default).
  ColumnPivot,  // Pmat from the rank-revealing column pivoting of the QR.
};

std::string toString(ColumnScalePolicy policy);
std::string toString(ColumnPermutationPolicy policy);
std::uint64_t squareRootPolicyFingerprint(ColumnScalePolicy scale,
                                          ColumnPermutationPolicy permutation);

// Pattern-level result of the symbolic analysis; reusable across windows whose
// sparsity pattern, dimensions and policies are identical.  It never contains
// numerical factors: a pattern change forces a new analysis, and equal patterns
// still rebuild R because R depends on values.
struct SquareRootSymbolicPlan {
  std::uint64_t pattern_fingerprint = 0;
  std::vector<int> detector_only_rows;   // rows of H that are identically zero
  std::vector<int> pattern_column_order; // columns ordered by ascending nnz
  bool sameAs(const SquareRootSymbolicPlan& other) const;
};

// Cheap per-window certificate.  `ok()` is the gate that lets consumers use the
// context; a failing certificate sends them to the reference fallback instead
// of silently serving degraded numbers.
struct SquareRootCertificate {
  bool full_column_rank = false;
  bool identity_ok = false;
  bool parity_matches_reference = false;
  bool solution_matches_reference = false;
  bool forward_error_ok = false;
  double identity_residual_relative = std::numeric_limits<double>::infinity();
  double parity_relative_difference = std::numeric_limits<double>::infinity();
  double solution_relative_difference = std::numeric_limits<double>::infinity();
  double forward_error_bound = std::numeric_limits<double>::infinity();
  double r_diagonal_min = 0.0;
  double r_diagonal_max = 0.0;
  double condition_estimate = std::numeric_limits<double>::infinity();
  bool ok() const;
};

class FrozenSquareRootContext {
 public:
  // Builds the context from the frozen snapshot.  `reference_solution` and
  // `reference_statistic` are the SVD products; they are used only for the
  // certificate cross-check, never served to consumers.
  static std::shared_ptr<const FrozenSquareRootContext> build(
      const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
      const Eigen::Matrix<double, 3, Eigen::Dynamic>& protected_state_map,
      ColumnScalePolicy scale_policy,
      ColumnPermutationPolicy permutation_policy,
      double rank_tolerance,
      const Eigen::VectorXd& reference_solution,
      double reference_statistic);

  int rows() const { return rows_; }
  int columns() const { return columns_; }
  int rank() const { return rank_; }
  int dof() const { return rows_ - rank_; }
  double statistic() const { return statistic_; }
  const Eigen::VectorXd& parity() const { return parity_; }
  const Eigen::VectorXd& baseStateIncrement() const { return state_increment_; }
  const SquareRootCertificate& certificate() const { return certificate_; }
  bool usable() const { return usable_; }
  const std::string& reason() const { return reason_; }
  ColumnScalePolicy scalePolicy() const { return scale_policy_; }
  ColumnPermutationPolicy permutationPolicy() const { return permutation_policy_; }
  const SquareRootSymbolicPlan& symbolicPlan() const { return plan_; }
  int detectorOnlyRows() const {
    return static_cast<int>(plan_.detector_only_rows.size());
  }
  std::uint64_t scaleVectorFingerprint() const { return scale_fingerprint_; }
  const std::vector<int>& columnPermutation() const { return permutation_; }
  const Eigen::VectorXd& columnScale() const { return scale_; }
  const Eigen::MatrixXd& rFactor() const { return r_; }
  const Eigen::MatrixXd& scaledPermutedJacobian() const { return scaled_; }

  // C~ = C * S * Pmat (3 x n): the protected map expressed in the factored
  // basis.  C must always travel with the same S and Pmat as H.
  const Eigen::MatrixXd& protectedMap() const { return protected_map_; }

  // x = (H^T H)^-1 rhs for every column of rhs (n x k).
  Eigen::MatrixXd informationSolve(const Eigen::MatrixXd& rhs) const;

  // x = argmin ||H x - b|| (m x k right-hand side), minimum norm.
  Eigen::MatrixXd leastSquaresSolve(const Eigen::MatrixXd& rhs) const;

  // Q^T M without forming Q (m x k).  Rows [0, rank) and [rank, m) of the
  // result are Q1^T M and Q2^T M respectively.
  Eigen::MatrixXd applyQt(const Eigen::MatrixXd& rhs) const;

  // Fault response split: Y = Q1^T D, Z = Q2^T D.
  void faultResponse(const Eigen::MatrixXd& dense, Eigen::MatrixXd* y,
                     Eigen::MatrixXd* z) const;

  // U = C~ R^-1 (3 x n) and Sigma_p = U U^T, without forming P.
  Eigen::MatrixXd protectedResponse() const;
  Eigen::Matrix3d protectedCovariance() const;

  std::uint64_t contentFingerprint() const { return content_fingerprint_; }
  std::uint64_t policyFingerprint() const { return policy_fingerprint_; }

  // Symbolic cache instrumentation (process wide), reset with the counters.
  struct SymbolicCacheStats {
    std::uint64_t entries = 0;
    std::uint64_t hits = 0;
    std::uint64_t misses = 0;
  };
  static SymbolicCacheStats symbolicCacheStats();
  static void clearSymbolicCache();

 private:
  FrozenSquareRootContext() = default;

  int rows_ = 0;
  int columns_ = 0;
  int rank_ = 0;
  double statistic_ = std::numeric_limits<double>::infinity();
  Eigen::VectorXd parity_;
  Eigen::VectorXd state_increment_;
  Eigen::MatrixXd r_;
  Eigen::MatrixXd scaled_;             // A = (H*S)*Pmat
  Eigen::VectorXd scale_;
  std::vector<int> permutation_;       // A.col(j) = (H*S).col(permutation_[j])
  Eigen::MatrixXd protected_map_;      // C~
  SquareRootSymbolicPlan plan_;
  SquareRootCertificate certificate_;
  ColumnScalePolicy scale_policy_ = ColumnScalePolicy::Unit;
  ColumnPermutationPolicy permutation_policy_ = ColumnPermutationPolicy::Natural;
  std::uint64_t scale_fingerprint_ = 0;
  std::uint64_t content_fingerprint_ = 0;
  std::uint64_t policy_fingerprint_ = 0;
  bool usable_ = false;
  std::string reason_;

  // Implicit Householder sequence; the object owns the reflector vectors.
  struct ImplicitQ;
  std::shared_ptr<const ImplicitQ> q_;
};

}  // namespace uwb_imu_pl
