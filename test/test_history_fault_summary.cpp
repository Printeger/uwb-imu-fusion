// C1-a module-level identity tests for the history fault summary
// (design freeze history-summary-design.md §1-§2; evidence
// history-summary-module.md).
//
// Oracle policy: every reference quantity is recomputed with dense linear
// algebra implemented in this file (thin JacobiSVD projectors, explicit
// pseudo-inverses, dense joint solves).  The module under test
// (buildHistoryFaultSummary) is never used to build an oracle; only its
// outputs are compared against the independent references.  No production
// solver (frozen window numerics, rank-update kernel, monitor) is touched.
//
// Tolerance basis (ADR 0002): relative 1e-9 for well-conditioned systems,
// 1e-7 for moderate conditioning.  The harsh tier (kappa(H_o)=1e10) measures
// just beyond the 1e-7 moderate budget, so it is compared at the fixed 1e-6
// tier and tagged; it does not claim the moderate tier and is not placed at
// the rank gate (pivot ratio ~9.4e-10 vs the 1e-12 gate).

#include <gtest/gtest.h>

#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "uwb_imu_pl/integrity/history_fault_summary.hpp"

namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

using uwb_imu_pl::buildCertifiedHistoryFaultSummary;
using uwb_imu_pl::buildHistoryFaultSummary;
using uwb_imu_pl::CertifiedHistoryFaultSummary;
using uwb_imu_pl::HistoryCarrierRankCertificate;
using uwb_imu_pl::HistoryFaultSummary;
using uwb_imu_pl::HistoryFaultSummaryInput;
using uwb_imu_pl::HistoryFaultSummaryOptions;
using uwb_imu_pl::splitBlockSystem;

constexpr double kTolWell = 1e-9;      // ADR 0002, well-conditioned
constexpr double kTolModerate = 1e-7;  // ADR 0002, moderate
constexpr double kTolNearGate = 1e-6;  // ADR 0002, near-gate (tagged)
constexpr double kOracleRankTolerance = 1e-12;

// ---------------------------------------------------------------------------
// Deterministic random helpers
// ---------------------------------------------------------------------------

class Rng {
 public:
  explicit Rng(std::uint32_t seed) : gen_(seed) {}

  MatrixXd normal(int rows, int cols) {
    std::normal_distribution<double> dist(0.0, 1.0);
    MatrixXd out(rows, cols);
    for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; ++j) {
        out(i, j) = dist(gen_);
      }
    }
    return out;
  }

  VectorXd normalVector(int size) {
    std::normal_distribution<double> dist(0.0, 1.0);
    VectorXd out(size);
    for (int i = 0; i < size; ++i) {
      out(i) = dist(gen_);
    }
    return out;
  }

  // Orthonormal columns: thin U of a Gaussian matrix.
  MatrixXd orthonormal(int rows, int cols) {
    Eigen::JacobiSVD<MatrixXd> svd(normal(rows, cols), Eigen::ComputeThinU);
    return svd.matrixU();
  }

  std::vector<int> permutation(int n) {
    std::vector<int> perm(n);
    for (int i = 0; i < n; ++i) {
      perm[i] = i;
    }
    std::shuffle(perm.begin(), perm.end(), gen_);
    return perm;
  }

 private:
  std::mt19937 gen_;
};

// H_o with prescribed condition number: singular values 1 .. 1/kappa.
MatrixXd conditionedOldState(Rng& rng, int rows, int cols, double kappa) {
  const MatrixXd u = rng.orthonormal(rows, cols);
  const MatrixXd v = rng.orthonormal(cols, cols);
  VectorXd singular(cols);
  for (int i = 0; i < cols; ++i) {
    const double t = (cols == 1) ? 0.0 : static_cast<double>(i) / (cols - 1);
    singular(i) = std::pow(kappa, -t);
  }
  return u * singular.asDiagonal() * v.transpose();
}

HistoryFaultSummaryInput makeInput(Rng& rng, int m, int n_old, int n_boundary,
                                   int n_fault, double kappa_old = 1.0) {
  HistoryFaultSummaryInput input;
  input.h_old_state = conditionedOldState(rng, m, n_old, kappa_old);
  input.h_boundary = rng.normal(m, n_boundary);
  input.fault_map = rng.normal(m, n_fault);
  input.rhs = rng.normalVector(m);
  return input;
}

MatrixXd permutationMatrix(const std::vector<int>& perm) {
  const int n = static_cast<int>(perm.size());
  MatrixXd out = MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    out(i, perm[i]) = 1.0;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Independent dense oracles
// ---------------------------------------------------------------------------

struct ThinSvd {
  MatrixXd u;  // thin left singular vectors above the rank gate
  VectorXd singular;
  int rank = 0;
};

ThinSvd thinSvd(const MatrixXd& matrix) {
  ThinSvd out;
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    return out;
  }
  Eigen::JacobiSVD<MatrixXd> svd(matrix, Eigen::ComputeThinU);
  out.singular = svd.singularValues();
  const double largest = out.singular.size() ? out.singular(0) : 0.0;
  const double gate = (largest == 0.0) ? 0.0 : kOracleRankTolerance * largest;
  int rank = 0;
  for (int i = 0; i < out.singular.size(); ++i) {
    if (out.singular(i) > gate) {
      ++rank;
    }
  }
  out.rank = rank;
  out.u = svd.matrixU().leftCols(rank);
  return out;
}

MatrixXd orthogonalProjector(const MatrixXd& matrix) {
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    return MatrixXd::Zero(matrix.rows(), matrix.rows());
  }
  const ThinSvd factors = thinSvd(matrix);
  return factors.u * factors.u.transpose();
}

MatrixXd pseudoInverse(const MatrixXd& matrix) {
  if (matrix.rows() == 0 || matrix.cols() == 0) {
    return MatrixXd::Zero(matrix.cols(), matrix.rows());
  }
  Eigen::JacobiSVD<MatrixXd> svd(matrix,
                                 Eigen::ComputeThinU | Eigen::ComputeThinV);
  const VectorXd singular = svd.singularValues();
  const double largest = singular.size() ? singular(0) : 0.0;
  const double gate = (largest == 0.0) ? 0.0 : kOracleRankTolerance * largest;
  MatrixXd inverse = MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());
  for (int i = 0; i < singular.size(); ++i) {
    if (singular(i) > gate) {
      inverse(i, i) = 1.0 / singular(i);
    }
  }
  return svd.matrixV() * inverse * svd.matrixU().transpose();
}

MatrixXd joinBlocks(const MatrixXd& left, const MatrixXd& right) {
  MatrixXd joined(left.rows(), left.cols() + right.cols());
  if (left.cols() > 0) {
    joined.leftCols(left.cols()) = left;
  }
  if (right.cols() > 0) {
    joined.rightCols(right.cols()) = right;
  }
  return joined;
}

// min_{x_o} || H_o x_o + w ||^2 with w = H_b x_b + A f - z.
double minOldStateCost(const MatrixXd& h_old_state, const VectorXd& w) {
  const MatrixXd projector = orthogonalProjector(h_old_state);
  const VectorXd residual = w - projector * w;
  return residual.squaredNorm();
}

// Information of x_b after marginalizing x_o: H_b^T (I - P_{H_o}) H_b.
// This is the dense Schur complement of the [H_o H_b] normal matrix in the
// x_b block (no elimination of x_b itself).
MatrixXd marginalBoundaryInformation(const MatrixXd& h_old_state,
                                     const MatrixXd& h_boundary) {
  const MatrixXd projector = orthogonalProjector(h_old_state);
  return h_boundary.transpose() * (h_boundary - projector * h_boundary);
}

// A^T (I - P_{[H_o H_b]}) A (fault Gram after eliminating all states).
MatrixXd faultGramOracle(const MatrixXd& h_old_state,
                         const MatrixXd& h_boundary,
                         const MatrixXd& fault_map) {
  const MatrixXd joined = joinBlocks(h_old_state, h_boundary);
  const MatrixXd projector = orthogonalProjector(joined);
  return fault_map.transpose() * (fault_map - projector * fault_map);
}

// min over (x_o, x_b) of the residual energy at f = 0.
double minJointResidualCost(const MatrixXd& h_old_state,
                            const MatrixXd& h_boundary, const VectorXd& rhs) {
  const MatrixXd joined = joinBlocks(h_old_state, h_boundary);
  const MatrixXd projector = orthogonalProjector(joined);
  const VectorXd residual = rhs - projector * rhs;
  return residual.squaredNorm();
}

// Independent O01 oracle for the corrected carrier contract.  Build an
// orthonormal basis of the residual subspace of [H_o|H_b], project [A|z]
// into it, and apply the same documented numerical-contract inequality to
// independently computed singular values.  This is deliberately not a
// hard-coded expected rank and does not inspect production F_b/d_perp.
int effectiveCarrierRankOracle(const HistoryFaultSummaryInput& input,
                               double rank_tolerance = 1e-12) {
  const MatrixXd joined = joinBlocks(input.h_old_state, input.h_boundary);
  Eigen::JacobiSVD<MatrixXd> joined_svd(joined,
                                        Eigen::ComputeFullU |
                                            Eigen::ComputeThinV);
  const double joined_scale = joined_svd.singularValues().size() == 0
                                  ? 0.0
                                  : joined_svd.singularValues()(0);
  int joined_rank = 0;
  for (Eigen::Index i = 0; i < joined_svd.singularValues().size(); ++i) {
    if (joined_svd.singularValues()(i) > rank_tolerance * joined_scale)
      ++joined_rank;
  }
  MatrixXd raw(input.rows(), input.faultColumns() + 1);
  if (input.faultColumns() > 0)
    raw.leftCols(input.faultColumns()) = input.fault_map;
  raw.col(input.faultColumns()) = input.rhs;
  const MatrixXd carrier =
      joined_svd.matrixU().rightCols(input.rows() - joined_rank).transpose() *
      raw;
  Eigen::JacobiSVD<MatrixXd> carrier_svd(carrier);
  const VectorXd singular = carrier_svd.singularValues();
  const double scale = singular.size() == 0 ? 0.0 : singular(0);
  const double dimension = static_cast<double>(
      std::max<Eigen::Index>(carrier.rows(), carrier.cols()));
  const double eps = std::numeric_limits<double>::epsilon();
  const double gamma = (dimension * eps) / (1.0 - dimension * eps);
  const double threshold =
      rank_tolerance * scale + gamma * carrier.norm();
  int rank = 0;
  for (Eigen::Index i = 0; i < singular.size(); ++i)
    if (singular(i) > threshold) ++rank;
  return rank;
}

// x_b part of the joint minimizer for rhs.
VectorXd jointBoundarySolve(const MatrixXd& h_old_state,
                            const MatrixXd& h_boundary, const VectorXd& rhs) {
  const MatrixXd joined = joinBlocks(h_old_state, h_boundary);
  const VectorXd solution = pseudoInverse(joined) * rhs;
  return solution.tail(h_boundary.cols());
}

// ---------------------------------------------------------------------------
// Error metrics
// ---------------------------------------------------------------------------

double relErr(double got, double want) {
  const double scale = std::max({1.0, std::abs(got), std::abs(want)});
  return std::abs(got - want) / scale;
}

double relErrMatrix(const MatrixXd& got, const MatrixXd& want) {
  if (got.size() == 0 && want.size() == 0) {
    return 0.0;
  }
  const double scale = std::max({1.0, got.norm(), want.norm()});
  return (got - want).norm() / scale;
}

struct IdentityErrors {
  double cost = 0.0;
  double schur = 0.0;
  double omega = 0.0;
  double kappa = 0.0;
  double shift = 0.0;
  double delta = 0.0;
  double nu_mismatch = 0.0;
  bool shift_checked = false;
};

IdentityErrors measureIdentities(const HistoryFaultSummary& summary,
                                 const HistoryFaultSummaryInput& input,
                                 Rng& rng, int probes) {
  IdentityErrors errors;
  for (int t = 0; t < probes; ++t) {
    const VectorXd x_b = rng.normalVector(input.boundaryColumns());
    const VectorXd f = rng.normalVector(input.faultColumns());
    const VectorXd w = input.h_boundary * x_b + input.fault_map * f - input.rhs;
    const double lhs = minOldStateCost(input.h_old_state, w);
    const double rhs =
        (summary.R_b * x_b + summary.T_b * f - summary.d_b).squaredNorm() +
        (summary.F_b * f - summary.d_perp).squaredNorm();
    errors.cost = std::max(errors.cost,
                           std::abs(lhs - rhs) / std::max(1.0, std::abs(lhs)));
    if (summary.boundaryShiftUsable()) {
      const VectorXd xb_f = jointBoundarySolve(
          input.h_old_state, input.h_boundary, input.rhs - input.fault_map * f);
      const VectorXd xb_0 =
          jointBoundarySolve(input.h_old_state, input.h_boundary, input.rhs);
      const VectorXd dense_delta = xb_f - xb_0;
      const VectorXd shift = summary.boundaryMeanShiftForFault(f);
      const VectorXd delta = summary.conditionalBoundaryMeanDelta(f);
      const double scale = std::max(1.0, std::max(shift.norm(), delta.norm()));
      errors.shift =
          std::max(errors.shift, (shift + dense_delta).norm() / scale);
      errors.delta =
          std::max(errors.delta, (delta - dense_delta).norm() / scale);
      errors.shift_checked = true;
    }
  }
  errors.schur = relErrMatrix(
      summary.R_b.transpose() * summary.R_b,
      marginalBoundaryInformation(input.h_old_state, input.h_boundary));
  // Omega/kappa/nu are pure invariant checks only when the boundary block is
  // full rank (otherwise part of the f-response sits in the boundary block
  // and the identity is still carried by the cost check above).
  const bool well_posed = summary.rank_boundary == summary.n_boundary &&
                          summary.rank_h_old_state == summary.n_old_state;
  if (well_posed) {
    errors.omega = relErrMatrix(
        summary.omegaBoundary(),
        faultGramOracle(input.h_old_state, input.h_boundary, input.fault_map));
    errors.kappa = relErr(
        summary.kappaBoundary(),
        minJointResidualCost(input.h_old_state, input.h_boundary, input.rhs));
    const double nu_oracle = effectiveCarrierRankOracle(input);
    errors.nu_mismatch = std::abs(nu_oracle - summary.nuPerp());
  }
  return errors;
}

bool allNan(const VectorXd& vector) {
  if (vector.size() == 0) {
    return false;
  }
  for (int i = 0; i < vector.size(); ++i) {
    if (!std::isnan(vector(i))) {
      return false;
    }
  }
  return true;
}

std::string fmt(double value) {
  std::ostringstream out;
  out << std::scientific << std::setprecision(3) << value;
  return out.str();
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(HistoryFaultSummary, P001CorrelatedRawFactorOracle) {
  // Independent O01 fixture: unique unwhitened rows, non-zero RHS and a
  // genuinely correlated covariance.  The production summary only receives
  // the whitened system; every reference below is rebuilt here from the raw
  // factors and dense SVD/LLT algebra.
  Rng rng(2026092301u);
  constexpr int m = 12;
  const MatrixXd h_old_raw = rng.normal(m, 3);
  const MatrixXd h_boundary_raw = rng.normal(m, 3);
  const MatrixXd fault_raw = rng.normal(m, 2);
  const VectorXd rhs_raw = rng.normalVector(m) +
      VectorXd::LinSpaced(m, -0.7, 1.1);
  MatrixXd correlation = rng.normal(m, m);
  MatrixXd covariance = correlation * correlation.transpose();
  covariance.diagonal().array() += 2.0;
  ASSERT_GT((covariance -
             MatrixXd(covariance.diagonal().asDiagonal())).norm(),
            1.0);
  Eigen::LLT<MatrixXd> llt(covariance);
  ASSERT_EQ(llt.info(), Eigen::Success);
  HistoryFaultSummaryInput input;
  input.h_old_state = llt.matrixL().solve(h_old_raw);
  input.h_boundary = llt.matrixL().solve(h_boundary_raw);
  input.fault_map = llt.matrixL().solve(fault_raw);
  input.rhs = llt.matrixL().solve(rhs_raw);
  const CertifiedHistoryFaultSummary certified =
      buildCertifiedHistoryFaultSummary(input);
  const HistoryFaultSummary& summary = certified.summary;
  ASSERT_TRUE(summary.valid) << summary.invalid_reason;

  const MatrixXd old_projector = orthogonalProjector(input.h_old_state);
  const MatrixXd old_residual =
      MatrixXd::Identity(m, m) - old_projector;
  EXPECT_LE(relErrMatrix(summary.R_b.transpose() * summary.R_b,
                         input.h_boundary.transpose() * old_residual *
                             input.h_boundary),
            kTolWell);
  EXPECT_LE(relErrMatrix(summary.R_b.transpose() * summary.T_b,
                         input.h_boundary.transpose() * old_residual *
                             input.fault_map),
            kTolWell);
  EXPECT_LE(relErrMatrix(summary.T_b.transpose() * summary.T_b +
                             summary.F_b.transpose() * summary.F_b,
                         input.fault_map.transpose() * old_residual *
                             input.fault_map),
            kTolWell);
  EXPECT_LE(relErrMatrix(summary.F_b.transpose() * summary.F_b,
                         faultGramOracle(input.h_old_state,
                                         input.h_boundary, input.fault_map)),
            kTolWell);

  const MatrixXd information = summary.R_b.transpose() * summary.R_b;
  EXPECT_LE(relErrMatrix(pseudoInverse(information),
                         pseudoInverse(marginalBoundaryInformation(
                             input.h_old_state, input.h_boundary))),
            kTolWell);
  EXPECT_LE(relErr(summary.kappaBoundary(),
                   minJointResidualCost(input.h_old_state,
                                        input.h_boundary, input.rhs)),
            kTolWell);
  EXPECT_EQ(summary.nuPerp(), effectiveCarrierRankOracle(input));
  EXPECT_TRUE(certified.carrier_rank.valid);
  EXPECT_EQ(certified.carrier_rank.raw_residual_dof,
            m - thinSvd(joinBlocks(input.h_old_state,
                                  input.h_boundary)).rank);

  for (int probe = 0; probe < 20; ++probe) {
    const VectorXd x_b = rng.normalVector(input.boundaryColumns());
    const VectorXd fault = rng.normalVector(input.faultColumns());
    const VectorXd raw_w =
        h_boundary_raw * x_b + fault_raw * fault - rhs_raw;
    const VectorXd whitened_w = llt.matrixL().solve(raw_w);
    const double raw_objective =
        minOldStateCost(input.h_old_state, whitened_w);
    const double condensed_objective =
        (summary.R_b * x_b + summary.T_b * fault - summary.d_b)
            .squaredNorm() +
        (summary.F_b * fault - summary.d_perp).squaredNorm();
    EXPECT_LE(relErr(raw_objective, condensed_objective), kTolWell);
  }

  const VectorXd expected_state = jointBoundarySolve(
      input.h_old_state, input.h_boundary, input.rhs);
  const VectorXd represented_state =
      pseudoInverse(summary.R_b) * summary.d_b;
  EXPECT_LE(relErrMatrix(represented_state, expected_state), kTolWell);
}

TEST(HistoryFaultSummary, HISM1CostIdentity) {
  // Identity 1: min_{x_o} cost == ||R_b x_b + T_b f - d_b||^2 +
  // ||F_b f - d_perp||^2 for many random (x_b, f), relative 1e-9
  // (ADR 0002 well-conditioned tier).
  Rng rng(20260921u);
  const std::vector<std::array<int, 4>> sizes = {
      {12, 4, 3, 2}, {30, 7, 5, 3}, {60, 10, 6, 4}};
  for (const auto& size : sizes) {
    const HistoryFaultSummaryInput input =
        makeInput(rng, size[0], size[1], size[2], size[3]);
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    ASSERT_TRUE(summary.valid) << summary.invalid_reason;
    const IdentityErrors errors = measureIdentities(summary, input, rng, 16);
    EXPECT_LE(errors.cost, kTolWell)
        << "size " << size[0] << "x" << size[1] << "/" << size[2];
  }
}

TEST(HistoryFaultSummary, HISM1SchurEquivalence) {
  // Identity 2: f = 0 boundary marginal information R_b^T R_b equals the
  // dense Schur complement H_b^T (I - P) H_b; the corresponding marginal
  // covariance (R_b^T R_b)^-1 equals the dense covariance of x_b.
  Rng rng(20260922u);
  const std::vector<std::array<int, 4>> sizes = {
      {12, 4, 3, 2}, {30, 7, 5, 3}, {60, 10, 6, 4}};
  for (const auto& size : sizes) {
    const HistoryFaultSummaryInput input =
        makeInput(rng, size[0], size[1], size[2], size[3]);
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    ASSERT_TRUE(summary.valid) << summary.invalid_reason;
    const MatrixXd information = summary.R_b.transpose() * summary.R_b;
    const MatrixXd schur =
        marginalBoundaryInformation(input.h_old_state, input.h_boundary);
    EXPECT_LE(relErrMatrix(information, schur), kTolWell);
    EXPECT_LE(relErrMatrix(pseudoInverse(information), pseudoInverse(schur)),
              kTolWell);  // marginal covariance (R_b^T R_b)^-1
  }
}

TEST(HistoryFaultSummary, HISM1BoundaryResponseSignDuality) {
  // Identity 3: R_b^-1 T_b f equals the dense conditional boundary mean
  // response up to the frozen sign convention: shift == -(dense delta) and
  // delta == +(dense delta), and the two functions are negatives.
  Rng rng(20260923u);
  const HistoryFaultSummaryInput input = makeInput(rng, 30, 7, 5, 3);
  const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
  ASSERT_TRUE(summary.valid) << summary.invalid_reason;
  ASSERT_TRUE(summary.boundaryShiftUsable());
  const VectorXd xb_0 =
      jointBoundarySolve(input.h_old_state, input.h_boundary, input.rhs);
  for (int t = 0; t < 16; ++t) {
    const VectorXd f = rng.normalVector(input.faultColumns());
    const VectorXd xb_f = jointBoundarySolve(
        input.h_old_state, input.h_boundary, input.rhs - input.fault_map * f);
    const VectorXd dense_delta = xb_f - xb_0;
    const VectorXd shift = summary.boundaryMeanShiftForFault(f);
    const VectorXd delta = summary.conditionalBoundaryMeanDelta(f);
    const double scale =
        std::max({1.0, dense_delta.norm(), shift.norm(), delta.norm()});
    // Both sides of the sign duality are checked on every probe.
    EXPECT_LE((shift - (-dense_delta)).norm() / scale, kTolWell);
    EXPECT_LE((delta - dense_delta).norm() / scale, kTolWell);
    EXPECT_LE((shift + delta).norm() / scale, kTolWell);  // sign duality
  }
}

TEST(HistoryFaultSummary, HISM1FaultGramKappaNu) {
  // Identity 4: Omega_b = F_b^T F_b equals the dense A^T (I - P) A; kappa_b
  // equals the minimal joint residual energy; nu_perp equals the independent
  // effective-rank oracle for [F_b|d_perp], not its raw storage row count.
  Rng rng(20260924u);
  const std::vector<std::array<int, 4>> sizes = {
      {12, 4, 3, 2}, {30, 7, 5, 3}, {60, 10, 6, 4}};
  for (const auto& size : sizes) {
    const HistoryFaultSummaryInput input =
        makeInput(rng, size[0], size[1], size[2], size[3]);
    const CertifiedHistoryFaultSummary certified =
        buildCertifiedHistoryFaultSummary(input);
    const HistoryFaultSummary& summary = certified.summary;
    ASSERT_TRUE(summary.valid) << summary.invalid_reason;
    EXPECT_LE(relErrMatrix(summary.omegaBoundary(),
                           faultGramOracle(input.h_old_state, input.h_boundary,
                                           input.fault_map)),
              kTolWell);
    EXPECT_LE(relErr(summary.kappaBoundary(),
                     minJointResidualCost(input.h_old_state, input.h_boundary,
                                          input.rhs)),
              kTolWell);
    EXPECT_EQ(summary.nuPerp(), effectiveCarrierRankOracle(input));
    EXPECT_EQ(certified.carrier_rank.raw_residual_dof,
              size[0] - thinSvd(joinBlocks(input.h_old_state,
                                           input.h_boundary)).rank);
    EXPECT_EQ(summary.nuPerp(), summary.F_b.rows());
    EXPECT_LE(relErrMatrix(summary.xiBoundary(),
                           summary.F_b.transpose() * summary.d_perp),
              kTolWell);  // xi is stored verbatim from F_b / d_perp
  }
}

TEST(HistoryFaultSummary, HISM1EliminationOrderInvariance) {
  // Identity 5: invariants (R_b^T R_b, F_b^T F_b, kappa_b, nu_perp) and the
  // cost identity are invariant under row permutations and column
  // permutations of x_o, x_b and f (multiple elimination orders).
  Rng rng(20260925u);
  const HistoryFaultSummaryInput base = makeInput(rng, 40, 6, 4, 3);
  const HistoryFaultSummary base_summary = buildHistoryFaultSummary(base);
  ASSERT_TRUE(base_summary.valid) << base_summary.invalid_reason;

  const std::vector<int> row_perm = rng.permutation(base.rows());
  const std::vector<int> old_perm = rng.permutation(base.oldStateColumns());
  const std::vector<int> bound_perm = rng.permutation(base.boundaryColumns());
  const std::vector<int> fault_perm = rng.permutation(base.faultColumns());

  const MatrixXd p_row = permutationMatrix(row_perm);
  const MatrixXd p_old = permutationMatrix(old_perm);
  const MatrixXd p_bound = permutationMatrix(bound_perm);
  const MatrixXd p_fault = permutationMatrix(fault_perm);

  // (a) row permutation.
  HistoryFaultSummaryInput row_input;
  row_input.h_old_state = p_row * base.h_old_state;
  row_input.h_boundary = p_row * base.h_boundary;
  row_input.fault_map = p_row * base.fault_map;
  row_input.rhs = p_row * base.rhs;
  const HistoryFaultSummary row_summary = buildHistoryFaultSummary(row_input);
  ASSERT_TRUE(row_summary.valid) << row_summary.invalid_reason;
  EXPECT_LE(relErrMatrix(row_summary.R_b.transpose() * row_summary.R_b,
                         base_summary.R_b.transpose() * base_summary.R_b),
            kTolWell);
  EXPECT_LE(
      relErrMatrix(row_summary.omegaBoundary(), base_summary.omegaBoundary()),
      kTolWell);
  EXPECT_LE(relErr(row_summary.kappaBoundary(), base_summary.kappaBoundary()),
            kTolWell);
  EXPECT_EQ(row_summary.nuPerp(), base_summary.nuPerp());
  const IdentityErrors row_errors =
      measureIdentities(row_summary, row_input, rng, 8);
  EXPECT_LE(row_errors.cost, kTolWell);

  // (b) x_o column permutation.
  HistoryFaultSummaryInput old_input = base;
  old_input.h_old_state = base.h_old_state * p_old;
  const HistoryFaultSummary old_summary = buildHistoryFaultSummary(old_input);
  ASSERT_TRUE(old_summary.valid) << old_summary.invalid_reason;
  EXPECT_LE(relErrMatrix(old_summary.R_b, base_summary.R_b), kTolWell);
  EXPECT_LE(relErrMatrix(old_summary.R_b.transpose() * old_summary.R_b,
                         base_summary.R_b.transpose() * base_summary.R_b),
            kTolWell);
  EXPECT_LE(
      relErrMatrix(old_summary.omegaBoundary(), base_summary.omegaBoundary()),
      kTolWell);
  EXPECT_LE(relErr(old_summary.kappaBoundary(), base_summary.kappaBoundary()),
            kTolWell);

  // (c) x_b column permutation: QR is not equivariant under column
  // permutation, so only the invariants are compared (R_b^T R_b up to the
  // permutation, F_b^T F_b, kappa_b, nu_perp) plus the full cost identity on
  // the permuted system.
  HistoryFaultSummaryInput bound_input = base;
  bound_input.h_boundary = base.h_boundary * p_bound;
  const HistoryFaultSummary bound_summary =
      buildHistoryFaultSummary(bound_input);
  ASSERT_TRUE(bound_summary.valid) << bound_summary.invalid_reason;
  EXPECT_LE(relErrMatrix(bound_summary.R_b.transpose() * bound_summary.R_b,
                         p_bound.transpose() * base_summary.R_b.transpose() *
                             base_summary.R_b * p_bound),
            kTolWell);
  EXPECT_LE(
      relErrMatrix(bound_summary.omegaBoundary(), base_summary.omegaBoundary()),
      kTolWell);
  EXPECT_LE(relErr(bound_summary.kappaBoundary(), base_summary.kappaBoundary()),
            kTolWell);
  EXPECT_EQ(bound_summary.nuPerp(), base_summary.nuPerp());
  const IdentityErrors bound_errors =
      measureIdentities(bound_summary, bound_input, rng, 8);
  EXPECT_LE(bound_errors.cost, kTolWell);
  EXPECT_LE(bound_errors.schur, kTolWell);
  EXPECT_LE(bound_errors.omega, kTolWell);

  // (d) f column permutation: T_b' = T_b P_f, F_b' = F_b P_f.
  HistoryFaultSummaryInput fault_input = base;
  fault_input.fault_map = base.fault_map * p_fault;
  const HistoryFaultSummary fault_summary =
      buildHistoryFaultSummary(fault_input);
  ASSERT_TRUE(fault_summary.valid) << fault_summary.invalid_reason;
  EXPECT_LE(relErrMatrix(fault_summary.T_b, base_summary.T_b * p_fault),
            kTolWell);
  EXPECT_LE(relErrMatrix(fault_summary.F_b, base_summary.F_b * p_fault),
            kTolWell);
  EXPECT_LE(relErrMatrix(
                fault_summary.omegaBoundary(),
                p_fault.transpose() * base_summary.omegaBoundary() * p_fault),
            kTolWell);
  EXPECT_LE(relErr(fault_summary.kappaBoundary(), base_summary.kappaBoundary()),
            kTolWell);
}

TEST(HistoryFaultSummary, HISM1DegenerateRejection) {
  // Identity 6: non-finite input, rank-deficient H_o (structural) and empty
  // input are rejected with an explicit reason and no numeric output.
  Rng rng(20260926u);

  // (a) non-finite rhs.
  {
    HistoryFaultSummaryInput input = makeInput(rng, 10, 3, 2, 2);
    input.rhs(0) = std::numeric_limits<double>::quiet_NaN();
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    EXPECT_FALSE(summary.valid);
    EXPECT_EQ(summary.invalid_reason.rfind("non_finite_input", 0), 0u)
        << summary.invalid_reason;
    EXPECT_EQ(summary.R_b.size(), 0);
    EXPECT_EQ(summary.F_b.size(), 0);
    EXPECT_FALSE(summary.boundaryShiftUsable());
  }
  // (b) non-finite H_o.
  {
    HistoryFaultSummaryInput input = makeInput(rng, 10, 3, 2, 2);
    input.h_old_state(1, 1) = std::numeric_limits<double>::infinity();
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    EXPECT_FALSE(summary.valid);
    EXPECT_EQ(summary.invalid_reason.rfind("non_finite_input", 0), 0u);
  }
  // (c) structurally rank-deficient H_o (zero column).
  {
    HistoryFaultSummaryInput input = makeInput(rng, 10, 3, 2, 2);
    input.h_old_state.col(1).setZero();
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    EXPECT_FALSE(summary.valid);
    EXPECT_EQ(summary.invalid_reason.rfind("h_o_rank_deficient", 0), 0u)
        << summary.invalid_reason;
    EXPECT_EQ(summary.rank_h_old_state, 2);  // adjudicated rank is reported
    EXPECT_EQ(summary.R_b.size(), 0);
    EXPECT_EQ(summary.T_b.size(), 0);
    EXPECT_EQ(summary.F_b.size(), 0);
    EXPECT_EQ(summary.d_perp.size(), 0);
  }
  // (d) duplicated H_o column (exact linear dependence).
  {
    HistoryFaultSummaryInput input = makeInput(rng, 10, 3, 2, 2);
    input.h_old_state.col(2) = input.h_old_state.col(0);
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    EXPECT_FALSE(summary.valid);
    EXPECT_EQ(summary.invalid_reason.rfind("h_o_rank_deficient", 0), 0u);
  }
  // (e) more old-state columns than rows.
  {
    HistoryFaultSummaryInput input = makeInput(rng, 3, 5, 2, 2);
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    EXPECT_FALSE(summary.valid);
    EXPECT_EQ(summary.invalid_reason.rfind("h_o_rank_deficient", 0), 0u);
  }
  // (f) empty system.
  {
    const HistoryFaultSummary summary =
        buildHistoryFaultSummary(HistoryFaultSummaryInput{});
    EXPECT_FALSE(summary.valid);
    EXPECT_EQ(summary.invalid_reason.rfind("empty_input", 0), 0u);
  }
  // (g) inconsistent shapes (H_b has one row too many; the extra row is
  // zeroed so the shape check, not the finiteness check, reports it).
  {
    HistoryFaultSummaryInput input = makeInput(rng, 10, 3, 2, 2);
    input.h_boundary.conservativeResize(11, 2);
    input.h_boundary.row(10).setZero();
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    EXPECT_FALSE(summary.valid);
    EXPECT_EQ(summary.invalid_reason.rfind("shape_mismatch", 0), 0u);
    EXPECT_EQ(summary.F_b.size(), 0);
  }
  // (h) sign-convention functions on a rejected summary are all-NaN.
  {
    HistoryFaultSummaryInput input = makeInput(rng, 10, 3, 2, 2);
    input.rhs(0) = std::numeric_limits<double>::infinity();
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    ASSERT_FALSE(summary.valid);
    const VectorXd shift = summary.boundaryMeanShiftForFault(VectorXd::Zero(2));
    const VectorXd delta =
        summary.conditionalBoundaryMeanDelta(VectorXd::Zero(2));
    EXPECT_TRUE(allNan(shift));
    EXPECT_TRUE(allNan(delta));
    EXPECT_EQ(shift.size(), 2);
  }
}

TEST(HistoryFaultSummary, HISM1DetectorOnlyAndNominal) {
  // Identity 6b / explicit small cases:
  // (a) detection-only rows (no x_b support, A/z nonzero) enter F_b/d_perp;
  // (b) q = 0 yields correctly sized zero-column T_b/F_b (nominal boundary);
  // (c) a rank-deficient boundary block is NOT rejected: the identity still
  // holds, rank_boundary is reported and the shift functions return NaN.
  Rng rng(20260927u);

  // (a) two boundary variables, two detection-only rows (one fault row, one
  // pure-rhs row).
  {
    HistoryFaultSummaryInput input;
    input.h_old_state = MatrixXd::Zero(4, 0);
    input.h_boundary = MatrixXd(4, 2);
    input.h_boundary << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    input.fault_map = MatrixXd(4, 1);
    input.fault_map << 0.0, 0.0, 3.0, 0.0;
    input.rhs = VectorXd(4);
    input.rhs << 1.0, 2.0, 4.0, 9.0;
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    ASSERT_TRUE(summary.valid) << summary.invalid_reason;
    EXPECT_EQ(summary.nuPerp(), 2);
    EXPECT_EQ(summary.rank_boundary, 2);
    EXPECT_EQ(summary.F_b.rows(), 2);
    EXPECT_LE(std::abs(summary.F_b.squaredNorm() - 9.0), 1e-12);
    EXPECT_LE(std::abs(summary.d_perp.squaredNorm() - 97.0), 1e-12);
    EXPECT_LE(std::abs(summary.F_b.col(0).dot(summary.d_perp) - 12.0), 1e-12);
    EXPECT_LE(std::abs(summary.kappaBoundary() - 97.0), 1e-9);
    const IdentityErrors errors = measureIdentities(summary, input, rng, 8);
    EXPECT_LE(errors.cost, kTolWell);
    EXPECT_LE(errors.omega, kTolWell);
  }
  // (b) q = 0 (nominal boundary, no fault response).
  {
    const HistoryFaultSummaryInput input = makeInput(rng, 12, 4, 3, 0);
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    ASSERT_TRUE(summary.valid) << summary.invalid_reason;
    EXPECT_EQ(summary.T_b.rows(), 3);
    EXPECT_EQ(summary.T_b.cols(), 0);
    EXPECT_EQ(summary.F_b.cols(), 0);
    EXPECT_EQ(summary.omegaBoundary().size(), 0);
    EXPECT_EQ(summary.xiBoundary().size(), 0);
    EXPECT_LE(relErr(summary.kappaBoundary(),
                     minJointResidualCost(input.h_old_state, input.h_boundary,
                                          input.rhs)),
              kTolWell);
    const VectorXd shift = summary.boundaryMeanShiftForFault(VectorXd::Zero(0));
    EXPECT_EQ(shift.size(), 3);
    EXPECT_LE(shift.norm(), 1e-12);
    EXPECT_LE(relErrMatrix(summary.R_b.transpose() * summary.R_b,
                           marginalBoundaryInformation(input.h_old_state,
                                                       input.h_boundary)),
              kTolWell);
  }
  // (c) rank-deficient boundary (zero boundary column) is handled, not
  // rejected.
  {
    HistoryFaultSummaryInput input;
    input.h_old_state = MatrixXd::Zero(2, 0);
    input.h_boundary = MatrixXd::Zero(2, 1);
    input.fault_map = MatrixXd(2, 1);
    input.fault_map << 1.0, 0.0;
    input.rhs = VectorXd(2);
    input.rhs << 2.0, 5.0;
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    ASSERT_TRUE(summary.valid) << summary.invalid_reason;
    EXPECT_EQ(summary.rank_boundary, 0);
    EXPECT_FALSE(summary.boundaryShiftUsable());
    const VectorXd shift = summary.boundaryMeanShiftForFault(VectorXd::Ones(1));
    EXPECT_TRUE(allNan(shift));
    EXPECT_LE(std::abs(summary.kappaBoundary() - 25.0), 1e-9);
    const IdentityErrors errors = measureIdentities(summary, input, rng, 8);
    EXPECT_LE(errors.cost, kTolWell);
  }
}

TEST(HistoryFaultSummary, HISM1OracleTable) {
  // Identity 7: multi-size oracle table (well / moderate / ill / near-gate
  // tiers).  Each row prints the measured maximum relative error of every
  // identity so the evidence file can quote real numbers; assertions use the
  // ADR 0002 tier tolerances.
  struct Case {
    const char* name;
    int m;
    int n_old;
    int n_boundary;
    int n_fault;
    double kappa;
    double tolerance;
    const char* tier;
  };
  const std::vector<Case> cases = {
      {"small_well", 12, 4, 3, 2, 1.0, kTolWell, "well"},
      {"medium_well", 60, 10, 6, 4, 1.0, kTolWell, "well"},
      {"moderate_k1e4", 40, 8, 5, 3, 1e4, kTolModerate, "moderate"},
      {"ill_k1e8", 36, 6, 4, 3, 1e8, kTolModerate, "ill"},
      {"harsh_k1e10", 30, 5, 3, 2, 1e10, kTolNearGate, "harsh(tagged,1e-6)"},
  };
  Rng rng(20260928u);
  for (const Case& test_case : cases) {
    const HistoryFaultSummaryInput input =
        makeInput(rng, test_case.m, test_case.n_old, test_case.n_boundary,
                  test_case.n_fault, test_case.kappa);
    const HistoryFaultSummary summary = buildHistoryFaultSummary(input);
    ASSERT_TRUE(summary.valid)
        << test_case.name << ": " << summary.invalid_reason;
    const IdentityErrors errors = measureIdentities(summary, input, rng, 8);
    std::ostringstream row;
    row << "[HISM1-TABLE] case=" << test_case.name << " m=" << test_case.m
        << " no=" << test_case.n_old << " nb=" << test_case.n_boundary
        << " q=" << test_case.n_fault << " kappa_Ho=" << fmt(test_case.kappa)
        << " pivot_ratio_Ho=" << fmt(summary.old_state_pivot_ratio)
        << " tier=" << test_case.tier << " maxrel_cost=" << fmt(errors.cost)
        << " maxrel_schur=" << fmt(errors.schur)
        << " maxrel_omega=" << fmt(errors.omega)
        << " maxrel_kappa=" << fmt(errors.kappa)
        << " maxrel_shift=" << fmt(errors.shift)
        << " maxrel_delta=" << fmt(errors.delta)
        << " nu_mismatch=" << errors.nu_mismatch
        << " tol=" << fmt(test_case.tolerance);
    std::cout << row.str() << std::endl;
    EXPECT_LE(errors.cost, test_case.tolerance) << test_case.name;
    EXPECT_LE(errors.schur, test_case.tolerance) << test_case.name;
    EXPECT_LE(errors.omega, test_case.tolerance) << test_case.name;
    EXPECT_LE(errors.kappa, test_case.tolerance) << test_case.name;
    EXPECT_LE(errors.nu_mismatch, 0.0) << test_case.name;
    if (errors.shift_checked) {
      EXPECT_LE(errors.shift, test_case.tolerance) << test_case.name;
      EXPECT_LE(errors.delta, test_case.tolerance) << test_case.name;
    }
  }
}

TEST(HistoryFaultSummary, HISM1SplitHelperConsistency) {
  // splitBlockSystem() must reproduce the direct construction.
  Rng rng(20260929u);
  const HistoryFaultSummaryInput input = makeInput(rng, 20, 5, 3, 2);
  MatrixXd block(input.rows(), input.oldStateColumns() +
                                   input.boundaryColumns() +
                                   input.faultColumns() + 1);
  block.leftCols(input.oldStateColumns()) = input.h_old_state;
  block.middleCols(input.oldStateColumns(), input.boundaryColumns()) =
      input.h_boundary;
  block.middleCols(input.oldStateColumns() + input.boundaryColumns(),
                   input.faultColumns()) = input.fault_map;
  block.col(block.cols() - 1) = input.rhs;
  const auto split =
      splitBlockSystem(block, input.oldStateColumns(), input.boundaryColumns(),
                       input.faultColumns());
  ASSERT_TRUE(split.has_value());
  EXPECT_EQ((split->h_old_state - input.h_old_state).norm(), 0.0);
  EXPECT_EQ((split->h_boundary - input.h_boundary).norm(), 0.0);
  EXPECT_EQ((split->fault_map - input.fault_map).norm(), 0.0);
  EXPECT_EQ((split->rhs - input.rhs).norm(), 0.0);
  EXPECT_FALSE(splitBlockSystem(block, 1, 1, 1).has_value());  // wrong counts
  const HistoryFaultSummary from_split = buildHistoryFaultSummary(*split);
  const HistoryFaultSummary direct = buildHistoryFaultSummary(input);
  ASSERT_TRUE(from_split.valid);
  EXPECT_EQ((from_split.R_b - direct.R_b).norm(), 0.0);
  EXPECT_EQ((from_split.d_perp - direct.d_perp).norm(), 0.0);
}

TEST(HistoryFaultSummary, HISM2VersionDigestBindsAllComponents) {
  // C1-c/C3: the binding digest must be deterministic and must change when
  // ANY component (linearization / whitening / mode set / capacity) changes,
  // so a cached summary can never survive a rebuild with a changed binding.
  const uwb_imu_pl::HistorySummaryVersion base;
  const std::uint64_t base_digest =
      uwb_imu_pl::digestHistorySummaryVersion(base);
  EXPECT_EQ(base_digest, uwb_imu_pl::digestHistorySummaryVersion(
                             uwb_imu_pl::HistorySummaryVersion{}));
  uwb_imu_pl::HistorySummaryVersion changed = base;
  changed.linearization = 1;
  EXPECT_NE(base_digest, uwb_imu_pl::digestHistorySummaryVersion(changed));
  changed = base;
  changed.whitening = 1;
  EXPECT_NE(base_digest, uwb_imu_pl::digestHistorySummaryVersion(changed));
  changed = base;
  changed.mode_set = 1;
  EXPECT_NE(base_digest, uwb_imu_pl::digestHistorySummaryVersion(changed));
  changed = base;
  changed.capacity = 1;
  EXPECT_NE(base_digest, uwb_imu_pl::digestHistorySummaryVersion(changed));
  // Component order matters: swapping two distinct values is a change.
  uwb_imu_pl::HistorySummaryVersion swapped;
  swapped.linearization = 7;
  swapped.whitening = 9;
  uwb_imu_pl::HistorySummaryVersion reversed;
  reversed.linearization = 9;
  reversed.whitening = 7;
  EXPECT_NE(uwb_imu_pl::digestHistorySummaryVersion(swapped),
            uwb_imu_pl::digestHistorySummaryVersion(reversed));
}

TEST(HistoryFaultSummary, P103HierarchicalOrthogonalThirtyStepFeasibility) {
  using uwb_imu_pl::HistoryRootCacheRequest;
  using uwb_imu_pl::IncrementalHistoryRootCache;
  Rng rng(10301u);
  const auto owner = std::make_shared<const int>(17);
  constexpr int kOld = 5;
  constexpr int kBoundary = 4;
  constexpr int kFault = 3;
  constexpr int kColumns = kOld + kBoundary + kFault + 1;
  std::map<std::uint64_t, Eigen::MatrixXd> groups;
  for (std::uint64_t group = 100; group < 108; ++group)
    groups.emplace(group, rng.normal(4, kColumns));
  std::vector<int> state_order(kOld + kBoundary);
  std::iota(state_order.begin(), state_order.end(), 0);
  std::uint64_t ordering = 12;
  std::uint64_t marginalization = 14;
  std::uint64_t recovery = 15;

  auto make_request = [&]() {
    HistoryRootCacheRequest request;
    request.owner = owner;
    request.owner_payload = owner.get();
    request.linearization_version = 11;
    request.ordering_version = ordering;
    request.whitening_version = 13;
    request.marginalization_version = marginalization;
    request.recovery_version = recovery;
    request.column_order_digest = ordering;
    int rows = 0;
    for (const auto& item : groups) rows += item.second.rows();
    request.input.h_old_state.resize(rows, kOld);
    request.input.h_boundary.resize(rows, kBoundary);
    request.input.fault_map.resize(rows, kFault);
    request.input.rhs.resize(rows);
    for (int column = 0; column < kOld + kBoundary; ++column)
      request.state_column_uids.push_back(
          "state:" + std::to_string(state_order[column]));
    request.fault_column_uids = {"fault:0", "fault:1", "fault:2"};
    int cursor = 0;
    for (const auto& item : groups) {
      for (int row = 0; row < item.second.rows(); ++row, ++cursor) {
        for (int column = 0; column < kOld; ++column)
          request.input.h_old_state(cursor, column) =
              item.second(row, state_order[column]);
        for (int column = 0; column < kBoundary; ++column)
          request.input.h_boundary(cursor, column) =
              item.second(row, state_order[kOld + column]);
        request.input.fault_map.row(cursor) =
            item.second.row(row).segment(kOld + kBoundary, kFault);
        request.input.rhs(cursor) = item.second(row, kColumns - 1);
        request.row_uids.push_back("group:" + std::to_string(item.first) +
                                   ":row:" + std::to_string(row));
        request.row_provenance.push_back(item.first);
      }
    }
    return request;
  };

  auto relative_matrix = [](const Eigen::MatrixXd& a,
                            const Eigen::MatrixXd& b) {
    return (a - b).norm() / std::max({1.0, a.norm(), b.norm()});
  };
  auto relative_vector = [](const Eigen::VectorXd& a,
                            const Eigen::VectorXd& b) {
    return (a - b).norm() / std::max({1.0, a.norm(), b.norm()});
  };
  IncrementalHistoryRootCache cache;
  std::uint64_t next_group = 1000;
  for (int step = 0; step < 30; ++step) {
    if (step != 0) {
      switch (step % 6) {
        case 0:
          groups.emplace(next_group++, rng.normal(3, kColumns));
          break;
        case 1:
          groups.begin()->second(0, kColumns - 1) += 0.01 * step;
          break;
        case 2:
          if (groups.size() > 5) groups.erase(groups.begin());
          break;
        case 3:
          ++marginalization;
          groups.rbegin()->second(0, 0) -= 0.02 * step;
          break;
        case 4:
          std::swap(state_order[0], state_order[1]);
          ++ordering;
          break;
        case 5:
          ++recovery;
          break;
      }
    }
    const HistoryRootCacheRequest request = make_request();
    const HistoryFaultSummary incremental = cache.update(request);
    const HistoryFaultSummary rebuilt = buildHistoryFaultSummary(request.input);
    ASSERT_TRUE(incremental.valid) << "step=" << step << " "
                                   << incremental.invalid_reason;
    ASSERT_TRUE(rebuilt.valid) << rebuilt.invalid_reason;
    EXPECT_EQ(incremental.rank_h_old_state, rebuilt.rank_h_old_state);
    EXPECT_EQ(incremental.rank_boundary, rebuilt.rank_boundary);
    EXPECT_EQ(incremental.nuPerp(), rebuilt.nuPerp());
    EXPECT_LE(relative_matrix(incremental.R_b.transpose() * incremental.R_b,
                              rebuilt.R_b.transpose() * rebuilt.R_b), 2e-10);
    EXPECT_LE(relative_matrix(incremental.R_b.transpose() * incremental.T_b,
                              rebuilt.R_b.transpose() * rebuilt.T_b), 2e-10);
    EXPECT_LE(relative_vector(incremental.R_b.transpose() * incremental.d_b,
                              rebuilt.R_b.transpose() * rebuilt.d_b), 2e-10);
    EXPECT_LE(relative_matrix(incremental.omegaBoundary(),
                              rebuilt.omegaBoundary()), 2e-10);
    EXPECT_LE(relative_vector(incremental.xiBoundary(), rebuilt.xiBoundary()),
              2e-10);
    EXPECT_NEAR(incremental.kappaBoundary(), rebuilt.kappaBoundary(),
                2e-10 * std::max({1.0, incremental.kappaBoundary(),
                                  rebuilt.kappaBoundary()}));
    const Eigen::VectorXd boundary = rng.normalVector(kBoundary);
    const Eigen::VectorXd fault = rng.normalVector(kFault);
    const double incremental_objective =
        (incremental.R_b * boundary + incremental.T_b * fault -
         incremental.d_b).squaredNorm() +
        (incremental.F_b * fault - incremental.d_perp).squaredNorm();
    const double rebuilt_objective =
        (rebuilt.R_b * boundary + rebuilt.T_b * fault - rebuilt.d_b)
            .squaredNorm() +
        (rebuilt.F_b * fault - rebuilt.d_perp).squaredNorm();
    EXPECT_NEAR(incremental_objective, rebuilt_objective,
                3e-10 * std::max({1.0, incremental_objective,
                                  rebuilt_objective}));
  }
  const auto audit = cache.audit();
  EXPECT_GT(audit.incremental_path_updates, 0u);
  EXPECT_GT(audit.incremental_add_paths, 0u);
  EXPECT_GT(audit.incremental_remove_paths, 0u);
  EXPECT_GT(audit.incremental_relinearize_paths, 0u);
  EXPECT_LT(audit.full_tree_rebuilds, audit.requests);
  std::printf("[P103-FEASIBILITY] requests=%llu path_updates=%llu add=%llu "
              "remove=%llu relinearize=%llu internal=%llu full_tree=%llu\n",
              static_cast<unsigned long long>(audit.requests),
              static_cast<unsigned long long>(audit.incremental_path_updates),
              static_cast<unsigned long long>(audit.incremental_add_paths),
              static_cast<unsigned long long>(audit.incremental_remove_paths),
              static_cast<unsigned long long>(audit.incremental_relinearize_paths),
              static_cast<unsigned long long>(audit.internal_nodes_recomputed),
              static_cast<unsigned long long>(audit.full_tree_rebuilds));
}

TEST(HistoryFaultSummary, P103OwnerCollisionTamperAndStaleCacheFailClosed) {
  using uwb_imu_pl::HistoryRootCacheRequest;
  using uwb_imu_pl::IncrementalHistoryRootCache;
  Rng rng(10302u);
  HistoryRootCacheRequest request;
  request.input = makeInput(rng, 18, 4, 3, 2);
  auto owner_a = std::make_shared<const int>(7);
  request.owner = owner_a;
  request.owner_payload = owner_a.get();
  request.linearization_version = 21;
  request.ordering_version = 22;
  request.whitening_version = 23;
  request.marginalization_version = 24;
  request.recovery_version = 25;
  request.column_order_digest = 0xfeedbeefULL;
  for (int row = 0; row < request.input.rows(); ++row) {
    request.row_uids.push_back("owner-row:" + std::to_string(row));
    request.row_provenance.push_back(2000 + row);
  }
  for (int column = 0;
       column < request.input.oldStateColumns() +
                    request.input.boundaryColumns();
       ++column) {
    request.state_column_uids.push_back("owner-state:" +
                                        std::to_string(column));
  }
  request.fault_column_uids = {"owner-fault:0", "owner-fault:1"};
  IncrementalHistoryRootCache cache;
  ASSERT_TRUE(cache.update(request).valid);

  // Equal content and every scalar identity still cannot cross an ownership
  // boundary.  A deliberately unchanged digest models a forced collision.
  auto transplanted = request;
  auto owner_b = std::make_shared<const int>(7);
  transplanted.owner = owner_b;
  transplanted.owner_payload = owner_b.get();
  ASSERT_TRUE(cache.update(transplanted).valid);
  EXPECT_NE(cache.audit().last_reason.find("full tree rebuild"),
            std::string::npos);

  // Same owner and IDs but a one-bit payload change is not reusable.
  auto tampered = transplanted;
  tampered.input.fault_map(0, 0) =
      std::nextafter(tampered.input.fault_map(0, 0), 1.0);
  ASSERT_TRUE(cache.update(tampered).valid);
  EXPECT_NE(cache.audit().last_reason.find("changed group paths"),
            std::string::npos);

  // Missing or duplicate provenance never reaches a cached or rebuilt root.
  auto stale = tampered;
  stale.row_provenance.pop_back();
  const auto missing = cache.update(stale);
  EXPECT_FALSE(missing.valid);
  EXPECT_NE(missing.invalid_reason.find("census"), std::string::npos);
  stale = tampered;
  stale.row_uids[1] = stale.row_uids[0];
  const auto duplicate = cache.update(stale);
  EXPECT_FALSE(duplicate.valid);
  EXPECT_NE(duplicate.invalid_reason.find("not unique"), std::string::npos);
  EXPECT_EQ(cache.audit().retained_rows, 0u);
  EXPECT_EQ(cache.audit().retained_bytes, 0u);
}

TEST(HistoryFaultSummary, P103CarrierRankCertificateRejectsNumericalNoise) {
  HistoryFaultSummaryInput input;
  input.h_old_state = MatrixXd::Zero(4, 0);
  input.h_boundary = MatrixXd::Zero(4, 0);
  input.fault_map = MatrixXd::Zero(4, 2);
  input.rhs = VectorXd::Zero(4);
  input.fault_map(0, 0) = 1.0;
  input.fault_map(1, 1) = 1e-8;
  input.rhs(2) = 1e-14;  // deliberately nonzero storage noise
  HistoryFaultSummaryOptions options;
  options.rank_tolerance = 1e-10;
  const CertifiedHistoryFaultSummary certified =
      buildCertifiedHistoryFaultSummary(input, options);
  const HistoryFaultSummary& summary = certified.summary;
  const HistoryCarrierRankCertificate& rank = certified.carrier_rank;
  ASSERT_TRUE(summary.valid) << summary.invalid_reason;
  ASSERT_TRUE(rank.valid);
  EXPECT_EQ(rank.storage_rows_before_compression, 4);
  EXPECT_EQ(rank.raw_residual_dof, 4);
  EXPECT_EQ(rank.effective_rank, 2);
  EXPECT_EQ(summary.nuPerp(), 2);
  EXPECT_EQ(summary.F_b.rows(), 2);
  EXPECT_EQ(summary.d_perp.size(), 2);
  EXPECT_EQ(rank.singular_values.size(), 3);
  EXPECT_GT(rank.rank_threshold, 1e-14);
  EXPECT_GE(rank.discarded_frobenius_bound, 1e-14);
  EXPECT_NE(rank.proof_identity, 0u);
  int independent_rank = 0;
  for (Eigen::Index i = 0;
       i < rank.singular_values.size(); ++i) {
    if (rank.singular_values(i) > rank.rank_threshold) ++independent_rank;
  }
  EXPECT_EQ(independent_rank, summary.nuPerp());

  // Relative conditioning, rather than absolute element magnitude, controls
  // the decision.  Uniform scaling preserves effective rank while changing
  // the proof identity and all dimensional bounds coherently.
  HistoryFaultSummaryInput scaled = input;
  scaled.fault_map *= 1e6;
  scaled.rhs *= 1e6;
  const CertifiedHistoryFaultSummary scaled_certified =
      buildCertifiedHistoryFaultSummary(scaled, options);
  const HistoryFaultSummary& scaled_summary = scaled_certified.summary;
  ASSERT_TRUE(scaled_summary.valid) << scaled_summary.invalid_reason;
  EXPECT_EQ(scaled_summary.nuPerp(), summary.nuPerp());
  EXPECT_NE(scaled_certified.carrier_rank.proof_identity,
            rank.proof_identity);
}

}  // namespace
