// B1 (P3): tests for the unified square-root numerical context.
//
// NUM-01  scale/permutation invariance, with the protected map C synchronized
// NUM-02  detector-only rows (zero H rows) keep residual, response and dof
// NUM-03  rank deficiency -> explicit STATE_OUTPUT_UNOBSERVABLE, and a failing
//         certificate falls back to the reference path (counted, never silent)
// NUM-04  symbolic plan caching: pattern reuse hits, pattern change misses,
//         and numerical factors are rebuilt (never reused) across windows
// COV-02  auxiliary subset / projection identity: Z^T Z == D^T (I - H P H^T) D
//         and the subset quadratic form matches the dense reference
#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"

#include <gtest/gtest.h>

#include <Eigen/SVD>

#include <cmath>
#include <random>
#include <sstream>

namespace {

using uwb_imu_pl::ColumnPermutationPolicy;
using uwb_imu_pl::ColumnScalePolicy;
using uwb_imu_pl::FrozenSquareRootContext;
using uwb_imu_pl::FrozenWindowNumerics;
using uwb_imu_pl::LinearizedIntegrityWindow;
using uwb_imu_pl::NumericalWorkCounters;
using uwb_imu_pl::solveFrozenInformation;

Eigen::MatrixXd randomMatrix(int rows, int columns, unsigned seed) {
  Eigen::MatrixXd matrix(rows, columns);
  std::mt19937 rng(seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  for (int column = 0; column < columns; ++column) {
    for (int row = 0; row < rows; ++row) matrix(row, column) = normal(rng);
  }
  return matrix;
}

Eigen::MatrixXd referenceInverse(const Eigen::MatrixXd& matrix) {
  return matrix.fullPivLu().inverse();
}

double maxAbsDifference(const Eigen::MatrixXd& left,
                        const Eigen::MatrixXd& right) {
  return (left - right).cwiseAbs().maxCoeff();
}

std::string certificateSummary(
    const uwb_imu_pl::SquareRootCertificate& certificate) {
  std::ostringstream text;
  text << "rank=" << certificate.full_column_rank
       << " identity=" << certificate.identity_ok
       << " (" << certificate.identity_residual_relative << ")"
       << " parity=" << certificate.parity_matches_reference
       << " (" << certificate.parity_relative_difference << ")"
       << " solution=" << certificate.solution_matches_reference
       << " (" << certificate.solution_relative_difference << ")"
       << " forward=" << certificate.forward_error_ok
       << " (" << certificate.forward_error_bound << ")"
       << " condition=" << certificate.condition_estimate;
  return text.str();
}

Eigen::Matrix<double, 3, Eigen::Dynamic> protectedMapFor(int columns) {
  Eigen::Matrix<double, 3, Eigen::Dynamic> map =
      Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, columns);
  map(0, 0) = 1.0;
  map(1, 1) = 1.0;
  map(2, 2) = 1.0;
  return map;
}

std::shared_ptr<const FrozenSquareRootContext> buildContext(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& C,
    ColumnScalePolicy scale, ColumnPermutationPolicy permutation,
    const Eigen::VectorXd& reference_solution, double reference_statistic) {
  return FrozenSquareRootContext::build(H, z, C, scale, permutation, 1e-10,
                                        reference_solution, reference_statistic);
}

TEST(SquareRootContext, NUM01_ScaleAndPermutationInvarianceWithProtectedSync) {
  const int m = 24, n = 8;
  const Eigen::MatrixXd H = randomMatrix(m, n, 11);
  const Eigen::VectorXd z = randomMatrix(m, 1, 12);
  const auto C = protectedMapFor(n);
  const Eigen::MatrixXd information = H.transpose() * H;
  const Eigen::MatrixXd reference = referenceInverse(information);
  const Eigen::MatrixXd rhs = randomMatrix(n, 4, 13);
  const Eigen::MatrixXd expected = reference * rhs;
  const Eigen::VectorXd reference_solution =
      (H.transpose() * H).ldlt().solve(H.transpose() * z);
  const double reference_statistic =
      (z - H * reference_solution).squaredNorm();
  const Eigen::Matrix3d expected_protected_covariance =
      (C * reference * C.transpose()).eval();

  for (const auto scale : {ColumnScalePolicy::Unit,
                           ColumnScalePolicy::ColumnNorm}) {
    for (const auto permutation : {ColumnPermutationPolicy::Natural,
                                   ColumnPermutationPolicy::ColumnPivot}) {
      const auto context = buildContext(H, z, C, scale, permutation,
                                        reference_solution,
                                        reference_statistic);
      ASSERT_TRUE(context != nullptr);
      EXPECT_TRUE(context->certificate().ok())
          << uwb_imu_pl::toString(scale) << "/"
          << uwb_imu_pl::toString(permutation)
          << ": " << certificateSummary(context->certificate());
      EXPECT_TRUE(context->usable());
      EXPECT_EQ(context->rank(), n);
      EXPECT_EQ(context->dof(), m - n);

      // Information solve is invariant under S and Pmat.
      const Eigen::MatrixXd solved = context->informationSolve(rhs);
      ASSERT_EQ(solved.rows(), n);
      EXPECT_LT(maxAbsDifference(solved, expected), 1e-9)
          << uwb_imu_pl::toString(scale) << "/"
          << uwb_imu_pl::toString(permutation);

      // Least squares on the measurement side agrees with the information
      // solve applied to H^T b.
      const Eigen::MatrixXd b = randomMatrix(m, 3, 14);
      const Eigen::MatrixXd least_squares = context->leastSquaresSolve(b);
      EXPECT_LT(maxAbsDifference(
                    least_squares, context->informationSolve(H.transpose() * b)),
                1e-9);

      // The nominal increment and the parity statistic match the SVD reference.
      EXPECT_LT((context->baseStateIncrement() - reference_solution).norm() /
                    std::max(1.0, reference_solution.norm()),
                1e-8);
      EXPECT_LT(std::abs(context->statistic() - reference_statistic) /
                    std::max(1.0, reference_statistic),
                1e-9);

      // Protected covariance uses C~ = C S P; with non-trivial policies the
      // naive map C would give a different (wrong) answer, which is exactly the
      // mistake this test is here to catch.
      const Eigen::Matrix3d covariance = context->protectedCovariance();
      EXPECT_LT(maxAbsDifference(covariance, expected_protected_covariance), 1e-9)
          << uwb_imu_pl::toString(scale) << "/"
          << uwb_imu_pl::toString(permutation);
      const Eigen::MatrixXd expected_map =
          (C * context->columnScale().asDiagonal()).eval() *
          Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>(
              Eigen::Map<const Eigen::VectorXi>(
                  context->columnPermutation().data(), n));
      EXPECT_LT(maxAbsDifference(context->protectedMap(), expected_map), 1e-12);
      const bool trivial = scale == ColumnScalePolicy::Unit &&
          permutation == ColumnPermutationPolicy::Natural;
      const bool map_equals_C =
          maxAbsDifference(context->protectedMap(), C) < 1e-12;
      EXPECT_EQ(map_equals_C, trivial);
      // Forgetting S/Pmat (i.e. using C against the factored R and the scaled
      // basis) must NOT reproduce the protected covariance unless the policies
      // are trivial; this is the mistake the test exists to catch.
      const Eigen::MatrixXd factored_information =
          context->rFactor().transpose() * context->rFactor();
      const Eigen::Matrix3d naive_covariance =
          (C * referenceInverse(factored_information) * C.transpose()).eval();
      if (!trivial) {
        EXPECT_GT(maxAbsDifference(naive_covariance, covariance), 1e-12)
            << "C must be permuted/scaled together with H";
      } else {
        EXPECT_LT(maxAbsDifference(naive_covariance, covariance), 1e-9);
      }
    }
  }
}

TEST(SquareRootContext, NUM02_DetectorOnlyRowsKeepResidualDofAndResponse) {
  const int m = 20, n = 6;
  Eigen::MatrixXd H = randomMatrix(m, n, 21);
  // Rows 3 and 11 carry no state information at all: pure detection rows.
  H.row(3).setZero();
  H.row(11).setZero();
  Eigen::VectorXd z = randomMatrix(m, 1, 22);
  const auto C = protectedMapFor(n);
  const Eigen::VectorXd reference_solution =
      (H.transpose() * H).ldlt().solve(H.transpose() * z);
  const double reference_statistic = (z - H * reference_solution).squaredNorm();
  const auto context = buildContext(H, z, C, ColumnScalePolicy::Unit,
                                    ColumnPermutationPolicy::Natural,
                                    reference_solution, reference_statistic);
  ASSERT_TRUE(context != nullptr);
  ASSERT_TRUE(context->usable());
  EXPECT_EQ(context->detectorOnlyRows(), 2);
  EXPECT_EQ(context->rank(), n);
  EXPECT_EQ(context->dof(), m - n);  // the zero rows stay in the dof count

  // The residual on the zero rows is preserved in the statistic.
  ASSERT_GT(context->parity().size(), 3);
  EXPECT_GT(std::abs(context->parity()(3)), 1e-12);
  EXPECT_GT(std::abs(context->parity()(11)), 1e-12);
  EXPECT_NEAR(context->parity().squaredNorm(), context->statistic(),
              std::max(1e-12, 1e-9 * context->statistic()));

  // A fault that lives only on the zero rows still reaches the detection space.
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(m, 2);
  D(3, 0) = 1.0;
  D(11, 1) = 1.0;
  Eigen::MatrixXd Y, Z;
  context->faultResponse(D, &Y, &Z);
  ASSERT_EQ(Z.cols(), 2);
  EXPECT_GT(Z.col(0).norm(), 0.5);
  EXPECT_GT(Z.col(1).norm(), 0.5);
  EXPECT_LT(Y.norm(), 1e-12);  // no state-space response, detection only
  // Projection identity: Z^T Z == D^T (I - H P H^T) D.
  const Eigen::MatrixXd P = referenceInverse(H.transpose() * H);
  const Eigen::MatrixXd projector = Eigen::MatrixXd::Identity(m, m) - H * P * H.transpose();
  EXPECT_LT(maxAbsDifference(Z.transpose() * Z, D.transpose() * projector * D),
            1e-12);
}

TEST(SquareRootContext, NUM03_RankDeficientWindowIsExplicitlyUnobservable) {
  const int m = 18, n = 5;
  Eigen::MatrixXd H = randomMatrix(m, n, 31);
  H.col(3) = H.col(1);  // duplicated column -> structural rank deficiency
  const Eigen::VectorXd z = randomMatrix(m, 1, 32);
  const auto C = protectedMapFor(n);
  const auto context = buildContext(H, z, C, ColumnScalePolicy::Unit,
                                    ColumnPermutationPolicy::Natural,
                                    Eigen::VectorXd(),
                                    std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(context != nullptr);
  EXPECT_FALSE(context->usable());
  EXPECT_LT(context->rank(), n);
  EXPECT_NE(context->reason().find("STATE_OUTPUT_UNOBSERVABLE"),
            std::string::npos);
  // No small regularization may fake a solve.
  const Eigen::MatrixXd rhs = randomMatrix(n, 2, 33);
  EXPECT_EQ(context->informationSolve(rhs).size(), 0);
  EXPECT_EQ(context->leastSquaresSolve(randomMatrix(m, 2, 34)).size(), 0);
  EXPECT_EQ(context->protectedCovariance().array().isNaN().count(), 9);
}

TEST(SquareRootContext, NUM03b_CertificateFailureFallsBackToReferencePath) {
  const int m = 16, n = 4;
  const Eigen::MatrixXd H = randomMatrix(m, n, 41);
  const Eigen::VectorXd z = randomMatrix(m, 1, 42);
  const auto C = protectedMapFor(n);
  const auto context = buildContext(H, z, C, ColumnScalePolicy::Unit,
                                    ColumnPermutationPolicy::Natural,
                                    Eigen::VectorXd(),
                                    std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(context != nullptr);
  ASSERT_TRUE(context->usable()) << certificateSummary(context->certificate());

  // Reference fallback machinery: an LLT-based numerics record.
  FrozenWindowNumerics numerics;
  const Eigen::MatrixXd information = H.transpose() * H;
  numerics.information_factorization =
      std::make_shared<const Eigen::LLT<Eigen::MatrixXd>>(information);
  numerics.numerical_contract.forward_error_limit = 1e-7;
  numerics.exact_condition = 4.0;
  const Eigen::MatrixXd rhs = randomMatrix(n, 3, 43);
  const Eigen::MatrixXd expected = referenceInverse(information) * rhs;

  const auto counters_before = NumericalWorkCounters::snapshot();
  // A usable context serves the solve without touching the LLT path.
  const Eigen::MatrixXd direct =
      solveFrozenInformation(context.get(), numerics, information, rhs);
  EXPECT_LT(maxAbsDifference(direct, expected), 1e-9);
  EXPECT_EQ(NumericalWorkCounters::snapshot().square_root_fallbacks,
            counters_before.square_root_fallbacks);

  // An unusable context (here: explicitly built rank deficient) is never used
  // silently: the solve is counted as a fallback and matches the reference.
  Eigen::MatrixXd deficient = H;
  deficient.col(2) = deficient.col(0);
  const auto unusable = buildContext(deficient, z, C, ColumnScalePolicy::Unit,
                                     ColumnPermutationPolicy::Natural,
                                     Eigen::VectorXd(),
                                     std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(unusable != nullptr);
  ASSERT_FALSE(unusable->usable());
  const auto before_fallback = NumericalWorkCounters::snapshot();
  const Eigen::MatrixXd fallback =
      solveFrozenInformation(unusable.get(), numerics, information, rhs);
  EXPECT_LT(maxAbsDifference(fallback, expected), 1e-9);
  EXPECT_EQ(NumericalWorkCounters::snapshot().square_root_fallbacks,
            before_fallback.square_root_fallbacks + 1);
}

TEST(SquareRootContext, NUM04_SymbolicPlanCacheAndNumericRebuild) {
  FrozenSquareRootContext::clearSymbolicCache();
  const int m = 22, n = 7;
  const Eigen::MatrixXd H = randomMatrix(m, n, 51);
  const std::array<double, 3> z_values{0.5, -0.25, 1.0};
  double statistic_reference = 0.0;
  Eigen::VectorXd reference_solution;
  Eigen::VectorXd last_z;
  std::vector<Eigen::MatrixXd> r_factors;
  std::vector<std::uint64_t> fingerprints;
  for (int repeat = 0; repeat < 3; ++repeat) {
    const Eigen::VectorXd z =
        Eigen::VectorXd::Constant(m, z_values[static_cast<std::size_t>(repeat)]);
    last_z = z;
    reference_solution = (H.transpose() * H).ldlt().solve(H.transpose() * z);
    statistic_reference = (z - H * reference_solution).squaredNorm();
    const auto context = buildContext(H, z, protectedMapFor(n),
                                      ColumnScalePolicy::Unit,
                                      ColumnPermutationPolicy::Natural,
                                      reference_solution, statistic_reference);
    ASSERT_TRUE(context != nullptr);
    r_factors.push_back(context->rFactor());
    fingerprints.push_back(context->contentFingerprint());
  }
  // Same pattern: one miss then two hits, one entry.  R depends on H and the
  // policies, not on z, so the three repeats share R but must carry different
  // content fingerprints (numeric factors are rebuilt per window, never cached
  // by value).
  const auto stats = FrozenSquareRootContext::symbolicCacheStats();
  EXPECT_EQ(stats.misses, 1u);
  EXPECT_EQ(stats.hits, 2u);
  EXPECT_EQ(stats.entries, 1u);
  EXPECT_NE(fingerprints[0], fingerprints[1]);
  EXPECT_LT(maxAbsDifference(r_factors[0], r_factors[2]), 1e-14)
      << "R is a function of H, not of z";

  // Different H values with the same pattern: the symbolic plan is reused but
  // the numeric factor must change.
  Eigen::MatrixXd H_scaled = H * 3.0;
  const auto rescaled = buildContext(H_scaled, last_z, protectedMapFor(n),
                                     ColumnScalePolicy::Unit,
                                     ColumnPermutationPolicy::Natural,
                                     Eigen::VectorXd(),
                                     std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(rescaled != nullptr);
  EXPECT_GT(maxAbsDifference(rescaled->rFactor(), r_factors[0]), 1e-6)
      << "R must be rebuilt when H changes";
  const auto after_same_pattern = FrozenSquareRootContext::symbolicCacheStats();
  EXPECT_EQ(after_same_pattern.misses, 1u);
  EXPECT_EQ(after_same_pattern.hits, 3u);

  // A pattern change must miss the cache again.
  Eigen::MatrixXd H_changed = H;
  H_changed(5, 2) = 0.0;  // remove a nonzero -> new pattern
  const Eigen::VectorXd z = Eigen::VectorXd::Constant(m, 0.5);
  const auto changed = buildContext(H_changed, z, protectedMapFor(n),
                                    ColumnScalePolicy::Unit,
                                    ColumnPermutationPolicy::Natural,
                                    Eigen::VectorXd(),
                                    std::numeric_limits<double>::quiet_NaN());
  ASSERT_TRUE(changed != nullptr);
  const auto after = FrozenSquareRootContext::symbolicCacheStats();
  EXPECT_EQ(after.misses, 2u);
  EXPECT_EQ(after.hits, 3u);
  EXPECT_EQ(after.entries, 2u);
  FrozenSquareRootContext::clearSymbolicCache();
}

TEST(SquareRootContext, COV02_AuxiliarySubsetProjectionIdentity) {
  const int m = 26, n = 7;
  const Eigen::MatrixXd H = randomMatrix(m, n, 71);
  const Eigen::VectorXd z = randomMatrix(m, 1, 72);
  const auto C = protectedMapFor(n);
  const Eigen::VectorXd reference_solution =
      (H.transpose() * H).ldlt().solve(H.transpose() * z);
  const auto context = buildContext(H, z, C, ColumnScalePolicy::Unit,
                                    ColumnPermutationPolicy::Natural,
                                    reference_solution,
                                    (z - H * reference_solution).squaredNorm());
  ASSERT_TRUE(context != nullptr);
  ASSERT_TRUE(context->usable());

  const Eigen::MatrixXd D = randomMatrix(m, 5, 73);
  Eigen::MatrixXd Y, Z;
  context->faultResponse(D, &Y, &Z);
  const Eigen::MatrixXd P = referenceInverse(H.transpose() * H);
  const Eigen::MatrixXd projector =
      Eigen::MatrixXd::Identity(m, m) - H * P * H.transpose();
  // Projection identity for the full fault set.
  EXPECT_LT(maxAbsDifference(Z.transpose() * Z, D.transpose() * projector * D),
            1e-12);

  // Auxiliary subset: keep columns {0,2,3}; the subset Gram and the subset
  // protected quadratic form must match the dense reference computed with the
  // same covariance P and projection.
  Eigen::MatrixXi selection = Eigen::MatrixXi::Zero(5, 3);
  selection(0, 0) = 1; selection(2, 1) = 1; selection(3, 2) = 1;
  const Eigen::MatrixXd D_subset = D * selection.cast<double>();
  const Eigen::MatrixXd Z_subset = Z * selection.cast<double>();
  const Eigen::MatrixXd gamma_subset = Z_subset.transpose() * Z_subset;
  EXPECT_LT(maxAbsDifference(gamma_subset,
                             D_subset.transpose() * projector * D_subset),
            1e-12);
  const Eigen::MatrixXd G_subset = C * P * H.transpose() * D_subset;
  const Eigen::MatrixXd subset_form = G_subset *
      gamma_subset.ldlt().solve(Eigen::MatrixXd::Identity(3, 3)) *
      G_subset.transpose();
  // Dense reference of the same subset quadratic form.
  const Eigen::MatrixXd dense_reference =
      (G_subset * referenceInverse(gamma_subset) * G_subset.transpose());
  EXPECT_LT(maxAbsDifference(subset_form, dense_reference), 1e-12);

  // The subset route must reproduce the corresponding columns of the full
  // formulation when the discarded directions are set to zero.
  Eigen::MatrixXd D_full_like = D;
  D_full_like.col(1).setZero();
  D_full_like.col(4).setZero();
  Eigen::MatrixXd Y_full, Z_full;
  context->faultResponse(D_full_like, &Y_full, &Z_full);
  const Eigen::MatrixXd gamma_full = Z_full.transpose() * Z_full;
  const Eigen::MatrixXd G_full = C * P * H.transpose() * D_full_like;
  for (int kept = 0; kept < 3; ++kept) {
    const int source = kept == 0 ? 0 : (kept == 1 ? 2 : 3);
    EXPECT_LT((G_full.col(source) - G_subset.col(kept)).norm(), 1e-12);
    for (int other = 0; other < 3; ++other) {
      const int other_source = other == 0 ? 0 : (other == 1 ? 2 : 3);
      EXPECT_LT(std::abs(gamma_full(source, other_source) -
                          gamma_subset(kept, other)),
                1e-12);
    }
  }
}

TEST(SquareRootContext, ClassifiesDetectionResponseTriState) {
  // Full rank: two independent response directions.
  Eigen::MatrixXd z_full(5, 2);
  z_full << 1, 0,
            0, 2,
            1, 1,
            0.5, 0.25,
            0, 1;
  Eigen::MatrixXd g_full(3, 2);
  g_full << 1, 0, 0, 1, 0, 0;
  double sigma = 0.0, condition = 0.0;
  int rank = 0;
  EXPECT_EQ(uwb_imu_pl::classifyDetectionResponse(z_full, g_full, 1e-10,
                                                  &sigma, &condition, &rank),
            1);
  EXPECT_EQ(rank, 2);

  // Harmless nullspace: the second response direction is invisible (zero
  // column) and has no protected-state effect.
  Eigen::MatrixXd z_harmless(5, 2);
  z_harmless << 1, 0,
                0, 0,
                1, 0,
                0.5, 0,
                0, 0;
  Eigen::MatrixXd g_harmless(3, 2);
  g_harmless << 1, 0, 0, 0, 0, 0;
  EXPECT_EQ(uwb_imu_pl::classifyDetectionResponse(z_harmless, g_harmless, 1e-10,
                                                  &sigma, &condition, &rank),
            2);
  EXPECT_EQ(rank, 1);

  // Dangerous nullspace: invisible direction that does move the protected
  // state (the column is invisible to detection but the response is not zero).
  Eigen::MatrixXd g_dangerous = g_harmless;
  g_dangerous(0, 1) = 0.5;
  EXPECT_EQ(uwb_imu_pl::classifyDetectionResponse(z_harmless, g_dangerous,
                                                  1e-10, &sigma, &condition,
                                                  &rank),
            3);

  // Numerically indistinguishable: smallest singular value just inside the
  // rank-tolerance band.
  Eigen::MatrixXd z_ambiguous(4, 2);
  z_ambiguous << 1, 0,
                 0, 1e-11,
                 1, 0,
                 0, 1e-11;
  EXPECT_EQ(uwb_imu_pl::classifyDetectionResponse(z_ambiguous, g_dangerous,
                                                  1e-10, &sigma, &condition,
                                                  &rank),
            4);
}

}  // namespace
