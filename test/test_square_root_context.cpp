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
#include <cstdlib>
#include <cstring>
#include "../src/uwb_imu_pl/estimation/classification_numerics.hpp"
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

  // Below the common relative gate the second direction is a certified
  // nullspace, and its nonzero protected response is dangerous.
  Eigen::MatrixXd z_ambiguous(4, 2);
  z_ambiguous << 1, 0,
                 0, 1e-11,
                 1, 0,
                 0, 1e-11;
  EXPECT_EQ(uwb_imu_pl::classifyDetectionResponse(z_ambiguous, g_dangerous,
                                                  1e-10, &sigma, &condition,
                                                  &rank),
            3);
}

TEST(SquareRootContext, P003UnifiedPsdRankAndNullspaceCertificate) {
  using uwb_imu_pl::GramNullspaceClass;
  using uwb_imu_pl::certifyGramAndProtectedResponse;
  using uwb_imu_pl::certifySymmetricPsd;

  const Eigen::Matrix2d indefinite =
      Eigen::Vector2d(1.0, -1e-6).asDiagonal();
  const auto rejected = certifySymmetricPsd(indefinite, 1e-10, 17);
  EXPECT_FALSE(rejected.valid);
  EXPECT_NE(rejected.reason.find("not PSD"), std::string::npos);

  Eigen::Matrix2d asymmetric;
  asymmetric << 1.0, 1e-3, 0.0, 1.0;
  const auto nonsymmetric = certifySymmetricPsd(asymmetric, 1e-10, 17);
  EXPECT_FALSE(nonsymmetric.valid);
  EXPECT_NE(nonsymmetric.reason.find("not symmetric"), std::string::npos);

  // A tiny but nonzero map is not structural zero for an unbounded fault.
  Eigen::Matrix<double, 1, 1> tiny_gram;
  tiny_gram(0, 0) = 1e-26;
  Eigen::Matrix<double, 3, 1> protected_response;
  protected_response << 1.0, 0.0, 0.0;
  const auto tiny = certifyGramAndProtectedResponse(
      tiny_gram, protected_response, 1e-10, 19, 1.0);
  ASSERT_TRUE(tiny.valid) << tiny.reason;
  EXPECT_EQ(tiny.nullspace_class, GramNullspaceClass::Dangerous);

  protected_response.setZero();
  const auto harmless = certifyGramAndProtectedResponse(
      tiny_gram, protected_response, 1e-10, 19, 1.0);
  ASSERT_TRUE(harmless.valid) << harmless.reason;
  EXPECT_EQ(harmless.nullspace_class, GramNullspaceClass::Harmless);
  EXPECT_TRUE(harmless.protected_slopes.isZero(0.0));

  // Exactly on the rank boundary the certificate refuses a discrete answer.
  Eigen::Matrix2d transition = Eigen::Matrix2d::Zero();
  transition(0, 0) = 1.0;
  transition(1, 1) = 1e-20;
  const auto boundary = certifySymmetricPsd(transition, 1e-10, 23);
  EXPECT_FALSE(boundary.valid);
  EXPECT_NE(boundary.reason.find("rank transition"), std::string::npos);
}

TEST(SquareRootContext, P003ReviewerScaleRankAndProofIdentityProbes) {
  using uwb_imu_pl::GramNullspaceClass;
  using uwb_imu_pl::certifyGramAndProtectedResponse;
  using uwb_imu_pl::certifySymmetricPsd;

  Eigen::Matrix<double, 1, 1> tiny_negative;
  tiny_negative(0, 0) = -1e-16;
  const auto negative = certifySymmetricPsd(tiny_negative, 1e-10, 31);
  EXPECT_FALSE(negative.valid) << negative.reason;

  Eigen::Matrix2d tiny_asymmetric = Eigen::Matrix2d::Zero();
  tiny_asymmetric.diagonal().setConstant(1e-20);
  tiny_asymmetric(0, 1) = 1e-28;
  const auto asymmetric = certifySymmetricPsd(tiny_asymmetric, 1e-10, 31);
  EXPECT_FALSE(asymmetric.valid) << asymmetric.reason;

  Eigen::Matrix<double, 1, 1> zero_gram;
  zero_gram.setZero();
  Eigen::Matrix<double, 3, 1> nonzero_response;
  nonzero_response << 1e-16, 0.0, 0.0;
  const auto unbounded = certifyGramAndProtectedResponse(
      zero_gram, nonzero_response, 1e-10, 31);
  ASSERT_TRUE(unbounded.valid) << unbounded.reason;
  EXPECT_EQ(unbounded.nullspace_class, GramNullspaceClass::Dangerous);

  const double singular_values[] = {
      1e-12, 1e-11, 1e-10, 1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4};
  for (const double smallest : singular_values) {
    SCOPED_TRACE(smallest);
    Eigen::Matrix2d z = Eigen::Matrix2d::Identity();
    z(1, 1) = smallest;
    int raw_rank = 0;
    const int classification = uwb_imu_pl::classifyDetectionResponse(
        z, Eigen::Matrix<double, 3, 2>::Zero(), 1e-10,
        nullptr, nullptr, &raw_rank, nullptr, nullptr);
    const auto gram = certifySymmetricPsd(z.transpose() * z, 1e-10, 37);
    if (smallest == 1e-10) {
      EXPECT_FALSE(gram.valid);
      EXPECT_EQ(classification,
                static_cast<int>(GramNullspaceClass::Indeterminate));
    } else {
      ASSERT_TRUE(gram.valid) << gram.reason;
      const int expected_rank = smallest > 1e-10 ? 2 : 1;
      EXPECT_EQ(gram.rank, expected_rank);
      EXPECT_EQ(raw_rank, expected_rank);
    }
  }

  Eigen::Matrix2d first = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d second = Eigen::Matrix2d::Zero();
  first(0, 0) = 1.0;
  second(0, 0) = 2.0;
  const auto first_id = certifySymmetricPsd(first, 1e-10, 41);
  const auto second_id = certifySymmetricPsd(second, 1e-10, 41);
  ASSERT_TRUE(first_id.valid);
  ASSERT_TRUE(second_id.valid);
  EXPECT_NE(first_id.proof_identity, second_id.proof_identity);

  // A constructive factor proof accepts its own rounded Gram (including an
  // exact structural zero), but cannot be used to launder a different matrix.
  Eigen::MatrixXd factor = Eigen::MatrixXd::Zero(4, 2);
  factor(0, 0) = 1.0;
  factor(1, 1) = 1e-6;
  const Eigen::MatrixXd factor_gram = factor.transpose() * factor;
  const auto factor_certificate = uwb_imu_pl::certifyFactorGram(
      factor, factor_gram, 1e-10, 43);
  ASSERT_TRUE(factor_certificate.valid) << factor_certificate.reason;
  EXPECT_EQ(factor_certificate.rank, 2);
  Eigen::MatrixXd tampered_gram = factor_gram;
  tampered_gram(1, 1) = -1e-16;
  const auto tampered_factor_certificate = uwb_imu_pl::certifyFactorGram(
      factor, tampered_gram, 1e-10, 43);
  EXPECT_FALSE(tampered_factor_certificate.valid);
  EXPECT_NE(tampered_factor_certificate.reason.find("does not match"),
            std::string::npos);
  EXPECT_NE(factor_certificate.proof_identity,
            tampered_factor_certificate.proof_identity);

  // The direct raw-Z and squared Gram contracts agree away from the frozen
  // boundary for the reviewer probe diag(1, 1e-6), tol=1e-10.
  int direct_rank = 0;
  const int direct_class = uwb_imu_pl::classifyDetectionResponse(
      factor, Eigen::Matrix<double, 3, 2>::Zero(), 1e-10,
      nullptr, nullptr, &direct_rank);
  EXPECT_EQ(direct_rank, 2);
  EXPECT_EQ(direct_class, static_cast<int>(GramNullspaceClass::FullRank));
}

TEST(SquareRootContext, P003LegacyEightArgumentClassificationAbiLinks) {
  using LegacySignature = int (*)(
      const Eigen::MatrixXd&, const Eigen::MatrixXd&, double,
      double*, double*, int*, Eigen::Vector3d*, Eigen::Vector3d*);
  const LegacySignature legacy = static_cast<LegacySignature>(
      &uwb_imu_pl::classifyDetectionResponse);
  ASSERT_NE(legacy, nullptr);
  const Eigen::MatrixXd z = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::MatrixXd g = Eigen::MatrixXd::Zero(3, 2);
  int rank = 0;
  EXPECT_EQ(legacy(z, g, 1e-10, nullptr, nullptr, &rank, nullptr, nullptr),
            static_cast<int>(uwb_imu_pl::GramNullspaceClass::FullRank));
  EXPECT_EQ(rank, 2);
}

TEST(SquareRootContext, P003RootProofBindsInputsPolicyPathAndServedResult) {
  const int rows = 12, columns = 4;
  const Eigen::MatrixXd H = randomMatrix(rows, columns, 951);
  const Eigen::VectorXd z = randomMatrix(rows, 1, 952);
  const auto C = protectedMapFor(columns);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H,
                                        Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd state = svd.solve(z);
  const double statistic = (z - H * state).squaredNorm();
  const auto natural = buildContext(H, z, C, ColumnScalePolicy::Unit,
                                    ColumnPermutationPolicy::Natural,
                                    state, statistic);
  const auto pivoted = buildContext(H, z, C, ColumnScalePolicy::Unit,
                                    ColumnPermutationPolicy::ColumnPivot,
                                    state, statistic);
  ASSERT_TRUE(natural && natural->usable());
  ASSERT_TRUE(pivoted && pivoted->usable());
  EXPECT_NE(natural->proofIdentity(), pivoted->proofIdentity());
  Eigen::MatrixXd changed_h = H;
  changed_h(0, 0) = std::nextafter(changed_h(0, 0),
                                   std::numeric_limits<double>::infinity());
  Eigen::JacobiSVD<Eigen::MatrixXd> changed_svd(
      changed_h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd changed_state = changed_svd.solve(z);
  const auto changed = buildContext(
      changed_h, z, C, ColumnScalePolicy::Unit,
      ColumnPermutationPolicy::Natural, changed_state,
      (z - changed_h * changed_state).squaredNorm());
  ASSERT_TRUE(changed && changed->usable());
  EXPECT_NE(natural->proofIdentity(), changed->proofIdentity());
}

TEST(SquareRootContext, P003EveryRhsUsesCertifiedRootAndSvdReference) {
  const int rows = 9;
  const int columns = 4;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rows, columns);
  H.diagonal().head(columns) << 1.0, 1e-2, 1e-4, 1e-5;
  H.bottomRows(rows - columns) =
      1e-3 * randomMatrix(rows - columns, columns, 901);
  const Eigen::VectorXd z = randomMatrix(rows, 1, 902);
  const auto protected_map = protectedMapFor(columns);
  Eigen::JacobiSVD<Eigen::MatrixXd> reference(
      H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd state = reference.solve(z);
  const auto context = buildContext(
      H, z, protected_map, ColumnScalePolicy::Unit,
      ColumnPermutationPolicy::Natural, state,
      (z - H * state).squaredNorm());
  ASSERT_TRUE(context != nullptr);
  ASSERT_TRUE(context->usable()) << certificateSummary(context->certificate());

  FrozenWindowNumerics numerics;
  const Eigen::MatrixXd information = H.transpose() * H;
  numerics.information_factorization =
      std::make_shared<const Eigen::LLT<Eigen::MatrixXd>>(information);
  numerics.exact_condition = reference.singularValues()(0) /
      reference.singularValues()(columns - 1);
  numerics.spectral_vectors =
      std::make_shared<const Eigen::MatrixXd>(reference.matrixV());
  numerics.spectral_inverse_squared =
      reference.singularValues().array().square().inverse();

  // One batch contains the nominal-state normal RHS, all protected covariance
  // columns, two fault RHS and two bridge RHS.  The independent oracle is the
  // direct raw-Jacobian SVD, not the normal-equation LLT under test.
  Eigen::MatrixXd rhs(columns, 8);
  rhs.col(0) = H.transpose() * z;
  rhs.middleCols<3>(1) = protected_map.transpose();
  rhs.middleCols(4, 2) = randomMatrix(columns, 2, 903);
  rhs.rightCols(2) = randomMatrix(columns, 2, 904);
  const Eigen::MatrixXd actual = solveFrozenInformation(
      context.get(), numerics, information, rhs);
  ASSERT_EQ(actual.rows(), columns);
  ASSERT_EQ(actual.cols(), rhs.cols());
  const Eigen::MatrixXd expected = reference.matrixV() *
      numerics.spectral_inverse_squared.asDiagonal() *
      reference.matrixV().transpose() * rhs;
  for (int column = 0; column < actual.cols(); ++column) {
    const double absolute = (actual.col(column) - expected.col(column)).norm();
    const double relative = absolute / std::max(1.0, expected.col(column).norm());
    EXPECT_LE(absolute, 1e-10 + 1e-9 * expected.col(column).norm());
    EXPECT_LE(relative, 1e-9);
  }
}

}  // namespace

namespace {
void expectSameBits(double a, double b) {
  EXPECT_EQ(std::memcmp(&a, &b, sizeof(double)), 0);
}
template <class A, class B>
void expectSameMatrixBits(const A& a, const B& b) {
  ASSERT_EQ(a.rows(), b.rows()); ASSERT_EQ(a.cols(), b.cols());
  for (Eigen::Index row = 0; row < a.rows(); ++row)
    for (Eigen::Index col = 0; col < a.cols(); ++col)
      expectSameBits(a(row, col), b(row, col));
}
void expectSameNumericalCertificate(const uwb_imu_pl::GramResponseCertificate& a,
                                    const uwb_imu_pl::GramResponseCertificate& b) {
  const auto& x = a.gram; const auto& y = b.gram;
  expectSameMatrixBits(x.symmetric_matrix, y.symmetric_matrix);
  expectSameMatrixBits(x.eigenvalues, y.eigenvalues);
  expectSameMatrixBits(x.eigenvectors, y.eigenvectors);
  expectSameMatrixBits(x.eigenvalue_errors, y.eigenvalue_errors);
#define CHECK_DOUBLE(field) expectSameBits(x.field, y.field)
  CHECK_DOUBLE(matrix_scale); CHECK_DOUBLE(symmetry_error);
  CHECK_DOUBLE(symmetry_tolerance); CHECK_DOUBLE(eigenvalue_error);
  CHECK_DOUBLE(rank_eigenvalue_gate); CHECK_DOUBLE(sigma_min);
  CHECK_DOUBLE(sigma_max); CHECK_DOUBLE(condition);
#undef CHECK_DOUBLE
  EXPECT_EQ(x.rank, y.rank); EXPECT_EQ(x.symmetric, y.symmetric);
  EXPECT_EQ(x.psd, y.psd); EXPECT_EQ(x.rank_certified, y.rank_certified);
  EXPECT_EQ(x.valid, y.valid); EXPECT_EQ(x.reason, y.reason);
  EXPECT_EQ(a.nullspace_class, b.nullspace_class);
  expectSameMatrixBits(a.axis_residual, b.axis_residual);
  expectSameMatrixBits(a.protected_slopes, b.protected_slopes);
  expectSameBits(a.response_tolerance, b.response_tolerance);
  EXPECT_EQ(a.valid, b.valid); EXPECT_EQ(a.reason, b.reason);
  EXPECT_EQ(b.proof_identity, 0u); EXPECT_EQ(b.gram.proof_identity, 0u);
}
struct ClassificationHashEnvironment {
  const char* key = "UWB_IMU_PL_EXHAUSTIVE_CLASSIFICATION_HASHES";
  bool existed = std::getenv(key) != nullptr;
  std::string original = existed ? std::getenv(key) : "";
  ~ClassificationHashEnvironment() {
    if (existed) setenv(key, original.c_str(), 1); else unsetenv(key);
  }
};
}

TEST(Phase2ClassificationHashes, CompleteNumericalCertificateIncludingFailureReasons) {
  using namespace uwb_imu_pl;
  const auto compare = [](const Eigen::MatrixXd& z, const Eigen::MatrixXd& gram,
                          const Eigen::MatrixXd& g, double tol, double scale) {
    expectSameNumericalCertificate(
        certifyFactorGramAndProtectedResponse(z, gram, g, tol, 0, scale),
        detail::classificationNumericsWithoutProofIdentity(z, gram, g, tol, scale));
  };
  for (int cols = 1; cols <= 4; ++cols) for (int rows = 1; rows <= 8; ++rows) {
    Eigen::MatrixXd z = randomMatrix(rows, cols, rows * 11 + cols);
    Eigen::MatrixXd g = randomMatrix(3, cols, 19 + cols);
    for (int kind = 0; kind < 4; ++kind) {
      if (kind == 1) z.setZero();
      if (kind == 2) { z.setZero(); z(0,0)=1.; }
      if (kind == 3) g.setZero();
      const Eigen::MatrixXd gram=z.transpose()*z;
      for (double tol : {1e-12, 1., std::nextafter(1., 0.), 0., -1.})
        for (double scale : {1., 0., -1., std::numeric_limits<double>::infinity()})
          compare(z, gram, g, tol, scale);
      Eigen::MatrixXd bad=gram; bad(0,0)=std::nextafter(bad(0,0), 100.);
      compare(z,bad,g,1e-12,1.);
      if(cols>1) { bad=gram;bad(0,1)+=1.;compare(z,bad,g,1e-12,1.); }
      bad=gram;bad(0,0)=std::numeric_limits<double>::quiet_NaN();
      compare(z,bad,g,1e-12,1.);
    }
  }
  for(double scale : {1e-200, 1e-100, 1e100, 1e200}) {
    const Eigen::MatrixXd z=scale*Eigen::MatrixXd::Identity(3,3);
    compare(z,z.transpose()*z,z,1e-12,scale);
  }
  compare(Eigen::MatrixXd(),Eigen::MatrixXd(),Eigen::MatrixXd(),1e-12,1.);
  Eigen::MatrixXd invalid=Eigen::MatrixXd::Identity(3,3);
  invalid(0,0)=std::numeric_limits<double>::quiet_NaN();
  compare(invalid,Eigen::MatrixXd::Identity(3,3),Eigen::MatrixXd::Identity(3,3),1e-12,1.);
  compare(Eigen::MatrixXd::Identity(3,3),Eigen::MatrixXd::Identity(3,3),invalid,1e-12,1.);
  compare(Eigen::MatrixXd::Identity(3,3),Eigen::MatrixXd::Identity(2,2),
          Eigen::MatrixXd::Identity(3,3),1e-12,1.);
  compare(Eigen::MatrixXd::Identity(2,2),Eigen::MatrixXd::Identity(2,2),
          Eigen::MatrixXd::Zero(2,2),1e-12,1.);
}

TEST(Phase2ClassificationHashes, ClassifierAllOutputsAndNullPointersAreBitIdentical) {
  using namespace uwb_imu_pl;
  ClassificationHashEnvironment env;
  const auto compare = [&](const Eigen::MatrixXd& z, const Eigen::MatrixXd& g,
                           double tol, double scale) {
    double sigma[2]={123.,123.},condition[2]={456.,456.};int rank[2]={789,789};
    Eigen::Vector3d axis[2],slopes[2];int classes[2];
    for(int side=0;side<2;++side) {
      if(side==0)setenv(env.key,"1",1);else unsetenv(env.key);
      classes[side]=classifyDetectionResponse(z,g,tol,&sigma[side],&condition[side],
          &rank[side],&axis[side],&slopes[side],scale);
      EXPECT_EQ(classes[side],classifyDetectionResponse(z,g,tol,nullptr,nullptr,
          nullptr,nullptr,nullptr,scale));
    }
    EXPECT_EQ(classes[0],classes[1]);EXPECT_EQ(rank[0],rank[1]);
    expectSameBits(sigma[0],sigma[1]);expectSameBits(condition[0],condition[1]);
    expectSameMatrixBits(axis[0],axis[1]);expectSameMatrixBits(slopes[0],slopes[1]);
    if(classes[0]==0) { EXPECT_EQ(sigma[1],123.);EXPECT_EQ(condition[1],456.);
      EXPECT_EQ(rank[1],789);EXPECT_TRUE(axis[1].isZero(0.)); }
  };
  for(int cols=1;cols<=4;++cols)for(int rows=1;rows<=8;++rows) {
    Eigen::MatrixXd z=randomMatrix(rows,cols,42+rows+cols);
    Eigen::MatrixXd g=randomMatrix(3,cols,73+cols);
    for(int kind=0;kind<4;++kind) {
      if(kind==1) z.setZero();
      if(kind==2) { z.setZero(); z(0,0)=1.; }
      if(kind==3) g.setZero();
      for(double tol : {1e-12,1.,std::nextafter(1.,0.),std::nextafter(1.,2.),0.,-1.,
                         std::numeric_limits<double>::quiet_NaN()})
        for(double scale : {1.,0.,-1.,std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::quiet_NaN()})
          compare(z,g,tol,scale);
    }
  }
  compare(Eigen::MatrixXd(),Eigen::MatrixXd(),1e-12,1.);
  Eigen::MatrixXd z=Eigen::MatrixXd::Identity(3,3),g=z;
  z(0,0)=std::numeric_limits<double>::quiet_NaN();compare(z,g,1e-12,1.);
  z.setIdentity();g(0,0)=std::numeric_limits<double>::infinity();compare(z,g,1e-12,1.);
  for(double scale : {1e-200,1e-100,1e100,1e200})compare(scale*z,z,1e-12,scale);
}

TEST(Phase2ClassificationHashes, PublicProofIdentitiesKeepPreChangeByteOrder) {
  using namespace uwb_imu_pl;
  ClassificationHashEnvironment env;
  // Captured from 902ff99's DSO using the target's -march=native/NDEBUG ABI.
  for (int side=0; side<2; ++side) {
    if(side==0)setenv(env.key,"1",1);else unsetenv(env.key);
    Eigen::MatrixXd z=Eigen::MatrixXd::Identity(3,3),g=2.*z;
    auto c=certifyFactorGramAndProtectedResponse(z,z.transpose()*z,g,1e-12,123,1.);
    EXPECT_EQ(c.gram.proof_identity,5586683906216623275ULL);
    EXPECT_EQ(c.proof_identity,4623873127458493770ULL);
    z(2,2)=0.;c=certifyFactorGramAndProtectedResponse(z,z.transpose()*z,g,1e-12,123,1.);
    EXPECT_EQ(c.gram.proof_identity,2213805839774945581ULL);
    EXPECT_EQ(c.proof_identity,14122314800303469774ULL);
  }
}
