#include <gtest/gtest.h>

#include <boost/math/distributions/chi_squared.hpp>

#include <Eigen/Cholesky>

#include "uifgo/pl_conditional_raim.h"

namespace {

uifgo::PlConditionalInput Input(size_t rows) {
  uifgo::PlConditionalInput input;
  input.prior_covariance.setIdentity();
  input.prior_covariance *= 0.01;
  input.physical_jacobian = Eigen::MatrixXd::Zero(rows, 15);
  input.physical_covariance = Eigen::MatrixXd::Identity(rows, rows);
  input.physical_innovation = Eigen::VectorXd::Zero(rows);
  for (size_t i = 0; i < rows; ++i)
    input.physical_jacobian(static_cast<Eigen::Index>(i),
                            static_cast<Eigen::Index>(i)) = 1.0;
  return input;
}

void Near(double expected, double actual) {
  const double scale = std::max({1.0, std::abs(expected), std::abs(actual)});
  EXPECT_NEAR(expected, actual, 1e-12 + 1e-10 * scale);
}

TEST(PlConditionalCore, MatchesIndependentLockedPlEquations) {
  auto input = Input(3);
  input.physical_covariance << 1.0, 0.2, 0.0,
                               0.2, 2.0, 0.1,
                               0.0, 0.1, 1.5;
  input.physical_innovation << 0.2, -0.3, 0.1;
  const auto actual = uifgo::EvaluatePlConditional(input);

  Eigen::LLT<Eigen::MatrixXd> llt(input.physical_covariance);
  const Eigen::MatrixXd w = llt.matrixL().solve(Eigen::MatrixXd::Identity(3,3));
  const Eigen::VectorXd nu_w = w * input.physical_innovation;
  const Eigen::MatrixXd h_w = w * input.physical_jacobian;
  const Eigen::MatrixXd s_w = h_w * input.prior_covariance * h_w.transpose() +
                              Eigen::MatrixXd::Identity(3,3);
  const double statistic = nu_w.dot(s_w.ldlt().solve(nu_w));
  const double threshold = boost::math::quantile(
      boost::math::chi_squared(3), 1.0 - 1e-5);
  ASSERT_TRUE(actual.numerically_valid);
  EXPECT_EQ(actual.dof, 3);
  EXPECT_EQ(actual.passed, statistic <= threshold);
  Near(statistic, actual.statistic);
  Near(threshold, actual.threshold);
  EXPECT_TRUE(actual.whitener.isApprox(w, 1e-12));
  EXPECT_TRUE(actual.whitened_innovation.isApprox(nu_w, 1e-12));
  EXPECT_TRUE(actual.whitened_jacobian.isApprox(h_w, 1e-12));
  EXPECT_TRUE(actual.innovation_covariance_whitened.isApprox(s_w, 1e-12));
  EXPECT_TRUE(actual.innovation_covariance_physical.isApprox(
      input.physical_jacobian * input.prior_covariance *
          input.physical_jacobian.transpose() + input.physical_covariance,
      1e-12));
}

TEST(PlConditionalCore, PassUsesInclusiveThreshold) {
  auto input = Input(1);
  const double threshold = boost::math::quantile(
      boost::math::chi_squared(1), 1.0 - 1e-5);
  input.physical_innovation[0] = std::sqrt(threshold * 1.01);
  const auto probe = uifgo::EvaluatePlConditional(input);
  input.physical_innovation[0] *= std::sqrt(threshold / probe.statistic);
  const auto equal = uifgo::EvaluatePlConditional(input);
  Near(equal.threshold, equal.statistic);
  EXPECT_TRUE(equal.passed);
}

TEST(PlConditionalCore, RejectsInvalidCovariance) {
  auto input = Input(2);
  input.physical_covariance(0,0) = -1.0;
  EXPECT_FALSE(uifgo::EvaluatePlConditional(input).numerically_valid);
  input = Input(2);
  input.prior_covariance(0,0) = 0.0;
  EXPECT_FALSE(uifgo::EvaluatePlConditional(input).numerically_valid);
}

TEST(PlConditionalRows, DiagonalCovarianceKeepsMarginalValues) {
  Eigen::Vector2d nu(2.0, -3.0);
  Eigen::Matrix2d s = Eigen::Matrix2d::Zero();
  s(0,0) = 4.0;
  s(1,1) = 9.0;
  const auto result = uifgo::EvaluatePlConditionalRows(nu, s);
  ASSERT_TRUE(result.numerically_valid);
  ASSERT_EQ(result.rows.size(), 2u);
  Near(2.0, result.rows[0].conditional_innovation);
  Near(4.0, result.rows[0].conditional_variance);
  Near(1.0, result.rows[0].conditional_z);
  Near(-3.0, result.rows[1].conditional_innovation);
  Near(9.0, result.rows[1].conditional_variance);
  Near(-1.0, result.rows[1].conditional_z);
}

TEST(PlConditionalRows, CorrelatedSpdMatchesAnalyticTwoAnchorFormula) {
  Eigen::Vector2d nu(2.0, 3.0);
  Eigen::Matrix2d s;
  s << 4.0, 1.0, 1.0, 9.0;
  const auto result = uifgo::EvaluatePlConditionalRows(nu, s);
  ASSERT_TRUE(result.numerically_valid);
  Near(2.0 - 3.0 / 9.0, result.rows[0].conditional_innovation);
  Near(4.0 - 1.0 / 9.0, result.rows[0].conditional_variance);
  Near(3.0 - 2.0 / 4.0, result.rows[1].conditional_innovation);
  Near(9.0 - 1.0 / 4.0, result.rows[1].conditional_variance);
}

TEST(PlConditionalRows, NearSingularButValidCovarianceRemainsFinite) {
  constexpr double rho = 0.999999;
  Eigen::Vector2d nu(1.0, 1.0);
  Eigen::Matrix2d s;
  s << 1.0, rho, rho, 1.0;
  const auto result = uifgo::EvaluatePlConditionalRows(nu, s);
  ASSERT_TRUE(result.numerically_valid);
  for (const auto& row : result.rows) {
    Near(1.0 - rho, row.conditional_innovation);
    Near(1.0 - rho * rho, row.conditional_variance);
    EXPECT_TRUE(std::isfinite(row.conditional_z));
  }
}

TEST(PlConditionalRows, TwoAnchorQuadraticDecompositionIsConsistent) {
  Eigen::Vector2d nu(2.0, 3.0);
  Eigen::Matrix2d s;
  s << 4.0, 1.0, 1.0, 9.0;
  const auto result = uifgo::EvaluatePlConditionalRows(nu, s);
  ASSERT_TRUE(result.numerically_valid);
  const double additive_sum = result.rows[0].additive_quadratic_contribution +
                              result.rows[1].additive_quadratic_contribution;
  Near(result.statistic, additive_sum);
  const double without_zero = nu[1] * nu[1] / s(1,1);
  Near(result.statistic - without_zero,
       result.rows[0].conditional_quadratic_increment);
}

TEST(PlConditionalRows, FiveAnchorDiagonalAnalyticCase) {
  Eigen::VectorXd nu(5);
  nu << 1.0, -2.0, 3.0, -4.0, 5.0;
  Eigen::MatrixXd s = Eigen::MatrixXd::Zero(5,5);
  s.diagonal() << 1.0, 4.0, 9.0, 16.0, 25.0;
  const auto result = uifgo::EvaluatePlConditionalRows(nu, s);
  ASSERT_TRUE(result.numerically_valid);
  ASSERT_EQ(result.rows.size(), 5u);
  for (size_t i = 0; i < result.rows.size(); ++i) {
    Near(nu[static_cast<Eigen::Index>(i)],
         result.rows[i].conditional_innovation);
    Near(s(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)),
         result.rows[i].conditional_variance);
    Near(i % 2 == 0 ? 1.0 : -1.0, result.rows[i].conditional_z);
  }
  Near(5.0, result.statistic);
}

TEST(PlConditionalIsolation, PositiveAndReverseFaultUseSignOnlyAfterUnique) {
  auto positive = Input(3);
  positive.physical_innovation[0] = 10.0;
  const auto p = uifgo::EvaluatePlConditionalLoao(
      positive, {10,20,30}, {100,200,300});
  EXPECT_EQ(p.outcome,
            uifgo::PlConditionalGroupOutcome::UNIQUE_ISOLATION_POSITIVE);
  EXPECT_EQ(p.isolated_anchor_id, 10);
  EXPECT_EQ(p.isolated_obs_id, 100u);
  EXPECT_TRUE(p.positive_excess_candidate);
  EXPECT_EQ(p.committed_rows, (std::vector<size_t>{1,2}));

  positive.physical_innovation[0] = -10.0;
  const auto n = uifgo::EvaluatePlConditionalLoao(
      positive, {10,20,30}, {100,200,300});
  EXPECT_EQ(n.outcome,
            uifgo::PlConditionalGroupOutcome::UNIQUE_ISOLATION_NONPOSITIVE);
  EXPECT_FALSE(n.positive_excess_candidate);
  EXPECT_EQ(n.committed_rows, (std::vector<size_t>{1,2}));
}

TEST(PlConditionalIsolation, AmbiguousAndUnisolatedFailClosed) {
  auto ambiguous = Input(3);
  ambiguous.physical_jacobian.setZero();
  ambiguous.physical_innovation.setConstant(std::sqrt(10.0));
  const auto a = uifgo::EvaluatePlConditionalLoao(
      ambiguous, {10,20,30}, {100,200,300});
  EXPECT_EQ(a.outcome,
            uifgo::PlConditionalGroupOutcome::FDE_ISOLATION_AMBIGUOUS);
  EXPECT_TRUE(a.prior_degradation_event);
  EXPECT_TRUE(a.committed_rows.empty());

  auto unisolated = Input(3);
  unisolated.physical_innovation << 10.0, 10.0, 0.0;
  const auto u = uifgo::EvaluatePlConditionalLoao(
      unisolated, {10,20,30}, {100,200,300});
  EXPECT_EQ(u.outcome,
            uifgo::PlConditionalGroupOutcome::FDE_UNISOLATED_FAULT);
  EXPECT_TRUE(u.prior_degradation_event);
  EXPECT_TRUE(u.committed_rows.empty());
}

TEST(PlConditionalIsolation, NominalCommitsWholeGroupAndDuplicateAnchorCloses) {
  auto input = Input(3);
  const auto pass = uifgo::EvaluatePlConditionalLoao(
      input, {10,20,30}, {100,200,300});
  EXPECT_EQ(pass.outcome, uifgo::PlConditionalGroupOutcome::GROUP_PASS);
  EXPECT_EQ(pass.committed_rows, (std::vector<size_t>{0,1,2}));
  const auto invalid = uifgo::EvaluatePlConditionalLoao(
      input, {10,10,30}, {100,200,300});
  EXPECT_EQ(invalid.outcome,
            uifgo::PlConditionalGroupOutcome::NUMERICAL_FAILURE);
  EXPECT_TRUE(invalid.prior_degradation_event);

  input.prior_covariance(0,0) = 0.0;
  const auto numerical = uifgo::EvaluatePlConditionalLoao(
      input, {10,20,30}, {100,200,300});
  EXPECT_EQ(numerical.outcome,
            uifgo::PlConditionalGroupOutcome::NUMERICAL_FAILURE);
  EXPECT_TRUE(numerical.prior_degradation_event);
  EXPECT_TRUE(numerical.committed_rows.empty());
}

TEST(PlConditionalSupport, NonCandidateInterruptsPersistentRun) {
  std::vector<uifgo::PlConditionalCandidateRecord> rows;
  for (size_t i = 0; i < 6; ++i) {
    uifgo::PlConditionalCandidateRecord row;
    row.obs_id = 100 + i;
    row.tag_id = 1;
    row.anchor_id = 2;
    row.keyframe_id = i;
    row.sensor_time = 0.01 * i;
    row.planned = true;
    row.candidate = i != 2;
    row.reason = row.candidate ? "UNIQUE_POSITIVE" : "GROUP_PASS";
    rows.push_back(row);
  }
  uifgo::PlConditionalSupportContext context;
  context.input_plan_hash = "plan";
  context.source_hash = "source";
  context.config_hash = "config";
  context.calibration_hash = "calibration";
  context.solver_config_hash = "solver";
  context.detector_identity_hash = "detector";
  const auto support = uifgo::BuildPlConditionalSupport(
      &rows, 1.0, 2, 0.01, context);
  ASSERT_EQ(support.segments.size(), 2u);
  EXPECT_EQ(support.segments[0].obs_ids,
            (std::vector<std::uint64_t>{100,101}));
  EXPECT_EQ(support.segments[1].obs_ids,
            (std::vector<std::uint64_t>{103,104,105}));
  EXPECT_EQ(support.partition_rule_version,
            uifgo::kPlConditionalPartitionRule);
}

TEST(PlConditionalIdentity, BindsLockedLifecycleParameters) {
  uifgo::PlConditionalSupportContext context;
  context.input_plan_hash = "plan";
  context.source_hash = "source";
  context.config_hash = "config";
  context.calibration_hash = "calibration";
  context.solver_config_hash = "solver";
  const auto a = uifgo::PlConditionalDetectorIdentity(
      context, 4, 200, 0.02, 0.1, 200);
  const auto b = uifgo::PlConditionalDetectorIdentity(
      context, 5, 200, 0.02, 0.1, 200);
  const auto c = uifgo::PlConditionalDetectorIdentity(
      context, 4, 200, 0.03, 0.1, 200);
  EXPECT_NE(a, b);
  EXPECT_NE(a, c);
  EXPECT_EQ(a.substr(0,7), "sha256:");
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
