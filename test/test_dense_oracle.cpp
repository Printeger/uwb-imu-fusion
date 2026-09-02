#include "uwb_imu_pl/integrity/dense_oracle.hpp"

#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <random>

TEST(DenseOracle, MatchesIndependentNormalEquationForRandomFullRankCases) {
  std::mt19937_64 random(20260901);
  std::normal_distribution<double> normal;
  for (int trial = 0; trial < 1000; ++trial) {
    Eigen::MatrixXd h(8, 3);
    Eigen::VectorXd residual(8);
    for (Eigen::Index row = 0; row < h.rows(); ++row) {
      residual(row) = normal(random);
      for (Eigen::Index column = 0; column < h.cols(); ++column) {
        h(row, column) = normal(random);
      }
    }
    const Eigen::MatrixXd covariance =
        Eigen::VectorXd::LinSpaced(8, 0.2, 1.0).array().square().matrix().asDiagonal();
    Eigen::VectorXd incidence = Eigen::VectorXd::Zero(8);
    incidence(trial % 8) = 1.0;
    const auto oracle = uwb_imu_pl::evaluateDenseSvdOracle(
        h, residual, covariance, incidence, Eigen::Matrix3d::Identity());
    ASSERT_TRUE(oracle.model_valid) << oracle.reason;
    const Eigen::MatrixXd information =
        h.transpose() * covariance.inverse() * h;
    const Eigen::MatrixXd expected_covariance = information.inverse();
    EXPECT_LT((oracle.covariance - expected_covariance).norm() /
                  expected_covariance.norm(),
              1e-8);
  }
}

TEST(DenseOracle, RankDeficiencyAndUnmonitorableFaultRemainInfinite) {
  Eigen::MatrixXd h(5, 3);
  h << 1, 0, 0,
       0, 1, 0,
       1, 1, 0,
       2, 1, 0,
       1, 2, 0;
  const auto rank_deficient = uwb_imu_pl::evaluateDenseSvdOracle(
      h, Eigen::VectorXd::Zero(5), Eigen::MatrixXd::Identity(5, 5),
      Eigen::VectorXd::Ones(5), Eigen::Matrix3d::Identity());
  EXPECT_FALSE(rank_deficient.model_valid);
  EXPECT_FALSE(rank_deficient.protected_slope.allFinite());

  Eigen::MatrixXd square = Eigen::MatrixXd::Identity(3, 3);
  const auto no_redundancy = uwb_imu_pl::evaluateDenseSvdOracle(
      square, Eigen::VectorXd::Zero(3), Eigen::MatrixXd::Identity(3, 3),
      Eigen::VectorXd::Unit(3, 0), Eigen::Matrix3d::Identity());
  EXPECT_TRUE(no_redundancy.model_valid);
  EXPECT_FALSE(no_redundancy.fault_monitorable);
  EXPECT_FALSE(no_redundancy.protected_slope.allFinite());
}
