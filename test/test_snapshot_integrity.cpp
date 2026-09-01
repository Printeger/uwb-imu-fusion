#include "uwb_imu_pl/estimation/snapshot_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <gtest/gtest.h>

#include <boost/math/distributions/non_central_chi_squared.hpp>

#include <set>

namespace {

uwb_imu_pl::UwbBatch makeBatch(const Eigen::Vector3d& truth) {
  const std::vector<Eigen::Vector3d> anchors = {
      {-5,-5,0}, {5,-5,0.5}, {5,5,2.5}, {-5,5,3}, {0,-6,4}, {0,6,1}};
  uwb_imu_pl::UwbBatch batch;
  batch.id = uwb_imu_pl::BatchId(1);
  batch.timestamp = uwb_imu_pl::TimestampNs(1000000000);
  batch.covariance_model_id = "test";
  for (std::size_t i = 0; i < anchors.size(); ++i) {
    uwb_imu_pl::UwbMeasurement m;
    m.id = uwb_imu_pl::MeasurementId(i + 1);
    m.factor_id = uwb_imu_pl::FactorId(i + 1);
    m.anchor_id = uwb_imu_pl::AnchorId(i + 10);
    m.timestamp = batch.timestamp;
    m.anchor_position_m = anchors[i];
    m.range_m = (truth - anchors[i]).norm();
    m.sigma_m = 0.1 + 0.01 * i;
    batch.measurements.push_back(m);
  }
  return batch;
}

uwb_imu_pl::RiskBudget risk() {
  uwb_imu_pl::RiskBudget risk;
  risk.p_fa = 1e-5;
  risk.p_hmi_total = 4e-5;
  risk.nominal_axis_tail = 1e-5;
  risk.p_nm = 1e-7;
  risk.horizontal_alert_limit_m = 10.0;
  risk.vertical_alert_limit_m = 10.0;
  for (std::uint64_t id = 10; id < 16; ++id) {
    uwb_imu_pl::FaultHypothesis allocation;
    allocation.id = uwb_imu_pl::HypothesisId(id);
    allocation.anchor_id = uwb_imu_pl::AnchorId(id);
    allocation.missed_detection_allocation = 1e-3;
    allocation.prior_probability_bound = 1e-4;
    risk.hypotheses.push_back(allocation);
  }
  return risk;
}

}  // namespace

TEST(SnapshotIntegrity, NoiselessHeteroscedasticGeometryConverges) {
  const Eigen::Vector3d truth(0.4, -0.2, 1.1);
  const auto batch = makeBatch(truth);
  uwb_imu_pl::SnapshotUwbEstimator estimator({3, 30, 1e-9, 1e-10, 1e10});
  const auto solution = estimator.estimate(batch, Eigen::Vector3d(1, 1, 2));
  EXPECT_TRUE(solution.converged);
  EXPECT_TRUE(solution.diagnostics.model_valid);
  EXPECT_NEAR((solution.position_world_m - truth).norm(), 0.0, 1e-7);
  EXPECT_NEAR(solution.residual_whitened.norm(), 0.0, 1e-7);
  EXPECT_TRUE((solution.whitener * solution.covariance_measurement_m2 *
               solution.whitener.transpose()).isApprox(
                  Eigen::MatrixXd::Identity(6, 6), 1e-10));
  EXPECT_TRUE(solution.residual_projector.isApprox(
      solution.residual_projector.transpose(), 1e-12));
  EXPECT_TRUE((solution.residual_projector * solution.residual_projector)
                  .isApprox(solution.residual_projector, 1e-12));
  EXPECT_NEAR(solution.residual_projector.trace(), 3.0, 1e-10);
}

TEST(SnapshotIntegrity, FullCovarianceAndAnalyticJacobianAreConsistent) {
  const Eigen::Vector3d truth(0.4, -0.2, 1.1);
  auto batch = makeBatch(truth);
  batch.covariance_m2 = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  batch.covariance_m2.array() += 0.001;
  uwb_imu_pl::SnapshotUwbEstimator estimator({3, 30, 1e-9, 1e-10, 1e10});
  const auto solution = estimator.estimate(batch, truth + Eigen::Vector3d(0.1, 0.1, -0.1));
  const double epsilon = 1e-6;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d plus = solution.position_world_m;
    Eigen::Vector3d minus = plus;
    plus(axis) += epsilon;
    minus(axis) -= epsilon;
    for (std::size_t row = 0; row < batch.measurements.size(); ++row) {
      const auto& m = batch.measurements[row];
      const double finite_difference =
          ((plus - m.anchor_position_m).norm() -
           (minus - m.anchor_position_m).norm()) / (2.0 * epsilon);
      EXPECT_NEAR(solution.jacobian_raw(static_cast<Eigen::Index>(row), axis),
                  finite_difference, 1e-6);
    }
  }
}

TEST(SnapshotIntegrity, ChiSquareDofAndPhysicalAnchorHypotheses) {
  const Eigen::Vector3d truth(0.4, -0.2, 1.1);
  const auto batch = makeBatch(truth);
  uwb_imu_pl::SnapshotUwbEstimator estimator({3, 30, 1e-9, 1e-10, 1e10});
  const auto solution = estimator.estimate(batch, truth);
  uwb_imu_pl::IntegrityMonitor monitor(risk(), 1e-10, 1e10);
  const auto output = monitor.evaluateSnapshot(batch, solution);
  EXPECT_EQ(output.detector.dof, 3);
  EXPECT_TRUE(output.detector.passed);
  EXPECT_EQ(output.sensitivities.size(), batch.measurements.size());
  std::set<std::uint64_t> anchors;
  for (const auto& sensitivity : output.sensitivities) {
    EXPECT_TRUE(sensitivity.finite);
    const std::size_t index = sensitivity.anchor_id.value() - 10;
    const Eigen::VectorXd incidence = Eigen::VectorXd::Unit(6, index);
    const Eigen::VectorXd whitened = solution.whitener * incidence;
    const Eigen::Vector3d mapped = solution.gain_whitened * whitened;
    const double gram = (whitened.transpose() *
        solution.residual_projector * whitened)(0, 0);
    EXPECT_TRUE(sensitivity.slope_xyz.isApprox(
        mapped.cwiseAbs() / std::sqrt(gram), 1e-12));
    anchors.insert(sensitivity.anchor_id.value());
  }
  EXPECT_EQ(anchors.size(), batch.measurements.size());
  EXPECT_TRUE(output.protection_level.pl_xyz_m.allFinite());
  EXPECT_TRUE(output.protection_level.formal_eligible);
  EXPECT_NEAR(output.protection_level.hpl_m,
              std::hypot(output.protection_level.pl_xyz_m.x(),
                         output.protection_level.pl_xyz_m.y()), 1e-12);
}

TEST(SnapshotIntegrity, RankDeficientGeometryIsUnavailable) {
  const Eigen::Vector3d truth(0, 0, 1);
  auto batch = makeBatch(truth);
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    batch.measurements[i].anchor_position_m = Eigen::Vector3d(i, 0, 0);
    batch.measurements[i].range_m =
        (truth - batch.measurements[i].anchor_position_m).norm();
  }
  uwb_imu_pl::SnapshotUwbEstimator estimator({3, 30, 1e-9, 1e-10, 1e10});
  const auto solution = estimator.estimate(batch, Eigen::Vector3d(0, 0, 0.8));
  uwb_imu_pl::IntegrityMonitor monitor(risk(), 1e-10, 1e10);
  const auto output = monitor.evaluateSnapshot(batch, solution);
  EXPECT_FALSE(solution.diagnostics.model_valid);
  EXPECT_EQ(output.protection_level.availability,
            uwb_imu_pl::Availability::Unavailable);
  EXPECT_FALSE(output.protection_level.pl_xyz_m.allFinite());
}

TEST(SnapshotIntegrity, NoncentralityBoundaryMeetsMissedDetectionAllocation) {
  const double lambda = uwb_imu_pl::IntegrityMonitor::noncentralityBoundary(
      3, 20.0, 1e-3);
  EXPECT_GT(lambda, 0.0);
  EXPECT_TRUE(std::isfinite(lambda));
  const double missed = boost::math::cdf(
      boost::math::non_central_chi_squared_distribution<double>(3, lambda),
      20.0);
  EXPECT_NEAR(missed, 1e-3, 1e-10);
}

TEST(SnapshotIntegrity, DiagonalAndExplicitCovarianceWhiteningAreEquivalent) {
  const Eigen::Vector3d truth(0.4, -0.2, 1.1);
  auto implicit = makeBatch(truth);
  auto explicit_covariance = implicit;
  explicit_covariance.covariance_m2 = Eigen::MatrixXd::Zero(6, 6);
  for (int i = 0; i < 6; ++i) {
    explicit_covariance.covariance_m2(i, i) =
        implicit.measurements[static_cast<std::size_t>(i)].sigma_m *
        implicit.measurements[static_cast<std::size_t>(i)].sigma_m;
  }
  uwb_imu_pl::SnapshotUwbEstimator estimator({3, 30, 1e-9, 1e-10, 1e10});
  const auto left = estimator.estimate(implicit, Eigen::Vector3d(1, 1, 2));
  const auto right = estimator.estimate(explicit_covariance,
                                        Eigen::Vector3d(1, 1, 2));
  EXPECT_TRUE(left.position_world_m.isApprox(right.position_world_m, 1e-12));
  EXPECT_TRUE(left.covariance_m2.isApprox(right.covariance_m2, 1e-12));
  EXPECT_TRUE(left.whitener.isApprox(right.whitener, 1e-12));
}

TEST(SnapshotIntegrity, FormalGateRiskAlarmAndAlertLimitBranchesFailClose) {
  const Eigen::Vector3d truth(0.4, -0.2, 1.1);
  auto batch = makeBatch(truth);
  uwb_imu_pl::SnapshotUwbEstimator estimator({3, 30, 1e-9, 1e-10, 1e10});
  const auto nominal = estimator.estimate(batch, truth);

  auto incomplete_risk = risk();
  incomplete_risk.hypotheses.pop_back();
  auto output = uwb_imu_pl::IntegrityMonitor(incomplete_risk, 1e-10, 1e10)
                    .evaluateSnapshot(batch, nominal);
  EXPECT_FALSE(output.measurement_model_valid);
  EXPECT_FALSE(output.protection_level.formal_eligible);
  EXPECT_EQ(output.protection_level.label,
            uwb_imu_pl::IntegrityLabel::ImplementedUnverified);
  EXPECT_FALSE(output.protection_level.pl_xyz_m.allFinite());

  auto over_budget = risk();
  over_budget.p_hmi_total = 1e-8;
  output = uwb_imu_pl::IntegrityMonitor(over_budget, 1e-10, 1e10)
               .evaluateSnapshot(batch, nominal);
  EXPECT_TRUE(output.measurement_model_valid);
  EXPECT_FALSE(output.protection_level.risk_budget_valid);
  EXPECT_EQ(output.protection_level.availability,
            uwb_imu_pl::Availability::Unavailable);

  auto low_limits = risk();
  low_limits.horizontal_alert_limit_m = 1e-9;
  low_limits.vertical_alert_limit_m = 1e-9;
  output = uwb_imu_pl::IntegrityMonitor(low_limits, 1e-10, 1e10)
               .evaluateSnapshot(batch, nominal);
  EXPECT_TRUE(output.protection_level.formal_eligible);
  EXPECT_TRUE(output.protection_level.pl_xyz_m.allFinite());
  EXPECT_EQ(output.protection_level.availability,
            uwb_imu_pl::Availability::Unavailable);

  batch.measurements.front().range_m += 20.0;
  auto faulted = nominal;
  faulted.residual_raw_m(0) += 20.0;
  faulted.residual_whitened = faulted.whitener * faulted.residual_raw_m;
  output = uwb_imu_pl::IntegrityMonitor(risk(), 1e-10, 1e10)
               .evaluateSnapshot(batch, faulted);
  ASSERT_TRUE(output.detector.numerically_valid);
  EXPECT_FALSE(output.detector.passed);
  EXPECT_TRUE(output.protection_level.formal_eligible);
  EXPECT_EQ(output.protection_level.label,
            uwb_imu_pl::IntegrityLabel::FormalLocalCurrentFaultOnly);
  EXPECT_EQ(output.protection_level.availability,
            uwb_imu_pl::Availability::Alert);
  EXPECT_FALSE(output.protection_level.pl_xyz_m.allFinite());
}
