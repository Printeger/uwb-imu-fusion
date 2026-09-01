#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"

#include <gtest/gtest.h>

namespace {

uwb_imu_pl::UwbBatch makeBatch(uwb_imu_pl::TimestampNs timestamp,
                               const Eigen::Vector3d& position) {
  uwb_imu_pl::UwbBatch batch;
  batch.id = uwb_imu_pl::BatchId(timestamp.value());
  batch.timestamp = timestamp;
  batch.covariance_model_id = "test_diagonal";
  const std::vector<Eigen::Vector3d> anchors = {
      {-5,-5,0}, {5,-5,1}, {5,5,3}, {-5,5,4}, {0,-6,2}};
  for (std::size_t i = 0; i < anchors.size(); ++i) {
    uwb_imu_pl::UwbMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(i + 1);
    measurement.factor_id = uwb_imu_pl::FactorId(i + 1);
    measurement.anchor_id = uwb_imu_pl::AnchorId(i + 1);
    measurement.timestamp = timestamp;
    measurement.anchor_position_m = anchors[i];
    measurement.range_m = (position - anchors[i]).norm();
    measurement.sigma_m = 0.1;
    batch.measurements.push_back(measurement);
  }
  return batch;
}

uwb_imu_pl::IntegrityConfig config() {
  uwb_imu_pl::IntegrityConfig config;
  config.incremental.relinearize_threshold = 0.1;
  config.incremental.relinearize_skip = 1;
  config.incremental.smoothness_sigma_m = 0.5;
  config.incremental.method_b_max_condition = 1e12;
  config.snapshot.rank_tolerance = 1e-10;
  config.snapshot.max_condition_number = 1e12;
  config.imu.accelerometer_sigma = 0.1;
  config.imu.gyroscope_sigma = 0.01;
  config.imu.accelerometer_bias_rw_sigma = 0.001;
  config.imu.gyroscope_bias_rw_sigma = 0.0001;
  config.imu.gravity_mps2 = 9.80665;
  return config;
}

}  // namespace

TEST(RealtimeFactors, WorldPositionUsesPoseTangentRotation) {
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.2, -0.1, 0.4),
                          gtsam::Point3(1, 2, 3));
  const auto analytic = uwb_imu_pl::worldPositionPoseTangentJacobian(pose);
  const double epsilon = 1e-7;
  for (int column = 0; column < 6; ++column) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta(column) = epsilon;
    const Eigen::Vector3d numerical =
        (pose.retract(delta).translation() - pose.retract(-delta).translation()) /
        (2.0 * epsilon);
    EXPECT_TRUE(analytic.col(column).isApprox(numerical, 1e-7));
  }
}

TEST(UwbIncremental, RegularizerRowsAreExcludedByRole) {
  uwb_imu_pl::IncrementalUwbEstimator estimator(config().incremental);
  estimator.initialize(uwb_imu_pl::TimestampNs(0), Eigen::Vector3d(0,0,1),
                       Eigen::Vector3d::Zero());
  estimator.update(makeBatch(uwb_imu_pl::TimestampNs(50000000),
                             Eigen::Vector3d(0,0,1)));
  const auto snapshot = estimator.snapshot();
  ASSERT_EQ(snapshot->rowBlocks().size(), 2u);
  EXPECT_EQ(snapshot->rowBlocks()[0].role, uwb_imu_pl::RowRole::Regularizer);
  EXPECT_EQ(snapshot->rowBlocks()[1].role, uwb_imu_pl::RowRole::Measurement);
  EXPECT_EQ(snapshot->diagnostics().rows, 5);
}

TEST(UwbImuIncremental, MethodAPriorExcludesCurrentBatch) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = Eigen::Vector3d(0,0,1);
  Eigen::Matrix<double, 15, 1> sigmas =
      Eigen::Matrix<double, 15, 1>::Constant(0.1);
  estimator.initialize(initial, sigmas);
  for (int i = 1; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement imu;
    imu.id = uwb_imu_pl::MeasurementId(i);
    imu.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    imu.specific_force_mps2 = Eigen::Vector3d(0, 0, cfg.imu.gravity_mps2);
    estimator.ingestImu(imu);
  }
  const auto timestamp = uwb_imu_pl::TimestampNs(10000000);
  estimator.predictTo(timestamp);
  const auto snapshot = estimator.preMeasurementSnapshot(
      makeBatch(timestamp, Eigen::Vector3d(0,0,1)));
  ASSERT_TRUE(snapshot->currentPrior().has_value());
  EXPECT_TRUE(snapshot->currentPrior()->excludes_current_uwb);
  EXPECT_TRUE(snapshot->capabilities().pre_measurement_prior);
  EXPECT_FALSE(snapshot->capabilities().fixed_lag);
  EXPECT_FALSE(snapshot->capabilities().historical_fault_provenance);
}
