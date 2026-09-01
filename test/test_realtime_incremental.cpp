#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <gtest/gtest.h>

#include <Eigen/LU>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>

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
  config.imu.max_gap_s = 0.02;
  config.risk.p_fa = 1e-5;
  config.risk.p_hmi_total = 4e-5;
  config.risk.nominal_axis_tail = 1e-5;
  config.risk.p_nm = 1e-7;
  config.risk.horizontal_alert_limit_m = 100.0;
  config.risk.vertical_alert_limit_m = 100.0;
  for (std::uint64_t id = 1; id <= 5; ++id) {
    uwb_imu_pl::FaultHypothesis allocation;
    allocation.id = uwb_imu_pl::HypothesisId(id);
    allocation.anchor_id = uwb_imu_pl::AnchorId(id);
    allocation.missed_detection_allocation = 1e-3;
    allocation.prior_probability_bound = 1e-4;
    config.risk.hypotheses.push_back(allocation);
  }
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

TEST(RealtimeFactors, NonzeroLeverArmRangeJacobianMatchesCentralDifference) {
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.2, -0.1, 0.4),
                          gtsam::Point3(1, 2, 3));
  const Eigen::Vector3d lever(0.3, -0.2, 0.1);
  auto batch = makeBatch(uwb_imu_pl::TimestampNs(1),
                         pose.transformFrom(lever));
  uwb_imu_pl::UwbPoseBatchFactor factor(gtsam::Symbol('x', 0), batch, lever);
  gtsam::Matrix analytic;
  factor.evaluateError(pose, analytic);
  const double epsilon = 1e-6;
  for (int column = 0; column < 6; ++column) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta(column) = epsilon;
    const Eigen::VectorXd numerical =
        (factor.evaluateError(pose.retract(delta)) -
         factor.evaluateError(pose.retract(-delta))) / (2.0 * epsilon);
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

TEST(UwbIncremental, TwentyEpochsMatchBatchPositionAndMarginal) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbEstimator incremental(cfg.incremental);
  const Eigen::Vector3d truth(0.2, -0.1, 1.0);
  incremental.initialize(uwb_imu_pl::TimestampNs(0), truth,
                         Eigen::Vector3d::Zero());

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  auto pkey = [](std::size_t epoch) { return gtsam::Symbol('p', epoch); };
  auto vkey = [](std::size_t epoch) { return gtsam::Symbol('v', epoch); };
  graph.addPrior(pkey(0), gtsam::Point3(truth),
                 gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  const gtsam::Vector3 zero_velocity = gtsam::Vector3::Zero();
  graph.addPrior(vkey(0), zero_velocity,
                 gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  values.insert(pkey(0), gtsam::Point3(truth));
  values.insert(vkey(0), zero_velocity);
  for (std::size_t epoch = 1; epoch <= 20; ++epoch) {
    const auto time = uwb_imu_pl::TimestampNs(
        static_cast<std::int64_t>(epoch) * 50000000LL);
    const auto batch = makeBatch(time, truth);
    incremental.update(batch);
    graph.add(boost::make_shared<uwb_imu_pl::ConstantVelocityRegularizer>(
        pkey(epoch - 1), vkey(epoch - 1), pkey(epoch), vkey(epoch), 0.05,
        cfg.incremental.smoothness_sigma_m));
    graph.add(boost::make_shared<uwb_imu_pl::UwbPositionBatchFactor>(
        pkey(epoch), batch));
    values.insert(pkey(epoch), gtsam::Point3(truth));
    values.insert(vkey(epoch), zero_velocity);
  }
  const gtsam::Values batch_values =
      gtsam::LevenbergMarquardtOptimizer(graph, values).optimize();
  const auto state = incremental.currentState();
  EXPECT_LT((state.position_world_m -
             batch_values.at<gtsam::Point3>(pkey(20))).norm(), 1e-6);
  gtsam::Marginals marginals(graph, batch_values);
  const gtsam::KeyVector keys{pkey(20), vkey(20)};
  const auto joint = marginals.jointMarginalCovariance(keys);
  Eigen::Matrix<double, 6, 6> expected;
  expected.block<3, 3>(0, 0) = joint.at(keys[0], keys[0]);
  expected.block<3, 3>(0, 3) = joint.at(keys[0], keys[1]);
  expected.block<3, 3>(3, 0) = joint.at(keys[1], keys[0]);
  expected.block<3, 3>(3, 3) = joint.at(keys[1], keys[1]);
  const Eigen::MatrixXd actual = incremental.snapshot()->currentMarginal();
  EXPECT_LT((actual - expected).norm() / expected.norm(), 1e-6);
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
  for (int i = 0; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement imu;
    imu.id = uwb_imu_pl::MeasurementId(i);
    imu.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    imu.specific_force_mps2 = Eigen::Vector3d(0, 0, cfg.imu.gravity_mps2);
    estimator.ingestImu(imu);
  }
  const auto timestamp = uwb_imu_pl::TimestampNs(10000000);
  estimator.predictTo(timestamp);
  const auto before_snapshot = estimator.audit();
  const auto snapshot = estimator.preMeasurementSnapshot(
      makeBatch(timestamp, Eigen::Vector3d(0,0,1)));
  const auto after_snapshot = estimator.audit();
  ASSERT_TRUE(snapshot->currentPrior().has_value());
  EXPECT_TRUE(snapshot->currentPrior()->excludes_current_uwb);
  EXPECT_TRUE(snapshot->capabilities().pre_measurement_prior);
  EXPECT_FALSE(snapshot->capabilities().fixed_lag);
  EXPECT_FALSE(snapshot->capabilities().historical_fault_provenance);
  EXPECT_EQ(before_snapshot.factor_count, after_snapshot.factor_count);
  EXPECT_TRUE(before_snapshot.version == after_snapshot.version);
  EXPECT_TRUE(after_snapshot.committed_uwb_batch_ids.empty());
  EXPECT_TRUE(snapshot->currentPrior()->version == after_snapshot.version);
}

TEST(UwbImuIncremental, ConditionalDetectorUsesInnovationDimension) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = Eigen::Vector3d(0,0,1);
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  for (int i = 0; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement imu;
    imu.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    imu.specific_force_mps2 = Eigen::Vector3d(0, 0, cfg.imu.gravity_mps2);
    estimator.ingestImu(imu);
  }
  const auto timestamp = uwb_imu_pl::TimestampNs(10000000);
  const auto batch = makeBatch(timestamp, Eigen::Vector3d(0,0,1));
  estimator.predictTo(timestamp);
  const auto snapshot = estimator.preMeasurementSnapshot(batch);
  uwb_imu_pl::IntegrityMonitor monitor(cfg.risk, cfg.snapshot.rank_tolerance,
                                       cfg.snapshot.max_condition_number);
  const auto output = monitor.evaluateConditional(batch, *snapshot);
  EXPECT_EQ(output.detector.dof, static_cast<int>(batch.measurements.size()));
  EXPECT_EQ(output.detector.detector_type,
            "conditional_current_uwb_innovation_chi_square");
  EXPECT_TRUE(std::isfinite(output.conditional_innovation_statistic));
  EXPECT_TRUE(std::isfinite(output.uwb_postfit_residual_statistic));
}

TEST(UwbImuIncremental, MethodBDowndateCandidateRecoversPrior) {
  auto cfg = config();
  cfg.incremental.method_b_max_condition = 1e6;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  const Eigen::Matrix<double, 15, 15> prior =
      Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(4, 15);
  h.block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
  const Eigen::Matrix4d r = 0.5 * Eigen::Matrix4d::Identity();
  const Eigen::Matrix<double, 15, 15> all_in =
      (prior.inverse() + h.transpose() * r.inverse() * h).inverse();
  const auto recovered = estimator.methodBCandidatePrior(all_in, h, r);
  ASSERT_TRUE(recovered.has_value());
  EXPECT_TRUE(recovered->isApprox(prior, 1e-10));
}

TEST(UwbImuIncremental, MethodAAndLinearMethodBCandidateGiveEquivalentOutput) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  for (int i = 0; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement sample;
    sample.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
    estimator.ingestImu(sample);
  }
  const auto time = uwb_imu_pl::TimestampNs(10000000);
  const auto batch = makeBatch(time, initial.position_world_m);
  estimator.predictTo(time);
  const auto method_a = estimator.preMeasurementSnapshot(batch);
  const auto prior_a = method_a->currentPrior().value();
  const Eigen::MatrixXd h = method_a->rowBlocks().front().jacobian;
  const Eigen::Matrix<double, 15, 15> all_in =
      (prior_a.covariance.inverse() + h.transpose() * h).inverse();
  const auto recovered = estimator.methodBCandidatePrior(
      all_in, h, Eigen::MatrixXd::Identity(h.rows(), h.rows()));
  ASSERT_TRUE(recovered.has_value());
  uwb_imu_pl::CurrentStatePrior prior_b = prior_a;
  prior_b.covariance = *recovered;
  const Eigen::Matrix<double, 15, 15> information_b = recovered->inverse();
  auto method_b = std::make_shared<uwb_imu_pl::ImmutableEstimationSnapshot>(
      method_a->state(), method_a->version(), method_a->diagnostics(),
      method_a->rowBlocks(), method_a->capabilities(), method_a->consistency(),
      prior_b, *recovered, information_b);
  uwb_imu_pl::IntegrityMonitor monitor(
      cfg.risk, cfg.snapshot.rank_tolerance,
      cfg.snapshot.max_condition_number);
  const auto output_a = monitor.evaluateConditional(batch, *method_a);
  const auto output_b = monitor.evaluateConditional(batch, *method_b);
  EXPECT_NEAR(output_a.detector.statistic, output_b.detector.statistic, 1e-10);
  EXPECT_TRUE(output_a.protection_level.pl_xyz_m.isApprox(
      output_b.protection_level.pl_xyz_m, 1e-10));
  EXPECT_EQ(output_a.protection_level.availability,
            output_b.protection_level.availability);
}

TEST(UwbImuIncremental, ConditionalFormalGateBranchesFailClosed) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  for (int i = 0; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement sample;
    sample.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
    estimator.ingestImu(sample);
  }
  const auto time = uwb_imu_pl::TimestampNs(10000000);
  const auto batch = makeBatch(time, initial.position_world_m);
  estimator.predictTo(time);
  const auto valid = estimator.preMeasurementSnapshot(batch);
  uwb_imu_pl::IntegrityMonitor monitor(
      cfg.risk, cfg.snapshot.rank_tolerance,
      cfg.snapshot.max_condition_number);
  auto expect_fail_closed = [&](uwb_imu_pl::SnapshotCapabilities capabilities,
                                uwb_imu_pl::LinearizationConsistency consistency,
                                uwb_imu_pl::CurrentStatePrior prior,
                                std::vector<uwb_imu_pl::WhitenedRowBlock> rows) {
    uwb_imu_pl::ImmutableEstimationSnapshot snapshot(
        valid->state(), valid->version(), valid->diagnostics(),
        std::move(rows), capabilities, consistency, prior, prior.covariance,
        Eigen::Matrix<double, 15, 15>::Identity());
    const auto output = monitor.evaluateConditional(batch, snapshot);
    EXPECT_FALSE(output.measurement_model_valid);
    EXPECT_FALSE(output.protection_level.formal_eligible);
    EXPECT_EQ(output.protection_level.label,
              uwb_imu_pl::IntegrityLabel::ImplementedUnverified);
    EXPECT_EQ(output.protection_level.availability,
              uwb_imu_pl::Availability::Unavailable);
    EXPECT_FALSE(output.protection_level.pl_xyz_m.allFinite());
  };

  auto capabilities = valid->capabilities();
  auto prior = valid->currentPrior().value();
  auto rows = valid->rowBlocks();
  capabilities.pre_measurement_prior = false;
  expect_fail_closed(capabilities, valid->consistency(), prior, rows);

  capabilities = valid->capabilities();
  prior.excludes_current_uwb = false;
  expect_fail_closed(capabilities, valid->consistency(), prior, rows);

  prior = valid->currentPrior().value();
  ++prior.version.graph_version;
  expect_fail_closed(capabilities, valid->consistency(), prior, rows);

  prior = valid->currentPrior().value();
  expect_fail_closed(capabilities,
                     uwb_imu_pl::LinearizationConsistency::CachedBlind,
                     prior, rows);

  prior.covariance(0, 0) = -1.0;
  expect_fail_closed(capabilities, valid->consistency(), prior, rows);

  prior = valid->currentPrior().value();
  rows.front().measurement_ids.clear();
  expect_fail_closed(capabilities, valid->consistency(), prior, rows);
}

TEST(UwbImuIncremental, FiveSecondIsamMatchesBatchPoseAndMarginal) {
  auto cfg = config();
  const Eigen::Vector3d truth(0.2, -0.1, 1.0);
  const Eigen::Matrix<double, 15, 1> sigmas =
      Eigen::Matrix<double, 15, 1>::Constant(0.1);
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = truth;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(
      cfg, Eigen::Vector3d::Zero());
  estimator.initialize(initial, sigmas);

  auto xkey = [](std::size_t epoch) { return gtsam::Symbol('x', epoch); };
  auto vkey = [](std::size_t epoch) { return gtsam::Symbol('v', epoch); };
  auto bkey = [](std::size_t epoch) { return gtsam::Symbol('b', epoch); };
  const gtsam::Pose3 pose{gtsam::Rot3(), gtsam::Point3(truth)};
  const gtsam::Vector3 velocity = gtsam::Vector3::Zero();
  const gtsam::imuBias::ConstantBias bias;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.addPrior(xkey(0), pose,
                 gtsam::noiseModel::Diagonal::Sigmas(sigmas.head<6>()));
  graph.addPrior(vkey(0), velocity,
                 gtsam::noiseModel::Diagonal::Sigmas(sigmas.segment<3>(6)));
  graph.addPrior(bkey(0), bias,
                 gtsam::noiseModel::Diagonal::Sigmas(sigmas.tail<6>()));
  values.insert(xkey(0), pose);
  values.insert(vkey(0), velocity);
  values.insert(bkey(0), bias);
  auto params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(
      cfg.imu.gravity_mps2);
  params->accelerometerCovariance = Eigen::Matrix3d::Identity() *
      std::pow(cfg.imu.accelerometer_sigma, 2);
  params->gyroscopeCovariance = Eigen::Matrix3d::Identity() *
      std::pow(cfg.imu.gyroscope_sigma, 2);
  params->integrationCovariance = Eigen::Matrix3d::Identity() * 1e-9;
  params->biasAccCovariance = Eigen::Matrix3d::Identity() *
      std::pow(cfg.imu.accelerometer_bias_rw_sigma, 2);
  params->biasOmegaCovariance = Eigen::Matrix3d::Identity() *
      std::pow(cfg.imu.gyroscope_bias_rw_sigma, 2);

  uwb_imu_pl::ImuMeasurement boundary;
  boundary.timestamp = initial.timestamp;
  boundary.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  constexpr int kEpochs = 50;
  for (int epoch = 1; epoch <= kEpochs; ++epoch) {
    gtsam::PreintegratedCombinedMeasurements pim(params, bias);
    for (int sample_index = (epoch - 1) * 10 + 1;
         sample_index <= epoch * 10; ++sample_index) {
      uwb_imu_pl::ImuMeasurement sample;
      sample.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>(sample_index) * 10000000LL);
      sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
      estimator.ingestImu(sample);
      pim.integrateMeasurement(sample.specific_force_mps2,
                               sample.angular_velocity_radps, 0.01);
    }
    const auto time = uwb_imu_pl::TimestampNs(
        static_cast<std::int64_t>(epoch) * 100000000LL);
    auto batch = makeBatch(time, truth);
    estimator.predictTo(time);
    estimator.preMeasurementSnapshot(batch);
    estimator.commitUwbBatch(batch);

    graph.add(gtsam::CombinedImuFactor(
        xkey(epoch - 1), vkey(epoch - 1), xkey(epoch), vkey(epoch),
        bkey(epoch - 1), bkey(epoch), pim));
    graph.add(boost::make_shared<uwb_imu_pl::UwbPoseBatchFactor>(
        xkey(epoch), batch, Eigen::Vector3d::Zero()));
    values.insert(xkey(epoch), pose);
    values.insert(vkey(epoch), velocity);
    values.insert(bkey(epoch), bias);
  }
  const gtsam::Values batch_values =
      gtsam::LevenbergMarquardtOptimizer(graph, values).optimize();
  const auto state = estimator.currentState();
  const gtsam::Pose3 isam_pose(
      gtsam::Rot3(state.q_world_body.toRotationMatrix()),
      gtsam::Point3(state.position_world_m));
  EXPECT_LT(gtsam::Pose3::Logmap(
                batch_values.at<gtsam::Pose3>(xkey(kEpochs)).between(isam_pose))
                .norm(),
            1e-5);
  gtsam::Marginals marginals(graph, batch_values);
  const gtsam::KeyVector keys{xkey(kEpochs), vkey(kEpochs), bkey(kEpochs)};
  const auto joint = marginals.jointMarginalCovariance(keys);
  Eigen::Matrix<double, 15, 15> expected =
      Eigen::Matrix<double, 15, 15>::Zero();
  const int offsets[] = {0, 6, 9};
  const int dimensions[] = {6, 3, 6};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      expected.block(offsets[row], offsets[column], dimensions[row],
                     dimensions[column]) = joint.at(keys[row], keys[column]);
    }
  }
  const auto actual = estimator.audit().current_marginal;
  EXPECT_LT((actual - expected).norm() / expected.norm(), 1e-5);
}

TEST(UwbImuIncremental, GapFailsWithoutChangingEstimatorAudit) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  uwb_imu_pl::ImuMeasurement boundary;
  boundary.timestamp = initial.timestamp;
  boundary.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  auto late = boundary;
  late.timestamp = uwb_imu_pl::TimestampNs(30000000);
  estimator.ingestImu(late);
  const auto before = estimator.audit();
  EXPECT_THROW(estimator.ingestImu(late), std::invalid_argument);
  EXPECT_THROW(estimator.predictTo(late.timestamp), std::runtime_error);
  const auto after = estimator.audit();
  EXPECT_EQ(before.epoch, after.epoch);
  EXPECT_EQ(before.state_timestamp, after.state_timestamp);
  EXPECT_EQ(before.factor_count, after.factor_count);
  EXPECT_TRUE(before.version == after.version);
  EXPECT_TRUE(before.current_marginal.isApprox(after.current_marginal, 0.0));
}

TEST(UwbImuIncremental, FutureImuDoesNotAffectCurrentPrediction) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator causal(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::IncrementalUwbImuEstimator with_future(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  const auto sigmas = Eigen::Matrix<double, 15, 1>::Constant(0.1);
  causal.initialize(initial, sigmas);
  with_future.initialize(initial, sigmas);
  for (int i = 0; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement sample;
    sample.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    sample.specific_force_mps2 = {0.2 * i, 0, cfg.imu.gravity_mps2};
    causal.ingestImu(sample);
    with_future.ingestImu(sample);
  }
  uwb_imu_pl::ImuMeasurement future;
  future.timestamp = uwb_imu_pl::TimestampNs(15000000);
  future.specific_force_mps2 = {1000, 1000, 1000};
  with_future.ingestImu(future);
  const auto target = uwb_imu_pl::TimestampNs(10000000);
  causal.predictTo(target);
  with_future.predictTo(target);
  const auto left = causal.currentState();
  const auto right = with_future.currentState();
  EXPECT_TRUE(left.position_world_m.isApprox(right.position_world_m, 0.0));
  EXPECT_TRUE(left.velocity_world_mps.isApprox(right.velocity_world_mps, 0.0));
  EXPECT_TRUE(left.q_world_body.coeffs().isApprox(
      right.q_world_body.coeffs(), 0.0));
}

TEST(UwbImuIncremental, FaultAlarmRejectsOnlyCurrentUwbAndNextBatchRecovers) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::IncrementalUwbImuEstimator imu_only(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  imu_only.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          cfg.risk, cfg.snapshot.rank_tolerance,
          cfg.snapshot.max_condition_number));
  for (int i = 0; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement sample;
    sample.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
    pipeline.ingestImu(sample);
    imu_only.ingestImu(sample);
  }
  auto faulted = makeBatch(uwb_imu_pl::TimestampNs(10000000),
                           initial.position_world_m);
  faulted.measurements.front().range_m += 20.0;
  imu_only.predictTo(faulted.timestamp);
  const auto imu_only_state = imu_only.currentState();
  const auto imu_only_audit = imu_only.audit();
  const auto output = pipeline.processUwbBatch(faulted);
  const auto after_alarm = estimator.audit();
  EXPECT_TRUE(output.detector.numerically_valid);
  EXPECT_FALSE(output.detector.passed);
  EXPECT_FALSE(output.batch_committed);
  EXPECT_EQ(output.protection_level.availability,
            uwb_imu_pl::Availability::Alert);
  EXPECT_FALSE(output.protection_level.pl_xyz_m.allFinite());
  EXPECT_EQ(after_alarm.factor_count, imu_only_audit.factor_count);
  EXPECT_TRUE(after_alarm.version == imu_only_audit.version);
  EXPECT_TRUE(after_alarm.committed_uwb_batch_ids.empty());
  EXPECT_TRUE(output.state.position_world_m.isApprox(
      imu_only_state.position_world_m, 0.0));
  EXPECT_TRUE(output.state.velocity_world_mps.isApprox(
      imu_only_state.velocity_world_mps, 0.0));
  EXPECT_TRUE(output.state.accel_bias_mps2.isApprox(
      imu_only_state.accel_bias_mps2, 0.0));
  EXPECT_TRUE(output.state.gyro_bias_radps.isApprox(
      imu_only_state.gyro_bias_radps, 0.0));
  EXPECT_TRUE(output.state.q_world_body.coeffs().isApprox(
      imu_only_state.q_world_body.coeffs(), 0.0));

  for (int i = 3; i <= 4; ++i) {
    uwb_imu_pl::ImuMeasurement sample;
    sample.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
    pipeline.ingestImu(sample);
  }
  auto recovery_batch = makeBatch(
      uwb_imu_pl::TimestampNs(20000000), initial.position_world_m);
  recovery_batch.measurements.pop_back();
  const auto recovered = pipeline.processUwbBatch(recovery_batch);
  EXPECT_TRUE(recovered.detector.passed);
  EXPECT_TRUE(recovered.batch_committed);
  ASSERT_EQ(estimator.audit().committed_uwb_batch_ids.size(), 1u);
}

TEST(UwbImuIncremental, RiskOrAlertLimitUnavailableStillCommitsValidUwb) {
  for (int mode = 0; mode < 2; ++mode) {
    auto cfg = config();
    if (mode == 0) cfg.risk.p_hmi_total = 1e-8;
    else {
      cfg.risk.horizontal_alert_limit_m = 1e-9;
      cfg.risk.vertical_alert_limit_m = 1e-9;
    }
    uwb_imu_pl::IncrementalUwbImuEstimator estimator(
        cfg, Eigen::Vector3d::Zero());
    uwb_imu_pl::NavigationState initial;
    initial.timestamp = uwb_imu_pl::TimestampNs(0);
    initial.position_world_m = {0, 0, 1};
    estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
    uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
        &estimator, uwb_imu_pl::IntegrityMonitor(
            cfg.risk, cfg.snapshot.rank_tolerance,
            cfg.snapshot.max_condition_number));
    for (int i = 0; i <= 2; ++i) {
      uwb_imu_pl::ImuMeasurement sample;
      sample.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
      sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
      pipeline.ingestImu(sample);
    }
    const auto output = pipeline.processUwbBatch(makeBatch(
        uwb_imu_pl::TimestampNs(10000000), initial.position_world_m));
    EXPECT_TRUE(output.detector.passed);
    EXPECT_TRUE(output.measurement_model_valid);
    EXPECT_EQ(output.protection_level.availability,
              uwb_imu_pl::Availability::Unavailable);
    EXPECT_TRUE(output.batch_committed);
    EXPECT_EQ(estimator.audit().committed_uwb_batch_ids.size(), 1u);
  }
}

TEST(UwbImuIncremental, BatchSkewFailsBeforeGraphMutation) {
  auto cfg = config();
  cfg.incremental.epoch_bin_s = 0.015;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  uwb_imu_pl::ImuMeasurement boundary;
  boundary.timestamp = initial.timestamp;
  boundary.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  auto batch = makeBatch(uwb_imu_pl::TimestampNs(10000000),
                         Eigen::Vector3d(0, 0, 1));
  batch.measurements.front().timestamp = uwb_imu_pl::TimestampNs(21000000);
  const auto before = estimator.audit();
  EXPECT_THROW(estimator.validateUwbBatch(batch), std::invalid_argument);
  batch.measurements.front().timestamp = uwb_imu_pl::TimestampNs(19000000);
  batch.measurements.back().timestamp = uwb_imu_pl::TimestampNs(1000000);
  EXPECT_THROW(estimator.validateUwbBatch(batch), std::invalid_argument);
  const auto after = estimator.audit();
  EXPECT_EQ(before.factor_count, after.factor_count);
  EXPECT_EQ(before.epoch, after.epoch);
  EXPECT_TRUE(before.version == after.version);
}

TEST(UwbImuIncremental, RepeatedExecutionIsDeterministic) {
  auto run = [] {
    auto cfg = config();
    uwb_imu_pl::IncrementalUwbImuEstimator estimator(
        cfg, Eigen::Vector3d::Zero());
    uwb_imu_pl::NavigationState initial;
    initial.timestamp = uwb_imu_pl::TimestampNs(0);
    initial.position_world_m = {0, 0, 1};
    estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
    uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
        &estimator, uwb_imu_pl::IntegrityMonitor(
            cfg.risk, cfg.snapshot.rank_tolerance,
            cfg.snapshot.max_condition_number));
    uwb_imu_pl::ImuMeasurement boundary;
    boundary.timestamp = initial.timestamp;
    boundary.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
    pipeline.ingestImu(boundary);
    std::vector<uwb_imu_pl::IntegrityOutput> outputs;
    for (int epoch = 1; epoch <= 3; ++epoch) {
      for (int i = (epoch - 1) * 2 + 1; i <= epoch * 2; ++i) {
        uwb_imu_pl::ImuMeasurement sample;
        sample.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
        sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
        pipeline.ingestImu(sample);
      }
      const auto time = uwb_imu_pl::TimestampNs(epoch * 10000000);
      outputs.push_back(pipeline.processUwbBatch(
          makeBatch(time, initial.position_world_m)));
    }
    return outputs;
  };
  const auto left = run();
  const auto right = run();
  ASSERT_EQ(left.size(), right.size());
  for (std::size_t i = 0; i < left.size(); ++i) {
    EXPECT_DOUBLE_EQ(left[i].detector.statistic, right[i].detector.statistic);
    EXPECT_TRUE(left[i].state.position_world_m.isApprox(
        right[i].state.position_world_m, 0.0));
    EXPECT_TRUE(left[i].protection_level.pl_xyz_m.isApprox(
        right[i].protection_level.pl_xyz_m, 0.0));
    EXPECT_EQ(left[i].batch_committed, right[i].batch_committed);
  }
}
