#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <gtest/gtest.h>

#include <Eigen/LU>
#include <Eigen/SVD>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <limits>
#include <set>

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

uwb_imu_pl::UwbBatch makeBatchWithCount(
    uwb_imu_pl::TimestampNs timestamp, const Eigen::Vector3d& position,
    std::size_t count) {
  const std::vector<Eigen::Vector3d> anchors = {
      {-5,-5,0}, {5,-5,1}, {5,5,3}, {-5,5,4},
      {0,-6,2}, {0,6,4}, {-7,0,1}, {7,0,5}};
  uwb_imu_pl::UwbBatch batch;
  batch.id = uwb_imu_pl::BatchId(timestamp.value());
  batch.timestamp = timestamp;
  batch.covariance_model_id = "changing_anchor_set_diagonal";
  for (std::size_t index = 0; index < count; ++index) {
    uwb_imu_pl::UwbMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(
        static_cast<std::uint64_t>(timestamp.value()) + index);
    measurement.factor_id = uwb_imu_pl::FactorId(measurement.id.value());
    measurement.anchor_id = uwb_imu_pl::AnchorId(index + 1);
    measurement.timestamp = timestamp;
    measurement.anchor_position_m = anchors[index];
    measurement.range_m = (position - anchors[index]).norm();
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

Eigen::Vector3d auditPosition(double t) {
  return {2.0 * std::sin(.25 * t), 1.5 * std::sin(.37 * t),
          1.2 + .5 * std::sin(.19 * t)};
}

Eigen::Vector3d auditVelocity(double t) {
  return {.5 * std::cos(.25 * t), .555 * std::cos(.37 * t),
          .095 * std::cos(.19 * t)};
}

Eigen::Vector3d auditAcceleration(double t) {
  return {-.125 * std::sin(.25 * t), -.20535 * std::sin(.37 * t),
          -.01805 * std::sin(.19 * t)};
}

Eigen::Quaterniond auditOrientation(double t) {
  const double roll = .12 * std::sin(.31 * t);
  const double pitch = .10 * std::sin(.27 * t);
  const double yaw = .35 * std::sin(.21 * t) + .08 * t;
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}

Eigen::Vector3d auditAngularVelocity(double t) {
  constexpr double kEpsilon = 1e-6;
  Eigen::Quaterniond delta =
      auditOrientation(t).conjugate() * auditOrientation(t + kEpsilon);
  delta.normalize();
  const Eigen::AngleAxisd axis_angle(delta);
  return axis_angle.axis() * axis_angle.angle() / kEpsilon;
}

uwb_imu_pl::UwbBatch auditBatch(std::size_t epoch, double t,
                                const Eigen::Vector3d& lever) {
  static const std::vector<Eigen::Vector3d> anchors = {
      {-5, -5, 1}, {-5, -5, 5}, {-5, 5, 1}, {-5, 5, 5},
      {5, -5, 1},  {5, -5, 5},  {5, 5, 1},  {5, 5, 5}};
  uwb_imu_pl::UwbBatch batch;
  batch.id = uwb_imu_pl::BatchId(epoch);
  batch.timestamp = uwb_imu_pl::TimestampNs(
      static_cast<std::int64_t>(std::llround(t * 1e9)));
  batch.covariance_model_id = "p1_06_full_graph_oracle";
  const Eigen::Vector3d antenna =
      auditPosition(t) + auditOrientation(t) * lever;
  for (std::size_t i = 0; i < anchors.size(); ++i) {
    uwb_imu_pl::UwbMeasurement m;
    m.id = uwb_imu_pl::MeasurementId(epoch * 100 + i + 1);
    m.factor_id = uwb_imu_pl::FactorId(m.id.value());
    m.anchor_id = uwb_imu_pl::AnchorId(i + 1);
    m.timestamp = batch.timestamp;
    m.anchor_position_m = anchors[i];
    m.range_m = (antenna - anchors[i]).norm();
    m.sigma_m = 0.1;
    batch.measurements.push_back(m);
  }
  return batch;
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

TEST(RealtimeFactors, P106FullTrajectoryBreaksSingleEpochLeverNullDirection) {
  // This is the complete frozen synthetic model, not the isolated per-epoch
  // UWB rotational block: initial pose/velocity/bias priors, gravity-aware
  // CombinedImuFactor intervals (including bias evolution), and non-zero
  // lever-arm UWB observations all enter the same whitened graph.
  auto cfg = config();
  cfg.imu.accelerometer_sigma = 0.0007071067811865475;
  cfg.imu.gyroscope_sigma = 0.00007071067811865475;
  const Eigen::Vector3d lever(0.25, -0.10, 0.08);
  Eigen::Matrix<double, 15, 1> prior_sigmas;
  prior_sigmas << .1, .1, .1, .5, .5, .5, .5, .5, .5,
                  .1, .1, .1, .01, .01, .01;
  auto xkey = [](std::size_t epoch) { return gtsam::Symbol('x', epoch); };
  auto vkey = [](std::size_t epoch) { return gtsam::Symbol('v', epoch); };
  auto bkey = [](std::size_t epoch) { return gtsam::Symbol('b', epoch); };
  auto pose_at = [](double t) {
    return gtsam::Pose3(gtsam::Rot3(auditOrientation(t).toRotationMatrix()),
                        gtsam::Point3(auditPosition(t)));
  };
  const gtsam::imuBias::ConstantBias zero_bias;
  gtsam::NonlinearFactorGraph graph;
  gtsam::NonlinearFactorGraph prior_graph;
  gtsam::NonlinearFactorGraph imu_graph;
  gtsam::NonlinearFactorGraph uwb_graph;
  gtsam::Values values;
  prior_graph.addPrior(xkey(0), pose_at(0.0),
                       gtsam::noiseModel::Diagonal::Sigmas(
                           prior_sigmas.head<6>()));
  prior_graph.addPrior(vkey(0), gtsam::Vector3(auditVelocity(0.0)),
                       gtsam::noiseModel::Diagonal::Sigmas(
                           prior_sigmas.segment<3>(6)));
  prior_graph.addPrior(bkey(0), zero_bias,
                       gtsam::noiseModel::Diagonal::Sigmas(
                           prior_sigmas.tail<6>()));
  graph.push_back(prior_graph);
  values.insert(xkey(0), pose_at(0.0));
  values.insert(vkey(0), gtsam::Vector3(auditVelocity(0.0)));
  values.insert(bkey(0), zero_bias);
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

  constexpr int kEpochs = 12;
  for (int epoch = 1; epoch <= kEpochs; ++epoch) {
    gtsam::PreintegratedCombinedMeasurements pim(params, zero_bias);
    for (int sample = 1; sample <= 10; ++sample) {
      const double t = (epoch - 1) * .05 + sample * .005;
      const Eigen::Quaterniond q = auditOrientation(t);
      pim.integrateMeasurement(
          q.conjugate() *
              (auditAcceleration(t) +
               Eigen::Vector3d(0, 0, cfg.imu.gravity_mps2)),
          auditAngularVelocity(t), .005);
    }
    const double t = epoch * .05;
    imu_graph.add(gtsam::CombinedImuFactor(
        xkey(epoch - 1), vkey(epoch - 1), xkey(epoch), vkey(epoch),
        bkey(epoch - 1), bkey(epoch), pim));
    uwb_graph.add(boost::make_shared<uwb_imu_pl::UwbPoseBatchFactor>(
        xkey(epoch), auditBatch(epoch, t, lever), lever));
    graph.add(imu_graph.back());
    graph.add(uwb_graph.back());
    values.insert(xkey(epoch), pose_at(t));
    values.insert(vkey(epoch), gtsam::Vector3(auditVelocity(t)));
    values.insert(bkey(epoch), zero_bias);

    if (epoch == 4 || epoch == 8 || epoch == kEpochs) {
      const auto linear = graph.linearize(values);
      const auto system = linear->jacobian();
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(system.first,
                                            Eigen::ComputeThinV);
      const auto singular = svd.singularValues();
      const double tolerance = singular(0) *
          std::max(system.first.rows(), system.first.cols()) *
          std::numeric_limits<double>::epsilon();
      const int rank = static_cast<int>((singular.array() > tolerance).count());
      EXPECT_EQ(rank, system.first.cols()) << "epoch=" << epoch;
      EXPECT_GT(singular(singular.size() - 1), tolerance)
          << "epoch=" << epoch;
      std::printf("[P106-FULL-GRAPH] epoch=%d rows=%lld cols=%lld rank=%d "
                  "sigma_max=%.17g sigma_min=%.17g tolerance=%.17g\n",
                  epoch, static_cast<long long>(system.first.rows()),
                  static_cast<long long>(system.first.cols()), rank,
                  singular(0), singular(singular.size() - 1), tolerance);
    }
  }

  // Apply the isolated UWB null motion (right rotation around the body-frame
  // lever) to every pose.  It preserves every antenna point/range, but the
  // complete trajectory's IMU/gravity and initial-prior objectives change.
  gtsam::Values perturbed(values);
  const Eigen::Vector3d axis = lever.normalized();
  for (int epoch = 0; epoch <= kEpochs; ++epoch) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta.head<3>() = .05 * axis;
    perturbed.update(xkey(epoch), values.at<gtsam::Pose3>(xkey(epoch)).retract(delta));
  }
  const double uwb_before = uwb_graph.error(values);
  const double uwb_after = uwb_graph.error(perturbed);
  const double imu_before = imu_graph.error(values);
  const double imu_after = imu_graph.error(perturbed);
  const double prior_before = prior_graph.error(values);
  const double prior_after = prior_graph.error(perturbed);
  EXPECT_NEAR(uwb_before, uwb_after, 1e-18);
  EXPECT_GT(imu_after, imu_before + 1e-6);
  EXPECT_GT(prior_after, prior_before + 1e-6);
  EXPECT_GT(graph.error(perturbed), graph.error(values) + 1e-6);
  std::printf("[P106-TRAJECTORY-PERTURBATION] uwb_before=%.17g "
              "uwb_after=%.17g imu_before=%.17g imu_after=%.17g "
              "prior_before=%.17g prior_after=%.17g\n",
              uwb_before, uwb_after, imu_before, imu_after,
              prior_before, prior_after);

  // Independent central-difference check of the complete whitened graph's
  // first derivative.  This covers the native CombinedImuFactor Jacobians
  // (including bias evolution and gravity), the three initial priors, and the
  // UWB/lever factors without reusing any individual factor's analytic
  // derivative as the oracle.
  auto check_directional_derivative = [&](const gtsam::NonlinearFactorGraph& g,
                                          const char* name) {
    std::set<gtsam::Key> keys;
    for (const auto& factor : g) {
      keys.insert(factor->keys().begin(), factor->keys().end());
    }
    gtsam::Ordering ordering;
    for (const auto key : keys) ordering.push_back(key);
    const auto dense = g.linearize(perturbed)->jacobian(ordering);
    Eigen::VectorXd direction(dense.first.cols());
    for (Eigen::Index i = 0; i < direction.size(); ++i)
      direction(i) = std::sin(static_cast<double>(i + 1));
    direction.normalize();
    gtsam::VectorValues tangent;
    Eigen::Index offset = 0;
    for (const auto key : ordering) {
      const char type = gtsam::Symbol(key).chr();
      const int dimension = type == 'v' ? 3 : 6;
      tangent.insert(key, direction.segment(offset, dimension));
      offset += dimension;
    }
    ASSERT_EQ(offset, direction.size());
    constexpr double epsilon = 1e-6;
    gtsam::VectorValues plus_delta = tangent;
    gtsam::VectorValues minus_delta = tangent;
    for (auto& item : plus_delta) item.second *= epsilon;
    for (auto& item : minus_delta) item.second *= -epsilon;
    const double numerical =
        (g.error(perturbed.retract(plus_delta)) -
         g.error(perturbed.retract(minus_delta))) / (2.0 * epsilon);
    const double analytic =
        (-dense.first.transpose() * dense.second).dot(direction);
    const double scale = std::max({1.0, std::abs(numerical),
                                   std::abs(analytic)});
    EXPECT_NEAR(analytic, numerical, 2e-6 * scale) << name;
    std::printf("[P106-GRAPH-JACOBIAN] class=%s analytic=%.17g "
                "finite_difference=%.17g relative_error=%.17g\n",
                name, analytic, numerical,
                std::abs(analytic - numerical) / scale);
  };
  check_directional_derivative(prior_graph, "initial_prior");
  check_directional_derivative(imu_graph, "combined_imu_bias_gravity");
  check_directional_derivative(uwb_graph, "uwb_lever");
  check_directional_derivative(graph, "complete_graph");
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

TEST(UwbImuIncremental, DirectConstructionRejectsSingleEpochFixedLag) {
  auto cfg = config();
  cfg.incremental.fixed_lag_epochs = 1;
  EXPECT_THROW(
      uwb_imu_pl::IncrementalUwbImuEstimator(cfg, Eigen::Vector3d::Zero()),
      std::invalid_argument);
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

TEST(UwbImuIncremental, MethodBDowndateRejectsIllConditionedSpdPrior) {
  auto cfg = config();
  cfg.incremental.method_b_max_condition = 10.0;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  Eigen::Matrix<double, 15, 15> prior =
      Eigen::Matrix<double, 15, 15>::Identity();
  prior(14, 14) = 100.0;
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(4, 15);
  h.block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
  const Eigen::Matrix4d r = Eigen::Matrix4d::Identity();
  const Eigen::Matrix<double, 15, 15> all_in =
      (prior.inverse() + h.transpose() * r.inverse() * h).inverse();
  EXPECT_FALSE(estimator.methodBCandidatePrior(all_in, h, r).has_value());
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

TEST(UwbImuIncremental, MethodBInformationVectorDowndateRecoversMeanAndCovariance) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  Eigen::Matrix<double, 15, 15> prior_covariance =
      Eigen::Matrix<double, 15, 15>::Identity() * 0.5;
  Eigen::Matrix<double, 15, 1> prior_mean =
      Eigen::Matrix<double, 15, 1>::LinSpaced(15, -0.2, 0.3);
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(5, 15);
  h.leftCols(5) = Eigen::MatrixXd::Identity(5, 5);
  Eigen::MatrixXd r = Eigen::MatrixXd::Identity(5, 5) * 0.04;
  Eigen::VectorXd y = Eigen::VectorXd::LinSpaced(5, 1.0, 2.0);
  const Eigen::Matrix<double, 15, 15> prior_information =
      prior_covariance.inverse();
  const Eigen::Matrix<double, 15, 15> all_in_information =
      prior_information + h.transpose() * r.inverse() * h;
  const Eigen::Matrix<double, 15, 15> all_in_covariance =
      all_in_information.inverse();
  const Eigen::Matrix<double, 15, 1> all_in_mean = all_in_covariance *
      (prior_information * prior_mean + h.transpose() * r.inverse() * y);
  const auto recovered = estimator.methodBCandidatePrior(
      all_in_mean, all_in_covariance, h, r, y);
  ASSERT_TRUE(recovered.has_value());
  EXPECT_LT((recovered->covariance - prior_covariance).norm() /
                prior_covariance.norm(), 1e-10);
  EXPECT_LT((recovered->mean - prior_mean).norm(), 1e-10);
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
  const double pose_delta = gtsam::Pose3::Logmap(
      batch_values.at<gtsam::Pose3>(xkey(kEpochs)).between(isam_pose)).norm();
  EXPECT_LT(pose_delta, 1e-5);
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
  const double marginal_relative_error =
      (actual - expected).norm() / expected.norm();
  EXPECT_LT(marginal_relative_error, 1e-5);
  std::printf("[P106-PRODUCTION-BATCH] epochs=%d pose_tangent_delta=%.17g "
              "marginal_relative_error=%.17g\n",
              kEpochs, pose_delta, marginal_relative_error);
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
  auto faulted = makeBatch(uwb_imu_pl::TimestampNs(10000000),
                           initial.position_world_m);
  faulted.measurements.front().range_m += 20.0;
  const auto output = pipeline.processUwbBatch(faulted);
  const auto after_alarm = estimator.audit();
  EXPECT_TRUE(output.detector.numerically_valid);
  EXPECT_FALSE(output.detector.passed);
  EXPECT_FALSE(output.batch_committed);
  EXPECT_EQ(output.protection_level.availability,
            uwb_imu_pl::Availability::Unavailable);
  EXPECT_FALSE(output.protection_level.pl_xyz_m.allFinite());
  // V2 fail-closed discard leaves current IMU and UWB pending and performs no
  // backend mutation.  The published state retains its committed timestamp.
  EXPECT_EQ(output.backend_updates, 0u);
  EXPECT_TRUE(output.stale_state);
  EXPECT_EQ(after_alarm.factor_count, 3u);
  EXPECT_EQ(after_alarm.epoch, 0u);
  EXPECT_EQ(after_alarm.state_timestamp, initial.timestamp);
  EXPECT_TRUE(after_alarm.committed_uwb_batch_ids.empty());
  EXPECT_TRUE(output.state.position_world_m.isApprox(
      initial.position_world_m, 0.0));
  EXPECT_TRUE(output.state.velocity_world_mps.isApprox(
      initial.velocity_world_mps, 0.0));
  EXPECT_TRUE(output.state.accel_bias_mps2.isApprox(
      initial.accel_bias_mps2, 0.0));
  EXPECT_TRUE(output.state.gyro_bias_radps.isApprox(
      initial.gyro_bias_radps, 0.0));
  EXPECT_TRUE(output.state.q_world_body.coeffs().isApprox(
      initial.q_world_body.coeffs(), 0.0));

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

TEST(UwbImuIncremental, FinalDeadlineCheckKeepsRealCommitButDowngradesPublish) {
  auto cfg = config();
  cfg.publication.deadline_ms = 1e-9;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
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
  EXPECT_TRUE(output.batch_committed);
  EXPECT_EQ(output.backend_updates, 1u);
  EXPECT_TRUE(output.deadline_missed);
  EXPECT_FALSE(output.publication.protected_output);
  EXPECT_TRUE(output.publication.unprotected_output);
  EXPECT_NE(std::find(output.reason_codes.begin(), output.reason_codes.end(),
                      "FINISH_DEADLINE_MISSED"),
            output.reason_codes.end());
  EXPECT_EQ(estimator.currentEpoch(), 1u);
}

TEST(UwbImuIncremental, PlausibleMultiAnchorAmbiguityDoesNotClaimHistoryLoss) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
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
  const auto nominal = pipeline.processUwbBatch(
      makeBatch(uwb_imu_pl::TimestampNs(10000000), initial.position_world_m));
  ASSERT_TRUE(nominal.batch_committed);
  ASSERT_EQ(estimator.audit().epoch, 1u);

  for (int i = 3; i <= 4; ++i) {
    uwb_imu_pl::ImuMeasurement sample;
    sample.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
    pipeline.ingestImu(sample);
  }
  auto faulted = makeBatch(
      uwb_imu_pl::TimestampNs(20000000), initial.position_world_m);
  faulted.measurements.front().range_m += 20.0;
  const auto output = pipeline.processUwbBatch(faulted);

  EXPECT_FALSE(output.detector.passed);
  EXPECT_EQ(output.fde_status, "AMBIGUOUS_UNAVAILABLE");
  EXPECT_FALSE(output.batch_committed);
  EXPECT_EQ(output.backend_updates, 0u);
  EXPECT_TRUE(output.stale_state);
  EXPECT_FALSE(output.controlled_reinitialization_required);
  EXPECT_EQ(output.timestamp, uwb_imu_pl::TimestampNs(10000000));
  EXPECT_EQ(estimator.audit().epoch, 1u);
  EXPECT_EQ(estimator.audit().committed_uwb_batch_ids.size(), 1u);
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

TEST(UwbImuIncremental, StaleUwbIsRejectedWithoutGraphOrVersionMutation) {
  auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          cfg.risk, cfg.snapshot.rank_tolerance,
          cfg.snapshot.max_condition_number));
  for (int index = 0; index <= 2; ++index) {
    uwb_imu_pl::ImuMeasurement sample;
    sample.timestamp = uwb_imu_pl::TimestampNs(index * 5000000);
    sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
    pipeline.ingestImu(sample);
  }
  pipeline.processUwbBatch(makeBatch(
      uwb_imu_pl::TimestampNs(10000000), initial.position_world_m));
  const auto before = estimator.audit();
  EXPECT_THROW(pipeline.processUwbBatch(makeBatch(
                   uwb_imu_pl::TimestampNs(5000000), initial.position_world_m)),
               std::invalid_argument);
  const auto after = estimator.audit();
  EXPECT_EQ(before.factor_count, after.factor_count);
  EXPECT_EQ(before.epoch, after.epoch);
  EXPECT_TRUE(before.version == after.version);
  EXPECT_TRUE(before.current_marginal.isApprox(after.current_marginal, 0.0));
}

TEST(UwbImuIncremental, OneSecondUwbDropAndChangingAnchorSetRecover) {
  auto cfg = config();
  cfg.incremental.fixed_lag_epochs = 3;
  // Keep this legacy short-lag regression valid under the V2 maturity rule:
  // fixed_lag_epochs must be strictly larger than window + recovery margin.
  cfg.integrity_window.epochs = 1;
  cfg.integrity_window.recovery_margin_epochs = 1;
  cfg.risk.hypotheses.clear();
  for (std::uint64_t id = 1; id <= 8; ++id) {
    uwb_imu_pl::FaultHypothesis hypothesis;
    hypothesis.id = uwb_imu_pl::HypothesisId(id);
    hypothesis.anchor_id = uwb_imu_pl::AnchorId(id);
    hypothesis.missed_detection_allocation = 1e-3;
    hypothesis.prior_probability_bound = 1e-4;
    cfg.risk.hypotheses.push_back(hypothesis);
  }
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          cfg.risk, cfg.snapshot.rank_tolerance,
          cfg.snapshot.max_condition_number));
  uwb_imu_pl::ImuMeasurement boundary;
  boundary.timestamp = initial.timestamp;
  boundary.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
  pipeline.ingestImu(boundary);
  const std::vector<std::size_t> counts{8, 6, 4, 8};
  const std::vector<int> uwb_times_ms{10, 1010, 1020, 1030};
  int imu_index = 1;
  for (std::size_t epoch = 0; epoch < counts.size(); ++epoch) {
    const int target_index = uwb_times_ms[epoch] / 5;
    for (; imu_index <= target_index; ++imu_index) {
      uwb_imu_pl::ImuMeasurement sample;
      sample.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>(imu_index) * 5000000LL);
      sample.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
      pipeline.ingestImu(sample);
    }
    const auto output = pipeline.processUwbBatch(makeBatchWithCount(
        uwb_imu_pl::TimestampNs(
            static_cast<std::int64_t>(uwb_times_ms[epoch]) * 1000000LL),
        initial.position_world_m, counts[epoch]));
    // A one-interval integrity window has two states.  UWB attached only to
    // the boundary state remains an explicit monitored block, so after the
    // first epoch the joint-window residual DOF includes both the boundary
    // and current UWB groups (each factor exactly once).
    const auto expected_window_uwb = counts[epoch] +
        (epoch == 0 ? 0U : counts[epoch - 1]);
    //
    // C1-b re-baseline (2026-09-21): the condensed boundary is now the
    // fault-preserving history summary, whose pooled dof identity is
    // nu_pooled = nu_c + nu_perp.  The legacy UWB-only count therefore gains
    // exactly the summary's detector-only rows (nu_perp, exported in the
    // attempt diagnostics).  Measured at the time of the change: nu_perp = 0
    // while the A3 horizon is empty (epochs 0-1: dof 8 and 14, unchanged),
    // and nu_perp = 31 / 35 once historical fault columns exist (epochs 2-3).
    // Tolerance: exact integer identity, no slack.
    EXPECT_EQ(output.detector.dof,
              static_cast<int>(expected_window_uwb) +
                  output.diagnostics.history_summary.nu_perp);
    EXPECT_EQ(output.measurement_group_size, counts[epoch]);
    EXPECT_TRUE(output.measurement_model_valid) << output.detector.reason;
    EXPECT_TRUE(output.batch_committed) << output.detector.reason;
  }
  const auto final_audit = estimator.audit();
  EXPECT_TRUE(final_audit.fixed_lag_active);
  EXPECT_EQ(final_audit.retained_epochs, 3u);
  EXPECT_EQ(final_audit.marginalization_count, 2u);
  EXPECT_EQ(final_audit.committed_uwb_batch_ids.size(), 3u);

  uwb_imu_pl::IncrementalUwbImuEstimator reinitialized(
      cfg, Eigen::Vector3d::Zero());
  reinitialized.initialize(estimator.currentState(),
                           Eigen::Matrix<double, 15, 1>::Constant(0.1));
  const auto reset_audit = reinitialized.audit();
  EXPECT_TRUE(reset_audit.fixed_lag_active);
  EXPECT_EQ(reset_audit.retained_epochs, 1u);
  EXPECT_EQ(reset_audit.active_value_count, 3u);
  EXPECT_EQ(reset_audit.marginalization_count, 0u);
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
    for (int axis = 0; axis < 3; ++axis) {
      const double lhs = left[i].protection_level.pl_xyz_m(axis);
      const double rhs = right[i].protection_level.pl_xyz_m(axis);
      EXPECT_TRUE((std::isfinite(lhs) && lhs == rhs) ||
                  (std::isinf(lhs) && std::isinf(rhs)) ||
                  (std::isnan(lhs) && std::isnan(rhs)));
    }
    EXPECT_EQ(left[i].batch_committed, right[i].batch_committed);
  }
}

TEST(UwbImuIncremental, FixedLagRetainsThreeCompleteEpochsAndBoundedMetadata) {
  auto cfg = config();
  cfg.incremental.fixed_lag_epochs = 3;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(
      cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));

  uwb_imu_pl::ImuMeasurement boundary;
  boundary.timestamp = initial.timestamp;
  boundary.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
  estimator.ingestImu(boundary);

  std::size_t maximum_factor_slots = 0;
  std::size_t maximum_ledger_entries = 0;
  for (int epoch = 1; epoch <= 10; ++epoch) {
    for (int sample_index = 2 * epoch - 1; sample_index <= 2 * epoch;
         ++sample_index) {
      auto sample = boundary;
      sample.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>(sample_index) * 5000000LL);
      estimator.ingestImu(sample);
    }
    const auto time = uwb_imu_pl::TimestampNs(
        static_cast<std::int64_t>(epoch) * 10000000LL);
    const auto batch = makeBatchWithCount(
        time, initial.position_world_m, 4 + (epoch % 2));
    estimator.predictTo(time);
    const auto snapshot = estimator.preMeasurementSnapshot(batch);
    EXPECT_TRUE(snapshot->capabilities().fixed_lag);
    EXPECT_FALSE(snapshot->capabilities().historical_fault_provenance);
    if (epoch % 4 == 0) estimator.rejectUwbBatch(batch, "controlled reject");
    else estimator.commitUwbBatch(batch);

    const auto audit = estimator.audit();
    maximum_factor_slots = std::max(maximum_factor_slots,
                                    audit.factor_slot_count);
    maximum_ledger_entries = std::max(
        maximum_ledger_entries, estimator.factorLedger().entries().size());
    EXPECT_TRUE(audit.fixed_lag_active);
    EXPECT_TRUE(audit.historical_fault_provenance);
    EXPECT_LE(audit.active_value_count, 9u);
    EXPECT_LE(audit.committed_uwb_batch_ids.size(), 3u);
    EXPECT_EQ(audit.timestamp_count, audit.active_value_count);
    if (epoch >= 2) {
      EXPECT_EQ(audit.retained_epochs, 3u);
      EXPECT_EQ(audit.pose_value_count, 3u);
      EXPECT_EQ(audit.velocity_value_count, 3u);
      EXPECT_EQ(audit.bias_value_count, 3u);
      EXPECT_EQ(audit.oldest_retained_epoch,
                static_cast<std::size_t>(epoch - 2));
    }
    if (epoch >= 3) {
      EXPECT_EQ(audit.marginalization_count,
                static_cast<std::uint64_t>(epoch - 2));
      EXPECT_GT(audit.boundary_prior_factor_count, 0u);
    }
  }
  EXPECT_LE(maximum_factor_slots, 18u);
  EXPECT_LE(maximum_ledger_entries, 30u)
      << "finalized factor provenance must be pruned with the fixed lag";
  EXPECT_TRUE(std::isfinite(estimator.globalGraphResidualStatistic()));
}

TEST(UwbImuIncremental, FixedLagMatchesUnboundedBeforeAndAfterMarginalization) {
  auto unbounded_cfg = config();
  auto fixed_cfg = unbounded_cfg;
  fixed_cfg.incremental.fixed_lag_epochs = 3;
  uwb_imu_pl::IncrementalUwbImuEstimator unbounded(
      unbounded_cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::IncrementalUwbImuEstimator fixed(
      fixed_cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  const auto sigmas = Eigen::Matrix<double, 15, 1>::Constant(0.1);
  unbounded.initialize(initial, sigmas);
  fixed.initialize(initial, sigmas);
  uwb_imu_pl::ImuMeasurement boundary;
  boundary.timestamp = initial.timestamp;
  boundary.specific_force_mps2 = {0, 0, unbounded_cfg.imu.gravity_mps2};
  unbounded.ingestImu(boundary);
  fixed.ingestImu(boundary);
  uwb_imu_pl::IntegrityMonitor unbounded_monitor(
      unbounded_cfg.risk, unbounded_cfg.snapshot.rank_tolerance,
      unbounded_cfg.snapshot.max_condition_number);
  uwb_imu_pl::IntegrityMonitor fixed_monitor(
      fixed_cfg.risk, fixed_cfg.snapshot.rank_tolerance,
      fixed_cfg.snapshot.max_condition_number);

  for (int epoch = 1; epoch <= 8; ++epoch) {
    for (int sample_index = 2 * epoch - 1; sample_index <= 2 * epoch;
         ++sample_index) {
      auto sample = boundary;
      sample.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>(sample_index) * 5000000LL);
      unbounded.ingestImu(sample);
      fixed.ingestImu(sample);
    }
    const auto time = uwb_imu_pl::TimestampNs(
        static_cast<std::int64_t>(epoch) * 10000000LL);
    const auto batch = makeBatch(time, initial.position_world_m);
    unbounded.predictTo(time);
    fixed.predictTo(time);
    const auto unbounded_snapshot = unbounded.preMeasurementSnapshot(batch);
    const auto fixed_snapshot = fixed.preMeasurementSnapshot(batch);
    const auto left = unbounded.currentState();
    const auto right = fixed.currentState();
    EXPECT_LT((left.position_world_m - right.position_world_m).norm(), 1e-5);
    EXPECT_LT((left.velocity_world_mps - right.velocity_world_mps).norm(), 1e-5);
    EXPECT_LT((left.accel_bias_mps2 - right.accel_bias_mps2).norm(), 1e-5);
    EXPECT_LT((left.gyro_bias_radps - right.gyro_bias_radps).norm(), 1e-5);
    EXPECT_LT(Eigen::AngleAxisd(left.q_world_body.conjugate() *
                               right.q_world_body).angle(), 1e-5);
    const Eigen::MatrixXd left_cov = unbounded_snapshot->currentMarginal();
    const Eigen::MatrixXd right_cov = fixed_snapshot->currentMarginal();
    EXPECT_LT((left_cov - right_cov).norm() / left_cov.norm(), 1e-5);
    const auto left_integrity =
        unbounded_monitor.evaluateConditional(batch, *unbounded_snapshot);
    const auto right_integrity =
        fixed_monitor.evaluateConditional(batch, *fixed_snapshot);
    EXPECT_EQ(left_integrity.detector.passed, right_integrity.detector.passed);
    EXPECT_EQ(left_integrity.batch_committed, right_integrity.batch_committed);
    EXPECT_EQ(left_integrity.protection_level.availability,
              right_integrity.protection_level.availability);
    EXPECT_EQ(left_integrity.protection_level.label,
              uwb_imu_pl::IntegrityLabel::FormalLocalCurrentFaultOnly);
    EXPECT_EQ(right_integrity.protection_level.label,
              uwb_imu_pl::IntegrityLabel::FormalLocalCurrentFaultOnly);
    const double statistic_scale = std::max(
        1.0, std::abs(left_integrity.detector.statistic));
    EXPECT_LT(std::abs(left_integrity.detector.statistic -
                       right_integrity.detector.statistic) / statistic_scale,
              1e-5);
    const double pl_scale = std::max(
        1.0, left_integrity.protection_level.pl_xyz_m.norm());
    EXPECT_LT((left_integrity.protection_level.pl_xyz_m -
               right_integrity.protection_level.pl_xyz_m).norm() / pl_scale,
              1e-4);
    if (epoch <= 2) {
      EXPECT_TRUE(left_cov.isApprox(right_cov, 1e-12));
      EXPECT_NEAR(left_integrity.detector.statistic,
                  right_integrity.detector.statistic, 1e-12);
      EXPECT_TRUE(left_integrity.protection_level.pl_xyz_m.isApprox(
          right_integrity.protection_level.pl_xyz_m, 1e-12));
    }
    unbounded.commitUwbBatch(batch);
    fixed.commitUwbBatch(batch);
  }
}
