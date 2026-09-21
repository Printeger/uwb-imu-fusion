#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/estimation/reinitialization.hpp"
#include "uwb_imu_pl/factors/kinematic_bridge_factor.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/coverage_envelope.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/integrity/health_manager.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <gtest/gtest.h>
#include <boost/math/distributions/normal.hpp>
#include <random>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include "uwb_imu_pl/estimation/candidate_replay.hpp"
#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"
#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"
#include <future>
#include <atomic>
#include <cstdio>
#include <unistd.h>
#include <algorithm>
#include <cmath>

namespace {

uwb_imu_pl::LinearizedFactorBlock block(
    std::uint64_t id, const Eigen::MatrixXd& h, const Eigen::VectorXd& z,
    const uwb_imu_pl::LinearizationVersion& version) {
  uwb_imu_pl::LinearizedFactorBlock value;
  value.group_id = uwb_imu_pl::FactorGroupId(id);
  value.jacobian_whitened = h;
  value.residual_whitened = z;
  value.jacobian_raw = h;
  value.residual_raw = z;
  value.covariance = Eigen::MatrixXd::Identity(h.rows(), h.rows());
  value.whitener = value.covariance;
  value.version = version;
  return value;
}

uwb_imu_pl::LinearizedIntegrityWindow syntheticWindow() {
  uwb_imu_pl::LinearizedIntegrityWindow window;
  window.id = uwb_imu_pl::WindowId(4);
  window.version = {7, 2, 1, 9};
  Eigen::MatrixXd h0 = Eigen::MatrixXd::Identity(2, 2) * 3.0;
  Eigen::MatrixXd h1(3, 2);
  h1 << 1.0, 0.2, 0.1, 1.0, 0.7, -0.4;
  Eigen::MatrixXd h2(2, 2);
  h2 << 0.3, 0.8, -0.6, 0.2;
  window.blocks.push_back(block(1, h0, Eigen::Vector2d(0.1, -0.2), window.version));
  window.blocks.push_back(block(2, h1, Eigen::Vector3d(0.3, -0.1, 0.2), window.version));
  window.blocks.push_back(block(3, h2, Eigen::Vector2d(-0.2, 0.4), window.version));
  window.protected_state_map = Eigen::MatrixXd::Zero(3, 2);
  window.protected_state_map(0, 0) = 1.0;
  window.protected_state_map(1, 1) = 1.0;
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-10, 1e10);
  return window;
}

uwb_imu_pl::IntegrityConfig researchConfig() {
  return uwb_imu_pl::IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/config/realtime_uwb_imu_pl_research.yaml");
}

uwb_imu_pl::UwbBatch batch(const uwb_imu_pl::IntegrityConfig& config,
                           std::int64_t time_ns) {
  uwb_imu_pl::UwbBatch value;
  value.id = uwb_imu_pl::BatchId(static_cast<std::uint64_t>(time_ns));
  value.timestamp = uwb_imu_pl::TimestampNs(time_ns);
  for (std::size_t i = 0; i < config.anchors.size(); ++i) {
    uwb_imu_pl::UwbMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(i + 1);
    measurement.factor_id = uwb_imu_pl::FactorId(i + 1);
    measurement.anchor_id = config.anchors[i].id;
    measurement.timestamp = value.timestamp;
    measurement.anchor_position_m = config.anchors[i].position_world_m;
    measurement.range_m = (measurement.anchor_position_m -
                           Eigen::Vector3d(0, 0, 1)).norm();
    measurement.sigma_m = 0.05;
    value.measurements.push_back(measurement);
  }
  return value;
}

void addImu(uwb_imu_pl::IncrementalUwbImuEstimator* estimator,
            double gravity) {
  for (int i = 0; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(100 + i);
    measurement.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    measurement.specific_force_mps2 = {0, 0, gravity};
    estimator->ingestImu(measurement);
  }
}

}  // namespace

TEST(IntegrityV2Transaction, PrepareAndDiscardNeverMutateBackend) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  const auto before = estimator.audit();
  const auto ledger_version_before = estimator.factorLedger().version();
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  EXPECT_EQ(estimator.backendUpdateCount(), 1u);
  EXPECT_EQ(estimator.audit().epoch, before.epoch);
  EXPECT_EQ(estimator.audit().state_timestamp, before.state_timestamp);
  EXPECT_EQ(estimator.factorLedger().version(), ledger_version_before);
  auto window = estimator.buildIntegrityWindow(
      transaction, uwb_imu_pl::IntegrityWindowRequest{});
  EXPECT_TRUE(window.capabilities.includes_pending_imu);
  EXPECT_TRUE(window.capabilities.includes_pending_uwb);
  EXPECT_TRUE(window.capabilities.every_active_factor_accounted_once);
  EXPECT_TRUE(window.capabilities.frozen_slot_identity_valid);
  for (const auto& slot : window.slot_accounting) {
    EXPECT_NE(slot.explicit_window_block, slot.boundary_input);
    EXPECT_TRUE(slot.pointer_identity_valid);
  }
  const auto imu_block = estimator.buildPendingFactorBlock(
      transaction, transaction.imu_group.id);
  const auto bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bridge_group.id);
  EXPECT_EQ(transaction.generic_bridge_group.factors.size(), 1u);
  EXPECT_EQ(transaction.generic_bias_continuity_group.factors.size(), 1u);
  EXPECT_EQ(bridge_block.residual_whitened.size(), 9);
  const auto bias_bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bias_continuity_group.id);
  EXPECT_EQ(bias_bridge_block.residual_whitened.size(), 6);
  const auto analytic =
      uwb_imu_pl::ImuFaultSubspaceBuilder().buildAnalytic(transaction, imu_block);
  EXPECT_TRUE(analytic.analytic_input_valid);
  EXPECT_TRUE(analytic.analytic_computation_valid);
  EXPECT_FALSE(analytic.oracle_executed);
  EXPECT_FALSE(analytic.oracle_verified);
  const auto sensitivity = uwb_imu_pl::ImuFaultSubspaceBuilder()
      .verifyFiniteDifferenceOracle(transaction, imu_block);
  EXPECT_TRUE(sensitivity.oracle_verified) << sensitivity.oracle_relative_error;
  EXPECT_EQ(sensitivity.oracle_reintegrations, 12u);
  const auto receipt = estimator.discardEpoch(
      std::move(transaction), {uwb_imu_pl::FdeStatus::ModelInvalid, "test", false});
  EXPECT_EQ(receipt.backend_updates, 0u);
  EXPECT_EQ(estimator.backendUpdateCount(), 1u);
  EXPECT_EQ(estimator.currentState().timestamp, before.state_timestamp);
  EXPECT_EQ(estimator.factorLedger().version(), ledger_version_before);
}

TEST(IntegrityV2Transaction, CommitIsOneAtomicBackendUpdateAndLedgerIsComplete) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  const auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
  const auto receipt = estimator.commitEpoch(std::move(transaction), plan);
  EXPECT_EQ(receipt.backend_updates, 1u);
  EXPECT_EQ(estimator.backendUpdateCount(), 2u);
  EXPECT_EQ(estimator.currentEpoch(), 1u);
  EXPECT_TRUE(estimator.factorLedger().hasCompleteActiveProvenance());
}

TEST(IntegrityV2Transaction, BoundaryInventoryKeepsFirstStateUwbExplicitExactlyOnce) {
  using namespace uwb_imu_pl;
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary;
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  for (std::size_t epoch = 1; epoch <= 22; ++epoch) {
    for (int sample = 1; sample <= 10; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(epoch * 1000 + sample);
      imu.timestamp = TimestampNs(static_cast<std::int64_t>(
          ((epoch - 1) * 10 + sample) * 5000000));
      imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
      estimator.ingestImu(imu);
    }
    auto input = batch(config, static_cast<std::int64_t>(epoch * 50000000));
    for (std::size_t row = 0; row < input.measurements.size(); ++row) {
      input.measurements[row].id = MeasurementId(epoch * 100 + row + 1);
      input.measurements[row].factor_id = FactorId(epoch * 100 + row + 1);
    }
    auto transaction = estimator.prepareEpoch(input);
    const auto window = estimator.buildIntegrityWindow(
        transaction, IntegrityWindowRequest{});
    ASSERT_TRUE(window.model_valid) << "epoch=" << epoch << ": " << window.reason;
    if (epoch >= 19) {
      for (const auto& slot : window.slot_accounting) {
        EXPECT_NE(slot.explicit_window_block, slot.boundary_input)
            << "epoch=" << epoch << " slot=" << slot.slot;
      }
      if (epoch >= 21) {
        const std::size_t first = epoch - 20;
        const auto uwb = std::find_if(window.factor_inventory.begin(),
            window.factor_inventory.end(), [&](const auto& entry) {
              return entry.epoch == first && entry.kind == FactorKind::UwbBatch;
            });
        const auto imu = std::find_if(window.factor_inventory.begin(),
            window.factor_inventory.end(), [&](const auto& entry) {
              return entry.epoch == first && entry.kind == FactorKind::CombinedImu;
            });
        ASSERT_NE(uwb, window.factor_inventory.end());
        ASSERT_NE(imu, window.factor_inventory.end());
        EXPECT_EQ(uwb->disposition,
                  FrozenFactorDisposition::ExplicitMeasurement);
        EXPECT_EQ(imu->disposition, FrozenFactorDisposition::BoundaryInput);
        EXPECT_TRUE(std::any_of(window.blocks.begin(), window.blocks.end(),
            [&](const auto& value) { return value.group_id == uwb->group_id; }));
        EXPECT_FALSE(std::any_of(window.blocks.begin(), window.blocks.end(),
            [&](const auto& value) { return value.group_id == imu->group_id; }));
      }
    }
    const auto plan = EpochCommitPlan::nominalPlan(transaction);
    estimator.commitEpoch(std::move(transaction), plan);
  }
}

TEST(IntegrityV2Transaction, ReusingFinalizedTransactionFailsWithoutMutation) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  auto duplicate = transaction;
  const auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
  estimator.commitEpoch(std::move(transaction), plan);
  const auto updates = estimator.backendUpdateCount();
  EXPECT_THROW(estimator.commitEpoch(std::move(duplicate), plan), std::logic_error);
  EXPECT_EQ(estimator.backendUpdateCount(), updates);
}

TEST(IntegrityV2Transaction, CorrelatedUwbExclusionUsesPrincipalCovariance) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto correlated = batch(config, 10000000);
  const Eigen::Index n = static_cast<Eigen::Index>(correlated.measurements.size());
  correlated.covariance_m2 = Eigen::MatrixXd::Identity(n, n) * 0.04;
  for (Eigen::Index i = 0; i + 1 < n; ++i) {
    correlated.covariance_m2(i, i + 1) = 0.005;
    correlated.covariance_m2(i + 1, i) = 0.005;
  }
  auto transaction = estimator.prepareEpoch(correlated);
  ASSERT_GE(transaction.uwb_groups.size(), 2u);
  const auto replacement = std::find_if(
      transaction.uwb_groups.begin(), transaction.uwb_groups.end(),
      [&](const uwb_imu_pl::PendingFactorGroup& group) {
        return group.excluded_fault_units.size() == 1 &&
               group.excluded_fault_units.front().value() ==
                   correlated.measurements.front().anchor_id.value();
      });
  ASSERT_NE(replacement, transaction.uwb_groups.end());
  EXPECT_TRUE(replacement->raw_covariance.isApprox(
      correlated.covariance_m2.bottomRightCorner(n - 1, n - 1), 0.0));
  estimator.discardEpoch(std::move(transaction),
      {uwb_imu_pl::FdeStatus::ModelInvalid, "test complete", false});
}

TEST(IntegrityV2FaultModel, GeneratesCompactUwbModesAndEveryImuAxis) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  const auto window = estimator.buildIntegrityWindow(
      transaction, uwb_imu_pl::IntegrityWindowRequest{});
  ASSERT_TRUE(window.model_valid) << window.reason;
  const auto imu_block = estimator.buildPendingFactorBlock(
      transaction, transaction.imu_group.id);
  const auto bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bridge_group.id);
  const auto imu_maps =
      uwb_imu_pl::ImuFaultSubspaceBuilder().build(transaction, imu_block);
  const auto models = uwb_imu_pl::HypothesisGenerator().generate(
      window, transaction, imu_maps, bridge_block);

  std::size_t epoch_modes = 0, persistent_modes = 0, ramp_modes = 0;
  std::size_t imu_modes = 0;
  for (const auto& mode : models.modes) {
    if (mode.sensor == uwb_imu_pl::SensorType::Uwb) {
      epoch_modes += mode.kind ==
          uwb_imu_pl::FaultKind::AnchorBiasEpochIndependent;
      persistent_modes += mode.kind ==
          uwb_imu_pl::FaultKind::AnchorBiasPersistentConstant;
      ramp_modes += mode.kind == uwb_imu_pl::FaultKind::AnchorBiasRamp;
      EXPECT_EQ(mode.affected_groups.size(), 1u);
      if (mode.kind == uwb_imu_pl::FaultKind::AnchorBiasRamp) {
        EXPECT_EQ(mode.parameter_dimension, 2);
        EXPECT_TRUE(mode.effective_basis_certified);
        EXPECT_EQ(mode.effective_parameter_dimension, 1);
        EXPECT_EQ(mode.effective_parameter_basis.rows(), 2);
        EXPECT_EQ(mode.effective_parameter_basis.cols(), 1);
        EXPECT_LE(mode.discarded_measurement_norm, 1e-12);
        EXPECT_LE(mode.discarded_protected_response_norm, 1e-12);
      }
    } else {
      ++imu_modes;
    }
  }
  EXPECT_EQ(epoch_modes, config.anchors.size());
  EXPECT_EQ(persistent_modes, config.anchors.size());
  EXPECT_EQ(ramp_modes, config.anchors.size());
  EXPECT_EQ(imu_modes, 6u);
  EXPECT_TRUE(std::all_of(models.hypotheses.begin(), models.hypotheses.end(),
                          [](const auto& hypothesis) {
    return hypothesis.A.size() == 0 && !hypothesis.modes.empty() &&
           !hypothesis.affected_groups.empty();
  }));

  uwb_imu_pl::HypothesisGeneratorConfig both_config;
  both_config.single_faults_enabled = true;
  both_config.double_faults_enabled = true;
  const auto both = uwb_imu_pl::HypothesisGenerator(both_config).generate(
      window, transaction, imu_maps, bridge_block);
  EXPECT_EQ(both.modes.size(), models.modes.size());
  EXPECT_EQ(both.single_uwb_hypotheses, 3 * config.anchors.size());
  EXPECT_EQ(both.single_accel_hypotheses, 3u);
  EXPECT_EQ(both.single_gyro_hypotheses, 3u);
  EXPECT_EQ(both.double_uwb_accel_hypotheses,
            9 * config.anchors.size());
  EXPECT_EQ(both.double_uwb_gyro_hypotheses,
            9 * config.anchors.size());
  EXPECT_EQ(both.effective_max_cardinality, 2u);
  ASSERT_FALSE(both.hypotheses.empty());
  EXPECT_DOUBLE_EQ(both.hypotheses.front().hmi_allocation,
                   both_config.total_hmi_allocation / both.hypotheses.size());

  auto double_only_config = both_config;
  double_only_config.single_faults_enabled = false;
  const auto double_only =
      uwb_imu_pl::HypothesisGenerator(double_only_config).generate(
          window, transaction, imu_maps, bridge_block);
  EXPECT_EQ(double_only.modes.size(), models.modes.size());
  EXPECT_EQ(double_only.single_uwb_hypotheses, 0u);
  EXPECT_EQ(double_only.single_accel_hypotheses, 0u);
  EXPECT_EQ(double_only.single_gyro_hypotheses, 0u);
  EXPECT_EQ(double_only.hypotheses.size(),
            both.double_uwb_accel_hypotheses +
                both.double_uwb_gyro_hypotheses);
  EXPECT_TRUE(std::all_of(double_only.hypotheses.begin(),
                          double_only.hypotheses.end(), [](const auto& h) {
    return h.modes.size() == 2;
  }));

  auto gyro_only_config = double_only_config;
  gyro_only_config.include_uwb_accel_combinations = false;
  const auto gyro_only =
      uwb_imu_pl::HypothesisGenerator(gyro_only_config).generate(
          window, transaction, imu_maps, bridge_block);
  EXPECT_EQ(gyro_only.double_uwb_accel_hypotheses, 0u);
  EXPECT_EQ(gyro_only.double_uwb_gyro_hypotheses,
            both.double_uwb_gyro_hypotheses);
  estimator.discardEpoch(std::move(transaction),
      {uwb_imu_pl::FdeStatus::ModelInvalid, "test complete", false});
}

TEST(IntegrityV2Bridge, AnalyticJacobiansMatchNumericalOracleAndCvIsExact) {
  const double dt = 0.05;
  const auto covariance = Eigen::Matrix<double, 9, 9>::Identity();
  uwb_imu_pl::KinematicPoseVelocityBridgeFactor factor(
      gtsam::Symbol('x', 0), gtsam::Symbol('v', 0),
      gtsam::Symbol('x', 1), gtsam::Symbol('v', 1), dt, covariance);
  const gtsam::Pose3 p0(
      gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3), gtsam::Point3(1.0, 2.0, -0.5));
  const gtsam::Vector3 v0(0.4, -0.2, 0.1);
  const gtsam::Pose3 p1(
      p0.rotation(), p0.translation() + dt * v0);
  const gtsam::Vector3 v1 = v0;
  gtsam::Matrix h1, h2, h3, h4;
  const auto error = factor.evaluateError(p0, v0, p1, v1, h1, h2, h3, h4);
  EXPECT_LT(error.norm(), 1e-12);
  const auto fn = [&](const gtsam::Pose3& a, const gtsam::Vector3& b,
                      const gtsam::Pose3& c, const gtsam::Vector3& d) {
    return factor.evaluateError(a, b, c, d);
  };
  EXPECT_TRUE(h1.isApprox(gtsam::numericalDerivative41<
      gtsam::Vector, gtsam::Pose3, gtsam::Vector3, gtsam::Pose3,
      gtsam::Vector3>(fn, p0, v0, p1, v1), 1e-7));
  EXPECT_TRUE(h2.isApprox(gtsam::numericalDerivative42<
      gtsam::Vector, gtsam::Pose3, gtsam::Vector3, gtsam::Pose3,
      gtsam::Vector3>(fn, p0, v0, p1, v1), 1e-7));
  EXPECT_TRUE(h3.isApprox(gtsam::numericalDerivative43<
      gtsam::Vector, gtsam::Pose3, gtsam::Vector3, gtsam::Pose3,
      gtsam::Vector3>(fn, p0, v0, p1, v1), 1e-7));
  EXPECT_TRUE(h4.isApprox(gtsam::numericalDerivative44<
      gtsam::Vector, gtsam::Pose3, gtsam::Vector3, gtsam::Pose3,
      gtsam::Vector3>(fn, p0, v0, p1, v1), 1e-7));
}

TEST(IntegrityV2Bridge, MultiplePoseMarginsAddAndBiasMarginStaysSeparate) {
  Eigen::Matrix<double, 3, Eigen::Dynamic> protected_map(3, 3);
  protected_map.setIdentity();
  Eigen::Matrix3d first_gain = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d second_gain;
  second_gain << 0.0, 2.0, 0.0,
                 0.0, 0.0, -1.0,
                 3.0, 0.0, 0.0;
  const Eigen::Vector3d first_bound(1.0, 2.0, 3.0);
  const Eigen::Vector3d second_bound(0.5, 1.5, 2.5);
  const uwb_imu_pl::BridgeFactory factory;
  const Eigen::Vector3d first = factory.propagateBoxMargin(
      protected_map, first_gain, first_bound);
  const Eigen::Vector3d second = factory.propagateBoxMargin(
      protected_map, second_gain, second_bound);
  const Eigen::Vector3d accumulated = first + second;
  EXPECT_TRUE(accumulated.isApprox(Eigen::Vector3d(4.0, 4.5, 4.5), 0.0));

  uwb_imu_pl::EpochTransaction transaction;
  transaction.previous_epoch = 1;
  transaction.proposed_epoch = 2;
  transaction.begin = uwb_imu_pl::TimestampNs(1000000000);
  transaction.end = uwb_imu_pl::TimestampNs(1050000000);
  transaction.generic_bias_continuity_group.id = uwb_imu_pl::FactorGroupId(9);
  const auto bias = factory.makeBiasContinuity(
      transaction, uwb_imu_pl::GenericBridgeSpec{});
  EXPECT_EQ(bias.kind, uwb_imu_pl::FactorKind::BiasContinuity);
  EXPECT_EQ(bias.model_id, "constant_bias_continuity");
  uwb_imu_pl::BridgeAuditRecord audit;
  EXPECT_FALSE(audit.bias_continuity_maneuver_margin_applied);
}

TEST(IntegrityV2Transaction, HistoricalUwbReplacementCommitsAtomically) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto first = estimator.prepareEpoch(batch(config, 10000000));
  const auto first_plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(first);
  estimator.commitEpoch(std::move(first), first_plan);

  for (int i = 3; i <= 4; ++i) {
    uwb_imu_pl::ImuMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(100 + i);
    measurement.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    measurement.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
    estimator.ingestImu(measurement);
  }
  auto transaction = estimator.prepareEpoch(batch(config, 20000000));
  ASSERT_EQ(transaction.recoverable_history.size(), 1u);
  const auto& history = transaction.recoverable_history.front();
  const auto historical_uwb = std::find_if(
      history.groups.begin(), history.groups.end(), [&](const auto& group) {
        return group.kind == uwb_imu_pl::FactorKind::UwbBatch &&
            std::find(history.selected_groups.begin(), history.selected_groups.end(),
                      group.id) != history.selected_groups.end();
      });
  ASSERT_NE(historical_uwb, history.groups.end());
  const auto replacement = std::find_if(
      history.groups.begin(), history.groups.end(), [&](const auto& group) {
        return group.kind == uwb_imu_pl::FactorKind::UwbBatch &&
            group.replaces_group && *group.replaces_group == historical_uwb->id &&
            group.excluded_fault_units.size() == 1 &&
            group.excluded_fault_units.front().value() == 1;
      });
  ASSERT_NE(replacement, history.groups.end());
  auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
  plan.groups_to_remove = {historical_uwb->id};
  plan.groups_to_add.push_back(replacement->id);
  plan.groups_to_add.erase(std::remove(plan.groups_to_add.begin(),
                                       plan.groups_to_add.end(),
                                       transaction.imu_group.id),
                           plan.groups_to_add.end());
  plan.groups_to_add.push_back(transaction.generic_bridge_group.id);
  plan.groups_to_add.push_back(
      transaction.generic_bias_continuity_group.id);
  plan.replacement_relations[historical_uwb->id.value()] = replacement->id.value();
  plan.recovery_epoch_begin = history.proposed_epoch;
  plan.recovery_epoch_end = history.proposed_epoch;
  const auto historical_id = historical_uwb->id;
  const auto replacement_id = replacement->id;
  const auto updates_before = estimator.backendUpdateCount();
  const auto receipt = estimator.commitEpoch(std::move(transaction), plan);
  EXPECT_EQ(estimator.backendUpdateCount(), updates_before + 1);
  EXPECT_EQ(receipt.backend_updates, 1u);
  EXPECT_FALSE(receipt.historical_groups_removed.empty());
  EXPECT_FALSE(receipt.historical_groups_added.empty());
  EXPECT_TRUE(estimator.factorLedger().activeSlots(historical_id).empty());
  EXPECT_FALSE(estimator.factorLedger().activeSlots(replacement_id).empty());

  // The historical replacement belongs only to epoch 1, while the bridge
  // pair belongs to epoch 2.  The next inventory must recover both catalogs
  // without attributing either group to the wrong committed epoch.
  for (int i = 5; i <= 6; ++i) {
    uwb_imu_pl::ImuMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(100 + i);
    measurement.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    measurement.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
    estimator.ingestImu(measurement);
  }
  auto next = estimator.prepareEpoch(batch(config, 30000000));
  const auto next_window = estimator.buildIntegrityWindow(
      next, uwb_imu_pl::IntegrityWindowRequest{});
  EXPECT_TRUE(next_window.capabilities.complete_factor_provenance)
      << next_window.reason;
  EXPECT_EQ(next_window.reason.find("provenance is incomplete"),
            std::string::npos) << next_window.reason;
  estimator.discardEpoch(std::move(next),
      {uwb_imu_pl::FdeStatus::ModelInvalid, "catalog regression complete", false});
}

TEST(IntegrityV2Reinitialization, ExecutesControlledStateSequence) {
  uwb_imu_pl::ControlledReinitializer controller;
  uwb_imu_pl::NavigationState committed;
  committed.timestamp = uwb_imu_pl::TimestampNs(100);
  const auto id = controller.request(
      uwb_imu_pl::FdeStatus::BridgeTimeout, "timeout", committed);
  EXPECT_NE(id.value(), 0u);
  EXPECT_EQ(controller.directive().state,
            uwb_imu_pl::ReinitializationState::Requested);
  controller.beginWaiting();
  uwb_imu_pl::ImuMeasurement stale;
  stale.timestamp = committed.timestamp;
  EXPECT_FALSE(controller.acceptTrustedImu(stale).has_value());
  uwb_imu_pl::ImuMeasurement trusted;
  trusted.timestamp = uwb_imu_pl::TimestampNs(101);
  trusted.specific_force_mps2.setZero();
  trusted.angular_velocity_radps.setZero();
  EXPECT_TRUE(controller.acceptTrustedImu(trusted).has_value());
  EXPECT_EQ(controller.directive().state,
            uwb_imu_pl::ReinitializationState::Reinitialized);
  controller.complete();
  EXPECT_EQ(controller.directive().state,
            uwb_imu_pl::ReinitializationState::Running);
}

TEST(IntegrityV2Reinitialization, BridgeTimeoutAcceptsBoundaryAndRejectsFirstExcess) {
  using namespace uwb_imu_pl;
  EXPECT_FALSE(bridgeTimeoutExceeded(20, 1.0, 20, 1.0));
  EXPECT_TRUE(bridgeTimeoutExceeded(21, 1.0, 20, 1.0));
  EXPECT_TRUE(bridgeTimeoutExceeded(
      20, std::nextafter(1.0, 2.0), 20, 1.0));
  EXPECT_FALSE(bridgeTimeoutExceeded(19, 0.999999999, 20, 1.0));
}

TEST(IntegrityV2Window, RejectsFixedLagWithoutMaturityMargin) {
  auto config = researchConfig();
  config.incremental.fixed_lag_epochs =
      config.integrity_window.epochs + config.integrity_window.recovery_margin_epochs;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  const auto window = estimator.buildIntegrityWindow(
      transaction, uwb_imu_pl::IntegrityWindowRequest{});
  EXPECT_FALSE(window.model_valid);
  EXPECT_FALSE(window.capabilities.fixed_lag_maturity_valid);
  EXPECT_EQ(window.reason, "fixed lag violates integrity-window maturity delay");
  estimator.discardEpoch(std::move(transaction),
      {uwb_imu_pl::FdeStatus::ModelInvalid, "test complete", false});
}

TEST(IntegrityV2RankUpdate, MatchesIndependentDenseOracle) {
  auto window = syntheticWindow();
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-12, 1e10);
  ASSERT_TRUE(window.model_valid) << window.reason;
  uwb_imu_pl::ExclusionAction action;
  action.id = uwb_imu_pl::ExclusionActionId(2);
  action.groups_to_remove = {uwb_imu_pl::FactorGroupId(3)};
  const uwb_imu_pl::RankUpdateConfig config{1e-12, 1e10, 10.0};
  const auto base = uwb_imu_pl::RankUpdateEvaluator(config).factorizeOnce(window);
  const auto fast = uwb_imu_pl::RankUpdateEvaluator(config).evaluate(base, action);
  const auto dense = uwb_imu_pl::DenseCandidateOracle(config).evaluate(window, action);
  ASSERT_TRUE(fast.valid) << fast.reason;
  ASSERT_TRUE(dense.valid) << dense.reason;
  EXPECT_TRUE(fast.state_increment.isApprox(dense.state_increment, 1e-10));
  EXPECT_TRUE(fast.covariance.isApprox(dense.covariance, 1e-10));
  EXPECT_NEAR(fast.statistic, dense.statistic, 1e-10);
}

TEST(IntegrityV2RankUpdate, RejectsUnresolvedRemovalBeforeSelection) {
  const auto window = syntheticWindow();
  uwb_imu_pl::ExclusionAction action;
  action.id = uwb_imu_pl::ExclusionActionId(9);
  action.groups_to_remove = {uwb_imu_pl::FactorGroupId(999)};
  const uwb_imu_pl::RankUpdateEvaluator evaluator;
  const auto fast = evaluator.evaluate(evaluator.factorizeOnce(window), action);
  const auto dense = uwb_imu_pl::DenseCandidateOracle().evaluate(window, action);
  EXPECT_FALSE(fast.valid);
  EXPECT_FALSE(dense.valid);
  EXPECT_EQ(fast.reason, "candidate removal block is absent from frozen window");
  EXPECT_EQ(dense.reason, fast.reason);
}

TEST(IntegrityV2RankUpdate, OnlinePathKeepsOnlySharedBaseAndLowRankCorrections) {
  auto window = syntheticWindow();
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-12, 1e10);
  uwb_imu_pl::ExclusionAction action;
  action.id = uwb_imu_pl::ExclusionActionId(2);
  action.groups_to_remove = {uwb_imu_pl::FactorGroupId(3)};
  uwb_imu_pl::RankUpdateConfig config;
  config.rank_tolerance = 1e-12;
  config.max_condition_number = 1e10;
  config.max_linearization_step_norm = 10.0;
  config.materialize_dense_oracle_fields = false;
  const uwb_imu_pl::RankUpdateEvaluator evaluator(config);
  const auto candidate = evaluator.evaluate(evaluator.factorizeOnce(window), action);
  const auto dense = uwb_imu_pl::DenseCandidateOracle(config).evaluate(window, action);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  ASSERT_TRUE(candidate.shared_base_factorization);
  EXPECT_EQ(candidate.covariance.size(), 0);
  EXPECT_EQ(candidate.retained_jacobian.size(), 0);
  EXPECT_EQ(candidate.retained_jacobian_view, nullptr);
  EXPECT_TRUE(candidate.covarianceTimes(Eigen::MatrixXd::Identity(
      window.H.cols(), window.H.cols())).isApprox(dense.covariance, 1e-10));
  EXPECT_NEAR(candidate.statistic, dense.statistic, 1e-10);
  EXPECT_NEAR(candidate.information_logdet, dense.information_logdet, 1e-10);
}

TEST(IntegrityV2Detector, UsesSquaredParityDofAndUnionBound) {
  const auto window = syntheticWindow();
  uwb_imu_pl::DetectorRiskContext risk;
  risk.p_fa_per_test = 1e-6;
  risk.continuity_horizon_tests = 1000;
  const auto result = uwb_imu_pl::JointWindowDetector().evaluate(window, risk);
  EXPECT_TRUE(result.numerically_valid);
  EXPECT_EQ(result.dof, window.H.rows() - window.rank);
  EXPECT_GT(result.squared_threshold, 0.0);
  EXPECT_DOUBLE_EQ(result.operation_p_fa_upper_bound, 1e-3);
}

TEST(IntegrityV2ProtectionLevel, RecomputesPostCandidateAndStaysResearchOnly) {
  auto window = syntheticWindow();
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-12, 1e10);
  uwb_imu_pl::ExclusionAction keep;
  keep.id = uwb_imu_pl::ExclusionActionId(1);
  const uwb_imu_pl::RankUpdateConfig config{1e-12, 1e10, 10.0};
  const uwb_imu_pl::RankUpdateEvaluator evaluator(config);
  const auto candidate = evaluator.evaluate(evaluator.factorizeOnce(window), keep);
  uwb_imu_pl::DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  const auto detector = uwb_imu_pl::JointWindowDetector().evaluateCandidate(
      candidate, detector_risk);
  uwb_imu_pl::FaultHypothesisV2 first;
  first.id = uwb_imu_pl::HypothesisId(1);
  first.A = Eigen::MatrixXd::Zero(candidate.rows, 1);
  first.A(4, 0) = 1.0;
  first.p_md_allocation = 1e-3;
  first.hmi_allocation = 4e-6;
  first.prior_probability_bound = 1e-3;
  auto second = first;
  second.id = uwb_imu_pl::HypothesisId(2);
  second.A.setZero();
  second.A(5, 0) = 1.0;
  std::vector<uwb_imu_pl::FaultHypothesisV2> remaining{first, second};
  const auto result = uwb_imu_pl::ProtectionLevelV2().compute(
      window, candidate, detector, &remaining, uwb_imu_pl::RiskBudgetV2{});
  EXPECT_TRUE(result.model_valid) << result.reason;
  EXPECT_TRUE(result.pl_xyz_m.allFinite());
  EXPECT_TRUE(result.risk_budget_valid);
  EXPECT_FALSE(result.formal_eligible);
  EXPECT_TRUE(remaining[0].monitored);
  EXPECT_TRUE(remaining[1].monitored);
}

TEST(IntegrityV2ProtectionLevel,
     FrozenEvidenceContextMatchesKeepAllAndInvalidatesOnMutation) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  finalizeIntegrityWindow(&window, 1e-10, 1e10);

  FaultModeBasis uwb;
  uwb.id = FaultModeId(1);
  uwb.sensor = SensorType::Uwb;
  uwb.parameter_dimension = 1;
  uwb.raw_group_maps[FactorGroupId(2)] =
      (Eigen::Vector3d() << 1.0, 0.25, -0.5).finished();
  FaultModeBasis imu;
  imu.id = FaultModeId(2);
  imu.sensor = SensorType::ImuAccelerometer;
  imu.parameter_dimension = 1;
  imu.raw_group_maps[FactorGroupId(3)] =
      (Eigen::Vector2d() << 0.5, 1.0).finished();
  std::vector<FaultModeBasis> modes{uwb, imu};
  auto make_hypothesis = [](std::uint64_t id,
                            std::initializer_list<FaultModeId> mode_ids) {
    FaultHypothesisV2 value;
    value.id = HypothesisId(id);
    value.modes.assign(mode_ids.begin(), mode_ids.end());
    value.prior_probability_bound = 1e-4;
    value.p_md_allocation = 1e-3;
    value.hmi_allocation = 1e-6;
    return value;
  };
  std::vector<FaultHypothesisV2> reference_hypotheses{
      make_hypothesis(1, {FaultModeId(1)}),
      make_hypothesis(2, {FaultModeId(2)}),
      make_hypothesis(3, {FaultModeId(1), FaultModeId(2)})};
  auto optimized_hypotheses = reference_hypotheses;
  HypothesisEvaluationConfig reference_config;
  reference_config.enable_shared_context = false;
  reference_config.enable_low_dim_batch = false;
  const auto reference_evidence = HypothesisEvidenceEvaluator(reference_config)
      .evaluateAll(window, modes, &reference_hypotheses, 100.0);
  HypothesisEvaluationConfig optimized_config;
  optimized_config.hypothesis_workers = 4;
  CandidateWorkerPool pool(4);
  std::shared_ptr<const FrozenHypothesisNumerics> frozen;
  const auto optimized_evidence = HypothesisEvidenceEvaluator(optimized_config)
      .evaluateAll(window, modes, &optimized_hypotheses, 100.0, &frozen, &pool);
  ASSERT_TRUE(frozen && frozen->valid) << (frozen ? frozen->reason : "missing");
  ASSERT_EQ(reference_evidence.size(), optimized_evidence.size());
  EXPECT_EQ(frozen->low_dimensional_count, optimized_hypotheses.size());
  EXPECT_EQ(frozen->generic_fallback_count, 0u);
  for (std::size_t i = 0; i < reference_evidence.size(); ++i) {
    EXPECT_EQ(reference_hypotheses[i].monitored,
              optimized_hypotheses[i].monitored);
    EXPECT_EQ(reference_evidence[i].plausible, optimized_evidence[i].plausible);
    EXPECT_NEAR(reference_evidence[i].conditioned_statistic,
                optimized_evidence[i].conditioned_statistic, 1e-12);
    EXPECT_NEAR(reference_evidence[i].explained_energy,
                optimized_evidence[i].explained_energy, 1e-12);
    EXPECT_TRUE(reference_evidence[i].fault_gram.isApprox(
        optimized_evidence[i].fault_gram, 1e-12));
  }

  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  RankUpdateConfig rank_config{1e-10, 1e10, 10.0};
  const RankUpdateEvaluator evaluator(rank_config);
  auto candidate = evaluator.evaluate(evaluator.factorizeOnce(window), keep);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  const auto detector = JointWindowDetector().evaluateCandidate(
      candidate, detector_risk);
  ProtectionLevelSharedContext ordinary_context;
  ordinary_context.mode_maps[1] = Eigen::MatrixXd::Zero(window.H.rows(), 1);
  ordinary_context.mode_maps[1].middleRows(2, 3) = uwb.raw_group_maps.begin()->second;
  ordinary_context.mode_maps[2] = Eigen::MatrixXd::Zero(window.H.rows(), 1);
  ordinary_context.mode_maps[2].bottomRows(2) = imu.raw_group_maps.begin()->second;
  auto ordinary_hypotheses = optimized_hypotheses;
  auto ordinary_candidate = candidate;
  const auto ordinary = ProtectionLevelV2().computeShared(
      window, &ordinary_candidate, detector, &ordinary_hypotheses,
      ordinary_context, RiskBudgetV2{});
  auto frozen_candidate = candidate;
  const auto reused = ProtectionLevelV2().computeFrozenAllIn(
      window, &frozen_candidate, detector, optimized_hypotheses,
      modes,
      *frozen, frozen->fault_model_policy_fingerprint, RiskBudgetV2{});
  ASSERT_EQ(ordinary.model_valid, reused.model_valid);
  EXPECT_TRUE(ordinary.pl_xyz_m.isApprox(reused.pl_xyz_m, 1e-12));
  EXPECT_NEAR(ordinary.hpl_m, reused.hpl_m, 1e-12);
  EXPECT_NEAR(ordinary.vpl_m, reused.vpl_m, 1e-12);

  const auto stale_policy = ProtectionLevelV2().computeFrozenAllIn(
      window, &frozen_candidate, detector, optimized_hypotheses, modes,
      *frozen, frozen->fault_model_policy_fingerprint + 1, RiskBudgetV2{});
  EXPECT_FALSE(stale_policy.model_valid);
  EXPECT_NE(stale_policy.reason.find("identity mismatch"), std::string::npos);

  auto changed_modes = modes;
  changed_modes[0].raw_group_maps.begin()->second(0, 0) =
      std::nextafter(changed_modes[0].raw_group_maps.begin()->second(0, 0),
                     10.0);
  const auto stale_mode = ProtectionLevelV2().computeFrozenAllIn(
      window, &frozen_candidate, detector, optimized_hypotheses,
      changed_modes, *frozen, frozen->fault_model_policy_fingerprint,
      RiskBudgetV2{});
  EXPECT_FALSE(stale_mode.model_valid);
  EXPECT_NE(stale_mode.reason.find("identity mismatch"), std::string::npos);

  window.H(0, 0) = std::nextafter(window.H(0, 0), 10.0);
  const auto stale = ProtectionLevelV2().computeFrozenAllIn(
      window, &frozen_candidate, detector, optimized_hypotheses,
      modes,
      *frozen, frozen->fault_model_policy_fingerprint, RiskBudgetV2{});
  EXPECT_FALSE(stale.model_valid);
  EXPECT_NE(stale.reason.find("identity mismatch"), std::string::npos);
}

TEST(IntegrityV2ProtectionLevel,
     FixedLowDimKeepsSingularStatusAndUncommonDimensionFallsBack) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  FaultModeBasis first;
  first.id = FaultModeId(11);
  first.sensor = SensorType::Uwb;
  first.parameter_dimension = 1;
  first.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Ones();
  auto duplicate = first;
  duplicate.id = FaultModeId(12);
  FaultModeBasis uncommon;
  uncommon.id = FaultModeId(13);
  uncommon.sensor = SensorType::ImuAccelerometer;
  uncommon.parameter_dimension = 4;
  Eigen::Matrix<double, 3, 4> uncommon_map;
  uncommon_map << 1.0, 0.0, 0.0, 1.0,
                  0.0, 1.0, 0.0, 1.0,
                  0.0, 0.0, 1.0, 1.0;
  uncommon.raw_group_maps[FactorGroupId(2)] = uncommon_map;
  const std::vector<FaultModeBasis> modes{first, duplicate, uncommon};
  FaultHypothesisV2 singular;
  singular.id = HypothesisId(11);
  singular.modes = {first.id, duplicate.id};
  FaultHypothesisV2 dynamic;
  dynamic.id = HypothesisId(12);
  dynamic.modes = {uncommon.id};
  std::vector<FaultHypothesisV2> reference{singular, dynamic};
  auto optimized = reference;
  HypothesisEvaluationConfig reference_config;
  reference_config.enable_shared_context = false;
  reference_config.enable_low_dim_batch = false;
  const auto reference_evidence = HypothesisEvidenceEvaluator(reference_config)
      .evaluateAll(window, modes, &reference, 100.0);
  std::shared_ptr<const FrozenHypothesisNumerics> frozen;
  const auto optimized_evidence = HypothesisEvidenceEvaluator()
      .evaluateAll(window, modes, &optimized, 100.0, &frozen);
  ASSERT_TRUE(frozen && frozen->valid);
  ASSERT_EQ(reference_evidence.size(), optimized_evidence.size());
  EXPECT_EQ(frozen->low_dimensional_count, 1u);
  EXPECT_EQ(frozen->generic_fallback_count, 1u);
  // B2 (§5.7): the optimized path refuses the concatenated hypothesis whose
  // two modes share one direction (A12 = [A1, A2] doubles the same column), so
  // no rank/Gram is computed for it.  The reference path has no such guard and
  // still reports the singular rank.
  EXPECT_EQ(reference[0].monitorability.rank, 1);
  EXPECT_EQ(optimized[0].monitorability.rank, 0);
  EXPECT_FALSE(reference[0].monitored);
  EXPECT_FALSE(optimized[0].monitored);
  EXPECT_NE(optimized[0].monitorability.reason.find("concatenation refused"),
            std::string::npos)
      << optimized[0].monitorability.reason;
  EXPECT_FALSE(optimized_evidence[0].plausible);
  EXPECT_EQ(optimized_evidence[0].fault_gram.rows(), 0);
  for (std::size_t i = 1; i < optimized.size(); ++i) {
    EXPECT_EQ(reference[i].monitored, optimized[i].monitored);
    EXPECT_EQ(reference[i].monitorability.rank,
              optimized[i].monitorability.rank);
    EXPECT_EQ(reference_evidence[i].plausible,
              optimized_evidence[i].plausible);
    EXPECT_TRUE(reference_evidence[i].fault_gram.isApprox(
        optimized_evidence[i].fault_gram, 1e-12));
  }
}

TEST(IntegrityV2Fde, CoversEntirePlausibleSetBeforeSelection) {
  uwb_imu_pl::DetectorResultV2 detector;
  detector.numerically_valid = true;
  detector.passed = false;
  uwb_imu_pl::FaultHypothesisV2 h1, h2;
  h1.id = uwb_imu_pl::HypothesisId(1);
  h1.units = {uwb_imu_pl::FaultUnitId(11)};
  h2.id = uwb_imu_pl::HypothesisId(2);
  h2.units = {uwb_imu_pl::FaultUnitId(12)};
  uwb_imu_pl::FaultModeEvidence e1, e2;
  e1.hypothesis = h1.id;
  e2.hypothesis = h2.id;
  e1.plausible = e2.plausible = true;
  uwb_imu_pl::CandidateEvaluation one, both;
  one.valid = both.valid = true;
  one.post_detector_passed = both.post_detector_passed = true;
  one.hpl_m = one.vpl_m = both.hpl_m = both.vpl_m = 1.0;
  one.action.id = uwb_imu_pl::ExclusionActionId(1);
  one.action.covered_units = {h1.units.front()};
  one.action.exclusion_cardinality = 1;
  both.action.id = uwb_imu_pl::ExclusionActionId(2);
  both.action.covered_units = {h1.units.front(), h2.units.front()};
  both.action.exclusion_cardinality = 2;
  std::vector<uwb_imu_pl::CandidateEvaluation> candidates{one, both};
  const auto decision = uwb_imu_pl::FdeManager().decide(
      detector, {h1, h2}, {e1, e2}, &candidates, {},
      uwb_imu_pl::RiskBudgetV2{});
  ASSERT_TRUE(decision.selected_action.has_value());
  EXPECT_EQ(decision.selected_action->id, both.action.id);
  EXPECT_EQ(decision.status, uwb_imu_pl::FdeStatus::AmbiguousUnionExclusion);
}

TEST(IntegrityV2Fde, PlausibleStrictSupersetRetainsCoverageDutyInSelection) {
  using namespace uwb_imu_pl;
  DetectorResultV2 detector;
  detector.numerically_valid = true;
  detector.passed = false;
  FaultHypothesisV2 singleton;
  singleton.id = HypothesisId(1);
  singleton.modes = {FaultModeId(11)};
  FaultHypothesisV2 superset;
  superset.id = HypothesisId(2);
  superset.modes = {FaultModeId(11), FaultModeId(12)};
  FaultModeEvidence e1, e2;
  e1.hypothesis = singleton.id;
  e2.hypothesis = superset.id;
  e1.plausible = e2.plausible = true;
  CandidateEvaluation only_uwb;
  only_uwb.valid = true;
  only_uwb.post_detector_passed = true;
  only_uwb.hpl_m = only_uwb.vpl_m = 1.0;
  only_uwb.action.id = ExclusionActionId(1);
  only_uwb.action.covered_modes = {FaultModeId(11)};
  only_uwb.action.exclusion_cardinality = 1;
  std::vector<CandidateEvaluation> candidates{only_uwb};
  const auto decision = FdeManager().decide(
      detector, {singleton, superset}, {e1, e2}, &candidates,
      {}, RiskBudgetV2{});
  EXPECT_EQ(decision.plausible_hypotheses,
            (std::vector<HypothesisId>{singleton.id, superset.id}));
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_FALSE(decision.commit_allowed);
  EXPECT_EQ(decision.status, FdeStatus::AmbiguousUnavailable);
  EXPECT_FALSE(candidates.front().covers_plausible_set);
}

TEST(IntegrityV2Fde, ActionDedupRejectsSameShapeAndNormDifferentContent) {
  using namespace uwb_imu_pl;
  ExclusionAction left, right;
  left.groups_to_remove = right.groups_to_remove = {FactorGroupId(10)};
  left.groups_to_add = right.groups_to_add = {FactorGroupId(11)};
  left.bridge_mode = right.bridge_mode = BridgeMode::GenericKinematic;
  LinearizedFactorBlock a, b;
  a.group_id = b.group_id = FactorGroupId(11);
  a.version = b.version = {1, 2, 3, 4};
  a.jacobian_whitened = Eigen::Matrix2d::Identity();
  b.jacobian_whitened = (Eigen::Matrix2d() << 0.0, 1.0, 1.0, 0.0).finished();
  a.residual_whitened = Eigen::Vector2d(1.0, 0.0);
  b.residual_whitened = Eigen::Vector2d(0.0, 1.0);
  ASSERT_DOUBLE_EQ(a.jacobian_whitened.squaredNorm(),
                   b.jacobian_whitened.squaredNorm());
  ASSERT_DOUBLE_EQ(a.residual_whitened.squaredNorm(),
                   b.residual_whitened.squaredNorm());
  left.added_blocks = {a};
  right.added_blocks = {b};
  EXPECT_FALSE(equivalentActionOperation(left, right));
  right.added_blocks = left.added_blocks;
  EXPECT_TRUE(equivalentActionOperation(left, right));
}

TEST(IntegrityV2Fde, MandatoryGroupIsIndependentOfProbabilisticPlausibility) {
  using namespace uwb_imu_pl;
  DetectorResultV2 barrier;
  barrier.numerically_valid = true;
  barrier.passed = false;
  CandidateEvaluation keep, exclude;
  keep.valid = exclude.valid = true;
  keep.post_detector_passed = exclude.post_detector_passed = true;
  keep.hpl_m = keep.vpl_m = exclude.hpl_m = exclude.vpl_m = 1.0;
  keep.action.id = ExclusionActionId(1);
  keep.action.action_model_id = "KEEP_ALL";
  exclude.action.id = ExclusionActionId(2);
  exclude.action.action_model_id = "UWB_CURRENT_HEALTH_BARRIER";
  exclude.action.groups_to_remove = {FactorGroupId(77)};
  exclude.action.exclusion_cardinality = 1;
  std::vector<CandidateEvaluation> candidates{keep, exclude};
  const auto decision = FdeManager().decide(
      barrier, {}, {}, &candidates, {FactorGroupId(77)}, RiskBudgetV2{});
  ASSERT_TRUE(decision.selected_action.has_value());
  EXPECT_EQ(decision.selected_action->id, ExclusionActionId(2));
  EXPECT_TRUE(decision.plausible_hypotheses.empty());
  EXPECT_EQ(decision.mandatory_exclusion_groups,
            (std::vector<FactorGroupId>{FactorGroupId(77)}));
  EXPECT_FALSE(candidates[0].covers_plausible_set);
  EXPECT_TRUE(candidates[1].covers_plausible_set);
}

TEST(IntegrityV2Health, QuarantineRequiresConsecutiveRecovery) {
  uwb_imu_pl::HealthConfigV2 config;
  config.suspect_evidence_count = 2;
  config.recovery_shadow_passes = 2;
  config.recovery_test_passes = 2;
  uwb_imu_pl::HealthManager health(config);
  health.registerSource("accel:x", uwb_imu_pl::SensorType::ImuAccelerometer);
  EXPECT_EQ(health.observeEvidence("accel:x", true).current,
            uwb_imu_pl::HealthState::Healthy);
  EXPECT_EQ(health.observeEvidence("accel:x", true).current,
            uwb_imu_pl::HealthState::Suspect);
  EXPECT_EQ(health.quarantine("accel:x", "exclude").current,
            uwb_imu_pl::HealthState::Quarantined);
  EXPECT_EQ(health.observeShadowRecovery("accel:x", true).current,
            uwb_imu_pl::HealthState::Quarantined);
  EXPECT_EQ(health.observeShadowRecovery("accel:x", true).current,
            uwb_imu_pl::HealthState::RecoveryTest);
  EXPECT_EQ(health.observeShadowRecovery("accel:x", true).current,
            uwb_imu_pl::HealthState::RecoveryTest);
  EXPECT_EQ(health.observeShadowRecovery("accel:x", true).current,
            uwb_imu_pl::HealthState::Healthy);
  EXPECT_EQ(health.fail("accel:x", "hardware barrier").current,
            uwb_imu_pl::HealthState::Failed);
  EXPECT_FALSE(health.allowedInFormalEstimator("accel:x"));
}

TEST(IntegrityV2Health, RecoveryFailureResetsCountersAndRequarantines) {
  uwb_imu_pl::HealthConfigV2 config;
  config.suspect_evidence_count = 1;
  config.recovery_shadow_passes = 2;
  config.recovery_test_passes = 10;
  uwb_imu_pl::HealthManager health(config);
  health.registerSource("accel:x", uwb_imu_pl::SensorType::ImuAccelerometer);
  health.observeEvidence("accel:x", true);
  health.quarantine("accel:x", "exclude");
  health.observeShadowRecovery("accel:x", true);
  health.observeShadowRecovery("accel:x", true);
  for (int pass = 1; pass < 5; ++pass) {
    health.observeShadowRecovery("accel:x", true);
  }
  EXPECT_EQ(health.state("accel:x"), uwb_imu_pl::HealthState::RecoveryTest);
  health.observeShadowRecovery("accel:x", false);
  EXPECT_EQ(health.state("accel:x"), uwb_imu_pl::HealthState::Quarantined);
  ASSERT_EQ(health.snapshot().size(), 1u);
  EXPECT_EQ(health.snapshot().front().shadow_pass_count, 0u);
  EXPECT_EQ(health.snapshot().front().recovery_pass_count, 0u);
  EXPECT_EQ(health.snapshot().front().recovery_reset_count, 1u);
}

TEST(GateDNumerics, CompleteReplacementSurvivesSingularDeletionIntermediate) {
  using namespace uwb_imu_pl;
  LinearizedIntegrityWindow window;
  window.id = WindowId(80); window.version = {9, 8, 7, 6};
  window.blocks = {block(1, Eigen::Matrix2d::Identity(), Eigen::Vector2d(.01, -.02), window.version),
                   block(2, Eigen::MatrixXd::Zero(1, 2), Eigen::VectorXd::Constant(1, .03), window.version)};
  window.protected_state_map = Eigen::MatrixXd::Zero(3, 2);
  window.protected_state_map.topRows(2).setIdentity();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  ASSERT_TRUE(window.model_valid) << window.reason;
  ExclusionAction action;
  action.groups_to_remove = {FactorGroupId(1)};
  action.added_blocks = {block(3, 2.0 * Eigen::Matrix2d::Identity(), Eigen::Vector2d(.03, .04), window.version)};
  RankUpdateConfig config; config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator evaluator(config);
  const auto base = evaluator.factorizeOnce(window);
  const auto result = evaluator.evaluate(base, action);
  const auto oracle = DenseCandidateOracle(config).evaluate(window, action);
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_TRUE(oracle.valid) << oracle.reason;
  EXPECT_TRUE(result.diagnostics.recovered_replacement);
  EXPECT_TRUE(result.state_increment.isApprox(oracle.state_increment, 1e-9));
  EXPECT_TRUE(result.covarianceTimes(window.protected_state_map.transpose()).isApprox(
      oracle.covarianceTimes(window.protected_state_map.transpose()), 1e-9));
  EXPECT_NEAR(result.statistic, oracle.statistic, 1e-9);
  EXPECT_NEAR(result.information_logdet, oracle.information_logdet, 1e-9);
  EXPECT_EQ(result.dof, oracle.dof);
  EXPECT_EQ(result.covariance.size(), 0);
  action.added_blocks.clear();
  const auto singular = evaluator.evaluate(base, action);
  EXPECT_FALSE(singular.valid);
  EXPECT_EQ(singular.reason, "candidate rank loss");
}

TEST(GateDNumerics, FrozenNumericalContractMismatchFailsClosed) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  DetectorRiskContext detector;
  detector.rank_tolerance = 2e-12;
  detector.max_condition_number = 1e10;
  const auto detected = JointWindowDetector().evaluate(window, detector);
  EXPECT_FALSE(detected.numerically_valid);
  EXPECT_NE(detected.reason.find("numerical-contract-mismatched"),
            std::string::npos);

  RankUpdateConfig rank;
  rank.rank_tolerance = 2e-12;
  rank.max_condition_number = 1e10;
  const auto base = RankUpdateEvaluator(rank).factorizeOnce(window);
  EXPECT_FALSE(base.valid);
  EXPECT_NE(base.reason.find("stale frozen-window numerics"),
            std::string::npos);

  finalizeIntegrityWindow(&window, detector.rank_tolerance,
                          detector.max_condition_number);
  EXPECT_TRUE(JointWindowDetector().evaluate(window, detector).numerically_valid);
  EXPECT_TRUE(RankUpdateEvaluator(rank).factorizeOnce(window).valid);
}

TEST(GateDNumerics, FinalSvdRejectsConditionHiddenByCholeskyDiagonal) {
  using namespace uwb_imu_pl;
  LinearizedIntegrityWindow window;
  window.version = {1, 2, 3, 4};
  Eigen::MatrixXd h(3, 2); h << 1, 1000, 0, 1, 0, 0;
  window.blocks = {block(1, h, Eigen::Vector3d(.01, .001, .02), window.version),
                  block(2, 1000 * Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero(), window.version)};
  finalizeIntegrityWindow(&window, 1e-10, 1e4);
  ASSERT_LT(window.condition_number, 2);
  Eigen::LLT<Eigen::MatrixXd> misleading(h.transpose()*h);
  ASSERT_EQ(misleading.info(), Eigen::Success);
  const auto diag = misleading.matrixL().toDenseMatrix().diagonal().eval();
  EXPECT_NEAR(diag.maxCoeff()/diag.minCoeff(), 1, 1e-9);
  RankUpdateConfig config; config.max_condition_number = 1e4;
  config.exact_slow_path_condition = 1e4;
  config.enable_early_step_gate = false;
  config.materialize_dense_oracle_fields = false;
  ExclusionAction action; action.groups_to_remove = {FactorGroupId(2)};
  const RankUpdateEvaluator evaluator(config);
  const auto result = evaluator.evaluate(evaluator.factorizeOnce(window), action);
  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.condition_number, 1e5);
  EXPECT_EQ(result.reason, "candidate condition gate failed");
  EXPECT_TRUE(result.exact_slow_path);
}

TEST(GateDNumerics, EuclideanStepGateBothSidesUsesReferenceNearBoundary) {
  using namespace uwb_imu_pl;
  for (double scale : {1 - 1e-8, 1 + 1e-8}) {
    auto window = syntheticWindow();
    Eigen::Vector2d delta(.15, .2); delta *= scale;
    for (auto& b : window.blocks) b.residual_whitened = b.jacobian_whitened * delta;
    finalizeIntegrityWindow(&window, 1e-10, 1e10);
    RankUpdateConfig config; config.materialize_dense_oracle_fields = false;
    const RankUpdateEvaluator evaluator(config);
    const auto result = evaluator.evaluate(evaluator.factorizeOnce(window), ExclusionAction{});
    EXPECT_EQ(result.valid, scale < 1) << result.reason;
    EXPECT_TRUE(result.diagnostics.near_gate);
    EXPECT_TRUE(result.exact_slow_path);
    EXPECT_NEAR(result.state_increment.norm(), .25*scale, 1e-12);
    EXPECT_EQ(result.covariance.size(), 0);
  }
}

TEST(GateDNumerics, MalformedFullChangeRejectedBeforeAnyPartialOperation) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  const RankUpdateEvaluator evaluator;
  const auto base = evaluator.factorizeOnce(window);
  ExclusionAction action; action.groups_to_remove = {FactorGroupId(1)};
  auto addition = window.blocks.front(); addition.group_id = FactorGroupId(90);
  action.added_blocks = {addition};
  for (int which=0; which<7; ++which) {
    auto& b = action.added_blocks.front(); b = addition;
    if(which==0) b.version.ordering_version++;
    if(which==1) b.version.noise_model_version++;
    if(which==2) b.residual_whitened.resize(0);
    if(which==3) b.jacobian_whitened(0,0) = std::numeric_limits<double>::quiet_NaN();
    if(which==4) b.whitener(0,0) = std::numeric_limits<double>::quiet_NaN();
    if(which==5) b.covariance.resize(1,3);
    if(which==6) b.jacobian_raw.resize(1,3);
    const auto c = evaluator.evaluate(base, action);
    EXPECT_FALSE(c.valid);
    EXPECT_EQ(c.reason, "candidate addition block/version is invalid");
    EXPECT_GE(c.wall_ms, 0);
  }
}

TEST(GateDCache, CacheContentVersionWindowAndSwitchRemainEquivalent) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  ExclusionAction a; a.id = ExclusionActionId(7); a.groups_to_remove = {FactorGroupId(3)};
  auto added = window.blocks.back(); added.group_id = FactorGroupId(10);
  a.added_blocks = {added};
  auto changed = a; changed.id = ExclusionActionId(8);
  changed.added_blocks.front().jacobian_whitened *= 1.2;
  auto actions = std::vector<ExclusionAction>{a, changed};
  RankUpdateConfig config; config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator cached(config);
  config.enable_shared_cache = false;
  const RankUpdateEvaluator uncached(config);
  const auto base = cached.factorizeOnce(window, actions);
  ASSERT_TRUE(base.block_cache);
  EXPECT_EQ(base.block_cache->blocks.at(FactorGroupId(10)).size(), 2u);
  for (const auto& action : actions) {
    const auto c = cached.evaluate(base, action);
    const auto u = uncached.evaluate(uncached.factorizeOnce(window, actions), action);
    ASSERT_TRUE(c.valid) << c.reason;
    EXPECT_TRUE(c.state_increment.isApprox(u.state_increment, 1e-9));
    EXPECT_TRUE(c.covarianceTimes(window.protected_state_map.transpose()).isApprox(
        u.covarianceTimes(window.protected_state_map.transpose()), 1e-9));
    EXPECT_EQ(c.diagnostics.cache_hits, 2u);
    EXPECT_EQ(u.diagnostics.cache_hits, 0u);
  }
  // Unseen content with an existing ID must miss instead of using the old solve.
  auto unseen = a; unseen.added_blocks.front().residual_raw *= 2;
  EXPECT_EQ(cached.evaluate(base, unseen).diagnostics.cache_hits, 1u);
  auto stale = a; stale.added_blocks.front().version.linpoint_version++;
  EXPECT_FALSE(cached.evaluate(base, stale).valid);
  auto next = window; next.id = WindowId(999);
  finalizeIntegrityWindow(&next, 1e-10, 1e10);
  EXPECT_EQ(cached.factorizeOnce(next, actions).block_cache->window_id, next.id);
  next.version.noise_model_version++;
  for (auto& b : next.blocks) b.version = next.version;
  finalizeIntegrityWindow(&next, 1e-10, 1e10);
  EXPECT_FALSE(cached.evaluate(cached.factorizeOnce(next, actions), a).valid);
}

TEST(GateDCache, EarlyStepSwitchAgreesWithExactFinalJacobian) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  for(auto& b:window.blocks) b.residual_whitened = b.jacobian_whitened * Eigen::Vector2d(1,2);
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  RankUpdateConfig config; config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator early(config);
  config.enable_early_step_gate = false;
  const RankUpdateEvaluator exact(config);
  const auto fast = early.evaluate(early.factorizeOnce(window), ExclusionAction{});
  const auto full = exact.evaluate(exact.factorizeOnce(window), ExclusionAction{});
  EXPECT_FALSE(fast.valid); EXPECT_FALSE(full.valid);
  EXPECT_EQ(fast.reason, full.reason);
  EXPECT_EQ(fast.diagnostics.numerical_path, "QR_CERTIFIED_STEP_REJECTION");
  EXPECT_TRUE(fast.state_increment.isApprox(full.state_increment, 1e-9));
}

TEST(GateDReplay, LosslessRoundTripAllBlocksAndActions) {
  using namespace uwb_imu_pl;
  FrozenCandidateReplay replay;
  replay.window = syntheticWindow(); replay.input_attempt_id = 123;
  replay.transaction_id = 33; replay.input_timestamp = TimestampNs(99887766);
  replay.config.enable_shared_cache = false; replay.config.enable_early_step_gate = false;
  ExclusionAction action; action.id = ExclusionActionId(42);
  action.groups_to_remove = {FactorGroupId(3)};
  action.added_blocks = {replay.window.blocks.back()};
  action.added_blocks.front().group_id = FactorGroupId(100);
  action.covered_modes = {FaultModeId(91)}; action.covered_units = {FaultUnitId(19)};
  action.physical_source_ids = {"source with spaces"};
  action.recovery_epoch_begin = 5; action.recovery_epoch_end = 8;
  replay.actions = {action, action}; replay.actions.back().id = ExclusionActionId(43);
  const std::string path = "/tmp/gate-d-replay-" + std::to_string(getpid()) + ".bin";
  writeCandidateReplay(path, replay);
  const auto loaded = readCandidateReplay(path);
  std::remove(path.c_str());
  EXPECT_EQ(loaded.input_attempt_id, replay.input_attempt_id);
  EXPECT_EQ(loaded.config.enable_shared_cache, replay.config.enable_shared_cache);
  EXPECT_EQ(loaded.config.enable_early_step_gate, replay.config.enable_early_step_gate);
  EXPECT_EQ(loaded.input_timestamp, replay.input_timestamp);
  EXPECT_EQ(loaded.window.version, replay.window.version);
  EXPECT_TRUE(loaded.window.H.isApprox(replay.window.H, 0));
  EXPECT_TRUE(loaded.window.protected_state_map.isApprox(replay.window.protected_state_map, 0));
  ASSERT_EQ(loaded.actions.size(), 2u);
  EXPECT_EQ(loaded.actions[0].covered_modes, action.covered_modes);
  EXPECT_EQ(loaded.actions[0].physical_source_ids, action.physical_source_ids);
  EXPECT_EQ(loaded.actions[0].recovery_epoch_end, action.recovery_epoch_end);
  EXPECT_TRUE(loaded.actions[0].added_blocks[0].jacobian_whitened.isApprox(
      action.added_blocks[0].jacobian_whitened, 0));
  const RankUpdateEvaluator evaluator;
  auto base = evaluator.factorizeOnce(loaded.window, loaded.actions);
  EXPECT_TRUE(evaluator.evaluate(base, loaded.actions[0]).valid);
}

TEST(GateDOracle, RealCurrentHistoricalBridgeAndUnionActionsMatchDenseAndWorkers) {
  using namespace uwb_imu_pl;
  const auto cfg = researchConfig();
  IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  NavigationState initial; initial.position_world_m = {0,0,1};
  estimator.initialize(initial, cfg.realtime.prior_sigmas);
  addImu(&estimator, cfg.imu.gravity_mps2);
  auto first = estimator.prepareEpoch(batch(cfg, 10000000));
  auto plan = EpochCommitPlan::nominalPlan(first);
  estimator.commitEpoch(std::move(first), plan);
  for(int i=3;i<=4;++i) {
    ImuMeasurement imu; imu.timestamp=TimestampNs(i*5000000);
    imu.specific_force_mps2={0,0,cfg.imu.gravity_mps2}; estimator.ingestImu(imu);
  }
  auto input = batch(cfg, 20000000);
  const auto n = input.measurements.size();
  input.covariance_m2 = .0025 * Eigen::MatrixXd::Identity(n,n) +
      .0001 * Eigen::MatrixXd::Ones(n,n);
  auto tx = estimator.prepareEpoch(input);
  const auto updates = estimator.backendUpdateCount();
  const auto window = estimator.buildIntegrityWindow(tx, IntegrityWindowRequest{});
  auto imu = estimator.buildPendingFactorBlock(tx, tx.imu_group.id);
  auto bridge = estimator.buildPendingFactorBlock(tx, tx.generic_bridge_group.id);
  auto models = HypothesisGenerator().generate(window, tx, ImuFaultSubspaceBuilder().build(tx,imu), bridge);
  std::vector<FaultModeEvidence> evidence;
  for(const auto& h:models.hypotheses) { FaultModeEvidence e; e.hypothesis=h.id; e.plausible=true; evidence.push_back(e); }
  auto actions = HypothesisGenerator().actionsForPlausibleSet(window, tx, &models,
                                                              evidence);
  ASSERT_GT(actions.size(), 10u);
  RankUpdateConfig config; config.materialize_dense_oracle_fields=false;
  config.enable_early_step_gate=false;
  const RankUpdateEvaluator evaluator(config);
  const auto base=evaluator.factorizeOnce(window,actions);
  std::vector<CandidateEvaluation> serial;
  for(const auto& a:actions) serial.push_back(evaluator.evaluate(base,a));
  std::vector<std::future<std::vector<std::pair<std::size_t,CandidateEvaluation>>>> workers;
  for(std::size_t worker=0;worker<4;++worker) workers.push_back(std::async(std::launch::async,[&,worker] {
    std::vector<std::pair<std::size_t,CandidateEvaluation>> out;
    for(std::size_t i=worker;i<actions.size();i+=4) out.emplace_back(i,evaluator.evaluate(base,actions[i]));
    return out;
  }));
  for(auto& worker:workers) for(const auto& item:worker.get()) {
    EXPECT_EQ(item.second.action.id, serial[item.first].action.id);
    EXPECT_EQ(item.second.valid, serial[item.first].valid);
    EXPECT_TRUE(item.second.state_increment.isApprox(serial[item.first].state_increment,1e-9));
  }
  std::size_t valid=0, bridges=0, history=0;
  for(std::size_t i=0;i<actions.size();++i) {
    const auto& c=serial[i]; const auto oracle=DenseCandidateOracle(config).evaluate(window,actions[i]);
    ASSERT_EQ(c.valid,oracle.valid) << i << ": " << c.reason << "/" << oracle.reason;
    if(!c.valid) continue;
    ++valid; bridges += actions[i].bridge_mode != BridgeMode::None;
    history += actions[i].recovery_epoch_begin.has_value();
    EXPECT_LE((c.state_increment-oracle.state_increment).norm(),1e-9);
    const auto pc=c.covarianceTimes(window.protected_state_map.transpose());
    const auto po=oracle.covarianceTimes(window.protected_state_map.transpose());
    EXPECT_LE((pc-po).norm()/std::max(1e-15,po.norm()),1e-7);
    EXPECT_NEAR(c.statistic,oracle.statistic,1e-7);
    EXPECT_LE(std::abs(c.information_logdet-oracle.information_logdet)/std::max(1e-15,std::abs(oracle.information_logdet)),1e-7);
    EXPECT_EQ(c.dof,oracle.dof);
    const auto dc=JointWindowDetector().evaluateCandidate(c,DetectorRiskContext{});
    const auto od=JointWindowDetector().evaluateCandidate(oracle,DetectorRiskContext{});
    EXPECT_EQ(dc.passed,od.passed); EXPECT_NEAR(dc.squared_threshold,od.squared_threshold,1e-9);
    Eigen::Vector3d bc=Eigen::Vector3d::Zero(),bo=bc;
    for(const auto& b:actions[i].added_blocks) if(b.kind==FactorKind::KinematicBridge) {
      const auto bounds=BridgeFactory().uncertainty(tx,cfg.bridge.generic).deterministic_bound;
      bc+=BridgeFactory().propagateBoxMargin(window.protected_state_map,
          c.covarianceTimes(b.jacobian_whitened.transpose()),bounds);
      bo+=BridgeFactory().propagateBoxMargin(window.protected_state_map,
          oracle.covarianceTimes(b.jacobian_whitened.transpose()),bounds);
    }
    EXPECT_LE((bc-bo).norm(),1e-7);
    // An independent residual-row fault fixture exercises the downstream PL
    // formulas for every real action without changing production hypotheses.
    FaultHypothesisV2 fault; fault.id=HypothesisId(1);
    fault.A=Eigen::MatrixXd::Zero(c.rows,1); fault.A(c.rows-1,0)=1;
    fault.p_md_allocation=1e-3; fault.hmi_allocation=1e-6; fault.prior_probability_bound=1e-3;
    std::vector<FaultHypothesisV2> fc{fault},fo{fault};
    const auto pl=ProtectionLevelV2().compute(window,c,dc,&fc,RiskBudgetV2{},bc);
    const auto op=ProtectionLevelV2().compute(window,oracle,od,&fo,RiskBudgetV2{},bo);
    EXPECT_EQ(pl.model_valid,op.model_valid);
    if(pl.model_valid) { EXPECT_LE((pl.pl_xyz_m-op.pl_xyz_m).norm()/std::max(1e-15,op.pl_xyz_m.norm()),1e-7); }
  }
  EXPECT_GT(valid,0u); EXPECT_GT(bridges,0u); EXPECT_GT(history,0u);
  EXPECT_EQ(estimator.backendUpdateCount(),updates);
  estimator.discardEpoch(std::move(tx),{FdeStatus::ModelInvalid,"oracle comparison done",false});
  EXPECT_EQ(estimator.backendUpdateCount(),updates);
}

TEST(GateDNumerics, RankAndConditionThresholdsBothSidesKeepExactDefinitions) {
  using namespace uwb_imu_pl;
  for (bool rank_gate : {false,true}) for (double side : {1-1e-7,1+1e-7}) {
    LinearizedIntegrityWindow w; w.version={1,2,3,4};
    Eigen::MatrixXd h=Eigen::MatrixXd::Zero(3,2);
    h(0,0)=1; h(1,1)=(rank_gate?1e-10:1e-4)*side;
    w.blocks={block(1,h,Eigen::Vector3d::Zero(),w.version),
              block(2,Eigen::Matrix2d::Identity(),Eigen::Vector2d::Zero(),w.version)};
    RankUpdateConfig config; config.materialize_dense_oracle_fields=false;
    config.rank_tolerance=rank_gate?1e-10:1e-12;
    config.max_condition_number=rank_gate?1e12:1e4;
    finalizeIntegrityWindow(&w, config.rank_tolerance,
                            config.max_condition_number);
    ExclusionAction a; a.groups_to_remove={FactorGroupId(2)};
    const RankUpdateEvaluator evaluator(config);
    const auto c=evaluator.evaluate(evaluator.factorizeOnce(w),a);
    EXPECT_EQ(c.valid,side>1) << c.reason;
    const auto oracle=DenseCandidateOracle(config).evaluate(w,a);
    EXPECT_EQ(c.valid,oracle.valid);
    if(c.valid) {
      const auto covariance=c.covarianceTimes(Eigen::Matrix2d::Identity());
      EXPECT_LE((covariance-oracle.covariance).norm()/oracle.covariance.norm(),1e-6);
      EXPECT_LE((c.state_increment-oracle.state_increment).norm(),1e-6);
      EXPECT_NEAR(c.statistic,oracle.statistic,1e-6);
    }
    EXPECT_TRUE(c.diagnostics.near_gate);
    if(side<1) { EXPECT_EQ(c.reason,rank_gate?"candidate rank loss":"candidate condition gate failed"); }
  }
}

TEST(GateDSelection, DenseAndOperatorPathsSelectSameSuccessfulExclusion) {
  using namespace uwb_imu_pl;
  auto w=syntheticWindow(); w.protected_state_map.row(2)=w.protected_state_map.row(0);
  finalizeIntegrityWindow(&w, 1e-10, 1e10);
  ExclusionAction keep; keep.id=ExclusionActionId(1); keep.action_model_id="KEEP_ALL";
  ExclusionAction exclude; exclude.id=ExclusionActionId(2); exclude.action_model_id="UWB_EXCLUSION";
  exclude.groups_to_remove={FactorGroupId(3)}; exclude.covered_units={FaultUnitId(11)};
  exclude.exclusion_cardinality=1;
  RankUpdateConfig cfg; cfg.materialize_dense_oracle_fields=false;
  const RankUpdateEvaluator evaluator(cfg); const auto base=evaluator.factorizeOnce(w,{keep,exclude});
  std::vector<CandidateEvaluation> fast,dense;
  for(const auto& action:{keep,exclude}) {
    fast.push_back(evaluator.evaluate(base,action)); dense.push_back(DenseCandidateOracle(cfg).evaluate(w,action));
    for(auto* c:{&fast.back(),&dense.back()}) {
      const auto detector=JointWindowDetector().evaluateCandidate(*c,DetectorRiskContext{});
      ASSERT_TRUE(c->valid); ASSERT_TRUE(detector.passed);
      FaultHypothesisV2 remaining; remaining.id=HypothesisId(9);
      remaining.A=Eigen::MatrixXd::Zero(c->rows,1); remaining.A(3,0)=1;
      remaining.p_md_allocation=1e-3; remaining.hmi_allocation=1e-6; remaining.prior_probability_bound=1e-3;
      std::vector<FaultHypothesisV2> faults{remaining};
      const auto pl=ProtectionLevelV2().compute(w,*c,detector,&faults,RiskBudgetV2{});
      ASSERT_TRUE(pl.model_valid) << pl.reason;
      c->post_detector_passed=true; c->hpl_m=pl.hpl_m; c->vpl_m=pl.vpl_m;
    }
  }
  FaultHypothesisV2 fault; fault.id=HypothesisId(1); fault.units=exclude.covered_units;
  fault.affected_groups=exclude.groups_to_remove;
  FaultModeEvidence evidence; evidence.hypothesis=fault.id; evidence.plausible=true;
  DetectorResultV2 alarm; alarm.numerically_valid=true; alarm.passed=false;
  const auto f=FdeManager().decide(alarm,{fault},{evidence},&fast,{},RiskBudgetV2{});
  const auto d=FdeManager().decide(alarm,{fault},{evidence},&dense,{},RiskBudgetV2{});
  ASSERT_TRUE(f.commit_allowed); ASSERT_TRUE(d.commit_allowed);
  EXPECT_EQ(f.selected_action->id,exclude.id); EXPECT_EQ(f.selected_action->id,d.selected_action->id);
  EXPECT_EQ(f.status,FdeStatus::SuccessUwbExclusion); EXPECT_EQ(f.status,d.status);
}

TEST(GateDDiagnostics, PreparationExceptionKeepsAttemptTimingAndZeroUpdates) {
  using namespace uwb_imu_pl;
  const auto cfg=researchConfig();
  IncrementalUwbImuEstimator estimator(cfg,Eigen::Vector3d::Zero());
  NavigationState initial; initial.position_world_m={0,0,1};
  estimator.initialize(initial,cfg.realtime.prior_sigmas); addImu(&estimator,cfg.imu.gravity_mps2);
  RealtimeIntegrityPipeline pipeline(&estimator,IntegrityMonitor(cfg.risk,cfg.snapshot.rank_tolerance,cfg.snapshot.max_condition_number));
  auto bad=batch(cfg,10000000); bad.measurements.front().range_m=std::numeric_limits<double>::quiet_NaN();
  const auto before=estimator.backendUpdateCount();
  EXPECT_ANY_THROW(pipeline.processUwbBatch(bad));
  const auto& output=pipeline.lastAttemptOutput();
  EXPECT_EQ(output.diagnostics.input_attempt_id,1u);
  EXPECT_EQ(output.diagnostics.input_timestamp,bad.timestamp);
  EXPECT_EQ(output.diagnostics.status,"EXCEPTION");
  EXPECT_EQ(estimator.backendUpdateCount(),before);
  const auto prepare=std::find_if(output.stage_timings.begin(),output.stage_timings.end(),
      [](const auto& s){return s.stage=="prepare";});
  ASSERT_NE(prepare,output.stage_timings.end()); EXPECT_EQ(prepare->status,"EXCEPTION");
  EXPECT_TRUE(std::isfinite(prepare->wall_ms));
  EXPECT_FALSE(prepare->success);
}

TEST(GateDNumericalCertificate, ClearCandidateAvoidsFinalJacobianSvd) {
  using namespace uwb_imu_pl;
  const auto window = syntheticWindow();
  ExclusionAction action;
  action.id = ExclusionActionId(77);
  action.groups_to_remove = {FactorGroupId(3)};
  RankUpdateConfig config;
  config.materialize_dense_oracle_fields = false;
  config.max_linearization_step_norm = 10.0;
  const RankUpdateEvaluator evaluator(config);
  const auto candidate = evaluator.evaluate(
      evaluator.factorizeOnce(window, {action}), action);
  const auto oracle = DenseCandidateOracle(config).evaluate(window, action);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  EXPECT_TRUE(candidate.diagnostics.certificate_passed);
  EXPECT_FALSE(candidate.exact_slow_path);
  EXPECT_EQ(candidate.diagnostics.condition_value_kind, "CERTIFIED_BOUNDS");
  EXPECT_TRUE(std::isnan(candidate.condition_number));
  EXPECT_LT(candidate.diagnostics.condition_upper_bound,
            config.max_condition_number);
  EXPECT_TRUE(candidate.state_increment.isApprox(oracle.state_increment, 1e-9));
  EXPECT_NEAR(candidate.statistic, oracle.statistic, 1e-9);
}

TEST(GateDProtectionLevel, SharedModesUseOneCombinedCovarianceSolve) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  RankUpdateConfig config;
  config.materialize_dense_oracle_fields = false;
  config.max_linearization_step_norm = 10.0;
  const RankUpdateEvaluator evaluator(config);
  const auto base = evaluator.factorizeOnce(window);
  auto candidate = evaluator.evaluate(base, ExclusionAction{});
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  const auto detector = JointWindowDetector().evaluateCandidate(
      candidate, DetectorRiskContext{});
  ASSERT_TRUE(detector.passed);
  Eigen::MatrixXd mode = Eigen::MatrixXd::Zero(candidate.rows, 1);
  mode(4, 0) = 1.0;
  FaultHypothesisV2 first;
  first.id = HypothesisId(101);
  first.modes = {FaultModeId(501)};
  first.A = mode;
  first.p_md_allocation = 1e-3;
  first.hmi_allocation = 1e-6;
  first.prior_probability_bound = 1e-3;
  auto second = first;
  second.id = HypothesisId(102);
  std::vector<FaultHypothesisV2> legacy{first, second};
  std::vector<FaultHypothesisV2> optimized{first, second};
  const auto reference = ProtectionLevelV2().compute(
      window, candidate, detector, &legacy, RiskBudgetV2{});
  ProtectionLevelSharedContext shared;
  shared.mode_maps.emplace(501, mode);
  const auto result = ProtectionLevelV2().computeShared(
      window, &candidate, detector, &optimized, shared, RiskBudgetV2{});
  ASSERT_TRUE(reference.model_valid) << reference.reason;
  ASSERT_TRUE(result.model_valid) << result.reason;
  EXPECT_TRUE(result.pl_xyz_m.isApprox(reference.pl_xyz_m, 1e-9));
  EXPECT_EQ(candidate.diagnostics.covariance_solve_count, 1u);
  EXPECT_TRUE(optimized[0].monitorability.protected_slopes.isApprox(
      legacy[0].monitorability.protected_slopes, 1e-9));
}

TEST(GateDWorkerPool, StaticSchedulingExceptionBarrierAndScratchReuse) {
  using namespace uwb_imu_pl;
  CandidateWorkerPool pool(4);
  std::vector<std::size_t> owners(20, 99);
  pool.run(owners.size(), 4,
      [&](std::size_t index, std::size_t worker, RankUpdateScratch& scratch) {
        owners[index] = worker;
        scratch.update_signs = Eigen::Vector2d::Ones();
      });
  for (std::size_t i = 0; i < owners.size(); ++i) EXPECT_EQ(owners[i], i % 4);
  std::atomic<int> reused{0};
  pool.run(4, 4, [&](std::size_t, std::size_t, RankUpdateScratch& scratch) {
    if (scratch.update_signs.size() == 2) ++reused;
  });
  EXPECT_EQ(reused.load(), 4);
  EXPECT_THROW(pool.run(12, 4,
      [](std::size_t index, std::size_t, RankUpdateScratch&) {
        if (index == 3) throw std::runtime_error("worker failure");
      }), std::runtime_error);
  std::atomic<int> completed{0};
  pool.run(8, 1, [&](std::size_t, std::size_t worker, RankUpdateScratch&) {
    EXPECT_EQ(worker, 0u);
    ++completed;
  });
  EXPECT_EQ(completed.load(), 8);
}

TEST(GateDStatistics, ExactBitKeyProducesCacheHitWithoutQuantization) {
  using namespace uwb_imu_pl;
  const auto before = StatisticalBoundsCache::stats();
  const double first = StatisticalBoundsCache::chiSquaredThreshold(123, 0.123456789);
  const auto middle = StatisticalBoundsCache::stats();
  const double second = StatisticalBoundsCache::chiSquaredThreshold(123, 0.123456789);
  const auto after = StatisticalBoundsCache::stats();
  EXPECT_DOUBLE_EQ(first, second);
  EXPECT_EQ(middle.misses, before.misses + 1);
  EXPECT_EQ(after.hits, middle.hits + 1);
}

TEST(GateDHistoryCache, FrozenFactorContentHitsAndMutationMisses) {
  using namespace uwb_imu_pl;
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  (void)estimator.buildIntegrityWindow(transaction, IntegrityWindowRequest{});
  const auto before = estimator.cacheAudit();
  const auto first = estimator.buildPendingFactorBlock(
      transaction, transaction.imu_group.id);
  const auto hit = estimator.cacheAudit();
  EXPECT_GT(hit.factor_block_hits, before.factor_block_hits);
  auto changed = transaction;
  changed.imu_group.model_id += ":changed";
  const auto second = estimator.buildPendingFactorBlock(
      changed, changed.imu_group.id);
  const auto miss = estimator.cacheAudit();
  EXPECT_GT(miss.factor_block_misses, hit.factor_block_misses);
  EXPECT_TRUE(first.jacobian_whitened.isApprox(second.jacobian_whitened, 0));
  estimator.discardEpoch(std::move(transaction),
      {FdeStatus::ModelInvalid, "cache test complete", false});
}

TEST(IntegrityV2Numerics, FrozenWindowSharesOneBaseSvdAndLlt) {
  uwb_imu_pl::NumericalWorkCounters::reset();
  auto window = syntheticWindow();
  uwb_imu_pl::DetectorRiskContext detector_risk;
  detector_risk.max_condition_number = 1e10;
  const auto detector = uwb_imu_pl::JointWindowDetector().evaluate(
      window, detector_risk);
  std::vector<uwb_imu_pl::FaultHypothesisV2> hypotheses;
  (void)uwb_imu_pl::HypothesisEvidenceEvaluator().evaluateAll(
      window, &hypotheses, detector.squared_threshold);
  const auto base = uwb_imu_pl::RankUpdateEvaluator().factorizeOnce(window);
  ASSERT_TRUE(base.valid) << base.reason;
  uwb_imu_pl::ExclusionAction keep;
  keep.id = uwb_imu_pl::ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  const auto unchanged = uwb_imu_pl::RankUpdateEvaluator().evaluate(base, keep);
  ASSERT_TRUE(unchanged.valid) << unchanged.reason;
  EXPECT_EQ(unchanged.diagnostics.numerical_path, "KEEP_BASE_CANONICAL");
  EXPECT_NEAR(unchanged.statistic, window.numerics->statistic, 1e-12);
  const auto counts = uwb_imu_pl::NumericalWorkCounters::snapshot();
  EXPECT_EQ(counts.base_svd, 1u);
  EXPECT_EQ(counts.base_llt, 1u);
  EXPECT_EQ(counts.base_state_solves, 2u);
  EXPECT_EQ(counts.llt_state_solve_calls, 1u);
  EXPECT_EQ(counts.svd_state_solve_calls, 1u);
  EXPECT_EQ(counts.detector_reference_qr, 0u);
  EXPECT_EQ(counts.candidate_reference_svd, 0u);
}

TEST(IntegrityV2Numerics, RejectsReuseAfterAnyFrozenIdentityMutation) {
  using namespace uwb_imu_pl;
  const RankUpdateEvaluator evaluator;
  for (int mutation = 0; mutation < 8; ++mutation) {
    auto window = syntheticWindow();
    const auto base = evaluator.factorizeOnce(window);
    ASSERT_TRUE(base.valid) << mutation << ": " << base.reason;
    switch (mutation) {
      case 0: ++window.version.graph_version; break;
      case 1: ++window.version.ordering_version; break;
      case 2: ++window.version.noise_model_version; break;
      case 3: ++window.version.linpoint_version; break;
      case 4: window.H(0, 0) = std::nextafter(window.H(0, 0), 10.0); break;
      case 5: window.protected_state_map(0, 0) = 2.0; break;
      case 6: window.blocks.front().covariance(0, 0) = 2.0; break;
      case 7:
        window.state_layout.push_back(
            {9, {gtsam::Symbol('x', 9)}, 0, 1, false});
        break;
    }
    const auto candidate = evaluator.evaluate(base, ExclusionAction{});
    EXPECT_FALSE(candidate.valid) << mutation;
    if (mutation < 4) {
      EXPECT_EQ(candidate.reason, "candidate frozen block/version is invalid");
    } else {
      EXPECT_NE(candidate.diagnostics.fallback_reason.find("stale frozen numerics"),
                std::string::npos) << mutation << ": " << candidate.reason;
    }
  }
}

TEST(IntegrityV2Risk, EqualAllocationNeverOvershootsAndRealExcessFails) {
  uwb_imu_pl::RiskBudgetV2 risk;
  const std::size_t count = 1386;
  std::vector<uwb_imu_pl::FaultHypothesisV2> hypotheses(count);
  const double available =
      uwb_imu_pl::conservativeRemainingHypothesisRisk(risk);
  const double allocation =
      uwb_imu_pl::conservativeEqualRiskAllocation(available, count);
  for (auto& hypothesis : hypotheses) hypothesis.hmi_allocation = allocation;
  const auto closed = uwb_imu_pl::auditRiskBudget(risk, hypotheses);
  EXPECT_TRUE(closed.valid);
  EXPECT_GE(closed.margin, 0.0);
  hypotheses.front().hmi_allocation += 1e-12;
  const auto exceeded = uwb_imu_pl::auditRiskBudget(risk, hypotheses);
  EXPECT_FALSE(exceeded.valid);
  EXPECT_LT(exceeded.margin, 0.0);
}

TEST(IntegrityV2ImuOracle, AnalyticProductionHasZeroReintegrationAcrossMotions) {
  using namespace uwb_imu_pl;
  struct Motion {
    double duration_s;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_rate;
    Eigen::Vector3d accel_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Quaterniond orientation;
  };
  const std::vector<Motion> motions{
      {0.02, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
      {0.05, Eigen::Vector3d(0.4, -0.2, 0.1), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
      {0.10, Eigen::Vector3d(0.2, 0.1, 0.0), Eigen::Vector3d(0.7, -0.3, 0.2),
       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
      {0.05, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d(0.0, 0.0, 0.4), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
      {0.075, Eigen::Vector3d(0.1, 0.2, -0.1), Eigen::Vector3d(0.2, 0.1, 0.0),
       Eigen::Vector3d(0.2, -0.15, 0.3), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(),
       Eigen::Quaterniond(Eigen::AngleAxisd(0.35, Eigen::Vector3d(1, 2, 3).normalized()))},
      {0.05, Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::Vector3d::Zero(),
       Eigen::Vector3d(0.1, -0.05, 0.2), Eigen::Vector3d(0.03, -0.02, 0.01),
       Eigen::Vector3d(0.002, -0.003, 0.001), Eigen::Quaterniond::Identity()},
      // Long-pending oracle-cost observation: production remains zero, while
      // the explicit oracle is still exactly six central differences.
      {0.50, Eigen::Vector3d(0.2, 0.2, 0.0), Eigen::Vector3d(0.1, 0.0, 0.0),
       Eigen::Vector3d(0.1, 0.2, -0.1), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
  };
  for (const auto& motion : motions) {
    auto config = researchConfig();
    IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
    NavigationState initial;
    initial.timestamp = TimestampNs(0);
    initial.position_world_m = {0, 0, 1};
    initial.velocity_world_mps = motion.velocity;
    initial.accel_bias_mps2 = motion.accel_bias;
    initial.gyro_bias_radps = motion.gyro_bias;
    initial.q_world_body = motion.orientation;
    estimator.initialize(initial, config.realtime.prior_sigmas);
    const int samples = static_cast<int>(std::llround(motion.duration_s / 0.005));
    for (int sample = 0; sample <= samples; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(sample + 1);
      imu.timestamp = TimestampNs(sample * 5000000LL);
      imu.specific_force_mps2 = motion.acceleration + motion.accel_bias +
          Eigen::Vector3d(0, 0, config.imu.gravity_mps2);
      imu.angular_velocity_radps = motion.angular_rate + motion.gyro_bias;
      estimator.ingestImu(imu);
    }
    auto input = batch(config, static_cast<std::int64_t>(
        std::llround(motion.duration_s * 1e9)));
    auto transaction = estimator.prepareEpoch(input);
    const auto imu_block = estimator.buildPendingFactorBlock(
        transaction, transaction.imu_group.id);
    NumericalWorkCounters::reset();
    const auto analytic = ImuFaultSubspaceBuilder().buildAnalytic(
        transaction, imu_block);
    EXPECT_TRUE(analytic.analytic_input_valid);
    EXPECT_TRUE(analytic.analytic_computation_valid);
    EXPECT_FALSE(analytic.oracle_executed);
    EXPECT_EQ(NumericalWorkCounters::snapshot().imu_oracle_reintegrations, 0u);
    const auto verified = ImuFaultSubspaceBuilder()
        .verifyFiniteDifferenceOracle(transaction, imu_block);
    EXPECT_TRUE(verified.oracle_verified) << motion.duration_s << ": "
                                         << verified.oracle_relative_error;
    EXPECT_EQ(verified.oracle_reintegrations, 12u);
    EXPECT_EQ(NumericalWorkCounters::snapshot().imu_oracle_reintegrations, 12u);
    estimator.discardEpoch(std::move(transaction),
        {FdeStatus::ModelInvalid, "oracle development test", false});
  }
}

TEST(IntegrityV2ImuOracle, FiniteDifferenceSweepMatchesAnalyticAcrossStepSizes) {
  using namespace uwb_imu_pl;
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d(0.1, 0.2, 0.3));
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  initial.velocity_world_mps = {0.2, 0.1, 0.0};
  initial.q_world_body =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d(1, 2, 3).normalized()));
  estimator.initialize(initial, config.realtime.prior_sigmas);
  for (int sample = 0; sample <= 10; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(sample + 1);
    imu.timestamp = TimestampNs(sample * 5000000LL);
    imu.specific_force_mps2 = {0.3, -0.1, config.imu.gravity_mps2 + 0.05};
    imu.angular_velocity_radps = {0.1, -0.2, 0.15};
    estimator.ingestImu(imu);
  }
  auto input = batch(config, 55000000);
  auto transaction = estimator.prepareEpoch(input);
  const auto imu_block = estimator.buildPendingFactorBlock(
      transaction, transaction.imu_group.id);
  const auto sweep = ImuFaultSubspaceBuilder().verifyFiniteDifferenceSweep(
      transaction, imu_block);
  EXPECT_TRUE(sweep.sweep_executed);
  EXPECT_TRUE(sweep.sweep_verified) << sweep.sweep_worst_relative_error;
  EXPECT_EQ(sweep.sweep_epsilons.size(), 5u);
  EXPECT_EQ(sweep.sweep_relative_errors.size(), 5u);
  EXPECT_EQ(sweep.sweep_reintegrations, 60u);
  for (const double error : sweep.sweep_relative_errors) {
    EXPECT_TRUE(std::isfinite(error));
  }
  EXPECT_TRUE(std::isfinite(sweep.sweep_worst_relative_error));
  estimator.discardEpoch(std::move(transaction),
      {FdeStatus::ModelInvalid, "sweep development test", false});
}

TEST(IntegrityV2ImuOracle, InvalidAnalyticInputIsReportedWithoutOracle) {
  const uwb_imu_pl::EpochTransaction transaction;
  const uwb_imu_pl::LinearizedFactorBlock block;
  const auto result = uwb_imu_pl::ImuFaultSubspaceBuilder().buildAnalytic(
      transaction, block);
  EXPECT_FALSE(result.analytic_input_valid);
  EXPECT_FALSE(result.analytic_computation_valid);
  EXPECT_FALSE(result.oracle_executed);
  EXPECT_FALSE(result.analytic_reason.empty());
  const auto oracle =
      uwb_imu_pl::ImuFaultSubspaceBuilder().verifyFiniteDifferenceOracle(
          transaction, block);
  EXPECT_FALSE(oracle.oracle_executed);
  EXPECT_EQ(oracle.oracle_reintegrations, 0u);
}

// ---------------------------------------------------------------------------
// B2 (§5.7): one registry for order=1 and order=2.  Family classification,
// shared-parameter fail-closed behaviour, and on-demand cross blocks.
// ---------------------------------------------------------------------------

TEST(B2Registry, PairFamilySupportClassifiesFamilies) {
  using namespace uwb_imu_pl;
  FaultModeBasis uwb;
  uwb.sensor = SensorType::Uwb;
  uwb.physical_source_id = "uwb:1";
  FaultModeBasis uwb_other = uwb;
  uwb_other.physical_source_id = "uwb:2";
  FaultModeBasis accel;
  accel.sensor = SensorType::ImuAccelerometer;
  accel.physical_source_id = "imu_accel:0:interval:3";
  FaultModeBasis gyro = accel;
  gyro.sensor = SensorType::ImuGyroscope;

  EXPECT_EQ(pairFamilySupport(uwb, uwb_other), PairFamilySupport::Unsupported);
  EXPECT_EQ(pairFamilySupport(accel, gyro), PairFamilySupport::Unsupported);
  EXPECT_EQ(pairFamilySupport(uwb, accel), PairFamilySupport::Independent);
  EXPECT_EQ(pairFamilySupport(uwb, gyro), PairFamilySupport::Independent);
  FaultModeBasis same_source_imu = accel;
  same_source_imu.physical_source_id = uwb.physical_source_id;
  EXPECT_EQ(pairFamilySupport(uwb, same_source_imu),
            PairFamilySupport::SharedParameters);
  EXPECT_EQ(toString(PairFamilySupport::Unsupported), "UNSUPPORTED");
}

TEST(B2Registry, SharedDirectionFailsClosedInTheConcatenationGuard) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  // Two modes touching the same factor group with identical raw maps: the
  // stacked map is rank deficient, so A12 = [A1, A2] would double count.
  FaultModeBasis left;
  left.id = FaultModeId(1);
  left.sensor = SensorType::Uwb;
  left.parameter_dimension = 1;
  left.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Ones();
  FaultModeBasis right = left;
  right.id = FaultModeId(2);
  right.sensor = SensorType::ImuAccelerometer;
  const std::vector<FaultModeBasis> modes{left, right};
  std::string reason;
  EXPECT_FALSE(hypothesisParametersIndependent(
      modes, {left.id, right.id}, &reason));
  EXPECT_NE(reason.find("shared direction"), std::string::npos) << reason;

  // Disjoint groups (UWB group 2 vs IMU group 1) are independent.
  FaultModeBasis imu;
  imu.id = FaultModeId(3);
  imu.sensor = SensorType::ImuAccelerometer;
  imu.parameter_dimension = 1;
  imu.raw_group_maps[FactorGroupId(1)] = Eigen::Vector3d(0.0, 1.0, 0.0);
  EXPECT_TRUE(hypothesisParametersIndependent(
      {left, imu}, {left.id, imu.id}, &reason)) << reason;

  // Unsupported family is refused with its own reason.
  FaultModeBasis uwb_pair = left;
  uwb_pair.id = FaultModeId(4);
  EXPECT_FALSE(hypothesisParametersIndependent(
      {left, uwb_pair}, {left.id, uwb_pair.id}, &reason));
  EXPECT_NE(reason.find("unsupported pair family"), std::string::npos) << reason;
}

TEST(B2Registry, SharedParametersAreNotMonitoredByTheEvidencePath) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  FaultModeBasis left;
  left.id = FaultModeId(1);
  left.sensor = SensorType::Uwb;
  left.parameter_dimension = 1;
  left.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Ones();
  FaultModeBasis right = left;
  right.id = FaultModeId(2);
  right.sensor = SensorType::ImuAccelerometer;
  const std::vector<FaultModeBasis> modes{left, right};
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(1);
  hypothesis.modes = {left.id, right.id};
  std::vector<FaultHypothesisV2> hypotheses{hypothesis};
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      window, modes, &hypotheses, 100.0);
  ASSERT_EQ(evidence.size(), 1u);
  EXPECT_FALSE(hypotheses[0].monitored);
  EXPECT_NE(hypotheses[0].monitorability.reason.find("shared direction"),
            std::string::npos)
      << hypotheses[0].monitorability.reason;
  EXPECT_FALSE(evidence[0].plausible);
}

TEST(B2Registry, CrossBlocksAreRequestedOnlyWhereNeeded) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  FaultModeBasis first;
  first.id = FaultModeId(1);
  first.sensor = SensorType::Uwb;
  first.parameter_dimension = 1;
  first.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Ones();
  FaultModeBasis second;
  second.id = FaultModeId(2);
  second.sensor = SensorType::Uwb;
  second.parameter_dimension = 1;
  // Group 3 carries two rows in the synthetic fixture: the raw map must match
  // the group's native row count or the mode is rejected as invalid.
  second.raw_group_maps[FactorGroupId(3)] =
      (Eigen::MatrixXd(2, 1) << 1.0, 0.0).finished();
  FaultModeBasis imu;
  imu.id = FaultModeId(3);
  imu.sensor = SensorType::ImuAccelerometer;
  imu.parameter_dimension = 1;
  imu.raw_group_maps[FactorGroupId(1)] =
      (Eigen::MatrixXd(2, 1) << 0.0, 1.0).finished();
  const std::vector<FaultModeBasis> modes{first, second, imu};

  // Order=1 only: exactly one self block per evaluated hypothesis, and the
  // all-mode Gram is never materialized.
  {
    std::vector<FaultHypothesisV2> singles(3);
    for (std::size_t index = 0; index < 3; ++index) {
      singles[index].id = HypothesisId(index + 1);
      singles[index].modes = {modes[index].id};
      singles[index].prior_probability_bound = 1e-4;
      singles[index].p_md_allocation = 1e-3;
    }
    const auto before = NumericalWorkCounters::snapshot();
    (void)HypothesisEvidenceEvaluator().evaluateAll(window, modes, &singles,
                                                    100.0);
    const auto after = NumericalWorkCounters::snapshot();
    EXPECT_EQ(after.fault_cross_blocks - before.fault_cross_blocks, 3u)
        << "one self block per hypothesis, no pair cross terms";
    EXPECT_EQ(after.all_mode_gram_columns, 0u)
        << "the unconditional all-mode Gram must not be built";
  }
  // Order=2 enabled: the pair cross blocks appear, one per referenced pair.
  {
    std::vector<FaultHypothesisV2> pairs(2);
    pairs[0].id = HypothesisId(10);
    pairs[0].modes = {first.id, imu.id};
    pairs[0].prior_probability_bound = 1e-8;
    pairs[0].p_md_allocation = 1e-3;
    pairs[1].id = HypothesisId(11);
    pairs[1].modes = {second.id, imu.id};
    pairs[1].prior_probability_bound = 1e-8;
    pairs[1].p_md_allocation = 1e-3;
    const auto before = NumericalWorkCounters::snapshot();
    (void)HypothesisEvidenceEvaluator().evaluateAll(window, modes, &pairs,
                                                    100.0);
    const auto after = NumericalWorkCounters::snapshot();
    // self(first) + self(second) + self(imu) + cross(first,imu) +
    // cross(second,imu); no cross(first,second) because no hypothesis needs it.
    EXPECT_EQ(after.fault_cross_blocks - before.fault_cross_blocks, 5u);
  }
}

// ---------------------------------------------------------------------------
// B2 Stage 2 (roadmap R2): compact mode descriptors, capacity-bounded buffers,
// and the counted padded fallback.  Storage form must never change results.
// ---------------------------------------------------------------------------
namespace {

struct B2CompactFixture {
  uwb_imu_pl::LinearizedIntegrityWindow window;
  std::vector<uwb_imu_pl::FaultModeBasis> modes;
  std::vector<uwb_imu_pl::FaultHypothesisV2> hypotheses;
};

B2CompactFixture makeCompactFixture() {
  using namespace uwb_imu_pl;
  B2CompactFixture fixture;
  fixture.window = syntheticWindow();
  finalizeIntegrityWindow(&fixture.window, 1e-10, 1e10);
  FaultModeBasis first;
  first.id = FaultModeId(1);
  first.sensor = SensorType::Uwb;
  first.parameter_dimension = 1;
  first.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Ones();
  FaultModeBasis imu;
  imu.id = FaultModeId(2);
  imu.sensor = SensorType::ImuAccelerometer;
  imu.parameter_dimension = 1;
  imu.raw_group_maps[FactorGroupId(1)] =
      (Eigen::MatrixXd(2, 1) << 0.0, 1.0).finished();
  fixture.modes = {first, imu};
  FaultHypothesisV2 pair;
  pair.id = HypothesisId(1);
  pair.modes = {first.id, imu.id};
  pair.prior_probability_bound = 1e-8;
  pair.p_md_allocation = 1e-3;
  FaultHypothesisV2 single;
  single.id = HypothesisId(2);
  single.modes = {first.id};
  single.prior_probability_bound = 1e-4;
  single.p_md_allocation = 1e-3;
  fixture.hypotheses = {pair, single};
  return fixture;
}

}  // namespace

TEST(B2Compact, CompactPathAllocatesNoPaddedModeBlocks) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  const auto before = NumericalWorkCounters::snapshot();
  std::shared_ptr<const FrozenHypothesisNumerics> shared;
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      fixture.window, fixture.modes, &fixture.hypotheses, 100.0, &shared);
  const auto after = NumericalWorkCounters::snapshot();
  ASSERT_EQ(evidence.size(), 2u);
  ASSERT_TRUE(shared);
  // The compact path stores only the rows the modes touch: 3 (UWB group 2)
  // plus 2 (IMU group 1) rows, never window.H.rows() per mode.
  EXPECT_EQ(after.compact_mode_rows - before.compact_mode_rows, 5u);
  EXPECT_EQ(after.compact_mode_columns - before.compact_mode_columns, 2u);
  EXPECT_EQ(after.mode_dense_allocations - before.mode_dense_allocations, 0u)
      << "the compact path must not pad modes across the window rows";
  EXPECT_EQ(after.compact_capacity_fallbacks - before.compact_capacity_fallbacks,
            0u);
  EXPECT_TRUE(shared->compact_mode_used);
  EXPECT_FALSE(shared->compact_capacity_exceeded);
  EXPECT_LT(shared->compact_rows,
            static_cast<std::size_t>(fixture.window.H.rows()) *
                fixture.modes.size());
}

TEST(B2Compact, CapacityExhaustionFallsBackWithIdenticalResults) {
  using namespace uwb_imu_pl;
  auto compact_fixture = makeCompactFixture();
  std::shared_ptr<const FrozenHypothesisNumerics> compact_shared;
  const auto compact_evidence = HypothesisEvidenceEvaluator().evaluateAll(
      compact_fixture.window, compact_fixture.modes,
      &compact_fixture.hypotheses, 100.0, &compact_shared);

  auto padded_fixture = makeCompactFixture();
  HypothesisEvaluationConfig config;
  config.compact_capacity.max_total_compact_rows = 1;  // force the fallback
  std::shared_ptr<const FrozenHypothesisNumerics> padded_shared;
  const auto before = NumericalWorkCounters::snapshot();
  const auto padded_evidence = HypothesisEvidenceEvaluator(config).evaluateAll(
      padded_fixture.window, padded_fixture.modes, &padded_fixture.hypotheses,
      100.0, &padded_shared);
  const auto after = NumericalWorkCounters::snapshot();
  ASSERT_EQ(compact_evidence.size(), padded_evidence.size());
  ASSERT_TRUE(padded_shared);
  EXPECT_TRUE(padded_shared->compact_capacity_exceeded);
  EXPECT_FALSE(padded_shared->compact_mode_used);
  EXPECT_EQ(after.compact_capacity_fallbacks - before.compact_capacity_fallbacks,
            1u);
  // One padded block per valid mode, each across all window rows.
  EXPECT_EQ(after.mode_dense_allocations - before.mode_dense_allocations, 2u);
  EXPECT_EQ(after.mode_dense_allocation_rows - before.mode_dense_allocation_rows,
            static_cast<std::uint64_t>(2 * padded_fixture.window.H.rows()));
  // Identical results: the storage form is not allowed to change the numbers.
  for (std::size_t index = 0; index < compact_evidence.size(); ++index) {
    EXPECT_EQ(compact_evidence[index].all_in_statistic,
              padded_evidence[index].all_in_statistic);
    EXPECT_EQ(compact_evidence[index].conditioned_statistic,
              padded_evidence[index].conditioned_statistic);
    EXPECT_EQ(compact_evidence[index].log_evidence,
              padded_evidence[index].log_evidence);
    EXPECT_EQ(compact_evidence[index].plausible, padded_evidence[index].plausible);
    EXPECT_EQ(compact_evidence[index].estimated_fault.size(),
              padded_evidence[index].estimated_fault.size());
    if (compact_evidence[index].estimated_fault.size() ==
        padded_evidence[index].estimated_fault.size()) {
      EXPECT_EQ((compact_evidence[index].estimated_fault -
                 padded_evidence[index].estimated_fault).cwiseAbs().maxCoeff(),
                0.0);
    }
    EXPECT_EQ(compact_evidence[index].fault_gram.rows(),
              padded_evidence[index].fault_gram.rows());
    if (compact_evidence[index].fault_gram.rows() ==
            padded_evidence[index].fault_gram.rows() &&
        compact_evidence[index].fault_gram.cols() ==
            padded_evidence[index].fault_gram.cols()) {
      EXPECT_EQ((compact_evidence[index].fault_gram -
                 padded_evidence[index].fault_gram).cwiseAbs().maxCoeff(),
                0.0);
    }
  }
}

TEST(B2Compact, HypothesisDimensionCapRefusesFailClosed) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  HypothesisEvaluationConfig config;
  config.compact_capacity.max_hypothesis_dimension = 0;
  const auto before = NumericalWorkCounters::snapshot();
  const auto evidence = HypothesisEvidenceEvaluator(config).evaluateAll(
      fixture.window, fixture.modes, &fixture.hypotheses, 100.0);
  const auto after = NumericalWorkCounters::snapshot();
  ASSERT_EQ(evidence.size(), 2u);
  EXPECT_EQ(after.hypothesis_capacity_refusals -
                before.hypothesis_capacity_refusals,
            2u);
  for (std::size_t index = 0; index < evidence.size(); ++index) {
    EXPECT_FALSE(fixture.hypotheses[index].monitored);
    EXPECT_FALSE(evidence[index].plausible);
    EXPECT_NE(fixture.hypotheses[index].monitorability.reason.find("capacity"),
              std::string::npos)
        << fixture.hypotheses[index].monitorability.reason;
  }
}

TEST(B2Compact, LegacyMappedRouteCountsPaddedAllocations) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  HypothesisEvaluationConfig config;
  config.enable_low_dim_batch = false;  // route through evaluateMapped
  const auto before = NumericalWorkCounters::snapshot();
  const auto evidence = HypothesisEvidenceEvaluator(config).evaluateAll(
      fixture.window, fixture.modes, &fixture.hypotheses, 100.0);
  const auto after = NumericalWorkCounters::snapshot();
  EXPECT_EQ(evidence.size(), 2u);
  // The legacy route pads every mode across the window rows, and that cost is
  // visible in the counters instead of being implicit.
  EXPECT_EQ(after.mode_dense_allocations - before.mode_dense_allocations, 2u);
  EXPECT_EQ(after.compact_mode_rows - before.compact_mode_rows, 0u);
}

// ---------------------------------------------------------------------------
// B2 Stage 3 (§5.8): exact traversal, grouped envelopes with inclusion proofs,
// dominance verification, and the fail-closed coverage certificate.
// ---------------------------------------------------------------------------
TEST(B2Coverage, ExactTraversalCertifiesEveryTask) {
  using namespace uwb_imu_pl;
  const auto fixture = makeCompactFixture();
  const auto certificate = buildExactCoverageCertificate(fixture.modes);
  EXPECT_TRUE(certificate.complete);
  EXPECT_EQ(certificate.exact_count, fixture.modes.size());
  EXPECT_EQ(certificate.enveloped_count, 0u);
  EXPECT_EQ(certificate.uncovered_count, 0u);
  for (const auto& mode : fixture.modes) {
    EXPECT_EQ(coverageLabelFor(certificate, mode.id), CoverageLabel::Exact);
    EXPECT_EQ(coverageEnvelopeIdFor(certificate, mode.id), 0u);
  }
  EXPECT_EQ(std::string(toString(CoverageLabel::UpperEnvelope)), "UPPER_ENVELOPE");
}

TEST(B2Coverage, GroupedEnvelopeDischargesProofAndDominance) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // A group mode on the same factor group with a scaled map spans the leaf
  // exactly: A_leaf = A_group * 2.
  FaultModeBasis group;
  group.id = FaultModeId(10);
  group.sensor = SensorType::Uwb;
  group.parameter_dimension = 1;
  group.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Constant(0.5);
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[0]}, {group});
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  const auto& envelope = certificate.envelopes.front();
  EXPECT_TRUE(envelope.accepted) << envelope.reason;
  ASSERT_EQ(envelope.proofs.size(), 1u);
  EXPECT_TRUE(envelope.proofs.front().verified);
  EXPECT_LT(envelope.proofs.front().relative_residual, 1e-12);
  EXPECT_NEAR(envelope.proofs.front().transform(0, 0), 2.0, 1e-12);
  EXPECT_TRUE(envelope.dominant) << envelope.dominance_margin;
  EXPECT_GE(envelope.dominance_margin, -1e-9);
  EXPECT_EQ(certificate.enveloped_count, 1u);
  EXPECT_TRUE(certificate.complete);
  EXPECT_EQ(coverageLabelFor(certificate, fixture.modes[0].id),
            CoverageLabel::UpperEnvelope);
  EXPECT_EQ(coverageEnvelopeIdFor(certificate, fixture.modes[0].id), 1u);
}

TEST(B2Coverage, MismatchedModesAreRejectedByTheInclusionProof) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // The group touches a different factor group: no transform can reproduce the
  // leaf map, so the envelope must be rejected and the leaf stays EXACT.
  FaultModeBasis group;
  group.id = FaultModeId(10);
  group.sensor = SensorType::Uwb;
  group.parameter_dimension = 1;
  group.raw_group_maps[FactorGroupId(3)] =
      (Eigen::MatrixXd(2, 1) << 1.0, 0.0).finished();
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[0]}, {group});
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  const auto& envelope = certificate.envelopes.front();
  EXPECT_FALSE(envelope.accepted);
  EXPECT_NE(envelope.reason.find("inclusion proof"), std::string::npos)
      << envelope.reason;
  EXPECT_EQ(certificate.exact_count, 1u);
  EXPECT_EQ(coverageLabelFor(certificate, fixture.modes[0].id),
            CoverageLabel::Exact);
}

TEST(B2Coverage, UnsupportedFamilyCannotFormAnEnvelope) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // A UWB group cannot span an IMU leaf (different factor groups and row
  // counts): the proof fails and no label is promoted.
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[1]}, {fixture.modes[0]});
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  EXPECT_FALSE(certificate.envelopes.front().accepted);
  EXPECT_EQ(coverageLabelFor(certificate, fixture.modes[1].id),
            CoverageLabel::Exact);
}

TEST(B2Coverage, IncompleteCoverageMakesProtectedOutputUnavailable) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // Capacity exhaustion: no envelope may be built, so the certificate can no
  // longer vouch for the window.
  CoverageCapacity capacity;
  capacity.max_envelopes = 0;
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, fixture.modes, {fixture.modes[0]}, capacity);
  EXPECT_TRUE(certificate.capacity_exceeded);
  EXPECT_FALSE(certificate.complete);
  EXPECT_NE(certificate.reason.find("protected output unavailable"),
            std::string::npos)
      << certificate.reason;

  // An unusable mode map is UNCOVERED and also makes coverage incomplete.
  auto broken_window = fixture.window;
  FaultModeBasis broken;
  broken.id = FaultModeId(20);
  broken.sensor = SensorType::ImuGyroscope;
  broken.parameter_dimension = 1;
  broken.raw_group_maps[FactorGroupId(999)] = Eigen::Vector3d::Ones();
  const auto uncovered = buildGroupedCoverageCertificate(
      broken_window, {fixture.modes[0], broken}, {fixture.modes[0]});
  EXPECT_FALSE(uncovered.complete);
  ASSERT_EQ(uncovered.uncovered_modes.size(), 1u);
  EXPECT_EQ(uncovered.uncovered_modes.front().value(), 20u);
  EXPECT_EQ(coverageLabelFor(uncovered, broken.id), CoverageLabel::Uncovered);
}

TEST(B2Coverage, DominanceRejectsAnUnderCoveringEnvelope) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // A degenerate group map can satisfy a loose inclusion tolerance while
  // claiming the leaf is unmonitored.  The dominance obligation must catch it:
  // the envelope bound (zero) is below the leaf's exact bound, so the envelope
  // is rejected and the leaf stays EXACT.
  FaultModeBasis degenerate;
  degenerate.id = FaultModeId(12);
  degenerate.sensor = SensorType::Uwb;
  degenerate.parameter_dimension = 1;
  degenerate.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Zero();
  CoverageCapacity capacity;
  capacity.inclusion_tolerance = 2.0;  // loose enough to pass the proof
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[0]}, {degenerate}, capacity);
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  const auto& envelope = certificate.envelopes.front();
  EXPECT_FALSE(envelope.dominant);
  EXPECT_FALSE(envelope.accepted);
  EXPECT_NE(envelope.reason.find("below the leaf exact bound"),
            std::string::npos)
      << envelope.reason;
  EXPECT_EQ(coverageLabelFor(certificate, fixture.modes[0].id),
            CoverageLabel::Exact);
  EXPECT_TRUE(certificate.complete);
}

// ---------------------------------------------------------------------------
// B3 Stage 1 (roadmap section 5.9): verified joint bound and risk ledger.
// ---------------------------------------------------------------------------

TEST(B3Risk, RSK01UnionBoundDoesNotDoubleChargeAndSolvesConservatively) {
  using namespace uwb_imu_pl;
  // Charged tail and per-axis split: the case pays alpha_h for the hypothesis,
  // so every axis may only use alpha_h / 3 (using alpha_h per axis would
  // understate the tail the ledger pays for).
  const double allocation = 3.0e-8;
  const double prior = 1.0e-4;
  const double p_md = 1.0e-3;
  const AxisTailSplit split = axisTailSplit(allocation, prior, p_md, 1e-5);
  ASSERT_TRUE(split.valid) << split.reason;
  EXPECT_NEAR(split.hypothesis_tail, allocation / prior, 1e-18);
  EXPECT_NEAR(split.axis_tail, split.hypothesis_tail / 3.0, 1e-18);
  EXPECT_DOUBLE_EQ(split.charge, std::max(split.hypothesis_tail, p_md));
  // The charge times the prior is exactly the allocation (no double charge).
  EXPECT_NEAR(prior * split.charge, std::max(allocation, prior * p_md), 1e-18);

  // The normal multiplier must grow when the tail is split over the axes.
  const double k_axis = StatisticalBoundsCache::normalTwoSidedMultiplier(
      split.axis_tail);
  const double k_single = StatisticalBoundsCache::normalTwoSidedMultiplier(
      split.hypothesis_tail);
  EXPECT_GT(k_axis, k_single);
  EXPECT_LT(k_single, k_axis);

  // dof = 1 has an independent closed form:
  //   P(chi'^2_1(lambda) <= tau) = Phi(sqrt(tau) - sqrt(lambda))
  //                               - Phi(-sqrt(tau) - sqrt(lambda))
  const int dof = 1;
  const double threshold = 6.0;
  const StatisticalBoundKey key;
  const auto boundary = StatisticalBoundsCache::noncentralityBoundaryVerified(
      dof, threshold, p_md, key);
  ASSERT_TRUE(boundary.valid) << boundary.reason;
  EXPECT_TRUE(boundary.converged);
  auto miss = [&](double noncentrality) {
    const double root = std::sqrt(noncentrality);
    const double t = std::sqrt(threshold);
    const auto normal = boost::math::normal();
    return boost::math::cdf(normal, t - root) -
        boost::math::cdf(normal, -t - root);
  };
  EXPECT_LE(miss(boundary.value), p_md)
      << "the returned boundary must be on the conservative side";
  EXPECT_LT(boundary.residual, 1e-12);
  EXPECT_LT(boundary.bracket_width, 1e-9 * std::max(1.0, boundary.value));
  // Monotone in p_md: a smaller miss allocation needs a larger noncentrality.
  const auto tighter = StatisticalBoundsCache::noncentralityBoundaryVerified(
      dof, threshold, 0.1 * p_md, key);
  ASSERT_TRUE(tighter.valid) << tighter.reason;
  EXPECT_GT(tighter.value, boundary.value);

  // Seeded synthetic acceptance sample (statistical, not a proof): with the
  // split tails, the measured per-axis exceedance of the k-sigma bound stays
  // within a 4-sigma band of the nominal two-sided tail.  Stated as sampling
  // evidence only; the deterministic claim is the union bound above.
  std::mt19937_64 generator(20260921);
  std::normal_distribution<double> standard(0.0, 1.0);
  const std::size_t samples = 200000;
  const double sigma = 0.37;
  std::size_t exceedances = 0;
  for (std::size_t index = 0; index < samples; ++index) {
    // Correlated three-axis protected error with equal marginals.
    const double common = standard(generator);
    for (int axis = 0; axis < 3; ++axis) {
      const double value = sigma * (0.6 * common + 0.8 * standard(generator));
      if (std::abs(value) > k_axis * sigma) ++exceedances;
    }
  }
  const double total = static_cast<double>(samples) * 3.0;
  const double rate = static_cast<double>(exceedances) / total;
  const double expected = split.axis_tail;
  const double sample_sigma = std::sqrt(expected * (1.0 - expected) / total);
  EXPECT_LT(rate, expected + 4.0 * sample_sigma)
      << "per-axis exceedance must stay inside the charged tail band";
}

TEST(B3Risk, RSK02LedgerHasNoHiddenZerosAndOverlapGivesNoDividend) {
  using namespace uwb_imu_pl;
  RiskBudgetV2 risk;
  FaultHypothesisV2 first;
  first.id = HypothesisId(1);
  first.hmi_allocation = 3.0e-8;
  first.prior_probability_bound = 1.0e-4;
  first.p_md_allocation = 1.0e-3;
  FaultHypothesisV2 second = first;
  second.id = HypothesisId(2);
  RiskLedgerInputs inputs;
  inputs.omitted_event_set = {"two_uwb", "imu_imu"};
  const RiskLedger one = buildRiskLedger(risk, {first}, inputs);
  ASSERT_TRUE(one.closes) << one.reason;
  ASSERT_TRUE(one.inputs_valid);
  // Every term is present with a status and a source: nothing hides as a
  // silent zero.
  for (const char* id : {"nominal", "p_nm", "hypotheses",
                         "hypotheses_miss_channel", "bridge", "history",
                         "model", "omitted", "envelope", "selection"}) {
    const auto* term = one.find(id);
    ASSERT_NE(term, nullptr) << id;
    EXPECT_FALSE(term->source.empty()) << id;
    EXPECT_FALSE(term->note.empty()) << id;
  }
  EXPECT_EQ(one.find("omitted")->status, RiskTermStatus::NotImplemented);
  EXPECT_NE(one.find("omitted")->note.find("two_uwb"), std::string::npos);
  EXPECT_EQ(one.find("bridge")->status, RiskTermStatus::AssumedUnvalidated);
  EXPECT_FALSE(one.all_terms_validated);
  EXPECT_FALSE(one.formal_eligible);
  EXPECT_TRUE(one.declared_total >= one.charged_total);

  // Overlapping coverage gives no risk dividend: adding a hypothesis never
  // lowers the charged total, and the miss channel stays explicit.
  const RiskLedger two = buildRiskLedger(risk, {first, second}, inputs);
  ASSERT_TRUE(two.closes) << two.reason;
  EXPECT_GT(two.charged_total, one.charged_total);
  const auto* one_fault = one.find("hypotheses");
  const auto* two_fault = two.find("hypotheses");
  ASSERT_NE(one_fault, nullptr);
  ASSERT_NE(two_fault, nullptr);
  EXPECT_NEAR(two_fault->value, 2.0 * one_fault->value, 1e-18);
  // The detector miss channel is reported but not charged.
  const auto* miss = two.find("hypotheses_miss_channel");
  ASSERT_NE(miss, nullptr);
  EXPECT_GT(miss->value, 0.0);
  EXPECT_EQ(miss->status, RiskTermStatus::AssumedUnvalidated);
  EXPECT_NEAR(two.charged_total,
              one.find("nominal")->value + one.find("p_nm")->value +
                  2.0 * one_fault->value,
              1e-18);
}

TEST(B3Risk, RSK03PositionMarginAndResidualThresholdActTogether) {
  using namespace uwb_imu_pl;
  // Both the detector threshold (through Lambda) and the position margin
  // (through k * sigma) enter the same bound: neither channel alone may be
  // used to claim the other.
  const StatisticalBoundKey key;
  const double p_md = 1e-3;
  const auto loose = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 10.0, p_md, key);
  const auto tight = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 40.0, p_md, key);
  ASSERT_TRUE(loose.valid);
  ASSERT_TRUE(tight.valid);
  EXPECT_GT(tight.value, loose.value)
      << "a higher detector threshold must require more noncentrality";
  const AxisTailSplit split = axisTailSplit(3e-8, 1e-4, p_md, 1e-5);
  ASSERT_TRUE(split.valid);
  const double k = StatisticalBoundsCache::normalTwoSidedMultiplier(
      split.axis_tail);
  const double sigma = 0.5;
  const double fault_component =
      std::sqrt(tight.value) * 0.25 + k * sigma;
  EXPECT_GT(fault_component, k * sigma)
      << "the fault size channel and the noise channel must both contribute";
  EXPECT_GT(fault_component, std::sqrt(tight.value) * 0.25);
}

TEST(B3Risk, BoundaryInputsAreRejectedAndCacheIdentityIsVersioned) {
  using namespace uwb_imu_pl;
  StatisticalBoundsCache::clear();
  const StatisticalBoundKey key;
  // beta = 0, beta >= 1, zero dof, non-positive threshold.
  EXPECT_FALSE(StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 10.0, 0.0, key).valid);
  EXPECT_FALSE(StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 10.0, 1.0, key).valid);
  EXPECT_FALSE(StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 10.0, 1.5, key).valid);
  EXPECT_FALSE(StatisticalBoundsCache::noncentralityBoundaryVerified(
      0, 10.0, 1e-3, key).valid);
  EXPECT_FALSE(StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 0.0, 1e-3, key).valid);
  EXPECT_FALSE(StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, -1.0, 1e-3, key).valid);
  // Key/dof disagreement is a policy mismatch, not a silent reuse.
  StatisticalBoundKey wrong_dof;
  wrong_dof.dof = 7;
  EXPECT_FALSE(StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 10.0, 1e-3, wrong_dof).valid);
  EXPECT_GE(StatisticalBoundsCache::stats().policy_mismatches, 1u);

  // Illegal priors are rejected by the split, never clamped.
  const AxisTailSplit bad = axisTailSplit(1e-8, 0.0, 1e-3, 1e-5);
  EXPECT_FALSE(bad.valid);
  EXPECT_NE(bad.reason.find("prior"), std::string::npos);
  const AxisTailSplit negative = axisTailSplit(-1.0, 1e-4, 1e-3, 1e-5);
  EXPECT_FALSE(negative.valid);
  const AxisTailSplit zero = axisTailSplit(0.0, 1e-4, 1e-3, 1e-5);
  EXPECT_TRUE(zero.valid);
  EXPECT_DOUBLE_EQ(zero.charge, 1e-3);

  // Tiny tails must stay finite (complement evaluation, no 1 - tiny).
  bool valid = false;
  const double k_tiny =
      StatisticalBoundsCache::normalTwoSidedMultiplierVerified(1e-300, &valid);
  EXPECT_TRUE(valid);
  EXPECT_TRUE(std::isfinite(k_tiny));
  EXPECT_GT(k_tiny, 30.0);
  const double k_invalid =
      StatisticalBoundsCache::normalTwoSidedMultiplierVerified(0.0, &valid);
  EXPECT_FALSE(valid);
  EXPECT_FALSE(std::isfinite(k_invalid));

  // Cache identity: same key hits, a contract/envelope version change misses.
  const auto before = StatisticalBoundsCache::stats();
  const auto first = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 21.0, 1e-3, key);
  const auto hit = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 21.0, 1e-3, key);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(hit.valid);
  EXPECT_DOUBLE_EQ(first.value, hit.value);
  StatisticalBoundKey new_contract;
  new_contract.contract_version = key.contract_version + 1;
  const auto versioned = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 21.0, 1e-3, new_contract);
  ASSERT_TRUE(versioned.valid);
  EXPECT_DOUBLE_EQ(versioned.value, first.value);
  StatisticalBoundKey envelope_changed;
  envelope_changed.envelope_fingerprint = 0xABCDEFULL;
  const auto enveloped = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 21.0, 1e-3, envelope_changed);
  ASSERT_TRUE(enveloped.valid);
  const auto after = StatisticalBoundsCache::stats();
  EXPECT_GE(after.hits - before.hits, 1u);
  EXPECT_GE(after.misses - before.misses, 3u)
      << "a contract or envelope version change must miss the cache";
}

// ---------------------------------------------------------------------------
// B4 Stage 3: lazy FDE action entities and health-state preservation.
// ---------------------------------------------------------------------------

TEST(B4LazyFde, FDE01HealthyFrameBuildsNoActionEntitiesButKeepsSensitivity) {
  using namespace uwb_imu_pl;
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d(0.1, 0.2, 0.3));
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  initial.velocity_world_mps = {0.2, 0.1, 0.0};
  initial.q_world_body = Eigen::Quaterniond(Eigen::AngleAxisd(
      0.2, Eigen::Vector3d(1, 2, 3).normalized()));
  estimator.initialize(initial, config.realtime.prior_sigmas);
  for (int sample = 0; sample <= 10; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(sample + 1);
    imu.timestamp = TimestampNs(sample * 5000000LL);
    imu.specific_force_mps2 = {0.3, -0.1, config.imu.gravity_mps2 + 0.05};
    imu.angular_velocity_radps = {0.1, -0.2, 0.15};
    estimator.ingestImu(imu);
  }
  auto input = batch(config, 55000000);
  auto transaction = estimator.prepareEpoch(input);
  auto window = estimator.buildIntegrityWindow(transaction, IntegrityWindowRequest{});
  auto imu_block = estimator.buildPendingFactorBlock(transaction, transaction.imu_group.id);
  auto bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bridge_group.id);
  const auto imu_maps = ImuFaultSubspaceBuilder().build(transaction, imu_block);

  // Lazy (default): mode descriptions and hypotheses exist, entities do not.
  auto lazy = HypothesisGenerator().generate(window, transaction, imu_maps,
                                             bridge_block);
  EXPECT_TRUE(lazy.action_entities_built == false);
  EXPECT_TRUE(lazy.single_mode_actions.empty());
  EXPECT_EQ(lazy.action_entities_constructed, 0u);
  EXPECT_EQ(lazy.bridge_blocks_built, 0u);
  EXPECT_EQ(lazy.action_entities_deferred, lazy.modes.size());
  ASSERT_EQ(lazy.actions.size(), 1u);
  EXPECT_EQ(lazy.actions.front().id.value(), 1u);
  EXPECT_EQ(lazy.actions.front().action_model_id, "KEEP_ALL");
  // Sensitivity is preserved: every mode still carries its fault map and every
  // hypothesis its risk allocation, which is what the PL path consumes.
  ASSERT_FALSE(lazy.modes.empty());
  for (const auto& mode : lazy.modes) {
    EXPECT_FALSE(mode.raw_group_maps.empty());
  }
  ASSERT_FALSE(lazy.hypotheses.empty());
  for (const auto& hypothesis : lazy.hypotheses) {
    EXPECT_GT(hypothesis.hmi_allocation, 0.0);
  }

  // Eager compatibility path produces the same identities.
  HypothesisGeneratorConfig eager_config;
  eager_config.lazy_action_entities = false;
  auto eager = HypothesisGenerator(eager_config).generate(window, transaction,
                                                          imu_maps, bridge_block);
  EXPECT_TRUE(eager.action_entities_built);
  EXPECT_EQ(eager.action_entities_constructed, eager.modes.size());
  EXPECT_EQ(eager.action_entities_deferred, 0u);
  ASSERT_EQ(eager.single_mode_actions.size(), eager.modes.size());
  EXPECT_GT(eager.actions.size(), 1u)
      << "the eager path still deduplicates recoverable actions";

  // On-demand materialization is idempotent and reproduces the eager ids.
  HypothesisGenerator::ensureActionEntities(transaction, window, &lazy);
  EXPECT_TRUE(lazy.action_entities_built);
  // The deduplicated catalog is reproduced identically (same ids, same order).
  ASSERT_EQ(lazy.actions.size(), eager.actions.size());
  for (std::size_t index = 0; index < lazy.actions.size(); ++index) {
    EXPECT_EQ(lazy.actions[index].id.value(), eager.actions[index].id.value());
  }
  ASSERT_EQ(lazy.single_mode_actions.size(), eager.single_mode_actions.size());
  for (std::size_t index = 0; index < lazy.single_mode_actions.size(); ++index) {
    EXPECT_EQ(lazy.single_mode_actions[index].id.value(),
              eager.single_mode_actions[index].id.value());
    EXPECT_EQ(lazy.single_mode_actions[index].action_model_id,
              eager.single_mode_actions[index].action_model_id);
    EXPECT_EQ(lazy.single_mode_actions[index].groups_to_remove.size(),
              eager.single_mode_actions[index].groups_to_remove.size());
  }
  const std::size_t constructed = lazy.action_entities_constructed;
  const std::size_t bridges = lazy.bridge_blocks_built;
  HypothesisGenerator::ensureActionEntities(transaction, window, &lazy);
  EXPECT_EQ(lazy.action_entities_constructed, constructed)
      << "ensureActionEntities must be idempotent";
  EXPECT_EQ(lazy.bridge_blocks_built, bridges);
  // On-demand materialization builds exactly the blocks the eager path builds.
  EXPECT_EQ(lazy.bridge_blocks_built, eager.bridge_blocks_built);
  estimator.discardEpoch(std::move(transaction),
      {FdeStatus::ModelInvalid, "lazy FDE development test", false});
}

TEST(B4LazyFde, FDE02IsolationPathStillMaterializesEvidenceOnDemand) {
  using namespace uwb_imu_pl;
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d(0.1, 0.2, 0.3));
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  initial.velocity_world_mps = {0.2, 0.1, 0.0};
  initial.q_world_body = Eigen::Quaterniond(Eigen::AngleAxisd(
      0.2, Eigen::Vector3d(1, 2, 3).normalized()));
  estimator.initialize(initial, config.realtime.prior_sigmas);
  for (int sample = 0; sample <= 10; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(sample + 1);
    imu.timestamp = TimestampNs(sample * 5000000LL);
    imu.specific_force_mps2 = {0.3, -0.1, config.imu.gravity_mps2 + 0.05};
    imu.angular_velocity_radps = {0.1, -0.2, 0.15};
    estimator.ingestImu(imu);
  }
  auto input = batch(config, 55000000);
  auto transaction = estimator.prepareEpoch(input);
  auto window = estimator.buildIntegrityWindow(transaction, IntegrityWindowRequest{});
  auto imu_block = estimator.buildPendingFactorBlock(transaction, transaction.imu_group.id);
  auto bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bridge_group.id);
  const auto imu_maps = ImuFaultSubspaceBuilder().build(transaction, imu_block);
  auto models = HypothesisGenerator().generate(window, transaction, imu_maps,
                                               bridge_block);
  std::vector<FaultModeEvidence> evidence;
  for (const auto& hypothesis : models.hypotheses) {
    FaultModeEvidence item;
    item.hypothesis = hypothesis.id;
    item.plausible = true;
    evidence.push_back(item);
  }
  EXPECT_EQ(models.action_entities_constructed, 0u);
  const auto actions = HypothesisGenerator().actionsForPlausibleSet(
      window, transaction, &models, evidence);
  EXPECT_GE(actions.size(), 2u)
      << "the isolation path must still build its candidate actions";
  EXPECT_GT(models.action_entities_constructed, 0u)
      << "entities are materialized on demand, not eagerly";
  EXPECT_TRUE(models.action_entities_built);
  EXPECT_EQ(models.action_entities_deferred, 0u);
  // Action identity is stable: KEEP_ALL first, then the mode actions in order.
  bool keep_all_seen = false;
  std::set<std::uint64_t> ids;
  for (const auto& action : actions) {
    EXPECT_TRUE(ids.insert(action.id.value()).second)
        << "duplicate action id " << action.id.value();
    keep_all_seen = keep_all_seen || action.action_model_id == "KEEP_ALL";
  }
  EXPECT_TRUE(keep_all_seen);
  estimator.discardEpoch(std::move(transaction),
      {FdeStatus::ModelInvalid, "lazy FDE isolation test", false});
}

// ---------------------------------------------------------------------------
// B3 Stage 2: the detection-space tri-state decides availability with
// direction-level reasons (no dictionary-wide rejection).
// ---------------------------------------------------------------------------

TEST(B3ZeroSpace, DangerousNullspaceIsUnavailableWithDirectionLevelReason) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  // A fault map proportional to the nominal design column is invisible in the
  // detection space but moves the protected state: exactly the dangerous case.
  const Eigen::Index rows = window.H.rows();
  const Eigen::VectorXd state_direction = window.H.col(0);
  FaultModeBasis mode;
  mode.id = FaultModeId(1);
  mode.sensor = SensorType::Uwb;
  mode.parameter_dimension = 1;
  Eigen::Index row_offset = 0;
  for (const auto& block : window.blocks) {
    const Eigen::Index block_rows = block.residual_raw.size();
    mode.raw_group_maps[block.group_id] = state_direction.segment(row_offset,
                                                                  block_rows);
    row_offset += block_rows;
  }
  ASSERT_EQ(row_offset, rows);
  const std::vector<FaultModeBasis> modes{mode};
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(1);
  hypothesis.modes = {mode.id};
  hypothesis.prior_probability_bound = 1e-4;
  hypothesis.p_md_allocation = 1e-3;
  std::vector<FaultHypothesisV2> hypotheses{hypothesis};
  std::shared_ptr<const FrozenHypothesisNumerics> shared;
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      window, modes, &hypotheses, 100.0, &shared);
  ASSERT_EQ(evidence.size(), 1u);
  ASSERT_TRUE(shared);
  ASSERT_EQ(shared->pl_entries.size(), 1u);
  const auto& entry = shared->pl_entries[0];
  EXPECT_EQ(entry.z_classification, 3) << entry.z_smallest_singular_value;
  EXPECT_FALSE(hypotheses[0].monitored);
  EXPECT_FALSE(evidence[0].plausible);
  const std::string reason = hypotheses[0].monitorability.reason;
  EXPECT_NE(reason.find("dangerous detection nullspace"), std::string::npos)
      << reason;
  EXPECT_NE(reason.find("axis residuals"), std::string::npos) << reason;
  EXPECT_NE(reason.find("modes 1"), std::string::npos) << reason;
  EXPECT_FALSE(entry.valid);
}

TEST(B3ZeroSpace, HarmlessNullspaceKeepsAFiniteProjectedBound) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  // Degenerate harmless limit expressible on the two-state synthetic fixture:
  // the protected map is full column rank there, so the only direction that is
  // both invisible and harmless is the zero response.  The classification must
  // still return the harmless branch with a finite (zero) projected bound
  // instead of rejecting the hypothesis or reporting a dangerous nullspace.
  FaultModeBasis mode;
  mode.id = FaultModeId(1);
  mode.sensor = SensorType::Uwb;
  mode.parameter_dimension = 1;
  for (const auto& block : window.blocks) {
    mode.raw_group_maps[block.group_id] =
        Eigen::VectorXd::Zero(block.residual_raw.size());
  }
  const std::vector<FaultModeBasis> modes{mode};
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(1);
  hypothesis.modes = {mode.id};
  hypothesis.prior_probability_bound = 1e-4;
  hypothesis.p_md_allocation = 1e-3;
  std::vector<FaultHypothesisV2> hypotheses{hypothesis};
  std::shared_ptr<const FrozenHypothesisNumerics> shared;
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      window, modes, &hypotheses, 100.0, &shared);
  ASSERT_EQ(evidence.size(), 1u);
  ASSERT_TRUE(shared);
  const auto& entry = shared->pl_entries[0];
  EXPECT_EQ(entry.z_classification, 2) << entry.z_smallest_singular_value;
  EXPECT_TRUE(entry.bound_from_projected_path);
  EXPECT_TRUE(entry.monitorability.monitorable);
  EXPECT_TRUE(entry.protected_slopes.allFinite());
  EXPECT_TRUE(entry.protected_slopes.isZero(1e-12))
      << entry.protected_slopes.transpose();
  EXPECT_TRUE(hypotheses[0].monitored);
}

// ---------------------------------------------------------------------------
// Stage 0 (C-round): physical attribution of a step-gate rejection.  The gate
// logic and thresholds are untouched; only the exported reason grows.
// ---------------------------------------------------------------------------
TEST(GateAttribution, StepAttributionNamesPhysicalBlocksAndDominantEpoch) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  StateLayoutEntry old_epoch;
  old_epoch.epoch = 3;
  old_epoch.column_offset = 0;
  old_epoch.dimension = 15;
  StateLayoutEntry current;
  current.epoch = 4;
  current.column_offset = 15;
  current.dimension = 15;
  current.protected_current_state = true;
  window.state_layout = {old_epoch, current};
  Eigen::VectorXd increment = Eigen::VectorXd::Zero(30);
  increment.segment(0, 3) = Eigen::Vector3d::Constant(0.1);     // rotation
  increment.segment(3, 3) = Eigen::Vector3d::Constant(0.2);     // position
  increment.segment(6, 3) = Eigen::Vector3d::Constant(0.05);    // velocity
  increment.segment(9, 3) = Eigen::Vector3d::Constant(0.01);    // accel bias
  increment.segment(12, 3) = Eigen::Vector3d::Constant(0.0);    // gyro bias
  increment.segment(15, 3) = Eigen::Vector3d::Constant(0.03);   // rotation
  increment.segment(18, 3) = Eigen::Vector3d::Constant(0.9);    // position
  increment.segment(21, 3) = Eigen::Vector3d::Constant(0.4);    // velocity
  increment.segment(24, 3) = Eigen::Vector3d::Constant(0.0);
  increment.segment(27, 3) = Eigen::Vector3d::Constant(0.0);
  const std::string attribution =
      stateStepAttribution(window, increment);
  EXPECT_NE(attribution.find("step attribution"), std::string::npos) << attribution;
  EXPECT_NE(attribution.find("rotation="), std::string::npos) << attribution;
  EXPECT_NE(attribution.find("position="), std::string::npos) << attribution;
  EXPECT_NE(attribution.find("velocity="), std::string::npos) << attribution;
  EXPECT_NE(attribution.find("accel_bias="), std::string::npos) << attribution;
  EXPECT_NE(attribution.find("gyro_bias="), std::string::npos) << attribution;
  // The dominant block is the current-epoch position (0.9 * sqrt(3)).
  EXPECT_NE(attribution.find("dominant=position(epoch 4)"), std::string::npos)
      << attribution;
  EXPECT_EQ(stateStepAttribution(window, Eigen::VectorXd()),
            "step attribution: empty increment");
}

// ---------------------------------------------------------------------------
// P7/S0-2: narrow-band regression for the one-sided relative dominance rule.
// The old absolute rule (env + tol >= leaf) accepted a 1e-7 relative shortfall;
// the relative rule must reject it while keeping binary64 noise accepted.
// ---------------------------------------------------------------------------
TEST(EnvelopeDominance, OneSidedRelativeRuleRejectsNarrowBandShortfall) {
  using namespace uwb_imu_pl;
  const double leaf = 0.37;
  const double tolerance = 1e-9;
  // 1e-7 relative shortfall: rejected (this is the case the old absolute rule
  // wrongly accepted).
  const double shortfall = leaf * (1.0 - 1e-7);
  EXPECT_FALSE(envelopeDominanceAccepts(shortfall, leaf, tolerance));
  EXPECT_LT(envelopeDominanceMargin(shortfall, leaf), -1e-8);
  EXPECT_LT(envelopeDominanceMargin(shortfall, leaf), -tolerance);
  // 1e-10 relative shortfall: inside the binary64 noise floor: accepted, with a
  // raw margin >= -tolerance.
  const double noise = leaf * (1.0 - 1e-10);
  EXPECT_TRUE(envelopeDominanceAccepts(noise, leaf, tolerance));
  const double noise_margin = envelopeDominanceMargin(noise, leaf);
  EXPECT_GE(noise_margin, -tolerance);
  EXPECT_LT(noise_margin, 0.0);
  // Exact and dominant envelopes.
  EXPECT_TRUE(envelopeDominanceAccepts(leaf, leaf, tolerance));
  EXPECT_GE(envelopeDominanceMargin(leaf, leaf), 0.0);
  EXPECT_TRUE(envelopeDominanceAccepts(2.0 * leaf, leaf, tolerance));
  EXPECT_NEAR(envelopeDominanceMargin(2.0 * leaf, leaf), 1.0, 1e-15);
  // Degenerate leaf (no monitorable direction) is trivially dominated; a
  // non-finite input is never accepted.
  EXPECT_TRUE(envelopeDominanceAccepts(0.0, 0.0, tolerance));
  EXPECT_FALSE(envelopeDominanceAccepts(
      std::numeric_limits<double>::quiet_NaN(), leaf, tolerance));
  EXPECT_FALSE(envelopeDominanceAccepts(
      std::numeric_limits<double>::infinity(), leaf, tolerance));

  // The same rule drives the builder: a scaled-group envelope (exactly equal
  // bounds) is accepted with a margin inside the noise floor.
  auto fixture = makeCompactFixture();
  FaultModeBasis group;
  group.id = FaultModeId(10);
  group.sensor = SensorType::Uwb;
  group.parameter_dimension = 1;
  group.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Constant(0.5);
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[0]}, {group});
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  const auto& envelope = certificate.envelopes.front();
  EXPECT_TRUE(envelope.accepted) << envelope.reason;
  EXPECT_GE(envelope.dominance_margin, -1e-9);
  EXPECT_GE(envelope.dominance_ratio, 1.0 - 1e-9);
  EXPECT_TRUE(envelope.dominant);
}
