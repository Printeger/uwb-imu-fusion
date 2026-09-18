#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/estimation/reinitialization.hpp"
#include "uwb_imu_pl/factors/kinematic_bridge_factor.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/integrity/health_manager.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <gtest/gtest.h>
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
  for (std::size_t i = 0; i < optimized.size(); ++i) {
    EXPECT_EQ(reference[i].monitored, optimized[i].monitored);
    EXPECT_EQ(reference[i].monitorability.rank,
              optimized[i].monitorability.rank);
    EXPECT_EQ(reference_evidence[i].plausible,
              optimized_evidence[i].plausible);
    EXPECT_TRUE(reference_evidence[i].fault_gram.isApprox(
        optimized_evidence[i].fault_gram, 1e-12));
  }
  EXPECT_FALSE(optimized[0].monitored);
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
  auto actions = HypothesisGenerator().actionsForPlausibleSet(window, tx, models, evidence);
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
