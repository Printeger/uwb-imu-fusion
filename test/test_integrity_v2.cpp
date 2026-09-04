#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/estimation/reinitialization.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/health_manager.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

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
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-12, 1e10);
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
  const auto sensitivity =
      uwb_imu_pl::ImuFaultSubspaceBuilder().build(transaction, imu_block);
  EXPECT_TRUE(sensitivity.analytic_verified)
      << sensitivity.finite_difference_relative_error;
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
  const auto window = syntheticWindow();
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
  const auto window = syntheticWindow();
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
  ASSERT_TRUE(candidate.shared_base_covariance);
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
  uwb_imu_pl::ExclusionAction keep;
  keep.id = uwb_imu_pl::ExclusionActionId(1);
  const uwb_imu_pl::RankUpdateConfig config{1e-12, 1e10, 10.0};
  const uwb_imu_pl::RankUpdateEvaluator evaluator(config);
  const auto candidate = evaluator.evaluate(evaluator.factorizeOnce(window), keep);
  uwb_imu_pl::DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  const auto detector = uwb_imu_pl::JointWindowDetector().evaluateCandidate(
      candidate, detector_risk);
  std::vector<uwb_imu_pl::FaultHypothesisV2> remaining;
  const auto result = uwb_imu_pl::ProtectionLevelV2().compute(
      window, candidate, detector, &remaining, uwb_imu_pl::RiskBudgetV2{});
  EXPECT_TRUE(result.model_valid) << result.reason;
  EXPECT_TRUE(result.pl_xyz_m.allFinite());
  EXPECT_TRUE(result.risk_budget_valid);
  EXPECT_FALSE(result.formal_eligible);
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
      detector, {h1, h2}, {e1, e2}, &candidates, uwb_imu_pl::RiskBudgetV2{});
  ASSERT_TRUE(decision.selected_action.has_value());
  EXPECT_EQ(decision.selected_action->id, both.action.id);
  EXPECT_EQ(decision.status, uwb_imu_pl::FdeStatus::AmbiguousUnionExclusion);
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
