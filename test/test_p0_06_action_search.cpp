#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/fde_post_selection.hpp"
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"
#include "uwb_imu_pl/publication/final_output_packet.hpp"

#include <boost/filesystem.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <set>
#include <tuple>

namespace {

using namespace uwb_imu_pl;

ActionSearchValidationV1 validateActionSearchCensusV1(
    const ActionSearchResultV1& result) {
  return uwb_imu_pl::validateActionSearchCensusV1(
      result.census, result.generated_snapshot,
      result.max_evaluated_actions, result.lifecycle, result.actions);
}

void bindActionSearch(const ActionSearchResultV1& result,
                      FdeDecisionContextV2* context) {
  context->action_search = &result.census;
  context->trusted_generated_actions = &result.generated_snapshot;
  context->max_evaluated_actions = result.max_evaluated_actions;
  context->action_search_lifecycle = result.lifecycle;
}

LinearizedFactorBlock oracleBlock(std::uint64_t id,
                                  const Eigen::MatrixXd& h,
                                  const Eigen::VectorXd& z,
                                  const LinearizationVersion& version) {
  LinearizedFactorBlock value;
  value.group_id = FactorGroupId(id);
  value.jacobian_whitened = h;
  value.residual_whitened = z;
  value.jacobian_raw = h;
  value.residual_raw = z;
  value.covariance = Eigen::MatrixXd::Identity(h.rows(), h.rows());
  value.whitener = value.covariance;
  value.version = version;
  return value;
}

LinearizedIntegrityWindow productionOracleWindow(double fault_amplitude) {
  LinearizedIntegrityWindow window;
  window.id = WindowId(6006);
  window.version = {60, 6, 0, 6};
  const Eigen::Matrix2d h0 = Eigen::Matrix2d::Identity() * 3.0;
  Eigen::Matrix<double, 3, 2> h1;
  h1 << 1.0, 0.2, 0.1, 1.0, 0.7, -0.4;
  Eigen::Matrix2d h2;
  h2 << 0.3, 0.8, -0.6, 0.2;
  window.blocks.push_back(oracleBlock(
      1, h0, Eigen::Vector2d(0.1, -0.2), window.version));
  window.blocks.push_back(oracleBlock(
      2, h1, Eigen::Vector3d(0.3, -0.1, 0.2), window.version));
  window.blocks.push_back(oracleBlock(
      3, h2, Eigen::Vector2d(fault_amplitude, -fault_amplitude),
      window.version));
  window.protected_state_map = Eigen::MatrixXd::Zero(3, 2);
  window.protected_state_map(0, 0) = 1.0;
  window.protected_state_map(1, 1) = 1.0;
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  return window;
}

ExclusionAction action(std::uint64_t id, double a, double b) {
  ExclusionAction out;
  out.id = ExclusionActionId(id);
  out.groups_to_remove = {FactorGroupId(id + 1000)};
  out.exclusion_cardinality = 1;
  out.action_model_id = "O07_ACTION";
  LinearizedFactorBlock block;
  block.group_id = FactorGroupId(id + 2000);
  block.kind = FactorKind::KinematicBridge;
  block.sensor = SensorType::ImuAccelerometer;
  block.role = RowRole::Measurement;
  block.window_column_indices = {0, 1};
  block.jacobian_whitened.resize(1, 2);
  block.jacobian_whitened << a, b;
  block.residual_whitened = Eigen::VectorXd::Constant(1, a - b);
  block.effective_weight = 1.0;
  block.whitening_model_id = "o07";
  block.version.graph_version = id;
  block.version.ordering_version = id + 1;
  block.version.noise_model_version = id + 2;
  block.version.linpoint_version = id + 3;
  out.added_blocks.push_back(std::move(block));
  out.groups_to_add.push_back(out.added_blocks.front().group_id);
  return out;
}

IntegrityConfig productionPipelineConfig() {
  IntegrityConfig config;
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
    FaultHypothesis allocation;
    allocation.id = HypothesisId(id);
    allocation.anchor_id = AnchorId(id);
    allocation.missed_detection_allocation = 1e-3;
    allocation.prior_probability_bound = 1e-4;
    config.risk.hypotheses.push_back(allocation);
  }
  return config;
}

UwbBatch productionPipelineBatch(TimestampNs timestamp) {
  UwbBatch batch;
  batch.id = BatchId(static_cast<std::uint64_t>(timestamp.value()));
  batch.timestamp = timestamp;
  batch.covariance_model_id = "p006_exception_diagonal";
  const Eigen::Vector3d position(0.0, 0.0, 1.0);
  const std::vector<Eigen::Vector3d> anchors = {
      {-5, -5, 0}, {5, -5, 1}, {5, 5, 3}, {-5, 5, 4}, {0, -6, 2}};
  for (std::size_t index = 0; index < anchors.size(); ++index) {
    UwbMeasurement measurement;
    measurement.id = MeasurementId(index + 1);
    measurement.factor_id = FactorId(index + 1);
    measurement.anchor_id = AnchorId(index + 1);
    measurement.timestamp = timestamp;
    measurement.anchor_position_m = anchors[index];
    measurement.range_m = (position - anchors[index]).norm();
    measurement.sigma_m = 0.1;
    batch.measurements.push_back(measurement);
  }
  return batch;
}

struct RealGeneratedInputs {
  FrozenIntegrityWindow frozen_window;
  EpochTransaction transaction;
  ImuFaultSubspaces imu_subspaces;
  LinearizedFactorBlock bridge_block;

  const LinearizedIntegrityWindow& window() const {
    return *frozen_window.seal->payload;
  }
};

RealGeneratedInputs realGeneratedInputs(double amplitude,
                                        std::size_t onset_sample,
                                        int axis,
                                        bool joint) {
  IntegrityConfig config = productionPipelineConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0.0, 0.0, 1.0};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  constexpr std::size_t kLastSample = 128;
  for (std::size_t sample = 0; sample <= kLastSample; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(600000 + sample);
    imu.timestamp = TimestampNs(static_cast<std::int64_t>(sample) * 1000000LL);
    imu.specific_force_mps2 = {0.0, 0.0, config.imu.gravity_mps2};
    if (sample >= onset_sample) {
      imu.specific_force_mps2(axis) += amplitude;
    }
    estimator.ingestImu(imu);
  }
  UwbBatch input = productionPipelineBatch(TimestampNs(129000000));
  if (joint) input.measurements.front().range_m += amplitude;
  RealGeneratedInputs out;
  out.transaction = estimator.prepareEpoch(input);
  out.frozen_window = estimator.buildFrozenIntegrityWindow(
      out.transaction, IntegrityWindowRequest{});
  const auto imu_block = estimator.buildPendingFactorBlock(
      out.transaction, out.transaction.imu_group.id);
  out.bridge_block = estimator.buildPendingFactorBlock(
      out.transaction, out.transaction.generic_bridge_group.id);
  out.imu_subspaces =
      ImuFaultSubspaceBuilder().build(out.transaction, imu_block);
  return out;
}

struct ProductionActionFixture {
  LinearizedIntegrityWindow window;
  EpochTransaction transaction;
  GeneratedFaultModelSet models;
  std::vector<FaultModeEvidence> all_plausible;
};

// Production-generated semantic universe for the 130-action O07 boundary:
// KEEP_ALL plus 128 single-epoch requests and the production full-union.
struct CompleteSemanticActionFixture {
  LinearizedIntegrityWindow window;
  EpochTransaction transaction;
  HypothesisGeneratorConfig generator_config;
  GeneratedFaultModelSet models;
  std::vector<FaultHypothesisV2> hypotheses;
  std::vector<FaultModeEvidence> evidence;
  std::vector<ExclusionAction> raw_actions;
  DetectorResultV2 all_in;
};

CompleteSemanticActionFixture completeSemanticActionFixture() {
  CompleteSemanticActionFixture out;
  out.window.id = WindowId(8606);
  out.window.version = {86, 6, 0, 6};
  out.window.protected_state_map = Eigen::Matrix3d::Identity();

  auto correlated_block = [&](std::uint64_t id, const Eigen::MatrixXd& raw_h,
                              const Eigen::VectorXd& raw_z,
                              const Eigen::MatrixXd& covariance) {
    LinearizedFactorBlock block;
    block.group_id = FactorGroupId(id);
    block.jacobian_raw = raw_h;
    block.residual_raw = raw_z;
    block.covariance = covariance;
    Eigen::LLT<Eigen::MatrixXd> llt(covariance);
    EXPECT_EQ(llt.info(), Eigen::Success);
    block.whitener = llt.matrixL().solve(
        Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
    block.jacobian_whitened = block.whitener * raw_h;
    block.residual_whitened = block.whitener * raw_z;
    block.whitening_model_id = "p006_correlated_raw";
    block.version = out.window.version;
    return block;
  };

  Eigen::Matrix<double, 6, 3> support_h = Eigen::Matrix<double, 6, 3>::Zero();
  support_h << 4.0, 0.2, 0.0,
               0.1, 3.8, 0.1,
               0.0, 0.2, 4.1,
               2.0, 0.0, 0.1,
               0.1, 2.1, 0.0,
               0.0, 0.1, 2.2;
  Eigen::Matrix<double, 6, 6> support_cov =
      Eigen::Matrix<double, 6, 6>::Identity();
  support_cov(0, 1) = support_cov(1, 0) = 0.23;
  support_cov(2, 3) = support_cov(3, 2) = -0.17;
  support_cov(4, 5) = support_cov(5, 4) = 0.19;
  auto support = correlated_block(7000, support_h,
      (Eigen::Matrix<double, 6, 1>() << 0.13, -0.09, 0.07, 0.04, -0.05, 0.11)
          .finished(), support_cov);
  support.kind = FactorKind::BoundaryPrior;
  support.sensor = SensorType::Prior;
  support.role = RowRole::Measurement;
  out.window.blocks.push_back(support);
  out.transaction.oldest_recoverable_epoch = 1;
  out.transaction.previous_epoch = 128;
  out.transaction.proposed_epoch = 129;
  out.transaction.begin = TimestampNs(128000000);
  out.transaction.end = TimestampNs(129000000);
  out.transaction.history_recoverability = HistoryRecoverability::Recoverable;
  out.window.detector_first_epoch = 1;

  // Every raw epoch is a real UWB occurrence with a nominal two-row group and
  // its registered leave-one-anchor-out replacement.  Correlated covariance
  // and nonzero RHS exercise production whitening. All epochs share one
  // physical anchor, which is exactly the production condition for adding a
  // FULL_PLAUSIBLE_UNION_RECOVERY after the single requests.
  for (std::size_t index = 0; index < 128; ++index) {
    const std::uint64_t nominal_id = 8000 + index;
    const std::uint64_t replacement_id = 18000 + index;
    const std::uint64_t anchor_id = 1;
    const MeasurementId first(28000 + 2 * index);
    const MeasurementId second(28001 + 2 * index);
    HistoricalEpochContext history;
    history.previous_epoch = index;
    history.proposed_epoch = index + 1;
    history.begin = TimestampNs(static_cast<std::int64_t>(index) * 1000000);
    history.end = TimestampNs(static_cast<std::int64_t>(index + 1) * 1000000);
    for (const MeasurementId id : {first, second}) {
      UwbMeasurement measurement;
      measurement.id = id;
      measurement.factor_id = FactorId(id.value());
      measurement.anchor_id = AnchorId(anchor_id);
      measurement.timestamp = history.end;
      measurement.anchor_position_m = Eigen::Vector3d(
          static_cast<double>(index % 7), static_cast<double>(index % 11), 0.0);
      measurement.range_m = 1.0;
      measurement.sigma_m = 1.0;
      history.uwb_batch.measurements.push_back(measurement);
    }
    PendingFactorGroup nominal;
    nominal.id = FactorGroupId(nominal_id);
    nominal.kind = FactorKind::UwbBatch;
    nominal.sensor = SensorType::Uwb;
    nominal.nominal = true;
    nominal.source_measurements = {first, second};
    nominal.raw_covariance.resize(2, 2);
    nominal.raw_covariance << 1.0, 0.31, 0.31, 1.4;
    nominal.noise_model_id = "p006_correlated_raw";
    PendingFactorGroup replacement;
    replacement.id = FactorGroupId(replacement_id);
    replacement.kind = FactorKind::UwbBatch;
    replacement.sensor = SensorType::Uwb;
    replacement.nominal = false;
    replacement.source_measurements = {second};
    replacement.excluded_fault_units = {FaultUnitId(anchor_id)};
    replacement.replaces_group = nominal.id;
    replacement.raw_covariance = Eigen::Matrix<double, 1, 1>::Constant(1.4);
    replacement.noise_model_id = nominal.noise_model_id;
    history.groups = {nominal, replacement};
    history.selected_groups = {nominal.id};
    out.transaction.recoverable_history.push_back(std::move(history));

    Eigen::Matrix<double, 2, 3> raw_h;
    const double angle = 0.071 * static_cast<double>(index + 1);
    raw_h << std::cos(angle), std::sin(angle), 0.2,
             -0.25 * std::sin(angle), 0.25 * std::cos(angle), 0.65;
    Eigen::Vector2d raw_z(1.0, 0.0);
    Eigen::Matrix2d covariance;
    covariance << 1.0, 0.31, 0.31, 1.4;
    auto block = correlated_block(nominal_id, raw_h, raw_z, covariance);
    block.kind = FactorKind::UwbBatch;
    block.sensor = SensorType::Uwb;
    block.role = RowRole::Measurement;
    out.window.blocks.push_back(std::move(block));
    FrozenWindowFactorInventoryEntry inventory;
    inventory.group_id = nominal.id;
    inventory.epoch = index + 1;
    inventory.kind = FactorKind::UwbBatch;
    inventory.sensor = SensorType::Uwb;
    inventory.disposition = FrozenFactorDisposition::ExplicitMeasurement;
    out.window.factor_inventory.push_back(std::move(inventory));
  }
  finalizeIntegrityWindow(&out.window, 1e-10, 1e10);
  // Set raw observations (never the acceptance threshold) so the production
  // all-in statistic is only 1e-3 above its unchanged chi-square threshold.
  // Each genuine epoch mode explains more than that excess, making all 128
  // hypotheses plausible under the real evidence rule.
  DetectorRiskContext scale_risk;
  scale_risk.p_fa_per_test = 1e-6;
  scale_risk.rank_tolerance = 1e-10;
  scale_risk.max_condition_number = 1e10;
  const auto unscaled = JointWindowDetector().evaluate(out.window, scale_risk);
  EXPECT_TRUE(unscaled.numerically_valid) << unscaled.reason;
  EXPECT_GT(unscaled.squared_parity_statistic, 0.0);
  const double scale = std::sqrt(
      (unscaled.squared_threshold + 1e-3) /
      unscaled.squared_parity_statistic);
  for (auto& block : out.window.blocks) {
    block.residual_raw *= scale;
    block.residual_whitened = block.whitener * block.residual_raw;
  }
  finalizeIntegrityWindow(&out.window, 1e-10, 1e10);
  out.generator_config.include_imu_faults = false;
  out.generator_config.include_epoch_independent_uwb = true;
  out.generator_config.include_persistent_uwb = false;
  out.generator_config.include_ramp_uwb = false;
  out.generator_config.double_faults_enabled = false;
  out.generator_config.max_candidate_count = 130;
  out.generator_config.max_exclusion_cardinality = 2;
  ImuFaultSubspaces unused_imu;
  LinearizedFactorBlock unused_bridge;
  out.models = HypothesisGenerator(out.generator_config).generate(
      out.window, out.transaction, unused_imu, unused_bridge);
  out.hypotheses = out.models.hypotheses;
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.rank_tolerance = 1e-10;
  detector_risk.max_condition_number = 1e10;
  out.all_in = JointWindowDetector().evaluate(out.window, detector_risk);
  HypothesisEvaluationConfig evidence_config;
  evidence_config.rank_tolerance = 1e-10;
  evidence_config.max_condition_number = 1e10;
  out.evidence = HypothesisEvidenceEvaluator(evidence_config).evaluateAll(
      out.window, out.models.modes, &out.models.hypotheses,
      out.all_in.squared_threshold);
  out.hypotheses = out.models.hypotheses;
  const auto search = HypothesisGenerator(out.generator_config)
      .actionsForPlausibleSetV1(out.window, out.transaction, &out.models,
                                out.evidence);
  out.raw_actions = search.generated_snapshot.actions;
  return out;
}

std::vector<FaultHypothesisV2> projectCompleteSemanticSet(
    const CompleteSemanticActionFixture& fixture,
    const ExclusionAction& action) {
  std::vector<FaultHypothesisV2> remaining =
      projectRemainingHypothesesForActionV1(fixture.hypotheses, action);
  for (auto& projected : remaining) {
    Eigen::Index rows = 0;
    for (const auto& block : fixture.window.blocks) {
      if (std::find(action.groups_to_remove.begin(),
                    action.groups_to_remove.end(), block.group_id) ==
          action.groups_to_remove.end()) rows += block.residual_whitened.size();
    }
    projected.A = Eigen::MatrixXd::Zero(rows, 1);
    Eigen::Index offset = 0;
    for (const auto& block : fixture.window.blocks) {
      if (std::find(action.groups_to_remove.begin(),
                    action.groups_to_remove.end(), block.group_id) !=
          action.groups_to_remove.end()) continue;
      for (const auto mode_id : projected.modes) {
        const auto mode = std::find_if(
            fixture.models.modes.begin(), fixture.models.modes.end(),
            [&](const FaultModeBasis& value) { return value.id == mode_id; });
        if (mode == fixture.models.modes.end()) continue;
        const auto raw = mode->raw_group_maps.find(block.group_id);
        if (raw != mode->raw_group_maps.end()) {
          projected.A.middleRows(offset, block.residual_whitened.size()) +=
              block.whitener * raw->second;
        }
      }
      offset += block.residual_whitened.size();
    }
  }
  return remaining;
}

struct IndependentDenseActionOracle {
  bool valid = false;
  bool post_passed = false;
  bool pl_valid = false;
  bool covers_complete_plausible = false;
  int rank = 0;
  int dof = 0;
  double statistic = std::numeric_limits<double>::infinity();
  double threshold = std::numeric_limits<double>::infinity();
  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;
  Eigen::Vector3d pl = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double hpl = std::numeric_limits<double>::infinity();
  double vpl = std::numeric_limits<double>::infinity();
  std::vector<HypothesisId> remaining;
  bool saw_correlated_covariance = false;
  bool saw_nonzero_raw_rhs = false;
  double production_whitening_h_error = 0.0;
  double production_whitening_z_error = 0.0;
};

Eigen::MatrixXd independentStableWhitener(const Eigen::MatrixXd& covariance) {
  const Eigen::MatrixXd symmetric =
      0.5 * (covariance + covariance.transpose());
  Eigen::LLT<Eigen::MatrixXd> llt(symmetric);
  if (llt.info() == Eigen::Success) {
    const Eigen::MatrixXd whitener = llt.matrixL().solve(
        Eigen::MatrixXd::Identity(symmetric.rows(), symmetric.cols()));
    if (whitener.allFinite()) return whitener;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(symmetric);
  if (eigen.info() != Eigen::Success ||
      eigen.eigenvalues().minCoeff() <= 0.0) return {};
  return eigen.eigenvalues().cwiseInverse().cwiseSqrt().asDiagonal() *
      eigen.eigenvectors().transpose();
}

IndependentDenseActionOracle independentDenseActionOracle(
    const CompleteSemanticActionFixture& fixture,
    const ExclusionAction& action, const DetectorRiskContext& detector_risk,
    const RiskBudgetV2& risk) {
  IndependentDenseActionOracle out;
  std::vector<const LinearizedFactorBlock*> retained;
  for (const auto& block : fixture.window.blocks) {
    if (std::find(action.groups_to_remove.begin(),
                  action.groups_to_remove.end(), block.group_id) ==
        action.groups_to_remove.end()) retained.push_back(&block);
  }
  for (const auto& block : action.added_blocks) retained.push_back(&block);
  Eigen::Index rows = 0;
  for (const auto* block : retained) rows += block->residual_raw.size();
  Eigen::MatrixXd h(rows, fixture.window.H.cols());
  Eigen::VectorXd z(rows);
  Eigen::Index offset = 0;
  for (const auto* block : retained) {
    const Eigen::Index count = block->residual_raw.size();
    if (block->covariance.rows() != count ||
        block->covariance.cols() != count ||
        block->jacobian_raw.rows() != count) return out;
    const Eigen::MatrixXd whitener =
        independentStableWhitener(block->covariance);
    if (whitener.rows() != count || whitener.cols() != count) return out;
    const Eigen::MatrixXd whitened_h = whitener * block->jacobian_raw;
    const Eigen::VectorXd whitened_z = whitener * block->residual_raw;
    h.middleRows(offset, count) = whitened_h;
    z.segment(offset, count) = whitened_z;
    out.production_whitening_h_error = std::max(
        out.production_whitening_h_error,
        (whitened_h - block->jacobian_whitened).norm());
    out.production_whitening_z_error = std::max(
        out.production_whitening_z_error,
        (whitened_z - block->residual_whitened).norm());
    out.saw_nonzero_raw_rhs = out.saw_nonzero_raw_rhs ||
        block->residual_raw.cwiseAbs().maxCoeff() > 0.0;
    out.saw_correlated_covariance = out.saw_correlated_covariance ||
        (block->covariance -
         block->covariance.diagonal().asDiagonal().toDenseMatrix())
            .cwiseAbs().maxCoeff() > 0.0;
    offset += count;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular = svd.singularValues();
  const double floor = detector_risk.rank_tolerance *
      (singular.size() ? singular(0) : 0.0);
  for (Eigen::Index index = 0; index < singular.size(); ++index) {
    if (singular(index) > floor) ++out.rank;
  }
  if (out.rank != h.cols()) return out;
  const Eigen::MatrixXd u = svd.matrixU().leftCols(out.rank);
  const Eigen::MatrixXd v = svd.matrixV().leftCols(out.rank);
  const Eigen::VectorXd inv = singular.head(out.rank).cwiseInverse();
  out.state = v * inv.asDiagonal() * u.transpose() * z;
  out.covariance = v * inv.array().square().matrix().asDiagonal() * v.transpose();
  out.statistic = (z - h * out.state).squaredNorm();
  out.dof = static_cast<int>(rows) - out.rank;
  out.threshold = boost::math::quantile(
      boost::math::complement(
          boost::math::chi_squared_distribution<double>(out.dof),
          detector_risk.p_fa_per_test));
  out.valid = out.dof > 0 && out.state.allFinite() &&
      out.covariance.allFinite() && std::isfinite(out.statistic) &&
      std::isfinite(out.threshold);
  out.post_passed = out.valid && out.statistic <= out.threshold;

  const auto plausible = completePlausibleHypotheses(
      fixture.hypotheses, fixture.evidence);
  out.covers_complete_plausible = std::all_of(
      plausible.begin(), plausible.end(), [&](HypothesisId id) {
        const auto found = std::find_if(
            fixture.hypotheses.begin(), fixture.hypotheses.end(),
            [&](const FaultHypothesisV2& hypothesis) {
              return hypothesis.id == id;
            });
        if (found == fixture.hypotheses.end()) return false;
        return std::all_of(found->modes.begin(), found->modes.end(),
                   [&](FaultModeId mode) {
                     return std::find(action.covered_modes.begin(),
                                      action.covered_modes.end(), mode) !=
                         action.covered_modes.end();
                   }) &&
            std::all_of(found->affected_groups.begin(),
                        found->affected_groups.end(), [&](FactorGroupId group) {
                          return std::find(action.groups_to_remove.begin(),
                                           action.groups_to_remove.end(), group) !=
                              action.groups_to_remove.end();
                        });
      });
  if (!out.post_passed) return out;

  auto remaining = projectRemainingHypothesesForActionV1(
      fixture.hypotheses, action);
  for (auto& projected : remaining) {
    projected.A = Eigen::MatrixXd::Zero(rows, 1);
    Eigen::Index raw_offset = 0;
    for (const auto* block : retained) {
      const Eigen::Index count = block->residual_raw.size();
      const Eigen::MatrixXd whitener =
          independentStableWhitener(block->covariance);
      for (const auto mode_id : projected.modes) {
        const auto mode = std::find_if(
            fixture.models.modes.begin(), fixture.models.modes.end(),
            [&](const FaultModeBasis& value) { return value.id == mode_id; });
        if (mode == fixture.models.modes.end()) continue;
        const auto raw = mode->raw_group_maps.find(block->group_id);
        if (raw != mode->raw_group_maps.end()) {
          projected.A.middleRows(raw_offset, count) += whitener * raw->second;
        }
      }
      raw_offset += count;
    }
  }
  for (const auto& hypothesis : remaining) out.remaining.push_back(hypothesis.id);
  const Eigen::Matrix3d protected_covariance =
      fixture.window.protected_state_map * out.covariance *
      fixture.window.protected_state_map.transpose();
  const Eigen::Vector3d sigma = protected_covariance.diagonal().cwiseSqrt();
  const double nominal_tail = std::max(1e-15, risk.nominal_axis_tail);
  const double nominal_k = boost::math::quantile(
      boost::math::complement(boost::math::normal_distribution<double>(),
                              nominal_tail / 2.0));
  Eigen::Vector3d fault_component = Eigen::Vector3d::Zero();
  for (const auto& hypothesis : remaining) {
    if (hypothesis.A.rows() != rows || hypothesis.A.cols() != 1) return out;
    const Eigen::MatrixXd residual_fault =
        hypothesis.A - u * (u.transpose() * hypothesis.A);
    const double gram = residual_fault.squaredNorm();
    if (!(gram > 0.0) || !std::isfinite(gram)) return out;
    const Eigen::Vector3d protected_response =
        fixture.window.protected_state_map * v * inv.asDiagonal() *
        u.transpose() * hypothesis.A;
    const double p_md = hypothesis.p_md_allocation;
    double low = 0.0;
    double high = 1.0;
    auto miss = [&](double lambda) {
      return boost::math::cdf(
          boost::math::non_central_chi_squared_distribution<double>(
              out.dof, lambda), out.threshold);
    };
    while (miss(high) > p_md && high < 1e12) high *= 2.0;
    if (!(high < 1e12)) return out;
    for (int iteration = 0; iteration < 160; ++iteration) {
      const double middle = 0.5 * (low + high);
      if (miss(middle) > p_md) low = middle;
      else high = middle;
    }
    const double tail = std::min(
        0.5, hypothesis.hmi_allocation /
                 std::max(hypothesis.prior_probability_bound, 1e-15));
    const double k = boost::math::quantile(
        boost::math::complement(boost::math::normal_distribution<double>(),
                                (tail / 3.0) / 2.0));
    const Eigen::Vector3d bounded = protected_response.cwiseAbs() *
        std::sqrt(high / gram) + k * sigma;
    fault_component = fault_component.cwiseMax(bounded);
  }
  out.pl = (nominal_k * sigma).cwiseMax(fault_component);
  out.hpl = std::hypot(out.pl.x(), out.pl.y());
  out.vpl = out.pl.z();
  out.pl_valid = out.pl.allFinite();
  return out;
}

ProductionActionFixture productionActionFixture(
    std::size_t mode_count, std::size_t fault_mode_index,
    double fault_amplitude, int axis_seed = 0, std::size_t onset_seed = 0,
    std::optional<std::size_t> second_fault_mode_index = std::nullopt) {
  ProductionActionFixture out;
  out.window.id = WindowId(7006 + onset_seed);
  out.window.version = {70 + onset_seed, 6, 0, 6};
  out.window.protected_state_map = Eigen::MatrixXd::Identity(3, 3);
  out.transaction.previous_epoch = mode_count + 1;
  out.transaction.proposed_epoch = mode_count + 2;

  // Independent state support stays in every candidate graph.
  Eigen::Matrix3d support_h = 4.0 * Eigen::Matrix3d::Identity();
  auto support = oracleBlock(1, support_h, Eigen::Vector3d::Zero(),
                             out.window.version);
  support.kind = FactorKind::BoundaryPrior;
  support.sensor = SensorType::Prior;
  support.role = RowRole::Regularizer;
  out.window.blocks.push_back(std::move(support));

  for (std::size_t index = 0; index < mode_count; ++index) {
    const std::uint64_t nominal_id = 1000 + index;
    const std::uint64_t replacement_id = 10000 + index;
    const std::uint64_t anchor_id = index + 1;
    const std::uint64_t measurement0 = 20000 + 2 * index;
    const std::uint64_t measurement1 = measurement0 + 1;
    HistoricalEpochContext history;
    history.previous_epoch = onset_seed + index;
    history.proposed_epoch = onset_seed + index + 1;
    history.begin = TimestampNs(static_cast<std::int64_t>(
        1000000 * history.previous_epoch));
    history.end = TimestampNs(static_cast<std::int64_t>(
        1000000 * history.proposed_epoch));

    PendingFactorGroup nominal;
    nominal.id = FactorGroupId(nominal_id);
    nominal.kind = FactorKind::UwbBatch;
    nominal.sensor = SensorType::Uwb;
    nominal.nominal = true;
    nominal.source_measurements = {MeasurementId(measurement0),
                                   MeasurementId(measurement1)};
    nominal.raw_covariance = Eigen::Matrix2d::Identity();
    nominal.noise_model_id = "p006_production_covariance";

    PendingFactorGroup replacement;
    replacement.id = FactorGroupId(replacement_id);
    replacement.kind = FactorKind::UwbBatch;
    replacement.sensor = SensorType::Uwb;
    replacement.nominal = false;
    replacement.source_measurements = {MeasurementId(measurement1)};
    replacement.excluded_fault_units = {FaultUnitId(anchor_id)};
    replacement.replaces_group = nominal.id;
    replacement.raw_covariance = Eigen::Matrix<double, 1, 1>::Identity();
    replacement.noise_model_id = nominal.noise_model_id;
    history.groups = {nominal, replacement};
    history.selected_groups = {nominal.id};
    out.transaction.recoverable_history.push_back(std::move(history));

    Eigen::Matrix<double, 2, 3> h;
    const int axis = (axis_seed + static_cast<int>(index)) % 3;
    h.setZero();
    h(0, axis) = 1.0 + 0.001 * static_cast<double>(onset_seed + index);
    h(0, (axis + 1) % 3) = 0.15;
    h(1, axis) = 0.2;
    h(1, (axis + 2) % 3) = 0.35 + 0.0005 * onset_seed;
    Eigen::Vector2d residual = Eigen::Vector2d::Zero();
    if (index == fault_mode_index ||
        (second_fault_mode_index && index == *second_fault_mode_index)) {
      residual(0) = fault_amplitude;
    }
    auto block = oracleBlock(nominal_id, h, residual, out.window.version);
    block.kind = FactorKind::UwbBatch;
    block.sensor = SensorType::Uwb;
    block.role = RowRole::Measurement;
    out.window.blocks.push_back(std::move(block));

    FaultModeBasis mode;
    mode.id = FaultModeId(index + 1);
    mode.kind = FaultKind::AnchorBiasEpochIndependent;
    mode.sensor = SensorType::Uwb;
    mode.physical_source_id = "uwb:" + std::to_string(anchor_id);
    mode.anchor_id = AnchorId(anchor_id);
    mode.axis = axis;
    mode.onset_epoch = onset_seed + index;
    mode.onset_time = TimestampNs(static_cast<std::int64_t>(
        1000000 * mode.onset_epoch));
    mode.parameter_dimension = 1;
    mode.effective_parameter_dimension = 1;
    mode.effective_parameter_basis = Eigen::Matrix<double, 1, 1>::Identity();
    mode.effective_basis_certified = true;
    mode.raw_group_maps.emplace(
        nominal.id, Eigen::Matrix<double, 2, 1>::Ones());
    mode.affected_groups = {nominal.id};
    mode.affected_measurements = {MeasurementId(measurement0)};
    mode.prior_probability_bound = 1e-3;
    out.models.modes.push_back(mode);

    FaultUnit unit;
    unit.id = FaultUnitId(mode.id.value());
    unit.sensor = SensorType::Uwb;
    unit.kind = mode.kind;
    unit.physical_source_id = mode.physical_source_id;
    unit.axis = axis;
    unit.epoch_begin = mode.onset_epoch;
    unit.epoch_end = mode.onset_epoch;
    unit.affected_groups = mode.affected_groups;
    unit.affected_measurements = mode.affected_measurements;
    unit.parameter_dimension = 1;
    unit.prior_probability_bound = mode.prior_probability_bound;
    out.models.units.push_back(std::move(unit));

    FaultHypothesisV2 hypothesis;
    hypothesis.id = HypothesisId(index + 1);
    hypothesis.units = {FaultUnitId(mode.id.value())};
    hypothesis.modes = {mode.id};
    hypothesis.affected_groups = mode.affected_groups;
    hypothesis.prior_probability_bound = 1e-3;
    hypothesis.p_md_allocation = 1e-3;
    hypothesis.hmi_allocation = 1e-6;
    out.models.hypotheses.push_back(hypothesis);

    FaultModeEvidence evidence;
    evidence.hypothesis = hypothesis.id;
    evidence.plausible = true;
    evidence.unit_kind = FaultUnitKind::UwbRangeMeters;
    evidence.parameter_dimension = 1;
    evidence.profile_j = 1.0 + 0.01 * index;
    evidence.profile_valid = true;
    out.all_plausible.push_back(std::move(evidence));
  }
  finalizeIntegrityWindow(&out.window, 1e-10, 1e10);
  return out;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> independentRetainedSvd(
    const LinearizedIntegrityWindow& window, const ExclusionAction& action) {
  std::vector<const LinearizedFactorBlock*> retained;
  for (const auto& block : window.blocks) {
    if (std::find(action.groups_to_remove.begin(),
                  action.groups_to_remove.end(), block.group_id) ==
        action.groups_to_remove.end()) retained.push_back(&block);
  }
  for (const auto& block : action.added_blocks) retained.push_back(&block);
  Eigen::Index rows = 0;
  for (const auto* block : retained) rows += block->jacobian_whitened.rows();
  Eigen::MatrixXd h(rows, window.H.cols());
  Eigen::VectorXd z(rows);
  Eigen::Index offset = 0;
  for (const auto* block : retained) {
    const Eigen::Index n = block->jacobian_whitened.rows();
    h.block(offset, 0, n, window.H.cols()) = block->jacobian_whitened;
    z.segment(offset, n) = block->residual_whitened;
    offset += n;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd inverse = svd.singularValues().cwiseInverse();
  return {svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose() * z,
          svd.matrixV() * inverse.array().square().matrix().asDiagonal() *
              svd.matrixV().transpose()};
}

TEST(P006ActionSearchRed, DistinctActionBeyond128MakesSearchIncomplete) {
  std::vector<ExclusionAction> generated;
  for (std::uint64_t i = 0; i < 130; ++i) {
    generated.push_back(action(i + 1, static_cast<double>(i + 1), 0.25));
  }
  const auto result = censusAndCapActionsV1(generated, 128);
  EXPECT_EQ(result.census.generated, 130u);
  EXPECT_EQ(result.census.evaluated, 128u);
  EXPECT_EQ(result.census.omitted, 2u);
  EXPECT_FALSE(result.census.exhaustive);
  EXPECT_EQ(result.census.terminal_reason, "SEARCH_INCOMPLETE");
  ASSERT_EQ(result.census.omitted_actions.size(), 2u);
  EXPECT_EQ(result.census.omitted_actions.front().action_id, 129u);
  EXPECT_FALSE(result.census.omitted_actions.front().proven_safe);
  EXPECT_EQ(result.census.omitted_actions.front().reason,
            "RESOURCE_CAP_UNPROVEN");
}

TEST(P006ActionSearchRed, ProductionGeneratorReportsEveryTruncatedIdentity) {
  GeneratedFaultModelSet models;
  models.action_entities_built = true;
  std::vector<FaultModeEvidence> evidence;
  for (std::uint64_t i = 0; i < 130; ++i) {
    FaultModeBasis mode;
    mode.id = FaultModeId(i + 1);
    mode.sensor = SensorType::Uwb;
    mode.anchor_id = AnchorId(i + 1);
    mode.parameter_dimension = 1;
    models.modes.push_back(mode);
    FaultHypothesisV2 hypothesis;
    hypothesis.id = HypothesisId(i + 1);
    hypothesis.modes = {mode.id};
    models.hypotheses.push_back(hypothesis);
    FaultModeEvidence item;
    item.hypothesis = hypothesis.id;
    item.plausible = true;
    evidence.push_back(item);
    ExclusionAction single = action(i + 1, static_cast<double>(i + 1), 0.25);
    single.covered_modes = {mode.id};
    single.covered_units = {FaultUnitId(i + 1)};
    single.physical_source_ids = {"uwb:" + std::to_string(i + 1)};
    models.single_mode_actions.push_back(std::move(single));
  }
  HypothesisGeneratorConfig config;
  config.max_candidate_count = 128;
  LinearizedIntegrityWindow window;
  EpochTransaction transaction;
  const auto result = HypothesisGenerator(config).actionsForPlausibleSetV1(
      window, transaction, &models, evidence);
  EXPECT_EQ(result.census.generated, 131u);  // KEEP_ALL + 130 admissible actions
  EXPECT_EQ(result.census.evaluated, 128u);
  EXPECT_EQ(result.census.omitted, 3u);
  EXPECT_FALSE(result.census.exhaustive);
  ASSERT_EQ(result.census.omitted_actions.size(), 3u);
  std::set<std::uint64_t> omitted_ids;
  for (const auto& omitted : result.census.omitted_actions) {
    omitted_ids.insert(omitted.action_id);
    EXPECT_EQ(omitted.reason, "RESOURCE_CAP_UNPROVEN");
    EXPECT_FALSE(omitted.operation_identity.empty());
  }
  EXPECT_EQ(omitted_ids.size(), 3u);
}

TEST(P006ActionSearchRed, ExactDuplicateOnlyIsProofSafe) {
  const ExclusionAction first = action(1, 3.0, 4.0);
  ExclusionAction duplicate = first;
  const auto result = censusAndCapActionsV1({first, duplicate}, 128);
  ASSERT_EQ(result.actions.size(), 1u);
  EXPECT_TRUE(result.census.exhaustive);
  EXPECT_EQ(result.census.generated, 2u);
  EXPECT_EQ(result.census.evaluated, 1u);
  EXPECT_EQ(result.census.omitted, 1u);
  ASSERT_EQ(result.census.omitted_actions.size(), 1u);
  EXPECT_TRUE(result.census.omitted_actions[0].proven_safe);
  EXPECT_EQ(result.census.omitted_actions[0].reason,
            "EXACT_SEMANTIC_DUPLICATE");
  EXPECT_EQ(result.actions[0].id, first.id);
  EXPECT_EQ(result.census.omitted_actions[0].semantic_identity,
            exactActionSemanticIdentityV1(first));
}

TEST(P006ActionSearchRed, SameCompactHashShapeDifferentOperationIsNotMerged) {
  // These blocks have the same group ids, dimensions, whitening metadata,
  // residual norm and Jacobian squared norm, so the old compact actionKey is
  // identical.  Their exact graph operations differ.
  const ExclusionAction left = action(7, 3.0, 4.0);
  ExclusionAction right = left;
  right.id = ExclusionActionId(8);
  right.added_blocks[0].jacobian_whitened << 4.0, 3.0;
  const auto result = censusAndCapActionsV1({left, right}, 128);
  EXPECT_EQ(result.actions.size(), 2u);
  EXPECT_TRUE(result.census.exhaustive);
  EXPECT_NE(exactActionOperationIdentityV1(left),
            exactActionOperationIdentityV1(right));
}

TEST(P006ActionSearchRed, ReorderingPreservesCompleteCensusAndRefusal) {
  std::vector<ExclusionAction> generated;
  for (std::uint64_t i = 0; i < 130; ++i) {
    generated.push_back(action(i + 1, static_cast<double>(i + 1), 0.5));
  }
  auto reversed = generated;
  std::reverse(reversed.begin(), reversed.end());
  const auto forward = censusAndCapActionsV1(generated, 128);
  const auto backward = censusAndCapActionsV1(reversed, 128);
  EXPECT_FALSE(forward.census.exhaustive);
  EXPECT_FALSE(backward.census.exhaustive);
  EXPECT_EQ(forward.census.terminal_reason, backward.census.terminal_reason);
  EXPECT_EQ(forward.census.generated, backward.census.generated);
  std::set<std::string> forward_all, backward_all;
  for (const auto& record : forward.census.generated_records) {
    forward_all.insert(record.operation_identity);
  }
  for (const auto& record : backward.census.generated_records) {
    backward_all.insert(record.operation_identity);
  }
  EXPECT_EQ(forward_all, backward_all);
}

TEST(P006ActionSearchRed, ByteExactZeroNaNAndDuplicateProof) {
  const ExclusionAction positive = action(7, +0.0, 1.0);
  ExclusionAction negative = positive;
  negative.id = ExclusionActionId(8);
  negative.added_blocks[0].jacobian_whitened(0, 0) = -0.0;
  const auto signed_zero = censusAndCapActionsV1({positive, negative}, 2);
  EXPECT_EQ(signed_zero.actions.size(), 2u);
  EXPECT_NE(exactActionOperationIdentityV1(positive),
            exactActionOperationIdentityV1(negative));

  ExclusionAction duplicate = positive;
  const auto exact = censusAndCapActionsV1({positive, duplicate}, 2);
  ASSERT_EQ(exact.census.omitted_actions.size(), 1u);
  EXPECT_EQ(exact.census.omitted_actions[0].duplicate_of_identity,
            exact.census.evaluated_identities[0]);
  const auto exact_validation = validateActionSearchCensusV1(exact);
  EXPECT_TRUE(exact_validation.valid) << exact_validation.reason;

  ExclusionAction nan = positive;
  nan.id = ExclusionActionId(10);
  nan.added_blocks[0].jacobian_whitened(0, 0) =
      std::numeric_limits<double>::quiet_NaN();
  const auto invalid = censusAndCapActionsV1({positive, nan}, 2);
  ASSERT_EQ(invalid.census.omitted_actions.size(), 1u);
  EXPECT_EQ(invalid.census.omitted_actions[0].reason,
            "INVALID_OPERATION_ENCODING");
  EXPECT_FALSE(invalid.census.exhaustive);
}

TEST(P006ActionSearchRed, CapOneAndAbortUseActualEvaluatedCount) {
  ExclusionAction first = action(1, 1.0, 0.0);
  ExclusionAction duplicate = first;
  ActionSearchResultV1 result = censusAndCapActionsV1(
      {first, duplicate, action(2, 2.0, 0.0)}, 1);
  EXPECT_EQ(result.census.evaluated, 1u);
  ASSERT_TRUE(std::any_of(result.census.omitted_actions.begin(),
                          result.census.omitted_actions.end(),
      [](const ActionOmissionV1& value) {
        return value.reason == "EXACT_SEMANTIC_DUPLICATE";
      }));
  abortIncompleteActionSearchBeforeEvaluationV1(&result);
  EXPECT_TRUE(result.actions.empty());
  EXPECT_EQ(result.census.evaluated, 0u);
  EXPECT_EQ(result.census.generated, 3u);
  EXPECT_EQ(result.census.omitted, 3u);
  EXPECT_EQ(std::count_if(result.census.omitted_actions.begin(),
                          result.census.omitted_actions.end(),
      [](const ActionOmissionV1& value) {
        return value.reason == "EXACT_SEMANTIC_DUPLICATE" &&
            value.proven_safe && !value.duplicate_of_identity.empty();
      }), 1);
  const auto validation = validateActionSearchCensusV1(result);
  EXPECT_TRUE(validation.valid) << validation.reason;
  EXPECT_FALSE(validation.exhaustive);
}

TEST(P006ActionSearchRed, StrictValidatorRejectsTamperOverflowAndFakeSafe) {
  const auto source = censusAndCapActionsV1(
      {action(1, 1.0, 0.0), action(2, 2.0, 0.0)}, 1);
  for (int mutation = 0; mutation < 7; ++mutation) {
    auto tampered = source;
    auto& census = tampered.census;
    if (mutation == 0) census.protocol_version = 99;
    if (mutation == 1) census.exhaustive = true;
    if (mutation == 2) census.omitted_actions[0].proven_safe = true;
    if (mutation == 3) {
      census.omitted_actions[0].reason = "EXACT_SEMANTIC_DUPLICATE";
      census.omitted_actions[0].duplicate_of_identity =
          census.evaluated_identities[0];
      census.omitted_actions[0].proven_safe = true;
    }
    if (mutation == 4) census.generated_identities[0] = "duplicate";
    if (mutation == 5) {
      census.evaluated = std::numeric_limits<std::uint64_t>::max();
      census.omitted = 1;
    }
    if (mutation == 6) census.generated_records[0].operation_identity += "x";
    const auto validation = validateActionSearchCensusV1(tampered);
    EXPECT_FALSE(validation.valid) << mutation;
  }
}

TEST(P006ActionSearchRed,
     SelfConsistentSidecarAndEmbeddedRawCannotReplaceTrustedSnapshot) {
  const ExclusionAction retained = action(1, 1.0, 0.0);
  ExclusionAction exact_duplicate = retained;
  const auto forged = censusAndCapActionsV1(
      {retained, exact_duplicate}, 2);
  ASSERT_TRUE(forged.census.exhaustive);
  ASSERT_EQ(forged.actions.size(), 1u);

  ExclusionAction actually_generated_omitted = retained;
  actually_generated_omitted.added_blocks[0].jacobian_whitened(0, 0) = 9.0;
  const auto trusted = censusAndCapActionsV1(
      {retained, actually_generated_omitted}, 2);
  ASSERT_FALSE(equivalentActionOperation(
      retained, actually_generated_omitted));
  const auto validation = uwb_imu_pl::validateActionSearchCensusV1(
      forged.census, trusted.generated_snapshot,
      forged.max_evaluated_actions, forged.lifecycle, forged.actions);
  EXPECT_FALSE(validation.valid);
  EXPECT_FALSE(validation.exhaustive);
  EXPECT_NE(validation.reason.find("raw"), std::string::npos)
      << validation.reason;
}

TEST(P006ActionSearchRed, CapLifecycleAndEvaluatedOrderAreConsumerBound) {
  auto ready = censusAndCapActionsV1(
      {action(1, 1.0, 0.0), action(2, 2.0, 0.0)}, 1);
  ASSERT_TRUE(validateActionSearchCensusV1(ready).valid);
  EXPECT_FALSE(uwb_imu_pl::validateActionSearchCensusV1(
      ready.census, ready.generated_snapshot, 2, ready.lifecycle,
      ready.actions).valid);

  auto aborted = ready;
  abortIncompleteActionSearchBeforeEvaluationV1(&aborted);
  ASSERT_TRUE(validateActionSearchCensusV1(aborted).valid);
  EXPECT_FALSE(uwb_imu_pl::validateActionSearchCensusV1(
      aborted.census, aborted.generated_snapshot,
      aborted.max_evaluated_actions,
      ActionSearchLifecycleV1::ReadyForEvaluation, aborted.actions).valid);

  auto complete = censusAndCapActionsV1(
      {action(1, 1.0, 0.0), action(2, 2.0, 0.0)}, 2);
  ASSERT_TRUE(validateActionSearchCensusV1(complete).valid);
  std::reverse(complete.actions.begin(), complete.actions.end());
  EXPECT_FALSE(validateActionSearchCensusV1(complete).valid);
}

TEST(P006ActionSearchRed, DuplicateActionIdDoesNotAliasOccurrenceOrOperation) {
  ExclusionAction left = action(42, 1.0, 0.0);
  ExclusionAction right = action(42, 2.0, 0.0);
  const auto result = censusAndCapActionsV1({left, right}, 2);
  ASSERT_TRUE(result.actions.empty());
  ASSERT_EQ(result.census.generated_records.size(), 2u);
  EXPECT_NE(result.census.generated_records[0].occurrence_identity,
            result.census.generated_records[1].occurrence_identity);
  EXPECT_NE(result.census.generated_records[0].operation_identity,
            result.census.generated_records[1].operation_identity);
  EXPECT_FALSE(result.census.exhaustive);
  ASSERT_EQ(result.census.omitted_actions.size(), 2u);
  for (const auto& omitted : result.census.omitted_actions) {
    EXPECT_EQ(omitted.reason, "DUPLICATE_ACTION_ID_CONFLICT");
    EXPECT_FALSE(omitted.proven_safe);
  }
  EXPECT_TRUE(validateActionSearchCensusV1(result).valid);

  auto tampered = result;
  tampered.census.generated_records[1].occurrence_identity =
      tampered.census.generated_records[0].occurrence_identity;
  tampered.census.generated_identities[1] =
      tampered.census.generated_identities[0];
  EXPECT_FALSE(validateActionSearchCensusV1(tampered).valid);
}

TEST(P006ActionSearchRed, ProductionRawDuplicateOccurrenceIsRecorded) {
  GeneratedFaultModelSet models;
  models.action_entities_built = true;
  FaultModeBasis mode;
  mode.id = FaultModeId(1);
  mode.sensor = SensorType::Uwb;
  mode.anchor_id = AnchorId(1);
  models.modes = {mode};
  for (std::uint64_t id : {1u, 2u}) {
    FaultHypothesisV2 hypothesis;
    hypothesis.id = HypothesisId(id);
    hypothesis.modes = {mode.id};
    models.hypotheses.push_back(hypothesis);
  }
  ExclusionAction single = action(10, 3.0, 4.0);
  single.covered_modes = {mode.id};
  models.single_mode_actions = {single};
  FaultModeEvidence one, two;
  one.hypothesis = HypothesisId(1);
  two.hypothesis = HypothesisId(2);
  one.plausible = two.plausible = true;
  HypothesisGeneratorConfig config;
  config.max_candidate_count = 16;
  LinearizedIntegrityWindow window;
  EpochTransaction transaction;
  const auto result = HypothesisGenerator(config).actionsForPlausibleSetV1(
      window, transaction, &models, {one, two});
  EXPECT_GE(result.census.generated, 3u);
  // The two requests have different generated action IDs, so they are
  // operation-equivalent but not semantic duplicates and must both survive.
  EXPECT_EQ(result.actions.size(), result.census.generated);
  EXPECT_TRUE(result.census.omitted_actions.empty());
  ASSERT_GE(result.actions.size(), 3u);
  EXPECT_EQ(exactActionOperationIdentityV1(result.actions[1]),
            exactActionOperationIdentityV1(result.actions[2]));
  EXPECT_NE(exactActionSemanticIdentityV1(result.actions[1]),
            exactActionSemanticIdentityV1(result.actions[2]));
  const auto validation = validateActionSearchCensusV1(result);
  EXPECT_TRUE(validation.valid) << validation.reason;
  EXPECT_TRUE(validation.exhaustive);
}

TEST(P006ActionSearchRed, ZeroCapAndOversizeInputFailClosedWithoutOverflow) {
  std::vector<ExclusionAction> generated{action(1, 1.0, 0.0),
                                         action(2, 2.0, 0.0)};
  const auto result = censusAndCapActionsV1(generated, 0);
  EXPECT_TRUE(result.actions.empty());
  EXPECT_EQ(result.census.generated, 2u);
  EXPECT_EQ(result.census.evaluated, 0u);
  EXPECT_EQ(result.census.omitted, 2u);
  EXPECT_FALSE(result.census.exhaustive);
  for (const auto& omitted : result.census.omitted_actions) {
    EXPECT_FALSE(omitted.proven_safe);
    EXPECT_FALSE(omitted.operation_identity.empty());
  }
}

TEST(P006ActionSearch, ManagerRefusesBeforeWinnerOrRiskClaim) {
  std::vector<ExclusionAction> generated;
  for (std::uint64_t i = 0; i < 130; ++i) {
    generated.push_back(action(i + 1, static_cast<double>(i + 1), 0.25));
  }
  const auto search = censusAndCapActionsV1(generated, 128);
  DetectorResultV2 alarm;
  alarm.numerically_valid = true;
  alarm.passed = false;
  std::vector<CandidateEvaluation> candidates;
  for (const auto& generated_action : search.actions) {
    CandidateEvaluation candidate;
    candidate.action = generated_action;
    candidate.valid = true;
    candidate.post_detector_passed = true;
    candidate.hpl_m = candidate.vpl_m = 0.1;
    candidates.push_back(std::move(candidate));
  }
  FdeRiskDecisionV1 risk;
  FdeDecisionContextV2 context;
  context.v1.risk_result = &risk;
  bindActionSearch(search, &context);
  const auto decision = FdeManager().decide(
      alarm, {}, {}, &candidates, {}, RiskBudgetV2{}, &context);
  EXPECT_EQ(decision.status, FdeStatus::SearchIncomplete);
  EXPECT_FALSE(decision.commit_allowed);
  EXPECT_FALSE(decision.integrity_available);
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_NE(decision.reason.find("generated=130"), std::string::npos);
  EXPECT_NE(decision.reason.find("evaluated=128"), std::string::npos);
  EXPECT_NE(decision.reason.find("omitted=2"), std::string::npos);
  EXPECT_FALSE(risk.complete_bound_closes);
  EXPECT_FALSE(risk.formal_eligible);
}

TEST(P006O07, ProductionBackedAmplitudeOnsetAxisJointGrid) {
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.rank_tolerance = 1e-10;
  detector_risk.max_condition_number = 1e10;
  std::size_t alarms = 0;
  std::size_t recoverable = 0;
  std::size_t refusals = 0;
  std::size_t healthy_winners = 0;
  std::size_t rank_routes = 0;
  std::size_t dense_routes = 0;
  std::size_t evidence_refusals = 0;
  std::size_t profile_refusals = 0;
  std::size_t plausible_hypotheses_seen = 0;
  std::size_t plausible_joint_hypotheses_seen = 0;
  resetCandidateRouteWorkV1();
  for (const double amplitude : {0.25, 4.0, 50.0}) {
    for (const std::size_t onset : {std::size_t{0}, std::size_t{31},
                                    std::size_t{127}}) {
      for (const int axis : {0, 1, 2}) {
        for (const bool joint : {false, true}) {
          SCOPED_TRACE(::testing::Message() << "amplitude=" << amplitude
              << " onset=" << onset << " axis=" << axis
              << " joint=" << joint);
          // The onset is an actual raw-IMU sample boundary (0/31/127), the
          // axis is an actual accelerometer channel and joint=true also
          // injects a raw UWB range.  Each of the 54 cells constructs its own
          // production transaction/window and analytic IMU subspace.
          auto inputs = realGeneratedInputs(amplitude, onset, axis, joint);
          ASSERT_TRUE(inputs.window().model_valid) << inputs.window().reason;
          ASSERT_TRUE(inputs.imu_subspaces.analytic_input_valid);
          ASSERT_TRUE(inputs.imu_subspaces.analytic_computation_valid);

          HypothesisGeneratorConfig generator_config;
          generator_config.max_candidate_count = 256;
          generator_config.max_exclusion_cardinality = 2;
          generator_config.double_faults_enabled = true;
          generator_config.max_model_cardinality = 2;
          // This is the production mode/hypothesis entrypoint under review,
          // not a hand-filled FaultModeBasis/FaultUnit/Hypothesis fixture.
          auto models = HypothesisGenerator(generator_config).generate(
              inputs.window(), inputs.transaction, inputs.imu_subspaces,
              inputs.bridge_block);
          ASSERT_FALSE(models.modes.empty());
          ASSERT_FALSE(models.hypotheses.empty());
          const auto imu_mode = std::find_if(
              models.modes.begin(), models.modes.end(), [&](const auto& mode) {
                return mode.sensor == SensorType::ImuAccelerometer &&
                    mode.axis == axis &&
                    mode.affected_groups == std::vector<FactorGroupId>{
                        inputs.transaction.imu_group.id};
              });
          ASSERT_NE(imu_mode, models.modes.end());
          auto target_it = std::find_if(
              models.hypotheses.begin(), models.hypotheses.end(),
              [&](const FaultHypothesisV2& hypothesis) {
                if (std::find(hypothesis.modes.begin(), hypothesis.modes.end(),
                              imu_mode->id) == hypothesis.modes.end()) {
                  return false;
                }
                const bool has_uwb = std::any_of(
                    hypothesis.modes.begin(), hypothesis.modes.end(),
                    [&](FaultModeId id) {
                      return models.modes.at(id.value() - 1).sensor ==
                          SensorType::Uwb;
                    });
                return hypothesis.modes.size() == (joint ? 2u : 1u) &&
                    has_uwb == joint;
              });
          ASSERT_NE(target_it, models.hypotheses.end());
          const FaultHypothesisV2 target = *target_it;

          const auto& window = inputs.window();
          const auto all_in = JointWindowDetector().evaluate(
              window, detector_risk);
          ASSERT_TRUE(all_in.numerically_valid) << all_in.reason;
          HypothesisEvaluationConfig evidence_config;
          ASSERT_TRUE(window.numerics);
          ASSERT_TRUE(window.numerics->valid);
          EXPECT_EQ(window.numerics->content_fingerprint,
                    integrityWindowFingerprint(window));
          ASSERT_TRUE(window.numerics->information_factorization);
          std::string numerical_proof_reason;
          const bool numerical_proof_valid = validateFrozenWindowNumericalProof(
              window, *window.numerics, &numerical_proof_reason);
          evidence_config.rank_tolerance =
              window.numerics->numerical_contract.rank_tolerance;
          evidence_config.max_condition_number =
              window.numerics->numerical_contract.max_condition_number;
          // Run the same production evidence/profile/plausibility component as
          // IntegrityMonitor.  Nothing below writes plausible/profile_j.
          auto evidence = HypothesisEvidenceEvaluator(evidence_config)
              .evaluateAll(window, models.modes, &models.hypotheses,
                           all_in.squared_threshold);
          if (numerical_proof_valid) {
            ASSERT_EQ(evidence.size(), models.hypotheses.size());
          } else {
            EXPECT_TRUE(evidence.empty()) << numerical_proof_reason;
            ++evidence_refusals;
          }
          const auto plausible = completePlausibleHypotheses(
              models.hypotheses, evidence);
          plausible_hypotheses_seen += plausible.size();
          bool plausible_profiles_valid = true;
          for (const auto id : plausible) {
            const auto found = std::find_if(
                evidence.begin(), evidence.end(),
                [&](const FaultModeEvidence& item) {
                  return item.hypothesis == id;
                });
            ASSERT_NE(found, evidence.end());
            EXPECT_TRUE(found->plausible);
            plausible_profiles_valid = plausible_profiles_valid &&
                found->profile_valid && std::isfinite(found->profile_j);
          }
          if (!plausible_profiles_valid) ++profile_refusals;

          auto generated = HypothesisGenerator(generator_config)
              .actionsForPlausibleSetV1(
                  inputs.window(), inputs.transaction, &models,
                  evidence);
          ASSERT_TRUE(validateActionSearchCensusV1(generated).valid);
          ASSERT_TRUE(generated.census.exhaustive);
          for (const auto id : plausible) {
            const auto hypothesis = std::find_if(
                models.hypotheses.begin(), models.hypotheses.end(),
                [&](const FaultHypothesisV2& item) { return item.id == id; });
            ASSERT_NE(hypothesis, models.hypotheses.end());
            if (hypothesis->modes.size() > 1) {
              ++plausible_joint_hypotheses_seen;
            }
            EXPECT_FALSE(hypothesis->affected_groups.empty());
            const bool generated_full_cover = std::any_of(
                generated.actions.begin(), generated.actions.end(),
                [&](const ExclusionAction& action) {
                  return std::all_of(
                      hypothesis->affected_groups.begin(),
                      hypothesis->affected_groups.end(),
                      [&](FactorGroupId group) {
                        return std::find(action.groups_to_remove.begin(),
                                         action.groups_to_remove.end(), group) !=
                            action.groups_to_remove.end();
                      });
                });
            EXPECT_TRUE(generated_full_cover) << id.value();
          }
          // Add one action genuinely materialized by the production mode-to-
          // action path, but outside the plausible set, as the wrong exclusion.
          ASSERT_GT(models.single_mode_actions.size(), 1u);
          std::set<std::uint64_t> required_groups;
          for (const auto id : plausible) {
            const auto hypothesis = std::find_if(
                models.hypotheses.begin(), models.hypotheses.end(),
                [&](const FaultHypothesisV2& item) { return item.id == id; });
            ASSERT_NE(hypothesis, models.hypotheses.end());
            for (const auto group : hypothesis->affected_groups) {
              required_groups.insert(group.value());
            }
          }
          const auto wrong_action = std::find_if(
              models.single_mode_actions.begin(),
              models.single_mode_actions.end(), [&](const auto& candidate) {
                return !required_groups.empty() && std::any_of(
                    required_groups.begin(), required_groups.end(),
                    [&](std::uint64_t group) {
                      return std::find(candidate.groups_to_remove.begin(),
                                       candidate.groups_to_remove.end(),
                                       FactorGroupId(group)) ==
                          candidate.groups_to_remove.end();
                    });
              });
          std::vector<ExclusionAction> raw_actions = generated.actions;
          std::optional<ExclusionActionId> wrong_action_id;
          if (wrong_action != models.single_mode_actions.end()) {
            ExclusionAction wrong_request = *wrong_action;
            const auto max_id = std::max_element(
                raw_actions.begin(), raw_actions.end(),
                [](const auto& left, const auto& right) {
                  return left.id < right.id;
                })->id.value();
            wrong_request.id = ExclusionActionId(max_id + 1);
            wrong_action_id = wrong_request.id;
            raw_actions.push_back(std::move(wrong_request));
          }
          const auto search = censusAndCapActionsV1(raw_actions, 256);
          ASSERT_TRUE(validateActionSearchCensusV1(search).valid);
          ASSERT_TRUE(search.census.exhaustive);

          RankUpdateConfig rank;
          rank.rank_tolerance =
              window.numerics->numerical_contract.rank_tolerance;
          rank.max_condition_number =
              window.numerics->numerical_contract.max_condition_number;
          rank.max_linearization_step_norm = 100.0;
          rank.enable_shared_cache = false;
          const RankUpdateEvaluator evaluator(rank);
          const auto base = evaluator.factorizeOnce(window, search.actions);
          ASSERT_TRUE(base.valid) << base.reason;
          const CandidateEvaluationRouterV1 router(rank);
          const DenseCandidateOracle dense_oracle(rank);
          std::vector<CandidateEvaluation> candidates;
          std::vector<CandidateEvaluation> dense_candidates;
          std::size_t expected_index = search.actions.size();
          std::optional<std::size_t> wrong_index;
          std::size_t cell_rank_routes = 0;
          std::size_t cell_dense_routes = 0;
          for (std::size_t index = 0; index < search.actions.size(); ++index) {
            const auto& action = search.actions[index];
            CandidateRouteSafetyV1 route;
            auto candidate = router.evaluate(window, base, action, nullptr,
                                             &route);
            if (route.rank_path_safe) {
              ++rank_routes;
              ++cell_rank_routes;
            } else {
              ++dense_routes;
              ++cell_dense_routes;
            }
            const auto post = JointWindowDetector().evaluateCandidate(
                window, candidate, detector_risk);
            auto dense_reference = dense_oracle.evaluate(window, action);
            const auto dense_post = JointWindowDetector().evaluateCandidate(
                window, dense_reference, detector_risk);
            dense_reference.post_detector_passed = dense_post.passed;
            if (route.rank_path_safe) {
              EXPECT_EQ(candidate.valid, dense_reference.valid);
              EXPECT_EQ(candidate.rank, dense_reference.rank);
              EXPECT_EQ(candidate.dof, dense_reference.dof);
              ASSERT_EQ(candidate.state_increment.size(),
                        dense_reference.state_increment.size());
              EXPECT_TRUE(candidate.state_increment.isApprox(
                  dense_reference.state_increment, 1e-7));
              const auto routed_covariance = candidate.covarianceTimes(
                  Eigen::MatrixXd::Identity(window.H.cols(), window.H.cols()));
              ASSERT_EQ(routed_covariance.rows(),
                        dense_reference.covariance.rows());
              ASSERT_EQ(routed_covariance.cols(),
                        dense_reference.covariance.cols());
              EXPECT_TRUE(routed_covariance.isApprox(
                  dense_reference.covariance, 1e-7));
              EXPECT_EQ(post.numerically_valid, dense_post.numerically_valid);
              EXPECT_EQ(post.passed, dense_post.passed);
            }
            candidate.post_detector_passed = post.passed;
            if (candidate.valid && post.numerically_valid && post.passed) {
              const auto pl = ProtectionLevelV2().compute(
                  window, candidate, post, &models.hypotheses, RiskBudgetV2{});
              candidate.valid = candidate.valid && pl.model_valid;
              candidate.pl_xyz_m = pl.pl_xyz_m;
              candidate.hpl_m = pl.hpl_m;
              candidate.vpl_m = pl.vpl_m;
            }
            if (dense_reference.valid && dense_post.numerically_valid &&
                dense_post.passed) {
              const auto dense_pl = ProtectionLevelV2().compute(
                  window, dense_reference, dense_post, &models.hypotheses,
                  RiskBudgetV2{});
              dense_reference.valid =
                  dense_reference.valid && dense_pl.model_valid;
              dense_reference.pl_xyz_m = dense_pl.pl_xyz_m;
              dense_reference.hpl_m = dense_pl.hpl_m;
              dense_reference.vpl_m = dense_pl.vpl_m;
            }
            bool covers = true;
            for (const auto group : required_groups) {
              covers = covers && std::find(action.groups_to_remove.begin(),
                  action.groups_to_remove.end(), FactorGroupId(group)) !=
                  action.groups_to_remove.end();
            }
            if (covers && (required_groups.empty()
                    ? action.exclusion_cardinality == 0
                    : true)) {
              expected_index = index;
            }
            if (wrong_action_id && action.id == *wrong_action_id) {
              wrong_index = index;
            }
            candidates.push_back(std::move(candidate));
            dense_candidates.push_back(std::move(dense_reference));
          }
          if (amplitude < 50.0 && numerical_proof_valid) {
            EXPECT_GT(cell_rank_routes, 0u);
          } else {
            EXPECT_GT(cell_dense_routes, 0u);
          }
          ASSERT_LT(expected_index, candidates.size());
          if (wrong_action_id) {
            ASSERT_TRUE(wrong_index.has_value());
          }
          const auto oracle = independentRetainedSvd(
              window, candidates[expected_index].action);
          auto candidate_covariance =
              candidates[expected_index].covarianceTimes(
                  Eigen::MatrixXd::Identity(window.H.cols(), window.H.cols()));
          if (candidate_covariance.size() == 0) {
            candidate_covariance = candidates[expected_index].covariance;
          }
          ASSERT_EQ(candidates[expected_index].state_increment.size(),
                    oracle.first.size());
          ASSERT_EQ(candidate_covariance.rows(), oracle.second.rows());
          ASSERT_EQ(candidate_covariance.cols(), oracle.second.cols());
          EXPECT_TRUE(candidates[expected_index].state_increment.isApprox(
              oracle.first, 1e-8));
          EXPECT_TRUE(candidate_covariance.isApprox(oracle.second, 1e-8));

          RiskBudgetV2 risk;
          risk.p_hmi_total = 1.0;
          risk.nominal_axis_tail = 0.0;
          risk.p_nm = 0.0;
          risk.calibration_id = "p006-independent-qualified-grid";
          FdeRiskDecisionV1 risk_result;
          FdeDecisionContextV2 context;
          context.v1.risk_result = &risk_result;
          bindActionSearch(search, &context);
          const auto decision = FdeManager().decide(
              all_in, models.hypotheses, evidence, &candidates, {}, risk,
              &context);
          FdeRiskDecisionV1 dense_risk_result;
          FdeDecisionContextV2 dense_context;
          dense_context.v1.risk_result = &dense_risk_result;
          bindActionSearch(search, &dense_context);
          const auto dense_decision = FdeManager().decide(
              all_in, models.hypotheses, evidence, &dense_candidates, {}, risk,
              &dense_context);
          EXPECT_EQ(decision.status, dense_decision.status);
          EXPECT_EQ(decision.commit_allowed, dense_decision.commit_allowed);
          EXPECT_EQ(decision.integrity_available,
                    dense_decision.integrity_available);
          EXPECT_EQ(decision.plausible_hypotheses,
                    dense_decision.plausible_hypotheses);
          EXPECT_EQ(decision.candidate_dispositions,
                    dense_decision.candidate_dispositions);
          EXPECT_EQ(decision.selected_action.has_value(),
                    dense_decision.selected_action.has_value());
          if (decision.selected_action && dense_decision.selected_action) {
            EXPECT_EQ(decision.selected_action->id,
                      dense_decision.selected_action->id);
            EXPECT_EQ(decision.selected_action->action_model_id,
                      dense_decision.selected_action->action_model_id);
          }
          EXPECT_EQ(risk_result.complete_bound_closes,
                    dense_risk_result.complete_bound_closes);
          EXPECT_EQ(risk_result.all_terms_validated,
                    dense_risk_result.all_terms_validated);
          if (decision.candidate_dispositions.empty()) {
            EXPECT_FALSE(decision.commit_allowed);
            EXPECT_FALSE(decision.selected_action.has_value());
          } else {
            ASSERT_EQ(decision.candidate_dispositions.size(), candidates.size());
          }
          EXPECT_EQ(decision.plausible_hypotheses.size(),
                    all_in.passed ? 0u : plausible.size());
          if (!numerical_proof_valid && !all_in.passed) {
            EXPECT_FALSE(decision.commit_allowed);
            EXPECT_FALSE(decision.selected_action.has_value());
          }
          if (!plausible_profiles_valid && !all_in.passed) {
            EXPECT_EQ(decision.status, FdeStatus::ModelInvalid);
            EXPECT_FALSE(decision.commit_allowed);
            EXPECT_FALSE(decision.selected_action.has_value());
          }
          if (!all_in.passed) {
            ++alarms;
            if (wrong_index && !decision.candidate_dispositions.empty()) {
              EXPECT_NE(decision.candidate_dispositions[*wrong_index],
                        static_cast<int>(
                            CandidateDisposition::UseInReferenceEstimate));
            }
            bool winner_covers_all = decision.selected_action.has_value();
            if (decision.selected_action) {
              for (const auto id : decision.plausible_hypotheses) {
                const auto hypothesis = std::find_if(
                    models.hypotheses.begin(), models.hypotheses.end(),
                    [&](const FaultHypothesisV2& item) {
                      return item.id == id;
                    });
                ASSERT_NE(hypothesis, models.hypotheses.end());
                for (const auto group : hypothesis->affected_groups) {
                  winner_covers_all = winner_covers_all &&
                      std::find(decision.selected_action->groups_to_remove.begin(),
                                decision.selected_action->groups_to_remove.end(),
                                group) !=
                          decision.selected_action->groups_to_remove.end();
                }
              }
            }
            if (winner_covers_all) {
              ++recoverable;
            } else {
              ++refusals;
            }
          } else {
            if (decision.selected_action) {
              ++healthy_winners;
              EXPECT_EQ(decision.selected_action->action_model_id, "KEEP_ALL");
            } else {
              ++refusals;
              EXPECT_FALSE(decision.commit_allowed) << decision.reason;
            }
          }
        }
      }
    }
  }
  EXPECT_GT(alarms, 0u);
  EXPECT_GT(rank_routes, 0u);
  EXPECT_GT(dense_routes, 0u);
  EXPECT_GT(evidence_refusals, 0u);
  EXPECT_GT(profile_refusals, 0u);
  const auto route_work = candidateRouteWorkV1();
  EXPECT_EQ(route_work.routed_rank, rank_routes);
  EXPECT_EQ(route_work.routed_dense, dense_routes);
  EXPECT_EQ(recoverable + refusals + healthy_winners, 54u);
  std::cout << "P006_GRID cells=54 alarms=" << alarms
            << " recoverable=" << recoverable
            << " refusals=" << refusals
            << " healthy_winners=" << healthy_winners
            << " evidence_refusals=" << evidence_refusals
            << " profile_refusals=" << profile_refusals
            << " plausible=" << plausible_hypotheses_seen
            << " plausible_joint=" << plausible_joint_hypotheses_seen
            << " rank_routes=" << rank_routes
            << " dense_routes=" << dense_routes << '\n';
}

TEST(P006O07, RouterFailsDenseForLargeIllConditionedAndUnknownReplacement) {
  RankUpdateConfig rank;
  rank.rank_tolerance = 1e-10;
  rank.max_condition_number = 1e10;
  rank.max_linearization_step_norm = 0.25;
  rank.enable_shared_cache = false;
  const RankUpdateEvaluator evaluator(rank);
  const CandidateEvaluationRouterV1 router(rank);

  auto large = productionOracleWindow(50.0);
  ASSERT_TRUE(large.model_valid) << large.reason;
  ExclusionAction replacement;
  replacement.id = ExclusionActionId(7001);
  replacement.groups_to_remove = {FactorGroupId(3)};
  replacement.groups_to_add = {FactorGroupId(30)};
  replacement.exclusion_cardinality = 1;
  replacement.action_model_id = "LARGE_REPLACEMENT";
  auto added = large.blocks.back();
  added.group_id = FactorGroupId(30);
  replacement.added_blocks = {added};
  auto large_base = evaluator.factorizeOnce(large, {replacement});
  ASSERT_TRUE(large_base.valid) << large_base.reason;
  CandidateRouteSafetyV1 large_safety;
  const auto large_result = router.evaluate(
      large, large_base, replacement, nullptr, &large_safety);
  EXPECT_FALSE(large_safety.rank_path_safe);
  EXPECT_EQ(large_result.diagnostics.numerical_path, "DENSE_ROUTER_EXACT");
  EXPECT_NE(large_result.diagnostics.fallback_reason.find(
                "RANK_PRECONDITION"), std::string::npos);

  auto ill_conditioned = productionOracleWindow(0.1);
  for (auto& block : ill_conditioned.blocks) {
    if (block.jacobian_whitened.cols() == 2) {
      block.jacobian_whitened.col(1) =
          block.jacobian_whitened.col(0) * (1.0 + 1e-14);
      block.jacobian_raw = block.jacobian_whitened;
    }
  }
  finalizeIntegrityWindow(&ill_conditioned, rank.rank_tolerance,
                          rank.max_condition_number);
  const auto ill_base = evaluator.factorizeOnce(ill_conditioned);
  CandidateRouteSafetyV1 ill_safety;
  const auto ill_result = router.evaluate(
      ill_conditioned, ill_base, ExclusionAction{}, nullptr, &ill_safety);
  EXPECT_FALSE(ill_safety.rank_path_safe);
  EXPECT_EQ(ill_result.diagnostics.numerical_path, "DENSE_ROUTER_EXACT");

  auto mapped = productionOracleWindow(0.1);
  const auto mapped_base = evaluator.factorizeOnce(mapped);
  ASSERT_TRUE(mapped_base.valid);
  ExclusionAction unknown;
  unknown.id = ExclusionActionId(7002);
  unknown.groups_to_remove = {FactorGroupId(999999)};
  unknown.exclusion_cardinality = 1;
  CandidateRouteSafetyV1 mapping_safety;
  const auto mapping_result = router.evaluate(
      mapped, mapped_base, unknown, nullptr, &mapping_safety);
  EXPECT_FALSE(mapping_safety.rank_path_safe);
  EXPECT_EQ(mapping_safety.reason, "RANK_PRECONDITION_REMOVAL_MAPPING");
  EXPECT_EQ(mapping_result.diagnostics.numerical_path,
            "DENSE_ROUTER_EXACT");
}

TEST(P006O07, RouterRecoversCatchableRankExceptionWithExactTerminal) {
  auto window = productionOracleWindow(0.01);
  ASSERT_TRUE(window.model_valid) << window.reason;
  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  RankUpdateConfig rank;
  rank.rank_tolerance = 1e-10;
  rank.max_condition_number = 1e10;
  rank.max_linearization_step_norm = 100.0;
  rank.enable_shared_cache = false;
  const RankUpdateEvaluator evaluator(rank);
  const auto base = evaluator.factorizeOnce(window, {keep});
  ASSERT_TRUE(base.valid) << base.reason;
  const DenseCandidateOracle dense(rank);
  const auto expected = dense.evaluate(window, keep);
  resetCandidateRouteWorkV1();
  const CandidateEvaluationRouterV1 router(
      rank, [](const BaseCandidateKernel&, const ExclusionAction&,
               RankUpdateScratch*) -> CandidateEvaluation {
        throw std::runtime_error("injected rank evaluator failure");
      });
  CandidateRouteSafetyV1 safety;
  const auto actual = router.evaluate(window, base, keep, nullptr, &safety);
  EXPECT_FALSE(safety.rank_path_safe);
  EXPECT_NE(safety.reason.find("RANK_EXCEPTION_DENSE_EXACT"),
            std::string::npos);
  EXPECT_EQ(actual.valid, expected.valid);
  EXPECT_EQ(actual.rank, expected.rank);
  EXPECT_EQ(actual.dof, expected.dof);
  EXPECT_TRUE(actual.state_increment.isApprox(expected.state_increment, 1e-15));
  EXPECT_TRUE(actual.covariance.isApprox(expected.covariance, 1e-15));
  EXPECT_EQ(actual.diagnostics.numerical_path, "DENSE_ROUTER_EXACT");
  EXPECT_NE(actual.diagnostics.fallback_reason.find(
                "RANK_EXCEPTION_DENSE_EXACT"), std::string::npos);
  DetectorRiskContext detector_risk;
  detector_risk.rank_tolerance = rank.rank_tolerance;
  detector_risk.max_condition_number = rank.max_condition_number;
  const auto post = JointWindowDetector().evaluateCandidate(
      window, actual, detector_risk);
  EXPECT_TRUE(post.numerically_valid) << post.reason;
  const auto search = censusAndCapActionsV1({keep}, 1);
  const auto search_validation = validateActionSearchCensusV1(search);
  EXPECT_TRUE(search_validation.valid) << search_validation.reason;
  EXPECT_TRUE(search_validation.exhaustive);
  const auto work = candidateRouteWorkV1();
  EXPECT_EQ(work.routed_rank, 0u);
  EXPECT_EQ(work.routed_dense, 1u);
  EXPECT_EQ(work.rank_exceptions_recovered, 1u);

  // An invalid dense terminal remains invalid: catching an optimization
  // exception never launders an uncertified candidate into a valid one.
  ExclusionAction invalid_action = keep;
  invalid_action.id = ExclusionActionId(2);
  invalid_action.groups_to_remove = {FactorGroupId(999999)};
  invalid_action.exclusion_cardinality = 1;
  CandidateRouteSafetyV1 invalid_safety;
  const auto invalid = router.evaluate(
      window, base, invalid_action, nullptr, &invalid_safety);
  EXPECT_FALSE(invalid.valid);
  EXPECT_FALSE(invalid_safety.rank_path_safe);
}

TEST(P006O07, Production130IncludesFullUnionBeyondCap128) {
  auto fixture = completeSemanticActionFixture();
  ASSERT_TRUE(fixture.window.model_valid) << fixture.window.reason;
  ASSERT_TRUE(fixture.all_in.numerically_valid) << fixture.all_in.reason;
  ASSERT_FALSE(fixture.all_in.passed);
  ASSERT_EQ(fixture.hypotheses.size(), 128u);
  ASSERT_EQ(fixture.evidence.size(), fixture.hypotheses.size());
  ASSERT_EQ(completePlausibleHypotheses(fixture.hypotheses, fixture.evidence)
                .size(), 128u);
  const auto generated = HypothesisGenerator(fixture.generator_config)
      .actionsForPlausibleSetV1(fixture.window, fixture.transaction,
                                &fixture.models, fixture.evidence);
  ASSERT_EQ(generated.actions.size(), 130u);
  ASSERT_EQ(generated.generated_snapshot.generator_identity,
            "HypothesisGenerator::actionsForPlausibleSetV1/production-v1");
  ASSERT_EQ(generated.actions[128].id.value(), 129u);
  ASSERT_EQ(generated.actions[129].id.value(), 130u);
  EXPECT_EQ(generated.actions[129].action_model_id,
            "FULL_PLAUSIBLE_UNION_RECOVERY");
  EXPECT_EQ(generated.actions[129].covered_modes.size(), 128u);
  auto limited_config = fixture.generator_config;
  limited_config.max_candidate_count = 128;
  const auto limited = HypothesisGenerator(limited_config)
      .actionsForPlausibleSetV1(fixture.window, fixture.transaction,
                                &fixture.models, fixture.evidence);
  EXPECT_FALSE(limited.census.exhaustive);
  ASSERT_EQ(limited.census.omitted_actions.size(), 2u);
  EXPECT_EQ(limited.census.omitted_actions[0].action_id, 129u);
  EXPECT_EQ(limited.census.omitted_actions[0].reason,
            "RESOURCE_CAP_UNPROVEN");
  EXPECT_EQ(limited.census.omitted_actions[1].action_id, 130u);
  EXPECT_EQ(limited.census.omitted_actions[1].reason,
            "RESOURCE_CAP_UNPROVEN");
}

TEST(P006O07, ProductionKernelDetectorPlAndFdeFindBeyondCapAction) {
  auto fixture = completeSemanticActionFixture();
  ASSERT_TRUE(fixture.window.model_valid) << fixture.window.reason;
  const auto uncapped = HypothesisGenerator(fixture.generator_config)
      .actionsForPlausibleSetV1(fixture.window, fixture.transaction,
                                &fixture.models, fixture.evidence);
  ASSERT_EQ(uncapped.actions.size(), 130u);
  ASSERT_TRUE(uncapped.census.exhaustive);
  ASSERT_TRUE(validateActionSearchCensusV1(uncapped).valid);
  ASSERT_EQ(uncapped.actions[128].id.value(), 129u);
  ASSERT_EQ(uncapped.actions[129].id.value(), 130u);
  for (const auto& action : uncapped.actions) {
    for (const auto group : action.groups_to_remove) {
      EXPECT_TRUE(std::any_of(fixture.window.blocks.begin(),
                              fixture.window.blocks.end(),
          [&](const LinearizedFactorBlock& block) {
            return block.group_id == group;
          })) << group.value();
    }
  }

  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.rank_tolerance = 1e-10;
  detector_risk.max_condition_number = 1e10;
  const auto all_in = fixture.all_in;
  ASSERT_TRUE(all_in.numerically_valid) << all_in.reason;
  ASSERT_FALSE(all_in.passed) << all_in.reason;

  RankUpdateConfig rank;
  rank.rank_tolerance = 1e-10;
  rank.max_condition_number = 1e10;
  rank.max_linearization_step_norm = 100.0;
  RankUpdateEvaluator evaluator(rank);
  const CandidateEvaluationRouterV1 router(rank);
  RiskBudgetV2 risk;
  risk.p_hmi_total = 1.0;
  risk.nominal_axis_tail = 1e-5;
  risk.p_nm = 0.0;
  risk.calibration_id = "p006-independent-qualified-130";

  struct EvaluationBundle {
    FdeDecision decision;
    FdeRiskDecisionV1 risk;
    std::vector<CandidateEvaluation> candidates;
    std::vector<IndependentDenseActionOracle> independent;
  };
  auto evaluate = [&](const ActionSearchResultV1& search) {
    EvaluationBundle bundle;
    const auto base = evaluator.factorizeOnce(fixture.window, search.actions);
    EXPECT_TRUE(base.valid) << base.reason;
    std::size_t production_pre_risk_passes = 0;
    std::size_t independent_pre_risk_passes = 0;
    for (const auto& candidate_action : search.actions) {
      auto candidate = router.evaluate(
          fixture.window, base, candidate_action);
      const auto post = JointWindowDetector().evaluateCandidate(
          fixture.window, candidate, detector_risk);
      candidate.post_detector_passed = post.passed;
      auto projected_remaining = projectCompleteSemanticSet(
          fixture, candidate_action);
      const std::size_t expected_remaining = candidate_action.id.value() == 1
          ? 128u : (candidate_action.id.value() == 130 ? 0u : 127u);
      EXPECT_EQ(projected_remaining.size(), expected_remaining);
      const auto pl = ProtectionLevelV2().compute(
          fixture.window, candidate, post, &projected_remaining, risk);
      if (candidate.valid && post.numerically_valid && post.passed) {
        candidate.pl_xyz_m = pl.pl_xyz_m;
        candidate.hpl_m = pl.hpl_m;
        candidate.vpl_m = pl.vpl_m;
        candidate.valid = candidate.valid && pl.model_valid;
        const bool covers_all = std::all_of(
            fixture.hypotheses.begin(), fixture.hypotheses.end(),
            [&](const FaultHypothesisV2& hypothesis) {
              return std::all_of(hypothesis.modes.begin(), hypothesis.modes.end(),
                  [&](FaultModeId mode) {
                    return std::find(candidate_action.covered_modes.begin(),
                                     candidate_action.covered_modes.end(), mode) !=
                        candidate_action.covered_modes.end();
                  });
            });
        if (pl.model_valid && covers_all) ++production_pre_risk_passes;
      }
      const auto independent = independentDenseActionOracle(
          fixture, candidate_action, detector_risk, risk);
      EXPECT_EQ(independent.post_passed, post.passed)
          << candidate_action.id.value();
      EXPECT_EQ(independent.covers_complete_plausible,
                candidate_action.id.value() == 130)
          << candidate_action.id.value();
      EXPECT_TRUE(independent.saw_correlated_covariance);
      EXPECT_TRUE(independent.saw_nonzero_raw_rhs);
      EXPECT_LT(independent.production_whitening_h_error, 1e-12);
      EXPECT_LT(independent.production_whitening_z_error, 1e-12);
      if (candidate.valid || independent.valid) {
        EXPECT_EQ(candidate.rank, independent.rank);
        EXPECT_EQ(candidate.dof, independent.dof);
        EXPECT_NEAR(candidate.statistic, independent.statistic, 1e-9);
        EXPECT_TRUE(candidate.state_increment.isApprox(
            independent.state, 1e-11));
        const Eigen::MatrixXd candidate_covariance = candidate.covarianceTimes(
            Eigen::MatrixXd::Identity(fixture.window.H.cols(),
                                      fixture.window.H.cols()));
        EXPECT_TRUE(candidate_covariance.isApprox(
            independent.covariance, 1e-11));
      }
      if (independent.post_passed && independent.covers_complete_plausible &&
          independent.pl_valid) {
        ++independent_pre_risk_passes;
        EXPECT_TRUE(pl.model_valid) << pl.reason;
        EXPECT_TRUE(pl.pl_xyz_m.isApprox(independent.pl, 1e-8));
        EXPECT_NEAR(pl.hpl_m, independent.hpl, 1e-8);
        EXPECT_NEAR(pl.vpl_m, independent.vpl, 1e-8);
      }
      bundle.independent.push_back(independent);
      bundle.candidates.push_back(std::move(candidate));
    }
    EXPECT_EQ(production_pre_risk_passes, 1u);
    EXPECT_EQ(independent_pre_risk_passes, 1u);
    FdeDecisionContextV2 context;
    context.v1.risk_result = &bundle.risk;
    bindActionSearch(search, &context);
    bundle.decision = FdeManager().decide(
        all_in, fixture.hypotheses, fixture.evidence,
        &bundle.candidates, {}, risk, &context);
    return bundle;
  };

  const auto forward = evaluate(uncapped);
  ASSERT_EQ(forward.candidates.size(), 130u);
  ASSERT_TRUE(forward.candidates[129].valid)
      << forward.candidates[129].reason;
  ASSERT_TRUE(forward.candidates[129].post_detector_passed);
  ASSERT_TRUE(forward.independent[129].post_passed);
  ASSERT_TRUE(forward.independent[129].pl_valid);
  ASSERT_TRUE(forward.independent[129].covers_complete_plausible);
  ASSERT_EQ(forward.candidates[129].action.id.value(), 130u);
  for (std::size_t index = 0; index < forward.candidates.size(); ++index) {
    if (index == 129) continue;
    EXPECT_FALSE(forward.independent[index].post_passed &&
                 forward.independent[index].covers_complete_plausible &&
                 forward.independent[index].pl_valid)
        << forward.candidates[index].action.id.value();
  }
  // The D09 oracle establishes the sole complete, numerically viable recovery
  // candidate independently;
  // the unchanged P0-04 contract then honestly refuses commit because this
  // synthetic fixture has no production qualification artifact for the
  // omitted/envelope/escape risk terms.  No test-only risk override exists.
  EXPECT_FALSE(forward.decision.selected_action.has_value());
  EXPECT_EQ(forward.decision.status, FdeStatus::RiskBudgetInvalid);
  EXPECT_FALSE(forward.decision.commit_allowed);
  EXPECT_FALSE(forward.risk.complete_bound_closes);
  EXPECT_FALSE(forward.risk.all_terms_validated);
  EXPECT_EQ(forward.decision.plausible_hypotheses.size(), 128u);
  ASSERT_EQ(forward.decision.candidate_dispositions.size(), 130u);
  // Production identifies exactly the omitted full union as its pre-risk
  // reference candidate, then honestly refuses final selection/commit because
  // the unchanged P0-04 bound is not qualified for this synthetic fixture.
  EXPECT_EQ(forward.decision.candidate_dispositions[129],
            static_cast<int>(CandidateDisposition::UseInReferenceEstimate));
  EXPECT_EQ(std::count(forward.decision.candidate_dispositions.begin(),
                       forward.decision.candidate_dispositions.end(),
                       static_cast<int>(CandidateDisposition::UseInReferenceEstimate)),
            1);

  for (int permutation = 0; permutation < 3; ++permutation) {
    auto reordered_raw = fixture.raw_actions;
    std::rotate(reordered_raw.begin(),
                reordered_raw.begin() + (17 + 23 * permutation),
                reordered_raw.end());
    if (permutation != 1) std::reverse(reordered_raw.begin(), reordered_raw.end());
    const auto reordered = censusAndCapActionsV1(
        reordered_raw, reordered_raw.size());
    ASSERT_TRUE(validateActionSearchCensusV1(reordered).valid);
    const auto actual_bundle = evaluate(reordered);
    EXPECT_FALSE(actual_bundle.decision.selected_action.has_value());
    EXPECT_EQ(actual_bundle.decision.status, forward.decision.status);
    EXPECT_EQ(actual_bundle.decision.commit_allowed,
              forward.decision.commit_allowed);
    EXPECT_EQ(actual_bundle.decision.integrity_available,
              forward.decision.integrity_available);
    EXPECT_EQ(actual_bundle.decision.plausible_hypotheses,
              forward.decision.plausible_hypotheses);
    EXPECT_EQ(actual_bundle.decision.candidate_dispositions,
              forward.decision.candidate_dispositions);
    EXPECT_EQ(actual_bundle.decision.selection_event_class_ids,
              forward.decision.selection_event_class_ids);
    EXPECT_EQ(actual_bundle.decision.selection_risk_proof_id,
              forward.decision.selection_risk_proof_id);
    EXPECT_EQ(actual_bundle.risk.complete_bound_closes,
              forward.risk.complete_bound_closes);
    EXPECT_EQ(actual_bundle.risk.all_terms_validated,
              forward.risk.all_terms_validated);
    EXPECT_DOUBLE_EQ(actual_bundle.risk.charged_total,
                     forward.risk.charged_total);
    ASSERT_EQ(actual_bundle.candidates.size(), forward.candidates.size());
    ASSERT_EQ(actual_bundle.independent.size(), forward.independent.size());
    for (std::size_t index = 0; index < forward.candidates.size(); ++index) {
      const auto& expected = forward.candidates[index];
      const auto& actual = actual_bundle.candidates[index];
      EXPECT_EQ(exactActionSemanticIdentityV1(actual.action),
                exactActionSemanticIdentityV1(expected.action));
      EXPECT_EQ(actual.valid, expected.valid);
      EXPECT_EQ(actual.post_detector_passed,
                expected.post_detector_passed);
      EXPECT_EQ(actual.rank, expected.rank);
      EXPECT_EQ(actual.dof, expected.dof);
      EXPECT_EQ(actual.reason, expected.reason);
      EXPECT_EQ(actual.diagnostics.numerical_path,
                expected.diagnostics.numerical_path);
      EXPECT_TRUE(actual.state_increment.isApprox(
          expected.state_increment, 1e-12));
      const Eigen::MatrixXd actual_covariance = actual.covarianceTimes(
          Eigen::MatrixXd::Identity(fixture.window.H.cols(),
                                    fixture.window.H.cols()));
      const Eigen::MatrixXd expected_covariance = expected.covarianceTimes(
          Eigen::MatrixXd::Identity(fixture.window.H.cols(),
                                    fixture.window.H.cols()));
      EXPECT_TRUE(actual_covariance.isApprox(expected_covariance, 1e-12));
      for (int axis = 0; axis < 3; ++axis) {
        if (std::isfinite(expected.pl_xyz_m(axis))) {
          EXPECT_NEAR(actual.pl_xyz_m(axis), expected.pl_xyz_m(axis), 1e-12);
        } else {
          EXPECT_EQ(std::isnan(actual.pl_xyz_m(axis)),
                    std::isnan(expected.pl_xyz_m(axis)));
          EXPECT_EQ(std::isinf(actual.pl_xyz_m(axis)),
                    std::isinf(expected.pl_xyz_m(axis)));
          if (std::isinf(expected.pl_xyz_m(axis))) {
            EXPECT_EQ(std::signbit(actual.pl_xyz_m(axis)),
                      std::signbit(expected.pl_xyz_m(axis)));
          }
        }
      }
      EXPECT_DOUBLE_EQ(actual.hpl_m, expected.hpl_m);
      EXPECT_DOUBLE_EQ(actual.vpl_m, expected.vpl_m);
      const auto& expected_oracle = forward.independent[index];
      const auto& actual_oracle = actual_bundle.independent[index];
      EXPECT_EQ(actual_oracle.post_passed, expected_oracle.post_passed);
      EXPECT_EQ(actual_oracle.pl_valid, expected_oracle.pl_valid);
      EXPECT_EQ(actual_oracle.covers_complete_plausible,
                expected_oracle.covers_complete_plausible);
      EXPECT_EQ(actual_oracle.remaining, expected_oracle.remaining);
      EXPECT_TRUE(actual_oracle.state.isApprox(expected_oracle.state, 1e-12));
      EXPECT_TRUE(actual_oracle.covariance.isApprox(
          expected_oracle.covariance, 1e-12));
      if (expected_oracle.pl_valid) {
        EXPECT_TRUE(actual_oracle.pl.isApprox(expected_oracle.pl, 1e-12));
      }
    }
  }

  auto limited_config = fixture.generator_config;
  limited_config.max_candidate_count = 128;
  auto limited = HypothesisGenerator(limited_config)
      .actionsForPlausibleSetV1(fixture.window, fixture.transaction,
                                &fixture.models, fixture.evidence);
  ASSERT_EQ(limited.census.generated, 130u);
  ASSERT_EQ(limited.census.evaluated, 128u);
  ASSERT_EQ(limited.census.omitted, 2u);
  EXPECT_EQ(limited.census.omitted_actions[0].action_id, 129u);
  abortIncompleteActionSearchBeforeEvaluationV1(&limited);
  FdeDecisionContextV2 limited_context;
  bindActionSearch(limited, &limited_context);
  std::vector<CandidateEvaluation> none;
  const auto refusal = FdeManager().decide(
      all_in, fixture.hypotheses, fixture.evidence,
      &none, {}, risk, &limited_context);
  EXPECT_EQ(refusal.status, FdeStatus::SearchIncomplete);
  EXPECT_FALSE(refusal.commit_allowed);
  EXPECT_FALSE(refusal.selected_action.has_value());
  EXPECT_FALSE(refusal.integrity_available);
}

TEST(P006ActionSearch,
     OperationEquivalentSemanticVariantsAreOrderIndependentEndToEnd) {
  ExclusionAction uwb = action(42, 0.75, -0.25);
  uwb.id = ExclusionActionId(1);
  uwb.action_model_id = "UWB_EXCLUSION";
  uwb.bridge_mode = BridgeMode::GenericKinematic;
  uwb.recoverability = HistoryRecoverability::Recoverable;
  uwb.model_error_validated = true;
  ExclusionAction imu = uwb;
  imu.id = ExclusionActionId(99);
  imu.action_model_id = "IMU_EXCLUSION";
  ASSERT_EQ(exactActionOperationIdentityV1(uwb),
            exactActionOperationIdentityV1(imu));
  ASSERT_NE(exactActionSemanticIdentityV1(uwb),
            exactActionSemanticIdentityV1(imu));

  DetectorResultV2 alarm;
  alarm.numerically_valid = true;
  alarm.passed = false;
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(1);
  hypothesis.affected_groups = uwb.groups_to_remove;
  hypothesis.prior_probability_bound = 1e-3;
  hypothesis.p_md_allocation = 1e-3;
  hypothesis.hmi_allocation = 1e-3;
  FaultModeEvidence evidence;
  evidence.hypothesis = hypothesis.id;
  evidence.plausible = true;
  evidence.unit_kind = FaultUnitKind::UwbRangeMeters;
  evidence.parameter_dimension = 1;
  evidence.profile_j = 1.0;
  evidence.profile_valid = true;
  RiskBudgetV2 risk;
  risk.p_hmi_total = 1.0;
  risk.nominal_axis_tail = 0.0;
  risk.p_nm = 0.0;
  risk.horizontal_alert_limit_m = 10.0;
  risk.vertical_alert_limit_m = 10.0;
  risk.calibration_id = "p006-semantic-order-proof";

  auto decide_order = [&](std::vector<ExclusionAction> raw) {
    const auto search = censusAndCapActionsV1(raw, raw.size());
    EXPECT_TRUE(search.census.exhaustive);
    EXPECT_EQ(search.actions.size(), 2u);
    EXPECT_EQ(search.actions[0].id.value(), 1u);
    EXPECT_EQ(search.actions[1].id.value(), 99u);
    std::vector<CandidateEvaluation> candidates;
    for (const auto& candidate_action : search.actions) {
      CandidateEvaluation candidate;
      candidate.action = candidate_action;
      candidate.valid = true;
      candidate.post_detector_passed = true;
      candidate.information_logdet = 1.0;
      candidate.hpl_m = 1.0;
      candidate.vpl_m = 1.0;
      candidate.pl_xyz_m = Eigen::Vector3d::Ones();
      candidates.push_back(std::move(candidate));
    }
    FdeRiskDecisionV1 risk_result;
    FdeDecisionContextV2 context;
    context.v1.risk_result = &risk_result;
    bindActionSearch(search, &context);
    const auto decision = FdeManager().decide(
        alarm, {hypothesis}, {evidence}, &candidates, {}, risk, &context);
    return std::make_tuple(decision, risk_result, candidates);
  };

  const auto forward = decide_order({uwb, imu});
  const auto reverse = decide_order({imu, uwb});
  const auto& forward_decision = std::get<0>(forward);
  const auto& reverse_decision = std::get<0>(reverse);
  EXPECT_FALSE(forward_decision.selected_action.has_value());
  EXPECT_FALSE(reverse_decision.selected_action.has_value());
  EXPECT_EQ(forward_decision.status, FdeStatus::RiskBudgetInvalid);
  EXPECT_EQ(reverse_decision.status, forward_decision.status);
  EXPECT_EQ(reverse_decision.commit_allowed,
            forward_decision.commit_allowed);
  EXPECT_EQ(reverse_decision.integrity_available,
            forward_decision.integrity_available);
  EXPECT_EQ(reverse_decision.reason, forward_decision.reason);
  EXPECT_EQ(reverse_decision.candidate_dispositions,
            forward_decision.candidate_dispositions);
  EXPECT_EQ(reverse_decision.selection_event_class_ids,
            forward_decision.selection_event_class_ids);
  EXPECT_EQ(reverse_decision.selection_risk_proof_id,
            forward_decision.selection_risk_proof_id);
  EXPECT_DOUBLE_EQ(std::get<1>(reverse).charged_total,
                   std::get<1>(forward).charged_total);

  auto packet_for = [](const FdeDecision& decision,
                       const FdeRiskDecisionV1& risk_result) {
    IntegrityOutput output;
    output.timestamp = TimestampNs(100);
    output.attempted_timestamp = TimestampNs(100);
    output.state.timestamp = TimestampNs(100);
    output.fde_status = toString(decision.status);
    output.selected_action_id = decision.selected_action
        ? decision.selected_action->id.value() : 0;
    output.selected_action_type = decision.selected_action
        ? decision.selected_action->action_model_id : "NONE";
    output.batch_committed = decision.commit_allowed;
    output.protection_level.risk_budget_valid =
        risk_result.complete_bound_closes && risk_result.all_terms_validated;
    output.protection_level.formal_eligible = risk_result.formal_eligible;
    output.protection_level.availability = decision.integrity_available
        ? Availability::Available : Availability::Unavailable;
    output.protection_level.pl_xyz_m = Eigen::Vector3d::Ones();
    output.protection_level.hpl_m = 1.0;
    output.protection_level.vpl_m = 1.0;
    FinalPacketTiming timing;
    timing.arrival_steady_ns = 100;
    timing.compute_done_steady_ns = 110;
    timing.packet_ready_steady_ns = 120;
    timing.publish_call_steady_ns = 125;
    timing.publish_return_steady_ns = 130;
    return finalizeOutputPacket(std::move(output), timing, 1000);
  };
  const auto forward_packet = packet_for(forward_decision, std::get<1>(forward));
  const auto reverse_packet = packet_for(reverse_decision, std::get<1>(reverse));
  EXPECT_EQ(finalOutputPacketDigest(forward_packet),
            finalOutputPacketDigest(reverse_packet));
  EXPECT_EQ(forward_packet.output().selected_action_id,
            reverse_packet.output().selected_action_id);
  EXPECT_EQ(forward_packet.output().selected_action_type,
            reverse_packet.output().selected_action_type);
  EXPECT_EQ(forward_packet.output().fde_status,
            reverse_packet.output().fde_status);
  EXPECT_EQ(forward_packet.output().protection_level.availability,
            reverse_packet.output().protection_level.availability);
}

TEST(P006ActionSearch,
     ProductionExceptionsPreserveCensusThroughPacketAndCsv) {
  for (const std::string stage : {
           "candidate_kernel", "post_detector", "protection_level"}) {
    SCOPED_TRACE(stage);
    IntegrityConfig config = productionPipelineConfig();
    IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
    NavigationState initial;
    initial.timestamp = TimestampNs(0);
    initial.position_world_m = {0.0, 0.0, 1.0};
    estimator.initialize(initial,
                         Eigen::Matrix<double, 15, 1>::Constant(0.1));
    RealtimeIntegrityPipeline pipeline(
        &estimator, IntegrityMonitor(config.risk,
            config.snapshot.rank_tolerance,
            config.snapshot.max_condition_number));
    for (int sample = 0; sample <= 2; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(100 + sample);
      imu.timestamp = TimestampNs(sample * 5000000LL);
      imu.specific_force_mps2 = {0.0, 0.0, config.imu.gravity_mps2};
      pipeline.ingestImu(imu);
    }
    auto seams = std::make_shared<PipelineTestDependencySeamsV1>();
    seams->after_action_generation_before_census =
        [](std::vector<ExclusionAction>* raw) {
          if (!raw || raw->empty()) {
            throw std::runtime_error(
                "production generator returned no raw action to duplicate");
          }
          // This is an exact occurrence duplicate at the real generator ->
          // census boundary, not a hand-built IntegrityOutput sidecar.
          raw->push_back(raw->front());
        };
    auto fail = [stage](const std::string& expected) {
      if (stage == expected) {
        throw std::runtime_error("injected production " + expected +
                                 " dependency failure");
      }
    };
    seams->before_candidate = [fail](ExclusionActionId) {
      fail("candidate_kernel");
    };
    seams->before_post_detector = [fail](ExclusionActionId) {
      fail("post_detector");
    };
    seams->before_protection_level = [fail](ExclusionActionId) {
      fail("protection_level");
    };
    pipeline.setTestDependencySeamsV1(seams);
    EXPECT_THROW(
        pipeline.processUwbBatch(
            productionPipelineBatch(TimestampNs(10000000))),
        std::runtime_error);
    const IntegrityOutput attempt = pipeline.lastAttemptOutput();
    EXPECT_EQ(attempt.diagnostics.status, "EXCEPTION");
    EXPECT_FALSE(attempt.batch_committed);
    EXPECT_FALSE(attempt.publication.protected_output);
    EXPECT_FALSE(attempt.protection_level.formal_eligible);
    EXPECT_EQ(attempt.diagnostics.generated_actions, 2u);
    const auto summary = std::find_if(
        attempt.reason_codes.begin(), attempt.reason_codes.end(),
        [](const std::string& reason) {
          return reason.rfind("ACTION_SEARCH_CERTIFICATE_V1:", 0) == 0;
        });
    ASSERT_NE(summary, attempt.reason_codes.end());
    EXPECT_NE(summary->find("validation_valid=1"), std::string::npos);
    EXPECT_NE(summary->find("trusted_snapshot="), std::string::npos);
    EXPECT_NE(summary->find("generator_identity="), std::string::npos);
    EXPECT_NE(summary->find("planned_evaluations="), std::string::npos);
    EXPECT_NE(summary->find("actual_candidate_evaluations="),
              std::string::npos);
    EXPECT_NE(summary->find("actual_post_evaluations="),
              std::string::npos);
    EXPECT_NE(summary->find("actual_pl_evaluations="),
              std::string::npos);
    std::vector<const CandidateAuditRecord*> occurrence_records;
    std::size_t duplicate_records = 0;
    for (const auto& record : attempt.candidate_audit) {
      if (record.action_type != "ACTION_SEARCH_OCCURRENCE_V1") continue;
      occurrence_records.push_back(&record);
      EXPECT_FALSE(record.physical_source_ids.empty());  // occurrence
      EXPECT_NE(record.model_error_record.find("operation_identity="),
                std::string::npos);
      EXPECT_NE(record.model_error_record.find("semantic_identity="),
                std::string::npos);
      EXPECT_FALSE(record.removal_data_source.empty());  // trusted snapshot
      EXPECT_TRUE(record.model_error_validated);
      EXPECT_FALSE(record.diagnostics.skip_reason.empty());
      if (record.model_error_record.find(
              "omission_reason=EXACT_SEMANTIC_DUPLICATE") !=
          std::string::npos) {
        ++duplicate_records;
        EXPECT_EQ(record.diagnostics.skip_reason, "OMITTED");
        EXPECT_FALSE(record.bridge_mode.empty());  // duplicate_of occurrence
        EXPECT_NE(record.reason.find("duplicate_of="), std::string::npos);
      }
    }
    EXPECT_EQ(occurrence_records.size(), attempt.diagnostics.generated_actions);
    EXPECT_EQ(duplicate_records, 1u);
    EXPECT_NE(summary->find("generated=2"), std::string::npos);
    EXPECT_NE(summary->find("planned_evaluations=1"), std::string::npos);
    EXPECT_NE(summary->find("omitted=1"), std::string::npos);
    if (stage == "candidate_kernel") {
      EXPECT_NE(summary->find("actual_candidate_evaluations=0"),
                std::string::npos);
    } else if (stage == "post_detector") {
      EXPECT_NE(summary->find("actual_candidate_evaluations=1"),
                std::string::npos);
      EXPECT_NE(summary->find("actual_post_evaluations=0"),
                std::string::npos);
    } else {
      EXPECT_NE(summary->find("actual_post_evaluations=1"),
                std::string::npos);
      EXPECT_NE(summary->find("actual_pl_evaluations=0"),
                std::string::npos);
    }

    FinalPacketTiming timing;
    timing.arrival_steady_ns = 100;
    timing.compute_done_steady_ns = 110;
    timing.packet_ready_steady_ns = 120;
    timing.publish_call_steady_ns = 125;
    timing.publish_return_steady_ns = 130;
    timing.attempt_kind = FinalAttemptKind::Exception;
    const auto packet = finalizeOutputPacket(attempt, timing, 1000);
    EXPECT_FALSE(packet.output().publication.protected_output);
    EXPECT_TRUE(packet.output().publication.unprotected_output);
    EXPECT_FALSE(packet.output().protection_level.formal_eligible);
    EXPECT_EQ(packet.output().protection_level.availability,
              Availability::Unavailable);
    ASSERT_EQ(packet.output().candidate_audit.size(),
              attempt.candidate_audit.size());
    EXPECT_NE(std::find(packet.output().reason_codes.begin(),
                        packet.output().reason_codes.end(), *summary),
              packet.output().reason_codes.end());
    EXPECT_EQ(packet.output().attempted_timestamp,
              attempt.attempted_timestamp);
    EXPECT_EQ(packet.output().diagnostics.input_attempt_id,
              attempt.diagnostics.input_attempt_id);
    for (std::size_t row = 0; row < attempt.candidate_audit.size(); ++row) {
      const auto& staged = attempt.candidate_audit[row];
      const auto& finalized = packet.output().candidate_audit[row];
      EXPECT_EQ(finalized.action_id, staged.action_id);
      EXPECT_EQ(finalized.action_type, staged.action_type);
      EXPECT_EQ(finalized.physical_source_ids, staged.physical_source_ids);
      EXPECT_EQ(finalized.removal_data_source, staged.removal_data_source);
      EXPECT_EQ(finalized.model_error_record, staged.model_error_record);
      EXPECT_EQ(finalized.bridge_mode, staged.bridge_mode);
      EXPECT_EQ(finalized.diagnostics.skip_reason,
                staged.diagnostics.skip_reason);
      EXPECT_EQ(finalized.reason, staged.reason);
    }

    const std::string directory =
        "/tmp/uwb_imu_pl_p006_exception_" + stage;
    boost::filesystem::remove_all(directory);
    {
      RunLogger logger(directory, false, false);
      logger.writeIntegrity(packet.output());
      logger.flush();
    }
    std::ifstream integrity(directory + "/integrity.csv");
    const std::string integrity_text{
        std::istreambuf_iterator<char>(integrity),
        std::istreambuf_iterator<char>()};
    EXPECT_NE(integrity_text.find("ACTION_SEARCH_CERTIFICATE_V1:generated="),
              std::string::npos);
    EXPECT_NE(integrity_text.find("generated=2"), std::string::npos);
    EXPECT_NE(integrity_text.find("planned_evaluations=1"),
              std::string::npos);
    EXPECT_NE(integrity_text.find("omitted=1"), std::string::npos);
    std::ifstream candidates_file(directory + "/candidates.csv");
    const std::string candidates_text{
        std::istreambuf_iterator<char>(candidates_file),
        std::istreambuf_iterator<char>()};
    EXPECT_NE(candidates_text.find("ACTION_SEARCH_OCCURRENCE_V1"),
              std::string::npos);
    for (const auto* occurrence : occurrence_records) {
      EXPECT_NE(candidates_text.find(occurrence->physical_source_ids),
                std::string::npos);
      EXPECT_NE(candidates_text.find(occurrence->removal_data_source),
                std::string::npos);
      EXPECT_NE(candidates_text.find(occurrence->model_error_record),
                std::string::npos);
      if (!occurrence->bridge_mode.empty()) {
        EXPECT_NE(candidates_text.find(occurrence->bridge_mode),
                  std::string::npos);
      }
    }
    std::size_t csv_occurrence_rows = 0;
    std::istringstream candidate_rows(candidates_text);
    for (std::string row; std::getline(candidate_rows, row);) {
      if (row.find("ACTION_SEARCH_OCCURRENCE_V1") != std::string::npos) {
        ++csv_occurrence_rows;
      }
    }
    EXPECT_EQ(csv_occurrence_rows, occurrence_records.size());
    boost::filesystem::remove_all(directory);
  }
}

TEST(P006ActionSearch, ReleaseRuntimeIgnoresLegacyP006EnvironmentVariable) {
#ifndef NDEBUG
  FAIL() << "environment-invariance regression must execute an NDEBUG build";
#endif
  auto run = []() {
    const IntegrityConfig config = productionPipelineConfig();
    IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
    NavigationState initial;
    initial.timestamp = TimestampNs(0);
    initial.position_world_m = {0.0, 0.0, 1.0};
    estimator.initialize(initial,
                         Eigen::Matrix<double, 15, 1>::Constant(0.1));
    RealtimeIntegrityPipeline pipeline(
        &estimator, IntegrityMonitor(config.risk,
            config.snapshot.rank_tolerance,
            config.snapshot.max_condition_number));
    for (int sample = 0; sample <= 2; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(100 + sample);
      imu.timestamp = TimestampNs(sample * 5000000LL);
      imu.specific_force_mps2 = {0.0, 0.0, config.imu.gravity_mps2};
      pipeline.ingestImu(imu);
    }
    return pipeline.processUwbBatch(
        productionPipelineBatch(TimestampNs(10000000)));
  };
  auto raw_census = []() {
    ExclusionAction keep;
    keep.id = ExclusionActionId(1);
    keep.action_model_id = "KEEP_ALL";
    return censusAndCapActionsV1({keep}, 1);
  };
  auto compare_output = [](const IntegrityOutput& expected,
                           const IntegrityOutput& actual) {
    auto same_double = [](double left, double right) {
      return (std::isnan(left) && std::isnan(right)) ||
          (std::isinf(left) && std::isinf(right) &&
           std::signbit(left) == std::signbit(right)) || left == right;
    };
    EXPECT_EQ(actual.timestamp, expected.timestamp);
    EXPECT_EQ(actual.attempted_timestamp, expected.attempted_timestamp);
    EXPECT_EQ(actual.fde_status, expected.fde_status);
    EXPECT_EQ(actual.reason_codes, expected.reason_codes);
    EXPECT_EQ(actual.selected_action_id, expected.selected_action_id);
    EXPECT_EQ(actual.selected_action_type, expected.selected_action_type);
    EXPECT_EQ(actual.batch_committed, expected.batch_committed);
    EXPECT_EQ(actual.stale_state, expected.stale_state);
    EXPECT_EQ(actual.diagnostics.generated_actions,
              expected.diagnostics.generated_actions);
    EXPECT_EQ(actual.protection_level.availability,
              expected.protection_level.availability);
    EXPECT_EQ(actual.protection_level.risk_budget_valid,
              expected.protection_level.risk_budget_valid);
    EXPECT_EQ(actual.protection_level.formal_eligible,
              expected.protection_level.formal_eligible);
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_TRUE(same_double(actual.protection_level.pl_xyz_m(axis),
                              expected.protection_level.pl_xyz_m(axis)));
    }
    EXPECT_TRUE(same_double(actual.protection_level.hpl_m,
                            expected.protection_level.hpl_m));
    EXPECT_TRUE(same_double(actual.protection_level.vpl_m,
                            expected.protection_level.vpl_m));
    EXPECT_EQ(actual.publication.protected_output,
              expected.publication.protected_output);
    EXPECT_EQ(actual.publication.unprotected_output,
              expected.publication.unprotected_output);
    EXPECT_TRUE(actual.state.position_world_m.isApprox(
        expected.state.position_world_m, 0.0));
    EXPECT_TRUE(actual.state.velocity_world_mps.isApprox(
        expected.state.velocity_world_mps, 0.0));
    EXPECT_TRUE(actual.state.accel_bias_mps2.isApprox(
        expected.state.accel_bias_mps2, 0.0));
    EXPECT_TRUE(actual.state.gyro_bias_radps.isApprox(
        expected.state.gyro_bias_radps, 0.0));
    EXPECT_TRUE(actual.state.q_world_body.coeffs().isApprox(
        expected.state.q_world_body.coeffs(), 0.0));
    ASSERT_EQ(actual.candidate_audit.size(), expected.candidate_audit.size());
    for (std::size_t i = 0; i < actual.candidate_audit.size(); ++i) {
      EXPECT_EQ(actual.candidate_audit[i].action_id,
                expected.candidate_audit[i].action_id);
      EXPECT_EQ(actual.candidate_audit[i].action_type,
                expected.candidate_audit[i].action_type);
      EXPECT_EQ(actual.candidate_audit[i].reason,
                expected.candidate_audit[i].reason);
      EXPECT_EQ(actual.candidate_audit[i].valid,
                expected.candidate_audit[i].valid);
    }
  };

  ASSERT_EQ(::unsetenv("UWB_IMU_PL_P006_THROW_STAGE"), 0);
  const auto baseline_raw = raw_census();
  const IntegrityOutput baseline = run();
  // This is the existing lawful production transaction path under the
  // unchanged P0-04 risk contract: the nominal state may commit as explicitly
  // unprotected best effort, but no synthetic FDE winner/risk qualification is
  // invented.  The 130-action oracle below remains a separate honest refusal.
  EXPECT_TRUE(baseline.batch_committed);
  EXPECT_GT(baseline.backend_updates, 0u);
  EXPECT_FALSE(baseline.protection_level.formal_eligible);
  for (const char* legacy_value : {
           "candidate_kernel", "post_detector", "protection_level",
           "arbitrary-nonempty-value"}) {
    SCOPED_TRACE(legacy_value);
    ASSERT_EQ(::setenv("UWB_IMU_PL_P006_THROW_STAGE", legacy_value, 1), 0);
    const auto under_env_raw = raw_census();
    const IntegrityOutput under_env = run();
    EXPECT_EQ(under_env_raw.generated_snapshot.snapshot_identity,
              baseline_raw.generated_snapshot.snapshot_identity);
    EXPECT_EQ(under_env_raw.census.generated_identities,
              baseline_raw.census.generated_identities);
    EXPECT_EQ(under_env_raw.census.evaluated_identities,
              baseline_raw.census.evaluated_identities);
    EXPECT_EQ(under_env_raw.census.generated, baseline_raw.census.generated);
    EXPECT_EQ(under_env_raw.census.evaluated, baseline_raw.census.evaluated);
    EXPECT_EQ(under_env_raw.census.omitted, baseline_raw.census.omitted);
    compare_output(baseline, under_env);

    FinalPacketTiming timing;
    timing.arrival_steady_ns = 100;
    timing.compute_done_steady_ns = 110;
    timing.packet_ready_steady_ns = 120;
    timing.publish_call_steady_ns = 125;
    timing.publish_return_steady_ns = 130;
    const auto baseline_packet = finalizeOutputPacket(baseline, timing, 1000);
    const auto under_env_packet = finalizeOutputPacket(under_env, timing, 1000);
    EXPECT_EQ(finalOutputPacketDigest(under_env_packet),
              finalOutputPacketDigest(baseline_packet));
    compare_output(baseline_packet.output(), under_env_packet.output());
  }
  ASSERT_EQ(::unsetenv("UWB_IMU_PL_P006_THROW_STAGE"), 0);
}

TEST(P006ActionSearch, FinalPacketAlwaysRefusesIncompleteSearch) {
  for (const auto kind : {FinalAttemptKind::Normal,
                          FinalAttemptKind::QueueOverflow,
                          FinalAttemptKind::Exception}) {
    IntegrityOutput output;
    output.attempted_timestamp = TimestampNs(100);
    output.state.timestamp = TimestampNs(100);
    output.fde_status = "SEARCH_INCOMPLETE";
    output.reason_codes = {"SEARCH_INCOMPLETE"};
    output.publication.protected_output = true;
    output.publication.unprotected_output = false;
    output.protection_level.formal_eligible = true;
    output.protection_level.risk_budget_valid = true;
    output.protection_level.availability = Availability::Available;
    FinalPacketTiming timing;
    timing.arrival_steady_ns = 100;
    timing.compute_done_steady_ns = 110;
    timing.packet_ready_steady_ns = 120;
    timing.publish_call_steady_ns = 130;
    timing.publish_return_steady_ns = kind == FinalAttemptKind::Normal
        ? 140 : 200;
    timing.attempt_kind = kind;
    const auto packet = finalizeOutputPacket(std::move(output), timing, 50);
    EXPECT_FALSE(packet.output().publication.protected_output);
    EXPECT_TRUE(packet.output().publication.unprotected_output);
    EXPECT_FALSE(packet.output().protection_level.formal_eligible);
    EXPECT_FALSE(packet.output().protection_level.risk_budget_valid);
    EXPECT_EQ(packet.output().protection_level.availability,
              Availability::Unavailable);
    EXPECT_NE(std::find(packet.output().reason_codes.begin(),
                        packet.output().reason_codes.end(),
                        "SEARCH_INCOMPLETE"),
              packet.output().reason_codes.end());
  }
}

TEST(P006ActionSearch, CsvRetainsSearchReasonAndEveryOmittedIdentity) {
  const std::string directory = "/tmp/uwb_imu_pl_p006_search_csv";
  boost::filesystem::remove_all(directory);
  IntegrityOutput output;
  output.timestamp = TimestampNs(100);
  output.attempted_timestamp = TimestampNs(100);
  output.state.timestamp = TimestampNs(100);
  output.fde_status = "SEARCH_INCOMPLETE";
  output.reason_codes = {"SEARCH_INCOMPLETE"};
  output.protection_level.reason =
      "SEARCH_INCOMPLETE: generated=130 evaluated=128 omitted=2";
  output.protection_level.availability = Availability::Unavailable;
  for (std::uint64_t id : {129u, 130u}) {
    CandidateAuditRecord omitted;
    omitted.action_id = id;
    omitted.action_type = "ACTION_CENSUS_V1";
    omitted.reason = "RESOURCE_CAP_UNPROVEN:ACTION_OPERATION_V1|" +
        std::to_string(id);
    omitted.diagnostics.skip_reason = omitted.reason;
    output.candidate_audit.push_back(std::move(omitted));
  }
  {
    RunLogger logger(directory, false, false);
    logger.writeIntegrity(output);
    logger.flush();
  }
  std::ifstream integrity(directory + "/integrity.csv");
  const std::string integrity_text{
      std::istreambuf_iterator<char>(integrity),
      std::istreambuf_iterator<char>()};
  EXPECT_NE(integrity_text.find("SEARCH_INCOMPLETE"), std::string::npos);
  std::ifstream candidates(directory + "/candidates.csv");
  const std::string candidate_text{
      std::istreambuf_iterator<char>(candidates),
      std::istreambuf_iterator<char>()};
  EXPECT_NE(candidate_text.find("RESOURCE_CAP_UNPROVEN:ACTION_OPERATION_V1|129"),
            std::string::npos);
  EXPECT_NE(candidate_text.find("RESOURCE_CAP_UNPROVEN:ACTION_OPERATION_V1|130"),
            std::string::npos);
  boost::filesystem::remove_all(directory);
}

}  // namespace
