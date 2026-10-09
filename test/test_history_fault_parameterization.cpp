// C1-b Part A tests: historical fault parameterization.
//
// Every reference quantity here is computed by an independent oracle written
// in this file (selection patterns recomputed from the raw batch material,
// whitening round-trips recomputed from the covariance, and the generator's
// window-side columns compared on shared material).  The subsystem under
// test is `history_fault_parameterization.{hpp,cpp}`.

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>

#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/history_fault_parameterization.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"

namespace {

using namespace uwb_imu_pl;

IntegrityConfig researchConfig() {
  return IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/config/realtime_uwb_imu_pl_research.yaml");
}

IntegrityConfig jointOrder2Config() {
  return IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/config/fde_joint_order2.yaml");
}

UwbBatch batch(const IntegrityConfig& config, std::int64_t time_ns) {
  UwbBatch value;
  value.id = BatchId(static_cast<std::uint64_t>(time_ns));
  value.timestamp = TimestampNs(time_ns);
  for (std::size_t i = 0; i < config.anchors.size(); ++i) {
    UwbMeasurement measurement;
    measurement.id = MeasurementId(i + 1);
    measurement.factor_id = FactorId(i + 1);
    measurement.anchor_id = config.anchors[i].id;
    measurement.timestamp = value.timestamp;
    measurement.anchor_position_m = config.anchors[i].position_world_m;
    measurement.range_m =
        (measurement.anchor_position_m - Eigen::Vector3d(0, 0, 1)).norm();
    measurement.sigma_m = 0.05;
    value.measurements.push_back(measurement);
  }
  return value;
}

// Runs `count` committed epochs (10 IMU samples per 50 ms epoch) and returns
// the prepared-but-uncommitted transaction of the final epoch.
EpochTransaction matureTransaction(IncrementalUwbImuEstimator* estimator,
                                   const IntegrityConfig& config,
                                   std::size_t count) {
  ImuMeasurement boundary;
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator->ingestImu(boundary);
  EpochTransaction last;
  for (std::size_t epoch = 1; epoch <= count; ++epoch) {
    for (int sample = 1; sample <= 10; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(epoch * 1000 + sample);
      imu.timestamp = TimestampNs(
          static_cast<std::int64_t>(((epoch - 1) * 10 + sample) * 5000000));
      imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
      estimator->ingestImu(imu);
    }
    auto input = batch(config, static_cast<std::int64_t>(epoch * 50000000));
    for (std::size_t row = 0; row < input.measurements.size(); ++row) {
      input.measurements[row].id = MeasurementId(epoch * 100 + row + 1);
      input.measurements[row].factor_id = FactorId(epoch * 100 + row + 1);
    }
    auto transaction = estimator->prepareEpoch(input);
    if (epoch == count) {
      last = transaction;
    } else {
      const auto plan = EpochCommitPlan::nominalPlan(transaction);
      estimator->commitEpoch(std::move(transaction), plan);
    }
  }
  return last;
}

const PendingFactorGroup* selectedGroup(const HistoricalEpochContext& epoch,
                                        FactorKind kind) {
  for (const auto& group : epoch.groups) {
    if (group.kind != kind) continue;
    if (std::find(epoch.selected_groups.begin(), epoch.selected_groups.end(),
                  group.id) != epoch.selected_groups.end()) {
      return &group;
    }
  }
  return nullptr;
}

const UwbMeasurement* measurementOf(const UwbBatch& batch, MeasurementId id) {
  for (const auto& measurement : batch.measurements) {
    if (measurement.id == id) return &measurement;
  }
  return nullptr;
}

const HistoricalEpochContext* recordOf(const EpochTransaction& tx,
                                       std::size_t epoch) {
  for (const auto& record : tx.recoverable_history) {
    if (record.proposed_epoch == epoch) return &record;
  }
  return nullptr;
}

}  // namespace

TEST(HistoryFaultParameterization,
     PlanCoversRecoverableHorizonWithoutOverclaiming) {
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  const EpochTransaction tx = matureTransaction(&estimator, config, 26);

  const auto plan = planHistoryFaultParameterization(tx, 10);
  const std::size_t window_first = tx.proposed_epoch - 10;
  EXPECT_EQ(plan.horizon.window_first_epoch, window_first);
  EXPECT_EQ(plan.horizon.first_epoch, tx.oldest_recoverable_epoch + 1);
  EXPECT_EQ(plan.horizon.source.rfind("derived:", 0), 0u);
  ASSERT_GT(plan.q_hist(), 0u);

  // Every column epoch lies inside the covered range.
  std::set<std::size_t> observed;
  std::set<std::uint64_t> kinds_seen;
  for (const auto& column : plan.columns) {
    EXPECT_GE(column.id.epoch, plan.horizon.first_epoch);
    EXPECT_LT(column.id.epoch, window_first);
    observed.insert(column.id.epoch);
    kinds_seen.insert(static_cast<std::uint64_t>(column.id.kind));
    EXPECT_GT(column.raw_map.rows(), 0);
    EXPECT_EQ(column.raw_map.cols(), 1);
    EXPECT_EQ(column.whitened_map.rows(), column.raw_map.rows());
  }
  EXPECT_EQ(kinds_seen.size(), 3u);  // constant, time-linear, IMU axes
  EXPECT_EQ(plan.material_gap_epoch_count, 0u);
  EXPECT_EQ(plan.omitted_epoch_count, plan.horizon.first_epoch);
  EXPECT_FALSE(plan.claimsFullCoverage());

  const std::string assumptions = plan.validityAssumptions();
  EXPECT_NE(assumptions.find("history_fault_coverage=[" +
                             std::to_string(plan.horizon.first_epoch)),
            std::string::npos);
  EXPECT_NE(assumptions.find("omitted_epochs=" +
                             std::to_string(plan.omitted_epoch_count)),
            std::string::npos);
  EXPECT_NE(assumptions.find("no detection or protection claim"),
            std::string::npos);
  EXPECT_EQ(plan.omittedRiskSource().rfind("history_fault_omitted:", 0), 0u);

  std::printf(
      "[HFP-TABLE] case=mature26 q_hist=%zu epochs_observed=%zu omitted=%zu "
      "gaps=%zu skipped=%zu first=%zu window_first=%zu\n",
      plan.q_hist(), observed.size(), plan.omitted_epoch_count,
      plan.material_gap_epoch_count, plan.skipped_columns,
      plan.horizon.first_epoch, window_first);

  // An explicit widened horizon must NOT claim the missing material.
  HistoryFaultParameterizationOptions configured;
  configured.configured_first_epoch = plan.horizon.first_epoch - 3;
  const auto widened = planHistoryFaultParameterization(tx, 10, configured);
  EXPECT_EQ(widened.horizon.source.rfind("configured:", 0), 0u);
  EXPECT_EQ(widened.horizon.first_epoch, plan.horizon.first_epoch - 3);
  EXPECT_GT(widened.material_gap_epoch_count, 0u);
  EXPECT_FALSE(widened.claimsFullCoverage());
  const std::size_t widened_observed = [&] {
    std::set<std::size_t> epochs;
    for (const auto& column : widened.columns) epochs.insert(column.id.epoch);
    return epochs.size();
  }();
  EXPECT_EQ(widened.material_gap_epoch_count,
            (window_first - widened.horizon.first_epoch) - widened_observed);
  EXPECT_EQ(widened.q_hist(), plan.q_hist());
}

TEST(HistoryFaultParameterization, InactiveProvidersCreateNoHistoricalColumns) {
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  const EpochTransaction tx = matureTransaction(&estimator, config, 26);

  HistoryFaultParameterizationOptions uwb_only;
  uwb_only.include_uwb_faults = true;
  uwb_only.include_imu_faults = false;
  uwb_only.scope_digest = "scope-uwb";
  const auto uwb = planHistoryFaultParameterization(tx, 10, uwb_only);
  ASSERT_GT(uwb.q_hist(), 0u);
  EXPECT_EQ(uwb.scope_digest, "scope-uwb");
  EXPECT_TRUE(std::none_of(uwb.columns.begin(), uwb.columns.end(),
      [](const HistoryFaultColumn& column) {
        return column.id.kind == HistoryFaultBasisKind::ImuAxisConstant;
      }));

  HistoryFaultParameterizationOptions imu_only;
  imu_only.include_uwb_faults = false;
  imu_only.include_imu_faults = true;
  imu_only.scope_digest = "scope-imu";
  const auto imu = planHistoryFaultParameterization(tx, 10, imu_only);
  ASSERT_GT(imu.q_hist(), 0u);
  EXPECT_EQ(imu.scope_digest, "scope-imu");
  EXPECT_TRUE(std::all_of(imu.columns.begin(), imu.columns.end(),
      [](const HistoryFaultColumn& column) {
        return column.id.kind == HistoryFaultBasisKind::ImuAxisConstant;
      }));

  HistoryFaultParameterizationOptions off;
  off.include_uwb_faults = false;
  off.include_imu_faults = false;
  off.scope_digest = "scope-off";
  const auto none = planHistoryFaultParameterization(tx, 10, off);
  EXPECT_TRUE(none.columns.empty());
  EXPECT_EQ(none.scope_digest, "scope-off");
}

TEST(HistoryFaultParameterization, ColumnsMatchIndependentSelectionOracle) {
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  const EpochTransaction tx = matureTransaction(&estimator, config, 26);
  const auto plan = planHistoryFaultParameterization(tx, 10);
  ASSERT_GT(plan.q_hist(), 0u);

  std::size_t checked_uwb = 0;
  std::size_t checked_imu = 0;
  double worst_uwb = 0.0;
  double worst_imu = 0.0;
  for (const auto& column : plan.columns) {
    const HistoricalEpochContext* record = recordOf(tx, column.id.epoch);
    ASSERT_NE(record, nullptr) << "column without material record";
    if (column.id.kind == HistoryFaultBasisKind::UwbAnchorConstant ||
        column.id.kind == HistoryFaultBasisKind::UwbAnchorTimeLinear) {
      const PendingFactorGroup* group =
          selectedGroup(*record, FactorKind::UwbBatch);
      ASSERT_NE(group, nullptr);
      const std::size_t rows = group->source_measurements.size();
      Eigen::VectorXd expected = Eigen::VectorXd::Zero(rows);
      for (std::size_t row = 0; row < rows; ++row) {
        const UwbMeasurement* measurement =
            measurementOf(record->uwb_batch, group->source_measurements[row]);
        if (!measurement ||
            measurement->anchor_id.value() != column.id.source) {
          continue;
        }
        expected(row) =
            column.id.kind == HistoryFaultBasisKind::UwbAnchorConstant
                ? 1.0
                : measurement->timestamp.seconds() - record->begin.seconds();
      }
      worst_uwb = std::max(worst_uwb, (column.raw_map - expected).norm());
      ++checked_uwb;
    } else {
      const PendingFactorGroup* group =
          selectedGroup(*record, FactorKind::CombinedImu);
      ASSERT_NE(group, nullptr);
      ASSERT_TRUE(record->preintegration);
      Eigen::LLT<Eigen::MatrixXd> llt(record->preintegration->preintMeasCov());
      ASSERT_EQ(llt.info(), Eigen::Success);
      // The physical factor uses GTSAM's upper information root. A
      // lower-Cholesky round-trip alone would miss a rotated fault column.
      const Eigen::VectorXd expected_whitened =
          gtsam::noiseModel::Gaussian::Covariance(
              record->preintegration->preintMeasCov())->R() * column.raw_map;
      const double scale = std::max(1.0, column.whitened_map.norm());
      worst_imu = std::max(
          worst_imu, (column.whitened_map - expected_whitened).norm() / scale);
      EXPECT_EQ(column.raw_map.rows(), 15);
      ++checked_imu;
    }
  }
  EXPECT_GT(checked_uwb, 0u);
  EXPECT_GT(checked_imu, 0u);
  EXPECT_EQ(worst_uwb, 0.0) << "raw UWB selections must be bitwise identical "
                               "to the independent oracle";
  EXPECT_LE(worst_imu, 1e-9);
  std::printf(
      "[HFP-TABLE] case=oracle checked_uwb=%zu worst_uwb=%.3e checked_imu=%zu "
      "worst_imu_rel=%.3e\n",
      checked_uwb, worst_uwb, checked_imu, worst_imu);
}

TEST(HistoryFaultParameterization,
     WindowAndHistoryColumnsAgreeOnSharedMaterial) {
  // A2: the same event produces identical columns through the window path
  // (HypothesisGenerator over the frozen window) and through the new
  // historical subsystem (same group material, historical-style context).
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  const EpochTransaction tx = matureTransaction(&estimator, config, 26);

  IntegrityWindowRequest request;
  request.epochs = 10;
  const auto window = estimator.buildIntegrityWindow(tx, request);
  ASSERT_TRUE(window.model_valid) << window.reason;
  const auto imu_block = estimator.buildPendingFactorBlock(tx, tx.imu_group.id);
  const auto bridge_block =
      estimator.buildPendingFactorBlock(tx, tx.generic_bridge_group.id);
  const auto imu_maps = ImuFaultSubspaceBuilder().build(tx, imu_block);
  const auto models =
      HypothesisGenerator().generate(window, tx, imu_maps, bridge_block);

  // P0-01/O03 census is an explicit three-stage contract.  The expected set
  // is enumerated from the frozen physical transaction/history scope, the
  // represented set from the mode registry, and the evaluated set from the
  // hypotheses.  Default research scope includes every single mode, so the
  // three identity sets must agree exactly.
  EXPECT_EQ(models.expected_mode_identities,
            models.represented_mode_identities);
  EXPECT_EQ(models.represented_mode_identities,
            models.evaluated_mode_identities);
  EXPECT_EQ(models.represented_mode_identities.size(), models.modes.size());

  const auto history_block = std::find_if(
      window.blocks.begin(), window.blocks.end(), [](const auto& block) {
        return block.whitening_model_id == "history_summary_sqrt_d1";
      });
  ASSERT_NE(history_block, window.blocks.end());
  bool persistent_history = false;
  bool ramp_history = false;
  bool imu_history = false;
  std::size_t history_mapped_modes = 0;
  std::size_t termwise_checked = 0;
  double worst_history_map = 0.0;
  std::map<std::size_t, double> physical_epoch_begin_s;
  for (const auto& record : tx.recoverable_history) {
    physical_epoch_begin_s[record.proposed_epoch] = record.begin.seconds();
  }
  for (const auto& mode : models.modes) {
    const auto mapped = mode.raw_group_maps.find(history_block->group_id);
    if (mapped == mode.raw_group_maps.end()) continue;
    ++history_mapped_modes;
    ASSERT_EQ(mapped->second.rows(), history_block->residual_raw.rows());
    EXPECT_TRUE(mapped->second.allFinite());
    EXPECT_GT(mapped->second.norm(), 0.0);

    // Independent, term-by-term physical oracle.  Construct the coefficient
    // matrix directly from the mode definition and frozen epoch times, then
    // apply it to the two initialized history carriers.  No production
    // persistent/ramp combination helper or generated mode map is reused.
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(
        static_cast<Eigen::Index>(window.history_summary.column_ids.size()),
        mode.parameter_dimension);
    const std::size_t effective_onset =
        (mode.onset_epoch == window.detector_first_epoch &&
         (mode.kind == FaultKind::AnchorBiasPersistentConstant ||
          mode.kind == FaultKind::AnchorBiasRamp))
            ? tx.oldest_recoverable_epoch
            : mode.onset_epoch;
    for (std::size_t column = 0;
         column < window.history_summary.column_ids.size(); ++column) {
      const auto& id = window.history_summary.column_ids[column];
      if (mode.kind == FaultKind::AnchorBiasEpochIndependent) {
        if (id.kind == HistoryFaultBasisKind::UwbAnchorConstant &&
            id.source == mode.anchor_id.value() &&
            id.epoch == mode.onset_epoch) {
          coefficients(static_cast<Eigen::Index>(column), 0) = 1.0;
        }
      } else if (mode.kind == FaultKind::AnchorBiasPersistentConstant) {
        if (id.kind == HistoryFaultBasisKind::UwbAnchorConstant &&
            id.source == mode.anchor_id.value() &&
            id.epoch >= effective_onset) {
          coefficients(static_cast<Eigen::Index>(column), 0) = 1.0;
        }
      } else if (mode.kind == FaultKind::AnchorBiasRamp) {
        if (id.source != mode.anchor_id.value() ||
            id.epoch < effective_onset) {
          continue;
        }
        if (id.kind == HistoryFaultBasisKind::UwbAnchorConstant) {
          const auto begin = physical_epoch_begin_s.find(id.epoch);
          ASSERT_NE(begin, physical_epoch_begin_s.end());
          coefficients(static_cast<Eigen::Index>(column), 0) = 1.0;
          coefficients(static_cast<Eigen::Index>(column), 1) =
              begin->second - mode.onset_time.seconds();
        } else if (id.kind == HistoryFaultBasisKind::UwbAnchorTimeLinear) {
          coefficients(static_cast<Eigen::Index>(column), 1) = 1.0;
        }
      } else if (mode.kind == FaultKind::AccelAxisIntervalConstant ||
                 mode.kind == FaultKind::GyroAxisIntervalConstant) {
        const std::uint64_t physical_axis =
            static_cast<std::uint64_t>(mode.axis +
                (mode.kind == FaultKind::GyroAxisIntervalConstant ? 3 : 0));
        // IMU mode onset is the interval's previous epoch; the history basis
        // labels the interval by its ending epoch.
        if (id.kind == HistoryFaultBasisKind::ImuAxisConstant &&
            id.source == physical_axis && id.epoch == mode.onset_epoch + 1) {
          coefficients(static_cast<Eigen::Index>(column), 0) = 1.0;
        }
      }
    }
    Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(
        window.history_summary.response.rows() +
            window.history_summary.detector_response.rows(),
        mode.parameter_dimension);
    expected.topRows(window.history_summary.response.rows()).noalias() =
        window.history_summary.response * coefficients;
    expected.bottomRows(
        window.history_summary.detector_response.rows()).noalias() =
        window.history_summary.detector_response * coefficients;
    const double scale = std::max(1.0, expected.norm());
    worst_history_map = std::max(
        worst_history_map, (mapped->second - expected).norm() / scale);
    EXPECT_LE((mapped->second - expected).norm() / scale, 1e-12)
        << "mode=" << mode.physical_source_id
        << " onset=" << mode.onset_epoch;
    ++termwise_checked;
    persistent_history = persistent_history ||
        mode.kind == FaultKind::AnchorBiasPersistentConstant;
    ramp_history = ramp_history || mode.kind == FaultKind::AnchorBiasRamp;
    imu_history = imu_history ||
        mode.kind == FaultKind::AccelAxisIntervalConstant ||
        mode.kind == FaultKind::GyroAxisIntervalConstant;
  }
  EXPECT_TRUE(persistent_history);
  EXPECT_TRUE(ramp_history);
  EXPECT_TRUE(imu_history);
  EXPECT_EQ(termwise_checked, history_mapped_modes);
  EXPECT_LE(worst_history_map, 1e-12);
  std::printf(
      "[P0-01-O03] expected=%zu represented=%zu evaluated=%zu "
      "history_mapped=%zu termwise_worst=%.3e\n",
      models.expected_mode_identities.size(),
      models.represented_mode_identities.size(),
      models.evaluated_mode_identities.size(), history_mapped_modes,
      worst_history_map);

  // Current-epoch material as a historical-style context.
  HistoricalEpochContext current;
  current.previous_epoch = tx.previous_epoch;
  current.proposed_epoch = tx.proposed_epoch;
  current.begin = tx.begin;
  current.end = tx.end;
  current.previous_state = tx.previous_state;
  current.current_state = tx.cv_predicted_state;
  ASSERT_TRUE(tx.frozen_values);
  for (const auto entry : {std::make_pair(tx.previous_epoch,&current.previous_state),
                           std::make_pair(tx.proposed_epoch,&current.current_state)}) {
    const auto p=tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',entry.first));
    const auto b=tx.frozen_values->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',entry.first));
    entry.second->q_world_body=Eigen::Quaterniond(p.rotation().matrix());
    entry.second->position_world_m=p.translation();
    entry.second->velocity_world_mps=tx.frozen_values->at<gtsam::Vector3>(gtsam::Symbol('v',entry.first));
    entry.second->accel_bias_mps2=b.accelerometer();
    entry.second->gyro_bias_radps=b.gyroscope();
  }
  current.raw_imu_slice = tx.raw_imu_slice;
  current.preintegration = tx.preintegration;
  current.uwb_batch = tx.uwb_batch;
  current.groups = tx.uwb_groups;
  current.groups.push_back(tx.imu_group);
  const PendingFactorGroup* nominal_uwb = nullptr;
  for (const auto& group : tx.uwb_groups) {
    if (!group.nominal) continue;
    current.selected_groups.push_back(group.id);
    nominal_uwb = &group;
  }
  current.selected_groups.push_back(tx.imu_group.id);
  ASSERT_NE(nominal_uwb, nullptr);

  const auto uwb_columns = buildHistoricalUwbColumnsForEpoch(current);
  const auto imu_columns = buildHistoricalImuColumnsForEpoch(current);
  const UwbMeasurement* first =
      measurementOf(tx.uwb_batch, nominal_uwb->source_measurements.front());
  ASSERT_NE(first, nullptr);
  const std::uint64_t anchor = first->anchor_id.value();

  // Window path: the generator's epoch-independent anchor mode for this event.
  const FaultModeBasis* anchor_mode = nullptr;
  for (const auto& mode : models.modes) {
    if (mode.kind != FaultKind::AnchorBiasEpochIndependent) continue;
    if (mode.anchor_id.value() != anchor) continue;
    if (std::find(mode.affected_groups.begin(), mode.affected_groups.end(),
                  nominal_uwb->id) == mode.affected_groups.end()) {
      continue;
    }
    anchor_mode = &mode;
  }
  ASSERT_NE(anchor_mode, nullptr) << "generator produced no anchor mode";
  const auto window_map = anchor_mode->raw_group_maps.find(nominal_uwb->id);
  ASSERT_NE(window_map, anchor_mode->raw_group_maps.end());

  const HistoryFaultColumn* history_column = nullptr;
  for (const auto& column : uwb_columns) {
    if (column.id.kind == HistoryFaultBasisKind::UwbAnchorConstant &&
        column.id.source == anchor && column.group == nominal_uwb->id) {
      history_column = &column;
    }
  }
  ASSERT_NE(history_column, nullptr);
  EXPECT_EQ(history_column->raw_map.rows(), window_map->second.rows());
  EXPECT_EQ((history_column->raw_map - window_map->second).norm(), 0.0)
      << "window (generator) and history (subsystem) columns must be bitwise "
         "identical on shared material";
  EXPECT_EQ(anchor_mode->parameter_dimension, 1u);
  EXPECT_EQ(anchor_mode->onset_epoch, current.proposed_epoch);

  // IMU side: identical analytic path for all six axes.
  double worst_imu = 0.0;
  std::size_t matched_axes = 0;
  for (int axis = 0; axis < 6; ++axis) {
    const FaultKind kind = axis < 3 ? FaultKind::AccelAxisIntervalConstant
                                    : FaultKind::GyroAxisIntervalConstant;
    const FaultModeBasis* imu_mode = nullptr;
    for (const auto& mode : models.modes) {
      if (mode.kind != kind || mode.axis != axis % 3) continue;
      if (mode.onset_epoch != tx.previous_epoch) continue;
      if (std::find(mode.affected_groups.begin(), mode.affected_groups.end(),
                    tx.imu_group.id) == mode.affected_groups.end()) {
        continue;
      }
      imu_mode = &mode;
    }
    ASSERT_NE(imu_mode, nullptr) << "axis " << axis;
    const auto imu_map = imu_mode->raw_group_maps.find(tx.imu_group.id);
    ASSERT_NE(imu_map, imu_mode->raw_group_maps.end());
    const HistoryFaultColumn* imu_column = nullptr;
    for (const auto& column : imu_columns) {
      if (column.id.kind == HistoryFaultBasisKind::ImuAxisConstant &&
          column.id.source == static_cast<std::uint64_t>(axis) &&
          column.group == tx.imu_group.id) {
        imu_column = &column;
      }
    }
    ASSERT_NE(imu_column, nullptr) << "axis " << axis;
    const double scale = std::max(1.0, imu_map->second.norm());
    worst_imu = std::max(
        worst_imu, (imu_column->raw_map - imu_map->second).norm() / scale);
    ++matched_axes;
  }
  EXPECT_EQ(matched_axes, 6u);
  EXPECT_LE(worst_imu, 1e-12)
      << "window and history IMU columns must share the analytic path";
  std::printf(
      "[HFP-TABLE] case=window_vs_history anchor=%llu uwb_exact=1 "
      "imu_axes=%zu worst_imu_rel=%.3e\n",
      static_cast<unsigned long long>(anchor), matched_axes, worst_imu);
}

TEST(HistoryFaultParameterization,
     JointOrder2PhysicalPairCensusReachesNumericalTerminalState) {
  const auto config = jointOrder2Config();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  EpochTransaction tx = matureTransaction(&estimator, config, 26);
  IntegrityWindowRequest request;
  request.epochs = config.integrity_window.epochs;
  const auto window = estimator.buildIntegrityWindow(tx, request);
  ASSERT_TRUE(window.model_valid) << window.reason;
  const auto imu_block = estimator.buildPendingFactorBlock(tx, tx.imu_group.id);
  const auto bridge_block =
      estimator.buildPendingFactorBlock(tx, tx.generic_bridge_group.id);
  const auto imu_maps = ImuFaultSubspaceBuilder().build(tx, imu_block);

  HypothesisGeneratorConfig generator;
  generator.include_uwb_faults = config.resolved_scope.requiresUwbFaults();
  generator.include_imu_faults = config.resolved_scope.requiresImuFaults();
  generator.single_faults_enabled = true;
  generator.double_faults_enabled = true;
  generator.max_model_cardinality = config.fault_models.max_cardinality;
  generator.max_exclusion_cardinality = config.fde.max_exclusion_cardinality;
  generator.max_candidate_count = config.fde.max_candidate_count;
  generator.include_uwb_accel_combinations =
      config.fault_models.combinations.uwb_plus_accel;
  generator.include_uwb_gyro_combinations =
      config.fault_models.combinations.uwb_plus_gyro;
  generator.include_epoch_independent_uwb =
      config.fault_models.uwb.epoch_single_anchor_bias;
  generator.include_persistent_uwb =
      config.fault_models.uwb.persistent_anchor_bias;
  generator.include_ramp_uwb = config.fault_models.uwb.ramp_bias;
  auto models = HypothesisGenerator(generator).generate(
      window, tx, imu_maps, bridge_block);

  std::size_t uwb_modes = 0, accel_modes = 0, gyro_modes = 0;
  for (const auto& mode : models.modes) {
    uwb_modes += mode.sensor == SensorType::Uwb;
    accel_modes += mode.sensor == SensorType::ImuAccelerometer;
    gyro_modes += mode.sensor == SensorType::ImuGyroscope;
  }
  EXPECT_EQ(uwb_modes, 504u);
  EXPECT_EQ(accel_modes, 60u);
  EXPECT_EQ(gyro_modes, 60u);
  EXPECT_EQ(models.double_uwb_accel_hypotheses, 30240u);
  EXPECT_EQ(models.double_uwb_gyro_hypotheses, 30240u);
  EXPECT_EQ(models.hypotheses.size(), 61104u);
  EXPECT_EQ(models.expected_pair_identities,
            models.represented_pair_identities);
  EXPECT_EQ(models.represented_pair_identities,
            models.evaluated_pair_identities);
  EXPECT_EQ(models.expected_pair_identities.size(), 60480u);
  EXPECT_EQ(models.pair_candidates_rejected_shared, 0u);

  // Traverse every registered hypothesis through the real numerical evidence
  // path.  Each pair must reach a terminal state: monitored, or explicitly
  // fail-closed with a nonempty numerical reason.  None may disappear in a
  // pre-evaluation local-rank filter.
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      window, models.modes, &models.hypotheses, 100.0);
  ASSERT_EQ(evidence.size(), models.hypotheses.size());
  std::size_t terminal_pairs = 0;
  std::size_t monitored_pairs = 0;
  std::size_t failed_closed_pairs = 0;
  for (std::size_t index = 0; index < models.hypotheses.size(); ++index) {
    const auto& hypothesis = models.hypotheses[index];
    if (hypothesis.modes.size() != 2) continue;
    ASSERT_EQ(evidence[index].hypothesis, hypothesis.id);
    if (hypothesis.monitored) {
      ++monitored_pairs;
    } else {
      EXPECT_FALSE(hypothesis.monitorability.reason.empty());
      ++failed_closed_pairs;
    }
    ++terminal_pairs;
  }
  EXPECT_EQ(terminal_pairs, models.expected_pair_identities.size());
  std::printf(
      "[P0-01-PAIR-CENSUS] uwb=%zu accel=%zu gyro=%zu "
      "expected=%zu represented=%zu evaluated=%zu terminal=%zu "
      "monitored=%zu fail_closed=%zu hypotheses=%zu\n",
      uwb_modes, accel_modes, gyro_modes,
      models.expected_pair_identities.size(),
      models.represented_pair_identities.size(),
      models.evaluated_pair_identities.size(), terminal_pairs,
      monitored_pairs, failed_closed_pairs, models.hypotheses.size());
  estimator.discardEpoch(
      std::move(tx), {FdeStatus::ModelInvalid, "test complete", false});
}

TEST(HistoryFaultParameterization, RampCombinationIsExactOverTheStepBasis) {
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  const EpochTransaction tx = matureTransaction(&estimator, config, 26);
  const auto plan = planHistoryFaultParameterization(tx, 10);

  // An anchor present in at least two historical epochs.
  std::map<std::uint64_t, std::set<std::size_t>> epochs_of_anchor;
  for (const auto& column : plan.columns) {
    if (column.id.kind == HistoryFaultBasisKind::UwbAnchorConstant) {
      epochs_of_anchor[column.id.source].insert(column.id.epoch);
    }
  }
  std::uint64_t anchor = 0;
  for (const auto& item : epochs_of_anchor) {
    if (item.second.size() >= 2) {
      anchor = item.first;
      break;
    }
  }
  ASSERT_NE(anchor, 0u);
  const std::size_t onset = *epochs_of_anchor.at(anchor).begin();
  const HistoricalEpochContext* onset_record = recordOf(tx, onset);
  ASSERT_NE(onset_record, nullptr);
  const double onset_time_s = onset_record->begin.seconds();

  std::vector<HistoryFaultColumnId> basis;
  std::map<std::size_t, double> epoch_begin_s;
  for (const auto& column : plan.columns) {
    basis.push_back(column.id);
    const HistoricalEpochContext* record = recordOf(tx, column.id.epoch);
    if (record) epoch_begin_s[column.id.epoch] = record->begin.seconds();
  }

  // Persistent: coefficient 1 on every constant column at/after onset.
  const auto persistent = persistentUwbCombination(anchor, onset, basis);
  ASSERT_GE(persistent.columns.size(), 2u);
  for (std::size_t i = 0; i < persistent.columns.size(); ++i) {
    EXPECT_EQ(persistent.columns[i].kind,
              HistoryFaultBasisKind::UwbAnchorConstant);
    EXPECT_GE(persistent.columns[i].epoch, onset);
    EXPECT_DOUBLE_EQ(persistent.coefficients(i), 1.0);
  }

  // Ramp: value t - t_onset on the anchor rows, exactly represented by
  // 1 * (t - t_epoch_begin) + (t_epoch_begin - t_onset) * 1.
  const auto ramp =
      rampUwbCombination(anchor, onset, onset_time_s, epoch_begin_s, basis);
  std::set<std::size_t> ramp_epochs;
  double worst = 0.0;
  for (std::size_t i = 0; i < ramp.columns.size(); ++i) {
    const auto& id = ramp.columns[i];
    if (id.source != anchor || id.epoch < onset) continue;
    ramp_epochs.insert(id.epoch);
    const HistoricalEpochContext* record = recordOf(tx, id.epoch);
    ASSERT_NE(record, nullptr);
    const PendingFactorGroup* group =
        selectedGroup(*record, FactorKind::UwbBatch);
    ASSERT_NE(group, nullptr);
    // Response of the ramp combination restricted to this column's rows.
    const double coefficient = ramp.coefficients(i);
    for (std::size_t row = 0; row < group->source_measurements.size(); ++row) {
      const UwbMeasurement* measurement =
          measurementOf(record->uwb_batch, group->source_measurements[row]);
      if (!measurement || measurement->anchor_id.value() != anchor) continue;
      const double t = measurement->timestamp.seconds();
      const double direct = t - onset_time_s;
      double response = 0.0;
      if (id.kind == HistoryFaultBasisKind::UwbAnchorConstant) {
        response = coefficient * 1.0;
      } else {
        response = coefficient * (t - record->begin.seconds());
      }
      worst = std::max(worst, std::abs(direct - response));
    }
  }
  // Add the constant-column contribution of the same epoch when evaluating
  // the full row identity: the direct scheme above compares per column; the
  // full combination identity is checked by summing both columns of an epoch.
  for (const std::size_t epoch : ramp_epochs) {
    const HistoricalEpochContext* record = recordOf(tx, epoch);
    const PendingFactorGroup* group =
        selectedGroup(*record, FactorKind::UwbBatch);
    for (std::size_t row = 0; row < group->source_measurements.size(); ++row) {
      const UwbMeasurement* measurement =
          measurementOf(record->uwb_batch, group->source_measurements[row]);
      if (!measurement || measurement->anchor_id.value() != anchor) continue;
      const double t = measurement->timestamp.seconds();
      double total = 0.0;
      for (std::size_t i = 0; i < ramp.columns.size(); ++i) {
        if (ramp.columns[i].epoch != epoch) continue;
        if (ramp.columns[i].kind ==
            HistoryFaultBasisKind::UwbAnchorTimeLinear) {
          total += ramp.coefficients(i) * (t - record->begin.seconds());
        } else {
          total += ramp.coefficients(i) * 1.0;
        }
      }
      EXPECT_LE(std::abs((t - onset_time_s) - total), 1e-12);
    }
  }
  ASSERT_GE(ramp_epochs.size(), 2u);
  std::printf(
      "[HFP-TABLE] case=ramp anchor=%llu epochs=%zu per_column_worst=%.3e\n",
      static_cast<unsigned long long>(anchor), ramp_epochs.size(), worst);
}

TEST(HistoryFaultParameterization, InjectionKeepsFaultKeysInSeparator) {
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  const EpochTransaction tx = matureTransaction(&estimator, config, 26);
  const auto plan = planHistoryFaultParameterization(tx, 10);
  ASSERT_GT(plan.q_hist(), 0u);

  // One historical epoch's UWB columns only (one group -> one rebuilt factor).
  std::size_t chosen_epoch = 0;
  for (const auto& column : plan.columns) {
    if (column.id.kind == HistoryFaultBasisKind::UwbAnchorConstant) {
      chosen_epoch = std::max(chosen_epoch, column.id.epoch);
    }
  }
  ASSERT_GT(chosen_epoch, 0u);
  const HistoricalEpochContext* record = recordOf(tx, chosen_epoch);
  ASSERT_NE(record, nullptr);
  std::vector<HistoryFaultColumn> columns;
  for (const auto& column : plan.columns) {
    if (column.id.epoch == chosen_epoch &&
        (column.id.kind == HistoryFaultBasisKind::UwbAnchorConstant ||
         column.id.kind == HistoryFaultBasisKind::UwbAnchorTimeLinear)) {
      columns.push_back(column);
    }
  }
  ASSERT_FALSE(columns.empty());

  const auto injection =
      buildHistoricalFaultInjection(*record, *tx.frozen_values, columns);
  ASSERT_TRUE(injection.valid) << injection.reason;
  EXPECT_EQ(injection.fault_keys.size(), columns.size());
  EXPECT_FALSE(injection.state_keys.empty());
  ASSERT_EQ(injection.graph.size(), 1u);

  // Fault keys survive eliminating every state key (they are in the
  // separator); state keys do not.
  gtsam::KeyVector state_keys = injection.state_keys;
  const auto reduced =
      injection.graph
          .eliminatePartialMultifrontal(state_keys, gtsam::EliminateQR)
          .second;
  std::set<gtsam::Key> remaining;
  for (const auto& factor : *reduced) {
    for (const gtsam::Key key : factor->keys()) remaining.insert(key);
  }
  for (const gtsam::Key key : injection.fault_keys) {
    EXPECT_GT(remaining.count(key), 0u);
  }
  for (const gtsam::Key key : injection.state_keys) {
    EXPECT_EQ(remaining.count(key), 0u);
  }

  // The appended columns and the residual are preserved: second difference of
  // the augmented error recovers 2 * ||a_i||^2 for every fault column.
  const auto factor = boost::dynamic_pointer_cast<gtsam::JacobianFactor>(
      (*injection.graph.begin()));
  ASSERT_NE(factor, nullptr);
  const Eigen::MatrixXd A = factor->getA();
  const Eigen::VectorXd b = factor->getb();
  // Column layout: the state keys contribute their tangent blocks (x/b: 6,
  // v: 3) and the fault columns follow.  Re-baselined with the injection
  // width fix (2026-09-21): the fault block starts at the summed tangent
  // dimension of the state keys, not at the key count -- a per-key single
  // column would truncate a 6-dimensional navigation key.
  int state_width = 0;
  for (const gtsam::Key key : injection.state_keys) {
    state_width += gtsam::Symbol(key).chr() == 'v' ? 3 : 6;
  }
  ASSERT_EQ(A.cols(), state_width + static_cast<int>(columns.size()));
  double worst_norm = 0.0;
  for (std::size_t i = 0; i < columns.size(); ++i) {
    const Eigen::VectorXd column = A.col(state_width + static_cast<int>(i));
    EXPECT_EQ((column - columns[i].whitened_map).norm(), 0.0)
        << "the injected column must be the whitened map exactly";
    const double plus = (column - b).squaredNorm();
    const double minus = (column + b).squaredNorm();
    const double second_difference = plus + minus - 2.0 * b.squaredNorm();
    const double expected = 2.0 * columns[i].whitened_map.squaredNorm();
    worst_norm = std::max(worst_norm, std::abs(second_difference - expected) /
                                          std::max(1.0, expected));
  }
  EXPECT_LE(worst_norm, 1e-12);
  std::printf(
      "[HFP-TABLE] case=injection epoch=%zu columns=%zu state_keys=%zu "
      "state_width=%d fault_keys=%zu rows=%zu worst_norm_rel=%.3e\n",
      chosen_epoch, columns.size(), injection.state_keys.size(), state_width,
      injection.fault_keys.size(), injection.rows, worst_norm);
}

TEST(HistoryFaultParameterization, CapacityRefusesWithoutTruncation) {
  HistoryFaultCapacityLimits limits;
  limits.max_fault_columns = 4;
  limits.capacity_action = "REFUSE";
  const auto fits = evaluateHistoryFaultCapacity(4, limits);
  EXPECT_TRUE(fits.fits);
  EXPECT_FALSE(fits.unusable);
  EXPECT_TRUE(fits.reason.empty());

  const auto refused = evaluateHistoryFaultCapacity(5, limits);
  EXPECT_FALSE(refused.fits);
  EXPECT_TRUE(refused.unusable);
  EXPECT_EQ(refused.reason, "HISTORY_CAPACITY_EXCEEDED");
  EXPECT_EQ(refused.action, "REFUSE");

  HistoryFaultCapacityLimits reset = limits;
  reset.capacity_action = "RESET";
  const auto reset_decision = evaluateHistoryFaultCapacity(9, reset);
  EXPECT_FALSE(reset_decision.fits);
  EXPECT_TRUE(reset_decision.unusable);
  EXPECT_EQ(reset_decision.reason, "HISTORY_CAPACITY_EXCEEDED");
  EXPECT_EQ(reset_decision.action, "RESET");

  HistoryFaultCapacityLimits stop = limits;
  stop.capacity_action = "STOP_PROTECTED";
  const auto stop_decision = evaluateHistoryFaultCapacity(9, stop);
  EXPECT_FALSE(stop_decision.fits);
  EXPECT_TRUE(stop_decision.unusable);

  HistoryFaultCapacityLimits unknown = limits;
  unknown.capacity_action = "DROP_OLDEST";
  const auto unknown_decision = evaluateHistoryFaultCapacity(1, unknown);
  EXPECT_FALSE(unknown_decision.fits);
  EXPECT_NE(unknown_decision.reason.find("unknown"), std::string::npos);

  // The zero-capacity default refuses any non-empty plan; nothing is ever
  // truncated (there is no truncation API).
  HistoryFaultCapacityLimits defaults;
  const auto empty = evaluateHistoryFaultCapacity(0, defaults);
  EXPECT_TRUE(empty.fits);
  const auto nonempty = evaluateHistoryFaultCapacity(1, defaults);
  EXPECT_TRUE(nonempty.unusable);
  EXPECT_EQ(nonempty.reason, "HISTORY_CAPACITY_EXCEEDED");
}

namespace {

// D round MOD-03 oracle: replicates ImuFaultSubspaceBuilder's private
// reintegration loop, but with the additive sample perturbation active only
// from `first_interval` on (interval i covers raw samples i-1 -> i, matching
// the production code).
gtsam::PreintegratedCombinedMeasurements reintegrateIntervalOnset(
    const EpochTransaction& tx, int axis, double perturbation,
    std::size_t first_interval, int sample_support = 0) {
  gtsam::PreintegratedCombinedMeasurements pim(*tx.preintegration);
  pim.resetIntegration();
  for (std::size_t i = 1; i < tx.raw_imu_slice.size(); ++i) {
    const auto& left = tx.raw_imu_slice[i - 1];
    const auto& right = tx.raw_imu_slice[i];
    const double dt = right.timestamp.seconds() - left.timestamp.seconds();
    Eigen::Vector3d acc =
        0.5 * (left.specific_force_mps2 + right.specific_force_mps2);
    Eigen::Vector3d gyro =
        0.5 * (left.angular_velocity_radps + right.angular_velocity_radps);
    if (i - 1 >= first_interval) {
      const double weight = sample_support == 1 ? (i == 1 ? .5 : 1.) :
                            sample_support == 2 ? (i == 1 ? .5 : 0.) : 1.;
      if (axis < 3) acc(axis) += weight * perturbation;
      else gyro(axis - 3) += weight * perturbation;
    }
    pim.integrateMeasurement(acc, gyro, dt);
  }
  return pim;
}

gtsam::Vector intervalOnsetFactorError(
    const EpochTransaction& tx,
    const gtsam::PreintegratedCombinedMeasurements& pim) {
  gtsam::CombinedImuFactor factor(1, 2, 3, 4, 5, 6, pim);
  const auto pose = [](const NavigationState& state) {
    return gtsam::Pose3(
        gtsam::Rot3(state.q_world_body.normalized().toRotationMatrix()),
        state.position_world_m);
  };
  const auto bias = [](const NavigationState& state) {
    return gtsam::imuBias::ConstantBias(state.accel_bias_mps2,
                                        state.gyro_bias_radps);
  };
  if (tx.frozen_values) {
    const auto& v=*tx.frozen_values;
    return factor.evaluateError(
        v.at<gtsam::Pose3>(gtsam::Symbol('x',tx.previous_epoch)),
        v.at<gtsam::Vector3>(gtsam::Symbol('v',tx.previous_epoch)),
        v.at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch)),
        v.at<gtsam::Vector3>(gtsam::Symbol('v',tx.proposed_epoch)),
        v.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',tx.previous_epoch)),
        v.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',tx.proposed_epoch)));
  }
  return factor.evaluateError(pose(tx.previous_state),
                              tx.previous_state.velocity_world_mps,
                              pose(tx.nominal_predicted_state),
                              tx.nominal_predicted_state.velocity_world_mps,
                              bias(tx.previous_state),
                              bias(tx.nominal_predicted_state));
}

Eigen::MatrixXd intervalOnsetOracleMap(const EpochTransaction& tx,
                                       const LinearizedFactorBlock& block,
                                       int axis, double epsilon,
                                       std::size_t first_interval, int sample_support = 0) {
  const auto plus =
      reintegrateIntervalOnset(tx, axis, epsilon, first_interval,sample_support);
  const auto minus =
      reintegrateIntervalOnset(tx, axis, -epsilon, first_interval,sample_support);
  const Eigen::VectorXd derivative =
      (intervalOnsetFactorError(tx, plus) -
       intervalOnsetFactorError(tx, minus)) / (2.0 * epsilon);
  return -block.whitener * derivative;
}

}  // namespace

// ---------------------------------------------------------------------------
// D round MOD-01: UWB template support under dropouts and non-uniform sample
// times.  The declared UWB shapes are per-epoch occurrences; a dropped anchor
// in one history epoch must remove exactly that (epoch, source) occurrence
// from the parameterization (no fabricated template, no material-gap
// overclaim), and the time-linear companion column must follow the *actual*
// measurement timestamps when an epoch is not on the uniform grid.
// ---------------------------------------------------------------------------
TEST(HistoryFaultParameterization,
     Mod01UwbDropoutAndNonUniformTimestampsKeepTheDeclaredSupport) {
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary;
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  const std::size_t dropout_epoch = 8;
  const std::size_t skew_epoch = 9;
  const std::size_t dropout_anchor = 0;
  constexpr double kSkewSeconds = 0.003;  // inside imu.max_time_skew_s = 0.01
  EpochTransaction last;
  for (std::size_t epoch = 1; epoch <= 26; ++epoch) {
    for (int sample = 1; sample <= 10; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(epoch * 1000 + sample);
      imu.timestamp = TimestampNs(
          static_cast<std::int64_t>(((epoch - 1) * 10 + sample) * 5000000));
      imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
      estimator.ingestImu(imu);
    }
    auto input = batch(config, static_cast<std::int64_t>(epoch * 50000000));
    for (std::size_t row = 0; row < input.measurements.size(); ++row) {
      input.measurements[row].id = MeasurementId(epoch * 100 + row + 1);
      input.measurements[row].factor_id = FactorId(epoch * 100 + row + 1);
      if (epoch == skew_epoch) {
        input.measurements[row].timestamp = TimestampNs(
            static_cast<std::int64_t>(epoch * 50000000 +
                                      kSkewSeconds * 1e9));
      }
    }
    if (epoch == dropout_epoch) {
      input.measurements.erase(input.measurements.begin() +
                               static_cast<std::ptrdiff_t>(dropout_anchor));
    }
    auto transaction = estimator.prepareEpoch(input);
    if (epoch == 26) {
      last = transaction;
    } else {
      const auto plan = EpochCommitPlan::nominalPlan(transaction);
      estimator.commitEpoch(std::move(transaction), plan);
    }
  }
  const auto plan = planHistoryFaultParameterization(last, 10);
  const std::size_t window_first = last.proposed_epoch - 10;
  // UWB occurrences only: IMU columns use axis indexes as their source, which
  // would collide with anchor ids in a combined pair set.
  std::set<std::pair<std::size_t, std::uint64_t>> pairs;
  for (const auto& column : plan.columns) {
    if (column.id.kind == HistoryFaultBasisKind::UwbAnchorConstant ||
        column.id.kind == HistoryFaultBasisKind::UwbAnchorTimeLinear) {
      pairs.insert({column.id.epoch, column.id.source});
    }
  }
  // Dropout: exactly that occurrence disappears; the epoch keeps material.
  EXPECT_EQ(pairs.count({dropout_epoch,
                         config.anchors[dropout_anchor].id.value()}),
            0u);
  EXPECT_GT(pairs.count({dropout_epoch,
                         config.anchors[1].id.value()}),
            0u);
  EXPECT_EQ(plan.material_gap_epoch_count, 0u);
  // The declared support still reaches the window boundary (no tail cut).
  EXPECT_GT(pairs.count({window_first - 1, config.anchors[1].id.value()}), 0u);

  // Non-uniform times: recompute the time-linear oracle from the actual
  // timestamps of the shifted epoch.  An implementation that reused the
  // uniform grid (index * dt) would mismatch here.
  std::size_t checked = 0;
  double worst = 0.0;
  for (const auto& column : plan.columns) {
    if (column.id.epoch != skew_epoch ||
        column.id.kind != HistoryFaultBasisKind::UwbAnchorTimeLinear) {
      continue;
    }
    const HistoricalEpochContext* record = recordOf(last, column.id.epoch);
    ASSERT_NE(record, nullptr);
    const PendingFactorGroup* group =
        selectedGroup(*record, FactorKind::UwbBatch);
    ASSERT_NE(group, nullptr);
    Eigen::VectorXd expected = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(group->source_measurements.size()));
    for (std::size_t row = 0; row < group->source_measurements.size(); ++row) {
      const UwbMeasurement* measurement =
          measurementOf(record->uwb_batch, group->source_measurements[row]);
      if (!measurement || measurement->anchor_id.value() != column.id.source) {
        continue;
      }
      expected(static_cast<Eigen::Index>(row)) =
          measurement->timestamp.seconds() - record->begin.seconds();
    }
    worst = std::max(worst, (column.raw_map - expected).norm());
    ++checked;
  }
  EXPECT_GT(checked, 0u);
  EXPECT_EQ(worst, 0.0) << "time-linear template must follow actual sample "
                           "times, not an assumed uniform grid";
}

// ---------------------------------------------------------------------------
// D round MOD-03: an interval-interior onset must not reuse the whole-interval
// bias Jacobian.  The production event declarations are interval-constant per
// epoch, so the positive control is that the full-interval finite-difference
// oracle reproduces the analytic template; when the same perturbation covers
// only the second half of the interval the map changes by O(50 %), i.e. a
// non-whole-interval template would be plainly wrong if it kept the complete
// bias Jacobian.
// ---------------------------------------------------------------------------
TEST(HistoryFaultParameterization,
     Mod03IntervalInteriorOnsetCannotReuseTheWholeIntervalTemplate) {
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  const EpochTransaction tx = matureTransaction(&estimator, config, 1);
  ASSERT_TRUE(tx.preintegration);
  ASSERT_GE(tx.raw_imu_slice.size(), 3u);
  const LinearizedFactorBlock block =
      estimator.buildPendingFactorBlock(tx, tx.imu_group.id);
  ASSERT_EQ(block.kind, FactorKind::CombinedImu);
  const auto maps = ImuFaultSubspaceBuilder().build(tx, block);
  ASSERT_TRUE(maps.analytic_input_valid);
  ASSERT_TRUE(maps.analytic_computation_valid);
  const double epsilon = 1e-6;
  const Eigen::MatrixXd full = intervalOnsetOracleMap(tx, block, 0, epsilon, 0);
  const Eigen::MatrixXd half =
      intervalOnsetOracleMap(tx, block, 0, epsilon, tx.raw_imu_slice.size() / 2);
  const double reference = std::max(1e-12, maps.accel_axis[0].norm());
  // Positive control: full-interval support == analytic template.
  EXPECT_LE((full - maps.accel_axis[0]).norm() / reference, 1e-4);
  // Interior onset: the whole-interval template must not be reused.
  const double mismatch =
      (half - maps.accel_axis[0]).norm() / reference;
  EXPECT_GT(mismatch, 0.2)
      << "a half-interval onset cannot be represented by the full-interval "
         "bias Jacobian";
  const double ratio = half.norm() / std::max(1e-12, full.norm());
  EXPECT_GT(ratio, 0.2);
  EXPECT_LT(ratio, 0.8);
}

TEST(FdeOperabilityImuSupport, NewSamplesOnlyIsNotTheWholeIntervalFaultTemplate) {
  const auto config=researchConfig();
  IncrementalUwbImuEstimator estimator(config,Eigen::Vector3d::Zero());
  NavigationState initial;initial.position_world_m={0,0,1};
  estimator.initialize(initial,config.realtime.prior_sigmas);
  const auto tx=matureTransaction(&estimator,config,1);
  ASSERT_EQ(tx.raw_imu_slice.size(),11u);
  const auto block=estimator.buildPendingFactorBlock(tx,tx.imu_group.id);
  const auto whole=intervalOnsetOracleMap(tx,block,0,1e-5,0);
  // Existing stream corrupts the ten new samples, leaving the shared old
  // boundary sample healthy. The next epoch then inherits a bad left sample.
  const auto new_only=intervalOnsetOracleMap(tx,block,0,1e-5,0,1);
  const auto boundary_only=intervalOnsetOracleMap(tx,block,0,1e-5,0,2);
  const Eigen::VectorXd a=block.whitener.partialPivLu().solve(whole);
  const Eigen::VectorXd b=block.whitener.partialPivLu().solve(new_only);
  const Eigen::VectorXd c=block.whitener.partialPivLu().solve(boundary_only);
  EXPECT_NEAR(std::abs(a(3)/a(6)),.025,1e-8);
  EXPECT_NEAR(std::abs(b(3)/b(6)),.02381578947368421,1e-8);
  EXPECT_NEAR(std::abs(c(6)/a(6)),.05,1e-8);
  const double scale=whole.col(0).dot(new_only.col(0))/whole.squaredNorm();
  const double off_template=(new_only-scale*whole).norm()/whole.norm();
  EXPECT_GT(off_template,1e-6);
  std::printf("[FDE-IMU-SUPPORT] raw_full_dp_over_dv=%.17g raw_new_dp_over_dv=%.17g next_boundary_dv_fraction=%.17g best_scalar_relative_residual=%.17g\n",
      std::abs(a(3)/a(6)),std::abs(b(3)/b(6)),std::abs(c(6)/a(6)),off_template);
}
