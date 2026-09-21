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
#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"

namespace {

using namespace uwb_imu_pl;

IntegrityConfig researchConfig() {
  return IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/config/realtime_uwb_imu_pl_research.yaml");
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
      // raw = L * whitened  <=>  whitened = L^-1 * raw
      const Eigen::VectorXd expected_whitened =
          llt.matrixL().solve(column.raw_map);
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

  // Current-epoch material as a historical-style context.
  HistoricalEpochContext current;
  current.previous_epoch = tx.previous_epoch;
  current.proposed_epoch = tx.proposed_epoch;
  current.begin = tx.begin;
  current.end = tx.end;
  current.previous_state = tx.previous_state;
  current.current_state = tx.nominal_predicted_state;
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
