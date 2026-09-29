// C1-b/C1-c pipeline tests: the fault-preserving history summary wired into
// `buildIntegrityWindow` (design freeze history-summary-design.md §3/§7.2/
// §7.6; readiness list history-fault-parameterization.md §6 items 1-6).
//
// The tests drive the real estimator through epochs (no synthetic windows) and
// check, on the produced frozen windows:
//   * the condensed boundary IS the square-root summary (route (i)),
//   * pooled semantics: statistic = residual content + kappa_b, dof counts the
//     detection-only rows exactly once,
//   * A3 horizon honesty (no coverage claim beyond recoverable material),
//   * history fault response persists after the fault epoch left the window
//     (HIS-02: response + detection content carried by the summary),
//   * the summary version digest binds the window fingerprint / cache identity
//     and a changed digest invalidates the frozen numerics (HIS-04),
//   * capacity REFUSE marks the window explicitly unusable (HIS-06),
//   * row attribution: explicit XOR boundary exactly once, no double counting
//     (HIS-03),
//   * ordering: material deleted before the summary update fails explicitly.

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <tuple>

#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/integrity/history_fault_parameterization.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"

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

UwbBatch correlatedBiasedBatch(const IntegrityConfig& config,
                               std::size_t epoch) {
  UwbBatch value =
      batch(config, static_cast<std::int64_t>(epoch) * 50000000);
  const Eigen::Index rows =
      static_cast<Eigen::Index>(value.measurements.size());
  const double variance = 0.05 * 0.05;
  value.covariance_m2 = Eigen::MatrixXd::Constant(rows, rows,
                                                  0.12 * variance);
  value.covariance_m2.diagonal().setConstant(variance);
  value.covariance_model_id = "p0_01_correlated_raw_covariance";
  for (std::size_t row = 0; row < value.measurements.size(); ++row) {
    value.measurements[row].id = MeasurementId(epoch * 100 + row + 1);
    value.measurements[row].factor_id = FactorId(epoch * 100 + row + 1);
    value.measurements[row].range_m +=
        0.015 * std::sin(0.31 * static_cast<double>(epoch) +
                         0.17 * static_cast<double>(row + 1));
  }
  return value;
}

// Drives the estimator with clean IMU + clean UWB up to `epochs` and keeps the
// window of the final epoch (plus the previous one for comparison).
struct DrivenRun {
  std::vector<LinearizedIntegrityWindow> windows;
};

void appendImu(IncrementalUwbImuEstimator* estimator, double gravity,
               std::size_t epoch, int samples) {
  for (int sample = 1; sample <= samples; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(epoch * 1000 + static_cast<std::size_t>(sample));
    imu.timestamp = TimestampNs(
        static_cast<std::int64_t>(((epoch - 1) * 10 + sample) * 5000000));
    imu.specific_force_mps2 = {0, 0, gravity};
    estimator->ingestImu(imu);
  }
}

std::vector<LinearizedIntegrityWindow> driveEstimator(
    const IntegrityConfig& config, std::size_t epochs,
    std::map<std::size_t, std::size_t>* window_epochs = nullptr,
    std::map<std::size_t, std::size_t>* unique_boundary_rows = nullptr,
    std::map<std::size_t, std::uint64_t>* marginalization_counts = nullptr,
    std::map<std::size_t, std::size_t>* oldest_retained_epochs = nullptr,
    HistoryRootCacheAudit* root_cache_audit = nullptr) {
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  estimator.enableHistoryRootOracleForTesting(root_cache_audit != nullptr);
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary;
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  std::vector<LinearizedIntegrityWindow> windows;
  for (std::size_t epoch = 1; epoch <= epochs; ++epoch) {
    appendImu(&estimator, config.imu.gravity_mps2, epoch, 10);
    auto input = batch(config, static_cast<std::int64_t>(epoch) * 50000000);
    for (std::size_t row = 0; row < input.measurements.size(); ++row) {
      input.measurements[row].id = MeasurementId(epoch * 100 + row + 1);
      input.measurements[row].factor_id = FactorId(epoch * 100 + row + 1);
    }
    auto transaction = estimator.prepareEpoch(input);
    if (window_epochs != nullptr && window_epochs->count(epoch) != 0) {
      const auto audit = estimator.audit();
      if (marginalization_counts != nullptr) {
        (*marginalization_counts)[epoch] = audit.marginalization_count;
      }
      if (oldest_retained_epochs != nullptr) {
        (*oldest_retained_epochs)[epoch] = audit.oldest_retained_epoch;
      }
      IntegrityWindowRequest request;
      request.epochs = 10;
      windows.push_back(estimator.buildIntegrityWindow(transaction, request));
      if (unique_boundary_rows != nullptr) {
        std::size_t rows = 0;
        for (const auto& accounting : windows.back().slot_accounting) {
          if (!accounting.boundary_input) continue;
          const auto frozen = std::find_if(
              transaction.frozen_slots.begin(), transaction.frozen_slots.end(),
              [&](const auto& item) { return item.slot == accounting.slot; });
          if (frozen == transaction.frozen_slots.end()) {
            ADD_FAILURE() << "boundary slot missing from frozen transaction";
            continue;
          }
          const auto linear = frozen->factor->linearize(*transaction.frozen_values);
          if (const auto jacobian =
                  boost::dynamic_pointer_cast<gtsam::JacobianFactor>(linear)) {
            rows += static_cast<std::size_t>(jacobian->rows());
          } else if (const auto hessian =
                         boost::dynamic_pointer_cast<gtsam::HessianFactor>(linear)) {
            rows += static_cast<std::size_t>(hessian->info().rows() - 1);
          } else {
            ADD_FAILURE() << "boundary factor has no independent row count";
          }
        }
        (*unique_boundary_rows)[epoch] = rows;
      }
    }
    const auto plan = EpochCommitPlan::nominalPlan(transaction);
    estimator.commitEpoch(std::move(transaction), plan);
  }
  if (root_cache_audit != nullptr) {
    *root_cache_audit = estimator.historyRootCacheAuditForTesting();
  }
  return windows;
}

double relErr(double got, double want) {
  const double scale = std::max({1.0, std::abs(got), std::abs(want)});
  return std::abs(got - want) / scale;
}

double matrixRelErr(const Eigen::MatrixXd& got, const Eigen::MatrixXd& want) {
  return (got - want).norm() /
      std::max({1.0, got.norm(), want.norm()});
}

int tangentWidth(gtsam::Key key) {
  const char symbol = gtsam::Symbol(key).chr();
  return symbol == 'v' ? 3 : 6;
}

Eigen::MatrixXd projectorOf(const Eigen::MatrixXd& matrix) {
  if (matrix.cols() == 0) {
    return Eigen::MatrixXd::Zero(matrix.rows(), matrix.rows());
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU);
  const auto singular = svd.singularValues();
  const double gate = singular.size() == 0 ? 0.0 : singular(0) * 1e-12;
  const int rank = static_cast<int>((singular.array() > gate).count());
  return svd.matrixU().leftCols(rank) * svd.matrixU().leftCols(rank).transpose();
}

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd inverse = Eigen::VectorXd::Zero(svd.singularValues().size());
  const double gate = svd.singularValues().size() == 0
      ? 0.0 : svd.singularValues()(0) * 1e-12;
  for (int index = 0; index < inverse.size(); ++index) {
    if (svd.singularValues()(index) > gate) {
      inverse(index) = 1.0 / svd.singularValues()(index);
    }
  }
  return svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose();
}

int effectiveCarrierRankOracle(const Eigen::MatrixXd& state,
                               const Eigen::MatrixXd& fault,
                               const Eigen::VectorXd& rhs,
                               double rank_tolerance,
                               int* raw_residual_dof = nullptr) {
  Eigen::JacobiSVD<Eigen::MatrixXd> state_svd(state, Eigen::ComputeFullU);
  const Eigen::VectorXd state_singular = state_svd.singularValues();
  const double state_scale =
      state_singular.size() == 0 ? 0.0 : state_singular(0);
  int state_rank = 0;
  for (Eigen::Index i = 0; i < state_singular.size(); ++i)
    if (state_singular(i) > rank_tolerance * state_scale) ++state_rank;
  const int residual_rows = static_cast<int>(state.rows()) - state_rank;
  if (raw_residual_dof) *raw_residual_dof = residual_rows;
  Eigen::MatrixXd raw(state.rows(), fault.cols() + 1);
  if (fault.cols() > 0) raw.leftCols(fault.cols()) = fault;
  raw.col(fault.cols()) = rhs;
  const Eigen::MatrixXd carrier =
      state_svd.matrixU().rightCols(residual_rows).transpose() * raw;
  Eigen::JacobiSVD<Eigen::MatrixXd> carrier_svd(carrier);
  const Eigen::VectorXd singular = carrier_svd.singularValues();
  const double scale = singular.size() == 0 ? 0.0 : singular(0);
  const double dimension = static_cast<double>(
      std::max<Eigen::Index>(carrier.rows(), carrier.cols()));
  const double eps = std::numeric_limits<double>::epsilon();
  const double gamma = (dimension * eps) / (1.0 - dimension * eps);
  const double threshold =
      rank_tolerance * scale + gamma * carrier.norm();
  int rank = 0;
  for (Eigen::Index i = 0; i < singular.size(); ++i)
    if (singular(i) > threshold) ++rank;
  return rank;
}

const PendingFactorGroup* selectedHistoricalGroup(
    const HistoricalEpochContext& record, FactorKind kind) {
  for (const auto id : record.selected_groups) {
    for (const auto& group : record.groups) {
      if (group.id == id && group.kind == kind) return &group;
    }
  }
  return nullptr;
}

const UwbMeasurement* rawMeasurement(const HistoricalEpochContext& record,
                                     MeasurementId id) {
  for (const auto& measurement : record.uwb_batch.measurements) {
    if (measurement.id == id) return &measurement;
  }
  return nullptr;
}

struct RawOracleSystem {
  Eigen::MatrixXd old_state;
  Eigen::MatrixXd boundary;
  Eigen::MatrixXd fault;
  Eigen::VectorXd rhs;
  double constant_offset = 0.0;
  std::vector<HistoryFaultColumnId> fault_ids;
};

// Independent reconstruction used only by the P0-01 production oracle.  It
// starts from frozen slots and raw group covariance, independently creates
// per-(anchor,epoch) constant/time-linear columns, and never calls the
// history parameterization or summary builders.
RawOracleSystem reconstructRawUwbBoundary(
    const EpochTransaction& tx, const LinearizedIntegrityWindow& window,
    std::size_t window_epochs) {
  struct Metadata {
    const HistoricalEpochContext* record = nullptr;
    const PendingFactorGroup* group = nullptr;
  };
  std::map<std::uint64_t, Metadata> metadata;
  std::map<std::size_t, const HistoricalEpochContext*> records;
  for (const auto& record : tx.recoverable_history) {
    records[record.proposed_epoch] = &record;
    const auto* group = selectedHistoricalGroup(record, FactorKind::UwbBatch);
    if (group) metadata[group->id.value()] = {&record, group};
  }
  const std::size_t first = tx.oldest_recoverable_epoch + 1;
  const std::size_t window_first = tx.proposed_epoch - window_epochs;
  std::map<HistoryFaultColumnId, int> fault_index;
  std::vector<HistoryFaultColumnId> fault_ids;
  for (std::size_t epoch = first; epoch < window_first; ++epoch) {
    const auto record_it = records.find(epoch);
    if (record_it == records.end()) continue;
    const auto* group =
        selectedHistoricalGroup(*record_it->second, FactorKind::UwbBatch);
    if (!group) continue;
    std::set<std::uint64_t> anchors;
    for (const auto measurement_id : group->source_measurements) {
      const auto* measurement = rawMeasurement(*record_it->second,
                                               measurement_id);
      if (measurement) anchors.insert(measurement->anchor_id.value());
    }
    for (const auto anchor : anchors) {
      for (const auto kind : {HistoryFaultBasisKind::UwbAnchorConstant,
                              HistoryFaultBasisKind::UwbAnchorTimeLinear}) {
        HistoryFaultColumnId id{kind, anchor, epoch};
        fault_index[id] = static_cast<int>(fault_ids.size());
        fault_ids.push_back(id);
      }
    }
  }

  struct RowBlock {
    std::vector<gtsam::Key> keys;
    Eigen::MatrixXd state;
    Eigen::MatrixXd fault;
    Eigen::VectorXd rhs;
  };
  std::vector<RowBlock> blocks;
  std::set<gtsam::Key> state_keys;
  double constant_offset = 0.0;
  for (const auto& accounting : window.slot_accounting) {
    if (!accounting.boundary_input || !accounting.group_id) continue;
    const auto frozen = std::find_if(
        tx.frozen_slots.begin(), tx.frozen_slots.end(),
        [&](const auto& item) { return item.slot == accounting.slot; });
    EXPECT_NE(frozen, tx.frozen_slots.end());
    if (frozen == tx.frozen_slots.end()) continue;
    const auto linear = frozen->factor->linearize(*tx.frozen_values);
    RowBlock block;
    if (const auto jacobian =
            boost::dynamic_pointer_cast<gtsam::JacobianFactor>(linear)) {
      block.keys.assign(jacobian->keys().begin(), jacobian->keys().end());
      block.state = jacobian->getA();
      block.rhs = jacobian->getb();
    } else if (const auto hessian =
                   boost::dynamic_pointer_cast<gtsam::HessianFactor>(linear)) {
      block.keys.assign(hessian->keys().begin(), hessian->keys().end());
      int dimension = 0;
      for (const auto key : block.keys) dimension += tangentWidth(key);
      const Eigen::MatrixXd info = hessian->info().selfadjointView();
      const Eigen::MatrixXd lambda =
          info.topLeftCorner(dimension, dimension);
      const Eigen::VectorXd eta = info.topRightCorner(dimension, 1);
      Eigen::LLT<Eigen::MatrixXd> llt(lambda);
      EXPECT_EQ(llt.info(), Eigen::Success);
      block.state = llt.matrixU();
      block.rhs = llt.matrixU().transpose().solve(eta);
      constant_offset += info(dimension, dimension) - block.rhs.squaredNorm();
    } else {
      ADD_FAILURE() << "raw oracle encountered unknown linear factor";
      continue;
    }
    block.fault = Eigen::MatrixXd::Zero(
        block.state.rows(), static_cast<int>(fault_ids.size()));
    const auto meta = metadata.find(accounting.group_id->value());
    if (meta != metadata.end() &&
        meta->second.record->proposed_epoch >= first &&
        meta->second.record->proposed_epoch < window_first) {
      const auto& record = *meta->second.record;
      const auto& group = *meta->second.group;
      EXPECT_EQ(block.state.rows(),
                static_cast<Eigen::Index>(group.source_measurements.size()));
      Eigen::LLT<Eigen::MatrixXd> llt(group.raw_covariance);
      EXPECT_EQ(llt.info(), Eigen::Success);
      const Eigen::MatrixXd information = llt.solve(
          Eigen::MatrixXd::Identity(group.raw_covariance.rows(),
                                    group.raw_covariance.cols()));
      Eigen::LLT<Eigen::MatrixXd> information_llt(information);
      EXPECT_EQ(information_llt.info(), Eigen::Success);
      const Eigen::MatrixXd whitener = information_llt.matrixU();
      for (std::size_t row = 0; row < group.source_measurements.size(); ++row) {
        const auto* measurement =
            rawMeasurement(record, group.source_measurements[row]);
        EXPECT_NE(measurement, nullptr);
        if (!measurement) continue;
        const HistoryFaultColumnId constant{
            HistoryFaultBasisKind::UwbAnchorConstant,
            measurement->anchor_id.value(), record.proposed_epoch};
        const HistoryFaultColumnId linear_id{
            HistoryFaultBasisKind::UwbAnchorTimeLinear,
            measurement->anchor_id.value(), record.proposed_epoch};
        Eigen::VectorXd raw_constant = Eigen::VectorXd::Zero(block.state.rows());
        Eigen::VectorXd raw_linear = Eigen::VectorXd::Zero(block.state.rows());
        raw_constant(static_cast<Eigen::Index>(row)) = 1.0;
        raw_linear(static_cast<Eigen::Index>(row)) =
            measurement->timestamp.seconds() - record.begin.seconds();
        block.fault.col(fault_index.at(constant)) +=
            whitener * raw_constant;
        block.fault.col(fault_index.at(linear_id)) += whitener * raw_linear;
      }
    }
    for (const auto key : block.keys) state_keys.insert(key);
    blocks.push_back(std::move(block));
  }

  std::vector<gtsam::Key> old_keys;
  for (const auto key : state_keys) {
    const std::size_t epoch = gtsam::Symbol(key).index();
    if (epoch < window.detector_first_epoch || epoch > tx.previous_epoch) {
      old_keys.push_back(key);
    }
  }
  std::map<gtsam::Key, int> old_offset;
  int old_width = 0;
  for (const auto key : old_keys) {
    old_offset[key] = old_width;
    old_width += tangentWidth(key);
  }
  const int boundary_width = static_cast<int>((window_epochs + 1) * 15);
  int rows = 0;
  for (const auto& block : blocks) rows += block.state.rows();
  RawOracleSystem out;
  out.old_state = Eigen::MatrixXd::Zero(rows, old_width);
  out.boundary = Eigen::MatrixXd::Zero(rows, boundary_width);
  out.fault = Eigen::MatrixXd::Zero(rows, fault_ids.size());
  out.rhs = Eigen::VectorXd::Zero(rows);
  out.constant_offset = constant_offset;
  out.fault_ids = fault_ids;
  int row_offset = 0;
  for (const auto& block : blocks) {
    int local_offset = 0;
    for (const auto key : block.keys) {
      const int width = tangentWidth(key);
      const auto old = old_offset.find(key);
      if (old != old_offset.end()) {
        out.old_state.block(row_offset, old->second, block.state.rows(), width) =
            block.state.middleCols(local_offset, width);
      } else {
        const std::size_t epoch = gtsam::Symbol(key).index();
        const int type_offset = gtsam::Symbol(key).chr() == 'x'
            ? 0 : (gtsam::Symbol(key).chr() == 'v' ? 6 : 9);
        const int target =
            static_cast<int>((epoch - window.detector_first_epoch) * 15) +
            type_offset;
        out.boundary.block(row_offset, target, block.state.rows(), width) =
            block.state.middleCols(local_offset, width);
      }
      local_offset += width;
    }
    out.fault.middleRows(row_offset, block.state.rows()) = block.fault;
    out.rhs.segment(row_offset, block.state.rows()) = block.rhs;
    row_offset += block.state.rows();
  }
  return out;
}

// 1. The condensed boundary is the square-root history summary.
TEST(HistorySummaryPipeline, BoundaryIsTheSquareRootHistorySummary) {
  auto config = researchConfig();
  std::map<std::size_t, std::size_t> request;
  request[22] = 1;
  const auto windows = driveEstimator(config, 22, &request);
  ASSERT_EQ(windows.size(), 1u);
  const auto& window = windows.front();
  ASSERT_TRUE(window.model_valid) << window.reason;
  const auto& history = window.history_summary;
  ASSERT_TRUE(history.present);
  ASSERT_TRUE(history.valid) << history.reason;
  EXPECT_TRUE(window.capabilities.includes_boundary_prior);
  EXPECT_TRUE(window.capabilities.history_summary_present);
  EXPECT_TRUE(window.capabilities.history_summary_valid);
  EXPECT_TRUE(history.capacity_ok);
  EXPECT_EQ(history.capacity_action, "REFUSE");

  // The boundary block is the module's square-root output.
  const auto boundary = std::find_if(
      window.blocks.begin(), window.blocks.end(), [](const auto& block) {
        return block.whitening_model_id == "history_summary_sqrt_d1";
      });
  ASSERT_NE(boundary, window.blocks.end());
  EXPECT_EQ(boundary->kind, FactorKind::BoundaryPrior);
  EXPECT_EQ(static_cast<std::size_t>(boundary->jacobian_whitened.rows()),
            history.emitted_rows);
  EXPECT_GT(history.boundary_rows, 0u);
  EXPECT_GT(history.boundary_columns, 0u);
  EXPECT_GT(history.fault_columns, 0u);
  EXPECT_GT(history.injected_epochs, 0u);
  EXPECT_EQ(history.fault_columns, history.constant_columns +
                                       history.time_linear_columns +
                                       history.imu_columns);
  EXPECT_GT(history.constant_columns, 0u);
  EXPECT_GT(history.time_linear_columns, 0u);
  EXPECT_GT(history.imu_columns, 0u);
  EXPECT_EQ(history.column_ids.size(), history.fault_columns);
  EXPECT_GT(history.version_digest, 0u);

  // P0-01/D03-A: T_b owns only the state-supported condensed rows.  F_b owns
  // the detector-only rows; the two carriers are independently sized and
  // fully written, rather than sharing an emitted-row allocation with an
  // unwritten tail.
  ASSERT_GE(history.emitted_rows,
            static_cast<std::size_t>(history.nu_perp));
  const std::size_t supported_rows =
      history.emitted_rows - static_cast<std::size_t>(history.nu_perp);
  EXPECT_EQ(static_cast<std::size_t>(history.response.rows()), supported_rows);
  EXPECT_EQ(static_cast<std::size_t>(history.detector_response.rows()),
            static_cast<std::size_t>(history.nu_perp));
  EXPECT_EQ(static_cast<std::size_t>(history.response.cols()),
            history.fault_columns);
  EXPECT_EQ(static_cast<std::size_t>(history.detector_response.cols()),
            history.fault_columns);
  EXPECT_TRUE(history.response.allFinite());
  EXPECT_TRUE(history.detector_response.allFinite());

  // D-1 granularity: per (anchor, epoch) constant + time-linear columns and
  // per (axis, epoch) IMU columns, every column inside the A3 horizon.
  for (const auto& id : history.column_ids) {
    EXPECT_GE(id.epoch, history.horizon_first_epoch);
    EXPECT_LT(id.epoch, history.window_first_epoch);
    if (id.kind == HistoryFaultBasisKind::ImuAxisConstant) {
      EXPECT_LT(id.source, 6u);
    }
  }
  // The emitted rows cover exactly the block rows and the perp rows are
  // detector-only rows of the square-root context.
  EXPECT_EQ(window.square_root->detectorOnlyRows() >= history.nu_perp, true);
  SUCCEED();
}

TEST(HistorySummaryPipeline,
     ProductionWindowMatchesIndependentFrozenRawFactorOracle) {
  auto config = IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_uwb_order1.yaml");
  ASSERT_TRUE(config.resolved_scope.requiresUwbFaults());
  ASSERT_FALSE(config.resolved_scope.requiresImuFaults());
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary_sample;
  boundary_sample.timestamp = TimestampNs(0);
  boundary_sample.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary_sample);
  EpochTransaction tx;
  for (std::size_t epoch = 1; epoch <= 26; ++epoch) {
    appendImu(&estimator, config.imu.gravity_mps2, epoch, 10);
    auto pending = estimator.prepareEpoch(correlatedBiasedBatch(config, epoch));
    if (epoch == 26) {
      tx = std::move(pending);
    } else {
      const auto plan = EpochCommitPlan::nominalPlan(pending);
      estimator.commitEpoch(std::move(pending), plan);
    }
  }
  IntegrityWindowRequest request;
  request.epochs = config.integrity_window.epochs;
  const auto window = estimator.buildIntegrityWindow(tx, request);
  ASSERT_TRUE(window.model_valid) << window.reason;
  ASSERT_TRUE(window.history_summary.valid) << window.history_summary.reason;
  const auto summary_block = std::find_if(
      window.blocks.begin(), window.blocks.end(), [](const auto& block) {
        return block.whitening_model_id == "history_summary_sqrt_d1";
      });
  ASSERT_NE(summary_block, window.blocks.end());

  const RawOracleSystem raw = reconstructRawUwbBoundary(
      tx, window, config.integrity_window.epochs);
  ASSERT_GT(raw.old_state.rows(), 0);
  ASSERT_GT(raw.old_state.cols(), 0);
  ASSERT_GT(raw.boundary.cols(), 0);
  ASSERT_GT(raw.fault.cols(), 0);
  ASSERT_GT(raw.rhs.norm(), 0.0);
  ASSERT_EQ(raw.fault_ids, window.history_summary.column_ids);
  ASSERT_EQ(raw.fault.cols(),
            static_cast<Eigen::Index>(window.history_summary.fault_columns));
  bool correlated = false;
  for (const auto& record : tx.recoverable_history) {
    const auto* group = selectedHistoricalGroup(record, FactorKind::UwbBatch);
    if (group && group->raw_covariance.rows() > 1) {
      correlated = correlated ||
          (group->raw_covariance -
           Eigen::MatrixXd(group->raw_covariance.diagonal().asDiagonal()))
                  .norm() > 0.0;
    }
  }
  EXPECT_TRUE(correlated);

  const Eigen::Index supported = window.history_summary.response.rows();
  const Eigen::Index perpendicular =
      window.history_summary.detector_response.rows();
  ASSERT_EQ(summary_block->jacobian_whitened.rows(),
            supported + perpendicular);
  Eigen::MatrixXd represented_fault = Eigen::MatrixXd::Zero(
      supported + perpendicular, raw.fault.cols());
  represented_fault.topRows(supported) = window.history_summary.response;
  represented_fault.bottomRows(perpendicular) =
      window.history_summary.detector_response;
  const Eigen::MatrixXd& represented_boundary =
      summary_block->jacobian_whitened;
  const Eigen::VectorXd& represented_rhs =
      summary_block->residual_whitened;

  const Eigen::MatrixXd old_residual =
      Eigen::MatrixXd::Identity(raw.old_state.rows(), raw.old_state.rows()) -
      projectorOf(raw.old_state);
  Eigen::MatrixXd joined(raw.old_state.rows(),
                         raw.old_state.cols() + raw.boundary.cols());
  joined << raw.old_state, raw.boundary;
  const Eigen::MatrixXd all_state_residual =
      Eigen::MatrixXd::Identity(raw.old_state.rows(), raw.old_state.rows()) -
      projectorOf(joined);

  const Eigen::MatrixXd raw_state_info =
      raw.boundary.transpose() * old_residual * raw.boundary;
  const Eigen::VectorXd raw_state_rhs =
      raw.boundary.transpose() * old_residual * raw.rhs;
  const Eigen::MatrixXd represented_state_info =
      represented_boundary.transpose() * represented_boundary;
  const Eigen::VectorXd represented_state_rhs =
      represented_boundary.transpose() * represented_rhs;
  const Eigen::MatrixXd raw_state_covariance = pseudoInverse(raw_state_info);
  const Eigen::MatrixXd represented_state_covariance =
      pseudoInverse(represented_state_info);
  const Eigen::VectorXd raw_state = raw_state_covariance * raw_state_rhs;
  const Eigen::VectorXd represented_state =
      represented_state_covariance * represented_state_rhs;
  EXPECT_LE(matrixRelErr(represented_state_info, raw_state_info), 1e-8);
  EXPECT_LE(matrixRelErr(represented_state_covariance, raw_state_covariance),
            1e-8);
  EXPECT_LE(matrixRelErr(represented_state, raw_state), 1e-8);

  // T_b is checked through both its state cross-information and the complete
  // [T_b;F_b] Gram.  F_b is independently checked after eliminating every
  // state direction; these are invariant to square-root row rotations.
  const double t_cross_error = matrixRelErr(
      represented_boundary.transpose() * represented_fault,
      raw.boundary.transpose() * old_residual * raw.fault);
  const double tf_gram_error = matrixRelErr(
      represented_fault.transpose() * represented_fault,
      raw.fault.transpose() * old_residual * raw.fault);
  const double f_gram_error = matrixRelErr(
      window.history_summary.detector_response.transpose() *
          window.history_summary.detector_response,
      raw.fault.transpose() * all_state_residual * raw.fault);
  EXPECT_LE(t_cross_error, 1e-8);
  EXPECT_LE(tf_gram_error, 1e-8);
  EXPECT_LE(f_gram_error, 1e-8);

  const double raw_constant =
      raw.rhs.dot(old_residual * raw.rhs) + raw.constant_offset;
  const double represented_constant =
      represented_rhs.squaredNorm() + window.history_summary.constant_offset;
  EXPECT_LE(relErr(represented_constant, raw_constant), 1e-8);
  EXPECT_LE(relErr(window.history_summary.constant_offset,
                   raw.constant_offset), 1e-10);

  int raw_residual_dof = 0;
  const int effective_rank = effectiveCarrierRankOracle(
      joined, raw.fault, raw.rhs, config.integrity_window.rank_tolerance,
      &raw_residual_dof);
  EXPECT_EQ(window.history_summary.nu_perp, effective_rank);
  EXPECT_GT(raw_residual_dof, effective_rank);
  EXPECT_NE(window.history_summary.version_digest, 0u);

  double worst_objective = 0.0;
  for (int probe = 1; probe <= 8; ++probe) {
    Eigen::VectorXd state(raw.boundary.cols());
    Eigen::VectorXd fault(raw.fault.cols());
    for (int index = 0; index < state.size(); ++index) {
      state(index) = std::sin(0.07 * probe * (index + 1));
    }
    for (int index = 0; index < fault.size(); ++index) {
      fault(index) = std::cos(0.11 * probe * (index + 1));
    }
    const Eigen::VectorXd raw_w =
        raw.boundary * state + raw.fault * fault - raw.rhs;
    const double expected =
        raw_w.dot(old_residual * raw_w) + raw.constant_offset;
    const double represented =
        (represented_boundary * state + represented_fault * fault -
         represented_rhs).squaredNorm() +
        window.history_summary.constant_offset;
    worst_objective = std::max(worst_objective,
                               relErr(represented, expected));
  }
  EXPECT_LE(worst_objective, 1e-8);
  std::printf(
      "[P0-01-O01-PRODUCTION] rows=%ld old=%ld boundary=%ld faults=%ld "
      "rhs_norm=%.6e covariance_correlated=%d objective=%.3e state=%.3e "
      "covariance=%.3e Tcross=%.3e TFgram=%.3e Fgram=%.3e "
      "constant=%.3e dof=%d\n",
      static_cast<long>(raw.old_state.rows()),
      static_cast<long>(raw.old_state.cols()),
      static_cast<long>(raw.boundary.cols()),
      static_cast<long>(raw.fault.cols()), raw.rhs.norm(), correlated ? 1 : 0,
      worst_objective, matrixRelErr(represented_state, raw_state),
      matrixRelErr(represented_state_covariance, raw_state_covariance),
      t_cross_error, tf_gram_error, f_gram_error,
      relErr(represented_constant, raw_constant),
      window.history_summary.nu_perp);
  estimator.discardEpoch(
      std::move(tx), {FdeStatus::ModelInvalid, "oracle complete", false});
}

// 2. Pooled semantics: statistic = residual of the state-supported rows at the
//    frozen solution + kappa_b, and the dof counts the perp rows exactly once.
TEST(HistorySummaryPipeline, PooledStatisticAndDofIdentity) {
  auto config = researchConfig();
  std::map<std::size_t, std::size_t> request;
  request[22] = 1;
  const auto windows = driveEstimator(config, 22, &request);
  const auto& window = windows.front();
  ASSERT_TRUE(window.model_valid) << window.reason;
  ASSERT_TRUE(window.history_summary.valid);
  ASSERT_TRUE(window.numerics && window.numerics->valid);
  const auto& history = window.history_summary;
  ASSERT_GT(history.nu_perp, 0);

  // Rows that carry the detection-only content are the trailing rows of the
  // boundary block; identify them structurally (zero Jacobian rows) since the
  // context does exactly the same classification.
  std::vector<int> detector_only;
  for (int row = 0; row < window.H.rows(); ++row) {
    if (window.H.row(row).cwiseAbs().maxCoeff() == 0.0) {
      detector_only.push_back(row);
    }
  }
  ASSERT_GE(static_cast<int>(detector_only.size()), history.nu_perp);

  // Independently: kappa_b = ||d_perp||^2 must equal the residual energy of
  // the detector-only rows at the frozen solution (their Jacobian is zero).
  const Eigen::VectorXd& increment = window.numerics->spectral_state_increment;
  double pooled = 0.0;
  double state_supported = 0.0;
  for (int row = 0; row < window.H.rows(); ++row) {
    const double residual = window.z(row) - window.H.row(row).dot(increment);
    if (window.H.row(row).cwiseAbs().maxCoeff() == 0.0) {
      pooled += residual * residual;
    } else {
      state_supported += residual * residual;
    }
  }
  EXPECT_LE(relErr(pooled, history.kappa_b), 1e-9)
      << "detector-only residual energy must be kappa_b";
  EXPECT_LE(relErr(state_supported + pooled, window.numerics->statistic), 1e-9)
      << "T_pooled identity: ||r_c||^2 + kappa_b";
  // The pooled dof: removing the detection-only rows removes exactly nu_perp
  // residual degrees of freedom and changes nothing else.
  const int rank_without = window.numerics->exact_rank;  // zero rows add none
  EXPECT_EQ(window.dof, window.H.rows() - rank_without);
  EXPECT_EQ(window.dof, (window.H.rows() - history.nu_perp) - rank_without +
                            history.nu_perp);
}

// 3. A3 horizon honesty: the summary never claims material it does not cover.
TEST(HistorySummaryPipeline, HorizonBookkeepingDoesNotOverclaim) {
  auto config = researchConfig();
  std::map<std::size_t, std::size_t> request;
  request[22] = 1;
  const auto windows = driveEstimator(config, 22, &request);
  const auto& window = windows.front();
  ASSERT_TRUE(window.model_valid) << window.reason;
  const auto& history = window.history_summary;
  ASSERT_TRUE(history.valid);
  EXPECT_GT(history.window_first_epoch, history.horizon_first_epoch);
  EXPECT_FALSE(history.omitted_risk_source.empty());
  EXPECT_NE(history.omitted_risk_source.find("history_fault_omitted"),
            std::string::npos);
  if (history.claims_full_coverage) {
    EXPECT_EQ(history.omitted_epoch_count, 0u);
    EXPECT_EQ(history.material_gap_epoch_count, 0u);
  } else {
    EXPECT_TRUE(history.omitted_epoch_count > 0 ||
                history.material_gap_epoch_count > 0);
  }
  // The assumption text names the covered range and denies detection claims
  // outside it.
  EXPECT_NE(history.assumptions.find("no detection"), std::string::npos);
}

// 4. HIS-02: a fault whose epoch left the window keeps its response AND its
//    detection content; the protected response stays finite (monitorable).
TEST(HistorySummaryPipeline, HistoryFaultResponsePersistsAcrossBoundary) {
  auto config = researchConfig();
  std::map<std::size_t, std::size_t> request;
  request[22] = 1;
  const auto windows = driveEstimator(config, 22, &request);
  const auto& window = windows.front();
  ASSERT_TRUE(window.model_valid) << window.reason;
  const auto& history = window.history_summary;
  ASSERT_TRUE(history.valid);
  ASSERT_GT(history.fault_columns, 0u);

  // Persistent bias on the first anchor with onset inside the horizon: the
  // mode map is the exact combination of the step columns (D-1).
  const std::uint64_t anchor = history.column_ids.front().source;
  std::vector<HistoryFaultColumnId> basis = history.column_ids;
  const HistoryFaultCombination combination =
      persistentUwbCombination(anchor, history.horizon_first_epoch, basis);
  ASSERT_GT(combination.columns.size(), 0u);

  // Mode map over the emitted boundary block rows: [T_b; F_b] columns.  The
  // map lives in the window's *row* space, so it is placed at the boundary
  // block's row offset inside the stacked system (the boundary block is the
  // summary carrier; all other rows are unrelated to this mode).
  const std::size_t rows = history.emitted_rows;
  const std::size_t boundary_rows = rows - history.nu_perp;
  int boundary_row_offset = 0;
  {
    int offset = 0;
    for (const auto& block : window.blocks) {
      if (block.whitening_model_id == "history_summary_sqrt_d1") {
        boundary_row_offset = offset;
        break;
      }
      offset += static_cast<int>(block.jacobian_whitened.rows());
    }
  }
  Eigen::VectorXd local = Eigen::VectorXd::Zero(rows);
  for (std::size_t index = 0; index < combination.columns.size(); ++index) {
    std::size_t column = history.fault_columns;
    for (std::size_t candidate = 0; candidate < history.column_ids.size();
         ++candidate) {
      if (history.column_ids[candidate] == combination.columns[index]) {
        column = candidate;
        break;
      }
    }
    ASSERT_LT(column, history.fault_columns);
    local.head(boundary_rows) +=
        combination.coefficients(index) * history.response.col(column);
    local.tail(history.nu_perp) +=
        combination.coefficients(index) * history.detector_response.col(column);
  }
  EXPECT_GT(local.head(boundary_rows).norm(), 0.0)
      << "the fault still shifts the boundary (response preserved)";
  EXPECT_GT(local.tail(history.nu_perp).norm(), 0.0)
      << "the fault still has detection content (F_b preserved)";
  ASSERT_LE(boundary_row_offset + static_cast<int>(rows), window.H.rows());
  Eigen::VectorXd map = Eigen::VectorXd::Zero(window.H.rows());
  map.segment(boundary_row_offset, rows) = local;

  // Detection content in the window system: Gamma = D^T D - D^T H (H^T H)^-1
  // H^T D, computed through the same square-root context the evidence path
  // uses.  A strictly positive Gram means the history fault is monitorable.
  const Eigen::MatrixXd normal = window.H.transpose() * map;
  Eigen::MatrixXd rhs(window.H.cols(), 1);
  rhs.col(0) = normal;
  const Eigen::MatrixXd solved = solveFrozenInformation(
      window.square_root.get(), *window.numerics, window.base_information, rhs);
  const double gamma = map.dot(map) - (normal.transpose() * solved)(0, 0);
  EXPECT_GT(gamma, 0.0) << "history detection contribution must be positive";

  // Protected-state response stays finite (PL slopes are computable).
  const Eigen::MatrixXd protected_covariance =
      window.square_root->protectedResponse();
  EXPECT_TRUE(protected_covariance.allFinite());
  EXPECT_TRUE(std::isfinite(gamma));
}

// 5. The summary digest binds the window fingerprint (cache identity).
TEST(HistorySummaryPipeline, SummaryVersionBindsWindowFingerprint) {
  auto config = researchConfig();
  std::map<std::size_t, std::size_t> request;
  request[22] = 1;
  const auto windows = driveEstimator(config, 22, &request);
  const auto& window = windows.front();
  ASSERT_TRUE(window.model_valid) << window.reason;
  ASSERT_TRUE(window.history_summary.valid);
  const std::uint64_t base = integrityWindowFingerprint(window);
  auto changed = window;
  changed.history_summary.version_digest ^= 1ULL;
  EXPECT_NE(integrityWindowFingerprint(changed), base)
      << "a changed summary version must change the cache identity";
  // The detector refuses to serve frozen numerics whose content fingerprint no
  // longer matches the window (the invalidation semantics).
  DetectorRiskContext risk;
  risk.rank_tolerance = config.integrity_window.rank_tolerance;
  risk.max_condition_number = config.integrity_window.max_condition_number;
  const auto detector = JointWindowDetector().evaluate(window, risk);
  EXPECT_TRUE(detector.numerically_valid) << detector.reason;
  const auto stale = JointWindowDetector().evaluate(changed, risk);
  EXPECT_FALSE(stale.numerically_valid);
  EXPECT_NE(stale.reason.find("stale"), std::string::npos);
}

// 6. Capacity REFUSE is explicit and fail-closed (HIS-06): no truncation, the
//    window is unusable and the refusal is counted.
TEST(HistorySummaryPipeline, CapacityRefusalIsExplicitAndCounted) {
  auto config = researchConfig();
  config.history.max_fault_columns = 1;  // deliberately below q_hist
  std::map<std::size_t, std::size_t> request;
  request[22] = 1;
  NumericalWorkCounters::reset();
  const auto windows = driveEstimator(config, 22, &request);
  const auto& window = windows.front();
  ASSERT_TRUE(window.history_summary.present);
  EXPECT_FALSE(window.history_summary.valid);
  EXPECT_FALSE(window.model_valid);
  EXPECT_FALSE(window.history_summary.capacity_ok);
  EXPECT_NE(window.reason.find("capacity"), std::string::npos);
  const auto counters = NumericalWorkCounters::snapshot();
  EXPECT_GT(counters.history_capacity_refusals, 0u);
  EXPECT_EQ(window.history_summary.capacity_action, "REFUSE");
}

// 7. Row attribution (HIS-03): every frozen slot is explicit XOR boundary and
//    the condensed block never duplicates a ledger group.
TEST(HistorySummaryPipeline, RowAttributionExplicitXorBoundary) {
  auto config = researchConfig();
  std::map<std::size_t, std::size_t> request;
  request[22] = 1;
  std::map<std::size_t, std::size_t> unique_boundary_rows;
  const auto windows =
      driveEstimator(config, 22, &request, &unique_boundary_rows);
  const auto& window = windows.front();
  ASSERT_TRUE(window.model_valid) << window.reason;
  EXPECT_TRUE(window.capabilities.frozen_slot_identity_valid);
  EXPECT_TRUE(window.capabilities.every_active_factor_accounted_once);
  EXPECT_TRUE(window.capabilities.no_duplicate_rows);
  EXPECT_TRUE(window.capabilities.history_summary_valid);
  ASSERT_EQ(unique_boundary_rows.count(22), 1u);
  EXPECT_EQ(window.history_summary.boundary_rows, unique_boundary_rows.at(22))
      << "each raw boundary row/covariance owner must occur exactly once";
  for (const auto& slot : window.slot_accounting) {
    EXPECT_NE(slot.explicit_window_block, slot.boundary_input);
    EXPECT_TRUE(slot.pointer_identity_valid);
  }
  // No ledger group appears both as an explicit window block and as a
  // boundary input (no double counting with the marginal prior).
  std::set<std::uint64_t> block_groups;
  for (const auto& block : window.blocks) {
    block_groups.insert(block.group_id.value());
  }
  for (const auto& entry : window.factor_inventory) {
    const bool has_block = block_groups.count(entry.group_id.value()) != 0;
    if (entry.disposition == FrozenFactorDisposition::ExplicitMeasurement) {
      EXPECT_TRUE(has_block) << "explicit group must own a window block";
    } else if (entry.disposition == FrozenFactorDisposition::BoundaryInput) {
      EXPECT_FALSE(has_block) << "boundary group must not appear as a block";
    }
  }
  // The condensed block is not a ledger group: its content is the summary.
  const std::uint64_t synthetic = window.id.value() * 1000u;
  EXPECT_EQ(block_groups.count(synthetic), 1u);
}

TEST(HistorySummaryPipeline, UniqueOwnershipAcrossMultipleLagCrossings) {
  auto config = researchConfig();
  // The research default retains 200 epochs and never crossed a backend
  // fixed-lag boundary in the old 18/22/26 fixture.  Twenty-one is the
  // smallest valid lag for the configured 10 detector + 10 recovery epochs;
  // requests 22/24/26 therefore observe three distinct, real crossings.
  config.incremental.fixed_lag_epochs = 21;
  std::map<std::size_t, std::size_t> request{{22, 1}, {24, 1}, {26, 1}};
  std::map<std::size_t, std::size_t> unique_boundary_rows;
  std::map<std::size_t, std::uint64_t> marginalization_counts;
  std::map<std::size_t, std::size_t> oldest_retained_epochs;
  const auto windows =
      driveEstimator(config, 26, &request, &unique_boundary_rows,
                     &marginalization_counts, &oldest_retained_epochs);
  ASSERT_EQ(windows.size(), request.size());
  std::size_t index = 0;
  std::uint64_t previous_crossings = 0;
  std::size_t previous_oldest = 0;
  for (const auto& item : request) {
    const auto& window = windows.at(index++);
    ASSERT_TRUE(window.model_valid) << "epoch " << item.first << ": "
                                    << window.reason;
    const auto& history = window.history_summary;
    ASSERT_TRUE(history.valid) << history.reason;
    ASSERT_EQ(unique_boundary_rows.count(item.first), 1u);
    EXPECT_EQ(history.boundary_rows, unique_boundary_rows.at(item.first));
    EXPECT_EQ(history.response.rows(),
              static_cast<Eigen::Index>(history.emitted_rows -
                                        static_cast<std::size_t>(history.nu_perp)));
    EXPECT_EQ(history.detector_response.rows(), history.nu_perp);
    EXPECT_TRUE(history.response.allFinite());
    EXPECT_TRUE(history.detector_response.allFinite());
    ASSERT_EQ(marginalization_counts.count(item.first), 1u);
    ASSERT_EQ(oldest_retained_epochs.count(item.first), 1u);
    EXPECT_GT(marginalization_counts.at(item.first), previous_crossings);
    EXPECT_GT(oldest_retained_epochs.at(item.first), previous_oldest);
    previous_crossings = marginalization_counts.at(item.first);
    previous_oldest = oldest_retained_epochs.at(item.first);
    std::printf(
        "[P0-01-CROSSING] epoch=%zu marginalizations=%llu "
        "oldest_retained=%zu boundary_rows=%zu\n",
        item.first,
        static_cast<unsigned long long>(marginalization_counts.at(item.first)),
        oldest_retained_epochs.at(item.first), history.boundary_rows);
  }
}

// 8. Ordering: material removed before the summary update fails explicitly
//    (the guard never silently builds a summary without the deleted content).
TEST(HistorySummaryPipeline, DeletedMaterialFailsBeforeSummaryUpdate) {
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary;
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  for (std::size_t epoch = 1; epoch <= 22; ++epoch) {
    appendImu(&estimator, config.imu.gravity_mps2, epoch, 10);
    auto input = batch(config, static_cast<std::int64_t>(epoch) * 50000000);
    for (std::size_t row = 0; row < input.measurements.size(); ++row) {
      input.measurements[row].id = MeasurementId(epoch * 100 + row + 1);
      input.measurements[row].factor_id = FactorId(epoch * 100 + row + 1);
    }
    auto transaction = estimator.prepareEpoch(input);
    if (epoch == 22) {
      // Simulate the failure mode "fixed-lag deletion happened before the
      // summary consumed the material": drop one recoverable epoch record for
      // an epoch that is still inside the A3 horizon.  The guard must fail the
      // window instead of silently claiming the interval.
      IntegrityWindowRequest request;
      request.epochs = 10;
      const auto reference =
          estimator.buildIntegrityWindow(transaction, request);
      ASSERT_TRUE(reference.history_summary.valid) << reference.reason;
      const auto before_reuse = estimator.historyRootCacheAuditForTesting();
      const auto repeated = estimator.buildIntegrityWindow(transaction, request);
      ASSERT_TRUE(repeated.history_summary.valid) << repeated.reason;
      const auto after_reuse = estimator.historyRootCacheAuditForTesting();
      EXPECT_EQ(after_reuse.exact_hits, before_reuse.exact_hits + 1);
      EXPECT_EQ(after_reuse.full_rebuilds, before_reuse.full_rebuilds);
      EXPECT_EQ((repeated.history_summary.response -
                 reference.history_summary.response).norm(), 0.0);
      EXPECT_EQ((repeated.history_summary.detector_response -
                 reference.history_summary.detector_response).norm(), 0.0);
      EXPECT_EQ((repeated.history_summary.d_perp -
                 reference.history_summary.d_perp).norm(), 0.0);
      const std::size_t horizon_first =
          reference.history_summary.horizon_first_epoch;
      ASSERT_GE(transaction.recoverable_history.size(), 2u);
      std::vector<HistoricalEpochContext> keep;
      bool dropped = false;
      for (const auto& record : transaction.recoverable_history) {
        if (!dropped && record.proposed_epoch == horizon_first) {
          dropped = true;
          continue;
        }
        keep.push_back(record);
      }
      ASSERT_TRUE(dropped) << "horizon first epoch has no recoverable record";
      transaction.recoverable_history = keep;
      const auto window = estimator.buildIntegrityWindow(transaction, request);
      EXPECT_FALSE(window.model_valid);
      EXPECT_NE(window.reason.find("deleted before the summary update"),
                std::string::npos)
          << window.reason;
      // §6 cold start / §7.6 row 7: the state is explicit and the summary is
      // unusable -- reconstruction from a nominal marginal is forbidden, so
      // the window refuses instead of degrading silently.
      EXPECT_FALSE(window.history_summary.valid);
      EXPECT_EQ(window.history_summary.state, "HISTORY_SUMMARY_INVALID");
      EXPECT_FALSE(window.capabilities.history_summary_valid);
    }
    const auto plan = EpochCommitPlan::nominalPlan(transaction);
    estimator.commitEpoch(std::move(transaction), plan);
  }
}

TEST(HistorySummaryPipeline, P103ProductionClosureAudit) {
  const auto config = researchConfig();
  std::map<std::size_t, std::size_t> requested;
  for (std::size_t epoch = 1; epoch <= 30; ++epoch) requested[epoch] = epoch;
  HistoryRootCacheAudit audit;
  const auto windows = driveEstimator(config, 30, &requested, nullptr, nullptr,
                                      nullptr, &audit);
  ASSERT_EQ(windows.size(), 30u);
  EXPECT_EQ(audit.requests, 30u);
  EXPECT_EQ(audit.full_tree_rebuilds + audit.incremental_path_updates +
                audit.exact_hits,
            audit.requests);
  EXPECT_GT(audit.incremental_path_updates, 0u);
  EXPECT_LT(audit.full_tree_rebuilds, audit.requests);
  EXPECT_GT(audit.incremental_group_updates, 0u);
  EXPECT_GT(audit.reused_groups, 0u);
  EXPECT_GT(audit.factor_group_hits, 0u);
  EXPECT_EQ(audit.full_oracle_checks, audit.requests);
  EXPECT_EQ(audit.full_oracle_mismatches, 0u);
  std::printf("[P103-PRODUCTION-CLOSURE] requests=%llu append=%llu paths=%llu "
              "add=%llu remove=%llu relinearize=%llu internal=%llu tree_full=%llu "
              "group_updates=%llu reused_groups=%llu rebuilt_groups=%llu "
              "factor_hits=%llu factor_misses=%llu "
              "exact=%llu rebuilds=%llu invalidations=%llu oracle=%llu/%llu "
              "oracle_max=%.3e rows=%zu bytes=%zu reason=%s\n",
              static_cast<unsigned long long>(audit.requests),
              static_cast<unsigned long long>(audit.incremental_appends),
              static_cast<unsigned long long>(audit.incremental_path_updates),
              static_cast<unsigned long long>(audit.incremental_add_paths),
              static_cast<unsigned long long>(audit.incremental_remove_paths),
              static_cast<unsigned long long>(audit.incremental_relinearize_paths),
              static_cast<unsigned long long>(audit.internal_nodes_recomputed),
              static_cast<unsigned long long>(audit.full_tree_rebuilds),
              static_cast<unsigned long long>(audit.incremental_group_updates),
              static_cast<unsigned long long>(audit.reused_groups),
              static_cast<unsigned long long>(audit.rebuilt_groups),
              static_cast<unsigned long long>(audit.factor_group_hits),
              static_cast<unsigned long long>(audit.factor_group_misses),
              static_cast<unsigned long long>(audit.exact_hits),
              static_cast<unsigned long long>(audit.full_rebuilds),
              static_cast<unsigned long long>(audit.invalidations),
              static_cast<unsigned long long>(audit.full_oracle_checks),
              static_cast<unsigned long long>(audit.full_oracle_mismatches),
              audit.max_oracle_relative_error,
              audit.retained_rows, audit.retained_bytes,
              audit.last_reason.c_str());
}

// 9. HIS-01 closure / readiness gaps 3+4: oracle on the *shipped* carrier.
//    The boundary block plus the carrier describe exactly the condensed row
//    system [R_b  T_b ; 0  F_b] with rhs [d_b ; d_perp].  Re-summarizing those
//    rows must reproduce the shipped quantities, and doing it in the reversed
//    row order must reproduce every sign-invariant quantity (multiple
//    elimination orders at the pipeline level, running on the artifact the
//    detector consumes).  The same test records how large the information-form
//    constant offset is, i.e. the part of the detector constant that rows
//    cannot express.
TEST(HistorySummaryPipeline, CarrierInvariantsUnderRowPermutation) {
  auto config = researchConfig();
  std::map<std::size_t, std::size_t> request;
  request[22] = 1;
  const auto windows = driveEstimator(config, 22, &request);
  ASSERT_EQ(windows.size(), 1u);
  const auto& window = windows.front();
  ASSERT_TRUE(window.model_valid) << window.reason;
  const auto& history = window.history_summary;
  ASSERT_TRUE(history.valid) << history.reason;
  ASSERT_GT(history.nu_perp, 0);
  ASSERT_GT(history.fault_columns, 0u);
  const auto boundary_block =
      std::find_if(window.blocks.begin(), window.blocks.end(),
                   [](const auto& block) {
                     return block.whitening_model_id ==
                            "history_summary_sqrt_d1";
                   });
  ASSERT_NE(boundary_block, window.blocks.end());
  const int rows = static_cast<int>(history.emitted_rows);
  ASSERT_EQ(boundary_block->jacobian_whitened.rows(), rows);
  ASSERT_EQ(boundary_block->residual_whitened.size(), rows);

  // Row system exactly as the carrier stores it.  The boundary epochs need not
  // be a prefix of the window layout, so the boundary columns are recovered
  // structurally: they are exactly the nonzero columns of the shipped triangle
  // (R_b has a nonzero diagonal), in window column order (which is the
  // summary's canonical epoch-ascending order).
  std::vector<int> boundary_columns;
  for (int column = 0; column < boundary_block->jacobian_whitened.cols();
       ++column) {
    if (boundary_block->jacobian_whitened.col(column).cwiseAbs().maxCoeff() >
        0.0) {
      boundary_columns.push_back(column);
    }
  }
  ASSERT_EQ(boundary_columns.size(), history.boundary_columns);
  const int boundary_rows = static_cast<int>(history.emitted_rows) - history.nu_perp;
  Eigen::MatrixXd state_map =
      Eigen::MatrixXd::Zero(boundary_rows + history.nu_perp,
                            static_cast<int>(boundary_columns.size()));
  for (std::size_t index = 0; index < boundary_columns.size(); ++index) {
    state_map.col(static_cast<int>(index)) =
        boundary_block->jacobian_whitened.col(boundary_columns[index]);
  }
  // The shipped state-supported rows must form an upper triangle in this order
  // (the summary's contract); a violation means the window mapping is wrong.
  int diagonal_zeros = 0;
  if (boundary_rows > 1) {
    const Eigen::MatrixXd triangle = state_map.topRows(boundary_rows);
    const Eigen::MatrixXd upper =
        Eigen::MatrixXd(triangle.triangularView<Eigen::Upper>());
    EXPECT_LE((triangle - upper).norm(), 1e-12 * (1.0 + triangle.norm()))
        << "R_b must be upper triangular in the window column order";
    for (int index = 0; index < triangle.rows() && index < triangle.cols();
         ++index) {
      if (std::abs(triangle(index, index)) == 0.0) ++diagonal_zeros;
    }
    EXPECT_EQ(diagonal_zeros, 0)
        << "the shipped boundary triangle must have a nonzero diagonal";
  }
  Eigen::MatrixXd fault_map = Eigen::MatrixXd::Zero(rows, history.fault_columns);
  fault_map.topRows(boundary_rows) = history.response;
  fault_map.bottomRows(history.nu_perp) = history.detector_response;
  Eigen::VectorXd rhs = boundary_block->residual_whitened;
  // The carrier stores only the detector-only residual; d_b is the leading part
  // of the block residual, so both are read from the shipped block.
  const Eigen::VectorXd shipped_d_b = rhs.head(boundary_rows);
  // The carrier's d_perp is the detector-only residual the block carries.
  EXPECT_LE((rhs.tail(history.nu_perp) - history.d_perp).norm(),
            1e-12 * (1.0 + history.d_perp.norm()));

  auto summarize = [&](const Eigen::MatrixXd& h, const Eigen::MatrixXd& a,
                       const Eigen::VectorXd& z) {
    HistoryFaultSummaryInput input;
    input.h_old_state = Eigen::MatrixXd::Zero(h.rows(), 0);
    input.h_boundary = h;
    input.fault_map = a;
    input.rhs = z;
    return buildHistoryFaultSummary(input);
  };
  // (a) Shipped sign-invariant forms (R_b is read from the shipped block, T_b /
  //     F_b / d_perp from the carrier, d_b from the block residual).  The
  //     compared quantities are the normal-equation forms R_b'R_b, R_b'T_b,
  //     R_b'd_b, F_b'F_b and kappa_b: each is invariant under the +-1 row signs
  //     a different elimination order may introduce, and together they are the
  //     condensed system the detector consumes.
  const Eigen::MatrixXd shipped_r = state_map.topRows(boundary_rows);
  const Eigen::MatrixXd shipped_gram = shipped_r.transpose() * shipped_r;
  const Eigen::MatrixXd shipped_response = shipped_r.transpose() * history.response;
  const Eigen::VectorXd shipped_linear = shipped_r.transpose() * shipped_d_b;
  const Eigen::MatrixXd shipped_perp =
      history.detector_response.transpose() * history.detector_response;
  const double shipped_kappa = history.d_perp.squaredNorm();
  auto compareToShipped = [&](const HistoryFaultSummary& candidate,
                              const char* label) {
    ASSERT_TRUE(candidate.valid) << candidate.invalid_reason;
    EXPECT_EQ(candidate.n_boundary, static_cast<int>(boundary_rows)) << label;
    EXPECT_EQ(candidate.nuPerp(), history.nu_perp) << label;
    const Eigen::MatrixXd candidate_r = candidate.R_b;
    EXPECT_LE((candidate_r.transpose() * candidate_r - shipped_gram).norm(),
              1e-9 * std::max(1.0, shipped_gram.norm()))
        << label << " (R_b'R_b)";
    EXPECT_LE((candidate_r.transpose() * candidate.T_b - shipped_response).norm(),
              1e-9 * std::max(1.0, shipped_response.norm()))
        << label << " (R_b'T_b)";
    EXPECT_LE((candidate_r.transpose() * candidate.d_b - shipped_linear).norm(),
              1e-9 * std::max(1.0, shipped_linear.norm()))
        << label << " (R_b'd_b)";
    EXPECT_LE((candidate.F_b.transpose() * candidate.F_b - shipped_perp).norm(),
              1e-9 * std::max(1.0, shipped_perp.norm()))
        << label << " (F_b'F_b)";
    EXPECT_LE(std::abs(candidate.d_perp.squaredNorm() - shipped_kappa),
              1e-9 * std::max(1.0, std::abs(shipped_kappa)))
        << label << " (kappa_b)";
  };
  // (b) Round trip: the shipped rows reproduce the shipped summary.
  const HistoryFaultSummary identity = summarize(state_map, fault_map, rhs);
  compareToShipped(identity, "round trip");
  // (c) Reversed row order: a different elimination order of the same rows must
  //     reproduce the same sign-invariant content.
  Eigen::MatrixXd reversed_state = state_map.colwise().reverse();
  Eigen::MatrixXd reversed_fault = fault_map.colwise().reverse();
  Eigen::VectorXd reversed_rhs = rhs.reverse();
  const HistoryFaultSummary permuted =
      summarize(reversed_state, reversed_fault, reversed_rhs);
  compareToShipped(permuted, "reversed row order");

  // (c) Constant accounting (readiness gap 4): the row constant is kappa_b; the
  // information-form part can only be added explicitly, so it is exported.  It
  // must be finite and its measured size is reported by the table below.
  EXPECT_TRUE(std::isfinite(history.kappa_b));
  EXPECT_TRUE(std::isfinite(history.constant_offset));
  std::printf(
      "[HSP-ORACLE] rows=%d boundary_rows=%d q=%zu nu_perp=%d kappa=%.6e "
      "offset=%.3e offset_ratio=%.3e omega_trace=%.6e xi_norm=%.6e "
      "reversed_rows=%d permuted_nu=%d\n",
      rows, boundary_rows, history.fault_columns, history.nu_perp,
      history.kappa_b, history.constant_offset,
      std::abs(history.constant_offset) /
          std::max(1.0, std::abs(history.kappa_b)),
      history.omega().trace(), history.xi().norm(), rows, permuted.nuPerp());
}

}  // namespace
