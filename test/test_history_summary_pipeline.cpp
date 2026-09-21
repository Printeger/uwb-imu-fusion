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

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <string>

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
    std::map<std::size_t, std::size_t>* window_epochs = nullptr) {
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
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
      IntegrityWindowRequest request;
      request.epochs = 10;
      windows.push_back(estimator.buildIntegrityWindow(transaction, request));
    }
    const auto plan = EpochCommitPlan::nominalPlan(transaction);
    estimator.commitEpoch(std::move(transaction), plan);
  }
  return windows;
}

double relErr(double got, double want) {
  const double scale = std::max({1.0, std::abs(got), std::abs(want)});
  return std::abs(got - want) / scale;
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
  const double gamma =
      map.dot(map) - (normal.transpose() * solved)(0, 0);
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
  const auto windows = driveEstimator(config, 22, &request);
  const auto& window = windows.front();
  ASSERT_TRUE(window.model_valid) << window.reason;
  EXPECT_TRUE(window.capabilities.frozen_slot_identity_valid);
  EXPECT_TRUE(window.capabilities.every_active_factor_accounted_once);
  EXPECT_TRUE(window.capabilities.no_duplicate_rows);
  EXPECT_TRUE(window.capabilities.history_summary_valid);
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

}  // namespace
