#pragma once

// C1-b Part A (design freeze history-summary-design.md §7.6/§11.2; D-1/D-2
// decisions): historical fault parameterization.
//
// Purpose: build the fault columns of the *recoverable history* (epochs that
// have left the integrity window but are still recoverable) from the same
// material the boundary construction consumes, and provide the injection
// mechanism that rebuilds linear factors with fault keys in the separator
// (state keys are eliminated, fault keys never).  The frozen graph, the
// window and groups marked `UnrecoverableHistory` are never touched.
//
// Material (single source of truth): `EpochTransaction::recoverable_history`
// (`HistoricalEpochContext`: previous/current state, preintegration, raw IMU
// slice, UWB batch, selected groups).
//
// Granularity (D-1, frozen):
//   * UWB: one constant column per (anchor, historical epoch), rows in the
//     epoch group's `source_measurements` order;
//   * UWB: one time-linear companion column per (anchor, historical epoch)
//     (value t_meas - t_epoch_begin) so that ramp modes are EXACT linear
//     combinations of the step basis (see `rampUwbCombination`);
//   * IMU: one interval-constant column per (axis, historical epoch), built
//     with the same analytic sensitivity path as the window generator
//     (`ImuFaultSubspaceBuilder::buildAnalytic`); axis 0..2 accel, 3..5 gyro;
//   * double faults get no columns: they are combinations of single-source
//     columns by construction.
//
// Whitening: columns are produced RAW (selection/sensitivity patterns in the
// group's residual row order) and WHITENED exactly once with the group
// whitener (`W = L^-1`, `covariance = L L^T`, mirroring
// `linearizePendingGroup`); the window generator applies the identical
// convention.
//
// Horizon (A3): the covered range is the recoverable range
// `[oldest_recoverable_epoch + 1, window_first_epoch)` - the boundary epoch
// itself is excluded because no recoverable record exists for it (the
// estimator builds records strictly newer than `oldest_recoverable_epoch`);
// epochs before it are omitted / unrecoverable and are never claimed.  The
// canonical assumption text and the risk-ledger omitted-source id are
// produced by the plan (`validityAssumptions()`, `omittedRiskSource()`); the
// production plumbing into the run identity and the ledger is Part C.
//
// Capacity (A4/D-2): `evaluateHistoryFaultCapacity()` implements the REFUSE
// semantics (`HISTORY_CAPACITY_EXCEEDED`); RESET/STOP_PROTECTED are explicit
// actions and the oldest fault is never silently dropped.  Enforcement is not
// wired into the pipeline yet (Part C).
//
// Status: Part A module implemented and locked by
// `HistoryFaultParameterization.*` tests.  Pipeline wiring is Part C.

#include <gtsam/linear/GaussianFactorGraph.h>

#include <Eigen/Core>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "uwb_imu_pl/estimation/epoch_transaction.hpp"

namespace uwb_imu_pl {

enum class HistoryFaultBasisKind {
  UwbAnchorConstant = 0,
  UwbAnchorTimeLinear = 1,
  ImuAxisConstant = 2,
};

const char* toString(HistoryFaultBasisKind kind);

struct HistoryFaultColumnId {
  HistoryFaultBasisKind kind = HistoryFaultBasisKind::UwbAnchorConstant;
  // UWB: anchor id.  IMU: 0..2 accelerometer axis, 3..5 gyroscope axis.
  std::uint64_t source = 0;
  std::size_t epoch = 0;
  bool operator==(const HistoryFaultColumnId& other) const;
  bool operator<(const HistoryFaultColumnId& other) const;
};

struct HistoryFaultColumn {
  HistoryFaultColumnId id;
  FactorGroupId group;
  Eigen::MatrixXd raw_map;       // rows x 1, group residual row order
  Eigen::MatrixXd whitened_map;  // rows x 1 = group whitener * raw_map
  double time_begin_s = 0.0;
  double time_end_s = 0.0;
};

struct HistoryFaultHorizon {
  std::size_t first_epoch = 0;
  std::size_t window_first_epoch = 0;
  std::string source = "derived:tx.oldest_recoverable_epoch";
  bool covers(std::size_t epoch) const {
    return epoch >= first_epoch && epoch < window_first_epoch;
  }
};

struct HistoryFaultParameterizationOptions {
  bool include_uwb_faults = true;
  bool include_imu_faults = true;
  bool include_time_linear_basis = true;
  std::string scope_digest;
  // Explicit horizon override (recorded in `horizon.source`); defaults to the
  // derived recoverable boundary.
  std::optional<std::size_t> configured_first_epoch;
};

struct HistoryFaultParameterizationPlan {
  HistoryFaultHorizon horizon;
  // Exact five-mode scope under which these columns were constructed.  It is
  // copied into the summary version/fingerprint so material from another
  // profile can never be reused as cache-equivalent history.
  std::string scope_digest;
  std::vector<HistoryFaultColumn> columns;
  // Epochs strictly below the horizon: unrecoverable, never claimed.
  std::size_t omitted_epoch_count = 0;
  std::size_t omitted_epoch_begin = 0;
  std::size_t omitted_epoch_end = 0;
  // Requested-but-material-missing epochs (only possible with an explicit
  // widened horizon): the plan must not claim them either.
  std::size_t material_gap_epoch_count = 0;
  std::size_t skipped_columns = 0;
  std::size_t q_hist() const { return columns.size(); }
  bool claimsFullCoverage() const {
    return omitted_epoch_count == 0 && material_gap_epoch_count == 0;
  }
  std::string validityAssumptions() const;
  std::string omittedRiskSource() const;
};

HistoryFaultParameterizationPlan planHistoryFaultParameterization(
    const EpochTransaction& tx, std::size_t window_epochs,
    const HistoryFaultParameterizationOptions& options = {});

// Single-epoch builders (public so the window/generator path and the
// historical path can be compared on identical material; the plan calls
// these).  `skipped` counts columns that could not be built (non-SPD
// covariance, invalid preintegration contract).
std::vector<HistoryFaultColumn> buildHistoricalUwbColumnsForEpoch(
    const HistoricalEpochContext& epoch, std::size_t* skipped = nullptr);
std::vector<HistoryFaultColumn> buildHistoricalImuColumnsForEpoch(
    const HistoricalEpochContext& epoch, std::size_t* skipped = nullptr);

// Exact single-source mode combinations over the step basis (constructive T;
// the zero-residual claim is asserted by tests against the direct mode
// formula).  Coefficients are returned parallel to `columns`.
struct HistoryFaultCombination {
  std::vector<HistoryFaultColumnId> columns;
  Eigen::VectorXd coefficients;
};

// Persistent anchor bias at `onset_epoch`: coefficient 1 on every constant
// column of the anchor for epochs >= onset.
HistoryFaultCombination persistentUwbCombination(
    std::uint64_t anchor, std::size_t onset_epoch,
    const std::vector<HistoryFaultColumnId>& basis);

// Ramp anchor bias at `onset_epoch`: value t_meas - t_onset on the anchor
// rows.  Exact over the step basis because
// t - t_onset = (t - t_epoch_begin) + (t_epoch_begin - t_onset).
HistoryFaultCombination rampUwbCombination(
    std::uint64_t anchor, std::size_t onset_epoch, double onset_time_s,
    const std::map<std::size_t, double>& epoch_begin_s,
    const std::vector<HistoryFaultColumnId>& basis);

// Rebuilt linear factors with fault keys in the separator.  The input epoch
// group factors are linearized at `linearization`; the epoch's mapped fault
// columns are appended as new key columns (`gtsam::Symbol('f', index)`).
// `first_fault_index` lets a caller inject several epochs into one system
// without colliding fault keys; the assignment order (and therefore the key
// -> column association) is the column order passed in.  The input context
// is never modified.
struct HistoryFaultInjectionResult {
  bool valid = false;
  std::string reason;
  gtsam::GaussianFactorGraph graph;
  gtsam::KeyVector state_keys;
  gtsam::KeyVector fault_keys;
  std::size_t rows = 0;
};

HistoryFaultInjectionResult buildHistoricalFaultInjection(
    const HistoricalEpochContext& epoch, const gtsam::Values& linearization,
    const std::vector<HistoryFaultColumn>& columns,
    std::uint64_t first_fault_index = 1);

// D-2 capacity semantics (enforcement is not wired into the pipeline yet):
// REFUSE is the default action; an over-limit plan is unusable and counted by
// the caller, never truncated.  RESET / STOP_PROTECTED are explicit actions
// that require their own evidence.
struct HistoryFaultCapacityLimits {
  std::uint64_t max_fault_columns = 0;
  std::string capacity_action = "REFUSE";
};

struct HistoryFaultCapacityDecision {
  bool fits = true;
  bool unusable = false;
  std::string reason;  // "HISTORY_CAPACITY_EXCEEDED" when refusing
  std::string action;
};

HistoryFaultCapacityDecision evaluateHistoryFaultCapacity(
    std::size_t q_hist, const HistoryFaultCapacityLimits& limits);

}  // namespace uwb_imu_pl
