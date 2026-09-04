#pragma once

#include "uwb_imu_pl/common/integrity_ids.hpp"
#include "uwb_imu_pl/integrity/estimation_snapshot.hpp"

#include <gtsam/inference/Key.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace uwb_imu_pl {

struct PendingFactorGroup {
  FactorGroupId id;
  FactorKind kind = FactorKind::Unknown;
  SensorType sensor = SensorType::Unknown;
  gtsam::NonlinearFactorGraph factors;
  std::vector<gtsam::Key> keys;
  std::vector<MeasurementId> source_measurements;
  std::vector<FaultUnitId> fault_units;
  std::vector<FaultUnitId> excluded_fault_units;
  std::string noise_model_id;
  std::string model_id;
  // Exactly one transition group and the all-in UWB group are nominal.
  // Alternative retained-covariance groups are prepared for atomic FDE commits.
  bool nominal = true;
  Eigen::MatrixXd raw_covariance;
};

struct ImuQueueCursor {
  std::size_t consume_count = 0;
  std::optional<ImuMeasurement> new_boundary;
};

struct EpochTransaction {
  TransactionId id;
  std::uint64_t base_graph_version = 0;
  LinearizationVersion base_version;
  std::size_t previous_epoch = 0;
  std::size_t proposed_epoch = 0;
  TimestampNs begin;
  TimestampNs end;
  NavigationState previous_state;
  NavigationState nominal_predicted_state;
  std::vector<ImuMeasurement> raw_imu_slice;
  std::shared_ptr<const gtsam::PreintegratedCombinedMeasurements> preintegration;
  UwbBatch uwb_batch;
  PendingFactorGroup imu_group;
  std::vector<PendingFactorGroup> uwb_groups;
  PendingFactorGroup generic_bridge_group;
  std::optional<PendingFactorGroup> dynamics_bridge_group;
  ImuQueueCursor imu_cursor;
  bool backend_mutated = false;
};

struct EpochCommitPlan {
  // Empty means no new group. Call nominalPlan() for the all-in action.
  std::vector<FactorGroupId> groups_to_add;
  std::vector<FactorGroupId> groups_to_remove;
  BridgeMode bridge_mode = BridgeMode::None;
  ExclusionActionId action_id;
  FdeStatus fde_status = FdeStatus::NotTriggered;
  bool best_effort_integrity_unavailable = false;

  static EpochCommitPlan nominalPlan(const EpochTransaction& transaction) {
    EpochCommitPlan plan;
    plan.groups_to_add.push_back(transaction.imu_group.id);
    for (const auto& group : transaction.uwb_groups) {
      if (group.nominal) plan.groups_to_add.push_back(group.id);
    }
    plan.fde_status = FdeStatus::SuccessKeepAll;
    return plan;
  }
};

struct CommitReceipt {
  TransactionId transaction_id;
  ExclusionActionId action_id;
  std::uint64_t graph_version = 0;
  std::size_t committed_epoch = 0;
  TimestampNs state_timestamp;
  std::vector<std::size_t> added_factor_slots;
  std::vector<std::size_t> removed_factor_slots;
  std::vector<gtsam::Key> marginalized_keys;
  std::vector<std::size_t> boundary_factor_slots;
  std::uint32_t backend_updates = 0;
  bool integrity_available = false;
};

struct DiscardReason {
  FdeStatus status = FdeStatus::ModelInvalid;
  std::string detail;
  bool controlled_reinitialization_required = false;
};

struct DiscardReceipt {
  TransactionId transaction_id;
  TimestampNs attempted_timestamp;
  TimestampNs last_committed_timestamp;
  std::uint32_t backend_updates = 0;
  bool stale_state = true;
  bool controlled_reinitialization_required = false;
  std::string reason;
};

}  // namespace uwb_imu_pl
