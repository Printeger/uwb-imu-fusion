#pragma once

#include "uwb_imu_pl/estimation/epoch_transaction.hpp"

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <map>
#include <optional>
#include <vector>

namespace uwb_imu_pl {

struct FactorLedgerEntry {
  FactorId factor_id;
  FactorGroupId group_id;
  SensorType sensor = SensorType::Unknown;
  FactorKind kind = FactorKind::Unknown;
  std::vector<gtsam::Key> keys;
  std::vector<MeasurementId> source_measurements;
  std::vector<FaultUnitId> associated_fault_units;
  std::size_t epoch_begin = 0;
  std::size_t epoch_end = 0;
  TimestampNs time_begin;
  TimestampNs time_end;
  std::optional<gtsam::FactorIndex> backend_slot;
  FactorLifecycle lifecycle = FactorLifecycle::Pending;
  HealthState health_at_commit = HealthState::Healthy;
  std::string noise_model_id;
  std::string model_id;
  LinearizationVersion commit_version;
  gtsam::NonlinearFactor::shared_ptr factor;
};

class FactorLedger {
 public:
  void recordInitialPriors(const gtsam::NonlinearFactorGraph& priors,
                           const std::vector<std::size_t>& slots,
                           TimestampNs timestamp,
                           const LinearizationVersion& version);
  void recordPending(const EpochTransaction& transaction);
  void activate(FactorGroupId group, const std::vector<std::size_t>& slots,
                const LinearizationVersion& version);
  void transition(FactorGroupId group, FactorLifecycle lifecycle);
  void markSlotsAbsent(const gtsam::NonlinearFactorGraph& active_graph,
                       std::size_t oldest_retained_epoch);
  std::vector<FactorLedgerEntry> entries() const;
  std::vector<FactorLedgerEntry> activeEntries(std::size_t epoch_begin,
                                                std::size_t epoch_end) const;
  std::vector<std::size_t> activeSlots(FactorGroupId group) const;
  bool hasCompleteActiveProvenance() const;
  bool hasCompleteActiveProvenance(
      const gtsam::NonlinearFactorGraph& active_graph) const;
  std::uint64_t version() const { return version_; }

 private:
  std::map<std::uint64_t, std::vector<FactorLedgerEntry>> groups_;
  std::uint64_t version_ = 0;
};

}  // namespace uwb_imu_pl
