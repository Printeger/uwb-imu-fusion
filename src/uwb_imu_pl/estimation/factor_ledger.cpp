#include "uwb_imu_pl/estimation/factor_ledger.hpp"

#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <algorithm>
#include <set>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

void appendGroup(std::map<std::uint64_t, std::vector<FactorLedgerEntry>>* groups,
                 const PendingFactorGroup& group, const EpochTransaction& tx) {
  auto& entries = (*groups)[group.id.value()];
  if (!entries.empty()) throw std::logic_error("factor group ID reused");
  std::uint64_t local = 0;
  for (const auto& factor : group.factors) {
    FactorLedgerEntry entry;
    entry.factor_id = FactorId((group.id.value() << 8U) | (++local & 0xffU));
    entry.group_id = group.id;
    entry.sensor = group.sensor;
    entry.kind = group.kind;
    entry.keys = group.keys;
    entry.source_measurements = group.source_measurements;
    entry.associated_fault_units = group.fault_units;
    entry.epoch_begin = tx.previous_epoch;
    entry.epoch_end = tx.proposed_epoch;
    entry.time_begin = tx.begin;
    entry.time_end = tx.end;
    entry.noise_model_id = group.noise_model_id;
    entry.model_id = group.model_id;
    entry.factor = factor;
    entries.push_back(std::move(entry));
  }
}

}  // namespace

void FactorLedger::recordInitialPriors(
    const gtsam::NonlinearFactorGraph& priors,
    const std::vector<std::size_t>& slots, TimestampNs timestamp,
    const LinearizationVersion& version) {
  if (priors.size() != slots.size()) {
    throw std::logic_error("initial prior slot count mismatch");
  }
  for (std::size_t i = 0; i < priors.size(); ++i) {
    const FactorGroupId group(0xf000000000000000ULL + i);
    FactorLedgerEntry entry;
    entry.factor_id = FactorId(0xf100000000000000ULL + i);
    entry.group_id = group;
    entry.sensor = SensorType::Prior;
    entry.kind = FactorKind::BoundaryPrior;
    entry.keys.assign(priors[i]->keys().begin(), priors[i]->keys().end());
    entry.time_begin = timestamp;
    entry.time_end = timestamp;
    entry.backend_slot = slots[i];
    entry.lifecycle = FactorLifecycle::Active;
    entry.noise_model_id = "initial_navigation_prior_covariance";
    entry.model_id = "initial_navigation_prior_v1";
    entry.commit_version = version;
    entry.factor = priors[i];
    groups_[group.value()].push_back(std::move(entry));
  }
  ++version_;
}

void FactorLedger::recordPending(const EpochTransaction& tx) {
  appendGroup(&groups_, tx.imu_group, tx);
  for (const auto& group : tx.uwb_groups) appendGroup(&groups_, group, tx);
  appendGroup(&groups_, tx.generic_bridge_group, tx);
  if (tx.dynamics_bridge_group) appendGroup(&groups_, *tx.dynamics_bridge_group, tx);
  ++version_;
}

void FactorLedger::activate(FactorGroupId group,
                            const std::vector<std::size_t>& slots,
                            const LinearizationVersion& version) {
  auto found = groups_.find(group.value());
  if (found == groups_.end()) throw std::logic_error("unknown pending factor group");
  if (found->second.size() != slots.size()) {
    throw std::logic_error("factor slot count does not match factor group");
  }
  for (std::size_t i = 0; i < slots.size(); ++i) {
    auto& entry = found->second[i];
    if (entry.lifecycle != FactorLifecycle::Pending) {
      throw std::logic_error("factor group already finalized");
    }
    entry.backend_slot = slots[i];
    entry.lifecycle = FactorLifecycle::Active;
    entry.commit_version = version;
  }
  ++version_;
}

void FactorLedger::transition(FactorGroupId group, FactorLifecycle lifecycle) {
  auto found = groups_.find(group.value());
  if (found == groups_.end()) throw std::logic_error("unknown factor group");
  for (auto& entry : found->second) {
    entry.lifecycle = lifecycle;
    if (lifecycle != FactorLifecycle::Active) entry.backend_slot.reset();
  }
  ++version_;
}

void FactorLedger::markSlotsAbsent(const gtsam::NonlinearFactorGraph& graph,
                                   std::size_t oldest_retained_epoch) {
  bool changed = false;
  for (auto& pair : groups_) {
    for (auto& entry : pair.second) {
      if (entry.lifecycle != FactorLifecycle::Active || !entry.backend_slot) continue;
      const std::size_t slot = *entry.backend_slot;
      const bool same = slot < graph.size() && graph[slot] &&
                        graph[slot].get() == entry.factor.get();
      if (!same) {
        entry.lifecycle = entry.epoch_end < oldest_retained_epoch
                              ? FactorLifecycle::Marginalized
                              : FactorLifecycle::RemovedByFde;
        entry.backend_slot.reset();
        changed = true;
      }
    }
  }
  if (changed) ++version_;
}

std::vector<FactorLedgerEntry> FactorLedger::entries() const {
  std::vector<FactorLedgerEntry> out;
  for (const auto& pair : groups_) {
    out.insert(out.end(), pair.second.begin(), pair.second.end());
  }
  return out;
}

std::vector<FactorLedgerEntry> FactorLedger::activeEntries(
    std::size_t begin, std::size_t end) const {
  std::vector<FactorLedgerEntry> out;
  for (const auto& pair : groups_) {
    for (const auto& entry : pair.second) {
      if (entry.lifecycle == FactorLifecycle::Active &&
          entry.epoch_end >= begin && entry.epoch_begin <= end) {
        out.push_back(entry);
      }
    }
  }
  return out;
}

std::vector<std::size_t> FactorLedger::activeSlots(FactorGroupId group) const {
  std::vector<std::size_t> out;
  const auto found = groups_.find(group.value());
  if (found == groups_.end()) return out;
  for (const auto& entry : found->second) {
    if (entry.lifecycle == FactorLifecycle::Active && entry.backend_slot) {
      out.push_back(*entry.backend_slot);
    }
  }
  return out;
}

bool FactorLedger::hasCompleteActiveProvenance() const {
  std::set<std::size_t> slots;
  for (const auto& pair : groups_) {
    for (const auto& entry : pair.second) {
      if (entry.lifecycle != FactorLifecycle::Active) continue;
      if (!entry.backend_slot || !entry.factor || entry.model_id.empty() ||
          entry.noise_model_id.empty() || entry.keys.empty() ||
          !slots.insert(*entry.backend_slot).second) {
        return false;
      }
    }
  }
  return true;
}

bool FactorLedger::hasCompleteActiveProvenance(
    const gtsam::NonlinearFactorGraph& graph) const {
  if (!hasCompleteActiveProvenance()) return false;
  std::map<std::size_t, const gtsam::NonlinearFactor*> recorded;
  for (const auto& pair : groups_) {
    for (const auto& entry : pair.second) {
      if (entry.lifecycle == FactorLifecycle::Active && entry.backend_slot) {
        recorded[*entry.backend_slot] = entry.factor.get();
      }
    }
  }
  for (std::size_t slot = 0; slot < graph.size(); ++slot) {
    if (!graph[slot]) continue;
    // Fixed-lag smoothers synthesize LinearContainerFactor boundary rows;
    // their slot identity is audited separately by BackendUpdateAudit.
    if (dynamic_cast<const gtsam::LinearContainerFactor*>(graph[slot].get())) {
      continue;
    }
    const auto found = recorded.find(slot);
    if (found == recorded.end() || found->second != graph[slot].get()) {
      return false;
    }
  }
  return true;
}

}  // namespace uwb_imu_pl
