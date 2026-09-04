#include "uwb_imu_pl/integrity/health_manager.hpp"

#include <stdexcept>

namespace uwb_imu_pl {

void HealthManager::registerSource(const std::string& id, SensorType type) {
  if (id.empty()) throw std::invalid_argument("health source ID must not be empty");
  auto inserted = records_.emplace(id, HealthRecord{id, type});
  if (!inserted.second && inserted.first->second.source_type != type) {
    throw std::logic_error("health source ID type changed");
  }
}

HealthTransition HealthManager::transition(HealthRecord* record,
                                           HealthState next,
                                           const std::string& trigger) {
  HealthTransition out{record->source_id, record->source_type,
                       record->state, next, trigger};
  record->state = next;
  return out;
}

HealthTransition HealthManager::observeEvidence(const std::string& id,
                                                bool suspicious) {
  auto found = records_.find(id);
  if (found == records_.end()) throw std::out_of_range("unknown health source");
  auto& record = found->second;
  if (record.state != HealthState::Healthy && record.state != HealthState::Suspect) {
    return transition(&record, record.state, "evidence ignored while quarantined/recovering");
  }
  if (suspicious) {
    ++record.suspicion_count;
    if (record.suspicion_count >= config_.suspect_evidence_count) {
      return transition(&record, HealthState::Suspect, "fault-mode evidence");
    }
  } else {
    record.suspicion_count = 0;
    if (record.state == HealthState::Suspect) {
      return transition(&record, HealthState::Healthy, "evidence cleared");
    }
  }
  return transition(&record, record.state, "no state change");
}

HealthTransition HealthManager::quarantine(const std::string& id,
                                           const std::string& trigger) {
  auto found = records_.find(id);
  if (found == records_.end()) throw std::out_of_range("unknown health source");
  found->second.shadow_pass_count = 0;
  found->second.recovery_pass_count = 0;
  return transition(&found->second, HealthState::Quarantined, trigger);
}

HealthTransition HealthManager::observeShadowRecovery(const std::string& id,
                                                       bool passed) {
  auto found = records_.find(id);
  if (found == records_.end()) throw std::out_of_range("unknown health source");
  auto& record = found->second;
  if (!passed) {
    record.shadow_pass_count = 0;
    record.recovery_pass_count = 0;
    if (record.state == HealthState::RecoveryTest) {
      return transition(&record, HealthState::Quarantined, "recovery test failed");
    }
    return transition(&record, record.state, "shadow residual failed");
  }
  if (record.state == HealthState::Quarantined) {
    ++record.shadow_pass_count;
    if (config_.allow_auto_recovery &&
        record.shadow_pass_count >= config_.recovery_shadow_passes) {
      return transition(&record, HealthState::RecoveryTest,
                        "shadow recovery precondition met");
    }
  } else if (record.state == HealthState::RecoveryTest) {
    ++record.recovery_pass_count;
    if (record.recovery_pass_count >= config_.recovery_test_passes) {
      record.suspicion_count = 0;
      record.shadow_pass_count = 0;
      record.recovery_pass_count = 0;
      return transition(&record, HealthState::Healthy,
                        "consecutive recovery tests passed");
    }
  }
  return transition(&record, record.state, "recovery counter advanced");
}

HealthTransition HealthManager::fail(const std::string& id,
                                     const std::string& trigger) {
  auto found = records_.find(id);
  if (found == records_.end()) throw std::out_of_range("unknown health source");
  return transition(&found->second, HealthState::Failed, trigger);
}

HealthState HealthManager::state(const std::string& id) const {
  const auto found = records_.find(id);
  if (found == records_.end()) throw std::out_of_range("unknown health source");
  return found->second.state;
}

bool HealthManager::allowedInFormalEstimator(const std::string& id) const {
  const HealthState value = state(id);
  return value == HealthState::Healthy || value == HealthState::Suspect;
}

HealthSnapshot HealthManager::snapshot() const {
  HealthSnapshot out;
  for (const auto& pair : records_) out.push_back(pair.second);
  return out;
}

}  // namespace uwb_imu_pl
