#pragma once

#include "uwb_imu_pl/common/integrity_ids.hpp"

#include <map>
#include <string>
#include <vector>

namespace uwb_imu_pl {

struct HealthConfigV2 {
  std::uint32_t suspect_evidence_count = 2;
  bool quarantine_after_exclusion = true;
  std::uint32_t recovery_shadow_passes = 20;
  std::uint32_t recovery_test_passes = 10;
  bool allow_auto_recovery = true;
};

struct HealthRecord {
  std::string source_id;
  SensorType source_type = SensorType::Unknown;
  HealthState state = HealthState::Healthy;
  std::uint32_t suspicion_count = 0;
  std::uint32_t shadow_pass_count = 0;
  std::uint32_t recovery_pass_count = 0;
};

struct HealthTransition {
  std::string source_id;
  SensorType source_type = SensorType::Unknown;
  HealthState previous = HealthState::Healthy;
  HealthState current = HealthState::Healthy;
  std::string trigger;
};

using HealthSnapshot = std::vector<HealthRecord>;

class HealthManager {
 public:
  explicit HealthManager(HealthConfigV2 config = {}) : config_(config) {}
  void registerSource(const std::string& id, SensorType type);
  HealthTransition observeEvidence(const std::string& id, bool suspicious);
  HealthTransition quarantine(const std::string& id,
                              const std::string& trigger);
  HealthTransition observeShadowRecovery(const std::string& id, bool passed);
  HealthTransition fail(const std::string& id, const std::string& trigger);
  HealthState state(const std::string& id) const;
  bool allowedInFormalEstimator(const std::string& id) const;
  HealthSnapshot snapshot() const;

 private:
  HealthTransition transition(HealthRecord* record, HealthState state,
                              const std::string& trigger);
  HealthConfigV2 config_;
  std::map<std::string, HealthRecord> records_;
};

}  // namespace uwb_imu_pl
