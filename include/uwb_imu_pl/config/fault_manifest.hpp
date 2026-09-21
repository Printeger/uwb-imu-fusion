#pragma once

#include <string>
#include <vector>

namespace uwb_imu_pl {

// Event entry of config/integrity_fault_manifest.yaml (roadmap section 4.3).
// The loader is strict: unknown keys, unsupported model types or families, and
// inconsistent projection eligibility are rejected instead of ignored.
struct FaultManifestEvent {
  std::string source_id;
  std::string common_cause_group;
  std::string event_id;
  std::string model_type;
  std::string parameter_units;
  std::string parameter_sharing;
  std::string onset_domain;
  std::string end_condition;
  std::string physical_time_support;
  std::string amplitude_model;
  std::string prior_bound_source;
  std::string prior_time_basis;
  std::string evidence_status;
  std::vector<std::string> included_hypothesis_families;
  std::vector<std::string> omitted_event_set;
  std::string history_retention_or_reset_rule;
  std::vector<std::string> allowed_actions;
  std::string projection_eligibility;
};

struct FaultManifestFamily {
  std::string family_id;
  std::string scope;  // "single" | "pair"
  std::vector<std::string> members;
};

struct FaultManifest {
  int schema_version = 0;
  std::string manifest_id;
  std::string protected_quantity;
  std::string position_reference;
  int max_fault_order = 1;
  std::vector<std::string> enabled_single_families;
  std::vector<std::string> enabled_pair_families;
  std::vector<std::string> unsupported_families;
  std::vector<FaultManifestEvent> events;
  std::vector<FaultManifestFamily> families;
  std::vector<std::string> warnings;
  std::string resolved_yaml;
  std::string digest;
};

class FaultManifestLoader {
 public:
  // Throws std::runtime_error on parse/schema/capability violations.
  static FaultManifest load(const std::string& path);
  // Family/model capability validation plus structural rules that keep
  // projection eligibility independent from allowed_actions.
  static void validateAgainstCapabilities(const FaultManifest& manifest);
  static std::string serializeResolvedManifest(const FaultManifest& manifest);
  static bool modelTypeImplemented(const std::string& model_type);
};

}  // namespace uwb_imu_pl
