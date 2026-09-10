#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace uifgo {

// Source-neutral, immutable Stage-2 support description. Oracle and automatic
// providers must populate the same type; downstream refit/scoring must not
// inspect provider-specific inputs.
struct SupportSegment {
  std::string segment_id;
  int tag_id = 0;
  int anchor_id = 0;
  size_t segment_ordinal = 0;
  std::vector<std::uint64_t> obs_ids;
  double start_time = 0.0;
  double end_time = 0.0;
  size_t observation_count = 0;
  double duration = 0.0;
  // Immutable A01 representative computed from the frozen discovery snapshot.
  double merge_snapshot_mean_m = 0.0;
  bool short_support_debug = false;
  std::vector<std::string> parent_segment_ids;
};

struct SupportPartition {
  std::string schema;
  std::string provider;
  std::string source_path;
  std::string hash_algorithm;
  std::string partition_rule_version;
  std::string discovery_context_hash;
  std::string input_plan_hash;
  std::string source_hash;
  std::string config_hash;
  std::string calibration_hash;
  std::string solver_config_hash;
  std::string discovery_snapshot_hash;
  std::string partition_hash;
  std::vector<SupportSegment> segments;
};

// T04 ABI compatibility types. The provider boundary owns these names; all
// new downstream code converts once to SupportPartition.
struct OracleSegment {
  std::string segment_id;
  int tag_id = 0;
  int anchor_id = 0;
  size_t segment_ordinal = 0;
  std::vector<std::uint64_t> obs_ids;
  double start_time = 0.0;
  double end_time = 0.0;
  size_t observation_count = 0;
  double duration = 0.0;
  bool short_support_debug = false;
};

struct OracleSupport {
  std::string schema;
  std::string source_path;
  std::vector<OracleSegment> segments;
};

inline SupportPartition ToSupportPartition(const OracleSupport& oracle) {
  SupportPartition support;
  support.schema = oracle.schema;
  support.provider = "oracle_debug";
  support.source_path = oracle.source_path;
  support.segments.reserve(oracle.segments.size());
  for (const auto& input : oracle.segments) {
    SupportSegment output;
    output.segment_id = input.segment_id;
    output.tag_id = input.tag_id;
    output.anchor_id = input.anchor_id;
    output.segment_ordinal = input.segment_ordinal;
    output.obs_ids = input.obs_ids;
    output.start_time = input.start_time;
    output.end_time = input.end_time;
    output.observation_count = input.observation_count;
    output.duration = input.duration;
    output.merge_snapshot_mean_m = 0.0;
    output.short_support_debug = input.short_support_debug;
    output.parent_segment_ids = {input.segment_id};
    support.segments.push_back(std::move(output));
  }
  return support;
}

}  // namespace uifgo
