#pragma once

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "uifgo/nlos_discovery.h"

namespace a19r01 {

struct Counts {
  size_t calls = 0;
  size_t trials = 0;
  size_t accepted = 0;
  size_t rejected = 0;
  size_t unresolved = 0;
  bool exact = true;
};

inline std::string JsonEscape(const std::string& input) {
  std::ostringstream out;
  for (unsigned char c : input) {
    switch (c) {
      case '\"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          static const char hex[] = "0123456789abcdef";
          out << "\\u00" << hex[c >> 4] << hex[c & 0x0f];
        } else {
          out << static_cast<char>(c);
        }
    }
  }
  return out.str();
}

template <class Writer>
inline void WriteAtomic(const std::string& path, Writer writer) {
  namespace fs = std::filesystem;
  const std::string temporary = path + ".tmp";
  {
    std::ofstream file(temporary);
    if (!file) throw std::runtime_error("OPEN_CHECKPOINT_FAILED:" + path);
    writer(file);
    file.flush();
    if (!file) throw std::runtime_error("WRITE_CHECKPOINT_FAILED:" + path);
  }
  fs::rename(temporary, path);
}

inline size_t WriteStage1Checkpoint(
    const std::string& root,
    const std::vector<uifgo::DiscoveryObservation>& snapshot,
    const uifgo::SupportPartition& partition) {
  namespace fs = std::filesystem;
  fs::create_directories(root + "/stage1");
  WriteAtomic(root + "/stage1/support_snapshot.csv", [&](std::ostream& file) {
    file << "obs_id,tag_id,anchor_id,sensor_time,weight,bias_m,chain_id,active_run_id\n";
    for (const auto& item : snapshot)
      file << item.obs_id << ',' << item.tag_id << ',' << item.anchor_id << ','
           << item.sensor_time << ',' << item.weight << ',' << item.bias_m
           << ',' << item.chain_id << ',' << item.active_run_id << '\n';
  });
  size_t candidate_observations = 0;
  WriteAtomic(root + "/stage1/partition.csv", [&](std::ostream& file) {
    file << "segment_id,ordinal,tag_id,anchor_id,obs_count,start,end,duration,"
            "short_support,merge_mean_m,parent_ids,obs_ids\n";
    for (const auto& segment : partition.segments) {
      candidate_observations += segment.obs_ids.size();
      file << segment.segment_id << ',' << segment.segment_ordinal << ','
           << segment.tag_id << ',' << segment.anchor_id << ','
           << segment.obs_ids.size() << ',' << segment.start_time << ','
           << segment.end_time << ',' << segment.duration << ','
           << segment.short_support_debug << ','
           << segment.merge_snapshot_mean_m << ',';
      for (const auto& parent : segment.parent_segment_ids)
        file << parent << ';';
      file << ',';
      for (auto obs_id : segment.obs_ids) file << obs_id << ';';
      file << '\n';
    }
  });
  WriteAtomic(root + "/stage1/partition_identity.json",
              [&](std::ostream& file) {
    file << "{\"schema\":\"A19_R01_STAGE1_PARTITION_V1\","
            "\"input_plan_hash\":\""
         << JsonEscape(partition.input_plan_hash)
         << "\",\"discovery_context_hash\":\""
         << JsonEscape(partition.discovery_context_hash)
         << "\",\"snapshot_hash\":\""
         << JsonEscape(partition.discovery_snapshot_hash)
         << "\",\"partition_hash\":\""
         << JsonEscape(partition.partition_hash)
         << "\",\"solver_config_hash\":\""
         << JsonEscape(partition.solver_config_hash)
         << "\",\"segments\":" << partition.segments.size()
         << ",\"candidate_observations\":" << candidate_observations
         << "}\n";
  });
  WriteAtomic(root + "/pipeline_status.json", [&](std::ostream& file) {
    file << "{\"schema\":\"A19_R01_PIPELINE_STATUS_V1\","
            "\"status\":\"STAGE1_CHECKPOINTED\","
            "\"Stage1\":\"CONVERGED\",\"Stage2\":\"NOT_RUN\","
            "\"scoring\":\"NOT_RUN\",\"segments\":"
         << partition.segments.size()
         << ",\"candidate_observations\":" << candidate_observations
         << ",\"groups\":null,\"eligible\":null,"
            "\"unavailable\":null}\n";
  });
  return candidate_observations;
}

inline void WriteFailure(const std::string& root, const std::string& reason,
                         const std::string& stage1_status,
                         const std::string& stage2_status,
                         const Counts& stage1, const Counts& stage2) {
  namespace fs = std::filesystem;
  fs::create_directories(root);
  if (stage2_status.rfind("FAILED", 0) == 0) {
    fs::create_directories(root + "/stage2");
    std::ofstream stage(root + "/stage2/status.json");
    if (!stage) throw std::runtime_error("OPEN_STAGE2_STATUS_FAILED");
    stage << "{\"schema\":\"A19_R01_STAGE_STATUS_V1\","
             "\"status\":\"" << JsonEscape(stage2_status)
          << "\",\"reason\":\"" << JsonEscape(reason)
          << "\",\"calls\":" << stage2.calls
          << ",\"trials\":" << stage2.trials
          << ",\"accepted\":" << stage2.accepted
          << ",\"rejected\":" << stage2.rejected
          << ",\"unresolved\":" << stage2.unresolved
          << ",\"counter_state\":\""
          << (stage2.exact ? "COMPLETE" : "CONFIRMED_LOWER_BOUND_CURRENT_OPERATION_UNKNOWN")
          << "\",\"segments\":null,\"groups\":null,"
             "\"eligible\":null,\"unavailable\":null,"
             "\"scoring\":\"NOT_RUN\"}\n";
  }
  std::ofstream pipeline(root + "/pipeline_status.json");
  if (!pipeline) throw std::runtime_error("OPEN_PIPELINE_STATUS_FAILED");
  pipeline << "{\"schema\":\"A19_R01_PIPELINE_STATUS_V1\","
              "\"status\":\"FAILED\",\"reason\":\""
           << JsonEscape(reason) << "\",\"Stage1\":\""
           << JsonEscape(stage1_status) << "\",\"Stage2\":\""
           << JsonEscape(stage2_status)
           << "\",\"scoring\":\"NOT_RUN\","
              "\"stage1_counts\":{\"calls\":" << stage1.calls
           << ",\"trials\":" << stage1.trials
           << ",\"accepted\":" << stage1.accepted
           << ",\"rejected\":" << stage1.rejected
           << ",\"unresolved\":" << stage1.unresolved
           << ",\"counter_state\":\""
           << (stage1.exact ? "COMPLETE" : "CONFIRMED_LOWER_BOUND_CURRENT_OPERATION_UNKNOWN")
           << "\"},\"stage2_counts\":{\"calls\":" << stage2.calls
           << ",\"trials\":" << stage2.trials
           << ",\"accepted\":" << stage2.accepted
           << ",\"rejected\":" << stage2.rejected
           << ",\"unresolved\":" << stage2.unresolved
           << ",\"counter_state\":\""
           << (stage2.exact ? "COMPLETE" : "CONFIRMED_LOWER_BOUND_CURRENT_OPERATION_UNKNOWN")
           << "\"},\"segments\":null,\"groups\":null,"
              "\"eligible\":null,\"unavailable\":null}\n";
}

}  // namespace a19r01
