#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <fstream>
#include <mutex>
#include <string>

namespace uwb_imu_pl {

struct IntegrityConfig;

class RunLogger {
 public:
  explicit RunLogger(const std::string& output_directory,
                     bool write_residuals = true, bool write_timing = true);
  void writeResolvedConfig(const std::string& yaml) const;
  void writeManifest(const RunManifest& manifest) const;
  void writeState(const NavigationState& state);
  void writeResidual(TimestampNs timestamp, FactorId factor, AnchorId anchor,
                     RowRole role, double raw, double whitened);
  void writeIntegrity(const IntegrityOutput& output);
  void writeTiming(TimestampNs timestamp, const std::string& stage,
                   double wall_ms, bool success);
  void writeTiming(const TimingRecord& record);
  void writeEvent(TimestampNs timestamp, const std::string& event,
                  const std::string& detail);
  void writeEvent(TimestampNs timestamp, std::uint64_t sequence,
                  const std::string& event, const std::string& detail);
  void writeGroundTruth(const GroundTruthRecord& record);
  void writeFaultTruth(const FaultTruthRecord& record);
  void writeSummary(const std::string& status,
                    const std::string& detail) const;
  void writeSummary(const RunSummary& summary) const;

 private:
  std::string directory_;
  std::ofstream states_;
  std::ofstream residuals_;
  std::ofstream integrity_;
  std::ofstream timing_;
  std::ofstream events_;
  std::ofstream ground_truth_;
  std::ofstream fault_truth_;
  bool write_residuals_ = true;
  bool write_timing_ = true;
  std::uint64_t next_event_sequence_ = 1;
  mutable std::mutex mutex_;
};

RunManifest makeRunManifest(const IntegrityConfig& config,
                            const std::string& git_sha, bool git_dirty);

}  // namespace uwb_imu_pl
