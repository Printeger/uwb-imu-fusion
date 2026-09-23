#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

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
  void flush();

 private:
  std::string directory_;
  std::ofstream states_;
  std::ofstream residuals_;
  std::ofstream integrity_;
  std::ofstream timing_;
  std::ofstream events_;
  std::ofstream ground_truth_;
  std::ofstream fault_truth_;
  std::ofstream transactions_;
  std::ofstream hypotheses_;
  std::ofstream candidates_;
  std::ofstream factor_ledger_;
  std::ofstream health_;
  std::ofstream bridge_;
  std::ofstream attempts_;
  std::ofstream timing_links_;
  std::uint64_t timing_row_ = 0;
  std::ofstream diagnostic_stages_;
  std::ofstream diagnostic_candidates_;
  std::ofstream diagnostic_coverage_;
  std::ofstream diagnostic_steps_;
  std::ofstream diagnostic_identity_;
  std::ofstream diagnostic_square_root_;
  // C1-c diagnostics v15: the condensed boundary's fault-preserving history
  // summary and the pooled detector terms it contributes.
  std::ofstream diagnostic_history_summary_;
  bool write_residuals_ = true;
  bool write_timing_ = true;
  std::uint64_t next_event_sequence_ = 1;
  mutable std::mutex mutex_;
};

// Centralized optional facade used by interactive ROS runs.  A disabled
// session never constructs RunLogger and every write is a true no-op.
class RunLoggingSession {
 public:
  RunLoggingSession() = default;
  RunLoggingSession(const RunLoggingSession&) = delete;
  RunLoggingSession& operator=(const RunLoggingSession&) = delete;

  void enable(const std::string& output_directory,
              bool write_residuals, bool write_timing);
  bool enabled() const { return static_cast<bool>(logger_); }
  const std::string& directory() const { return directory_; }

  void writeResolvedConfig(const std::string& yaml) const;
  void writeManifest(const RunManifest& manifest) const;
  void writeState(const NavigationState& state);
  void writeResidual(TimestampNs timestamp, FactorId factor, AnchorId anchor,
                     RowRole role, double raw, double whitened);
  void writeIntegrity(const IntegrityOutput& output);
  void writeTiming(const TimingRecord& record);
  void writeEvent(TimestampNs timestamp, const std::string& event,
                  const std::string& detail);
  void writeEvent(TimestampNs timestamp, std::uint64_t sequence,
                  const std::string& event, const std::string& detail);
  void writeGroundTruth(const GroundTruthRecord& record);
  void writeFaultTruth(const FaultTruthRecord& record);
  void writeSummary(const RunSummary& summary) const;
  void flush();

 private:
  std::string directory_;
  std::unique_ptr<RunLogger> logger_;
};

// Replace or append resolved roslaunch-style key:=value arguments.  This is
// used immediately before manifest creation so the saved command identifies
// what the node actually ran, rather than unresolved launch defaults.
std::string bindExecutionCommandArguments(
    const std::string& command,
    const std::vector<std::pair<std::string, std::string>>& arguments);

RunManifest makeRunManifest(const IntegrityConfig& config,
                            const std::string& git_sha, bool git_dirty,
                            const std::string& execution_command = "unknown");

}  // namespace uwb_imu_pl
