#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <fstream>
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
  void writeEvent(TimestampNs timestamp, const std::string& event,
                  const std::string& detail);
  void writeSummary(const std::string& status,
                    const std::string& detail) const;

 private:
  std::string directory_;
  std::ofstream states_;
  std::ofstream residuals_;
  std::ofstream integrity_;
  std::ofstream timing_;
  std::ofstream events_;
  bool write_residuals_ = true;
  bool write_timing_ = true;
};

RunManifest makeRunManifest(const IntegrityConfig& config,
                            const std::string& git_sha, bool git_dirty);

}  // namespace uwb_imu_pl
