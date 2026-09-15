#pragma once
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include <string>

namespace uwb_imu_pl {
struct FrozenCandidateReplay {
  std::uint64_t input_attempt_id = 0;
  TimestampNs input_timestamp;
  std::uint64_t transaction_id = 0;
  LinearizedIntegrityWindow window;
  std::vector<ExclusionAction> actions;
  RankUpdateConfig config;
};
// Versioned, lossless binary diagnostic artifact. Never enabled by default.
void writeCandidateReplay(const std::string& path, const FrozenCandidateReplay& replay);
FrozenCandidateReplay readCandidateReplay(const std::string& path);
}  // namespace uwb_imu_pl
