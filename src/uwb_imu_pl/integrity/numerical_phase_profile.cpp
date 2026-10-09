#include "numerical_phase_profile.hpp"

#include <array>
#include <atomic>
#include <cstdlib>
#include <ostream>

namespace uwb_imu_pl {
namespace detail {
namespace {
constexpr unsigned count = static_cast<unsigned>(NumericalProfilePhase::Count);
std::array<std::atomic<std::uint64_t>, count> calls{};
std::array<std::atomic<std::uint64_t>, count> elapsed{};
constexpr const char* names[] = {"factor_gram", "factor_gram_svd",
    "detection_classification", "frozen_hypothesis_validation", "pl_payload_validation", "root_response_reuse", "root_response_fallback"};
}
bool numericalPhaseProfilingEnabled() {
  static const bool enabled = std::getenv("UWB_IMU_PL_PROFILE_NUMERICAL_PHASES") != nullptr;
  return enabled;
}
void recordNumericalPhase(NumericalProfilePhase phase, std::uint64_t ns) {
  const auto index = static_cast<unsigned>(phase);
  calls[index].fetch_add(1, std::memory_order_relaxed);
  elapsed[index].fetch_add(ns, std::memory_order_relaxed);
}
void writeNumericalPhaseProfile(std::ostream& stream) {
  if (!numericalPhaseProfilingEnabled()) return;
  for (unsigned i = 0; i < count; ++i)
    stream << "numerical_phase_profile phase=" << names[i]
           << " calls=" << calls[i].load()
           << " inclusive_elapsed_work_ns=" << elapsed[i].load()
           << " audit_only=1\n";
}
}  // namespace detail
}  // namespace uwb_imu_pl
