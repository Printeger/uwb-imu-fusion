#pragma once

#include <chrono>
#include <cstdint>
#include <iosfwd>

namespace uwb_imu_pl {
namespace detail {
enum class NumericalProfilePhase : unsigned {
  FactorGram, FactorGramSvd, DetectionClassification,
  FrozenHypothesisValidation, ProtectionPayloadValidation, RootResponseReuse, RootResponseFallback, ContinuousProofBatches, ContinuousProofLeaves, Count
};
// Process-start opt-in diagnostics. Elapsed work overlaps nested calls and
// worker threads; it is not pipeline wall time or measured thread CPU time.
bool numericalPhaseProfilingEnabled();
void recordNumericalPhase(NumericalProfilePhase phase, std::uint64_t ns);
void writeNumericalPhaseProfile(std::ostream& stream);
class NumericalPhaseScope {
 public:
  explicit NumericalPhaseScope(NumericalProfilePhase phase)
      : phase_(phase), enabled_(numericalPhaseProfilingEnabled()) {
    if (enabled_) start_ = std::chrono::steady_clock::now();
  }
  ~NumericalPhaseScope() { finish(); }
  void finish() {
    if (enabled_) recordNumericalPhase(phase_,
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start_).count());
    enabled_ = false;
  }
 private:
  NumericalProfilePhase phase_;
  bool enabled_;
  std::chrono::steady_clock::time_point start_;
};
}  // namespace detail
}  // namespace uwb_imu_pl
