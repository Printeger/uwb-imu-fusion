#include "uwb_imu_pl/common/types.hpp"

#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {

TimestampNs TimestampNs::fromSeconds(double seconds) {
  if (!std::isfinite(seconds)) {
    throw std::invalid_argument("timestamp seconds must be finite");
  }
  constexpr double kNsPerSecond = 1e9;
  const double ns = std::round(seconds * kNsPerSecond);
  if (ns < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      ns > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    throw std::out_of_range("timestamp exceeds int64 nanoseconds");
  }
  return TimestampNs(static_cast<std::int64_t>(ns));
}

double TimestampNs::seconds() const { return static_cast<double>(value_) * 1e-9; }

const char* toString(Availability value) {
  switch (value) {
    case Availability::Available: return "AVAILABLE";
    case Availability::Alert: return "ALERT";
    case Availability::Unavailable: return "UNAVAILABLE";
  }
  return "UNAVAILABLE";
}

const char* toString(IntegrityLabel value) {
  switch (value) {
    case IntegrityLabel::FormalLocalCurrentFaultOnly:
      return "FORMAL_LOCAL_CURRENT_FAULT_ONLY";
    case IntegrityLabel::ImplementedUnverified: return "IMPLEMENTED_UNVERIFIED";
    case IntegrityLabel::HeuristicDebug: return "HEURISTIC_DEBUG";
  }
  return "IMPLEMENTED_UNVERIFIED";
}

}  // namespace uwb_imu_pl
