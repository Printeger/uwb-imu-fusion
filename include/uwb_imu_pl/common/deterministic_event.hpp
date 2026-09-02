#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <cstdint>
#include <tuple>

namespace uwb_imu_pl {

// Stable ordering shared by live ROS ingestion and deterministic replay.
// IMU is deliberately ordered before UWB at an identical timestamp.
enum class EventKind : std::uint8_t { Imu = 0, Uwb = 1 };

struct EventOrderKey {
  TimestampNs timestamp;
  EventKind kind = EventKind::Imu;
  std::uint64_t sequence = 0;
};

inline bool eventOrderLess(const EventOrderKey& left,
                           const EventOrderKey& right) {
  return std::tie(left.timestamp, left.kind, left.sequence) <
         std::tie(right.timestamp, right.kind, right.sequence);
}

template <typename Event>
struct DeterministicEventLess {
  bool operator()(const Event& left, const Event& right) const {
    return eventOrderLess(left.orderKey(), right.orderKey());
  }
};

}  // namespace uwb_imu_pl
