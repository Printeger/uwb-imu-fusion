#include "uwb_imu_pl/common/deterministic_event.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

TEST(DeterministicEvent, OrdersTimestampThenImuBeforeUwbThenSequence) {
  using uwb_imu_pl::EventKind;
  using uwb_imu_pl::EventOrderKey;
  std::vector<EventOrderKey> events = {
      {uwb_imu_pl::TimestampNs(20), EventKind::Uwb, 1},
      {uwb_imu_pl::TimestampNs(10), EventKind::Uwb, 2},
      {uwb_imu_pl::TimestampNs(10), EventKind::Imu, 9},
      {uwb_imu_pl::TimestampNs(10), EventKind::Imu, 3}};
  std::sort(events.begin(), events.end(), uwb_imu_pl::eventOrderLess);
  ASSERT_EQ(events.size(), 4u);
  EXPECT_EQ(events[0].sequence, 3u);
  EXPECT_EQ(events[1].sequence, 9u);
  EXPECT_EQ(events[2].sequence, 2u);
  EXPECT_EQ(events[3].timestamp.value(), 20);
}

TEST(DeterministicEvent, TenSecondSyntheticOrderingIsRepeatable) {
  auto make = [] {
    std::vector<uwb_imu_pl::EventOrderKey> events;
    std::uint64_t sequence = 1;
    for (std::int64_t ms = 0; ms <= 10000; ms += 5) {
      if (ms % 50 == 0) {
        events.push_back({uwb_imu_pl::TimestampNs(ms * 1000000),
                          uwb_imu_pl::EventKind::Uwb, sequence++});
      }
      events.push_back({uwb_imu_pl::TimestampNs(ms * 1000000),
                        uwb_imu_pl::EventKind::Imu, sequence++});
    }
    std::reverse(events.begin(), events.end());
    std::sort(events.begin(), events.end(), uwb_imu_pl::eventOrderLess);
    return events;
  };
  const auto left = make();
  const auto right = make();
  ASSERT_EQ(left.size(), right.size());
  for (std::size_t index = 0; index < left.size(); ++index) {
    EXPECT_EQ(left[index].timestamp, right[index].timestamp);
    EXPECT_EQ(left[index].kind, right[index].kind);
    EXPECT_EQ(left[index].sequence, right[index].sequence);
  }
}
