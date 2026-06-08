// test/test_outlier.cpp — UT-2: OutlierFilter tests
#include "uifgo/outlier_filter.h"
#include "uifgo/config.h"
#include <gtest/gtest.h>

TEST(OutlierFilter, NLOSRejection) {
  uifgo::Config cfg;
  cfg.nlos_rssi_diff = 6.0;
  uifgo::OutlierFilter filter(cfg);

  uifgo::UwbFrame frame;
  frame.t = 0.0;
  // Good measurement
  uifgo::UwbRange r_good{1, 5.0, -70.0, -75.0};  // fp=-70, rx=-75, diff=5 < 6
  // NLOS measurement
  uifgo::UwbRange r_nlos{2, 5.0, -80.0, -70.0};  // fp=-80, rx=-70, diff=10 > 6
  frame.ranges = {r_good, r_nlos};

  auto retained = filter.PreFilter(frame);
  EXPECT_EQ(retained.size(), 1u);
  EXPECT_EQ(retained[0].anchor_id, 1);
}

TEST(OutlierFilter, RangeBounds) {
  uifgo::Config cfg;
  cfg.min_range = 0.3;
  cfg.max_range = 100.0;
  uifgo::OutlierFilter filter(cfg);

  uifgo::UwbFrame frame;
  frame.t = 0.0;
  uifgo::UwbRange r_low{1, 0.1, -60.0, -61.0};
  uifgo::UwbRange r_ok{2, 5.0, -60.0, -61.0};
  uifgo::UwbRange r_high{3, 200.0, -60.0, -61.0};
  frame.ranges = {r_low, r_ok, r_high};

  auto retained = filter.PreFilter(frame);
  EXPECT_EQ(retained.size(), 1u);
  EXPECT_EQ(retained[0].anchor_id, 2);
}

TEST(OutlierFilter, ConsistencyCheck) {
  uifgo::Config cfg;
  cfg.dist_consistency_thresh = 1.0;
  uifgo::OutlierFilter filter(cfg);

  gtsam::Point3 tag_pred(0, 0, 0);
  gtsam::Point3 anchor(3, 4, 0);  // distance = 5.0

  uifgo::UwbRange r{1, 5.2, 0, 0};  // residual = 0.2 < 1.0
  EXPECT_TRUE(filter.ConsistencyCheck(r, tag_pred, anchor, true));

  uifgo::UwbRange r_bad{1, 7.0, 0, 0};  // residual = 2.0 > 1.0
  EXPECT_FALSE(filter.ConsistencyCheck(r_bad, tag_pred, anchor, true));

  // Not warmed up: always ok
  EXPECT_TRUE(filter.ConsistencyCheck(r_bad, tag_pred, anchor, false));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
