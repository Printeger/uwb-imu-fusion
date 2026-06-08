// test/test_trilaterate.cpp — UT-3: Trilateration tests
#include "uifgo/initializer.h"
#include "uifgo/config.h"
#include <gtest/gtest.h>

TEST(Trilaterate, PerfectNoiseless) {
  uifgo::Config cfg;
  cfg.anchors = {
    {101, gtsam::Point3(0, 0, 0), 0.08},
    {102, gtsam::Point3(5, 0, 0), 0.08},
    {103, gtsam::Point3(0, 5, 0), 0.08},
    {104, gtsam::Point3(0, 0, 3), 0.08},
  };

  gtsam::Point3 true_pos(2, 2, 1);
  std::vector<uifgo::UwbRange> ranges;
  for (const auto& a : cfg.anchors) {
    double d = (gtsam::Vector3(true_pos) - gtsam::Vector3(a.pos)).norm();
    ranges.push_back({a.id, d, -50.0, -51.0});
  }

  uifgo::Initializer init(cfg);
  gtsam::Point3 result;
  EXPECT_TRUE(init.Trilaterate(ranges, cfg.anchors, &result));
  EXPECT_NEAR(result.x(), true_pos.x(), 1e-4);
  EXPECT_NEAR(result.y(), true_pos.y(), 1e-4);
  EXPECT_NEAR(result.z(), true_pos.z(), 1e-4);
}

TEST(Trilaterate, Noisy) {
  uifgo::Config cfg;
  cfg.anchors = {
    {101, gtsam::Point3(0, 0, 0), 0.08},
    {102, gtsam::Point3(5, 0, 0), 0.08},
    {103, gtsam::Point3(0, 5, 0), 0.08},
    {104, gtsam::Point3(0, 0, 3), 0.08},
  };

  gtsam::Point3 true_pos(2, 2, 1);
  // Add noise ~0.1m
  std::vector<uifgo::UwbRange> ranges;
  for (const auto& a : cfg.anchors) {
    double d = (gtsam::Vector3(true_pos) - gtsam::Vector3(a.pos)).norm();
    d += 0.05;  // small bias
    ranges.push_back({a.id, d, -50.0, -51.0});
  }

  uifgo::Initializer init(cfg);
  gtsam::Point3 result;
  EXPECT_TRUE(init.Trilaterate(ranges, cfg.anchors, &result));
  double err = (gtsam::Vector3(result) - gtsam::Vector3(true_pos)).norm();
  EXPECT_LT(err, 0.5);  // rough bound
}

TEST(Trilaterate, InsufficientAnchors) {
  uifgo::Config cfg;
  cfg.anchors = {
    {101, gtsam::Point3(0, 0, 0), 0.08},
    {102, gtsam::Point3(5, 0, 0), 0.08},
  };

  std::vector<uifgo::UwbRange> ranges;
  for (const auto& a : cfg.anchors) {
    ranges.push_back({a.id, 3.0, -50.0, -51.0});
  }

  uifgo::Initializer init(cfg);
  gtsam::Point3 result;
  EXPECT_FALSE(init.Trilaterate(ranges, cfg.anchors, &result));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
