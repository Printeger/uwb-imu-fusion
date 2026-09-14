// test/test_trilaterate.cpp — UT-3: Trilateration tests
#include "uifgo/initializer.h"
#include "uifgo/config.h"
#include <gtest/gtest.h>

#include <cmath>

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

TEST(Trilaterate, AutomaticCoplanar2p5dChoosesWorldZeroSide) {
  uifgo::Config cfg;
  cfg.anchors = {
      {101, gtsam::Point3(-2, -2, 2), 0.08},
      {102, gtsam::Point3(2, -2, 2), 0.08},
      {103, gtsam::Point3(2, 2, 2), 0.08},
      {104, gtsam::Point3(-2, 2, 2), 0.08},
  };
  const gtsam::Point3 true_pos(0.4, -0.3, 0.5);
  std::vector<uifgo::UwbRange> ranges;
  for (const auto& anchor : cfg.anchors) {
    ranges.push_back(
        {anchor.id,
         (gtsam::Vector3(true_pos) - gtsam::Vector3(anchor.pos)).norm(),
         -50.0, -51.0});
  }

  uifgo::Initializer init(cfg);
  gtsam::Point3 result;
  ASSERT_TRUE(init.Trilaterate(ranges, cfg.anchors, &result));
  EXPECT_NEAR(result.x(), true_pos.x(), 1e-6);
  EXPECT_NEAR(result.y(), true_pos.y(), 1e-6);
  EXPECT_NEAR(result.z(), true_pos.z(), 1e-6);
}

TEST(Trilaterate, NonConvergedGeometryDoesNotPublishIterate) {
  uifgo::Config cfg;
  cfg.anchors = {
      {101, gtsam::Point3(0, 0, 1), 0.08},
      {102, gtsam::Point3(2, 0, 1), 0.08},
      {103, gtsam::Point3(4, 0, 1), 0.08},
  };
  std::vector<uifgo::UwbRange> ranges = {
      {101, 2.0, -50.0, -51.0},
      {102, 1.0, -50.0, -51.0},
      {103, 2.0, -50.0, -51.0},
  };

  uifgo::Initializer init(cfg);
  gtsam::Point3 result(91.0, 92.0, 93.0);
  EXPECT_FALSE(init.Trilaterate(ranges, cfg.anchors, &result));
  EXPECT_DOUBLE_EQ(result.x(), 91.0);
  EXPECT_DOUBLE_EQ(result.y(), 92.0);
  EXPECT_DOUBLE_EQ(result.z(), 93.0);
}

TEST(InitializerStaticDetection, UsesConfiguredAccelerometerNoise) {
  std::vector<uifgo::ImuSample> imu(4);
  const double norms[] = {9.66, 9.96, 9.66, 9.96};
  for (size_t i = 0; i < imu.size(); ++i) {
    imu[i].t = 0.01 * static_cast<double>(i);
    imu[i].acc = Eigen::Vector3d(0.0, 0.0, norms[i]);
    imu[i].gyro = Eigen::Vector3d::Zero();
  }

  uifgo::Config permissive;
  permissive.sigma_a = 0.2;  // variance threshold 0.04
  uifgo::Initializer permissive_init(permissive);
  EXPECT_TRUE(permissive_init.DetectStatic(imu, 0, imu.size()));

  uifgo::Config strict;
  strict.sigma_a = 0.1;  // variance threshold 0.01
  uifgo::Initializer strict_init(strict);
  EXPECT_FALSE(strict_init.DetectStatic(imu, 0, imu.size()));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
