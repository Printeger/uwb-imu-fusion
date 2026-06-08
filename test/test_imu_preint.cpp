// test/test_imu_preint.cpp — UT-4: IMU preintegration tests
#include "uifgo/imu_preint.h"
#include "uifgo/config.h"
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>
#include <gtest/gtest.h>
#include <cmath>

// Helper: generate constant-acceleration IMU samples
static std::vector<uifgo::ImuSample> GenImu(
    double t0, double t1, double freq,
    const gtsam::Vector3& acc_body,
    const gtsam::Vector3& gyro_body) {
  std::vector<uifgo::ImuSample> imu;
  double dt = 1.0 / freq;
  for (double t = t0; t <= t1 + 0.5*dt; t += dt) {
    imu.push_back({t, acc_body, gyro_body});
  }
  return imu;
}

TEST(ImuPreintegrator, StaticZeroVelocity) {
  uifgo::Config cfg;
  // Use zero gravity for this test — focus on integration, not gravity compensation
  gtsam::Vector3 g(0, 0, 0);
  uifgo::ImuPreintegrator pim(cfg, g);

  gtsam::imuBias::ConstantBias bias;
  pim.Reset(bias);

  // Static: no acceleration, no rotation
  gtsam::Vector3 acc_body(0, 0, 0);
  gtsam::Vector3 gyro(0, 0, 0);
  double dt = 0.01;
  for (int i = 0; i < 100; ++i) {
    pim.Integrate(acc_body, gyro, dt);
  }

  gtsam::Pose3 T0;
  gtsam::Vector3 v0(0, 0, 0);
  auto pred = pim.Pim().predict(gtsam::NavState(T0, v0), bias);

  // After 1s static: velocity should be near zero, position near zero
  EXPECT_NEAR(pred.pose().translation().x(), 0.0, 0.01);
  EXPECT_NEAR(pred.pose().translation().y(), 0.0, 0.01);
  EXPECT_NEAR(pred.pose().translation().z(), 0.0, 0.01);
  EXPECT_NEAR(pred.velocity().norm(), 0.0, 0.01);
}

TEST(ImuPreintegrator, ForwardAcceleration) {
  uifgo::Config cfg;
  gtsam::Vector3 g(0, 0, 0);  // zero gravity for simple test
  uifgo::ImuPreintegrator pim(cfg, g);

  gtsam::imuBias::ConstantBias bias;
  pim.Reset(bias);

  // Accelerate at 1 m/s^2 in +X, no rotation
  gtsam::Vector3 acc_body(1.0, 0, 0);
  gtsam::Vector3 gyro(0, 0, 0);
  double dt = 0.01;
  for (int i = 0; i < 200; ++i) {  // 2 seconds
    pim.Integrate(acc_body, gyro, dt);
  }

  gtsam::Pose3 T0;
  gtsam::Vector3 v0(0, 0, 0);
  auto pred = pim.Pim().predict(gtsam::NavState(T0, v0), bias);

  // After 2s at 1 m/s^2: vx = 2.0, px = 0.5*a*t^2 = 2.0
  EXPECT_NEAR(pred.velocity().x(), 2.0, 0.05);
  EXPECT_NEAR(pred.pose().translation().x(), 2.0, 0.05);
}

TEST(ImuSync, InterpolateImu) {
  std::vector<uifgo::ImuSample> imu = {
    {0.0, gtsam::Vector3(0,0,0), gtsam::Vector3(0,0,0)},
    {1.0, gtsam::Vector3(1,0,0), gtsam::Vector3(0.1,0,0)},
  };

  auto s = uifgo::InterpolateImu(imu, 0.5);
  EXPECT_DOUBLE_EQ(s.t, 0.5);
  EXPECT_NEAR(s.acc.x(), 0.5, 1e-12);
  EXPECT_NEAR(s.gyro.x(), 0.05, 1e-12);

  // Endpoint clamping
  auto s0 = uifgo::InterpolateImu(imu, -0.1);
  EXPECT_DOUBLE_EQ(s0.acc.x(), 0.0);
  auto s1 = uifgo::InterpolateImu(imu, 2.0);
  EXPECT_DOUBLE_EQ(s1.acc.x(), 1.0);
}

TEST(ImuSync, IntegrateBetween) {
  uifgo::Config cfg;
  gtsam::Vector3 g(0, 0, 0);
  uifgo::ImuPreintegrator pim(cfg, g);

  auto imu = GenImu(0.0, 1.0, 100.0,
      gtsam::Vector3(2.0, 0, 0), gtsam::Vector3(0, 0, 0));  // 2 m/s^2 in X
  gtsam::imuBias::ConstantBias bias;
  pim.Reset(bias);

  // Integrate from 0.2 to 0.8 (0.6 seconds)
  size_t next = uifgo::IntegrateBetween(imu, 0, 0.2, 0.8, &pim);
  EXPECT_GT(next, 0u);

  gtsam::Pose3 T0;
  gtsam::Vector3 v0(0, 0, 0);
  auto pred = pim.Pim().predict(gtsam::NavState(T0, v0), bias);
  // v = a*t = 2*0.6 = 1.2, p = 0.5*2*0.36 = 0.36
  EXPECT_NEAR(pred.velocity().x(), 1.2, 0.05);
  EXPECT_NEAR(pred.pose().translation().x(), 0.36, 0.05);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
