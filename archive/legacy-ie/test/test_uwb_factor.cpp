// test/test_uwb_factor.cpp — UT-5: UWB factor basic tests
#include "uifgo/uwb_factor.h"
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtest/gtest.h>
#include <cmath>

using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::L;
using symbol_shorthand::A;
using symbol_shorthand::Z;

TEST(UwbFactor, AllCalibOff_OnlyPoseKey) {
  Point3 anchor(5.0, 4.0, 1.0);
  Point3 lever_init(0.10, 0.0, -0.05);
  double z = 4.0, sigma = 0.1;
  auto f = uifgo::MakeUwbFactor(X(0), L(0), A(0), Z(0),
      anchor, lever_init, z, sigma, false, false, false);
  EXPECT_EQ(f->keys().size(), 1u);
  EXPECT_EQ(f->keys()[0], X(0));
}

TEST(UwbFactor, AllCalibOn_FourKeys) {
  Point3 anchor(5.0, 4.0, 1.0);
  Point3 lever_init(0.10, 0.0, -0.05);
  double z = 4.0, sigma = 0.1;
  auto f = uifgo::MakeUwbFactor(X(0), L(0), A(0), Z(0),
      anchor, lever_init, z, sigma, true, true, true);
  EXPECT_EQ(f->keys().size(), 4u);
}

TEST(UwbFactor, ResidualValue) {
  Point3 anchor(5.0, 4.0, 1.0);
  Point3 lever_init(0.10, 0.0, -0.05);
  Pose3 T(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1.0, 2.0, 0.5));
  double sigma = 0.1;
  Point3 antenna = T.transformFrom(lever_init);
  double rho = (antenna - anchor).norm();
  double z = rho + 0.05;
  auto f = uifgo::MakeUwbFactor(X(0), L(0), A(0), Z(0),
      anchor, lever_init, z, sigma, false, false, false);
  Values v; v.insert(X(0), T);
  double err = f->error(v);
  double expected = 0.5 * std::pow((rho - z) / sigma, 2);
  EXPECT_NEAR(err, expected, 1e-6);
}

TEST(UwbFactor, WithBiasResidual) {
  Point3 anchor(5.0, 4.0, 1.0);
  Point3 lever_init(0.10, 0.0, -0.05);
  Pose3 T(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(1.0, 2.0, 0.5));
  double sigma = 0.1, beta = 0.04;
  Point3 antenna = T.transformFrom(lever_init);
  double rho = (antenna - anchor).norm();
  double z = rho + beta + 0.1;
  auto f = uifgo::MakeUwbFactor(X(0), L(0), A(0), Z(0),
      anchor, lever_init, z, sigma, false, false, true);
  Values v; v.insert(X(0), T); v.insert<double>(Z(0), beta);
  double err = f->error(v);
  double expected = 0.5 * std::pow((rho + beta - z) / sigma, 2);
  EXPECT_NEAR(err, expected, 1e-6);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
