// test/test_config.cpp — UT-1: ConfigLoader tests
#include "uifgo/config.h"
#include <gtest/gtest.h>
#include <fstream>
#include <boost/filesystem.hpp>

TEST(ConfigLoader, LoadSampleYaml) {
  // Write a temporary config
  std::string tmp_path = "/tmp/test_slam.yaml";
  std::ofstream f(tmp_path);
  f << "anchors:\n"
    << "  - {id: 101, pos: [0.0, 0.0, 0.0], prior_sigma: 0.08}\n"
    << "  - {id: 102, pos: [5.0, 0.0, 0.0], prior_sigma: 0.08}\n"
    << "extrinsics:\n"
    << "  lever_arm_init: [0.1, 0.0, -0.05]\n"
    << "  calib_lever: true\n"
    << "  lever_prior_sigma: 0.03\n"
    << "imu:\n"
    << "  sigma_a: 0.2\n"
    << "  sigma_g: 0.02\n"
    << "  gravity: 9.80\n"
    << "uwb:\n"
    << "  sigma_range: 0.15\n"
    << "  v_max: 2.0\n"
    << "  nlos_rssi_diff: 8.0\n"
    << "solver:\n"
    << "  lm_max_iter: 50\n"
    << "  cauchy_k: 1.5\n";
  f.close();

  auto cfg = uifgo::ConfigLoader::Load(tmp_path);
  EXPECT_EQ(cfg.anchors.size(), 2u);
  EXPECT_EQ(cfg.anchors[0].id, 101);
  EXPECT_DOUBLE_EQ(cfg.anchors[0].pos.x(), 0.0);
  EXPECT_DOUBLE_EQ(cfg.anchors[1].pos.y(), 0.0);
  EXPECT_DOUBLE_EQ(cfg.lever_arm_init.x(), 0.1);
  EXPECT_TRUE(cfg.calib_lever);
  EXPECT_DOUBLE_EQ(cfg.lever_prior_sigma, 0.03);
  EXPECT_DOUBLE_EQ(cfg.sigma_a, 0.2);
  EXPECT_DOUBLE_EQ(cfg.sigma_g, 0.02);
  EXPECT_DOUBLE_EQ(cfg.gravity, 9.80);
  EXPECT_DOUBLE_EQ(cfg.sigma_range, 0.15);
  EXPECT_DOUBLE_EQ(cfg.v_max, 2.0);
  EXPECT_DOUBLE_EQ(cfg.nlos_rssi_diff, 8.0);
  EXPECT_EQ(cfg.lm_max_iter, 50);
  EXPECT_DOUBLE_EQ(cfg.cauchy_k, 1.5);
}

TEST(ConfigLoader, DefaultValues) {
  std::string tmp_path = "/tmp/test_default.yaml";
  std::ofstream f(tmp_path);
  f << "anchors:\n"
    << "  - {id: 1, pos: [1.0, 2.0, 3.0]}\n";
  f.close();

  auto cfg = uifgo::ConfigLoader::Load(tmp_path);
  EXPECT_EQ(cfg.anchors.size(), 1u);
  EXPECT_TRUE(cfg.calib_lever);  // default is true
  EXPECT_DOUBLE_EQ(cfg.sigma_a, 0.1);
  EXPECT_DOUBLE_EQ(cfg.gravity, 9.81);
  EXPECT_DOUBLE_EQ(cfg.nlos_rssi_diff, 6.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
