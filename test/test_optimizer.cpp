// test/test_optimizer.cpp — UT-7: Optimizer basic test
#include "uifgo/optimizer.h"
#include "uifgo/graph_builder.h"
#include "uifgo/config.h"
#include "uifgo/initializer.h"
#include <gtsam/inference/Symbol.h>
#include <gtest/gtest.h>
#include <cmath>

using namespace gtsam;
using symbol_shorthand::X;

// Reuse synthetic data generators (copied from test_graph_builder.cpp)
static std::vector<uifgo::ImuSample> GenStaticImu(double t0, double t1) {
  std::vector<uifgo::ImuSample> imu;
  const int count = static_cast<int>(std::llround((t1 - t0) / 0.01));
  for (int i = 0; i <= count; ++i) {
    const double t = t0 + 0.01 * i;
    imu.push_back({t, Vector3(0, 0, 9.81), Vector3(0, 0, 0)});
  }
  return imu;
}

static std::vector<uifgo::UwbFrame> GenConstUwb(
    const std::vector<double>& times,
    const gtsam::Point3& tag_pos) {
  gtsam::Point3 anchors[4] = {
    {0, 0, 0}, {5, 0, 0}, {0, 5, 0}, {0, 0, 3}
  };
  std::vector<uifgo::UwbFrame> frames;
  for (double t : times) {
    uifgo::UwbFrame f;
    f.t = t;
    for (int aid = 0; aid < 4; ++aid) {
      double d = (Vector3(tag_pos) - Vector3(anchors[aid])).norm();
      f.ranges.push_back({aid, d, -50.0, -51.0});
    }
    frames.push_back(f);
  }
  return frames;
}

TEST(Optimizer, StaticConverges) {
  uifgo::Config cfg;
  cfg.anchors = {
    {0, Point3(0, 0, 0), 0.08},
    {1, Point3(5, 0, 0), 0.08},
    {2, Point3(0, 5, 0), 0.08},
    {3, Point3(0, 0, 3), 0.08},
  };
  cfg.calib_lever = false;
  cfg.calib_anchor = false;
  cfg.calib_range_bias = false;
  cfg.lm_max_iter = 50;

  auto imu = GenStaticImu(0.0, 2.0);
  auto uwb = GenConstUwb({0.5, 1.0, 1.5, 2.0}, Point3(2, 2, 1));

  uifgo::Initializer init(cfg);
  auto init_res = init.Run(imu, uwb);

  uifgo::GraphBuilder builder(cfg);
  NonlinearFactorGraph graph;
  Values values;
  std::vector<size_t> uwb_indices;
  builder.Build(uwb, imu, init_res, &graph, &values, &uwb_indices);

  uifgo::Optimizer opt(cfg);
  auto result = opt.Optimize(graph, values, uwb_indices);

  EXPECT_LT(result.final_error, result.initial_error * 1.1);
  EXPECT_GT(result.inlier_uwb_indices.size(), 0u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
