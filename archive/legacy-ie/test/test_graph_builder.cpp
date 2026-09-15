// test/test_graph_builder.cpp — UT-6: GraphBuilder basic test
#include "uifgo/graph_builder.h"
#include "uifgo/config.h"
#include "uifgo/initializer.h"
#include "uifgo/types.h"
#include <gtsam/inference/Symbol.h>
#include <gtest/gtest.h>
#include <cmath>

using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;

// Helper: generate synthetic data
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
    const std::vector<int>& anchor_ids,
    const gtsam::Point3& tag_pos) {

  gtsam::Point3 anchors[4] = {
    {0, 0, 0}, {5, 0, 0}, {0, 5, 0}, {0, 0, 3}
  };

  std::vector<uifgo::UwbFrame> frames;
  for (double t : times) {
    uifgo::UwbFrame f;
    f.t = t;
    for (int aid : anchor_ids) {
      double d = (Vector3(tag_pos) - Vector3(anchors[aid % 4])).norm();
      f.ranges.push_back({aid, d, -50.0, -51.0});
    }
    frames.push_back(f);
  }
  return frames;
}

TEST(GraphBuilder, BuildsGraph) {
  uifgo::Config cfg;
  cfg.anchors = {
    {0, gtsam::Point3(0, 0, 0), 0.08},
    {1, gtsam::Point3(5, 0, 0), 0.08},
    {2, gtsam::Point3(0, 5, 0), 0.08},
    {3, gtsam::Point3(0, 0, 3), 0.08},
  };
  cfg.calib_lever = false;
  cfg.calib_anchor = false;
  cfg.calib_range_bias = false;

  auto imu   = GenStaticImu(0.0, 2.0);
  auto uwb   = GenConstUwb({0.5, 1.0, 1.5, 2.0}, {0, 1, 2, 3}, gtsam::Point3(2, 2, 1));

  uifgo::Initializer init(cfg);
  auto init_res = init.Run(imu, uwb);

  uifgo::GraphBuilder builder(cfg);
  NonlinearFactorGraph graph;
  Values values;
  std::vector<size_t> uwb_indices;

  builder.Build(uwb, imu, init_res, &graph, &values, &uwb_indices);

  EXPECT_GT(graph.size(), 0u);
  EXPECT_GT(uwb_indices.size(), 0u);
  // 4 keyframes => 3 IMU factors + 4 UWB factor groups + priors
  EXPECT_GE(graph.size(), 3u + 4u * 4u + 3u);  // IMU + UWB ranges + priors
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
