#include "uifgo/graph_builder.h"
#include "uifgo/initializer.h"
#include "uifgo/paper_input.h"

#include <gtsam/inference/Symbol.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>

namespace {

uifgo::Config BaseConfig() {
  uifgo::Config cfg;
  cfg.anchors = {
      {101, gtsam::Point3(0, 0, 0), 0.08},
      {102, gtsam::Point3(5, 0, 0), 0.08},
      {103, gtsam::Point3(0, 5, 0), 0.08},
      {104, gtsam::Point3(0, 0, 3), 0.08},
  };
  cfg.calib_lever = false;
  cfg.calib_anchor = false;
  cfg.calib_range_bias = false;
  cfg.lever_arm_init = gtsam::Point3(0, 0, 0);
  cfg.yaw_align_frames = 0;
  cfg.kf_step = 1;
  cfg.min_range = 0.3;
  cfg.max_range = 100.0;
  cfg.nlos_rssi_diff = 6.0;
  return cfg;
}

std::vector<uifgo::ImuSample> StaticImu(double end) {
  std::vector<uifgo::ImuSample> imu;
  for (double t = 0.0; t <= end + 1e-9; t += 0.01) {
    imu.push_back({t, gtsam::Vector3(0, 0, 9.81),
                   gtsam::Vector3::Zero()});
  }
  return imu;
}

}  // namespace

TEST(PaperInput, LegalHighRssiDifferenceRemainsInPool) {
  auto cfg = BaseConfig();
  uifgo::UwbFrame frame;
  frame.t = 1.0;
  frame.tag_id = 7;
  frame.ranges.push_back({101, 4.0, -90.0, -70.0});
  const auto plan = uifgo::BuildPaperInputPlan({frame}, cfg, "high-rssi");
  ASSERT_EQ(plan.observations.size(), 1u);
  EXPECT_TRUE(plan.observations[0].valid);
  EXPECT_TRUE(plan.observations[0].suspected_nlos);
  EXPECT_TRUE(plan.observations[0].planned);
  const auto frames = uifgo::MaterializePaperKeyframes(
      plan, uifgo::AllPlannedObservationMask(plan));
  ASSERT_EQ(frames.size(), 1u);
  ASSERT_EQ(frames[0].ranges.size(), 1u);
  EXPECT_TRUE(frames[0].ranges[0].suspected_nlos);
}

TEST(PaperInput, SourceInvalidEntryIsLedgeredBeforeValidityFiltering) {
  auto cfg = BaseConfig();
  uifgo::UwbFrame frame;
  frame.t = 1.0;
  frame.tag_id = 7;
  uifgo::UwbRange invalid{101, 4.0, -70.0, -71.0};
  invalid.source_obs_index = 42;
  invalid.source_valid = false;
  invalid.source_validity_reason = "PROTOCOL_INVALID_FIXTURE";
  frame.ranges.push_back(invalid);
  frame.ranges.push_back({101, 4.1, -70.0, -71.0});
  frame.ranges.back().source_obs_index = 43;

  const auto plan = uifgo::BuildPaperInputPlan({frame}, cfg, "raw-ledger");
  ASSERT_EQ(plan.observations.size(), 2u);
  EXPECT_EQ(plan.observations[0].source_observation_index, 42u);
  EXPECT_FALSE(plan.observations[0].valid);
  EXPECT_EQ(plan.observations[0].validity_reason,
            "PROTOCOL_INVALID_FIXTURE");
  EXPECT_FALSE(plan.observations[0].suspected_nlos);
  EXPECT_TRUE(plan.observations[1].valid);
}

TEST(PaperInput, LoaderSourceOrdinalKeepsObsIdAcrossInputReordering) {
  auto cfg = BaseConfig();
  uifgo::UwbFrame first{2.0, 7, {{101, 4.0, -70.0, -71.0}}};
  uifgo::UwbFrame second{1.0, 7, {{102, 3.0, -70.0, -71.0}}};
  first.ranges[0].source_obs_index = 100;
  first.ranges[0].source_message_index = 25;
  first.ranges[0].source_range_index = 0;
  first.ranges[0].source_time = 2.0;
  first.ranges[0].source_tag_id = 7;
  second.ranges[0].source_obs_index = 101;
  second.ranges[0].source_message_index = 13;
  second.ranges[0].source_range_index = 0;
  second.ranges[0].source_time = 1.0;
  second.ranges[0].source_tag_id = 7;
  const auto a =
      uifgo::BuildPaperInputPlan({first, second}, cfg, "source-order");
  const auto b =
      uifgo::BuildPaperInputPlan({second, first}, cfg, "source-order");
  std::map<std::uint64_t, std::uint64_t> ids_a;
  std::map<std::uint64_t, std::uint64_t> ids_b;
  for (const auto& record : a.observations)
    ids_a[record.source_observation_index] = record.obs_id;
  for (const auto& record : b.observations)
    ids_b[record.source_observation_index] = record.obs_id;
  EXPECT_EQ(ids_a, ids_b);
}

TEST(PaperInput, RawLedgerUsesPerObservationTimeTagAndSourceIdentity) {
  auto cfg = BaseConfig();
  uifgo::UwbFrame grouped;
  grouped.t = 10.0;
  grouped.tag_id = 999;
  grouped.ranges.push_back({101, 4.0, -70.0, -71.0});
  grouped.ranges.push_back({102, 4.1, -70.0, -71.0});
  grouped.ranges[0].source_message_index = 40;
  grouped.ranges[0].source_range_index = 2;
  grouped.ranges[0].source_time = 9.91;
  grouped.ranges[0].source_tag_id = 7;
  grouped.ranges[1].source_message_index = 41;
  grouped.ranges[1].source_range_index = 0;
  grouped.ranges[1].source_time = 9.99;
  grouped.ranges[1].source_tag_id = 7;

  const auto plan =
      uifgo::BuildPaperInputPlan({grouped}, cfg, "raw-provenance");
  ASSERT_EQ(plan.observations.size(), 2u);
  EXPECT_DOUBLE_EQ(plan.observations[0].sensor_time, 9.91);
  EXPECT_EQ(plan.observations[0].tag_id, 7);
  EXPECT_EQ(plan.observations[0].source_message_index, 40u);
  EXPECT_EQ(plan.observations[0].source_range_index, 2u);
  EXPECT_DOUBLE_EQ(plan.observations[1].sensor_time, 9.99);
  EXPECT_EQ(plan.observations[1].tag_id, 7);
  EXPECT_NE(plan.observations[0].obs_id, plan.observations[1].obs_id);
}

TEST(PaperInput, StrategyMasksKeepIdsKeyframesAndSigmaSnapshot) {
  auto cfg = BaseConfig();
  cfg.kf_step = 2;
  std::vector<uifgo::UwbFrame> raw;
  for (int k = 0; k < 5; ++k) {
    uifgo::UwbFrame frame;
    frame.t = 0.5 + 0.1 * k;
    frame.tag_id = 7;
    frame.ranges.push_back({101, 4.0 + 0.01 * k, -80.0, -70.0});
    frame.ranges.push_back({102, 3.0 + 0.01 * k, -70.0, -71.0});
    raw.push_back(frame);
  }
  const auto plan = uifgo::BuildPaperInputPlan(raw, cfg, "mask-test");
  const auto repeated = uifgo::BuildPaperInputPlan(raw, cfg, "mask-test");
  ASSERT_EQ(plan.plan_hash, repeated.plan_hash);
  ASSERT_EQ(plan.observations.size(), repeated.observations.size());
  for (size_t i = 0; i < plan.observations.size(); ++i) {
    EXPECT_EQ(plan.observations[i].obs_id, repeated.observations[i].obs_id);
    EXPECT_EQ(plan.observations[i].keyframe_id,
              repeated.observations[i].keyframe_id);
    EXPECT_DOUBLE_EQ(plan.observations[i].nominal_sigma,
                     repeated.observations[i].nominal_sigma);
  }
  auto injected = raw;
  injected.front().ranges.front().dist += 0.5;
  const auto injected_plan =
      uifgo::BuildPaperInputPlan(injected, cfg, "mask-test");
  EXPECT_EQ(plan.observations.front().obs_id,
            injected_plan.observations.front().obs_id);
  EXPECT_NE(plan.plan_hash, injected_plan.plan_hash);

  auto mask_all = uifgo::AllPlannedObservationMask(plan);
  auto mask_without_suspected = mask_all;
  for (size_t i = 0; i < plan.observations.size(); ++i) {
    if (plan.observations[i].suspected_nlos) mask_without_suspected[i] = false;
  }
  const auto all = uifgo::MaterializePaperKeyframes(plan, mask_all);
  const auto reduced =
      uifgo::MaterializePaperKeyframes(plan, mask_without_suspected);
  ASSERT_EQ(all.size(), reduced.size());
  for (size_t k = 0; k < all.size(); ++k) {
    EXPECT_DOUBLE_EQ(all[k].t, reduced[k].t);
    EXPECT_EQ(all[k].tag_id, reduced[k].tag_id);
  }
  std::set<std::uint64_t> reduced_ids;
  for (const auto& frame : reduced)
    for (const auto& range : frame.ranges) reduced_ids.insert(range.obs_id);
  for (const auto& frame : all) {
    for (const auto& range : frame.ranges) {
      if (reduced_ids.count(range.obs_id)) {
        auto match = std::find_if(
            reduced.begin(), reduced.end(), [&](const uifgo::UwbFrame& f) {
              return std::any_of(f.ranges.begin(), f.ranges.end(),
                                 [&](const uifgo::UwbRange& r) {
                                   return r.obs_id == range.obs_id &&
                                          r.nominal_sigma ==
                                              range.nominal_sigma;
                                 });
            });
        EXPECT_NE(match, reduced.end());
      }
    }
  }
}

TEST(PaperInput, SyntheticFixedBetaCorrectsInitializationOnceAndAddsNoKey) {
  auto cfg = BaseConfig();
  const int tag_id = 7;
  const double beta = 0.3;  // explicit synthetic fixture, not calibration
  for (const auto& anchor : cfg.anchors) {
    cfg.fixed_beta_by_link[uifgo::RangeLinkKey(tag_id, anchor.id)] = beta;
  }
  const gtsam::Point3 true_position(2, 2, 1);
  std::vector<uifgo::UwbFrame> raw;
  for (double time : {0.5, 1.0}) {
    uifgo::UwbFrame frame;
    frame.t = time;
    frame.tag_id = tag_id;
    for (const auto& anchor : cfg.anchors) {
      const double rho =
          (gtsam::Vector3(true_position) - gtsam::Vector3(anchor.pos)).norm();
      frame.ranges.push_back({anchor.id, rho + beta, -70.0, -71.0});
    }
    raw.push_back(frame);
  }
  const double first_raw = raw.front().ranges.front().dist;
  const auto plan = uifgo::BuildPaperInputPlan(raw, cfg, "beta-0.3-fixture");
  const auto keyframes = uifgo::MaterializePaperKeyframes(
      plan, uifgo::AllPlannedObservationMask(plan));
  auto imu = StaticImu(1.0);
  uifgo::Initializer initializer(cfg);
  const auto init = initializer.Run(imu, keyframes);
  EXPECT_TRUE(init.ok);
  EXPECT_NEAR((gtsam::Vector3(init.T0.translation()) -
               gtsam::Vector3(true_position))
                  .norm(),
              0.0, 1e-3);
  EXPECT_DOUBLE_EQ(raw.front().ranges.front().dist, first_raw);

  uifgo::GraphBuilder builder(cfg);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::vector<size_t> uwb_indices;
  builder.Build(keyframes, imu, init, &graph, &values, &uwb_indices);
  using gtsam::symbol_shorthand::Z;
  for (const auto& anchor : cfg.anchors) EXPECT_FALSE(values.exists(Z(anchor.id)));
  ASSERT_FALSE(builder.factor_meta().empty());
  EXPECT_NE(builder.factor_meta().front().obs_id, 0u);
  EXPECT_NEAR(graph.at(uwb_indices.front())->error(values), 0.0, 1e-6);
}

TEST(PaperInput, PaperFixedBetaRequiresCoverageWhenConfigured) {
  auto cfg = BaseConfig();
  uifgo::UwbFrame frame;
  frame.t = 1.0;
  frame.tag_id = 7;
  frame.ranges.push_back({101, 4.0, -70.0, -71.0});
  frame.ranges.push_back({102, 4.1, -70.0, -71.0});
  const auto plan = uifgo::BuildPaperInputPlan({frame}, cfg, "beta-coverage");

  const auto missing = uifgo::ValidatePaperFixedBeta(plan, cfg);
  EXPECT_EQ(missing.status, "MISSING_CALIBRATION_DEVELOPMENT_ONLY");
  EXPECT_EQ(missing.used_links, 2u);

  cfg.fixed_beta_by_link["7:101"] = 0.3;
  EXPECT_THROW(uifgo::ValidatePaperFixedBeta(plan, cfg),
               std::invalid_argument);

  cfg.fixed_beta_by_link["7:102"] = 0.3;
  const auto complete = uifgo::ValidatePaperFixedBeta(plan, cfg);
  EXPECT_EQ(complete.status, "EXPLICIT_COMPLETE_NOT_PROVENANCE_VERIFIED");
  EXPECT_EQ(complete.configured_links, 2u);
}

TEST(PaperInput, PaperFixedBetaRejectsConfiguredUnknownAnchor) {
  auto cfg = BaseConfig();
  uifgo::UwbFrame frame{1.0, 7, {{101, 4.0, -70.0, -71.0}}};
  const auto plan = uifgo::BuildPaperInputPlan({frame}, cfg, "beta-anchor");
  cfg.fixed_beta_by_link["7:101"] = 0.3;
  cfg.fixed_beta_by_link["7:999"] = 0.3;
  EXPECT_THROW(uifgo::ValidatePaperFixedBeta(plan, cfg),
               std::invalid_argument);
}

TEST(PaperInput, MultipleTagsFailExplicitly) {
  auto cfg = BaseConfig();
  uifgo::UwbFrame a{0.0, 7, {{101, 1.0, -70.0, -71.0}}};
  uifgo::UwbFrame b{0.1, 8, {{101, 1.0, -70.0, -71.0}}};
  EXPECT_THROW(uifgo::BuildPaperInputPlan({a, b}, cfg, "multi-tag"),
               std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
