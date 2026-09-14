#include "uifgo/graph_builder.h"
#include "uifgo/initializer.h"
#include "uifgo/paper_input.h"

#include <gtsam/inference/Symbol.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
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
  EXPECT_TRUE(plan.observations[0].source_valid);
  EXPECT_TRUE(plan.observations[0].suspected_nlos);
  EXPECT_TRUE(plan.measurements[0].estimator_usable);
  EXPECT_TRUE(plan.measurements[0].selected);
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
  EXPECT_FALSE(plan.observations[0].source_valid);
  EXPECT_EQ(plan.observations[0].source_validity_reason,
            "PROTOCOL_INVALID_FIXTURE");
  EXPECT_FALSE(plan.observations[0].suspected_nlos);
  EXPECT_FALSE(plan.measurements[0].estimator_usable);
  EXPECT_TRUE(plan.observations[1].source_valid);
  EXPECT_TRUE(plan.measurements[1].estimator_usable);
}

TEST(PaperInputIntegrity, InvalidUnusableUsableAndNlosAreDistinct) {
  auto cfg = BaseConfig();
  uifgo::UwbFrame frame;
  frame.t = 1.0;
  frame.tag_id = 7;
  frame.ranges.push_back(
      {101, std::numeric_limits<double>::quiet_NaN(), -70.0, -71.0});
  frame.ranges.push_back({999, 4.0, -70.0, -71.0});
  uifgo::UwbRange malformed{101, 4.0, -70.0, -71.0};
  malformed.source_valid = false;
  malformed.source_validity_reason = "PROTOCOL_IMPOSSIBLE_FORMAT";
  frame.ranges.push_back(malformed);
  frame.ranges.push_back({101, 4.1, -90.0, -70.0});

  const auto plan =
      uifgo::BuildPaperInputPlan({frame}, cfg, "integrity-states");
  ASSERT_EQ(plan.observations.size(), 4u);
  EXPECT_EQ(plan.measurements[0].integrity_status,
            uifgo::UwbIntegrityStatus::SOURCE_INVALID);
  EXPECT_FALSE(plan.observations[0].source_valid);
  EXPECT_EQ(plan.measurements[1].integrity_status,
            uifgo::UwbIntegrityStatus::ESTIMATOR_UNUSABLE);
  EXPECT_TRUE(plan.observations[1].source_valid);
  EXPECT_EQ(plan.measurements[1].usability_reason, "UNKNOWN_ANCHOR");
  EXPECT_EQ(plan.measurements[2].integrity_status,
            uifgo::UwbIntegrityStatus::SOURCE_INVALID);
  EXPECT_EQ(plan.measurements[3].integrity_status,
            uifgo::UwbIntegrityStatus::USABLE);
  EXPECT_TRUE(plan.observations[3].suspected_nlos);
  EXPECT_TRUE(plan.measurements[3].selected);
  EXPECT_EQ(plan.observations.size(), 4u)
      << "invalid and suspected rows must both remain in the raw ledger";
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
    EXPECT_EQ(plan.measurements[i].keyframe_id,
              repeated.measurements[i].keyframe_id);
    EXPECT_DOUBLE_EQ(plan.measurements[i].sensor_sigma,
                     repeated.measurements[i].sensor_sigma);
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

TEST(PaperInputV2, RawLedgerAndObsIdsAreInvariantAcrossStateSteps) {
  auto cfg1 = BaseConfig();
  cfg1.kf_step = 1;
  auto cfg2 = cfg1;
  cfg2.kf_step = 2;
  auto cfg4 = cfg1;
  cfg4.kf_step = 4;

  std::vector<uifgo::UwbFrame> raw;
  for (size_t frame_index = 0; frame_index < 5; ++frame_index) {
    uifgo::UwbFrame frame;
    frame.t = 1.0 + 0.1 * static_cast<double>(frame_index);
    frame.tag_id = 7;
    uifgo::UwbRange range{101, 4.0, -70.0, -71.0};
    range.source_obs_index = 100 + frame_index;
    range.source_message_index = 200 + frame_index;
    range.source_range_index = 0;
    range.source_time = frame.t;
    range.source_tag_id = frame.tag_id;
    frame.ranges.push_back(range);
    raw.push_back(frame);
  }

  const auto plan1 = uifgo::BuildPaperInputPlan(raw, cfg1, "step-decoupled");
  const auto plan2 = uifgo::BuildPaperInputPlan(raw, cfg2, "step-decoupled");
  const auto plan4 = uifgo::BuildPaperInputPlan(raw, cfg4, "step-decoupled");
  ASSERT_EQ(plan1.observations.size(), plan2.observations.size());
  ASSERT_EQ(plan1.observations.size(), plan4.observations.size());
  EXPECT_EQ(plan1.observation_ledger_sha256,
            plan2.observation_ledger_sha256);
  EXPECT_EQ(plan1.observation_ledger_sha256,
            plan4.observation_ledger_sha256);
  EXPECT_EQ(plan1.integrity_plan_sha256, plan2.integrity_plan_sha256);
  EXPECT_EQ(plan1.integrity_plan_sha256, plan4.integrity_plan_sha256);
  for (size_t i = 0; i < plan1.observations.size(); ++i) {
    EXPECT_EQ(plan1.observations[i].obs_id, plan2.observations[i].obs_id);
    EXPECT_EQ(plan1.observations[i].obs_id, plan4.observations[i].obs_id);
    EXPECT_EQ(plan1.observations[i].source_observation_index,
              plan4.observations[i].source_observation_index);
    EXPECT_DOUBLE_EQ(plan1.observations[i].sensor_time,
                     plan2.observations[i].sensor_time);
    EXPECT_DOUBLE_EQ(plan1.observations[i].raw_range,
                     plan4.observations[i].raw_range);
  }
}

TEST(PaperInputV2, SensorSigmaIsInvariantAcrossStateSteps) {
  auto cfg1 = BaseConfig();
  cfg1.kf_step = 1;
  cfg1.sigma_range = 0.17;
  auto cfg2 = cfg1;
  cfg2.kf_step = 2;
  auto cfg4 = cfg1;
  cfg4.kf_step = 4;
  std::vector<uifgo::UwbFrame> raw;
  for (int k = 0; k < 9; ++k) {
    uifgo::UwbFrame frame;
    frame.t = 1.0 + (k == 0 ? 0.0 : 0.01 * k * k);
    frame.tag_id = 7;
    frame.ranges.push_back({101, 4.0, -70.0, -71.0});
    raw.push_back(frame);
  }
  const auto plan1 = uifgo::BuildPaperInputPlan(raw, cfg1, "sigma-fixed");
  const auto plan2 = uifgo::BuildPaperInputPlan(raw, cfg2, "sigma-fixed");
  const auto plan4 = uifgo::BuildPaperInputPlan(raw, cfg4, "sigma-fixed");
  for (size_t i = 0; i < plan1.measurements.size(); ++i) {
    EXPECT_DOUBLE_EQ(plan1.measurements[i].sensor_sigma, 0.17);
    EXPECT_DOUBLE_EQ(plan1.measurements[i].sensor_sigma,
                     plan2.measurements[i].sensor_sigma);
    EXPECT_DOUBLE_EQ(plan1.measurements[i].sensor_sigma,
                     plan4.measurements[i].sensor_sigma);
  }
}

TEST(PaperInputV2, StateStepChangesStatesWithoutMutatingSourceLedger) {
  auto cfg1 = BaseConfig();
  cfg1.kf_step = 1;
  auto cfg4 = cfg1;
  cfg4.kf_step = 4;
  std::vector<uifgo::UwbFrame> raw;
  for (int k = 0; k < 9; ++k) {
    uifgo::UwbFrame frame;
    frame.t = 0.5 + 0.1 * k;
    frame.tag_id = 7;
    frame.ranges.push_back({101, 4.0 + k, -70.0, -71.0});
    raw.push_back(frame);
  }
  const auto plan1 = uifgo::BuildPaperInputPlan(raw, cfg1, "state-only");
  const auto plan4 = uifgo::BuildPaperInputPlan(raw, cfg4, "state-only");
  EXPECT_EQ(plan1.keyframes.size(), 9u);
  EXPECT_EQ(plan4.keyframes.size(), 3u);
  EXPECT_NE(plan1.state_timeline_sha256, plan4.state_timeline_sha256);
  EXPECT_EQ(plan1.observation_ledger_sha256,
            plan4.observation_ledger_sha256);
  EXPECT_NE(plan1.measurement_plan_sha256, plan4.measurement_plan_sha256);
  EXPECT_EQ(plan1.integrity_plan_sha256, plan4.integrity_plan_sha256);
}

TEST(PaperInputV2, StateStepOneSelectsEveryUsableObservationAtFixedSigma) {
  auto cfg = BaseConfig();
  cfg.kf_step = 1;
  cfg.kf_min_interval = 0.0;
  cfg.sigma_range = 0.13;
  std::vector<uifgo::UwbFrame> raw;
  size_t ordinal = 0;
  for (double time : {0.5, 0.51, 0.9, 1.7}) {
    uifgo::UwbFrame frame;
    frame.t = time;
    frame.tag_id = 7;
    frame.ranges.push_back(
        {101, 4.0 + 0.01 * ordinal, -70.0, -71.0});
    frame.ranges.push_back(
        {102, 4.2 + 0.01 * ordinal, -70.0, -71.0});
    raw.push_back(frame);
    ++ordinal;
  }
  const auto plan = uifgo::BuildPaperInputPlan(raw, cfg, "all-usable");
  const size_t usable = std::count_if(
      plan.measurements.begin(), plan.measurements.end(),
      [](const auto& row) { return row.estimator_usable; });
  const size_t selected = std::count_if(
      plan.measurements.begin(), plan.measurements.end(),
      [](const auto& row) { return row.selected; });
  EXPECT_EQ(usable, 8u);
  EXPECT_EQ(selected, usable);
  const auto frames = uifgo::MaterializePaperKeyframes(
      plan, uifgo::AllSelectedObservationMask(plan));
  size_t factor_inputs = 0;
  for (const auto& frame : frames) {
    factor_inputs += frame.ranges.size();
    for (const auto& range : frame.ranges) {
      EXPECT_DOUBLE_EQ(range.nominal_sigma, 0.13);
      EXPECT_EQ(range.noise_semantics,
                uifgo::UwbNoiseSemantics::FIXED_SENSOR_SIGMA_V2);
    }
  }
  EXPECT_EQ(factor_inputs, usable);
}

TEST(PaperInputV2, LegacyAdaptiveNoiseModeIsExplicitlyMarked) {
  const uifgo::UwbRange legacy{101, 4.0, -70.0, -71.0};
  EXPECT_EQ(legacy.noise_semantics,
            uifgo::UwbNoiseSemantics::LEGACY_SELECTED_GAP_ADAPTIVE_V1);
  auto cfg = BaseConfig();
  uifgo::UwbFrame frame{1.0, 7, {{101, 4.0, -70.0, -71.0}}};
  const auto plan = uifgo::BuildPaperInputPlan({frame}, cfg, "noise-mode");
  const auto materialized = uifgo::MaterializePaperKeyframes(
      plan, uifgo::AllSelectedObservationMask(plan));
  ASSERT_EQ(materialized.size(), 1u);
  ASSERT_EQ(materialized[0].ranges.size(), 1u);
  EXPECT_EQ(materialized[0].ranges[0].noise_semantics,
            uifgo::UwbNoiseSemantics::FIXED_SENSOR_SIGMA_V2);
}

TEST(PaperInputIntegrity,
     ExactAdjacentPayloadsPreserveLedgerButUseOneFactorPerGroup) {
  auto cfg = BaseConfig();
  std::vector<uifgo::UwbFrame> raw;
  for (size_t frame_index = 0; frame_index < 2; ++frame_index) {
    uifgo::UwbFrame frame;
    frame.t = 0.5 + 0.5 * static_cast<double>(frame_index);
    frame.tag_id = 7;
    for (const auto& anchor : cfg.anchors) {
      uifgo::UwbRange range{anchor.id, 4.0, -70.0, -71.0};
      range.source_obs_index = 10 * frame_index + frame.ranges.size();
      range.source_message_index = frame_index;
      range.source_range_index = frame.ranges.size();
      range.source_time = frame.t;
      range.source_tag_id = frame.tag_id;
      frame.ranges.push_back(range);
    }
    raw.push_back(frame);
  }
  const auto plan =
      uifgo::BuildPaperInputPlan(raw, cfg, "duplicate-integrity");
  const auto keyframes = uifgo::MaterializePaperKeyframes(
      plan, uifgo::AllPlannedObservationMask(plan));
  ASSERT_EQ(plan.observations.size(), 8u);
  ASSERT_EQ(keyframes.size(), 2u);
  EXPECT_EQ(keyframes.front().ranges.size(), 4u);
  EXPECT_TRUE(keyframes.back().ranges.empty());
  for (size_t anchor_index = 0; anchor_index < 4; ++anchor_index) {
    const auto& first = plan.measurements[anchor_index];
    const auto& repeated = plan.measurements[4 + anchor_index];
    EXPECT_NE(plan.observations[anchor_index].obs_id,
              plan.observations[4 + anchor_index].obs_id);
    EXPECT_EQ(repeated.integrity_status,
              uifgo::UwbIntegrityStatus::STALE_REPEAT);
    EXPECT_EQ(first.correlation_group_id, repeated.correlation_group_id);
    EXPECT_EQ(first.correlation_group_size, 2u);
    EXPECT_TRUE(first.independent_likelihood_representative);
    EXPECT_FALSE(repeated.independent_likelihood_representative);
    EXPECT_EQ(repeated.selection_reason, "NOT_SELECTED_CORRELATED_REPEAT");
  }

  uifgo::InitResult init;
  init.ok = true;
  init.T0 = gtsam::Pose3();
  init.v0 = gtsam::Vector3::Zero();
  init.ba0 = gtsam::Vector3::Zero();
  init.bg0 = gtsam::Vector3::Zero();
  init.gravity_world = gtsam::Vector3(0.0, 0.0, -9.81);
  uifgo::GraphBuilder builder(cfg);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::vector<size_t> uwb_indices;
  builder.Build(keyframes, StaticImu(1.0), init, &graph, &values,
                &uwb_indices);

  EXPECT_EQ(uwb_indices.size(), 4u);
  std::set<std::uint64_t> factor_obs_ids;
  for (const auto& meta : builder.factor_meta()) {
    if (meta.factor_type == "uwb_range") factor_obs_ids.insert(meta.obs_id);
  }
  EXPECT_EQ(factor_obs_ids.size(), 4u);
  EXPECT_TRUE(factor_obs_ids.count(keyframes.front().ranges.front().obs_id));
  EXPECT_FALSE(factor_obs_ids.count(plan.observations[4].obs_id));
}

TEST(PaperInputIntegrity, PersistentPositiveBiasIsNotAnIntegrityRejection) {
  auto cfg = BaseConfig();
  std::vector<uifgo::UwbFrame> raw;
  for (size_t k = 0; k < 4; ++k) {
    uifgo::UwbFrame frame;
    frame.t = 1.0 + 0.1 * k;
    frame.tag_id = 7;
    uifgo::UwbRange biased{101, 5.5 + 0.01 * k, -90.0, -70.0};
    biased.source_message_index = k;
    biased.source_range_index = 0;
    biased.source_obs_index = 100 + k;
    biased.source_time = frame.t;
    biased.source_tag_id = frame.tag_id;
    frame.ranges.push_back(biased);
    raw.push_back(frame);
  }
  const auto plan =
      uifgo::BuildPaperInputPlan(raw, cfg, "persistent-positive-bias");
  ASSERT_EQ(plan.observations.size(), 4u);
  for (size_t i = 0; i < plan.observations.size(); ++i) {
    EXPECT_TRUE(plan.observations[i].suspected_nlos);
    EXPECT_DOUBLE_EQ(plan.observations[i].raw_range, 5.5 + 0.01 * i);
    EXPECT_EQ(plan.measurements[i].integrity_status,
              uifgo::UwbIntegrityStatus::USABLE);
    EXPECT_TRUE(plan.measurements[i].selected);
    EXPECT_TRUE(plan.measurements[i].independent_likelihood_representative);
    const auto& stage1_row =
        uifgo::MeasurementForObservation(plan, plan.observations[i]);
    EXPECT_EQ(stage1_row.obs_id, plan.observations[i].obs_id);
    EXPECT_EQ(plan.observations[i].source_message_index, i);
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
