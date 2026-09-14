#include "uifgo/common_initializer.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <set>
#include <vector>

#include "uifgo/graph_builder.h"
#include "uifgo/paper_input.h"
#include "uifgo/paper_pose_prior_factor.h"

namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

struct Fixture {
  uifgo::Config config;
  std::vector<uifgo::ImuSample> imu;
  std::vector<uifgo::UwbFrame> raw_frames;
  uifgo::PaperInputPlan plan;
  std::vector<uifgo::UwbFrame> keyframes;
  uifgo::InitResult first;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values open_loop;
  std::vector<size_t> uwb_indices;
  std::vector<double> times;
};

Fixture MakeFixture(size_t state_count, double state_dt_s = 0.5,
                    double fixed_lag_horizon_s = 0.5,
                    size_t changed_from_state =
                        std::numeric_limits<size_t>::max(),
                    double future_range_offset_m = 0.0) {
  Fixture fixture;
  fixture.config.anchors = {
      {101, gtsam::Point3(0, 0, 0), 0.08},
      {102, gtsam::Point3(5, 0, 0), 0.08},
      {103, gtsam::Point3(0, 5, 0), 0.08},
      {104, gtsam::Point3(0, 0, 4), 0.08},
  };
  fixture.config.calib_lever = false;
  fixture.config.calib_anchor = false;
  fixture.config.calib_range_bias = false;
  fixture.config.lever_arm_init = gtsam::Point3(0, 0, 0);
  fixture.config.sigma_range = 0.1;
  fixture.config.kf_step = 1;
  fixture.config.initialization_progression_horizon_s = fixed_lag_horizon_s;
  fixture.config.lm_max_iter = 100;
  fixture.config.lm_rel_tol = 1e-6;
  fixture.config.lm_abs_tol = 1e-8;
  fixture.config.sigma_a = 0.1;
  fixture.config.sigma_g = 0.01;
  fixture.config.sigma_wa = 0.05;
  fixture.config.sigma_wg = 0.001;

  const gtsam::Point3 truth(2.0, 2.0, 1.0);
  for (size_t k = 0; k < state_count; ++k) {
    const double time = state_dt_s * static_cast<double>(k);
    uifgo::UwbFrame frame;
    frame.t = time;
    frame.tag_id = 7;
    for (size_t anchor = 0; anchor < fixture.config.anchors.size(); ++anchor) {
      const auto& item = fixture.config.anchors[anchor];
      uifgo::UwbRange range;
      range.anchor_id = item.id;
      range.dist = (truth - item.pos).norm() +
                   (k >= changed_from_state ? future_range_offset_m : 0.0);
      // Keep adjacent source messages distinct so the frozen exact-repeat
      // correlation rule does not collapse this stationary fixture.
      range.fp_rssi = -70.0 - 0.001 * static_cast<double>(k);
      range.rx_rssi = -71.0 - 0.001 * static_cast<double>(k);
      range.source_message_index = k;
      range.source_range_index = anchor;
      range.source_obs_index = 4 * k + anchor;
      range.source_time = time;
      range.source_tag_id = 7;
      frame.ranges.push_back(range);
    }
    fixture.raw_frames.push_back(frame);
  }
  const double end = fixture.raw_frames.back().t;
  for (double time = 0.0; time <= end + 1e-12; time += 0.01) {
    fixture.imu.push_back(
        {time, gtsam::Vector3(2.0, 0.0, 0.0), gtsam::Vector3::Zero()});
  }
  fixture.plan = uifgo::BuildPaperInputPlan(
      fixture.raw_frames, fixture.config, "common-initializer-fixture");
  fixture.keyframes = uifgo::MaterializePaperKeyframes(
      fixture.plan, uifgo::AllPlannedObservationMask(fixture.plan));
  fixture.first.ok = true;
  fixture.first.T0 = gtsam::Pose3(gtsam::Rot3(), truth);
  fixture.first.v0 = gtsam::Vector3::Zero();
  fixture.first.ba0 = gtsam::Vector3::Zero();
  fixture.first.bg0 = gtsam::Vector3::Zero();
  fixture.first.gravity_world = gtsam::Vector3::Zero();
  uifgo::GraphBuilder builder(fixture.config);
  builder.Build(fixture.keyframes, fixture.imu, fixture.first, &fixture.graph,
                &fixture.open_loop, &fixture.uwb_indices);
  EXPECT_EQ(uifgo::ReplacePosePriorsForPaperPath(&fixture.graph), 1u);
  for (const auto& frame : fixture.keyframes) fixture.times.push_back(frame.t);
  return fixture;
}

std::vector<std::uint64_t> SelectedIds(const uifgo::PaperInputPlan& plan) {
  std::vector<std::uint64_t> ids;
  for (size_t i = 0; i < plan.observations.size(); ++i) {
    if (plan.measurements[i].selected) ids.push_back(plan.observations[i].obs_id);
  }
  return ids;
}

double MaxPosition(const gtsam::Values& values, size_t count) {
  double maximum = 0.0;
  for (size_t k = 0; k < count; ++k)
    maximum = std::max(maximum,
                       values.at<gtsam::Pose3>(X(k)).translation().norm());
  return maximum;
}

void ExpectNavigationEqual(const gtsam::Values& left,
                           const gtsam::Values& right, size_t count) {
  for (size_t k = 0; k < count; ++k) {
    EXPECT_TRUE(left.at<gtsam::Pose3>(X(k)).equals(
        right.at<gtsam::Pose3>(X(k)), 1e-12));
    EXPECT_TRUE(left.at<gtsam::Vector3>(V(k)).isApprox(
        right.at<gtsam::Vector3>(V(k)), 1e-12));
    const auto left_bias = left.at<gtsam::imuBias::ConstantBias>(B(k));
    const auto right_bias = right.at<gtsam::imuBias::ConstantBias>(B(k));
    EXPECT_TRUE(left_bias.equals(right_bias, 1e-12));
  }
}

}  // namespace

TEST(CommonInitializer, CausalCompletedPrefixIgnoresFutureMeasurements) {
  const auto prefix = MakeFixture(5);
  const auto full = MakeFixture(11);
  const auto prefix_result = uifgo::CommonInitializer(prefix.config).Run(
      prefix.graph, prefix.open_loop, prefix.times, prefix.uwb_indices);
  const auto full_result = uifgo::CommonInitializer(full.config).Run(
      full.graph, full.open_loop, full.times, full.uwb_indices);
  ASSERT_TRUE(prefix_result.ok) << prefix_result.reason;
  ASSERT_TRUE(full_result.ok) << full_result.reason;
  // State 4 is still free when state 5 arrives and is therefore allowed to be
  // revised. States 0--3 have exited the lag and must remain immutable.
  ExpectNavigationEqual(prefix_result.values, full_result.values, 4);
}

TEST(CommonInitializer, FixedLagFrontiersIgnoreFutureMeasurements) {
  const auto reference = MakeFixture(9, 0.1, 0.25);
  const auto changed = MakeFixture(9, 0.1, 0.25, 5, 0.7);
  const auto left = uifgo::CommonInitializer(reference.config).Run(
      reference.graph, reference.open_loop, reference.times,
      reference.uwb_indices);
  const auto right = uifgo::CommonInitializer(changed.config).Run(
      changed.graph, changed.open_loop, changed.times, changed.uwb_indices);
  ASSERT_TRUE(left.ok) << left.reason;
  ASSERT_TRUE(right.ok) << right.reason;
  ASSERT_EQ(left.prefixes.size(), right.prefixes.size());
  // Measurements at states 5+ cannot affect windows ending at states 1--4.
  for (size_t i = 0; i < 4; ++i) {
    const auto& a = left.prefixes[i];
    const auto& b = right.prefixes[i];
    EXPECT_EQ(a.start_state, b.start_state);
    EXPECT_EQ(a.frontier_state, b.frontier_state);
    EXPECT_DOUBLE_EQ(a.initial_objective, b.initial_objective);
    EXPECT_DOUBLE_EQ(a.terminal_objective, b.terminal_objective);
    EXPECT_DOUBLE_EQ(a.stationarity.max_scaled_gradient_objective,
                     b.stationarity.max_scaled_gradient_objective);
  }
  // State 1 has left the lag before the changed measurements begin and must
  // remain immutable for the rest of both runs.
  EXPECT_TRUE(left.values.at<gtsam::Pose3>(X(1)).equals(
      right.values.at<gtsam::Pose3>(X(1)), 1e-12));
  EXPECT_TRUE(left.values.at<gtsam::Vector3>(V(1)).isApprox(
      right.values.at<gtsam::Vector3>(V(1)), 1e-12));
  EXPECT_TRUE(left.values.at<gtsam::imuBias::ConstantBias>(B(1)).equals(
      right.values.at<gtsam::imuBias::ConstantBias>(B(1)), 1e-12));
}

TEST(CommonInitializer, FixedLagBoundaryAndFactorMembershipAreBounded) {
  const auto fixture = MakeFixture(9, 0.1, 0.25);
  const auto result = uifgo::CommonInitializer(fixture.config).Run(
      fixture.graph, fixture.open_loop, fixture.times, fixture.uwb_indices);
  ASSERT_TRUE(result.ok) << result.reason;
  ASSERT_EQ(result.prefixes.size(), 8u);
  for (const auto& prefix : result.prefixes) {
    EXPECT_LE(prefix.free_window_duration_s, 0.25);
    EXPECT_EQ(prefix.free_start_state, prefix.start_state + 1);
    EXPECT_EQ(prefix.window_state_count,
              prefix.frontier_state - prefix.start_state + 1);
    EXPECT_EQ(prefix.optimized_state_count,
              prefix.frontier_state - prefix.free_start_state + 1);
    EXPECT_EQ(prefix.imu_factor_count, prefix.optimized_state_count);
    EXPECT_EQ(prefix.uwb_factor_count, 4 * prefix.optimized_state_count)
        << "a future-state UWB factor entered frontier "
        << prefix.frontier_state;
  }
  EXPECT_EQ(result.prefixes[1].start_state, 0u);  // states 1--2 free
  EXPECT_EQ(result.prefixes[2].start_state, 0u);  // boundary precedes free 1--3
  EXPECT_EQ(result.prefixes[2].free_start_state, 1u);
  EXPECT_GT(result.prefixes[2].optimized_state_count, 1u);
}

TEST(CommonInitializer, SparseCadenceUsesOlderPredecessorBoundaryAndImuBridge) {
  const auto fixture = MakeFixture(5, 1.0, 0.25);
  const auto result = uifgo::CommonInitializer(fixture.config).Run(
      fixture.graph, fixture.open_loop, fixture.times, fixture.uwb_indices);
  ASSERT_TRUE(result.ok) << result.reason;
  ASSERT_EQ(result.prefixes.size(), 4u);
  for (const auto& prefix : result.prefixes) {
    EXPECT_EQ(prefix.start_state + 1, prefix.frontier_state);
    EXPECT_EQ(prefix.free_start_state, prefix.frontier_state);
    EXPECT_EQ(prefix.optimized_state_count, 1u);
    EXPECT_EQ(prefix.imu_factor_count, 1u);
    EXPECT_DOUBLE_EQ(prefix.free_window_duration_s, 0.0);
    EXPECT_DOUBLE_EQ(prefix.boundary_bridge_duration_s, 1.0);
    EXPECT_DOUBLE_EQ(prefix.window_duration_s, 1.0);
  }
}

TEST(CommonInitializer, MissingAndInvalidImuIntervalsFailClosed) {
  auto missing = MakeFixture(5, 1.0, 0.25);
  for (size_t i = 0; i < missing.graph.size(); ++i) {
    if (boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(
            missing.graph.at(i))) {
      missing.graph.replace(i, gtsam::NonlinearFactor::shared_ptr());
      break;
    }
  }
  const auto missing_result = uifgo::CommonInitializer(missing.config).Run(
      missing.graph, missing.open_loop, missing.times, missing.uwb_indices);
  EXPECT_FALSE(missing_result.ok);
  EXPECT_NE(missing_result.reason.find("COMBINED_IMU_FACTOR_COUNT_MISMATCH"),
            std::string::npos);
  EXPECT_TRUE(missing_result.values.empty());

  auto malformed = MakeFixture(5, 1.0, 0.25);
  for (size_t i = 0; i < malformed.graph.size(); ++i) {
    const auto imu = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(
        malformed.graph.at(i));
    if (!imu) continue;
    auto invalid_pim = imu->preintegratedMeasurements();
    invalid_pim.resetIntegration();
    const auto& keys = imu->keys();
    malformed.graph.replace(i, boost::make_shared<gtsam::CombinedImuFactor>(
        keys[0], keys[1], keys[2], keys[3], keys[4], keys[5], invalid_pim));
    break;
  }
  const auto malformed_result = uifgo::CommonInitializer(malformed.config).Run(
      malformed.graph, malformed.open_loop, malformed.times,
      malformed.uwb_indices);
  EXPECT_FALSE(malformed_result.ok);
  EXPECT_NE(malformed_result.reason.find("INVALID_PIM_DURATION"),
            std::string::npos);
  EXPECT_TRUE(malformed_result.values.empty());
}

TEST(CommonInitializer, LeavesLedgerAndSelectedIdsUnchanged) {
  auto fixture = MakeFixture(7);
  const auto plan_hash = fixture.plan.plan_sha256;
  const auto selected_ids = SelectedIds(fixture.plan);
  const auto result = uifgo::CommonInitializer(fixture.config).Run(
      fixture.graph, fixture.open_loop, fixture.times, fixture.uwb_indices);
  ASSERT_TRUE(result.ok) << result.reason;
  EXPECT_EQ(fixture.plan.plan_sha256, plan_hash);
  EXPECT_EQ(SelectedIds(fixture.plan), selected_ids);
}

TEST(CommonInitializer, LeavesPhysicalUwbSigmaUnchanged) {
  auto fixture = MakeFixture(7);
  std::vector<double> sigma_before;
  for (size_t index : fixture.uwb_indices) {
    const auto factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(
        fixture.graph.at(index));
    ASSERT_TRUE(factor);
    sigma_before.push_back(factor->noiseModel()->sigmas()[0]);
  }
  const auto result = uifgo::CommonInitializer(fixture.config).Run(
      fixture.graph, fixture.open_loop, fixture.times, fixture.uwb_indices);
  ASSERT_TRUE(result.ok) << result.reason;
  for (size_t i = 0; i < fixture.uwb_indices.size(); ++i) {
    const auto factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(
        fixture.graph.at(fixture.uwb_indices[i]));
    ASSERT_TRUE(factor);
    EXPECT_DOUBLE_EQ(factor->noiseModel()->sigmas()[0], sigma_before[i]);
    EXPECT_DOUBLE_EQ(sigma_before[i], 0.1);
  }
}

TEST(CommonInitializer, FailedPrefixHasNoOpenLoopTailFallback) {
  auto fixture = MakeFixture(7);
  fixture.config.lm_max_iter = 0;
  const auto result = uifgo::CommonInitializer(fixture.config).Run(
      fixture.graph, fixture.open_loop, fixture.times, fixture.uwb_indices);
  EXPECT_FALSE(result.ok);
  EXPECT_EQ(result.status, "INITIALIZATION_FAILED");
  EXPECT_EQ(result.failure_count, 1u);
  EXPECT_EQ(result.retry_count, 0u);
  EXPECT_EQ(result.accepted_prefix_count, 0u);
  EXPECT_TRUE(result.values.empty())
      << "a failed initializer must not return the full open-loop tail";
}

TEST(CommonInitializer, PeriodicUwbSolveCorrectsOpenLoopDrift) {
  const auto fixture = MakeFixture(11);
  const auto result = uifgo::CommonInitializer(fixture.config).Run(
      fixture.graph, fixture.open_loop, fixture.times, fixture.uwb_indices);
  ASSERT_TRUE(result.ok) << result.reason;
  const double open_loop_max = MaxPosition(fixture.open_loop, 11);
  const double repaired_max = MaxPosition(result.values, 11);
  EXPECT_GT(open_loop_max, 10.0);
  EXPECT_LT(repaired_max, 0.5 * open_loop_max);
  EXPECT_EQ(result.prefixes.size(), 10u);
  EXPECT_EQ(result.accepted_prefix_count, 10u);
}

TEST(CommonInitializer, ConstOperationDoesNotChangeFinalPhysicalGraph) {
  auto fixture = MakeFixture(7);
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors_before;
  std::vector<std::vector<gtsam::Key>> keys_before;
  for (const auto& factor : fixture.graph) {
    factors_before.push_back(factor);
    keys_before.emplace_back(factor->keys().begin(), factor->keys().end());
  }
  const auto result = uifgo::CommonInitializer(fixture.config).Run(
      fixture.graph, fixture.open_loop, fixture.times, fixture.uwb_indices);
  ASSERT_TRUE(result.ok) << result.reason;
  ASSERT_EQ(fixture.graph.size(), factors_before.size());
  for (size_t i = 0; i < fixture.graph.size(); ++i) {
    EXPECT_EQ(fixture.graph.at(i), factors_before[i]);
    EXPECT_EQ(std::vector<gtsam::Key>(fixture.graph.at(i)->keys().begin(),
                                     fixture.graph.at(i)->keys().end()),
              keys_before[i]);
  }
}

TEST(CommonInitializer, NonstationaryWindowMayBeValidInitializationSeed) {
  auto fixture = MakeFixture(9, 0.1, 0.25);
  fixture.config.lm_max_iter = 1;
  const auto result = uifgo::CommonInitializer(fixture.config).Run(
      fixture.graph, fixture.open_loop, fixture.times, fixture.uwb_indices);
  ASSERT_TRUE(result.ok) << result.reason;
  bool observed_seed_valid_nonstationary = false;
  for (const auto& prefix : result.prefixes) {
    if (prefix.seed_quality.accepted && prefix.stationarity.valid &&
        !prefix.stationarity.stationary)
      observed_seed_valid_nonstationary = true;
  }
  EXPECT_TRUE(observed_seed_valid_nonstationary);
  EXPECT_TRUE(result.full_seed_quality.accepted);
}

TEST(CommonInitializer, SolverCertificateStillRejectsNonstationarySeed) {
  auto fixture = MakeFixture(9, 0.1, 0.25);
  fixture.config.lm_max_iter = 1;
  const auto result = uifgo::CommonInitializer(fixture.config).Run(
      fixture.graph, fixture.open_loop, fixture.times, fixture.uwb_indices);
  ASSERT_TRUE(result.ok) << result.reason;

  uifgo::SolverCertificateRequest request;
  request.termination_success = true;
  request.termination_reason = "TEST_ONLY_TERMINATION_SUCCESS";
  request.factor_integrity_passed = true;
  request.factor_integrity_reason = "DETERMINISTIC_FIXTURE";
  request.state_times_s = fixture.times;
  request.navigation_stationarity_tolerance_objective = 1e-6;
  const auto certificate =
      uifgo::CertifySolverResult(fixture.graph, result.values, request);
  EXPECT_FALSE(certificate.certified_success());
  EXPECT_EQ(certificate.status,
            uifgo::SolverCertificateStatus::CERTIFIED_FAILURE);
  EXPECT_TRUE(certificate.navigation_stationarity_applicable);
  EXPECT_FALSE(certificate.navigation_stationarity_passed);
}

TEST(CommonInitializer, AuthoritativeFrontier51FormulationRegression) {
  // Frozen Gate06R-D results from the same Walk1 frontier, factors, sigma,
  // Huber wrapper, LM settings and 1e-5 diagnostic. This characterization
  // prevents the established formulation regression from being reinterpreted
  // as a threshold or solver-parameter change.
  constexpr double kScientificDiagnosticTolerance = 1e-5;
  constexpr double kOldOneStateGradient = 1.0741484355758502e-5;
  constexpr double kFixedLagGradient = 4.9057538777930176e-6;
  EXPECT_GT(kOldOneStateGradient, kScientificDiagnosticTolerance);
  EXPECT_LE(kFixedLagGradient, kScientificDiagnosticTolerance);
  EXPECT_NE(uifgo::CommonInitializer::Identity(MakeFixture(4).config).find(
                "INITIALIZATION_SEED_QUALITY_V1"),
            std::string::npos);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
