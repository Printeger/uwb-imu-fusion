#include <gtest/gtest.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>

#include "uifgo/nlos_fde.h"
#include "uifgo/optimizer.h"

namespace uifgo {
namespace {

FdeOptions Options() {
  FdeOptions options;
  options.gap_threshold_s = 1.0;
  options.minimum_count = 2;
  options.minimum_duration_s = 0.5;
  options.preliminary_lm.max_iterations = 20;
  options.preliminary_lm.relative_tolerance = 1e-6;
  options.preliminary_lm.absolute_tolerance = 1e-8;
  return options;
}

FdeContext Context() {
  return {"plan", "source", "config", "calibration", "solver", "common",
          "graph", "values"};
}

FdeObservationRecord Row(std::uint64_t id, double time, double residual,
                         bool candidate = true, int anchor = 1) {
  FdeObservationRecord row;
  row.obs_id = id;
  row.tag_id = 0;
  row.anchor_id = anchor;
  row.sensor_time = time;
  row.valid = true;
  row.planned = true;
  row.tested = true;
  row.nlos_candidate = candidate;
  row.fault_detected = candidate;
  row.positive_excess = candidate;
  row.residual_m = residual;
  row.factor_sigma_m = 0.1;
  row.candidate_filter_reason = candidate ? "PENDING_TEMPORAL_FILTER"
                                          : "NOT_FAULT";
  return row;
}

TEST(NlosFde, ResidualDirectionAndStrictThresholdAreFrozen) {
  const auto options = Options();
  FdeObservationRecord negative;
  FdeObservationRecord positive;
  ClassifyFdeResidual(-0.4, 0.1, options, &negative);
  ClassifyFdeResidual(0.4, 0.1, options, &positive);
  EXPECT_TRUE(negative.fault_detected);
  EXPECT_TRUE(negative.positive_excess);
  EXPECT_TRUE(negative.nlos_candidate);
  EXPECT_TRUE(positive.fault_detected);
  EXPECT_FALSE(positive.positive_excess);
  EXPECT_FALSE(positive.nlos_candidate);

  const double boundary = std::sqrt(Chi2inv(0.99, 1));
  FdeObservationRecord equal;
  ClassifyFdeResidual(boundary, 1.0, options, &equal);
  EXPECT_FALSE(equal.fault_detected);
  FdeObservationRecord above;
  ClassifyFdeResidual(std::nextafter(boundary, INFINITY), 1.0, options,
                      &above);
  EXPECT_TRUE(above.fault_detected);
}

TEST(NlosFde, TemporalAggregationHonorsHealthStrictGapAndShortFiltering) {
  auto options = Options();
  std::vector<FdeObservationRecord> rows = {
      Row(1, 0.0, -0.4), Row(2, 0.5, -0.4),
      Row(3, 0.75, 0.0, false),
      Row(4, 1.0, -0.4), Row(5, 2.0, -0.4),
      Row(6, 3.01, -0.4)};
  size_t raw = 0;
  size_t filtered = 0;
  const auto partition = BuildFdeSupportPartition(
      &rows, options, Context(), &raw, &filtered);
  ASSERT_EQ(raw, 3u);
  EXPECT_EQ(filtered, 1u);
  ASSERT_EQ(partition.segments.size(), 2u);
  EXPECT_EQ(partition.segments[0].obs_ids,
            (std::vector<std::uint64_t>{1, 2}));
  EXPECT_EQ(partition.segments[1].obs_ids,
            (std::vector<std::uint64_t>{4, 5}));
  EXPECT_EQ(rows[5].candidate_filter_reason,
            "FILTERED_MIN_COUNT_AND_DURATION");
  EXPECT_TRUE(rows[5].nlos_candidate);
}

TEST(NlosFde, PersistentPositiveBiasFixtureProducesOneSegment) {
  auto options = Options();
  std::vector<FdeObservationRecord> rows = {
      Row(10, 4.0, -0.4), Row(11, 4.5, -0.6)};
  size_t raw = 0;
  size_t filtered = 0;
  const auto partition = BuildFdeSupportPartition(
      &rows, options, Context(), &raw, &filtered);
  ASSERT_EQ(partition.segments.size(), 1u);
  EXPECT_EQ(partition.segments[0].tag_id, 0);
  EXPECT_EQ(partition.segments[0].anchor_id, 1);
  EXPECT_DOUBLE_EQ(rows[0].residual_m, -0.4);
  EXPECT_DOUBLE_EQ(rows[1].residual_m, -0.6);
}

PaperInputPlan OneObservationPlan(double ledger_sigma) {
  PaperInputPlan plan;
  plan.plan_sha256 = "plan";
  ObservationRecord row;
  row.obs_id = 42;
  row.sensor_time = 1.0;
  row.valid = true;
  row.planned = true;
  row.nominal_sigma = ledger_sigma;
  row.anchor_id = 1;
  plan.observations.push_back(row);
  return plan;
}

TEST(NlosFde, ProviderUsesFactorNoiseAndEmptyCandidateIsSuccess) {
  const gtsam::Key key = gtsam::Symbol('z', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<double>(
      key, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 1e-3)));
  graph.add(gtsam::PriorFactor<double>(
      key, 3.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0)));
  gtsam::Values initial;
  initial.insert<double>(key, 0.0);
  FactorMeta meta;
  meta.factor_index = 1;
  meta.obs_id = 42;
  meta.factor_type = "uwb_range";
  meta.keys = {key};
  Config cfg;
  cfg.nlos_mode = "imu_aided_fde";
  cfg.calib_lever = false;
  cfg.calib_anchor = false;
  cfg.calib_range_bias = false;
  cfg.calib_td = false;
  auto options = Options();
  options.minimum_count = 1;
  options.minimum_duration_s = 0.0;
  const auto result = ImuAidedFdeSupportProvider(options).Run(
      graph, initial, {meta}, OneObservationPlan(1e-6), cfg, Context());
  ASSERT_TRUE(result.success()) << result.reason;
  ASSERT_EQ(result.observations.size(), 1u);
  EXPECT_DOUBLE_EQ(result.observations[0].factor_sigma_m, 1.0);
  EXPECT_GT(result.observations[0].statistic, Chi2inv(0.99, 1));
  EXPECT_TRUE(result.observations[0].nlos_candidate);

  graph[1] = boost::make_shared<gtsam::PriorFactor<double>>(
      key, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
  const auto empty = ImuAidedFdeSupportProvider(options).Run(
      graph, initial, {meta}, OneObservationPlan(1e-6), cfg, Context());
  ASSERT_TRUE(empty.success()) << empty.reason;
  EXPECT_TRUE(empty.partition.segments.empty());
  EXPECT_EQ(empty.reason, "REFERENCE_AND_ALL_TESTS_COMPLETE_EMPTY_SUPPORT");
}

TEST(NlosFde, ProviderRejectsIncompleteMappingAndIdentityIsSensitive) {
  const gtsam::Key key = gtsam::Symbol('z', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<double>(
      key, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0)));
  gtsam::Values initial;
  initial.insert<double>(key, 0.0);
  Config cfg;
  cfg.nlos_mode = "imu_aided_fde";
  cfg.calib_lever = false;
  cfg.calib_anchor = false;
  cfg.calib_range_bias = false;
  cfg.calib_td = false;
  const auto failed = ImuAidedFdeSupportProvider(Options()).Run(
      graph, initial, {}, OneObservationPlan(0.1), cfg, Context());
  EXPECT_EQ(failed.status, FdeStatus::INVALID_INPUT);

  const auto base = ComputeFdeIdentity(Options(), Context());
  auto options = Options();
  options.gap_threshold_s = 2.0;
  EXPECT_NE(base, ComputeFdeIdentity(options, Context()));
  options = Options();
  options.preliminary_lm.max_iterations++;
  EXPECT_NE(base, ComputeFdeIdentity(options, Context()));
  auto context = Context();
  context.calibration_hash = "changed";
  EXPECT_NE(base, ComputeFdeIdentity(Options(), context));
  context = Context();
  context.input_plan_hash = "changed";
  EXPECT_NE(base, ComputeFdeIdentity(Options(), context));
  context = Context();
  context.common_preparation_id = "changed";
  EXPECT_NE(base, ComputeFdeIdentity(Options(), context));
}

}  // namespace
}  // namespace uifgo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
