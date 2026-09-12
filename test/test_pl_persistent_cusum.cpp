#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <sstream>
#include <type_traits>

#include "uifgo/pl_persistent_cusum.h"

namespace {

uifgo::PlCusumInputRow Row(double time, int anchor, double z,
                           std::uint64_t obs, bool valid = true) {
  uifgo::PlCusumInputRow row;
  row.timestamp = time;
  row.group_id = "g" + std::to_string(obs);
  row.keyframe_id = static_cast<size_t>(obs);
  row.source_order = static_cast<size_t>(anchor);
  row.tag_id = 7;
  row.anchor_id = anchor;
  row.obs_id = obs;
  row.conditional_z = z;
  row.diagnostic_valid = valid;
  return row;
}

uifgo::PlCusumOptions Options(double threshold = 5.0) {
  uifgo::PlCusumOptions options;
  options.threshold = threshold;
  options.calibration_hash = "sha256:calibration";
  options.split_manifest_hash = "sha256:split";
  return options;
}

std::string Canonical(const uifgo::PlCusumResult& result) {
  std::ostringstream out;
  out.precision(17);
  out << result.valid << '|' << result.status << '|'
      << result.detector_identity << '|' << result.support.partition_hash;
  for (const auto& row : result.trace)
    out << '\n' << row.input.timestamp << ',' << row.input.anchor_id << ','
        << row.input.obs_id << ',' << row.g_before << ',' << row.g_after << ','
        << row.reset_reason << ',' << row.excursion_id << ','
        << row.threshold_crossing << ',' << row.candidate;
  return out.str();
}

TEST(PlPersistentCusum, RecurrenceAndNegativeEvidence) {
  const auto result = uifgo::EvaluatePlPersistentCusum(
      {Row(0, 1, 1.0, 1), Row(0.1, 1, 0.25, 2),
       Row(0.2, 1, -2.0, 3)}, Options());
  ASSERT_TRUE(result.valid) << result.status;
  ASSERT_EQ(result.trace.size(), 3u);
  EXPECT_DOUBLE_EQ(result.trace[0].g_after, 0.5);
  EXPECT_DOUBLE_EQ(result.trace[1].g_after, 0.25);
  EXPECT_DOUBLE_EQ(result.trace[2].g_after, 0.0);
  EXPECT_EQ(result.alarm_count, 0u);
}

TEST(PlPersistentCusum, PersistentModestSignalAndInclusiveThreshold) {
  const auto result = uifgo::EvaluatePlPersistentCusum(
      {Row(0, 1, 1.0, 1), Row(0.1, 1, 1.0, 2)}, Options(1.0));
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.trace[0].threshold_crossing);
  EXPECT_TRUE(result.trace[1].threshold_crossing);
  EXPECT_DOUBLE_EQ(result.trace[1].g_after, 1.0);
  EXPECT_EQ(result.alarm_count, 1u);
}

TEST(PlPersistentCusum, InterleavedLinksAreIndependent) {
  const auto result = uifgo::EvaluatePlPersistentCusum(
      {Row(0, 1, 1.5, 1), Row(0, 2, -3.0, 2),
       Row(0.1, 1, 1.5, 3), Row(0.1, 2, 1.0, 4)}, Options(2.0));
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.trace[0].g_after, 1.0);
  EXPECT_DOUBLE_EQ(result.trace[1].g_after, 0.0);
  EXPECT_DOUBLE_EQ(result.trace[2].g_after, 2.0);
  EXPECT_DOUBLE_EQ(result.trace[3].g_after, 0.5);
  EXPECT_EQ(result.alarm_link_count, 1u);
}

TEST(PlPersistentCusum, TimeGapResetIsStrictlyGreaterThanOneSecond) {
  const auto result = uifgo::EvaluatePlPersistentCusum(
      {Row(0, 1, 1.5, 1), Row(1.0, 1, 1.0, 2),
       Row(2.01, 1, 1.0, 3)}, Options());
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.trace[1].reset);
  EXPECT_TRUE(result.trace[2].reset);
  EXPECT_EQ(result.trace[2].reset_reason, "TIME_GAP_RESET");
  EXPECT_DOUBLE_EQ(result.trace[2].g_before, 0.0);
  EXPECT_DOUBLE_EQ(result.trace[2].g_after, 0.5);
}

TEST(PlPersistentCusum, InvalidAndNonfiniteRowsResetWithoutAlarm) {
  const auto result = uifgo::EvaluatePlPersistentCusum(
      {Row(0, 1, 5.0, 1), Row(0.1, 1, 9.0, 2, false),
       Row(0.2, 1, std::numeric_limits<double>::quiet_NaN(), 3)},
      Options(10.0));
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.invalid_row_count, 2u);
  EXPECT_TRUE(result.trace[1].reset);
  EXPECT_TRUE(result.trace[2].reset);
  EXPECT_EQ(result.alarm_count, 0u);
  EXPECT_FALSE(result.trace[1].candidate);
  EXPECT_FALSE(result.trace[2].candidate);
}

TEST(PlPersistentCusum, UnsuccessfulExcursionNeverBecomesCandidate) {
  const auto result = uifgo::EvaluatePlPersistentCusum(
      {Row(0, 1, 1.0, 1), Row(0.1, 1, 0.0, 2)}, Options(2.0));
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.trace[0].candidate);
  EXPECT_FALSE(result.trace[1].candidate);
  EXPECT_TRUE(result.support.segments.empty());
}

TEST(PlPersistentCusum, LastZeroBackfillAndPostAlarmPersistence) {
  const auto result = uifgo::EvaluatePlPersistentCusum(
      {Row(0, 1, 1.0, 1), Row(0.1, 1, 1.0, 2),
       Row(0.2, 1, 0.6, 3), Row(0.3, 1, -2.0, 4)}, Options(1.0));
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.trace.size(), 4u);
  EXPECT_TRUE(result.trace[0].candidate);
  EXPECT_TRUE(result.trace[1].candidate);
  EXPECT_TRUE(result.trace[2].candidate);
  EXPECT_FALSE(result.trace[3].candidate);
  ASSERT_EQ(result.support.segments.size(), 1u);
  EXPECT_EQ(result.support.segments[0].obs_ids,
            (std::vector<std::uint64_t>{1, 2, 3}));
  EXPECT_DOUBLE_EQ(result.support.segments[0].start_time, 0.0);
}

TEST(PlPersistentCusum, ReverseTimeFailsClosedBeforeSorting) {
  const auto result = uifgo::EvaluatePlPersistentCusum(
      {Row(1.0, 1, 1.0, 1), Row(0.9, 2, 1.0, 2),
       Row(0.8, 1, 1.0, 3)}, Options());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, "REVERSE_TIME_FAIL_CLOSED:7:1");
}

TEST(PlPersistentCusum, DeterministicReplayAndStableIdentity) {
  const std::vector<uifgo::PlCusumInputRow> rows{
      Row(0, 1, 1.0, 1), Row(0.1, 1, 1.0, 2)};
  const auto first = uifgo::EvaluatePlPersistentCusum(rows, Options(1.0));
  const auto second = uifgo::EvaluatePlPersistentCusum(rows, Options(1.0));
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(Canonical(first), Canonical(second));
  EXPECT_EQ(uifgo::PlPersistentCusumIdentity(Options(1.0)),
            uifgo::PlPersistentCusumIdentity(Options(1.0)));
}

TEST(PlPersistentCusum, GroupStatisticCannotInfluenceCore) {
  // PlCusumInputRow intentionally has no omnibus/group-statistic fields. Two
  // upstream records with identical projected fields therefore become the
  // exact same source-neutral input and must produce identical output.
  auto a = Row(0, 1, 1.0, 1);
  auto b = a;
  const auto one = uifgo::EvaluatePlPersistentCusum({a}, Options());
  const auto two = uifgo::EvaluatePlPersistentCusum({b}, Options());
  EXPECT_EQ(Canonical(one), Canonical(two));
}

TEST(PlPersistentCusum, ValidationStartsFromZeroNotCalibrationState) {
  const auto calibration = uifgo::EvaluatePlPersistentCusum(
      {Row(0, 1, 5.0, 1)}, Options(100.0));
  ASSERT_TRUE(calibration.valid);
  ASSERT_GT(calibration.trace.back().g_after, 0.0);
  const auto validation = uifgo::EvaluatePlPersistentCusum(
      {Row(10, 1, 1.0, 2)}, Options(100.0));
  ASSERT_TRUE(validation.valid);
  EXPECT_DOUBLE_EQ(validation.trace.front().g_before, 0.0);
  EXPECT_DOUBLE_EQ(validation.trace.front().g_after, 0.5);
}

TEST(PlPersistentCusum, LockedOptionsFailClosed) {
  auto wrong_kappa = Options();
  wrong_kappa.kappa = 0.49;
  EXPECT_FALSE(uifgo::EvaluatePlPersistentCusum({}, wrong_kappa).valid);
  auto wrong_gap = Options();
  wrong_gap.gap_threshold_s = 1.01;
  EXPECT_FALSE(uifgo::EvaluatePlPersistentCusum({}, wrong_gap).valid);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
