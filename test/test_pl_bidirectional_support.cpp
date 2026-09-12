#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <vector>

#include "uifgo/pl_bidirectional_provider.h"
#include "uifgo/pl_bidirectional_support.h"

namespace {

TEST(PlBidirectionalProductionProvider, LockedParameterIdentityIsDeterministic) {
  uifgo::Config cfg;
  cfg.cusum_forward_kappa = 0.5;
  cfg.cusum_forward_h = 7.0234689587858723;
  cfg.cusum_backward_kappa = 0.5;
  cfg.cusum_backward_h = 7.0234689587858714;
  cfg.discovery_gap_threshold_s = 1.0;
  cfg.cusum_parameter_provenance =
      "PL_BIDIRECTIONAL_CUSUM_SUPPORT_20260912_LOCKED";
  const auto first = uifgo::PlBidirectionalParameterIdentity(cfg);
  const auto second = uifgo::PlBidirectionalParameterIdentity(cfg);
  EXPECT_EQ(first, second);
  EXPECT_EQ(std::string(uifgo::kPlBidirectionalProductionProvider),
            "PL_BIDIRECTIONAL_CUSUM_V1");
  cfg.cusum_forward_h = 7.0;
  EXPECT_NE(first, uifgo::PlBidirectionalParameterIdentity(cfg));
}

uifgo::PlCusumInputRow Row(double time, int anchor, double z,
                           std::uint64_t obs, bool valid = true) {
  uifgo::PlCusumInputRow row;
  row.timestamp = time;
  row.group_id = "g" + std::to_string(obs);
  row.keyframe_id = static_cast<size_t>(time * 10 + 100);
  row.source_order = static_cast<size_t>(anchor);
  row.tag_id = 7;
  row.anchor_id = anchor;
  row.obs_id = obs;
  row.conditional_z = z;
  row.diagnostic_valid = valid;
  return row;
}

uifgo::PlBackwardCusumOptions BackwardOptions(double threshold = 5.0) {
  uifgo::PlBackwardCusumOptions options;
  options.threshold = threshold;
  options.original_clean_split_manifest_hash = "sha256:split";
  options.original_clean_input_hash = "sha256:clean";
  options.calibration_hash = "sha256:calibration";
  return options;
}

std::vector<uifgo::PlBidirectionalMembershipRow> Membership(
    const std::vector<uifgo::PlCusumInputRow>& rows,
    const std::vector<bool>& candidate, bool forward) {
  std::vector<uifgo::PlBidirectionalMembershipRow> output;
  for (size_t i = 0; i < rows.size(); ++i) {
    uifgo::PlBidirectionalMembershipRow row;
    row.input = rows[i];
    if (forward) row.forward_candidate = candidate[i];
    else row.backward_candidate = candidate[i];
    output.push_back(row);
  }
  return output;
}

TEST(PlBackwardCusum, ReverseRecurrenceAndInclusiveCrossing) {
  const auto result = uifgo::EvaluatePlBackwardCusum(
      {Row(0, 1, 1.0, 1), Row(1, 1, 1.0, 2)}, BackwardOptions(1.0));
  ASSERT_TRUE(result.valid) << result.status;
  ASSERT_EQ(result.trace.size(), 2u);
  EXPECT_EQ(result.trace[0].input.obs_id, 2u);
  EXPECT_DOUBLE_EQ(result.trace[0].b_after, 0.5);
  EXPECT_DOUBLE_EQ(result.trace[1].b_after, 1.0);
  EXPECT_TRUE(result.trace[1].threshold_crossing);
  EXPECT_EQ(result.alarm_count, 1u);
}

TEST(PlBackwardCusum, InterleavedLinksAreIndependent) {
  const auto result = uifgo::EvaluatePlBackwardCusum(
      {Row(0, 1, 1.5, 1), Row(0, 2, -2.0, 2),
       Row(0.5, 1, 1.5, 3), Row(0.5, 2, 1.0, 4)},
      BackwardOptions(2.0));
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.alarm_link_count, 1u);
  EXPECT_DOUBLE_EQ(result.per_link_max_b.at({7, 1}), 2.0);
  EXPECT_DOUBLE_EQ(result.per_link_max_b.at({7, 2}), 0.5);
}

TEST(PlBackwardCusum, InvalidAndGapResetUseBackwardReasons) {
  const auto invalid = uifgo::EvaluatePlBackwardCusum(
      {Row(0, 1, 1.5, 1), Row(0.5, 1,
          std::numeric_limits<double>::quiet_NaN(), 2)}, BackwardOptions());
  ASSERT_TRUE(invalid.valid);
  EXPECT_EQ(invalid.trace[0].reset_reason,
            "BACKWARD_NUMERICAL_INVALID_RESET");
  const auto gap = uifgo::EvaluatePlBackwardCusum(
      {Row(0, 1, 1.5, 1), Row(1.01, 1, 1.5, 2)}, BackwardOptions());
  ASSERT_TRUE(gap.valid);
  EXPECT_TRUE(gap.trace[1].reset);
  EXPECT_EQ(gap.trace[1].reset_reason, "BACKWARD_TIME_GAP_RESET");
  EXPECT_DOUBLE_EQ(gap.trace[1].b_before, 0.0);
}

TEST(PlBackwardCusum, ReverseLastZeroBackfillsOnlyDetectedExcursion) {
  const auto result = uifgo::EvaluatePlBackwardCusum(
      {Row(0, 1, -5.0, 1), Row(0.1, 1, 1.0, 2),
       Row(0.2, 1, 1.0, 3), Row(0.3, 1, 1.0, 4)},
      BackwardOptions(1.0));
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.trace.size(), 4u);
  EXPECT_TRUE(result.trace[0].backward_candidate);
  EXPECT_TRUE(result.trace[1].backward_candidate);
  EXPECT_TRUE(result.trace[2].backward_candidate);
  EXPECT_FALSE(result.trace[3].backward_candidate);
}

TEST(PlBidirectionalSupport, IntersectionRecoversMiddlePersistentInterval) {
  const std::vector<uifgo::PlCusumInputRow> rows{
      Row(0, 1, -10, 1), Row(0.1, 1, 1.5, 2),
      Row(0.2, 1, 1.5, 3), Row(0.3, 1, 1.5, 4),
      Row(0.4, 1, 1.5, 5), Row(0.5, 1, 0.2, 6),
      Row(0.6, 1, 0.2, 7), Row(0.7, 1, 0.2, 8),
      Row(0.8, 1, -10, 9)};
  const auto forward = uifgo::EvaluatePlPersistentCusum(
      rows, [] { auto o = uifgo::PlCusumOptions{}; o.threshold = 2.0; return o; }());
  const auto backward = uifgo::EvaluatePlBackwardCusum(rows, BackwardOptions(2.0));
  ASSERT_TRUE(forward.valid);
  ASSERT_TRUE(backward.valid);
  std::vector<bool> f(rows.size()), b(rows.size());
  for (const auto& trace : forward.trace) f[trace.input.obs_id - 1] = trace.candidate;
  for (const auto& trace : backward.trace) b[trace.input.obs_id - 1] = trace.backward_candidate;
  EXPECT_TRUE(f[7]);  // Forward support overshoots the physical end.
  EXPECT_TRUE(b[1]);  // Backward support reaches the onset side.
  const auto final = uifgo::IntersectPlCusumSupport(
      Membership(rows, f, true), Membership(rows, b, false),
      forward.detector_identity, backward.closure_identity);
  ASSERT_TRUE(final.valid) << final.status;
  ASSERT_EQ(final.support.segments.size(), 1u);
  EXPECT_EQ(final.support.segments[0].obs_ids,
            (std::vector<std::uint64_t>{2, 3, 4, 5}));
}

TEST(PlBidirectionalSupport, BackwardAloneCannotCreateDetection) {
  const std::vector<uifgo::PlCusumInputRow> rows{
      Row(0, 1, 1.5, 1), Row(0.1, 1, 1.5, 2)};
  const auto final = uifgo::IntersectPlCusumSupport(
      Membership(rows, {false, false}, true),
      Membership(rows, {true, true}, false), "forward", "backward");
  ASSERT_TRUE(final.valid);
  EXPECT_TRUE(final.support.segments.empty());
}

TEST(PlBidirectionalSupport, MissingBackwardAlarmCannotFallbackToForward) {
  const std::vector<uifgo::PlCusumInputRow> rows{
      Row(0, 1, 1.5, 1), Row(0.1, 1, 1.5, 2)};
  const auto final = uifgo::IntersectPlCusumSupport(
      Membership(rows, {true, true}, true),
      Membership(rows, {false, false}, false), "forward", "backward");
  ASSERT_TRUE(final.valid);
  EXPECT_TRUE(final.support.segments.empty());
}

TEST(PlBidirectionalSupport, DifferentAnchorsNeverIntersect) {
  const std::vector<uifgo::PlCusumInputRow> rows{
      Row(0, 1, 1.5, 1), Row(0, 2, 1.5, 2)};
  const auto final = uifgo::IntersectPlCusumSupport(
      Membership(rows, {true, false}, true),
      Membership(rows, {false, true}, false), "forward", "backward");
  ASSERT_TRUE(final.valid);
  EXPECT_TRUE(final.support.segments.empty());
}

TEST(PlBidirectionalSupport, ExactLinkObsUniverseIsRequired) {
  const auto forward = Membership({Row(0, 1, 1, 1)}, {true}, true);
  const auto backward = Membership({Row(0, 1, 1, 2)}, {true}, false);
  const auto final = uifgo::IntersectPlCusumSupport(
      forward, backward, "forward", "backward");
  EXPECT_FALSE(final.valid);
  EXPECT_EQ(final.status, "FORWARD_BACKWARD_IDENTITY_UNIVERSE_MISMATCH");
}

TEST(PlBidirectionalSupport, ScientificInputsContainNoGroupOrTruthFields) {
  // Both cores receive PlCusumInputRow, whose only scientific scalar is
  // conditional_z. There is no group statistic, truth, target, or injection
  // member that could influence either decision.
  const auto row = Row(0, 1, 1.0, 1);
  const auto first = uifgo::EvaluatePlBackwardCusum({row}, BackwardOptions());
  const auto second = uifgo::EvaluatePlBackwardCusum({row}, BackwardOptions());
  EXPECT_EQ(first.closure_identity, second.closure_identity);
  EXPECT_DOUBLE_EQ(first.trace[0].b_after, second.trace[0].b_after);
}

TEST(PlBidirectionalSupport, DeterministicIdentityAndPartitionHash) {
  const std::vector<uifgo::PlCusumInputRow> rows{
      Row(0, 1, 1.5, 1), Row(0.1, 1, 1.5, 2)};
  const auto backward1 = uifgo::EvaluatePlBackwardCusum(rows, BackwardOptions(1.0));
  const auto backward2 = uifgo::EvaluatePlBackwardCusum(rows, BackwardOptions(1.0));
  ASSERT_TRUE(backward1.valid);
  ASSERT_TRUE(backward2.valid);
  EXPECT_EQ(backward1.closure_identity, backward2.closure_identity);
  EXPECT_EQ(backward1.support.partition_hash, backward2.support.partition_hash);
  const auto final1 = uifgo::IntersectPlCusumSupport(
      Membership(rows, {true, true}, true),
      Membership(rows, {true, true}, false), "forward", backward1.closure_identity);
  const auto final2 = uifgo::IntersectPlCusumSupport(
      Membership(rows, {true, true}, true),
      Membership(rows, {true, true}, false), "forward", backward2.closure_identity);
  EXPECT_EQ(final1.support_identity, final2.support_identity);
  EXPECT_EQ(final1.support.partition_hash, final2.support.partition_hash);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
