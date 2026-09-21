// C1-c lifecycle tests (design freeze §7.6 rows 1-8 + cold start).
//
// Every rule is exercised in both directions: the allowed path and the
// fail-closed path.  These are decision-level tests: they lock the semantics
// that the Part C pipeline consumes (the pipeline-level effects are covered by
// test_history_summary_pipeline.cpp).

#include <gtest/gtest.h>

#include <string>

#include "uwb_imu_pl/integrity/history_summary_lifecycle.hpp"

namespace uwb_imu_pl {
namespace {

// Row 1: only a genuinely future onset may use zero fault columns.
TEST(HistorySummaryLifecycle, FutureOnsetZeroColumnsRule) {
  EXPECT_FALSE(futureOnsetAllowsZeroColumns(10, 10))
      << "onset at the horizon boundary";
  EXPECT_FALSE(futureOnsetAllowsZeroColumns(9, 10))
      << "onset before the horizon";
  EXPECT_TRUE(futureOnsetAllowsZeroColumns(11, 10));
  EXPECT_TRUE(futureOnsetAllowsZeroColumns(40, 26));
}

// Row 2: response + detection cost are eliminated together; identity, time
// support and a positive probability basis are required.
TEST(HistorySummaryLifecycle, ModeAbsorptionKeepsResponseAndCostTogether) {
  HistoryModeAbsorptionRequest request;
  request.has_input_response = true;
  request.has_detection_cost = true;
  request.has_event_identity = true;
  request.has_time_support = true;
  request.prior_probability_bound = 1e-4;
  EXPECT_TRUE(absorbHistoryMode(request).accepted);

  HistoryModeAbsorptionRequest no_cost = request;
  no_cost.has_detection_cost = false;
  const auto no_cost_result = absorbHistoryMode(no_cost);
  EXPECT_FALSE(no_cost_result.accepted);
  EXPECT_NE(no_cost_result.reason.find("together"), std::string::npos);

  HistoryModeAbsorptionRequest no_response = request;
  no_response.has_input_response = false;
  EXPECT_FALSE(absorbHistoryMode(no_response).accepted);

  HistoryModeAbsorptionRequest no_event = request;
  no_event.has_event_identity = false;
  EXPECT_FALSE(absorbHistoryMode(no_event).accepted);

  HistoryModeAbsorptionRequest no_time = request;
  no_time.has_time_support = false;
  EXPECT_FALSE(absorbHistoryMode(no_time).accepted);

  HistoryModeAbsorptionRequest no_probability = request;
  no_probability.prior_probability_bound = 0.0;
  EXPECT_FALSE(absorbHistoryMode(no_probability).accepted);
}

// Row 3: leaving the window is not a deletion.
TEST(HistorySummaryLifecycle, SourceLeavingWindowKeepsMode) {
  EXPECT_TRUE(sourceLeavingWindowKeepsMode(true, false))
      << "still explicit: nothing to delete";
  EXPECT_TRUE(sourceLeavingWindowKeepsMode(false, true))
      << "left the window but still represented in the history basis";
  EXPECT_FALSE(sourceLeavingWindowKeepsMode(false, false))
      << "left the window and gone from the basis: a deletion, not a rule";
}

// Row 4: a fault ending is not clearance.
TEST(HistorySummaryLifecycle, FaultEndIsNotClearance) {
  HistoryClearanceEvidence live;
  live.response_norm = 0.5;
  live.detector_norm = 0.25;
  live.tolerance = 1e-12;
  const auto refused = evaluateHistoryClearance(live);
  EXPECT_FALSE(refused.cleared);
  EXPECT_NE(refused.reason.find("not clearance"), std::string::npos);

  HistoryClearanceEvidence response_only = live;
  response_only.response_norm = 0.0;
  EXPECT_FALSE(evaluateHistoryClearance(response_only).cleared)
      << "detection content still carries the mode";

  HistoryClearanceEvidence detector_only = live;
  detector_only.detector_norm = 0.0;
  EXPECT_FALSE(evaluateHistoryClearance(detector_only).cleared)
      << "response still carries the mode";

  HistoryClearanceEvidence cleared = live;
  cleared.response_norm = 0.0;
  cleared.detector_norm = 0.0;
  EXPECT_TRUE(evaluateHistoryClearance(cleared).cleared);

  HistoryClearanceEvidence proven = live;
  proven.response_proven_zero = true;
  proven.detector_proven_zero = true;
  EXPECT_TRUE(evaluateHistoryClearance(proven).cleared)
      << "an independent zero proof is admissible";
}

// Row 5: retirement needs the full evidence package or a clean reset.
TEST(HistorySummaryLifecycle, RetirementRequiresProofEnvelopeAndRisk) {
  HistoryRetirementRequest request;
  EXPECT_FALSE(evaluateHistoryRetirement(request).retire);
  request.no_impact_proof = true;
  EXPECT_FALSE(evaluateHistoryRetirement(request).retire);
  request.envelope_covers = true;
  EXPECT_FALSE(evaluateHistoryRetirement(request).retire);
  request.risk_handled = true;
  EXPECT_TRUE(evaluateHistoryRetirement(request).retire);

  HistoryRetirementRequest no_proof;
  no_proof.envelope_covers = true;
  no_proof.risk_handled = true;
  EXPECT_FALSE(evaluateHistoryRetirement(no_proof).retire);

  HistoryRetirementRequest reset;
  reset.clean_reset = true;
  EXPECT_TRUE(evaluateHistoryRetirement(reset).retire);
}

// Row 6: order/family hot switch is refused by default; a changed mode set
// requires a rebuild, never a silent reuse of the old summary.
TEST(HistorySummaryLifecycle, HotSwitchDefaultsToRefuse) {
  HistoryHotSwitchRequest unchanged;
  unchanged.active_mode_set = 7;
  unchanged.requested_mode_set = 7;
  EXPECT_TRUE(evaluateHistoryHotSwitch(unchanged).accepted);

  HistoryHotSwitchRequest changed;
  changed.active_mode_set = 7;
  changed.requested_mode_set = 9;
  const auto refused = evaluateHistoryHotSwitch(changed);
  EXPECT_FALSE(refused.accepted);
  EXPECT_FALSE(refused.rebuild_required);
  EXPECT_NE(refused.reason.find("refused"), std::string::npos);

  HistoryHotSwitchRequest changed_with_rebuild = changed;
  changed_with_rebuild.rebuild_available = true;
  const auto rebuilt = evaluateHistoryHotSwitch(changed_with_rebuild);
  EXPECT_FALSE(rebuilt.accepted);
  EXPECT_TRUE(rebuilt.rebuild_required);
}

// Rows 7/8 + cold start: explicit state names and the auto_shrink rule.
TEST(HistorySummaryLifecycle, ExplicitStatesAndAutoShrink) {
  EXPECT_STREQ(toString(HistorySummaryState::NotRequired),
               "HISTORY_SUMMARY_NOT_REQUIRED");
  EXPECT_STREQ(toString(HistorySummaryState::Valid), "HISTORY_SUMMARY_VALID");
  EXPECT_STREQ(toString(HistorySummaryState::ColdStartInvalid),
               "HISTORY_SUMMARY_INVALID");
  EXPECT_STREQ(toString(HistorySummaryState::BuildInvalid),
               "HISTORY_SUMMARY_BUILD_INVALID");
  EXPECT_STREQ(toString(HistorySummaryState::CapacityExceeded),
               "HISTORY_CAPACITY_EXCEEDED");
  EXPECT_FALSE(autoShrinkEnabled());
}

}  // namespace
}  // namespace uwb_imu_pl
