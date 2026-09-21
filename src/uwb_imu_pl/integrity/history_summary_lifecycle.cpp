// C1-c lifecycle decisions (see header for the design-freeze mapping).

#include "uwb_imu_pl/integrity/history_summary_lifecycle.hpp"

#include <cmath>

namespace uwb_imu_pl {

bool futureOnsetAllowsZeroColumns(std::size_t onset_epoch,
                                  std::size_t compressed_through_epoch) {
  // Strictly later: an onset exactly at the compressed boundary may already
  // have material inside the compressed range.
  return onset_epoch > compressed_through_epoch;
}

HistoryAbsorptionDecision absorbHistoryMode(
    const HistoryModeAbsorptionRequest& request) {
  HistoryAbsorptionDecision decision;
  if (!request.has_input_response || !request.has_detection_cost) {
    decision.reason =
        "mode cannot enter history: input response and detection cost must be "
        "eliminated together";
    return decision;
  }
  if (!request.has_event_identity) {
    decision.reason = "mode cannot enter history: no event identity";
    return decision;
  }
  if (!request.has_time_support) {
    decision.reason = "mode cannot enter history: no time support";
    return decision;
  }
  if (!(request.prior_probability_bound > 0.0) ||
      !std::isfinite(request.prior_probability_bound)) {
    decision.reason =
        "mode cannot enter history: probability basis is not positive";
    return decision;
  }
  decision.accepted = true;
  return decision;
}

bool sourceLeavingWindowKeepsMode(bool mode_epoch_inside_window,
                                  bool mode_present_in_history_basis) {
  // Moving out of the explicit window is not a deletion: the mode must remain
  // represented in the history basis for its whole span.  While the epoch is
  // still explicit the question does not arise.
  return mode_epoch_inside_window || mode_present_in_history_basis;
}

HistoryClearanceDecision evaluateHistoryClearance(
    const HistoryClearanceEvidence& evidence) {
  HistoryClearanceDecision decision;
  if (!(evidence.tolerance >= 0.0) || !std::isfinite(evidence.tolerance)) {
    decision.reason = "clearance evidence has no admissible tolerance";
    return decision;
  }
  const bool response_zero = evidence.response_proven_zero ||
                             (std::isfinite(evidence.response_norm) &&
                              evidence.response_norm <= evidence.tolerance);
  const bool detector_zero = evidence.detector_proven_zero ||
                             (std::isfinite(evidence.detector_norm) &&
                              evidence.detector_norm <= evidence.tolerance);
  if (!response_zero || !detector_zero) {
    decision.reason =
        "fault end is not clearance: response and detection content are not "
        "both proven zero";
    return decision;
  }
  decision.cleared = true;
  return decision;
}

HistoryRetirementDecision evaluateHistoryRetirement(
    const HistoryRetirementRequest& request) {
  HistoryRetirementDecision decision;
  if (request.clean_reset) {
    // A genuine clean reset rebuilds everything consistently; it does not
    // pretend the retired content was proven harmless.
    decision.retire = true;
    decision.reason = "clean reset";
    return decision;
  }
  if (!request.no_impact_proof) {
    decision.reason =
        "retirement refused: no proof that the removed content cannot affect "
        "future protection";
    return decision;
  }
  if (!request.envelope_covers) {
    decision.reason =
        "retirement refused: no conservative envelope covering the removed "
        "content";
    return decision;
  }
  if (!request.risk_handled) {
    decision.reason = "retirement refused: removed risk is not handled";
    return decision;
  }
  decision.retire = true;
  decision.reason = "no-impact proof + envelope + risk handling";
  return decision;
}

HistoryHotSwitchDecision evaluateHistoryHotSwitch(
    const HistoryHotSwitchRequest& request) {
  HistoryHotSwitchDecision decision;
  if (request.active_mode_set == request.requested_mode_set) {
    decision.accepted = true;
    decision.reason = "mode set unchanged";
    return decision;
  }
  if (request.rebuild_available) {
    decision.accepted = false;
    decision.rebuild_required = true;
    decision.reason =
        "mode set change requires a rebuild: the old summary is invalidated";
    return decision;
  }
  decision.reason =
      "hot switch of order/family refused: changed mode set without a rebuild";
  return decision;
}

const char* toString(HistorySummaryState state) {
  switch (state) {
    case HistorySummaryState::NotRequired:
      return "HISTORY_SUMMARY_NOT_REQUIRED";
    case HistorySummaryState::Valid:
      return "HISTORY_SUMMARY_VALID";
    case HistorySummaryState::ColdStartInvalid:
      return "HISTORY_SUMMARY_INVALID";
    case HistorySummaryState::BuildInvalid:
      return "HISTORY_SUMMARY_BUILD_INVALID";
    case HistorySummaryState::CapacityExceeded:
      return "HISTORY_CAPACITY_EXCEEDED";
  }
  return "HISTORY_SUMMARY_INVALID";
}

}  // namespace uwb_imu_pl
