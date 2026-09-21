#pragma once

// C1-c lifecycle decisions for the fault-preserving history summary
// (design freeze history-summary-design.md §7.6 rows 1-8, §6 cold start).
//
// Every function is a pure, fail-closed decision: a rule that cannot be
// established refuses (or requires a rebuild / a protected stop) instead of
// silently weakening protection.  Nothing here changes a threshold; the
// functions only decide whether a mode may be created / absorbed / cleared /
// retired / hot-switched, and what the explicit state of the summary is.
//
// Status: C1-c module implemented; consumed by the Part C pipeline
// (`incremental_estimator.cpp` boundary segment) and locked by
// `HistorySummaryLifecycle.*` tests.

#include <cstdint>
#include <string>

namespace uwb_imu_pl {

// ---------------------------------------------------------------------------
// Row 1: future onset parameter.
// ---------------------------------------------------------------------------
// A mode whose physical onset is genuinely later than the data already
// compressed into the summary may use zero fault columns (it has no material
// in the history yet).  An onset at or before the compressed horizon must
// NOT be represented by zero columns.
bool futureOnsetAllowsZeroColumns(std::size_t onset_epoch,
                                  std::size_t compressed_through_epoch);

// ---------------------------------------------------------------------------
// Row 2: a mode entering the history.
// ---------------------------------------------------------------------------
// The input response and the detection cost are eliminated together: a mode
// that cannot carry both (response block + detector content) is refused, and
// the event identity / time support / probability basis must be present.
struct HistoryModeAbsorptionRequest {
  bool has_input_response = false;  // T_b-style response available
  bool has_detection_cost = false;  // F_b/d_perp content available
  bool has_event_identity = false;  // event id present
  bool has_time_support = false;    // finite [begin, end) support
  double prior_probability_bound = 0.0;
};

struct HistoryAbsorptionDecision {
  bool accepted = false;
  std::string reason;
};

HistoryAbsorptionDecision absorbHistoryMode(
    const HistoryModeAbsorptionRequest& request);

// ---------------------------------------------------------------------------
// Row 3: a source leaving the window does not delete its history mode.
// ---------------------------------------------------------------------------
// The mode must remain in the history basis for every epoch it covers; the
// caller passes whether the mode is still represented there.  Leaving the
// window (moving from explicit blocks to the condensed boundary) is not a
// deletion.
bool sourceLeavingWindowKeepsMode(bool mode_epoch_inside_window,
                                  bool mode_present_in_history_basis);

// ---------------------------------------------------------------------------
// Row 4: fault end != clearance.
// ---------------------------------------------------------------------------
// Clearing a mode requires positive evidence that BOTH its response and its
// detection content are zero (the residuals really stopped carrying it), not
// merely that the injection was switched off.
struct HistoryClearanceEvidence {
  double response_norm = 0.0;  // ||T_b column(s)|| at the evaluation point
  double detector_norm = 0.0;  // ||F_b column(s)|| / ||d_perp|| content
  double tolerance = 0.0;      // admissibility floor (not a detection gate)
  bool response_proven_zero = false;  // independent argument for T_b = 0
  bool detector_proven_zero = false;  // independent argument for F_b = 0
};

struct HistoryClearanceDecision {
  bool cleared = false;
  std::string reason;
};

HistoryClearanceDecision evaluateHistoryClearance(
    const HistoryClearanceEvidence& evidence);

// ---------------------------------------------------------------------------
// Row 5: retiring a history mode.
// ---------------------------------------------------------------------------
// Retiring (deleting) a mode requires a no-impact proof that also covers
// future protection, a conservative envelope covering the removed content,
// and a recorded risk handling -- or a genuinely clean reset.
struct HistoryRetirementRequest {
  bool no_impact_proof = false;
  bool envelope_covers = false;
  bool risk_handled = false;
  bool clean_reset = false;
};

struct HistoryRetirementDecision {
  bool retire = false;
  bool requires_rebuild = false;
  std::string reason;
};

HistoryRetirementDecision evaluateHistoryRetirement(
    const HistoryRetirementRequest& request);

// ---------------------------------------------------------------------------
// Row 6: order/family hot switch.
// ---------------------------------------------------------------------------
// Changing the monitored mode set at runtime is refused by default; the only
// sound paths are an unchanged set (no-op) or an explicit rebuild.  A changed
// set must never be served from a stale summary.
struct HistoryHotSwitchRequest {
  std::uint64_t active_mode_set = 0;  // digest of the summary's mode set
  std::uint64_t requested_mode_set = 0;
  bool rebuild_available = false;
};

struct HistoryHotSwitchDecision {
  bool accepted = false;
  bool rebuild_required = false;
  std::string reason;
};

HistoryHotSwitchDecision evaluateHistoryHotSwitch(
    const HistoryHotSwitchRequest& request);

// ---------------------------------------------------------------------------
// Rows 7/8 + cold start: explicit summary state.
// ---------------------------------------------------------------------------
// HistorySummaryInvalid is the cold-start / deleted-material state: there is
// original history (boundary input rows) but no valid summary can be
// established.  Reconstructing detection content from a nominal marginal is
// forbidden, so the window is explicitly unusable (fail-closed) instead.
enum class HistorySummaryState {
  NotRequired = 0,       // no condensed material at all (empty boundary)
  Valid = 1,             // summary built and bound
  ColdStartInvalid = 2,  // history exists, summary cannot be established
  BuildInvalid = 3,      // the module refused the assembled system
  CapacityExceeded = 4,  // §7.6 row 7 REFUSE (or explicit RESET/STOP)
};

const char* toString(HistorySummaryState state);

// Row 8: `auto_shrink` stays off until its proof obligations are discharged.
// The repository has no setting point at all today, which is recorded as
// vacuous-safe by the evidence; this helper exists so the rule is testable.
constexpr bool autoShrinkEnabled() { return false; }

}  // namespace uwb_imu_pl
