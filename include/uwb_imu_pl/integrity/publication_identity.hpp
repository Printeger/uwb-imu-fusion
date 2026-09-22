#pragma once

// C4 (§8.6): atomic commit and publication identity.
//
// Contract (roadmap §8.6):
//
// * A published output is bound to the identity of everything it was computed
//   from: snapshot, state solution, history summary, manifest, health state,
//   detector set, risk proof, protection level, position reference, timestamp
//   and frame.  Nothing is published unless the certificate that carries those
//   identities still matches the input actually being published.
// * State machine: READY -> EVALUATING -> PROTECTED / UNAVAILABLE, and
//   UNAVAILABLE -> FDE_EVALUATING -> VERIFIED_CANDIDATE -> ATOMIC_COMMIT ->
//   PROTECTED.  Commit is atomic: a partially verified candidate never becomes
//   the published solution.
// * If the backend changes the solution at the *same* time point, the
//   certificate may be rebound only through a proven centre-shift bound.  A
//   solution at a *different* time point requires a time-propagation proof; a
//   difference of two estimates is NOT a proof.
// * Protected and unprotected outputs are distinguished in the published
//   contract; certificates may never be reused across time, frame or position
//   reference; watchdog uses a monotonic wall clock while data freshness uses
//   the sensor timestamp, so a replay `/clock` jump or pause cannot distort the
//   wall-time statistics.

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace uwb_imu_pl {

// ---------------------------------------------------------------------------
// Identity binding.
// ---------------------------------------------------------------------------
struct PublicationIdentity {
  std::uint64_t snapshot_id = 0;
  std::uint64_t state_solution_id = 0;
  std::uint64_t history_summary_id = 0;
  std::uint64_t manifest_digest = 0;
  std::uint64_t health_state = 0;
  std::vector<std::uint64_t> detector_ids;
  std::uint64_t risk_proof_id = 0;
  Eigen::Vector3d protection_level_m = Eigen::Vector3d::Zero();
  std::string position_reference;
  std::int64_t timestamp_ns = 0;
  std::string frame_id;
  // Certificate the identity was proven against.
  std::uint64_t certificate_id = 0;
  bool protected_output = false;
};

// Reasons a publication identity may be refused.
struct IdentityCheck {
  bool admissible = false;
  bool same_time_rebind = false;      // proven centre shift at the same time
  bool requires_propagation = false;  // different time point
  std::string reason;
};

// Checks a candidate identity against the certificate's identity.  When
// `centre_shift_bound_m` is non-negative the caller claims a proven centre
// shift at the same time point and the rebind is admissible; a time change is
// always refused (it needs the separate propagation proof below).
IdentityCheck checkPublicationIdentity(const PublicationIdentity& candidate,
                                       const PublicationIdentity& certificate,
                                       double centre_shift_bound_m = -1.0);

// A time-propagation proof: the certificate of an earlier instant may only be
// propagated when a documented bound for the intervening motion exists.
struct PropagationProof {
  bool available = false;
  double bound_m = 0.0;
  std::uint64_t proof_id = 0;
  std::string reason;
};

IdentityCheck propagateToTime(const PublicationIdentity& candidate,
                              const PublicationIdentity& certificate,
                              const PropagationProof& proof);

// ---------------------------------------------------------------------------
// State machine.
// ---------------------------------------------------------------------------
enum class PublicationState {
  Ready = 0,
  Evaluating = 1,
  Protected = 2,
  Unavailable = 3,
  FdeEvaluating = 4,
  VerifiedCandidate = 5,
  AtomicCommit = 6,
};

const char* toString(PublicationState state);

struct StateTransition {
  bool accepted = false;
  PublicationState next = PublicationState::Ready;
  std::string reason;
};

// Legal transitions only; anything else is refused (no partial commit, no
// going straight from UNAVAILABLE to PROTECTED).
StateTransition advancePublicationState(PublicationState current,
                                        PublicationState requested);

// ---------------------------------------------------------------------------
// Watchdog / freshness separation.
// ---------------------------------------------------------------------------
struct ClockSample {
  std::int64_t wall_monotonic_ns = 0;  // monotonic wall clock
  std::int64_t sensor_timestamp_ns = 0;
  bool replay_clock_jumped = false;     // /clock jump or pause detected
};

struct WatchdogState {
  bool valid = false;
  std::int64_t last_wall_ns = 0;      // last monotonic wall sample
  std::int64_t last_sensor_ns = 0;    // last sensor timestamp
  std::int64_t wall_elapsed_ns = 0;   // accumulated monotonic wall time
  std::int64_t sensor_elapsed_ns = 0; // accumulated sensor time
  // Accumulated divergence wall-delta minus sensor-delta: a stream whose data
  // is held/frozen while the wall clock advances is stale even though the last
  // sample-to-sample delta looks small.
  std::int64_t sensor_lag_ns = 0;
  bool sensor_stale = false;
  bool wall_timeout = false;
  std::string reason;
};

// Updates the watchdog with a new sample.  A replay `/clock` jump or pause must
// not distort the wall-time statistics: the wall clock stays monotonic, while
// staleness is judged on the sensor timestamp only.
WatchdogState updateWatchdog(const WatchdogState& previous,
                             const ClockSample& sample,
                             std::int64_t sensor_stale_limit_ns,
                             std::int64_t wall_timeout_limit_ns);

// ---------------------------------------------------------------------------
// C4/W2 production wiring: the explicit publication state holder.
// ---------------------------------------------------------------------------
//
// The pipeline drives the publication gate through ONE explicit holder object
// (never through a const path, a mutable member or a second state source):
//
//   beginAttempt(sample)      -- freshness/watchdog judgement, attempt entry
//   finalizeAttempt(...)      -- identity check + white-listed state machine
//
// The holder is the only place the publication state and the last admitted
// certificate live; the identity check itself stays a pure function of the
// two identities it compares.

// Watchdog limits used by the production attempt gate.  These are NEW policy
// defaults introduced with the C4 wiring (no existing threshold is modified);
// they are constructor-explicit so a scenario or test can state the policy it
// ran under.  The defaults are tied to the platform's own designed outage
// tolerance `bridge.max_duration_s = 1 s`: a UWB gap the bridge can still cover
// must never be classified as stale, so the staleness limit is twice that
// outage and the wall timeout is twice the staleness limit again.
struct PublicationLimits {
  std::int64_t sensor_stale_limit_ns = 2000000000;   // 2 s (2 x design outage)
  std::int64_t wall_timeout_limit_ns = 4000000000;   // 4 s (2 x stale limit)
};

// Limits for OFFLINE replay harnesses.  A replay run consumes pre-generated
// batches as fast as the machine can and its wall clock is not a real-time
// deadline source, so the wall-timeout judgement is disabled by construction
// while the freeze/vintage checks stay fully active.  The helper is named so
// every call site states the policy it ran under.
PublicationLimits offlineReplayPublicationLimits();

// Record of one publication-gate judgement.  Everything here is a measured or
// derived production value; a field that production cannot supply is reported
// through `missing_identity_fields` instead of being invented.
struct PublicationDiagnosis {
  bool gate_executed = false;
  // "ADMISSIBLE" / "SAME_TIME_REBIND" / "REQUIRES_PROPAGATION" / "REFUSED" /
  // "NOT_EVALUATED" (watchdog refused the attempt before any identity work).
  std::string identity_check = "NOT_EVALUATED";
  std::string identity_reason;
  std::string state_before;
  std::string state_after;
  std::string transition;
  bool transition_accepted = false;
  // What the consumer is allowed to assume about this output.
  bool protected_output = false;
  bool unprotected_output = true;
  std::string refusal;
  std::vector<std::string> missing_identity_fields;
  // Watchdog channel (independent of the FDE latency).
  bool watchdog_valid = false;
  std::int64_t wall_elapsed_ns = 0;
  std::int64_t sensor_elapsed_ns = 0;
  std::int64_t sensor_delta_ns = 0;   // sample-to-sample sensor advance
  std::int64_t sensor_lag_ns = 0;
  bool sensor_stale = false;
  bool wall_timeout = false;
  bool replay_clock_jumped = false;
  bool clock_refused = false;
  std::string watchdog_reason;
};

std::vector<std::string> missingPublicationIdentityFields(
    const PublicationIdentity& identity);

class PublicationController {
 public:
  explicit PublicationController(PublicationLimits limits = PublicationLimits{})
      : limits_(limits) {}

  // Attempt entry, before any heavy FDE work.  Opens the attempt on the state
  // machine (READY/PROTECTED -> EVALUATING, UNAVAILABLE -> FDE_EVALUATING) and
  // updates the watchdog with the caller's clock sample.
  const WatchdogState& beginAttempt(const ClockSample& sample);

  // True when the watchdog already decided UNAVAILABLE for this attempt.  The
  // refusal covers exactly the two cases where consuming the attempt cannot
  // publish anything trustworthy:
  //   * the data is FROZEN -- the sensor timestamp did not move at all while
  //     the wall clock advanced and the accumulated divergence exceeded the
  //     staleness limit (a stream that merely lags but still progresses is NOT
  //     refused: refusing fresh data would drop it), and
  //   * the wall clock timed out (the pipeline itself is not progressing).
  // A backwards clock is not a refusal here: it is refused the way the
  // platform refuses invalid batches (see watchdog().valid).
  bool watchdogRefused() const { return watchdog_refused_; }

  // Finalizes the attempt:
  //   candidate               -- identity assembled from the published output
  //   certificate             -- identity the attempt's proofs were issued for
  //   certificate_available   -- false: no certificate exists (fail-closed)
  //   certification_available -- the platform may certify protection (Gate J);
  //                              false keeps every output explicitly unprotected
  // The admitted branch walks VERIFIED_CANDIDATE -> ATOMIC_COMMIT -> PROTECTED;
  // no other path may reach PROTECTED.
  PublicationDiagnosis finalizeAttempt(const PublicationIdentity& candidate,
                                       const PublicationIdentity& certificate,
                                       bool certificate_available,
                                       bool certification_available);

  PublicationState state() const { return state_; }
  bool hasCertificate() const { return has_certificate_; }
  const PublicationIdentity& certificate() const { return certificate_; }
  const WatchdogState& watchdog() const { return watchdog_; }
  const PublicationLimits& limits() const { return limits_; }

 private:
  StateTransition request(PublicationState next);
  // Legal shortest path back to UNAVAILABLE (documented in the .cpp).
  void dropToUnavailable();
  PublicationDiagnosis watchdogDiagnosis() const;

  PublicationLimits limits_;
  PublicationState state_ = PublicationState::Ready;
  WatchdogState watchdog_;
  bool watchdog_refused_ = false;
  bool last_sample_jumped_ = false;
  std::int64_t last_sensor_delta_ns_ = 0;
  PublicationIdentity certificate_;
  bool has_certificate_ = false;
};

}  // namespace uwb_imu_pl
