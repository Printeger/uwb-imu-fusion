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

}  // namespace uwb_imu_pl
