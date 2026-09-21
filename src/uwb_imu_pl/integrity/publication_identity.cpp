// C4 implementation: §8.6 identity binding, state machine and the
// watchdog/freshness separation.

#include "uwb_imu_pl/integrity/publication_identity.hpp"

#include <algorithm>
#include <cmath>

namespace uwb_imu_pl {

const char* toString(PublicationState state) {
  switch (state) {
    case PublicationState::Ready:
      return "READY";
    case PublicationState::Evaluating:
      return "EVALUATING";
    case PublicationState::Protected:
      return "PROTECTED";
    case PublicationState::Unavailable:
      return "UNAVAILABLE";
    case PublicationState::FdeEvaluating:
      return "FDE_EVALUATING";
    case PublicationState::VerifiedCandidate:
      return "VERIFIED_CANDIDATE";
    case PublicationState::AtomicCommit:
      return "ATOMIC_COMMIT";
  }
  return "UNKNOWN";
}

namespace {

// Fields that must match exactly for a certificate to describe the candidate.
bool sameStructuralIdentity(const PublicationIdentity& a,
                            const PublicationIdentity& b) {
  if (a.snapshot_id != b.snapshot_id || a.manifest_digest != b.manifest_digest ||
      a.history_summary_id != b.history_summary_id ||
      a.risk_proof_id != b.risk_proof_id || a.frame_id != b.frame_id ||
      a.position_reference != b.position_reference) {
    return false;
  }
  if (a.health_state != b.health_state) return false;
  if (a.detector_ids.size() != b.detector_ids.size()) return false;
  for (std::size_t index = 0; index < a.detector_ids.size(); ++index) {
    if (a.detector_ids[index] != b.detector_ids[index]) return false;
  }
  return true;
}

}  // namespace

IdentityCheck checkPublicationIdentity(const PublicationIdentity& candidate,
                                       const PublicationIdentity& certificate,
                                       double centre_shift_bound_m) {
  IdentityCheck out;
  if (certificate.certificate_id == 0) {
    out.reason = "certificate has no identity";
    return out;
  }
  if (!sameStructuralIdentity(candidate, certificate)) {
    out.reason =
        "snapshot/manifest/history/detector/risk/frame/reference identity "
        "mismatch: the certificate does not describe this candidate";
    return out;
  }
  if (candidate.timestamp_ns != certificate.timestamp_ns) {
    // A different instant needs the propagation proof; it is never patched by
    // the difference of two estimates.
    out.requires_propagation = true;
    out.reason =
        "certificate is at a different time point: a time-propagation proof is "
        "required (an estimate difference is not a proof)";
    return out;
  }
  if (candidate.state_solution_id == certificate.state_solution_id) {
    out.admissible = true;
    out.reason = "identity matches the certificate exactly";
    return out;
  }
  // Same time point, different solution id: only a proven centre shift may
  // rebind the certificate.
  if (!(centre_shift_bound_m >= 0.0) || !std::isfinite(centre_shift_bound_m)) {
    out.reason =
        "backend changed the solution at the same time point: a proven centre "
        "shift bound is required to rebind the certificate";
    return out;
  }
  out.admissible = true;
  out.same_time_rebind = true;
  out.reason =
      "same-time rebind with a proven centre shift bound of " +
      std::to_string(centre_shift_bound_m) + " m";
  return out;
}

IdentityCheck propagateToTime(const PublicationIdentity& candidate,
                              const PublicationIdentity& certificate,
                              const PropagationProof& proof) {
  IdentityCheck out;
  if (proof.available && proof.bound_m >= 0.0 && std::isfinite(proof.bound_m) &&
      proof.proof_id != 0) {
    IdentityCheck base =
        checkPublicationIdentity(candidate, certificate, -1.0);
    if (!sameStructuralIdentity(candidate, certificate)) {
      out.reason = base.reason;
      return out;
    }
    out.admissible = true;
    out.reason = "propagated with proof id " + std::to_string(proof.proof_id) +
                 " and bound " + std::to_string(proof.bound_m) + " m";
    return out;
  }
  out.requires_propagation = true;
  out.reason =
      "no admissible time-propagation proof: the certificate may not be reused "
      "at this instant";
  return out;
}

StateTransition advancePublicationState(PublicationState current,
                                        PublicationState requested) {
  StateTransition out;
  out.next = current;
  const bool legal =
      // nominal evaluation path
      (current == PublicationState::Ready &&
       requested == PublicationState::Evaluating) ||
      (current == PublicationState::Evaluating &&
       (requested == PublicationState::Protected ||
        requested == PublicationState::Unavailable)) ||
      // recovery path
      (current == PublicationState::Unavailable &&
       requested == PublicationState::FdeEvaluating) ||
      (current == PublicationState::FdeEvaluating &&
       (requested == PublicationState::VerifiedCandidate ||
        requested == PublicationState::Unavailable)) ||
      (current == PublicationState::VerifiedCandidate &&
       requested == PublicationState::AtomicCommit) ||
      (current == PublicationState::AtomicCommit &&
       requested == PublicationState::Protected) ||
      // a protected output may drop to evaluating/unavailable at any time
      (current == PublicationState::Protected &&
       (requested == PublicationState::Evaluating ||
        requested == PublicationState::Unavailable));
  if (!legal) {
    out.reason = std::string("illegal transition ") + toString(current) +
                 " -> " + toString(requested);
    return out;
  }
  out.accepted = true;
  out.next = requested;
  out.reason = std::string("accepted ") + toString(current) + " -> " +
               toString(requested);
  return out;
}

WatchdogState updateWatchdog(const WatchdogState& previous,
                             const ClockSample& sample,
                             std::int64_t sensor_stale_limit_ns,
                             std::int64_t wall_timeout_limit_ns) {
  WatchdogState out;
  if (sensor_stale_limit_ns <= 0 || wall_timeout_limit_ns <= 0) {
    out.reason = "watchdog limits must be positive";
    return out;
  }
  if (sample.wall_monotonic_ns < 0 || sample.sensor_timestamp_ns < 0) {
    out.reason = "clock samples must be non-negative";
    return out;
  }
  out.wall_elapsed_ns = previous.wall_elapsed_ns;
  out.sensor_elapsed_ns = previous.sensor_elapsed_ns;
  out.sensor_lag_ns = previous.sensor_lag_ns;
  out.last_wall_ns = sample.wall_monotonic_ns;
  out.last_sensor_ns = sample.sensor_timestamp_ns;
  if (previous.valid) {
    // Wall-time statistics advance only on a monotonic, non-jumped sample: a
    // replay /clock jump or pause must not distort them.
    std::int64_t wall_delta = 0;
    if (!sample.replay_clock_jumped) {
      wall_delta = sample.wall_monotonic_ns - previous.last_wall_ns;
      if (wall_delta < 0) {
        out.reason = "wall clock moved backwards without a replay marker";
        return out;
      }
      out.wall_elapsed_ns = previous.wall_elapsed_ns + wall_delta;
    }
    const std::int64_t sensor_delta =
        sample.sensor_timestamp_ns - previous.last_sensor_ns;
    if (sensor_delta < 0) {
      out.reason = "sensor timestamp moved backwards";
      return out;
    }
    out.sensor_elapsed_ns = previous.sensor_elapsed_ns + sensor_delta;
    // Lag accumulates when the wall clock runs ahead of the data; it shrinks
    // only when the sensor data advances faster (which cannot happen here by
    // more than the wall advance).  A replay jump contributes no wall advance,
    // so it cannot create artificial staleness either.
    out.sensor_lag_ns = std::max<std::int64_t>(
        0, previous.sensor_lag_ns + wall_delta - sensor_delta);
    out.sensor_stale = out.sensor_lag_ns > sensor_stale_limit_ns ||
                       sensor_delta > sensor_stale_limit_ns;
    out.wall_timeout =
        !sample.replay_clock_jumped && wall_delta > wall_timeout_limit_ns;
  }
  out.valid = true;
  if (sample.replay_clock_jumped) {
    out.reason =
        "replay clock jump/pause: wall-time watchdog statistics held, "
        "freshness judged on the sensor timestamp";
  } else if (out.sensor_stale) {
    out.reason = "data is stale by the sensor timestamp";
  } else if (out.wall_timeout) {
    out.reason = "wall-clock watchdog timeout";
  }
  return out;
}

}  // namespace uwb_imu_pl
