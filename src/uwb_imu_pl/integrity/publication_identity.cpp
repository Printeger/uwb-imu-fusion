// C4 implementation: §8.6 identity binding, state machine and the
// watchdog/freshness separation.

#include "uwb_imu_pl/integrity/publication_identity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
      a.scope_digest != b.scope_digest ||
      a.history_summary_id != b.history_summary_id ||
      a.risk_proof_id != b.risk_proof_id ||
      a.pl_detector_certificate_id != b.pl_detector_certificate_id ||
      a.frame_id != b.frame_id ||
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

// ---------------------------------------------------------------------------
// C4/W2 production wiring: the explicit publication state holder.
// ---------------------------------------------------------------------------

PublicationLimits offlineReplayPublicationLimits() {
  PublicationLimits limits;
  // ~73 years: the wall-timeout branch stays wired and reported but cannot
  // fire in an offline replay, whatever the machine load.
  limits.wall_timeout_limit_ns = std::numeric_limits<std::int64_t>::max() / 4;
  return limits;
}

namespace {

std::string joinFieldNames(const std::vector<std::string>& fields) {
  std::string out;
  for (const auto& field : fields) {
    if (!out.empty()) out += ';';
    out += field;
  }
  return out;
}

}  // namespace

// Identity elements the pipeline must be able to name before a certificate can
// exist.  A zero/empty value means "production did not supply this element":
// the field is reported, never replaced by a placeholder.
std::vector<std::string> missingPublicationIdentityFields(
    const PublicationIdentity& id) {
  std::vector<std::string> missing;
  if (id.snapshot_id == 0) missing.push_back("snapshot_id");
  if (id.state_solution_id == 0) missing.push_back("state_solution_id");
  if (id.history_summary_id == 0) missing.push_back("history_summary_id");
  if (id.manifest_digest == 0) missing.push_back("manifest_digest");
  if (id.scope_digest == 0) missing.push_back("scope_digest");
  if (id.health_state == 0) missing.push_back("health_state");
  if (id.detector_ids.empty()) missing.push_back("detector_ids");
  if (id.risk_proof_id == 0) missing.push_back("risk_proof_id");
  if (id.pl_detector_certificate_id == 0)
    missing.push_back("pl_detector_certificate_id");
  if (!id.protection_level_m.allFinite()) missing.push_back("protection_level_m");
  if (id.position_reference.empty()) missing.push_back("position_reference");
  if (id.timestamp_ns == 0) missing.push_back("timestamp_ns");
  if (id.frame_id.empty()) missing.push_back("frame_id");
  return missing;
}

const WatchdogState& PublicationController::beginAttempt(
    const ClockSample& sample) {
  last_sample_jumped_ = sample.replay_clock_jumped;
  const bool had_previous = watchdog_.valid;
  const std::int64_t previous_sensor_ns = watchdog_.last_sensor_ns;
  const std::int64_t previous_wall_ns = watchdog_.last_wall_ns;
  watchdog_ = updateWatchdog(watchdog_, sample, limits_.sensor_stale_limit_ns,
                             limits_.wall_timeout_limit_ns);
  last_sensor_delta_ns_ =
      had_previous ? watchdog_.last_sensor_ns - previous_sensor_ns : 0;
  const std::int64_t wall_delta =
      had_previous ? watchdog_.last_wall_ns - previous_wall_ns : 0;
  // FROZEN data: the sensor timestamp did not move at all while the wall clock
  // advanced, and the accumulated divergence passed the staleness limit.  A
  // stream that merely lags behind the wall clock (sensor delta > 0) is NOT
  // refused here: its attempts consume the freshest data available, and
  // refusing them would drop fresh input instead of protecting anything.  The
  // unfiltered module verdict stays visible in the diagnosis.
  const bool frozen = !sample.replay_clock_jumped && wall_delta > 0 &&
      last_sensor_delta_ns_ == 0 &&
      watchdog_.sensor_lag_ns > limits_.sensor_stale_limit_ns;
  watchdog_refused_ =
      !watchdog_.valid || frozen || watchdog_.wall_timeout;
  if (watchdog_refused_) {
    // The freshness channel alone already decides UNAVAILABLE: this happens
    // before the caller enters the heavy FDE.
    dropToUnavailable();
    return watchdog_;
  }
  // Open the attempt on the white-listed state machine.  READY/PROTECTED enter
  // EVALUATING; UNAVAILABLE enters the recovery chain at FDE_EVALUATING.  A
  // leftover FDE_EVALUATING is an attempt that aborted before its synchronous
  // chain completed and is kept as-is (legal to continue or to drop).
  if (state_ == PublicationState::Ready ||
      state_ == PublicationState::Protected) {
    (void)request(PublicationState::Evaluating);
  } else if (state_ == PublicationState::Unavailable) {
    (void)request(PublicationState::FdeEvaluating);
  }
  return watchdog_;
}

StateTransition PublicationController::request(PublicationState next) {
  const StateTransition step = advancePublicationState(state_, next);
  if (step.accepted) state_ = step.next;
  return step;
}

void PublicationController::dropToUnavailable() {
  // EXACTLY the white-listed downgrade paths, no shortcut:
  //   EVALUATING -> UNAVAILABLE, FDE_EVALUATING -> UNAVAILABLE,
  //   READY -> EVALUATING -> UNAVAILABLE,
  //   PROTECTED -> EVALUATING -> UNAVAILABLE (a protected output may be
  //   downgraded at any time).
  switch (state_) {
    case PublicationState::Unavailable:
      return;
    case PublicationState::Evaluating:
    case PublicationState::FdeEvaluating:
      (void)request(PublicationState::Unavailable);
      return;
    case PublicationState::Ready:
    case PublicationState::Protected:
      (void)request(PublicationState::Evaluating);
      (void)request(PublicationState::Unavailable);
      return;
    case PublicationState::VerifiedCandidate:
    case PublicationState::AtomicCommit:
      // The commit chain is synchronous inside finalizeAttempt, so these states
      // are not reachable between attempts.  If one is observed the state
      // machine is left untouched (no illegal reset, no bypass commit) and the
      // caller records the refusal.
      return;
  }
}

PublicationDiagnosis PublicationController::watchdogDiagnosis() const {
  PublicationDiagnosis out;
  out.watchdog_valid = watchdog_.valid;
  out.wall_elapsed_ns = watchdog_.wall_elapsed_ns;
  out.sensor_elapsed_ns = watchdog_.sensor_elapsed_ns;
  out.sensor_delta_ns = last_sensor_delta_ns_;
  out.sensor_lag_ns = watchdog_.sensor_lag_ns;
  out.sensor_stale = watchdog_.sensor_stale;
  out.wall_timeout = watchdog_.wall_timeout;
  out.replay_clock_jumped = last_sample_jumped_;
  out.clock_refused = !watchdog_.valid;
  out.watchdog_reason = watchdog_.reason;
  return out;
}

PublicationDiagnosis PublicationController::finalizeAttempt(
    const PublicationIdentity& candidate, const PublicationIdentity& certificate,
    bool certificate_available, bool certification_available) {
  if (watchdog_refused_) {
    // The watchdog decided UNAVAILABLE before any identity work; the identity
    // check is honestly reported as NOT_EVALUATED instead of implied to pass.
    PublicationDiagnosis out = watchdogDiagnosis();
    out.gate_executed = false;
    out.identity_check = "NOT_EVALUATED";
    out.identity_reason =
        "watchdog refused the attempt before the identity check";
    out.refusal = watchdog_.reason;
    out.unprotected_output = true;
    out.protected_output = false;
    out.state_before = toString(state_);
    out.state_after = toString(state_);
    out.transition = out.state_before + " -> " + out.state_after;
    out.transition_accepted = false;
    return out;
  }

  PublicationDiagnosis out = watchdogDiagnosis();
  out.gate_executed = true;
  out.state_before = toString(state_);

  const std::vector<std::string> missing =
      missingPublicationIdentityFields(candidate);
  out.missing_identity_fields = missing;
  const bool have_certificate =
      certificate_available && certificate.certificate_id != 0;

  IdentityCheck check;
  if (!have_certificate) {
    check.reason =
        "no publication certificate was issued for this attempt";
  } else {
    // Production passes no centre-shift bound: the pipeline has no proven
    // centre-shift path, and a difference of two estimates is NOT a proof.
    check = checkPublicationIdentity(candidate, certificate, -1.0);
  }

  auto record_transition = [&](PublicationState next) {
    if (state_ == next) {
      // Already there through a documented downgrade path: nothing to move.
      out.transition_accepted = true;
      out.transition = std::string(toString(state_)) + " -> " + toString(next);
      return true;
    }
    const StateTransition step = request(next);
    out.transition_accepted = step.accepted;
    out.transition = out.state_before + " -> " + toString(step.next);
    if (!step.accepted) out.transition = step.reason;
    return step.accepted;
  };
  auto refuse = [&](const std::string& why) {
    out.identity_check = "REFUSED";
    out.identity_reason = why;
    out.refusal = why;
    out.protected_output = false;
    out.unprotected_output = true;
    dropToUnavailable();
    record_transition(PublicationState::Unavailable);
    out.state_after = toString(state_);
    return out;
  };
  auto release_unprotected = [&](const std::string& what) {
    out.identity_check = "REQUIRES_PROPAGATION";
    out.identity_reason = what;
    out.refusal = what;
    out.protected_output = false;
    out.unprotected_output = true;
    // The old certificate is NOT reused at a different instant.
    dropToUnavailable();
    record_transition(PublicationState::Unavailable);
    out.state_after = toString(state_);
    return out;
  };
  auto admit = [&](const std::string& what, bool rebind) {
    out.identity_check = rebind ? "SAME_TIME_REBIND" : "ADMISSIBLE";
    out.identity_reason = what;
    if (!certification_available) {
      // The identity proves the binding, but the platform may not certify a
      // protected output yet (Gate J): the output is published unprotected and
      // the reason is recorded instead of being hidden.
      out.refusal =
          "identity admissible but platform certification is incomplete "
          "(IMPLEMENTED_UNVERIFIED; Gate J pending): published as unprotected "
          "output";
      out.protected_output = false;
      out.unprotected_output = true;
      dropToUnavailable();
      record_transition(PublicationState::Unavailable);
      out.state_after = toString(state_);
      return out;
    }
    // Atomic chain only: VERIFIED_CANDIDATE -> ATOMIC_COMMIT -> PROTECTED.
    bool ok = true;
    if (state_ == PublicationState::FdeEvaluating) {
      ok = ok && record_transition(PublicationState::VerifiedCandidate);
      ok = ok && record_transition(PublicationState::AtomicCommit);
    }
    ok = ok && record_transition(PublicationState::Protected);
    out.transition_accepted = ok;
    out.state_after = toString(state_);
    out.protected_output = ok;
    out.unprotected_output = !ok;
    if (!ok) out.refusal = "publication state machine refused the commit chain";
    if (ok) {
      certificate_ = candidate;
      // The certificate identity is the admitted one; its id is carried on.
      certificate_.certificate_id = certificate.certificate_id;
      has_certificate_ = true;
    }
    return out;
  };

  if (!have_certificate) {
    std::string why = check.reason;
    if (!missing.empty()) {
      why += "; missing identity: " + joinFieldNames(missing);
    }
    return refuse(why);
  }
  if (check.admissible) {
    return admit(check.reason, check.same_time_rebind);
  }
  if (check.requires_propagation) {
    // No propagation proof path exists in production: the module's own refusal
    // is used (a difference of two estimates is not a proof) and the output is
    // published explicitly unprotected.
    const PropagationProof absent;
    const IdentityCheck propagated =
        propagateToTime(candidate, certificate, absent);
    return release_unprotected(propagated.reason);
  }
  std::string why = check.reason;
  if (!missing.empty()) {
    why += "; missing identity: " + joinFieldNames(missing);
  }
  return refuse(why);
}

}  // namespace uwb_imu_pl
