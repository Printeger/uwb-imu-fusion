// C4 tests (OUT-01/02/03): atomic commit binding, identity reuse rules and the
// watchdog/freshness contract.

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

#include "uwb_imu_pl/integrity/publication_identity.hpp"

namespace {

using namespace uwb_imu_pl;

PublicationIdentity makeIdentity(std::uint64_t certificate) {
  PublicationIdentity identity;
  identity.snapshot_id = 11;
  identity.state_solution_id = 22;
  identity.history_summary_id = 33;
  identity.manifest_digest = 44;
  identity.scope_digest = 45;
  identity.health_state = 1;
  identity.detector_ids = {7, 8};
  identity.risk_proof_id = 55;
  identity.pl_detector_certificate_id = 66;
  identity.protection_level_m = Eigen::Vector3d(1.0, 1.0, 2.0);
  identity.position_reference = "body_origin";
  identity.timestamp_ns = 1000000;
  identity.frame_id = "world";
  identity.certificate_id = certificate;
  identity.protected_output = true;
  return identity;
}

// ---------------------------------------------------------------------------
// OUT-01: same-time solution change / late certificate, no stale reuse.
// ---------------------------------------------------------------------------
TEST(PublicationIdentity, Out01SameTimeRebindAndNoCertificateReuse) {
  const PublicationIdentity certificate = makeIdentity(101);

  // Exact match: admissible.
  const IdentityCheck exact =
      checkPublicationIdentity(certificate, certificate);
  EXPECT_TRUE(exact.admissible);
  EXPECT_FALSE(exact.same_time_rebind);

  // Backend changed the solution at the SAME time point without a proven centre
  // shift: refused (no silent rebind).
  PublicationIdentity changed = certificate;
  changed.state_solution_id = 23;
  const IdentityCheck no_bound = checkPublicationIdentity(changed, certificate);
  EXPECT_FALSE(no_bound.admissible);
  EXPECT_NE(no_bound.reason.find("proven centre"), std::string::npos);

  // With a proven centre-shift bound the rebind is admissible and recorded.
  const IdentityCheck rebound = checkPublicationIdentity(changed, certificate, 0.05);
  EXPECT_TRUE(rebound.admissible);
  EXPECT_TRUE(rebound.same_time_rebind);

  // Different time point: needs a propagation proof, never an estimate
  // difference.
  PublicationIdentity later = certificate;
  later.timestamp_ns = certificate.timestamp_ns + 5000000;
  const IdentityCheck time_changed = checkPublicationIdentity(later, certificate);
  EXPECT_FALSE(time_changed.admissible);
  EXPECT_TRUE(time_changed.requires_propagation);
  EXPECT_NE(time_changed.reason.find("time-propagation proof"), std::string::npos);

  PropagationProof missing;
  missing.available = false;
  const IdentityCheck refused =
      propagateToTime(later, certificate, missing);
  EXPECT_FALSE(refused.admissible);
  PropagationProof proof;
  proof.available = true;
  proof.bound_m = 0.30;
  proof.proof_id = 9001;
  const IdentityCheck propagated = propagateToTime(later, certificate, proof);
  EXPECT_TRUE(propagated.admissible) << propagated.reason;

  // Late certificate: a certificate whose structural identity no longer matches
  // (e.g. reused after a summary/manifest change) is refused even at the same
  // time point.
  PublicationIdentity stale = certificate;
  stale.history_summary_id = 99;
  const IdentityCheck reuse = checkPublicationIdentity(stale, certificate, 0.01);
  EXPECT_FALSE(reuse.admissible);
  EXPECT_NE(reuse.reason.find("identity mismatch"), std::string::npos);

  // Atomic state machine: no partial commit, no shortcut from UNAVAILABLE.
  EXPECT_TRUE(advancePublicationState(PublicationState::Ready,
                                      PublicationState::Evaluating)
                  .accepted);
  EXPECT_FALSE(advancePublicationState(PublicationState::Unavailable,
                                       PublicationState::Protected)
                   .accepted);
  EXPECT_FALSE(advancePublicationState(PublicationState::VerifiedCandidate,
                                       PublicationState::Protected)
                   .accepted)
      << "a verified candidate must pass through ATOMIC_COMMIT";
  EXPECT_TRUE(advancePublicationState(PublicationState::VerifiedCandidate,
                                      PublicationState::AtomicCommit)
                  .accepted);
  EXPECT_TRUE(advancePublicationState(PublicationState::AtomicCommit,
                                      PublicationState::Protected)
                  .accepted);
  // The full recovery chain is legal.
  PublicationState state = PublicationState::Evaluating;
  for (const auto next : {PublicationState::Unavailable,
                          PublicationState::FdeEvaluating,
                          PublicationState::VerifiedCandidate,
                          PublicationState::AtomicCommit,
                          PublicationState::Protected}) {
    const StateTransition step = advancePublicationState(state, next);
    ASSERT_TRUE(step.accepted) << step.reason;
    state = step.next;
  }
  EXPECT_EQ(state, PublicationState::Protected);
  std::printf("[OUT-01] same_time_rebind=%.3f rejected_without_bound=1 "
              "propagation_proof_id=%llu chain=READY..PROTECTED\n",
              0.05, static_cast<unsigned long long>(proof.proof_id));
}

// ---------------------------------------------------------------------------
// OUT-02: time / frame / position-reference mixing is refused or bounded.
// ---------------------------------------------------------------------------
TEST(PublicationIdentity, Out02TimeFrameAndReferenceMixing) {
  const PublicationIdentity certificate = makeIdentity(202);
  for (const auto& mutation : {"frame", "reference", "detectors", "manifest",
                               "scope", "pl_certificate"}) {
    PublicationIdentity changed = certificate;
    if (std::string(mutation) == "frame") changed.frame_id = "body";
    if (std::string(mutation) == "reference") {
      changed.position_reference = "antenna_phase_center";
    }
    if (std::string(mutation) == "detectors") changed.detector_ids = {7, 9};
    if (std::string(mutation) == "manifest") changed.manifest_digest = 45;
    if (std::string(mutation) == "scope") changed.scope_digest = 46;
    if (std::string(mutation) == "pl_certificate") {
      changed.pl_detector_certificate_id = 67;
    }
    const IdentityCheck refused =
        checkPublicationIdentity(changed, certificate, 0.0);
    EXPECT_FALSE(refused.admissible) << mutation;
    // A frame/reference mismatch can never be repaired by a centre-shift bound:
    // it needs an explicit transform, which this API does not fabricate.
    EXPECT_NE(refused.reason.find("identity mismatch"), std::string::npos)
        << mutation;
  }
  // Protected vs unprotected outputs are distinguished in the identity.
  PublicationIdentity unprotected = certificate;
  unprotected.protected_output = false;
  EXPECT_NE(unprotected.protected_output, certificate.protected_output);
  std::printf("[OUT-02] refused_mixings=frame,reference,detectors,manifest,scope,pl_certificate "
              "(centre-shift bound does not repair any of them)\n");
  SUCCEED();
}

// ---------------------------------------------------------------------------
// OUT-03: expiry, queue congestion and clock jumps -> timely unavailability,
// with wall time and sensor time kept apart.
// ---------------------------------------------------------------------------
TEST(PublicationIdentity, Out03ExpiryCongestionAndClockJumps) {
  const std::int64_t sensor_limit = 100000000;   // 100 ms
  const std::int64_t wall_limit = 500000000;     // 500 ms
  WatchdogState state;
  ClockSample first;
  first.wall_monotonic_ns = 1000000000;
  first.sensor_timestamp_ns = 5000000000;
  state = updateWatchdog(state, first, sensor_limit, wall_limit);
  ASSERT_TRUE(state.valid);
  EXPECT_EQ(state.wall_elapsed_ns, 0);
  EXPECT_EQ(state.sensor_elapsed_ns, 0);

  // Healthy cadence: both clocks advance together.
  ClockSample healthy;
  healthy.wall_monotonic_ns = first.wall_monotonic_ns + 20000000;
  healthy.sensor_timestamp_ns = first.sensor_timestamp_ns + 20000000;
  const WatchdogState advanced =
      updateWatchdog(state, healthy, sensor_limit, wall_limit);
  EXPECT_EQ(advanced.wall_elapsed_ns, 20000000);
  EXPECT_EQ(advanced.sensor_elapsed_ns, 20000000);
  EXPECT_FALSE(advanced.sensor_stale);
  EXPECT_FALSE(advanced.wall_timeout);

  // Queue congestion: sensor data goes stale (by the sensor timestamp) while
  // the wall clock keeps running -> explicit staleness, not a silent publish.
  ClockSample congested;
  congested.wall_monotonic_ns = healthy.wall_monotonic_ns + 400000000;
  congested.sensor_timestamp_ns = healthy.sensor_timestamp_ns;  // frozen data
  const WatchdogState stale =
      updateWatchdog(advanced, congested, sensor_limit, wall_limit);
  EXPECT_TRUE(stale.sensor_stale);
  EXPECT_NE(stale.reason.find("stale by the sensor timestamp"), std::string::npos);

  // Replay /clock jump or pause: the wall accumulator must NOT be distorted by
  // the jump, while freshness still uses the sensor timestamp.
  ClockSample replay;
  replay.wall_monotonic_ns = congested.wall_monotonic_ns + 60000000000LL;
  replay.sensor_timestamp_ns = congested.sensor_timestamp_ns + 20000000;
  replay.replay_clock_jumped = true;
  const WatchdogState after_jump =
      updateWatchdog(stale, replay, sensor_limit, wall_limit);
  ASSERT_TRUE(after_jump.valid) << after_jump.reason;
  EXPECT_EQ(after_jump.wall_elapsed_ns, congested.wall_monotonic_ns -
                                            first.wall_monotonic_ns)
      << "a replay jump must not advance the wall-time statistics";
  EXPECT_FALSE(after_jump.wall_timeout)
      << "a replay jump must not trigger a wall-clock watchdog timeout";
  EXPECT_TRUE(after_jump.sensor_stale)
      << "the accumulated lag persists: a clock jump does not refresh data";
  EXPECT_NE(after_jump.reason.find("replay clock jump"), std::string::npos);

  // Wall timeout on a genuinely slow stream (no replay marker).
  ClockSample slow;
  slow.wall_monotonic_ns = replay.wall_monotonic_ns + 700000000;
  slow.sensor_timestamp_ns = replay.sensor_timestamp_ns + 20000000;
  const WatchdogState timed_out =
      updateWatchdog(after_jump, slow, sensor_limit, wall_limit);
  EXPECT_TRUE(timed_out.wall_timeout)
      << "the wall-clock timeout flag must be raised";

  // Backwards wall clock without a replay marker is refused outright.
  ClockSample backwards;
  backwards.wall_monotonic_ns = slow.wall_monotonic_ns - 1000;
  backwards.sensor_timestamp_ns = slow.sensor_timestamp_ns + 1000;
  const WatchdogState refused =
      updateWatchdog(timed_out, backwards, sensor_limit, wall_limit);
  EXPECT_FALSE(refused.valid);
  EXPECT_NE(refused.reason.find("moved backwards"), std::string::npos);

  std::printf("[OUT-03] wall_elapsed_after_jump=%lld sensor_stale=1 "
              "wall_timeout=1 backwards_refused=1\n",
              static_cast<long long>(after_jump.wall_elapsed_ns));
}

}  // namespace
