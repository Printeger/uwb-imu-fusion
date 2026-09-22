// C4/W2 tests (OUT-04/OUT-05): the publication gate as wired into the
// production paths -- the explicit state holder, the watchdog channel judged
// before the heavy FDE, and the explicit unprotected marking of the stateless
// monitor outputs.

#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <gtest/gtest.h>

namespace {

uwb_imu_pl::UwbBatch makeBatch(uwb_imu_pl::TimestampNs timestamp,
                               const Eigen::Vector3d& position) {
  uwb_imu_pl::UwbBatch batch;
  batch.id = uwb_imu_pl::BatchId(timestamp.value());
  batch.timestamp = timestamp;
  batch.covariance_model_id = "test_diagonal";
  const std::vector<Eigen::Vector3d> anchors = {
      {-5, -5, 0}, {5, -5, 1}, {5, 5, 3}, {-5, 5, 4}, {0, -6, 2}};
  for (std::size_t i = 0; i < anchors.size(); ++i) {
    uwb_imu_pl::UwbMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(i + 1);
    measurement.factor_id = uwb_imu_pl::FactorId(i + 1);
    measurement.anchor_id = uwb_imu_pl::AnchorId(i + 1);
    measurement.timestamp = timestamp;
    measurement.anchor_position_m = anchors[i];
    measurement.range_m = (position - anchors[i]).norm();
    measurement.sigma_m = 0.1;
    batch.measurements.push_back(measurement);
  }
  return batch;
}

uwb_imu_pl::IntegrityConfig config() {
  uwb_imu_pl::IntegrityConfig config;
  config.incremental.relinearize_threshold = 0.1;
  config.incremental.relinearize_skip = 1;
  config.incremental.smoothness_sigma_m = 0.5;
  config.snapshot.rank_tolerance = 1e-10;
  config.snapshot.max_condition_number = 1e12;
  config.imu.accelerometer_sigma = 0.1;
  config.imu.gyroscope_sigma = 0.01;
  config.imu.accelerometer_bias_rw_sigma = 0.001;
  config.imu.gyroscope_bias_rw_sigma = 0.0001;
  config.imu.gravity_mps2 = 9.80665;
  config.imu.max_gap_s = 0.02;
  config.risk.p_fa = 1e-5;
  config.risk.p_hmi_total = 4e-5;
  config.risk.nominal_axis_tail = 1e-5;
  config.risk.p_nm = 1e-7;
  config.risk.horizontal_alert_limit_m = 100.0;
  config.risk.vertical_alert_limit_m = 100.0;
  for (std::uint64_t id = 1; id <= 5; ++id) {
    uwb_imu_pl::FaultHypothesis allocation;
    allocation.id = uwb_imu_pl::HypothesisId(id);
    allocation.anchor_id = uwb_imu_pl::AnchorId(id);
    allocation.missed_detection_allocation = 1e-3;
    allocation.prior_probability_bound = 1e-4;
    config.risk.hypotheses.push_back(allocation);
  }
  return config;
}

void primeEstimator(const uwb_imu_pl::IntegrityConfig& cfg,
                    uwb_imu_pl::IncrementalUwbImuEstimator* estimator) {
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = Eigen::Vector3d(0, 0, 1);
  estimator->initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  for (int index = 0; index <= 2; ++index) {
    uwb_imu_pl::ImuMeasurement sample;
    sample.id = uwb_imu_pl::MeasurementId(index);
    sample.timestamp = uwb_imu_pl::TimestampNs(index * 5000000);
    sample.specific_force_mps2 = Eigen::Vector3d(0, 0, cfg.imu.gravity_mps2);
    estimator->ingestImu(sample);
  }
}

uwb_imu_pl::PublicationIdentity makeIdentity(std::int64_t timestamp_ns,
                                             std::uint64_t solution_id) {
  uwb_imu_pl::PublicationIdentity identity;
  identity.snapshot_id = 11;
  identity.state_solution_id = solution_id;
  identity.history_summary_id = 33;
  identity.manifest_digest = 44;
  identity.health_state = 1;
  identity.detector_ids = {7, 8};
  identity.risk_proof_id = 55;
  identity.protection_level_m = Eigen::Vector3d(1.0, 1.0, 2.0);
  identity.position_reference = "body_origin";
  identity.timestamp_ns = timestamp_ns;
  identity.frame_id = "world";
  identity.certificate_id = 101;
  return identity;
}

uwb_imu_pl::ClockSample makeClock(std::int64_t wall_ns,
                                  std::int64_t sensor_ns) {
  uwb_imu_pl::ClockSample sample;
  sample.wall_monotonic_ns = wall_ns;
  sample.sensor_timestamp_ns = sensor_ns;
  sample.replay_clock_jumped = false;
  return sample;
}

}  // namespace

// ---------------------------------------------------------------------------
// OUT-04: the pipeline publication record is assembled from production values.
// ---------------------------------------------------------------------------
TEST(PublicationWiring, PipelineAttemptCarriesProductionIdentity) {
  const auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  primeEstimator(cfg, &estimator);
  uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          cfg.risk, cfg.snapshot.rank_tolerance,
          cfg.snapshot.max_condition_number));
  const auto output = pipeline.processUwbBatch(
      makeBatch(uwb_imu_pl::TimestampNs(10000000), Eigen::Vector3d(0, 0, 1)),
      makeClock(1000000000, 10000000));

  const auto& publication = output.publication;
  EXPECT_TRUE(publication.gate_executed);
  // Nothing may be published as protected while the platform is
  // IMPLEMENTED_UNVERIFIED; the output says so explicitly.
  EXPECT_FALSE(publication.protected_output);
  EXPECT_TRUE(publication.unprotected_output);
  EXPECT_FALSE(publication.refusal.empty());
  EXPECT_EQ(publication.state_after, "UNAVAILABLE");
  EXPECT_EQ(pipeline.publication().state(),
            uwb_imu_pl::PublicationState::Unavailable);
  // Identity elements come from the attempt itself.
  if (output.window_id != 0 && output.transaction_id != 0) {
    EXPECT_NE(publication.snapshot_id, 0u);
  }
  EXPECT_EQ(publication.state_solution_id, output.linearization_version);
  EXPECT_FALSE(publication.detector_ids.empty());
  EXPECT_NE(publication.detector_ids.find(output.detector.detector_type),
            std::string::npos);
  // A certificate exists only when every identity element is present; the
  // refusal names the missing ones instead of inventing them.
  if (publication.certificate_id == 0) {
    EXPECT_EQ(publication.identity_check, "REFUSED");
    EXPECT_FALSE(publication.missing_identity_fields.empty());
  } else {
    EXPECT_TRUE(publication.identity_check == "ADMISSIBLE" ||
                publication.identity_check == "SAME_TIME_REBIND" ||
                publication.identity_check == "REQUIRES_PROPAGATION");
  }
  // The state machine moved exactly through its white-listed transitions.
  EXPECT_FALSE(publication.transition.empty());
  std::printf("[OUT-04] check=%s state=%s missing=%s\n",
              publication.identity_check.c_str(),
              publication.state_after.c_str(),
              publication.missing_identity_fields.c_str());
}

// ---------------------------------------------------------------------------
// OUT-05: frozen data is decided UNAVAILABLE by the watchdog channel before
// the heavy FDE, and a backwards clock is refused like input validation.
// ---------------------------------------------------------------------------
TEST(PublicationWiring, FrozenDataIsUnavailableBeforeHeavyFde) {
  const auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  primeEstimator(cfg, &estimator);
  uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          cfg.risk, cfg.snapshot.rank_tolerance,
          cfg.snapshot.max_condition_number));
  const auto nominal = pipeline.processUwbBatch(
      makeBatch(uwb_imu_pl::TimestampNs(10000000), Eigen::Vector3d(0, 0, 1)),
      makeClock(1000000000, 10000000));
  EXPECT_FALSE(nominal.publication.sensor_stale);

  // A stream that LAGS the wall clock but still advances is not refused: its
  // attempt consumes the freshest data available, and refusing it would drop
  // fresh input.  (Measured on the mature offline scenario: the raw staleness
  // verdict would have refused ~80% of its attempts.)  The module verdict stays
  // visible in the record.
  const auto lagging = pipeline.processUwbBatch(
      makeBatch(uwb_imu_pl::TimestampNs(20000000), Eigen::Vector3d(0, 0, 1)),
      makeClock(4000000000, 20000000));
  EXPECT_GT(lagging.publication.sensor_lag_ns, 2000000000);
  EXPECT_TRUE(lagging.publication.sensor_stale);
  EXPECT_NE(lagging.diagnostics.status, "WATCHDOG_REFUSED");
  EXPECT_TRUE(lagging.publication.gate_executed);
  EXPECT_TRUE(lagging.diagnostics.transaction_opened);

  // Frozen data -- the sensor timestamp does not move at all while the wall
  // clock advances past the staleness limit -- is decided UNAVAILABLE by the
  // watchdog channel alone, before the heavy FDE.
  const auto before = estimator.audit();
  const auto frozen = pipeline.processUwbBatch(
      makeBatch(uwb_imu_pl::TimestampNs(20000000), Eigen::Vector3d(0, 0, 1)),
      makeClock(7000000000, 20000000));
  EXPECT_TRUE(frozen.publication.watchdog_valid);
  EXPECT_EQ(frozen.publication.sensor_delta_ns, 0);
  EXPECT_GT(frozen.publication.sensor_lag_ns, 2000000000);
  EXPECT_EQ(frozen.diagnostics.status, "WATCHDOG_REFUSED");
  EXPECT_FALSE(frozen.diagnostics.transaction_opened);
  EXPECT_FALSE(frozen.batch_committed);
  EXPECT_TRUE(frozen.stale_state);
  EXPECT_EQ(frozen.protection_level.availability,
            uwb_imu_pl::Availability::Unavailable);
  // The identity check did not run: the refusal is recorded as NOT_EVALUATED.
  EXPECT_FALSE(frozen.publication.gate_executed);
  EXPECT_EQ(frozen.publication.identity_check, "NOT_EVALUATED");
  EXPECT_TRUE(frozen.publication.unprotected_output);
  EXPECT_EQ(frozen.publication.state_after, "UNAVAILABLE");
  // No epoch was consumed and no graph state was touched: the heavy FDE never
  // ran for this attempt.
  const auto after = estimator.audit();
  EXPECT_EQ(before.epoch, after.epoch);
  EXPECT_EQ(before.factor_count, after.factor_count);
  EXPECT_TRUE(before.version == after.version);
  std::printf("[OUT-05] wall_elapsed_ns=%lld sensor_delta_ns=%lld "
              "sensor_lag_ns=%lld frozen=1\n",
              static_cast<long long>(frozen.publication.wall_elapsed_ns),
              static_cast<long long>(frozen.publication.sensor_delta_ns),
              static_cast<long long>(frozen.publication.sensor_lag_ns));
}

TEST(PublicationWiring, WatchdogClauseSemanticsAreExplicit) {
  // The gate refuses exactly two cases (frozen data past the limit, wall-clock
  // timeout); the two remaining module clauses are reported without refusing,
  // so a consumer can tell "no new data" from "data far ahead of the wall".
  uwb_imu_pl::PublicationLimits limits;
  uwb_imu_pl::PublicationController controller(limits);
  controller.beginAttempt(makeClock(1000000000, 10000000));
  // One large forward sensor gap: stale by the gap clause, not refused.
  controller.beginAttempt(makeClock(1100000000, 5000000000));
  EXPECT_TRUE(controller.watchdog().sensor_stale);
  EXPECT_FALSE(controller.watchdogRefused());
  // Frozen data long enough: refused.
  controller.beginAttempt(makeClock(6000000000, 5000000000));
  EXPECT_TRUE(controller.watchdogRefused());
  EXPECT_EQ(controller.state(), uwb_imu_pl::PublicationState::Unavailable);
  // A replay-marked jump holds the wall statistics and refuses nothing.
  uwb_imu_pl::PublicationController replay(limits);
  replay.beginAttempt(makeClock(1000000000, 10000000));
  uwb_imu_pl::ClockSample jump = makeClock(1000000000, 20000000);
  jump.replay_clock_jumped = true;
  replay.beginAttempt(jump);
  EXPECT_FALSE(replay.watchdogRefused());
  EXPECT_EQ(replay.watchdog().wall_elapsed_ns, 0);
}

TEST(PublicationWiring, BackwardsClockIsRefusedLikeInputValidation) {
  const auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  primeEstimator(cfg, &estimator);
  uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          cfg.risk, cfg.snapshot.rank_tolerance,
          cfg.snapshot.max_condition_number));
  (void)pipeline.processUwbBatch(
      makeBatch(uwb_imu_pl::TimestampNs(10000000), Eigen::Vector3d(0, 0, 1)),
      makeClock(1000000000, 10000000));
  const auto before = estimator.audit();
  // The sensor timestamp moves backwards without a replay marker: refused
  // exactly the way the platform's own stale-batch validation refuses it.
  EXPECT_THROW(pipeline.processUwbBatch(
                   makeBatch(uwb_imu_pl::TimestampNs(5000000),
                             Eigen::Vector3d(0, 0, 1)),
                   makeClock(1000100000, 5000000)),
               std::invalid_argument);
  EXPECT_FALSE(pipeline.publication().watchdog().valid);
  const auto after = estimator.audit();
  EXPECT_EQ(before.epoch, after.epoch);
  EXPECT_EQ(before.factor_count, after.factor_count);
}

// ---------------------------------------------------------------------------
// OUT-06: the explicit state holder owns the tri-state decision.
// ---------------------------------------------------------------------------
TEST(PublicationWiring, ControllerTriStateAndAtomicChain) {
  uwb_imu_pl::PublicationController controller;
  const auto certificate = makeIdentity(1000000, 22);

  // Exact match at the same time with the platform certification available:
  // the only path to PROTECTED walks the atomic chain.
  controller.beginAttempt(makeClock(1000, 1000000));
  auto admitted = controller.finalizeAttempt(certificate, certificate, true, true);
  EXPECT_EQ(admitted.identity_check, "ADMISSIBLE");
  EXPECT_TRUE(admitted.protected_output);
  EXPECT_EQ(admitted.state_after, "PROTECTED");
  EXPECT_TRUE(controller.hasCertificate());

  // A different instant needs a propagation proof; production has none, so the
  // output is published explicitly unprotected and the old certificate is NOT
  // reused.
  controller.beginAttempt(makeClock(2000, 2000000));
  auto later = makeIdentity(2000000, 22);
  auto propagated = controller.finalizeAttempt(later, certificate, true, true);
  EXPECT_EQ(propagated.identity_check, "REQUIRES_PROPAGATION");
  EXPECT_FALSE(propagated.protected_output);
  EXPECT_TRUE(propagated.unprotected_output);
  EXPECT_EQ(propagated.state_after, "UNAVAILABLE");
  EXPECT_EQ(controller.certificate().timestamp_ns, 1000000);

  // Same time, changed solution, no proven centre shift: fail-closed refusal.
  controller.beginAttempt(makeClock(3000, 3000000));
  auto changed = makeIdentity(1000000, 23);
  auto refused = controller.finalizeAttempt(changed, certificate, true, true);
  EXPECT_EQ(refused.identity_check, "REFUSED");
  EXPECT_NE(refused.identity_reason.find("proven centre"), std::string::npos);
  EXPECT_EQ(refused.state_after, "UNAVAILABLE");

  // Admissible identity but no platform certification: published unprotected
  // with the reason recorded.
  controller.beginAttempt(makeClock(4000, 4000000));
  auto uncertified =
      controller.finalizeAttempt(certificate, certificate, true, false);
  EXPECT_EQ(uncertified.identity_check, "ADMISSIBLE");
  EXPECT_FALSE(uncertified.protected_output);
  EXPECT_NE(uncertified.refusal.find("certification"), std::string::npos);
  EXPECT_EQ(uncertified.state_after, "UNAVAILABLE");

  // A missing certificate refuses the submission and lists what is missing.
  controller.beginAttempt(makeClock(5000, 5000000));
  uwb_imu_pl::PublicationIdentity incomplete = certificate;
  incomplete.manifest_digest = 0;
  auto missing = controller.finalizeAttempt(incomplete, certificate, false, false);
  EXPECT_EQ(missing.identity_check, "REFUSED");
  EXPECT_NE(missing.refusal.find("no publication certificate"),
            std::string::npos);
}

// ---------------------------------------------------------------------------
// OUT-07: the stateless monitor paths are explicitly unprotected.
// ---------------------------------------------------------------------------
TEST(PublicationWiring, StatelessMonitorPathIsExplicitlyUnprotected) {
  const auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  primeEstimator(cfg, &estimator);
  const auto timestamp = uwb_imu_pl::TimestampNs(10000000);
  const auto batch = makeBatch(timestamp, Eigen::Vector3d(0, 0, 1));
  estimator.predictTo(timestamp);
  const auto snapshot = estimator.preMeasurementSnapshot(batch);
  uwb_imu_pl::IntegrityMonitor monitor(cfg.risk, cfg.snapshot.rank_tolerance,
                                       cfg.snapshot.max_condition_number);
  const auto output = monitor.evaluateConditional(batch, *snapshot);
  EXPECT_FALSE(output.publication.gate_executed);
  EXPECT_EQ(output.publication.identity_check, "NOT_EVALUATED");
  EXPECT_TRUE(output.publication.unprotected_output);
  EXPECT_FALSE(output.publication.protected_output);
  EXPECT_NE(output.publication.refusal.find("no publication state holder"),
            std::string::npos);
  // With an explicit holder the identity check runs and the missing
  // production elements are reported instead of being invented.
  uwb_imu_pl::PublicationController controller;
  controller.beginAttempt(makeClock(1000000000, 10000000));
  const auto gated = monitor.evaluateConditional(batch, *snapshot, &controller);
  EXPECT_TRUE(gated.publication.gate_executed);
  EXPECT_EQ(gated.publication.identity_check, "REFUSED");
  EXPECT_TRUE(gated.publication.unprotected_output);
  EXPECT_FALSE(gated.publication.missing_identity_fields.empty());
}
