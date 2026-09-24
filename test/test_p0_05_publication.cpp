#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/publication/final_output_packet.hpp"

#include <gtest/gtest.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

namespace {

double o08_worst_antisymmetry_ratio = -1.0;
double o08_worst_antisymmetry_norm = 0.0;
double o08_worst_antisymmetry_tolerance = 0.0;

uwb_imu_pl::IntegrityConfig makeConfig() {
  uwb_imu_pl::IntegrityConfig config;
  config.incremental.relinearize_threshold = 0.1;
  config.incremental.relinearize_skip = 1;
  config.snapshot.rank_tolerance = 1e-10;
  config.snapshot.max_condition_number = 1e12;
  config.imu.accelerometer_sigma = 0.1;
  config.imu.gyroscope_sigma = 0.01;
  config.imu.accelerometer_bias_rw_sigma = 0.001;
  config.imu.gyroscope_bias_rw_sigma = 0.0001;
  config.imu.gravity_mps2 = 9.80665;
  config.realtime.world_frame = "map";
  config.realtime.body_frame = "base_link";
  return config;
}

uwb_imu_pl::UwbBatch makeBatch(uwb_imu_pl::TimestampNs timestamp) {
  uwb_imu_pl::UwbBatch batch;
  batch.id = uwb_imu_pl::BatchId(static_cast<std::uint64_t>(timestamp.value()));
  batch.timestamp = timestamp;
  batch.covariance_model_id = "p0_05_diagonal";
  const std::vector<Eigen::Vector3d> anchors = {
      {-5, -5, 0}, {5, -5, 1}, {5, 5, 3}, {-5, 5, 4}, {0, -6, 2}};
  const Eigen::Vector3d position(0, 0, 1);
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

std::unique_ptr<uwb_imu_pl::IncrementalUwbImuEstimator> makeEstimator(
    const uwb_imu_pl::IntegrityConfig& config) {
  auto estimator = std::make_unique<uwb_imu_pl::IncrementalUwbImuEstimator>(
      config, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator->initialize(initial, config.realtime.prior_sigmas);
  for (int i = 0; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement imu;
    imu.id = uwb_imu_pl::MeasurementId(i + 1);
    imu.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
    estimator->ingestImu(imu);
  }
  return estimator;
}

void expectNavigationEqual(const uwb_imu_pl::NavigationState& actual,
                           const uwb_imu_pl::NavigationState& expected) {
  EXPECT_EQ(actual.id, expected.id);
  EXPECT_EQ(actual.timestamp, expected.timestamp);
  EXPECT_TRUE(actual.q_world_body.coeffs().isApprox(
      expected.q_world_body.coeffs(), 0.0));
  EXPECT_TRUE(actual.position_world_m.isApprox(expected.position_world_m, 0.0));
  EXPECT_TRUE(actual.velocity_world_mps.isApprox(
      expected.velocity_world_mps, 0.0));
  EXPECT_TRUE(actual.accel_bias_mps2.isApprox(expected.accel_bias_mps2, 0.0));
  EXPECT_TRUE(actual.gyro_bias_radps.isApprox(expected.gyro_bias_radps, 0.0));
}

void expectImuEqual(const uwb_imu_pl::ImuMeasurement& actual,
                    const uwb_imu_pl::ImuMeasurement& expected) {
  EXPECT_EQ(actual.id, expected.id);
  EXPECT_EQ(actual.timestamp, expected.timestamp);
  EXPECT_TRUE(actual.specific_force_mps2.isApprox(
      expected.specific_force_mps2, 0.0));
  EXPECT_TRUE(actual.angular_velocity_radps.isApprox(
      expected.angular_velocity_radps, 0.0));
  EXPECT_EQ(actual.quality_flags, expected.quality_flags);
}

void expectLedgerEntryEqual(const uwb_imu_pl::FactorLedgerEntry& actual,
                            const uwb_imu_pl::FactorLedgerEntry& expected) {
  EXPECT_EQ(actual.factor_id, expected.factor_id);
  EXPECT_EQ(actual.group_id, expected.group_id);
  EXPECT_EQ(actual.sensor, expected.sensor);
  EXPECT_EQ(actual.kind, expected.kind);
  EXPECT_EQ(actual.keys, expected.keys);
  EXPECT_EQ(actual.source_measurements, expected.source_measurements);
  EXPECT_EQ(actual.source_ids, expected.source_ids);
  EXPECT_EQ(actual.associated_fault_units, expected.associated_fault_units);
  EXPECT_EQ(actual.fault_units, expected.fault_units);
  EXPECT_EQ(actual.epoch_begin, expected.epoch_begin);
  EXPECT_EQ(actual.epoch_end, expected.epoch_end);
  EXPECT_EQ(actual.time_begin, expected.time_begin);
  EXPECT_EQ(actual.time_end, expected.time_end);
  EXPECT_EQ(actual.backend_slot, expected.backend_slot);
  EXPECT_EQ(actual.lifecycle, expected.lifecycle);
  EXPECT_EQ(actual.health_at_commit, expected.health_at_commit);
  EXPECT_EQ(actual.noise_model_id, expected.noise_model_id);
  EXPECT_EQ(actual.model_id, expected.model_id);
  EXPECT_TRUE(actual.commit_version == expected.commit_version);
  EXPECT_EQ(actual.removed_version, expected.removed_version);
  EXPECT_EQ(actual.replacement_group, expected.replacement_group);
  EXPECT_EQ(actual.replaces_group, expected.replaces_group);
  EXPECT_EQ(actual.recovery_epoch, expected.recovery_epoch);
  EXPECT_EQ(actual.factor.get(), expected.factor.get());
}

void expectHealthEqual(const uwb_imu_pl::HealthSnapshot& actual,
                       const uwb_imu_pl::HealthSnapshot& expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_EQ(actual[i].source_id, expected[i].source_id);
    EXPECT_EQ(actual[i].source_type, expected[i].source_type);
    EXPECT_EQ(actual[i].state, expected[i].state);
    EXPECT_EQ(actual[i].suspicion_count, expected[i].suspicion_count);
    EXPECT_EQ(actual[i].shadow_pass_count, expected[i].shadow_pass_count);
    EXPECT_EQ(actual[i].recovery_pass_count, expected[i].recovery_pass_count);
    EXPECT_EQ(actual[i].recovery_reset_count,
              expected[i].recovery_reset_count);
  }
}

void expectPublishedEstimatorMetadataFrozen(
    const uwb_imu_pl::CommitBoundaryAuditV1& actual,
    const uwb_imu_pl::CommitBoundaryAuditV1& expected) {
  expectNavigationEqual(actual.published_state, expected.published_state);
  EXPECT_TRUE(actual.published_covariance.isApprox(
      expected.published_covariance, 0.0));
  EXPECT_EQ(actual.ledger_version, expected.ledger_version);
  ASSERT_EQ(actual.ledger_entries.size(), expected.ledger_entries.size());
  for (std::size_t i = 0; i < actual.ledger_entries.size(); ++i) {
    expectLedgerEntryEqual(actual.ledger_entries[i], expected.ledger_entries[i]);
  }
  EXPECT_EQ(actual.active_ledger_slots, expected.active_ledger_slots);
  ASSERT_EQ(actual.committed_epoch_catalog.size(),
            expected.committed_epoch_catalog.size());
  for (std::size_t i = 0; i < actual.committed_epoch_catalog.size(); ++i) {
    const auto& a = actual.committed_epoch_catalog[i];
    const auto& e = expected.committed_epoch_catalog[i];
    EXPECT_EQ(a.transaction_id, e.transaction_id);
    EXPECT_EQ(a.previous_epoch, e.previous_epoch);
    EXPECT_EQ(a.proposed_epoch, e.proposed_epoch);
    EXPECT_EQ(a.begin, e.begin);
    EXPECT_EQ(a.end, e.end);
    EXPECT_EQ(a.uwb_batch_id, e.uwb_batch_id);
    EXPECT_EQ(a.selected_groups, e.selected_groups);
  }
  EXPECT_EQ(actual.committed_batch_catalog, expected.committed_batch_catalog);
  ASSERT_EQ(actual.state_history.size(), expected.state_history.size());
  auto a_state = actual.state_history.begin();
  auto e_state = expected.state_history.begin();
  for (; a_state != actual.state_history.end(); ++a_state, ++e_state) {
    EXPECT_EQ(a_state->first, e_state->first);
    expectNavigationEqual(a_state->second, e_state->second);
  }
  ASSERT_EQ(actual.covariance_history.size(),
            expected.covariance_history.size());
  auto a_cov = actual.covariance_history.begin();
  auto e_cov = expected.covariance_history.begin();
  for (; a_cov != actual.covariance_history.end(); ++a_cov, ++e_cov) {
    EXPECT_EQ(a_cov->first, e_cov->first);
    EXPECT_TRUE(a_cov->second.isApprox(e_cov->second, 0.0));
  }
  ASSERT_EQ(actual.imu_queue.size(), expected.imu_queue.size());
  for (std::size_t i = 0; i < actual.imu_queue.size(); ++i) {
    expectImuEqual(actual.imu_queue[i], expected.imu_queue[i]);
  }
  ASSERT_EQ(actual.imu_boundary.has_value(), expected.imu_boundary.has_value());
  if (actual.imu_boundary) {
    expectImuEqual(*actual.imu_boundary, *expected.imu_boundary);
  }
  EXPECT_EQ(actual.last_received_imu_timestamp,
            expected.last_received_imu_timestamp);
  EXPECT_EQ(actual.epoch, expected.epoch);
  EXPECT_EQ(actual.state_timestamp, expected.state_timestamp);
  EXPECT_TRUE(actual.version == expected.version);
}

struct IndependentBoundarySnapshot {
  uwb_imu_pl::NavigationState state;
  Eigen::Matrix<double, 15, 15> covariance;
  std::uint64_t ledger_version = 0;
  std::vector<uwb_imu_pl::FactorLedgerEntry> ledger;
  std::vector<std::size_t> active_slots;
  std::vector<uwb_imu_pl::CommittedEpochCatalogAuditV1> epochs;
  std::vector<std::pair<std::size_t, uwb_imu_pl::BatchId>> batches;
  std::map<std::size_t, uwb_imu_pl::NavigationState> states;
  std::map<std::size_t, Eigen::Matrix<double, 15, 15>> covariances;
  std::vector<uwb_imu_pl::ImuMeasurement> imu_queue;
  std::optional<uwb_imu_pl::ImuMeasurement> imu_boundary;
  std::optional<uwb_imu_pl::TimestampNs> last_imu;
  std::size_t epoch = 0;
  uwb_imu_pl::TimestampNs timestamp;
  uwb_imu_pl::LinearizationVersion version;
  std::optional<uwb_imu_pl::BatchId> pending_batch;
  uwb_imu_pl::HealthSnapshot health;
  std::uint32_t bridge_epochs = 0;
  std::optional<uwb_imu_pl::TimestampNs> bridge_start;
};

IndependentBoundarySnapshot captureIndependentBoundary(
    const uwb_imu_pl::IncrementalUwbImuEstimator& estimator,
    const uwb_imu_pl::RealtimeIntegrityPipeline& pipeline) {
  IndependentBoundarySnapshot out;
  out.state = estimator.currentState();
  const auto audit = estimator.audit();
  const auto& covariance_history = estimator.debugCovarianceHistory();
  out.covariance = covariance_history.at(estimator.currentEpoch());
  out.ledger_version = estimator.factorLedger().version();
  out.ledger = estimator.factorLedger().entries();
  for (const auto& entry : out.ledger) {
    if (entry.backend_slot &&
        entry.lifecycle == uwb_imu_pl::FactorLifecycle::Active) {
      out.active_slots.push_back(*entry.backend_slot);
    }
  }
  std::sort(out.active_slots.begin(), out.active_slots.end());
  out.epochs = estimator.debugCommittedEpochCatalog();
  out.batches = estimator.debugCommittedBatchCatalog();
  out.states = estimator.debugStateHistory();
  out.covariances = estimator.debugCovarianceHistory();
  out.imu_queue = estimator.debugImuQueue();
  out.imu_boundary = estimator.debugImuBoundary();
  out.last_imu = estimator.debugLastReceivedImuTimestamp();
  out.epoch = estimator.currentEpoch();
  out.timestamp = audit.state_timestamp;
  out.version = estimator.debugVersion();
  out.pending_batch = estimator.debugPendingBatchId();
  out.health = pipeline.debugHealthSnapshot();
  out.bridge_epochs = pipeline.debugConsecutiveBridgeEpochs();
  out.bridge_start = pipeline.debugBridgeStartTimestamp();
  return out;
}

void expectIndependentBoundaryEqual(const IndependentBoundarySnapshot& actual,
                                    const IndependentBoundarySnapshot& expected) {
  expectNavigationEqual(actual.state, expected.state);
  EXPECT_TRUE(actual.covariance.isApprox(expected.covariance, 0.0));
  EXPECT_EQ(actual.ledger_version, expected.ledger_version);
  ASSERT_EQ(actual.ledger.size(), expected.ledger.size());
  for (std::size_t i = 0; i < actual.ledger.size(); ++i)
    expectLedgerEntryEqual(actual.ledger[i], expected.ledger[i]);
  EXPECT_EQ(actual.active_slots, expected.active_slots);
  ASSERT_EQ(actual.epochs.size(), expected.epochs.size());
  for (std::size_t i = 0; i < actual.epochs.size(); ++i) {
    EXPECT_EQ(actual.epochs[i].transaction_id, expected.epochs[i].transaction_id);
    EXPECT_EQ(actual.epochs[i].previous_epoch, expected.epochs[i].previous_epoch);
    EXPECT_EQ(actual.epochs[i].proposed_epoch, expected.epochs[i].proposed_epoch);
    EXPECT_EQ(actual.epochs[i].begin, expected.epochs[i].begin);
    EXPECT_EQ(actual.epochs[i].end, expected.epochs[i].end);
    EXPECT_EQ(actual.epochs[i].uwb_batch_id, expected.epochs[i].uwb_batch_id);
    EXPECT_EQ(actual.epochs[i].selected_groups, expected.epochs[i].selected_groups);
  }
  EXPECT_EQ(actual.batches, expected.batches);
  ASSERT_EQ(actual.states.size(), expected.states.size());
  auto as = actual.states.begin();
  auto es = expected.states.begin();
  for (; as != actual.states.end(); ++as, ++es) {
    EXPECT_EQ(as->first, es->first);
    expectNavigationEqual(as->second, es->second);
  }
  ASSERT_EQ(actual.covariances.size(), expected.covariances.size());
  auto ac = actual.covariances.begin();
  auto ec = expected.covariances.begin();
  for (; ac != actual.covariances.end(); ++ac, ++ec) {
    EXPECT_EQ(ac->first, ec->first);
    EXPECT_TRUE(ac->second.isApprox(ec->second, 0.0));
  }
  ASSERT_EQ(actual.imu_queue.size(), expected.imu_queue.size());
  for (std::size_t i = 0; i < actual.imu_queue.size(); ++i)
    expectImuEqual(actual.imu_queue[i], expected.imu_queue[i]);
  ASSERT_EQ(actual.imu_boundary.has_value(), expected.imu_boundary.has_value());
  if (actual.imu_boundary) expectImuEqual(*actual.imu_boundary, *expected.imu_boundary);
  EXPECT_EQ(actual.last_imu, expected.last_imu);
  EXPECT_EQ(actual.epoch, expected.epoch);
  EXPECT_EQ(actual.timestamp, expected.timestamp);
  EXPECT_TRUE(actual.version == expected.version);
  expectHealthEqual(actual.health, expected.health);
  EXPECT_EQ(actual.bridge_epochs, expected.bridge_epochs);
  EXPECT_EQ(actual.bridge_start, expected.bridge_start);
}

// The O08 fixture deliberately reaches fixed-lag marginalization many times.
// Range factors with a zero lever arm constrain position but not attitude, so
// relying on an old pose prior propagated through several 10 ms IMU intervals
// made the test graph needlessly close to singular after that prior was
// marginalized.  Give each fixture epoch its own finite, positive-definite
// navigation-state anchor.  These are test-only factors; they do not change a
// production threshold, risk allocation, fault family, or action census.
void addMatureFixtureStatePriors(uwb_imu_pl::EpochTransaction* transaction) {
  ASSERT_NE(transaction, nullptr);
  const auto nominal = std::find_if(
      transaction->uwb_groups.begin(), transaction->uwb_groups.end(),
      [](const auto& group) { return group.nominal; });
  ASSERT_NE(nominal, transaction->uwb_groups.end());

  const auto epoch = transaction->proposed_epoch;
  const gtsam::Key pose_key = gtsam::Symbol('x', epoch);
  const gtsam::Key velocity_key = gtsam::Symbol('v', epoch);
  const gtsam::Key bias_key = gtsam::Symbol('b', epoch);
  const auto& state = transaction->cv_predicted_state;
  const gtsam::Pose3 pose(
      gtsam::Rot3(state.q_world_body.toRotationMatrix()),
      state.position_world_m);
  const gtsam::imuBias::ConstantBias bias(
      state.accel_bias_mps2, state.gyro_bias_radps);
  const auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector6::Constant(0.25));
  const auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.25);
  const auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector6::Constant(0.05));
  nominal->factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      pose_key, pose, pose_noise));
  nominal->factors.add(gtsam::PriorFactor<gtsam::Vector3>(
      velocity_key, state.velocity_world_mps, velocity_noise));
  nominal->factors.add(
      gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
          bias_key, bias, bias_noise));
  for (const auto key : {pose_key, velocity_key, bias_key}) {
    if (std::find(nominal->keys.begin(), nominal->keys.end(), key) ==
        nominal->keys.end()) {
      nominal->keys.push_back(key);
    }
  }
  nominal->model_id += "+p005_mature_full_state_priors_v1";
}

void assertMatureFixtureFullyConstrained(
    const uwb_imu_pl::EpochTransaction& transaction,
    const uwb_imu_pl::EpochCommitPlan& plan) {
  ASSERT_TRUE(transaction.frozen_graph);
  ASSERT_TRUE(transaction.frozen_values);
  ASSERT_EQ(transaction.proposed_epoch, transaction.previous_epoch + 1);
  const gtsam::Key pose_key = gtsam::Symbol('x', transaction.proposed_epoch);
  const gtsam::Key velocity_key =
      gtsam::Symbol('v', transaction.proposed_epoch);
  const gtsam::Key bias_key = gtsam::Symbol('b', transaction.proposed_epoch);
  ASSERT_TRUE(transaction.frozen_values->exists(pose_key));
  ASSERT_TRUE(transaction.frozen_values->exists(velocity_key));
  ASSERT_TRUE(transaction.frozen_values->exists(bias_key));

  gtsam::NonlinearFactorGraph graph = *transaction.frozen_graph;
  std::set<gtsam::Key> directly_anchored_keys;
  for (const auto id : plan.groups_to_add) {
    const auto append = [&](const uwb_imu_pl::PendingFactorGroup& group) {
      if (group.id != id) return false;
      for (const auto& factor : group.factors) graph.add(factor);
      if (group.model_id.find("p005_mature_full_state_priors_v1") !=
          std::string::npos) {
        directly_anchored_keys.insert(pose_key);
        directly_anchored_keys.insert(velocity_key);
        directly_anchored_keys.insert(bias_key);
      }
      return true;
    };
    bool found = append(transaction.imu_group);
    for (const auto& group : transaction.uwb_groups) {
      found = append(group) || found;
    }
    found = append(transaction.generic_bridge_group) || found;
    found = append(transaction.generic_bias_continuity_group) || found;
    ASSERT_TRUE(found) << "fixture plan references an unknown group";
  }
  EXPECT_EQ(directly_anchored_keys,
            (std::set<gtsam::Key>{pose_key, velocity_key, bias_key}));

  // Independent batch elimination must certify a finite positive-definite
  // marginal for all 15 components before the fixture is allowed to exercise
  // a post-mutation fault point.
  ASSERT_NO_THROW(([&] {
    const gtsam::Marginals marginals(graph, *transaction.frozen_values);
    const gtsam::KeyVector keys{pose_key, velocity_key, bias_key};
    const auto joint = marginals.jointMarginalCovariance(keys);
    Eigen::Matrix<double, 15, 15> covariance;
    const int offsets[] = {0, 6, 9};
    const int dimensions[] = {6, 3, 6};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        covariance.block(offsets[row], offsets[column], dimensions[row],
                         dimensions[column]) =
            joint.at(keys[row], keys[column]);
      }
    }
    ASSERT_TRUE(covariance.allFinite());
    // Check the raw Marginals result before symmetrizing it. This is a tight,
    // scale-aware roundoff bound, not a tolerance for repairing asymmetry.
    constexpr double kRawCovarianceSymmetryRelativeTolerance = 1e-12;
    const double covariance_scale = std::max(1.0, covariance.norm());
    const double antisymmetry_norm =
        (covariance - covariance.transpose()).norm();
    const double antisymmetry_tolerance =
        kRawCovarianceSymmetryRelativeTolerance * covariance_scale;
    const double antisymmetry_ratio =
        antisymmetry_norm / antisymmetry_tolerance;
    if (antisymmetry_ratio > o08_worst_antisymmetry_ratio) {
      o08_worst_antisymmetry_ratio = antisymmetry_ratio;
      o08_worst_antisymmetry_norm = antisymmetry_norm;
      o08_worst_antisymmetry_tolerance = antisymmetry_tolerance;
    }
    EXPECT_LE(antisymmetry_norm, antisymmetry_tolerance)
        << "raw Marginals covariance antisymmetry_norm=" << antisymmetry_norm
        << " tolerance=" << antisymmetry_tolerance;
    ASSERT_TRUE(covariance.isApprox(
        covariance.transpose(),
        kRawCovarianceSymmetryRelativeTolerance));
    const Eigen::Matrix<double, 15, 15> symmetric_covariance =
        0.5 * (covariance + covariance.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> solve(
        symmetric_covariance);
    ASSERT_EQ(solve.info(), Eigen::Success);
    ASSERT_GT(solve.eigenvalues().minCoeff(), 0.0);
  }()));
}

void matureEstimator(uwb_imu_pl::IncrementalUwbImuEstimator* estimator,
                     const uwb_imu_pl::IntegrityConfig& config,
                     int final_epoch) {
  for (int epoch = 1; epoch <= final_epoch; ++epoch) {
    for (int half = (epoch == 1 ? 3 : 1); half <= 2; ++half) {
      uwb_imu_pl::ImuMeasurement imu;
      imu.id = uwb_imu_pl::MeasurementId(100 + epoch * 2 + half);
      imu.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>((epoch - 1) * 10000000 + half * 5000000));
      imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
      estimator->ingestImu(imu);
    }
    auto transaction = estimator->prepareEpoch(makeBatch(
        uwb_imu_pl::TimestampNs(static_cast<std::int64_t>(epoch) * 10000000)));
    addMatureFixtureStatePriors(&transaction);
    const auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
    assertMatureFixtureFullyConstrained(transaction, plan);
    estimator->commitEpoch(std::move(transaction), plan);
  }
}

class TestPublicationTransaction final
    : public uwb_imu_pl::FinalPacketPublicationTransaction {
 public:
  std::function<void(const uwb_imu_pl::FinalOutputPacket&)> candidate_callback;
  std::uint64_t candidate_digest = 0;
  bool candidate_protected = true;
  std::uint64_t committed_digest = 0;
  bool committed = false;
  bool aborted = false;

  void publishCandidate(
      const uwb_imu_pl::FinalOutputPacket& candidate) override {
    candidate_digest = uwb_imu_pl::finalOutputPacketDigest(candidate);
    candidate_protected = candidate.output().publication.protected_output;
    if (candidate_callback) candidate_callback(candidate);
  }
  bool supportsAtomicProtectedReceipt() const noexcept override { return true; }
  void commitReceipt(
      const uwb_imu_pl::FinalOutputPacket& packet) noexcept override {
    committed_digest = uwb_imu_pl::finalOutputPacketDigest(packet);
    committed = true;
  }
  void abort() noexcept override { aborted = true; }
};

class NonAtomicPublicationTransaction final
    : public uwb_imu_pl::FinalPacketPublicationTransaction {
 public:
  bool candidate_protected = true;
  bool receipt_protected = true;
  std::uint64_t receipt_digest = 0;
  bool receipt_committed = false;
  void publishCandidate(
      const uwb_imu_pl::FinalOutputPacket& candidate) override {
    candidate_protected = candidate.output().publication.protected_output;
  }
  void commitReceipt(
      const uwb_imu_pl::FinalOutputPacket& receipt) noexcept override {
    receipt_protected = receipt.output().publication.protected_output;
    receipt_digest = receipt.metadata().digest;
    receipt_committed = true;
  }
  void abort() noexcept override {}
};

class ProductionFanoutProbe final
    : public uwb_imu_pl::FinalPacketPublicationTransaction {
 public:
  std::vector<std::function<void(
      const uwb_imu_pl::FinalOutputPacket&)>> sinks;
  std::vector<std::uint64_t> visible_digests;
  bool commit_called = false;
  bool aborted = false;

  void publishCandidate(
      const uwb_imu_pl::FinalOutputPacket& candidate) override {
    for (const auto& sink : sinks) {
      sink(candidate);
      visible_digests.push_back(candidate.metadata().digest);
    }
  }
  void commitReceipt(
      const uwb_imu_pl::FinalOutputPacket&) noexcept override {
    commit_called = true;
  }
  void abort() noexcept override { aborted = true; }
};

}  // namespace

TEST(P005Transaction, EveryPostMutationBoundaryHasExplicitTerminalReceipt) {
  using uwb_imu_pl::CommitFaultPoint;
  o08_worst_antisymmetry_ratio = -1.0;
  o08_worst_antisymmetry_norm = 0.0;
  o08_worst_antisymmetry_tolerance = 0.0;
  const std::vector<std::pair<CommitFaultPoint, std::string>> points = {
      {CommitFaultPoint::AfterBackendUpdate, "after_backend_update"},
      {CommitFaultPoint::DuringBackendUpdate, "backend_update"},
      {CommitFaultPoint::BeforeStateQuery, "state_query"},
      {CommitFaultPoint::BeforeMarginalQuery, "marginal_query"},
      {CommitFaultPoint::BeforeLedgerBind, "ledger_bind"},
      {CommitFaultPoint::BeforeSlotBind, "slot_bind"},
      {CommitFaultPoint::BeforeMetadataPrune, "metadata_prune"},
      {CommitFaultPoint::BeforeReceiptCreation, "receipt_creation"},
  };
  std::set<CommitFaultPoint> reached;
  std::set<std::uint64_t> reached_nonces;
  for (const auto& item : points) {
    const auto point = item.first;
    auto config = makeConfig();
    config.integrity_window.epochs = 1;
    config.integrity_window.recovery_margin_epochs = 0;
    config.incremental.fixed_lag_epochs = 3;
    auto estimator = makeEstimator(config);
    matureEstimator(estimator.get(), config, 5);
    for (int sample = 11; sample <= 12; ++sample) {
      uwb_imu_pl::ImuMeasurement imu;
      imu.id = uwb_imu_pl::MeasurementId(1000 + sample);
      imu.timestamp = uwb_imu_pl::TimestampNs(sample * 5000000LL);
      imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
      estimator->ingestImu(imu);
    }
    auto bridge_transaction = estimator->prepareEpoch(
        makeBatch(uwb_imu_pl::TimestampNs(60000000)));
    addMatureFixtureStatePriors(&bridge_transaction);
    auto bridge_plan =
        uwb_imu_pl::EpochCommitPlan::nominalPlan(bridge_transaction);
    bridge_plan.groups_to_add.erase(
        std::remove(bridge_plan.groups_to_add.begin(),
                    bridge_plan.groups_to_add.end(),
                    bridge_transaction.imu_group.id),
        bridge_plan.groups_to_add.end());
    bridge_plan.groups_to_add.push_back(
        bridge_transaction.generic_bridge_group.id);
    bridge_plan.groups_to_add.push_back(
        bridge_transaction.generic_bias_continuity_group.id);
    bridge_plan.bridge_mode = uwb_imu_pl::BridgeMode::GenericKinematic;
    bridge_plan.fde_status =
        uwb_imu_pl::FdeStatus::SuccessImuExclusionGenericBridge;
    assertMatureFixtureFullyConstrained(bridge_transaction, bridge_plan);
    (void)estimator->commitEpoch(std::move(bridge_transaction), bridge_plan);
    uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
        estimator.get(), uwb_imu_pl::IntegrityMonitor(
            config.risk, config.snapshot.rank_tolerance,
            config.snapshot.max_condition_number));
    for (int sample = 13; sample <= 14; ++sample) {
      uwb_imu_pl::ImuMeasurement imu;
      imu.id = uwb_imu_pl::MeasurementId(1000 + sample);
      imu.timestamp = uwb_imu_pl::TimestampNs(sample * 5000000LL);
      imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
      pipeline.ingestImu(imu);
    }
    auto transaction = estimator->prepareEpoch(
        makeBatch(uwb_imu_pl::TimestampNs(70000000)));
    addMatureFixtureStatePriors(&transaction);
    const auto transaction_id = transaction.id;
    const auto updates_before = estimator->backendUpdateCount();
    const auto snapshot_before = captureIndependentBoundary(*estimator, pipeline);
    ASSERT_GT(snapshot_before.epoch, 1u);
    ASSERT_FALSE(snapshot_before.ledger.empty());
    ASSERT_FALSE(snapshot_before.active_slots.empty());
    ASSERT_FALSE(snapshot_before.epochs.empty());
    ASSERT_FALSE(snapshot_before.batches.empty());
    ASSERT_FALSE(snapshot_before.states.empty());
    ASSERT_FALSE(snapshot_before.covariances.empty());
    ASSERT_FALSE(snapshot_before.imu_queue.empty());
    ASSERT_TRUE(snapshot_before.imu_boundary.has_value());
    ASSERT_TRUE(snapshot_before.last_imu.has_value());
    ASSERT_FALSE(snapshot_before.health.empty());
    ASSERT_GT(snapshot_before.bridge_epochs, 0u);
    ASSERT_TRUE(snapshot_before.bridge_start.has_value());
    ASSERT_GT(estimator->marginalizationCount(), 0u);
    const auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
    assertMatureFixtureFullyConstrained(transaction, plan);
    const std::uint64_t armed_nonce =
        estimator->setCommitFaultPointForTesting(point);
    ASSERT_NE(armed_nonce, 0u);
    const auto armed = estimator->commitFaultInjectionAuditForTesting();
    ASSERT_TRUE(armed.armed);
    ASSERT_FALSE(armed.hit);
    ASSERT_FALSE(armed.consumed);
    ASSERT_EQ(armed.armed_point, point);
    ASSERT_EQ(armed.armed_nonce, armed_nonce);
    try {
      (void)estimator->commitEpoch(std::move(transaction), plan);
      FAIL() << "fault point did not throw";
    } catch (const uwb_imu_pl::CommitTerminalError& error) {
      const auto& terminal = error.receipt();
      EXPECT_EQ(terminal.transaction_id, transaction_id);
      EXPECT_TRUE(terminal.backend_mutated);
      EXPECT_TRUE(terminal.backend_poisoned);
      EXPECT_TRUE(terminal.committed_unprotected);
      EXPECT_EQ(terminal.backend_updates, 1u);
      EXPECT_EQ(terminal.boundary, item.second);
      EXPECT_TRUE(terminal.injected_fault_hit);
      EXPECT_EQ(terminal.injected_fault_point, point);
      EXPECT_EQ(terminal.injected_fault_nonce, armed_nonce);
      const auto hit = estimator->commitFaultInjectionAuditForTesting();
      EXPECT_FALSE(hit.armed);
      EXPECT_TRUE(hit.hit);
      EXPECT_TRUE(hit.consumed);
      EXPECT_EQ(hit.hit_point, point);
      EXPECT_EQ(hit.hit_nonce, armed_nonce);
      if (terminal.injected_fault_hit &&
          terminal.injected_fault_point == point &&
          terminal.injected_fault_nonce == armed_nonce && hit.hit &&
          hit.consumed && hit.hit_point == point &&
          hit.hit_nonce == armed_nonce) {
        reached.insert(point);
        reached_nonces.insert(armed_nonce);
      }
    }
    EXPECT_TRUE(estimator->backendPoisoned());
    EXPECT_FALSE(estimator->hasPendingEpoch());
    EXPECT_LE(estimator->backendUpdateCount() - updates_before, 1u);
    EXPECT_EQ(estimator->currentEpoch(), snapshot_before.epoch)
        << "staged metadata must not publish without a receipt";
    const auto snapshot_after = captureIndependentBoundary(*estimator, pipeline);
    expectIndependentBoundaryEqual(snapshot_after, snapshot_before);
    EXPECT_TRUE(snapshot_before.pending_batch.has_value());
    EXPECT_FALSE(snapshot_after.pending_batch.has_value());
    EXPECT_NE(estimator->debugActiveTransactionId(), transaction_id.value());
    EXPECT_FALSE(estimator->hasPendingEpoch());
    EXPECT_TRUE(estimator->backendPoisoned());
  }
  EXPECT_EQ(reached.size(), points.size());
  EXPECT_EQ(reached_nonces.size(), points.size());
  std::cout << "[ O08 RAW COVARIANCE ] worst_antisymmetry_norm="
            << o08_worst_antisymmetry_norm
            << " tolerance=" << o08_worst_antisymmetry_tolerance << '\n';
}

TEST(P005Transaction, PreMutationFailureRemainsHonestlyDiscardable) {
  const auto config = makeConfig();
  auto estimator = makeEstimator(config);
  auto transaction = estimator->prepareEpoch(
      makeBatch(uwb_imu_pl::TimestampNs(10000000)));
  const auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
  const auto updates_before = estimator->backendUpdateCount();
  estimator->setCommitFaultPointForTesting(
      uwb_imu_pl::CommitFaultPoint::BeforeBackendUpdate);
  EXPECT_THROW((void)estimator->commitEpoch(std::move(transaction), plan),
               std::runtime_error);
  EXPECT_FALSE(estimator->backendPoisoned());
  EXPECT_TRUE(estimator->hasPendingEpoch());
  EXPECT_EQ(estimator->backendUpdateCount(), updates_before);
  EXPECT_NO_THROW((void)estimator->discardEpoch(
      std::move(transaction),
      {uwb_imu_pl::FdeStatus::ModelInvalid, "injected pre-mutation", false}));
}

TEST(P005Reference, FabricatedProofIsRejectedBeforeMutationAndLegacyIsUnprotected) {
  for (const auto mode : {0, 1, 2, 3}) {
    const auto config = makeConfig();
    auto estimator = makeEstimator(config);
    auto transaction = estimator->prepareEpoch(
        makeBatch(uwb_imu_pl::TimestampNs(10000000)));
    auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
    uwb_imu_pl::CommitProtectionEvidenceV1 forged;
    forged.transaction_id = transaction.id;
    forged.window_id = uwb_imu_pl::WindowId(7);
    forged.base_version = transaction.base_version;
    forged.candidate_proof_identity = 1234;
    forged.protection_proof_identity = 1234;
    forged.reference.valid = true;
    forged.reference.mean_world_m = {0.25, -0.5, 0.75};
    forged.reference.pl_at_reference_m = {1.0, 2.0, 3.0};
    forged.reference.timestamp = transaction.end;
    forged.reference.frame_id = "map";
    forged.reference.position_reference = "body_origin";
    forged.reference.numerical_proof_id = 1234;
    forged.token_identity = mode == 0 ? 1234 : 4321;
    if (mode == 1) forged.reference.timestamp = uwb_imu_pl::TimestampNs(1);
    if (mode == 2) forged.reference.mean_world_m.x() =
        std::numeric_limits<double>::infinity();
    if (mode == 3) forged.reference.pl_at_reference_m.z() =
        std::numeric_limits<double>::max();
    const auto updates_before = estimator->backendUpdateCount();
    uwb_imu_pl::CommitCertificationV1 rejected;
    EXPECT_THROW((void)estimator->commitEpochCertified(
                     std::move(transaction), plan, &forged, &rejected),
                 std::invalid_argument);
    EXPECT_EQ(estimator->backendUpdateCount(), updates_before);
    EXPECT_TRUE(estimator->hasPendingEpoch());
    EXPECT_FALSE(estimator->backendPoisoned());

    uwb_imu_pl::CommitCertificationV1 unprotected;
    const auto receipt = estimator->commitEpochCertified(
        std::move(transaction), plan, nullptr, &unprotected);
    EXPECT_EQ(receipt.backend_updates, 1u);
    EXPECT_FALSE(receipt.integrity_available);
    EXPECT_FALSE(unprotected.reference_bound);
    EXPECT_FALSE(unprotected.integrity_available);
    EXPECT_TRUE(unprotected.committed_covariance.allFinite());
    const auto actual_state = estimator->currentState();
    const auto actual_backend = estimator->audit();
    EXPECT_TRUE(unprotected.committed_mean_world_m.isApprox(
        actual_state.position_world_m, 0.0));
    EXPECT_TRUE(unprotected.committed_covariance.isApprox(
        actual_backend.current_marginal, 1e-12));
  }
}

TEST(P005Reference,
     GenuineP003CandidateCommitsActualBackendAndTransfersReference) {
  using namespace uwb_imu_pl;
  const auto config = makeConfig();
  auto estimator = makeEstimator(config);
  auto transaction = estimator->prepareEpoch(makeBatch(TimestampNs(10000000)));
  const auto window = estimator->buildIntegrityWindow(
      transaction, IntegrityWindowRequest{});
  ASSERT_TRUE(window.model_valid) << window.reason;

  ExclusionAction keep;
  keep.id = ExclusionActionId(5001);
  keep.action_model_id = "KEEP_ALL";
  RankUpdateConfig rank_config;
  ASSERT_TRUE(window.numerics);
  rank_config.rank_tolerance =
      window.numerics->numerical_contract.rank_tolerance;
  rank_config.max_condition_number =
      window.numerics->numerical_contract.max_condition_number;
  rank_config.max_linearization_step_norm = 100.0;
  const RankUpdateEvaluator evaluator(rank_config);
  const auto base = evaluator.factorizeOnce(window, {keep});
  auto candidate = evaluator.evaluate(base, keep);
  ASSERT_TRUE(candidate.valid) << candidate.reason;

  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.rank_tolerance = rank_config.rank_tolerance;
  detector_risk.max_condition_number = rank_config.max_condition_number;
  const auto detector = JointWindowDetector().evaluateCandidate(
      window, candidate, detector_risk);
  ASSERT_TRUE(detector.numerically_valid) << detector.reason;
  ASSERT_TRUE(detector.passed) << detector.reason;

  Eigen::MatrixXd mode = Eigen::MatrixXd::Zero(candidate.rows, 1);
  mode(0, 0) = 1.0;
  mode(mode.rows() - 1, 0) = 1.0;
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(5002);
  hypothesis.modes = {FaultModeId(5003)};
  hypothesis.A = mode;
  hypothesis.p_md_allocation = 1e-3;
  hypothesis.hmi_allocation = 1e-6;
  hypothesis.prior_probability_bound = 1e-3;
  std::vector<FaultHypothesisV2> hypotheses{hypothesis};
  ProtectionLevelSharedContext shared;
  shared.mode_maps.emplace(5003, mode);
  const auto pl = ProtectionLevelV2().computeShared(
      window, &candidate, detector, &hypotheses, shared, RiskBudgetV2{});
  ASSERT_TRUE(pl.model_valid) << pl.reason;
  ASSERT_TRUE(pl.pl_xyz_m.allFinite());
  const auto proof = protectionLevelV2ProofIdentity(candidate, pl);
  ASSERT_NE(proof, 0u);

  CommitProtectionEvidenceV1 evidence;
  std::string reason;
  ASSERT_TRUE(mintCommitProtectionEvidenceV1(
      transaction, window, candidate, pl, proof, "map", &evidence, &reason))
      << reason;

  // Independent nonlinear oracle: assemble the raw factor graph and values
  // directly from the frozen backend plus the selected factor objects.  This
  // intentionally does not call the window/candidate audit covariance or the
  // production nominal+protected-map reference formula.
  ASSERT_TRUE(transaction.frozen_graph);
  ASSERT_TRUE(transaction.frozen_values);
  gtsam::NonlinearFactorGraph raw_graph = *transaction.frozen_graph;
  for (const auto& factor : transaction.imu_group.factors) raw_graph.add(factor);
  for (const auto& group : transaction.uwb_groups) {
    if (group.nominal) {
      for (const auto& factor : group.factors) raw_graph.add(factor);
    }
  }
  const gtsam::Values raw_batch_values =
      gtsam::LevenbergMarquardtOptimizer(
          raw_graph, *transaction.frozen_values).optimize();
  const gtsam::Key pose_key = gtsam::Symbol('x', transaction.proposed_epoch);
  const gtsam::Key velocity_key =
      gtsam::Symbol('v', transaction.proposed_epoch);
  const gtsam::Key bias_key = gtsam::Symbol('b', transaction.proposed_epoch);
  const auto raw_pose = raw_batch_values.at<gtsam::Pose3>(pose_key);
  const auto raw_velocity = raw_batch_values.at<gtsam::Vector3>(velocity_key);
  const auto raw_bias =
      raw_batch_values.at<gtsam::imuBias::ConstantBias>(bias_key);
  gtsam::Marginals raw_marginals(raw_graph, raw_batch_values);
  const gtsam::KeyVector raw_keys{pose_key, velocity_key, bias_key};
  const auto raw_joint = raw_marginals.jointMarginalCovariance(raw_keys);
  Eigen::Matrix<double, 15, 15> raw_covariance =
      Eigen::Matrix<double, 15, 15>::Zero();
  const int offsets[] = {0, 6, 9};
  const int dimensions[] = {6, 3, 6};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      raw_covariance.block(offsets[row], offsets[column], dimensions[row],
                           dimensions[column]) =
          raw_joint.at(raw_keys[row], raw_keys[column]);
    }
  }
  const Eigen::Vector3d independent_frozen_reference = raw_pose.translation();
  EXPECT_LT((evidence.reference.mean_world_m - independent_frozen_reference)
                .norm(),
            1e-4);

  const auto plan = EpochCommitPlan::nominalPlan(transaction);
  CommitCertificationV1 certification;
  const auto receipt = estimator->commitEpochCertified(
      std::move(transaction), plan, &evidence, &certification);
  ASSERT_EQ(receipt.backend_updates, 1u);
  ASSERT_TRUE(receipt.integrity_available);
  ASSERT_TRUE(certification.reference_bound);
  const auto actual_state = estimator->currentState();
  const auto actual_backend = estimator->audit();
  const gtsam::Pose3 actual_pose(
      gtsam::Rot3(actual_state.q_world_body.toRotationMatrix()),
      actual_state.position_world_m);
  EXPECT_LT(gtsam::Pose3::Logmap(raw_pose.between(actual_pose)).norm(), 1e-5);
  EXPECT_LT((actual_state.velocity_world_mps - raw_velocity).norm(), 1e-5);
  EXPECT_LT((actual_state.accel_bias_mps2 - raw_bias.accelerometer()).norm(),
            1e-5);
  EXPECT_LT((actual_state.gyro_bias_radps - raw_bias.gyroscope()).norm(),
            1e-5);
  EXPECT_LT((actual_backend.current_marginal - raw_covariance).norm() /
                raw_covariance.norm(),
            1e-5);
  EXPECT_TRUE(certification.committed_mean_world_m.isApprox(
      actual_state.position_world_m, 0.0));
  EXPECT_TRUE(certification.committed_covariance.isApprox(
      actual_backend.current_marginal, 1e-12));
  const Eigen::Vector3d independent_transfer =
      (actual_state.position_world_m -
       evidence.reference.mean_world_m).cwiseAbs();
  EXPECT_TRUE(certification.reference_transfer_m.isApprox(
      independent_transfer, 0.0));
  EXPECT_TRUE(certification.transferred_pl_m.isApprox(
      pl.pl_xyz_m + independent_transfer, 0.0));

  std::string packet_id;
  ASSERT_TRUE(bindTransferredProtectionLevelPublicationPacket(
      candidate, pl, proof, evidence.reference.mean_world_m,
      actual_state.position_world_m, certification.transferred_pl_m,
      receipt.state_timestamp.value(), "map", "body_origin", &packet_id));
  ProtectionLevelPublicationPacketV2 packet;
  ASSERT_TRUE(protectionLevelPublicationPacketV2(packet_id, &packet));
  EXPECT_TRUE(packet.reference_mean_world_m.isApprox(
      evidence.reference.mean_world_m, 0.0));
  EXPECT_TRUE(packet.committed_mean_world_m.isApprox(
      actual_state.position_world_m, 0.0));
  EXPECT_TRUE(validateProtectionLevelPublicationProof(
      proof, certification.transferred_pl_m, packet_id, &reason)) << reason;

  // Boundary probes are separate from the genuine commit: a very large
  // finite transfer is accepted componentwise, while overflow/Inf and a
  // mismatched committed mean fail closed.
  const Eigen::Vector3d large_committed =
      evidence.reference.mean_world_m + Eigen::Vector3d::Constant(1e150);
  const Eigen::Vector3d large_transfer =
      (large_committed - evidence.reference.mean_world_m).cwiseAbs();
  ASSERT_TRUE(bindTransferredProtectionLevelPublicationPacket(
      candidate, pl, proof, evidence.reference.mean_world_m, large_committed,
      pl.pl_xyz_m + large_transfer, receipt.state_timestamp.value(), "map",
      "body_origin", &packet_id));
  EXPECT_TRUE(validateProtectionLevelPublicationProof(
      proof, pl.pl_xyz_m + large_transfer, packet_id, &reason));
  EXPECT_FALSE(bindTransferredProtectionLevelPublicationPacket(
      candidate, pl, proof, evidence.reference.mean_world_m,
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
      receipt.state_timestamp.value(), "map", "body_origin", &packet_id));
  EXPECT_FALSE(bindTransferredProtectionLevelPublicationPacket(
      candidate, pl, proof, evidence.reference.mean_world_m,
      actual_state.position_world_m + Eigen::Vector3d::UnitX(),
      certification.transferred_pl_m, receipt.state_timestamp.value(), "map",
      "body_origin", &packet_id));
}

TEST(P005Publication, ImmutablePacketOwnsDeadlineAndAllSinkMaterial) {
  uwb_imu_pl::IntegrityOutput draft;
  draft.attempted_timestamp = uwb_imu_pl::TimestampNs(2000000000);
  draft.state.timestamp = uwb_imu_pl::TimestampNs(1900000000);
  draft.timestamp = draft.state.timestamp;
  draft.publication.protected_output = true;
  draft.publication.unprotected_output = false;
  draft.protection_level.formal_eligible = true;
  draft.reason_codes = {"UPSTREAM"};
  uwb_imu_pl::FinalPacketTiming timing;
  timing.arrival_steady_ns = 100;
  timing.compute_done_steady_ns = 180;
  timing.packet_ready_steady_ns = 240;
  timing.publish_call_steady_ns = 260;
  const auto packet = uwb_imu_pl::finalizeOutputPacket(
      std::move(draft), timing, 150);
  EXPECT_TRUE(packet.output().deadline_missed);
  EXPECT_FALSE(packet.output().publication.protected_output);
  EXPECT_TRUE(packet.output().publication.unprotected_output);
  EXPECT_NEAR(packet.stateAgeSeconds(), 0.1, 1e-15);
  EXPECT_TRUE(packet.output().stale_state);
  EXPECT_EQ(packet.timing().deadline_boundary,
            uwb_imu_pl::FinalPacketBoundary::PublishCall);
  EXPECT_EQ(uwb_imu_pl::finalOutputPacketDigest(packet),
            uwb_imu_pl::finalOutputPacketDigest(packet));
  EXPECT_NE(std::find(packet.output().reason_codes.begin(),
                      packet.output().reason_codes.end(),
                      "PUBLICATION_DEADLINE_MISSED"),
            packet.output().reason_codes.end());
}

TEST(P005Publication, ClockJumpOverflowAndExceptionAreFailClosedPackets) {
  for (const auto kind : {uwb_imu_pl::FinalAttemptKind::ClockInvalid,
                          uwb_imu_pl::FinalAttemptKind::QueueOverflow,
                          uwb_imu_pl::FinalAttemptKind::Exception}) {
    uwb_imu_pl::IntegrityOutput draft;
    draft.attempted_timestamp = uwb_imu_pl::TimestampNs(20);
    draft.state.timestamp = uwb_imu_pl::TimestampNs(10);
    draft.publication.protected_output = true;
    draft.protection_level.formal_eligible = true;
    uwb_imu_pl::FinalPacketTiming timing;
    timing.arrival_steady_ns = 100;
    timing.compute_done_steady_ns = kind == uwb_imu_pl::FinalAttemptKind::ClockInvalid
        ? 90 : 110;
    timing.packet_ready_steady_ns = 120;
    timing.publish_call_steady_ns = 130;
    timing.attempt_kind = kind;
    const auto packet = uwb_imu_pl::finalizeOutputPacket(
        std::move(draft), timing, 1000);
    EXPECT_TRUE(packet.output().deadline_missed ||
                kind != uwb_imu_pl::FinalAttemptKind::ClockInvalid);
    EXPECT_FALSE(packet.output().publication.protected_output);
    EXPECT_TRUE(packet.output().publication.unprotected_output);
    EXPECT_FALSE(packet.output().reason_codes.empty());
  }
}

TEST(P005Publication, SlowPrepareReturnFreezesOneDeadlineMissedPacket) {
  uwb_imu_pl::IntegrityOutput draft;
  draft.attempted_timestamp = uwb_imu_pl::TimestampNs(20);
  draft.state.timestamp = uwb_imu_pl::TimestampNs(20);
  draft.publication.protected_output = true;
  draft.protection_level.formal_eligible = true;
  uwb_imu_pl::FinalPacketTiming timing;
  timing.arrival_steady_ns = 100;
  timing.compute_done_steady_ns = 120;
  timing.packet_ready_steady_ns = 130;
  std::vector<std::uint64_t> clock_values{140, 1170};
  std::size_t clock_index = 0;
  TestPublicationTransaction transaction;
  std::string publication_error;
  const auto terminal = uwb_imu_pl::invokeFinalOutputPacket(
      std::move(draft), timing, 100,
      [&] { return clock_values.at(clock_index++); },
      &transaction,
      &publication_error);
  ASSERT_TRUE(transaction.committed);
  EXPECT_FALSE(transaction.candidate_protected);
  EXPECT_FALSE(transaction.aborted);
  EXPECT_EQ(terminal.timing().publish_call_steady_ns, 140u);
  EXPECT_EQ(terminal.timing().publish_return_steady_ns, 1170u);
  EXPECT_EQ(terminal.timing().publish_outcome,
            uwb_imu_pl::FinalPublishOutcome::Success);
  EXPECT_TRUE(terminal.output().deadline_missed);
  EXPECT_FALSE(terminal.output().publication.protected_output);
  EXPECT_TRUE(terminal.output().publication.unprotected_output);
  EXPECT_EQ(transaction.committed_digest,
            uwb_imu_pl::finalOutputPacketDigest(terminal));
  EXPECT_TRUE(publication_error.empty());
}

TEST(P005Publication, FailedPreparationAbortsBeforeAnySinkSeesAPacket) {
  uwb_imu_pl::IntegrityOutput draft;
  draft.attempted_timestamp = uwb_imu_pl::TimestampNs(20);
  draft.state.timestamp = uwb_imu_pl::TimestampNs(20);
  draft.publication.protected_output = true;
  draft.protection_level.formal_eligible = true;
  uwb_imu_pl::FinalPacketTiming timing;
  timing.arrival_steady_ns = 100;
  timing.compute_done_steady_ns = 120;
  timing.packet_ready_steady_ns = 130;
  std::vector<std::uint64_t> clock_values{140, 170};
  std::size_t clock_index = 0;
  TestPublicationTransaction transaction;
  transaction.candidate_callback = [](const uwb_imu_pl::FinalOutputPacket&) {
    throw std::runtime_error("injected compound preparation failure");
  };
  std::string publication_error;
  const auto terminal = uwb_imu_pl::invokeFinalOutputPacket(
      std::move(draft), timing, 1000,
      [&] { return clock_values.at(clock_index++); }, &transaction,
      &publication_error);
  EXPECT_FALSE(transaction.committed);
  EXPECT_TRUE(transaction.aborted);
  EXPECT_EQ(terminal.timing().publish_outcome,
            uwb_imu_pl::FinalPublishOutcome::PublisherException);
  EXPECT_FALSE(terminal.output().publication.protected_output);
  EXPECT_TRUE(terminal.output().publication.unprotected_output);
  EXPECT_EQ(publication_error, "injected compound preparation failure");
}

TEST(P005Publication,
     PartialSinkFailureCanLeaveOnlyNonAuthoritativeUnprotectedCandidate) {
  uwb_imu_pl::IntegrityOutput draft;
  draft.attempted_timestamp = uwb_imu_pl::TimestampNs(20);
  draft.state.timestamp = uwb_imu_pl::TimestampNs(20);
  draft.publication.protected_output = true;
  draft.protection_level.formal_eligible = true;
  uwb_imu_pl::FinalPacketTiming timing;
  timing.arrival_steady_ns = 100;
  timing.compute_done_steady_ns = 120;
  timing.packet_ready_steady_ns = 130;
  std::vector<std::uint64_t> clock_values{140, 170};
  std::size_t clock_index = 0;
  TestPublicationTransaction transaction;
  bool first_sink_saw_packet = false;
  bool first_sink_saw_protected = true;
  transaction.candidate_callback = [&](
      const uwb_imu_pl::FinalOutputPacket& candidate) {
    first_sink_saw_packet = true;
    first_sink_saw_protected =
        candidate.output().publication.protected_output;
    throw std::runtime_error("injected second sink failure");
  };
  std::string publication_error;
  const auto terminal = uwb_imu_pl::invokeFinalOutputPacket(
      std::move(draft), timing, 1000,
      [&] { return clock_values.at(clock_index++); }, &transaction,
      &publication_error);
  EXPECT_TRUE(first_sink_saw_packet);
  EXPECT_FALSE(first_sink_saw_protected);
  EXPECT_TRUE(transaction.aborted);
  EXPECT_FALSE(transaction.committed);
  EXPECT_EQ(terminal.timing().publish_outcome,
            uwb_imu_pl::FinalPublishOutcome::PublisherException);
  EXPECT_FALSE(terminal.output().publication.protected_output);
  EXPECT_FALSE(terminal.output().protection_level.formal_eligible);
  EXPECT_EQ(publication_error, "injected second sink failure");
}

TEST(P005Publication, NonAtomicFanoutCanNeverActivateProtectedReceipt) {
  uwb_imu_pl::IntegrityOutput draft;
  draft.attempted_timestamp = uwb_imu_pl::TimestampNs(20);
  draft.state.timestamp = uwb_imu_pl::TimestampNs(20);
  draft.publication.protected_output = true;
  draft.publication.unprotected_output = false;
  draft.protection_level.formal_eligible = true;
  uwb_imu_pl::FinalPacketTiming timing;
  timing.arrival_steady_ns = 100;
  timing.compute_done_steady_ns = 110;
  timing.packet_ready_steady_ns = 120;
  std::vector<std::uint64_t> clock_values{130, 140};
  std::size_t index = 0;
  NonAtomicPublicationTransaction transaction;
  const auto terminal = uwb_imu_pl::invokeFinalOutputPacket(
      std::move(draft), timing, 1000,
      [&] { return clock_values.at(index++); }, &transaction);
  EXPECT_FALSE(transaction.candidate_protected);
  EXPECT_FALSE(transaction.receipt_committed);
  EXPECT_FALSE(terminal.output().publication.protected_output);
  EXPECT_FALSE(terminal.output().protection_level.formal_eligible);
  EXPECT_FALSE(terminal.metadata().authoritative);
  EXPECT_EQ(transaction.receipt_digest, 0u);
  EXPECT_NE(std::find(terminal.output().reason_codes.begin(),
                      terminal.output().reason_codes.end(),
                      "ATOMIC_PROTECTED_RECEIPT_UNAVAILABLE"),
            terminal.output().reason_codes.end());
}

TEST(P005Publication,
     ProductionFanoutTimesActualCallsAndNeverClaimsAtomicAuthority) {
  auto draft = [] {
    uwb_imu_pl::IntegrityOutput value;
    value.attempted_timestamp = uwb_imu_pl::TimestampNs(20);
    value.state.timestamp = uwb_imu_pl::TimestampNs(20);
    value.publication.protected_output = true;
    value.publication.unprotected_output = false;
    value.protection_level.formal_eligible = true;
    return value;
  };
  auto timing = [] {
    uwb_imu_pl::FinalPacketTiming value;
    value.arrival_steady_ns = 100;
    value.compute_done_steady_ns = 120;
    value.packet_ready_steady_ns = 130;
    return value;
  };

  ProductionFanoutProbe slow;
  slow.sinks = {
      [](const auto&) {},  // compound
      [](const auto&) {},  // odometry
      [](const auto&) {},  // status
      [](const auto&) {},  // CSV/logger
  };
  std::vector<std::uint64_t> slow_clock{140, 5000};
  std::size_t slow_index = 0;
  const auto slow_terminal = uwb_imu_pl::invokeFinalOutputPacket(
      draft(), timing(), 100,
      [&] { return slow_clock.at(slow_index++); }, &slow);
  EXPECT_EQ(slow_terminal.timing().publish_return_steady_ns, 5000u);
  EXPECT_TRUE(slow_terminal.output().deadline_missed);
  EXPECT_FALSE(slow_terminal.output().publication.protected_output);
  EXPECT_FALSE(slow_terminal.metadata().authoritative);
  EXPECT_FALSE(slow.commit_called);
  ASSERT_EQ(slow.visible_digests.size(), 4u);
  for (const auto digest : slow.visible_digests)
    EXPECT_EQ(digest, slow_terminal.metadata().digest);

  ProductionFanoutProbe partial;
  bool first_visible = false;
  partial.sinks = {
      [&](const auto& packet) {
        first_visible = true;
        EXPECT_FALSE(packet.output().publication.protected_output);
        EXPECT_FALSE(packet.metadata().authoritative);
      },
      [](const auto&) { throw std::runtime_error("second ROS sink failed"); },
      [](const auto&) {},
  };
  std::vector<std::uint64_t> partial_clock{140, 170};
  std::size_t partial_index = 0;
  std::string error;
  const auto partial_terminal = uwb_imu_pl::invokeFinalOutputPacket(
      draft(), timing(), 1000,
      [&] { return partial_clock.at(partial_index++); }, &partial, &error);
  EXPECT_TRUE(first_visible);
  EXPECT_TRUE(partial.aborted);
  EXPECT_FALSE(partial.commit_called);
  EXPECT_FALSE(partial_terminal.metadata().authoritative);
  EXPECT_FALSE(partial_terminal.output().publication.protected_output);
  EXPECT_EQ(error, "second ROS sink failed");
  ASSERT_EQ(partial.visible_digests.size(), 1u);
  EXPECT_EQ(partial.visible_digests.front(), partial_terminal.metadata().digest);

  ProductionFanoutProbe logger_failure;
  logger_failure.sinks = {
      [](const auto&) {}, [](const auto&) {}, [](const auto&) {},
      [](const auto&) { throw std::runtime_error("CSV/logger failed"); },
  };
  std::vector<std::uint64_t> logger_clock{140, 180};
  std::size_t logger_index = 0;
  error.clear();
  const auto logger_terminal = uwb_imu_pl::invokeFinalOutputPacket(
      draft(), timing(), 1000,
      [&] { return logger_clock.at(logger_index++); },
      &logger_failure, &error);
  EXPECT_TRUE(logger_failure.aborted);
  EXPECT_FALSE(logger_failure.commit_called);
  EXPECT_FALSE(logger_terminal.metadata().authoritative);
  EXPECT_FALSE(logger_terminal.output().publication.protected_output);
  EXPECT_EQ(error, "CSV/logger failed");
}

TEST(P005Odometry, NinetyDegreeYawRotatesVelocityAndCovarianceToChildFrame) {
  constexpr double kPi = 3.14159265358979323846;
  const Eigen::Quaterniond q_world_body(
      Eigen::AngleAxisd(0.5 * kPi, Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d covariance_world = Eigen::Matrix3d::Zero();
  covariance_world << 1.0, 0.2, 0.3,
                      0.2, 4.0, 0.5,
                      0.3, 0.5, 9.0;
  const auto child = uwb_imu_pl::childFrameLinearTwist(
      q_world_body, Eigen::Vector3d(1, 0, 0), covariance_world);
  EXPECT_TRUE(child.velocity_body_mps.isApprox(Eigen::Vector3d(0, -1, 0),
                                               1e-12));
  Eigen::Matrix3d expected;
  expected << 4.0, -0.2, 0.5,
             -0.2,  1.0, -0.3,
              0.5, -0.3, 9.0;
  EXPECT_TRUE(child.covariance_body_m2ps2.isApprox(expected, 1e-12));
}

TEST(P005Abi, PublicationPacketV1AndV2LayoutsRemainFrozen) {
  EXPECT_EQ(sizeof(uwb_imu_pl::PublicationDiagnostics), 424u);
  EXPECT_EQ(offsetof(uwb_imu_pl::PublicationDiagnostics, certificate_id),
            416u);
  EXPECT_EQ(sizeof(uwb_imu_pl::IntegrityOutput), 4128u);
  EXPECT_EQ(offsetof(uwb_imu_pl::IntegrityOutput, bridge_audit), 3864u);
  EXPECT_EQ(sizeof(uwb_imu_pl::ProtectionLevelPublicationPacketV1), 88u);
  EXPECT_EQ(offsetof(uwb_imu_pl::ProtectionLevelPublicationPacketV1,
                     packet_identity), 80u);
  EXPECT_EQ(sizeof(uwb_imu_pl::ProtectionLevelPublicationPacketV2), 256u);
  EXPECT_EQ(offsetof(uwb_imu_pl::ProtectionLevelPublicationPacketV2,
                     packet_identity), 248u);
}
