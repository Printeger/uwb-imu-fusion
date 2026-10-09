#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <boost/filesystem.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <future>
#include <mutex>
#include <new>
#include <random>
#include <thread>

#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/candidate_replay.hpp"
#include "uwb_imu_pl/integrity/fde_post_selection.hpp"
#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/estimation/reinitialization.hpp"
#include "uwb_imu_pl/factors/kinematic_bridge_factor.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include "uwb_imu_pl/integrity/coverage_envelope.hpp"
#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"
#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/health_manager.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"
#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"
#include "uwb_imu_pl/publication/final_output_packet.hpp"

namespace {

uwb_imu_pl::LinearizedFactorBlock block(
    std::uint64_t id, const Eigen::MatrixXd& h, const Eigen::VectorXd& z,
    const uwb_imu_pl::LinearizationVersion& version) {
  uwb_imu_pl::LinearizedFactorBlock value;
  value.group_id = uwb_imu_pl::FactorGroupId(id);
  value.jacobian_whitened = h;
  value.residual_whitened = z;
  value.jacobian_raw = h;
  value.residual_raw = z;
  value.covariance = Eigen::MatrixXd::Identity(h.rows(), h.rows());
  value.whitener = value.covariance;
  value.version = version;
  return value;
}

uwb_imu_pl::LinearizedIntegrityWindow syntheticWindow() {
  uwb_imu_pl::LinearizedIntegrityWindow window;
  window.id = uwb_imu_pl::WindowId(4);
  window.version = {7, 2, 1, 9};
  Eigen::MatrixXd h0 = Eigen::MatrixXd::Identity(2, 2) * 3.0;
  Eigen::MatrixXd h1(3, 2);
  h1 << 1.0, 0.2, 0.1, 1.0, 0.7, -0.4;
  Eigen::MatrixXd h2(2, 2);
  h2 << 0.3, 0.8, -0.6, 0.2;
  window.blocks.push_back(
      block(1, h0, Eigen::Vector2d(0.1, -0.2), window.version));
  window.blocks.push_back(
      block(2, h1, Eigen::Vector3d(0.3, -0.1, 0.2), window.version));
  window.blocks.push_back(
      block(3, h2, Eigen::Vector2d(-0.2, 0.4), window.version));
  window.protected_state_map = Eigen::MatrixXd::Zero(3, 2);
  window.protected_state_map(0, 0) = 1.0;
  window.protected_state_map(1, 1) = 1.0;
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-10, 1e10);
  return window;
}

uwb_imu_pl::LinearizedIntegrityWindow p002HistoryWindow(
    double history_residual, double history_constant) {
  uwb_imu_pl::LinearizedIntegrityWindow window;
  window.id = uwb_imu_pl::WindowId(2002);
  window.version = {20, 2, 0, 2};
  for (std::uint64_t id = 1; id <= 3; ++id) {
    window.blocks.push_back(block(
        id, Eigen::Matrix<double, 1, 1>::Ones(),
        Eigen::VectorXd::Zero(1), window.version));
  }
  auto history = block(
      4, Eigen::Matrix<double, 1, 1>::Zero(),
      Eigen::VectorXd::Constant(1, history_residual), window.version);
  history.kind = uwb_imu_pl::FactorKind::BoundaryPrior;
  history.sensor = uwb_imu_pl::SensorType::Prior;
  history.role = uwb_imu_pl::RowRole::TrustedPrior;
  history.whitening_model_id = "history_summary_sqrt_d1";
  window.blocks.push_back(std::move(history));
  window.history_summary.present = true;
  window.history_summary.valid = true;
  window.history_summary.nu_perp = 1;
  window.history_summary.kappa_b = history_residual * history_residual;
  window.history_summary.constant_offset = history_constant;
  window.history_summary.emitted_rows = 1;
  window.history_summary.detector_response = Eigen::MatrixXd::Zero(1, 0);
  window.protected_state_map = Eigen::MatrixXd::Ones(3, 1);
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-12, 1e10);
  return window;
}

struct P002RawOracle {
  Eigen::MatrixXd h;
  Eigen::VectorXd z;
  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;
  std::vector<uwb_imu_pl::CandidateDetectorRowRole> row_roles;
  double current_statistic = 0.0;
  double history_statistic = 0.0;
  int current_rank = 0;
  int current_dof = 0;
  int history_rank = 0;
  int history_dof = 0;
};

P002RawOracle p002IndependentRawOracle(
    const uwb_imu_pl::LinearizedIntegrityWindow& window,
    const uwb_imu_pl::ExclusionAction& action, double rank_tolerance) {
  using uwb_imu_pl::CandidateDetectorRowRole;
  P002RawOracle out;
  std::vector<const uwb_imu_pl::LinearizedFactorBlock*> retained;
  auto removed = [&](uwb_imu_pl::FactorGroupId id) {
    return std::find(action.groups_to_remove.begin(),
                     action.groups_to_remove.end(), id) !=
        action.groups_to_remove.end();
  };
  for (const auto& value : window.blocks) {
    if (!removed(value.group_id)) retained.push_back(&value);
  }
  for (const auto& value : action.added_blocks) retained.push_back(&value);
  int rows = 0;
  for (const auto* value : retained) rows += value->jacobian_whitened.rows();
  out.h.resize(rows, window.H.cols());
  out.z.resize(rows);
  int offset = 0;
  for (const auto* value : retained) {
    const int count = value->jacobian_whitened.rows();
    out.h.middleRows(offset, count) = value->jacobian_whitened;
    out.z.segment(offset, count) = value->residual_whitened;
    const int history_rows =
        value->whitening_model_id == "history_summary_sqrt_d1"
        ? window.history_summary.nu_perp : 0;
    out.row_roles.insert(
        out.row_roles.end(), count - history_rows,
        CandidateDetectorRowRole::CurrentStateSupported);
    out.row_roles.insert(
        out.row_roles.end(), history_rows,
        CandidateDetectorRowRole::HistoryDetectorOnly);
    offset += count;
  }
  // Independent O05 reference: every state/covariance RHS is served directly
  // from raw-H SVD.  No production certificate/oracle and no normal-equation
  // covariance solve is used here.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      out.h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd inverse = svd.singularValues().cwiseInverse();
  out.state = svd.matrixV() * inverse.asDiagonal() *
      svd.matrixU().transpose() * out.z;
  out.covariance = svd.matrixV() * inverse.array().square().matrix().asDiagonal() *
      svd.matrixV().transpose();
  const Eigen::VectorXd residual = out.z - out.h * out.state;
  std::vector<Eigen::Index> current_rows;
  std::vector<Eigen::Index> history_rows;
  for (Eigen::Index row = 0; row < residual.size(); ++row) {
    if (out.row_roles[static_cast<std::size_t>(row)] ==
        CandidateDetectorRowRole::HistoryDetectorOnly) {
      history_rows.push_back(row);
      out.history_statistic += residual(row) * residual(row);
    } else {
      current_rows.push_back(row);
      out.current_statistic += residual(row) * residual(row);
    }
  }
  out.history_statistic += window.history_summary.constant_offset;
  auto channel_rank = [&](const std::vector<Eigen::Index>& selected) {
    Eigen::MatrixXd channel(selected.size(), out.h.cols());
    for (std::size_t row = 0; row < selected.size(); ++row) {
      channel.row(static_cast<Eigen::Index>(row)) = out.h.row(selected[row]);
    }
    if (channel.rows() == 0) return 0;
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> channel_qr(channel);
    channel_qr.setThreshold(rank_tolerance);
    return static_cast<int>(channel_qr.rank());
  };
  out.current_rank = channel_rank(current_rows);
  out.current_dof = static_cast<int>(current_rows.size()) - out.current_rank;
  out.history_rank = channel_rank(history_rows);
  out.history_dof = static_cast<int>(history_rows.size()) - out.history_rank;
  return out;
}

Eigen::Vector3d p002IndependentPlOracle(
    const uwb_imu_pl::LinearizedIntegrityWindow& window,
    const P002RawOracle& raw, const Eigen::MatrixXd& fault_map,
    const uwb_imu_pl::DetectorRiskContext& detector_risk,
    const uwb_imu_pl::FaultHypothesisV2& hypothesis,
    const uwb_imu_pl::RiskBudgetV2& risk) {
  Eigen::JacobiSVD<Eigen::MatrixXd> raw_svd(
      raw.h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd inverse = raw_svd.singularValues().cwiseInverse();
  const Eigen::MatrixXd coordinates =
      raw_svd.matrixU().transpose() * fault_map;
  const Eigen::MatrixXd residual_factor =
      fault_map - raw_svd.matrixU() * coordinates;
  const Eigen::MatrixXd gram =
      residual_factor.transpose() * residual_factor;
  Eigen::MatrixXd history_gram = Eigen::MatrixXd::Zero(
      fault_map.cols(), fault_map.cols());
  for (Eigen::Index row = 0; row < fault_map.rows(); ++row) {
    if (raw.row_roles[static_cast<std::size_t>(row)] ==
        uwb_imu_pl::CandidateDetectorRowRole::HistoryDetectorOnly) {
      history_gram.noalias() += fault_map.row(row).transpose() *
                                fault_map.row(row);
    }
  }
  const Eigen::MatrixXd protected_response = window.protected_state_map *
      raw_svd.matrixV() * inverse.asDiagonal() * coordinates;
  const Eigen::Matrix3d protected_covariance =
      window.protected_state_map * raw.covariance *
      window.protected_state_map.transpose();
  const Eigen::Vector3d sigma =
      protected_covariance.diagonal().cwiseSqrt();
  const double hypothesis_tail = std::min(
      0.5, hypothesis.hmi_allocation /
          hypothesis.prior_probability_bound);
  const double axis_tail = hypothesis_tail / 3.0;
  const boost::math::normal_distribution<double> standard_normal;
  const double k = boost::math::quantile(
      standard_normal, 1.0 - axis_tail / 2.0);
  auto threshold = [&](int dof) {
    const boost::math::chi_squared_distribution<double> central(dof);
    return boost::math::quantile(
        boost::math::complement(central, detector_risk.p_fa_per_test));
  };
  auto noncentrality = [&](int dof, double tau) {
    auto miss = [&](double lambda) {
      const boost::math::non_central_chi_squared_distribution<double>
          distribution(dof, lambda);
      return boost::math::cdf(distribution, tau);
    };
    double low = 0.0;
    double high = 1.0;
    while (miss(high) > hypothesis.p_md_allocation && high < 1e12) {
      high *= 2.0;
    }
    for (int iteration = 0; iteration < 200; ++iteration) {
      const double middle = 0.5 * (low + high);
      if (miss(middle) > hypothesis.p_md_allocation) low = middle;
      else high = middle;
    }
    return high;
  };
  std::vector<std::pair<Eigen::MatrixXd, double>> channels;
  const Eigen::MatrixXd current_gram = gram - history_gram;
  if (current_gram.norm() > 0.0) {
    channels.emplace_back(
        current_gram,
        noncentrality(raw.current_dof, threshold(raw.current_dof)));
  }
  if (raw.history_dof > 0 && history_gram.norm() > 0.0) {
    channels.emplace_back(
        history_gram,
        noncentrality(raw.history_dof, threshold(raw.history_dof)));
  }
  if (channels.empty()) {
    return Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
  }
  Eigen::MatrixXd w = Eigen::MatrixXd::Zero(gram.rows(), gram.cols());
  const double weight = 1.0 / static_cast<double>(channels.size());
  for (const auto& channel : channels) {
    w.noalias() += weight / channel.second * channel.first;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(w, Eigen::ComputeThinV);
  const Eigen::VectorXd singular = svd.singularValues();
  const double floor = singular.size() ? 1e-12 * singular(0) : 0.0;
  Eigen::Vector3d fault_component = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::VectorXd response = protected_response.row(axis).transpose();
    double quadratic = 0.0;
    for (Eigen::Index index = 0; index < singular.size(); ++index) {
      if (singular(index) <= floor) continue;
      const double projection = svd.matrixV().col(index).dot(response);
      quadratic += projection * projection / singular(index);
    }
    fault_component(axis) = std::sqrt(std::max(0.0, quadratic)) +
                            k * sigma(axis);
  }
  const double nominal_k = boost::math::quantile(
      standard_normal, 1.0 - risk.nominal_axis_tail / 2.0);
  return fault_component.cwiseMax(nominal_k * sigma);
}

uwb_imu_pl::IntegrityConfig researchConfig() {
  return uwb_imu_pl::IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/config/realtime_uwb_imu_pl_research.yaml");
}

uwb_imu_pl::UwbBatch batch(const uwb_imu_pl::IntegrityConfig& config,
                           std::int64_t time_ns) {
  uwb_imu_pl::UwbBatch value;
  value.id = uwb_imu_pl::BatchId(static_cast<std::uint64_t>(time_ns));
  value.timestamp = uwb_imu_pl::TimestampNs(time_ns);
  for (std::size_t i = 0; i < config.anchors.size(); ++i) {
    uwb_imu_pl::UwbMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(i + 1);
    measurement.factor_id = uwb_imu_pl::FactorId(i + 1);
    measurement.anchor_id = config.anchors[i].id;
    measurement.timestamp = value.timestamp;
    measurement.anchor_position_m = config.anchors[i].position_world_m;
    measurement.range_m =
        (measurement.anchor_position_m - Eigen::Vector3d(0, 0, 1)).norm();
    measurement.sigma_m = 0.05;
    value.measurements.push_back(measurement);
  }
  return value;
}

void addImu(uwb_imu_pl::IncrementalUwbImuEstimator* estimator, double gravity) {
  for (int i = 0; i <= 2; ++i) {
    uwb_imu_pl::ImuMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(100 + i);
    measurement.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    measurement.specific_force_mps2 = {0, 0, gravity};
    estimator->ingestImu(measurement);
  }
}

}  // namespace

TEST(FdeProfiles, NominalPipelineAndFaultCensusFollowResolvedScope) {
  struct Expectation {
    const char* name;
    bool uwb;
    bool imu;
    bool pairs;
  };
  const Expectation expectations[] = {
      {"off", false, false, false},
      {"uwb_order1", true, false, false},
      {"imu_order1", false, true, false},
      {"joint_order1", true, true, false},
      {"joint_order2", true, true, true},
  };
  for (const auto& expected : expectations) {
    SCOPED_TRACE(expected.name);
    auto config = uwb_imu_pl::IntegrityConfigLoader::load(
        std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_" +
        expected.name + ".yaml");
    uwb_imu_pl::NavigationState initial;
    initial.timestamp = uwb_imu_pl::TimestampNs(0);
    initial.position_world_m = {0, 0, 1};
    uwb_imu_pl::IncrementalUwbImuEstimator estimator(
        config, config.realtime.lever_arm_body_m);
    estimator.initialize(initial, config.realtime.prior_sigmas);
    uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
        &estimator,
        uwb_imu_pl::IntegrityMonitor(config.risk,
                                     config.snapshot.rank_tolerance,
                                     config.snapshot.max_condition_number),
        uwb_imu_pl::offlineReplayPublicationLimits());
    for (int i = 0; i <= 2; ++i) {
      uwb_imu_pl::ImuMeasurement measurement;
      measurement.id = uwb_imu_pl::MeasurementId(100 + i);
      measurement.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
      measurement.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
      pipeline.ingestImu(measurement);
    }
    const auto output = pipeline.processUwbBatch(batch(config, 10000000));
    const std::string logging_directory =
        std::string("/tmp/uwb_imu_pl_profile_logging_") + expected.name;
    boost::filesystem::remove_all(logging_directory);
    uwb_imu_pl::RunLoggingSession disabled_logging;
    disabled_logging.writeIntegrity(output);
    {
      uwb_imu_pl::RunLoggingSession enabled_logging;
      enabled_logging.enable(logging_directory, false, false);
      enabled_logging.writeIntegrity(output);
    }
    // Logging is downstream of the immutable algorithm result. Enabling it
    // must not alter profile identity, transaction, census or PL state.
    EXPECT_EQ(output.fde_profile, expected.name);
    EXPECT_EQ(output.scope_digest, config.resolved_scope.scope_digest);
    EXPECT_EQ(output.backend_updates, 1u);
    EXPECT_TRUE(output.batch_committed);
    EXPECT_EQ(output.state.timestamp, uwb_imu_pl::TimestampNs(10000000));
    EXPECT_TRUE(output.fresh);
    EXPECT_EQ(output.transaction_id, 1u);
    EXPECT_EQ(output.diagnostics.single_uwb_hypotheses > 0, expected.uwb);
    EXPECT_EQ(output.diagnostics.single_accel_hypotheses > 0, expected.imu);
    boost::filesystem::remove_all(logging_directory);
    if (std::string(expected.name) == "off") {
      EXPECT_EQ(output.fde_status, "FDE_DISABLED");
      EXPECT_EQ(output.detector.detector_type, "NOT_RUN");
      EXPECT_EQ(output.pl_status,
                uwb_imu_pl::ProtectionLevelStatus::NotComputed);
      EXPECT_FALSE(output.protection_level.pl_xyz_m.allFinite());
      EXPECT_EQ(output.diagnostics.hypothesis_count, 0u);
      EXPECT_EQ(output.diagnostics.generated_actions, 0u);
      continue;
    }
    EXPECT_EQ(output.detector.detector_type, "dual_channel_v1");
    EXPECT_EQ(output.protection_level.risk_budget_valid,
              output.diagnostics.risk_ledger_closes &&
                  output.diagnostics.risk_ledger_all_validated);
    EXPECT_FALSE(output.protection_level.risk_budget_valid);
    EXPECT_FALSE(output.protection_level.formal_eligible);
    EXPECT_EQ(output.protection_level.availability,
              uwb_imu_pl::Availability::Unavailable);
    EXPECT_EQ(output.diagnostics.single_uwb_hypotheses > 0, expected.uwb);
    EXPECT_EQ(output.diagnostics.single_accel_hypotheses > 0, expected.imu);
    EXPECT_EQ(output.diagnostics.single_gyro_hypotheses > 0, expected.imu);
    EXPECT_EQ(output.diagnostics.double_uwb_accel_hypotheses > 0,
              expected.pairs);
    EXPECT_EQ(output.diagnostics.double_uwb_gyro_hypotheses > 0,
              expected.pairs);
    EXPECT_EQ(output.diagnostics.effective_fault_cardinality,
              expected.pairs ? 2u : 1u);
    if (std::string(expected.name) == "uwb_order1") {
      // Regression: disabling the IMU fault provider must not remove the
      // nominal IMU group from historical provenance on the next epoch.
      for (int i = 3; i <= 4; ++i) {
        uwb_imu_pl::ImuMeasurement measurement;
        measurement.id = uwb_imu_pl::MeasurementId(100 + i);
        measurement.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
        measurement.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
        pipeline.ingestImu(measurement);
      }
      auto next_batch = batch(config, 20000000);
      next_batch.id = uwb_imu_pl::BatchId(2);
      for (std::size_t i = 0; i < next_batch.measurements.size(); ++i) {
        next_batch.measurements[i].id = uwb_imu_pl::MeasurementId(200 + i);
        next_batch.measurements[i].factor_id = uwb_imu_pl::FactorId(200 + i);
      }
      const auto next = pipeline.processUwbBatch(next_batch);
      EXPECT_TRUE(next.batch_committed);
      EXPECT_EQ(std::find(next.reason_codes.begin(), next.reason_codes.end(),
                          "provenance is incomplete"),
                next.reason_codes.end());
    }
  }
}

TEST(P103ProofArena,
     ScopedAttemptFreezesSelfContainedBundleAndLegacyPathRemainsCompatible) {
  using namespace uwb_imu_pl;
  const auto frozen_window = freezeIntegrityWindowCopy(
      p002HistoryWindow(0.0, 0.0));
  const auto admission = admitFrozenIntegrityWindow(frozen_window);
  ASSERT_TRUE(admission) << admission.reason;
  const auto& window = admission.window();
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.continuity_horizon_tests = 1000;
  detector_risk.rank_tolerance = 1e-12;
  detector_risk.max_condition_number = 1e10;
  ExclusionAction action;
  action.id = ExclusionActionId(10301);
  action.action_model_id = "KEEP_ALL";
  RankUpdateConfig rank_config;
  rank_config.rank_tolerance = 1e-12;
  rank_config.max_condition_number = 1e10;
  rank_config.max_linearization_step_norm = 10.0;
  auto scoped_candidate = DenseCandidateOracle(rank_config).evaluate(
      admission, action);
  auto legacy_candidate = DenseCandidateOracle(rank_config).evaluate(
      admission, action);
  ASSERT_TRUE(scoped_candidate.valid);
  const auto detector = JointWindowDetector().evaluateCandidate(
      window, scoped_candidate, detector_risk);
  ASSERT_TRUE(detector.passed) << detector.reason;
  Eigen::MatrixXd mode = Eigen::MatrixXd::Zero(scoped_candidate.rows, 1);
  mode(0, 0) = 1.0;
  mode(mode.rows() - 1, 0) = 1.0;
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(10302);
  hypothesis.modes = {FaultModeId(10303)};
  hypothesis.A = mode;
  hypothesis.p_md_allocation = 1e-3;
  hypothesis.hmi_allocation = 1e-6;
  hypothesis.prior_probability_bound = 1e-3;
  std::vector<FaultHypothesisV2> scoped_hypotheses{hypothesis};
  std::vector<FaultHypothesisV2> legacy_hypotheses{hypothesis};
  ProtectionLevelSharedContext shared;
  shared.mode_maps.emplace(10303, mode);
  AttemptProofArena arena;
  ProtectionLevelV2ProofV1 audit;
  const auto scoped = ProtectionLevelV2().computeShared(
      window, &scoped_candidate, detector, &scoped_hypotheses, shared,
      RiskBudgetV2{}, &audit, &arena);
  ASSERT_TRUE(scoped.model_valid) << scoped.reason;
  ASSERT_TRUE(validateProtectionLevelV2Proof(
      scoped_candidate, detector, scoped_hypotheses, scoped, arena, nullptr));
  const auto scoped_identity = protectionLevelV2ProofIdentity(
      scoped_candidate, scoped, arena);
  ASSERT_NE(scoped_identity, 0u);
  EpochTransaction transaction;
  transaction.id = TransactionId(window.id.value());
  transaction.base_version = window.version;
  transaction.end = TimestampNs(1030000000);
  transaction.nominal_predicted_state.position_world_m =
      Eigen::Vector3d(4.0, -3.0, 2.0);
  CommitProtectionEvidenceV1 unconsumed;
  std::string reason;
  ASSERT_TRUE(mintCommitProtectionEvidenceV1(
      transaction, admission, scoped_candidate, scoped, scoped_identity,
      "map", &arena, &unconsumed, &reason)) << reason;
  EXPECT_EQ(detail::AttemptProofArenaAccess::liveConsumableCountForTesting(),
            1u);
  const Eigen::Vector3d reference(1.0, 2.0, 3.0);
  const Eigen::Vector3d committed(1.1, 1.8, 3.3);
  const Eigen::Vector3d transferred = scoped.pl_xyz_m +
      (committed - reference).cwiseAbs();
  std::string packet_id;
  ASSERT_TRUE(bindTransferredProtectionLevelPublicationPacket(
      scoped_candidate, scoped, scoped_identity, reference, committed,
      transferred, 1030000000, "map", "body_origin", &arena, &packet_id));
  AttemptProofLease lease = arena.lease();
  arena.close();
  EXPECT_EQ(detail::AttemptProofArenaAccess::liveConsumableCountForTesting(),
            0u);
  EXPECT_FALSE(consumeCommitProtectionEvidenceV1(
      unconsumed, transaction, "map", &reason));
  FinalProtectionProofBundleV1 bundle;
  ASSERT_TRUE(freezeFinalProtectionProofBundleV1(
      lease, packet_id, &bundle, &reason)) << reason;
  const auto retained_entries = lease.entryCount();
  lease.reset();
  ASSERT_TRUE(validateFinalProtectionProofBundleV1(
      bundle, transferred, packet_id, &reason)) << reason;
  EXPECT_GT(retained_entries, 0u);
  IntegrityOutput terminal_output;
  terminal_output.attempted_timestamp = TimestampNs(1030000000);
  terminal_output.state.timestamp = terminal_output.attempted_timestamp;
  terminal_output.protection_level.pl_xyz_m = transferred;
  terminal_output.protection_level.detector_certificate_id = packet_id;
  FinalPacketTiming timing;
  timing.arrival_steady_ns = 1;
  timing.compute_done_steady_ns = 2;
  timing.packet_ready_steady_ns = 3;
  const auto final_packet = finalizeOutputPacket(
      terminal_output, timing, 100, bundle);
  EXPECT_EQ(final_packet.metadata().protection_packet_id, packet_id);
  EXPECT_EQ(final_packet.metadata().protected_frame_id, "map");
  EXPECT_EQ(final_packet.metadata().protected_position_reference,
            "body_origin");
  CommittedStatePublicationV1 committed_sidecar;
  committed_sidecar.transaction_id = transaction.id.value();
  committed_sidecar.state_timestamp = transaction.end;
  committed_sidecar.covariance.setIdentity();
  AttemptProofArena committed_arena;
  ASSERT_TRUE(recordCommittedStatePublicationV1(
      committed_sidecar, &committed_arena));
  auto committed_lease = committed_arena.lease();
  committed_arena.close();
  CommittedStatePublicationV1 recovered_sidecar;
  ASSERT_TRUE(committedStatePublicationV1(
      committed_sidecar.transaction_id, committed_sidecar.state_timestamp,
      committed_lease, &recovered_sidecar));
  EXPECT_TRUE(recovered_sidecar.covariance.isApprox(
      committed_sidecar.covariance, 0.0));
  auto tampered_bundle = bundle;
  tampered_bundle.publication_packet.served_pl.x() = std::nextafter(
      tampered_bundle.publication_packet.served_pl.x(),
      std::numeric_limits<double>::infinity());
  EXPECT_FALSE(validateFinalProtectionProofBundleV1(
      tampered_bundle, transferred, packet_id, &reason));

  std::vector<std::future<bool>> readers;
  for (int worker = 0; worker < 4; ++worker) {
    readers.push_back(std::async(std::launch::async, [&, worker] {
      std::string local_reason;
      auto local = bundle;
      if (worker == 3) local.arena_generation = bundle.arena_generation;
      return validateFinalProtectionProofBundleV1(
          local, transferred, packet_id, &local_reason);
    }));
  }
  for (auto& reader : readers) EXPECT_TRUE(reader.get());

  const auto legacy = ProtectionLevelV2().computeShared(
      window, &legacy_candidate, detector, &legacy_hypotheses, shared,
      RiskBudgetV2{});
  ASSERT_TRUE(legacy.model_valid) << legacy.reason;
  EXPECT_TRUE(scoped.pl_xyz_m.isApprox(legacy.pl_xyz_m, 0.0));
  EXPECT_EQ(scoped.detector_certificate_id, legacy.detector_certificate_id);
  EXPECT_EQ(scoped.risk_budget_valid, legacy.risk_budget_valid);
  EXPECT_EQ(scoped.availability, legacy.availability);
}

TEST(P104ProofOwnership,
     BatchArenaHighWaterTracksCapacityAndOnlyWinnerReachesMainArena) {
  using namespace uwb_imu_pl;
  const auto frozen_window = freezeIntegrityWindowCopy(
      p002HistoryWindow(0.0, 0.0));
  const auto admission = admitFrozenIntegrityWindow(frozen_window);
  ASSERT_TRUE(admission) << admission.reason;
  const auto& window = admission.window();
  RankUpdateConfig rank_config;
  rank_config.rank_tolerance = 1e-12;
  rank_config.max_condition_number = 1e10;
  rank_config.max_linearization_step_norm = 10.0;
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.continuity_horizon_tests = 1000;
  detector_risk.rank_tolerance = rank_config.rank_tolerance;
  detector_risk.max_condition_number = rank_config.max_condition_number;

  auto exercise = [&](std::size_t capacity) {
    constexpr std::size_t kActions = 65;
    std::vector<ExclusionAction> actions;
    actions.reserve(kActions);
    for (std::size_t index = 0; index < kActions; ++index) {
      ExclusionAction action;
      action.id = ExclusionActionId(index + 1);
      action.action_model_id = "P104_PROOF_OWNER_" +
          std::to_string(index + 1);
      LinearizedFactorBlock added;
      added.group_id = FactorGroupId(104000 + index);
      added.kind = FactorKind::UwbBatch;
      added.sensor = SensorType::Uwb;
      added.role = RowRole::Measurement;
      added.window_column_indices = {0};
      added.jacobian_whitened = Eigen::MatrixXd::Constant(
          1, 1, 0.01 + 0.0001 * static_cast<double>(index));
      added.residual_whitened = Eigen::VectorXd::Constant(
          1, 1e-5 * static_cast<double>(index));
      added.effective_weight = 1.0;
      added.whitening_model_id = "p104-proof-owner";
      added.version = window.version;
      action.groups_to_add = {added.group_id};
      action.added_blocks.push_back(std::move(added));
      actions.push_back(std::move(action));
    }
    RankUpdateEvaluator evaluator(rank_config);
    auto base = evaluator.factorizeOnce(admission, actions);
    ASSERT_TRUE(base.valid) << base.reason;
    AttemptProofArena main_arena;
    std::size_t peak = 0;
    std::set<std::uint64_t> all_proof_ids;
    for (std::size_t begin = 0; begin < actions.size(); begin += capacity) {
      AttemptProofArena batch_arena;
      const std::size_t end = std::min(actions.size(), begin + capacity);
      for (std::size_t index = begin; index < end; ++index) {
        auto candidate = DenseCandidateOracle(rank_config).evaluate(
            admission, actions[index]);
        ASSERT_TRUE(candidate.valid) << index << ": " << candidate.reason;
        const auto detector = JointWindowDetector().evaluateCandidate(
            window, candidate, detector_risk);
        ASSERT_TRUE(detector.passed) << index << ": " << detector.reason;
        Eigen::MatrixXd mode = Eigen::MatrixXd::Zero(candidate.rows, 1);
        mode(0, 0) = 1.0;
        mode(mode.rows() - 1, 0) = 1.0;
        FaultHypothesisV2 hypothesis;
        hypothesis.id = HypothesisId(104500 + index);
        hypothesis.modes = {FaultModeId(105000 + index)};
        hypothesis.A = mode;
        hypothesis.p_md_allocation = 1e-3;
        hypothesis.hmi_allocation = 1e-6;
        hypothesis.prior_probability_bound = 1e-3;
        std::vector<FaultHypothesisV2> hypotheses{hypothesis};
        ProtectionLevelSharedContext shared;
        shared.mode_maps.emplace(hypothesis.modes.front().value(), mode);
        ProtectionLevelV2ProofV1 proof;
        const auto result = ProtectionLevelV2().computeShared(
            admission, &candidate, detector, &hypotheses, shared,
            RiskBudgetV2{}, &proof, &batch_arena);
        ASSERT_TRUE(result.model_valid) << index << ": " << result.reason;
        ASSERT_NE(proof.proof_identity, 0u);
        EXPECT_TRUE(all_proof_ids.insert(proof.proof_identity).second);
        if (index + 1 == actions.size()) {
          std::string reason;
          ASSERT_TRUE(retainProtectionLevelV2Proof(
              candidate, result, proof, &main_arena, &reason)) << reason;
        }
      }
      const std::size_t resident = protectionLevelV2FullProofCount(batch_arena);
      peak = std::max(peak, resident);
      EXPECT_EQ(resident, end - begin);
      EXPECT_LE(resident, capacity);
      batch_arena.close();
      EXPECT_EQ(protectionLevelV2FullProofCount(main_arena),
                end == actions.size() ? 1u : 0u);
    }
    EXPECT_EQ(all_proof_ids.size(), kActions);
    EXPECT_EQ(peak, capacity);
    EXPECT_EQ(protectionLevelV2FullProofCount(main_arena), 1u);
  };

  exercise(7);
  exercise(31);
}

TEST(P103ProofArena, CloseConsumeRaceHasOneLinearizedWinner) {
  using namespace uwb_imu_pl;
  constexpr std::uint32_t kind = 103;
  for (std::uint64_t iteration = 1; iteration <= 128; ++iteration) {
    AttemptProofArena arena;
    ASSERT_TRUE(detail::AttemptProofArenaAccess::registerConsumable(
        &arena, kind, iteration,
        std::make_shared<const std::uint64_t>(iteration)));
    std::atomic<bool> start{false};
    std::shared_ptr<const void> consumed;
    auto closer = std::async(std::launch::async, [&] {
      while (!start.load(std::memory_order_acquire)) {}
      arena.close();
    });
    auto consumer = std::async(std::launch::async, [&] {
      while (!start.load(std::memory_order_acquire)) {}
      consumed = detail::AttemptProofArenaAccess::consumeConsumable(
          kind, iteration);
    });
    start.store(true, std::memory_order_release);
    closer.get();
    consumer.get();
    EXPECT_TRUE(arena.closed());
    if (consumed) {
      EXPECT_EQ(*std::static_pointer_cast<const std::uint64_t>(consumed),
                iteration);
    } else {
      EXPECT_FALSE(detail::AttemptProofArenaAccess::consumeConsumable(
          kind, iteration));
    }
  }
  EXPECT_EQ(detail::AttemptProofArenaAccess::liveConsumableCountForTesting(),
            0u);
}

TEST(P103ProofArena, PreConsumeExceptionRevokesLiveToken) {
  using namespace uwb_imu_pl;
  EXPECT_THROW(
      {
        AttemptProofArena arena;
        ASSERT_TRUE(detail::AttemptProofArenaAccess::registerConsumable(
            &arena, 104, 1,
            std::make_shared<const std::uint64_t>(1)));
        EXPECT_EQ(detail::AttemptProofArenaAccess::
                      liveConsumableCountForTesting(),
                  1u);
        throw std::runtime_error("pre-consume failure");
      },
      std::runtime_error);
  EXPECT_EQ(detail::AttemptProofArenaAccess::liveConsumableCountForTesting(),
            0u);
  EXPECT_FALSE(detail::AttemptProofArenaAccess::consumeConsumable(104, 1));
}

TEST(P103ProofArena, NonemptyInvalidFinalBundleFailsClosed) {
  using namespace uwb_imu_pl;
  IntegrityOutput output;
  output.attempted_timestamp = TimestampNs(1);
  output.state.timestamp = TimestampNs(1);
  output.protection_level.detector_certificate_id = "scoped-nonempty";
  output.protection_level.formal_eligible = true;
  output.publication.protected_output = true;
  output.publication.unprotected_output = false;
  FinalPacketTiming timing;
  timing.arrival_steady_ns = 1;
  timing.compute_done_steady_ns = 2;
  timing.packet_ready_steady_ns = 3;
  const auto packet = finalizeOutputPacket(
      std::move(output), timing, 100, FinalProtectionProofBundleV1{});
  EXPECT_FALSE(packet.output().publication.protected_output);
  EXPECT_TRUE(packet.output().publication.unprotected_output);
  EXPECT_FALSE(packet.output().protection_level.formal_eligible);
  EXPECT_TRUE(packet.output().protection_level.detector_certificate_id.empty());
  EXPECT_NE(std::find(packet.output().reason_codes.begin(),
                      packet.output().reason_codes.end(),
                      "FINAL_PROOF_BUNDLE_INVALID"),
            packet.output().reason_codes.end());
}

TEST(P103ProofArena, ScopedCommittedStateDoesNotGrowLegacyRegistry) {
  using namespace uwb_imu_pl;
  const auto before = committedStatePublicationCountForTesting();
  for (std::uint64_t index = 1; index <= 512; ++index) {
    AttemptProofLease lease;
    {
      AttemptProofArena arena;
      CommittedStatePublicationV1 sidecar;
      sidecar.transaction_id = 100000 + index;
      sidecar.state_timestamp = TimestampNs(200000 + index);
      sidecar.covariance.setIdentity();
      ASSERT_TRUE(recordCommittedStatePublicationV1(sidecar, &arena));
      lease = arena.lease();
      arena.close();
      CommittedStatePublicationV1 recovered;
      ASSERT_TRUE(committedStatePublicationV1(
          sidecar.transaction_id, sidecar.state_timestamp, lease,
          &recovered));
    }
    lease.reset();
  }
  EXPECT_EQ(committedStatePublicationCountForTesting(), before);
}

TEST(P103ProofArena, CloseIsIdempotentAndLeaseSurvivesOwner) {
  uwb_imu_pl::AttemptProofLease lease;
  std::uint64_t generation = 0;
  {
    uwb_imu_pl::AttemptProofArena arena;
    generation = arena.generation();
    lease = arena.lease();
    EXPECT_FALSE(arena.closed());
    arena.close();
    arena.close();
    EXPECT_TRUE(arena.closed());
  }
  EXPECT_TRUE(lease.valid());
  EXPECT_TRUE(lease.closed());
  EXPECT_EQ(lease.generation(), generation);
  EXPECT_EQ(lease.entryCount(), 0u);
  lease.reset();
  EXPECT_FALSE(lease.valid());
}

TEST(P103ProofArena, ScopedPipelineExceptionClosesArenaAndDiscardsAttempt) {
  using namespace uwb_imu_pl;
  auto config = IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_uwb_order1.yaml");
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  IncrementalUwbImuEstimator estimator(
      config, config.realtime.lever_arm_body_m);
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  RealtimeIntegrityPipeline pipeline(
      &estimator,
      IntegrityMonitor(config.risk, config.snapshot.rank_tolerance,
                       config.snapshot.max_condition_number),
      offlineReplayPublicationLimits());
  auto seams = std::make_shared<PipelineTestDependencySeamsV1>();
  seams->before_candidate = [](ExclusionActionId) {
    throw std::runtime_error("P103 scoped exception injection");
  };
  pipeline.setTestDependencySeamsV1(seams);
  AttemptProofLease lease;
  EXPECT_THROW((void)pipeline.processUwbBatch(
                   batch(config, 10000000), &lease),
               std::runtime_error);
  EXPECT_TRUE(lease.valid());
  EXPECT_TRUE(lease.closed());
  EXPECT_GT(lease.entryCount(), 0u);
  EXPECT_FALSE(estimator.hasPendingEpoch());
  EXPECT_EQ(estimator.currentEpoch(), 0u);
  EXPECT_EQ(uwb_imu_pl::detail::AttemptProofArenaAccess::
                liveConsumableCountForTesting(),
            0u);
  lease.reset();
}

TEST(IntegrityV2Transaction, PrepareAndDiscardNeverMutateBackend) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config,
                                                   Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  const auto before = estimator.audit();
  const auto ledger_version_before = estimator.factorLedger().version();
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  EXPECT_EQ(estimator.backendUpdateCount(), 1u);
  EXPECT_EQ(estimator.audit().epoch, before.epoch);
  EXPECT_EQ(estimator.audit().state_timestamp, before.state_timestamp);
  EXPECT_EQ(estimator.factorLedger().version(), ledger_version_before);
  auto window = estimator.buildIntegrityWindow(
      transaction, uwb_imu_pl::IntegrityWindowRequest{});
  EXPECT_TRUE(window.capabilities.includes_pending_imu);
  EXPECT_TRUE(window.capabilities.includes_pending_uwb);
  EXPECT_TRUE(window.capabilities.every_active_factor_accounted_once);
  EXPECT_TRUE(window.capabilities.frozen_slot_identity_valid);
  for (const auto& slot : window.slot_accounting) {
    EXPECT_NE(slot.explicit_window_block, slot.boundary_input);
    EXPECT_TRUE(slot.pointer_identity_valid);
  }
  const auto imu_block =
      estimator.buildPendingFactorBlock(transaction, transaction.imu_group.id);
  const auto bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bridge_group.id);
  EXPECT_EQ(transaction.generic_bridge_group.factors.size(), 1u);
  EXPECT_EQ(transaction.generic_bias_continuity_group.factors.size(), 1u);
  EXPECT_EQ(bridge_block.residual_whitened.size(), 9);
  const auto bias_bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bias_continuity_group.id);
  EXPECT_EQ(bias_bridge_block.residual_whitened.size(), 6);
  const auto analytic = uwb_imu_pl::ImuFaultSubspaceBuilder().buildAnalytic(
      transaction, imu_block);
  EXPECT_TRUE(analytic.analytic_input_valid);
  EXPECT_TRUE(analytic.analytic_computation_valid);
  EXPECT_FALSE(analytic.oracle_executed);
  EXPECT_FALSE(analytic.oracle_verified);
  const auto sensitivity =
      uwb_imu_pl::ImuFaultSubspaceBuilder().verifyFiniteDifferenceOracle(
          transaction, imu_block);
  EXPECT_TRUE(sensitivity.oracle_verified) << sensitivity.oracle_relative_error;
  EXPECT_EQ(sensitivity.oracle_reintegrations, 12u);
  const auto receipt = estimator.discardEpoch(
      std::move(transaction),
      {uwb_imu_pl::FdeStatus::ModelInvalid, "test", false});
  EXPECT_EQ(receipt.backend_updates, 0u);
  EXPECT_EQ(estimator.backendUpdateCount(), 1u);
  EXPECT_EQ(estimator.currentState().timestamp, before.state_timestamp);
  EXPECT_EQ(estimator.factorLedger().version(), ledger_version_before);
}

TEST(IntegrityV2Transaction,
     CommitIsOneAtomicBackendUpdateAndLedgerIsComplete) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config,
                                                   Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  const auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
  const auto receipt = estimator.commitEpoch(std::move(transaction), plan);
  EXPECT_EQ(receipt.backend_updates, 1u);
  EXPECT_EQ(estimator.backendUpdateCount(), 2u);
  EXPECT_EQ(estimator.currentEpoch(), 1u);
  EXPECT_TRUE(estimator.factorLedger().hasCompleteActiveProvenance());
}

TEST(IntegrityV2Transaction,
     BoundaryInventoryKeepsFirstStateUwbExplicitExactlyOnce) {
  using namespace uwb_imu_pl;
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary;
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  for (std::size_t epoch = 1; epoch <= 22; ++epoch) {
    for (int sample = 1; sample <= 10; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(epoch * 1000 + sample);
      imu.timestamp = TimestampNs(
          static_cast<std::int64_t>(((epoch - 1) * 10 + sample) * 5000000));
      imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
      estimator.ingestImu(imu);
    }
    auto input = batch(config, static_cast<std::int64_t>(epoch * 50000000));
    for (std::size_t row = 0; row < input.measurements.size(); ++row) {
      input.measurements[row].id = MeasurementId(epoch * 100 + row + 1);
      input.measurements[row].factor_id = FactorId(epoch * 100 + row + 1);
    }
    auto transaction = estimator.prepareEpoch(input);
    const auto window =
        estimator.buildIntegrityWindow(transaction, IntegrityWindowRequest{});
    ASSERT_TRUE(window.model_valid)
        << "epoch=" << epoch << ": " << window.reason;
    if (epoch >= 19) {
      for (const auto& slot : window.slot_accounting) {
        EXPECT_NE(slot.explicit_window_block, slot.boundary_input)
            << "epoch=" << epoch << " slot=" << slot.slot;
      }
      if (epoch >= 21) {
        const std::size_t first = epoch - 20;
        const auto uwb = std::find_if(
            window.factor_inventory.begin(), window.factor_inventory.end(),
            [&](const auto& entry) {
              return entry.epoch == first && entry.kind == FactorKind::UwbBatch;
            });
        const auto imu =
            std::find_if(window.factor_inventory.begin(),
                         window.factor_inventory.end(), [&](const auto& entry) {
                           return entry.epoch == first &&
                                  entry.kind == FactorKind::CombinedImu;
                         });
        ASSERT_NE(uwb, window.factor_inventory.end());
        ASSERT_NE(imu, window.factor_inventory.end());
        EXPECT_EQ(uwb->disposition,
                  FrozenFactorDisposition::ExplicitMeasurement);
        EXPECT_EQ(imu->disposition, FrozenFactorDisposition::BoundaryInput);
        EXPECT_TRUE(std::any_of(window.blocks.begin(), window.blocks.end(),
                                [&](const auto& value) {
                                  return value.group_id == uwb->group_id;
                                }));
        EXPECT_FALSE(std::any_of(window.blocks.begin(), window.blocks.end(),
                                 [&](const auto& value) {
                                   return value.group_id == imu->group_id;
                                 }));
      }
    }
    const auto plan = EpochCommitPlan::nominalPlan(transaction);
    estimator.commitEpoch(std::move(transaction), plan);
  }
}

TEST(IntegrityV2Transaction, ReusingFinalizedTransactionFailsWithoutMutation) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config,
                                                   Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  auto duplicate = transaction;
  const auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
  estimator.commitEpoch(std::move(transaction), plan);
  const auto updates = estimator.backendUpdateCount();
  EXPECT_THROW(estimator.commitEpoch(std::move(duplicate), plan),
               std::logic_error);
  EXPECT_EQ(estimator.backendUpdateCount(), updates);
}

TEST(IntegrityV2Transaction, CorrelatedUwbExclusionUsesPrincipalCovariance) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config,
                                                   Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto correlated = batch(config, 10000000);
  const Eigen::Index n =
      static_cast<Eigen::Index>(correlated.measurements.size());
  correlated.covariance_m2 = Eigen::MatrixXd::Identity(n, n) * 0.04;
  for (Eigen::Index i = 0; i + 1 < n; ++i) {
    correlated.covariance_m2(i, i + 1) = 0.005;
    correlated.covariance_m2(i + 1, i) = 0.005;
  }
  auto transaction = estimator.prepareEpoch(correlated);
  ASSERT_GE(transaction.uwb_groups.size(), 2u);
  const auto replacement = std::find_if(
      transaction.uwb_groups.begin(), transaction.uwb_groups.end(),
      [&](const uwb_imu_pl::PendingFactorGroup& group) {
        return group.excluded_fault_units.size() == 1 &&
               group.excluded_fault_units.front().value() ==
                   correlated.measurements.front().anchor_id.value();
      });
  ASSERT_NE(replacement, transaction.uwb_groups.end());
  EXPECT_TRUE(replacement->raw_covariance.isApprox(
      correlated.covariance_m2.bottomRightCorner(n - 1, n - 1), 0.0));
  estimator.discardEpoch(
      std::move(transaction),
      {uwb_imu_pl::FdeStatus::ModelInvalid, "test complete", false});
}

TEST(IntegrityV2FaultModel, GeneratesCompactUwbModesAndEveryImuAxis) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config,
                                                   Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  const auto window = estimator.buildIntegrityWindow(
      transaction, uwb_imu_pl::IntegrityWindowRequest{});
  ASSERT_TRUE(window.model_valid) << window.reason;
  const auto imu_block =
      estimator.buildPendingFactorBlock(transaction, transaction.imu_group.id);
  const auto bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bridge_group.id);
  const auto imu_maps =
      uwb_imu_pl::ImuFaultSubspaceBuilder().build(transaction, imu_block);
  const auto models = uwb_imu_pl::HypothesisGenerator().generate(
      window, transaction, imu_maps, bridge_block);

  std::size_t epoch_modes = 0, persistent_modes = 0, ramp_modes = 0;
  std::size_t imu_modes = 0;
  for (const auto& mode : models.modes) {
    if (mode.sensor == uwb_imu_pl::SensorType::Uwb) {
      epoch_modes +=
          mode.kind == uwb_imu_pl::FaultKind::AnchorBiasEpochIndependent;
      persistent_modes +=
          mode.kind == uwb_imu_pl::FaultKind::AnchorBiasPersistentConstant;
      ramp_modes += mode.kind == uwb_imu_pl::FaultKind::AnchorBiasRamp;
      EXPECT_EQ(mode.affected_groups.size(), 1u);
      if (mode.kind == uwb_imu_pl::FaultKind::AnchorBiasRamp) {
        EXPECT_EQ(mode.parameter_dimension, 2);
        EXPECT_TRUE(mode.effective_basis_certified);
        EXPECT_EQ(mode.effective_parameter_dimension, 1);
        EXPECT_EQ(mode.effective_parameter_basis.rows(), 2);
        EXPECT_EQ(mode.effective_parameter_basis.cols(), 1);
        EXPECT_LE(mode.discarded_measurement_norm, 1e-12);
        EXPECT_LE(mode.discarded_protected_response_norm, 1e-12);
      }
    } else {
      ++imu_modes;
    }
  }
  EXPECT_EQ(epoch_modes, config.anchors.size());
  EXPECT_EQ(persistent_modes, config.anchors.size());
  EXPECT_EQ(ramp_modes, config.anchors.size());
  EXPECT_EQ(imu_modes, 6u);
  EXPECT_TRUE(std::all_of(models.hypotheses.begin(), models.hypotheses.end(),
                          [](const auto& hypothesis) {
                            return hypothesis.A.size() == 0 &&
                                   !hypothesis.modes.empty() &&
                                   !hypothesis.affected_groups.empty();
                          }));

  uwb_imu_pl::HypothesisGeneratorConfig both_config;
  both_config.single_faults_enabled = true;
  both_config.double_faults_enabled = true;
  const auto both = uwb_imu_pl::HypothesisGenerator(both_config)
                        .generate(window, transaction, imu_maps, bridge_block);
  EXPECT_EQ(both.modes.size(), models.modes.size());
  EXPECT_EQ(both.single_uwb_hypotheses, 3 * config.anchors.size());
  EXPECT_EQ(both.single_accel_hypotheses, 3u);
  EXPECT_EQ(both.single_gyro_hypotheses, 3u);
  EXPECT_EQ(both.double_uwb_accel_hypotheses, 9 * config.anchors.size());
  EXPECT_EQ(both.double_uwb_gyro_hypotheses, 9 * config.anchors.size());
  EXPECT_EQ(both.effective_max_cardinality, 2u);
  ASSERT_FALSE(both.hypotheses.empty());
  EXPECT_DOUBLE_EQ(both.hypotheses.front().hmi_allocation,
                   both_config.total_hmi_allocation / both.hypotheses.size());

  auto double_only_config = both_config;
  double_only_config.single_faults_enabled = false;
  const auto double_only =
      uwb_imu_pl::HypothesisGenerator(double_only_config)
          .generate(window, transaction, imu_maps, bridge_block);
  EXPECT_EQ(double_only.modes.size(), models.modes.size());
  EXPECT_EQ(double_only.single_uwb_hypotheses, 0u);
  EXPECT_EQ(double_only.single_accel_hypotheses, 0u);
  EXPECT_EQ(double_only.single_gyro_hypotheses, 0u);
  EXPECT_EQ(double_only.hypotheses.size(),
            both.double_uwb_accel_hypotheses + both.double_uwb_gyro_hypotheses);
  EXPECT_TRUE(std::all_of(double_only.hypotheses.begin(),
                          double_only.hypotheses.end(),
                          [](const auto& h) { return h.modes.size() == 2; }));

  auto gyro_only_config = double_only_config;
  gyro_only_config.include_uwb_accel_combinations = false;
  const auto gyro_only =
      uwb_imu_pl::HypothesisGenerator(gyro_only_config)
          .generate(window, transaction, imu_maps, bridge_block);
  EXPECT_EQ(gyro_only.double_uwb_accel_hypotheses, 0u);
  EXPECT_EQ(gyro_only.double_uwb_gyro_hypotheses,
            both.double_uwb_gyro_hypotheses);
  estimator.discardEpoch(
      std::move(transaction),
      {uwb_imu_pl::FdeStatus::ModelInvalid, "test complete", false});
}

TEST(IntegrityV2Bridge, AnalyticJacobiansMatchNumericalOracleAndCvIsExact) {
  const double dt = 0.05;
  const auto covariance = Eigen::Matrix<double, 9, 9>::Identity();
  uwb_imu_pl::KinematicPoseVelocityBridgeFactor factor(
      gtsam::Symbol('x', 0), gtsam::Symbol('v', 0), gtsam::Symbol('x', 1),
      gtsam::Symbol('v', 1), dt, covariance);
  const gtsam::Pose3 p0(gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3),
                        gtsam::Point3(1.0, 2.0, -0.5));
  const gtsam::Vector3 v0(0.4, -0.2, 0.1);
  const gtsam::Pose3 p1(p0.rotation(), p0.translation() + dt * v0);
  const gtsam::Vector3 v1 = v0;
  gtsam::Matrix h1, h2, h3, h4;
  const auto error = factor.evaluateError(p0, v0, p1, v1, h1, h2, h3, h4);
  EXPECT_LT(error.norm(), 1e-12);
  const auto fn = [&](const gtsam::Pose3& a, const gtsam::Vector3& b,
                      const gtsam::Pose3& c, const gtsam::Vector3& d) {
    return factor.evaluateError(a, b, c, d);
  };
  EXPECT_TRUE(h1.isApprox(
      gtsam::numericalDerivative41<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
                                   gtsam::Pose3, gtsam::Vector3>(fn, p0, v0, p1,
                                                                 v1),
      1e-7));
  EXPECT_TRUE(h2.isApprox(
      gtsam::numericalDerivative42<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
                                   gtsam::Pose3, gtsam::Vector3>(fn, p0, v0, p1,
                                                                 v1),
      1e-7));
  EXPECT_TRUE(h3.isApprox(
      gtsam::numericalDerivative43<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
                                   gtsam::Pose3, gtsam::Vector3>(fn, p0, v0, p1,
                                                                 v1),
      1e-7));
  EXPECT_TRUE(h4.isApprox(
      gtsam::numericalDerivative44<gtsam::Vector, gtsam::Pose3, gtsam::Vector3,
                                   gtsam::Pose3, gtsam::Vector3>(fn, p0, v0, p1,
                                                                 v1),
      1e-7));
}

TEST(IntegrityV2Bridge, MultiplePoseMarginsAddAndBiasMarginStaysSeparate) {
  Eigen::Matrix<double, 3, Eigen::Dynamic> protected_map(3, 3);
  protected_map.setIdentity();
  Eigen::Matrix3d first_gain = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d second_gain;
  second_gain << 0.0, 2.0, 0.0, 0.0, 0.0, -1.0, 3.0, 0.0, 0.0;
  const Eigen::Vector3d first_bound(1.0, 2.0, 3.0);
  const Eigen::Vector3d second_bound(0.5, 1.5, 2.5);
  const uwb_imu_pl::BridgeFactory factory;
  const Eigen::Vector3d first =
      factory.propagateBoxMargin(protected_map, first_gain, first_bound);
  const Eigen::Vector3d second =
      factory.propagateBoxMargin(protected_map, second_gain, second_bound);
  const Eigen::Vector3d accumulated = first + second;
  EXPECT_TRUE(accumulated.isApprox(Eigen::Vector3d(4.0, 4.5, 4.5), 0.0));

  uwb_imu_pl::EpochTransaction transaction;
  transaction.previous_epoch = 1;
  transaction.proposed_epoch = 2;
  transaction.begin = uwb_imu_pl::TimestampNs(1000000000);
  transaction.end = uwb_imu_pl::TimestampNs(1050000000);
  transaction.generic_bias_continuity_group.id = uwb_imu_pl::FactorGroupId(9);
  const auto bias =
      factory.makeBiasContinuity(transaction, uwb_imu_pl::GenericBridgeSpec{});
  EXPECT_EQ(bias.kind, uwb_imu_pl::FactorKind::BiasContinuity);
  EXPECT_EQ(bias.model_id, "constant_bias_continuity");
  uwb_imu_pl::BridgeAuditRecord audit;
  EXPECT_FALSE(audit.bias_continuity_maneuver_margin_applied);
}

TEST(IntegrityV2Transaction, HistoricalUwbReplacementCommitsAtomically) {
  const auto config = researchConfig();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config,
                                                   Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto first = estimator.prepareEpoch(batch(config, 10000000));
  const auto first_plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(first);
  estimator.commitEpoch(std::move(first), first_plan);

  for (int i = 3; i <= 4; ++i) {
    uwb_imu_pl::ImuMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(100 + i);
    measurement.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    measurement.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
    estimator.ingestImu(measurement);
  }
  auto transaction = estimator.prepareEpoch(batch(config, 20000000));
  ASSERT_EQ(transaction.recoverable_history.size(), 1u);
  const auto& history = transaction.recoverable_history.front();
  const auto historical_uwb = std::find_if(
      history.groups.begin(), history.groups.end(), [&](const auto& group) {
        return group.kind == uwb_imu_pl::FactorKind::UwbBatch &&
               std::find(history.selected_groups.begin(),
                         history.selected_groups.end(),
                         group.id) != history.selected_groups.end();
      });
  ASSERT_NE(historical_uwb, history.groups.end());
  const auto replacement = std::find_if(
      history.groups.begin(), history.groups.end(), [&](const auto& group) {
        return group.kind == uwb_imu_pl::FactorKind::UwbBatch &&
               group.replaces_group &&
               *group.replaces_group == historical_uwb->id &&
               group.excluded_fault_units.size() == 1 &&
               group.excluded_fault_units.front().value() == 1;
      });
  ASSERT_NE(replacement, history.groups.end());
  auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
  plan.groups_to_remove = {historical_uwb->id};
  plan.groups_to_add.push_back(replacement->id);
  plan.groups_to_add.erase(
      std::remove(plan.groups_to_add.begin(), plan.groups_to_add.end(),
                  transaction.imu_group.id),
      plan.groups_to_add.end());
  plan.groups_to_add.push_back(transaction.generic_bridge_group.id);
  plan.groups_to_add.push_back(transaction.generic_bias_continuity_group.id);
  plan.replacement_relations[historical_uwb->id.value()] =
      replacement->id.value();
  plan.recovery_epoch_begin = history.proposed_epoch;
  plan.recovery_epoch_end = history.proposed_epoch;
  const auto historical_id = historical_uwb->id;
  const auto replacement_id = replacement->id;
  const auto updates_before = estimator.backendUpdateCount();
  const auto receipt = estimator.commitEpoch(std::move(transaction), plan);
  EXPECT_EQ(estimator.backendUpdateCount(), updates_before + 1);
  EXPECT_EQ(receipt.backend_updates, 1u);
  EXPECT_FALSE(receipt.historical_groups_removed.empty());
  EXPECT_FALSE(receipt.historical_groups_added.empty());
  EXPECT_TRUE(estimator.factorLedger().activeSlots(historical_id).empty());
  EXPECT_FALSE(estimator.factorLedger().activeSlots(replacement_id).empty());

  // The historical replacement belongs only to epoch 1, while the bridge
  // pair belongs to epoch 2.  The next inventory must recover both catalogs
  // without attributing either group to the wrong committed epoch.
  for (int i = 5; i <= 6; ++i) {
    uwb_imu_pl::ImuMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(100 + i);
    measurement.timestamp = uwb_imu_pl::TimestampNs(i * 5000000);
    measurement.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
    estimator.ingestImu(measurement);
  }
  auto next = estimator.prepareEpoch(batch(config, 30000000));
  const auto next_window = estimator.buildIntegrityWindow(
      next, uwb_imu_pl::IntegrityWindowRequest{});
  EXPECT_TRUE(next_window.capabilities.complete_factor_provenance)
      << next_window.reason;
  EXPECT_EQ(next_window.reason.find("provenance is incomplete"),
            std::string::npos)
      << next_window.reason;
  estimator.discardEpoch(std::move(next),
                         {uwb_imu_pl::FdeStatus::ModelInvalid,
                          "catalog regression complete", false});
}

TEST(IntegrityV2Reinitialization, ExecutesControlledStateSequence) {
  uwb_imu_pl::ControlledReinitializer controller;
  uwb_imu_pl::NavigationState committed;
  committed.timestamp = uwb_imu_pl::TimestampNs(100);
  const auto id = controller.request(uwb_imu_pl::FdeStatus::BridgeTimeout,
                                     "timeout", committed);
  EXPECT_NE(id.value(), 0u);
  EXPECT_EQ(controller.directive().state,
            uwb_imu_pl::ReinitializationState::Requested);
  controller.beginWaiting();
  uwb_imu_pl::ImuMeasurement stale;
  stale.timestamp = committed.timestamp;
  EXPECT_FALSE(controller.acceptTrustedImu(stale).has_value());
  uwb_imu_pl::ImuMeasurement trusted;
  trusted.timestamp = uwb_imu_pl::TimestampNs(101);
  trusted.specific_force_mps2.setZero();
  trusted.angular_velocity_radps.setZero();
  EXPECT_TRUE(controller.acceptTrustedImu(trusted).has_value());
  EXPECT_EQ(controller.directive().state,
            uwb_imu_pl::ReinitializationState::Reinitialized);
  controller.complete();
  EXPECT_EQ(controller.directive().state,
            uwb_imu_pl::ReinitializationState::Running);
}

TEST(IntegrityV2Reinitialization,
     BridgeTimeoutAcceptsBoundaryAndRejectsFirstExcess) {
  using namespace uwb_imu_pl;
  EXPECT_FALSE(bridgeTimeoutExceeded(20, 1.0, 20, 1.0));
  EXPECT_TRUE(bridgeTimeoutExceeded(21, 1.0, 20, 1.0));
  EXPECT_TRUE(bridgeTimeoutExceeded(20, std::nextafter(1.0, 2.0), 20, 1.0));
  EXPECT_FALSE(bridgeTimeoutExceeded(19, 0.999999999, 20, 1.0));
}

TEST(IntegrityV2Window, RejectsFixedLagWithoutMaturityMargin) {
  auto config = researchConfig();
  config.incremental.fixed_lag_epochs =
      config.integrity_window.epochs +
      config.integrity_window.recovery_margin_epochs;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(config,
                                                   Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  const auto window = estimator.buildIntegrityWindow(
      transaction, uwb_imu_pl::IntegrityWindowRequest{});
  EXPECT_FALSE(window.model_valid);
  EXPECT_FALSE(window.capabilities.fixed_lag_maturity_valid);
  EXPECT_EQ(window.reason,
            "fixed lag violates integrity-window maturity delay");
  const auto frozen = estimator.buildFrozenIntegrityWindow(
      transaction, uwb_imu_pl::IntegrityWindowRequest{});
  const auto admission = uwb_imu_pl::admitFrozenIntegrityWindow(frozen);
  ASSERT_TRUE(admission) << admission.reason;
  EXPECT_FALSE(admission.window().model_valid);
  EXPECT_FALSE(admission.window().capabilities.fixed_lag_maturity_valid);
  EXPECT_EQ(admission.window().reason,
            "fixed lag violates integrity-window maturity delay");
  estimator.discardEpoch(
      std::move(transaction),
      {uwb_imu_pl::FdeStatus::ModelInvalid, "test complete", false});
}

TEST(IntegrityV2RankUpdate, MatchesIndependentDenseOracle) {
  auto window = syntheticWindow();
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-12, 1e10);
  ASSERT_TRUE(window.model_valid) << window.reason;
  uwb_imu_pl::ExclusionAction action;
  action.id = uwb_imu_pl::ExclusionActionId(2);
  action.groups_to_remove = {uwb_imu_pl::FactorGroupId(3)};
  const uwb_imu_pl::RankUpdateConfig config{1e-12, 1e10, 10.0};
  const auto base =
      uwb_imu_pl::RankUpdateEvaluator(config).factorizeOnce(window);
  const auto fast =
      uwb_imu_pl::RankUpdateEvaluator(config).evaluate(base, action);
  const auto dense =
      uwb_imu_pl::DenseCandidateOracle(config).evaluate(window, action);
  ASSERT_TRUE(fast.valid) << fast.reason;
  ASSERT_TRUE(dense.valid) << dense.reason;
  EXPECT_TRUE(fast.state_increment.isApprox(dense.state_increment, 1e-10));
  EXPECT_TRUE(fast.covariance.isApprox(dense.covariance, 1e-10));
  EXPECT_NEAR(fast.statistic, dense.statistic, 1e-10);
}

TEST(IntegrityV2RankUpdate, RejectsUnresolvedRemovalBeforeSelection) {
  const auto window = syntheticWindow();
  uwb_imu_pl::ExclusionAction action;
  action.id = uwb_imu_pl::ExclusionActionId(9);
  action.groups_to_remove = {uwb_imu_pl::FactorGroupId(999)};
  const uwb_imu_pl::RankUpdateEvaluator evaluator;
  const auto fast = evaluator.evaluate(evaluator.factorizeOnce(window), action);
  const auto dense =
      uwb_imu_pl::DenseCandidateOracle().evaluate(window, action);
  EXPECT_FALSE(fast.valid);
  EXPECT_FALSE(dense.valid);
  EXPECT_EQ(fast.reason,
            "candidate removal block is absent from frozen window");
  EXPECT_EQ(dense.reason, fast.reason);
}

TEST(IntegrityV2RankUpdate,
     OnlinePathKeepsOnlySharedBaseAndLowRankCorrections) {
  auto window = syntheticWindow();
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-12, 1e10);
  uwb_imu_pl::ExclusionAction action;
  action.id = uwb_imu_pl::ExclusionActionId(2);
  action.groups_to_remove = {uwb_imu_pl::FactorGroupId(3)};
  uwb_imu_pl::RankUpdateConfig config;
  config.rank_tolerance = 1e-12;
  config.max_condition_number = 1e10;
  config.max_linearization_step_norm = 10.0;
  config.materialize_dense_oracle_fields = false;
  const uwb_imu_pl::RankUpdateEvaluator evaluator(config);
  const auto candidate =
      evaluator.evaluate(evaluator.factorizeOnce(window), action);
  const auto dense =
      uwb_imu_pl::DenseCandidateOracle(config).evaluate(window, action);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  ASSERT_TRUE(candidate.shared_base_factorization);
  EXPECT_EQ(candidate.covariance.size(), 0);
  EXPECT_EQ(candidate.retained_jacobian.size(), 0);
  EXPECT_EQ(candidate.retained_jacobian_view, nullptr);
  EXPECT_TRUE(candidate
                  .covarianceTimes(Eigen::MatrixXd::Identity(window.H.cols(),
                                                             window.H.cols()))
                  .isApprox(dense.covariance, 1e-10));
  EXPECT_NEAR(candidate.statistic, dense.statistic, 1e-10);
  EXPECT_NEAR(candidate.information_logdet, dense.information_logdet, 1e-10);
}

TEST(IntegrityV2Detector, UsesSquaredParityDofAndUnionBound) {
  const auto window = syntheticWindow();
  uwb_imu_pl::DetectorRiskContext risk;
  risk.p_fa_per_test = 1e-6;
  risk.continuity_horizon_tests = 1000;
  const auto result = uwb_imu_pl::JointWindowDetector().evaluate(window, risk);
  EXPECT_TRUE(result.numerically_valid);
  EXPECT_EQ(result.dof, window.H.rows() - window.rank);
  EXPECT_GT(result.squared_threshold, 0.0);
  EXPECT_DOUBLE_EQ(result.operation_p_fa_upper_bound, 1e-3);
}

TEST(P002DualChannelContract, PooledPassDualFailRejectsDenseAndMatrixFree) {
  using namespace uwb_imu_pl;
  const auto window = p002HistoryWindow(5.0, 0.0);
  ASSERT_TRUE(window.model_valid) << window.reason;
  DetectorRiskContext risk;
  risk.p_fa_per_test = 1e-6;
  risk.rank_tolerance = 1e-12;
  risk.max_condition_number = 1e10;
  const auto all_in = JointWindowDetector().evaluate(window, risk);
  ASSERT_TRUE(all_in.numerically_valid) << all_in.reason;
  EXPECT_FALSE(all_in.passed);
  EXPECT_GT(all_in.channel_history_statistic,
            all_in.channel_history_threshold);
  ExclusionAction keep;
  keep.id = ExclusionActionId(2002);
  keep.action_model_id = "KEEP_ALL";
  RankUpdateConfig config;
  config.rank_tolerance = 1e-12;
  config.max_condition_number = 1e10;
  config.max_linearization_step_norm = 10.0;
  config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator evaluator(config);
  const auto matrix_free = evaluator.evaluate(
      evaluator.factorizeOnce(window, {keep}), keep);
  const auto dense = DenseCandidateOracle(config).evaluate(window, keep);
  ASSERT_TRUE(matrix_free.valid) << matrix_free.reason;
  ASSERT_TRUE(dense.valid) << dense.reason;
  for (const auto* candidate : {&matrix_free, &dense}) {
    const auto post = JointWindowDetector().evaluateCandidate(
        window, *candidate, risk);
    EXPECT_TRUE(post.channel_split_valid) << post.reason;
    EXPECT_FALSE(post.passed) << "pooled-pass/dual-fail must reject";
    EXPECT_DOUBLE_EQ(post.channel_history_statistic, 25.0);
    EXPECT_EQ(post.channel_history_dof, 1);
  }
}

TEST(P002DualChannelContract, NonzeroHistoryConstantRejectsEveryCandidatePath) {
  using namespace uwb_imu_pl;
  const auto window = p002HistoryWindow(0.0, 25.0);
  ASSERT_TRUE(window.model_valid) << window.reason;
  DetectorRiskContext risk;
  risk.p_fa_per_test = 1e-6;
  risk.rank_tolerance = 1e-12;
  risk.max_condition_number = 1e10;
  const auto all_in = JointWindowDetector().evaluate(window, risk);
  ASSERT_TRUE(all_in.numerically_valid) << all_in.reason;
  EXPECT_FALSE(all_in.passed);
  ExclusionAction keep;
  keep.id = ExclusionActionId(2003);
  keep.action_model_id = "KEEP_ALL";
  RankUpdateConfig config;
  config.rank_tolerance = 1e-12;
  config.max_condition_number = 1e10;
  config.max_linearization_step_norm = 10.0;
  config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator evaluator(config);
  const auto matrix_free = evaluator.evaluate(
      evaluator.factorizeOnce(window, {keep}), keep);
  const auto dense = DenseCandidateOracle(config).evaluate(window, keep);
  for (const auto* candidate : {&matrix_free, &dense}) {
    ASSERT_TRUE(candidate->valid) << candidate->reason;
    const auto post = JointWindowDetector().evaluateCandidate(
        window, *candidate, risk);
    EXPECT_TRUE(post.channel_split_valid) << post.reason;
    EXPECT_FALSE(post.passed) << "history constant is part of accepted event";
    EXPECT_DOUBLE_EQ(post.channel_history_statistic, 25.0);
  }
}

TEST(P002DualChannelContract, MissingPayloadIsInvalidAndCannotCertifyFinitePl) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  finalizeIntegrityWindow(&window, 1e-12, 1e10);
  RankUpdateConfig config;
  config.rank_tolerance = 1e-12;
  config.max_condition_number = 1e10;
  config.max_linearization_step_norm = 10.0;
  const auto source = DenseCandidateOracle(config).evaluate(
      window, ExclusionAction{});
  ASSERT_TRUE(source.valid) << source.reason;
  CandidateEvaluation missing;
  missing.action = source.action;
  missing.base_version = source.base_version;
  missing.state_increment = source.state_increment;
  missing.covariance = source.covariance;
  missing.rows = source.rows;
  missing.rank = source.rank;
  missing.dof = source.dof;
  missing.statistic = source.statistic;
  missing.information_logdet = source.information_logdet;
  missing.valid = true;
  DetectorRiskContext risk;
  risk.p_fa_per_test = 1e-6;
  const auto detector = JointWindowDetector().evaluateCandidate(
      window, missing, risk);
  EXPECT_FALSE(detector.numerically_valid);
  EXPECT_FALSE(detector.passed);
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(2004);
  hypothesis.A = Eigen::MatrixXd::Zero(missing.rows, 1);
  hypothesis.A(4, 0) = 1.0;
  hypothesis.p_md_allocation = 1e-3;
  hypothesis.hmi_allocation = 1e-6;
  hypothesis.prior_probability_bound = 1e-3;
  std::vector<FaultHypothesisV2> remaining{hypothesis};
  const auto pl = ProtectionLevelV2().compute(
      window, missing, detector, &remaining, RiskBudgetV2{});
  EXPECT_FALSE(pl.model_valid);
  EXPECT_FALSE(pl.pl_xyz_m.allFinite());
}

TEST(P002DualChannelContract,
     AllInKeepAndModifiedShareDenseMatrixFreeCertificateAndPl) {
  using namespace uwb_imu_pl;
  auto window = p002HistoryWindow(0.0, 0.0);
  ASSERT_TRUE(window.model_valid) << window.reason;
  DetectorRiskContext risk;
  risk.p_fa_per_test = 1e-6;
  risk.rank_tolerance = 1e-12;
  risk.max_condition_number = 1e10;
  const auto all_in = JointWindowDetector().evaluate(window, risk);
  ASSERT_TRUE(all_in.numerically_valid) << all_in.reason;
  ASSERT_TRUE(all_in.passed) << all_in.reason;

  ExclusionAction keep;
  keep.id = ExclusionActionId(2010);
  keep.action_model_id = "KEEP_ALL";
  ExclusionAction modified;
  modified.id = ExclusionActionId(2011);
  modified.action_model_id = "O02_MODIFIED";
  modified.groups_to_remove = {FactorGroupId(1)};
  RankUpdateConfig config;
  config.rank_tolerance = 1e-12;
  config.max_condition_number = 1e10;
  config.max_linearization_step_norm = 10.0;
  config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator evaluator(config);
  const auto base = evaluator.factorizeOnce(window, {keep, modified});

  for (const auto& action : {keep, modified}) {
    auto matrix_free = evaluator.evaluate(base, action);
    auto dense = DenseCandidateOracle(config).evaluate(window, action);
    ASSERT_TRUE(matrix_free.valid) << matrix_free.reason;
    ASSERT_TRUE(dense.valid) << dense.reason;
    ASSERT_TRUE(matrix_free.detector_certificate.valid)
        << matrix_free.detector_certificate.reason;
    ASSERT_TRUE(dense.detector_certificate.valid)
        << dense.detector_certificate.reason;
    EXPECT_EQ(matrix_free.detector_certificate.row_roles,
              dense.detector_certificate.row_roles);
    EXPECT_DOUBLE_EQ(matrix_free.detector_certificate.current_statistic,
                     dense.detector_certificate.current_statistic);
    EXPECT_DOUBLE_EQ(matrix_free.detector_certificate.history_statistic,
                     dense.detector_certificate.history_statistic);
    EXPECT_DOUBLE_EQ(matrix_free.detector_certificate.history_constant,
                     dense.detector_certificate.history_constant);
    EXPECT_EQ(matrix_free.detector_certificate.current_rank,
              dense.detector_certificate.current_rank);
    EXPECT_EQ(matrix_free.detector_certificate.current_dof,
              dense.detector_certificate.current_dof);
    EXPECT_EQ(matrix_free.detector_certificate.history_rank,
              dense.detector_certificate.history_rank);
    EXPECT_EQ(matrix_free.detector_certificate.history_dof,
              dense.detector_certificate.history_dof);
    EXPECT_EQ(matrix_free.detector_certificate.accepted_event_id,
              "dual_channel_intersection_v6");
    EXPECT_EQ(matrix_free.detector_certificate.numerical_identity,
              dense.detector_certificate.numerical_identity);

    const auto matrix_free_post =
        JointWindowDetector().evaluateCandidate(window, matrix_free, risk);
    const auto dense_post = JointWindowDetector().evaluateCandidate(
        window, dense, risk);
    ASSERT_TRUE(matrix_free_post.passed) << matrix_free_post.reason;
    ASSERT_TRUE(dense_post.passed) << dense_post.reason;
    EXPECT_EQ(matrix_free_post.accepted_event_id,
              dense_post.accepted_event_id);
    EXPECT_EQ(matrix_free_post.candidate_numerical_identity,
              dense_post.candidate_numerical_identity);
    EXPECT_DOUBLE_EQ(matrix_free_post.channel_current_statistic,
                     dense_post.channel_current_statistic);
    EXPECT_DOUBLE_EQ(matrix_free_post.channel_history_statistic,
                     dense_post.channel_history_statistic);
    EXPECT_EQ(matrix_free_post.channel_current_dof,
              dense_post.channel_current_dof);
    EXPECT_EQ(matrix_free_post.channel_history_dof,
              dense_post.channel_history_dof);
    EXPECT_DOUBLE_EQ(matrix_free_post.operation_p_fa_upper_bound,
                     dense_post.operation_p_fa_upper_bound);
    if (action.action_model_id == "KEEP_ALL") {
      EXPECT_EQ(matrix_free_post.accepted_event_id, all_in.accepted_event_id);
      EXPECT_EQ(matrix_free_post.candidate_numerical_identity,
                all_in.candidate_numerical_identity);
      EXPECT_DOUBLE_EQ(matrix_free_post.channel_current_statistic,
                       all_in.channel_current_statistic);
      EXPECT_DOUBLE_EQ(matrix_free_post.channel_history_statistic,
                       all_in.channel_history_statistic);
      EXPECT_EQ(matrix_free_post.channel_current_dof,
                all_in.channel_current_dof);
      EXPECT_EQ(matrix_free_post.channel_history_dof,
                all_in.channel_history_dof);
      EXPECT_DOUBLE_EQ(matrix_free_post.operation_p_fa_upper_bound,
                       all_in.operation_p_fa_upper_bound);
    }

    Eigen::MatrixXd mode = Eigen::MatrixXd::Zero(matrix_free.rows, 1);
    mode(0, 0) = 1.0;
    mode(mode.rows() - 1, 0) = 1.0;
    FaultHypothesisV2 hypothesis;
    hypothesis.id = HypothesisId(2012);
    hypothesis.modes = {FaultModeId(2013)};
    hypothesis.A = mode;
    hypothesis.p_md_allocation = 1e-3;
    hypothesis.hmi_allocation = 1e-6;
    hypothesis.prior_probability_bound = 1e-3;
    std::vector<FaultHypothesisV2> matrix_free_faults{hypothesis};
    std::vector<FaultHypothesisV2> dense_faults{hypothesis};
    ProtectionLevelSharedContext matrix_free_shared;
    matrix_free_shared.mode_maps.emplace(2013, mode);
    ProtectionLevelSharedContext dense_shared = matrix_free_shared;
    const auto matrix_free_pl = ProtectionLevelV2().computeShared(
        window, &matrix_free, matrix_free_post, &matrix_free_faults,
        matrix_free_shared, RiskBudgetV2{});
    const auto dense_pl = ProtectionLevelV2().computeShared(
        window, &dense, dense_post, &dense_faults, dense_shared,
        RiskBudgetV2{});
    ASSERT_TRUE(matrix_free_pl.model_valid) << matrix_free_pl.reason;
    ASSERT_TRUE(dense_pl.model_valid) << dense_pl.reason;
    EXPECT_TRUE(matrix_free_pl.pl_xyz_m.isApprox(dense_pl.pl_xyz_m, 1e-12));
    EXPECT_EQ(matrix_free_pl.detector_certificate_id,
              dense_pl.detector_certificate_id);
    const std::uint64_t matrix_free_proof =
        protectionLevelV2ProofIdentity(matrix_free, matrix_free_pl);
    const std::uint64_t dense_proof =
        protectionLevelV2ProofIdentity(dense, dense_pl);
    ASSERT_NE(0u, matrix_free_proof);
    ASSERT_NE(0u, dense_proof);
    ASSERT_NE(matrix_free_proof, dense_proof);
    EpochTransaction frozen_transaction;
    frozen_transaction.id = TransactionId(window.id.value());
    frozen_transaction.base_version = window.version;
    frozen_transaction.end = TimestampNs(2002000000);
    frozen_transaction.nominal_predicted_state.position_world_m =
        Eigen::Vector3d(4.0, -3.0, 2.0);
    CommitProtectionEvidenceV1 evidence;
    std::string evidence_reason;
    ASSERT_TRUE(mintCommitProtectionEvidenceV1(
        frozen_transaction, window, matrix_free, matrix_free_pl,
        matrix_free_proof, "map", &evidence, &evidence_reason))
        << evidence_reason;
    const Eigen::Vector3d independent_reference =
        frozen_transaction.nominal_predicted_state.position_world_m +
        window.protected_state_map * matrix_free.state_increment;
    EXPECT_TRUE(evidence.reference.mean_world_m.isApprox(
        independent_reference, 0.0));

    auto tampered = evidence;
    tampered.reference.mean_world_m.x() += 1.0;
    EXPECT_FALSE(consumeCommitProtectionEvidenceV1(
        tampered, frozen_transaction, "map", &evidence_reason));
    // A failed/tampered consumption burns the token.
    EXPECT_FALSE(consumeCommitProtectionEvidenceV1(
        evidence, frozen_transaction, "map", &evidence_reason));

    ASSERT_TRUE(mintCommitProtectionEvidenceV1(
        frozen_transaction, window, matrix_free, matrix_free_pl,
        matrix_free_proof, "map", &evidence, &evidence_reason));
    EXPECT_TRUE(consumeCommitProtectionEvidenceV1(
        evidence, frozen_transaction, "map", &evidence_reason));
    EXPECT_FALSE(consumeCommitProtectionEvidenceV1(
        evidence, frozen_transaction, "map", &evidence_reason));

    ASSERT_TRUE(mintCommitProtectionEvidenceV1(
        frozen_transaction, window, matrix_free, matrix_free_pl,
        matrix_free_proof, "map", &evidence, &evidence_reason));
    auto mismatched_transaction = frozen_transaction;
    mismatched_transaction.id = TransactionId(window.id.value() + 1);
    EXPECT_FALSE(consumeCommitProtectionEvidenceV1(
        evidence, mismatched_transaction, "map", &evidence_reason));

    const Eigen::Vector3d large_committed =
        independent_reference + Eigen::Vector3d::Constant(1e150);
    const Eigen::Vector3d large_transfer =
        (large_committed - independent_reference).cwiseAbs();
    std::string large_packet;
    ASSERT_TRUE(bindTransferredProtectionLevelPublicationPacket(
        matrix_free, matrix_free_pl, matrix_free_proof,
        independent_reference, large_committed,
        matrix_free_pl.pl_xyz_m + large_transfer,
        frozen_transaction.end.value(), "map", "body_origin",
        &large_packet));
    EXPECT_TRUE(validateProtectionLevelPublicationProof(
        matrix_free_proof, matrix_free_pl.pl_xyz_m + large_transfer,
        large_packet, nullptr));
    EXPECT_FALSE(bindTransferredProtectionLevelPublicationPacket(
        matrix_free, matrix_free_pl, matrix_free_proof,
        independent_reference,
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
        frozen_transaction.end.value(), "map", "body_origin",
        &large_packet));
    std::string matrix_free_packet;
    std::string dense_packet;
    ASSERT_TRUE(bindProtectionLevelPublicationPacket(
        matrix_free, matrix_free_pl, matrix_free_proof,
        &matrix_free_packet));
    ASSERT_TRUE(bindProtectionLevelPublicationPacket(
        dense, dense_pl, dense_proof, &dense_packet));
    EXPECT_NE(matrix_free_packet, dense_packet);
    EXPECT_TRUE(validateProtectionLevelPublicationProof(
        matrix_free_proof, matrix_free_pl.pl_xyz_m,
        matrix_free_packet, nullptr));
    EXPECT_TRUE(validateProtectionLevelPublicationProof(
        dense_proof, dense_pl.pl_xyz_m, dense_packet, nullptr));
    // Reviewer collision probe: identical served PL/certificate values do not
    // authorize crossing the selected-candidate packet/proof pair.
    EXPECT_FALSE(validateProtectionLevelPublicationProof(
        matrix_free_proof, matrix_free_pl.pl_xyz_m,
        dense_packet, nullptr));
    EXPECT_FALSE(validateProtectionLevelPublicationProof(
        dense_proof, dense_pl.pl_xyz_m,
        matrix_free_packet, nullptr));
  }
}

TEST(P002DualChannelContract, MissingHistoryConstantFailsClosed) {
  using namespace uwb_imu_pl;
  const auto window = p002HistoryWindow(0.0, 0.0);
  RankUpdateConfig config;
  config.rank_tolerance = 1e-12;
  config.max_condition_number = 1e10;
  config.max_linearization_step_norm = 10.0;
  auto candidate = DenseCandidateOracle(config).evaluate(
      window, ExclusionAction{});
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  ASSERT_TRUE(candidate.detector_certificate.valid)
      << candidate.detector_certificate.reason;
  candidate.detector_certificate.history_constant_present = false;
  const auto detector = JointWindowDetector().evaluateCandidate(
      window, candidate, DetectorRiskContext{});
  EXPECT_FALSE(detector.numerically_valid);
  EXPECT_FALSE(detector.passed);
  EXPECT_NE(detector.reason.find("certificate"), std::string::npos);
}

TEST(P002DualChannelContract,
     IndependentRawQrGramAndPlOracleCoversAllCandidatePaths) {
  using namespace uwb_imu_pl;
  const auto window = p002HistoryWindow(0.0, 0.0);
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.continuity_horizon_tests = 1000;
  detector_risk.rank_tolerance = 1e-12;
  detector_risk.max_condition_number = 1e10;
  const auto all_in = JointWindowDetector().evaluate(window, detector_risk);
  ASSERT_TRUE(all_in.passed) << all_in.reason;

  ExclusionAction keep;
  keep.id = ExclusionActionId(2020);
  keep.action_model_id = "KEEP_ALL";
  ExclusionAction modified;
  modified.id = ExclusionActionId(2021);
  modified.action_model_id = "O02_MODIFIED";
  modified.groups_to_remove = {FactorGroupId(1)};
  RankUpdateConfig config;
  config.rank_tolerance = detector_risk.rank_tolerance;
  config.max_condition_number = detector_risk.max_condition_number;
  config.max_linearization_step_norm = 10.0;
  config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator evaluator(config);
  const auto base = evaluator.factorizeOnce(window, {keep, modified});
  RiskBudgetV2 pl_risk;

  for (const auto& action : {keep, modified}) {
    const P002RawOracle raw = p002IndependentRawOracle(
        window, action, config.rank_tolerance);
    if (action.action_model_id == "KEEP_ALL") {
      EXPECT_DOUBLE_EQ(all_in.channel_current_statistic,
                       raw.current_statistic);
      EXPECT_DOUBLE_EQ(all_in.channel_history_statistic,
                       raw.history_statistic);
      EXPECT_EQ(all_in.channel_current_dof, raw.current_dof);
      EXPECT_EQ(all_in.channel_history_dof, raw.history_dof);
    }
    auto matrix_free = evaluator.evaluate(base, action);
    auto dense = DenseCandidateOracle(config).evaluate(window, action);
    for (auto* candidate : {&matrix_free, &dense}) {
      ASSERT_TRUE(candidate->valid) << candidate->reason;
      ASSERT_TRUE(validateCandidateDetectorCertificate(
          window, *candidate, nullptr));
      EXPECT_TRUE(candidate->state_increment.isApprox(raw.state, 1e-12));
      EXPECT_EQ(candidate->detector_certificate.row_roles, raw.row_roles);
      EXPECT_DOUBLE_EQ(candidate->detector_certificate.current_statistic,
                       raw.current_statistic);
      EXPECT_DOUBLE_EQ(candidate->detector_certificate.history_statistic,
                       raw.history_statistic);
      EXPECT_EQ(candidate->detector_certificate.current_rank,
                raw.current_rank);
      EXPECT_EQ(candidate->detector_certificate.current_dof,
                raw.current_dof);
      EXPECT_EQ(candidate->detector_certificate.history_rank,
                raw.history_rank);
      EXPECT_EQ(candidate->detector_certificate.history_dof,
                raw.history_dof);
      const auto detector = JointWindowDetector().evaluateCandidate(
          window, *candidate, detector_risk);
      ASSERT_TRUE(detector.passed) << detector.reason;

      Eigen::MatrixXd mode = Eigen::MatrixXd::Zero(candidate->rows, 1);
      mode(0, 0) = 1.0;
      mode(mode.rows() - 1, 0) = 1.0;
      Eigen::JacobiSVD<Eigen::MatrixXd> direct_svd(
          raw.h, Eigen::ComputeThinU | Eigen::ComputeThinV);
      const Eigen::VectorXd direct_inverse =
          direct_svd.singularValues().cwiseInverse();
      const Eigen::MatrixXd direct_covariance = direct_svd.matrixV() *
          direct_inverse.array().square().matrix().asDiagonal() *
          direct_svd.matrixV().transpose();
      Eigen::MatrixXd all_rhs(raw.h.cols(), 6);
      all_rhs.leftCols<3>() = window.protected_state_map.transpose();
      all_rhs.col(3) = raw.h.transpose() * mode;
      all_rhs.rightCols<2>().setConstant(0.375);  // bridge RHS fixtures
      const Eigen::MatrixXd direct_all_rhs = direct_covariance * all_rhs;
      EXPECT_TRUE(candidate->covarianceTimes(all_rhs).isApprox(
          direct_all_rhs, 1e-11));
      const Eigen::MatrixXd direct_y =
          direct_svd.matrixU().transpose() * mode;
      const Eigen::MatrixXd direct_z =
          mode - direct_svd.matrixU() * direct_y;
      const Eigen::MatrixXd direct_gamma =
          direct_z.transpose() * direct_z;
      const Eigen::VectorXd direct_parity = raw.z - raw.h * raw.state;
      const Eigen::VectorXd direct_t = mode.transpose() * direct_parity;
      const Eigen::MatrixXd direct_g = window.protected_state_map *
          direct_svd.matrixV() * direct_inverse.asDiagonal() * direct_y;
      const double direct_j = direct_parity.squaredNorm() -
          direct_t.dot(direct_gamma.ldlt().solve(direct_t));
      FaultHypothesisV2 hypothesis;
      hypothesis.id = HypothesisId(2022);
      hypothesis.modes = {FaultModeId(2023)};
      hypothesis.A = mode;
      hypothesis.p_md_allocation = 1e-3;
      hypothesis.hmi_allocation = 1e-6;
      hypothesis.prior_probability_bound = 1e-3;
      std::vector<FaultHypothesisV2> hypotheses{hypothesis};
      ProtectionLevelSharedContext shared;
      shared.mode_maps.emplace(2023, mode);
      const auto pl = ProtectionLevelV2().computeShared(
          window, candidate, detector, &hypotheses, shared, pl_risk);
      ASSERT_TRUE(pl.model_valid) << pl.reason;
      ASSERT_TRUE(validateProtectionLevelV2Proof(
          *candidate, detector, hypotheses, pl, nullptr));
      auto tampered_pl = pl;
      tampered_pl.pl_xyz_m(0) = std::nextafter(
          tampered_pl.pl_xyz_m(0),
          std::numeric_limits<double>::infinity());
      EXPECT_FALSE(validateProtectionLevelV2Proof(
          *candidate, detector, hypotheses, tampered_pl, nullptr));
      ProtectionLevelV2ProofV1 pl_proof;
      ASSERT_TRUE(protectionLevelV2Proof(pl, &pl_proof));
      ASSERT_TRUE(validateProtectionLevelV2ProofPayload(pl_proof, nullptr));
      EXPECT_EQ(protectionLevelV2ProofIdentity(*candidate, pl),
                pl_proof.proof_identity);
      ASSERT_EQ(pl_proof.hypothesis_proofs.size(), 1u);
      ASSERT_EQ(pl_proof.dual_channel_proofs.size(), 1u);
      auto tampered_pl_proof = pl_proof;
      tampered_pl_proof.served_result.pl_xyz_m(0) = std::nextafter(
          tampered_pl_proof.served_result.pl_xyz_m(0),
          std::numeric_limits<double>::infinity());
      EXPECT_FALSE(validateProtectionLevelV2ProofPayload(
          tampered_pl_proof, nullptr));
      tampered_pl_proof = pl_proof;
      tampered_pl_proof.protected_covariance(0, 0) = std::nextafter(
          tampered_pl_proof.protected_covariance(0, 0),
          std::numeric_limits<double>::infinity());
      EXPECT_FALSE(validateProtectionLevelV2ProofPayload(
          tampered_pl_proof, nullptr));
      tampered_pl_proof = pl_proof;
      tampered_pl_proof.fixed_bridge_margin(0) =
          std::numeric_limits<double>::denorm_min();
      EXPECT_FALSE(validateProtectionLevelV2ProofPayload(
          tampered_pl_proof, nullptr));
      tampered_pl_proof = pl_proof;
      tampered_pl_proof.nominal_tail = std::nextafter(
          tampered_pl_proof.nominal_tail,
          std::numeric_limits<double>::infinity());
      EXPECT_FALSE(validateProtectionLevelV2ProofPayload(
          tampered_pl_proof, nullptr));
      tampered_pl_proof = pl_proof;
      tampered_pl_proof.component_proofs[0].served_component(0) =
          std::nextafter(
              tampered_pl_proof.component_proofs[0].served_component(0),
              std::numeric_limits<double>::infinity());
      EXPECT_FALSE(validateProtectionLevelV2ProofPayload(
          tampered_pl_proof, nullptr));
      tampered_pl_proof = pl_proof;
      tampered_pl_proof.hypotheses[0].prior_probability_bound =
          std::nextafter(
              tampered_pl_proof.hypotheses[0].prior_probability_bound,
              std::numeric_limits<double>::infinity());
      EXPECT_FALSE(validateProtectionLevelV2ProofPayload(
          tampered_pl_proof, nullptr));
      tampered_pl_proof = pl_proof;
      tampered_pl_proof.dual_channel_proofs[0].w_matrix(0, 0) =
          std::nextafter(
              tampered_pl_proof.dual_channel_proofs[0].w_matrix(0, 0),
              std::numeric_limits<double>::infinity());
      EXPECT_FALSE(validateProtectionLevelV2ProofPayload(
          tampered_pl_proof, nullptr));
      auto tampered_hypothesis_proof = pl_proof.hypothesis_proofs[0];
      tampered_hypothesis_proof.protected_response(0, 0) =
          std::nextafter(
              tampered_hypothesis_proof.protected_response(0, 0),
              std::numeric_limits<double>::infinity());
      EXPECT_FALSE(validateFrozenHypothesisPlEntry(
          tampered_hypothesis_proof, nullptr));
      tampered_hypothesis_proof = pl_proof.hypothesis_proofs[0];
      tampered_hypothesis_proof.served_entry.z_classification ^= 1;
      EXPECT_FALSE(validateFrozenHypothesisPlEntry(
          tampered_hypothesis_proof, nullptr));
      tampered_hypothesis_proof = pl_proof.hypothesis_proofs[0];
      tampered_hypothesis_proof.served_entry.valid =
          !tampered_hypothesis_proof.served_entry.valid;
      EXPECT_FALSE(validateFrozenHypothesisPlEntry(
          tampered_hypothesis_proof, nullptr));
      auto tampered_dual_proof = pl_proof.dual_channel_proofs[0];
      tampered_dual_proof.proof_identity ^= 1;
      EXPECT_FALSE(validateDualChannelNumericalProof(
          tampered_dual_proof, nullptr));
      tampered_dual_proof = pl_proof.dual_channel_proofs[0];
      tampered_dual_proof.served_result.w_rank += 1.0;
      EXPECT_FALSE(validateDualChannelNumericalProof(
          tampered_dual_proof, nullptr));
      tampered_dual_proof = pl_proof.dual_channel_proofs[0];
      tampered_dual_proof.served_result.model_error_validated =
          !tampered_dual_proof.served_result.model_error_validated;
      EXPECT_FALSE(validateDualChannelNumericalProof(
          tampered_dual_proof, nullptr));
      EXPECT_TRUE(pl_proof.hypothesis_proofs[0].certified_gram.isApprox(
          direct_gamma, 1e-12));
      EXPECT_TRUE(pl_proof.hypothesis_proofs[0].protected_response.isApprox(
          direct_g, 1e-12));
      const Eigen::Vector3d direct_slopes =
          (direct_g * direct_gamma.ldlt().solve(direct_g.transpose()))
              .diagonal().cwiseMax(0.0).cwiseSqrt();
      EXPECT_TRUE(pl_proof.hypothesis_proofs[0]
                      .served_entry.protected_slopes.isApprox(
          direct_slopes, 1e-10));
      std::string publication_packet;
      ASSERT_TRUE(bindProtectionLevelPublicationPacket(
          *candidate, pl, pl_proof.proof_identity,
          &publication_packet));
      ProtectionLevelPublicationPacketV1 packet_payload;
      ASSERT_TRUE(protectionLevelPublicationPacket(
          publication_packet, &packet_payload));
      EXPECT_EQ(packet_payload.candidate_proof_identity,
                candidateNumericalProofIdentity(window, *candidate));
      EXPECT_TRUE(validateProtectionLevelPublicationProof(
          pl_proof.proof_identity, pl.pl_xyz_m,
          publication_packet, nullptr));
      Eigen::Vector3d tampered_publication_pl = pl.pl_xyz_m;
      tampered_publication_pl(0) = std::nextafter(
          tampered_publication_pl(0),
          std::numeric_limits<double>::infinity());
      EXPECT_FALSE(validateProtectionLevelPublicationProof(
          pl_proof.proof_identity, tampered_publication_pl,
          publication_packet, nullptr));
      if (action.action_model_id == "KEEP_ALL") {
        auto profile_hypotheses = std::vector<FaultHypothesisV2>{hypothesis};
        profile_hypotheses[0].A = mode;
        HypothesisEvaluationConfig profile_config;
        profile_config.rank_tolerance = config.rank_tolerance;
        profile_config.max_condition_number = config.max_condition_number;
        const auto evidence = HypothesisEvidenceEvaluator(profile_config)
            .evaluateAll(window, &profile_hypotheses,
                         std::numeric_limits<double>::infinity());
        ASSERT_EQ(evidence.size(), 1u);
        EXPECT_TRUE(evidence[0].fault_gram.isApprox(direct_gamma, 1e-12));
        EXPECT_TRUE(evidence[0].estimated_fault.isApprox(
            direct_gamma.ldlt().solve(direct_t), 1e-11));
        EXPECT_NEAR(evidence[0].profile_j, direct_j, 1e-11);
      }
      const Eigen::Vector3d independent_pl = p002IndependentPlOracle(
          window, raw, mode, detector_risk, hypothesis, pl_risk);
      ASSERT_TRUE(independent_pl.allFinite());
      EXPECT_TRUE(pl.pl_xyz_m.isApprox(independent_pl, 1e-10));
    }
  }

  // Independent discrete rank/condition sweep, including the exact boundary.
  for (const double small : {1e-12, 1e-11, 1e-10, 1e-9, 1e-8,
                             1e-7, 1e-6, 1e-5, 1e-4}) {
    Eigen::Matrix2d raw_h = Eigen::Matrix2d::Zero();
    raw_h(0, 0) = 1.0;
    raw_h(1, 1) = small;
    Eigen::JacobiSVD<Eigen::Matrix2d> direct(raw_h);
    const int direct_rank = static_cast<int>((
        direct.singularValues().array() > 1e-10).count());
    if (small == 1e-10) {
      EXPECT_DOUBLE_EQ(direct.singularValues()(1), 1e-10);  // transition band
    } else {
      EXPECT_EQ(direct_rank, small > 1e-10 ? 2 : 1);
    }
    const double direct_condition = direct.singularValues()(0) /
        direct.singularValues()(1);
    EXPECT_DOUBLE_EQ(direct_condition, 1.0 / small);

    // The same discrete decision must be made by the production candidate's
    // final raw-H SVD fallback, not merely by this test-local reference.
    LinearizedIntegrityWindow sweep_window;
    sweep_window.version = {71, 72, 73, 74};
    Eigen::MatrixXd candidate_h = Eigen::MatrixXd::Zero(3, 2);
    candidate_h(0, 0) = 1.0;
    candidate_h(1, 1) = small;
    sweep_window.blocks = {
        block(7101, candidate_h, Eigen::Vector3d(0.1, small, 0.2),
              sweep_window.version),
        block(7102, Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero(),
              sweep_window.version)};
    finalizeIntegrityWindow(&sweep_window, 1e-10, 1e13);
    ASSERT_TRUE(sweep_window.model_valid) << sweep_window.reason;
    ExclusionAction sweep_action;
    sweep_action.id = ExclusionActionId(7103);
    sweep_action.groups_to_remove = {FactorGroupId(7102)};
    RankUpdateConfig sweep_config;
    sweep_config.rank_tolerance = 1e-10;
    sweep_config.max_condition_number = 1e13;
    sweep_config.max_linearization_step_norm = 1e13;
    sweep_config.force_exact_condition_number = true;
    sweep_config.materialize_dense_oracle_fields = false;
    const RankUpdateEvaluator sweep_evaluator(sweep_config);
    const auto production = sweep_evaluator.evaluate(
        sweep_evaluator.factorizeOnce(sweep_window, {sweep_action}),
        sweep_action);
    EXPECT_TRUE(production.exact_slow_path);
    EXPECT_EQ(production.diagnostics.numerical_path,
              "FINAL_JACOBIAN_REFERENCE");
    EXPECT_EQ(production.rank, direct_rank);
    EXPECT_EQ(production.valid, direct_rank == 2) << production.reason;
    if (direct_rank == 2) {
      EXPECT_NEAR(production.condition_number, direct_condition,
                  1e-10 * direct_condition);
    }
  }

  RankUpdateConfig candidate_fallback_config = config;
  candidate_fallback_config.force_exact_condition_number = true;
  const RankUpdateEvaluator candidate_fallback_evaluator(
      candidate_fallback_config);
  const auto candidate_fallback = candidate_fallback_evaluator.evaluate(
      candidate_fallback_evaluator.factorizeOnce(window, {modified}),
      modified);
  ASSERT_TRUE(candidate_fallback.valid) << candidate_fallback.reason;
  EXPECT_TRUE(candidate_fallback.exact_slow_path);
  EXPECT_EQ(candidate_fallback.diagnostics.numerical_path,
            "FINAL_JACOBIAN_REFERENCE");
  const P002RawOracle candidate_fallback_raw = p002IndependentRawOracle(
      window, modified, config.rank_tolerance);
  EXPECT_TRUE(candidate_fallback.state_increment.isApprox(
      candidate_fallback_raw.state, 1e-11));
  EXPECT_TRUE(candidate_fallback.covarianceTimes(
      Eigen::MatrixXd::Identity(window.H.cols(), window.H.cols())).isApprox(
          candidate_fallback_raw.covariance, 1e-10));

  // Force the production operator off its LLT path and compare the spectral
  // fallback against the direct raw-H SVD for several simultaneous RHS.
  const P002RawOracle keep_raw = p002IndependentRawOracle(
      window, keep, config.rank_tolerance);
  FrozenWindowNumerics forced = *window.numerics;
  forced.information_factorization =
      std::make_shared<const Eigen::LLT<Eigen::MatrixXd>>(
          7.0 * Eigen::MatrixXd::Identity(keep_raw.h.cols(),
                                          keep_raw.h.cols()));
  Eigen::MatrixXd fallback_rhs(keep_raw.h.cols(), 5);
  fallback_rhs.setRandom();
  bool used_spectral_fallback = false;
  const Eigen::MatrixXd fallback = solveFrozenInformation(
      forced, window.base_information, fallback_rhs,
      &used_spectral_fallback);
  EXPECT_TRUE(used_spectral_fallback);
  EXPECT_TRUE(fallback.isApprox(keep_raw.covariance * fallback_rhs, 1e-11));
}

TEST(P003ProofChain, FrozenWindowServedResultIsConsumerRecomputable) {
  using namespace uwb_imu_pl;
  const auto window = syntheticWindow();
  ASSERT_TRUE(window.numerics);
  ASSERT_TRUE(validateFrozenWindowNumericalProof(
      window, *window.numerics, nullptr));
  EXPECT_FALSE(window.numerics->canonical_spectral_solution)
      << "usable QR is the served path";

  auto tampered = *window.numerics;
  tampered.base_state_increment(0) = std::nextafter(
      tampered.base_state_increment(0),
      std::numeric_limits<double>::infinity());
  EXPECT_FALSE(validateFrozenWindowNumericalProof(window, tampered, nullptr));
  tampered = *window.numerics;
  tampered.parity(0) = std::nextafter(
      tampered.parity(0), std::numeric_limits<double>::infinity());
  EXPECT_FALSE(validateFrozenWindowNumericalProof(window, tampered, nullptr));
  tampered = *window.numerics;
  tampered.statistic = std::nextafter(
      tampered.statistic, std::numeric_limits<double>::infinity());
  EXPECT_FALSE(validateFrozenWindowNumericalProof(window, tampered, nullptr));
  tampered = *window.numerics;
  tampered.canonical_spectral_solution = true;
  EXPECT_FALSE(validateFrozenWindowNumericalProof(window, tampered, nullptr));

  auto fallback_window = window;
  fallback_window.square_root.reset();
  auto fallback = *window.numerics;
  fallback.canonical_spectral_solution = true;
  fallback.square_root_certificate_ok = false;
  fallback.base_state_increment = fallback.spectral_state_increment;
  fallback.parity = fallback_window.z -
      fallback_window.H * fallback.base_state_increment;
  fallback.statistic = fallback.parity.squaredNorm();
  EXPECT_TRUE(validateFrozenWindowNumericalProof(
      fallback_window, fallback, nullptr));
  // Reviewer probe: even synchronized cached derivatives and a public rehash
  // cannot hide a changed canonical SVD state, because validation recomputes
  // independently from raw H/z.
  fallback.spectral_state_increment(0) += 0.125;
  fallback.base_state_increment = fallback.spectral_state_increment;
  fallback.parity = fallback_window.z -
      fallback_window.H * fallback.base_state_increment;
  fallback.statistic = fallback.parity.squaredNorm();
  EXPECT_NE(0u, frozenWindowNumericalProofIdentity(fallback_window, fallback));
  EXPECT_FALSE(validateFrozenWindowNumericalProof(
      fallback_window, fallback, nullptr));

  // Reviewer probe: altering both spectral factors and rehashing must not
  // replace the covariance/all-RHS operator derived from actual raw H.
  fallback = *window.numerics;
  fallback.canonical_spectral_solution = true;
  fallback.square_root_certificate_ok = false;
  fallback.base_state_increment = fallback.spectral_state_increment;
  fallback.parity = fallback_window.z -
      fallback_window.H * fallback.base_state_increment;
  fallback.statistic = fallback.parity.squaredNorm();
  Eigen::MatrixXd changed_vectors = *fallback.spectral_vectors;
  changed_vectors(0, 0) += 0.125;
  fallback.spectral_vectors =
      std::make_shared<const Eigen::MatrixXd>(changed_vectors);
  fallback.spectral_inverse_squared(0) *= 0.75;
  EXPECT_NE(0u, frozenWindowNumericalProofIdentity(fallback_window, fallback));
  EXPECT_FALSE(validateFrozenWindowNumericalProof(
      fallback_window, fallback, nullptr));
}

TEST(P002DualChannelContract,
     StaleSwappedAndTamperedContractsFailClosedBeforeFinitePl) {
  using namespace uwb_imu_pl;
  const auto window = p002HistoryWindow(0.0, 0.0);
  DetectorRiskContext risk;
  risk.p_fa_per_test = 1e-6;
  risk.continuity_horizon_tests = 1000;
  risk.rank_tolerance = 1e-12;
  risk.max_condition_number = 1e10;
  ExclusionAction keep;
  keep.id = ExclusionActionId(2030);
  keep.action_model_id = "KEEP_ALL";
  ExclusionAction modified;
  modified.id = ExclusionActionId(2031);
  modified.action_model_id = "O02_MODIFIED";
  modified.groups_to_remove = {FactorGroupId(1)};
  RankUpdateConfig config;
  config.rank_tolerance = risk.rank_tolerance;
  config.max_condition_number = risk.max_condition_number;
  config.max_linearization_step_norm = 10.0;
  const auto keep_candidate = DenseCandidateOracle(config).evaluate(window, keep);
  const auto modified_candidate =
      DenseCandidateOracle(config).evaluate(window, modified);
  ASSERT_TRUE(keep_candidate.valid);
  ASSERT_TRUE(modified_candidate.valid);
  const auto keep_detector = JointWindowDetector().evaluateCandidate(
      window, keep_candidate, risk);
  ASSERT_TRUE(keep_detector.passed) << keep_detector.reason;

  auto finite_pl = [&](const CandidateEvaluation& candidate,
                       const DetectorResultV2& detector) {
    FaultHypothesisV2 hypothesis;
    hypothesis.id = HypothesisId(2032);
    hypothesis.A = Eigen::MatrixXd::Zero(candidate.rows, 1);
    hypothesis.A(0, 0) = 1.0;
    hypothesis.A(candidate.rows - 1, 0) = 1.0;
    hypothesis.p_md_allocation = 1e-3;
    hypothesis.hmi_allocation = 1e-6;
    hypothesis.prior_probability_bound = 1e-3;
    std::vector<FaultHypothesisV2> hypotheses{hypothesis};
    return ProtectionLevelV2().compute(
        window, candidate, detector, &hypotheses, RiskBudgetV2{});
  };
  auto reject_certificate = [&](const char* label,
                                CandidateEvaluation tampered) {
    const auto detector = JointWindowDetector().evaluateCandidate(
        window, tampered, risk);
    EXPECT_FALSE(detector.numerically_valid) << label;
    EXPECT_FALSE(detector.passed) << label;
    const auto pl = finite_pl(tampered, detector);
    EXPECT_FALSE(pl.model_valid) << label;
    EXPECT_FALSE(pl.pl_xyz_m.allFinite()) << label;
  };

  {
    auto tampered = keep_candidate;
    std::swap(tampered.detector_certificate.row_roles.front(),
              tampered.detector_certificate.row_roles.back());
    reject_certificate("row-role swap", tampered);
  }
  for (const auto& mutation :
       std::vector<std::function<void(CandidateEvaluation&)>>{
           [](CandidateEvaluation& value) {
             value.detector_certificate.current_statistic += 0.25;
           },
           [](CandidateEvaluation& value) {
             --value.detector_certificate.current_dof;
           },
           [](CandidateEvaluation& value) {
             value.detector_certificate.history_constant += 0.5;
           },
           [](CandidateEvaluation& value) {
             value.detector_certificate.accepted_event_id += "_tampered";
           },
           [](CandidateEvaluation& value) {
             value.detector_certificate.numerical_identity ^= 1;
           },
           [](CandidateEvaluation& value) {
             value.detector_certificate.certificate_digest ^= 1;
           },
           [](CandidateEvaluation& value) {
             value.action.action_model_id += "_stale";
           }}) {
    auto tampered = keep_candidate;
    mutation(tampered);
    reject_certificate("certificate payload", tampered);
  }
  {
    auto tampered = keep_candidate;
    tampered.state_increment(0) = std::nextafter(
        tampered.state_increment(0),
        tampered.state_increment(0) + 1.0);
    reject_certificate("candidate state increment", tampered);
  }
  {
    auto tampered = keep_candidate;
    ASSERT_GT(tampered.covariance.size(), 0);
    tampered.covariance(0, 0) = std::nextafter(
        tampered.covariance(0, 0), tampered.covariance(0, 0) + 1.0);
    reject_certificate("dense candidate covariance", tampered);
  }
  {
    RankUpdateConfig matrix_free_config = config;
    matrix_free_config.materialize_dense_oracle_fields = false;
    const RankUpdateEvaluator evaluator(matrix_free_config);
    const auto matrix_free_candidate = evaluator.evaluate(
        evaluator.factorizeOnce(window, {modified}), modified);
    ASSERT_TRUE(matrix_free_candidate.valid) << matrix_free_candidate.reason;
    ASSERT_EQ(matrix_free_candidate.covariance.size(), 0);
    ASSERT_FALSE(matrix_free_candidate.reference_vectors);
    ASSERT_TRUE(matrix_free_candidate.shared_base_factorization);
    ASSERT_GT(matrix_free_candidate.covariance_plus_factor.size(), 0);
    {
      auto tampered = matrix_free_candidate;
      const Eigen::MatrixXd base_factor =
          tampered.shared_base_factorization->matrixL().toDenseMatrix();
      Eigen::MatrixXd changed_information =
          base_factor * base_factor.transpose();
      changed_information(0, 0) += 1.0;
      auto changed_base =
          std::make_shared<Eigen::LLT<Eigen::MatrixXd>>(changed_information);
      ASSERT_EQ(changed_base->info(), Eigen::Success);
      tampered.shared_base_factorization = changed_base;
      reject_certificate("matrix-free shared-base covariance", tampered);
    }
    auto tampered = matrix_free_candidate;
    tampered.covariance_plus_factor(0, 0) = std::nextafter(
        tampered.covariance_plus_factor(0, 0),
        tampered.covariance_plus_factor(0, 0) + 1.0);
    reject_certificate("matrix-free covariance correction", tampered);
  }

  auto transplanted = modified_candidate;
  transplanted.detector_certificate = keep_candidate.detector_certificate;
  const auto transplanted_pl = finite_pl(transplanted, keep_detector);
  EXPECT_FALSE(transplanted_pl.model_valid);
  EXPECT_FALSE(transplanted_pl.pl_xyz_m.allFinite());

  auto stale_window = window;
  stale_window.id = WindowId(99999);
  const auto stale_detector = JointWindowDetector().evaluateCandidate(
      stale_window, keep_candidate, risk);
  EXPECT_FALSE(stale_detector.numerically_valid);

  auto reject_detector_semantics = [&](const char* label,
                                       DetectorResultV2 tampered) {
    EXPECT_EQ(tampered.detector_contract_digest,
              detectorContractDigest(tampered)) << label;
    const auto pl = finite_pl(keep_candidate, tampered);
    EXPECT_FALSE(pl.model_valid) << label;
    EXPECT_FALSE(pl.pl_xyz_m.allFinite()) << label;
  };
  for (const auto& mutation :
       std::vector<std::function<void(DetectorResultV2&)>>{
           [](DetectorResultV2& value) {
             value.channel_current_threshold += 1.0;
           },
           [](DetectorResultV2& value) {
             value.channel_history_threshold += 1.0;
           },
           [](DetectorResultV2& value) { value.p_fa_per_test *= 2.0; },
           [](DetectorResultV2& value) {
             value.operation_p_fa_upper_bound *= 0.5;
           },
           [](DetectorResultV2& value) { ++value.active_channel_count; },
           [](DetectorResultV2& value) { ++value.continuity_horizon_tests; },
           [](DetectorResultV2& value) {
             value.channel_current_statistic += 0.125;
           },
           [](DetectorResultV2& value) { --value.channel_current_dof; },
           [](DetectorResultV2& value) {
             value.channel_history_statistic += 0.125;
           },
           [](DetectorResultV2& value) { --value.channel_history_dof; },
           [](DetectorResultV2& value) {
             value.squared_parity_statistic += 0.125;
           },
           [](DetectorResultV2& value) { value.squared_threshold += 1.0; },
           [](DetectorResultV2& value) { ++value.rows; },
           [](DetectorResultV2& value) { --value.rank; },
           [](DetectorResultV2& value) { --value.dof; },
           [](DetectorResultV2& value) { value.history_constant += 0.25; },
           [](DetectorResultV2& value) {
             value.accepted_event_id += "_tampered";
           },
           [](DetectorResultV2& value) {
             value.candidate_numerical_identity ^= 1;
           }}) {
    auto tampered = keep_detector;
    mutation(tampered);
    tampered.detector_contract_digest = detectorContractDigest(tampered);
    reject_detector_semantics("detector semantic payload", tampered);
  }
  {
    auto tampered = keep_detector;
    tampered.detector_contract_digest ^= 1;
    EXPECT_NE(tampered.detector_contract_digest,
              detectorContractDigest(tampered));
    const auto pl = finite_pl(keep_candidate, tampered);
    EXPECT_FALSE(pl.model_valid) << "detector digest only";
    EXPECT_FALSE(pl.pl_xyz_m.allFinite()) << "detector digest only";
  }
}

TEST(IntegrityV2ProtectionLevel, RecomputesPostCandidateAndStaysResearchOnly) {
  auto window = syntheticWindow();
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  uwb_imu_pl::finalizeIntegrityWindow(&window, 1e-12, 1e10);
  uwb_imu_pl::ExclusionAction keep;
  keep.id = uwb_imu_pl::ExclusionActionId(1);
  const uwb_imu_pl::RankUpdateConfig config{1e-12, 1e10, 10.0};
  const uwb_imu_pl::RankUpdateEvaluator evaluator(config);
  const auto candidate =
      evaluator.evaluate(evaluator.factorizeOnce(window), keep);
  uwb_imu_pl::DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  const auto detector = uwb_imu_pl::JointWindowDetector().evaluateCandidate(
      window, candidate, detector_risk);
  uwb_imu_pl::FaultHypothesisV2 first;
  first.id = uwb_imu_pl::HypothesisId(1);
  first.A = Eigen::MatrixXd::Zero(candidate.rows, 1);
  first.A(4, 0) = 1.0;
  first.p_md_allocation = 1e-3;
  first.hmi_allocation = 4e-6;
  first.prior_probability_bound = 1e-3;
  auto second = first;
  second.id = uwb_imu_pl::HypothesisId(2);
  second.A.setZero();
  second.A(5, 0) = 1.0;
  std::vector<uwb_imu_pl::FaultHypothesisV2> remaining{first, second};
  const auto result = uwb_imu_pl::ProtectionLevelV2().compute(
      window, candidate, detector, &remaining, uwb_imu_pl::RiskBudgetV2{});
  EXPECT_TRUE(result.model_valid) << result.reason;
  EXPECT_TRUE(result.pl_xyz_m.allFinite());
  EXPECT_FALSE(result.risk_budget_valid)
      << "PL alone cannot close omitted/escape/selection events";
  EXPECT_EQ(result.availability, uwb_imu_pl::Availability::Unavailable);
  EXPECT_FALSE(result.formal_eligible);
  EXPECT_TRUE(remaining[0].monitored);
  EXPECT_TRUE(remaining[1].monitored);
}

TEST(IntegrityV2ProtectionLevel,
     FrozenEvidenceContextMatchesKeepAllAndInvalidatesOnMutation) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  finalizeIntegrityWindow(&window, 1e-10, 1e10);

  FaultModeBasis uwb;
  uwb.id = FaultModeId(1);
  uwb.sensor = SensorType::Uwb;
  uwb.parameter_dimension = 1;
  uwb.raw_group_maps[FactorGroupId(2)] =
      (Eigen::Vector3d() << 1.0, 0.25, -0.5).finished();
  FaultModeBasis imu;
  imu.id = FaultModeId(2);
  imu.sensor = SensorType::ImuAccelerometer;
  imu.parameter_dimension = 1;
  imu.raw_group_maps[FactorGroupId(3)] =
      (Eigen::Vector2d() << 0.5, 1.0).finished();
  std::vector<FaultModeBasis> modes{uwb, imu};
  auto make_hypothesis = [](std::uint64_t id,
                            std::initializer_list<FaultModeId> mode_ids) {
    FaultHypothesisV2 value;
    value.id = HypothesisId(id);
    value.modes.assign(mode_ids.begin(), mode_ids.end());
    value.prior_probability_bound = 1e-4;
    value.p_md_allocation = 1e-3;
    value.hmi_allocation = 1e-6;
    return value;
  };
  std::vector<FaultHypothesisV2> reference_hypotheses{
      make_hypothesis(1, {FaultModeId(1)}),
      make_hypothesis(2, {FaultModeId(2)}),
      make_hypothesis(3, {FaultModeId(1), FaultModeId(2)})};
  auto optimized_hypotheses = reference_hypotheses;
  HypothesisEvaluationConfig reference_config;
  reference_config.enable_shared_context = false;
  reference_config.enable_low_dim_batch = false;
  const auto reference_evidence =
      HypothesisEvidenceEvaluator(reference_config)
          .evaluateAll(window, modes, &reference_hypotheses, 100.0);
  HypothesisEvaluationConfig optimized_config;
  optimized_config.hypothesis_workers = 4;
  CandidateWorkerPool pool(4);
  std::shared_ptr<const FrozenHypothesisNumerics> frozen;
  const auto optimized_evidence =
      HypothesisEvidenceEvaluator(optimized_config)
          .evaluateAll(window, modes, &optimized_hypotheses, 100.0, &frozen,
                       &pool);
  ASSERT_TRUE(frozen && frozen->valid) << (frozen ? frozen->reason : "missing");
  ASSERT_FALSE(frozen->pl_entries.empty());
  FrozenHypothesisPlProofV1 frozen_proof;
  ASSERT_TRUE(frozenHypothesisPlProof(
      frozen->pl_entries.front(), &frozen_proof));
  ASSERT_TRUE(validateFrozenHypothesisPlEntry(frozen_proof, nullptr));
  auto tampered_entry = frozen_proof;
  tampered_entry.certified_gram(0, 0) = std::nextafter(
      tampered_entry.certified_gram(0, 0),
      std::numeric_limits<double>::infinity());
  EXPECT_FALSE(validateFrozenHypothesisPlEntry(tampered_entry, nullptr));
  tampered_entry = frozen_proof;
  tampered_entry.gram_eigenvalues(0) = std::nextafter(
      tampered_entry.gram_eigenvalues(0),
      std::numeric_limits<double>::infinity());
  EXPECT_FALSE(validateFrozenHypothesisPlEntry(tampered_entry, nullptr));
  ASSERT_EQ(reference_evidence.size(), optimized_evidence.size());
  EXPECT_EQ(frozen->low_dimensional_count, optimized_hypotheses.size());
  EXPECT_EQ(frozen->generic_fallback_count, 0u);
  for (std::size_t i = 0; i < reference_evidence.size(); ++i) {
    EXPECT_EQ(reference_hypotheses[i].monitored,
              optimized_hypotheses[i].monitored);
    EXPECT_EQ(reference_evidence[i].plausible, optimized_evidence[i].plausible);
    EXPECT_NEAR(reference_evidence[i].conditioned_statistic,
                optimized_evidence[i].conditioned_statistic, 1e-12);
    EXPECT_NEAR(reference_evidence[i].explained_energy,
                optimized_evidence[i].explained_energy, 1e-12);
    EXPECT_TRUE(reference_evidence[i].fault_gram.isApprox(
        optimized_evidence[i].fault_gram, 1e-12));
  }

  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  RankUpdateConfig rank_config{1e-10, 1e10, 10.0};
  const RankUpdateEvaluator evaluator(rank_config);
  auto candidate = evaluator.evaluate(evaluator.factorizeOnce(window), keep);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  const auto detector =
      JointWindowDetector().evaluateCandidate(window, candidate, detector_risk);
  ProtectionLevelSharedContext ordinary_context;
  ordinary_context.mode_maps[1] = Eigen::MatrixXd::Zero(window.H.rows(), 1);
  ordinary_context.mode_maps[1].middleRows(2, 3) =
      uwb.raw_group_maps.begin()->second;
  ordinary_context.mode_maps[2] = Eigen::MatrixXd::Zero(window.H.rows(), 1);
  ordinary_context.mode_maps[2].bottomRows(2) =
      imu.raw_group_maps.begin()->second;
  auto ordinary_hypotheses = optimized_hypotheses;
  auto ordinary_candidate = candidate;
  const auto ordinary = ProtectionLevelV2().computeShared(
      window, &ordinary_candidate, detector, &ordinary_hypotheses,
      ordinary_context, RiskBudgetV2{});
  auto frozen_candidate = candidate;
  const auto frozen_work_before = NumericalWorkCounters::snapshot();
  const auto reused = ProtectionLevelV2().computeFrozenAllIn(
      window, &frozen_candidate, detector, optimized_hypotheses, modes, *frozen,
      frozen->fault_model_policy_fingerprint, RiskBudgetV2{});
  const auto frozen_work_after = NumericalWorkCounters::snapshot();
  ASSERT_EQ(ordinary.model_valid, reused.model_valid)
      << "ordinary=" << ordinary.reason << ";reused=" << reused.reason;
  EXPECT_EQ(frozen_work_after.square_root_fallbacks -
                frozen_work_before.square_root_fallbacks,
            0u);
  EXPECT_EQ(frozen_work_after.spectral_rhs_solves -
                frozen_work_before.spectral_rhs_solves,
            0u);
  EXPECT_TRUE(validateProtectionLevelV2Proof(
      frozen_candidate, detector, optimized_hypotheses, reused, nullptr));
  EXPECT_TRUE(ordinary.pl_xyz_m.isApprox(reused.pl_xyz_m, 1e-12));
  EXPECT_NEAR(ordinary.hpl_m, reused.hpl_m, 1e-12);
  EXPECT_NEAR(ordinary.vpl_m, reused.vpl_m, 1e-12);

  const auto stale_policy = ProtectionLevelV2().computeFrozenAllIn(
      window, &frozen_candidate, detector, optimized_hypotheses, modes, *frozen,
      frozen->fault_model_policy_fingerprint + 1, RiskBudgetV2{});
  EXPECT_FALSE(stale_policy.model_valid);
  EXPECT_NE(stale_policy.reason.find("identity mismatch"), std::string::npos);

  auto changed_modes = modes;
  changed_modes[0].raw_group_maps.begin()->second(0, 0) = std::nextafter(
      changed_modes[0].raw_group_maps.begin()->second(0, 0), 10.0);
  const auto stale_mode = ProtectionLevelV2().computeFrozenAllIn(
      window, &frozen_candidate, detector, optimized_hypotheses, changed_modes,
      *frozen, frozen->fault_model_policy_fingerprint, RiskBudgetV2{});
  EXPECT_FALSE(stale_mode.model_valid);
  EXPECT_NE(stale_mode.reason.find("identity mismatch"), std::string::npos);

  window.H(0, 0) = std::nextafter(window.H(0, 0), 10.0);
  const auto stale = ProtectionLevelV2().computeFrozenAllIn(
      window, &frozen_candidate, detector, optimized_hypotheses, modes, *frozen,
      frozen->fault_model_policy_fingerprint, RiskBudgetV2{});
  EXPECT_FALSE(stale.model_valid);
  EXPECT_NE(stale.reason.find("identity mismatch"), std::string::npos)
      << stale.reason;
}

TEST(P102SharedDualNumerics,
     ActiveDualKeepAllReusesEvidenceSmallBlocksWithoutChangingPl) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  auto history = block(
      4, Eigen::MatrixXd::Zero(2, 2), Eigen::Vector2d::Zero(),
      window.version);
  history.kind = FactorKind::BoundaryPrior;
  history.sensor = SensorType::Prior;
  history.role = RowRole::TrustedPrior;
  history.whitening_model_id = "history_summary_sqrt_d1";
  window.blocks.push_back(std::move(history));
  window.history_summary.present = true;
  window.history_summary.valid = true;
  window.history_summary.nu_perp = 2;
  window.history_summary.kappa_b = 0.0;
  window.history_summary.constant_offset = 0.0;
  window.history_summary.emitted_rows = 2;
  window.history_summary.detector_response = Eigen::MatrixXd::Zero(2, 0);
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  ASSERT_TRUE(window.model_valid) << window.reason;
  std::string window_proof_reason;
  ASSERT_TRUE(validateFrozenWindowNumericalProof(
      window, *window.numerics, &window_proof_reason))
      << window_proof_reason;
  const auto frozen_window = freezeIntegrityWindowCopy(window);
  const auto admission = admitFrozenIntegrityWindow(frozen_window);
  ASSERT_TRUE(admission) << admission.reason;
  FaultModeBasis mode;
  mode.id = FaultModeId(2101);
  mode.sensor = SensorType::Uwb;
  mode.parameter_dimension = 1;
  mode.raw_group_maps[FactorGroupId(1)] =
      Eigen::Vector2d::Ones();
  std::vector<FaultModeBasis> modes{mode};
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(2101);
  hypothesis.modes = {mode.id};
  hypothesis.prior_probability_bound = 1e-4;
  hypothesis.p_md_allocation = 1e-3;
  hypothesis.hmi_allocation = 1e-6;
  std::vector<FaultHypothesisV2> hypotheses{hypothesis};
  std::shared_ptr<const FrozenHypothesisNumerics> frozen;
  std::shared_ptr<const FrozenHypothesisDualNumerics> frozen_dual;
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      admission, modes, &hypotheses, 100.0, &frozen, &frozen_dual);
  ASSERT_EQ(evidence.size(), 1u);
  ASSERT_TRUE(frozen && frozen->valid);
  ASSERT_TRUE(frozen_dual && frozen_dual->valid);

  ExclusionAction keep;
  keep.id = ExclusionActionId(2101);
  keep.action_model_id = "KEEP_ALL";
  RankUpdateConfig rank_config{1e-10, 1e10, 10.0};
  RankUpdateEvaluator evaluator(rank_config);
  auto candidate = evaluator.evaluate(
      admission, evaluator.factorizeOnce(admission), keep);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  const auto detector = JointWindowDetector().evaluateCandidate(
      admission, candidate, detector_risk);
  ASSERT_EQ(detector.contract_mode, DetectorContractMode::ActiveDualChannelV6);
  ASSERT_GT(detector.channel_history_dof, 0);
  ASSERT_TRUE(detector.passed) << detector.reason;

  ProtectionLevelSharedContext reference_context;
  reference_context.mode_maps[mode.id.value()] =
      Eigen::MatrixXd::Zero(window.H.rows(), 1);
  reference_context.mode_maps[mode.id.value()].topRows(2) =
      Eigen::Vector2d::Ones();
  auto reference_hypotheses = hypotheses;
  auto reference_candidate = candidate;
  const auto reference = ProtectionLevelV2().computeShared(
      admission, &reference_candidate, detector, &reference_hypotheses,
      reference_context, RiskBudgetV2{});

  ASSERT_EQ(frozen_dual->dual_blocks.size(), hypotheses.size());
  ASSERT_TRUE(frozen_dual->dual_blocks.front().valid);
  EXPECT_EQ(frozen_dual->dual_blocks.front().dimension, 1);
  EXPECT_EQ(frozen_dual->dual_blocks.front().profile_statistic,
            evidence.front().profile_j);
  auto reused_candidate = candidate;
  const auto reused = ProtectionLevelV2().computeFrozenAllIn(
      admission, &reused_candidate, detector, hypotheses, modes, *frozen,
      *frozen_dual,
      frozen->fault_model_policy_fingerprint, RiskBudgetV2{});
  EXPECT_EQ(reference.model_valid, reused.model_valid)
      << "reference=" << reference.reason << ";reused=" << reused.reason;
  EXPECT_TRUE(reference.pl_xyz_m.isApprox(reused.pl_xyz_m, 1e-13))
      << "reference=" << reference.pl_xyz_m.transpose() << " ("
      << reference.reason << ");reused=" << reused.pl_xyz_m.transpose()
      << " (" << reused.reason << ")";
  EXPECT_EQ(reference.detector_certificate_id,
            reused.detector_certificate_id);
  std::string proof_reason;
  EXPECT_TRUE(validateProtectionLevelV2Proof(
      reused_candidate, detector, hypotheses, reused, &proof_reason))
      << proof_reason;

  std::string sealed_reason;
  ASSERT_TRUE(validateFrozenHypothesisDualProof(
      admission, *frozen, *frozen_dual, &sealed_reason)) << sealed_reason;

  // Without a typed admission there is no control-block proof.  The legacy
  // overload must not consume R08 matrices, even when every scalar hash is
  // identical.
  auto legacy_candidate = candidate;
  const auto legacy_rejected = ProtectionLevelV2().computeFrozenAllIn(
      admission.window(), &legacy_candidate, detector, hypotheses, modes,
      *frozen, frozen->fault_model_policy_fingerprint, RiskBudgetV2{});
  EXPECT_FALSE(legacy_rejected.model_valid);
  EXPECT_NE(legacy_rejected.reason.find("immutable owner binding"),
            std::string::npos) << legacy_rejected.reason;

  // Equal content is not equal ownership: a cache from A cannot be
  // transplanted into a separately sealed B.
  const auto other_window = freezeIntegrityWindowCopy(window);
  const auto other_admission = admitFrozenIntegrityWindow(other_window);
  ASSERT_TRUE(other_admission);
  ASSERT_EQ(admission.owner->content_hash,
            other_admission.owner->content_hash);
  ASSERT_NE(&admission.window(), &other_admission.window());
  auto other_candidate = evaluator.evaluate(
      other_admission, evaluator.factorizeOnce(other_admission), keep);
  ASSERT_TRUE(other_candidate.valid) << other_candidate.reason;
  const auto other_detector = JointWindowDetector().evaluateCandidate(
      other_admission, other_candidate, detector_risk);
  ASSERT_TRUE(other_detector.passed) << other_detector.reason;
  const auto transplanted = ProtectionLevelV2().computeFrozenAllIn(
      other_admission, &other_candidate, other_detector, hypotheses, modes,
      *frozen, *frozen_dual, frozen->fault_model_policy_fingerprint,
      RiskBudgetV2{});
  EXPECT_FALSE(transplanted.model_valid);
  EXPECT_NE(transplanted.reason.find("owner/payload identity mismatch"),
            std::string::npos) << transplanted.reason;

  // A forced 64-bit collision still cannot substitute ownership.  Admission
  // B is internally self-consistent, but its distinct payload/control block
  // cannot consume A's context.
  auto changed = window;
  changed.blocks.front().residual_whitened(0) = std::nextafter(
      changed.blocks.front().residual_whitened(0), 1.0);
  changed.blocks.front().residual_raw(0) =
      changed.blocks.front().residual_whitened(0);
  finalizeIntegrityWindow(&changed, 1e-10, 1e10);
  const auto ordinary_changed = freezeIntegrityWindowCopy(changed);
  auto collision_payload = std::make_shared<LinearizedIntegrityWindow>(
      *ordinary_changed.seal->payload);
  auto collision_numerics = std::make_shared<FrozenWindowNumerics>(
      *collision_payload->numerics);
  collision_numerics->content_fingerprint = admission.owner->content_hash;
  collision_payload->numerics = collision_numerics;
  auto collision_seal = std::make_shared<FrozenIntegrityWindowSeal>(
      *ordinary_changed.seal);
  collision_seal->content_hash = admission.owner->content_hash;
  collision_seal->payload = collision_payload;
  collision_seal->numerical_identity.content_fingerprint =
      admission.owner->content_hash;
  collision_seal->numerical_identity.proof_identity =
      frozenWindowNumericalProofIdentity(
          *collision_payload, *collision_numerics);
  collision_seal->numerical_identity.valid = true;
  const auto collision_admission = admitFrozenIntegrityWindow(
      FrozenIntegrityWindow{collision_seal});
  ASSERT_TRUE(collision_admission) << collision_admission.reason;
  std::string collision_reason;
  EXPECT_FALSE(validateFrozenHypothesisDualProof(
      collision_admission, *frozen, *frozen_dual, &collision_reason));
  EXPECT_NE(collision_reason.find("owner/payload identity mismatch"),
            std::string::npos) << collision_reason;

  auto expect_tamper_rejected = [&](FrozenHypothesisDualNumerics tampered) {
    std::string reason;
    EXPECT_FALSE(validateFrozenHypothesisDualProof(
        admission, *frozen, tampered, &reason));
    EXPECT_TRUE(reason.find("proof digest mismatch") != std::string::npos ||
                reason.find("structure mismatch") != std::string::npos)
        << reason;
  };
  auto stale_matrix = *frozen_dual;
  stale_matrix.candidate_detection_modes(0, 0) = std::nextafter(
      stale_matrix.candidate_detection_modes(0, 0), 1.0);
  expect_tamper_rejected(std::move(stale_matrix));
  auto stale_census = *frozen_dual;
  stale_census.candidate_hypothesis_fingerprint ^= 1;
  expect_tamper_rejected(std::move(stale_census));
  auto stale_id = *frozen_dual;
  stale_id.dual_blocks.front().hypothesis = HypothesisId(999999);
  expect_tamper_rejected(std::move(stale_id));
  auto missing_block = *frozen_dual;
  missing_block.dual_blocks.clear();
  std::string missing_reason;
  EXPECT_FALSE(validateFrozenHypothesisDualProof(
      admission, *frozen, missing_block, &missing_reason));
  EXPECT_NE(missing_reason.find("census structure mismatch"),
            std::string::npos) << missing_reason;
  auto bad_dimension = *frozen_dual;
  bad_dimension.dual_blocks.front().dimension = 4;
  std::string dimension_reason;
  EXPECT_FALSE(validateFrozenHypothesisDualProof(
      admission, *frozen, bad_dimension, &dimension_reason));
  EXPECT_NE(dimension_reason.find("block structure mismatch"),
            std::string::npos) << dimension_reason;

  auto modified_candidate = candidate;
  modified_candidate.action.action_model_id += "_MODIFIED";
  const auto stale_candidate = ProtectionLevelV2().computeFrozenAllIn(
      admission, &modified_candidate, detector, hypotheses, modes, *frozen,
      *frozen_dual,
      frozen->fault_model_policy_fingerprint, RiskBudgetV2{});
  EXPECT_FALSE(stale_candidate.model_valid);
  EXPECT_FALSE(stale_candidate.reason.empty());
}

TEST(P105FlatConcurrency,
     CartesianCandidateHypothesisBatchMatchesSerialForOneTwoFourWorkers) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  const auto owner = freezeIntegrityWindowCopy(window);
  const auto admission = admitFrozenIntegrityWindow(owner);
  ASSERT_TRUE(admission) << admission.reason;

  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  RankUpdateConfig rank_config{1e-10, 1e10, 10.0};
  RankUpdateEvaluator evaluator(rank_config);
  const auto base = evaluator.factorizeOnce(admission);
  auto candidate = evaluator.evaluate(admission, base, keep);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  const auto detector = JointWindowDetector().evaluateCandidate(
      admission, candidate, detector_risk);
  ASSERT_TRUE(detector.numerically_valid) << detector.reason;

  ProtectionLevelSharedContext context;
  context.mode_maps[41] = Eigen::MatrixXd::Zero(candidate.rows, 1);
  context.mode_maps[41](candidate.rows - 1, 0) = 1.0;
  std::vector<FaultHypothesisV2> hypotheses;
  for (std::uint64_t id = 1; id <= 4; ++id) {
    FaultHypothesisV2 hypothesis;
    hypothesis.id = HypothesisId(4100 + id);
    hypothesis.modes = {FaultModeId(41)};
    hypothesis.prior_probability_bound = 1e-4;
    hypothesis.p_md_allocation = 1e-3;
    hypothesis.hmi_allocation = 1e-6;
    hypotheses.push_back(std::move(hypothesis));
  }
  auto serial_candidate = candidate;
  auto serial_hypotheses = hypotheses;
  AttemptProofArena serial_arena;
  const auto serial = ProtectionLevelV2().computeShared(
      admission, &serial_candidate, detector, &serial_hypotheses, context,
      RiskBudgetV2{}, nullptr, &serial_arena);
  ASSERT_TRUE(serial.model_valid) << serial.reason;

  struct Digest {
    std::vector<ProtectionLevelV2Result> results;
    std::vector<std::uint64_t> proof_ids;
    std::vector<std::vector<bool>> monitored;
  };
  auto run = [&](std::size_t workers) {
    std::array<CandidateEvaluation, 2> candidates{
        evaluator.evaluate(admission, base, keep),
        evaluator.evaluate(admission, base, keep)};
    std::array<DetectorResultV2, 2> detectors{
        JointWindowDetector().evaluateCandidate(
            admission, candidates[0], detector_risk),
        JointWindowDetector().evaluateCandidate(
            admission, candidates[1], detector_risk)};
    std::array<std::vector<FaultHypothesisV2>, 2> candidate_hypotheses{
        hypotheses, hypotheses};
    std::array<ProtectionLevelSharedContext, 2> contexts{context, context};
    std::array<ProtectionLevelV2Result, 2> results;
    std::array<AttemptProofArena, 2> arenas;
    std::vector<FlatProtectionCandidateV1> jobs(2);
    for (std::size_t index = 0; index < jobs.size(); ++index) {
      jobs[index].candidate = &candidates[index];
      jobs[index].detector = &detectors[index];
      jobs[index].hypotheses = &candidate_hypotheses[index];
      jobs[index].shared = &contexts[index];
      jobs[index].result = &results[index];
      jobs[index].proof_arena = &arenas[index];
    }
    CandidateWorkerPool pool(4);
    ProtectionLevelV2().computeSharedFlatBatch(
        admission, &jobs, RiskBudgetV2{}, &pool, workers, 1u << 20);
    Digest digest;
    for (std::size_t index = 0; index < jobs.size(); ++index) {
      EXPECT_TRUE(results[index].model_valid) << results[index].reason;
      EXPECT_TRUE(results[index].pl_xyz_m.isApprox(serial.pl_xyz_m, 0.0));
      EXPECT_DOUBLE_EQ(results[index].hpl_m, serial.hpl_m);
      EXPECT_DOUBLE_EQ(results[index].vpl_m, serial.vpl_m);
      EXPECT_EQ(results[index].reason, serial.reason);
      EXPECT_TRUE(validateProtectionLevelV2Proof(
          candidates[index], detectors[index], candidate_hypotheses[index],
          results[index], arenas[index], nullptr));
      digest.results.push_back(results[index]);
      digest.proof_ids.push_back(protectionLevelV2ProofIdentity(
          candidates[index], results[index], arenas[index]));
      std::vector<bool> monitored;
      for (const auto& hypothesis : candidate_hypotheses[index]) {
        monitored.push_back(hypothesis.monitored);
      }
      digest.monitored.push_back(std::move(monitored));
    }
    return digest;
  };
  const auto one = run(1);
  for (const auto workers : {2u, 4u}) {
    const auto other = run(workers);
    ASSERT_EQ(other.results.size(), one.results.size());
    EXPECT_EQ(other.proof_ids, one.proof_ids);
    EXPECT_EQ(other.monitored, one.monitored);
    for (std::size_t index = 0; index < one.results.size(); ++index) {
      EXPECT_TRUE(other.results[index].pl_xyz_m.isApprox(
          one.results[index].pl_xyz_m, 0.0));
      EXPECT_DOUBLE_EQ(other.results[index].hpl_m, one.results[index].hpl_m);
      EXPECT_DOUBLE_EQ(other.results[index].vpl_m, one.results[index].vpl_m);
      EXPECT_EQ(other.results[index].model_valid,
                one.results[index].model_valid);
      EXPECT_EQ(other.results[index].availability,
                one.results[index].availability);
      EXPECT_EQ(other.results[index].reason, one.results[index].reason);
    }
  }
}

TEST(IntegrityV2ProtectionLevel,
     FixedLowDimKeepsSingularStatusAndUncommonDimensionFallsBack) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  FaultModeBasis first;
  first.id = FaultModeId(11);
  first.sensor = SensorType::Uwb;
  first.parameter_dimension = 1;
  first.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Ones();
  auto duplicate = first;
  duplicate.id = FaultModeId(12);
  FaultModeBasis uncommon;
  uncommon.id = FaultModeId(13);
  uncommon.sensor = SensorType::ImuAccelerometer;
  uncommon.parameter_dimension = 4;
  Eigen::Matrix<double, 3, 4> uncommon_map;
  uncommon_map << 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0;
  uncommon.raw_group_maps[FactorGroupId(2)] = uncommon_map;
  const std::vector<FaultModeBasis> modes{first, duplicate, uncommon};
  FaultHypothesisV2 singular;
  singular.id = HypothesisId(11);
  singular.modes = {first.id, duplicate.id};
  FaultHypothesisV2 dynamic;
  dynamic.id = HypothesisId(12);
  dynamic.modes = {uncommon.id};
  std::vector<FaultHypothesisV2> reference{singular, dynamic};
  auto optimized = reference;
  HypothesisEvaluationConfig reference_config;
  reference_config.enable_shared_context = false;
  reference_config.enable_low_dim_batch = false;
  const auto reference_evidence =
      HypothesisEvidenceEvaluator(reference_config)
          .evaluateAll(window, modes, &reference, 100.0);
  std::shared_ptr<const FrozenHypothesisNumerics> frozen;
  const auto optimized_evidence = HypothesisEvidenceEvaluator().evaluateAll(
      window, modes, &optimized, 100.0, &frozen);
  ASSERT_TRUE(frozen && frozen->valid);
  ASSERT_EQ(reference_evidence.size(), optimized_evidence.size());
  EXPECT_EQ(frozen->low_dimensional_count, 1u);
  EXPECT_EQ(frozen->generic_fallback_count, 1u);
  // B2 (§5.7): the optimized path refuses the concatenated hypothesis whose
  // two modes share one direction (A12 = [A1, A2] doubles the same column), so
  // no rank/Gram is computed for it.  The reference path has no such guard and
  // still reports the singular rank.
  EXPECT_EQ(reference[0].monitorability.rank, 1);
  EXPECT_EQ(optimized[0].monitorability.rank, 0);
  EXPECT_FALSE(reference[0].monitored);
  EXPECT_FALSE(optimized[0].monitored);
  EXPECT_NE(optimized[0].monitorability.reason.find("concatenation refused"),
            std::string::npos)
      << optimized[0].monitorability.reason;
  EXPECT_FALSE(optimized_evidence[0].plausible);
  EXPECT_EQ(optimized_evidence[0].fault_gram.rows(), 0);
  for (std::size_t i = 1; i < optimized.size(); ++i) {
    EXPECT_EQ(reference[i].monitored, optimized[i].monitored)
        << "reference sigma/condition="
        << reference[i].monitorability.sigma_min << "/"
        << reference[i].monitorability.condition_number
        << " optimized sigma/condition="
        << optimized[i].monitorability.sigma_min << "/"
        << optimized[i].monitorability.condition_number;
    EXPECT_EQ(reference[i].monitorability.rank,
              optimized[i].monitorability.rank);
    EXPECT_EQ(reference_evidence[i].plausible, optimized_evidence[i].plausible);
    EXPECT_TRUE(reference_evidence[i].fault_gram.isApprox(
        optimized_evidence[i].fault_gram, 1e-12));
  }
}

TEST(IntegrityV2Fde, CoversEntirePlausibleSetBeforeSelection) {
  uwb_imu_pl::DetectorResultV2 detector;
  detector.numerically_valid = true;
  detector.passed = false;
  uwb_imu_pl::FaultHypothesisV2 h1, h2;
  h1.id = uwb_imu_pl::HypothesisId(1);
  h1.units = {uwb_imu_pl::FaultUnitId(11)};
  h2.id = uwb_imu_pl::HypothesisId(2);
  h2.units = {uwb_imu_pl::FaultUnitId(12)};
  uwb_imu_pl::FaultModeEvidence e1, e2;
  e1.hypothesis = h1.id;
  e2.hypothesis = h2.id;
  e1.plausible = e2.plausible = true;
  // C3: a plausible hypothesis carries its comparability identity and raw
  // profile evidence; the FDE manager refuses to publish without them.
  e1.unit_kind = e2.unit_kind = uwb_imu_pl::FaultUnitKind::UwbRangeMeters;
  e1.parameter_dimension = e2.parameter_dimension = 1;
  e1.profile_j = e2.profile_j = 1.0;
  e1.profile_valid = e2.profile_valid = true;
  uwb_imu_pl::CandidateEvaluation one, both;
  one.valid = both.valid = true;
  one.post_detector_passed = both.post_detector_passed = true;
  one.hpl_m = one.vpl_m = both.hpl_m = both.vpl_m = 1.0;
  one.action.id = uwb_imu_pl::ExclusionActionId(1);
  one.action.covered_units = {h1.units.front()};
  one.action.exclusion_cardinality = 1;
  both.action.id = uwb_imu_pl::ExclusionActionId(2);
  both.action.covered_units = {h1.units.front(), h2.units.front()};
  both.action.exclusion_cardinality = 2;
  std::vector<uwb_imu_pl::CandidateEvaluation> candidates{one, both};
  uwb_imu_pl::FdeRiskDecisionV1 final_risk;
  uwb_imu_pl::FdeDecisionContextV1 context;
  context.risk_result = &final_risk;
  const auto decision = uwb_imu_pl::FdeManager().decide(
      detector, {h1, h2}, {e1, e2}, &candidates, {},
      uwb_imu_pl::RiskBudgetV2{}, &context);
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_EQ(decision.status, uwb_imu_pl::FdeStatus::RiskBudgetInvalid);
  EXPECT_FALSE(final_risk.complete_bound_closes);
  EXPECT_FALSE(decision.commit_allowed);
}

TEST(IntegrityV2Fde, PlausibleStrictSupersetRetainsCoverageDutyInSelection) {
  using namespace uwb_imu_pl;
  DetectorResultV2 detector;
  detector.numerically_valid = true;
  detector.passed = false;
  FaultHypothesisV2 singleton;
  singleton.id = HypothesisId(1);
  singleton.modes = {FaultModeId(11)};
  FaultHypothesisV2 superset;
  superset.id = HypothesisId(2);
  superset.modes = {FaultModeId(11), FaultModeId(12)};
  FaultModeEvidence e1, e2;
  e1.hypothesis = singleton.id;
  e2.hypothesis = superset.id;
  e1.plausible = e2.plausible = true;
  // C3: comparability identity + raw profile evidence of the fixture.
  e1.unit_kind = e2.unit_kind = FaultUnitKind::UwbRangeMeters;
  e1.parameter_dimension = e2.parameter_dimension = 1;
  e1.profile_j = e2.profile_j = 1.0;
  e1.profile_valid = e2.profile_valid = true;
  CandidateEvaluation only_uwb;
  only_uwb.valid = true;
  only_uwb.post_detector_passed = true;
  only_uwb.hpl_m = only_uwb.vpl_m = 1.0;
  only_uwb.action.id = ExclusionActionId(1);
  only_uwb.action.covered_modes = {FaultModeId(11)};
  only_uwb.action.exclusion_cardinality = 1;
  std::vector<CandidateEvaluation> candidates{only_uwb};
  const auto decision =
      FdeManager().decide(detector, {singleton, superset}, {e1, e2},
                          &candidates, {}, RiskBudgetV2{});
  EXPECT_EQ(decision.plausible_hypotheses,
            (std::vector<HypothesisId>{singleton.id, superset.id}));
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_FALSE(decision.commit_allowed);
  EXPECT_EQ(decision.status, FdeStatus::AmbiguousUnavailable);
  EXPECT_FALSE(candidates.front().covers_plausible_set);
}

// C3 wiring (W1): a plausible hypothesis without usable RAW profile evidence is
// a defect of the pipeline and refuses fail-closed instead of publishing on an
// unaudited pool.
TEST(IntegrityV2Fde, ProfilePoolRefusesMissingEvidenceFailClosed) {
  using namespace uwb_imu_pl;
  DetectorResultV2 detector;
  detector.numerically_valid = true;
  detector.passed = false;
  FaultHypothesisV2 fault;
  fault.id = HypothesisId(1);
  fault.units = {FaultUnitId(11)};
  FaultModeEvidence evidence;
  evidence.hypothesis = fault.id;
  evidence.plausible = true;
  // unit_kind / parameter_dimension / profile_j deliberately left unset.
  CandidateEvaluation candidate;
  candidate.valid = true;
  candidate.post_detector_passed = true;
  candidate.hpl_m = candidate.vpl_m = 1.0;
  candidate.action.id = ExclusionActionId(1);
  candidate.action.covered_units = {fault.units.front()};
  candidate.action.exclusion_cardinality = 1;
  std::vector<CandidateEvaluation> candidates{candidate};
  const auto decision =
      FdeManager().decide(detector, {fault}, {evidence}, &candidates, {},
                          RiskBudgetV2{});
  EXPECT_EQ(decision.status, FdeStatus::ModelInvalid);
  EXPECT_FALSE(decision.commit_allowed);
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_FALSE(decision.profile_pool_valid);
  EXPECT_NE(decision.reason.find("profile"), std::string::npos)
      << decision.reason;
}

// C3 wiring (W1): the selection risk is charged over the union of everything
// that may be published.  Alternative protections of the same plausible set are
// one failure event (one class, one charge); the ledger is reported on the
// decision.
TEST(IntegrityV2Fde, SelectionRiskChargedOverPublishableSet) {
  using namespace uwb_imu_pl;
  DetectorResultV2 detector;
  detector.numerically_valid = true;
  detector.passed = false;
  FaultHypothesisV2 h1, h2;
  h1.id = HypothesisId(1);
  h1.units = {FaultUnitId(11)};
  h1.hmi_allocation = 1e-6;
  h2.id = HypothesisId(2);
  h2.units = {FaultUnitId(12)};
  h2.hmi_allocation = 2e-6;
  FaultModeEvidence e1, e2;
  e1.hypothesis = h1.id;
  e2.hypothesis = h2.id;
  e1.plausible = e2.plausible = true;
  e1.unit_kind = e2.unit_kind = FaultUnitKind::UwbRangeMeters;
  e1.parameter_dimension = e2.parameter_dimension = 1;
  e1.profile_j = e2.profile_j = 1.0;
  e1.profile_valid = e2.profile_valid = true;
  CandidateEvaluation one, both;
  one.valid = both.valid = true;
  one.post_detector_passed = both.post_detector_passed = true;
  one.hpl_m = one.vpl_m = both.hpl_m = both.vpl_m = 1.0;
  one.action.id = ExclusionActionId(1);
  one.action.covered_units = {h1.units.front()};
  one.action.exclusion_cardinality = 1;
  both.action.id = ExclusionActionId(2);
  both.action.covered_units = {h1.units.front(), h2.units.front()};
  both.action.exclusion_cardinality = 2;
  std::vector<CandidateEvaluation> candidates{one, both};
  FdeRiskDecisionV1 final_risk;
  FdeDecisionContextV1 context;
  context.risk_result = &final_risk;
  const auto decision = FdeManager().decide(
      detector, {h1, h2}, {e1, e2}, &candidates, {}, RiskBudgetV2{},
      &context);
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_TRUE(decision.profile_pool_valid);
  EXPECT_TRUE(decision.profile_pool_comparable);
  EXPECT_TRUE(decision.profile_pool_ranked);
  EXPECT_EQ(decision.plausible_profile_j, (std::vector<double>{1.0, 1.0}));
  // Only `both` covers the complete plausible set, so exactly one original
  // eligible action enters grouping.  The independent P0-04 oracle covers the
  // two-eligible-action disjoint-event counterexample.
  EXPECT_EQ(decision.selection_event_classes, 1u);
  EXPECT_TRUE(decision.selection_budget_ok);
  EXPECT_DOUBLE_EQ(decision.selection_charged_budget, 3e-6);
  EXPECT_DOUBLE_EQ(decision.selection_available_budget,
                   RiskBudgetV2{}.p_hmi_total);
  EXPECT_FALSE(final_risk.complete_bound_closes);
  EXPECT_FALSE(decision.commit_allowed);
  ASSERT_EQ(decision.candidate_dispositions.size(), 2u);
  // The candidate that covers the complete plausible set with a finite
  // protected reference is the reference-estimate path.
  EXPECT_EQ(decision.candidate_dispositions[1],
            static_cast<int>(CandidateDisposition::UseInReferenceEstimate));
}

// C3 wiring (W1): profiles from different physical units are recorded as not
// comparable and are never ranked; the frozen action ordering is unaffected.
TEST(IntegrityV2Fde, MixedUnitPoolIsRecordedAndNotRanked) {
  using namespace uwb_imu_pl;
  DetectorResultV2 detector;
  detector.numerically_valid = true;
  detector.passed = false;
  FaultHypothesisV2 uwb, imu;
  uwb.id = HypothesisId(1);
  uwb.units = {FaultUnitId(11)};
  imu.id = HypothesisId(2);
  imu.units = {FaultUnitId(12)};
  FaultModeEvidence e1, e2;
  e1.hypothesis = uwb.id;
  e2.hypothesis = imu.id;
  e1.plausible = e2.plausible = true;
  e1.unit_kind = FaultUnitKind::UwbRangeMeters;
  e2.unit_kind = FaultUnitKind::ImuAccelMps2;
  e1.parameter_dimension = 1;
  e2.parameter_dimension = 3;
  e1.profile_j = 1.0;
  e2.profile_j = 2.0;
  e1.profile_valid = e2.profile_valid = true;
  CandidateEvaluation both;
  both.valid = true;
  both.post_detector_passed = true;
  both.hpl_m = both.vpl_m = 1.0;
  both.action.id = ExclusionActionId(1);
  both.action.covered_units = {uwb.units.front(), imu.units.front()};
  both.action.exclusion_cardinality = 2;
  std::vector<CandidateEvaluation> candidates{both};
  const auto decision =
      FdeManager().decide(detector, {uwb, imu}, {e1, e2}, &candidates, {},
                          RiskBudgetV2{});
  EXPECT_TRUE(decision.profile_pool_valid);
  EXPECT_TRUE(decision.profile_pool_mixed_units);
  EXPECT_TRUE(decision.profile_pool_mixed_dimensions);
  EXPECT_FALSE(decision.profile_pool_comparable);
  EXPECT_FALSE(decision.profile_pool_ranked);
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_EQ(decision.status, FdeStatus::RiskBudgetInvalid);
  EXPECT_FALSE(decision.commit_allowed);
}

TEST(IntegrityV2Fde, ActionDedupRejectsSameShapeAndNormDifferentContent) {
  using namespace uwb_imu_pl;
  ExclusionAction left, right;
  left.groups_to_remove = right.groups_to_remove = {FactorGroupId(10)};
  left.groups_to_add = right.groups_to_add = {FactorGroupId(11)};
  left.bridge_mode = right.bridge_mode = BridgeMode::GenericKinematic;
  LinearizedFactorBlock a, b;
  a.group_id = b.group_id = FactorGroupId(11);
  a.version = b.version = {1, 2, 3, 4};
  a.jacobian_whitened = Eigen::Matrix2d::Identity();
  b.jacobian_whitened = (Eigen::Matrix2d() << 0.0, 1.0, 1.0, 0.0).finished();
  a.residual_whitened = Eigen::Vector2d(1.0, 0.0);
  b.residual_whitened = Eigen::Vector2d(0.0, 1.0);
  ASSERT_DOUBLE_EQ(a.jacobian_whitened.squaredNorm(),
                   b.jacobian_whitened.squaredNorm());
  ASSERT_DOUBLE_EQ(a.residual_whitened.squaredNorm(),
                   b.residual_whitened.squaredNorm());
  left.added_blocks = {a};
  right.added_blocks = {b};
  EXPECT_FALSE(equivalentActionOperation(left, right));
  right.added_blocks = left.added_blocks;
  EXPECT_TRUE(equivalentActionOperation(left, right));
}

TEST(IntegrityV2Fde, MandatoryGroupIsIndependentOfProbabilisticPlausibility) {
  using namespace uwb_imu_pl;
  DetectorResultV2 barrier;
  barrier.numerically_valid = true;
  barrier.passed = false;
  CandidateEvaluation keep, exclude;
  keep.valid = exclude.valid = true;
  keep.post_detector_passed = exclude.post_detector_passed = true;
  keep.hpl_m = keep.vpl_m = exclude.hpl_m = exclude.vpl_m = 1.0;
  keep.action.id = ExclusionActionId(1);
  keep.action.action_model_id = "KEEP_ALL";
  exclude.action.id = ExclusionActionId(2);
  exclude.action.action_model_id = "UWB_CURRENT_HEALTH_BARRIER";
  exclude.action.groups_to_remove = {FactorGroupId(77)};
  exclude.action.exclusion_cardinality = 1;
  std::vector<CandidateEvaluation> candidates{keep, exclude};
  const auto decision = FdeManager().decide(
      barrier, {}, {}, &candidates, {FactorGroupId(77)}, RiskBudgetV2{});
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_EQ(decision.status, FdeStatus::RiskBudgetInvalid);
  EXPECT_TRUE(decision.plausible_hypotheses.empty());
  EXPECT_EQ(decision.mandatory_exclusion_groups,
            (std::vector<FactorGroupId>{FactorGroupId(77)}));
  EXPECT_FALSE(candidates[0].covers_plausible_set);
  EXPECT_TRUE(candidates[1].covers_plausible_set);
}

TEST(IntegrityV2Health, QuarantineRequiresConsecutiveRecovery) {
  uwb_imu_pl::HealthConfigV2 config;
  config.suspect_evidence_count = 2;
  config.recovery_shadow_passes = 2;
  config.recovery_test_passes = 2;
  uwb_imu_pl::HealthManager health(config);
  health.registerSource("accel:x", uwb_imu_pl::SensorType::ImuAccelerometer);
  EXPECT_EQ(health.observeEvidence("accel:x", true).current,
            uwb_imu_pl::HealthState::Healthy);
  EXPECT_EQ(health.observeEvidence("accel:x", true).current,
            uwb_imu_pl::HealthState::Suspect);
  EXPECT_EQ(health.quarantine("accel:x", "exclude").current,
            uwb_imu_pl::HealthState::Quarantined);
  EXPECT_EQ(health.observeShadowRecovery("accel:x", true).current,
            uwb_imu_pl::HealthState::Quarantined);
  EXPECT_EQ(health.observeShadowRecovery("accel:x", true).current,
            uwb_imu_pl::HealthState::RecoveryTest);
  EXPECT_EQ(health.observeShadowRecovery("accel:x", true).current,
            uwb_imu_pl::HealthState::RecoveryTest);
  EXPECT_EQ(health.observeShadowRecovery("accel:x", true).current,
            uwb_imu_pl::HealthState::Healthy);
  EXPECT_EQ(health.fail("accel:x", "hardware barrier").current,
            uwb_imu_pl::HealthState::Failed);
  EXPECT_FALSE(health.allowedInFormalEstimator("accel:x"));
}

TEST(IntegrityV2Health, RecoveryFailureResetsCountersAndRequarantines) {
  uwb_imu_pl::HealthConfigV2 config;
  config.suspect_evidence_count = 1;
  config.recovery_shadow_passes = 2;
  config.recovery_test_passes = 10;
  uwb_imu_pl::HealthManager health(config);
  health.registerSource("accel:x", uwb_imu_pl::SensorType::ImuAccelerometer);
  health.observeEvidence("accel:x", true);
  health.quarantine("accel:x", "exclude");
  health.observeShadowRecovery("accel:x", true);
  health.observeShadowRecovery("accel:x", true);
  for (int pass = 1; pass < 5; ++pass) {
    health.observeShadowRecovery("accel:x", true);
  }
  EXPECT_EQ(health.state("accel:x"), uwb_imu_pl::HealthState::RecoveryTest);
  health.observeShadowRecovery("accel:x", false);
  EXPECT_EQ(health.state("accel:x"), uwb_imu_pl::HealthState::Quarantined);
  ASSERT_EQ(health.snapshot().size(), 1u);
  EXPECT_EQ(health.snapshot().front().shadow_pass_count, 0u);
  EXPECT_EQ(health.snapshot().front().recovery_pass_count, 0u);
  EXPECT_EQ(health.snapshot().front().recovery_reset_count, 1u);
}

TEST(GateDNumerics, CompleteReplacementSurvivesSingularDeletionIntermediate) {
  using namespace uwb_imu_pl;
  LinearizedIntegrityWindow window;
  window.id = WindowId(80);
  window.version = {9, 8, 7, 6};
  window.blocks = {block(1, Eigen::Matrix2d::Identity(),
                         Eigen::Vector2d(.01, -.02), window.version),
                   block(2, Eigen::MatrixXd::Zero(1, 2),
                         Eigen::VectorXd::Constant(1, .03), window.version)};
  window.protected_state_map = Eigen::MatrixXd::Zero(3, 2);
  window.protected_state_map.topRows(2).setIdentity();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  ASSERT_TRUE(window.model_valid) << window.reason;
  ExclusionAction action;
  action.groups_to_remove = {FactorGroupId(1)};
  action.added_blocks = {block(3, 2.0 * Eigen::Matrix2d::Identity(),
                               Eigen::Vector2d(.03, .04), window.version)};
  RankUpdateConfig config;
  config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator evaluator(config);
  const auto base = evaluator.factorizeOnce(window);
  const auto result = evaluator.evaluate(base, action);
  const auto oracle = DenseCandidateOracle(config).evaluate(window, action);
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_TRUE(oracle.valid) << oracle.reason;
  EXPECT_TRUE(result.diagnostics.recovered_replacement);
  EXPECT_TRUE(result.state_increment.isApprox(oracle.state_increment, 1e-9));
  EXPECT_TRUE(result.covarianceTimes(window.protected_state_map.transpose())
                  .isApprox(oracle.covarianceTimes(
                                window.protected_state_map.transpose()),
                            1e-9));
  EXPECT_NEAR(result.statistic, oracle.statistic, 1e-9);
  EXPECT_NEAR(result.information_logdet, oracle.information_logdet, 1e-9);
  EXPECT_EQ(result.dof, oracle.dof);
  EXPECT_EQ(result.covariance.size(), 0);
  action.added_blocks.clear();
  const auto singular = evaluator.evaluate(base, action);
  EXPECT_FALSE(singular.valid);
  EXPECT_EQ(singular.reason, "candidate rank loss");
}

TEST(GateDNumerics, FrozenNumericalContractMismatchFailsClosed) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  DetectorRiskContext detector;
  detector.rank_tolerance = 2e-12;
  detector.max_condition_number = 1e10;
  const auto detected = JointWindowDetector().evaluate(window, detector);
  EXPECT_FALSE(detected.numerically_valid);
  EXPECT_NE(detected.reason.find("numerical-contract-mismatched"),
            std::string::npos);

  RankUpdateConfig rank;
  rank.rank_tolerance = 2e-12;
  rank.max_condition_number = 1e10;
  const auto base = RankUpdateEvaluator(rank).factorizeOnce(window);
  EXPECT_FALSE(base.valid);
  EXPECT_NE(base.reason.find("stale frozen-window numerics"),
            std::string::npos);

  finalizeIntegrityWindow(&window, detector.rank_tolerance,
                          detector.max_condition_number);
  EXPECT_TRUE(
      JointWindowDetector().evaluate(window, detector).numerically_valid);
  EXPECT_TRUE(RankUpdateEvaluator(rank).factorizeOnce(window).valid);
}

TEST(GateDNumerics, FinalSvdRejectsConditionHiddenByCholeskyDiagonal) {
  using namespace uwb_imu_pl;
  LinearizedIntegrityWindow window;
  window.version = {1, 2, 3, 4};
  Eigen::MatrixXd h(3, 2);
  h << 1, 1000, 0, 1, 0, 0;
  window.blocks = {block(1, h, Eigen::Vector3d(.01, .001, .02), window.version),
                   block(2, 1000 * Eigen::Matrix2d::Identity(),
                         Eigen::Vector2d::Zero(), window.version)};
  finalizeIntegrityWindow(&window, 1e-10, 1e4);
  ASSERT_LT(window.condition_number, 2);
  Eigen::LLT<Eigen::MatrixXd> misleading(h.transpose() * h);
  ASSERT_EQ(misleading.info(), Eigen::Success);
  const auto diag = misleading.matrixL().toDenseMatrix().diagonal().eval();
  EXPECT_NEAR(diag.maxCoeff() / diag.minCoeff(), 1, 1e-9);
  RankUpdateConfig config;
  config.max_condition_number = 1e4;
  config.exact_slow_path_condition = 1e4;
  config.enable_early_step_gate = false;
  config.materialize_dense_oracle_fields = false;
  ExclusionAction action;
  action.groups_to_remove = {FactorGroupId(2)};
  const RankUpdateEvaluator evaluator(config);
  const auto result =
      evaluator.evaluate(evaluator.factorizeOnce(window), action);
  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.condition_number, 1e5);
  EXPECT_EQ(result.reason, "candidate condition gate failed");
  EXPECT_TRUE(result.exact_slow_path);
}

TEST(GateDNumerics, EuclideanStepGateBothSidesUsesReferenceNearBoundary) {
  using namespace uwb_imu_pl;
  for (double scale : {1 - 1e-8, 1 + 1e-8}) {
    auto window = syntheticWindow();
    Eigen::Vector2d delta(.15, .2);
    delta *= scale;
    for (auto& b : window.blocks)
      b.residual_whitened = b.jacobian_whitened * delta;
    finalizeIntegrityWindow(&window, 1e-10, 1e10);
    RankUpdateConfig config;
    config.materialize_dense_oracle_fields = false;
    const RankUpdateEvaluator evaluator(config);
    const auto result =
        evaluator.evaluate(evaluator.factorizeOnce(window), ExclusionAction{});
    EXPECT_EQ(result.valid, scale < 1) << result.reason;
    EXPECT_TRUE(result.diagnostics.near_gate);
    EXPECT_TRUE(result.exact_slow_path);
    EXPECT_NEAR(result.state_increment.norm(), .25 * scale, 1e-12);
    EXPECT_EQ(result.covariance.size(), 0);
  }
}

TEST(GateDNumerics, MalformedFullChangeRejectedBeforeAnyPartialOperation) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  const RankUpdateEvaluator evaluator;
  const auto base = evaluator.factorizeOnce(window);
  ExclusionAction action;
  action.groups_to_remove = {FactorGroupId(1)};
  auto addition = window.blocks.front();
  addition.group_id = FactorGroupId(90);
  action.added_blocks = {addition};
  for (int which = 0; which < 7; ++which) {
    auto& b = action.added_blocks.front();
    b = addition;
    if (which == 0) b.version.ordering_version++;
    if (which == 1) b.version.noise_model_version++;
    if (which == 2) b.residual_whitened.resize(0);
    if (which == 3)
      b.jacobian_whitened(0, 0) = std::numeric_limits<double>::quiet_NaN();
    if (which == 4) b.whitener(0, 0) = std::numeric_limits<double>::quiet_NaN();
    if (which == 5) b.covariance.resize(1, 3);
    if (which == 6) b.jacobian_raw.resize(1, 3);
    const auto c = evaluator.evaluate(base, action);
    EXPECT_FALSE(c.valid);
    EXPECT_EQ(c.reason, "candidate addition block/version is invalid");
    EXPECT_GE(c.wall_ms, 0);
  }
}

TEST(GateDCache, CacheContentVersionWindowAndSwitchRemainEquivalent) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  ExclusionAction a;
  a.id = ExclusionActionId(7);
  a.groups_to_remove = {FactorGroupId(3)};
  auto added = window.blocks.back();
  added.group_id = FactorGroupId(10);
  a.added_blocks = {added};
  auto changed = a;
  changed.id = ExclusionActionId(8);
  changed.added_blocks.front().jacobian_whitened *= 1.2;
  auto actions = std::vector<ExclusionAction>{a, changed};
  RankUpdateConfig config;
  config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator cached(config);
  config.enable_shared_cache = false;
  const RankUpdateEvaluator uncached(config);
  const auto base = cached.factorizeOnce(window, actions);
  ASSERT_TRUE(base.block_cache);
  EXPECT_EQ(base.block_cache->blocks.at(FactorGroupId(10)).size(), 2u);
  for (const auto& action : actions) {
    const auto c = cached.evaluate(base, action);
    const auto u =
        uncached.evaluate(uncached.factorizeOnce(window, actions), action);
    ASSERT_TRUE(c.valid) << c.reason;
    EXPECT_TRUE(c.state_increment.isApprox(u.state_increment, 1e-9));
    EXPECT_TRUE(
        c.covarianceTimes(window.protected_state_map.transpose())
            .isApprox(u.covarianceTimes(window.protected_state_map.transpose()),
                      1e-9));
    EXPECT_EQ(c.diagnostics.cache_hits, 2u);
    EXPECT_EQ(u.diagnostics.cache_hits, 0u);
  }
  // Unseen content with an existing ID must miss instead of using the old
  // solve.
  auto unseen = a;
  unseen.added_blocks.front().residual_raw *= 2;
  EXPECT_EQ(cached.evaluate(base, unseen).diagnostics.cache_hits, 1u);
  auto stale = a;
  stale.added_blocks.front().version.linpoint_version++;
  EXPECT_FALSE(cached.evaluate(base, stale).valid);
  auto next = window;
  next.id = WindowId(999);
  finalizeIntegrityWindow(&next, 1e-10, 1e10);
  EXPECT_EQ(cached.factorizeOnce(next, actions).block_cache->window_id,
            next.id);
  next.version.noise_model_version++;
  for (auto& b : next.blocks) b.version = next.version;
  finalizeIntegrityWindow(&next, 1e-10, 1e10);
  EXPECT_FALSE(cached.evaluate(cached.factorizeOnce(next, actions), a).valid);
}

TEST(GateDCache, EarlyStepSwitchAgreesWithExactFinalJacobian) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  for (auto& b : window.blocks)
    b.residual_whitened = b.jacobian_whitened * Eigen::Vector2d(1, 2);
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  RankUpdateConfig config;
  config.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator early(config);
  config.enable_early_step_gate = false;
  const RankUpdateEvaluator exact(config);
  const auto fast =
      early.evaluate(early.factorizeOnce(window), ExclusionAction{});
  const auto full =
      exact.evaluate(exact.factorizeOnce(window), ExclusionAction{});
  EXPECT_FALSE(fast.valid);
  EXPECT_FALSE(full.valid);
  EXPECT_EQ(fast.reason, full.reason);
  EXPECT_EQ(fast.diagnostics.numerical_path, "QR_CERTIFIED_STEP_REJECTION");
  EXPECT_TRUE(fast.state_increment.isApprox(full.state_increment, 1e-9));
}

TEST(GateDReplay, LosslessRoundTripAllBlocksAndActions) {
  using namespace uwb_imu_pl;
  FrozenCandidateReplay replay;
  replay.window = syntheticWindow();
  replay.input_attempt_id = 123;
  replay.transaction_id = 33;
  replay.input_timestamp = TimestampNs(99887766);
  replay.config.enable_shared_cache = false;
  replay.config.enable_early_step_gate = false;
  ExclusionAction action;
  action.id = ExclusionActionId(42);
  action.groups_to_remove = {FactorGroupId(3)};
  action.added_blocks = {replay.window.blocks.back()};
  action.added_blocks.front().group_id = FactorGroupId(100);
  action.covered_modes = {FaultModeId(91)};
  action.covered_units = {FaultUnitId(19)};
  action.physical_source_ids = {"source with spaces"};
  action.recovery_epoch_begin = 5;
  action.recovery_epoch_end = 8;
  // C4/W2 replay v6: the removal provenance is explicit wire content.
  action.removal_data_source = "imu_accel_interval_7";
  action.model_error_record = "model_error:imu_interval_bound";
  action.model_error_validated = true;
  replay.actions = {action, action};
  replay.actions.back().id = ExclusionActionId(43);
  // D round: v6 is documented as "v5 layout + removal provenance"; the v5
  // factor-inventory section must stay on the wire.  A regression that dropped
  // it (as the W2 v6 writer did) desynchronizes every v4/v5/v6 reader that
  // follows the documented layout, so pin it in the round-trip.
  replay.window.factor_inventory.clear();
  FrozenWindowFactorInventoryEntry inventory_entry;
  inventory_entry.group_id = FactorGroupId(7);
  inventory_entry.epoch = 3;
  inventory_entry.kind = FactorKind::UwbBatch;
  inventory_entry.sensor = SensorType::Uwb;
  inventory_entry.disposition = FrozenFactorDisposition::ExplicitMeasurement;
  inventory_entry.keys = {gtsam::Symbol('x', 3)};
  inventory_entry.slots = {0, 1};
  replay.window.factor_inventory.push_back(inventory_entry);
  const std::string path =
      "/tmp/gate-d-replay-" + std::to_string(getpid()) + ".bin";
  writeCandidateReplay(path, replay);
  const auto loaded = readCandidateReplay(path);
  std::remove(path.c_str());
  ASSERT_EQ(loaded.window.factor_inventory.size(),
            replay.window.factor_inventory.size());
  EXPECT_EQ(loaded.window.factor_inventory.front().group_id.value(),
            replay.window.factor_inventory.front().group_id.value());
  EXPECT_EQ(loaded.window.factor_inventory.front().epoch,
            replay.window.factor_inventory.front().epoch);
  EXPECT_EQ(loaded.window.factor_inventory.front().kind,
            replay.window.factor_inventory.front().kind);
  EXPECT_EQ(loaded.window.factor_inventory.front().sensor,
            replay.window.factor_inventory.front().sensor);
  EXPECT_EQ(loaded.window.factor_inventory.front().disposition,
            replay.window.factor_inventory.front().disposition);
  EXPECT_EQ(loaded.window.factor_inventory.front().keys,
            replay.window.factor_inventory.front().keys);
  EXPECT_EQ(loaded.window.factor_inventory.front().slots,
            replay.window.factor_inventory.front().slots);
  EXPECT_EQ(loaded.input_attempt_id, replay.input_attempt_id);
  EXPECT_EQ(loaded.config.enable_shared_cache,
            replay.config.enable_shared_cache);
  EXPECT_EQ(loaded.config.enable_early_step_gate,
            replay.config.enable_early_step_gate);
  EXPECT_EQ(loaded.input_timestamp, replay.input_timestamp);
  EXPECT_EQ(loaded.window.version, replay.window.version);
  EXPECT_TRUE(loaded.window.H.isApprox(replay.window.H, 0));
  EXPECT_TRUE(loaded.window.protected_state_map.isApprox(
      replay.window.protected_state_map, 0));
  ASSERT_EQ(loaded.actions.size(), 2u);
  EXPECT_EQ(loaded.actions[0].covered_modes, action.covered_modes);
  EXPECT_EQ(loaded.actions[0].physical_source_ids, action.physical_source_ids);
  EXPECT_EQ(loaded.actions[0].recovery_epoch_end, action.recovery_epoch_end);
  // v6 round-trip: the removal provenance fields survive byte-for-byte.
  EXPECT_EQ(loaded.actions[0].removal_data_source,
            action.removal_data_source);
  EXPECT_EQ(loaded.actions[0].model_error_record, action.model_error_record);
  EXPECT_EQ(loaded.actions[0].model_error_validated,
            action.model_error_validated);
  EXPECT_TRUE(loaded.actions[0].added_blocks[0].jacobian_whitened.isApprox(
      action.added_blocks[0].jacobian_whitened, 0));
  const RankUpdateEvaluator evaluator;
  auto base = evaluator.factorizeOnce(loaded.window, loaded.actions);
  EXPECT_TRUE(evaluator.evaluate(base, loaded.actions[0]).valid);
}

TEST(GateDOracle,
     RealCurrentHistoricalBridgeAndUnionActionsMatchDenseAndWorkers) {
  using namespace uwb_imu_pl;
  const auto cfg = researchConfig();
  IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, cfg.realtime.prior_sigmas);
  addImu(&estimator, cfg.imu.gravity_mps2);
  auto first = estimator.prepareEpoch(batch(cfg, 10000000));
  auto plan = EpochCommitPlan::nominalPlan(first);
  estimator.commitEpoch(std::move(first), plan);
  for (int i = 3; i <= 4; ++i) {
    ImuMeasurement imu;
    imu.timestamp = TimestampNs(i * 5000000);
    imu.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
    estimator.ingestImu(imu);
  }
  auto input = batch(cfg, 20000000);
  const auto n = input.measurements.size();
  input.covariance_m2 = .0025 * Eigen::MatrixXd::Identity(n, n) +
                        .0001 * Eigen::MatrixXd::Ones(n, n);
  auto tx = estimator.prepareEpoch(input);
  const auto updates = estimator.backendUpdateCount();
  const auto window =
      estimator.buildIntegrityWindow(tx, IntegrityWindowRequest{});
  auto imu = estimator.buildPendingFactorBlock(tx, tx.imu_group.id);
  auto bridge =
      estimator.buildPendingFactorBlock(tx, tx.generic_bridge_group.id);
  auto models = HypothesisGenerator().generate(
      window, tx, ImuFaultSubspaceBuilder().build(tx, imu), bridge);
  std::vector<FaultModeEvidence> evidence;
  for (const auto& h : models.hypotheses) {
    FaultModeEvidence e;
    e.hypothesis = h.id;
    e.plausible = true;
    evidence.push_back(e);
  }
  auto actions = HypothesisGenerator().actionsForPlausibleSet(
      window, tx, &models, evidence);
  ASSERT_GT(actions.size(), 10u);
  RankUpdateConfig config;
  config.materialize_dense_oracle_fields = false;
  config.enable_early_step_gate = false;
  const RankUpdateEvaluator evaluator(config);
  const auto base = evaluator.factorizeOnce(window, actions);
  std::vector<CandidateEvaluation> serial;
  for (const auto& a : actions) serial.push_back(evaluator.evaluate(base, a));
  std::vector<
      std::future<std::vector<std::pair<std::size_t, CandidateEvaluation>>>>
      workers;
  for (std::size_t worker = 0; worker < 4; ++worker)
    workers.push_back(std::async(std::launch::async, [&, worker] {
      std::vector<std::pair<std::size_t, CandidateEvaluation>> out;
      for (std::size_t i = worker; i < actions.size(); i += 4)
        out.emplace_back(i, evaluator.evaluate(base, actions[i]));
      return out;
    }));
  for (auto& worker : workers)
    for (const auto& item : worker.get()) {
      EXPECT_EQ(item.second.action.id, serial[item.first].action.id);
      EXPECT_EQ(item.second.valid, serial[item.first].valid);
      ASSERT_EQ(item.second.state_increment.rows(),
                serial[item.first].state_increment.rows())
          << item.first << ": " << item.second.reason << "/"
          << serial[item.first].reason;
      EXPECT_TRUE(item.second.state_increment.isApprox(
          serial[item.first].state_increment, 1e-9));
    }
  std::size_t valid = 0, bridges = 0, history = 0;
  for (std::size_t i = 0; i < actions.size(); ++i) {
    const auto& c = serial[i];
    const auto oracle =
        DenseCandidateOracle(config).evaluate(window, actions[i]);
    ASSERT_EQ(c.valid, oracle.valid)
        << i << ": " << c.reason << "/" << oracle.reason;
    if (!c.valid) continue;
    ++valid;
    bridges += actions[i].bridge_mode != BridgeMode::None;
    history += actions[i].recovery_epoch_begin.has_value();
    EXPECT_LE((c.state_increment - oracle.state_increment).norm(), 1e-9);
    const auto pc = c.covarianceTimes(window.protected_state_map.transpose());
    const auto po =
        oracle.covarianceTimes(window.protected_state_map.transpose());
    ASSERT_EQ(pc.rows(), po.rows())
        << i << ": certified covariance solve returned " << pc.rows()
        << " rows for action " << actions[i].action_model_id;
    ASSERT_EQ(pc.cols(), po.cols()) << i;
    EXPECT_LE((pc - po).norm() / std::max(1e-15, po.norm()), 1e-7);
    EXPECT_NEAR(c.statistic, oracle.statistic, 1e-7);
    EXPECT_LE(std::abs(c.information_logdet - oracle.information_logdet) /
                  std::max(1e-15, std::abs(oracle.information_logdet)),
              1e-7);
    EXPECT_EQ(c.dof, oracle.dof);
    const auto dc =
        JointWindowDetector().evaluateCandidate(window, c, DetectorRiskContext{});
    const auto od =
        JointWindowDetector().evaluateCandidate(
            window, oracle, DetectorRiskContext{});
    EXPECT_EQ(dc.passed, od.passed);
    EXPECT_NEAR(dc.squared_threshold, od.squared_threshold, 1e-9);
    Eigen::Vector3d bc = Eigen::Vector3d::Zero(), bo = bc;
    for (const auto& b : actions[i].added_blocks)
      if (b.kind == FactorKind::KinematicBridge) {
        const auto bounds = BridgeFactory()
                                .uncertainty(tx, cfg.bridge.generic)
                                .deterministic_bound;
        bc += BridgeFactory().propagateBoxMargin(
            window.protected_state_map,
            c.covarianceTimes(b.jacobian_whitened.transpose()), bounds);
        bo += BridgeFactory().propagateBoxMargin(
            window.protected_state_map,
            oracle.covarianceTimes(b.jacobian_whitened.transpose()), bounds);
      }
    EXPECT_LE((bc - bo).norm(), 1e-7);
    // An independent residual-row fault fixture exercises the downstream PL
    // formulas for every real action without changing production hypotheses.
    FaultHypothesisV2 fault;
    fault.id = HypothesisId(1);
    fault.A = Eigen::MatrixXd::Zero(c.rows, 1);
    fault.A(c.rows - 1, 0) = 1;
    fault.p_md_allocation = 1e-3;
    fault.hmi_allocation = 1e-6;
    fault.prior_probability_bound = 1e-3;
    std::vector<FaultHypothesisV2> fc{fault}, fo{fault};
    const auto pl =
        ProtectionLevelV2().compute(window, c, dc, &fc, RiskBudgetV2{}, bc);
    const auto op = ProtectionLevelV2().compute(window, oracle, od, &fo,
                                                RiskBudgetV2{}, bo);
    EXPECT_EQ(pl.model_valid, op.model_valid)
        << i << ": " << pl.reason << "/" << op.reason;
    if (pl.model_valid) {
      EXPECT_LE((pl.pl_xyz_m - op.pl_xyz_m).norm() /
                    std::max(1e-15, op.pl_xyz_m.norm()),
                1e-7);
    }
  }
  EXPECT_GT(valid, 0u);
  EXPECT_GT(bridges, 0u);
  EXPECT_GT(history, 0u);
  EXPECT_EQ(estimator.backendUpdateCount(), updates);
  estimator.discardEpoch(std::move(tx), {FdeStatus::ModelInvalid,
                                         "oracle comparison done", false});
  EXPECT_EQ(estimator.backendUpdateCount(), updates);
}

TEST(GateDNumerics, RankAndConditionThresholdsBothSidesKeepExactDefinitions) {
  using namespace uwb_imu_pl;
  for (bool rank_gate : {false, true})
    for (double side : {1 - 1e-7, 1 + 1e-7}) {
      LinearizedIntegrityWindow w;
      w.version = {1, 2, 3, 4};
      Eigen::MatrixXd h = Eigen::MatrixXd::Zero(3, 2);
      h(0, 0) = 1;
      h(1, 1) = (rank_gate ? 1e-10 : 1e-4) * side;
      w.blocks = {block(1, h, Eigen::Vector3d::Zero(), w.version),
                  block(2, Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero(),
                        w.version)};
      RankUpdateConfig config;
      config.materialize_dense_oracle_fields = false;
      config.rank_tolerance = rank_gate ? 1e-10 : 1e-12;
      config.max_condition_number = rank_gate ? 1e12 : 1e4;
      finalizeIntegrityWindow(&w, config.rank_tolerance,
                              config.max_condition_number);
      ExclusionAction a;
      a.groups_to_remove = {FactorGroupId(2)};
      const RankUpdateEvaluator evaluator(config);
      const auto c = evaluator.evaluate(evaluator.factorizeOnce(w), a);
      EXPECT_EQ(c.valid, side > 1) << c.reason;
      const auto oracle = DenseCandidateOracle(config).evaluate(w, a);
      EXPECT_EQ(c.valid, oracle.valid);
      if (c.valid) {
        const auto covariance = c.covarianceTimes(Eigen::Matrix2d::Identity());
        EXPECT_LE(
            (covariance - oracle.covariance).norm() / oracle.covariance.norm(),
            1e-6);
        EXPECT_LE((c.state_increment - oracle.state_increment).norm(), 1e-6);
        EXPECT_NEAR(c.statistic, oracle.statistic, 1e-6);
      }
      EXPECT_TRUE(c.diagnostics.near_gate);
      if (side < 1) {
        EXPECT_EQ(c.reason, rank_gate ? "candidate rank loss"
                                      : "candidate condition gate failed");
      }
    }
}

TEST(GateDSelection, DenseAndOperatorPathsSelectSameSuccessfulExclusion) {
  using namespace uwb_imu_pl;
  auto w = syntheticWindow();
  w.protected_state_map.row(2) = w.protected_state_map.row(0);
  finalizeIntegrityWindow(&w, 1e-10, 1e10);
  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  ExclusionAction exclude;
  exclude.id = ExclusionActionId(2);
  exclude.action_model_id = "UWB_EXCLUSION";
  exclude.groups_to_remove = {FactorGroupId(3)};
  exclude.covered_units = {FaultUnitId(11)};
  exclude.exclusion_cardinality = 1;
  RankUpdateConfig cfg;
  cfg.materialize_dense_oracle_fields = false;
  const RankUpdateEvaluator evaluator(cfg);
  const auto base = evaluator.factorizeOnce(w, {keep, exclude});
  std::vector<CandidateEvaluation> fast, dense;
  for (const auto& action : {keep, exclude}) {
    fast.push_back(evaluator.evaluate(base, action));
    dense.push_back(DenseCandidateOracle(cfg).evaluate(w, action));
    for (auto* c : {&fast.back(), &dense.back()}) {
      const auto detector =
          JointWindowDetector().evaluateCandidate(
              w, *c, DetectorRiskContext{});
      ASSERT_TRUE(c->valid);
      ASSERT_TRUE(detector.passed);
      FaultHypothesisV2 remaining;
      remaining.id = HypothesisId(9);
      remaining.A = Eigen::MatrixXd::Zero(c->rows, 1);
      remaining.A(3, 0) = 1;
      remaining.p_md_allocation = 1e-3;
      remaining.hmi_allocation = 1e-6;
      remaining.prior_probability_bound = 1e-3;
      std::vector<FaultHypothesisV2> faults{remaining};
      const auto pl =
          ProtectionLevelV2().compute(w, *c, detector, &faults, RiskBudgetV2{});
      ASSERT_TRUE(pl.model_valid) << pl.reason;
      c->post_detector_passed = true;
      c->hpl_m = pl.hpl_m;
      c->vpl_m = pl.vpl_m;
    }
  }
  FaultHypothesisV2 fault;
  fault.id = HypothesisId(1);
  fault.units = exclude.covered_units;
  fault.affected_groups = exclude.groups_to_remove;
  FaultModeEvidence evidence;
  evidence.hypothesis = fault.id;
  evidence.plausible = true;
  // C3: comparability identity + raw profile evidence of the fixture.
  evidence.unit_kind = FaultUnitKind::UwbRangeMeters;
  evidence.parameter_dimension = 1;
  evidence.profile_j = 1.0;
  evidence.profile_valid = true;
  DetectorResultV2 alarm;
  alarm.numerically_valid = true;
  alarm.passed = false;
  FdeRiskDecisionV1 f_risk, d_risk;
  FdeDecisionContextV1 f_context, d_context;
  f_context.risk_result = &f_risk;
  d_context.risk_result = &d_risk;
  const auto f = FdeManager().decide(
      alarm, {fault}, {evidence}, &fast, {}, RiskBudgetV2{}, &f_context);
  const auto d = FdeManager().decide(
      alarm, {fault}, {evidence}, &dense, {}, RiskBudgetV2{}, &d_context);
  ASSERT_FALSE(f.commit_allowed);
  ASSERT_FALSE(d.commit_allowed);
  EXPECT_FALSE(f.selected_action.has_value());
  EXPECT_FALSE(d.selected_action.has_value());
  EXPECT_EQ(f.status, FdeStatus::RiskBudgetInvalid);
  EXPECT_FALSE(f_risk.complete_bound_closes);
  EXPECT_EQ(f_risk.complete_bound_closes, d_risk.complete_bound_closes);
  EXPECT_EQ(f.status, d.status);
}

TEST(GateDDiagnostics, PreparationExceptionKeepsAttemptTimingAndZeroUpdates) {
  using namespace uwb_imu_pl;
  const auto cfg = researchConfig();
  IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, cfg.realtime.prior_sigmas);
  addImu(&estimator, cfg.imu.gravity_mps2);
  RealtimeIntegrityPipeline pipeline(
      &estimator, IntegrityMonitor(cfg.risk, cfg.snapshot.rank_tolerance,
                                   cfg.snapshot.max_condition_number));
  auto bad = batch(cfg, 10000000);
  bad.measurements.front().range_m = std::numeric_limits<double>::quiet_NaN();
  const auto before = estimator.backendUpdateCount();
  EXPECT_ANY_THROW(pipeline.processUwbBatch(bad));
  const auto& output = pipeline.lastAttemptOutput();
  EXPECT_EQ(output.diagnostics.input_attempt_id, 1u);
  EXPECT_EQ(output.diagnostics.input_timestamp, bad.timestamp);
  EXPECT_EQ(output.diagnostics.status, "EXCEPTION");
  EXPECT_EQ(estimator.backendUpdateCount(), before);
  const auto prepare =
      std::find_if(output.stage_timings.begin(), output.stage_timings.end(),
                   [](const auto& s) { return s.stage == "prepare"; });
  ASSERT_NE(prepare, output.stage_timings.end());
  EXPECT_EQ(prepare->status, "EXCEPTION");
  EXPECT_TRUE(std::isfinite(prepare->wall_ms));
  EXPECT_FALSE(prepare->success);
  const auto core_total_count = static_cast<std::size_t>(std::count_if(
      output.stage_timings.begin(), output.stage_timings.end(),
      [](const auto& timing) { return timing.stage == "core_total"; }));
  EXPECT_EQ(core_total_count, 1u);
  const auto core_total =
      std::find_if(output.stage_timings.begin(), output.stage_timings.end(),
                   [](const auto& timing) {
                     return timing.stage == "core_total";
                   });
  ASSERT_NE(core_total, output.stage_timings.end());
  EXPECT_TRUE(std::isfinite(core_total->wall_ms));
  EXPECT_FALSE(core_total->success);
  EXPECT_EQ(core_total->status, "EXCEPTION");
}

TEST(GateDNumericalCertificate, ClearCandidateAvoidsFinalJacobianSvd) {
  using namespace uwb_imu_pl;
  const auto window = syntheticWindow();
  ExclusionAction action;
  action.id = ExclusionActionId(77);
  action.groups_to_remove = {FactorGroupId(3)};
  RankUpdateConfig config;
  config.materialize_dense_oracle_fields = false;
  config.max_linearization_step_norm = 10.0;
  const RankUpdateEvaluator evaluator(config);
  const auto candidate =
      evaluator.evaluate(evaluator.factorizeOnce(window, {action}), action);
  const auto oracle = DenseCandidateOracle(config).evaluate(window, action);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  EXPECT_TRUE(candidate.diagnostics.certificate_passed);
  EXPECT_FALSE(candidate.exact_slow_path);
  EXPECT_EQ(candidate.diagnostics.condition_value_kind, "CERTIFIED_BOUNDS");
  EXPECT_TRUE(std::isnan(candidate.condition_number));
  EXPECT_LT(candidate.diagnostics.condition_upper_bound,
            config.max_condition_number);
  EXPECT_TRUE(candidate.state_increment.isApprox(oracle.state_increment, 1e-9));
  EXPECT_NEAR(candidate.statistic, oracle.statistic, 1e-9);
}

TEST(GateDProtectionLevel, SharedModesUseOneCombinedCovarianceSolve) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  window.protected_state_map.row(2) = window.protected_state_map.row(0);
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  RankUpdateConfig config;
  config.materialize_dense_oracle_fields = false;
  config.max_linearization_step_norm = 10.0;
  const RankUpdateEvaluator evaluator(config);
  const auto base = evaluator.factorizeOnce(window);
  auto candidate = evaluator.evaluate(base, ExclusionAction{});
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  const auto detector =
      JointWindowDetector().evaluateCandidate(
          window, candidate, DetectorRiskContext{});
  ASSERT_TRUE(detector.passed);
  Eigen::MatrixXd mode = Eigen::MatrixXd::Zero(candidate.rows, 1);
  mode(4, 0) = 1.0;
  FaultHypothesisV2 first;
  first.id = HypothesisId(101);
  first.modes = {FaultModeId(501)};
  first.A = mode;
  first.p_md_allocation = 1e-3;
  first.hmi_allocation = 1e-6;
  first.prior_probability_bound = 1e-3;
  auto second = first;
  second.id = HypothesisId(102);
  std::vector<FaultHypothesisV2> legacy{first, second};
  std::vector<FaultHypothesisV2> optimized{first, second};
  const auto reference = ProtectionLevelV2().compute(
      window, candidate, detector, &legacy, RiskBudgetV2{});
  ProtectionLevelSharedContext shared;
  shared.mode_maps.emplace(501, mode);
  const auto result = ProtectionLevelV2().computeShared(
      window, &candidate, detector, &optimized, shared, RiskBudgetV2{});
  ASSERT_TRUE(reference.model_valid) << reference.reason;
  ASSERT_TRUE(result.model_valid) << result.reason;
  EXPECT_TRUE(result.pl_xyz_m.isApprox(reference.pl_xyz_m, 1e-9));
  EXPECT_EQ(candidate.diagnostics.covariance_solve_count, 1u);
  EXPECT_TRUE(optimized[0].monitorability.protected_slopes.isApprox(
      legacy[0].monitorability.protected_slopes, 1e-9));
}

TEST(GateDProtectionLevel, ProtectedCovarianceMatchesItsActualDynamicFactor) {
  using namespace uwb_imu_pl;
  for (const int columns : {6, 30, 150}) {
    const int rows = columns > 30 ? columns + 42 : 72;
    LinearizedIntegrityWindow window;
    window.id = WindowId(710 + columns);
    window.version = {1, 2, 3, 4};
    Eigen::MatrixXd h(rows, columns);
    for (int row = 0; row < h.rows(); ++row) {
      for (int col = 0; col < h.cols(); ++col) {
        h(row, col) = std::sin(0.13 * (row + 1) * (col + 1)) +
            (row == col ? 3.0 : 0.0);
      }
    }
    window.blocks.push_back(block(1, h, Eigen::VectorXd::Zero(rows), window.version));
    window.protected_state_map = Eigen::MatrixXd::Zero(3, columns);
    window.protected_state_map.leftCols<3>().setIdentity();
    finalizeIntegrityWindow(&window, 1e-10, 1e10);
    const auto owner = freezeIntegrityWindowCopy(window);
    const auto admission = admitFrozenIntegrityWindow(owner);
    ASSERT_TRUE(admission) << admission.reason;
    RankUpdateConfig config{1e-10, 1e10, 10.0};
    RankUpdateEvaluator evaluator(config);
    const auto base = evaluator.factorizeOnce(admission);
    auto candidate = evaluator.evaluate(admission, base, ExclusionAction{});
    ASSERT_TRUE(candidate.valid) << candidate.reason;
    const auto detector = JointWindowDetector().evaluateCandidate(
        admission, candidate, DetectorRiskContext{});
    ASSERT_TRUE(detector.passed) << detector.reason;
    ProtectionLevelSharedContext context;
    context.mode_maps[11] = Eigen::MatrixXd::Zero(rows, 1);
    context.mode_maps[11](rows - 2, 0) = 1.0;
    FaultHypothesisV2 hypothesis;
    hypothesis.id = HypothesisId(11);
    hypothesis.modes = {FaultModeId(11)};
    hypothesis.prior_probability_bound = 1e-4;
    hypothesis.p_md_allocation = 1e-3;
    hypothesis.hmi_allocation = 1e-6;
    std::vector<FaultHypothesisV2> hypotheses{hypothesis};
    ProtectionLevelV2ProofV1 proof;
    AttemptProofArena arena;
    const auto result = ProtectionLevelV2().computeShared(admission, &candidate,
        detector, &hypotheses, context, RiskBudgetV2{}, &proof, &arena);
    ASSERT_TRUE(result.model_valid) << result.reason;
    const Eigen::MatrixXd actual_gram = proof.protected_covariance_factor.transpose() *
        proof.protected_covariance_factor;
    const Eigen::MatrixXd transposed_factor = proof.protected_covariance_factor.transpose();
    const Eigen::Matrix3d legacy_fixed_gram = transposed_factor * transposed_factor.transpose();
    // Preserve evidence of the platform's old fixed/dynamic reduction mismatch
    // without requiring every Eigen/compiler combination to round differently.
    RecordProperty("legacy_fixed_gram_matches_actual_factor_n" + std::to_string(columns),
        static_cast<int>((legacy_fixed_gram.array() == actual_gram.array()).all()));
    EXPECT_TRUE((actual_gram.array() == proof.protected_covariance.array()).all());
    EXPECT_TRUE(certifyFactorGram(proof.protected_covariance_factor,
        proof.protected_covariance, proof.covariance_rank_tolerance,
        proof.candidate_proof_identity).valid);
    EXPECT_TRUE(validateProtectionLevelV2Proof(candidate, detector, hypotheses,
        result, arena, nullptr));
  }
}

TEST(GateDWorkerPool, StaticSchedulingExceptionBarrierAndScratchReuse) {
  using namespace uwb_imu_pl;
  CandidateWorkerPool pool(4);
  std::vector<std::size_t> owners(20, 99);
  pool.run(
      owners.size(), 4,
      [&](std::size_t index, std::size_t worker, RankUpdateScratch& scratch) {
        owners[index] = worker;
        scratch.update_signs = Eigen::Vector2d::Ones();
      });
  for (std::size_t i = 0; i < owners.size(); ++i) EXPECT_EQ(owners[i], i % 4);
  std::atomic<int> reused{0};
  pool.run(4, 4, [&](std::size_t, std::size_t, RankUpdateScratch& scratch) {
    if (scratch.update_signs.size() == 2) ++reused;
  });
  EXPECT_EQ(reused.load(), 4);
  EXPECT_THROW(pool.run(12, 4,
                        [](std::size_t index, std::size_t, RankUpdateScratch&) {
                          if (index == 3)
                            throw std::runtime_error("worker failure");
                        }),
               std::runtime_error);
  std::atomic<int> completed{0};
  pool.run(8, 1, [&](std::size_t, std::size_t worker, RankUpdateScratch&) {
    EXPECT_EQ(worker, 0u);
    ++completed;
  });
  EXPECT_EQ(completed.load(), 8);
}

TEST(P105FlatConcurrency,
     DynamicSlotsBarrierNestedRejectionAndScratchBoundAreDeterministic) {
  using namespace uwb_imu_pl;
  CandidateWorkerPool pool(4);
  auto execute = [&](std::size_t workers) {
    std::vector<std::uint64_t> slots(64, 0);
    pool.runFlat(slots.size(), workers, 1u << 20,
        [&](std::size_t index, std::size_t, RankUpdateScratch&) {
          if (index % 7 == 0) std::this_thread::yield();
          slots[index] = 0x9e3779b97f4a7c15ULL ^
              (static_cast<std::uint64_t>(index) * 0x100000001b3ULL);
        });
    return slots;
  };
  const auto one = execute(1);
  EXPECT_EQ(execute(2), one);
  EXPECT_EQ(execute(4), one);

  std::vector<std::atomic<int>> terminal(24);
  for (auto& value : terminal) value = 0;
  try {
    pool.runFlat(terminal.size(), 4, 1u << 20,
        [&](std::size_t index, std::size_t, RankUpdateScratch&) {
          ++terminal[index];
          if (index == 17) throw std::runtime_error("flat-17");
          if (index == 3) throw std::runtime_error("flat-3");
        });
    FAIL() << "flat worker failure must be rethrown after the barrier";
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(), "flat-3");
  }
  for (const auto& value : terminal) EXPECT_EQ(value.load(), 1);

  std::atomic<int> nested_attempts{0};
  EXPECT_THROW(
      pool.runFlat(8, 4, 1u << 20,
          [&](std::size_t index, std::size_t, RankUpdateScratch&) {
            if (index == 2) {
              ++nested_attempts;
              pool.runFlat(1, 1, 1u << 20,
                  [](std::size_t, std::size_t, RankUpdateScratch&) {});
            }
          }),
      std::logic_error);
  EXPECT_EQ(nested_attempts.load(), 1);

  std::array<std::atomic<bool>, 4> oversize_workers;
  for (auto& seen : oversize_workers) seen = false;
  std::vector<std::atomic<int>> oversize_terminal(24);
  for (auto& value : oversize_terminal) value = 0;
  std::atomic<std::size_t> oversize_started{0};
  try {
    pool.runFlat(oversize_terminal.size(), 4, 1024,
        [&](std::size_t index, std::size_t worker,
            RankUpdateScratch& scratch) {
          ++oversize_terminal[index];
          if (index < 4) {
            oversize_workers[worker] = true;
            scratch.update_columns = Eigen::MatrixXd::Zero(128, 128);
            ++oversize_started;
            while (oversize_started.load() < 4) std::this_thread::yield();
          }
        });
    FAIL() << "scratch limit violation must fail closed after the barrier";
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(),
                 "candidate worker scratch limit exceeded at task 0");
  }
  EXPECT_EQ(static_cast<std::size_t>(std::count_if(
                oversize_workers.begin(), oversize_workers.end(),
                [](const auto& seen) { return seen.load(); })),
            4u);
  for (const auto& value : oversize_terminal) EXPECT_EQ(value.load(), 1);
  EXPECT_GE(pool.scratchHighWaterBytes(), 128u * 128u * sizeof(double));
  EXPECT_EQ(pool.scratchBytes(), 0u)
      << "every oversized worker scratch must be released at the barrier";

  std::vector<std::atomic<int>> competing_terminal(24);
  for (auto& value : competing_terminal) value = 0;
  try {
    pool.runFlat(competing_terminal.size(), 4, 1024,
        [&](std::size_t index, std::size_t, RankUpdateScratch& scratch) {
          ++competing_terminal[index];
          if (index == 7 || index == 15) {
            scratch.update_columns = Eigen::MatrixXd::Zero(128, 128);
          }
          if (index == 18) throw std::runtime_error("ordinary-18");
          if (index == 3) throw std::runtime_error("ordinary-3");
        });
    FAIL() << "lowest task exception must win over scratch violations";
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(), "ordinary-3");
  }
  for (const auto& value : competing_terminal) EXPECT_EQ(value.load(), 1);
  EXPECT_EQ(pool.scratchBytes(), 0u);

  std::vector<std::uint64_t> reusable_slots(32, 0);
  EXPECT_NO_THROW(pool.runFlat(
      reusable_slots.size(), 4, 1u << 20,
      [&](std::size_t index, std::size_t, RankUpdateScratch&) {
        reusable_slots[index] = index + 1;
      }));
  for (std::size_t index = 0; index < reusable_slots.size(); ++index) {
    EXPECT_EQ(reusable_slots[index], index + 1);
  }
}

TEST(P105FlatConcurrency,
     StatisticalCacheSameKeyIsSingleFlightAndDifferentKeysAreComplete) {
  using namespace uwb_imu_pl;
  CandidateWorkerPool pool(4);

  StatisticalBoundsCache::clear();
  const auto same_before = StatisticalBoundsCache::stats();
  std::vector<double> same_values(64, 0.0);
  StatisticalBoundKey same_key;
  same_key.detector_id = 0x105;
  same_key.dof = 4;
  same_key.contract_version = 0x505;
  pool.runFlat(same_values.size(), 4, 1u << 20,
      [&](std::size_t index, std::size_t, RankUpdateScratch&) {
        same_values[index] = StatisticalBoundsCache::
            noncentralityBoundaryVerified(4, 21.125, 1e-3, same_key).value;
      });
  const auto same_after = StatisticalBoundsCache::stats();
  ASSERT_TRUE(std::isfinite(same_values.front()));
  for (const double value : same_values) {
    EXPECT_DOUBLE_EQ(value, same_values.front());
  }
  EXPECT_EQ(same_after.misses - same_before.misses, 1u)
      << "one exact cache key must have one actual solve";
  EXPECT_EQ(same_after.hits - same_before.hits, same_values.size() - 1);
  EXPECT_EQ(same_after.entries, 1u);

  StatisticalBoundsCache::clear();
  const auto distinct_before = StatisticalBoundsCache::stats();
  constexpr std::size_t kDistinctKeys = 24;
  std::vector<double> distinct_values(kDistinctKeys, 0.0);
  pool.runFlat(kDistinctKeys, 4, 1u << 20,
      [&](std::size_t index, std::size_t, RankUpdateScratch&) {
        StatisticalBoundKey key;
        key.detector_id = 0x205;
        key.dof = 4;
        key.contract_version = 0x605;
        key.envelope_fingerprint = index + 1;
        distinct_values[index] = StatisticalBoundsCache::
            noncentralityBoundaryVerified(
                4, 18.0 + 0.125 * static_cast<double>(index), 1e-3, key)
                .value;
      });
  const auto distinct_after = StatisticalBoundsCache::stats();
  for (const double value : distinct_values) EXPECT_TRUE(std::isfinite(value));
  EXPECT_EQ(distinct_after.misses - distinct_before.misses, kDistinctKeys);
  EXPECT_EQ(distinct_after.hits - distinct_before.hits, 0u);
  EXPECT_EQ(distinct_after.entries, kDistinctKeys);
}

TEST(P105FlatConcurrency,
     StatisticalCacheFailureCountersAreRaceFreeAtConcurrentBoundaries) {
  using namespace uwb_imu_pl;
  CandidateWorkerPool pool(4);
  StatisticalBoundsCache::clear();
  const auto before = StatisticalBoundsCache::stats();
  constexpr std::size_t kCallsPerBoundary = 16;
  pool.runFlat(4 * kCallsPerBoundary, 4, 1u << 20,
      [&](std::size_t index, std::size_t, RankUpdateScratch&) {
        const std::size_t boundary = index / kCallsPerBoundary;
        StatisticalBoundKey key;
        key.dof = boundary == 3 ? 7 : 0;
        NoncentralityBoundaryResult result;
        if (boundary == 0) {
          result = StatisticalBoundsCache::noncentralityBoundaryVerified(
              0, 21.0, 1e-3, key);
        } else if (boundary == 1) {
          result = StatisticalBoundsCache::noncentralityBoundaryVerified(
              4, 0.0, 1e-3, key);
        } else if (boundary == 2) {
          result = StatisticalBoundsCache::noncentralityBoundaryVerified(
              4, 21.0, 1.0, key);
        } else {
          result = StatisticalBoundsCache::noncentralityBoundaryVerified(
              4, 21.0, 1e-3, key);
        }
        EXPECT_FALSE(result.valid);
      });
  const auto after = StatisticalBoundsCache::stats();
  EXPECT_EQ(after.invalid_inputs - before.invalid_inputs,
            3 * kCallsPerBoundary);
  EXPECT_EQ(after.policy_mismatches - before.policy_mismatches,
            kCallsPerBoundary);
  EXPECT_EQ(after.non_converged - before.non_converged, 0u);
  EXPECT_EQ(after.hits - before.hits, 0u);
  EXPECT_EQ(after.misses - before.misses, 0u);
  EXPECT_EQ(after.entries, 0u);
}

TEST(P105FlatConcurrency,
     StatisticalCacheAlgorithmWorkIsExactForOneTwoFourAndRepeatedFourWorkers) {
  using namespace uwb_imu_pl;
  CandidateWorkerPool pool(4);
  struct Outcome {
    std::vector<double> values;
    std::uint64_t hits = 0;
    std::uint64_t misses = 0;
    std::uint64_t entries = 0;
  };
  auto execute = [&](std::size_t workers) {
    StatisticalBoundsCache::clear();
    const auto before = StatisticalBoundsCache::stats();
    Outcome outcome;
    outcome.values.resize(96);
    pool.runFlat(outcome.values.size(), workers, 1u << 20,
        [&](std::size_t index, std::size_t, RankUpdateScratch&) {
          const std::size_t canonical_key = index % 12;
          StatisticalBoundKey key;
          key.detector_id = 0x305;
          key.dof = 4;
          key.contract_version = 0x705;
          key.envelope_kind = canonical_key % 3;
          key.envelope_fingerprint = canonical_key + 1;
          outcome.values[index] = StatisticalBoundsCache::
              noncentralityBoundaryVerified(
                  4, 17.0 + 0.25 * static_cast<double>(canonical_key),
                  1e-3, key).value;
        });
    const auto after = StatisticalBoundsCache::stats();
    outcome.hits = after.hits - before.hits;
    outcome.misses = after.misses - before.misses;
    outcome.entries = after.entries;
    return outcome;
  };
  auto expect_same = [](const Outcome& actual, const Outcome& expected) {
    EXPECT_EQ(actual.values, expected.values);
    EXPECT_EQ(actual.hits, expected.hits);
    EXPECT_EQ(actual.misses, expected.misses);
    EXPECT_EQ(actual.entries, expected.entries);
  };

  const Outcome one = execute(1);
  EXPECT_EQ(one.misses, 12u);
  EXPECT_EQ(one.hits, 84u);
  EXPECT_EQ(one.entries, 12u);
  expect_same(execute(2), one);
  const Outcome four = execute(4);
  expect_same(four, one);
  for (int repeat = 0; repeat < 10; ++repeat) {
    expect_same(execute(4), four);
  }
}

TEST(GateDStatistics, ExactBitKeyProducesCacheHitWithoutQuantization) {
  using namespace uwb_imu_pl;
  const auto before = StatisticalBoundsCache::stats();
  const double first =
      StatisticalBoundsCache::chiSquaredThreshold(123, 0.123456789);
  const auto middle = StatisticalBoundsCache::stats();
  const double second =
      StatisticalBoundsCache::chiSquaredThreshold(123, 0.123456789);
  const auto after = StatisticalBoundsCache::stats();
  EXPECT_DOUBLE_EQ(first, second);
  EXPECT_EQ(middle.misses, before.misses + 1);
  EXPECT_EQ(after.hits, middle.hits + 1);
}

TEST(GateDHistoryCache, FrozenFactorContentHitsAndMutationMisses) {
  using namespace uwb_imu_pl;
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  auto transaction = estimator.prepareEpoch(batch(config, 10000000));
  (void)estimator.buildIntegrityWindow(transaction, IntegrityWindowRequest{});
  const auto before = estimator.cacheAudit();
  const auto first =
      estimator.buildPendingFactorBlock(transaction, transaction.imu_group.id);
  const auto hit = estimator.cacheAudit();
  EXPECT_GT(hit.factor_block_hits, before.factor_block_hits);
  auto changed = transaction;
  changed.imu_group.model_id += ":changed";
  const auto second =
      estimator.buildPendingFactorBlock(changed, changed.imu_group.id);
  const auto miss = estimator.cacheAudit();
  EXPECT_GT(miss.factor_block_misses, hit.factor_block_misses);
  EXPECT_TRUE(first.jacobian_whitened.isApprox(second.jacobian_whitened, 0));
  estimator.discardEpoch(
      std::move(transaction),
      {FdeStatus::ModelInvalid, "cache test complete", false});
}

TEST(IntegrityV2Numerics, FrozenWindowSharesOneBaseSvdAndLlt) {
  uwb_imu_pl::NumericalWorkCounters::reset();
  auto window = syntheticWindow();
  uwb_imu_pl::DetectorRiskContext detector_risk;
  detector_risk.max_condition_number = 1e10;
  const auto detector =
      uwb_imu_pl::JointWindowDetector().evaluate(window, detector_risk);
  std::vector<uwb_imu_pl::FaultHypothesisV2> hypotheses;
  (void)uwb_imu_pl::HypothesisEvidenceEvaluator().evaluateAll(
      window, &hypotheses, detector.squared_threshold);
  const auto base = uwb_imu_pl::RankUpdateEvaluator().factorizeOnce(window);
  ASSERT_TRUE(base.valid) << base.reason;
  uwb_imu_pl::ExclusionAction keep;
  keep.id = uwb_imu_pl::ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  const auto unchanged = uwb_imu_pl::RankUpdateEvaluator().evaluate(base, keep);
  ASSERT_TRUE(unchanged.valid) << unchanged.reason;
  EXPECT_EQ(unchanged.diagnostics.numerical_path, "KEEP_BASE_CANONICAL");
  EXPECT_NEAR(unchanged.statistic, window.numerics->statistic, 1e-12);
  const auto counts = uwb_imu_pl::NumericalWorkCounters::snapshot();
  EXPECT_EQ(counts.base_svd, 1u);
  EXPECT_EQ(counts.base_llt, 1u);
  EXPECT_EQ(counts.base_state_solves, 2u);
  EXPECT_EQ(counts.llt_state_solve_calls, 1u);
  EXPECT_EQ(counts.svd_state_solve_calls, 1u);
  EXPECT_EQ(counts.detector_reference_qr, 0u);
  EXPECT_EQ(counts.candidate_reference_svd, 0u);
}

TEST(IntegrityV2Numerics, RejectsReuseAfterAnyFrozenIdentityMutation) {
  using namespace uwb_imu_pl;
  const RankUpdateEvaluator evaluator;
  for (int mutation = 0; mutation < 8; ++mutation) {
    auto window = syntheticWindow();
    const auto base = evaluator.factorizeOnce(window);
    ASSERT_TRUE(base.valid) << mutation << ": " << base.reason;
    switch (mutation) {
      case 0:
        ++window.version.graph_version;
        break;
      case 1:
        ++window.version.ordering_version;
        break;
      case 2:
        ++window.version.noise_model_version;
        break;
      case 3:
        ++window.version.linpoint_version;
        break;
      case 4:
        window.H(0, 0) = std::nextafter(window.H(0, 0), 10.0);
        break;
      case 5:
        window.protected_state_map(0, 0) = 2.0;
        break;
      case 6:
        window.blocks.front().covariance(0, 0) = 2.0;
        break;
      case 7:
        window.state_layout.push_back(
            {9, {gtsam::Symbol('x', 9)}, 0, 1, false});
        break;
    }
    const auto candidate = evaluator.evaluate(base, ExclusionAction{});
    EXPECT_FALSE(candidate.valid) << mutation;
    if (mutation < 4) {
      EXPECT_EQ(candidate.reason, "candidate frozen block/version is invalid");
    } else {
      EXPECT_NE(
          candidate.diagnostics.fallback_reason.find("stale frozen numerics"),
          std::string::npos)
          << mutation << ": " << candidate.reason;
    }
  }
}

TEST(IntegrityV2Risk, EqualAllocationNeverOvershootsAndRealExcessFails) {
  uwb_imu_pl::RiskBudgetV2 risk;
  const std::size_t count = 1386;
  std::vector<uwb_imu_pl::FaultHypothesisV2> hypotheses(count);
  const double available =
      uwb_imu_pl::conservativeRemainingHypothesisRisk(risk);
  const double allocation =
      uwb_imu_pl::conservativeEqualRiskAllocation(available, count);
  for (auto& hypothesis : hypotheses) hypothesis.hmi_allocation = allocation;
  const auto closed = uwb_imu_pl::auditRiskBudget(risk, hypotheses);
  EXPECT_TRUE(closed.valid);
  EXPECT_GE(closed.margin, 0.0);
  hypotheses.front().hmi_allocation += 1e-12;
  const auto exceeded = uwb_imu_pl::auditRiskBudget(risk, hypotheses);
  EXPECT_FALSE(exceeded.valid);
  EXPECT_LT(exceeded.margin, 0.0);
}

TEST(IntegrityV2ImuOracle,
     AnalyticProductionHasZeroReintegrationAcrossMotions) {
  using namespace uwb_imu_pl;
  struct Motion {
    double duration_s;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_rate;
    Eigen::Vector3d accel_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Quaterniond orientation;
  };
  const std::vector<Motion> motions{
      {0.02, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
      {0.05, Eigen::Vector3d(0.4, -0.2, 0.1), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
      {0.10, Eigen::Vector3d(0.2, 0.1, 0.0), Eigen::Vector3d(0.7, -0.3, 0.2),
       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
      {0.05, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
       Eigen::Vector3d(0.0, 0.0, 0.4), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
      {0.075, Eigen::Vector3d(0.1, 0.2, -0.1), Eigen::Vector3d(0.2, 0.1, 0.0),
       Eigen::Vector3d(0.2, -0.15, 0.3), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(),
       Eigen::Quaterniond(
           Eigen::AngleAxisd(0.35, Eigen::Vector3d(1, 2, 3).normalized()))},
      {0.05, Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::Vector3d::Zero(),
       Eigen::Vector3d(0.1, -0.05, 0.2), Eigen::Vector3d(0.03, -0.02, 0.01),
       Eigen::Vector3d(0.002, -0.003, 0.001), Eigen::Quaterniond::Identity()},
      // Long-pending oracle-cost observation: production remains zero, while
      // the explicit oracle is still exactly six central differences.
      {0.50, Eigen::Vector3d(0.2, 0.2, 0.0), Eigen::Vector3d(0.1, 0.0, 0.0),
       Eigen::Vector3d(0.1, 0.2, -0.1), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
  };
  for (const auto& motion : motions) {
    auto config = researchConfig();
    IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
    NavigationState initial;
    initial.timestamp = TimestampNs(0);
    initial.position_world_m = {0, 0, 1};
    initial.velocity_world_mps = motion.velocity;
    initial.accel_bias_mps2 = motion.accel_bias;
    initial.gyro_bias_radps = motion.gyro_bias;
    initial.q_world_body = motion.orientation;
    estimator.initialize(initial, config.realtime.prior_sigmas);
    const int samples =
        static_cast<int>(std::llround(motion.duration_s / 0.005));
    for (int sample = 0; sample <= samples; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(sample + 1);
      imu.timestamp = TimestampNs(sample * 5000000LL);
      imu.specific_force_mps2 = motion.acceleration + motion.accel_bias +
                                Eigen::Vector3d(0, 0, config.imu.gravity_mps2);
      imu.angular_velocity_radps = motion.angular_rate + motion.gyro_bias;
      estimator.ingestImu(imu);
    }
    auto input =
        batch(config,
              static_cast<std::int64_t>(std::llround(motion.duration_s * 1e9)));
    auto transaction = estimator.prepareEpoch(input);
    const auto imu_block = estimator.buildPendingFactorBlock(
        transaction, transaction.imu_group.id);
    NumericalWorkCounters::reset();
    const auto analytic =
        ImuFaultSubspaceBuilder().buildAnalytic(transaction, imu_block);
    EXPECT_TRUE(analytic.analytic_input_valid);
    EXPECT_TRUE(analytic.analytic_computation_valid);
    EXPECT_FALSE(analytic.oracle_executed);
    EXPECT_EQ(NumericalWorkCounters::snapshot().imu_oracle_reintegrations, 0u);
    const auto verified =
        ImuFaultSubspaceBuilder().verifyFiniteDifferenceOracle(transaction,
                                                               imu_block);
    EXPECT_TRUE(verified.oracle_verified)
        << motion.duration_s << ": " << verified.oracle_relative_error;
    EXPECT_EQ(verified.oracle_reintegrations, 12u);
    EXPECT_EQ(NumericalWorkCounters::snapshot().imu_oracle_reintegrations, 12u);
    estimator.discardEpoch(
        std::move(transaction),
        {FdeStatus::ModelInvalid, "oracle development test", false});
  }
}

TEST(IntegrityV2ImuOracle,
     FiniteDifferenceSweepMatchesAnalyticAcrossStepSizes) {
  using namespace uwb_imu_pl;
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d(0.1, 0.2, 0.3));
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  initial.velocity_world_mps = {0.2, 0.1, 0.0};
  initial.q_world_body = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.2, Eigen::Vector3d(1, 2, 3).normalized()));
  estimator.initialize(initial, config.realtime.prior_sigmas);
  for (int sample = 0; sample <= 10; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(sample + 1);
    imu.timestamp = TimestampNs(sample * 5000000LL);
    imu.specific_force_mps2 = {0.3, -0.1, config.imu.gravity_mps2 + 0.05};
    imu.angular_velocity_radps = {0.1, -0.2, 0.15};
    estimator.ingestImu(imu);
  }
  auto input = batch(config, 55000000);
  auto transaction = estimator.prepareEpoch(input);
  const auto imu_block =
      estimator.buildPendingFactorBlock(transaction, transaction.imu_group.id);
  const auto sweep = ImuFaultSubspaceBuilder().verifyFiniteDifferenceSweep(
      transaction, imu_block);
  EXPECT_TRUE(sweep.sweep_executed);
  EXPECT_TRUE(sweep.sweep_verified) << sweep.sweep_worst_relative_error;
  EXPECT_EQ(sweep.sweep_epsilons.size(), 5u);
  EXPECT_EQ(sweep.sweep_relative_errors.size(), 5u);
  EXPECT_EQ(sweep.sweep_reintegrations, 60u);
  for (const double error : sweep.sweep_relative_errors) {
    EXPECT_TRUE(std::isfinite(error));
  }
  EXPECT_TRUE(std::isfinite(sweep.sweep_worst_relative_error));
  estimator.discardEpoch(
      std::move(transaction),
      {FdeStatus::ModelInvalid, "sweep development test", false});
}

TEST(IntegrityV2ImuOracle, InvalidAnalyticInputIsReportedWithoutOracle) {
  const uwb_imu_pl::EpochTransaction transaction;
  const uwb_imu_pl::LinearizedFactorBlock block;
  const auto result =
      uwb_imu_pl::ImuFaultSubspaceBuilder().buildAnalytic(transaction, block);
  EXPECT_FALSE(result.analytic_input_valid);
  EXPECT_FALSE(result.analytic_computation_valid);
  EXPECT_FALSE(result.oracle_executed);
  EXPECT_FALSE(result.analytic_reason.empty());
  const auto oracle =
      uwb_imu_pl::ImuFaultSubspaceBuilder().verifyFiniteDifferenceOracle(
          transaction, block);
  EXPECT_FALSE(oracle.oracle_executed);
  EXPECT_EQ(oracle.oracle_reintegrations, 0u);
}

// ---------------------------------------------------------------------------
// B2 (§5.7): one registry for order=1 and order=2.  Family classification,
// shared-parameter fail-closed behaviour, and on-demand cross blocks.
// ---------------------------------------------------------------------------

TEST(B2Registry, PairFamilySupportClassifiesFamilies) {
  using namespace uwb_imu_pl;
  FaultModeBasis uwb;
  uwb.sensor = SensorType::Uwb;
  uwb.physical_source_id = "uwb:1";
  FaultModeBasis uwb_other = uwb;
  uwb_other.physical_source_id = "uwb:2";
  FaultModeBasis accel;
  accel.sensor = SensorType::ImuAccelerometer;
  accel.physical_source_id = "imu_accel:0:interval:3";
  FaultModeBasis gyro = accel;
  gyro.sensor = SensorType::ImuGyroscope;

  EXPECT_EQ(pairFamilySupport(uwb, uwb_other), PairFamilySupport::Unsupported);
  EXPECT_EQ(pairFamilySupport(accel, gyro), PairFamilySupport::Unsupported);
  EXPECT_EQ(pairFamilySupport(uwb, accel), PairFamilySupport::Independent);
  EXPECT_EQ(pairFamilySupport(uwb, gyro), PairFamilySupport::Independent);
  FaultModeBasis same_source_imu = accel;
  same_source_imu.physical_source_id = uwb.physical_source_id;
  EXPECT_EQ(pairFamilySupport(uwb, same_source_imu),
            PairFamilySupport::SharedParameters);
  EXPECT_EQ(toString(PairFamilySupport::Unsupported), "UNSUPPORTED");
}

TEST(B2Registry, LocalCollinearityDoesNotEraseGloballyIndependentPair) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  // The modes are collinear on one common factor group, but the IMU mode has
  // an additional response in a second group.  A local rank gate would erase
  // this valid pair even though its global stacked response has full rank.
  FaultModeBasis left;
  left.id = FaultModeId(1);
  left.sensor = SensorType::Uwb;
  left.parameter_dimension = 1;
  const Eigen::VectorXd state_direction = window.H.col(0);
  Eigen::Index row_offset = 0;
  for (const auto& block : window.blocks) {
    left.raw_group_maps[block.group_id] = state_direction.segment(
        row_offset, block.residual_raw.size());
    row_offset += block.residual_raw.size();
  }
  FaultModeBasis right = left;
  right.id = FaultModeId(2);
  right.sensor = SensorType::ImuAccelerometer;
  right.physical_source_id = "imu_accel:0:interval:2";
  right.raw_group_maps[FactorGroupId(1)] =
      (Eigen::MatrixXd(2, 1) << 0.0, 1.0).finished();
  const std::vector<FaultModeBasis> modes{left, right};
  std::string reason;
  EXPECT_TRUE(
      hypothesisParametersIndependent(modes, {left.id, right.id}, &reason))
      << reason;

  // Disjoint groups (UWB group 2 vs IMU group 1) are independent.
  FaultModeBasis imu;
  imu.id = FaultModeId(3);
  imu.sensor = SensorType::ImuAccelerometer;
  imu.parameter_dimension = 1;
  imu.raw_group_maps[FactorGroupId(1)] = Eigen::Vector3d(0.0, 1.0, 0.0);
  EXPECT_TRUE(
      hypothesisParametersIndependent({left, imu}, {left.id, imu.id}, &reason))
      << reason;

  // Unsupported family is refused with its own reason.
  FaultModeBasis uwb_pair = left;
  uwb_pair.id = FaultModeId(4);
  EXPECT_FALSE(hypothesisParametersIndependent(
      {left, uwb_pair}, {left.id, uwb_pair.id}, &reason));
  EXPECT_NE(reason.find("unsupported pair family"), std::string::npos)
      << reason;
}

TEST(B2Registry, GlobalDangerousNullspaceIsPreservedAndFailsClosedNumerically) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  FaultModeBasis left;
  left.id = FaultModeId(1);
  left.sensor = SensorType::Uwb;
  left.parameter_dimension = 1;
  const Eigen::VectorXd state_direction = window.H.col(0);
  Eigen::Index row_offset = 0;
  for (const auto& block : window.blocks) {
    left.raw_group_maps[block.group_id] = state_direction.segment(
        row_offset, block.residual_raw.size());
    row_offset += block.residual_raw.size();
  }
  FaultModeBasis right = left;
  right.id = FaultModeId(2);
  right.sensor = SensorType::ImuAccelerometer;
  right.physical_source_id = "imu_accel:0:interval:2";
  const std::vector<FaultModeBasis> modes{left, right};
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(1);
  hypothesis.modes = {left.id, right.id};
  std::vector<FaultHypothesisV2> hypotheses{hypothesis};
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      window, modes, &hypotheses, 100.0);
  ASSERT_EQ(evidence.size(), 1u);
  EXPECT_FALSE(hypotheses[0].monitored);
  EXPECT_TRUE(hypothesisParametersIndependent(
      modes, {left.id, right.id}, nullptr));
  EXPECT_FALSE(hypotheses[0].monitorability.reason.empty());
  EXPECT_FALSE(evidence[0].plausible);
}

TEST(B2Registry, CrossBlocksAreRequestedOnlyWhereNeeded) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  FaultModeBasis first;
  first.id = FaultModeId(1);
  first.sensor = SensorType::Uwb;
  first.parameter_dimension = 1;
  first.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Ones();
  FaultModeBasis second;
  second.id = FaultModeId(2);
  second.sensor = SensorType::Uwb;
  second.parameter_dimension = 1;
  // Group 3 carries two rows in the synthetic fixture: the raw map must match
  // the group's native row count or the mode is rejected as invalid.
  second.raw_group_maps[FactorGroupId(3)] =
      (Eigen::MatrixXd(2, 1) << 1.0, 0.0).finished();
  FaultModeBasis imu;
  imu.id = FaultModeId(3);
  imu.sensor = SensorType::ImuAccelerometer;
  imu.parameter_dimension = 1;
  imu.raw_group_maps[FactorGroupId(1)] =
      (Eigen::MatrixXd(2, 1) << 0.0, 1.0).finished();
  const std::vector<FaultModeBasis> modes{first, second, imu};

  // Order=1 only: exactly one self block per evaluated hypothesis, and the
  // all-mode Gram is never materialized.
  {
    std::vector<FaultHypothesisV2> singles(3);
    for (std::size_t index = 0; index < 3; ++index) {
      singles[index].id = HypothesisId(index + 1);
      singles[index].modes = {modes[index].id};
      singles[index].prior_probability_bound = 1e-4;
      singles[index].p_md_allocation = 1e-3;
    }
    const auto before = NumericalWorkCounters::snapshot();
    (void)HypothesisEvidenceEvaluator().evaluateAll(window, modes, &singles,
                                                    100.0);
    const auto after = NumericalWorkCounters::snapshot();
    EXPECT_EQ(after.fault_cross_blocks - before.fault_cross_blocks, 3u)
        << "one self block per hypothesis, no pair cross terms";
    EXPECT_EQ(after.all_mode_gram_columns, 0u)
        << "the unconditional all-mode Gram must not be built";
  }
  // Order=2 enabled: the pair cross blocks appear, one per referenced pair.
  {
    std::vector<FaultHypothesisV2> pairs(2);
    pairs[0].id = HypothesisId(10);
    pairs[0].modes = {first.id, imu.id};
    pairs[0].prior_probability_bound = 1e-8;
    pairs[0].p_md_allocation = 1e-3;
    pairs[1].id = HypothesisId(11);
    pairs[1].modes = {second.id, imu.id};
    pairs[1].prior_probability_bound = 1e-8;
    pairs[1].p_md_allocation = 1e-3;
    const auto before = NumericalWorkCounters::snapshot();
    (void)HypothesisEvidenceEvaluator().evaluateAll(window, modes, &pairs,
                                                    100.0);
    const auto after = NumericalWorkCounters::snapshot();
    // self(first) + self(second) + self(imu) + cross(first,imu) +
    // cross(second,imu); no cross(first,second) because no hypothesis needs it.
    EXPECT_EQ(after.fault_cross_blocks - before.fault_cross_blocks, 5u);
  }
}

// ---------------------------------------------------------------------------
// B2 Stage 2 (roadmap R2): compact mode descriptors, capacity-bounded buffers,
// and the counted padded fallback.  Storage form must never change results.
// ---------------------------------------------------------------------------
namespace {

struct B2CompactFixture {
  uwb_imu_pl::LinearizedIntegrityWindow window;
  std::vector<uwb_imu_pl::FaultModeBasis> modes;
  std::vector<uwb_imu_pl::FaultHypothesisV2> hypotheses;
};

B2CompactFixture makeCompactFixture() {
  using namespace uwb_imu_pl;
  B2CompactFixture fixture;
  fixture.window = syntheticWindow();
  finalizeIntegrityWindow(&fixture.window, 1e-10, 1e10);
  FaultModeBasis first;
  first.id = FaultModeId(1);
  first.sensor = SensorType::Uwb;
  first.parameter_dimension = 1;
  first.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Ones();
  FaultModeBasis imu;
  imu.id = FaultModeId(2);
  imu.sensor = SensorType::ImuAccelerometer;
  imu.parameter_dimension = 1;
  imu.raw_group_maps[FactorGroupId(1)] =
      (Eigen::MatrixXd(2, 1) << 0.0, 1.0).finished();
  fixture.modes = {first, imu};
  FaultHypothesisV2 pair;
  pair.id = HypothesisId(1);
  pair.modes = {first.id, imu.id};
  pair.prior_probability_bound = 1e-8;
  pair.p_md_allocation = 1e-3;
  FaultHypothesisV2 single;
  single.id = HypothesisId(2);
  single.modes = {first.id};
  single.prior_probability_bound = 1e-4;
  single.p_md_allocation = 1e-3;
  fixture.hypotheses = {pair, single};
  return fixture;
}

}  // namespace

TEST(B2Compact, CompactPathAllocatesNoPaddedModeBlocks) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  const auto before = NumericalWorkCounters::snapshot();
  std::shared_ptr<const FrozenHypothesisNumerics> shared;
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      fixture.window, fixture.modes, &fixture.hypotheses, 100.0, &shared);
  const auto after = NumericalWorkCounters::snapshot();
  ASSERT_EQ(evidence.size(), 2u);
  ASSERT_TRUE(shared);
  // The compact path stores only the rows the modes touch: 3 (UWB group 2)
  // plus 2 (IMU group 1) rows, never window.H.rows() per mode.
  EXPECT_EQ(after.compact_mode_rows - before.compact_mode_rows, 5u);
  EXPECT_EQ(after.compact_mode_columns - before.compact_mode_columns, 2u);
  EXPECT_EQ(after.mode_dense_allocations - before.mode_dense_allocations, 0u)
      << "the compact path must not pad modes across the window rows";
  EXPECT_EQ(
      after.compact_capacity_fallbacks - before.compact_capacity_fallbacks, 0u);
  EXPECT_TRUE(shared->compact_mode_used);
  EXPECT_FALSE(shared->compact_capacity_exceeded);
  EXPECT_LT(
      shared->compact_rows,
      static_cast<std::size_t>(fixture.window.H.rows()) * fixture.modes.size());
}

TEST(B2Compact, CapacityExhaustionFallsBackWithIdenticalResults) {
  using namespace uwb_imu_pl;
  auto compact_fixture = makeCompactFixture();
  std::shared_ptr<const FrozenHypothesisNumerics> compact_shared;
  const auto compact_evidence = HypothesisEvidenceEvaluator().evaluateAll(
      compact_fixture.window, compact_fixture.modes,
      &compact_fixture.hypotheses, 100.0, &compact_shared);

  auto padded_fixture = makeCompactFixture();
  HypothesisEvaluationConfig config;
  config.compact_capacity.max_total_compact_rows = 1;  // force the fallback
  std::shared_ptr<const FrozenHypothesisNumerics> padded_shared;
  const auto before = NumericalWorkCounters::snapshot();
  const auto padded_evidence = HypothesisEvidenceEvaluator(config).evaluateAll(
      padded_fixture.window, padded_fixture.modes, &padded_fixture.hypotheses,
      100.0, &padded_shared);
  const auto after = NumericalWorkCounters::snapshot();
  ASSERT_EQ(compact_evidence.size(), padded_evidence.size());
  ASSERT_TRUE(padded_shared);
  EXPECT_TRUE(padded_shared->compact_capacity_exceeded);
  EXPECT_FALSE(padded_shared->compact_mode_used);
  EXPECT_EQ(
      after.compact_capacity_fallbacks - before.compact_capacity_fallbacks, 1u);
  // One padded block per valid mode, each across all window rows.
  EXPECT_EQ(after.mode_dense_allocations - before.mode_dense_allocations, 2u);
  EXPECT_EQ(
      after.mode_dense_allocation_rows - before.mode_dense_allocation_rows,
      static_cast<std::uint64_t>(2 * padded_fixture.window.H.rows()));
  // Identical results: the storage form is not allowed to change the numbers.
  for (std::size_t index = 0; index < compact_evidence.size(); ++index) {
    EXPECT_EQ(compact_evidence[index].all_in_statistic,
              padded_evidence[index].all_in_statistic);
    EXPECT_EQ(compact_evidence[index].conditioned_statistic,
              padded_evidence[index].conditioned_statistic);
    EXPECT_EQ(compact_evidence[index].log_evidence,
              padded_evidence[index].log_evidence);
    EXPECT_EQ(compact_evidence[index].plausible,
              padded_evidence[index].plausible);
    EXPECT_EQ(compact_evidence[index].estimated_fault.size(),
              padded_evidence[index].estimated_fault.size());
    if (compact_evidence[index].estimated_fault.size() ==
        padded_evidence[index].estimated_fault.size()) {
      EXPECT_EQ((compact_evidence[index].estimated_fault -
                 padded_evidence[index].estimated_fault)
                    .cwiseAbs()
                    .maxCoeff(),
                0.0);
    }
    EXPECT_EQ(compact_evidence[index].fault_gram.rows(),
              padded_evidence[index].fault_gram.rows());
    if (compact_evidence[index].fault_gram.rows() ==
            padded_evidence[index].fault_gram.rows() &&
        compact_evidence[index].fault_gram.cols() ==
            padded_evidence[index].fault_gram.cols()) {
      EXPECT_EQ((compact_evidence[index].fault_gram -
                 padded_evidence[index].fault_gram)
                    .cwiseAbs()
                    .maxCoeff(),
                0.0);
    }
  }
}

TEST(B2Compact, HypothesisDimensionCapRefusesFailClosed) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  HypothesisEvaluationConfig config;
  config.compact_capacity.max_hypothesis_dimension = 0;
  const auto before = NumericalWorkCounters::snapshot();
  const auto evidence = HypothesisEvidenceEvaluator(config).evaluateAll(
      fixture.window, fixture.modes, &fixture.hypotheses, 100.0);
  const auto after = NumericalWorkCounters::snapshot();
  ASSERT_EQ(evidence.size(), 2u);
  EXPECT_EQ(
      after.hypothesis_capacity_refusals - before.hypothesis_capacity_refusals,
      2u);
  for (std::size_t index = 0; index < evidence.size(); ++index) {
    EXPECT_FALSE(fixture.hypotheses[index].monitored);
    EXPECT_FALSE(evidence[index].plausible);
    EXPECT_NE(fixture.hypotheses[index].monitorability.reason.find("capacity"),
              std::string::npos)
        << fixture.hypotheses[index].monitorability.reason;
  }
}

TEST(B2Compact, LegacyMappedRouteCountsPaddedAllocations) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  HypothesisEvaluationConfig config;
  config.enable_low_dim_batch = false;  // route through evaluateMapped
  const auto before = NumericalWorkCounters::snapshot();
  const auto evidence = HypothesisEvidenceEvaluator(config).evaluateAll(
      fixture.window, fixture.modes, &fixture.hypotheses, 100.0);
  const auto after = NumericalWorkCounters::snapshot();
  EXPECT_EQ(evidence.size(), 2u);
  // The legacy route pads every mode across the window rows, and that cost is
  // visible in the counters instead of being implicit.
  EXPECT_EQ(after.mode_dense_allocations - before.mode_dense_allocations, 2u);
  EXPECT_EQ(after.compact_mode_rows - before.compact_mode_rows, 0u);
  EXPECT_GT(after.square_root_information_solves -
                before.square_root_information_solves,
            0u);
  EXPECT_GT(after.square_root_qt_applications -
                before.square_root_qt_applications,
            0u);
  EXPECT_EQ(after.spectral_rhs_solves - before.spectral_rhs_solves, 0u);
  EXPECT_EQ(after.square_root_fallbacks - before.square_root_fallbacks, 0u);
}

// ---------------------------------------------------------------------------
// B2 Stage 3 (§5.8): exact traversal, grouped envelopes with inclusion proofs,
// dominance verification, and the fail-closed coverage certificate.
// ---------------------------------------------------------------------------
TEST(B2Coverage, ExactTraversalCertifiesEveryTask) {
  using namespace uwb_imu_pl;
  const auto fixture = makeCompactFixture();
  const auto certificate = buildExactCoverageCertificate(fixture.modes);
  EXPECT_TRUE(certificate.complete);
  EXPECT_EQ(certificate.exact_count, fixture.modes.size());
  EXPECT_EQ(certificate.enveloped_count, 0u);
  EXPECT_EQ(certificate.uncovered_count, 0u);
  for (const auto& mode : fixture.modes) {
    EXPECT_EQ(coverageLabelFor(certificate, mode.id), CoverageLabel::Exact);
    EXPECT_EQ(coverageEnvelopeIdFor(certificate, mode.id), 0u);
  }
  EXPECT_EQ(std::string(toString(CoverageLabel::UpperEnvelope)),
            "UPPER_ENVELOPE");
}

TEST(B2Coverage, GroupedEnvelopeDischargesProofAndDominance) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // A group mode on the same factor group with a scaled map spans the leaf
  // exactly: A_leaf = A_group * 2.
  FaultModeBasis group;
  group.id = FaultModeId(10);
  group.sensor = SensorType::Uwb;
  group.parameter_dimension = 1;
  group.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Constant(0.5);
  const auto work_before = NumericalWorkCounters::snapshot();
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[0]}, {group});
  const auto work_after = NumericalWorkCounters::snapshot();
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  const auto& envelope = certificate.envelopes.front();
  EXPECT_TRUE(envelope.accepted) << envelope.reason;
  ASSERT_EQ(envelope.proofs.size(), 1u);
  EXPECT_TRUE(envelope.proofs.front().verified);
  EXPECT_LT(envelope.proofs.front().relative_residual, 1e-12);
  EXPECT_NEAR(envelope.proofs.front().transform(0, 0), 2.0, 1e-12);
  EXPECT_TRUE(envelope.dominant) << envelope.dominance_margin;
  EXPECT_GE(envelope.dominance_margin, -1e-9);
  EXPECT_EQ(certificate.enveloped_count, 1u);
  EXPECT_TRUE(certificate.complete);
  EXPECT_EQ(coverageLabelFor(certificate, fixture.modes[0].id),
            CoverageLabel::UpperEnvelope);
  EXPECT_EQ(coverageEnvelopeIdFor(certificate, fixture.modes[0].id), 1u);
  EXPECT_GT(work_after.square_root_information_solves -
                work_before.square_root_information_solves,
            0u);
  EXPECT_EQ(work_after.spectral_rhs_solves - work_before.spectral_rhs_solves,
            0u);
  EXPECT_EQ(work_after.square_root_fallbacks -
                work_before.square_root_fallbacks,
            0u);
}

TEST(B2Coverage, MismatchedModesAreRejectedByTheInclusionProof) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // The group touches a different factor group: no transform can reproduce the
  // leaf map, so the envelope must be rejected and the leaf stays EXACT.
  FaultModeBasis group;
  group.id = FaultModeId(10);
  group.sensor = SensorType::Uwb;
  group.parameter_dimension = 1;
  group.raw_group_maps[FactorGroupId(3)] =
      (Eigen::MatrixXd(2, 1) << 1.0, 0.0).finished();
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[0]}, {group});
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  const auto& envelope = certificate.envelopes.front();
  EXPECT_FALSE(envelope.accepted);
  EXPECT_NE(envelope.reason.find("inclusion proof"), std::string::npos)
      << envelope.reason;
  EXPECT_EQ(certificate.exact_count, 1u);
  EXPECT_EQ(coverageLabelFor(certificate, fixture.modes[0].id),
            CoverageLabel::Exact);
}

TEST(B2Coverage, UnsupportedFamilyCannotFormAnEnvelope) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // A UWB group cannot span an IMU leaf (different factor groups and row
  // counts): the proof fails and no label is promoted.
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[1]}, {fixture.modes[0]});
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  EXPECT_FALSE(certificate.envelopes.front().accepted);
  EXPECT_EQ(coverageLabelFor(certificate, fixture.modes[1].id),
            CoverageLabel::Exact);
}

TEST(B2Coverage, IncompleteCoverageMakesProtectedOutputUnavailable) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // Capacity exhaustion: no envelope may be built, so the certificate can no
  // longer vouch for the window.
  CoverageCapacity capacity;
  capacity.max_envelopes = 0;
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, fixture.modes, {fixture.modes[0]}, capacity);
  EXPECT_TRUE(certificate.capacity_exceeded);
  EXPECT_FALSE(certificate.complete);
  EXPECT_NE(certificate.reason.find("protected output unavailable"),
            std::string::npos)
      << certificate.reason;

  // An unusable mode map is UNCOVERED and also makes coverage incomplete.
  auto broken_window = fixture.window;
  FaultModeBasis broken;
  broken.id = FaultModeId(20);
  broken.sensor = SensorType::ImuGyroscope;
  broken.parameter_dimension = 1;
  broken.raw_group_maps[FactorGroupId(999)] = Eigen::Vector3d::Ones();
  const auto uncovered = buildGroupedCoverageCertificate(
      broken_window, {fixture.modes[0], broken}, {fixture.modes[0]});
  EXPECT_FALSE(uncovered.complete);
  ASSERT_EQ(uncovered.uncovered_modes.size(), 1u);
  EXPECT_EQ(uncovered.uncovered_modes.front().value(), 20u);
  EXPECT_EQ(coverageLabelFor(uncovered, broken.id), CoverageLabel::Uncovered);
}

TEST(B2Coverage, DominanceRejectsAnUnderCoveringEnvelope) {
  using namespace uwb_imu_pl;
  auto fixture = makeCompactFixture();
  // A degenerate group map can satisfy a loose inclusion tolerance while
  // claiming the leaf is unmonitored.  The dominance obligation must catch it:
  // the envelope bound (zero) is below the leaf's exact bound, so the envelope
  // is rejected and the leaf stays EXACT.
  FaultModeBasis degenerate;
  degenerate.id = FaultModeId(12);
  degenerate.sensor = SensorType::Uwb;
  degenerate.parameter_dimension = 1;
  degenerate.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Zero();
  CoverageCapacity capacity;
  capacity.inclusion_tolerance = 2.0;  // loose enough to pass the proof
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[0]}, {degenerate}, capacity);
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  const auto& envelope = certificate.envelopes.front();
  EXPECT_FALSE(envelope.dominant);
  EXPECT_FALSE(envelope.accepted);
  EXPECT_NE(envelope.reason.find("below the leaf exact bound"),
            std::string::npos)
      << envelope.reason;
  EXPECT_EQ(coverageLabelFor(certificate, fixture.modes[0].id),
            CoverageLabel::Exact);
  EXPECT_TRUE(certificate.complete);
}

// ---------------------------------------------------------------------------
// B3 Stage 1 (roadmap section 5.9): verified joint bound and risk ledger.
// ---------------------------------------------------------------------------

TEST(B3Risk, RSK01UnionBoundDoesNotDoubleChargeAndSolvesConservatively) {
  using namespace uwb_imu_pl;
  // Charged tail and per-axis split: the case pays alpha_h for the hypothesis,
  // so every axis may only use alpha_h / 3 (using alpha_h per axis would
  // understate the tail the ledger pays for).
  const double allocation = 3.0e-8;
  const double prior = 1.0e-4;
  const double p_md = 1.0e-3;
  const AxisTailSplit split = axisTailSplit(allocation, prior, p_md, 1e-5);
  ASSERT_TRUE(split.valid) << split.reason;
  EXPECT_NEAR(split.hypothesis_tail, allocation / prior, 1e-18);
  EXPECT_NEAR(split.axis_tail, split.hypothesis_tail / 3.0, 1e-18);
  EXPECT_DOUBLE_EQ(split.charge, std::max(split.hypothesis_tail, p_md));
  // The charge times the prior is exactly the allocation (no double charge).
  EXPECT_NEAR(prior * split.charge, std::max(allocation, prior * p_md), 1e-18);

  // The normal multiplier must grow when the tail is split over the axes.
  const double k_axis =
      StatisticalBoundsCache::normalTwoSidedMultiplier(split.axis_tail);
  const double k_single =
      StatisticalBoundsCache::normalTwoSidedMultiplier(split.hypothesis_tail);
  EXPECT_GT(k_axis, k_single);
  EXPECT_LT(k_single, k_axis);

  // dof = 1 has an independent closed form:
  //   P(chi'^2_1(lambda) <= tau) = Phi(sqrt(tau) - sqrt(lambda))
  //                               - Phi(-sqrt(tau) - sqrt(lambda))
  const int dof = 1;
  const double threshold = 6.0;
  const StatisticalBoundKey key;
  const auto boundary = StatisticalBoundsCache::noncentralityBoundaryVerified(
      dof, threshold, p_md, key);
  ASSERT_TRUE(boundary.valid) << boundary.reason;
  EXPECT_TRUE(boundary.converged);
  auto miss = [&](double noncentrality) {
    const double root = std::sqrt(noncentrality);
    const double t = std::sqrt(threshold);
    const auto normal = boost::math::normal();
    return boost::math::cdf(normal, t - root) -
           boost::math::cdf(normal, -t - root);
  };
  EXPECT_LE(miss(boundary.value), p_md)
      << "the returned boundary must be on the conservative side";
  EXPECT_LT(boundary.residual, 1e-12);
  EXPECT_LT(boundary.bracket_width, 1e-9 * std::max(1.0, boundary.value));
  // Monotone in p_md: a smaller miss allocation needs a larger noncentrality.
  const auto tighter = StatisticalBoundsCache::noncentralityBoundaryVerified(
      dof, threshold, 0.1 * p_md, key);
  ASSERT_TRUE(tighter.valid) << tighter.reason;
  EXPECT_GT(tighter.value, boundary.value);

  // Seeded synthetic acceptance sample (statistical, not a proof): with the
  // split tails, the measured per-axis exceedance of the k-sigma bound stays
  // within a 4-sigma band of the nominal two-sided tail.  Stated as sampling
  // evidence only; the deterministic claim is the union bound above.
  std::mt19937_64 generator(20260921);
  std::normal_distribution<double> standard(0.0, 1.0);
  const std::size_t samples = 200000;
  const double sigma = 0.37;
  std::size_t exceedances = 0;
  for (std::size_t index = 0; index < samples; ++index) {
    // Correlated three-axis protected error with equal marginals.
    const double common = standard(generator);
    for (int axis = 0; axis < 3; ++axis) {
      const double value = sigma * (0.6 * common + 0.8 * standard(generator));
      if (std::abs(value) > k_axis * sigma) ++exceedances;
    }
  }
  const double total = static_cast<double>(samples) * 3.0;
  const double rate = static_cast<double>(exceedances) / total;
  const double expected = split.axis_tail;
  const double sample_sigma = std::sqrt(expected * (1.0 - expected) / total);
  EXPECT_LT(rate, expected + 4.0 * sample_sigma)
      << "per-axis exceedance must stay inside the charged tail band";
}

TEST(B3Risk, RSK02LedgerHasNoHiddenZerosAndOverlapGivesNoDividend) {
  using namespace uwb_imu_pl;
  RiskBudgetV2 risk;
  FaultHypothesisV2 first;
  first.id = HypothesisId(1);
  first.hmi_allocation = 3.0e-8;
  first.prior_probability_bound = 1.0e-4;
  first.p_md_allocation = 1.0e-3;
  FaultHypothesisV2 second = first;
  second.id = HypothesisId(2);
  RiskLedgerInputs inputs;
  inputs.omitted_event_set = {"two_uwb", "imu_imu"};
  const RiskLedger unknown = buildRiskLedger(risk, {first}, inputs);
  EXPECT_FALSE(unknown.closes) << unknown.reason;
  EXPECT_FALSE(unknown.all_terms_validated);
  // The legacy sidecar has no canonical known/validated zero proofs, so the
  // complete contract is invalid as well as non-closing; it remains useful as
  // a diagnostic ledger and must expose every unknown term below.
  ASSERT_FALSE(unknown.inputs_valid);
  // Every term is present with a status and a source: nothing hides as a
  // silent zero.
  for (const char* id :
       {"nominal", "p_nm", "hypotheses", "hypotheses_miss_channel", "bridge",
        "history", "model", "omitted", "envelope", "selection"}) {
    const auto* term = unknown.find(id);
    ASSERT_NE(term, nullptr) << id;
    EXPECT_FALSE(term->source.empty()) << id;
    EXPECT_FALSE(term->note.empty()) << id;
  }
  EXPECT_EQ(unknown.find("omitted")->status, RiskTermStatus::NotImplemented);
  EXPECT_NE(unknown.find("omitted")->note.find("two_uwb"), std::string::npos);
  EXPECT_EQ(unknown.find("bridge")->status, RiskTermStatus::AssumedUnvalidated);
  EXPECT_TRUE(std::isnan(unknown.find("bridge")->value));
  EXPECT_TRUE(std::isnan(unknown.find("omitted")->value));
  EXPECT_FALSE(unknown.formal_eligible);
  EXPECT_TRUE(std::isnan(unknown.declared_total));

  // Once every zero/bound carries an explicit proof, the same values close;
  // this is distinct from deployment formal eligibility.
  CompleteRiskInputsV1 qualified;
  qualified.legacy = inputs;
  qualified.omitted_scope_complete = true;
  qualified.bridge_escape_validated = true;
  qualified.history_escape_validated = true;
  qualified.model_escape_validated = true;
  qualified.omitted_event_bound = 0.0;
  qualified.omitted_event_bound_known = true;
  qualified.omitted_event_bound_validated = true;
  qualified.envelope_event_bound = 0.0;
  qualified.envelope_event_bound_known = true;
  qualified.envelope_event_bound_validated = true;
  qualified.legacy.selection_contract_frozen = true;
  qualified.selection_extra_bound = 0.0;
  qualified.selection_bound_known = true;
  qualified.selection_bound_validated = true;
  CompleteRiskStatusV1 one_status;
  const RiskLedger one = buildRiskLedger(
      risk, {first}, qualified, &one_status);
  ASSERT_TRUE(one_status.complete_bound_closes) << one.reason;
  ASSERT_TRUE(one.all_terms_validated) << one.reason;

  // Overlapping coverage gives no risk dividend: adding a hypothesis never
  // lowers the charged total, and the miss channel stays explicit.
  CompleteRiskStatusV1 two_status;
  const RiskLedger two = buildRiskLedger(
      risk, {first, second}, qualified, &two_status);
  ASSERT_TRUE(two_status.complete_bound_closes) << two.reason;
  EXPECT_GT(two.charged_total, one.charged_total);
  const auto* one_fault = one.find("hypotheses");
  const auto* two_fault = two.find("hypotheses");
  ASSERT_NE(one_fault, nullptr);
  ASSERT_NE(two_fault, nullptr);
  EXPECT_NEAR(two_fault->value, 2.0 * one_fault->value, 1e-18);
  // The detector miss channel is validated and charged.
  const auto* miss = two.find("hypotheses_miss_channel");
  ASSERT_NE(miss, nullptr);
  EXPECT_GT(miss->value, 0.0);
  EXPECT_EQ(miss->status, RiskTermStatus::Validated);
  EXPECT_NEAR(two.charged_total,
              one.find("nominal")->value + one.find("p_nm")->value +
                  2.0 * one_fault->value + miss->value,
              1e-18);
}

TEST(B3Risk, RSK03PositionMarginAndResidualThresholdActTogether) {
  using namespace uwb_imu_pl;
  // Both the detector threshold (through Lambda) and the position margin
  // (through k * sigma) enter the same bound: neither channel alone may be
  // used to claim the other.
  const StatisticalBoundKey key;
  const double p_md = 1e-3;
  const auto loose =
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 10.0, p_md, key);
  const auto tight =
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 40.0, p_md, key);
  ASSERT_TRUE(loose.valid);
  ASSERT_TRUE(tight.valid);
  EXPECT_GT(tight.value, loose.value)
      << "a higher detector threshold must require more noncentrality";
  const AxisTailSplit split = axisTailSplit(3e-8, 1e-4, p_md, 1e-5);
  ASSERT_TRUE(split.valid);
  const double k =
      StatisticalBoundsCache::normalTwoSidedMultiplier(split.axis_tail);
  const double sigma = 0.5;
  const double fault_component = std::sqrt(tight.value) * 0.25 + k * sigma;
  EXPECT_GT(fault_component, k * sigma)
      << "the fault size channel and the noise channel must both contribute";
  EXPECT_GT(fault_component, std::sqrt(tight.value) * 0.25);
}

TEST(B3Risk, BoundaryInputsAreRejectedAndCacheIdentityIsVersioned) {
  using namespace uwb_imu_pl;
  StatisticalBoundsCache::clear();
  const StatisticalBoundKey key;
  // beta = 0, beta >= 1, zero dof, non-positive threshold.
  EXPECT_FALSE(
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 10.0, 0.0, key)
          .valid);
  EXPECT_FALSE(
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 10.0, 1.0, key)
          .valid);
  EXPECT_FALSE(
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 10.0, 1.5, key)
          .valid);
  EXPECT_FALSE(
      StatisticalBoundsCache::noncentralityBoundaryVerified(0, 10.0, 1e-3, key)
          .valid);
  EXPECT_FALSE(
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 0.0, 1e-3, key)
          .valid);
  EXPECT_FALSE(
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, -1.0, 1e-3, key)
          .valid);
  // Key/dof disagreement is a policy mismatch, not a silent reuse.
  StatisticalBoundKey wrong_dof;
  wrong_dof.dof = 7;
  EXPECT_FALSE(StatisticalBoundsCache::noncentralityBoundaryVerified(
                   4, 10.0, 1e-3, wrong_dof)
                   .valid);
  EXPECT_GE(StatisticalBoundsCache::stats().policy_mismatches, 1u);

  // Illegal priors are rejected by the split, never clamped.
  const AxisTailSplit bad = axisTailSplit(1e-8, 0.0, 1e-3, 1e-5);
  EXPECT_FALSE(bad.valid);
  EXPECT_NE(bad.reason.find("prior"), std::string::npos);
  const AxisTailSplit negative = axisTailSplit(-1.0, 1e-4, 1e-3, 1e-5);
  EXPECT_FALSE(negative.valid);
  const AxisTailSplit zero = axisTailSplit(0.0, 1e-4, 1e-3, 1e-5);
  EXPECT_TRUE(zero.valid);
  EXPECT_DOUBLE_EQ(zero.charge, 1e-3);

  // Tiny tails must stay finite (complement evaluation, no 1 - tiny).
  bool valid = false;
  const double k_tiny =
      StatisticalBoundsCache::normalTwoSidedMultiplierVerified(1e-300, &valid);
  EXPECT_TRUE(valid);
  EXPECT_TRUE(std::isfinite(k_tiny));
  EXPECT_GT(k_tiny, 30.0);
  const double k_invalid =
      StatisticalBoundsCache::normalTwoSidedMultiplierVerified(0.0, &valid);
  EXPECT_FALSE(valid);
  EXPECT_FALSE(std::isfinite(k_invalid));

  // Cache identity: same key hits, a contract/envelope version change misses.
  const auto before = StatisticalBoundsCache::stats();
  const auto first =
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 21.0, 1e-3, key);
  const auto hit =
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 21.0, 1e-3, key);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(hit.valid);
  EXPECT_DOUBLE_EQ(first.value, hit.value);
  StatisticalBoundKey new_contract;
  new_contract.contract_version = key.contract_version + 1;
  const auto versioned = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 21.0, 1e-3, new_contract);
  ASSERT_TRUE(versioned.valid);
  EXPECT_DOUBLE_EQ(versioned.value, first.value);
  StatisticalBoundKey envelope_changed;
  envelope_changed.envelope_fingerprint = 0xABCDEFULL;
  const auto enveloped = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 21.0, 1e-3, envelope_changed);
  ASSERT_TRUE(enveloped.valid);
  const auto after = StatisticalBoundsCache::stats();
  EXPECT_GE(after.hits - before.hits, 1u);
  EXPECT_GE(after.misses - before.misses, 3u)
      << "a contract or envelope version change must miss the cache";
}

TEST(B3Risk, HistorySummaryVersionIsPartOfCacheIdentity) {
  using namespace uwb_imu_pl;
  StatisticalBoundsCache::clear();
  const StatisticalBoundKey base;
  const auto first = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 21.0, 1e-3, base);
  ASSERT_TRUE(first.valid);
  const auto hit = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 21.0, 1e-3, base);
  ASSERT_TRUE(hit.valid);
  EXPECT_DOUBLE_EQ(first.value, hit.value);
  const auto before = StatisticalBoundsCache::stats();

  // C1-c: a rebuilt summary with a different binding digest is a different
  // cache identity; the value is recomputed rather than reused.
  StatisticalBoundKey rebuilt_summary = base;
  rebuilt_summary.history_summary_version = 0x1234ULL;
  const auto versioned = StatisticalBoundsCache::noncentralityBoundaryVerified(
      4, 21.0, 1e-3, rebuilt_summary);
  ASSERT_TRUE(versioned.valid);
  EXPECT_DOUBLE_EQ(versioned.value, first.value);
  const auto versioned_again =
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 21.0, 1e-3,
                                                            rebuilt_summary);
  ASSERT_TRUE(versioned_again.valid);
  EXPECT_DOUBLE_EQ(versioned_again.value, first.value);

  // C1-c fix: `envelope_kind` is a first-class identity axis (it was declared
  // on the struct but missing from the stored tuple, silently aliasing two
  // different kinds onto one entry).
  StatisticalBoundKey kind_one = base;
  kind_one.envelope_kind = 1;
  StatisticalBoundKey kind_two = base;
  kind_two.envelope_kind = 2;
  ASSERT_TRUE(StatisticalBoundsCache::noncentralityBoundaryVerified(
                  4, 21.0, 1e-3, kind_one)
                  .valid);
  ASSERT_TRUE(StatisticalBoundsCache::noncentralityBoundaryVerified(
                  4, 21.0, 1e-3, kind_two)
                  .valid);
  const auto kind_one_again =
      StatisticalBoundsCache::noncentralityBoundaryVerified(4, 21.0, 1e-3,
                                                            kind_one);
  ASSERT_TRUE(kind_one_again.valid);

  const auto after = StatisticalBoundsCache::stats();
  // Hits: the rebuilt-summary reuse and the envelope-kind reuse.  Misses: one
  // entry per distinct identity (summary version, kind one, kind two).
  EXPECT_GE(after.hits - before.hits, 2u);
  EXPECT_GE(after.misses - before.misses, 3u)
      << "a summary version or envelope kind change must miss the cache";
}

// ---------------------------------------------------------------------------
// B4 Stage 3: lazy FDE action entities and health-state preservation.
// ---------------------------------------------------------------------------

TEST(B4LazyFde, FDE01HealthyFrameBuildsNoActionEntitiesButKeepsSensitivity) {
  using namespace uwb_imu_pl;
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d(0.1, 0.2, 0.3));
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  initial.velocity_world_mps = {0.2, 0.1, 0.0};
  initial.q_world_body = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.2, Eigen::Vector3d(1, 2, 3).normalized()));
  estimator.initialize(initial, config.realtime.prior_sigmas);
  for (int sample = 0; sample <= 10; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(sample + 1);
    imu.timestamp = TimestampNs(sample * 5000000LL);
    imu.specific_force_mps2 = {0.3, -0.1, config.imu.gravity_mps2 + 0.05};
    imu.angular_velocity_radps = {0.1, -0.2, 0.15};
    estimator.ingestImu(imu);
  }
  auto input = batch(config, 55000000);
  auto transaction = estimator.prepareEpoch(input);
  auto window =
      estimator.buildIntegrityWindow(transaction, IntegrityWindowRequest{});
  auto imu_block =
      estimator.buildPendingFactorBlock(transaction, transaction.imu_group.id);
  auto bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bridge_group.id);
  const auto imu_maps = ImuFaultSubspaceBuilder().build(transaction, imu_block);

  // Lazy (default): mode descriptions and hypotheses exist, entities do not.
  auto lazy = HypothesisGenerator().generate(window, transaction, imu_maps,
                                             bridge_block);
  EXPECT_TRUE(lazy.action_entities_built == false);
  EXPECT_TRUE(lazy.single_mode_actions.empty());
  EXPECT_EQ(lazy.action_entities_constructed, 0u);
  EXPECT_EQ(lazy.bridge_blocks_built, 0u);
  EXPECT_EQ(lazy.action_entities_deferred, lazy.modes.size());
  ASSERT_EQ(lazy.actions.size(), 1u);
  EXPECT_EQ(lazy.actions.front().id.value(), 1u);
  EXPECT_EQ(lazy.actions.front().action_model_id, "KEEP_ALL");
  // Sensitivity is preserved: every mode still carries its fault map and every
  // hypothesis its risk allocation, which is what the PL path consumes.
  ASSERT_FALSE(lazy.modes.empty());
  for (const auto& mode : lazy.modes) {
    EXPECT_FALSE(mode.raw_group_maps.empty());
  }
  ASSERT_FALSE(lazy.hypotheses.empty());
  for (const auto& hypothesis : lazy.hypotheses) {
    EXPECT_GT(hypothesis.hmi_allocation, 0.0);
  }

  // A bridge spanning a state before the frozen detector window must fail
  // closed as missing provenance.  Before P1-06 this unsigned epoch
  // subtraction wrote an Eigen block outside the matrix and corrupted the
  // Gaussian factor graph during fault/recovery attempts.
  auto narrow_window = window;
  narrow_window.detector_first_epoch = transaction.proposed_epoch;
  auto narrow = lazy;
  HypothesisGenerator::ensureActionEntities(transaction, narrow_window, &narrow);
  bool saw_unrepresentable_imu_bridge = false;
  for (const auto& action : narrow.single_mode_actions) {
    if (action.action_model_id == "IMU_HISTORY_BRIDGE_RECOVERY") {
      saw_unrepresentable_imu_bridge = true;
      EXPECT_EQ(action.recoverability, HistoryRecoverability::MissingProvenance);
      EXPECT_TRUE(action.added_blocks.empty());
    }
  }
  EXPECT_TRUE(saw_unrepresentable_imu_bridge);

  // Eager compatibility path produces the same identities.
  HypothesisGeneratorConfig eager_config;
  eager_config.lazy_action_entities = false;
  auto eager = HypothesisGenerator(eager_config)
                   .generate(window, transaction, imu_maps, bridge_block);
  EXPECT_TRUE(eager.action_entities_built);
  EXPECT_EQ(eager.action_entities_constructed, eager.modes.size());
  EXPECT_EQ(eager.action_entities_deferred, 0u);
  ASSERT_EQ(eager.single_mode_actions.size(), eager.modes.size());
  EXPECT_GT(eager.actions.size(), 1u)
      << "the eager path still deduplicates recoverable actions";

  // On-demand materialization is idempotent and reproduces the eager ids.
  HypothesisGenerator::ensureActionEntities(transaction, window, &lazy);
  EXPECT_TRUE(lazy.action_entities_built);
  // The deduplicated catalog is reproduced identically (same ids, same order).
  ASSERT_EQ(lazy.actions.size(), eager.actions.size());
  for (std::size_t index = 0; index < lazy.actions.size(); ++index) {
    EXPECT_EQ(lazy.actions[index].id.value(), eager.actions[index].id.value());
  }
  ASSERT_EQ(lazy.single_mode_actions.size(), eager.single_mode_actions.size());
  for (std::size_t index = 0; index < lazy.single_mode_actions.size();
       ++index) {
    EXPECT_EQ(lazy.single_mode_actions[index].id.value(),
              eager.single_mode_actions[index].id.value());
    EXPECT_EQ(lazy.single_mode_actions[index].action_model_id,
              eager.single_mode_actions[index].action_model_id);
    EXPECT_EQ(lazy.single_mode_actions[index].groups_to_remove.size(),
              eager.single_mode_actions[index].groups_to_remove.size());
  }
  const std::size_t constructed = lazy.action_entities_constructed;
  const std::size_t bridges = lazy.bridge_blocks_built;
  HypothesisGenerator::ensureActionEntities(transaction, window, &lazy);
  EXPECT_EQ(lazy.action_entities_constructed, constructed)
      << "ensureActionEntities must be idempotent";
  EXPECT_EQ(lazy.bridge_blocks_built, bridges);
  // On-demand materialization builds exactly the blocks the eager path builds.
  EXPECT_EQ(lazy.bridge_blocks_built, eager.bridge_blocks_built);
  estimator.discardEpoch(
      std::move(transaction),
      {FdeStatus::ModelInvalid, "lazy FDE development test", false});
}

TEST(B4LazyFde, FDE02IsolationPathStillMaterializesEvidenceOnDemand) {
  using namespace uwb_imu_pl;
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d(0.1, 0.2, 0.3));
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  initial.velocity_world_mps = {0.2, 0.1, 0.0};
  initial.q_world_body = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.2, Eigen::Vector3d(1, 2, 3).normalized()));
  estimator.initialize(initial, config.realtime.prior_sigmas);
  for (int sample = 0; sample <= 10; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(sample + 1);
    imu.timestamp = TimestampNs(sample * 5000000LL);
    imu.specific_force_mps2 = {0.3, -0.1, config.imu.gravity_mps2 + 0.05};
    imu.angular_velocity_radps = {0.1, -0.2, 0.15};
    estimator.ingestImu(imu);
  }
  auto input = batch(config, 55000000);
  auto transaction = estimator.prepareEpoch(input);
  auto window =
      estimator.buildIntegrityWindow(transaction, IntegrityWindowRequest{});
  auto imu_block =
      estimator.buildPendingFactorBlock(transaction, transaction.imu_group.id);
  auto bridge_block = estimator.buildPendingFactorBlock(
      transaction, transaction.generic_bridge_group.id);
  const auto imu_maps = ImuFaultSubspaceBuilder().build(transaction, imu_block);
  auto models = HypothesisGenerator().generate(window, transaction, imu_maps,
                                               bridge_block);
  std::vector<FaultModeEvidence> evidence;
  for (const auto& hypothesis : models.hypotheses) {
    FaultModeEvidence item;
    item.hypothesis = hypothesis.id;
    item.plausible = true;
    evidence.push_back(item);
  }
  HypothesisGeneratorConfig compact_config;
  compact_config.max_candidate_count = 2;
  auto compact_arena = HypothesisGenerator(compact_config)
      .actionsForPlausibleSetV3(window, transaction, models, evidence);
  auto compact_lease = compact_arena.lease();
  const auto compact_validation =
      validateCompactActionSearchV3(compact_lease);
  ASSERT_TRUE(compact_validation.valid) << compact_validation.reason;
  ASSERT_TRUE(compact_validation.exhaustive) << compact_validation.reason;
  EXPECT_EQ(compact_lease.search().generated,
            compact_lease.search().evaluated);
  EXPECT_EQ(compact_lease.search().omitted, 0u);
  EXPECT_LE(compact_lease.search().peak_batch_actions, 2u);
  EXPECT_GT(compact_lease.search().peak_batch_heavy_bytes, 0u);
  std::uint64_t legacy_canonical_bytes = 0;
  for (std::size_t descriptor_index = 0;
       descriptor_index < compact_lease.search().actions.size();
       ++descriptor_index) {
    const auto& descriptor =
        compact_lease.search().actions[descriptor_index];
    const ExclusionAction exact = compact_lease.materialize(descriptor_index);
    legacy_canonical_bytes += exactActionOperationIdentityV1(exact).size();
    legacy_canonical_bytes += exactActionSemanticIdentityV1(exact).size();
    for (const auto& block : descriptor.ordered_blocks) {
      EXPECT_EQ(block.strong_digest.size(), 64u);
    }
  }
  EXPECT_LT(compact_lease.search().sidecar_length,
            legacy_canonical_bytes)
      << "content-addressed blocks must not be repeated in every action";
  EXPECT_GE(compact_lease.search().referenced_block_count,
            compact_lease.search().unique_block_count);
  auto collision_arena = HypothesisGenerator(compact_config)
      .actionsForPlausibleSetV3(
          window, transaction, models, evidence, {}, true);
  auto collision_lease = collision_arena.lease();
  const auto collision_validation =
      validateCompactActionSearchV3(collision_lease);
  ASSERT_TRUE(collision_validation.valid) << collision_validation.reason;
  ASSERT_TRUE(collision_validation.exhaustive)
      << collision_validation.reason;
  EXPECT_EQ(collision_lease.search().evaluated,
            compact_lease.search().evaluated)
      << "distinct canonical bytes must survive a forced digest collision";
  EXPECT_GT(collision_lease.search().digest_collision_comparisons, 0u);
  collision_lease.close();
  collision_arena.close();
  auto io_failure_arena = HypothesisGenerator(compact_config)
      .actionsForPlausibleSetV3(window, transaction, models, evidence);
  auto io_failure_lease = io_failure_arena.lease();
  io_failure_arena.injectSidecarReadFailureForTest();
  const auto io_failure = validateCompactActionSearchV3(io_failure_lease);
  EXPECT_FALSE(io_failure.valid);
  EXPECT_FALSE(io_failure.exhaustive);
  io_failure_lease.close();
  io_failure_arena.close();
  std::vector<ExclusionAction> compact_actions;
  for (std::size_t begin = 0;
       begin < compact_lease.search().actions.size(); begin += 2) {
    const auto count = std::min<std::size_t>(
        2, compact_lease.search().actions.size() - begin);
    auto materialized = compact_lease.materializeBatch(begin, count);
    compact_actions.insert(compact_actions.end(),
                           std::make_move_iterator(materialized.begin()),
                           std::make_move_iterator(materialized.end()));
  }
  auto compact_terminals = compact_actions;
  for (auto& action : compact_terminals) action.added_blocks.clear();
  const auto consumption = validateCompactActionConsumptionV3(
      compact_lease, compact_terminals);
  EXPECT_TRUE(consumption.valid) << consumption.reason;
  ASSERT_FALSE(compact_terminals.empty());
  compact_terminals.front().action_model_id = "TAMPERED";
  EXPECT_FALSE(validateCompactActionConsumptionV3(
      compact_lease, compact_terminals).valid);
  EXPECT_EQ(models.action_entities_constructed, 0u);
  const auto actions = HypothesisGenerator().actionsForPlausibleSet(
      window, transaction, &models, evidence);
  ASSERT_EQ(compact_actions.size(), actions.size());
  for (std::size_t index = 0; index < actions.size(); ++index) {
    EXPECT_EQ(exactActionOperationIdentityV1(compact_actions[index]),
              exactActionOperationIdentityV1(actions[index]));
    EXPECT_EQ(exactActionSemanticIdentityV1(compact_actions[index]),
              exactActionSemanticIdentityV1(actions[index]));
  }
  compact_lease.close();
  EXPECT_FALSE(compact_lease.valid());
  compact_arena.close();
  EXPECT_FALSE(compact_arena.valid());
  EXPECT_GE(actions.size(), 2u)
      << "the isolation path must still build its candidate actions";
  EXPECT_GT(models.action_entities_constructed, 0u)
      << "entities are materialized on demand, not eagerly";
  EXPECT_TRUE(models.action_entities_built);
  EXPECT_EQ(models.action_entities_deferred, 0u);
  // Action identity is stable: KEEP_ALL first, then the mode actions in order.
  bool keep_all_seen = false;
  std::set<std::uint64_t> ids;
  for (const auto& action : actions) {
    EXPECT_TRUE(ids.insert(action.id.value()).second)
        << "duplicate action id " << action.id.value();
    keep_all_seen = keep_all_seen || action.action_model_id == "KEEP_ALL";
  }
  EXPECT_TRUE(keep_all_seen);
  estimator.discardEpoch(
      std::move(transaction),
      {FdeStatus::ModelInvalid, "lazy FDE isolation test", false});
}
// ---------------------------------------------------------------------------
// B3 Stage 2: the detection-space tri-state decides availability with
// direction-level reasons (no dictionary-wide rejection).
// ---------------------------------------------------------------------------

TEST(B3ZeroSpace, DangerousNullspaceIsUnavailableWithDirectionLevelReason) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  // A fault map proportional to the nominal design column is invisible in the
  // detection space but moves the protected state: exactly the dangerous case.
  const Eigen::Index rows = window.H.rows();
  const Eigen::VectorXd state_direction = window.H.col(0);
  FaultModeBasis mode;
  mode.id = FaultModeId(1);
  mode.sensor = SensorType::Uwb;
  mode.parameter_dimension = 1;
  Eigen::Index row_offset = 0;
  for (const auto& block : window.blocks) {
    const Eigen::Index block_rows = block.residual_raw.size();
    mode.raw_group_maps[block.group_id] =
        state_direction.segment(row_offset, block_rows);
    row_offset += block_rows;
  }
  ASSERT_EQ(row_offset, rows);
  const std::vector<FaultModeBasis> modes{mode};
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(1);
  hypothesis.modes = {mode.id};
  hypothesis.prior_probability_bound = 1e-4;
  hypothesis.p_md_allocation = 1e-3;
  std::vector<FaultHypothesisV2> hypotheses{hypothesis};
  std::shared_ptr<const FrozenHypothesisNumerics> shared;
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      window, modes, &hypotheses, 100.0, &shared);
  ASSERT_EQ(evidence.size(), 1u);
  ASSERT_TRUE(shared);
  ASSERT_EQ(shared->pl_entries.size(), 1u);
  const auto& entry = shared->pl_entries[0];
  EXPECT_EQ(entry.z_classification, 3) << entry.z_smallest_singular_value;
  EXPECT_FALSE(hypotheses[0].monitored);
  EXPECT_FALSE(evidence[0].plausible);
  const std::string reason = hypotheses[0].monitorability.reason;
  EXPECT_NE(reason.find("dangerous detection nullspace"), std::string::npos)
      << reason;
  EXPECT_NE(reason.find("axis residuals"), std::string::npos) << reason;
  EXPECT_NE(reason.find("modes 1"), std::string::npos) << reason;
  EXPECT_FALSE(entry.valid);
}

TEST(B3ZeroSpace, HarmlessNullspaceKeepsAFiniteProjectedBound) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  // Degenerate harmless limit expressible on the two-state synthetic fixture:
  // the protected map is full column rank there, so the only direction that is
  // both invisible and harmless is the zero response.  The classification must
  // still return the harmless branch with a finite (zero) projected bound
  // instead of rejecting the hypothesis or reporting a dangerous nullspace.
  FaultModeBasis mode;
  mode.id = FaultModeId(1);
  mode.sensor = SensorType::Uwb;
  mode.parameter_dimension = 1;
  for (const auto& block : window.blocks) {
    mode.raw_group_maps[block.group_id] =
        Eigen::VectorXd::Zero(block.residual_raw.size());
  }
  const std::vector<FaultModeBasis> modes{mode};
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(1);
  hypothesis.modes = {mode.id};
  hypothesis.prior_probability_bound = 1e-4;
  hypothesis.p_md_allocation = 1e-3;
  std::vector<FaultHypothesisV2> hypotheses{hypothesis};
  std::shared_ptr<const FrozenHypothesisNumerics> shared;
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      window, modes, &hypotheses, 100.0, &shared);
  ASSERT_EQ(evidence.size(), 1u);
  ASSERT_TRUE(shared);
  const auto& entry = shared->pl_entries[0];
  EXPECT_EQ(entry.z_classification, 2) << entry.z_smallest_singular_value;
  EXPECT_TRUE(entry.bound_from_projected_path);
  EXPECT_TRUE(entry.monitorability.monitorable);
  EXPECT_TRUE(entry.protected_slopes.allFinite());
  EXPECT_TRUE(entry.protected_slopes.isZero(1e-12))
      << entry.protected_slopes.transpose();
  EXPECT_TRUE(hypotheses[0].monitored);
}

// ---------------------------------------------------------------------------
// Stage 0 (C-round): physical attribution of a step-gate rejection.  The gate
// logic and thresholds are untouched; only the exported reason grows.
// ---------------------------------------------------------------------------
TEST(GateAttribution, StepAttributionNamesPhysicalBlocksAndDominantEpoch) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  StateLayoutEntry old_epoch;
  old_epoch.epoch = 3;
  old_epoch.column_offset = 0;
  old_epoch.dimension = 15;
  StateLayoutEntry current;
  current.epoch = 4;
  current.column_offset = 15;
  current.dimension = 15;
  current.protected_current_state = true;
  window.state_layout = {old_epoch, current};
  Eigen::VectorXd increment = Eigen::VectorXd::Zero(30);
  increment.segment(0, 3) = Eigen::Vector3d::Constant(0.1);    // rotation
  increment.segment(3, 3) = Eigen::Vector3d::Constant(0.2);    // position
  increment.segment(6, 3) = Eigen::Vector3d::Constant(0.05);   // velocity
  increment.segment(9, 3) = Eigen::Vector3d::Constant(0.01);   // accel bias
  increment.segment(12, 3) = Eigen::Vector3d::Constant(0.0);   // gyro bias
  increment.segment(15, 3) = Eigen::Vector3d::Constant(0.03);  // rotation
  increment.segment(18, 3) = Eigen::Vector3d::Constant(0.9);   // position
  increment.segment(21, 3) = Eigen::Vector3d::Constant(0.4);   // velocity
  increment.segment(24, 3) = Eigen::Vector3d::Constant(0.0);
  increment.segment(27, 3) = Eigen::Vector3d::Constant(0.0);
  const std::string attribution = stateStepAttribution(window, increment);
  EXPECT_NE(attribution.find("step attribution"), std::string::npos)
      << attribution;
  EXPECT_NE(attribution.find("rotation="), std::string::npos) << attribution;
  EXPECT_NE(attribution.find("position="), std::string::npos) << attribution;
  EXPECT_NE(attribution.find("velocity="), std::string::npos) << attribution;
  EXPECT_NE(attribution.find("accel_bias="), std::string::npos) << attribution;
  EXPECT_NE(attribution.find("gyro_bias="), std::string::npos) << attribution;
  // The dominant block is the current-epoch position (0.9 * sqrt(3)).
  EXPECT_NE(attribution.find("dominant=position(epoch 4)"), std::string::npos)
      << attribution;
  EXPECT_EQ(stateStepAttribution(window, Eigen::VectorXd()),
            "step attribution: empty increment");
}

// ---------------------------------------------------------------------------
// P7/S0-2: narrow-band regression for the one-sided relative dominance rule.
// The old absolute rule (env + tol >= leaf) accepted a 1e-7 relative shortfall;
// the relative rule must reject it while keeping binary64 noise accepted.
// ---------------------------------------------------------------------------
TEST(EnvelopeDominance, OneSidedRelativeRuleRejectsNarrowBandShortfall) {
  using namespace uwb_imu_pl;
  const double leaf = 0.37;
  const double tolerance = 1e-9;
  // 1e-7 relative shortfall: rejected (this is the case the old absolute rule
  // wrongly accepted).
  const double shortfall = leaf * (1.0 - 1e-7);
  EXPECT_FALSE(envelopeDominanceAccepts(shortfall, leaf, tolerance));
  EXPECT_LT(envelopeDominanceMargin(shortfall, leaf), -1e-8);
  EXPECT_LT(envelopeDominanceMargin(shortfall, leaf), -tolerance);
  // 1e-10 relative shortfall: inside the binary64 noise floor: accepted, with a
  // raw margin >= -tolerance.
  const double noise = leaf * (1.0 - 1e-10);
  EXPECT_TRUE(envelopeDominanceAccepts(noise, leaf, tolerance));
  const double noise_margin = envelopeDominanceMargin(noise, leaf);
  EXPECT_GE(noise_margin, -tolerance);
  EXPECT_LT(noise_margin, 0.0);
  // Exact and dominant envelopes.
  EXPECT_TRUE(envelopeDominanceAccepts(leaf, leaf, tolerance));
  EXPECT_GE(envelopeDominanceMargin(leaf, leaf), 0.0);
  EXPECT_TRUE(envelopeDominanceAccepts(2.0 * leaf, leaf, tolerance));
  EXPECT_NEAR(envelopeDominanceMargin(2.0 * leaf, leaf), 1.0, 1e-15);
  // Degenerate leaf (no monitorable direction) is trivially dominated; a
  // non-finite input is never accepted.
  EXPECT_TRUE(envelopeDominanceAccepts(0.0, 0.0, tolerance));
  EXPECT_FALSE(envelopeDominanceAccepts(
      std::numeric_limits<double>::quiet_NaN(), leaf, tolerance));
  EXPECT_FALSE(envelopeDominanceAccepts(std::numeric_limits<double>::infinity(),
                                        leaf, tolerance));

  // The same rule drives the builder: a scaled-group envelope (exactly equal
  // bounds) is accepted with a margin inside the noise floor.
  auto fixture = makeCompactFixture();
  FaultModeBasis group;
  group.id = FaultModeId(10);
  group.sensor = SensorType::Uwb;
  group.parameter_dimension = 1;
  group.raw_group_maps[FactorGroupId(2)] = Eigen::Vector3d::Constant(0.5);
  const auto certificate = buildGroupedCoverageCertificate(
      fixture.window, {fixture.modes[0]}, {group});
  ASSERT_EQ(certificate.envelopes.size(), 1u);
  const auto& envelope = certificate.envelopes.front();
  EXPECT_TRUE(envelope.accepted) << envelope.reason;
  EXPECT_GE(envelope.dominance_margin, -1e-9);
  EXPECT_GE(envelope.dominance_ratio, 1.0 - 1e-9);
  EXPECT_TRUE(envelope.dominant);
}

// ---------------------------------------------------------------------------
// P1-01/R07: immutable identity reuse, descriptor indices, exact operation
// deduplication and batched unique-block RHS.  The legacy identity ablation is
// retained so this test proves numerical equivalence while exposing the work
// removed by the optimized path.
// ---------------------------------------------------------------------------
TEST(P101IdentityIndexing, FrozenProofAndDirectDescriptorsAreExact) {
  using namespace uwb_imu_pl;
  auto optimized = makeCompactFixture();
  NumericalWorkCounters::reset();
  unsetenv("UWB_IMU_PL_DISABLE_FROZEN_IDENTITY_HANDLE");
  const FrozenIntegrityWindow optimized_handle =
      freezeIntegrityWindowCopy(optimized.window);
  const FrozenWindowAdmission optimized_admission =
      admitFrozenIntegrityWindow(optimized_handle);
  ASSERT_TRUE(optimized_admission) << optimized_admission.reason;
  std::shared_ptr<const FrozenHypothesisNumerics> optimized_shared;
  const auto optimized_evidence = HypothesisEvidenceEvaluator().evaluateAll(
      optimized_admission, optimized.modes, &optimized.hypotheses, 100.0,
      &optimized_shared);
  const auto optimized_work = NumericalWorkCounters::snapshot();

  auto legacy = makeCompactFixture();
  NumericalWorkCounters::reset();
  ASSERT_EQ(setenv("UWB_IMU_PL_DISABLE_FROZEN_IDENTITY_HANDLE", "1", 1), 0);
  std::shared_ptr<const FrozenHypothesisNumerics> legacy_shared;
  const auto legacy_evidence = HypothesisEvidenceEvaluator().evaluateAll(
      legacy.window, legacy.modes, &legacy.hypotheses, 100.0, &legacy_shared);
  unsetenv("UWB_IMU_PL_DISABLE_FROZEN_IDENTITY_HANDLE");
  const auto legacy_work = NumericalWorkCounters::snapshot();

  ASSERT_EQ(optimized_evidence.size(), legacy_evidence.size());
  ASSERT_TRUE(optimized_shared);
  ASSERT_TRUE(legacy_shared);
  ASSERT_EQ(optimized_shared->pl_entries.size(),
            legacy_shared->pl_entries.size());
  for (std::size_t index = 0; index < optimized_evidence.size(); ++index) {
    EXPECT_EQ(optimized_evidence[index].plausible,
              legacy_evidence[index].plausible);
    EXPECT_EQ(optimized_evidence[index].log_evidence,
              legacy_evidence[index].log_evidence);
    EXPECT_EQ(optimized_evidence[index].all_in_statistic,
              legacy_evidence[index].all_in_statistic);
    EXPECT_EQ(optimized_evidence[index].conditioned_statistic,
              legacy_evidence[index].conditioned_statistic);
    EXPECT_EQ(optimized_evidence[index].explained_energy,
              legacy_evidence[index].explained_energy);
    EXPECT_EQ(optimized_evidence[index].profile_j,
              legacy_evidence[index].profile_j);
    EXPECT_EQ(optimized_evidence[index].profile_valid,
              legacy_evidence[index].profile_valid);
    EXPECT_EQ(optimized_evidence[index].monitorability.rank,
              legacy_evidence[index].monitorability.rank);
    EXPECT_EQ(optimized_evidence[index].monitorability.condition_number,
              legacy_evidence[index].monitorability.condition_number);
    EXPECT_EQ(optimized_evidence[index].monitorability.monitorable,
              legacy_evidence[index].monitorability.monitorable);
    EXPECT_TRUE((optimized_evidence[index].estimated_fault.array() ==
                 legacy_evidence[index].estimated_fault.array()).all());
    EXPECT_TRUE((optimized_evidence[index].fault_gram.array() ==
                 legacy_evidence[index].fault_gram.array()).all());
    FrozenHypothesisPlProofV1 optimized_proof;
    FrozenHypothesisPlProofV1 legacy_proof;
    ASSERT_TRUE(frozenHypothesisPlProof(
        optimized_shared->pl_entries[index], &optimized_proof));
    ASSERT_TRUE(frozenHypothesisPlProof(
        legacy_shared->pl_entries[index], &legacy_proof));
    EXPECT_EQ(optimized_proof.proof_identity, legacy_proof.proof_identity);
    EXPECT_EQ(optimized_proof.parent_proof_identity,
              legacy_proof.parent_proof_identity);
    EXPECT_EQ(optimized_proof.nullspace_class,
              legacy_proof.nullspace_class);
    EXPECT_TRUE((optimized_proof.certified_gram.array() ==
                 legacy_proof.certified_gram.array()).all());
    EXPECT_TRUE((optimized_proof.protected_response.array() ==
                 legacy_proof.protected_response.array()).all());
    EXPECT_TRUE((optimized_proof.gram_eigenvalues.array() ==
                 legacy_proof.gram_eigenvalues.array()).all());
    EXPECT_TRUE((optimized_proof.gram_eigenvectors.array() ==
                 legacy_proof.gram_eigenvectors.array()).all());
    EXPECT_TRUE((optimized_proof.gram_eigenvalue_errors.array() ==
                 legacy_proof.gram_eigenvalue_errors.array()).all());
    EXPECT_TRUE((optimized_proof.nullspace_axis_residual.array() ==
                 legacy_proof.nullspace_axis_residual.array()).all());
    EXPECT_TRUE((optimized_proof.raw_detection_factor.array() ==
                 legacy_proof.raw_detection_factor.array()).all());
    EXPECT_EQ(optimized_proof.raw_factor_scale,
              legacy_proof.raw_factor_scale);
    EXPECT_TRUE((optimized_shared->pl_entries[index].protected_slopes.array() ==
                 legacy_shared->pl_entries[index].protected_slopes.array())
                    .all());
  }
  EXPECT_EQ(optimized_work.frozen_identity_builds, 0u);
  EXPECT_EQ(optimized_work.frozen_identity_reuses,
            optimized.hypotheses.size());
  EXPECT_EQ(legacy_work.frozen_identity_builds, legacy.hypotheses.size());
  EXPECT_EQ(legacy_work.frozen_identity_reuses, 0u);
  EXPECT_EQ(optimized_work.descriptor_id_lookups, 3u);
  EXPECT_EQ(optimized_work.descriptor_linear_scans, 0u);
  EXPECT_EQ(optimized_work.window_content_hash_scans, 1u);
}

TEST(P101IdentityIndexing, UniqueBlockRhsUsesOneBatch) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  ExclusionAction action;
  action.id = ExclusionActionId(101);
  action.action_model_id = "P1_01_TWO_BLOCK_REMOVAL";
  action.groups_to_remove = {FactorGroupId(1), FactorGroupId(2)};
  action.exclusion_cardinality = 2;
  NumericalWorkCounters::reset();
  const auto base = RankUpdateEvaluator().factorizeOnce(window, {action});
  const auto work = NumericalWorkCounters::snapshot();
  ASSERT_TRUE(base.valid) << base.reason;
  ASSERT_TRUE(base.block_cache);
  EXPECT_EQ(base.block_cache->blocks.size(), 2u);
  EXPECT_EQ(work.block_rhs_solve_batches, 1u);
  EXPECT_EQ(work.block_rhs_unique_blocks, 2u);
  EXPECT_EQ(work.square_root_information_solves, 1u);
  EXPECT_EQ(work.square_root_information_columns, 5u);
  EXPECT_EQ(work.covariance_rhs_solves, 0u);
  for (const auto& group : base.block_cache->blocks) {
    ASSERT_EQ(group.second.size(), 1u);
    const auto& entry = group.second.front();
    EXPECT_EQ(entry.base_solve.rows(), window.H.cols());
    EXPECT_EQ(entry.base_solve.cols(),
              entry.block.jacobian_whitened.rows());
    EXPECT_TRUE(entry.base_solve.allFinite());
  }
}

TEST(P101IdentityIndexing, BatchedRhsMatchesUncachedCandidateExactly) {
  using namespace uwb_imu_pl;
  auto window = syntheticWindow();
  ExclusionAction action;
  action.id = ExclusionActionId(102);
  action.action_model_id = "P1_01_REPLACEMENT";
  action.groups_to_remove = {FactorGroupId(1)};
  action.groups_to_add = {FactorGroupId(1)};
  action.added_blocks = {window.blocks.front()};
  action.added_blocks.front().residual_whitened(0) += 0.01;
  action.added_blocks.front().residual_raw(0) += 0.01;
  action.exclusion_cardinality = 1;

  unsetenv("UWB_IMU_PL_DISABLE_BLOCK_CACHE");
  RankUpdateEvaluator evaluator;
  const auto batched_base = evaluator.factorizeOnce(window, {action});
  const auto batched = evaluator.evaluate(batched_base, action);
  ASSERT_TRUE(batched.valid) << batched.reason;
  ASSERT_EQ(setenv("UWB_IMU_PL_DISABLE_BLOCK_CACHE", "1", 1), 0);
  const auto uncached_base = evaluator.factorizeOnce(window, {action});
  const auto uncached = evaluator.evaluate(uncached_base, action);
  unsetenv("UWB_IMU_PL_DISABLE_BLOCK_CACHE");
  ASSERT_TRUE(uncached.valid) << uncached.reason;
  EXPECT_TRUE((batched.state_increment.array() ==
               uncached.state_increment.array()).all());
  EXPECT_TRUE((batched.covariance.array() == uncached.covariance.array()).all());
  EXPECT_EQ(batched.statistic, uncached.statistic);
  EXPECT_EQ(batched.rank, uncached.rank);
  EXPECT_EQ(batched.dof, uncached.dof);
  EXPECT_EQ(batched.detector_certificate.certificate_digest,
            uncached.detector_certificate.certificate_digest);
}

TEST(P101IdentityIndexing, ExactOperationDedupKeepsSemanticCoverage) {
  using namespace uwb_imu_pl;
  ExclusionAction first;
  first.id = ExclusionActionId(201);
  first.action_model_id = "EXACT";
  first.groups_to_remove = {FactorGroupId(3), FactorGroupId(1)};
  first.covered_modes = {FaultModeId(8)};
  first.exclusion_cardinality = 2;
  ExclusionAction duplicate = first;
  const auto result = censusAndCapActionsV1({first, duplicate}, 2);
  ASSERT_EQ(result.actions.size(), 1u);
  ASSERT_EQ(result.census.omitted_actions.size(), 1u);
  EXPECT_TRUE(result.census.omitted_actions.front().proven_safe);
  EXPECT_EQ(result.census.omitted_actions.front().reason,
            "EXACT_SEMANTIC_DUPLICATE");
  EXPECT_EQ(result.census.generated, 2u);
  EXPECT_EQ(result.census.evaluated, 1u);
  EXPECT_EQ(result.census.omitted, 1u);
  EXPECT_TRUE(result.census.exhaustive);
}

TEST(P101IdentityIndexing, LegacyV1V2DuplicateAddedGroupBehaviorIsPreserved) {
  using namespace uwb_imu_pl;
  ExclusionAction action;
  action.id = ExclusionActionId(202);
  action.action_model_id = "LEGACY_DUPLICATE_ADDED_GROUP";
  const auto window = syntheticWindow();
  action.groups_to_add = {window.blocks.front().group_id,
                          window.blocks.front().group_id};
  action.added_blocks = {window.blocks.front(), window.blocks.front()};
  action.exclusion_cardinality = 0;

  // golden-p1-05 treats this finite operation as representable.  The V3
  // storage sidecar must not retroactively make V1/V2 search incomplete.
  EXPECT_FALSE(exactActionOperationIdentityV1(action).empty());
  const auto result = censusAndCapActionsV1({action}, 1);
  ASSERT_EQ(result.actions.size(), 1u);
  EXPECT_TRUE(result.census.exhaustive);
  EXPECT_EQ(result.census.omitted, 0u);
  EXPECT_TRUE(validateActionSearchCensusV1(
      result.census, result.generated_snapshot, result.max_evaluated_actions,
      result.lifecycle, result.actions).valid);
}

TEST(P101IdentityIndexing, FrozenOwnerIsImmutableAndAdmissionIsConstantWork) {
  using namespace uwb_imu_pl;
  auto builder = syntheticWindow();
  const Eigen::MatrixXd expected_h = builder.H;
  const Eigen::VectorXd expected_z = builder.z;
  NumericalWorkCounters::reset();
  auto frozen = freezeIntegrityWindowCopy(builder);
  const auto after_freeze = NumericalWorkCounters::snapshot();
  ASSERT_TRUE(frozen.seal);
  EXPECT_EQ(after_freeze.window_content_hash_scans, 1u);

  builder.H.setConstant(1234.0);
  builder.z.setConstant(-5678.0);
  const auto admission = admitFrozenIntegrityWindow(frozen);
  ASSERT_TRUE(admission) << admission.reason;
  EXPECT_TRUE((admission.window().H.array() == expected_h.array()).all());
  EXPECT_TRUE((admission.window().z.array() == expected_z.array()).all());
  const auto after_admission = NumericalWorkCounters::snapshot();
  EXPECT_EQ(after_admission.window_content_hash_scans, 1u);
  EXPECT_EQ(after_admission.frozen_admission_constant_validations, 1u);
}

TEST(P101IdentityIndexing,
     ImmutableInvalidWindowWithoutNumericsIsAdmittedOnlyToFailClosed) {
  using namespace uwb_imu_pl;
  auto builder = syntheticWindow();
  builder.model_valid = false;
  builder.reason = "history summary invalid: non_finite_input";
  builder.numerics.reset();
  const auto frozen = freezeIntegrityWindowCopy(builder);
  const auto admission = admitFrozenIntegrityWindow(frozen);
  ASSERT_TRUE(admission) << admission.reason;
  EXPECT_FALSE(admission.window().model_valid);
  EXPECT_FALSE(admission.window().numerics);
  const auto detector = JointWindowDetector().evaluate(
      admission, DetectorRiskContext{});
  EXPECT_FALSE(detector.numerically_valid);
  EXPECT_FALSE(detector.passed);
  EXPECT_NE(detector.reason.find("invalid integrity window"),
            std::string::npos) << detector.reason;

  builder.model_valid = true;
  const auto forged_valid = freezeIntegrityWindowCopy(builder);
  const auto rejected = admitFrozenIntegrityWindow(forged_valid);
  EXPECT_FALSE(rejected);
  EXPECT_NE(rejected.reason.find("no completed numerics"), std::string::npos)
      << rejected.reason;
}

TEST(P101IdentityIndexing, SealedCandidateIsBoundToItsExactOwner) {
  using namespace uwb_imu_pl;
  NumericalWorkCounters::reset();
  const auto seal_a = freezeIntegrityWindowCopy(syntheticWindow());
  const auto seal_b = freezeIntegrityWindowCopy(syntheticWindow());
  const auto admission_a = admitFrozenIntegrityWindow(seal_a);
  const auto admission_b = admitFrozenIntegrityWindow(seal_b);
  ASSERT_TRUE(admission_a);
  ASSERT_TRUE(admission_b);
  ASSERT_NE(&admission_a.window(), &admission_b.window());
  ASSERT_EQ(seal_a.seal->content_hash, seal_b.seal->content_hash);

  ExclusionAction keep;
  keep.id = ExclusionActionId(10101);
  keep.action_model_id = "P1_01_OWNER_BOUND_KEEP";
  RankUpdateEvaluator evaluator;
  const auto base = evaluator.factorizeOnce(admission_a, {keep});
  const auto candidate = evaluator.evaluate(admission_a, base, keep);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  const auto before_validation = NumericalWorkCounters::snapshot();
  std::string reason;
  EXPECT_TRUE(validateCandidateDetectorCertificate(
      admission_a, candidate, &reason)) << reason;
  EXPECT_FALSE(validateCandidateDetectorCertificate(
      admission_b, candidate, &reason));
  EXPECT_NE(reason.find("source window owner/payload identity mismatch"),
            std::string::npos)
      << reason;
  const auto rejected = JointWindowDetector().evaluateCandidate(
      admission_b, candidate, DetectorRiskContext{});
  EXPECT_FALSE(rejected.numerically_valid);
  EXPECT_NE(rejected.reason.find("source window owner/payload identity mismatch"),
            std::string::npos) << rejected.reason;
  const auto after_validation = NumericalWorkCounters::snapshot();
  EXPECT_EQ(after_validation.window_content_hash_scans,
            before_validation.window_content_hash_scans);
}

TEST(P101IdentityIndexing,
     ForcedHashCollisionTransplantIsRejectedByRealSealedConsumer) {
  using namespace uwb_imu_pl;
  NumericalWorkCounters::reset();
  const auto seal_a = freezeIntegrityWindowCopy(syntheticWindow());
  const auto admission_a = admitFrozenIntegrityWindow(seal_a);
  ASSERT_TRUE(admission_a);
  ExclusionAction keep;
  keep.id = ExclusionActionId(10102);
  keep.action_model_id = "P1_01_COLLISION_KEEP";
  RankUpdateEvaluator evaluator;
  const auto base = evaluator.factorizeOnce(admission_a, {keep});
  const auto candidate = evaluator.evaluate(admission_a, base, keep);
  ASSERT_TRUE(candidate.valid) << candidate.reason;

  auto different = syntheticWindow();
  different.blocks.front().residual_whitened(0) = std::nextafter(
      different.blocks.front().residual_whitened(0), 1.0);
  different.blocks.front().residual_raw(0) =
      different.blocks.front().residual_whitened(0);
  finalizeIntegrityWindow(&different, 1e-10, 1e10);
  const auto ordinary_b = freezeIntegrityWindowCopy(different);
  auto forced_payload = std::make_shared<LinearizedIntegrityWindow>(
      *ordinary_b.seal->payload);
  auto forced_numerics = std::make_shared<FrozenWindowNumerics>(
      *forced_payload->numerics);
  forced_numerics->content_fingerprint = seal_a.seal->content_hash;
  forced_payload->numerics = forced_numerics;
  auto forced_seal = std::make_shared<FrozenIntegrityWindowSeal>(
      *ordinary_b.seal);
  forced_seal->content_hash = seal_a.seal->content_hash;
  forced_seal->payload = forced_payload;
  forced_seal->numerical_identity.content_fingerprint =
      seal_a.seal->content_hash;
  forced_seal->numerical_identity.proof_identity =
      frozenWindowNumericalProofIdentity(*forced_payload, *forced_numerics);
  forced_seal->numerical_identity.valid = true;
  const FrozenIntegrityWindow forged_b{forced_seal};
  const auto admission_b = admitFrozenIntegrityWindow(forged_b);
  ASSERT_TRUE(admission_b) << admission_b.reason;
  ASSERT_EQ(seal_a.seal->content_hash, forced_seal->content_hash);
  ASSERT_FALSE(frozenWindowCanonicalEqual(*seal_a.seal, *forced_seal));

  const auto before_validation = NumericalWorkCounters::snapshot();
  const auto rejected = JointWindowDetector().evaluateCandidate(
      admission_b, candidate, DetectorRiskContext{});
  EXPECT_FALSE(rejected.numerically_valid);
  EXPECT_NE(rejected.reason.find("source window owner/payload identity mismatch"),
            std::string::npos) << rejected.reason;
  const auto after_validation = NumericalWorkCounters::snapshot();
  EXPECT_EQ(after_validation.window_content_hash_scans,
            before_validation.window_content_hash_scans);
}

TEST(P101IdentityIndexing, ProductionMoveIsolatedFromReusedSourceObject) {
  using namespace uwb_imu_pl;
  auto source = syntheticWindow();
  const Eigen::MatrixXd expected_h = source.H;
  const auto frozen = freezeIntegrityWindow(std::move(source));
  source = syntheticWindow();
  source.H.setConstant(9999.0);
  source.z.setConstant(-9999.0);
  const auto admission = admitFrozenIntegrityWindow(frozen);
  ASSERT_TRUE(admission) << admission.reason;
  EXPECT_TRUE((admission.window().H.array() == expected_h.array()).all());
  ExclusionAction keep;
  keep.id = ExclusionActionId(10103);
  keep.action_model_id = "P1_01_MOVE_KEEP";
  RankUpdateEvaluator evaluator;
  const auto base = evaluator.factorizeOnce(admission, {keep});
  const auto candidate = evaluator.evaluate(admission, base, keep);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  EXPECT_TRUE(validateCandidateDetectorCertificate(admission, candidate));
}

TEST(P101IdentityIndexing, ReusedPayloadAddressCannotInheritOldOwner) {
  using namespace uwb_imu_pl;
  alignas(LinearizedIntegrityWindow)
      unsigned char storage[sizeof(LinearizedIntegrityWindow)];
  auto make_at_storage = [&](LinearizedIntegrityWindow value) {
    auto* payload = new (storage) LinearizedIntegrityWindow(std::move(value));
    return std::shared_ptr<const LinearizedIntegrityWindow>(
        payload, [](const LinearizedIntegrityWindow* pointer) {
          const_cast<LinearizedIntegrityWindow*>(pointer)
              ->~LinearizedIntegrityWindow();
        });
  };
  auto bind_payload = [&](const FrozenIntegrityWindow& minted,
                          std::shared_ptr<const LinearizedIntegrityWindow> payload) {
    auto seal = std::make_shared<FrozenIntegrityWindowSeal>(*minted.seal);
    seal->payload = std::move(payload);
    seal->content_hash = seal->payload->numerics->content_fingerprint;
    seal->numerical_identity.window_id = seal->payload->id;
    seal->numerical_identity.version = seal->payload->version;
    seal->numerical_identity.content_fingerprint = seal->content_hash;
    seal->numerical_identity.numerical_contract_fingerprint =
        seal->payload->numerics->numerical_contract_fingerprint;
    seal->numerical_identity.proof_identity = frozenWindowNumericalProofIdentity(
        *seal->payload, *seal->payload->numerics);
    seal->numerical_identity.valid = true;
    return FrozenIntegrityWindow{std::move(seal)};
  };

  CandidateEvaluation held_candidate;
  std::weak_ptr<const FrozenIntegrityWindowSeal> old_owner_lifetime;
  std::uint64_t old_owner = 0;
  const void* reused_address = nullptr;
  {
    auto first_payload = make_at_storage(syntheticWindow());
    reused_address = first_payload.get();
    const auto first_minted = freezeIntegrityWindowCopy(*first_payload);
    auto first = bind_payload(first_minted, std::move(first_payload));
    auto first_admission = admitFrozenIntegrityWindow(first);
    ASSERT_TRUE(first_admission);
    ExclusionAction keep;
    keep.id = ExclusionActionId(10104);
    keep.action_model_id = "P1_01_ADDRESS_REUSE_KEEP";
    RankUpdateEvaluator evaluator;
    const auto base = evaluator.factorizeOnce(first_admission, {keep});
    held_candidate = evaluator.evaluate(first_admission, base, keep);
    ASSERT_TRUE(held_candidate.valid) << held_candidate.reason;
    old_owner = first_admission.ownerToken();
    old_owner_lifetime = first.seal;
    first_admission = FrozenWindowAdmission{};
    first = FrozenIntegrityWindow{};
    // The candidate aliases the seal's control block, so an allocator cannot
    // reuse the payload address while a stale candidate still exists.
    EXPECT_FALSE(old_owner_lifetime.expired());
  }
  EXPECT_FALSE(old_owner_lifetime.expired());
  held_candidate = CandidateEvaluation{};
  EXPECT_TRUE(old_owner_lifetime.expired());

  auto second_value = syntheticWindow();
  auto second_payload = make_at_storage(std::move(second_value));
  ASSERT_EQ(reused_address, second_payload.get());
  const auto second_minted = freezeIntegrityWindowCopy(*second_payload);
  auto second = bind_payload(second_minted, std::move(second_payload));
  const auto second_admission = admitFrozenIntegrityWindow(second);
  ASSERT_TRUE(second_admission);
  ASSERT_NE(old_owner, second_admission.ownerToken());
  ASSERT_EQ(reused_address,
            static_cast<const void*>(&second_admission.window()));
  ExclusionAction keep;
  keep.id = ExclusionActionId(10104);
  keep.action_model_id = "P1_01_ADDRESS_REUSE_KEEP";
  RankUpdateEvaluator evaluator;
  const auto base = evaluator.factorizeOnce(second_admission, {keep});
  const auto new_candidate = evaluator.evaluate(second_admission, base, keep);
  ASSERT_TRUE(new_candidate.valid) << new_candidate.reason;
  EXPECT_TRUE(validateCandidateDetectorCertificate(
      second_admission, new_candidate));
}

TEST(P101IdentityIndexing, ForcedHashCollisionRequiresCanonicalEquality) {
  using namespace uwb_imu_pl;
  auto left_builder = syntheticWindow();
  auto right_builder = syntheticWindow();
  ASSERT_FALSE(right_builder.blocks.empty());
  right_builder.blocks.front().residual_whitened(0) = std::nextafter(
      right_builder.blocks.front().residual_whitened(0), 1.0);
  right_builder.blocks.front().residual_raw(0) = std::nextafter(
      right_builder.blocks.front().residual_raw(0), 1.0);
  finalizeIntegrityWindow(&right_builder, 1e-10, 1e10);
  auto left = freezeIntegrityWindowCopy(left_builder);
  auto right = freezeIntegrityWindowCopy(right_builder);
  ASSERT_TRUE(left.seal);
  ASSERT_TRUE(right.seal);
  FrozenIntegrityWindowSeal forced_left = *left.seal;
  FrozenIntegrityWindowSeal forced_right = *right.seal;
  forced_left.content_hash = 0x123456789abcdef0ULL;
  forced_right.content_hash = forced_left.content_hash;
  EXPECT_FALSE(frozenWindowCanonicalEqual(forced_left, forced_right));

  FrozenIntegrityWindowSeal same_payload = forced_left;
  EXPECT_TRUE(frozenWindowCanonicalEqual(forced_left, same_payload));
}

TEST(P101IdentityIndexing, OldHandleCannotAdmitNewObjectOrAddressReuse) {
  using namespace uwb_imu_pl;
  auto first = freezeIntegrityWindowCopy(syntheticWindow());
  const auto first_admission = admitFrozenIntegrityWindow(first);
  ASSERT_TRUE(first_admission);
  auto second_builder = syntheticWindow();
  second_builder.id = WindowId(first_admission.window().id.value() + 1);
  finalizeIntegrityWindow(&second_builder, 1e-10, 1e10);
  auto second = freezeIntegrityWindowCopy(second_builder);
  const auto old_admission = admitFrozenIntegrityWindow(first);
  const auto new_admission = admitFrozenIntegrityWindow(second);
  ASSERT_TRUE(old_admission);
  ASSERT_TRUE(new_admission);
  EXPECT_NE(old_admission.ownerToken(), new_admission.ownerToken());
  EXPECT_FALSE(frozenWindowHandleMatches(old_admission, second));
  EXPECT_FALSE(frozenWindowHandleMatches(new_admission, first));

  const std::uint64_t expired_token = old_admission.ownerToken();
  first = FrozenIntegrityWindow{};
  EXPECT_NE(expired_token, new_admission.ownerToken());
  EXPECT_TRUE((new_admission.window().H.array() ==
               second.seal->payload->H.array()).all());
}

TEST(P101IdentityIndexing, CopyMoveLifetimeAndConcurrentAdmissionAreStable) {
  using namespace uwb_imu_pl;
  auto frozen = freezeIntegrityWindowCopy(syntheticWindow());
  auto copied = frozen;
  auto moved = std::move(copied);
  const auto baseline = admitFrozenIntegrityWindow(moved);
  ASSERT_TRUE(baseline);
  constexpr int kReaders = 16;
  std::vector<std::future<std::pair<std::uint64_t, const void*>>> readers;
  readers.reserve(kReaders);
  for (int i = 0; i < kReaders; ++i) {
    readers.emplace_back(std::async(std::launch::async, [&moved]()
        -> std::pair<std::uint64_t, const void*> {
      const auto admitted = admitFrozenIntegrityWindow(moved);
      if (!admitted) return {0, nullptr};
      return std::make_pair(admitted.ownerToken(),
                            static_cast<const void*>(&admitted.window()));
    }));
  }
  for (auto& reader : readers) {
    const auto result = reader.get();
    EXPECT_EQ(result.first, baseline.ownerToken());
    EXPECT_EQ(result.second, static_cast<const void*>(&baseline.window()));
  }
  frozen = FrozenIntegrityWindow{};
  EXPECT_TRUE(admitFrozenIntegrityWindow(moved));
}

TEST(P101IdentityIndexing, FrozenClockMakesTerminalDecisionExact) {
  using namespace uwb_imu_pl;
  const auto config = researchConfig();
  IncrementalUwbImuEstimator left_estimator(config, Eigen::Vector3d::Zero());
  IncrementalUwbImuEstimator right_estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0.0, 0.0, 1.0};
  left_estimator.initialize(initial, config.realtime.prior_sigmas);
  right_estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&left_estimator, config.imu.gravity_mps2);
  addImu(&right_estimator, config.imu.gravity_mps2);
  RealtimeIntegrityPipeline left(
      &left_estimator,
      IntegrityMonitor(config.risk, config.snapshot.rank_tolerance,
                       config.snapshot.max_condition_number));
  RealtimeIntegrityPipeline right(
      &right_estimator,
      IntegrityMonitor(config.risk, config.snapshot.rank_tolerance,
                       config.snapshot.max_condition_number));
  const UwbBatch input = batch(config, 10000000);
  ClockSample frozen_clock;
  frozen_clock.wall_monotonic_ns = 10000000;
  frozen_clock.sensor_timestamp_ns = input.timestamp.value();
  const auto left_output = left.processUwbBatchWithFrozenFinishElapsed(
      input, frozen_clock, 0.0);
  const auto right_output = right.processUwbBatchWithFrozenFinishElapsed(
      input, frozen_clock, 0.0);
  EXPECT_EQ(left_output.batch_committed, right_output.batch_committed);
  EXPECT_EQ(left_output.fde_status, right_output.fde_status);
  EXPECT_EQ(left_output.reason_codes, right_output.reason_codes);
  EXPECT_EQ(left_output.protection_level.formal_eligible,
            right_output.protection_level.formal_eligible);
  EXPECT_EQ(left_output.protection_level.reason,
            right_output.protection_level.reason);
  EXPECT_EQ(left_output.publication.protected_output,
            right_output.publication.protected_output);
  EXPECT_EQ(left_output.publication.unprotected_output,
            right_output.publication.unprotected_output);
  EXPECT_EQ(left_output.publication.wall_timeout,
            right_output.publication.wall_timeout);
  EXPECT_EQ(left_output.publication.refusal,
            right_output.publication.refusal);
  const auto exact_nonfinite = [](double lhs, double rhs) {
    return (std::isnan(lhs) && std::isnan(rhs)) || lhs == rhs;
  };
  EXPECT_TRUE(exact_nonfinite(left_output.global_detector.statistic,
                              right_output.global_detector.statistic));
  EXPECT_TRUE(exact_nonfinite(left_output.postfit_detector.statistic,
                              right_output.postfit_detector.statistic));
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_TRUE(exact_nonfinite(left_output.protection_level.pl_xyz_m(axis),
                                right_output.protection_level.pl_xyz_m(axis)));
  }
}

TEST(P105FlatConcurrency, WatchdogAdmissionExceptionStillHasOneAttemptTimer) {
  using namespace uwb_imu_pl;
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0.0, 0.0, 1.0};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  addImu(&estimator, config.imu.gravity_mps2);
  RealtimeIntegrityPipeline pipeline(
      &estimator,
      IntegrityMonitor(config.risk, config.snapshot.rank_tolerance,
                       config.snapshot.max_condition_number));
  const UwbBatch input = batch(config, 10000000);
  ClockSample first_clock;
  first_clock.wall_monotonic_ns = 10000000;
  first_clock.sensor_timestamp_ns = input.timestamp.value();
  (void)pipeline.processUwbBatchWithFrozenFinishElapsed(
      input, first_clock, 0.0);

  ClockSample backwards_clock = first_clock;
  backwards_clock.wall_monotonic_ns -= 1;
  EXPECT_ANY_THROW(pipeline.processUwbBatchWithFrozenFinishElapsed(
      input, backwards_clock, 0.0));
  const auto& failed = pipeline.lastAttemptOutput();
  EXPECT_EQ(failed.diagnostics.status, "EXCEPTION");
  EXPECT_NE(failed.diagnostics.reason.find("wall clock moved backwards"),
            std::string::npos);
  const auto core_total_count = static_cast<std::size_t>(std::count_if(
      failed.stage_timings.begin(), failed.stage_timings.end(),
      [](const auto& timing) { return timing.stage == "core_total"; }));
  EXPECT_EQ(core_total_count, 1u);
  const auto core_total =
      std::find_if(failed.stage_timings.begin(), failed.stage_timings.end(),
                   [](const auto& timing) {
                     return timing.stage == "core_total";
                   });
  ASSERT_NE(core_total, failed.stage_timings.end());
  EXPECT_TRUE(std::isfinite(core_total->wall_ms));
  EXPECT_FALSE(core_total->success);
  EXPECT_EQ(core_total->status, "EXCEPTION");
}

// These witnesses certify individual numerical layers. They do not qualify a
// production profile, close its complete risk ledger, or authorize publication.
TEST(Phase2GramReuse, ExactEvidenceAndProofAcrossDimensionsAndRankCases) {
  using namespace uwb_imu_pl;
  const char* key = "UWB_IMU_PL_EXHAUSTIVE_GRAM_CERTIFICATES";
  struct Restore {
    const char* key;
    bool present;
    std::string value;
    ~Restore() { if (present) setenv(key, value.c_str(), 1); else unsetenv(key); }
  } restore{key, std::getenv(key) != nullptr,
            std::getenv(key) ? std::getenv(key) : ""};
  const auto scalar_bits = [](double a, double b) {
    EXPECT_EQ(std::memcmp(&a, &b, sizeof(a)), 0);
  };
  const auto matrix_bits = [&scalar_bits](const Eigen::MatrixXd& a,
                                         const Eigen::MatrixXd& b) {
    ASSERT_EQ(a.rows(), b.rows());
    ASSERT_EQ(a.cols(), b.cols());
    for (Eigen::Index col = 0; col < a.cols(); ++col)
      for (Eigen::Index row = 0; row < a.rows(); ++row)
        scalar_bits(a(row, col), b(row, col));
  };
  const auto monitor_bits = [&](const MonitorabilityResult& a,
                                const MonitorabilityResult& b) {
    EXPECT_EQ(a.rank, b.rank);
    EXPECT_EQ(a.parameter_dimension, b.parameter_dimension);
    EXPECT_EQ(a.physical_parameter_dimension, b.physical_parameter_dimension);
    EXPECT_EQ(a.monitorable, b.monitorable);
    EXPECT_EQ(a.reason, b.reason);
    scalar_bits(a.sigma_min, b.sigma_min);
    scalar_bits(a.sigma_max, b.sigma_max);
    scalar_bits(a.condition_number, b.condition_number);
    matrix_bits(a.protected_slopes, b.protected_slopes);
  };
  std::uint64_t reference_svds = 0, optimized_svds = 0;
  for (const bool fixed : {false, true})
    for (int dimension = 1; dimension <= 4; ++dimension)
      for (int rank_case = 0; rank_case < 4; ++rank_case) {
        SCOPED_TRACE(::testing::Message() << fixed << '/' << dimension << '/' << rank_case);
        auto window = syntheticWindow();
        if (rank_case == 1) window.protected_state_map.setZero();
        finalizeIntegrityWindow(&window, 1e-10, 1e10);
        FaultModeBasis mode;
        mode.id = FaultModeId(1);
        mode.sensor = SensorType::Uwb;
        mode.parameter_dimension = dimension;
        Eigen::Index offset = 0;
        for (const auto& block : window.blocks) {
          Eigen::MatrixXd map(block.residual_raw.size(), dimension);
          for (Eigen::Index row = 0; row < map.rows(); ++row)
            for (int col = 0; col < dimension; ++col)
              map(row, col) = std::sin((offset + row + 1) * (col + 1.3));
          if (rank_case == 1) map.setZero();
          if (rank_case == 2) map.col(dimension - 1) =
              block.jacobian_raw.col(0);  // Invisible but protected direction.
          if (rank_case == 3) map.col(dimension - 1) *= 1e-10;
          mode.raw_group_maps[block.group_id] = map;
          offset += map.rows();
        }
        FaultHypothesisV2 h;
        h.id = HypothesisId(1);
        h.modes = {mode.id};
        HypothesisEvaluationConfig config;
        config.enable_low_dim_batch = fixed;
        std::vector<FaultHypothesisV2> reference{h}, optimized{h};
        std::shared_ptr<const FrozenHypothesisNumerics> rf, of;
        ASSERT_EQ(setenv(key, "1", 1), 0);
        auto before = NumericalWorkCounters::snapshot().fault_gram_svd;
        const auto re = HypothesisEvidenceEvaluator(config).evaluateAll(
            window, {mode}, &reference, 100.0, &rf);
        reference_svds += NumericalWorkCounters::snapshot().fault_gram_svd - before;
        // Legacy proof lookup is keyed by served entry identity. Capture the
        // reference before the second evaluator can replace that registry key.
        ASSERT_TRUE(rf);
        std::vector<FrozenHypothesisPlProofV1> reference_proofs(rf->pl_entries.size());
        std::vector<bool> reference_found;
        for (std::size_t i = 0; i < rf->pl_entries.size(); ++i)
          reference_found.push_back(frozenHypothesisPlProof(
              rf->pl_entries[i], &reference_proofs[i]));
        ASSERT_EQ(unsetenv(key), 0);
        before = NumericalWorkCounters::snapshot().fault_gram_svd;
        const auto oe = HypothesisEvidenceEvaluator(config).evaluateAll(
            window, {mode}, &optimized, 100.0, &of);
        optimized_svds += NumericalWorkCounters::snapshot().fault_gram_svd - before;
        ASSERT_EQ(re.size(), 1u);
        ASSERT_EQ(oe.size(), 1u);
        EXPECT_EQ(reference[0].monitored, optimized[0].monitored);
        monitor_bits(reference[0].monitorability, optimized[0].monitorability);
        monitor_bits(re[0].monitorability, oe[0].monitorability);
        EXPECT_EQ(re[0].plausible, oe[0].plausible);
        EXPECT_EQ(re[0].profile_valid, oe[0].profile_valid);
        scalar_bits(re[0].all_in_statistic, oe[0].all_in_statistic);
        scalar_bits(re[0].conditioned_statistic, oe[0].conditioned_statistic);
        scalar_bits(re[0].explained_energy, oe[0].explained_energy);
        scalar_bits(re[0].log_evidence, oe[0].log_evidence);
        scalar_bits(re[0].profile_j, oe[0].profile_j);
        matrix_bits(re[0].estimated_fault, oe[0].estimated_fault);
        matrix_bits(re[0].fault_gram, oe[0].fault_gram);
        ASSERT_TRUE(rf && of);
        ASSERT_EQ(rf->pl_entries.size(), of->pl_entries.size());
        for (std::size_t i = 0; i < rf->pl_entries.size(); ++i) {
          EXPECT_EQ(frozenHypothesisPlEntryIdentity(rf->pl_entries[i]),
                    frozenHypothesisPlEntryIdentity(of->pl_entries[i]));
          const auto& rp = reference_proofs[i];
          FrozenHypothesisPlProofV1 op;
          const bool rfound = reference_found[i];
          const bool ofound = frozenHypothesisPlProof(of->pl_entries[i], &op);
          ASSERT_EQ(rfound, ofound);
          if (!rfound) continue;
          EXPECT_EQ(rp.proof_identity, op.proof_identity);
          EXPECT_EQ(rp.parent_proof_identity, op.parent_proof_identity);
          EXPECT_EQ(rp.served_entry_identity, op.served_entry_identity);
          EXPECT_EQ(rp.nullspace_class, op.nullspace_class);
          scalar_bits(rp.rank_tolerance, op.rank_tolerance);
          scalar_bits(rp.raw_factor_scale, op.raw_factor_scale);
          matrix_bits(rp.certified_gram, op.certified_gram);
          matrix_bits(rp.raw_detection_factor, op.raw_detection_factor);
          matrix_bits(rp.protected_response, op.protected_response);
          matrix_bits(rp.gram_eigenvalues, op.gram_eigenvalues);
          matrix_bits(rp.gram_eigenvectors, op.gram_eigenvectors);
          matrix_bits(rp.gram_eigenvalue_errors, op.gram_eigenvalue_errors);
          matrix_bits(rp.nullspace_axis_residual, op.nullspace_axis_residual);
        }
      }
  EXPECT_LT(optimized_svds, reference_svds);
  RecordProperty("reference_raw_gram_svds", reference_svds);
  RecordProperty("optimized_raw_gram_svds", optimized_svds);
}

TEST(Phase2NumericLayerPositive, KeepHasFinitePlAndGenuineProofWithFormalClosed) {
  using namespace uwb_imu_pl;
  const auto owner = freezeIntegrityWindowCopy(p002HistoryWindow(0.0, 0.0));
  const auto admission = admitFrozenIntegrityWindow(owner);
  ASSERT_TRUE(admission) << admission.reason;
  RankUpdateConfig rank{1e-12, 1e10, 10.0};  // Existing P103 fixture contract.
  ExclusionAction keep;
  keep.id = ExclusionActionId(10301);
  keep.action_model_id = "KEEP_ALL";
  auto candidate = DenseCandidateOracle(rank).evaluate(admission, keep);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.rank_tolerance = rank.rank_tolerance;
  detector_risk.max_condition_number = rank.max_condition_number;
  const auto detector = JointWindowDetector().evaluateCandidate(
      admission, candidate, detector_risk);
  ASSERT_TRUE(detector.passed) << detector.reason;
  // Preserve the complete existing P103 single-mode fixture universe.
  Eigen::MatrixXd mode = Eigen::MatrixXd::Zero(candidate.rows, 1);
  mode(0, 0) = 1.0;
  mode(mode.rows() - 1, 0) = 1.0;
  FaultHypothesisV2 h;
  h.id = HypothesisId(10302);
  h.modes = {FaultModeId(10303)};
  h.p_md_allocation = 1e-3;
  h.hmi_allocation = 1e-6;
  h.prior_probability_bound = 1e-3;
  std::vector<FaultHypothesisV2> hypotheses{h};
  ProtectionLevelSharedContext shared;
  shared.mode_maps.emplace(10303, mode);
  AttemptProofArena arena;
  ProtectionLevelV2ProofV1 proof;
  const auto pl = ProtectionLevelV2().computeShared(admission, &candidate,
      detector, &hypotheses, shared, RiskBudgetV2{}, &proof, &arena);
  ASSERT_TRUE(pl.model_valid) << pl.reason;
  EXPECT_TRUE(pl.pl_xyz_m.allFinite());
  EXPECT_FALSE(pl.risk_budget_valid);
  EXPECT_EQ(pl.availability, Availability::Unavailable);
  EXPECT_FALSE(pl.formal_eligible);
  EXPECT_TRUE(validateProtectionLevelV2Proof(candidate, detector, hypotheses,
      pl, arena, nullptr));
  ASSERT_EQ(proof.hypothesis_proofs.size(), hypotheses.size());
  RecordProperty("qualification", "NUMERIC_LAYER_POSITIVE");
}

TEST(Phase2NumericLayerPositive, UwbExclusionRemovesExactlyItsCoveredScope) {
  using namespace uwb_imu_pl;
  auto window = p002HistoryWindow(0.0, 0.0);
  // Fault only the existing third current measurement; preserve every row,
  // its original noise, and the current/history detector contracts.
  window.blocks[2].sensor = SensorType::Uwb;
  window.blocks[2].kind = FactorKind::UwbBatch;
  window.blocks[2].residual_raw(0) = 20.0;
  window.blocks[2].residual_whitened(0) = 20.0;
  finalizeIntegrityWindow(&window, 1e-12, 1e10);
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.rank_tolerance = 1e-12;
  detector_risk.max_condition_number = 1e10;
  const auto all_in = JointWindowDetector().evaluate(window, detector_risk);
  ASSERT_TRUE(all_in.numerically_valid) << all_in.reason;
  ASSERT_FALSE(all_in.passed);
  FaultHypothesisV2 fault;
  fault.id = HypothesisId(10302);
  fault.modes = {FaultModeId(10303)};
  fault.affected_groups = {FactorGroupId(3)};
  fault.p_md_allocation = 1e-3;
  fault.hmi_allocation = 1e-6;
  fault.prior_probability_bound = 1e-3;
  const std::vector<FaultHypothesisV2> complete_scope{fault};
  ExclusionAction action;
  action.id = ExclusionActionId(10304);
  action.groups_to_remove = {FactorGroupId(3)};
  action.covered_modes = fault.modes;
  action.exclusion_cardinality = 1;
  // The production semantic projector, not a test filter, removes the sole
  // represented fault. An uncovered copy must remain in the original scope.
  auto remaining = projectRemainingHypothesesForActionV1(complete_scope, action);
  ASSERT_TRUE(remaining.empty());
  auto uncovered = action;
  uncovered.covered_modes.clear();
  EXPECT_EQ(projectRemainingHypothesesForActionV1(complete_scope, uncovered).size(),
            complete_scope.size());
  const auto admission = admitFrozenIntegrityWindow(freezeIntegrityWindowCopy(window));
  ASSERT_TRUE(admission) << admission.reason;
  RankUpdateConfig rank{1e-12, 1e10, 10.0};
  auto candidate = DenseCandidateOracle(rank).evaluate(admission, action);
  ASSERT_TRUE(candidate.valid) << candidate.reason;
  const auto detector = JointWindowDetector().evaluateCandidate(
      admission, candidate, detector_risk);
  ASSERT_TRUE(detector.passed) << detector.reason;
  AttemptProofArena arena;
  ProtectionLevelV2ProofV1 proof;
  const auto pl = ProtectionLevelV2().computeShared(admission, &candidate,
      detector, &remaining, ProtectionLevelSharedContext{}, RiskBudgetV2{},
      &proof, &arena);
  ASSERT_TRUE(pl.model_valid) << pl.reason;
  EXPECT_TRUE(pl.pl_xyz_m.allFinite());
  EXPECT_FALSE(pl.risk_budget_valid);
  EXPECT_EQ(pl.availability, Availability::Unavailable);
  EXPECT_FALSE(pl.formal_eligible);
  EXPECT_TRUE(validateProtectionLevelV2Proof(candidate, detector, remaining,
      pl, arena, nullptr));
  // Removed-source risk is still due at FDE selection/publication. This test
  // does not fabricate a closing selection ledger or a protected packet.
  RecordProperty("qualification", "NUMERIC_LAYER_POSITIVE");
}

TEST(Phase2NumericLayerPositive, ImuBridgeIsIndependentAndItsBoxMarginFinite) {
  using namespace uwb_imu_pl;
  const auto config = researchConfig();
  std::vector<LinearizedFactorBlock> bridges;
  std::vector<BridgeUncertainty> uncertainties;
  std::vector<Eigen::MatrixXd> cv_jacobians;
  std::vector<Eigen::VectorXd> cv_residuals;
  for (const double fault : {0.0, 10.0}) {
    IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
    NavigationState initial;
    initial.timestamp = TimestampNs(0);
    initial.position_world_m = {0, 0, 1};
    estimator.initialize(initial, config.realtime.prior_sigmas);
    for (int sample = 0; sample <= 2; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(100 + sample);
      imu.timestamp = TimestampNs(sample * 5000000LL);
      imu.specific_force_mps2 = {fault, 0, config.imu.gravity_mps2};
      estimator.ingestImu(imu);
    }
    auto transaction = estimator.prepareEpoch(batch(config, 10000000));
    const auto window = estimator.buildIntegrityWindow(transaction, IntegrityWindowRequest{});
    const auto imu = estimator.buildPendingFactorBlock(transaction, transaction.imu_group.id);
    bridges.push_back(estimator.buildPendingFactorBlock(transaction,
        transaction.generic_bridge_group.id));
    // The pending block's residual is evaluated at the IMU-influenced frozen
    // linearization point. Independence belongs to the bridge factor/model:
    // compare it at the same previous/CV state, not at two different points.
    const auto factor = boost::dynamic_pointer_cast<KinematicPoseVelocityBridgeFactor>(
        transaction.generic_bridge_group.factors.at(0));
    ASSERT_TRUE(factor);
    const auto pose = [](const NavigationState& state) {
      return gtsam::Pose3(gtsam::Rot3(state.q_world_body.toRotationMatrix()),
                         state.position_world_m);
    };
    gtsam::Matrix h0, v0, h1, v1;
    cv_residuals.push_back(factor->evaluateError(pose(transaction.previous_state),
        transaction.previous_state.velocity_world_mps,
        pose(transaction.cv_predicted_state),
        transaction.cv_predicted_state.velocity_world_mps, h0, v0, h1, v1));
    Eigen::MatrixXd cv_h(9, 18);
    cv_h << h0, v0, h1, v1;
    cv_jacobians.push_back(std::move(cv_h));
    EXPECT_LT(cv_residuals.back().norm(), 1e-12);
    EXPECT_TRUE(bridges.back().jacobian_raw.allFinite());
    EXPECT_TRUE(bridges.back().residual_raw.allFinite());
    auto models = HypothesisGenerator().generate(window, transaction,
        ImuFaultSubspaceBuilder().build(transaction, imu), bridges.back());
    // Preserve every production-generated mode/hypothesis. Materialization
    // supplies interval recovery operations but is not a successful selection.
    const auto scope_count = models.hypotheses.size();
    HypothesisGenerator::ensureActionEntities(transaction, window, &models);
    EXPECT_EQ(models.hypotheses.size(), scope_count);
    EXPECT_EQ(models.single_accel_hypotheses, 3u);
    EXPECT_EQ(models.single_gyro_hypotheses, 3u);
    const auto bridge_action = std::find_if(models.single_mode_actions.begin(),
        models.single_mode_actions.end(), [](const auto& a) {
          return a.bridge_mode == BridgeMode::GenericKinematic;
        });
    ASSERT_NE(bridge_action, models.single_mode_actions.end());
    EXPECT_NE(std::find(bridge_action->groups_to_remove.begin(),
        bridge_action->groups_to_remove.end(), transaction.imu_group.id),
        bridge_action->groups_to_remove.end());
    uncertainties.push_back(BridgeFactory().uncertainty(transaction, GenericBridgeSpec{}));
    EXPECT_EQ(uncertainties.back().integrity_model,
              BridgeUncertainty::IntegrityModel::DeterministicBox);
    EXPECT_TRUE(uncertainties.back().optimization_covariance.allFinite());
    EXPECT_TRUE(uncertainties.back().deterministic_bound.allFinite());
    Eigen::Matrix<double, 3, Eigen::Dynamic> protected_map = Eigen::MatrixXd::Zero(3, 9);
    protected_map.middleCols(3, 3).setIdentity();
    const auto margin = BridgeFactory().propagateBoxMargin(protected_map,
        Eigen::MatrixXd::Identity(9, 9), uncertainties.back().deterministic_bound);
    EXPECT_TRUE(margin.allFinite());
    EXPECT_GT(margin.minCoeff(), 0.0);
    estimator.discardEpoch(std::move(transaction),
        {FdeStatus::ModelInvalid, "NUMERIC_LAYER_POSITIVE bridge witness", false});
  }
  EXPECT_TRUE((cv_jacobians[0].array() == cv_jacobians[1].array()).all());
  EXPECT_TRUE((cv_residuals[0].array() == cv_residuals[1].array()).all());
  EXPECT_TRUE((bridges[0].covariance.array() == bridges[1].covariance.array()).all());
  EXPECT_TRUE((uncertainties[0].deterministic_bound.array() ==
               uncertainties[1].deterministic_bound.array()).all());
  RecordProperty("qualification", "NUMERIC_LAYER_POSITIVE_BRIDGE_ONLY_NOT_FULL_PL");
}

TEST(P106SimulationAcceptance, PublishCallNotAnalysisControlsDeadline) {
  using namespace uwb_imu_pl;
  IntegrityOutput draft;
  draft.attempted_timestamp = TimestampNs(1);
  draft.state.timestamp = TimestampNs(1);
  draft.timestamp = TimestampNs(1);
  draft.publication.protected_output = true;
  draft.protection_level.formal_eligible = true;
  FinalPacketTiming timing;
  timing.arrival_steady_ns = 1000000000ULL;
  timing.compute_done_steady_ns = timing.arrival_steady_ns + 49000000ULL;
  timing.packet_ready_steady_ns = timing.compute_done_steady_ns;
  timing.publish_call_steady_ns = timing.arrival_steady_ns + 51000000ULL;
  timing.deadline_boundary = FinalPacketBoundary::PublishCall;
  const auto packet = finalizeOutputPacket(std::move(draft), timing,
                                           50000000ULL);
  EXPECT_TRUE(packet.output().deadline_missed);
  EXPECT_FALSE(packet.output().publication.protected_output);
  EXPECT_TRUE(packet.output().publication.unprotected_output);
  EXPECT_FALSE(packet.output().protection_level.formal_eligible);
}
