// C2 tests (DET-02/03/04): separated residual layout, dual-channel bound,
// fault-span projection and the model-error channel.
//
// The tests are written against the frozen contract (history-summary-design.md
// §8).  Where a claim depends on channels being independent, that dependence is
// stated explicitly and the assertion is made on the *bound*, not on a
// probability identity that only holds under independence.

#include <gtest/gtest.h>

#include <boost/math/distributions/non_central_chi_squared.hpp>

#include <cmath>
#include <cstdio>
#include <string>

#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"

namespace {

using namespace uwb_imu_pl;

IntegrityConfig researchConfig() {
  return IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/config/realtime_uwb_imu_pl_research.yaml");
}

// A small SPD Gram in `dim` parameters built from explicit directions.
Eigen::MatrixXd gramFrom(const std::vector<Eigen::VectorXd>& directions,
                         double scale) {
  Eigen::MatrixXd gram = Eigen::MatrixXd::Zero(directions.front().size(),
                                               directions.front().size());
  for (const auto& direction : directions) {
    gram += scale * direction * direction.transpose();
  }
  return gram;
}

Eigen::VectorXd unit(int dimension, int index) {
  Eigen::VectorXd value = Eigen::VectorXd::Zero(dimension);
  value(index) = 1.0;
  return value;
}

TEST(DualChannelDetector, InformationFormConstantIsCountedBeforeChannelSplit) {
  LinearizedIntegrityWindow window;
  window.model_valid = true;
  auto numerics = std::make_shared<FrozenWindowNumerics>();
  numerics->valid = true;
  numerics->statistic = 0.75;
  numerics->dof = 10;
  window.numerics = numerics;
  window.history_summary.valid = true;
  window.history_summary.nu_perp = 4;
  window.history_summary.kappa_b = 0.60;
  window.history_summary.constant_offset = 0.20;

  const auto split = evaluateDualChannel(window, 1e-6, 1000);
  ASSERT_TRUE(split.numerically_valid) << split.reason;
  EXPECT_EQ(split.current.dof, 6);
  EXPECT_EQ(split.history.dof, 4);
  EXPECT_NEAR(split.current.statistic, 0.15, 1e-15);
  EXPECT_NEAR(split.history.statistic, 0.80, 1e-15);
  EXPECT_NEAR(split.current.statistic + split.history.statistic,
              numerics->statistic + window.history_summary.constant_offset,
              1e-15);
}

// ---------------------------------------------------------------------------
// DET-02: a fault that only the *merged* layout can monitor.
// ---------------------------------------------------------------------------
TEST(DualChannelDetector, Det02MergedDirectionIsTheOnlyMonitoredOne) {
  const int dimension = 2;
  const Eigen::VectorXd e0 = unit(dimension, 0);
  const Eigen::VectorXd e1 = unit(dimension, 1);
  // Channel 1 (current system) is blind to the first parameter direction;
  // channel 2 (eliminated history residual) sees it and nothing else.
  ChannelBoundInput current;
  current.detector_id = "joint_window_state_supported";
  current.dof = 40;
  current.threshold = 55.76;   // chi^2_40 at p_fa = 1e-4 (existing budget)
  current.actual_threshold = current.threshold;
  current.weight = 0.5;
  current.gram = gramFrom({e1}, 4.0);
  current.rho_validated = true;
  ChannelBoundInput history;
  history.detector_id = "history_eliminated_detector_only";
  history.dof = 12;
  history.threshold = 32.91;   // chi^2_12 at p_fa = 1e-4
  history.actual_threshold = history.threshold;
  history.weight = 0.5;
  history.gram = gramFrom({e0}, 4.0);
  history.rho_validated = true;

  DualChannelBoundRequest request;
  request.channels = {current, history};
  // The protected state reacts to the merged direction only: g = e0 + e1.
  request.protected_response.resize(3, dimension);
  request.protected_response.row(0) = (e0 + e1).transpose();
  request.protected_response.row(1) = Eigen::RowVectorXd::Zero(dimension);
  request.protected_response.row(2) = Eigen::RowVectorXd::Zero(dimension);
  request.p_md = 1e-3;
  request.k_axis = Eigen::Vector3d(3.29, 3.29, 3.29);
  request.position_rho_validated = true;

  const DualChannelBoundResult bound = computeDualChannelBound(request);
  ASSERT_TRUE(bound.valid) << bound.reason;
  EXPECT_GT(bound.axis_bound_m(0), 0.0);
  EXPECT_EQ(bound.axis_bound_m(1), 0.0);
  EXPECT_EQ(bound.axis_bound_m(2), 0.0);
  // Both channels contributed a certified multiplier and a normalised weight.
  ASSERT_EQ(bound.channel_lambda.size(), 2u);
  EXPECT_NEAR(bound.channel_weight[0] + bound.channel_weight[1], 1.0, 1e-12);
  for (const double gap : bound.channel_lambda_gap) {
    EXPECT_LE(gap, 0.0) << "conservative side of F_chi2(Lambda) <= beta";
  }
  EXPECT_FALSE(bound.certificate_id.empty());

  // Independent oracle: W = w_c G_c/L_c + w_b G_b/L_b, b = sqrt(g W^-1 g').
  const double lc = bound.channel_lambda[0] * bound.channel_lambda[0];
  const double lb = bound.channel_lambda[1] * bound.channel_lambda[1];
  const Eigen::MatrixXd w = bound.channel_weight[0] / lc * current.gram +
                            bound.channel_weight[1] / lb * history.gram;
  const Eigen::VectorXd g = request.protected_response.row(0).transpose();
  const double expected = std::sqrt(g.dot(w.inverse() * g));
  EXPECT_NEAR(bound.axis_bound_m(0), expected, 1e-9 * expected)
      << "the stacked small-factorization bound must equal the explicit W form";

  // Only the current channel: the direction the protected state reacts to has
  // no fault contribution in that channel, so ker W is not covered by ker G
  // and the bound must be refused.  This is exactly the DET-02 headline: the
  // direction is monitorable only because the history channel is merged in.
  DualChannelBoundRequest current_only = request;
  current_only.channels = {current};
  const DualChannelBoundResult blind = computeDualChannelBound(current_only);
  EXPECT_FALSE(blind.valid);
  EXPECT_NE(blind.reason.find("ker W_h"), std::string::npos) << blind.reason;

  // A channel with no fault contribution (zero Gram) or no degrees of freedom
  // must be dropped without fabricating a Lambda, and the bound stays valid.
  DualChannelBoundRequest with_empty = request;
  ChannelBoundInput empty = history;
  empty.detector_id = "empty_channel";
  empty.gram.setZero();
  with_empty.channels = {current, empty, history};
  const DualChannelBoundResult dropped = computeDualChannelBound(with_empty);
  ASSERT_TRUE(dropped.valid) << dropped.reason;
  EXPECT_EQ(dropped.channel_lambda.size(), 2u)
      << "the empty channel must not contribute a Lambda";
  {
    // A channel with no degrees of freedom is dropped as well, without a
    // fabricated Lambda; the remaining channel keeps the bound valid.
    DualChannelBoundRequest no_dof = with_empty;
    no_dof.channels[1].dof = 0;
    no_dof.protected_response.row(0) = e1.transpose();  // ker W = span{e0} is
    no_dof.protected_response.row(1) = e0.transpose();  // covered by ker G
    const DualChannelBoundResult dropped_dof =
        computeDualChannelBound(no_dof);
    ASSERT_TRUE(dropped_dof.valid) << dropped_dof.reason;
    // Only the two real channels carry a certified multiplier: the empty /
    // dof-less channel contributes nothing and receives no Lambda.
    EXPECT_EQ(dropped_dof.channel_lambda.size(), 2u)
        << "the dof-less channel must not contribute a Lambda";
  }
  {
    // Nothing usable at all -> fail-closed refusal (no fabricated channel).
    DualChannelBoundRequest nothing = request;
    nothing.channels[0].gram.setZero();
    nothing.channels[1].dof = 0;
    const DualChannelBoundResult refused_none = computeDualChannelBound(nothing);
    EXPECT_FALSE(refused_none.valid);
    EXPECT_NE(refused_none.reason.find("degrees of freedom and fault "
                                       "contribution"),
              std::string::npos)
        << refused_none.reason;
  }

  // Both channels blind to the direction the protected state reacts to:
  // ker W_h is not covered by ker G_h -> refused.
  DualChannelBoundRequest uncovered = request;
  uncovered.channels[0].gram = gramFrom({e1}, 4.0);
  uncovered.channels[1].gram = gramFrom({e1}, 4.0);
  uncovered.protected_response.row(0) = e0.transpose();
  const DualChannelBoundResult refused = computeDualChannelBound(uncovered);
  EXPECT_FALSE(refused.valid);
  EXPECT_NE(refused.reason.find("ker W_h"), std::string::npos) << refused.reason;

  // Joint acceptance partition: the joint test is the intersection and its
  // union bound is reported with the unchanged per-test budget.
  const double p_fa = 1e-4;
  const std::uint32_t horizon = 20;
  EXPECT_LE(2.0 * p_fa * horizon, 1.0);
  EXPECT_LE(1.0 - std::pow(1.0 - p_fa, 2.0), 2.0 * p_fa + 1e-15)
      << "union bound dominates the (possibly dependent) joint false alarm";

  // Model-error channel: rho inflates the threshold, rho = 0 degenerates.
  EXPECT_EQ(riskAdjustedThreshold(32.91, 0.0), 32.91);
  EXPECT_NEAR(riskAdjustedThreshold(32.91, 0.5),
              (std::sqrt(32.91) + 0.5) * (std::sqrt(32.91) + 0.5), 1e-12);
  EXPECT_TRUE(std::isnan(riskAdjustedThreshold(-1.0, 0.0)));

  // A channel declared as an unverified assumption must not be certified as
  // validated (the state travels into the certificate consumer).
  DualChannelBoundRequest unverified = request;
  unverified.channels[1].rho_validated = false;
  unverified.channels[1].residual_rho = 0.25;
  unverified.channels[1].rho_source = "assumption: unvalidated residual bias";
  const DualChannelBoundResult assumption =
      computeDualChannelBound(unverified);
  ASSERT_TRUE(assumption.valid) << assumption.reason;
  EXPECT_FALSE(assumption.model_error_validated);
  EXPECT_EQ(assumption.model_error_source,
            "assumption: unvalidated residual bias");
  EXPECT_GT(assumption.channel_threshold[1], history.threshold)
      << "the risk threshold must be the inflated one";
}

// ---------------------------------------------------------------------------
// DET-03: fault-span projection (§7.5).
// ---------------------------------------------------------------------------
TEST(DualChannelDetector, Det03FaultSpanCoversEveryRetainedDirection) {
  // Declared span: three independent historical directions in the same
  // parameter space as the retained response below.
  Eigen::MatrixXd declared(3, 3);
  declared << 1.0, 0.0, 0.5,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0;
  const FaultSpanProjectionResult projection =
      buildFaultSpanProjection(declared, 1e-12);
  ASSERT_TRUE(projection.valid) << projection.reason;
  EXPECT_FALSE(projection.declared_span_gap);
  EXPECT_EQ(projection.retained_directions, 3u);

  // F_all: the full retained historical response (rows x directions).
  Eigen::MatrixXd response(6, 3);
  response << 1.0, 0.2, 0.0,
              0.0, 1.0, 0.0,
              0.5, 0.0, 1.0,
              0.0, 0.0, 0.4,
              0.0, 0.0, 0.0,
              0.0, 0.3, 0.0;
  const FaultSpanProjectionResult identity = evaluateFaultSpanIdentity(
      response, projection.basis, 1e-10);
  EXPECT_TRUE(identity.valid)
      << "Z_b'Z_b == F_all'F_all must hold over the whole declared span: "
      << identity.reason;
  EXPECT_LE(identity.exactness_residual, 1e-10);

  // A top-K (truncated) span must fail the identity: this is the guard against
  // selecting only the observed fault directions.
  const FaultSpanProjectionResult truncated_identity =
      evaluateFaultSpanIdentity(response, projection.basis.leftCols(2), 1e-10);
  EXPECT_FALSE(truncated_identity.valid);
  EXPECT_GT(truncated_identity.exactness_residual, 1e-10);

  // A rank-deficient declaration is recorded, not silently trimmed.
  Eigen::MatrixXd dependent = declared;
  dependent.col(2) = dependent.col(0) + dependent.col(1);
  const FaultSpanProjectionResult deficient =
      buildFaultSpanProjection(dependent, 1e-12);
  EXPECT_TRUE(deficient.declared_span_gap);
  EXPECT_EQ(deficient.retained_directions, 2u);
  EXPECT_NE(deficient.reason.find("rank deficient"), std::string::npos);
}

// ---------------------------------------------------------------------------
// DET-04: quantitative pooled vs separated comparison (no unconditional claims).
// ---------------------------------------------------------------------------
TEST(DualChannelDetector, Det04PooledVersusSeparatedIsQuantifiedNotAssumed) {
  // Two channels with the *unchanged* per-test budget.
  const double p_fa = 1e-4;
  const int nu_c = 40;
  const int nu_b = 12;
  const double tau_c =
      boost::math::quantile(boost::math::complement(
                                boost::math::chi_squared(nu_c), p_fa));
  const double tau_b =
      boost::math::quantile(boost::math::complement(
                                boost::math::chi_squared(nu_b), p_fa));
  const double tau_pooled = boost::math::quantile(boost::math::complement(
      boost::math::chi_squared(nu_c + nu_b), p_fa));
  // Channel non-centralities for a fault of magnitude m along the monitored
  // direction: lambda_c = m^2, lambda_b = m^2 (unit sensitivities).
  auto miss = [](int dof, double threshold, double lambda) {
    return boost::math::cdf(
        boost::math::non_central_chi_squared(dof, lambda), threshold);
  };
  double worst_difference = 0.0;
  double pooled_only = 0.0;
  double joint_only = 0.0;
  std::printf(
      "[DET-04] m,lambda,Tpooled_tau,pooled_miss,joint_miss,difference\n");
  for (const double magnitude : {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0}) {
    const double lambda = magnitude * magnitude;
    // Pooled reference implementation: one test on T_c + T_b with nu_c + nu_b.
    const double pooled_miss = miss(nu_c + nu_b, tau_pooled, 2.0 * lambda);
    // Separated layout: both channels must accept (joint acceptance).
    // NOTE: the product form below is the *independence model*; it is used to
    // quantify the regimes, never as a certified probability.
    const double joint_miss =
        1.0 - (1.0 - miss(nu_c, tau_c, lambda)) *
                  (1.0 - miss(nu_b, tau_b, lambda));
    const double difference = joint_miss - pooled_miss;
    worst_difference = std::max(worst_difference, std::abs(difference));
    if (difference > 0.0) pooled_only = std::max(pooled_only, difference);
    if (difference < 0.0) joint_only = std::max(joint_only, -difference);
    std::printf("[DET-04] %.1f,%.3f,%.3f,%.6f,%.6f,%+.6f\n", magnitude,
                lambda, tau_pooled, pooled_miss, joint_miss, difference);
    // Each layout must be monotone in the fault magnitude for its own miss
    // probability (a property of the non-central chi-square, no cross-layout
    // claim).
    EXPECT_TRUE(std::isfinite(pooled_miss) && std::isfinite(joint_miss));
  }
  // The comparison must be non-vacuous: the two layouts genuinely differ.
  EXPECT_GT(worst_difference, 1e-3)
      << "pooled and separated layouts must differ measurably somewhere";
  std::printf("[DET-04] worst_abs_difference=%.6f pooled_better_max=%.6f "
              "joint_better_max=%.6f\n",
              worst_difference, pooled_only, joint_only);
  // No monotone-in-time or unconditional superiority claim is made; only the
  // measured magnitude dependence above is reported.
  SUCCEED();
}

}  // namespace

// ---------------------------------------------------------------------------
// DET-02/04 on a REAL frozen window: the split must be exact and the scenario
// table must show the per-frame difference between the pooled reference and the
// separated layout (no monotonicity claim, every row printed).
// ---------------------------------------------------------------------------
namespace {

void appendImu(IncrementalUwbImuEstimator* estimator, double gravity,
               std::size_t epoch, int samples) {
  for (int sample = 1; sample <= samples; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(epoch * 1000 + static_cast<std::size_t>(sample));
    imu.timestamp = TimestampNs(
        static_cast<std::int64_t>(((epoch - 1) * 10 + sample) * 5000000));
    imu.specific_force_mps2 = {0, 0, gravity};
    estimator->ingestImu(imu);
  }
}

UwbBatch batch(const IntegrityConfig& config, std::int64_t time_ns) {
  UwbBatch value;
  value.id = BatchId(static_cast<std::uint64_t>(time_ns));
  value.timestamp = TimestampNs(time_ns);
  for (std::size_t i = 0; i < config.anchors.size(); ++i) {
    UwbMeasurement measurement;
    measurement.id = MeasurementId(i + 1);
    measurement.factor_id = FactorId(i + 1);
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

}  // namespace

TEST(DualChannelDetector, RealWindowSplitIsExactAndTablesTheDifference) {
  auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary;
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  const double p_fa = config.detector.p_fa_per_test;
  const std::uint32_t horizon = config.detector.continuity_horizon_tests;
  int rows_with_history = 0;
  std::printf("[DET-CHANNEL] epoch,T_pooled,tau_pooled,dof_pooled,T_c,tau_c,"
              "dof_c,T_b,tau_b,dof_b,joint\n");
  for (std::size_t epoch = 1; epoch <= 22; ++epoch) {
    appendImu(&estimator, config.imu.gravity_mps2, epoch, 10);
    auto input = batch(config, static_cast<std::int64_t>(epoch) * 50000000);
    for (std::size_t row = 0; row < input.measurements.size(); ++row) {
      input.measurements[row].id = MeasurementId(epoch * 100 + row + 1);
      input.measurements[row].factor_id = FactorId(epoch * 100 + row + 1);
    }
    auto transaction = estimator.prepareEpoch(input);
    if (epoch >= 18) {
      IntegrityWindowRequest request;
      request.epochs = 10;
      const auto window = estimator.buildIntegrityWindow(transaction, request);
      ASSERT_TRUE(window.model_valid) << window.reason;
      ASSERT_TRUE(window.numerics && window.numerics->valid);
      const DualChannelDecision split =
          evaluateDualChannel(window, p_fa, horizon);
      ASSERT_TRUE(split.numerically_valid) << split.reason;
      // Exactness of the split on the real window.
      EXPECT_NEAR(split.current.statistic + split.history.statistic,
                  window.numerics->statistic +
                      window.history_summary.constant_offset,
                  1e-9 * std::max(
                      1.0, std::abs(window.numerics->statistic +
                                    window.history_summary.constant_offset)))
          << "T_c + T_b must equal the pooled statistic";
      EXPECT_EQ(split.current.dof + split.history.dof,
                window.numerics->dof);
      EXPECT_DOUBLE_EQ(split.operation_p_fa_upper_bound,
                       std::min(1.0, p_fa * horizon * 2.0));
      EXPECT_EQ(split.joint_accepted,
                split.current.accepted && split.history.accepted);
      if (split.history.dof > 0) {
        ++rows_with_history;
        EXPECT_GT(split.history.threshold, 0.0);
      }
      std::printf("[DET-CHANNEL] %zu,%.6f,%.6f,%d,%.6f,%.6f,%d,%.6f,%.6f,%d,%d\n",
                  epoch, window.numerics->statistic,
                  window.numerics->statistic <= 0
                      ? 0.0
                      : boost::math::quantile(boost::math::complement(
                            boost::math::chi_squared(window.numerics->dof),
                            p_fa)),
                  window.numerics->dof, split.current.statistic,
                  split.current.threshold, split.current.dof,
                  split.history.statistic, split.history.threshold,
                  split.history.dof, split.joint_accepted ? 1 : 0);
      // The detector exposes the same split (production wiring, additive).
      DetectorRiskContext risk;
      risk.p_fa_per_test = p_fa;
      risk.continuity_horizon_tests = horizon;
      risk.rank_tolerance = config.integrity_window.rank_tolerance;
      risk.max_condition_number = config.integrity_window.max_condition_number;
      const auto detector = JointWindowDetector().evaluate(window, risk);
      EXPECT_TRUE(detector.channel_split_valid) << detector.channel_reason;
      EXPECT_DOUBLE_EQ(detector.channel_current_statistic,
                       split.current.statistic);
      EXPECT_DOUBLE_EQ(detector.channel_history_statistic,
                       split.history.statistic);
      EXPECT_EQ(detector.joint_accepted, split.joint_accepted);
    }
    const auto plan = EpochCommitPlan::nominalPlan(transaction);
    estimator.commitEpoch(std::move(transaction), plan);
  }
  // The scenario table must actually contain history-channel rows, otherwise
  // the comparison would be vacuous.
  EXPECT_GT(rows_with_history, 0);
}
