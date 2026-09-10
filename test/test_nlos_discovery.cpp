#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/make_shared.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "uifgo/nlos_discovery.h"
#include "uifgo/hash_utils.h"
#include "uifgo/paper_pose_prior_factor.h"
#include "uifgo/uwb_factor.h"

namespace {

class QuadraticRecoveryFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  explicit QuadraticRecoveryFactor(gtsam::Key key)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(1, 1.0), key) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    constexpr double kLinear = 0.003;
    // With the linked defaults, the lambda=1e-5 trial returns to the same
    // objective (small-change/no update), while lambda=1e-4 is a true descent.
    constexpr double kQuadratic = 0.000038;
    const double x = pose.translation().x();
    if (jacobian) {
      jacobian->setZero(1, 6);
      (*jacobian)(0, 3) = kLinear + kQuadratic * x;
    }
    gtsam::Vector error(1);
    error << 1.0 + kLinear * x + 0.5 * kQuadratic * x * x;
    return error;
  }
};

class ObservedQuadraticFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  ObservedQuadraticFactor(gtsam::Key key, double quadratic)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(1, 1.0), key),
        quadratic_(quadratic) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    constexpr double kLinear = 0.003;
    const double x = pose.translation().x();
    if (jacobian) {
      ++linearize_call_count;
      jacobian->setZero(1, 6);
      (*jacobian)(0, 3) = kLinear + quadratic_ * x;
    } else {
      ++error_call_count;
      if (error_call_count > 1) ++candidate_error_call_count;
    }
    gtsam::Vector error(1);
    error << 1.0 + kLinear * x + 0.5 * quadratic_ * x * x;
    return error;
  }

  mutable size_t linearize_call_count = 0;
  mutable size_t error_call_count = 0;
  mutable size_t candidate_error_call_count = 0;

 private:
  double quadratic_;
};

class OppositeJacobianFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  explicit OppositeJacobianFactor(gtsam::Key key)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(1, 1.0), key) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    const double x = pose.translation().x();
    if (jacobian) {
      ++linearize_call_count;
      jacobian->setZero(1, 6);
      // Deliberately inconsistent engineering fixture: every proposed move
      // follows the wrong direction, forcing bounded lambda exhaustion.
      (*jacobian)(0, 3) = -1.0;
    } else {
      ++error_call_count;
      if (error_call_count > 1) ++candidate_error_call_count;
    }
    gtsam::Vector error(1);
    error << 1.0 + x;
    return error;
  }

  mutable size_t linearize_call_count = 0;
  mutable size_t error_call_count = 0;
  mutable size_t candidate_error_call_count = 0;
};

class ConstantResidualFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  explicit ConstantResidualFactor(gtsam::Key key)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(1, 1.0), key) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3&,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    if (jacobian) jacobian->setZero(1, 6);
    gtsam::Vector error(1);
    error << 2.0;
    return error;
  }
};

class ThrowOnLinearizeFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  explicit ThrowOnLinearizeFactor(gtsam::Key key)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(1, 1.0), key) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3&,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    if (jacobian) {
      ++linearize_call_count;
      throw std::runtime_error("regression linearize exception");
    }
    ++ordinary_error_call_count;
    gtsam::Vector error(1);
    error << 1.0;
    return error;
  }

  mutable size_t linearize_call_count = 0;
  mutable size_t ordinary_error_call_count = 0;
};

class ThrowDuringTrialErrorFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  explicit ThrowDuringTrialErrorFactor(gtsam::Key key)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(1, 1.0), key) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    const double x = pose.translation().x();
    if (jacobian) {
      ++linearize_call_count;
      jacobian->setZero(1, 6);
      (*jacobian)(0, 3) = 1.0;
    } else if (std::abs(x) > 1e-12) {
      ++candidate_error_call_count;
      throw std::runtime_error("regression trial error exception");
    } else {
      ++initial_error_call_count;
    }
    gtsam::Vector error(1);
    error << 1.0 + x;
    return error;
  }

  mutable size_t linearize_call_count = 0;
  mutable size_t initial_error_call_count = 0;
  mutable size_t candidate_error_call_count = 0;
};

class ThrowOnSecondTrialErrorFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  explicit ThrowOnSecondTrialErrorFactor(gtsam::Key key)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Isotropic::Sigma(1, 1.0), key) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override {
    const double x = pose.translation().x();
    if (jacobian) {
      ++linearize_call_count;
      jacobian->setZero(1, 6);
      (*jacobian)(0, 3) = 1.0;
    } else {
      ++error_call_count;
      if (error_call_count > 1 && ++candidate_error_call_count == 2)
        throw std::runtime_error("regression second-trial exception");
    }
    gtsam::Vector error(1);
    error << 1.0 + x;
    return error;
  }

  mutable size_t linearize_call_count = 0;
  mutable size_t error_call_count = 0;
  mutable size_t candidate_error_call_count = 0;
};

uifgo::FusedLassoOptions TightOptions() {
  uifgo::FusedLassoOptions options;
  options.lambda_l1 = 0.05;
  options.lambda_tv = 0.10;
  options.primal_absolute_tolerance_m = 1e-10;
  options.primal_relative_tolerance = 1e-8;
  options.dual_absolute_tolerance_objective_per_m = 1e-10;
  options.dual_relative_tolerance = 1e-8;
  options.kkt_tolerance_objective_per_m = 2e-7;
  options.tv_subgradient_tolerance_objective_per_m = 2e-7;
  options.max_iterations = 30000;
  return options;
}

uifgo::DiscoveryObservation Obs(std::uint64_t id, double t, double b,
                                size_t chain = 1, size_t run = 1,
                                double weight = 1.0) {
  uifgo::DiscoveryObservation observation;
  observation.obs_id = id;
  observation.tag_id = 1;
  observation.anchor_id = 2;
  observation.chain_id = chain;
  observation.active_run_id = run;
  observation.sensor_time = t;
  observation.weight = weight;
  observation.bias_m = b;
  return observation;
}

uifgo::DiscoveryOptions A02Options(
    uifgo::ConditionalLmPolicy policy = uifgo::ConditionalLmPolicy::
        GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1) {
  uifgo::DiscoveryOptions options;
  options.conditional_lm.policy = policy;
  options.navigation_stationarity_tolerance_objective = 2e-6;
  options.gradient_roundoff_safety_factor = 4.0;
  options.navigation_scales.pose_rotation_rad = 2.0;
  options.navigation_scales.pose_translation_m = 3.0;
  options.navigation_scales.velocity_mps = 4.0;
  options.navigation_scales.accel_bias_mps2 = 5.0;
  options.navigation_scales.gyro_bias_radps = 6.0;
  options.conditional_lm.navigation_stationarity_tolerance_objective =
      options.navigation_stationarity_tolerance_objective;
  options.conditional_lm.gradient_roundoff_safety_factor =
      options.gradient_roundoff_safety_factor;
  options.conditional_lm.navigation_scales = options.navigation_scales;
  return options;
}

void IntroduceA02NavigationParameterMismatch(
    size_t field_index, uifgo::DiscoveryOptions* options) {
  switch (field_index) {
    case 0:
      options->conditional_lm.navigation_stationarity_tolerance_objective +=
          1e-6;
      break;
    case 1:
      options->conditional_lm.gradient_roundoff_safety_factor += 1.0;
      break;
    case 2:
      options->conditional_lm.navigation_scales.pose_rotation_rad += 1.0;
      break;
    case 3:
      options->conditional_lm.navigation_scales.pose_translation_m += 1.0;
      break;
    case 4:
      options->conditional_lm.navigation_scales.velocity_mps += 1.0;
      break;
    case 5:
      options->conditional_lm.navigation_scales.accel_bias_mps2 += 1.0;
      break;
    case 6:
      options->conditional_lm.navigation_scales.gyro_bias_radps += 1.0;
      break;
    default:
      throw std::invalid_argument("unknown A02 mismatch test field");
  }
}

}  // namespace

TEST(FusedLasso, SinglePointMatchesAnalyticNonnegativeL1Solution) {
  auto options = TightOptions();
  options.lambda_l1 = 0.6;
  options.lambda_tv = 9.0;  // D is empty for one point.
  const auto result =
      uifgo::SolveNonnegativeFusedLassoChain({1.0}, {2.0}, options);
  ASSERT_TRUE(result.converged()) << result.reason;
  ASSERT_EQ(result.u.size(), 1u);
  EXPECT_NEAR(result.u[0], 0.7, 2e-7);
  EXPECT_TRUE(result.p.empty());
  EXPECT_LE(result.primal_residual_m, result.primal_threshold_m);
  EXPECT_LE(result.dual_residual_objective_per_m,
            result.dual_threshold_objective_per_m);
}

TEST(PaperPosePriorFactor,
     PreservesLinkedResidualAndCorrectsPoseRetractJacobian) {
  using gtsam::symbol_shorthand::X;
  const gtsam::Pose3 prior(
      gtsam::Rot3::RzRyRx(-0.41, 0.29, -0.17),
      gtsam::Point3(1.3, -2.1, 0.8));
  const gtsam::Pose3 current(
      gtsam::Rot3::RzRyRx(0.36, -0.22, 0.31),
      gtsam::Point3(-0.7, 1.6, 2.4));
  const auto noise = gtsam::noiseModel::Unit::Create(6);
  const auto linked = boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(0), prior, noise);
  const auto paper = boost::make_shared<uifgo::PaperPosePriorFactor>(
      X(0), prior, noise);
  gtsam::Values values;
  values.insert(X(0), current);

  EXPECT_DOUBLE_EQ(linked->error(values), paper->error(values));
  const gtsam::Vector6 residual =
      -gtsam::traits<gtsam::Pose3>::Local(current, prior);
  EXPECT_DOUBLE_EQ(paper->error(values), 0.5 * residual.squaredNorm());

  gtsam::Vector6 direction;
  direction << 0.29, -0.43, 0.17, 0.71, -0.38, 0.24;
  direction.normalize();
  const auto directional_derivative = [&](const auto& factor) {
    return factor->linearize(values)->gradientAtZero().at(X(0)).dot(direction);
  };
  const double linked_analytic = directional_derivative(linked);
  const double paper_analytic = directional_derivative(paper);
  bool linked_failed_at_least_once = false;
  for (const double step : {1e-4, 1e-5, 1e-6}) {
    gtsam::Values plus = values;
    gtsam::Values minus = values;
    plus.update(X(0), current.retract(step * direction));
    minus.update(X(0), current.retract(-step * direction));
    const double central =
        (paper->error(plus) - paper->error(minus)) / (2.0 * step);
    const double paper_tolerance = 5e-9 + 0.005 * std::abs(paper_analytic);
    const double linked_tolerance = 5e-9 + 0.005 * std::abs(linked_analytic);
    EXPECT_LE(std::abs(central - paper_analytic), paper_tolerance);
    linked_failed_at_least_once = linked_failed_at_least_once ||
        std::abs(central - linked_analytic) > linked_tolerance;
  }
  EXPECT_TRUE(linked_failed_at_least_once);
}

TEST(PaperPosePriorFactor, ReplacementIsExplicitAndPreservesGraphObjective) {
  using gtsam::symbol_shorthand::X;
  const gtsam::Pose3 prior(
      gtsam::Rot3::RzRyRx(0.2, -0.1, 0.3),
      gtsam::Point3(0.4, -0.8, 1.2));
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06).finished());
  gtsam::NonlinearFactorGraph legacy_graph;
  legacy_graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), prior, noise));
  ASSERT_NE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(
                legacy_graph.at(0)),
            nullptr);
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(
      gtsam::Rot3::RzRyRx(-0.1, 0.15, -0.2),
      gtsam::Point3(0.7, -0.3, 0.2)));
  const double old_objective = legacy_graph.error(values);

  gtsam::NonlinearFactorGraph paper_graph = legacy_graph;
  EXPECT_EQ(uifgo::ReplacePosePriorsForPaperPath(&paper_graph), 1u);
  EXPECT_NE(boost::dynamic_pointer_cast<uifgo::PaperPosePriorFactor>(
                paper_graph.at(0)),
            nullptr);
  EXPECT_DOUBLE_EQ(paper_graph.error(values), old_objective);
  EXPECT_NE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(
                legacy_graph.at(0)),
            nullptr);
}

TEST(HashUtils, Sha256KnownVector) {
  EXPECT_EQ(uifgo::Sha256Hex("abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(FusedLasso, NoBiasAndConstantSegmentHaveExpectedSolutions) {
  auto options = TightOptions();
  options.lambda_l1 = 0.1;
  options.lambda_tv = 0.2;
  const auto zero = uifgo::SolveNonnegativeFusedLassoChain(
      {-0.2, 0.0, -0.1}, {1.0, 3.0, 2.0}, options);
  ASSERT_TRUE(zero.converged()) << zero.reason;
  for (double value : zero.u) EXPECT_NEAR(value, 0.0, 2e-7);

  const auto constant = uifgo::SolveNonnegativeFusedLassoChain(
      {0.5, 0.5, 0.5, 0.5}, {2.0, 2.0, 2.0, 2.0}, options);
  ASSERT_TRUE(constant.converged()) << constant.reason;
  for (double value : constant.u) EXPECT_NEAR(value, 0.45, 2e-7);
}

TEST(FusedLasso, ZeroRegularizersRecoverWeightedPointwiseRamp) {
  auto options = TightOptions();
  options.lambda_l1 = 0.0;
  options.lambda_tv = 0.0;
  const std::vector<double> ramp{-0.2, 0.0, 0.2, 0.6, 1.0};
  const std::vector<double> weights{0.25, 1.0, 4.0, 16.0, 3.0};
  const auto result =
      uifgo::SolveNonnegativeFusedLassoChain(ramp, weights, options);
  ASSERT_TRUE(result.converged()) << result.reason;
  for (size_t i = 0; i < ramp.size(); ++i)
    EXPECT_NEAR(result.u[i], std::max(0.0, ramp[i]), 2e-7);
}

TEST(FusedLasso, RhoScaleChangesConvergencePathNotDeclaredSolution) {
  const std::vector<double> e{0.0, 0.8, 0.2, 1.1, 0.4};
  const std::vector<double> w{0.5, 3.0, 1.0, 9.0, 2.0};
  std::vector<std::vector<double>> solutions;
  for (double rho_scale : {0.1, 1.0, 10.0}) {
    auto options = TightOptions();
    options.rho_scale = rho_scale;
    const auto result =
        uifgo::SolveNonnegativeFusedLassoChain(e, w, options);
    ASSERT_TRUE(result.converged())
        << "rho_scale=" << rho_scale << " " << result.reason;
    EXPECT_LE(result.max_kkt_violation_objective_per_m,
              options.kkt_tolerance_objective_per_m);
    EXPECT_LE(result.max_tv_subgradient_violation_objective_per_m,
              options.tv_subgradient_tolerance_objective_per_m);
    solutions.push_back(result.u);
  }
  for (size_t k = 1; k < solutions.size(); ++k)
    for (size_t i = 0; i < e.size(); ++i)
      EXPECT_NEAR(solutions[0][i], solutions[k][i], 5e-6);
}

TEST(FusedLasso, NonConvergenceIsExplicit) {
  auto options = TightOptions();
  options.max_iterations = 1;
  const auto result = uifgo::SolveNonnegativeFusedLassoChain(
      {0.0, 1.0, 0.0}, {1.0, 1.0, 1.0}, options);
  EXPECT_EQ(result.status, uifgo::FusedLassoStatus::MAX_ITERATIONS);
  EXPECT_FALSE(result.converged());
}

TEST(Partition, GapInactiveChangeEqualityAndShortAreStrict) {
  uifgo::DiscoveryOptions options;
  options.active_bias_min_m = 0.2;
  options.change_point_min_m = 0.25;
  options.merge_max_difference_m = 0.0;
  options.short_min_count = 2;
  options.short_min_duration_s = 0.1;
  std::vector<uifgo::DiscoveryObservation> snapshot{
      Obs(1, 0.0, 0.2, 1, 1),       // b_min equality is active
      Obs(2, 0.1, 0.45, 1, 1),      // change equality splits
      Obs(3, 0.2, 0.1, 1, 0),       // inactive separates
      Obs(4, 0.3, 0.45, 1, 2),
      Obs(5, 2.0, 0.45, 2, 3)};      // preassigned gap chain
  const auto partition =
      uifgo::BuildAutomaticSupportPartition(snapshot, options);
  ASSERT_EQ(partition.segments.size(), 4u);
  EXPECT_EQ(partition.segments[0].obs_ids, std::vector<std::uint64_t>({1}));
  EXPECT_EQ(partition.segments[1].obs_ids, std::vector<std::uint64_t>({2}));
  EXPECT_EQ(partition.segments[2].obs_ids, std::vector<std::uint64_t>({4}));
  EXPECT_EQ(partition.segments[3].obs_ids, std::vector<std::uint64_t>({5}));
  for (const auto& segment : partition.segments)
    EXPECT_TRUE(segment.short_support_debug);
}

TEST(Partition, GapEqualityConnectsAndStrictlyGreaterGapBreaks) {
  std::vector<uifgo::DiscoveryObservation> observations{
      Obs(3, std::nextafter(2.0, 3.0), 0.3),
      Obs(1, 0.0, 0.3), Obs(2, 1.0, 0.3)};
  uifgo::AssignDiscoveryChains(&observations, 1.0);
  ASSERT_EQ(observations.size(), 3u);
  EXPECT_EQ(observations[0].chain_id, observations[1].chain_id);
  EXPECT_NE(observations[1].chain_id, observations[2].chain_id);
}

TEST(Partition, A01IsSinglePassWithRecomputedAccumulatorAndOriginalNext) {
  uifgo::DiscoveryOptions options;
  options.active_bias_min_m = 0.1;
  options.change_point_min_m = 0.2;
  options.merge_max_difference_m = 0.25;
  options.short_min_count = 1;
  options.short_min_duration_s = 0.0;
  std::vector<uifgo::DiscoveryObservation> snapshot{
      Obs(10, 0.0, 0.25, 1, 1, 1.0),
      Obs(11, 1.0, 0.50, 1, 1, 1.0),
      Obs(12, 2.0, 0.75, 1, 1, 1.0)};
  const auto partition =
      uifgo::BuildAutomaticSupportPartition(snapshot, options);
  ASSERT_EQ(partition.segments.size(), 2u);
  EXPECT_EQ(partition.segments[0].obs_ids,
            std::vector<std::uint64_t>({10, 11}));
  EXPECT_EQ(partition.segments[0].parent_segment_ids.size(), 2u);
  EXPECT_NEAR(partition.segments[0].merge_snapshot_mean_m, 0.375, 1e-15);
  EXPECT_EQ(partition.segments[1].obs_ids,
            std::vector<std::uint64_t>({12}));
  EXPECT_EQ(partition.segments[1].parent_segment_ids.size(), 1u);
  EXPECT_FALSE(partition.discovery_snapshot_hash.empty());
  EXPECT_FALSE(partition.partition_hash.empty());

  std::set<std::uint64_t> ownership;
  for (const auto& segment : partition.segments)
    for (auto id : segment.obs_ids) EXPECT_TRUE(ownership.insert(id).second);
}

TEST(Partition, EqualityMergeAndImmutableHashAreDeterministic) {
  uifgo::DiscoveryOptions options;
  options.active_bias_min_m = 0.0;
  options.change_point_min_m = 0.25;
  options.merge_max_difference_m = 0.25;
  options.short_min_count = 1;
  options.short_min_duration_s = 0.0;
  std::vector<uifgo::DiscoveryObservation> snapshot{
      Obs(2, 1.0, 0.50), Obs(1, 0.0, 0.25)};
  const auto a = uifgo::BuildAutomaticSupportPartition(snapshot, options);
  std::reverse(snapshot.begin(), snapshot.end());
  const auto b = uifgo::BuildAutomaticSupportPartition(snapshot, options);
  ASSERT_EQ(a.segments.size(), 1u);
  EXPECT_EQ(a.segments[0].obs_ids,
            std::vector<std::uint64_t>({1, 2}));
  EXPECT_EQ(a.discovery_snapshot_hash, b.discovery_snapshot_hash);
  EXPECT_EQ(a.partition_hash, b.partition_hash);
  EXPECT_EQ(a.segments[0].segment_id, b.segments[0].segment_id);
  EXPECT_EQ(a.segments[0].parent_segment_ids,
            b.segments[0].parent_segment_ids);
}

TEST(Partition, NoActiveObservationProducesHashedEmptyPartition) {
  uifgo::DiscoveryOptions options;
  options.active_bias_min_m = 0.2;
  const auto partition = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.0), Obs(2, 1.0, 0.1)}, options);
  EXPECT_TRUE(partition.segments.empty());
  EXPECT_FALSE(partition.discovery_snapshot_hash.empty());
  EXPECT_FALSE(partition.partition_hash.empty());
}

TEST(Partition, StableWeightedRepresentativeHandlesOverflowAndNonuniformWeights) {
  uifgo::DiscoveryOptions options;
  options.active_bias_min_m = 0.1;
  options.change_point_min_m = 1.0;
  options.merge_max_difference_m = 1.0;
  options.short_min_count = 1;
  options.short_min_duration_s = 0.0;
  const auto equal_extreme = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.4, 1, 1, 1e308),
       Obs(2, 1.0, 0.4, 1, 1, 1e308)}, options);
  ASSERT_EQ(equal_extreme.segments.size(), 1u);
  EXPECT_DOUBLE_EQ(equal_extreme.segments[0].merge_snapshot_mean_m, 0.4);

  const auto nonuniform = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.4, 1, 1, 1e308),
       Obs(2, 1.0, 0.8, 1, 1, 1e307)}, options);
  ASSERT_EQ(nonuniform.segments.size(), 1u);
  EXPECT_NEAR(nonuniform.segments[0].merge_snapshot_mean_m,
              (0.4 + 0.08) / 1.1, 1e-15);
}

TEST(Partition, Sha256SnapshotIncludesSolverAndExternalContext) {
  uifgo::DiscoveryOptions first;
  first.active_bias_min_m = 0.1;
  first.short_min_count = 1;
  first.short_min_duration_s = 0.0;
  uifgo::DiscoveryContext context;
  context.input_plan_hash = "sha256:plan";
  context.source_hash = "sha256:source";
  context.config_hash = "sha256:config";
  context.calibration_hash = "sha256:calibration";
  const auto a = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.4)}, first, context);
  auto second = first;
  second.fused_lasso.lambda_l1 += 1.0;
  const auto b = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.4)}, second, context);
  EXPECT_EQ(a.hash_algorithm, "SHA-256");
  EXPECT_EQ(a.discovery_snapshot_hash.rfind("sha256:", 0), 0u);
  EXPECT_EQ(a.partition_hash.rfind("sha256:", 0), 0u);
  EXPECT_NE(a.discovery_snapshot_hash, b.discovery_snapshot_hash);
  EXPECT_NE(a.partition_hash, b.partition_hash);
  EXPECT_EQ(a.input_plan_hash, context.input_plan_hash);
}

TEST(Partition, ConditionalNavigationPolicyChangesSupportIdentity) {
  uifgo::DiscoveryOptions baseline;
  baseline.active_bias_min_m = 0.1;
  baseline.short_min_count = 1;
  baseline.short_min_duration_s = 0.0;
  const auto old_policy = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.4)}, baseline);
  auto qualified = baseline;
  qualified.conditional_lm.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1;
  const auto new_policy = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.4)}, qualified);
  auto v2 = baseline;
  v2.conditional_lm.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  const auto v2_policy = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.4)}, v2);
  EXPECT_NE(old_policy.solver_config_hash, new_policy.solver_config_hash);
  EXPECT_NE(old_policy.solver_config_hash, v2_policy.solver_config_hash);
  EXPECT_NE(new_policy.solver_config_hash, v2_policy.solver_config_hash);
  EXPECT_NE(old_policy.discovery_context_hash,
            new_policy.discovery_context_hash);
  EXPECT_NE(old_policy.discovery_snapshot_hash,
            new_policy.discovery_snapshot_hash);
  EXPECT_NE(old_policy.partition_hash, new_policy.partition_hash);
}

TEST(A02ParameterConsistency,
     MatchingParametersAreAcceptedByStandalonePartitionEntry) {
  for (const auto policy : {
           uifgo::ConditionalLmPolicy::
               GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1,
           uifgo::ConditionalLmPolicy::
               GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2}) {
    const auto options = A02Options(policy);
    const auto partition = uifgo::BuildAutomaticSupportPartition(
        {Obs(1, 0.0, 0.4)}, options);
    ASSERT_EQ(partition.segments.size(), 1u);
    EXPECT_FALSE(partition.partition_hash.empty());
  }
}

TEST(A02ParameterConsistency,
     ToleranceRoundoffAndEveryScaleMismatchAreRejectedAtBothEntries) {
  const std::vector<std::string> fields = {
      "navigation_stationarity_tolerance_objective",
      "gradient_roundoff_safety_factor",
      "navigation_scales.pose_rotation_rad",
      "navigation_scales.pose_translation_m",
      "navigation_scales.velocity_mps",
      "navigation_scales.accel_bias_mps2",
      "navigation_scales.gyro_bias_radps"};
  for (const auto policy : {
           uifgo::ConditionalLmPolicy::
               GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1,
           uifgo::ConditionalLmPolicy::
               GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2}) {
    for (size_t field_index = 0; field_index < fields.size(); ++field_index) {
      auto options = A02Options(policy);
      IntroduceA02NavigationParameterMismatch(field_index, &options);

    try {
      static_cast<void>(uifgo::BuildAutomaticSupportPartition(
          {Obs(1, 0.0, 0.4)}, options));
      FAIL() << "partition entry accepted A02 mismatch: "
             << fields[field_index];
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string(error.what()).find(
                    "A02 conditional and outer navigation parameters disagree"),
                std::string::npos)
          << fields[field_index];
      EXPECT_NE(std::string(error.what()).find(fields[field_index]),
                std::string::npos);
    }

      const auto provider_result =
          uifgo::AutomaticSupportProvider(options).Run(
              gtsam::NonlinearFactorGraph(), gtsam::Values(), {},
              uifgo::PaperInputPlan(), uifgo::Config(),
              uifgo::DiscoveryContext());
      EXPECT_EQ(provider_result.status, uifgo::DiscoveryStatus::INVALID_INPUT)
          << fields[field_index];
      EXPECT_NE(provider_result.reason.find(
                    "A02 conditional and outer navigation parameters disagree"),
                std::string::npos)
          << fields[field_index];
      EXPECT_NE(provider_result.reason.find(fields[field_index]),
                std::string::npos);
    }
  }
}

TEST(CheckedConditionalLmPolicyV2,
     FixedFailureEquivalentContinuesSearchAndAcceptsRealDescent) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(boost::make_shared<QuadraticRecoveryFactor>(X(0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3());
  uifgo::CheckedLmOptions v1;
  v1.max_iterations = 4;
  v1.relative_tolerance = 1e-6;
  v1.absolute_tolerance = 1e-8;
  v1.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1;
  const auto stalled = uifgo::RunCheckedConditionalLm(graph, initial, v1);
  EXPECT_FALSE(stalled.converged);
  EXPECT_EQ(stalled.convergence.lambda_trial_accounting_status, "COMPLETE");
  EXPECT_EQ(stalled.convergence.accepted_update_count, 0u);
  EXPECT_EQ(stalled.convergence.no_update_return_count, 4u);
  EXPECT_DOUBLE_EQ(stalled.convergence.optimizer_internal_relative_tolerance,
                   v1.relative_tolerance);

  auto v2 = v1;
  v2.max_iterations = 50;
  v2.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  const auto recovered = uifgo::RunCheckedConditionalLm(graph, initial, v2);
  ASSERT_TRUE(recovered.converged) << recovered.reason;
  EXPECT_EQ(recovered.convergence.lambda_trial_accounting_status,
            "COMPLETE");
  EXPECT_EQ(recovered.reason,
            "CONDITIONAL_LM_CONVERGED_AND_NAVIGATION_STATIONARY");
  EXPECT_DOUBLE_EQ(
      recovered.convergence.optimizer_internal_relative_tolerance, 0.0);
  EXPECT_FALSE(
      recovered.convergence.optimizer_internal_small_change_stop_enabled);
  EXPECT_DOUBLE_EQ(recovered.convergence.relative_tolerance,
                   v2.relative_tolerance);
  EXPECT_DOUBLE_EQ(recovered.convergence.absolute_tolerance,
                   v2.absolute_tolerance);
  EXPECT_GE(recovered.convergence.lambda_trial_count, 2u);
  EXPECT_GE(recovered.convergence.rejected_lambda_trial_count, 1u);
  EXPECT_GE(recovered.convergence.accepted_update_count, 1u);
  EXPECT_LT(graph.error(recovered.values), graph.error(initial));
  EXPECT_TRUE(recovered.last_qualification_stationarity.valid);
  EXPECT_TRUE(recovered.last_qualification_stationarity.stationary);
}

TEST(CheckedConditionalLmDiagnostics,
     NormalReturnTrialAccountingMatchesIndependentFactorObservations) {
  using gtsam::symbol_shorthand::X;
  const auto run_quadratic = [](double quadratic,
                                uifgo::ConditionalLmPolicy policy) {
    gtsam::NonlinearFactorGraph graph;
    const auto factor =
        boost::make_shared<ObservedQuadraticFactor>(X(0), quadratic);
    graph.add(factor);
    gtsam::Values initial;
    initial.insert(X(0), gtsam::Pose3());
    uifgo::CheckedLmOptions options;
    options.max_iterations = 1;
    options.relative_tolerance = 1e-6;
    options.absolute_tolerance = 1e-8;
    options.policy = policy;
    return std::make_pair(
        uifgo::RunCheckedConditionalLm(graph, initial, options), factor);
  };

  const auto direct_small_change = run_quadratic(
      0.000038,
      uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1);
  EXPECT_EQ(direct_small_change.second->candidate_error_call_count, 1u);
  EXPECT_EQ(direct_small_change.first.inner_iterations, 0);
  EXPECT_EQ(direct_small_change.first.convergence.lambda_trial_count, 1u);
  EXPECT_EQ(
      direct_small_change.first.convergence.rejected_lambda_trial_count, 0u);
  EXPECT_EQ(direct_small_change.first.convergence.accepted_update_count, 0u);
  EXPECT_EQ(direct_small_change.first.convergence.no_update_return_count, 1u);
  EXPECT_EQ(direct_small_change.first.convergence.lambda_trial_accounting_status,
            "COMPLETE");

  for (const auto policy : {
           uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1,
           uifgo::ConditionalLmPolicy::
               GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1}) {
    const auto rejected_then_small_change = run_quadratic(0.000218, policy);
    EXPECT_EQ(rejected_then_small_change.second->candidate_error_call_count,
              2u);
    EXPECT_EQ(rejected_then_small_change.first.inner_iterations, 1);
    EXPECT_EQ(
        rejected_then_small_change.first.convergence.lambda_trial_count, 2u);
    EXPECT_EQ(rejected_then_small_change.first.convergence
                  .rejected_lambda_trial_count,
              1u);
    EXPECT_EQ(rejected_then_small_change.first.convergence
                  .accepted_update_count,
              0u);
    EXPECT_EQ(rejected_then_small_change.first.convergence
                  .no_update_return_count,
              1u);
    EXPECT_EQ(rejected_then_small_change.first.convergence
                  .lambda_trial_accounting_status,
              "COMPLETE");
  }

  const auto rejected_then_accepted = run_quadratic(
      0.000038,
      uifgo::ConditionalLmPolicy::
          GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2);
  EXPECT_EQ(rejected_then_accepted.second->candidate_error_call_count, 2u);
  EXPECT_EQ(rejected_then_accepted.first.inner_iterations, 2);
  EXPECT_EQ(rejected_then_accepted.first.convergence.lambda_trial_count, 2u);
  EXPECT_EQ(rejected_then_accepted.first.convergence
                .rejected_lambda_trial_count,
            1u);
  EXPECT_EQ(rejected_then_accepted.first.convergence.accepted_update_count,
            1u);
  EXPECT_EQ(rejected_then_accepted.first.convergence.no_update_return_count,
            0u);
  EXPECT_EQ(rejected_then_accepted.first.convergence
                .lambda_trial_accounting_status,
            "COMPLETE");
}

TEST(CheckedConditionalLmPolicyV2,
     CallCapAndLambdaExhaustionRemainExplicitFailures) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph cap_graph;
  cap_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(
      gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));
  uifgo::CheckedLmOptions options;
  options.max_iterations = 1;
  options.relative_tolerance = 0.0;
  options.absolute_tolerance = 0.0;
  options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  options.navigation_stationarity_tolerance_objective = 0.0;
  const auto capped =
      uifgo::RunCheckedConditionalLm(cap_graph, initial, options);
  EXPECT_FALSE(capped.converged);
  EXPECT_EQ(capped.convergence.iterate_call_count, 1u);
  EXPECT_EQ(capped.convergence.lambda_trial_accounting_status, "COMPLETE");
  EXPECT_TRUE(capped.values.empty());

  gtsam::NonlinearFactorGraph exhaustion_graph;
  const auto exhaustion_factor =
      boost::make_shared<OppositeJacobianFactor>(X(0));
  exhaustion_graph.add(exhaustion_factor);
  gtsam::Values exhaustion_initial;
  exhaustion_initial.insert(X(0), gtsam::Pose3());
  options.max_iterations = 50;
  options.relative_tolerance = 1e-6;
  options.absolute_tolerance = 1e-8;
  const auto exhausted = uifgo::RunCheckedConditionalLm(
      exhaustion_graph, exhaustion_initial, options);
  EXPECT_FALSE(exhausted.converged);
  EXPECT_EQ(exhausted.reason, "CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED");
  EXPECT_EQ(exhausted.convergence.iterate_call_count, 1u);
  EXPECT_EQ(exhausted.convergence.lambda_trial_accounting_status,
            "COMPLETE");
  EXPECT_GT(exhausted.convergence.lambda_trial_count, 1u);
  EXPECT_EQ(exhausted.convergence.lambda_trial_count,
            exhaustion_factor->candidate_error_call_count);
  EXPECT_EQ(exhausted.convergence.lambda_trial_count,
            exhausted.convergence.rejected_lambda_trial_count);
  EXPECT_EQ(exhausted.convergence.accepted_update_count, 0u);
  EXPECT_EQ(exhausted.convergence.no_update_return_count, 1u);
  EXPECT_TRUE(exhausted.values.empty());
}

TEST(CheckedConditionalLmPolicyV2,
     StationaryNonzeroResidualExhaustionBoundaryIsPreserved) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  graph.add(boost::make_shared<ConstantResidualFactor>(X(0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3());

  uifgo::CheckedLmOptions options;
  options.max_iterations = 5;
  options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1;
  const auto v1 = uifgo::RunCheckedConditionalLm(graph, initial, options);
  ASSERT_TRUE(v1.converged) << v1.reason;
  EXPECT_EQ(v1.convergence.lambda_trial_accounting_status, "COMPLETE");
  EXPECT_EQ(v1.convergence.convergence_check_count, 1u);
  ASSERT_TRUE(v1.last_qualification_stationarity.valid);
  EXPECT_TRUE(v1.last_qualification_stationarity.stationary);
  EXPECT_DOUBLE_EQ(
      v1.last_qualification_stationarity.max_scaled_gradient_objective, 0.0);

  options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  const auto v2 = uifgo::RunCheckedConditionalLm(graph, initial, options);
  EXPECT_FALSE(v2.converged);
  EXPECT_EQ(v2.reason, "CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED");
  EXPECT_EQ(v2.convergence.lambda_trial_accounting_status, "COMPLETE");
  EXPECT_EQ(v2.convergence.iterate_call_count, 1u);
  EXPECT_EQ(v2.convergence.convergence_check_count, 0u);
  EXPECT_EQ(v2.convergence.qualification_evaluation_count, 0u);
  EXPECT_GT(v2.convergence.lambda_trial_count, 1u);
  EXPECT_EQ(v2.convergence.lambda_trial_count,
            v2.convergence.rejected_lambda_trial_count);
  EXPECT_EQ(v2.convergence.accepted_update_count, 0u);
  EXPECT_TRUE(v2.values.empty());
}

TEST(CheckedConditionalLmPolicy,
     ExceptionAccountingUsesConfirmedCountsAndMarksIncompleteTrial) {
  using gtsam::symbol_shorthand::X;
  const auto run_for_both_policies = [](auto make_throwing_factor) {
    for (const auto policy : {
             uifgo::ConditionalLmPolicy::
                 GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1,
             uifgo::ConditionalLmPolicy::
                 GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2}) {
      gtsam::NonlinearFactorGraph graph;
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(
          X(0), gtsam::Pose3(),
          gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
      const auto factor = make_throwing_factor(X(0));
      graph.add(factor);
      gtsam::Values initial;
      initial.insert(X(0), gtsam::Pose3());
      uifgo::CheckedLmOptions options;
      options.max_iterations = 5;
      options.policy = policy;
      const auto result =
          uifgo::RunCheckedConditionalLm(graph, initial, options);
      EXPECT_FALSE(result.converged);
      EXPECT_EQ(result.reason.find("CONDITIONAL_LM_EXCEPTION:"), 0u);
      EXPECT_EQ(result.convergence.iterate_call_count, 1u);
      EXPECT_EQ(result.convergence.lambda_trial_count, 0u);
      EXPECT_EQ(result.convergence.rejected_lambda_trial_count, 0u);
      EXPECT_EQ(result.convergence.accepted_update_count, 0u);
      EXPECT_EQ(result.convergence.no_update_return_count, 0u);
      EXPECT_EQ(result.convergence.lambda_trial_accounting_status,
                "INCOMPLETE_EXCEPTION_DURING_ITERATE");
      EXPECT_EQ(result.convergence.convergence_check_count, 0u);
      EXPECT_EQ(result.convergence.generic_convergence_count, 0u);
      EXPECT_EQ(result.convergence.qualification_evaluation_count, 0u);
      EXPECT_FALSE(result.convergence.check_evaluated);
      EXPECT_FALSE(result.convergence.qualification_last_evaluated);
      EXPECT_FALSE(result.last_qualification_stationarity.valid);
      EXPECT_TRUE(result.values.empty());
      make_throwing_factor.assert_calls(factor);
    }
  };

  struct LinearizeFixture {
    boost::shared_ptr<ThrowOnLinearizeFactor> operator()(gtsam::Key key) const {
      return boost::make_shared<ThrowOnLinearizeFactor>(key);
    }
    void assert_calls(
        const boost::shared_ptr<ThrowOnLinearizeFactor>& factor) const {
      EXPECT_EQ(factor->linearize_call_count, 1u);
      // Only the optimizer-construction error evaluation ran. The fixture
      // proves no candidate-error evaluation and hence no lambda trial began.
      EXPECT_EQ(factor->ordinary_error_call_count, 1u);
    }
  };
  run_for_both_policies(LinearizeFixture{});

  struct TrialFixture {
    boost::shared_ptr<ThrowDuringTrialErrorFactor> operator()(
        gtsam::Key key) const {
      return boost::make_shared<ThrowDuringTrialErrorFactor>(key);
    }
    void assert_calls(
        const boost::shared_ptr<ThrowDuringTrialErrorFactor>& factor) const {
      EXPECT_EQ(factor->linearize_call_count, 1u);
      EXPECT_EQ(factor->initial_error_call_count, 1u);
      // The linked optimizer entered tryLambda and evaluated one tentative
      // Values, but threw before committing its inner counter. The wrapper's
      // exact total is therefore intentionally INCOMPLETE, not fabricated.
      EXPECT_EQ(factor->candidate_error_call_count, 1u);
    }
  };
  run_for_both_policies(TrialFixture{});
}

TEST(CheckedConditionalLmPolicy,
     ExceptionAccountingPreservesEarlierConfirmedNormalReturnCounts) {
  using gtsam::symbol_shorthand::X;
  for (const auto policy : {
           uifgo::ConditionalLmPolicy::
               GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1,
           uifgo::ConditionalLmPolicy::
               GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2}) {
    gtsam::NonlinearFactorGraph graph;
    const auto factor = boost::make_shared<ThrowOnSecondTrialErrorFactor>(X(0));
    graph.add(factor);
    gtsam::Values initial;
    initial.insert(X(0), gtsam::Pose3());
    uifgo::CheckedLmOptions options;
    options.max_iterations = 3;
    options.policy = policy;
    const auto result =
        uifgo::RunCheckedConditionalLm(graph, initial, options);
    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.reason.find("CONDITIONAL_LM_EXCEPTION:"), 0u);
    EXPECT_EQ(factor->candidate_error_call_count, 2u);
    EXPECT_EQ(result.convergence.iterate_call_count, 2u);
    EXPECT_EQ(result.convergence.lambda_trial_count, 1u);
    EXPECT_EQ(result.convergence.rejected_lambda_trial_count, 0u);
    EXPECT_EQ(result.convergence.accepted_update_count, 1u);
    EXPECT_EQ(result.convergence.no_update_return_count, 0u);
    EXPECT_EQ(result.convergence.lambda_trial_accounting_status,
              "INCOMPLETE_EXCEPTION_DURING_ITERATE");
    EXPECT_TRUE(result.values.empty());
  }
}

TEST(A02ParameterConsistency,
     DefaultPolicyIgnoresUnusedConditionalStationarityCopies) {
  uifgo::DiscoveryOptions baseline;
  baseline.active_bias_min_m = 0.1;
  baseline.short_min_count = 1;
  baseline.short_min_duration_s = 0.0;
  const auto original = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.4)}, baseline);
  auto conflicting_but_unused = baseline;
  for (size_t field_index = 0; field_index < 7; ++field_index)
    IntroduceA02NavigationParameterMismatch(field_index,
                                             &conflicting_but_unused);
  const auto compatible = uifgo::BuildAutomaticSupportPartition(
      {Obs(1, 0.0, 0.4)}, conflicting_but_unused);
  EXPECT_EQ(original.solver_config_hash, compatible.solver_config_hash);
  EXPECT_EQ(original.discovery_context_hash,
            compatible.discovery_context_hash);
  EXPECT_EQ(original.discovery_snapshot_hash,
            compatible.discovery_snapshot_hash);
  EXPECT_EQ(original.partition_hash, compatible.partition_hash);
}

TEST(DiscoveryStopping, CombinedStepUsesGtsamLocalCoordinatesAndBiasScale) {
  using gtsam::symbol_shorthand::B;
  using gtsam::symbol_shorthand::V;
  using gtsam::symbol_shorthand::X;
  gtsam::Values before;
  before.insert(X(0), gtsam::Pose3());
  const gtsam::Vector3 zero_velocity = gtsam::Vector3::Zero();
  before.insert(V(0), zero_velocity);
  before.insert(B(0), gtsam::imuBias::ConstantBias());
  gtsam::Values after = before;
  after.update(X(0), before.at<gtsam::Pose3>(X(0)).retract(
      (gtsam::Vector(6) << 0.02, 0.0, 0.0, 0.3, 0.0, 0.0).finished()));
  after.update(V(0), gtsam::Vector3(0.0, 0.4, 0.0));
  uifgo::NavigationScales scales;
  scales.pose_rotation_rad = 0.1;
  scales.pose_translation_m = 2.0;
  scales.velocity_mps = 2.0;
  const auto audit = uifgo::AuditNavigationAndBiasScaledStep(
      before, after, {0.0, 0.5}, {0.1, 0.9}, scales, 2.0);
  ASSERT_TRUE(audit.valid) << audit.reason;
  EXPECT_NEAR(audit.max_navigation_step, 0.2, 1e-14);
  EXPECT_NEAR(audit.max_bias_step, 0.2, 1e-14);
  EXPECT_NEAR(audit.max_combined_step, 0.2, 1e-14);
}

TEST(CheckedConditionalLmDiagnostics,
     TriggerPredicatesRecomputeAuthoritativeLinkedGtsamDecision) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(
      gtsam::Rot3::RzRyRx(0.3, -0.2, 0.1),
      gtsam::Point3(1.0, -2.0, 0.5)));
  uifgo::CheckedLmOptions options;
  options.max_iterations = 20;
  options.relative_tolerance = 1.0;
  options.absolute_tolerance = 1e9;
  const auto result =
      uifgo::RunCheckedConditionalLm(graph, initial, options);
  ASSERT_TRUE(result.converged) << result.reason;
  const auto& audit = result.convergence;
  ASSERT_TRUE(audit.initial_error_valid);
  ASSERT_TRUE(audit.check_evaluated);
  EXPECT_EQ(audit.check_status, "EVALUATED_MATCHED_LINKED_GTSAM");
  EXPECT_DOUBLE_EQ(audit.initial_error, graph.error(initial));
  EXPECT_DOUBLE_EQ(audit.absolute_decrease,
                   audit.previous_error - audit.current_error);
  EXPECT_EQ(audit.relative_decrease_valid,
            std::isfinite(audit.absolute_decrease / audit.previous_error));
  if (audit.relative_decrease_valid) {
    EXPECT_DOUBLE_EQ(audit.relative_decrease,
                     audit.absolute_decrease / audit.previous_error);
  }
  EXPECT_DOUBLE_EQ(audit.relative_tolerance, options.relative_tolerance);
  EXPECT_DOUBLE_EQ(audit.absolute_tolerance, options.absolute_tolerance);
  EXPECT_DOUBLE_EQ(audit.error_tolerance, 0.0);
  EXPECT_TRUE(audit.relative_tolerance_enabled);
  const bool union_of_recorded_predicates =
      audit.error_tolerance_triggered ||
      audit.absolute_tolerance_triggered ||
      audit.relative_tolerance_triggered;
  EXPECT_EQ(audit.check_result, union_of_recorded_predicates);
  EXPECT_TRUE(audit.predicate_union_matches_check_result);
  EXPECT_TRUE(audit.check_result);
  EXPECT_TRUE(audit.absolute_tolerance_triggered);
  EXPECT_TRUE(audit.relative_tolerance_triggered);
  EXPECT_GE(audit.added_diagnostics_seconds, 0.0);
  EXPECT_GT(result.iterations, 0u);
  EXPECT_GE(result.inner_iterations, 0);
  EXPECT_TRUE(std::isfinite(result.lambda));
  EXPECT_EQ(audit.lambda_trial_accounting_status, "COMPLETE");
  EXPECT_EQ(audit.lambda_trial_count,
            static_cast<size_t>(result.inner_iterations));
  EXPECT_EQ(audit.accepted_update_count, result.iterations);
  EXPECT_EQ(audit.rejected_lambda_trial_count,
            audit.lambda_trial_count - audit.accepted_update_count);
  EXPECT_EQ(audit.no_update_return_count, 0u);
}

TEST(CheckedConditionalLmDiagnostics,
     OneIterationLimitIsNotMisreportedAsConvergence) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(
      gtsam::Rot3::RzRyRx(0.4, 0.2, -0.3),
      gtsam::Point3(4.0, -3.0, 2.0)));
  uifgo::CheckedLmOptions options;
  options.max_iterations = 1;
  options.relative_tolerance = 0.0;
  options.absolute_tolerance = 0.0;
  const auto result =
      uifgo::RunCheckedConditionalLm(graph, initial, options);
  ASSERT_FALSE(result.converged);
  EXPECT_EQ(result.reason, "CONDITIONAL_LM_MAX_ITERATIONS");
  ASSERT_TRUE(result.convergence.check_evaluated);
  EXPECT_FALSE(result.convergence.relative_tolerance_enabled);
  EXPECT_FALSE(result.convergence.relative_tolerance_triggered);
  EXPECT_FALSE(result.convergence.check_result);
  EXPECT_TRUE(result.convergence.predicate_union_matches_check_result);
  EXPECT_EQ(result.iterations, 1u);
}

TEST(CheckedConditionalLmDiagnostics,
     OptionalBranchAndFiniteDifferenceCapturePreservesSolverResult) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(
      gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3),
      gtsam::Point3(0.4, -0.5, 0.6)));
  uifgo::CheckedLmOptions options;
  options.max_iterations = 3;
  options.relative_tolerance = 1.0;
  options.absolute_tolerance = 1e9;

  const auto baseline =
      uifgo::RunCheckedConditionalLm(graph, initial, options);
  uifgo::CheckedLmDiagnosticRequest request;
  request.finite_difference_steps = {1e-3, 1e-4, 1e-5, 1e-6};
  const auto diagnosed =
      uifgo::RunCheckedConditionalLm(graph, initial, options, &request);

  ASSERT_TRUE(baseline.converged) << baseline.reason;
  ASSERT_TRUE(diagnosed.converged) << diagnosed.reason;
  EXPECT_FALSE(baseline.diagnostic.requested);
  EXPECT_EQ(diagnosed.reason, baseline.reason);
  EXPECT_EQ(diagnosed.iterations, baseline.iterations);
  EXPECT_EQ(diagnosed.inner_iterations, baseline.inner_iterations);
  EXPECT_DOUBLE_EQ(diagnosed.lambda, baseline.lambda);
  EXPECT_DOUBLE_EQ(graph.error(diagnosed.values), graph.error(baseline.values));
  EXPECT_LT(baseline.values.localCoordinates(diagnosed.values).norm(), 1e-15);

  const auto& capture = diagnosed.diagnostic;
  ASSERT_TRUE(capture.requested);
  ASSERT_TRUE(capture.valid) << capture.reason;
  EXPECT_EQ(capture.graph_factor_count, graph.size());
  EXPECT_EQ(capture.values_key_count, initial.size());
  ASSERT_EQ(capture.calls.size(), 1u);
  EXPECT_TRUE(capture.calls.front().first_try.valid);
  EXPECT_EQ(capture.calls.front().optimizer_iterations_after,
            diagnosed.iterations);
  EXPECT_TRUE(capture.calls.front().accepted_state_update);
  EXPECT_EQ(capture.factors.size(), graph.size());
  ASSERT_TRUE(capture.finite_difference.valid)
      << capture.finite_difference.reason;
  ASSERT_TRUE(capture.stationarity_at_final.valid)
      << capture.stationarity_at_final.reason;
  EXPECT_EQ(capture.finite_difference.points.size(),
            request.finite_difference_steps.size());
  EXPECT_TRUE(std::any_of(
      capture.finite_difference.points.begin(),
      capture.finite_difference.points.end(),
      [](const auto& point) { return point.agrees; }));
}

TEST(CheckedConditionalLmDiagnostics,
     ActualLinkedTryDeltaIsCompletePreciseAndRoundTripsNavigationKeys) {
  using gtsam::symbol_shorthand::B;
  using gtsam::symbol_shorthand::V;
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      V(0), gtsam::Vector3::Zero(),
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0)));
  graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(0), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(
      gtsam::Rot3::RzRyRx(0.03, -0.02, 0.01),
      gtsam::Point3(0.4, -0.2, 0.1)));
  initial.insert(V(0), gtsam::Vector3(0.2, -0.1, 0.3));
  initial.insert(B(0), gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.01, -0.02, 0.03),
      gtsam::Vector3(-0.04, 0.05, -0.06)));

  uifgo::CheckedLmOptions options;
  options.max_iterations = 2;
  options.relative_tolerance = 1.0;
  options.absolute_tolerance = 1e9;
  const auto baseline =
      uifgo::RunCheckedConditionalLm(graph, initial, options);
  uifgo::CheckedLmDiagnosticRequest request;
  request.emit_linked_gtsam_trylambda = true;
  request.capture_linked_gtsam_trydelta = true;
  request.direction_evaluation_lambdas = {1e-5};
  request.direction_finite_difference_steps = {1e-4, 1e-5, 1e-6};
  const auto diagnosed =
      uifgo::RunCheckedConditionalLm(graph, initial, options, &request);

  ASSERT_TRUE(baseline.converged) << baseline.reason;
  ASSERT_TRUE(diagnosed.converged) << diagnosed.reason;
  EXPECT_EQ(diagnosed.reason, baseline.reason);
  EXPECT_EQ(diagnosed.iterations, baseline.iterations);
  EXPECT_EQ(diagnosed.inner_iterations, baseline.inner_iterations);
  EXPECT_DOUBLE_EQ(diagnosed.lambda, baseline.lambda);
  EXPECT_DOUBLE_EQ(graph.error(diagnosed.values), graph.error(baseline.values));
  EXPECT_LT(baseline.values.localCoordinates(diagnosed.values).norm(), 1e-15);

  const auto& capture = diagnosed.diagnostic;
  ASSERT_EQ(capture.calls.size(), 1u);
  const auto& call = capture.calls.front();
  ASSERT_FALSE(call.linked_trydelta_stdout.empty());
  EXPECT_NE(call.linked_trydelta_stdout.find("delta: 3 elements"),
            std::string::npos);
  EXPECT_NE(call.linked_trydelta_stdout.find("x0:"), std::string::npos);
  EXPECT_NE(call.linked_trydelta_stdout.find("v0:"), std::string::npos);
  EXPECT_NE(call.linked_trydelta_stdout.find("b0:"), std::string::npos);
  ASSERT_EQ(call.actual_trial_directions.size(), 1u);
  const auto& trial = call.actual_trial_directions.front();
  ASSERT_TRUE(trial.parsed) << trial.parse_reason;
  EXPECT_EQ(trial.parse_reason,
            "OK_COMPLETE_KEYS_DIMENSIONS_MAX_DIGITS10");
  EXPECT_EQ(trial.declared_key_count, 3u);
  EXPECT_EQ(trial.parsed_key_count, 3u);
  EXPECT_EQ(trial.parsed_dimension_count, 15u);
  EXPECT_TRUE(trial.delta.exists(X(0)));
  EXPECT_TRUE(trial.delta.exists(V(0)));
  EXPECT_TRUE(trial.delta.exists(B(0)));
  EXPECT_EQ(trial.delta.at(X(0)).size(), 6);
  EXPECT_EQ(trial.delta.at(V(0)).size(), 3);
  EXPECT_EQ(trial.delta.at(B(0)).size(), 6);
  EXPECT_NEAR(trial.linked_reported_delta_norm,
              trial.parsed_delta_norm, 2e-16);
  const gtsam::VectorValues accepted_delta =
      call.values_before.localCoordinates(capture.values_at_final);
  EXPECT_TRUE(accepted_delta.equals(trial.delta, 2e-15));
  const gtsam::Values parsed_tentative = call.values_before.retract(trial.delta);
  const gtsam::VectorValues round_trip =
      call.values_before.localCoordinates(parsed_tentative);
  EXPECT_TRUE(round_trip.equals(trial.delta, 2e-15));
  ASSERT_TRUE(trial.selected_for_evaluation);
  ASSERT_TRUE(trial.evaluation_valid) << trial.evaluation_reason;
  ASSERT_EQ(trial.directional_finite_difference.size(), 3u);
  EXPECT_TRUE(std::all_of(
      trial.directional_finite_difference.begin(),
      trial.directional_finite_difference.end(),
      [](const auto& point) { return point.agrees; }));
  EXPECT_EQ(trial.factors.size(), graph.size());
  long double factor_gradient_sum = 0.0L;
  std::vector<long double> factor_fd_sums(3, 0.0L);
  for (const auto& factor : trial.factors) {
    ASSERT_TRUE(factor.directional_derivative_valid)
        << factor.factor_index << ':' << factor.directional_derivative_reason;
    ASSERT_EQ(factor.directional_finite_difference.size(), 3u);
    factor_gradient_sum +=
        static_cast<long double>(factor.gradient_dot_unit_direction);
    for (size_t i = 0; i < factor.directional_finite_difference.size(); ++i)
      factor_fd_sums[i] += static_cast<long double>(
          factor.directional_finite_difference[i].central_derivative);
  }
  EXPECT_NEAR(static_cast<double>(factor_gradient_sum),
              trial.gradient_dot_unit_direction, 2e-13);
  for (size_t i = 0; i < trial.directional_finite_difference.size(); ++i)
    EXPECT_NEAR(static_cast<double>(factor_fd_sums[i]),
                trial.directional_finite_difference[i].central_derivative,
                2e-10);
}

TEST(CheckedConditionalLmDiagnostics,
     FixedCheckpointRecoveryIsExplicitShadowAndUsesMatchedStart) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(
      gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3),
      gtsam::Point3(0.4, -0.5, 0.6)));
  uifgo::CheckedLmOptions options;
  options.max_iterations = 3;
  options.relative_tolerance = 1.0;
  options.absolute_tolerance = 1e9;
  uifgo::CheckedLmDiagnosticRequest request;
  request.finite_difference_steps = {1e-3, 1e-4, 1e-5};
  request.run_fixed_checkpoint_lm_recovery = true;
  request.fixed_checkpoint_max_calls = 3;
  request.fixed_checkpoint_max_seconds = 10.0;

  const auto result =
      uifgo::RunCheckedConditionalLm(graph, initial, options, &request);
  ASSERT_TRUE(result.converged) << result.reason;
  const auto& recovery = result.diagnostic.fixed_checkpoint_recovery;
  ASSERT_TRUE(recovery.requested);
  ASSERT_TRUE(recovery.valid) << recovery.reason;
  EXPECT_EQ(recovery.max_calls_per_arm, 3u);
  EXPECT_DOUBLE_EQ(recovery.max_seconds_per_arm, 10.0);
  EXPECT_DOUBLE_EQ(recovery.current_behavior.initial_error,
                   recovery.captured_error);
  EXPECT_DOUBLE_EQ(recovery.continue_lambda_search.initial_error,
                   recovery.captured_error);
  EXPECT_DOUBLE_EQ(recovery.current_behavior.initial_lambda,
                   recovery.captured_lambda);
  EXPECT_DOUBLE_EQ(recovery.continue_lambda_search.initial_lambda,
                   recovery.captured_lambda);
  EXPECT_FALSE(
      recovery.current_behavior.internal_relative_tolerance_changed);
  EXPECT_DOUBLE_EQ(recovery.current_behavior.internal_relative_tolerance,
                   options.relative_tolerance);
  EXPECT_TRUE(
      recovery.continue_lambda_search.internal_relative_tolerance_changed);
  EXPECT_DOUBLE_EQ(
      recovery.continue_lambda_search.internal_relative_tolerance, 0.0);
  EXPECT_DOUBLE_EQ(
      recovery.continue_lambda_search.external_relative_tolerance,
      options.relative_tolerance);
  EXPECT_DOUBLE_EQ(
      recovery.continue_lambda_search.external_absolute_tolerance,
      options.absolute_tolerance);
  EXPECT_LE(recovery.current_behavior.call_count, 3u);
  EXPECT_LE(recovery.continue_lambda_search.call_count, 3u);
  EXPECT_LE(recovery.current_behavior.elapsed_seconds, 10.0);
  EXPECT_LE(recovery.continue_lambda_search.elapsed_seconds, 10.0);
}

TEST(CheckedConditionalLmPolicy,
     GenericConvergenceContinuesUntilStationarityOrOriginalBudget) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(
      gtsam::Rot3::RzRyRx(0.4, 0.2, -0.3),
      gtsam::Point3(4.0, -3.0, 2.0)));
  uifgo::CheckedLmOptions options;
  options.max_iterations = 2;
  options.relative_tolerance = 1.0;
  options.absolute_tolerance = 1e9;
  options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1;
  options.navigation_stationarity_tolerance_objective = 0.0;
  options.gradient_roundoff_safety_factor = 1.0;
  const auto result =
      uifgo::RunCheckedConditionalLm(graph, initial, options);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.reason, "CONDITIONAL_LM_STATIONARITY_NOT_REACHED");
  EXPECT_EQ(result.iterations, 2u);
  EXPECT_EQ(result.convergence.convergence_check_count, 2u);
  EXPECT_EQ(result.convergence.generic_convergence_count, 2u);
  EXPECT_EQ(result.convergence.qualification_evaluation_count, 2u);
  EXPECT_TRUE(result.convergence.qualification_last_evaluated);
  EXPECT_FALSE(result.convergence.qualification_last_passed);
  EXPECT_EQ(result.convergence.qualification_status,
            "NOT_STATIONARY_BUDGET_EXHAUSTED");
  EXPECT_TRUE(result.last_qualification_stationarity.valid);
  EXPECT_FALSE(result.last_qualification_stationarity.stationary);
  EXPECT_EQ(result.convergence.qualification_last_iteration, 2u);
  EXPECT_GE(result.convergence.qualification_seconds, 0.0);
}

TEST(CheckedConditionalLmPolicy,
     SimultaneousGenericAndStationarityStopMatchesDefault) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3());
  uifgo::CheckedLmOptions baseline_options;
  baseline_options.max_iterations = 2;
  baseline_options.relative_tolerance = 1.0;
  baseline_options.absolute_tolerance = 1e9;
  const auto baseline =
      uifgo::RunCheckedConditionalLm(graph, initial, baseline_options);
  auto qualified_options = baseline_options;
  qualified_options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1;
  const auto qualified =
      uifgo::RunCheckedConditionalLm(graph, initial, qualified_options);
  ASSERT_TRUE(baseline.converged) << baseline.reason;
  ASSERT_TRUE(qualified.converged) << qualified.reason;
  EXPECT_EQ(baseline.iterations, qualified.iterations);
  EXPECT_EQ(qualified.reason,
            "CONDITIONAL_LM_CONVERGED_AND_NAVIGATION_STATIONARY");
  EXPECT_TRUE(qualified.last_qualification_stationarity.valid);
  EXPECT_TRUE(qualified.last_qualification_stationarity.stationary);
  const gtsam::VectorValues delta =
      baseline.values.localCoordinates(qualified.values);
  EXPECT_LT(delta.norm(), 1e-12);
}

TEST(CheckedConditionalLmPolicy,
     InvalidStationarityAuditFailsWithoutReturningValues) {
  const gtsam::Key unknown = gtsam::Symbol('d', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<double>(
      unknown, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0)));
  gtsam::Values initial;
  initial.insert<double>(unknown, 1.0);
  uifgo::CheckedLmOptions options;
  options.max_iterations = 2;
  options.relative_tolerance = 1.0;
  options.absolute_tolerance = 1e9;
  options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1;
  const auto result =
      uifgo::RunCheckedConditionalLm(graph, initial, options);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.reason.find("CONDITIONAL_LM_STATIONARITY_AUDIT_INVALID:"),
            0u);
  EXPECT_TRUE(result.values.empty());
  EXPECT_TRUE(result.convergence.qualification_last_evaluated);
  EXPECT_EQ(result.convergence.qualification_evaluation_count, 1u);
  EXPECT_FALSE(result.last_qualification_stationarity.valid);
  EXPECT_EQ(result.last_qualification_stationarity.reason.find(
                "no declared scale for free key"), 0u);
}

TEST(CheckedConditionalLmPolicy,
     ZeroPreviousErrorPreservesMathematicalPredicatesAndShortCircuit) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3());
  uifgo::CheckedLmOptions options;
  options.max_iterations = 1;
  options.relative_tolerance = 1.0;
  options.absolute_tolerance = 1e-8;
  const auto result =
      uifgo::RunCheckedConditionalLm(graph, initial, options);
  ASSERT_TRUE(result.converged) << result.reason;
  EXPECT_DOUBLE_EQ(result.convergence.previous_error, 0.0);
  EXPECT_DOUBLE_EQ(result.convergence.current_error, 0.0);
  EXPECT_TRUE(result.convergence.error_tolerance_triggered);
  EXPECT_TRUE(result.convergence.absolute_tolerance_triggered);
  EXPECT_FALSE(result.convergence.relative_decrease_valid);
  EXPECT_FALSE(result.convergence.relative_tolerance_triggered);
  EXPECT_FALSE(
      result.convergence.decrease_predicates_reached_by_linked_check);
  EXPECT_TRUE(result.convergence.check_result);
  EXPECT_TRUE(result.convergence.predicate_union_matches_check_result);
}

TEST(CheckedConditionalLmPolicy,
     NonfiniteAndExceptionPathsCannotReportSuccess) {
  using gtsam::symbol_shorthand::V;
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  uifgo::CheckedLmOptions options;
  options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1;

  gtsam::Values nonfinite;
  nonfinite.insert(X(0), gtsam::Pose3(
      gtsam::Rot3(), gtsam::Point3(
          std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)));
  const auto nonfinite_result =
      uifgo::RunCheckedConditionalLm(graph, nonfinite, options);
  EXPECT_FALSE(nonfinite_result.converged);
  EXPECT_EQ(nonfinite_result.reason,
            "CONDITIONAL_LM_INITIAL_OBJECTIVE_NONFINITE");

  gtsam::Values mismatched;
  const gtsam::Vector3 zero_velocity = gtsam::Vector3::Zero();
  mismatched.insert(V(0), zero_velocity);
  const auto exception_result =
      uifgo::RunCheckedConditionalLm(graph, mismatched, options);
  EXPECT_FALSE(exception_result.converged);
  EXPECT_EQ(exception_result.reason.find("CONDITIONAL_LM_EXCEPTION:"), 0u);
  EXPECT_EQ(exception_result.convergence.iterate_call_count, 0u);
  EXPECT_EQ(exception_result.convergence.lambda_trial_accounting_status,
            "NOT_EXECUTED");
  EXPECT_EQ(exception_result.convergence.lambda_trial_count, 0u);
  EXPECT_EQ(exception_result.convergence.rejected_lambda_trial_count, 0u);
  EXPECT_TRUE(exception_result.values.empty());

  options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  const auto v2_exception_result =
      uifgo::RunCheckedConditionalLm(graph, mismatched, options);
  EXPECT_FALSE(v2_exception_result.converged);
  EXPECT_EQ(v2_exception_result.reason.find("CONDITIONAL_LM_EXCEPTION:"), 0u);
  EXPECT_EQ(v2_exception_result.convergence.iterate_call_count, 0u);
  EXPECT_EQ(v2_exception_result.convergence.lambda_trial_accounting_status,
            "NOT_EXECUTED");
  EXPECT_EQ(v2_exception_result.convergence.lambda_trial_count, 0u);
  EXPECT_EQ(v2_exception_result.convergence.rejected_lambda_trial_count, 0u);
  EXPECT_TRUE(v2_exception_result.values.empty());
}

TEST(Stage1RegularizedResult, UsesOnePhysicalGraphAndOnlyValidPlannedBiases) {
  using gtsam::symbol_shorthand::X;
  uifgo::Config cfg;
  cfg.calib_anchor = false;
  cfg.calib_lever = false;
  cfg.calib_range_bias = false;
  cfg.calib_td = false;
  cfg.anchors.push_back({2, gtsam::Point3(1.0, 0.0, 0.0), 0.1});
  cfg.lever_arm_init = gtsam::Point3(0.0, 0.0, 0.0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  graph.add(uifgo::MakeUwbFactor(
      X(0), gtsam::Symbol('l', 0), gtsam::Symbol('a', 2),
      gtsam::Symbol('z', 2), cfg.anchors[0].pos, cfg.lever_arm_init,
      1.4, 1.0, false, false, false, 0.0));
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3());
  uifgo::FactorMeta metadata;
  metadata.factor_index = 1;
  metadata.obs_id = 7;
  metadata.factor_type = "uwb_range";
  metadata.keys = {X(0)};
  uifgo::PaperInputPlan plan;
  plan.plan_sha256 = "sha256:plan";
  uifgo::ObservationRecord valid;
  valid.obs_id = 7;
  valid.sensor_time = 0.0;
  valid.tag_id = 1;
  valid.anchor_id = 2;
  valid.raw_range = 1.4;
  valid.valid = true;
  valid.planned = true;
  valid.keyframe_id = 0;
  valid.nominal_sigma = 1.0;
  auto invalid = valid;
  invalid.obs_id = 8;
  invalid.valid = false;
  invalid.planned = false;
  plan.observations = {valid, invalid};
  uifgo::DiscoveryResult discovery;
  discovery.status = uifgo::DiscoveryStatus::CONVERGED;
  discovery.navigation_values = values;
  discovery.snapshot = {Obs(7, 0.0, 0.4), Obs(8, 1.0, 0.9)};
  discovery.partition.partition_hash = "sha256:partition";
  uifgo::DiscoveryContext context;
  context.input_plan_hash = plan.plan_sha256;
  context.source_hash = "sha256:source";
  context.config_hash = "sha256:config";
  context.calibration_hash = "sha256:calibration";
  context.solver_config_hash = "sha256:solver";
  uifgo::DiscoveryOptions options;
  options.fused_lasso.lambda_l1 = 0.05;
  options.fused_lasso.lambda_tv = 0.1;
  const auto result = uifgo::BuildStage1RegularizedResult(
      discovery, graph, {metadata}, plan, cfg, context, options);
  ASSERT_TRUE(result.valid) << result.reason;
  ASSERT_EQ(result.observation_bias_snapshot.size(), 1u);
  EXPECT_EQ(result.observation_bias_snapshot[0].obs_id, 7u);
  EXPECT_EQ(result.physical_graph.size(), 2u);
  EXPECT_NEAR(result.objective_physical, 0.0, 1e-14);
  EXPECT_NEAR(result.objective_l1, 0.02, 1e-14);
  EXPECT_NEAR(result.objective_tv, 0.0, 1e-14);
  EXPECT_NEAR(result.objective_total, 0.02, 1e-14);
  EXPECT_EQ(result.covariance_status,
            "NOT_APPLICABLE_REGULARIZED_STAGE1_NON_GAUSSIAN");
  EXPECT_FALSE(result.graph_linearization_sha256.empty());
  EXPECT_FALSE(result.values_sha256.empty());
  EXPECT_FALSE(result.snapshot_sha256.empty());
  EXPECT_FALSE(result.result_id.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(A11FirstBlock, FullDirectionGateRejectsMissingFactorFailedPointAndWrongDelta) {
  uifgo::CheckedLmDiagnosticCapture capture;
  capture.first_block_budget_diagnostic = true;
  capture.graph_factor_count = 2;
  capture.values_key_count = 1;
  capture.values_at_call50.insert(gtsam::symbol_shorthand::X(0), gtsam::Pose3());
  capture.calls.resize(50);
  for (size_t index : {size_t(48), size_t(49)}) {
    auto& call = capture.calls[index];
    call.call_index = index + 1;
    call.accepted_state_update = true;
    call.optimizer_iterations_before = index;
    call.optimizer_iterations_after = index + 1;
    uifgo::CheckedLmTrialDirectionDiagnostics trial;
    trial.parsed = trial.accepted_retract_matches = true;
    trial.accepted_retract_max_difference = 0.0;
    trial.parsed_key_count = 1;
    trial.selected_for_evaluation = trial.evaluation_valid = true;
    for (double step : {1e-4, 1e-5, 1e-6}) {
      uifgo::CheckedLmDirectionFiniteDifferencePoint point;
      point.step = step;
      point.central_derivative = 1.0;
      point.absolute_difference_from_analytic = 0.0;
      point.agreement_tolerance = 5e-9 + 5e-3;
      point.agrees = true;
      trial.directional_finite_difference.push_back(point);
    }
    for (size_t i = 0; i < 2; ++i) {
      uifgo::CheckedLmDirectionFactorDiagnostics factor;
      factor.factor_index = i;
      factor.directional_derivative_valid = true;
      factor.directional_finite_difference = trial.directional_finite_difference;
      trial.factors.push_back(factor);
    }
    call.actual_trial_directions.push_back(trial);
  }
  capture.finite_difference_at_call50.valid = true;
  for (double step : {1e-4, 1e-5, 1e-6}) {
    uifgo::CheckedLmFiniteDifferencePoint point;
    point.step = step;
    point.central_derivative = 1.0;
    point.absolute_difference_from_analytic = 0.0;
    point.agreement_tolerance = 5e-9 + 5e-3;
    point.agrees = true;
    capture.finite_difference_at_call50.points.push_back(point);
  }
  ASSERT_TRUE(uifgo::FirstBlockDerivativesConsistent(capture));
  auto changed = capture;
  changed.calls[48].actual_trial_directions.back().factors.pop_back();
  EXPECT_FALSE(uifgo::FirstBlockDerivativesConsistent(changed));
  changed = capture;
  changed.calls[49].actual_trial_directions.back().factors[1]
      .directional_finite_difference[2].agrees = false;
  EXPECT_FALSE(uifgo::FirstBlockDerivativesConsistent(changed));
  changed = capture;
  changed.calls[48].actual_trial_directions.back().accepted_retract_max_difference = 1e-8;
  EXPECT_FALSE(uifgo::FirstBlockDerivativesConsistent(changed));
  changed = capture;
  changed.calls[48].actual_trial_directions.back().directional_finite_difference.clear();
  EXPECT_FALSE(uifgo::FirstBlockDerivativesConsistent(changed));
  changed = capture;
  changed.finite_difference_at_call50.points[0].agrees = false;
  EXPECT_FALSE(uifgo::FirstBlockDerivativesConsistent(changed));
  changed = capture;
  changed.calls.resize(49);
  EXPECT_FALSE(uifgo::FirstBlockDerivativesConsistent(changed));
}

TEST(A11FirstBlock, EarlyOriginalConvergencePreservesValuesLambdaAndSkipsShadowSolve) {
  using gtsam::symbol_shorthand::X;
  gtsam::NonlinearFactorGraph graph;
  graph.add(uifgo::PaperPosePriorFactor(
      X(0), gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1.0)));
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1, 0.2, 0.3)));
  uifgo::CheckedLmOptions options;
  options.max_iterations = 50;
  options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  const auto baseline = uifgo::RunCheckedConditionalLm(graph, initial, options);
  uifgo::CheckedLmDiagnosticRequest request;
  request.first_block_budget_diagnostic = true;
  request.emit_linked_gtsam_trylambda = true;
  request.capture_linked_gtsam_trydelta = true;
  request.direction_finite_difference_steps = {1e-4, 1e-5, 1e-6};
  request.finite_difference_steps = {1e-4, 1e-5, 1e-6};
  const auto observed = uifgo::RunCheckedConditionalLm(graph, initial, options, &request);
  ASSERT_TRUE(baseline.converged);
  ASSERT_TRUE(observed.converged) << observed.reason;
  EXPECT_EQ(baseline.iterations, observed.iterations);
  EXPECT_DOUBLE_EQ(baseline.lambda, observed.lambda);
  EXPECT_DOUBLE_EQ(baseline.convergence.current_error, observed.convergence.current_error);
  EXPECT_LT(baseline.values.localCoordinates(observed.values).norm(), 1e-15);
  EXPECT_LT(observed.diagnostic.calls.size(), 49u);
  EXPECT_FALSE(observed.diagnostic.derivative_gate_passed);
  for (const auto& call : observed.diagnostic.calls) {
    EXPECT_EQ(call.first_try.reason, "NOT_RUN_A11_NO_SHADOW_SOLVE");
    EXPECT_TRUE(call.stationarity_after.valid);
  }
  options.max_iterations = 200;
  const auto invalid = uifgo::RunCheckedConditionalLm(graph, initial, options, &request);
  EXPECT_EQ(invalid.reason, "A11_INVALID_FROZEN_DIAGNOSTIC_REQUEST");
  EXPECT_EQ(invalid.convergence.iterate_call_count, 0u);
}

TEST(T10A19R05Identity,
     ExactSharedRequestAcceptedAndEveryProtectedFieldFailsClosed) {
  uifgo::DiscoveryOptions options;
  options.conditional_lm.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  const std::string identity = "a19-policy-sha256:r05-engineering";
  uifgo::DiscoveryContext context;
  context.solver_config_hash = identity;
  const auto callback = [](
      size_t, const gtsam::NonlinearFactorGraph&, const gtsam::Values& values,
      const uifgo::CheckedLmOptions&,
      const std::vector<uifgo::DevelopmentRangeConstant>&) {
    uifgo::CheckedLmResult result;
    result.values = values;
    return result;
  };
  const auto request =
      uifgo::MakeA19DevelopmentStage1Request(identity, callback);
  EXPECT_EQ(request.output_schema, uifgo::kA19DevelopmentStage1Schema);
  EXPECT_EQ(request.output_schema,
            "A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY");
  EXPECT_EQ(request.output_provider,
            uifgo::kDevelopmentNonconsumableProvider);
  EXPECT_EQ(request.policy, uifgo::kA19DevelopmentStage1Policy);
  EXPECT_TRUE(uifgo::ValidateDevelopmentStage1Request(
                  request, context, options).accepted);

  const auto reject = [&](const std::string& expected,
                          uifgo::DevelopmentStage1Request changed,
                          uifgo::DiscoveryContext changed_context,
                          uifgo::DiscoveryOptions changed_options,
                          bool diagnostic) {
    const auto check = uifgo::ValidateDevelopmentStage1Request(
        changed, changed_context, changed_options, diagnostic);
    EXPECT_FALSE(check.accepted);
    EXPECT_EQ(check.reason, expected);
  };
  auto changed = request;
  changed.output_schema = "A19_UNKNOWN_DEVELOPMENT_SCHEMA";
  reject("DEVELOPMENT_STAGE1_SCHEMA_REJECTED", changed, context, options,
         false);
  changed = request;
  changed.role = "validation";
  reject("DEVELOPMENT_STAGE1_ROLE_REJECTED", changed, context, options,
         false);
  changed = request;
  changed.output_provider = "formal_cache";
  reject("DEVELOPMENT_STAGE1_PROVIDER_REJECTED", changed, context, options,
         false);
  changed = request;
  changed.policy = "PAPER_UNKNOWN_POLICY";
  reject("DEVELOPMENT_STAGE1_POLICY_REJECTED", changed, context, options,
         false);
  changed = request;
  changed.implementation_identity = "r05-unqualified-identity";
  reject("DEVELOPMENT_STAGE1_IMPLEMENTATION_IDENTITY_REJECTED", changed,
         context, options, false);
  auto wrong_context = context;
  wrong_context.solver_config_hash = "a19-policy-sha256:other";
  reject("DEVELOPMENT_STAGE1_CONTEXT_IDENTITY_REJECTED", request,
         wrong_context, options, false);
  changed = request;
  changed.conditional_navigation = {};
  reject("DEVELOPMENT_STAGE1_CALLBACK_REQUIRED", changed, context, options,
         false);
  auto wrong_options = options;
  wrong_options.conditional_lm.policy =
      uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1;
  reject("DEVELOPMENT_STAGE1_CONDITIONAL_POLICY_REJECTED", request, context,
         wrong_options, false);
  reject("DEVELOPMENT_AND_DIAGNOSTIC_REQUEST_CONFLICT", request, context,
         options, true);
}

TEST(T10A19R08ValidationIdentity,
     ValidationEnvelopeIsBoundAndLegacyRequestCannotCarryIt) {
  uifgo::DiscoveryOptions options;
  options.conditional_lm.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  const std::string identity = "a19-policy-sha256:r08-validation";
  const auto callback = [](
      size_t, const gtsam::NonlinearFactorGraph&, const gtsam::Values& values,
      const uifgo::CheckedLmOptions&,
      const std::vector<uifgo::DevelopmentRangeConstant>&) {
    uifgo::CheckedLmResult result;
    result.values = values;
    return result;
  };
  uifgo::DevelopmentStage1Request request;
  request.policy = uifgo::kA19DevelopmentStage1Policy;
  request.implementation_identity = identity;
  request.role = uifgo::kValidationRole;
  request.output_schema = uifgo::kA19ValidationStage1Schema;
  request.output_provider = uifgo::kValidationNonconsumableProvider;
  request.validation_context_sha256 = "sha256:validation-context";
  request.conditional_navigation = callback;
  uifgo::DiscoveryContext context;
  context.solver_config_hash = identity;
  context.validation_context_sha256 = request.validation_context_sha256;
  EXPECT_TRUE(uifgo::ValidateDevelopmentStage1Request(
                  request, context, options).accepted);
  request.validation_context_sha256 = "sha256:wrong";
  EXPECT_EQ(uifgo::ValidateDevelopmentStage1Request(
                request, context, options).reason,
            "VALIDATION_STAGE1_CONTEXT_BINDING_REJECTED");
  auto legacy = uifgo::MakeA19DevelopmentStage1Request(identity, callback);
  legacy.validation_context_sha256 = context.validation_context_sha256;
  EXPECT_EQ(uifgo::ValidateDevelopmentStage1Request(
                legacy, context, options).reason,
            "DEVELOPMENT_STAGE1_VALIDATION_CONTEXT_REJECTED");
}
