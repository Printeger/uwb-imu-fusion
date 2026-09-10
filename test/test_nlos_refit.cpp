#include "uifgo/nlos_refit.h"
#include "uifgo/nlos_discovery.h"
#include "uifgo/nlos_scoring.h"
#include "uifgo/nlos_inference.h"
#include "uifgo/nlos_inference_io.h"
#include "uifgo/hash_utils.h"
#include "uifgo/paper_run_io.h"
#include "uifgo/uwb_factor.h"

#include <boost/filesystem.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>

namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::Z;

struct TempYaml {
  boost::filesystem::path path;
  explicit TempYaml(const std::string& contents) {
    path = boost::filesystem::temp_directory_path() /
           boost::filesystem::unique_path("uifgo-t04-%%%%-%%%%.yaml");
    std::ofstream out(path.string());
    out << contents;
  }
  ~TempYaml() { boost::filesystem::remove(path); }
};

uifgo::PaperInputPlan ParserPlan() {
  uifgo::PaperInputPlan plan;
  plan.keyframes = {{0, 0, 1.0, 7}, {1, 1, 2.0, 7}};
  for (size_t i = 0; i < 4; ++i) {
    uifgo::ObservationRecord record;
    record.obs_id = 100 + i;
    record.sensor_time = 1.0 + (i / 2);
    record.tag_id = 7;
    record.anchor_id = (i % 2) + 1;
    record.raw_range = 4.0;
    record.valid = true;
    record.planned = true;
    record.keyframe_id = i / 2;
    record.nominal_sigma = 0.1;
    plan.observations.push_back(record);
  }
  return plan;
}

struct RefitFixture {
  uifgo::Config cfg;
  uifgo::PaperInputPlan plan;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::vector<uifgo::FactorMeta> metadata;
  uifgo::OracleSupport support;
};

RefitFixture MakeRefitFixture(double candidate_amplitude) {
  RefitFixture fixture;
  fixture.cfg.calib_lever = false;
  fixture.cfg.calib_anchor = false;
  fixture.cfg.calib_range_bias = false;
  fixture.cfg.calib_td = false;
  fixture.cfg.lever_arm_init = gtsam::Point3(0.12, -0.04, 0.03);
  fixture.cfg.anchors = {
      {1, gtsam::Point3(-4, -3, 1), 0.1},
      {2, gtsam::Point3(5, -3, 2), 0.1},
      {3, gtsam::Point3(-3, 5, 3), 0.1},
      {4, gtsam::Point3(4, 4, -1), 0.1},
  };
  fixture.plan.keyframes = {{0, 0, 0.0, 7}, {1, 1, 1.0, 7}};
  fixture.support.schema = "t04_oracle_support_v1";
  uifgo::OracleSegment segment;
  segment.segment_id = "shared-anchor-1";
  segment.tag_id = 7;
  segment.anchor_id = 1;
  segment.segment_ordinal = 0;
  segment.start_time = 0.0;
  segment.end_time = 1.0;
  segment.duration = 1.0;

  const std::vector<gtsam::Pose3> truth = {
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.08, -0.04, 0.15),
                   gtsam::Point3(1.0, 1.5, 0.6)),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.10, -0.03, 0.20),
                   gtsam::Point3(1.4, 1.8, 0.7)),
  };
  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
  auto vector_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  std::uint64_t obs_id = 1000;
  for (size_t k = 0; k < truth.size(); ++k) {
    fixture.graph.addPrior(X(k), truth[k], pose_noise);
    const gtsam::Vector3 zero_velocity = gtsam::Vector3::Zero();
    fixture.graph.addPrior(V(k), zero_velocity, vector_noise);
    fixture.graph.addPrior(
        B(k), gtsam::imuBias::ConstantBias(), bias_noise);
    fixture.values.insert(
        X(k), truth[k].retract((gtsam::Vector(6) << 0.02, -0.01, 0.015,
                               0.05, -0.04, 0.03)
                                  .finished()));
    fixture.values.insert(V(k), gtsam::Vector3(0.01, -0.01, 0.02));
    fixture.values.insert(B(k), gtsam::imuBias::ConstantBias());
    for (const auto& anchor : fixture.cfg.anchors) {
      const double geometric =
          (truth[k].transformFrom(fixture.cfg.lever_arm_init) - anchor.pos)
              .norm();
      const bool candidate = anchor.id == 1;
      const double raw = geometric + (candidate ? candidate_amplitude : 0.0);
      const size_t factor_index = fixture.graph.size();
      const auto factor = uifgo::MakeUwbFactor(
          X(k), 0, 0, 0, anchor.pos, fixture.cfg.lever_arm_init, raw, 0.05,
          false, false, false, 0.0);
      fixture.graph.add(factor);
      fixture.metadata.push_back(
          {factor_index, obs_id, "uwb_range",
           std::vector<gtsam::Key>(factor->keys().begin(), factor->keys().end())});
      uifgo::ObservationRecord record;
      record.obs_id = obs_id;
      record.sensor_time = static_cast<double>(k);
      record.tag_id = 7;
      record.anchor_id = anchor.id;
      record.raw_range = raw;
      record.valid = true;
      record.planned = true;
      record.keyframe_id = k;
      record.nominal_sigma = 0.05;
      fixture.plan.observations.push_back(record);
      if (candidate) segment.obs_ids.push_back(obs_id);
      ++obs_id;
    }
  }
  segment.observation_count = segment.obs_ids.size();
  fixture.support.segments.push_back(segment);
  return fixture;
}

}  // namespace

TEST(SegmentUwbFactor, ResidualKeysAndWhitenedManifoldJacobian) {
  const gtsam::Point3 anchor(4.2, -1.7, 2.3);
  const gtsam::Point3 lever(0.31, -0.12, 0.18);
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.27, -0.19, 0.41),
                          gtsam::Point3(1.1, 2.2, -0.4));
  const double beta = 0.23;
  const double amplitude = 0.61;
  const double sigma = 0.17;
  const double geometric = (pose.transformFrom(lever) - anchor).norm();
  const double raw = geometric + beta + amplitude - 0.08;
  const auto factor = uifgo::MakeSegmentUwbFactor(
      X(3), C(9), anchor, lever, raw, sigma, beta);
  ASSERT_EQ(factor->keys().size(), 2u);
  EXPECT_EQ(std::set<gtsam::Key>(factor->keys().begin(), factor->keys().end()),
            (std::set<gtsam::Key>{X(3), C(9)}));
  gtsam::Values values;
  values.insert(X(3), pose);
  values.insert<double>(C(9), amplitude);
  EXPECT_NEAR(factor->error(values), 0.5 * std::pow(0.08 / sigma, 2),
              1e-12);
  EXPECT_NEAR(uifgo::SegmentUwbResidual(geometric, raw, beta, amplitude),
              0.08, 1e-14);

  const auto gaussian = factor->linearize(values);
  const auto jacobian =
      boost::dynamic_pointer_cast<gtsam::JacobianFactor>(gaussian);
  ASSERT_TRUE(jacobian);
  const gtsam::Matrix augmented = jacobian->augmentedJacobian();
  ASSERT_EQ(augmented.rows(), 1);
  ASSERT_EQ(augmented.cols(), 8);
  const double eps = 1e-6;
  int column = 0;
  for (gtsam::Key key : factor->keys()) {
    const int dimension = key == X(3) ? 6 : 1;
    for (int coordinate = 0; coordinate < dimension;
         ++coordinate, ++column) {
    gtsam::VectorValues plus_delta = values.zeroVectors();
    gtsam::VectorValues minus_delta = values.zeroVectors();
    if (key == X(3)) {
      plus_delta.at(X(3))[coordinate] = eps;
      minus_delta.at(X(3))[coordinate] = -eps;
    } else {
      plus_delta.at(C(9))[0] = eps;
      minus_delta.at(C(9))[0] = -eps;
    }
    const double plus = std::sqrt(2.0 * factor->error(values.retract(plus_delta)));
    const double minus = std::sqrt(2.0 * factor->error(values.retract(minus_delta)));
    // This fixture keeps the residual positive, so sqrt(2*error) is the
    // signed whitened residual locally.
    const double numerical = (plus - minus) / (2.0 * eps);
    EXPECT_NEAR(augmented(0, column), numerical, 2e-6) << column;
    }
  }
}

TEST(OracleSupportProvider, AcceptsClosedIntervalAndRecordsShortStatus) {
  const auto plan = ParserPlan();
  TempYaml manifest(
      "schema: t04_oracle_support_v1\n"
      "T04_ORACLE_SUPPORT_DEBUG_ONLY: true\n"
      "segments:\n"
      "  - segment_id: s0\n"
      "    link: '7:1'\n"
      "    start_time: 1.0\n"
      "    end_time: 2.0\n");
  const auto support = uifgo::OracleSupportProvider::Load(
      manifest.path.string(), plan, 3, 1.5);
  ASSERT_EQ(support.segments.size(), 1u);
  EXPECT_EQ(support.segments[0].obs_ids,
            (std::vector<std::uint64_t>{100, 102}));
  EXPECT_EQ(support.segments[0].observation_count, 2u);
  EXPECT_DOUBLE_EQ(support.segments[0].duration, 1.0);
  EXPECT_TRUE(support.segments[0].short_support_debug);
}

TEST(OracleSupportProvider, RejectsAmplitudeUnknownAndDuplicateOwnership) {
  const auto plan = ParserPlan();
  TempYaml amplitude(
      "schema: t04_oracle_support_v1\n"
      "T04_ORACLE_SUPPORT_DEBUG_ONLY: true\n"
      "segments:\n"
      "  - {segment_id: s0, link: '7:1', obs_ids: [100], amplitude: 2}\n");
  EXPECT_THROW(uifgo::OracleSupportProvider::Load(amplitude.path.string(), plan,
                                                  1, 0.0),
               std::invalid_argument);
  TempYaml duplicate(
      "schema: t04_oracle_support_v1\n"
      "T04_ORACLE_SUPPORT_DEBUG_ONLY: true\n"
      "segments:\n"
      "  - {segment_id: s0, link: '7:1', obs_ids: [100]}\n"
      "  - {segment_id: s1, link: '7:1', obs_ids: [100]}\n");
  EXPECT_THROW(uifgo::OracleSupportProvider::Load(duplicate.path.string(), plan,
                                                  1, 0.0),
               std::invalid_argument);
}

TEST(OracleSupportProvider, RejectsCrossLinkUnplannedAndEmptyInterval) {
  auto plan = ParserPlan();
  plan.observations[0].planned = false;
  TempYaml cross_link(
      "schema: t04_oracle_support_v1\n"
      "T04_ORACLE_SUPPORT_DEBUG_ONLY: true\n"
      "segments:\n"
      "  - {segment_id: s0, link: '7:1', obs_ids: [101]}\n");
  EXPECT_THROW(uifgo::OracleSupportProvider::Load(cross_link.path.string(), plan,
                                                  1, 0.0),
               std::invalid_argument);
  TempYaml unplanned(
      "schema: t04_oracle_support_v1\n"
      "T04_ORACLE_SUPPORT_DEBUG_ONLY: true\n"
      "segments:\n"
      "  - {segment_id: s0, link: '7:1', obs_ids: [100]}\n");
  EXPECT_THROW(uifgo::OracleSupportProvider::Load(unplanned.path.string(), plan,
                                                  1, 0.0),
               std::invalid_argument);
  TempYaml empty(
      "schema: t04_oracle_support_v1\n"
      "T04_ORACLE_SUPPORT_DEBUG_ONLY: true\n"
      "segments:\n"
      "  - {segment_id: s0, link: '7:1', start_time: 5, end_time: 6}\n");
  EXPECT_THROW(uifgo::OracleSupportProvider::Load(empty.path.string(), plan, 1,
                                                  0.0),
               std::invalid_argument);
}

TEST(SegmentRefitter, KnownNavigationUpdateIsExactAndProjected) {
  auto positive = uifgo::ComputeNonnegativeSegmentAmplitude(
      {5.5, 7.4}, {5.0, 7.0}, {0.5, 1.0});
  ASSERT_TRUE(positive.valid);
  EXPECT_DOUBLE_EQ(positive.denominator, 5.0);
  EXPECT_NEAR(positive.amplitude_m, 0.48, 1e-15);
  auto boundary = uifgo::ComputeNonnegativeSegmentAmplitude(
      {4.5, 6.8}, {5.0, 7.0}, {0.5, 1.0});
  ASSERT_TRUE(boundary.valid);
  EXPECT_DOUBLE_EQ(boundary.amplitude_m, 0.0);
  // At c=0 the derivative sum w*(h-z) is nonnegative, hence
  // max(0,-g)=0 satisfies the boundary KKT rule.
  EXPECT_GE(-boundary.numerator, 0.0);
  EXPECT_FALSE(uifgo::ComputeNonnegativeSegmentAmplitude(
                   {1.0}, {1.0}, {0.0})
                   .valid);
}

TEST(SegmentRefitter, JointSmallGraphConvergesWithOneSharedLiveCAndNoPrior) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.lm_max_iterations = 100;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  options.max_refit_iterations = 20;
  uifgo::SegmentRefitter refitter(options);
  const auto result = refitter.Run(fixture.graph, fixture.values,
                                   fixture.metadata, fixture.plan, fixture.cfg,
                                   fixture.support);
  ASSERT_EQ(result.status, uifgo::SegmentRefitStatus::CONVERGED)
      << result.reason << " gradient="
      << (result.iterations.empty()
              ? -1.0
              : result.iterations.back()
                    .max_scaled_navigation_gradient_objective)
      << " allowance="
      << (result.iterations.empty()
              ? -1.0
              : result.iterations.back()
                    .navigation_gradient_roundoff_allowance_objective)
      << " objective="
      << (result.iterations.empty() ? -1.0
                                    : result.iterations.back().objective_after);
  ASSERT_TRUE(result.values.exists(C(0)));
  EXPECT_NEAR(result.values.at<double>(C(0)), 0.4, 2e-3);
  ASSERT_EQ(result.segments.size(), 1u);
  EXPECT_FALSE(result.segments[0].boundary);
  EXPECT_LE(result.segments[0].kkt_violation,
            options.projected_gradient_tolerance);
  ASSERT_FALSE(result.iterations.empty());
  EXPECT_TRUE(result.iterations.back().navigation_stationarity_ok);
  EXPECT_LE(
      result.iterations.back().max_scaled_navigation_gradient_objective,
      options.navigation_stationarity_tolerance_objective +
          result.iterations.back()
              .navigation_gradient_roundoff_allowance_objective);
  size_t segment_factor_count = 0;
  std::map<std::uint64_t, size_t> factors_per_obs;
  for (const auto& meta : result.factor_metadata) {
    if (meta.obs_id != 0) ++factors_per_obs[meta.obs_id];
    if (meta.factor_type == "uwb_segment_range") {
      ++segment_factor_count;
      ASSERT_EQ(meta.keys.size(), 2u);
      EXPECT_EQ(std::set<gtsam::Key>(meta.keys.begin(), meta.keys.end()),
                (std::set<gtsam::Key>{X(segment_factor_count - 1), C(0)}));
    }
    EXPECT_FALSE(meta.factor_type.find("prior") != std::string::npos &&
                 meta.keys.size() == 1 && meta.keys[0] == C(0));
  }
  EXPECT_EQ(segment_factor_count, 2u);
  for (const auto& record : fixture.plan.observations)
    EXPECT_EQ(factors_per_obs[record.obs_id], 1u);
}

TEST(PaperRunIo, ProductionStage2BundleRoundTripsAllLiveTypesAndFailsClosed) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.lm_max_iterations = 100;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  options.max_refit_iterations = 20;
  const auto refit = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, fixture.support);
  ASSERT_TRUE(refit.converged()) << refit.reason;

  auto graph = refit.graph;
  auto values = refit.values;
  auto metadata = refit.factor_metadata;
  values.insert(C(1), 0.0);
  graph.add(gtsam::PriorFactor<double>(
      C(1), 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 0.25)));
  uifgo::RefitFactorMeta second;
  second.factor_index = graph.size() - 1;
  second.factor_type = "engineering_second_segment_boundary";
  second.segment_id = "engineering-segment-1";
  second.keys = {C(1)};
  metadata.push_back(second);

  uifgo::InferenceIdentityContext context;
  context.input_sha256 = "sha256:r06-fixture-input";
  context.config_sha256 = "sha256:r06-fixture-config";
  context.input_plan_sha256 = "sha256:r06-fixture-plan";
  context.support_partition_sha256 = "sha256:r06-two-segments";
  context.calibration_sha256 = "sha256:r06-fixture-calibration";
  context.solver_config_sha256 = "sha256:r06-frozen-solver";

  const char* preserved_root = std::getenv("UIFGO_R06_EXPORT_FIXTURE_ROOT");
  const bool preserve = preserved_root && std::string(preserved_root).size();
  const auto root = preserve
      ? boost::filesystem::path(preserved_root)
      : boost::filesystem::temp_directory_path() /
            boost::filesystem::unique_path("uifgo-r06-%%%%-%%%%");
  ASSERT_TRUE(boost::filesystem::create_directories(root));
  const auto exported = uifgo::WriteDevelopmentStage2Bundle(
      root.string(), graph, values, metadata, context);
  ASSERT_TRUE(exported.ok) << exported.reason;
  EXPECT_TRUE(boost::filesystem::is_regular_file(
      root / "stage2_export_manifest.json"));
  const auto restored = uifgo::ReadDevelopmentStage2Values(
      (root / "final_values.csv").string());
  std::string bit_reason;
  EXPECT_TRUE(uifgo::DevelopmentStage2ValuesBitEqual(
      values, restored, &bit_reason)) << bit_reason;
  EXPECT_DOUBLE_EQ(values.at<double>(C(0)), restored.at<double>(C(0)));
  EXPECT_DOUBLE_EQ(0.0, restored.at<double>(C(1)));
  const auto before_identity = uifgo::ComputeInferenceContentIdentity(
      graph, values, context);
  const auto after_identity = uifgo::ComputeInferenceContentIdentity(
      graph, restored, context);
  EXPECT_EQ(before_identity.graph_linearization_sha256,
            after_identity.graph_linearization_sha256);
  EXPECT_EQ(before_identity.values_sha256, after_identity.values_sha256);
  EXPECT_DOUBLE_EQ(graph.error(values), graph.error(restored));
  const auto before_gradient = graph.linearize(values)->gradientAtZero();
  const auto after_gradient = graph.linearize(restored)->gradientAtZero();
  ASSERT_EQ(before_gradient.size(), after_gradient.size());
  for (const auto& item : before_gradient) {
    ASSERT_TRUE(after_gradient.exists(item.first));
    EXPECT_TRUE(item.second == after_gradient.at(item.first));
  }
  const auto before_nav = uifgo::AuditNavigationStationarity(
      graph, values, {}, 1.0e6, 8.0);
  const auto after_nav = uifgo::AuditNavigationStationarity(
      graph, restored, {}, 1.0e6, 8.0);
  EXPECT_EQ(before_nav.valid, after_nav.valid);
  EXPECT_DOUBLE_EQ(before_nav.max_scaled_gradient_objective,
                   after_nav.max_scaled_gradient_objective);
  for (gtsam::Key key : {C(0), C(1)}) {
    const double g_before = before_gradient.at(key)[0];
    const double g_after = after_gradient.at(key)[0];
    const double kkt_before = values.at<double>(key) > 0.0
                                  ? std::abs(g_before)
                                  : std::max(0.0, -g_before);
    const double kkt_after = restored.at<double>(key) > 0.0
                                 ? std::abs(g_after)
                                 : std::max(0.0, -g_after);
    EXPECT_DOUBLE_EQ(kkt_before, kkt_after);
  }

  const auto bad_root = [&](const std::string& name) {
    const auto path = root / name;
    EXPECT_TRUE(boost::filesystem::create_directory(path));
    return path;
  };
  auto missing = values;
  missing.erase(C(1));
  const auto missing_dir = bad_root("missing");
  EXPECT_FALSE(uifgo::WriteDevelopmentStage2Bundle(
      missing_dir.string(), graph, missing, metadata, context).ok);
  EXPECT_FALSE(boost::filesystem::exists(
      missing_dir / "stage2_export_manifest.json"));
  auto wrong_type = values;
  wrong_type.erase(C(0));
  wrong_type.insert(C(0), gtsam::Vector1(0.4));
  const auto wrong_dir = bad_root("wrong_type");
  EXPECT_FALSE(uifgo::WriteDevelopmentStage2Bundle(
      wrong_dir.string(), graph, wrong_type, metadata, context).ok);
  auto unsupported = values;
  unsupported.insert(Z(99), 1.0);
  const auto unsupported_dir = bad_root("unsupported");
  EXPECT_FALSE(uifgo::WriteDevelopmentStage2Bundle(
      unsupported_dir.string(), graph, unsupported, metadata, context).ok);
  auto nonfinite = values;
  nonfinite.update(C(0), std::numeric_limits<double>::infinity());
  const auto nonfinite_dir = bad_root("nonfinite");
  EXPECT_FALSE(uifgo::WriteDevelopmentStage2Bundle(
      nonfinite_dir.string(), graph, nonfinite, metadata, context).ok);
  EXPECT_FALSE(uifgo::WriteDevelopmentStage2Bundle(
      (root / "does_not_exist").string(), graph, values, metadata, context).ok);
  EXPECT_FALSE(uifgo::WriteDevelopmentStage2Bundle(
      root.string(), graph, values, metadata, context).ok);
  if (!preserve) boost::filesystem::remove_all(root);
}

TEST(SegmentRefitter,
     A19DevelopmentStage2LocalCallbackCapturesAllRangesAndRejectsWrongIdentity) {
  auto fixture = MakeRefitFixture(0.4);
  fixture.cfg.fixed_beta_by_link["7:1"] = 0.125;
  // Keep the same physical observations after introducing the explicit beta.
  for (auto& record : fixture.plan.observations)
    if (record.anchor_id == 1) record.raw_range += 0.125;
  auto support = uifgo::ToSupportPartition(fixture.support);
  support.solver_config_hash = "a19-policy-sha256:engineering-fixture";
  uifgo::RefitOptions options;
  options.max_refit_iterations = 3;
  options.lm_relative_tolerance = 1e-12;
  options.lm_absolute_tolerance = 1e-12;
  uifgo::DevelopmentStage2Request request;
  request.policy = "PAPER_CERTIFIED_PAIR_REDUCTION_V1";
  request.role = "development";
  request.implementation_identity = support.solver_config_hash;
  bool saw_nonzero = false;
  request.conditional_navigation = [&](
      size_t outer, const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& values, const uifgo::CheckedLmOptions& lm_options,
      const std::vector<uifgo::DevelopmentRefitRangeConstant>& ranges)
      -> uifgo::CheckedLmResult {
    EXPECT_EQ(lm_options.policy, uifgo::ConditionalLmPolicy::
        GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2);
    EXPECT_EQ(ranges.size(), 8u);
    size_t candidate_count = 0;
    size_t reference_count = 0;
    for (const auto& item : ranges) {
      const size_t observation_index = item.obs_id - 1000;
      EXPECT_EQ(item.pose_key, X(observation_index / 4));
      EXPECT_DOUBLE_EQ(item.measurement,
                       fixture.plan.observations[observation_index].raw_range);
      EXPECT_DOUBLE_EQ(item.sigma, 0.05);
      EXPECT_EQ(item.factor_index < graph.size(), true);
      EXPECT_DOUBLE_EQ(item.fixed_beta,
                       item.candidate ? 0.125 : 0.0);
      EXPECT_DOUBLE_EQ(item.conditional_beta,
                       item.fixed_beta + item.segment_amplitude);
      const auto actual = boost::dynamic_pointer_cast<
          gtsam::NoiseModelFactor>(graph.at(item.factor_index));
      EXPECT_TRUE(actual);
      if (actual) {
        const auto actual_error = actual->unwhitenedError(values);
        EXPECT_EQ(actual_error.size(), 1);
        if (actual_error.size() == 1)
          EXPECT_DOUBLE_EQ(actual_error[0],
                           item.expected_unwhitened_residual);
      }
      candidate_count += item.candidate;
      reference_count += !item.candidate;
    }
    EXPECT_EQ(candidate_count, 2u);
    EXPECT_EQ(reference_count, 6u);
    if (outer > 1) {
      for (const auto& item : ranges) {
        if (item.candidate) EXPECT_GT(item.conditional_beta, 0.125);
      }
      saw_nonzero = true;
      uifgo::CheckedLmResult stopped;
      stopped.values = values;
      stopped.reason = "A19_ENGINEERING_CAPTURE_STOP";
      return stopped;
    }
    uifgo::CheckedLmResult pass_through;
    pass_through.converged = true;
    pass_through.values = values;
    pass_through.reason = "A19_ENGINEERING_PASS_THROUGH";
    return pass_through;
  };
  const auto captured = uifgo::SegmentRefitter(options).RunDevelopmentStage2(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, support, &request);
  EXPECT_TRUE(saw_nonzero);
  EXPECT_EQ(captured.status, uifgo::SegmentRefitStatus::CONDITIONAL_LM_FAILED);
  EXPECT_EQ(captured.reason, "A19_ENGINEERING_CAPTURE_STOP");

  request.role = "validation";
  const auto wrong_role = uifgo::SegmentRefitter(options).RunDevelopmentStage2(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, support, &request);
  EXPECT_EQ(wrong_role.status, uifgo::SegmentRefitStatus::INVALID_INPUT);
  request.role = "development";
  request.implementation_identity = "a18-policy-sha256:wrong";
  const auto wrong_identity =
      uifgo::SegmentRefitter(options).RunDevelopmentStage2(
          fixture.graph, fixture.values, fixture.metadata, fixture.plan,
          fixture.cfg, support, &request);
  EXPECT_EQ(wrong_identity.status, uifgo::SegmentRefitStatus::INVALID_INPUT);
}

TEST(SegmentRefitter,
     A19R03QualifiedInexactHandoffRunsCUpdateAndStillRequiresJointAnd) {
  auto fixture = MakeRefitFixture(0.4);
  auto support = uifgo::ToSupportPartition(fixture.support);
  support.solver_config_hash = "a19-policy-sha256:r03-mixed-fixture";
  uifgo::RefitOptions options;
  options.max_refit_iterations = 20;
  options.lm_max_iterations = 50;
  options.lm_relative_tolerance = 1e-6;
  options.lm_absolute_tolerance = 1e-8;
  uifgo::DevelopmentStage2Request request;
  request.policy =
      "PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1";
  request.role = "development";
  request.implementation_identity = support.solver_config_hash;
  request.allow_inexact_handoff = true;
  size_t callback_count = 0;
  bool saw_outer_after_handoff = false;
  request.conditional_navigation = [&](
      size_t outer, const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& values, const uifgo::CheckedLmOptions& lm_options,
      const std::vector<uifgo::DevelopmentRefitRangeConstant>& ranges) {
    ++callback_count;
    EXPECT_FALSE(ranges.empty());
    EXPECT_DOUBLE_EQ(lm_options.inexact_handoff_scaled_step_tolerance,
                     options.scaled_step_tolerance);
    auto fixture_options = lm_options;
    fixture_options.policy = uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1;
    auto inner =
        uifgo::RunCheckedConditionalLm(graph, values, fixture_options);
    EXPECT_TRUE(inner.converged) << inner.reason;
    if (outer == 1) {
      inner.converged = false;
      inner.reason = "INNER_NUMERICAL_STALL_INEXACT";
      inner.inexact_handoff.enabled = true;
      inner.inexact_handoff.qualified = true;
      inner.inexact_handoff.status = "INNER_NUMERICAL_STALL_INEXACT";
      inner.inexact_handoff.handoff_count = 1;
      inner.inexact_handoff.last_accepted_values_identity =
          "r03-fixture-last-accepted";
    } else {
      saw_outer_after_handoff = true;
    }
    return inner;
  };
  const auto result = uifgo::SegmentRefitter(options).RunDevelopmentStage2(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, support, &request);
  ASSERT_TRUE(result.converged()) << result.reason;
  ASSERT_EQ(result.inexact_handoffs.size(), 1u);
  EXPECT_FALSE(result.inexact_handoffs.front().qualified == false);
  EXPECT_TRUE(saw_outer_after_handoff);
  EXPECT_GT(callback_count, 1u);
  ASSERT_FALSE(result.iterations.empty());
  EXPECT_TRUE(result.iterations.front().conditional_inexact_handoff);
  const auto& last = result.iterations.back();
  EXPECT_TRUE(last.objective_ok);
  EXPECT_TRUE(last.step_ok);
  EXPECT_TRUE(last.kkt_ok);
  EXPECT_TRUE(last.navigation_stationarity_ok);
  const auto scores = uifgo::ScoreRefitRecoverability(
      result, support, fixture.plan, fixture.cfg);
  ASSERT_EQ(scores.size(), 1u);
  EXPECT_TRUE(scores.front().valid_score_exported) << scores.front().status;

  auto wrong = request;
  wrong.allow_inexact_handoff = false;
  const auto rejected = uifgo::SegmentRefitter(options).RunDevelopmentStage2(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, support, &wrong);
  EXPECT_EQ(rejected.status, uifgo::SegmentRefitStatus::INVALID_INPUT);
}

TEST(SegmentRefitter,
     A19DevelopmentRejectsRangeContractMismatchesBeforeCallback) {
  using Mutation = std::function<void(RefitFixture*)>;
  const auto run_case = [](const std::string& name, const Mutation& mutate) {
    SCOPED_TRACE(name);
    auto fixture = MakeRefitFixture(0.4);
    mutate(&fixture);
    auto support = uifgo::ToSupportPartition(fixture.support);
    support.solver_config_hash = "a19-r01-policy-sha256:negative-fixture";
    uifgo::DevelopmentStage2Request request;
    request.policy = "PAPER_CERTIFIED_PAIR_REDUCTION_V1";
    request.role = "development";
    request.implementation_identity = support.solver_config_hash;
    size_t callback_count = 0;
    request.conditional_navigation = [&callback_count](
        size_t, const gtsam::NonlinearFactorGraph&, const gtsam::Values& values,
        const uifgo::CheckedLmOptions&,
        const std::vector<uifgo::DevelopmentRefitRangeConstant>&) {
      ++callback_count;
      uifgo::CheckedLmResult result;
      result.converged = true;
      result.values = values;
      return result;
    };
    uifgo::RefitOptions options;
    options.max_refit_iterations = 2;
    const auto result = uifgo::SegmentRefitter(options).RunDevelopmentStage2(
        fixture.graph, fixture.values, fixture.metadata, fixture.plan,
        fixture.cfg, support, &request);
    EXPECT_EQ(result.status, uifgo::SegmentRefitStatus::INVALID_INPUT)
        << result.reason;
    EXPECT_EQ(callback_count, 0u);
  };

  run_case("raw measurement", [](RefitFixture* fixture) {
    fixture->plan.observations[1].raw_range += 0.25;
  });
  run_case("sigma", [](RefitFixture* fixture) {
    fixture->plan.observations[1].nominal_sigma = 0.075;
  });
  run_case("anchor", [](RefitFixture* fixture) {
    fixture->cfg.anchors[1].pos.x() += 0.3;
  });
  run_case("lever arm", [](RefitFixture* fixture) {
    fixture->cfg.lever_arm_init.y() -= 0.2;
  });
  run_case("fixed beta", [](RefitFixture* fixture) {
    fixture->cfg.fixed_beta_by_link["7:2"] = 0.15;
  });
  run_case("factor metadata pose key", [](RefitFixture* fixture) {
    fixture->metadata[1].keys = {X(1)};
  });
  run_case("unknown observation metadata", [](RefitFixture* fixture) {
    fixture->metadata[1].obs_id = 999999;
  });
  run_case("omitted factor metadata", [](RefitFixture* fixture) {
    fixture->metadata.pop_back();
  });
  run_case("duplicate factor metadata", [](RefitFixture* fixture) {
    fixture->metadata[1].factor_index = fixture->metadata[0].factor_index;
  });
}

TEST(SegmentRefitter, ZeroAmplitudeStopsAtBoundaryWithValidKkt) {
  auto fixture = MakeRefitFixture(0.0);
  uifgo::RefitOptions options;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  options.max_refit_iterations = 20;
  uifgo::SegmentRefitter refitter(options);
  const auto result = refitter.Run(fixture.graph, fixture.values,
                                   fixture.metadata, fixture.plan, fixture.cfg,
                                   fixture.support);
  ASSERT_EQ(result.status, uifgo::SegmentRefitStatus::CONVERGED)
      << result.reason << " gradient="
      << (result.iterations.empty()
              ? -1.0
              : result.iterations.back()
                    .max_scaled_navigation_gradient_objective)
      << " allowance="
      << (result.iterations.empty()
              ? -1.0
              : result.iterations.back()
                    .navigation_gradient_roundoff_allowance_objective)
      << " objective="
      << (result.iterations.empty() ? -1.0
                                    : result.iterations.back().objective_after);
  ASSERT_EQ(result.segments.size(), 1u);
  EXPECT_DOUBLE_EQ(result.segments[0].amplitude_m, 0.0);
  EXPECT_TRUE(result.segments[0].boundary);
  EXPECT_DOUBLE_EQ(result.segments[0].kkt_violation, 0.0);
}

TEST(SegmentRefitter, OneOuterIterationFailsAndGraphMismatchIsExplicit) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.max_refit_iterations = 1;
  options.relative_objective_tolerance = 0.0;
  options.scaled_step_tolerance = 0.0;
  options.projected_gradient_tolerance = 0.0;
  uifgo::SegmentRefitter refitter(options);
  const auto limited = refitter.Run(fixture.graph, fixture.values,
                                    fixture.metadata, fixture.plan, fixture.cfg,
                                    fixture.support);
  EXPECT_EQ(limited.status,
            uifgo::SegmentRefitStatus::MAX_REFIT_ITERATIONS);

  fixture.values.erase(B(1));
  const auto mismatch = refitter.Run(fixture.graph, fixture.values,
                                     fixture.metadata, fixture.plan,
                                     fixture.cfg, fixture.support);
  EXPECT_EQ(mismatch.status,
            uifgo::SegmentRefitStatus::GRAPH_VALUES_KEY_MISMATCH);
}

TEST(SegmentRefitter, EmptyPartitionStillRunsSharedRawStage2Refit) {
  auto fixture = MakeRefitFixture(0.0);
  uifgo::SupportPartition empty;
  empty.schema = "t06_automatic_support_v1";
  empty.provider = "automatic_discovery";
  uifgo::RefitOptions options;
  options.lm_relative_tolerance = 1e-12;
  options.lm_absolute_tolerance = 1e-12;
  const auto result = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, empty);
  ASSERT_TRUE(result.converged()) << result.reason;
  EXPECT_EQ(result.reason,
            "NO_CANDIDATES_RAW_STAGE2_ALL_APPLICABLE_STOP_CONDITIONS_SATISFIED");
  ASSERT_FALSE(result.iterations.empty());
  EXPECT_TRUE(result.iterations.back().objective_ok);
  EXPECT_TRUE(result.iterations.back().step_ok);
  EXPECT_TRUE(result.iterations.back().kkt_ok);
  EXPECT_TRUE(result.iterations.back().navigation_stationarity_ok);
  EXPECT_LE(result.iterations.back().max_scaled_navigation_gradient_objective,
            options.navigation_stationarity_tolerance_objective +
                result.iterations.back()
                    .navigation_gradient_roundoff_allowance_objective);
  EXPECT_TRUE(result.segments.empty());
  EXPECT_EQ(result.graph.size(), fixture.graph.size());
  EXPECT_EQ(result.values.size(), fixture.values.size());
  for (gtsam::Key key : result.values.keys())
    EXPECT_NE(gtsam::Symbol(key).chr(), 'c');
}

TEST(SegmentRefitter, EmptyPartitionDoesNotMaskRawStage2Failure) {
  auto fixture = MakeRefitFixture(0.0);
  uifgo::SupportPartition empty;
  empty.schema = "t06_automatic_support_v1";
  empty.provider = "automatic_discovery";
  uifgo::RefitOptions options;
  options.lm_max_iterations = 1;
  const auto result = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, empty);
  EXPECT_EQ(result.status,
            uifgo::SegmentRefitStatus::CONDITIONAL_LM_FAILED);
  EXPECT_FALSE(result.converged());
}

TEST(SegmentRefitter, EmptyPartitionCannotConvergeWithoutFinalStationarity) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::SupportPartition empty;
  empty.schema = "t06_automatic_support_v2";
  empty.provider = "automatic_discovery";
  uifgo::RefitOptions options;
  options.max_refit_iterations = 1;
  options.lm_relative_tolerance = 1.0;
  options.navigation_stationarity_tolerance_objective = 1e-6;
  const auto result = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, empty);
  EXPECT_EQ(result.status,
            uifgo::SegmentRefitStatus::MAX_REFIT_ITERATIONS);
  ASSERT_EQ(result.iterations.size(), 1u);
  EXPECT_FALSE(result.iterations.back().navigation_stationarity_ok);
  EXPECT_FALSE(result.converged());
}

TEST(SegmentRefitter, ConditionalLmFailureIsExplicit) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.lm_max_iterations = 1;
  uifgo::SegmentRefitter refitter(options);
  const auto result = refitter.Run(fixture.graph, fixture.values,
                                   fixture.metadata, fixture.plan, fixture.cfg,
                                   fixture.support);
  EXPECT_EQ(result.status,
            uifgo::SegmentRefitStatus::CONDITIONAL_LM_FAILED);
}

TEST(SegmentRefitter,
     AmplitudeKktAndLmStagnationDoNotReplaceJointNavigationStationarity) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.max_refit_iterations = 1;
  // Isolate the new condition: the prior three stop checks are deliberately
  // nonbinding, while final navigation stationarity remains strict.
  options.relative_objective_tolerance = 1.0;
  options.scaled_step_tolerance = 1.0e6;
  options.projected_gradient_tolerance = 1.0e-8;
  options.navigation_stationarity_tolerance_objective = 1.0e-6;
  uifgo::SegmentRefitter refitter(options);
  const auto result = refitter.Run(fixture.graph, fixture.values,
                                   fixture.metadata, fixture.plan, fixture.cfg,
                                   fixture.support);
  ASSERT_EQ(result.status,
            uifgo::SegmentRefitStatus::MAX_REFIT_ITERATIONS);
  ASSERT_EQ(result.iterations.size(), 1u);
  const auto& trace = result.iterations.front();
  EXPECT_GT(trace.conditional_lm_iterations, 0u);
  EXPECT_TRUE(trace.objective_ok);
  EXPECT_TRUE(trace.step_ok);
  EXPECT_TRUE(trace.kkt_ok);
  EXPECT_FALSE(trace.navigation_stationarity_ok);
  EXPECT_GT(trace.max_scaled_navigation_gradient_objective,
            trace.navigation_gradient_roundoff_allowance_objective);
}

TEST(NlosScoring, ClosedIntervalsUseTransitiveOverlapWithoutEpsilon) {
  std::vector<uifgo::SegmentEstimate> segments(4);
  segments[0].segment_id = "a";
  segments[0].segment_ordinal = 0;
  segments[0].start_time = 0.0;
  segments[0].end_time = 1.0;
  segments[1].segment_id = "b";
  segments[1].segment_ordinal = 1;
  segments[1].start_time = 1.0;  // closed-endpoint overlap with a
  segments[1].end_time = 2.0;
  segments[2].segment_id = "c";
  segments[2].segment_ordinal = 2;
  segments[2].start_time = 1.5;  // transitive overlap through b
  segments[2].end_time = 3.0;
  segments[3].segment_id = "d";
  segments[3].segment_ordinal = 3;
  segments[3].start_time = std::nextafter(3.0, 4.0);  // no epsilon merge
  segments[3].end_time = 4.0;
  const auto groups = uifgo::BuildClosedIntervalOverlapGroups(segments);
  ASSERT_EQ(groups.size(), 2u);
  EXPECT_EQ(groups[0].segment_ordinals,
            (std::vector<size_t>{0, 1, 2}));
  EXPECT_EQ(groups[1].segment_ordinals, (std::vector<size_t>{3}));
}

TEST(NlosScoring, RealGtsamWhitenedGraphExcludesRhsAndScoresCandidateOnce) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  const auto refit = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, fixture.support);
  ASSERT_TRUE(refit.converged()) << refit.reason;
  const auto scores = uifgo::ScoreRefitRecoverability(
      refit, fixture.support, fixture.plan, fixture.cfg);
  ASSERT_EQ(scores.size(), 1u);
  const auto& score = scores.front();
  ASSERT_TRUE(score.numerical.valid_score()) << score.numerical.reason;
  EXPECT_EQ(score.G_whitened.cols(), 1);
  EXPECT_EQ(score.rhs_whitened.size(), score.F_whitened.rows());
  size_t candidate_rows = 0;
  for (const auto& factor : score.factor_rows)
    if (factor.factor_type == "uwb_segment_range") candidate_rows += factor.row_count;
  EXPECT_EQ(candidate_rows, 2u);
  size_t nonzero_g = 0;
  for (Eigen::Index row = 0; row < score.G_whitened.rows(); ++row) {
    if (score.G_whitened(row, 0) != 0.0) {
      ++nonzero_g;
      EXPECT_NEAR(std::abs(score.G_whitened(row, 0)), 1.0 / 0.05, 1e-12);
    }
  }
  EXPECT_EQ(nonzero_g, 2u);
  EXPECT_EQ(score.key_columns.back().role, "GROUP_AMPLITUDE");
  EXPECT_EQ(score.key_columns.back().offset,
            static_cast<size_t>(score.F_whitened.cols()));
  ASSERT_EQ(score.segment_fit.size(), 1u);
  EXPECT_LT(score.segment_fit.front().gamma, 1e-8);
}

TEST(NlosScoring, RejectsInvalidGraphMetadataSupportAndPlanBeforeScoring) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  const auto refit = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, fixture.support);
  ASSERT_TRUE(refit.converged()) << refit.reason;
  auto score = [&](const uifgo::SegmentRefitResult& candidate,
                   const uifgo::OracleSupport& support,
                   const uifgo::PaperInputPlan& plan) {
    return uifgo::ScoreRefitRecoverability(
        candidate, support, plan, fixture.cfg);
  };

  auto wrong_index = refit;
  wrong_index.factor_metadata[0].factor_index = refit.graph.size();
  EXPECT_THROW(score(wrong_index, fixture.support, fixture.plan),
               std::invalid_argument);

  auto wrong_keys = refit;
  wrong_keys.factor_metadata[0].keys.clear();
  EXPECT_THROW(score(wrong_keys, fixture.support, fixture.plan),
               std::invalid_argument);

  auto unknown_type = refit;
  unknown_type.factor_metadata[0].factor_type = "unknown_type";
  EXPECT_THROW(score(unknown_type, fixture.support, fixture.plan),
               std::invalid_argument);

  auto duplicate_obs = refit;
  std::vector<size_t> uwb_indices;
  for (size_t i = 0; i < duplicate_obs.factor_metadata.size(); ++i)
    if (duplicate_obs.factor_metadata[i].obs_id != 0) uwb_indices.push_back(i);
  ASSERT_GE(uwb_indices.size(), 2u);
  duplicate_obs.factor_metadata[uwb_indices[1]].obs_id =
      duplicate_obs.factor_metadata[uwb_indices[0]].obs_id;
  EXPECT_THROW(score(duplicate_obs, fixture.support, fixture.plan),
               std::invalid_argument);

  auto incomplete_candidate = refit;
  auto candidate_meta = std::find_if(
      incomplete_candidate.factor_metadata.begin(),
      incomplete_candidate.factor_metadata.end(),
      [](const uifgo::RefitFactorMeta& meta) {
        return meta.factor_type == "uwb_segment_range";
      });
  ASSERT_NE(candidate_meta, incomplete_candidate.factor_metadata.end());
  candidate_meta->factor_type = "uwb_range";
  candidate_meta->segment_id.clear();
  EXPECT_THROW(score(incomplete_candidate, fixture.support, fixture.plan),
               std::invalid_argument);

  auto duplicate_plan = fixture.plan;
  duplicate_plan.observations.push_back(duplicate_plan.observations.front());
  EXPECT_THROW(score(refit, fixture.support, duplicate_plan),
               std::invalid_argument);

  auto duplicate_support = fixture.support;
  duplicate_support.segments.push_back(duplicate_support.segments.front());
  EXPECT_THROW(score(refit, duplicate_support, fixture.plan),
               std::invalid_argument);
}

TEST(NlosScoring, LinearizationIdHashesCanonicalPhysicalLinearization) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  const auto refit = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, fixture.support);
  ASSERT_TRUE(refit.converged()) << refit.reason;
  const auto first = uifgo::ScoreRefitRecoverability(
      refit, fixture.support, fixture.plan, fixture.cfg);
  const auto repeated = uifgo::ScoreRefitRecoverability(
      refit, fixture.support, fixture.plan, fixture.cfg);
  ASSERT_EQ(first.size(), 1u);
  ASSERT_EQ(repeated.size(), 1u);
  EXPECT_EQ(first[0].linearization_id, repeated[0].linearization_id);
  EXPECT_EQ(first[0].linearization_id.rfind("t05lin-sha256:", 0), 0u);
  EXPECT_EQ(first[0].linearization_id.size(),
            std::string("t05lin-sha256:").size() + 64u);

  auto narrow = refit;
  auto wide = refit;
  const auto velocity = refit.values.at<gtsam::Vector3>(V(0));
  narrow.graph.addPrior(V(0), velocity,
                        gtsam::noiseModel::Isotropic::Sigma(3, 0.2));
  wide.graph.addPrior(V(0), velocity,
                      gtsam::noiseModel::Isotropic::Sigma(3, 0.5));
  narrow.factor_metadata.push_back(
      {narrow.graph.size() - 1, 0, "preserved_non_uwb", {V(0)}, ""});
  wide.factor_metadata.push_back(
      {wide.graph.size() - 1, 0, "preserved_non_uwb", {V(0)}, ""});
  const auto narrow_score = uifgo::ScoreRefitRecoverability(
      narrow, fixture.support, fixture.plan, fixture.cfg);
  const auto wide_score = uifgo::ScoreRefitRecoverability(
      wide, fixture.support, fixture.plan, fixture.cfg);
  ASSERT_EQ(narrow_score.size(), 1u);
  ASSERT_EQ(wide_score.size(), 1u);
  EXPECT_FALSE(narrow_score[0].F_whitened.isApprox(
      wide_score[0].F_whitened, 0.0));
  EXPECT_NE(narrow_score[0].linearization_id,
            wide_score[0].linearization_id);
}

TEST(NlosScoring, MeasurementRhsChangeDoesNotEnterInformationColumns) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  const auto refit = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, fixture.support);
  ASSERT_TRUE(refit.converged()) << refit.reason;
  const auto baseline = uifgo::ScoreRefitRecoverability(
      refit, fixture.support, fixture.plan, fixture.cfg);
  ASSERT_EQ(baseline.size(), 1u);

  uifgo::SegmentRefitResult changed = refit;
  bool replaced = false;
  for (size_t index = 0; index < changed.factor_metadata.size(); ++index) {
    const auto& meta = changed.factor_metadata[index];
    if (meta.factor_type != "uwb_segment_range") continue;
    const auto record = std::find_if(
        fixture.plan.observations.begin(), fixture.plan.observations.end(),
        [&](const uifgo::ObservationRecord& item) {
          return item.obs_id == meta.obs_id;
        });
    ASSERT_NE(record, fixture.plan.observations.end());
    const auto anchor = std::find_if(
        fixture.cfg.anchors.begin(), fixture.cfg.anchors.end(),
        [&](const uifgo::AnchorConfig& item) {
          return item.id == record->anchor_id;
        });
    ASSERT_NE(anchor, fixture.cfg.anchors.end());
    changed.graph.replace(index, uifgo::MakeSegmentUwbFactor(
        X(record->keyframe_id), C(0), anchor->pos,
        fixture.cfg.lever_arm_init, record->raw_range + 3.0,
        record->nominal_sigma, 0.0));
    replaced = true;
    break;
  }
  ASSERT_TRUE(replaced);
  const auto perturbed = uifgo::ScoreRefitRecoverability(
      changed, fixture.support, fixture.plan, fixture.cfg);
  ASSERT_EQ(perturbed.size(), 1u);
  EXPECT_TRUE(baseline[0].F_whitened.isApprox(
      perturbed[0].F_whitened, 0.0));
  EXPECT_TRUE(baseline[0].G_whitened.isApprox(
      perturbed[0].G_whitened, 0.0));
  EXPECT_FALSE(baseline[0].rhs_whitened.isApprox(
      perturbed[0].rhs_whitened, 0.0));
  EXPECT_NE(baseline[0].linearization_id, perturbed[0].linearization_id);
  EXPECT_TRUE(baseline[0].numerical.N.isApprox(
      perturbed[0].numerical.N, 0.0));
  EXPECT_TRUE(baseline[0].numerical.R.isApprox(
      perturbed[0].numerical.R, 0.0));
}

TEST(NlosScoring, ShortAndBoundaryRemainStage2ResultsButDoNotExportScore) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  const auto converged = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, fixture.support);
  ASSERT_TRUE(converged.converged()) << converged.reason;

  for (int mode = 0; mode < 2; ++mode) {
    uifgo::SegmentRefitResult marked = converged;
    marked.segments[0].short_support_debug = mode == 0;
    marked.segments[0].boundary = mode == 1;
    const auto scores = uifgo::ScoreRefitRecoverability(
        marked, fixture.support, fixture.plan, fixture.cfg);
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_TRUE(scores[0].numerical.valid_score());
    EXPECT_FALSE(scores[0].eligible);
    EXPECT_FALSE(scores[0].valid_score_exported);
    EXPECT_EQ(scores[0].status, "INELIGIBLE_SHORT_OR_BOUNDARY_DEBUG");
    ASSERT_EQ(scores[0].segment_fit.size(), 1u);
    EXPECT_TRUE(std::isfinite(scores[0].segment_fit[0].gamma));
    EXPECT_TRUE(marked.converged());
  }
}

TEST(NlosScoring, PhysicalInformationIsIndependentOfLmDampingSettings) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::RefitOptions options;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  const auto refit = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, fixture.support);
  ASSERT_TRUE(refit.converged()) << refit.reason;

  gtsam::Values trial_initial = refit.values;
  trial_initial.update(C(0), refit.values.at<double>(C(0)) + 0.1);
  gtsam::LevenbergMarquardtParams low;
  low.setlambdaInitial(1e-9);
  low.setlambdaFactor(2.0);
  low.setDiagonalDamping(false);
  gtsam::LevenbergMarquardtParams high;
  high.setlambdaInitial(1e3);
  high.setlambdaFactor(20.0);
  high.setDiagonalDamping(true);
  gtsam::LevenbergMarquardtOptimizer low_optimizer(refit.graph, trial_initial,
                                                   low);
  gtsam::LevenbergMarquardtOptimizer high_optimizer(refit.graph, trial_initial,
                                                    high);
  low_optimizer.iterate();
  high_optimizer.iterate();
  EXPECT_GT(low_optimizer.values()
                .localCoordinates(high_optimizer.values())
                .vector()
                .norm(),
            1e-8);

  const auto first = uifgo::ScoreRefitRecoverability(
      refit, fixture.support, fixture.plan, fixture.cfg);
  const auto second = uifgo::ScoreRefitRecoverability(
      refit, fixture.support, fixture.plan, fixture.cfg);
  ASSERT_EQ(first.size(), 1u);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_TRUE(first[0].F_whitened.isApprox(second[0].F_whitened, 0.0));
  EXPECT_TRUE(first[0].G_whitened.isApprox(second[0].G_whitened, 0.0));
  EXPECT_TRUE(first[0].numerical.N.isApprox(second[0].numerical.N, 0.0));
  EXPECT_TRUE(first[0].numerical.R.isApprox(second[0].numerical.R, 0.0));
  EXPECT_EQ(first[0].linearization_id, second[0].linearization_id);
  EXPECT_DOUBLE_EQ(first[0].numerical.eta, second[0].numerical.eta);
  EXPECT_DOUBLE_EQ(first[0].numerical.s_m, second[0].numerical.s_m);
}

TEST(NlosScoring, ExcludedOtherCandidateGroupCannotAffectCurrentGroup) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::OracleSegment other;
  other.segment_id = "separate-anchor-2";
  other.segment_ordinal = 1;
  other.tag_id = 7;
  other.anchor_id = 2;
  other.start_time = 2.0;
  other.end_time = 3.0;
  other.duration = 1.0;
  for (const auto& observation : fixture.plan.observations)
    if (observation.anchor_id == 2) other.obs_ids.push_back(observation.obs_id);
  other.observation_count = other.obs_ids.size();
  fixture.support.segments.push_back(other);
  uifgo::RefitOptions options;
  options.lm_relative_tolerance = 1.0e-12;
  options.lm_absolute_tolerance = 1.0e-12;
  const auto refit = uifgo::SegmentRefitter(options).Run(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, fixture.support);
  ASSERT_TRUE(refit.converged()) << refit.reason;
  const auto baseline = uifgo::ScoreRefitRecoverability(
      refit, fixture.support, fixture.plan, fixture.cfg);
  ASSERT_EQ(baseline.size(), 2u);

  uifgo::SegmentRefitResult changed = refit;
  for (size_t index = 0; index < changed.factor_metadata.size(); ++index) {
    const auto& meta = changed.factor_metadata[index];
    if (meta.segment_id != other.segment_id) continue;
    const auto record = std::find_if(
        fixture.plan.observations.begin(), fixture.plan.observations.end(),
        [&](const uifgo::ObservationRecord& item) {
          return item.obs_id == meta.obs_id;
        });
    ASSERT_NE(record, fixture.plan.observations.end());
    const auto anchor = std::find_if(
        fixture.cfg.anchors.begin(), fixture.cfg.anchors.end(),
        [&](const uifgo::AnchorConfig& item) {
          return item.id == record->anchor_id;
        });
    changed.graph.replace(index, uifgo::MakeSegmentUwbFactor(
        X(record->keyframe_id), C(1), anchor->pos,
        fixture.cfg.lever_arm_init, record->raw_range + 10.0, 0.5, 0.0));
  }
  const auto perturbed = uifgo::ScoreRefitRecoverability(
      changed, fixture.support, fixture.plan, fixture.cfg);
  ASSERT_EQ(perturbed.size(), 2u);
  EXPECT_TRUE(baseline[0].F_whitened.isApprox(perturbed[0].F_whitened, 0.0));
  EXPECT_TRUE(baseline[0].G_whitened.isApprox(perturbed[0].G_whitened, 0.0));
  EXPECT_TRUE(baseline[0].numerical.N.isApprox(perturbed[0].numerical.N, 0.0));
  EXPECT_TRUE(baseline[0].numerical.R.isApprox(perturbed[0].numerical.R, 0.0));
  EXPECT_EQ(baseline[0].linearization_id, perturbed[0].linearization_id);
  EXPECT_NE(baseline[1].linearization_id, perturbed[1].linearization_id);
  EXPECT_FALSE(baseline[1].G_whitened.isApprox(perturbed[1].G_whitened, 0.0));
}

TEST(NlosScoring, OnlineBetaIsNuisanceAndItsRealPriorResolvesConfounding) {
  uifgo::Config cfg;
  cfg.calib_lever = false;
  cfg.calib_anchor = false;
  cfg.calib_range_bias = true;
  cfg.calib_td = false;
  cfg.lever_arm_init = gtsam::Point3(0.1, 0.0, 0.0);
  cfg.anchors = {{1, gtsam::Point3(4.0, 0.0, 0.0), 0.1}};
  const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(1.0, 1.0, 0.0));
  const double beta = 0.2, amplitude = 0.4, sigma = 0.1;
  const double geometric =
      (pose.transformFrom(cfg.lever_arm_init) - cfg.anchors[0].pos).norm();
  const double raw = geometric + beta + amplitude;
  const auto range = uifgo::MakeOnlineBetaSegmentUwbFactor(
      X(0), Z(1), C(0), cfg.anchors[0].pos, cfg.lever_arm_init, raw, sigma);
  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
  auto beta_noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.2);

  uifgo::SegmentRefitResult refit;
  refit.status = uifgo::SegmentRefitStatus::CONVERGED;
  refit.values.insert(X(0), pose);
  refit.values.insert<double>(Z(1), beta);
  refit.values.insert<double>(C(0), amplitude);
  refit.graph.addPrior(X(0), pose, pose_noise);
  refit.factor_metadata.push_back({0, 0, "preserved_non_uwb", {X(0)}, ""});
  refit.graph.addPrior<double>(Z(1), beta, beta_noise);
  refit.factor_metadata.push_back({1, 0, "preserved_non_uwb", {Z(1)}, ""});
  refit.graph.add(range);
  refit.factor_metadata.push_back(
      {2, 42, "uwb_segment_range",
       std::vector<gtsam::Key>(range->keys().begin(), range->keys().end()),
       "online"});
  uifgo::SegmentEstimate estimate;
  estimate.segment_id = "online";
  estimate.segment_ordinal = 0;
  estimate.amplitude_key = C(0);
  estimate.tag_id = 7;
  estimate.anchor_id = 1;
  estimate.start_time = 0.0;
  estimate.end_time = 1.0;
  estimate.observation_count = 1;
  refit.segments.push_back(estimate);
  uifgo::OracleSupport support;
  uifgo::OracleSegment segment;
  segment.segment_id = "online";
  segment.segment_ordinal = 0;
  segment.tag_id = 7;
  segment.anchor_id = 1;
  segment.obs_ids = {42};
  segment.observation_count = 1;
  segment.start_time = 0.0;
  segment.end_time = 1.0;
  support.segments.push_back(segment);
  uifgo::PaperInputPlan plan;
  uifgo::ObservationRecord observation;
  observation.obs_id = 42;
  observation.tag_id = 7;
  observation.anchor_id = 1;
  observation.keyframe_id = 0;
  observation.raw_range = raw;
  observation.nominal_sigma = sigma;
  observation.valid = observation.planned = true;
  plan.observations.push_back(observation);

  const auto with_prior = uifgo::ScoreRefitRecoverability(
      refit, support, plan, cfg);
  ASSERT_EQ(with_prior.size(), 1u);
  EXPECT_TRUE(with_prior[0].numerical.valid_score());
  EXPECT_TRUE(std::any_of(with_prior[0].key_columns.begin(),
                          with_prior[0].key_columns.end(),
                          [](const uifgo::KeyColumnMeta& meta) {
                            return meta.key == Z(1) &&
                                   meta.role == "STATIC_BETA_NUISANCE";
                          }));

  uifgo::SegmentRefitResult no_prior = refit;
  no_prior.graph = gtsam::NonlinearFactorGraph();
  no_prior.factor_metadata.clear();
  no_prior.graph.addPrior(X(0), pose, pose_noise);
  no_prior.factor_metadata.push_back(
      {0, 0, "preserved_non_uwb", {X(0)}, ""});
  no_prior.graph.add(range);
  no_prior.factor_metadata.push_back(
      {1, 42, "uwb_segment_range",
       std::vector<gtsam::Key>(range->keys().begin(), range->keys().end()),
       "online"});
  const auto without_prior = uifgo::ScoreRefitRecoverability(
      no_prior, support, plan, cfg);
  ASSERT_EQ(without_prior.size(), 1u);
  EXPECT_EQ(without_prior[0].numerical.status,
            uifgo::RecoverabilityStatus::RANK_DEFICIENT);
  EXPECT_TRUE(without_prior[0].numerical.s_is_infinite);
  EXPECT_GT(with_prior[0].numerical.R(0, 0),
            without_prior[0].numerical.R(0, 0));
}

TEST(AutomaticEngineeringPipeline,
     NonemptyEligibleCandidateReachesValidScoreWithoutOracleOrGt) {
  auto fixture = MakeRefitFixture(0.4);
  uifgo::DiscoveryOptions discovery_options;
  discovery_options.fused_lasso.lambda_l1 = 0.01;
  discovery_options.fused_lasso.lambda_tv = 0.1;
  discovery_options.active_bias_min_m = 0.1;
  discovery_options.change_point_min_m = 1.0;
  discovery_options.merge_max_difference_m = 1.0;
  discovery_options.short_min_count = 2;
  discovery_options.short_min_duration_s = 0.5;
  discovery_options.max_outer_iterations = 50;
  discovery_options.scaled_step_tolerance = 1e-6;
  discovery_options.observation_bias_scale_m = 1.0;
  discovery_options.conditional_lm.max_iterations = 100;
  discovery_options.conditional_lm.relative_tolerance = 1e-9;
  discovery_options.conditional_lm.absolute_tolerance = 1e-12;
  discovery_options.conditional_lm.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  uifgo::DiscoveryContext context;
  context.input_plan_hash = "sha256:" + uifgo::Sha256Hex("engineering-plan");
  context.source_hash = "sha256:" + uifgo::Sha256Hex("engineering-input");
  context.config_hash = "sha256:" + uifgo::Sha256Hex("engineering-config");
  context.calibration_hash =
      "sha256:" + uifgo::Sha256Hex("explicit-synthetic-calibration");
  context.solver_config_hash =
      "a19-policy-sha256:" + uifgo::Sha256Hex("r05-engineering-solver");

  size_t stage1_callbacks = 0;
  auto stage1_request = uifgo::MakeA19DevelopmentStage1Request(
      context.solver_config_hash,
      [&](size_t, const gtsam::NonlinearFactorGraph& graph,
          const gtsam::Values& values,
          const uifgo::CheckedLmOptions& options,
          const std::vector<uifgo::DevelopmentRangeConstant>& constants) {
        ++stage1_callbacks;
        EXPECT_EQ(constants.size(), fixture.plan.observations.size());
        auto engineering_options = options;
        engineering_options.policy =
            uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1;
        return uifgo::RunCheckedConditionalLm(
            graph, values, engineering_options);
      });
  const auto discovery = uifgo::AutomaticSupportProvider(discovery_options)
      .RunDevelopmentStage1(
          fixture.graph, fixture.values, fixture.metadata, fixture.plan,
          fixture.cfg, context, nullptr, &stage1_request);
  ASSERT_TRUE(discovery.converged()) << discovery.reason;
  EXPECT_GT(stage1_callbacks, 0u);
  EXPECT_EQ(discovery.partition.schema,
            uifgo::kA19DevelopmentStage1Schema);
  EXPECT_EQ(discovery.partition.provider,
            uifgo::kDevelopmentNonconsumableProvider);
  EXPECT_EQ(discovery.partition.solver_config_hash,
            context.solver_config_hash);
  ASSERT_FALSE(discovery.iterations.empty());
  for (const auto& trace : discovery.iterations) {
    const auto& check = trace.conditional_lm_convergence;
    EXPECT_TRUE(check.initial_error_valid);
    EXPECT_TRUE(check.check_evaluated);
    EXPECT_EQ(check.check_status, "EVALUATED_MATCHED_LINKED_GTSAM");
    EXPECT_TRUE(check.check_result);
    EXPECT_TRUE(check.predicate_union_matches_check_result);
    EXPECT_TRUE(trace.pre_chain_navigation_stationarity.valid)
        << trace.pre_chain_navigation_stationarity.reason;
    EXPECT_TRUE(trace.post_chain_navigation_stationarity.valid)
        << trace.post_chain_navigation_stationarity.reason;
    EXPECT_TRUE(trace.stationarity_audits_share_navigation_values);
    EXPECT_DOUBLE_EQ(
        trace.navigation_gradient_objective,
        trace.post_chain_navigation_stationarity.max_scaled_gradient_objective);
    EXPECT_DOUBLE_EQ(
        trace.navigation_roundoff_allowance_objective,
        trace.post_chain_navigation_stationarity.roundoff_allowance_objective);
    EXPECT_EQ(trace.navigation_stationarity_ok,
              trace.post_chain_navigation_stationarity.stationary);
    EXPECT_GE(trace.pre_chain_stationarity_seconds, 0.0);
    EXPECT_GE(trace.post_chain_stationarity_seconds, 0.0);
    EXPECT_GE(trace.added_diagnostics_seconds, 0.0);
  }
  ASSERT_FALSE(discovery.partition.segments.empty());
  EXPECT_TRUE(discovery.partition.source_path.empty());
  bool has_eligible = false;
  for (const auto& segment : discovery.partition.segments)
    has_eligible = has_eligible || !segment.short_support_debug;
  ASSERT_TRUE(has_eligible);

  uifgo::RefitOptions refit_options;
  refit_options.max_refit_iterations = 50;
  refit_options.lm_relative_tolerance = 1e-9;
  refit_options.lm_absolute_tolerance = 1e-12;
  size_t stage2_callbacks = 0;
  uifgo::DevelopmentStage2Request stage2_request;
  stage2_request.policy = "PAPER_CERTIFIED_PAIR_REDUCTION_V1";
  stage2_request.role = "development";
  stage2_request.implementation_identity = context.solver_config_hash;
  stage2_request.conditional_navigation = [&](
      size_t, const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& values, const uifgo::CheckedLmOptions& options,
      const std::vector<uifgo::DevelopmentRefitRangeConstant>& constants) {
    ++stage2_callbacks;
    EXPECT_EQ(constants.size(), fixture.plan.observations.size());
    auto engineering_options = options;
    engineering_options.policy =
        uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1;
    return uifgo::RunCheckedConditionalLm(
        graph, values, engineering_options);
  };
  const auto refit = uifgo::SegmentRefitter(refit_options)
      .RunDevelopmentStage2(
          fixture.graph, discovery.navigation_values, fixture.metadata,
          fixture.plan, fixture.cfg, discovery.partition, &stage2_request);
  ASSERT_TRUE(refit.converged()) << refit.reason;
  EXPECT_GT(stage2_callbacks, 0u);
  const char* evidence_root = std::getenv("UIFGO_R05_HANDSHAKE_OUTPUT_ROOT");
  if (evidence_root && std::string(evidence_root).size()) {
    const boost::filesystem::path root(evidence_root);
    ASSERT_TRUE(boost::filesystem::create_directories(root / "stage2"));
    uifgo::InferenceIdentityContext export_context;
    export_context.input_sha256 = context.source_hash;
    export_context.config_sha256 = context.config_hash;
    export_context.input_plan_sha256 = context.input_plan_hash;
    export_context.support_partition_sha256 = discovery.partition.partition_hash;
    export_context.calibration_sha256 = context.calibration_hash;
    export_context.solver_config_sha256 = context.solver_config_hash;
    const auto stage2_export = uifgo::WriteDevelopmentStage2Bundle(
        (root / "stage2").string(), refit.graph, refit.values,
        refit.factor_metadata, export_context);
    ASSERT_TRUE(stage2_export.ok) << stage2_export.reason;
  }
  const auto scores = uifgo::ScoreRefitRecoverability(
      refit, discovery.partition, fixture.plan, fixture.cfg);
  ASSERT_FALSE(scores.empty());
  EXPECT_TRUE(std::any_of(scores.begin(), scores.end(), [](const auto& score) {
    return score.eligible && score.valid_score_exported;
  }));
  if (evidence_root && std::string(evidence_root).size())
    uifgo::MarkDevelopmentStage2ScoreComplete(
        (boost::filesystem::path(evidence_root) / "stage2").string());

  uifgo::GateThresholds gate;
  gate.tau_eta = 0.0;
  gate.tau_s_m = 1.0e6;
  gate.tau_gamma = 1.0e6;
  gate.parameter_provenance = uifgo::kT08DevelopmentGateLabel;
  uifgo::InferenceIdentityContext final_context;
  final_context.input_sha256 = context.source_hash;
  final_context.config_sha256 = context.config_hash;
  final_context.input_plan_sha256 = context.input_plan_hash;
  final_context.support_partition_sha256 =
      discovery.partition.partition_hash;
  final_context.calibration_sha256 = context.calibration_hash;
  final_context.solver_config_sha256 = context.solver_config_hash;
  const auto final = uifgo::FinalInferenceEngine(
      gate, refit_options, {}, {}, uifgo::FinalGatePolicy::FULL_GATE,
      &stage2_request)
      .Run(fixture.graph, fixture.metadata, refit, discovery.partition,
           scores, fixture.plan, fixture.cfg, final_context);
  ASSERT_TRUE(final.valid_estimate()) << final.reason;
  EXPECT_TRUE(std::any_of(final.decisions.begin(), final.decisions.end(),
                          [](const auto& decision) {
                            return decision.decision ==
                                   uifgo::GroupDecision::USE;
                          }));
  EXPECT_EQ(final.factor_audit.corrected_pseudo_range_count, 0u);

  if (evidence_root && std::string(evidence_root).size()) {
    const boost::filesystem::path root(evidence_root);
    ASSERT_TRUE(boost::filesystem::create_directories(root / "final"));
    uifgo::WriteInferenceArtifacts((root / "final").string(), final);
    std::ofstream truth((root / "evaluation_only_truth.csv").string());
    truth << "timestamp,x,y,z\n0,1,1.5,0.6\n1,1.4,1.8,0.7\n";
    std::ofstream handshake((root / "handshake.json").string());
    handshake
        << "{\"schema\":\"T10_A19_R05_DIRECT_HANDSHAKE_V1\","
           "\"development_schema\":\""
        << discovery.partition.schema << "\",\"provider\":\""
        << discovery.partition.provider << "\",\"stage1_policy\":\""
        << stage1_request.policy << "\",\"stage2_policy\":\""
        << stage2_request.policy << "\",\"solver_identity\":\""
        << context.solver_config_hash << "\",\"partition_hash\":\""
        << discovery.partition.partition_hash
        << "\",\"final_parent_partition_hash\":\""
        << final.identity_context.support_partition_sha256
        << "\",\"final_solver_identity\":\""
        << final.identity_context.solver_config_sha256
        << "\",\"stage1_callbacks\":" << stage1_callbacks
        << ",\"stage2_callbacks\":" << stage2_callbacks
        << ",\"score_groups\":" << scores.size()
        << ",\"final_valid\":true,\"consumable\":false}\n";
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


TEST(SegmentRefitter, ExactZeroResidualMetadataAcceptsGtsamNegativeZero) {
  auto fixture = MakeRefitFixture(0.4);
  // Set a real noncandidate range to exactly its prediction at these Values.
  // ExpressionFactor returns -0 while the arithmetic residual returns +0.
  auto& record = fixture.plan.observations.at(1);
  const auto pose = fixture.values.at<gtsam::Pose3>(X(record.keyframe_id));
  const auto anchor = fixture.cfg.anchors.at(1).pos;
  record.raw_range = (pose.transformFrom(fixture.cfg.lever_arm_init) - anchor).norm();
  for (const auto& meta : fixture.metadata) {
    if (meta.obs_id == record.obs_id)
      fixture.graph.at(meta.factor_index) = uifgo::MakeUwbFactor(
          X(record.keyframe_id), 0, 0, 0, anchor,
          fixture.cfg.lever_arm_init, record.raw_range, record.nominal_sigma,
          false, false, false, 0.0);
  }
  auto support = uifgo::ToSupportPartition(fixture.support);
  support.solver_config_hash = "a19-policy-sha256:signed-zero-regression";
  uifgo::DevelopmentStage2Request request;
  request.policy = "PAPER_CERTIFIED_PAIR_REDUCTION_V1";
  request.role = "development";
  request.implementation_identity = support.solver_config_hash;
  bool reached = false;
  request.conditional_navigation = [&](size_t,
      const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
      const uifgo::CheckedLmOptions&,
      const std::vector<uifgo::DevelopmentRefitRangeConstant>& ranges) {
    reached = true;
    for (const auto& range : ranges) {
      if (range.obs_id != record.obs_id) continue;
      const auto factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(
          graph.at(range.factor_index));
      const double actual = factor->unwhitenedError(values)[0];
      EXPECT_EQ(actual, 0.0);
      EXPECT_TRUE(std::signbit(actual));
      EXPECT_EQ(range.expected_unwhitened_residual, 0.0);
      EXPECT_FALSE(std::signbit(range.expected_unwhitened_residual));
    }
    uifgo::CheckedLmResult result;
    result.values = values;
    result.reason = "SIGNED_ZERO_CALLBACK_REACHED";
    return result;
  };
  const auto result = uifgo::SegmentRefitter(uifgo::RefitOptions{}).RunDevelopmentStage2(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, support, &request);
  EXPECT_TRUE(reached) << result.reason;
  EXPECT_EQ(result.reason, "SIGNED_ZERO_CALLBACK_REACHED");
  // A genuine changed measurement is still rejected before callback.
  reached = false;
  record.raw_range += 0.001;
  const auto invalid = uifgo::SegmentRefitter(uifgo::RefitOptions{}).RunDevelopmentStage2(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, support, &request);
  EXPECT_FALSE(reached);
  EXPECT_EQ(invalid.status, uifgo::SegmentRefitStatus::INVALID_INPUT);
}
