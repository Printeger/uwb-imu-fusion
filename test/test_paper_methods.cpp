#include <gtest/gtest.h>
#include <iomanip>
#include "uifgo/paper_robust_noise.h"
#include <gtsam/linear/JacobianFactor.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuBias.h>

#include <set>
#include <cmath>

#include "uifgo/paper_methods.h"
#include "uifgo/uwb_factor.h"

namespace uifgo {
namespace {

GroupRecoverabilityScore Score(double eta, double s, double gamma,
                               bool eligible = true) {
  GroupRecoverabilityScore score;
  score.group.group_id = "g";
  score.group.segment_ordinals = {0};
  score.eligible = eligible;
  score.status = eligible ? "OK" : "SHORT_SUPPORT";
  score.linearization_id = "lin";
  score.numerical.status = RecoverabilityStatus::OK;
  score.numerical.eta = eta;
  score.numerical.s_m = s;
  score.numerical.s_is_infinite = false;
  score.numerical.N = Eigen::MatrixXd::Constant(1, 1, 4.0);
  SegmentFitScore fit;
  fit.gamma = gamma;
  score.segment_fit.push_back(fit);
  return score;
}

PolicyThresholds Thresholds() {
  PolicyThresholds value;
  value.tau_eta = 0.5;
  value.tau_s_m = 2.0;
  value.tau_gamma = 3.0;
  value.tau_nominal_curvature_m2_inv = 4.0;
  value.parameter_provenance = "TEST_ONLY";
  return value;
}

struct WarmStartFixture {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::vector<FactorMeta> metadata;
};

WarmStartFixture MakeWarmStartFixture() {
  WarmStartFixture fixture;
  fixture.graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      gtsam::Symbol('x', 0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 0.1)));
  fixture.graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      gtsam::Symbol('v', 0), gtsam::Vector3(0.0, 0.0, 0.0),
      gtsam::noiseModel::Isotropic::Sigma(3, 0.1)));
  fixture.graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      gtsam::Symbol('b', 0), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Isotropic::Sigma(6, 0.1)));
  fixture.graph.add(MakeUwbFactor(
      gtsam::Symbol('x', 0), 0, 0, 0,
      gtsam::Point3(10.0, 0.0, 0.0),
      gtsam::Point3(0.0, 0.0, 0.0), 10.0, 0.15,
      false, false, false));
  fixture.values.insert(gtsam::Symbol('x', 0), gtsam::Pose3());
  fixture.values.insert(gtsam::Symbol('v', 0),
                        gtsam::Vector3(0.0, 0.0, 0.0));
  fixture.values.insert(gtsam::Symbol('b', 0),
                        gtsam::imuBias::ConstantBias());
  FactorMeta meta;
  meta.factor_index = 3;
  meta.factor_type = "uwb_range";
  meta.obs_id = 42;
  meta.keys.assign(fixture.graph.at(3)->keys().begin(),
                   fixture.graph.at(3)->keys().end());
  fixture.metadata.push_back(meta);
  return fixture;
}

BaselineOptions RobustOptions(PaperMethod method, double scale) {
  BaselineOptions options;
  options.method = method;
  options.robust_scale = scale;
  options.parameter_provenance = "TEST_ONLY";
  options.keyframe_times_s = {0.0};
  options.lm.max_iterations = 20;
  return options;
}

IntermediateSeedQualityOptions WarmSeedQualityOptions() {
  IntermediateSeedQualityOptions options;
  options.position_envelope_m = 110.0;
  options.velocity_envelope_mps = 403.0;
  return options;
}

TEST(PaperMethods, RegistryContainsEveryContractModeExactlyOnce) {
  const auto& registry = CanonicalPaperMethodRegistry();
  EXPECT_EQ(registry.size(), 15u);
  std::set<std::string> names;
  for (const auto& item : registry) names.insert(item.canonical_name);
  EXPECT_EQ(names.size(), registry.size());
  EXPECT_FALSE(PaperMethodByName("all_range").requires_stage1);
  EXPECT_TRUE(PaperMethodByName("structured_bias_only").requires_stage1);
  EXPECT_TRUE(PaperMethodByName("full_gate").requires_stage2_cache);
  EXPECT_FALSE(PaperMethodByName("nominal_curvature").permits_final_trajectory);
  EXPECT_THROW(PaperMethodByName("gnc_rejection"), std::invalid_argument);
}

TEST(PaperMethods, PolicyEqualityPassesForAllComparators) {
  const auto score = Score(0.5, 2.0, 3.0);
  for (PaperMethod method : {PaperMethod::FIT_ONLY, PaperMethod::S_FIT,
                             PaperMethod::FULL_GATE, PaperMethod::ETA_ONLY,
                             PaperMethod::NOMINAL_CURVATURE}) {
    const auto rows = FreezePaperPolicyDecisions(method, {score}, Thresholds());
    ASSERT_EQ(rows.size(), 1u);
    EXPECT_EQ(rows[0].decision, GroupDecision::USE);
  }
}

TEST(PaperMethods, RequiredScoreAndIneligibleAreSuppressed) {
  auto unavailable = Score(0.5, 2.0, 3.0);
  unavailable.numerical.status = RecoverabilityStatus::NUMERICAL_FAILURE;
  const auto full = FreezePaperPolicyDecisions(
      PaperMethod::FULL_GATE, {unavailable}, Thresholds());
  EXPECT_EQ(full[0].reason_code, "SUPPRESS_REQUIRED_SCORE_UNAVAILABLE");
  const auto fit = FreezePaperPolicyDecisions(
      PaperMethod::FIT_ONLY, {unavailable}, Thresholds());
  EXPECT_EQ(fit[0].decision, GroupDecision::USE)
      << "fit-only must not require eta or s";
  const auto ineligible = FreezePaperPolicyDecisions(
      PaperMethod::FIT_ONLY, {Score(1.0, 0.1, 0.1, false)}, Thresholds());
  EXPECT_EQ(ineligible[0].decision, GroupDecision::SUPPRESS);
  EXPECT_NE(ineligible[0].reason_code.find("SUPPRESS_INELIGIBLE_"),
            std::string::npos);
}

TEST(PaperMethods, FinalEnginePoliciesUseTheSameRequiredPredicates) {
  GateThresholds gate;
  gate.tau_eta = 0.5;
  gate.tau_s_m = 2.0;
  gate.tau_gamma = 3.0;
  gate.parameter_provenance = kT08DevelopmentGateLabel;
  auto score = Score(0.5, 2.0, 3.0);
  score.valid_score_exported = true;
  EXPECT_EQ(FreezeGroupDecisions(
                {score}, gate, FinalGatePolicy::FIT_ONLY)[0].decision,
            GroupDecision::USE);
  EXPECT_EQ(FreezeGroupDecisions(
                {score}, gate, FinalGatePolicy::S_FIT)[0].decision,
            GroupDecision::USE);
  EXPECT_EQ(FreezeGroupDecisions(
                {score}, gate, FinalGatePolicy::ETA_ONLY)[0].decision,
            GroupDecision::USE);

  score.numerical.status = RecoverabilityStatus::NUMERICAL_FAILURE;
  EXPECT_EQ(FreezeGroupDecisions(
                {score}, gate, FinalGatePolicy::FIT_ONLY)[0].decision,
            GroupDecision::USE)
      << "fit-only final audit must not require eta or s";
  EXPECT_EQ(FreezeGroupDecisions(
                {score}, gate, FinalGatePolicy::S_FIT)[0].reason_code,
            "SUPPRESS_REQUIRED_SCORE_UNAVAILABLE");
  EXPECT_EQ(FreezeGroupDecisions(
                {score}, gate, FinalGatePolicy::ETA_ONLY)[0].reason_code,
            "SUPPRESS_REQUIRED_SCORE_UNAVAILABLE");
}

TEST(PaperMethods, StandardRobustLossMatchesFiniteDifferenceAndLinearization) {
  const auto kernel = gtsam::noiseModel::mEstimator::Cauchy::Create(2.3849);
  const auto noise = boost::make_shared<PaperRobustNoise>(kernel,
      gtsam::noiseModel::Isotropic::Sigma(1, 0.15));
  const gtsam::Key key = gtsam::Symbol('z', 0);
  gtsam::PriorFactor<double> factor(key, 0.0, noise);
  gtsam::Values values; values.insert<double>(key, 0.6);
  auto plus = values, minus = values;
  plus.update<double>(key, 0.600001); minus.update<double>(key, 0.599999);
  std::cout << std::setprecision(17) << "LINKED_ROBUST_LOSS actual=" << factor.error(values)
            << " expected=" << kernel->loss(4.0) << " finite_difference="
            << (factor.error(plus)-factor.error(minus))/2e-6
            << " expected_gradient=" << kernel->weight(4.0)*0.6/(0.15*0.15) << std::endl;
  EXPECT_NEAR(factor.error(values), kernel->loss(4.0), 1e-12);
  const auto linear = boost::dynamic_pointer_cast<gtsam::JacobianFactor>(factor.linearize(values));
  ASSERT_TRUE(linear);
  const auto ab = linear->jacobian();
  const double gradient = -(ab.first.transpose()*ab.second)[0];
  EXPECT_NEAR((factor.error(plus)-factor.error(minus))/2e-6, gradient, 1e-8);
  const auto huber = gtsam::noiseModel::mEstimator::Huber::Create(1.345);
  gtsam::PriorFactor<double> hf(key, 0.0, boost::make_shared<PaperRobustNoise>(huber,
      gtsam::noiseModel::Isotropic::Sigma(1, 0.15)));
  EXPECT_NEAR(hf.error(values), huber->loss(4.0), 1e-12);
  EXPECT_NEAR((hf.error(plus)-hf.error(minus))/2e-6, huber->weight(4.)*.6/(.15*.15), 1e-8);
}

TEST(PaperMethods, CatastrophicUwbCannotDominateDirectRobustBaseline) {
  const gtsam::Key key = gtsam::Symbol('x', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      key, gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 0.1)));
  graph.add(MakeUwbFactor(
      key, 0, 0, 0, gtsam::Point3(20.0, 0.0, 0.0),
      gtsam::Point3(0.0, 0.0, 0.0), 10.0, 0.1,
      false, false, false));
  gtsam::Values initial;
  initial.insert<gtsam::Pose3>(key, gtsam::Pose3());
  FactorMeta meta;
  meta.factor_index = 1; meta.obs_id = 42;
  meta.factor_type = "uwb_range"; meta.keys = {key};
  BaselineOptions options;
  options.parameter_provenance = "TEST_ONLY";
  options.keyframe_times_s = {0.0};
  options.robust_scale = 2.3849;
  options.lm.max_iterations = 20;
  const auto raw = RunPaperBaseline(graph, initial, {meta}, options);
  ASSERT_TRUE(raw.valid) << raw.reason;
  ASSERT_TRUE(raw.preliminary.converged);
  EXPECT_EQ(raw.solver_certificate.status,
            SolverCertificateStatus::CERTIFIED_SUCCESS);
  EXPECT_TRUE(raw.final_values.at<gtsam::Pose3>(key).equals(
      raw.preliminary.values.at<gtsam::Pose3>(key), 1e-12));
  EXPECT_EQ(raw.final.iterations, raw.preliminary.iterations);
  EXPECT_GT(std::abs(raw.final_values.at<gtsam::Pose3>(key).x()), 4.0)
      << "plain high-confidence Gaussian UWB should expose the stress case";
  options.method = PaperMethod::ROBUST_CAUCHY;
  const auto robust = RunPaperBaseline(graph, initial, {meta}, options);
  ASSERT_TRUE(robust.valid) << robust.reason;
  EXPECT_EQ(robust.initialization_path,
            "COMMON_INITIAL_VALUES_DIRECT_ROBUST_V1");
  EXPECT_FALSE(robust.preliminary.converged);
  EXPECT_EQ(robust.preliminary.reason,
            "NOT_RUN_ROBUST_DIRECT_INITIALIZATION");
  EXPECT_EQ(robust.solver_certificate.status,
            SolverCertificateStatus::CERTIFIED_SUCCESS);
  EXPECT_LT(std::abs(robust.final_values.at<gtsam::Pose3>(key).x()), 0.1);
  EXPECT_GT(std::abs(raw.final_values.at<gtsam::Pose3>(key).x() -
                     robust.final_values.at<gtsam::Pose3>(key).x()), 4.0);
}

TEST(PaperMethods, FixedRejectionIsSingleFrozenMaskWithEqualityRetained) {
  EXPECT_TRUE(FixedRejectionKeeps(3.0, 3.0));
  EXPECT_FALSE(FixedRejectionKeeps(std::nextafter(3.0, 4.0), 3.0));
  const gtsam::Key key = gtsam::Symbol('x', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      key, gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 0.1)));
  graph.add(MakeUwbFactor(
      key, 0, 0, 0, gtsam::Point3(10.0, 0.0, 0.0),
      gtsam::Point3(0.0, 0.0, 0.0), 8.0, 1.0,
      false, false, false));
  gtsam::Values initial;
  initial.insert<gtsam::Pose3>(key, gtsam::Pose3());
  FactorMeta meta;
  meta.factor_index = 1;
  meta.obs_id = 42;
  meta.factor_type = "uwb_range";
  meta.keys = {key};
  BaselineOptions options;
  options.method = PaperMethod::FIXED_REJECTION;
  options.rejection_threshold_sigma = 0.5;
  options.parameter_provenance = "TEST_ONLY";
  options.keyframe_times_s = {0.0};
  options.lm.max_iterations = 20;
  const auto result = RunPaperBaseline(graph, initial, {meta}, options);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.kept_uwb_obs_ids.empty());
  EXPECT_EQ(result.rejected_uwb_obs_ids, std::vector<std::uint64_t>({42}));
  EXPECT_EQ(result.final_graph.size(), 1u);
  EXPECT_TRUE(result.solver_certificate.certified_success());
}

TEST(PaperMethods, SmallObjectiveTerminationCannotCertifyNonstationaryState) {
  const gtsam::Key key = gtsam::Symbol('x', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      key, gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 0.1)));
  gtsam::Values bad;
  bad.insert(key, gtsam::Pose3(gtsam::Rot3(),
                               gtsam::Point3(10.0, 0.0, 0.0)));
  SolverCertificateRequest request;
  request.termination_success = true;
  request.termination_reason =
      "CONDITIONAL_LM_CONVERGED_SMALL_OBJECTIVE_CHANGE";
  request.factor_integrity_passed = true;
  request.factor_integrity_reason = "SYNTHETIC_FACTOR_KEYS_OK";
  request.state_times_s = {0.0};
  const auto certificate = CertifySolverResult(graph, bad, request);
  EXPECT_EQ(certificate.status,
            SolverCertificateStatus::CERTIFIED_FAILURE);
  EXPECT_TRUE(certificate.termination_success);
  EXPECT_EQ(certificate.termination_reason,
            "CONDITIONAL_LM_CONVERGED_SMALL_OBJECTIVE_CHANGE");
  EXPECT_TRUE(certificate.navigation_stationarity.valid);
  EXPECT_FALSE(certificate.navigation_stationarity_passed);
  EXPECT_EQ(certificate.reason,
            "NAVIGATION_STATIONARITY_FAILED:NOT_STATIONARY");
}

TEST(PaperMethods,
     WellBehavedGraphCertifiesFromGraphValuesAndDeclaredIntegrityOnly) {
  const gtsam::Key key = gtsam::Symbol('x', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      key, gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(6, 0.1)));
  gtsam::Values values;
  values.insert(key, gtsam::Pose3());
  SolverCertificateRequest request;
  request.termination_success = true;
  request.termination_reason = "SYNTHETIC_TERMINATION_SUCCESS";
  request.factor_integrity_passed = true;
  request.factor_integrity_reason = "SYNTHETIC_FACTOR_KEYS_OK";
  request.state_times_s = {0.0};
  const auto certificate = CertifySolverResult(graph, values, request);
  EXPECT_TRUE(certificate.certified_success()) << certificate.reason;
  EXPECT_EQ(certificate.policy_version, "PAPER_SOLVER_CERTIFICATE_V1");
  EXPECT_TRUE(certificate.state_values_finite);
  EXPECT_TRUE(certificate.objective_finite);
  EXPECT_TRUE(certificate.graph_values_keys_match);
  EXPECT_TRUE(certificate.temporal_integrity_passed);
  EXPECT_TRUE(certificate.navigation_stationarity_passed);
  // The certificate API deliberately has no GT, ATE, truth, or oracle input.
  EXPECT_TRUE(certificate.max_position_norm_m >= 0.0);
}

TEST(PaperMethods,
     UncertifiedFiniteHuberTerminalIsRetainedOnlyAsIntermediateSeed) {
  const auto fixture = MakeWarmStartFixture();
  auto huber_options = RobustOptions(PaperMethod::ROBUST_HUBER, 1.345);
  huber_options.keyframe_times_s.clear();  // Force certificate failure only.
  const auto huber = RunPaperBaseline(
      fixture.graph, fixture.values, fixture.metadata, huber_options);
  ASSERT_FALSE(huber.valid);
  ASSERT_EQ(huber.solver_certificate.status,
            SolverCertificateStatus::CERTIFIED_FAILURE);
  ASSERT_FALSE(huber.final.values.empty());
  EXPECT_TRUE(huber.final_values.empty());

  const auto seed = RetainIntermediateOptimizationSeed(
      huber, fixture.values, fixture.metadata, {0.0},
      WarmSeedQualityOptions());
  EXPECT_EQ(seed.role, "INTERMEDIATE_OPTIMIZATION_SEED");
  EXPECT_TRUE(seed.available);
  EXPECT_TRUE(seed.quality.accepted) << seed.quality.reason;
  EXPECT_TRUE(seed.eligible_for_cauchy_initialization);
  EXPECT_FALSE(seed.eligible_for_trajectory_export);
  EXPECT_FALSE(seed.eligible_for_gt_evaluation);
  EXPECT_FALSE(seed.values.empty());
}

TEST(PaperMethods, InvalidHuberTerminalCannotBecomeWarmStart) {
  const auto fixture = MakeWarmStartFixture();
  BaselineResult huber;
  huber.final_graph = fixture.graph;
  huber.final.values = fixture.values;
  gtsam::Vector3 invalid_velocity = gtsam::Vector3::Zero();
  invalid_velocity[0] = std::numeric_limits<double>::quiet_NaN();
  huber.final.values.update(gtsam::Symbol('v', 0), invalid_velocity);
  const auto seed = RetainIntermediateOptimizationSeed(
      huber, fixture.values, fixture.metadata, {0.0},
      WarmSeedQualityOptions());
  EXPECT_TRUE(seed.available);
  EXPECT_FALSE(seed.quality.accepted);
  EXPECT_FALSE(seed.eligible_for_cauchy_initialization);
  EXPECT_TRUE(seed.values.empty());
  EXPECT_FALSE(seed.eligible_for_trajectory_export);
  EXPECT_FALSE(seed.eligible_for_gt_evaluation);
}

TEST(PaperMethods,
     WarmStartPreservesPhysicalGraphAndCauchyRequiresFinalCertificate) {
  const auto fixture = MakeWarmStartFixture();
  std::vector<const void*> factor_addresses;
  for (const auto& factor : fixture.graph) factor_addresses.push_back(factor.get());
  const double physical_error_before = fixture.graph.error(fixture.values);
  const auto result = RunHuberToCauchyWarmStart(
      fixture.graph, fixture.values, fixture.metadata,
      RobustOptions(PaperMethod::ROBUST_HUBER, 1.345),
      RobustOptions(PaperMethod::ROBUST_CAUCHY, 2.3849),
      WarmSeedQualityOptions());
  ASSERT_TRUE(result.intermediate_seed.quality.accepted)
      << result.intermediate_seed.quality.reason;
  ASSERT_TRUE(result.cauchy_attempted);
  ASSERT_TRUE(result.cauchy_final.valid) << result.cauchy_final.reason;
  EXPECT_TRUE(result.cauchy_final.solver_certificate.certified_success());
  EXPECT_FALSE(result.cauchy_final.final_values.empty());
  EXPECT_EQ(fixture.graph.error(fixture.values), physical_error_before);
  ASSERT_EQ(fixture.graph.size(), factor_addresses.size());
  for (size_t i = 0; i < fixture.graph.size(); ++i)
    EXPECT_EQ(fixture.graph.at(i).get(), factor_addresses[i]);

  auto bad_timeline = RobustOptions(PaperMethod::ROBUST_CAUCHY, 2.3849);
  bad_timeline.keyframe_times_s.clear();
  const auto uncertified = RunPaperBaseline(
      fixture.graph, fixture.values, fixture.metadata, bad_timeline);
  EXPECT_FALSE(uncertified.valid);
  EXPECT_EQ(uncertified.solver_certificate.status,
            SolverCertificateStatus::CERTIFIED_FAILURE);
  EXPECT_TRUE(uncertified.final_values.empty());
  EXPECT_FALSE(uncertified.final.values.empty());
}

}  // namespace
}  // namespace uifgo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
