#include <gtest/gtest.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

#include <set>
#include <cmath>

#include "uifgo/paper_methods.h"

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

TEST(PaperMethods, FixedRejectionIsSingleFrozenMaskWithEqualityRetained) {
  EXPECT_TRUE(FixedRejectionKeeps(3.0, 3.0));
  EXPECT_FALSE(FixedRejectionKeeps(std::nextafter(3.0, 4.0), 3.0));
  const gtsam::Key key = gtsam::Symbol('z', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<double>(
      key, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0)));
  graph.add(gtsam::PriorFactor<double>(
      key, 2.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0)));
  gtsam::Values initial;
  initial.insert<double>(key, 0.0);
  FactorMeta meta;
  meta.factor_index = 1;
  meta.obs_id = 42;
  meta.factor_type = "uwb_range";
  meta.keys = {key};
  BaselineOptions options;
  options.method = PaperMethod::FIXED_REJECTION;
  options.rejection_threshold_sigma = 0.5;
  options.parameter_provenance = "TEST_ONLY";
  options.lm.max_iterations = 20;
  const auto result = RunPaperBaseline(graph, initial, {meta}, options);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_TRUE(result.kept_uwb_obs_ids.empty());
  EXPECT_EQ(result.rejected_uwb_obs_ids, std::vector<std::uint64_t>({42}));
  EXPECT_EQ(result.final_graph.size(), 1u);
}

}  // namespace
}  // namespace uifgo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
