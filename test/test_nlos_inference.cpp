#include "uifgo/nlos_inference.h"
#include "uifgo/nlos_inference_io.h"
#include "uifgo/nlos_discovery.h"
#include "uifgo/hash_utils.h"

#include <boost/filesystem.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include <map>
#include <set>

#include "uifgo/uwb_factor.h"

namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

uifgo::GateThresholds DevelopmentGate(double eta = 0.0,
                                      double s_m = 1e6,
                                      double gamma = 1e6) {
  uifgo::GateThresholds gate;
  gate.tau_eta = eta;
  gate.tau_s_m = s_m;
  gate.tau_gamma = gamma;
  gate.parameter_provenance = uifgo::kT08DevelopmentGateLabel;
  return gate;
}

uifgo::InferenceIdentityContext IdentityContext(
    const std::string& suffix = "default") {
  uifgo::InferenceIdentityContext context;
  context.input_sha256 = "sha256:input-" + suffix;
  context.config_sha256 = "sha256:config-" + suffix;
  context.input_plan_sha256 = "sha256:plan-" + suffix;
  context.support_partition_sha256 = "sha256:support-" + suffix;
  context.calibration_sha256 = "sha256:calibration-" + suffix;
  context.solver_config_sha256 = "sha256:solver-" + suffix;
  return context;
}

uifgo::GroupRecoverabilityScore SyntheticScore(
    double eta, double s_m, double gamma,
    uifgo::RecoverabilityStatus status = uifgo::RecoverabilityStatus::OK,
    bool eligible = true) {
  uifgo::GroupRecoverabilityScore score;
  score.group.group_id = "group_0";
  score.group.start_time = 0.0;
  score.group.end_time = 1.0;
  score.group.segment_ordinals = {0, 1};
  score.numerical.status = status;
  score.numerical.eta = eta;
  score.numerical.s_m = s_m;
  score.numerical.s_is_infinite = !std::isfinite(s_m);
  score.eligible = eligible;
  score.valid_score_exported = eligible && score.numerical.valid_score();
  score.linearization_id = "decision-linearity";
  for (size_t ordinal : score.group.segment_ordinals) {
    uifgo::SegmentFitScore fit;
    fit.segment_ordinal = ordinal;
    fit.segment_id = "s" + std::to_string(ordinal);
    fit.gamma = gamma;
    score.segment_fit.push_back(fit);
  }
  return score;
}

struct Fixture {
  uifgo::Config cfg;
  uifgo::PaperInputPlan plan;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::vector<uifgo::FactorMeta> metadata;
  uifgo::SupportPartition support;
};

Fixture MakeFixture(bool separated_groups = false,
                    bool inject_candidate_bias = true) {
  Fixture fixture;
  fixture.cfg.calib_lever = false;
  fixture.cfg.calib_anchor = false;
  fixture.cfg.calib_range_bias = false;
  fixture.cfg.calib_td = false;
  fixture.cfg.lever_arm_init = gtsam::Point3(0.1, -0.03, 0.02);
  fixture.cfg.anchors = {
      {1, gtsam::Point3(-4, -3, 1), 0.1},
      {2, gtsam::Point3(5, -3, 2), 0.1},
      {3, gtsam::Point3(-3, 5, 3), 0.1},
      {4, gtsam::Point3(4, 4, -1), 0.1},
  };
  fixture.support.schema = "t08_fixture_support_v1";
  fixture.support.provider = "automatic_engineering_fixture_no_oracle_no_gt";
  const size_t segment_count = 2u;
  for (size_t ordinal = 0; ordinal < segment_count; ++ordinal) {
    uifgo::SupportSegment segment;
    segment.segment_id = "s" + std::to_string(ordinal);
    segment.segment_ordinal = ordinal;
    segment.tag_id = 7;
    segment.anchor_id = static_cast<int>(ordinal + 1);
    segment.start_time = separated_groups ? 2.0 * ordinal : 0.0;
    segment.end_time = separated_groups ? 2.0 * ordinal + 1.0 : 1.0;
    segment.duration = 1.0;
    fixture.support.segments.push_back(segment);
  }

  const std::vector<gtsam::Pose3> truth = {
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.08, -0.04, 0.15),
                   gtsam::Point3(1.0, 1.5, 0.6)),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.10, -0.03, 0.20),
                   gtsam::Point3(1.4, 1.8, 0.7)),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.12, -0.02, 0.24),
                   gtsam::Point3(1.8, 2.0, 0.8)),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.14, -0.01, 0.28),
                   gtsam::Point3(2.1, 2.2, 0.9)),
  };
  const size_t pose_count = separated_groups ? 4u : 2u;
  for (size_t k = 0; k < pose_count; ++k)
    fixture.plan.keyframes.push_back(
        {k, k, static_cast<double>(k), 7});
  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
  auto vector_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  std::uint64_t obs_id = 8000;
  const gtsam::Vector3 zero_velocity = gtsam::Vector3::Zero();
  for (size_t k = 0; k < pose_count; ++k) {
    fixture.graph.addPrior(X(k), truth[k], pose_noise);
    fixture.graph.addPrior(V(k), zero_velocity, vector_noise);
    fixture.graph.addPrior(B(k), gtsam::imuBias::ConstantBias(), bias_noise);
    fixture.values.insert(
        X(k), truth[k].retract((gtsam::Vector(6)
                                   << 0.01, -0.01, 0.01,
                               0.03, -0.02, 0.02).finished()));
    fixture.values.insert(V(k), gtsam::Vector3(0.01, -0.01, 0.01));
    fixture.values.insert(B(k), gtsam::imuBias::ConstantBias());
    for (const auto& anchor : fixture.cfg.anchors) {
      const double geometric =
          (truth[k].transformFrom(fixture.cfg.lever_arm_init) - anchor.pos)
              .norm();
      const bool candidate_observation =
          (!separated_groups
              ? anchor.id <= 2
              : ((anchor.id == 1 && k < 2) ||
                 (anchor.id == 2 && k >= 2)));
      const double amplitude = candidate_observation && inject_candidate_bias
                                   ? (anchor.id == 1 ? 0.4 :
                                      (anchor.id == 2 ? 0.3 :
                                       (anchor.id == 3 ? 0.2 : 0.1)))
                                   : 0.0;
      const double raw = geometric + amplitude;
      const auto factor = uifgo::MakeUwbFactor(
          X(k), 0, 0, 0, anchor.pos, fixture.cfg.lever_arm_init, raw, 0.05,
          false, false, false, 0.0);
      const size_t factor_index = fixture.graph.size();
      fixture.graph.add(factor);
      fixture.metadata.push_back(
          {factor_index, obs_id, "uwb_range",
           std::vector<gtsam::Key>(factor->keys().begin(),
                                   factor->keys().end())});
      uifgo::ObservationRecord observation;
      observation.obs_id = obs_id;
      observation.sensor_time = static_cast<double>(k);
      observation.tag_id = 7;
      observation.anchor_id = anchor.id;
      observation.raw_range = raw;
      observation.valid = true;
      observation.planned = true;
      observation.keyframe_id = k;
      observation.nominal_sigma = 0.05;
      fixture.plan.observations.push_back(observation);
      if (candidate_observation)
        fixture.support.segments[anchor.id - 1].obs_ids.push_back(obs_id);
      ++obs_id;
    }
  }
  for (auto& segment : fixture.support.segments)
    segment.observation_count = segment.obs_ids.size();
  return fixture;
}

struct Stage2Fixture {
  Fixture input;
  uifgo::RefitOptions options;
  uifgo::SegmentRefitResult stage2;
  std::vector<uifgo::GroupRecoverabilityScore> scores;
};

Stage2Fixture MakeStage2(bool separated_groups = false) {
  Stage2Fixture output;
  output.input = MakeFixture(separated_groups);
  output.options.max_refit_iterations = 50;
  output.options.lm_relative_tolerance = 1e-10;
  output.options.lm_absolute_tolerance = 1e-12;
  output.stage2 = uifgo::SegmentRefitter(output.options).Run(
      output.input.graph, output.input.values, output.input.metadata,
      output.input.plan, output.input.cfg, output.input.support);
  EXPECT_TRUE(output.stage2.converged())
      << uifgo::SegmentRefitStatusName(output.stage2.status) << ": "
      << output.stage2.reason << " trace=" << output.stage2.iterations.size()
      << (output.stage2.iterations.empty()
              ? 0.0
              : output.stage2.iterations.back().max_kkt_violation)
      << "/"
      << (output.stage2.iterations.empty()
              ? 0.0
              : output.stage2.iterations.back()
                    .max_scaled_navigation_gradient_objective);
  if (output.stage2.converged()) {
    output.scores = uifgo::ScoreRefitRecoverability(
        output.stage2, output.input.support, output.input.plan,
        output.input.cfg);
  }
  return output;
}

uifgo::InferenceResult RunEngine(
    const Stage2Fixture& fixture, const uifgo::GateThresholds& gate,
    uifgo::InferenceTestHooks hooks = {},
    std::vector<uifgo::GroupRecoverabilityScore> scores = {}) {
  if (scores.empty()) scores = fixture.scores;
  return uifgo::FinalInferenceEngine(gate, fixture.options, {}, hooks).Run(
      fixture.input.graph, fixture.input.metadata, fixture.stage2,
      fixture.input.support, scores, fixture.input.plan, fixture.input.cfg,
      IdentityContext());
}

void MaybeWriteEvidence(const std::string& name,
                        const uifgo::InferenceResult& result) {
  const char* root_env = std::getenv("UIFGO_T08_EVIDENCE_OUTPUT_ROOT");
  if (!root_env || std::string(root_env).empty()) return;
  const boost::filesystem::path directory =
      boost::filesystem::path(root_env) / name;
  ASSERT_TRUE(boost::filesystem::create_directories(directory));
  uifgo::WriteInferenceArtifacts(directory.string(), result);
}

}  // namespace

TEST(T08Decision, EqualityOnEtaSAndGammaPassesAtomically) {
  auto score = SyntheticScore(0.4, 0.8, 1.2);
  const auto decisions = uifgo::FreezeGroupDecisions(
      {score}, DevelopmentGate(0.4, 0.8, 1.2));
  ASSERT_EQ(decisions.size(), 1u);
  EXPECT_EQ(decisions[0].decision, uifgo::GroupDecision::USE);
  EXPECT_TRUE(decisions[0].eta_pass);
  EXPECT_TRUE(decisions[0].s_pass);
  EXPECT_TRUE(decisions[0].gamma_pass);
  EXPECT_EQ(decisions[0].group.segment_ordinals,
            (std::vector<size_t>{0, 1}));
}

TEST(T08Decision, StableReasonsCoverThresholdInvalidRankShortAndBoundary) {
  auto eta = SyntheticScore(0.39, 0.8, 1.2);
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {eta}, DevelopmentGate(0.4, 0.8, 1.2))[0].reason_code,
            "SUPPRESS_ETA_BELOW_THRESHOLD");
  auto s = SyntheticScore(0.4, 0.81, 1.2);
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {s}, DevelopmentGate(0.4, 0.8, 1.2))[0].reason_code,
            "SUPPRESS_S_ABOVE_THRESHOLD");
  auto gamma = SyntheticScore(0.4, 0.8, 1.21);
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {gamma}, DevelopmentGate(0.4, 0.8, 1.2))[0].reason_code,
            "SUPPRESS_GAMMA_ABOVE_THRESHOLD");
  auto invalid = SyntheticScore(
      0.4, 0.8, 1.2, uifgo::RecoverabilityStatus::NUMERICAL_FAILURE);
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {invalid}, DevelopmentGate(0.4, 0.8, 1.2))[0].reason_code,
            "SUPPRESS_NUMERICAL_FAILURE");
  auto invalid_score = SyntheticScore(0.4, 0.8, 1.2);
  invalid_score.valid_score_exported = false;
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {invalid_score}, DevelopmentGate())[0].reason_code,
            "SUPPRESS_INVALID_SCORE");
  auto incomplete_group_fit = SyntheticScore(0.4, 0.8, 1.2);
  incomplete_group_fit.segment_fit.pop_back();
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {incomplete_group_fit}, DevelopmentGate())[0].reason_code,
            "SUPPRESS_INVALID_SCORE");
  auto rank = SyntheticScore(
      0.0, std::numeric_limits<double>::infinity(), 1.2,
      uifgo::RecoverabilityStatus::RANK_DEFICIENT);
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {rank}, DevelopmentGate(0.4, 0.8, 1.2))[0].reason_code,
            "SUPPRESS_RANK_FAILURE");
  auto short_score = SyntheticScore(0.4, 0.8, 1.2);
  short_score.eligible = short_score.valid_score_exported = false;
  short_score.segment_fit[1].short_support_debug = true;
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {short_score}, DevelopmentGate())[0].reason_code,
            "SUPPRESS_SHORT_SUPPORT");
  auto boundary = short_score;
  boundary.segment_fit[1].short_support_debug = false;
  boundary.segment_fit[0].boundary = true;
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {boundary}, DevelopmentGate())[0].reason_code,
            "SUPPRESS_BOUNDARY_OR_KKT_INVALID");
  auto insufficient = short_score;
  insufficient.segment_fit[1].short_support_debug = false;
  EXPECT_EQ(uifgo::FreezeGroupDecisions(
                {insufficient}, DevelopmentGate())[0].reason_code,
            "SUPPRESS_INSUFFICIENT_SUPPORT");
}

TEST(T08Decision, GateMustBeFiniteExplicitAndDevelopmentOnly) {
  auto score = SyntheticScore(0.4, 0.8, 1.2);
  auto gate = DevelopmentGate();
  gate.tau_eta = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(uifgo::FreezeGroupDecisions({score}, gate),
               std::invalid_argument);
  gate = DevelopmentGate();
  gate.parameter_provenance = "T10_LOCKED";
  EXPECT_THROW(uifgo::FreezeGroupDecisions({score}, gate),
               std::invalid_argument);
}

TEST(T10A19R03Decision, SuppressAllAndStructuredDebiasHaveExplicitSemantics) {
  auto eligible = SyntheticScore(0.4, 0.8, 1.2);
  auto ineligible = eligible;
  ineligible.group.group_id = "group_ineligible";
  ineligible.group.segment_ordinals = {2, 3};
  ineligible.eligible = false;
  ineligible.valid_score_exported = false;
  const auto suppressed = uifgo::FreezeGroupDecisions(
      {eligible, ineligible}, DevelopmentGate(),
      uifgo::FinalGatePolicy::SUPPRESS_ALL);
  ASSERT_EQ(suppressed.size(), 2u);
  EXPECT_EQ(suppressed[0].decision, uifgo::GroupDecision::SUPPRESS);
  EXPECT_EQ(suppressed[1].decision, uifgo::GroupDecision::SUPPRESS);

  const auto structured = uifgo::FreezeGroupDecisions(
      {eligible, ineligible}, DevelopmentGate(),
      uifgo::FinalGatePolicy::STRUCTURED_DEBIAS);
  ASSERT_EQ(structured.size(), 2u);
  EXPECT_EQ(structured[0].decision, uifgo::GroupDecision::USE);
  EXPECT_EQ(structured[1].decision, uifgo::GroupDecision::USE);
  EXPECT_EQ(structured[1].reason_code, "USE_STRUCTURED_DEBIAS_NO_GATE");
}

TEST(T08Inference, AtomicAcceptedGroupHasLiveCAndExactRawFactorMasks) {
  const auto fixture = MakeStage2();
  ASSERT_EQ(fixture.scores.size(), 1u);
  const auto result = RunEngine(fixture, DevelopmentGate());
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  EXPECT_EQ(result.status, uifgo::InferenceStatus::OK);
  ASSERT_EQ(result.decisions.size(), 1u);
  EXPECT_EQ(result.decisions[0].decision, uifgo::GroupDecision::USE);
  EXPECT_TRUE(result.final_values.exists(C(0)));
  EXPECT_TRUE(result.final_values.exists(C(1)));
  EXPECT_TRUE(result.factor_audit.ok);
  EXPECT_EQ(result.factor_audit.accepted_candidate_count, 4u);
  EXPECT_EQ(result.factor_audit.suppressed_candidate_count, 0u);
  EXPECT_EQ(result.factor_audit.noncandidate_reference_count, 4u);
  EXPECT_EQ(result.factor_audit.corrected_pseudo_range_count, 0u);
  for (const auto& row : result.factor_audit.observations)
    EXPECT_TRUE(row.ok) << row.obs_id;
  std::set<bool> candidate_final_use;
  for (const auto& mask : result.frozen_masks)
    if (mask.candidate) candidate_final_use.insert(mask.final_use);
  EXPECT_EQ(candidate_final_use, (std::set<bool>{true}));
  ASSERT_EQ(result.final_scores.size(), 1u);
  EXPECT_TRUE(result.final_scores[0].evaluated);
  EXPECT_NE(result.final_scores[0].linearization_id,
            result.decisions[0].decision_linearization_id);
}

TEST(T08Inference,
     FrozenPolicySeparatesAcceptedSuppressedAndReferenceInOneFinalGraph) {
  const auto fixture = MakeStage2();
  const auto mixed = uifgo::SegmentRefitter(fixture.options)
                         .RunFrozenCandidatePolicy(
                             fixture.input.graph, fixture.stage2.values,
                             fixture.input.metadata, fixture.input.plan,
                             fixture.input.cfg, fixture.input.support, {0});
  ASSERT_TRUE(mixed.converged()) << mixed.reason;
  EXPECT_TRUE(mixed.values.exists(C(0)));
  EXPECT_FALSE(mixed.values.exists(C(1)));
  std::set<std::uint64_t> accepted(
      fixture.input.support.segments[0].obs_ids.begin(),
      fixture.input.support.segments[0].obs_ids.end());
  std::set<std::uint64_t> suppressed(
      fixture.input.support.segments[1].obs_ids.begin(),
      fixture.input.support.segments[1].obs_ids.end());
  std::map<std::uint64_t, size_t> final_counts;
  for (const auto& meta : mixed.factor_metadata) {
    if (meta.obs_id != 0) ++final_counts[meta.obs_id];
    EXPECT_EQ(meta.factor_type.find("corrected"), std::string::npos);
    EXPECT_EQ(meta.factor_type.find("pseudo"), std::string::npos);
  }
  for (const auto& observation : fixture.input.plan.observations) {
    const size_t expected = suppressed.count(observation.obs_id) ? 0u : 1u;
    EXPECT_EQ(final_counts[observation.obs_id], expected)
        << observation.obs_id;
    if (accepted.count(observation.obs_id)) {
      const auto found = std::find_if(
          mixed.factor_metadata.begin(), mixed.factor_metadata.end(),
          [&](const auto& meta) { return meta.obs_id == observation.obs_id; });
      ASSERT_NE(found, mixed.factor_metadata.end());
      EXPECT_EQ(found->factor_type, "uwb_segment_range");
      EXPECT_NE(std::find(found->keys.begin(), found->keys.end(), C(0)),
                found->keys.end());
    }
  }
}

TEST(T08Inference,
     TwoIndependentGroupsUseAndSuppressRemainAtomicThroughFullEngine) {
  const auto fixture = MakeStage2(true);
  ASSERT_EQ(fixture.scores.size(), 2u);
  auto scores = fixture.scores;
  ASSERT_GT(scores[0].numerical.eta, 0.0);
  scores[1].numerical.eta = 0.0;
  const auto result = RunEngine(
      fixture, DevelopmentGate(scores[0].numerical.eta * 0.5, 1e6, 1e6),
      {}, scores);
  MaybeWriteEvidence("two_groups_use_suppress", result);
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  ASSERT_EQ(result.decisions.size(), 2u);
  EXPECT_EQ(result.decisions[0].decision, uifgo::GroupDecision::USE);
  EXPECT_EQ(result.decisions[1].decision, uifgo::GroupDecision::SUPPRESS);
  EXPECT_TRUE(result.final_values.exists(C(0)));
  EXPECT_FALSE(result.final_values.exists(C(1)));
  EXPECT_EQ(result.factor_audit.accepted_candidate_count, 2u);
  EXPECT_EQ(result.factor_audit.suppressed_candidate_count, 2u);
  EXPECT_EQ(result.factor_audit.noncandidate_reference_count, 12u);
  EXPECT_EQ(result.factor_audit.corrected_pseudo_range_count, 0u);
  ASSERT_EQ(result.final_scores.size(), 2u);
  EXPECT_TRUE(result.final_scores[0].evaluated);
  EXPECT_FALSE(result.final_scores[1].evaluated);
  EXPECT_EQ(result.final_scores[1].status,
            "FROZEN_SUPPRESSED_NO_FINAL_C_KEY_NO_READMISSION");
  for (const auto& mask : result.frozen_masks) {
    if (mask.segment_id == "s1") {
      EXPECT_FALSE(mask.final_use);
    }
  }
}

TEST(T08Inference, ZeroEligibleAndZeroAcceptedHaveExplicitStatusAndFrozenMasks) {
  const auto fixture = MakeStage2();
  auto ineligible = fixture.scores;
  ineligible[0].eligible = false;
  ineligible[0].valid_score_exported = false;
  ineligible[0].segment_fit[0].short_support_debug = true;
  const auto no_eligible = RunEngine(
      fixture, DevelopmentGate(), {}, ineligible);
  ASSERT_TRUE(no_eligible.valid_estimate()) << no_eligible.reason;
  EXPECT_EQ(no_eligible.status,
            uifgo::InferenceStatus::NO_ELIGIBLE_CANDIDATES);
  EXPECT_FALSE(no_eligible.final_values.exists(C(0)));
  EXPECT_FALSE(no_eligible.final_values.exists(C(1)));
  EXPECT_EQ(no_eligible.factor_audit.suppressed_candidate_count, 4u);

  auto rejected = fixture.scores;
  rejected[0].numerical.eta = 0.0;
  const auto zero_accepted = RunEngine(
      fixture, DevelopmentGate(0.5, 1e6, 1e6), {}, rejected);
  ASSERT_TRUE(zero_accepted.valid_estimate()) << zero_accepted.reason;
  EXPECT_EQ(zero_accepted.status, uifgo::InferenceStatus::ZERO_ACCEPTED);
  ASSERT_EQ(zero_accepted.final_scores.size(), 1u);
  EXPECT_FALSE(zero_accepted.final_scores[0].evaluated);
  EXPECT_EQ(zero_accepted.final_scores[0].status,
            "FROZEN_SUPPRESSED_NO_FINAL_C_KEY_NO_READMISSION");
  EXPECT_FALSE(zero_accepted.final_values.exists(C(0)));
  EXPECT_FALSE(zero_accepted.final_values.exists(C(1)));
  for (const auto& mask : zero_accepted.frozen_masks) {
    if (mask.candidate) {
      EXPECT_FALSE(mask.final_use);
    }
  }
}

TEST(T08Inference, NoCandidatesUsesReferenceGraphWithoutFallback) {
  Stage2Fixture fixture;
  fixture.input = MakeFixture(false, false);
  fixture.input.support.segments.clear();
  fixture.options.max_refit_iterations = 50;
  fixture.options.lm_relative_tolerance = 1e-10;
  fixture.options.lm_absolute_tolerance = 1e-12;
  fixture.stage2 = uifgo::SegmentRefitter(fixture.options).Run(
      fixture.input.graph, fixture.input.values, fixture.input.metadata,
      fixture.input.plan, fixture.input.cfg, fixture.input.support);
  ASSERT_TRUE(fixture.stage2.converged()) << fixture.stage2.reason;
  const auto result = RunEngine(fixture, DevelopmentGate());
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  EXPECT_EQ(result.status, uifgo::InferenceStatus::NO_CANDIDATES);
  EXPECT_FALSE(result.fallback.attempted);
  EXPECT_TRUE(result.decisions.empty());
  EXPECT_TRUE(result.final_scores.empty());
  EXPECT_EQ(result.factor_audit.noncandidate_reference_count, 8u);
  MaybeWriteEvidence("no_candidates", result);
}

TEST(T08Inference, ForcedRecoveryFailureFallsBackExactlyOnceFromReference) {
  const auto fixture = MakeStage2();
  uifgo::InferenceTestHooks hooks;
  hooks.force_recovery_failure = true;
  const auto result = RunEngine(fixture, DevelopmentGate(), hooks);
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  EXPECT_EQ(result.status, uifgo::InferenceStatus::FALLBACK_OK);
  EXPECT_TRUE(result.fallback.attempted);
  EXPECT_EQ(result.fallback.attempt_count, 1u);
  EXPECT_EQ(result.fallback.status, "SUCCESS");
  EXPECT_EQ(result.fallback.recovery_failure_reason,
            "TEST_ONLY_FORCED_RECOVERY_FAILURE");
  EXPECT_EQ(result.recovery_attempt.execution_status,
            "INVALIDATED_TEST_ONLY");
  EXPECT_EQ(result.fallback_refit_attempt.execution_status, "SUCCEEDED");
  EXPECT_EQ(result.final_result_refit.execution_status,
            "FINAL_RESULT_FROM_FALLBACK");
  ASSERT_EQ(result.recovery_final_scores.size(), result.decisions.size());
  ASSERT_EQ(result.final_scores.size(), result.decisions.size());
  EXPECT_FALSE(result.final_scores[0].evaluated);
  EXPECT_EQ(
      result.final_scores[0].status,
      "FALLBACK_ALL_CANDIDATES_SUPPRESSED_NO_FINAL_C_KEY_NO_READMISSION");
  EXPECT_FALSE(result.final_values.exists(C(0)));
  EXPECT_FALSE(result.final_values.exists(C(1)));
  EXPECT_EQ(result.factor_audit.suppressed_candidate_count, 4u);
  for (const auto& mask : result.frozen_masks) {
    if (mask.candidate) {
      EXPECT_FALSE(mask.final_use);
    }
  }
}

TEST(T08Inference,
     ActualFinalAcceptanceFailurePreservesRecoveryScoresAndFallsBackOnce) {
  const auto fixture = MakeStage2();
  const auto probe = RunEngine(fixture, DevelopmentGate());
  ASSERT_TRUE(probe.valid_estimate()) << probe.reason;
  ASSERT_EQ(probe.final_scores.size(), 1u);
  ASSERT_TRUE(probe.final_scores[0].evaluated);
  ASSERT_FALSE(probe.final_scores[0].score.segment_fit.empty());
  double final_gamma = 0.0;
  for (const auto& fit : probe.final_scores[0].score.segment_fit)
    final_gamma = std::max(final_gamma, fit.gamma);
  ASSERT_GT(final_gamma, 0.0);

  auto decision_scores = fixture.scores;
  for (auto& fit : decision_scores[0].segment_fit) fit.gamma = 0.0;
  const auto result = RunEngine(
      fixture, DevelopmentGate(0.0, 1e6, final_gamma * 0.5), {},
      decision_scores);
  MaybeWriteEvidence("actual_reaudit_failure_fallback_success", result);
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  EXPECT_EQ(result.status, uifgo::InferenceStatus::FALLBACK_OK);
  EXPECT_EQ(result.fallback.attempt_count, 1u);
  EXPECT_NE(result.fallback.recovery_failure_reason.find(
                "FINAL_ACCEPTANCE_REAUDIT_FAILED_"), std::string::npos);
  ASSERT_EQ(result.recovery_final_scores.size(), 1u);
  EXPECT_TRUE(result.recovery_final_scores[0].evaluated);
  EXPECT_FALSE(result.recovery_final_scores[0].linearization_id.empty());
  ASSERT_FALSE(result.recovery_final_scores[0].score.segment_fit.empty());
  EXPECT_GT(result.recovery_final_scores[0].score.segment_fit[0].gamma,
            final_gamma * 0.5);
  ASSERT_EQ(result.final_scores.size(), 1u);
  EXPECT_FALSE(result.final_scores[0].evaluated);
  EXPECT_EQ(result.final_scores[0].linearization_id, "NOT_APPLICABLE");
  EXPECT_EQ(result.final_result_refit.execution_status,
            "FINAL_RESULT_FROM_FALLBACK");
}

TEST(T08Inference,
     ActualFinalAcceptanceFailureThenForcedFallbackFailureRetainsBoth) {
  const auto fixture = MakeStage2();
  const auto probe = RunEngine(fixture, DevelopmentGate());
  ASSERT_TRUE(probe.valid_estimate()) << probe.reason;
  ASSERT_TRUE(probe.final_scores[0].evaluated);
  double final_gamma = 0.0;
  for (const auto& fit : probe.final_scores[0].score.segment_fit)
    final_gamma = std::max(final_gamma, fit.gamma);
  ASSERT_GT(final_gamma, 0.0);
  auto decision_scores = fixture.scores;
  for (auto& fit : decision_scores[0].segment_fit) fit.gamma = 0.0;
  uifgo::InferenceTestHooks hooks;
  hooks.force_fallback_failure = true;
  const auto result = RunEngine(
      fixture, DevelopmentGate(0.0, 1e6, final_gamma * 0.5), hooks,
      decision_scores);
  MaybeWriteEvidence("actual_reaudit_failure_fallback_failure", result);
  EXPECT_EQ(result.status, uifgo::InferenceStatus::ESTIMATION_FAILED);
  EXPECT_FALSE(result.valid_estimate());
  EXPECT_EQ(result.fallback.attempt_count, 1u);
  EXPECT_EQ(result.fallback.status, "FAILED");
  EXPECT_NE(result.fallback.recovery_failure_reason.find(
                "FINAL_ACCEPTANCE_REAUDIT_FAILED_"), std::string::npos);
  EXPECT_EQ(result.fallback.fallback_failure_reason,
            "TEST_ONLY_FORCED_FALLBACK_FAILURE");
  ASSERT_EQ(result.recovery_final_scores.size(), 1u);
  EXPECT_TRUE(result.recovery_final_scores[0].evaluated);
  EXPECT_TRUE(result.final_graph.empty());
  EXPECT_TRUE(result.final_values.empty());
}

TEST(T08Inference,
     AutomaticEngineeringFixtureReachesInferenceWithoutOracleOrGtInput) {
  auto fixture = MakeFixture();
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
  uifgo::DiscoveryContext context;
  context.input_plan_hash = "sha256:" + uifgo::Sha256Hex("t08-auto-plan");
  context.source_hash = "sha256:" + uifgo::Sha256Hex("t08-auto-input");
  context.config_hash = "sha256:" + uifgo::Sha256Hex("t08-auto-config");
  context.calibration_hash =
      "sha256:" + uifgo::Sha256Hex("t08-explicit-synthetic-calibration");
  context.solver_config_hash =
      "sha256:" + uifgo::Sha256Hex("t08-engineering-solver");

  // AutomaticSupportProvider::Run deliberately exposes no oracle path, GT,
  // labels, or reference trajectory argument (U11/T08 engineering fixture).
  const auto discovery =
      uifgo::AutomaticSupportProvider(discovery_options)
          .Run(fixture.graph, fixture.values, fixture.metadata, fixture.plan,
               fixture.cfg, context);
  ASSERT_TRUE(discovery.converged()) << discovery.reason;
  ASSERT_EQ(discovery.partition.provider, "automatic_discovery");
  ASSERT_TRUE(discovery.partition.source_path.empty());
  ASSERT_FALSE(discovery.partition.segments.empty());

  uifgo::RefitOptions refit_options;
  refit_options.max_refit_iterations = 50;
  refit_options.lm_relative_tolerance = 1e-10;
  refit_options.lm_absolute_tolerance = 1e-12;
  const auto stage2 = uifgo::SegmentRefitter(refit_options).Run(
      fixture.graph, discovery.navigation_values, fixture.metadata,
      fixture.plan, fixture.cfg, discovery.partition);
  ASSERT_TRUE(stage2.converged()) << stage2.reason;
  const auto scores = uifgo::ScoreRefitRecoverability(
      stage2, discovery.partition, fixture.plan, fixture.cfg);
  ASSERT_FALSE(scores.empty());
  auto final_context = IdentityContext("automatic");
  const auto mismatch =
      uifgo::FinalInferenceEngine(DevelopmentGate(), refit_options).Run(
          fixture.graph, fixture.metadata, stage2, discovery.partition,
          scores, fixture.plan, fixture.cfg, final_context);
  EXPECT_EQ(mismatch.reason, "FINAL_SUPPORT_CALIBRATION_IMU_CONTEXT_MISMATCH");
  EXPECT_TRUE(mismatch.decisions.empty());
  // A14 requires the actual producer calibration/model context at consumption.
  final_context.calibration_sha256 = context.calibration_hash;
  const auto result =
      uifgo::FinalInferenceEngine(DevelopmentGate(), refit_options).Run(
          fixture.graph, fixture.metadata, stage2, discovery.partition,
          scores, fixture.plan, fixture.cfg, final_context);
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  EXPECT_EQ(result.frozen_masks.size(), fixture.plan.observations.size());
}

TEST(T08Inference, ForcedFallbackFailureReturnsNoValidGraphOrTrajectoryState) {
  const auto fixture = MakeStage2();
  uifgo::InferenceTestHooks hooks;
  hooks.force_recovery_failure = true;
  hooks.force_fallback_failure = true;
  const auto result = RunEngine(fixture, DevelopmentGate(), hooks);
  EXPECT_EQ(result.status, uifgo::InferenceStatus::ESTIMATION_FAILED);
  EXPECT_FALSE(result.valid_estimate());
  EXPECT_TRUE(result.final_graph.empty());
  EXPECT_TRUE(result.final_values.empty());
  EXPECT_EQ(result.fallback.attempt_count, 1u);
  EXPECT_EQ(result.fallback.status, "FAILED");
  ASSERT_EQ(result.final_scores.size(), result.decisions.size());
  EXPECT_FALSE(result.final_scores[0].evaluated);
  EXPECT_EQ(result.final_scores[0].status,
            "ESTIMATION_FAILED_NO_FINAL_LINEARIZATION");
}

TEST(T08Inference,
     DevelopmentSuppressRecoveryAndFallbackUseSeparateNoCRequests) {
  auto fixture = MakeStage2();
  const std::string identity =
      "a19-policy-sha256:r07-empty-certified-engineering";
  fixture.input.support.partition_hash =
      "sha256:r07-empty-certified-support";
  fixture.input.support.solver_config_hash = identity;
  size_t recovery_callbacks = 0;
  size_t fallback_callbacks = 0;
  auto make_request = [&](size_t* callbacks) {
    uifgo::DevelopmentStage2Request request;
    request.policy = "PAPER_CERTIFIED_PAIR_REDUCTION_V1";
    request.role = "development";
    request.implementation_identity = identity;
    request.allow_inexact_handoff = false;
    request.conditional_navigation =
        [&, callbacks](
            size_t, const gtsam::NonlinearFactorGraph& graph,
            const gtsam::Values& values,
            const uifgo::CheckedLmOptions& options,
            const std::vector<uifgo::DevelopmentRefitRangeConstant>&
                constants) {
          ++*callbacks;
          EXPECT_EQ(options.policy, uifgo::ConditionalLmPolicy::
              GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2);
          EXPECT_EQ(graph.size(), 10u);
          EXPECT_EQ(values.size(), 6u);
          EXPECT_EQ(constants.size(), 4u);
          for (const auto& item : constants) {
            EXPECT_FALSE(item.candidate);
            EXPECT_DOUBLE_EQ(item.segment_amplitude, 0.0);
            EXPECT_DOUBLE_EQ(item.conditional_beta, item.fixed_beta);
            EXPECT_TRUE(std::isfinite(item.expected_unwhitened_residual));
            EXPECT_LT(item.factor_index, graph.size());
          }
          return uifgo::RunCheckedConditionalLm(graph, values, options);
        };
    return request;
  };
  auto recovery_request = make_request(&recovery_callbacks);
  auto fallback_request = make_request(&fallback_callbacks);
  uifgo::InferenceTestHooks hooks;
  hooks.force_recovery_failure = true;
  auto context = IdentityContext("r07-empty-certified");
  context.support_partition_sha256 = fixture.input.support.partition_hash;
  context.solver_config_sha256 = identity;
  const auto result = uifgo::FinalInferenceEngine(
      DevelopmentGate(), fixture.options, {}, hooks,
      uifgo::FinalGatePolicy::SUPPRESS_ALL, nullptr, &recovery_request,
      &fallback_request)
      .Run(fixture.input.graph, fixture.input.metadata, fixture.stage2,
           fixture.input.support, fixture.scores, fixture.input.plan,
           fixture.input.cfg, context);
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  EXPECT_EQ(result.status, uifgo::InferenceStatus::FALLBACK_OK);
  EXPECT_GT(recovery_callbacks, 0u);
  EXPECT_GT(fallback_callbacks, 0u);
  EXPECT_EQ(result.fallback.attempt_count, 1u);
  EXPECT_EQ(result.fallback.status, "SUCCESS");
  EXPECT_EQ(result.final_values.size(), 6u);
  for (gtsam::Key key : result.final_values.keys())
    EXPECT_NE(gtsam::Symbol(key).chr(), 'c');

  auto forbidden = recovery_request;
  forbidden.policy =
      "PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1";
  forbidden.allow_inexact_handoff = true;
  const auto rejected = uifgo::SegmentRefitter(fixture.options)
      .RunDevelopmentFrozenCandidatePolicy(
          fixture.input.graph, fixture.stage2.values, fixture.input.metadata,
          fixture.input.plan, fixture.input.cfg, fixture.input.support, {},
          &forbidden);
  EXPECT_EQ(rejected.status, uifgo::SegmentRefitStatus::INVALID_INPUT);
  EXPECT_EQ(rejected.reason, "DEVELOPMENT_NO_C_FORBIDS_INEXACT_HANDOFF");
}

TEST(T08Inference, CovarianceAvailableAndUnavailableAreExplicit) {
  const auto fixture = MakeStage2();
  const auto available = RunEngine(fixture, DevelopmentGate());
  ASSERT_TRUE(available.valid_estimate()) << available.reason;
  ASSERT_EQ(available.final_graph_covariance.status,
            uifgo::CovarianceStatus::AVAILABLE)
      << available.final_graph_covariance.reason;
  ASSERT_FALSE(available.final_graph_covariance.blocks.empty());
  std::set<gtsam::Key> covariance_keys;
  for (const auto& block : available.final_graph_covariance.blocks) {
    covariance_keys.insert(block.key);
    EXPECT_TRUE(block.covariance.allFinite());
    EXPECT_FALSE(block.coordinates.empty());
  }
  const auto value_keys_vector = available.final_values.keys();
  EXPECT_EQ(covariance_keys,
            (std::set<gtsam::Key>(value_keys_vector.begin(),
                                  value_keys_vector.end())));

  uifgo::InferenceTestHooks hooks;
  hooks.force_covariance_unavailable = true;
  const auto unavailable = RunEngine(fixture, DevelopmentGate(), hooks);
  ASSERT_TRUE(unavailable.valid_estimate()) << unavailable.reason;
  EXPECT_EQ(unavailable.final_graph_covariance.status,
            uifgo::CovarianceStatus::UNAVAILABLE);
  EXPECT_TRUE(unavailable.final_graph_covariance.blocks.empty());
  EXPECT_EQ(unavailable.final_graph_covariance.reason,
            "TEST_ONLY_FORCED_COVARIANCE_UNAVAILABLE");
  MaybeWriteEvidence("covariance_unavailable", unavailable);
}

TEST(T08Inference, FinalExportsHaveOneInferenceIdentityAndNoLegacyPolicyState) {
  const auto fixture = MakeStage2();
  const auto result = RunEngine(fixture, DevelopmentGate());
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  EXPECT_EQ(result.inference_id.rfind("t08inference-sha256:", 0), 0u);
  EXPECT_EQ(result.final_factor_metadata.size(), result.final_graph.size());
  EXPECT_EQ(result.final_graph_covariance.source,
            "FULL_FINAL_GRAPH_MARGINAL_COVARIANCE");
  std::string identity_reason;
  EXPECT_TRUE(uifgo::VerifyInferenceContentIdentity(result, &identity_reason))
      << identity_reason;
  const auto repeated = RunEngine(fixture, DevelopmentGate());
  EXPECT_EQ(result.inference_id, repeated.inference_id);
  EXPECT_EQ(result.content_identity.graph_linearization_sha256,
            repeated.content_identity.graph_linearization_sha256);
  EXPECT_EQ(result.content_identity.values_sha256,
            repeated.content_identity.values_sha256);
  for (const auto& meta : result.final_factor_metadata) {
    EXPECT_NE(meta.factor_type, "corrected_pseudo_range");
    EXPECT_NE(meta.factor_type, "legacy_gnc_range");
  }
}

TEST(T08Identity,
     DistinguishesValuesAndFactorNumbersWithEqualStructureAndTotalError) {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  gtsam::NonlinearFactorGraph graph_zero;
  graph_zero.addPrior<double>(C(0), 0.0, noise);
  gtsam::Values plus, minus;
  plus.insert<double>(C(0), 1.0);
  minus.insert<double>(C(0), -1.0);
  ASSERT_DOUBLE_EQ(graph_zero.error(plus), graph_zero.error(minus));
  const auto plus_id = uifgo::ComputeInferenceContentIdentity(
      graph_zero, plus, IdentityContext("values-equal-error"));
  const auto minus_id = uifgo::ComputeInferenceContentIdentity(
      graph_zero, minus, IdentityContext("values-equal-error"));
  EXPECT_NE(plus_id.values_sha256, minus_id.values_sha256);
  EXPECT_NE(plus_id.graph_linearization_sha256,
            minus_id.graph_linearization_sha256);

  gtsam::NonlinearFactorGraph graph_plus_measurement, graph_minus_measurement;
  graph_plus_measurement.addPrior<double>(C(0), 1.0, noise);
  graph_minus_measurement.addPrior<double>(C(0), -1.0, noise);
  gtsam::Values zero;
  zero.insert<double>(C(0), 0.0);
  ASSERT_DOUBLE_EQ(graph_plus_measurement.error(zero),
                   graph_minus_measurement.error(zero));
  const auto factor_plus = uifgo::ComputeInferenceContentIdentity(
      graph_plus_measurement, zero, IdentityContext("factor-equal-error"));
  const auto factor_minus = uifgo::ComputeInferenceContentIdentity(
      graph_minus_measurement, zero, IdentityContext("factor-equal-error"));
  EXPECT_EQ(factor_plus.values_sha256, factor_minus.values_sha256);
  EXPECT_NE(factor_plus.graph_linearization_sha256,
            factor_minus.graph_linearization_sha256);
}

TEST(T08Identity, ArtifactExportRejectsValuesChangedAfterIdentitySeal) {
  const auto fixture = MakeStage2();
  auto result = RunEngine(fixture, DevelopmentGate());
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  result.final_values.update<double>(
      C(0), result.final_values.at<double>(C(0)) + 0.01);
  std::string identity_reason;
  EXPECT_FALSE(uifgo::VerifyInferenceContentIdentity(result, &identity_reason));
  EXPECT_EQ(identity_reason,
            "FINAL_GRAPH_VALUES_CONTEXT_FINGERPRINT_MISMATCH");
  const auto root = boost::filesystem::temp_directory_path() /
                    boost::filesystem::unique_path("uifgo-t08-id-%%%%-%%%%");
  ASSERT_TRUE(boost::filesystem::create_directory(root));
  EXPECT_THROW(uifgo::WriteInferenceArtifacts(root.string(), result),
               std::invalid_argument);
  boost::filesystem::remove_all(root);
}

TEST(T08Inference, ArtifactWriterAcceptsOnlyOneInferenceResultIdentity) {
  const auto fixture = MakeStage2();
  auto result = RunEngine(fixture, DevelopmentGate());
  ASSERT_TRUE(result.valid_estimate()) << result.reason;
  result.decisions[0].reason_code = "quoted,\"reason\"\nline";
  const auto root = boost::filesystem::temp_directory_path() /
                    boost::filesystem::unique_path("uifgo-t08-%%%%-%%%%");
  ASSERT_TRUE(boost::filesystem::create_directory(root));
  const auto written =
      uifgo::WriteInferenceArtifacts(root.string(), result);
  EXPECT_GE(written.files.size(), 23u);
  for (const std::string& required : {
           "decisions.csv", "scores_decision.csv", "scores_final.csv",
           "scores_decision_segments.csv",
           "scores_recovery_attempt.csv",
           "scores_recovery_attempt_segments.csv",
           "scores_final_segments.csv", "recovery_refit_iterations.csv",
           "fallback_refit_iterations.csv", "final_refit_iterations.csv",
           "final_inference_summary.json", "fallback_attempt.json",
           "final_content_identity.json",
           "final_factor_audit.csv", "covariance_status.json",
           "covariance.csv", "trajectory.tum", "imu_bias.csv",
           "static_bias.csv", "segment_bias.csv", "residuals.csv",
           "final_factor_metadata.csv", "final_masks.csv"}) {
    EXPECT_TRUE(boost::filesystem::is_regular_file(root / required))
        << required;
  }
  for (const std::string& name : {"decisions.csv", "scores_decision.csv",
                                  "scores_decision_segments.csv",
                                  "scores_recovery_attempt.csv",
                                  "scores_recovery_attempt_segments.csv",
                                  "scores_final.csv", "scores_final_segments.csv",
                                  "final_masks.csv",
                                  "final_factor_audit.csv", "covariance.csv",
                                  "imu_bias.csv", "segment_bias.csv",
                                  "residuals.csv",
                                  "final_factor_metadata.csv"}) {
    std::ifstream input((root / name).string(), std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>());
    EXPECT_NE(bytes.find(result.inference_id), std::string::npos) << name;
  }
  std::ifstream decisions((root / "decisions.csv").string(), std::ios::binary);
  const std::string decision_bytes(
      (std::istreambuf_iterator<char>(decisions)),
      std::istreambuf_iterator<char>());
  EXPECT_NE(decision_bytes.find("\"quoted,\"\"reason\"\"\nline\""),
            std::string::npos);
  EXPECT_THROW(uifgo::WriteInferenceArtifacts(root.string(), result),
               std::runtime_error);
  boost::filesystem::remove_all(root);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
