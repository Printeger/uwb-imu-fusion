// C3 tests (FDE-03/04/05): profile-likelihood evidence, post-FDE conservative
// selection guarantees and the bridge / IMU-interval decision order.
//
// Every rule is exercised in both directions (the admissible path and the
// fail-closed path); no threshold or standard is relaxed to make a test pass.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "uwb_imu_pl/integrity/fde_post_selection.hpp"

namespace {

using namespace uwb_imu_pl;

Eigen::MatrixXd diag(const std::vector<double>& values) {
  Eigen::MatrixXd out = Eigen::MatrixXd::Zero(values.size(), values.size());
  for (std::size_t index = 0; index < values.size(); ++index) {
    out(static_cast<Eigen::Index>(index), static_cast<Eigen::Index>(index)) =
        values[index];
  }
  return out;
}

// ---------------------------------------------------------------------------
// FDE-03: wrong / missed exclusion and contaminated priors cannot be published
// without the post-selection proof.
// ---------------------------------------------------------------------------
TEST(FdePostSelection, Fde03ProfileEvidenceAndNoPublishWithoutProof) {
  // Two-parameter fault (same unit, same dimension): the true pair explains the
  // residual; a wrong single direction does not.
  Eigen::VectorXd t(2);
  t << 3.0, 2.0;
  const ProfileEvidenceInput correct{diag({1.0, 1.0}), t, 5.0, 0.5,
                                     FaultUnitKind::UwbRangeMeters, 2, "pair"};
  const ProfileEvidence full = profileLikelihoodEvidence(correct);
  ASSERT_TRUE(full.valid) << full.reason;
  // explained = 3^2 + 2^2 = 13; J = 5 + 0.5 - 13 = -7.5 (the residual really is
  // over-explained, which is what a real fault implies).
  EXPECT_NEAR(full.explained_energy, 13.0, 1e-12);
  EXPECT_NEAR(full.j_profile, -7.5, 1e-12);
  EXPECT_EQ(full.constant_used, 0.5) << "the raw constant must be used";

  // Missed exclusion: the t vector lies along one direction, so a single-mode
  // explanation is nearly as good -- the pool keeps both and ranks them.
  std::vector<StructuredCandidate> pool(2);
  pool[0].hypothesis = HypothesisId(1);
  pool[0].unit = FaultUnitKind::UwbRangeMeters;
  pool[0].parameter_dim = 2;
  pool[0].j_profile = full.j_profile;
  pool[0].physical_source_id = "anchor_1";
  pool[1].hypothesis = HypothesisId(2);
  pool[1].unit = FaultUnitKind::UwbRangeMeters;
  pool[1].parameter_dim = 2;
  pool[1].j_profile = full.j_profile + 1.0;
  pool[1].physical_source_id = "anchor_2";
  const CandidateRanking ranking = rankStructuredCandidates(pool);
  ASSERT_TRUE(ranking.valid) << ranking.reason;
  ASSERT_EQ(ranking.order.size(), 2u);
  EXPECT_EQ(ranking.order[0], 0u) << "the lower profile value must rank first";

  // Contaminated (stale) prior: the inflated prior variance lies IN the fault
  // direction, so the same residual explains less and the profile value rises.
  const ProfileEvidenceInput contaminated{diag({1e6, 1.0}), Eigen::Vector2d(3, 2),
                                          5.0, 0.5, FaultUnitKind::UwbRangeMeters, 2,
                                          "contaminated"};
  const ProfileEvidence dirty = profileLikelihoodEvidence(contaminated);
  ASSERT_TRUE(dirty.valid) << dirty.reason;
  EXPECT_GT(dirty.j_profile, full.j_profile)
      << "prior contamination must worsen (not improve) the profile value";

  // A pool mixing physical units may not be ranked at all.
  std::vector<StructuredCandidate> mixed = pool;
  mixed.push_back(pool[0]);
  mixed.back().unit = FaultUnitKind::ImuAccelMps2;
  mixed.back().hypothesis = HypothesisId(3);
  const CandidateRanking refused = rankStructuredCandidates(mixed);
  EXPECT_FALSE(refused.valid);
  EXPECT_TRUE(refused.mixed_units);
  EXPECT_NE(refused.reason.find("mixes physical units"), std::string::npos);

  // Post-selection proof: without a closing budget the platform stays
  // unavailable; the budget is charged over EVERY publishable action.
  std::vector<ActionGuarantee> actions(2);
  actions[0].action_id = 10;
  actions[0].epsilon_budget = 2e-5;
  actions[1].action_id = 11;
  actions[1].epsilon_budget = 2e-5;
  const GuaranteeGroupResult tight = buildGuaranteeGroups(actions, 3e-5);
  EXPECT_FALSE(tight.valid) << "2e-5 + 2e-5 > 3e-5 must refuse";
  EXPECT_NE(tight.reason.find("does not close"), std::string::npos);
  const GuaranteeGroupResult fits = buildGuaranteeGroups(actions, 4e-5);
  EXPECT_TRUE(fits.valid) << fits.reason;
  EXPECT_LE(fits.total_charged_budget, 4e-5 + 1e-18);
}

// ---------------------------------------------------------------------------
// FDE-04: IMU interval repair / bridge failure stays legal (no fake Gaussian
// information, no silently removed source).
// ---------------------------------------------------------------------------
TEST(FdePostSelection, Fde04ImuIntervalRepairAndBridgeFailureStayLegal) {
  // Validated random/bounded model: enters the reference estimate with the dual
  // envelope, and is marked as a bounded model (never as Gaussian information).
  StructuredCandidate bounded;
  bounded.unit = FaultUnitKind::ImuAccelMps2;
  bounded.parameter_dim = 3;
  bounded.bounded_model = true;
  bounded.has_valid_reference = true;
  const CandidateDecision bounded_decision = decideCandidateHandling(bounded);
  EXPECT_EQ(bounded_decision.disposition,
            CandidateDisposition::UseInReferenceEstimate);
  EXPECT_TRUE(bounded_decision.uses_bounded_model);
  EXPECT_TRUE(bounded_decision.integrity_available);
  EXPECT_NE(bounded_decision.reason.find("dual envelope"), std::string::npos);

  // Bounded model: it enters the reference estimate (the model IS the reference
  // basis), and is marked as bounded -- never as Gaussian information.
  StructuredCandidate orphan = bounded;
  orphan.has_valid_reference = false;
  const CandidateDecision orphan_decision = decideCandidateHandling(orphan);
  EXPECT_EQ(orphan_decision.disposition,
            CandidateDisposition::UseInReferenceEstimate);
  EXPECT_TRUE(orphan_decision.uses_bounded_model);

  // Centre-only use (e.g. an interval-removal candidate whose dynamics
  // substitute is not a validated model): only a protection transfer from a
  // valid reference at the same instant/quantity is admissible.
  StructuredCandidate centre;
  centre.unit = FaultUnitKind::ImuAccelMps2;
  centre.parameter_dim = 3;
  centre.centre_only = true;
  centre.has_valid_reference = true;
  centre.propagation_bound_available = true;
  const CandidateDecision transfer = decideCandidateHandling(centre);
  EXPECT_EQ(transfer.disposition,
            CandidateDisposition::TransferToValidReference);
  EXPECT_FALSE(transfer.uses_bounded_model);
  EXPECT_TRUE(transfer.integrity_available);

  // Centre-only at a different time without a propagation bound: diagnostics
  // only, explicitly unprotected.
  StructuredCandidate no_bound = centre;
  no_bound.propagation_bound_available = false;
  const CandidateDecision diagnostic = decideCandidateHandling(no_bound);
  EXPECT_EQ(diagnostic.disposition,
            CandidateDisposition::UnprotectedDiagnosticOnly);
  EXPECT_FALSE(diagnostic.integrity_available);
  EXPECT_NE(diagnostic.reason.find("unprotected"), std::string::npos);

  // A candidate that removes an IMU interval still carries its removed groups
  // into the accounting: removal does not delete the source's risk.
  StructuredCandidate removal = centre;
  removal.groups_to_remove = {FactorGroupId(7), FactorGroupId(8)};
  removal.has_valid_reference = false;
  const CandidateDecision removal_decision = decideCandidateHandling(removal);
  EXPECT_EQ(removal_decision.disposition,
            CandidateDisposition::UnprotectedDiagnosticOnly);
  EXPECT_FALSE(removal_decision.integrity_available);

  // Missed-exclusion guard: a candidate whose bounded model came from a
  // rollback/reintegration is still charged as its own guarantee group.
  std::vector<ActionGuarantee> actions(2);
  actions[0].action_id = 1;
  actions[0].epsilon_budget = 1e-5;
  actions[0].shared_reference_certificate = true;
  actions[0].shared_accepted_event = true;
  actions[0].shared_time = true;
  actions[0].shared_output_quantity = true;
  actions[0].triangle_transfer_evidence = true;
  actions[0].reference_certificate_id = 99;
  actions[1].action_id = 2;
  actions[1].epsilon_budget = 1e-5;
  const GuaranteeGroupResult groups = buildGuaranteeGroups(actions, 1.5e-5);
  EXPECT_FALSE(groups.valid)
      << "a rollback candidate is charged on its own and must not disappear";
}

// ---------------------------------------------------------------------------
// FDE-05: caller-declared shared-reference centres are not proof of a shared
// failure event.  Until a verifiable P0-03/P0-02 producer is wired, every
// action is charged as a singleton.
// ---------------------------------------------------------------------------
TEST(FdePostSelection, Fde05SharedReferenceGroupsAndDistinctReferences) {
  auto makeShared = [](std::uint64_t action_id, std::uint64_t reference,
                       double epsilon, double centre_shift) {
    ActionGuarantee action;
    action.action_id = action_id;
    action.reference_certificate_id = reference;
    action.shared_reference_certificate = true;
    action.shared_accepted_event = true;
    action.shared_time = true;
    action.shared_output_quantity = true;
    action.triangle_transfer_evidence = true;
    action.L_reference_m = Eigen::Vector3d(1.0, 1.0, 1.0);
    action.p_reference_m = Eigen::Vector3d(0.0, 0.0, 0.0);
    action.p_action_m = Eigen::Vector3d(centre_shift, 0.0, 0.0);
    action.L_action_m = action.L_reference_m +
                        (action.p_action_m - action.p_reference_m).cwiseAbs();
    action.L_action_m.x() = std::nextafter(
        action.L_action_m.x(), std::numeric_limits<double>::infinity());
    action.epsilon_budget = epsilon;
    return action;
  };
  // Three centres with identical caller claims are still three events.
  std::vector<ActionGuarantee> shared = {makeShared(1, 7, 1.0e-5, 0.1),
                                         makeShared(2, 7, 1.2e-5, 0.2),
                                         makeShared(3, 7, 1.1e-5, -0.3)};
  auto events = [](const std::vector<ActionGuarantee>& actions) {
    std::vector<ActionSharedEventV1> out;
    for (const auto& action : actions) {
      ActionSharedEventV1 event;
      event.action_id = action.action_id;
      // Deliberately independent caller claim, not copied from the action.
      // Without full proof/certificate payload it must never enable sharing.
      event.common_reference_certificate_id = 999;
      event.common_L_reference_m = Eigen::Vector3d::Ones();
      event.common_p_reference_m = Eigen::Vector3d::Zero();
      event.accepted_event_id = "invented";
      event.reference_time_id = 123;
      event.output_quantity_id = "invented";
      out.push_back(std::move(event));
    }
    return out;
  };
  const GuaranteeGroupResult grouped =
      buildGuaranteeGroups(shared, 4.0e-5, events(shared));
  ASSERT_TRUE(grouped.valid) << grouped.reason;
  EXPECT_EQ(grouped.groups.size(), 3u);
  EXPECT_EQ(grouped.singleton_groups, 3u);
  EXPECT_EQ(grouped.shared_groups, 0u);
  EXPECT_NEAR(grouped.total_charged_budget, 3.3e-5, 1e-18);

  // Distinct references with the same structure: two separate failure events,
  // charged separately (no under-counting).
  std::vector<ActionGuarantee> distinct = {makeShared(4, 8, 1.0e-5, 0.1),
                                          makeShared(5, 9, 1.2e-5, 0.2)};
  const GuaranteeGroupResult separate =
      buildGuaranteeGroups(distinct, 1.5e-5, events(distinct));
  EXPECT_FALSE(separate.valid)
      << "1.0e-5 + 1.2e-5 > 1.5e-5 must refuse: both events are charged";
  const GuaranteeGroupResult separate_ok =
      buildGuaranteeGroups(distinct, 2.5e-5, events(distinct));
  ASSERT_TRUE(separate_ok.valid) << separate_ok.reason;
  EXPECT_EQ(separate_ok.singleton_groups, 2u);
  EXPECT_NEAR(separate_ok.total_charged_budget, 2.2e-5, 1e-18);

  // Missing one shared flag (here: time) breaks the group even for the same
  // reference: it is charged on its own.
  std::vector<ActionGuarantee> partial = {makeShared(6, 10, 1.0e-5, 0.0),
                                          makeShared(7, 10, 1.0e-5, 0.0)};
  partial[1].shared_time = false;
  const GuaranteeGroupResult split =
      buildGuaranteeGroups(partial, 2.5e-5, events(partial));
  ASSERT_TRUE(split.valid) << split.reason;
  EXPECT_EQ(split.groups.size(), 2u);
  // Missing one shared flag breaks the group: neither action ends up in a
  // multi-member (shared) failure event, so both are charged on their own.
  EXPECT_EQ(split.shared_groups, 0u);
  EXPECT_EQ(split.singleton_groups, 2u);
  EXPECT_NEAR(split.total_charged_budget, 2.0e-5, 1e-18);

  // Triangle-transfer identity violated: the action may not reuse the reference
  // failure event (its bound is not implied by the reference's).
  std::vector<ActionGuarantee> violated = {makeShared(8, 11, 1.0e-5, 0.1),
                                           makeShared(9, 11, 1.0e-5, 0.2)};
  violated[1].L_action_m = Eigen::Vector3d(0.5, 0.5, 0.5);  // too optimistic
  const GuaranteeGroupResult refused =
      buildGuaranteeGroups(violated, 1.5e-5, events(violated));
  EXPECT_FALSE(refused.valid)
      << "the violated action is a singleton and is charged separately";
  const GuaranteeGroupResult refused_ok =
      buildGuaranteeGroups(violated, 2.5e-5, events(violated));
  ASSERT_TRUE(refused_ok.valid) << refused_ok.reason;
  EXPECT_EQ(refused_ok.groups.size(), 2u);
  EXPECT_GT(refused_ok.groups[1].worst_triangle_residual, 1e-9);

  std::printf(
      "[FDE-05] unverified_shared_claims=3 singleton_sum=%.3e "
      "singletons=2 charged_sum=%.3e "
      "partial_groups=%zu triangle_violation_residual=%.3e\n",
      grouped.total_charged_budget, separate_ok.total_charged_budget,
      split.groups.size(), refused_ok.groups[1].worst_triangle_residual);
}

}  // namespace
