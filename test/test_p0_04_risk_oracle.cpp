#include <gtest/gtest.h>

#include <boost/multiprecision/cpp_bin_float.hpp>

#include <cmath>
#include <limits>
#include <vector>

#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/fde_post_selection.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

namespace {

using namespace uwb_imu_pl;
using Quad = boost::multiprecision::cpp_bin_float_quad;

FaultHypothesisV2 hypothesis(std::uint64_t id, double allocation,
                             double prior, double p_md) {
  FaultHypothesisV2 out;
  out.id = HypothesisId(id);
  out.units = {FaultUnitId(id + 100)};
  out.hmi_allocation = allocation;
  out.prior_probability_bound = prior;
  out.p_md_allocation = p_md;
  return out;
}

FaultModeEvidence evidenceFor(const FaultHypothesisV2& hypothesis) {
  FaultModeEvidence out;
  out.hypothesis = hypothesis.id;
  out.plausible = true;
  out.unit_kind = FaultUnitKind::UwbRangeMeters;
  out.parameter_dimension = 1;
  out.profile_j = 1.0;
  out.profile_valid = true;
  return out;
}

CandidateEvaluation candidateFor(std::uint64_t id,
                                 const FaultHypothesisV2& fault) {
  CandidateEvaluation out;
  out.valid = true;
  out.post_detector_passed = true;
  out.hpl_m = 1.0;
  out.vpl_m = 1.0;
  out.action.id = ExclusionActionId(id);
  out.action.covered_units = fault.units;
  out.action.exclusion_cardinality = 1;
  out.action.recoverability = HistoryRecoverability::Recoverable;
  return out;
}

CompleteRiskInputsV1 qualifiedInputs() {
  CompleteRiskInputsV1 out;
  out.omitted_scope_complete = true;
  out.omitted_event_bound = 0.0;
  out.omitted_event_bound_known = true;
  out.omitted_event_bound_validated = true;
  out.legacy.envelope_online = false;
  out.legacy.envelope_leaf_count = 0;
  out.envelope_event_bound = 0.0;
  out.envelope_event_bound_known = true;
  out.envelope_event_bound_validated = true;
  out.bridge_escape_validated = true;
  out.history_escape_validated = true;
  out.model_escape_validated = true;
  out.legacy.selection_contract_frozen = true;
  out.selection_extra_bound = 0.0;
  out.selection_bound_known = true;
  out.selection_bound_validated = true;
  out.model_formal_eligible = true;
  return out;
}

// Independent O06 arithmetic: pi*max(allocation/pi, p_md) must be in the
// total.  It is deliberately expressed directly in quad precision and does
// not call axisTailSplit or another production helper.
TEST(P004RiskOracle, MissChannelIsChargedIntoTheSameTotal) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 4.0e-5;
  risk.nominal_axis_tail = 1.0e-6;
  risk.p_nm = 1.0e-7;
  const FaultHypothesisV2 fault = hypothesis(1, 3.0e-8, 1.0e-4, 1.0e-3);

  CompleteRiskInputsV1 inputs;
  inputs.legacy.omitted_event_set.clear();
  const RiskLedger ledger = buildRiskLedger(risk, {fault}, inputs, nullptr);

  const Quad alpha = Quad(fault.hmi_allocation) /
      Quad(fault.prior_probability_bound);
  const Quad beta(fault.p_md_allocation);
  const Quad expected = Quad(3) * Quad(risk.nominal_axis_tail) +
      Quad(risk.p_nm) +
      Quad(fault.prior_probability_bound) *
          (alpha > beta ? alpha : beta);
  const RiskLedgerTerm* miss = ledger.find("hypotheses_miss_channel");
  ASSERT_NE(miss, nullptr);
  EXPECT_EQ(miss->status, RiskTermStatus::Validated);
  EXPECT_GT(miss->value, 0.0);
  EXPECT_NEAR(ledger.charged_total, static_cast<double>(expected), 1e-20);
}

TEST(P004RiskOracle, ZeroAllocationStillChargesPriorWeightedMiss) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 4.0e-5;
  risk.nominal_axis_tail = 0.0;
  risk.p_nm = 0.0;
  const FaultHypothesisV2 fault = hypothesis(1, 0.0, 1.0e-2, 1.0e-3);
  const RiskLedger ledger = buildRiskLedger(
      risk, {fault}, qualifiedInputs(), nullptr);
  const RiskLedgerTerm* miss = ledger.find("hypotheses_miss_channel");
  ASSERT_NE(miss, nullptr);
  EXPECT_EQ(miss->status, RiskTermStatus::Validated);
  EXPECT_NEAR(miss->value, 1.0e-5, 1e-20);
  EXPECT_NEAR(ledger.charged_total, 1.0e-5, 1e-20);
}

TEST(P004RiskOracle, FourStatusesAreIndependentAndUnknownIsNotZero) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 4.0e-5;
  risk.nominal_axis_tail = 1.0e-6;
  risk.p_nm = 1.0e-7;
  const FaultHypothesisV2 fault = hypothesis(1, 3.0e-8, 1.0e-4, 1.0e-5);

  CompleteRiskStatusV1 unknown_status;
  const RiskLedger unknown = buildRiskLedger(
      risk, {fault}, CompleteRiskInputsV1{}, &unknown_status);
  EXPECT_TRUE(unknown_status.allocation_valid);
  EXPECT_FALSE(unknown_status.complete_bound_closes);
  EXPECT_FALSE(unknown.all_terms_validated);
  EXPECT_FALSE(unknown.formal_eligible);
  ASSERT_NE(unknown.find("bridge"), nullptr);
  ASSERT_NE(unknown.find("omitted"), nullptr);
  ASSERT_NE(unknown.find("selection"), nullptr);
  EXPECT_TRUE(std::isnan(unknown.find("bridge")->value));
  EXPECT_TRUE(std::isnan(unknown.find("omitted")->value));
  EXPECT_TRUE(std::isnan(unknown.find("selection")->value));

  CompleteRiskInputsV1 false_validation = qualifiedInputs();
  false_validation.omitted_scope_complete = false;
  false_validation.omitted_event_bound_validated = true;
  false_validation.omitted_event_bound_known = false;
  CompleteRiskStatusV1 still_unknown_status;
  const RiskLedger still_unknown = buildRiskLedger(
      risk, {fault}, false_validation, &still_unknown_status);
  EXPECT_FALSE(still_unknown.all_terms_validated);
  EXPECT_FALSE(still_unknown_status.complete_bound_closes);
  EXPECT_TRUE(std::isnan(still_unknown.find("omitted")->value));

  CompleteRiskStatusV1 qualified_status;
  const RiskLedger qualified = buildRiskLedger(
      risk, {fault}, qualifiedInputs(), &qualified_status);
  EXPECT_TRUE(qualified_status.allocation_valid);
  EXPECT_TRUE(qualified_status.complete_bound_closes) << qualified.reason;
  EXPECT_TRUE(qualified.all_terms_validated) << qualified.reason;
  EXPECT_TRUE(qualified.formal_eligible) << qualified.reason;
}

TEST(P004RiskOracle, AllocationCanFitWhileCompleteMissBoundExceedsBudget) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 4.0e-5;
  risk.nominal_axis_tail = 1.0e-5;
  risk.p_nm = 0.0;
  // The allocation precheck sees 3e-5 + 1e-6 <= 4e-5.  The complete bound
  // sees pi*beta=1e-4 and must refuse without changing any input budget.
  const FaultHypothesisV2 fault = hypothesis(1, 1.0e-6, 1.0e-1, 1.0e-3);
  CompleteRiskStatusV1 ledger_status;
  const RiskLedger ledger = buildRiskLedger(
      risk, {fault}, qualifiedInputs(), &ledger_status);
  EXPECT_TRUE(ledger_status.allocation_valid);
  EXPECT_TRUE(ledger.all_terms_validated);
  EXPECT_FALSE(ledger_status.complete_bound_closes);
  EXPECT_FALSE(ledger.formal_eligible);
  EXPECT_GT(ledger.charged_total, risk.p_hmi_total);
}

TEST(P004RiskOracle, QuadBoundaryCannotBeRoundedBackIntoBudget) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 0.3;
  risk.nominal_axis_tail = 0.1;
  risk.p_nm = 0.0;
  CompleteRiskStatusV1 status;
  const RiskLedger ledger = buildRiskLedger(
      risk, {}, qualifiedInputs(), &status);

  // Exact(binary64(0.1))*3 > exact(binary64(0.3)); a binary64 recomputation
  // is not authoritative and exported totals must round upward.
  EXPECT_FALSE(status.allocation_valid);
  EXPECT_FALSE(status.complete_bound_closes);
  EXPECT_TRUE(ledger.all_terms_validated);
  EXPECT_FALSE(ledger.formal_eligible);
  EXPECT_GT(ledger.charged_total, risk.p_hmi_total);
}

TEST(P004RiskOracle, ReviewerExactMissBoundaryRemainsOverBudget) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 1.2075810982130189e-14;
  risk.nominal_axis_tail = 0.0;
  risk.p_nm = 0.0;
  const FaultHypothesisV2 fault = hypothesis(
      1, 5.226175689748075e-15, 2.0503228027277203e-12,
      0.005889712081465759);

  CompleteRiskStatusV1 status;
  const RiskLedger ledger = buildRiskLedger(
      risk, {fault}, qualifiedInputs(), &status);
  const Quad expected = Quad(fault.prior_probability_bound) *
      Quad(fault.p_md_allocation);

  ASSERT_GT(expected, Quad(risk.p_hmi_total));
  EXPECT_TRUE(status.allocation_valid);
  EXPECT_FALSE(status.complete_bound_closes);
  EXPECT_TRUE(ledger.all_terms_validated);
  EXPECT_FALSE(ledger.formal_eligible);
  EXPECT_GE(Quad(ledger.charged_total), expected);
  EXPECT_GT(ledger.charged_total, risk.p_hmi_total);
}

TEST(P004RiskOracle, InvalidCompleteRiskBoundsFailClosedWithoutNegativeExport) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 4.0e-5;
  const FaultHypothesisV2 fault = hypothesis(1, 0.0, 1.0e-2, 1.0e-3);
  const std::vector<double> invalid{
      -2.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};

  for (const double value : invalid) {
    CompleteRiskInputsV1 omitted = qualifiedInputs();
    omitted.omitted_scope_complete = false;
    omitted.legacy.omitted_event_set = {"independent-omitted-event"};
    omitted.omitted_event_bound = value;
    omitted.omitted_event_bound_known = true;
    omitted.omitted_event_bound_validated = true;
    CompleteRiskStatusV1 omitted_status;
    const RiskLedger omitted_ledger = buildRiskLedger(
        risk, {fault}, omitted, &omitted_status);
    ASSERT_NE(omitted_ledger.find("omitted"), nullptr);
    EXPECT_TRUE(std::isnan(omitted_ledger.find("omitted")->value));
    EXPECT_FALSE(omitted_status.complete_bound_closes);
    EXPECT_FALSE(omitted_ledger.all_terms_validated);
    EXPECT_FALSE(omitted_ledger.formal_eligible);
    EXPECT_FALSE(omitted_ledger.charged_total < 0.0);

    CompleteRiskInputsV1 envelope = qualifiedInputs();
    envelope.legacy.envelope_online = true;
    envelope.legacy.envelope_leaf_count = 1;
    envelope.envelope_event_bound = value;
    envelope.envelope_event_bound_known = true;
    envelope.envelope_event_bound_validated = true;
    CompleteRiskStatusV1 envelope_status;
    const RiskLedger envelope_ledger = buildRiskLedger(
        risk, {fault}, envelope, &envelope_status);
    ASSERT_NE(envelope_ledger.find("envelope"), nullptr);
    EXPECT_TRUE(std::isnan(envelope_ledger.find("envelope")->value));
    EXPECT_FALSE(envelope_status.complete_bound_closes);
    EXPECT_FALSE(envelope_ledger.all_terms_validated);
    EXPECT_FALSE(envelope_ledger.formal_eligible);
    EXPECT_FALSE(envelope_ledger.charged_total < 0.0);

    CompleteRiskInputsV1 selection = qualifiedInputs();
    selection.selection_extra_bound = value;
    CompleteRiskStatusV1 selection_status;
    const RiskLedger selection_ledger = buildRiskLedger(
        risk, {fault}, selection, &selection_status);
    ASSERT_NE(selection_ledger.find("selection"), nullptr);
    EXPECT_TRUE(std::isnan(selection_ledger.find("selection")->value));
    EXPECT_FALSE(selection_status.complete_bound_closes);
    EXPECT_FALSE(selection_ledger.all_terms_validated);
    EXPECT_FALSE(selection_ledger.formal_eligible);
    EXPECT_FALSE(selection_ledger.charged_total < 0.0);
  }
}

// Independent representation oracle for the three sidecar terms.  This is a
// full Boolean Cartesian product, not a replay of production predicates: each
// raw context/known/validated combination is crossed with all IEEE boundary
// classes requested by the P0-04 review.
TEST(P004RiskOracle, SidecarTermsObeyCanonicalCartesianContract) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 1.0;
  const std::vector<double> values{
      -std::numeric_limits<double>::infinity(), -0.25, -0.0, 0.0, 0.25,
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()};

  const auto finite_nonnegative = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  const auto check = [&](const RiskLedger& ledger,
                         const CompleteRiskStatusV1& status,
                         const char* id, bool representation_valid,
                         bool term_known, bool term_validated,
                         double expected_value) {
    const RiskLedgerTerm* term = ledger.find(id);
    ASSERT_NE(term, nullptr);
    EXPECT_EQ(ledger.inputs_valid, representation_valid);
    EXPECT_EQ(status.complete_bound_closes, term_known);
    EXPECT_EQ(ledger.all_terms_validated,
              term_known && term_validated);
    EXPECT_EQ(ledger.formal_eligible, term_known && term_validated);
    if (term_known) {
      EXPECT_TRUE(std::isfinite(term->value));
      EXPECT_EQ(term->value, expected_value);
      EXPECT_EQ(term->status, term_validated
          ? RiskTermStatus::Validated
          : RiskTermStatus::AssumedUnvalidated);
    } else {
      EXPECT_TRUE(std::isnan(term->value));
      EXPECT_EQ(term->status, RiskTermStatus::NotImplemented);
    }
  };

  // Omitted-event raw state: scope-complete x event-present x known x
  // validated x seven bound classes (112 cases).
  for (bool scope_complete : {false, true}) {
    for (bool event_present : {false, true}) {
      for (bool known : {false, true}) {
        for (bool validated : {false, true}) {
          for (double value : values) {
            SCOPED_TRACE(::testing::Message()
                << "omitted scope=" << scope_complete
                << " event=" << event_present << " known=" << known
                << " validated=" << validated << " value=" << value);
            CompleteRiskInputsV1 inputs = qualifiedInputs();
            inputs.omitted_scope_complete = scope_complete;
            inputs.legacy.omitted_event_set = event_present
                ? std::vector<std::string>{"independent-event"}
                : std::vector<std::string>{};
            inputs.omitted_event_bound = value;
            inputs.omitted_event_bound_known = known;
            inputs.omitted_event_bound_validated = validated;

            const bool numeric = finite_nonnegative(value);
            const bool flag_consistent = !validated || known;
            const bool zero_proof = scope_complete && !event_present;
            const bool explicit_events = scope_complete && event_present;
            const bool canonical_zero = zero_proof && numeric &&
                value == 0.0 && known && validated;
            const bool explicit_known = explicit_events && numeric &&
                flag_consistent && known;
            const bool representation_valid = zero_proof
                ? canonical_zero
                : (explicit_events
                    ? numeric && flag_consistent
                    : numeric && flag_consistent && !known && !validated &&
                        value == 0.0);
            const bool term_known = canonical_zero || explicit_known;
            const bool term_validated = canonical_zero ||
                (explicit_known && validated);
            CompleteRiskStatusV1 status;
            const RiskLedger ledger = buildRiskLedger(
                risk, {}, inputs, &status);
            check(ledger, status, "omitted", representation_valid,
                  term_known, term_validated, value);
          }
        }
      }
    }
  }

  // Envelope raw state: online x leaf-present x known x validated x seven
  // bound classes (112 cases).  The mixed online/leaf states are invalid.
  for (bool online : {false, true}) {
    for (bool leaf_present : {false, true}) {
      for (bool known : {false, true}) {
        for (bool validated : {false, true}) {
          for (double value : values) {
            SCOPED_TRACE(::testing::Message()
                << "envelope online=" << online
                << " leaf=" << leaf_present << " known=" << known
                << " validated=" << validated << " value=" << value);
            CompleteRiskInputsV1 inputs = qualifiedInputs();
            inputs.legacy.envelope_online = online;
            inputs.legacy.envelope_leaf_count = leaf_present ? 1u : 0u;
            inputs.envelope_event_bound = value;
            inputs.envelope_event_bound_known = known;
            inputs.envelope_event_bound_validated = validated;

            const bool numeric = finite_nonnegative(value);
            const bool flag_consistent = !validated || known;
            const bool zero_proof = !online && !leaf_present;
            const bool explicit_events = online && leaf_present;
            const bool canonical_zero = zero_proof && numeric &&
                value == 0.0 && known && validated;
            const bool explicit_known = explicit_events && numeric &&
                flag_consistent && known;
            const bool representation_valid = zero_proof
                ? canonical_zero
                : (explicit_events && numeric && flag_consistent);
            const bool term_known = canonical_zero || explicit_known;
            const bool term_validated = canonical_zero ||
                (explicit_known && validated);
            CompleteRiskStatusV1 status;
            const RiskLedger ledger = buildRiskLedger(
                risk, {}, inputs, &status);
            check(ledger, status, "envelope", representation_valid,
                  term_known, term_validated, value);
          }
        }
      }
    }
  }

  // Selection raw state: event-set-frozen x known x validated x seven bound
  // classes (56 cases).  There is no implicit zero proof before freezing.
  for (bool frozen : {false, true}) {
    for (bool known : {false, true}) {
      for (bool validated : {false, true}) {
        for (double value : values) {
          SCOPED_TRACE(::testing::Message()
              << "selection frozen=" << frozen << " known=" << known
              << " validated=" << validated << " value=" << value);
          CompleteRiskInputsV1 inputs = qualifiedInputs();
          inputs.legacy.selection_contract_frozen = frozen;
          inputs.selection_extra_bound = value;
          inputs.selection_bound_known = known;
          inputs.selection_bound_validated = validated;

          const bool numeric = finite_nonnegative(value);
          const bool flag_consistent = !validated || known;
          const bool representation_valid = frozen
              ? numeric && flag_consistent
              : numeric && flag_consistent && !known && !validated &&
                  value == 0.0;
          const bool term_known = frozen && numeric && flag_consistent &&
              known;
          const bool term_validated = term_known && validated;
          CompleteRiskStatusV1 status;
          const RiskLedger ledger = buildRiskLedger(
              risk, {}, inputs, &status);
          check(ledger, status, "selection", representation_valid,
                term_known, term_validated, value);
        }
      }
    }
  }
}

TEST(P004RiskOracle, RejectedHypothesisRiskIsUnknownRatherThanKnownZero) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 4.0e-5;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<FaultHypothesisV2> rejected{
      hypothesis(1, -1.0, 1.0e-2, 1.0e-3),
      hypothesis(2, nan, 1.0e-2, 1.0e-3),
      hypothesis(3, inf, 1.0e-2, 1.0e-3),
      hypothesis(4, 1.0e-8, -1.0, 1.0e-3),
      hypothesis(5, 1.0e-8, nan, 1.0e-3),
      hypothesis(6, 1.0e-8, inf, 1.0e-3),
      hypothesis(7, 1.0e-8, 1.0e-2, -1.0),
      hypothesis(8, 1.0e-8, 1.0e-2, nan),
      hypothesis(9, 1.0e-8, 1.0e-2, inf)};
  for (const auto& item : rejected) {
    CompleteRiskStatusV1 status;
    const RiskLedger ledger = buildRiskLedger(
        risk, {item}, qualifiedInputs(), &status);
    ASSERT_NE(ledger.find("hypotheses"), nullptr);
    ASSERT_NE(ledger.find("hypotheses_miss_channel"), nullptr);
    EXPECT_TRUE(std::isnan(ledger.find("hypotheses")->value));
    EXPECT_TRUE(std::isnan(ledger.find("hypotheses_miss_channel")->value));
    EXPECT_FALSE(status.complete_bound_closes);
    EXPECT_FALSE(ledger.all_terms_validated);
    EXPECT_FALSE(ledger.formal_eligible);
    EXPECT_FALSE(ledger.charged_total < 0.0);
  }
}

TEST(P004RiskOracle, OverflowFailsClosedAndProvenZeroTermsRemainFinite) {
  RiskBudgetV2 overflow;
  overflow.p_hmi_total = std::numeric_limits<double>::max();
  overflow.nominal_axis_tail = std::numeric_limits<double>::max();
  CompleteRiskStatusV1 overflow_status;
  const RiskLedger overflow_ledger = buildRiskLedger(
      overflow, {}, qualifiedInputs(), &overflow_status);
  EXPECT_FALSE(overflow_status.complete_bound_closes);
  EXPECT_FALSE(overflow_ledger.all_terms_validated);
  EXPECT_FALSE(overflow_ledger.formal_eligible);
  EXPECT_FALSE(overflow_ledger.charged_total < 0.0);

  RiskBudgetV2 zero;
  zero.p_hmi_total = 4.0e-5;
  zero.nominal_axis_tail = 0.0;
  zero.p_nm = 0.0;
  zero.p_bridge_escape = 0.0;
  zero.p_history_contamination = 0.0;
  zero.p_model_escape = 0.0;
  CompleteRiskStatusV1 zero_status;
  const RiskLedger zero_ledger = buildRiskLedger(
      zero, {}, qualifiedInputs(), &zero_status);
  ASSERT_TRUE(zero_status.complete_bound_closes) << zero_ledger.reason;
  ASSERT_TRUE(zero_ledger.all_terms_validated) << zero_ledger.reason;
  EXPECT_TRUE(zero_ledger.formal_eligible);
  for (const auto& term : zero_ledger.terms) {
    EXPECT_TRUE(std::isfinite(term.value));
    EXPECT_GE(term.value, 0.0);
  }
  EXPECT_EQ(zero_ledger.charged_total, 0.0);
}

TEST(P004RiskOracle, UnverifiableSharedEventClaimsRemainSingletons) {
  auto action = [](std::uint64_t id) {
    ActionGuarantee out;
    out.action_id = id;
    out.reference_certificate_id = 77;
    out.shared_reference_certificate = true;
    out.shared_accepted_event = true;
    out.shared_time = true;
    out.shared_output_quantity = true;
    out.triangle_transfer_evidence = true;
    out.L_reference_m = Eigen::Vector3d::Ones();
    out.p_reference_m = Eigen::Vector3d::Zero();
    out.p_action_m = Eigen::Vector3d(0.1 * id, 0.0, 0.0);
    out.L_action_m = out.L_reference_m + out.p_action_m.cwiseAbs();
    out.L_action_m.x() = std::nextafter(
        out.L_action_m.x(), std::numeric_limits<double>::infinity());
    out.epsilon_budget = 3.0e-5;
    return out;
  };
  ActionGuarantee first = action(1);
  ActionGuarantee second = action(2);
  // Independent producer/reference fixture: these values are deliberately
  // built without copying ActionGuarantee fields.  The current API still lacks
  // the full P0-03/P0-02 payload and recomputable frozen-identity digest, so
  // even internally consistent claims are not authority for event sharing.
  auto externalClaim = [](std::uint64_t action_id) {
    ActionSharedEventV1 out;
    out.action_id = action_id;
    out.common_reference_certificate_id = 77;
    out.common_L_reference_m = Eigen::Vector3d::Ones();
    out.common_p_reference_m = Eigen::Vector3d::Zero();
    out.accepted_event_id = "dual_channel_intersection_v6";
    out.reference_time_id = 1234;
    out.output_quantity_id = "protected_position_xyz_m";
    return out;
  };
  const std::vector<ActionSharedEventV1> same_event{
      externalClaim(1), externalClaim(2)};
  const GuaranteeGroupResult shared =
      buildGuaranteeGroups({first, second}, 7.0e-5, same_event);
  ASSERT_TRUE(shared.valid) << shared.reason;
  EXPECT_EQ(shared.groups.size(), 2u);
  EXPECT_EQ(shared.singleton_groups, 2u);
  EXPECT_EQ(shared.shared_groups, 0u);
  EXPECT_NEAR(shared.total_charged_budget, 6.0e-5, 1e-20);

  // Exact reviewer forgery probe: caller-invented identifiers cannot turn two
  // disjoint actions into one 3e-5 event under the unchanged 4e-5 budget.
  auto forged = same_event;
  for (auto& claim : forged) {
    claim.common_reference_certificate_id = 999;
    claim.accepted_event_id = "invented";
    claim.reference_time_id = 123;
    claim.output_quantity_id = "invented";
  }
  const GuaranteeGroupResult forged_result =
      buildGuaranteeGroups({first, second}, 4.0e-5, forged);
  EXPECT_FALSE(forged_result.valid);
  EXPECT_EQ(forged_result.groups.size(), 2u);
  EXPECT_EQ(forged_result.singleton_groups, 2u);
  EXPECT_EQ(forged_result.shared_groups, 0u);
  EXPECT_NEAR(forged_result.total_charged_budget, 6.0e-5, 1e-20);

  const GuaranteeGroupResult no_sidecar =
      buildGuaranteeGroups({first, second}, 4.0e-5);
  EXPECT_EQ(no_sidecar.groups.size(), 2u);
  EXPECT_FALSE(no_sidecar.valid);

  auto tampered = same_event;
  tampered[1].common_p_reference_m.x() = 99.0;
  tampered[1].accepted_event_id = "tampered";
  const GuaranteeGroupResult tampered_result =
      buildGuaranteeGroups({first, second}, 4.0e-5, tampered);
  EXPECT_FALSE(tampered_result.valid);
  EXPECT_EQ(tampered_result.groups.size(), 2u);
  EXPECT_NEAR(tampered_result.total_charged_budget, 6.0e-5, 1e-20);

  ActionGuarantee duplicate = second;
  duplicate.action_id = first.action_id;
  EXPECT_FALSE(buildGuaranteeGroups(
      {first, duplicate}, 1.0, same_event).valid);

}

// D06's exact counterexample at the production caller: equal plausible IDs
// are labels, not a shared failure event.  With no numeric transfer proof the
// two original actions are singletons and 2*3e-5 exceeds the unchanged 4e-5.
TEST(P004RiskOracle, ManagerDoesNotPremergeDisjointActionsByPlausibleId) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 4.0e-5;
  risk.nominal_axis_tail = 0.0;
  risk.p_nm = 0.0;
  const FaultHypothesisV2 fault = hypothesis(1, 3.0e-5, 1.0e-2, 1.0e-4);

  DetectorResultV2 detector;
  detector.numerically_valid = true;
  detector.passed = false;
  std::vector<CandidateEvaluation> candidates{
      candidateFor(10, fault), candidateFor(11, fault)};
  const FdeDecision decision = FdeManager().decide(
      detector, {fault}, {evidenceFor(fault)}, &candidates, {}, risk);

  EXPECT_EQ(decision.selection_event_classes, 2u) << decision.reason;
  EXPECT_NEAR(decision.selection_charged_budget, 6.0e-5, 1e-20);
  EXPECT_FALSE(decision.selection_budget_ok);
  EXPECT_FALSE(decision.commit_allowed);
  EXPECT_FALSE(decision.integrity_available);
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_FALSE(candidates[0].selected);
  EXPECT_FALSE(candidates[1].selected);
}

TEST(P004RiskOracle, NoAlarmDoesNotMakeActionSelectionEventsFree) {
  RiskBudgetV2 risk;
  risk.p_hmi_total = 4.0e-5;
  risk.nominal_axis_tail = 0.0;
  risk.p_nm = 0.0;
  const FaultHypothesisV2 fault = hypothesis(1, 3.0e-5, 1.0e-2, 1.0e-4);

  DetectorResultV2 detector;
  detector.numerically_valid = true;
  detector.passed = true;
  std::vector<CandidateEvaluation> candidates{
      candidateFor(20, fault), candidateFor(21, fault)};
  const FdeDecision decision = FdeManager().decide(
      detector, {fault}, {}, &candidates, {}, risk);

  EXPECT_TRUE(decision.plausible_hypotheses.empty());
  EXPECT_EQ(decision.selection_event_classes, 2u);
  EXPECT_NEAR(decision.selection_charged_budget, 6.0e-5, 1e-20);
  EXPECT_FALSE(decision.selection_budget_ok);
  EXPECT_FALSE(decision.commit_allowed);
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_FALSE(candidates[0].selected);
  EXPECT_FALSE(candidates[1].selected);
}

}  // namespace
