#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/common/integrity_identity.hpp"
#include "uwb_imu_pl/integrity/fde_post_selection.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <boost/multiprecision/cpp_bin_float.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <tuple>

namespace uwb_imu_pl {
namespace {

using ExactRisk = boost::multiprecision::cpp_bin_float_quad;

double roundRiskUp(const ExactRisk& value) {
  double rounded = static_cast<double>(value);
  while (std::isfinite(rounded) && ExactRisk(rounded) < value) {
    rounded = std::nextafter(
        rounded, std::numeric_limits<double>::infinity());
  }
  return rounded;
}

bool covers(const ExclusionAction& action, const FaultHypothesisV2& hypothesis) {
  if (!hypothesis.modes.empty()) {
    for (const auto mode : hypothesis.modes) {
      if (std::find(action.covered_modes.begin(), action.covered_modes.end(), mode) ==
          action.covered_modes.end()) return false;
    }
  } else {
    for (const auto unit : hypothesis.units) {
      if (std::find(action.covered_units.begin(), action.covered_units.end(), unit) ==
          action.covered_units.end()) return false;
    }
  }
  for (const auto group : hypothesis.affected_groups) {
    if (std::find(action.groups_to_remove.begin(), action.groups_to_remove.end(), group) ==
        action.groups_to_remove.end()) return false;
  }
  return true;
}

FdeStatus statusFor(const ExclusionAction& action) {
  if (action.exclusion_cardinality == 0) return FdeStatus::SuccessKeepAll;
  const bool bridge = action.bridge_mode != BridgeMode::None;
  const bool uwb = action.action_model_id.find("UWB") != std::string::npos;
  if (bridge && uwb) return FdeStatus::SuccessMultiSensorExclusion;
  if (bridge && action.bridge_mode == BridgeMode::GenericKinematic) {
    return FdeStatus::SuccessImuExclusionGenericBridge;
  }
  if (bridge) return FdeStatus::SuccessImuExclusionDynamicsBridge;
  return FdeStatus::SuccessUwbExclusion;
}

}  // namespace

FdeDecision FdeManager::decide(
    const DetectorResultV2& all_in,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const std::vector<FaultModeEvidence>& evidence,
    std::vector<CandidateEvaluation>* candidates,
    const std::vector<FactorGroupId>& mandatory_groups,
    const RiskBudgetV2& risk) const {
  return decide(all_in, hypotheses, evidence, candidates, mandatory_groups,
                risk, static_cast<const FdeDecisionContextV1*>(nullptr));
}

FdeDecision FdeManager::decide(
    const DetectorResultV2& all_in,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const std::vector<FaultModeEvidence>& evidence,
    std::vector<CandidateEvaluation>* candidates,
    const std::vector<FactorGroupId>& mandatory_groups,
    const RiskBudgetV2& risk,
    const FdeDecisionContextV3* context) const {
  std::vector<ExclusionAction> consumed;
  if (candidates) {
    consumed.reserve(candidates->size());
    for (const auto& candidate : *candidates) {
      consumed.push_back(candidate.action);
    }
  }
  const ActionSearchValidationV1 validation =
      context && context->action_lease
          ? validateCompactActionConsumptionV3(*context->action_lease,
                                               consumed)
          : ActionSearchValidationV1{
                false, false, "compact action lease missing"};
  if (!validation.valid || !validation.exhaustive) {
    if (context && context->v1.risk_result) {
      *context->v1.risk_result = {};
      context->v1.risk_result->charged_total =
          std::numeric_limits<double>::quiet_NaN();
      context->v1.risk_result->declared_total =
          std::numeric_limits<double>::quiet_NaN();
      context->v1.risk_result->margin =
          std::numeric_limits<double>::quiet_NaN();
      context->v1.risk_result->terms =
          "SEARCH_INCOMPLETE: compact census validation=" +
          validation.reason;
    }
    FdeDecision decision;
    decision.status = FdeStatus::SearchIncomplete;
    decision.commit_allowed = false;
    decision.integrity_available = false;
    decision.reason =
        "SEARCH_INCOMPLETE: compact validation=" + validation.reason;
    return decision;
  }
  return decide(all_in, hypotheses, evidence, candidates, mandatory_groups,
                risk, context ? &context->v1 : nullptr);
}

FdeDecision FdeManager::decide(
    const DetectorResultV2& all_in,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const std::vector<FaultModeEvidence>& evidence,
    std::vector<CandidateEvaluation>* candidates,
    const std::vector<FactorGroupId>& mandatory_groups,
    const RiskBudgetV2& risk,
    const FdeDecisionContextV2* context) const {
  ActionSearchValidationV1 search_validation;
  if (context && context->action_search) {
    std::vector<ExclusionAction> consumed_actions;
    if (candidates) {
      consumed_actions.reserve(candidates->size());
      for (const auto& candidate : *candidates) {
        consumed_actions.push_back(candidate.action);
      }
    }
    if (!context->trusted_generated_actions) {
      search_validation = {false, false,
                           "trusted raw generated snapshot missing"};
    } else {
      search_validation = validateActionSearchCensusV1(
          *context->action_search, *context->trusted_generated_actions,
          context->max_evaluated_actions, context->action_search_lifecycle,
          consumed_actions);
    }
  }
  if (context && context->action_search &&
      (!search_validation.valid || !search_validation.exhaustive)) {
    if (context->v1.risk_result) {
      *context->v1.risk_result = {};
      context->v1.risk_result->charged_total =
          std::numeric_limits<double>::quiet_NaN();
      context->v1.risk_result->declared_total =
          std::numeric_limits<double>::quiet_NaN();
      context->v1.risk_result->margin =
          std::numeric_limits<double>::quiet_NaN();
      context->v1.risk_result->terms =
          "SEARCH_INCOMPLETE: census validation=" + search_validation.reason;
    }
    FdeDecision decision;
    decision.status = FdeStatus::SearchIncomplete;
    decision.commit_allowed = false;
    decision.integrity_available = false;
    decision.reason = "SEARCH_INCOMPLETE: generated=" +
        std::to_string(context->action_search->generated) +
        " evaluated=" +
        std::to_string(context->action_search->evaluated) +
        " omitted=" +
        std::to_string(context->action_search->omitted) +
        " validation=" + search_validation.reason;
    return decision;
  }
  return decide(all_in, hypotheses, evidence, candidates, mandatory_groups,
                risk, context ? &context->v1 : nullptr);
}

FdeDecision FdeManager::decide(
    const DetectorResultV2& all_in,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const std::vector<FaultModeEvidence>& evidence,
    std::vector<CandidateEvaluation>* candidates,
    const std::vector<FactorGroupId>& mandatory_groups,
    const RiskBudgetV2& risk,
    const FdeDecisionContextV1* context) const {
  FdeDecision decision;
  FdeRiskDecisionV1 local_risk;
  FdeRiskDecisionV1* risk_result = context && context->risk_result
      ? context->risk_result : &local_risk;
  *risk_result = {};
  risk_result->charged_total = std::numeric_limits<double>::quiet_NaN();
  risk_result->declared_total = std::numeric_limits<double>::quiet_NaN();
  risk_result->margin = std::numeric_limits<double>::quiet_NaN();
  risk_result->terms =
      "complete risk ledger unavailable before selection closure";
  const auto* protection_proof_ids = context
      ? context->protection_proof_ids : nullptr;
  if (!candidates || !all_in.numerically_valid) {
    decision.status = FdeStatus::ModelInvalid;
    decision.reason = "joint detector or candidate set invalid";
    return decision;
  }
  const RiskBudgetAudit risk_audit = auditRiskBudget(risk, hypotheses);
  risk_result->allocation_valid = risk_audit.valid;
  if (!risk_result->allocation_valid) {
    decision.status = FdeStatus::RiskBudgetInvalid;
    decision.reason = "V2 outcome risk budget does not close";
    return decision;
  }
  if (!all_in.passed) {
    decision.plausible_hypotheses =
        completePlausibleHypotheses(hypotheses, evidence);
    if (decision.plausible_hypotheses.empty() && mandatory_groups.empty()) {
      decision.status = FdeStatus::NoValidCandidate;
      decision.reason = "alarm has no risk-preserving plausible hypothesis";
      return decision;
    }
  }
  decision.mandatory_exclusion_groups = mandatory_groups;
  // §8.4 wiring (C3): the structured evidence pool of the plausible hypotheses
  // is the comparability audit of everything the ordering below may consume.
  // Malformed evidence (missing record / non-finite raw profile) refuses
  // fail-closed; a pool that mixes units or dimensions is *recorded* as not
  // comparable and is never ranked, while the action order itself stays on the
  // frozen criterion (it never compares profile values across units).
  {
    std::vector<StructuredCandidate> pool;
    pool.reserve(decision.plausible_hypotheses.size());
    bool malformed = false;
    std::string malformed_reason;
    std::set<FaultUnitKind> pool_units;
    std::set<std::size_t> pool_dimensions;
    for (const auto id : decision.plausible_hypotheses) {
      const FaultModeEvidence* match = nullptr;
      for (const auto& entry : evidence) {
        if (entry.hypothesis == id) {
          match = &entry;
          break;
        }
      }
      if (match == nullptr) {
        malformed = true;
        malformed_reason = "plausible hypothesis has no evidence record";
        break;
      }
      if (!match->profile_valid || !std::isfinite(match->profile_j)) {
        malformed = true;
        malformed_reason =
            "plausible hypothesis has no finite raw profile evidence";
        break;
      }
      StructuredCandidate pooled;
      pooled.hypothesis = id;
      pooled.unit = match->unit_kind;
      pooled.parameter_dim = match->parameter_dimension;
      pooled.j_profile = match->profile_j;
      pooled.explained_energy = match->explained_energy;
      pooled.has_valid_reference = match->profile_valid;
      pool_units.insert(match->unit_kind);
      pool_dimensions.insert(match->parameter_dimension);
      decision.plausible_profile_j.push_back(match->profile_j);
      pool.push_back(pooled);
    }
    decision.profile_pool_valid = !malformed;
    if (malformed) {
      decision.status = FdeStatus::ModelInvalid;
      decision.reason = malformed_reason;
      return decision;
    }
    decision.profile_pool_mixed_units = pool_units.size() > 1;
    decision.profile_pool_mixed_dimensions = pool_dimensions.size() > 1;
    if (!pool.empty()) {
      const CandidateRanking ranking = rankStructuredCandidates(pool);
      decision.profile_pool_ranked = ranking.valid;
      decision.profile_pool_comparable = ranking.valid &&
          !ranking.mixed_units && !ranking.mixed_dimensions;
    } else {
      decision.profile_pool_comparable = true;
    }
  }
  std::map<HypothesisId, const FaultHypothesisV2*> hypothesis_index;
  for (const auto& h : hypotheses) hypothesis_index.emplace(h.id, &h);
  std::vector<std::size_t> eligible;
  decision.candidate_dispositions.assign(candidates->size(), 0);
  for (std::size_t i = 0; i < candidates->size(); ++i) {
    auto& candidate = (*candidates)[i];
    bool covers_all = true;
    for (const auto id : decision.plausible_hypotheses) {
      const auto found = hypothesis_index.find(id);
      if (found == hypothesis_index.end() || !covers(candidate.action, *found->second)) {
        covers_all = false;
        break;
      }
    }
    if (covers_all) {
      covers_all = std::all_of(mandatory_groups.begin(), mandatory_groups.end(),
          [&](FactorGroupId group) {
            return std::find(candidate.action.groups_to_remove.begin(),
                             candidate.action.groups_to_remove.end(), group) !=
                candidate.action.groups_to_remove.end();
          });
    }
    candidate.covers_plausible_set = covers_all;
    const bool finite_pl = std::isfinite(candidate.hpl_m) &&
        std::isfinite(candidate.vpl_m);
    const bool action_model_valid = candidate.action.bridge_mode ==
        BridgeMode::None || candidate.action.model_error_validated;
    StructuredCandidate view;
    view.bounded_model = candidate.covers_plausible_set && candidate.valid &&
        candidate.action.recoverability == HistoryRecoverability::Recoverable &&
        finite_pl && action_model_valid;
    view.centre_only = !candidate.covers_plausible_set;
    view.has_valid_reference = candidate.valid;
    view.propagation_bound_available = finite_pl;
    const CandidateDecision handling = decideCandidateHandling(view);
    decision.candidate_dispositions[i] =
        static_cast<int>(handling.disposition);
    const bool reference_estimate = handling.disposition ==
        CandidateDisposition::UseInReferenceEstimate;
    if (candidate.valid && candidate.post_detector_passed && covers_all &&
        candidate.action.recoverability == HistoryRecoverability::Recoverable &&
        finite_pl && reference_estimate) {
      eligible.push_back(i);
    }
  }
  if (eligible.empty()) {
    decision.status = decision.plausible_hypotheses.size() > 1
        ? FdeStatus::AmbiguousUnavailable : FdeStatus::NoValidCandidate;
    decision.reason = "no valid action covers the complete plausible set";
    return decision;
  }
  std::stable_sort(eligible.begin(), eligible.end(), [&](std::size_t a, std::size_t b) {
    const auto& x = (*candidates)[a];
    const auto& y = (*candidates)[b];
    return std::make_tuple(x.action.exclusion_cardinality,
                           std::max(x.hpl_m, x.vpl_m),
                           -x.information_logdet, x.action.id.value()) <
           std::make_tuple(y.action.exclusion_cardinality,
                           std::max(y.hpl_m, y.vpl_m),
                           -y.information_logdet, y.action.id.value());
  });
  const std::size_t winner_index = eligible.front();
  const auto& winner = (*candidates)[winner_index];
  const bool winner_within_alert =
      winner.hpl_m <= risk.horizontal_alert_limit_m &&
      winner.vpl_m <= risk.vertical_alert_limit_m;
  // §8.5 / P0-04: pass EVERY original eligible action into the strict grouping
  // layer.  Plausible IDs are fault labels, never a shared failure event.  A
  // candidate may reuse another action's event only through the actual numeric
  // certificate/event/time/output identities and the triangle transfer.
  {
    std::vector<ActionGuarantee> guarantees;
    guarantees.reserve(eligible.size());
    // Each action's protected PL is conditioned on the complete represented
    // hypothesis family, not only on the labels that happened to be plausible
    // in this detector outcome.  Charging only plausible IDs would make the
    // nominal/no-alarm action event spuriously free and would reintroduce the
    // D06 label/event conflation through the amount rather than the key.
    const double per_action_epsilon = risk_audit.hypotheses;
    for (const auto index : eligible) {
      const auto& candidate = (*candidates)[index];
      ActionGuarantee guarantee;
      guarantee.action_id = candidate.action.id.value();
      if (protection_proof_ids) {
        const auto proof = protection_proof_ids->find(
            candidate.action.id.value());
        if (proof != protection_proof_ids->end()) {
          guarantee.reference_certificate_id = proof->second;
        }
      }
      guarantee.L_action_m = candidate.pl_xyz_m.allFinite()
          ? candidate.pl_xyz_m.cwiseAbs()
          : Eigen::Vector3d(candidate.hpl_m, candidate.hpl_m,
                            candidate.vpl_m).cwiseAbs();
      guarantee.L_reference_m.setZero();
      guarantee.p_reference_m.setZero();
      bool numeric_candidate = candidate.detector_certificate.valid &&
          guarantee.L_action_m.allFinite();
      if (candidate.window_view && candidate.state_increment.allFinite() &&
          candidate.window_view->protected_state_map.cols() ==
              candidate.state_increment.size()) {
        guarantee.p_action_m =
            candidate.window_view->protected_state_map *
            candidate.state_increment;
      } else {
        numeric_candidate = false;
      }
      // A candidate proof is not an independent common-reference sidecar.
      // Until such a sidecar supplies the same proof/p_ref/L_ref/event/time/
      // output for multiple actions, production actions are singletons.
      guarantee.shared_reference_certificate = false;
      guarantee.shared_accepted_event = false;
      guarantee.shared_time = false;
      guarantee.shared_output_quantity = false;
      guarantee.triangle_transfer_evidence = false;
      if (!numeric_candidate) guarantee.L_action_m.setZero();
      guarantee.epsilon_budget = per_action_epsilon;
      guarantees.push_back(std::move(guarantee));
    }
    const GuaranteeGroupResult groups =
        buildGuaranteeGroups(guarantees, risk.p_hmi_total);
    decision.selection_event_classes = groups.groups.size();
    decision.selection_charged_budget = groups.total_charged_budget;
    decision.selection_available_budget = risk.p_hmi_total;
    decision.selection_budget_ok = groups.valid;
    decision.selection_event_class_ids.clear();
    for (const auto& group : groups.groups) {
      decision.selection_event_class_ids.push_back(group.group_id);
    }
    std::sort(decision.selection_event_class_ids.begin(),
              decision.selection_event_class_ids.end());
    {
      std::ostringstream proof;
      proof << std::setprecision(17) << groups.total_charged_budget;
      for (const auto id : decision.selection_event_class_ids) {
        proof << '|' << id;
      }
      decision.selection_risk_proof_id = identityHash64(proof.str());
    }
    // Reconcile the union selection event with the allocation/miss/escape
    // ledger.  Hypothesis allocation is already charged once; only the excess
    // from disjoint action events is added as the selection term.
    CompleteRiskInputsV1 ledger_inputs;
    ledger_inputs.legacy.envelope_online = false;
    ledger_inputs.legacy.envelope_leaf_count = 0;
    ledger_inputs.legacy.selection_contract_frozen = true;
    ExactRisk exact_hypothesis_allocation = 0;
    for (const auto& hypothesis : hypotheses) {
      exact_hypothesis_allocation += ExactRisk(hypothesis.hmi_allocation);
    }
    const ExactRisk selection_extra = std::max(
        ExactRisk(0), ExactRisk(groups.total_charged_budget) -
            exact_hypothesis_allocation);
    ledger_inputs.selection_extra_bound = roundRiskUp(selection_extra);
    ledger_inputs.selection_bound_known = true;
    ledger_inputs.selection_bound_validated = true;
    ledger_inputs.model_formal_eligible = !risk.calibration_id.empty();
    CompleteRiskStatusV1 complete_status;
    const RiskLedger complete = buildRiskLedger(
        risk, hypotheses, ledger_inputs, &complete_status);
    risk_result->allocation_valid = complete_status.allocation_valid;
    risk_result->complete_bound_closes =
        complete_status.complete_bound_closes;
    risk_result->all_terms_validated = complete.all_terms_validated;
    risk_result->formal_eligible = complete.formal_eligible;
    risk_result->charged_total = complete.charged_total;
    risk_result->declared_total = complete.declared_total;
    risk_result->margin = complete.margin;
    risk_result->terms.clear();
    for (const auto& term : complete.terms) {
      if (term.status == RiskTermStatus::Validated) {
        ++risk_result->validated_terms;
      } else if (term.status == RiskTermStatus::AssumedUnvalidated) {
        ++risk_result->unvalidated_terms;
      } else {
        ++risk_result->not_implemented_terms;
      }
      if (!risk_result->terms.empty()) {
        risk_result->terms += ";";
      }
      risk_result->terms += term.id + "=" +
          (std::isfinite(term.value) ? std::to_string(term.value)
                                     : std::string("UNKNOWN")) +
          ":" + toString(term.status);
    }
    if (!groups.valid) {
      // The publishable set cannot be bounded: stay unavailable, never relax.
      decision.commit_allowed = false;
      decision.integrity_available = false;
      decision.status = FdeStatus::RiskBudgetInvalid;
      decision.reason = groups.reason;
    } else if (!risk_result->complete_bound_closes) {
      decision.commit_allowed = false;
      decision.integrity_available = false;
      decision.status = FdeStatus::RiskBudgetInvalid;
      decision.reason = complete.reason;
    } else if (!risk_result->all_terms_validated) {
      decision.commit_allowed = false;
      decision.integrity_available = false;
      decision.status = FdeStatus::RiskBudgetInvalid;
      decision.reason = complete.reason;
    } else {
      // Winner state is published only after the complete risk gate passes.
      (*candidates)[winner_index].selected = true;
      decision.selected_action = winner.action;
      decision.status = statusFor(winner.action);
      if (decision.plausible_hypotheses.size() > 1 &&
          winner.action.exclusion_cardinality > 1) {
        decision.status = FdeStatus::AmbiguousUnionExclusion;
      }
      decision.commit_allowed = true;
      decision.integrity_available = winner_within_alert;
      if (!winner_within_alert) {
        decision.reason = "post-FDE PL exceeds alert limit";
      }
    }
  }
  return decision;
}

}  // namespace uwb_imu_pl
