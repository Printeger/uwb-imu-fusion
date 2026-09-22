#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/common/integrity_identity.hpp"
#include "uwb_imu_pl/integrity/fde_post_selection.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <map>
#include <sstream>
#include <string>
#include <tuple>

namespace uwb_imu_pl {
namespace {

// FNV-1a; decision-level event-class id (never a PL transfer certificate).
std::uint64_t eventClassId(const std::string& key) {
  std::uint64_t hash = 1469598103934665603ull;
  for (const unsigned char byte : key) {
    hash ^= byte;
    hash *= 1099511628211ull;
  }
  return hash == 0 ? 1 : hash;
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
  FdeDecision decision;
  if (!candidates || !all_in.numerically_valid) {
    decision.status = FdeStatus::ModelInvalid;
    decision.reason = "joint detector or candidate set invalid";
    return decision;
  }
  const RiskBudgetAudit risk_audit = auditRiskBudget(risk, hypotheses);
  if (!risk_audit.valid) {
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
    if (candidate.valid && candidate.post_detector_passed && covers_all &&
        candidate.action.recoverability == HistoryRecoverability::Recoverable &&
        std::isfinite(candidate.hpl_m) && std::isfinite(candidate.vpl_m)) {
      eligible.push_back(i);
    }
  }
  if (eligible.empty()) {
    decision.status = decision.plausible_hypotheses.size() > 1
        ? FdeStatus::AmbiguousUnavailable : FdeStatus::NoValidCandidate;
    decision.reason = "no valid action covers the complete plausible set";
    return decision;
  }
  // §8.3 wiring (C3): every candidate is passed through the disposition order
  // of fde_post_selection.  The mapping is the pipeline's own criterion -- a
  // candidate that covers the complete plausible set with a finite protected
  // reference estimate is the reference-estimate path, anything else can only
  // be a centre candidate or a diagnostic.  The disposition is recorded per
  // candidate; selection below still requires the reference-estimate path.
  decision.candidate_dispositions.assign(candidates->size(), 0);
  for (std::size_t i = 0; i < candidates->size(); ++i) {
    const auto& candidate = (*candidates)[i];
    // The mapping uses exactly the frozen eligibility criterion (finite
    // protected PL on a recovered history), so this view adds no new gate.
    const bool finite_pl = std::isfinite(candidate.hpl_m) &&
        std::isfinite(candidate.vpl_m);
    StructuredCandidate view;
    view.bounded_model = candidate.covers_plausible_set && candidate.valid &&
        candidate.action.recoverability == HistoryRecoverability::Recoverable &&
        finite_pl;
    view.centre_only = !candidate.covers_plausible_set;
    view.has_valid_reference = candidate.valid;
    view.propagation_bound_available = finite_pl;
    const CandidateDecision handling = decideCandidateHandling(view);
    decision.candidate_dispositions[i] =
        static_cast<int>(handling.disposition);
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
  auto& winner = (*candidates)[eligible.front()];
  winner.selected = true;
  decision.selected_action = winner.action;
  decision.status = statusFor(winner.action);
  if (decision.plausible_hypotheses.size() > 1 &&
      winner.action.exclusion_cardinality > 1) {
    decision.status = FdeStatus::AmbiguousUnionExclusion;
  }
  decision.commit_allowed = true;
  decision.integrity_available = winner.hpl_m <= risk.horizontal_alert_limit_m &&
      winner.vpl_m <= risk.vertical_alert_limit_m;
  if (!decision.integrity_available) decision.reason = "post-FDE PL exceeds alert limit";
  // §8.5 wiring (C3): charge the union of everything that may be published.
  // Eligible actions that cover the same plausible-hypothesis union share one
  // failure event (they are alternative protections of one reference event at
  // one epoch), so they collapse into one event class; a different union is a
  // different event and is charged on its own.  No triangle-protection-level
  // transfer is claimed here (triangle_transfer_evidence stays false), which is
  // exactly why the module keeps every class charge separate.
  {
    std::map<std::string, ActionGuarantee> by_class;
    for (const auto index : eligible) {
      const auto& candidate = (*candidates)[index];
      std::string key;
      double epsilon = 0.0;
      for (const auto id : decision.plausible_hypotheses) {
        const auto found = hypothesis_index.find(id);
        if (found == hypothesis_index.end()) continue;
        epsilon += std::max(0.0, found->second->hmi_allocation);
        key += std::to_string(id.value());
        key += ',';
      }
      ActionGuarantee guarantee;
      guarantee.action_id = candidate.action.id.value();
      guarantee.reference_certificate_id = eventClassId(key);
      guarantee.shared_reference_certificate = true;
      guarantee.shared_accepted_event = true;
      guarantee.shared_time = true;
      guarantee.shared_output_quantity = true;
      guarantee.triangle_transfer_evidence = false;
      guarantee.epsilon_budget = epsilon;
      const auto found = by_class.find(key);
      if (found == by_class.end()) {
        by_class.emplace(key, guarantee);
      } else if (guarantee.epsilon_budget > found->second.epsilon_budget) {
        found->second.epsilon_budget = guarantee.epsilon_budget;
      }
    }
    std::vector<ActionGuarantee> classes;
    classes.reserve(by_class.size());
    for (const auto& item : by_class) classes.push_back(item.second);
    const GuaranteeGroupResult groups =
        buildGuaranteeGroups(classes, risk.p_hmi_total);
    decision.selection_event_classes = classes.size();
    decision.selection_charged_budget = groups.total_charged_budget;
    decision.selection_available_budget = risk.p_hmi_total;
    decision.selection_budget_ok = groups.valid;
    // Export the charged class identity: the ids come from the same
    // eventClassId() derivation that minted the guarantee groups, and the
    // proof id is the documented FNV-1a binding of (charged budget, ids).
    decision.selection_event_class_ids.clear();
    for (const auto& item : by_class) {
      decision.selection_event_class_ids.push_back(
          item.second.reference_certificate_id);
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
    if (!groups.valid) {
      // The publishable set cannot be bounded: stay unavailable, never relax.
      decision.commit_allowed = false;
      decision.integrity_available = false;
      decision.reason = groups.reason;
    }
  }
  return decision;
}

}  // namespace uwb_imu_pl
