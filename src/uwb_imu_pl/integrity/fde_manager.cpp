#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <algorithm>
#include <set>
#include <map>
#include <tuple>

namespace uwb_imu_pl {
namespace {

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
  return decision;
}

}  // namespace uwb_imu_pl
