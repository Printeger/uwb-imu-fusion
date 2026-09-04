#include "uwb_imu_pl/integrity/fde_manager.hpp"

#include <algorithm>
#include <set>
#include <tuple>

namespace uwb_imu_pl {
namespace {

bool covers(const ExclusionAction& action, const FaultHypothesisV2& hypothesis) {
  for (const auto unit : hypothesis.units) {
    if (std::find(action.covered_units.begin(), action.covered_units.end(), unit) ==
        action.covered_units.end()) return false;
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
    const RiskBudgetV2& risk) const {
  FdeDecision decision;
  if (!candidates || !all_in.numerically_valid) {
    decision.status = FdeStatus::ModelInvalid;
    decision.reason = "joint detector or candidate set invalid";
    return decision;
  }
  double risk_sum = 3.0 * risk.nominal_axis_tail + risk.p_nm +
      risk.p_bridge_escape + risk.p_history_contamination + risk.p_model_escape;
  for (const auto& hypothesis : hypotheses) {
    risk_sum += hypothesis.hmi_allocation;
  }
  if (risk_sum > risk.p_hmi_total) {
    decision.status = FdeStatus::RiskBudgetInvalid;
    decision.reason = "V2 outcome risk budget does not close";
    return decision;
  }
  if (!all_in.passed) {
    for (const auto& item : evidence) {
      if (item.plausible) decision.plausible_hypotheses.push_back(item.hypothesis);
    }
    if (decision.plausible_hypotheses.empty()) {
      decision.status = FdeStatus::NoValidCandidate;
      decision.reason = "alarm has no risk-preserving plausible hypothesis";
      return decision;
    }
  }
  std::vector<std::size_t> eligible;
  for (std::size_t i = 0; i < candidates->size(); ++i) {
    auto& candidate = (*candidates)[i];
    bool covers_all = true;
    for (const auto id : decision.plausible_hypotheses) {
      const auto found = std::find_if(hypotheses.begin(), hypotheses.end(),
          [&](const FaultHypothesisV2& h) { return h.id == id; });
      if (found == hypotheses.end() || !covers(candidate.action, *found)) {
        covers_all = false;
        break;
      }
    }
    candidate.covers_plausible_set = covers_all;
    if (candidate.valid && candidate.post_detector_passed && covers_all &&
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
