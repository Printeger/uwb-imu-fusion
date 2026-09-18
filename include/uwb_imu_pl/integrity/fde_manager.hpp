#pragma once

#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"

#include <optional>

namespace uwb_imu_pl {

struct RiskBudgetV2 {
  double p_hmi_total = 4e-5;
  double nominal_axis_tail = 1e-5;
  double p_nm = 1e-7;
  double p_bridge_escape = 0.0;
  double p_history_contamination = 0.0;
  double p_model_escape = 0.0;
  double horizontal_alert_limit_m = 2.0;
  double vertical_alert_limit_m = 3.0;
  std::string allocation_policy = "prior_weighted_outcome_conditioned";
  std::string calibration_id;
};

struct FdeDecision {
  FdeStatus status = FdeStatus::NotTriggered;
  std::optional<ExclusionAction> selected_action;
  std::vector<HypothesisId> plausible_hypotheses;
  std::vector<FactorGroupId> mandatory_exclusion_groups;
  bool commit_allowed = false;
  bool integrity_available = false;
  std::string reason;
};

class FdeManager {
 public:
  FdeDecision decide(const DetectorResultV2& all_in,
                     const std::vector<FaultHypothesisV2>& hypotheses,
                     const std::vector<FaultModeEvidence>& evidence,
                     std::vector<CandidateEvaluation>* candidates,
                     const std::vector<FactorGroupId>& mandatory_groups,
                     const RiskBudgetV2& risk) const;
};

}  // namespace uwb_imu_pl
