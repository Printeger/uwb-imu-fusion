#pragma once

#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"

#include <optional>
#include <map>

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
  // C3 wiring (§8.4/§8.5/§8.3), all additive and fail-closed:
  // §8.4 structured evidence pool of the plausible hypotheses.
  bool profile_pool_valid = false;
  bool profile_pool_comparable = false;
  bool profile_pool_ranked = false;
  bool profile_pool_mixed_units = false;
  bool profile_pool_mixed_dimensions = false;
  std::vector<double> plausible_profile_j;  // same order as above
  // §8.5 union charge over the publishable action set (event classes).
  bool selection_budget_ok = false;
  double selection_charged_budget = 0.0;
  double selection_available_budget = 0.0;
  std::size_t selection_event_classes = 0;
  // Identity of the charged event classes (C4 diagnostics v16 / publication
  // risk proof): the class ids actually charged, sorted, and a derived id that
  // the publication gate binds as the W1 risk proof identity.  A decision with
  // no charged class carries id 0 and the gate reports it as missing.
  std::vector<std::uint64_t> selection_event_class_ids;
  std::uint64_t selection_risk_proof_id = 0;
  // §8.3 disposition per candidate, parallel to the candidate vector.
  std::vector<int> candidate_dispositions;
};

struct FdeRiskDecisionV1 {
  bool allocation_valid = false;
  bool complete_bound_closes = false;
  bool all_terms_validated = false;
  bool formal_eligible = false;
  double charged_total = 0.0;
  double declared_total = 0.0;
  double margin = 0.0;
  std::size_t validated_terms = 0;
  std::size_t unvalidated_terms = 0;
  std::size_t not_implemented_terms = 0;
  std::string terms;
};

struct FdeDecisionContextV1 {
  const std::map<std::uint64_t, std::uint64_t>* protection_proof_ids = nullptr;
  FdeRiskDecisionV1* risk_result = nullptr;
};

// P0-06 additive context.  V1 remains layout-stable for existing callers.
struct FdeDecisionContextV2 {
  FdeDecisionContextV1 v1;
  const ActionSearchCensusV1* action_search = nullptr;
  const GeneratedActionSnapshotV1* trusted_generated_actions = nullptr;
  std::size_t max_evaluated_actions = 0;
  ActionSearchLifecycleV1 action_search_lifecycle =
      ActionSearchLifecycleV1::ReadyForEvaluation;
};

// P1-06 additive compact-action consumer.  V1/V2 layouts and overloads remain
// unchanged.  The lease is the independent source used to reconstruct exact
// action semantics at the FDE boundary.
struct FdeDecisionContextV3 {
  FdeDecisionContextV1 v1;
  const AttemptActionLeaseV3* action_lease = nullptr;
};

class FdeManager {
 public:
  FdeDecision decide(const DetectorResultV2& all_in,
                     const std::vector<FaultHypothesisV2>& hypotheses,
                     const std::vector<FaultModeEvidence>& evidence,
                     std::vector<CandidateEvaluation>* candidates,
                     const std::vector<FactorGroupId>& mandatory_groups,
                     const RiskBudgetV2& risk) const;
  FdeDecision decide(const DetectorResultV2& all_in,
                     const std::vector<FaultHypothesisV2>& hypotheses,
                     const std::vector<FaultModeEvidence>& evidence,
                     std::vector<CandidateEvaluation>* candidates,
                     const std::vector<FactorGroupId>& mandatory_groups,
                     const RiskBudgetV2& risk,
                     const FdeDecisionContextV1* context) const;
  FdeDecision decide(const DetectorResultV2& all_in,
                     const std::vector<FaultHypothesisV2>& hypotheses,
                     const std::vector<FaultModeEvidence>& evidence,
                     std::vector<CandidateEvaluation>* candidates,
                     const std::vector<FactorGroupId>& mandatory_groups,
                     const RiskBudgetV2& risk,
                     const FdeDecisionContextV2* context) const;
  FdeDecision decide(const DetectorResultV2& all_in,
                     const std::vector<FaultHypothesisV2>& hypotheses,
                     const std::vector<FaultModeEvidence>& evidence,
                     std::vector<CandidateEvaluation>* candidates,
                     const std::vector<FactorGroupId>& mandatory_groups,
                     const RiskBudgetV2& risk,
                     const FdeDecisionContextV3* context) const;
};

}  // namespace uwb_imu_pl
