#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/fde_post_selection.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <cstddef>
#include <iostream>
#include <vector>

int main() {
  using namespace uwb_imu_pl;
  std::cout << "FdeDecision " << sizeof(FdeDecision) << ' '
            << offsetof(FdeDecision, candidate_dispositions) << '\n';
  std::cout << "ActionGuarantee " << sizeof(ActionGuarantee) << ' '
            << offsetof(ActionGuarantee, epsilon_budget) << '\n';
  std::cout << "RiskBudgetAudit " << sizeof(RiskBudgetAudit) << ' '
            << offsetof(RiskBudgetAudit, valid) << '\n';
  std::cout << "RiskLedger " << sizeof(RiskLedger) << ' '
            << offsetof(RiskLedger, reason) << '\n';
  std::cout << "RiskLedgerInputs " << sizeof(RiskLedgerInputs) << ' '
            << offsetof(RiskLedgerInputs, selection_contract_frozen) << '\n';

  DetectorResultV2 detector;
  std::vector<FaultHypothesisV2> hypotheses;
  std::vector<FaultModeEvidence> evidence;
  std::vector<CandidateEvaluation> candidates;
  const FdeDecision decision = FdeManager().decide(
      detector, hypotheses, evidence, &candidates, {}, RiskBudgetV2{});
  std::cout << "decide6 " << static_cast<int>(decision.status) << ' '
            << decision.commit_allowed << ' '
            << decision.selected_action.has_value() << '\n';
  return 0;
}
