#pragma once
#include "range_position_bound.hpp"
#include "uwb_imu_pl/integrity/fde_post_selection.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"
#include <tuple>

namespace uwb_imu_pl::research {
// Independent consumer validation applies to KEEP and successful IMU decisions
// as well as the additional shared-reference UWB selector. Logical proof IDs
// are only indexes; actual source and served numerical semantics are checked.
inline void verifyRangeOutputs(const RangePositionSet& served,const UwbBatch& raw,
    const Eigen::Vector3d& origin,const LinearizedIntegrityWindow& window,
    const RiskBudgetV2& risk,const std::vector<FaultHypothesisV2>& hypotheses,
    const BoundRiskContextV1& bound,const std::vector<CandidateEvaluation>& candidates) {
  if(bound.model_id!="simulation-only/deterministic-range-union-v1" ||
      bound.scope_id!="simulation-only/one-episode-v1" || bound.manifest_id!="simulation-only/raw-batch-v1" ||
      bound.inputs.model_formal_eligible || bound.risk_contract_id!=riskContractIdentityV1(risk,hypotheses) ||
      raw.timestamp!=bound.valid_from || bound.valid_until!=bound.valid_from || bound.window_id!=window.id || !(bound.version==window.version))
    throw std::runtime_error("research range consumer contract/source mismatch");
  for(const auto* term:{"omitted","envelope","bridge","history","model"}) {
    const auto evidence=bound.evidence.find(term);
    if(evidence==bound.evidence.end() || evidence->second.domain!=RiskEvidenceDomainV1::SimulationConditional ||
        evidence->second.source!=bound.model_id+"|"+term)
      throw std::runtime_error("research range consumer missing conditional evidence domain");
  }
  const auto actual=rangePositionSet(raw);
  if(!validRangeSet(served) || !validRangeSet(actual) || actual.centres.size()!=served.centres.size())
    throw std::runtime_error("research range consumer malformed source proof");
  for(std::size_t i=0;i<actual.centres.size();++i)
    if((actual.centres[i].array()!=served.centres[i].array()).any() || actual.radius[i]!=served.radius[i])
      throw std::runtime_error("research range consumer full source semantics mismatch");
  for(const auto& c:candidates)if(c.valid && c.post_detector_passed && c.pl_xyz_m.allFinite()) {
    const Eigen::Vector3d expected=relativeTransferBound(actual,origin,window.protected_state_map*c.state_increment);
    if((expected.array()!=c.pl_xyz_m.array()).any() || c.hpl_m!=expected.head<2>().norm() || c.vpl_m!=expected.z())
      throw std::runtime_error("research range consumer served PL mismatch");
  }
}
// Additional research selector contract. Production FdeManager remains
// unchanged. It has already checked search, evidence, coverage and original
// candidate eligibility; this function can only repair a RISK_BUDGET_INVALID
// result using a separately verified deterministic common position set.
inline FdeDecision commonRangeReferenceSelection(FdeDecision previous,
    const RangePositionSet& position_set,const UwbBatch& raw_batch,const Eigen::Vector3d& frozen_origin,
    const LinearizedIntegrityWindow& window,
    const RiskBudgetV2& risk,const std::vector<FaultHypothesisV2>& hypotheses,
    const BoundRiskContextV1& bound,std::vector<CandidateEvaluation>& candidates,
    RiskLedger& ledger) {
  if(previous.status!=FdeStatus::RiskBudgetInvalid || !position_set.valid)return previous;
  if(bound.model_id!="simulation-only/deterministic-range-union-v1" ||
      bound.scope_id!="simulation-only/one-episode-v1" || bound.manifest_id!="simulation-only/raw-batch-v1" ||
      bound.inputs.model_formal_eligible || bound.risk_contract_id!=riskContractIdentityV1(risk,hypotheses))return previous;
  verifyRangeOutputs(position_set,raw_batch,frozen_origin,window,risk,hypotheses,bound,candidates);
  const auto rebuilt=rangePositionSet(raw_batch);
  if(!rebuilt.valid || position_set.radius.size()!=position_set.centres.size() || rebuilt.centres.size()!=position_set.centres.size() ||
      raw_batch.timestamp!=bound.valid_from || bound.valid_until!=bound.valid_from || bound.window_id!=window.id || !(bound.version==window.version))
    throw std::runtime_error("research common reference source/time mismatch");
  for(std::size_t i=0;i<rebuilt.centres.size();++i)
    if((rebuilt.centres[i].array()!=position_set.centres[i].array()).any() || rebuilt.radius[i]!=position_set.radius[i])
      throw std::runtime_error("research common reference full numerical mismatch");
  std::vector<std::size_t> eligible;std::vector<ActionGuarantee> guarantees;
  const double epsilon=auditRiskBudget(risk,hypotheses).hypotheses;
  const Eigen::Vector3d reference=position_set.centres.front();
  const Eigen::Vector3d reference_bound=transferBound(position_set,reference);
  for(std::size_t i=0;i<candidates.size();++i) {
    const auto& c=candidates[i];
    if(!c.valid || !c.post_detector_passed || !c.covers_plausible_set ||
        c.action.recoverability!=HistoryRecoverability::Recoverable ||
        (c.action.bridge_mode!=BridgeMode::None && !c.action.model_error_validated) ||
        !c.pl_xyz_m.allFinite())continue;
    const Eigen::Vector3d shift=window.protected_state_map*c.state_increment;
    const Eigen::Vector3d expected=relativeTransferBound(position_set,frozen_origin,shift);
    // Full served numerical semantics: modified external bound/centre cannot
    // borrow the source proof's index or group id.
    if((expected.array()!=c.pl_xyz_m.array()).any() || c.hpl_m!=expected.head<2>().norm() || c.vpl_m!=expected.z())
      throw std::runtime_error("research external range PL/reference mismatch");
    ActionGuarantee g;g.action_id=c.action.id.value();g.reference_certificate_id=window.id.value()+1;
    g.p_action_m=shift;g.p_reference_m=reference-frozen_origin;
    g.L_action_m=c.pl_xyz_m;g.L_reference_m=reference_bound;g.epsilon_budget=epsilon;
    g.shared_reference_certificate=true;g.shared_accepted_event=true;g.shared_time=true;
    g.shared_output_quantity=true;g.triangle_transfer_evidence=true;
    eligible.push_back(i);guarantees.push_back(g);
  }
  if(eligible.empty())return previous;
  // Production buildGuaranteeGroups deliberately ignores caller shared flags.
  // Do not alter it or pretend those flags are a producer proof. This isolated
  // contract instead reconstructs the actual deterministic raw-range union,
  // verifies EVERY output's complete triangle numerics above, then bounds one
  // common failure event. The original strict singleton failure is retained.
  GuaranteeGroupResult groups;groups.valid=true;groups.total_charged_budget=epsilon;
  groups.shared_groups=1;GuaranteeGroup group;group.group_id=window.id.value()+1;
  group.reference_certificate_id=group.group_id;group.charged_budget=epsilon;
  for(const auto& g:guarantees)group.action_ids.push_back(g.action_id);
  groups.groups.push_back(std::move(group));
  auto inputs=bound.inputs;inputs.legacy.selection_contract_frozen=true;
  // Selector owns the event union. Nothing subtracts miss charges or source
  // priors. Shared output reference is a deterministic raw-data proof.
  inputs.selection_extra_bound=std::max(0.,groups.total_charged_budget-epsilon);
  inputs.selection_bound_known=inputs.selection_bound_validated=true;
  ledger=buildRiskLedger(risk,hypotheses,inputs,nullptr);
  for(auto& term:ledger.terms) {
    const auto evidence=bound.evidence.find(term.id);
    if(evidence!=bound.evidence.end())term.source=evidence->second.source+";domain=2";
  }
  if(!ledger.closes || !ledger.all_terms_validated)return previous;
  std::stable_sort(eligible.begin(),eligible.end(),[&](auto a,auto b) {
    const auto& x=candidates[a];const auto& y=candidates[b];
    return std::make_tuple(x.action.exclusion_cardinality,std::max(x.hpl_m,x.vpl_m),-x.information_logdet,x.action.id.value())<
        std::make_tuple(y.action.exclusion_cardinality,std::max(y.hpl_m,y.vpl_m),-y.information_logdet,y.action.id.value());
  });
  auto& winner=candidates[eligible.front()];
  previous.selected_action=winner.action;winner.selected=true;
  previous.commit_allowed=true;previous.integrity_available=winner.hpl_m<=risk.horizontal_alert_limit_m && winner.vpl_m<=risk.vertical_alert_limit_m;
  previous.status=winner.action.bridge_mode==BridgeMode::None?FdeStatus::SuccessUwbExclusion:FdeStatus::SuccessImuExclusionGenericBridge;
  previous.reason="CONDITIONAL simulation-only/common-range-reference/v1";
  previous.selection_event_classes=groups.groups.size();previous.selection_charged_budget=groups.total_charged_budget;
  previous.selection_budget_ok=true;return previous;
}
} // namespace
