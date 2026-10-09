#pragma once

#include "uwb_imu_pl/integrity/fde_manager.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace uwb_imu_pl {

struct RiskBudgetAudit {
  double nominal = 0.0;
  double p_nm = 0.0;
  double bridge = 0.0;
  double history = 0.0;
  double model = 0.0;
  double hypotheses = 0.0;
  double total = 0.0;
  double upper_bound = 0.0;
  double margin = 0.0;
  std::size_t hypothesis_count = 0;
  std::size_t single_fault_hypothesis_count = 0;
  std::size_t double_fault_hypothesis_count = 0;
  std::size_t other_cardinality_hypothesis_count = 0;
  std::uint32_t effective_max_cardinality = 0;
  bool inputs_valid = false;
  // Allocation precheck only; retained at the golden ABI location/name.
  bool valid = false;
};

// B3: every ledger term carries where its value comes from and whether that
// value is a validated bound.  A term is never silently zero: a zero with an
// unvalidated status is reported as such and recorded in the certificate.
enum class RiskTermStatus {
  Validated = 0,
  AssumedUnvalidated = 1,
  NotImplemented = 2,
};

const char* toString(RiskTermStatus status);

struct RiskLedgerTerm {
  std::string id;
  double value = 0.0;
  RiskTermStatus status = RiskTermStatus::NotImplemented;
  std::string source;
  std::string note;
};

// Complete HMI ledger.  Every finite term is charged whether or not its bound
// has completed qualification.  An unknown term is NaN, never zero, and makes
// complete_bound_closes false.
struct RiskLedger {
  std::vector<RiskLedgerTerm> terms;
  double budget = 0.0;
  double charged_total = 0.0;
  double declared_total = 0.0;
  double margin = 0.0;
  bool closes = false;
  bool inputs_valid = false;
  bool all_terms_validated = false;
  bool formal_eligible = false;
  std::size_t hypothesis_count = 0;
  std::string reason;
  const RiskLedgerTerm* find(const std::string& id) const;
};

struct RiskLedgerInputs {
  // Omitted-event mass: the manifest lists which event kinds are omitted; the
  // mass itself is not quantified yet, so the ledger reports the list and the
  // status instead of a silent zero.
  std::vector<std::string> omitted_event_set;
  // Envelope qualification: envelopes are offline by default, so the envelope
  // term is an explicit zero tied to the coverage certificate.
  bool envelope_online = false;
  std::size_t envelope_leaf_count = 0;
  // Selection is the extra union charge beyond the once-charged hypothesis
  // allocation.  It is reconciled in this same total budget.
  bool selection_contract_frozen = false;
};

// Versioned P0-04 input sidecar.  The pre-existing RiskLedgerInputs layout is
// unchanged so a golden-header client remains binary compatible.
struct CompleteRiskInputsV1 {
  RiskLedgerInputs legacy;
  // When omitted_scope_complete is true and omitted_event_set is empty, the
  // only canonical zero-proof representation is bound==0 with known and
  // validated both true.  Declared omitted events likewise require a complete
  // scope and a known finite non-negative bound.
  bool omitted_scope_complete = false;
  double omitted_event_bound = 0.0;
  bool omitted_event_bound_known = false;
  bool omitted_event_bound_validated = false;
  // Offline traversal with zero leaves is a zero proof only when the bound is
  // canonically represented as known, validated and exactly zero.
  double envelope_event_bound = 0.0;
  bool envelope_event_bound_known = false;
  bool envelope_event_bound_validated = false;
  bool bridge_escape_validated = false;
  bool history_escape_validated = false;
  bool model_escape_validated = false;
  double selection_extra_bound = 0.0;
  bool selection_bound_known = false;
  bool selection_bound_validated = false;
  bool model_formal_eligible = false;
};

// Evidence domain separates software/model conditions from hardware approval.
enum class RiskEvidenceDomainV1 { Missing, Structural, SimulationConditional, Deployment };
struct RiskTermEvidenceV1 {
  RiskEvidenceDomainV1 domain = RiskEvidenceDomainV1::Missing;
  std::string source;
};
struct BoundRiskContextV1 {
  std::uint64_t schema_version = 1;
  WindowId window_id;
  LinearizationVersion version;
  TimestampNs valid_from;
  TimestampNs valid_until;
  std::string scope_id;
  std::string manifest_id;
  std::string model_id;
  // Binds noise/prior/tail/AL and every represented hypothesis allocation.
  std::uint64_t risk_contract_id = 0;
  CompleteRiskInputsV1 inputs;
  std::map<std::string, RiskTermEvidenceV1> evidence;
};
std::uint64_t riskContractIdentityV1(
    const RiskBudgetV2& risk, const std::vector<FaultHypothesisV2>& hypotheses);
bool validateBoundRiskContextV1(
    const BoundRiskContextV1& bound, const FdeDecisionContextV4& expected,
    const DetectorResultV2& detector,
    const std::vector<CandidateEvaluation>& candidates,
    const RiskBudgetV2& risk, const std::vector<FaultHypothesisV2>& hypotheses,
    std::string* reason);

struct CompleteRiskStatusV1 {
  bool allocation_valid = false;
  bool complete_bound_closes = false;
};

RiskLedger buildRiskLedger(const RiskBudgetV2& risk,
                           const std::vector<FaultHypothesisV2>& hypotheses,
                           const RiskLedgerInputs& inputs = {});
RiskLedger buildRiskLedger(const RiskBudgetV2& risk,
                           const std::vector<FaultHypothesisV2>& hypotheses,
                           const CompleteRiskInputsV1& inputs,
                           CompleteRiskStatusV1* status);

// B3 (§5.9): per-axis tail split for one hypothesis.
//
//   L_{h,d} = s_{h,d} * sqrt(Lambda_h) + k_{h,d} * sigma_d
//   alpha_h = allocation_h / pi_h          (charged per-hypothesis tail)
//   alpha_{h,d} = alpha_h / axis_count     (equal split, union bound)
//   k_{h,d} = Phi^-1(1 - alpha_{h,d} / 2)
//   charge_h = pi_h * max(sum_d alpha_{h,d}, beta_h)
//
// The union bound over the axes is what makes the split necessary: charging
// alpha_h for the hypothesis while using alpha_h on every axis would
// understate the tails.  Invalid inputs are rejected, never clamped.
struct AxisTailSplit {
  int axis_count = 3;
  double allocation = 0.0;
  double prior_bound = 0.0;
  double beta = 0.0;
  double hypothesis_tail = 0.0;
  double axis_tail = 0.0;
  double charge = 0.0;
  bool valid = false;
  std::string reason;
};

AxisTailSplit axisTailSplit(double allocation, double prior_bound, double p_md,
                            double nominal_tail, int axis_count = 3);

RiskBudgetAudit auditRiskBudget(
    const RiskBudgetV2& risk,
    const std::vector<FaultHypothesisV2>& hypotheses);

// Equal binary64 allocation that is guaranteed not to exceed total when
// summed count times in the high-precision audit. Any rounding remainder stays
// explicitly unallocated.
double conservativeEqualRiskAllocation(double total, std::size_t count);

// Remaining hypothesis allocation represented as binary64 without allowing
// the exact sum of the fixed outcome terms to cross p_hmi_total.
double conservativeRemainingHypothesisRisk(const RiskBudgetV2& risk);

}  // namespace uwb_imu_pl
