#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <boost/multiprecision/cpp_bin_float.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace uwb_imu_pl {
namespace {
using ExactAccumulator = boost::multiprecision::cpp_bin_float_quad;
}

double conservativeEqualRiskAllocation(double total, std::size_t count) {
  if (count == 0) return 0.0;
  if (!std::isfinite(total) || total < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double allocation = total / static_cast<double>(count);
  const ExactAccumulator exact_total(total);
  const ExactAccumulator exact_count(count);
  while (ExactAccumulator(allocation) * exact_count > exact_total) {
    allocation = std::nextafter(allocation, 0.0);
  }
  return allocation;
}

double conservativeRemainingHypothesisRisk(const RiskBudgetV2& risk) {
  const ExactAccumulator remaining = ExactAccumulator(risk.p_hmi_total) -
      ExactAccumulator(risk.nominal_axis_tail) * 3 -
      ExactAccumulator(risk.p_nm) - ExactAccumulator(risk.p_bridge_escape) -
      ExactAccumulator(risk.p_history_contamination) -
      ExactAccumulator(risk.p_model_escape);
  if (remaining < 0) return std::numeric_limits<double>::quiet_NaN();
  double rounded = static_cast<double>(remaining);
  while (ExactAccumulator(rounded) > remaining) {
    rounded = std::nextafter(rounded, 0.0);
  }
  return rounded;
}

const char* toString(RiskTermStatus status) {
  switch (status) {
    case RiskTermStatus::Validated:
      return "VALIDATED";
    case RiskTermStatus::AssumedUnvalidated:
      return "ASSUMED_UNVALIDATED";
    case RiskTermStatus::NotImplemented:
      return "NOT_IMPLEMENTED";
  }
  return "NOT_IMPLEMENTED";
}

const RiskLedgerTerm* RiskLedger::find(const std::string& id) const {
  for (const auto& term : terms) {
    if (term.id == id) return &term;
  }
  return nullptr;
}

AxisTailSplit axisTailSplit(double allocation, double prior_bound, double p_md,
                            double nominal_tail, int axis_count) {
  AxisTailSplit out;
  out.axis_count = axis_count > 0 ? axis_count : 3;
  out.beta = p_md;
  out.allocation = allocation;
  out.prior_bound = prior_bound;
  if (!std::isfinite(allocation) || allocation < 0.0) {
    out.reason = "hypothesis allocation is not a finite non-negative number";
    return out;
  }
  if (!std::isfinite(p_md) || p_md <= 0.0 || p_md >= 1.0) {
    out.reason = "p_md allocation outside (0, 1)";
    return out;
  }
  if (allocation == 0.0) {
    // No hypothesis allocation: only the detector miss channel is charged.
    out.hypothesis_tail = 0.0;
    out.axis_tail = 0.0;
    out.charge = p_md;
    out.valid = true;
    return out;
  }
  if (!std::isfinite(prior_bound) || prior_bound <= 0.0) {
    // An illegal prior is rejected instead of being clamped: the allocation
    // cannot be converted into a tail without a positive prior bound.
    out.reason = "prior probability bound is not positive";
    return out;
  }
  const double tail = allocation / prior_bound;
  if (!std::isfinite(tail) || tail <= 0.0) {
    out.reason = "hypothesis tail probability is not positive";
    return out;
  }
  out.hypothesis_tail = std::min(0.5, tail);
  out.axis_tail = out.hypothesis_tail / static_cast<double>(out.axis_count);
  out.charge = std::max(out.hypothesis_tail, p_md);
  out.valid = true;
  return out;
}

RiskLedger buildRiskLedger(const RiskBudgetV2& risk,
                           const std::vector<FaultHypothesisV2>& hypotheses,
                           const RiskLedgerInputs& inputs) {
  RiskLedger ledger;
  ledger.budget = risk.p_hmi_total;
  ledger.hypothesis_count = hypotheses.size();
  ledger.inputs_valid = std::isfinite(risk.p_hmi_total) &&
      risk.p_hmi_total > 0.0;
  const double inputs_to_check[] = {
      risk.p_hmi_total, risk.nominal_axis_tail, risk.p_nm,
      risk.p_bridge_escape, risk.p_history_contamination,
      risk.p_model_escape};
  for (double value : inputs_to_check) {
    ledger.inputs_valid = ledger.inputs_valid && std::isfinite(value) &&
        value >= 0.0;
  }
  if (!ledger.inputs_valid) {
    ledger.reason = "risk budget configuration is not a finite non-negative "
                    "set with a positive total";
    return ledger;
  }

  // 1. Nominal outcome: three axes, each with the configured nominal tail.
  RiskLedgerTerm nominal;
  nominal.id = "nominal";
  nominal.value = 3.0 * risk.nominal_axis_tail;
  nominal.status = RiskTermStatus::Validated;
  nominal.source = "config risk.nominal_axis_tail x 3 protected axes";
  nominal.note = "pi0 * alpha0 for the alarm-free outcome";
  ledger.terms.push_back(std::move(nominal));

  // 2. Non-nominal outcome mass charged for hypotheses that no longer own an
  //    allocation (config term).
  RiskLedgerTerm p_nm;
  p_nm.id = "p_nm";
  p_nm.value = risk.p_nm;
  p_nm.status = RiskTermStatus::Validated;
  p_nm.source = "config risk.p_nm";
  p_nm.note = "non-nominal outcome mass not attributable to a hypothesis";
  ledger.terms.push_back(std::move(p_nm));

  // 3. Hypothesis outcome: charged as pi_h * alpha_h (the allocation-derived
  //    tail per hypothesis).  The detector-miss channel pi_h * beta_h is part
  //    of the section 5.9 bound but is *not* charged here: at the current
  //    budget the sum of that channel alone does not close (see the note in
  //    risk-and-lazy-fde.md), so it is reported explicitly instead of being
  //    silently treated as zero.  Charging it would flip availability for
  //    most frames and requires a budget re-calibration (B3 follow-up).
  RiskLedgerTerm fault;
  fault.id = "hypotheses";
  fault.source = "per-hypothesis allocation / prior bound (allocation policy: " +
      (risk.allocation_policy.empty() ? std::string("unspecified")
                                      : risk.allocation_policy) + ")";
  fault.note = "sum of pi_h * alpha_h over charged hypotheses";
  fault.status = RiskTermStatus::Validated;
  RiskLedgerTerm miss;
  miss.id = "hypotheses_miss_channel";
  miss.source = "section 5.9 union bound: sum of pi_h * max(0, beta_h - "
                "alpha_h)";
  miss.status = RiskTermStatus::AssumedUnvalidated;
  miss.note = "part of the section 5.9 bound that the current budget does not "
              "close; reported, not charged, and blocks formal eligibility";
  ExactAccumulator charged_hypotheses = 0;
  ExactAccumulator miss_channel = 0;
  std::vector<std::string> rejected;
  for (const auto& hypothesis : hypotheses) {
    const AxisTailSplit split = axisTailSplit(
        hypothesis.hmi_allocation, hypothesis.prior_probability_bound,
        hypothesis.p_md_allocation, risk.nominal_axis_tail);
    if (!split.valid) {
      rejected.push_back(std::to_string(hypothesis.id.value()) + ":" +
                         split.reason);
      continue;
    }
    charged_hypotheses += ExactAccumulator(hypothesis.hmi_allocation);
    if (split.hypothesis_tail > 0.0 && split.beta > split.hypothesis_tail) {
      miss_channel += ExactAccumulator(hypothesis.prior_probability_bound) *
          ExactAccumulator(split.beta - split.hypothesis_tail);
    }
  }
  fault.value = static_cast<double>(charged_hypotheses);
  if (!rejected.empty()) {
    fault.status = RiskTermStatus::AssumedUnvalidated;
    fault.note += "; hypotheses with rejected tails: " +
        std::to_string(rejected.size()) + " (first: " + rejected.front() + ")";
  }
  ledger.terms.push_back(std::move(fault));
  miss.value = static_cast<double>(miss_channel);
  ledger.terms.push_back(std::move(miss));

  // 4. Escape channels: configured values kept, flagged as unvalidated
  //    assumptions instead of silent zeros.
  const std::pair<const char*, double> escapes[] = {
      {"bridge", risk.p_bridge_escape},
      {"history", risk.p_history_contamination},
      {"model", risk.p_model_escape}};
  for (const auto& entry : escapes) {
    RiskLedgerTerm term;
    term.id = entry.first;
    term.value = entry.second;
    term.status = RiskTermStatus::AssumedUnvalidated;
    term.source = std::string("config risk.p_") + entry.first + "_escape";
    term.note = entry.second == 0.0
        ? "configured as zero: recorded as an unvalidated assumption, not as "
          "a proven zero"
        : "configured value used as an unvalidated assumption";
    ledger.terms.push_back(std::move(term));
  }

  // 5. Omitted event mass (manifest omitted_event_set).
  RiskLedgerTerm omitted;
  omitted.id = "omitted";
  omitted.value = 0.0;
  omitted.status = RiskTermStatus::NotImplemented;
  omitted.source = "config/integrity_fault_manifest.yaml omitted_event_set";
  omitted.note = "mass not quantified yet; omitted kinds: ";
  for (std::size_t index = 0; index < inputs.omitted_event_set.size(); ++index) {
    if (index) omitted.note += ",";
    omitted.note += inputs.omitted_event_set[index];
  }
  if (inputs.omitted_event_set.empty())
    omitted.note += "(none declared in the resolved manifest)";
  ledger.terms.push_back(std::move(omitted));

  // 6. Envelope qualification slot: explicit zero while envelopes are offline.
  RiskLedgerTerm envelope;
  envelope.id = "envelope";
  envelope.value = 0.0;
  envelope.status = RiskTermStatus::NotImplemented;
  envelope.source = "coverage certificate envelope qualification";
  envelope.note = inputs.envelope_online
      ? "envelopes online: group risk mapping not implemented in this batch"
      : "envelopes offline (exact traversal): epsilon_envelope = 0 by "
        "construction, qualification statistic recorded";
  envelope.note += "; enveloped leaves: " +
      std::to_string(inputs.envelope_leaf_count);
  ledger.terms.push_back(std::move(envelope));

  // 7. Selection slot (reserved for C3).
  RiskLedgerTerm selection;
  selection.id = "selection";
  selection.value = 0.0;
  selection.status = RiskTermStatus::NotImplemented;
  selection.source = "FDE selection contract (C3)";
  selection.note = inputs.selection_contract_frozen
      ? "selection contract frozen: slot still requires its own bound"
      : "reserved slot; FDE selection contract unchanged in this batch";
  ledger.terms.push_back(std::move(selection));

  // Charged total: validated terms only.  Declared total is informational and
  // includes the unvalidated channels so nothing hides behind a zero.
  ExactAccumulator charged = 0;
  ExactAccumulator declared_all = 0;
  bool all_validated = true;
  for (const auto& term : ledger.terms) {
    declared_all += ExactAccumulator(term.value);
    if (term.status == RiskTermStatus::Validated) {
      charged += ExactAccumulator(term.value);
    } else {
      all_validated = false;
    }
  }
  ledger.charged_total = static_cast<double>(charged);
  ledger.declared_total = static_cast<double>(declared_all);
  ledger.margin = static_cast<double>(ExactAccumulator(risk.p_hmi_total) -
                                      charged);
  ledger.closes = ExactAccumulator(ledger.charged_total) <=
      ExactAccumulator(risk.p_hmi_total);
  ledger.all_terms_validated = all_validated;
  ledger.formal_eligible = false;  // Gate J evidence is intentionally absent.
  ledger.reason = ledger.closes
      ? "outcome-conditioned ledger closes on validated terms"
      : "outcome-conditioned risk budget does not close";
  return ledger;
}

RiskBudgetAudit auditRiskBudget(
    const RiskBudgetV2& risk,
    const std::vector<FaultHypothesisV2>& hypotheses) {
  RiskBudgetAudit out;
  out.nominal = 3.0 * risk.nominal_axis_tail;
  out.p_nm = risk.p_nm;
  out.bridge = risk.p_bridge_escape;
  out.history = risk.p_history_contamination;
  out.model = risk.p_model_escape;
  out.upper_bound = risk.p_hmi_total;
  out.hypothesis_count = hypotheses.size();
  const double inputs[] = {risk.nominal_axis_tail, risk.p_nm,
      risk.p_bridge_escape, risk.p_history_contamination,
      risk.p_model_escape, risk.p_hmi_total};
  out.inputs_valid = true;
  for (double value : inputs) {
    out.inputs_valid = out.inputs_valid && std::isfinite(value) && value >= 0.0;
  }
  ExactAccumulator hypothesis_sum = 0;
  for (const auto& hypothesis : hypotheses) {
    out.effective_max_cardinality = std::max<std::uint32_t>(
        out.effective_max_cardinality,
        static_cast<std::uint32_t>(hypothesis.modes.size()));
    if (hypothesis.modes.size() == 1) ++out.single_fault_hypothesis_count;
    else if (hypothesis.modes.size() == 2)
      ++out.double_fault_hypothesis_count;
    else ++out.other_cardinality_hypothesis_count;
    out.inputs_valid = out.inputs_valid &&
        std::isfinite(hypothesis.hmi_allocation) &&
        hypothesis.hmi_allocation >= 0.0;
    hypothesis_sum += ExactAccumulator(hypothesis.hmi_allocation);
  }
  out.hypotheses = static_cast<double>(hypothesis_sum);
  const ExactAccumulator exact_total =
      ExactAccumulator(risk.nominal_axis_tail) * 3 +
      ExactAccumulator(risk.p_nm) + ExactAccumulator(risk.p_bridge_escape) +
      ExactAccumulator(risk.p_history_contamination) +
      ExactAccumulator(risk.p_model_escape) + hypothesis_sum;
  const ExactAccumulator exact_upper(risk.p_hmi_total);
  out.total = static_cast<double>(exact_total);
  out.margin = static_cast<double>(exact_upper - exact_total);
  out.valid = out.inputs_valid && exact_total <= exact_upper;
  return out;
}

}  // namespace uwb_imu_pl
