#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <boost/multiprecision/cpp_bin_float.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace uwb_imu_pl {
namespace {
using ExactAccumulator = boost::multiprecision::cpp_bin_float_quad;

double roundUp(const ExactAccumulator& value) {
  double rounded = static_cast<double>(value);
  while (std::isfinite(rounded) && ExactAccumulator(rounded) < value) {
    rounded = std::nextafter(rounded,
                             std::numeric_limits<double>::infinity());
  }
  return rounded;
}

double roundDown(const ExactAccumulator& value) {
  double rounded = static_cast<double>(value);
  while (std::isfinite(rounded) && ExactAccumulator(rounded) > value) {
    rounded = std::nextafter(rounded,
                             -std::numeric_limits<double>::infinity());
  }
  return rounded;
}
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
  if (!std::isfinite(prior_bound) || prior_bound <= 0.0) {
    // The miss event is weighted by the prior even when no tail allocation is
    // assigned, so a zero allocation does not make the prior optional.
    out.reason = "prior probability bound is not positive";
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
  CompleteRiskInputsV1 complete;
  complete.legacy = inputs;
  return buildRiskLedger(risk, hypotheses, complete, nullptr);
}

RiskLedger buildRiskLedger(const RiskBudgetV2& risk,
                           const std::vector<FaultHypothesisV2>& hypotheses,
                           const CompleteRiskInputsV1& inputs,
                           CompleteRiskStatusV1* status) {
  RiskLedger ledger;
  if (status) *status = {};
  std::vector<std::pair<bool, ExactAccumulator>> exact_terms;
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
  // The versioned sidecar carries numeric event bounds even when the
  // corresponding event is asserted absent.  Treat every supplied numeric
  // field as data, not as an ignorable scratch slot: NaN, infinity, and
  // negative values make the complete contract invalid.  This also prevents
  // a contradictory known/validated flag combination from laundering an
  // invalid number into a zero charge.
  const auto validBound = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  const bool sidecar_numeric_inputs_valid =
      validBound(inputs.omitted_event_bound) &&
      validBound(inputs.envelope_event_bound) &&
      validBound(inputs.selection_extra_bound);
  ledger.inputs_valid = ledger.inputs_valid && sidecar_numeric_inputs_valid;

  // 1. Nominal outcome: three axes, each with the configured nominal tail.
  RiskLedgerTerm nominal;
  nominal.id = "nominal";
  const ExactAccumulator exact_nominal =
      ExactAccumulator(risk.nominal_axis_tail) * 3;
  nominal.value = roundUp(exact_nominal);
  nominal.status = RiskTermStatus::Validated;
  nominal.source = "config risk.nominal_axis_tail x 3 protected axes";
  nominal.note = "pi0 * alpha0 for the alarm-free outcome";
  ledger.terms.push_back(std::move(nominal));
  exact_terms.push_back({true, exact_nominal});

  // 2. Non-nominal outcome mass charged for hypotheses that no longer own an
  //    allocation (config term).
  RiskLedgerTerm p_nm;
  p_nm.id = "p_nm";
  p_nm.value = risk.p_nm;
  p_nm.status = RiskTermStatus::Validated;
  p_nm.source = "config risk.p_nm";
  p_nm.note = "non-nominal outcome mass not attributable to a hypothesis";
  ledger.terms.push_back(std::move(p_nm));
  exact_terms.push_back({true, ExactAccumulator(risk.p_nm)});

  // 3. Hypothesis outcome.  Allocation and the detector-miss excess are two
  //    parts of the same pi_h * max(alpha_h,beta_h) union bound.  Both are
  //    charged; an over-budget profile becomes unavailable rather than
  //    relabelling the miss channel as diagnostic.
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
  miss.status = RiskTermStatus::Validated;
  miss.note = "charged detector-miss excess above the allocation-derived tail";
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
    const ExactAccumulator exact_prior(hypothesis.prior_probability_bound);
    const ExactAccumulator exact_alpha = hypothesis.hmi_allocation == 0.0
        ? ExactAccumulator(0)
        : std::min(ExactAccumulator(0.5),
                   ExactAccumulator(hypothesis.hmi_allocation) / exact_prior);
    const ExactAccumulator exact_beta(hypothesis.p_md_allocation);
    if (exact_beta > exact_alpha) {
      miss_channel += exact_prior * (exact_beta - exact_alpha);
    }
  }
  const bool hypotheses_known = rejected.empty();
  fault.value = hypotheses_known
      ? roundUp(charged_hypotheses)
      : std::numeric_limits<double>::quiet_NaN();
  if (!rejected.empty()) {
    fault.status = RiskTermStatus::NotImplemented;
    fault.note += "; hypotheses with rejected tails: " +
        std::to_string(rejected.size()) + " (first: " + rejected.front() + ")";
  }
  ledger.terms.push_back(std::move(fault));
  exact_terms.push_back({hypotheses_known, charged_hypotheses});
  miss.value = hypotheses_known
      ? roundUp(miss_channel)
      : std::numeric_limits<double>::quiet_NaN();
  if (!rejected.empty()) {
    miss.status = RiskTermStatus::NotImplemented;
    miss.note += "; rejected hypothesis tails make the miss bound UNKNOWN";
  }
  ledger.terms.push_back(std::move(miss));
  exact_terms.push_back({hypotheses_known, miss_channel});

  // 4. Escape channels.  A positive configured conservative bound remains in
  //    the arithmetic even before qualification.  A configured zero without
  //    a zero proof is unknown (NaN), never silently free.
  struct EscapeInput {
    const char* id;
    double value;
    bool validated;
    const char* source;
  };
  const EscapeInput escapes[] = {
      {"bridge", risk.p_bridge_escape, inputs.bridge_escape_validated,
       "config risk.p_bridge_escape"},
      {"history", risk.p_history_contamination,
       inputs.history_escape_validated,
       "config risk.p_history_contamination"},
      {"model", risk.p_model_escape, inputs.model_escape_validated,
       "config risk.p_model_escape"}};
  for (const auto& entry : escapes) {
    RiskLedgerTerm term;
    term.id = entry.id;
    term.value = entry.value == 0.0 && !entry.validated
        ? std::numeric_limits<double>::quiet_NaN() : entry.value;
    term.status = entry.validated ? RiskTermStatus::Validated
                                  : RiskTermStatus::AssumedUnvalidated;
    term.source = entry.source;
    term.note = entry.validated
        ? "qualified conservative escape bound"
        : (entry.value == 0.0
            ? "configured zero has no zero proof: value is UNKNOWN"
            : "configured bound is charged but qualification is incomplete");
    ledger.terms.push_back(std::move(term));
    const bool known = entry.value != 0.0 || entry.validated;
    exact_terms.push_back({known, ExactAccumulator(entry.value)});
  }

  // 5. Omitted event mass (manifest omitted_event_set).  A structural proof
  // that there are no omitted events does not authorize this consumer to
  // overwrite contradictory sidecar evidence.  Its only accepted wire
  // representation is a known, validated, finite zero.  When omitted events
  // are present, the scope must be complete and the supplied bound must be
  // known; an unvalidated finite bound is still charged but cannot support
  // formal eligibility.
  RiskLedgerTerm omitted;
  omitted.id = "omitted";
  const bool omitted_events_present =
      !inputs.legacy.omitted_event_set.empty();
  const bool no_omissions = inputs.omitted_scope_complete &&
      !omitted_events_present;
  const bool omitted_numeric_valid = validBound(inputs.omitted_event_bound);
  const bool omitted_flag_consistent =
      !inputs.omitted_event_bound_validated ||
      inputs.omitted_event_bound_known;
  const bool omitted_zero_canonical = no_omissions &&
      omitted_numeric_valid && inputs.omitted_event_bound == 0.0 &&
      inputs.omitted_event_bound_known &&
      inputs.omitted_event_bound_validated;
  const bool omitted_explicit_context = inputs.omitted_scope_complete &&
      omitted_events_present;
  const bool omitted_bound_known = omitted_explicit_context &&
      omitted_numeric_valid && omitted_flag_consistent &&
      inputs.omitted_event_bound_known;
  const bool omitted_known = omitted_zero_canonical || omitted_bound_known;
  const bool omitted_representation_valid = no_omissions
      ? omitted_zero_canonical
      : (omitted_explicit_context
          ? omitted_numeric_valid && omitted_flag_consistent
          : omitted_numeric_valid && omitted_flag_consistent &&
              !inputs.omitted_event_bound_known &&
              !inputs.omitted_event_bound_validated &&
              inputs.omitted_event_bound == 0.0);
  omitted.value = omitted_known ? inputs.omitted_event_bound
      : std::numeric_limits<double>::quiet_NaN();
  omitted.status = omitted_known &&
      (omitted_zero_canonical || inputs.omitted_event_bound_validated)
      ? RiskTermStatus::Validated
      : (omitted_known ? RiskTermStatus::AssumedUnvalidated
                       : RiskTermStatus::NotImplemented);
  omitted.source = "config/integrity_fault_manifest.yaml omitted_event_set";
  omitted.note = omitted_zero_canonical
      ? "resolved scope and canonical known/validated zero prove there are no omitted events"
      : (no_omissions
          ? "zero-event proof has a contradictory sidecar representation; bound is UNKNOWN"
          : "omitted-event mass is UNKNOWN unless the complete scope carries a finite known bound; kinds: ");
  for (std::size_t index = 0;
       index < inputs.legacy.omitted_event_set.size(); ++index) {
    if (index) omitted.note += ",";
    omitted.note += inputs.legacy.omitted_event_set[index];
  }
  if (!no_omissions && inputs.legacy.omitted_event_set.empty())
    omitted.note += "(none declared in the resolved manifest)";
  ledger.terms.push_back(std::move(omitted));
  exact_terms.push_back({omitted_known, ExactAccumulator(
      omitted_numeric_valid ? inputs.omitted_event_bound : 0.0)});
  ledger.inputs_valid = ledger.inputs_valid &&
      omitted_representation_valid;

  // 6. Envelope qualification.  As above, an exact traversal/no-leaf proof is
  // accepted only in its canonical known, validated, finite-zero form.  An
  // online envelope must identify at least one represented leaf and carry a
  // known numeric bound.  The two mixed online/leaf states are contradictory.
  RiskLedgerTerm envelope;
  envelope.id = "envelope";
  const bool exact_no_envelope = !inputs.legacy.envelope_online &&
      inputs.legacy.envelope_leaf_count == 0;
  const bool envelope_events_present = inputs.legacy.envelope_online &&
      inputs.legacy.envelope_leaf_count > 0;
  const bool envelope_numeric_valid = validBound(inputs.envelope_event_bound);
  const bool envelope_flag_consistent =
      !inputs.envelope_event_bound_validated ||
      inputs.envelope_event_bound_known;
  const bool envelope_zero_canonical = exact_no_envelope &&
      envelope_numeric_valid && inputs.envelope_event_bound == 0.0 &&
      inputs.envelope_event_bound_known &&
      inputs.envelope_event_bound_validated;
  const bool envelope_bound_known = envelope_events_present &&
      envelope_numeric_valid && envelope_flag_consistent &&
      inputs.envelope_event_bound_known;
  const bool envelope_known = envelope_zero_canonical ||
      envelope_bound_known;
  const bool envelope_representation_valid = exact_no_envelope
      ? envelope_zero_canonical
      : (envelope_events_present && envelope_numeric_valid &&
          envelope_flag_consistent);
  envelope.value = envelope_known ? inputs.envelope_event_bound
      : std::numeric_limits<double>::quiet_NaN();
  envelope.status = envelope_known &&
      (envelope_zero_canonical || inputs.envelope_event_bound_validated)
      ? RiskTermStatus::Validated
      : (envelope_known ? RiskTermStatus::AssumedUnvalidated
                        : RiskTermStatus::NotImplemented);
  envelope.source = "coverage certificate envelope qualification";
  envelope.note = inputs.legacy.envelope_online
      ? "envelopes online: a finite qualified event bound is required"
      : (envelope_zero_canonical
          ? "exact traversal with zero enveloped leaves and canonical sidecar proves zero"
          : (exact_no_envelope
              ? "zero-envelope proof has a contradictory sidecar representation; bound UNKNOWN"
              : "offline/leaf certificate state or bound representation is inconsistent; bound UNKNOWN"));
  envelope.note += "; enveloped leaves: " +
      std::to_string(inputs.legacy.envelope_leaf_count);
  ledger.terms.push_back(std::move(envelope));
  exact_terms.push_back({envelope_known, ExactAccumulator(
      envelope_numeric_valid ? inputs.envelope_event_bound : 0.0)});
  ledger.inputs_valid = ledger.inputs_valid &&
      envelope_representation_valid;

  // 7. Selection surcharge in the same total budget.
  RiskLedgerTerm selection;
  selection.id = "selection";
  const bool selection_numeric_valid = validBound(inputs.selection_extra_bound);
  const bool selection_flag_consistent =
      !inputs.selection_bound_validated || inputs.selection_bound_known;
  const bool selection_context_valid =
      inputs.legacy.selection_contract_frozen;
  const bool selection_known = inputs.legacy.selection_contract_frozen &&
      inputs.selection_bound_known && selection_numeric_valid &&
      selection_flag_consistent;
  const bool selection_representation_valid = selection_context_valid
      ? selection_numeric_valid && selection_flag_consistent
      : selection_numeric_valid && selection_flag_consistent &&
          !inputs.selection_bound_known &&
          !inputs.selection_bound_validated &&
          inputs.selection_extra_bound == 0.0;
  selection.value = selection_known ? inputs.selection_extra_bound
      : std::numeric_limits<double>::quiet_NaN();
  selection.status = selection_known && inputs.selection_bound_validated
      ? RiskTermStatus::Validated
      : (selection_known ? RiskTermStatus::AssumedUnvalidated
                         : RiskTermStatus::NotImplemented);
  selection.source = "numeric FDE guarantee groups over every eligible action";
  selection.note = inputs.legacy.selection_contract_frozen
      ? "extra union charge beyond the once-charged hypothesis allocation"
      : "selection event set/bound is UNKNOWN before action grouping";
  ledger.terms.push_back(std::move(selection));
  exact_terms.push_back({selection_known, ExactAccumulator(
      selection_numeric_valid ? inputs.selection_extra_bound : 0.0)});
  ledger.inputs_valid = ledger.inputs_valid &&
      selection_representation_valid;

  // Every finite numeric bound is charged even if qualification is pending.
  // Unknown terms are NaN and make complete closure false.
  ExactAccumulator charged = 0;
  ExactAccumulator declared_all = 0;
  bool all_validated = true;
  bool all_numeric = true;
  for (std::size_t index = 0; index < ledger.terms.size(); ++index) {
    auto& term = ledger.terms[index];
    const auto& exact = exact_terms[index];
    const bool finite_nonnegative_export = std::isfinite(term.value) &&
        term.value >= 0.0;
    if (exact.first) {
      declared_all += exact.second;
      charged += exact.second;
    }
    if (!exact.first || !finite_nonnegative_export) {
      all_numeric = false;
      if (term.status == RiskTermStatus::Validated) {
        term.status = RiskTermStatus::NotImplemented;
        term.note += "; finite non-negative exported bound is unavailable";
      }
    }
    if (term.status != RiskTermStatus::Validated) all_validated = false;
  }
  ledger.charged_total = roundUp(charged);
  ledger.declared_total = all_numeric
      ? roundUp(declared_all)
      : std::numeric_limits<double>::quiet_NaN();
  ledger.margin = all_numeric
      ? roundDown(ExactAccumulator(risk.p_hmi_total) - charged)
      : std::numeric_limits<double>::quiet_NaN();
  const RiskBudgetAudit allocation = auditRiskBudget(risk, hypotheses);
  const bool complete_bound_closes = ledger.inputs_valid && all_numeric &&
      charged <= ExactAccumulator(risk.p_hmi_total);
  ledger.closes = complete_bound_closes;
  ledger.all_terms_validated = ledger.inputs_valid && all_validated;
  ledger.formal_eligible = allocation.valid &&
      complete_bound_closes && ledger.all_terms_validated &&
      inputs.model_formal_eligible;
  if (status) {
    status->allocation_valid = allocation.valid;
    status->complete_bound_closes = complete_bound_closes;
  }
  ledger.reason = !ledger.inputs_valid
      ? "complete risk inputs are not finite non-negative bounds"
      : (!all_numeric
      ? "complete risk bound contains UNKNOWN terms"
      : (complete_bound_closes
          ? (ledger.all_terms_validated
              ? "complete risk bound closes"
              : "numeric bound closes but one or more terms are unvalidated")
          : "complete risk bound exceeds the unchanged total budget"));
  return ledger;
}

RiskBudgetAudit auditRiskBudget(
    const RiskBudgetV2& risk,
    const std::vector<FaultHypothesisV2>& hypotheses) {
  RiskBudgetAudit out;
  out.hypothesis_count = hypotheses.size();
  const auto validBound = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  const double inputs[] = {risk.nominal_axis_tail, risk.p_nm,
      risk.p_bridge_escape, risk.p_history_contamination,
      risk.p_model_escape, risk.p_hmi_total};
  out.inputs_valid = true;
  for (double value : inputs) {
    out.inputs_valid = out.inputs_valid && validBound(value);
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
    const bool allocation_valid = validBound(hypothesis.hmi_allocation);
    out.inputs_valid = out.inputs_valid && allocation_valid;
    if (allocation_valid) {
      hypothesis_sum += ExactAccumulator(hypothesis.hmi_allocation);
    }
  }
  if (!out.inputs_valid) {
    const double unknown = std::numeric_limits<double>::quiet_NaN();
    out.nominal = unknown;
    out.p_nm = unknown;
    out.bridge = unknown;
    out.history = unknown;
    out.model = unknown;
    out.hypotheses = unknown;
    out.total = unknown;
    out.upper_bound = validBound(risk.p_hmi_total)
        ? risk.p_hmi_total : unknown;
    out.margin = unknown;
    out.valid = false;
    return out;
  }
  out.nominal = roundUp(ExactAccumulator(risk.nominal_axis_tail) * 3);
  out.p_nm = risk.p_nm;
  out.bridge = risk.p_bridge_escape;
  out.history = risk.p_history_contamination;
  out.model = risk.p_model_escape;
  out.upper_bound = risk.p_hmi_total;
  out.hypotheses = roundUp(hypothesis_sum);
  const ExactAccumulator exact_total =
      ExactAccumulator(risk.nominal_axis_tail) * 3 +
      ExactAccumulator(risk.p_nm) + ExactAccumulator(risk.p_bridge_escape) +
      ExactAccumulator(risk.p_history_contamination) +
      ExactAccumulator(risk.p_model_escape) + hypothesis_sum;
  const ExactAccumulator exact_upper(risk.p_hmi_total);
  out.total = roundUp(exact_total);
  out.margin = roundDown(exact_upper - exact_total);
  out.valid = out.inputs_valid && exact_total <= exact_upper;
  return out;
}

}  // namespace uwb_imu_pl
