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
