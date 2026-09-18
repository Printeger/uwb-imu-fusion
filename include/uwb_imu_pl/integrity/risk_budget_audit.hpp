#pragma once

#include "uwb_imu_pl/integrity/fde_manager.hpp"

#include <cstddef>
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
  bool valid = false;
};

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
