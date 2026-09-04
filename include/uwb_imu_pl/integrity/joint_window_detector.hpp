#pragma once

#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"

namespace uwb_imu_pl {

struct DetectorRiskContext {
  double p_fa_per_test = 1e-6;
  std::uint64_t continuity_horizon_tests = 1000;
  double rank_tolerance = 1e-10;
  double max_condition_number = 1e10;
};

struct DetectorResultV2 {
  WindowId window_id;
  double squared_parity_statistic = std::numeric_limits<double>::infinity();
  double squared_threshold = std::numeric_limits<double>::infinity();
  int rows = 0;
  int rank = 0;
  int dof = 0;
  double p_fa_per_test = 0.0;
  double operation_p_fa_upper_bound = 0.0;
  bool passed = false;
  bool numerically_valid = false;
  std::string reason;
};

class JointWindowDetector {
 public:
  DetectorResultV2 evaluate(const LinearizedIntegrityWindow& window,
                            const DetectorRiskContext& risk) const;
  DetectorResultV2 evaluateCandidate(const CandidateEvaluation& candidate,
                                     const DetectorRiskContext& risk) const;
};

}  // namespace uwb_imu_pl
