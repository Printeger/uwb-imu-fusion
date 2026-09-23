#pragma once

#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"

namespace uwb_imu_pl {
// P6/C-round Stage 0: physical attribution of a frozen-window state increment.
// Splits the 15-dimensional epoch blocks into rotation / position / velocity /
// accelerometer-bias / gyroscope-bias magnitudes and names the dominant block,
// so a step-gate rejection can be explained without changing the gate itself.
std::string stateStepAttribution(const LinearizedIntegrityWindow& window,
                                 const Eigen::VectorXd& increment);



struct DetectorRiskContext {
  double p_fa_per_test = 1e-6;
  std::uint64_t continuity_horizon_tests = 1000;
  double rank_tolerance = 1e-10;
  double max_condition_number = 1e10;
};

enum class DetectorContractMode {
  ActiveDualChannelV6,
  LegacyPooledOffline,
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
  // C2 (§7.3): separated residual channels.  `pooled` above stays the product
  // decision (unchanged thresholds); these fields expose the two-channel split
  // and the joint acceptance A = {T_c <= tau_c} AND {T_b <= tau_b} so no
  // consumer has to re-derive it.
  double channel_current_statistic = 0.0;
  double channel_current_threshold = 0.0;
  int channel_current_dof = 0;
  bool channel_current_accepted = false;
  double channel_history_statistic = 0.0;
  double channel_history_threshold = 0.0;
  int channel_history_dof = 0;
  bool channel_history_accepted = false;
  bool channel_split_valid = false;
  bool joint_accepted = false;
  std::string channel_reason;
  DetectorContractMode contract_mode = DetectorContractMode::ActiveDualChannelV6;
  std::string accepted_event_id;
  std::uint64_t candidate_numerical_identity = 0;
  std::uint64_t candidate_certificate_digest = 0;
  std::uint64_t detector_contract_digest = 0;
  std::uint64_t continuity_horizon_tests = 0;
  int active_channel_count = 0;
  double history_constant = std::numeric_limits<double>::quiet_NaN();
};

std::uint64_t detectorContractDigest(const DetectorResultV2& detector);

class JointWindowDetector {
 public:
  DetectorResultV2 evaluate(const LinearizedIntegrityWindow& window,
                            const DetectorRiskContext& risk) const;
  DetectorResultV2 evaluateCandidate(const LinearizedIntegrityWindow& window,
                                     const CandidateEvaluation& candidate,
                                     const DetectorRiskContext& risk) const;
};

}  // namespace uwb_imu_pl
