#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "uifgo/nlos_support.h"

namespace uifgo {

inline constexpr const char* kPlConditionalSourceCommit =
    "ae54fb8ca55dfbfaf64fe45615b6bcd106548a93";
inline constexpr const char* kPlConditionalPreflightProvider =
    "pl_conditional_raim_preflight_v1";
inline constexpr const char* kPlConditionalProductionProvider =
    "pl_conditional_raim_fde_v1";
inline constexpr const char* kPlConditionalIdentityVersion =
    "UIFGO_PL_CONDITIONAL_RAIM_FDE_V1";
inline constexpr const char* kPlConditionalPartitionRule =
    "PL_CONDITIONAL_UNIQUE_LOAO_POSITIVE_TEMPORAL_V1";

struct PlConditionalInput {
  Eigen::Matrix<double, 15, 15> prior_covariance =
      Eigen::Matrix<double, 15, 15>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  Eigen::MatrixXd physical_jacobian;
  Eigen::MatrixXd physical_covariance;
  Eigen::VectorXd physical_innovation;
  double p_fa = 1e-5;
};

struct PlConditionalDecision {
  bool numerically_valid = false;
  bool passed = false;
  std::string status = "NOT_EVALUATED";
  int dof = 0;
  double p_fa = 1e-5;
  double threshold = std::numeric_limits<double>::quiet_NaN();
  double statistic = std::numeric_limits<double>::quiet_NaN();
  Eigen::MatrixXd whitener;
  Eigen::VectorXd whitened_innovation;
  Eigen::MatrixXd whitened_jacobian;
  Eigen::MatrixXd innovation_covariance_whitened;
  Eigen::MatrixXd innovation_covariance_physical;
};

// Source-neutral implementation of the locked PL current-group conditional
// innovation test. It has no graph, truth, support, or commit side effects.
PlConditionalDecision EvaluatePlConditional(const PlConditionalInput& input);

struct PlConditionalHypothesis {
  int excluded_anchor_id = 0;
  std::vector<std::uint64_t> retained_obs_ids;
  PlConditionalDecision decision;
};

enum class PlConditionalGroupOutcome {
  BOOTSTRAP_HISTORY,
  GROUP_PASS,
  UNIQUE_ISOLATION_POSITIVE,
  UNIQUE_ISOLATION_NONPOSITIVE,
  FDE_UNISOLATED_FAULT,
  FDE_ISOLATION_AMBIGUOUS,
  NUMERICAL_FAILURE,
};

const char* PlConditionalGroupOutcomeName(PlConditionalGroupOutcome outcome);

struct PlConditionalGroupResult {
  PlConditionalDecision omnibus;
  std::vector<PlConditionalHypothesis> hypotheses;
  PlConditionalGroupOutcome outcome =
      PlConditionalGroupOutcome::NUMERICAL_FAILURE;
  int isolated_anchor_id = 0;
  std::uint64_t isolated_obs_id = 0;
  double isolated_innovation_m = std::numeric_limits<double>::quiet_NaN();
  bool positive_excess_candidate = false;
  bool prior_degradation_event = false;
  std::vector<size_t> committed_rows;
};

// Alarm isolation evaluates every leave-one-anchor-out subset against exactly
// the same prior. Anchor IDs must be unique in v1.
PlConditionalGroupResult EvaluatePlConditionalLoao(
    const PlConditionalInput& input, const std::vector<int>& anchor_ids,
    const std::vector<std::uint64_t>& obs_ids);

struct PlConditionalCandidateRecord {
  std::uint64_t obs_id = 0;
  int tag_id = 0;
  int anchor_id = 0;
  size_t keyframe_id = 0;
  double sensor_time = std::numeric_limits<double>::quiet_NaN();
  bool planned = false;
  bool candidate = false;
  std::string reason;
  std::string segment_id;
  size_t segment_ordinal = std::numeric_limits<size_t>::max();
};

struct PlConditionalSupportContext {
  std::string input_plan_hash;
  std::string source_hash;
  std::string config_hash;
  std::string calibration_hash;
  std::string solver_config_hash;
  std::string detector_identity_hash;
  bool production = false;
};

// Every non-candidate planned row interrupts the current run for that link.
SupportPartition BuildPlConditionalSupport(
    std::vector<PlConditionalCandidateRecord>* records,
    double gap_threshold_s, size_t minimum_count,
    double minimum_duration_s, const PlConditionalSupportContext& context);

std::string PlConditionalDetectorIdentity(
    const PlConditionalSupportContext& context, size_t bootstrap_last_keyframe,
    size_t fixed_lag_epochs, double imu_max_gap_s,
    double relinearize_threshold, size_t relinearize_skip);

}  // namespace uifgo
