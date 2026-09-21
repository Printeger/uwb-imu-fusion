#pragma once

#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"
#include "uwb_imu_pl/integrity/fault_model.hpp"

namespace uwb_imu_pl {

struct HypothesisGeneratorConfig {
  bool single_faults_enabled = true;
  bool double_faults_enabled = false;
  std::uint32_t max_model_cardinality = 2;
  std::uint32_t max_exclusion_cardinality = 2;
  std::uint32_t max_candidate_count = 128;
  double uwb_prior_bound = 1e-4;
  double accel_prior_bound = 1e-5;
  double gyro_prior_bound = 1e-5;
  double uwb_p_md = 1e-3;
  double imu_p_md = 1e-3;
  double total_hmi_allocation = 9e-6;
  double rank_tolerance = 1e-10;
  bool include_uwb_accel_combinations = true;
  bool include_uwb_gyro_combinations = true;
  bool include_epoch_independent_uwb = true;
  bool include_persistent_uwb = true;
  bool include_ramp_uwb = true;
};

struct GeneratedFaultModelSet {
  double historical_sensitivity_ms = 0.0;
  std::vector<FaultUnit> units;
  std::vector<FaultModeBasis> modes;
  std::vector<FaultHypothesisV2> hypotheses;
  std::vector<ExclusionAction> actions;
  std::vector<ExclusionAction> single_mode_actions;
  std::size_t single_uwb_hypotheses = 0;
  std::size_t single_accel_hypotheses = 0;
  std::size_t single_gyro_hypotheses = 0;
  std::size_t double_uwb_accel_hypotheses = 0;
  std::size_t double_uwb_gyro_hypotheses = 0;
  std::uint32_t effective_max_cardinality = 0;
  // B2: runtime family/parameter defence counters (see pairFamilySupport).
  std::size_t pair_candidates_considered = 0;
  std::size_t pair_candidates_rejected_unsupported = 0;
  std::size_t pair_candidates_rejected_shared = 0;
};

// Exact second-stage check for canonical-action deduplication.  The compact
// key is only an index; equal keys never authorize merging without this full
// operation/content comparison.
bool equivalentActionOperation(const ExclusionAction& left,
                               const ExclusionAction& right);

// Stable hardware-health identity.  IMU interval identity remains on the
// FaultUnit; this projection is used only for the source quarantine barrier.
std::string healthSourceId(const FaultUnit& unit);

// Generates only the initial V2 claim: physical single-anchor UWB faults,
// six interval-constant IMU axes, and one-anchor-plus-one-IMU-axis modes.
class HypothesisGenerator {
 public:
  explicit HypothesisGenerator(HypothesisGeneratorConfig config = {})
      : config_(config) {}

  GeneratedFaultModelSet generate(
      const LinearizedIntegrityWindow& window,
      const EpochTransaction& transaction,
      const ImuFaultSubspaces& imu_subspaces,
      const LinearizedFactorBlock& generic_bridge_block) const;

  std::vector<ExclusionAction> actionsForPlausibleSet(
      const LinearizedIntegrityWindow& window,
      const EpochTransaction& transaction,
      const GeneratedFaultModelSet& models,
      const std::vector<FaultModeEvidence>& evidence,
      const std::vector<std::string>& mandatory_health_sources = {}) const;

 private:
  HypothesisGeneratorConfig config_;
};

}  // namespace uwb_imu_pl
