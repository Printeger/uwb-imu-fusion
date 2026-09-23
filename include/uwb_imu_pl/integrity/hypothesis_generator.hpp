#pragma once

#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"
#include "uwb_imu_pl/integrity/fault_model.hpp"

namespace uwb_imu_pl {

struct HypothesisGeneratorConfig {
  bool include_uwb_faults = true;
  bool include_imu_faults = true;
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
  // B4: healthy, alarm-free frames need the mode descriptions, their
  // sensitivity/slopes and the KEEP_ALL solution, but not the exclusion-action
  // entities (removal sets, replacement groups, bridge blocks).  When enabled,
  // the generator stores only the seeds and the entities are materialized on
  // demand by ensureActionEntities() on the paths that really need evidence.
  bool lazy_action_entities = true;
};

struct GeneratedFaultModelSet {
  // P0-01/O03 physical census.  Expected identities are enumerated from the
  // frozen transaction/history scope before mode construction, represented
  // identities come from the registry, and evaluated identities are those
  // referenced by at least one generated hypothesis.
  std::vector<std::string> expected_mode_identities;
  std::vector<std::string> represented_mode_identities;
  std::vector<std::string> evaluated_mode_identities;
  // Order-2 census uses stable physical identities (not mode ids).  Expected
  // pairs are the manifest-enabled Cartesian product of independently
  // enumerated physical modes, represented pairs are registry hypotheses,
  // and evaluated pairs are hypotheses handed to the numerical evidence
  // stage.  Equality is a generation invariant.
  std::vector<std::string> expected_pair_identities;
  std::vector<std::string> represented_pair_identities;
  std::vector<std::string> evaluated_pair_identities;
  double historical_sensitivity_ms = 0.0;
  std::vector<FaultUnit> units;
  std::vector<FaultModeBasis> modes;
  std::vector<FaultHypothesisV2> hypotheses;
  std::vector<ExclusionAction> actions;
  std::vector<ExclusionAction> single_mode_actions;
  // B4 counters and lazy state.  `action_entities_built` tells whether
  // single_mode_actions has been materialized; the counters make the lazy path
  // observable (a healthy frame must report zero bridge blocks).
  bool action_entities_built = false;
  std::size_t action_entities_constructed = 0;
  std::size_t bridge_blocks_built = 0;
  std::size_t action_entities_deferred = 0;
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
      GeneratedFaultModelSet* models,
      const std::vector<FaultModeEvidence>& evidence,
      const std::vector<std::string>& mandatory_health_sources = {}) const;

  // B4: materializes the exclusion-action entities (single-mode actions and
  // their bridge/rollback blocks) exactly once, on demand.  Idempotent.
  static void ensureActionEntities(const EpochTransaction& transaction,
                                   const LinearizedIntegrityWindow& window,
                                   GeneratedFaultModelSet* models);

 private:
  HypothesisGeneratorConfig config_;
};

}  // namespace uwb_imu_pl
