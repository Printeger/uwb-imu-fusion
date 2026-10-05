#pragma once

#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"
#include "uwb_imu_pl/integrity/fault_model.hpp"

#include <memory>

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

// P0-06 versioned sidecar.  This intentionally does not add fields to the
// long-lived output or action layouts: callers that need the completeness
// certificate opt in to the V1 result below.
struct ActionOmissionV1 {
  std::uint64_t action_id = 0;
  // Identity of this raw occurrence (not merely of its graph operation).
  std::string occurrence_identity;
  std::string operation_identity;
  // For EXACT_SEMANTIC_DUPLICATE this names the retained occurrence whose
  // canonical operation and downstream semantic bytes are identical.
  std::string duplicate_of_identity;
  std::string reason;
  bool proven_safe = false;
  // Complete downstream semantics. Equal graph operations alone are not safe
  // to merge because action identity/model/coverage affect later decisions.
  std::string semantic_identity;
};

struct ActionOccurrenceV1 {
  std::uint64_t action_id = 0;
  std::string occurrence_identity;
  // Complete canonical operation serialization.  The compact digest/index is
  // never sufficient for equality; validators compare these bytes.
  std::string operation_identity;
  std::string semantic_identity;
};

struct ActionSearchCensusV1 {
  std::uint32_t protocol_version = 1;
  std::uint64_t generated = 0;
  std::uint64_t evaluated = 0;
  std::uint64_t omitted = 0;
  bool exhaustive = false;
  std::vector<ActionOccurrenceV1> generated_records;
  std::vector<std::string> generated_identities;
  std::vector<std::string> evaluated_identities;
  std::vector<ActionOmissionV1> omitted_actions;
  // Exact (not compact-hash) identity of the separately held raw snapshot.
  std::string generated_snapshot_identity;
  std::string terminal_reason;
};

struct ActionSearchValidationV1 {
  bool valid = false;
  bool exhaustive = false;
  std::string reason;
};

struct GeneratedActionSnapshotV1 {
  std::uint32_t protocol_version = 1;
  std::string generator_identity;
  std::string snapshot_identity;
  std::vector<ExclusionAction> actions;
};

enum class ActionSearchLifecycleV1 : std::uint8_t {
  ReadyForEvaluation = 0,
  AbortedIncompleteBeforeEvaluation = 1
};

struct ActionSearchResultV1 {
  GeneratedActionSnapshotV1 generated_snapshot;
  std::vector<ExclusionAction> actions;
  ActionSearchCensusV1 census;
  std::size_t max_evaluated_actions = 0;
  ActionSearchLifecycleV1 lifecycle =
      ActionSearchLifecycleV1::ReadyForEvaluation;
};

// P1-04 additive streaming contract.  V1 remains the fail-closed capped API
// used by old callers.  In V2, max_candidate_count is a resident batch bound,
// never a total-search cap: every exact-semantic representative is present in
// `complete.actions`, while exact duplicates remain omission records with a
// proof.  The batch ranges are deterministic and cover the complete vector.
struct CompleteActionStreamV2 {
  std::uint32_t protocol_version = 2;
  ActionSearchResultV1 complete;
  std::size_t batch_capacity = 0;
  std::size_t batch_count = 0;
  // Heavy numerical work admitted concurrently by the stream.  The compact
  // action census remains proportional to generated identities for audit.
  std::size_t peak_batch_actions = 0;
};

// P1-06 additive production sidecar.  The frozen V1/V2 objects above retain
// their layouts and behavior for ABI/API compatibility.  V3 keeps only an
// attempt-scoped recipe for each terminal action.  Dense replacement/bridge
// blocks are reconstructed into a leased batch and disappear when that lease
// is destroyed; they are never resident for the complete action set.
struct CompactActionDescriptorV3 {
  struct BlockReference {
    std::uint64_t block_id = 0;
    std::uint64_t offset = 0;
    std::uint64_t length = 0;
    // SHA-256 of the exact, unhexed canonical V1 block payload.  The digest
    // selects a candidate bucket only; equality is established by bounded
    // pread comparison of the sidecar bytes.
    std::string strong_digest;
  };

  std::uint64_t action_id = 0;
  std::vector<std::size_t> mode_indices;
  std::string action_model_override;
  bool cardinality_contract_rejected = false;
  bool force_missing_provenance = false;
  std::uint64_t operation_digest = 0;
  std::uint64_t semantic_digest = 0;
  std::uint64_t operation_offset = 0;
  std::uint64_t operation_length = 0;
  std::uint64_t semantic_offset = 0;
  std::uint64_t semantic_length = 0;
  std::vector<BlockReference> ordered_blocks;
  std::size_t materialized_heavy_bytes = 0;
};

struct CompactActionOmissionV3 {
  CompactActionDescriptorV3 occurrence;
  std::uint64_t representative_action_id = 0;
  std::string reason;
  bool proven_safe = false;
};

struct CompactActionSearchV3 {
  std::uint32_t protocol_version = 3;
  std::uint64_t generated = 0;
  std::uint64_t evaluated = 0;
  std::uint64_t omitted = 0;
  bool exhaustive = false;
  std::uint64_t snapshot_digest = 0;
  std::uint64_t sidecar_digest = 0;
  std::uint64_t sidecar_length = 0;
  std::uint64_t unique_block_count = 0;
  std::uint64_t referenced_block_count = 0;
  // The producer sets this only after every canonical byte range, census
  // record and snapshot/root binding has been checked against the sealed
  // attempt-owned sidecar.  Consumers may re-read the sealed bytes, but never
  // rebuild all canonical strings in memory.
  bool producer_certificate_valid = false;
  std::size_t batch_capacity = 0;
  std::size_t batch_count = 0;
  std::size_t peak_batch_actions = 0;
  std::size_t peak_batch_heavy_bytes = 0;
  std::uint64_t digest_collision_comparisons = 0;
  std::vector<CompactActionDescriptorV3> actions;
  std::vector<CompactActionOmissionV3> omitted_actions;
  std::string terminal_reason;
};

class AttemptActionArenaV3;

class AttemptActionLeaseV3 {
 public:
  AttemptActionLeaseV3() = default;

  std::vector<ExclusionAction> materializeBatch(
      std::size_t begin, std::size_t count) const;
  ExclusionAction materialize(std::size_t index) const;
  // Exact census/eligibility/FDE fields without dense block payload copies.
  // Numerical consumers must use materialize()/materializeBatch().
  ExclusionAction materializeMetadata(std::size_t index) const;
  const CompactActionSearchV3& search() const;
  bool valid() const;
  void close();

 private:
  friend class AttemptActionArenaV3;
  friend class HypothesisGenerator;
  friend ActionSearchValidationV1 validateCompactActionSearchV3(
      const AttemptActionLeaseV3& lease);
  struct State;
  explicit AttemptActionLeaseV3(std::shared_ptr<const State> state)
      : state_(std::move(state)) {}
  std::shared_ptr<const State> state_;
};

class AttemptActionArenaV3 {
 public:
  AttemptActionArenaV3() = default;
  AttemptActionLeaseV3 lease() const;
  const CompactActionSearchV3& search() const;
  bool valid() const;
  void close();
  // Explicit per-arena fault seam used by directed fail-closed tests.  It is
  // never process-global and cannot be activated by an environment variable.
  void injectSidecarReadFailureForTest();

 private:
  friend class HypothesisGenerator;
  explicit AttemptActionArenaV3(
      std::shared_ptr<const AttemptActionLeaseV3::State> state)
      : state_(std::move(state)) {}
  std::shared_ptr<const AttemptActionLeaseV3::State> state_;
};

ActionSearchValidationV1 validateCompactActionSearchV3(
    const AttemptActionLeaseV3& lease);

// Consumer boundary for compact terminal candidates.  `consumed` may omit
// heavy added_blocks after its numerical proof has been minted, but every
// operation/coverage/selection field is compared with a freshly materialized
// exact action from the lease.
ActionSearchValidationV1 validateCompactActionConsumptionV3(
    const AttemptActionLeaseV3& lease,
    const std::vector<ExclusionAction>& consumed);

CompleteActionStreamV2 makeCompleteActionStreamV2(
    const std::vector<ExclusionAction>& generated,
    std::size_t batch_capacity,
    GeneratedActionSnapshotV1* trusted_generated_snapshot = nullptr);

ActionSearchValidationV1 validateCompleteActionStreamV2(
    const CompleteActionStreamV2& stream,
    const GeneratedActionSnapshotV1& trusted_generated_snapshot);

// Canonical, byte-exact graph-operation identity. This proves operation
// equivalence only; it is deliberately insufficient for deduplication.
std::string exactActionOperationIdentityV1(const ExclusionAction& action);

// Canonical downstream semantic identity. It includes the operation identity
// and every action field that affects FDE, selection, risk, commit or output.
std::string exactActionSemanticIdentityV1(const ExclusionAction& action);

// Independently useful O07 seam: exact-semantic-deduplicate actions, then
// apply the resource cap.  Any non-duplicate omission makes the result
// SEARCH_INCOMPLETE; raising the cap is not treated as a proof.
ActionSearchResultV1 censusAndCapActionsV1(
    const std::vector<ExclusionAction>& generated,
    std::size_t max_evaluated_actions,
    GeneratedActionSnapshotV1* trusted_generated_snapshot = nullptr);

// Strict consumer validation.  Every census field is regenerated from the
// complete raw action snapshot and cap.  The evaluated objects are checked
// against that same derived bundle, so a caller cannot make an omitted action
// safe by replacing/reordering self-consistent sidecar strings.
ActionSearchValidationV1 validateActionSearchCensusV1(
    const ActionSearchCensusV1& census,
    const GeneratedActionSnapshotV1& trusted_generated_snapshot,
    std::size_t max_evaluated_actions,
    ActionSearchLifecycleV1 lifecycle,
    const std::vector<ExclusionAction>& actual_evaluated_actions);

// Converts a retained-but-not-yet-evaluated incomplete search into an honest
// zero-evaluation census before entering any candidate kernel.
void abortIncompleteActionSearchBeforeEvaluationV1(
    ActionSearchResultV1* result);

// Exact graph-operation comparison only. Callers must not infer downstream
// semantic equivalence from this predicate.
bool equivalentActionOperation(const ExclusionAction& left,
                               const ExclusionAction& right);

// Production semantic projection shared by candidate PL and O07 evidence.
// It removes exactly the modes/units certified as covered by the action and
// retains every other represented hypothesis.  Numerical row remapping is a
// separate graph-specific step and must not delete an uncovered hypothesis.
std::vector<FaultHypothesisV2> projectRemainingHypothesesForActionV1(
    const std::vector<FaultHypothesisV2>& hypotheses,
    const ExclusionAction& action);

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

  ActionSearchResultV1 actionsForPlausibleSetV1(
      const LinearizedIntegrityWindow& window,
      const EpochTransaction& transaction,
      GeneratedFaultModelSet* models,
      const std::vector<FaultModeEvidence>& evidence,
      const std::vector<std::string>& mandatory_health_sources = {},
      GeneratedActionSnapshotV1* trusted_generated_snapshot = nullptr) const;

  CompleteActionStreamV2 actionsForPlausibleSetV2(
      const LinearizedIntegrityWindow& window,
      const EpochTransaction& transaction,
      GeneratedFaultModelSet* models,
      const std::vector<FaultModeEvidence>& evidence,
      const std::vector<std::string>& mandatory_health_sources = {},
      GeneratedActionSnapshotV1* trusted_generated_snapshot = nullptr) const;

  // Production-only bounded sidecar.  Unlike V2 this never calls
  // ensureActionEntities(): complete logical coverage is represented by
  // compact mode recipes and heavy ExclusionAction objects exist only inside
  // the returned lease's fixed-size materialization batches.
  AttemptActionArenaV3 actionsForPlausibleSetV3(
      const LinearizedIntegrityWindow& window,
      const EpochTransaction& transaction,
      const GeneratedFaultModelSet& models,
      const std::vector<FaultModeEvidence>& evidence,
      const std::vector<std::string>& mandatory_health_sources = {},
      bool force_digest_collision_for_test = false) const;

  // B4: materializes the exclusion-action entities (single-mode actions and
  // their bridge/rollback blocks) exactly once, on demand.  Idempotent.
  static void ensureActionEntities(const EpochTransaction& transaction,
                                   const LinearizedIntegrityWindow& window,
                                   GeneratedFaultModelSet* models);

 private:
  HypothesisGeneratorConfig config_;
};

}  // namespace uwb_imu_pl
