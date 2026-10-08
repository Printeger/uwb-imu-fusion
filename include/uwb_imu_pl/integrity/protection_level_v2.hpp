#pragma once

#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"
#include "uwb_imu_pl/estimation/epoch_transaction.hpp"

#include <functional>
#include <map>

namespace uwb_imu_pl {

struct ProtectionLevelV2Result {
  Eigen::Vector3d pl_xyz_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d nominal_component_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d fault_component_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d bridge_component_m = Eigen::Vector3d::Zero();
  double hpl_m = std::numeric_limits<double>::infinity();
  double vpl_m = std::numeric_limits<double>::infinity();
  double allocated_outcome_risk = std::numeric_limits<double>::infinity();
  bool risk_budget_valid = false;
  bool model_valid = false;
  bool formal_eligible = false;
  Availability availability = Availability::Unavailable;
  // B3 (§5.9): the bound actually used for this result, so the certificate can
  // be checked without re-deriving it.  `hypothesis_tail_used` is the charged
  // per-hypothesis tail alpha_h, `axis_tail_used` its equal split over the
  // three protected axes (alpha_h / 3) and `fault_multiplier_used` the
  // resulting normal multiplier k = Phi^-1(1 - alpha_h / 6).
  double hypothesis_tail_used = 0.0;
  double axis_tail_used = 0.0;
  double fault_multiplier_used = 0.0;
  double noncentrality_used = 0.0;
  std::string detector_certificate_id;
  std::string reason;
};

struct ProtectionBridgeProofV1 {
  Eigen::MatrixXd projected_gain;
  Eigen::VectorXd deterministic_bound;
  Eigen::Vector3d served_component = Eigen::Vector3d::Zero();
};

struct ProtectionHypothesisComponentProofV1 {
  HypothesisId hypothesis;
  int detector_dof = 0;
  double detector_threshold = 0.0;
  double hypothesis_tail = 0.0;
  double axis_tail = 0.0;
  double multiplier = 0.0;
  double noncentrality = 0.0;
  bool dual_channel = false;
  Eigen::Vector3d served_component = Eigen::Vector3d::Zero();
};

struct ProtectionLevelV2ProofV1 {
  std::uint64_t schema_version = 1;
  std::uint64_t candidate_proof_identity = 0;
  std::uint64_t detector_proof_identity = 0;
  std::uint64_t detector_candidate_numerical_identity = 0;
  bool pooled_only = false;
  bool detector_passed = false;
  RiskBudgetV2 risk;
  Eigen::MatrixXd protected_covariance_factor;
  Eigen::Matrix3d protected_covariance = Eigen::Matrix3d::Zero();
  double covariance_rank_tolerance = 1e-10;
  double nominal_tail = 0.0;
  double nominal_multiplier = 0.0;
  Eigen::Vector3d nominal_sigma = Eigen::Vector3d::Zero();
  Eigen::Vector3d fixed_bridge_margin = Eigen::Vector3d::Zero();
  std::vector<ProtectionBridgeProofV1> bridge_proofs;
  std::vector<FaultHypothesisV2> hypotheses;
  std::vector<FrozenHypothesisPlProofV1> hypothesis_proofs;
  std::vector<DualChannelNumericalProofV1> dual_channel_proofs;
  std::vector<ProtectionHypothesisComponentProofV1> component_proofs;
  ProtectionLevelV2Result served_result;
  std::uint64_t proof_identity = 0;
};

struct ProtectionLevelPublicationPacketV1 {
  std::uint64_t schema_version = 1;
  std::uint64_t candidate_proof_identity = 0;
  std::uint64_t protection_proof_identity = 0;
  Eigen::Vector3d served_pl = Eigen::Vector3d::Zero();
  std::string detector_certificate_id;
  std::uint64_t packet_identity = 0;
};

// P0-05 transfer metadata is deliberately carried by a new versioned type.
// The V1 definition above is part of the golden-p0-04 public ABI and must not
// grow or change offsets.
struct ProtectionLevelPublicationPacketV2 {
  std::uint64_t schema_version = 2;
  std::uint64_t candidate_proof_identity = 0;
  std::uint64_t protection_proof_identity = 0;
  Eigen::Vector3d served_pl = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_pl = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_mean_world_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d committed_mean_world_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d triangle_transfer_m = Eigen::Vector3d::Zero();
  std::int64_t timestamp_ns = 0;
  std::string frame_id;
  std::string position_reference;
  std::string detector_certificate_id;
  std::uint64_t packet_identity = 0;
};

// Self-contained terminal handoff. It owns both the selected numerical proof
// and publication binding, so delayed verification never depends on an arena
// or process-global registry after the attempt lease is released.
struct FinalProtectionProofBundleV1 {
  std::uint64_t schema_version = 1;
  std::uint64_t arena_generation = 0;
  ProtectionLevelV2ProofV1 protection_proof;
  ProtectionLevelPublicationPacketV2 publication_packet;
  std::string packet_id;
  bool valid = false;
};

bool freezeFinalProtectionProofBundleV1(
    const AttemptProofLease& lease, const std::string& packet_id,
    FinalProtectionProofBundleV1* bundle,
    std::string* reason = nullptr);
bool validateFinalProtectionProofBundleV1(
    const FinalProtectionProofBundleV1& bundle,
    const Eigen::Vector3d& served_pl,
    const std::string& packet_id,
    std::string* reason = nullptr);

bool validateProtectionLevelV2ProofPayload(
    const ProtectionLevelV2ProofV1& proof,
    std::string* reason = nullptr);

bool validateProtectionLevelV2Proof(
    const CandidateEvaluation& candidate, const DetectorResultV2& detector,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const ProtectionLevelV2Result& result,
    std::string* reason = nullptr);
bool validateProtectionLevelV2Proof(
    const CandidateEvaluation& candidate, const DetectorResultV2& detector,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const ProtectionLevelV2Result& result, const AttemptProofArena& arena,
    std::string* reason = nullptr);

bool protectionLevelV2Proof(const ProtectionLevelV2Result& result,
                            ProtectionLevelV2ProofV1* proof);
bool protectionLevelV2Proof(const ProtectionLevelV2Result& result,
                            const CandidateEvaluation& candidate,
                            const AttemptProofArena& arena,
                            ProtectionLevelV2ProofV1* proof);
// P1-04 proof ownership helpers. Importing is an exact move/copy of a
// self-contained proof produced for the same candidate/result; it performs no
// numerical recomputation and cannot mint a different identity. The null-arena
// overload preserves the legacy registry behavior for old V1 callers.
bool retainProtectionLevelV2Proof(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    const ProtectionLevelV2ProofV1& proof,
    AttemptProofArena* arena,
    std::string* reason = nullptr);
std::size_t protectionLevelV2FullProofCount(
    const AttemptProofArena& arena);
std::size_t protectionLevelV2FullProofCount(
    const AttemptProofLease& lease);
std::uint64_t protectionLevelV2ProofIdentity(
    const ProtectionLevelV2Result& result);
std::uint64_t protectionLevelV2ProofIdentity(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    const AttemptProofArena& arena);
std::uint64_t protectionLevelV2ProofIdentity(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result);
bool bindProtectionLevelPublicationPacket(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    std::uint64_t protection_proof_identity,
    std::string* packet_id);
bool bindTransferredProtectionLevelPublicationPacket(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    std::uint64_t protection_proof_identity,
    const Eigen::Vector3d& reference_mean_world_m,
    const Eigen::Vector3d& committed_mean_world_m,
    const Eigen::Vector3d& transferred_pl_m,
    std::int64_t timestamp_ns,
    const std::string& frame_id,
    const std::string& position_reference,
    std::string* packet_id);
bool bindTransferredProtectionLevelPublicationPacket(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    std::uint64_t protection_proof_identity,
    const Eigen::Vector3d& reference_mean_world_m,
    const Eigen::Vector3d& committed_mean_world_m,
    const Eigen::Vector3d& transferred_pl_m,
    std::int64_t timestamp_ns,
    const std::string& frame_id,
    const std::string& position_reference,
    AttemptProofArena* arena,
    std::string* packet_id);
bool protectionLevelPublicationPacket(
    const std::string& packet_id,
    ProtectionLevelPublicationPacketV1* packet);
bool protectionLevelPublicationPacketV2(
    const std::string& packet_id,
    ProtectionLevelPublicationPacketV2* packet);
bool protectionLevelPublicationPacketV2(
    const std::string& packet_id, const AttemptProofLease& lease,
    ProtectionLevelPublicationPacketV2* packet);
std::uint64_t protectionLevelPublicationProofIdentity(
    const Eigen::Vector3d& served_pl,
    const std::string& detector_certificate_id);
std::uint64_t protectionLevelPublicationProofIdentity(
    const Eigen::Vector3d& served_pl,
    const std::string& detector_certificate_id,
    const AttemptProofArena& arena);
bool validateProtectionLevelPublicationProof(
    std::uint64_t proof_identity,
    const Eigen::Vector3d& served_pl,
    const std::string& detector_certificate_id,
    std::string* reason = nullptr);
bool validateProtectionLevelPublicationProof(
    std::uint64_t proof_identity,
    const Eigen::Vector3d& served_pl,
    const std::string& detector_certificate_id,
    const AttemptProofArena& arena,
    std::string* reason = nullptr);

// Mint/consume the single-use handoff used by the P0-05 commit boundary.
// Minting independently validates the P0-03 candidate and PL proof payloads
// and derives the reference mean from the frozen window and candidate step.
// Consumption compares the complete registered value and erases it before
// any backend mutation, so fabricated, stale, replayed, or altered evidence
// can never mark a receipt protected.
bool mintCommitProtectionEvidenceV1(
    const EpochTransaction& transaction,
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    std::uint64_t protection_proof_identity,
    const std::string& frame_id,
    CommitProtectionEvidenceV1* evidence,
    std::string* reason = nullptr);
bool mintCommitProtectionEvidenceV1(
    const EpochTransaction& transaction,
    const FrozenWindowAdmission& admission,
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    std::uint64_t protection_proof_identity,
    const std::string& frame_id,
    AttemptProofArena* arena,
    CommitProtectionEvidenceV1* evidence,
    std::string* reason = nullptr);
bool mintCommitProtectionEvidenceV1(
    const EpochTransaction& transaction,
    const FrozenWindowAdmission& admission,
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    std::uint64_t protection_proof_identity,
    const std::string& frame_id,
    CommitProtectionEvidenceV1* evidence,
    std::string* reason = nullptr);
bool consumeCommitProtectionEvidenceV1(
    const CommitProtectionEvidenceV1& evidence,
    const EpochTransaction& transaction,
    const std::string& expected_frame_id,
    std::string* reason = nullptr);

struct FrozenBridgeProjection {
  Eigen::MatrixXd jacobian_whitened;
  Eigen::VectorXd deterministic_bound;
};

struct ProtectionLevelSharedContext {
  // Candidate-row maps keyed by the exact frozen fault-mode ID.
  std::map<std::uint64_t, Eigen::MatrixXd> mode_maps;
  std::vector<FrozenBridgeProjection> bridges;
};

// R11 batch boundary. The caller owns every pointer through the synchronous
// call. Candidate order and each hypothesis vector order define the canonical
// Cartesian slot order; workers never append to shared output containers.
struct FlatProtectionCandidateV1 {
  CandidateEvaluation* candidate = nullptr;
  const DetectorResultV2* detector = nullptr;
  std::vector<FaultHypothesisV2>* hypotheses = nullptr;
  const ProtectionLevelSharedContext* shared = nullptr;
  ProtectionLevelV2Result* result = nullptr;
  ProtectionLevelV2ProofV1* computation_audit = nullptr;
  AttemptProofArena* proof_arena = nullptr;
  // Optional P1-02 KEEP_ALL fast root. When all fields are present the flat
  // scheduler consumes the same shared dual blocks/proofs as
  // computeFrozenAllIn; absence selects the ordinary candidate-specific root.
  const std::vector<FaultModeBasis>* frozen_modes = nullptr;
  const FrozenHypothesisNumerics* frozen = nullptr;
  const FrozenHypothesisDualNumerics* frozen_dual = nullptr;
  const AttemptProofArena* frozen_proof_arena = nullptr;
  std::uint64_t fault_model_policy_fingerprint = 0;
};

struct ProtectionModeResponseCacheStatsV1 {
  std::uint64_t unique_mode_products = 0;
  std::uint64_t cached_hypothesis_products = 0;
  std::uint64_t exhaustive_hypothesis_products = 0;
  std::uint64_t retained_bytes = 0;
};

class ProtectionLevelV2 {
 public:
  using FaultMapProvider = std::function<Eigen::MatrixXd(
      const FaultHypothesisV2&)>;

  ProtectionLevelV2Result compute(
      const LinearizedIntegrityWindow& window,
      const CandidateEvaluation& candidate,
      const DetectorResultV2& detector,
      std::vector<FaultHypothesisV2>* remaining_hypotheses,
      const RiskBudgetV2& risk,
      const Eigen::Vector3d& bridge_margin_m = Eigen::Vector3d::Zero()) const;

  // Compact-mode path: the provider assembles one hypothesis map at a time
  // from unique preprojected modes, so combinations never retain dense A.
  ProtectionLevelV2Result computeStreaming(
      const LinearizedIntegrityWindow& window,
      const CandidateEvaluation& candidate,
      const DetectorResultV2& detector,
      std::vector<FaultHypothesisV2>* remaining_hypotheses,
      const FaultMapProvider& fault_map_provider,
      const RiskBudgetV2& risk,
      const Eigen::Vector3d& bridge_margin_m = Eigen::Vector3d::Zero()) const;

  // Optimized production path: protected covariance, every unique remaining
  // mode and all bridge gains share exactly one covarianceTimes() call.
  ProtectionLevelV2Result computeShared(
      const LinearizedIntegrityWindow& window,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      std::vector<FaultHypothesisV2>* remaining_hypotheses,
      const ProtectionLevelSharedContext& shared,
      const RiskBudgetV2& risk) const;
  ProtectionLevelV2Result computeShared(
      const FrozenWindowAdmission& admission,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      std::vector<FaultHypothesisV2>* remaining_hypotheses,
      const ProtectionLevelSharedContext& shared,
      const RiskBudgetV2& risk) const;

  // Additive P0-07 overload: returns the exact in-call proof payload even
  // when the served result fails closed before it can enter the ordinary
  // valid-result proof registry. Existing signatures/layouts stay unchanged.
  ProtectionLevelV2Result computeShared(
      const LinearizedIntegrityWindow& window,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      std::vector<FaultHypothesisV2>* remaining_hypotheses,
      const ProtectionLevelSharedContext& shared,
      const RiskBudgetV2& risk,
      ProtectionLevelV2ProofV1* computation_audit) const;

  // Phase 2 of R11: prepare all candidate roots, execute one flat Cartesian
  // candidate×hypothesis task list, then reduce in canonical slot order.
  // Root/precondition failures still receive terminal no-op leaf slots and
  // fail closed after the common barrier.
  void computeSharedFlatBatch(
      const FrozenWindowAdmission& admission,
      std::vector<FlatProtectionCandidateV1>* candidates,
      const RiskBudgetV2& risk,
      CandidateWorkerPool* worker_pool,
      std::size_t active_workers,
      std::size_t scratch_limit_bytes) const;
  // The exhaustive mode computes the original raw-H response for every
  // hypothesis. Both modes retain all covariance, Gram, nullspace and proof
  // checks. Stats cover this synchronous call and are reduced canonically.
  void computeSharedFlatBatch(
      const FrozenWindowAdmission& admission,
      std::vector<FlatProtectionCandidateV1>* candidates,
      const RiskBudgetV2& risk,
      CandidateWorkerPool* worker_pool,
      std::size_t active_workers,
      std::size_t scratch_limit_bytes,
      bool enable_mode_response_cache,
      ProtectionModeResponseCacheStatsV1* stats = nullptr) const;
  ProtectionLevelV2Result computeShared(
      const FrozenWindowAdmission& admission,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      std::vector<FaultHypothesisV2>* remaining_hypotheses,
      const ProtectionLevelSharedContext& shared,
      const RiskBudgetV2& risk,
      ProtectionLevelV2ProofV1* computation_audit,
      AttemptProofArena* proof_arena) const;
  ProtectionLevelV2Result computeShared(
      const LinearizedIntegrityWindow& window,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      std::vector<FaultHypothesisV2>* remaining_hypotheses,
      const ProtectionLevelSharedContext& shared,
      const RiskBudgetV2& risk,
      ProtectionLevelV2ProofV1* computation_audit,
      AttemptProofArena* proof_arena) const;
  ProtectionLevelV2Result computeShared(
      const FrozenWindowAdmission& admission,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      std::vector<FaultHypothesisV2>* remaining_hypotheses,
      const ProtectionLevelSharedContext& shared,
      const RiskBudgetV2& risk,
      ProtectionLevelV2ProofV1* computation_audit) const;

  // Exact KEEP_ALL fast path.  Only the portions mathematically identical to
  // all-in evidence are reused; PL keeps its own rank semantics recorded in
  // FrozenHypothesisPlEntry.  Any identity mismatch returns an invalid result
  // and the caller must use the ordinary candidate-specific path.
  ProtectionLevelV2Result computeFrozenAllIn(
      const LinearizedIntegrityWindow& window,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      const std::vector<FaultHypothesisV2>& remaining_hypotheses,
      const std::vector<FaultModeBasis>& modes,
      const FrozenHypothesisNumerics& frozen,
      std::uint64_t fault_model_policy_fingerprint,
      const RiskBudgetV2& risk) const;
  ProtectionLevelV2Result computeFrozenAllIn(
      const FrozenWindowAdmission& admission,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      const std::vector<FaultHypothesisV2>& remaining_hypotheses,
      const std::vector<FaultModeBasis>& modes,
      const FrozenHypothesisNumerics& frozen,
      const FrozenHypothesisDualNumerics& frozen_dual,
      std::uint64_t fault_model_policy_fingerprint,
      const RiskBudgetV2& risk,
      AttemptProofArena* proof_arena) const;
  // P1-04 bounded-output overload: frozen hypothesis inputs may remain in the
  // attempt arena while the per-action full proof is written to a short-lived
  // batch arena. A null input arena preserves the legacy registry source.
  ProtectionLevelV2Result computeFrozenAllIn(
      const FrozenWindowAdmission& admission,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      const std::vector<FaultHypothesisV2>& remaining_hypotheses,
      const std::vector<FaultModeBasis>& modes,
      const FrozenHypothesisNumerics& frozen,
      const FrozenHypothesisDualNumerics& frozen_dual,
      std::uint64_t fault_model_policy_fingerprint,
      const RiskBudgetV2& risk,
      const AttemptProofArena* frozen_proof_arena,
      AttemptProofArena* output_proof_arena) const;
  ProtectionLevelV2Result computeFrozenAllIn(
      const FrozenWindowAdmission& admission,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      const std::vector<FaultHypothesisV2>& remaining_hypotheses,
      const std::vector<FaultModeBasis>& modes,
      const FrozenHypothesisNumerics& frozen,
      const FrozenHypothesisDualNumerics& frozen_dual,
      std::uint64_t fault_model_policy_fingerprint,
      const RiskBudgetV2& risk) const;

  static double detectionBoundaryNoncentralitySquared(
      int dof, double squared_threshold, double p_md);

 private:
  ProtectionLevelV2Result computeFrozenAllInImpl(
      const LinearizedIntegrityWindow& window,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      const std::vector<FaultHypothesisV2>& remaining_hypotheses,
      const std::vector<FaultModeBasis>& modes,
      const FrozenHypothesisNumerics& frozen,
      const FrozenHypothesisDualNumerics* frozen_dual,
      std::uint64_t fault_model_policy_fingerprint,
      const RiskBudgetV2& risk,
      const FrozenWindowAdmission* admission,
      const AttemptProofArena* frozen_proof_arena,
      AttemptProofArena* output_proof_arena) const;
  ProtectionLevelV2Result computeSharedImpl(
      const LinearizedIntegrityWindow& window,
      CandidateEvaluation* candidate,
      const DetectorResultV2& detector,
      std::vector<FaultHypothesisV2>* remaining_hypotheses,
      const ProtectionLevelSharedContext& shared,
      const RiskBudgetV2& risk,
      ProtectionLevelV2ProofV1* computation_audit,
      const FrozenWindowAdmission* admission,
      AttemptProofArena* proof_arena) const;
};

}  // namespace uwb_imu_pl
