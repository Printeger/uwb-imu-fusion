#pragma once

#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"

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

bool validateProtectionLevelV2ProofPayload(
    const ProtectionLevelV2ProofV1& proof,
    std::string* reason = nullptr);

bool validateProtectionLevelV2Proof(
    const CandidateEvaluation& candidate, const DetectorResultV2& detector,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const ProtectionLevelV2Result& result,
    std::string* reason = nullptr);

bool protectionLevelV2Proof(const ProtectionLevelV2Result& result,
                            ProtectionLevelV2ProofV1* proof);
std::uint64_t protectionLevelV2ProofIdentity(
    const ProtectionLevelV2Result& result);
std::uint64_t protectionLevelV2ProofIdentity(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result);
bool bindProtectionLevelPublicationPacket(
    const CandidateEvaluation& candidate,
    const ProtectionLevelV2Result& result,
    std::uint64_t protection_proof_identity,
    std::string* packet_id);
bool protectionLevelPublicationPacket(
    const std::string& packet_id,
    ProtectionLevelPublicationPacketV1* packet);
std::uint64_t protectionLevelPublicationProofIdentity(
    const Eigen::Vector3d& served_pl,
    const std::string& detector_certificate_id);
bool validateProtectionLevelPublicationProof(
    std::uint64_t proof_identity,
    const Eigen::Vector3d& served_pl,
    const std::string& detector_certificate_id,
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

  static double detectionBoundaryNoncentralitySquared(
      int dof, double squared_threshold, double p_md);
};

}  // namespace uwb_imu_pl
