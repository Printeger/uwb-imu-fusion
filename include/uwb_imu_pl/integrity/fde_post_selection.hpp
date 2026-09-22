#pragma once

// C3 (§8.4/§8.5/§8.3): FDE profile-likelihood evidence, post-FDE conservative
// selection guarantees and the bridge/IMU decision order.
//
// Contract (roadmap §8.3–8.5, frozen design history-summary-design.md §8):
//
// * §8.4 evidence is computed from the RAW whitened likelihood -- the same
//   quantities the C1/C2 channels use (state-supported map Z_c, channel-1
//   residual r_c, detector-only constant kappa_b, cross terms Omega/xi).  The
//   C2 risk-scaled quantities (w_j, Lambda_{h,j}, W_h) must NOT enter here.
//     J_h^profile = ||r_c||^2 + kappa_b - t_h' Gamma_{h,full}^dagger t_h
//     t_h         = Z_{h,c}' r_c + xi_h
//     Gamma_full  = Z_{h,c}' Z_{h,c} + Omega_h
// * Candidate pools keep STRUCTURED fault explanations (unit, parameter
//   dimension, physical source, action) and may not rank candidates with
//   different units or parameter dimensions against each other without an
//   explicit declaration.
// * §8.5 selection risk is controlled over EVERY action that may be published,
//   not only the winner: actions that share a reference certificate / accepted
//   event / time / output quantity and carry a triangle-transfer proof form one
//   `GuaranteeGroup` and share a single failure event; anything else is a
//   singleton whose budget is charged separately.  If no candidate satisfies the
//   requirement the platform stays unavailable -- standards are never relaxed.

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

#include "uwb_imu_pl/integrity/fault_model.hpp"

namespace uwb_imu_pl {

// ---------------------------------------------------------------------------
// §8.4 profile-likelihood evidence.
// ---------------------------------------------------------------------------
// FaultUnitKind lives in fault_model.hpp (shared with the evidence structs).

struct ProfileEvidenceInput {
  // Gamma_{h,full} = Z_c'Z_c + Omega (parameter_dim x parameter_dim), raw.
  Eigen::MatrixXd fault_gram;
  // t_h = Z_{h,c}' r_c + xi_h (parameter_dim).
  Eigen::VectorXd t;
  // Channel-1 (state-supported) residual energy and the RAW detector-only
  // constant; both are the un-scaled channel quantities.
  double channel_current_residual = 0.0;
  double kappa_b = 0.0;
  FaultUnitKind unit = FaultUnitKind::Unknown;
  std::size_t parameter_dim = 0;
  std::string source_id;
};

struct ProfileEvidence {
  bool valid = false;
  std::string reason;
  double j_profile = std::numeric_limits<double>::infinity();
  double explained_energy = 0.0;  // t' Gamma^dagger t
  double t_norm = 0.0;
  int rank = 0;                   // adjudicated rank of Gamma_{h,full}
  double condition_proxy =
      std::numeric_limits<double>::infinity();  // sigma_max / sigma_min(kept)
  // Raw constant actually used, kept so a consumer can audit that no C2 risk
  // scaling leaked into the evidence.
  double constant_used = 0.0;
};

ProfileEvidence profileLikelihoodEvidence(const ProfileEvidenceInput& input);

// ---------------------------------------------------------------------------
// Structured candidate pool.
// ---------------------------------------------------------------------------
struct StructuredCandidate {
  HypothesisId hypothesis;
  FaultUnitKind unit = FaultUnitKind::Unknown;
  std::size_t parameter_dim = 0;
  std::string physical_source_id;
  double j_profile = std::numeric_limits<double>::infinity();
  double explained_energy = 0.0;
  std::vector<FactorGroupId> groups_to_remove;
  std::vector<FactorGroupId> groups_to_add;
  bool bounded_model = false;  // validated random/bounded model available
  bool centre_only = false;    // usable for the estimate centre only
  bool has_valid_reference = false;
  bool propagation_bound_available = false;
};

struct CandidateRanking {
  bool valid = false;
  std::string reason;
  // Sorted best-first; only populated when the pool is comparable.
  std::vector<std::size_t> order;
  // True when the pool mixes units/dimensions; such a pool may not be ranked.
  bool mixed_units = false;
  bool mixed_dimensions = false;
};

// Ranks a structured pool.  Mixing fault units (metres vs m/s^2 vs rad/s) or
// parameter dimensions is refused explicitly instead of silently comparing
// incomparable numbers.
CandidateRanking rankStructuredCandidates(
    const std::vector<StructuredCandidate>& candidates);

// ---------------------------------------------------------------------------
// §8.5 post-FDE conservative selection guarantee.
// ---------------------------------------------------------------------------
struct ActionGuarantee {
  std::uint64_t action_id = 0;
  std::uint64_t reference_certificate_id = 0;
  // Shared-evidence flags: a group may only reuse the reference failure event
  // when all of them hold together with the triangle-transfer proof.
  bool shared_reference_certificate = false;
  bool shared_accepted_event = false;
  bool shared_time = false;
  bool shared_output_quantity = false;
  bool triangle_transfer_evidence = false;
  Eigen::Vector3d L_reference_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d L_action_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_action_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_reference_m = Eigen::Vector3d::Zero();
  // eps_{a,h}: the budget this action would charge if it were published.
  double epsilon_budget = 0.0;
};

struct GuaranteeGroup {
  std::uint64_t group_id = 0;
  std::uint64_t reference_certificate_id = 0;
  std::vector<std::uint64_t> action_ids;
  // The group failure event is the shared reference event; the budget charged
  // to it is the largest per-action budget inside the group (one shared event).
  double charged_budget = 0.0;
  // Triangle identity residual: ||(L_ref + |p_a - p_ref|) - L_a|| relative.
  double worst_triangle_residual = 0.0;
};

struct GuaranteeGroupResult {
  bool valid = false;
  std::string reason;
  std::vector<GuaranteeGroup> groups;
  double total_charged_budget = 0.0;
  std::size_t singleton_groups = 0;
  std::size_t shared_groups = 0;
};

// Builds the guarantee groups and charges the union budget.  `available_budget`
// is the platform's selection-risk allocation; exceeding it refuses (the caller
// must then keep the platform unavailable, never relax the standard).
GuaranteeGroupResult buildGuaranteeGroups(
    const std::vector<ActionGuarantee>& actions, double available_budget);

// ---------------------------------------------------------------------------
// §8.3 bridge / candidate decision order.
// ---------------------------------------------------------------------------
enum class CandidateMode {
  ValidatedModel = 0,   // validated random / bounded model -> reference estimate
  CentreOnly = 1,       // usable only for the estimate centre
  Unusable = 2,         // no valid reference, no transfer bound
};

enum class CandidateDisposition {
  UseInReferenceEstimate = 0,  // enters the reference estimate + dual envelope
  TransferToValidReference = 1,  // protection transferred at the same instant
  UnprotectedDiagnosticOnly = 2,  // diagnostics only, explicitly unprotected
  Refused = 3,                   // no admissible handling
};

struct CandidateDecision {
  CandidateDisposition disposition = CandidateDisposition::Refused;
  bool integrity_available = false;
  bool uses_bounded_model = false;  // never treated as Gaussian information
  std::string reason;
};

CandidateDecision decideCandidateHandling(const StructuredCandidate& candidate);

}  // namespace uwb_imu_pl
