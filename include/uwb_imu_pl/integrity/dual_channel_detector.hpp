#pragma once

// C2 (§7.3/§7.4/§7.5 + §8.2): separated residual detection layout, dual-channel
// protection-level bound, fault-span projection and the model-error channel.
//
// Contract (frozen design, history-summary-design.md §8):
//
// * Channel `current` is the residual of the *current system*, which INCLUDES
//   the condensed history boundary factor; it is never "new data only".  With
//   the C1 summary the state-supported rows [R_b | T_b] and the surrounding
//   measurement blocks carry it.
// * Channel `history` is the state-eliminated residual of the summary: the
//   detector-only rows [0 | F_b] contribute exactly `kappa_b` (+ the recorded
//   information-form constant offset) with `nu_perp` degrees of freedom.
// * Joint acceptance is the intersection A = {T_c <= tau_c} AND
//   {T_b <= tau_b}; the pooled aggregate remains available as the short-series
//   reference implementation (C1) but is not the product decision.
//
// Nothing here changes a threshold: each channel is tested against the
// *existing* false-alarm budget, and the platform-level union bound over the
// channels is reported explicitly instead of being hidden.

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"

namespace uwb_imu_pl {

// ---------------------------------------------------------------------------
// §7.3 separated residual layout.
// ---------------------------------------------------------------------------
struct ChannelTest {
  std::string detector_id;
  int dof = 0;             // nu_j
  double statistic = 0.0;  // T_j
  double threshold = 0.0;  // tau_j (from the unchanged per-test p_fa)
  double p_fa = 0.0;
  bool numerically_valid = false;
  bool accepted = false;   // T_j <= tau_j
};

struct DualChannelDecision {
  ChannelTest current;  // channel 1: current system incl. history boundary
  ChannelTest history;  // channel 2: eliminated history residual
  bool joint_accepted = false;
  bool numerically_valid = false;
  // Union bound over the channels with the unchanged per-test budget: reported
  // so the platform accounting cannot silently double the false-alarm claim.
  double operation_p_fa_upper_bound = 0.0;
  int channel_count = 0;
  std::string reason;
};

// Splits the frozen window into the two residual channels.  The split is exact:
// T_current + T_history == T_pooled and nu_current + nu_history == nu_pooled.
DualChannelDecision evaluateDualChannel(
    const LinearizedIntegrityWindow& window, double p_fa_per_test,
    std::uint32_t continuity_horizon_tests);

// ---------------------------------------------------------------------------
// §7.4 dual-channel bound (independent formula review item).
// ---------------------------------------------------------------------------
//
//   lambda_{h,j} = f_h' Gamma_{h,j} f_h                    (per channel)
//   Lambda_{h,j} solves F_{chi2_{nu_j}(Lambda)}(tau_j) <= beta_h   (conservative)
//   W_h = sum_j w_j Gamma_{h,j} / Lambda_{h,j},   w_j > 0, sum_j w_j = 1
//   ker W_h subseteq ker G_h
//   b_{h,d} = sqrt(g_{h,d} W_h^dagger g_{h,d}')
//   L_{h,d} = b_{h,d} + k_{h,d} sigma_d
//
// Implementation: the channel maps are scaled by sqrt(w_j / Lambda_{h,j}) and
// stacked, so W_h is only ever used through small factorizations -- no explicit
// inverse is formed.  A channel with no degrees of freedom or no fault
// contribution may NOT fabricate a Lambda: it is dropped from the sum, and if
// no channel survives the bound is unavailable (fail-closed).  A single valid
// channel degenerates exactly to the B3 form s*sqrt(Lambda).
struct ChannelBoundInput {
  std::string detector_id;
  int dof = 0;                    // nu_j
  double threshold = 0.0;         // tau_j (after the §8.2 model-error channel)
  double actual_threshold = 0.0;  // tau_{j,actual} before the inflation
  double weight = 0.0;            // w_j
  Eigen::MatrixXd gram;           // Gamma_{h,j}: parameter_dim x parameter_dim
  // Residual model-error inflation (sqrt(tau) + rho)^2; zero means no claim.
  double residual_rho = 0.0;
  // Whether rho comes from a validated model (false => labelled assumption).
  bool rho_validated = false;
  std::string rho_source;
};

struct DualChannelBoundResult {
  bool valid = false;
  std::string reason;
  // Per-axis halves of the bound: b_{h,d}.
  Eigen::Vector3d axis_bound_m = Eigen::Vector3d::Zero();
  // Certified per-channel non-centrality multipliers sqrt(Lambda_{h,j}).
  std::vector<double> channel_lambda;
  std::vector<double> channel_weight;
  std::vector<double> channel_lambda_gap;  // F_{chi2}(tau) - beta, must be <= 0
  std::vector<int> channel_dof;
  std::vector<double> channel_threshold;
  std::string certificate_id;
  // W_h is exposed for audit only (the feasibility check and the proof id);
  // consumers use the per-axis bounds.
  Eigen::MatrixXd w_matrix;
  double w_rank = 0.0;
  bool w_kernel_covered = false;  // ker W_h subseteq ker G_h
  // Residual model-error accounting (§8.2), per axis and per channel.
  Eigen::Vector3d position_rho_m = Eigen::Vector3d::Zero();
  bool model_error_validated = true;
  std::string model_error_source;
};

struct DualChannelBoundRequest {
  std::vector<ChannelBoundInput> channels;
  // G_h: protected-state response rows (rows x parameter_dim), one row per
  // protected axis, in the same parameter space as the Grams.
  Eigen::MatrixXd protected_response;
  // Charged miss probability beta_h and the per-axis tail multiplier k_{h,d}.
  double p_md = 0.0;
  Eigen::Vector3d k_axis = Eigen::Vector3d::Zero();
  // Model-error position terms rho_{p,d} (added per axis).
  Eigen::Vector3d position_rho_m = Eigen::Vector3d::Zero();
  bool position_rho_validated = false;
  std::string position_rho_source;
};

DualChannelBoundResult computeDualChannelBound(
    const DualChannelBoundRequest& request);

// Versioned P0-03 proof sidecar.  Existing request/result layouts are left
// byte-for-byte compatible with the previous golden ABI.
struct DualChannelNumericalProofV1 {
  std::uint64_t schema_version = 1;
  DualChannelBoundRequest request;
  double rank_tolerance = 1e-10;
  std::uint64_t parent_proof_identity = 0;
  Eigen::MatrixXd w_matrix;
  Eigen::VectorXd w_eigenvalues;
  Eigen::MatrixXd w_eigenvectors;
  Eigen::VectorXd w_eigenvalue_errors;
  DualChannelBoundResult served_result;
  std::uint64_t proof_identity = 0;
};

DualChannelBoundResult computeDualChannelBoundCertified(
    const DualChannelBoundRequest& request, double rank_tolerance,
    std::uint64_t parent_proof_identity,
    DualChannelNumericalProofV1* proof);
bool validateDualChannelNumericalProof(
    const DualChannelNumericalProofV1& proof,
    std::string* reason = nullptr);

// ---------------------------------------------------------------------------
// §7.5 fault-span projection.
// ---------------------------------------------------------------------------
// `U` must cover EVERY retained historical fault direction inside the declared
// span -- never a top-K selection by observed fault.  The projection is only
// admissible when Z_b' Z_b == F_all' F_all holds exactly (to rounding).
struct FaultSpanProjectionResult {
  bool valid = false;
  std::string reason;
  Eigen::MatrixXd basis;            // U (parameter_dim x kept)
  std::size_t retained_directions = 0;
  std::size_t declared_directions = 0;
  double exactness_residual = 0.0;
  bool declared_span_gap = false;   // some declared direction was NOT retained
};

FaultSpanProjectionResult buildFaultSpanProjection(
    const Eigen::MatrixXd& declared_directions, double tolerance);

// Compares the projected history Gram with the unprojected one, i.e. the
// §7.5 identity Z_b' Z_b == F_all' F_all on actual matrices.
FaultSpanProjectionResult evaluateFaultSpanIdentity(
    const Eigen::MatrixXd& full_response,     // F_all (rows x dim)
    const Eigen::MatrixXd& projected_basis,   // U (dim x kept)
    double tolerance);

// ---------------------------------------------------------------------------
// §8.2 model-error channel.
// ---------------------------------------------------------------------------
// tau_{j,risk} = (sqrt(tau_{j,actual}) + rho_{r,j})^2 ; rho = 0 degenerates to
// the standard chi-square threshold.  rho must come from a validated model or
// from a labelled, explicitly unverified assumption; the state travels in the
// certificate.
double riskAdjustedThreshold(double actual_threshold, double residual_rho);

}  // namespace uwb_imu_pl
