#pragma once

// C1-a: module-level construction of the fault-preserving history summary
// (square-root form).  Frozen contract: history-summary-design.md §1-§2.
// NOT wired into the production pipeline in C1-a.
//
// Input: a whitened linear block system
//
//     || H_o x_o + H_b x_b + A f - z ||²
//
// with old state x_o (H_o), separator/boundary state x_b (H_b), fault map
// A, and right-hand side z.  The output (R_b, T_b, d_b, F_b, d_perp)
// satisfies, for all (x_b, f),
//
//     min_{x_o} || H_o x_o + H_b x_b + A f - z ||²
//         == || R_b x_b + T_b f - d_b ||² + || F_b f - d_perp ||²
//
// i.e. the reduced block system (design freeze §2):
//
//     [ R_b  T_b ] [ x_b ]   [ d_b    ]
//     [ 0    F_b ] [ f   ] = [ d_perp ].
//
// Construction policy (frozen): orthogonal elimination only.  Stage A
// eliminates x_o with Householder reflectors; stage B splits the remaining
// rows into rows carrying x_b support (-> R_b, T_b, d_b, upper triangular)
// and rows without x_b support (-> F_b, d_perp).  Normal equations and
// explicit inverses are forbidden; the output stays in square-root form.
//
// Sign convention (frozen, tested):
//   * boundaryMeanShiftForFault(f)    = R_b⁻¹ T_b f       (positive
//     injection response of the boundary mean),
//   * conditionalBoundaryMeanDelta(f) = -R_b⁻¹ T_b f      (boundary mean
//     conditioned on f, relative to the marginal).
// The two functions are negatives of each other for the same (R_b, T_b, f).
//
// Audit views (recomputed on demand from F_b / d_perp, never part of the
// solve): Omega_b = F_bᵀ F_b, xi_b = F_bᵀ d_perp, kappa_b = ‖d_perp‖².
// The carrier [F_b | d_perp] is orthogonally compressed after Stage B.
// nu_perp is its certified effective numerical rank, not the number of raw
// residual storage rows and not a count of element-wise nonzero rows.
//
// Degenerate handling (explicit, never silent; no numeric output on
// rejection):
//   * non-finite input                       -> invalid "non_finite_input"
//   * inconsistent shapes                    -> invalid "shape_mismatch"
//   * empty system (no rows)                 -> invalid "empty_input"
//   * rank(H_o) < n_o, structural or not
//     safely adjudicable                     -> invalid "h_o_rank_deficient"
// A rank-deficient boundary block is NOT a rejection: the identity still
// holds (R_b is then singular, possibly with zero rows).  `rank_boundary`
// reports the adjudicated rank; the sign-convention functions return a
// NaN-filled vector when R_b is not usable (so nothing is silently misused).
// q == 0 yields correctly sized zero-column T_b / F_b (nominal boundary with
// no fault response).  Detection-only rows (no x_b support but nonzero A/z)
// are carried into F_b / d_perp whenever they are not needed to fill the
// boundary triangle; they are never dropped, and min-cost information is
// preserved in every case (identity above).
//
// C1-b input mapping (expected; adapter NOT wired this round).  The future
// caller (`incremental_estimator.cpp::buildIntegrityWindow`, boundary segment
// ~1187-1270) will assemble the input from the multifrontal elimination of
// the frozen graph (`boundary_graph.linearize(frozen_values)` +
// `eliminatePartialMultifrontal(..., EliminateQR)`):
//   * H_o <- whitened Jacobian columns of the old-state variables being
//     eliminated/summarized (reduced factor rows from the eliminated history,
//     restricted through `LinearizedIntegrityWindow::blocks[]`,
//     `window_column_indices`),
//   * H_b <- columns of the separator/boundary state that stays explicit,
//   * A   <- fault columns added to the linearization *before* elimination
//     (shared mode maps with `HypothesisGenerator`; only state variables are
//     eliminated, never fault columns),
//   * z   <- whitened right-hand side of those same rows.
// Row lineage reuses `slot_accounting` / `frozen_slots`; the summary version
// must enter the `FrozenWindowNumerics` fingerprint and the
// `StatisticalBoundKey` (new `history_summary_version` key) so cached
// windows cannot silently reuse a stale summary.  nu_perp feeds the pooled
// detection degrees of freedom (nu_pooled = nu_c + nu_perp).  This module is
// not called from the pipeline in C1-a; production wiring is C1-b.
//
// Status: C1-a module level implemented, locked by `HistoryFaultSummary.*`
// tests (see doc/evidence/integrity-kernel-refactor/history-summary-module.md).
// C1-b (pipeline wiring, lifecycle, capacity, cold start) and C1-c: NOT_RUN.

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace uwb_imu_pl {

struct HistoryFaultSummaryInput {
  Eigen::MatrixXd h_old_state;  // H_o: m x n_o
  Eigen::MatrixXd h_boundary;   // H_b: m x n_b
  Eigen::MatrixXd fault_map;    // A:   m x q
  Eigen::VectorXd rhs;          // z:   m

  int rows() const { return static_cast<int>(rhs.size()); }
  int oldStateColumns() const { return static_cast<int>(h_old_state.cols()); }
  int boundaryColumns() const { return static_cast<int>(h_boundary.cols()); }
  int faultColumns() const { return static_cast<int>(fault_map.cols()); }
};

// Splits a block matrix [H_o | H_b | A | z] with the given column counts.
// Returns nullopt when the block does not have exactly
// n_old_state + n_boundary + n_fault + 1 columns.
std::optional<HistoryFaultSummaryInput> splitBlockSystem(
    const Eigen::MatrixXd& block, int n_old_state, int n_boundary, int n_fault);

struct HistoryFaultSummaryOptions {
  // Relative pivot floor used to adjudicate rank(H_o) and to report the
  // boundary rank.  Pivot magnitudes from the Householder reduction are
  // compared against `rank_tolerance * max_pivot`.  A structural rank
  // deficiency (zero/duplicate columns, n_o > m) falls below any positive
  // floor; a full-rank block must stay above it (accepted inputs are
  // κ(H_o) <~ 1/rank_tolerance).  Not a production threshold: this module is
  // not wired into any published output yet (that decision belongs to
  // C1-b and its validation).
  double rank_tolerance = 1e-12;
};

// Conditioning-aware proof for the effective residual dimension.  The raw
// Stage-B carrier C=[F_b|d_perp] has `storage_rows_before_compression` rows.
// Its singular directions are retained exactly when sigma > rank_threshold;
// directions at or below the threshold are certified numerical zero and are
// removed by an orthogonal SVD compression.  `discarded_frobenius_bound` is
// the norm of precisely those discarded directions.  The proof identity
// binds dimensions, tolerance, threshold, error bound and every singular
// value, so downstream certificates cannot silently consume a different dof.
struct HistoryCarrierRankCertificate {
  bool valid = false;
  int storage_rows_before_compression = 0;
  int raw_residual_dof = 0;
  int effective_rank = 0;
  double rank_tolerance = 0.0;
  double factor_scale = 0.0;
  double roundoff_error_bound = 0.0;
  double rank_threshold = 0.0;
  double discarded_frobenius_bound = 0.0;
  Eigen::VectorXd singular_values;
  std::uint64_t proof_identity = 0;
};

struct HistoryFaultSummary {
  bool valid = false;
  std::string invalid_reason;  // non-empty iff !valid

  // Original problem dimensions.
  int n_rows = 0;            // m
  int n_old_state = 0;       // n_o
  int n_boundary = 0;        // n_b
  int n_fault = 0;           // q
  int rank_h_old_state = 0;  // adjudicated rank(H_o); == n_o when valid
  int rank_boundary = 0;     // adjudicated rank of R_b (informational)
  // Adjudicated pivot ratio min|R_ii| / max|R_ii| of the H_o reduction
  // (1.0 when n_o == 0, 0.0 when the reduction scale is zero).  The rank
  // decision compares the individual pivots against
  // `rank_tolerance * max|R_ii|`, so this ratio is the direct distance to
  // the acceptance gate and is kept for audit even when the input is
  // rejected as rank deficient.
  double old_state_pivot_ratio = 0.0;

  // Square-root summary, in the original x_b / f column order.  When the
  // boundary rows are fewer than n_b columns, the trailing rows of
  // R_b / T_b / d_b are exact zeros (the identity still holds).
  Eigen::MatrixXd R_b;     // n_b x n_b, upper triangular
  Eigen::MatrixXd T_b;     // n_b x q
  Eigen::VectorXd d_b;     // n_b
  Eigen::MatrixXd F_b;     // nu_perp x q
  Eigen::VectorXd d_perp;  // nu_perp

  // Audit views (on demand; never stored or inverted in the solve).
  Eigen::MatrixXd omegaBoundary() const { return F_b.transpose() * F_b; }
  Eigen::VectorXd xiBoundary() const { return F_b.transpose() * d_perp; }
  double kappaBoundary() const { return d_perp.squaredNorm(); }
  int nuPerp() const { return static_cast<int>(d_perp.size()); }

  // Sign convention (§2 frozen).  boundaryShiftUsable() is false when the
  // summary is invalid or R_b is rank deficient; the two functions then
  // return NaN-filled vectors instead of a meaningless solve.
  bool boundaryShiftUsable() const {
    return valid && (n_boundary == 0 || rank_boundary == n_boundary);
  }
  Eigen::VectorXd boundaryMeanShiftForFault(
      const Eigen::VectorXd& f) const;  // R_b⁻¹ T_b f
  Eigen::VectorXd conditionalBoundaryMeanDelta(
      const Eigen::VectorXd& f) const;  // -R_b⁻¹ T_b f
};

HistoryFaultSummary buildHistoryFaultSummary(
    const HistoryFaultSummaryInput& input,
    const HistoryFaultSummaryOptions& options = {});

// Additive ABI-stable API: the golden HistoryFaultSummary layout is unchanged;
// callers that need the effective-rank proof request this wrapper explicitly.
struct CertifiedHistoryFaultSummary {
  HistoryFaultSummary summary;
  HistoryCarrierRankCertificate carrier_rank;
};

CertifiedHistoryFaultSummary buildCertifiedHistoryFaultSummary(
    const HistoryFaultSummaryInput& input,
    const HistoryFaultSummaryOptions& options = {});

// R09 dependency-closure implementation. Factor-group leaves retain exact raw-row
// provenance and semantic column identities; a deterministic orthogonal tree
// recomputes only paths containing byte-changed leaves. Ordering, whitening,
// recovery, or immutable-owner changes rebuild the tree. Both the incremental
// path and a monolithic rebuild apply the same certified effective-rank carrier
// contract above; raw provenance and raw residual dof remain audit metadata.
struct HistoryRootCacheRequest {
  std::shared_ptr<const void> owner;
  const void* owner_payload = nullptr;
  std::vector<std::string> row_uids;
  std::vector<std::uint64_t> row_provenance;
  std::vector<std::string> state_column_uids;
  std::vector<std::string> fault_column_uids;
  std::uint64_t linearization_version = 0;
  std::uint64_t ordering_version = 0;
  std::uint64_t whitening_version = 0;
  std::uint64_t marginalization_version = 0;
  std::uint64_t recovery_version = 0;
  std::uint64_t column_order_digest = 0;
  bool verify_full_oracle = false;  // test-only; never selects published data
  HistoryFaultSummaryInput input;
};

struct HistoryRootCacheAudit {
  std::uint64_t requests = 0;
  std::uint64_t exact_hits = 0;
  std::uint64_t incremental_appends = 0;
  std::uint64_t incremental_group_updates = 0;
  std::uint64_t incremental_path_updates = 0;
  std::uint64_t incremental_add_paths = 0;
  std::uint64_t incremental_remove_paths = 0;
  std::uint64_t incremental_relinearize_paths = 0;
  std::uint64_t internal_nodes_recomputed = 0;
  std::uint64_t full_tree_rebuilds = 0;
  std::uint64_t full_oracle_checks = 0;
  std::uint64_t full_oracle_mismatches = 0;
  double max_oracle_relative_error = 0.0;
  std::uint64_t reused_groups = 0;
  std::uint64_t rebuilt_groups = 0;
  std::uint64_t factor_group_hits = 0;
  std::uint64_t factor_group_misses = 0;
  std::uint64_t full_rebuilds = 0;
  std::uint64_t invalidations = 0;
  std::size_t retained_rows = 0;
  std::size_t retained_bytes = 0;
  std::string last_reason;
};

class IncrementalHistoryRootCache {
 public:
  HistoryFaultSummary update(
      const HistoryRootCacheRequest& request,
      const HistoryFaultSummaryOptions& options = {});
  HistoryFaultSummary update(
      const HistoryRootCacheRequest& request,
      HistoryCarrierRankCertificate* carrier_rank,
      const HistoryFaultSummaryOptions& options = {});
  void invalidate(const std::string& reason);
  const HistoryRootCacheAudit& audit() const { return audit_; }

 private:
  struct Entry;
  std::shared_ptr<Entry> entry_;
  HistoryRootCacheAudit audit_;
};

// C1-c/C3: binding identity of a history summary (design freeze §3/§5).  The
// component digests are filled by the producer when a summary is built:
//   * linearization - linearization point / local-coordinate identity,
//   * whitening     - whitening model identity,
//   * mode_set      - the monitored fault mode set (anchors / onsets / ramps),
//   * capacity      - the capacity policy and budget identity.
// The digest is what a consumer binds into caches (StatisticalBoundKey
// `history_summary_version`) and fingerprints.  A rebuild with ANY changed
// component therefore can never be served from a stale entry; changing a
// human-readable id alone is not a binding and is forbidden by the design.
// The pipeline that fills these components is C1-b and is NOT wired in this
// round (see the blocker record); this type and digest are the binding
// carrier the pipeline will use.
struct HistorySummaryVersion {
  std::uint64_t linearization = 0;
  std::uint64_t whitening = 0;
  std::uint64_t mode_set = 0;
  std::uint64_t capacity = 0;
};

// Deterministic 64-bit fold of the four components in fixed order.  Uses the
// same constants/order as the integrity-config hash (`fnv1a64`), so the
// convention is identical across the codebase.  Determinism and sensitivity
// to every component are the only properties claimed.
std::uint64_t digestHistorySummaryVersion(const HistorySummaryVersion& version);

}  // namespace uwb_imu_pl
