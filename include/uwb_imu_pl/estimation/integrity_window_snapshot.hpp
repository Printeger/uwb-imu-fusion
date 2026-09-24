#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "uwb_imu_pl/estimation/factor_ledger.hpp"
#include "uwb_imu_pl/estimation/square_root_context.hpp"
#include "uwb_imu_pl/integrity/history_fault_parameterization.hpp"
#include "uwb_imu_pl/integrity/history_fault_summary.hpp"

namespace uwb_imu_pl {

struct StateLayoutEntry {
  std::size_t epoch = 0;
  std::vector<gtsam::Key> keys;
  int column_offset = 0;
  int dimension = 0;
  bool protected_current_state = false;
};

struct LinearizedFactorBlock {
  FactorGroupId group_id;
  FactorKind kind = FactorKind::Unknown;
  SensorType sensor = SensorType::Unknown;
  RowRole role = RowRole::Measurement;
  Eigen::MatrixXd jacobian_raw;
  Eigen::VectorXd residual_raw;
  Eigen::MatrixXd covariance;
  Eigen::MatrixXd whitener;
  Eigen::MatrixXd jacobian_whitened;
  Eigen::VectorXd residual_whitened;
  std::vector<int> window_column_indices;
  std::vector<FaultUnitId> fault_units;
  double effective_weight = 1.0;
  std::string whitening_model_id;
  LinearizationVersion version;
};

struct WindowCapabilities {
  bool includes_boundary_prior = false;
  bool includes_pending_imu = false;
  bool includes_pending_uwb = false;
  bool complete_factor_provenance = false;
  bool history_provenance_valid = false;
  bool fixed_lag_maturity_valid = false;
  bool no_duplicate_rows = false;
  bool every_active_factor_accounted_once = false;
  bool frozen_slot_identity_valid = false;
  // C1-b: the condensed boundary is the fault-preserving history summary
  // (square-root form), not the legacy eigendecomposition prior.
  bool history_summary_present = false;
  bool history_summary_valid = false;
  bool history_summary_capacity_ok = true;
};

// C1-b: the fault-preserving history summary carried by a frozen window.
//
// The boundary factor block holds the condensed rows [R_b; 0] plus the
// detection-only rows [0 | d_perp] (the square_root_context classifies the
// latter automatically: zero Jacobian rows).  This carrier holds everything
// that is NOT a window row: the fault response (T_b over the boundary
// columns, F_b over the detection-only rows), the fault column identity
// (per (kind, source, epoch), D-1 granularity), the A3 horizon bookkeeping,
// the design-freeze binding digest and the capacity decision.  Mode maps for
// historical faults are linear combinations of `response`/`detector_response`
// columns (persistent = coefficient 1 on every epoch >= onset; ramp uses the
// time-linear companion basis, exact by construction).
struct WindowHistorySummary {
  bool present = false;  // the boundary graph had material to summarize
  bool valid = false;
  std::string reason;

  // Counts and audit views (kappa_b feeds the pooled statistic identity
  // T_pooled = ||r_c||^2 + kappa_b, nu_perp the pooled dof).
  std::size_t fault_columns = 0;
  std::size_t boundary_rows = 0;     // rows extracted from the boundary graph
  std::size_t boundary_columns = 0;  // x_b columns fed to the module
  std::size_t emitted_rows = 0;      // rows the boundary block contributes
  int rank_boundary = 0;
  int nu_perp = 0;
  double kappa_b = 0.0;
  double constant_energy = 0.0;  // route (i) keeps the full constant content
  std::size_t injected_epochs = 0;
  std::vector<HistoryFaultColumnId> column_ids;
  std::size_t constant_columns = 0;
  std::size_t time_linear_columns = 0;
  std::size_t imu_columns = 0;

  // Fault response in the condensed basis: residual = [R_b;0] x - [d_b;d_perp]
  // plus [T_b;F_b] f, so a mode map over the boundary block is the stacked
  // column [response.col(i); detector_response.col(i)].
  Eigen::MatrixXd response;           // T_b: boundary_columns x q
  Eigen::MatrixXd detector_response;  // F_b: nu_perp x q
  Eigen::VectorXd d_perp;             // nu_perp
  Eigen::MatrixXd omega() const {
    return detector_response.transpose() * detector_response;
  }
  Eigen::VectorXd xi() const { return detector_response.transpose() * d_perp; }

  // Route (i) residual-constant accounting.  `kappa_b` is the row-space
  // constant; `constant_offset` is the part of the constant that rows cannot
  // represent, contributed by information-form boundary factors (the fixed-lag
  // marginalization's LinearContainerFactors, whose rows are rebuilt by
  // Cholesky).  Both are exported so nothing is dropped silently: the total
  // detector constant is kappa_b + constant_offset.
  double constant_offset = 0.0;
  std::size_t information_form_factors = 0;

  // A3 horizon / no-overclaim bookkeeping (plan text exported verbatim).
  std::size_t horizon_first_epoch = 0;
  std::size_t window_first_epoch = 0;
  std::size_t omitted_epoch_count = 0;
  std::size_t material_gap_epoch_count = 0;
  bool claims_full_coverage = false;
  std::string assumptions;
  std::string omitted_risk_source;
  std::size_t skipped_columns = 0;

  // Design-freeze binding (§3/§5): what caches and fingerprints carry.
  std::string scope_digest;
  HistorySummaryVersion version;
  std::uint64_t version_digest = 0;

  // §7.6 row 7 / cold start: explicit state, never silent.
  bool capacity_ok = true;
  std::string capacity_action;
  // Explicit state name (HistorySummaryState, lifecycle module): one of
  // HISTORY_SUMMARY_{NOT_REQUIRED,VALID,INVALID,BUILD_INVALID} or
  // HISTORY_CAPACITY_EXCEEDED.  Diagnostics and the cold-start contract read
  // this instead of inferring a state from the counts.
  std::string state;
};

struct FactorSlotAccounting {
  std::size_t slot = 0;
  std::optional<FactorGroupId> group_id;
  bool explicit_window_block = false;
  bool boundary_input = false;
  bool pointer_identity_valid = false;
};

enum class FrozenFactorDisposition {
  ExplicitMeasurement,
  BoundaryInput,
  PendingExplicit,
  UnrecoverableHistory
};

// The single source of truth for assigning frozen factor groups to the
// explicit integrity window or its condensed boundary.  Entries are immutable
// once the window has been finalized; hypothesis/action construction must not
// independently infer membership from epoch inequalities.
struct FrozenWindowFactorInventoryEntry {
  FactorGroupId group_id;
  std::size_t epoch = 0;
  FactorKind kind = FactorKind::Unknown;
  SensorType sensor = SensorType::Unknown;
  FrozenFactorDisposition disposition =
      FrozenFactorDisposition::UnrecoverableHistory;
  std::vector<gtsam::Key> keys;
  std::vector<std::size_t> slots;
};

struct IntegrityWindowRequest {
  std::uint32_t epochs = 20;
  bool include_pending_imu = true;
  bool include_pending_uwb = true;
  bool include_generic_bridge = false;
};

// Every field that can change the meaning or acceptance of a shared base
// numerical result.  Store the values as well as the fingerprint so a dump or
// debugger can explain a mismatch without reverse-engineering the hash.
struct FrozenNumericalContract {
  std::uint64_t policy_version = 3;
  double rank_tolerance = 1e-10;
  double max_condition_number = 1e12;
  double solve_residual_limit = 1e-7;
  double forward_error_limit = 1e-7;
  double reference_relative_tolerance = 1e-7;
};

// Immutable numerical products tied to the complete frozen-window identity.
// Eigen decomposition objects are deliberately not serialized.
struct FrozenWindowNumerics {
  WindowId window_id;
  LinearizationVersion version;
  std::uint64_t content_fingerprint = 0;
  FrozenNumericalContract numerical_contract;
  std::uint64_t numerical_contract_fingerprint = 0;
  std::shared_ptr<const Eigen::LLT<Eigen::MatrixXd>> information_factorization;
  std::shared_ptr<const Eigen::MatrixXd> spectral_vectors;
  Eigen::VectorXd spectral_inverse_squared;
  // Reference (SVD) nominal increment; kept only for the certificate and as
  // the B1 fallback when the square-root context is not usable.
  Eigen::VectorXd spectral_state_increment;
  Eigen::VectorXd base_state_increment;
  Eigen::VectorXd parity;
  std::vector<Eigen::Index> block_row_offsets;
  double statistic = std::numeric_limits<double>::infinity();
  double information_logdet = -std::numeric_limits<double>::infinity();
  double smallest_singular_value = 0.0;
  double largest_singular_value = 0.0;
  double smallest_information_lower_bound = 0.0;
  double largest_information_upper_bound =
      std::numeric_limits<double>::infinity();
  double solve_relative_residual = std::numeric_limits<double>::infinity();
  double normal_equation_forward_error_bound =
      std::numeric_limits<double>::infinity();
  double llt_svd_relative_difference = std::numeric_limits<double>::infinity();
  bool canonical_spectral_solution = false;
  // B1 square-root context summary (values owned by the context; mirrored here
  // for diagnostics and for consumers that only hold the numerics struct).
  double square_root_statistic = std::numeric_limits<double>::infinity();
  double square_root_condition_estimate =
      std::numeric_limits<double>::infinity();
  int square_root_detector_only_rows = 0;
  bool square_root_certificate_ok = false;
  int exact_rank = 0;
  int dof = 0;
  double exact_condition = std::numeric_limits<double>::infinity();
  bool valid = false;
  std::string reason;
};

// Rebuilds and validates the identity from the values actually served by the
// frozen window.  This is a consumer gate, not merely a producer-side hash.
struct LinearizedIntegrityWindow;
std::uint64_t frozenWindowNumericalProofIdentity(
    const LinearizedIntegrityWindow& window,
    const FrozenWindowNumerics& numerics);
bool validateFrozenWindowNumericalProof(
    const LinearizedIntegrityWindow& window,
    const FrozenWindowNumerics& numerics,
    std::string* reason = nullptr);

// Kept in the window value so every consumer records the same nested wall-time
// intervals without introducing a process-global profiler or mutable cache.
struct WindowPreparationTiming {
  double boundary_and_provenance_ms = 0.0;
  double factor_linearization_whitening_ms = 0.0;
  double dense_assembly_ms = 0.0;
  double svd_ms = 0.0;
  double normal_equations_ms = 0.0;
  double llt_and_state_solves_ms = 0.0;
  double fingerprint_ms = 0.0;
};

struct LinearizedIntegrityWindow {
  WindowId id;
  LinearizationVersion version;
  std::vector<StateLayoutEntry> state_layout;
  std::vector<LinearizedFactorBlock> blocks;
  std::vector<FactorSlotAccounting> slot_accounting;
  std::vector<FrozenWindowFactorInventoryEntry> factor_inventory;
  std::size_t detector_first_epoch = 0;
  std::size_t recovery_first_epoch = 0;
  Eigen::MatrixXd H;
  Eigen::VectorXd z;
  int rank = 0;
  int dof = 0;
  double condition_number = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd base_information;
  Eigen::VectorXd base_information_rhs;
  Eigen::Matrix<double, 3, Eigen::Dynamic> protected_state_map;
  WindowCapabilities capabilities;
  WindowPreparationTiming preparation_timing;
  std::shared_ptr<const FrozenWindowNumerics> numerics;
  // B1: the single square-root context of this frozen window.  All consumers
  // (detector statistic, hypothesis evidence, PL slopes, candidate base solve)
  // read from it; the LLT/spectral products above are reference fallbacks.
  std::shared_ptr<const FrozenSquareRootContext> square_root;
  // C1-b: fault-preserving history summary of the condensed boundary.
  WindowHistorySummary history_summary;
  bool model_valid = false;
  std::string reason;
};

// Rebuilds the aggregate matrices and validates that every block uses the
// same frozen linearization version. This is shared by the estimator and
// synthetic/dense-oracle tests.
void finalizeIntegrityWindow(LinearizedIntegrityWindow* window,
                             double rank_tolerance,
                             double max_condition_number);

std::uint64_t integrityWindowFingerprint(
    const LinearizedIntegrityWindow& window);

std::uint64_t numericalContractFingerprint(double rank_tolerance,
                                           double max_condition_number);
std::uint64_t numericalContractFingerprint(
    const FrozenNumericalContract& contract);

Eigen::MatrixXd solveFrozenInformation(const FrozenWindowNumerics& numerics,
                                       const Eigen::MatrixXd& information,
                                       const Eigen::MatrixXd& rhs,
                                       bool* used_spectral_fallback = nullptr);

// B1 consumer entry point: solve (H^T H)^-1 rhs from the frozen window's single
// square-root context when it is usable and certifies itself, and only then
// fall back to the LLT/spectral reference path.  A fallback is always counted
// (NumericalWorkCounters::square_rootFallbacks) so mixed use is visible.
Eigen::MatrixXd solveFrozenInformation(const FrozenSquareRootContext* context,
                                       const FrozenWindowNumerics& numerics,
                                       const Eigen::MatrixXd& information,
                                       const Eigen::MatrixXd& rhs,
                                       bool* used_spectral_fallback = nullptr);

}  // namespace uwb_imu_pl
