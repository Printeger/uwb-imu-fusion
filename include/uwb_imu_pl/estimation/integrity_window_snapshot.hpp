#pragma once

#include "uwb_imu_pl/estimation/factor_ledger.hpp"

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <limits>
#include <memory>
#include <string>
#include <vector>

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
  std::uint64_t policy_version = 2;
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
  Eigen::VectorXd base_state_increment;
  Eigen::VectorXd parity;
  std::vector<Eigen::Index> block_row_offsets;
  double statistic = std::numeric_limits<double>::infinity();
  double information_logdet = -std::numeric_limits<double>::infinity();
  double smallest_singular_value = 0.0;
  double largest_singular_value = 0.0;
  double smallest_information_lower_bound = 0.0;
  double largest_information_upper_bound = std::numeric_limits<double>::infinity();
  double solve_relative_residual = std::numeric_limits<double>::infinity();
  double normal_equation_forward_error_bound =
      std::numeric_limits<double>::infinity();
  double llt_svd_relative_difference = std::numeric_limits<double>::infinity();
  bool canonical_spectral_solution = false;
  int exact_rank = 0;
  int dof = 0;
  double exact_condition = std::numeric_limits<double>::infinity();
  bool valid = false;
  std::string reason;
};

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

}  // namespace uwb_imu_pl
