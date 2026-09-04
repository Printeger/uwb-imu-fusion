#pragma once

#include "uwb_imu_pl/estimation/factor_ledger.hpp"

#include <Eigen/Core>

#include <limits>
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
};

struct IntegrityWindowRequest {
  std::uint32_t epochs = 20;
  bool include_pending_imu = true;
  bool include_pending_uwb = true;
  bool include_generic_bridge = false;
};

struct LinearizedIntegrityWindow {
  WindowId id;
  LinearizationVersion version;
  std::vector<StateLayoutEntry> state_layout;
  std::vector<LinearizedFactorBlock> blocks;
  Eigen::MatrixXd H;
  Eigen::VectorXd z;
  int rank = 0;
  int dof = 0;
  double condition_number = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd base_information;
  Eigen::VectorXd base_information_rhs;
  Eigen::Matrix<double, 3, Eigen::Dynamic> protected_state_map;
  WindowCapabilities capabilities;
  bool model_valid = false;
  std::string reason;
};

// Rebuilds the aggregate matrices and validates that every block uses the
// same frozen linearization version. This is shared by the estimator and
// synthetic/dense-oracle tests.
void finalizeIntegrityWindow(LinearizedIntegrityWindow* window,
                             double rank_tolerance,
                             double max_condition_number);

}  // namespace uwb_imu_pl
