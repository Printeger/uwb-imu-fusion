#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace uifgo {

// T05 Gate-A numerical core.  Inputs are already-whitened Jacobian blocks.
// This interface intentionally has no dependency on a runner or a GTSAM graph.
enum class RecoverabilityStatus {
  OK,
  RANK_DEFICIENT,
  SPARSE_RANK_UNCERTAIN,
  NUMERICAL_RESOLUTION_LOST,
  NUMERICAL_FAILURE,
  INVALID_INPUT,
};

const char* RecoverabilityStatusName(RecoverabilityStatus status);

struct RecoverabilityOptions {
  double rank_absolute_tolerance = 1e-12;
  double rank_relative_tolerance = 1e-10;
  double symmetry_absolute_tolerance = 1e-12;
  double symmetry_relative_tolerance = 1e-10;
  double psd_absolute_tolerance_m2_inv = 1e-12;
  double psd_relative_tolerance = 1e-10;
  double pd_absolute_tolerance_m2_inv = 1e-12;
  double pd_relative_tolerance = 1e-10;
  double eta_absolute_tolerance = 1e-12;
  double orthogonality_absolute_tolerance = 1e-12;
  double orthogonality_relative_tolerance = 1e-10;
  double roundoff_safety_factor = 8.0;
  size_t condition_estimator_max_iterations = 8;
};

struct NuisanceColumnAudit {
  size_t original_index = 0;
  int active_index = -1;
  int representative_original_index = -1;
  double original_norm = 0.0;
  double scale = 0.0;
  bool exact_zero = false;
  bool exact_duplicate = false;
};

struct RecoverabilityResult {
  RecoverabilityStatus status = RecoverabilityStatus::INVALID_INPUT;
  std::string reason;

  Eigen::SparseMatrix<double> F_scaled;
  Eigen::MatrixXd E;
  Eigen::MatrixXd N;
  Eigen::MatrixXd R;
  std::vector<NuisanceColumnAudit> nuisance_columns;
  std::vector<int> active_qr_permutation;
  std::vector<double> qr_pivots_desc;

  size_t rows = 0;
  size_t nuisance_columns_total = 0;
  size_t nuisance_columns_active = 0;
  size_t amplitude_columns = 0;
  size_t exact_zero_columns = 0;
  size_t exact_duplicate_columns = 0;
  int naive_qr_pivot_rank = 0;
  int sparse_qr_rank = 0;
  int frozen_F_rank = -1;
  int N_rank = 0;
  int R_rank = 0;

  double qr_condition_estimate_1 = 1.0;
  double qr_rcond_estimate_1 = 1.0;
  size_t condition_estimator_iterations = 0;
  bool condition_estimator_converged = true;
  double retained_pivot_min = 0.0;
  double retained_pivot_max = 0.0;
  double orthogonality_residual = 0.0;
  double rank_threshold_lower = 0.0;
  double rank_threshold_upper = 0.0;
  double rank_certificate_sigma_min_lower = 0.0;
  double rank_certificate_sigma_max_upper = 0.0;
  double rank_certificate_inverse_residual = 0.0;
  bool frozen_rank_certified = false;
  double roundoff_multiplier = 0.0;
  double projection_residual_floor = 0.0;
  double n_minus_r_min_eigenvalue = 0.0;
  double n_minus_r_psd_tolerance = 0.0;
  double N_rank_pd_threshold = 0.0;
  double R_rank_pd_threshold = 0.0;
  double R_psd_threshold = 0.0;
  double orthogonality_tolerance = 0.0;
  double eta = 0.0;
  double s_m = 0.0;
  bool s_is_infinite = true;

  bool valid_score() const {
    return status == RecoverabilityStatus::OK ||
           status == RecoverabilityStatus::RANK_DEFICIENT;
  }
};

RecoverabilityResult ComputeSparseRecoverability(
    const Eigen::SparseMatrix<double>& F_whitened,
    const Eigen::MatrixXd& G_whitened,
    const RecoverabilityOptions& options = RecoverabilityOptions());

}  // namespace uifgo
