#pragma once

#include "uwb_imu_pl/integrity/fault_model.hpp"

#include <Eigen/Cholesky>

#include <memory>
#include <cstdint>

namespace uwb_imu_pl {

struct RankUpdateConfig {
  double rank_tolerance = 1e-10;
  double max_condition_number = 1e10;
  double max_linearization_step_norm = 0.25;
  bool materialize_dense_oracle_fields = true;
  double exact_slow_path_condition = 1e8;
  bool enable_shared_cache = true;
  bool enable_early_step_gate = true;
  bool enable_numerical_certificate = true;
  bool force_exact_condition_number = false;
};

struct RankUpdateScratch {
  Eigen::MatrixXd update_columns;
  Eigen::VectorXd update_signs;
  Eigen::MatrixXd whitened_update;
  Eigen::MatrixXd small_symmetric;
};

struct BlockSolveCacheEntry {
  LinearizedFactorBlock block;
  Eigen::MatrixXd base_solve;
};
struct FrozenBlockSolveCache {
  WindowId window_id;
  LinearizationVersion version;
  std::map<FactorGroupId, std::vector<BlockSolveCacheEntry>> blocks;
};

struct BaseCandidateKernel {
  WindowId window_id;
  LinearizationVersion version;
  std::shared_ptr<const FrozenBlockSolveCache> block_cache;
  std::shared_ptr<const FrozenWindowNumerics> numerics;
  const LinearizedIntegrityWindow* window_view = nullptr;
  std::shared_ptr<const Eigen::LLT<Eigen::MatrixXd>> information_factorization;
  Eigen::VectorXd state_increment;
  double information_logdet = -std::numeric_limits<double>::infinity();
  double smallest_singular_value = 0.0;
  int exact_rank = 0;
  double exact_condition = std::numeric_limits<double>::infinity();
  bool valid = false;
  std::string reason;
};

struct CandidateEvaluation {
  CandidateDiagnostics diagnostics;
  ExclusionAction action;
  LinearizationVersion base_version;
  Eigen::VectorXd state_increment;
  Eigen::MatrixXd covariance;
  std::shared_ptr<const Eigen::LLT<Eigen::MatrixXd>> shared_base_factorization;
  // SVD reference covariance operator V diag(1/s^2) V^T; never a full inverse.
  std::shared_ptr<const Eigen::MatrixXd> reference_vectors;
  Eigen::VectorXd reference_inverse_squared;
  Eigen::MatrixXd covariance_plus_factor;
  Eigen::MatrixXd covariance_minus_factor;
  const LinearizedIntegrityWindow* window_view = nullptr;
  Eigen::MatrixXd retained_jacobian;
  const Eigen::MatrixXd* retained_jacobian_view = nullptr;
  Eigen::VectorXd retained_residual;
  int rows = 0;
  int rank = 0;
  int dof = 0;
  double statistic = std::numeric_limits<double>::infinity();
  double squared_threshold = std::numeric_limits<double>::infinity();
  double condition_number = std::numeric_limits<double>::infinity();
  double information_logdet = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d pl_xyz_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double hpl_m = std::numeric_limits<double>::infinity();
  double vpl_m = std::numeric_limits<double>::infinity();
  bool post_detector_passed = false;
  bool covers_plausible_set = false;
  bool valid = false;
  bool selected = false;
  bool exact_slow_path = false;
  double wall_ms = 0.0;
  std::string reason;

  Eigen::MatrixXd covarianceTimes(const Eigen::MatrixXd& value) const;
  Eigen::MatrixXd normalCross(const Eigen::MatrixXd& row_map) const;
  const Eigen::MatrixXd& retainedJacobian() const {
    return retained_jacobian_view ? *retained_jacobian_view : retained_jacobian;
  }
};

class RankUpdateEvaluator {
 public:
  explicit RankUpdateEvaluator(RankUpdateConfig config = {})
      : config_(config) {}
  BaseCandidateKernel factorizeOnce(
      const LinearizedIntegrityWindow& window,
      const std::vector<ExclusionAction>& actions = {}) const;
  void buildSharedCache(BaseCandidateKernel* base,
                        const std::vector<ExclusionAction>& actions) const;
  CandidateEvaluation evaluate(const BaseCandidateKernel& base,
                               const ExclusionAction& action) const;
  CandidateEvaluation evaluate(const BaseCandidateKernel& base,
                               const ExclusionAction& action,
                               RankUpdateScratch* scratch) const;

 private:
  RankUpdateConfig config_;
};

class DenseCandidateOracle {
 public:
  explicit DenseCandidateOracle(RankUpdateConfig config = {})
      : config_(config) {}
  CandidateEvaluation evaluate(const LinearizedIntegrityWindow& window,
                               const ExclusionAction& action) const;

 private:
  RankUpdateConfig config_;
};

}  // namespace uwb_imu_pl
