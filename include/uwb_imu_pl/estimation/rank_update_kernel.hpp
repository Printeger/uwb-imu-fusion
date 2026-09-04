#pragma once

#include "uwb_imu_pl/integrity/fault_model.hpp"

namespace uwb_imu_pl {

struct RankUpdateConfig {
  double rank_tolerance = 1e-10;
  double max_condition_number = 1e10;
  double max_linearization_step_norm = 0.25;
};

struct BaseCandidateKernel {
  WindowId window_id;
  LinearizationVersion version;
  LinearizedIntegrityWindow window;
  Eigen::MatrixXd covariance;
  Eigen::VectorXd state_increment;
  bool valid = false;
  std::string reason;
};

struct CandidateEvaluation {
  ExclusionAction action;
  LinearizationVersion base_version;
  Eigen::VectorXd state_increment;
  Eigen::MatrixXd covariance;
  Eigen::MatrixXd retained_jacobian;
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
  std::string reason;
};

class RankUpdateEvaluator {
 public:
  explicit RankUpdateEvaluator(RankUpdateConfig config = {})
      : config_(config) {}
  BaseCandidateKernel factorizeOnce(
      const LinearizedIntegrityWindow& window) const;
  CandidateEvaluation evaluate(const BaseCandidateKernel& base,
                               const ExclusionAction& action) const;

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
