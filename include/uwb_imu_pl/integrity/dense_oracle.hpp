#pragma once

#include <Eigen/Core>

#include <limits>
#include <string>

namespace uwb_imu_pl {

// Independent high-precision linear oracle used only by validation tools and
// tests. It deliberately shares no decompositions or intermediate matrices
// with SnapshotUwbEstimator/IntegrityMonitor.
struct DenseOracleResult {
  bool model_valid = false;
  bool fault_monitorable = false;
  int rank = 0;
  double condition_number = std::numeric_limits<double>::infinity();
  double detector_statistic = std::numeric_limits<double>::infinity();
  double detector_gram = 0.0;
  Eigen::MatrixXd pseudoinverse;
  Eigen::MatrixXd residual_projector;
  Eigen::MatrixXd covariance;
  Eigen::VectorXd state_delta;
  Eigen::VectorXd protected_slope = Eigen::VectorXd::Constant(
      1, std::numeric_limits<double>::infinity());
  std::string reason;
};

DenseOracleResult evaluateDenseSvdOracle(
    const Eigen::MatrixXd& design_raw, const Eigen::VectorXd& residual_raw,
    const Eigen::MatrixXd& measurement_covariance,
    const Eigen::VectorXd& fault_incidence,
    const Eigen::MatrixXd& protected_state_jacobian,
    double rank_tolerance = 1e-12, double max_condition_number = 1e12);

}  // namespace uwb_imu_pl
