#include "uwb_imu_pl/integrity/dense_oracle.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {

DenseOracleResult evaluateDenseSvdOracle(
    const Eigen::MatrixXd& design_raw, const Eigen::VectorXd& residual_raw,
    const Eigen::MatrixXd& measurement_covariance,
    const Eigen::VectorXd& fault_incidence,
    const Eigen::MatrixXd& protected_state_jacobian, double rank_tolerance,
    double max_condition_number) {
  DenseOracleResult result;
  const Eigen::Index rows = design_raw.rows();
  const Eigen::Index columns = design_raw.cols();
  result.protected_slope = Eigen::VectorXd::Constant(
      protected_state_jacobian.rows(), std::numeric_limits<double>::infinity());
  if (rows <= 0 || columns <= 0 || residual_raw.size() != rows ||
      measurement_covariance.rows() != rows ||
      measurement_covariance.cols() != rows || fault_incidence.size() != rows ||
      protected_state_jacobian.cols() != columns ||
      !design_raw.allFinite() || !residual_raw.allFinite() ||
      !measurement_covariance.allFinite() || !fault_incidence.allFinite() ||
      !protected_state_jacobian.allFinite() || rank_tolerance <= 0.0 ||
      max_condition_number <= 0.0) {
    result.reason = "invalid dense-oracle dimensions or values";
    return result;
  }
  if (!measurement_covariance.isApprox(
          measurement_covariance.transpose(), 1e-13)) {
    result.reason = "measurement covariance is not symmetric";
    return result;
  }
  Eigen::LLT<Eigen::MatrixXd> covariance_llt(measurement_covariance);
  if (covariance_llt.info() != Eigen::Success) {
    result.reason = "measurement covariance is not SPD";
    return result;
  }
  const Eigen::MatrixXd whitener = covariance_llt.matrixL().solve(
      Eigen::MatrixXd::Identity(rows, rows));
  const Eigen::MatrixXd design = whitener * design_raw;
  const Eigen::VectorXd residual = whitener * residual_raw;
  const Eigen::VectorXd incidence = whitener * fault_incidence;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      design, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::VectorXd singular = svd.singularValues();
  const double scale = singular.size() == 0 ? 0.0 : singular.maxCoeff();
  const double cutoff = rank_tolerance * std::max(1.0, scale);
  result.rank = static_cast<int>((singular.array() > cutoff).count());
  if (result.rank != columns) {
    result.reason = "design is rank deficient";
    return result;
  }
  const double minimum = singular.minCoeff();
  result.condition_number = scale / minimum;
  if (!std::isfinite(result.condition_number) ||
      result.condition_number > max_condition_number) {
    result.reason = "design exceeds condition-number gate";
    return result;
  }

  Eigen::MatrixXd sigma_inverse = Eigen::MatrixXd::Zero(columns, rows);
  for (Eigen::Index index = 0; index < singular.size(); ++index) {
    sigma_inverse(index, index) = 1.0 / singular(index);
  }
  result.pseudoinverse =
      svd.matrixV() * sigma_inverse * svd.matrixU().transpose();
  result.residual_projector = Eigen::MatrixXd::Identity(rows, rows) -
      design * result.pseudoinverse;
  result.residual_projector = 0.5 *
      (result.residual_projector + result.residual_projector.transpose());
  result.covariance = result.pseudoinverse * result.pseudoinverse.transpose();
  result.state_delta = result.pseudoinverse * residual;
  const Eigen::VectorXd postfit = result.residual_projector * residual;
  result.detector_statistic = postfit.squaredNorm();
  const Eigen::VectorXd protected_fault =
      protected_state_jacobian * result.pseudoinverse * incidence;
  result.detector_gram = incidence.dot(result.residual_projector * incidence);
  if (result.detector_gram <= cutoff) {
    if (protected_fault.norm() <= cutoff) {
      result.protected_slope.setZero();
      result.fault_monitorable = true;
      result.reason = "fault has no protected-state effect";
    } else {
      // Keep the initialized infinities: an unmonitorable fault must never be
      // hidden by a pseudoinverse or a large finite sentinel.
      result.reason = "fault is unmonitorable";
    }
  } else {
    result.protected_slope = protected_fault.cwiseAbs() /
        std::sqrt(result.detector_gram);
    result.fault_monitorable = result.protected_slope.allFinite();
    if (!result.fault_monitorable) result.reason = "non-finite fault slope";
  }
  result.model_valid = true;
  return result;
}

}  // namespace uwb_imu_pl
