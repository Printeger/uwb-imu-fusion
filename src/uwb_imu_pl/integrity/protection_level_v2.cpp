#include "uwb_imu_pl/integrity/protection_level_v2.hpp"

#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {

double ProtectionLevelV2::detectionBoundaryNoncentralitySquared(
    int dof, double threshold, double p_md) {
  if (dof <= 0 || !std::isfinite(threshold) || threshold <= 0.0 ||
      !std::isfinite(p_md) || p_md <= 0.0 || p_md >= 1.0) {
    throw std::invalid_argument("invalid noncentrality boundary inputs");
  }
  auto missed = [&](double lambda_squared) {
    return boost::math::cdf(
        boost::math::non_central_chi_squared(dof, lambda_squared), threshold);
  };
  double lower = 0.0;
  double upper = 1.0;
  while (missed(upper) > p_md && upper < 1e12) upper *= 2.0;
  if (upper >= 1e12 && missed(upper) > p_md) {
    throw std::runtime_error("cannot bracket noncentrality boundary");
  }
  for (int i = 0; i < 120; ++i) {
    const double middle = 0.5 * (lower + upper);
    if (missed(middle) > p_md) lower = middle;
    else upper = middle;
  }
  return 0.5 * (lower + upper);
}

ProtectionLevelV2Result ProtectionLevelV2::compute(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate,
    const DetectorResultV2& detector,
    std::vector<FaultHypothesisV2>* hypotheses,
    const RiskBudgetV2& risk,
    const Eigen::Vector3d& bridge_margin) const {
  ProtectionLevelV2Result result;
  result.bridge_component_m = bridge_margin.cwiseAbs();
  if (!hypotheses || !candidate.valid || !detector.numerically_valid ||
      window.protected_state_map.cols() != candidate.covariance.rows() ||
      window.protected_state_map.rows() != 3) {
    result.reason = "invalid post-FDE candidate/protected-state map";
    return result;
  }
  const Eigen::Matrix3d protected_covariance =
      window.protected_state_map * candidate.covariance *
      window.protected_state_map.transpose();
  if (!protected_covariance.allFinite() ||
      (protected_covariance.diagonal().array() <= 0.0).any()) {
    result.reason = "post-FDE protected covariance is invalid";
    return result;
  }
  double allocated = 3.0 * risk.nominal_axis_tail + risk.p_nm +
      risk.p_bridge_escape + risk.p_history_contamination + risk.p_model_escape;
  for (const auto& hypothesis : *hypotheses) {
    allocated += hypothesis.hmi_allocation;
  }
  result.allocated_outcome_risk = allocated;
  result.risk_budget_valid = allocated <= risk.p_hmi_total;
  if (!result.risk_budget_valid) {
    result.reason = "outcome-conditioned risk budget does not close";
    return result;
  }
  const double nominal_tail = std::max(1e-15, risk.nominal_axis_tail);
  const double nominal_k = boost::math::quantile(
      boost::math::normal(), 1.0 - 0.5 * nominal_tail);
  result.nominal_component_m = nominal_k *
      protected_covariance.diagonal().cwiseSqrt();
  result.fault_component_m.setZero();
  const Eigen::MatrixXd& h = candidate.retained_jacobian;
  const Eigen::MatrixXd gain = candidate.covariance * h.transpose();
  for (auto& hypothesis : *hypotheses) {
    if (hypothesis.A.rows() != h.rows() || hypothesis.A.cols() == 0) {
      hypothesis.monitored = false;
      hypothesis.monitorability.reason = "post-FDE fault map row mismatch";
      result.reason = "remaining hypothesis cannot be mapped post-FDE";
      return result;
    }
    const Eigen::MatrixXd projected = hypothesis.A - h * gain * hypothesis.A;
    const Eigen::MatrixXd gram = projected.transpose() * projected;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(gram);
    const auto singular = svd.singularValues();
    const double largest = singular.size() ? singular(0) : 0.0;
    const double gate = 1e-10 * std::max(1.0, largest);
    hypothesis.monitorability.parameter_dimension = hypothesis.A.cols();
    hypothesis.monitorability.rank =
        static_cast<int>((singular.array() > gate).count());
    const double smallest = hypothesis.monitorability.rank > 0
        ? singular(hypothesis.monitorability.rank - 1) : 0.0;
    hypothesis.monitorability.sigma_min = std::sqrt(std::max(0.0, smallest));
    hypothesis.monitorability.sigma_max = std::sqrt(std::max(0.0, largest));
    hypothesis.monitorability.condition_number = smallest > 0.0
        ? largest / smallest : std::numeric_limits<double>::infinity();
    hypothesis.monitorability.monitorable =
        hypothesis.monitorability.rank == hypothesis.A.cols();
    hypothesis.monitored = hypothesis.monitorability.monitorable;
    if (!hypothesis.monitored) {
      result.reason = "remaining post-FDE fault is unmonitorable";
      return result;
    }
    Eigen::LDLT<Eigen::MatrixXd> gram_solve(gram);
    if (gram_solve.info() != Eigen::Success || !gram_solve.isPositive()) {
      result.reason = "remaining fault Gram matrix is not SPD";
      return result;
    }
    const Eigen::MatrixXd protected_fault =
        window.protected_state_map * gain * hypothesis.A;
    Eigen::Vector3d slopes;
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::VectorXd response = protected_fault.row(axis).transpose();
      slopes(axis) = std::sqrt(std::max(
          0.0, response.dot(gram_solve.solve(response))));
    }
    hypothesis.monitorability.protected_slopes = slopes;
    const double lambda = std::sqrt(detectionBoundaryNoncentralitySquared(
        detector.dof, detector.squared_threshold, hypothesis.p_md_allocation));
    const double fault_tail = std::max(1e-15, hypothesis.hmi_allocation > 0.0
        ? hypothesis.hmi_allocation / std::max(hypothesis.prior_probability_bound, 1e-15)
        : nominal_tail);
    const double k_fault = boost::math::quantile(
        boost::math::normal(), 1.0 - 0.5 * std::min(0.5, fault_tail));
    const Eigen::Vector3d component = slopes * lambda + k_fault *
        protected_covariance.diagonal().cwiseSqrt();
    result.fault_component_m = result.fault_component_m.cwiseMax(component);
  }
  result.pl_xyz_m = result.nominal_component_m
      .cwiseMax(result.fault_component_m) + result.bridge_component_m;
  result.hpl_m = std::hypot(result.pl_xyz_m.x(), result.pl_xyz_m.y());
  result.vpl_m = result.pl_xyz_m.z();
  result.model_valid = result.pl_xyz_m.allFinite() && detector.passed;
  result.availability = result.model_valid &&
      result.hpl_m <= risk.horizontal_alert_limit_m &&
      result.vpl_m <= risk.vertical_alert_limit_m
      ? Availability::Available : Availability::Unavailable;
  result.formal_eligible = false;  // Gate J evidence is intentionally absent.
  if (!detector.passed) result.reason = "post-FDE detector alarm";
  else if (result.availability != Availability::Available) {
    result.reason = "post-FDE protection level exceeds alert limit";
  } else {
    result.reason = "research V2 implemented; Gate J calibration/review pending";
  }
  return result;
}

}  // namespace uwb_imu_pl
