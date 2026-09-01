#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include "uwb_imu_pl/estimation/incremental_estimator.hpp"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>

#include <algorithm>
#include <Eigen/Cholesky>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

double chiSquareThreshold(int dof, double p_fa) {
  if (dof <= 0) throw std::invalid_argument("chi-square DOF must be positive");
  return boost::math::quantile(boost::math::chi_squared(dof), 1.0 - p_fa);
}

double nominalMultiplier(double two_sided_tail) {
  return boost::math::quantile(boost::math::normal_distribution<double>(),
                               1.0 - 0.5 * two_sided_tail);
}

Eigen::MatrixXd covarianceForConditional(const UwbBatch& batch) {
  const Eigen::Index n = static_cast<Eigen::Index>(batch.measurements.size());
  Eigen::MatrixXd covariance = batch.covariance_m2;
  if (covariance.size() == 0) {
    covariance = Eigen::MatrixXd::Zero(n, n);
    for (Eigen::Index i = 0; i < n; ++i) {
      const double sigma = batch.measurements[static_cast<std::size_t>(i)].sigma_m;
      if (!std::isfinite(sigma) || sigma <= 0.0) {
        throw std::invalid_argument("conditional UWB sigma must be finite and > 0");
      }
      covariance(i, i) = sigma * sigma;
    }
  }
  if (covariance.rows() != n || covariance.cols() != n ||
      !covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-12)) {
    throw std::invalid_argument("invalid conditional UWB covariance");
  }
  return covariance;
}

}  // namespace

double IntegrityMonitor::noncentralityBoundary(int dof, double threshold,
                                               double p_md) {
  if (dof <= 0 || threshold <= 0.0 || p_md <= 0.0 || p_md >= 1.0) {
    throw std::invalid_argument("invalid noncentral chi-square boundary input");
  }
  auto missed = [dof, threshold](double lambda) {
    return boost::math::cdf(
        boost::math::non_central_chi_squared_distribution<double>(dof, lambda),
        threshold);
  };
  if (missed(0.0) < p_md) return 0.0;
  double upper = 1.0;
  while (missed(upper) > p_md && upper < 1e8) upper *= 2.0;
  if (upper >= 1e8 && missed(upper) > p_md) {
    throw std::runtime_error("failed to bracket noncentrality boundary");
  }
  double lower = 0.0;
  for (int i = 0; i < 100; ++i) {
    const double middle = 0.5 * (lower + upper);
    if (missed(middle) > p_md) lower = middle;
    else upper = middle;
  }
  return upper;
}

DetectorResult IntegrityMonitor::snapshotDetector(
    const SnapshotSolution& solution) const {
  DetectorResult result;
  result.detector_type = "snapshot_postfit_chi_square";
  result.p_fa = risk_.p_fa;
  result.dof = solution.diagnostics.rows - solution.diagnostics.rank;
  if (!solution.converged || !solution.diagnostics.model_valid || result.dof <= 0 ||
      solution.residual_whitened.size() != solution.diagnostics.rows) {
    result.reason = solution.diagnostics.reason.empty() ?
        "invalid snapshot detector inputs" : solution.diagnostics.reason;
    return result;
  }
  result.threshold = chiSquareThreshold(result.dof, result.p_fa);
  result.statistic = solution.residual_whitened.squaredNorm();
  result.numerically_valid = std::isfinite(result.statistic) &&
      std::isfinite(result.threshold);
  result.passed = result.numerically_valid && result.statistic <= result.threshold;
  if (!result.passed) result.reason = result.numerically_valid ?
      "post-fit detector threshold exceeded" : "non-finite detector result";
  return result;
}

std::vector<FaultHypothesis> IntegrityMonitor::currentAnchorHypotheses(
    const UwbBatch& batch) const {
  const double p_md = risk_.hypotheses.empty() ? 1e-3 :
      risk_.hypotheses.front().missed_detection_allocation;
  const double prior = risk_.hypotheses.empty() ? 1e-4 :
      risk_.hypotheses.front().prior_probability_bound;
  std::map<std::uint64_t, std::vector<MeasurementId>> affected;
  for (const auto& measurement : batch.measurements) {
    affected[measurement.anchor_id.value()].push_back(measurement.id);
  }
  std::vector<FaultHypothesis> hypotheses;
  for (const auto& item : affected) {
    FaultHypothesis hypothesis;
    hypothesis.id = HypothesisId(item.first);
    hypothesis.anchor_id = AnchorId(item.first);
    hypothesis.affected_measurements = item.second;
    hypothesis.prior_probability_bound = prior;
    hypothesis.missed_detection_allocation = p_md;
    hypotheses.push_back(std::move(hypothesis));
  }
  return hypotheses;
}

SensitivityResult IntegrityMonitor::snapshotSensitivity(
    const UwbBatch& batch, const SnapshotSolution& solution,
    const FaultHypothesis& hypothesis, double threshold) const {
  SensitivityResult result;
  result.hypothesis_id = hypothesis.id;
  result.anchor_id = hypothesis.anchor_id;
  Eigen::VectorXd incidence = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(batch.measurements.size()));
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    if (batch.measurements[i].anchor_id == hypothesis.anchor_id) {
      incidence(static_cast<Eigen::Index>(i)) = 1.0;
    }
  }
  const Eigen::VectorXd a = solution.whitener * incidence;
  const Eigen::Vector3d state_map = solution.gain_whitened * a;
  result.detector_gram = (a.transpose() * solution.residual_projector * a)(0, 0);
  if (!std::isfinite(result.detector_gram) || result.detector_gram <= rank_tolerance_) {
    if (state_map.norm() <= rank_tolerance_) {
      result.slope_xyz.setZero();
      result.finite = true;
      result.reason = "fault has no protected-state effect";
      return result;
    }
    result.reason = "single-anchor fault is unmonitorable";
    return result;
  }
  result.slope_xyz = state_map.cwiseAbs() / std::sqrt(result.detector_gram);
  result.noncentrality_boundary = noncentralityBoundary(
      solution.diagnostics.rows - solution.diagnostics.rank, threshold,
      hypothesis.missed_detection_allocation);
  result.finite = result.slope_xyz.allFinite() &&
      std::isfinite(result.noncentrality_boundary);
  if (!result.finite) result.reason = "non-finite failure slope";
  return result;
}

ProtectionLevelResult IntegrityMonitor::protectionLevel(
    TimestampNs timestamp, const Eigen::Matrix3d& covariance,
    const DetectorResult& detector,
    const std::vector<SensitivityResult>& sensitivities,
    const LinearizationDiagnostics& diagnostics) const {
  ProtectionLevelResult result;
  result.timestamp = timestamp;
  result.label = IntegrityLabel::FormalLocalCurrentFaultOnly;
  result.consistency = LinearizationConsistency::Strict;
  if (!detector.numerically_valid || !diagnostics.model_valid ||
      !covariance.allFinite()) {
    result.reason = detector.reason.empty() ? diagnostics.reason : detector.reason;
    return result;
  }
  result.nominal_component_m = nominalMultiplier(risk_.nominal_axis_tail) *
      covariance.diagonal().cwiseMax(0.0).cwiseSqrt();
  result.fault_component_m.setZero();
  for (const auto& sensitivity : sensitivities) {
    if (!sensitivity.finite) {
      result.reason = sensitivity.reason;
      return result;
    }
    const Eigen::Vector3d component = sensitivity.slope_xyz *
        std::sqrt(sensitivity.noncentrality_boundary);
    for (int axis = 0; axis < 3; ++axis) {
      if (component(axis) > result.fault_component_m(axis)) {
        result.fault_component_m(axis) = component(axis);
        result.maximizing_anchor[static_cast<std::size_t>(axis)] =
            sensitivity.anchor_id;
      }
    }
  }
  result.pl_xyz_m = result.nominal_component_m + result.fault_component_m;
  result.hpl_box_m = std::max(result.pl_xyz_m.x(), result.pl_xyz_m.y());
  result.vpl_m = result.pl_xyz_m.z();
  if (!detector.passed) {
    result.availability = Availability::Alert;
    result.reason = detector.reason;
  } else if (result.hpl_box_m <= risk_.horizontal_alert_limit_m &&
             result.vpl_m <= risk_.vertical_alert_limit_m) {
    result.availability = Availability::Available;
  } else {
    result.availability = Availability::Unavailable;
    result.reason = "protection level exceeds alert limit";
  }
  return result;
}

IntegrityOutput IntegrityMonitor::evaluateSnapshot(
    const UwbBatch& batch, const SnapshotSolution& solution) const {
  IntegrityOutput output;
  output.timestamp = batch.timestamp;
  output.state.timestamp = batch.timestamp;
  output.state.position_world_m = solution.position_world_m;
  output.detector = snapshotDetector(solution);
  if (output.detector.numerically_valid) {
    for (const auto& hypothesis : currentAnchorHypotheses(batch)) {
      output.sensitivities.push_back(snapshotSensitivity(
          batch, solution, hypothesis, output.detector.threshold));
    }
  }
  output.protection_level = protectionLevel(
      batch.timestamp, solution.covariance_m2, output.detector,
      output.sensitivities, solution.diagnostics);
  output.batch_committed = output.detector.passed;
  return output;
}

SensitivityResult IntegrityMonitor::conditionalSensitivity(
    const UwbBatch& batch, const FaultHypothesis& hypothesis,
    const Eigen::MatrixXd& w, const Eigen::MatrixXd& innovation_cov,
    const Eigen::Matrix<double, 15, Eigen::Dynamic>& gain,
    const Eigen::Matrix<double, 3, 15>& protected_jacobian, int dof,
    double threshold) const {
  SensitivityResult result;
  result.hypothesis_id = hypothesis.id;
  result.anchor_id = hypothesis.anchor_id;
  Eigen::VectorXd incidence = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(batch.measurements.size()));
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    if (batch.measurements[i].anchor_id == hypothesis.anchor_id) {
      incidence(static_cast<Eigen::Index>(i)) = 1.0;
    }
  }
  const Eigen::VectorXd a = w * incidence;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(innovation_cov);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    result.reason = "conditional innovation covariance is not positive definite";
    return result;
  }
  result.detector_gram = a.dot(ldlt.solve(a));
  const Eigen::Vector3d protected_map = protected_jacobian * gain * a;
  if (!std::isfinite(result.detector_gram) || result.detector_gram <= rank_tolerance_) {
    if (protected_map.norm() <= rank_tolerance_) {
      result.slope_xyz.setZero();
      result.finite = true;
      result.reason = "fault has no protected-state effect";
      return result;
    }
    result.reason = "current-anchor fault is conditionally unmonitorable";
    return result;
  }
  result.slope_xyz = protected_map.cwiseAbs() / std::sqrt(result.detector_gram);
  result.noncentrality_boundary = noncentralityBoundary(
      dof, threshold, hypothesis.missed_detection_allocation);
  result.finite = result.slope_xyz.allFinite() &&
      std::isfinite(result.noncentrality_boundary);
  if (!result.finite) result.reason = "non-finite conditional failure slope";
  return result;
}

IntegrityOutput IntegrityMonitor::evaluateConditional(
    const UwbBatch& batch, const EstimationSnapshot& snapshot) const {
  IntegrityOutput output;
  output.timestamp = batch.timestamp;
  output.state = snapshot.state();
  output.detector.detector_type = "conditional_current_uwb_innovation_chi_square";
  output.detector.p_fa = risk_.p_fa;
  output.detector.dof = static_cast<int>(batch.measurements.size());
  output.protection_level.timestamp = batch.timestamp;
  output.protection_level.label = IntegrityLabel::FormalLocalCurrentFaultOnly;
  output.protection_level.consistency = snapshot.consistency();

  if (!snapshot.capabilities().pre_measurement_prior ||
      !snapshot.capabilities().consistent_relinearization ||
      snapshot.consistency() == LinearizationConsistency::CachedBlind) {
    output.detector.reason = "snapshot lacks formal conditional-integrity capabilities";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
  const auto prior_optional = snapshot.currentPrior();
  if (!prior_optional || !prior_optional->excludes_current_uwb) {
    output.detector.reason = "pre-measurement prior does not exclude current UWB";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
  if (!(prior_optional->version == snapshot.version())) {
    output.detector.reason = "prior and linearized rows have different versions";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
  if (snapshot.rowBlocks().size() != 1 ||
      snapshot.rowBlocks().front().role != RowRole::Measurement) {
    output.detector.reason = "conditional snapshot must contain one current measurement group";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
  const auto& rows = snapshot.rowBlocks().front();
  if (!(rows.version == snapshot.version()) || rows.jacobian.cols() != 15 ||
      rows.jacobian.rows() != output.detector.dof ||
      rows.residual.size() != output.detector.dof) {
    output.detector.reason = "conditional row/version dimension mismatch";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
  const auto& prior = *prior_optional;
  const Eigen::MatrixXd innovation_cov = rows.jacobian * prior.covariance *
      rows.jacobian.transpose() + Eigen::MatrixXd::Identity(
          output.detector.dof, output.detector.dof);
  Eigen::LDLT<Eigen::MatrixXd> innovation_ldlt(innovation_cov);
  if (innovation_ldlt.info() != Eigen::Success || !innovation_ldlt.isPositive()) {
    output.detector.reason = "conditional innovation covariance factorization failed";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
  output.detector.threshold = chiSquareThreshold(output.detector.dof, risk_.p_fa);
  output.detector.statistic = rows.residual.dot(
      innovation_ldlt.solve(rows.residual));
  output.conditional_innovation_statistic = output.detector.statistic;
  output.detector.numerically_valid = snapshot.diagnostics().model_valid &&
      std::isfinite(output.detector.statistic) &&
      std::isfinite(output.detector.threshold);
  output.detector.passed = output.detector.numerically_valid &&
      output.detector.statistic <= output.detector.threshold;
  for (int i = 0; i < output.detector.dof; ++i) {
    output.detector.local_anchor_scores.push_back(
        rows.residual(i) / std::sqrt(innovation_cov(i, i)));
  }

  const Eigen::Matrix<double, 15, Eigen::Dynamic> gain =
      prior.covariance * rows.jacobian.transpose() *
      innovation_ldlt.solve(Eigen::MatrixXd::Identity(
          output.detector.dof, output.detector.dof));
  const Eigen::Matrix<double, 15, 15> posterior_covariance =
      prior.covariance - gain * rows.jacobian * prior.covariance;
  const Eigen::Matrix<double, 15, 1> posterior_increment = gain * rows.residual;
  output.uwb_postfit_residual_statistic =
      (rows.residual - rows.jacobian * posterior_increment).squaredNorm();

  Eigen::Matrix<double, 3, 15> protected_jacobian =
      Eigen::Matrix<double, 3, 15>::Zero();
  protected_jacobian.block<3, 3>(0, 3) =
      prior.mean.q_world_body.normalized().toRotationMatrix();
  const Eigen::Matrix3d protected_covariance = protected_jacobian *
      posterior_covariance * protected_jacobian.transpose();
  const Eigen::MatrixXd covariance = covarianceForConditional(batch);
  Eigen::LLT<Eigen::MatrixXd> covariance_llt(covariance);
  if (covariance_llt.info() != Eigen::Success) {
    output.detector.numerically_valid = false;
    output.detector.passed = false;
    output.detector.reason = "current UWB covariance is not positive definite";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
  const Eigen::MatrixXd w = covariance_llt.matrixL().solve(
      Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
  for (const auto& hypothesis : currentAnchorHypotheses(batch)) {
    output.sensitivities.push_back(conditionalSensitivity(
        batch, hypothesis, w, innovation_cov, gain, protected_jacobian,
        output.detector.dof, output.detector.threshold));
  }
  output.protection_level = protectionLevel(
      batch.timestamp, protected_covariance, output.detector,
      output.sensitivities, snapshot.diagnostics());
  output.protection_level.label = IntegrityLabel::FormalLocalCurrentFaultOnly;
  output.protection_level.consistency = snapshot.consistency();
  if (!output.detector.passed) {
    output.detector.reason = output.detector.numerically_valid ?
        "conditional innovation threshold exceeded" :
        (snapshot.diagnostics().reason.empty() ? "conditional model invalid" :
         snapshot.diagnostics().reason);
    output.protection_level.pl_xyz_m.setConstant(
        std::numeric_limits<double>::infinity());
    output.protection_level.nominal_component_m = output.protection_level.pl_xyz_m;
    output.protection_level.fault_component_m = output.protection_level.pl_xyz_m;
    output.protection_level.hpl_box_m = std::numeric_limits<double>::infinity();
    output.protection_level.vpl_m = std::numeric_limits<double>::infinity();
    output.protection_level.availability = output.detector.numerically_valid ?
        Availability::Alert : Availability::Unavailable;
    output.protection_level.reason = output.detector.reason;
  }
  output.batch_committed = false;
  return output;
}

RealtimeIntegrityPipeline::RealtimeIntegrityPipeline(
    IncrementalUwbImuEstimator* estimator, IntegrityMonitor monitor)
    : estimator_(estimator), monitor_(std::move(monitor)) {
  if (estimator_ == nullptr) throw std::invalid_argument("estimator must not be null");
}

void RealtimeIntegrityPipeline::ingestImu(const ImuMeasurement& measurement) {
  estimator_->ingestImu(measurement);
}

IntegrityOutput RealtimeIntegrityPipeline::processUwbBatch(const UwbBatch& batch) {
  estimator_->predictTo(batch.timestamp);
  const auto snapshot = estimator_->preMeasurementSnapshot(batch);
  IntegrityOutput output = monitor_.evaluateConditional(batch, *snapshot);
  if (output.detector.passed) {
    estimator_->commitUwbBatch(batch);
    output.state = estimator_->currentState();
    output.batch_committed = true;
  } else {
    estimator_->rejectUwbBatch(batch, output.detector.reason);
    output.state = estimator_->currentState();
    output.batch_committed = false;
  }
  output.global_graph_residual_statistic =
      estimator_->globalGraphResidualStatistic();
  return output;
}

}  // namespace uwb_imu_pl
