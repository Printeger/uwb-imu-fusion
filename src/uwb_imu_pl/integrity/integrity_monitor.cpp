#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>

#include <algorithm>
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

}  // namespace uwb_imu_pl
