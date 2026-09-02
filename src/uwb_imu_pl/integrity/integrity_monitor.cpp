#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include "uwb_imu_pl/estimation/incremental_estimator.hpp"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>

#include <algorithm>
#include <Eigen/Cholesky>
#include <chrono>
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
  std::map<std::uint64_t, std::vector<MeasurementId>> affected;
  for (const auto& measurement : batch.measurements) {
    affected[measurement.anchor_id.value()].push_back(measurement.id);
  }
  std::vector<FaultHypothesis> hypotheses;
  for (const auto& item : affected) {
    const auto allocation = std::find_if(
        risk_.hypotheses.begin(), risk_.hypotheses.end(),
        [&item](const FaultHypothesis& candidate) {
          return candidate.anchor_id.value() == item.first;
        });
    FaultHypothesis hypothesis = allocation == risk_.hypotheses.end()
        ? FaultHypothesis{} : *allocation;
    if (allocation == risk_.hypotheses.end()) {
      hypothesis.id = HypothesisId(item.first);
      hypothesis.anchor_id = AnchorId(item.first);
      hypothesis.monitored = false;
      hypothesis.pruning_reason = "missing configured physical-anchor hypothesis";
    }
    hypothesis.affected_measurements = item.second;
    hypotheses.push_back(std::move(hypothesis));
  }
  return hypotheses;
}

bool IntegrityMonitor::completePhysicalFaultMap(
    const UwbBatch& batch, const std::vector<FaultHypothesis>& hypotheses,
    std::string* reason) const {
  std::set<std::uint64_t> physical;
  for (const auto& measurement : batch.measurements) {
    physical.insert(measurement.anchor_id.value());
  }
  std::set<std::uint64_t> mapped;
  for (const auto& hypothesis : hypotheses) {
    if (!hypothesis.monitored || hypothesis.anchor_id.value() == 0 ||
        !mapped.insert(hypothesis.anchor_id.value()).second ||
        physical.count(hypothesis.anchor_id.value()) == 0 ||
        hypothesis.affected_measurements.empty()) {
      if (reason) *reason = "incomplete or duplicate physical fault mapping";
      return false;
    }
  }
  if (mapped != physical) {
    if (reason) *reason = "physical anchor set is not fully mapped to hypotheses";
    return false;
  }
  return true;
}

bool IntegrityMonitor::riskBudgetValid(
    const std::vector<FaultHypothesis>& hypotheses, double* allocated) const {
  if (!std::isfinite(risk_.nominal_axis_tail) ||
      risk_.nominal_axis_tail <= 0.0 || risk_.nominal_axis_tail >= 1.0 ||
      !std::isfinite(risk_.p_nm) || risk_.p_nm <= 0.0 || risk_.p_nm >= 1.0) {
    if (allocated) *allocated = std::numeric_limits<double>::infinity();
    return false;
  }
  double value = 3.0 * risk_.nominal_axis_tail + risk_.p_nm;
  for (const auto& hypothesis : hypotheses) {
    if (!std::isfinite(hypothesis.prior_probability_bound) ||
        !std::isfinite(hypothesis.missed_detection_allocation) ||
        hypothesis.prior_probability_bound <= 0.0 ||
        hypothesis.prior_probability_bound >= 1.0 ||
        hypothesis.missed_detection_allocation <= 0.0 ||
        hypothesis.missed_detection_allocation >= 1.0) {
      if (allocated) *allocated = std::numeric_limits<double>::infinity();
      return false;
    }
    value += hypothesis.prior_probability_bound *
        hypothesis.missed_detection_allocation;
  }
  if (allocated) *allocated = value;
  return std::isfinite(value) && std::isfinite(risk_.p_hmi_total) &&
      risk_.p_hmi_total > 0.0 && value <= risk_.p_hmi_total;
}

bool IntegrityMonitor::snapshotFormalGate(
    const UwbBatch& batch, const SnapshotSolution& solution,
    const std::vector<FaultHypothesis>& hypotheses, std::string* reason) const {
  const Eigen::Index n = static_cast<Eigen::Index>(batch.measurements.size());
  if (!completePhysicalFaultMap(batch, hypotheses, reason)) return false;
  if (!solution.converged || !solution.diagnostics.model_valid ||
      !std::isfinite(solution.diagnostics.linearization_step_norm) ||
      solution.diagnostics.rank != 3 || solution.diagnostics.rows != n ||
      !std::isfinite(solution.diagnostics.condition_number) ||
      solution.diagnostics.condition_number > max_condition_number_) {
    if (reason) *reason = solution.diagnostics.reason.empty()
        ? "invalid snapshot rank, conditioning, or linearization"
        : solution.diagnostics.reason;
    return false;
  }
  if (solution.covariance_measurement_m2.rows() != n ||
      solution.whitener.rows() != n || solution.whitener.cols() != n ||
      solution.jacobian_whitened.rows() != n ||
      solution.jacobian_whitened.cols() != 3 ||
      solution.residual_whitened.size() != n ||
      !solution.covariance_m2.allFinite()) {
    if (reason) *reason = "snapshot covariance/whitening dimensions are invalid";
    return false;
  }
  Eigen::LLT<Eigen::Matrix3d> covariance_llt(solution.covariance_m2);
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(n, n);
  const Eigen::MatrixXd whitened_covariance = solution.whitener *
      solution.covariance_measurement_m2 * solution.whitener.transpose();
  const bool whitened_rows_valid =
      solution.jacobian_raw.rows() == n && solution.jacobian_raw.cols() == 3 &&
      solution.residual_raw_m.size() == n &&
      solution.jacobian_whitened.isApprox(
          solution.whitener * solution.jacobian_raw, 1e-10) &&
      solution.residual_whitened.isApprox(
          solution.whitener * solution.residual_raw_m, 1e-10);
  const bool projector_valid = solution.residual_projector.rows() == n &&
      solution.residual_projector.cols() == n &&
      solution.residual_projector.isApprox(
          solution.residual_projector.transpose(), 1e-10) &&
      (solution.residual_projector * solution.residual_projector).isApprox(
          solution.residual_projector, 1e-10);
  if (covariance_llt.info() != Eigen::Success ||
      !whitened_covariance.isApprox(identity, 1e-10) ||
      !whitened_rows_valid || !projector_valid) {
    if (reason) *reason = "snapshot whitening, covariance, or projector audit failed";
    return false;
  }
  return true;
}

SensitivityResult IntegrityMonitor::snapshotSensitivity(
    const UwbBatch& batch, const SnapshotSolution& solution,
    const FaultHypothesis& hypothesis, double threshold) const {
  SensitivityResult result;
  result.hypothesis_id = hypothesis.id;
  result.anchor_id = hypothesis.anchor_id;
  if (!hypothesis.monitored || hypothesis.missed_detection_allocation <= 0.0 ||
      hypothesis.missed_detection_allocation >= 1.0) {
    result.reason = hypothesis.pruning_reason.empty()
        ? "physical fault hypothesis is not monitored"
        : hypothesis.pruning_reason;
    return result;
  }
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
    const std::vector<FaultHypothesis>& hypotheses,
    const LinearizationDiagnostics& diagnostics,
    LinearizationConsistency consistency, bool model_formal_eligible,
    const std::string& gate_reason) const {
  ProtectionLevelResult result;
  result.timestamp = timestamp;
  result.unmonitored_risk = risk_.p_nm;
  result.hmi_risk_requirement = risk_.p_hmi_total;
  result.risk_budget_valid = riskBudgetValid(
      hypotheses, &result.allocated_hmi_risk);
  result.consistency = consistency;
  result.formal_eligible = model_formal_eligible && result.risk_budget_valid;
  if (!result.formal_eligible) {
    result.reason = model_formal_eligible
        ? "allocated HMI risk exceeds total requirement" : gate_reason;
    return result;
  }
  result.label = IntegrityLabel::FormalLocalCurrentFaultOnly;
  if (!detector.numerically_valid || !diagnostics.model_valid ||
      !covariance.allFinite()) {
    result.reason = detector.reason.empty() ? diagnostics.reason : detector.reason;
    result.formal_eligible = false;
    result.label = IntegrityLabel::ImplementedUnverified;
    return result;
  }
  Eigen::LLT<Eigen::Matrix3d> protected_covariance_llt(
      0.5 * (covariance + covariance.transpose()));
  if (protected_covariance_llt.info() != Eigen::Success) {
    result.reason = "protected-state covariance is not finite SPD";
    result.formal_eligible = false;
    result.label = IntegrityLabel::ImplementedUnverified;
    return result;
  }
  if (!detector.passed) {
    result.availability = Availability::Alert;
    result.reason = detector.reason.empty()
        ? "detector threshold exceeded" : detector.reason;
    return result;
  }
  result.nominal_component_m = nominalMultiplier(risk_.nominal_axis_tail) *
      covariance.diagonal().cwiseMax(0.0).cwiseSqrt();
  result.fault_component_m.setZero();
  for (const auto& sensitivity : sensitivities) {
    if (!sensitivity.finite) {
      result.reason = sensitivity.reason;
      result.formal_eligible = false;
      result.label = IntegrityLabel::ImplementedUnverified;
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
  result.hpl_m = std::hypot(result.pl_xyz_m.x(), result.pl_xyz_m.y());
  result.vpl_m = result.pl_xyz_m.z();
  if (!result.pl_xyz_m.allFinite() || !std::isfinite(result.hpl_m) ||
      !std::isfinite(result.vpl_m)) {
    result.formal_eligible = false;
    result.label = IntegrityLabel::ImplementedUnverified;
    result.reason = "non-finite protection level";
  } else if (result.hpl_m <= risk_.horizontal_alert_limit_m &&
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
  output.measurement_group_size = batch.measurements.size();
  output.state.timestamp = batch.timestamp;
  output.state.position_world_m = solution.position_world_m;
  output.detector = snapshotDetector(solution);
  output.postfit_detector = output.detector;
  output.uwb_postfit_residual_statistic = output.detector.statistic;
  const auto hypotheses = currentAnchorHypotheses(batch);
  if (output.detector.numerically_valid) {
    for (const auto& hypothesis : hypotheses) {
      output.sensitivities.push_back(snapshotSensitivity(
          batch, solution, hypothesis, output.detector.threshold));
    }
  }
  std::string gate_reason;
  bool model_gate = snapshotFormalGate(batch, solution, hypotheses, &gate_reason);
  model_gate = model_gate && std::all_of(
      output.sensitivities.begin(), output.sensitivities.end(),
      [](const SensitivityResult& sensitivity) { return sensitivity.finite; });
  if (gate_reason.empty() && !model_gate) {
    gate_reason = "snapshot contains an unmonitorable physical fault mode";
  }
  output.measurement_model_valid = model_gate;
  output.protection_level = protectionLevel(
      batch.timestamp, solution.covariance_m2, output.detector,
      output.sensitivities, hypotheses, solution.diagnostics,
      LinearizationConsistency::Strict, model_gate, gate_reason);
  for (std::size_t i = 0; i < batch.measurements.size() &&
       i < static_cast<std::size_t>(solution.residual_whitened.size()); ++i) {
    output.residual_records.push_back(ResidualRecord{
        batch.timestamp, batch.measurements[i].factor_id,
        batch.measurements[i].anchor_id, RowRole::Measurement,
        solution.residual_raw_m(static_cast<Eigen::Index>(i)),
        solution.residual_whitened(static_cast<Eigen::Index>(i))});
  }
  output.batch_committed = output.detector.passed && model_gate;
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
  if (!hypothesis.monitored || hypothesis.missed_detection_allocation <= 0.0 ||
      hypothesis.missed_detection_allocation >= 1.0) {
    result.reason = hypothesis.pruning_reason.empty()
        ? "physical fault hypothesis is not monitored"
        : hypothesis.pruning_reason;
    return result;
  }
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
  auto stage_start = std::chrono::steady_clock::now();
  IntegrityOutput output;
  output.timestamp = batch.timestamp;
  output.measurement_group_size = batch.measurements.size();
  output.state = snapshot.state();
  output.detector.detector_type = "conditional_current_uwb_innovation_chi_square";
  output.detector.p_fa = risk_.p_fa;
  output.detector.dof = static_cast<int>(batch.measurements.size());
  output.protection_level.timestamp = batch.timestamp;
  output.protection_level.consistency = snapshot.consistency();
  const auto hypotheses = currentAnchorHypotheses(batch);
  output.protection_level.hmi_risk_requirement = risk_.p_hmi_total;
  output.protection_level.risk_budget_valid = riskBudgetValid(
      hypotheses, &output.protection_level.allocated_hmi_risk);

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
      rows.residual.size() != output.detector.dof ||
      rows.measurement_ids.size() != batch.measurements.size() ||
      rows.anchor_ids.size() != batch.measurements.size() ||
      rows.column_indices.size() != 15 || rows.whitening_model_id.empty() ||
      rows.robust_weight != 1.0 || rows.covariance.rows() != output.detector.dof ||
      rows.covariance.cols() != output.detector.dof ||
      rows.whitener.rows() != output.detector.dof ||
      rows.whitener.cols() != output.detector.dof ||
      rows.jacobian_raw.rows() != output.detector.dof ||
      rows.jacobian_raw.cols() != 15 ||
      rows.residual_raw.size() != output.detector.dof) {
    output.detector.reason = "conditional row/version dimension mismatch";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
  for (int column = 0; column < 15; ++column) {
    if (rows.column_indices[static_cast<std::size_t>(column)] != column) {
      output.detector.reason = "conditional row column provenance is incomplete";
      output.protection_level.reason = output.detector.reason;
      return output;
    }
  }
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    if (rows.measurement_ids[i] != batch.measurements[i].id ||
        rows.anchor_ids[i] != batch.measurements[i].anchor_id) {
      output.detector.reason = "conditional measurement provenance mismatch";
      output.protection_level.reason = output.detector.reason;
      return output;
    }
  }
  const Eigen::MatrixXd whitened_identity = rows.whitener * rows.covariance *
      rows.whitener.transpose();
  if (!whitened_identity.isApprox(
          Eigen::MatrixXd::Identity(output.detector.dof, output.detector.dof),
          1e-10) ||
      !rows.jacobian.isApprox(rows.whitener * rows.jacobian_raw, 1e-10) ||
      !rows.residual.isApprox(rows.whitener * rows.residual_raw, 1e-10)) {
    output.detector.reason = "conditional whitening audit failed";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
  std::string gate_reason;
  if (!completePhysicalFaultMap(batch, hypotheses, &gate_reason)) {
    output.detector.reason = gate_reason;
    output.protection_level.reason = gate_reason;
    return output;
  }
  const auto& prior = *prior_optional;
  Eigen::LLT<Eigen::Matrix<double, 15, 15>> prior_llt(prior.covariance);
  if (!prior.covariance.allFinite() || prior_llt.info() != Eigen::Success) {
    output.detector.reason = "conditional prior is not finite SPD 15x15";
    output.protection_level.reason = output.detector.reason;
    return output;
  }
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
  output.stage_timings.push_back({
      "conditional_statistic",
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - stage_start).count(), true});
  stage_start = std::chrono::steady_clock::now();
  if (!output.detector.passed) {
    output.detector.reason = output.detector.numerically_valid
        ? "conditional innovation threshold exceeded"
        : (snapshot.diagnostics().reason.empty() ? "conditional model invalid"
                                                 : snapshot.diagnostics().reason);
  }
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
  output.postfit_detector.detector_type =
      "current_uwb_posterior_residual_heuristic";
  output.postfit_detector.statistic = output.uwb_postfit_residual_statistic;
  output.postfit_detector.dof = output.detector.dof;
  output.postfit_detector.numerically_valid =
      std::isfinite(output.postfit_detector.statistic);
  // A threshold is intentionally not invented here. The diagnostic must be
  // calibrated on independent data before `passed` has meaning.
  output.postfit_detector.threshold =
      std::numeric_limits<double>::quiet_NaN();
  output.postfit_detector.passed = false;
  output.postfit_detector.reason =
      "independent empirical calibration required";

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
  for (const auto& hypothesis : hypotheses) {
    output.sensitivities.push_back(conditionalSensitivity(
        batch, hypothesis, w, innovation_cov, gain, protected_jacobian,
        output.detector.dof, output.detector.threshold));
  }
  output.stage_timings.push_back({
      "hypothesis_incidence_slope",
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - stage_start).count(), true});
  stage_start = std::chrono::steady_clock::now();
  bool model_gate = snapshot.diagnostics().model_valid &&
      snapshot.diagnostics().rank == 15 &&
      std::isfinite(snapshot.diagnostics().condition_number) &&
      snapshot.diagnostics().condition_number <= max_condition_number_ &&
      std::isfinite(snapshot.diagnostics().linearization_step_norm) &&
      std::all_of(output.sensitivities.begin(), output.sensitivities.end(),
                  [](const SensitivityResult& sensitivity) {
                    return sensitivity.finite;
                  });
  if (!model_gate) {
    gate_reason = snapshot.diagnostics().reason.empty()
        ? "conditional formal model gate failed" : snapshot.diagnostics().reason;
  }
  output.measurement_model_valid = model_gate;
  output.protection_level = protectionLevel(
      batch.timestamp, protected_covariance, output.detector,
      output.sensitivities, hypotheses, snapshot.diagnostics(),
      snapshot.consistency(), model_gate, gate_reason);
  output.stage_timings.push_back({
      "protection_level_risk_availability",
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - stage_start).count(),
      output.protection_level.formal_eligible});
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    output.residual_records.push_back(ResidualRecord{
        batch.timestamp, batch.measurements[i].factor_id,
        batch.measurements[i].anchor_id, RowRole::Measurement,
        rows.residual_raw(static_cast<Eigen::Index>(i)),
        rows.residual(static_cast<Eigen::Index>(i))});
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
  estimator_->validateUwbBatch(batch);
  estimator_->predictTo(batch.timestamp);
  try {
    const auto snapshot = estimator_->preMeasurementSnapshot(batch);
    IntegrityOutput output = monitor_.evaluateConditional(batch, *snapshot);
    if (output.detector.passed && output.measurement_model_valid) {
      estimator_->commitUwbBatch(batch);
      output.state = estimator_->currentState();
      output.batch_committed = true;
    } else {
      estimator_->rejectUwbBatch(batch, output.detector.reason);
      output.state = estimator_->currentState();
      output.batch_committed = false;
    }
    if (estimator_->globalDiagnosticsEnabled()) {
      output.global_graph_residual_statistic =
          estimator_->globalGraphResidualStatistic();
    }
    output.global_detector.detector_type =
        "all_in_graph_residual_diagnostic";
    output.global_detector.statistic = output.global_graph_residual_statistic;
    output.global_detector.threshold =
        std::numeric_limits<double>::quiet_NaN();
    output.global_detector.numerically_valid =
        std::isfinite(output.global_detector.statistic);
    output.global_detector.reason = estimator_->globalDiagnosticsEnabled()
        ? "independent empirical calibration required"
        : "disabled on formal/performance path";
    return output;
  } catch (...) {
    if (estimator_->hasPendingEpoch()) {
      estimator_->rejectUwbBatch(batch, "conditional processing exception");
    }
    throw;
  }
}

}  // namespace uwb_imu_pl
