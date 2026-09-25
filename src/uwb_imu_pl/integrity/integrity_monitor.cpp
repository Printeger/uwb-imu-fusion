#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include "uwb_imu_pl/common/failure_reason.hpp"
#include "uwb_imu_pl/common/integrity_identity.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/coverage_envelope.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"
#include "uwb_imu_pl/publication/final_output_packet.hpp"

#include "uwb_imu_pl/estimation/candidate_replay.hpp"
#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"
#include <boost/filesystem.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>

#include <algorithm>
#include <Eigen/Cholesky>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iterator>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace uwb_imu_pl {

bool bridgeTimeoutExceeded(std::uint32_t consecutive_epochs,
                           double duration_s,
                           std::uint32_t max_consecutive_epochs,
                           double max_duration_s) {
  return consecutive_epochs > max_consecutive_epochs ||
      duration_s > max_duration_s;
}
namespace {

// Explicit test dependencies are an external sidecar so the public pipeline
// object retains its golden-p0-05 layout. Entries are never ambient: only the
// setter creates one, and construction/destruction erase by object address so
// address reuse cannot inherit a prior object's seam.
std::mutex g_pipeline_test_seams_mutex;
std::unordered_map<const RealtimeIntegrityPipeline*,
                   std::shared_ptr<const PipelineTestDependencySeamsV1>>
    g_pipeline_test_seams;

std::shared_ptr<const PipelineTestDependencySeamsV1> pipelineTestSeams(
    const RealtimeIntegrityPipeline* pipeline) {
  std::lock_guard<std::mutex> lock(g_pipeline_test_seams_mutex);
  const auto found = g_pipeline_test_seams.find(pipeline);
  return found == g_pipeline_test_seams.end() ? nullptr : found->second;
}

void clearPipelineTestSeams(const RealtimeIntegrityPipeline* pipeline) {
  std::lock_guard<std::mutex> lock(g_pipeline_test_seams_mutex);
  g_pipeline_test_seams.erase(pipeline);
}

double chiSquareThreshold(int dof, double p_fa) {
  if (dof <= 0) throw std::invalid_argument("chi-square DOF must be positive");
  return StatisticalBoundsCache::chiSquaredThreshold(dof, p_fa);
}

double nominalMultiplier(double two_sided_tail) {
  return StatisticalBoundsCache::normalTwoSidedMultiplier(two_sided_tail);
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
  return StatisticalBoundsCache::noncentralityBoundary(dof, threshold, p_md);
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

IntegrityOutput IntegrityMonitor::evaluateSnapshotImpl(
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

IntegrityOutput IntegrityMonitor::evaluateConditionalImpl(
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

namespace {

bool actionCovers(const ExclusionAction& action,
                  const FaultHypothesisV2& hypothesis) {
  const bool groups_covered = std::all_of(
      hypothesis.affected_groups.begin(), hypothesis.affected_groups.end(),
      [&](FactorGroupId group) {
        return std::find(action.groups_to_remove.begin(),
                         action.groups_to_remove.end(), group) !=
               action.groups_to_remove.end();
      });
  if (!groups_covered) return false;
  if (!hypothesis.modes.empty()) {
    return std::all_of(hypothesis.modes.begin(), hypothesis.modes.end(),
        [&](FaultModeId mode) {
          return std::find(action.covered_modes.begin(),
                           action.covered_modes.end(), mode) !=
                 action.covered_modes.end();
        });
  }
  return std::all_of(hypothesis.units.begin(), hypothesis.units.end(),
      [&](FaultUnitId unit) {
        return std::find(action.covered_units.begin(),
                         action.covered_units.end(), unit) !=
               action.covered_units.end();
      });
}

// Pure, frozen-input eligibility check.  A negative result proves that an
// action cannot become the FDE winner regardless of its numerical kernel or
// PL result.  The exhaustive development mode bypasses this check so the same
// frozen input can be used as a decision-equivalence oracle.
std::string actionIneligibilityReason(
    const ExclusionAction& action,
    const LinearizedIntegrityWindow& window,
    const std::vector<HypothesisId>& plausible,
    const std::map<std::uint64_t, const FaultHypothesisV2*>& hypotheses,
    const std::vector<FactorGroupId>& mandatory_groups,
    std::uint32_t max_cardinality) {
  if (action.exclusion_cardinality > static_cast<int>(max_cardinality)) {
    return "cardinality exceeds frozen contract";
  }
  if (action.recoverability != HistoryRecoverability::Recoverable) {
    return "history replacement is not recoverable";
  }
  for (const auto id : plausible) {
    const auto found = hypotheses.find(id.value());
    if (found == hypotheses.end() || !actionCovers(action, *found->second)) {
      return "does not cover complete frozen plausible set";
    }
  }
  for (const auto group : mandatory_groups) {
    if (std::find(action.groups_to_remove.begin(), action.groups_to_remove.end(),
                  group) == action.groups_to_remove.end()) {
      return "does not satisfy mandatory health exclusion";
    }
  }
  std::set<std::uint64_t> frozen_groups;
  for (const auto& block : window.blocks) {
    frozen_groups.insert(block.group_id.value());
  }
  std::set<std::uint64_t> removed;
  for (const auto group : action.groups_to_remove) {
    if (!removed.insert(group.value()).second) {
      return "duplicate removal target";
    }
    if (!frozen_groups.count(group.value())) {
      return "removal target absent from frozen window";
    }
  }
  std::map<std::uint64_t, const LinearizedFactorBlock*> added;
  for (const auto& block : action.added_blocks) {
    const auto inserted = added.emplace(block.group_id.value(), &block);
    if (!inserted.second) return "replacement blocks share a group id";
  }
  std::set<std::uint64_t> added_ids;
  for (const auto group : action.groups_to_add) {
    if (!added_ids.insert(group.value()).second) {
      return "duplicate addition target";
    }
    if (!added.count(group.value())) {
      return "addition target has no frozen block content";
    }
  }
  return {};
}

// Independent dense formulation retained as a diagnostic/oracle; production
// post-FDE PL uses projectPostActionModes() below and never stores dense A per
// combination hypothesis.
[[maybe_unused]] std::vector<FaultHypothesisV2>
denseRemapRemainingHypothesesOracle(
    const LinearizedIntegrityWindow& window, const EpochTransaction& tx,
    const GeneratedFaultModelSet& models, const ExclusionAction& action) {
  std::vector<FaultHypothesisV2> remaining;
  std::map<std::uint64_t, const FaultUnit*> unit_by_id;
  for (const auto& unit : models.units) unit_by_id[unit.id.value()] = &unit;
  std::map<std::uint64_t, AnchorId> anchor_by_measurement;
  std::map<std::uint64_t, TimestampNs> time_by_measurement;
  for (const auto& measurement : tx.uwb_batch.measurements) {
    anchor_by_measurement[measurement.id.value()] = measurement.anchor_id;
    time_by_measurement[measurement.id.value()] = measurement.timestamp;
  }
  for (const auto& history : tx.recoverable_history) {
    for (const auto& measurement : history.uwb_batch.measurements) {
      anchor_by_measurement[measurement.id.value()] = measurement.anchor_id;
      time_by_measurement[measurement.id.value()] = measurement.timestamp;
    }
  }
  std::map<std::uint64_t, const FaultModeBasis*> mode_by_id;
  for (const auto& mode : models.modes) mode_by_id[mode.id.value()] = &mode;
  std::map<std::uint64_t, int> block_offset;
  int aggregate_offset = 0;
  for (const auto& block : window.blocks) {
    block_offset[block.group_id.value()] = aggregate_offset;
    aggregate_offset += block.residual_whitened.size();
  }
  for (const auto& hypothesis : models.hypotheses) {
    if (actionCovers(action, hypothesis)) continue;
    FaultHypothesisV2 mapped = hypothesis;
    int parameter_dimension = 0;
    for (const auto id : hypothesis.modes) {
      const auto found = mode_by_id.find(id.value());
      if (found != mode_by_id.end()) parameter_dimension += found->second->parameter_dimension;
    }
    Eigen::MatrixXd full_map = Eigen::MatrixXd::Zero(window.H.rows(), parameter_dimension);
    int parameter_offset = 0;
    for (const auto id : hypothesis.modes) {
      const auto found = mode_by_id.find(id.value());
      if (found == mode_by_id.end()) continue;
      const auto& mode = *found->second;
      for (const auto& group_map : mode.raw_group_maps) {
        const auto offset = block_offset.find(group_map.first.value());
        const auto block = std::find_if(window.blocks.begin(), window.blocks.end(),
            [&](const LinearizedFactorBlock& value) {
              return value.group_id == group_map.first;
            });
        if (offset != block_offset.end() && block != window.blocks.end()) {
          full_map.block(offset->second, parameter_offset,
                         group_map.second.rows(), group_map.second.cols()) =
              block->whitener * group_map.second;
        }
      }
      parameter_offset += mode.parameter_dimension;
    }
    mapped.A = Eigen::MatrixXd::Zero(0, parameter_dimension);
    int source_offset = 0;
    for (const auto& block : window.blocks) {
      const int rows = block.residual_whitened.size();
      if (std::find(action.groups_to_remove.begin(),
                    action.groups_to_remove.end(), block.group_id) ==
          action.groups_to_remove.end()) {
        const int previous_rows = mapped.A.rows();
        mapped.A.conservativeResize(previous_rows + rows, Eigen::NoChange);
        mapped.A.bottomRows(rows) = full_map.middleRows(source_offset, rows);
      }
      source_offset += rows;
    }
    for (const auto& added : action.added_blocks) {
      const int previous_rows = mapped.A.rows();
      const int rows = added.residual_whitened.size();
      mapped.A.conservativeResize(previous_rows + rows, Eigen::NoChange);
      mapped.A.bottomRows(rows).setZero();
      if (added.kind != FactorKind::UwbBatch) continue;
      const auto group = std::find_if(tx.uwb_groups.begin(), tx.uwb_groups.end(),
          [&](const PendingFactorGroup& value) {
            return value.id == added.group_id;
          });
      const PendingFactorGroup* selected_group =
          group == tx.uwb_groups.end() ? nullptr : &*group;
      if (!selected_group) {
        for (const auto& history : tx.recoverable_history) {
          const auto historical = std::find_if(
              history.groups.begin(), history.groups.end(),
              [&](const PendingFactorGroup& value) {
                return value.id == added.group_id;
              });
          if (historical != history.groups.end()) {
            selected_group = &*historical;
            break;
          }
        }
      }
      if (!selected_group) continue;
      int column = 0;
      for (const auto mode_id : hypothesis.modes) {
        const auto mode = mode_by_id.find(mode_id.value());
        if (mode == mode_by_id.end()) continue;
        if (mode->second->sensor != SensorType::Uwb) {
          column += mode->second->parameter_dimension;
          continue;
        }
        Eigen::MatrixXd raw = Eigen::MatrixXd::Zero(
            rows, mode->second->parameter_dimension);
        for (std::size_t row = 0; row < selected_group->source_measurements.size(); ++row) {
          const auto measurement_id = selected_group->source_measurements[row].value();
          const auto anchor = anchor_by_measurement.find(measurement_id);
          if (anchor != anchor_by_measurement.end() &&
              anchor->second.value() == mode->second->anchor_id.value()) {
            raw(static_cast<Eigen::Index>(row), 0) = 1.0;
            if (mode->second->kind == FaultKind::AnchorBiasRamp &&
                mode->second->parameter_dimension == 2) {
              const auto time = time_by_measurement.find(measurement_id);
              if (time != time_by_measurement.end()) {
                raw(static_cast<Eigen::Index>(row), 1) =
                    time->second.seconds() - mode->second->onset_time.seconds();
              }
            }
          }
        }
        mapped.A.block(previous_rows, column, rows,
                       mode->second->parameter_dimension) =
            added.whitener * raw;
        column += mode->second->parameter_dimension;
      }
    }
    remaining.push_back(std::move(mapped));
  }
  return remaining;
}

struct FrozenCandidateIndexes {
  std::map<std::uint64_t, const FaultUnit*> units;
  std::map<std::uint64_t, const FaultModeBasis*> modes;
  std::map<std::uint64_t, const FaultHypothesisV2*> hypotheses;
  std::map<std::uint64_t, const FaultHypothesisV2*> single_unit_hypotheses;
  std::map<std::uint64_t, std::size_t> evidence;
  std::map<std::uint64_t, const PendingFactorGroup*> groups;
  std::map<std::uint64_t, const HistoricalEpochContext*> history_by_group;
  std::map<std::uint64_t, AnchorId> anchor_by_measurement;
  std::map<std::uint64_t, TimestampNs> time_by_measurement;
  FrozenCandidateIndexes(const EpochTransaction& tx, const GeneratedFaultModelSet& models,
                         const std::vector<FaultModeEvidence>& values) {
    for (const auto& unit : models.units) units.emplace(unit.id.value(), &unit);
    for (const auto& mode : models.modes) modes.emplace(mode.id.value(), &mode);
    for (const auto& h : models.hypotheses) {
      hypotheses.emplace(h.id.value(), &h);
      if (h.units.size() == 1) single_unit_hypotheses.emplace(h.units.front().value(), &h);
    }
    for (std::size_t i=0; i<values.size(); ++i) evidence.emplace(values[i].hypothesis.value(), i);
    auto batch = [&](const UwbBatch& b) {
      for (const auto& m : b.measurements) {
        anchor_by_measurement[m.id.value()] = m.anchor_id;
        time_by_measurement[m.id.value()] = m.timestamp;
      }
    };
    batch(tx.uwb_batch);
    for (const auto& group : tx.uwb_groups) groups.emplace(group.id.value(), &group);
    for (const auto& history : tx.recoverable_history) {
      batch(history.uwb_batch);
      for (const auto& group : history.groups) {
        groups.emplace(group.id.value(), &group);
        history_by_group.emplace(group.id.value(), &history);
      }
    }
  }
};

struct ProjectedPostActionModes {
  std::vector<FaultHypothesisV2> hypotheses;
  std::map<std::uint64_t, Eigen::MatrixXd> mode_maps;
};

ProjectedPostActionModes projectPostActionModes(
    const LinearizedIntegrityWindow& window, const EpochTransaction& tx,
    const GeneratedFaultModelSet& models, const ExclusionAction& action,
    const FrozenCandidateIndexes& indexes) {
  ProjectedPostActionModes projected;
  projected.hypotheses = projectRemainingHypothesesForActionV1(
      models.hypotheses, action);
  for (auto& residual : projected.hypotheses) {
    residual.affected_groups.clear();
    for (const auto mode_id : residual.modes) {
      const auto mode = indexes.modes.find(mode_id.value());
      if (mode == indexes.modes.end()) continue;
      residual.affected_groups.insert(residual.affected_groups.end(),
          mode->second->affected_groups.begin(),
          mode->second->affected_groups.end());
    }
    std::sort(residual.affected_groups.begin(), residual.affected_groups.end());
    residual.affected_groups.erase(std::unique(residual.affected_groups.begin(),
                                               residual.affected_groups.end()),
                                   residual.affected_groups.end());
    residual.A.resize(0, 0);
  }
  std::set<std::uint64_t> required_modes;
  for (const auto& hypothesis : projected.hypotheses)
    for (const auto id : hypothesis.modes) required_modes.insert(id.value());
  const auto& anchor_by_measurement = indexes.anchor_by_measurement;
  const auto& time_by_measurement = indexes.time_by_measurement;
  auto pending_group = [&](FactorGroupId id) -> const PendingFactorGroup* {
    const auto found = indexes.groups.find(id.value());
    return found == indexes.groups.end() ? nullptr : found->second;
  };
  int candidate_rows = 0;
  for (const auto& block : window.blocks) {
    if (std::find(action.groups_to_remove.begin(), action.groups_to_remove.end(),
                  block.group_id) == action.groups_to_remove.end()) {
      candidate_rows += block.residual_whitened.size();
    }
  }
  for (const auto& block : action.added_blocks) {
    candidate_rows += block.residual_whitened.size();
  }
  for (const auto& mode : models.modes) {
    if (!required_modes.count(mode.id.value())) continue;
    Eigen::MatrixXd mode_map = Eigen::MatrixXd::Zero(
        candidate_rows, mode.parameter_dimension);
    int row_offset = 0;
    for (const auto& block : window.blocks) {
      if (std::find(action.groups_to_remove.begin(), action.groups_to_remove.end(),
                    block.group_id) != action.groups_to_remove.end()) continue;
      const auto raw = mode.raw_group_maps.find(block.group_id);
      if (raw != mode.raw_group_maps.end()) {
        mode_map.middleRows(row_offset, block.residual_whitened.size()) =
            block.whitener * raw->second;
      }
      row_offset += block.residual_whitened.size();
    }
    for (const auto& added : action.added_blocks) {
      if (added.kind == FactorKind::UwbBatch && mode.sensor == SensorType::Uwb) {
        const auto* group = pending_group(added.group_id);
        if (group) {
          Eigen::MatrixXd raw = Eigen::MatrixXd::Zero(
              added.residual_whitened.size(), mode.parameter_dimension);
          for (std::size_t row = 0; row < group->source_measurements.size(); ++row) {
            const auto measurement_id = group->source_measurements[row].value();
            const auto anchor = anchor_by_measurement.find(measurement_id);
            if (anchor == anchor_by_measurement.end() ||
                anchor->second != mode.anchor_id) continue;
            raw(static_cast<Eigen::Index>(row), 0) = 1.0;
            if (mode.kind == FaultKind::AnchorBiasRamp &&
                mode.parameter_dimension == 2) {
              const auto time = time_by_measurement.find(measurement_id);
              if (time != time_by_measurement.end()) {
                raw(static_cast<Eigen::Index>(row), 1) =
                    time->second.seconds() - mode.onset_time.seconds();
              }
            }
          }
          mode_map.middleRows(row_offset, added.residual_whitened.size()) =
              added.whitener * raw;
        }
      }
      row_offset += added.residual_whitened.size();
    }
    if (mode.effective_basis_certified &&
        mode.effective_parameter_dimension > 0 &&
        mode.effective_parameter_basis.rows() == mode.parameter_dimension &&
        mode.effective_parameter_basis.cols() ==
            mode.effective_parameter_dimension) {
      mode_map = mode_map * mode.effective_parameter_basis;
    }
    projected.mode_maps.emplace(mode.id.value(), std::move(mode_map));
  }

  // A post-action graph can remove every row on which a mode acts (for
  // example, an epoch-local UWB bias whose complete batch was replaced by a
  // retained-measurement block).  Such a mode is no longer a fault of the
  // candidate graph.  Keeping its exactly all-zero map in the post-FDE census makes
  // the fault Gram rank deficient by construction and incorrectly rejects an
  // otherwise valid exclusion.  Drop only directions whose *candidate raw
  // measurement effect* is structurally (bitwise) zero; a tiny nonzero map is
  // retained because the fault amplitude is unbounded.  Non-finite maps are
  // also retained so the numerical gate fails closed downstream.
  std::set<std::uint64_t> inactive_modes;
  for (auto it = projected.mode_maps.begin(); it != projected.mode_maps.end();) {
    const Eigen::MatrixXd& map = it->second;
    const bool structurally_zero = map.size() == 0 || map.isZero(0.0);
    if (!structurally_zero) {
      ++it;
      continue;
    }
    inactive_modes.insert(it->first);
    it = projected.mode_maps.erase(it);
  }
  std::vector<FaultHypothesisV2> active_hypotheses;
  active_hypotheses.reserve(projected.hypotheses.size());
  for (auto& hypothesis : projected.hypotheses) {
    hypothesis.modes.erase(std::remove_if(
        hypothesis.modes.begin(), hypothesis.modes.end(),
        [&](FaultModeId id) { return inactive_modes.count(id.value()) != 0; }),
        hypothesis.modes.end());
    if (hypothesis.modes.empty()) continue;
    hypothesis.affected_groups.clear();
    for (const auto mode_id : hypothesis.modes) {
      const auto mode = indexes.modes.find(mode_id.value());
      if (mode == indexes.modes.end()) continue;
      for (const auto group : mode->second->affected_groups) {
        if (std::find(action.groups_to_remove.begin(),
                      action.groups_to_remove.end(), group) ==
            action.groups_to_remove.end()) {
          hypothesis.affected_groups.push_back(group);
        }
      }
    }
    std::sort(hypothesis.affected_groups.begin(),
              hypothesis.affected_groups.end());
    hypothesis.affected_groups.erase(
        std::unique(hypothesis.affected_groups.begin(),
                    hypothesis.affected_groups.end()),
        hypothesis.affected_groups.end());
    active_hypotheses.push_back(std::move(hypothesis));
  }
  projected.hypotheses = std::move(active_hypotheses);
  return projected;
}

}  // namespace

namespace {

// ---------------------------------------------------------------------------
// C4/W2: publication identity assembly from production values only.
// ---------------------------------------------------------------------------
//
// Every element below is a value the running pipeline actually produced:
//   * the frozen window / transaction identity,
//   * the linearization version the proofs were computed at,
//   * the C1 history summary version_digest,
//   * the C2 detector identity labels of the detectors that ran,
//   * the W1 §8.5 selection risk proof id,
//   * the fault-manifest digest and the health snapshot,
//   * the protection level, reference, timestamp and frame.
// String-valued identities are bound through the documented FNV-1a 64 mapping
// (identityHash64); nothing is substituted by a placeholder.  A value the run
// cannot supply stays zero/empty and is reported as missing.

struct IdentityMaterial {
  std::string snapshot_id_text;
  std::uint64_t state_solution_id = 0;
  std::uint64_t history_summary_id = 0;
  std::string manifest_digest_text;
  std::string scope_digest_text;
  std::string health_state_text;
  std::vector<std::string> detector_ids;
  std::uint64_t risk_proof_id = 0;
  std::string pl_detector_certificate_id;
  std::uint64_t pl_numerical_proof_identity = 0;
  Eigen::Vector3d protection_level_m = Eigen::Vector3d::Zero();
  std::string position_reference;
  std::int64_t timestamp_ns = 0;
  std::string frame_id;
};

PublicationIdentity publicationIdentityFrom(const IdentityMaterial& material) {
  PublicationIdentity identity;
  identity.snapshot_id = material.snapshot_id_text.empty()
      ? 0 : identityHash64(material.snapshot_id_text);
  identity.state_solution_id = material.state_solution_id;
  identity.history_summary_id = material.history_summary_id;
  identity.manifest_digest = material.manifest_digest_text.empty()
      ? 0 : identityHash64(material.manifest_digest_text);
  identity.scope_digest = material.scope_digest_text.empty()
      ? 0 : identityHash64(material.scope_digest_text);
  identity.health_state = material.health_state_text.empty()
      ? 0 : identityHash64(material.health_state_text);
  for (const auto& detector : material.detector_ids) {
    if (!detector.empty()) {
      identity.detector_ids.push_back(identityHash64(detector));
    }
  }
  identity.risk_proof_id = material.risk_proof_id;
  // The pre-existing numeric PL-certificate slot carries the versioned
  // sidecar proof identity.  No public ABI layout extension is needed.
  identity.pl_detector_certificate_id =
      material.pl_numerical_proof_identity;
  identity.protection_level_m = material.protection_level_m;
  identity.position_reference = material.position_reference;
  identity.timestamp_ns = material.timestamp_ns;
  identity.frame_id = material.frame_id;
  return identity;
}

std::uint64_t publicationCertificateDigest(const PublicationIdentity& identity) {
  std::ostringstream text;
  text << identity.snapshot_id << '\x1f' << identity.state_solution_id << '\x1f'
       << identity.history_summary_id << '\x1f' << identity.manifest_digest
       << '\x1f' << identity.scope_digest
       << '\x1f' << identity.health_state << '\x1f';
  for (const auto detector : identity.detector_ids) text << detector << ',';
  text << '\x1f' << identity.risk_proof_id << '\x1f'
       << identity.pl_detector_certificate_id << '\x1f'
       << identity.timestamp_ns
       << '\x1f' << identity.frame_id << '\x1f' << identity.position_reference;
  return identityHash64(text.str());
}

std::string serializeHealthSnapshot(const HealthSnapshot& snapshot) {
  std::string text;
  for (const auto& entry : snapshot) {
    if (!text.empty()) text += '|';
    text += entry.source_id;
    text += ':';
    text += toString(entry.state);
  }
  return text;
}

std::string joinNames(const std::vector<std::string>& names) {
  std::string text;
  for (const auto& name : names) {
    if (!text.empty()) text += ';';
    text += name;
  }
  return text;
}

void copyPublicationIdentityValues(const PublicationIdentity& candidate,
                                   const std::string& detector_labels,
                                   std::uint64_t certificate_id,
                                   PublicationDiagnostics* target) {
  if (target == nullptr) return;
  target->snapshot_id = candidate.snapshot_id;
  target->state_solution_id = candidate.state_solution_id;
  target->history_summary_id = candidate.history_summary_id;
  target->manifest_digest = candidate.manifest_digest;
  target->scope_digest = candidate.scope_digest;
  target->health_state = candidate.health_state;
  target->detector_ids = detector_labels;
  target->risk_proof_id = candidate.risk_proof_id;
  target->pl_detector_certificate_id =
      candidate.pl_detector_certificate_id;
  target->certificate_id = certificate_id;
}

void runPublicationGate(PublicationController* controller,
                        IntegrityOutput* output, const UwbBatch& batch,
                        const IdentityMaterial& material,
                        const std::string& detector_labels) {
  IdentityMaterial certificate_material = material;
  certificate_material.timestamp_ns = batch.timestamp.value();
  IdentityMaterial candidate_material = material;
  candidate_material.timestamp_ns = output->timestamp.value();
  PublicationIdentity certificate = publicationIdentityFrom(certificate_material);
  const std::vector<std::string> certificate_missing =
      missingPublicationIdentityFields(certificate);
  const bool certificate_available = certificate_missing.empty();
  certificate.certificate_id = certificate_available
      ? publicationCertificateDigest(certificate) : 0;
  PublicationIdentity candidate = publicationIdentityFrom(candidate_material);
  std::string pl_proof_reason;
  const bool pl_proof_valid = material.pl_numerical_proof_identity == 0 ||
      validateProtectionLevelPublicationProof(
          material.pl_numerical_proof_identity,
          material.protection_level_m,
          material.pl_detector_certificate_id,
          &pl_proof_reason);
  if (controller == nullptr) {
    // No state holder was supplied: the gate did not run and the output must
    // say so instead of implying a check passed.
    const std::string reason =
        "no publication state holder supplied for this stateless monitor path";
    output->publication = PublicationDiagnostics{};
    output->publication.gate_executed = false;
    output->publication.identity_check = "NOT_EVALUATED";
    output->publication.identity_reason = reason;
    output->publication.refusal =
        reason + ": output is explicitly unprotected";
    output->publication.unprotected_output = true;
    output->publication.missing_identity_fields =
        joinNames(missingPublicationIdentityFields(candidate));
    copyPublicationIdentityValues(candidate, detector_labels, 0,
                                  &output->publication);
    return;
  }
  const bool certification_available =
      output->protection_level.formal_eligible &&
      output->measurement_model_valid && pl_proof_valid;
  const PublicationDiagnosis diagnosis = controller->finalizeAttempt(
      candidate, certificate, certificate_available, certification_available);
  output->publication = PublicationDiagnostics{};
  output->publication.gate_executed = diagnosis.gate_executed;
  output->publication.identity_check = diagnosis.identity_check;
  output->publication.identity_reason = diagnosis.identity_reason;
  output->publication.state_before = diagnosis.state_before;
  output->publication.state_after = diagnosis.state_after;
  output->publication.transition = diagnosis.transition;
  output->publication.transition_accepted = diagnosis.transition_accepted;
  output->publication.protected_output = diagnosis.protected_output;
  output->publication.unprotected_output = diagnosis.unprotected_output;
  output->publication.refusal = diagnosis.refusal;
  output->publication.missing_identity_fields =
      joinNames(diagnosis.missing_identity_fields);
  output->publication.watchdog_valid = diagnosis.watchdog_valid;
  output->publication.wall_elapsed_ns = diagnosis.wall_elapsed_ns;
  output->publication.sensor_elapsed_ns = diagnosis.sensor_elapsed_ns;
  output->publication.sensor_delta_ns = diagnosis.sensor_delta_ns;
  output->publication.sensor_lag_ns = diagnosis.sensor_lag_ns;
  output->publication.sensor_stale = diagnosis.sensor_stale;
  output->publication.wall_timeout = diagnosis.wall_timeout;
  output->publication.replay_clock_jumped = diagnosis.replay_clock_jumped;
  output->publication.clock_refused = diagnosis.clock_refused;
  output->publication.watchdog_reason = diagnosis.watchdog_reason;
  copyPublicationIdentityValues(candidate, detector_labels,
                                certificate.certificate_id,
                                &output->publication);
}

constexpr const char* kActionSearchOccurrenceAudit =
    "ACTION_SEARCH_OCCURRENCE_V1";
constexpr const char* kActionSearchCertificateAudit =
    "ACTION_SEARCH_CERTIFICATE_V1";

const char* actionSearchLifecycleName(ActionSearchLifecycleV1 lifecycle) {
  switch (lifecycle) {
    case ActionSearchLifecycleV1::ReadyForEvaluation:
      return "READY_FOR_EVALUATION";
    case ActionSearchLifecycleV1::AbortedIncompleteBeforeEvaluation:
      return "ABORTED_INCOMPLETE_BEFORE_EVALUATION";
  }
  return "INVALID_LIFECYCLE";
}

CandidateAuditRecord* actionSearchOccurrenceFor(
    IntegrityOutput* output, const ExclusionAction& action) {
  const std::string operation = exactActionOperationIdentityV1(action);
  const std::string semantic = exactActionSemanticIdentityV1(action);
  const std::string planned = "PLANNED_EVALUATION;operation_identity=" +
      operation + ";semantic_identity=" + semantic;
  const auto found = std::find_if(
      output->candidate_audit.begin(), output->candidate_audit.end(),
      [&](const CandidateAuditRecord& record) {
        return record.action_type == kActionSearchOccurrenceAudit &&
            record.model_error_record == planned;
      });
  return found == output->candidate_audit.end() ? nullptr : &*found;
}

void markActionSearchTerminal(IntegrityOutput* output,
                              const ExclusionAction& action,
                              const std::string& terminal) {
  if (auto* record = actionSearchOccurrenceFor(output, action)) {
    record->diagnostics.skip_reason = terminal;
    record->diagnostics.kernel_evaluated =
        terminal == "CANDIDATE_COMPLETE" || terminal == "POST_COMPLETE" ||
        terminal == "PL_COMPLETE" || terminal == "POST_EXCEPTION" ||
        terminal == "PL_EXCEPTION";
    record->diagnostics.pl_evaluated = terminal == "PL_COMPLETE";
    record->valid = terminal == "PL_COMPLETE";
    record->reason = "terminal=" + terminal;
  }
}

void refreshActionSearchAudit(IntegrityOutput* output) {
  std::uint64_t planned = 0;
  std::uint64_t candidate_actual = 0;
  std::uint64_t post_actual = 0;
  std::uint64_t pl_actual = 0;
  std::uint64_t omitted = 0;
  for (auto& record : output->candidate_audit) {
    if (record.action_type != kActionSearchOccurrenceAudit) continue;
    if (record.model_error_record.rfind("PLANNED_EVALUATION;", 0) == 0) {
      ++planned;
    }
    else ++omitted;
    const std::string& state = record.diagnostics.skip_reason;
    if (state == "CANDIDATE_COMPLETE" || state == "POST_COMPLETE" ||
        state == "PL_COMPLETE" || state == "POST_EXCEPTION" ||
        state == "PL_EXCEPTION") ++candidate_actual;
    if (state == "POST_COMPLETE" || state == "PL_COMPLETE" ||
        state == "PL_EXCEPTION") ++post_actual;
    if (state == "PL_COMPLETE") ++pl_actual;
  }
  output->diagnostics.kernel_evaluated_actions = candidate_actual;
  output->diagnostics.post_passed_actions = post_actual;
  output->diagnostics.pl_evaluated_actions = pl_actual;
  const std::string prefix = "ACTION_SEARCH_CERTIFICATE_V1:";
  auto reason = std::find_if(output->reason_codes.begin(),
                             output->reason_codes.end(),
      [&](const std::string& value) { return value.rfind(prefix, 0) == 0; });
  if (reason == output->reason_codes.end()) return;
  const std::size_t static_begin = reason->find("lifecycle=");
  const std::string static_certificate = static_begin == std::string::npos
      ? std::string() : ";" + reason->substr(static_begin);
  *reason = prefix + "generated=" +
      std::to_string(output->diagnostics.generated_actions) +
      ";planned_evaluations=" + std::to_string(planned) +
      ";actual_candidate_evaluations=" + std::to_string(candidate_actual) +
      ";actual_post_evaluations=" + std::to_string(post_actual) +
      ";actual_pl_evaluations=" + std::to_string(pl_actual) +
      ";omitted=" + std::to_string(omitted) + static_certificate;
}

void stageActionSearchAudit(IntegrityOutput* output,
                            const ActionSearchResultV1& search,
                            const GeneratedActionSnapshotV1& trusted) {
  const ActionSearchValidationV1 validation = validateActionSearchCensusV1(
      search.census, trusted, search.max_evaluated_actions,
      search.lifecycle, search.actions);
  std::map<std::string, const ActionOmissionV1*> omissions;
  for (const auto& omission : search.census.omitted_actions) {
    omissions.emplace(omission.occurrence_identity, &omission);
  }
  std::set<std::uint64_t> used_audit_ids;
  for (const auto& action : search.actions) {
    if (action.id.value() != 0) used_audit_ids.insert(action.id.value());
  }
  std::uint64_t next_audit_id = 1;
  auto allocate_audit_id = [&]() {
    while (next_audit_id != 0 && used_audit_ids.count(next_audit_id)) {
      ++next_audit_id;
    }
    const std::uint64_t allocated = next_audit_id;
    if (next_audit_id != 0) {
      used_audit_ids.insert(next_audit_id);
      ++next_audit_id;
    }
    return allocated;
  };
  auto join_group_ids = [](const std::vector<FactorGroupId>& ids) {
    std::string joined;
    for (const auto id : ids) {
      if (!joined.empty()) joined += ';';
      joined += std::to_string(id.value());
    }
    return joined;
  };
  for (const auto& occurrence : search.census.generated_records) {
    CandidateAuditRecord record;
    record.action_type = kActionSearchOccurrenceAudit;
    record.physical_source_ids = occurrence.occurrence_identity;
    record.removal_data_source = trusted.snapshot_identity;
    const auto omitted = omissions.find(occurrence.occurrence_identity);
    const auto source = std::find_if(
        trusted.actions.begin(), trusted.actions.end(),
        [&](const ExclusionAction& action) {
          return exactActionSemanticIdentityV1(action) ==
              occurrence.semantic_identity;
        });
    if (source != trusted.actions.end()) {
      record.removed_group_ids = join_group_ids(source->groups_to_remove);
      record.added_group_ids = join_group_ids(source->groups_to_add);
    }
    if (omitted == omissions.end()) {
      record.action_id = occurrence.action_id;
      record.model_error_record = "PLANNED_EVALUATION;operation_identity=" +
          occurrence.operation_identity + ";semantic_identity=" +
          occurrence.semantic_identity;
      record.model_error_validated = validation.valid;
      record.diagnostics.skip_reason = "PLANNED_NOT_RUN";
      record.reason = "terminal=PLANNED_NOT_RUN";
    } else {
      // Omitted duplicate occurrences may intentionally share their source
      // action ID.  Give audit-only rows a positive unique CSV key while the
      // immutable occurrence identity above retains the original action ID.
      record.action_id = allocate_audit_id();
      record.model_error_record = "omission_reason=" +
          omitted->second->reason + ";operation_identity=" +
          occurrence.operation_identity + ";semantic_identity=" +
          occurrence.semantic_identity;
      record.model_error_validated = validation.valid &&
          omitted->second->proven_safe;
      record.bridge_mode = omitted->second->duplicate_of_identity;
      record.diagnostics.skip_reason = "OMITTED";
      record.reason = "terminal=OMITTED;reason=" + omitted->second->reason +
          ";duplicate_of=" + omitted->second->duplicate_of_identity;
    }
    output->candidate_audit.push_back(std::move(record));
  }
  output->reason_codes.push_back(std::string(kActionSearchCertificateAudit) +
      ":lifecycle=" + actionSearchLifecycleName(search.lifecycle) +
      ";cap=" + std::to_string(search.max_evaluated_actions) +
      ";trusted_snapshot=" + trusted.snapshot_identity +
      ";generator_identity=" + trusted.generator_identity +
      ";terminal_reason=" + search.census.terminal_reason +
      ";validation=" + validation.reason +
      ";validation_valid=" + (validation.valid ? "1" : "0") +
      ";validation_exhaustive=" + (validation.exhaustive ? "1" : "0"));
  refreshActionSearchAudit(output);
}

}  // namespace

// The public stateless entry points append the C4 publication gate to the two
// bodies above.  They are defined here (after the gate helpers) because the
// gate needs the identity assembly; the bodies themselves stay unchanged.
IntegrityOutput IntegrityMonitor::evaluateSnapshot(
    const UwbBatch& batch, const SnapshotSolution& solution,
    PublicationController* publication) const {
  IntegrityOutput output = evaluateSnapshotImpl(batch, solution);
  IdentityMaterial material;
  // The snapshot solution carries no version/solution identity: nothing is
  // invented for it, the element is reported as missing instead.
  material.protection_level_m = output.protection_level.pl_xyz_m;
  if (!output.detector.detector_type.empty()) {
    material.detector_ids.push_back(output.detector.detector_type);
  }
  if (!output.postfit_detector.detector_type.empty()) {
    material.detector_ids.push_back(output.postfit_detector.detector_type);
  }
  runPublicationGate(publication, &output, batch, material,
                     joinNames(material.detector_ids));
  return output;
}

IntegrityOutput IntegrityMonitor::evaluateConditional(
    const UwbBatch& batch, const EstimationSnapshot& snapshot,
    PublicationController* publication) const {
  IntegrityOutput output = evaluateConditionalImpl(batch, snapshot);
  const auto& version = snapshot.version();
  IdentityMaterial material;
  material.snapshot_id_text = "snapshot:" + std::to_string(version.graph_version) +
      ":linpoint:" + std::to_string(version.linpoint_version) +
      ":ordering:" + std::to_string(version.ordering_version) +
      ":noise:" + std::to_string(version.noise_model_version);
  material.state_solution_id = version.linpoint_version;
  material.protection_level_m = output.protection_level.pl_xyz_m;
  if (!output.detector.detector_type.empty()) {
    material.detector_ids.push_back(output.detector.detector_type);
  }
  if (!output.postfit_detector.detector_type.empty()) {
    material.detector_ids.push_back(output.postfit_detector.detector_type);
  }
  runPublicationGate(publication, &output, batch, material,
                     joinNames(material.detector_ids));
  return output;
}

RealtimeIntegrityPipeline::RealtimeIntegrityPipeline(
    IncrementalUwbImuEstimator* estimator, IntegrityMonitor monitor,
    PublicationLimits publication_limits)
    : estimator_(estimator), monitor_(std::move(monitor)),
      health_(estimator ? estimator->config().health : HealthConfigV2{}),
      publication_(publication_limits),
      candidate_workers_(new CandidateWorkerPool(4)) {
  clearPipelineTestSeams(this);
  if (estimator_ == nullptr) throw std::invalid_argument("estimator must not be null");
  if (estimator_->config().resolved_scope.requiresUwbFaults()) {
    for (const auto& anchor : estimator_->config().anchors) {
      health_.registerSource("anchor:" + std::to_string(anchor.id.value()),
                             SensorType::Uwb);
    }
  }
  static const char* axes[] = {"x", "y", "z"};
  if (estimator_->config().resolved_scope.requiresImuFaults()) {
    for (int axis = 0; axis < 3; ++axis) {
      health_.registerSource(std::string("accel:") + axes[axis],
                             SensorType::ImuAccelerometer);
      health_.registerSource(std::string("gyro:") + axes[axis],
                             SensorType::ImuGyroscope);
    }
  }
  if (estimator_->config().resolved_scope.enabled()) {
    health_.registerSource("generic_bridge", SensorType::Bridge);
  }
  // Reconstruct the bridge watchdog when a pipeline is attached to an
  // already-committed estimator (restart/replay and fault-injection cases).
  // The ledger is authoritative: only active kinematic bridge factors count.
  for (const auto& entry : estimator_->factorLedger().entries()) {
    if (entry.lifecycle != FactorLifecycle::Active ||
        entry.kind != FactorKind::KinematicBridge) {
      continue;
    }
    ++consecutive_bridge_epochs_;
    if (!bridge_start_timestamp_ || entry.time_begin < *bridge_start_timestamp_)
      bridge_start_timestamp_ = entry.time_begin;
  }
}

RealtimeIntegrityPipeline::~RealtimeIntegrityPipeline() {
  clearPipelineTestSeams(this);
}

void RealtimeIntegrityPipeline::setTestDependencySeamsV1(
    std::shared_ptr<const PipelineTestDependencySeamsV1> seams) {
  std::lock_guard<std::mutex> lock(g_pipeline_test_seams_mutex);
  if (seams) g_pipeline_test_seams[this] = std::move(seams);
  else g_pipeline_test_seams.erase(this);
}

void RealtimeIntegrityPipeline::ingestImu(const ImuMeasurement& measurement) {
  if (reinitializer_.directive().state == ReinitializationState::Requested) {
    reinitializer_.beginWaiting();
  }
  if (reinitializer_.directive().state ==
      ReinitializationState::WaitingForTrustedImu) {
    pending_reinitialization_seed_ =
        reinitializer_.acceptTrustedImu(measurement);
    return;
  }
  if (reinitializer_.blocksUwb()) return;
  estimator_->ingestImu(measurement);
}

void RealtimeIntegrityPipeline::completeReinitialization(
    IncrementalUwbImuEstimator* estimator,
    const ImuMeasurement& trusted_boundary) {
  if (!estimator || !pending_reinitialization_seed_ ||
      reinitializer_.directive().state != ReinitializationState::Reinitialized) {
    throw std::logic_error("reinitialization completion precondition failed");
  }
  estimator_ = estimator;
  candidate_workers_->clearScratch();
  estimator_->ingestImu(trusted_boundary);
  pending_reinitialization_seed_.reset();
  consecutive_bridge_epochs_ = 0;
  bridge_start_timestamp_.reset();
  awaiting_first_clean_uwb_ = true;
  reinitializer_.complete();
}

Eigen::Matrix<double, 15, 1>
RealtimeIntegrityPipeline::reinitializationPriorSigmas(
    const Eigen::Matrix<double, 15, 1>& base) const {
  Eigen::VectorXd bound = Eigen::VectorXd::Zero(9);
  if (pending_reinitialization_seed_) {
    const double dt = std::max(0.0,
        pending_reinitialization_seed_->timestamp.seconds() -
        reinitializer_.directive().last_committed_timestamp.seconds());
    const auto& spec = estimator_->config().bridge.generic;
    bound.head<3>().setConstant(spec.angular_rate_bound_radps * dt +
        0.5 * spec.angular_acceleration_bound_radps2 * dt * dt);
    bound.segment<3>(3).setConstant(
        0.5 * spec.acceleration_bound_mps2 * dt * dt);
    bound.tail<3>().setConstant(spec.acceleration_bound_mps2 * dt);
  }
  return reinitializer_.inflatedPriorSigmas(base, bound);
}

IntegrityOutput RealtimeIntegrityPipeline::processUwbBatch(const UwbBatch& batch) {
  ClockSample sample;
  sample.wall_monotonic_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  sample.sensor_timestamp_ns = batch.timestamp.value();
  // No replay marker exists on the production path, so a backwards clock is
  // refused by the watchdog instead of being excused as a replay jump.
  sample.replay_clock_jumped = false;
  return processUwbBatch(batch, sample);
}

void RealtimeIntegrityPipeline::applyPublicationGate(IntegrityOutput* output,
                                                     const UwbBatch& batch) {
  const IntegrityConfig& cfg = estimator_->config();
  IdentityMaterial material;
  if (output->window_id != 0 && output->transaction_id != 0) {
    material.snapshot_id_text = "window:" + std::to_string(output->window_id) +
        ":transaction:" + std::to_string(output->transaction_id);
  }
  material.state_solution_id = output->linearization_version;
  const auto& summary = output->diagnostics.history_summary;
  if (summary.present && summary.valid) {
    material.history_summary_id = summary.version_digest;
  }
  if (cfg.fault_manifest) material.manifest_digest_text = cfg.fault_manifest->digest;
  material.scope_digest_text = cfg.resolved_scope.scope_digest;
  material.health_state_text = serializeHealthSnapshot(health_.snapshot());
  if (!output->detector.detector_type.empty()) {
    material.detector_ids.push_back(output->detector.detector_type);
  }
  if (!output->postfit_detector.detector_type.empty()) {
    material.detector_ids.push_back(output->postfit_detector.detector_type);
  }
  if (!output->global_detector.detector_type.empty()) {
    material.detector_ids.push_back(output->global_detector.detector_type);
  }
  material.risk_proof_id = output->diagnostics.selection_risk_proof_id;
  material.pl_detector_certificate_id =
      output->protection_level.detector_certificate_id;
  material.pl_numerical_proof_identity =
      protectionLevelPublicationProofIdentity(
          output->protection_level.pl_xyz_m,
          output->protection_level.detector_certificate_id);
  material.protection_level_m = output->protection_level.pl_xyz_m;
  material.position_reference = output->snapshot_identity.position_reference;
  material.frame_id = cfg.realtime.world_frame;
  runPublicationGate(&publication_, output, batch, material,
                     joinNames(material.detector_ids));
}

IntegrityOutput RealtimeIntegrityPipeline::processUwbBatch(
    const UwbBatch& batch, const ClockSample& clock_sample) {
  const auto start = std::chrono::steady_clock::now();
  const auto before = estimator_->currentEpoch();
  ++input_attempt_count_;
  last_attempt_output_ = IntegrityOutput();
  auto finish = [&](IntegrityOutput& output) {
    const IntegrityConfig& config = estimator_->config();
    output.attempted_timestamp = batch.timestamp;
    output.fde_profile = toString(config.resolved_scope.profile);
    output.scope_digest = config.resolved_scope.scope_digest;
    output.detector_contract_id = config.resolved_scope.detector_contract_id;
    output.state_valid = output.state.q_world_body.coeffs().allFinite() &&
        output.state.position_world_m.allFinite() &&
        output.state.velocity_world_mps.allFinite() &&
        output.state.accel_bias_mps2.allFinite() &&
        output.state.gyro_bias_radps.allFinite();
    output.fresh = output.state.timestamp == batch.timestamp &&
        output.batch_committed;
    output.deadline_missed = output.publication.wall_timeout;
    if (config.resolved_scope.profile != FdeProfile::Off) {
      if (output.protection_level.pl_xyz_m.allFinite() &&
          std::isfinite(output.protection_level.hpl_m) &&
          std::isfinite(output.protection_level.vpl_m)) {
        output.pl_status = ProtectionLevelStatus::Finite;
        output.within_alert_limits =
            output.protection_level.hpl_m <=
                config.risk.horizontal_alert_limit_m &&
            output.protection_level.vpl_m <=
                config.risk.vertical_alert_limit_m;
      } else if (output.measurement_model_valid) {
        output.pl_status = ProtectionLevelStatus::Unbounded;
      } else {
        output.pl_status = ProtectionLevelStatus::Invalid;
      }
    }
    auto& d = output.diagnostics;
    d.input_attempt_id = input_attempt_count_;
    d.input_timestamp = batch.timestamp;
    d.backend_epoch_before = before;
    d.backend_epoch_after = estimator_->currentEpoch();
    consecutive_rejections_ = output.backend_updates ? 0 : consecutive_rejections_ + 1;
    d.consecutive_rejections = consecutive_rejections_;
    d.marginalization_count = estimator_->marginalizationCount();
    const auto estimator_cache = estimator_->cacheAudit();
    const auto statistical_cache = StatisticalBoundsCache::stats();
    d.factor_block_cache_hits = estimator_cache.factor_block_hits;
    d.factor_block_cache_misses = estimator_cache.factor_block_misses;
    d.factor_block_cache_invalidations = estimator_cache.invalidations;
    d.statistical_cache_hits = statistical_cache.hits;
    d.statistical_cache_misses = statistical_cache.misses;
    d.cache_entries = estimator_cache.factor_block_entries +
        statistical_cache.entries;
    d.cache_bytes = estimator_cache.factor_block_bytes;
    d.cache_invalidation_reason = estimator_cache.last_invalidation_reason;
    const std::vector<std::string> stages = {"prepare", "integrity_window",
        "all_in_detector", "current_sensitivity", "historical_sensitivity",
        "model_generation", "hypothesis_evidence", "health_actions", "hypothesis_audit",
        "base_factorization", "candidate_evaluation", "fde_decision", "candidate_audit",
        "finalize_commit_audit", "commit", "discard", "state_audit", "shared_cache"};
    for (const auto& stage : stages) {
      if (std::none_of(output.stage_timings.begin(), output.stage_timings.end(),
          [&](const StageTiming& t) { return t.stage == stage; })) {
        const bool skipped_profile =
            config.resolved_scope.profile == FdeProfile::Off;
        output.stage_timings.push_back({
            stage, 0.0, true,
            skipped_profile ? "SKIPPED_PROFILE" : "SKIPPED",
            skipped_profile ? "FDE_DISABLED" :
                (d.reason.empty() ? "upstream gate did not reach stage"
                                  : d.reason)});
      }
    }
    output.stage_timings.push_back({"core_total", std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count(), d.status != "EXCEPTION",
        d.status, d.reason});
  };
  // C4/W2: the freshness channel is judged before any heavy FDE work.
  publication_.beginAttempt(clock_sample);
  const WatchdogState& watchdog = publication_.watchdog();
  if (!watchdog.valid) {
    // Backwards clock without a replay marker (or an unusable sample): refused
    // the same way the platform's own input validation refuses stale batches --
    // no FDE is entered and no estimator state is touched.
    throw std::invalid_argument(
        "publication watchdog refused the batch: " + watchdog.reason);
  }
  if (publication_.watchdogRefused()) {
    // Frozen data / wall-clock timeout: UNAVAILABLE is decided here, before the
    // heavy FDE, exactly so the judgement does not wait for it.
    IntegrityOutput output;
    output.timestamp = estimator_->currentState().timestamp;
    output.state = estimator_->currentState();
    output.stale_state = true;
    output.protection_level.availability = Availability::Unavailable;
    output.protection_level.formal_eligible = false;
    output.protection_level.reason = watchdog.reason;
    output.diagnostics.status = "WATCHDOG_REFUSED";
    output.diagnostics.reason = watchdog.reason;
    output.diagnostics.transaction_opened = false;
    output.stage_timings.push_back({"publication_watchdog", 0.0, false,
        "REFUSED", watchdog.reason});
    runPublicationGate(&publication_, &output, batch, IdentityMaterial{}, "");
    finish(output);
    return output;
  }
  try {
    auto output = processUwbBatchImpl(batch);
    const double finish_elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    const bool finish_deadline_missed =
        finish_elapsed_ms > estimator_->config().publication.deadline_ms;
    if (finish_deadline_missed) {
      output.protection_level.formal_eligible = false;
      const std::string prior_reason = output.protection_level.reason;
      output.protection_level.reason =
          "FINISH_DEADLINE_MISSED after " +
          std::to_string(finish_elapsed_ms) + " ms" +
          (prior_reason.empty() ? std::string() : "; upstream=" + prior_reason);
      output.reason_codes.push_back("FINISH_DEADLINE_MISSED");
    }
    applyPublicationGate(&output, batch);
    if (finish_deadline_missed) {
      // A commit that already happened remains a real commit.  The end gate
      // only downgrades publication; it never fabricates a rollback.
      output.publication.protected_output = false;
      output.publication.unprotected_output = true;
      output.publication.wall_timeout = true;
      output.publication.refusal = "FINISH_DEADLINE_MISSED";
    }
    finish(output);
    return output;
  } catch (const std::exception& error) {
    last_attempt_output_.timestamp = estimator_->currentState().timestamp;
    last_attempt_output_.state = estimator_->currentState();
    last_attempt_output_.diagnostics.status = "EXCEPTION";
    last_attempt_output_.diagnostics.reason = error.what();
    if (const auto* terminal =
            dynamic_cast<const CommitTerminalError*>(&error)) {
      last_attempt_output_.transaction_id =
          terminal->receipt().transaction_id.value();
      last_attempt_output_.backend_updates =
          terminal->receipt().backend_updates;
      last_attempt_output_.batch_committed = false;
      last_attempt_output_.stale_state = true;
      last_attempt_output_.controlled_reinitialization_required = true;
      last_attempt_output_.protection_level.formal_eligible = false;
      last_attempt_output_.protection_level.availability =
          Availability::Unavailable;
      last_attempt_output_.protection_level.reason = terminal->what();
      last_attempt_output_.reason_codes.push_back(
          "POST_MUTATION_COMMIT_TERMINAL");
      last_attempt_output_.reason_codes.push_back(
          "COMMITTED_UNPROTECTED_REINITIALIZE");
    }
    if (last_attempt_output_.stage_timings.empty()) {
      last_attempt_output_.stage_timings.push_back({"prepare", std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count(), false, "EXCEPTION", error.what()});
    }
    finish(last_attempt_output_);
    throw;
  }
}

IntegrityOutput RealtimeIntegrityPipeline::processUwbBatchImpl(const UwbBatch& batch) {
  if (reinitializer_.blocksUwb()) {
    IntegrityOutput output;
    output.timestamp = estimator_->currentState().timestamp;
    output.state = estimator_->currentState();
    output.stale_state = true;
    output.controlled_reinitialization_required = true;
    output.reinitialization_request_id =
        reinitializer_.directive().request_id.value();
    output.reinitialization_phase = toString(reinitializer_.directive().state);
    output.reinitialization_reason = reinitializer_.directive().reason;
    output.protection_level.availability = Availability::Unavailable;
    output.protection_level.formal_eligible = false;
    output.protection_level.reason = output.reinitialization_reason;
    return output;
  }
  const IntegrityConfig& cfg = estimator_->config();
  if (cfg.resolved_scope.profile == FdeProfile::Off) {
    IntegrityOutput output;
    output.attempted_timestamp = batch.timestamp;
    output.fde_profile = toString(cfg.resolved_scope.profile);
    output.scope_digest = cfg.resolved_scope.scope_digest;
    output.detector_contract_id = cfg.resolved_scope.detector_contract_id;
    const auto prepare_start = std::chrono::steady_clock::now();
    EpochTransaction transaction = estimator_->prepareEpoch(
        batch, EpochPreparationOptions::nominalOnly());
    output.transaction_id = transaction.id.value();
    output.base_graph_version = transaction.base_graph_version;
    output.linearization_version = transaction.base_version.linpoint_version;
    output.measurement_group_size = batch.measurements.size();
    output.stage_timings.push_back({
        "prepare",
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - prepare_start).count(),
        true, "EXECUTED", "nominal-only preparation"});
    EpochCommitPlan plan = EpochCommitPlan::nominalPlan(transaction);
    plan.best_effort_integrity_unavailable = true;
    const auto commit_start = std::chrono::steady_clock::now();
    CommitCertificationV1 certification;
    const CommitReceipt receipt = estimator_->commitEpochCertified(
        std::move(transaction), plan, nullptr, &certification);
    output.stage_timings.push_back({
        "commit",
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - commit_start).count(),
        receipt.backend_updates == 1, "EXECUTED", "nominal commit"});
    output.timestamp = receipt.state_timestamp;
    output.state = estimator_->currentState();
    (void)recordCommittedStatePublicationV1({
        1, receipt.transaction_id.value(), receipt.state_timestamp,
        certification.committed_covariance});
    output.batch_committed = receipt.backend_updates == 1;
    output.backend_updates = receipt.backend_updates;
    output.selected_action_type = "KEEP_ALL";
    output.fde_status = "FDE_DISABLED";
    output.detector.detector_type = "NOT_RUN";
    output.detector.reason = "FDE_DISABLED";
    output.protection_level.timestamp = receipt.state_timestamp;
    output.protection_level.formal_eligible = false;
    output.protection_level.availability = Availability::Unavailable;
    output.protection_level.reason = "FDE_DISABLED";
    output.pl_status = ProtectionLevelStatus::NotComputed;
    output.within_alert_limits = false;
    output.measurement_model_valid = true;
    output.reason_codes.push_back("FDE_DISABLED");
    output.diagnostics.status = output.batch_committed ? "COMMITTED" : "ERROR";
    output.diagnostics.reason = "FDE_DISABLED";
    return output;
  }
  const NumericalWorkSnapshot numerical_before =
      NumericalWorkCounters::snapshot();
  const auto prepare_start = std::chrono::steady_clock::now();
  EpochPreparationOptions preparation_options;
  preparation_options.build_integrity_material = true;
  preparation_options.build_uwb_recovery_material =
      cfg.resolved_scope.requiresUwbFaults();
  preparation_options.build_imu_recovery_material =
      cfg.resolved_scope.requiresImuFaults();
  preparation_options.build_recoverable_history = true;
  EpochTransaction transaction =
      estimator_->prepareEpoch(batch, preparation_options);
  IntegrityOutput output;
  std::string executing_stage = "prepare";
  auto operation_start = prepare_start;
  auto start_operation = [&](const std::string& name) {
    executing_stage = name; operation_start = std::chrono::steady_clock::now();
  };
  try {
    output.timestamp = batch.timestamp;
    output.diagnostics.ordering_version = transaction.base_version.ordering_version;
    output.diagnostics.noise_model_version = transaction.base_version.noise_model_version;
    output.diagnostics.raw_imu_samples = transaction.raw_imu_slice.size();
    output.diagnostics.pending_duration_s = transaction.end.seconds() - transaction.begin.seconds();
    output.stage_timings.push_back({"prepare", std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - prepare_start).count(), true});
    output.transaction_id = transaction.id.value();
    output.base_graph_version = transaction.base_graph_version;
    output.linearization_version = transaction.base_version.linpoint_version;
    output.measurement_group_size = batch.measurements.size();
    output.protection_level.label = IntegrityLabel::ImplementedUnverified;
    auto stage_start = std::chrono::steady_clock::now();
    auto record_stage = [&](const std::string& name, bool success = true) {
      const auto now = std::chrono::steady_clock::now();
      output.stage_timings.push_back({name,
          std::chrono::duration<double, std::milli>(now - stage_start).count(),
          success, "EXECUTED", success ? "" : "model/numerical gate rejected"});
      stage_start = now;
    };
    // Health transitions are staged with the transaction.  They become
    // externally visible only after commit/discard finalizes the epoch.
    HealthManager pending_health = health_;
    auto capture_state_audit = [&]() {
      start_operation("state_audit");
      const auto audit_start = std::chrono::steady_clock::now();
      output.factor_ledger_audit.clear();
      if (cfg.output.write_factor_ledger) {
      for (const auto& entry : estimator_->factorLedger().entries()) {
        FactorLedgerAuditRecord record;
        record.factor_id = entry.factor_id.value();
        record.group_id = entry.group_id.value();
        record.sensor = toString(entry.sensor);
        record.factor_kind = toString(entry.kind);
        record.lifecycle = toString(entry.lifecycle);
        record.epoch_begin = entry.epoch_begin;
        record.epoch_end = entry.epoch_end;
        record.time_begin = entry.time_begin;
        record.time_end = entry.time_end;
        record.backend_slot = entry.backend_slot
            ? std::to_string(*entry.backend_slot) : "";
        record.noise_model_id = entry.noise_model_id;
        record.model_id = entry.model_id;
        record.health = toString(entry.health_at_commit);
        auto append_id = [](std::string* target, std::uint64_t value) {
          if (!target->empty()) *target += ';';
          *target += std::to_string(value);
        };
        for (const auto& source : entry.source_ids) {
          if (!record.source_ids.empty()) record.source_ids += ';';
          record.source_ids += source;
        }
        for (const auto id : entry.source_measurements) {
          append_id(&record.measurement_ids, id.value());
        }
        for (const auto& unit : entry.fault_units) {
          if (!record.fault_units.empty()) record.fault_units += ';';
          record.fault_units += unit;
        }
        record.commit_graph_version = entry.commit_version.graph_version;
        record.removed_graph_version = entry.removed_version
            ? entry.removed_version->graph_version : 0;
        record.replacement_group_id = entry.replacement_group
            ? entry.replacement_group->value() : 0;
        record.replaces_group_id = entry.replaces_group
            ? entry.replaces_group->value() : 0;
        record.recovery_epoch = entry.recovery_epoch;
        output.factor_ledger_audit.push_back(std::move(record));
      }
      }
      output.health_audit.clear();
      if (cfg.output.write_health) {
      for (const auto& entry : health_.snapshot()) {
        HealthAuditRecord record;
        record.source_id = entry.source_id;
        record.sensor = toString(entry.source_type);
        record.previous_state = toString(entry.state);
        record.current_state = toString(entry.state);
        record.trigger = "EPOCH_SNAPSHOT";
        record.evidence_statistic = output.detector.statistic;
        record.evidence_threshold = output.detector.threshold;
        for (const auto& hypothesis : output.hypothesis_audit) {
          if (!hypothesis.plausible) continue;
          if (!record.plausible_hypothesis_ids.empty()) {
            record.plausible_hypothesis_ids += ';';
          }
          record.plausible_hypothesis_ids +=
              std::to_string(hypothesis.hypothesis_id);
        }
        record.selected_action_id = output.selected_action_id;
        record.suspicion_count = entry.suspicion_count;
        record.shadow_pass_count = entry.shadow_pass_count;
        record.recovery_pass_count = entry.recovery_pass_count;
        record.bridge_count = consecutive_bridge_epochs_;
        record.recovery_reset_count = entry.recovery_reset_count;
        output.health_audit.push_back(std::move(record));
      }
      }
      output.stage_timings.push_back({"state_audit", std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - audit_start).count(), true, "EXECUTED", ""});
    };

    IntegrityWindowRequest request;
    request.epochs = cfg.integrity_window.epochs;
    start_operation("integrity_window");
    const auto window = estimator_->buildIntegrityWindow(transaction, request);
    record_stage("integrity_window", window.model_valid);
    // A3: freeze the result identity block.  Only schema-provided values are
    // used; fields the schema cannot supply are marked NOT_AVAILABLE_IN_SCHEMA
    // instead of being fabricated.
    {
      IntegritySnapshotIdentity& identity = output.snapshot_identity;
      std::ostringstream fingerprint;
      fingerprint << std::hex << std::setfill('0') << std::setw(16)
                  << integrityWindowFingerprint(window);
      identity.snapshot_id = "window:" + std::to_string(window.id.value()) +
          ":transaction:" + std::to_string(transaction.id.value());
      identity.source_revision = kNotAvailableInSchema;
      identity.config_digest = cfg.config_hash.empty()
          ? kNotAvailableInSchema : cfg.config_hash;
      identity.manifest_digest = cfg.fault_manifest
          ? cfg.fault_manifest->digest : kNotAvailableInSchema;
      identity.state_solution_id = kNotAvailableInSchema;
      identity.sensor_timestamp_ns = batch.timestamp.value();
      identity.frame_id = cfg.realtime.world_frame;
      identity.position_reference = "body_origin";
      identity.tangent_convention =
          "gtsam_pose3_local_rotation_then_body_translation";
      identity.state_scale = "identity";
      identity.output_jacobian_contract =
          "[0_3x3 | R] on the final epoch pose block (body origin position)";
      identity.whitening_id = "frozen_block_whitener:" + fingerprint.str();
      identity.noise_model_id = "per_block_frozen_noise_model";
      identity.boundary_summary_id = window.capabilities.includes_boundary_prior
          ? "partial_qr_schur_boundary" : "NONE";
      identity.history_lineage_id = kNotAvailableInSchema;
      identity.protected_reference_center = "body_origin";
      std::set<std::uint64_t> frozen_groups;
      for (const auto& block : window.blocks) {
        frozen_groups.insert(block.group_id.value());
      }
      std::ostringstream groups;
      bool first_group = true;
      for (const auto group : frozen_groups) {
        if (!first_group) groups << ';';
        first_group = false;
        groups << group;
      }
      identity.active_observation_index = groups.str();
      identity.coverage_epoch = kNotAvailableInSchema;
      identity.validity_assumptions =
          "frozen_linearization;active_window_fault_support;"
          "history_nominal_boundary_only";
      identityDigest(&identity);
      if (window.square_root) {
        const auto& context = *window.square_root;
        const auto& certificate = context.certificate();
        SquareRootAuditRecord audit;
        // input_attempt_count_ is the 1-based attempt id used by the
        // diagnostics CSV and by the replay export; the diagnostics field
        // is assigned later in this function.
        audit.attempt_id = input_attempt_count_;
        audit.rows = context.rows();
        audit.columns = context.columns();
        audit.rank = context.rank();
        audit.dof = context.dof();
        audit.r_diagonal_min = certificate.r_diagonal_min;
        audit.r_diagonal_max = certificate.r_diagonal_max;
        audit.condition_estimate = certificate.condition_estimate;
        audit.identity_residual_relative =
            certificate.identity_residual_relative;
        audit.parity_relative_difference =
            certificate.parity_relative_difference;
        audit.solution_relative_difference =
            certificate.solution_relative_difference;
        audit.forward_error_bound = certificate.forward_error_bound;
        audit.certificate_ok = certificate.ok();
        audit.usable = context.usable();
        audit.detector_only_rows = context.detectorOnlyRows();
        audit.statistic = context.statistic();
        audit.scale_policy = toString(context.scalePolicy());
        audit.permutation_policy = toString(context.permutationPolicy());
        audit.reason = context.reason();
        output.square_root_audit.push_back(std::move(audit));
      }
    }
    const auto& preparation = window.preparation_timing;
    output.stage_timings.push_back({"window_boundary_provenance",
        preparation.boundary_and_provenance_ms, true, "EXECUTED", ""});
    output.stage_timings.push_back({"window_factor_linearization_whitening",
        preparation.factor_linearization_whitening_ms, true, "EXECUTED", ""});
    output.stage_timings.push_back({"window_dense_assembly",
        preparation.dense_assembly_ms, true, "EXECUTED", ""});
    output.stage_timings.push_back({"window_svd", preparation.svd_ms,
        true, "EXECUTED", ""});
    output.stage_timings.push_back({"window_normal_equations",
        preparation.normal_equations_ms, true, "EXECUTED", ""});
    output.stage_timings.push_back({"window_llt_state_solves",
        preparation.llt_and_state_solves_ms, true, "EXECUTED", ""});
    output.stage_timings.push_back({"window_fingerprint",
        preparation.fingerprint_ms, true, "EXECUTED", ""});
    output.window_id = window.id.value();
    {
      // C1-c diagnostics v15: export the condensed summary as produced (the
      // pooled terms are read here, never recomputed downstream).
      const auto& source = window.history_summary;
      auto& target = output.diagnostics.history_summary;
      target.present = source.present;
      target.valid = source.valid;
      target.capacity_ok = source.capacity_ok;
      target.state = source.state;
      target.reason = source.reason;
      target.version_digest = source.version_digest;
      target.scope_digest = source.scope_digest;
      target.fault_columns = source.fault_columns;
      target.boundary_rows = source.boundary_rows;
      target.emitted_rows = source.emitted_rows;
      target.boundary_columns = source.boundary_columns;
      target.rank_boundary = source.rank_boundary;
      target.nu_perp = source.nu_perp;
      target.kappa_b = source.kappa_b;
      target.constant_offset = source.constant_offset;
      target.omega_trace = source.detector_response.size() > 0
                               ? source.omega().trace()
                               : 0.0;
      target.xi_norm = source.d_perp.size() > 0 ? source.xi().norm() : 0.0;
      target.information_form_factors = source.information_form_factors;
      target.injected_epochs = source.injected_epochs;
      target.horizon_first_epoch = source.horizon_first_epoch;
      target.window_first_epoch = source.window_first_epoch;
      target.omitted_epoch_count = source.omitted_epoch_count;
      target.material_gap_epoch_count = source.material_gap_epoch_count;
      target.claims_full_coverage = source.claims_full_coverage;
      target.assumptions = source.assumptions;
      target.omitted_risk_source = source.omitted_risk_source;
    }
    std::set<std::uint64_t> frozen_groups;
    for (const auto& block : window.blocks) frozen_groups.insert(block.group_id.value());
    output.diagnostics.frozen_group_ids.assign(frozen_groups.begin(), frozen_groups.end());
    output.history_provenance_valid =
        window.capabilities.history_provenance_valid;
    DetectorRiskContext detector_risk;
    detector_risk.p_fa_per_test = cfg.detector.p_fa_per_test;
    detector_risk.continuity_horizon_tests =
        cfg.detector.continuity_horizon_tests;
    detector_risk.rank_tolerance = cfg.integrity_window.rank_tolerance;
    detector_risk.max_condition_number =
        cfg.integrity_window.max_condition_number;
    start_operation("all_in_detector");
    const DetectorResultV2 all_in =
        JointWindowDetector().evaluate(window, detector_risk);
    record_stage("all_in_detector", all_in.numerically_valid);
    output.detector.detector_type = "dual_channel_v1";
    output.detector.statistic = all_in.squared_parity_statistic;
    output.detector.threshold = all_in.squared_threshold;
    output.detector.dof = all_in.dof;
    output.detector.p_fa = all_in.p_fa_per_test;
    output.detector.passed = all_in.passed;
    output.detector.numerically_valid = all_in.numerically_valid;
    output.detector.reason = all_in.reason;
    output.measurement_model_valid = window.model_valid &&
        window.capabilities.fixed_lag_maturity_valid &&
        (!cfg.integrity_window.require_history_provenance ||
         window.capabilities.history_provenance_valid);

    auto timed_commit = [&](EpochTransaction&& tx, const EpochCommitPlan& plan,
                            const CommitProtectionEvidenceV1* evidence = nullptr,
                            CommitCertificationV1* certification = nullptr) {
      start_operation("commit");
      const auto start = std::chrono::steady_clock::now();
      CommitCertificationV1 local_certification;
      auto receipt = estimator_->commitEpochCertified(
          std::move(tx), plan, evidence,
          certification ? certification : &local_certification);
      const auto& final_certification = certification
          ? *certification : local_certification;
      (void)recordCommittedStatePublicationV1({
          1, receipt.transaction_id.value(), receipt.state_timestamp,
          final_certification.committed_covariance});
      output.stage_timings.push_back({"commit", std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count(), true, "EXECUTED", ""});
      return receipt;
    };
    auto timed_discard = [&](EpochTransaction&& tx, const DiscardReason& reason) {
      start_operation("discard");
      const auto start = std::chrono::steady_clock::now();
      auto receipt = estimator_->discardEpoch(std::move(tx), reason);
      output.stage_timings.push_back({"discard", std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count(), true, "EXECUTED", ""});
      return receipt;
    };
    // A3 failure accounting.  Codes come from the frozen taxonomy; the legacy
    // status/reason text stays untouched.  Checks the online path did not
    // reach are recorded as NOT_EVALUATED instead of being implied to pass.
    std::vector<FailureReason> failure_reasons;
    std::vector<std::string> not_evaluated_checks;
    auto note_failure = [&](const std::string& text) {
      const FailureReason reason = classifyFailureReason(text);
      if (reason != FailureReason::None) failure_reasons.push_back(reason);
    };
    auto note_not_evaluated = [&](const char* check) {
      for (const auto& existing : not_evaluated_checks) {
        if (existing == check) return;
      }
      not_evaluated_checks.emplace_back(check);
    };
    auto finalize_failure_accounting = [&]() {
      output.diagnostics.all_failures = joinFailureReasons(failure_reasons);
      output.diagnostics.primary_failure = failure_reasons.empty()
          ? "NONE" : toString(failure_reasons.front());
      std::ostringstream joined;
      for (std::size_t i = 0; i < not_evaluated_checks.size(); ++i) {
        if (i) joined << ';';
        joined << not_evaluated_checks[i];
      }
      output.diagnostics.not_evaluated_checks = joined.str();
    };
    auto commit_best_effort = [&](FdeStatus status,
                                  const std::string& reason) {
      EpochCommitPlan plan = EpochCommitPlan::nominalPlan(transaction);
      plan.fde_status = status;
      plan.best_effort_integrity_unavailable = true;
      const CommitReceipt receipt = timed_commit(
          std::move(transaction), plan);
      health_ = pending_health;
      output.backend_updates = receipt.backend_updates;
      output.fde_status = toString(status);
      output.selected_action_type = "KEEP_ALL_BEST_EFFORT";
      output.state = estimator_->currentState();
      output.batch_committed = true;
      output.protection_level.availability = Availability::Unavailable;
      output.protection_level.formal_eligible = false;
      output.protection_level.reason = reason;
      note_failure(reason);
      note_not_evaluated("POST_FDE_DETECTOR_AND_PL");
      note_not_evaluated("ATOMIC_CERTIFIED_PUBLISH");
      capture_state_audit();
    };
    auto discard_fail_closed = [&](FdeStatus status,
                                   const std::string& detail,
                                   bool reinitialize) {
      DiscardReason reason{status, detail, reinitialize};
      const DiscardReceipt receipt = timed_discard(
          std::move(transaction), reason);
      health_ = pending_health;
      output.timestamp = receipt.last_committed_timestamp;
      output.state = estimator_->currentState();
      output.backend_updates = receipt.backend_updates;
      output.stale_state = true;
      output.controlled_reinitialization_required =
          receipt.controlled_reinitialization_required;
      output.fde_status = toString(status);
      output.protection_level.availability = Availability::Unavailable;
      output.protection_level.reason = detail;
      if (reinitialize) {
        const auto request = reinitializer_.request(
            status, detail, estimator_->currentState());
        output.reinitialization_request_id = request.value();
        output.reinitialization_phase =
            toString(reinitializer_.directive().state);
        output.reinitialization_reason = detail;
      }
      note_failure(detail);
      note_not_evaluated("HYPOTHESIS_EVIDENCE");
      note_not_evaluated("CANDIDATE_NUMERICS");
      note_not_evaluated("POST_FDE_DETECTOR_AND_PL");
      capture_state_audit();
    };

    if (!output.measurement_model_valid || !all_in.numerically_valid) {
      const bool history_invalid =
          !window.capabilities.history_provenance_valid;
      discard_fail_closed(
          history_invalid ? FdeStatus::HistoryPriorContaminated
                          : FdeStatus::ModelInvalid,
          window.reason.empty() ? all_in.reason : window.reason,
          history_invalid);
    } else {
      start_operation("current_sensitivity");
      ImuFaultSubspaces imu_subspaces;
      LinearizedFactorBlock bridge_block;
      const bool run_imu_fd_oracle =
          cfg.resolved_scope.requiresImuFaults() &&
          std::getenv("UWB_IMU_PL_IMU_FD_ORACLE") != nullptr;
      if (cfg.resolved_scope.requiresImuFaults()) {
        const auto imu_block = estimator_->buildPendingFactorBlock(
            transaction, transaction.imu_group.id);
        bridge_block = estimator_->buildPendingFactorBlock(
            transaction, transaction.generic_bridge_group.id);
        imu_subspaces = run_imu_fd_oracle
            ? ImuFaultSubspaceBuilder().verifyFiniteDifferenceOracle(
                  transaction, imu_block)
            : ImuFaultSubspaceBuilder().buildAnalytic(transaction, imu_block);
      }
      output.diagnostics.analytic_input_valid =
          imu_subspaces.analytic_input_valid;
      output.diagnostics.analytic_computation_valid =
          imu_subspaces.analytic_computation_valid;
      output.diagnostics.oracle_executed = imu_subspaces.oracle_executed;
      output.diagnostics.oracle_relative_error =
          imu_subspaces.oracle_relative_error;
      output.diagnostics.oracle_verified = imu_subspaces.oracle_verified;
      if (run_imu_fd_oracle) {
        // A4: multi-step sweep (both signs per step, several step sizes).
        const auto imu_block = estimator_->buildPendingFactorBlock(
            transaction, transaction.imu_group.id);
        const ImuFaultSubspaces sweep = ImuFaultSubspaceBuilder()
            .verifyFiniteDifferenceSweep(transaction, imu_block);
        auto joinNumbers = [](const std::vector<double>& values) {
          std::ostringstream joined;
          joined << std::setprecision(10);
          for (std::size_t i = 0; i < values.size(); ++i) {
            if (i) joined << ';';
            joined << values[i];
          }
          return joined.str();
        };
        output.diagnostics.oracle_sweep_executed = sweep.sweep_executed;
        output.diagnostics.oracle_sweep_verified = sweep.sweep_verified;
        output.diagnostics.oracle_sweep_worst_relative_error =
            sweep.sweep_worst_relative_error;
        output.diagnostics.oracle_sweep_reintegrations =
            sweep.sweep_reintegrations;
        output.diagnostics.oracle_sweep_epsilons =
            joinNumbers(sweep.sweep_epsilons);
        output.diagnostics.oracle_sweep_relative_errors =
            joinNumbers(sweep.sweep_relative_errors);
      }
      record_stage("current_sensitivity");
      if (cfg.resolved_scope.requiresImuFaults() &&
          (!imu_subspaces.analytic_input_valid ||
           !imu_subspaces.analytic_computation_valid)) {
        discard_fail_closed(
            FdeStatus::ModelInvalid,
            "analytic IMU sensitivity input/computation invalid",
            false);
      } else {
        start_operation("model_generation");
        HypothesisGeneratorConfig generator_config;
        generator_config.include_uwb_faults =
            cfg.resolved_scope.requiresUwbFaults();
        generator_config.include_imu_faults =
            cfg.resolved_scope.requiresImuFaults();
        generator_config.single_faults_enabled =
            cfg.resolved_scope.max_fault_order >= 1;
        generator_config.double_faults_enabled =
            cfg.resolved_scope.max_fault_order >= 2;
        generator_config.max_model_cardinality =
            cfg.fault_models.max_cardinality;
        generator_config.max_exclusion_cardinality =
            cfg.fde.max_exclusion_cardinality;
        generator_config.max_candidate_count = cfg.fde.max_candidate_count;
        generator_config.uwb_prior_bound =
            cfg.fault_models.uwb.prior_probability_bound;
        generator_config.accel_prior_bound =
            cfg.fault_models.imu.accel_prior_probability_bound;
        generator_config.gyro_prior_bound =
            cfg.fault_models.imu.gyro_prior_probability_bound;
        generator_config.uwb_p_md = cfg.fault_models.uwb.p_md;
        generator_config.imu_p_md = cfg.fault_models.imu.p_md;
        generator_config.include_uwb_accel_combinations =
            cfg.fault_models.combinations.uwb_plus_accel;
        generator_config.include_uwb_gyro_combinations =
            cfg.fault_models.combinations.uwb_plus_gyro;
        generator_config.include_epoch_independent_uwb =
            cfg.fault_models.uwb.epoch_single_anchor_bias;
        generator_config.include_persistent_uwb =
            cfg.fault_models.uwb.persistent_anchor_bias;
        generator_config.include_ramp_uwb = cfg.fault_models.uwb.ramp_bias;
        generator_config.total_hmi_allocation =
            conservativeRemainingHypothesisRisk(cfg.risk_v2);
        generator_config.rank_tolerance =
            cfg.integrity_window.rank_tolerance;
        auto models = HypothesisGenerator(generator_config).generate(
            window, transaction, imu_subspaces, bridge_block);
        // B2/B3: the online traversal is exact, so the certificate for this
        // window is built from the frozen registry before any decision is
        // published.  (Grouped envelopes stay an opt-in path.)
        const CoverageCertificate coverage_certificate =
            buildExactCoverageCertificate(models.modes);
        const RiskBudgetAudit risk_audit =
            auditRiskBudget(cfg.risk_v2, models.hypotheses);
        output.diagnostics.risk_nominal = risk_audit.nominal;
        output.diagnostics.risk_p_nm = risk_audit.p_nm;
        output.diagnostics.risk_bridge = risk_audit.bridge;
        output.diagnostics.risk_history = risk_audit.history;
        output.diagnostics.risk_model = risk_audit.model;
        output.diagnostics.risk_hypotheses = risk_audit.hypotheses;
        output.diagnostics.risk_total = risk_audit.total;
        output.diagnostics.risk_upper_bound = risk_audit.upper_bound;
        output.diagnostics.risk_margin = risk_audit.margin;
        output.diagnostics.hypothesis_count = risk_audit.hypothesis_count;
        // B3 (§5.9): the ledger certificate travels with the published
        // attempt: every term with its value, source and status, so an
        // unvalidated channel can never pass as a proven zero.
        {
          RiskLedgerInputs ledger_inputs;
          if (cfg.fault_manifest) {
            for (const auto& event : cfg.fault_manifest->events) {
              for (const auto& omitted : event.omitted_event_set) {
                if (std::find(ledger_inputs.omitted_event_set.begin(),
                              ledger_inputs.omitted_event_set.end(),
                              omitted) ==
                    ledger_inputs.omitted_event_set.end()) {
                  ledger_inputs.omitted_event_set.push_back(omitted);
                }
              }
            }
          }
          for (const auto& omitted : cfg.resolved_scope.omitted_families) {
            const std::string scoped = "inactive_profile_family:" + omitted;
            if (std::find(ledger_inputs.omitted_event_set.begin(),
                          ledger_inputs.omitted_event_set.end(), scoped) ==
                ledger_inputs.omitted_event_set.end()) {
              ledger_inputs.omitted_event_set.push_back(scoped);
            }
          }
          ledger_inputs.envelope_online = false;
          ledger_inputs.envelope_leaf_count =
              coverage_certificate.enveloped_count;
          ledger_inputs.selection_contract_frozen = false;
          const RiskLedger ledger = buildRiskLedger(
              cfg.risk_v2, models.hypotheses, ledger_inputs);
          output.diagnostics.risk_ledger_charged_total =
              ledger.charged_total;
          output.diagnostics.risk_ledger_declared_total =
              ledger.declared_total;
          output.diagnostics.risk_ledger_closes = ledger.closes;
          output.diagnostics.risk_ledger_all_validated =
              ledger.all_terms_validated;
          for (const auto& term : ledger.terms) {
            if (term.status == RiskTermStatus::Validated) {
              ++output.diagnostics.risk_ledger_validated_terms;
            } else if (term.status == RiskTermStatus::AssumedUnvalidated) {
              ++output.diagnostics.risk_ledger_unvalidated_terms;
            } else {
              ++output.diagnostics.risk_ledger_not_implemented_terms;
            }
            if (!output.diagnostics.risk_ledger_terms.empty()) {
              output.diagnostics.risk_ledger_terms += ";";
            }
            output.diagnostics.risk_ledger_terms += term.id + "=" +
                std::to_string(term.value) + ":" + toString(term.status);
          }
        }
        // B3 (§5.8): the coverage certificate is part of the same attempt
        // output that crosses the existing history-validity gate; the
        // certificate itself never bypasses that gate.
        output.diagnostics.coverage_status = coverage_certificate.complete
            ? "COMPLETE" : "INCOMPLETE";
        output.diagnostics.coverage_exact_leaves =
            coverage_certificate.exact_count;
        output.diagnostics.coverage_enveloped_leaves =
            coverage_certificate.enveloped_count;
        output.diagnostics.coverage_uncovered_leaves =
            coverage_certificate.uncovered_count;
        output.diagnostics.coverage_envelope_count =
            coverage_certificate.envelopes.size();
        output.diagnostics.coverage_proof_count = 0;
        for (const auto& envelope : coverage_certificate.envelopes) {
          if (envelope.accepted) {
            ++output.diagnostics.coverage_accepted_envelope_count;
          }
          output.diagnostics.coverage_proof_count += envelope.proofs.size();
        }
        output.diagnostics.coverage_envelope_online = false;
        output.diagnostics.single_uwb_hypotheses =
            models.single_uwb_hypotheses;
        output.diagnostics.single_accel_hypotheses =
            models.single_accel_hypotheses;
        output.diagnostics.single_gyro_hypotheses =
            models.single_gyro_hypotheses;
        output.diagnostics.double_uwb_accel_hypotheses =
            models.double_uwb_accel_hypotheses;
        output.diagnostics.double_uwb_gyro_hypotheses =
            models.double_uwb_gyro_hypotheses;
        output.diagnostics.effective_fault_cardinality =
            risk_audit.effective_max_cardinality;
        if (risk_audit.single_fault_hypothesis_count !=
                models.single_uwb_hypotheses +
                    models.single_accel_hypotheses +
                    models.single_gyro_hypotheses ||
            risk_audit.double_fault_hypothesis_count !=
                models.double_uwb_accel_hypotheses +
                    models.double_uwb_gyro_hypotheses ||
            risk_audit.other_cardinality_hypothesis_count != 0 ||
            risk_audit.effective_max_cardinality !=
                models.effective_max_cardinality) {
          throw std::logic_error(
              "generated fault census and risk audit disagree");
        }
        record_stage("model_generation");
        output.stage_timings.back().wall_ms -= models.historical_sensitivity_ms;
        output.stage_timings.push_back({"historical_sensitivity", models.historical_sensitivity_ms, true});
        start_operation("hypothesis_evidence");
        HypothesisEvaluationConfig evidence_config;
        evidence_config.rank_tolerance = cfg.integrity_window.rank_tolerance;
        evidence_config.max_condition_number =
            cfg.integrity_window.max_condition_number;
        evidence_config.min_fault_gram_sigma =
            cfg.fault_models.imu.min_fault_gram_sigma;
        evidence_config.max_fault_gram_condition =
            cfg.fault_models.imu.max_fault_gram_condition;
        evidence_config.enable_shared_context =
            std::getenv("UWB_IMU_PL_DISABLE_HYPOTHESIS_SHARED") == nullptr;
        evidence_config.enable_low_dim_batch =
            std::getenv("UWB_IMU_PL_DISABLE_HYPOTHESIS_BATCH") == nullptr;
        evidence_config.retain_detailed_results =
            cfg.output.write_hypothesis_evidence ||
            std::getenv("UWB_IMU_PL_IMU_CALIBRATION_AUDIT") != nullptr;
        constexpr std::size_t kCandidateWorkers = 4;
        const char* worker_override = std::getenv("UWB_IMU_PL_CANDIDATE_WORKERS");
        std::size_t active_workers = kCandidateWorkers;
        if (worker_override) {
          try {
            active_workers = std::max<std::size_t>(1, std::min<std::size_t>(
                kCandidateWorkers, static_cast<std::size_t>(
                    std::stoul(worker_override))));
          } catch (...) {
            throw std::invalid_argument(
                "UWB_IMU_PL_CANDIDATE_WORKERS must be an integer in [1,4]");
          }
        }
        evidence_config.hypothesis_workers = active_workers;
        evidence_config.fault_model_policy_fingerprint =
            faultScopePolicyFingerprint(cfg.fault_models, cfg.resolved_scope);
        std::shared_ptr<const FrozenHypothesisNumerics>
            shared_hypothesis_numerics;
        NumericalWorkCounters::evidenceCallFaultPath();
        auto evidence = HypothesisEvidenceEvaluator(evidence_config).evaluateAll(
            window, models.modes, &models.hypotheses,
            all_in.squared_threshold, &shared_hypothesis_numerics,
            candidate_workers_.get());
        record_stage("hypothesis_evidence", true);
        if (shared_hypothesis_numerics && !output.stage_timings.empty()) {
          const auto& shared = *shared_hypothesis_numerics;
          output.stage_timings.back().reason =
              "effective_dimensions=1:" +
              std::to_string(shared.dimension_one_count) + ",2:" +
              std::to_string(shared.dimension_two_count) + ",3:" +
              std::to_string(shared.dimension_three_count) + ",other:" +
              std::to_string(shared.dimension_other_count) +
              ";context_bytes=" + std::to_string(shared.bytes) +
              ";worker_blocks=" + std::to_string(shared.worker_blocks);
        }
        start_operation("health_actions");
        const FrozenCandidateIndexes indexes(transaction, models, evidence);
        // B2 (§5.8): coverage certificate for this window.  The online
        // traversal is exact, so every enumerated mode is an EXACT leaf; the
        // label is exported per hypothesis (diagnostics v13).  Grouped
        // envelopes are an opt-in capacity path and are only ever used when
        // both the inclusion proof and the dominance obligation hold
        // (see coverage_envelope.hpp).

        bool hardware_barrier = false;
        struct SourceEvidence {
          SensorType sensor = SensorType::Unknown;
          bool suspicious = false;
          std::vector<std::size_t> evidence_indices;
        };
        std::map<std::string, SourceEvidence> source_evidence;
        for (const auto& unit : models.units) {
          const std::string source = healthSourceId(unit);
          if (source.empty()) continue;
          const auto single = indexes.single_unit_hypotheses.find(unit.id.value());
          if (single == indexes.single_unit_hypotheses.end()) continue;
          const auto item = indexes.evidence.find(single->second->id.value());
          auto& aggregate = source_evidence[source];
          aggregate.sensor = unit.sensor;
          if (item != indexes.evidence.end()) {
            aggregate.suspicious = aggregate.suspicious ||
                (evidence[item->second].plausible && !all_in.passed);
            aggregate.evidence_indices.push_back(item->second);
          }
        }
        std::vector<std::string> mandatory_health_sources;
        std::vector<FactorGroupId> mandatory_exclusion_groups;
        for (auto& source_item : source_evidence) {
          const std::string& source = source_item.first;
          auto& aggregate = source_item.second;
          pending_health.registerSource(source, aggregate.sensor);
          if (pending_health.allowedInFormalEstimator(source)) {
            NumericalWorkCounters::evidenceCallHealthPath();
            (void)pending_health.observeEvidence(source, aggregate.suspicious);
          } else {
            (void)pending_health.observeShadowRecovery(
                source, !aggregate.suspicious);
          }
          if (!pending_health.allowedInFormalEstimator(source)) {
            hardware_barrier = true;
            mandatory_health_sources.push_back(source);
            const bool is_uwb = source.rfind("anchor:", 0) == 0;
            FactorGroupId group = transaction.imu_group.id;
            if (is_uwb) {
              const auto nominal = std::find_if(
                  transaction.uwb_groups.begin(), transaction.uwb_groups.end(),
                  [](const PendingFactorGroup& value) { return value.nominal; });
              if (nominal == transaction.uwb_groups.end()) {
                throw std::logic_error(
                    "quarantined UWB source has no pending nominal group");
              }
              group = nominal->id;
            }
            if (std::find(mandatory_exclusion_groups.begin(),
                          mandatory_exclusion_groups.end(), group) ==
                mandatory_exclusion_groups.end()) {
              mandatory_exclusion_groups.push_back(group);
            }
          }
        }
        NumericalWorkCounters::actionEntitiesDeferred(
            models.action_entities_deferred);
        NumericalWorkCounters::bridgeBlocksBuilt(models.bridge_blocks_built);
        NumericalWorkCounters::actionEntitiesConstructed(
            models.action_entities_constructed);
        ActionSearchResultV1 action_search;
        // Held independently from the certificate/result object and passed
        // directly from the production generator boundary to the FDE
        // consumer.  The certificate cannot substitute its own raw vector.
        GeneratedActionSnapshotV1 trusted_generated_actions;
        if (all_in.passed && !hardware_barrier) {
          ExclusionAction keep;
          keep.id = ExclusionActionId(1);
          keep.action_model_id = "KEEP_ALL";
          action_search = censusAndCapActionsV1(
              {keep}, 1, &trusted_generated_actions);
          models.actions = action_search.actions;
        } else {
          // B4: this is the alarm/mandatory path: the exclusion-action
          // entities and their bridge blocks are materialized here, once.
          NumericalWorkCounters::candidateGraphBuilt();
          action_search = HypothesisGenerator(generator_config)
              .actionsForPlausibleSetV1(window, transaction, &models, evidence,
                                        mandatory_health_sources,
                                        &trusted_generated_actions);
          // Unsafe omission is terminal before any candidate kernel/post/PL.
          // This makes evaluated an actual count (zero), and ensures an outer
          // exception still carries the complete fail-closed census.
          abortIncompleteActionSearchBeforeEvaluationV1(&action_search);
          models.actions = action_search.actions;
          NumericalWorkCounters::actionEntitiesConstructed(
              models.action_entities_constructed);
          NumericalWorkCounters::bridgeBlocksBuilt(models.bridge_blocks_built);
        }
        const auto generation_seams = pipelineTestSeams(this);
        if (generation_seams &&
            generation_seams->after_action_generation_before_census) {
          // Exercise the actual pipeline boundary with a test-supplied raw
          // occurrence mutation, then rebuild both the trusted snapshot and
          // census from those exact bytes.  The callback cannot edit a staged
          // certificate and has no global/environment activation path.
          std::vector<ExclusionAction> raw_generated =
              action_search.generated_snapshot.actions;
          generation_seams->after_action_generation_before_census(
              &raw_generated);
          action_search = censusAndCapActionsV1(
              raw_generated, action_search.max_evaluated_actions,
              &trusted_generated_actions);
          trusted_generated_actions.generator_identity =
              "RealtimeIntegrityPipeline/test-seam-after-production-generator";
          action_search.generated_snapshot.generator_identity =
              trusted_generated_actions.generator_identity;
          models.actions = action_search.actions;
        }
        const ActionSearchCensusV1& action_search_census = action_search.census;
        output.diagnostics.generated_actions = action_search_census.generated;
        // Freeze the complete raw search certificate before the first
        // candidate/router/post/PL/FDE call.  Every exception path below moves
        // this same output into last_attempt_output_; no catch reconstructs a
        // second, self-reported census.
        stageActionSearchAudit(
            &output, action_search, trusted_generated_actions);
        if (!action_search_census.exhaustive) {
          output.fde_status = "SEARCH_INCOMPLETE";
          output.reason_codes.push_back("SEARCH_INCOMPLETE");
          output.protection_level.availability = Availability::Unavailable;
          output.protection_level.risk_budget_valid = false;
          output.protection_level.formal_eligible = false;
          note_failure("SEARCH_INCOMPLETE");
        }
        const bool exhaustive_candidates =
            std::getenv("UWB_IMU_PL_EXHAUSTIVE_CANDIDATES") != nullptr;
        const std::vector<HypothesisId> frozen_plausible =
            (!all_in.passed || hardware_barrier)
                ? completePlausibleHypotheses(models.hypotheses, evidence)
                : std::vector<HypothesisId>{};
        std::map<std::uint64_t, const FaultHypothesisV2*> frozen_hypotheses;
        for (const auto& hypothesis : models.hypotheses) {
          frozen_hypotheses.emplace(hypothesis.id.value(), &hypothesis);
        }
        std::map<std::uint64_t, std::string> action_ineligibility;
        std::vector<ExclusionAction> kernel_actions;
        kernel_actions.reserve(models.actions.size());
        for (const auto& action : models.actions) {
          std::string reason = actionIneligibilityReason(
              action, window, frozen_plausible, frozen_hypotheses,
              mandatory_exclusion_groups,
              cfg.fde.max_exclusion_cardinality);
          if (!reason.empty()) action_ineligibility.emplace(action.id.value(), reason);
          if (exhaustive_candidates || reason.empty()) kernel_actions.push_back(action);
        }
        record_stage("health_actions");
        start_operation("hypothesis_audit");
        const bool calibration_hypothesis_audit =
            std::getenv("UWB_IMU_PL_IMU_CALIBRATION_AUDIT") != nullptr;
        if (cfg.output.write_hypothesis_evidence || calibration_hypothesis_audit) {
        std::map<double, double> noncentrality_boundaries;
        for (const auto& hypothesis : models.hypotheses) {
          if (!noncentrality_boundaries.count(hypothesis.p_md_allocation)) {
            noncentrality_boundaries[hypothesis.p_md_allocation] =
                IntegrityMonitor::noncentralityBoundary(
                    all_in.dof, all_in.squared_threshold,
                    hypothesis.p_md_allocation);
          }
        }
        for (std::size_t i = 0; i < models.hypotheses.size(); ++i) {
          const auto& hypothesis = models.hypotheses[i];
          if (!cfg.output.write_hypothesis_evidence) {
            bool target = hypothesis.modes.size() == 1;
            if (target) {
              const auto mode = indexes.modes.find(hypothesis.modes.front().value());
              target = mode != indexes.modes.end() &&
                  mode->second->sensor == SensorType::ImuAccelerometer &&
                  mode->second->axis == 0;
            }
            if (!target) continue;
          }
          HypothesisAuditRecord record;
          record.hypothesis_id = hypothesis.id.value();
          const bool format_hypothesis_strings = true;
          for (std::size_t j = 0; j < hypothesis.units.size(); ++j) {
            if (format_hypothesis_strings) {
              if (j) record.fault_unit_ids += ';';
              record.fault_unit_ids += std::to_string(hypothesis.units[j].value());
            }
            const auto unit = indexes.units.find(hypothesis.units[j].value());
            if (format_hypothesis_strings && unit != indexes.units.end()) {
              if (!record.physical_source_ids.empty()) record.physical_source_ids += ';';
              record.physical_source_ids += unit->second->physical_source_id;
            }
          }
          std::set<std::string> sensors;
          std::set<std::string> kinds;
          record.onset_epoch = std::numeric_limits<std::uint64_t>::max();
          record.onset_time_ns = std::numeric_limits<std::int64_t>::max();
          for (const auto mode_id : hypothesis.modes) {
            if (format_hypothesis_strings) {
              if (!record.mode_ids.empty()) record.mode_ids += ';';
              record.mode_ids += std::to_string(mode_id.value());
            }
            const auto found = indexes.modes.find(mode_id.value());
            if (found == indexes.modes.end()) continue;
            const auto* mode = found->second;
            sensors.insert(toString(mode->sensor)); kinds.insert(toString(mode->kind));
            record.onset_epoch = std::min<std::uint64_t>(record.onset_epoch, mode->onset_epoch);
            record.onset_time_ns = std::min(record.onset_time_ns, mode->onset_time.value());
            record.parameter_dimension += mode->parameter_dimension;
          }
          auto join_strings = [](const std::set<std::string>& values) {
            std::string joined;
            for (const auto& value : values) {
              if (!joined.empty()) joined += ';';
              joined += value;
            }
            return joined;
          };
          if (format_hypothesis_strings) {
            record.sensor = join_strings(sensors);
            record.fault_kind = join_strings(kinds);
          }
          if (record.onset_epoch == std::numeric_limits<std::uint64_t>::max()) {
            record.onset_epoch = 0;
          }
          if (record.onset_time_ns == std::numeric_limits<std::int64_t>::max()) {
            record.onset_time_ns = 0;
          }
          record.prior_bound = hypothesis.prior_probability_bound;
          record.p_md_allocation = hypothesis.p_md_allocation;
          record.hmi_allocation = hypothesis.hmi_allocation;
          record.monitorable = hypothesis.monitored;
          // C4 diagnostics v16 (§7.1): the comparability identity and the RAW
          // whitened profile value of this hypothesis, copied from the
          // evidence record that produced them.
          {
            const auto evidence_index =
                indexes.evidence.find(hypothesis.id.value());
            if (evidence_index != indexes.evidence.end() &&
                evidence_index->second < evidence.size()) {
              const FaultModeEvidence& hypothesis_evidence =
                  evidence[evidence_index->second];
              record.unit_kind = toString(hypothesis_evidence.unit_kind);
              record.profile_j = hypothesis_evidence.profile_j;
              record.profile_valid = hypothesis_evidence.profile_valid;
            }
          }
          {
            bool covered = !hypothesis.modes.empty();
            for (const auto mode_id : hypothesis.modes) {
              if (indexes.modes.find(mode_id.value()) == indexes.modes.end()) {
                covered = false;
                break;
              }
            }
            const FaultModeId reference =
                covered ? hypothesis.modes.front() : FaultModeId(0);
            record.coverage_label = toString(
                covered ? coverageLabelFor(coverage_certificate, reference)
                        : CoverageLabel::Uncovered);
            record.coverage_envelope_id =
                covered ? coverageEnvelopeIdFor(coverage_certificate, reference)
                        : 0;
          }
          if (i < evidence.size()) {
            record.plausible = evidence[i].plausible;
            record.conditioned_statistic = evidence[i].conditioned_statistic;
            record.log_evidence = evidence[i].log_evidence;
            if (format_hypothesis_strings) {
              record.reason = evidence[i].monitorability.reason;
              if (record.reason.empty()) record.reason = "MONITORABLE";
            }
            record.fault_rank = evidence[i].monitorability.rank;
            record.sigma_min = evidence[i].monitorability.sigma_min;
            record.sigma_max = evidence[i].monitorability.sigma_max;
            record.condition_number = evidence[i].monitorability.condition_number;
            record.slope_xyz = evidence[i].monitorability.protected_slopes;
            if (evidence[i].fault_gram.rows() > 0) {
              const int direction = evidence[i].fault_gram.rows() == 2 ? 1 : 0;
              record.boundary_direction_gram =
                  evidence[i].fault_gram(direction, direction);
            }
            record.noncentrality_boundary =
                noncentrality_boundaries.at(hypothesis.p_md_allocation);
            if (shared_hypothesis_numerics &&
                i < shared_hypothesis_numerics->pl_entries.size()) {
              const auto& entry =
                  shared_hypothesis_numerics->pl_entries[i];
              record.z_rank = entry.z_rank;
              record.z_smallest_singular_value =
                  entry.z_smallest_singular_value;
              record.z_condition = entry.z_condition;
              record.z_classification = entry.z_classification;
            }
          }
          output.hypothesis_audit.push_back(std::move(record));
        }
        record_stage("hypothesis_audit");
        } else {
          output.stage_timings.push_back({"hypothesis_audit", 0.0, true,
              "SKIPPED", "disabled by output.write_hypothesis_evidence"});
          stage_start = std::chrono::steady_clock::now();
        }
        start_operation("base_factorization");
        RankUpdateConfig rank_config;
        rank_config.rank_tolerance = cfg.integrity_window.rank_tolerance;
        rank_config.max_condition_number =
            cfg.integrity_window.max_condition_number;
        rank_config.max_linearization_step_norm =
            cfg.integrity_window.max_linearization_step_norm;
        rank_config.materialize_dense_oracle_fields = false;
        if (const char* destination = std::getenv("UWB_IMU_PL_REPLAY_EXPORT_DIR")) {
          const char* requested = std::getenv("UWB_IMU_PL_REPLAY_ATTEMPTS");
          const std::string filter = std::string(",") + (requested ? requested : "") + ",";
          if (filter.find("," + std::to_string(input_attempt_count_) + ",") != std::string::npos) {
            boost::filesystem::create_directories(destination);
            FrozenCandidateReplay replay;
            replay.input_attempt_id = input_attempt_count_;
            replay.input_timestamp = batch.timestamp;
            replay.transaction_id = transaction.id.value();
            replay.window = window; replay.actions = models.actions; replay.config = rank_config;
            replay.identity = output.snapshot_identity;
            writeCandidateReplay(std::string(destination) + "/attempt-" +
                std::to_string(input_attempt_count_) + ".bin", replay);
          }
        }
        const RankUpdateEvaluator evaluator(rank_config);
        const CandidateEvaluationRouterV1 candidate_router(rank_config);
        BaseCandidateKernel base = evaluator.factorizeOnce(window);
        output.diagnostics.base_step_norm = base.state_increment.norm();
        auto append_step_audit = [&](const Eigen::VectorXd& increment,
                                     const std::string& source) {
          for (const auto& layout : window.state_layout) {
            if (layout.dimension < 15 || layout.column_offset < 0 ||
                layout.column_offset + 15 > increment.size()) continue;
            StateStepAuditRecord record;
            record.source = source; record.epoch = layout.epoch;
            const Eigen::Index offset = layout.column_offset;
            record.rotation_norm = increment.segment(offset, 3).norm();
            record.position_norm = increment.segment(offset + 3, 3).norm();
            record.velocity_norm = increment.segment(offset + 6, 3).norm();
            record.accel_bias_norm = increment.segment(offset + 9, 3).norm();
            record.gyro_bias_norm = increment.segment(offset + 12, 3).norm();
            record.epoch_norm = increment.segment(offset, 15).norm();
            output.state_step_audit.push_back(std::move(record));
          }
        };
        append_step_audit(base.state_increment, "BASE");
        record_stage("base_factorization", base.valid);
        start_operation("shared_cache");
        evaluator.buildSharedCache(&base, kernel_actions);
        record_stage("shared_cache");
        start_operation("candidate_evaluation");
        std::vector<CandidateEvaluation> candidates;
        std::map<std::uint64_t, ProtectionLevelV2Result> candidate_pl;
        std::map<std::uint64_t, std::uint64_t> candidate_pl_proof;
        candidates.reserve(models.actions.size());
        struct EvaluatedAction {
          CandidateEvaluation candidate;
          DetectorResultV2 post;
          ProtectionLevelV2Result protection_level;
          std::uint64_t protection_level_proof_identity = 0;
          bool has_protection_level = false;
        };
        std::vector<const ExclusionAction*> action_order;
        action_order.reserve(models.actions.size());
        for (const auto& action : models.actions) action_order.push_back(&action);
        std::sort(action_order.begin(), action_order.end(),
                  [](const auto* left, const auto* right) {
          return left->id < right->id;
        });
        std::vector<EvaluatedAction> evaluated_actions(action_order.size());
        std::vector<std::size_t> kernel_action_indices;
        kernel_action_indices.reserve(action_order.size());
        for (std::size_t index = 0; index < action_order.size(); ++index) {
          const auto& action = *action_order[index];
          const auto rejected = action_ineligibility.find(action.id.value());
          if (!exhaustive_candidates && rejected != action_ineligibility.end()) {
            auto& candidate = evaluated_actions[index].candidate;
            candidate.action = action;
            candidate.base_version = window.version;
            candidate.reason = "SKIPPED_INELIGIBLE: " + rejected->second;
            candidate.diagnostics.skip_reason = candidate.reason;
            candidate.diagnostics.numerical_valid = false;
            markActionSearchTerminal(
                &output, action, "NOT_RUN_INELIGIBLE");
          } else {
            kernel_action_indices.push_back(index);
          }
        }
        // Phase 1: all numerical kernels and post detectors.  Every action has
        // a fixed slot; a worker exception is rethrown only after all workers
        // reach the pool barrier.
        candidate_workers_->run(kernel_action_indices.size(), active_workers,
            [&](std::size_t work_index, std::size_t, RankUpdateScratch& scratch) {
          const std::size_t index = kernel_action_indices[work_index];
          const ExclusionAction& action = *action_order[index];
          EvaluatedAction& evaluated = evaluated_actions[index];
          const auto candidate_start = std::chrono::steady_clock::now();
          // P6/C-round Stage 0: a step-gate rejection carries its physical
          // attribution (rotation/position/velocity/bias magnitudes and the
          // dominant block).  The gate itself is untouched: this only explains
          // an existing rejection.
          CandidateEvaluation& candidate = evaluated.candidate;
          // P0-06: the router proves the action/window/changed-row numerical
          // precondition before it is permitted to invoke the optimized rank
          // kernel.  Unknown or boundary cases execute the exact dense/SVD
          // terminal path with the same action and census slot.
          try {
            const auto candidate_seams = pipelineTestSeams(this);
            if (candidate_seams && candidate_seams->before_candidate) {
              candidate_seams->before_candidate(action.id);
            }
            candidate = candidate_router.evaluate(
                window, base, action, &scratch);
            markActionSearchTerminal(
                &output, action, "CANDIDATE_COMPLETE");
          } catch (...) {
            markActionSearchTerminal(
                &output, action, "CANDIDATE_EXCEPTION");
            throw;
          }
          // Stage 0 (C-round): explain a step-gate rejection with the physical
          // attribution of the frozen-window increment.  Threshold and
          // acceptance logic are unchanged; only the exported reason grows.
          const bool step_gate_rejected =
              candidate.diagnostics.matrix_free_step_rejected ||
              candidate.reason.find("step gate failed") != std::string::npos ||
              candidate.diagnostics.skip_reason.find("step gate") !=
                  std::string::npos ||
              candidate.diagnostics.skip_reason.find("exceeds gate") !=
                  std::string::npos;
          if (!candidate.valid && step_gate_rejected &&
              candidate.state_increment.size() ==
                  base.state_increment.size() &&
              candidate.state_increment.allFinite()) {
            const std::string attribution =
                stateStepAttribution(window, candidate.state_increment);
            if (candidate.reason.empty()) {
              candidate.reason = "candidate linearization step gate failed";
            }
            candidate.reason += "; " + attribution;
            candidate.diagnostics.skip_reason += "; " + attribution;
          }
          candidate.diagnostics.kernel_evaluated = true;
          candidate.diagnostics.numerical_valid = candidate.valid;
          candidate.diagnostics.slow_path = candidate.exact_slow_path;
          candidate.diagnostics.kernel_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - candidate_start).count();
          auto part_start = std::chrono::steady_clock::now();
          auto part_ms = [&]() {
            auto now = std::chrono::steady_clock::now();
            double ms = std::chrono::duration<double, std::milli>(now - part_start).count();
            part_start = now;
            return ms;
          };
          try {
            const auto post_seams = pipelineTestSeams(this);
            if (post_seams && post_seams->before_post_detector) {
              post_seams->before_post_detector(action.id);
            }
            evaluated.post = JointWindowDetector().evaluateCandidate(
                window, candidate, detector_risk);
            markActionSearchTerminal(&output, action, "POST_COMPLETE");
          } catch (...) {
            markActionSearchTerminal(&output, action, "POST_EXCEPTION");
            throw;
          }
          candidate.squared_threshold = evaluated.post.squared_threshold;
          candidate.post_detector_passed = evaluated.post.passed;
          candidate.diagnostics.post_ms = part_ms();
          candidate.wall_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - candidate_start).count();
        });

        // Bridge uncertainty depends only on a frozen group/transaction and is
        // shared by every successful candidate that contains the same bridge.
        std::map<std::uint64_t, Eigen::VectorXd> bridge_bounds;
        for (const auto* action : action_order) {
          if (action->bridge_mode != BridgeMode::GenericKinematic) continue;
          for (const auto& added : action->added_blocks) {
            if (added.kind != FactorKind::KinematicBridge ||
                bridge_bounds.count(added.group_id.value())) continue;
            const EpochTransaction* bridge_transaction = &transaction;
            EpochTransaction historical_transaction;
            const auto historical = indexes.history_by_group.find(added.group_id.value());
            if (historical != indexes.history_by_group.end()) {
              const auto& history = *historical->second;
              historical_transaction.previous_epoch = history.previous_epoch;
              historical_transaction.proposed_epoch = history.proposed_epoch;
              historical_transaction.begin = history.begin;
              historical_transaction.end = history.end;
              bridge_transaction = &historical_transaction;
            }
            bridge_bounds.emplace(added.group_id.value(),
                BridgeFactory().uncertainty(*bridge_transaction,
                    cfg.bridge.generic).deterministic_bound);
          }
        }

        std::vector<std::size_t> post_passed;
        for (std::size_t index = 0; index < evaluated_actions.size(); ++index) {
          const auto& evaluated = evaluated_actions[index];
          if (evaluated.candidate.valid && evaluated.post.numerically_valid &&
              evaluated.post.passed) post_passed.push_back(index);
        }
        // Phase 2: bridge/fault mapping/PL only for candidates that passed the
        // post detector.
        candidate_workers_->run(post_passed.size(), active_workers,
            [&](std::size_t passed_index, std::size_t, RankUpdateScratch&) {
          EvaluatedAction& evaluated = evaluated_actions[post_passed[passed_index]];
          CandidateEvaluation& candidate = evaluated.candidate;
          const ExclusionAction& action = candidate.action;
          auto part_start = std::chrono::steady_clock::now();
          auto part_ms = [&]() {
            const auto now = std::chrono::steady_clock::now();
            const double ms = std::chrono::duration<double, std::milli>(now - part_start).count();
            part_start = now;
            return ms;
          };
            ProtectionLevelSharedContext shared_pl;
            if (action.bridge_mode == BridgeMode::GenericKinematic) {
              for (const auto& added : action.added_blocks) {
                if (added.kind != FactorKind::KinematicBridge) continue;
                FrozenBridgeProjection bridge;
                bridge.jacobian_whitened = added.jacobian_whitened;
                bridge.deterministic_bound = bridge_bounds.at(
                    added.group_id.value());
                shared_pl.bridges.push_back(std::move(bridge));
              }
            }
            candidate.diagnostics.bridge_ms = part_ms();
            // Production PL always consumes candidate-specific dual-channel
            // detector material.  The pooled/frozen path remains an explicit
            // offline reference and is not allowed to certify a five-profile
            // output.
            auto projected_modes = projectPostActionModes(
                window, transaction, models, action, indexes);
            shared_pl.mode_maps = std::move(projected_modes.mode_maps);
            candidate.diagnostics.fault_map_ms = part_ms();
            try {
              const auto pl_seams = pipelineTestSeams(this);
              if (pl_seams && pl_seams->before_protection_level) {
                pl_seams->before_protection_level(action.id);
              }
              evaluated.protection_level = ProtectionLevelV2().computeShared(
                  window, &candidate, evaluated.post,
                  &projected_modes.hypotheses, shared_pl, cfg.risk_v2);
              markActionSearchTerminal(&output, action, "PL_COMPLETE");
            } catch (...) {
              markActionSearchTerminal(&output, action, "PL_EXCEPTION");
              throw;
            }
            if (evaluated.protection_level.model_valid) {
              std::string proof_reason;
              if (!validateProtectionLevelV2Proof(
                      candidate, evaluated.post, projected_modes.hypotheses,
                      evaluated.protection_level, &proof_reason)) {
                evaluated.protection_level.model_valid = false;
                evaluated.protection_level.availability =
                    Availability::Unavailable;
                evaluated.protection_level.reason =
                    "candidate PL proof consumer rejected: " + proof_reason;
              }
            }
            if (evaluated.protection_level.model_valid) {
              evaluated.protection_level_proof_identity =
                  protectionLevelV2ProofIdentity(
                      candidate, evaluated.protection_level);
              if (evaluated.protection_level_proof_identity == 0) {
                evaluated.protection_level.model_valid = false;
                evaluated.protection_level.availability =
                    Availability::Unavailable;
                evaluated.protection_level.reason =
                    "candidate PL exact proof identity is unavailable";
              }
            }
            evaluated.has_protection_level = true;
            candidate.diagnostics.pl_evaluated = true;
            candidate.diagnostics.pl_ms = part_ms();
            const auto& pl = evaluated.protection_level;
            candidate.pl_xyz_m = pl.pl_xyz_m;
            candidate.hpl_m = pl.hpl_m;
            candidate.vpl_m = pl.vpl_m;
            candidate.valid = candidate.valid && pl.model_valid;
            if (!pl.model_valid && candidate.reason.empty()) {
              candidate.reason = pl.reason;
            }
          candidate.wall_ms = candidate.diagnostics.kernel_ms +
              candidate.diagnostics.post_ms + candidate.diagnostics.bridge_ms +
              candidate.diagnostics.fault_map_ms + candidate.diagnostics.pl_ms;
        });
        for (auto& evaluated : evaluated_actions) {
          auto& candidate = evaluated.candidate;
          if (!candidate.diagnostics.pl_evaluated &&
              candidate.diagnostics.skip_reason.empty()) {
            candidate.diagnostics.skip_reason =
                !candidate.diagnostics.numerical_valid
                    ? "numerical rejection" : "post detector failed";
          }
          // Every invalid candidate is an auditable fail-closed result.  Some
          // low-rank kernels signal invalidity only through the boolean; do
          // not let that become an empty reason in the v6 CSV contract.
          if (!candidate.valid && candidate.reason.empty()) {
            candidate.reason = candidate.diagnostics.skip_reason.empty()
                ? "candidate numerical kernel invalid"
                : candidate.diagnostics.skip_reason;
          }
          if (evaluated.has_protection_level) {
            candidate_pl[candidate.action.id.value()] =
                evaluated.protection_level;
            candidate_pl_proof[candidate.action.id.value()] =
                evaluated.protection_level_proof_identity;
          }
          candidates.push_back(std::move(candidate));
        }
        refreshActionSearchAudit(&output);
        record_stage("candidate_evaluation", true);
        std::size_t routed_rank = 0;
        std::size_t routed_dense = 0;
        std::map<std::string, std::size_t> dense_reasons;
        for (const auto& candidate : candidates) {
          if (candidate.diagnostics.numerical_path == "DENSE_ROUTER_EXACT") {
            ++routed_dense;
            ++dense_reasons[candidate.diagnostics.fallback_reason];
          } else if (candidate.diagnostics.kernel_evaluated) {
            ++routed_rank;
          }
        }
        std::string route_reason = "rank=" + std::to_string(routed_rank) +
            ";dense=" + std::to_string(routed_dense);
        for (const auto& item : dense_reasons) {
          route_reason += ";" + item.first + "=" +
              std::to_string(item.second);
        }
        output.stage_timings.back().reason = std::move(route_reason);
        for (const auto& candidate : candidates) {
          if (!candidate.valid && !candidate.reason.empty()) {
            note_failure(candidate.reason);
          }
          if (!candidate.diagnostics.pl_evaluated &&
              candidate.diagnostics.skip_reason.rfind(
                  "SKIPPED_INELIGIBLE", 0) == 0) {
            note_not_evaluated("POST_FDE_DETECTOR_AND_PL");
          }
        }
        start_operation("fde_decision");
        DetectorResultV2 fde_trigger = all_in;
        if (hardware_barrier) {
          fde_trigger.passed = false;
          fde_trigger.reason = "quarantined/failed source hardware barrier";
        }
        FdeRiskDecisionV1 final_risk;
        FdeDecisionContextV2 fde_context;
        fde_context.v1.protection_proof_ids = &candidate_pl_proof;
        fde_context.v1.risk_result = &final_risk;
        fde_context.action_search = &action_search.census;
        fde_context.trusted_generated_actions =
            &trusted_generated_actions;
        fde_context.max_evaluated_actions =
            action_search.max_evaluated_actions;
        fde_context.action_search_lifecycle = action_search.lifecycle;
        FdeDecision decision = FdeManager().decide(
            fde_trigger, models.hypotheses, evidence, &candidates,
            mandatory_exclusion_groups, cfg.risk_v2, &fde_context);
        if (!action_search_census.exhaustive) {
          output.reason_codes.push_back("SEARCH_INCOMPLETE");
          note_failure("SEARCH_INCOMPLETE");
        }
        record_stage("fde_decision", true);
        // P0-04 final ledger supersedes the pre-selection diagnostic.  It uses
        // the same total budget and includes the actual eligible-action union.
        output.diagnostics.risk_ledger_charged_total =
            final_risk.charged_total;
        output.diagnostics.risk_ledger_declared_total =
            final_risk.declared_total;
        output.diagnostics.risk_margin = final_risk.margin;
        output.diagnostics.risk_ledger_closes =
            final_risk.complete_bound_closes;
        output.diagnostics.risk_ledger_all_validated =
            final_risk.all_terms_validated;
        output.diagnostics.risk_ledger_validated_terms =
            final_risk.validated_terms;
        output.diagnostics.risk_ledger_unvalidated_terms =
            final_risk.unvalidated_terms;
        output.diagnostics.risk_ledger_not_implemented_terms =
            final_risk.not_implemented_terms;
        output.diagnostics.risk_ledger_terms =
            final_risk.terms;
        // The action-aware ledger is authoritative for every final consumer,
        // including fail-closed early exits where its numeric value is
        // UNKNOWN.  Never retain the pre-selection PL diagnostic as the final
        // complete-risk result.
        output.protection_level.risk_budget_valid =
            final_risk.complete_bound_closes &&
            final_risk.all_terms_validated;
        output.protection_level.formal_eligible =
            final_risk.formal_eligible;
        if (!output.protection_level.risk_budget_valid) {
          output.protection_level.availability = Availability::Unavailable;
        }
        // C4 diagnostics v16: export the charged event-class identity and the
        // derived risk-proof id exactly as the decision produced them.
        output.diagnostics.selection_risk_proof_id =
            decision.selection_risk_proof_id;
        {
          std::string classes;
          for (const auto id : decision.selection_event_class_ids) {
            if (!classes.empty()) classes += ';';
            classes += std::to_string(id);
          }
          output.diagnostics.selection_event_class_ids = classes;
        }
        if (!decision.commit_allowed || !decision.selected_action) {
          note_failure(decision.reason);
        }
        start_operation("candidate_audit");
        for (const auto& candidate : candidates) {
          CandidateAuditRecord* staged = actionSearchOccurrenceFor(
              &output, candidate.action);
          // Production action-search rows are staged before the first
          // fallible candidate call.  On the normal path enrich that same row
          // with the actual result instead of creating a second CSV action
          // identity.  Thus exception and success outputs share one attempt
          // object and the v6 (window, action_id) uniqueness contract remains
          // intact.
          CandidateAuditRecord standalone;
          CandidateAuditRecord& record = staged ? *staged : standalone;
          const std::string terminal = record.diagnostics.skip_reason;
          record.diagnostics = candidate.diagnostics;
          if (staged) record.diagnostics.skip_reason = terminal;
          record.action_id = candidate.action.id.value();
          if (cfg.output.write_candidates) {
            if (!staged) record.action_type = candidate.action.action_model_id;
          for (const auto& source : candidate.action.physical_source_ids) {
            if (!record.physical_source_ids.empty() && !staged) {
              record.physical_source_ids += ';';
            }
            if (!staged) record.physical_source_ids += source;
          }
          auto join_group_ids = [](const std::vector<FactorGroupId>& ids) {
            std::string joined;
            for (const auto id : ids) {
              if (!joined.empty()) joined += ';';
              joined += std::to_string(id.value());
            }
            return joined;
          };
          record.removed_group_ids = join_group_ids(
              candidate.action.groups_to_remove);
          record.added_group_ids = join_group_ids(candidate.action.groups_to_add);
          record.bridge_mode = toString(candidate.action.bridge_mode);
          // C4 diagnostics v16 (§7.3): the removal provenance of this action.
          if (!staged) {
            record.removal_data_source = candidate.action.removal_data_source;
            record.model_error_record = candidate.action.model_error_record;
          }
          record.model_error_validated = staged
              ? (record.model_error_validated &&
                 candidate.action.model_error_validated)
              : candidate.action.model_error_validated;
          }
          record.cardinality = candidate.action.exclusion_cardinality;
          record.valid = candidate.valid;
          record.post_detector_passed = candidate.post_detector_passed;
          record.covers_plausible_set = candidate.covers_plausible_set;
          record.statistic = candidate.statistic;
          record.threshold = candidate.squared_threshold;
          record.rank = candidate.rank;
          record.dof = candidate.dof;
          record.condition_number = candidate.condition_number;
          record.information_logdet = candidate.information_logdet;
          record.risk_allocation = cfg.risk_v2.p_hmi_total /
              std::max<std::size_t>(1, models.actions.size());
          record.hpl_m = candidate.hpl_m;
          record.vpl_m = candidate.vpl_m;
          record.selected = candidate.selected;
          record.wall_ms = candidate.wall_ms;
          // Compact logging may omit descriptive fields, but an invalid
          // candidate must always retain its fail-closed reason so the run is
          // schema-valid and independently auditable.
          if (cfg.output.write_candidates || !candidate.valid) {
            record.reason = candidate.reason;
          }
          if (!record.reason.empty()) record.reason += ";";
          record.reason += "ACTION_OPERATION_IDENTITY=" +
              exactActionOperationIdentityV1(candidate.action);
          if (!staged) {
            output.candidate_audit.push_back(std::move(standalone));
          }
          if (cfg.output.write_candidates) {
            CoverageAuditRecord coverage;
            coverage.action_id = candidate.action.id.value();
            coverage.action_type = candidate.action.action_model_id;
            auto append = [](std::string* target, std::uint64_t value) {
              if (!target->empty()) *target += ';';
              *target += std::to_string(value);
            };
            for (const auto id : decision.plausible_hypotheses) {
              append(&coverage.plausible_hypothesis_ids, id.value());
              const auto hypothesis = indexes.hypotheses.find(id.value());
              if (hypothesis == indexes.hypotheses.end() ||
                  actionCovers(candidate.action, *hypothesis->second)) continue;
              append(&coverage.uncovered_hypothesis_ids, id.value());
              for (const auto mode : hypothesis->second->modes) {
                append(&coverage.uncovered_mode_ids, mode.value());
              }
              for (const auto group : hypothesis->second->affected_groups) {
                append(&coverage.uncovered_group_ids, group.value());
              }
            }
            for (const auto& source : mandatory_health_sources) {
              if (!coverage.mandatory_health_sources.empty()) {
                coverage.mandatory_health_sources += ';';
              }
              coverage.mandatory_health_sources += source;
            }
            for (const auto group : decision.mandatory_exclusion_groups) {
              append(&coverage.mandatory_group_ids, group.value());
              if (std::find(candidate.action.groups_to_remove.begin(),
                            candidate.action.groups_to_remove.end(), group) ==
                  candidate.action.groups_to_remove.end()) {
                append(&coverage.uncovered_mandatory_group_ids, group.value());
              }
            }
            for (const auto mode : candidate.action.covered_modes) {
              append(&coverage.covered_mode_ids, mode.value());
            }
            for (const auto group : candidate.action.groups_to_remove) {
              append(&coverage.removed_group_ids, group.value());
            }
            for (const auto group : candidate.action.groups_to_add) {
              append(&coverage.added_group_ids, group.value());
            }
            if (candidate.action.exclusion_cardinality >
                static_cast<int>(cfg.fde.max_exclusion_cardinality)) {
              coverage.outcome = "CARDINALITY_CONTRACT";
              coverage.reason = "action physical-source cardinality exceeds configured maximum";
            } else if (candidate.action.recoverability !=
                       HistoryRecoverability::Recoverable) {
              coverage.outcome = "UNRECOVERABLE";
              coverage.reason = "action replacement/bridge provenance is incomplete";
            } else if (!coverage.uncovered_hypothesis_ids.empty() ||
                       !coverage.uncovered_mandatory_group_ids.empty()) {
              coverage.outcome = "COVERAGE_GAP";
              coverage.reason = "action does not cover the frozen plausible set and mandatory exclusions";
            } else if (!candidate.valid) {
              coverage.outcome = "NUMERICAL_REJECTION";
              coverage.reason = candidate.reason;
            } else {
              coverage.outcome = candidate.selected ? "SELECTED" : "COVERED";
              coverage.reason = candidate.reason;
            }
            output.coverage_audit.push_back(std::move(coverage));
          }
        }
        record_stage("candidate_audit");
        for (const auto& coverage : output.coverage_audit) {
          if (coverage.outcome == "COVERAGE_GAP" ||
              coverage.outcome == "UNRECOVERABLE" ||
              coverage.outcome == "CARDINALITY_CONTRACT") {
            note_failure(coverage.reason);
          }
        }
        start_operation("finalize_commit_audit");
        if (!decision.commit_allowed || !decision.selected_action) {
          if (all_in.passed &&
              decision.status != FdeStatus::SearchIncomplete) {
            commit_best_effort(decision.status, decision.reason);
          } else {
          DiscardReason reason;
          reason.status = decision.status;
          reason.detail = decision.reason;
          reason.controlled_reinitialization_required =
              decision.status == FdeStatus::BridgeTimeout ||
              decision.status == FdeStatus::HistoryPriorContaminated;
          const DiscardReceipt receipt = timed_discard(
              std::move(transaction), reason);
          health_ = pending_health;
          output.timestamp = receipt.last_committed_timestamp;
          output.state = estimator_->currentState();
          output.backend_updates = receipt.backend_updates;
          output.stale_state = true;
          output.controlled_reinitialization_required =
              receipt.controlled_reinitialization_required;
          output.fde_status = toString(decision.status);
          output.protection_level.availability = Availability::Unavailable;
          output.protection_level.reason = decision.reason;
          if (reason.controlled_reinitialization_required) {
            const auto request = reinitializer_.request(
                decision.status, decision.reason, estimator_->currentState());
            output.reinitialization_request_id = request.value();
            output.reinitialization_phase =
                toString(reinitializer_.directive().state);
            output.reinitialization_reason = decision.reason;
          }
          capture_state_audit();
          }
        } else {
          const ExclusionAction& action = *decision.selected_action;
          EpochCommitPlan plan = EpochCommitPlan::nominalPlan(transaction);
          for (const auto removed : action.groups_to_remove) {
            const auto pending = std::find(plan.groups_to_add.begin(),
                                           plan.groups_to_add.end(), removed);
            if (pending != plan.groups_to_add.end()) {
              plan.groups_to_add.erase(pending);
            } else {
              plan.groups_to_remove.push_back(removed);
            }
          }
          for (const auto added : action.groups_to_add) {
            if (std::find(plan.groups_to_add.begin(), plan.groups_to_add.end(),
                          added) == plan.groups_to_add.end()) {
              plan.groups_to_add.push_back(added);
            }
          }
          plan.action_id = action.id;
          plan.bridge_mode = action.bridge_mode;
          plan.fde_status = decision.status;
          plan.recovery_epoch_begin = action.recovery_epoch_begin;
          plan.recovery_epoch_end = action.recovery_epoch_end;
          auto record_relation = [&](const PendingFactorGroup& group) {
            if (std::find(plan.groups_to_add.begin(), plan.groups_to_add.end(),
                          group.id) == plan.groups_to_add.end()) {
              return;
            }
            if (group.replaces_group) {
              plan.replacement_relations[group.replaces_group->value()] =
                  group.id.value();
            }
            plan.group_health[group.id.value()] = group.health;
          };
          for (const auto& group : transaction.uwb_groups) record_relation(group);
          record_relation(transaction.generic_bridge_group);
          record_relation(transaction.generic_bias_continuity_group);
          for (const auto& history : transaction.recoverable_history) {
            for (const auto& group : history.groups) record_relation(group);
          }
          // Gate J is deliberately not complete, so runtime output remains
          // integrity-unavailable even when the numerical PL is below limits.
          plan.best_effort_integrity_unavailable = true;
          if (cfg.health.quarantine_after_exclusion &&
              action.exclusion_cardinality > 0) {
            for (const auto covered : action.covered_units) {
              const auto unit = std::find_if(models.units.begin(), models.units.end(),
                  [&](const FaultUnit& value) { return value.id == covered; });
              if (unit != models.units.end()) {
                const std::string source = healthSourceId(*unit);
                if (!source.empty() &&
                    pending_health.allowedInFormalEstimator(source)) {
                  (void)pending_health.quarantine(source, action.action_model_id);
                }
              }
            }
          }

          const bool current_bridge_selected = std::find(
              plan.groups_to_add.begin(), plan.groups_to_add.end(),
              transaction.generic_bridge_group.id) != plan.groups_to_add.end();
          auto staged_bridge_start_timestamp = bridge_start_timestamp_;
          std::uint32_t staged_consecutive_bridge_epochs =
              consecutive_bridge_epochs_;
          if (current_bridge_selected) {
            if (!staged_bridge_start_timestamp) {
              staged_bridge_start_timestamp = batch.timestamp;
            }
            ++staged_consecutive_bridge_epochs;
            const double duration = batch.timestamp.seconds() -
                staged_bridge_start_timestamp->seconds();
            BridgeAuditRecord bridge;
            bridge.mode = toString(action.bridge_mode);
            bridge.consecutive_epochs = staged_consecutive_bridge_epochs;
            bridge.duration_s = duration;
            bridge.model_id = "GENERIC_KINEMATIC_CV";
            bridge.dt_s = transaction.end.seconds() - transaction.begin.seconds();
            bridge.integrity_model = "DETERMINISTIC_BOX";
            bridge.calibration_id = cfg.bridge.generic.calibration_id;
            const BridgeUncertainty bridge_uncertainty =
                BridgeFactory().uncertainty(transaction, cfg.bridge.generic);
            bridge.optimization_covariance_diagonal =
                bridge_uncertainty.optimization_covariance.diagonal();
            bridge.bound = bridge_uncertainty.deterministic_bound.head<3>();
            bridge.control_available = false;
            bridge.bias_continuity_maneuver_margin_applied = false;
            bridge.active = true;
            bridge.timeout = false;
            bridge.status = "ACTIVE";
            output.bridge_audit = bridge;
            if (bridgeTimeoutExceeded(
                    staged_consecutive_bridge_epochs, duration,
                    cfg.bridge.max_consecutive_epochs,
                    cfg.bridge.max_duration_s)) {
              (void)pending_health.fail("generic_bridge", "BRIDGE_TIMEOUT");
              DiscardReason reason{FdeStatus::BridgeTimeout, "BRIDGE_TIMEOUT",
                                   true};
              const auto receipt = timed_discard(
                  std::move(transaction), reason);
              health_ = pending_health;
              bridge_start_timestamp_ = staged_bridge_start_timestamp;
              consecutive_bridge_epochs_ = staged_consecutive_bridge_epochs;
              output.timestamp = receipt.last_committed_timestamp;
              output.state = estimator_->currentState();
              output.stale_state = true;
              output.controlled_reinitialization_required = true;
              output.fde_status = toString(FdeStatus::BridgeTimeout);
              output.protection_level.reason =
                  "BRIDGE_TIMEOUT; CONTROLLED_REINITIALIZATION_REQUIRED";
              output.bridge_audit->status = "BRIDGE_TIMEOUT";
              output.bridge_audit->timeout = true;
              output.bridge_audit->active = false;
              const auto request = reinitializer_.request(
                  FdeStatus::BridgeTimeout, "BRIDGE_TIMEOUT",
                  estimator_->currentState());
              output.reinitialization_request_id = request.value();
              output.reinitialization_phase =
                  toString(reinitializer_.directive().state);
              output.reinitialization_reason = "BRIDGE_TIMEOUT";
              note_failure("BRIDGE_TIMEOUT");
              note_not_evaluated("ATOMIC_CERTIFIED_PUBLISH");
              finalize_failure_accounting();
              capture_state_audit();
              return output;
            }
          } else {
            staged_consecutive_bridge_epochs = 0;
            staged_bridge_start_timestamp.reset();
          }
          const bool uwb_committed = std::any_of(
              plan.groups_to_add.begin(), plan.groups_to_add.end(),
              [&](FactorGroupId id) {
                return id != transaction.imu_group.id &&
                       id != transaction.generic_bridge_group.id &&
                       id != transaction.generic_bias_continuity_group.id;
              });
          const auto selected = std::find_if(
              candidates.begin(), candidates.end(),
              [](const CandidateEvaluation& value) { return value.selected; });
          auto selected_pl = candidate_pl.end();
          std::uint64_t selected_pl_proof_identity = 0;
          if (selected != candidates.end()) {
            selected_pl = candidate_pl.find(selected->action.id.value());
            const auto proof = candidate_pl_proof.find(
                selected->action.id.value());
            selected_pl_proof_identity = proof == candidate_pl_proof.end()
                ? 0 : proof->second;
          }
          CommitProtectionEvidenceV1 protection_evidence;
          bool protection_minted = false;
          std::string protection_mint_reason;
          if (selected != candidates.end() && selected_pl != candidate_pl.end() &&
              selected_pl_proof_identity != 0 &&
              selected->state_increment.allFinite() &&
              selected_pl->second.pl_xyz_m.allFinite() &&
              (selected_pl->second.pl_xyz_m.array() >= 0.0).all() &&
              window.protected_state_map.cols() ==
                  selected->state_increment.size()) {
            protection_minted = mintCommitProtectionEvidenceV1(
                transaction, window, *selected, selected_pl->second,
                selected_pl_proof_identity, cfg.realtime.world_frame,
                &protection_evidence, &protection_mint_reason);
          }
          CommitCertificationV1 certification;
          const CommitReceipt receipt = timed_commit(
              std::move(transaction), plan,
              protection_minted ? &protection_evidence : nullptr,
              &certification);
          health_ = pending_health;
          bridge_start_timestamp_ = staged_bridge_start_timestamp;
          consecutive_bridge_epochs_ = staged_consecutive_bridge_epochs;
          output.backend_updates = receipt.backend_updates;
          for (const auto id : receipt.historical_groups_removed) {
            output.historical_groups_removed.push_back(id.value());
          }
          for (const auto id : receipt.historical_groups_added) {
            output.historical_groups_added.push_back(id.value());
          }
          output.recovery_epoch_begin = receipt.recovery_epoch_begin.value_or(0);
          output.recovery_epoch_end = receipt.recovery_epoch_end.value_or(0);
          output.selected_action_id = action.id.value();
          output.selected_action_type = action.action_model_id;
          output.fde_status = toString(decision.status);
          output.state = estimator_->currentState();
          output.batch_committed = uwb_committed;
          if (selected != candidates.end()) {
            output.diagnostics.selected_actions = 1;
            output.diagnostics.selected_step_norm =
                selected->state_increment.norm();
            append_step_audit(selected->state_increment, "SELECTED");
            const auto pl = selected_pl;
            if (pl != candidate_pl.end() && certification.reference_bound) {
              output.protection_level.pl_xyz_m = certification.transferred_pl_m;
              output.protection_level.nominal_component_m =
                  pl->second.nominal_component_m +
                  certification.reference_transfer_m;
              output.protection_level.fault_component_m =
                  pl->second.fault_component_m;
              output.bridge_component_m = pl->second.bridge_component_m;
              output.protection_level.hpl_m = std::hypot(
                  certification.transferred_pl_m.x(),
                  certification.transferred_pl_m.y());
              output.protection_level.vpl_m = certification.transferred_pl_m.z();
              output.protection_level.allocated_hmi_risk =
                  pl->second.allocated_outcome_risk;
              output.protection_level.risk_budget_valid =
                  final_risk.complete_bound_closes &&
                  final_risk.all_terms_validated;
              std::string pl_proof_reason;
              std::string publication_packet;
              const bool packet_bound = selected_pl_proof_identity != 0 &&
                  bindTransferredProtectionLevelPublicationPacket(
                      *selected, pl->second, selected_pl_proof_identity,
                      certification.protected_reference.mean_world_m,
                      certification.committed_mean_world_m,
                      certification.transferred_pl_m,
                      receipt.state_timestamp.value(),
                      certification.protected_reference.frame_id,
                      certification.protected_reference.position_reference,
                      &publication_packet);
              if (!packet_bound ||
                  !validateProtectionLevelPublicationProof(
                      selected_pl_proof_identity,
                      certification.transferred_pl_m,
                      publication_packet,
                      &pl_proof_reason)) {
                output.protection_level.detector_certificate_id.clear();
                output.protection_level.risk_budget_valid = false;
                output.protection_level.reason =
                    "P0-03 PL proof rejected during P0-02 conversion: " +
                    pl_proof_reason;
              } else {
                // Existing ABI string slot carries a versioned packet whose
                // payload explicitly binds the selected candidate identity.
                output.protection_level.detector_certificate_id =
                    publication_packet;
              }
            } else {
              output.protection_level.pl_xyz_m.setConstant(
                  std::numeric_limits<double>::infinity());
              output.protection_level.hpl_m =
                  std::numeric_limits<double>::infinity();
              output.protection_level.vpl_m =
                  std::numeric_limits<double>::infinity();
              output.protection_level.risk_budget_valid = false;
              output.protection_level.detector_certificate_id.clear();
              output.protection_level.reason =
                  "committed nonlinear mean has no bound PL reference" +
                  (protection_mint_reason.empty()
                       ? std::string()
                       : ": " + protection_mint_reason);
            }
          }
          output.protection_level.availability = Availability::Unavailable;
          output.protection_level.formal_eligible =
              final_risk.formal_eligible;
          output.protection_level.hmi_risk_requirement = cfg.risk_v2.p_hmi_total;
          output.protection_level.reason =
              "IMPLEMENTED_UNVERIFIED: Gate J calibration and independent review pending";
          if (awaiting_first_clean_uwb_) {
            output.protection_level.availability = Availability::Unavailable;
            output.protection_level.reason =
                "IMPLEMENTED_UNVERIFIED: first clean UWB committed after reinitialization";
            awaiting_first_clean_uwb_ = false;
          }
          capture_state_audit();
        }
      }
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
    record_stage("finalize_commit_audit");
    double nested_ms = 0.0;
    for (const auto& stage : output.stage_timings)
      if (stage.stage == "commit" || stage.stage == "discard" || stage.stage == "state_audit")
        nested_ms += stage.wall_ms;
    output.stage_timings.back().wall_ms = std::max(0.0, output.stage_timings.back().wall_ms - nested_ms);
    output.stage_timings.back().reason = "excludes separately timed commit/discard/state audit";
    output.global_detector.reason = estimator_->globalDiagnosticsEnabled()
        ? "independent empirical calibration required"
        : "disabled on formal/performance path";
    output.diagnostics.state_age_s = batch.timestamp.seconds() -
        output.timestamp.seconds();
    const NumericalWorkSnapshot numerical_after =
        NumericalWorkCounters::snapshot();
    output.diagnostics.base_svd = numerical_after.base_svd - numerical_before.base_svd;
    output.diagnostics.base_llt = numerical_after.base_llt - numerical_before.base_llt;
    output.diagnostics.base_state_solves =
        numerical_after.base_state_solves - numerical_before.base_state_solves;
    output.diagnostics.llt_state_solve_calls =
        numerical_after.llt_state_solve_calls - numerical_before.llt_state_solve_calls;
    output.diagnostics.svd_state_solve_calls =
        numerical_after.svd_state_solve_calls - numerical_before.svd_state_solve_calls;
    output.diagnostics.detector_reference_qr =
        numerical_after.detector_reference_qr - numerical_before.detector_reference_qr;
    output.diagnostics.candidate_reference_svd =
        numerical_after.candidate_reference_svd - numerical_before.candidate_reference_svd;
    output.diagnostics.candidate_inner_llt =
        numerical_after.candidate_inner_llt - numerical_before.candidate_inner_llt;
    output.diagnostics.fault_gram_eigen =
        numerical_after.fault_gram_eigen - numerical_before.fault_gram_eigen;
    output.diagnostics.fault_gram_svd =
        numerical_after.fault_gram_svd - numerical_before.fault_gram_svd;
    output.diagnostics.fault_gram_ldlt =
        numerical_after.fault_gram_ldlt - numerical_before.fault_gram_ldlt;
    output.diagnostics.low_dim_fault_gram =
        numerical_after.low_dim_fault_gram - numerical_before.low_dim_fault_gram;
    output.diagnostics.generic_fault_gram_fallback =
        numerical_after.generic_fault_gram_fallback -
        numerical_before.generic_fault_gram_fallback;
    output.diagnostics.hypothesis_parallel_blocks =
        numerical_after.hypothesis_parallel_blocks -
        numerical_before.hypothesis_parallel_blocks;
    output.diagnostics.hypothesis_shared_hits =
        numerical_after.hypothesis_shared_hits -
        numerical_before.hypothesis_shared_hits;
    output.diagnostics.hypothesis_shared_misses =
        numerical_after.hypothesis_shared_misses -
        numerical_before.hypothesis_shared_misses;
    output.diagnostics.covariance_rhs_solves =
        numerical_after.covariance_rhs_solves - numerical_before.covariance_rhs_solves;
    output.diagnostics.covariance_rhs_columns =
        numerical_after.covariance_rhs_columns - numerical_before.covariance_rhs_columns;
    output.diagnostics.spectral_rhs_solves =
        numerical_after.spectral_rhs_solves - numerical_before.spectral_rhs_solves;
    output.diagnostics.spectral_rhs_columns =
        numerical_after.spectral_rhs_columns - numerical_before.spectral_rhs_columns;
    output.diagnostics.numerical_contract_mismatches =
        numerical_after.numerical_contract_mismatches -
        numerical_before.numerical_contract_mismatches;
    output.diagnostics.imu_oracle_reintegrations =
        numerical_after.imu_oracle_reintegrations - numerical_before.imu_oracle_reintegrations;
    finalize_failure_accounting();
    return output;
  } catch (...) {
    refreshActionSearchAudit(&output);
    auto existing = std::find_if(output.stage_timings.begin(), output.stage_timings.end(),
        [&](const StageTiming& timing) { return timing.stage == executing_stage; });
    if (existing == output.stage_timings.end()) {
      output.stage_timings.push_back({executing_stage, 0.0, false, "EXCEPTION", "processing exception"});
      existing = std::prev(output.stage_timings.end());
    }
    existing->wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - operation_start).count();
    existing->status = "EXCEPTION"; existing->success = false;
    existing->reason = "processing exception; see attempt reason";
    last_attempt_output_ = std::move(output);
    if (estimator_->hasPendingEpoch()) {
      DiscardReason reason;
      reason.status = FdeStatus::ModelInvalid;
      reason.detail = "V2 processing exception";
      (void)estimator_->discardEpoch(std::move(transaction), reason);
    }
    throw;
  }
}

}  // namespace uwb_imu_pl
