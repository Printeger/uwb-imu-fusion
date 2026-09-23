#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"
#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {

namespace {

// B3 (§5.9): the bound used by every PL path.
//
//   L_{h,d} = s_{h,d} * sqrt(Lambda_h) + k_{h,d} * sigma_d
//   alpha_h   = min(0.5, allocation_h / pi_h)          (charged tail)
//   alpha_{h,d} = alpha_h / axis_count                 (equal split)
//   k_{h,d}   = Phi^-1(1 - alpha_{h,d} / 2)
//   Lambda_h  solves F_{chi2_nu(Lambda_h)}(tau) <= beta_h, conservative side
//
// The per-axis split is what makes the charged risk alpha_h sufficient for the
// union bound over the three axes: using alpha_h on every axis (the pre-B3
// form) understates the tail that the ledger pays for.  Invalid inputs are
// rejected; the caller turns that into an unavailable result instead of a
// clamped bound.
struct FaultAxisBounds {
  double lambda = 0.0;
  double k = 0.0;
  double hypothesis_tail = 0.0;
  double axis_tail = 0.0;
  bool valid = false;
  std::string reason;
};

bool detectorContractMatchesCandidate(const LinearizedIntegrityWindow& window,
                                      const CandidateEvaluation& candidate,
                                      const DetectorResultV2& detector,
                                      std::string* reason) {
  if (detector.contract_mode == DetectorContractMode::LegacyPooledOffline) {
    return true;
  }
  std::string candidate_reason;
  if (!validateCandidateDetectorCertificate(window, candidate,
                                            &candidate_reason)) {
    if (reason) *reason = candidate_reason;
    return false;
  }
  const auto& certificate = candidate.detector_certificate;
  const bool p_fa_valid = std::isfinite(detector.p_fa_per_test) &&
      detector.p_fa_per_test > 0.0 && detector.p_fa_per_test < 1.0;
  const int expected_channel_count = certificate.history_rows > 0 ? 2 : 1;
  const std::uint64_t expected_horizon =
      std::max<std::uint64_t>(1, detector.continuity_horizon_tests);
  const double expected_current_threshold = p_fa_valid
      ? StatisticalBoundsCache::chiSquaredThreshold(
            certificate.current_dof, detector.p_fa_per_test)
      : std::numeric_limits<double>::infinity();
  const double expected_history_threshold = certificate.history_rows > 0 &&
      p_fa_valid
      ? StatisticalBoundsCache::chiSquaredThreshold(
            certificate.history_dof, detector.p_fa_per_test)
      : 0.0;
  const double expected_pooled_threshold = p_fa_valid
      ? StatisticalBoundsCache::chiSquaredThreshold(
            certificate.pooled_dof, detector.p_fa_per_test)
      : std::numeric_limits<double>::infinity();
  const double expected_operation_bound = p_fa_valid
      ? std::min(1.0, detector.p_fa_per_test *
            static_cast<double>(expected_horizon) *
            static_cast<double>(expected_channel_count))
      : std::numeric_limits<double>::infinity();
  const bool expected_current_accepted = p_fa_valid &&
      certificate.current_statistic <= expected_current_threshold;
  const bool expected_history_accepted = certificate.history_rows == 0 ||
      (p_fa_valid &&
       certificate.history_statistic <= expected_history_threshold);
  const bool valid = certificate.valid &&
      certificate.history_constant_present && detector.channel_split_valid &&
      p_fa_valid && detector.window_id == window.id &&
      detector.accepted_event_id == certificate.accepted_event_id &&
      detector.accepted_event_id == "dual_channel_intersection_v6" &&
      detector.candidate_numerical_identity == certificate.numerical_identity &&
      detector.candidate_numerical_identity != 0 &&
      detector.candidate_certificate_digest == certificate.certificate_digest &&
      detector.detector_contract_digest != 0 &&
      detector.detector_contract_digest == detectorContractDigest(detector) &&
      detector.rows == candidate.rows && detector.rank == candidate.rank &&
      detector.dof == candidate.dof &&
      detector.squared_parity_statistic ==
          certificate.current_statistic + certificate.history_statistic &&
      detector.squared_threshold == expected_pooled_threshold &&
      std::isfinite(detector.history_constant) &&
      detector.history_constant == certificate.history_constant &&
      detector.channel_current_statistic == certificate.current_statistic &&
      detector.channel_history_statistic == certificate.history_statistic &&
      detector.channel_current_dof == certificate.current_dof &&
      detector.channel_history_dof == certificate.history_dof &&
      detector.channel_current_threshold == expected_current_threshold &&
      detector.channel_history_threshold == expected_history_threshold &&
      detector.continuity_horizon_tests == expected_horizon &&
      detector.active_channel_count == expected_channel_count &&
      detector.operation_p_fa_upper_bound == expected_operation_bound &&
      detector.channel_current_accepted == expected_current_accepted &&
      detector.channel_history_accepted == expected_history_accepted &&
      detector.channel_current_accepted && detector.channel_history_accepted &&
      detector.joint_accepted && detector.passed;
  if (!valid && reason) {
    *reason = "active-v6 detector certificate/event/numerical identity mismatch";
  }
  return valid;
}

Eigen::MatrixXd historyGramFromCertificate(
    const CandidateEvaluation& candidate, const Eigen::MatrixXd& fault_map) {
  Eigen::MatrixXd history_gram = Eigen::MatrixXd::Zero(
      fault_map.cols(), fault_map.cols());
  const auto& roles = candidate.detector_certificate.row_roles;
  if (!candidate.detector_certificate.valid ||
      roles.size() != static_cast<std::size_t>(fault_map.rows())) {
    return Eigen::MatrixXd();
  }
  for (Eigen::Index row = 0; row < fault_map.rows(); ++row) {
    if (roles[static_cast<std::size_t>(row)] ==
        CandidateDetectorRowRole::HistoryDetectorOnly) {
      history_gram.noalias() += fault_map.row(row).transpose() *
                                fault_map.row(row);
    }
  }
  return history_gram;
}

FaultAxisBounds dualChannelAxisBounds(
    const DetectorResultV2& detector,
    const FaultHypothesisV2& hypothesis,
    const Eigen::MatrixXd& total_gram,
    const Eigen::MatrixXd& history_gram,
    const Eigen::MatrixXd& protected_response,
    const Eigen::Vector3d& sigma,
    double nominal_tail,
    Eigen::Vector3d* component,
    std::string* certificate_id) {
  FaultAxisBounds out;
  double tail = hypothesis.hmi_allocation > 0.0
      ? hypothesis.hmi_allocation /
            std::max(hypothesis.prior_probability_bound, 1e-15)
      : nominal_tail;
  // The production path is required to carry an explicit dual-channel
  // certificate.  The older pooled numerical API remains available only when
  // an offline caller explicitly selects LegacyPooledOffline; an incomplete
  // active-v6 detector always fails closed.
  const bool legacy_single_channel =
      detector.contract_mode == DetectorContractMode::LegacyPooledOffline;
  if (!legacy_single_channel &&
      (detector.accepted_event_id != "dual_channel_intersection_v6" ||
       !detector.channel_split_valid ||
       detector.candidate_numerical_identity == 0 ||
       !std::isfinite(detector.history_constant))) {
    out.reason = "active-v6 dual-channel detector certificate is incomplete";
    return out;
  }
  if (!std::isfinite(tail) || tail <= 0.0) {
    out.reason = "hypothesis tail probability is not positive";
    return out;
  }
  tail = std::min(0.5, tail);
  out.hypothesis_tail = tail;
  out.axis_tail = tail / 3.0;
  bool multiplier_valid = false;
  out.k = StatisticalBoundsCache::normalTwoSidedMultiplierVerified(
      out.axis_tail, &multiplier_valid);
  if (!multiplier_valid) {
    out.reason = "per-axis tail multiplier could not be evaluated";
    return out;
  }
  DualChannelBoundRequest request;
  request.protected_response = protected_response;
  request.p_md = hypothesis.p_md_allocation;
  request.k_axis.setConstant(out.k);
  request.position_rho_m.setZero();
  request.position_rho_validated = true;
  request.position_rho_source = "none";

  ChannelBoundInput current;
  current.detector_id = "joint_window_state_supported";
  current.dof = legacy_single_channel ? detector.dof
                                      : detector.channel_current_dof;
  current.threshold = legacy_single_channel ? detector.squared_threshold
                                            : detector.channel_current_threshold;
  current.actual_threshold = current.threshold;
  current.weight = !legacy_single_channel && detector.channel_history_dof > 0
                       ? 0.5
                       : 1.0;
  const Eigen::MatrixXd current_gram = legacy_single_channel
      ? total_gram : (total_gram - history_gram);
  current.gram = 0.5 * (current_gram + current_gram.transpose());
  current.rho_validated = true;
  current.rho_source = "none";
  request.channels.push_back(std::move(current));
  if (!legacy_single_channel && detector.channel_history_dof > 0 &&
      history_gram.norm() > 0.0) {
    ChannelBoundInput history;
    history.detector_id = "history_eliminated_detector_only";
    history.dof = detector.channel_history_dof;
    history.threshold = detector.channel_history_threshold;
    history.actual_threshold = detector.channel_history_threshold;
    history.weight = 0.5;
    history.gram = 0.5 * (history_gram + history_gram.transpose());
    history.rho_validated = true;
    history.rho_source = "none";
    request.channels.push_back(std::move(history));
  }
  const DualChannelBoundResult bound = computeDualChannelBound(request);
  if (!bound.valid) {
    out.reason = "dual-channel PL bound: " + bound.reason;
    return out;
  }
  *component = bound.axis_bound_m + out.k * sigma +
               bound.position_rho_m;
  *certificate_id = bound.certificate_id;
  for (const double value : bound.channel_lambda) {
    out.lambda = std::max(out.lambda, value);
  }
  out.valid = component->allFinite();
  if (!out.valid) out.reason = "dual-channel PL component is non-finite";
  return out;
}

FaultAxisBounds faultAxisBounds(const DetectorResultV2& detector,
                                const FaultHypothesisV2& hypothesis,
                                double nominal_tail) {
  FaultAxisBounds out;
  double tail = hypothesis.hmi_allocation > 0.0
      ? hypothesis.hmi_allocation /
            std::max(hypothesis.prior_probability_bound, 1e-15)
      : nominal_tail;
  if (!std::isfinite(tail) || tail <= 0.0) {
    out.reason = "hypothesis tail probability is not positive";
    return out;
  }
  tail = std::min(0.5, tail);
  out.hypothesis_tail = tail;
  out.axis_tail = tail / 3.0;
  bool multiplier_valid = false;
  out.k = StatisticalBoundsCache::normalTwoSidedMultiplierVerified(
      out.axis_tail, &multiplier_valid);
  if (!multiplier_valid) {
    out.reason = "per-axis tail multiplier could not be evaluated";
    return out;
  }
  const StatisticalBoundKey key;
  const NoncentralityBoundaryResult boundary =
      StatisticalBoundsCache::noncentralityBoundaryVerified(
          detector.dof, detector.squared_threshold,
          hypothesis.p_md_allocation, key);
  if (!boundary.valid) {
    out.reason = "detection boundary noncentrality: " + boundary.reason;
    return out;
  }
  out.lambda = std::sqrt(boundary.value);
  out.valid = true;
  return out;
}

}  // namespace

double ProtectionLevelV2::detectionBoundaryNoncentralitySquared(
    int dof, double threshold, double p_md) {
  return StatisticalBoundsCache::noncentralityBoundary(
      dof, threshold, p_md);
}

ProtectionLevelV2Result ProtectionLevelV2::compute(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate,
    const DetectorResultV2& detector,
    std::vector<FaultHypothesisV2>* hypotheses,
    const RiskBudgetV2& risk,
    const Eigen::Vector3d& bridge_margin) const {
  return computeStreaming(
      window, candidate, detector, hypotheses,
      [](const FaultHypothesisV2& hypothesis) { return hypothesis.A; },
      risk, bridge_margin);
}

ProtectionLevelV2Result ProtectionLevelV2::computeStreaming(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate,
    const DetectorResultV2& detector,
    std::vector<FaultHypothesisV2>* hypotheses,
    const FaultMapProvider& fault_map_provider,
    const RiskBudgetV2& risk,
    const Eigen::Vector3d& bridge_margin) const {
  ProtectionLevelV2Result result;
  result.bridge_component_m = bridge_margin.cwiseAbs();
  std::string detector_contract_reason;
  if (!hypotheses || !candidate.valid || !detector.numerically_valid ||
      !detectorContractMatchesCandidate(window, candidate, detector,
                                        &detector_contract_reason) ||
      window.protected_state_map.cols() != candidate.state_increment.rows() ||
      window.protected_state_map.rows() != 3) {
    result.reason = detector_contract_reason.empty()
        ? "invalid post-FDE candidate/protected-state map"
        : detector_contract_reason;
    return result;
  }
  const RiskBudgetAudit risk_audit = auditRiskBudget(risk, *hypotheses);
  result.allocated_outcome_risk = risk_audit.total;
  result.risk_budget_valid = risk_audit.valid;
  if (!result.risk_budget_valid) {
    result.reason = "outcome-conditioned risk budget does not close";
    return result;
  }
  std::vector<Eigen::MatrixXd> fault_maps;
  std::vector<Eigen::MatrixXd> crosses;
  fault_maps.reserve(hypotheses->size());
  crosses.reserve(hypotheses->size());
  int solve_columns = 3;
  for (auto& hypothesis : *hypotheses) {
    Eigen::MatrixXd fault_map = fault_map_provider(hypothesis);
    if (fault_map.rows() != candidate.rows || fault_map.cols() == 0 ||
        !fault_map.allFinite()) {
      hypothesis.monitored = false;
      hypothesis.monitorability.reason = "post-FDE fault map row mismatch";
      result.reason = "remaining hypothesis cannot be mapped post-FDE";
      return result;
    }
    Eigen::MatrixXd cross = candidate.normalCross(fault_map);
    if (cross.rows() != candidate.state_increment.rows()) {
      result.reason = "candidate normal cross-product failed";
      return result;
    }
    solve_columns += cross.cols();
    fault_maps.push_back(std::move(fault_map));
    crosses.push_back(std::move(cross));
  }
  Eigen::MatrixXd solve_rhs(candidate.state_increment.rows(), solve_columns);
  solve_rhs.leftCols<3>() = window.protected_state_map.transpose();
  int solve_column = 3;
  for (const auto& cross : crosses) {
    solve_rhs.middleCols(solve_column, cross.cols()) = cross;
    solve_column += cross.cols();
  }
  const Eigen::MatrixXd covariance_solutions = candidate.covarianceTimes(solve_rhs);
  if (covariance_solutions.rows() != candidate.state_increment.rows() ||
      covariance_solutions.cols() != solve_columns) {
    result.reason = "candidate covariance solve failed";
    return result;
  }
  const Eigen::MatrixXd covariance_protected_transpose =
      covariance_solutions.leftCols<3>();
  const Eigen::Matrix3d protected_covariance = window.protected_state_map *
      covariance_protected_transpose;
  if (!protected_covariance.allFinite() ||
      (protected_covariance.diagonal().array() <= 0.0).any()) {
    result.reason = "post-FDE protected covariance is invalid";
    return result;
  }
  const double nominal_tail = std::max(1e-15, risk.nominal_axis_tail);
  const double nominal_k = StatisticalBoundsCache::normalTwoSidedMultiplier(
      nominal_tail);
  const Eigen::Vector3d sigma =
      protected_covariance.diagonal().cwiseSqrt();
  result.nominal_component_m = nominal_k * sigma;
  result.fault_component_m.setZero();
  solve_column = 3;
  for (std::size_t index = 0; index < hypotheses->size(); ++index) {
    auto& hypothesis = (*hypotheses)[index];
    const Eigen::MatrixXd& fault_map = fault_maps[index];
    const Eigen::MatrixXd& cross = crosses[index];
    const Eigen::MatrixXd covariance_cross = covariance_solutions.middleCols(
        solve_column, cross.cols());
    solve_column += cross.cols();
    const Eigen::MatrixXd gram = fault_map.transpose() * fault_map -
        cross.transpose() * covariance_cross;
    NumericalWorkCounters::faultGramSvd();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(gram);
    const auto singular = svd.singularValues();
    const double largest = singular.size() ? singular(0) : 0.0;
    const double gate = 1e-10 * std::max(1.0, largest);
    hypothesis.monitorability.parameter_dimension = fault_map.cols();
    hypothesis.monitorability.rank =
        static_cast<int>((singular.array() > gate).count());
    const double smallest = hypothesis.monitorability.rank > 0
        ? singular(hypothesis.monitorability.rank - 1) : 0.0;
    hypothesis.monitorability.sigma_min = std::sqrt(std::max(0.0, smallest));
    hypothesis.monitorability.sigma_max = std::sqrt(std::max(0.0, largest));
    hypothesis.monitorability.condition_number = smallest > 0.0
        ? largest / smallest : std::numeric_limits<double>::infinity();
    hypothesis.monitorability.monitorable =
        hypothesis.monitorability.rank == fault_map.cols();
    hypothesis.monitored = hypothesis.monitorability.monitorable;
    if (!hypothesis.monitored) {
      result.reason = "remaining post-FDE fault is unmonitorable";
      return result;
    }
    NumericalWorkCounters::faultGramLdlt();
    Eigen::LDLT<Eigen::MatrixXd> gram_solve(gram);
    if (gram_solve.info() != Eigen::Success || !gram_solve.isPositive()) {
      result.reason = "remaining fault Gram matrix is not SPD";
      return result;
    }
    const Eigen::MatrixXd protected_fault =
        window.protected_state_map * covariance_cross;
    Eigen::Vector3d slopes;
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::VectorXd response = protected_fault.row(axis).transpose();
      slopes(axis) = std::sqrt(std::max(
          0.0, response.dot(gram_solve.solve(response))));
    }
    hypothesis.monitorability.protected_slopes = slopes;
    Eigen::MatrixXd history_gram = historyGramFromCertificate(candidate,
                                                               fault_map);
    if (history_gram.rows() != fault_map.cols()) {
      result.reason = "candidate detector row-role payload is missing";
      return result;
    }
    Eigen::Vector3d component;
    std::string detector_certificate;
    const FaultAxisBounds bounds = dualChannelAxisBounds(
        detector, hypothesis, gram, history_gram, protected_fault, sigma,
        nominal_tail, &component, &detector_certificate);
    if (!bounds.valid) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " has no valid section 5.9 bound: " + bounds.reason;
      return result;
    }
    result.hypothesis_tail_used = std::max(result.hypothesis_tail_used,
                                           bounds.hypothesis_tail);
    result.axis_tail_used = std::max(result.axis_tail_used, bounds.axis_tail);
    result.fault_multiplier_used = std::max(result.fault_multiplier_used,
                                            bounds.k);
    result.noncentrality_used = std::max(result.noncentrality_used,
                                         bounds.lambda);
    result.detector_certificate_id = detector_certificate;
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

ProtectionLevelV2Result ProtectionLevelV2::computeShared(
    const LinearizedIntegrityWindow& window,
    CandidateEvaluation* candidate,
    const DetectorResultV2& detector,
    std::vector<FaultHypothesisV2>* hypotheses,
    const ProtectionLevelSharedContext& shared,
    const RiskBudgetV2& risk) const {
  ProtectionLevelV2Result result;
  std::string detector_contract_reason;
  if (!candidate || !hypotheses || !candidate->valid ||
      !detector.numerically_valid ||
      !detectorContractMatchesCandidate(window, *candidate, detector,
                                        &detector_contract_reason) ||
      window.protected_state_map.cols() != candidate->state_increment.rows() ||
      window.protected_state_map.rows() != 3) {
    result.reason = detector_contract_reason.empty()
        ? "invalid post-FDE candidate/protected-state map"
        : detector_contract_reason;
    return result;
  }
  std::map<std::uint64_t, const Eigen::MatrixXd*> modes;
  for (const auto& hypothesis : *hypotheses) {
    for (const auto id : hypothesis.modes) {
      const auto found = shared.mode_maps.find(id.value());
      if (found == shared.mode_maps.end() || found->second.rows() != candidate->rows ||
          found->second.cols() == 0 || !found->second.allFinite()) {
        result.reason = "remaining hypothesis cannot be mapped post-FDE";
        return result;
      }
      modes.emplace(id.value(), &found->second);
    }
  }
  const RiskBudgetAudit risk_audit = auditRiskBudget(risk, *hypotheses);
  result.allocated_outcome_risk = risk_audit.total;
  result.risk_budget_valid = risk_audit.valid;
  if (!result.risk_budget_valid) {
    result.reason = "outcome-conditioned risk budget does not close";
    return result;
  }
  struct SolvedMode {
    const Eigen::MatrixXd* map = nullptr;
    Eigen::MatrixXd cross;
    Eigen::Index solve_offset = 0;
  };
  std::map<std::uint64_t, SolvedMode> solved_modes;
  Eigen::Index solve_columns = 3;
  for (const auto& item : modes) {
    SolvedMode mode;
    mode.map = item.second;
    mode.cross = candidate->normalCross(*item.second);
    if (mode.cross.rows() != candidate->state_increment.rows()) {
      result.reason = "candidate normal cross-product failed";
      return result;
    }
    mode.solve_offset = solve_columns;
    solve_columns += mode.cross.cols();
    solved_modes.emplace(item.first, std::move(mode));
  }
  std::vector<Eigen::Index> bridge_offsets;
  bridge_offsets.reserve(shared.bridges.size());
  for (const auto& bridge : shared.bridges) {
    if (bridge.jacobian_whitened.cols() != candidate->state_increment.rows() ||
        bridge.jacobian_whitened.rows() != bridge.deterministic_bound.size() ||
        !bridge.jacobian_whitened.allFinite() ||
        !bridge.deterministic_bound.allFinite() ||
        (bridge.deterministic_bound.array() < 0.0).any()) {
      result.reason = "bridge projection dimensions invalid";
      return result;
    }
    bridge_offsets.push_back(solve_columns);
    solve_columns += bridge.jacobian_whitened.rows();
  }
  Eigen::MatrixXd rhs(candidate->state_increment.rows(), solve_columns);
  rhs.leftCols<3>() = window.protected_state_map.transpose();
  for (const auto& item : solved_modes) {
    rhs.middleCols(item.second.solve_offset, item.second.cross.cols()) =
        item.second.cross;
  }
  for (std::size_t i = 0; i < shared.bridges.size(); ++i) {
    const auto& bridge = shared.bridges[i];
    rhs.middleCols(bridge_offsets[i], bridge.jacobian_whitened.rows()) =
        bridge.jacobian_whitened.transpose();
  }
  const Eigen::MatrixXd solutions = candidate->covarianceTimes(rhs);
  ++candidate->diagnostics.covariance_solve_count;
  if (solutions.rows() != candidate->state_increment.rows() ||
      solutions.cols() != solve_columns || !solutions.allFinite()) {
    result.reason = "candidate covariance solve failed";
    return result;
  }
  const Eigen::MatrixXd protected_solve = solutions.leftCols<3>();
  const Eigen::Matrix3d protected_covariance =
      window.protected_state_map * protected_solve;
  if (!protected_covariance.allFinite() ||
      (protected_covariance.diagonal().array() <= 0.0).any()) {
    result.reason = "post-FDE protected covariance is invalid";
    return result;
  }
  result.bridge_component_m.setZero();
  for (std::size_t i = 0; i < shared.bridges.size(); ++i) {
    const auto& bridge = shared.bridges[i];
    const Eigen::MatrixXd gain = solutions.middleCols(
        bridge_offsets[i], bridge.jacobian_whitened.rows());
    result.bridge_component_m +=
        (window.protected_state_map * gain).cwiseAbs() *
        bridge.deterministic_bound;
  }
  const double nominal_tail = std::max(1e-15, risk.nominal_axis_tail);
  const double nominal_k = StatisticalBoundsCache::normalTwoSidedMultiplier(
      nominal_tail);
  const Eigen::Vector3d sigma = protected_covariance.diagonal().cwiseSqrt();
  result.nominal_component_m = nominal_k * sigma;
  result.fault_component_m.setZero();
  std::map<std::pair<std::uint64_t, std::uint64_t>, Eigen::MatrixXd>
      pair_grams;
  for (auto& hypothesis : *hypotheses) {
    Eigen::Index columns = 0;
    for (const auto id : hypothesis.modes)
      columns += solved_modes.at(id.value()).map->cols();
    Eigen::MatrixXd gram = Eigen::MatrixXd::Zero(columns, columns);
    Eigen::MatrixXd protected_fault(3, columns);
    Eigen::MatrixXd fault_map(candidate->rows, columns);
    Eigen::Index offset = 0;
    for (const auto left_id : hypothesis.modes) {
      const auto& left = solved_modes.at(left_id.value());
      const Eigen::Index left_count = left.map->cols();
      protected_fault.middleCols(offset, left_count) =
          window.protected_state_map * solutions.middleCols(
              left.solve_offset, left_count);
      fault_map.middleCols(offset, left_count) = *left.map;
      Eigen::Index right_offset = 0;
      for (const auto right_id : hypothesis.modes) {
        const auto& right = solved_modes.at(right_id.value());
        const Eigen::Index right_count = right.map->cols();
        const std::pair<std::uint64_t, std::uint64_t> key{
            std::min(left_id.value(), right_id.value()),
            std::max(left_id.value(), right_id.value())};
        auto found = pair_grams.find(key);
        if (found == pair_grams.end()) {
          const auto& first = solved_modes.at(key.first);
          const auto& second = solved_modes.at(key.second);
          Eigen::MatrixXd value = first.map->transpose() * (*second.map) -
              first.cross.transpose() * solutions.middleCols(
                  second.solve_offset, second.map->cols());
          found = pair_grams.emplace(key, std::move(value)).first;
        }
        gram.block(offset, right_offset, left_count, right_count) =
            left_id.value() <= right_id.value()
                ? found->second : found->second.transpose();
        right_offset += right_count;
      }
      offset += left_count;
    }
    gram = 0.5 * (gram + gram.transpose());
    NumericalWorkCounters::faultGramSvd();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(gram);
    const auto singular = svd.singularValues();
    const double largest = singular.size() ? singular(0) : 0.0;
    const double gate = 1e-10 * std::max(1.0, largest);
    auto& monitor = hypothesis.monitorability;
    monitor.parameter_dimension = columns;
    monitor.rank = static_cast<int>((singular.array() > gate).count());
    const double smallest = monitor.rank > 0 ? singular(monitor.rank - 1) : 0.0;
    monitor.sigma_min = std::sqrt(std::max(0.0, smallest));
    monitor.sigma_max = std::sqrt(std::max(0.0, largest));
    monitor.condition_number = smallest > 0.0 ? largest / smallest
        : std::numeric_limits<double>::infinity();
    monitor.monitorable = monitor.rank == columns;
    hypothesis.monitored = monitor.monitorable;
    if (!hypothesis.monitored) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " is unmonitorable: rank=" + std::to_string(monitor.rank) +
          "/" + std::to_string(columns) + ";sigma_min=" +
          std::to_string(monitor.sigma_min) + ";condition=" +
          std::to_string(monitor.condition_number);
      return result;
    }
    NumericalWorkCounters::faultGramLdlt();
    Eigen::LDLT<Eigen::MatrixXd> gram_solve(gram);
    if (gram_solve.info() != Eigen::Success || !gram_solve.isPositive()) {
      result.reason = "remaining fault Gram matrix is not SPD";
      return result;
    }
    Eigen::Vector3d slopes;
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::VectorXd response = protected_fault.row(axis).transpose();
      slopes(axis) = std::sqrt(std::max(
          0.0, response.dot(gram_solve.solve(response))));
    }
    monitor.protected_slopes = slopes;
    Eigen::MatrixXd history_gram = historyGramFromCertificate(*candidate,
                                                               fault_map);
    if (history_gram.rows() != columns) {
      result.reason = "candidate detector row-role payload is missing";
      return result;
    }
    Eigen::Vector3d component;
    std::string detector_certificate;
    const FaultAxisBounds bounds = dualChannelAxisBounds(
        detector, hypothesis, gram, history_gram, protected_fault, sigma,
        nominal_tail, &component, &detector_certificate);
    if (!bounds.valid) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " has no valid section 5.9 bound: " + bounds.reason;
      return result;
    }
    result.hypothesis_tail_used = std::max(result.hypothesis_tail_used,
                                           bounds.hypothesis_tail);
    result.axis_tail_used = std::max(result.axis_tail_used, bounds.axis_tail);
    result.fault_multiplier_used = std::max(result.fault_multiplier_used,
                                            bounds.k);
    result.noncentrality_used = std::max(result.noncentrality_used,
                                         bounds.lambda);
    result.detector_certificate_id = detector_certificate;
    result.fault_component_m = result.fault_component_m.cwiseMax(component);
  }
  result.pl_xyz_m = result.nominal_component_m.cwiseMax(
      result.fault_component_m) + result.bridge_component_m;
  result.hpl_m = std::hypot(result.pl_xyz_m.x(), result.pl_xyz_m.y());
  result.vpl_m = result.pl_xyz_m.z();
  result.model_valid = result.pl_xyz_m.allFinite() && detector.passed;
  result.availability = result.model_valid &&
      result.hpl_m <= risk.horizontal_alert_limit_m &&
      result.vpl_m <= risk.vertical_alert_limit_m
      ? Availability::Available : Availability::Unavailable;
  result.formal_eligible = false;
  if (!detector.passed) result.reason = "post-FDE detector alarm";
  else if (result.availability != Availability::Available)
    result.reason = "post-FDE protection level exceeds alert limit";
  else result.reason = "research V2 implemented; Gate J calibration/review pending";
  return result;
}

ProtectionLevelV2Result ProtectionLevelV2::computeFrozenAllIn(
    const LinearizedIntegrityWindow& window,
    CandidateEvaluation* candidate,
    const DetectorResultV2& detector,
    const std::vector<FaultHypothesisV2>& hypotheses,
    const std::vector<FaultModeBasis>& modes,
    const FrozenHypothesisNumerics& frozen,
    std::uint64_t fault_model_policy_fingerprint,
    const RiskBudgetV2& risk) const {
  ProtectionLevelV2Result result;
  std::string detector_contract_reason;
  const bool keep_all = candidate && candidate->action.groups_to_remove.empty() &&
      candidate->action.groups_to_add.empty() && candidate->action.added_blocks.empty();
  const bool identity_valid = frozen.valid && window.numerics &&
      frozen.window_id == window.id && frozen.version == window.version &&
      frozen.window_content_fingerprint ==
          window.numerics->content_fingerprint &&
      frozen.window_content_fingerprint == integrityWindowFingerprint(window) &&
      frozen.numerical_contract_fingerprint ==
          window.numerics->numerical_contract_fingerprint &&
      frozen.hypothesis_fingerprint == hypothesisSetFingerprint(hypotheses) &&
      frozen.fault_mode_fingerprint == faultModeSetFingerprint(modes) &&
      frozen.fault_model_policy_fingerprint ==
          fault_model_policy_fingerprint &&
      frozen.hypothesis_count == hypotheses.size() &&
      frozen.pl_entries.size() == hypotheses.size();
  if (!candidate || !candidate->valid || !detector.numerically_valid ||
      !detectorContractMatchesCandidate(window, *candidate, detector,
                                        &detector_contract_reason) ||
      !keep_all || !identity_valid) {
    NumericalWorkCounters::hypothesisSharedMiss();
    result.reason = detector_contract_reason.empty()
        ? "frozen all-in hypothesis context identity mismatch"
        : detector_contract_reason;
    return result;
  }
  if (detector.contract_mode == DetectorContractMode::ActiveDualChannelV6 &&
      detector.channel_history_dof > 0) {
    NumericalWorkCounters::hypothesisSharedMiss();
    result.reason =
        "frozen pooled hypothesis context cannot certify active dual-channel PL";
    return result;
  }
  NumericalWorkCounters::hypothesisSharedHit();
  const RiskBudgetAudit risk_audit = auditRiskBudget(risk, hypotheses);
  result.allocated_outcome_risk = risk_audit.total;
  result.risk_budget_valid = risk_audit.valid;
  if (!result.risk_budget_valid) {
    result.reason = "outcome-conditioned risk budget does not close";
    return result;
  }
  const Eigen::Matrix3d& protected_covariance = frozen.protected_covariance;
  if (!protected_covariance.allFinite() ||
      (protected_covariance.diagonal().array() <= 0.0).any()) {
    result.reason = "post-FDE protected covariance is invalid";
    return result;
  }
  const double nominal_tail = std::max(1e-15, risk.nominal_axis_tail);
  const double nominal_k = StatisticalBoundsCache::normalTwoSidedMultiplier(
      nominal_tail);
  const Eigen::Vector3d sigma =
      protected_covariance.diagonal().cwiseSqrt();
  result.nominal_component_m = nominal_k * sigma;
  result.fault_component_m.setZero();
  result.bridge_component_m.setZero();
  for (std::size_t index = 0; index < hypotheses.size(); ++index) {
    const auto& hypothesis = hypotheses[index];
    const auto& cached = frozen.pl_entries[index];
    if (cached.hypothesis != hypothesis.id || !cached.valid ||
        !(cached.gram_spd || cached.bound_from_projected_path) ||
        !cached.monitorability.monitorable) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " is unmonitorable: rank=" +
          std::to_string(cached.monitorability.rank) + "/" +
          std::to_string(cached.monitorability.parameter_dimension) +
          ";sigma_min=" + std::to_string(cached.monitorability.sigma_min) +
          ";condition=" +
          std::to_string(cached.monitorability.condition_number);
      return result;
    }
    const FaultAxisBounds bounds =
        faultAxisBounds(detector, hypothesis, nominal_tail);
    if (!bounds.valid) {
      result.reason = "remaining post-FDE hypothesis " +
          std::to_string(hypothesis.id.value()) +
          " has no valid section 5.9 bound: " + bounds.reason;
      return result;
    }
    result.hypothesis_tail_used = std::max(result.hypothesis_tail_used,
                                           bounds.hypothesis_tail);
    result.axis_tail_used = std::max(result.axis_tail_used, bounds.axis_tail);
    result.fault_multiplier_used = std::max(result.fault_multiplier_used,
                                            bounds.k);
    result.noncentrality_used = std::max(result.noncentrality_used,
                                         bounds.lambda);
    result.fault_component_m = result.fault_component_m.cwiseMax(
        cached.protected_slopes * bounds.lambda + bounds.k * sigma);
  }
  result.pl_xyz_m = result.nominal_component_m.cwiseMax(
      result.fault_component_m);
  result.hpl_m = std::hypot(result.pl_xyz_m.x(), result.pl_xyz_m.y());
  result.vpl_m = result.pl_xyz_m.z();
  result.model_valid = result.pl_xyz_m.allFinite() && detector.passed;
  result.availability = result.model_valid &&
      result.hpl_m <= risk.horizontal_alert_limit_m &&
      result.vpl_m <= risk.vertical_alert_limit_m
      ? Availability::Available : Availability::Unavailable;
  result.formal_eligible = false;
  if (!detector.passed) result.reason = "post-FDE detector alarm";
  else if (result.availability != Availability::Available)
    result.reason = "post-FDE protection level exceeds alert limit";
  else result.reason = "research V2 implemented; Gate J calibration/review pending";
  return result;
}

}  // namespace uwb_imu_pl
