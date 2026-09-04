#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"

#include <Eigen/SVD>

#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const LinearizedIntegrityWindow& window,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold) const {
  if (!hypotheses) throw std::invalid_argument("hypotheses must not be null");
  std::vector<FaultModeEvidence> results;
  if (!window.model_valid) return results;
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> state_solve(window.H);
  state_solve.setThreshold(config_.rank_tolerance);
  const Eigen::VectorXd parity = window.z - window.H * state_solve.solve(window.z);
  const double all_in = parity.squaredNorm();
  for (auto& hypothesis : *hypotheses) {
    FaultModeEvidence evidence;
    evidence.hypothesis = hypothesis.id;
    evidence.all_in_statistic = all_in;
    auto& monitor = hypothesis.monitorability;
    monitor.parameter_dimension = hypothesis.A.cols();
    if (hypothesis.A.rows() != window.H.rows() || hypothesis.A.cols() == 0 ||
        !hypothesis.A.allFinite()) {
      monitor.reason = "fault map dimension/non-finite gate failed";
      hypothesis.monitored = false;
      evidence.monitorability = monitor;
      results.push_back(std::move(evidence));
      continue;
    }
    const Eigen::MatrixXd projected = hypothesis.A -
        window.H * state_solve.solve(hypothesis.A);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(projected,
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = svd.singularValues();
    monitor.sigma_max = singular.size() ? singular(0) : 0.0;
    const double rank_gate = config_.rank_tolerance *
        std::max(1.0, monitor.sigma_max);
    monitor.rank = static_cast<int>((singular.array() > rank_gate).count());
    monitor.sigma_min = monitor.rank > 0 ? singular(monitor.rank - 1) : 0.0;
    monitor.condition_number = monitor.sigma_min > 0.0
        ? monitor.sigma_max / monitor.sigma_min
        : std::numeric_limits<double>::infinity();
    monitor.monitorable = monitor.rank == monitor.parameter_dimension &&
        monitor.sigma_min >= config_.min_fault_gram_sigma &&
        monitor.condition_number <= config_.max_fault_gram_condition;
    if (!monitor.monitorable) monitor.reason = "fault subspace is unmonitorable";
    hypothesis.monitored = monitor.monitorable;
    if (monitor.monitorable) {
      evidence.estimated_fault = svd.solve(parity);
      const Eigen::VectorXd conditioned = parity - projected * evidence.estimated_fault;
      evidence.conditioned_statistic = conditioned.squaredNorm();
      evidence.explained_energy = std::max(0.0, all_in - evidence.conditioned_statistic);
      evidence.log_evidence = 0.5 * evidence.explained_energy;
      evidence.plausible = evidence.conditioned_statistic <=
          squared_detector_threshold + config_.plausible_conditioned_statistic_margin;
    }
    evidence.monitorability = monitor;
    results.push_back(std::move(evidence));
  }
  return results;
}

}  // namespace uwb_imu_pl
