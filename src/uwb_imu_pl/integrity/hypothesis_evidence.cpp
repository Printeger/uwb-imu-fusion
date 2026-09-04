#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"

#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <algorithm>
#include <map>
#include <stdexcept>

namespace uwb_imu_pl {

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold) const {
  if (!hypotheses) throw std::invalid_argument("hypotheses must not be null");
  std::vector<FaultModeEvidence> results;
  if (!window.model_valid) return results;
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> state_solve(window.H);
  state_solve.setThreshold(config_.rank_tolerance);
  const Eigen::VectorXd parity = window.z - window.H * state_solve.solve(window.z);
  const double all_in = parity.squaredNorm();
  std::map<std::uint64_t, Eigen::MatrixXd> projected_modes;
  std::map<std::uint64_t, int> block_offsets;
  int row_offset = 0;
  for (const auto& block : window.blocks) {
    block_offsets[block.group_id.value()] = row_offset;
    row_offset += block.residual_whitened.size();
  }
  for (const auto& mode : modes) {
    Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(
        window.H.rows(), mode.parameter_dimension);
    bool valid = mode.parameter_dimension > 0;
    for (const auto& item : mode.raw_group_maps) {
      const auto offset = block_offsets.find(item.first.value());
      const auto block = std::find_if(window.blocks.begin(), window.blocks.end(),
          [&](const LinearizedFactorBlock& value) {
            return value.group_id == item.first;
          });
      if (offset == block_offsets.end() || block == window.blocks.end() ||
          item.second.rows() != block->residual_raw.size() ||
          item.second.cols() != mode.parameter_dimension) {
        valid = false;
        break;
      }
      dense.block(offset->second, 0, item.second.rows(), item.second.cols()) =
          block->whitener * item.second;
    }
    if (!valid || !dense.allFinite()) continue;
    projected_modes.emplace(mode.id.value(),
        dense - window.H * state_solve.solve(dense));
  }
  for (auto& hypothesis : *hypotheses) {
    FaultModeEvidence evidence;
    evidence.hypothesis = hypothesis.id;
    evidence.all_in_statistic = all_in;
    auto& monitor = hypothesis.monitorability;
    Eigen::MatrixXd fault_map;
    Eigen::MatrixXd projected;
    Eigen::MatrixXd gram;
    Eigen::VectorXd score;
    if (!hypothesis.modes.empty()) {
      int columns = 0;
      bool found_all = true;
      std::vector<const Eigen::MatrixXd*> parts;
      for (const auto id : hypothesis.modes) {
        const auto found = projected_modes.find(id.value());
        if (found == projected_modes.end()) { found_all = false; break; }
        columns += found->second.cols();
        parts.push_back(&found->second);
      }
      if (found_all) {
        gram = Eigen::MatrixXd::Zero(columns, columns);
        score = Eigen::VectorXd::Zero(columns);
        int left_column = 0;
        for (std::size_t left = 0; left < parts.size(); ++left) {
          const int left_size = parts[left]->cols();
          score.segment(left_column, left_size) = parts[left]->transpose() * parity;
          int right_column = 0;
          for (std::size_t right = 0; right < parts.size(); ++right) {
            const int right_size = parts[right]->cols();
            gram.block(left_column, right_column, left_size, right_size) =
                parts[left]->transpose() * (*parts[right]);
            right_column += right_size;
          }
          left_column += left_size;
        }
      }
      monitor.parameter_dimension = columns;
    } else {
      fault_map = hypothesis.A;
      if (fault_map.rows() == window.H.rows() && fault_map.cols() > 0 &&
          fault_map.allFinite()) {
        projected = fault_map - window.H * state_solve.solve(fault_map);
        gram = projected.transpose() * projected;
        score = projected.transpose() * parity;
      }
      monitor.parameter_dimension = fault_map.cols();
    }
    if (monitor.parameter_dimension == 0 ||
        gram.rows() != monitor.parameter_dimension || !gram.allFinite() ||
        score.size() != monitor.parameter_dimension || !score.allFinite()) {
      monitor.reason = "fault map dimension/non-finite gate failed";
      hypothesis.monitored = false;
      evidence.monitorability = monitor;
      results.push_back(std::move(evidence));
      continue;
    }
    evidence.fault_gram = gram;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(gram);
    if (eigen.info() != Eigen::Success) {
      monitor.reason = "fault Gram eigensolve failed";
      hypothesis.monitored = false;
      evidence.monitorability = monitor;
      results.push_back(std::move(evidence));
      continue;
    }
    const Eigen::VectorXd singular = eigen.eigenvalues().cwiseMax(0.0)
        .cwiseSqrt().reverse();
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
      Eigen::LDLT<Eigen::MatrixXd> solve(gram);
      if (solve.info() != Eigen::Success || !solve.isPositive()) {
        monitor.monitorable = false;
        monitor.reason = "fault Gram solve failed";
        hypothesis.monitored = false;
        evidence.monitorability = monitor;
        results.push_back(std::move(evidence));
        continue;
      }
      evidence.estimated_fault = solve.solve(score);
      evidence.explained_energy = std::max(
          0.0, score.dot(evidence.estimated_fault));
      evidence.conditioned_statistic = std::max(
          0.0, all_in - evidence.explained_energy);
      evidence.log_evidence = 0.5 * evidence.explained_energy;
      evidence.plausible = evidence.conditioned_statistic <=
          squared_detector_threshold + config_.plausible_conditioned_statistic_margin;
    }
    evidence.monitorability = monitor;
    results.push_back(std::move(evidence));
  }
  return results;
}

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const LinearizedIntegrityWindow& window,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold) const {
  return evaluateAll(window, {}, hypotheses, squared_detector_threshold);
}

}  // namespace uwb_imu_pl
