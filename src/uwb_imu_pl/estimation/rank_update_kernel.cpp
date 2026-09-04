#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <set>

namespace uwb_imu_pl {
namespace {

bool selected(FactorGroupId id, const std::vector<FactorGroupId>& ids) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

bool candidateRows(const LinearizedIntegrityWindow& window,
                   const ExclusionAction& action, Eigen::MatrixXd* h,
                   Eigen::VectorXd* z) {
  std::set<std::uint64_t> requested;
  std::set<std::uint64_t> resolved;
  for (const auto id : action.groups_to_remove) requested.insert(id.value());
  int rows = 0;
  for (const auto& block : window.blocks) {
    if (selected(block.group_id, action.groups_to_remove)) {
      resolved.insert(block.group_id.value());
    } else {
      rows += block.jacobian_whitened.rows();
    }
  }
  if (requested != resolved) return false;
  for (const auto& block : action.added_blocks) rows += block.jacobian_whitened.rows();
  h->setZero(rows, window.H.cols());
  z->setZero(rows);
  int offset = 0;
  auto append = [&](const LinearizedFactorBlock& block) {
    const int n = block.jacobian_whitened.rows();
    h->block(offset, 0, n, block.jacobian_whitened.cols()) = block.jacobian_whitened;
    z->segment(offset, n) = block.residual_whitened;
    offset += n;
  };
  for (const auto& block : window.blocks) {
    if (!selected(block.group_id, action.groups_to_remove)) append(block);
  }
  for (const auto& block : action.added_blocks) append(block);
  return true;
}

bool covarianceFromInformation(const Eigen::MatrixXd& information,
                               Eigen::MatrixXd* covariance) {
  Eigen::LLT<Eigen::MatrixXd> llt(information);
  if (llt.info() != Eigen::Success) return false;
  covariance->setIdentity(information.rows(), information.cols());
  *covariance = llt.solve(*covariance);
  *covariance = 0.5 * (*covariance + covariance->transpose());
  return covariance->allFinite();
}

void finishCandidate(CandidateEvaluation* result, const Eigen::MatrixXd& h,
                     const Eigen::VectorXd& z, double rank_tolerance,
                     double max_condition, double max_step) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(h);
  const auto s = svd.singularValues();
  const double largest = s.size() ? s(0) : 0.0;
  const double threshold = rank_tolerance * std::max(1.0, largest);
  result->rank = static_cast<int>((s.array() > threshold).count());
  result->rows = h.rows();
  result->dof = result->rows - result->rank;
  const double smallest = result->rank > 0 ? s(result->rank - 1) : 0.0;
  result->condition_number = smallest > 0.0 ? largest / smallest
                                             : std::numeric_limits<double>::infinity();
  const Eigen::VectorXd residual = z - h * result->state_increment;
  result->statistic = residual.squaredNorm();
  Eigen::LLT<Eigen::MatrixXd> llt(h.transpose() * h);
  if (llt.info() == Eigen::Success) {
    result->information_logdet =
        2.0 * llt.matrixL().toDenseMatrix().diagonal().array().log().sum();
  }
  result->valid = result->covariance.allFinite() &&
      result->state_increment.allFinite() && result->rank == h.cols() &&
      result->dof > 0 && result->condition_number <= max_condition &&
      result->state_increment.norm() <= max_step &&
      std::isfinite(result->statistic);
  if (!result->valid && result->reason.empty()) {
    if (result->rank != h.cols()) result->reason = "candidate rank loss";
    else if (result->dof <= 0) result->reason = "candidate has no residual dof";
    else if (result->condition_number > max_condition) result->reason = "candidate condition gate failed";
    else if (result->state_increment.norm() > max_step) result->reason = "candidate linearization step gate failed";
    else result->reason = "candidate contains non-finite values";
  }
}

}  // namespace

BaseCandidateKernel RankUpdateEvaluator::factorizeOnce(
    const LinearizedIntegrityWindow& window) const {
  BaseCandidateKernel base;
  base.window_id = window.id;
  base.version = window.version;
  base.window = window;
  if (!window.model_valid) {
    base.reason = "invalid base window: " + window.reason;
    return base;
  }
  if (!covarianceFromInformation(window.base_information, &base.covariance)) {
    base.reason = "base information is not SPD";
    return base;
  }
  base.state_increment = base.covariance * window.base_information_rhs;
  // A large all-in step is itself a fault symptom. Keep the one-time base
  // factorization usable so exclusion candidates can recover; apply the
  // linearization-step gate independently to every candidate below.
  base.valid = base.state_increment.allFinite();
  if (!base.valid) base.reason = "base solution contains non-finite values";
  return base;
}

CandidateEvaluation RankUpdateEvaluator::evaluate(
    const BaseCandidateKernel& base, const ExclusionAction& action) const {
  CandidateEvaluation result;
  const auto wall_start = std::chrono::steady_clock::now();
  result.action = action;
  result.base_version = base.version;
  if (!base.valid) {
    result.reason = base.reason;
    return result;
  }
  Eigen::MatrixXd covariance = base.covariance;
  Eigen::VectorXd information_rhs = base.window.base_information_rhs;

  std::vector<const LinearizedFactorBlock*> removals;
  for (const auto& block : base.window.blocks) {
    if (selected(block.group_id, action.groups_to_remove)) removals.push_back(&block);
  }
  std::sort(removals.begin(), removals.end(), [](const auto* a, const auto* b) {
    return a->group_id < b->group_id;
  });
  std::set<std::uint64_t> requested_removals;
  for (const auto id : action.groups_to_remove) {
    requested_removals.insert(id.value());
  }
  std::set<std::uint64_t> resolved_removals;
  for (const auto* block : removals) {
    resolved_removals.insert(block->group_id.value());
  }
  if (requested_removals != resolved_removals) {
    result.reason = "candidate removal block is absent from frozen window";
    result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    return result;
  }
  auto stacked = [&](const std::vector<const LinearizedFactorBlock*>& blocks) {
    int rows = 0;
    for (const auto* block : blocks) rows += block->jacobian_whitened.rows();
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> value{
        Eigen::MatrixXd::Zero(rows, base.window.H.cols()),
        Eigen::VectorXd::Zero(rows)};
    int offset = 0;
    for (const auto* block : blocks) {
      value.first.middleRows(offset, block->jacobian_whitened.rows()) =
          block->jacobian_whitened;
      value.second.segment(offset, block->residual_whitened.size()) =
          block->residual_whitened;
      offset += block->jacobian_whitened.rows();
    }
    return value;
  };
  if (!removals.empty()) {
    const auto removal = stacked(removals);
    const Eigen::MatrixXd& j = removal.first;
    const Eigen::VectorXd& b = removal.second;
    const Eigen::MatrixXd inner = Eigen::MatrixXd::Identity(j.rows(), j.rows()) -
        j * covariance * j.transpose();
    Eigen::LLT<Eigen::MatrixXd> llt(inner);
    if (llt.info() != Eigen::Success) {
      result.reason = "Woodbury downdate inner matrix is not SPD";
      result.wall_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
      return result;
    }
    const Eigen::MatrixXd cj = covariance * j.transpose();
    covariance += cj * llt.solve(cj.transpose());
    covariance = 0.5 * (covariance + covariance.transpose());
    information_rhs.noalias() -= j.transpose() * b;
  }
  std::vector<const LinearizedFactorBlock*> additions;
  for (const auto& block : action.added_blocks) {
    if (!(block.version == base.version) ||
        block.jacobian_whitened.cols() != base.window.H.cols() ||
        block.jacobian_whitened.rows() != block.residual_whitened.size() ||
        !block.jacobian_whitened.allFinite() ||
        !block.residual_whitened.allFinite()) {
      result.reason = "candidate addition block/version is invalid";
      result.wall_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
      return result;
    }
    additions.push_back(&block);
  }
  std::sort(additions.begin(), additions.end(), [](const auto* a, const auto* b) {
    return a->group_id < b->group_id;
  });
  if (!additions.empty()) {
    const auto addition = stacked(additions);
    const Eigen::MatrixXd& j = addition.first;
    const Eigen::VectorXd& b = addition.second;
    const Eigen::MatrixXd inner = Eigen::MatrixXd::Identity(j.rows(), j.rows()) +
        j * covariance * j.transpose();
    Eigen::LLT<Eigen::MatrixXd> llt(inner);
    if (llt.info() != Eigen::Success) {
      result.reason = "Woodbury update inner matrix is not SPD";
      result.wall_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
      return result;
    }
    const Eigen::MatrixXd cj = covariance * j.transpose();
    covariance -= cj * llt.solve(cj.transpose());
    covariance = 0.5 * (covariance + covariance.transpose());
    information_rhs.noalias() += j.transpose() * b;
  }
  result.covariance = covariance;
  result.state_increment = covariance * information_rhs;
  Eigen::MatrixXd h;
  Eigen::VectorXd z;
  if (!candidateRows(base.window, action, &h, &z)) {
    result.reason = "candidate removal block is absent from frozen window";
    result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    return result;
  }
  result.retained_jacobian = h;
  result.retained_residual = z;
  finishCandidate(&result, h, z, config_.rank_tolerance,
                  config_.max_condition_number,
                  config_.max_linearization_step_norm);
  result.wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wall_start).count();
  return result;
}

CandidateEvaluation DenseCandidateOracle::evaluate(
    const LinearizedIntegrityWindow& window, const ExclusionAction& action) const {
  CandidateEvaluation result;
  const auto wall_start = std::chrono::steady_clock::now();
  result.action = action;
  result.base_version = window.version;
  for (const auto& block : action.added_blocks) {
    if (!(block.version == window.version) ||
        block.jacobian_whitened.cols() != window.H.cols() ||
        block.jacobian_whitened.rows() != block.residual_whitened.size() ||
        !block.jacobian_whitened.allFinite() ||
        !block.residual_whitened.allFinite()) {
      result.reason = "candidate addition block/version is invalid";
      result.wall_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
      return result;
    }
  }
  Eigen::MatrixXd h;
  Eigen::VectorXd z;
  if (!candidateRows(window, action, &h, &z)) {
    result.reason = "candidate removal block is absent from frozen window";
    result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    return result;
  }
  result.retained_jacobian = h;
  result.retained_residual = z;
  const Eigen::MatrixXd information = h.transpose() * h;
  if (!covarianceFromInformation(information, &result.covariance)) {
    result.reason = "dense candidate information is not SPD";
    result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    return result;
  }
  result.state_increment = result.covariance * h.transpose() * z;
  finishCandidate(&result, h, z, config_.rank_tolerance,
                  config_.max_condition_number,
                  config_.max_linearization_step_norm);
  result.wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wall_start).count();
  return result;
}

}  // namespace uwb_imu_pl
