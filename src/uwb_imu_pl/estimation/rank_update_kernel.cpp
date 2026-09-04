#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <set>

namespace uwb_imu_pl {
namespace {

bool groupSelected(FactorGroupId id, const std::vector<FactorGroupId>& ids) {
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
    if (groupSelected(block.group_id, action.groups_to_remove)) {
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
    if (!groupSelected(block.group_id, action.groups_to_remove)) append(block);
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

Eigen::MatrixXd CandidateEvaluation::covarianceTimes(
    const Eigen::MatrixXd& value) const {
  if (covariance.size() != 0) return covariance * value;
  if (!shared_base_covariance || shared_base_covariance->cols() != value.rows()) {
    return Eigen::MatrixXd();
  }
  Eigen::MatrixXd result = (*shared_base_covariance) * value;
  if (covariance_plus_factor.cols()) {
    result.noalias() += covariance_plus_factor *
        (covariance_plus_factor.transpose() * value);
  }
  if (covariance_minus_factor.cols()) {
    result.noalias() -= covariance_minus_factor *
        (covariance_minus_factor.transpose() * value);
  }
  return result;
}

Eigen::MatrixXd CandidateEvaluation::normalCross(
    const Eigen::MatrixXd& row_map) const {
  if (retainedJacobian().rows() == row_map.rows() &&
      retainedJacobian().cols() > 0) {
    return retainedJacobian().transpose() * row_map;
  }
  if (!window_view || row_map.rows() != rows) return Eigen::MatrixXd();
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(
      window_view->H.cols(), row_map.cols());
  Eigen::Index offset = 0;
  for (const auto& block : window_view->blocks) {
    if (groupSelected(block.group_id, action.groups_to_remove)) continue;
    const Eigen::Index count = block.jacobian_whitened.rows();
    result.noalias() += block.jacobian_whitened.transpose() *
        row_map.middleRows(offset, count);
    offset += count;
  }
  for (const auto& block : action.added_blocks) {
    const Eigen::Index count = block.jacobian_whitened.rows();
    result.noalias() += block.jacobian_whitened.transpose() *
        row_map.middleRows(offset, count);
    offset += count;
  }
  return offset == row_map.rows() ? result : Eigen::MatrixXd();
}

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
  Eigen::MatrixXd base_covariance;
  if (!covarianceFromInformation(window.base_information, &base_covariance)) {
    base.reason = "base information is not SPD";
    return base;
  }
  base.covariance = std::make_shared<const Eigen::MatrixXd>(
      std::move(base_covariance));
  base.state_increment = (*base.covariance) * window.base_information_rhs;
  Eigen::LLT<Eigen::MatrixXd> base_llt(window.base_information);
  if (base_llt.info() == Eigen::Success) {
    base.information_logdet = 2.0 * base_llt.matrixL().toDenseMatrix()
        .diagonal().array().log().sum();
  }
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
  result.window_view = &base.window;
  if (!base.valid) {
    result.reason = base.reason;
    return result;
  }
  result.shared_base_covariance = base.covariance;
  Eigen::VectorXd information_rhs = base.window.base_information_rhs;
  double retained_squared_norm = base.window.z.squaredNorm();
  result.information_logdet = base.information_logdet;

  std::vector<const LinearizedFactorBlock*> removals;
  for (const auto& block : base.window.blocks) {
    if (groupSelected(block.group_id, action.groups_to_remove)) removals.push_back(&block);
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
    const Eigen::MatrixXd cj = (*base.covariance) * j.transpose();
    const Eigen::MatrixXd inner = Eigen::MatrixXd::Identity(j.rows(), j.rows()) -
        j * cj;
    Eigen::LLT<Eigen::MatrixXd> llt(inner);
    if (llt.info() != Eigen::Success) {
      result.reason = "Woodbury downdate inner matrix is not SPD";
      result.wall_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
      return result;
    }
    result.covariance_plus_factor =
        llt.matrixL().solve(cj.transpose()).transpose();
    const Eigen::VectorXd diagonal =
        llt.matrixL().toDenseMatrix().diagonal().cwiseAbs();
    if (diagonal.minCoeff() > 0.0 &&
        std::pow(diagonal.maxCoeff() / diagonal.minCoeff(), 2) >=
            config_.exact_slow_path_condition) {
      result.exact_slow_path = true;
    }
    information_rhs.noalias() -= j.transpose() * b;
    retained_squared_norm -= b.squaredNorm();
    result.information_logdet += 2.0 * llt.matrixL().toDenseMatrix()
        .diagonal().array().log().sum();
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
    Eigen::MatrixXd cj = (*base.covariance) * j.transpose();
    if (result.covariance_plus_factor.cols()) {
      cj.noalias() += result.covariance_plus_factor *
          (result.covariance_plus_factor.transpose() * j.transpose());
    }
    const Eigen::MatrixXd inner = Eigen::MatrixXd::Identity(j.rows(), j.rows()) +
        j * cj;
    Eigen::LLT<Eigen::MatrixXd> llt(inner);
    if (llt.info() != Eigen::Success) {
      result.reason = "Woodbury update inner matrix is not SPD";
      result.wall_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
      return result;
    }
    result.covariance_minus_factor =
        llt.matrixL().solve(cj.transpose()).transpose();
    const Eigen::VectorXd diagonal =
        llt.matrixL().toDenseMatrix().diagonal().cwiseAbs();
    if (diagonal.minCoeff() > 0.0 &&
        std::pow(diagonal.maxCoeff() / diagonal.minCoeff(), 2) >=
            config_.exact_slow_path_condition) {
      result.exact_slow_path = true;
    }
    information_rhs.noalias() += j.transpose() * b;
    retained_squared_norm += b.squaredNorm();
    result.information_logdet += 2.0 * llt.matrixL().toDenseMatrix()
        .diagonal().array().log().sum();
  }
  result.state_increment = result.covarianceTimes(information_rhs);
  Eigen::MatrixXd h;
  Eigen::VectorXd z;
  result.rows = base.window.H.rows();
  for (const auto* block : removals) {
    result.rows -= block->jacobian_whitened.rows();
  }
  for (const auto* block : additions) {
    result.rows += block->jacobian_whitened.rows();
  }
  if ((config_.materialize_dense_oracle_fields || result.exact_slow_path) &&
      !candidateRows(base.window, action, &h, &z)) {
    result.reason = "candidate removal block is absent from frozen window";
    result.wall_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    return result;
  }
  if (config_.materialize_dense_oracle_fields || result.exact_slow_path) {
    result.retained_jacobian = h;
    result.retained_residual = z;
    const Eigen::MatrixXd dense_covariance = result.covarianceTimes(
        Eigen::MatrixXd::Identity(base.window.H.cols(), base.window.H.cols()));
    result.covariance = dense_covariance;
    finishCandidate(&result, result.retainedJacobian(), z, config_.rank_tolerance,
                    config_.max_condition_number,
                    config_.max_linearization_step_norm);
  } else {
    result.rank = base.window.H.cols();
    result.dof = result.rows - result.rank;
    result.condition_number = base.window.condition_number;
    result.statistic = std::max(0.0, retained_squared_norm -
        information_rhs.dot(result.state_increment));
    result.valid = result.state_increment.allFinite() && result.dof > 0 &&
        result.condition_number <= config_.max_condition_number &&
        result.state_increment.norm() <= config_.max_linearization_step_norm &&
        std::isfinite(result.statistic) && std::isfinite(result.information_logdet);
    if (!result.valid && result.reason.empty()) {
      result.reason = result.dof <= 0 ? "candidate has no residual dof" :
          (result.condition_number > config_.max_condition_number
              ? "candidate condition gate failed"
              : (result.state_increment.norm() > config_.max_linearization_step_norm
                  ? "candidate linearization step gate failed"
                  : "candidate contains non-finite values"));
    }
  }
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
