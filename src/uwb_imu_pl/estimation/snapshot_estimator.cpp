#include "uwb_imu_pl/estimation/snapshot_estimator.hpp"

#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <cmath>
#include <set>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

Eigen::MatrixXd covarianceFor(const UwbBatch& batch) {
  const Eigen::Index n = static_cast<Eigen::Index>(batch.measurements.size());
  Eigen::MatrixXd covariance;
  if (batch.covariance_m2.size() == 0) {
    covariance = Eigen::MatrixXd::Zero(n, n);
    for (Eigen::Index i = 0; i < n; ++i) {
      const double sigma = batch.measurements[static_cast<std::size_t>(i)].sigma_m;
      if (!std::isfinite(sigma) || sigma <= 0.0) {
        throw std::invalid_argument("all UWB sigma values must be finite and > 0");
      }
      covariance(i, i) = sigma * sigma;
    }
  } else {
    if (batch.covariance_m2.rows() != n || batch.covariance_m2.cols() != n) {
      throw std::invalid_argument("UWB covariance dimension mismatch");
    }
    covariance = batch.covariance_m2;
  }
  if (!covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-12)) {
    throw std::invalid_argument("UWB covariance must be finite and symmetric");
  }
  Eigen::LLT<Eigen::MatrixXd> llt(covariance);
  if (llt.info() != Eigen::Success) {
    throw std::invalid_argument("UWB covariance must be positive definite");
  }
  return covariance;
}

void linearize(const UwbBatch& batch, const Eigen::Vector3d& position,
               Eigen::MatrixXd* jacobian, Eigen::VectorXd* residual) {
  const Eigen::Index n = static_cast<Eigen::Index>(batch.measurements.size());
  jacobian->resize(n, 3);
  residual->resize(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto& measurement = batch.measurements[static_cast<std::size_t>(i)];
    const Eigen::Vector3d delta = position - measurement.anchor_position_m;
    const double predicted = delta.norm();
    if (!std::isfinite(measurement.range_m) || measurement.range_m <= 0.0 ||
        !measurement.anchor_position_m.allFinite() || predicted < 1e-9) {
      throw std::invalid_argument("invalid range geometry in UWB batch");
    }
    jacobian->row(i) = delta.transpose() / predicted;
    (*residual)(i) = measurement.range_m - predicted;
  }
}

LinearizationDiagnostics diagnose(const Eigen::MatrixXd& h, double rank_tolerance,
                                  double step_norm, double max_condition) {
  LinearizationDiagnostics d;
  d.rows = static_cast<int>(h.rows());
  d.columns = static_cast<int>(h.cols());
  d.linearization_step_norm = step_norm;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.singularValues().size() == 0) {
    d.reason = "empty singular-value spectrum";
    return d;
  }
  const double largest = svd.singularValues()(0);
  const double cutoff = rank_tolerance * std::max(h.rows(), h.cols()) * largest;
  d.rank = static_cast<int>((svd.singularValues().array() > cutoff).count());
  const double smallest = svd.singularValues()(svd.singularValues().size() - 1);
  d.condition_number = smallest > cutoff ? largest / smallest :
      std::numeric_limits<double>::infinity();
  d.covariance_valid = d.rank == h.cols() && std::isfinite(d.condition_number);
  d.model_valid = d.covariance_valid && d.condition_number <= max_condition;
  if (d.rank != h.cols()) d.reason = "rank-deficient UWB geometry";
  else if (!std::isfinite(d.condition_number) || d.condition_number > max_condition)
    d.reason = "UWB geometry condition limit exceeded";
  return d;
}

}  // namespace

SnapshotSolution SnapshotUwbEstimator::estimate(
    const UwbBatch& batch, const Eigen::Vector3d& initial_position_world_m) const {
  SnapshotSolution result;
  result.timestamp = batch.timestamp;
  result.position_world_m = initial_position_world_m;
  if (batch.measurements.size() < 4) {
    result.diagnostics.rows = static_cast<int>(batch.measurements.size());
    result.diagnostics.columns = 3;
    result.diagnostics.reason = "3D snapshot needs at least four ranges for residual redundancy";
    return result;
  }
  if (!initial_position_world_m.allFinite()) {
    throw std::invalid_argument("initial snapshot position must be finite");
  }
  result.covariance_measurement_m2 = covarianceFor(batch);
  Eigen::LLT<Eigen::MatrixXd> llt(result.covariance_measurement_m2);
  result.whitener = llt.matrixL().solve(
      Eigen::MatrixXd::Identity(llt.matrixL().rows(), llt.matrixL().cols()));

  double step_norm = std::numeric_limits<double>::infinity();
  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    linearize(batch, result.position_world_m, &result.jacobian_raw,
              &result.residual_raw_m);
    result.jacobian_whitened = result.whitener * result.jacobian_raw;
    const Eigen::VectorXd whitened = result.whitener * result.residual_raw_m;
    const Eigen::Vector3d step = result.jacobian_whitened
        .completeOrthogonalDecomposition().solve(whitened);
    if (!step.allFinite()) {
      result.diagnostics.reason = "non-finite NLS step";
      return result;
    }
    step_norm = step.norm();
    result.position_world_m += step;
    result.iterations = iteration + 1;
    if (step_norm <= config_.step_tolerance_m) {
      result.converged = true;
      break;
    }
  }

  linearize(batch, result.position_world_m, &result.jacobian_raw,
            &result.residual_raw_m);
  result.jacobian_whitened = result.whitener * result.jacobian_raw;
  result.residual_whitened = result.whitener * result.residual_raw_m;
  result.diagnostics = diagnose(result.jacobian_whitened, config_.rank_tolerance,
                                step_norm, config_.max_condition_number);
  if (!result.converged) {
    result.diagnostics.model_valid = false;
    result.diagnostics.reason = "snapshot NLS did not converge";
    return result;
  }
  if (!result.diagnostics.covariance_valid) return result;

  const Eigen::Matrix3d information =
      result.jacobian_whitened.transpose() * result.jacobian_whitened;
  Eigen::LDLT<Eigen::Matrix3d> ldlt(information);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    result.diagnostics.covariance_valid = false;
    result.diagnostics.model_valid = false;
    result.diagnostics.reason = "snapshot information matrix is not positive definite";
    return result;
  }
  result.covariance_m2 = ldlt.solve(Eigen::Matrix3d::Identity());
  result.gain_whitened = result.covariance_m2 * result.jacobian_whitened.transpose();
  result.residual_projector = Eigen::MatrixXd::Identity(
      result.jacobian_whitened.rows(), result.jacobian_whitened.rows()) -
      result.jacobian_whitened * result.gain_whitened;
  return result;
}

std::shared_ptr<const EstimationSnapshot> SnapshotUwbEstimator::makeSnapshot(
    const UwbBatch& batch, const SnapshotSolution& solution,
    std::uint64_t version_number) const {
  NavigationState state;
  state.id = StateId(version_number);
  state.timestamp = batch.timestamp;
  state.position_world_m = solution.position_world_m;
  LinearizationVersion version{version_number, 1, 1, version_number};
  std::vector<WhitenedRowBlock> rows;
  WhitenedRowBlock row;
  row.factor_id = batch.measurements.front().factor_id;
  row.role = RowRole::Measurement;
  row.jacobian = solution.jacobian_whitened;
  row.residual = solution.residual_whitened;
  row.covariance = solution.covariance_measurement_m2;
  row.whitener = solution.whitener;
  row.jacobian_raw = solution.jacobian_raw;
  row.residual_raw = solution.residual_raw_m;
  row.column_indices = {0, 1, 2};
  row.whitening_model_id = batch.covariance_model_id;
  row.version = version;
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    row.measurement_ids.push_back(batch.measurements[i].id);
    row.anchor_ids.push_back(batch.measurements[i].anchor_id);
  }
  rows.push_back(std::move(row));
  SnapshotCapabilities capabilities;
  capabilities.dense_snapshot = true;
  capabilities.sparse_solve = true;
  capabilities.factor_provenance = true;
  capabilities.consistent_relinearization = true;
  const Eigen::Matrix3d information = solution.jacobian_whitened.transpose() *
      solution.jacobian_whitened;
  return std::make_shared<ImmutableEstimationSnapshot>(
      state, version, solution.diagnostics, std::move(rows), capabilities,
      LinearizationConsistency::Strict, std::nullopt, solution.covariance_m2,
      information);
}

}  // namespace uwb_imu_pl
