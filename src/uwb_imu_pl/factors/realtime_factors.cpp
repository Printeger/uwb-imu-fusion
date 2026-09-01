#include "uwb_imu_pl/factors/realtime_factors.hpp"

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>

#include <Eigen/Cholesky>

#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {

Eigen::MatrixXd validatedUwbCovariance(const UwbBatch& batch) {
  const Eigen::Index n = static_cast<Eigen::Index>(batch.measurements.size());
  if (n == 0) throw std::invalid_argument("UWB batch must not be empty");
  Eigen::MatrixXd covariance = batch.covariance_m2;
  if (covariance.size() == 0) {
    covariance = Eigen::MatrixXd::Zero(n, n);
    for (Eigen::Index i = 0; i < n; ++i) {
      const double sigma = batch.measurements[static_cast<std::size_t>(i)].sigma_m;
      if (!std::isfinite(sigma) || sigma <= 0.0) {
        throw std::invalid_argument("UWB sigma must be finite and > 0");
      }
      covariance(i, i) = sigma * sigma;
    }
  }
  if (covariance.rows() != n || covariance.cols() != n ||
      !covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-12)) {
    throw std::invalid_argument("invalid UWB group covariance");
  }
  Eigen::LLT<Eigen::MatrixXd> llt(covariance);
  if (llt.info() != Eigen::Success) {
    throw std::invalid_argument("UWB group covariance must be positive definite");
  }
  return covariance;
}

UwbPositionBatchFactor::UwbPositionBatchFactor(gtsam::Key position_key,
                                               const UwbBatch& batch)
    : gtsam::NoiseModelFactor1<gtsam::Point3>(
          gtsam::noiseModel::Gaussian::Covariance(validatedUwbCovariance(batch)),
          position_key),
      batch_(batch) {}

gtsam::Vector UwbPositionBatchFactor::evaluateError(
    const gtsam::Point3& position,
    boost::optional<gtsam::Matrix&> jacobian) const {
  const Eigen::Index n = static_cast<Eigen::Index>(batch_.measurements.size());
  gtsam::Vector error(n);
  if (jacobian) *jacobian = gtsam::Matrix::Zero(n, 3);
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto& measurement = batch_.measurements[static_cast<std::size_t>(i)];
    const gtsam::Vector3 delta = position - measurement.anchor_position_m;
    const double range = delta.norm();
    if (range < 1e-9) throw std::runtime_error("UWB factor singular at anchor");
    error(i) = range - measurement.range_m;
    if (jacobian) jacobian->row(i) = delta.transpose() / range;
  }
  return error;
}

UwbPoseBatchFactor::UwbPoseBatchFactor(gtsam::Key pose_key,
                                       const UwbBatch& batch,
                                       const gtsam::Point3& lever_arm_body_m)
    : gtsam::NoiseModelFactor1<gtsam::Pose3>(
          gtsam::noiseModel::Gaussian::Covariance(validatedUwbCovariance(batch)),
          pose_key),
      batch_(batch), lever_arm_body_m_(lever_arm_body_m) {}

gtsam::Vector UwbPoseBatchFactor::evaluateError(
    const gtsam::Pose3& pose,
    boost::optional<gtsam::Matrix&> jacobian) const {
  const Eigen::Index n = static_cast<Eigen::Index>(batch_.measurements.size());
  gtsam::Vector error(n);
  gtsam::Matrix36 antenna_jacobian;
  const gtsam::Point3 antenna = pose.transformFrom(lever_arm_body_m_, antenna_jacobian);
  if (jacobian) *jacobian = gtsam::Matrix::Zero(n, 6);
  for (Eigen::Index i = 0; i < n; ++i) {
    const auto& measurement = batch_.measurements[static_cast<std::size_t>(i)];
    const gtsam::Vector3 delta = antenna - measurement.anchor_position_m;
    const double range = delta.norm();
    if (range < 1e-9) throw std::runtime_error("UWB pose factor singular at anchor");
    error(i) = range - measurement.range_m;
    if (jacobian) jacobian->row(i) = (delta.transpose() / range) * antenna_jacobian;
  }
  return error;
}

ConstantVelocityRegularizer::ConstantVelocityRegularizer(
    gtsam::Key previous_position, gtsam::Key previous_velocity,
    gtsam::Key current_position, gtsam::Key current_velocity, double dt_seconds,
    double sigma)
    : gtsam::NoiseModelFactor4<gtsam::Point3, gtsam::Vector3,
                               gtsam::Point3, gtsam::Vector3>(
          gtsam::noiseModel::Isotropic::Sigma(6, sigma), previous_position,
          previous_velocity, current_position, current_velocity),
      dt_seconds_(dt_seconds) {
  if (!std::isfinite(dt_seconds_) || dt_seconds_ <= 0.0) {
    throw std::invalid_argument("constant-velocity dt must be finite and > 0");
  }
}

gtsam::Vector ConstantVelocityRegularizer::evaluateError(
    const gtsam::Point3& p0, const gtsam::Vector3& v0,
    const gtsam::Point3& p1, const gtsam::Vector3& v1,
    boost::optional<gtsam::Matrix&> h1, boost::optional<gtsam::Matrix&> h2,
    boost::optional<gtsam::Matrix&> h3, boost::optional<gtsam::Matrix&> h4) const {
  gtsam::Vector6 error;
  error.head<3>() = p1 - p0 - dt_seconds_ * v0;
  error.tail<3>() = v1 - v0;
  if (h1) {
    *h1 = gtsam::Matrix::Zero(6, 3);
    h1->topRows<3>() = -gtsam::Matrix3::Identity();
  }
  if (h2) {
    *h2 = gtsam::Matrix::Zero(6, 3);
    h2->topRows<3>() = -dt_seconds_ * gtsam::Matrix3::Identity();
    h2->bottomRows<3>() = -gtsam::Matrix3::Identity();
  }
  if (h3) {
    *h3 = gtsam::Matrix::Zero(6, 3);
    h3->topRows<3>() = gtsam::Matrix3::Identity();
  }
  if (h4) {
    *h4 = gtsam::Matrix::Zero(6, 3);
    h4->bottomRows<3>() = gtsam::Matrix3::Identity();
  }
  return error;
}

Eigen::Matrix<double, 3, 6> worldPositionPoseTangentJacobian(
    const gtsam::Pose3& pose) {
  Eigen::Matrix<double, 3, 6> jacobian = Eigen::Matrix<double, 3, 6>::Zero();
  jacobian.rightCols<3>() = pose.rotation().matrix();
  return jacobian;
}

}  // namespace uwb_imu_pl
