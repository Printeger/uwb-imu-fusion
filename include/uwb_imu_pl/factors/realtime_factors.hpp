#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace uwb_imu_pl {

Eigen::MatrixXd validatedUwbCovariance(const UwbBatch& batch);

class UwbPositionBatchFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Point3> {
 public:
  UwbPositionBatchFactor(gtsam::Key position_key, const UwbBatch& batch);
  gtsam::Vector evaluateError(
      const gtsam::Point3& position,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override;
  const UwbBatch& batch() const { return batch_; }

 private:
  UwbBatch batch_;
};

class UwbPoseBatchFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  UwbPoseBatchFactor(gtsam::Key pose_key, const UwbBatch& batch,
                     const gtsam::Point3& lever_arm_body_m);
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> jacobian = boost::none) const override;
  const UwbBatch& batch() const { return batch_; }

 private:
  UwbBatch batch_;
  gtsam::Point3 lever_arm_body_m_;
};

class ConstantVelocityRegularizer final
    : public gtsam::NoiseModelFactor4<gtsam::Point3, gtsam::Vector3,
                                      gtsam::Point3, gtsam::Vector3> {
 public:
  ConstantVelocityRegularizer(gtsam::Key previous_position,
                              gtsam::Key previous_velocity,
                              gtsam::Key current_position,
                              gtsam::Key current_velocity, double dt_seconds,
                              double sigma);
  gtsam::Vector evaluateError(
      const gtsam::Point3& previous_position,
      const gtsam::Vector3& previous_velocity,
      const gtsam::Point3& current_position,
      const gtsam::Vector3& current_velocity,
      boost::optional<gtsam::Matrix&> h1 = boost::none,
      boost::optional<gtsam::Matrix&> h2 = boost::none,
      boost::optional<gtsam::Matrix&> h3 = boost::none,
      boost::optional<gtsam::Matrix&> h4 = boost::none) const override;

 private:
  double dt_seconds_;
};

// Jacobian of body-origin world position with respect to GTSAM Pose3 tangent
// coordinates (rotation, local translation). This is [0, R_WB], not a direct
// selection of translation covariance indices.
Eigen::Matrix<double, 3, 6> worldPositionPoseTangentJacobian(
    const gtsam::Pose3& pose);

}  // namespace uwb_imu_pl
