#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ImuBias.h>

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

// Independent replacement for a suspect CombinedImuFactor. It consumes only
// the two navigation states and dt; no current-interval IMU sample is read.
// Residual order is rotation(local), position(world), velocity(world).
class KinematicPoseVelocityBridgeFactor final
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3,
                                      gtsam::Pose3, gtsam::Vector3> {
 public:
  KinematicPoseVelocityBridgeFactor(
      gtsam::Key previous_pose, gtsam::Key previous_velocity,
      gtsam::Key current_pose, gtsam::Key current_velocity, double dt_seconds,
      const Eigen::Matrix<double, 9, 9>& covariance);

  gtsam::Vector evaluateError(
      const gtsam::Pose3& previous_pose,
      const gtsam::Vector3& previous_velocity,
      const gtsam::Pose3& current_pose,
      const gtsam::Vector3& current_velocity,
      boost::optional<gtsam::Matrix&> h1 = boost::none,
      boost::optional<gtsam::Matrix&> h2 = boost::none,
      boost::optional<gtsam::Matrix&> h3 = boost::none,
      boost::optional<gtsam::Matrix&> h4 = boost::none) const override;

  double dtSeconds() const { return dt_seconds_; }

 private:
  gtsam::Vector9 residual(const gtsam::Pose3& previous_pose,
                          const gtsam::Vector3& previous_velocity,
                          const gtsam::Pose3& current_pose,
                          const gtsam::Vector3& current_velocity) const;
  double dt_seconds_;
};

// Jacobian of body-origin world position with respect to GTSAM Pose3 tangent
// coordinates (rotation, local translation). This is [0, R_WB], not a direct
// selection of translation covariance indices.
Eigen::Matrix<double, 3, 6> worldPositionPoseTangentJacobian(
    const gtsam::Pose3& pose);

}  // namespace uwb_imu_pl
