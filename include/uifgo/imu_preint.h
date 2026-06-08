#pragma once

#include "uifgo/types.h"
#include "uifgo/config.h"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/base/Vector.h>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace uifgo {

// Thin wrapper around GTSAM PreintegratedCombinedMeasurements.
// Manages reset-integrate-predict lifecycle for one IMU segment.
class ImuPreintegrator {
 public:
  ImuPreintegrator(const Config& cfg, const gtsam::Vector3& gravity_world);

  // Reset internal state and set bias for new integration segment.
  void Reset(const gtsam::imuBias::ConstantBias& bias);

  // Integrate one IMU sample with explicit dt.
  void Integrate(const gtsam::Vector3& acc,
                 const gtsam::Vector3& gyro,
                 double dt);

  // Access the accumulated preintegrated measurements (for factor construction).
  const gtsam::PreintegratedCombinedMeasurements& Pim() const { return pim_; }

  // Access shared params (noise, gravity).
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
  Params() const { return params_; }

 private:
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params_;
  gtsam::PreintegratedCombinedMeasurements pim_;
};

// Linear interpolation of IMU samples to exact time t.
ImuSample InterpolateImu(const std::vector<ImuSample>& imu, double t);

// Integrate IMU samples between [t0, t1] into pim.
// Returns the updated index (first sample with time >= t1).
// Convention: each sample imu[i] covers interval (t_prev, imu[i].t];
// boundary samples at t0 and t1 are linearly interpolated.
size_t IntegrateBetween(const std::vector<ImuSample>& imu, size_t i_start,
                        double t0, double t1, ImuPreintegrator* pim);

}  // namespace uifgo
