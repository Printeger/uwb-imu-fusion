#include "uifgo/imu_preint.h"
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/base/numericalDerivative.h>

namespace uifgo {

ImuPreintegrator::ImuPreintegrator(const Config& cfg,
                                     const gtsam::Vector3& gravity_world) {
  auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(gravity_world.z());
  p->accelerometerCovariance =
      Eigen::Matrix3d::Identity() * cfg.sigma_a * cfg.sigma_a;
  p->gyroscopeCovariance =
      Eigen::Matrix3d::Identity() * cfg.sigma_g * cfg.sigma_g;
  p->integrationCovariance =
      Eigen::Matrix3d::Identity() * 1e-9;  // small integration noise
  p->biasAccCovariance =
      Eigen::Matrix3d::Identity() * cfg.sigma_wa * cfg.sigma_wa;
  p->biasOmegaCovariance =
      Eigen::Matrix3d::Identity() * cfg.sigma_wg * cfg.sigma_wg;
  params_ = p;
  pim_    = gtsam::PreintegratedCombinedMeasurements(params_);
}

void ImuPreintegrator::Reset(const gtsam::imuBias::ConstantBias& bias) {
  pim_.resetIntegrationAndSetBias(bias);
}

void ImuPreintegrator::Integrate(const gtsam::Vector3& acc,
                                  const gtsam::Vector3& gyro,
                                  double dt) {
  pim_.integrateMeasurement(acc, gyro, dt);
}

ImuSample InterpolateImu(const std::vector<ImuSample>& imu, double t) {
  if (imu.empty()) return {t, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  if (t <= imu.front().t) return imu.front();
  if (t >= imu.back().t)  return imu.back();

  auto it = std::lower_bound(imu.begin(), imu.end(), t,
      [](const ImuSample& s, double v) { return s.t < v; });
  if (it == imu.begin()) return *it;
  if (it == imu.end())   return imu.back();

  const ImuSample& s1 = *it;
  const ImuSample& s0 = *(it - 1);
  double dt = s1.t - s0.t;
  if (dt < 1e-12) return s0;

  double r = (t - s0.t) / dt;
  ImuSample out;
  out.t    = t;
  out.acc  = s0.acc  + r * (s1.acc  - s0.acc);
  out.gyro = s0.gyro + r * (s1.gyro - s0.gyro);
  return out;
}

size_t IntegrateBetween(const std::vector<ImuSample>& imu, size_t i_start,
                        double t0, double t1, ImuPreintegrator* pim) {
  size_t i = i_start;
  // Advance to first sample after t0
  while (i < imu.size() && imu[i].t <= t0) ++i;

  double t_prev = t0;
  while (i < imu.size() && imu[i].t < t1) {
    double dt = imu[i].t - t_prev;
    if (dt > 1e-9) pim->Integrate(imu[i].acc, imu[i].gyro, dt);
    t_prev = imu[i].t;
    ++i;
  }

  // Final segment: interpolate to t1
  ImuSample s_end = InterpolateImu(imu, t1);
  double dt = t1 - t_prev;
  if (dt > 1e-9) pim->Integrate(s_end.acc, s_end.gyro, dt);

  return i;
}

}  // namespace uifgo
