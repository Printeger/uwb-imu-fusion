#include "uifgo/imu_preint.h"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/PreintegrationParams.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uifgo {

ImuPreintegrator::ImuPreintegrator(const Config& cfg,
                                   const gtsam::Vector3& gravity_world) {
  if (!gravity_world.allFinite()) {
    throw std::invalid_argument("world gravity must be finite");
  }
  auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(
      gravity_world.norm());
  p->n_gravity = gravity_world;
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
  pim_ = gtsam::PreintegratedCombinedMeasurements(params_);
}

void ImuPreintegrator::Reset(const gtsam::imuBias::ConstantBias& bias) {
  pim_.resetIntegrationAndSetBias(bias);
}

void ImuPreintegrator::Integrate(const gtsam::Vector3& acc,
                                 const gtsam::Vector3& gyro, double dt) {
  if (!acc.allFinite() || !gyro.allFinite()) {
    throw std::invalid_argument("IMU sample contains a non-finite value");
  }
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("IMU integration dt must be finite and > 0");
  }
  pim_.integrateMeasurement(acc, gyro, dt);
}

ImuSample InterpolateImu(const std::vector<ImuSample>& imu, double t) {
  if (imu.empty()) return {t, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  if (t <= imu.front().t) return imu.front();
  if (t >= imu.back().t) return imu.back();

  auto it =
      std::lower_bound(imu.begin(), imu.end(), t,
                       [](const ImuSample& s, double v) { return s.t < v; });
  if (it == imu.begin()) return *it;
  if (it == imu.end()) return imu.back();

  const ImuSample& s1 = *it;
  const ImuSample& s0 = *(it - 1);
  double dt = s1.t - s0.t;
  if (dt < 1e-12) return s0;

  double r = (t - s0.t) / dt;
  ImuSample out;
  out.t = t;
  out.acc = s0.acc + r * (s1.acc - s0.acc);
  out.gyro = s0.gyro + r * (s1.gyro - s0.gyro);
  return out;
}

size_t IntegrateBetween(const std::vector<ImuSample>& imu, size_t i_start,
                        double t0, double t1, ImuPreintegrator* pim,
                        double max_gap_s) {
  if (pim == nullptr) throw std::invalid_argument("IMU preintegrator is null");
  if (!std::isfinite(t0) || !std::isfinite(t1) || t1 <= t0) {
    throw std::invalid_argument("IMU integration interval must satisfy t1 > t0");
  }
  if (!std::isfinite(max_gap_s) || max_gap_s <= 0.0) {
    throw std::invalid_argument("IMU max gap must be finite and > 0");
  }
  if (imu.size() < 2 || imu.front().t > t0 || imu.back().t < t1) {
    throw std::invalid_argument("IMU samples do not cover the integration interval");
  }
  for (std::size_t i = 0; i < imu.size(); ++i) {
    if (!std::isfinite(imu[i].t) || !imu[i].acc.allFinite() ||
        !imu[i].gyro.allFinite()) {
      throw std::invalid_argument("IMU sequence contains non-finite values");
    }
    if (i > 0 && !(imu[i - 1].t < imu[i].t)) {
      throw std::invalid_argument("IMU sequence must be strictly ordered");
    }
    if (i > 0 && imu[i - 1].t < t1 && imu[i].t > t0 &&
        imu[i].t - imu[i - 1].t > max_gap_s + 1e-12) {
      throw std::runtime_error("IMU gap exceeds configured maximum");
    }
  }

  auto sample_at = [&imu](double time) {
    auto upper = std::lower_bound(
        imu.begin(), imu.end(), time,
        [](const ImuSample& sample, double value) { return sample.t < value; });
    if (upper != imu.end() && std::abs(upper->t - time) <= 1e-12) {
      ImuSample exact = *upper;
      exact.t = time;
      return exact;
    }
    if (upper == imu.begin() || upper == imu.end()) {
      throw std::invalid_argument("IMU boundary interpolation is not covered");
    }
    const ImuSample& right = *upper;
    const ImuSample& left = *(upper - 1);
    const double ratio = (time - left.t) / (right.t - left.t);
    return ImuSample{time,
                     left.acc + ratio * (right.acc - left.acc),
                     left.gyro + ratio * (right.gyro - left.gyro)};
  };

  std::vector<ImuSample> knots;
  knots.push_back(sample_at(t0));
  auto first = std::upper_bound(
      imu.begin(), imu.end(), t0,
      [](double value, const ImuSample& sample) { return value < sample.t; });
  for (auto it = first; it != imu.end() && it->t < t1; ++it) {
    knots.push_back(*it);
  }
  knots.push_back(sample_at(t1));
  for (std::size_t i = 1; i < knots.size(); ++i) {
    const double dt = knots[i].t - knots[i - 1].t;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > max_gap_s + 1e-12) {
      throw std::runtime_error("invalid IMU integration segment");
    }
    pim->Integrate(0.5 * (knots[i - 1].acc + knots[i].acc),
                   0.5 * (knots[i - 1].gyro + knots[i].gyro), dt);
  }
  (void)i_start;
  return static_cast<std::size_t>(std::distance(
      imu.begin(), std::lower_bound(
                       imu.begin(), imu.end(), t1,
                       [](const ImuSample& sample, double value) {
                         return sample.t < value;
                       })));
}

}  // namespace uifgo
