#include "uifgo/initializer.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace uifgo {

Initializer::Initializer(const Config& cfg) : cfg_(cfg) {}

bool Initializer::DetectStatic(const std::vector<ImuSample>& imu, size_t i0,
                               size_t i1, double* accel_norm_mean) {
  if (i1 <= i0 || i1 > imu.size()) return false;
  size_t n = i1 - i0;
  double sum_norm = 0.0, sum_norm2 = 0.0;
  for (size_t i = i0; i < i1; ++i) {
    double norm = imu[i].acc.norm();
    sum_norm += norm;
    sum_norm2 += norm * norm;
  }
  double mean = sum_norm / n;
  double var = sum_norm2 / n - mean * mean;
  if (accel_norm_mean) *accel_norm_mean = mean;

  // Static if accelerometer norm variance is very small
  // and mean is close to gravity
  static const double kStaticVarThresh = 0.01;  // (m/s^2)^2
  static const double kGravityTol = 1.0;        // m/s^2
  return (var < kStaticVarThresh) &&
         (std::abs(mean - cfg_.gravity) < kGravityTol);
}

bool Initializer::Trilaterate(const std::vector<UwbRange>& ranges,
                              const std::vector<AnchorConfig>& anchors,
                              gtsam::Point3* p_out) {
  // Build anchor lookup
  std::unordered_map<int, gtsam::Point3> anchor_map;
  for (const auto& a : anchors) anchor_map[a.id] = a.pos;

  // Collect valid anchor-range pairs
  std::vector<gtsam::Point3> A;
  std::vector<double> r;
  for (const auto& range : ranges) {
    auto it = anchor_map.find(range.anchor_id);
    if (it == anchor_map.end()) continue;
    A.push_back(it->second);
    r.push_back(range.dist);
  }

  if (A.size() < 3) {
    std::cerr << "Trilaterate: need >= 3 anchors, got " << A.size() << "\n";
    return false;
  }

  // Initial guess: centroid of anchors
  gtsam::Point3 p(0, 0, 0);
  for (const auto& a : A) p = p + gtsam::Point3(gtsam::Vector3(a) / A.size());

  // Gauss-Newton: minimize sum (||A_i - p|| - r_i)^2
  for (int iter = 0; iter < 30; ++iter) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < A.size(); ++i) {
      Eigen::Vector3d d = gtsam::Vector3(p) - gtsam::Vector3(A[i]);
      double dn = d.norm();
      if (dn < 1e-9) continue;
      Eigen::Vector3d u = d / dn;  // unit vector from anchor to p
      double e = dn - r[i];        // residual
      H += u * u.transpose();
      b -= u * e;
    }

    Eigen::Vector3d dp = H.ldlt().solve(b);
    p = gtsam::Point3(gtsam::Vector3(p) + dp);
    if (dp.norm() < 1e-6) break;
  }

  *p_out = p;
  return true;
}

InitResult Initializer::Run(const std::vector<ImuSample>& imu,
                            const std::vector<UwbFrame>& uwb_frames) {
  InitResult res;

  if (imu.empty() || uwb_frames.empty()) {
    res.ok = false;
    return res;
  }

  // --- 1. Static detection on first 2 seconds of IMU ---
  double t0 = imu[0].t;
  size_t i0 = 0, i1 = 0;
  double static_dur = 2.0;
  for (size_t i = 0; i < imu.size(); ++i) {
    if (imu[i].t - t0 > static_dur) {
      i1 = i;
      break;
    }
  }
  if (i1 == 0) i1 = std::min(imu.size(), (size_t)200);

  bool is_static = DetectStatic(imu, i0, i1);
  if (is_static) {
    // Average accel = gravity direction in body frame
    Eigen::Vector3d acc_avg = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_avg = Eigen::Vector3d::Zero();
    for (size_t i = i0; i < i1; ++i) {
      acc_avg += imu[i].acc;
      gyro_avg += imu[i].gyro;
    }
    acc_avg /= (i1 - i0);
    gyro_avg /= (i1 - i0);

    // Diagnostic: print mean accel norm to verify IMU units (should be ~9.81)
    std::cout
        << "Initializer: static acc norm = " << acc_avg.norm()
        << " m/s^2 (expect ~9.81; if ~1.0, IMU data is gravity-normalized)\n";

    res.ba0 = gtsam::Vector3::Zero();  // small bias, refined later
    res.bg0 = gyro_avg;

    // Align gravity direction to get roll/pitch.
    // Key insight: the accelerometer measures PROPER acceleration (reaction
    // force opposing gravity). When static on a table, acc_avg points UP
    // (sky direction).  The world-frame gravity g_world points DOWN.
    //
    // We want: R_wb * (body gravity direction) = (world gravity direction).
    // Body gravity direction = -acc_avg (pointing DOWN in body frame).
    // World gravity direction = (0,0,-g) (pointing DOWN in world frame).
    Eigen::Vector3d g_body =
        -acc_avg.normalized() * cfg_.gravity;      // DOWN in body
    Eigen::Vector3d g_world(0, 0, -cfg_.gravity);  // DOWN in world
    // Find rotation that aligns g_body to g_world
    Eigen::Vector3d axis = g_body.cross(g_world);
    double axis_norm = axis.norm();
    double cos_angle = g_body.dot(g_world) / (g_body.norm() * g_world.norm());
    gtsam::Rot3 R_wb;
    if (axis_norm < 1e-9) {
      // Parallel or anti-parallel
      if (cos_angle > 0) {
        R_wb = gtsam::Rot3::identity();  // already aligned
      } else {
        // 180° about any perpendicular axis (e.g. X)
        R_wb = gtsam::Rot3::Rx(M_PI);
      }
    } else {
      axis.normalize();
      double angle = std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
      R_wb = gtsam::Rot3::AxisAngle(gtsam::Unit3(axis), angle);
    }
    res.T0 = gtsam::Pose3(R_wb, gtsam::Point3(0, 0, 0));
    res.gravity_world = gtsam::Vector3(0, 0, -cfg_.gravity);
    std::cout << "Initializer: static detected, aligned gravity. bg0=["
              << res.bg0.transpose() << "]\n";
  } else {
    std::cout << "Initializer: no static interval found, using identity "
                 "orientation.\n";
    res.T0 = gtsam::Pose3();
    res.ba0 = gtsam::Vector3::Zero();
    res.bg0 = gtsam::Vector3::Zero();
    res.gravity_world = gtsam::Vector3(0, 0, -cfg_.gravity);
  }

  // --- 2. UWB trilateration for initial position ---
  if (!uwb_frames.empty()) {
    gtsam::Point3 p0;
    if (Trilaterate(uwb_frames[0].ranges, cfg_.anchors, &p0)) {
      res.T0 = gtsam::Pose3(res.T0.rotation(), p0);
      std::cout << "Initializer: trilateration success, p0="
                << gtsam::Vector3(p0).transpose() << "\n";
    } else {
      std::cerr << "Initializer: trilateration failed, using origin.\n";
    }
  }

  // --- 3. Yaw alignment (simplified: skip if no motion data) ---
  // A proper yaw alignment needs a short trajectory segment; for now, keep
  // identity yaw.

  res.v0 = gtsam::Vector3::Zero();
  res.ok = true;
  return res;
}

}  // namespace uifgo
