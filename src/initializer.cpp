#include "uifgo/initializer.h"
#include "uifgo/imu_preint.h"

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
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

  // --- 3. Yaw alignment via grid search over initial motion ---
  // Design doc §6.3: fix roll/pitch (from gravity) and p0 (from trilateration),
  // search yaw ∈ [0, 2π) minimizing IMU-predicted UWB range residuals.
  if (cfg_.yaw_align_frames > 0 && uwb_frames.size() >= 4) {
    size_t n_yaw_kf = std::min((size_t)cfg_.yaw_align_frames, uwb_frames.size());
    double best_yaw =
        AlignYaw(imu, uwb_frames, res.T0.rotation(),
                 gtsam::Point3(res.T0.translation()), n_yaw_kf);
    gtsam::Rot3 R_yaw = gtsam::Rot3::Rz(best_yaw);
    res.T0 = gtsam::Pose3(res.T0.rotation() * R_yaw, res.T0.translation());
    std::cout << "Initializer: yaw aligned = " << best_yaw * 180.0 / M_PI
              << " deg (using " << n_yaw_kf << " keyframes)\n";
  } else if (cfg_.yaw_align_frames <= 0) {
    std::cout << "Initializer: yaw alignment disabled (yaw_align_frames=0).\n";
  } else {
    std::cout << "Initializer: too few keyframes for yaw alignment, "
                 "keeping identity yaw.\n";
  }

  res.v0 = gtsam::Vector3::Zero();
  res.ok = true;
  return res;
}

// ---------------------------------------------------------------------------
double Initializer::AlignYaw(const std::vector<ImuSample>& imu,
                              const std::vector<UwbFrame>& uwb_frames,
                              const gtsam::Rot3& R_rp,
                              const gtsam::Point3& p0,
                              size_t num_keyframes) const {
  // --- Build anchor lookup ---
  std::unordered_map<int, gtsam::Point3> anchor_map;
  for (const auto& a : cfg_.anchors) anchor_map[a.id] = a.pos;

  // --- Preintegrate IMU between consecutive keyframes ---
  // Store preintegrated measurements for each interval [k-1, k]
  struct Seg {
    gtsam::PreintegratedCombinedMeasurements pim;
    double t0, t1;
  };
  std::vector<Seg> segs;
  {
    ImuPreintegrator pi(cfg_, gtsam::Vector3(0, 0, -cfg_.gravity));
    gtsam::imuBias::ConstantBias bias0(gtsam::Vector3::Zero(), gtsam::Vector3::Zero());
    size_t i_imu = 0;
    while (i_imu < imu.size() && imu[i_imu].t <= uwb_frames[0].t) ++i_imu;
    for (size_t k = 1; k < num_keyframes && k < uwb_frames.size(); ++k) {
      pi.Reset(bias0);
      i_imu = IntegrateBetween(imu, i_imu,
                                uwb_frames[k - 1].t, uwb_frames[k].t, &pi);
      segs.push_back({pi.Pim(), uwb_frames[k - 1].t, uwb_frames[k].t});
    }
  }
  if (segs.empty()) {
    std::cerr << "AlignYaw: no preintegration segments.\n";
    return 0.0;
  }

  // --- Helper: evaluate total UWB squared residual for a given yaw ---
  auto EvaluateYaw = [&](double yaw_rad) -> double {
    gtsam::Rot3 R0 = R_rp * gtsam::Rot3::Rz(yaw_rad);
    gtsam::Pose3 T_k(R0, p0);
    gtsam::Vector3 v_k = gtsam::Vector3::Zero();
    gtsam::imuBias::ConstantBias bias0(gtsam::Vector3::Zero(), gtsam::Vector3::Zero());
    double cost = 0.0;
    size_t n_ranges = 0;

    for (size_t k = 0; k <= segs.size(); ++k) {
      // Evaluate UWB ranges at this keyframe
      size_t kf_idx = k;  // k=0 is the first keyframe, k≥1 uses segs[k-1]
      if (kf_idx >= uwb_frames.size()) break;
      for (const auto& r : uwb_frames[kf_idx].ranges) {
        auto it = anchor_map.find(r.anchor_id);
        if (it == anchor_map.end()) continue;
        // Antenna position in world
        gtsam::Point3 ant = T_k.transformFrom(cfg_.lever_arm_init);
        double pred = (gtsam::Vector3(ant) - gtsam::Vector3(it->second)).norm();
        double err = pred - r.dist;
        cost += err * err;
        ++n_ranges;
      }

      // Propagate to next keyframe using IMU preintegration
      if (k < segs.size()) {
        gtsam::NavState ns_i(T_k, v_k);
        gtsam::NavState ns_j = segs[k].pim.predict(ns_i, bias0);
        T_k = ns_j.pose();
        v_k = ns_j.velocity();
      }
    }
    return (n_ranges > 0) ? cost : 1e12;
  };

  // --- Coarse grid search: 5° steps ---
  const double kCoarseStep = 5.0 * M_PI / 180.0;
  double best_yaw = 0.0, best_cost = 1e12;
  for (double yaw = 0.0; yaw < 2.0 * M_PI; yaw += kCoarseStep) {
    double c = EvaluateYaw(yaw);
    if (c < best_cost) { best_cost = c; best_yaw = yaw; }
  }

  // --- Local refinement: ±5° around best, 0.5° steps ---
  const double kFineRange = 5.0 * M_PI / 180.0;
  const double kFineStep  = 0.5 * M_PI / 180.0;
  double refined_yaw = best_yaw;
  double refined_cost = best_cost;
  for (double dy = -kFineRange; dy <= kFineRange; dy += kFineStep) {
    double yaw = best_yaw + dy;
    if (yaw < 0.0) yaw += 2.0 * M_PI;
    if (yaw >= 2.0 * M_PI) yaw -= 2.0 * M_PI;
    double c = EvaluateYaw(yaw);
    if (c < refined_cost) { refined_cost = c; refined_yaw = yaw; }
  }

  std::cout << "AlignYaw: coarse best=" << best_yaw * 180.0 / M_PI
            << " deg (cost=" << best_cost << "), refined="
            << refined_yaw * 180.0 / M_PI << " deg (cost="
            << refined_cost << ")\n";
  return refined_yaw;
}

}  // namespace uifgo
