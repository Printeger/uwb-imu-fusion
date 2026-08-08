#include "uifgo/trajectory_io.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>

#include <Eigen/SVD>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace uifgo {
using namespace gtsam;
using symbol_shorthand::A;
using symbol_shorthand::B;
using symbol_shorthand::L;
using symbol_shorthand::V;
using symbol_shorthand::X;
using symbol_shorthand::Z;

void TrajectoryIO::WriteTum(const std::string& path,
                            const std::vector<NavState>& traj) {
  std::ofstream f(path);
  if (!f) {
    std::cerr << "TrajectoryIO: cannot open " << path << "\n";
    return;
  }
  f << std::fixed << std::setprecision(9);
  for (const auto& s : traj) {
    auto q = s.T.rotation().toQuaternion();
    f << s.t << " " << s.T.translation().x() << " " << s.T.translation().y()
      << " " << s.T.translation().z() << " " << q.x() << " " << q.y() << " "
      << q.z() << " " << q.w() << "\n";
  }
  f.close();
  std::cout << "TrajectoryIO: wrote " << traj.size() << " poses to " << path
            << "\n";
}

void TrajectoryIO::WriteCalib(const std::string& path, const Values& values,
                              const std::vector<AnchorConfig>& anchors) {
  std::ofstream f(path);
  if (!f) {
    std::cerr << "TrajectoryIO: cannot open " << path << "\n";
    return;
  }
  f << std::fixed << std::setprecision(6);

  // Lever arm
  if (values.exists(L(0))) {
    Point3 lever = values.at<Point3>(L(0));
    f << "# Lever arm (UWB antenna in IMU frame)\n";
    f << "lever_arm: [" << lever.x() << ", " << lever.y() << ", " << lever.z()
      << "]\n\n";
  }

  // Anchor corrections
  f << "# Anchor position corrections (nominal + correction = estimated)\n";
  for (const auto& a : anchors) {
    Point3 corr(0, 0, 0);
    if (values.exists(A(a.id))) corr = values.at<Point3>(A(a.id));
    Point3 est = a.pos + corr;
    f << "anchor_" << a.id << ": nom=[" << a.pos.x() << "," << a.pos.y() << ","
      << a.pos.z() << "]"
      << " corr=[" << corr.x() << "," << corr.y() << "," << corr.z() << "]"
      << " est=[" << est.x() << "," << est.y() << "," << est.z() << "]\n";
  }

  // Range biases
  f << "\n# Range biases\n";
  for (const auto& a : anchors) {
    double beta = 0.0;
    if (values.exists(Z(a.id))) beta = values.at<double>(Z(a.id));
    f << "bias_" << a.id << ": " << beta << " m\n";
  }

  f.close();
  std::cout << "TrajectoryIO: wrote calibration to " << path << "\n";
}

std::vector<NavState> TrajectoryIO::ExtractTrajectory(
    const Values& values, const std::vector<double>& keyframe_times) {
  std::vector<NavState> traj;
  for (size_t k = 0; k < keyframe_times.size(); ++k) {
    Key xk = X(k);
    Key vk = V(k);
    Key bk = B(k);
    if (!values.exists(xk)) break;

    NavState s;
    s.t = keyframe_times[k];
    s.T = values.at<Pose3>(xk);
    s.v = values.exists(vk) ? values.at<Vector3>(vk) : Vector3::Zero();
    auto bias = values.exists(bk) ? values.at<imuBias::ConstantBias>(bk)
                                  : imuBias::ConstantBias();
    s.ba = bias.accelerometer();
    s.bg = bias.gyroscope();
    traj.push_back(s);
  }
  return traj;
}

// ============ ATE / RPE Evaluators ============

bool TrajectoryIO::AlignSE3(const std::vector<Eigen::Vector3d>& est_pts,
                            const std::vector<Eigen::Vector3d>& gt_pts,
                            Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out,
                            double* scale_out) {
  if (est_pts.empty() || gt_pts.empty() || est_pts.size() != gt_pts.size())
    return false;
  const size_t n = est_pts.size();
  if (n < 3) return false;  // need at least 3 points for rigid alignment

  // 1. Centroids
  Eigen::Vector3d c_est = Eigen::Vector3d::Zero();
  Eigen::Vector3d c_gt = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < n; ++i) {
    c_est += est_pts[i];
    c_gt += gt_pts[i];
  }
  c_est /= n;
  c_gt /= n;

  // 2. Cross-covariance matrix H = Σ (p_est_i - c_est) * (p_gt_i - c_gt)^T
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < n; ++i) {
    Eigen::Vector3d d_est = est_pts[i] - c_est;
    Eigen::Vector3d d_gt = gt_pts[i] - c_gt;
    H += d_est * d_gt.transpose();
  }

  // 3. SVD: H = U * S * V^T
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // 4. Rotation R = V * U^T  (with determinant check for reflection)
  Eigen::Matrix3d R = V * U.transpose();
  if (R.determinant() < 0.0) {
    // Reflection detected — flip sign of last column of V
    Eigen::Matrix3d Vf = V;
    Vf.col(2) *= -1.0;
    R = Vf * U.transpose();
  }

  // 5. Scale (forced to 1.0 for rigid SE(3))
  double scale = 1.0;

  // 6. Translation t = c_gt - scale * R * c_est
  Eigen::Vector3d t = c_gt - scale * R * c_est;

  if (R_out) *R_out = R;
  if (t_out) *t_out = t;
  if (scale_out) *scale_out = scale;
  return true;
}

ATEStats TrajectoryIO::ComputeATE(const std::vector<NavState>& est,
                                  const std::vector<NavState>& gt) {
  ATEStats stats;
  if (est.empty() || gt.empty()) return stats;

  // --- Match timestamps and collect position pairs ---
  std::vector<Eigen::Vector3d> est_pts, gt_pts;
  std::vector<size_t> est_idx;  // track which est frames were matched
  est_pts.reserve(est.size());
  gt_pts.reserve(est.size());
  est_idx.reserve(est.size());

  for (size_t i = 0; i < est.size(); ++i) {
    double t = est[i].t;
    auto it = std::lower_bound(
        gt.begin(), gt.end(), t,
        [](const NavState& g, double tval) { return g.t < tval; });
    if (it == gt.end()) continue;
    const NavState* g = nullptr;
    if (it == gt.begin()) {
      g = &(*it);
    } else {
      double d1 = it->t - t;
      double d0 = t - (it - 1)->t;
      g = (d0 < d1) ? &(*(it - 1)) : &(*it);
    }
    est_pts.push_back(Eigen::Vector3d(est[i].T.translation().x(),
                                      est[i].T.translation().y(),
                                      est[i].T.translation().z()));
    gt_pts.push_back(Eigen::Vector3d(g->T.translation().x(),
                                     g->T.translation().y(),
                                     g->T.translation().z()));
    est_idx.push_back(i);
  }

  if (est_pts.size() < 3) {
    std::cerr << "ComputeATE: too few matched poses (" << est_pts.size()
              << ") for alignment.\n";
    return stats;
  }

  // --- SE(3) Umeyama alignment ---
  Eigen::Matrix3d R_align;
  Eigen::Vector3d t_align;
  double scale;
  if (!AlignSE3(est_pts, gt_pts, &R_align, &t_align, &scale)) {
    std::cerr << "ComputeATE: SE(3) alignment failed.\n";
    return stats;
  }

  stats.scale = scale;
  stats.T_align = gtsam::Pose3(gtsam::Rot3(R_align), gtsam::Point3(t_align));
  stats.matched = est_pts.size();

  // --- Compute aligned errors ---
  std::vector<double> errors;
  errors.reserve(est_pts.size());
  double sum_err = 0.0, sum_err2 = 0.0;
  for (size_t i = 0; i < est_pts.size(); ++i) {
    Eigen::Vector3d aligned = scale * R_align * est_pts[i] + t_align;
    double err = (aligned - gt_pts[i]).norm();
    errors.push_back(err);
    sum_err += err;
    sum_err2 += err * err;
  }

  std::sort(errors.begin(), errors.end());
  size_t n = errors.size();
  stats.rmse = std::sqrt(sum_err2 / n);
  stats.mean_err = sum_err / n;
  stats.p50 = errors[n * 50 / 100];
  stats.p95 = errors[n * 95 / 100];
  stats.p99 = errors[n * 99 / 100];
  stats.max_err = errors.back();
  stats.ok = true;
  return stats;
}

double TrajectoryIO::ComputeRPE(const std::vector<NavState>& est,
                                const std::vector<NavState>& gt,
                                double segment_duration) {
  if (est.size() < 2 || gt.size() < 2) return -1.0;
  double sum_err2 = 0.0;
  size_t count = 0;
  for (size_t i = 0; i + 1 < est.size(); ++i) {
    double dt = est[i + 1].t - est[i].t;
    if (std::abs(dt - segment_duration) > 0.5 * segment_duration) continue;
    auto it_i =
        std::lower_bound(gt.begin(), gt.end(), est[i].t,
                         [](const NavState& g, double t) { return g.t < t; });
    auto it_j =
        std::lower_bound(gt.begin(), gt.end(), est[i + 1].t,
                         [](const NavState& g, double t) { return g.t < t; });
    if (it_i == gt.end() || it_j == gt.end()) continue;
    gtsam::Vector3 d_est = gtsam::Vector3(est[i + 1].T.translation()) -
                           gtsam::Vector3(est[i].T.translation());
    gtsam::Vector3 d_gt = gtsam::Vector3(it_j->T.translation()) -
                          gtsam::Vector3(it_i->T.translation());
    gtsam::Vector3 err = d_est - d_gt;
    sum_err2 += err.dot(err);
    ++count;
  }
  return (count > 0) ? std::sqrt(sum_err2 / count) : -1.0;
}

}  // namespace uifgo
