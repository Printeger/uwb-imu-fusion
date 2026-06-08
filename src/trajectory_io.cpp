#include "uifgo/trajectory_io.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace uifgo {
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::L;
using symbol_shorthand::A;
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
    f << s.t << " "
      << s.T.translation().x() << " "
      << s.T.translation().y() << " "
      << s.T.translation().z() << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  f.close();
  std::cout << "TrajectoryIO: wrote " << traj.size() << " poses to " << path << "\n";
}

void TrajectoryIO::WriteCalib(const std::string& path,
                               const Values& values,
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
    f << "lever_arm: [" << lever.x() << ", " << lever.y() << ", " << lever.z() << "]\n\n";
  }

  // Anchor corrections
  f << "# Anchor position corrections (nominal + correction = estimated)\n";
  for (const auto& a : anchors) {
    Point3 corr(0,0,0);
    if (values.exists(A(a.id))) corr = values.at<Point3>(A(a.id));
    Point3 est = a.pos + corr;
    f << "anchor_" << a.id << ": nom=[" << a.pos.x() << "," << a.pos.y() << "," << a.pos.z() << "]"
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
    const Values& values,
    const std::vector<double>& keyframe_times) {
  std::vector<NavState> traj;
  for (size_t k = 0; k < keyframe_times.size(); ++k) {
    Key xk = X(k);
    Key vk = V(k);
    Key bk = B(k);
    if (!values.exists(xk)) break;

    NavState s;
    s.t  = keyframe_times[k];
    s.T  = values.at<Pose3>(xk);
    s.v  = values.exists(vk) ? values.at<Vector3>(vk) : Vector3::Zero();
    auto bias = values.exists(bk) ? values.at<imuBias::ConstantBias>(bk)
                                  : imuBias::ConstantBias();
    s.ba = bias.accelerometer();
    s.bg = bias.gyroscope();
    traj.push_back(s);
  }
  return traj;
}

// ============ ATE / RPE Evaluators ============

double TrajectoryIO::ComputeATE(const std::vector<NavState>& est,
                                 const std::vector<NavState>& gt) {
  if (est.empty() || gt.empty()) return -1.0;
  double sum_err2 = 0.0;
  size_t count = 0;
  for (const auto& e : est) {
    auto it = std::lower_bound(gt.begin(), gt.end(), e.t,
        [](const NavState& g, double t) { return g.t < t; });
    if (it == gt.end()) continue;
    const NavState* g = nullptr;
    if (it == gt.begin()) g = &(*it);
    else {
      double d1 = it->t - e.t;
      double d0 = e.t - (it-1)->t;
      g = (d0 < d1) ? &(*(it-1)) : &(*it);
    }
    gtsam::Vector3 diff = gtsam::Vector3(e.T.translation()) - gtsam::Vector3(g->T.translation());
    sum_err2 += diff.dot(diff);
    ++count;
  }
  return (count > 0) ? std::sqrt(sum_err2 / count) : -1.0;
}

double TrajectoryIO::ComputeRPE(const std::vector<NavState>& est,
                                 const std::vector<NavState>& gt,
                                 double segment_duration) {
  if (est.size() < 2 || gt.size() < 2) return -1.0;
  double sum_err2 = 0.0;
  size_t count = 0;
  for (size_t i = 0; i + 1 < est.size(); ++i) {
    double dt = est[i+1].t - est[i].t;
    if (std::abs(dt - segment_duration) > 0.5 * segment_duration) continue;
    auto it_i = std::lower_bound(gt.begin(), gt.end(), est[i].t,
        [](const NavState& g, double t) { return g.t < t; });
    auto it_j = std::lower_bound(gt.begin(), gt.end(), est[i+1].t,
        [](const NavState& g, double t) { return g.t < t; });
    if (it_i == gt.end() || it_j == gt.end()) continue;
    gtsam::Vector3 d_est = gtsam::Vector3(est[i+1].T.translation()) - gtsam::Vector3(est[i].T.translation());
    gtsam::Vector3 d_gt  = gtsam::Vector3(it_j->T.translation()) - gtsam::Vector3(it_i->T.translation());
    gtsam::Vector3 err   = d_est - d_gt;
    sum_err2 += err.dot(err);
    ++count;
  }
  return (count > 0) ? std::sqrt(sum_err2 / count) : -1.0;
}

}  // namespace uifgo
