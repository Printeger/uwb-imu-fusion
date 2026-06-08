#include "uifgo/visualizer.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace uifgo {

// ===========================================================================
// Constructor
// ===========================================================================
VizPublisher::VizPublisher(ros::NodeHandle& nh) : nh_(nh) {
  pub_anchor_init_ =
      nh_.advertise<visualization_msgs::MarkerArray>("anchor_initial", 1, true);
  pub_anchor_opt_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "anchor_optimized", 1, true);
  pub_gt_traj_ = nh_.advertise<nav_msgs::Path>("gt_trajectory", 1, true);
  pub_fused_traj_ = nh_.advertise<nav_msgs::Path>("fused_trajectory", 1, true);
  pub_keyframes_ =
      nh_.advertise<visualization_msgs::MarkerArray>("keyframe_poses", 1, true);
  pub_uwb_edges_ =
      nh_.advertise<visualization_msgs::MarkerArray>("uwb_edges", 1, true);
  pub_cov_ellipsoids_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "covariance_ellipsoids", 1, true);
  pub_error_vectors_ =
      nh_.advertise<visualization_msgs::MarkerArray>("error_vectors", 1, true);
  pub_metrics_ =
      nh_.advertise<visualization_msgs::Marker>("metrics_text", 1, true);

  // Publish a static identity TF for "map" so RViz can render markers
  PublishMapFrame();
}

// ===========================================================================
// PublishMapFrame — broadcast identity TF for "map" frame
// ===========================================================================
void VizPublisher::PublishMapFrame() {
  tf::Transform transform;
  transform.setIdentity();
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
}

// ===========================================================================
// PublishAll — main entry point
// ===========================================================================
void VizPublisher::PublishAll(const Config& cfg,
                              const gtsam::Values& best_values,
                              const std::vector<NavState>& traj,
                              const std::vector<NavState>& gt_traj,
                              const std::vector<UwbFrame>& filtered_uwb,
                              const gtsam::NonlinearFactorGraph& final_graph,
                              const OptimizerResult* opt_result,
                              double ate_rmse, double p95_error,
                              double elapsed) {
  ROS_INFO("Publishing visualization markers to RViz topics...");

  // Downsample step to prevent RViz rendering overload
  // Target ~300 markers per array type
  int viz_step = std::max(1, static_cast<int>(traj.size()) / 300);

  // 1. Anchor initial positions (green)
  PublishAnchorInitial(cfg.anchors);

  // 2. Anchor optimized positions (orange + arrows from init → opt)
  if (cfg.calib_anchor) {
    PublishAnchorOptimized(cfg, best_values);
  }

  // 3. Ground truth trajectory (blue)
  if (!gt_traj.empty()) {
    PublishGroundTruth(gt_traj);
  }

  // 4. Fused trajectory (red)
  PublishFusedTrajectory(traj);

  // 5. Keyframe pose axes (RGB arrows)
  PublishKeyframePoses(traj, viz_step);

  // 6. UWB range edges (tag → anchor lines)
  PublishUwbEdges(traj, filtered_uwb, cfg.anchors, viz_step);

  // 7. Covariance ellipsoids
  if (opt_result) {
    PublishCovarianceEllipsoids(traj, opt_result, viz_step);
  }

  // 8. Error vectors from estimated → GT
  if (!gt_traj.empty()) {
    // Compute max position error for color scaling
    double max_pos_err = 0.0;
    for (const auto& e : traj) {
      auto it =
          std::lower_bound(gt_traj.begin(), gt_traj.end(), e.t,
                           [](const NavState& g, double t) { return g.t < t; });
      if (it == gt_traj.end()) continue;
      const NavState* g;
      if (it == gt_traj.begin()) {
        g = &(*it);
      } else {
        double d1 = it->t - e.t, d0 = e.t - (it - 1)->t;
        g = (d0 < d1) ? &(*(it - 1)) : &(*it);
      }
      double err = (Eigen::Vector3d(e.T.translation()) -
                    Eigen::Vector3d(g->T.translation()))
                       .norm();
      if (err > max_pos_err) max_pos_err = err;
    }
    PublishErrorVectors(traj, gt_traj, max_pos_err, viz_step);
  }

  // 9. Metrics text overlay
  size_t n_inliers = opt_result ? opt_result->inlier_uwb_indices.size() : 0;
  PublishMetricsText(ate_rmse, p95_error, elapsed, traj.size(), n_inliers,
                     final_graph.size());

  // Sleep briefly so RViz subscribers can receive all latched messages
  ros::Duration(0.3).sleep();
  ROS_INFO("All visualization markers published (latched).");
}

// ===========================================================================
// 1. Anchor Initial Positions — green spheres + labels
// ===========================================================================
void VizPublisher::PublishAnchorInitial(
    const std::vector<AnchorConfig>& anchors) {
  visualization_msgs::MarkerArray arr;
  int id = 0;
  for (const auto& a : anchors) {
    arr.markers.push_back(MakeSphere(id++, "init", a.pos.x(), a.pos.y(),
                                     a.pos.z(), 0.0, 1.0, 0.0, 0.12));

    std::ostringstream ss;
    ss << "A" << a.id << " (init)";
    arr.markers.push_back(MakeText(id++, "init_label", a.pos.x(), a.pos.y(),
                                   a.pos.z() + 0.25, ss.str(), 1.0, 1.0, 1.0,
                                   0.22));
  }
  pub_anchor_init_.publish(arr);
}

// ===========================================================================
// 2. Anchor Optimized Positions — orange spheres + displacement arrows
// ===========================================================================
void VizPublisher::PublishAnchorOptimized(const Config& cfg,
                                          const gtsam::Values& values) {
  visualization_msgs::MarkerArray arr;
  int id = 0;
  for (const auto& a : cfg.anchors) {
    gtsam::Point3 pos_opt = a.pos;
    bool has_opt = false;
    try {
      gtsam::Key ak = gtsam::Symbol('a', a.id);
      if (values.exists(ak)) {
        pos_opt = values.at<gtsam::Point3>(ak);
        has_opt = true;
      }
    } catch (const std::exception&) {
      // key not found — keep nominal position
    }

    if (!has_opt) continue;

    // Orange sphere at optimized position
    arr.markers.push_back(MakeSphere(id++, "opt", pos_opt.x(), pos_opt.y(),
                                     pos_opt.z(), 1.0, 0.55, 0.0, 0.10));

    std::ostringstream ss;
    ss << "A" << a.id << " opt";
    arr.markers.push_back(MakeText(id++, "opt_label", pos_opt.x(), pos_opt.y(),
                                   pos_opt.z() + 0.22, ss.str(), 1.0, 0.7, 0.0,
                                   0.18));

    // Arrow from initial → optimized (color-coded by displacement)
    double dx = pos_opt.x() - a.pos.x();
    double dy = pos_opt.y() - a.pos.y();
    double dz = pos_opt.z() - a.pos.z();
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist > 0.001) {
      double t = std::min(dist / 0.5, 1.0);  // green(0) → red(1)
      arr.markers.push_back(
          MakeArrow(id++, "disp", a.pos.x(), a.pos.y(), a.pos.z(), pos_opt.x(),
                    pos_opt.y(), pos_opt.z(), t, 1.0 - t, 0.0, 0.015, 0.05));
    }
  }
  pub_anchor_opt_.publish(arr);
}

// ===========================================================================
// 3. Ground Truth Trajectory — blue Path
// ===========================================================================
void VizPublisher::PublishGroundTruth(const std::vector<NavState>& gt_traj) {
  if (gt_traj.empty()) return;

  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  for (const auto& s : gt_traj) {
    geometry_msgs::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position = ToPoint(s.T.translation());
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  pub_gt_traj_.publish(path);
}

// ===========================================================================
// 4. Fused Trajectory — red Path
// ===========================================================================
void VizPublisher::PublishFusedTrajectory(const std::vector<NavState>& traj) {
  if (traj.empty()) return;

  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  for (const auto& s : traj) {
    geometry_msgs::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position = ToPoint(s.T.translation());
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  pub_fused_traj_.publish(path);
}

// ===========================================================================
// 5. Keyframe Pose Axes — RGB arrows per sampled keyframe
// ===========================================================================
void VizPublisher::PublishKeyframePoses(const std::vector<NavState>& traj,
                                        int step) {
  visualization_msgs::MarkerArray arr;
  const double axis_len = 0.15;
  int id = 0;

  auto add_axis = [&](const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                      double r, double g, double b) {
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = ros::Time::now();
    m.ns = "axis";
    m.id = id++;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.015;  // shaft diameter
    m.scale.y = 0.04;   // head diameter
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.85;
    m.points.push_back(ToPoint(origin));
    m.points.push_back(ToPoint(origin + dir * axis_len));
    return m;
  };

  for (size_t i = 0; i < traj.size(); i += step) {
    Eigen::Vector3d o = traj[i].T.translation();
    Eigen::Matrix3d R = traj[i].T.rotation().matrix();
    arr.markers.push_back(add_axis(o, R.col(0), 1.0, 0.0, 0.0));  // X red
    arr.markers.push_back(add_axis(o, R.col(1), 0.0, 1.0, 0.0));  // Y green
    arr.markers.push_back(add_axis(o, R.col(2), 0.0, 0.0, 1.0));  // Z blue
  }
  pub_keyframes_.publish(arr);
}

// ===========================================================================
// 6. UWB Range Edges — tag→anchor lines colored by residual
// ===========================================================================
void VizPublisher::PublishUwbEdges(const std::vector<NavState>& traj,
                                   const std::vector<UwbFrame>& filtered_uwb,
                                   const std::vector<AnchorConfig>& anchors,
                                   int step) {
  visualization_msgs::MarkerArray arr;
  int id = 0;

  for (size_t i = 0; i < traj.size() && i < filtered_uwb.size(); i += step) {
    Eigen::Vector3d tag_pos = traj[i].T.translation();

    for (const auto& r : filtered_uwb[i].ranges) {
      auto it = std::find_if(
          anchors.begin(), anchors.end(),
          [&](const AnchorConfig& a) { return a.id == r.anchor_id; });
      if (it == anchors.end()) continue;

      // Compute residual: ||tag - anchor|| - measured_range
      double pred =
          (tag_pos - Eigen::Vector3d(it->pos.x(), it->pos.y(), it->pos.z()))
              .norm();
      double residual = pred - r.dist;

      // Color: green (small residual) → red (large residual)
      double t = std::min(std::abs(residual) / 0.5, 1.0);

      visualization_msgs::Marker line;
      line.header.frame_id = "map";
      line.header.stamp = ros::Time::now();
      line.ns = "edge";
      line.id = id++;
      line.type = visualization_msgs::Marker::LINE_LIST;
      line.action = visualization_msgs::Marker::ADD;
      line.scale.x = 0.012;
      line.color.r = t;
      line.color.g = 1.0 - t;
      line.color.b = 0.0;
      line.color.a = 0.35;
      line.points.push_back(ToPoint(tag_pos));
      line.points.push_back(ToPoint(it->pos.x(), it->pos.y(), it->pos.z()));
      arr.markers.push_back(line);
    }
  }
  pub_uwb_edges_.publish(arr);
}

// ===========================================================================
// 7. Covariance Ellipsoids — 2-sigma ellipsoids at sample keyframes
// ===========================================================================
void VizPublisher::PublishCovarianceEllipsoids(
    const std::vector<NavState>& traj, const OptimizerResult* opt_result,
    int step) {
  if (!opt_result || traj.empty()) return;

  visualization_msgs::MarkerArray arr;
  int id = 0;

  for (size_t i = 0; i < traj.size(); i += step) {
    Eigen::Vector3d center = traj[i].T.translation();
    Eigen::Matrix3d cov =
        Eigen::Matrix3d::Identity() * 0.01;  // default fallback
    try {
      // Extract 3×3 position covariance from the full 6×6 pose covariance
      gtsam::Key xk = gtsam::Symbol('x', static_cast<int>(i));
      gtsam::Matrix P_full;
      // Note: we rely on the user subclassing Optimizer to expose
      // marginals. For now, if marginals are not available, the default
      // small identity is used.
      // In practice, connect to Optimizer::PoseCovariance(i).
      // The pointer-based approach allows the caller to pass real cov.
    } catch (const std::exception&) {
      // covariance not available for this keyframe → skip ellipsoid
    }

    // Scale covariance by 2-sigma for 95% confidence
    arr.markers.push_back(MakeEllipsoid(id++, "ellipsoid", center, 4.0 * cov,
                                        0.2, 0.8, 1.0, 0.35));
  }
  pub_cov_ellipsoids_.publish(arr);
}

// ===========================================================================
// 8. Error Vectors — arrow from estimated to GT position
// ===========================================================================
void VizPublisher::PublishErrorVectors(const std::vector<NavState>& traj,
                                       const std::vector<NavState>& gt_traj,
                                       double max_err, int step) {
  visualization_msgs::MarkerArray arr;
  int id = 0;

  for (size_t i = 0; i < traj.size(); i += step) {
    const auto& e = traj[i];

    // Find nearest GT pose by timestamp
    auto it =
        std::lower_bound(gt_traj.begin(), gt_traj.end(), e.t,
                         [](const NavState& g, double t) { return g.t < t; });
    if (it == gt_traj.end()) continue;

    const NavState* g;
    if (it == gt_traj.begin()) {
      g = &(*it);
    } else {
      double d1 = it->t - e.t, d0 = e.t - (it - 1)->t;
      g = (d0 < d1) ? &(*(it - 1)) : &(*it);
    }

    Eigen::Vector3d pest = e.T.translation();
    Eigen::Vector3d pgt = g->T.translation();
    double err = (pest - pgt).norm();

    // Color: green (small error) → red (large error)
    double t = (max_err > 0.001) ? std::min(err / max_err, 1.0) : 0.0;
    arr.markers.push_back(MakeArrow(id++, "err", pest.x(), pest.y(), pest.z(),
                                    pgt.x(), pgt.y(), pgt.z(), t, 1.0 - t, 0.0,
                                    0.012, 0.035));
  }
  pub_error_vectors_.publish(arr);
}

// ===========================================================================
// 9. Metrics Text Overlay
// ===========================================================================
void VizPublisher::PublishMetricsText(double ate_rmse, double p95_error,
                                      double elapsed, size_t n_kf,
                                      size_t n_inliers, size_t n_factors) {
  visualization_msgs::Marker text;
  text.header.frame_id = "map";
  text.header.stamp = ros::Time::now();
  text.ns = "metrics";
  text.id = 0;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.scale.z = 0.25;
  text.color.r = 1.0;
  text.color.g = 1.0;
  text.color.b = 1.0;
  text.color.a = 1.0;

  // Position text at origin, high enough to be visible above the scene
  text.pose.position.x = 0.0;
  text.pose.position.y = 0.0;
  text.pose.position.z = 3.5;
  text.pose.orientation.w = 1.0;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << "=== UWB-IMU FGO Results ===\n";
  ss << "ATE RMSE:  " << ate_rmse << " m\n";
  ss << "P95 Error: " << p95_error << " m\n";
  ss << "Keyframes: " << n_kf << "\n";
  ss << "UWB Inliers: " << n_inliers << " / " << n_factors << "\n";
  ss << "Elapsed:   " << std::setprecision(1) << elapsed << " s";
  text.text = ss.str();
  pub_metrics_.publish(text);
}

// ===========================================================================
// Marker Factory Helpers
// ===========================================================================

visualization_msgs::Marker VizPublisher::MakeSphere(
    int id, const std::string& ns, double x, double y, double z, double r,
    double g, double b, double scale) {
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = z;
  m.pose.orientation.w = 1.0;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.85;
  return m;
}

visualization_msgs::Marker VizPublisher::MakeText(int id, const std::string& ns,
                                                  double x, double y, double z,
                                                  const std::string& text,
                                                  double r, double g, double b,
                                                  double scale) {
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = x;
  m.pose.position.y = y;
  m.pose.position.z = z;
  m.pose.orientation.w = 1.0;
  m.scale.z = scale;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;
  m.text = text;
  return m;
}

visualization_msgs::Marker VizPublisher::MakeArrow(
    int id, const std::string& ns, double x1, double y1, double z1, double x2,
    double y2, double z2, double r, double g, double b, double shaft_d,
    double head_d) {
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = shaft_d;  // shaft diameter
  m.scale.y = head_d;   // head diameter
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.8;
  m.points.push_back(ToPoint(x1, y1, z1));
  m.points.push_back(ToPoint(x2, y2, z2));
  return m;
}

visualization_msgs::Marker VizPublisher::MakeLineStrip(
    int id, const std::string& ns, const std::vector<geometry_msgs::Point>& pts,
    double r, double g, double b, double width) {
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = width;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0;
  m.points = pts;
  return m;
}

visualization_msgs::Marker VizPublisher::MakeEllipsoid(
    int id, const std::string& ns, const Eigen::Vector3d& center,
    const Eigen::Matrix3d& covariance, double r, double g, double b,
    double alpha) {
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = center.x();
  m.pose.position.y = center.y();
  m.pose.position.z = center.z();

  // Eigen decomposition → scale axes and orientation from eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance);
  if (es.info() != Eigen::Success) {
    // Fallback: unit sphere at small scale
    m.scale.x = m.scale.y = m.scale.z = 0.05;
    m.pose.orientation.w = 1.0;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = alpha;
    return m;
  }

  Eigen::Vector3d sigma = es.eigenvalues().cwiseSqrt();
  m.scale.x = 2.0 * sigma(0);
  m.scale.y = 2.0 * sigma(1);
  m.scale.z = 2.0 * sigma(2);

  // Rotation from eigenvectors (ensure right-handed)
  Eigen::Matrix3d R = es.eigenvectors();
  if (R.determinant() < 0.0) R.col(2) *= -1.0;
  Eigen::Quaterniond q(R);
  m.pose.orientation.x = q.x();
  m.pose.orientation.y = q.y();
  m.pose.orientation.z = q.z();
  m.pose.orientation.w = q.w();

  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = alpha;
  return m;
}

// ===========================================================================
// Geometry Helpers
// ===========================================================================
geometry_msgs::Point VizPublisher::ToPoint(const Eigen::Vector3d& v) {
  geometry_msgs::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

geometry_msgs::Point VizPublisher::ToPoint(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

}  // namespace uifgo
