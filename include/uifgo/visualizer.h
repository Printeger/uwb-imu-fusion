#pragma once

#include <geometry_msgs/Point.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

#include "uifgo/config.h"
#include "uifgo/optimizer.h"
#include "uifgo/types.h"

namespace uifgo {

/// Publishes visualization markers for RViz after batch optimization completes.
///
/// Call VizPublisher::PublishAll(...) after optimization and GT evaluation
/// are finished. All markers are published as latched topics so RViz can
/// receive them even if it starts after the node finishes processing.
class VizPublisher {
 public:
  explicit VizPublisher(ros::NodeHandle& nh);

  /// Publish ALL visualization markers at once.
  void PublishAll(const Config& cfg, const gtsam::Values& best_values,
                  const std::vector<NavState>& traj,
                  const std::vector<NavState>& gt_traj,
                  const std::vector<UwbFrame>& filtered_uwb,
                  const gtsam::NonlinearFactorGraph& final_graph,
                  const OptimizerResult* opt_result = nullptr,
                  double ate_rmse = -1.0, double p95_error = -1.0,
                  double elapsed = -1.0);

  /// Broadcast a static identity TF for the "map" frame.
  /// RViz requires a valid TF tree to display markers in a given frame.
  /// Call this periodically (or after PublishAll) to keep the TF alive.
  void PublishMapFrame();

 private:
  // --- Individual visualization publishers ---

  /// Anchor initial positions → green spheres + text labels.
  void PublishAnchorInitial(const std::vector<AnchorConfig>& anchors);

  /// Anchor optimized positions → orange spheres + displacement arrows.
  void PublishAnchorOptimized(const Config& cfg, const gtsam::Values& values);

  /// Ground truth trajectory → blue Path.
  void PublishGroundTruth(const std::vector<NavState>& gt_traj);

  /// Fused (estimated) trajectory → red Path.
  void PublishFusedTrajectory(const std::vector<NavState>& traj);

  /// Coordinate axes (RGB arrows) at downsampled keyframes.
  void PublishKeyframePoses(const std::vector<NavState>& traj, int step);

  /// UWB range measurement edges (tag → anchor lines).
  void PublishUwbEdges(const std::vector<NavState>& traj,
                       const std::vector<UwbFrame>& filtered_uwb,
                       const std::vector<AnchorConfig>& anchors, int step);

  /// Covariance ellipsoids at sampled keyframes (2-sigma).
  void PublishCovarianceEllipsoids(const std::vector<NavState>& traj,
                                   const OptimizerResult* opt_result, int step);

  /// Error vectors from estimated → ground truth positions.
  void PublishErrorVectors(const std::vector<NavState>& traj,
                           const std::vector<NavState>& gt_traj, double max_err,
                           int step);

  /// Text overlay with ATE, P95, elapsed time, etc.
  void PublishMetricsText(double ate_rmse, double p95_error, double elapsed,
                          size_t n_kf, size_t n_inliers, size_t n_factors);

  // --- Marker factory helpers ---

  visualization_msgs::Marker MakeSphere(int id, const std::string& ns, double x,
                                        double y, double z, double r, double g,
                                        double b, double scale = 0.15);

  visualization_msgs::Marker MakeText(int id, const std::string& ns, double x,
                                      double y, double z,
                                      const std::string& text, double r = 1.0,
                                      double g = 1.0, double b = 1.0,
                                      double scale = 0.3);

  visualization_msgs::Marker MakeArrow(int id, const std::string& ns, double x1,
                                       double y1, double z1, double x2,
                                       double y2, double z2, double r, double g,
                                       double b, double shaft_d = 0.03,
                                       double head_d = 0.08);

  visualization_msgs::Marker MakeLineStrip(
      int id, const std::string& ns,
      const std::vector<geometry_msgs::Point>& pts, double r, double g,
      double b, double width = 0.02);

  visualization_msgs::Marker MakeEllipsoid(int id, const std::string& ns,
                                           const Eigen::Vector3d& center,
                                           const Eigen::Matrix3d& covariance,
                                           double r, double g, double b,
                                           double alpha = 0.5);

  // --- Geometry helpers ---

  static geometry_msgs::Point ToPoint(const Eigen::Vector3d& v);
  static geometry_msgs::Point ToPoint(double x, double y, double z);

  // --- ROS publishers (all latched) ---

  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher pub_anchor_init_;
  ros::Publisher pub_anchor_opt_;
  ros::Publisher pub_gt_traj_;
  ros::Publisher pub_fused_traj_;
  ros::Publisher pub_keyframes_;
  ros::Publisher pub_uwb_edges_;
  ros::Publisher pub_cov_ellipsoids_;
  ros::Publisher pub_error_vectors_;
  ros::Publisher pub_metrics_;
};

}  // namespace uifgo
