#pragma once

#include <gtsam/nonlinear/Values.h>

#include <string>
#include <vector>

#include "uifgo/types.h"

namespace uifgo {

class TrajectoryIO {
 public:
  // Write trajectory in TUM format: timestamp tx ty tz qx qy qz qw
  static void WriteTum(const std::string& path,
                       const std::vector<NavState>& traj);

  // Write calibration report (lever arm, anchor corrections, biases).
  static void WriteCalib(const std::string& path, const gtsam::Values& values,
                         const std::vector<AnchorConfig>& anchors);

  // Extract NavState vector from GTSAM Values.
  static std::vector<NavState> ExtractTrajectory(
      const gtsam::Values& values, const std::vector<double>& keyframe_times);

  // Evaluate: Absolute Trajectory Error (RMSE after SE(3) Umeyama alignment).
  // The alignment finds the optimal rigid transform T_align that maps the
  // estimated trajectory into the GT frame.  ATE is then computed on the
  // aligned positions.  Returns ATEStats with rmse, percentiles, etc.
  static ATEStats ComputeATE(const std::vector<NavState>& est,
                             const std::vector<NavState>& gt);

  // SE(3) Umeyama alignment: finds R, t, scale that minimizes
  //   Σ || (s*R*p_est_i + t) - p_gt_i ||²
  // Returns true on success.  Scale is forced to 1.0 for rigid alignment.
  static bool AlignSE3(const std::vector<Eigen::Vector3d>& est_pts,
                       const std::vector<Eigen::Vector3d>& gt_pts,
                       Eigen::Matrix3d* R_out, Eigen::Vector3d* t_out,
                       double* scale_out);

  // Evaluate: Relative Pose Error per segment.
  static double ComputeRPE(const std::vector<NavState>& est,
                           const std::vector<NavState>& gt,
                           double segment_duration = 1.0);
};

}  // namespace uifgo
