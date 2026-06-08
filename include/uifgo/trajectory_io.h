#pragma once

#include "uifgo/types.h"
#include <gtsam/nonlinear/Values.h>
#include <string>
#include <vector>

namespace uifgo {

class TrajectoryIO {
 public:
  // Write trajectory in TUM format: timestamp tx ty tz qx qy qz qw
  static void WriteTum(const std::string& path,
                       const std::vector<NavState>& traj);

  // Write calibration report (lever arm, anchor corrections, biases).
  static void WriteCalib(const std::string& path,
                         const gtsam::Values& values,
                         const std::vector<AnchorConfig>& anchors);

  // Extract NavState vector from GTSAM Values.
  static std::vector<NavState> ExtractTrajectory(
      const gtsam::Values& values,
      const std::vector<double>& keyframe_times);

  // Evaluate: Absolute Trajectory Error (RMSE after Umeyama alignment).
  static double ComputeATE(const std::vector<NavState>& est,
                           const std::vector<NavState>& gt);

  // Evaluate: Relative Pose Error per segment.
  static double ComputeRPE(const std::vector<NavState>& est,
                           const std::vector<NavState>& gt,
                           double segment_duration = 1.0);
};

}  // namespace uifgo
