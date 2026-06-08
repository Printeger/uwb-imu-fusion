#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Vector.h>
#include <vector>
#include <string>
#include <unordered_map>

namespace uifgo {

// --- Raw sensor samples ---

struct ImuSample {
  double t;                     // seconds
  Eigen::Vector3d acc;          // m/s^2, body frame
  Eigen::Vector3d gyro;         // rad/s, body frame
};

struct UwbRange {
  int    anchor_id;             // anchor identifier
  double dist;                  // measured range (m)
  double fp_rssi;               // first-path RSSI (dB)
  double rx_rssi;               // total received RSSI (dB)
};

struct UwbFrame {
  double t;                     // seconds (rosbag timestamp)
  int    tag_id;
  std::vector<UwbRange> ranges;
};

// --- Configuration ---

struct AnchorConfig {
  int id;
  gtsam::Point3 pos;            // nominal world-frame position
  double prior_sigma;           // prior std for anchor position correction (m)
};

// --- Per-keyframe output state ---

struct NavState {
  double t;
  gtsam::Pose3  T;              // body -> world
  gtsam::Vector3 v;             // world-frame velocity
  gtsam::Vector3 ba;            // accelerometer bias (body)
  gtsam::Vector3 bg;            // gyroscope bias (body)
};

}  // namespace uifgo
