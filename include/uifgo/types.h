#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <unordered_map>
#include <vector>

namespace uifgo {

// --- Raw sensor samples ---

struct ImuSample {
  double t;              // seconds
  Eigen::Vector3d acc;   // m/s^2, body frame
  Eigen::Vector3d gyro;  // rad/s, body frame
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  bool has_orientation = false;
};

struct UwbRange {
  int anchor_id;   // anchor identifier
  double dist;     // measured range (m)
  double fp_rssi;  // first-path RSSI (dB)
  double rx_rssi;  // total received RSSI (dB)
};

struct UwbFrame {
  double t;  // seconds (rosbag timestamp)
  int tag_id;
  std::vector<UwbRange> ranges;
};

// --- Configuration ---

struct AnchorConfig {
  int id;
  gtsam::Point3 pos;   // nominal world-frame position
  double prior_sigma;  // prior std for anchor position correction (m)
};

// --- Per-keyframe output state ---

struct NavState {
  double t;
  gtsam::Pose3 T;     // body -> world
  gtsam::Vector3 v;   // world-frame velocity
  gtsam::Vector3 ba;  // accelerometer bias (body)
  gtsam::Vector3 bg;  // gyroscope bias (body)
};

// --- ATE evaluation result (after SE(3) Umeyama alignment) ---

struct ATEStats {
  double rmse = -1.0;      // root mean squared error (m)
  double mean_err = -1.0;  // mean error (m)
  double p50 = -1.0;       // median error (m)
  double p95 = -1.0;       // 95th percentile error (m)
  double p99 = -1.0;       // 99th percentile error (m)
  double max_err = -1.0;   // maximum error (m)
  size_t matched = 0;      // number of matched pose pairs
  double scale = 1.0;      // Umeyama scale (should be ≈1 for rigid)
  gtsam::Pose3 T_align;    // SE(3) alignment: est_aligned = T_align * est
  bool ok = false;         // true if alignment succeeded
};

}  // namespace uifgo
