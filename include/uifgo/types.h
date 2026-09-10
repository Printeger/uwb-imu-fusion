#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <limits>
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
  // Paper-path metadata. Legacy aggregate initializers leave these defaults.
  std::uint64_t obs_id = 0;
  double nominal_sigma = std::numeric_limits<double>::quiet_NaN();
  bool suspected_nlos = false;
  // Loader provenance captured before paper-path filtering/group reordering.
  // max() means that a legacy or synthetic caller did not provide a source
  // ordinal. source_valid is protocol validity, separate from NLOS suspicion.
  std::uint64_t source_obs_index = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t source_message_index =
      std::numeric_limits<std::uint64_t>::max();
  std::uint64_t source_range_index =
      std::numeric_limits<std::uint64_t>::max();
  double source_time = std::numeric_limits<double>::quiet_NaN();
  int source_tag_id = std::numeric_limits<int>::min();
  bool source_valid = true;
  std::string source_validity_reason;
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

// Rebuilt for every graph. factor_index is never a persistent observation ID.
struct FactorMeta {
  size_t factor_index = 0;
  std::uint64_t obs_id = 0;
  std::string factor_type;
  std::vector<gtsam::Key> keys;
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
