#pragma once

#include <string>
#include <vector>

#include "uifgo/types.h"

namespace uifgo {

struct Config {
  // --- Anchors ---
  std::vector<AnchorConfig> anchors;

  // --- Extrinsics & calibration switches ---
  gtsam::Point3 lever_arm_init;  // UWB antenna in IMU frame (m)
  bool calib_lever = true;
  double lever_prior_sigma = 0.05;

  bool calib_anchor = true;
  std::vector<double> anchor_prior_sigmas;

  bool calib_range_bias = true;
  double range_bias_sigma = 0.05;

  bool calib_td = false;
  double td_init = 0.03;  // IMU-UWB time offset (s)

  // --- IMU noise ---
  double sigma_a = 0.1;      // accel noise (m/s^2)
  double sigma_g = 0.01;     // gyro noise (rad/s)
  double sigma_wa = 0.01;    // accel random walk (m/s^3)
  double sigma_wg = 2.0e-5;  // gyro random walk (rad/s^2)
  double gravity = 9.81;

  // --- UWB noise & adaptive ---
  double sigma_range = 0.10;  // base range std (m)
  double v_max = 3.0;         // max velocity for adaptive sigma (m/s)

  // --- Outlier filter thresholds ---
  double nlos_rssi_diff = 6.0;           // rx_rssi - fp_rssi > this => NLOS
  double min_range = 0.3;                // minimum valid range (m)
  double max_range = 100.0;              // maximum valid range (m)
  double dist_consistency_thresh = 1.0;  // distance consistency check (m)
  int warmup_frames = 20;  // frames before enabling consistency check

  // --- Solver ---
  int lm_max_iter = 100;
  double lm_rel_tol = 1.0e-6;
  double lm_abs_tol = 1.0e-8;

  // --- GNC (Graduated Non-Convexity) with TLS kernel ---
  double gnc_mu_step = 1.4;        // mu homotopy step (GTSAM default)
  int gnc_max_iter = 50;           // GNC outer-loop max iterations
  double gnc_rel_cost_tol = 1e-5;  // GNC convergence tolerance on relative cost
  double gnc_inlier_prob = 0.99;   // chi2 confidence for inlier cost threshold
  double gnc_weight_thresh =
      0.01;  // GNC weight below which UWB factor is hard-rejected
             // (conservative: only extreme outliers)
  // --- Chi-square rejection loop (post-GNC) ---
  double chi2_reject_prob = 0.99;  // chi2 rejection confidence
  int max_rejection_rounds = 3;

  // --- Keyframe ---
  double kf_min_interval =
      0.0;          // minimum keyframe interval (s), 0=all UWB frames
  int kf_step = 1;  // use every Nth UWB frame as keyframe (1=all)
  int yaw_align_frames =
      15;  // # keyframes for yaw alignment grid search (0=skip)

  // --- Debug logging ---
  bool debug_log =
      false;  // true → save detailed logs to logs/<timestamp>_<bag>/

  // --- Default path defaults for ConfigLoader ---
  std::string imu_topic = "/imu/data";
  std::string uwb_topic = "/nlink_linktrack_nodeframe3";
  std::string vicon_topic = "";  // VICON GT topic (PoseStamped)
  std::string bag_path = "";
  double bag_start = 0.0;
  double bag_durr = -1.0;  // -1 = full bag
};

class ConfigLoader {
 public:
  static Config Load(const std::string& yaml_path);

  // Resolve bag path: tries cwd-relative, config-dir-relative,
  // project-root-relative
  static std::string ResolveBagPath(const std::string& config_dir,
                                    const std::string& bag_path);
};

}  // namespace uifgo
