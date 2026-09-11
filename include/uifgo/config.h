#pragma once

#include <map>
#include <string>
#include <vector>

#include "uifgo/types.h"

namespace uifgo {

enum class ImuCovarianceModel {
  LEGACY_GTSAM_COMBINED_DEFAULT_V1,
  PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1,
};
const char* ImuCovarianceModelName(ImuCovarianceModel model);
ImuCovarianceModel ParseImuCovarianceModel(const std::string& name);

struct Config {
  // Explicit paper opt-in. Legacy entry points keep their default constructor.
  ImuCovarianceModel paper_imu_covariance_model =
      ImuCovarianceModel::LEGACY_GTSAM_COMBINED_DEFAULT_V1;
  // --- Dataset interface ---
  // "original": one rosbag containing IMU/UWB and optional ROS GT topics.
  // "mcd": split IMU/UWB bags plus pose_inW-style CSV ground truth.
  // "viral": NTU VIRAL dataset — single rosbag with uwb_driver::UwbRange +
  //          /imu/imu + /leica/pose/relative ground truth.
  std::string data_interface = "original";
  std::string imu_bag_path = "";
  std::string uwb_bag_path = "";
  std::string gt_csv_path = "";
  // T07 self-contained estimator input cache.  The cache stores already
  // normalized IMU samples and final observed UWB ranges; it never contains
  // scenario recipes, injected-component truth, or support labels.
  std::string t07_cache_manifest = "";
  double t07_cache_start_s = 0.0;
  double t07_cache_duration_s = -1.0;

  // --- VIRAL dataset settings ---
  // Requester node IDs on the UAV (e.g. 200, 201). All seen requesters
  // are used when empty.
  std::vector<int> viral_requester_ids;
  // Responder (anchor) node IDs (e.g. 100, 101, 102). All seen responders
  // are used when empty.
  std::vector<int> viral_responder_ids;
  // If true, auto-extract anchor positions from UwbRange.responder_location
  // when the field is populated (not 99999). If false, anchors MUST be
  // provided in the 'anchors' list.
  bool viral_auto_anchors = true;
  // Per-responder antenna offsets in the IMU/body frame. When auto_anchors
  // is enabled, these are the requester antenna lever-arms keyed by
  // requester_id.  Maps can be populated via YAML.
  std::map<int, gtsam::Point3> viral_requester_levers;
  // Time window (s) to group individual UwbRange messages into a single
  // UwbFrame.  0 = treat each range as its own frame (recommended).
  double viral_uwb_group_window = 0.0;
  // Override topics for VIRAL (defaults match the dataset convention).
  std::string viral_imu_topic = "/imu/imu";
  std::string viral_uwb_topic = "/uwb_endorange_info";
  std::string viral_gt_topic = "/leica/pose/relative";

  // --- VIUNet dataset settings (CSV-based, not rosbag) ---
  // Path to sequence root directory containing imu/, uwb/, gt/ subdirs.
  std::string viunet_data_dir = "";
  // Relative paths to CSV files within the data dir.
  std::string viunet_imu_csv = "imu/data.csv";
  std::string viunet_uwb_csv = "uwb/data.csv";
  std::string viunet_gt_csv = "gt/data.csv";
  // If true, reorder UWB anchor columns from (x,z,y) to (x,y,z).
  bool viunet_swap_uwb_yz = true;
  // If true, rotate IMU from D435i frame (y-up) to ENU (z-up).
  bool viunet_rotate_imu_yup = true;

  // --- MILUV dataset settings (CSV-based, PX4 IMU + Decawave UWB + Mocap) ---
  std::string miluv_data_dir = "";
  std::string miluv_robot_dir = "ifo001";
  std::string miluv_imu_csv = "imu_px4.csv";
  std::string miluv_uwb_csv = "uwb_range.csv";
  std::string miluv_gt_csv = "mocap.csv";
  double miluv_uwb_group_window = 0.05;
  std::map<int, gtsam::Point3> miluv_tag_levers;

  // --- SFUISE dataset settings (rosbag-based, ISAS-Walk ToA UWB+IMU+GT) ---
  // Path to directory containing ISAS-Walk1/2/3.bag
  std::string sfuise_data_dir = "";
  // Which ISAS-Walk sequence to load (1, 2, or 3)
  int sfuise_sequence = 1;
  // Time window (s) to group individual RTLSRange messages into a UwbFrame.
  // 0 = treat each range as its own frame.
  double sfuise_uwb_group_window = 0.0;
  // Override topics for SFUISE (defaults match the ISAS-Walk convention).
  std::string sfuise_imu_topic = "/waveshare_sense_hat_b";
  std::string sfuise_uwb_topic = "/rtls_flares";
  std::string sfuise_anchor_topic = "/anchor_list";
  std::string sfuise_gt_topic = "/vive/transform/tracker_1_ref";

  // --- Anchors ---
  std::vector<AnchorConfig> anchors;

  // --- Synthetic anchor ablation study ---
  // When enabled: after baseline optimization, noise-corrupted UWB ranges
  // are generated from GT trajectory to synthetic anchors placed at
  // geometrically-diverse positions. Then re-optimizes with baseline + N
  // synthetic anchors (N = 1,2,3,4) to measure accuracy vs geometry.
  bool synthetic_enabled = false;
  double synthetic_sigma_range = 0.10;                    // range noise std (m)
  std::vector<int> synthetic_test_counts = {1, 2, 3, 4};  // N to test
  std::vector<AnchorConfig> synthetic_anchors;  // positions in anchor frame

  // --- Extrinsics & calibration switches ---
  gtsam::Point3 lever_arm_init;  // UWB antenna in IMU frame (m)
  bool calib_lever = true;
  double lever_prior_sigma = 0.05;

  bool calib_anchor = true;
  std::vector<double> anchor_prior_sigmas;

  bool calib_range_bias = true;
  double range_bias_sigma = 0.05;

  // Fixed static LOS range bias in metres, keyed by "tag_id:anchor_id".
  // This constant path is mutually exclusive with calib_range_bias.
  // An empty map means calibration is unavailable/unused; it is not an
  // assertion that every link has a calibrated zero bias.
  std::map<std::string, double> fixed_beta_by_link;

  bool calib_td = false;
  double td_init = 0.03;  // IMU-UWB time offset (s)

  // --- IMU noise ---
  double sigma_a = 0.1;      // accel density (m/s^2)/sqrt(Hz)
  double sigma_g = 0.01;     // gyro density (rad/s)/sqrt(Hz)
  double sigma_wa = 0.01;    // accel bias RW (m/s^2)/sqrt(s)
  double sigma_wg = 2.0e-5;  // gyro bias RW (rad/s)/sqrt(s)
  double gravity = 9.81;
  bool imu_acc_in_g =
      true;  // true=multiply by gravity (Livox), false=raw (sim)

  // Use sensor_msgs/Imu.orientation when static initialization is unavailable.
  // imu_orientation_world is "enu" or "ned" (converted to ENU internally).
  bool use_imu_orientation_init = false;
  std::string imu_orientation_world = "enu";

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

  // --- Paper-path NLOS debug/refit (T04) ---
  // "disabled" preserves the T02 all-range path. "oracle_debug" and
  // "fixed_partition_debug" read only their isolated support manifests.
  // "automatic_discovery" is the legacy T06 L1/TV development path.
  // "imu_aided_fde" is the 0911 residual-FDE paper producer and cannot read
  // an oracle manifest.
  std::string nlos_mode = "disabled";
  std::string oracle_support_path = "";
  bool score_recoverability = false;
  // T08 is an explicit opt-in layered on oracle_debug or
  // automatic_discovery.  Gate numbers have no defaults: current fixtures
  // must identify them as development-only pending validation.
  bool final_inference_enabled = false;
  double gate_tau_eta = std::numeric_limits<double>::quiet_NaN();
  double gate_tau_s_m = std::numeric_limits<double>::quiet_NaN();
  double gate_tau_gamma = std::numeric_limits<double>::quiet_NaN();
  std::string gate_parameter_provenance;
  double refit_boundary_epsilon_m = 1.0e-9;
  double refit_relative_objective_tolerance = 1.0e-8;
  double refit_scaled_step_tolerance = 1.0e-6;
  double refit_projected_gradient_tolerance = 1.0e-8;
  // Maximum final joint objective gradient with respect to one normalized
  // free navigation coordinate. See RefitOptions for the physical scales.
  double refit_navigation_stationarity_tolerance_objective = 1.0e-6;
  double refit_gradient_roundoff_safety_factor = 8.0;
  double refit_pose_rotation_scale_rad = 1.0;
  double refit_pose_translation_scale_m = 1.0;
  double refit_velocity_scale_mps = 1.0;
  double refit_accel_bias_scale_mps2 = 1.0;
  double refit_gyro_bias_scale_radps = 1.0;
  double refit_segment_amplitude_scale_m = 1.0;
  int max_refit_iterations = 20;
  int oracle_short_min_count_debug = 2;
  double oracle_short_min_duration_debug = 0.01;

  // --- T06 automatic support discovery (pending scientific validation) ---
  double discovery_lambda_l1 = 0.05;
  double discovery_lambda_tv = 0.10;
  double discovery_gap_threshold_s = 1.0;
  double discovery_active_bias_min_m = 0.02;
  double discovery_change_point_min_m = 0.05;
  double discovery_merge_max_difference_m = 0.05;
  int discovery_short_min_count = 2;
  double discovery_short_min_duration_s = 0.01;
  int discovery_max_outer_iterations = 50;
  // A02 development-only numerical policy. The default preserves the
  // accepted Stage-1 conditional solve behavior.
  std::string discovery_conditional_navigation_policy =
      "GTSAM_CHECK_ONLY_V1";
  double discovery_scaled_step_tolerance = 1e-6;
  double discovery_observation_bias_scale_m = 1.0;
  double discovery_rho_scale = 1.0;
  double discovery_primal_abs_tolerance_m = 1e-8;
  double discovery_primal_rel_tolerance = 1e-6;
  double discovery_dual_abs_tolerance_objective_per_m = 1e-8;
  double discovery_dual_rel_tolerance = 1e-6;
  double discovery_kkt_tolerance_objective_per_m = 1e-8;
  double discovery_tv_subgradient_tolerance_objective_per_m = 1e-8;
  int discovery_admm_max_iterations = 10000;

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
  std::string gt_odom_topic =
      "";  // Odom-based GT topic (Odometry), e.g. /sim/odom
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

std::string RangeLinkKey(int tag_id, int anchor_id);
bool ParseRangeLinkKey(const std::string& link, int* tag_id, int* anchor_id);
double FixedBetaForLink(const Config& cfg, int tag_id, int anchor_id);

}  // namespace uifgo
