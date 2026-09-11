#include "uifgo/config.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace uifgo {

const char* ImuCovarianceModelName(ImuCovarianceModel model) {
  switch (model) {
    case ImuCovarianceModel::LEGACY_GTSAM_COMBINED_DEFAULT_V1:
      return "LEGACY_GTSAM_COMBINED_DEFAULT_V1";
    case ImuCovarianceModel::PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1:
      return "PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1";
  }
  throw std::invalid_argument("INVALID_IMU_COVARIANCE_MODEL");
}
ImuCovarianceModel ParseImuCovarianceModel(const std::string& name) {
  for (auto m : {ImuCovarianceModel::LEGACY_GTSAM_COMBINED_DEFAULT_V1,
                 ImuCovarianceModel::PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1})
    if (name == ImuCovarianceModelName(m)) return m;
  throw std::runtime_error("INVALID_IMU_COVARIANCE_MODEL: " + name);
}

Config ConfigLoader::Load(const std::string& yaml_path) {
  namespace fs = boost::filesystem;
  Config cfg;

  if (!fs::exists(yaml_path)) {
    throw std::runtime_error("Config file not found: " + yaml_path);
  }

  YAML::Node node = YAML::LoadFile(yaml_path);
  if (node["paper"]) {
    const auto paper = node["paper"];
    if (!paper.IsMap()) throw std::runtime_error("paper must be a map");
    for (const auto& item : paper)
      if (item.first.as<std::string>() != "imu_covariance_model")
        throw std::runtime_error("Unsupported paper key: " + item.first.as<std::string>());
    if (paper["imu_covariance_model"])
      cfg.paper_imu_covariance_model = ParseImuCovarianceModel(
          paper["imu_covariance_model"].as<std::string>());
  }

  // --- Dataset interface ---
  if (node["dataset"]) {
    auto ds = node["dataset"];
    const std::set<std::string> allowed = {
        "interface", "imu_bag_path", "uwb_bag_path", "gt_csv_path",
        "cache_manifest", "cache_start_s", "cache_duration_s"};
    for (const auto& kv : ds) {
      const std::string key = kv.first.as<std::string>();
      if (!allowed.count(key))
        throw std::runtime_error("Unknown dataset field: " + key);
    }
    if (ds["interface"]) cfg.data_interface = ds["interface"].as<std::string>();
    if (ds["imu_bag_path"])
      cfg.imu_bag_path = ds["imu_bag_path"].as<std::string>();
    if (ds["uwb_bag_path"])
      cfg.uwb_bag_path = ds["uwb_bag_path"].as<std::string>();
    if (ds["gt_csv_path"])
      cfg.gt_csv_path = ds["gt_csv_path"].as<std::string>();
    if (ds["cache_manifest"])
      cfg.t07_cache_manifest = ds["cache_manifest"].as<std::string>();
    if (ds["cache_start_s"])
      cfg.t07_cache_start_s = ds["cache_start_s"].as<double>();
    if (ds["cache_duration_s"])
      cfg.t07_cache_duration_s = ds["cache_duration_s"].as<double>();
  }
  if (cfg.data_interface != "original" && cfg.data_interface != "mcd" &&
      cfg.data_interface != "viral" && cfg.data_interface != "viunet" &&
      cfg.data_interface != "miluv" && cfg.data_interface != "sfuise" &&
      cfg.data_interface != "t07_cache") {
    throw std::runtime_error("Unsupported dataset.interface: " +
                             cfg.data_interface);
  }
  if (cfg.data_interface == "t07_cache") {
    if (cfg.t07_cache_manifest.empty())
      throw std::runtime_error(
          "dataset.cache_manifest is required for t07_cache");
    if (!std::isfinite(cfg.t07_cache_start_s) ||
        cfg.t07_cache_start_s < 0.0 ||
        !std::isfinite(cfg.t07_cache_duration_s) ||
        (cfg.t07_cache_duration_s < 0.0 &&
         cfg.t07_cache_duration_s != -1.0)) {
      throw std::runtime_error(
          "invalid t07_cache start/duration in sensor-time seconds");
    }
  }

  // --- Anchors ---
  if (node["anchors"]) {
    for (const auto& a : node["anchors"]) {
      AnchorConfig ac;
      ac.id = a["id"].as<int>();
      auto pos = a["pos"].as<std::vector<double>>();
      ac.pos = gtsam::Point3(pos[0], pos[1], pos[2]);
      ac.prior_sigma = a["prior_sigma"] ? a["prior_sigma"].as<double>() : 0.08;
      cfg.anchors.push_back(ac);
    }
  }

  // --- Extrinsics ---
  if (node["extrinsics"]) {
    auto ex = node["extrinsics"];
    if (ex["lever_arm_init"]) {
      auto la = ex["lever_arm_init"].as<std::vector<double>>();
      cfg.lever_arm_init = gtsam::Point3(la[0], la[1], la[2]);
    }
    if (ex["calib_lever"]) cfg.calib_lever = ex["calib_lever"].as<bool>();
    if (ex["lever_prior_sigma"])
      cfg.lever_prior_sigma = ex["lever_prior_sigma"].as<double>();
  }

  // --- Calibration ---
  if (node["calibration"]) {
    auto cal = node["calibration"];
    if (cal["calib_anchor"]) cfg.calib_anchor = cal["calib_anchor"].as<bool>();
    if (cal["calib_range_bias"])
      cfg.calib_range_bias = cal["calib_range_bias"].as<bool>();
    if (cal["range_bias_sigma"])
      cfg.range_bias_sigma = cal["range_bias_sigma"].as<double>();
    if (cal["calib_td"]) cfg.calib_td = cal["calib_td"].as<bool>();
    if (cal["td_init"]) cfg.td_init = cal["td_init"].as<double>();
    if (cal["fixed_beta_by_link"]) {
      for (const auto& kv : cal["fixed_beta_by_link"]) {
        const std::string link = kv.first.as<std::string>();
        int tag_id = 0, anchor_id = 0;
        if (!ParseRangeLinkKey(link, &tag_id, &anchor_id)) {
          throw std::runtime_error(
              "Invalid fixed_beta_by_link key (expected canonical "
              "nonnegative tag_id:anchor_id): " +
              link);
        }
        const double beta = kv.second.as<double>();
        if (!std::isfinite(beta)) {
          throw std::runtime_error("Non-finite fixed beta for link: " + link);
        }
        cfg.fixed_beta_by_link[link] = beta;
      }
    }
  }

  if (cfg.calib_range_bias && !cfg.fixed_beta_by_link.empty()) {
    throw std::runtime_error(
        "calibration.calib_range_bias and fixed_beta_by_link are mutually "
        "exclusive");
  }

  // --- IMU ---
  if (node["imu"]) {
    auto im = node["imu"];
    if (im["sigma_a"]) cfg.sigma_a = im["sigma_a"].as<double>();
    if (im["sigma_g"]) cfg.sigma_g = im["sigma_g"].as<double>();
    if (im["sigma_wa"]) cfg.sigma_wa = im["sigma_wa"].as<double>();
    if (im["sigma_wg"]) cfg.sigma_wg = im["sigma_wg"].as<double>();
    if (im["gravity"]) cfg.gravity = im["gravity"].as<double>();
    if (im["imu_acc_in_g"]) cfg.imu_acc_in_g = im["imu_acc_in_g"].as<bool>();
  }

  // --- Initialization ---
  if (node["initialization"]) {
    auto init = node["initialization"];
    if (init["use_imu_orientation"])
      cfg.use_imu_orientation_init = init["use_imu_orientation"].as<bool>();
    if (init["imu_orientation_world"])
      cfg.imu_orientation_world =
          init["imu_orientation_world"].as<std::string>();
  }
  if (cfg.imu_orientation_world != "enu" &&
      cfg.imu_orientation_world != "ned") {
    throw std::runtime_error(
        "Unsupported initialization.imu_orientation_world: " +
        cfg.imu_orientation_world);
  }

  // --- UWB ---
  if (node["uwb"]) {
    auto uw = node["uwb"];
    if (uw["sigma_range"]) cfg.sigma_range = uw["sigma_range"].as<double>();
    if (uw["v_max"]) cfg.v_max = uw["v_max"].as<double>();
    if (uw["nlos_rssi_diff"])
      cfg.nlos_rssi_diff = uw["nlos_rssi_diff"].as<double>();
    if (uw["min_range"]) cfg.min_range = uw["min_range"].as<double>();
    if (uw["max_range"]) cfg.max_range = uw["max_range"].as<double>();
    if (uw["dist_consistency_thresh"])
      cfg.dist_consistency_thresh = uw["dist_consistency_thresh"].as<double>();
    if (uw["warmup_frames"]) cfg.warmup_frames = uw["warmup_frames"].as<int>();
  }

  // --- Solver ---
  if (node["solver"]) {
    auto sv = node["solver"];
    if (sv["lm_max_iter"]) cfg.lm_max_iter = sv["lm_max_iter"].as<int>();
    if (sv["rel_error_tol"]) cfg.lm_rel_tol = sv["rel_error_tol"].as<double>();
    if (sv["abs_error_tol"]) cfg.lm_abs_tol = sv["abs_error_tol"].as<double>();
    if (sv["gnc_mu_step"]) cfg.gnc_mu_step = sv["gnc_mu_step"].as<double>();
    if (sv["gnc_max_iter"]) cfg.gnc_max_iter = sv["gnc_max_iter"].as<int>();
    if (sv["gnc_rel_cost_tol"])
      cfg.gnc_rel_cost_tol = sv["gnc_rel_cost_tol"].as<double>();
    if (sv["gnc_inlier_prob"])
      cfg.gnc_inlier_prob = sv["gnc_inlier_prob"].as<double>();
    if (sv["gnc_weight_thresh"])
      cfg.gnc_weight_thresh = sv["gnc_weight_thresh"].as<double>();
    if (sv["chi2_reject_prob"])
      cfg.chi2_reject_prob = sv["chi2_reject_prob"].as<double>();
    if (sv["max_rejection_rounds"])
      cfg.max_rejection_rounds = sv["max_rejection_rounds"].as<int>();
  }

  // --- Paper-path NLOS debug/refit ---
  if (node["nlos"]) {
    auto nl = node["nlos"];
    if (!nl.IsMap()) throw std::runtime_error("nlos must be a map");
    const std::set<std::string> allowed = {
        "mode", "fde_grouped_test", "oracle_support", "boundary_epsilon_m",
        "relative_objective_tolerance", "scaled_step_tolerance",
        "projected_gradient_tolerance",
        "navigation_stationarity_tolerance_objective",
        "gradient_roundoff_safety_factor", "pose_rotation_scale_rad",
        "pose_translation_scale_m", "velocity_scale_mps",
        "accel_bias_scale_mps2", "gyro_bias_scale_radps",
        "segment_amplitude_scale_m", "max_refit_iterations",
        "short_min_count_debug", "short_min_duration_debug",
        "score_recoverability", "final_inference_enabled", "tau_eta",
        "tau_s_m", "tau_gamma", "gate_parameter_provenance",
        "lambda_l1", "lambda_tv",
        "gap_threshold_s", "active_bias_min_m", "change_point_min_m",
        "merge_max_difference_m", "discovery_short_min_count",
        "discovery_short_min_duration_s", "discovery_max_outer_iterations",
        "discovery_conditional_navigation_policy",
        "discovery_scaled_step_tolerance",
        "discovery_observation_bias_scale_m",
        "rho_scale", "admm_primal_abs_tolerance_m",
        "admm_primal_rel_tolerance",
        "admm_dual_abs_tolerance_objective_per_m",
        "admm_dual_rel_tolerance", "admm_kkt_tolerance_objective_per_m",
        "admm_tv_subgradient_tolerance_objective_per_m",
        "admm_max_iterations"};
    for (const auto& kv : nl) {
      const std::string key = kv.first.as<std::string>();
      if (!allowed.count(key))
        throw std::runtime_error("Unknown nlos field: " + key);
    }
    if (nl["mode"]) cfg.nlos_mode = nl["mode"].as<std::string>();
    if (nl["oracle_support"])
      cfg.oracle_support_path = nl["oracle_support"].as<std::string>();
    if (nl["score_recoverability"])
      cfg.score_recoverability = nl["score_recoverability"].as<bool>();
    if (nl["final_inference_enabled"])
      cfg.final_inference_enabled = nl["final_inference_enabled"].as<bool>();
    if (nl["tau_eta"])
      cfg.gate_tau_eta = nl["tau_eta"].as<double>();
    if (nl["tau_s_m"])
      cfg.gate_tau_s_m = nl["tau_s_m"].as<double>();
    if (nl["tau_gamma"])
      cfg.gate_tau_gamma = nl["tau_gamma"].as<double>();
    if (nl["gate_parameter_provenance"])
      cfg.gate_parameter_provenance =
          nl["gate_parameter_provenance"].as<std::string>();
    if (nl["boundary_epsilon_m"])
      cfg.refit_boundary_epsilon_m = nl["boundary_epsilon_m"].as<double>();
    if (nl["relative_objective_tolerance"])
      cfg.refit_relative_objective_tolerance =
          nl["relative_objective_tolerance"].as<double>();
    if (nl["scaled_step_tolerance"])
      cfg.refit_scaled_step_tolerance =
          nl["scaled_step_tolerance"].as<double>();
    if (nl["projected_gradient_tolerance"])
      cfg.refit_projected_gradient_tolerance =
          nl["projected_gradient_tolerance"].as<double>();
    if (nl["navigation_stationarity_tolerance_objective"])
      cfg.refit_navigation_stationarity_tolerance_objective =
          nl["navigation_stationarity_tolerance_objective"].as<double>();
    if (nl["gradient_roundoff_safety_factor"])
      cfg.refit_gradient_roundoff_safety_factor =
          nl["gradient_roundoff_safety_factor"].as<double>();
    if (nl["pose_rotation_scale_rad"])
      cfg.refit_pose_rotation_scale_rad =
          nl["pose_rotation_scale_rad"].as<double>();
    if (nl["pose_translation_scale_m"])
      cfg.refit_pose_translation_scale_m =
          nl["pose_translation_scale_m"].as<double>();
    if (nl["velocity_scale_mps"])
      cfg.refit_velocity_scale_mps = nl["velocity_scale_mps"].as<double>();
    if (nl["accel_bias_scale_mps2"])
      cfg.refit_accel_bias_scale_mps2 =
          nl["accel_bias_scale_mps2"].as<double>();
    if (nl["gyro_bias_scale_radps"])
      cfg.refit_gyro_bias_scale_radps =
          nl["gyro_bias_scale_radps"].as<double>();
    if (nl["segment_amplitude_scale_m"])
      cfg.refit_segment_amplitude_scale_m =
          nl["segment_amplitude_scale_m"].as<double>();
    if (nl["max_refit_iterations"])
      cfg.max_refit_iterations = nl["max_refit_iterations"].as<int>();
    if (nl["short_min_count_debug"])
      cfg.oracle_short_min_count_debug =
          nl["short_min_count_debug"].as<int>();
    if (nl["short_min_duration_debug"])
      cfg.oracle_short_min_duration_debug =
          nl["short_min_duration_debug"].as<double>();
    if (nl["lambda_l1"])
      cfg.discovery_lambda_l1 = nl["lambda_l1"].as<double>();
    if (nl["lambda_tv"])
      cfg.discovery_lambda_tv = nl["lambda_tv"].as<double>();
    if (nl["fde_grouped_test"])
      cfg.fde_grouped_test = nl["fde_grouped_test"].as<bool>();
    if (nl["gap_threshold_s"])
      cfg.discovery_gap_threshold_s = nl["gap_threshold_s"].as<double>();
    if (nl["active_bias_min_m"])
      cfg.discovery_active_bias_min_m = nl["active_bias_min_m"].as<double>();
    if (nl["change_point_min_m"])
      cfg.discovery_change_point_min_m =
          nl["change_point_min_m"].as<double>();
    if (nl["merge_max_difference_m"])
      cfg.discovery_merge_max_difference_m =
          nl["merge_max_difference_m"].as<double>();
    if (nl["discovery_short_min_count"])
      cfg.discovery_short_min_count =
          nl["discovery_short_min_count"].as<int>();
    if (nl["discovery_short_min_duration_s"])
      cfg.discovery_short_min_duration_s =
          nl["discovery_short_min_duration_s"].as<double>();
    if (nl["discovery_max_outer_iterations"])
      cfg.discovery_max_outer_iterations =
          nl["discovery_max_outer_iterations"].as<int>();
    if (nl["discovery_conditional_navigation_policy"])
      cfg.discovery_conditional_navigation_policy =
          nl["discovery_conditional_navigation_policy"].as<std::string>();
    if (nl["discovery_scaled_step_tolerance"])
      cfg.discovery_scaled_step_tolerance =
          nl["discovery_scaled_step_tolerance"].as<double>();
    if (nl["discovery_observation_bias_scale_m"])
      cfg.discovery_observation_bias_scale_m =
          nl["discovery_observation_bias_scale_m"].as<double>();
    if (nl["rho_scale"])
      cfg.discovery_rho_scale = nl["rho_scale"].as<double>();
    if (nl["admm_primal_abs_tolerance_m"])
      cfg.discovery_primal_abs_tolerance_m =
          nl["admm_primal_abs_tolerance_m"].as<double>();
    if (nl["admm_primal_rel_tolerance"])
      cfg.discovery_primal_rel_tolerance =
          nl["admm_primal_rel_tolerance"].as<double>();
    if (nl["admm_dual_abs_tolerance_objective_per_m"])
      cfg.discovery_dual_abs_tolerance_objective_per_m =
          nl["admm_dual_abs_tolerance_objective_per_m"].as<double>();
    if (nl["admm_dual_rel_tolerance"])
      cfg.discovery_dual_rel_tolerance =
          nl["admm_dual_rel_tolerance"].as<double>();
    if (nl["admm_kkt_tolerance_objective_per_m"])
      cfg.discovery_kkt_tolerance_objective_per_m =
          nl["admm_kkt_tolerance_objective_per_m"].as<double>();
    if (nl["admm_tv_subgradient_tolerance_objective_per_m"])
      cfg.discovery_tv_subgradient_tolerance_objective_per_m =
          nl["admm_tv_subgradient_tolerance_objective_per_m"].as<double>();
    if (nl["admm_max_iterations"])
      cfg.discovery_admm_max_iterations =
          nl["admm_max_iterations"].as<int>();
  }
  if (cfg.nlos_mode != "disabled" && cfg.nlos_mode != "oracle_debug" &&
      cfg.nlos_mode != "fixed_partition_debug" &&
      cfg.nlos_mode != "automatic_discovery" &&
      cfg.nlos_mode != "imu_aided_fde")
    throw std::runtime_error("Unsupported nlos.mode: " + cfg.nlos_mode);
  if (cfg.discovery_conditional_navigation_policy !=
          "GTSAM_CHECK_ONLY_V1" &&
      cfg.discovery_conditional_navigation_policy !=
          "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1" &&
      cfg.discovery_conditional_navigation_policy !=
          "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2")
    throw std::runtime_error(
        "Unsupported nlos.discovery_conditional_navigation_policy: " +
        cfg.discovery_conditional_navigation_policy);
  if (cfg.nlos_mode != "automatic_discovery" &&
      cfg.discovery_conditional_navigation_policy !=
          "GTSAM_CHECK_ONLY_V1")
    throw std::runtime_error(
        "stationarity-qualified conditional policy is only available for "
        "automatic_discovery");
  if ((cfg.nlos_mode == "oracle_debug" ||
       cfg.nlos_mode == "fixed_partition_debug") &&
      cfg.oracle_support_path.empty())
    throw std::runtime_error(
        "nlos.oracle_support is required for oracle_debug mode");
  if (cfg.nlos_mode == "automatic_discovery") {
    const auto nl = node["nlos"];
    if (nl["oracle_support"].IsDefined())
      throw std::runtime_error(
          "nlos.oracle_support field is forbidden for automatic_discovery mode");
    if (!nl["score_recoverability"].IsDefined() ||
        !nl["score_recoverability"].as<bool>())
      throw std::runtime_error(
          "automatic_discovery requires score_recoverability: true");
    const std::vector<std::string> required = {
        "lambda_l1", "lambda_tv", "gap_threshold_s",
        "active_bias_min_m", "change_point_min_m",
        "merge_max_difference_m", "discovery_short_min_count",
        "discovery_short_min_duration_s",
        "discovery_scaled_step_tolerance",
        "discovery_observation_bias_scale_m"};
    for (const auto& key : required) {
      if (!nl[key].IsDefined())
        throw std::runtime_error(
            "automatic_discovery requires explicit nlos." + key);
    }
  }
  if (cfg.nlos_mode == "imu_aided_fde") {
    const auto nl = node["nlos"];
    if (nl["oracle_support"].IsDefined())
      throw std::runtime_error(
          "nlos.oracle_support field is forbidden for imu_aided_fde mode");
    if (!nl["score_recoverability"].IsDefined() ||
        !nl["score_recoverability"].as<bool>())
      throw std::runtime_error(
          "imu_aided_fde requires score_recoverability: true");
    const std::vector<std::string> required = {
        "gap_threshold_s", "discovery_short_min_count",
        "discovery_short_min_duration_s"};
    for (const auto& key : required) {
      if (!nl[key].IsDefined())
        throw std::runtime_error(
            "imu_aided_fde requires explicit nlos." + key);
    }
    if (!node["solver"] ||
        !node["solver"]["chi2_reject_prob"].IsDefined())
      throw std::runtime_error(
          "imu_aided_fde requires explicit solver.chi2_reject_prob");
    if (cfg.chi2_reject_prob != 0.99)
      throw std::runtime_error(
          "imu_aided_fde requires solver.chi2_reject_prob exactly 0.99");
    if (!std::isfinite(cfg.discovery_gap_threshold_s) ||
        cfg.discovery_gap_threshold_s < 0.0 ||
        cfg.discovery_short_min_count <= 0 ||
        !std::isfinite(cfg.discovery_short_min_duration_s) ||
        cfg.discovery_short_min_duration_s < 0.0)
      throw std::runtime_error(
          "imu_aided_fde temporal support parameters are invalid");
  }
  if (cfg.final_inference_enabled) {
    const auto nl = node["nlos"];
    if (cfg.nlos_mode != "oracle_debug" &&
        cfg.nlos_mode != "automatic_discovery" &&
        cfg.nlos_mode != "imu_aided_fde")
      throw std::runtime_error(
          "final_inference_enabled requires oracle_debug, automatic_discovery, or imu_aided_fde");
    if (!cfg.score_recoverability)
      throw std::runtime_error(
          "final_inference_enabled requires score_recoverability: true");
    const std::vector<std::string> required = {
        "tau_eta", "tau_s_m", "tau_gamma", "gate_parameter_provenance"};
    for (const auto& key : required) {
      if (!nl[key].IsDefined())
        throw std::runtime_error(
            "final_inference_enabled requires explicit nlos." + key);
    }
    if (!std::isfinite(cfg.gate_tau_eta) ||
        !std::isfinite(cfg.gate_tau_s_m) ||
        !std::isfinite(cfg.gate_tau_gamma) || cfg.gate_tau_eta < 0.0 ||
        cfg.gate_tau_eta > 1.0 || cfg.gate_tau_s_m < 0.0 ||
        cfg.gate_tau_gamma < 0.0)
      throw std::runtime_error(
          "T08 gate thresholds must be finite and within declared domains");
    if (cfg.gate_parameter_provenance !=
        "T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION")
      throw std::runtime_error(
          "T08 gate parameters must remain explicitly development-only pending validation");
  }

  // --- Keyframe ---
  if (node["keyframe"]) {
    auto kf = node["keyframe"];
    if (kf["step"]) cfg.kf_step = kf["step"].as<int>();
    if (kf["min_interval"])
      cfg.kf_min_interval = kf["min_interval"].as<double>();
    if (kf["yaw_align_frames"])
      cfg.yaw_align_frames = kf["yaw_align_frames"].as<int>();
  }

  // --- Debug ---
  if (node["debug"]) {
    auto db = node["debug"];
    if (db["log"]) cfg.debug_log = db["log"].as<bool>();
  }

  // --- Topics ---
  if (node["topics"]) {
    auto tp = node["topics"];
    if (tp["imu"]) cfg.imu_topic = tp["imu"].as<std::string>();
    if (tp["uwb"]) cfg.uwb_topic = tp["uwb"].as<std::string>();
    if (tp["vicon"]) cfg.vicon_topic = tp["vicon"].as<std::string>();
    if (tp["gt_odom"]) cfg.gt_odom_topic = tp["gt_odom"].as<std::string>();
  }

  // --- Bag ---
  if (node["bag"]) {
    auto bg = node["bag"];
    if (bg["path"]) cfg.bag_path = bg["path"].as<std::string>();
    if (bg["start"]) cfg.bag_start = bg["start"].as<double>();
    if (bg["durr"]) cfg.bag_durr = bg["durr"].as<double>();
  }

  // --- VIRAL ---
  if (node["viral"]) {
    auto vr = node["viral"];
    if (vr["requester_ids"]) {
      cfg.viral_requester_ids = vr["requester_ids"].as<std::vector<int>>();
    }
    if (vr["responder_ids"]) {
      cfg.viral_responder_ids = vr["responder_ids"].as<std::vector<int>>();
    }
    if (vr["auto_anchors"])
      cfg.viral_auto_anchors = vr["auto_anchors"].as<bool>();
    if (vr["uwb_group_window"])
      cfg.viral_uwb_group_window = vr["uwb_group_window"].as<double>();
    if (vr["imu_topic"])
      cfg.viral_imu_topic = vr["imu_topic"].as<std::string>();
    if (vr["uwb_topic"])
      cfg.viral_uwb_topic = vr["uwb_topic"].as<std::string>();
    if (vr["gt_topic"]) cfg.viral_gt_topic = vr["gt_topic"].as<std::string>();
    // requester_levers: map of node_id -> [x, y, z]
    if (vr["requester_levers"]) {
      for (const auto& kv : vr["requester_levers"]) {
        int nid = kv.first.as<int>();
        auto lv = kv.second.as<std::vector<double>>();
        cfg.viral_requester_levers[nid] = gtsam::Point3(lv[0], lv[1], lv[2]);
      }
    }
  }

  // --- VIUNet ---
  if (node["viunet"]) {
    auto vn = node["viunet"];
    if (vn["data_dir"]) cfg.viunet_data_dir = vn["data_dir"].as<std::string>();
    if (vn["imu_csv"]) cfg.viunet_imu_csv = vn["imu_csv"].as<std::string>();
    if (vn["uwb_csv"]) cfg.viunet_uwb_csv = vn["uwb_csv"].as<std::string>();
    if (vn["gt_csv"]) cfg.viunet_gt_csv = vn["gt_csv"].as<std::string>();
    if (vn["swap_uwb_yz"])
      cfg.viunet_swap_uwb_yz = vn["swap_uwb_yz"].as<bool>();
    if (vn["rotate_imu_yup"])
      cfg.viunet_rotate_imu_yup = vn["rotate_imu_yup"].as<bool>();
  }

  // --- MILUV ---
  if (node["miluv"]) {
    auto ml = node["miluv"];
    if (ml["data_dir"]) cfg.miluv_data_dir = ml["data_dir"].as<std::string>();
    if (ml["robot_dir"])
      cfg.miluv_robot_dir = ml["robot_dir"].as<std::string>();
    if (ml["imu_csv"]) cfg.miluv_imu_csv = ml["imu_csv"].as<std::string>();
    if (ml["uwb_csv"]) cfg.miluv_uwb_csv = ml["uwb_csv"].as<std::string>();
    if (ml["gt_csv"]) cfg.miluv_gt_csv = ml["gt_csv"].as<std::string>();
    if (ml["uwb_group_window"])
      cfg.miluv_uwb_group_window = ml["uwb_group_window"].as<double>();
    if (ml["tag_levers"]) {
      for (const auto& kv : ml["tag_levers"]) {
        int tid = kv.first.as<int>();
        auto lv = kv.second.as<std::vector<double>>();
        cfg.miluv_tag_levers[tid] = gtsam::Point3(lv[0], lv[1], lv[2]);
      }
    }
  }

  // --- SFUISE ---
  if (node["sfuise"]) {
    auto sf = node["sfuise"];
    if (sf["data_dir"]) cfg.sfuise_data_dir = sf["data_dir"].as<std::string>();
    if (sf["sequence"]) cfg.sfuise_sequence = sf["sequence"].as<int>();
    if (sf["uwb_group_window"])
      cfg.sfuise_uwb_group_window = sf["uwb_group_window"].as<double>();
    if (sf["imu_topic"])
      cfg.sfuise_imu_topic = sf["imu_topic"].as<std::string>();
    if (sf["uwb_topic"])
      cfg.sfuise_uwb_topic = sf["uwb_topic"].as<std::string>();
    if (sf["anchor_topic"])
      cfg.sfuise_anchor_topic = sf["anchor_topic"].as<std::string>();
    if (sf["gt_topic"]) cfg.sfuise_gt_topic = sf["gt_topic"].as<std::string>();
  }

  // --- Synthetic anchors ---
  if (node["synthetic"]) {
    auto sy = node["synthetic"];
    if (sy["enabled"]) cfg.synthetic_enabled = sy["enabled"].as<bool>();
    if (sy["sigma_range"])
      cfg.synthetic_sigma_range = sy["sigma_range"].as<double>();
    if (sy["test_counts"]) {
      cfg.synthetic_test_counts = sy["test_counts"].as<std::vector<int>>();
    }
    if (sy["anchors"]) {
      for (const auto& a : sy["anchors"]) {
        AnchorConfig ac;
        ac.id = a["id"].as<int>();
        auto pos = a["pos"].as<std::vector<double>>();
        ac.pos = gtsam::Point3(pos[0], pos[1], pos[2]);
        ac.prior_sigma =
            a["prior_sigma"] ? a["prior_sigma"].as<double>() : 0.05;
        cfg.synthetic_anchors.push_back(ac);
      }
    }
  }

  // populate per-anchor prior sigma list
  for (const auto& a : cfg.anchors) {
    cfg.anchor_prior_sigmas.push_back(a.prior_sigma);
  }

  return cfg;
}

std::string ConfigLoader::ResolveBagPath(const std::string& config_dir,
                                         const std::string& bag_path) {
  namespace fs = boost::filesystem;
  if (bag_path.empty()) return bag_path;
  if (fs::exists(bag_path)) return bag_path;

  // Try relative to config dir
  fs::path rel = fs::path(config_dir) / bag_path;
  if (fs::exists(rel)) return rel.string();

  // Try with data/ prefix relative to project
  fs::path pkg_root = fs::path(config_dir).parent_path();
  fs::path data_rel =
      pkg_root / "data" / fs::path(bag_path).filename().string();
  if (fs::exists(data_rel)) return data_rel.string();

  return bag_path;  // return as-is, let caller handle
}

std::string RangeLinkKey(int tag_id, int anchor_id) {
  return std::to_string(tag_id) + ":" + std::to_string(anchor_id);
}

bool ParseRangeLinkKey(const std::string& link, int* tag_id, int* anchor_id) {
  if (!tag_id || !anchor_id) return false;
  const size_t colon = link.find(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 >= link.size() ||
      link.find(':', colon + 1) != std::string::npos) {
    return false;
  }
  try {
    size_t tag_used = 0, anchor_used = 0;
    const long long parsed_tag = std::stoll(link.substr(0, colon), &tag_used);
    const long long parsed_anchor =
        std::stoll(link.substr(colon + 1), &anchor_used);
    if (tag_used != colon || anchor_used != link.size() - colon - 1 ||
        parsed_tag < 0 || parsed_anchor < 0 ||
        parsed_tag > std::numeric_limits<int>::max() ||
        parsed_anchor > std::numeric_limits<int>::max()) {
      return false;
    }
    *tag_id = static_cast<int>(parsed_tag);
    *anchor_id = static_cast<int>(parsed_anchor);
    return RangeLinkKey(*tag_id, *anchor_id) == link;
  } catch (const std::exception&) {
    return false;
  }
}

double FixedBetaForLink(const Config& cfg, int tag_id, int anchor_id) {
  const auto it = cfg.fixed_beta_by_link.find(RangeLinkKey(tag_id, anchor_id));
  return it == cfg.fixed_beta_by_link.end() ? 0.0 : it->second;
}

}  // namespace uifgo
