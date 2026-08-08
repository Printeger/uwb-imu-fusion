#include "uifgo/config.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <stdexcept>

namespace uifgo {

Config ConfigLoader::Load(const std::string& yaml_path) {
  namespace fs = boost::filesystem;
  Config cfg;

  if (!fs::exists(yaml_path)) {
    throw std::runtime_error("Config file not found: " + yaml_path);
  }

  YAML::Node node = YAML::LoadFile(yaml_path);

  // --- Dataset interface ---
  if (node["dataset"]) {
    auto ds = node["dataset"];
    if (ds["interface"]) cfg.data_interface = ds["interface"].as<std::string>();
    if (ds["imu_bag_path"])
      cfg.imu_bag_path = ds["imu_bag_path"].as<std::string>();
    if (ds["uwb_bag_path"])
      cfg.uwb_bag_path = ds["uwb_bag_path"].as<std::string>();
    if (ds["gt_csv_path"])
      cfg.gt_csv_path = ds["gt_csv_path"].as<std::string>();
  }
  if (cfg.data_interface != "original" && cfg.data_interface != "mcd" &&
      cfg.data_interface != "viral" && cfg.data_interface != "viunet" &&
      cfg.data_interface != "miluv" && cfg.data_interface != "sfuise") {
    throw std::runtime_error("Unsupported dataset.interface: " +
                             cfg.data_interface);
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

  // --- Keyframe ---
  if (node["keyframe"]) {
    auto kf = node["keyframe"];
    if (kf["step"]) cfg.kf_step = kf["step"].as<int>();
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

}  // namespace uifgo
