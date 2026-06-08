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
    if (sv["cauchy_k"]) cfg.cauchy_k = sv["cauchy_k"].as<double>();
    if (sv["chi2_reject_prob"])
      cfg.chi2_reject_prob = sv["chi2_reject_prob"].as<double>();
    if (sv["max_rejection_rounds"])
      cfg.max_rejection_rounds = sv["max_rejection_rounds"].as<int>();
  }

  // --- Keyframe ---
  if (node["keyframe"]) {
    auto kf = node["keyframe"];
    if (kf["step"]) cfg.kf_step = kf["step"].as<int>();
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
  }

  // --- Bag ---
  if (node["bag"]) {
    auto bg = node["bag"];
    if (bg["path"]) cfg.bag_path = bg["path"].as<std::string>();
    if (bg["start"]) cfg.bag_start = bg["start"].as<double>();
    if (bg["durr"]) cfg.bag_durr = bg["durr"].as<double>();
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
