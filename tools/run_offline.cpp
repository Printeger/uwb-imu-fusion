// run_offline.cpp — ROS entry point for batch UWB-IMU FGO processing.
// Multi-pass pipeline (design doc §6.3):
//   Pass 1: GNC+TLS robust init (all calib OFF) → outlier identification
//   Pass 2..4: Pure-LM refinement with progressive calibration unlock
//   Pass Final: Joint LM refinement with chi2 monitoring
//
// Usage:
//   rosrun uwb_imu_pl uwb_imu_pl_node _config_path:=/path/to/slam.yaml

#include <gtsam/inference/Symbol.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <atomic>
#include <boost/filesystem.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <thread>

#include "uifgo/config.h"
#include "uifgo/data_loader.h"
#include "uifgo/graph_builder.h"
#include "uifgo/initializer.h"
#include "uifgo/logger.h"
#include "uifgo/optimizer.h"
#include "uifgo/outlier_filter.h"
#include "uifgo/trajectory_io.h"
#include "uifgo/visualizer.h"

namespace fs = boost::filesystem;

// Helper: run one optimization pass with given calibration config
static uifgo::OptimizerResult RunPass(const uifgo::Config& cfg,
                                      const std::vector<uifgo::UwbFrame>& kfs,
                                      const std::vector<uifgo::ImuSample>& imu,
                                      const uifgo::InitResult& init,
                                      const std::string& pass_name) {
  uifgo::GraphBuilder builder(cfg);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::vector<size_t> uwb_indices;
  builder.Build(kfs, imu, init, &graph, &values, &uwb_indices);

  std::cout << "\n--- Pass [" << pass_name << "] ---\n";
  std::cout << "  Graph: " << graph.size() << " factors, " << uwb_indices.size()
            << " UWB\n";
  std::cout << "  Calib: lever=" << cfg.calib_lever
            << " anchor=" << cfg.calib_anchor
            << " bias=" << cfg.calib_range_bias << "\n";

  uifgo::Optimizer opt(cfg);
  auto result = opt.Optimize(graph, values, uwb_indices);
  std::cout << "  Result: error=" << result.final_error
            << " chi2=" << result.reduced_chi2
            << " inliers=" << result.inlier_uwb_indices.size() << "/"
            << uwb_indices.size() << "\n";
  return result;
}

// Helper: run one optimization pass with plain LM (no GNC, for synthetic data)
static uifgo::OptimizerResult RunPassPlainLM(
    const uifgo::Config& cfg, const std::vector<uifgo::UwbFrame>& kfs,
    const std::vector<uifgo::ImuSample>& imu, const uifgo::InitResult& init,
    const std::string& pass_name) {
  uifgo::GraphBuilder builder(cfg);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::vector<size_t> uwb_indices;
  builder.Build(kfs, imu, init, &graph, &values, &uwb_indices);

  std::cout << "\n--- Pass [" << pass_name << "] (plain LM) ---\n";
  std::cout << "  Graph: " << graph.size() << " factors, " << uwb_indices.size()
            << " UWB\n";

  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setMaxIterations(cfg.lm_max_iter);
  lm_params.setRelativeErrorTol(cfg.lm_rel_tol);
  lm_params.setAbsoluteErrorTol(cfg.lm_abs_tol);
  lm_params.setLinearSolverType("SEQUENTIAL_CHOLESKY");

  uifgo::OptimizerResult out;
  out.initial_error = graph.error(values);
  gtsam::LevenbergMarquardtOptimizer lmopt(graph, values, lm_params);
  out.values = lmopt.optimize();
  out.final_error = graph.error(out.values);
  out.reduced_chi2 = 0.0;
  for (size_t idx : uwb_indices) out.inlier_uwb_indices.push_back(idx);
  std::cout << "  Result: error=" << out.final_error
            << " inliers=" << out.inlier_uwb_indices.size() << "/"
            << uwb_indices.size() << "\n";
  return out;
}

int main(int argc, char** argv) {
  // Force flush stdout immediately
  setvbuf(stdout, nullptr, _IONBF, 0);
  std::cout << std::unitbuf;

  // Allow startup without rosmaster (we only read rosbag offline)
  ros::init(argc, argv, "uwb_imu_pl");
  ros::NodeHandle nh("~");

  // --- Keep TF "map" frame alive from the very start ---
  // RViz needs a valid Fixed Frame immediately, otherwise even the grid won't
  // render.
  std::atomic<bool> tf_running{true};
  std::thread tf_thread([&]() {
    tf::TransformBroadcaster tf_br;
    tf::Transform tf_id;
    tf_id.setIdentity();
    ros::Rate r(10);  // 10 Hz
    while (tf_running.load() && ros::ok()) {
      tf_br.sendTransform(
          tf::StampedTransform(tf_id, ros::Time::now(), "world", "map"));
      r.sleep();
    }
  });
  ROS_INFO("TF broadcaster started (world→map).");

  // --- Load config ---
  std::string config_path;
  nh.param<std::string>("config_path", config_path, "");
  if (config_path.empty()) {
    std::cerr << "ERROR: config_path parameter required.\n"
              << "Usage: rosrun uwb_imu_pl uwb_imu_pl_node "
                 "_config_path:=/path/to/slam.yaml\n";
    return 1;
  }
  if (!fs::exists(config_path)) {
    std::cerr << "ERROR: config file not found: " << config_path << "\n";
    return 1;
  }

  auto t_start = std::chrono::high_resolution_clock::now();
  std::string config_dir = fs::path(config_path).parent_path().string();
  std::string data_dir = fs::path(config_dir).parent_path().string() + "/data";
  if (!fs::exists(data_dir)) fs::create_directories(data_dir);

  std::cout << "============================================\n";
  std::cout << " UWB-IMU FGO — Batch Multi-Pass Processing\n";
  std::cout << " Config: " << config_path << "\n";
  std::cout << "============================================\n";

  // ============ M1: Config & Data Loading ============
  std::cout << "\n[1/6] Loading data from rosbag..." << std::flush;
  auto t1 = std::chrono::high_resolution_clock::now();
  uifgo::Config base_cfg = uifgo::ConfigLoader::Load(config_path);
  const bool is_mcd = base_cfg.data_interface == "mcd";
  const bool is_viral = base_cfg.data_interface == "viral";
  const bool is_viunet = base_cfg.data_interface == "viunet";
  const bool is_miluv = base_cfg.data_interface == "miluv";
  const bool is_sfuise = base_cfg.data_interface == "sfuise";
  std::string imu_bag_path;
  std::string uwb_bag_path;
  std::string gt_csv_path;
  std::string bag_path;  // primary bag, also used as the run/log identifier
  if (is_mcd) {
    imu_bag_path =
        uifgo::ConfigLoader::ResolveBagPath(config_dir, base_cfg.imu_bag_path);
    uwb_bag_path =
        uifgo::ConfigLoader::ResolveBagPath(config_dir, base_cfg.uwb_bag_path);
    gt_csv_path =
        uifgo::ConfigLoader::ResolveBagPath(config_dir, base_cfg.gt_csv_path);
    bag_path = uwb_bag_path;
    std::cout << "Interface: MCD split bags + CSV GT\n";
    std::cout << "IMU bag:   " << imu_bag_path << "\n";
    std::cout << "UWB bag:   " << uwb_bag_path << "\n";
    std::cout << "GT CSV:    " << gt_csv_path << "\n";
  } else if (is_viral) {
    bag_path =
        uifgo::ConfigLoader::ResolveBagPath(config_dir, base_cfg.bag_path);
    // Override topics for VIRAL dataset convention.
    base_cfg.imu_topic = base_cfg.viral_imu_topic;
    base_cfg.uwb_topic = base_cfg.viral_uwb_topic;
    base_cfg.vicon_topic = base_cfg.viral_gt_topic;
    base_cfg.gt_odom_topic = "";
    std::cout << "Interface: NTU VIRAL single bag\n";
    std::cout << "Bag:       " << bag_path << "\n";
    std::cout << "IMU topic: " << base_cfg.imu_topic << "\n";
    std::cout << "UWB topic: " << base_cfg.uwb_topic << "\n";
    std::cout << "GT topic:  " << base_cfg.vicon_topic << "\n";
  } else if (is_viunet) {
    bag_path = uifgo::ConfigLoader::ResolveBagPath(config_dir,
                                                   base_cfg.viunet_data_dir);
    std::cout << "Interface: VIUNet CSV\n";
    std::cout << "Data dir:  " << bag_path << "\n";
    std::cout << "IMU csv:   " << base_cfg.viunet_imu_csv << "\n";
    std::cout << "UWB csv:   " << base_cfg.viunet_uwb_csv << "\n";
    std::cout << "GT csv:    " << base_cfg.viunet_gt_csv << "\n";
    std::cout << "Y-up→ENU:  "
              << (base_cfg.viunet_rotate_imu_yup ? "yes" : "no") << "\n";
  } else if (is_miluv) {
    bag_path = uifgo::ConfigLoader::ResolveBagPath(config_dir,
                                                   base_cfg.miluv_data_dir);
    std::cout << "Interface: MILUV CSV (PX4 IMU + Decawave UWB + Mocap)\n";
    std::cout << "Data dir:  " << bag_path << "\n";
    std::cout << "Robot:     " << base_cfg.miluv_robot_dir << "\n";
    std::cout << "IMU csv:   " << base_cfg.miluv_imu_csv << "\n";
    std::cout << "UWB csv:   " << base_cfg.miluv_uwb_csv << "\n";
    std::cout << "GT csv:    " << base_cfg.miluv_gt_csv << "\n";
    std::cout << "UWB group: " << base_cfg.miluv_uwb_group_window << "s\n";
  } else if (is_sfuise) {
    bag_path = uifgo::ConfigLoader::ResolveBagPath(config_dir,
                                                   base_cfg.sfuise_data_dir);
    std::cout << "Interface: SFUISE ISAS-Walk" << base_cfg.sfuise_sequence
              << " (ToA UWB+IMU+GT)\n";
    std::cout << "Data dir:  " << bag_path << "\n";
    std::cout << "Sequence:  " << base_cfg.sfuise_sequence << "\n";
    std::cout << "IMU topic: " << base_cfg.sfuise_imu_topic << "\n";
    std::cout << "UWB topic: " << base_cfg.sfuise_uwb_topic << "\n";
    std::cout << "GT topic:  " << base_cfg.sfuise_gt_topic << "\n";
    std::cout << "UWB group: " << base_cfg.sfuise_uwb_group_window << "s\n";
  } else {
    bag_path =
        uifgo::ConfigLoader::ResolveBagPath(config_dir, base_cfg.bag_path);
    std::cout << "Interface: original single bag\n";
    std::cout << "Bag:       " << bag_path << "\n";
  }
  std::cout << "Anchors:   " << base_cfg.anchors.size() << "\n";
  for (const auto& a : base_cfg.anchors)
    std::cout << "  #" << a.id << " @ [" << a.pos.x() << "," << a.pos.y() << ","
              << a.pos.z() << "]\n";

  // --- Debug logger ---
  uifgo::Logger logger;
  if (base_cfg.debug_log) {
    logger.Init(config_dir, bag_path);
    logger.LogConfig(base_cfg);
  }

  uifgo::DataLoader loader(base_cfg);
  std::vector<uifgo::ImuSample> imu_samples;
  std::vector<uifgo::UwbFrame> uwb_frames;
  bool load_ok = false;

  if (is_mcd) {
    load_ok = loader.LoadFromBags(imu_bag_path, uwb_bag_path, &imu_samples,
                                  &uwb_frames);
  } else if (is_viral) {
    // VIRAL: auto-extract anchors from UWB messages.
    std::vector<uifgo::AnchorConfig> viral_anchors;
    load_ok = loader.LoadViralBag(bag_path, &imu_samples, &uwb_frames,
                                  &viral_anchors);
    // If auto-anchors were discovered and no anchors were provided in config,
    // use the auto-discovered ones.
    if (!viral_anchors.empty() && base_cfg.anchors.empty()) {
      base_cfg.anchors = std::move(viral_anchors);
      // Rebuild per-anchor prior sigma list.
      base_cfg.anchor_prior_sigmas.clear();
      for (const auto& a : base_cfg.anchors)
        base_cfg.anchor_prior_sigmas.push_back(a.prior_sigma);
      std::cout << "VIRAL: using " << base_cfg.anchors.size()
                << " auto-discovered anchors.\n";
    }
  } else if (is_viunet) {
    // VIUNet: CSV-based dataset, anchors auto-extracted from UWB CSV.
    std::string viunet_data_dir = uifgo::ConfigLoader::ResolveBagPath(
        config_dir, base_cfg.viunet_data_dir);
    std::vector<uifgo::AnchorConfig> viunet_anchors;
    load_ok = loader.LoadViunetCsv(viunet_data_dir, &imu_samples, &uwb_frames,
                                   &viunet_anchors);
    if (!viunet_anchors.empty() && base_cfg.anchors.empty()) {
      base_cfg.anchors = std::move(viunet_anchors);
      base_cfg.anchor_prior_sigmas.clear();
      for (const auto& a : base_cfg.anchors)
        base_cfg.anchor_prior_sigmas.push_back(a.prior_sigma);
      std::cout << "VIUNet: using " << base_cfg.anchors.size()
                << " anchors from UWB CSV.\n";
    }
  } else if (is_miluv) {
    std::string miluv_data_dir = uifgo::ConfigLoader::ResolveBagPath(
        config_dir, base_cfg.miluv_data_dir);
    std::vector<uifgo::AnchorConfig> miluv_anchors;
    load_ok = loader.LoadMiluvCsv(miluv_data_dir, &imu_samples, &uwb_frames,
                                  &miluv_anchors);
    if (base_cfg.anchors.empty() && !miluv_anchors.empty()) {
      base_cfg.anchors = std::move(miluv_anchors);
      base_cfg.anchor_prior_sigmas.clear();
      for (const auto& a : base_cfg.anchors)
        base_cfg.anchor_prior_sigmas.push_back(a.prior_sigma);
      std::cout << "MILUV: using " << base_cfg.anchors.size()
                << " anchors from config.\n";
    }
  } else if (is_sfuise) {
    std::string sfuise_data_dir = uifgo::ConfigLoader::ResolveBagPath(
        config_dir, base_cfg.sfuise_data_dir);
    std::vector<uifgo::AnchorConfig> sfuise_anchors;
    load_ok = loader.LoadSfuiseBag(sfuise_data_dir, base_cfg.sfuise_sequence,
                                   &imu_samples, &uwb_frames, &sfuise_anchors);
    if (base_cfg.anchors.empty() && !sfuise_anchors.empty()) {
      base_cfg.anchors = std::move(sfuise_anchors);
      base_cfg.anchor_prior_sigmas.clear();
      for (const auto& a : base_cfg.anchors)
        base_cfg.anchor_prior_sigmas.push_back(a.prior_sigma);
      std::cout << "SFUISE: using " << base_cfg.anchors.size()
                << " auto-discovered anchors.\n";
    }
  } else {
    load_ok = loader.LoadFromBag(bag_path, &imu_samples, &uwb_frames);
  }
  if (!load_ok) {
    std::cerr << "ERROR: failed to load data from bag.\n";
    return 1;
  }
  auto t1e = std::chrono::high_resolution_clock::now();
  std::cout << " done (" << std::chrono::duration<double>(t1e - t1).count()
            << "s, " << imu_samples.size() << " IMU, " << uwb_frames.size()
            << " UWB frames)\n";

  // --- Outlier pre-filtering ---
  std::cout << "[2/6] Pre-filtering + keyframe downsampling..." << std::flush;
  auto t2 = std::chrono::high_resolution_clock::now();
  uifgo::OutlierFilter filter(base_cfg);
  std::vector<uifgo::UwbFrame> filtered_uwb;
  for (const auto& f : uwb_frames) {
    auto retained = filter.PreFilter(f);
    if (!retained.empty()) {
      uifgo::UwbFrame ff = f;
      ff.ranges = retained;
      filtered_uwb.push_back(ff);
    }
  }
  std::cout << "UWB frames: " << uwb_frames.size() << " → "
            << filtered_uwb.size() << " (after NLOS/range filter)\n";

  // Keyframe downsampling
  if (base_cfg.kf_step > 1) {
    std::vector<uifgo::UwbFrame> downsampled;
    for (size_t i = 0; i < filtered_uwb.size(); i += base_cfg.kf_step) {
      downsampled.push_back(filtered_uwb[i]);
    }
    std::cout << "Keyframes:  " << filtered_uwb.size() << " → "
              << downsampled.size() << " (step=" << base_cfg.kf_step << ")\n";
    filtered_uwb = downsampled;
  }
  auto t2e = std::chrono::high_resolution_clock::now();
  std::cout << " done (" << std::chrono::duration<double>(t2e - t2).count()
            << "s, " << filtered_uwb.size() << " keyframes)\n";

  if (filtered_uwb.empty()) {
    std::cerr << "ERROR: no UWB data after filtering.\n";
    return 1;
  }

  // ============ M2: Initialization ============
  std::cout
      << "[3/6] Initialization (static detect + trilateration + yaw align)..."
      << std::flush;
  auto t3 = std::chrono::high_resolution_clock::now();
  uifgo::Initializer init(base_cfg);
  auto init_result = init.Run(imu_samples, filtered_uwb);
  auto t3e = std::chrono::high_resolution_clock::now();
  std::cout << " done (" << std::chrono::duration<double>(t3e - t3).count()
            << "s, p0=[" << init_result.T0.translation().transpose() << "])\n";

  // ============ M5-M7: Multi-Pass Optimization ============
  std::cout << "[4/6] Multi-pass optimization...\n";
  auto t4 = std::chrono::high_resolution_clock::now();
  // Pass 1 — All calibration OFF: GNC+TLS robust trajectory initialization
  //         (setKnownInliers pins IMU/priors; only UWB factors are weighted)
  auto cfg_pass1 = base_cfg;
  cfg_pass1.calib_lever = false;
  cfg_pass1.calib_anchor = false;
  cfg_pass1.calib_range_bias = false;
  auto res1 = RunPass(cfg_pass1, filtered_uwb, imu_samples, init_result,
                      "Pass 1: GNC Robust Init (no calib)");
  auto best_values = res1.values;
  auto best_inliers = res1.inlier_uwb_indices;

  // If calibration is enabled in config, run progressive passes
  if (base_cfg.calib_range_bias || base_cfg.calib_anchor ||
      base_cfg.calib_lever) {
    bool need_final_pass = false;  // true if Pass Final adds new calib vars

    // Pass 2 — Open range bias only
    if (base_cfg.calib_range_bias) {
      auto cfg_pass2 = base_cfg;
      cfg_pass2.calib_lever = false;
      cfg_pass2.calib_anchor = false;
      cfg_pass2.calib_range_bias = true;
      auto res2 = RunPass(cfg_pass2, filtered_uwb, imu_samples, init_result,
                          "Pass 2: Open range bias");
      if (res2.reduced_chi2 < res1.reduced_chi2 * 1.5) {
        best_values = res2.values;
        best_inliers = res2.inlier_uwb_indices;
      } else {
        std::cout << "  [WARN] range bias calib degraded chi2, keeping Pass 1 "
                     "result.\n";
      }
    }

    // Pass 3 — Open anchor correction
    if (base_cfg.calib_anchor) {
      auto cfg_pass3 = base_cfg;
      cfg_pass3.calib_lever = false;
      cfg_pass3.calib_anchor = true;
      cfg_pass3.calib_range_bias = base_cfg.calib_range_bias;
      auto res3 = RunPass(cfg_pass3, filtered_uwb, imu_samples, init_result,
                          "Pass 3: Open anchor calib");
      if (res3.reduced_chi2 < res1.reduced_chi2 * 1.5) {
        best_values = res3.values;
        best_inliers = res3.inlier_uwb_indices;
      }
      // If lever is disabled, Pass 3 already covers all enabled calib vars
      if (!base_cfg.calib_lever)
        need_final_pass = false;
      else
        need_final_pass = true;
    }

    // Pass 4 — Open lever arm
    if (base_cfg.calib_lever) {
      auto cfg_pass4 = base_cfg;
      cfg_pass4.calib_lever = true;
      cfg_pass4.calib_anchor = base_cfg.calib_anchor;
      cfg_pass4.calib_range_bias = base_cfg.calib_range_bias;
      auto res4 = RunPass(cfg_pass4, filtered_uwb, imu_samples, init_result,
                          "Pass 4: Open lever arm");
      if (res4.reduced_chi2 < res1.reduced_chi2 * 1.5) {
        best_values = res4.values;
        best_inliers = res4.inlier_uwb_indices;
      }
      need_final_pass = true;  // Pass 4 only opens lever; final joint needed
    }

    // Pass Final — All calibration open, joint refinement
    // Only run when it actually adds new calibration variables vs prior passes.
    if (need_final_pass) {
      auto res_final = RunPass(base_cfg, filtered_uwb, imu_samples, init_result,
                               "Pass Final: All calib joint");
      if (res_final.reduced_chi2 < res1.reduced_chi2 * 2.0) {
        best_values = res_final.values;
        best_inliers = res_final.inlier_uwb_indices;
      }
    } else {
      std::cout << "\n>>> Skipping Pass Final (all enabled calib vars already "
                   "covered by Pass 3).\n";
    }
    std::cout << "\n>>> Final result adopted from best pass.\n";
  }

  auto t4e = std::chrono::high_resolution_clock::now();
  std::cout << "  optimization done ("
            << std::chrono::duration<double>(t4e - t4).count() << "s)\n";

  // ============ M7: Covariance on best result ============
  std::cout << "[5/6] Covariance diagnostics..." << std::flush;
  auto t5 = std::chrono::high_resolution_clock::now();
  // Build a clean graph for marginalization using the best result directly,
  // without re-optimizing (best_values is already optimal from multi-pass).
  uifgo::GraphBuilder final_builder(base_cfg);
  gtsam::NonlinearFactorGraph final_graph;
  gtsam::Values final_init;
  std::vector<size_t> final_uwb_idx;
  final_builder.Build(filtered_uwb, imu_samples, init_result, &final_graph,
                      &final_init, &final_uwb_idx);
  uifgo::Optimizer final_opt(base_cfg);
  // Use best_values as initial guess so the optimizer starts near the optimum;
  // this makes the subsequent LM solve fast (often 1–2 iterations).
  final_opt.Optimize(final_graph, best_values, final_uwb_idx);

  // Covariance diagnostics
  std::cout << "\n============ Covariance Diagnostics ============\n";
  try {
    auto P0 = final_opt.PoseCovariance(0);
    if (P0.rows() > 0) {
      std::cout << "Pose[0] σ_xyz: " << std::sqrt(P0(0, 0)) << " "
                << std::sqrt(P0(1, 1)) << " " << std::sqrt(P0(2, 2))
                << " (m)\n";
    }

    size_t mid = filtered_uwb.size() / 2;
    auto Pm = final_opt.PoseCovariance(mid);
    if (Pm.rows() > 0) {
      std::cout << "Pose[" << mid << "] σ_xyz: " << std::sqrt(Pm(0, 0)) << " "
                << std::sqrt(Pm(1, 1)) << " " << std::sqrt(Pm(2, 2))
                << " (m)\n";
    }

    auto Pn = final_opt.PoseCovariance(filtered_uwb.size() - 1);
    if (Pn.rows() > 0) {
      std::cout << "Pose[" << filtered_uwb.size() - 1
                << "] σ_xyz: " << std::sqrt(Pn(0, 0)) << " "
                << std::sqrt(Pn(1, 1)) << " " << std::sqrt(Pn(2, 2))
                << " (m)\n";
    }

    // --- Per-keyframe σ_z diagnostic (every 10th keyframe) ---
    std::cout << "\n--- Per-Keyframe σ_z (height uncertainty) ---\n";
    size_t kf_count = filtered_uwb.size();
    for (size_t k = 0; k < kf_count; k += std::max(size_t(1), kf_count / 20)) {
      auto Pk = final_opt.PoseCovariance(k);
      if (Pk.rows() == 0) continue;
      double sigma_z = std::sqrt(Pk(2, 2));
      std::cout << "  KF[" << k << "] σ_z=" << sigma_z << " m";
      if (k > 0 && k % 5 == 0) std::cout << "\n";
    }
    std::cout << "\n";

    if (base_cfg.calib_lever) {
      auto PL = final_opt.LeverCovariance();
      std::cout << "Lever σ: " << std::sqrt(PL(0, 0)) << " "
                << std::sqrt(PL(1, 1)) << " " << std::sqrt(PL(2, 2))
                << " (m)\n";
    }
    for (const auto& a : base_cfg.anchors) {
      if (base_cfg.calib_anchor) {
        auto PA = final_opt.AnchorCovariance(a.id);
        if (PA.rows() == 3)
          std::cout << "Anchor[" << a.id << "] σ: " << std::sqrt(PA(0, 0))
                    << " " << std::sqrt(PA(1, 1)) << " " << std::sqrt(PA(2, 2))
                    << " (m)\n";
      }
    }
  } catch (const std::exception& e) {
    std::cout << "Marginals: " << e.what()
              << " (some variables not observable)\n";
  }

  auto t5e = std::chrono::high_resolution_clock::now();
  std::cout << " done (" << std::chrono::duration<double>(t5e - t5).count()
            << "s)\n";

  // ============ M8: Output ============
  std::cout << "[6/6] Writing output..." << std::flush;
  auto t6 = std::chrono::high_resolution_clock::now();
  std::vector<double> kf_times;
  for (const auto& f : filtered_uwb) kf_times.push_back(f.t);
  auto traj = uifgo::TrajectoryIO::ExtractTrajectory(best_values, kf_times);

  // --- Log state trace, residuals, covariance ---
  if (logger.enabled()) {
    logger.LogStateTrace(traj);
    logger.LogResiduals(traj, filtered_uwb, base_cfg.anchors);
    logger.LogCovarianceDiag(traj, res1, base_cfg);
    // Log Pass 1 baseline metrics
    logger.LogPass("Pass1_RobustInit", res1, 0.0, final_graph.size(),
                   final_uwb_idx.size());
  }

  uifgo::TrajectoryIO::WriteTum(data_dir + "/trajectory.txt", traj);
  uifgo::TrajectoryIO::WriteCalib(data_dir + "/calibration.txt", best_values,
                                  base_cfg.anchors);

  // --- Copy trajectory + calibration to log dir ---
  if (logger.enabled()) {
    logger.LogTrajectory(data_dir + "/trajectory.txt", "trajectory");
    logger.LogCalibration(data_dir + "/calibration.txt");
  }

  // Also copy the original calibration config to data/ for reference
  std::string calib_src = config_dir + "/calibration.txt";
  if (fs::exists(calib_src)) {
    std::string calib_dst = data_dir + "/calibration_config.txt";
    fs::copy_file(calib_src, calib_dst, fs::copy_option::overwrite_if_exists);
    std::cout << "TrajectoryIO: copied " << calib_src << " -> " << calib_dst
              << "\n";
  }

  // ============ Ground Truth Comparison (VICON + Odometry fallback)
  // ============
  uifgo::DataLoader gt_loader(base_cfg);
  std::vector<uifgo::NavState> gt_traj;
  if (is_miluv) {
    std::string miluv_data_dir = uifgo::ConfigLoader::ResolveBagPath(
        config_dir, base_cfg.miluv_data_dir);
    std::string gt_csv = miluv_data_dir + "/" + base_cfg.miluv_robot_dir + "/" +
                         base_cfg.miluv_gt_csv;
    gt_traj = gt_loader.LoadGroundTruthMiluv(gt_csv);
  } else if (is_viunet) {
    std::string viunet_data_dir = uifgo::ConfigLoader::ResolveBagPath(
        config_dir, base_cfg.viunet_data_dir);
    std::string gt_csv = viunet_data_dir + "/" + base_cfg.viunet_gt_csv;
    gt_traj = gt_loader.LoadGroundTruthViunet(gt_csv);
  } else if (is_sfuise) {
    std::string sfuise_data_dir = uifgo::ConfigLoader::ResolveBagPath(
        config_dir, base_cfg.sfuise_data_dir);
    std::string sfuise_bag = sfuise_data_dir + "/ISAS-Walk" +
                             std::to_string(base_cfg.sfuise_sequence) + ".bag";
    gt_traj = gt_loader.LoadGroundTruthSfuise(sfuise_bag);
  } else if (is_mcd) {
    gt_traj = gt_loader.LoadGroundTruthCsv(gt_csv_path);
  } else {
    gt_traj = gt_loader.LoadGroundTruth(bag_path);
    // If no VICON PoseStamped GT, try Odometry-based GT (e.g. /sim/odom)
    if (gt_traj.empty()) {
      gt_traj = gt_loader.LoadGroundTruthOdom(bag_path);
    }
  }
  if (!gt_traj.empty()) {
    uifgo::TrajectoryIO::WriteTum(data_dir + "/groundtruth.txt", gt_traj);
    if (logger.enabled()) logger.LogGroundTruth(data_dir + "/groundtruth.txt");
  }
  double ate_rmse = -1.0, p50 = -1.0, p95 = -1.0, p99 = -1.0, max_err = -1.0;
  size_t gt_match_count = 0;
  uifgo::ATEStats stats;  // kept in outer scope for synthetic ablation

  if (!gt_traj.empty()) {
    // Compute ATE with SE(3) Umeyama alignment (handles different GT frames).
    stats = uifgo::TrajectoryIO::ComputeATE(traj, gt_traj);
    if (stats.ok) {
      ate_rmse = stats.rmse;
      p50 = stats.p50;
      p95 = stats.p95;
      p99 = stats.p99;
      max_err = stats.max_err;
      gt_match_count = stats.matched;
    }

    std::cout << "\n--- Ground Truth Comparison ---\n";
    std::cout << "GT source:      "
              << (is_sfuise  ? base_cfg.sfuise_gt_topic
                  : is_miluv ? base_cfg.miluv_data_dir + "/" +
                                   base_cfg.miluv_robot_dir + "/" +
                                   base_cfg.miluv_gt_csv
                  : is_viunet
                      ? base_cfg.viunet_data_dir + "/" + base_cfg.viunet_gt_csv
                  : is_mcd ? gt_csv_path
                           : base_cfg.vicon_topic)
              << "\n";
    std::cout << "GT poses:       " << gt_traj.size() << "\n";
    std::cout << "Matched frames: " << gt_match_count << "\n";
    if (stats.ok) {
      std::cout << "Align scale:    " << stats.scale << "\n";
      std::cout << "Align R:\n" << stats.T_align.rotation().matrix() << "\n";
      std::cout << "Align t:        " << stats.T_align.translation().transpose()
                << "\n";
    }
    std::cout << "ATE RMSE:       " << ate_rmse << " m\n";
    std::cout << "P50 error:      " << p50 << " m\n";
    std::cout << "P95 error:      " << p95 << " m\n";
    std::cout << "P99 error:      " << p99 << " m\n";
    std::cout << "Max error:      " << max_err << " m\n";

    // --- Log GT comparison ---
    if (logger.enabled())
      logger.LogGtComparison(ate_rmse, p50, p95, p99, max_err, gt_match_count,
                             gt_traj.size());
  } else {
    std::cout << "\n[INFO] No ground truth loaded.\n";
  }

  // ============ Summary ============
  auto t6e = std::chrono::high_resolution_clock::now();
  std::cout << " done (" << std::chrono::duration<double>(t6e - t6).count()
            << "s)\n";
  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed = std::chrono::duration<double>(t_end - t_start).count();

  std::cout << "\n============================================\n";
  std::cout << " FINAL SUMMARY\n";
  std::cout << "============================================\n";
  std::cout << "Keyframes:        " << filtered_uwb.size() << "\n";
  std::cout << "Total factors:    " << final_graph.size() << "\n";
  std::cout << "UWB inliers:      " << best_inliers.size() << " / "
            << final_uwb_idx.size() << "\n";
  std::cout << "Trajectory length:" << kf_times.back() - kf_times.front()
            << " s\n";

  // Per-anchor range statistics from best_values
  std::cout << "\nPer-Anchor Residuals (on best result):\n";
  std::map<int, double> anchor_rmse_map;
  for (const auto& a : base_cfg.anchors) {
    double sum_r2 = 0;
    int count = 0;
    for (size_t ki = 0; ki < filtered_uwb.size(); ++ki) {
      for (const auto& r : filtered_uwb[ki].ranges) {
        if (r.anchor_id != a.id) continue;
        gtsam::Pose3 Tk = best_values.at<gtsam::Pose3>(gtsam::Symbol('x', ki));
        gtsam::Point3 tag = Tk.translation();
        double pred = (gtsam::Vector3(tag) - gtsam::Vector3(a.pos)).norm();
        double res = pred - r.dist;
        sum_r2 += res * res;
        ++count;
      }
    }
    if (count > 0) {
      double rmse = std::sqrt(sum_r2 / count);
      anchor_rmse_map[a.id] = rmse;
      std::cout << "  Anchor #" << a.id << ": " << count
                << " ranges, RMSE=" << rmse << " m\n";
    }
  }

  std::cout << "\nElapsed:           " << elapsed << " s\n";
  std::cout << "ATE RMSE:           " << ate_rmse << " m\n";
  std::cout << "P95 Error:          " << p95 << " m\n";
  std::cout << "Output:             " << data_dir << "/trajectory.txt\n";
  std::cout << "GroundTruth:        " << data_dir << "/groundtruth.txt\n";
  std::cout << "Calibration:        " << data_dir << "/calibration.txt\n";

  // ============ Synthetic Anchor Ablation ============
  if (base_cfg.synthetic_enabled && stats.ok && !gt_traj.empty()) {
    std::cout << "\n============================================\n";
    std::cout << " SYNTHETIC ANCHOR ABLATION STUDY\n";
    std::cout << "============================================\n";
    std::cout << "Baseline ATE: " << stats.rmse << " m (" << stats.matched
              << " matched poses)\n";
    std::cout << "Noise sigma:  " << base_cfg.synthetic_sigma_range << " m\n";
    std::cout << "Synth anchors: " << base_cfg.synthetic_anchors.size()
              << " defined\n";

    // SE(3): estimate → GT. Invert for GT → anchor frame.
    gtsam::Pose3 T_gt_to_anchor = stats.T_align.inverse();
    const double syn_sigma = base_cfg.synthetic_sigma_range;

    // Pre-transform all GT positions to anchor frame
    std::vector<double> gt_t;
    std::vector<Eigen::Vector3d> gt_pa;
    for (const auto& g : gt_traj) {
      gtsam::Point3 pa = T_gt_to_anchor.transformFrom(g.T.translation());
      gt_t.push_back(g.t);
      gt_pa.push_back(Eigen::Vector3d(pa.x(), pa.y(), pa.z()));
    }

    std::mt19937 rng(42);
    std::normal_distribution<double> ndist(0.0, syn_sigma);

    // Summary rows
    struct SR {
      int n;
      double ate, p50, p95, p99, mx;
      int inl, tot;
      double c2;
    };
    std::vector<SR> sres;

    for (int ns : base_cfg.synthetic_test_counts) {
      if (ns > (int)base_cfg.synthetic_anchors.size()) {
        std::cout << "  Skipping N=" << ns << " (only "
                  << base_cfg.synthetic_anchors.size() << " defined)\n";
        continue;
      }

      std::cout << "\n--- Synthetic N=" << ns << " ---\n" << std::flush;

      // Augment anchors
      uifgo::Config acfg = base_cfg;
      acfg.anchors = base_cfg.anchors;
      for (int i = 0; i < ns; ++i)
        acfg.anchors.push_back(base_cfg.synthetic_anchors[i]);
      acfg.anchor_prior_sigmas.clear();
      for (const auto& a : acfg.anchors)
        acfg.anchor_prior_sigmas.push_back(a.prior_sigma);
      acfg.calib_anchor = false;
      acfg.calib_range_bias = false;

      // Augment UWB frames with noise-corrupted synthetic ranges
      auto aug_uwb = uwb_frames;
      int added = 0;
      for (auto& f : aug_uwb) {
        // Interpolate GT position in anchor frame at frame.t
        auto it = std::lower_bound(gt_t.begin(), gt_t.end(), f.t);
        Eigen::Vector3d pa;
        if (it == gt_t.end()) {
          pa = gt_pa.back();
        } else if (it == gt_t.begin()) {
          pa = gt_pa.front();
        } else {
          size_t i2 = it - gt_t.begin(), i1 = i2 - 1;
          double alpha = (f.t - gt_t[i1]) / (gt_t[i2] - gt_t[i1]);
          pa = gt_pa[i1] * (1.0 - alpha) + gt_pa[i2] * alpha;
        }

        for (int i = 0; i < ns; ++i) {
          const auto& sa = base_cfg.synthetic_anchors[i];
          Eigen::Vector3d sap(sa.pos.x(), sa.pos.y(), sa.pos.z());
          double tr = (pa - sap).norm();
          double nr = tr + ndist(rng);
          if (nr < 0.1) nr = 0.1;
          uifgo::UwbRange sr;
          sr.anchor_id = sa.id;
          sr.dist = nr;
          sr.fp_rssi = 0.0;
          sr.rx_rssi = 0.0;
          f.ranges.push_back(sr);
          ++added;
        }
      }
      std::cout << "  Added " << added << " synthetic ranges to "
                << aug_uwb.size() << " frames\n";

      // Filter + downsample
      uifgo::OutlierFilter filt(acfg);
      std::vector<uifgo::UwbFrame> kfs;
      for (const auto& f : aug_uwb) {
        auto ret = filt.PreFilter(f);
        if (!ret.empty()) {
          uifgo::UwbFrame ff = f;
          ff.ranges = ret;
          kfs.push_back(ff);
        }
      }
      if (acfg.kf_step > 1) {
        std::vector<uifgo::UwbFrame> ds;
        for (size_t i = 0; i < kfs.size(); i += acfg.kf_step)
          ds.push_back(kfs[i]);
        kfs = ds;
      }
      if (kfs.empty()) {
        std::cerr << "  ERROR: no keyframes\n";
        continue;
      }
      std::cout << "  Keyframes: " << kfs.size() << "\n";

      // Init + optimize
      uifgo::Initializer init(acfg);
      auto ires = init.Run(imu_samples, kfs);
      acfg.calib_lever = true;
      auto ores = RunPassPlainLM(acfg, kfs, imu_samples, ires, "Synth");

      // ATE
      std::vector<double> kt;
      for (const auto& f : kfs) kt.push_back(f.t);
      auto straj = uifgo::TrajectoryIO::ExtractTrajectory(ores.values, kt);
      auto sstat = uifgo::TrajectoryIO::ComputeATE(straj, gt_traj);

      SR r;
      r.n = ns;
      r.ate = sstat.ok ? sstat.rmse : -1.0;
      r.p50 = sstat.ok ? sstat.p50 : -1.0;
      r.p95 = sstat.ok ? sstat.p95 : -1.0;
      r.p99 = sstat.ok ? sstat.p99 : -1.0;
      r.mx = sstat.ok ? sstat.max_err : -1.0;
      r.inl = ores.inlier_uwb_indices.size();
      r.tot = ores.inlier_uwb_indices.size() + ores.outlier_uwb_indices.size();
      r.c2 = ores.reduced_chi2;
      sres.push_back(r);

      std::cout << "  ATE: " << r.ate << " m  P95: " << r.p95
                << " m  inliers: " << r.inl << "/" << r.tot
                << "  chi2: " << r.c2 << "\n";

      // Save trajectory
      std::string od = data_dir + "/synthetic_n" + std::to_string(ns);
      fs::create_directories(od);
      uifgo::TrajectoryIO::WriteTum(od + "/trajectory.txt", straj);
    }

    // Print summary table
    std::cout << "\n--- Synthetic Anchor Ablation Summary ---\n";
    std::cout << " N  | ATE(m) | P50(m) | P95(m) | P99(m) | Max(m) | Inl/Tot | "
                 "chi2\n";
    std::cout << " ---|--------|--------|--------|--------|--------|---------|-"
                 "-----\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << std::setw(3) << 0 << " | " << std::setw(6) << stats.rmse
              << " | " << std::setw(6) << stats.p50 << " | " << std::setw(6)
              << stats.p95 << " | " << std::setw(6) << stats.p99 << " | "
              << std::setw(6) << stats.max_err << " |   ---   |  ---\n";
    for (const auto& r : sres) {
      std::cout << std::setw(3) << r.n << " | " << std::setw(6) << r.ate
                << " | " << std::setw(6) << r.p50 << " | " << std::setw(6)
                << r.p95 << " | " << std::setw(6) << r.p99 << " | "
                << std::setw(6) << r.mx << " | " << std::setw(4) << r.inl << "/"
                << std::setw(4) << r.tot << " | " << r.c2 << "\n";
    }
    std::cout << "============================================\n";
  }

  // --- Log Summary ---
  if (logger.enabled()) {
    logger.LogSummary(elapsed, filtered_uwb.size(), final_graph.size(),
                      best_inliers.size(), final_uwb_idx.size(), ate_rmse, p95,
                      anchor_rmse_map);
    logger.Close();
  }

  // ============ M9: RViz Visualization ============
  {
    uifgo::VizPublisher viz(nh);
    viz.PublishAll(base_cfg, best_values, traj, gt_traj, filtered_uwb,
                   final_graph, &res1, ate_rmse, p95, elapsed);

    ROS_INFO("Visualization published. TF is live. Press Ctrl-C to exit.");
    ros::Rate r(10);
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
  }

  // Clean up TF thread
  tf_running.store(false);
  if (tf_thread.joinable()) tf_thread.join();

  return 0;
}
