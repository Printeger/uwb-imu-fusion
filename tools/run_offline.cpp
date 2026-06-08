// run_offline.cpp — ROS entry point for batch UWB-IMU FGO processing.
// Multi-pass pipeline: robust init → calibration unlock → joint refinement.
//
// Usage:
//   rosrun uwb_imu_fgo uwb_imu_fgo_node _config_path:=/path/to/slam.yaml

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

int main(int argc, char** argv) {
  // Force flush stdout immediately
  setvbuf(stdout, nullptr, _IONBF, 0);
  std::cout << std::unitbuf;

  // Allow startup without rosmaster (we only read rosbag offline)
  ros::init(argc, argv, "uwb_imu_fgo");
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
              << "Usage: rosrun uwb_imu_fgo uwb_imu_fgo_node "
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
  uifgo::Config base_cfg = uifgo::ConfigLoader::Load(config_path);
  std::string bag_path =
      uifgo::ConfigLoader::ResolveBagPath(config_dir, base_cfg.bag_path);

  std::cout << "Bag:       " << bag_path << "\n";
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
  if (!loader.LoadFromBag(bag_path, &imu_samples, &uwb_frames)) {
    std::cerr << "ERROR: failed to load data from bag.\n";
    return 1;
  }

  // --- Outlier pre-filtering ---
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

  if (filtered_uwb.empty()) {
    std::cerr << "ERROR: no UWB data after filtering.\n";
    return 1;
  }

  // ============ M2: Initialization ============
  uifgo::Initializer init(base_cfg);
  auto init_result = init.Run(imu_samples, filtered_uwb);
  std::cout << "Init:  ok=" << init_result.ok << " p0=["
            << init_result.T0.translation().transpose() << "]\n";

  // ============ M5-M7: Multi-Pass Optimization ============
  // Pass 1 — All calibration OFF: robust trajectory initialization
  auto cfg_pass1 = base_cfg;
  cfg_pass1.calib_lever = false;
  cfg_pass1.calib_anchor = false;
  cfg_pass1.calib_range_bias = false;
  auto res1 = RunPass(cfg_pass1, filtered_uwb, imu_samples, init_result,
                      "Pass 1: Robust Init (no calib)");
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

  // ============ M7: Covariance on best result ============
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
    std::cout << "Pose[0] σ_xyz: " << std::sqrt(P0(0, 0)) << " "
              << std::sqrt(P0(1, 1)) << " " << std::sqrt(P0(2, 2)) << " (m)\n";

    size_t mid = filtered_uwb.size() / 2;
    auto Pm = final_opt.PoseCovariance(mid);
    std::cout << "Pose[" << mid << "] σ_xyz: " << std::sqrt(Pm(0, 0)) << " "
              << std::sqrt(Pm(1, 1)) << " " << std::sqrt(Pm(2, 2)) << " (m)\n";

    auto Pn = final_opt.PoseCovariance(filtered_uwb.size() - 1);
    std::cout << "Pose[" << filtered_uwb.size() - 1
              << "] σ_xyz: " << std::sqrt(Pn(0, 0)) << " "
              << std::sqrt(Pn(1, 1)) << " " << std::sqrt(Pn(2, 2)) << " (m)\n";

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

  // ============ M8: Output ============
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

  // ============ Ground Truth Comparison (VICON) ============
  uifgo::DataLoader gt_loader(base_cfg);
  auto gt_traj = gt_loader.LoadGroundTruth(bag_path);
  if (!gt_traj.empty()) {
    uifgo::TrajectoryIO::WriteTum(data_dir + "/groundtruth.txt", gt_traj);
    if (logger.enabled()) logger.LogGroundTruth(data_dir + "/groundtruth.txt");
  }
  double ate_rmse = -1.0, p50 = -1.0, p95 = -1.0, p99 = -1.0, max_err = -1.0;
  size_t gt_match_count = 0;

  if (!gt_traj.empty()) {
    // Compute per-frame position errors
    std::vector<double> pos_errors;
    for (const auto& e : traj) {
      auto it = std::lower_bound(
          gt_traj.begin(), gt_traj.end(), e.t,
          [](const uifgo::NavState& g, double t) { return g.t < t; });
      if (it == gt_traj.end()) continue;
      const uifgo::NavState* g;
      if (it == gt_traj.begin())
        g = &(*it);
      else {
        double d1 = it->t - e.t, d0 = e.t - (it - 1)->t;
        g = (d0 < d1) ? &(*(it - 1)) : &(*it);
      }
      double err = (gtsam::Vector3(e.T.translation()) -
                    gtsam::Vector3(g->T.translation()))
                       .norm();
      pos_errors.push_back(err);
    }

    if (!pos_errors.empty()) {
      std::sort(pos_errors.begin(), pos_errors.end());
      gt_match_count = pos_errors.size();
      ate_rmse = std::sqrt(
          std::accumulate(pos_errors.begin(), pos_errors.end(), 0.0,
                          [](double s, double v) { return s + v * v; }) /
          pos_errors.size());
      p50 = pos_errors[pos_errors.size() * 50 / 100];
      p95 = pos_errors[pos_errors.size() * 95 / 100];
      p99 = pos_errors[pos_errors.size() * 99 / 100];
      max_err = pos_errors.back();
    }

    std::cout << "\n--- VICON Ground Truth Comparison ---\n";
    std::cout << "GT topic:       " << base_cfg.vicon_topic << "\n";
    std::cout << "GT poses:       " << gt_traj.size() << "\n";
    std::cout << "Matched frames: " << gt_match_count << "\n";
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
    std::cout << "\n[INFO] No VICON ground truth loaded (topic not configured "
                 "or empty).\n";
  }

  // ============ Summary ============
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
  std::cout << "============================================\n";

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
