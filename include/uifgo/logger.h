#pragma once

#include <gtsam/nonlinear/Values.h>

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "uifgo/config.h"
#include "uifgo/optimizer.h"
#include "uifgo/types.h"

namespace uifgo {

/// Debug logging system for UWB-IMU FGO.
///
/// Creates timestamped log directories under logs/ with a "latest" symlink.
/// Saves intermediate optimization data in CSV/JSON for post-run analysis.
///
/// Usage:
///   Logger logger;
///   if (cfg.debug_log) {
///     logger.Init(config_dir, bag_path);
///     logger.LogConfig(cfg);
///   }
///   // ... after each pass ...
///   logger.LogPass(name, result, elapsed, n_factors, n_uwb);
///   // ... after trajectory ...
///   logger.LogStateTrace(traj);
///   logger.LogResiduals(traj, uwb_frames, anchors);
///   // ... finalize ...
///   logger.LogSummary(...);
///   logger.Close();   // creates logs/latest → this run
class Logger {
 public:
  Logger() = default;
  ~Logger();

  /// Initialize: create logs/<timestamp>_<bag_shortname>/
  /// Returns the log directory path (empty if disabled).
  std::string Init(const std::string& config_dir, const std::string& bag_path);

  /// Save a snapshot of the config used for this run.
  void LogConfig(const Config& cfg);

  /// Per-optimization-pass metrics → optimization.csv
  void LogPass(const std::string& pass_name, const OptimizerResult& result,
               double elapsed_sec, size_t n_factors, size_t n_uwb);

  /// Per-keyframe state → state_trace.csv (t,x,y,z,qw,qx,qy,qz,vx,vy,vz)
  void LogStateTrace(const std::vector<NavState>& traj);

  /// Per-keyframe per-anchor residuals → residuals.csv
  void LogResiduals(const std::vector<NavState>& traj,
                    const std::vector<UwbFrame>& filtered_uwb,
                    const std::vector<AnchorConfig>& anchors);

  /// Covariance diagonal over keyframes → covariance_diag.csv
  void LogCovarianceDiag(const std::vector<NavState>& traj,
                         const OptimizerResult& opt_result, const Config& cfg);

  /// Ground truth comparison → gt_comparison.csv
  void LogGtComparison(double ate_rmse, double p50, double p95, double p99,
                       double max_err, size_t matched_frames, size_t gt_total);

  /// Final summary → summary.json
  void LogSummary(double total_elapsed, size_t n_keyframes, size_t n_factors,
                  size_t n_inliers, size_t n_uwb_total, double ate_rmse,
                  double p95_error, const std::map<int, double>& anchor_rmse);

  /// Copy trajectory / GT / calibration files into the log directory.
  void LogTrajectory(const std::string& src_path, const std::string& label);
  void LogGroundTruth(const std::string& src_path);
  void LogCalibration(const std::string& src_path);

  /// Close all file handles and create/update logs/latest symlink.
  void Close();

  bool enabled() const { return enabled_; }
  const std::string& log_dir() const { return log_dir_; }

 private:
  void EnsureOpen();
  static std::string Timestamp();

  bool enabled_ = false;
  std::string log_dir_;
  std::string config_dir_;
  std::ofstream opt_csv_;
  std::ofstream state_csv_;
  std::ofstream resid_csv_;
  std::ofstream cov_csv_;
  bool opt_header_ = false;
  bool state_header_ = false;
  bool resid_header_ = false;
  bool cov_header_ = false;
};

}  // namespace uifgo
