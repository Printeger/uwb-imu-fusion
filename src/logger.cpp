#include "uifgo/logger.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <sys/stat.h>
#include <unistd.h>

#include <boost/filesystem.hpp>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fs = boost::filesystem;

namespace uifgo {

// ===========================================================================
// Lifecycle
// ===========================================================================
Logger::~Logger() { Close(); }

std::string Logger::Timestamp() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  return ss.str();
}

std::string Logger::Init(const std::string& config_dir,
                         const std::string& bag_path) {
  config_dir_ = config_dir;

  // Derive a short, human-readable name from the bag filename
  std::string bag_short = fs::path(bag_path).stem().string();
  if (bag_short.size() > 50) bag_short = bag_short.substr(0, 50);

  log_dir_ = config_dir + "/../logs/" + Timestamp() + "_" + bag_short;
  if (!fs::exists(log_dir_)) {
    fs::create_directories(log_dir_);
  }
  enabled_ = true;
  std::cout << "[Logger] " << log_dir_ << "\n";
  return log_dir_;
}

// ===========================================================================
// Config snapshot
// ===========================================================================
void Logger::LogConfig(const Config& cfg) {
  if (!enabled_) return;
  std::ofstream f(log_dir_ + "/config.yaml");
  if (!f) return;
  f << "# Config snapshot — " << Timestamp() << "\n\n";
  f << "anchors:\n";
  for (const auto& a : cfg.anchors) {
    f << "  - {id: " << a.id << ", pos: [" << a.pos.x() << ", " << a.pos.y()
      << ", " << a.pos.z() << "], prior_sigma: " << a.prior_sigma << "}\n";
  }
  f << "\ncalib_lever: " << cfg.calib_lever << "\n";
  f << "calib_anchor: " << cfg.calib_anchor << "\n";
  f << "calib_range_bias: " << cfg.calib_range_bias << "\n";
  f << "lever_prior_sigma: " << cfg.lever_prior_sigma << "\n";
  f << "sigma_range: " << cfg.sigma_range << "\n";
  f << "cauchy_k: " << cfg.cauchy_k << "\n";
  f << "chi2_reject_prob: " << cfg.chi2_reject_prob << "\n";
  f << "max_rejection_rounds: " << cfg.max_rejection_rounds << "\n";
  f << "kf_step: " << cfg.kf_step << "\n";
  f << "lm_max_iter: " << cfg.lm_max_iter << "\n";
  f << "bag_path: " << cfg.bag_path << "\n";
  f.close();
}

// ===========================================================================
// Per-pass optimization metrics
// ===========================================================================
void Logger::LogPass(const std::string& pass_name,
                     const OptimizerResult& result, double elapsed_sec,
                     size_t n_factors, size_t n_uwb) {
  if (!enabled_) return;
  EnsureOpen();
  if (!opt_header_) {
    opt_csv_
        << "pass,initial_error,final_error,reduced_chi2,"
        << "inliers,outliers,n_factors,n_uwb,elapsed_sec,num_reject_rounds\n";
    opt_header_ = true;
  }
  opt_csv_ << "\"" << pass_name << "\"," << result.initial_error << ","
           << result.final_error << "," << result.reduced_chi2 << ","
           << result.inlier_uwb_indices.size() << ","
           << result.outlier_uwb_indices.size() << "," << n_factors << ","
           << n_uwb << "," << elapsed_sec << "," << result.num_rejection_rounds
           << "\n";
  opt_csv_.flush();
}

// ===========================================================================
// Per-keyframe state trace
// ===========================================================================
void Logger::LogStateTrace(const std::vector<NavState>& traj) {
  if (!enabled_ || traj.empty()) return;
  EnsureOpen();
  if (!state_header_) {
    state_csv_ << "t,x,y,z,qw,qx,qy,qz,vx,vy,vz\n";
    state_header_ = true;
  }
  for (const auto& s : traj) {
    auto p = s.T.translation();
    auto q = s.T.rotation().toQuaternion();
    state_csv_ << std::fixed << std::setprecision(6) << s.t << "," << p.x()
               << "," << p.y() << "," << p.z() << "," << q.w() << "," << q.x()
               << "," << q.y() << "," << q.z() << "," << s.v.x() << ","
               << s.v.y() << "," << s.v.z() << "\n";
  }
  state_csv_.flush();
}

// ===========================================================================
// Per-anchor per-keyframe residuals
// ===========================================================================
void Logger::LogResiduals(const std::vector<NavState>& traj,
                          const std::vector<UwbFrame>& filtered_uwb,
                          const std::vector<AnchorConfig>& anchors) {
  if (!enabled_ || traj.empty()) return;
  EnsureOpen();
  if (!resid_header_) {
    resid_csv_ << "kf_idx,t";
    for (const auto& a : anchors)
      resid_csv_ << ",resid_a" << a.id << ",n_a" << a.id;
    resid_csv_ << "\n";
    resid_header_ = true;
  }
  for (size_t i = 0; i < traj.size() && i < filtered_uwb.size(); ++i) {
    resid_csv_ << i << "," << std::fixed << std::setprecision(6) << traj[i].t;
    for (const auto& a : anchors) {
      double sum_r2 = 0.0;
      int count = 0;
      for (const auto& r : filtered_uwb[i].ranges) {
        if (r.anchor_id != a.id) continue;
        double pred = (Eigen::Vector3d(traj[i].T.translation()) -
                       Eigen::Vector3d(a.pos.x(), a.pos.y(), a.pos.z()))
                          .norm();
        double res = pred - r.dist;
        sum_r2 += res * res;
        ++count;
      }
      double rmse = (count > 0) ? std::sqrt(sum_r2 / count) : -1.0;
      resid_csv_ << "," << rmse << "," << count;
    }
    resid_csv_ << "\n";
  }
  resid_csv_.flush();
}

// ===========================================================================
// Covariance diagonal over keyframes
// ===========================================================================
void Logger::LogCovarianceDiag(const std::vector<NavState>& traj,
                               const OptimizerResult& /*opt_result*/,
                               const Config& /*cfg*/) {
  if (!enabled_ || traj.empty()) return;
  EnsureOpen();
  if (!cov_header_) {
    cov_csv_ << "kf_idx,t,sigma_x,sigma_y,sigma_z\n";
    cov_header_ = true;
  }
  // Covariance extraction requires marginals from Optimizer.
  // Write -1.0 as placeholder; real implementation connects to
  // Optimizer::PoseCovariance() after marginal computation.
  for (size_t i = 0; i < traj.size(); ++i) {
    cov_csv_ << i << "," << std::fixed << std::setprecision(6) << traj[i].t
             << ",-1.0,-1.0,-1.0\n";
  }
  cov_csv_.flush();
}

// ===========================================================================
// Ground truth comparison
// ===========================================================================
void Logger::LogGtComparison(double ate_rmse, double p50, double p95,
                             double p99, double max_err, size_t matched_frames,
                             size_t gt_total) {
  if (!enabled_) return;
  std::ofstream f(log_dir_ + "/gt_comparison.csv");
  if (!f) return;
  f << std::fixed << std::setprecision(4);
  f << "metric,value\n";
  f << "ATE_RMSE," << ate_rmse << "\n";
  f << "P50," << p50 << "\n";
  f << "P95," << p95 << "\n";
  f << "P99," << p99 << "\n";
  f << "MaxError," << max_err << "\n";
  f << "MatchedFrames," << matched_frames << "\n";
  f << "GT_Total," << gt_total << "\n";
  f.close();
}

// ===========================================================================
// Final summary JSON
// ===========================================================================
void Logger::LogSummary(double total_elapsed, size_t n_keyframes,
                        size_t n_factors, size_t n_inliers, size_t n_uwb_total,
                        double ate_rmse, double p95_error,
                        const std::map<int, double>& anchor_rmse) {
  if (!enabled_) return;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4);
  ss << "{\n";
  ss << "  \"timestamp\": \"" << Timestamp() << "\",\n";
  ss << "  \"total_elapsed_sec\": " << total_elapsed << ",\n";
  ss << "  \"n_keyframes\": " << n_keyframes << ",\n";
  ss << "  \"n_factors\": " << n_factors << ",\n";
  ss << "  \"n_uwb_inliers\": " << n_inliers << ",\n";
  ss << "  \"n_uwb_total\": " << n_uwb_total << ",\n";
  ss << "  \"inlier_pct\": "
     << (n_uwb_total > 0 ? 100.0 * n_inliers / n_uwb_total : 0.0) << ",\n";
  ss << "  \"ate_rmse_m\": " << ate_rmse << ",\n";
  ss << "  \"p95_error_m\": " << p95_error << ",\n";
  ss << "  \"anchor_rmse_m\": {";
  bool first = true;
  for (const auto& kv : anchor_rmse) {
    if (!first) ss << ", ";
    ss << "\"a" << kv.first << "\": " << kv.second;
    first = false;
  }
  ss << "}\n";
  ss << "}\n";

  std::ofstream f(log_dir_ + "/summary.json");
  f << ss.str();
  f.close();
}

// ===========================================================================
// Copy trajectory / GT / calibration files into log dir
// ===========================================================================
void Logger::LogTrajectory(const std::string& src_path,
                           const std::string& label) {
  if (!enabled_ || !fs::exists(src_path)) return;
  std::string dst = log_dir_ + "/" + label + ".txt";
  fs::copy_file(src_path, dst, fs::copy_option::overwrite_if_exists);
  std::cout << "[Logger] " << label << " → " << dst << "\n";
}

void Logger::LogGroundTruth(const std::string& src_path) {
  LogTrajectory(src_path, "groundtruth");
}

void Logger::LogCalibration(const std::string& src_path) {
  if (!enabled_ || !fs::exists(src_path)) return;
  std::string dst = log_dir_ + "/calibration.txt";
  fs::copy_file(src_path, dst, fs::copy_option::overwrite_if_exists);
  std::cout << "[Logger] calibration → " << dst << "\n";
}

// ===========================================================================
// Close & symlink
// ===========================================================================
void Logger::Close() {
  if (opt_csv_.is_open()) opt_csv_.close();
  if (state_csv_.is_open()) state_csv_.close();
  if (resid_csv_.is_open()) resid_csv_.close();
  if (cov_csv_.is_open()) cov_csv_.close();

  if (!enabled_ || log_dir_.empty()) return;

  std::string latest_link = config_dir_ + "/../logs/latest";
  unlink(latest_link.c_str());  // ignore error if not exist
  if (symlink(log_dir_.c_str(), latest_link.c_str()) == 0) {
    std::cout << "[Logger] logs/latest → "
              << fs::path(log_dir_).filename().string() << "\n";
  }
  enabled_ = false;
}

// ===========================================================================
// Helpers
// ===========================================================================
void Logger::EnsureOpen() {
  if (!opt_csv_.is_open()) opt_csv_.open(log_dir_ + "/optimization.csv");
  if (!state_csv_.is_open()) state_csv_.open(log_dir_ + "/state_trace.csv");
  if (!resid_csv_.is_open()) resid_csv_.open(log_dir_ + "/residuals.csv");
  if (!cov_csv_.is_open()) cov_csv_.open(log_dir_ + "/covariance_diag.csv");
}

}  // namespace uifgo
