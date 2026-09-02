#include "uwb_imu_pl/config/integrity_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

void requireMap(const YAML::Node& node, const std::string& path) {
  if (!node || !node.IsMap()) throw std::runtime_error(path + " must be a map");
}

void rejectUnknown(const YAML::Node& node, const std::string& path,
                   const std::set<std::string>& allowed) {
  requireMap(node, path);
  for (const auto& item : node) {
    const std::string key = item.first.as<std::string>();
    if (allowed.count(key) == 0) {
      throw std::runtime_error("unknown key " + path + "." + key);
    }
  }
}

template <typename T>
T required(const YAML::Node& node, const char* key, const std::string& path) {
  if (!node[key]) throw std::runtime_error("missing required key " + path + "." + key);
  return node[key].as<T>();
}

void positive(double value, const std::string& path) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(path + " must be finite and > 0");
  }
}

void probability(double value, const std::string& path) {
  if (!std::isfinite(value) || value <= 0.0 || value >= 1.0) {
    throw std::runtime_error(path + " must be in (0, 1)");
  }
}

std::string readAll(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open config: " + path);
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

std::string fnv1a64(const std::string& input) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (unsigned char c : input) {
    hash ^= c;
    hash *= 1099511628211ULL;
  }
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << hash;
  return out.str();
}

Eigen::Vector3d vector3(const YAML::Node& node, const std::string& path) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error(path + " must contain exactly three numbers");
  }
  Eigen::Vector3d value(node[0].as<double>(), node[1].as<double>(),
                        node[2].as<double>());
  if (!value.allFinite()) throw std::runtime_error(path + " must be finite");
  return value;
}

}  // namespace

IntegrityConfig IntegrityConfigLoader::load(const std::string& yaml_path) {
  IntegrityConfig cfg;
  cfg.source_path = yaml_path;
  cfg.resolved_yaml = readAll(yaml_path);
  cfg.config_hash = fnv1a64(cfg.resolved_yaml);
  const YAML::Node root = YAML::Load(cfg.resolved_yaml);
  rejectUnknown(root, "root", {"seed", "snapshot", "incremental", "imu", "risk", "output", "realtime", "anchors"});
  cfg.seed = required<std::uint64_t>(root, "seed", "root");

  const auto snapshot = root["snapshot"];
  rejectUnknown(snapshot, "snapshot", {"dimensions", "max_iterations", "step_tolerance_m", "rank_tolerance", "max_condition_number"});
  cfg.snapshot.dimensions = required<int>(snapshot, "dimensions", "snapshot");
  cfg.snapshot.max_iterations = required<int>(snapshot, "max_iterations", "snapshot");
  cfg.snapshot.step_tolerance_m = required<double>(snapshot, "step_tolerance_m", "snapshot");
  cfg.snapshot.rank_tolerance = required<double>(snapshot, "rank_tolerance", "snapshot");
  cfg.snapshot.max_condition_number = required<double>(snapshot, "max_condition_number", "snapshot");
  if (cfg.snapshot.dimensions != 3) throw std::runtime_error("snapshot.dimensions must be 3 in this release");
  if (cfg.snapshot.max_iterations <= 0) throw std::runtime_error("snapshot.max_iterations must be > 0");
  positive(cfg.snapshot.step_tolerance_m, "snapshot.step_tolerance_m");
  positive(cfg.snapshot.rank_tolerance, "snapshot.rank_tolerance");
  positive(cfg.snapshot.max_condition_number, "snapshot.max_condition_number");

  const auto incremental = root["incremental"];
  rejectUnknown(incremental, "incremental", {"relinearize_threshold", "relinearize_skip", "smoothness_sigma_m", "epoch_bin_s", "max_time_skew_s", "enable_method_b", "method_b_max_condition", "fixed_lag_epochs"});
  cfg.incremental.relinearize_threshold = required<double>(incremental, "relinearize_threshold", "incremental");
  cfg.incremental.relinearize_skip = required<int>(incremental, "relinearize_skip", "incremental");
  cfg.incremental.smoothness_sigma_m = required<double>(incremental, "smoothness_sigma_m", "incremental");
  cfg.incremental.epoch_bin_s = required<double>(incremental, "epoch_bin_s", "incremental");
  cfg.incremental.max_time_skew_s = required<double>(incremental, "max_time_skew_s", "incremental");
  cfg.incremental.enable_method_b = required<bool>(incremental, "enable_method_b", "incremental");
  cfg.incremental.method_b_max_condition = required<double>(incremental, "method_b_max_condition", "incremental");
  const std::uint64_t fixed_lag_epochs = required<std::uint64_t>(
      incremental, "fixed_lag_epochs", "incremental");
  if (fixed_lag_epochs > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error(
        "incremental.fixed_lag_epochs must fit in uint32");
  }
  cfg.incremental.fixed_lag_epochs =
      static_cast<std::uint32_t>(fixed_lag_epochs);
  positive(cfg.incremental.relinearize_threshold, "incremental.relinearize_threshold");
  if (cfg.incremental.relinearize_skip <= 0) throw std::runtime_error("incremental.relinearize_skip must be > 0");
  positive(cfg.incremental.smoothness_sigma_m, "incremental.smoothness_sigma_m");
  positive(cfg.incremental.epoch_bin_s, "incremental.epoch_bin_s");
  if (cfg.incremental.max_time_skew_s < 0.0 || cfg.incremental.max_time_skew_s > cfg.incremental.epoch_bin_s) {
    throw std::runtime_error("incremental.max_time_skew_s must be in [0, epoch_bin_s]");
  }
  positive(cfg.incremental.method_b_max_condition, "incremental.method_b_max_condition");
  if (cfg.incremental.enable_method_b) {
    throw std::runtime_error(
        "incremental.enable_method_b=true is unsupported for online operation");
  }
  if (cfg.incremental.fixed_lag_epochs == 1) {
    throw std::runtime_error(
        "incremental.fixed_lag_epochs must be 0 or at least 2");
  }

  const auto imu = root["imu"];
  rejectUnknown(imu, "imu", {"accelerometer_sigma", "gyroscope_sigma", "accelerometer_bias_rw_sigma", "gyroscope_bias_rw_sigma", "gravity_mps2", "max_gap_s"});
  cfg.imu.accelerometer_sigma = required<double>(imu, "accelerometer_sigma", "imu");
  cfg.imu.gyroscope_sigma = required<double>(imu, "gyroscope_sigma", "imu");
  cfg.imu.accelerometer_bias_rw_sigma = required<double>(imu, "accelerometer_bias_rw_sigma", "imu");
  cfg.imu.gyroscope_bias_rw_sigma = required<double>(imu, "gyroscope_bias_rw_sigma", "imu");
  cfg.imu.gravity_mps2 = required<double>(imu, "gravity_mps2", "imu");
  cfg.imu.max_gap_s = required<double>(imu, "max_gap_s", "imu");
  positive(cfg.imu.accelerometer_sigma, "imu.accelerometer_sigma");
  positive(cfg.imu.gyroscope_sigma, "imu.gyroscope_sigma");
  positive(cfg.imu.accelerometer_bias_rw_sigma, "imu.accelerometer_bias_rw_sigma");
  positive(cfg.imu.gyroscope_bias_rw_sigma, "imu.gyroscope_bias_rw_sigma");
  positive(cfg.imu.gravity_mps2, "imu.gravity_mps2");
  positive(cfg.imu.max_gap_s, "imu.max_gap_s");

  const auto risk = root["risk"];
  rejectUnknown(risk, "risk", {"p_fa", "p_hmi_total", "nominal_axis_tail", "p_nm", "horizontal_alert_limit_m", "vertical_alert_limit_m", "single_anchor_p_md", "single_anchor_prior_bound"});
  cfg.risk.p_fa = required<double>(risk, "p_fa", "risk");
  cfg.risk.p_hmi_total = required<double>(risk, "p_hmi_total", "risk");
  cfg.risk.nominal_axis_tail = required<double>(risk, "nominal_axis_tail", "risk");
  cfg.risk.p_nm = required<double>(risk, "p_nm", "risk");
  cfg.risk.horizontal_alert_limit_m = required<double>(risk, "horizontal_alert_limit_m", "risk");
  cfg.risk.vertical_alert_limit_m = required<double>(risk, "vertical_alert_limit_m", "risk");
  probability(cfg.risk.p_fa, "risk.p_fa");
  probability(cfg.risk.p_hmi_total, "risk.p_hmi_total");
  probability(cfg.risk.nominal_axis_tail, "risk.nominal_axis_tail");
  probability(cfg.risk.p_nm, "risk.p_nm");
  positive(cfg.risk.horizontal_alert_limit_m, "risk.horizontal_alert_limit_m");
  positive(cfg.risk.vertical_alert_limit_m, "risk.vertical_alert_limit_m");
  const double p_md = required<double>(risk, "single_anchor_p_md", "risk");
  const double prior = required<double>(risk, "single_anchor_prior_bound", "risk");
  probability(p_md, "risk.single_anchor_p_md");
  probability(prior, "risk.single_anchor_prior_bound");

  const auto output = root["output"];
  rejectUnknown(output, "output", {"root", "write_residuals", "write_timing", "write_global_diagnostics"});
  cfg.output.root = required<std::string>(output, "root", "output");
  cfg.output.write_residuals = required<bool>(output, "write_residuals", "output");
  cfg.output.write_timing = required<bool>(output, "write_timing", "output");
  cfg.output.write_global_diagnostics =
      required<bool>(output, "write_global_diagnostics", "output");
  if (cfg.output.root.empty()) throw std::runtime_error("output.root must not be empty");

  const auto realtime = root["realtime"];
  rejectUnknown(realtime, "realtime", {"world_frame", "body_frame", "imu_topic", "uwb_topic", "odometry_topic", "integrity_topic", "diagnostics_topic", "initial_position_m", "initial_velocity_mps", "lever_arm_body_m", "range_sigma_m", "prior_sigmas"});
  cfg.realtime.world_frame = required<std::string>(realtime, "world_frame", "realtime");
  cfg.realtime.body_frame = required<std::string>(realtime, "body_frame", "realtime");
  cfg.realtime.imu_topic = required<std::string>(realtime, "imu_topic", "realtime");
  cfg.realtime.uwb_topic = required<std::string>(realtime, "uwb_topic", "realtime");
  cfg.realtime.odometry_topic = required<std::string>(realtime, "odometry_topic", "realtime");
  cfg.realtime.integrity_topic = required<std::string>(realtime, "integrity_topic", "realtime");
  cfg.realtime.diagnostics_topic = required<std::string>(realtime, "diagnostics_topic", "realtime");
  cfg.realtime.initial_position_m = vector3(realtime["initial_position_m"], "realtime.initial_position_m");
  cfg.realtime.initial_velocity_mps = vector3(realtime["initial_velocity_mps"], "realtime.initial_velocity_mps");
  cfg.realtime.lever_arm_body_m = vector3(realtime["lever_arm_body_m"], "realtime.lever_arm_body_m");
  cfg.realtime.range_sigma_m = required<double>(realtime, "range_sigma_m", "realtime");
  positive(cfg.realtime.range_sigma_m, "realtime.range_sigma_m");
  const auto prior_sigmas = realtime["prior_sigmas"];
  if (!prior_sigmas || !prior_sigmas.IsSequence() || prior_sigmas.size() != 15) {
    throw std::runtime_error("realtime.prior_sigmas must contain 15 values in Pose3,v,bias order");
  }
  for (int i = 0; i < 15; ++i) {
    cfg.realtime.prior_sigmas(i) = prior_sigmas[static_cast<std::size_t>(i)].as<double>();
    positive(cfg.realtime.prior_sigmas(i), "realtime.prior_sigmas");
  }

  const auto anchors = root["anchors"];
  if (!anchors || !anchors.IsSequence() || anchors.size() < 4) {
    throw std::runtime_error("anchors must contain at least four records");
  }
  std::set<std::uint64_t> anchor_ids;
  for (std::size_t i = 0; i < anchors.size(); ++i) {
    const auto anchor = anchors[i];
    rejectUnknown(anchor, "anchors[]", {"id", "position_m", "sigma_m", "frame", "map_version"});
    AnchorRecord record;
    record.id = AnchorId(required<std::uint64_t>(anchor, "id", "anchors[]"));
    if (!anchor_ids.insert(record.id.value()).second) {
      throw std::runtime_error("duplicate physical anchor id");
    }
    record.position_world_m = vector3(anchor["position_m"], "anchors[].position_m");
    const double sigma = required<double>(anchor, "sigma_m", "anchors[]");
    positive(sigma, "anchors[].sigma_m");
    record.covariance_m2 = Eigen::Matrix3d::Identity() * sigma * sigma;
    record.frame = required<std::string>(anchor, "frame", "anchors[]");
    record.map_version = required<std::string>(anchor, "map_version", "anchors[]");
    if (record.frame != cfg.realtime.world_frame) {
      throw std::runtime_error("anchor frame must equal realtime.world_frame in this release");
    }
    cfg.anchors.push_back(std::move(record));
  }
  for (const auto& anchor : cfg.anchors) {
    FaultHypothesis hypothesis;
    hypothesis.id = HypothesisId(anchor.id.value());
    hypothesis.anchor_id = anchor.id;
    hypothesis.prior_probability_bound = prior;
    hypothesis.missed_detection_allocation = p_md;
    cfg.risk.hypotheses.push_back(std::move(hypothesis));
  }
  const double allocated_hmi = 3.0 * cfg.risk.nominal_axis_tail + cfg.risk.p_nm +
      static_cast<double>(cfg.risk.hypotheses.size()) * prior * p_md;
  if (!std::isfinite(allocated_hmi) || allocated_hmi > cfg.risk.p_hmi_total) {
    throw std::runtime_error(
        "risk.p_hmi_total is smaller than the complete anchor-map allocation");
  }
  return cfg;
}

}  // namespace uwb_imu_pl
