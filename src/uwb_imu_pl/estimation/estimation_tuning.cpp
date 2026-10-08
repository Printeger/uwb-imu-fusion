#include "uwb_imu_pl/estimation/estimation_tuning.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <set>
#include <stdexcept>

namespace uwb_imu_pl {
EstimationTuningV1 readEstimationTuningV1(const IntegrityConfig& config) {
  EstimationTuningV1 out;
  if (config.resolved_yaml.empty()) return out;
  const auto root = YAML::Load(config.resolved_yaml);
  const auto node = root["estimation_tuning"];
  if (!node) return out;
  if (!node.IsMap() || !node["version"] || node["version"].as<int>() != 1)
    throw std::runtime_error("estimation_tuning requires version: 1");
  const std::set<std::string> allowed = {
      "version", "bias_integration_sigmas", "nominal_initial_guess",
      "nominal_lm_iterations", "nominal_lag_s", "nominal_robust_experimental",
      "causal_bootstrap", "bootstrap_prefer_below_anchors",
      "bootstrap_exact_uwb_times", "bootstrap_uwb_motion_check", "bootstrap_seed_only", "bootstrap_enforce_below_anchors"};
  for (const auto& item : node)
    if (!allowed.count(item.first.as<std::string>()))
      throw std::runtime_error("unknown key estimation_tuning." + item.first.as<std::string>());
  if (node["bias_integration_sigmas"]) {
    const auto values = node["bias_integration_sigmas"];
    if (!values.IsSequence() || values.size() != 6)
      throw std::runtime_error("bias_integration_sigmas requires six accel/gyro values");
    Eigen::Matrix<double, 6, 1> sigmas;
    for (int i = 0; i < 6; ++i) {
      sigmas(i) = values[i].as<double>();
      if (!std::isfinite(sigmas(i)) || sigmas(i) <= 0.0)
        throw std::runtime_error("bias_integration_sigmas must be finite and positive");
    }
    out.bias_integration_sigmas = sigmas;
  }
  if (node["nominal_initial_guess"])
    out.nominal_initial_guess = node["nominal_initial_guess"].as<std::string>();
  if (node["nominal_lm_iterations"])
    out.nominal_lm_iterations = node["nominal_lm_iterations"].as<int>();
  if (node["nominal_lag_s"]) out.nominal_lag_s = node["nominal_lag_s"].as<double>();
  if (node["nominal_robust_experimental"])
    out.nominal_robust_experimental = node["nominal_robust_experimental"].as<bool>();
  if (node["causal_bootstrap"]) out.causal_bootstrap = node["causal_bootstrap"].as<bool>();
  if (node["bootstrap_prefer_below_anchors"])
    out.bootstrap_prefer_below_anchors = node["bootstrap_prefer_below_anchors"].as<bool>();
  if (node["bootstrap_exact_uwb_times"])
    out.bootstrap_exact_uwb_times = node["bootstrap_exact_uwb_times"].as<bool>();
  if (node["bootstrap_uwb_motion_check"])
    out.bootstrap_uwb_motion_check = node["bootstrap_uwb_motion_check"].as<bool>();
  if (node["bootstrap_seed_only"])
    out.bootstrap_seed_only = node["bootstrap_seed_only"].as<bool>();
  if (node["bootstrap_enforce_below_anchors"])
    out.bootstrap_enforce_below_anchors = node["bootstrap_enforce_below_anchors"].as<bool>();
  if (out.nominal_initial_guess != "cv" && out.nominal_initial_guess != "imu")
    throw std::runtime_error("nominal_initial_guess must be cv or imu");
  if (out.nominal_lm_iterations < 0 || out.nominal_lm_iterations > 5 ||
      !std::isfinite(out.nominal_lag_s) || out.nominal_lag_s < 0.0)
    throw std::runtime_error("nominal LM iterations must be 0..5 and lag finite/nonnegative");
  if (out.nominal_lm_iterations && out.nominal_initial_guess != "imu")
    throw std::runtime_error("nominal LM warm start requires imu initial guess");
  if (config.fde.profile != FdeProfile::Off &&
      (out.nominal_initial_guess != "cv" || out.nominal_lm_iterations ||
       out.nominal_lag_s > 0.0 || out.nominal_robust_experimental || out.causal_bootstrap ||
       out.bootstrap_exact_uwb_times || out.bootstrap_uwb_motion_check || out.bootstrap_seed_only || out.bootstrap_enforce_below_anchors))
    throw std::runtime_error("nominal-only estimation_tuning requires fde.profile=off");
  return out;
}
}  // namespace uwb_imu_pl
