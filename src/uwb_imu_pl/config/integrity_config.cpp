#include "uwb_imu_pl/config/integrity_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace uwb_imu_pl {

ImuNoiseQualificationV1 assessImuNoiseQualificationV1(
    const ImuNoiseConfig& config) {
  (void)config;
  ImuNoiseQualificationV1 out;
  // V1 is intentionally fail-closed.  A free-form identifier or declaration
  // inside an ordinary run profile is not a qualification artifact.  Opening
  // this gate requires a separately versioned verifier that authenticates a
  // strict artifact schema, its content digest/allowlist, and its binding to
  // the complete model/config digest.  No such hardware artifact is shipped.
  // Keeping this state out of ImuNoiseConfig also preserves the frozen public
  // aggregate ABI for golden-p0-06 consumers.
  out.reason =
      "IMU_MODEL_UNQUALIFIED: no authenticated V1 calibration artifact "
      "is bound to the complete model/config digest; profile declarations "
      "and calibration identifiers are not qualification evidence;";
  return out;
}

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
  if (!node[key])
    throw std::runtime_error("missing required key " + path + "." + key);
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

void nonnegativeProbability(double value, const std::string& path) {
  if (!std::isfinite(value) || value < 0.0 || value >= 1.0) {
    throw std::runtime_error(path + " must be in [0, 1)");
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

std::uint32_t parseFixedLagEpochs(const std::string& text,
                                  const std::string& path) {
  if (text.empty() || !std::all_of(text.begin(), text.end(), [](char value) {
        return std::isdigit(static_cast<unsigned char>(value)) != 0;
      })) {
    throw std::runtime_error(path + " must be an unsigned integer");
  }
  std::uint64_t value = 0;
  try {
    value = std::stoull(text);
  } catch (const std::exception&) {
    throw std::runtime_error(path + " must be an unsigned integer");
  }
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error(path + " must fit in uint32");
  }
  if (value == 1) {
    throw std::runtime_error(path + " must be 0 or at least 2");
  }
  return static_cast<std::uint32_t>(value);
}

std::string emitResolvedYaml(const YAML::Node& root) {
  YAML::Emitter emitter;
  emitter << root;
  if (!emitter.good()) {
    throw std::runtime_error("cannot serialize resolved configuration: " +
                             emitter.GetLastError());
  }
  return std::string(emitter.c_str()) + "\n";
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

std::string resolveRelativePath(const std::string& base_file,
                                const std::string& path) {
  if (path.empty() || path.front() == '/') return path;
  const auto slash = base_file.find_last_of('/');
  if (slash == std::string::npos) return path;
  return base_file.substr(0, slash + 1) + path;
}

bool manifestLists(const std::vector<std::string>& values,
                   const std::string& value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

const FaultManifestEvent* manifestEvent(const FaultManifest& manifest,
                                        const std::string& event_id) {
  for (const auto& event : manifest.events) {
    if (event.event_id == event_id) return &event;
  }
  return nullptr;
}

// Cross-checks the manifest against the enabled configuration switches.  The
// checks fail closed: an enabled model whose event is missing from the
// manifest, declared NOT_IMPLEMENTED, or outside an enabled family rejects the
// configuration instead of being demoted silently.
void validateFaultManifestAgainstConfig(const IntegrityConfig& cfg,
                                        const FaultManifest& manifest) {
  const int declared_order = cfg.fault_models.double_faults_enabled ? 2 : 1;
  if (manifest.max_fault_order < declared_order) {
    throw std::runtime_error("fault manifest supports max_fault_order=" +
                             std::to_string(manifest.max_fault_order) +
                             " but the configuration declares order=" +
                             std::to_string(declared_order));
  }
  const auto require_event = [&](const char* event_id, bool enabled,
                                 const char* family_id) {
    const FaultManifestEvent* event = manifestEvent(manifest, event_id);
    if (!event) {
      throw std::runtime_error(std::string("fault manifest is missing event ") +
                               event_id);
    }
    if (!enabled) return;
    if (event->evidence_status == "NOT_IMPLEMENTED") {
      throw std::runtime_error(std::string("event ") + event_id +
                               " is enabled but declared NOT_IMPLEMENTED");
    }
    if (!manifestLists(manifest.enabled_single_families, family_id)) {
      throw std::runtime_error(std::string("event ") + event_id +
                               " is enabled but family " + family_id +
                               " is not enabled");
    }
    if (!FaultManifestLoader::modelTypeImplemented(event->model_type)) {
      throw std::runtime_error(std::string("event ") + event_id +
                               " uses unsupported model_type " +
                               event->model_type);
    }
  };
  require_event("uwb_anchor_epoch_independent_bias",
                cfg.fault_models.uwb.epoch_single_anchor_bias,
                "uwb_single_anchor");
  require_event("uwb_anchor_persistent_bias",
                cfg.fault_models.uwb.persistent_anchor_bias,
                "uwb_single_anchor");
  require_event("uwb_anchor_affine_ramp", cfg.fault_models.uwb.ramp_bias,
                "uwb_single_anchor");
  require_event("imu_accel_axis_interval_bias",
                cfg.fault_models.imu.accel_axis_interval_bias,
                "imu_single_axis");
  require_event("imu_gyro_axis_interval_bias",
                cfg.fault_models.imu.gyro_axis_interval_bias,
                "imu_single_axis");
  const bool pair_enabled = cfg.fault_models.double_faults_enabled &&
                            (cfg.fault_models.combinations.uwb_plus_accel ||
                             cfg.fault_models.combinations.uwb_plus_gyro);
  if (pair_enabled &&
      !manifestLists(manifest.enabled_pair_families, "uwb_imu")) {
    throw std::runtime_error(
        "configured UWB x IMU combinations require the declared pair family "
        "uwb_imu in the fault manifest");
  }
  if (manifestLists(manifest.unsupported_families, "two_uwb") &&
      cfg.fault_models.combinations.two_uwb) {
    throw std::runtime_error(
        "two_uwb is declared unsupported by the generator but enabled");
  }
}

}  // namespace

std::uint32_t enabledFaultHypothesisCardinality(
    const FaultModelsConfig& config) {
  return config.double_faults_enabled ? 2u : 1u;
}

std::uint64_t faultModelPolicyFingerprint(const FaultModelsConfig& config) {
  std::uint64_t hash = 1469598103934665603ULL;
  auto append = [&](std::uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) {
      hash ^= static_cast<unsigned char>((value >> (byte * 8)) & 0xffU);
      hash *= 1099511628211ULL;
    }
  };
  append(config.single_faults_enabled);
  append(config.double_faults_enabled);
  append(config.max_cardinality);
  append(config.uwb.enabled);
  append(config.uwb.epoch_single_anchor_bias);
  append(config.uwb.persistent_anchor_bias);
  append(config.uwb.ramp_bias);
  append(config.imu.accel_axis_interval_bias);
  append(config.imu.gyro_axis_interval_bias);
  append(config.combinations.uwb_plus_accel);
  append(config.combinations.uwb_plus_gyro);
  append(config.combinations.two_uwb);
  return hash;
}

std::uint64_t faultScopePolicyFingerprint(
    const FaultModelsConfig& config, const ResolvedFaultScope& scope) {
  std::uint64_t hash = faultModelPolicyFingerprint(config);
  auto append_bytes = [&](const void* data, std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
      hash ^= bytes[i];
      hash *= 1099511628211ULL;
    }
  };
  const auto profile = static_cast<unsigned>(scope.profile);
  append_bytes(&profile, sizeof(profile));
  append_bytes(&scope.max_fault_order, sizeof(scope.max_fault_order));
  append_bytes(&scope.allow_uwb_actions, sizeof(scope.allow_uwb_actions));
  append_bytes(&scope.allow_imu_actions, sizeof(scope.allow_imu_actions));
  append_bytes(scope.scope_digest.data(), scope.scope_digest.size());
  return hash;
}

IntegrityConfig IntegrityConfigLoader::load(
    const std::string& yaml_path,
    const std::optional<std::string>& fixed_lag_epochs_override) {
  IntegrityConfigOverrides overrides;
  if (fixed_lag_epochs_override && !fixed_lag_epochs_override->empty()) {
    overrides.fixed_lag_epochs = parseFixedLagEpochs(
        *fixed_lag_epochs_override, "incremental.fixed_lag_epochs override");
  }
  return load(yaml_path, overrides);
}

IntegrityConfig IntegrityConfigLoader::load(
    const std::string& yaml_path, const IntegrityConfigOverrides& overrides) {
  IntegrityConfig cfg;
  cfg.source_path = yaml_path;
  YAML::Node root = YAML::Load(readAll(yaml_path));
  requireMap(root, "root");
  const std::string source_schema =
      required<std::string>(root, "schema_version", "root");
  const bool is_v6 = source_schema == "uwb-imu-pl/v6";
  if (source_schema != "uwb-imu-pl/v4" &&
      source_schema != "uwb-imu-pl/v5" && !is_v6) {
    throw std::runtime_error(
        "schema_version must be uwb-imu-pl/v4, uwb-imu-pl/v5 or "
        "uwb-imu-pl/v6");
  }
  requireMap(root["fault_models"], "fault_models");
  requireMap(root["fde"], "fde");
  // A3 migration: remember which fault-order keys the user actually wrote so
  // that order/flags conflicts are rejected instead of silently overwritten.
  const bool has_single_flag =
      static_cast<bool>(root["fault_models"]["single_faults_enabled"]);
  const bool has_double_flag =
      static_cast<bool>(root["fault_models"]["double_faults_enabled"]);
  const bool has_max_fault_order =
      static_cast<bool>(root["fault_models"]["max_fault_order"]);
  if (is_v6 && (has_single_flag || has_double_flag || has_max_fault_order)) {
    throw std::runtime_error(
        "v6 fde.profile conflicts with legacy fault order/enable fields");
  }
  if (is_v6 && !root["fde"]["profile"]) {
    throw std::runtime_error("missing required key fde.profile");
  }
  if (!is_v6 && root["fde"]["profile"]) {
    throw std::runtime_error(
        "v5 legacy fault order fields and fde.profile cannot be combined");
  }
  if (is_v6) {
    const FdeProfile profile =
        parseFdeProfile(root["fde"]["profile"].as<std::string>());
    root["fault_models"]["single_faults_enabled"] =
        profile != FdeProfile::Off;
    root["fault_models"]["double_faults_enabled"] =
        profile == FdeProfile::JointOrder2;
    root["fault_models"]["max_fault_order"] =
        profile == FdeProfile::JointOrder2 ? 2 : 1;
  } else if (!root["fault_models"]["single_faults_enabled"]) {
    root["fault_models"]["single_faults_enabled"] = true;
  }
  if (!is_v6 && !root["fault_models"]["double_faults_enabled"]) {
    root["fault_models"]["double_faults_enabled"] = false;
  }
  if (!root["fde"]["max_exclusion_cardinality"]) {
    root["fde"]["max_exclusion_cardinality"] = 2;
  }
  if (overrides.seed) root["seed"] = *overrides.seed;
  if (overrides.fixed_lag_epochs) {
    requireMap(root["incremental"], "incremental");
    if (*overrides.fixed_lag_epochs == 1) {
      throw std::runtime_error(
          "incremental.fixed_lag_epochs override must be 0 or at least 2");
    }
    root["incremental"]["fixed_lag_epochs"] = *overrides.fixed_lag_epochs;
  }
  const bool has_output_override =
      overrides.write_global_diagnostics || overrides.write_residuals ||
      overrides.write_timing || overrides.output_root;
  if (has_output_override) requireMap(root["output"], "output");
  if (overrides.write_global_diagnostics) {
    root["output"]["write_global_diagnostics"] =
        *overrides.write_global_diagnostics;
  }
  if (overrides.write_residuals) {
    root["output"]["write_residuals"] = *overrides.write_residuals;
  }
  if (overrides.write_timing) {
    root["output"]["write_timing"] = *overrides.write_timing;
  }
  if (overrides.output_root) root["output"]["root"] = *overrides.output_root;
  if (overrides.fde_profile) {
    if (!is_v6) {
      throw std::runtime_error(
          "fde profile override requires a v6 base configuration");
    }
    root["fde"]["profile"] = toString(*overrides.fde_profile);
    root["fault_models"]["single_faults_enabled"] =
        *overrides.fde_profile != FdeProfile::Off;
    root["fault_models"]["double_faults_enabled"] =
        *overrides.fde_profile == FdeProfile::JointOrder2;
    root["fault_models"]["max_fault_order"] =
        *overrides.fde_profile == FdeProfile::JointOrder2 ? 2 : 1;
  }
  cfg.resolved_yaml = emitResolvedYaml(root);
  cfg.config_hash = fnv1a64(cfg.resolved_yaml);
  rejectUnknown(root, "root",
                {"schema_version", "seed", "snapshot", "incremental", "imu",
                 "integrity_window", "detector", "fault_models", "fde",
                 "bridge", "health", "risk", "robust_shadow", "output",
                 "publication", "realtime", "anchors", "history"});
  cfg.schema_version = required<std::string>(root, "schema_version", "root");
  cfg.seed = required<std::uint64_t>(root, "seed", "root");

  const auto snapshot = root["snapshot"];
  rejectUnknown(snapshot, "snapshot",
                {"dimensions", "max_iterations", "step_tolerance_m",
                 "rank_tolerance", "max_condition_number"});
  cfg.snapshot.dimensions = required<int>(snapshot, "dimensions", "snapshot");
  cfg.snapshot.max_iterations =
      required<int>(snapshot, "max_iterations", "snapshot");
  cfg.snapshot.step_tolerance_m =
      required<double>(snapshot, "step_tolerance_m", "snapshot");
  cfg.snapshot.rank_tolerance =
      required<double>(snapshot, "rank_tolerance", "snapshot");
  cfg.snapshot.max_condition_number =
      required<double>(snapshot, "max_condition_number", "snapshot");
  if (cfg.snapshot.dimensions != 3)
    throw std::runtime_error("snapshot.dimensions must be 3 in this release");
  if (cfg.snapshot.max_iterations <= 0)
    throw std::runtime_error("snapshot.max_iterations must be > 0");
  positive(cfg.snapshot.step_tolerance_m, "snapshot.step_tolerance_m");
  positive(cfg.snapshot.rank_tolerance, "snapshot.rank_tolerance");
  positive(cfg.snapshot.max_condition_number, "snapshot.max_condition_number");

  const auto incremental = root["incremental"];
  rejectUnknown(incremental, "incremental",
                {"relinearize_threshold", "relinearize_skip",
                 "smoothness_sigma_m", "epoch_bin_s", "max_time_skew_s",
                 "enable_method_b", "method_b_max_condition",
                 "fixed_lag_epochs", "single_transaction_per_epoch"});
  cfg.incremental.relinearize_threshold =
      required<double>(incremental, "relinearize_threshold", "incremental");
  cfg.incremental.relinearize_skip =
      required<int>(incremental, "relinearize_skip", "incremental");
  cfg.incremental.smoothness_sigma_m =
      required<double>(incremental, "smoothness_sigma_m", "incremental");
  cfg.incremental.epoch_bin_s =
      required<double>(incremental, "epoch_bin_s", "incremental");
  cfg.incremental.max_time_skew_s =
      required<double>(incremental, "max_time_skew_s", "incremental");
  cfg.incremental.enable_method_b =
      required<bool>(incremental, "enable_method_b", "incremental");
  cfg.incremental.method_b_max_condition =
      required<double>(incremental, "method_b_max_condition", "incremental");
  if (!incremental["fixed_lag_epochs"] ||
      !incremental["fixed_lag_epochs"].IsScalar()) {
    throw std::runtime_error(
        "missing required key incremental.fixed_lag_epochs");
  }
  cfg.incremental.fixed_lag_epochs = parseFixedLagEpochs(
      incremental["fixed_lag_epochs"].Scalar(), "incremental.fixed_lag_epochs");
  cfg.incremental.single_transaction_per_epoch = required<bool>(
      incremental, "single_transaction_per_epoch", "incremental");
  positive(cfg.incremental.relinearize_threshold,
           "incremental.relinearize_threshold");
  if (cfg.incremental.relinearize_skip <= 0)
    throw std::runtime_error("incremental.relinearize_skip must be > 0");
  positive(cfg.incremental.smoothness_sigma_m,
           "incremental.smoothness_sigma_m");
  positive(cfg.incremental.epoch_bin_s, "incremental.epoch_bin_s");
  if (cfg.incremental.max_time_skew_s < 0.0 ||
      cfg.incremental.max_time_skew_s > cfg.incremental.epoch_bin_s) {
    throw std::runtime_error(
        "incremental.max_time_skew_s must be in [0, epoch_bin_s]");
  }
  positive(cfg.incremental.method_b_max_condition,
           "incremental.method_b_max_condition");
  if (cfg.incremental.enable_method_b) {
    throw std::runtime_error(
        "incremental.enable_method_b=true is unsupported for online operation");
  }
  if (!cfg.incremental.single_transaction_per_epoch) {
    throw std::runtime_error(
        "incremental.single_transaction_per_epoch must be true in v4");
  }
  const auto imu = root["imu"];
  rejectUnknown(
      imu, "imu",
      {"accelerometer_sigma", "gyroscope_sigma", "accelerometer_bias_rw_sigma",
       "gyroscope_bias_rw_sigma", "gravity_mps2", "max_gap_s",
       "sigma_semantics", "trapezoid_correlation_model",
       "noise_overbound_calibration_id"});
  cfg.imu.accelerometer_sigma =
      required<double>(imu, "accelerometer_sigma", "imu");
  cfg.imu.gyroscope_sigma = required<double>(imu, "gyroscope_sigma", "imu");
  cfg.imu.accelerometer_bias_rw_sigma =
      required<double>(imu, "accelerometer_bias_rw_sigma", "imu");
  cfg.imu.gyroscope_bias_rw_sigma =
      required<double>(imu, "gyroscope_bias_rw_sigma", "imu");
  cfg.imu.gravity_mps2 = required<double>(imu, "gravity_mps2", "imu");
  cfg.imu.max_gap_s = required<double>(imu, "max_gap_s", "imu");
  // These optional declarations document research profiles only.  They are
  // deliberately not stored in the public aggregate and can never open the
  // qualification gate.  Old v5/v6 profiles without them remain loadable.
  const std::string sigma_semantics = imu["sigma_semantics"]
      ? imu["sigma_semantics"].as<std::string>()
      : "UNQUALIFIED_CONFIGURED_PARAMETER";
  const std::string trapezoid_correlation_model =
      imu["trapezoid_correlation_model"]
      ? imu["trapezoid_correlation_model"].as<std::string>()
      : "SHARED_SAMPLE_CORRELATION_UNQUALIFIED";
  cfg.imu.noise_overbound_calibration_id =
      required<std::string>(imu, "noise_overbound_calibration_id", "imu");
  positive(cfg.imu.accelerometer_sigma, "imu.accelerometer_sigma");
  positive(cfg.imu.gyroscope_sigma, "imu.gyroscope_sigma");
  positive(cfg.imu.accelerometer_bias_rw_sigma,
           "imu.accelerometer_bias_rw_sigma");
  positive(cfg.imu.gyroscope_bias_rw_sigma, "imu.gyroscope_bias_rw_sigma");
  positive(cfg.imu.gravity_mps2, "imu.gravity_mps2");
  positive(cfg.imu.max_gap_s, "imu.max_gap_s");
  if (sigma_semantics != "UNQUALIFIED_CONFIGURED_PARAMETER" &&
      sigma_semantics != "CALIBRATED_EFFECTIVE_INTERVAL_OVERBOUND") {
    throw std::runtime_error(
        "imu.sigma_semantics has an unsupported qualification value");
  }
  if (trapezoid_correlation_model !=
          "SHARED_SAMPLE_CORRELATION_UNQUALIFIED" &&
      trapezoid_correlation_model !=
          "CALIBRATED_CONSERVATIVE_INDEPENDENT_OVERBOUND") {
    throw std::runtime_error(
        "imu.trapezoid_correlation_model has an unsupported qualification value");
  }

  const auto window = root["integrity_window"];
  rejectUnknown(window, "integrity_window",
                {"epochs", "intervals", "recovery_margin_epochs",
                 "boundary_method", "rank_tolerance", "max_condition_number",
                 "max_linearization_step_norm", "require_history_provenance"});
  // A3 migration: `intervals` is the canonical name; `epochs` is a deprecated
  // alias for the same interval count.  Supplying both with different values
  // is rejected instead of picking one silently.
  {
    const auto epochs_node = window["epochs"];
    const auto intervals_node = window["intervals"];
    if (!epochs_node && !intervals_node) {
      throw std::runtime_error("integrity_window requires epochs or intervals");
    }
    if (epochs_node && intervals_node) {
      const auto epochs_value = epochs_node.as<std::uint32_t>();
      const auto intervals_value = intervals_node.as<std::uint32_t>();
      if (epochs_value != intervals_value) {
        throw std::runtime_error(
            "integrity_window.epochs conflicts with "
            "integrity_window.intervals");
      }
      cfg.migration_warnings.push_back(
          "integrity_window.epochs duplicated integrity_window.intervals; "
          "remove the legacy alias");
    } else if (epochs_node) {
      cfg.migration_warnings.push_back(
          "integrity_window.epochs is a deprecated alias of intervals "
          "(interval count, nodes = intervals + 1)");
    }
    cfg.integrity_window.epochs = epochs_node
                                      ? epochs_node.as<std::uint32_t>()
                                      : intervals_node.as<std::uint32_t>();
    root["integrity_window"]["intervals"] = cfg.integrity_window.epochs;
    root["integrity_window"]["epochs"] = cfg.integrity_window.epochs;
  }
  cfg.integrity_window.recovery_margin_epochs = required<std::uint32_t>(
      window, "recovery_margin_epochs", "integrity_window");
  cfg.integrity_window.boundary_method =
      required<std::string>(window, "boundary_method", "integrity_window");
  cfg.integrity_window.rank_tolerance =
      required<double>(window, "rank_tolerance", "integrity_window");
  cfg.integrity_window.max_condition_number =
      required<double>(window, "max_condition_number", "integrity_window");
  cfg.integrity_window.max_linearization_step_norm = required<double>(
      window, "max_linearization_step_norm", "integrity_window");
  cfg.integrity_window.require_history_provenance =
      required<bool>(window, "require_history_provenance", "integrity_window");
  if (cfg.integrity_window.epochs == 0)
    throw std::runtime_error("integrity_window.intervals must be > 0");
  cfg.migration_warnings.push_back(
      "integrity_window.max_linearization_step_norm retained unchanged: the "
      "0.25 mixed-unit Euclidean step gate remains a candidate admissibility "
      "check; physical-scale linearization diagnostics are deferred to B/C");
  if (cfg.integrity_window.boundary_method != "bayes_tree_schur")
    throw std::runtime_error(
        "integrity_window.boundary_method must be bayes_tree_schur");
  positive(cfg.integrity_window.rank_tolerance,
           "integrity_window.rank_tolerance");
  positive(cfg.integrity_window.max_condition_number,
           "integrity_window.max_condition_number");
  positive(cfg.integrity_window.max_linearization_step_norm,
           "integrity_window.max_linearization_step_norm");
  if (cfg.incremental.fixed_lag_epochs != 0 &&
      cfg.incremental.fixed_lag_epochs <=
          cfg.integrity_window.epochs +
              cfg.integrity_window.recovery_margin_epochs) {
    throw std::runtime_error(
        "incremental.fixed_lag_epochs must exceed integrity window plus "
        "recovery margin");
  }

  // C1-c/C2: history-summary capacity keys.  The section is optional (absent
  // means zero capacity: the summary path is not wired yet, so nothing is
  // produced); when present it is strict - unknown keys, a missing key or an
  // unknown capacity action are hard errors.
  {
    const auto history = root["history"];
    if (history) {
      rejectUnknown(history, "history",
                    {"max_summary_rows", "max_fault_columns", "max_perp_rows",
                     "capacity_action"});
      cfg.history.max_summary_rows =
          required<std::uint64_t>(history, "max_summary_rows", "history");
      cfg.history.max_fault_columns =
          required<std::uint64_t>(history, "max_fault_columns", "history");
      cfg.history.max_perp_rows =
          required<std::uint64_t>(history, "max_perp_rows", "history");
      cfg.history.capacity_action =
          required<std::string>(history, "capacity_action", "history");
      if (cfg.history.capacity_action != "REFUSE" &&
          cfg.history.capacity_action != "RESET" &&
          cfg.history.capacity_action != "STOP_PROTECTED") {
        throw std::runtime_error(
            "history.capacity_action must be REFUSE, RESET or STOP_PROTECTED");
      }
    }
    // Canonicalize the optional section so the resolved dump and its hash are
    // stable whether or not the source file declares it.
    root["history"]["max_summary_rows"] = cfg.history.max_summary_rows;
    root["history"]["max_fault_columns"] = cfg.history.max_fault_columns;
    root["history"]["max_perp_rows"] = cfg.history.max_perp_rows;
    root["history"]["capacity_action"] = cfg.history.capacity_action;
  }

  const auto detector = root["detector"];
  rejectUnknown(detector, "detector",
                {"type", "p_fa_per_test", "continuity_horizon_tests",
                 "continuity_accounting", "use_squared_norm_statistic"});
  cfg.detector.type = required<std::string>(detector, "type", "detector");
  cfg.detector.p_fa_per_test =
      required<double>(detector, "p_fa_per_test", "detector");
  cfg.detector.continuity_horizon_tests =
      required<std::uint64_t>(detector, "continuity_horizon_tests", "detector");
  cfg.detector.continuity_accounting =
      required<std::string>(detector, "continuity_accounting", "detector");
  cfg.detector.use_squared_norm_statistic =
      required<bool>(detector, "use_squared_norm_statistic", "detector");
  probability(cfg.detector.p_fa_per_test, "detector.p_fa_per_test");
  if (cfg.detector.type != "joint_window_residual_chi_square" ||
      cfg.detector.continuity_accounting != "union_bound" ||
      !cfg.detector.use_squared_norm_statistic ||
      cfg.detector.continuity_horizon_tests == 0) {
    throw std::runtime_error(
        "detector must use v4 squared joint-window union-bound semantics");
  }

  const auto fault_models = root["fault_models"];
  rejectUnknown(
      fault_models, "fault_models",
      {"single_faults_enabled", "double_faults_enabled", "max_cardinality",
       "max_fault_order", "manifest_path", "uwb", "imu", "combinations"});
  cfg.fault_models.single_faults_enabled =
      required<bool>(fault_models, "single_faults_enabled", "fault_models");
  cfg.fault_models.double_faults_enabled =
      required<bool>(fault_models, "double_faults_enabled", "fault_models");
  if (!cfg.fault_models.single_faults_enabled &&
      !cfg.fault_models.double_faults_enabled && !is_v6) {
    throw std::runtime_error(
        "fault_models must enable single_faults_enabled or "
        "double_faults_enabled");
  }
  // A3 migration: max_fault_order is the canonical declared order.  Legacy
  // flags remain readable; a conflict between the two spellings is rejected.
  if (is_v6) {
    cfg.fault_models.max_fault_order =
        cfg.fault_models.double_faults_enabled ? 2 : 1;
  } else if (has_max_fault_order) {
    const int order = fault_models["max_fault_order"].as<int>();
    if (order != 1 && order != 2) {
      throw std::runtime_error("fault_models.max_fault_order must be 1 or 2");
    }
    const int derived = cfg.fault_models.double_faults_enabled ? 2 : 1;
    if ((has_single_flag || has_double_flag) && order != derived) {
      throw std::runtime_error(
          "fault_models.max_fault_order conflicts with legacy "
          "single_faults_enabled/double_faults_enabled");
    }
    if (!has_single_flag && !has_double_flag) {
      cfg.fault_models.single_faults_enabled = true;
      cfg.fault_models.double_faults_enabled = (order == 2);
      root["fault_models"]["single_faults_enabled"] = true;
      root["fault_models"]["double_faults_enabled"] = (order == 2);
      cfg.migration_warnings.push_back(
          "fault_models.max_fault_order=" + std::to_string(order) +
          " derived legacy single/double flags (order-only declaration)");
    }
    cfg.fault_models.max_fault_order = order;
  } else {
    cfg.fault_models.max_fault_order =
        cfg.fault_models.double_faults_enabled ? 2 : 1;
    cfg.migration_warnings.push_back(
        "legacy fault_models.single_faults_enabled/double_faults_enabled "
        "mapped to max_fault_order=" +
        std::to_string(*cfg.fault_models.max_fault_order));
  }
  root["fault_models"]["max_fault_order"] = *cfg.fault_models.max_fault_order;
  if (fault_models["manifest_path"]) {
    cfg.fault_models.manifest_path =
        fault_models["manifest_path"].as<std::string>();
  }
  cfg.fault_models.max_cardinality =
      required<std::uint32_t>(fault_models, "max_cardinality", "fault_models");
  if (cfg.fault_models.max_cardinality != 2)
    throw std::runtime_error(
        "fault_models.max_cardinality must be 2 for the initial claim");
  const auto uwb_fault = fault_models["uwb"];
  rejectUnknown(
      uwb_fault, "fault_models.uwb",
      {"enabled", "epoch_single_anchor_bias", "persistent_anchor_bias",
       "ramp_bias", "prior_probability_bound", "p_md"});
  cfg.fault_models.uwb.enabled =
      required<bool>(uwb_fault, "enabled", "fault_models.uwb");
  cfg.fault_models.uwb.epoch_single_anchor_bias =
      required<bool>(uwb_fault, "epoch_single_anchor_bias", "fault_models.uwb");
  cfg.fault_models.uwb.persistent_anchor_bias =
      required<bool>(uwb_fault, "persistent_anchor_bias", "fault_models.uwb");
  cfg.fault_models.uwb.ramp_bias =
      required<bool>(uwb_fault, "ramp_bias", "fault_models.uwb");
  cfg.fault_models.uwb.prior_probability_bound = required<double>(
      uwb_fault, "prior_probability_bound", "fault_models.uwb");
  cfg.fault_models.uwb.p_md =
      required<double>(uwb_fault, "p_md", "fault_models.uwb");
  probability(cfg.fault_models.uwb.prior_probability_bound,
              "fault_models.uwb.prior_probability_bound");
  probability(cfg.fault_models.uwb.p_md, "fault_models.uwb.p_md");
  if (!cfg.fault_models.uwb.enabled ||
      !cfg.fault_models.uwb.epoch_single_anchor_bias ||
      !cfg.fault_models.uwb.persistent_anchor_bias ||
      !cfg.fault_models.uwb.ramp_bias) {
    throw std::runtime_error(
        "fault_models.uwb is outside the initial formal scope");
  }

  const auto imu_fault = fault_models["imu"];
  rejectUnknown(imu_fault, "fault_models.imu",
                {"accel_axis_interval_bias", "gyro_axis_interval_bias",
                 "accel_xyz_interval_bias", "gyro_xyz_interval_bias",
                 "bias_jump", "ramp_fault", "accel_prior_probability_bound",
                 "gyro_prior_probability_bound", "p_md", "min_fault_gram_sigma",
                 "max_fault_gram_condition"});
  cfg.fault_models.imu.accel_axis_interval_bias =
      required<bool>(imu_fault, "accel_axis_interval_bias", "fault_models.imu");
  cfg.fault_models.imu.gyro_axis_interval_bias =
      required<bool>(imu_fault, "gyro_axis_interval_bias", "fault_models.imu");
  cfg.fault_models.imu.accel_xyz_interval_bias =
      required<bool>(imu_fault, "accel_xyz_interval_bias", "fault_models.imu");
  cfg.fault_models.imu.gyro_xyz_interval_bias =
      required<bool>(imu_fault, "gyro_xyz_interval_bias", "fault_models.imu");
  cfg.fault_models.imu.bias_jump =
      required<bool>(imu_fault, "bias_jump", "fault_models.imu");
  cfg.fault_models.imu.ramp_fault =
      required<bool>(imu_fault, "ramp_fault", "fault_models.imu");
  cfg.fault_models.imu.accel_prior_probability_bound = required<double>(
      imu_fault, "accel_prior_probability_bound", "fault_models.imu");
  cfg.fault_models.imu.gyro_prior_probability_bound = required<double>(
      imu_fault, "gyro_prior_probability_bound", "fault_models.imu");
  cfg.fault_models.imu.p_md =
      required<double>(imu_fault, "p_md", "fault_models.imu");
  cfg.fault_models.imu.min_fault_gram_sigma =
      required<double>(imu_fault, "min_fault_gram_sigma", "fault_models.imu");
  cfg.fault_models.imu.max_fault_gram_condition = required<double>(
      imu_fault, "max_fault_gram_condition", "fault_models.imu");
  probability(cfg.fault_models.imu.accel_prior_probability_bound,
              "fault_models.imu.accel_prior_probability_bound");
  probability(cfg.fault_models.imu.gyro_prior_probability_bound,
              "fault_models.imu.gyro_prior_probability_bound");
  probability(cfg.fault_models.imu.p_md, "fault_models.imu.p_md");
  positive(cfg.fault_models.imu.min_fault_gram_sigma,
           "fault_models.imu.min_fault_gram_sigma");
  positive(cfg.fault_models.imu.max_fault_gram_condition,
           "fault_models.imu.max_fault_gram_condition");
  if (!cfg.fault_models.imu.accel_axis_interval_bias ||
      !cfg.fault_models.imu.gyro_axis_interval_bias ||
      cfg.fault_models.imu.accel_xyz_interval_bias ||
      cfg.fault_models.imu.gyro_xyz_interval_bias ||
      cfg.fault_models.imu.bias_jump || cfg.fault_models.imu.ramp_fault) {
    throw std::runtime_error(
        "fault_models.imu is outside the initial formal scope");
  }

  const auto combinations = fault_models["combinations"];
  rejectUnknown(combinations, "fault_models.combinations",
                {"uwb_plus_accel", "uwb_plus_gyro", "two_uwb",
                 "assume_independent_priors"});
  cfg.fault_models.combinations.uwb_plus_accel = required<bool>(
      combinations, "uwb_plus_accel", "fault_models.combinations");
  cfg.fault_models.combinations.uwb_plus_gyro = required<bool>(
      combinations, "uwb_plus_gyro", "fault_models.combinations");
  cfg.fault_models.combinations.two_uwb =
      required<bool>(combinations, "two_uwb", "fault_models.combinations");
  cfg.fault_models.combinations.assume_independent_priors = required<bool>(
      combinations, "assume_independent_priors", "fault_models.combinations");
  if (cfg.fault_models.combinations.two_uwb ||
      cfg.fault_models.combinations.assume_independent_priors) {
    throw std::runtime_error(
        "fault_models.combinations is outside the initial formal scope");
  }
  if (cfg.fault_models.double_faults_enabled &&
      !cfg.fault_models.combinations.uwb_plus_accel &&
      !cfg.fault_models.combinations.uwb_plus_gyro) {
    throw std::runtime_error(
        "double_faults_enabled requires uwb_plus_accel or uwb_plus_gyro");
  }

  const auto fde = root["fde"];
  rejectUnknown(
      fde, "fde",
      {"trigger", "isolation", "ambiguity_policy", "selection_primary",
       "selection_secondary", "dense_oracle_online_fallback",
       "max_candidate_count", "max_exclusion_cardinality", "profile",
       "on_no_valid_action", "on_integrity_model_invalid"});
  if (is_v6) {
    cfg.fde.profile = parseFdeProfile(required<std::string>(fde, "profile", "fde"));
    cfg.fde.on_no_valid_action =
        required<std::string>(fde, "on_no_valid_action", "fde");
    cfg.fde.on_integrity_model_invalid = required<std::string>(
        fde, "on_integrity_model_invalid", "fde");
  } else {
    if (!cfg.fault_models.single_faults_enabled &&
        cfg.fault_models.double_faults_enabled) {
      throw std::runtime_error(
          "v5 double-only configuration cannot migrate to a five-mode "
          "profile; enable singles or use the legacy research entry point");
    }
    cfg.fde.profile = cfg.fault_models.double_faults_enabled
                          ? FdeProfile::JointOrder2
                          : FdeProfile::JointOrder1;
    cfg.migration_warnings.push_back(
        std::string("legacy v5 fault switches mapped to fde.profile=") +
        toString(cfg.fde.profile));
  }
  cfg.fde.trigger = required<std::string>(fde, "trigger", "fde");
  cfg.fde.isolation = required<std::string>(fde, "isolation", "fde");
  cfg.fde.ambiguity_policy =
      required<std::string>(fde, "ambiguity_policy", "fde");
  cfg.fde.selection_primary =
      required<std::string>(fde, "selection_primary", "fde");
  cfg.fde.selection_secondary =
      required<std::string>(fde, "selection_secondary", "fde");
  cfg.fde.dense_oracle_online_fallback =
      required<bool>(fde, "dense_oracle_online_fallback", "fde");
  cfg.fde.max_candidate_count =
      required<std::uint32_t>(fde, "max_candidate_count", "fde");
  cfg.fde.max_exclusion_cardinality =
      required<std::uint32_t>(fde, "max_exclusion_cardinality", "fde");
  if (cfg.fde.trigger != "joint_detector_alarm_or_hardware_barrier" ||
      cfg.fde.isolation != "profile_parity_glrt" ||
      cfg.fde.ambiguity_policy != "union_exclusion_else_unavailable" ||
      cfg.fde.selection_primary != "minimum_cardinality" ||
      cfg.fde.selection_secondary != "minimum_protection_level" ||
      cfg.fde.dense_oracle_online_fallback ||
      cfg.fde.max_candidate_count == 0 ||
      cfg.fde.max_exclusion_cardinality == 0 ||
      cfg.fde.max_exclusion_cardinality > cfg.fault_models.max_cardinality ||
      cfg.fde.on_no_valid_action != "discard" ||
      cfg.fde.on_integrity_model_invalid != "discard") {
    throw std::runtime_error(
        "fde configuration does not match the v4 research architecture");
  }

  const auto bridge = root["bridge"];
  rejectUnknown(bridge, "bridge",
                {"generic", "quadrotor_dynamics", "max_consecutive_epochs",
                 "max_duration_s"});
  const auto generic = bridge["generic"];
  rejectUnknown(
      generic, "bridge.generic",
      {"enabled", "model", "optimization_sigma_position_m",
       "optimization_sigma_velocity_mps", "optimization_sigma_rotation_rad",
       "optimization_sigma_bias_accel", "optimization_sigma_bias_gyro",
       "acceleration_bound_mps2", "angular_rate_bound_radps",
       "angular_acceleration_bound_radps2", "integrity_model",
       "calibration_id"});
  cfg.bridge.generic.enabled =
      required<bool>(generic, "enabled", "bridge.generic");
  cfg.bridge.generic.model =
      required<std::string>(generic, "model", "bridge.generic");
  cfg.bridge.generic.optimization_sigma_position_m = required<double>(
      generic, "optimization_sigma_position_m", "bridge.generic");
  cfg.bridge.generic.optimization_sigma_velocity_mps = required<double>(
      generic, "optimization_sigma_velocity_mps", "bridge.generic");
  cfg.bridge.generic.optimization_sigma_rotation_rad = required<double>(
      generic, "optimization_sigma_rotation_rad", "bridge.generic");
  cfg.bridge.generic.optimization_sigma_bias_accel = required<double>(
      generic, "optimization_sigma_bias_accel", "bridge.generic");
  cfg.bridge.generic.optimization_sigma_bias_gyro = required<double>(
      generic, "optimization_sigma_bias_gyro", "bridge.generic");
  cfg.bridge.generic.acceleration_bound_mps2 =
      required<double>(generic, "acceleration_bound_mps2", "bridge.generic");
  cfg.bridge.generic.angular_rate_bound_radps =
      required<double>(generic, "angular_rate_bound_radps", "bridge.generic");
  cfg.bridge.generic.angular_acceleration_bound_radps2 = required<double>(
      generic, "angular_acceleration_bound_radps2", "bridge.generic");
  cfg.bridge.generic.integrity_model =
      required<std::string>(generic, "integrity_model", "bridge.generic");
  if (generic["calibration_id"])
    cfg.bridge.generic.calibration_id =
        generic["calibration_id"].as<std::string>();
  positive(cfg.bridge.generic.optimization_sigma_position_m,
           "bridge.generic.optimization_sigma_position_m");
  positive(cfg.bridge.generic.optimization_sigma_velocity_mps,
           "bridge.generic.optimization_sigma_velocity_mps");
  positive(cfg.bridge.generic.optimization_sigma_rotation_rad,
           "bridge.generic.optimization_sigma_rotation_rad");
  positive(cfg.bridge.generic.optimization_sigma_bias_accel,
           "bridge.generic.optimization_sigma_bias_accel");
  positive(cfg.bridge.generic.optimization_sigma_bias_gyro,
           "bridge.generic.optimization_sigma_bias_gyro");
  positive(cfg.bridge.generic.acceleration_bound_mps2,
           "bridge.generic.acceleration_bound_mps2");
  positive(cfg.bridge.generic.angular_rate_bound_radps,
           "bridge.generic.angular_rate_bound_radps");
  positive(cfg.bridge.generic.angular_acceleration_bound_radps2,
           "bridge.generic.angular_acceleration_bound_radps2");
  if (!cfg.bridge.generic.enabled ||
      cfg.bridge.generic.model != "constant_velocity_constant_attitude" ||
      cfg.bridge.generic.integrity_model != "deterministic_box") {
    throw std::runtime_error(
        "bridge.generic must be the bounded CV/constant-attitude bridge");
  }
  const auto dynamics = bridge["quadrotor_dynamics"];
  rejectUnknown(dynamics, "bridge.quadrotor_dynamics",
                {"enabled", "require_control_input", "model_calibration_id",
                 "integrity_model"});
  cfg.bridge.dynamics_enabled =
      required<bool>(dynamics, "enabled", "bridge.quadrotor_dynamics");
  cfg.bridge.dynamics_require_control_input = required<bool>(
      dynamics, "require_control_input", "bridge.quadrotor_dynamics");
  cfg.bridge.dynamics_model_calibration_id = required<std::string>(
      dynamics, "model_calibration_id", "bridge.quadrotor_dynamics");
  cfg.bridge.dynamics_integrity_model = required<std::string>(
      dynamics, "integrity_model", "bridge.quadrotor_dynamics");
  cfg.bridge.max_consecutive_epochs =
      required<std::uint32_t>(bridge, "max_consecutive_epochs", "bridge");
  cfg.bridge.max_duration_s =
      required<double>(bridge, "max_duration_s", "bridge");
  positive(cfg.bridge.max_duration_s, "bridge.max_duration_s");
  if (cfg.bridge.dynamics_enabled ||
      !cfg.bridge.dynamics_require_control_input ||
      cfg.bridge.max_consecutive_epochs == 0) {
    throw std::runtime_error(
        "formal dynamics bridge is unsupported and bridge limits must be "
        "positive");
  }

  const auto health = root["health"];
  rejectUnknown(health, "health",
                {"suspect_evidence_count", "quarantine_after_exclusion",
                 "recovery_shadow_passes", "recovery_test_passes",
                 "allow_auto_recovery"});
  cfg.health.suspect_evidence_count =
      required<std::uint32_t>(health, "suspect_evidence_count", "health");
  cfg.health.quarantine_after_exclusion =
      required<bool>(health, "quarantine_after_exclusion", "health");
  cfg.health.recovery_shadow_passes =
      required<std::uint32_t>(health, "recovery_shadow_passes", "health");
  cfg.health.recovery_test_passes =
      required<std::uint32_t>(health, "recovery_test_passes", "health");
  cfg.health.allow_auto_recovery =
      required<bool>(health, "allow_auto_recovery", "health");
  if (cfg.health.suspect_evidence_count == 0 ||
      cfg.health.recovery_shadow_passes == 0 ||
      cfg.health.recovery_test_passes == 0)
    throw std::runtime_error("health counters must be > 0");

  const auto risk = root["risk"];
  rejectUnknown(risk, "risk",
                {"calibration_id", "p_hmi_total", "nominal_axis_tail", "p_nm",
                 "p_bridge_escape", "p_history_contamination", "p_model_escape",
                 "horizontal_alert_limit_m", "vertical_alert_limit_m",
                 "allocation_policy"});
  cfg.risk.p_fa = cfg.detector.p_fa_per_test;
  cfg.risk.p_hmi_total = required<double>(risk, "p_hmi_total", "risk");
  cfg.risk.nominal_axis_tail =
      required<double>(risk, "nominal_axis_tail", "risk");
  cfg.risk.p_nm = required<double>(risk, "p_nm", "risk");
  cfg.risk.horizontal_alert_limit_m =
      required<double>(risk, "horizontal_alert_limit_m", "risk");
  cfg.risk.vertical_alert_limit_m =
      required<double>(risk, "vertical_alert_limit_m", "risk");
  cfg.risk_v2.p_hmi_total = cfg.risk.p_hmi_total;
  cfg.risk_v2.nominal_axis_tail = cfg.risk.nominal_axis_tail;
  cfg.risk_v2.p_nm = cfg.risk.p_nm;
  cfg.risk_v2.p_bridge_escape =
      required<double>(risk, "p_bridge_escape", "risk");
  cfg.risk_v2.p_history_contamination =
      required<double>(risk, "p_history_contamination", "risk");
  cfg.risk_v2.p_model_escape = required<double>(risk, "p_model_escape", "risk");
  cfg.risk_v2.horizontal_alert_limit_m = cfg.risk.horizontal_alert_limit_m;
  cfg.risk_v2.vertical_alert_limit_m = cfg.risk.vertical_alert_limit_m;
  cfg.risk_v2.allocation_policy =
      required<std::string>(risk, "allocation_policy", "risk");
  cfg.risk_v2.calibration_id =
      required<std::string>(risk, "calibration_id", "risk");
  probability(cfg.risk.p_fa, "risk.p_fa");
  probability(cfg.risk.p_hmi_total, "risk.p_hmi_total");
  probability(cfg.risk.nominal_axis_tail, "risk.nominal_axis_tail");
  probability(cfg.risk.p_nm, "risk.p_nm");
  positive(cfg.risk.horizontal_alert_limit_m, "risk.horizontal_alert_limit_m");
  positive(cfg.risk.vertical_alert_limit_m, "risk.vertical_alert_limit_m");
  nonnegativeProbability(cfg.risk_v2.p_bridge_escape, "risk.p_bridge_escape");
  nonnegativeProbability(cfg.risk_v2.p_history_contamination,
                         "risk.p_history_contamination");
  nonnegativeProbability(cfg.risk_v2.p_model_escape, "risk.p_model_escape");
  if (cfg.risk_v2.allocation_policy != "prior_weighted_outcome_conditioned") {
    throw std::runtime_error(
        "risk.allocation_policy must be prior_weighted_outcome_conditioned");
  }
  const double p_md = cfg.fault_models.uwb.p_md;
  const double prior = cfg.fault_models.uwb.prior_probability_bound;

  const auto robust = root["robust_shadow"];
  rejectUnknown(robust, "robust_shadow",
                {"gnc_enabled", "never_mutate_formal_weights"});
  cfg.robust_shadow.gnc_enabled =
      required<bool>(robust, "gnc_enabled", "robust_shadow");
  cfg.robust_shadow.never_mutate_formal_weights =
      required<bool>(robust, "never_mutate_formal_weights", "robust_shadow");
  if (!cfg.robust_shadow.never_mutate_formal_weights) {
    throw std::runtime_error(
        "robust_shadow.never_mutate_formal_weights must be true");
  }

  const auto output = root["output"];
  rejectUnknown(output, "output",
                {"root", "schema_version", "write_residuals", "write_timing",
                 "write_global_diagnostics", "write_factor_ledger",
                 "write_window_rows", "write_hypothesis_evidence",
                 "write_candidates", "write_health", "write_bridge"});
  cfg.output.root = required<std::string>(output, "root", "output");
  cfg.output.schema_version =
      required<std::string>(output, "schema_version", "output");
  cfg.output.write_residuals =
      required<bool>(output, "write_residuals", "output");
  cfg.output.write_timing = required<bool>(output, "write_timing", "output");
  cfg.output.write_global_diagnostics =
      required<bool>(output, "write_global_diagnostics", "output");
  cfg.output.write_factor_ledger =
      required<bool>(output, "write_factor_ledger", "output");
  cfg.output.write_window_rows =
      required<bool>(output, "write_window_rows", "output");
  cfg.output.write_hypothesis_evidence =
      required<bool>(output, "write_hypothesis_evidence", "output");
  cfg.output.write_candidates =
      required<bool>(output, "write_candidates", "output");
  cfg.output.write_health = required<bool>(output, "write_health", "output");
  cfg.output.write_bridge = required<bool>(output, "write_bridge", "output");
  if (cfg.output.root.empty())
    throw std::runtime_error("output.root must not be empty");
  if (cfg.output.schema_version != cfg.schema_version) {
    throw std::runtime_error(
        "output.schema_version must equal root schema_version");
  }

  if (is_v6) {
    const auto publication = root["publication"];
    rejectUnknown(publication, "publication",
                  {"allow_unprotected_output", "protected_output_enabled",
                   "deadline_ms"});
    cfg.publication.allow_unprotected_output = required<bool>(
        publication, "allow_unprotected_output", "publication");
    cfg.publication.protected_output_enabled = required<bool>(
        publication, "protected_output_enabled", "publication");
    cfg.publication.deadline_ms =
        required<double>(publication, "deadline_ms", "publication");
    positive(cfg.publication.deadline_ms, "publication.deadline_ms");
    if (!cfg.publication.allow_unprotected_output &&
        !cfg.publication.protected_output_enabled) {
      throw std::runtime_error(
          "publication must enable protected or unprotected output");
    }
  }

  const auto realtime = root["realtime"];
  rejectUnknown(realtime, "realtime",
                {"world_frame", "body_frame", "imu_topic", "uwb_topic",
                 "odometry_topic", "integrity_topic", "diagnostics_topic",
                 "initial_position_m", "initial_velocity_mps",
                 "lever_arm_body_m", "range_sigma_m", "prior_sigmas",
                 "max_queued_events", "imu_overflow_policy",
                 "uwb_overflow_policy"});
  cfg.realtime.world_frame =
      required<std::string>(realtime, "world_frame", "realtime");
  cfg.realtime.body_frame =
      required<std::string>(realtime, "body_frame", "realtime");
  cfg.realtime.imu_topic =
      required<std::string>(realtime, "imu_topic", "realtime");
  cfg.realtime.uwb_topic =
      required<std::string>(realtime, "uwb_topic", "realtime");
  cfg.realtime.odometry_topic =
      required<std::string>(realtime, "odometry_topic", "realtime");
  cfg.realtime.integrity_topic =
      required<std::string>(realtime, "integrity_topic", "realtime");
  cfg.realtime.diagnostics_topic =
      required<std::string>(realtime, "diagnostics_topic", "realtime");
  cfg.realtime.initial_position_m =
      vector3(realtime["initial_position_m"], "realtime.initial_position_m");
  cfg.realtime.initial_velocity_mps = vector3(realtime["initial_velocity_mps"],
                                              "realtime.initial_velocity_mps");
  cfg.realtime.lever_arm_body_m =
      vector3(realtime["lever_arm_body_m"], "realtime.lever_arm_body_m");
  cfg.realtime.range_sigma_m =
      required<double>(realtime, "range_sigma_m", "realtime");
  positive(cfg.realtime.range_sigma_m, "realtime.range_sigma_m");
  if (is_v6) {
    cfg.realtime.max_queued_events = required<std::uint32_t>(
        realtime, "max_queued_events", "realtime");
    cfg.realtime.imu_overflow_policy = required<std::string>(
        realtime, "imu_overflow_policy", "realtime");
    cfg.realtime.uwb_overflow_policy = required<std::string>(
        realtime, "uwb_overflow_policy", "realtime");
    if (cfg.realtime.max_queued_events == 0 ||
        cfg.realtime.imu_overflow_policy !=
            "invalidate_and_reinitialize" ||
        cfg.realtime.uwb_overflow_policy !=
            "drop_oldest_unprocessed_uwb") {
      throw std::runtime_error("realtime queue policy is invalid for v6");
    }
  }
  const auto prior_sigmas = realtime["prior_sigmas"];
  if (!prior_sigmas || !prior_sigmas.IsSequence() ||
      prior_sigmas.size() != 15) {
    throw std::runtime_error(
        "realtime.prior_sigmas must contain 15 values in Pose3,v,bias order");
  }
  for (int i = 0; i < 15; ++i) {
    cfg.realtime.prior_sigmas(i) =
        prior_sigmas[static_cast<std::size_t>(i)].as<double>();
    positive(cfg.realtime.prior_sigmas(i), "realtime.prior_sigmas");
  }

  const auto anchors = root["anchors"];
  if (!anchors || !anchors.IsSequence() || anchors.size() < 4) {
    throw std::runtime_error("anchors must contain at least four records");
  }
  std::set<std::uint64_t> anchor_ids;
  for (std::size_t i = 0; i < anchors.size(); ++i) {
    const auto anchor = anchors[i];
    rejectUnknown(anchor, "anchors[]",
                  {"id", "position_m", "sigma_m", "frame", "map_version"});
    AnchorRecord record;
    record.id = AnchorId(required<std::uint64_t>(anchor, "id", "anchors[]"));
    if (!anchor_ids.insert(record.id.value()).second) {
      throw std::runtime_error("duplicate physical anchor id");
    }
    record.position_world_m =
        vector3(anchor["position_m"], "anchors[].position_m");
    const double sigma = required<double>(anchor, "sigma_m", "anchors[]");
    positive(sigma, "anchors[].sigma_m");
    record.covariance_m2 = Eigen::Matrix3d::Identity() * sigma * sigma;
    record.frame = required<std::string>(anchor, "frame", "anchors[]");
    record.map_version =
        required<std::string>(anchor, "map_version", "anchors[]");
    if (record.frame != cfg.realtime.world_frame) {
      throw std::runtime_error(
          "anchor frame must equal realtime.world_frame in this release");
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
  const double allocated_hmi =
      3.0 * cfg.risk.nominal_axis_tail + cfg.risk.p_nm +
      cfg.risk_v2.p_bridge_escape + cfg.risk_v2.p_history_contamination +
      cfg.risk_v2.p_model_escape +
      static_cast<double>(cfg.risk.hypotheses.size()) * prior * p_md;
  if (!std::isfinite(allocated_hmi) || allocated_hmi > cfg.risk.p_hmi_total) {
    throw std::runtime_error(
        "risk.p_hmi_total is smaller than the complete anchor-map allocation");
  }

  // A3: optional fault manifest.  When declared it must be loadable and must
  // agree with the enabled fault-model switches; unsupported families are
  // rejected at startup instead of being silently accepted.
  if (!cfg.fault_models.manifest_path.empty()) {
    const std::string manifest_path =
        resolveRelativePath(yaml_path, cfg.fault_models.manifest_path);
    FaultManifest manifest = FaultManifestLoader::load(manifest_path);
    validateFaultManifestAgainstConfig(cfg, manifest);
    if (manifest.protected_quantity != "position_xyz" ||
        manifest.position_reference != "body_origin") {
      throw std::runtime_error(
          "fault manifest protected_quantity/position_reference must match the "
          "declared body-origin position protection");
    }
    cfg.fault_manifest = std::move(manifest);
  }

  FaultScopeCapabilities scope_capabilities;
  if (cfg.fault_manifest) {
    scope_capabilities.manifest_digest = cfg.fault_manifest->digest;
    scope_capabilities.uwb_anchor = manifestLists(
        cfg.fault_manifest->enabled_single_families, "uwb_single_anchor");
    scope_capabilities.imu_accel_axis = manifestLists(
        cfg.fault_manifest->enabled_single_families, "imu_single_axis");
    scope_capabilities.imu_gyro_axis = scope_capabilities.imu_accel_axis;
    scope_capabilities.uwb_imu_pair = manifestLists(
        cfg.fault_manifest->enabled_pair_families, "uwb_imu");
  } else if (is_v6 && cfg.fde.profile != FdeProfile::Off) {
    throw std::runtime_error(
        "active v6 fde.profile requires fault_models.manifest_path");
  }
  cfg.resolved_scope = resolveFaultScope(cfg.fde.profile, scope_capabilities);

  // Migration notes and the resolved fault manifest are part of the hashed
  // resolved configuration so a mismatch can never be silent.  Re-emit the
  // YAML first so migrated canonical keys (max_fault_order, intervals) appear
  // in the effective dump.
  {
    // Legacy switches are only an internal compatibility representation while
    // parsing.  A v6 resolved file has exactly one mode source: fde.profile.
    if (is_v6) {
      root["fault_models"].remove("single_faults_enabled");
      root["fault_models"].remove("double_faults_enabled");
      root["fault_models"].remove("max_fault_order");
    }
    cfg.resolved_yaml = emitResolvedYaml(root);
    cfg.resolved_yaml += "\n# --- migration notes (informational) ---\n";
    for (const auto& warning : cfg.migration_warnings) {
      cfg.resolved_yaml += "# " + warning + "\n";
    }
    if (cfg.fault_manifest) {
      cfg.resolved_yaml +=
          "# resolved_fault_manifest_id: " + cfg.fault_manifest->manifest_id +
          "\n";
      cfg.resolved_yaml +=
          "# resolved_fault_manifest_digest: " + cfg.fault_manifest->digest +
          "\n";
      cfg.resolved_yaml +=
          "# --- resolved fault manifest (read-only copy) ---\n";
      std::istringstream lines(cfg.fault_manifest->resolved_yaml);
      std::string line;
      while (std::getline(lines, line)) cfg.resolved_yaml += "# " + line + "\n";
    }
    cfg.resolved_yaml += "# resolved_fde_profile: " +
        std::string(toString(cfg.resolved_scope.profile)) + "\n";
    cfg.resolved_yaml += "# resolved_scope_digest: " +
        cfg.resolved_scope.scope_digest + "\n";
    cfg.resolved_yaml += "# detector_contract_id: " +
        cfg.resolved_scope.detector_contract_id + "\n";
    cfg.config_hash = fnv1a64(cfg.resolved_yaml);
  }
  return cfg;
}

std::string IntegrityConfigLoader::hashResolvedYaml(
    const std::string& resolved_yaml) {
  return fnv1a64(resolved_yaml);
}

}  // namespace uwb_imu_pl
