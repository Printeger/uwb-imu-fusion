// Paper-path runner for T02 all-range, T04/T05 oracle diagnostics, and T06
// automatic discovery/refit/score. Gate, fallback, and final covariance remain
// out of scope.

#include <boost/filesystem.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/Eigenvalues>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unistd.h>

#include "uifgo/config.h"
#include "uifgo/data_loader.h"
#include "uifgo/graph_builder.h"
#include "uifgo/hash_utils.h"
#include "uifgo/initializer.h"
#include "uifgo/nlos_discovery.h"
#include "uifgo/nlos_fde.h"
#include "uifgo/nlos_inference.h"
#include "uifgo/nlos_inference_io.h"
#include "uifgo/nlos_refit.h"
#include "uifgo/nlos_scoring.h"
#include "uifgo/optimizer.h"
#include "uifgo/paper_input.h"
#include "uifgo/paper_methods.h"
#include "uifgo/paper_pose_prior_factor.h"
#include "uifgo/paper_run_io.h"
#include "uifgo/paper_stage2_cache.h"
#include "uifgo/imu_preint.h"
#include <gtsam/navigation/CombinedImuFactor.h>
#include "uifgo/t07_scenario_cache.h"
#include "uifgo/uwb_factor.h"

namespace fs = boost::filesystem;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::C;

namespace {

constexpr char kRunnerElapsedSemantics[] =
    "RUNNER_WALL_FROM_MAIN_ENTRY_THROUGH_FINAL_STATUS_PREPARATION";

struct Args {
  bool prepare_only = false;
  std::string anchor_ids;
  std::string config_path;
  std::string output_root;
  std::string run_id;
  std::string method;
  std::string execution_type;
  std::string stage2_cache_manifest;
  std::string operating_point_id;
  size_t diagnostic_conditional_lm_outer = 0;
  bool diagnostic_fixed_checkpoint_lm_recovery = false;
  bool diagnostic_exact_lm_direction = false;
  bool diagnostic_first_block_budget = false;
  bool diagnostic_passive_terminal_capture = false;
};

std::string JsonEscape(const std::string& value) {
  std::ostringstream out;
  static const char hex[] = "0123456789abcdef";
  for (unsigned char c : value) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          out << "\\u00" << hex[(c >> 4) & 0x0f] << hex[c & 0x0f];
        } else {
          out << static_cast<char>(c);
        }
    }
  }
  return out.str();
}

std::string CsvEscape(const std::string& value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
  std::ostringstream out;
  out << '"';
  for (char c : value) {
    if (c == '"') out << '"';
    out << c;
  }
  out << '"';
  return out.str();
}

std::string JsonNumberOrNull(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

std::string CsvNumberOrEmpty(double value) {
  if (!std::isfinite(value)) return "";
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

std::string CsvLongDouble(long double value) {
  if (!std::isfinite(value)) return "";
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<long double>::max_digits10)
      << value;
  return out.str();
}

std::string DefaultRunId() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm utc;
  gmtime_r(&time, &utc);
  std::ostringstream out;
  out << "paper_t02_" << std::put_time(&utc, "%Y%m%dT%H%M%SZ") << '_'
      << getpid();
  return out.str();
}

Args ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto take = [&](const std::string& name) {
      if (i + 1 >= argc) throw std::invalid_argument(name + " needs a value");
      return std::string(argv[++i]);
    };
    if (arg == "--prepare-only") args.prepare_only = true;
    else if (arg == "--anchor-ids") args.anchor_ids = take(arg);
    else if (arg == "--config")
      args.config_path = take(arg);
    else if (arg == "--output-root")
      args.output_root = take(arg);
    else if (arg == "--run-id")
      args.run_id = take(arg);
    else if (arg == "--method")
      args.method = take(arg);
    else if (arg == "--execution-type")
      args.execution_type = take(arg);
    else if (arg == "--stage2-cache-manifest")
      args.stage2_cache_manifest = take(arg);
    else if (arg == "--operating-point-id")
      args.operating_point_id = take(arg);
    else if (arg == "--diagnostic-conditional-lm-outer") {
      const std::string value = take(arg);
      size_t consumed = 0;
      const unsigned long long parsed = std::stoull(value, &consumed);
      if (consumed != value.size() || parsed == 0)
        throw std::invalid_argument(
            "--diagnostic-conditional-lm-outer must be a positive integer");
      args.diagnostic_conditional_lm_outer =
          static_cast<size_t>(parsed);
    }
    else if (arg == "--diagnostic-fixed-checkpoint-lm-recovery")
      args.diagnostic_fixed_checkpoint_lm_recovery = true;
    else if (arg == "--diagnostic-passive-terminal-capture")
      args.diagnostic_passive_terminal_capture = true;
    else if (arg == "--diagnostic-first-block-budget")
      args.diagnostic_first_block_budget = true;
    else if (arg == "--diagnostic-exact-lm-direction")
      args.diagnostic_exact_lm_direction = true;
    else if (arg == "--help") {
      std::cout << "Usage: uwb_imu_fgo_paper_runner --config FILE "
                   "--output-root DIR [--run-id ID] [--method MODE] "
                   "[--execution-type TYPE] [--stage2-cache-manifest FILE] "
                   "[--diagnostic-conditional-lm-outer N] "
                   "[--diagnostic-fixed-checkpoint-lm-recovery] "
                   "[--diagnostic-exact-lm-direction] [--diagnostic-first-block-budget] [--diagnostic-passive-terminal-capture]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }
  if (args.config_path.empty() || args.output_root.empty()) {
    throw std::invalid_argument("--config and --output-root are required");
  }
  if (args.run_id.empty()) args.run_id = DefaultRunId();
  if (args.diagnostic_passive_terminal_capture &&
      (args.diagnostic_conditional_lm_outer != 1 ||
       args.diagnostic_first_block_budget || args.diagnostic_fixed_checkpoint_lm_recovery ||
       args.diagnostic_exact_lm_direction))
    throw std::invalid_argument("A15 passive capture requires outer1 and no other diagnostic mode");
  if (args.diagnostic_first_block_budget &&
      (args.diagnostic_conditional_lm_outer != 1 ||
       args.diagnostic_fixed_checkpoint_lm_recovery ||
       args.diagnostic_exact_lm_direction))
    throw std::invalid_argument(
        "--diagnostic-first-block-budget requires outer 1 and no other diagnostic arm");
  if (args.diagnostic_fixed_checkpoint_lm_recovery &&
      args.diagnostic_conditional_lm_outer == 0)
    throw std::invalid_argument(
        "--diagnostic-fixed-checkpoint-lm-recovery requires "
        "--diagnostic-conditional-lm-outer");
  if (args.diagnostic_exact_lm_direction &&
      args.diagnostic_conditional_lm_outer == 0)
    throw std::invalid_argument(
        "--diagnostic-exact-lm-direction requires "
        "--diagnostic-conditional-lm-outer");
  if (args.diagnostic_exact_lm_direction &&
      args.diagnostic_conditional_lm_outer != 123 &&
      args.diagnostic_conditional_lm_outer != 168)
    throw std::invalid_argument(
        "--diagnostic-exact-lm-direction is limited to frozen outer "
        "checkpoints 123 and 168");
  if (args.diagnostic_exact_lm_direction &&
      args.diagnostic_fixed_checkpoint_lm_recovery)
    throw std::invalid_argument(
        "--diagnostic-exact-lm-direction cannot enable fixed-checkpoint "
        "shadow recovery");
  if (!args.prepare_only && ((args.execution_type == "FINAL_TRAJECTORY") !=
      !args.stage2_cache_manifest.empty()))
    throw std::invalid_argument(
        "FINAL_TRAJECTORY requires exactly one Stage-2 cache manifest");
  if (args.run_id.find('/') != std::string::npos || args.run_id == "." ||
      args.run_id == "..") {
    throw std::invalid_argument("run-id must be one path component");
  }
  return args;
}

std::ofstream Open(const fs::path& path) {
  std::ofstream out(path.string());
  if (!out) throw std::runtime_error("cannot write " + path.string());
  return out;
}

std::uint64_t Fnv1aFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot hash " + path);
  std::uint64_t hash = 1469598103934665603ULL;
  char buffer[8192];
  while (in) {
    in.read(buffer, sizeof(buffer));
    for (std::streamsize i = 0; i < in.gcount(); ++i) {
      hash ^= static_cast<unsigned char>(buffer[i]);
      hash *= 1099511628211ULL;
    }
  }
  return hash;
}

std::string Hex(std::uint64_t value) {
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << value;
  return out.str();
}

std::string CalibrationContextHash(const uifgo::Config& cfg, const gtsam::Vector3& gravity) {
  std::ostringstream canonical;
  canonical << std::setprecision(17)
            << "uifgo-t14-fixed-calibration-and-imu-v1\n"
            << uifgo::ImuCovarianceModelIdentity(cfg, gravity, cfg.paper_imu_covariance_model) << '\n'
            << cfg.calib_anchor << ',' << cfg.calib_lever << ','
            << cfg.calib_range_bias << ',' << cfg.calib_td << '\n'
            << cfg.lever_arm_init.x() << ',' << cfg.lever_arm_init.y() << ','
            << cfg.lever_arm_init.z() << '\n' << cfg.td_init << '\n';
  for (const auto& anchor : cfg.anchors)
    canonical << anchor.id << ',' << anchor.pos.x() << ',' << anchor.pos.y()
              << ',' << anchor.pos.z() << ',' << anchor.prior_sigma << '\n';
  for (const auto& beta : cfg.fixed_beta_by_link)
    canonical << beta.first << ',' << beta.second << '\n';
  return "sha256:" + uifgo::Sha256Hex(canonical.str());
}

uifgo::ConditionalLmPolicy ConditionalLmPolicyFromConfig(
    const uifgo::Config& cfg) {
  if (cfg.discovery_conditional_navigation_policy ==
      "GTSAM_CHECK_ONLY_V1")
    return uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1;
  if (cfg.discovery_conditional_navigation_policy ==
      "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1")
    return uifgo::ConditionalLmPolicy::
        GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1;
  if (cfg.discovery_conditional_navigation_policy ==
      "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2")
    return uifgo::ConditionalLmPolicy::
        GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  throw std::invalid_argument(
      "unsupported conditional navigation policy after config validation");
}

bool ConditionalNavigationQualificationEnabled(const uifgo::Config& cfg) {
  return cfg.discovery_conditional_navigation_policy !=
         "GTSAM_CHECK_ONLY_V1";
}

double ConditionalNavigationInternalRelativeTolerance(
    const uifgo::Config& cfg) {
  return cfg.discovery_conditional_navigation_policy ==
                 "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2"
             ? 0.0
             : cfg.lm_rel_tol;
}

std::string DiscoverySolverConfigHash(const uifgo::Config& cfg) {
  std::ostringstream canonical;
  canonical << std::setprecision(17)
            << "uifgo-t06-stage1-solver-config-v3\n"
            << uifgo::PaperPosePriorJacobianIdentity() << '\n'
            << cfg.discovery_lambda_l1 << '\n' << cfg.discovery_lambda_tv << '\n'
            << cfg.discovery_gap_threshold_s << '\n'
            << cfg.discovery_active_bias_min_m << '\n'
            << cfg.discovery_change_point_min_m << '\n'
            << cfg.discovery_merge_max_difference_m << '\n'
            << cfg.discovery_short_min_count << '\n'
            << cfg.discovery_short_min_duration_s << '\n'
            << cfg.discovery_max_outer_iterations << '\n'
            << cfg.discovery_scaled_step_tolerance << '\n'
            << cfg.discovery_observation_bias_scale_m << '\n'
            << cfg.discovery_rho_scale << '\n'
            << cfg.discovery_primal_abs_tolerance_m << '\n'
            << cfg.discovery_primal_rel_tolerance << '\n'
            << cfg.discovery_dual_abs_tolerance_objective_per_m << '\n'
            << cfg.discovery_dual_rel_tolerance << '\n'
            << cfg.discovery_kkt_tolerance_objective_per_m << '\n'
            << cfg.discovery_tv_subgradient_tolerance_objective_per_m << '\n'
            << cfg.discovery_admm_max_iterations << '\n'
            << cfg.refit_relative_objective_tolerance << '\n'
            << cfg.refit_navigation_stationarity_tolerance_objective << '\n'
            << cfg.refit_gradient_roundoff_safety_factor << '\n'
            << cfg.refit_pose_rotation_scale_rad << '\n'
            << cfg.refit_pose_translation_scale_m << '\n'
            << cfg.refit_velocity_scale_mps << '\n'
            << cfg.refit_accel_bias_scale_mps2 << '\n'
            << cfg.refit_gyro_bias_scale_radps << '\n'
            << cfg.lm_max_iter << '\n' << cfg.lm_rel_tol << '\n'
            << cfg.lm_abs_tol << '\n';
  if (cfg.discovery_conditional_navigation_policy ==
      "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1") {
    canonical << "a02-conditional-navigation-policy-v1\n"
              << cfg.discovery_conditional_navigation_policy << '\n';
  } else if (cfg.discovery_conditional_navigation_policy ==
             "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2") {
    canonical << "a05-conditional-navigation-policy-v2\n"
              << cfg.discovery_conditional_navigation_policy << '\n'
              << "optimizer_internal_relative_tolerance=0\n"
              << "external_relative_tolerance=" << cfg.lm_rel_tol << '\n'
              << "external_absolute_tolerance=" << cfg.lm_abs_tol << '\n';
  }
  return "sha256:" + uifgo::Sha256Hex(canonical.str());
}

std::string Binary64Token(double value) {
  static_assert(sizeof(double) == sizeof(std::uint64_t),
                "T09 identities require binary64 doubles");
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << bits;
  return out.str();
}

std::string GateThresholdsHash(double tau_eta, double tau_s_m,
                               double tau_gamma) {
  std::ostringstream canonical;
  canonical << "uifgo-t09-thresholds-binary64-v1\n"
            << "tau_eta=" << Binary64Token(tau_eta) << '\n'
            << "tau_s_m=" << Binary64Token(tau_s_m) << '\n'
            << "tau_gamma=" << Binary64Token(tau_gamma) << '\n';
  return "sha256:" + uifgo::Sha256Hex(canonical.str());
}

std::string Stage2RefitConfigHash(const uifgo::Config& cfg) {
  std::ostringstream canonical;
  canonical << "uifgo-t14-stage2-refit-config-v1\n"
            << uifgo::ImuCovarianceModelIdentity(cfg, gtsam::Vector3(0, 0, -cfg.gravity), cfg.paper_imu_covariance_model) << '\n'
            << uifgo::PaperPosePriorJacobianIdentity() << '\n'
            << Binary64Token(cfg.refit_boundary_epsilon_m) << '\n'
            << Binary64Token(cfg.refit_relative_objective_tolerance) << '\n'
            << Binary64Token(cfg.refit_scaled_step_tolerance) << '\n'
            << Binary64Token(cfg.refit_projected_gradient_tolerance) << '\n'
            << Binary64Token(
                   cfg.refit_navigation_stationarity_tolerance_objective) << '\n'
            << Binary64Token(cfg.refit_gradient_roundoff_safety_factor) << '\n'
            << Binary64Token(cfg.refit_pose_rotation_scale_rad) << '\n'
            << Binary64Token(cfg.refit_pose_translation_scale_m) << '\n'
            << Binary64Token(cfg.refit_velocity_scale_mps) << '\n'
            << Binary64Token(cfg.refit_accel_bias_scale_mps2) << '\n'
            << Binary64Token(cfg.refit_gyro_bias_scale_radps) << '\n'
            << Binary64Token(cfg.refit_segment_amplitude_scale_m) << '\n'
            << cfg.max_refit_iterations << '\n' << cfg.lm_max_iter << '\n'
            << Binary64Token(cfg.lm_rel_tol) << '\n'
            << Binary64Token(cfg.lm_abs_tol) << '\n';
  return "sha256:" + uifgo::Sha256Hex(canonical.str());
}

std::string Stage3ScoreConfigHash() {
  const uifgo::RecoverabilityOptions cfg;
  std::ostringstream canonical;
  canonical << "uifgo-t09-stage3-score-config-v1\n"
            << Binary64Token(cfg.rank_absolute_tolerance) << '\n'
            << Binary64Token(cfg.rank_relative_tolerance) << '\n'
            << Binary64Token(cfg.symmetry_absolute_tolerance) << '\n'
            << Binary64Token(cfg.symmetry_relative_tolerance) << '\n'
            << Binary64Token(cfg.psd_absolute_tolerance_m2_inv) << '\n'
            << Binary64Token(cfg.psd_relative_tolerance) << '\n'
            << Binary64Token(cfg.pd_absolute_tolerance_m2_inv) << '\n'
            << Binary64Token(cfg.pd_relative_tolerance) << '\n'
            << Binary64Token(cfg.eta_absolute_tolerance) << '\n'
            << Binary64Token(cfg.orthogonality_absolute_tolerance) << '\n'
            << Binary64Token(cfg.orthogonality_relative_tolerance) << '\n'
            << Binary64Token(cfg.roundoff_safety_factor) << '\n'
            << cfg.condition_estimator_max_iterations << '\n';
  return "sha256:" + uifgo::Sha256Hex(canonical.str());
}

std::string FinalRefitScoreConfigHash(const uifgo::Config& cfg) {
  return "sha256:" + uifgo::Sha256Hex(
      "uifgo-t09-final-refit-score-config-v1\n" +
      Stage2RefitConfigHash(cfg) + "\n" + Stage3ScoreConfigHash() + "\n");
}

std::string Stage1ProducerConfigHash(const uifgo::Config& cfg,
                                     const uifgo::FdeContext* fde_context) {
  if (cfg.nlos_mode == "automatic_discovery")
    return DiscoverySolverConfigHash(cfg);
  if (cfg.nlos_mode == "imu_aided_fde") {
    if (!fde_context)
      throw std::invalid_argument("FDE Stage1 identity context is required");
    uifgo::FdeOptions options;
    options.chi2_probability = cfg.chi2_reject_prob;
    options.chi2_degrees_of_freedom = 1;
    options.gap_threshold_s = cfg.discovery_gap_threshold_s;
    options.minimum_count =
        static_cast<size_t>(cfg.discovery_short_min_count);
    options.minimum_duration_s = cfg.discovery_short_min_duration_s;
    options.preliminary_lm.max_iterations = cfg.lm_max_iter;
    options.preliminary_lm.relative_tolerance = cfg.lm_rel_tol;
    options.preliminary_lm.absolute_tolerance = cfg.lm_abs_tol;
    options.preliminary_lm.policy =
        uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1;
    return uifgo::ComputeFdeIdentity(options, *fde_context);
  }
  return "sha256:" + uifgo::Sha256Hex(
      std::string("uifgo-t09-stage1-config-v2\n") +
      uifgo::PaperPosePriorJacobianIdentity() +
      "\nNOT_APPLICABLE_FIXED_PARTITION\n");
}

std::string CanonicalExistingPath(const std::string& path) {
  const fs::path canonical = fs::canonical(fs::absolute(path));
  return canonical.string();
}

void RebuildAnchorSigmas(uifgo::Config* cfg) {
  cfg->anchor_prior_sigmas.clear();
  for (const auto& anchor : cfg->anchors)
    cfg->anchor_prior_sigmas.push_back(anchor.prior_sigma);
}

struct LoadedInput {
  std::string source;
  std::string source_hash_fnv1a64;
  std::string source_hash_sha256;
  std::string recording_id;
  std::string cache_id;
  double recording_time_origin_s =
      std::numeric_limits<double>::quiet_NaN();
  bool from_t07_cache = false;
};

LoadedInput LoadedFileInput(const std::string& source) {
  LoadedInput loaded;
  loaded.source = CanonicalExistingPath(source);
  loaded.source_hash_fnv1a64 = Hex(Fnv1aFile(loaded.source));
  loaded.source_hash_sha256 =
      "sha256:" + uifgo::Sha256FileHex(loaded.source);
  loaded.recording_id =
      "content-fnv1a64:" + loaded.source_hash_fnv1a64;
  return loaded;
}

LoadedInput LoadData(const std::string& config_dir, uifgo::Config* cfg,
                     std::vector<uifgo::ImuSample>* imu,
                     std::vector<uifgo::UwbFrame>* uwb) {
  uifgo::DataLoader loader(*cfg);
  if (cfg->data_interface == "sfuise") {
    const std::string dir = uifgo::ConfigLoader::ResolveBagPath(
        config_dir, cfg->sfuise_data_dir);
    std::vector<uifgo::AnchorConfig> discovered;
    if (!loader.LoadSfuiseBag(dir, cfg->sfuise_sequence, imu, uwb,
                              &discovered, true)) {
      throw std::runtime_error("SFUISE loader failed");
    }
    if (cfg->anchors.empty()) cfg->anchors = discovered;
    RebuildAnchorSigmas(cfg);
    return LoadedFileInput(
        dir + "/ISAS-Walk" + std::to_string(cfg->sfuise_sequence) + ".bag");
  }
  if (cfg->data_interface == "miluv") {
    if (cfg->miluv_uwb_group_window != 0.0 || !cfg->miluv_tag_levers.count(10))
      throw std::invalid_argument("MILUV paper requires group_window=0 and configured tag 10 lever");
    const std::string dir = uifgo::ConfigLoader::ResolveBagPath(config_dir,cfg->miluv_data_dir);
    std::vector<uifgo::AnchorConfig> unused_anchors;
    if (!loader.LoadMiluvCsv(dir,imu,uwb,&unused_anchors)) throw std::runtime_error("MILUV CSV loader failed");
    cfg->lever_arm_init=cfg->miluv_tag_levers.at(10);
    if (imu->empty() || uwb->empty()) throw std::runtime_error("MILUV empty input");
    const double origin=std::min(imu->front().t,uwb->front().t);
    const double begin=origin+cfg->bag_start;
    const double end=cfg->bag_durr<0 ? std::numeric_limits<double>::infinity() : begin+cfg->bag_durr;
    imu->erase(std::remove_if(imu->begin(),imu->end(),[&](const auto& x){return x.t<begin || x.t>end;}),imu->end());
    uwb->erase(std::remove_if(uwb->begin(),uwb->end(),[&](const auto& x){return x.tag_id!=10 || x.t<begin || x.t>end;}),uwb->end());
    const std::string base=dir+"/"+cfg->miluv_robot_dir+"/";
    auto loaded=LoadedFileInput(base+cfg->miluv_uwb_csv);
    loaded.source_hash_sha256="sha256:"+uifgo::Sha256Hex(
        loaded.source_hash_sha256+uifgo::Sha256FileHex(base+cfg->miluv_imu_csv));
    loaded.recording_id="miluv-"+uifgo::Sha256Hex(loaded.source_hash_sha256);
    loaded.recording_time_origin_s=origin;
    RebuildAnchorSigmas(cfg);
    return loaded;
  }
  if (cfg->data_interface == "original") {
    const std::string bag = uifgo::ConfigLoader::ResolveBagPath(
        config_dir, cfg->bag_path);
    if (!loader.LoadFromBag(bag, imu, uwb)) {
      throw std::runtime_error("original rosbag loader failed");
    }
    return LoadedFileInput(bag);
  }
  if (cfg->data_interface == "t07_cache") {
    fs::path manifest(cfg->t07_cache_manifest);
    if (manifest.is_relative()) manifest = fs::path(config_dir) / manifest;
    manifest = fs::canonical(fs::absolute(manifest));
    auto cache = uifgo::LoadT07ScenarioCache(
        manifest.string(), cfg->t07_cache_start_s,
        cfg->t07_cache_duration_s);
    *imu = std::move(cache.imu);
    *uwb = std::move(cache.uwb);
    LoadedInput loaded;
    loaded.source = manifest.string();
    loaded.source_hash_fnv1a64 = Hex(Fnv1aFile(loaded.source));
    loaded.source_hash_sha256 = cache.cache_id;
    loaded.recording_id = cache.base_recording_id;
    loaded.cache_id = cache.cache_id;
    loaded.recording_time_origin_s = cache.recording_time_origin_s;
    loaded.from_t07_cache = true;
    return loaded;
  }
  throw std::invalid_argument(
      "paper runner supports dataset.interface original, sfuise or t07_cache; got " +
      cfg->data_interface);
}

void ValidateSupportedConfig(const uifgo::Config& cfg) {
  if (cfg.calib_lever || cfg.calib_anchor || cfg.calib_range_bias ||
      cfg.calib_td) {
    throw std::invalid_argument(
        "T02 paper runner supports fixed extrinsics/anchors/time and no online "
        "range-bias state");
  }
  if (cfg.td_init != 0.0) {
    throw std::invalid_argument(
        "T02 paper runner does not implement fixed time offset; td_init must "
        "be exactly 0 s");
  }
  if (cfg.data_interface == "sfuise" &&
      cfg.sfuise_uwb_group_window != 0.0) {
    throw std::invalid_argument(
        "T02 paper runner requires sfuise.uwb_group_window=0 so every ledger "
        "observation keeps its source time and tag");
  }
  if (cfg.lm_max_iter <= 0) {
    throw std::invalid_argument("solver.lm_max_iter must be greater than 0");
  }
  if (!std::isfinite(cfg.lm_rel_tol) || cfg.lm_rel_tol < 0.0 ||
      !std::isfinite(cfg.lm_abs_tol) || cfg.lm_abs_tol < 0.0) {
    throw std::invalid_argument(
        "solver error tolerances must be finite and nonnegative");
  }
  if (cfg.nlos_mode == "oracle_debug" ||
      cfg.nlos_mode == "fixed_partition_debug" ||
      cfg.nlos_mode == "automatic_discovery" ||
      cfg.nlos_mode == "imu_aided_fde") {
    if (cfg.max_refit_iterations <= 0 ||
        cfg.oracle_short_min_count_debug < 0 ||
        !std::isfinite(cfg.oracle_short_min_duration_debug) ||
        cfg.oracle_short_min_duration_debug < 0.0 ||
        !std::isfinite(cfg.refit_boundary_epsilon_m) ||
        cfg.refit_boundary_epsilon_m < 0.0 ||
        !std::isfinite(cfg.refit_relative_objective_tolerance) ||
        cfg.refit_relative_objective_tolerance < 0.0 ||
        !std::isfinite(cfg.refit_scaled_step_tolerance) ||
        cfg.refit_scaled_step_tolerance < 0.0 ||
        !std::isfinite(cfg.refit_projected_gradient_tolerance) ||
        cfg.refit_projected_gradient_tolerance < 0.0 ||
        !std::isfinite(
            cfg.refit_navigation_stationarity_tolerance_objective) ||
        cfg.refit_navigation_stationarity_tolerance_objective < 0.0 ||
        !std::isfinite(cfg.refit_gradient_roundoff_safety_factor) ||
        cfg.refit_gradient_roundoff_safety_factor <= 0.0 ||
        !std::isfinite(cfg.refit_pose_rotation_scale_rad) ||
        cfg.refit_pose_rotation_scale_rad <= 0.0 ||
        !std::isfinite(cfg.refit_pose_translation_scale_m) ||
        cfg.refit_pose_translation_scale_m <= 0.0 ||
        !std::isfinite(cfg.refit_velocity_scale_mps) ||
        cfg.refit_velocity_scale_mps <= 0.0 ||
        !std::isfinite(cfg.refit_accel_bias_scale_mps2) ||
        cfg.refit_accel_bias_scale_mps2 <= 0.0 ||
        !std::isfinite(cfg.refit_gyro_bias_scale_radps) ||
        cfg.refit_gyro_bias_scale_radps <= 0.0 ||
        !std::isfinite(cfg.refit_segment_amplitude_scale_m) ||
        cfg.refit_segment_amplitude_scale_m <= 0.0) {
      throw std::invalid_argument(
          "support refit thresholds/iteration limits are invalid");
    }
  }
  if (cfg.nlos_mode == "automatic_discovery") {
    if (!cfg.score_recoverability)
      throw std::invalid_argument(
          "automatic discovery requires Stage-3 recoverability scoring");
    const double values[] = {
        cfg.discovery_lambda_l1,
        cfg.discovery_lambda_tv,
        cfg.discovery_gap_threshold_s,
        cfg.discovery_active_bias_min_m,
        cfg.discovery_change_point_min_m,
        cfg.discovery_merge_max_difference_m,
        cfg.discovery_short_min_duration_s,
        cfg.discovery_scaled_step_tolerance,
        cfg.discovery_observation_bias_scale_m,
        cfg.discovery_rho_scale,
        cfg.discovery_primal_abs_tolerance_m,
        cfg.discovery_primal_rel_tolerance,
        cfg.discovery_dual_abs_tolerance_objective_per_m,
        cfg.discovery_dual_rel_tolerance,
        cfg.discovery_kkt_tolerance_objective_per_m,
        cfg.discovery_tv_subgradient_tolerance_objective_per_m};
    for (double value : values)
      if (!std::isfinite(value) || value < 0.0)
        throw std::invalid_argument(
            "automatic discovery parameters must be finite/nonnegative");
    if (!(cfg.discovery_rho_scale > 0.0) ||
        !(cfg.discovery_observation_bias_scale_m > 0.0) ||
        cfg.discovery_short_min_count < 0 ||
        cfg.discovery_max_outer_iterations <= 0 ||
        cfg.discovery_admm_max_iterations <= 0)
      throw std::invalid_argument(
          "automatic discovery iteration/count/rho parameters are invalid");
  }
}

void WriteObservations(const fs::path& run_dir,
                       const uifgo::PaperInputPlan& plan,
                       const std::vector<bool>& mask) {
  auto out = Open(run_dir / "observations.csv");
  out << "obs_id,source_frame,source_message,source_range,"
         "source_observation,raw_time,raw_tag_id,anchor_id,raw_z_m,"
         "fp_rssi,rx_rssi,valid,validity_reason,suspected_nlos,planned,"
         "keyframe_id,nominal_sigma_m,strategy_used\n";
  out << std::setprecision(17);
  for (size_t i = 0; i < plan.observations.size(); ++i) {
    const auto& r = plan.observations[i];
    out << r.obs_id << ',' << r.source_frame_index << ','
        << r.source_message_index << ',' << r.source_range_index << ','
        << r.source_observation_index << ',' << r.sensor_time << ',' << r.tag_id
        << ',' << r.anchor_id << ',' << r.raw_range << ',' << r.fp_rssi << ','
        << r.rx_rssi << ',' << r.valid << ',' << r.validity_reason << ','
        << r.suspected_nlos << ',' << r.planned << ',';
    if (r.planned) out << r.keyframe_id;
    out << ',';
    if (r.planned) out << r.nominal_sigma;
    out << ',' << (mask[i] && r.valid && r.planned) << '\n';
  }
}

void WriteTrajectory(const fs::path& run_dir,
                     const std::vector<uifgo::UwbFrame>& keyframes,
                     const gtsam::Values& values) {
  auto out = Open(run_dir / "trajectory.tum");
  out << std::setprecision(17);
  for (size_t k = 0; k < keyframes.size(); ++k) {
    const auto pose = values.at<gtsam::Pose3>(X(k));
    const auto q = pose.rotation().toQuaternion();
    out << keyframes[k].t << ' ' << pose.x() << ' ' << pose.y() << ' '
        << pose.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
        << q.w() << '\n';
  }
}

void WriteBiases(const fs::path& run_dir,
                 const std::vector<uifgo::UwbFrame>& keyframes,
                 const gtsam::Values& values) {
  auto out = Open(run_dir / "imu_bias.csv");
  out << "keyframe_id,time,ba_x_mps2,ba_y_mps2,ba_z_mps2,"
         "bg_x_radps,bg_y_radps,bg_z_radps\n";
  out << std::setprecision(17);
  for (size_t k = 0; k < keyframes.size(); ++k) {
    const auto bias = values.at<gtsam::imuBias::ConstantBias>(B(k));
    const auto ba = bias.accelerometer();
    const auto bg = bias.gyroscope();
    out << k << ',' << keyframes[k].t << ',' << ba.x() << ',' << ba.y() << ','
        << ba.z() << ',' << bg.x() << ',' << bg.y() << ',' << bg.z() << '\n';
  }
}

void WriteResiduals(const fs::path& run_dir, const uifgo::Config& cfg,
                    const uifgo::PaperInputPlan& plan,
                    const gtsam::Values& values) {
  std::map<int, gtsam::Point3> anchors;
  for (const auto& anchor : cfg.anchors) anchors[anchor.id] = anchor.pos;
  auto out = Open(run_dir / "residuals.csv");
  out << "obs_id,keyframe_id,raw_z_m,geometric_range_m,fixed_beta_m,"
         "residual_m,nominal_sigma_m\n";
  out << std::setprecision(17);
  for (const auto& record : plan.observations) {
    if (!record.valid || !record.planned) continue;
    const auto pose = values.at<gtsam::Pose3>(X(record.keyframe_id));
    const gtsam::Point3 antenna = pose.transformFrom(cfg.lever_arm_init);
    const double geometric =
        (gtsam::Vector3(antenna) - gtsam::Vector3(anchors.at(record.anchor_id)))
            .norm();
    const double beta =
        uifgo::FixedBetaForLink(cfg, record.tag_id, record.anchor_id);
    const double residual =
        uifgo::UwbResidual(geometric, record.raw_range, beta);
    out << record.obs_id << ',' << record.keyframe_id << ','
        << record.raw_range << ',' << geometric << ',' << beta << ','
        << residual << ',' << record.nominal_sigma << '\n';
  }
}

std::string CsvKeys(const std::vector<gtsam::Key>& keys) {
  std::ostringstream out;
  for (size_t i = 0; i < keys.size(); ++i) {
    if (i) out << ';';
    out << gtsam::DefaultKeyFormatter(keys[i]);
  }
  return out.str();
}

template <typename Matrix>
void WriteDenseMatrixMarket(const fs::path& path, const Matrix& matrix) {
  auto out = Open(path);
  out << "%%MatrixMarket matrix array real general\n";
  out << matrix.rows() << ' ' << matrix.cols() << '\n' << std::setprecision(17);
  for (Eigen::Index column = 0; column < matrix.cols(); ++column)
    for (Eigen::Index row = 0; row < matrix.rows(); ++row)
      out << matrix(row, column) << '\n';
}

void WriteSparseMatrixMarket(const fs::path& path,
                             const Eigen::SparseMatrix<double>& matrix) {
  auto out = Open(path);
  out << "%%MatrixMarket matrix coordinate real general\n";
  out << matrix.rows() << ' ' << matrix.cols() << ' ' << matrix.nonZeros()
      << '\n' << std::setprecision(17);
  for (Eigen::Index column = 0; column < matrix.outerSize(); ++column)
    for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, column); it; ++it)
      out << it.row() + 1 << ' ' << it.col() + 1 << ' ' << it.value() << '\n';
}

bool WriteRecoverabilityScores(
    const fs::path& run_dir,
    const std::vector<uifgo::GroupRecoverabilityScore>& scores) {
  auto groups = Open(run_dir / "groups.csv");
  groups << "group_id,start_time,end_time,segment_ordinals,eligible,status,"
            "valid_score_exported,linearization_id\n";
  auto decisions = Open(run_dir / "scores_decision.csv");
  decisions << "group_id,status,eligible,valid_score_exported,score_availability,"
               "eta,s_m,s_is_infinite,lambda_min_N_m2_inv,s_semantics,debug_numerical_status,"
               "debug_eta,debug_s_m,debug_s_is_infinite,N_rank,R_rank,"
               "naive_qr_rank,sparse_qr_rank,frozen_F_rank,frozen_rank_certified,"
               "condition_1,rcond_1,orthogonality,rows,F_cols,G_cols,F_nnz,"
               "elapsed_seconds,linearization_id\n";
  auto fits = Open(run_dir / "segment_fit_scores.csv");
  fits << "group_id,segment_id,segment_ordinal,gamma,short_support_debug,"
          "boundary,status,linearization_id\n";
  groups << std::setprecision(17);
  decisions << std::setprecision(17);
  fits << std::setprecision(17);
  bool all_valid = true;
  for (const auto& score : scores) {
    std::ostringstream ordinals;
    for (size_t i = 0; i < score.group.segment_ordinals.size(); ++i) {
      if (i) ordinals << ';';
      ordinals << score.group.segment_ordinals[i];
    }
    groups << score.group.group_id << ',' << score.group.start_time << ','
           << score.group.end_time << ',' << ordinals.str() << ','
           << score.eligible << ',' << score.status << ','
           << score.valid_score_exported << ',' << score.linearization_id << '\n';
    const bool rank_deficient =
        score.numerical.status == uifgo::RecoverabilityStatus::RANK_DEFICIENT;
    const std::string availability = !score.eligible
        ? "NOT_APPLICABLE_SHORT_OR_BOUNDARY"
        : (!score.numerical.valid_score()
               ? "UNAVAILABLE_NUMERICAL"
               : (rank_deficient ? "AVAILABLE_RANK_DEFICIENT"
                                 : "AVAILABLE_FINITE"));
    const std::string s_semantics = !score.eligible
        ? "NOT_APPLICABLE"
        : (!score.numerical.valid_score()
               ? "UNAVAILABLE"
               : (rank_deficient ? "POSITIVE_INFINITY_RANK_DEFICIENT"
                                 : "FINITE"));
    decisions << score.group.group_id << ',' << score.status << ','
              << score.eligible << ',' << score.valid_score_exported << ','
              << availability << ',';
    if (score.valid_score_exported)
      decisions << JsonNumberOrNull(score.numerical.eta);
    decisions << ',';
    if (score.valid_score_exported && !score.numerical.s_is_infinite)
      decisions << JsonNumberOrNull(score.numerical.s_m);
    decisions << ',';
    if (score.valid_score_exported) decisions << score.numerical.s_is_infinite;
    double lambda_min_n = std::numeric_limits<double>::quiet_NaN();
    if (score.numerical.N.rows() > 0 &&
        score.numerical.N.rows() == score.numerical.N.cols() &&
        score.numerical.N.allFinite()) {
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(score.numerical.N);
      if (eig.info() == Eigen::Success)
        lambda_min_n = eig.eigenvalues().minCoeff();
    }
    decisions << ',' << JsonNumberOrNull(lambda_min_n)
              << ',' << s_semantics << ','
              << uifgo::RecoverabilityStatusName(score.numerical.status) << ','
              << JsonNumberOrNull(score.numerical.eta) << ',';
    if (!score.numerical.s_is_infinite)
      decisions << JsonNumberOrNull(score.numerical.s_m);
    decisions << ',' << score.numerical.s_is_infinite << ','
              << score.numerical.N_rank << ',' << score.numerical.R_rank << ','
              << score.numerical.naive_qr_pivot_rank << ','
              << score.numerical.sparse_qr_rank << ','
              << score.numerical.frozen_F_rank << ','
              << score.numerical.frozen_rank_certified << ','
              << JsonNumberOrNull(score.numerical.qr_condition_estimate_1) << ','
              << JsonNumberOrNull(score.numerical.qr_rcond_estimate_1) << ','
              << JsonNumberOrNull(score.numerical.orthogonality_residual) << ','
              << score.F_whitened.rows() << ',' << score.F_whitened.cols() << ','
              << score.G_whitened.cols() << ',' << score.F_whitened.nonZeros()
              << ',' << score.elapsed_seconds << ',' << score.linearization_id
              << '\n';
    for (const auto& fit : score.segment_fit)
      fits << score.group.group_id << ',' << CsvEscape(fit.segment_id) << ','
           << fit.segment_ordinal << ',' << fit.gamma << ','
           << fit.short_support_debug << ',' << fit.boundary << ','
           << score.status << ',' << score.linearization_id << '\n';

    const fs::path group_dir = run_dir / score.group.group_id;
    fs::create_directory(group_dir);
    WriteSparseMatrixMarket(group_dir / "F_whitened.mtx", score.F_whitened);
    WriteSparseMatrixMarket(group_dir / "F_scaled.mtx",
                            score.numerical.F_scaled);
    WriteDenseMatrixMarket(group_dir / "G_whitened.mtx", score.G_whitened);
    WriteDenseMatrixMarket(group_dir / "E.mtx", score.numerical.E);
    WriteDenseMatrixMarket(group_dir / "N.mtx", score.numerical.N);
    WriteDenseMatrixMarket(group_dir / "R.mtx", score.numerical.R);
    WriteDenseMatrixMarket(group_dir / "rhs_whitened.mtx",
                           score.rhs_whitened);
    auto factor_rows = Open(group_dir / "factor_rows.csv");
    factor_rows << "original_factor_index,row_offset,row_count,obs_id,"
                   "factor_type,segment_id\n";
    for (const auto& meta : score.factor_rows)
      factor_rows << meta.original_factor_index << ',' << meta.row_offset << ','
                  << meta.row_count << ',' << meta.obs_id << ','
                  << meta.factor_type << ',' << CsvEscape(meta.segment_id) << '\n';
    auto key_columns = Open(group_dir / "key_columns.csv");
    key_columns << "key,offset,dimension,role\n";
    for (const auto& meta : score.key_columns)
      key_columns << gtsam::DefaultKeyFormatter(meta.key) << ',' << meta.offset
                  << ',' << meta.dimension << ',' << meta.role << '\n';
    auto audit = Open(group_dir / "rank_condition_roundoff.json");
    audit << std::setprecision(17)
          << "{\n  \"status\": \"" << score.status << "\",\n"
          << "  \"numerical_status\": \""
          << uifgo::RecoverabilityStatusName(score.numerical.status) << "\",\n"
          << "  \"reason\": \"" << JsonEscape(score.numerical.reason)
          << "\",\n  \"rank_definition\": \"frozen scaled-F SVD; QR is diagnostic/guarded\",\n"
          << "  \"naive_qr_rank\": " << score.numerical.naive_qr_pivot_rank
          << ",\n  \"sparse_qr_rank\": " << score.numerical.sparse_qr_rank
          << ",\n  \"frozen_F_rank\": " << score.numerical.frozen_F_rank
          << ",\n  \"frozen_rank_certified\": "
          << (score.numerical.frozen_rank_certified ? "true" : "false")
          << ",\n  \"rank_threshold_lower\": "
          << JsonNumberOrNull(score.numerical.rank_threshold_lower)
          << ",\n  \"rank_threshold_upper\": "
          << JsonNumberOrNull(score.numerical.rank_threshold_upper)
          << ",\n  \"rank_certificate_sigma_min_lower\": "
          << JsonNumberOrNull(score.numerical.rank_certificate_sigma_min_lower)
          << ",\n  \"rank_certificate_sigma_max_upper\": "
          << JsonNumberOrNull(score.numerical.rank_certificate_sigma_max_upper)
          << ",\n  \"rank_certificate_inverse_residual\": "
          << JsonNumberOrNull(score.numerical.rank_certificate_inverse_residual)
          << ",\n  \"condition_estimate_1\": "
          << JsonNumberOrNull(score.numerical.qr_condition_estimate_1)
          << ",\n  \"rcond_estimate_1\": "
          << JsonNumberOrNull(score.numerical.qr_rcond_estimate_1)
          << ",\n  \"condition_iterations\": "
          << score.numerical.condition_estimator_iterations
          << ",\n  \"condition_converged\": "
          << (score.numerical.condition_estimator_converged ? "true" : "false")
          << ",\n  \"retained_pivot_min\": "
          << JsonNumberOrNull(score.numerical.retained_pivot_min)
          << ",\n  \"retained_pivot_max\": "
          << JsonNumberOrNull(score.numerical.retained_pivot_max)
          << ",\n  \"orthogonality_residual\": "
          << JsonNumberOrNull(score.numerical.orthogonality_residual)
          << ",\n  \"orthogonality_tolerance\": "
          << JsonNumberOrNull(score.numerical.orthogonality_tolerance)
          << ",\n  \"roundoff_multiplier\": "
          << JsonNumberOrNull(score.numerical.roundoff_multiplier)
          << ",\n  \"projection_residual_floor\": "
          << JsonNumberOrNull(score.numerical.projection_residual_floor)
          << ",\n  \"n_minus_r_min_eigenvalue\": "
          << JsonNumberOrNull(score.numerical.n_minus_r_min_eigenvalue)
          << ",\n  \"n_minus_r_psd_tolerance\": "
          << JsonNumberOrNull(score.numerical.n_minus_r_psd_tolerance)
          << ",\n  \"N_rank_pd_threshold\": "
          << JsonNumberOrNull(score.numerical.N_rank_pd_threshold)
          << ",\n  \"R_rank_pd_threshold\": "
          << JsonNumberOrNull(score.numerical.R_rank_pd_threshold)
          << ",\n  \"R_psd_threshold\": "
          << JsonNumberOrNull(score.numerical.R_psd_threshold)
          << ",\n  \"rhs_is_state_column\": false,\n"
          << "  \"whitening_applied_by_gtsam_once\": true\n}\n";
    all_valid = all_valid && score.valid_score_exported;
  }
  return all_valid;
}

void WriteRefitIterations(const fs::path& run_dir,
                          const uifgo::SegmentRefitResult& result,
                          const std::string& filename =
                              "refit_iterations.csv") {
  auto out = Open(run_dir / filename);
  out << "outer_iteration,conditional_lm_iterations,"
         "conditional_lm_inner_iterations,conditional_lm_lambda,"
         "objective_before,objective_after,relative_objective_change,"
         "scaled_state_step,max_kkt_violation,"
         "max_pose_rotation_gradient_objective_per_rad,"
         "max_pose_translation_gradient_objective_per_m,"
         "max_velocity_gradient_objective_per_mps,"
         "max_accel_bias_gradient_objective_per_mps2,"
         "max_gyro_bias_gradient_objective_per_radps,"
         "max_scaled_navigation_gradient_objective,"
         "navigation_gradient_roundoff_allowance_objective,"
         "navigation_stationarity_tolerance_objective,"
         "allowed_objective_increase,objective_ok,step_ok,kkt_ok,"
         "navigation_stationarity_ok\n";
  out << std::setprecision(17);
  for (const auto& trace : result.iterations) {
    out << trace.outer_iteration << ',' << trace.conditional_lm_iterations
        << ',' << trace.conditional_lm_inner_iterations << ','
        << trace.conditional_lm_lambda << ',' << trace.objective_before << ','
        << trace.objective_after << ',' << trace.relative_objective_change
        << ',' << trace.scaled_state_step << ',' << trace.max_kkt_violation
        << ',' << trace.max_pose_rotation_gradient_objective_per_rad << ','
        << trace.max_pose_translation_gradient_objective_per_m << ','
        << trace.max_velocity_gradient_objective_per_mps << ','
        << trace.max_accel_bias_gradient_objective_per_mps2 << ','
        << trace.max_gyro_bias_gradient_objective_per_radps << ','
        << trace.max_scaled_navigation_gradient_objective << ','
        << trace.navigation_gradient_roundoff_allowance_objective << ','
        << trace.navigation_stationarity_tolerance_objective << ','
        << trace.allowed_objective_increase << ',' << trace.objective_ok << ','
        << trace.step_ok << ',' << trace.kkt_ok << ','
        << trace.navigation_stationarity_ok << '\n';
  }
}

void WriteFactorMetadata(const fs::path& run_dir,
                         const uifgo::SegmentRefitResult& result) {
  auto out = Open(run_dir / "factor_metadata.csv");
  out << "factor_index,obs_id,factor_type,keys,segment_id\n";
  for (const auto& meta : result.factor_metadata) {
    out << meta.factor_index << ',';
    if (meta.obs_id != 0) out << meta.obs_id;
    out << ',' << meta.factor_type << ',' << CsvKeys(meta.keys) << ','
        << CsvEscape(meta.segment_id) << '\n';
  }
}

void WriteSegments(const fs::path& run_dir,
                   const uifgo::SegmentRefitResult& result,
                   const uifgo::Config& cfg) {
  auto out = Open(run_dir / "segments.csv");
  out << "segment_id,segment_ordinal,c_key,tag_id,anchor_id,obs_count,"
         "start_time,end_time,duration_s,amplitude_m,"
         "gradient_objective_per_m,kkt_violation,boundary,"
         "short_support_debug,short_min_count_debug,"
         "short_min_duration_debug_s,debug_label\n";
  out << std::setprecision(17);
  for (const auto& segment : result.segments) {
    out << CsvEscape(segment.segment_id) << ',' << segment.segment_ordinal << ','
        << gtsam::DefaultKeyFormatter(segment.amplitude_key) << ','
        << segment.tag_id << ',' << segment.anchor_id << ','
        << segment.observation_count << ',' << segment.start_time << ','
        << segment.end_time << ',' << segment.duration << ','
        << segment.amplitude_m << ',' << segment.gradient_objective_per_m << ','
        << segment.kkt_violation << ',' << segment.boundary << ','
        << segment.short_support_debug << ','
        << (cfg.nlos_mode == "automatic_discovery"
                ? cfg.discovery_short_min_count
                : cfg.oracle_short_min_count_debug)
        << ','
        << (cfg.nlos_mode == "automatic_discovery"
                ? cfg.discovery_short_min_duration_s
                : cfg.oracle_short_min_duration_debug)
        << ','
        << (cfg.nlos_mode == "automatic_discovery"
                ? uifgo::kT06AutomaticDiscoveryLabel
                : uifgo::kT04OracleDebugLabel)
        << '\n';
  }
}

std::vector<std::string> ParseCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quoted && c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
      field.push_back('"');
      ++i;
    } else if (c == '"') {
      quoted = !quoted;
    } else if (c == ',' && !quoted) {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(c);
    }
  }
  if (quoted) throw std::runtime_error("unterminated quoted CSV field");
  fields.push_back(field);
  return fields;
}

using CsvRow = std::map<std::string, std::string>;

std::vector<CsvRow> ReadCsv(const fs::path& path) {
  std::ifstream input(path.string());
  if (!input) throw std::runtime_error("cannot read " + path.string());
  std::string line;
  if (!std::getline(input, line)) return {};
  const auto header = ParseCsvLine(line);
  std::vector<CsvRow> rows;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto values = ParseCsvLine(line);
    if (values.size() != header.size())
      throw std::runtime_error("CSV column mismatch in " + path.string());
    CsvRow row;
    for (size_t i = 0; i < header.size(); ++i) row[header[i]] = values[i];
    rows.push_back(std::move(row));
  }
  return rows;
}

bool CsvBool(const std::string& value) {
  return value == "1" || value == "true" || value == "TRUE";
}

void WriteStage2Values(const fs::path& run_dir,
                       const gtsam::Values& values) {
  auto out = Open(run_dir / "stage2_values.csv");
  out << "key,type,v0,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11\n"
      << std::setprecision(17);
  for (gtsam::Key key : values.keys()) {
    const gtsam::Symbol symbol(key);
    out << gtsam::DefaultKeyFormatter(key) << ',';
    if (symbol.chr() == 'x') {
      const auto pose = values.at<gtsam::Pose3>(key);
      const auto R = pose.rotation().matrix();
      out << "POSE3";
      for (Eigen::Index row = 0; row < 3; ++row)
        for (Eigen::Index column = 0; column < 3; ++column)
          out << ',' << R(row, column);
      out << ',' << pose.x() << ',' << pose.y() << ',' << pose.z();
    } else if (symbol.chr() == 'v') {
      const auto value = values.at<gtsam::Vector3>(key);
      out << "VECTOR3," << value.x() << ',' << value.y() << ',' << value.z()
          << ",,,,,,,,,";
    } else if (symbol.chr() == 'b') {
      const auto value = values.at<gtsam::imuBias::ConstantBias>(key);
      const auto ba = value.accelerometer();
      const auto bg = value.gyroscope();
      out << "IMU_BIAS," << ba.x() << ',' << ba.y() << ',' << ba.z() << ','
          << bg.x() << ',' << bg.y() << ',' << bg.z() << ",,,,,,";
    } else if (symbol.chr() == 'c') {
      out << "DOUBLE," << values.at<double>(key) << ",,,,,,,,,,,";
    } else {
      throw std::runtime_error("unsupported Stage-2 cache Values key type");
    }
    out << '\n';
  }
}

gtsam::Values ReadStage2Values(const fs::path& path) {
  gtsam::Values values;
  for (const auto& row : ReadCsv(path)) {
    const std::string key_text = row.at("key");
    if (key_text.size() < 2)
      throw std::runtime_error("invalid Stage-2 cache key");
    const gtsam::Key key = gtsam::Symbol(
        key_text[0], static_cast<size_t>(std::stoull(key_text.substr(1))));
    const std::string& type = row.at("type");
    auto number = [&](const char* name) { return std::stod(row.at(name)); };
    if (type == "POSE3") {
      gtsam::Matrix3 R;
      size_t ordinal = 0;
      for (Eigen::Index r = 0; r < 3; ++r)
        for (Eigen::Index c = 0; c < 3; ++c)
          R(r, c) = number(("v" + std::to_string(ordinal++)).c_str());
      values.insert(key, gtsam::Pose3(
          gtsam::Rot3(R),
          gtsam::Point3(number("v9"), number("v10"), number("v11"))));
    } else if (type == "VECTOR3") {
      values.insert(key, gtsam::Vector3(number("v0"), number("v1"),
                                        number("v2")));
    } else if (type == "IMU_BIAS") {
      values.insert(key, gtsam::imuBias::ConstantBias(
          gtsam::Vector3(number("v0"), number("v1"), number("v2")),
          gtsam::Vector3(number("v3"), number("v4"), number("v5"))));
    } else if (type == "DOUBLE") {
      values.insert<double>(key, number("v0"));
    } else {
      throw std::runtime_error("unknown Stage-2 cache Values type");
    }
  }
  return values;
}

void WriteSupportPartition(const fs::path& path,
                           const uifgo::SupportPartition& partition) {
  auto out = Open(path);
  out << std::setprecision(17)
      << "{\n  \"schema\": \"uifgo_t09_support_partition_v1\",\n"
      << "  \"provider\": \"" << JsonEscape(partition.provider) << "\",\n"
      << "  \"partition_hash\": \"" << JsonEscape(partition.partition_hash)
      << "\",\n  \"input_plan_hash\": \""
      << JsonEscape(partition.input_plan_hash) << "\",\n  \"source_hash\": \""
      << JsonEscape(partition.source_hash) << "\",\n  \"config_hash\": \""
      << JsonEscape(partition.config_hash) << "\",\n  \"calibration_hash\": \""
      << JsonEscape(partition.calibration_hash)
      << "\",\n  \"solver_config_hash\": \""
      << JsonEscape(partition.solver_config_hash) << "\",\n  \"segments\": [";
  for (size_t i = 0; i < partition.segments.size(); ++i) {
    const auto& segment = partition.segments[i];
    if (i) out << ',';
    out << "\n    {\"segment_id\": \"" << JsonEscape(segment.segment_id)
        << "\", \"segment_ordinal\": " << segment.segment_ordinal
        << ", \"tag_id\": " << segment.tag_id
        << ", \"anchor_id\": " << segment.anchor_id
        << ", \"start_time\": " << segment.start_time
        << ", \"end_time\": " << segment.end_time
        << ", \"observation_count\": " << segment.observation_count
        << ", \"duration_s\": " << segment.duration
        << ", \"merge_snapshot_mean_m\": " << segment.merge_snapshot_mean_m
        << ", \"short\": " << (segment.short_support_debug ? "true" : "false")
        << ", \"obs_ids\": [";
    for (size_t j = 0; j < segment.obs_ids.size(); ++j) {
      if (j) out << ',';
      out << segment.obs_ids[j];
    }
    out << "]}";
  }
  out << "\n  ]\n}\n";
}

void WriteFdeArtifacts(const fs::path& run_dir,
                       const uifgo::FdeResult& result,
                       const uifgo::FdeOptions& options) {
  {
    auto out = Open(run_dir / "fde_status.json");
    out << std::setprecision(17)
        << "{\n  \"schema\": \"uifgo_fde_status_v1\",\n"
        << "  \"provider\": \"" << JsonEscape(result.provider) << "\",\n"
        << "  \"provider_version\": \""
        << JsonEscape(result.provider_version) << "\",\n"
        << "  \"status\": \"" << uifgo::FdeStatusName(result.status)
        << "\",\n  \"reason\": \"" << JsonEscape(result.reason) << "\",\n"
        << "  \"identity_hash\": \"" << JsonEscape(result.identity_hash)
        << "\",\n  \"reference_converged\": "
        << (result.reference.converged ? "true" : "false")
        << ",\n  \"reference_reason\": \""
        << JsonEscape(result.reference.reason) << "\",\n"
        << "  \"reference_iterations\": " << result.reference.iterations
        << ",\n  \"chi2_probability\": " << options.chi2_probability
        << ",\n  \"chi2_degrees_of_freedom\": "
        << options.chi2_degrees_of_freedom
        << ",\n  \"chi2_threshold\": "
        << JsonNumberOrNull(result.chi2_threshold)
        << ",\n  \"planned_count\": " << result.planned_count
        << ",\n  \"tested_count\": " << result.tested_count
        << ",\n  \"fault_count\": " << result.fault_count
        << ",\n  \"positive_candidate_count\": "
        << result.positive_candidate_count
        << ",\n  \"raw_run_count\": " << result.raw_run_count
        << ",\n  \"filtered_run_count\": " << result.filtered_run_count
        << ",\n  \"retained_segment_count\": "
        << result.retained_segment_count
        << ",\n  \"partition_hash\": \""
        << JsonEscape(result.partition.partition_hash)
        << "\",\n  \"gt_read\": false\n}\n";
  }
  {
    auto out = Open(run_dir / "fde_observations.csv");
    out << "obs_id,source_frame_index,source_message_index,source_range_index,"
           "source_observation_index,tag_id,anchor_id,sensor_time,valid,planned,"
           "keyframe_id,factor_index,ledger_nominal_sigma_m,tested,residual_m,"
           "factor_sigma_m,standardized_residual,statistic,fault_detected,"
           "positive_excess,nlos_candidate,raw_segment_id,segment_id,"
           "segment_ordinal,candidate_filter_reason\n"
        << std::setprecision(17);
    for (const auto& row : result.observations) {
      out << row.obs_id << ',' << row.source_frame_index << ','
          << row.source_message_index << ',' << row.source_range_index << ','
          << row.source_observation_index << ',' << row.tag_id << ','
          << row.anchor_id << ',' << JsonNumberOrNull(row.sensor_time) << ','
          << row.valid << ',' << row.planned << ',' << row.keyframe_id << ',';
      if (row.factor_index == std::numeric_limits<size_t>::max()) out << "";
      else out << row.factor_index;
      out << ',' << JsonNumberOrNull(row.ledger_nominal_sigma_m) << ','
          << row.tested << ',' << JsonNumberOrNull(row.residual_m) << ','
          << JsonNumberOrNull(row.factor_sigma_m) << ','
          << JsonNumberOrNull(row.standardized_residual) << ','
          << JsonNumberOrNull(row.statistic) << ',' << row.fault_detected << ','
          << row.positive_excess << ',' << row.nlos_candidate << ','
          << row.raw_segment_id << ',' << row.segment_id << ',';
      if (row.segment_ordinal == std::numeric_limits<size_t>::max()) out << "";
      else out << row.segment_ordinal;
      out << ',' << row.candidate_filter_reason << '\n';
    }
  }
  WriteSupportPartition(run_dir / "support_partition.json", result.partition);
  WriteSupportPartition(run_dir / "partition.json", result.partition);
}

uifgo::SupportPartition ReadSupportPartition(const fs::path& path) {
  const YAML::Node node = YAML::LoadFile(path.string());
  uifgo::SupportPartition output;
  output.schema = node["schema"].as<std::string>();
  output.provider = node["provider"].as<std::string>();
  output.partition_hash = node["partition_hash"].as<std::string>();
  if (node["input_plan_hash"]) output.input_plan_hash = node["input_plan_hash"].as<std::string>();
  if (node["source_hash"]) output.source_hash = node["source_hash"].as<std::string>();
  if (node["config_hash"]) output.config_hash = node["config_hash"].as<std::string>();
  if (node["calibration_hash"]) output.calibration_hash = node["calibration_hash"].as<std::string>();
  if (node["solver_config_hash"]) output.solver_config_hash = node["solver_config_hash"].as<std::string>();
  for (const auto& item : node["segments"]) {
    uifgo::SupportSegment segment;
    segment.segment_id = item["segment_id"].as<std::string>();
    segment.segment_ordinal = item["segment_ordinal"].as<size_t>();
    if (item["tag_id"]) {
      segment.tag_id = item["tag_id"].as<int>();
      segment.anchor_id = item["anchor_id"].as<int>();
    } else {
      const std::string link = item["link"].as<std::string>();
      const auto colon = link.find(':');
      segment.tag_id = std::stoi(link.substr(0, colon));
      segment.anchor_id = std::stoi(link.substr(colon + 1));
    }
    segment.start_time = item["start_time"].as<double>();
    segment.end_time = item["end_time"].as<double>();
    segment.observation_count = item["observation_count"].as<size_t>();
    segment.duration = item["duration_s"].as<double>();
    if (item["merge_snapshot_mean_m"])
      segment.merge_snapshot_mean_m = item["merge_snapshot_mean_m"].as<double>();
    segment.short_support_debug = item["short"].as<bool>();
    for (const auto& obs : item["obs_ids"])
      segment.obs_ids.push_back(obs.as<std::uint64_t>());
    output.segments.push_back(std::move(segment));
  }
  return output;
}

std::vector<size_t> ParseOrdinals(const std::string& text) {
  std::vector<size_t> output;
  std::istringstream input(text);
  std::string item;
  while (std::getline(input, item, ';'))
    if (!item.empty()) output.push_back(static_cast<size_t>(std::stoull(item)));
  return output;
}

std::vector<uifgo::GroupRecoverabilityScore> ReadCachedScores(
    const fs::path& root) {
  std::map<std::string, CsvRow> groups;
  for (const auto& row : ReadCsv(root / "groups.csv"))
    groups[row.at("group_id")] = row;
  std::map<std::string, std::vector<uifgo::SegmentFitScore>> fits;
  for (const auto& row : ReadCsv(root / "segment_fit_scores.csv")) {
    uifgo::SegmentFitScore fit;
    fit.segment_id = row.at("segment_id");
    fit.segment_ordinal = static_cast<size_t>(std::stoull(row.at("segment_ordinal")));
    fit.gamma = std::stod(row.at("gamma"));
    fit.short_support_debug = CsvBool(row.at("short_support_debug"));
    fit.boundary = CsvBool(row.at("boundary"));
    fits[row.at("group_id")].push_back(std::move(fit));
  }
  std::vector<uifgo::GroupRecoverabilityScore> output;
  for (const auto& row : ReadCsv(root / "scores_decision.csv")) {
    const auto found = groups.find(row.at("group_id"));
    if (found == groups.end()) throw std::runtime_error("cached score group missing");
    const auto& group = found->second;
    uifgo::GroupRecoverabilityScore score;
    score.group.group_id = row.at("group_id");
    score.group.start_time = std::stod(group.at("start_time"));
    score.group.end_time = std::stod(group.at("end_time"));
    score.group.segment_ordinals = ParseOrdinals(group.at("segment_ordinals"));
    score.eligible = CsvBool(row.at("eligible"));
    score.valid_score_exported = CsvBool(row.at("valid_score_exported"));
    score.status = row.at("status");
    score.linearization_id = row.at("linearization_id");
    score.segment_fit = fits[score.group.group_id];
    const std::string availability = row.at("score_availability");
    if (availability == "AVAILABLE_FINITE")
      score.numerical.status = uifgo::RecoverabilityStatus::OK;
    else if (availability == "AVAILABLE_RANK_DEFICIENT")
      score.numerical.status = uifgo::RecoverabilityStatus::RANK_DEFICIENT;
    else
      score.numerical.status = uifgo::RecoverabilityStatus::NUMERICAL_FAILURE;
    if (!row.at("eta").empty()) score.numerical.eta = std::stod(row.at("eta"));
    if (!row.at("s_m").empty()) score.numerical.s_m = std::stod(row.at("s_m"));
    score.numerical.s_is_infinite = CsvBool(row.at("s_is_infinite"));
    output.push_back(std::move(score));
  }
  return output;
}

void WriteSegmentResiduals(const fs::path& run_dir, const uifgo::Config& cfg,
                           const uifgo::PaperInputPlan& plan,
                           const uifgo::SupportPartition& support,
                           const uifgo::SegmentRefitResult& result) {
  std::map<int, gtsam::Point3> anchors;
  for (const auto& anchor : cfg.anchors) anchors[anchor.id] = anchor.pos;
  std::unordered_map<std::uint64_t, const uifgo::SupportSegment*> by_obs;
  for (const auto& segment : support.segments)
    for (std::uint64_t obs_id : segment.obs_ids) by_obs[obs_id] = &segment;
  auto out = Open(run_dir / "residuals.csv");
  out << "obs_id,keyframe_id,raw_z_m,geometric_range_m,fixed_beta_m,"
         "segment_id,segment_amplitude_m,residual_m,nominal_sigma_m\n";
  out << std::setprecision(17);
  for (const auto& record : plan.observations) {
    if (!record.valid || !record.planned) continue;
    const auto pose = result.values.at<gtsam::Pose3>(X(record.keyframe_id));
    const double geometric =
        (gtsam::Vector3(pose.transformFrom(cfg.lever_arm_init)) -
         gtsam::Vector3(anchors.at(record.anchor_id)))
            .norm();
    const double beta =
        uifgo::FixedBetaForLink(cfg, record.tag_id, record.anchor_id);
    double amplitude = 0.0;
    std::string segment_id;
    const auto found = by_obs.find(record.obs_id);
    if (found != by_obs.end()) {
      segment_id = found->second->segment_id;
      amplitude = result.values.at<double>(
          gtsam::symbol_shorthand::C(found->second->segment_ordinal));
    }
    const double residual = uifgo::SegmentUwbResidual(
        geometric, record.raw_range, beta, amplitude);
    out << record.obs_id << ',' << record.keyframe_id << ','
        << record.raw_range << ',' << geometric << ',' << beta << ','
        << CsvEscape(segment_id) << ',' << amplitude << ',' << residual << ','
        << record.nominal_sigma << '\n';
  }
}

void WriteDiscoveryArtifacts(const fs::path& run_dir,
                             const uifgo::DiscoveryResult& discovery) {
  {
    auto out = Open(run_dir / "support_snapshot.csv");
    out << "obs_id,tag_id,anchor_id,chain_id,active_run_id,sensor_time,"
           "nominal_weight,b_disc_m\n";
    out << std::setprecision(17);
    for (const auto& item : discovery.snapshot)
      out << item.obs_id << ',' << item.tag_id << ',' << item.anchor_id << ','
          << item.chain_id << ',' << item.active_run_id << ','
          << item.sensor_time << ',' << item.weight << ',' << item.bias_m
          << '\n';
  }
  {
    auto out = Open(run_dir / "discovery_iterations.csv");
    out << "outer_iteration,conditional_lm_iterations,objective_before,"
           "objective_after,allowed_objective_increase,"
           "relative_objective_change,max_navigation_scaled_step,"
           "max_bias_scaled_step,combined_scaled_step,active_bias_count,"
           "active_set_sha256,active_set_added_count,active_set_removed_count,"
           "active_set_symmetric_difference_count,max_chain_kkt_objective_per_m,"
           "max_chain_primal_residual_m,"
           "max_chain_dual_residual_objective_per_m,"
           "navigation_gradient_objective,"
           "navigation_roundoff_allowance_objective,objective_ok,step_ok,"
           "chain_optimality_ok,navigation_stationarity_ok,"
           "conditional_lm_inner_iterations,conditional_lm_lambda,"
           "conditional_lm_initial_error,conditional_lm_check_status,"
           "conditional_lm_check_evaluated,conditional_lm_previous_error,"
           "conditional_lm_current_error,conditional_lm_relative_tolerance,"
           "conditional_lm_absolute_tolerance,"
           "conditional_lm_optimizer_internal_relative_tolerance,"
           "conditional_lm_optimizer_internal_small_change_stop_enabled,"
           "conditional_lm_error_tolerance,"
           "conditional_lm_absolute_decrease,"
           "conditional_lm_relative_decrease,"
           "conditional_lm_relative_decrease_valid,"
           "conditional_lm_relative_tolerance_enabled,"
           "conditional_lm_decrease_predicates_reached_by_linked_check,"
           "conditional_lm_error_tolerance_triggered,"
           "conditional_lm_absolute_tolerance_triggered,"
           "conditional_lm_relative_tolerance_triggered,"
           "conditional_lm_check_result,"
           "conditional_lm_predicate_union_matches_check_result,"
           "conditional_lm_added_diagnostics_seconds,"
           "pre_chain_navigation_stationarity_valid,"
           "pre_chain_navigation_stationarity_status,"
           "pre_chain_max_pose_rotation_gradient_objective_per_rad,"
           "pre_chain_max_pose_translation_gradient_objective_per_m,"
           "pre_chain_max_velocity_gradient_objective_per_mps,"
           "pre_chain_max_accel_bias_gradient_objective_per_mps2,"
           "pre_chain_max_gyro_bias_gradient_objective_per_radps,"
           "pre_chain_max_scaled_gradient_objective,"
           "pre_chain_roundoff_allowance_objective,"
           "pre_chain_navigation_stationarity_ok,"
           "post_chain_navigation_stationarity_valid,"
           "post_chain_navigation_stationarity_status,"
           "post_chain_max_pose_rotation_gradient_objective_per_rad,"
           "post_chain_max_pose_translation_gradient_objective_per_m,"
           "post_chain_max_velocity_gradient_objective_per_mps,"
           "post_chain_max_accel_bias_gradient_objective_per_mps2,"
           "post_chain_max_gyro_bias_gradient_objective_per_radps,"
           "post_chain_max_scaled_gradient_objective,"
           "post_chain_roundoff_allowance_objective,"
           "post_chain_navigation_stationarity_ok,"
           "stationarity_audits_share_navigation_values,"
           "pre_chain_stationarity_seconds,post_chain_stationarity_seconds,"
           "added_diagnostics_seconds,conditional_lm_policy_version,"
           "conditional_lm_stationarity_qualification_enabled,"
           "conditional_lm_convergence_check_count,"
           "conditional_lm_generic_convergence_count,"
           "conditional_lm_generic_convergence_last_iteration,"
           "conditional_lm_qualification_evaluation_count,"
           "conditional_lm_qualification_last_evaluated,"
           "conditional_lm_qualification_last_passed,"
           "conditional_lm_qualification_status,"
           "conditional_lm_qualification_last_iteration,"
           "conditional_lm_qualification_seconds,"
           "conditional_lm_iterate_call_count,"
           "conditional_lm_lambda_trial_accounting_status,"
           "conditional_lm_lambda_trial_count,"
           "conditional_lm_rejected_lambda_trial_count,"
           "conditional_lm_accepted_update_count,"
           "conditional_lm_no_update_return_count,"
           "conditional_lm_qualification_audit_valid,"
           "conditional_lm_qualification_audit_status,"
           "conditional_lm_qualification_max_pose_rotation_gradient_objective_per_rad,"
           "conditional_lm_qualification_max_pose_translation_gradient_objective_per_m,"
           "conditional_lm_qualification_max_velocity_gradient_objective_per_mps,"
           "conditional_lm_qualification_max_accel_bias_gradient_objective_per_mps2,"
           "conditional_lm_qualification_max_gyro_bias_gradient_objective_per_radps,"
           "conditional_lm_qualification_max_scaled_gradient_objective,"
           "conditional_lm_qualification_roundoff_allowance_objective,"
           "conditional_lm_qualification_audit_stationary\n";
    out << std::setprecision(17);
    for (const auto& trace : discovery.iterations)
      out << trace.outer_iteration << ',' << trace.conditional_lm_iterations
          << ',' << trace.objective_before << ',' << trace.objective_after
          << ',' << trace.allowed_objective_increase << ','
          << trace.relative_objective_change << ','
          << trace.max_navigation_scaled_step << ','
          << trace.max_bias_scaled_step << ',' << trace.combined_scaled_step
          << ',' << trace.active_bias_count << ','
          << CsvEscape(trace.active_set_sha256) << ','
          << trace.active_set_added_count << ','
          << trace.active_set_removed_count << ','
          << trace.active_set_symmetric_difference_count << ','
          << trace.max_chain_kkt_objective_per_m << ','
          << trace.max_chain_primal_residual_m << ','
          << trace.max_chain_dual_residual_objective_per_m << ','
          << trace.navigation_gradient_objective << ','
          << trace.navigation_roundoff_allowance_objective << ','
          << trace.objective_ok << ',' << trace.step_ok << ','
          << trace.chain_optimality_ok << ','
          << trace.navigation_stationarity_ok << ','
          << trace.conditional_lm_inner_iterations << ','
          << CsvNumberOrEmpty(trace.conditional_lm_lambda) << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.initial_error) << ','
          << CsvEscape(trace.conditional_lm_convergence.check_status) << ','
          << trace.conditional_lm_convergence.check_evaluated << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.previous_error) << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.current_error) << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.relative_tolerance) << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.absolute_tolerance) << ','
          << CsvNumberOrEmpty(trace.conditional_lm_convergence
                                  .optimizer_internal_relative_tolerance)
          << ',' << trace.conditional_lm_convergence
                            .optimizer_internal_small_change_stop_enabled
          << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.error_tolerance) << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.absolute_decrease) << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.relative_decrease) << ','
          << trace.conditional_lm_convergence.relative_decrease_valid << ','
          << trace.conditional_lm_convergence.relative_tolerance_enabled << ','
          << trace.conditional_lm_convergence
                 .decrease_predicates_reached_by_linked_check << ','
          << trace.conditional_lm_convergence.error_tolerance_triggered << ','
          << trace.conditional_lm_convergence.absolute_tolerance_triggered << ','
          << trace.conditional_lm_convergence.relative_tolerance_triggered << ','
          << trace.conditional_lm_convergence.check_result << ','
          << trace.conditional_lm_convergence
                 .predicate_union_matches_check_result << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.added_diagnostics_seconds)
          << ',' << trace.pre_chain_navigation_stationarity.valid << ','
          << CsvEscape(trace.pre_chain_navigation_stationarity.reason) << ','
          << CsvNumberOrEmpty(trace.pre_chain_navigation_stationarity
                                  .max_pose_rotation_gradient_objective_per_rad)
          << ','
          << CsvNumberOrEmpty(trace.pre_chain_navigation_stationarity
                                  .max_pose_translation_gradient_objective_per_m)
          << ','
          << CsvNumberOrEmpty(trace.pre_chain_navigation_stationarity
                                  .max_velocity_gradient_objective_per_mps)
          << ','
          << CsvNumberOrEmpty(trace.pre_chain_navigation_stationarity
                                  .max_accel_bias_gradient_objective_per_mps2)
          << ','
          << CsvNumberOrEmpty(trace.pre_chain_navigation_stationarity
                                  .max_gyro_bias_gradient_objective_per_radps)
          << ','
          << CsvNumberOrEmpty(trace.pre_chain_navigation_stationarity
                                  .max_scaled_gradient_objective)
          << ','
          << CsvNumberOrEmpty(trace.pre_chain_navigation_stationarity
                                  .roundoff_allowance_objective)
          << ',' << trace.pre_chain_navigation_stationarity.stationary << ','
          << trace.post_chain_navigation_stationarity.valid << ','
          << CsvEscape(trace.post_chain_navigation_stationarity.reason) << ','
          << CsvNumberOrEmpty(trace.post_chain_navigation_stationarity
                                  .max_pose_rotation_gradient_objective_per_rad)
          << ','
          << CsvNumberOrEmpty(trace.post_chain_navigation_stationarity
                                  .max_pose_translation_gradient_objective_per_m)
          << ','
          << CsvNumberOrEmpty(trace.post_chain_navigation_stationarity
                                  .max_velocity_gradient_objective_per_mps)
          << ','
          << CsvNumberOrEmpty(trace.post_chain_navigation_stationarity
                                  .max_accel_bias_gradient_objective_per_mps2)
          << ','
          << CsvNumberOrEmpty(trace.post_chain_navigation_stationarity
                                  .max_gyro_bias_gradient_objective_per_radps)
          << ','
          << CsvNumberOrEmpty(trace.post_chain_navigation_stationarity
                                  .max_scaled_gradient_objective)
          << ','
          << CsvNumberOrEmpty(trace.post_chain_navigation_stationarity
                                  .roundoff_allowance_objective)
          << ',' << trace.post_chain_navigation_stationarity.stationary << ','
          << trace.stationarity_audits_share_navigation_values << ','
          << CsvNumberOrEmpty(trace.pre_chain_stationarity_seconds) << ','
          << CsvNumberOrEmpty(trace.post_chain_stationarity_seconds) << ','
          << CsvNumberOrEmpty(trace.added_diagnostics_seconds) << ','
          << CsvEscape(trace.conditional_lm_convergence.policy_version) << ','
          << trace.conditional_lm_convergence
                 .stationarity_qualification_enabled << ','
          << trace.conditional_lm_convergence.convergence_check_count << ','
          << trace.conditional_lm_convergence.generic_convergence_count << ','
          << trace.conditional_lm_convergence
                 .generic_convergence_last_iteration << ','
          << trace.conditional_lm_convergence
                 .qualification_evaluation_count << ','
          << trace.conditional_lm_convergence.qualification_last_evaluated
          << ','
          << trace.conditional_lm_convergence.qualification_last_passed << ','
          << CsvEscape(trace.conditional_lm_convergence.qualification_status)
          << ','
          << trace.conditional_lm_convergence.qualification_last_iteration
          << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_convergence.qualification_seconds)
          << ',' << trace.conditional_lm_convergence.iterate_call_count
          << ',' << CsvEscape(trace.conditional_lm_convergence
                                  .lambda_trial_accounting_status)
          << ',' << trace.conditional_lm_convergence.lambda_trial_count
          << ',' << trace.conditional_lm_convergence
                            .rejected_lambda_trial_count
          << ',' << trace.conditional_lm_convergence.accepted_update_count
          << ',' << trace.conditional_lm_convergence.no_update_return_count
          << ','
          << trace.conditional_lm_qualification_stationarity.valid << ','
          << CsvEscape(trace.conditional_lm_qualification_stationarity.reason)
          << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_qualification_stationarity
                     .max_pose_rotation_gradient_objective_per_rad)
          << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_qualification_stationarity
                     .max_pose_translation_gradient_objective_per_m)
          << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_qualification_stationarity
                     .max_velocity_gradient_objective_per_mps)
          << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_qualification_stationarity
                     .max_accel_bias_gradient_objective_per_mps2)
          << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_qualification_stationarity
                     .max_gyro_bias_gradient_objective_per_radps)
          << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_qualification_stationarity
                     .max_scaled_gradient_objective)
          << ','
          << CsvNumberOrEmpty(
                 trace.conditional_lm_qualification_stationarity
                     .roundoff_allowance_objective)
          << ','
          << trace.conditional_lm_qualification_stationarity.stationary
          << '\n';
  }
  {
    auto out = Open(run_dir / "admm_trace.csv");
    out << "chain_index,iteration,objective,primal_residual_m,"
           "primal_threshold_m,dual_residual_objective_per_m,"
           "dual_threshold_objective_per_m,"
           "max_kkt_violation_objective_per_m,"
           "max_tv_subgradient_violation_objective_per_m,rho_objective_per_m2\n";
    out << std::setprecision(17);
    for (size_t chain = 0; chain < discovery.chain_solutions.size(); ++chain)
      for (const auto& trace : discovery.chain_solutions[chain].trace)
        out << chain << ',' << trace.iteration << ',' << trace.objective << ','
            << trace.primal_residual_m << ',' << trace.primal_threshold_m << ','
            << trace.dual_residual_objective_per_m << ','
            << trace.dual_threshold_objective_per_m << ','
            << trace.max_kkt_violation_objective_per_m << ','
            << trace.max_tv_subgradient_violation_objective_per_m << ','
            << discovery.chain_solutions[chain].rho_objective_per_m2 << '\n';
  }
  {
    auto out = Open(run_dir / "partition.json");
    const auto& partition = discovery.partition;
    out << std::setprecision(17)
        << "{\n  \"schema\": \"" << JsonEscape(partition.schema)
        << "\",\n  \"provider\": \"" << JsonEscape(partition.provider)
        << "\",\n  \"hash_algorithm\": \"" << JsonEscape(partition.hash_algorithm)
        << "\",\n  \"partition_rule_version\": \""
        << JsonEscape(partition.partition_rule_version)
        << "\",\n  \"discovery_context_hash\": \""
        << JsonEscape(partition.discovery_context_hash)
        << "\",\n  \"input_plan_hash\": \""
        << JsonEscape(partition.input_plan_hash)
        << "\",\n  \"source_hash\": \"" << JsonEscape(partition.source_hash)
        << "\",\n  \"config_hash\": \"" << JsonEscape(partition.config_hash)
        << "\",\n  \"calibration_hash\": \""
        << JsonEscape(partition.calibration_hash)
        << "\",\n  \"solver_config_hash\": \""
        << JsonEscape(partition.solver_config_hash)
        << "\",\n  \"discovery_snapshot_hash\": \""
        << JsonEscape(partition.discovery_snapshot_hash)
        << "\",\n  \"partition_hash\": \""
        << JsonEscape(partition.partition_hash) << "\",\n"
        << "  \"stage2_partition_frozen\": true,\n  \"segments\": [";
    for (size_t i = 0; i < partition.segments.size(); ++i) {
      const auto& segment = partition.segments[i];
      if (i) out << ',';
      out << "\n    {\"segment_id\": \""
          << JsonEscape(segment.segment_id) << "\", \"segment_ordinal\": "
          << segment.segment_ordinal << ", \"link\": \""
          << segment.tag_id << ':' << segment.anchor_id
          << "\", \"start_time\": " << segment.start_time
          << ", \"end_time\": " << segment.end_time
          << ", \"observation_count\": " << segment.observation_count
          << ", \"duration_s\": " << segment.duration
          << ", \"merge_snapshot_mean_m\": "
          << segment.merge_snapshot_mean_m
          << ", \"short\": "
          << (segment.short_support_debug ? "true" : "false")
          << ", \"parent_segment_ids\": [";
      for (size_t j = 0; j < segment.parent_segment_ids.size(); ++j) {
        if (j) out << ',';
        out << '\"' << JsonEscape(segment.parent_segment_ids[j]) << '\"';
      }
      out << "], \"obs_ids\": [";
      for (size_t j = 0; j < segment.obs_ids.size(); ++j) {
        if (j) out << ',';
        out << segment.obs_ids[j];
      }
      out << "]}";
    }
    out << "\n  ]\n}\n";
  }
}

void WriteDiagnosticValues(const fs::path& path, const std::string& phase,
                           const gtsam::Values& values, bool append) {
  std::ofstream out;
  if (append)
    out.open(path.string(), std::ios::app);
  else
    out.open(path.string());
  if (!out) throw std::runtime_error("cannot write " + path.string());
  if (!append) out << "phase,key,key_value,value_type,coordinate,value\n";
  out << std::setprecision(17);
  for (gtsam::Key key : values.keys()) {
    const gtsam::Symbol symbol(key);
    const std::string formatted = gtsam::DefaultKeyFormatter(key);
    if (symbol.chr() == 'x' && values.exists<gtsam::Pose3>(key)) {
      const gtsam::Matrix4 matrix = values.at<gtsam::Pose3>(key).matrix();
      for (Eigen::Index row = 0; row < 4; ++row)
        for (Eigen::Index column = 0; column < 4; ++column)
          out << phase << ',' << formatted << ',' << key << ",Pose3,m"
              << row << column << ',' << matrix(row, column) << '\n';
    } else if (symbol.chr() == 'v' && values.exists<gtsam::Vector3>(key)) {
      const auto value = values.at<gtsam::Vector3>(key);
      for (Eigen::Index coordinate = 0; coordinate < value.size(); ++coordinate)
        out << phase << ',' << formatted << ',' << key << ",Vector3,v"
            << coordinate << ',' << value[coordinate] << '\n';
    } else if (symbol.chr() == 'b' &&
               values.exists<gtsam::imuBias::ConstantBias>(key)) {
      const auto bias = values.at<gtsam::imuBias::ConstantBias>(key);
      for (Eigen::Index coordinate = 0; coordinate < 3; ++coordinate)
        out << phase << ',' << formatted << ',' << key
            << ",ConstantBias,accel" << coordinate << ','
            << bias.accelerometer()[coordinate] << '\n';
      for (Eigen::Index coordinate = 0; coordinate < 3; ++coordinate)
        out << phase << ',' << formatted << ',' << key
            << ",ConstantBias,gyro" << coordinate << ','
            << bias.gyroscope()[coordinate] << '\n';
    } else {
      out << phase << ',' << formatted << ',' << key
          << ",UNSERIALIZED,NA,\n";
    }
  }
}

void WriteConditionalLmDiagnosticArtifacts(
    const fs::path& run_dir, const uifgo::CheckedLmDiagnosticCapture& capture) {
  if (!capture.requested) return;
  {
    auto out = Open(run_dir / "conditional_lm_stationarity.csv");
    out << "call,valid,stationary,rotation,translation,velocity,accel_bias,gyro_bias,max_scaled_gradient,roundoff\n" << std::setprecision(17);
    for (const auto& call : capture.calls) {
      const auto& a = call.stationarity_after;
      out << call.call_index << ',' << a.valid << ',' << a.stationary << ','
          << a.max_pose_rotation_gradient_objective_per_rad << ',' << a.max_pose_translation_gradient_objective_per_m << ','
          << a.max_velocity_gradient_objective_per_mps << ',' << a.max_accel_bias_gradient_objective_per_mps2 << ','
          << a.max_gyro_bias_gradient_objective_per_radps << ',' << a.max_scaled_gradient_objective << ',' << a.roundoff_allowance_objective << '\n';
    }
  }
  if (capture.first_block_budget_diagnostic) {
    auto status = Open(run_dir / "first_block_budget.json");
    status << "{\n  \"schema\": \"t10_a11_first_block_budget_result_v1\",\n  \"calls\": "
           << capture.calls.size() << ",\n  \"derivative_gate_passed\": "
           << (capture.derivative_gate_passed ? "true" : "false")
           << ",\n  \"continuation_status\": \""
           << JsonEscape(capture.continuation_status) << "\",\n  \"continuation_calls\": "
           << (capture.calls.size() > 50 ? capture.calls.size() - 50 : 0)
           << ",\n  \"stop_before_chain\": true\n}\n";
    auto trace = Open(run_dir / "first_block_stationarity.csv");
    trace << "call,valid,stationary,rotation,translation,velocity,accel_bias,gyro_bias,max_scaled_gradient,roundoff\n";
    trace << std::setprecision(17);
    for (const auto& call : capture.calls) {
      const auto& a = call.stationarity_after;
      trace << call.call_index << ',' << a.valid << ',' << a.stationary << ','
            << a.max_pose_rotation_gradient_objective_per_rad << ','
            << a.max_pose_translation_gradient_objective_per_m << ','
            << a.max_velocity_gradient_objective_per_mps << ','
            << a.max_accel_bias_gradient_objective_per_mps2 << ','
            << a.max_gyro_bias_gradient_objective_per_radps << ','
            << a.max_scaled_gradient_objective << ',' << a.roundoff_allowance_objective << '\n';
    }
    WriteDiagnosticValues(run_dir / "first_block_call50_values.csv", "CALL50", capture.values_at_call50, false);
    auto fdout = Open(run_dir / "first_block_call50_coordinate_fd.csv");
    fdout << "key,coordinate,unit,objective,analytic,step,plus,minus,central_fd,abs_difference,tolerance,agrees\n";
    fdout << std::setprecision(17);
    const auto& fd = capture.finite_difference_at_call50;
    for (const auto& point : fd.points)
      fdout << fd.key << ',' << fd.coordinate << ',' << fd.coordinate_unit << ','
            << fd.objective << ',' << fd.analytic_gradient << ',' << point.step << ','
            << point.objective_plus << ',' << point.objective_minus << ','
            << point.central_derivative << ',' << point.absolute_difference_from_analytic << ','
            << point.agreement_tolerance << ',' << point.agrees << '\n';
    auto accepted = Open(run_dir / "first_block_actual_accepted_delta.csv");
    accepted << "call,trial,parsed,selected,accepted_retract_matches,max_local_difference,key_count,dimension_count\n";
    accepted << std::setprecision(17);
    for (const auto& call : capture.calls) {
      if (call.call_index != 49 && call.call_index != 50) continue;
      for (const auto& trial : call.actual_trial_directions)
        accepted << call.call_index << ',' << trial.trial_index_within_call << ','
                 << trial.parsed << ',' << trial.selected_for_evaluation << ','
                 << trial.accepted_retract_matches << ','
                 << CsvNumberOrEmpty(trial.accepted_retract_max_difference) << ','
                 << trial.parsed_key_count << ',' << trial.parsed_dimension_count << '\n';
    }
  }
  {
    auto out = Open(run_dir / "conditional_lm_calls.csv");
    out << "call_index,optimizer_iterations_before,optimizer_iterations_after,"
           "inner_iterations_before,inner_iterations_after,lambda_before,"
           "lambda_after,error_before,error_after,accepted_values_delta_norm,"
           "accepted_values_max_abs_delta,accepted_state_update,"
           "rejected_lambda_trials_before_acceptance,observed_return_class,"
           "first_try_valid,first_try_reason,linear_system_solved,delta_norm,"
           "max_abs_delta,old_linearized_error,new_linearized_error,"
           "linearized_cost_change,linearized_resolution_threshold,"
           "linearized_step_valid,linearized_change_resolvable,tentative_error,"
           "tentative_cost_change,model_fidelity_valid,model_fidelity,"
           "minimum_model_fidelity,model_fidelity_passed,"
           "small_cost_change_threshold,small_cost_change,"
           "predicted_first_try_branch\n";
    out << std::setprecision(17);
    for (const auto& call : capture.calls) {
      const auto& first = call.first_try;
      out << call.call_index << ',' << call.optimizer_iterations_before << ','
          << call.optimizer_iterations_after << ','
          << call.inner_iterations_before << ',' << call.inner_iterations_after
          << ',' << CsvNumberOrEmpty(call.lambda_before) << ','
          << CsvNumberOrEmpty(call.lambda_after) << ','
          << CsvNumberOrEmpty(call.error_before) << ','
          << CsvNumberOrEmpty(call.error_after) << ','
          << CsvNumberOrEmpty(call.accepted_values_delta_norm) << ','
          << CsvNumberOrEmpty(call.accepted_values_max_abs_delta) << ','
          << call.accepted_state_update << ','
          << call.rejected_lambda_trials_before_acceptance << ','
          << CsvEscape(call.observed_return_class) << ',' << first.valid << ','
          << CsvEscape(first.reason) << ',' << first.linear_system_solved << ','
          << CsvNumberOrEmpty(first.delta_norm) << ','
          << CsvNumberOrEmpty(first.max_abs_delta) << ','
          << CsvNumberOrEmpty(first.old_linearized_error) << ','
          << CsvNumberOrEmpty(first.new_linearized_error) << ','
          << CsvNumberOrEmpty(first.linearized_cost_change) << ','
          << CsvNumberOrEmpty(first.linearized_resolution_threshold) << ','
          << first.linearized_step_valid << ','
          << first.linearized_change_resolvable << ','
          << CsvNumberOrEmpty(first.tentative_error) << ','
          << CsvNumberOrEmpty(first.tentative_cost_change) << ','
          << first.model_fidelity_valid << ','
          << CsvNumberOrEmpty(first.model_fidelity) << ','
          << CsvNumberOrEmpty(first.minimum_model_fidelity) << ','
          << first.model_fidelity_passed << ','
          << CsvNumberOrEmpty(first.small_cost_change_threshold) << ','
          << first.small_cost_change << ','
          << CsvEscape(first.predicted_first_try_branch) << '\n';
    }
  }
  {
    auto out = Open(run_dir / "conditional_lm_linked_trydelta.log");
    for (const auto& call : capture.calls) {
      out << "ACTUAL_LINKED_TRYDELTA_CALL_BEGIN call_index="
          << call.call_index << '\n'
          << call.linked_trydelta_stdout;
      if (!call.linked_trydelta_stdout.empty() &&
          call.linked_trydelta_stdout.back() != '\n')
        out << '\n';
      out << "ACTUAL_LINKED_TRYDELTA_CALL_END call_index="
          << call.call_index << '\n';
    }
  }
  {
    auto out = Open(run_dir / "conditional_lm_trial_deltas.csv");
    out << "call_index,trial_index_within_call,lambda,parsed,parse_reason,"
           "declared_key_count,parsed_key_count,parsed_dimension_count,"
           "linked_reported_delta_norm,parsed_delta_norm,key,key_value,"
           "coordinate,value\n";
    out << std::setprecision(17);
    for (const auto& call : capture.calls) {
      for (const auto& trial : call.actual_trial_directions) {
        if (trial.delta.size() == 0) {
          out << trial.call_index << ',' << trial.trial_index_within_call << ','
              << CsvNumberOrEmpty(trial.lambda) << ',' << trial.parsed << ','
              << CsvEscape(trial.parse_reason) << ','
              << trial.declared_key_count << ',' << trial.parsed_key_count
              << ',' << trial.parsed_dimension_count << ','
              << CsvNumberOrEmpty(trial.linked_reported_delta_norm) << ','
              << CsvNumberOrEmpty(trial.parsed_delta_norm) << ",,,,\n";
          continue;
        }
        for (const auto& key_vector : trial.delta) {
          for (Eigen::Index coordinate = 0;
               coordinate < key_vector.second.size(); ++coordinate) {
            out << trial.call_index << ','
                << trial.trial_index_within_call << ','
                << CsvNumberOrEmpty(trial.lambda) << ',' << trial.parsed << ','
                << CsvEscape(trial.parse_reason) << ','
                << trial.declared_key_count << ',' << trial.parsed_key_count
                << ',' << trial.parsed_dimension_count << ','
                << CsvNumberOrEmpty(trial.linked_reported_delta_norm) << ','
                << CsvNumberOrEmpty(trial.parsed_delta_norm) << ','
                << gtsam::DefaultKeyFormatter(key_vector.first) << ','
                << key_vector.first << ',' << coordinate << ','
                << key_vector.second[coordinate] << '\n';
          }
        }
      }
    }
  }
  {
    auto out = Open(run_dir / "conditional_lm_direction_summary.csv");
    out << "call_index,trial_index_within_call,lambda,parsed,parse_reason,"
           "selected_for_evaluation,evaluation_valid,evaluation_reason,"
           "base_graph_error,tentative_graph_error,direct_graph_decrease,"
           "factor_old_sum_double,factor_tentative_sum_double,"
           "factor_decrease_sum_double,factor_old_sum_long_double,"
           "factor_tentative_sum_long_double,"
           "factor_decrease_sum_long_double,old_linearized_error,"
           "new_linearized_error,predicted_decrease,gradient_dot_delta,"
           "gradient_dot_unit_direction\n";
    out << std::setprecision(17);
    for (const auto& call : capture.calls)
      for (const auto& trial : call.actual_trial_directions)
        out << trial.call_index << ',' << trial.trial_index_within_call << ','
            << CsvNumberOrEmpty(trial.lambda) << ',' << trial.parsed << ','
            << CsvEscape(trial.parse_reason) << ','
            << trial.selected_for_evaluation << ',' << trial.evaluation_valid
            << ',' << CsvEscape(trial.evaluation_reason) << ','
            << CsvNumberOrEmpty(trial.base_graph_error) << ','
            << CsvNumberOrEmpty(trial.tentative_graph_error) << ','
            << CsvNumberOrEmpty(trial.direct_graph_decrease) << ','
            << CsvNumberOrEmpty(trial.factor_old_sum_double) << ','
            << CsvNumberOrEmpty(trial.factor_tentative_sum_double) << ','
            << CsvNumberOrEmpty(trial.factor_decrease_sum_double) << ','
            << CsvLongDouble(trial.factor_old_sum_long_double) << ','
            << CsvLongDouble(trial.factor_tentative_sum_long_double) << ','
            << CsvLongDouble(trial.factor_decrease_sum_long_double) << ','
            << CsvNumberOrEmpty(trial.old_linearized_error) << ','
            << CsvNumberOrEmpty(trial.new_linearized_error) << ','
            << CsvNumberOrEmpty(trial.predicted_decrease) << ','
            << CsvNumberOrEmpty(trial.gradient_dot_delta) << ','
            << CsvNumberOrEmpty(trial.gradient_dot_unit_direction) << '\n';
  }
  {
    auto out = Open(run_dir / "conditional_lm_direction_finite_difference.csv");
    out << "call_index,trial_index_within_call,lambda,step,objective_plus,"
           "objective_minus,central_derivative,"
           "gradient_dot_unit_direction,absolute_difference_from_analytic,"
           "agreement_tolerance,agrees\n";
    out << std::setprecision(17);
    for (const auto& call : capture.calls)
      for (const auto& trial : call.actual_trial_directions)
        for (const auto& point : trial.directional_finite_difference)
          out << trial.call_index << ',' << trial.trial_index_within_call << ','
              << CsvNumberOrEmpty(trial.lambda) << ','
              << CsvNumberOrEmpty(point.step) << ','
              << CsvNumberOrEmpty(point.objective_plus) << ','
              << CsvNumberOrEmpty(point.objective_minus) << ','
              << CsvNumberOrEmpty(point.central_derivative) << ','
              << CsvNumberOrEmpty(trial.gradient_dot_unit_direction) << ','
              << CsvNumberOrEmpty(point.absolute_difference_from_analytic)
              << ',' << CsvNumberOrEmpty(point.agreement_tolerance) << ','
              << point.agrees << '\n';
  }
  {
    auto out = Open(run_dir / "conditional_lm_direction_factor_errors.csv");
    out << "call_index,trial_index_within_call,lambda,factor_index,"
           "dynamic_type,keys,error_at_base,error_at_tentative,decrease\n";
    out << std::setprecision(17);
    for (const auto& call : capture.calls) {
      for (const auto& trial : call.actual_trial_directions) {
        for (const auto& factor : trial.factors) {
          std::ostringstream keys;
          for (size_t index = 0; index < factor.keys.size(); ++index) {
            if (index) keys << ';';
            keys << gtsam::DefaultKeyFormatter(factor.keys[index]) << ':'
                 << factor.keys[index];
          }
          out << trial.call_index << ',' << trial.trial_index_within_call << ','
              << CsvNumberOrEmpty(trial.lambda) << ',' << factor.factor_index
              << ',' << CsvEscape(factor.dynamic_type) << ','
              << CsvEscape(keys.str()) << ','
              << CsvNumberOrEmpty(factor.error_at_base) << ','
              << CsvNumberOrEmpty(factor.error_at_tentative) << ','
              << CsvNumberOrEmpty(factor.decrease) << '\n';
        }
      }
    }
  }
  {
    auto out = Open(
        run_dir / "conditional_lm_direction_factor_derivatives.csv");
    out << "call_index,trial_index_within_call,lambda,factor_index,"
           "dynamic_type,keys,directional_derivative_valid,"
           "directional_derivative_reason,gradient_dot_unit_direction,step,"
           "objective_plus,objective_minus,central_derivative,"
           "absolute_difference_from_analytic,agreement_tolerance,agrees\n";
    out << std::setprecision(17);
    for (const auto& call : capture.calls) {
      for (const auto& trial : call.actual_trial_directions) {
        for (const auto& factor : trial.factors) {
          std::ostringstream keys;
          for (size_t index = 0; index < factor.keys.size(); ++index) {
            if (index) keys << ';';
            keys << gtsam::DefaultKeyFormatter(factor.keys[index]) << ':'
                 << factor.keys[index];
          }
          if (factor.directional_finite_difference.empty()) {
            out << trial.call_index << ','
                << trial.trial_index_within_call << ','
                << CsvNumberOrEmpty(trial.lambda) << ','
                << factor.factor_index << ','
                << CsvEscape(factor.dynamic_type) << ','
                << CsvEscape(keys.str()) << ','
                << factor.directional_derivative_valid << ','
                << CsvEscape(factor.directional_derivative_reason) << ','
                << CsvNumberOrEmpty(factor.gradient_dot_unit_direction)
                << ",,,,,,,\n";
          }
          for (const auto& point : factor.directional_finite_difference) {
            out << trial.call_index << ','
                << trial.trial_index_within_call << ','
                << CsvNumberOrEmpty(trial.lambda) << ','
                << factor.factor_index << ','
                << CsvEscape(factor.dynamic_type) << ','
                << CsvEscape(keys.str()) << ','
                << factor.directional_derivative_valid << ','
                << CsvEscape(factor.directional_derivative_reason) << ','
                << CsvNumberOrEmpty(factor.gradient_dot_unit_direction) << ','
                << CsvNumberOrEmpty(point.step) << ','
                << CsvNumberOrEmpty(point.objective_plus) << ','
                << CsvNumberOrEmpty(point.objective_minus) << ','
                << CsvNumberOrEmpty(point.central_derivative) << ','
                << CsvNumberOrEmpty(point.absolute_difference_from_analytic)
                << ',' << CsvNumberOrEmpty(point.agreement_tolerance) << ','
                << point.agrees << '\n';
          }
        }
      }
    }
  }
  {
    const fs::path base_values_path =
        run_dir / "conditional_lm_direction_base_values.csv";
    bool append = false;
    for (const auto& call : capture.calls) {
      WriteDiagnosticValues(base_values_path,
                            "CALL_" + std::to_string(call.call_index) +
                                "_BASE",
                            call.values_before, append);
      append = true;
    }
  }
  {
    auto out = Open(run_dir / "conditional_lm_finite_difference.csv");
    out << "key,key_value,coordinate,dimension,coordinate_unit,coordinate_scale,"
           "objective,analytic_gradient,scaled_analytic_gradient,step,"
           "objective_plus,objective_minus,objective_change_plus,"
           "objective_change_minus,central_derivative,"
           "absolute_difference_from_analytic,agreement_tolerance,agrees\n";
    out << std::setprecision(17);
    const auto& fd = capture.finite_difference;
    for (const auto& point : fd.points)
      out << fd.key << ',' << fd.key_value << ',' << fd.coordinate << ','
          << fd.dimension << ',' << fd.coordinate_unit << ','
          << CsvNumberOrEmpty(fd.coordinate_scale) << ','
          << CsvNumberOrEmpty(fd.objective) << ','
          << CsvNumberOrEmpty(fd.analytic_gradient) << ','
          << CsvNumberOrEmpty(fd.scaled_analytic_gradient) << ','
          << CsvNumberOrEmpty(point.step) << ','
          << CsvNumberOrEmpty(point.objective_plus) << ','
          << CsvNumberOrEmpty(point.objective_minus) << ','
          << CsvNumberOrEmpty(point.objective_change_plus) << ','
          << CsvNumberOrEmpty(point.objective_change_minus) << ','
          << CsvNumberOrEmpty(point.central_derivative) << ','
          << CsvNumberOrEmpty(point.absolute_difference_from_analytic) << ','
          << CsvNumberOrEmpty(point.agreement_tolerance) << ','
          << point.agrees << '\n';
  }
  {
    auto out = Open(run_dir / "conditional_lm_factor_audit.csv");
    out << "factor_index,dynamic_type,keys,error_at_capture_start,"
           "error_at_capture_final\n";
    out << std::setprecision(17);
    for (const auto& factor : capture.factors) {
      std::ostringstream keys;
      for (size_t i = 0; i < factor.keys.size(); ++i) {
        if (i) keys << ';';
        keys << gtsam::DefaultKeyFormatter(factor.keys[i]) << ':'
             << factor.keys[i];
      }
      out << factor.factor_index << ',' << CsvEscape(factor.dynamic_type) << ','
          << CsvEscape(keys.str()) << ','
          << CsvNumberOrEmpty(factor.error_at_capture_start) << ','
          << CsvNumberOrEmpty(factor.error_at_capture_final) << '\n';
    }
  }
  const fs::path values_path = run_dir / "conditional_lm_values.csv";
  WriteDiagnosticValues(values_path, "START", capture.values_at_start, false);
  WriteDiagnosticValues(values_path, "FINAL", capture.values_at_final, true);
  {
    size_t accepted = 0;
    size_t without_update_small = 0;
    size_t rejected_trials = 0;
    for (const auto& call : capture.calls) {
      accepted += call.accepted_state_update ? 1 : 0;
      without_update_small +=
          call.observed_return_class ==
                  "RETURN_WITHOUT_UPDATE_SMALL_COST_CHANGE"
              ? 1
              : 0;
      rejected_trials += call.rejected_lambda_trials_before_acceptance;
    }
    size_t agreeing_fd = 0;
    for (const auto& point : capture.finite_difference.points)
      agreeing_fd += point.agrees ? 1 : 0;
    size_t actual_direction_trial_count = 0;
    size_t complete_direction_trial_count = 0;
    size_t selected_direction_count = 0;
    size_t valid_direction_evaluation_count = 0;
    size_t factor_direction_count = 0;
    size_t factor_direction_fd_point_count = 0;
    size_t factor_direction_fd_agree_count = 0;
    for (const auto& call : capture.calls) {
      actual_direction_trial_count += call.actual_trial_directions.size();
      for (const auto& trial : call.actual_trial_directions) {
        complete_direction_trial_count += trial.parsed ? 1 : 0;
        selected_direction_count += trial.selected_for_evaluation ? 1 : 0;
        valid_direction_evaluation_count += trial.evaluation_valid ? 1 : 0;
        for (const auto& factor : trial.factors) {
          factor_direction_count +=
              factor.directional_derivative_valid ? 1 : 0;
          factor_direction_fd_point_count +=
              factor.directional_finite_difference.size();
          for (const auto& point : factor.directional_finite_difference)
            factor_direction_fd_agree_count += point.agrees ? 1 : 0;
        }
      }
    }
    auto out = Open(run_dir / "conditional_lm_diagnostic_summary.json");
    const auto& stationarity = capture.stationarity_at_final;
    out << "{\n  \"schema\": \"t10_conditional_lm_checkpoint_diagnostic_v4\",\n"
        << "  \"requested\": true,\n  \"valid\": "
        << (capture.valid ? "true" : "false") << ",\n  \"reason\": \""
        << JsonEscape(capture.reason) << "\",\n  \"graph_factor_count\": "
        << capture.graph_factor_count << ",\n  \"values_key_count\": "
        << capture.values_key_count << ",\n  \"iterate_call_count\": "
        << capture.calls.size() << ",\n  \"accepted_state_update_count\": "
        << accepted << ",\n  \"return_without_update_small_cost_count\": "
        << without_update_small << ",\n  \"rejected_lambda_trial_count\": "
        << rejected_trials << ",\n  \"actual_direction_trial_count\": "
        << actual_direction_trial_count
        << ",\n  \"complete_direction_trial_count\": "
        << complete_direction_trial_count
        << ",\n  \"selected_direction_count\": "
        << selected_direction_count
        << ",\n  \"valid_direction_evaluation_count\": "
        << valid_direction_evaluation_count
        << ",\n  \"valid_factor_direction_count\": "
        << factor_direction_count
        << ",\n  \"factor_direction_fd_point_count\": "
        << factor_direction_fd_point_count
        << ",\n  \"factor_direction_fd_agree_count\": "
        << factor_direction_fd_agree_count
        << ",\n  \"stationarity_at_final_valid\": "
        << (stationarity.valid ? "true" : "false")
        << ",\n  \"stationarity_at_final_reason\": \""
        << JsonEscape(stationarity.reason)
        << "\",\n  \"stationarity_at_final_passed\": "
        << (stationarity.stationary ? "true" : "false")
        << ",\n  \"stationarity_at_final_max_scaled_gradient\": "
        << JsonNumberOrNull(stationarity.max_scaled_gradient_objective)
        << ",\n  \"stationarity_at_final_roundoff_allowance\": "
        << JsonNumberOrNull(stationarity.roundoff_allowance_objective)
        << ",\n  \"finite_difference_valid\": "
        << (capture.finite_difference.valid ? "true" : "false")
        << ",\n  \"finite_difference_reason\": \""
        << JsonEscape(capture.finite_difference.reason)
        << "\",\n  \"finite_difference_agree_count\": " << agreeing_fd
        << ",\n  \"finite_difference_point_count\": "
        << capture.finite_difference.points.size() << "\n}\n";
  }
  const auto& recovery = capture.fixed_checkpoint_recovery;
  if (recovery.requested) {
    {
      auto out = Open(run_dir / "fixed_checkpoint_lm_recovery_calls.csv");
      out << "arm,call_index,optimizer_iterations_before,"
             "optimizer_iterations_after,inner_iterations_before,"
             "inner_iterations_after,lambda_before,lambda_after,error_before,"
             "error_after,values_delta_norm,values_max_abs_delta,"
             "accepted_state_update,accepted_strict_descent,"
             "rejected_lambda_trials_before_acceptance,observed_return_class,"
             "predicted_first_try_branch,first_try_delta_norm,"
             "first_try_tentative_error,first_try_tentative_cost_change,"
             "first_try_model_fidelity,first_try_model_fidelity_passed,"
             "first_try_small_cost_change,external_generic_convergence,"
             "stationarity_valid,max_scaled_gradient,roundoff_allowance,"
             "stationary,stationary_qualified\n";
      out << std::setprecision(17);
      auto write_arm = [&](const uifgo::FixedCheckpointLmArmDiagnostics& arm) {
        for (const auto& record : arm.calls) {
          const auto& call = record.linked;
          const auto& first = call.first_try;
          out << arm.arm << ',' << call.call_index << ','
              << call.optimizer_iterations_before << ','
              << call.optimizer_iterations_after << ','
              << call.inner_iterations_before << ','
              << call.inner_iterations_after << ','
              << CsvNumberOrEmpty(call.lambda_before) << ','
              << CsvNumberOrEmpty(call.lambda_after) << ','
              << CsvNumberOrEmpty(call.error_before) << ','
              << CsvNumberOrEmpty(call.error_after) << ','
              << CsvNumberOrEmpty(call.accepted_values_delta_norm) << ','
              << CsvNumberOrEmpty(call.accepted_values_max_abs_delta) << ','
              << call.accepted_state_update << ','
              << record.accepted_strict_descent << ','
              << call.rejected_lambda_trials_before_acceptance << ','
              << CsvEscape(call.observed_return_class) << ','
              << CsvEscape(first.predicted_first_try_branch) << ','
              << CsvNumberOrEmpty(first.delta_norm) << ','
              << CsvNumberOrEmpty(first.tentative_error) << ','
              << CsvNumberOrEmpty(first.tentative_cost_change) << ','
              << CsvNumberOrEmpty(first.model_fidelity) << ','
              << first.model_fidelity_passed << ','
              << first.small_cost_change << ','
              << record.external_generic_convergence << ','
              << record.stationarity.valid << ','
              << CsvNumberOrEmpty(
                     record.stationarity.max_scaled_gradient_objective)
              << ','
              << CsvNumberOrEmpty(
                     record.stationarity.roundoff_allowance_objective)
              << ',' << record.stationarity.stationary << ','
              << record.stationary_qualified << '\n';
        }
      };
      write_arm(recovery.current_behavior);
      write_arm(recovery.continue_lambda_search);
    }
    {
      auto out = Open(run_dir / "fixed_checkpoint_lm_recovery_summary.json");
      auto write_arm = [&](const char* key,
                           const uifgo::FixedCheckpointLmArmDiagnostics& arm,
                           bool trailing_comma) {
        out << "  \"" << key << "\": {\n"
            << "    \"arm\": \"" << JsonEscape(arm.arm) << "\",\n"
            << "    \"valid\": " << (arm.valid ? "true" : "false")
            << ",\n    \"reason\": \"" << JsonEscape(arm.reason) << "\",\n"
            << "    \"internal_relative_tolerance_changed\": "
            << (arm.internal_relative_tolerance_changed ? "true" : "false")
            << ",\n    \"internal_relative_tolerance\": "
            << JsonNumberOrNull(arm.internal_relative_tolerance)
            << ",\n    \"external_relative_tolerance\": "
            << JsonNumberOrNull(arm.external_relative_tolerance)
            << ",\n    \"external_absolute_tolerance\": "
            << JsonNumberOrNull(arm.external_absolute_tolerance)
            << ",\n    \"initial_error\": "
            << JsonNumberOrNull(arm.initial_error)
            << ",\n    \"final_error\": "
            << JsonNumberOrNull(arm.final_error)
            << ",\n    \"objective_decrease\": "
            << JsonNumberOrNull(arm.initial_error - arm.final_error)
            << ",\n    \"initial_lambda\": "
            << JsonNumberOrNull(arm.initial_lambda)
            << ",\n    \"final_lambda\": "
            << JsonNumberOrNull(arm.final_lambda)
            << ",\n    \"lambda_upper_bound\": "
            << JsonNumberOrNull(arm.lambda_upper_bound)
            << ",\n    \"elapsed_seconds\": "
            << JsonNumberOrNull(arm.elapsed_seconds)
            << ",\n    \"call_count\": " << arm.call_count
            << ",\n    \"lambda_trial_count\": " << arm.lambda_trial_count
            << ",\n    \"rejected_lambda_trial_count\": "
            << arm.rejected_lambda_trial_count
            << ",\n    \"accepted_update_count\": "
            << arm.accepted_update_count
            << ",\n    \"accepted_strict_descent_count\": "
            << arm.accepted_strict_descent_count
            << ",\n    \"timed_out\": "
            << (arm.timed_out ? "true" : "false")
            << ",\n    \"stationary_qualified\": "
            << (arm.stationary_qualified ? "true" : "false")
            << ",\n    \"final_stationarity_valid\": "
            << (arm.final_stationarity.valid ? "true" : "false")
            << ",\n    \"final_max_scaled_gradient\": "
            << JsonNumberOrNull(
                   arm.final_stationarity.max_scaled_gradient_objective)
            << ",\n    \"final_roundoff_allowance\": "
            << JsonNumberOrNull(
                   arm.final_stationarity.roundoff_allowance_objective)
            << "\n  }" << (trailing_comma ? "," : "") << "\n";
      };
      out << std::setprecision(17)
          << "{\n  \"schema\": \"t10_fixed_checkpoint_lm_recovery_v1\",\n"
          << "  \"scope\": \"DEVELOPMENT_FIXED_CHECKPOINT_LM_RECOVERY_DIAGNOSTIC\",\n"
          << "  \"shadow_only_not_estimator_output\": true,\n"
          << "  \"valid\": " << (recovery.valid ? "true" : "false")
          << ",\n  \"reason\": \"" << JsonEscape(recovery.reason) << "\",\n"
          << "  \"captured_error\": "
          << JsonNumberOrNull(recovery.captured_error)
          << ",\n  \"captured_lambda\": "
          << JsonNumberOrNull(recovery.captured_lambda)
          << ",\n  \"max_calls_per_arm\": "
          << recovery.max_calls_per_arm
          << ",\n  \"max_seconds_per_arm\": "
          << JsonNumberOrNull(recovery.max_seconds_per_arm) << ",\n";
      write_arm("arm_a_current_behavior", recovery.current_behavior, true);
      write_arm("arm_b_continue_lambda_search",
                recovery.continue_lambda_search, false);
      out << "}\n";
    }
    const fs::path values_path =
        run_dir / "fixed_checkpoint_lm_recovery_values.csv";
    WriteDiagnosticValues(values_path, "FIXED_CHECKPOINT",
                          capture.values_at_final, false);
    WriteDiagnosticValues(values_path, "ARM_A_FINAL",
                          recovery.current_behavior.values_at_final, true);
    WriteDiagnosticValues(values_path, "ARM_B_FINAL",
                          recovery.continue_lambda_search.values_at_final,
                          true);
  }
}

bool NavigationValuesFinite(const gtsam::Values& values, size_t keyframes) {
  for (size_t k = 0; k < keyframes; ++k) {
    if (!values.exists(X(k)) || !values.exists(B(k)) ||
        !values.exists(gtsam::symbol_shorthand::V(k))) {
      return false;
    }
    const auto pose = values.at<gtsam::Pose3>(X(k));
    const auto velocity =
        values.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(k));
    const auto bias = values.at<gtsam::imuBias::ConstantBias>(B(k));
    if (!pose.matrix().allFinite() || !velocity.allFinite() ||
        !bias.accelerometer().allFinite() || !bias.gyroscope().allFinite()) {
      return false;
    }
  }
  return true;
}

struct LmResult {
  gtsam::Values values;
  size_t iterations = 0;
  int inner_iterations = 0;
  double initial_error = 0.0;
  double final_error = 0.0;
  double final_lambda = 0.0;
  double last_abs_error_change = 0.0;
  double last_relative_error_change = 0.0;
  std::string termination;
};

class LmFailure : public std::runtime_error {
 public:
  LmFailure(const std::string& reason, const LmResult& result)
      : std::runtime_error(reason), result(result) {}

  LmResult result;
};

LmResult RunCheckedLm(const gtsam::NonlinearFactorGraph& graph,
                      const gtsam::Values& initial,
                      const gtsam::LevenbergMarquardtParams& params,
                      size_t keyframes) {
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  LmResult result;
  result.initial_error = optimizer.error();
  result.final_error = result.initial_error;
  if (!std::isfinite(result.initial_error) ||
      !NavigationValuesFinite(initial, keyframes)) {
    result.termination = "INITIAL_STATE_OR_OBJECTIVE_NONFINITE";
    throw LmFailure("LM_INITIAL_STATE_OR_OBJECTIVE_NONFINITE", result);
  }

  bool converged = false;
  double previous_error = result.initial_error;
  for (int attempt = 0; attempt < params.getMaxIterations(); ++attempt) {
    optimizer.iterate();
    result.iterations = optimizer.iterations();
    result.inner_iterations = optimizer.getInnerIterations();
    result.final_error = optimizer.error();
    result.final_lambda = optimizer.lambda();
    if (!std::isfinite(result.final_error) ||
        !std::isfinite(result.final_lambda) ||
        !NavigationValuesFinite(optimizer.values(), keyframes)) {
      result.termination = "RESULT_OR_OBJECTIVE_NONFINITE";
      throw LmFailure("LM_RESULT_OR_OBJECTIVE_NONFINITE", result);
    }
    result.last_abs_error_change =
        std::abs(previous_error - result.final_error);
    result.last_relative_error_change =
        result.last_abs_error_change /
        std::max(1.0, std::abs(previous_error));
    if (gtsam::checkConvergence(params, previous_error, result.final_error)) {
      converged = true;
      break;
    }
    previous_error = result.final_error;
  }

  if (result.iterations == 0) {
    result.termination = "NO_ITERATION_EXECUTED";
    throw LmFailure("LM_NO_ITERATION_EXECUTED", result);
  }
  if (!converged) {
    result.termination = "MAX_ITERATIONS_REACHED_WITHOUT_CONVERGENCE";
    throw LmFailure("LM_MAX_ITERATIONS_REACHED_WITHOUT_CONVERGENCE", result);
  }
  result.values = optimizer.values();
  result.termination = "CONVERGED_GTSAM_ERROR_CHECK";
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  fs::path run_dir;
  bool run_dir_created = false;
  bool oracle_debug_run = false;
  bool fixed_partition_debug_run = false;
  bool automatic_discovery_run = false;
  bool imu_aided_fde_run = false;
  bool segment_refit_estimate_exported = false;
  std::string failure_stage;
  double stage1_seconds = std::numeric_limits<double>::quiet_NaN();
  double stage2_seconds = std::numeric_limits<double>::quiet_NaN();
  double stage3_decision_score_seconds =
      std::numeric_limits<double>::quiet_NaN();
  double stage4_engine_seconds = std::numeric_limits<double>::quiet_NaN();
  double stage4_decision_seconds = std::numeric_limits<double>::quiet_NaN();
  double stage4_recovery_refit_seconds =
      std::numeric_limits<double>::quiet_NaN();
  double stage4_final_score_seconds =
      std::numeric_limits<double>::quiet_NaN();
  double stage4_fallback_seconds = std::numeric_limits<double>::quiet_NaN();
  double stage4_covariance_seconds = std::numeric_limits<double>::quiet_NaN();
  double inference_artifact_export_seconds =
      std::numeric_limits<double>::quiet_NaN();
  const auto started = std::chrono::steady_clock::now();
  try {
    const Args args = ParseArgs(argc, argv);
    const fs::path config_path = fs::absolute(args.config_path);
    if (!fs::exists(config_path))
      throw std::invalid_argument("config file does not exist");
    // Parse and validate configuration before creating any run directory or
    // loading estimator inputs. Invalid automatic/oracle configurations are
    // preflight failures, not failed estimation runs.
    uifgo::Config cfg = uifgo::ConfigLoader::Load(config_path.string());
    oracle_debug_run = cfg.nlos_mode == "oracle_debug";
    fixed_partition_debug_run = cfg.nlos_mode == "fixed_partition_debug";
    automatic_discovery_run = cfg.nlos_mode == "automatic_discovery";
    imu_aided_fde_run = cfg.nlos_mode == "imu_aided_fde";
    const bool estimated_support_run =
        automatic_discovery_run || imu_aided_fde_run;
    if (imu_aided_fde_run && args.method == "structured_bias_only")
      throw std::invalid_argument(
          "structured_bias_only is incompatible with imu_aided_fde");
    ValidateSupportedConfig(cfg);
    const fs::path output_root = fs::absolute(args.output_root);
    fs::create_directories(output_root);
    run_dir = output_root / args.run_id;
    if (fs::exists(run_dir))
      throw std::invalid_argument("run output already exists: " +
                                  run_dir.string());
    if (!fs::create_directory(run_dir))
      throw std::runtime_error("cannot create run directory");
    run_dir_created = true;

    fs::copy_file(config_path, run_dir / "config_original.yaml");

    std::vector<uifgo::ImuSample> imu;
    std::vector<uifgo::UwbFrame> raw_uwb;
    const LoadedInput loaded =
        LoadData(config_path.parent_path().string(), &cfg, &imu, &raw_uwb);
    if (imu.empty() || raw_uwb.empty() || cfg.anchors.empty())
      throw std::runtime_error("loader returned incomplete IMU/UWB/anchor data");

    if (!args.anchor_ids.empty()) {
      std::set<int> selected;
      std::istringstream ids(args.anchor_ids); std::string id;
      while(std::getline(ids,id,',')) selected.insert(std::stoi(id));
      for(int wanted:selected) {
        if(std::none_of(cfg.anchors.begin(),cfg.anchors.end(),[&](const auto& a){return a.id==wanted;}))
          throw std::invalid_argument("selected anchor absent from real input configuration");
      }
      cfg.anchors.erase(std::remove_if(cfg.anchors.begin(),cfg.anchors.end(),
          [&](const auto& a){return !selected.count(a.id);}),cfg.anchors.end());
      RebuildAnchorSigmas(&cfg);
    }
    const std::string& source = loaded.source;
    const std::string& source_hash = loaded.source_hash_fnv1a64;
    const std::string& source_sha256 = loaded.source_hash_sha256;
    const std::string config_sha256 =
        "sha256:" + uifgo::Sha256FileHex(config_path.string());
    const std::string& recording_id = loaded.recording_id;
    const auto plan = uifgo::BuildPaperInputPlan(raw_uwb, cfg, recording_id);
    if (plan.keyframes.size() < 2)
      throw std::runtime_error("paper input plan has fewer than two keyframes");
    const auto mask = uifgo::AllPlannedObservationMask(plan);
    const auto keyframes = uifgo::MaterializePaperKeyframes(plan, mask);
    const auto beta_validation = uifgo::ValidatePaperFixedBeta(plan, cfg);
    size_t valid = 0, planned = 0, suspected = 0;
    for (const auto& record : plan.observations) {
      valid += record.valid;
      planned += record.valid && record.planned;
      suspected += record.valid && record.planned && record.suspected_nlos;
    }
    WriteObservations(run_dir, plan, mask);

    uifgo::Initializer initializer(cfg);
    const auto init = initializer.Run(imu, keyframes);
    if (!init.ok) throw std::runtime_error("initializer failed");

    uifgo::GraphBuilder builder(cfg, cfg.paper_imu_covariance_model);
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    std::vector<size_t> uwb_indices;
    builder.Build(keyframes, imu, init, &graph, &initial, &uwb_indices);
    if (graph.empty() || uwb_indices.empty())
      throw std::runtime_error("graph contains no UWB factors");
    const std::size_t paper_pose_priors =
        uifgo::ReplacePosePriorsForPaperPath(&graph);
    if (paper_pose_priors != 1) {
      throw std::runtime_error(
          "paper path expected exactly one Pose3 prior, replaced " +
          std::to_string(paper_pose_priors));
    }

    // Audit actual PIM parameters, not merely a declared config field. All
    // discovery/refit/final rebuilds retain these base CombinedImuFactors.
    const std::string imu_model_identity = uifgo::ImuCovarianceModelIdentity(
        cfg, init.gravity_world, cfg.paper_imu_covariance_model);
    {
      auto out = Open(run_dir / "imu_covariance_model.txt");
      out << imu_model_identity << '\n';
      size_t count = 0;
      for (const auto& factor : graph) {
        const auto pim = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
        if (!pim) continue;
        const auto canonical = uifgo::ImuCovarianceParametersCanonical(
            pim->preintegratedMeasurements().p(), cfg.paper_imu_covariance_model);
        if ("imu-covariance-sha256:" + uifgo::Sha256Hex(canonical) != imu_model_identity)
          throw std::runtime_error("IMU_ACTUAL_PARAMETERS_MODEL_MISMATCH");
        out << "factor=" << count++ << '\n' << canonical;
      }
      if (count + 1 != keyframes.size())
        throw std::runtime_error("IMU_MODEL_FACTOR_COUNT_MISMATCH");
    }

    uifgo::InferenceIdentityContext common_identity_context;
    common_identity_context.input_sha256 = source_sha256;
    common_identity_context.config_sha256 = "t09-common-config:" +
        uifgo::Sha256Hex(plan.plan_sha256 + CalibrationContextHash(cfg, init.gravity_world));
    common_identity_context.input_plan_sha256 = plan.plan_sha256;
    common_identity_context.support_partition_sha256 =
        "t09-common-no-support:" + uifgo::Sha256Hex(plan.plan_sha256);
    common_identity_context.calibration_sha256 = CalibrationContextHash(cfg, init.gravity_world);
    common_identity_context.solver_config_sha256 =
        "t09-common-solver:" + uifgo::Sha256Hex(
            std::to_string(cfg.lm_max_iter) + ":" +
            std::to_string(cfg.lm_rel_tol) + ":" +
            std::to_string(cfg.lm_abs_tol) + ":" +
            uifgo::PaperPosePriorJacobianIdentity());
    const auto common_content_identity =
        uifgo::ComputeInferenceContentIdentity(
            graph, initial, common_identity_context);
    uifgo::CommonPreparationIdentityInput common_input;
    common_input.raw_source_sha256 = source_sha256;
    common_input.window_role_split_cutoff_sha256 =
        "sha256:" + uifgo::Sha256Hex(recording_id + ":development");
    common_input.observation_ledger_sha256 = plan.plan_sha256;
    common_input.input_plan_sha256 = plan.plan_sha256;
    common_input.nominal_sigma_sha256 =
        "sha256:" + uifgo::Sha256Hex("sigma:" + plan.plan_sha256);
    common_input.initialization_rule = "T02_SHARED_INITIALIZER_V1";
    common_input.initial_values_sha256 = common_content_identity.values_sha256;
    common_input.calibration_sha256 = CalibrationContextHash(cfg, init.gravity_world);
    common_input.physical_graph_sha256 =
        common_content_identity.graph_linearization_sha256;
    common_input.factor_metadata_sha256 =
        "sha256:" + uifgo::Sha256Hex("factor-meta:" + plan.plan_sha256);
    common_input.solver_precision_sha256 =
        common_identity_context.solver_config_sha256;
    const std::string common_preparation_id =
        uifgo::ComputeCommonPreparationId(common_input);
    uifgo::FdeContext fde_context;
    fde_context.input_plan_hash = plan.plan_sha256;
    fde_context.source_hash = source_sha256;
    // Bind only Stage1/common preparation semantics. Batch-generated final
    // configs legitimately differ in Stage4-only gate fields and file bytes.
    fde_context.config_hash = common_identity_context.config_sha256;
    fde_context.calibration_hash =
        CalibrationContextHash(cfg, init.gravity_world);
    fde_context.solver_config_hash = common_identity_context.solver_config_sha256;
    fde_context.common_preparation_id = common_preparation_id;
    fde_context.physical_graph_hash =
        common_content_identity.graph_linearization_sha256;
    fde_context.initial_values_hash = common_content_identity.values_sha256;
    {
      auto out = Open(run_dir / "common_preparation.json");
      out << "{\n  \"schema\": \"uifgo_t09_common_preparation_v1\",\n"
          << "  \"common_preparation_id\": \""
          << common_preparation_id << "\",\n"
          << "  \"graph_linearization_sha256\": \""
          << common_content_identity.graph_linearization_sha256 << "\",\n"
          << "  \"values_sha256\": \""
          << common_content_identity.values_sha256 << "\",\n"
          << "  \"context_sha256\": \""
          << common_content_identity.context_sha256 << "\"\n}\n";
    }

    if (args.diagnostic_passive_terminal_capture &&
        (common_content_identity.graph_linearization_sha256 !=
             "t08graphlin-sha256:99cdfbc7032cd3e2f6a2488b12d87b1219d75e60ba0c4f7ca80d56aff3c12710" ||
         common_content_identity.values_sha256 !=
             "t08values-sha256:7115e76d406123d3a30079dc115c6d3501ceca157d270eaced2d8be492b7cd3f" ||
         uifgo::Sha256FileHex(args.config_path) !=
             "479df3daae2096bb665582f60611c66841ed87edc496ddcbaf7cc9e180ed6c4f" ||
         args.execution_type == "FINAL_TRAJECTORY"))
      throw std::runtime_error("A15_INITIAL_IDENTITY_OR_CONFIG_GATE_FAILED");

    if (args.prepare_only) {
      if (!args.method.empty()) (void)uifgo::PaperMethodByName(args.method);
      auto out=Open(run_dir / "run_status.json");
      out << "{\"status\":\"PREPARED_ONLY\",\"optimizer_calls\":0,\"imu_count\":"
          << imu.size() << ",\"planned_observations\":" << planned
          << ",\"keyframes\":" << plan.keyframes.size()
          << ",\"method\":\"" << JsonEscape(args.method)
          << "\",\"anchors\":" << cfg.anchors.size()
          << ",\"source_sha256\":\"" << source_sha256 << "\"}\n";
      return 0;
    }
    // T09 Stage-4 cache replay is a separate entry point.  It performs common
    // preparation, restores typed Stage-2 Values, rebuilds the frozen Stage-2
    // graph for content verification, and then enters the unchanged T08 final
    // engine.  Neither Stage 1 nor the Stage-2 optimizer is reachable here.
    if (args.execution_type == "FINAL_TRAJECTORY") {
      failure_stage = "T09_CACHE_REPLAY";
      const fs::path cache_manifest_path =
          fs::canonical(fs::absolute(args.stage2_cache_manifest));
      const auto cache = uifgo::ReadStage2CacheManifest(
          cache_manifest_path.string());
      const fs::path cache_root = cache_manifest_path.parent_path();
      uifgo::RequireStage2CommonPreparation(cache, common_preparation_id);
      if (cache.source_identity != source_sha256)
        throw std::runtime_error("CACHE_SOURCE_IDENTITY_MISMATCH");
      if ((cache.cache_namespace == uifgo::Stage2CacheNamespace::AUTO_DISCOVERY &&
           cfg.nlos_mode != "automatic_discovery" &&
           cfg.nlos_mode != "imu_aided_fde") ||
          (cache.cache_namespace == uifgo::Stage2CacheNamespace::FIXED_PARTITION_DEBUG &&
           cfg.nlos_mode != "fixed_partition_debug"))
        throw std::runtime_error("CACHE_NAMESPACE_CONFIG_PATH_MISMATCH");
      const std::string expected_stage1_config =
          Stage1ProducerConfigHash(cfg, imu_aided_fde_run ? &fde_context
                                                         : nullptr);
      if (cache.stage1_config_sha256 != expected_stage1_config)
        throw std::runtime_error("CACHE_STAGE1_CONFIG_INCOMPATIBLE");
      if (cache.stage2_refit_config_sha256 != Stage2RefitConfigHash(cfg))
        throw std::runtime_error("CACHE_STAGE2_REFIT_CONFIG_INCOMPATIBLE");
      if (cache.stage3_score_config_sha256 != Stage3ScoreConfigHash())
        throw std::runtime_error("CACHE_STAGE3_SCORE_CONFIG_INCOMPATIBLE");

      const auto support = ReadSupportPartition(cache_root / "partition.json");
      if ((imu_aided_fde_run &&
           support.provider != uifgo::kImuAidedFdeProvider) ||
          (automatic_discovery_run &&
           support.provider != "automatic_discovery"))
        throw std::runtime_error("CACHE_STAGE1_PROVIDER_INCOMPATIBLE");
      if (!support.input_plan_hash.empty() &&
          support.input_plan_hash != plan.plan_sha256)
        throw std::runtime_error("CACHE_INPUT_PLAN_IDENTITY_MISMATCH");
      if (support.calibration_hash != CalibrationContextHash(cfg, init.gravity_world))
        throw std::runtime_error("CACHE_SUPPORT_IMU_CALIBRATION_INCOMPATIBLE");
      const gtsam::Values restored_values =
          ReadStage2Values(cache_root / "stage2_values.csv");
      std::set<size_t> all_segments;
      for (const auto& segment : support.segments)
        all_segments.insert(segment.segment_ordinal);

      uifgo::RefitOptions options;
      options.boundary_epsilon_m = cfg.refit_boundary_epsilon_m;
      options.relative_objective_tolerance = cfg.refit_relative_objective_tolerance;
      options.scaled_step_tolerance = cfg.refit_scaled_step_tolerance;
      options.projected_gradient_tolerance = cfg.refit_projected_gradient_tolerance;
      options.navigation_stationarity_tolerance_objective =
          cfg.refit_navigation_stationarity_tolerance_objective;
      options.gradient_roundoff_safety_factor = cfg.refit_gradient_roundoff_safety_factor;
      options.pose_rotation_scale_rad = cfg.refit_pose_rotation_scale_rad;
      options.pose_translation_scale_m = cfg.refit_pose_translation_scale_m;
      options.velocity_scale_mps = cfg.refit_velocity_scale_mps;
      options.accel_bias_scale_mps2 = cfg.refit_accel_bias_scale_mps2;
      options.gyro_bias_scale_radps = cfg.refit_gyro_bias_scale_radps;
      options.segment_amplitude_scale_m = cfg.refit_segment_amplitude_scale_m;
      options.max_refit_iterations = static_cast<size_t>(cfg.max_refit_iterations);
      options.lm_max_iterations = cfg.lm_max_iter;
      options.lm_relative_tolerance = cfg.lm_rel_tol;
      options.lm_absolute_tolerance = cfg.lm_abs_tol;
      const uifgo::SegmentRefitResult stage2 =
          uifgo::SegmentRefitter(options).RunFrozenCandidatePolicy(
              graph, restored_values, builder.factor_meta(), plan, cfg,
              support, all_segments, false);
      if (!stage2.converged())
        throw std::runtime_error("CACHE_STAGE2_REBUILD_FAILED: " + stage2.reason);
      uifgo::InferenceIdentityContext stage2_check_context;
      stage2_check_context.input_sha256 = source_sha256;
      stage2_check_context.config_sha256 = "CACHE_REPLAY_CONTENT_CHECK";
      stage2_check_context.input_plan_sha256 = plan.plan_sha256;
      stage2_check_context.support_partition_sha256 = support.partition_hash;
      stage2_check_context.calibration_sha256 = CalibrationContextHash(cfg, init.gravity_world);
      stage2_check_context.solver_config_sha256 = "CACHE_REPLAY_NO_SOLVER";
      const auto rebuilt_identity = uifgo::ComputeInferenceContentIdentity(
          stage2.graph, stage2.values, stage2_check_context);
      if (rebuilt_identity.graph_linearization_sha256 !=
              cache.stage2_graph_linearization_sha256 ||
          rebuilt_identity.values_sha256 != cache.stage2_values_sha256)
        throw std::runtime_error("CACHE_STAGE2_GRAPH_VALUES_IDENTITY_MISMATCH");
      // Fixed-offset policy needs the full physical R and amplitude column map.
      // Recompute at the verified immutable Stage2 graph/Values, with zero optimizer calls.
      const auto decision_scores = (!support.segments.empty() &&
                                    (args.method == "lcb_partial" ||
                                     args.method == "lcb_fixed_full"))
          ? uifgo::ScoreRefitRecoverability(stage2, support, plan, cfg)
          : ReadCachedScores(cache_root);

      uifgo::FinalGatePolicy final_policy = uifgo::FinalGatePolicy::FULL_GATE;
      if (args.method == "lcb_partial")
        final_policy = uifgo::FinalGatePolicy::LCB_PARTIAL;
      else if (args.method == "lcb_fixed_full")
        final_policy = uifgo::FinalGatePolicy::LCB_FIXED_FULL;
      else if (args.method == "suppress_all")
        final_policy = uifgo::FinalGatePolicy::SUPPRESS_ALL;
      else if (args.method == "structured_debias")
        final_policy = uifgo::FinalGatePolicy::STRUCTURED_DEBIAS;
      else if (args.method == "fit_only")
        final_policy = uifgo::FinalGatePolicy::FIT_ONLY;
      else if (args.method == "s_fit")
        final_policy = uifgo::FinalGatePolicy::S_FIT;
      else if (args.method == "eta_only") {
        if (std::getenv("UIFGO_T09_ETA_ONLY_SYNTHETIC_FINAL") == nullptr)
          throw std::invalid_argument("eta_only final requires synthetic-final provenance");
        final_policy = uifgo::FinalGatePolicy::ETA_ONLY;
      } else if (args.method != "full_gate") {
        throw std::invalid_argument("cache final mode is not implemented");
      }
      uifgo::GateThresholds gate;
      gate.tau_eta = cfg.gate_tau_eta;
      gate.tau_s_m = cfg.gate_tau_s_m;
      gate.tau_gamma = cfg.gate_tau_gamma;
      gate.parameter_provenance = "T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION";
      uifgo::InferenceIdentityContext final_context;
      final_context.input_sha256 = source_sha256;
      final_context.config_sha256 = config_sha256;
      final_context.input_plan_sha256 = plan.plan_sha256;
      final_context.support_partition_sha256 = support.partition_hash;
      final_context.calibration_sha256 = CalibrationContextHash(cfg, init.gravity_world);
      final_context.solver_config_sha256 = Stage2RefitConfigHash(cfg);

      uifgo::FinalRequestIdentityInput request_input;
      request_input.stage2_cache_id = cache.cache_id;
      request_input.canonical_mode = args.method;
      request_input.policy_version = std::string("T09_") +
          uifgo::FinalGatePolicyName(final_policy) + "_FINAL_AUDIT_V1";
      request_input.thresholds_sha256 = GateThresholdsHash(
          gate.tau_eta, gate.tau_s_m, gate.tau_gamma);
      request_input.threshold_provenance = gate.parameter_provenance;
      request_input.final_refit_score_config_sha256 =
          FinalRefitScoreConfigHash(cfg);
      request_input.solver_sha256 = Stage2RefitConfigHash(cfg);
      request_input.common_preparation_id = common_preparation_id;
      const std::string final_request_id =
          uifgo::ComputeFinalRequestId(request_input);

      const uifgo::InferenceResult inference =
          uifgo::FinalInferenceEngine(gate, options, {}, {}, final_policy).Run(
              graph, builder.factor_meta(), stage2, support, decision_scores,
              plan, cfg, final_context);
      stage4_engine_seconds = inference.timing.total_seconds;
      stage4_decision_seconds = inference.timing.decision_seconds;
      stage4_recovery_refit_seconds = inference.timing.recovery_refit_seconds;
      stage4_final_score_seconds = inference.timing.final_score_seconds;
      stage4_fallback_seconds = inference.timing.fallback_seconds;
      stage4_covariance_seconds = inference.timing.covariance_seconds;
      const auto artifact_started = std::chrono::steady_clock::now();
      const auto written = uifgo::WriteInferenceArtifacts(run_dir.string(), inference);
      const auto export_verification = uifgo::VerifyInferenceExport(
          run_dir.string(), inference, written.files);
      if (!export_verification.ok)
        throw std::runtime_error("CACHE_FINAL_EXPORT_VERIFICATION_FAILED: " +
                                 export_verification.reason);
      inference_artifact_export_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - artifact_started).count();
      const bool valid_final = inference.valid_estimate();
      {
        auto out = Open(run_dir / "cache_replay_status.json");
        out << "{\n  \"schema\": \"uifgo_t09_cache_replay_status_v1\",\n"
            << "  \"cache_id\": \"" << JsonEscape(cache.cache_id) << "\",\n"
            << "  \"stage1_execution\": \"NOT_RUN_CACHE_REPLAY\",\n"
            << "  \"stage2_optimizer_execution\": \"NOT_RUN_CACHE_REPLAY\",\n"
            << "  \"stage2_graph_values_verification\": \"VERIFIED\",\n"
            << "  \"decision_scores_source\": \"RESTORED_FROM_CACHE\"\n}\n";
      }
      {
        const double runner_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        auto out = Open(run_dir / "run_status.json");
        out << "{\n  \"status\": \"" << uifgo::InferenceStatusName(inference.status)
            << "\",\n  \"exit_code\": " << (valid_final ? 0 : 1)
            << ",\n  \"canonical_mode\": \"" << JsonEscape(args.method)
            << "\",\n  \"execution_type\": \"FINAL_TRAJECTORY\",\n"
            << "  \"operating_point_id\": \""
            << JsonEscape(args.operating_point_id) << "\",\n"
            << "  \"cache_id\": \"" << JsonEscape(cache.cache_id)
            << "\",\n  \"final_request_id\": \"" << JsonEscape(final_request_id)
            << "\",\n  \"inference_id\": \"" << JsonEscape(inference.inference_id)
            << "\",\n  \"valid_estimate_exported\": "
            << (valid_final ? "true" : "false")
            << ",\n  \"stage1_seconds\": null,\n  \"stage2_seconds\": null,\n"
            << "  \"stage1_execution\": \"NOT_RUN_CACHE_REPLAY\",\n"
            << "  \"stage2_execution\": \"NOT_RUN_CACHE_REPLAY\",\n"
            << "  \"elapsed_seconds_semantics\": \""
            << kRunnerElapsedSemantics << "\",\n"
            << "  \"elapsed_seconds\": " << JsonNumberOrNull(runner_elapsed)
            << ",\n  \"stage3_decision_score_seconds\": null,\n"
            << "  \"stage4_engine_seconds\": "
            << JsonNumberOrNull(stage4_engine_seconds)
            << ",\n  \"stage4_decision_seconds\": "
            << JsonNumberOrNull(stage4_decision_seconds)
            << ",\n  \"stage4_recovery_refit_seconds\": "
            << JsonNumberOrNull(stage4_recovery_refit_seconds)
            << ",\n  \"stage4_final_score_seconds\": "
            << JsonNumberOrNull(stage4_final_score_seconds)
            << ",\n  \"stage4_fallback_seconds\": "
            << JsonNumberOrNull(stage4_fallback_seconds)
            << ",\n  \"stage4_covariance_seconds\": "
            << JsonNumberOrNull(stage4_covariance_seconds)
            << ",\n  \"stage4_engine_semantics\": \"FROZEN_DECISION_THROUGH_FINAL_COVARIANCE_INCLUDING_RECOVERY_FINAL_SCORE_AND_ANY_FALLBACK\",\n"
            << "  \"inference_artifact_export_seconds\": "
            << JsonNumberOrNull(inference_artifact_export_seconds) << "\n}\n";
      }
      {
        auto out = Open(run_dir / "run_manifest.json");
        out << "{\n  \"schema\": \"uifgo_t09_run_manifest_v1\",\n"
            << "  \"run_id\": \"" << JsonEscape(args.run_id) << "\",\n"
            << "  \"canonical_mode\": \"" << JsonEscape(args.method) << "\",\n"
            << "  \"execution_type\": \"FINAL_TRAJECTORY\",\n"
            << "  \"operating_point_id\": \""
            << JsonEscape(args.operating_point_id) << "\",\n"
            << "  \"common_preparation_id\": \"" << common_preparation_id << "\",\n"
            << "  \"stage2_cache_id\": \"" << JsonEscape(cache.cache_id) << "\",\n"
            << "  \"final_request_id\": \"" << JsonEscape(final_request_id) << "\",\n"
            << "  \"policy_version\": \""
            << JsonEscape(request_input.policy_version) << "\",\n"
            << "  \"thresholds_sha256\": \""
            << JsonEscape(request_input.thresholds_sha256) << "\",\n"
            << "  \"threshold_provenance\": \""
            << JsonEscape(request_input.threshold_provenance) << "\",\n"
            << "  \"final_refit_score_config_sha256\": \""
            << JsonEscape(request_input.final_refit_score_config_sha256)
            << "\",\n  \"solver_sha256\": \""
            << JsonEscape(request_input.solver_sha256) << "\",\n"
            << "  \"inference_id\": \"" << JsonEscape(inference.inference_id) << "\",\n"
            << "  \"graph_linearization_sha256\": \""
            << JsonEscape(inference.content_identity.graph_linearization_sha256) << "\",\n"
            << "  \"values_sha256\": \""
            << JsonEscape(inference.content_identity.values_sha256) << "\",\n"
            << "  \"context_sha256\": \""
            << JsonEscape(inference.content_identity.context_sha256) << "\",\n"
            << "  \"export_verification_status\": \""
            << JsonEscape(export_verification.status) << "\",\n"
            << "  \"exported_file_count\": "
            << export_verification.checked_files << ",\n"
            << "  \"artifact_sha256\": [";
        for (size_t i = 0; i < export_verification.artifact_sha256.size(); ++i) {
          if (i) out << ',';
          out << "\n    \"" << JsonEscape(export_verification.artifact_sha256[i]) << "\"";
        }
        if (!export_verification.artifact_sha256.empty()) out << '\n';
        out << "  ]\n}\n";
      }
      if (!valid_final) {
        std::cerr << "paper runner ERROR: cache-replayed final "
                  << uifgo::InferenceStatusName(inference.status) << ": "
                  << inference.reason << '\n';
        return 1;
      }
      std::cout << "paper runner OK: cache-replayed Stage4 " << run_dir << '\n';
      return 0;
    }

    if (!args.method.empty()) {
      const auto& method = uifgo::PaperMethodByName(args.method);
      if (method.default_execution_type ==
          uifgo::PaperExecutionType::BASELINE_TRAJECTORY) {
        if (cfg.nlos_mode != "disabled")
          throw std::invalid_argument(
              "baseline method requires nlos.mode=disabled common input");
        uifgo::BaselineOptions baseline_options;
        baseline_options.method = method.method;
        baseline_options.parameter_provenance =
            "T09_DEVELOPMENT_ENGINEERING_TEST_ONLY";
        baseline_options.lm.max_iterations = cfg.lm_max_iter;
        baseline_options.lm.relative_tolerance = cfg.lm_rel_tol;
        baseline_options.lm.absolute_tolerance = cfg.lm_abs_tol;
        // T09 does not select scientific defaults. Batch-generated effective
        // configs must carry explicit TEST_ONLY values via environment for
        // these development methods.
        const char* robust_scale = std::getenv("UIFGO_T09_ROBUST_SCALE");
        const char* rejection =
            std::getenv("UIFGO_T09_REJECTION_THRESHOLD_SIGMA");
        if (robust_scale) baseline_options.robust_scale = std::stod(robust_scale);
        if (rejection)
          baseline_options.rejection_threshold_sigma = std::stod(rejection);
        if ((method.method == uifgo::PaperMethod::ROBUST_HUBER ||
             method.method == uifgo::PaperMethod::ROBUST_CAUCHY) &&
            !robust_scale)
          throw std::invalid_argument(
              "robust baseline requires explicit UIFGO_T09_ROBUST_SCALE");
        if (method.method == uifgo::PaperMethod::FIXED_REJECTION &&
            !rejection)
          throw std::invalid_argument(
              "fixed rejection requires explicit UIFGO_T09_REJECTION_THRESHOLD_SIGMA");
        failure_stage = "T09_BASELINE";
        const auto baseline = uifgo::RunPaperBaseline(
            graph, initial, builder.factor_meta(), baseline_options);
        if (baseline.valid) {
          WriteTrajectory(run_dir, keyframes, baseline.final_values);
          WriteBiases(run_dir, keyframes, baseline.final_values);
        }
        {
          auto audit = Open(run_dir / "baseline_factor_audit.csv");
          audit << "obs_id,final_use,reason\n";
          for (const auto id : baseline.kept_uwb_obs_ids)
            audit << id << ",1,RETAINED\n";
          for (const auto id : baseline.rejected_uwb_obs_ids)
            audit << id << ",0,FIXED_REJECTION_V1\n";
        }
        const auto baseline_identity = baseline.valid
            ? uifgo::ComputeInferenceContentIdentity(
                  baseline.final_graph, baseline.final_values,
                  common_identity_context)
            : uifgo::InferenceContentIdentity();
        {
          auto out = Open(run_dir / "run_manifest.json");
          out << "{\n  \"schema\": \"uifgo_t09_run_manifest_v1\",\n"
              << "  \"run_id\": \"" << JsonEscape(args.run_id) << "\",\n"
              << "  \"canonical_mode\": \"" << JsonEscape(args.method)
              << "\",\n  \"execution_type\": \"BASELINE_TRAJECTORY\",\n"
              << "  \"common_preparation_id\": \""
              << common_preparation_id << "\",\n"
              << "  \"final_request_id\": null,\n"
              << "  \"inference_id\": \"UNAVAILABLE:NOT_T08_FINAL_ENGINE\",\n"
              << "  \"graph_linearization_sha256\": \""
              << (baseline.valid
                      ? baseline_identity.graph_linearization_sha256
                      : "UNAVAILABLE:ESTIMATION_FAILURE")
              << "\",\n  \"values_sha256\": \""
              << (baseline.valid ? baseline_identity.values_sha256
                                 : "UNAVAILABLE:ESTIMATION_FAILURE")
              << "\",\n  \"context_sha256\": \""
              << (baseline.valid ? baseline_identity.context_sha256
                                 : "UNAVAILABLE:ESTIMATION_FAILURE")
              << "\",\n  \"export_verification_status\": \""
              << (baseline.valid ? "BASELINE_FACTOR_AUDIT_OK" : "FAILED")
              << "\"\n}\n";
        }
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
        {
          auto out = Open(run_dir / "run_status.json");
          out << "{\n  \"status\": \""
              << (baseline.valid ? "OK" : "ESTIMATION_FAILED")
              << "\",\n  \"exit_code\": " << (baseline.valid ? 0 : 1)
              << ",\n  \"canonical_mode\": \"" << JsonEscape(args.method)
              << "\",\n  \"reason\": \"" << JsonEscape(baseline.reason)
              << "\",\n  \"valid_estimate_exported\": "
              << (baseline.valid ? "true" : "false")
              << ",\n  \"elapsed_seconds\": " << JsonNumberOrNull(elapsed)
              << "\n}\n";
        }
        return baseline.valid ? 0 : 1;
      }
    }

    if (oracle_debug_run || fixed_partition_debug_run ||
        automatic_discovery_run || imu_aided_fde_run) {
      uifgo::SupportPartition support;
      fs::path support_path;
      uifgo::DiscoveryResult discovery;
      gtsam::Values refit_initial = initial;
      if (oracle_debug_run || fixed_partition_debug_run) {
        failure_stage = "ORACLE_SUPPORT_LOAD";
        support_path = fs::path(cfg.oracle_support_path);
        if (support_path.is_relative())
          support_path = config_path.parent_path() / support_path;
        support_path = fs::canonical(fs::absolute(support_path));
        const auto oracle = uifgo::OracleSupportProvider::Load(
            support_path.string(), plan,
            static_cast<size_t>(cfg.oracle_short_min_count_debug),
            cfg.oracle_short_min_duration_debug);
        support = uifgo::ToSupportPartition(oracle);
        support.provider = fixed_partition_debug_run
                               ? "fixed_partition_debug"
                               : "oracle_debug";
        support.partition_hash =
            "sha256:" + uifgo::Sha256FileHex(support_path.string());
        support.input_plan_hash = plan.plan_sha256;
        support.source_hash = source_sha256;
        support.config_hash = config_sha256;
        support.calibration_hash = CalibrationContextHash(cfg, init.gravity_world);
        support.solver_config_hash =
            "sha256:" + uifgo::Sha256Hex("t09-fixed-refit:" + config_sha256);
        fs::copy_file(support_path, run_dir / "oracle_support.yaml");
        failure_stage = "ORACLE_REFIT";
      } else if (automatic_discovery_run) {
        failure_stage = "AUTOMATIC_DISCOVERY";
        uifgo::DiscoveryOptions discovery_options;
        discovery_options.fused_lasso.lambda_l1 = cfg.discovery_lambda_l1;
        discovery_options.fused_lasso.lambda_tv = cfg.discovery_lambda_tv;
        discovery_options.fused_lasso.rho_scale = cfg.discovery_rho_scale;
        discovery_options.fused_lasso.primal_absolute_tolerance_m =
            cfg.discovery_primal_abs_tolerance_m;
        discovery_options.fused_lasso.primal_relative_tolerance =
            cfg.discovery_primal_rel_tolerance;
        discovery_options.fused_lasso
            .dual_absolute_tolerance_objective_per_m =
            cfg.discovery_dual_abs_tolerance_objective_per_m;
        discovery_options.fused_lasso.dual_relative_tolerance =
            cfg.discovery_dual_rel_tolerance;
        discovery_options.fused_lasso.kkt_tolerance_objective_per_m =
            cfg.discovery_kkt_tolerance_objective_per_m;
        discovery_options.fused_lasso
            .tv_subgradient_tolerance_objective_per_m =
            cfg.discovery_tv_subgradient_tolerance_objective_per_m;
        discovery_options.fused_lasso.max_iterations =
            static_cast<size_t>(cfg.discovery_admm_max_iterations);
        discovery_options.gap_threshold_s = cfg.discovery_gap_threshold_s;
        discovery_options.active_bias_min_m =
            cfg.discovery_active_bias_min_m;
        discovery_options.change_point_min_m =
            cfg.discovery_change_point_min_m;
        discovery_options.merge_max_difference_m =
            cfg.discovery_merge_max_difference_m;
        discovery_options.short_min_count =
            static_cast<size_t>(cfg.discovery_short_min_count);
        discovery_options.short_min_duration_s =
            cfg.discovery_short_min_duration_s;
        discovery_options.relative_objective_tolerance =
            cfg.refit_relative_objective_tolerance;
        discovery_options.scaled_step_tolerance =
            cfg.discovery_scaled_step_tolerance;
        discovery_options.observation_bias_scale_m =
            cfg.discovery_observation_bias_scale_m;
        discovery_options.navigation_stationarity_tolerance_objective =
            cfg.refit_navigation_stationarity_tolerance_objective;
        discovery_options.gradient_roundoff_safety_factor =
            cfg.refit_gradient_roundoff_safety_factor;
        discovery_options.navigation_scales.pose_rotation_rad =
            cfg.refit_pose_rotation_scale_rad;
        discovery_options.navigation_scales.pose_translation_m =
            cfg.refit_pose_translation_scale_m;
        discovery_options.navigation_scales.velocity_mps =
            cfg.refit_velocity_scale_mps;
        discovery_options.navigation_scales.accel_bias_mps2 =
            cfg.refit_accel_bias_scale_mps2;
        discovery_options.navigation_scales.gyro_bias_radps =
            cfg.refit_gyro_bias_scale_radps;
        discovery_options.max_outer_iterations =
            static_cast<size_t>(cfg.discovery_max_outer_iterations);
        discovery_options.conditional_lm.max_iterations = cfg.lm_max_iter;
        discovery_options.conditional_lm.relative_tolerance = cfg.lm_rel_tol;
        discovery_options.conditional_lm.absolute_tolerance = cfg.lm_abs_tol;
        discovery_options.conditional_lm.policy =
            ConditionalLmPolicyFromConfig(cfg);
        discovery_options.conditional_lm.navigation_scales =
            discovery_options.navigation_scales;
        discovery_options.conditional_lm
            .navigation_stationarity_tolerance_objective =
            discovery_options.navigation_stationarity_tolerance_objective;
        discovery_options.conditional_lm.gradient_roundoff_safety_factor =
            discovery_options.gradient_roundoff_safety_factor;
        const std::string calibration_sha256 = CalibrationContextHash(cfg, init.gravity_world);
        const std::string solver_config_sha256 =
            DiscoverySolverConfigHash(cfg);
        uifgo::DiscoveryContext discovery_context;
        discovery_context.input_plan_hash = plan.plan_sha256;
        discovery_context.source_hash = source_sha256;
        discovery_context.config_hash = config_sha256;
        discovery_context.calibration_hash = calibration_sha256;
        discovery_context.solver_config_hash = solver_config_sha256;
        uifgo::DiscoveryDiagnosticRequest discovery_diagnostic;
        const uifgo::DiscoveryDiagnosticRequest* discovery_diagnostic_ptr =
            nullptr;
        if (args.diagnostic_conditional_lm_outer > 0) {
          discovery_diagnostic.conditional_lm_outer_iteration =
              args.diagnostic_conditional_lm_outer;
          discovery_diagnostic.conditional_lm.emit_linked_gtsam_trylambda =
              true;
          discovery_diagnostic.conditional_lm.capture_linked_gtsam_trydelta =
              args.diagnostic_exact_lm_direction || args.diagnostic_first_block_budget || args.diagnostic_passive_terminal_capture;
          discovery_diagnostic.conditional_lm.first_block_budget_diagnostic =
              args.diagnostic_first_block_budget;
          if (args.diagnostic_exact_lm_direction) {
            discovery_diagnostic.conditional_lm
                .direction_evaluation_lambdas =
                args.diagnostic_conditional_lm_outer == 123
                    ? std::vector<double>{1e-6, 1.0, 1e4}
                    : std::vector<double>{1e-5, 1.0, 1e4};
            discovery_diagnostic.conditional_lm
                .direction_finite_difference_steps = {1e-4, 1e-5, 1e-6};
          }
          discovery_diagnostic.conditional_lm.finite_difference_steps = {
              1e-2, 3e-3, 1e-3, 3e-4, 1e-4, 3e-5, 1e-5,
              3e-6, 1e-6, 3e-7, 1e-7, 3e-8, 1e-8};
          discovery_diagnostic.conditional_lm
              .run_fixed_checkpoint_lm_recovery =
              args.diagnostic_fixed_checkpoint_lm_recovery;
          discovery_diagnostic.conditional_lm.fixed_checkpoint_max_calls = 50;
          discovery_diagnostic.conditional_lm.fixed_checkpoint_max_seconds =
              10.0;
          if (args.diagnostic_first_block_budget) {
            discovery_diagnostic.conditional_lm.direction_finite_difference_steps =
                {1e-4, 1e-5, 1e-6};
            discovery_diagnostic.conditional_lm.finite_difference_steps =
                {1e-4, 1e-5, 1e-6};
          }
          if (args.diagnostic_passive_terminal_capture) {
            discovery_diagnostic.conditional_lm.passive_terminal_capture = true;
            discovery_diagnostic.conditional_lm.finite_difference_steps.clear();
          }
          discovery_diagnostic_ptr = &discovery_diagnostic;
          auto out = Open(run_dir / "diagnostic_request.json");
          if (args.diagnostic_passive_terminal_capture) {
            out << R"A15({"schema":"t10_a15_passive_terminal_capture_v1","default_off":true,
"outer":1,"stop_before_chain":true,"additional_solve_count":0,"inline_fd_count":0,
"budget_extension":false,"actual_trydelta":true,"excluded_from_config_and_solver_identity":true})A15";
          } else if (args.diagnostic_first_block_budget) {
            out << R"A11({
  "schema": "t10_a11_first_block_budget_v1",
  "role": "development",
  "default_off": true,
  "outer": 1,
  "original_calls": 50,
  "conditional_total_cap": 200,
  "accepted_direction_calls": [49,50],
  "direction_source": "ACTUAL_AUTHORITATIVE_LINKED_TRYDELTA_LAST_ACCEPTED_TRIAL",
  "direction_normalization": "u=delta/||delta||2",
  "steps": [0.0001,0.00001,0.000001],
  "agreement_rule": "abs(fd-analytic)<=5e-9+5e-3*abs(analytic)",
  "continuation_rule": "ALL_GRAPH_FACTOR_AND_CALL50_COORDINATE_POINTS_PASS_COMPLETE_RETRACT_MATCH",
  "same_optimizer_lambda_and_values": true,
  "additional_shadow_solve_count": 0,
  "stop_before_chain": true,
  "production_cache_publish": false
}
)A11";
          } else {
          out << "{\n  \"schema\": \""
              << (args.diagnostic_exact_lm_direction
                      ? "t10_exact_lm_direction_request_v1"
                      : "t10_conditional_lm_checkpoint_request_v2")
              << "\",\n"
              << "  \"development_observer_only\": true,\n"
              << "  \"excluded_from_config_and_solver_identity\": true,\n"
              << "  \"conditional_lm_outer_iteration\": "
              << args.diagnostic_conditional_lm_outer << ",\n"
              << "  \"emit_linked_gtsam_trylambda\": true,\n"
              << "  \"capture_actual_linked_gtsam_trydelta\": "
              << (args.diagnostic_exact_lm_direction ? "true" : "false")
              << ",\n"
              << "  \"trydelta_is_authoritative_iterate_output\": true,\n"
              << "  \"post_capture_linearization_count_per_call\": "
              << (args.diagnostic_exact_lm_direction ? "1" : "0")
              << ",\n"
              << "  \"post_capture_additional_solve_count\": 0,\n"
              << "  \"existing_inspect_first_linked_lm_try_contains_diagnostic_solve\": true,\n"
              << "  \"linked_trylambda_numeric_precision\": "
                 "\"binary64_max_digits10\",\n"
              << "  \"linked_call_boundary_markers\": true,\n"
              << "  \"fixed_checkpoint_lm_recovery\": "
              << (args.diagnostic_fixed_checkpoint_lm_recovery
                      ? "true" : "false")
              << ",\n  \"fixed_checkpoint_max_calls_per_arm\": 50,\n"
              << "  \"fixed_checkpoint_max_seconds_per_arm\": 10,\n"
              << "  \"direction_selected_lambdas\": "
              << (args.diagnostic_exact_lm_direction
                      ? (args.diagnostic_conditional_lm_outer == 123
                             ? "[0.000001,1,10000]"
                             : "[0.00001,1,10000]")
                      : "[]")
              << ",\n"
              << "  \"direction_normalization\": \"u=delta/||delta||2\",\n"
              << "  \"direction_finite_difference_steps\": "
              << (args.diagnostic_exact_lm_direction
                      ? "[0.0001,0.00001,0.000001]"
                      : "[]")
              << ",\n"
              << "  \"finite_difference_steps_native_units\": "
                 "[0.01,0.003,0.001,0.0003,0.0001,0.00003,0.00001,"
                 "0.000003,0.000001,0.0000003,0.0000001,0.00000003,"
                 "0.00000001],\n"
              << "  \"agreement_rule\": "
                 "\"abs(fd-analytic) <= 5e-9 + 5e-3*abs(analytic)\"\n}\n";
          }
        }
        // These artifacts are deliberately committed before Stage 1 starts,
        // so every solver failure keeps its complete effective inputs.
        {
          auto out = Open(run_dir / "input_manifest.json");
          out << "{\n  \"schema\": \"paper_input_v2\",\n"
              << "  \"source\": \"" << JsonEscape(source) << "\",\n"
              << "  \"recording_id\": \"" << recording_id << "\",\n"
              << "  \"source_hash_sha256\": \"" << source_sha256 << "\",\n"
              << "  \"input_interface\": \"" << cfg.data_interface << "\",\n"
              << "  \"cache_id\": "
              << (loaded.from_t07_cache
                      ? "\"" + JsonEscape(loaded.cache_id) + "\""
                      : "null")
              << ",\n  \"cache_unit_conversion_applied\": false,\n"
              << "  \"recording_time_origin_s\": "
              << JsonNumberOrNull(loaded.recording_time_origin_s) << ",\n"
              << "  \"config_hash_sha256\": \"" << config_sha256 << "\",\n"
              << "  \"input_plan_hash_sha256\": \"" << plan.plan_sha256
              << "\",\n  \"calibration_hash_sha256\": \""
              << calibration_sha256
              << "\",\n  \"solver_config_hash_sha256\": \""
              << solver_config_sha256 << "\",\n"
              << "  \"conditional_navigation_policy\": \""
              << JsonEscape(cfg.discovery_conditional_navigation_policy)
              << "\",\n"
              << "  \"conditional_navigation_qualification_enabled\": "
              << (ConditionalNavigationQualificationEnabled(cfg)
                      ? "true"
                      : "false")
              << ",\n"
              << "  \"fixed_beta_status\": \"" << beta_validation.status
              << "\",\n  \"raw_observations\": " << plan.observations.size()
              << ",\n  \"valid_observations\": " << valid
              << ",\n  \"planned_observations\": " << planned
              << ",\n  \"suspected_nlos_planned\": " << suspected
              << ",\n  \"keyframes\": " << plan.keyframes.size()
              << ",\n  \"strategy\": \""
              << (cfg.final_inference_enabled
                      ? "t08_frozen_group_final_inference"
                      : "automatic_discovery_refit_score")
              << "\",\n"
              << "  \"debug_label\": \""
              << (cfg.final_inference_enabled
                      ? uifgo::kT08DevelopmentGateLabel
                      : uifgo::kT06AutomaticDiscoveryLabel)
              << "\",\n"
              << "  \"gt_read\": false,\n  \"oracle_support_read\": false,\n"
              << "  \"gt_or_oracle_read\": false,\n"
              << "  \"manifest_written_before_stage1\": true\n}\n";
        }
        {
          auto out = Open(run_dir / "config_effective.yaml");
          out << std::setprecision(17)
              << "paper_path: true\nstrategy: "
              << (cfg.final_inference_enabled
                      ? "t08_frozen_group_final_inference\n"
                      : "automatic_discovery_refit_score\n")
              << "debug_label: "
              << (cfg.final_inference_enabled
                      ? uifgo::kT08DevelopmentGateLabel
                      : uifgo::kT06AutomaticDiscoveryLabel)
              << '\n'
              << "parameter_provenance: PENDING_VALIDATION_DEVELOPMENT_ONLY\n"
              << "lambda_l1: " << cfg.discovery_lambda_l1
              << "\nlambda_tv: " << cfg.discovery_lambda_tv
              << "\ngap_threshold_s: " << cfg.discovery_gap_threshold_s
              << "\nactive_bias_min_m: " << cfg.discovery_active_bias_min_m
              << "\nchange_point_min_m: " << cfg.discovery_change_point_min_m
              << "\nmerge_max_difference_m: "
              << cfg.discovery_merge_max_difference_m
              << "\ndiscovery_short_min_count: "
              << cfg.discovery_short_min_count
              << "\ndiscovery_short_min_duration_s: "
              << cfg.discovery_short_min_duration_s
              << "\ndiscovery_relative_objective_tolerance: "
              << cfg.refit_relative_objective_tolerance
              << "\ndiscovery_scaled_step_tolerance: "
              << cfg.discovery_scaled_step_tolerance
              << "\ndiscovery_observation_bias_scale_m: "
              << cfg.discovery_observation_bias_scale_m
              << "\ndiscovery_navigation_stationarity_tolerance_objective: "
              << cfg.refit_navigation_stationarity_tolerance_objective
              << "\nrho_scale: " << cfg.discovery_rho_scale
              << "\nadmm_primal_abs_tolerance_m: "
              << cfg.discovery_primal_abs_tolerance_m
              << "\nadmm_primal_rel_tolerance: "
              << cfg.discovery_primal_rel_tolerance
              << "\nadmm_dual_abs_tolerance_objective_per_m: "
              << cfg.discovery_dual_abs_tolerance_objective_per_m
              << "\nadmm_dual_rel_tolerance: "
              << cfg.discovery_dual_rel_tolerance
              << "\nadmm_kkt_tolerance_objective_per_m: "
              << cfg.discovery_kkt_tolerance_objective_per_m
              << "\nadmm_tv_subgradient_tolerance_objective_per_m: "
              << cfg.discovery_tv_subgradient_tolerance_objective_per_m
              << "\nadmm_max_iterations: "
              << cfg.discovery_admm_max_iterations
              << "\ndiscovery_max_outer_iterations: "
              << cfg.discovery_max_outer_iterations
              << "\ndiscovery_conditional_navigation_policy: "
              << cfg.discovery_conditional_navigation_policy
              << "\ndiscovery_conditional_navigation_qualification_enabled: "
              << (ConditionalNavigationQualificationEnabled(cfg)
                      ? "true"
                      : "false")
              << "\ndiscovery_conditional_lm_optimizer_internal_relative_tolerance: "
              << ConditionalNavigationInternalRelativeTolerance(cfg)
              << "\ndiscovery_conditional_lm_external_relative_tolerance: "
              << cfg.lm_rel_tol
              << "\ndiscovery_conditional_lm_external_absolute_tolerance: "
              << cfg.lm_abs_tol
              << "\ninput_plan_hash_sha256: " << plan.plan_sha256
              << "\nsource_hash_sha256: " << source_sha256
              << "\nconfig_hash_sha256: " << config_sha256
              << "\ncalibration_hash_sha256: " << calibration_sha256
              << "\nsolver_config_hash_sha256: " << solver_config_sha256
              << "\nfinal_inference_enabled: "
              << (cfg.final_inference_enabled ? "true" : "false")
              << "\ngate_parameter_provenance: "
              << (cfg.final_inference_enabled
                      ? cfg.gate_parameter_provenance
                      : "NOT_APPLICABLE")
              << "\ntau_eta: "
              << (cfg.final_inference_enabled
                      ? JsonNumberOrNull(cfg.gate_tau_eta)
                      : "null")
              << "\ntau_s_m: "
              << (cfg.final_inference_enabled
                      ? JsonNumberOrNull(cfg.gate_tau_s_m)
                      : "null")
              << "\ntau_gamma: "
              << (cfg.final_inference_enabled
                      ? JsonNumberOrNull(cfg.gate_tau_gamma)
                      : "null")
              << '\n';
        }
        {
          auto out = Open(run_dir / "capability_status.json");
          out << "{\n  \"debug_label\": \""
              << (cfg.final_inference_enabled
                      ? uifgo::kT08DevelopmentGateLabel
                      : uifgo::kT06AutomaticDiscoveryLabel)
              << "\",\n"
              << "  \"discovery\": \"STAGE1_RUNNING\",\n"
              << "  \"segment_refit\": \"NOT_RUN\",\n"
              << "  \"recoverability_score\": \"NOT_RUN\",\n"
              << "  \"gate\": \""
              << (cfg.final_inference_enabled ? "NOT_RUN_STAGE1_PENDING"
                                              : "NOT_IMPLEMENTED")
              << "\",\n"
              << "  \"fallback\": \""
              << (cfg.final_inference_enabled ? "NOT_RUN_STAGE1_PENDING"
                                              : "NOT_IMPLEMENTED")
              << "\",\n"
              << "  \"covariance\": \"NOT_COMPUTED\",\n"
              << "  \"output_semantics\": \"NO_VALID_ESTIMATE_STAGE1_PENDING\"\n}\n";
        }
        const auto stage1_started = std::chrono::steady_clock::now();
        discovery = uifgo::AutomaticSupportProvider(discovery_options).Run(
            graph, initial, builder.factor_meta(), plan, cfg,
            discovery_context, discovery_diagnostic_ptr);
        stage1_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - stage1_started).count();
        WriteDiscoveryArtifacts(run_dir, discovery);
        WriteConditionalLmDiagnosticArtifacts(
            run_dir, discovery.last_conditional_lm.diagnostic);
        if (!discovery.converged()) {
          {
            auto capability = Open(run_dir / "capability_status.json");
            capability << "{\n  \"debug_label\": \""
                       << (cfg.final_inference_enabled
                               ? uifgo::kT08DevelopmentGateLabel
                               : uifgo::kT06AutomaticDiscoveryLabel)
                       << "\",\n"
                       << "  \"discovery\": \"FAILED_"
                       << uifgo::DiscoveryStatusName(discovery.status)
                       << "\",\n  \"segment_refit\": \"NOT_RUN\",\n"
                       << "  \"recoverability_score\": \"NOT_RUN\",\n"
                       << "  \"gate\": \""
                       << (cfg.final_inference_enabled
                               ? "NOT_RUN_STAGE1_FAILED"
                               : "NOT_IMPLEMENTED")
                       << "\",\n"
                       << "  \"fallback\": \""
                       << (cfg.final_inference_enabled
                               ? "NOT_RUN_STAGE1_FAILED"
                               : "NOT_IMPLEMENTED")
                       << "\",\n"
                       << "  \"covariance\": \"NOT_COMPUTED\",\n"
                       << "  \"output_semantics\": \"NO_VALID_ESTIMATE_STAGE1_FAILED\"\n}\n";
          }
          {
            auto diagnostics = Open(run_dir / "discovery_failure_diagnostics.json");
            diagnostics << "{\n  \"status\": \""
                        << uifgo::DiscoveryStatusName(discovery.status)
                        << "\",\n  \"reason\": \""
                        << JsonEscape(discovery.reason) << "\",\n"
                        << "  \"outer_trace_rows\": "
                        << discovery.iterations.size() << ",\n"
                        << "  \"chain_results_retained\": "
                        << discovery.chain_solutions.size() << ",\n"
                        << "  \"input_plan_hash_sha256\": \""
                        << plan.plan_sha256 << "\",\n"
                        << "  \"solver_config_hash_sha256\": \""
                        << solver_config_sha256 << "\",\n"
                        << "  \"diagnostic_schema\": "
                           "\"t10_stage1_diagnostics_v2\",\n"
                        << "  \"added_diagnostics_seconds_total\": "
                        << JsonNumberOrNull(
                               discovery.added_diagnostics_seconds_total)
                        << ",\n  \"last_conditional_lm\": ";
            if (!discovery.conditional_lm_attempted) {
              diagnostics << "null\n}\n";
            } else {
              const auto& lm = discovery.last_conditional_lm;
              const auto& check = lm.convergence;
              diagnostics
                  << "{\n    \"outer_iteration\": "
                  << discovery.conditional_lm_attempted_outer_iteration
                  << ",\n    \"converged\": "
                  << (lm.converged ? "true" : "false")
                  << ",\n    \"reason\": \"" << JsonEscape(lm.reason)
                  << "\",\n    \"iterations\": " << lm.iterations
                  << ",\n    \"inner_iterations\": " << lm.inner_iterations
                  << ",\n    \"lambda\": " << JsonNumberOrNull(lm.lambda)
                  << ",\n    \"initial_error_valid\": "
                  << (check.initial_error_valid ? "true" : "false")
                  << ",\n    \"initial_error\": "
                  << JsonNumberOrNull(check.initial_error)
                  << ",\n    \"check_evaluated\": "
                  << (check.check_evaluated ? "true" : "false")
                  << ",\n    \"check_status\": \""
                  << JsonEscape(check.check_status)
                  << "\",\n    \"previous_error\": "
                  << JsonNumberOrNull(check.previous_error)
                  << ",\n    \"current_error\": "
                  << JsonNumberOrNull(check.current_error)
                  << ",\n    \"relative_tolerance\": "
                  << JsonNumberOrNull(check.relative_tolerance)
                  << ",\n    \"absolute_tolerance\": "
                  << JsonNumberOrNull(check.absolute_tolerance)
                  << ",\n    \"optimizer_internal_relative_tolerance\": "
                  << JsonNumberOrNull(
                         check.optimizer_internal_relative_tolerance)
                  << ",\n    \"optimizer_internal_small_change_stop_enabled\": "
                  << (check.optimizer_internal_small_change_stop_enabled
                          ? "true" : "false")
                  << ",\n    \"error_tolerance\": "
                  << JsonNumberOrNull(check.error_tolerance)
                  << ",\n    \"absolute_decrease\": "
                  << JsonNumberOrNull(check.absolute_decrease)
                  << ",\n    \"relative_decrease\": "
                  << JsonNumberOrNull(check.relative_decrease)
                  << ",\n    \"relative_decrease_valid\": "
                  << (check.relative_decrease_valid ? "true" : "false")
                  << ",\n    \"relative_tolerance_enabled\": "
                  << (check.relative_tolerance_enabled ? "true" : "false")
                  << ",\n    \"decrease_predicates_reached_by_linked_check\": "
                  << (check.decrease_predicates_reached_by_linked_check
                          ? "true" : "false")
                  << ",\n    \"error_tolerance_triggered\": "
                  << (check.error_tolerance_triggered ? "true" : "false")
                  << ",\n    \"absolute_tolerance_triggered\": "
                  << (check.absolute_tolerance_triggered ? "true" : "false")
                  << ",\n    \"relative_tolerance_triggered\": "
                  << (check.relative_tolerance_triggered ? "true" : "false")
                  << ",\n    \"check_result\": "
                  << (check.check_result ? "true" : "false")
                  << ",\n    \"predicate_union_matches_check_result\": "
                  << (check.predicate_union_matches_check_result ? "true"
                                                                 : "false")
                  << ",\n    \"added_diagnostics_seconds\": "
                  << JsonNumberOrNull(check.added_diagnostics_seconds)
                  << ",\n    \"policy_version\": \""
                  << JsonEscape(check.policy_version)
                  << "\",\n    \"stationarity_qualification_enabled\": "
                  << (check.stationarity_qualification_enabled ? "true"
                                                               : "false")
                  << ",\n    \"convergence_check_count\": "
                  << check.convergence_check_count
                  << ",\n    \"generic_convergence_count\": "
                  << check.generic_convergence_count
                  << ",\n    \"generic_convergence_last_iteration\": "
                  << check.generic_convergence_last_iteration
                  << ",\n    \"qualification_evaluation_count\": "
                  << check.qualification_evaluation_count
                  << ",\n    \"qualification_last_evaluated\": "
                  << (check.qualification_last_evaluated ? "true" : "false")
                  << ",\n    \"qualification_last_passed\": "
                  << (check.qualification_last_passed ? "true" : "false")
                  << ",\n    \"qualification_status\": \""
                  << JsonEscape(check.qualification_status)
                  << "\",\n    \"qualification_last_iteration\": "
                  << check.qualification_last_iteration
                  << ",\n    \"qualification_seconds\": "
                  << JsonNumberOrNull(check.qualification_seconds)
                  << ",\n    \"iterate_call_count\": "
                  << check.iterate_call_count
                  << ",\n    \"lambda_trial_accounting_status\": \""
                  << JsonEscape(check.lambda_trial_accounting_status)
                  << "\""
                  << ",\n    \"lambda_trial_count\": "
                  << check.lambda_trial_count
                  << ",\n    \"rejected_lambda_trial_count\": "
                  << check.rejected_lambda_trial_count
                  << ",\n    \"accepted_update_count\": "
                  << check.accepted_update_count
                  << ",\n    \"no_update_return_count\": "
                  << check.no_update_return_count
                  << ",\n    \"last_qualification_stationarity\": {\n"
                  << "      \"valid\": "
                  << (lm.last_qualification_stationarity.valid ? "true"
                                                               : "false")
                  << ",\n      \"status\": \""
                  << JsonEscape(lm.last_qualification_stationarity.reason)
                  << "\",\n      \"max_pose_rotation_gradient_objective_per_rad\": "
                  << JsonNumberOrNull(
                         lm.last_qualification_stationarity
                             .max_pose_rotation_gradient_objective_per_rad)
                  << ",\n      \"max_pose_translation_gradient_objective_per_m\": "
                  << JsonNumberOrNull(
                         lm.last_qualification_stationarity
                             .max_pose_translation_gradient_objective_per_m)
                  << ",\n      \"max_velocity_gradient_objective_per_mps\": "
                  << JsonNumberOrNull(
                         lm.last_qualification_stationarity
                             .max_velocity_gradient_objective_per_mps)
                  << ",\n      \"max_accel_bias_gradient_objective_per_mps2\": "
                  << JsonNumberOrNull(
                         lm.last_qualification_stationarity
                             .max_accel_bias_gradient_objective_per_mps2)
                  << ",\n      \"max_gyro_bias_gradient_objective_per_radps\": "
                  << JsonNumberOrNull(
                         lm.last_qualification_stationarity
                             .max_gyro_bias_gradient_objective_per_radps)
                  << ",\n      \"max_scaled_gradient_objective\": "
                  << JsonNumberOrNull(
                         lm.last_qualification_stationarity
                             .max_scaled_gradient_objective)
                  << ",\n      \"roundoff_allowance_objective\": "
                  << JsonNumberOrNull(
                         lm.last_qualification_stationarity
                             .roundoff_allowance_objective)
                  << ",\n      \"stationary\": "
                  << (lm.last_qualification_stationarity.stationary ? "true"
                                                                    : "false")
                  << "\n    }\n  }\n}\n";
            }
          }
          auto out = Open(run_dir / "run_status.json");
          const double runner_elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - started).count();
          out << "{\n  \"status\": \"FAILED\",\n  \"exit_code\": 1,\n"
              << "  \"debug_label\": \""
              << uifgo::kT06AutomaticDiscoveryLabel << "\",\n"
              << "  \"failure_stage\": \"AUTOMATIC_DISCOVERY\",\n"
              << "  \"discovery_status\": \""
              << uifgo::DiscoveryStatusName(discovery.status) << "\",\n"
              << "  \"reason\": \"" << JsonEscape(discovery.reason)
              << "\",\n  \"stage2_refit_run\": false,\n"
              << "  \"gate_or_fallback_run\": false,\n"
              << "  \"elapsed_seconds_semantics\": \""
              << kRunnerElapsedSemantics << "\",\n"
              << "  \"elapsed_seconds\": "
              << JsonNumberOrNull(runner_elapsed) << ",\n"
              << "  \"stage1_seconds\": "
              << JsonNumberOrNull(stage1_seconds) << ",\n"
              << "  \"stage2_seconds\": null,\n"
              << "  \"stage3_decision_score_seconds\": null,\n"
              << "  \"stage4_engine_seconds\": null\n}\n";
          return 1;
        }
        if (args.method == "structured_bias_only") {
          const auto stage1_result = uifgo::BuildStage1RegularizedResult(
              discovery, graph, builder.factor_meta(), plan, cfg,
              discovery_context, discovery_options);
          if (stage1_result.valid) {
            WriteTrajectory(run_dir, keyframes,
                            stage1_result.navigation_values);
            WriteBiases(run_dir, keyframes, stage1_result.navigation_values);
            std::set<std::uint64_t> candidate_obs_ids;
            for (const auto& segment : discovery.partition.segments)
              candidate_obs_ids.insert(segment.obs_ids.begin(),
                                       segment.obs_ids.end());
            auto bias_out = Open(run_dir / "stage1_observation_bias.csv");
            bias_out << "obs_id,tag_id,anchor_id,sensor_time,chain_id,"
                        "active_run_id,candidate,bias_m\n"
                     << std::setprecision(17);
            for (const auto& item : stage1_result.observation_bias_snapshot)
              bias_out << item.obs_id << ',' << item.tag_id << ','
                       << item.anchor_id << ',' << item.sensor_time << ','
                       << item.chain_id << ',' << item.active_run_id << ','
                       << candidate_obs_ids.count(item.obs_id) << ','
                       << item.bias_m << '\n';
          }
          const double elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - started).count();
          {
            auto out = Open(run_dir / "stage1_regularized_result.json");
            out << std::setprecision(17)
                << "{\n  \"schema\": \"uifgo_t09_stage1_regularized_result_v1\",\n"
                << "  \"status\": \"" << stage1_result.status << "\",\n"
                << "  \"reason\": \"" << JsonEscape(stage1_result.reason)
                << "\",\n  \"result_id\": \""
                << JsonEscape(stage1_result.result_id) << "\",\n"
                << "  \"objective_physical\": "
                << JsonNumberOrNull(stage1_result.objective_physical) << ",\n"
                << "  \"objective_l1\": "
                << JsonNumberOrNull(stage1_result.objective_l1) << ",\n"
                << "  \"objective_tv\": "
                << JsonNumberOrNull(stage1_result.objective_tv) << ",\n"
                << "  \"objective_total\": "
                << JsonNumberOrNull(stage1_result.objective_total) << ",\n"
                << "  \"graph_linearization_sha256\": \""
                << JsonEscape(stage1_result.graph_linearization_sha256)
                << "\",\n  \"values_sha256\": \""
                << JsonEscape(stage1_result.values_sha256)
                << "\",\n  \"snapshot_sha256\": \""
                << JsonEscape(stage1_result.snapshot_sha256)
                << "\",\n  \"covariance_status\": \""
                << stage1_result.covariance_status << "\"\n}\n";
          }
          {
            auto out = Open(run_dir / "run_manifest.json");
            out << "{\n  \"schema\": \"uifgo_t09_run_manifest_v1\",\n"
                << "  \"run_id\": \"" << JsonEscape(args.run_id)
                << "\",\n  \"canonical_mode\": \"structured_bias_only\",\n"
                << "  \"execution_type\": \"STAGE1_TRAJECTORY\",\n"
                << "  \"common_preparation_id\": \""
                << common_preparation_id << "\",\n"
                << "  \"final_request_id\": null,\n"
                << "  \"inference_id\": \"UNAVAILABLE:FINAL_ENGINE_NOT_ENTERED\",\n"
                << "  \"graph_linearization_sha256\": \""
                << JsonEscape(stage1_result.graph_linearization_sha256)
                << "\",\n  \"values_sha256\": \""
                << JsonEscape(stage1_result.values_sha256)
                << "\",\n  \"context_sha256\": \""
                << JsonEscape(stage1_result.snapshot_sha256)
                << "\",\n  \"export_verification_status\": \""
                << (stage1_result.valid ? "STAGE1_PHYSICAL_GRAPH_AUDIT_OK"
                                        : "FAILED")
                << "\"\n}\n";
          }
          {
            auto out = Open(run_dir / "run_status.json");
            out << "{\n  \"status\": \""
                << (stage1_result.valid ? "OK" : "ESTIMATION_FAILED")
                << "\",\n  \"exit_code\": "
                << (stage1_result.valid ? 0 : 1)
                << ",\n  \"canonical_mode\": \"structured_bias_only\",\n"
                << "  \"reason\": \"" << JsonEscape(stage1_result.reason)
                << "\",\n  \"valid_estimate_exported\": "
                << (stage1_result.valid ? "true" : "false")
                << ",\n  \"stage1_seconds\": "
                << JsonNumberOrNull(stage1_seconds)
                << ",\n  \"stage2_seconds\": null,\n"
                << "  \"elapsed_seconds\": " << JsonNumberOrNull(elapsed)
                << "\n}\n";
          }
          return stage1_result.valid ? 0 : 1;
        }
        support = discovery.partition;
        refit_initial = discovery.navigation_values;
        failure_stage = "AUTOMATIC_STAGE2_REFIT";
      } else {
        failure_stage = "IMU_AIDED_FDE";
        uifgo::FdeOptions fde_options;
        fde_options.chi2_probability = cfg.chi2_reject_prob;
        fde_options.chi2_degrees_of_freedom = 1;
        fde_options.gap_threshold_s = cfg.discovery_gap_threshold_s;
        fde_options.minimum_count =
            static_cast<size_t>(cfg.discovery_short_min_count);
        fde_options.minimum_duration_s =
            cfg.discovery_short_min_duration_s;
        fde_options.preliminary_lm.max_iterations = cfg.lm_max_iter;
        fde_options.preliminary_lm.relative_tolerance = cfg.lm_rel_tol;
        fde_options.preliminary_lm.absolute_tolerance = cfg.lm_abs_tol;
        fde_options.preliminary_lm.policy =
            uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1;
        const auto stage1_started = std::chrono::steady_clock::now();
        const auto fde = uifgo::ImuAidedFdeSupportProvider(fde_options).Run(
            graph, initial, builder.factor_meta(), plan, cfg, fde_context);
        stage1_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - stage1_started).count();
        WriteFdeArtifacts(run_dir, fde, fde_options);
        if (!fde.success()) {
          {
            auto capability = Open(run_dir / "capability_status.json");
            capability
                << "{\n  \"debug_label\": \"RQ3_IMU_AIDED_FDE_STAGE1\",\n"
                << "  \"discovery\": \"FAILED_"
                << uifgo::FdeStatusName(fde.status)
                << "\",\n  \"segment_refit\": \"NOT_RUN\",\n"
                << "  \"recoverability_score\": \"NOT_RUN\",\n"
                << "  \"gate\": \"NOT_RUN_STAGE1_FAILED\",\n"
                << "  \"fallback\": \"NOT_RUN_STAGE1_FAILED\",\n"
                << "  \"covariance\": \"NOT_COMPUTED\",\n"
                << "  \"output_semantics\": \"NO_VALID_ESTIMATE_STAGE1_FAILED\"\n}\n";
          }
          const double elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - started).count();
          auto out = Open(run_dir / "run_status.json");
          out << "{\n  \"status\": \"FAILED\",\n  \"exit_code\": 1,\n"
              << "  \"debug_label\": \"RQ3_IMU_AIDED_FDE_STAGE1\",\n"
              << "  \"failure_stage\": \"IMU_AIDED_FDE\",\n"
              << "  \"fde_status\": \"" << uifgo::FdeStatusName(fde.status)
              << "\",\n  \"reason\": \"" << JsonEscape(fde.reason)
              << "\",\n  \"stage2_refit_run\": false,\n"
              << "  \"gate_or_fallback_run\": false,\n"
              << "  \"elapsed_seconds\": " << JsonNumberOrNull(elapsed)
              << ",\n  \"stage1_seconds\": "
              << JsonNumberOrNull(stage1_seconds)
              << ",\n  \"stage2_seconds\": null\n}\n";
          return 1;
        }
        support = fde.partition;
        refit_initial = fde.reference.values;
        failure_stage = "FDE_STAGE2_REFIT";
      }

      uifgo::RefitOptions options;
      options.boundary_epsilon_m = cfg.refit_boundary_epsilon_m;
      options.relative_objective_tolerance =
          cfg.refit_relative_objective_tolerance;
      options.scaled_step_tolerance = cfg.refit_scaled_step_tolerance;
      options.projected_gradient_tolerance =
          cfg.refit_projected_gradient_tolerance;
      options.navigation_stationarity_tolerance_objective =
          cfg.refit_navigation_stationarity_tolerance_objective;
      options.gradient_roundoff_safety_factor =
          cfg.refit_gradient_roundoff_safety_factor;
      options.pose_rotation_scale_rad = cfg.refit_pose_rotation_scale_rad;
      options.pose_translation_scale_m = cfg.refit_pose_translation_scale_m;
      options.velocity_scale_mps = cfg.refit_velocity_scale_mps;
      options.accel_bias_scale_mps2 = cfg.refit_accel_bias_scale_mps2;
      options.gyro_bias_scale_radps = cfg.refit_gyro_bias_scale_radps;
      options.segment_amplitude_scale_m =
          cfg.refit_segment_amplitude_scale_m;
      options.max_refit_iterations =
          static_cast<size_t>(cfg.max_refit_iterations);
      options.lm_max_iterations = cfg.lm_max_iter;
      options.lm_relative_tolerance = cfg.lm_rel_tol;
      options.lm_absolute_tolerance = cfg.lm_abs_tol;
      const auto stage2_started = std::chrono::steady_clock::now();
      const uifgo::SegmentRefitResult refit =
          uifgo::SegmentRefitter(options).Run(
              graph, refit_initial, builder.factor_meta(), plan, cfg, support);
      stage2_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - stage2_started).count();
      WriteRefitIterations(run_dir, refit);
      if (cfg.final_inference_enabled) {
        WriteRefitIterations(run_dir, refit,
                             "stage2_refit_iterations.csv");
        auto stage2_status = Open(run_dir / "stage2_refit_status.json");
        stage2_status
            << "{\n  \"schema\": \"t08_stage2_refit_diagnostics_v1\",\n"
            << "  \"phase\": \"STAGE2_DEBIASED_REFIT\",\n"
            << "  \"execution_status\": \"EXECUTED\",\n"
            << "  \"solver_status\": \""
            << uifgo::SegmentRefitStatusName(refit.status)
            << "\",\n  \"stop_reason\": \"" << JsonEscape(refit.reason)
            << "\",\n  \"iteration_count\": " << refit.iterations.size()
            << ",\n  \"estimate_semantics\": "
               "\"INTERMEDIATE_NOT_A_VALID_T08_FINAL_EXPORT_SOURCE\"\n}\n";
      }

      {
        auto out = Open(run_dir / "input_manifest.json");
        out << "{\n  \"schema\": \"paper_input_v2\",\n"
            << "  \"source\": \"" << JsonEscape(source) << "\",\n"
            << "  \"recording_id\": \"" << recording_id << "\",\n"
            << "  \"source_hash_fnv1a64\": \"" << source_hash << "\",\n"
            << "  \"source_hash_sha256\": \"" << source_sha256 << "\",\n"
            << "  \"input_interface\": \"" << cfg.data_interface << "\",\n"
            << "  \"cache_id\": "
            << (loaded.from_t07_cache
                    ? "\"" + JsonEscape(loaded.cache_id) + "\""
                    : "null")
            << ",\n  \"cache_unit_conversion_applied\": false,\n"
            << "  \"recording_time_origin_s\": "
            << JsonNumberOrNull(loaded.recording_time_origin_s) << ",\n"
            << "  \"config_hash_fnv1a64\": \""
            << Hex(Fnv1aFile(config_path.string())) << "\",\n"
            << "  \"config_hash_sha256\": \"" << config_sha256 << "\",\n";
        if (oracle_debug_run) {
          out << "  \"oracle_support_hash_fnv1a64\": \""
              << Hex(Fnv1aFile(support_path.string())) << "\",\n";
        } else {
          out << "  \"discovery_snapshot_hash\": \""
              << JsonEscape(support.discovery_snapshot_hash) << "\",\n"
              << "  \"discovery_context_hash\": \""
              << JsonEscape(support.discovery_context_hash) << "\",\n"
              << "  \"calibration_hash_sha256\": \""
              << JsonEscape(support.calibration_hash) << "\",\n"
              << "  \"solver_config_hash_sha256\": \""
              << JsonEscape(support.solver_config_hash) << "\",\n"
              << "  \"partition_hash\": \""
              << JsonEscape(support.partition_hash) << "\",\n";
        }
        out << "  \"input_plan_hash_fnv1a64\": \"" << plan.plan_hash
            << "\",\n  \"input_plan_hash_sha256\": \"" << plan.plan_sha256
            << "\",\n  \"fixed_beta_status\": \""
            << beta_validation.status << "\",\n"
            << "  \"raw_observations\": " << plan.observations.size()
            << ",\n  \"valid_observations\": " << valid
            << ",\n  \"planned_observations\": " << planned
            << ",\n  \"suspected_nlos_planned\": " << suspected
            << ",\n  \"keyframes\": " << plan.keyframes.size()
            << ",\n  \"strategy\": \""
            << (cfg.final_inference_enabled
                    ? "t08_frozen_group_final_inference"
                    : (estimated_support_run
                           ? (imu_aided_fde_run
                                  ? "imu_aided_fde_refit_score"
                                  : "automatic_discovery_refit_score")
                           : "oracle_debug_segment_refit"))
            << "\",\n  \"stage1_provider\": \""
            << (imu_aided_fde_run
                    ? uifgo::kImuAidedFdeProvider
                    : (automatic_discovery_run ? "automatic_discovery"
                                               : support.provider))
            << "\",\n  \"debug_label\": \""
            << (cfg.final_inference_enabled
                    ? uifgo::kT08DevelopmentGateLabel
                    : (estimated_support_run
                           ? uifgo::kT06AutomaticDiscoveryLabel
                           : uifgo::kT04OracleDebugLabel))
            << "\",\n  \"gt_read\": false,\n"
            << "  \"oracle_support_read\": "
            << (oracle_debug_run ? "true" : "false") << ",\n"
            << "  \"gt_or_oracle_read\": "
            << (oracle_debug_run ? "true" : "false") << "\n}\n";
      }
      {
        auto out = Open(run_dir / "config_effective.yaml");
        out << "paper_path: true\nstrategy: "
            << (cfg.final_inference_enabled
                    ? "t08_frozen_group_final_inference\n"
                    : (estimated_support_run
                           ? (imu_aided_fde_run
                                  ? "imu_aided_fde_refit_score\n"
                                  : "automatic_discovery_refit_score\n")
                           : "oracle_debug_segment_refit\n"))
            << "debug_label: "
            << (cfg.final_inference_enabled
                    ? uifgo::kT08DevelopmentGateLabel
                    : (estimated_support_run
                           ? uifgo::kT06AutomaticDiscoveryLabel
                           : uifgo::kT04OracleDebugLabel))
            << '\n';
        if (oracle_debug_run)
          out << "oracle_support: " << support_path.string() << '\n';
        out
            << "single_tag: true\nfixed_extrinsics: true\n"
            << "fixed_time_offset_seconds: 0\nonline_beta_state: false\n"
            << "fixed_beta_status: " << beta_validation.status << '\n'
            << "fixed_beta_entries: " << cfg.fixed_beta_by_link.size() << '\n'
            << "fixed_beta_provenance: "
            << (cfg.fixed_beta_by_link.empty()
                    ? "MISSING_CALIBRATION_DEVELOPMENT_ONLY"
                    : "EXPLICIT_CONFIG_NOT_VERIFIED_AS_REAL_CALIBRATION")
            << "\nrefit_boundary_epsilon_m: "
            << cfg.refit_boundary_epsilon_m
            << "\nrefit_relative_objective_tolerance: "
            << cfg.refit_relative_objective_tolerance
            << "\nrefit_scaled_step_tolerance: "
            << cfg.refit_scaled_step_tolerance
            << "\nrefit_projected_gradient_tolerance: "
            << cfg.refit_projected_gradient_tolerance
            << "\nrefit_navigation_stationarity_tolerance_objective: "
            << cfg.refit_navigation_stationarity_tolerance_objective
            << "\nrefit_gradient_roundoff_safety_factor: "
            << cfg.refit_gradient_roundoff_safety_factor
            << "\nrefit_navigation_stationarity_metric: "
               "max_abs_final_joint_gradient_times_physical_scale\n"
            << "refit_navigation_stationarity_units: "
               "objective_per_normalized_coordinate\n"
            << "refit_gradient_roundoff_model: "
               "factorwise_abs_sum_times_8_gamma_n_plus_u\n"
            << "refit_pose_rotation_scale_rad: "
            << cfg.refit_pose_rotation_scale_rad
            << "\nrefit_pose_translation_scale_m: "
            << cfg.refit_pose_translation_scale_m
            << "\nrefit_velocity_scale_mps: "
            << cfg.refit_velocity_scale_mps
            << "\nrefit_accel_bias_scale_mps2: "
            << cfg.refit_accel_bias_scale_mps2
            << "\nrefit_gyro_bias_scale_radps: "
            << cfg.refit_gyro_bias_scale_radps
            << "\nrefit_segment_amplitude_scale_m: "
            << cfg.refit_segment_amplitude_scale_m
            << "\nmax_refit_iterations: " << cfg.max_refit_iterations
            << "\nconditional_lm_max_iterations: " << cfg.lm_max_iter
            << "\nconditional_lm_relative_tolerance: " << cfg.lm_rel_tol
            << "\nconditional_lm_absolute_tolerance: " << cfg.lm_abs_tol
            << "\nshort_min_count_debug: "
            << (estimated_support_run ? cfg.discovery_short_min_count
                                        : cfg.oracle_short_min_count_debug)
            << "\nshort_min_duration_debug_s: "
            << (estimated_support_run
                    ? cfg.discovery_short_min_duration_s
                    : cfg.oracle_short_min_duration_debug)
            << "\nshort_threshold_is_t05_gate: false"
            << "\nscore_recoverability: "
            << (cfg.score_recoverability ? "true" : "false")
            << "\nfinal_inference_enabled: "
            << (cfg.final_inference_enabled ? "true" : "false")
            << "\ngate_parameter_provenance: "
            << (cfg.final_inference_enabled
                    ? cfg.gate_parameter_provenance
                    : "NOT_APPLICABLE")
            << "\ntau_eta: "
            << (cfg.final_inference_enabled
                    ? JsonNumberOrNull(cfg.gate_tau_eta)
                    : "null")
            << "\ntau_s_m: "
            << (cfg.final_inference_enabled
                    ? JsonNumberOrNull(cfg.gate_tau_s_m)
                    : "null")
            << "\ntau_gamma: "
            << (cfg.final_inference_enabled
                    ? JsonNumberOrNull(cfg.gate_tau_gamma)
                    : "null")
            << "\ninput_plan_hash: " << plan.plan_hash << '\n';
        if (automatic_discovery_run) {
          out << "lambda_l1: " << cfg.discovery_lambda_l1
              << "\nlambda_tv: " << cfg.discovery_lambda_tv
              << "\ngap_threshold_s: " << cfg.discovery_gap_threshold_s
              << "\nactive_bias_min_m: "
              << cfg.discovery_active_bias_min_m
              << "\nchange_point_min_m: "
              << cfg.discovery_change_point_min_m
              << "\nmerge_max_difference_m: "
              << cfg.discovery_merge_max_difference_m
              << "\nrho_scale: " << cfg.discovery_rho_scale
              << "\nadmm_primal_abs_tolerance_m: "
              << cfg.discovery_primal_abs_tolerance_m
              << "\nadmm_primal_rel_tolerance: "
              << cfg.discovery_primal_rel_tolerance
              << "\nadmm_dual_abs_tolerance_objective_per_m: "
              << cfg.discovery_dual_abs_tolerance_objective_per_m
              << "\nadmm_dual_rel_tolerance: "
              << cfg.discovery_dual_rel_tolerance
              << "\nadmm_kkt_tolerance_objective_per_m: "
              << cfg.discovery_kkt_tolerance_objective_per_m
              << "\nadmm_tv_subgradient_tolerance_objective_per_m: "
              << cfg.discovery_tv_subgradient_tolerance_objective_per_m
              << "\nadmm_max_iterations: "
              << cfg.discovery_admm_max_iterations
              << "\ndiscovery_max_outer_iterations: "
              << cfg.discovery_max_outer_iterations
              << "\ndiscovery_conditional_navigation_policy: "
              << cfg.discovery_conditional_navigation_policy
              << "\ndiscovery_conditional_navigation_qualification_enabled: "
              << (ConditionalNavigationQualificationEnabled(cfg)
                      ? "true"
                      : "false")
              << "\ndiscovery_conditional_lm_optimizer_internal_relative_tolerance: "
              << ConditionalNavigationInternalRelativeTolerance(cfg)
              << "\ndiscovery_conditional_lm_external_relative_tolerance: "
              << cfg.lm_rel_tol
              << "\ndiscovery_conditional_lm_external_absolute_tolerance: "
              << cfg.lm_abs_tol
              << "\ndiscovery_scaled_step_tolerance: "
              << cfg.discovery_scaled_step_tolerance
              << "\ndiscovery_observation_bias_scale_m: "
              << cfg.discovery_observation_bias_scale_m
              << "\ndiscovery_step_metric: max_gtsam_local_navigation_and_per_observation_bias_scaled_step\n"
              << "input_plan_hash_sha256: " << plan.plan_sha256
              << "\nsource_hash_sha256: " << source_sha256
              << "\nconfig_hash_sha256: " << config_sha256
              << "\ncalibration_hash_sha256: "
              << support.calibration_hash
              << "\nsolver_config_hash_sha256: "
              << support.solver_config_hash
              << "\ndiscovery_context_hash: "
              << support.discovery_context_hash
              << "\nparameter_provenance: PENDING_VALIDATION_DEVELOPMENT_ONLY\n";
        } else if (imu_aided_fde_run) {
          out << "fde_provider: " << uifgo::kImuAidedFdeProvider
              << "\nfde_identity_version: "
              << uifgo::kImuAidedFdeIdentityVersion
              << "\nchi2_probability: " << cfg.chi2_reject_prob
              << "\nchi2_degrees_of_freedom: 1"
              << "\nchi2_threshold: " << uifgo::Chi2inv(0.99, 1)
              << "\ngap_threshold_s: " << cfg.discovery_gap_threshold_s
              << "\nminimum_count: " << cfg.discovery_short_min_count
              << "\nminimum_duration_s: "
              << cfg.discovery_short_min_duration_s
              << "\npreliminary_lm_policy: GTSAM_CHECK_ONLY_V1"
              << "\ninput_plan_hash_sha256: " << plan.plan_sha256
              << "\nsource_hash_sha256: " << source_sha256
              << "\ncalibration_hash_sha256: " << support.calibration_hash
              << "\nsolver_config_hash_sha256: "
              << support.solver_config_hash
              << "\nfde_context_hash: " << support.discovery_context_hash
              << "\nparameter_provenance: 0911_FDE_STEP1_FROZEN\n";
        }
      }
      {
        auto out = Open(run_dir / "capability_status.json");
        out << "{\n"
              << "  \"debug_label\": \""
              << (cfg.final_inference_enabled
                      ? uifgo::kT08DevelopmentGateLabel
                      : (estimated_support_run
                             ? uifgo::kT06AutomaticDiscoveryLabel
                             : uifgo::kT04OracleDebugLabel))
            << "\",\n"
            << "  \"discovery\": \""
            << (estimated_support_run ? "CONVERGED_STAGE1"
                                        : "NOT_IMPLEMENTED_T04")
            << "\",\n"
            << "  \"segment_refit\": \""
            << (refit.converged()
                    ? (estimated_support_run ? "T06_STAGE2_CONVERGED"
                                               : "T04_ORACLE_DEBUG_CONVERGED")
                    : (estimated_support_run ? "T06_STAGE2_FAILED"
                                               : "T04_ORACLE_DEBUG_FAILED"))
            << "\",\n"
            << "  \"recoverability_score\": \""
            << (cfg.score_recoverability ? "PENDING_AFTER_STAGE2"
                                         : "DISABLED_T04")
            << "\",\n"
            << "  \"gate\": \""
            << (cfg.final_inference_enabled ? "NOT_RUN_STAGE2_PENDING"
                                            : "NOT_IMPLEMENTED")
            << "\",\n"
            << "  \"fallback\": \""
            << (cfg.final_inference_enabled ? "NOT_RUN_STAGE2_PENDING"
                                            : "NOT_IMPLEMENTED")
            << "\",\n"
            << "  \"covariance\": \"NOT_COMPUTED\",\n"
            << "  \"output_semantics\": \"STAGE2_INTERMEDIATE_NOT_T08_FINAL\"\n}\n";
      }
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
      if (!refit.converged()) {
        auto out = Open(run_dir / "run_status.json");
        out << std::setprecision(17)
            << "{\n  \"status\": \"FAILED\",\n  \"exit_code\": 1,\n"
            << "  \"debug_label\": \""
            << (cfg.final_inference_enabled
                    ? uifgo::kT08DevelopmentGateLabel
                    : (estimated_support_run
                           ? uifgo::kT06AutomaticDiscoveryLabel
                           : uifgo::kT04OracleDebugLabel))
            << "\",\n"
            << "  \"reason\": \"" << JsonEscape(refit.reason) << "\",\n"
            << "  \"solver_status\": \""
            << uifgo::SegmentRefitStatusName(refit.status) << "\",\n"
            << "  \"valid_estimate_exported\": false,\n"
            << "  \"refit_outer_iterations\": " << refit.iterations.size()
            << ",\n  \"elapsed_seconds_semantics\": \""
            << kRunnerElapsedSemantics << "\",\n"
            << "  \"elapsed_seconds\": " << JsonNumberOrNull(elapsed)
            << ",\n  \"stage1_seconds\": "
            << JsonNumberOrNull(stage1_seconds)
            << ",\n  \"stage2_seconds\": "
            << JsonNumberOrNull(stage2_seconds)
            << ",\n  \"stage3_decision_score_seconds\": null,\n"
            << "  \"stage4_engine_seconds\": null"
            << "\n}\n";
        std::cerr << "paper runner ERROR: support refit "
                  << uifgo::SegmentRefitStatusName(refit.status) << ": "
                  << refit.reason << '\n';
        return 1;
      }

      if (!cfg.final_inference_enabled) {
        WriteTrajectory(run_dir, keyframes, refit.values);
        WriteBiases(run_dir, keyframes, refit.values);
        WriteSegmentResiduals(run_dir, cfg, plan, support, refit);
        WriteSegments(run_dir, refit, cfg);
        WriteFactorMetadata(run_dir, refit);
        WriteStage2Values(run_dir, refit.values);
        if (!fs::exists(run_dir / "partition.json"))
          WriteSupportPartition(run_dir / "partition.json", support);
        uifgo::InferenceIdentityContext stage2_context;
        stage2_context.input_sha256 = source_sha256;
        stage2_context.config_sha256 = config_sha256;
        stage2_context.input_plan_sha256 = plan.plan_sha256;
        stage2_context.support_partition_sha256 =
            support.partition_hash.empty()
                ? "sha256:" + uifgo::Sha256Hex("empty-partition")
                : support.partition_hash;
        stage2_context.calibration_sha256 = CalibrationContextHash(cfg, init.gravity_world);
        stage2_context.solver_config_sha256 =
            support.solver_config_hash.empty()
                ? "sha256:" + uifgo::Sha256Hex(
                      "t09-stage2-refit-config:" + config_sha256)
                : support.solver_config_hash;
        const auto stage2_identity = uifgo::ComputeInferenceContentIdentity(
            refit.graph, refit.values, stage2_context);
        auto identity_out = Open(run_dir / "stage2_content_identity.json");
        identity_out
            << "{\n  \"schema\": \"uifgo_t09_stage2_content_identity_v1\",\n"
            << "  \"graph_linearization_sha256\": \""
            << JsonEscape(stage2_identity.graph_linearization_sha256)
            << "\",\n  \"values_sha256\": \""
            << JsonEscape(stage2_identity.values_sha256)
            << "\",\n  \"context_sha256\": \""
            << JsonEscape(stage2_identity.context_sha256)
            << "\",\n  \"factor_count\": " << refit.graph.size()
            << ",\n  \"values_count\": " << refit.values.size() << "\n}\n";
        {
          auto context_out = Open(run_dir / "stage2_producer_context.json");
          context_out
              << "{\n  \"schema\": \"uifgo_t09_stage2_producer_context_v1\",\n"
              << "  \"stage1_config_sha256\": \""
              << JsonEscape(Stage1ProducerConfigHash(
                     cfg, imu_aided_fde_run ? &fde_context : nullptr))
              << "\",\n  \"stage2_refit_config_sha256\": \""
              << JsonEscape(Stage2RefitConfigHash(cfg))
              << "\",\n  \"stage3_score_config_sha256\": \""
              << JsonEscape(Stage3ScoreConfigHash()) << "\"\n}\n";
        }
        auto manifest_out = Open(run_dir / "run_manifest.json");
        manifest_out
            << "{\n  \"schema\": \"uifgo_t09_run_manifest_v1\",\n"
            << "  \"run_id\": \"" << JsonEscape(args.run_id)
            << "\",\n  \"canonical_mode\": \"structured_debias\",\n"
            << "  \"execution_type\": \"STAGE2_CACHE_PRODUCER\",\n"
            << "  \"common_preparation_id\": \""
            << common_preparation_id << "\",\n"
            << "  \"final_request_id\": null,\n"
            << "  \"inference_id\": \"UNAVAILABLE:FINAL_ENGINE_NOT_ENTERED\",\n"
            << "  \"graph_linearization_sha256\": \""
            << JsonEscape(stage2_identity.graph_linearization_sha256)
            << "\",\n  \"values_sha256\": \""
            << JsonEscape(stage2_identity.values_sha256)
            << "\",\n  \"context_sha256\": \""
            << JsonEscape(stage2_identity.context_sha256)
            << "\",\n  \"export_verification_status\": \"STAGE2_GRAPH_VALUES_AUDIT_OK\"\n}\n";
      }
      segment_refit_estimate_exported = true;
      size_t short_segments = 0;
      size_t boundary_segments = 0;
      for (const auto& segment : refit.segments) {
        short_segments += segment.short_support_debug;
        boundary_segments += segment.boundary;
      }
      size_t recoverability_groups = 0;
      bool recoverability_all_valid = false;
      const bool no_candidates = support.segments.empty();
      std::string recoverability_summary =
          cfg.score_recoverability && no_candidates
              ? "NOT_APPLICABLE_NO_CANDIDATES"
              : "DISABLED_T04";
      if (cfg.final_inference_enabled) {
        failure_stage = "T08_FINAL_INFERENCE";
        uifgo::FinalGatePolicy final_policy =
            uifgo::FinalGatePolicy::FULL_GATE;
        if (args.method == "lcb_partial")
        final_policy = uifgo::FinalGatePolicy::LCB_PARTIAL;
      else if (args.method == "lcb_fixed_full")
        final_policy = uifgo::FinalGatePolicy::LCB_FIXED_FULL;
      else if (args.method == "suppress_all")
        final_policy = uifgo::FinalGatePolicy::SUPPRESS_ALL;
      else if (args.method == "structured_debias")
        final_policy = uifgo::FinalGatePolicy::STRUCTURED_DEBIAS;
      else if (args.method == "fit_only")
          final_policy = uifgo::FinalGatePolicy::FIT_ONLY;
        else if (args.method == "s_fit")
          final_policy = uifgo::FinalGatePolicy::S_FIT;
        else if (args.method == "eta_only") {
          if (std::getenv("UIFGO_T09_ETA_ONLY_SYNTHETIC_FINAL") == nullptr)
            throw std::invalid_argument(
                "eta_only final requires predeclared synthetic-final provenance");
          final_policy = uifgo::FinalGatePolicy::ETA_ONLY;
        } else if (args.method == "nominal_curvature") {
          throw std::invalid_argument(
              "nominal_curvature is diagnostic-only in T09");
        }
        std::vector<uifgo::GroupRecoverabilityScore> decision_scores;
        if (!no_candidates) {
          const auto stage3_started = std::chrono::steady_clock::now();
          decision_scores = uifgo::ScoreRefitRecoverability(
              refit, support, plan, cfg);
          stage3_decision_score_seconds = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - stage3_started).count();
        }
        uifgo::GateThresholds gate;
        gate.tau_eta = cfg.gate_tau_eta;
        gate.tau_s_m = cfg.gate_tau_s_m;
        gate.tau_gamma = cfg.gate_tau_gamma;
        gate.parameter_provenance = cfg.gate_parameter_provenance;
        uifgo::InferenceIdentityContext identity_context;
        identity_context.input_sha256 = source_sha256;
        identity_context.config_sha256 = config_sha256;
        identity_context.input_plan_sha256 = plan.plan_sha256;
        identity_context.support_partition_sha256 =
            support.partition_hash.empty()
                ? "sha256:" + uifgo::Sha256FileHex(support_path.string())
                : support.partition_hash;
        identity_context.calibration_sha256 = CalibrationContextHash(cfg, init.gravity_world);
        identity_context.solver_config_sha256 = Stage2RefitConfigHash(cfg);
        const auto stage2_identity = uifgo::ComputeInferenceContentIdentity(
            refit.graph, refit.values, identity_context);
        uifgo::FinalRequestIdentityInput final_request_input;
        final_request_input.stage2_cache_id =
            "t09-inline-stage2:" + uifgo::Sha256Hex(
                stage2_identity.graph_linearization_sha256 +
                stage2_identity.values_sha256 + support.partition_hash);
        final_request_input.canonical_mode =
            args.method.empty() ? "full_gate" : args.method;
        final_request_input.policy_version =
            std::string("T09_") + uifgo::FinalGatePolicyName(final_policy) +
            "_FINAL_AUDIT_V1";
        final_request_input.thresholds_sha256 = GateThresholdsHash(
            cfg.gate_tau_eta, cfg.gate_tau_s_m, cfg.gate_tau_gamma);
        final_request_input.threshold_provenance =
            cfg.gate_parameter_provenance;
        final_request_input.final_refit_score_config_sha256 =
            FinalRefitScoreConfigHash(cfg);
        final_request_input.solver_sha256 = Stage2RefitConfigHash(cfg);
        final_request_input.common_preparation_id =
            common_preparation_id;
        const std::string final_request_id =
            uifgo::ComputeFinalRequestId(final_request_input);
        const uifgo::InferenceResult inference =
            uifgo::FinalInferenceEngine(gate, options, {}, {},
                                        final_policy).Run(
                graph, builder.factor_meta(), refit, support, decision_scores,
                plan, cfg, identity_context);
        stage4_engine_seconds = inference.timing.total_seconds;
        stage4_decision_seconds = inference.timing.decision_seconds;
        stage4_recovery_refit_seconds = inference.timing.recovery_refit_seconds;
        stage4_final_score_seconds = inference.timing.final_score_seconds;
        stage4_fallback_seconds = inference.timing.fallback_seconds;
        stage4_covariance_seconds = inference.timing.covariance_seconds;
        const auto artifact_started = std::chrono::steady_clock::now();
        const auto written =
            uifgo::WriteInferenceArtifacts(run_dir.string(), inference);
        const auto export_verification = uifgo::VerifyInferenceExport(
            run_dir.string(), inference, written.files);
        if (!export_verification.ok)
          throw std::runtime_error("T09 inference export verification failed: " +
                                   export_verification.reason);
        inference_artifact_export_seconds =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - artifact_started).count();
        {
          auto out = Open(run_dir / "capability_status.json");
          out << "{\n  \"debug_label\": \""
              << uifgo::kT08DevelopmentGateLabel << "\",\n"
              << "  \"discovery\": \""
              << (estimated_support_run ? "CONVERGED_STAGE1"
                                          : "ORACLE_DEBUG_INPUT")
              << "\",\n  \"segment_refit\": \"CONVERGED_STAGE2\",\n"
              << "  \"recoverability_score\": \"DECISION_SET_RETAINED\",\n"
              << "  \"gate\": \"FROZEN_GROUP_POLICY\",\n"
              << "  \"fallback\": \""
              << (inference.fallback.attempted
                      ? (inference.fallback.status == "SUCCESS"
                             ? "ATTEMPTED_ONCE_SUCCESS"
                             : "ATTEMPTED_ONCE_FAILED")
                      : "NOT_TRIGGERED")
              << "\",\n  \"covariance\": \""
              << uifgo::CovarianceStatusName(
                     inference.final_graph_covariance.status)
              << "\",\n  \"output_semantics\": \"SINGLE_T08_INFERENCE_RESULT\"\n}\n";
        }
        const bool valid = inference.valid_estimate();
        {
          const double runner_elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - started).count();
          auto out = Open(run_dir / "run_status.json");
          out << std::setprecision(17)
              << "{\n  \"status\": \""
              << uifgo::InferenceStatusName(inference.status)
              << "\",\n  \"exit_code\": " << (valid ? 0 : 1)
              << ",\n  \"debug_label\": \""
              << uifgo::kT08DevelopmentGateLabel
              << "\",\n  \"run_id\": \"" << JsonEscape(args.run_id)
              << "\",\n  \"failure_stage\": "
              << (valid ? "null" : "\"T08_FINAL_INFERENCE\"")
              << ",\n  \"reason\": \"" << JsonEscape(inference.reason)
              << "\",\n  \"inference_id\": \""
              << JsonEscape(inference.inference_id)
              << "\",\n  \"final_request_id\": \""
              << JsonEscape(final_request_id)
              << "\",\n  \"valid_estimate_exported\": "
              << (valid ? "true" : "false")
              << ",\n  \"final_graph_factors\": "
              << inference.final_graph.size()
              << ",\n  \"final_values\": " << inference.final_values.size()
              << ",\n  \"fallback_attempt_count\": "
              << inference.fallback.attempt_count
              << ",\n  \"covariance_status\": \""
              << uifgo::CovarianceStatusName(
                     inference.final_graph_covariance.status)
              << "\",\n  \"elapsed_seconds_semantics\": \""
              << kRunnerElapsedSemantics << "\",\n"
              << "  \"elapsed_seconds\": "
              << JsonNumberOrNull(runner_elapsed)
              << ",\n  \"stage1_seconds\": "
              << JsonNumberOrNull(stage1_seconds)
              << ",\n  \"stage2_seconds\": "
              << JsonNumberOrNull(stage2_seconds)
              << ",\n  \"stage3_decision_score_seconds\": "
              << JsonNumberOrNull(stage3_decision_score_seconds)
              << ",\n  \"stage4_engine_seconds\": "
              << JsonNumberOrNull(inference.timing.total_seconds)
              << ",\n  \"stage4_decision_seconds\": "
              << JsonNumberOrNull(inference.timing.decision_seconds)
              << ",\n  \"stage4_recovery_refit_seconds\": "
              << JsonNumberOrNull(inference.timing.recovery_refit_seconds)
              << ",\n  \"stage4_final_score_seconds\": "
              << JsonNumberOrNull(inference.timing.final_score_seconds)
              << ",\n  \"stage4_fallback_seconds\": "
              << JsonNumberOrNull(inference.timing.fallback_seconds)
              << ",\n  \"stage4_covariance_seconds\": "
              << JsonNumberOrNull(inference.timing.covariance_seconds)
              << ",\n  \"stage4_engine_semantics\": \"FROZEN_DECISION_THROUGH_FINAL_COVARIANCE_INCLUDING_RECOVERY_FINAL_SCORE_AND_ANY_FALLBACK\",\n"
              << "  \"inference_artifact_export_seconds\": "
              << JsonNumberOrNull(inference_artifact_export_seconds)
              << "\n}\n";
        }
        {
          auto out = Open(run_dir / "run_manifest.json");
          out << "{\n  \"schema\": \"uifgo_t09_run_manifest_v1\",\n"
              << "  \"run_id\": \"" << JsonEscape(args.run_id) << "\",\n"
              << "  \"canonical_mode\": \""
              << JsonEscape(args.method.empty() ? "full_gate" : args.method)
              << "\",\n  \"common_preparation_id\": \""
              << common_preparation_id
              << "\",\n  \"final_request_id\": \""
              << JsonEscape(final_request_id)
              << "\",\n  \"operating_point_id\": \""
              << JsonEscape(args.operating_point_id)
              << "\",\n  \"policy_version\": \""
              << JsonEscape(final_request_input.policy_version)
              << "\",\n  \"thresholds_sha256\": \""
              << JsonEscape(final_request_input.thresholds_sha256)
              << "\",\n  \"threshold_provenance\": \""
              << JsonEscape(final_request_input.threshold_provenance)
              << "\",\n  \"final_refit_score_config_sha256\": \""
              << JsonEscape(
                     final_request_input.final_refit_score_config_sha256)
              << "\",\n  \"solver_sha256\": \""
              << JsonEscape(final_request_input.solver_sha256)
              << "\",\n  \"inference_id\": \""
              << JsonEscape(inference.inference_id)
              << "\",\n  \"graph_linearization_sha256\": \""
              << JsonEscape(
                     inference.content_identity.graph_linearization_sha256)
              << "\",\n  \"values_sha256\": \""
              << JsonEscape(inference.content_identity.values_sha256)
              << "\",\n  \"context_sha256\": \""
              << JsonEscape(inference.content_identity.context_sha256)
              << "\",\n  \"export_verification_status\": \""
              << export_verification.status
              << "\",\n  \"exported_file_count\": "
              << export_verification.checked_files
              << ",\n  \"artifact_sha256\": [";
          for (size_t i = 0;
               i < export_verification.artifact_sha256.size(); ++i) {
            if (i) out << ',';
            out << "\n    \""
                << JsonEscape(export_verification.artifact_sha256[i])
                << "\"";
          }
          if (!export_verification.artifact_sha256.empty()) out << '\n';
          out << "  ]\n}\n";
        }
        if (!valid) {
          std::cerr << "paper runner ERROR: T08 "
                    << uifgo::InferenceStatusName(inference.status) << ": "
                    << inference.reason << '\n';
          return 1;
        }
        std::cout << "paper runner OK: " << run_dir << '\n'
                  << "debug_label=" << uifgo::kT08DevelopmentGateLabel
                  << " status="
                  << uifgo::InferenceStatusName(inference.status)
                  << " groups=" << inference.decisions.size()
                  << " fallback_attempts="
                  << inference.fallback.attempt_count << '\n';
        return 0;
      }
      if (cfg.score_recoverability && !no_candidates) {
        failure_stage = "RECOVERABILITY_SCORE";
        const auto scores = uifgo::ScoreRefitRecoverability(
            refit, support, plan, cfg);
        recoverability_groups = scores.size();
        recoverability_all_valid = WriteRecoverabilityScores(run_dir, scores);
        recoverability_summary = recoverability_all_valid
                                     ? "ALL_GROUP_SCORES_EXPORTED"
                                     : "ONE_OR_MORE_GROUP_SCORES_UNAVAILABLE";
        {
          auto out = Open(run_dir / "capability_status.json");
          out << "{\n  \"debug_label\": \""
              << (estimated_support_run
                      ? uifgo::kT06AutomaticDiscoveryLabel
                      : uifgo::kT05OracleScoreDebugLabel)
              << "\",\n"
              << "  \"discovery\": \""
              << (estimated_support_run ? "CONVERGED_STAGE1"
                                          : "NOT_IMPLEMENTED_T05")
              << "\",\n"
              << "  \"segment_refit\": \"CONVERGED\",\n"
              << "  \"segment_refit_estimate_exported\": true,\n"
              << "  \"recoverability_score\": \""
              << recoverability_summary << "\",\n"
              << "  \"valid_score_exported\": "
              << (recoverability_all_valid ? "true" : "false") << ",\n"
              << "  \"gate\": \"NOT_IMPLEMENTED\",\n"
              << "  \"fallback\": \"NOT_IMPLEMENTED\",\n"
              << "  \"covariance\": \"NOT_COMPUTED\",\n"
              << "  \"output_semantics\": \"STAGE2_INTERMEDIATE_NOT_T08_FINAL\"\n}\n";
        }
        if (!recoverability_all_valid) {
          auto out = Open(run_dir / "run_status.json");
          out << "{\n  \"status\": \"PARTIAL_STAGE2_OK_SCORE_UNAVAILABLE\",\n"
              << "  \"exit_code\": 1,\n  \"debug_label\": \""
              << (estimated_support_run
                      ? uifgo::kT06AutomaticDiscoveryLabel
                      : uifgo::kT05OracleScoreDebugLabel)
              << "\",\n"
              << "  \"segment_refit_status\": \"CONVERGED\",\n"
              << "  \"segment_refit_estimate_exported\": true,\n"
              << "  \"recoverability_status\": \""
              << recoverability_summary << "\",\n"
              << "  \"segment_count\": " << refit.segments.size()
              << ",\n  \"short_segment_count\": " << short_segments
              << ",\n  \"boundary_segment_count\": "
              << boundary_segments << ",\n"
              << "  \"valid_score_exported\": false,\n"
              << "  \"gate_or_fallback_run\": false\n}\n";
          std::cerr << "paper runner ERROR: " << recoverability_summary << '\n';
          return 1;
        }
      } else if (cfg.score_recoverability && no_candidates) {
        WriteRecoverabilityScores(run_dir, {});
        auto out = Open(run_dir / "capability_status.json");
        out << "{\n  \"debug_label\": \""
            << uifgo::kT06AutomaticDiscoveryLabel << "\",\n"
            << "  \"discovery\": \"NO_CANDIDATES\",\n"
            << "  \"segment_refit\": \"RAW_STAGE2_CONVERGED\",\n"
            << "  \"segment_refit_estimate_exported\": true,\n"
            << "  \"recoverability_score\": \"NOT_APPLICABLE_NO_CANDIDATES\",\n"
            << "  \"valid_score_exported\": false,\n"
            << "  \"gate\": \"NOT_IMPLEMENTED\",\n"
            << "  \"fallback\": \"NOT_IMPLEMENTED\",\n"
            << "  \"covariance\": \"NOT_COMPUTED\",\n"
            << "  \"output_semantics\": \"STAGE2_INTERMEDIATE_NOT_T08_FINAL\"\n}\n";
      }
      {
        auto out = Open(run_dir / "run_status.json");
        out << std::setprecision(17)
            << "{\n  \"status\": \""
            << (estimated_support_run && no_candidates ? "NO_CANDIDATES"
                                                         : "OK")
            << "\",\n  \"exit_code\": 0,\n"
            << "  \"debug_label\": \""
            << (estimated_support_run
                    ? uifgo::kT06AutomaticDiscoveryLabel
                    : (cfg.score_recoverability
                           ? uifgo::kT05OracleScoreDebugLabel
                           : uifgo::kT04OracleDebugLabel))
            << "\",\n"
            << "  \"solver_status\": \"CONVERGED\",\n"
            << "  \"segment_refit_status\": \"CONVERGED\",\n"
            << "  \"segment_refit_estimate_exported\": true,\n"
            << "  \"recoverability_status\": \""
            << recoverability_summary << "\",\n"
            << "  \"valid_score_exported\": "
            << (recoverability_all_valid ? "true" : "false") << ",\n"
            << "  \"recoverability_groups\": " << recoverability_groups
            << ",\n"
            << "  \"solver_termination\": \"" << JsonEscape(refit.reason)
            << "\",\n  \"final_graph_factors\": " << refit.graph.size()
            << ",\n  \"final_values\": " << refit.values.size()
            << ",\n  \"segment_count\": " << refit.segments.size()
            << ",\n  \"short_segment_count\": " << short_segments
            << ",\n  \"boundary_segment_count\": " << boundary_segments
            << ",\n  \"partition_hash\": \""
            << JsonEscape(support.partition_hash) << "\""
            << ",\n  \"refit_outer_iterations\": " << refit.iterations.size()
            << ",\n  \"discovery_outer_iterations\": "
            << (automatic_discovery_run ? discovery.iterations.size() : 0)
            << ",\n  \"final_discovery_combined_scaled_step\": "
            << (automatic_discovery_run && !discovery.iterations.empty()
                    ? JsonNumberOrNull(
                          discovery.iterations.back().combined_scaled_step)
                    : "null")
            << ",\n  \"discovery_scaled_step_tolerance\": "
            << (automatic_discovery_run
                    ? JsonNumberOrNull(cfg.discovery_scaled_step_tolerance)
                    : "null")
            << ",\n  \"final_scaled_navigation_gradient_objective\": "
            << (refit.iterations.empty()
                    ? "null"
                    : JsonNumberOrNull(
                          refit.iterations.back()
                              .max_scaled_navigation_gradient_objective))
            << ",\n  \"navigation_stationarity_tolerance_objective\": "
            << (refit.iterations.empty()
                    ? "null"
                    : JsonNumberOrNull(
                          refit.iterations.back()
                              .navigation_stationarity_tolerance_objective))
            << ",\n  \"navigation_gradient_roundoff_allowance_objective\": "
            << (refit.iterations.empty()
                    ? "null"
                    : JsonNumberOrNull(
                          refit.iterations.back()
                              .navigation_gradient_roundoff_allowance_objective))
            << ",\n  \"final_error\": "
            << JsonNumberOrNull(refit.graph.error(refit.values))
            << ",\n  \"elapsed_seconds\": " << JsonNumberOrNull(elapsed)
            << "\n}\n";
      }
      std::cout << "paper runner OK: " << run_dir << '\n'
                << "debug_label="
                << (estimated_support_run
                        ? uifgo::kT06AutomaticDiscoveryLabel
                        : (cfg.score_recoverability
                               ? uifgo::kT05OracleScoreDebugLabel
                               : uifgo::kT04OracleDebugLabel))
                << " segments=" << refit.segments.size()
                << " outer_iterations=" << refit.iterations.size() << '\n';
      return 0;
    }

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(cfg.lm_max_iter);
    params.setRelativeErrorTol(cfg.lm_rel_tol);
    params.setAbsoluteErrorTol(cfg.lm_abs_tol);
    params.setLinearSolverType("SEQUENTIAL_CHOLESKY");
    const LmResult lm = RunCheckedLm(graph, initial, params, keyframes.size());
    const gtsam::Values& final_values = lm.values;

    WriteTrajectory(run_dir, keyframes, final_values);
    WriteBiases(run_dir, keyframes, final_values);
    WriteResiduals(run_dir, cfg, plan, final_values);

    {
      auto out = Open(run_dir / "input_manifest.json");
      out << "{\n  \"schema\": \"paper_input_v2\",\n"
          << "  \"source\": \"" << JsonEscape(source) << "\",\n"
          << "  \"recording_id\": \"" << recording_id << "\",\n"
          << "  \"source_hash_fnv1a64\": \"" << source_hash << "\",\n"
          << "  \"source_hash_sha256\": \"" << source_sha256 << "\",\n"
          << "  \"input_interface\": \"" << cfg.data_interface << "\",\n"
          << "  \"cache_id\": "
          << (loaded.from_t07_cache
                  ? "\"" + JsonEscape(loaded.cache_id) + "\""
                  : "null")
          << ",\n  \"cache_unit_conversion_applied\": false,\n"
          << "  \"recording_time_origin_s\": "
          << JsonNumberOrNull(loaded.recording_time_origin_s) << ",\n"
          << "  \"config_hash_fnv1a64\": \""
          << Hex(Fnv1aFile(config_path.string())) << "\",\n"
          << "  \"input_plan_hash_fnv1a64\": \"" << plan.plan_hash
          << "\",\n  \"fixed_beta_status\": \""
          << beta_validation.status
          << "\",\n  \"raw_observations\": " << plan.observations.size()
          << ",\n  \"valid_observations\": " << valid
          << ",\n  \"planned_observations\": " << planned
          << ",\n  \"suspected_nlos_planned\": " << suspected
          << ",\n  \"keyframes\": " << plan.keyframes.size()
          << ",\n  \"strategy\": \"all_valid_no_rejection\",\n"
          << "  \"gt_or_oracle_read\": false\n}\n";
    }
    {
      auto out = Open(run_dir / "config_effective.yaml");
      out << "paper_path: true\nstrategy: all_valid_no_rejection\n"
          << "single_tag: true\nfixed_extrinsics: true\n"
          << "fixed_time_offset_seconds: 0\nsfuise_group_window_seconds: 0\n"
          << "online_beta_state: false\nfixed_beta_status: "
          << beta_validation.status << "\nfixed_beta_entries: "
          << cfg.fixed_beta_by_link.size() << "\n"
          << "fixed_beta_provenance: "
          << (cfg.fixed_beta_by_link.empty()
                  ? "MISSING_CALIBRATION_DEVELOPMENT_ONLY"
                  : "EXPLICIT_CONFIG_NOT_VERIFIED_AS_REAL_CALIBRATION")
          << "\ninput_plan_hash: " << plan.plan_hash << '\n';
    }
    {
      auto out = Open(run_dir / "capability_status.json");
      out << "{\n"
          << "  \"discovery\": \"NOT_IMPLEMENTED_T02\",\n"
          << "  \"segment_refit\": \"NOT_IMPLEMENTED_T02\",\n"
          << "  \"recoverability_score\": \"NOT_IMPLEMENTED_T02\",\n"
          << "  \"gate\": \"NOT_IMPLEMENTED_T02\",\n"
          << "  \"covariance\": \"NOT_COMPUTED_T02\",\n"
          << "  \"fixed_time_offset\": \"ONLY_ZERO_TD_INIT_T02\",\n"
          << "  \"sfuise_grouping\": \"ONLY_ZERO_WINDOW_T02\"\n}\n";
    }
    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - started)
                               .count();
    {
      auto out = Open(run_dir / "run_status.json");
      out << std::setprecision(17)
          << "{\n  \"status\": \"OK\",\n  \"exit_code\": 0,\n"
          << "  \"final_graph_factors\": " << graph.size() << ",\n"
          << "  \"final_values\": " << final_values.size() << ",\n"
          << "  \"uwb_factors\": " << uwb_indices.size() << ",\n"
          << "  \"factor_meta_records\": " << builder.factor_meta().size()
          << ",\n  \"solver_status\": \"CONVERGED\",\n"
          << "  \"solver_termination\": \"" << lm.termination << "\",\n"
          << "  \"solver_iterations\": " << lm.iterations << ",\n"
          << "  \"solver_inner_iterations\": " << lm.inner_iterations
          << ",\n  \"solver_final_lambda\": "
          << JsonNumberOrNull(lm.final_lambda)
          << ",\n  \"solver_last_abs_error_change\": "
          << JsonNumberOrNull(lm.last_abs_error_change)
          << ",\n  \"solver_last_relative_error_change\": "
          << JsonNumberOrNull(lm.last_relative_error_change)
          << ",\n  \"initial_error\": " << JsonNumberOrNull(lm.initial_error)
          << ",\n  \"final_error\": " << JsonNumberOrNull(lm.final_error)
          << ",\n  \"elapsed_seconds\": " << JsonNumberOrNull(elapsed)
          << "\n}\n";
    }
    std::cout << "paper runner OK: " << run_dir << "\n"
              << "input_plan_hash=" << plan.plan_hash
              << " keyframes=" << plan.keyframes.size()
              << " uwb_factors=" << uwb_indices.size() << "\n";
    return 0;
  } catch (const LmFailure& error) {
    std::cerr << "paper runner ERROR: " << error.what() << '\n';
    if (run_dir_created) {
      try {
        const double elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - started)
                                   .count();
        const auto& lm = error.result;
        auto out = Open(run_dir / "run_status.json");
        out << std::setprecision(17)
            << "{\n  \"status\": \"FAILED\",\n  \"exit_code\": 1,\n"
            << "  \"reason\": \"" << JsonEscape(error.what()) << "\",\n"
            << "  \"solver_status\": \"FAILED\",\n"
            << "  \"solver_termination\": \"" << lm.termination << "\",\n"
            << "  \"solver_iterations\": " << lm.iterations << ",\n"
            << "  \"solver_inner_iterations\": " << lm.inner_iterations
            << ",\n  \"solver_final_lambda\": "
            << JsonNumberOrNull(lm.final_lambda)
            << ",\n  \"solver_last_abs_error_change\": "
            << JsonNumberOrNull(lm.last_abs_error_change)
            << ",\n  \"solver_last_relative_error_change\": "
            << JsonNumberOrNull(lm.last_relative_error_change)
            << ",\n  \"initial_error\": " << JsonNumberOrNull(lm.initial_error)
            << ",\n  \"final_error\": " << JsonNumberOrNull(lm.final_error)
            << ",\n  \"elapsed_seconds_semantics\": \""
            << kRunnerElapsedSemantics << "\",\n"
            << "  \"elapsed_seconds\": " << JsonNumberOrNull(elapsed)
            << ",\n  \"stage1_seconds\": "
            << JsonNumberOrNull(stage1_seconds)
            << ",\n  \"stage2_seconds\": "
            << JsonNumberOrNull(stage2_seconds)
            << ",\n  \"stage3_decision_score_seconds\": "
            << JsonNumberOrNull(stage3_decision_score_seconds)
            << ",\n  \"stage4_engine_seconds\": "
            << JsonNumberOrNull(stage4_engine_seconds)
            << ",\n  \"stage4_decision_seconds\": "
            << JsonNumberOrNull(stage4_decision_seconds)
            << ",\n  \"stage4_recovery_refit_seconds\": "
            << JsonNumberOrNull(stage4_recovery_refit_seconds)
            << ",\n  \"stage4_final_score_seconds\": "
            << JsonNumberOrNull(stage4_final_score_seconds)
            << ",\n  \"stage4_fallback_seconds\": "
            << JsonNumberOrNull(stage4_fallback_seconds)
            << ",\n  \"stage4_covariance_seconds\": "
            << JsonNumberOrNull(stage4_covariance_seconds)
            << ",\n  \"inference_artifact_export_seconds\": "
            << JsonNumberOrNull(inference_artifact_export_seconds)
            << "\n}\n";
      } catch (...) {
      }
    }
    return 1;
  } catch (const std::exception& error) {
    std::cerr << "paper runner ERROR: " << error.what() << '\n';
    if (run_dir_created) {
      try {
        const double elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - started)
                                   .count();
        auto out = Open(run_dir / "run_status.json");
        out << "{\n  \"status\": \"FAILED\",\n  \"exit_code\": 1,\n"
            << "  \"reason\": \"" << JsonEscape(error.what()) << "\"";
        if ((oracle_debug_run || fixed_partition_debug_run ||
             automatic_discovery_run || imu_aided_fde_run) &&
            failure_stage == "RECOVERABILITY_SCORE" &&
            segment_refit_estimate_exported) {
          out << ",\n  \"debug_label\": \""
              << ((automatic_discovery_run || imu_aided_fde_run)
                      ? uifgo::kT06AutomaticDiscoveryLabel
                      : uifgo::kT05OracleScoreDebugLabel)
              << "\",\n"
              << "  \"failure_stage\": \"RECOVERABILITY_SCORE\",\n"
              << "  \"segment_refit_status\": \"CONVERGED\",\n"
              << "  \"segment_refit_estimate_exported\": true,\n"
              << "  \"recoverability_status\": \"EXCEPTION\",\n"
              << "  \"valid_score_exported\": false,\n"
              << "  \"gate_or_fallback_run\": false";
        } else if (oracle_debug_run || fixed_partition_debug_run ||
                   automatic_discovery_run || imu_aided_fde_run) {
          out << ",\n  \"debug_label\": \""
              << ((automatic_discovery_run || imu_aided_fde_run)
                      ? uifgo::kT06AutomaticDiscoveryLabel
                      : uifgo::kT04OracleDebugLabel)
              << "\",\n"
              << "  \"failure_stage\": \"" << JsonEscape(failure_stage)
              << "\",\n  \"solver_status\": \""
              << (failure_stage == "ORACLE_SUPPORT_LOAD"
                      ? "INVALID_ORACLE_MANIFEST"
                      : "FAILED_BEFORE_VALID_EXPORT")
              << "\",\n  \"valid_estimate_exported\": false";
        }
        if (failure_stage == "T09_CACHE_REPLAY") {
          out << ",\n  \"stage1_execution\": \"NOT_RUN_CACHE_REPLAY\",\n"
              << "  \"stage2_execution\": \"NOT_RUN_CACHE_REPLAY\"";
        }
        out << ",\n  \"elapsed_seconds_semantics\": \""
            << kRunnerElapsedSemantics << "\",\n"
            << "  \"elapsed_seconds\": " << JsonNumberOrNull(elapsed)
            << ",\n  \"stage1_seconds\": "
            << JsonNumberOrNull(stage1_seconds)
            << ",\n  \"stage2_seconds\": "
            << JsonNumberOrNull(stage2_seconds)
            << ",\n  \"stage3_decision_score_seconds\": "
            << JsonNumberOrNull(stage3_decision_score_seconds)
            << ",\n  \"stage4_engine_seconds\": "
            << JsonNumberOrNull(stage4_engine_seconds)
            << ",\n  \"stage4_decision_seconds\": "
            << JsonNumberOrNull(stage4_decision_seconds)
            << ",\n  \"stage4_recovery_refit_seconds\": "
            << JsonNumberOrNull(stage4_recovery_refit_seconds)
            << ",\n  \"stage4_final_score_seconds\": "
            << JsonNumberOrNull(stage4_final_score_seconds)
            << ",\n  \"stage4_fallback_seconds\": "
            << JsonNumberOrNull(stage4_fallback_seconds)
            << ",\n  \"stage4_covariance_seconds\": "
            << JsonNumberOrNull(stage4_covariance_seconds)
            << ",\n  \"inference_artifact_export_seconds\": "
            << JsonNumberOrNull(inference_artifact_export_seconds)
            << "\n}\n";
      } catch (...) {
      }
    }
    return 1;
  }
}
