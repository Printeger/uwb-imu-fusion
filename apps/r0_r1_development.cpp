#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>

#ifndef UWB_IMU_PL_GIT_SHA
#define UWB_IMU_PL_GIT_SHA "unknown"
#endif
#ifndef UWB_IMU_PL_GIT_DIRTY
#define UWB_IMU_PL_GIT_DIRTY 1
#endif

namespace {
constexpr std::uint64_t kSeed = 20260901;
const Eigen::Vector3d kInitialPosition(0.0, 0.0, 1.2);
const Eigen::Vector3d kTruthVelocity(0.45, 0.45, 0.0);
constexpr double kYawRate = 0.20;

Eigen::Vector3d truthPosition(double t) {
  return kInitialPosition + t * kTruthVelocity;
}

Eigen::Quaterniond truthOrientation(double t) {
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(kYawRate * t, Eigen::Vector3d::UnitZ()));
}

bool oneOf(const std::string& value,
           std::initializer_list<const char*> choices) {
  return std::any_of(choices.begin(), choices.end(),
                     [&](const char* choice) { return value == choice; });
}

struct Scenario {
  std::string name;
  int fault_begin = -1;
  int fault_end = -1;
  bool uwb_fault = false;
  bool imu_fault = false;
  bool ramp_enabled = false;
  bool zero_initial_velocity = false;
  bool low_redundancy = false;
  bool force_rejection = false;
  bool mature = false;
  double uwb_fault_m = 0.0;
  double accel_fault_mps2 = 0.0;
  double gyro_fault_radps = 0.0;
  int accel_fault_axis = 0;
};

double environmentDouble(const char* name, double fallback) {
  const char* text = std::getenv(name);
  if (!text) return fallback;
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (text[consumed] != '\0' || !std::isfinite(value)) {
    throw std::invalid_argument(std::string("invalid ") + name);
  }
  return value;
}

Scenario parseScenario(const std::string& name,
                       const std::string& manifest_path) {
  Scenario s;
  s.name = name;
  if (name == "A_nominal") {}
  if (name == "A_default_ramp") {
    s.ramp_enabled = true;
  }
  if (name == "B_initial_velocity_error") {
    s.zero_initial_velocity = true;
  }
  if (name == "C_uwb_fde") {
    s.fault_begin = s.fault_end = 25; s.uwb_fault = true;
  }
  if (name == "D_imu_bridge") {
    s.fault_begin = s.fault_end = 25; s.imu_fault = true;
  }
  if (name == "E_union") {
    s.fault_begin = s.fault_end = 25; s.uwb_fault = s.imu_fault = true;
  }
  if (name == "F_ramp_unmonitorable") {
    s.ramp_enabled = true;
  }
  if (name == "F_low_redundancy") {
    s.low_redundancy = true;
  }
  if (name == "G_continuous_rejection") {
    s.fault_begin = 5; s.fault_end = 40; s.force_rejection = true;
  }
  if (name == "G_bridge_timeout") {
    s.fault_begin = 25; s.fault_end = 50; s.imu_fault = true;
  }
  if (name == "H_mature_union") {
    s.fault_begin = s.fault_end = 225; s.uwb_fault = s.imu_fault = true;
    s.mature = true;
  }
  if (name == "HIP_history_crossing_fault") {
    // C1 readiness item 8: an early-onset persistent single-anchor UWB bias.
    // The onset sits inside the A3 recoverable horizon while the fault itself
    // outlives the fixed-lag window, so the epoch that carries the onset is
    // compressed into the history summary and the fault must stay monitorable
    // through the summary's response (T_b) and detection content (F_b).
    s.uwb_fault = true;
    s.fault_begin = 6;
    s.fault_end = 200;
  }
  if (!oneOf(name, {"A_nominal", "A_default_ramp",
                   "B_initial_velocity_error", "C_uwb_fde",
                   "D_imu_bridge", "E_union", "F_ramp_unmonitorable",
                   "F_low_redundancy", "G_continuous_rejection",
                   "G_bridge_timeout", "H_mature_union",
                   "HIP_history_crossing_fault"})) {
    throw std::invalid_argument("unknown development scenario: " + name);
  }

  const YAML::Node root = YAML::LoadFile(manifest_path);
  if (root["schema"].as<std::string>() !=
      "uwb-imu-pl/r0-r1-development/v1" ||
      root["seed"].as<std::uint64_t>() != kSeed) {
    throw std::invalid_argument("development scenario manifest identity mismatch");
  }
  const YAML::Node node = root["scenarios"][name];
  if (!node || !node.IsMap()) {
    throw std::invalid_argument("scenario is absent from development manifest: " + name);
  }
  if (node["fault_epoch"]) {
    s.fault_begin = s.fault_end = node["fault_epoch"].as<int>();
  }
  if (node["fault_epoch_begin"]) {
    s.fault_begin = node["fault_epoch_begin"].as<int>();
  }
  if (node["fault_epoch_end"]) {
    s.fault_end = node["fault_epoch_end"].as<int>();
  }
  if (node["uwb_ramp_enabled"]) {
    s.ramp_enabled = node["uwb_ramp_enabled"].as<bool>();
  }
  if (node["initial_velocity_mps"]) {
    const auto velocity = node["initial_velocity_mps"];
    if (!velocity.IsSequence() || velocity.size() != 3) {
      throw std::invalid_argument("scenario initial_velocity_mps must have length three");
    }
    s.zero_initial_velocity = velocity[0].as<double>() == 0.0 &&
        velocity[1].as<double>() == 0.0 && velocity[2].as<double>() == 0.0;
  }
  if (node["raw_range_bias_m"]) {
    s.uwb_fault_m = node["raw_range_bias_m"].as<double>();
  }
  if (node["all_anchor_raw_range_bias_m"]) {
    s.uwb_fault_m = node["all_anchor_raw_range_bias_m"].as<double>();
  }
  if (node["raw_accel_bias_mps2"]) {
    s.accel_fault_mps2 = node["raw_accel_bias_mps2"].as<double>();
  }
  if (node["raw_gyro_bias_radps"]) {
    s.gyro_fault_radps = node["raw_gyro_bias_radps"].as<double>();
  }
  if (node["raw_imu_axis"] && node["raw_imu_axis"].as<std::string>() != "accel_x") {
    throw std::invalid_argument("development runner currently supports raw_imu_axis=accel_x");
  }
  return s;
}

void writeScenarioManifest(const std::string& directory, const Scenario& s,
                           int epochs, std::size_t anchors,
                           double uwb_fault_m, double accel_fault_mps2,
                           std::uint64_t uwb_fault_anchor,
                           int accel_fault_axis) {
  std::ofstream out(directory + "/development_scenario.json");
  if (!out) throw std::runtime_error("cannot write development scenario manifest");
  out << "{\n"
      << "  \"schema\": \"uwb-imu-pl/r0-r1-development/v1\",\n"
      << "  \"scenario\": \"" << s.name << "\",\n"
      << "  \"seed\": " << kSeed << ",\n"
      << "  \"imu_hz\": 200,\n"
      << "  \"uwb_hz\": 20,\n"
      << "  \"epochs\": " << epochs << ",\n"
      << "  \"anchors\": " << anchors << ",\n"
      << "  \"uwb_ramp_enabled\": " << std::boolalpha << s.ramp_enabled << ",\n"
      << "  \"initial_velocity_mps\": ["
      << (s.zero_initial_velocity ? 0.0 : kTruthVelocity.x()) << ", "
      << (s.zero_initial_velocity ? 0.0 : kTruthVelocity.y()) << ", 0.0],\n"
      << "  \"truth_velocity_mps\": [0.45, 0.45, 0.0],\n"
      << "  \"fault_epoch_begin\": " << s.fault_begin << ",\n"
      << "  \"fault_epoch_end\": " << s.fault_end << ",\n"
      << "  \"uwb_fault_bias_m\": " << uwb_fault_m << ",\n"
      << "  \"accel_x_fault_mps2\": " << accel_fault_mps2 << ",\n"
      << "  \"uwb_fault_anchor\": " << uwb_fault_anchor << ",\n"
      << "  \"accel_fault_axis\": " << accel_fault_axis << ",\n"
      << "  \"development_only\": true,\n"
      << "  \"formal_eligible\": false\n}\n";
}

std::string join(const std::set<std::string>& values) {
  std::ostringstream out;
  for (const auto& value : values) {
    if (out.tellp() > 0) out << '|';
    out << value;
  }
  return out.str();
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 5 && argc != 6) {
    std::cerr << "usage: r0_r1_development CONFIG OUTPUT_DIR EPOCHS SCENARIO [SCENARIO_YAML]\n";
    return 2;
  }
  try {
    auto config = uwb_imu_pl::IntegrityConfigLoader::load(argv[1]);
    const int epochs = std::stoi(argv[3]);
    const std::string scenario_manifest = argc == 6 ? argv[5] :
        (boost::filesystem::path(argv[1]).parent_path() /
         "r0_r1_development_scenarios.yaml").string();
    Scenario scenario = parseScenario(argv[4], scenario_manifest);
    if (epochs <= 0 || (scenario.mature && epochs < 226)) {
      throw std::invalid_argument("scenario epoch count is too short");
    }
    config.seed = kSeed;
    const double uwb_fault_m = environmentDouble(
        "UWB_IMU_PL_DEV_UWB_BIAS_M", scenario.uwb_fault_m);
    const double accel_fault_mps2 = environmentDouble(
        "UWB_IMU_PL_DEV_ACCEL_X_MPS2", scenario.accel_fault_mps2);
    const double gyro_fault_radps = environmentDouble(
        "UWB_IMU_PL_DEV_GYRO_RADPS", scenario.gyro_fault_radps);
    const bool use_gyro_fault = gyro_fault_radps != 0.0;
    const int accel_fault_axis = static_cast<int>(environmentDouble(
        "UWB_IMU_PL_DEV_ACCEL_AXIS", 0.0));
    if (accel_fault_axis < 0 || accel_fault_axis > 2) {
      throw std::invalid_argument("UWB_IMU_PL_DEV_ACCEL_AXIS must be 0, 1, or 2");
    }
    const std::uint64_t uwb_fault_anchor = static_cast<std::uint64_t>(
        environmentDouble("UWB_IMU_PL_DEV_UWB_ANCHOR",
                          config.anchors.front().id.value()));
    scenario.fault_begin = static_cast<int>(environmentDouble(
        "UWB_IMU_PL_DEV_FAULT_BEGIN", scenario.fault_begin));
    scenario.fault_end = static_cast<int>(environmentDouble(
        "UWB_IMU_PL_DEV_FAULT_END", scenario.fault_end));
    config.fault_models.uwb.ramp_bias = scenario.ramp_enabled;
    config.realtime.lever_arm_body_m = {0.30, 0.20, 0.15};
    if (scenario.low_redundancy && config.anchors.size() > 4) {
      config.anchors.resize(4);
    }
    // Necessary numerical/candidate records remain; expensive snapshot/string
    // formatting is explicitly disabled for this development runner.
    const bool full_audit =
        std::getenv("UWB_IMU_PL_DEVELOPMENT_FULL_AUDIT") != nullptr;
    config.output.write_factor_ledger = full_audit;
    config.output.write_hypothesis_evidence = full_audit;
    config.output.write_health = full_audit;
    config.resolved_yaml += "\n# development-only runtime overrides\n"
        "development_scenario: " + scenario.name +
        "\ndevelopment_scenario_manifest: " + scenario_manifest +
        "\ndevelopment_seed: 20260901\ndevelopment_formal_eligible: false\n"
        "development_lever_arm_body_m: [0.30, 0.20, 0.15]\n"
        "development_yaw_rate_radps: 0.20\n"
        "development_uwb_fault_bias_m: " + std::to_string(uwb_fault_m) + "\n"
        "development_accel_x_fault_mps2: " +
        std::to_string(accel_fault_mps2) + "\n"
        "development_accel_fault_axis: " +
        std::to_string(accel_fault_axis) + "\n"
        "development_gyro_fault_radps: " +
        std::to_string(gyro_fault_radps) + "\n"
        "development_uwb_fault_anchor: " +
        std::to_string(uwb_fault_anchor) + "\n"
        "development_anchor_count: " + std::to_string(config.anchors.size()) +
        "\n"
        "development_uwb_ramp_enabled: " +
        std::string(scenario.ramp_enabled ? "true\n" : "false\n");
    config.config_hash =
        uwb_imu_pl::IntegrityConfigLoader::hashResolvedYaml(
            config.resolved_yaml);

    const std::string output_directory = argv[2];
    uwb_imu_pl::RunLogger logger(output_directory, false, true);
    logger.writeResolvedConfig(config.resolved_yaml);
    const std::string execution_command =
        uwb_imu_pl::bindExecutionCommandArguments(
            std::string(argv[0]) + " " + argv[1] + " " + argv[2] + " " +
                argv[3] + " " + argv[4] + " " + scenario_manifest,
            {{"config_path", argv[1]},
             {"run_directory", output_directory},
             {"fde_profile",
              uwb_imu_pl::toString(config.resolved_scope.profile)},
             {"fixed_lag_epochs",
              std::to_string(config.incremental.fixed_lag_epochs)},
             {"seed", std::to_string(kSeed)},
             {"trajectory", "constant_velocity_yaw"},
             {"fault_mode", scenario.name},
             {"fault_anchor_id", std::to_string(uwb_fault_anchor)},
             {"fault_magnitude_m", std::to_string(uwb_fault_m)},
             {"accel_fault_mps2", std::to_string(accel_fault_mps2)},
             {"gyro_fault_radps", std::to_string(gyro_fault_radps)},
             {"fault_axis", std::to_string(accel_fault_axis)},
             {"packet_loss_prob", "0"},
             {"nlos_probability", "0"},
             {"enable_run_logging", "true"},
             {"write_residuals", "false"},
             {"write_timing", "true"},
             {"write_global_diagnostics",
              config.output.write_global_diagnostics ? "true" : "false"},
             {"output_root", config.output.root}});
    auto manifest = uwb_imu_pl::makeRunManifest(
        config, UWB_IMU_PL_GIT_SHA, UWB_IMU_PL_GIT_DIRTY != 0,
        execution_command);
    manifest.seed = kSeed;
    manifest.maturity = "DEVELOPMENT_ONLY / IMPLEMENTED_UNVERIFIED";
    manifest.formal_eligible = false;
    logger.writeManifest(manifest);
    writeScenarioManifest(output_directory, scenario, epochs,
                          config.anchors.size(), uwb_fault_m,
                          accel_fault_mps2, uwb_fault_anchor,
                          accel_fault_axis);

    uwb_imu_pl::NavigationState initial;
    initial.timestamp = uwb_imu_pl::TimestampNs(0);
    initial.position_world_m = kInitialPosition;
    initial.velocity_world_mps = scenario.zero_initial_velocity
        ? Eigen::Vector3d::Zero() : kTruthVelocity;
    uwb_imu_pl::IncrementalUwbImuEstimator estimator(
        config, config.realtime.lever_arm_body_m);
    estimator.initialize(initial, config.realtime.prior_sigmas);
    uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
        &estimator, uwb_imu_pl::IntegrityMonitor(
            config.risk, config.snapshot.rank_tolerance,
            config.snapshot.max_condition_number),
        // Offline replay harness: wall-clock deadlines are not a real-time
        // reference here, so the wall-timeout judgement is disabled by
        // construction; freeze/vintage checks stay active.
        uwb_imu_pl::offlineReplayPublicationLimits());
    uwb_imu_pl::ImuMeasurement boundary;
    boundary.timestamp = initial.timestamp;
    boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
    pipeline.ingestImu(boundary);

    uwb_imu_pl::NumericalWorkCounters::reset();
    uwb_imu_pl::RunSummary summary;
    summary.status = "DEVELOPMENT_ONLY / IMPLEMENTED_UNVERIFIED";
    std::set<std::string> selected_types;
    std::uint64_t finite_pl = 0, within_alert = 0, detector_alarms = 0;
    std::uint64_t actual_imu_injections = 0, actual_uwb_injections = 0;
    std::uint64_t formal_eligible = 0, reinitialization_requests = 0;
    std::uint64_t kernel_evaluations = 0, pl_evaluations = 0;
    std::uint64_t epochs_with_128_kernels = 0;
    std::uint64_t max_raw_imu_samples = 0, max_consecutive_rejections = 0;
    std::uint64_t max_hypothesis_count = 0;
    std::uint64_t max_single_uwb_hypotheses = 0;
    std::uint64_t max_single_accel_hypotheses = 0;
    std::uint64_t max_single_gyro_hypotheses = 0;
    std::uint64_t max_double_uwb_accel_hypotheses = 0;
    std::uint64_t max_double_uwb_gyro_hypotheses = 0;
    std::uint32_t max_effective_fault_cardinality = 0;
    double max_pending_duration_s = 0.0, max_state_age_s = 0.0;

    for (int index = 0; index < epochs; ++index) {
      const int epoch = index + 1;
      const double time = epoch * 0.05;
      const bool fault_active = scenario.fault_begin > 0 &&
          epoch >= scenario.fault_begin && epoch <= scenario.fault_end;
      for (int sample = 1; sample <= 10; ++sample) {
        const double sample_time = index * 0.05 + sample * 0.005;
        uwb_imu_pl::ImuMeasurement imu;
        imu.id = uwb_imu_pl::MeasurementId(index * 10 + sample);
        imu.timestamp = uwb_imu_pl::TimestampNs(
            static_cast<std::int64_t>(std::llround(sample_time * 1e9)));
        imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
        imu.angular_velocity_radps = {0, 0, kYawRate};
        if (fault_active && scenario.imu_fault) {
          if (use_gyro_fault) {
            imu.angular_velocity_radps(accel_fault_axis) += gyro_fault_radps;
          } else {
            imu.specific_force_mps2(accel_fault_axis) += accel_fault_mps2;
          }
          ++actual_imu_injections;
          uwb_imu_pl::FaultTruthRecord truth;
          truth.timestamp = imu.timestamp; truth.sequence = actual_imu_injections;
          truth.sensor_type = use_gyro_fault ? "IMU_GYRO" : "IMU_ACCEL";
          truth.fault_kind = "interval_constant";
          truth.axis = accel_fault_axis; truth.active = true; truth.epoch_begin = epoch;
          truth.epoch_end = epoch;
          truth.injected_value = use_gyro_fault ? gyro_fault_radps
                                                : accel_fault_mps2;
          truth.injected_units = use_gyro_fault ? "rad/s" : "m/s^2";
          logger.writeFaultTruth(truth);
        }
        pipeline.ingestImu(imu);
      }

      uwb_imu_pl::UwbBatch batch;
      batch.id = uwb_imu_pl::BatchId(epoch);
      batch.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>(std::llround(time * 1e9)));
      batch.covariance_model_id = "r0_r1_development_diagonal";
      for (const auto& anchor : config.anchors) {
        uwb_imu_pl::UwbMeasurement measurement;
        measurement.id = uwb_imu_pl::MeasurementId(
            static_cast<std::uint64_t>(epoch) * 100 + anchor.id.value());
        measurement.factor_id = uwb_imu_pl::FactorId(measurement.id.value());
        measurement.anchor_id = anchor.id; measurement.timestamp = batch.timestamp;
        measurement.anchor_position_m = anchor.position_world_m;
        measurement.sigma_m = config.realtime.range_sigma_m;
        const Eigen::Vector3d tag_position = truthPosition(time) +
            truthOrientation(time) * config.realtime.lever_arm_body_m;
        const double true_range =
            (tag_position - anchor.position_world_m).norm();
        // The development baseline uses the mean raw stream. Noise remains in
        // the frozen covariance model; stochastic campaigns are out of scope.
        measurement.range_m = true_range;
        const bool inject_uwb = fault_active &&
            (scenario.uwb_fault || scenario.force_rejection) &&
            (scenario.force_rejection || anchor.id.value() == uwb_fault_anchor);
        if (inject_uwb) {
          const double bias = uwb_fault_m;
          measurement.range_m += bias; ++actual_uwb_injections;
          uwb_imu_pl::FaultTruthRecord truth;
          truth.timestamp = batch.timestamp; truth.sequence = actual_uwb_injections;
          truth.anchor_id = anchor.id; truth.fault_mode = "RAW_RANGE_BIAS";
          truth.active = true; truth.injected_bias_m = bias;
          truth.true_range_m = true_range; truth.epoch_begin = epoch;
          truth.epoch_end = epoch; truth.injected_value = bias;
          logger.writeFaultTruth(truth);
        }
        batch.measurements.push_back(measurement);
      }

      const std::uint64_t marginalizations_before = estimator.marginalizationCount();
      const auto core_start = std::chrono::steady_clock::now();
      auto output = pipeline.processUwbBatch(batch);
      const double core_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - core_start).count();
      const auto outer_start = std::chrono::steady_clock::now();
      logger.writeState(output.state); logger.writeIntegrity(output);
      logger.writeGroundTruth({batch.timestamp, truthPosition(time),
                               truthOrientation(time)});
      logger.writeEvent(batch.timestamp,
          output.batch_committed ? "UWB_COMMIT" : "UWB_REJECT",
          output.fde_status + ":" + output.detector.reason);
      if (estimator.marginalizationCount() > marginalizations_before) {
        logger.writeEvent(batch.timestamp, "FIXED_LAG_MARGINALIZE",
                          "real pipeline marginalization");
      }
      logger.flush();
      const auto flush_complete = std::chrono::steady_clock::now();
      const double logging_flush_ms = std::chrono::duration<double, std::milli>(
          flush_complete - outer_start).count();
      const double outer_epoch_ms = std::chrono::duration<double, std::milli>(
          flush_complete - core_start).count();
      uwb_imu_pl::TimingRecord timing;
      timing.input_attempt_id = output.diagnostics.input_attempt_id;
      timing.transaction_id = output.transaction_id; timing.window_id = output.window_id;
      timing.timestamp = batch.timestamp; timing.epoch = estimator.currentEpoch();
      timing.stage = "core_total"; timing.wall_ms = core_ms;
      timing.problem_size = batch.measurements.size();
      timing.hypothesis_count = output.diagnostics.hypothesis_count;
      timing.factor_count = estimator.factorCount(); timing.cold = epoch <= 100;
      logger.writeTiming(timing);
      timing.stage = "outer_logging_flush"; timing.wall_ms = logging_flush_ms;
      logger.writeTiming(timing);
      timing.stage = "outer_epoch"; timing.wall_ms = outer_epoch_ms;
      logger.writeTiming(timing);
      logger.flush();

      summary.processed++; summary.committed += output.batch_committed;
      summary.rejected += !output.batch_committed;
      summary.core_total_ms += core_ms;
      summary.end_to_end_total_ms += outer_epoch_ms;
      detector_alarms += output.detector.numerically_valid && !output.detector.passed;
      formal_eligible += output.protection_level.formal_eligible;
      reinitialization_requests += output.controlled_reinitialization_required;
      kernel_evaluations += output.diagnostics.kernel_evaluated_actions;
      pl_evaluations += output.diagnostics.pl_evaluated_actions;
      epochs_with_128_kernels +=
          output.diagnostics.kernel_evaluated_actions == 128;
      max_raw_imu_samples = std::max(max_raw_imu_samples,
          output.diagnostics.raw_imu_samples);
      max_consecutive_rejections = std::max(max_consecutive_rejections,
          output.diagnostics.consecutive_rejections);
      max_hypothesis_count = std::max(max_hypothesis_count,
          output.diagnostics.hypothesis_count);
      max_single_uwb_hypotheses = std::max(max_single_uwb_hypotheses,
          output.diagnostics.single_uwb_hypotheses);
      max_single_accel_hypotheses = std::max(max_single_accel_hypotheses,
          output.diagnostics.single_accel_hypotheses);
      max_single_gyro_hypotheses = std::max(max_single_gyro_hypotheses,
          output.diagnostics.single_gyro_hypotheses);
      max_double_uwb_accel_hypotheses =
          std::max(max_double_uwb_accel_hypotheses,
                   output.diagnostics.double_uwb_accel_hypotheses);
      max_double_uwb_gyro_hypotheses =
          std::max(max_double_uwb_gyro_hypotheses,
                   output.diagnostics.double_uwb_gyro_hypotheses);
      max_effective_fault_cardinality =
          std::max(max_effective_fault_cardinality,
                   output.diagnostics.effective_fault_cardinality);
      max_pending_duration_s = std::max(max_pending_duration_s,
          output.diagnostics.pending_duration_s);
      max_state_age_s = std::max(max_state_age_s,
          output.diagnostics.state_age_s);
      if (std::isfinite(output.protection_level.hpl_m) &&
          std::isfinite(output.protection_level.vpl_m)) ++finite_pl;
      if (output.protection_level.hpl_m <= config.risk_v2.horizontal_alert_limit_m &&
          output.protection_level.vpl_m <= config.risk_v2.vertical_alert_limit_m) {
        ++within_alert;
      }
      if (output.selected_action_id) selected_types.insert(output.selected_action_type);
    }
    const auto work = uwb_imu_pl::NumericalWorkCounters::snapshot();
    summary.detail = "scenario=" + scenario.name +
        ";formal_eligible=false;alarms=" + std::to_string(detector_alarms) +
        ";single_faults_enabled=" +
            std::to_string(config.fault_models.single_faults_enabled) +
        ";double_faults_enabled=" +
            std::to_string(config.fault_models.double_faults_enabled) +
        ";supported_fault_cardinality=" +
            std::to_string(config.fault_models.max_cardinality) +
        ";effective_fault_cardinality=" +
            std::to_string(max_effective_fault_cardinality) +
        ";max_exclusion_cardinality=" +
            std::to_string(config.fde.max_exclusion_cardinality) +
        ";max_hypotheses=" + std::to_string(max_hypothesis_count) +
        ";single_uwb_hypotheses=" +
            std::to_string(max_single_uwb_hypotheses) +
        ";single_accel_hypotheses=" +
            std::to_string(max_single_accel_hypotheses) +
        ";single_gyro_hypotheses=" +
            std::to_string(max_single_gyro_hypotheses) +
        ";double_uwb_accel_hypotheses=" +
            std::to_string(max_double_uwb_accel_hypotheses) +
        ";double_uwb_gyro_hypotheses=" +
            std::to_string(max_double_uwb_gyro_hypotheses) +
        ";finite_pl=" + std::to_string(finite_pl) +
        ";within_alert=" + std::to_string(within_alert) +
        ";formal_eligible=" + std::to_string(formal_eligible) +
        ";imu_raw_injections=" + std::to_string(actual_imu_injections) +
        ";uwb_raw_injections=" + std::to_string(actual_uwb_injections) +
        ";selected_types=" + join(selected_types) +
        ";kernel_evaluations=" + std::to_string(kernel_evaluations) +
        ";pl_evaluations=" + std::to_string(pl_evaluations) +
        ";epochs_with_128_kernels=" +
            std::to_string(epochs_with_128_kernels) +
        ";max_pending_duration_s=" +
            std::to_string(max_pending_duration_s) +
        ";max_state_age_s=" + std::to_string(max_state_age_s) +
        ";max_raw_imu_samples=" + std::to_string(max_raw_imu_samples) +
        ";max_consecutive_rejections=" +
            std::to_string(max_consecutive_rejections) +
        ";reinitialization_requests=" +
            std::to_string(reinitialization_requests) +
        ";marginalizations=" + std::to_string(estimator.marginalizationCount()) +
        ";base_svd=" + std::to_string(work.base_svd) +
        ";base_llt=" + std::to_string(work.base_llt) +
        ";base_state_solves=" + std::to_string(work.base_state_solves) +
        ";llt_state_solve_calls=" +
            std::to_string(work.llt_state_solve_calls) +
        ";svd_state_solve_calls=" +
            std::to_string(work.svd_state_solve_calls) +
        ";detector_reference_qr=" + std::to_string(work.detector_reference_qr) +
        ";candidate_reference_svd=" +
            std::to_string(work.candidate_reference_svd) +
        ";candidate_inner_llt=" +
            std::to_string(work.candidate_inner_llt) +
        ";fault_gram_eigen=" + std::to_string(work.fault_gram_eigen) +
        ";fault_gram_svd=" + std::to_string(work.fault_gram_svd) +
        ";fault_gram_ldlt=" + std::to_string(work.fault_gram_ldlt) +
        ";low_dim_fault_gram=" + std::to_string(work.low_dim_fault_gram) +
        ";generic_fault_gram_fallback=" +
            std::to_string(work.generic_fault_gram_fallback) +
        ";hypothesis_parallel_blocks=" +
            std::to_string(work.hypothesis_parallel_blocks) +
        ";hypothesis_shared_hits=" +
            std::to_string(work.hypothesis_shared_hits) +
        ";hypothesis_shared_misses=" +
            std::to_string(work.hypothesis_shared_misses) +
        ";covariance_rhs_solves=" +
            std::to_string(work.covariance_rhs_solves) +
        ";covariance_rhs_columns=" +
            std::to_string(work.covariance_rhs_columns) +
        ";spectral_rhs_solves=" +
            std::to_string(work.spectral_rhs_solves) +
        ";spectral_rhs_columns=" +
            std::to_string(work.spectral_rhs_columns) +
        ";numerical_contract_mismatches=" +
            std::to_string(work.numerical_contract_mismatches) +
        ";oracle_reintegrations=" + std::to_string(work.imu_oracle_reintegrations) +
        ";square_root_factorizations=" +
            std::to_string(work.square_root_factorizations) +
        ";square_root_information_solves=" +
            std::to_string(work.square_root_information_solves) +
        ";square_root_information_columns=" +
            std::to_string(work.square_root_information_columns) +
        ";square_root_qt_applications=" +
            std::to_string(work.square_root_qt_applications) +
        ";square_root_qt_columns=" +
            std::to_string(work.square_root_qt_columns) +
        ";square_root_symbolic_hits=" +
            std::to_string(work.square_root_symbolic_hits) +
        ";square_root_symbolic_misses=" +
            std::to_string(work.square_root_symbolic_misses) +
        ";square_root_fallbacks=" +
            std::to_string(work.square_root_fallbacks) +
        ";square_root_certificate_holds=" +
            std::to_string(work.square_root_certificate_holds) +
        ";fault_mode_columns=" + std::to_string(work.fault_mode_columns) +
        ";fault_cross_blocks=" + std::to_string(work.fault_cross_blocks) +
        ";fault_cross_block_cache_hits=" +
            std::to_string(work.fault_cross_block_cache_hits) +
        ";all_mode_gram_columns=" + std::to_string(work.all_mode_gram_columns) +
        ";mode_dense_allocations=" + std::to_string(work.mode_dense_allocations) +
        ";mode_dense_allocation_rows=" +
            std::to_string(work.mode_dense_allocation_rows) +
        ";mode_dense_allocation_columns=" +
            std::to_string(work.mode_dense_allocation_columns) +
        ";compact_mode_rows=" + std::to_string(work.compact_mode_rows) +
        ";compact_mode_columns=" + std::to_string(work.compact_mode_columns) +
        ";compact_padded_equivalent_rows=" +
            std::to_string(work.compact_padded_equivalent_rows) +
        ";compact_capacity_fallbacks=" +
            std::to_string(work.compact_capacity_fallbacks) +
        ";hypothesis_capacity_refusals=" +
            std::to_string(work.hypothesis_capacity_refusals) +
        ";action_entities_constructed=" +
            std::to_string(work.action_entities_constructed) +
        ";action_entities_deferred=" +
            std::to_string(work.action_entities_deferred) +
        ";bridge_blocks_built=" + std::to_string(work.bridge_blocks_built) +
        ";candidate_graph_built=" +
            std::to_string(work.candidate_graph_built) +
        ";evidence_calls_fault_path=" +
            std::to_string(work.evidence_calls_fault_path) +
        ";evidence_calls_health_path=" +
            std::to_string(work.evidence_calls_health_path);
    logger.writeSummary(summary);
    std::cout << summary.detail << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}
