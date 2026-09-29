#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#ifndef UWB_IMU_PL_GIT_SHA
#define UWB_IMU_PL_GIT_SHA "unknown"
#endif
#ifndef UWB_IMU_PL_GIT_DIRTY
#define UWB_IMU_PL_GIT_DIRTY 1
#endif

namespace {
constexpr double kImuDt = 0.005;
constexpr double kPi = 3.14159265358979323846;
// The configured covariance is an integrity overbound.  The frozen nominal
// campaign samples inside that envelope rather than assuming equality.
constexpr double kPhysicalNoiseFraction = 0.05;
constexpr double kGyroPhysicalNoiseFraction = 0.01;

Eigen::Vector3d position(double time, bool degenerate = false) {
  if (degenerate) return {0.0, 0.0, 1.2};
  return {2.0*std::sin(.25*time), 1.5*std::sin(.37*time),
          1.2 + .5*std::sin(.19*time)};
}
Eigen::Vector3d velocity(double time, bool degenerate = false) {
  if (degenerate) return Eigen::Vector3d::Zero();
  return {.5*std::cos(.25*time), .555*std::cos(.37*time),
          .095*std::cos(.19*time)};
}
Eigen::Vector3d acceleration(double time, bool degenerate = false) {
  if (degenerate) return Eigen::Vector3d::Zero();
  return {-.125*std::sin(.25*time), -.20535*std::sin(.37*time),
          -.01805*std::sin(.19*time)};
}
Eigen::Quaterniond orientation(double time, bool degenerate = false) {
  if (degenerate) return Eigen::Quaterniond::Identity();
  const double roll = .12*std::sin(.31*time);
  const double pitch = .10*std::sin(.27*time);
  const double yaw = .35*std::sin(.21*time) + .08*time;
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}
Eigen::Vector3d angularVelocity(double time, bool degenerate = false) {
  if (degenerate) return Eigen::Vector3d::Zero();
  constexpr double epsilon = 1e-6;
  Eigen::Quaterniond delta =
      orientation(time).conjugate() * orientation(time + epsilon);
  delta.normalize();
  Eigen::AngleAxisd axis_angle(delta);
  return axis_angle.axis() * axis_angle.angle() / epsilon;
}

std::uint64_t environmentSeed(std::uint64_t fallback) {
  const char* text = std::getenv("UWB_IMU_PL_BENCHMARK_SEED");
  if (!text) return fallback;
  std::size_t used = 0;
  const auto value = std::stoull(text, &used);
  if (text[used] != '\0') throw std::invalid_argument("invalid benchmark seed");
  return value;
}

struct Scenario {
  std::string name = "nominal";
  bool noiseless = false;
  bool initial_error = false;
  bool degenerate = false;
  bool uwb = false;
  bool imu_accel = false;
  bool imu_gyro = false;
  bool persistent_reject = false;
  bool historical = false;
  bool single_epoch = false;
  double uwb_bias_m = 1.0;
  double accel_bias_mps2 = .8;
  double gyro_bias_radps = .12;
  int axis = 0;
};

Scenario scenarioFromEnvironment() {
  Scenario result;
  if (const char* text = std::getenv("UWB_IMU_PL_SCENARIO")) result.name = text;
  auto overrides = [](Scenario value) {
    auto load = [](const char* name, double fallback) {
      const char* text = std::getenv(name);
      if (!text) return fallback;
      std::size_t used = 0;
      const double parsed = std::stod(text, &used);
      if (text[used] != '\0' || !std::isfinite(parsed))
        throw std::invalid_argument(std::string("invalid ") + name);
      return parsed;
    };
    value.uwb_bias_m = load("UWB_IMU_PL_UWB_BIAS_M", value.uwb_bias_m);
    value.accel_bias_mps2 = load("UWB_IMU_PL_ACCEL_BIAS_MPS2", value.accel_bias_mps2);
    value.gyro_bias_radps = load("UWB_IMU_PL_GYRO_BIAS_RADPS", value.gyro_bias_radps);
    return value;
  };
  if (result.name == "nominal") return overrides(result);
  if (result.name == "noiseless") { result.noiseless = true; return overrides(result); }
  if (result.name == "initial_error") { result.initial_error = true; return overrides(result); }
  if (result.name == "degenerate") { result.degenerate = true; return overrides(result); }
  if (result.name == "uwb_fault" || result.name == "history_uwb") {
    result.uwb = true; result.historical = result.name == "history_uwb";
    return overrides(result);
  }
  if (result.name == "uwb_recovery") {
    result.uwb = true; result.single_epoch = true; result.uwb_bias_m = 2.25;
    return overrides(result);
  }
  if (result.name == "imu_recovery") {
    result.imu_accel = true; result.single_epoch = true;
    result.accel_bias_mps2 = 20.0; return overrides(result);
  }
  if (result.name == "joint_recovery") {
    result.uwb = true; result.imu_accel = true; result.single_epoch = true;
    result.uwb_bias_m = 2.25; result.accel_bias_mps2 = 20.0;
    return overrides(result);
  }
  if (result.name == "persistent_reject") {
    result.uwb = true; result.persistent_reject = true; return overrides(result);
  }
  if (result.name == "joint_fault") {
    result.uwb = true; result.imu_accel = true; result.axis = 0;
    return overrides(result);
  }
  const std::string accel_prefix = "imu_accel_";
  const std::string gyro_prefix = "imu_gyro_";
  if (result.name.compare(0, accel_prefix.size(), accel_prefix) == 0 ||
      result.name.compare(0, gyro_prefix.size(), gyro_prefix) == 0) {
    result.imu_accel = result.name.compare(0, accel_prefix.size(), accel_prefix) == 0;
    result.imu_gyro = !result.imu_accel;
    const char axis = result.name.back();
    if (axis < 'x' || axis > 'z') throw std::invalid_argument("invalid IMU fault axis");
    result.axis = axis - 'x';
    return overrides(result);
  }
  throw std::invalid_argument("unknown UWB_IMU_PL_SCENARIO: " + result.name);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: realtime_performance_benchmark CONFIG OUTPUT_DIR EPOCHS\n";
    return 2;
  }
  try {
    auto config = uwb_imu_pl::IntegrityConfigLoader::load(argv[1]);
    const Scenario scenario = scenarioFromEnvironment();
    config.seed = environmentSeed(config.seed);
    // This executable historically discarded these records after paying their
    // formatting cost. Preserve its external log mode while disabling their
    // creation at the source; numerical risk/action summaries remain.
    config.output.write_hypothesis_evidence = false;
    config.output.write_health = false;
    config.output.write_factor_ledger = false;
    config.resolved_yaml +=
        "\n# realtime_performance_benchmark runtime output mode\n"
        "benchmark_write_hypothesis_evidence: false\n"
        "benchmark_write_health: false\n"
        "benchmark_write_factor_ledger: false\n"
        "benchmark_scenario: " + scenario.name + "\n"
        "benchmark_effective_seed: " + std::to_string(config.seed) + "\n"
        "benchmark_uwb_bias_m: " + std::to_string(scenario.uwb_bias_m) + "\n"
        "benchmark_accel_bias_mps2: " +
            std::to_string(scenario.accel_bias_mps2) + "\n"
        "benchmark_gyro_bias_radps: " +
            std::to_string(scenario.gyro_bias_radps) + "\n";
    config.config_hash =
        uwb_imu_pl::IntegrityConfigLoader::hashResolvedYaml(
            config.resolved_yaml);
    const int epochs = std::stoi(argv[3]);
    if (epochs <= 0 || config.incremental.fixed_lag_epochs <=
                           config.integrity_window.epochs +
                               config.integrity_window.recovery_margin_epochs ||
        !config.output.write_timing || config.output.write_global_diagnostics) {
      throw std::runtime_error("performance config/epoch contract mismatch");
    }
    uwb_imu_pl::RunLogger logger(argv[2], false, true);
    logger.writeResolvedConfig(config.resolved_yaml);
    const std::string execution_command =
        uwb_imu_pl::bindExecutionCommandArguments(
            std::string(argv[0]) + " " + argv[1] + " " + argv[2] + " " +
                argv[3],
            {{"config_path", argv[1]},
             {"run_directory", argv[2]},
             {"fde_profile",
              uwb_imu_pl::toString(config.resolved_scope.profile)},
             {"fixed_lag_epochs",
              std::to_string(config.incremental.fixed_lag_epochs)},
             {"seed", std::to_string(config.seed)},
             {"trajectory", scenario.degenerate ? "static" : "benchmark_3d"},
             {"fault_mode", scenario.name},
             {"fault_anchor_id", "1"},
             {"fault_magnitude_m", std::to_string(scenario.uwb_bias_m)},
             {"accel_fault_mps2",
              std::to_string(scenario.accel_bias_mps2)},
             {"gyro_fault_radps",
              std::to_string(scenario.gyro_bias_radps)},
             {"fault_axis", std::to_string(scenario.axis)},
             {"packet_loss_prob", "0"},
             {"nlos_probability", "0"},
             {"enable_run_logging", "true"},
             {"write_residuals", "false"},
             {"write_timing", "true"},
             {"write_global_diagnostics", "false"},
             {"output_root", config.output.root}});
    logger.writeManifest(uwb_imu_pl::makeRunManifest(
        config, UWB_IMU_PL_GIT_SHA, UWB_IMU_PL_GIT_DIRTY != 0,
        execution_command));
    uwb_imu_pl::NavigationState initial;
    initial.timestamp = uwb_imu_pl::TimestampNs(0);
    initial.position_world_m = position(0, scenario.degenerate);
    initial.velocity_world_mps = velocity(0, scenario.degenerate);
    initial.q_world_body = orientation(0, scenario.degenerate);
    if (scenario.initial_error) {
      initial.position_world_m += Eigen::Vector3d(.2, -.2, .2);
      initial.velocity_world_mps += Eigen::Vector3d(.2, -.2, .2);
      initial.q_world_body = initial.q_world_body * Eigen::Quaterniond(
          Eigen::AngleAxisd(5.0*kPi/180.0, Eigen::Vector3d(1, 1, 1).normalized()));
    }
    uwb_imu_pl::IncrementalUwbImuEstimator estimator(
        config, config.realtime.lever_arm_body_m);
    estimator.initialize(initial, config.realtime.prior_sigmas);
  // D round: offline replay declares its publication policy explicitly (the
  // default wall timeout is a bridge/streaming policy; this harness computes
  // slower than real time by construction).
  uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          config.risk, config.snapshot.rank_tolerance,
          config.snapshot.max_condition_number),
      uwb_imu_pl::offlineReplayPublicationLimits());
    uwb_imu_pl::ImuMeasurement boundary;
    boundary.timestamp = initial.timestamp;
    boundary.specific_force_mps2 = orientation(0, scenario.degenerate).conjugate() *
        (acceleration(0, scenario.degenerate) +
         Eigen::Vector3d(0, 0, config.imu.gravity_mps2));
    boundary.angular_velocity_radps = angularVelocity(0, scenario.degenerate);
    pipeline.ingestImu(boundary);
    std::mt19937_64 random(config.seed);
    std::normal_distribution<double> normal;
    uwb_imu_pl::RunSummary summary;
    uwb_imu_pl::NumericalWorkCounters::reset();
    std::vector<double> candidate_wall_ms;
    const bool force_candidate_stress =
        std::getenv("UWB_IMU_PL_BENCHMARK_FORCE_ALARM") != nullptr;
    const int fault_begin = scenario.historical ? 5 : std::max(5, epochs / 3);
    const int fault_end = scenario.single_epoch ? fault_begin :
        (scenario.historical ? std::max(5, epochs - 25) :
                               std::max(fault_begin, 2*epochs/3-1));
    std::uint64_t fault_sequence = 0;
    summary.status = "IMPLEMENTED_UNVERIFIED";
    for (int epoch_index = 0; epoch_index < epochs; ++epoch_index) {
      const double time = (epoch_index+1)*.05;
      for (int sample = 1; sample <= 10; ++sample) {
        const double sample_time = epoch_index*.05 + sample*.005;
        uwb_imu_pl::ImuMeasurement imu;
        imu.id = uwb_imu_pl::MeasurementId(epoch_index*10+sample);
        imu.timestamp = uwb_imu_pl::TimestampNs(
            static_cast<std::int64_t>(std::llround(sample_time*1e9)));
        const auto truth_q = orientation(sample_time, scenario.degenerate);
        imu.specific_force_mps2 = truth_q.conjugate() *
            (acceleration(sample_time, scenario.degenerate) +
             Eigen::Vector3d(0, 0, config.imu.gravity_mps2));
        imu.angular_velocity_radps = angularVelocity(sample_time, scenario.degenerate);
        const bool fault_active = epoch_index >= fault_begin && epoch_index <= fault_end;
        if (fault_active && scenario.imu_accel)
          imu.specific_force_mps2(scenario.axis) += scenario.accel_bias_mps2;
        if (fault_active && scenario.imu_gyro)
          imu.angular_velocity_radps(scenario.axis) += scenario.gyro_bias_radps;
        for (int axis = 0; axis < 3; ++axis) {
          if (!scenario.noiseless) {
            imu.specific_force_mps2(axis) +=
                kPhysicalNoiseFraction*config.imu.accelerometer_sigma/
                std::sqrt(kImuDt)*normal(random);
            imu.angular_velocity_radps(axis) +=
                kGyroPhysicalNoiseFraction*config.imu.gyroscope_sigma/
                std::sqrt(kImuDt)*normal(random);
          }
        }
        pipeline.ingestImu(imu);
      }
      const bool fault_active = epoch_index >= fault_begin && epoch_index <= fault_end;
      if (fault_active && (scenario.imu_accel || scenario.imu_gyro)) {
        uwb_imu_pl::FaultTruthRecord truth;
        truth.timestamp = uwb_imu_pl::TimestampNs(
            static_cast<std::int64_t>(std::llround(time*1e9)));
        truth.sequence = ++fault_sequence; truth.active = true;
        truth.sensor_type = "IMU";
        truth.fault_mode = scenario.imu_accel ? "ACCEL_AXIS_BIAS" : "GYRO_AXIS_BIAS";
        truth.fault_kind = scenario.imu_accel ? "accel_axis_interval_bias" :
                                               "gyro_axis_interval_bias";
        truth.axis = scenario.axis; truth.epoch_begin = fault_begin + 1;
        truth.epoch_end = fault_end + 1;
        truth.injected_value = scenario.imu_accel ? scenario.accel_bias_mps2 :
                                                   scenario.gyro_bias_radps;
        truth.injected_units = scenario.imu_accel ? "m/s^2" : "rad/s";
        logger.writeFaultTruth(truth);
      }
      uwb_imu_pl::UwbBatch batch;
      batch.id = uwb_imu_pl::BatchId(epoch_index+1);
      batch.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>(std::llround(time*1e9)));
      batch.covariance_model_id = "week4_performance_diagonal";
      for (const auto& anchor : config.anchors) {
        uwb_imu_pl::UwbMeasurement measurement;
        measurement.id = uwb_imu_pl::MeasurementId(
            static_cast<std::uint64_t>(epoch_index+1)*100 + anchor.id.value());
        measurement.factor_id = uwb_imu_pl::FactorId(measurement.id.value());
        measurement.anchor_id = anchor.id;
        measurement.timestamp = batch.timestamp;
        measurement.anchor_position_m = anchor.position_world_m;
        measurement.sigma_m = config.realtime.range_sigma_m;
        const Eigen::Vector3d tag_position = position(time, scenario.degenerate) +
            orientation(time, scenario.degenerate) * config.realtime.lever_arm_body_m;
        const double true_range = (tag_position-anchor.position_world_m).norm();
        measurement.range_m = true_range +
            (scenario.noiseless ? 0.0 : kPhysicalNoiseFraction*
                                            measurement.sigma_m*normal(random));
        const bool inject_uwb = fault_active && scenario.uwb &&
            (scenario.persistent_reject || anchor.id == config.anchors.front().id);
        if (inject_uwb) {
          const double bias = scenario.persistent_reject ? 5.0 :
                                                            scenario.uwb_bias_m;
          measurement.range_m += bias;
          uwb_imu_pl::FaultTruthRecord truth;
          truth.timestamp = batch.timestamp; truth.sequence = ++fault_sequence;
          truth.anchor_id = anchor.id; truth.fault_mode = "RAW_RANGE_BIAS";
          truth.active = true; truth.injected_bias_m = bias;
          truth.true_range_m = true_range; truth.epoch_begin = fault_begin + 1;
          truth.epoch_end = fault_end + 1; truth.injected_value = bias;
          logger.writeFaultTruth(truth);
        }
        if (force_candidate_stress && epoch_index >= 60 &&
            anchor.id == config.anchors.front().id) {
          measurement.range_m += 5.0;
        }
        batch.measurements.push_back(measurement);
      }
      const std::uint64_t marginalizations_before = estimator.marginalizationCount();
      const auto start = std::chrono::steady_clock::now();
      uwb_imu_pl::IntegrityOutput result;
      uwb_imu_pl::AttemptProofLease proof_lease;
      try {
        result = pipeline.processUwbBatch(batch, &proof_lease);
      } catch (...) {
        logger.writeIntegrity(pipeline.lastAttemptOutput());
        throw;
      }
      const double core_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now()-start).count();
      for (auto& stage : result.stage_timings) {
        if (stage.stage == "core_total") stage.wall_ms = core_ms;
      }
      for (const auto& candidate : result.candidate_audit) {
        if (std::isfinite(candidate.wall_ms)) {
          candidate_wall_ms.push_back(candidate.wall_ms);
        }
      }
      const auto outer_start = std::chrono::steady_clock::now();
      logger.writeState(result.state);
      logger.writeIntegrity(result);
      logger.writeGroundTruth({batch.timestamp,
                               position(time, scenario.degenerate),
                               orientation(time, scenario.degenerate)});
      const std::size_t epoch = estimator.currentEpoch();
      auto timing = [&](const std::string& stage, double value, bool success=true) {
        uwb_imu_pl::TimingRecord record;
        record.input_attempt_id = result.diagnostics.input_attempt_id;
        record.transaction_id = result.transaction_id; record.window_id = result.window_id;
        record.timestamp = result.timestamp; record.epoch = epoch;
        record.stage = stage; record.wall_ms = value;
        record.problem_size = batch.measurements.size();
        record.hypothesis_count = result.sensitivities.size();
        record.factor_count = estimator.factorCount();
        record.cold = epoch_index < 100;
        record.success = success; logger.writeTiming(record);
      };
      timing("imu_preintegration", estimator.lastImuPreintegrationMs());
      timing("no_uwb_isam_update", estimator.lastNoUwbUpdateMs());
      timing("state_query", estimator.lastStateQueryMs());
      timing("current_joint_marginal", estimator.lastMarginalMs());
      timing("snapshot_extraction", estimator.lastSnapshotExtractionMs());
      for (const auto& stage : result.stage_timings) {
        if (stage.stage != "core_total" && stage.status == "EXECUTED")
          timing(stage.stage, stage.wall_ms, stage.success);
      }
      timing(result.batch_committed ? "uwb_commit" : "uwb_reject",
             estimator.lastUwbUpdateMs());
      timing("core_total", core_ms);
      logger.writeEvent(result.timestamp,
                        result.batch_committed ? "UWB_COMMIT" : "UWB_REJECT",
                        result.detector.reason);
      if (estimator.marginalizationCount() > marginalizations_before) {
        logger.writeEvent(result.timestamp, "FIXED_LAG_MARGINALIZE",
            "oldest_retained_epoch=" + std::to_string(estimator.oldestRetainedEpoch()) +
            ";retained_epochs=" + std::to_string(estimator.retainedEpochs()) +
            ";active_values=" + std::to_string(estimator.activeValueCount()) +
            ";active_factors=" + std::to_string(estimator.factorCount()));
      }
      logger.flush();
      const auto flush_complete = std::chrono::steady_clock::now();
      const double logging_flush_ms = std::chrono::duration<double, std::milli>(
          flush_complete - outer_start).count();
      const double outer_epoch_ms = std::chrono::duration<double, std::milli>(
          flush_complete - start).count();
      timing("outer_logging_flush", logging_flush_ms);
      timing("outer_epoch", outer_epoch_ms);
      logger.flush();
      summary.processed++;
      summary.committed += result.batch_committed;
      summary.rejected += !result.batch_committed;
      summary.core_total_ms += core_ms;
      summary.end_to_end_total_ms += outer_epoch_ms;
    }
    summary.detail = "deterministic 3D rotating raw-stream benchmark; scenario=" +
                     scenario.name + "; seed=" + std::to_string(config.seed);
    if (!candidate_wall_ms.empty()) {
      std::sort(candidate_wall_ms.begin(), candidate_wall_ms.end());
      auto percentile = [&](double p) {
        const std::size_t index = static_cast<std::size_t>(std::ceil(
            p * static_cast<double>(candidate_wall_ms.size()))) - 1;
        return candidate_wall_ms[std::min(index, candidate_wall_ms.size() - 1)];
      };
      std::cout << "candidate_wall_ms p50=" << percentile(0.50)
                << " p95=" << percentile(0.95)
                << " p99=" << percentile(0.99)
                << " max=" << candidate_wall_ms.back() << '\n';
    }
    logger.writeSummary(summary);
    const auto work = uwb_imu_pl::NumericalWorkCounters::snapshot();
    std::cout << "numerical_work base_svd=" << work.base_svd
              << " base_llt=" << work.base_llt
              << " base_state_solves=" << work.base_state_solves
              << " llt_state_solve_calls=" << work.llt_state_solve_calls
              << " svd_state_solve_calls=" << work.svd_state_solve_calls
              << " detector_reference_qr=" << work.detector_reference_qr
              << " candidate_reference_svd=" << work.candidate_reference_svd
              << " candidate_inner_llt=" << work.candidate_inner_llt
              << " covariance_rhs_solves=" << work.covariance_rhs_solves
              << " covariance_rhs_columns=" << work.covariance_rhs_columns
              << " spectral_rhs_solves=" << work.spectral_rhs_solves
              << " spectral_rhs_columns=" << work.spectral_rhs_columns
              << " numerical_contract_mismatches="
              << work.numerical_contract_mismatches
              << " imu_oracle_reintegrations=" << work.imu_oracle_reintegrations
              << " window_content_hash_scans="
              << work.window_content_hash_scans
              << " frozen_identity_builds=" << work.frozen_identity_builds
              << " frozen_identity_reuses=" << work.frozen_identity_reuses
              << " frozen_admission_constant_validations="
              << work.frozen_admission_constant_validations
              << " descriptor_id_lookups=" << work.descriptor_id_lookups
              << " descriptor_linear_scans="
              << work.descriptor_linear_scans
              << " block_rhs_solve_batches="
              << work.block_rhs_solve_batches
              << " block_rhs_unique_blocks="
              << work.block_rhs_unique_blocks
              << '\n';
    const auto history_root = estimator.historyRootCacheAuditForTesting();
    std::cout << "history_root_work requests=" << history_root.requests
              << " exact_hits=" << history_root.exact_hits
              << " incremental_path_updates="
              << history_root.incremental_path_updates
              << " incremental_add_paths="
              << history_root.incremental_add_paths
              << " incremental_remove_paths="
              << history_root.incremental_remove_paths
              << " incremental_relinearize_paths="
              << history_root.incremental_relinearize_paths
              << " internal_nodes_recomputed="
              << history_root.internal_nodes_recomputed
              << " full_tree_rebuilds=" << history_root.full_tree_rebuilds
              << " full_oracle_checks=" << history_root.full_oracle_checks
              << " full_oracle_mismatches="
              << history_root.full_oracle_mismatches
              << " reused_groups=" << history_root.reused_groups
              << " rebuilt_groups=" << history_root.rebuilt_groups
              << " factor_group_hits=" << history_root.factor_group_hits
              << " factor_group_misses=" << history_root.factor_group_misses
              << " retained_rows=" << history_root.retained_rows
              << " retained_bytes=" << history_root.retained_bytes
              << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}
