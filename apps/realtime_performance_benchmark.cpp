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
Eigen::Vector3d position(double time) {
  return {3*std::sin(.15*time), 1.5*std::sin(.30*time), 1.2};
}
Eigen::Vector3d acceleration(double time) {
  return {-.0675*std::sin(.15*time), -.135*std::sin(.30*time), 0};
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: realtime_performance_benchmark CONFIG OUTPUT_DIR EPOCHS\n";
    return 2;
  }
  try {
    auto config = uwb_imu_pl::IntegrityConfigLoader::load(argv[1]);
    // This executable historically discarded these records after paying their
    // formatting cost. Preserve its external log mode while disabling their
    // creation at the source; numerical risk/action summaries remain.
    config.output.write_hypothesis_evidence = false;
    config.output.write_health = false;
    config.resolved_yaml +=
        "\n# realtime_performance_benchmark runtime output mode\n"
        "benchmark_write_hypothesis_evidence: false\n"
        "benchmark_write_health: false\n";
    config.config_hash =
        uwb_imu_pl::IntegrityConfigLoader::hashResolvedYaml(
            config.resolved_yaml);
    const int epochs = std::stoi(argv[3]);
    if (epochs <= 0 || config.incremental.fixed_lag_epochs != 200 ||
        !config.output.write_timing || config.output.write_global_diagnostics) {
      throw std::runtime_error("performance config/epoch contract mismatch");
    }
    uwb_imu_pl::RunLogger logger(argv[2], false, true);
    logger.writeResolvedConfig(config.resolved_yaml);
    logger.writeManifest(uwb_imu_pl::makeRunManifest(
        config, UWB_IMU_PL_GIT_SHA, UWB_IMU_PL_GIT_DIRTY != 0,
        std::string(argv[0]) + " " + argv[1] + " " + argv[2] + " " + argv[3]));
    uwb_imu_pl::NavigationState initial;
    initial.timestamp = uwb_imu_pl::TimestampNs(0);
    initial.position_world_m = position(0);
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
    boundary.specific_force_mps2 = acceleration(0) +
        Eigen::Vector3d(0, 0, config.imu.gravity_mps2);
    pipeline.ingestImu(boundary);
    std::mt19937_64 random(config.seed);
    std::normal_distribution<double> normal;
    uwb_imu_pl::RunSummary summary;
    uwb_imu_pl::NumericalWorkCounters::reset();
    std::vector<double> candidate_wall_ms;
    const bool force_candidate_stress =
        std::getenv("UWB_IMU_PL_BENCHMARK_FORCE_ALARM") != nullptr;
    summary.status = "IMPLEMENTED_UNVERIFIED";
    for (int epoch_index = 0; epoch_index < epochs; ++epoch_index) {
      const double time = (epoch_index+1)*.05;
      for (int sample = 1; sample <= 10; ++sample) {
        const double sample_time = epoch_index*.05 + sample*.005;
        uwb_imu_pl::ImuMeasurement imu;
        imu.id = uwb_imu_pl::MeasurementId(epoch_index*10+sample);
        imu.timestamp = uwb_imu_pl::TimestampNs(
            static_cast<std::int64_t>(std::llround(sample_time*1e9)));
        imu.specific_force_mps2 = acceleration(sample_time) +
            Eigen::Vector3d(0, 0, config.imu.gravity_mps2);
        for (int axis = 0; axis < 3; ++axis) {
          imu.specific_force_mps2(axis) += config.imu.accelerometer_sigma*normal(random);
          imu.angular_velocity_radps(axis) += config.imu.gyroscope_sigma*normal(random);
        }
        pipeline.ingestImu(imu);
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
        measurement.range_m = (position(time)-anchor.position_world_m).norm() +
            measurement.sigma_m*normal(random);
        if (force_candidate_stress && epoch_index >= 60 &&
            anchor.id == config.anchors.front().id) {
          measurement.range_m += 5.0;
        }
        batch.measurements.push_back(measurement);
      }
      const std::uint64_t marginalizations_before = estimator.marginalizationCount();
      const auto start = std::chrono::steady_clock::now();
      uwb_imu_pl::IntegrityOutput result;
      try {
        result = pipeline.processUwbBatch(batch);
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
    summary.detail = "deterministic Week-4 figure-eight performance benchmark";
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
              << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}
