#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
  std::string config, output, dataset;
  std::uint64_t seed_start = 0, seed_count = 0;
  int epochs = 600;
  int fault_onset = 200;
  unsigned threads = std::max(1u, std::thread::hardware_concurrency());
  bool resume = false;
};

Options parse(int argc, char** argv) {
  if (argc < 6) throw std::runtime_error(
      "usage: sequential_detector_history CONFIG OUTPUT DATASET SEED_START SEED_COUNT "
      "[--epochs N] [--fault-onset N] [--threads N] [--resume]");
  Options result;
  result.config = argv[1]; result.output = argv[2]; result.dataset = argv[3];
  result.seed_start = std::stoull(argv[4]); result.seed_count = std::stoull(argv[5]);
  if (result.dataset != "calibration" && result.dataset != "nominal" &&
      result.dataset != "fault") throw std::runtime_error("invalid dataset");
  for (int index = 6; index < argc; ++index) {
    const std::string key = argv[index];
    if (key == "--resume") { result.resume = true; continue; }
    if (++index >= argc) throw std::runtime_error("missing option value");
    if (key == "--epochs") result.epochs = std::stoi(argv[index]);
    else if (key == "--fault-onset") result.fault_onset = std::stoi(argv[index]);
    else if (key == "--threads") result.threads = std::stoul(argv[index]);
    else throw std::runtime_error("unknown option: " + key);
  }
  if (!result.seed_count || result.epochs <= 0 || result.fault_onset < 0 ||
      result.fault_onset >= result.epochs || !result.threads) {
    throw std::runtime_error("counts must be positive");
  }
  return result;
}

std::uint64_t mixed(std::uint64_t value) {
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

Eigen::Vector3d position(const std::string& trajectory, double time) {
  if (trajectory == "straight") return {0.2*time, 0.0, 1.2};
  if (trajectory == "circle") return {3*std::cos(.15*time), 3*std::sin(.15*time), 1.2};
  return {3*std::sin(.15*time), 1.5*std::sin(.30*time), 1.2};
}

Eigen::Vector3d acceleration(const std::string& trajectory, double time) {
  if (trajectory == "straight") return Eigen::Vector3d::Zero();
  if (trajectory == "circle") return {-.0675*std::cos(.15*time),
                                       -.0675*std::sin(.15*time), 0};
  return {-.0675*std::sin(.15*time), -.135*std::sin(.30*time), 0};
}

std::string sequenceId(std::uint64_t seed, const std::string& trajectory) {
  return std::to_string(seed) + "|" + trajectory;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse(argc, argv);
    const auto config = uwb_imu_pl::IntegrityConfigLoader::load(options.config);
    if (!config.output.write_global_diagnostics) {
      throw std::runtime_error("sequential ROC config requires global diagnostics");
    }
    std::set<std::string> complete;
    const std::string checkpoint_path = options.output + ".completed";
    if (options.resume) {
      std::ifstream checkpoint(checkpoint_path);
      std::string line;
      while (std::getline(checkpoint, line)) complete.insert(line);
    }
    struct Job { std::uint64_t seed; std::string trajectory; };
    std::vector<Job> jobs;
    for (std::uint64_t offset = 0; offset < options.seed_count; ++offset) {
      for (const std::string trajectory : {"straight", "circle", "figure_eight"}) {
        if (!complete.count(sequenceId(options.seed_start+offset, trajectory))) {
          jobs.push_back({options.seed_start+offset, trajectory});
        }
      }
    }
    const bool append = options.resume && boost::filesystem::exists(options.output);
    boost::filesystem::create_directories(
        boost::filesystem::path(options.output).parent_path());
    std::ofstream output(options.output, append ? std::ios::app : std::ios::out);
    std::ofstream checkpoint(checkpoint_path, append ? std::ios::app : std::ios::out);
    if (!append) output << "seed,trajectory,graph_history,epoch,global_statistic,"
        "postfit_statistic,conditional_statistic,fault_active,fault_m,anchor_id\n";
    std::atomic<std::size_t> next{0};
    std::atomic<bool> failed{false};
    std::string worker_error;
    std::mutex write_mutex;
    auto worker = [&] {
      while (true) {
        if (failed.load()) return;
        const std::size_t index = next.fetch_add(1);
        if (index >= jobs.size()) return;
        try {
        const Job job = jobs[index];
        std::mt19937_64 random(mixed(job.seed));
        std::normal_distribution<double> normal;
        auto run_config = config;
        uwb_imu_pl::NavigationState initial;
        initial.timestamp = uwb_imu_pl::TimestampNs(0);
        initial.position_world_m = position(job.trajectory, 0);
        uwb_imu_pl::IncrementalUwbImuEstimator estimator(
            run_config, run_config.realtime.lever_arm_body_m);
        estimator.initialize(initial, run_config.realtime.prior_sigmas);
        uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
            &estimator, uwb_imu_pl::IntegrityMonitor(
                run_config.risk, run_config.snapshot.rank_tolerance,
                run_config.snapshot.max_condition_number));
        uwb_imu_pl::ImuMeasurement boundary;
        boundary.timestamp = initial.timestamp;
        boundary.specific_force_mps2 = acceleration(job.trajectory, 0) +
            Eigen::Vector3d(0, 0, run_config.imu.gravity_mps2);
        pipeline.ingestImu(boundary);
        const std::uint64_t fault_offset = job.seed - options.seed_start;
        const std::uint64_t fault_anchor = fault_offset % 8 + 1;
        const double magnitudes[] = {.1, .25, .5, 1., 2.};
        const double fault_magnitude = magnitudes[(fault_offset / 8) % 5];
        std::ostringstream rows;
        rows << std::setprecision(17);
        for (int epoch = 0; epoch < options.epochs; ++epoch) {
          const double time = (epoch+1)*.05;
          for (int sample = 1; sample <= 10; ++sample) {
            const double sample_time = epoch*.05 + sample*.005;
            uwb_imu_pl::ImuMeasurement imu;
            imu.id = uwb_imu_pl::MeasurementId(epoch*10+sample);
            imu.timestamp = uwb_imu_pl::TimestampNs(
                static_cast<std::int64_t>(std::llround(sample_time*1e9)));
            imu.specific_force_mps2 = acceleration(job.trajectory, sample_time) +
                Eigen::Vector3d(0, 0, run_config.imu.gravity_mps2);
            // Development noise uses a deterministic discrete approximation.
            for (int axis = 0; axis < 3; ++axis) {
              imu.specific_force_mps2(axis) += run_config.imu.accelerometer_sigma * normal(random);
              imu.angular_velocity_radps(axis) += run_config.imu.gyroscope_sigma * normal(random);
            }
            pipeline.ingestImu(imu);
          }
          uwb_imu_pl::UwbBatch batch;
          batch.id = uwb_imu_pl::BatchId(epoch+1);
          batch.timestamp = uwb_imu_pl::TimestampNs(
              static_cast<std::int64_t>(std::llround(time*1e9)));
          batch.covariance_model_id = "week4_sequential_diagonal";
          const bool fault_active = options.dataset == "fault" &&
              epoch >= options.fault_onset;
          for (const auto& anchor : run_config.anchors) {
            uwb_imu_pl::UwbMeasurement measurement;
            measurement.id = uwb_imu_pl::MeasurementId(
                static_cast<std::uint64_t>(epoch+1)*100 + anchor.id.value());
            measurement.factor_id = uwb_imu_pl::FactorId(measurement.id.value());
            measurement.anchor_id = anchor.id;
            measurement.timestamp = batch.timestamp;
            measurement.anchor_position_m = anchor.position_world_m;
            measurement.sigma_m = run_config.realtime.range_sigma_m;
            measurement.range_m = (position(job.trajectory, time) -
                                   anchor.position_world_m).norm() +
                measurement.sigma_m * normal(random);
            if (fault_active && anchor.id.value() == fault_anchor) {
              measurement.range_m += fault_magnitude;
            }
            batch.measurements.push_back(measurement);
          }
          const auto result = pipeline.processUwbBatch(batch);
          if (!std::isfinite(result.global_detector.statistic) ||
              !std::isfinite(result.postfit_detector.statistic) ||
              !std::isfinite(result.detector.statistic)) {
            throw std::runtime_error("nonfinite detector output");
          }
          rows << job.seed << ',' << job.trajectory << ','
               << run_config.incremental.fixed_lag_epochs << ',' << epoch << ','
               << result.global_detector.statistic << ','
               << result.postfit_detector.statistic << ','
               << result.detector.statistic << ',' << fault_active << ','
               << (fault_active ? fault_magnitude : 0.0) << ','
               << fault_anchor << '\n';
        }
        std::lock_guard<std::mutex> lock(write_mutex);
        output << rows.str();
        output.flush();
        checkpoint << sequenceId(job.seed, job.trajectory) << '\n';
        checkpoint.flush();
        } catch (const std::exception& error) {
          std::lock_guard<std::mutex> lock(write_mutex);
          failed.store(true);
          worker_error = error.what();
          return;
        }
      }
    };
    std::vector<std::thread> workers;
    for (unsigned index = 0; index < std::min<unsigned>(options.threads, jobs.size()); ++index) {
      workers.emplace_back(worker);
    }
    for (auto& thread : workers) thread.join();
    if (failed.load()) throw std::runtime_error(worker_error);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}
