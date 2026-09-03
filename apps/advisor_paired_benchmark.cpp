#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <sys/resource.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
  std::string config;
  std::string output;
  std::string dataset;
  std::uint64_t seed_start = 0;
  std::uint64_t seed_count = 0;
  int epochs = 0;
  int fault_onset = 200;
  int fault_duration_epochs = 0;
  int output_start_epoch = 0;
  int output_end_epoch = -1;
  std::vector<std::uint32_t> histories{0, 200};
  std::vector<std::string> trajectories{"straight", "circle", "figure_eight"};
  std::vector<double> fault_magnitudes{.1, .25, .5, 1., 2.};
  std::uint64_t forced_fault_anchor = 0;
  double forced_fault_magnitude = 0.0;
  unsigned threads = std::max(1u, std::thread::hardware_concurrency());
  bool resume = false;
};

std::vector<std::string> split(const std::string& value) {
  std::vector<std::string> output;
  std::istringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    if (!item.empty()) output.push_back(item);
  }
  if (output.empty()) throw std::runtime_error("empty comma-separated option");
  return output;
}

Options parse(int argc, char** argv) {
  if (argc < 8) throw std::runtime_error(
      "usage: advisor_paired_benchmark CONFIG OUTPUT DATASET SEED_START "
      "SEED_COUNT --epochs N [--histories 0,200] [--trajectories ...] "
      "[--fault-onset N] [--fault-magnitudes ...] [--fault-anchor N] "
      "[--fault-magnitude M] [--fault-duration-epochs N] "
      "[--output-start-epoch N] [--output-end-epoch N] "
      "[--threads N] [--resume]");
  Options result;
  result.config = argv[1];
  result.output = argv[2];
  result.dataset = argv[3];
  result.seed_start = std::stoull(argv[4]);
  result.seed_count = std::stoull(argv[5]);
  if (result.dataset != "nominal" && result.dataset != "fault") {
    throw std::runtime_error("dataset must be nominal or fault");
  }
  for (int index = 6; index < argc; ++index) {
    const std::string key = argv[index];
    if (key == "--resume") { result.resume = true; continue; }
    if (++index >= argc) throw std::runtime_error("missing value for " + key);
    const std::string value = argv[index];
    if (key == "--epochs") result.epochs = std::stoi(value);
    else if (key == "--fault-onset") result.fault_onset = std::stoi(value);
    else if (key == "--fault-duration-epochs") {
      result.fault_duration_epochs = std::stoi(value);
    } else if (key == "--output-start-epoch" ||
               key == "--output-epoch-start") {
      result.output_start_epoch = std::stoi(value);
    } else if (key == "--output-end-epoch" ||
               key == "--output-epoch-end") {
      result.output_end_epoch = std::stoi(value);
    }
    else if (key == "--threads") result.threads = std::stoul(value);
    else if (key == "--histories") {
      result.histories.clear();
      for (const auto& item : split(value)) result.histories.push_back(std::stoul(item));
    } else if (key == "--trajectories") result.trajectories = split(value);
    else if (key == "--fault-magnitudes") {
      result.fault_magnitudes.clear();
      for (const auto& item : split(value)) result.fault_magnitudes.push_back(std::stod(item));
    } else if (key == "--fault-anchor") result.forced_fault_anchor = std::stoull(value);
    else if (key == "--fault-magnitude") result.forced_fault_magnitude = std::stod(value);
    else throw std::runtime_error("unknown option: " + key);
  }
  if (result.output_end_epoch < 0) result.output_end_epoch = result.epochs - 1;
  if (!result.seed_count || result.epochs <= 0 || !result.threads ||
      result.fault_onset < 0 ||
      result.fault_duration_epochs < 0 || result.output_start_epoch < 0 ||
      result.output_end_epoch < result.output_start_epoch ||
      result.output_end_epoch >= result.epochs ||
      (result.dataset == "fault" && result.fault_onset >= result.epochs)) {
    throw std::runtime_error("invalid count/epoch option");
  }
  std::set<std::uint32_t> unique_histories;
  for (const auto history : result.histories) {
    if (history == 1 || !unique_histories.insert(history).second) {
      throw std::runtime_error("histories must be unique and each must be 0 or >=2");
    }
  }
  for (const auto& trajectory : result.trajectories) {
    if (trajectory != "straight" && trajectory != "circle" &&
        trajectory != "figure_eight") throw std::runtime_error("invalid trajectory");
  }
  if (result.fault_magnitudes.empty() ||
      std::any_of(result.fault_magnitudes.begin(), result.fault_magnitudes.end(),
                  [](double x) { return !std::isfinite(x) || x <= 0.0; })) {
    throw std::runtime_error("fault magnitudes must be finite and positive");
  }
  if (result.forced_fault_anchor > 8 || result.forced_fault_magnitude < 0.0) {
    throw std::runtime_error("forced fault option is invalid");
  }
  return result;
}

std::uint64_t mix(std::uint64_t value) {
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

struct Truth {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

Truth truth(const std::string& trajectory, double time) {
  Truth result;
  if (trajectory == "straight") {
    result.position = {0.2*time, 0.0, 1.2};
    result.velocity = {0.2, 0.0, 0.0};
    result.acceleration.setZero();
  } else if (trajectory == "circle") {
    result.position = {3*std::cos(.15*time), 3*std::sin(.15*time), 1.2};
    result.velocity = {-.45*std::sin(.15*time), .45*std::cos(.15*time), 0.0};
    result.acceleration = {-.0675*std::cos(.15*time),
                           -.0675*std::sin(.15*time), 0.0};
  } else {
    result.position = {3*std::sin(.15*time), 1.5*std::sin(.30*time), 1.2};
    result.velocity = {.45*std::cos(.15*time), .45*std::cos(.30*time), 0.0};
    result.acceleration = {-.0675*std::sin(.15*time),
                           -.135*std::sin(.30*time), 0.0};
  }
  return result;
}

class Digest {
 public:
  void bytes(const void* data, std::size_t count) {
    const auto* cursor = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < count; ++i) {
      value_ ^= cursor[i];
      value_ *= 1099511628211ULL;
    }
  }
  template <class T> void add(const T& value) { bytes(&value, sizeof(value)); }
  std::string hex() const {
    std::ostringstream output;
    output << std::hex << std::setfill('0') << std::setw(16) << value_;
    return output.str();
  }
 private:
  std::uint64_t value_ = 1469598103934665603ULL;
};

struct EpochInput {
  Truth truth;
  std::vector<uwb_imu_pl::ImuMeasurement> imu;
  uwb_imu_pl::UwbBatch uwb;
  bool fault_active = false;
};

struct Sequence {
  std::vector<EpochInput> epochs;
  std::string digest;
  std::uint64_t fault_anchor = 0;
  double fault_magnitude = 0.0;
};

Sequence generate(const Options& options, const uwb_imu_pl::IntegrityConfig& config,
                  std::uint64_t seed, const std::string& trajectory) {
  Sequence result;
  result.epochs.reserve(options.epochs);
  const std::uint64_t offset = seed - options.seed_start;
  result.fault_anchor = options.forced_fault_anchor ? options.forced_fault_anchor : offset % 8 + 1;
  result.fault_magnitude = options.forced_fault_magnitude > 0.0
      ? options.forced_fault_magnitude
      : options.fault_magnitudes[(offset / 8) % options.fault_magnitudes.size()];
  std::mt19937_64 random(mix(seed) ^ mix(std::hash<std::string>{}(trajectory)));
  std::normal_distribution<double> normal;
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Digest digest;
  digest.add(seed);
  digest.bytes(trajectory.data(), trajectory.size());
  for (int epoch = 0; epoch < options.epochs; ++epoch) {
    EpochInput input;
    const double time = (epoch + 1) * .05;
    input.truth = truth(trajectory, time);
    for (int sample = 1; sample <= 10; ++sample) {
      const double sample_time = epoch*.05 + sample*.005;
      const Truth sample_truth = truth(trajectory, sample_time);
      for (int axis = 0; axis < 3; ++axis) {
        accel_bias(axis) += config.imu.accelerometer_bias_rw_sigma *
                            std::sqrt(.005) * normal(random);
        gyro_bias(axis) += config.imu.gyroscope_bias_rw_sigma *
                           std::sqrt(.005) * normal(random);
      }
      uwb_imu_pl::ImuMeasurement measurement;
      measurement.id = uwb_imu_pl::MeasurementId(
          static_cast<std::uint64_t>(epoch)*10 + sample);
      measurement.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>(std::llround(sample_time*1e9)));
      measurement.specific_force_mps2 = sample_truth.acceleration +
          Eigen::Vector3d(0, 0, config.imu.gravity_mps2) + accel_bias;
      measurement.angular_velocity_radps = gyro_bias;
      for (int axis = 0; axis < 3; ++axis) {
        measurement.specific_force_mps2(axis) +=
            config.imu.accelerometer_sigma * normal(random);
        measurement.angular_velocity_radps(axis) +=
            config.imu.gyroscope_sigma * normal(random);
      }
      input.imu.push_back(measurement);
      digest.add(measurement.timestamp.value());
      digest.bytes(measurement.specific_force_mps2.data(), 3*sizeof(double));
      digest.bytes(measurement.angular_velocity_radps.data(), 3*sizeof(double));
    }
    input.uwb.id = uwb_imu_pl::BatchId(epoch + 1);
    input.uwb.timestamp = uwb_imu_pl::TimestampNs(
        static_cast<std::int64_t>(std::llround(time*1e9)));
    input.uwb.covariance_model_id = "advisor_diagonal_from_research_config";
    input.fault_active = options.dataset == "fault" && epoch >= options.fault_onset &&
        (options.fault_duration_epochs == 0 ||
         epoch < options.fault_onset + options.fault_duration_epochs);
    for (const auto& anchor : config.anchors) {
      uwb_imu_pl::UwbMeasurement measurement;
      measurement.id = uwb_imu_pl::MeasurementId(
          static_cast<std::uint64_t>(epoch + 1)*100 + anchor.id.value());
      measurement.factor_id = uwb_imu_pl::FactorId(measurement.id.value());
      measurement.anchor_id = anchor.id;
      measurement.timestamp = input.uwb.timestamp;
      measurement.anchor_position_m = anchor.position_world_m;
      measurement.sigma_m = config.realtime.range_sigma_m;
      measurement.range_m = (input.truth.position - anchor.position_world_m).norm() +
          measurement.sigma_m * normal(random);
      if (input.fault_active && anchor.id.value() == result.fault_anchor) {
        measurement.range_m += result.fault_magnitude;
      }
      input.uwb.measurements.push_back(measurement);
      digest.add(anchor.id.value());
      digest.add(measurement.range_m);
    }
    result.epochs.push_back(std::move(input));
  }
  result.digest = digest.hex();
  return result;
}

double orientationError(const Eigen::Quaterniond& estimate,
                        const Eigen::Quaterniond& reference) {
  Eigen::Quaterniond delta = reference.conjugate() * estimate;
  delta.normalize();
  return Eigen::AngleAxisd(delta).angle();
}

double peakRssMb() {
  struct rusage usage {};
  if (getrusage(RUSAGE_SELF, &usage) != 0) return std::numeric_limits<double>::quiet_NaN();
#ifdef __APPLE__
  return static_cast<double>(usage.ru_maxrss) / (1024.0*1024.0);
#else
  return static_cast<double>(usage.ru_maxrss) / 1024.0;
#endif
}

std::string finiteCsv(double value) {
  if (!std::isfinite(value)) return "";
  std::ostringstream output;
  output << std::setprecision(17) << value;
  return output.str();
}

std::string key(std::uint64_t seed, const std::string& trajectory,
                std::uint32_t history) {
  return std::to_string(seed) + "|" + trajectory + "|" + std::to_string(history);
}

std::string header() {
  return "seed,trajectory,graph_history,epoch,timestamp_ns,input_digest,"
      "fault_active,fault_m,anchor_id,fault_scenario,truth_px,truth_py,truth_pz,truth_vx,truth_vy,truth_vz,"
      "est_px,est_py,est_pz,est_vx,est_vy,est_vz,position_error_m,velocity_error_mps,"
      "orientation_error_rad,horizontal_error_m,vertical_error_m,cov_xx,cov_xy,cov_xz,cov_yy,cov_yz,cov_zz,position_nees,"
      "global_statistic,global_threshold,postfit_statistic,postfit_threshold,"
      "conditional_statistic,conditional_threshold,conditional_passed,batch_committed,"
      "hpl_m,vpl_m,finite_pl,availability,formal_eligible,measurement_model_valid,core_ms,estimator_ms,prior_extraction_ms,"
      "integrity_ms,remaining_overhead_ms,active_values,active_factors,retained_epochs,"
      "marginalization_count,boundary_prior_factors,peak_rss_mb\n";
}

std::string runMode(const Options& options, const uwb_imu_pl::IntegrityConfig& base,
                    const Sequence& sequence, std::uint64_t seed,
                    const std::string& trajectory, std::uint32_t history) {
  auto config = base;
  config.incremental.fixed_lag_epochs = history;
  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  const Truth initial_truth = truth(trajectory, 0.0);
  initial.position_world_m = initial_truth.position;
  initial.velocity_world_mps = initial_truth.velocity;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(
      config, config.realtime.lever_arm_body_m);
  estimator.initialize(initial, config.realtime.prior_sigmas);
  uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          config.risk, config.snapshot.rank_tolerance,
          config.snapshot.max_condition_number));
  uwb_imu_pl::ImuMeasurement boundary;
  boundary.timestamp = initial.timestamp;
  boundary.specific_force_mps2 = initial_truth.acceleration +
      Eigen::Vector3d(0, 0, config.imu.gravity_mps2);
  pipeline.ingestImu(boundary);
  std::ostringstream rows;
  rows << std::setprecision(17);
  const std::string fault_scenario = options.dataset == "nominal" ? "nominal" :
      (options.fault_duration_epochs == 1 ? "isolated_single_epoch" :
       "persistent_to_sequence_end");
  for (std::size_t epoch = 0; epoch < sequence.epochs.size(); ++epoch) {
    const auto& input = sequence.epochs[epoch];
    for (const auto& imu : input.imu) pipeline.ingestImu(imu);
    const auto start = std::chrono::steady_clock::now();
    const auto result = pipeline.processUwbBatch(input.uwb);
    const double core_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    const auto audit = estimator.audit();
    const Eigen::Vector3d position_error = result.state.position_world_m - input.truth.position;
    const Eigen::Vector3d velocity_error = result.state.velocity_world_mps - input.truth.velocity;
    const Eigen::Matrix3d position_covariance = audit.current_marginal.block<3, 3>(3, 3);
    Eigen::LDLT<Eigen::Matrix3d> ldlt(position_covariance);
    if (ldlt.info() != Eigen::Success || !position_covariance.allFinite() ||
        (ldlt.vectorD().array() <= 0.0).any()) {
      throw std::runtime_error("non-SPD position covariance");
    }
    const double nees = position_error.dot(ldlt.solve(position_error));
    if (!std::isfinite(nees) || !std::isfinite(core_ms)) {
      throw std::runtime_error("non-finite benchmark metric");
    }
    double integrity_ms = 0.0;
    for (const auto& stage : result.stage_timings) integrity_ms += stage.wall_ms;
    const double estimator_ms = estimator.lastNoUwbUpdateMs() + estimator.lastUwbUpdateMs();
    const double prior_ms = estimator.lastMarginalMs() + estimator.lastSnapshotExtractionMs();
    const double overhead_ms = std::max(0.0, core_ms-estimator_ms-prior_ms-integrity_ms);
    const bool finite_pl = std::isfinite(result.protection_level.hpl_m) &&
        std::isfinite(result.protection_level.vpl_m);
    if (static_cast<int>(epoch) < options.output_start_epoch ||
        static_cast<int>(epoch) > options.output_end_epoch) {
      continue;
    }
    rows << seed << ',' << trajectory << ',' << history << ',' << epoch << ','
         << input.uwb.timestamp.value() << ',' << sequence.digest << ','
         << input.fault_active << ','
         << (input.fault_active ? sequence.fault_magnitude : 0.0) << ','
         << sequence.fault_anchor << ',' << fault_scenario << ','
         << input.truth.position.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
         << input.truth.velocity.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
         << result.state.position_world_m.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
         << result.state.velocity_world_mps.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
         << position_error.norm() << ',' << velocity_error.norm() << ','
         << orientationError(result.state.q_world_body, input.truth.orientation) << ','
         << std::hypot(position_error.x(), position_error.y()) << ','
         << std::abs(position_error.z()) << ','
         << position_covariance(0,0) << ',' << position_covariance(0,1) << ','
         << position_covariance(0,2) << ',' << position_covariance(1,1) << ','
         << position_covariance(1,2) << ',' << position_covariance(2,2) << ','
         << nees << ',' << finiteCsv(result.global_detector.statistic) << ','
         << finiteCsv(result.global_detector.threshold) << ','
         << finiteCsv(result.postfit_detector.statistic) << ','
         << finiteCsv(result.postfit_detector.threshold) << ','
         << finiteCsv(result.detector.statistic) << ','
         << finiteCsv(result.detector.threshold) << ',' << result.detector.passed << ','
         << result.batch_committed << ',' << finiteCsv(result.protection_level.hpl_m) << ','
         << finiteCsv(result.protection_level.vpl_m) << ',' << finite_pl << ','
         << uwb_imu_pl::toString(result.protection_level.availability) << ','
         << result.protection_level.formal_eligible << ','
         << result.measurement_model_valid << ',' << core_ms << ','
         << estimator_ms << ',' << prior_ms << ',' << integrity_ms << ','
         << overhead_ms << ',' << audit.active_value_count << ',' << audit.factor_count << ','
         << audit.retained_epochs << ',' << audit.marginalization_count << ','
         << audit.boundary_prior_factor_count << ',' << peakRssMb() << '\n';
  }
  return rows.str();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse(argc, argv);
    const auto config = uwb_imu_pl::IntegrityConfigLoader::load(options.config);
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
      for (const auto& trajectory : options.trajectories) {
        bool missing = false;
        for (const auto history : options.histories) {
          missing = missing || !complete.count(key(options.seed_start+offset, trajectory, history));
        }
        if (missing) jobs.push_back({options.seed_start+offset, trajectory});
      }
    }
    const bool append = options.resume && std::ifstream(options.output).good();
    std::ofstream output(options.output, append ? std::ios::app : std::ios::out);
    std::ofstream checkpoint(checkpoint_path, append ? std::ios::app : std::ios::out);
    if (!output || !checkpoint) throw std::runtime_error("cannot open output/checkpoint");
    if (!append) output << header();
    std::atomic<std::size_t> next{0};
    std::atomic<bool> failed{false};
    std::mutex write_mutex;
    std::string worker_error;
    auto worker = [&] {
      while (!failed.load()) {
        const std::size_t index = next.fetch_add(1);
        if (index >= jobs.size()) return;
        try {
          const auto& job = jobs[index];
          const Sequence sequence = generate(options, config, job.seed, job.trajectory);
          std::ostringstream rows;
          std::vector<std::string> completed_keys;
          for (const auto history : options.histories) {
            const std::string item_key = key(job.seed, job.trajectory, history);
            if (complete.count(item_key)) continue;
            rows << runMode(options, config, sequence, job.seed, job.trajectory, history);
            completed_keys.push_back(item_key);
          }
          std::lock_guard<std::mutex> lock(write_mutex);
          output << rows.str(); output.flush();
          for (const auto& item_key : completed_keys) checkpoint << item_key << '\n';
          checkpoint.flush();
        } catch (const std::exception& error) {
          std::lock_guard<std::mutex> lock(write_mutex);
          failed.store(true);
          worker_error = error.what();
        }
      }
    };
    std::vector<std::thread> workers;
    const unsigned count = std::min<unsigned>(options.threads, jobs.size());
    for (unsigned index = 0; index < count; ++index) workers.emplace_back(worker);
    for (auto& thread : workers) thread.join();
    if (failed.load()) throw std::runtime_error(worker_error);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}
