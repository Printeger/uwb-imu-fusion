#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/snapshot_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
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
  std::string config;
  std::string output;
  std::uint64_t seed_start = 20260901;
  std::uint64_t seed_count = 100;
  int epochs = 200;
  unsigned threads = std::max(1u, std::thread::hardware_concurrency());
  bool resume = false;
};

Options parse(int argc, char** argv) {
  if (argc < 3) {
    throw std::runtime_error(
        "usage: snapshot_integrity_sweep CONFIG OUTPUT_CSV "
        "[--seed-start N] [--seed-count N] [--epochs N] [--threads N] [--resume]");
  }
  Options options;
  options.config = argv[1];
  options.output = argv[2];
  for (int index = 3; index < argc; ++index) {
    const std::string key = argv[index];
    if (key == "--resume") {
      options.resume = true;
      continue;
    }
    if (++index >= argc) throw std::runtime_error("missing option value");
    const std::string value = argv[index];
    if (key == "--seed-start") options.seed_start = std::stoull(value);
    else if (key == "--seed-count") options.seed_count = std::stoull(value);
    else if (key == "--epochs") options.epochs = std::stoi(value);
    else if (key == "--threads") options.threads = std::stoul(value);
    else throw std::runtime_error("unknown option: " + key);
  }
  if (options.seed_count == 0 || options.epochs <= 0 || options.threads == 0) {
    throw std::runtime_error("seed/epoch/thread counts must be positive");
  }
  return options;
}

Eigen::Vector3d trajectory(const std::string& name, double t) {
  if (name == "straight") return {0.3 * t - 3.0, 0.5, 1.2};
  if (name == "circle") {
    return {3.0 * std::cos(0.2 * t), 3.0 * std::sin(0.2 * t), 1.2};
  }
  return {3.0 * std::sin(0.2 * t), 1.5 * std::sin(0.4 * t), 1.2};
}

std::uint64_t fnv1a(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

struct Job {
  std::string id;
  std::string path;
  std::string geometry;
  int anchor_count = 0;
  double scale = 1.0;
  double sigma = 0.1;
  int fault_anchor = -1;
  double fault_m = 0.0;
  std::uint64_t seed = 0;
};

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse(argc, argv);
    const auto config = uwb_imu_pl::IntegrityConfigLoader::load(options.config);
    const std::string checkpoint_path = options.output + ".completed";
    std::set<std::string> completed;
    if (options.resume) {
      std::ifstream checkpoint(checkpoint_path);
      std::string id;
      while (std::getline(checkpoint, id)) completed.insert(id);
    }
    std::vector<Job> jobs;
    for (std::uint64_t offset = 0; offset < options.seed_count; ++offset) {
      const std::uint64_t seed = options.seed_start + offset;
      for (const std::string path : {"straight", "circle", "figure_eight"}) {
        for (int anchor_count : {4, 6, 8}) {
          for (const std::string geometry : {"regular", "near_degenerate"}) {
            const std::vector<double> scales = geometry == "regular"
                ? std::vector<double>{0.7, 1.0, 1.5}
                : std::vector<double>{1.0};
            for (double scale : scales) for (double sigma : {0.05, 0.10, 0.20}) {
              std::vector<std::pair<int, double>> faults{{-1, 0.0}};
              for (int anchor = 0; anchor < anchor_count; ++anchor) {
                for (double bias : {0.5, 1.0, 2.0}) faults.emplace_back(anchor, bias);
              }
              for (const auto& fault : faults) {
                std::ostringstream id;
                id << seed << '|' << path << '|' << geometry << '|'
                   << anchor_count << '|' << scale << '|' << sigma << '|'
                   << fault.first << '|' << fault.second;
                if (completed.count(id.str()) == 0) {
                  jobs.push_back({id.str(), path, geometry, anchor_count, scale,
                                  sigma, fault.first, fault.second, seed});
                }
              }
            }
          }
        }
      }
    }

    const bool append = options.resume && boost::filesystem::exists(options.output);
    std::ofstream output(options.output, append ? std::ios::app : std::ios::out);
    std::ofstream checkpoint(checkpoint_path, append ? std::ios::app : std::ios::out);
    if (!output || !checkpoint) throw std::runtime_error("cannot open sweep output/checkpoint");
    if (!append) {
      output << "scenario_id,trajectory,geometry,anchor_count,geometry_scale,sigma,"
                "fault_anchor,fault_m,p_fa,p_md,seed,derived_seed,epoch,pe_x,pe_y,"
                "pe_z,signed_pe_x,signed_pe_y,signed_pe_z,pred_var_x,pred_var_y,"
                "pred_var_z,hpe,vpe,pl_x,pl_y,pl_z,hpl,vpl,axis_covered,h_covered,"
                "v_covered,hal_exceeded,val_exceeded,operational_hmi,availability,"
                "detector_passed,model_valid\n";
    }
    const std::vector<Eigen::Vector3d> base_anchors = {
        {-5,-5,0}, {5,-5,0.5}, {5,5,2.5}, {-5,5,3.0},
        {0,-6,4.0}, {0,6,1.0}, {-7,0,2.0}, {7,0,3.5}};
    std::atomic<std::size_t> next{0};
    std::mutex output_mutex;
    auto worker = [&] {
      while (true) {
        const std::size_t index = next.fetch_add(1);
        if (index >= jobs.size()) return;
        const Job& job = jobs[index];
        const std::uint64_t scenario_seed = fnv1a(job.id);
        std::mt19937_64 random(scenario_seed);
        std::normal_distribution<double> noise(0.0, job.sigma);
        uwb_imu_pl::SnapshotUwbEstimator estimator(config.snapshot);
        uwb_imu_pl::IntegrityMonitor monitor(
            config.risk, config.snapshot.rank_tolerance,
            config.snapshot.max_condition_number);
        std::ostringstream lines;
        lines.precision(17);
        for (int epoch = 0; epoch < options.epochs; ++epoch) {
          const double time = epoch * 0.05;
          const Eigen::Vector3d truth = trajectory(job.path, time);
          uwb_imu_pl::UwbBatch batch;
          batch.id = uwb_imu_pl::BatchId(epoch + 1);
          batch.timestamp = uwb_imu_pl::TimestampNs::fromSeconds(time);
          batch.covariance_model_id = "synthetic_diagonal";
          for (int anchor = 0; anchor < job.anchor_count; ++anchor) {
            uwb_imu_pl::UwbMeasurement measurement;
            measurement.id = uwb_imu_pl::MeasurementId(epoch * 100 + anchor + 1);
            measurement.factor_id = uwb_imu_pl::FactorId(measurement.id.value());
            measurement.anchor_id = uwb_imu_pl::AnchorId(anchor + 1);
            measurement.timestamp = batch.timestamp;
            measurement.anchor_position_m = job.scale * base_anchors[anchor];
            if (job.geometry == "near_degenerate") {
              measurement.anchor_position_m.y() *= 0.02;
              measurement.anchor_position_m.z() *= 0.02;
            }
            measurement.sigma_m = job.sigma;
            measurement.range_m =
                (truth - measurement.anchor_position_m).norm() + noise(random);
            if (job.fault_anchor == anchor) measurement.range_m += job.fault_m;
            batch.measurements.push_back(measurement);
          }
          const auto solution = estimator.estimate(
              batch, truth + Eigen::Vector3d(0.2, -0.2, 0.1));
          const auto integrity = monitor.evaluateSnapshot(batch, solution);
          const Eigen::Vector3d error =
              (solution.position_world_m - truth).cwiseAbs();
          const Eigen::Vector3d signed_error = solution.position_world_m - truth;
          const double hpe = std::hypot(error.x(), error.y());
          const double vpe = error.z();
          const auto& pl = integrity.protection_level;
          const bool axis_covered = (error.array() <= pl.pl_xyz_m.array()).all();
          const bool h_covered = hpe <= pl.hpl_m;
          const bool v_covered = vpe <= pl.vpl_m;
          const bool hal_exceeded = hpe > config.risk.horizontal_alert_limit_m;
          const bool val_exceeded = vpe > config.risk.vertical_alert_limit_m;
          const bool operational_hmi =
              (hal_exceeded || val_exceeded) &&
              pl.availability == uwb_imu_pl::Availability::Available;
          lines << job.id << ',' << job.path << ',' << job.geometry << ','
                << job.anchor_count << ',' << job.scale << ',' << job.sigma << ','
                << job.fault_anchor << ',' << job.fault_m << ','
                << config.risk.p_fa << ','
                << config.risk.hypotheses.front().missed_detection_allocation
                << ',' << job.seed << ',' << scenario_seed << ',' << epoch << ','
                << error.x() << ',' << error.y() << ',' << error.z() << ','
                << signed_error.x() << ',' << signed_error.y() << ','
                << signed_error.z() << ',' << solution.covariance_m2(0, 0) << ','
                << solution.covariance_m2(1, 1) << ','
                << solution.covariance_m2(2, 2) << ','
                << hpe << ',' << vpe << ',' << pl.pl_xyz_m.x() << ','
                << pl.pl_xyz_m.y() << ',' << pl.pl_xyz_m.z() << ','
                << pl.hpl_m << ',' << pl.vpl_m << ',' << axis_covered << ','
                << h_covered << ',' << v_covered << ',' << hal_exceeded << ','
                << val_exceeded << ',' << operational_hmi << ','
                << uwb_imu_pl::toString(pl.availability) << ','
                << integrity.detector.passed << ','
                << integrity.measurement_model_valid << '\n';
        }
        std::lock_guard<std::mutex> lock(output_mutex);
        output << lines.str();
        output.flush();
        checkpoint << job.id << '\n';
        checkpoint.flush();
      }
    };
    std::vector<std::thread> workers;
    for (unsigned index = 0; index < std::min<unsigned>(options.threads, jobs.size());
         ++index) workers.emplace_back(worker);
    for (auto& thread : workers) thread.join();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
