#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"

#include <boost/filesystem.hpp>
#include <boost/math/distributions/beta.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifndef UWB_IMU_PL_GIT_SHA
#define UWB_IMU_PL_GIT_SHA "unknown"
#endif
#ifndef UWB_IMU_PL_GIT_DIRTY
#define UWB_IMU_PL_GIT_DIRTY 1
#endif

namespace {

struct Options {
  std::string config_path;
  std::string output_directory;
  std::uint64_t h0_trials = 200000;
  std::uint64_t noncentral_trials = 100000;
  unsigned threads = std::max(1u, std::thread::hardware_concurrency());
};

Options parse(int argc, char** argv) {
  if (argc < 3) {
    throw std::runtime_error(
        "usage: integrity_monte_carlo CONFIG OUTPUT_DIR "
        "[--h0-trials N] [--noncentral-trials N] [--threads N]");
  }
  Options options;
  options.config_path = argv[1];
  options.output_directory = argv[2];
  for (int index = 3; index < argc; index += 2) {
    if (index + 1 >= argc) throw std::runtime_error("missing option value");
    const std::string key = argv[index];
    const std::string value = argv[index + 1];
    if (key == "--h0-trials") options.h0_trials = std::stoull(value);
    else if (key == "--noncentral-trials") {
      options.noncentral_trials = std::stoull(value);
    } else if (key == "--threads") options.threads = std::stoul(value);
    else throw std::runtime_error("unknown option: " + key);
  }
  if (options.h0_trials == 0 || options.noncentral_trials == 0 ||
      options.threads == 0) throw std::runtime_error("trial/thread counts must be positive");
  return options;
}

std::uint64_t derivedSeed(std::uint64_t root, std::uint64_t scenario) {
  std::uint64_t value = root + 0x9e3779b97f4a7c15ULL * (scenario + 1);
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

std::string json(const std::string& value) {
  std::ostringstream output;
  output << '"';
  for (const char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  output << '"';
  return output.str();
}

std::pair<double, double> exactInterval(std::uint64_t successes,
                                        std::uint64_t trials,
                                        double alpha = 0.05) {
  const double lower = successes == 0 ? 0.0 : boost::math::quantile(
      boost::math::beta_distribution<double>(successes, trials - successes + 1),
      alpha / 2.0);
  const double upper = successes == trials ? 1.0 : boost::math::quantile(
      boost::math::beta_distribution<double>(successes + 1, trials - successes),
      1.0 - alpha / 2.0);
  return {lower, upper};
}

double ksPValue(double statistic, std::uint64_t trials) {
  const double scaled = (std::sqrt(static_cast<double>(trials)) + 0.12 +
      0.11 / std::sqrt(static_cast<double>(trials))) * statistic;
  double sum = 0.0;
  for (int term = 1; term <= 100; ++term) {
    const double value = std::exp(-2.0 * term * term * scaled * scaled);
    sum += (term % 2 ? 1.0 : -1.0) * value;
    if (value < 1e-15) break;
  }
  return std::clamp(2.0 * sum, 0.0, 1.0);
}

struct H0Result {
  int anchors = 0;
  int dof = 0;
  std::uint64_t seed = 0;
  std::uint64_t trials = 0;
  std::uint64_t alarms = 0;
  double threshold = 0.0;
  double ks = 0.0;
  double ks_p = 0.0;
  double ci_low = 0.0;
  double ci_high = 0.0;
  std::vector<double> sorted;
};

struct NoncentralResult {
  int dof = 0;
  std::uint64_t anchor = 0;
  double ratio = 0.0;
  std::uint64_t seed = 0;
  std::uint64_t trials = 0;
  std::uint64_t misses = 0;
  double threshold = 0.0;
  double boundary = 0.0;
  double lambda = 0.0;
  double bias_m = 0.0;
  double theoretical = 0.0;
  double ci_low = 0.0;
  double ci_high = 0.0;
};

template <typename Function>
void parallelFor(std::size_t count, unsigned threads, Function function) {
  std::atomic<std::size_t> next{0};
  std::vector<std::thread> workers;
  for (unsigned worker = 0; worker < std::min<unsigned>(threads, count); ++worker) {
    workers.emplace_back([&] {
      while (true) {
        const std::size_t index = next.fetch_add(1);
        if (index >= count) return;
        function(index);
      }
    });
  }
  for (auto& worker : workers) worker.join();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse(argc, argv);
    const auto config = uwb_imu_pl::IntegrityConfigLoader::load(options.config_path);
    boost::filesystem::create_directories(options.output_directory);
    const std::vector<int> anchor_counts{4, 5, 6, 8};
    const std::vector<int> dofs{1, 2, 3, 5};
    std::vector<std::pair<int, int>> h0_jobs;
    for (int anchors : anchor_counts) for (int dof : dofs) {
      h0_jobs.emplace_back(anchors, dof);
    }
    std::vector<H0Result> h0(h0_jobs.size());
    parallelFor(h0_jobs.size(), options.threads, [&](std::size_t index) {
      H0Result result;
      result.anchors = h0_jobs[index].first;
      result.dof = h0_jobs[index].second;
      result.seed = derivedSeed(config.seed, index);
      result.trials = options.h0_trials;
      std::mt19937_64 random(result.seed);
      std::gamma_distribution<double> chi_square(result.dof / 2.0, 2.0);
      const boost::math::chi_squared_distribution<double> theoretical(result.dof);
      result.threshold = boost::math::quantile(theoretical, 1.0 - config.risk.p_fa);
      result.sorted.resize(result.trials);
      for (double& sample : result.sorted) {
        sample = chi_square(random);
        result.alarms += sample > result.threshold;
      }
      std::sort(result.sorted.begin(), result.sorted.end());
      for (std::size_t sample = 0; sample < result.sorted.size(); ++sample) {
        const double cdf = boost::math::cdf(theoretical, result.sorted[sample]);
        const double before = static_cast<double>(sample) / result.trials;
        const double after = static_cast<double>(sample + 1) / result.trials;
        result.ks = std::max(result.ks, std::max(std::abs(cdf - before),
                                                std::abs(after - cdf)));
      }
      result.ks_p = ksPValue(result.ks, result.trials);
      std::tie(result.ci_low, result.ci_high) =
          exactInterval(result.alarms, result.trials);
      h0[index] = std::move(result);
    });

    std::ofstream h0_summary(options.output_directory + "/h0.csv");
    h0_summary << "anchor_count,dof,trials,seed,p_fa,threshold,alarms,empirical_p_fa,"
                  "ci95_low,ci95_high,target_in_ci,ks,ks_p,ks_pass\n";
    std::ofstream h0_cdf(options.output_directory + "/h0_cdf.csv");
    h0_cdf << "anchor_count,dof,seed,quantile,statistic,empirical_cdf,theoretical_cdf\n";
    for (const auto& result : h0) {
      h0_summary << result.anchors << ',' << result.dof << ',' << result.trials
                 << ',' << result.seed << ',' << config.risk.p_fa << ','
                 << result.threshold << ',' << result.alarms << ','
                 << static_cast<double>(result.alarms) / result.trials << ','
                 << result.ci_low << ',' << result.ci_high << ','
                 << (config.risk.p_fa >= result.ci_low &&
                     config.risk.p_fa <= result.ci_high) << ','
                 << result.ks << ',' << result.ks_p << ',' << (result.ks_p > 0.01)
                 << '\n';
      boost::math::chi_squared_distribution<double> distribution(result.dof);
      for (int q = 1; q < 1000; ++q) {
        const double probability = q / 1000.0;
        const std::size_t sample = std::min<std::size_t>(
            result.sorted.size() - 1, probability * result.sorted.size());
        const double statistic = result.sorted[sample];
        h0_cdf << result.anchors << ',' << result.dof << ',' << result.seed
               << ',' << probability << ',' << statistic << ','
               << static_cast<double>(sample + 1) / result.sorted.size() << ','
               << boost::math::cdf(distribution, statistic) << '\n';
      }
    }

    struct Job { int dof; std::uint64_t anchor; double ratio; };
    std::vector<Job> jobs;
    for (int dof : dofs) for (const auto& anchor : config.anchors) {
      for (double ratio : {0.25, 0.5, 1.0, 2.0, 4.0}) {
        jobs.push_back({dof, anchor.id.value(), ratio});
      }
    }
    std::vector<NoncentralResult> noncentral(jobs.size());
    parallelFor(jobs.size(), options.threads, [&](std::size_t index) {
      const Job job = jobs[index];
      NoncentralResult result;
      result.dof = job.dof;
      result.anchor = job.anchor;
      result.ratio = job.ratio;
      result.seed = derivedSeed(config.seed, 1000 + index);
      result.trials = options.noncentral_trials;
      boost::math::chi_squared_distribution<double> central(job.dof);
      result.threshold = boost::math::quantile(central, 1.0 - config.risk.p_fa);
      result.boundary = uwb_imu_pl::IntegrityMonitor::noncentralityBoundary(
          job.dof, result.threshold,
          config.risk.hypotheses.front().missed_detection_allocation);
      result.lambda = result.ratio * result.boundary;
      // Conditional single-range conversion for the configured physical
      // anchor under the stated range sigma: lambda=(bias/sigma)^2.
      result.bias_m = config.realtime.range_sigma_m * std::sqrt(result.lambda);
      boost::math::non_central_chi_squared_distribution<double> distribution(
          job.dof, result.lambda);
      result.theoretical = boost::math::cdf(distribution, result.threshold);
      std::mt19937_64 random(result.seed);
      std::normal_distribution<double> normal;
      for (std::uint64_t trial = 0; trial < result.trials; ++trial) {
        double statistic = 0.0;
        for (int dimension = 0; dimension < job.dof; ++dimension) {
          const double value = normal(random) +
              (dimension == 0 ? std::sqrt(result.lambda) : 0.0);
          statistic += value * value;
        }
        result.misses += statistic <= result.threshold;
      }
      std::tie(result.ci_low, result.ci_high) =
          exactInterval(result.misses, result.trials);
      noncentral[index] = result;
    });
    std::ofstream nc(options.output_directory + "/noncentral.csv");
    nc << "dof,anchor_id,eta_ratio,trials,seed,p_fa,p_md_target,threshold,"
          "eta_boundary,lambda,bias_m,theoretical_p_md,misses,empirical_p_md,"
          "ci95_low,ci95_high,theory_in_ci\n";
    for (const auto& result : noncentral) {
      nc << result.dof << ',' << result.anchor << ',' << result.ratio << ','
         << result.trials << ',' << result.seed << ',' << config.risk.p_fa << ','
         << config.risk.hypotheses.front().missed_detection_allocation << ','
         << result.threshold << ',' << result.boundary << ',' << result.lambda
         << ',' << result.bias_m << ',' << result.theoretical << ','
         << result.misses << ','
         << static_cast<double>(result.misses) / result.trials << ','
         << result.ci_low << ',' << result.ci_high << ','
         << (result.theoretical >= result.ci_low &&
             result.theoretical <= result.ci_high) << '\n';
    }
    const auto run_manifest = uwb_imu_pl::makeRunManifest(
        config, UWB_IMU_PL_GIT_SHA, UWB_IMU_PL_GIT_DIRTY != 0);
    const bool h0_gate_pass = std::all_of(
        h0.begin(), h0.end(), [&](const H0Result& result) {
          return config.risk.p_fa >= result.ci_low &&
              config.risk.p_fa <= result.ci_high && result.ks_p > 0.01;
        });
    const bool noncentral_gate_pass = std::all_of(
        noncentral.begin(), noncentral.end(), [](const NoncentralResult& result) {
          return result.theoretical >= result.ci_low &&
              result.theoretical <= result.ci_high;
        });
    std::ostringstream command;
    for (int index = 0; index < argc; ++index) {
      if (index != 0) command << ' ';
      command << argv[index];
    }
    std::ofstream manifest(options.output_directory +
                           "/monte_carlo_manifest.json");
    manifest << "{\n"
             << "  \"schema_version\": " << json(run_manifest.schema_version) << ",\n"
             << "  \"created_utc\": " << json(run_manifest.created_utc) << ",\n"
             << "  \"git_sha\": " << json(run_manifest.git_sha) << ",\n"
             << "  \"git_dirty\": " << (run_manifest.git_dirty ? "true" : "false") << ",\n"
             << "  \"config_path\": " << json(run_manifest.config_path) << ",\n"
             << "  \"config_hash\": " << json(run_manifest.config_hash) << ",\n"
             << "  \"seed\": " << run_manifest.seed << ",\n"
             << "  \"build_type\": " << json(run_manifest.build_type) << ",\n"
             << "  \"compiler\": " << json(run_manifest.compiler) << ",\n"
             << "  \"os\": " << json(run_manifest.os) << ",\n"
             << "  \"cpu\": " << json(run_manifest.cpu) << ",\n"
             << "  \"ram_bytes\": " << run_manifest.ram_bytes << ",\n"
             << "  \"gtsam_version\": " << json(run_manifest.gtsam_version) << ",\n"
             << "  \"eigen_version\": " << json(run_manifest.eigen_version) << ",\n"
             << "  \"command\": " << json(command.str()) << ",\n"
             << "  \"h0_trials_per_job\": " << options.h0_trials << ",\n"
             << "  \"noncentral_trials_per_job\": "
             << options.noncentral_trials << ",\n"
             << "  \"threads\": " << options.threads << ",\n"
             << "  \"h0_gate_pass\": " << (h0_gate_pass ? "true" : "false") << ",\n"
             << "  \"noncentral_gate_pass\": "
             << (noncentral_gate_pass ? "true" : "false") << "\n}\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
