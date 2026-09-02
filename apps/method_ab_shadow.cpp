#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"

#include <boost/filesystem.hpp>
#include <boost/math/distributions/chi_squared.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>

namespace {

double relativeError(const Eigen::MatrixXd& actual,
                     const Eigen::MatrixXd& expected) {
  return (actual - expected).norm() /
      std::max(expected.norm(), std::numeric_limits<double>::min());
}

std::uint64_t mix(std::uint64_t value) {
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: method_ab_shadow RESOLVED_CONFIG OUTPUT_CSV OUTPUT_JSON\n";
    return 2;
  }
  try {
    const auto config = uwb_imu_pl::IntegrityConfigLoader::load(argv[1]);
    if (config.incremental.fixed_lag_epochs != 200) {
      throw std::runtime_error("Method A/B shadow requires fixed_lag_epochs=200");
    }
    boost::filesystem::create_directories(
        boost::filesystem::path(argv[2]).parent_path());
    uwb_imu_pl::IncrementalUwbImuEstimator estimator(
        config, config.realtime.lever_arm_body_m);
    std::ofstream csv(argv[2]);
    csv << "trajectory,epoch,candidate_spd,condition,tolerance,"
           "prior_local_coordinate_relative_error,covariance_relative_error,"
           "statistic_relative_error,protection_level_relative_error,"
           "availability_equal,decision_equal,pass\n";
    const std::array<std::string, 3> trajectories{
        "straight", "circle", "figure_eight"};
    std::size_t candidates = 0;
    std::size_t passed = 0;
    double maximum_error = 0.0;
    for (std::size_t path = 0; path < trajectories.size(); ++path) {
      for (std::size_t epoch = 0; epoch < 200; ++epoch) {
        std::mt19937_64 random(mix(config.seed + path * 1000 + epoch));
        std::normal_distribution<double> normal;
        Eigen::Matrix<double, 15, 15> basis;
        Eigen::Matrix<double, 15, 1> prior_mean;
        for (int row = 0; row < 15; ++row) {
          prior_mean(row) = 0.05 * normal(random);
          for (int column = 0; column < 15; ++column) {
            basis(row, column) = normal(random) / std::sqrt(15.0);
          }
        }
        const Eigen::Matrix<double, 15, 15> prior_information =
            basis.transpose() * basis + Eigen::Matrix<double, 15, 15>::Identity();
        const Eigen::Matrix<double, 15, 15> prior_covariance =
            prior_information.ldlt().solve(
                Eigen::Matrix<double, 15, 15>::Identity());
        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(8, 15);
        Eigen::VectorXd measurement(8);
        for (int row = 0; row < 8; ++row) {
          Eigen::Vector3d direction(normal(random), normal(random), normal(random));
          direction.normalize();
          h.block<1, 3>(row, 3) = direction.transpose();
          h(row, 6 + row % 3) = 0.02 * normal(random);
          measurement(row) = normal(random) * config.realtime.range_sigma_m;
        }
        const Eigen::MatrixXd measurement_covariance =
            Eigen::MatrixXd::Identity(8, 8) *
            std::pow(config.realtime.range_sigma_m, 2);
        const Eigen::MatrixXd measurement_information =
            measurement_covariance.inverse();
        const Eigen::Matrix<double, 15, 15> all_information =
            prior_information + h.transpose() * measurement_information * h;
        const Eigen::Matrix<double, 15, 15> all_covariance =
            all_information.ldlt().solve(
                Eigen::Matrix<double, 15, 15>::Identity());
        const Eigen::Matrix<double, 15, 1> all_mean = all_covariance *
            (prior_information * prior_mean +
             h.transpose() * measurement_information * measurement);
        const auto candidate = estimator.methodBCandidatePrior(
            all_mean, all_covariance, h, measurement_covariance, measurement);
        ++candidates;
        bool spd = candidate.has_value();
        double condition = std::numeric_limits<double>::infinity();
        double mean_error = std::numeric_limits<double>::infinity();
        double covariance_error = std::numeric_limits<double>::infinity();
        double statistic_error = std::numeric_limits<double>::infinity();
        double pl_error = std::numeric_limits<double>::infinity();
        bool availability_equal = false;
        bool decision_equal = false;
        double tolerance = 1e-3;
        if (candidate) {
          condition = candidate->condition_number;
          tolerance = std::min(1e-3, std::max(1e-9,
              100.0 * std::numeric_limits<double>::epsilon() *
              std::max(1.0, condition)));
          mean_error = relativeError(candidate->mean, prior_mean);
          covariance_error = relativeError(candidate->covariance, prior_covariance);
          const auto statistic = [&](const Eigen::Matrix<double, 15, 1>& mean,
                                     const Eigen::Matrix<double, 15, 15>& covariance) {
            const Eigen::VectorXd innovation = measurement - h * mean;
            const Eigen::MatrixXd innovation_covariance =
                h * covariance * h.transpose() + measurement_covariance;
            return innovation.dot(innovation_covariance.ldlt().solve(innovation));
          };
          const double oracle_statistic = statistic(prior_mean, prior_covariance);
          const double candidate_statistic = statistic(candidate->mean,
                                                       candidate->covariance);
          statistic_error = std::abs(candidate_statistic - oracle_statistic) /
              std::max(std::abs(oracle_statistic),
                       std::numeric_limits<double>::min());
          const Eigen::Vector3d oracle_pl =
              prior_covariance.block<3, 3>(3, 3).diagonal().cwiseSqrt();
          const Eigen::Vector3d candidate_pl =
              candidate->covariance.block<3, 3>(3, 3).diagonal().cwiseSqrt();
          pl_error = relativeError(candidate_pl, oracle_pl);
          const bool oracle_available = oracle_pl.head<2>().norm() <=
                  config.risk.horizontal_alert_limit_m &&
              oracle_pl.z() <= config.risk.vertical_alert_limit_m;
          const bool candidate_available = candidate_pl.head<2>().norm() <=
                  config.risk.horizontal_alert_limit_m &&
              candidate_pl.z() <= config.risk.vertical_alert_limit_m;
          availability_equal = oracle_available == candidate_available;
          const boost::math::chi_squared_distribution<double> distribution(8);
          const double threshold = boost::math::quantile(
              distribution, 1.0 - config.risk.p_fa);
          decision_equal = (oracle_statistic <= threshold) ==
              (candidate_statistic <= threshold);
        }
        const double error = std::max({mean_error, covariance_error,
                                       statistic_error, pl_error});
        maximum_error = std::max(maximum_error, error);
        const bool pass = spd && condition <= 1e10 && error <= tolerance &&
            availability_equal && decision_equal;
        passed += pass;
        csv << trajectories[path] << ',' << epoch << ',' << spd << ','
            << std::setprecision(17) << condition << ',' << tolerance << ','
            << mean_error << ',' << covariance_error << ',' << statistic_error
            << ',' << pl_error << ',' << availability_equal << ','
            << decision_equal << ',' << pass << '\n';
      }
    }
    std::ofstream summary(argv[3]);
    summary << "{\n  \"status\": \""
            << (passed == candidates && candidates == 600 ? "PASS" : "FAIL")
            << "\",\n  \"candidates\": " << candidates
            << ",\n  \"passed\": " << passed
            << ",\n  \"maximum_relative_error\": " << maximum_error
            << ",\n  \"online_enablement\": false\n}\n";
    return passed == candidates && candidates == 600 ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
