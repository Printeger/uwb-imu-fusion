#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/snapshot_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {

Eigen::Vector3d trajectory(const std::string& name, double t) {
  if (name == "straight") return {0.3 * t - 3.0, 0.5, 1.2};
  if (name == "circle") return {3.0 * std::cos(0.2 * t), 3.0 * std::sin(0.2 * t), 1.2};
  return {3.0 * std::sin(0.2 * t), 1.5 * std::sin(0.4 * t), 1.2};
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: snapshot_integrity_sweep CONFIG_YAML OUTPUT_CSV\n";
    return 2;
  }
  try {
    const auto config = uwb_imu_pl::IntegrityConfigLoader::load(argv[1]);
    std::mt19937_64 rng(config.seed);
    std::ofstream out(argv[2]);
    if (!out) throw std::runtime_error("cannot open sweep output");
    out << "trajectory,geometry_scale,sigma,fault_anchor,fault_m,p_fa,p_md,seed,epoch,error_m,statistic,threshold,pl_x,pl_y,pl_z,availability\n";
    const std::vector<Eigen::Vector3d> base_anchors = {
        {-5,-5,0}, {5,-5,0.5}, {5,5,2.5}, {-5,5,3.0}, {0,-6,4.0}, {0,6,1.0}};
    for (const std::string& path : {"straight", "circle", "figure_eight"}) {
      for (double geometry_scale : {0.7, 1.0, 1.5}) {
        for (double sigma : {0.05, 0.10, 0.20}) {
          std::normal_distribution<double> noise(0.0, sigma);
          for (int fault_anchor : {-1, 0, 2}) {
            for (double fault_m : {0.0, 0.5, 1.0, 2.0}) {
              for (int epoch = 0; epoch < 200; ++epoch) {
                const double t = epoch * 0.05;
                const Eigen::Vector3d truth = trajectory(path, t);
                uwb_imu_pl::UwbBatch batch;
                batch.id = uwb_imu_pl::BatchId(epoch + 1);
                batch.timestamp = uwb_imu_pl::TimestampNs::fromSeconds(t);
                batch.covariance_model_id = "synthetic_diagonal";
                for (std::size_t i = 0; i < base_anchors.size(); ++i) {
                  uwb_imu_pl::UwbMeasurement m;
                  m.id = uwb_imu_pl::MeasurementId(epoch * 100 + i + 1);
                  m.factor_id = uwb_imu_pl::FactorId(epoch * 100 + i + 1);
                  m.anchor_id = uwb_imu_pl::AnchorId(i + 1);
                  m.timestamp = batch.timestamp;
                  m.anchor_position_m = geometry_scale * base_anchors[i];
                  m.sigma_m = sigma;
                  m.range_m = (truth - m.anchor_position_m).norm() + noise(rng);
                  if (fault_anchor == static_cast<int>(i)) m.range_m += fault_m;
                  batch.measurements.push_back(m);
                }
                uwb_imu_pl::SnapshotUwbEstimator estimator(config.snapshot);
                const auto solution = estimator.estimate(batch, truth + Eigen::Vector3d(0.2, -0.2, 0.1));
                uwb_imu_pl::IntegrityMonitor monitor(config.risk, config.snapshot.rank_tolerance,
                                                     config.snapshot.max_condition_number);
                const auto integrity = monitor.evaluateSnapshot(batch, solution);
                out << path << ',' << geometry_scale << ',' << sigma << ','
                    << fault_anchor << ',' << fault_m << ',' << config.risk.p_fa << ','
                    << config.risk.hypotheses.front().missed_detection_allocation << ','
                    << config.seed << ',' << epoch << ','
                    << (solution.position_world_m - truth).norm() << ','
                    << integrity.detector.statistic << ',' << integrity.detector.threshold << ','
                    << integrity.protection_level.pl_xyz_m.transpose().format(
                           Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ","))
                    << ',' << uwb_imu_pl::toString(integrity.protection_level.availability) << '\n';
              }
            }
          }
        }
      }
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
