#pragma once

#include "uwb_imu_pl/common/types.hpp"
#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/integrity/estimation_snapshot.hpp"

#include <Eigen/Core>

#include <memory>

namespace uwb_imu_pl {

struct SnapshotSolution {
  TimestampNs timestamp;
  Eigen::Vector3d position_world_m = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance_m2 = Eigen::Matrix3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::MatrixXd covariance_measurement_m2;
  Eigen::MatrixXd whitener;
  Eigen::MatrixXd jacobian_raw;
  Eigen::MatrixXd jacobian_whitened;
  Eigen::VectorXd residual_raw_m;
  Eigen::VectorXd residual_whitened;
  Eigen::MatrixXd gain_whitened;
  Eigen::MatrixXd residual_projector;
  LinearizationDiagnostics diagnostics;
  bool converged = false;
  int iterations = 0;
};

class SnapshotUwbEstimator {
 public:
  explicit SnapshotUwbEstimator(SnapshotConfig config) : config_(config) {}
  SnapshotSolution estimate(const UwbBatch& batch,
                            const Eigen::Vector3d& initial_position_world_m) const;
  std::shared_ptr<const EstimationSnapshot> makeSnapshot(
      const UwbBatch& batch, const SnapshotSolution& solution,
      std::uint64_t version) const;

 private:
  SnapshotConfig config_;
};

}  // namespace uwb_imu_pl
