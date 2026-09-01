#pragma once

#include "uwb_imu_pl/common/types.hpp"
#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/integrity/estimation_snapshot.hpp"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <deque>
#include <memory>
#include <optional>

namespace uwb_imu_pl {

class IncrementalUwbEstimator {
 public:
  explicit IncrementalUwbEstimator(const IncrementalConfig& config);
  void initialize(TimestampNs timestamp, const Eigen::Vector3d& position_world_m,
                  const Eigen::Vector3d& velocity_world_mps);
  IntegrityOutput update(const UwbBatch& batch);
  NavigationState currentState() const;
  std::shared_ptr<const EstimationSnapshot> snapshot() const;

 private:
  IncrementalConfig config_;
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph full_graph_;
  gtsam::Values estimate_;
  std::size_t epoch_ = 0;
  TimestampNs timestamp_;
  std::vector<WhitenedRowBlock> row_blocks_;
  LinearizationDiagnostics diagnostics_;
  Eigen::MatrixXd marginal_;
  bool initialized_ = false;
};

class IncrementalUwbImuEstimator {
 public:
  IncrementalUwbImuEstimator(const IntegrityConfig& config,
                             const Eigen::Vector3d& lever_arm_body_m);

  void initialize(const NavigationState& initial_state,
                  const Eigen::Matrix<double, 15, 1>& prior_sigmas);
  void ingestImu(const ImuMeasurement& measurement);

  // Method A lifecycle. predictTo commits only IMU/history and creates the
  // current state key. preMeasurementSnapshot is read-only and guarantees the
  // returned prior excludes the supplied UWB group.
  void predictTo(TimestampNs timestamp);
  std::shared_ptr<const EstimationSnapshot> preMeasurementSnapshot(
      const UwbBatch& batch);
  void commitUwbBatch(const UwbBatch& batch);
  void rejectUwbBatch(const UwbBatch& batch, const std::string& reason);

  NavigationState currentState() const;
  bool hasPendingEpoch() const { return pending_epoch_; }
  double lastNoUwbUpdateMs() const { return last_no_uwb_update_ms_; }
  double lastMarginalMs() const { return last_marginal_ms_; }
  double lastUwbUpdateMs() const { return last_uwb_update_ms_; }

  // Method B candidate: computes the leave-current-out information downdate
  // and gates it numerically. It is never used by the formal output unless the
  // explicit config switch is enabled after deferred equivalence validation.
  std::optional<Eigen::Matrix<double, 15, 15>> methodBCandidatePrior(
      const Eigen::Matrix<double, 15, 15>& all_in_covariance,
      const Eigen::MatrixXd& current_uwb_jacobian,
      const Eigen::MatrixXd& current_uwb_covariance) const;

 private:
  NavigationState navigationState(std::size_t epoch,
                                  TimestampNs timestamp) const;
  CurrentStatePrior queryCurrentPrior(TimestampNs timestamp);
  void appendGraph(const gtsam::NonlinearFactorGraph& graph);

  IntegrityConfig config_;
  Eigen::Vector3d lever_arm_body_m_;
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph full_graph_;
  gtsam::Values estimate_;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imu_params_;
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preintegrated_;
  std::deque<ImuMeasurement> imu_queue_;
  std::size_t epoch_ = 0;
  TimestampNs state_timestamp_;
  bool initialized_ = false;
  bool pending_epoch_ = false;
  bool current_uwb_committed_ = false;
  std::uint64_t graph_version_ = 0;
  std::uint64_t linpoint_version_ = 0;
  double last_no_uwb_update_ms_ = 0.0;
  double last_marginal_ms_ = 0.0;
  double last_uwb_update_ms_ = 0.0;
};

}  // namespace uwb_imu_pl
