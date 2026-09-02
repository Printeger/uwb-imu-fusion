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

class FixedLagBackend;

struct EstimatorAudit {
  Eigen::Matrix<double, 15, 15> current_marginal =
      Eigen::Matrix<double, 15, 15>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  std::size_t epoch = 0;
  TimestampNs state_timestamp;
  LinearizationVersion version;
  std::size_t factor_count = 0;
  std::size_t factor_slot_count = 0;
  std::size_t active_value_count = 0;
  std::size_t pose_value_count = 0;
  std::size_t velocity_value_count = 0;
  std::size_t bias_value_count = 0;
  std::size_t boundary_prior_factor_count = 0;
  std::size_t timestamp_count = 0;
  std::size_t oldest_retained_epoch = 0;
  std::uint32_t retained_epochs = 0;
  std::uint64_t marginalization_count = 0;
  bool fixed_lag_active = false;
  bool historical_fault_provenance = false;
  std::vector<BatchId> committed_uwb_batch_ids;
  bool pending_epoch = false;
  std::optional<BatchId> pending_batch_id;
};

struct MethodBCandidatePrior {
  Eigen::Matrix<double, 15, 1> mean =
      Eigen::Matrix<double, 15, 1>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  double condition_number = std::numeric_limits<double>::infinity();
};

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
  ~IncrementalUwbImuEstimator();

  void initialize(const NavigationState& initial_state,
                  const Eigen::Matrix<double, 15, 1>& prior_sigmas);
  void ingestImu(const ImuMeasurement& measurement);
  void validateUwbBatch(const UwbBatch& batch) const;

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
  double lastImuPreintegrationMs() const { return last_imu_preintegration_ms_; }
  double lastStateQueryMs() const { return last_state_query_ms_; }
  double lastSnapshotExtractionMs() const { return last_snapshot_extraction_ms_; }
  std::size_t currentEpoch() const { return epoch_; }
  std::size_t factorCount() const;
  std::size_t activeValueCount() const;
  bool fixedLagActive() const { return fixed_lag_backend_ != nullptr; }
  std::uint32_t retainedEpochs() const;
  std::uint64_t marginalizationCount() const { return marginalization_count_; }
  std::size_t oldestRetainedEpoch() const;
  double globalGraphResidualStatistic() const;
  bool globalDiagnosticsEnabled() const {
    return config_.output.write_global_diagnostics;
  }
  EstimatorAudit audit() const;

  // Method B candidate: computes the leave-current-out information downdate
  // and gates it numerically. It is never used by the formal output unless the
  // explicit config switch is enabled after deferred equivalence validation.
  std::optional<Eigen::Matrix<double, 15, 15>> methodBCandidatePrior(
      const Eigen::Matrix<double, 15, 15>& all_in_covariance,
      const Eigen::MatrixXd& current_uwb_jacobian,
      const Eigen::MatrixXd& current_uwb_covariance) const;
  std::optional<MethodBCandidatePrior> methodBCandidatePrior(
      const Eigen::Matrix<double, 15, 1>& all_in_mean,
      const Eigen::Matrix<double, 15, 15>& all_in_covariance,
      const Eigen::MatrixXd& current_uwb_jacobian,
      const Eigen::MatrixXd& current_uwb_covariance,
      const Eigen::VectorXd& current_uwb_linear_measurement) const;

 private:
  void queryCurrentState();
  CurrentStatePrior queryCurrentPrior(TimestampNs timestamp);
  Eigen::Matrix<double, 15, 15> currentJointMarginal() const;
  const gtsam::ISAM2& backendIsam() const;
  const gtsam::NonlinearFactorGraph& activeGraph() const;
  std::size_t backendUpdate(
      const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
      std::size_t timestamp_epoch, bool add_timestamps);
  void pruneRetainedMetadata();

  IntegrityConfig config_;
  Eigen::Vector3d lever_arm_body_m_;
  gtsam::ISAM2 isam2_;
  std::unique_ptr<FixedLagBackend> fixed_lag_backend_;
  NavigationState current_state_;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imu_params_;
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> preintegrated_;
  std::deque<ImuMeasurement> imu_queue_;
  std::optional<ImuMeasurement> imu_boundary_;
  std::optional<TimestampNs> last_received_imu_timestamp_;
  std::size_t epoch_ = 0;
  TimestampNs state_timestamp_;
  bool initialized_ = false;
  bool pending_epoch_ = false;
  bool current_uwb_committed_ = false;
  std::optional<BatchId> pending_batch_id_;
  std::deque<std::pair<std::size_t, BatchId>> committed_uwb_batches_;
  std::uint64_t marginalization_count_ = 0;
  std::uint64_t ordering_version_ = 1;
  std::uint64_t graph_version_ = 0;
  std::uint64_t linpoint_version_ = 0;
  double last_no_uwb_update_ms_ = 0.0;
  double last_marginal_ms_ = 0.0;
  double last_uwb_update_ms_ = 0.0;
  double last_imu_preintegration_ms_ = 0.0;
  double last_state_query_ms_ = 0.0;
  double last_snapshot_extraction_ms_ = 0.0;
};

}  // namespace uwb_imu_pl
