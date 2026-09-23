#pragma once

#include "uwb_imu_pl/common/types.hpp"
#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/integrity/estimation_snapshot.hpp"
#include "uwb_imu_pl/estimation/epoch_transaction.hpp"
#include "uwb_imu_pl/estimation/factor_ledger.hpp"
#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <deque>
#include <map>
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

struct EstimatorCacheAudit {
  std::uint64_t factor_block_hits = 0;
  std::uint64_t factor_block_misses = 0;
  std::uint64_t invalidations = 0;
  std::size_t factor_block_entries = 0;
  std::size_t factor_block_bytes = 0;
  std::string last_invalidation_reason;
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

  EpochTransaction prepareEpoch(const UwbBatch& batch);
  EpochTransaction prepareEpoch(const UwbBatch& batch,
                                const EpochPreparationOptions& options);
  LinearizedIntegrityWindow buildIntegrityWindow(
      const EpochTransaction& transaction,
      const IntegrityWindowRequest& request) const;
  LinearizedFactorBlock buildPendingFactorBlock(
      const EpochTransaction& transaction, FactorGroupId group) const;
  CommitReceipt commitEpoch(EpochTransaction&& transaction,
                            const EpochCommitPlan& plan);
  DiscardReceipt discardEpoch(EpochTransaction&& transaction,
                              const DiscardReason& reason);

  // Deprecated V1 adapter. It delegates to one V2 transaction and never
  // performs the historical two-update backend path.
  [[deprecated("use prepareEpoch/buildIntegrityWindow/commitEpoch")]]
  void predictTo(TimestampNs timestamp);
  [[deprecated("use buildIntegrityWindow")]]
  std::shared_ptr<const EstimationSnapshot> preMeasurementSnapshot(
      const UwbBatch& batch);
  [[deprecated("use commitEpoch")]]
  void commitUwbBatch(const UwbBatch& batch);
  [[deprecated("use commitEpoch/discardEpoch")]]
  void rejectUwbBatch(const UwbBatch& batch, const std::string& reason);

  NavigationState currentState() const;
  bool hasPendingEpoch() const { return active_transaction_id_.has_value(); }
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
  const IntegrityConfig& config() const { return config_; }
  EstimatorAudit audit() const;
  const FactorLedger& factorLedger() const { return factor_ledger_; }
  std::uint64_t backendUpdateCount() const { return backend_update_count_; }
  EstimatorCacheAudit cacheAudit() const;

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
  struct CommittedEpochRecord {
    EpochTransaction transaction;
    std::vector<FactorGroupId> selected_groups;
  };
  struct BackendUpdateAudit {
    std::size_t marginalized_epochs = 0;
    std::vector<std::size_t> new_factor_slots;
    std::vector<std::size_t> removed_factor_slots;
    std::vector<gtsam::Key> marginalized_keys;
    std::vector<std::size_t> boundary_factor_slots;
  };
  struct FactorBlockCacheEntry {
    TransactionId transaction_id;
    LinearizationVersion version;
    std::uint64_t content_fingerprint = 0;
    LinearizedFactorBlock block;
  };
  EpochTransaction prepareTransaction(TimestampNs timestamp,
                                      const UwbBatch* batch,
                                      const EpochPreparationOptions& options);
  void attachUwbGroups(EpochTransaction* transaction,
                       const UwbBatch& batch,
                       bool build_recovery_material) const;
  CurrentStatePrior pendingCurrentPrior(
      const EpochTransaction& transaction) const;
  LinearizedFactorBlock linearizePendingGroup(
      const PendingFactorGroup& group, const EpochTransaction& transaction,
      const Eigen::MatrixXd& full_jacobian,
      const Eigen::VectorXd& rhs) const;
  void queryCurrentState();
  CurrentStatePrior queryCurrentPrior(TimestampNs timestamp);
  Eigen::Matrix<double, 15, 15> currentJointMarginal() const;
  const gtsam::ISAM2& backendIsam() const;
  const gtsam::NonlinearFactorGraph& activeGraph() const;
  BackendUpdateAudit backendUpdate(
      const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
      std::size_t timestamp_epoch, bool add_timestamps,
      const std::vector<std::size_t>& remove_factor_slots = {});
  void pruneRetainedMetadata();
  Eigen::Matrix<double, 15, 15> jointMarginal(std::size_t epoch) const;

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
  bool pending_epoch_ = false;  // retained in EstimatorAudit ABI
  bool current_uwb_committed_ = false;
  std::optional<BatchId> pending_batch_id_;
  std::deque<std::pair<std::size_t, BatchId>> committed_uwb_batches_;
  std::deque<CommittedEpochRecord> committed_epochs_;
  std::map<std::size_t, NavigationState> state_history_;
  std::map<std::size_t, Eigen::Matrix<double, 15, 15>> prefix_covariances_;
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
  FactorLedger factor_ledger_;
  std::optional<TransactionId> active_transaction_id_;
  std::optional<EpochTransaction> adapter_transaction_;
  std::uint64_t next_transaction_id_ = 1;
  std::uint64_t next_window_id_ = 1;
  std::uint64_t backend_update_count_ = 0;
  bool backend_poisoned_ = false;
  mutable std::map<std::uint64_t, FactorBlockCacheEntry> factor_block_cache_;
  mutable EstimatorCacheAudit cache_audit_;
};

}  // namespace uwb_imu_pl
