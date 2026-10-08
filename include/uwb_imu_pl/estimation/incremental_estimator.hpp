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

struct CommittedEpochCatalogAuditV1 {
  TransactionId transaction_id;
  std::size_t previous_epoch = 0;
  std::size_t proposed_epoch = 0;
  TimestampNs begin;
  TimestampNs end;
  BatchId uwb_batch_id;
  std::vector<FactorGroupId> selected_groups;
};

struct CommitBoundaryAuditV1 {
  NavigationState published_state;
  Eigen::Matrix<double, 15, 15> published_covariance =
      Eigen::Matrix<double, 15, 15>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  std::uint64_t ledger_version = 0;
  std::vector<FactorLedgerEntry> ledger_entries;
  std::vector<std::size_t> active_ledger_slots;
  std::vector<CommittedEpochCatalogAuditV1> committed_epoch_catalog;
  std::vector<std::pair<std::size_t, BatchId>> committed_batch_catalog;
  std::map<std::size_t, NavigationState> state_history;
  std::map<std::size_t, Eigen::Matrix<double, 15, 15>> covariance_history;
  std::vector<ImuMeasurement> imu_queue;
  std::optional<ImuMeasurement> imu_boundary;
  std::optional<TimestampNs> last_received_imu_timestamp;
  std::size_t epoch = 0;
  TimestampNs state_timestamp;
  LinearizationVersion version;
  std::uint64_t active_transaction_id = 0;
  std::optional<BatchId> pending_batch_id;
  bool pending_epoch = false;
  bool backend_poisoned = false;
  std::uint64_t backend_update_count = 0;
};

// Read-only row ownership certificate for independent raw-factor tests and
// offline diagnostics. This is a non-virtual diagnostic method, so it does not
// alter any long-lived public object layout. Row identities are derived from
// production transaction/ledger inventory, never supplied by the caller.
struct RawRowOwnershipAuditV1 {
  std::string row_id;
  FactorGroupId owner_group;
  std::size_t row_in_group = 0;
  std::string covariance_placement;
};

// Out-of-object test/debug sidecar. Keeping this state in the implementation
// registry preserves the estimator's golden object layout while giving O08 an
// unambiguous one-shot identity for the exact maybeInject invocation reached.
struct CommitFaultInjectionAuditV1 {
  CommitFaultPoint armed_point = CommitFaultPoint::None;
  std::uint64_t armed_nonce = 0;
  CommitFaultPoint hit_point = CommitFaultPoint::None;
  std::uint64_t hit_nonce = 0;
  bool armed = false;
  bool hit = false;
  bool consumed = false;
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
  FrozenIntegrityWindow buildFrozenIntegrityWindow(
      const EpochTransaction& transaction,
      const IntegrityWindowRequest& request) const;
  LinearizedFactorBlock buildPendingFactorBlock(
      const EpochTransaction& transaction, FactorGroupId group) const;
  CommitReceipt commitEpoch(EpochTransaction&& transaction,
                            const EpochCommitPlan& plan);
  CommitReceipt commitEpochCertified(
      EpochTransaction&& transaction, const EpochCommitPlan& plan,
      const CommitProtectionEvidenceV1* protection_evidence,
      CommitCertificationV1* certification);
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
  // Optional diagnostic export; no backend updates, no new object members.
  std::vector<NavigationState> retainedSmoothedStatesV1() const;
  double globalGraphResidualStatistic() const;
  bool globalDiagnosticsEnabled() const {
    return config_.output.write_global_diagnostics;
  }
  const IntegrityConfig& config() const { return config_; }
  EstimatorAudit audit() const;
  std::vector<RawRowOwnershipAuditV1> auditRawRowOwnershipV1(
      const EpochTransaction& transaction) const;
  const FactorLedger& factorLedger() const { return factor_ledger_; }
  std::uint64_t backendUpdateCount() const { return backend_update_count_; }
  EstimatorCacheAudit cacheAudit() const;
  HistoryRootCacheAudit historyRootCacheAuditForTesting() const;
  void enableHistoryRootOracleForTesting(bool enabled);
  CommitBoundaryAuditV1 commitBoundaryAudit() const;
  // Narrow read-only debug views for independent fault-injection snapshots.
  // These expose the underlying containers instead of deriving both sides of
  // a comparison through commitBoundaryAudit().
  const std::map<std::size_t, NavigationState>& debugStateHistory() const {
    return state_history_;
  }
  const std::map<std::size_t, Eigen::Matrix<double, 15, 15>>&
  debugCovarianceHistory() const { return prefix_covariances_; }
  std::vector<ImuMeasurement> debugImuQueue() const {
    return {imu_queue_.begin(), imu_queue_.end()};
  }
  const std::optional<ImuMeasurement>& debugImuBoundary() const {
    return imu_boundary_;
  }
  const std::optional<TimestampNs>& debugLastReceivedImuTimestamp() const {
    return last_received_imu_timestamp_;
  }
  std::vector<CommittedEpochCatalogAuditV1> debugCommittedEpochCatalog() const;
  std::vector<std::pair<std::size_t, BatchId>>
  debugCommittedBatchCatalog() const {
    return {committed_uwb_batches_.begin(), committed_uwb_batches_.end()};
  }
  LinearizationVersion debugVersion() const {
    return {graph_version_, ordering_version_, 1, linpoint_version_};
  }
  std::uint64_t debugActiveTransactionId() const {
    return active_transaction_id_ ? active_transaction_id_->value() : 0;
  }
  const std::optional<BatchId>& debugPendingBatchId() const {
    return pending_batch_id_;
  }
  bool backendPoisoned() const;
  std::uint64_t setCommitFaultPointForTesting(CommitFaultPoint point);
  CommitFaultInjectionAuditV1 commitFaultInjectionAuditForTesting() const;

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
  // Narrow diagnostic peers.  They expose no public method or data member and
  // therefore preserve the P0 ABI/API while allowing current-instance tests
  // to inspect a frozen active graph and mutate only its history tree root.
  friend struct P106ReadOnlyGraphSnapshotPeer;
  friend struct P106EstimatorHistoryMutationPeer;
  void corruptHistoryTreeRootForTesting();
  void resetHistoryTreeForTesting();
  struct CommittedEpochRecord {
    EpochTransaction transaction;
    std::vector<FactorGroupId> selected_groups;
  };
  struct BackendUpdateAudit {
    std::size_t marginalized_epochs = 0;
    std::vector<std::size_t> new_factor_slots;
    std::vector<std::size_t> removed_factor_slots;
    std::vector<gtsam::Key> marginalized_keys;
    std::vector<gtsam::Key> relinearized_keys;
    std::vector<gtsam::Key> marked_keys;
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
  NavigationState queryState(std::size_t epoch, TimestampNs timestamp) const;
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
