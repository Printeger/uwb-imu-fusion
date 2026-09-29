#pragma once

#include "uwb_imu_pl/common/types.hpp"
#include "uwb_imu_pl/estimation/snapshot_estimator.hpp"
#include "uwb_imu_pl/integrity/health_manager.hpp"
#include "uwb_imu_pl/integrity/publication_identity.hpp"
#include "uwb_imu_pl/estimation/reinitialization.hpp"
#include "uwb_imu_pl/integrity/attempt_proof_arena.hpp"

#include <functional>
#include <memory>
#include <optional>

namespace uwb_imu_pl {

// Contract boundary helper shared by runtime and deterministic tests.  The
// configured epoch and duration values themselves are legal; the first value
// strictly beyond either limit must fail closed.
bool bridgeTimeoutExceeded(std::uint32_t consecutive_epochs,
                           double duration_s,
                           std::uint32_t max_consecutive_epochs,
                           double max_duration_s);

class IntegrityMonitor {
 public:
  IntegrityMonitor(RiskBudget risk, double rank_tolerance,
                   double max_condition_number)
      : risk_(std::move(risk)),
        rank_tolerance_(rank_tolerance),
        max_condition_number_(max_condition_number) {}

  IntegrityOutput evaluateSnapshot(const UwbBatch& batch,
                                   const SnapshotSolution& solution,
                                   PublicationController* publication = nullptr) const;
  IntegrityOutput evaluateConditional(const UwbBatch& batch,
                                      const EstimationSnapshot& snapshot,
                                      PublicationController* publication = nullptr) const;

  static double noncentralityBoundary(int dof, double threshold, double p_md);

 private:
  // Bodies of the two stateless paths; the public entry points above append the
  // C4 publication gate (identity assembly + tri-state check) to them.
  IntegrityOutput evaluateSnapshotImpl(const UwbBatch& batch,
                                       const SnapshotSolution& solution) const;
  IntegrityOutput evaluateConditionalImpl(const UwbBatch& batch,
                                          const EstimationSnapshot& snapshot) const;
  DetectorResult snapshotDetector(const SnapshotSolution& solution) const;
  std::vector<FaultHypothesis> currentAnchorHypotheses(
      const UwbBatch& batch) const;
  SensitivityResult snapshotSensitivity(const UwbBatch& batch,
                                        const SnapshotSolution& solution,
                                        const FaultHypothesis& hypothesis,
                                        double threshold) const;
  SensitivityResult conditionalSensitivity(
      const UwbBatch& batch, const FaultHypothesis& hypothesis,
      const Eigen::MatrixXd& whitener, const Eigen::MatrixXd& innovation_cov,
      const Eigen::Matrix<double, 15, Eigen::Dynamic>& gain,
      const Eigen::Matrix<double, 3, 15>& protected_jacobian, int dof,
      double threshold) const;
  ProtectionLevelResult protectionLevel(
      TimestampNs timestamp, const Eigen::Matrix3d& covariance,
      const DetectorResult& detector,
      const std::vector<SensitivityResult>& sensitivities,
      const std::vector<FaultHypothesis>& hypotheses,
      const LinearizationDiagnostics& diagnostics,
      LinearizationConsistency consistency, bool model_formal_eligible,
      const std::string& gate_reason) const;
  bool completePhysicalFaultMap(
      const UwbBatch& batch, const std::vector<FaultHypothesis>& hypotheses,
      std::string* reason) const;
  bool snapshotFormalGate(
      const UwbBatch& batch, const SnapshotSolution& solution,
      const std::vector<FaultHypothesis>& hypotheses,
      std::string* reason) const;
  bool riskBudgetValid(const std::vector<FaultHypothesis>& hypotheses,
                       double* allocated) const;

  RiskBudget risk_;
  double rank_tolerance_;
  double max_condition_number_;
};

class IncrementalUwbImuEstimator;
class CandidateWorkerPool;

// Explicit, object-owned dependency seams for release-mode exception tests.
// They are inert unless a test injects this object directly; production has
// no environment variable, global switch, or implicit activation path.
struct PipelineTestDependencySeamsV1 {
  // Test-only seam at the real generator/census boundary.  It receives the
  // complete raw occurrence vector before the immutable census is staged;
  // production never installs this object.
  std::function<void(std::vector<ExclusionAction>*)>
      after_action_generation_before_census;
  std::function<void(ExclusionActionId)> before_candidate;
  std::function<void(ExclusionActionId)> before_post_detector;
  std::function<void(ExclusionActionId)> before_protection_level;
};

// Additive P1-04 test-only extension.  V1 is a frozen public ABI used by
// golden clients and must remain exactly four std::function fields.  The
// extension is installed through its own setter and retained in the existing
// out-of-object sidecar, so neither V1 nor RealtimeIntegrityPipeline grows.
struct PipelineTestDependencySeamsV2 : PipelineTestDependencySeamsV1 {
  // Test-only trigger for the post-selection deterministic proof-recompute
  // path when the production risk qualification artifact intentionally
  // refuses every candidate. It does not change FDE status, risk, selection,
  // commit, or publication; it only names the action whose first-pass scalar
  // summary must be reproduced before a full proof is imported.
  std::function<std::optional<ExclusionActionId>()>
      proof_recompute_action;
};

struct PipelineCommitBoundaryAuditV1 {
  HealthSnapshot health;
  std::uint32_t consecutive_bridge_epochs = 0;
  TimestampNs bridge_start_timestamp;
  bool bridge_start_present = false;
};

// Orchestration layer enforcing "detect before commit" for each UWB group.
class RealtimeIntegrityPipeline {
 public:
  RealtimeIntegrityPipeline(IncrementalUwbImuEstimator* estimator,
                            IntegrityMonitor monitor,
                            PublicationLimits publication_limits =
                                PublicationLimits{});
  ~RealtimeIntegrityPipeline();
  void ingestImu(const ImuMeasurement& measurement);
  IntegrityOutput processUwbBatch(const UwbBatch& batch);
  IntegrityOutput processUwbBatch(const UwbBatch& batch,
                                  AttemptProofLease* proof_lease);
  // C4/W2: the watchdog sample is taken from the caller when supplied (replay
  // harnesses and deterministic tests); the default is the monotonic steady
  // clock plus the batch timestamp.  The publication state holder is the same
  // object in both cases -- no second state source exists.
  IntegrityOutput processUwbBatch(const UwbBatch& batch,
                                  const ClockSample& clock_sample);
  IntegrityOutput processUwbBatch(const UwbBatch& batch,
                                  const ClockSample& clock_sample,
                                  AttemptProofLease* proof_lease);
  // Deterministic replay/evidence seam. It freezes both watchdog time and
  // end-of-attempt elapsed time without changing the configured deadline.
  IntegrityOutput processUwbBatchWithFrozenFinishElapsed(
      const UwbBatch& batch, const ClockSample& clock_sample,
      double finish_elapsed_ms);
  const PublicationController& publication() const { return publication_; }
  PipelineCommitBoundaryAuditV1 commitBoundaryAudit() const {
    PipelineCommitBoundaryAuditV1 out;
    out.health = health_.snapshot();
    out.consecutive_bridge_epochs = consecutive_bridge_epochs_;
    out.bridge_start_present = bridge_start_timestamp_.has_value();
    if (bridge_start_timestamp_) {
      out.bridge_start_timestamp = *bridge_start_timestamp_;
    }
    return out;
  }
  HealthSnapshot debugHealthSnapshot() const { return health_.snapshot(); }
  std::uint32_t debugConsecutiveBridgeEpochs() const {
    return consecutive_bridge_epochs_;
  }
  const std::optional<TimestampNs>& debugBridgeStartTimestamp() const {
    return bridge_start_timestamp_;
  }
  const ReinitializationDirective& reinitializationDirective() const {
    return reinitializer_.directive();
  }
  std::optional<NavigationState> pendingReinitializationSeed() const {
    return pending_reinitialization_seed_;
  }
  Eigen::Matrix<double, 15, 1> reinitializationPriorSigmas(
      const Eigen::Matrix<double, 15, 1>& base) const;
  void completeReinitialization(IncrementalUwbImuEstimator* estimator,
                                const ImuMeasurement& trusted_boundary);
  void setTestDependencySeamsV1(
      std::shared_ptr<const PipelineTestDependencySeamsV1> seams);
  void setTestDependencySeamsV2(
      std::shared_ptr<const PipelineTestDependencySeamsV2> seams);

 private:
  IncrementalUwbImuEstimator* estimator_;
  IntegrityMonitor monitor_;
  HealthManager health_;
  // C4/W2: the explicit publication state holder of this pipeline.  It owns
  // the publication state machine, the last admitted certificate and the
  // watchdog; nothing else in the pipeline stores publication state.
  PublicationController publication_;
  std::uint32_t consecutive_bridge_epochs_ = 0;
  std::optional<TimestampNs> bridge_start_timestamp_;
  ControlledReinitializer reinitializer_;
  std::optional<NavigationState> pending_reinitialization_seed_;
  bool awaiting_first_clean_uwb_ = false;
  std::uint64_t input_attempt_count_ = 0;
  std::uint64_t consecutive_rejections_ = 0;
  IntegrityOutput processUwbBatchImpl(const UwbBatch& batch,
                                      AttemptProofArena* proof_arena);
  IntegrityOutput processUwbBatchWithFinishElapsedOverride(
      const UwbBatch& batch, const ClockSample& clock_sample,
      const std::optional<double>& finish_elapsed_ms,
      AttemptProofArena* proof_arena = nullptr);
  // C4/W2: assembles the published identity from production values and passes
  // the attempt through the publication state holder.
  void applyPublicationGate(IntegrityOutput* output, const UwbBatch& batch,
                            AttemptProofArena* proof_arena = nullptr);
 public:
  const IntegrityOutput& lastAttemptOutput() const { return last_attempt_output_; }
 private:
  IntegrityOutput last_attempt_output_;
  std::unique_ptr<CandidateWorkerPool> candidate_workers_;
};

}  // namespace uwb_imu_pl
