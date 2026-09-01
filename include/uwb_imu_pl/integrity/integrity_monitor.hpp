#pragma once

#include "uwb_imu_pl/common/types.hpp"
#include "uwb_imu_pl/estimation/snapshot_estimator.hpp"

#include <memory>

namespace uwb_imu_pl {

class IntegrityMonitor {
 public:
  IntegrityMonitor(RiskBudget risk, double rank_tolerance,
                   double max_condition_number)
      : risk_(std::move(risk)),
        rank_tolerance_(rank_tolerance),
        max_condition_number_(max_condition_number) {}

  IntegrityOutput evaluateSnapshot(const UwbBatch& batch,
                                   const SnapshotSolution& solution) const;
  IntegrityOutput evaluateConditional(const UwbBatch& batch,
                                      const EstimationSnapshot& snapshot) const;

  static double noncentralityBoundary(int dof, double threshold, double p_md);

 private:
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

// Orchestration layer enforcing "detect before commit" for each UWB group.
class RealtimeIntegrityPipeline {
 public:
  RealtimeIntegrityPipeline(IncrementalUwbImuEstimator* estimator,
                            IntegrityMonitor monitor);
  void ingestImu(const ImuMeasurement& measurement);
  IntegrityOutput processUwbBatch(const UwbBatch& batch);

 private:
  IncrementalUwbImuEstimator* estimator_;
  IntegrityMonitor monitor_;
};

}  // namespace uwb_imu_pl
