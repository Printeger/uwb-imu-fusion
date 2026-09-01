#pragma once

#include "uwb_imu_pl/common/types.hpp"
#include "uwb_imu_pl/estimation/snapshot_estimator.hpp"

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

  static double noncentralityBoundary(int dof, double threshold, double p_md);

 private:
  DetectorResult snapshotDetector(const SnapshotSolution& solution) const;
  std::vector<FaultHypothesis> currentAnchorHypotheses(
      const UwbBatch& batch) const;
  SensitivityResult snapshotSensitivity(const UwbBatch& batch,
                                        const SnapshotSolution& solution,
                                        const FaultHypothesis& hypothesis,
                                        double threshold) const;
  ProtectionLevelResult protectionLevel(
      TimestampNs timestamp, const Eigen::Matrix3d& covariance,
      const DetectorResult& detector,
      const std::vector<SensitivityResult>& sensitivities,
      const LinearizationDiagnostics& diagnostics) const;

  RiskBudget risk_;
  double rank_tolerance_;
  double max_condition_number_;
};

}  // namespace uwb_imu_pl
