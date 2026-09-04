#pragma once

#include "uwb_imu_pl/integrity/joint_window_detector.hpp"

namespace uwb_imu_pl {

struct HypothesisEvaluationConfig {
  double rank_tolerance = 1e-10;
  double min_fault_gram_sigma = 1e-8;
  double max_fault_gram_condition = 1e10;
  double plausible_conditioned_statistic_margin = 0.0;
};

class HypothesisEvidenceEvaluator {
 public:
  explicit HypothesisEvidenceEvaluator(HypothesisEvaluationConfig config = {})
      : config_(config) {}
  std::vector<FaultModeEvidence> evaluateAll(
      const LinearizedIntegrityWindow& window,
      const std::vector<FaultModeBasis>& modes,
      std::vector<FaultHypothesisV2>* hypotheses,
      double squared_detector_threshold) const;
  std::vector<FaultModeEvidence> evaluateAll(
      const LinearizedIntegrityWindow& window,
      std::vector<FaultHypothesisV2>* hypotheses,
      double squared_detector_threshold) const;

 private:
  HypothesisEvaluationConfig config_;
};

}  // namespace uwb_imu_pl
