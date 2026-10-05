#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"

#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

using namespace uwb_imu_pl;

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  std::cout << "ImuNoiseConfig " << sizeof(ImuNoiseConfig) << ' '
            << alignof(ImuNoiseConfig) << ' '
            << offsetof(ImuNoiseConfig, accelerometer_sigma) << ' '
            << offsetof(ImuNoiseConfig, gyroscope_sigma) << ' '
            << offsetof(ImuNoiseConfig, accelerometer_bias_rw_sigma) << ' '
            << offsetof(ImuNoiseConfig, gyroscope_bias_rw_sigma) << ' '
            << offsetof(ImuNoiseConfig, gravity_mps2) << ' '
            << offsetof(ImuNoiseConfig, max_gap_s) << ' '
            << offsetof(ImuNoiseConfig, noise_overbound_calibration_id) << '\n';
  std::cout << "IntegrityConfig " << sizeof(IntegrityConfig) << ' '
            << alignof(IntegrityConfig) << '\n';
  std::cout << "FdeDecisionContextV1 " << sizeof(FdeDecisionContextV1) << ' '
            << alignof(FdeDecisionContextV1) << ' '
            << offsetof(FdeDecisionContextV1, protection_proof_ids) << ' '
            << offsetof(FdeDecisionContextV1, risk_result) << '\n';
  std::cout << "FdeDecisionContextV2 " << sizeof(FdeDecisionContextV2) << ' '
            << alignof(FdeDecisionContextV2) << ' '
            << offsetof(FdeDecisionContextV2, v1) << ' '
            << offsetof(FdeDecisionContextV2, action_search) << ' '
            << offsetof(FdeDecisionContextV2, trusted_generated_actions) << ' '
            << offsetof(FdeDecisionContextV2, max_evaluated_actions) << ' '
            << offsetof(FdeDecisionContextV2, action_search_lifecycle) << '\n';

  IntegrityConfig config = IntegrityConfigLoader::load(std::string(argv[1]));
  DetectorResultV2 all_in;
  std::vector<FaultHypothesisV2> hypotheses;
  std::vector<FaultModeEvidence> evidence;
  std::vector<CandidateEvaluation> candidates;
  std::vector<FactorGroupId> mandatory;
  FdeDecisionContextV1 context_v1;
  FdeDecisionContextV2 context_v2;
  const FdeManager manager;
  const FdeDecision legacy = manager.decide(
      all_in, hypotheses, evidence, &candidates, mandatory, config.risk_v2);
  const FdeDecision v1 = manager.decide(
      all_in, hypotheses, evidence, &candidates, mandatory, config.risk_v2,
      &context_v1);
  const FdeDecision v2 = manager.decide(
      all_in, hypotheses, evidence, &candidates, mandatory, config.risk_v2,
      &context_v2);
  const bool ok = config.imu.accelerometer_sigma > 0.0 &&
      !config.resolved_yaml.empty() &&
      !legacy.reason.empty() && !v1.reason.empty() && !v2.reason.empty();
  std::cout << "load-by-value-vector-destruct-old-decide " << ok << '\n';
  return ok ? 0 : 1;
}
