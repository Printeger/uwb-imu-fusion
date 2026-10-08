#include <gtest/gtest.h>

#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"

#include <cmath>

namespace {
using namespace uwb_imu_pl;

LinearizedIntegrityWindow cacheWindow() {
  LinearizedIntegrityWindow window;
  window.id = WindowId(709);
  window.version = {1, 2, 3, 4};
  LinearizedFactorBlock block;
  block.group_id = FactorGroupId(1);
  block.version = window.version;
  // Two 15-dimensional navigation states exercise the production covariance
  // shape (3 protected coordinates by a multiple-of-15 state root).
  block.jacobian_whitened.resize(72, 30);
  for (int row = 0; row < 72; ++row) {
    for (int col = 0; col < 30; ++col) {
      block.jacobian_whitened(row, col) =
          std::sin(0.13 * (row + 1) * (col + 1)) +
          (row == col ? 3.0 : 0.0);
    }
  }
  block.residual_whitened = Eigen::VectorXd::Zero(72);
  block.jacobian_raw = block.jacobian_whitened;
  block.residual_raw = block.residual_whitened;
  block.covariance = Eigen::MatrixXd::Identity(72, 72);
  block.whitener = block.covariance;
  window.blocks.push_back(std::move(block));
  window.protected_state_map = Eigen::MatrixXd::Zero(3, 30);
  window.protected_state_map.leftCols<3>().setIdentity();
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  return window;
}

struct CacheOutcome {
  ProtectionLevelV2Result result;
  ProtectionLevelV2ProofV1 proof;
  ProtectionModeResponseCacheStatsV1 stats;
  std::vector<FaultHypothesisV2> hypotheses;
  std::uint64_t covariance_solves = 0;
};

CacheOutcome evaluate(bool enabled, std::size_t workers,
                      bool duplicate = false, bool second_candidate = false,
                      bool nearly_supported = false) {
  const auto owner = freezeIntegrityWindowCopy(cacheWindow());
  const auto admission = admitFrozenIntegrityWindow(owner);
  EXPECT_TRUE(admission) << admission.reason;
  RankUpdateConfig config{1e-10, 1e10, 10.0};
  RankUpdateEvaluator evaluator(config);
  const auto base = evaluator.factorizeOnce(admission);
  ExclusionAction action;
  action.id = ExclusionActionId(1);
  if (second_candidate) {
    // Rebuild a different numerical root using a replacement block; a cache
    // keyed only by mode ID would incorrectly inherit the first result.
    action.groups_to_remove = {FactorGroupId(1)};
    auto replacement = admission.window().blocks.front();
    replacement.group_id = FactorGroupId(2);
    replacement.jacobian_whitened *= 1.3;
    replacement.jacobian_raw = replacement.jacobian_whitened;
    action.groups_to_add = {FactorGroupId(2)};
    action.added_blocks = {replacement};
  }
  auto candidate = evaluator.evaluate(admission, base, action);
  EXPECT_TRUE(candidate.valid) << candidate.reason;
  const auto detector = JointWindowDetector().evaluateCandidate(
      admission, candidate, DetectorRiskContext{});
  EXPECT_TRUE(detector.passed) << detector.reason;
  ProtectionLevelSharedContext context;
  context.mode_maps[11] = Eigen::MatrixXd::Zero(72, 1);
  context.mode_maps[11](70, 0) = 1.0;
  context.mode_maps[12] = Eigen::MatrixXd::Zero(72, 1);
  context.mode_maps[12](71, 0) = 1.0;
  if (nearly_supported) {
    context.mode_maps[11] = admission.window().H.col(0);
    context.mode_maps[11](70, 0) += 8e-10;
  }
  if (duplicate) context.mode_maps[12] = context.mode_maps[11];
  CacheOutcome out;
  for (std::uint64_t i = 0; i < 16; ++i) {
    FaultHypothesisV2 h;
    h.id = HypothesisId(100 + i);
    h.modes = i % 4 == 0 ? std::vector<FaultModeId>{FaultModeId(11)}
        : i % 4 == 1 ? std::vector<FaultModeId>{FaultModeId(12)}
        : i % 4 == 2
            ? std::vector<FaultModeId>{FaultModeId(11), FaultModeId(12)}
            : std::vector<FaultModeId>{FaultModeId(12), FaultModeId(11)};
    h.prior_probability_bound = 1e-4;
    h.p_md_allocation = 1e-3;
    h.hmi_allocation = 1e-7;
    out.hypotheses.push_back(h);
  }
  AttemptProofArena arena;
  FlatProtectionCandidateV1 job;
  job.candidate = &candidate;
  job.detector = &detector;
  job.hypotheses = &out.hypotheses;
  job.shared = &context;
  job.result = &out.result;
  job.computation_audit = &out.proof;
  job.proof_arena = &arena;
  std::vector<FlatProtectionCandidateV1> jobs{job};
  CandidateWorkerPool pool(4);
  ProtectionLevelV2().computeSharedFlatBatch(admission, &jobs, RiskBudgetV2{},
      &pool, workers, 64u << 20, enabled, &out.stats);
  out.covariance_solves = candidate.diagnostics.covariance_solve_count;
  if (out.result.model_valid) {
    EXPECT_TRUE(validateProtectionLevelV2Proof(candidate, detector,
        out.hypotheses, out.result, arena, nullptr));
  }
  return out;
}

void compare(const CacheOutcome& reference, const CacheOutcome& actual) {
  EXPECT_EQ(actual.result.model_valid, reference.result.model_valid);
  EXPECT_EQ(actual.result.availability, reference.result.availability);
  EXPECT_EQ(actual.result.formal_eligible, reference.result.formal_eligible);
  EXPECT_EQ(actual.result.risk_budget_valid, reference.result.risk_budget_valid);
  EXPECT_DOUBLE_EQ(actual.result.allocated_outcome_risk,
                   reference.result.allocated_outcome_risk);
  EXPECT_EQ(actual.result.reason, reference.result.reason);
  EXPECT_EQ(actual.covariance_solves, reference.covariance_solves);
  ASSERT_EQ(actual.hypotheses.size(), reference.hypotheses.size());
  ASSERT_EQ(actual.proof.hypothesis_proofs.size(),
            reference.proof.hypothesis_proofs.size());
  ASSERT_EQ(actual.proof.component_proofs.size(),
            reference.proof.component_proofs.size());
  if (reference.result.model_valid) {
    EXPECT_TRUE(actual.result.pl_xyz_m.isApprox(reference.result.pl_xyz_m, 1e-12));
  }
  for (std::size_t i = 0; i < actual.proof.hypothesis_proofs.size(); ++i) {
    const auto& a = actual.proof.hypothesis_proofs[i];
    const auto& b = reference.proof.hypothesis_proofs[i];
    EXPECT_EQ(a.served_entry.hypothesis, b.served_entry.hypothesis);
    EXPECT_EQ(a.nullspace_class, b.nullspace_class);
    EXPECT_EQ(a.served_entry.monitorability.rank,
              b.served_entry.monitorability.rank);
    EXPECT_TRUE(a.certified_gram.isApprox(b.certified_gram, 1e-12));
    EXPECT_TRUE(a.protected_response.isApprox(b.protected_response, 1e-12));
    EXPECT_TRUE(a.raw_detection_factor.isApprox(b.raw_detection_factor, 1e-12));
  }
}
}  // namespace

TEST(ProtectionModeCache, FullHypothesisProofsMatchExhaustiveAndWorkers) {
  const auto reference = evaluate(false, 1);
  ASSERT_TRUE(reference.result.model_valid) << reference.result.reason;
  EXPECT_EQ(reference.stats.exhaustive_hypothesis_products, 16u);
  const auto one = evaluate(true, 1);
  compare(reference, one);
  EXPECT_EQ(one.stats.unique_mode_products, 2u);
  EXPECT_EQ(one.stats.cached_hypothesis_products, 16u);
  EXPECT_EQ(one.stats.exhaustive_hypothesis_products, 0u);
  EXPECT_EQ(one.proof.hypothesis_proofs.size(), 16u);
  for (const auto workers : {2u, 4u}) {
    const auto parallel = evaluate(true, workers);
    compare(one, parallel);
    EXPECT_EQ(parallel.proof.proof_identity, one.proof.proof_identity);
  }
}

TEST(ProtectionModeCache, SingularPairsRetainExhaustiveFallbackAndProofs) {
  const auto reference = evaluate(false, 1, true);
  const auto cached = evaluate(true, 4, true);
  compare(reference, cached);
  EXPECT_GE(cached.stats.exhaustive_hypothesis_products, 8u);
}

TEST(ProtectionModeCache, ChangedCandidateNeverReusesAnotherRoot) {
  const auto first = evaluate(true, 1);
  const auto changed = evaluate(true, 4, false, true);
  const auto reference = evaluate(false, 1, false, true);
  compare(reference, changed);
  ASSERT_TRUE(first.result.model_valid);
  ASSERT_TRUE(changed.result.model_valid) << changed.result.reason;
  EXPECT_FALSE(first.result.pl_xyz_m.isApprox(changed.result.pl_xyz_m, 1e-3));
}

TEST(ProtectionModeCache, NearStateSupportedFullRankUsesOriginalProducts) {
  const auto reference = evaluate(false, 1, false, false, true);
  const auto cached = evaluate(true, 4, false, false, true);
  compare(reference, cached);
  EXPECT_GT(cached.stats.exhaustive_hypothesis_products, 0u);
}
