#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <gtsam/inference/Key.h>

#include "uifgo/config.h"
#include "uifgo/nlos_recoverability.h"
#include "uifgo/nlos_refit.h"
#include "uifgo/paper_input.h"

namespace uifgo {

constexpr const char* kT05OracleScoreDebugLabel =
    "T05_ORACLE_SUPPORT_SCORE_DEBUG_ONLY";

struct SegmentOverlapGroup {
  std::string group_id;
  double start_time = 0.0;
  double end_time = 0.0;
  std::vector<size_t> segment_ordinals;
};

std::vector<SegmentOverlapGroup> BuildClosedIntervalOverlapGroups(
    const std::vector<SegmentEstimate>& segments);

struct KeyColumnMeta {
  gtsam::Key key = 0;
  size_t offset = 0;
  size_t dimension = 0;
  std::string role;
};

struct FactorRowMeta {
  size_t original_factor_index = 0;
  size_t row_offset = 0;
  size_t row_count = 0;
  std::uint64_t obs_id = 0;
  std::string factor_type;
  std::string segment_id;
};

struct SegmentFitScore {
  size_t segment_ordinal = 0;
  std::string segment_id;
  double gamma = 0.0;
  bool short_support_debug = false;
  bool boundary = false;
};

struct GroupRecoverabilityScore {
  SegmentOverlapGroup group;
  RecoverabilityResult numerical;
  Eigen::SparseMatrix<double> F_whitened;
  Eigen::MatrixXd G_whitened;
  Eigen::VectorXd rhs_whitened;
  std::vector<KeyColumnMeta> key_columns;
  std::vector<FactorRowMeta> factor_rows;
  std::vector<SegmentFitScore> segment_fit;
  std::string linearization_id;
  bool eligible = false;
  bool valid_score_exported = false;
  std::string status;
  double elapsed_seconds = 0.0;
};

std::vector<GroupRecoverabilityScore> ScoreRefitRecoverability(
    const SegmentRefitResult& refit, const SupportPartition& support,
    const PaperInputPlan& plan, const Config& cfg,
    const RecoverabilityOptions& options = RecoverabilityOptions());

std::vector<GroupRecoverabilityScore> ScoreRefitRecoverability(
    const SegmentRefitResult& refit, const OracleSupport& support,
    const PaperInputPlan& plan, const Config& cfg,
    const RecoverabilityOptions& options = RecoverabilityOptions());

// Stage-4 score audit.  The refit graph contains accepted candidates only;
// frozen_full_support identifies the suppressed candidate observations that
// are intentionally absent and must never be interpreted as noncandidates.
std::vector<GroupRecoverabilityScore> ScoreFinalRefitRecoverability(
    const SegmentRefitResult& final_refit,
    const SupportPartition& accepted_support,
    const SupportPartition& frozen_full_support,
    const PaperInputPlan& plan, const Config& cfg,
    const RecoverabilityOptions& options = RecoverabilityOptions());

}  // namespace uifgo
