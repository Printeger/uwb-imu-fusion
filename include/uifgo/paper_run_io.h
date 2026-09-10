#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "uifgo/nlos_inference.h"
#include "uifgo/nlos_refit.h"

namespace uifgo {

struct CommonPreparationIdentityInput {
  std::string raw_source_sha256;
  std::string window_role_split_cutoff_sha256;
  std::string observation_ledger_sha256;
  std::string input_plan_sha256;
  std::string nominal_sigma_sha256;
  std::string initialization_rule;
  std::string initial_values_sha256;
  std::string calibration_sha256;
  std::string physical_graph_sha256;
  std::string factor_metadata_sha256;
  std::string solver_precision_sha256;
};

std::string ComputeCommonPreparationId(
    const CommonPreparationIdentityInput& input);

struct FinalRequestIdentityInput {
  std::string stage2_cache_id;
  std::string canonical_mode;
  std::string policy_version;
  std::string thresholds_sha256;
  std::string threshold_provenance;
  std::string final_refit_score_config_sha256;
  std::string solver_sha256;
  std::string common_preparation_id;
};

std::string ComputeFinalRequestId(const FinalRequestIdentityInput& input);

struct ExportVerificationResult {
  bool ok = false;
  std::string status = "FAILED";
  std::string reason;
  size_t checked_files = 0;
  std::vector<std::string> artifact_sha256;
};

// Verifies the still-live result identity after export, checks the required
// identity artifact and hashes every file named by the writer. This is an
// additional post-write audit; it does not alter T08 content identity.
ExportVerificationResult VerifyInferenceExport(
    const std::string& output_directory, const InferenceResult& result,
    const std::vector<std::string>& exported_files);

struct DevelopmentStage2ExportResult {
  bool ok = false;
  std::string status = "FAILED";
  std::string reason;
  size_t values_count = 0;
  size_t scalar_count = 0;
  std::string graph_linearization_sha256;
  std::string values_sha256;
};

// Development-only Stage-2 checkpoint bundle.  The success manifest is
// published last; without it, any files left by an interrupted export are not
// a complete checkpoint.  This does not make the artifacts formally
// consumable.
DevelopmentStage2ExportResult WriteDevelopmentStage2Bundle(
    const std::string& stage2_directory,
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const std::vector<RefitFactorMeta>& factor_metadata,
    const InferenceIdentityContext& identity_context);

// Strict production reader for the Values file emitted by the function above.
// Only X/Pose3, V/Vector3, B/ConstantBias and C/double are accepted.
gtsam::Values ReadDevelopmentStage2Values(const std::string& values_csv);

// Atomically records completion only after score artifacts have been written.
// It refuses missing, malformed, incomplete, or already-completed manifests.
void MarkDevelopmentStage2ScoreComplete(
    const std::string& stage2_directory);

// Bitwise component comparison for the supported Stage-2 Values domain.
bool DevelopmentStage2ValuesBitEqual(const gtsam::Values& expected,
                                     const gtsam::Values& actual,
                                     std::string* reason);

}  // namespace uifgo
