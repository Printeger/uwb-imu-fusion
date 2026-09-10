#pragma once

#include <string>
#include <vector>

namespace uifgo {

enum class Stage2CacheNamespace { AUTO_DISCOVERY, FIXED_PARTITION_DEBUG };
const char* Stage2CacheNamespaceName(Stage2CacheNamespace value);

struct Stage2CachePayload {
  std::string name;
  std::string sha256;
};

struct Stage2CacheManifest {
  std::string schema = "uifgo_t09_stage2_cache_v2";
  Stage2CacheNamespace cache_namespace =
      Stage2CacheNamespace::AUTO_DISCOVERY;
  std::string debug_label;
  std::string common_preparation_id;
  std::string source_identity;
  std::string producer_common_config_sha256;
  std::string producer_stage2_config_sha256;
  std::string stage1_config_sha256;
  std::string stage2_refit_config_sha256;
  std::string stage3_score_config_sha256;
  std::string support_partition_sha256;
  std::string observation_mapping_sha256;
  std::string stage2_graph_linearization_sha256;
  std::string stage2_values_sha256;
  std::string factor_metadata_sha256;
  std::string stage2_trace_sha256;
  std::string score_table_sha256;
  std::string producer_commit;
  std::string producer_binary_sha256;
  std::string producer_abi_sha256;
  std::string producer_toolchain_sha256;
  std::string stage2_status;
  std::string score_status;
  std::vector<Stage2CachePayload> payloads;
  std::string cache_id;
};

// Checked before loading any cached Values. common ID binds actual physical
// graph, initial Values, calibration and versioned IMU covariance parameters.
void RequireStage2CommonPreparation(const Stage2CacheManifest& manifest,
                                    const std::string& requested_common_id);

std::string ComputeStage2CacheId(const Stage2CacheManifest& manifest);
bool ValidateStage2CacheManifest(const Stage2CacheManifest& manifest,
                                 std::string* reason = nullptr);

// Atomic, no-overwrite publication of a metadata manifest. Payloads already
// live beside the manifest and are addressed by their recorded SHA-256. The
// manifest contains no decision/final/fallback fields by construction.
void PublishStage2CacheManifest(const std::string& cache_directory,
                                Stage2CacheManifest manifest);
Stage2CacheManifest ReadStage2CacheManifest(
    const std::string& manifest_path);

}  // namespace uifgo
