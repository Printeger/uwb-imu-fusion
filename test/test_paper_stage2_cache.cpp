#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <fstream>

#include "uifgo/hash_utils.h"
#include "uifgo/imu_preint.h"
#include "uifgo/paper_run_io.h"
#include "uifgo/paper_stage2_cache.h"

namespace uifgo {
namespace {

Stage2CacheManifest Manifest(const boost::filesystem::path& root,
                             Stage2CacheNamespace cache_namespace) {
  std::ofstream(root / "scores.csv") << "group,status\ng,OK\n";
  Stage2CacheManifest value;
  value.cache_namespace = cache_namespace;
  value.debug_label = cache_namespace == Stage2CacheNamespace::AUTO_DISCOVERY
                          ? "RQ3_AUTO_DISCOVERY_END_TO_END"
                          : "RQ3_FIXED_PARTITION_DIAGNOSTIC_DEBUG_ONLY";
  value.common_preparation_id = "t09common-sha256:a";
  value.source_identity = "sha256:b";
  value.producer_common_config_sha256 = "t09commonconfig-sha256:c0";
  value.producer_stage2_config_sha256 = "t09stage2config-sha256:c1";
  value.stage1_config_sha256 = "sha256:stage1";
  value.stage2_refit_config_sha256 = "sha256:stage2";
  value.stage3_score_config_sha256 = "sha256:stage3";
  value.support_partition_sha256 = "sha256:c";
  value.observation_mapping_sha256 = "sha256:d";
  value.stage2_graph_linearization_sha256 = "sha256:e";
  value.stage2_values_sha256 = "sha256:f";
  value.factor_metadata_sha256 = "sha256:g";
  value.stage2_trace_sha256 = "sha256:h";
  value.score_table_sha256 = "sha256:i";
  value.producer_commit = "deadbeef";
  value.producer_binary_sha256 = "sha256:binary";
  value.producer_abi_sha256 = "t09abi-sha256:abi";
  value.producer_toolchain_sha256 = "t09toolchain-sha256:toolchain";
  value.stage2_status = "CONVERGED";
  value.score_status = "COMPLETE_WITH_SCORE_UNAVAILABLE";
  value.payloads.push_back(
      {"scores.csv", "sha256:" + Sha256FileHex((root / "scores.csv").string())});
  return value;
}

TEST(PaperStage2Cache, ProducerBinaryAndAbiInvalidateIdentity) {
  const auto root = boost::filesystem::temp_directory_path() /
                    boost::filesystem::unique_path("uifgo-t09-%%%%-%%%%");
  boost::filesystem::create_directories(root);
  auto manifest = Manifest(root, Stage2CacheNamespace::AUTO_DISCOVERY);
  const auto original = ComputeStage2CacheId(manifest);
  manifest.producer_binary_sha256 = "sha256:different-binary";
  EXPECT_NE(original, ComputeStage2CacheId(manifest));
  manifest.producer_binary_sha256 = "sha256:binary";
  manifest.producer_abi_sha256 = "t09abi-sha256:different-abi";
  EXPECT_NE(original, ComputeStage2CacheId(manifest));
  boost::filesystem::remove_all(root);
}

TEST(PaperStage2Cache, ProducerSemanticConfigurationInvalidatesIdentity) {
  const auto root = boost::filesystem::temp_directory_path() /
                    boost::filesystem::unique_path("uifgo-t09-%%%%-%%%%");
  boost::filesystem::create_directories(root);
  auto manifest = Manifest(root, Stage2CacheNamespace::AUTO_DISCOVERY);
  const auto original = ComputeStage2CacheId(manifest);
  manifest.stage1_config_sha256 = "sha256:changed-discovery";
  EXPECT_NE(original, ComputeStage2CacheId(manifest));
  manifest.stage1_config_sha256 = "sha256:stage1";
  manifest.stage2_refit_config_sha256 = "sha256:changed-refit";
  EXPECT_NE(original, ComputeStage2CacheId(manifest));
  manifest.stage2_refit_config_sha256 = "sha256:stage2";
  manifest.stage3_score_config_sha256 = "sha256:changed-score";
  EXPECT_NE(original, ComputeStage2CacheId(manifest));
  boost::filesystem::remove_all(root);
}

TEST(PaperStage2Cache, CacheIdentityMatchesPythonCanonicalVector) {
  const auto root = boost::filesystem::temp_directory_path() /
                    boost::filesystem::unique_path("uifgo-t09-%%%%-%%%%");
  boost::filesystem::create_directories(root);
  const auto manifest = Manifest(root, Stage2CacheNamespace::AUTO_DISCOVERY);
  EXPECT_EQ(ComputeStage2CacheId(manifest),
            "t09stage2cache-sha256:"
            "ac94b180a84c4d66cc2200d67139f1a557036523763c07624668e837c15a78d5");
  boost::filesystem::remove_all(root);
}

TEST(PaperStage2Cache, NamespacesAreContentSeparatedAndPartialScoresPublish) {
  const auto parent = boost::filesystem::temp_directory_path() /
                      boost::filesystem::unique_path("uifgo-t09-%%%%-%%%%");
  const auto auto_root = parent / "auto";
  const auto fixed_root = parent / "fixed";
  boost::filesystem::create_directories(auto_root);
  boost::filesystem::create_directories(fixed_root);
  auto automatic = Manifest(auto_root, Stage2CacheNamespace::AUTO_DISCOVERY);
  auto fixed = Manifest(fixed_root, Stage2CacheNamespace::FIXED_PARTITION_DEBUG);
  EXPECT_NE(ComputeStage2CacheId(automatic), ComputeStage2CacheId(fixed));
  EXPECT_NO_THROW(PublishStage2CacheManifest(auto_root.string(), automatic));
  EXPECT_NO_THROW(PublishStage2CacheManifest(fixed_root.string(), fixed));
  const auto read_auto = ReadStage2CacheManifest(
      (auto_root / "stage2_cache_manifest.json").string());
  EXPECT_EQ(read_auto.score_status, "COMPLETE_WITH_SCORE_UNAVAILABLE");
  EXPECT_EQ(read_auto.cache_namespace, Stage2CacheNamespace::AUTO_DISCOVERY);
  EXPECT_THROW(PublishStage2CacheManifest(auto_root.string(), automatic),
               std::runtime_error);
  boost::filesystem::remove_all(parent);
}

TEST(PaperStage2Cache, FixedNamespaceRequiresExactDebugLabel) {
  const auto root = boost::filesystem::temp_directory_path() /
                    boost::filesystem::unique_path("uifgo-t09-%%%%-%%%%");
  boost::filesystem::create_directories(root);
  auto manifest = Manifest(root, Stage2CacheNamespace::FIXED_PARTITION_DEBUG);
  manifest.debug_label = "T04_ORACLE_SUPPORT_DEBUG_ONLY";
  std::string reason;
  EXPECT_FALSE(ValidateStage2CacheManifest(manifest, &reason));
  EXPECT_EQ(reason, "FIXED_CACHE_DEBUG_LABEL_MISMATCH");
  boost::filesystem::remove_all(root);
}

TEST(PaperStage2Cache, ProducerConfigurationIdentityMustBeTyped) {
  const auto root = boost::filesystem::temp_directory_path() /
                    boost::filesystem::unique_path("uifgo-t09-%%%%-%%%%");
  boost::filesystem::create_directories(root);
  auto manifest = Manifest(root, Stage2CacheNamespace::AUTO_DISCOVERY);
  manifest.stage1_config_sha256 = "UNAVAILABLE";
  std::string reason;
  EXPECT_FALSE(ValidateStage2CacheManifest(manifest, &reason));
  EXPECT_EQ(reason, "CACHE_PRODUCER_CONFIG_IDENTITY_INVALID");
  boost::filesystem::remove_all(root);
}

TEST(PaperRunIo, IdentityDomainsDoNotConflateRequestAndInference) {
  CommonPreparationIdentityInput common;
  common.raw_source_sha256 = "sha256:a";
  common.window_role_split_cutoff_sha256 = "sha256:b";
  common.observation_ledger_sha256 = "sha256:c";
  common.input_plan_sha256 = "sha256:d";
  common.nominal_sigma_sha256 = "sha256:e";
  common.initialization_rule = "init";
  common.initial_values_sha256 = "sha256:f";
  common.calibration_sha256 = "sha256:g";
  common.physical_graph_sha256 = "sha256:h";
  common.factor_metadata_sha256 = "sha256:i";
  common.solver_precision_sha256 = "sha256:j";
  const auto common_id = ComputeCommonPreparationId(common);
  FinalRequestIdentityInput request;
  request.stage2_cache_id = "t09stage2cache-sha256:k";
  request.canonical_mode = "full_gate";
  request.policy_version = "v1";
  request.thresholds_sha256 = "sha256:l";
  request.threshold_provenance = "TEST_ONLY";
  request.final_refit_score_config_sha256 = "sha256:m";
  request.solver_sha256 = "sha256:n";
  request.common_preparation_id = common_id;
  const auto first = ComputeFinalRequestId(request);
  request.canonical_mode = "s_fit";
  EXPECT_NE(first, ComputeFinalRequestId(request));
  EXPECT_NE(first.find("t09finalrequest-sha256:"), std::string::npos);
}

TEST(PaperStage2Cache, A14PublishedCrossModelConsumerRejection) {
  const auto root=boost::filesystem::temp_directory_path()/
      boost::filesystem::unique_path("a14-cache-%%%%-%%%%");
  boost::filesystem::create_directories(root);
  Config cfg;
  const auto legacy=ImuCovarianceModelIdentity(cfg,gtsam::Vector3(0,0,-cfg.gravity),
      ImuCovarianceModel::LEGACY_GTSAM_COMBINED_DEFAULT_V1);
  const auto conditional=ImuCovarianceModelIdentity(cfg,gtsam::Vector3(0,0,-cfg.gravity),
      ImuCovarianceModel::PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1);
  auto m=Manifest(root,Stage2CacheNamespace::AUTO_DISCOVERY);
  m.common_preparation_id="t09common-sha256:"+Sha256Hex(legacy);
  PublishStage2CacheManifest(root.string(),m);
  auto loaded=ReadStage2CacheManifest((root/"stage2_cache_manifest.json").string());
  EXPECT_NO_THROW(RequireStage2CommonPreparation(loaded,m.common_preparation_id));
  EXPECT_THROW(RequireStage2CommonPreparation(loaded,
      "t09common-sha256:"+Sha256Hex(conditional)),std::runtime_error);
  loaded.common_preparation_id.clear();
  EXPECT_THROW(RequireStage2CommonPreparation(loaded,m.common_preparation_id),std::runtime_error);
  boost::filesystem::remove_all(root);
}

}  // namespace
}  // namespace uifgo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
