// A3 tests: strict fault manifest, configuration migration, failure taxonomy
// and the frozen result identity block.

#include "uwb_imu_pl/common/failure_reason.hpp"
#include "uwb_imu_pl/common/integrity_identity.hpp"
#include "uwb_imu_pl/config/fault_manifest.hpp"
#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

const std::string kSourceDir = UWB_IMU_PL_SOURCE_DIR;
const std::string kManifestPath =
    kSourceDir + "/config/integrity_fault_manifest.yaml";
const std::string kConfigPath =
    kSourceDir + "/config/realtime_uwb_imu_pl_research.yaml";

int g_temp_counter = 0;

std::string readFile(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot read " + path);
  std::ostringstream out;
  out << input.rdbuf();
  return out.str();
}

std::string writeTemp(const std::string& contents, const std::string& stem) {
  const std::string path = "/tmp/uwb_imu_pl_p2_" + stem + "_" +
      std::to_string(++g_temp_counter) + ".yaml";
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write " + path);
  out << contents;
  return path;
}

std::string tempConfig(const std::string& stem,
                       const std::vector<std::pair<std::string, std::string>>&
                           replacements) {
  std::string text = readFile(kConfigPath);
  for (const auto& item : replacements) {
    const auto position = text.find(item.first);
    if (position == std::string::npos) {
      throw std::runtime_error("config replacement not found: " + item.first);
    }
    text.replace(position, item.first.size(), item.second);
  }
  return writeTemp(text, stem);
}

bool loadThrows(const std::string& path, const std::string& needle) {
  try {
    (void)uwb_imu_pl::IntegrityConfigLoader::load(path);
  } catch (const std::exception& error) {
    if (needle.empty()) return true;
    return std::string(error.what()).find(needle) != std::string::npos;
  }
  return false;
}

bool manifestThrows(const std::string& path, const std::string& needle) {
  try {
    (void)uwb_imu_pl::FaultManifestLoader::load(path);
  } catch (const std::exception& error) {
    if (needle.empty()) return true;
    return std::string(error.what()).find(needle) != std::string::npos;
  }
  return false;
}

}  // namespace

TEST(FaultManifest, ShippedManifestLoadsValidatesAndDigests) {
  const uwb_imu_pl::FaultManifest manifest =
      uwb_imu_pl::FaultManifestLoader::load(kManifestPath);
  EXPECT_EQ(manifest.schema_version, 1);
  EXPECT_EQ(manifest.protected_quantity, "position_xyz");
  EXPECT_EQ(manifest.position_reference, "body_origin");
  EXPECT_EQ(manifest.max_fault_order, 2);
  EXPECT_EQ(manifest.digest.size(), 16u);
  EXPECT_GE(manifest.events.size(), 6u);
  EXPECT_TRUE(uwb_imu_pl::FaultManifestLoader::modelTypeImplemented(
      "epoch_independent_step"));
  EXPECT_TRUE(uwb_imu_pl::FaultManifestLoader::modelTypeImplemented(
      "interval_constant_axis"));
  EXPECT_FALSE(uwb_imu_pl::FaultManifestLoader::modelTypeImplemented(
      "interval_constant_multiaxis"));
  EXPECT_FALSE(manifest.resolved_yaml.empty());
}

TEST(FaultManifest, UnknownKeysAndUnknownFamiliesRejected) {
  std::string text = readFile(kManifestPath);
  text += "\n# appended bogus key test\n";
  std::string modified = text;
  const auto position = modified.find("schema_version: 1");
  modified.replace(position, std::string("schema_version: 1").size(),
                   "schema_version: 1\nbogus_unknown_key: 1");
  EXPECT_TRUE(manifestThrows(writeTemp(modified, "bogus"), "unknown key"));

  std::string family_text = readFile(kManifestPath);
  const auto enabled = family_text.find("enabled_pair_families:");
  family_text.replace(enabled, std::string("enabled_pair_families:").size(),
                      "enabled_pair_families:\n  - two_uwb");
  EXPECT_TRUE(manifestThrows(writeTemp(family_text, "two_uwb"),
                             "simultaneously declared unsupported"));
}

TEST(FaultManifest, EnabledFamilyWithUnsupportedModelTypeRejected) {
  std::string text = readFile(kManifestPath);
  // Keep a valid unsupported list but drop the multi-axis entry so only the
  // capability check can reject the enabled family.
  const auto unsupported = text.find("unsupported_families:");
  const auto events = text.find("events:", unsupported);
  text.replace(unsupported, events - unsupported,
               "unsupported_families:\n  - two_uwb\n  - imu_imu\n\n");
  const auto enabled = text.find("enabled_pair_families:");
  text.replace(enabled, std::string("enabled_pair_families:").size(),
               "enabled_pair_families:\n  - same_device_multiaxis");
  EXPECT_TRUE(manifestThrows(writeTemp(text, "multiaxis"),
                             "unsupported model_type"));
}

TEST(FaultManifest, ProjectionEligibilityIsIndependentFromActions) {
  std::string text = readFile(kManifestPath);
  // Changing action labels must not change numerical eligibility.
  const auto actions =
      text.find("allowed_actions: [keep_all, exclude_source_group]");
  ASSERT_NE(actions, std::string::npos);
  text.replace(actions, std::string(
      "allowed_actions: [keep_all, exclude_source_group]").size(),
      "allowed_actions: [keep_all]");
  const auto manifest = uwb_imu_pl::FaultManifestLoader::load(
      writeTemp(text, "actions"));
  EXPECT_TRUE(uwb_imu_pl::FaultManifestLoader::modelTypeImplemented(
      manifest.events.front().model_type));
  // But relabelling the numerical eligibility itself must fail.
  std::string broken = readFile(kManifestPath);
  const auto eligibility =
      broken.find("projection_eligibility: algebraic");
  ASSERT_NE(eligibility, std::string::npos);
  broken.replace(eligibility,
                 std::string("projection_eligibility: algebraic").size(),
                 "projection_eligibility: unsupported");
  EXPECT_TRUE(manifestThrows(writeTemp(broken, "eligibility"),
                             "projection_eligibility"));
}

TEST(FaultManifest, BoundedAmplitudeSetMustBeMarkedNotImplemented) {
  std::string text = readFile(kManifestPath);
  const auto amplitude = text.find(
      "amplitude_model: unbounded_subspace\n    prior_bound_source: "
      "fault_models.uwb.prior_probability_bound");
  ASSERT_NE(amplitude, std::string::npos);
  text.replace(amplitude,
               std::string("amplitude_model: unbounded_subspace").size(),
               "amplitude_model: validated_bounded_set");
  EXPECT_TRUE(manifestThrows(writeTemp(text, "bounded"),
                             "validated_bounded_set"));
}

TEST(ConfigMigration, OrderAndLegacyFlagConflictRejected) {
  const std::string path = tempConfig(
      "order_conflict",
      {{"double_faults_enabled: false", "double_faults_enabled: true"},
       {"manifest_path: integrity_fault_manifest.yaml",
        "manifest_path: " + kManifestPath}});
  EXPECT_TRUE(loadThrows(path, "conflicts with legacy"));
}

TEST(ConfigMigration, EpochsIntervalsConflictRejected) {
  const std::string path = tempConfig(
      "window_conflict",
      {{"intervals: 10", "intervals: 20"},
       {"manifest_path: integrity_fault_manifest.yaml",
        "manifest_path: " + kManifestPath}});
  EXPECT_TRUE(loadThrows(path, "conflicts with integrity_window.intervals"));
}

TEST(ConfigMigration, LegacyOnlyFieldsGainWarningsAndCanonicalKeys) {
  std::string text = readFile(kConfigPath);
  const auto order = text.find("  max_fault_order: 1\n");
  ASSERT_NE(order, std::string::npos);
  text.erase(order, std::string("  max_fault_order: 1\n").size());
  const auto intervals = text.find("  intervals: 10\n");
  ASSERT_NE(intervals, std::string::npos);
  text.erase(intervals, std::string("  intervals: 10\n").size());
  const auto manifest = text.find("  manifest_path: integrity_fault_manifest.yaml");
  ASSERT_NE(manifest, std::string::npos);
  text.replace(manifest,
               std::string("  manifest_path: integrity_fault_manifest.yaml").size(),
               "  manifest_path: " + kManifestPath);
  const auto config =
      uwb_imu_pl::IntegrityConfigLoader::load(writeTemp(text, "legacy_only"));
  ASSERT_TRUE(config.fault_models.max_fault_order.has_value());
  EXPECT_EQ(*config.fault_models.max_fault_order, 1);
  EXPECT_FALSE(config.migration_warnings.empty());
  EXPECT_NE(config.resolved_yaml.find("max_fault_order: 1"),
            std::string::npos);
  EXPECT_NE(config.resolved_yaml.find("intervals: 10"), std::string::npos);
  EXPECT_NE(config.resolved_yaml.find("migration notes"), std::string::npos);
  EXPECT_TRUE(config.fault_manifest.has_value());
  EXPECT_NE(config.resolved_yaml.find("resolved_fault_manifest_digest"),
            std::string::npos);
}

TEST(ConfigMigration, StartupRejectsManifestWithEnabledUnsupportedFamily) {
  std::string manifest = readFile(kManifestPath);
  const auto unsupported = manifest.find("unsupported_families:");
  const auto events = manifest.find("events:", unsupported);
  manifest.replace(unsupported, events - unsupported,
                   "unsupported_families:\n  - two_uwb\n  - imu_imu\n\n");
  const auto enabled = manifest.find("enabled_pair_families:");
  manifest.replace(enabled, std::string("enabled_pair_families:").size(),
                   "enabled_pair_families:\n  - same_device_multiaxis");
  const std::string manifest_path = writeTemp(manifest, "startup_manifest");
  const std::string path = tempConfig(
      "startup",
      {{"manifest_path: integrity_fault_manifest.yaml",
        "manifest_path: " + manifest_path}});
  EXPECT_TRUE(loadThrows(path, "unsupported model_type"));
}

TEST(FailureTaxonomy, MapsExistingReasonTexts) {
  using uwb_imu_pl::FailureReason;
  using uwb_imu_pl::classifyFailureReason;
  EXPECT_EQ(classifyFailureReason(""), FailureReason::None);
  EXPECT_EQ(classifyFailureReason("window state is rank deficient"),
            FailureReason::DangerousFaultNullspace);
  EXPECT_EQ(classifyFailureReason("remaining post-FDE fault is unmonitorable"),
            FailureReason::DangerousFaultNullspace);
  EXPECT_EQ(classifyFailureReason("candidate linearization step gate failed"),
            FailureReason::LinearizationUncertified);
  EXPECT_EQ(classifyFailureReason("outcome-conditioned risk budget does not close"),
            FailureReason::RiskBudgetInfeasible);
  EXPECT_EQ(classifyFailureReason("history prior contaminated"),
            FailureReason::HistorySummaryInvalid);
  EXPECT_EQ(classifyFailureReason("some novel text"),
            FailureReason::UnknownText);
  EXPECT_NE(uwb_imu_pl::failureReasonCatalogJson().find(
                "OUTPUT_IDENTITY_MISMATCH"),
            std::string::npos);
}

TEST(ResultIdentity, DigestIsStableAndSensitive) {
  uwb_imu_pl::IntegritySnapshotIdentity identity;
  identity.snapshot_id = "window:7:transaction:7";
  identity.source_revision = uwb_imu_pl::kNotAvailableInSchema;
  identity.config_digest = "abc123";
  identity.manifest_digest = "def456";
  identity.sensor_timestamp_ns = 123456789;
  identity.position_reference = "body_origin";
  const std::string first = uwb_imu_pl::identityDigest(&identity);
  EXPECT_EQ(first.size(), 16u);
  EXPECT_EQ(uwb_imu_pl::identityDigest(&identity), first);
  identity.sensor_timestamp_ns += 1;
  EXPECT_NE(uwb_imu_pl::identityDigest(&identity), first);
  const std::string serialized = uwb_imu_pl::serializeIdentity(identity);
  EXPECT_NE(serialized.find("position_reference=body_origin"),
            std::string::npos);
  EXPECT_NE(serialized.find("identity_digest="), std::string::npos);
}

TEST(DiagnosticsV12, LoggerExportsFailureAccountingIdentityAndSquareRootTables) {
  const std::string directory = "/tmp/uwb_imu_pl_p2_logger_v11";
  boost::filesystem::remove_all(directory);
  {
    uwb_imu_pl::RunLogger logger(directory, false, true);
    uwb_imu_pl::IntegrityOutput output;
    output.timestamp = uwb_imu_pl::TimestampNs(1000);
    output.diagnostics.input_attempt_id = 1;
    output.diagnostics.input_timestamp = output.timestamp;
    output.diagnostics.primary_failure = "LINEARIZATION_UNCERTIFIED";
    output.diagnostics.all_failures = "LINEARIZATION_UNCERTIFIED";
    output.diagnostics.not_evaluated_checks = "POST_FDE_DETECTOR_AND_PL";
    output.snapshot_identity.snapshot_id = "window:1:transaction:1";
    output.snapshot_identity.position_reference = "body_origin";
    output.snapshot_identity.identity_digest = "0123456789abcdef";
    // B1: the square-root audit row must be exported with the certificate.
    uwb_imu_pl::SquareRootAuditRecord audit;
    audit.attempt_id = 1;
    audit.rows = 253;
    audit.columns = 165;
    audit.rank = 165;
    audit.dof = 88;
    audit.condition_estimate = 328352.7;
    audit.certificate_ok = true;
    audit.usable = true;
    audit.scale_policy = "unit";
    audit.permutation_policy = "natural";
    output.square_root_audit.push_back(audit);
    output.diagnostics.status = "EXECUTED";
    logger.writeIntegrity(output);
    logger.flush();
  }
  std::ifstream attempts(directory + "/diagnostic_attempts.csv");
  std::string attempt_header;
  std::getline(attempts, attempt_header);
  std::string attempt_row;
  std::getline(attempts, attempt_row);
  EXPECT_NE(attempt_header.find("primary_failure"), std::string::npos);
  EXPECT_NE(attempt_header.find("all_failures"), std::string::npos);
  EXPECT_NE(attempt_header.find("not_evaluated_checks"), std::string::npos);
  EXPECT_NE(attempt_header.find("oracle_sweep_verified"), std::string::npos);
  EXPECT_NE(attempt_row.find("gate-d-diagnostics/v12"), std::string::npos);
  std::ifstream identity(directory + "/diagnostic_snapshot_identity.csv");
  ASSERT_TRUE(identity.good());
  std::string identity_header;
  std::getline(identity, identity_header);
  EXPECT_NE(identity_header.find("identity_digest"), std::string::npos);
  EXPECT_NE(identity_header.find("position_reference"), std::string::npos);
  std::string row;
  std::getline(identity, row);
  EXPECT_NE(row.find("window:1:transaction:1"), std::string::npos);
  std::ifstream square_root(directory + "/diagnostic_square_root.csv");
  ASSERT_TRUE(square_root.good());
  std::string square_root_header;
  std::getline(square_root, square_root_header);
  EXPECT_NE(square_root_header.find("certificate_ok"), std::string::npos);
  EXPECT_NE(square_root_header.find("detector_only_rows"), std::string::npos);
  EXPECT_NE(square_root_header.find("condition_estimate"), std::string::npos);
  std::string square_root_row;
  std::getline(square_root, square_root_row);
  EXPECT_NE(square_root_row.find("gate-d-diagnostics/v12"), std::string::npos);
  EXPECT_NE(square_root_row.find("unit"), std::string::npos);
  boost::filesystem::remove_all(directory);
}
