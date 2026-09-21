#include "uwb_imu_pl/config/integrity_config.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

const std::string kResearchConfig =
    std::string(UWB_IMU_PL_SOURCE_DIR) +
    "/config/realtime_uwb_imu_pl_research.yaml";

std::string readConfig() {
  std::ifstream input(kResearchConfig);
  std::ostringstream text;
  text << input.rdbuf();
  return text.str();
}

std::string replaceOnce(std::string text, const std::string& from,
                        const std::string& to) {
  const auto position = text.find(from);
  if (position == std::string::npos) {
    throw std::runtime_error("test mutation pattern not found: " + from);
  }
  text.replace(position, from.size(), to);
  return text;
}

std::string withFixedLag(std::string text, const std::string& value) {
  const std::string prefix = "  fixed_lag_epochs:";
  const auto position = text.find(prefix);
  if (position == std::string::npos) {
    throw std::runtime_error("fixed_lag_epochs key not found");
  }
  const auto end = text.find('\n', position);
  text.replace(position, end - position,
               prefix + " " + value);
  return text;
}

std::string writeTemp(const std::string& text, int index) {
  // A3: temp copies live in /tmp, so the relative manifest path of the shipped
  // research configuration is rewritten to the repository file.  The manifest
  // itself is loaded and cross-checked in every case.
  std::string body = text;
  const std::string relative =
      "  manifest_path: integrity_fault_manifest.yaml";
  const auto manifest = body.find(relative);
  if (manifest != std::string::npos) {
    body.replace(manifest, relative.size(),
                 "  manifest_path: " + std::string(UWB_IMU_PL_SOURCE_DIR) +
                     "/config/integrity_fault_manifest.yaml");
  }
  const std::string path = "/tmp/uwb_imu_pl_integrity_config_" +
      std::to_string(index) + ".yaml";
  std::ofstream output(path);
  output << body;
  return path;
}

void expectRejected(const std::string& text, int index) {
  EXPECT_THROW(
      uwb_imu_pl::IntegrityConfigLoader::load(writeTemp(text, index)),
      std::exception);
}

}  // namespace

TEST(IntegrityConfig, LoadsStrictResearchConfiguration) {
  const auto config = uwb_imu_pl::IntegrityConfigLoader::load(kResearchConfig);
  EXPECT_DOUBLE_EQ(config.risk.p_hmi_total, 4.0e-5);
  EXPECT_DOUBLE_EQ(config.imu.max_gap_s, 0.02);
  EXPECT_EQ(config.risk.hypotheses.size(), config.anchors.size());
  EXPECT_FALSE(config.incremental.enable_method_b);
  EXPECT_EQ(config.schema_version, "uwb-imu-pl/v5");
  EXPECT_DOUBLE_EQ(config.detector.p_fa_per_test, 1.0e-6);
  EXPECT_EQ(config.detector.continuity_horizon_tests, 1000u);
  EXPECT_EQ(config.integrity_window.epochs, 10u);
  EXPECT_EQ(config.integrity_window.recovery_margin_epochs, 10u);
  EXPECT_EQ(config.fault_models.max_cardinality, 2u);
  EXPECT_TRUE(config.fault_models.single_faults_enabled);
  EXPECT_FALSE(config.fault_models.double_faults_enabled);
  EXPECT_EQ(uwb_imu_pl::enabledFaultHypothesisCardinality(
                config.fault_models), 1u);
  EXPECT_EQ(config.fde.max_candidate_count, 128u);
  EXPECT_EQ(config.fde.max_exclusion_cardinality, 2u);
  EXPECT_TRUE(config.incremental.single_transaction_per_epoch);
  EXPECT_TRUE(config.incremental.fixed_lag_epochs == 0u ||
              config.incremental.fixed_lag_epochs >
                  config.integrity_window.epochs +
                  config.integrity_window.recovery_margin_epochs);
  EXPECT_FALSE(config.resolved_yaml.empty());
  EXPECT_EQ(config.resolved_yaml.back(), '\n');
}

TEST(IntegrityConfig, SupportsValidFaultCardinalityPolicies) {
  const std::string base = readConfig();
  const auto load = [&](bool single, bool double_enabled, int index) {
    auto text = replaceOnce(
        base, "  single_faults_enabled: true\n",
        std::string("  single_faults_enabled: ") +
            (single ? "true\n" : "false\n"));
    text = replaceOnce(
        text, "  double_faults_enabled: false\n",
        std::string("  double_faults_enabled: ") +
            (double_enabled ? "true\n" : "false\n"));
    // A3 migration: keep the canonical order key consistent with the legacy
    // switches instead of silently overriding either spelling.
    text = replaceOnce(
        text, "  max_fault_order: 1\n",
        std::string("  max_fault_order: ") + (double_enabled ? "2\n" : "1\n"));
    return uwb_imu_pl::IntegrityConfigLoader::load(writeTemp(text, index));
  };
  const auto single = load(true, false, 60);
  EXPECT_EQ(uwb_imu_pl::enabledFaultHypothesisCardinality(
                single.fault_models), 1u);
  const auto both = load(true, true, 61);
  EXPECT_EQ(uwb_imu_pl::enabledFaultHypothesisCardinality(
                both.fault_models), 2u);
  const auto double_only = load(false, true, 62);
  EXPECT_EQ(uwb_imu_pl::enabledFaultHypothesisCardinality(
                double_only.fault_models), 2u);
  EXPECT_EQ(double_only.fde.max_exclusion_cardinality, 2u);
  auto neither = replaceOnce(base, "  single_faults_enabled: true\n",
                             "  single_faults_enabled: false\n");
  expectRejected(neither, 63);
}

TEST(IntegrityConfig, ValidatesDoubleFaultSubtypesAndLegacyDefaults) {
  const std::string base = readConfig();
  auto double_enabled = replaceOnce(base,
      "  double_faults_enabled: false\n", "  double_faults_enabled: true\n");
  double_enabled = replaceOnce(double_enabled, "  max_fault_order: 1\n",
                               "  max_fault_order: 2\n");
  auto none = replaceOnce(double_enabled, "    uwb_plus_accel: true\n",
                          "    uwb_plus_accel: false\n");
  none = replaceOnce(none, "    uwb_plus_gyro: true\n",
                     "    uwb_plus_gyro: false\n");
  expectRejected(none, 64);
  const auto accel_only = uwb_imu_pl::IntegrityConfigLoader::load(
      writeTemp(replaceOnce(double_enabled, "    uwb_plus_gyro: true\n",
                            "    uwb_plus_gyro: false\n"), 65));
  EXPECT_TRUE(accel_only.fault_models.combinations.uwb_plus_accel);
  EXPECT_FALSE(accel_only.fault_models.combinations.uwb_plus_gyro);
  expectRejected(replaceOnce(double_enabled, "    two_uwb: false\n",
                             "    two_uwb: true\n"), 66);

  auto legacy = replaceOnce(base, "  single_faults_enabled: true\n", "");
  legacy = replaceOnce(legacy, "  double_faults_enabled: false\n", "");
  legacy = replaceOnce(legacy, "  max_exclusion_cardinality: 2\n", "");
  const auto loaded = uwb_imu_pl::IntegrityConfigLoader::load(
      writeTemp(legacy, 67));
  EXPECT_TRUE(loaded.fault_models.single_faults_enabled);
  EXPECT_FALSE(loaded.fault_models.double_faults_enabled);
  EXPECT_EQ(loaded.fde.max_exclusion_cardinality, 2u);
  EXPECT_NE(loaded.resolved_yaml.find("single_faults_enabled: true"),
            std::string::npos);
  EXPECT_NE(loaded.resolved_yaml.find("double_faults_enabled: false"),
            std::string::npos);
  EXPECT_NE(loaded.resolved_yaml.find("max_exclusion_cardinality: 2"),
            std::string::npos);
}

TEST(IntegrityConfig, RejectsMissingRequiredFieldInEverySection) {
  const std::string base = readConfig();
  const std::string fields[] = {
      "schema_version: uwb-imu-pl/v5\n",
      "seed: 20260901\n",
      "  dimensions: 3\n",
      "  relinearize_threshold: 0.1\n",
      "  accelerometer_sigma: 0.10\n",
      "  p_fa_per_test: 1.0e-6\n",
      "integrity_window:\n",
      "fault_models:\n",
      "fde:\n",
      "bridge:\n",
      "health:\n",
      "risk:\n",
      "robust_shadow:\n",
      "  root: results/realtime_uwb_imu_pl\n",
      "  world_frame: world\n",
      "anchors:\n"};
  int index = 0;
  for (const auto& field : fields) {
    expectRejected(replaceOnce(base, field, ""), index++);
  }
}

TEST(IntegrityConfig, RejectsUnknownRootSectionAndAnchorKeys) {
  const std::string base = readConfig();
  expectRejected("unknown_root: 1\n" + base, 20);
  expectRejected(replaceOnce(base, "snapshot:\n", "snapshot:\n  unknown: 1\n"),
                 21);
  expectRejected(replaceOnce(base, "  - {id: 1,", "  - {unknown: 1, id: 1,"),
                 22);
}

TEST(IntegrityConfig, RejectsInvalidRiskAndUnsupportedOnlineModes) {
  const std::string base = readConfig();
  expectRejected(replaceOnce(base, "p_hmi_total: 4.0e-5",
                             "p_hmi_total: 1.0e-8"), 30);
  expectRejected(replaceOnce(base, "enable_method_b: false",
                             "enable_method_b: true"), 31);
  expectRejected(withFixedLag(base, "1"), 32);
  expectRejected(withFixedLag(base, "-1"), 33);
  expectRejected(withFixedLag(base, "4294967296"), 34);
}

TEST(IntegrityConfig, RuntimeOverrideAcceptsFullHistoryAndFixedLag) {
  for (const std::uint32_t value : {0u, 31u, 400u}) {
    const auto loaded = uwb_imu_pl::IntegrityConfigLoader::load(
        kResearchConfig, std::to_string(value));
    EXPECT_EQ(loaded.incremental.fixed_lag_epochs, value);
    EXPECT_NE(loaded.resolved_yaml.find(
                  "fixed_lag_epochs: " + std::to_string(value)),
              std::string::npos);
  }
}

TEST(IntegrityConfig, RetainsV4ConfigurationReadCompatibility) {
  std::string legacy = readConfig();
  while (legacy.find("uwb-imu-pl/v5") != std::string::npos) {
    legacy = replaceOnce(legacy, "uwb-imu-pl/v5", "uwb-imu-pl/v4");
  }
  const auto loaded = uwb_imu_pl::IntegrityConfigLoader::load(
      writeTemp(legacy, 41));
  EXPECT_EQ(loaded.schema_version, "uwb-imu-pl/v4");
  EXPECT_EQ(loaded.output.schema_version, "uwb-imu-pl/v4");
}

TEST(IntegrityConfig, RuntimeOverrideRejectsIllegalValues) {
  for (const std::string& value : {"1", "-1", "4294967296", "2.5", "x"}) {
    EXPECT_THROW(uwb_imu_pl::IntegrityConfigLoader::load(kResearchConfig,
                                                         value),
                 std::exception)
        << value;
  }
}

TEST(IntegrityConfig, HashCoversResolvedConfigurationAfterOverride) {
  const auto full_history = uwb_imu_pl::IntegrityConfigLoader::load(
      kResearchConfig, "0");
  const auto fixed_lag = uwb_imu_pl::IntegrityConfigLoader::load(
      kResearchConfig, "200");
  EXPECT_NE(full_history.config_hash, fixed_lag.config_hash);
  EXPECT_NE(full_history.resolved_yaml, fixed_lag.resolved_yaml);

  const auto reloaded = uwb_imu_pl::IntegrityConfigLoader::load(
      writeTemp(full_history.resolved_yaml, 50));
  EXPECT_EQ(reloaded.incremental.fixed_lag_epochs, 0u);
  // A3: migration notes are part of the hashed resolved configuration, so a
  // reload is not byte-identical to the original run that still carried the
  // legacy aliases.  The required invariant is idempotence: loading the
  // already-migrated dump again reproduces the same effective hash and text.
  const auto reloaded_again = uwb_imu_pl::IntegrityConfigLoader::load(
      writeTemp(reloaded.resolved_yaml, 51));
  EXPECT_EQ(reloaded_again.config_hash, reloaded.config_hash);
  EXPECT_EQ(reloaded_again.resolved_yaml, reloaded.resolved_yaml);
  EXPECT_EQ(reloaded.fault_models.max_fault_order,
            full_history.fault_models.max_fault_order);
  EXPECT_EQ(reloaded.integrity_window.epochs,
            full_history.integrity_window.epochs);
}

TEST(IntegrityConfig, UnifiedOverridesAreResolvedAndHashed) {
  uwb_imu_pl::IntegrityConfigOverrides overrides;
  overrides.seed = 20260902;
  overrides.fixed_lag_epochs = 50;
  overrides.write_global_diagnostics = true;
  overrides.write_residuals = false;
  overrides.write_timing = false;
  overrides.output_root = "results/week4_fixture";
  const auto loaded = uwb_imu_pl::IntegrityConfigLoader::load(
      kResearchConfig, overrides);
  EXPECT_EQ(loaded.seed, 20260902u);
  EXPECT_EQ(loaded.incremental.fixed_lag_epochs, 50u);
  EXPECT_TRUE(loaded.output.write_global_diagnostics);
  EXPECT_FALSE(loaded.output.write_residuals);
  EXPECT_FALSE(loaded.output.write_timing);
  EXPECT_EQ(loaded.output.root, "results/week4_fixture");
  EXPECT_NE(loaded.resolved_yaml.find("seed: 20260902"), std::string::npos);
  EXPECT_NE(loaded.resolved_yaml.find("root: results/week4_fixture"),
            std::string::npos);
  EXPECT_NE(loaded.config_hash,
            uwb_imu_pl::IntegrityConfigLoader::load(kResearchConfig).config_hash);
}

TEST(IntegrityConfig, DevelopmentManifestHashTracksAppendedOverrides) {
  const auto base = uwb_imu_pl::IntegrityConfigLoader::load(kResearchConfig);
  const std::string resolved = base.resolved_yaml +
      "development_formal_eligible: false\n";
  EXPECT_NE(uwb_imu_pl::IntegrityConfigLoader::hashResolvedYaml(resolved),
            base.config_hash);
  EXPECT_EQ(uwb_imu_pl::IntegrityConfigLoader::hashResolvedYaml(
                base.resolved_yaml),
            base.config_hash);
}
