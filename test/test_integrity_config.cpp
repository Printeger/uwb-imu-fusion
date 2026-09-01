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

std::string writeTemp(const std::string& text, int index) {
  const std::string path = "/tmp/uwb_imu_pl_integrity_config_" +
      std::to_string(index) + ".yaml";
  std::ofstream output(path);
  output << text;
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
  EXPECT_EQ(config.incremental.fixed_lag_epochs, 0u);
}

TEST(IntegrityConfig, RejectsMissingRequiredFieldInEverySection) {
  const std::string base = readConfig();
  const std::string fields[] = {
      "seed: 20260901\n",
      "  dimensions: 3\n",
      "  relinearize_threshold: 0.1\n",
      "  accelerometer_sigma: 0.10\n",
      "  p_fa: 1.0e-5\n",
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
  expectRejected(replaceOnce(base, "fixed_lag_epochs: 0",
                             "fixed_lag_epochs: 1"), 32);
}
