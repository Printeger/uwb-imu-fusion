#include "uwb_imu_pl/io/run_logger.hpp"

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <sstream>
#include <vector>

namespace {
std::string firstLine(const std::string& path) {
  std::ifstream input(path);
  std::string line;
  std::getline(input, line);
  return line;
}

std::vector<std::string> fields(const std::string& line) {
  std::vector<std::string> result;
  std::istringstream input(line);
  std::string field;
  while (std::getline(input, field, ',')) result.push_back(field);
  return result;
}
}  // namespace

TEST(RunLogger, OptionalCsvCreationFollowsConfiguration) {
  const std::string disabled = "/tmp/uwb_imu_pl_logger_disabled";
  const std::string enabled = "/tmp/uwb_imu_pl_logger_enabled";
  boost::filesystem::remove_all(disabled);
  boost::filesystem::remove_all(enabled);
  {
    uwb_imu_pl::RunLogger logger(disabled, false, false);
    logger.writeResidual(uwb_imu_pl::TimestampNs(0), {}, {},
                         uwb_imu_pl::RowRole::Measurement, 1.0, 2.0);
    logger.writeTiming(uwb_imu_pl::TimestampNs(0), "test", 1.0, true);
  }
  EXPECT_FALSE(boost::filesystem::exists(disabled + "/residuals.csv"));
  EXPECT_FALSE(boost::filesystem::exists(disabled + "/timing.csv"));
  EXPECT_TRUE(boost::filesystem::exists(disabled + "/integrity.csv"));
  {
    uwb_imu_pl::RunLogger logger(enabled, true, true);
  }
  EXPECT_TRUE(boost::filesystem::exists(enabled + "/residuals.csv"));
  EXPECT_TRUE(boost::filesystem::exists(enabled + "/timing.csv"));
  boost::filesystem::remove_all(disabled);
  boost::filesystem::remove_all(enabled);
}

TEST(RunLogger, V2SchemaHasExactHeadersAndStructuredSummary) {
  const std::string directory = "/tmp/uwb_imu_pl_logger_v2";
  boost::filesystem::remove_all(directory);
  {
    uwb_imu_pl::RunLogger logger(directory, true, true);
    uwb_imu_pl::RunManifest manifest;
    manifest.git_sha = "254f290";
    manifest.config_hash = "0123456789abcdef";
    logger.writeManifest(manifest);
    uwb_imu_pl::RunSummary summary;
    summary.processed = 3;
    summary.committed = 2;
    summary.rejected = 1;
    logger.writeSummary(summary);
  }
  EXPECT_EQ(firstLine(directory + "/integrity.csv"),
            "timestamp_ns,group_size,measurement_model_valid,global_statistic,"
            "global_threshold,global_dof,global_passed,postfit_statistic,"
            "postfit_threshold,postfit_dof,postfit_passed,conditional_statistic,"
            "conditional_threshold,conditional_dof,conditional_passed,"
            "conditional_formal,pl_x,pl_y,pl_z,hpl_m,vpl_m,availability,label,"
            "formal_eligible,risk_budget_valid,allocated_hmi_risk,"
            "hmi_risk_requirement,batch_committed,reason");
  EXPECT_EQ(firstLine(directory + "/timing.csv"),
            "timestamp_ns,epoch,stage,wall_ms,problem_size,hypothesis_count,"
            "factor_count,cold_warm,success");
  EXPECT_EQ(firstLine(directory + "/events.csv"),
            "timestamp_ns,sequence,event,detail");
  EXPECT_TRUE(boost::filesystem::exists(directory + "/ground_truth.csv"));
  EXPECT_TRUE(boost::filesystem::exists(directory + "/fault_truth.csv"));
  std::ifstream summary(directory + "/summary.json");
  const std::string json((std::istreambuf_iterator<char>(summary)), {});
  EXPECT_NE(json.find("\"processed\": 3"), std::string::npos);
  EXPECT_NE(json.find("\"committed\": 2"), std::string::npos);
  boost::filesystem::remove_all(directory);
}

TEST(RunLogger, NumericallyInvalidConditionalIsNotMarkedFormal) {
  const std::string directory = "/tmp/uwb_imu_pl_logger_invalid_formal";
  boost::filesystem::remove_all(directory);
  {
    uwb_imu_pl::RunLogger logger(directory, false, false);
    uwb_imu_pl::IntegrityOutput output;
    output.detector.detector_type =
        "conditional_current_uwb_innovation_chi_square";
    output.detector.numerically_valid = false;
    logger.writeIntegrity(output);
  }
  std::ifstream input(directory + "/integrity.csv");
  std::string header;
  std::string row;
  std::getline(input, header);
  std::getline(input, row);
  const auto values = fields(row);
  ASSERT_GT(values.size(), 15u);
  EXPECT_EQ(values[15], "0");
  boost::filesystem::remove_all(directory);
}
