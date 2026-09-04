#include "uwb_imu_pl/io/run_logger.hpp"

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <algorithm>
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

std::size_t lineCount(const std::string& path) {
  std::ifstream input(path);
  return static_cast<std::size_t>(std::count(
      std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>(), '\n'));
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

TEST(RunLogger, V4SchemaHasExactHeadersAndStructuredSummary) {
  const std::string directory = "/tmp/uwb_imu_pl_logger_v2";
  boost::filesystem::remove_all(directory);
  {
    uwb_imu_pl::RunLogger logger(directory, true, true);
    uwb_imu_pl::RunManifest manifest;
    manifest.git_sha = "254f290";
    manifest.config_hash = "0123456789abcdef";
    manifest.fixed_lag_epochs = 200;
    manifest.execution_command =
        "roslaunch uwb_imu_pl realtime.launch fixed_lag_epochs:=200";
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
            "hmi_risk_requirement,batch_committed,transaction_id,window_id,"
            "base_graph_version,linearization_version,selected_action_id,"
            "selected_action_type,fde_status,bridge_pl_x,bridge_pl_y,bridge_pl_z,"
            "history_provenance_valid,backend_updates,stale_state,"
            "controlled_reinitialization_required,historical_groups_removed,"
            "historical_groups_added,recovery_epoch_begin,recovery_epoch_end,"
            "reinitialization_request_id,reinitialization_phase,"
            "reinitialization_reason,reason");
  EXPECT_EQ(firstLine(directory + "/timing.csv"),
            "timestamp_ns,epoch,stage,wall_ms,problem_size,hypothesis_count,"
            "factor_count,cold_warm,success");
  EXPECT_EQ(firstLine(directory + "/events.csv"),
            "timestamp_ns,sequence,event,detail");
  EXPECT_TRUE(boost::filesystem::exists(directory + "/ground_truth.csv"));
  EXPECT_TRUE(boost::filesystem::exists(directory + "/fault_truth.csv"));
  EXPECT_TRUE(boost::filesystem::exists(directory + "/transactions.csv"));
  EXPECT_TRUE(boost::filesystem::exists(directory + "/hypotheses.csv"));
  EXPECT_TRUE(boost::filesystem::exists(directory + "/candidates.csv"));
  EXPECT_TRUE(boost::filesystem::exists(directory + "/factor_ledger.csv"));
  EXPECT_TRUE(boost::filesystem::exists(directory + "/health.csv"));
  EXPECT_TRUE(boost::filesystem::exists(directory + "/bridge.csv"));
  std::ifstream summary(directory + "/summary.json");
  const std::string json((std::istreambuf_iterator<char>(summary)), {});
  EXPECT_NE(json.find("\"processed\": 3"), std::string::npos);
  EXPECT_NE(json.find("\"committed\": 2"), std::string::npos);
  std::ifstream manifest_input(directory + "/run_manifest.json");
  const std::string manifest_json(
      (std::istreambuf_iterator<char>(manifest_input)), {});
  EXPECT_NE(manifest_json.find("\"fixed_lag_epochs\": 200"),
            std::string::npos);
  EXPECT_NE(manifest_json.find("\"execution_command\":"),
            std::string::npos);
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

TEST(RunLogger, V4AuditTablesWriteDataRows) {
  const std::string directory = "/tmp/uwb_imu_pl_logger_v4_audit";
  boost::filesystem::remove_all(directory);
  {
    uwb_imu_pl::RunLogger logger(directory, false, false);
    uwb_imu_pl::IntegrityOutput output;
    output.timestamp = uwb_imu_pl::TimestampNs(42);
    output.transaction_id = 7;
    output.window_id = 8;
    output.hypothesis_audit.push_back({1, "11;12", 1e-4, 1e-3, 1e-6,
                                       true, true, 2.0, 3.0, "ok"});
    output.candidate_audit.push_back({2, "UWB_ANCHOR_EXCLUSION", 1, true,
                                     true, true, 1.0, 4.0, 3, 2, 5.0,
                                     0.5, 0.7, true, "ok"});
    uwb_imu_pl::FactorLedgerAuditRecord factor;
    factor.factor_id = 3;
    factor.group_id = 4;
    factor.sensor = "UWB";
    factor.factor_kind = "UWB_BATCH";
    factor.lifecycle = "ACTIVE";
    output.factor_ledger_audit.push_back(factor);
    uwb_imu_pl::HealthAuditRecord health;
    health.source_id = "anchor:1";
    health.sensor = "UWB";
    health.previous_state = health.current_state = "HEALTHY";
    output.health_audit.push_back(health);
    output.bridge_audit = uwb_imu_pl::BridgeAuditRecord{};
    logger.writeIntegrity(output);
  }
  EXPECT_EQ(lineCount(directory + "/hypotheses.csv"), 2u);
  EXPECT_EQ(lineCount(directory + "/candidates.csv"), 2u);
  EXPECT_EQ(lineCount(directory + "/factor_ledger.csv"), 2u);
  EXPECT_EQ(lineCount(directory + "/health.csv"), 2u);
  EXPECT_EQ(lineCount(directory + "/bridge.csv"), 2u);
  boost::filesystem::remove_all(directory);
}
