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

TEST(RunLogger, V5SchemaHasExactHeadersAndStructuredSummary) {
  const std::string directory = "/tmp/uwb_imu_pl_logger_v5";
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
  EXPECT_EQ(firstLine(directory + "/hypotheses.csv"),
            "timestamp_ns,window_id,hypothesis_id,fault_unit_ids,physical_source_ids,sensor,"
            "fault_kind,mode_ids,onset_epoch,onset_time_ns,parameter_dimension,"
            "fault_rank,sigma_min,sigma_max,condition_number,slope_x,slope_y,"
            "slope_z,boundary_direction_gram,noncentrality_boundary,prior_bound,p_md_allocation,"
            "hmi_allocation,monitorable,plausible,conditioned_statistic,"
            "log_evidence,reason");
  EXPECT_EQ(firstLine(directory + "/candidates.csv"),
            "timestamp_ns,window_id,action_id,action_type,physical_source_ids,removed_group_ids,"
            "added_group_ids,bridge_mode,cardinality,valid,post_detector_passed,"
            "covers_plausible_set,statistic,threshold,rank,dof,condition_number,"
            "information_logdet,risk_allocation,hpl_m,vpl_m,selected,"
            "evaluation_wall_ms,reason");
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

TEST(RunLogger, V5AuditTablesWriteDataRows) {
  const std::string directory = "/tmp/uwb_imu_pl_logger_v5_audit";
  boost::filesystem::remove_all(directory);
  {
    uwb_imu_pl::RunLogger logger(directory, false, false);
    uwb_imu_pl::IntegrityOutput output;
    output.timestamp = uwb_imu_pl::TimestampNs(42);
    output.transaction_id = 7;
    output.window_id = 8;
    uwb_imu_pl::HypothesisAuditRecord hypothesis;
    hypothesis.hypothesis_id = 1;
    hypothesis.fault_unit_ids = "11;12";
    hypothesis.physical_source_ids = "uwb:1;imu_accel:0:interval:4";
    hypothesis.sensor = "UWB";
    hypothesis.fault_kind = "ANCHOR_BIAS_RAMP";
    hypothesis.mode_ids = "4";
    hypothesis.parameter_dimension = 2;
    hypothesis.prior_bound = 1e-4;
    hypothesis.p_md_allocation = 1e-3;
    hypothesis.hmi_allocation = 1e-6;
    hypothesis.monitorable = true;
    hypothesis.plausible = true;
    hypothesis.reason = "ok";
    output.hypothesis_audit.push_back(hypothesis);
    uwb_imu_pl::CandidateAuditRecord candidate;
    candidate.action_id = 2;
    candidate.action_type = "UWB_ANCHOR_EXCLUSION";
    candidate.physical_source_ids = "uwb:1";
    candidate.removed_group_ids = "11";
    candidate.bridge_mode = "NONE";
    candidate.cardinality = 1;
    candidate.valid = true;
    candidate.post_detector_passed = true;
    candidate.covers_plausible_set = true;
    candidate.selected = true;
    candidate.reason = "ok";
    output.candidate_audit.push_back(candidate);
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
