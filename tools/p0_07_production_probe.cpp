#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/action_hypothesis_audit.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>

namespace {
using namespace uwb_imu_pl;

template <typename Matrix>
void writeMatrix(std::ostream& out, const Matrix& matrix) {
  out << matrix.rows() << '\t' << matrix.cols();
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      out << '\t' << matrix(row, column);
    }
  }
}

void writeSnapshot(const ActionHypothesisProofSnapshotV1& snapshot,
                   const IntegrityOutput& output, std::ostream& out) {
  out << std::setprecision(17);
  out << "P007_DECISION_V1\t" << output.selected_action_id << '\t'
      << output.batch_committed << '\t'
      << static_cast<int>(output.protection_level.availability) << '\t'
      << output.protection_level.formal_eligible << '\t'
      << output.publication.protected_output << '\t'
      << output.diagnostics.generated_actions << '\t'
      << output.diagnostics.kernel_evaluated_actions << '\t'
      << std::quoted(output.diagnostics.status) << '\t'
      << std::quoted(output.fde_status) << '\t'
      << std::quoted(output.protection_level.reason) << '\n';
  out << "P007_ACTUAL_V1\t" << snapshot.transaction_id << '\t'
      << snapshot.window_id << '\t' << snapshot.selected_action_id << '\t'
      << snapshot.snapshot_identity << '\t' << snapshot.served_rank << '\t'
      << snapshot.served_dof << '\t' << snapshot.served_statistic << '\t'
      << output.diagnostics.hypothesis_count << '\t'
      << output.diagnostics.generated_actions << '\t'
      << output.diagnostics.kernel_evaluated_actions << '\t'
      << output.protection_level.formal_eligible << '\t'
      << output.publication.protected_output << '\n';
  out << "RISK\t" << output.diagnostics.risk_nominal << '\t'
      << output.diagnostics.risk_p_nm << '\t'
      << output.diagnostics.risk_hypotheses << '\t'
      << output.diagnostics.risk_total << '\t'
      << output.diagnostics.risk_upper_bound << '\n';
  out << "OBSERVED\t" << snapshot.plausible_hypothesis_ids.size();
  for (const auto id : snapshot.plausible_hypothesis_ids) out << '\t' << id;
  out << '\t' << output.diagnostics.generated_actions << '\t'
      << output.selected_action_id << '\t'
      << std::quoted("same-run production sidecar") << '\n';
  out << "SERVED_H\t"; writeMatrix(out, snapshot.served_h); out << '\n';
  out << "SERVED_Z\t"; writeMatrix(out, snapshot.served_z); out << '\n';
  out << "BASE_INFORMATION\t";
  writeMatrix(out, snapshot.base_information); out << '\n';
  out << "BASE_RHS\t"; writeMatrix(out, snapshot.base_information_rhs);
  out << '\n';
  out << "BASE_STATE\t"; writeMatrix(out, snapshot.base_state_increment);
  out << '\n';
  out << "PROTECTED_MAP\t"; writeMatrix(out, snapshot.protected_state_map);
  out << '\n';
  out << "HISTORY_RESPONSE\t"; writeMatrix(out, snapshot.history_response);
  out << '\n';
  out << "HISTORY_DETECTOR_RESPONSE\t";
  writeMatrix(out, snapshot.history_detector_response); out << '\n';
  out << "HISTORY_D_PERP\t"; writeMatrix(out, snapshot.history_d_perp);
  out << '\n';
  for (const auto& column : snapshot.history_columns) {
    out << "HISTORY_COLUMN\t" << column.kind << '\t' << column.source
        << '\t' << column.epoch << '\n';
  }
  auto write_block = [&](const char* label, const RawFactorBlockAuditV1& block) {
    out << label << '\t' << block.group_id << '\t' << block.factor_kind
        << '\t' << block.sensor << '\t' << block.row_role << '\t';
    writeMatrix(out, block.jacobian_raw); out << '\t';
    writeMatrix(out, block.residual_raw); out << '\t';
    writeMatrix(out, block.covariance); out << '\t';
    writeMatrix(out, block.jacobian_whitened); out << '\t';
    writeMatrix(out, block.residual_whitened); out << '\t'
        << block.window_column_indices.size();
    for (const int column : block.window_column_indices) out << '\t' << column;
    out << '\n';
  };
  for (const auto& block : snapshot.raw_blocks) {
    write_block("BLOCK", block);
  }
  for (const auto& action : snapshot.action_inputs) {
    out << "ACTION_INPUT\t" << action.action_id << '\t'
        << action.removed_group_ids.size();
    for (const auto group : action.removed_group_ids) out << '\t' << group;
    out << '\t' << action.added_blocks.size() << '\n';
    for (const auto& block : action.added_blocks) {
      out << "ADDED_BLOCK\t" << action.action_id << '\t';
      // The action id precedes the ordinary block payload.
      out << block.group_id << '\t' << block.factor_kind << '\t'
          << block.sensor << '\t' << block.row_role << '\t';
      writeMatrix(out, block.jacobian_raw); out << '\t';
      writeMatrix(out, block.residual_raw); out << '\t';
      writeMatrix(out, block.covariance); out << '\t';
      writeMatrix(out, block.jacobian_whitened); out << '\t';
      writeMatrix(out, block.residual_whitened); out << '\t'
          << block.window_column_indices.size();
      for (const int column : block.window_column_indices) out << '\t' << column;
      out << '\n';
    }
  }
  for (const auto& owner : snapshot.raw_row_owners) {
    out << "OWNER\t" << std::quoted(owner.row_id) << '\t'
        << owner.owner_group_id << '\t' << owner.row_in_group << '\t'
        << std::quoted(owner.covariance_placement) << '\n';
  }
  for (const auto& record : snapshot.records) {
    out << "PROOF\t" << record.action_id << '\t' << record.hypothesis_id
        << '\t' << record.candidate_proof_identity << '\t'
        << record.protection_proof_identity << '\t'
        << record.hypothesis_proof_identity << '\t'
        << record.candidate_row_identity << '\t' << record.candidate_rows
        << '\t' << record.proof_available << '\t'
        << std::quoted(record.proof_unavailable_reason) << '\t';
    writeMatrix(out, record.gram); out << '\t';
    writeMatrix(out, record.protected_response); out << '\t';
    writeMatrix(out, record.protected_slopes); out << '\t'
        << record.nullspace_class << '\t';
    writeMatrix(out, record.pl_contribution_m); out << '\t'
        << record.prior_probability_bound << '\t' << record.p_md_allocation
        << '\t' << record.hmi_allocation << '\t' << record.hypothesis_tail
        << '\t' << record.candidate_valid << '\t'
        << record.post_detector_passed << '\t'
        << record.candidate_disposition << '\t' << record.action_eligible
        << '\t' << record.action_selected << '\t'
        << std::quoted(record.refusal) << '\n';
  }
}
}  // namespace

int main(int argc, char** argv) {
  using namespace uwb_imu_pl;
  try {
    if (argc != 5 && argc != 6) {
      std::cerr << "usage: p0_07_production_probe CONFIG RAW_INPUT TRUTH OUTPUT [--no-audit]\n";
      return 2;
    }
    const bool audit_enabled = argc == 5;
    IntegrityConfig config = IntegrityConfigLoader::load(argv[1]);
    const YAML::Node raw = YAML::LoadFile(argv[2]);
    const YAML::Node truth = YAML::LoadFile(argv[3]);
    const std::size_t epochs = raw["epochs"].as<std::size_t>();
    const std::size_t alarm_epoch = truth["fault"]["epoch"].as<std::size_t>();
    const std::size_t samples = raw["imu"]["samples_per_epoch"].as<std::size_t>();
    const std::int64_t imu_period = raw["imu"]["period_ns"].as<std::int64_t>();
    const std::int64_t epoch_period =
        static_cast<std::int64_t>(samples) * imu_period;
    config.incremental.fixed_lag_epochs = alarm_epoch - 3 + 1;
    config.publication.protected_output_enabled = true;
    config.publication.deadline_ms = 1e9;
    setActionHypothesisProofAuditEnabledV1(audit_enabled);

    std::map<std::size_t, UwbBatch> batches;
    for (const auto& row : raw["uwb_rows"]) {
      const std::size_t epoch = row["epoch"].as<std::size_t>();
      auto& batch = batches[epoch];
      batch.id = BatchId(700000 + epoch);
      batch.timestamp = TimestampNs(row["timestamp_ns"].as<std::int64_t>());
      batch.covariance_model_id = "p007-production-correlated-replay";
      UwbMeasurement measurement;
      measurement.id = MeasurementId(row["measurement_id"].as<std::uint64_t>());
      measurement.factor_id = FactorId(measurement.id.value());
      measurement.anchor_id = AnchorId(row["anchor_id"].as<std::uint64_t>());
      measurement.timestamp = batch.timestamp;
      for (int axis = 0; axis < 3; ++axis) {
        measurement.anchor_position_m(axis) =
            row["anchor_position_m"][axis].as<double>();
      }
      measurement.range_m = row["range_m"].as<double>();
      measurement.sigma_m = row["sigma_m"].as<double>();
      batch.measurements.push_back(std::move(measurement));
    }
    const double diagonal = raw["covariance"]["diagonal_m2"].as<double>();
    const double off_diagonal =
        raw["covariance"]["off_diagonal_m2"].as<double>();
    for (auto& item : batches) {
      const Eigen::Index count = item.second.measurements.size();
      item.second.covariance_m2 = Eigen::MatrixXd::Constant(
          count, count, off_diagonal);
      item.second.covariance_m2.diagonal().setConstant(diagonal);
    }

    IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
    NavigationState initial;
    initial.timestamp = TimestampNs(raw["imu"]["boundary_timestamp_ns"]
                                        .as<std::int64_t>());
    initial.position_world_m = {0, 0, 1};
    estimator.initialize(initial, config.realtime.prior_sigmas);
    ImuMeasurement boundary;
    boundary.id = MeasurementId(900000);
    boundary.timestamp = initial.timestamp;
    for (int axis = 0; axis < 3; ++axis) {
      boundary.specific_force_mps2(axis) =
          raw["imu"]["specific_force_mps2"][axis].as<double>();
      boundary.angular_velocity_radps(axis) =
          raw["imu"]["angular_velocity_radps"][axis].as<double>();
    }
    estimator.ingestImu(boundary);
    RealtimeIntegrityPipeline pipeline(
        &estimator,
        IntegrityMonitor(config.risk, config.integrity_window.rank_tolerance,
                         config.integrity_window.max_condition_number));
    IntegrityOutput alarm_output;
    for (std::size_t epoch = 1; epoch <= epochs; ++epoch) {
      for (std::size_t sample = 1; sample <= samples; ++sample) {
        ImuMeasurement imu = boundary;
        imu.id = MeasurementId(900000 + epoch * 10 + sample);
        imu.timestamp = TimestampNs(
            static_cast<std::int64_t>(epoch - 1) * epoch_period +
            static_cast<std::int64_t>(sample) * imu_period);
        estimator.ingestImu(imu);
      }
      ClockSample clock;
      clock.wall_monotonic_ns = 1000000000 + epoch * epoch_period;
      clock.sensor_timestamp_ns = epoch * epoch_period;
      IntegrityOutput output = pipeline.processUwbBatch(batches.at(epoch), clock);
      if (epoch == alarm_epoch) alarm_output = std::move(output);
    }
    ActionHypothesisProofSnapshotV1 snapshot;
    if (audit_enabled && !actionHypothesisProofAuditV1(alarm_output, &snapshot)) {
      throw std::runtime_error("same-run action proof snapshot unavailable");
    }
    std::ofstream output(argv[4], std::ios::trunc);
    if (!output) throw std::runtime_error("cannot open output protocol");
    if (audit_enabled) {
      std::set<std::uint64_t> input_anchors;
      for (const auto& row : raw["uwb_rows"]) {
        input_anchors.insert(row["anchor_id"].as<std::uint64_t>());
      }
      output << std::setprecision(17) << "INPUT\t" << epochs << '\t'
             << alarm_epoch << '\t' << samples << '\t' << imu_period << '\t'
             << input_anchors.size();
      for (const auto anchor : input_anchors) output << '\t' << anchor;
      output << '\n';
      writeSnapshot(snapshot, alarm_output, output);
    } else {
      output << std::setprecision(17) << "P007_DECISION_V1\t"
             << alarm_output.selected_action_id << '\t'
             << alarm_output.batch_committed << '\t'
             << static_cast<int>(alarm_output.protection_level.availability)
             << '\t' << alarm_output.protection_level.formal_eligible << '\t'
             << alarm_output.publication.protected_output << '\t'
             << alarm_output.diagnostics.generated_actions << '\t'
             << alarm_output.diagnostics.kernel_evaluated_actions << '\t'
             << std::quoted(alarm_output.diagnostics.status) << '\t'
             << std::quoted(alarm_output.fde_status) << '\t'
             << std::quoted(alarm_output.protection_level.reason) << '\n';
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
