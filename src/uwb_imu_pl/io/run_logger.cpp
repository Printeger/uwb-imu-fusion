#include "uwb_imu_pl/io/run_logger.hpp"

#include "uwb_imu_pl/common/failure_reason.hpp"
#include "uwb_imu_pl/config/integrity_config.hpp"

#include <boost/filesystem.hpp>
#include <gtsam/config.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <sys/sysinfo.h>
#include <sys/utsname.h>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

std::string csv(const std::string& value) {
  std::string escaped = value;
  std::size_t pos = 0;
  while ((pos = escaped.find('"', pos)) != std::string::npos) {
    escaped.insert(pos, 1, '"');
    pos += 2;
  }
  return "\"" + escaped + "\"";
}

std::string json(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size() + 2);
  for (char c : value) {
    switch (c) {
      case '"': escaped += "\\\""; break;
      case '\\': escaped += "\\\\"; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += c; break;
    }
  }
  return "\"" + escaped + "\"";
}

std::string joinIds(const std::vector<std::uint64_t>& ids) {
  std::ostringstream out;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i) out << ';';
    out << ids[i];
  }
  return out.str();
}

template <typename Id>
std::string joinStrongIds(const std::vector<Id>& ids) {
  std::ostringstream out;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i) out << ';';
    out << ids[i].value();
  }
  return out.str();
}

std::string joinVector(const Eigen::VectorXd& values) {
  std::ostringstream out;
  out << std::setprecision(17);
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (i) out << ';';
    out << values(i);
  }
  return out.str();
}

void requireOpen(const std::ofstream& stream, const std::string& path) {
  if (!stream) throw std::runtime_error("cannot open run output: " + path);
}

std::string cpuModel() {
  std::ifstream input("/proc/cpuinfo");
  std::string line;
  while (std::getline(input, line)) {
    const std::string key = "model name";
    if (line.compare(0, key.size(), key) == 0) {
      const auto colon = line.find(':');
      return colon == std::string::npos ? line : line.substr(colon + 2);
    }
  }
  return "unknown";
}

}  // namespace

RunLogger::RunLogger(const std::string& output_directory,
                     bool write_residuals, bool write_timing)
    : directory_(output_directory), write_residuals_(write_residuals),
      write_timing_(write_timing) {
  boost::filesystem::create_directories(directory_);
  timing_links_.open(directory_ + "/diagnostic_timing_links.csv");
  requireOpen(timing_links_, "diagnostic_timing_links.csv");
  timing_links_ << "timing_row,input_attempt_id,transaction_id,window_id,stage\n";
  attempts_.open(directory_ + "/diagnostic_attempts.csv");
  diagnostic_stages_.open(directory_ + "/diagnostic_stages.csv");
  diagnostic_candidates_.open(directory_ + "/diagnostic_candidates.csv");
  diagnostic_coverage_.open(directory_ + "/diagnostic_coverage.csv");
  diagnostic_steps_.open(directory_ + "/diagnostic_state_steps.csv");
  diagnostic_identity_.open(directory_ + "/diagnostic_snapshot_identity.csv");
  diagnostic_square_root_.open(directory_ + "/diagnostic_square_root.csv");
  requireOpen(attempts_, "diagnostic_attempts.csv");
  requireOpen(diagnostic_stages_, "diagnostic_stages.csv");
  requireOpen(diagnostic_candidates_, "diagnostic_candidates.csv");
  requireOpen(diagnostic_coverage_, "diagnostic_coverage.csv");
  requireOpen(diagnostic_steps_, "diagnostic_state_steps.csv");
  requireOpen(diagnostic_identity_, "diagnostic_snapshot_identity.csv");
  requireOpen(diagnostic_square_root_, "diagnostic_square_root.csv");
  attempts_ << std::setprecision(17);
  diagnostic_stages_ << std::setprecision(17);
  diagnostic_square_root_ << std::setprecision(17);
  diagnostic_square_root_
      << "schema_version,attempt_id,rows,columns,rank,dof,r_diagonal_min,"
         "r_diagonal_max,condition_estimate,identity_residual_relative,"
         "parity_relative_difference,solution_relative_difference,"
         "forward_error_bound,certificate_ok,usable,detector_only_rows,"
         "statistic,scale_policy,permutation_policy,reason\n";
  diagnostic_candidates_ << std::setprecision(17);
  diagnostic_coverage_ << std::setprecision(17);
  diagnostic_steps_ << std::setprecision(17);
  const std::string identity = "schema_version,input_attempt_id,input_timestamp_ns,transaction_id,window_id,graph_version,ordering_version,noise_model_version,linpoint_version,output_timestamp_ns,";
  attempts_ << identity << "frozen_group_ids,backend_epoch_before,backend_epoch_after,pending_duration_s,state_age_s,raw_imu_samples,consecutive_rejections,marginalization_count,factor_block_cache_hits,factor_block_cache_misses,factor_block_cache_invalidations,statistical_cache_hits,statistical_cache_misses,cache_entries,cache_bytes,cache_invalidation_reason,base_step_norm,selected_step_norm,risk_nominal,risk_p_nm,risk_bridge,risk_history,risk_model,risk_hypotheses,risk_total,risk_upper_bound,risk_margin,risk_ledger_charged_total,risk_ledger_declared_total,risk_ledger_closes,risk_ledger_all_validated,risk_ledger_validated_terms,risk_ledger_unvalidated_terms,risk_ledger_not_implemented_terms,risk_ledger_terms,coverage_status,coverage_exact_leaves,coverage_enveloped_leaves,coverage_uncovered_leaves,coverage_envelope_count,coverage_accepted_envelope_count,coverage_proof_count,coverage_envelope_online,hypothesis_count,single_uwb_hypotheses,single_accel_hypotheses,single_gyro_hypotheses,double_uwb_accel_hypotheses,double_uwb_gyro_hypotheses,effective_fault_cardinality,generated_actions,kernel_evaluated_actions,post_passed_actions,pl_evaluated_actions,selected_actions,base_svd,base_llt,base_state_solves,llt_state_solve_calls,svd_state_solve_calls,detector_reference_qr,candidate_reference_svd,candidate_inner_llt,fault_gram_eigen,fault_gram_svd,fault_gram_ldlt,low_dim_fault_gram,generic_fault_gram_fallback,hypothesis_parallel_blocks,hypothesis_shared_hits,hypothesis_shared_misses,covariance_rhs_solves,covariance_rhs_columns,spectral_rhs_solves,spectral_rhs_columns,numerical_contract_mismatches,imu_oracle_reintegrations,analytic_input_valid,analytic_computation_valid,oracle_executed,oracle_relative_error,oracle_verified,primary_failure,all_failures,not_evaluated_checks,oracle_sweep_executed,oracle_sweep_verified,oracle_sweep_worst_relative_error,oracle_sweep_reintegrations,oracle_sweep_epsilons,oracle_sweep_relative_errors,status,reason\n";
  diagnostic_stages_ << identity << "stage,status,wall_ms,reason\n";
  diagnostic_candidates_ << identity << "action_id,kernel_evaluated,numerical_valid,post_passed,pl_evaluated,coverage_rejected,selected,slow_path,near_gate,recovered_replacement,numerical_path,fallback_reason,skip_reason,cache_hits,certificate_passed,condition_value_kind,condition_lower_bound,condition_upper_bound,certificate_margin,matrix_free_step_rejected,covariance_solve_count,scratch_reuse_count,kernel_ms,post_ms,bridge_ms,fault_map_ms,pl_ms\n";
  diagnostic_coverage_ << identity << "action_id,action_type,plausible_hypothesis_ids,mandatory_health_sources,mandatory_group_ids,covered_mode_ids,removed_group_ids,added_group_ids,uncovered_hypothesis_ids,uncovered_mode_ids,uncovered_group_ids,uncovered_mandatory_group_ids,outcome,reason\n";
  diagnostic_steps_ << identity << "source,epoch,rotation_norm,position_norm,velocity_norm,accel_bias_norm,gyro_bias_norm,epoch_norm\n";
  diagnostic_identity_ << identity
      << "snapshot_id,source_revision,config_digest,manifest_digest,state_solution_id,sensor_timestamp_ns,frame_id,position_reference,tangent_convention,state_scale,output_jacobian_contract,whitening_id,noise_model_id,boundary_summary_id,history_lineage_id,protected_reference_center,active_observation_index,coverage_epoch,validity_assumptions,identity_digest\n";
  states_.open(directory_ + "/states.csv");
  if (write_residuals_) residuals_.open(directory_ + "/residuals.csv");
  integrity_.open(directory_ + "/integrity.csv");
  if (write_timing_) timing_.open(directory_ + "/timing.csv");
  events_.open(directory_ + "/events.csv");
  ground_truth_.open(directory_ + "/ground_truth.csv");
  fault_truth_.open(directory_ + "/fault_truth.csv");
  transactions_.open(directory_ + "/transactions.csv");
  hypotheses_.open(directory_ + "/hypotheses.csv");
  candidates_.open(directory_ + "/candidates.csv");
  factor_ledger_.open(directory_ + "/factor_ledger.csv");
  health_.open(directory_ + "/health.csv");
  bridge_.open(directory_ + "/bridge.csv");
  requireOpen(states_, directory_ + "/states.csv");
  if (write_residuals_) requireOpen(residuals_, directory_ + "/residuals.csv");
  requireOpen(integrity_, directory_ + "/integrity.csv");
  if (write_timing_) requireOpen(timing_, directory_ + "/timing.csv");
  requireOpen(events_, directory_ + "/events.csv");
  requireOpen(ground_truth_, directory_ + "/ground_truth.csv");
  requireOpen(fault_truth_, directory_ + "/fault_truth.csv");
  requireOpen(transactions_, directory_ + "/transactions.csv");
  requireOpen(hypotheses_, directory_ + "/hypotheses.csv");
  requireOpen(candidates_, directory_ + "/candidates.csv");
  requireOpen(factor_ledger_, directory_ + "/factor_ledger.csv");
  requireOpen(health_, directory_ + "/health.csv");
  requireOpen(bridge_, directory_ + "/bridge.csv");
  states_ << "timestamp_ns,state_id,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n";
  if (write_residuals_) {
    residuals_ << "timestamp_ns,factor_id,anchor_id,row_role,raw,whitened\n";
  }
  integrity_ << "timestamp_ns,group_size,measurement_model_valid,"
                "global_statistic,global_threshold,global_dof,global_passed,"
                "postfit_statistic,postfit_threshold,postfit_dof,postfit_passed,"
                "conditional_statistic,conditional_threshold,conditional_dof,"
                "conditional_passed,conditional_formal,pl_x,pl_y,pl_z,hpl_m,"
                "vpl_m,availability,label,formal_eligible,risk_budget_valid,"
                "allocated_hmi_risk,hmi_risk_requirement,batch_committed,"
                "transaction_id,window_id,base_graph_version,linearization_version,"
                "selected_action_id,selected_action_type,fde_status,bridge_pl_x,"
                "bridge_pl_y,bridge_pl_z,history_provenance_valid,backend_updates,"
                "stale_state,controlled_reinitialization_required,"
                "historical_groups_removed,historical_groups_added,"
                "recovery_epoch_begin,recovery_epoch_end,"
                "reinitialization_request_id,reinitialization_phase,"
                "reinitialization_reason,reason\n";
  if (write_timing_) {
    timing_ << "timestamp_ns,epoch,stage,wall_ms,problem_size,hypothesis_count,"
               "factor_count,cold_warm,success\n";
  }
  events_ << "timestamp_ns,sequence,event,detail\n";
  ground_truth_ << "timestamp_ns,px,py,pz,qw,qx,qy,qz\n";
  fault_truth_ << "timestamp_ns,sequence,anchor_id,fault_mode,active,outage,"
                  "injected_bias_m,true_range_m,sensor_type,fault_kind,axis,"
                  "epoch_begin,epoch_end,injected_value,injected_units\n";
  transactions_ << "timestamp_ns,transaction_id,window_id,base_graph_version,"
                   "linearization_version,selected_action_id,fde_status,"
                   "backend_updates,stale_state,reinitialization_required\n";
  hypotheses_ << "timestamp_ns,window_id,hypothesis_id,fault_unit_ids,physical_source_ids,sensor,"
                 "fault_kind,mode_ids,onset_epoch,onset_time_ns,parameter_dimension,"
                 "fault_rank,sigma_min,sigma_max,condition_number,slope_x,slope_y,"
                 "slope_z,boundary_direction_gram,noncentrality_boundary,prior_bound,p_md_allocation,"
                 "hmi_allocation,monitorable,plausible,conditioned_statistic,"
                 "log_evidence,z_rank,z_sigma_min,z_condition,z_classification,"
                 "coverage_label,coverage_envelope_id,"
                 "reason\n";
  candidates_ << "timestamp_ns,window_id,action_id,action_type,physical_source_ids,removed_group_ids,"
                 "added_group_ids,bridge_mode,cardinality,valid,"
                 "post_detector_passed,covers_plausible_set,statistic,threshold,"
                 "rank,dof,condition_number,information_logdet,risk_allocation,"
                 "hpl_m,vpl_m,selected,evaluation_wall_ms,reason\n";
  factor_ledger_ << "factor_id,group_id,sensor,factor_kind,lifecycle,epoch_begin,"
                    "epoch_end,time_begin_ns,time_end_ns,backend_slot,"
                    "noise_model_id,model_id,health,source_ids,measurement_ids,"
                    "fault_units,commit_graph_version,removed_graph_version,"
                    "replacement_group_id,replaces_group_id,recovery_epoch\n";
  health_ << "timestamp_ns,source_id,sensor,previous_state,current_state,trigger,"
             "evidence_statistic,evidence_threshold,plausible_hypothesis_ids,"
             "selected_action_id,suspicion_count,shadow_pass_count,"
             "recovery_pass_count,bridge_count,recovery_reset_count\n";
  bridge_ << "timestamp_ns,transaction_id,mode,consecutive_epochs,duration_s,"
             "model_id,dt_s,optimization_covariance_diagonal,integrity_model,"
             "calibration_id,bound_x,bound_y,bound_z,control_available,active,"
             "timeout,status\n";
}

void RunLogger::writeResolvedConfig(const std::string& yaml) const {
  std::ofstream out(directory_ + "/resolved_config.yaml");
  requireOpen(out, directory_ + "/resolved_config.yaml");
  out << yaml;
}

void RunLogger::writeManifest(const RunManifest& m) const {
  std::ofstream out(directory_ + "/run_manifest.json");
  requireOpen(out, directory_ + "/run_manifest.json");
  out << "{\n"
      << "  \"schema_version\": " << json(m.schema_version) << ",\n"
      << "  \"created_utc\": " << json(m.created_utc) << ",\n"
      << "  \"git_sha\": " << json(m.git_sha) << ",\n"
      << "  \"git_dirty\": " << (m.git_dirty ? "true" : "false") << ",\n"
      << "  \"config_path\": " << json(m.config_path) << ",\n"
      << "  \"config_hash\": " << json(m.config_hash) << ",\n"
      << "  \"protocol_sha256\": " << json(m.protocol_sha256) << ",\n"
      << "  \"protocol_path\": " << json(m.protocol_path) << ",\n"
      << "  \"raw_inventory_path\": " << json(m.raw_inventory_path) << ",\n"
      << "  \"raw_inventory_sha256\": " << json(m.raw_inventory_sha256) << ",\n"
      << "  \"artifact_checksum_path\": " << json(m.artifact_checksum_path) << ",\n"
      << "  \"seed_domain\": " << json(m.seed_domain) << ",\n"
      << "  \"attempt\": " << m.attempt << ",\n"
      << "  \"failure_catalog_path\": " << json(m.failure_catalog_path) << ",\n"
      << "  \"seed\": " << m.seed << ",\n"
      << "  \"fixed_lag_epochs\": " << m.fixed_lag_epochs << ",\n"
      << "  \"execution_command\": " << json(m.execution_command) << ",\n"
      << "  \"build_type\": " << json(m.build_type) << ",\n"
      << "  \"compiler\": " << json(m.compiler) << ",\n"
      << "  \"os\": " << json(m.os) << ",\n"
      << "  \"cpu\": " << json(m.cpu) << ",\n"
      << "  \"ram_bytes\": " << m.ram_bytes << ",\n"
      << "  \"gtsam_version\": " << json(m.gtsam_version) << ",\n"
      << "  \"eigen_version\": " << json(m.eigen_version) << ",\n"
      << "  \"maturity\": " << json(m.maturity) << ",\n"
      << "  \"formal_eligible\": "
      << (m.formal_eligible ? "true" : "false") << ",\n"
      << "  \"diagnostics_schema_version\": "
      << json(m.diagnostics_schema_version) << ",\n"
      << "  \"failure_catalog\": " << failureReasonCatalogJson() << ",\n"
      << "  \"fault_manifest_id\": " << json(m.fault_manifest_id) << ",\n"
      << "  \"fault_manifest_digest\": " << json(m.fault_manifest_digest)
      << ",\n"
      << "  \"protected_quantity\": " << json(m.protected_quantity) << ",\n"
      << "  \"position_reference\": " << json(m.position_reference) << ",\n"
      << "  \"scope\": {\"protected_state\": " << json(m.protected_state)
      << ", \"detector\": " << json(m.detector)
      << ", \"pl_method\": " << json(m.pl_method)
      << ", \"window_epochs\": " << m.window_epochs
      << ", \"single_faults_enabled\": "
      << (m.single_faults_enabled ? "true" : "false")
      << ", \"double_faults_enabled\": "
      << (m.double_faults_enabled ? "true" : "false")
      << ", \"supported_max_fault_cardinality\": "
      << m.supported_fault_cardinality
      << ", \"max_fault_cardinality\": " << m.monitored_fault_cardinality
      << ", \"max_exclusion_cardinality\": " << m.max_exclusion_cardinality
      << ", \"bridge_model\": " << json(m.bridge_model)
      << ", \"history_recovery\": " << json(m.history_recovery) << "},\n"
      << "  \"gate_j_evidence\": {\"risk_calibration_id\": "
      << json(m.risk_calibration_id)
      << ", \"noise_overbound_calibration_id\": "
      << json(m.noise_overbound_calibration_id)
      << ", \"bridge_calibration_id\": " << json(m.bridge_calibration_id)
      << ", \"gates_a_to_i_complete\": "
      << (m.gates_a_to_i_complete ? "true" : "false")
      << ", \"independent_review_complete\": "
      << (m.independent_review_complete ? "true" : "false") << "}\n}\n";
}

void RunLogger::writeState(const NavigationState& s) {
  states_ << s.timestamp.value() << ',' << s.id.value() << ','
          << s.position_world_m.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
          << s.q_world_body.w() << ',' << s.q_world_body.vec().transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
          << s.velocity_world_mps.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
          << s.accel_bias_mps2.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << ','
          << s.gyro_bias_radps.transpose().format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",")) << '\n';
}

void RunLogger::writeResidual(TimestampNs t, FactorId factor, AnchorId anchor,
                              RowRole role, double raw, double whitened) {
  if (!write_residuals_) return;
  const char* role_name = role == RowRole::Measurement ? "Measurement" :
      (role == RowRole::TrustedPrior ? "TrustedPrior" : "Regularizer");
  residuals_ << t.value() << ',' << factor.value() << ',' << anchor.value()
             << ',' << role_name << ',' << raw << ',' << whitened << '\n';
}

void RunLogger::writeIntegrity(const IntegrityOutput& o) {
  const auto& p = o.protection_level;
  const DetectorResult& conditional = o.detector;
  integrity_ << o.timestamp.value() << ',' << o.measurement_group_size << ','
             << o.measurement_model_valid << ','
             << o.global_detector.statistic << ','
             << o.global_detector.threshold << ',' << o.global_detector.dof
             << ',' << o.global_detector.passed << ','
             << o.postfit_detector.statistic << ','
             << o.postfit_detector.threshold << ',' << o.postfit_detector.dof
             << ',' << o.postfit_detector.passed << ','
             << conditional.statistic << ',' << conditional.threshold << ','
             << conditional.dof << ',' << conditional.passed << ','
             << (conditional.detector_type ==
                 "conditional_current_uwb_innovation_chi_square" &&
                 conditional.numerically_valid) << ','
             << p.pl_xyz_m.x() << ',' << p.pl_xyz_m.y() << ',' << p.pl_xyz_m.z()
             << ',' << p.hpl_m << ',' << p.vpl_m << ','
             << toString(p.availability) << ',' << toString(p.label) << ','
             << p.formal_eligible << ',' << p.risk_budget_valid << ','
             << p.allocated_hmi_risk << ',' << p.hmi_risk_requirement << ','
             << o.batch_committed << ',' << o.transaction_id << ','
             << o.window_id << ',' << o.base_graph_version << ','
             << o.linearization_version << ',' << o.selected_action_id << ','
             << csv(o.selected_action_type) << ',' << csv(o.fde_status) << ','
             << o.bridge_component_m.x() << ',' << o.bridge_component_m.y() << ','
             << o.bridge_component_m.z() << ',' << o.history_provenance_valid << ','
             << o.backend_updates << ',' << o.stale_state << ','
             << o.controlled_reinitialization_required << ','
             << csv(joinIds(o.historical_groups_removed)) << ','
             << csv(joinIds(o.historical_groups_added)) << ','
             << o.recovery_epoch_begin << ',' << o.recovery_epoch_end << ','
             << o.reinitialization_request_id << ','
             << csv(o.reinitialization_phase) << ','
             << csv(o.reinitialization_reason) << ','
             << csv(p.reason) << '\n';
  for (const auto& s : o.square_root_audit) {
    diagnostic_square_root_
        << "uwb-imu-pl/gate-d-diagnostics/v14," << s.attempt_id << ','
        << s.rows << ',' << s.columns << ',' << s.rank << ',' << s.dof << ','
        << s.r_diagonal_min << ',' << s.r_diagonal_max << ','
        << s.condition_estimate << ',' << s.identity_residual_relative << ','
        << s.parity_relative_difference << ','
        << s.solution_relative_difference << ',' << s.forward_error_bound << ','
        << s.certificate_ok << ',' << s.usable << ',' << s.detector_only_rows
        << ',' << s.statistic << ',' << csv(s.scale_policy) << ','
        << csv(s.permutation_policy) << ',' << csv(s.reason) << '\n';
  }
  if (o.diagnostics.input_attempt_id) {
    const auto& d = o.diagnostics;
    auto identity = [&](std::ostream& stream) -> std::ostream& {
      return stream << "uwb-imu-pl/gate-d-diagnostics/v14," << d.input_attempt_id << ','
          << d.input_timestamp.value() << ',' << o.transaction_id << ',' << o.window_id << ','
          << o.base_graph_version << ',' << d.ordering_version << ','
          << d.noise_model_version << ',' << o.linearization_version << ',' << o.timestamp.value() << ',';
    };
    identity(attempts_) << csv(joinIds(d.frozen_group_ids)) << ',' << d.backend_epoch_before << ',' << d.backend_epoch_after << ','
        << d.pending_duration_s << ',' << d.state_age_s << ',' << d.raw_imu_samples << ',' << d.consecutive_rejections << ','
        << d.marginalization_count << ',' << d.factor_block_cache_hits << ','
        << d.factor_block_cache_misses << ',' << d.factor_block_cache_invalidations << ','
        << d.statistical_cache_hits << ',' << d.statistical_cache_misses << ','
        << d.cache_entries << ',' << d.cache_bytes << ','
        << csv(d.cache_invalidation_reason) << ',' << d.base_step_norm << ','
        << d.selected_step_norm << ',' << d.risk_nominal << ',' << d.risk_p_nm << ','
        << d.risk_bridge << ',' << d.risk_history << ',' << d.risk_model << ','
        << d.risk_hypotheses << ',' << d.risk_total << ',' << d.risk_upper_bound << ','
        << d.risk_margin << ',' << d.risk_ledger_charged_total << ','
        << d.risk_ledger_declared_total << ','
        << d.risk_ledger_closes << ',' << d.risk_ledger_all_validated << ','
        << d.risk_ledger_validated_terms << ','
        << d.risk_ledger_unvalidated_terms << ','
        << d.risk_ledger_not_implemented_terms << ','
        << csv(d.risk_ledger_terms) << ','
        << csv(d.coverage_status) << ',' << d.coverage_exact_leaves << ','
        << d.coverage_enveloped_leaves << ',' << d.coverage_uncovered_leaves
        << ',' << d.coverage_envelope_count << ','
        << d.coverage_accepted_envelope_count << ','
        << d.coverage_proof_count << ','
        << (d.coverage_envelope_online ? 1 : 0) << ','
        << d.hypothesis_count << ','
        << d.single_uwb_hypotheses << ',' << d.single_accel_hypotheses << ','
        << d.single_gyro_hypotheses << ',' << d.double_uwb_accel_hypotheses << ','
        << d.double_uwb_gyro_hypotheses << ',' << d.effective_fault_cardinality << ','
        << d.generated_actions << ','
        << d.kernel_evaluated_actions << ',' << d.post_passed_actions << ','
        << d.pl_evaluated_actions << ',' << d.selected_actions << ','
        << d.base_svd << ',' << d.base_llt << ',' << d.base_state_solves << ','
        << d.llt_state_solve_calls << ',' << d.svd_state_solve_calls << ','
        << d.detector_reference_qr << ','
        << d.candidate_reference_svd << ',' << d.candidate_inner_llt << ','
        << d.fault_gram_eigen << ',' << d.fault_gram_svd << ','
        << d.fault_gram_ldlt << ',' << d.low_dim_fault_gram << ','
        << d.generic_fault_gram_fallback << ','
        << d.hypothesis_parallel_blocks << ','
        << d.hypothesis_shared_hits << ',' << d.hypothesis_shared_misses << ','
        << d.covariance_rhs_solves << ',' << d.covariance_rhs_columns << ','
        << d.spectral_rhs_solves << ',' << d.spectral_rhs_columns << ','
        << d.numerical_contract_mismatches << ','
        << d.imu_oracle_reintegrations << ','
        << d.analytic_input_valid << ',' << d.analytic_computation_valid << ','
        << d.oracle_executed << ',' << d.oracle_relative_error << ','
        << d.oracle_verified << ','
        << csv(d.primary_failure) << ',' << csv(d.all_failures) << ','
        << csv(d.not_evaluated_checks) << ','
        << d.oracle_sweep_executed << ',' << d.oracle_sweep_verified << ','
        << d.oracle_sweep_worst_relative_error << ','
        << d.oracle_sweep_reintegrations << ','
        << csv(d.oracle_sweep_epsilons) << ','
        << csv(d.oracle_sweep_relative_errors) << ','
        << csv(d.status) << ','
        << csv(d.reason) << '\n';
    for (const auto& stage : o.stage_timings) {
      identity(diagnostic_stages_) << csv(stage.stage) << ',' << csv(stage.status) << ',';
      if (stage.status != "SKIPPED") diagnostic_stages_ << stage.wall_ms;
      diagnostic_stages_ << ',' << csv(stage.reason) << '\n';
    }
    for (const auto& candidate : o.candidate_audit) {
      const auto& c = candidate.diagnostics;
      identity(diagnostic_candidates_) << candidate.action_id << ',' << c.kernel_evaluated << ','
          << c.numerical_valid << ',' << candidate.post_detector_passed << ',' << c.pl_evaluated << ','
          << !candidate.covers_plausible_set << ',' << candidate.selected << ',' << c.slow_path << ','
          << c.near_gate << ',' << c.recovered_replacement << ',' << csv(c.numerical_path) << ','
          << csv(c.fallback_reason) << ',' << csv(c.skip_reason) << ',' << c.cache_hits << ','
          << c.certificate_passed << ',' << csv(c.condition_value_kind) << ','
          << c.condition_lower_bound << ',' << c.condition_upper_bound << ','
          << c.certificate_margin << ',' << c.matrix_free_step_rejected << ','
          << c.covariance_solve_count << ',' << c.scratch_reuse_count << ',';
      if (c.kernel_evaluated) diagnostic_candidates_ << c.kernel_ms;
      diagnostic_candidates_ << ',';
      if (c.kernel_evaluated) diagnostic_candidates_ << c.post_ms;
      diagnostic_candidates_ << ',';
      if (c.pl_evaluated) diagnostic_candidates_ << c.bridge_ms;
      diagnostic_candidates_ << ',';
      if (c.pl_evaluated) diagnostic_candidates_ << c.fault_map_ms;
      diagnostic_candidates_ << ',';
      if (c.pl_evaluated) diagnostic_candidates_ << c.pl_ms;
      diagnostic_candidates_ << '\n';
    }
    for (const auto& coverage : o.coverage_audit) {
      identity(diagnostic_coverage_) << coverage.action_id << ','
          << csv(coverage.action_type) << ','
          << csv(coverage.plausible_hypothesis_ids) << ','
          << csv(coverage.mandatory_health_sources) << ','
          << csv(coverage.mandatory_group_ids) << ','
          << csv(coverage.covered_mode_ids) << ','
          << csv(coverage.removed_group_ids) << ','
          << csv(coverage.added_group_ids) << ','
          << csv(coverage.uncovered_hypothesis_ids) << ','
          << csv(coverage.uncovered_mode_ids) << ','
          << csv(coverage.uncovered_group_ids) << ','
          << csv(coverage.uncovered_mandatory_group_ids) << ','
          << csv(coverage.outcome) << ',' << csv(coverage.reason) << '\n';
    }
    for (const auto& step : o.state_step_audit) {
      identity(diagnostic_steps_) << csv(step.source) << ',' << step.epoch << ','
          << step.rotation_norm << ',' << step.position_norm << ','
          << step.velocity_norm << ',' << step.accel_bias_norm << ','
          << step.gyro_bias_norm << ',' << step.epoch_norm << '\n';
    }
    const auto& frozen_identity = o.snapshot_identity;
    identity(diagnostic_identity_)
        << csv(frozen_identity.snapshot_id) << ','
        << csv(frozen_identity.source_revision) << ','
        << csv(frozen_identity.config_digest) << ','
        << csv(frozen_identity.manifest_digest) << ','
        << csv(frozen_identity.state_solution_id) << ','
        << frozen_identity.sensor_timestamp_ns << ','
        << csv(frozen_identity.frame_id) << ','
        << csv(frozen_identity.position_reference) << ','
        << csv(frozen_identity.tangent_convention) << ','
        << csv(frozen_identity.state_scale) << ','
        << csv(frozen_identity.output_jacobian_contract) << ','
        << csv(frozen_identity.whitening_id) << ','
        << csv(frozen_identity.noise_model_id) << ','
        << csv(frozen_identity.boundary_summary_id) << ','
        << csv(frozen_identity.history_lineage_id) << ','
        << csv(frozen_identity.protected_reference_center) << ','
        << csv(frozen_identity.active_observation_index) << ','
        << csv(frozen_identity.coverage_epoch) << ','
        << csv(frozen_identity.validity_assumptions) << ','
        << csv(frozen_identity.identity_digest) << '\n';
  }
  transactions_ << o.timestamp.value() << ',' << o.transaction_id << ','
                << o.window_id << ',' << o.base_graph_version << ','
                << o.linearization_version << ',' << o.selected_action_id << ','
                << csv(o.fde_status) << ',' << o.backend_updates << ','
                << o.stale_state << ','
                << o.controlled_reinitialization_required << '\n';
  for (const auto& h : o.hypothesis_audit) {
    hypotheses_ << o.timestamp.value() << ',' << o.window_id << ','
                << h.hypothesis_id << ',' << csv(h.fault_unit_ids) << ','
                << csv(h.physical_source_ids) << ','
                << csv(h.sensor) << ',' << csv(h.fault_kind) << ','
                << csv(h.mode_ids) << ',' << h.onset_epoch << ','
                << h.onset_time_ns << ',' << h.parameter_dimension << ','
                << h.fault_rank << ',' << h.sigma_min << ',' << h.sigma_max
                << ',' << h.condition_number << ',' << h.slope_xyz.x() << ','
                << h.slope_xyz.y() << ',' << h.slope_xyz.z() << ','
                << h.boundary_direction_gram << ',' << h.noncentrality_boundary << ','
                << h.prior_bound << ',' << h.p_md_allocation << ','
                << h.hmi_allocation << ',' << h.monitorable << ','
                << h.plausible << ',' << h.conditioned_statistic << ','
                << h.log_evidence << ',' << h.z_rank << ','
                << h.z_smallest_singular_value << ',' << h.z_condition << ','
                << h.z_classification << ',' << csv(h.coverage_label) << ','
                << h.coverage_envelope_id << ',' << csv(h.reason) << '\n';
  }
  for (const auto& c : o.candidate_audit) {
    candidates_ << o.timestamp.value() << ',' << o.window_id << ','
                << c.action_id << ',' << csv(c.action_type) << ','
                << csv(c.physical_source_ids) << ','
                << csv(c.removed_group_ids) << ',' << csv(c.added_group_ids)
                << ',' << csv(c.bridge_mode) << ',' << c.cardinality << ','
                << c.valid << ','
                << c.post_detector_passed << ',' << c.covers_plausible_set << ','
                << c.statistic << ',' << c.threshold << ',' << c.rank << ','
                << c.dof << ',';
    if (c.diagnostics.condition_value_kind == "EXACT_SVD" ||
        c.diagnostics.condition_value_kind == "EXACT_BASE") {
      candidates_ << c.condition_number;
    }
    candidates_ << ',' << c.information_logdet << ',' << c.risk_allocation << ','
                << c.hpl_m << ',' << c.vpl_m << ',' << c.selected << ','
                << c.wall_ms << ','
                << csv(c.reason) << '\n';
  }
  for (const auto& f : o.factor_ledger_audit) {
    factor_ledger_ << f.factor_id << ',' << f.group_id << ',' << csv(f.sensor)
                   << ',' << csv(f.factor_kind) << ',' << csv(f.lifecycle) << ','
                   << f.epoch_begin << ',' << f.epoch_end << ','
                   << f.time_begin.value() << ',' << f.time_end.value() << ','
                   << csv(f.backend_slot) << ',' << csv(f.noise_model_id) << ','
                   << csv(f.model_id) << ',' << csv(f.health) << ','
                   << csv(f.source_ids) << ',' << csv(f.measurement_ids) << ','
                   << csv(f.fault_units) << ',' << f.commit_graph_version << ','
                   << f.removed_graph_version << ',' << f.replacement_group_id
                   << ',' << f.replaces_group_id << ',' << f.recovery_epoch << '\n';
  }
  for (const auto& h : o.health_audit) {
    health_ << o.timestamp.value() << ',' << csv(h.source_id) << ','
            << csv(h.sensor) << ',' << csv(h.previous_state) << ','
            << csv(h.current_state) << ',' << csv(h.trigger) << ','
            << h.evidence_statistic << ',' << h.evidence_threshold << ','
            << csv(h.plausible_hypothesis_ids) << ',' << h.selected_action_id << ','
            << h.suspicion_count << ',' << h.shadow_pass_count << ','
            << h.recovery_pass_count << ',' << h.bridge_count << ','
            << h.recovery_reset_count << '\n';
  }
  if (o.bridge_audit) {
    const auto& b = *o.bridge_audit;
    bridge_ << o.timestamp.value() << ',' << o.transaction_id << ','
            << csv(b.mode) << ',' << b.consecutive_epochs << ','
            << b.duration_s << ',' << csv(b.model_id) << ',' << b.dt_s << ','
            << csv(joinVector(b.optimization_covariance_diagonal)) << ','
            << csv(b.integrity_model) << ','
            << csv(b.calibration_id) << ',' << b.bound.x() << ','
            << b.bound.y() << ',' << b.bound.z() << ',' << b.control_available
            << ',' << b.active << ',' << b.timeout << ',' << csv(b.status)
            << '\n';
  }
}

void RunLogger::writeTiming(TimestampNs t, const std::string& stage,
                            double wall_ms, bool success) {
  TimingRecord record;
  record.timestamp = t;
  record.stage = stage;
  record.wall_ms = wall_ms;
  record.success = success;
  writeTiming(record);
}

void RunLogger::writeTiming(const TimingRecord& record) {
  if (!write_timing_) return;
  ++timing_row_;
  if (record.input_attempt_id) timing_links_ << timing_row_ << ',' << record.input_attempt_id << ','
      << record.transaction_id << ',' << record.window_id << ',' << csv(record.stage) << '\n';
  timing_ << record.timestamp.value() << ',' << record.epoch << ','
          << csv(record.stage) << ',' << record.wall_ms << ','
          << record.problem_size << ',' << record.hypothesis_count << ','
          << record.factor_count << ',' << (record.cold ? "cold" : "warm")
          << ',' << record.success << '\n';
}

void RunLogger::writeEvent(TimestampNs t, const std::string& event,
                           const std::string& detail) {
  writeEvent(t, next_event_sequence_++, event, detail);
}

void RunLogger::writeEvent(TimestampNs t, std::uint64_t sequence,
                           const std::string& event,
                           const std::string& detail) {
  std::lock_guard<std::mutex> lock(mutex_);
  next_event_sequence_ = std::max(next_event_sequence_, sequence + 1);
  events_ << t.value() << ',' << sequence << ',' << csv(event) << ','
          << csv(detail) << '\n';
}

void RunLogger::writeGroundTruth(const GroundTruthRecord& record) {
  std::lock_guard<std::mutex> lock(mutex_);
  ground_truth_ << record.timestamp.value() << ','
                << record.position_world_m.transpose().format(
                       Eigen::IOFormat(Eigen::FullPrecision,
                                       Eigen::DontAlignCols, ","))
                << ',' << record.q_world_body.w() << ','
                << record.q_world_body.vec().transpose().format(
                       Eigen::IOFormat(Eigen::FullPrecision,
                                       Eigen::DontAlignCols, ","))
                << '\n';
}

void RunLogger::writeFaultTruth(const FaultTruthRecord& record) {
  std::lock_guard<std::mutex> lock(mutex_);
  fault_truth_ << record.timestamp.value() << ',' << record.sequence << ','
               << record.anchor_id.value() << ',' << csv(record.fault_mode)
               << ',' << record.active << ',' << record.outage << ','
               << record.injected_bias_m << ',' << record.true_range_m << ','
               << csv(record.sensor_type) << ',' << csv(record.fault_kind) << ','
               << record.axis << ',' << record.epoch_begin << ','
               << record.epoch_end << ',' << record.injected_value << ','
               << csv(record.injected_units) << '\n';
}

void RunLogger::writeSummary(const std::string& status,
                             const std::string& detail) const {
  RunSummary summary;
  summary.status = status;
  summary.detail = detail;
  writeSummary(summary);
}

void RunLogger::writeSummary(const RunSummary& summary) const {
  std::ofstream out(directory_ + "/summary.json");
  requireOpen(out, directory_ + "/summary.json");
  out << "{\n"
      << "  \"status\": " << json(summary.status) << ",\n"
      << "  \"processed\": " << summary.processed << ",\n"
      << "  \"committed\": " << summary.committed << ",\n"
      << "  \"rejected\": " << summary.rejected << ",\n"
      << "  \"errors\": " << summary.errors << ",\n"
      << "  \"metrics\": {\"core_total_ms\": " << summary.core_total_ms
      << ", \"end_to_end_total_ms\": " << summary.end_to_end_total_ms
      << "},\n"
      << "  \"detail\": " << json(summary.detail) << "\n}\n";
}

void RunLogger::flush() {
  std::lock_guard<std::mutex> lock(mutex_);
  states_.flush(); residuals_.flush(); integrity_.flush(); timing_.flush();
  events_.flush(); ground_truth_.flush(); fault_truth_.flush();
  transactions_.flush(); hypotheses_.flush(); candidates_.flush();
  factor_ledger_.flush(); health_.flush(); bridge_.flush(); attempts_.flush();
  timing_links_.flush(); diagnostic_stages_.flush(); diagnostic_candidates_.flush();
  diagnostic_coverage_.flush();
  diagnostic_steps_.flush();
  diagnostic_identity_.flush();
  diagnostic_square_root_.flush();
}

RunManifest makeRunManifest(const IntegrityConfig& config,
                            const std::string& git_sha, bool git_dirty,
                            const std::string& execution_command) {
  RunManifest manifest;
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::ostringstream utc;
  utc << std::put_time(std::gmtime(&time), "%Y-%m-%dT%H:%M:%SZ");
  manifest.created_utc = utc.str();
  manifest.git_sha = git_sha;
  manifest.git_dirty = git_dirty;
  manifest.config_path = config.source_path;
  manifest.config_hash = config.config_hash;
  // Standalone runs have no campaign protocol. Campaign tooling replaces
  // these sentinel values with the frozen SHA-256/inventory metadata before
  // an artifact can be finalized as formal evidence.
  manifest.protocol_path = "UNAVAILABLE";
  manifest.protocol_sha256 = std::string(64, '0');
  manifest.raw_inventory_path = "UNAVAILABLE";
  manifest.raw_inventory_sha256 = std::string(64, '0');
  manifest.artifact_checksum_path = "UNAVAILABLE";
  manifest.failure_catalog_path = "UNAVAILABLE";
  manifest.resolved_config = config.resolved_yaml;
  manifest.seed = config.seed;
  manifest.fixed_lag_epochs = config.incremental.fixed_lag_epochs;
  manifest.schema_version = config.schema_version;
  manifest.window_epochs = config.integrity_window.epochs;
  manifest.single_faults_enabled = config.fault_models.single_faults_enabled;
  manifest.double_faults_enabled = config.fault_models.double_faults_enabled;
  manifest.supported_fault_cardinality = config.fault_models.max_cardinality;
  manifest.monitored_fault_cardinality =
      enabledFaultHypothesisCardinality(config.fault_models);
  manifest.max_exclusion_cardinality = config.fde.max_exclusion_cardinality;
  manifest.bridge_calibration_id = config.bridge.generic.calibration_id;
  manifest.risk_calibration_id = config.risk_v2.calibration_id;
  manifest.noise_overbound_calibration_id =
      config.imu.noise_overbound_calibration_id;
  manifest.protected_quantity = "position_xyz";
  manifest.position_reference = "body_origin";
  manifest.diagnostics_schema_version = "uwb-imu-pl/gate-d-diagnostics/v14";
  manifest.failure_catalog = failureReasonCatalogJson();
  if (config.fault_manifest) {
    manifest.fault_manifest_id = config.fault_manifest->manifest_id;
    manifest.fault_manifest_digest = config.fault_manifest->digest;
    manifest.protected_quantity = config.fault_manifest->protected_quantity;
    manifest.position_reference = config.fault_manifest->position_reference;
  } else {
    manifest.fault_manifest_id = kNotAvailableInSchema;
    manifest.fault_manifest_digest = kNotAvailableInSchema;
  }
  manifest.execution_command = execution_command;
#ifdef NDEBUG
  manifest.build_type = "Release";
#else
  manifest.build_type = "Debug";
#endif
  manifest.compiler = __VERSION__;
  manifest.eigen_version = std::to_string(EIGEN_WORLD_VERSION) + "." +
      std::to_string(EIGEN_MAJOR_VERSION) + "." + std::to_string(EIGEN_MINOR_VERSION);
  struct utsname system_name {};
  manifest.os = uname(&system_name) == 0 ?
      std::string(system_name.sysname) + " " + system_name.release : "unknown";
  manifest.cpu = cpuModel();
  struct sysinfo memory_info {};
  if (sysinfo(&memory_info) == 0) {
    manifest.ram_bytes = static_cast<std::uint64_t>(memory_info.totalram) *
        static_cast<std::uint64_t>(memory_info.mem_unit);
  }
#ifdef GTSAM_VERSION_STRING
  manifest.gtsam_version = GTSAM_VERSION_STRING;
#else
  manifest.gtsam_version = "4.2.x-required";
#endif
  return manifest;
}

}  // namespace uwb_imu_pl
