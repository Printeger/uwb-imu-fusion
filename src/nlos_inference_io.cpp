#include "uifgo/nlos_inference_io.h"

#include <boost/filesystem.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace uifgo {
namespace {

namespace fs = boost::filesystem;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

std::string Csv(const std::string& value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
  std::string output = "\"";
  for (char c : value) output += c == '"' ? "\"\"" : std::string(1, c);
  output += '"';
  return output;
}

std::string Json(const std::string& value) {
  std::ostringstream output;
  for (unsigned char c : value) {
    switch (c) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (c < 0x20) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned>(c) << std::dec;
        } else {
          output << static_cast<char>(c);
        }
    }
  }
  return output.str();
}

std::string NumberOrNull(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream output;
  output << std::setprecision(17) << value;
  return output.str();
}

std::ofstream OpenNew(const fs::path& path,
                      InferenceArtifactWriteResult* written) {
  if (fs::exists(path))
    throw std::runtime_error("refusing to overwrite inference artifact: " +
                             path.string());
  std::ofstream output(path.string(), std::ios::binary);
  if (!output) throw std::runtime_error("cannot create " + path.string());
  written->files.push_back(path.filename().string());
  return output;
}

std::string Ordinals(const std::vector<size_t>& values) {
  std::ostringstream output;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i) output << ';';
    output << values[i];
  }
  return output.str();
}

double MaxGamma(const GroupRecoverabilityScore& score, bool* available) {
  *available = !score.segment_fit.empty();
  double maximum = -std::numeric_limits<double>::infinity();
  for (const auto& fit : score.segment_fit) {
    *available = *available && std::isfinite(fit.gamma);
    if (std::isfinite(fit.gamma)) maximum = std::max(maximum, fit.gamma);
  }
  return maximum;
}

void WriteScoreHeader(std::ofstream* output) {
  *output << "inference_id,phase,group_id,segment_ordinals,status,eligible,"
             "evaluated,valid_score_exported,eta,s_m,s_is_infinite,max_gamma,"
             "numerical_status,linearization_id\n";
}

void WriteScoreRow(std::ofstream* output, const std::string& inference_id,
                   const std::string& phase,
                   const SegmentOverlapGroup& group, const std::string& status,
                   bool eligible, bool evaluated, bool valid_score_exported,
                   const RecoverabilityResult* numerical,
                   const std::vector<SegmentFitScore>* fits,
                   const std::string& linearization_id) {
  bool gamma_available = false;
  double gamma = 0.0;
  if (fits) {
    GroupRecoverabilityScore temporary;
    temporary.segment_fit = *fits;
    gamma = MaxGamma(temporary, &gamma_available);
  }
  *output << Csv(inference_id) << ',' << Csv(phase) << ','
          << Csv(group.group_id) << ',' << Csv(Ordinals(group.segment_ordinals))
          << ',' << Csv(status) << ',' << eligible << ',' << evaluated << ','
          << valid_score_exported << ',';
  if (numerical && valid_score_exported)
    *output << NumberOrNull(numerical->eta);
  *output << ',';
  if (numerical && valid_score_exported && !numerical->s_is_infinite)
    *output << NumberOrNull(numerical->s_m);
  *output << ',';
  if (numerical && valid_score_exported)
    *output << numerical->s_is_infinite;
  *output << ',';
  if (gamma_available) *output << NumberOrNull(gamma);
  *output << ','
          << Csv(numerical ? RecoverabilityStatusName(numerical->status)
                           : "NOT_EVALUATED")
          << ',' << Csv(linearization_id) << '\n';
}

void WriteSegmentScoreHeader(std::ofstream* output) {
  *output << "inference_id,phase,group_id,segment_ordinal,segment_id,"
             "evaluated,gamma,gamma_available,short_support_debug,boundary,"
             "linearization_id,status\n";
}

void WriteSegmentScoreRows(std::ofstream* output,
                           const std::string& inference_id,
                           const std::string& phase,
                           const SegmentOverlapGroup& group,
                           const std::vector<SegmentFitScore>* fits,
                           bool evaluated,
                           const std::string& linearization_id,
                           const std::string& status) {
  if (fits && !fits->empty()) {
    for (const auto& fit : *fits) {
      *output << Csv(inference_id) << ',' << Csv(phase) << ','
              << Csv(group.group_id) << ',' << fit.segment_ordinal << ','
              << Csv(fit.segment_id) << ',' << evaluated << ',';
      if (std::isfinite(fit.gamma)) *output << NumberOrNull(fit.gamma);
      *output << ',' << std::isfinite(fit.gamma) << ','
              << fit.short_support_debug << ',' << fit.boundary << ','
              << Csv(linearization_id) << ',' << Csv(status) << '\n';
    }
    return;
  }
  for (size_t ordinal : group.segment_ordinals)
    *output << Csv(inference_id) << ',' << Csv(phase) << ','
            << Csv(group.group_id) << ',' << ordinal << ",," << evaluated
            << ",,0,,," << Csv(linearization_id) << ',' << Csv(status)
            << '\n';
}

void WriteAttemptTrace(const fs::path& root, const std::string& filename,
                       const std::string& phase,
                       const RefitAttemptDiagnostics& attempt,
                       InferenceArtifactWriteResult* written) {
  auto output = OpenNew(root / filename, written);
  output << "phase,execution_status,solver_status,stop_reason,"
            "acceptance_audit_status,acceptance_failure_reason,"
            "outer_iteration,conditional_lm_iterations,"
            "conditional_lm_inner_iterations,conditional_lm_lambda,"
            "objective_before,objective_after,relative_objective_change,"
            "scaled_state_step,max_kkt_violation,"
            "max_scaled_navigation_gradient_objective,"
            "navigation_gradient_roundoff_allowance_objective,"
            "navigation_stationarity_tolerance_objective,"
            "allowed_objective_increase,objective_ok,step_ok,kkt_ok,"
            "navigation_stationarity_ok\n"
         << std::setprecision(17);
  auto prefix = [&]() {
    output << Csv(phase) << ',' << Csv(attempt.execution_status) << ','
           << Csv(attempt.solver_status) << ',' << Csv(attempt.stop_reason)
           << ',' << Csv(attempt.acceptance_audit_status) << ','
           << Csv(attempt.acceptance_failure_reason) << ',';
  };
  if (attempt.iterations.empty()) {
    prefix();
    output << ",,,,,,,,,,,,,,,,\n";
    return;
  }
  for (const auto& trace : attempt.iterations) {
    prefix();
    output << trace.outer_iteration << ',' << trace.conditional_lm_iterations
           << ',' << trace.conditional_lm_inner_iterations << ','
           << trace.conditional_lm_lambda << ',' << trace.objective_before
           << ',' << trace.objective_after << ','
           << trace.relative_objective_change << ',' << trace.scaled_state_step
           << ',' << trace.max_kkt_violation << ','
           << trace.max_scaled_navigation_gradient_objective << ','
           << trace.navigation_gradient_roundoff_allowance_objective << ','
           << trace.navigation_stationarity_tolerance_objective << ','
           << trace.allowed_objective_increase << ',' << trace.objective_ok
           << ',' << trace.step_ok << ',' << trace.kkt_ok << ','
           << trace.navigation_stationarity_ok << '\n';
  }
}

}  // namespace

InferenceArtifactWriteResult WriteInferenceArtifacts(
    const std::string& output_directory, const InferenceResult& result) {
  const fs::path root(output_directory);
  if (!fs::is_directory(root))
    throw std::invalid_argument("inference output directory does not exist");
  if (result.inference_id.empty())
    throw std::invalid_argument("InferenceResult has no inference_id");
  std::string identity_reason;
  if (!VerifyInferenceContentIdentity(result, &identity_reason))
    throw std::invalid_argument("InferenceResult identity is not content-bound: " +
                                identity_reason);
  InferenceArtifactWriteResult written;

  {
    auto output = OpenNew(root / "decisions.csv", &written);
    output << "inference_id,group_id,start_time,end_time,segment_ordinals,"
              "decision,reason_code,eligible,eta_pass,s_pass,gamma_pass,"
              "max_gamma,max_gamma_available,decision_linearization_id\n"
           << std::setprecision(17);
    for (const auto& decision : result.decisions) {
      output << Csv(result.inference_id) << ','
             << Csv(decision.group.group_id) << ','
             << decision.group.start_time << ',' << decision.group.end_time
             << ',' << Csv(Ordinals(decision.group.segment_ordinals)) << ','
             << GroupDecisionName(decision.decision) << ','
             << Csv(decision.reason_code) << ',' << decision.eligible << ','
             << decision.eta_pass << ',' << decision.s_pass << ','
             << decision.gamma_pass << ',';
      if (decision.max_gamma_available) output << decision.max_gamma;
      output << ',' << decision.max_gamma_available << ','
             << Csv(decision.decision_linearization_id) << '\n';
    }
  }
  {
    auto output = OpenNew(root / "scores_decision.csv", &written);
    WriteScoreHeader(&output);
    for (const auto& score : result.decision_scores)
      WriteScoreRow(&output, result.inference_id, "DECISION",
                    score.group, score.status, score.eligible, true,
                    score.valid_score_exported, &score.numerical,
                    &score.segment_fit, score.linearization_id);
  }
  {
    auto output = OpenNew(root / "scores_decision_segments.csv", &written);
    WriteSegmentScoreHeader(&output);
    for (const auto& score : result.decision_scores)
      WriteSegmentScoreRows(&output, result.inference_id, "DECISION",
                            score.group, &score.segment_fit, true,
                            score.linearization_id, score.status);
  }
  {
    auto output = OpenNew(root / "scores_recovery_attempt.csv", &written);
    WriteScoreHeader(&output);
    for (const auto& score : result.recovery_final_scores) {
      const auto* numerical = score.evaluated ? &score.score.numerical : nullptr;
      const auto* fits = score.evaluated ? &score.score.segment_fit : nullptr;
      WriteScoreRow(&output, result.inference_id, "RECOVERY_ATTEMPT",
                    score.group, score.status,
                    score.evaluated && score.score.eligible, score.evaluated,
                    score.evaluated && score.score.valid_score_exported,
                    numerical, fits, score.linearization_id);
    }
  }
  {
    auto output =
        OpenNew(root / "scores_recovery_attempt_segments.csv", &written);
    WriteSegmentScoreHeader(&output);
    for (const auto& score : result.recovery_final_scores)
      WriteSegmentScoreRows(
          &output, result.inference_id, "RECOVERY_ATTEMPT", score.group,
          score.evaluated ? &score.score.segment_fit : nullptr,
          score.evaluated, score.linearization_id, score.status);
  }
  {
    auto output = OpenNew(root / "scores_final.csv", &written);
    WriteScoreHeader(&output);
    for (const auto& final_score : result.final_scores) {
      const auto* numerical = final_score.evaluated
                                  ? &final_score.score.numerical
                                  : nullptr;
      const auto* fits = final_score.evaluated
                             ? &final_score.score.segment_fit
                             : nullptr;
      WriteScoreRow(&output, result.inference_id, "FINAL",
                    final_score.group, final_score.status,
                    final_score.evaluated && final_score.score.eligible,
                    final_score.evaluated,
                    final_score.evaluated &&
                        final_score.score.valid_score_exported,
                    numerical, fits, final_score.linearization_id);
    }
  }
  {
    auto output = OpenNew(root / "scores_final_segments.csv", &written);
    WriteSegmentScoreHeader(&output);
    for (const auto& score : result.final_scores)
      WriteSegmentScoreRows(&output, result.inference_id, "FINAL_RESULT",
                            score.group,
                            score.evaluated ? &score.score.segment_fit : nullptr,
                            score.evaluated, score.linearization_id,
                            score.status);
  }
  WriteAttemptTrace(root, "recovery_refit_iterations.csv",
                    "RECOVERY_ATTEMPT", result.recovery_attempt, &written);
  WriteAttemptTrace(root, "fallback_refit_iterations.csv",
                    "FALLBACK_ATTEMPT", result.fallback_refit_attempt,
                    &written);
  WriteAttemptTrace(root, "final_refit_iterations.csv", "FINAL_RESULT",
                    result.final_result_refit, &written);
  {
    auto output = OpenNew(root / "final_masks.csv", &written);
    output << "inference_id,obs_id,candidate,noncandidate_reference,"
              "decision_use,final_use,fallback_use,segment_id,group_id,"
              "reason_code\n";
    for (const auto& mask : result.frozen_masks)
      output << Csv(result.inference_id) << ',' << mask.obs_id << ','
             << mask.candidate << ',' << mask.noncandidate_reference << ','
             << mask.decision_use << ',' << mask.final_use << ','
             << mask.fallback_use << ',' << Csv(mask.segment_id) << ','
             << Csv(mask.group_id) << ',' << Csv(mask.reason_code) << '\n';
  }
  {
    auto output = OpenNew(root / "final_factor_audit.csv", &written);
    output << "inference_id,obs_id,classification,final_factor_count,"
              "expected_count,ok\n";
    for (const auto& row : result.factor_audit.observations)
      output << Csv(result.inference_id) << ',' << row.obs_id << ','
             << Csv(row.classification) << ',' << row.final_factor_count << ','
             << Csv(row.expected_count) << ',' << row.ok << '\n';
  }
  {
    auto output = OpenNew(root / "fallback_attempt.json", &written);
    output << "{\n  \"schema\": \"t08_fallback_attempt_v1\",\n"
           << "  \"inference_id\": \"" << Json(result.inference_id)
           << "\",\n  \"attempted\": "
           << (result.fallback.attempted ? "true" : "false")
           << ",\n  \"attempt_count\": " << result.fallback.attempt_count
           << ",\n  \"trigger_reason\": \""
           << Json(result.fallback.trigger_reason)
           << "\",\n  \"recovery_failure_reason\": \""
           << Json(result.fallback.recovery_failure_reason)
           << "\",\n  \"status\": \"" << Json(result.fallback.status)
           << "\",\n  \"fallback_solver_status\": \""
           << Json(result.fallback.fallback_solver_status)
           << "\",\n  \"fallback_failure_reason\": \""
           << Json(result.fallback.fallback_failure_reason) << "\"\n}\n";
  }
  {
    auto output = OpenNew(root / "covariance_status.json", &written);
    output << "{\n  \"schema\": \"t08_full_final_graph_covariance_v1\",\n"
           << "  \"inference_id\": \"" << Json(result.inference_id)
           << "\",\n  \"status\": \""
           << CovarianceStatusName(result.final_graph_covariance.status)
           << "\",\n  \"reason\": \""
           << Json(result.final_graph_covariance.reason)
           << "\",\n  \"source\": \""
           << Json(result.final_graph_covariance.source)
           << "\",\n  \"factorization\": \""
           << Json(result.final_graph_covariance.factorization)
           << "\",\n  \"reference_group_R_c_is_covariance\": false,\n"
           << "  \"block_count\": "
           << result.final_graph_covariance.blocks.size() << "\n}\n";
  }
  if (result.final_graph_covariance.status == CovarianceStatus::AVAILABLE) {
    auto output = OpenNew(root / "covariance.csv", &written);
    output << "inference_id,key,role,coordinates,row,column,covariance\n"
           << std::setprecision(17);
    for (const auto& block : result.final_graph_covariance.blocks) {
      for (Eigen::Index row = 0; row < block.covariance.rows(); ++row) {
        for (Eigen::Index column = 0; column < block.covariance.cols();
             ++column) {
          output << Csv(result.inference_id) << ','
                 << Csv(gtsam::DefaultKeyFormatter(block.key)) << ','
                 << Csv(block.role) << ',' << Csv(block.coordinates) << ','
                 << row << ',' << column << ','
                 << block.covariance(row, column) << '\n';
        }
      }
    }
  }
  {
    auto output = OpenNew(root / "final_content_identity.json", &written);
    output << "{\n  \"schema\": \""
           << Json(result.content_identity.schema)
           << "\",\n  \"inference_id\": \"" << Json(result.inference_id)
           << "\",\n  \"graph_content_semantics\": "
              "\"factor-order/type/keys/error/whitened-final-linearization-"
              "augmented-Jacobian; IEEE-754 bits\",\n"
           << "  \"values_content_semantics\": "
              "\"key/type/canonical state coordinates; IEEE-754 bits\",\n"
           << "  \"graph_linearization_sha256\": \""
           << Json(result.content_identity.graph_linearization_sha256)
           << "\",\n  \"values_sha256\": \""
           << Json(result.content_identity.values_sha256)
           << "\",\n  \"context_sha256\": \""
           << Json(result.content_identity.context_sha256)
           << "\",\n  \"input_sha256\": \""
           << Json(result.identity_context.input_sha256)
           << "\",\n  \"config_sha256\": \""
           << Json(result.identity_context.config_sha256)
           << "\",\n  \"input_plan_sha256\": \""
           << Json(result.identity_context.input_plan_sha256)
           << "\",\n  \"support_partition_sha256\": \""
           << Json(result.identity_context.support_partition_sha256)
           << "\",\n  \"calibration_sha256\": \""
           << Json(result.identity_context.calibration_sha256)
           << "\",\n  \"solver_config_sha256\": \""
           << Json(result.identity_context.solver_config_sha256)
           << "\",\n  \"validation_context_sha256\": \""
           << Json(result.identity_context.validation_context_sha256)
           << "\",\n  \"run_id_semantics\": "
              "\"external execution namespace; not part of content ID\",\n"
           << "  \"linearization_id_semantics\": "
              "\"per-group decision/recovery score identity; distinct from "
              "final inference content ID\"\n}\n";
  }
  {
    auto output = OpenNew(root / "final_inference_summary.json", &written);
    output << "{\n  \"schema\": \"t08_inference_summary_v2\",\n"
           << "  \"inference_id\": \"" << Json(result.inference_id)
           << "\",\n  \"status\": \"" << InferenceStatusName(result.status)
           << "\",\n  \"reason\": \"" << Json(result.reason)
           << "\",\n  \"valid_estimate\": "
           << (result.valid_estimate() ? "true" : "false")
           << ",\n  \"final_graph_factor_count\": "
           << result.final_graph.size()
           << ",\n  \"final_values_count\": " << result.final_values.size()
           << ",\n  \"candidate_group_count\": " << result.decisions.size()
           << ",\n  \"gate_parameter_provenance\": \""
           << Json(result.gate_thresholds.parameter_provenance)
           << "\",\n  \"tau_eta\": "
           << NumberOrNull(result.gate_thresholds.tau_eta)
           << ",\n  \"tau_s_m\": "
           << NumberOrNull(result.gate_thresholds.tau_s_m)
           << ",\n  \"tau_gamma\": "
           << NumberOrNull(result.gate_thresholds.tau_gamma)
           << ",\n  \"fallback_attempt_count\": "
           << result.fallback.attempt_count
           << ",\n  \"recovery_attempt_execution_status\": \""
           << Json(result.recovery_attempt.execution_status)
           << "\",\n  \"recovery_attempt_solver_status\": \""
           << Json(result.recovery_attempt.solver_status)
           << "\",\n  \"recovery_attempt_acceptance_audit_status\": \""
           << Json(result.recovery_attempt.acceptance_audit_status)
           << "\",\n  \"recovery_attempt_failure_reason\": \""
           << Json(result.recovery_attempt.acceptance_failure_reason)
           << "\",\n  \"fallback_refit_execution_status\": \""
           << Json(result.fallback_refit_attempt.execution_status)
           << "\",\n  \"final_result_refit_source\": \""
           << Json(result.final_result_refit.execution_status)
           << "\",\n  \"final_optimizer_calls\": "
           << (result.final_result_refit.solver_status == "SUCCESS_EMPTY" ? "0" : "null")
           << ",\n  \"recovery_refit_iteration_count\": "
           << result.recovery_attempt.iterations.size()
           << ",\n  \"fallback_refit_iteration_count\": "
           << result.fallback_refit_attempt.iterations.size()
           << ",\n  \"final_refit_iteration_count\": "
           << result.final_result_refit.iterations.size()
           << ",\n  \"factor_audit_status\": \""
           << (result.factor_audit.ok ? "OK" : "FAILED")
           << "\",\n  \"factor_audit_reason\": \""
           << Json(result.factor_audit.reason)
           << "\",\n  \"corrected_pseudo_range_count\": "
           << result.factor_audit.corrected_pseudo_range_count
           << ",\n  \"covariance_status\": \""
           << CovarianceStatusName(result.final_graph_covariance.status)
           << "\",\n  \"timing_seconds\": {\n"
           << "    \"decision\": "
           << NumberOrNull(result.timing.decision_seconds)
           << ",\n    \"recovery_refit\": "
           << NumberOrNull(result.timing.recovery_refit_seconds)
           << ",\n    \"final_score\": "
           << NumberOrNull(result.timing.final_score_seconds)
           << ",\n    \"fallback\": "
           << NumberOrNull(result.timing.fallback_seconds)
           << ",\n    \"covariance\": "
           << NumberOrNull(result.timing.covariance_seconds)
           << ",\n    \"total\": "
           << NumberOrNull(result.timing.total_seconds) << "\n  }\n}\n";
  }

  if (!result.valid_estimate()) return written;

  {
    auto output = OpenNew(root / "trajectory.tum", &written);
    output << std::setprecision(17);
    for (size_t k = 0; k < result.final_keyframe_times_s.size(); ++k) {
      const auto pose = result.final_values.at<gtsam::Pose3>(X(k));
      const auto q = pose.rotation().toQuaternion();
      output << result.final_keyframe_times_s[k] << ' ' << pose.x() << ' '
             << pose.y() << ' ' << pose.z() << ' ' << q.x() << ' ' << q.y()
             << ' ' << q.z() << ' ' << q.w() << '\n';
    }
  }
  {
    auto output = OpenNew(root / "imu_bias.csv", &written);
    output << "inference_id,keyframe_id,sensor_time,bax,bay,baz,bgx,bgy,bgz\n"
           << std::setprecision(17);
    for (size_t k = 0; k < result.final_keyframe_times_s.size(); ++k) {
      const auto bias =
          result.final_values.at<gtsam::imuBias::ConstantBias>(B(k));
      const auto ba = bias.accelerometer();
      const auto bg = bias.gyroscope();
      output << Csv(result.inference_id) << ',' << k << ','
             << result.final_keyframe_times_s[k] << ',' << ba.x() << ','
             << ba.y() << ',' << ba.z() << ',' << bg.x() << ',' << bg.y()
             << ',' << bg.z() << '\n';
    }
  }
  {
    auto output = OpenNew(root / "static_bias.csv", &written);
    output << "inference_id,link,key,value_m,source\n"
           << std::setprecision(17);
    for (const auto& item : result.fixed_static_biases_m)
      output << Csv(result.inference_id) << ',' << Csv(item.first)
             << ",," << item.second << ",FIXED_CALIBRATION_CONSTANT\n";
    for (gtsam::Key key : result.final_values.keys()) {
      if (gtsam::Symbol(key).chr() != 'z') continue;
      output << Csv(result.inference_id) << ",,"
             << Csv(gtsam::DefaultKeyFormatter(key)) << ','
             << result.final_values.at<double>(key)
             << ",FINAL_GRAPH_STATE\n";
    }
  }
  {
    auto output = OpenNew(root / "segment_bias.csv", &written);
    output << "inference_id,segment_id,segment_ordinal,c_key,amplitude_m\n"
           << std::setprecision(17);
    for (const auto& segment : result.final_segments)
      output << Csv(result.inference_id) << ',' << Csv(segment.segment_id)
             << ',' << segment.segment_ordinal << ','
             << Csv(gtsam::DefaultKeyFormatter(segment.amplitude_key)) << ','
             << result.final_values.at<double>(C(segment.segment_ordinal))
             << '\n';
  }
  {
    if (!result.requested_fixed_method.empty()) {
      auto fixed = OpenNew(root / "fixed_compensations.csv", &written);
      fixed << std::setprecision(17)
            << "segment_ordinal,segment_id,tag_id,anchor_id,candidate_observation_count,c_hat_stage2_m,sigma_c_local_m,sigma_available,delta_c_fixed_m,decision_use,final_use,reason\n";
      for (const auto& c : result.fixed_compensations)
        fixed << c.segment_ordinal << ',' << c.segment_id << ',' << c.tag_id
              << ',' << c.anchor_id << ',' << c.candidate_observation_count
              << ',' << c.c_hat_stage2_m
              << ',' << c.sigma_c_local_m
              << ',' << c.sigma_available << ',' << c.delta_c_fixed_m << ',' << c.use
              << ',' << (c.use && !result.fallback.attempted) << ',' << c.reason << '\n';
      auto method = OpenNew(root / "fixed_method.csv", &written);
      method << "requested,actual,kappa,uncertainty_semantics\n"
             << result.requested_fixed_method << ',' << result.actual_fixed_method
             << ",2,STAGE2_LOCAL_DIAGNOSTIC_NOT_FINAL_BIAS_POSTERIOR\n";
    }
    auto output = OpenNew(root / "residuals.csv", &written);
    output << "inference_id,factor_index,obs_id,factor_type,segment_id,"
              "residual_coordinate,residual_value\n"
           << std::setprecision(17);
    for (const auto& meta : result.final_factor_metadata) {
      if (meta.obs_id == 0) continue;
      const auto noise_factor = boost::dynamic_pointer_cast<
          gtsam::NoiseModelFactor>(result.final_graph.at(meta.factor_index));
      if (!noise_factor)
        throw std::runtime_error(
            "final UWB metadata points to a non-noise-model factor");
      const auto residual = noise_factor->unwhitenedError(result.final_values);
      for (Eigen::Index coordinate = 0; coordinate < residual.size();
           ++coordinate) {
        output << Csv(result.inference_id) << ',' << meta.factor_index << ','
               << meta.obs_id << ',' << Csv(meta.factor_type) << ','
               << Csv(meta.segment_id) << ',' << coordinate << ','
               << residual[coordinate] << '\n';
      }
    }
  }
  {
    auto output = OpenNew(root / "final_factor_metadata.csv", &written);
    output << "inference_id,factor_index,obs_id,factor_type,segment_id,keys\n";
    for (const auto& meta : result.final_factor_metadata) {
      std::ostringstream keys;
      for (size_t i = 0; i < meta.keys.size(); ++i) {
        if (i) keys << ';';
        keys << gtsam::DefaultKeyFormatter(meta.keys[i]);
      }
      output << Csv(result.inference_id) << ',' << meta.factor_index << ',';
      if (meta.obs_id != 0) output << meta.obs_id;
      output << ',' << Csv(meta.factor_type) << ',' << Csv(meta.segment_id)
             << ',' << Csv(keys.str()) << '\n';
    }
  }
  return written;
}

}  // namespace uifgo
