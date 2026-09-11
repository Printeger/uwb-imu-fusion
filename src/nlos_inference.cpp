#include "uifgo/imu_preint.h"
#include "uifgo/nlos_inference.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/Marginals.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <typeinfo>

#include "uifgo/hash_utils.h"
#include "uifgo/uwb_factor.h"

namespace uifgo {
namespace {

using Clock = std::chrono::steady_clock;

double SecondsSince(const Clock::time_point& start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

std::set<size_t> OrdinalSet(const SegmentOverlapGroup& group) {
  return std::set<size_t>(group.segment_ordinals.begin(),
                          group.segment_ordinals.end());
}

std::string NumericalSuppressReason(RecoverabilityStatus status) {
  switch (status) {
    case RecoverabilityStatus::RANK_DEFICIENT:
    case RecoverabilityStatus::SPARSE_RANK_UNCERTAIN:
      return "SUPPRESS_RANK_FAILURE";
    case RecoverabilityStatus::NUMERICAL_RESOLUTION_LOST:
    case RecoverabilityStatus::NUMERICAL_FAILURE:
      return "SUPPRESS_NUMERICAL_FAILURE";
    case RecoverabilityStatus::INVALID_INPUT:
      return "SUPPRESS_INVALID_SCORE";
    case RecoverabilityStatus::OK:
      break;
  }
  return "SUPPRESS_INVALID_SCORE";
}

std::string CovarianceRole(gtsam::Key key) {
  switch (gtsam::Symbol(key).chr()) {
    case 'x': return "TRAJECTORY_POSE";
    case 'v': return "TRAJECTORY_VELOCITY";
    case 'b': return "IMU_BIAS";
    case 'z': return "STATIC_RANGE_BIAS";
    case 'c': return "SEGMENT_BIAS";
    case 'a': return "ANCHOR_CORRECTION";
    case 'l': return "LEVER_ARM";
    default: return "OTHER_STATE";
  }
}

std::string CovarianceCoordinates(gtsam::Key key, size_t dimension) {
  switch (gtsam::Symbol(key).chr()) {
    case 'x':
      return "GTSAM_POSE3_LOCAL_RX_RY_RZ_TX_TY_TZ";
    case 'v': return "WORLD_VX_VY_VZ";
    case 'b': return "IMU_BAX_BAY_BAZ_BGX_BGY_BGZ";
    case 'z': return "STATIC_RANGE_BIAS_M";
    case 'c': return "SEGMENT_EXCESS_RANGE_M";
    case 'a': return "ANCHOR_DX_DY_DZ_M";
    case 'l': return "LEVER_DX_DY_DZ_M";
    default: return "GTSAM_LOCAL_COORDINATES_DIM_" + std::to_string(dimension);
  }
}

SupportPartition AcceptedSupport(
    const SupportPartition& full, const std::set<size_t>& accepted) {
  SupportPartition output = full;
  output.segments.clear();
  for (const auto& segment : full.segments) {
    if (accepted.count(segment.segment_ordinal))
      output.segments.push_back(segment);
  }
  return output;
}

FinalFactorAudit AuditFinalFactors(
    const SegmentRefitResult& refit, const SupportPartition& support,
    const std::set<size_t>& accepted, const PaperInputPlan& plan,
    bool fixed_mode = false) {
  FinalFactorAudit audit;
  std::unordered_map<std::uint64_t, const SupportSegment*> candidate_by_obs;
  for (const auto& segment : support.segments) {
    for (std::uint64_t obs_id : segment.obs_ids) {
      if (!candidate_by_obs.emplace(obs_id, &segment).second) {
        audit.reason = "DUPLICATE_FROZEN_CANDIDATE_OBS_ID";
        return audit;
      }
    }
  }

  std::unordered_map<std::uint64_t, size_t> count_by_obs;
  std::unordered_map<std::uint64_t, std::vector<const RefitFactorMeta*>>
      metadata_by_obs;
  for (size_t index = 0; index < refit.factor_metadata.size(); ++index) {
    const auto& meta = refit.factor_metadata[index];
    if (meta.factor_index != index) {
      audit.reason = "FINAL_FACTOR_INDEX_METADATA_MISMATCH";
      return audit;
    }
    const auto& factor = refit.graph.at(index);
    if (!factor || std::vector<gtsam::Key>(factor->keys().begin(),
                                          factor->keys().end()) != meta.keys) {
      audit.reason = "FINAL_FACTOR_ACTUAL_KEYS_METADATA_MISMATCH";
      return audit;
    }
    if (meta.factor_type.find("corrected") != std::string::npos ||
        meta.factor_type.find("pseudo") != std::string::npos) {
      ++audit.corrected_pseudo_range_count;
    }
    if (meta.obs_id != 0) {
      ++count_by_obs[meta.obs_id];
      metadata_by_obs[meta.obs_id].push_back(&meta);
    }
  }

  bool rows_ok = true;
  for (const auto& observation : plan.observations) {
    if (!observation.valid || !observation.planned) continue;
    FinalFactorAuditRow row;
    row.obs_id = observation.obs_id;
    row.final_factor_count = count_by_obs[observation.obs_id];
    const auto found = candidate_by_obs.find(observation.obs_id);
    if (found == candidate_by_obs.end()) {
      row.classification = "NONCANDIDATE_REFERENCE";
      row.expected_count = "1";
      row.ok = row.final_factor_count == 1 &&
               metadata_by_obs[observation.obs_id].size() == 1 &&
               metadata_by_obs[observation.obs_id][0]->factor_type ==
                   "uwb_range";
      ++audit.noncandidate_reference_count;
    } else if (accepted.count(found->second->segment_ordinal)) {
      row.classification = "ACCEPTED_CANDIDATE_RAW_WITH_LIVE_C";
      row.expected_count = "1";
      const gtsam::Key expected_c =
          gtsam::Symbol('c', found->second->segment_ordinal);
      row.ok = row.final_factor_count == 1 &&
               metadata_by_obs[observation.obs_id].size() == 1 &&
               metadata_by_obs[observation.obs_id][0]->factor_type ==
                   "uwb_segment_range" &&
               std::find(metadata_by_obs[observation.obs_id][0]->keys.begin(),
                         metadata_by_obs[observation.obs_id][0]->keys.end(),
                         expected_c) !=
                   metadata_by_obs[observation.obs_id][0]->keys.end();
      if (fixed_mode) {
        row.classification = "ACCEPTED_CANDIDATE_RAW_WITH_FIXED_OFFSET";
        row.ok = row.final_factor_count == 1 &&
            metadata_by_obs[observation.obs_id].size() == 1 &&
            metadata_by_obs[observation.obs_id][0]->factor_type == "uwb_fixed_offset_range";
      }
      ++audit.accepted_candidate_count;
    } else {
      row.classification = "SUPPRESSED_CANDIDATE";
      row.expected_count = "0";
      row.ok = row.final_factor_count == 0;
      ++audit.suppressed_candidate_count;
    }
    rows_ok = rows_ok && row.ok;
    audit.observations.push_back(std::move(row));
  }

  bool keys_ok = true;
  for (const auto& segment : support.segments) {
    const bool should_exist = !fixed_mode && accepted.count(segment.segment_ordinal) != 0;
    const bool exists = refit.values.exists(
        gtsam::Symbol('c', segment.segment_ordinal));
    keys_ok = keys_ok && should_exist == exists;
  }
  if (fixed_mode) {
    for (auto key : refit.values.keys()) keys_ok = keys_ok && gtsam::Symbol(key).chr() != 'c';
    for (const auto& factor : refit.graph)
      for (auto key : factor->keys()) keys_ok = keys_ok && gtsam::Symbol(key).chr() != 'c';
  }
  audit.ok = rows_ok && keys_ok && audit.corrected_pseudo_range_count == 0 &&
             refit.factor_metadata.size() == refit.graph.size();
  if (!rows_ok)
    audit.reason = "FINAL_RAW_RANGE_FACTOR_MULTIPLICITY_MISMATCH";
  else if (!keys_ok)
    audit.reason = "FINAL_C_KEY_ACCEPTANCE_MISMATCH";
  else if (audit.corrected_pseudo_range_count != 0)
    audit.reason = "CORRECTED_PSEUDO_RANGE_PRESENT";
  else if (refit.factor_metadata.size() != refit.graph.size())
    audit.reason = "FINAL_FACTOR_METADATA_SIZE_MISMATCH";
  else
    audit.reason = "FINAL_FACTOR_AND_KEY_AUDIT_OK";
  return audit;
}

std::vector<FrozenObservationMask> BuildMasks(
    const SupportPartition& support,
    const std::vector<GroupDecisionRecord>& decisions,
    const PaperInputPlan& plan) {
  std::unordered_map<size_t, const GroupDecisionRecord*> group_by_ordinal;
  for (const auto& decision : decisions) {
    for (size_t ordinal : decision.group.segment_ordinals)
      group_by_ordinal.emplace(ordinal, &decision);
  }
  std::unordered_map<std::uint64_t, const SupportSegment*> segment_by_obs;
  for (const auto& segment : support.segments)
    for (std::uint64_t obs_id : segment.obs_ids)
      segment_by_obs.emplace(obs_id, &segment);

  std::vector<FrozenObservationMask> output;
  output.reserve(plan.observations.size());
  for (const auto& observation : plan.observations) {
    FrozenObservationMask mask;
    mask.obs_id = observation.obs_id;
    if (!observation.valid || !observation.planned) {
      mask.reason_code = "NOT_IN_FROZEN_VALID_PLAN";
      output.push_back(std::move(mask));
      continue;
    }
    const auto segment = segment_by_obs.find(observation.obs_id);
    if (segment == segment_by_obs.end()) {
      mask.noncandidate_reference = true;
      mask.reason_code = "NONCANDIDATE_REFERENCE";
      output.push_back(std::move(mask));
      continue;
    }
    mask.candidate = true;
    mask.segment_id = segment->second->segment_id;
    const auto group = group_by_ordinal.find(segment->second->segment_ordinal);
    if (group == group_by_ordinal.end()) {
      mask.reason_code = "CANDIDATE_GROUP_DECISION_MISSING";
    } else {
      mask.group_id = group->second->group.group_id;
      mask.decision_use = group->second->decision == GroupDecision::USE;
      mask.reason_code = group->second->reason_code;
    }
    output.push_back(std::move(mask));
  }
  return output;
}

void SetFinalMaskUse(std::vector<FrozenObservationMask>* masks,
                     bool fallback, bool valid_estimate) {
  for (auto& mask : *masks) {
    mask.final_use = valid_estimate &&
        (mask.noncandidate_reference || (!fallback && mask.decision_use));
    mask.fallback_use = valid_estimate && fallback &&
                        mask.noncandidate_reference;
  }
}

void AppendString(std::ostringstream* bytes, const std::string& value) {
  *bytes << value.size() << ':' << value << '\n';
}

void AppendUint64(std::ostringstream* bytes, std::uint64_t value) {
  *bytes << std::hex << std::setw(16) << std::setfill('0') << value
         << std::dec << '\n';
}

void AppendDouble(std::ostringstream* bytes, double value) {
  static_assert(sizeof(double) == sizeof(std::uint64_t),
                "T08 identity requires binary64 doubles");
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  AppendUint64(bytes, bits);
}

void AppendMatrix(std::ostringstream* bytes, const Eigen::MatrixXd& matrix) {
  *bytes << matrix.rows() << ':' << matrix.cols() << '\n';
  for (Eigen::Index row = 0; row < matrix.rows(); ++row)
    for (Eigen::Index column = 0; column < matrix.cols(); ++column)
      AppendDouble(bytes, matrix(row, column));
}

std::string ValuesContentHash(const gtsam::Values& values) {
  std::ostringstream bytes;
  bytes << "uifgo-t08-values-ieee754-v1\n" << values.size() << '\n';
  for (gtsam::Key key : values.keys()) {
    AppendUint64(&bytes, key);
    const char symbol = gtsam::Symbol(key).chr();
    bytes << symbol << '\n';
    if (symbol == 'x') {
      AppendString(&bytes, "Pose3-matrix4x4");
      AppendMatrix(&bytes, values.at<gtsam::Pose3>(key).matrix());
    } else if (symbol == 'v' || symbol == 'a' || symbol == 'l') {
      AppendString(&bytes, "Vector3");
      const gtsam::Vector3 value = values.at<gtsam::Vector3>(key);
      AppendMatrix(&bytes, value);
    } else if (symbol == 'b') {
      AppendString(&bytes, "ConstantBias-accel-gyro");
      const auto bias = values.at<gtsam::imuBias::ConstantBias>(key);
      AppendMatrix(&bytes, bias.accelerometer());
      AppendMatrix(&bytes, bias.gyroscope());
    } else if (symbol == 'c' || symbol == 'z' || symbol == 't') {
      AppendString(&bytes, "double");
      AppendDouble(&bytes, values.at<double>(key));
    } else {
      throw std::runtime_error(
          "unsupported Values key type in T08 content identity: " +
          gtsam::DefaultKeyFormatter(key));
    }
  }
  return "t08values-sha256:" + Sha256Hex(bytes.str());
}

std::string GraphLinearizationContentHash(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) {
  std::ostringstream bytes;
  bytes << "uifgo-t08-factorwise-final-linearization-ieee754-v1\n"
        << graph.size() << '\n';
  for (size_t index = 0; index < graph.size(); ++index) {
    const auto& factor = graph.at(index);
    bytes << index << '\n';
    if (!factor) {
      AppendString(&bytes, "NULL_FACTOR");
      continue;
    }
    AppendString(&bytes, typeid(*factor).name());
    bytes << factor->keys().size() << '\n';
    for (gtsam::Key key : factor->keys()) AppendUint64(&bytes, key);
    AppendDouble(&bytes, factor->error(values));
    const auto linear = factor->linearize(values);
    if (!linear)
      throw std::runtime_error("factor returned null final linearization");
    AppendString(&bytes, typeid(*linear).name());
    AppendMatrix(&bytes, linear->augmentedJacobian());
  }
  return "t08graphlin-sha256:" + Sha256Hex(bytes.str());
}

std::string IdentityContextHash(const InferenceIdentityContext& context) {
  if (context.input_sha256.empty() || context.config_sha256.empty() ||
      context.input_plan_sha256.empty() ||
      context.support_partition_sha256.empty()) {
    throw std::invalid_argument(
        "T08 identity requires input/config/plan/support content identities");
  }
  std::ostringstream bytes;
  bytes << (context.validation_context_sha256.empty()
                ? "uifgo-t08-identity-context-v1\n"
                : "uifgo-t10-validation-identity-context-v2\n");
  AppendString(&bytes, context.input_sha256);
  AppendString(&bytes, context.config_sha256);
  AppendString(&bytes, context.input_plan_sha256);
  AppendString(&bytes, context.support_partition_sha256);
  AppendString(&bytes, context.calibration_sha256);
  AppendString(&bytes, context.solver_config_sha256);
  if (!context.validation_context_sha256.empty())
    AppendString(&bytes, context.validation_context_sha256);
  return "t08context-sha256:" + Sha256Hex(bytes.str());
}

std::string InferenceId(const InferenceResult& result) {
  std::ostringstream bytes;
  bytes << "uifgo-t08-inference-content-bound-v2\n";
  AppendString(&bytes, InferenceStatusName(result.status));
  AppendString(&bytes, result.content_identity.graph_linearization_sha256);
  AppendString(&bytes, result.content_identity.values_sha256);
  AppendString(&bytes, result.content_identity.context_sha256);
  for (const auto& meta : result.final_factor_metadata) {
    bytes << meta.factor_index << ':' << meta.obs_id << ':' << meta.factor_type
          << ':' << meta.segment_id;
    for (gtsam::Key key : meta.keys) bytes << ':' << key;
    bytes << '\n';
  }
  for (const auto& mask : result.frozen_masks)
    bytes << mask.obs_id << ':' << mask.candidate << ':' << mask.decision_use
          << ':' << mask.final_use << ':' << mask.fallback_use << '\n';
  if (!result.requested_fixed_method.empty()) {
    AppendString(&bytes, result.requested_fixed_method);
    AppendString(&bytes, result.actual_fixed_method);
    AppendDouble(&bytes, result.fixed_kappa);
    for (const auto& c : result.fixed_compensations) {
      AppendUint64(&bytes, c.segment_ordinal); AppendString(&bytes,c.segment_id);
      AppendDouble(&bytes,c.c_hat_stage2_m); AppendDouble(&bytes,c.sigma_c_local_m);
      AppendDouble(&bytes,c.delta_c_fixed_m); bytes << c.use << c.sigma_available;
      AppendString(&bytes,c.reason);
    }
  }
  return "t08inference-sha256:" + Sha256Hex(bytes.str());
}

void SealInferenceIdentity(InferenceResult* result) {
  result->content_identity = ComputeInferenceContentIdentity(
      result->final_graph, result->final_values, result->identity_context);
  result->inference_id = InferenceId(*result);
}

void SetInapplicableFinalScores(
    const std::vector<GroupDecisionRecord>& decisions,
    const std::string& status, std::vector<FinalGroupScore>* output) {
  output->clear();
  output->reserve(decisions.size());
  for (const auto& decision : decisions) {
    FinalGroupScore final_score;
    final_score.group = decision.group;
    final_score.status = status;
    final_score.linearization_id = "NOT_APPLICABLE";
    output->push_back(std::move(final_score));
  }
}

void EnsureExplicitScoreRows(
    const std::vector<GroupDecisionRecord>& decisions,
    const std::string& missing_status, std::vector<FinalGroupScore>* output) {
  for (const auto& decision : decisions) {
    const auto found = std::find_if(output->begin(), output->end(),
        [&](const FinalGroupScore& score) {
          return OrdinalSet(score.group) == OrdinalSet(decision.group);
        });
    if (found != output->end()) continue;
    FinalGroupScore score;
    score.group = decision.group;
    score.status = missing_status;
    score.linearization_id = "NOT_RUN";
    output->push_back(std::move(score));
  }
}

RefitAttemptDiagnostics AttemptDiagnostics(const SegmentRefitResult& refit) {
  RefitAttemptDiagnostics output;
  output.executed = refit.status != SegmentRefitStatus::SUCCESS_EMPTY;
  output.execution_status = refit.status == SegmentRefitStatus::SUCCESS_EMPTY
      ? "REUSED_REFERENCE_ZERO_OPTIMIZER_CALLS" : (refit.successful() ? "SUCCEEDED" : "FAILED");
  output.solver_status = SegmentRefitStatusName(refit.status);
  output.stop_reason = refit.reason;
  output.iterations = refit.iterations;
  return output;
}

}  // namespace

InferenceContentIdentity ComputeInferenceContentIdentity(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const InferenceIdentityContext& context) {
  InferenceContentIdentity identity;
  identity.graph_linearization_sha256 =
      GraphLinearizationContentHash(graph, values);
  identity.values_sha256 = ValuesContentHash(values);
  identity.context_sha256 = IdentityContextHash(context);
  return identity;
}

bool VerifyInferenceContentIdentity(const InferenceResult& result,
                                    std::string* reason) {
  auto fail = [&](const std::string& text) {
    if (reason) *reason = text;
    return false;
  };
  try {
    const auto actual = ComputeInferenceContentIdentity(
        result.final_graph, result.final_values, result.identity_context);
    if (actual.schema != result.content_identity.schema ||
        actual.graph_linearization_sha256 !=
            result.content_identity.graph_linearization_sha256 ||
        actual.values_sha256 != result.content_identity.values_sha256 ||
        actual.context_sha256 != result.content_identity.context_sha256)
      return fail("FINAL_GRAPH_VALUES_CONTEXT_FINGERPRINT_MISMATCH");
    if (InferenceId(result) != result.inference_id)
      return fail("AGGREGATE_INFERENCE_ID_MISMATCH");
  } catch (const std::exception& error) {
    return fail(std::string("IDENTITY_RECOMPUTE_FAILED: ") + error.what());
  }
  if (reason) reason->clear();
  return true;
}

bool ValidDevelopmentGateThresholds(const GateThresholds& thresholds,
                                    std::string* reason) {
  auto fail = [&](const std::string& text) {
    if (reason) *reason = text;
    return false;
  };
  if (!std::isfinite(thresholds.tau_eta) ||
      !std::isfinite(thresholds.tau_s_m) ||
      !std::isfinite(thresholds.tau_gamma))
    return fail("gate thresholds must be finite");
  if (thresholds.tau_eta < 0.0 || thresholds.tau_eta > 1.0 ||
      thresholds.tau_s_m < 0.0 || thresholds.tau_gamma < 0.0)
    return fail("gate thresholds are outside their declared domains");
  if (thresholds.parameter_provenance != kT08DevelopmentGateLabel)
    return fail("T08 gate provenance must remain development-only pending validation");
  if (reason) reason->clear();
  return true;
}

const char* GroupDecisionName(GroupDecision decision) {
  return decision == GroupDecision::USE ? "USE" : "SUPPRESS";
}

const char* FinalGatePolicyName(FinalGatePolicy policy) {
  switch (policy) {
    case FinalGatePolicy::LCB_PARTIAL: return "lcb_partial";
    case FinalGatePolicy::LCB_FIXED_FULL: return "lcb_fixed_full";
    case FinalGatePolicy::SUPPRESS_ALL: return "suppress_all";
    case FinalGatePolicy::STRUCTURED_DEBIAS: return "structured_debias";
    case FinalGatePolicy::FULL_GATE: return "full_gate";
    case FinalGatePolicy::FIT_ONLY: return "fit_only";
    case FinalGatePolicy::S_FIT: return "s_fit";
    case FinalGatePolicy::ETA_ONLY: return "eta_only";
  }
  return "full_gate";
}

std::vector<GroupDecisionRecord> FreezeGroupDecisions(
    const std::vector<GroupRecoverabilityScore>& scores,
    const GateThresholds& thresholds, FinalGatePolicy policy) {
  if (policy == FinalGatePolicy::LCB_PARTIAL || policy == FinalGatePolicy::LCB_FIXED_FULL)
    throw std::invalid_argument("LCB requires Stage2 amplitude/key mapping");
  std::string threshold_reason;
  if (!ValidDevelopmentGateThresholds(thresholds, &threshold_reason))
    throw std::invalid_argument(threshold_reason);
  std::vector<GroupDecisionRecord> output;
  output.reserve(scores.size());
  for (const auto& score : scores) {
    GroupDecisionRecord decision;
    decision.group = score.group;
    decision.eligible = score.eligible;
    decision.decision_linearization_id = score.linearization_id;
    bool short_support = false;
    bool boundary = false;
    bool gamma_valid = !score.segment_fit.empty();
    std::set<size_t> fit_ordinals;
    double max_gamma = -std::numeric_limits<double>::infinity();
    for (const auto& fit : score.segment_fit) {
      short_support = short_support || fit.short_support_debug;
      boundary = boundary || fit.boundary;
      gamma_valid = gamma_valid && std::isfinite(fit.gamma);
      gamma_valid = gamma_valid &&
                    fit_ordinals.insert(fit.segment_ordinal).second;
      if (std::isfinite(fit.gamma)) max_gamma = std::max(max_gamma, fit.gamma);
    }
    gamma_valid = gamma_valid && fit_ordinals == OrdinalSet(score.group);
    if (gamma_valid) {
      decision.max_gamma = max_gamma;
      decision.max_gamma_available = true;
    }
    if (!score.eligible) {
      if (policy == FinalGatePolicy::STRUCTURED_DEBIAS) {
        decision.decision = GroupDecision::USE;
        decision.reason_code = "USE_STRUCTURED_DEBIAS_NO_GATE";
      } else {
        decision.reason_code = short_support
                                 ? "SUPPRESS_SHORT_SUPPORT"
                                 : (boundary
                                        ? "SUPPRESS_BOUNDARY_OR_KKT_INVALID"
                                        : "SUPPRESS_INSUFFICIENT_SUPPORT");
      }
    } else if (policy == FinalGatePolicy::SUPPRESS_ALL) {
      decision.reason_code = "SUPPRESS_ALL_REFERENCE_POLICY";
    } else if (policy == FinalGatePolicy::STRUCTURED_DEBIAS) {
      decision.decision = GroupDecision::USE;
      decision.reason_code = "USE_STRUCTURED_DEBIAS_NO_GATE";
    } else if (policy != FinalGatePolicy::FULL_GATE) {
      const bool eta_available = score.numerical.valid_score() &&
                                 std::isfinite(score.numerical.eta);
      const bool s_available = score.numerical.valid_score() &&
                               (std::isfinite(score.numerical.s_m) ||
                                score.numerical.s_is_infinite);
      const bool required_available =
          policy == FinalGatePolicy::FIT_ONLY
              ? gamma_valid
              : (policy == FinalGatePolicy::S_FIT
                     ? s_available && gamma_valid
                     : eta_available);
      decision.eta_pass = eta_available &&
                          score.numerical.eta >= thresholds.tau_eta;
      decision.s_pass = s_available && !score.numerical.s_is_infinite &&
                        score.numerical.s_m <= thresholds.tau_s_m;
      decision.gamma_pass = gamma_valid &&
                            max_gamma <= thresholds.tau_gamma;
      const bool passed =
          policy == FinalGatePolicy::FIT_ONLY
              ? decision.gamma_pass
              : (policy == FinalGatePolicy::S_FIT
                     ? decision.s_pass && decision.gamma_pass
                     : decision.eta_pass);
      if (!required_available) {
        decision.reason_code = "SUPPRESS_REQUIRED_SCORE_UNAVAILABLE";
      } else if (!passed) {
        decision.reason_code =
            policy == FinalGatePolicy::FIT_ONLY
                ? "SUPPRESS_GAMMA_ABOVE_THRESHOLD"
                : (policy == FinalGatePolicy::S_FIT
                       ? (!decision.s_pass
                              ? "SUPPRESS_S_ABOVE_THRESHOLD"
                              : "SUPPRESS_GAMMA_ABOVE_THRESHOLD")
                       : "SUPPRESS_ETA_BELOW_THRESHOLD");
      } else {
        decision.decision = GroupDecision::USE;
        decision.reason_code = "USE_ALL_REQUIRED_PREDICATES_PASSED";
      }
    } else if (score.numerical.status == RecoverabilityStatus::RANK_DEFICIENT ||
               score.numerical.status ==
                   RecoverabilityStatus::SPARSE_RANK_UNCERTAIN) {
      decision.reason_code = "SUPPRESS_RANK_FAILURE";
    } else if (!score.valid_score_exported ||
               score.numerical.status != RecoverabilityStatus::OK) {
      decision.reason_code = NumericalSuppressReason(score.numerical.status);
    } else if (!std::isfinite(score.numerical.eta) ||
               !std::isfinite(score.numerical.s_m) || !gamma_valid) {
      decision.reason_code = "SUPPRESS_INVALID_SCORE";
    } else {
      decision.eta_pass = score.numerical.eta >= thresholds.tau_eta;
      decision.s_pass = score.numerical.s_m <= thresholds.tau_s_m;
      decision.gamma_pass = max_gamma <= thresholds.tau_gamma;
      if (!decision.eta_pass)
        decision.reason_code = "SUPPRESS_ETA_BELOW_THRESHOLD";
      else if (!decision.s_pass)
        decision.reason_code = "SUPPRESS_S_ABOVE_THRESHOLD";
      else if (!decision.gamma_pass)
        decision.reason_code = "SUPPRESS_GAMMA_ABOVE_THRESHOLD";
      else {
        decision.decision = GroupDecision::USE;
        decision.reason_code = "USE_ALL_THRESHOLDS_PASSED";
      }
    }
    output.push_back(std::move(decision));
  }
  return output;
}

const char* CovarianceStatusName(CovarianceStatus status) {
  return status == CovarianceStatus::AVAILABLE ? "AVAILABLE" : "UNAVAILABLE";
}

FinalCovarianceResult ComputeFinalGraphCovariance(
    const gtsam::NonlinearFactorGraph& final_graph,
    const gtsam::Values& final_values) {
  FinalCovarianceResult output;
  if (final_graph.empty() || final_values.empty()) {
    output.reason = "EMPTY_FINAL_GRAPH_OR_VALUES";
    return output;
  }
  try {
    gtsam::Marginals marginals(final_graph, final_values,
                               gtsam::Marginals::CHOLESKY);
    for (gtsam::Key key : final_values.keys()) {
      FinalCovarianceBlock block;
      block.key = key;
      block.role = CovarianceRole(key);
      block.covariance = marginals.marginalCovariance(key);
      block.coordinates = CovarianceCoordinates(
          key, static_cast<size_t>(block.covariance.rows()));
      if (block.covariance.rows() == 0 ||
          block.covariance.rows() != block.covariance.cols() ||
          !block.covariance.allFinite()) {
        output.blocks.clear();
        output.reason = "NONFINITE_OR_INVALID_MARGINAL_BLOCK";
        return output;
      }
      output.blocks.push_back(std::move(block));
    }
    output.status = CovarianceStatus::AVAILABLE;
    output.reason = "FULL_FINAL_GRAPH_MARGINALS_COMPUTED";
  } catch (const std::exception& error) {
    output.blocks.clear();
    output.reason = std::string("MARGINAL_SOLVE_FAILED: ") + error.what();
  }
  return output;
}

const char* InferenceStatusName(InferenceStatus status) {
  switch (status) {
    case InferenceStatus::OK: return "OK";
    case InferenceStatus::NO_CANDIDATES: return "NO_CANDIDATES";
    case InferenceStatus::NO_ELIGIBLE_CANDIDATES:
      return "NO_ELIGIBLE_CANDIDATES";
    case InferenceStatus::ZERO_ACCEPTED: return "ZERO_ACCEPTED";
    case InferenceStatus::FALLBACK_OK: return "FALLBACK_OK";
    case InferenceStatus::ESTIMATION_FAILED: return "ESTIMATION_FAILED";
  }
  return "ESTIMATION_FAILED";
}

InferenceResult FinalInferenceEngine::Run(
    const gtsam::NonlinearFactorGraph& frozen_raw_graph,
    const std::vector<FactorMeta>& frozen_raw_uwb_metadata,
    const SegmentRefitResult& stage2_refit,
    const SupportPartition& frozen_full_support,
    const std::vector<GroupRecoverabilityScore>& decision_scores,
    const PaperInputPlan& plan, const Config& cfg,
    const InferenceIdentityContext& identity_context) const {
  const auto total_started = Clock::now();
  InferenceResult result;
  result.gate_thresholds = thresholds_;
  result.identity_context = identity_context;
  // Validate mandatory identity inputs before doing Stage 4 work.
  IdentityContextHash(identity_context);
  auto finish = [&]() {
    result.timing.total_seconds = SecondsSince(total_started);
    SealInferenceIdentity(&result);
  };
  std::string imu_model_reason;
  if (!PaperImuCovarianceModelMatchesGraph(frozen_raw_graph, cfg, &imu_model_reason) ||
      !PaperImuCovarianceModelMatchesGraph(stage2_refit.graph, cfg, &imu_model_reason)) {
    result.reason = imu_model_reason;
    finish();
    return result;
  }
  if (!frozen_full_support.calibration_hash.empty() &&
      frozen_full_support.calibration_hash != identity_context.calibration_sha256) {
    result.reason = "FINAL_SUPPORT_CALIBRATION_IMU_CONTEXT_MISMATCH";
    finish();
    return result;
  }
  for (const auto& keyframe : plan.keyframes)
    result.final_keyframe_times_s.push_back(keyframe.sensor_time);
  for (const auto& item : cfg.fixed_beta_by_link)
    result.fixed_static_biases_m.push_back(item);
  result.decision_scores = decision_scores;
  std::string gate_reason;
  if (!ValidDevelopmentGateThresholds(thresholds_, &gate_reason)) {
    result.reason = gate_reason;
    finish();
    return result;
  }
  if (!stage2_refit.successful()) {
    result.reason = "STAGE2_REFIT_NOT_CONVERGED";
    finish();
    return result;
  }

  if (stage2_refit.status == SegmentRefitStatus::SUCCESS_EMPTY) {
    if (!frozen_full_support.segments.empty() || !stage2_refit.segments.empty() ||
        !stage2_refit.iterations.empty() ||
        frozen_full_support.provider !=
            (cfg.fde_windowed_test
                 ? "imu_aided_windowed_fde_v4"
                 : (cfg.fde_grouped_test
                        ? "imu_aided_grouped_fde_v3"
                        : "imu_aided_postfit_fde_v2"))) {
      result.reason = "SUCCESS_EMPTY_NONEMPTY_SUPPORT_OR_TRACE";
      finish(); return result;
    }
    const auto rebuilt = SegmentRefitter(refit_options_).RunFrozenCandidatePolicy(
        frozen_raw_graph, stage2_refit.values, frozen_raw_uwb_metadata, plan, cfg,
        frozen_full_support, {}, false);
    if (!rebuilt.successful()) {
      result.reason = "SUCCESS_EMPTY_FINAL_REBUILD_FAILED:" + rebuilt.reason;
      finish(); return result;
    }
    const auto actual = ComputeInferenceContentIdentity(stage2_refit.graph, stage2_refit.values, identity_context);
    const auto expected = ComputeInferenceContentIdentity(rebuilt.graph, rebuilt.values, identity_context);
    if (actual.graph_linearization_sha256 != expected.graph_linearization_sha256 ||
        actual.values_sha256 != expected.values_sha256) {
      result.reason = "SUCCESS_EMPTY_FINAL_GRAPH_IDENTITY_MISMATCH";
      finish(); return result;
    }
  }

  const bool fixed_mode = policy_ == FinalGatePolicy::LCB_PARTIAL ||
                          policy_ == FinalGatePolicy::LCB_FIXED_FULL;
  std::map<size_t,double> fixed_offsets;
  if (fixed_mode) {
    result.requested_fixed_method = FinalGatePolicyName(policy_);
    result.actual_fixed_method = result.requested_fixed_method;
  }
  const auto decision_started = Clock::now();
  try {
    result.decisions = FreezeGroupDecisions(decision_scores, thresholds_,
                                            fixed_mode ? FinalGatePolicy::SUPPRESS_ALL : policy_);
  } catch (const std::exception& error) {
    result.reason = std::string("DECISION_INPUT_INVALID: ") + error.what();
    finish();
    return result;
  }
  if (fixed_mode) {
    result.fixed_compensations = FreezeFixedCompensations(stage2_refit, decision_scores,
                                  policy_ == FinalGatePolicy::LCB_FIXED_FULL);
    for (const auto& c : result.fixed_compensations)
      if (c.use) fixed_offsets.emplace(c.segment_ordinal,c.delta_c_fixed_m);
    for (auto& d : result.decisions) {
      d.reason_code = "LCB_PER_SEGMENT_SEE_FIXED_COMPENSATIONS";
      for (size_t ordinal : d.group.segment_ordinals)
        if (fixed_offsets.count(ordinal)) d.decision = GroupDecision::USE;
    }
  }
  result.timing.decision_seconds = SecondsSince(decision_started);
  result.frozen_masks = BuildMasks(frozen_full_support, result.decisions, plan);

  if (!frozen_full_support.segments.empty()) {
    const auto expected_groups = BuildClosedIntervalOverlapGroups(
        stage2_refit.segments);
    if (expected_groups.size() != result.decisions.size()) {
      result.reason = "FROZEN_GROUP_SCORE_COVERAGE_MISMATCH";
      finish();
      return result;
    }
    std::multiset<std::set<size_t>> expected, actual;
    for (const auto& group : expected_groups) expected.insert(OrdinalSet(group));
    for (const auto& decision : result.decisions)
      actual.insert(OrdinalSet(decision.group));
    if (expected != actual) {
      result.reason = "FROZEN_GROUP_IDENTITY_MISMATCH";
      finish();
      return result;
    }
  } else if (!decision_scores.empty()) {
    result.reason = "SCORES_PRESENT_WITHOUT_CANDIDATES";
    finish();
    return result;
  }

  std::set<size_t> accepted;
  size_t eligible_groups = 0;
  for (const auto& decision : result.decisions) {
    eligible_groups += decision.eligible;
    if (decision.decision == GroupDecision::USE)
      accepted.insert(decision.group.segment_ordinals.begin(),
                      decision.group.segment_ordinals.end());
  }

  if (fixed_mode) {
    accepted.clear();
    for (const auto& c : result.fixed_compensations) if(c.use) accepted.insert(c.segment_ordinal);
    for(auto& mask: result.frozen_masks) {
      if(!mask.candidate) continue;
      mask.decision_use = false;
      for(const auto& c:result.fixed_compensations) if(c.segment_id==mask.segment_id) {
        mask.decision_use=c.use; mask.reason_code=c.reason;
      }
    }
  }
  SegmentRefitResult recovery;
  bool recovery_ok = true;
  std::string recovery_failure;
  const auto recovery_started = Clock::now();
  if (frozen_full_support.segments.empty()) {
    recovery = stage2_refit;
  } else if (fixed_mode) {
    recovery = SegmentRefitter(refit_options_).RunFixedOffsets(
        frozen_raw_graph, stage2_refit.values, frozen_raw_uwb_metadata,
        plan, cfg, frozen_full_support, fixed_offsets,
        development_empty_recovery_request_);
  } else if (development_refit_request_ && !accepted.empty()) {
    recovery = SegmentRefitter(refit_options_)
        .RunDevelopmentFrozenCandidatePolicy(
            frozen_raw_graph, stage2_refit.values, frozen_raw_uwb_metadata,
            plan, cfg, frozen_full_support, accepted,
            development_refit_request_);
  } else if (development_empty_recovery_request_ && accepted.empty()) {
    recovery = SegmentRefitter(refit_options_)
        .RunDevelopmentFrozenCandidatePolicy(
            frozen_raw_graph, stage2_refit.values, frozen_raw_uwb_metadata,
            plan, cfg, frozen_full_support, accepted,
            development_empty_recovery_request_);
  } else {
    recovery = SegmentRefitter(refit_options_).RunFrozenCandidatePolicy(
        frozen_raw_graph, stage2_refit.values, frozen_raw_uwb_metadata, plan,
        cfg, frozen_full_support, accepted);
  }
  result.timing.recovery_refit_seconds = SecondsSince(recovery_started);
  result.recovery_attempt = AttemptDiagnostics(recovery);
  if (!recovery.successful()) {
    recovery_ok = false;
    recovery_failure = std::string("RECOVERY_REFIT_FAILED_") +
                       SegmentRefitStatusName(recovery.status) + ": " +
                       recovery.reason;
  }
  if (test_hooks_.force_recovery_failure) {
    recovery_ok = false;
    recovery_failure = "TEST_ONLY_FORCED_RECOVERY_FAILURE";
    result.recovery_attempt.execution_status = "INVALIDATED_TEST_ONLY";
  }

  if (recovery_ok) {
    result.factor_audit = AuditFinalFactors(
        recovery, frozen_full_support, accepted, plan, fixed_mode);
    if (fixed_mode && result.factor_audit.ok) {
      for (const auto& meta : recovery.factor_metadata) {
        if (meta.factor_type != "uwb_fixed_offset_range") continue;
        const auto obs = std::find_if(plan.observations.begin(),plan.observations.end(),
            [&](const auto& r){return r.obs_id==meta.obs_id;});
        const auto c = std::find_if(result.fixed_compensations.begin(),result.fixed_compensations.end(),
            [&](const auto& r){return r.segment_id==meta.segment_id && r.use;});
        if (obs==plan.observations.end() || c==result.fixed_compensations.end()) {
          result.factor_audit.ok=false; result.factor_audit.reason="FIXED_OFFSET_MAPPING_MISSING"; break;
        }
        const auto anchor=std::find_if(cfg.anchors.begin(),cfg.anchors.end(),
            [&](const auto& a){return a.id==obs->anchor_id;});
        if(anchor==cfg.anchors.end()) {result.factor_audit.ok=false;result.factor_audit.reason="FIXED_ANCHOR_MISSING";break;}
        const auto expected=MakeFixedOffsetUwbFactor(gtsam::Symbol('x',obs->keyframe_id),
            anchor->pos,cfg.lever_arm_init,obs->raw_range,obs->nominal_sigma,
            FixedBetaForLink(cfg,obs->tag_id,obs->anchor_id),c->delta_c_fixed_m);
        const auto actual=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(recovery.graph.at(meta.factor_index));
        const auto reference=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(expected);
        if(!actual || (actual->unwhitenedError(recovery.values)-reference->unwhitenedError(recovery.values)).norm()>1e-12 ||
            (actual->linearize(recovery.values)->augmentedJacobian()-reference->linearize(recovery.values)->augmentedJacobian()).norm()>1e-12) {
          result.factor_audit.ok=false; result.factor_audit.reason="FIXED_OFFSET_FACTOR_CONTENT_MISMATCH";break;
        }
      }
    }
    if (!result.factor_audit.ok) {
      recovery_ok = false;
      recovery_failure = "RECOVERY_FACTOR_AUDIT_FAILED: " +
                         result.factor_audit.reason;
      result.recovery_attempt.acceptance_audit_status = "FAILED";
      result.recovery_attempt.acceptance_failure_reason = recovery_failure;
    }
  }

  if (recovery_ok && fixed_mode) {
    SetInapplicableFinalScores(result.decisions,
        "NOT_APPLICABLE_FIXED_OFFSET_NO_LIVE_C", &result.recovery_final_scores);
    result.recovery_attempt.acceptance_audit_status = "PASSED_FIXED_GRAPH_AUDIT";
  } else if (recovery_ok && !accepted.empty()) {
    const auto score_started = Clock::now();
    try {
      const SupportPartition accepted_support =
          AcceptedSupport(frozen_full_support, accepted);
      const auto scored = ScoreFinalRefitRecoverability(
          recovery, accepted_support, frozen_full_support, plan, cfg,
          score_options_);
      for (const auto& decision : result.decisions) {
        FinalGroupScore final_score;
        final_score.group = decision.group;
        if (decision.decision == GroupDecision::SUPPRESS) {
          final_score.status =
              "FROZEN_SUPPRESSED_NO_FINAL_C_KEY_NO_READMISSION";
          final_score.linearization_id = "t08final-sha256:" + Sha256Hex(
              decision.group.group_id + ":suppressed:" +
              decision.decision_linearization_id);
          result.recovery_final_scores.push_back(std::move(final_score));
          continue;
        }
        const auto found = std::find_if(
            scored.begin(), scored.end(), [&](const auto& item) {
              return OrdinalSet(item.group) == OrdinalSet(decision.group);
            });
        if (found == scored.end())
          throw std::runtime_error("accepted group missing from final score set");
        final_score.evaluated = true;
        final_score.score = *found;
        final_score.status = found->status;
        final_score.linearization_id = "t08final-sha256:" + Sha256Hex(
            decision.group.group_id + ":" + found->linearization_id);
        auto final_decision = FreezeGroupDecisions({*found}, thresholds_,
                                                   fixed_mode ? FinalGatePolicy::SUPPRESS_ALL : policy_);
        if (final_decision.size() != 1 ||
            final_decision.front().decision != GroupDecision::USE) {
          recovery_ok = false;
          recovery_failure = "FINAL_ACCEPTANCE_REAUDIT_FAILED_" +
              decision.group.group_id + ":" +
              (final_decision.empty() ? std::string("MISSING")
                                      : final_decision.front().reason_code);
          result.recovery_attempt.acceptance_audit_status = "FAILED";
          result.recovery_attempt.acceptance_failure_reason = recovery_failure;
        }
        result.recovery_final_scores.push_back(std::move(final_score));
      }
      if (recovery_ok)
        result.recovery_attempt.acceptance_audit_status = "PASSED";
    } catch (const std::exception& error) {
      recovery_ok = false;
      recovery_failure = std::string("FINAL_SCORE_AUDIT_FAILED: ") +
                         error.what();
      result.recovery_attempt.acceptance_audit_status = "FAILED";
      result.recovery_attempt.acceptance_failure_reason = recovery_failure;
    }
    result.timing.final_score_seconds = SecondsSince(score_started);
  } else if (recovery_ok) {
    for (const auto& decision : result.decisions) {
      FinalGroupScore final_score;
      final_score.group = decision.group;
      final_score.status = "FROZEN_SUPPRESSED_NO_FINAL_C_KEY_NO_READMISSION";
      final_score.linearization_id = "t08final-sha256:" + Sha256Hex(
          decision.group.group_id + ":suppressed:" +
          decision.decision_linearization_id);
      result.recovery_final_scores.push_back(std::move(final_score));
    }
    result.recovery_attempt.acceptance_audit_status = "NOT_APPLICABLE";
  }
  if (!recovery_ok) {
    if (result.recovery_attempt.acceptance_audit_status == "NOT_RUN") {
      result.recovery_attempt.acceptance_audit_status = "FAILED_BEFORE_SCORE";
      result.recovery_attempt.acceptance_failure_reason = recovery_failure;
    }
    EnsureExplicitScoreRows(
        result.decisions, "RECOVERY_SCORE_NOT_RUN_DUE_TO_PRIOR_FAILURE",
        &result.recovery_final_scores);
  }

  bool used_fallback = false;
  if (!recovery_ok) {
    result.fallback.attempted = true;
    result.fallback.attempt_count = 1;
    result.fallback.trigger_reason = "RECOVERY_ATTEMPT_INVALID";
    result.fallback.recovery_failure_reason = recovery_failure;
    const auto fallback_started = Clock::now();
    SegmentRefitResult fallback;
    const DevelopmentStage2Request* development_fallback_request =
        development_empty_fallback_request_
            ? development_empty_fallback_request_
            : development_empty_recovery_request_;
    if (development_fallback_request) {
      fallback = SegmentRefitter(refit_options_)
          .RunDevelopmentFrozenCandidatePolicy(
              frozen_raw_graph, stage2_refit.values, frozen_raw_uwb_metadata,
              plan, cfg, frozen_full_support, {},
              development_fallback_request);
    } else {
      fallback = SegmentRefitter(refit_options_).RunFrozenCandidatePolicy(
          frozen_raw_graph, stage2_refit.values, frozen_raw_uwb_metadata,
          plan, cfg, frozen_full_support, {});
    }
    result.timing.fallback_seconds = SecondsSince(fallback_started);
    if (test_hooks_.force_fallback_failure) {
      fallback.status = SegmentRefitStatus::CONDITIONAL_LM_FAILED;
      fallback.reason = "TEST_ONLY_FORCED_FALLBACK_FAILURE";
    }
    result.fallback_refit_attempt = AttemptDiagnostics(fallback);
    result.fallback.fallback_solver_status =
        SegmentRefitStatusName(fallback.status);
    if (!fallback.successful()) {
      result.fallback_refit_attempt.acceptance_audit_status =
          "FAILED_BEFORE_AUDIT";
      result.fallback_refit_attempt.acceptance_failure_reason =
          fallback.reason;
      SetInapplicableFinalScores(
          result.decisions, "ESTIMATION_FAILED_NO_FINAL_LINEARIZATION",
          &result.final_scores);
      result.fallback.status = "FAILED";
      result.fallback.fallback_failure_reason = fallback.reason;
      result.status = InferenceStatus::ESTIMATION_FAILED;
      result.reason = recovery_failure + "; FALLBACK_FAILED_" +
                      SegmentRefitStatusName(fallback.status) + ": " +
                      fallback.reason;
      SetFinalMaskUse(&result.frozen_masks, true, false);
      result.final_graph.resize(0);
      result.final_values.clear();
      result.final_result_refit.execution_status = "NOT_RUN";
      result.final_result_refit.stop_reason = "NO_VALID_FINAL_RESULT";
      finish();
      return result;
    }
    const auto fallback_audit = AuditFinalFactors(
        fallback, frozen_full_support, {}, plan);
    if (!fallback_audit.ok) {
      result.fallback_refit_attempt.acceptance_audit_status = "FAILED";
      result.fallback_refit_attempt.acceptance_failure_reason =
          fallback_audit.reason;
      SetInapplicableFinalScores(
          result.decisions, "ESTIMATION_FAILED_NO_FINAL_LINEARIZATION",
          &result.final_scores);
      result.fallback.status = "FAILED";
      result.fallback.fallback_failure_reason = fallback_audit.reason;
      result.status = InferenceStatus::ESTIMATION_FAILED;
      result.reason = recovery_failure +
                      "; FALLBACK_FACTOR_AUDIT_FAILED: " +
                      fallback_audit.reason;
      SetFinalMaskUse(&result.frozen_masks, true, false);
      result.final_result_refit.execution_status = "NOT_RUN";
      result.final_result_refit.stop_reason = "NO_VALID_FINAL_RESULT";
      finish();
      return result;
    }
    recovery = std::move(fallback);
    SetInapplicableFinalScores(
        result.decisions,
        "FALLBACK_ALL_CANDIDATES_SUPPRESSED_NO_FINAL_C_KEY_NO_READMISSION",
        &result.final_scores);
    result.factor_audit = fallback_audit;
    result.fallback_refit_attempt.acceptance_audit_status = "PASSED";
    result.fallback.status = "SUCCESS";
    result.status = InferenceStatus::FALLBACK_OK;
    result.reason = "ALL_CANDIDATES_SUPPRESSED_FALLBACK_OK";
    result.final_result_refit = result.fallback_refit_attempt;
    result.final_result_refit.execution_status = "FINAL_RESULT_FROM_FALLBACK";
    result.final_result_refit.acceptance_audit_status = "NOT_APPLICABLE";
    used_fallback = true;
    if (fixed_mode) result.actual_fixed_method = "suppress_all";
  } else if (frozen_full_support.segments.empty()) {
    result.status = InferenceStatus::NO_CANDIDATES;
    result.reason = "NO_CANDIDATES_FINAL_REFERENCE_GRAPH";
  } else if (eligible_groups == 0) {
    result.status = InferenceStatus::NO_ELIGIBLE_CANDIDATES;
    result.reason = "ALL_CANDIDATE_GROUPS_INELIGIBLE";
  } else if (accepted.empty()) {
    result.status = InferenceStatus::ZERO_ACCEPTED;
    result.reason = "ELIGIBLE_GROUPS_PRESENT_NONE_PASSED_GATE";
  } else {
    result.status = InferenceStatus::OK;
    result.reason = "FROZEN_GROUP_POLICY_FINAL_JOINT_INFERENCE_OK";
  }

  if (!used_fallback) {
    result.final_scores = result.recovery_final_scores;
    result.final_result_refit = result.recovery_attempt;
    result.final_result_refit.execution_status = "FINAL_RESULT_FROM_RECOVERY";
  }

  result.final_graph = std::move(recovery.graph);
  result.final_values = std::move(recovery.values);
  result.final_factor_metadata = std::move(recovery.factor_metadata);
  result.final_segments = std::move(recovery.segments);
  result.final_refit_iterations = recovery.iterations;
  SetFinalMaskUse(&result.frozen_masks, used_fallback, true);

  const auto covariance_started = Clock::now();
  if (test_hooks_.force_covariance_unavailable) {
    result.final_graph_covariance.status = CovarianceStatus::UNAVAILABLE;
    result.final_graph_covariance.reason =
        "TEST_ONLY_FORCED_COVARIANCE_UNAVAILABLE";
  } else {
    result.final_graph_covariance = ComputeFinalGraphCovariance(
        result.final_graph, result.final_values);
  }
  result.timing.covariance_seconds = SecondsSince(covariance_started);
  finish();
  return result;
}

}  // namespace uifgo

namespace uifgo {
std::vector<FixedCompensation> FreezeFixedCompensations(
    const SegmentRefitResult& stage2,
    const std::vector<GroupRecoverabilityScore>& scores, bool full_variant) {
  std::vector<FixedCompensation> output;
  for (const auto& segment : stage2.segments) {
    FixedCompensation c;
    c.segment_ordinal=segment.segment_ordinal; c.segment_id=segment.segment_id;
    c.c_hat_stage2_m=segment.amplitude_m;
    c.reason="SUPPRESS_LOCAL_SIGMA_UNAVAILABLE";
    size_t memberships=0;
    for (const auto& score:scores) {
      if (!OrdinalSet(score.group).count(segment.segment_ordinal)) continue;
      ++memberships;
      if (!score.eligible || !score.valid_score_exported || segment.boundary ||
          segment.short_support_debug || !std::isfinite(segment.amplitude_m) ||
          segment.amplitude_m<=0 || !stage2.values.exists(segment.amplitude_key) ||
          stage2.values.at<double>(segment.amplitude_key)!=segment.amplitude_m) continue;
      const auto sigma=LocalAmplitudeSigmas(score.numerical);
      if(sigma.empty()) continue;
      std::map<size_t,gtsam::Key> columns;
      std::set<gtsam::Key> keys;
      bool valid=true;
      for(const auto& col:score.key_columns) if(col.role=="GROUP_AMPLITUDE") {
        valid=valid && col.dimension==1 && keys.insert(col.key).second &&
              columns.emplace(col.offset,col.key).second &&
              gtsam::Symbol(col.key).chr()=='c' &&
              OrdinalSet(score.group).count(gtsam::Symbol(col.key).index());
      }
      if(!valid || columns.size()!=sigma.size() || keys.size()!=score.group.segment_ordinals.size()) continue;
      size_t index=0;
      for(const auto& col:columns) {
        if(col.second==segment.amplitude_key) {
          c.sigma_c_local_m=sigma[index]; c.sigma_available=true;
          const double partial=std::max(0.0,segment.amplitude_m-2.0*sigma[index]);
          c.use=partial>0;
          c.delta_c_fixed_m=c.use ? (full_variant ? segment.amplitude_m : partial) : 0.0;
          c.reason=c.use ? "USE_POSITIVE_LCB_FIXED_OFFSET" : "SUPPRESS_ZERO_LCB";
        }
        ++index;
      }
    }
    if(memberships!=1) { c.use=false; c.delta_c_fixed_m=0; c.reason="SUPPRESS_GROUP_MAPPING_INVALID"; }
    output.push_back(c);
  }
  return output;
}
}
