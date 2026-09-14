#include "uifgo/paper_methods.h"

#include "uifgo/nlos_fde.h"
#include "uifgo/paper_robust_noise.h"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <limits>

namespace uifgo {
namespace {

const std::vector<PaperMethodSpec> kRegistry = {
    {PaperMethod::LCB_PARTIAL, "lcb_partial", PaperExecutionType::FINAL_TRAJECTORY, false, true, true},
    {PaperMethod::LCB_FIXED_FULL, "lcb_fixed_full", PaperExecutionType::FINAL_TRAJECTORY, false, true, true},
    {PaperMethod::SUPPRESS_ALL, "suppress_all", PaperExecutionType::FINAL_TRAJECTORY, false, true, true},
    {PaperMethod::ALL_RANGE, "all_range",
     PaperExecutionType::BASELINE_TRAJECTORY, false, false, true},
    {PaperMethod::ROBUST_HUBER, "robust_huber",
     PaperExecutionType::BASELINE_TRAJECTORY, false, false, true},
    {PaperMethod::ROBUST_CAUCHY, "robust_cauchy",
     PaperExecutionType::BASELINE_TRAJECTORY, false, false, true},
    {PaperMethod::FIXED_REJECTION, "fixed_rejection",
     PaperExecutionType::BASELINE_TRAJECTORY, false, false, true},
    {PaperMethod::STRUCTURED_BIAS_ONLY, "structured_bias_only",
     PaperExecutionType::STAGE1_TRAJECTORY, true, false, true},
    {PaperMethod::STRUCTURED_DEBIAS, "structured_debias",
     PaperExecutionType::AUTOMATIC_STAGE2_TRAJECTORY, true, true, true},
    {PaperMethod::FIT_ONLY, "fit_only",
     PaperExecutionType::CACHE_DIAGNOSTIC, false, true, true},
    {PaperMethod::S_FIT, "s_fit", PaperExecutionType::CACHE_DIAGNOSTIC,
     false, true, true},
    {PaperMethod::FULL_GATE, "full_gate",
     PaperExecutionType::CACHE_DIAGNOSTIC, false, true, true},
    {PaperMethod::ETA_ONLY, "eta_only",
     PaperExecutionType::CACHE_DIAGNOSTIC, false, true, true},
    {PaperMethod::NOMINAL_CURVATURE, "nominal_curvature",
     PaperExecutionType::CACHE_DIAGNOSTIC, false, true, false},
    {PaperMethod::ORACLE_REFERENCE, "oracle_reference",
     PaperExecutionType::EVALUATION_REFERENCE, false, false, false},
};

bool FiniteNonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool GammaAvailable(const GroupRecoverabilityScore& score,
                    double* maximum) {
  if (score.segment_fit.empty()) return false;
  *maximum = 0.0;
  for (const auto& fit : score.segment_fit) {
    if (!std::isfinite(fit.gamma) || fit.gamma < 0.0) return false;
    *maximum = std::max(*maximum, fit.gamma);
  }
  return true;
}

bool EtaAvailable(const GroupRecoverabilityScore& score) {
  return score.numerical.valid_score() &&
         std::isfinite(score.numerical.eta);
}

bool SAvailable(const GroupRecoverabilityScore& score) {
  return score.numerical.valid_score() &&
         (score.numerical.s_is_infinite ||
          std::isfinite(score.numerical.s_m));
}

bool NominalAvailable(const GroupRecoverabilityScore& score,
                      double* lambda_min) {
  if (score.numerical.N.rows() == 0 ||
      score.numerical.N.rows() != score.numerical.N.cols() ||
      !score.numerical.N.allFinite())
    return false;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(score.numerical.N);
  if (eig.info() != Eigen::Success || eig.eigenvalues().size() == 0)
    return false;
  *lambda_min = eig.eigenvalues().minCoeff();
  return std::isfinite(*lambda_min);
}

std::unordered_map<size_t, const FactorMeta*> MetadataByIndex(
    const std::vector<FactorMeta>& metadata) {
  std::unordered_map<size_t, const FactorMeta*> output;
  for (const auto& meta : metadata) {
    if (meta.factor_type != "uwb_range") continue;
    if (!output.emplace(meta.factor_index, &meta).second)
      throw std::invalid_argument("duplicate UWB factor metadata index");
  }
  return output;
}

void ValidateBaselineInputs(const gtsam::NonlinearFactorGraph& graph,
                            const gtsam::Values& values,
                            const std::vector<FactorMeta>& metadata) {
  if (graph.empty() || values.empty() || !GraphAndValuesKeysMatch(graph, values))
    throw std::invalid_argument("baseline common graph/Values are invalid");
  std::set<std::uint64_t> obs_ids;
  for (const auto& meta : metadata) {
    if (meta.factor_type != "uwb_range") continue;
    if (meta.factor_index >= graph.size() || !graph.at(meta.factor_index))
      throw std::invalid_argument("baseline UWB metadata index is invalid");
    if (std::vector<gtsam::Key>(graph.at(meta.factor_index)->keys().begin(),
                               graph.at(meta.factor_index)->keys().end()) !=
        meta.keys)
      throw std::invalid_argument("baseline UWB metadata keys differ");
    if (!obs_ids.insert(meta.obs_id).second)
      throw std::invalid_argument("baseline UWB obs_id is duplicated");
  }
}

bool BaselineFactorIntegrity(
    const gtsam::NonlinearFactorGraph& graph,
    const std::vector<FactorMeta>& metadata, std::string* reason) {
  if (metadata.size() != graph.size()) {
    *reason = "BASELINE_FACTOR_METADATA_SIZE_MISMATCH";
    return false;
  }
  std::set<std::uint64_t> uwb_obs_ids;
  for (size_t index = 0; index < metadata.size(); ++index) {
    const auto& meta = metadata[index];
    const auto& factor = graph.at(index);
    if (!factor || meta.factor_index != index ||
        std::vector<gtsam::Key>(factor->keys().begin(),
                                factor->keys().end()) != meta.keys) {
      *reason = "BASELINE_FACTOR_INDEX_OR_KEYS_MISMATCH";
      return false;
    }
    if (meta.factor_type == "uwb_range" &&
        (!meta.obs_id || !uwb_obs_ids.insert(meta.obs_id).second)) {
      *reason = "BASELINE_UWB_OBS_ID_INVALID_OR_DUPLICATED";
      return false;
    }
  }
  *reason = "BASELINE_FACTOR_INDEX_KEYS_AND_OBS_ID_AUDIT_OK";
  return true;
}

gtsam::NonlinearFactorGraph RobustGraph(
    const gtsam::NonlinearFactorGraph& base,
    const std::unordered_map<size_t, const FactorMeta*>& uwb,
    PaperMethod method, double scale) {
  gtsam::NonlinearFactorGraph output;
  for (size_t i = 0; i < base.size(); ++i) {
    const auto& factor = base.at(i);
    if (!uwb.count(i)) {
      output.add(factor);
      continue;
    }
    const auto noise_factor =
        boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
    if (!noise_factor)
      throw std::invalid_argument("UWB factor is not a NoiseModelFactor");
    const auto base_noise = noise_factor->noiseModel();
    gtsam::noiseModel::mEstimator::Base::shared_ptr estimator;
    if (method == PaperMethod::ROBUST_HUBER)
      estimator = gtsam::noiseModel::mEstimator::Huber::Create(scale);
    else
      estimator = gtsam::noiseModel::mEstimator::Cauchy::Create(scale);
    output.add(noise_factor->cloneWithNewNoiseModel(
        boost::make_shared<PaperRobustNoise>(estimator, base_noise)));
  }
  return output;
}

std::vector<FactorMeta> RetainedMetadata(
    const gtsam::NonlinearFactorGraph& base,
    const std::vector<FactorMeta>& uwb_metadata,
    const std::set<size_t>& rejected,
    gtsam::NonlinearFactorGraph* output) {
  std::unordered_map<size_t, const FactorMeta*> uwb =
      MetadataByIndex(uwb_metadata);
  std::vector<FactorMeta> metadata;
  for (size_t i = 0; i < base.size(); ++i) {
    if (rejected.count(i)) continue;
    const size_t new_index = output->size();
    output->add(base.at(i));
    FactorMeta meta;
    meta.factor_index = new_index;
    const auto found = uwb.find(i);
    if (found != uwb.end()) {
      meta = *found->second;
      meta.factor_index = new_index;
    } else {
      meta.factor_type = "physical_non_uwb";
      meta.keys.assign(base.at(i)->keys().begin(), base.at(i)->keys().end());
    }
    metadata.push_back(std::move(meta));
  }
  return metadata;
}

}  // namespace

IntermediateSeedQualityAudit AuditIntermediateOptimizationSeed(
    const gtsam::NonlinearFactorGraph& huber_graph,
    const gtsam::Values& huber_input,
    const gtsam::Values& huber_terminal,
    const std::vector<FactorMeta>& base_uwb_metadata,
    const std::vector<double>& state_times_s,
    const IntermediateSeedQualityOptions& options) {
  IntermediateSeedQualityAudit audit;
  audit.evaluated = true;
  audit.position_envelope_m = options.position_envelope_m;
  audit.velocity_envelope_mps = options.velocity_envelope_mps;
  audit.state_count = state_times_s.size();
  try {
    audit.graph_keys_valid = GraphAndValuesKeysMatch(huber_graph, huber_input) &&
        GraphAndValuesKeysMatch(huber_graph, huber_terminal);
    audit.all_xvb_finite = audit.graph_keys_valid && !state_times_s.empty();
    if (audit.all_xvb_finite) {
      for (size_t k = 0; k < state_times_s.size(); ++k) {
        if (!std::isfinite(state_times_s[k]) ||
            (k && !(state_times_s[k] > state_times_s[k - 1])) ||
            !huber_terminal.exists(gtsam::Symbol('x', k)) ||
            !huber_terminal.exists(gtsam::Symbol('v', k)) ||
            !huber_terminal.exists(gtsam::Symbol('b', k))) {
          audit.all_xvb_finite = false;
          break;
        }
        const auto pose = huber_terminal.at<gtsam::Pose3>(
            gtsam::Symbol('x', k));
        const auto velocity = huber_terminal.at<gtsam::Vector3>(
            gtsam::Symbol('v', k));
        const auto bias =
            huber_terminal.at<gtsam::imuBias::ConstantBias>(
                gtsam::Symbol('b', k));
        const bool finite = pose.matrix().allFinite() && velocity.allFinite() &&
            bias.accelerometer().allFinite() && bias.gyroscope().allFinite();
        if (!finite) {
          audit.all_xvb_finite = false;
          break;
        }
        audit.max_position_norm_m =
            std::max(audit.max_position_norm_m, pose.translation().norm());
        audit.max_velocity_norm_mps =
            std::max(audit.max_velocity_norm_mps, velocity.norm());
      }
    }

    audit.input_objective = huber_graph.error(huber_input);
    audit.terminal_objective = huber_graph.error(huber_terminal);
    audit.objective_finite = std::isfinite(audit.input_objective) &&
        std::isfinite(audit.terminal_objective);
    audit.objective_roundoff_allowance =
        64.0 * std::numeric_limits<double>::epsilon() *
        std::max({1.0, std::abs(audit.input_objective),
                  std::abs(audit.terminal_objective)});
    audit.objective_not_worse = audit.objective_finite &&
        audit.terminal_objective <=
            audit.input_objective + audit.objective_roundoff_allowance;

    audit.uwb_factor_count = base_uwb_metadata.size();
    audit.uwb_residuals_finite = !base_uwb_metadata.empty();
    for (const auto& meta : base_uwb_metadata) {
      if (meta.factor_type != "uwb_range" ||
          meta.factor_index >= huber_graph.size() ||
          !huber_graph.at(meta.factor_index)) {
        audit.uwb_residuals_finite = false;
        break;
      }
      const auto factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(
          huber_graph.at(meta.factor_index));
      if (!factor) {
        audit.uwb_residuals_finite = false;
        break;
      }
      const auto residual = factor->unwhitenedError(huber_terminal);
      auto noise = factor->noiseModel();
      const auto robust =
          boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(noise);
      if (robust) noise = robust->noise();
      const auto sigmas = noise->sigmas();
      if (residual.size() != 1 || sigmas.size() != 1 ||
          !std::isfinite(residual[0]) || !(sigmas[0] > 0.0) ||
          !std::isfinite(sigmas[0])) {
        audit.uwb_residuals_finite = false;
        break;
      }
      const double q = std::abs(residual[0]) / sigmas[0];
      if (!std::isfinite(q)) {
        audit.uwb_residuals_finite = false;
        break;
      }
      audit.max_abs_uwb_residual_m = std::max(
          audit.max_abs_uwb_residual_m, std::abs(residual[0]));
      audit.max_abs_standardized_uwb_residual = std::max(
          audit.max_abs_standardized_uwb_residual, q);
    }
    audit.physically_plausible = audit.all_xvb_finite &&
        std::isfinite(options.position_envelope_m) &&
        std::isfinite(options.velocity_envelope_mps) &&
        options.position_envelope_m > 0.0 &&
        options.velocity_envelope_mps > 0.0 &&
        audit.max_position_norm_m <= options.position_envelope_m &&
        audit.max_velocity_norm_mps <= options.velocity_envelope_mps;
  } catch (...) {
    audit.graph_keys_valid = false;
  }

  audit.accepted = audit.graph_keys_valid && audit.all_xvb_finite &&
      audit.objective_finite && audit.objective_not_worse &&
      audit.uwb_residuals_finite && audit.physically_plausible;
  if (!audit.graph_keys_valid) audit.reason = "GRAPH_VALUES_KEY_MISMATCH";
  else if (!audit.all_xvb_finite) audit.reason = "NONFINITE_OR_INVALID_XVB";
  else if (!audit.objective_finite) audit.reason = "NONFINITE_HUBER_OBJECTIVE";
  else if (!audit.objective_not_worse)
    audit.reason = "HUBER_TERMINAL_OBJECTIVE_WORSENED";
  else if (!audit.uwb_residuals_finite)
    audit.reason = "NONFINITE_OR_MISSING_UWB_RESIDUALS";
  else if (!audit.physically_plausible)
    audit.reason = "CATASTROPHIC_INTERMEDIATE_STATE";
  else audit.reason = "INTERMEDIATE_OPTIMIZATION_SEED_ACCEPTED";
  return audit;
}

const std::vector<PaperMethodSpec>& CanonicalPaperMethodRegistry() {
  return kRegistry;
}

const PaperMethodSpec& PaperMethodByName(const std::string& name) {
  const auto found = std::find_if(kRegistry.begin(), kRegistry.end(),
      [&](const PaperMethodSpec& item) { return name == item.canonical_name; });
  if (found == kRegistry.end())
    throw std::invalid_argument("unknown canonical paper method: " + name);
  return *found;
}

const char* PaperExecutionTypeName(PaperExecutionType type) {
  switch (type) {
    case PaperExecutionType::BASELINE_TRAJECTORY: return "BASELINE_TRAJECTORY";
    case PaperExecutionType::STAGE1_TRAJECTORY: return "STAGE1_TRAJECTORY";
    case PaperExecutionType::AUTOMATIC_STAGE2_TRAJECTORY:
      return "AUTOMATIC_STAGE2_TRAJECTORY";
    case PaperExecutionType::CACHE_DIAGNOSTIC: return "CACHE_DIAGNOSTIC";
    case PaperExecutionType::FINAL_TRAJECTORY: return "FINAL_TRAJECTORY";
    case PaperExecutionType::EVALUATION_REFERENCE:
      return "EVALUATION_REFERENCE";
  }
  return "UNKNOWN";
}

bool FixedRejectionKeeps(double standardized_absolute_residual,
                         double rejection_threshold_sigma) {
  if (!FiniteNonnegative(standardized_absolute_residual) ||
      !FiniteNonnegative(rejection_threshold_sigma))
    throw std::invalid_argument("fixed rejection comparator input is invalid");
  return standardized_absolute_residual <= rejection_threshold_sigma;
}

std::vector<GroupDecisionRecord> FreezePaperPolicyDecisions(
    PaperMethod method,
    const std::vector<GroupRecoverabilityScore>& scores,
    const PolicyThresholds& thresholds) {
  if (method != PaperMethod::FIT_ONLY && method != PaperMethod::S_FIT &&
      method != PaperMethod::FULL_GATE && method != PaperMethod::ETA_ONLY &&
      method != PaperMethod::NOMINAL_CURVATURE)
    throw std::invalid_argument("method is not a cache decision policy");
  if (!FiniteNonnegative(thresholds.tau_eta) ||
      thresholds.tau_eta > 1.0 || !FiniteNonnegative(thresholds.tau_s_m) ||
      !FiniteNonnegative(thresholds.tau_gamma) ||
      !FiniteNonnegative(thresholds.tau_nominal_curvature_m2_inv) ||
      thresholds.parameter_provenance.empty())
    throw std::invalid_argument("policy thresholds/provenance are invalid");

  std::vector<GroupDecisionRecord> output;
  for (const auto& score : scores) {
    GroupDecisionRecord row;
    row.group = score.group;
    row.eligible = score.eligible;
    row.decision_linearization_id = score.linearization_id;
    row.decision = GroupDecision::SUPPRESS;
    if (!score.eligible) {
      row.reason_code = "SUPPRESS_INELIGIBLE_" + score.status;
      output.push_back(std::move(row));
      continue;
    }
    double gamma = 0.0;
    double nominal = 0.0;
    const bool gamma_available = GammaAvailable(score, &gamma);
    const bool eta_available = EtaAvailable(score);
    const bool s_available = SAvailable(score);
    const bool nominal_available = NominalAvailable(score, &nominal);
    row.max_gamma = gamma;
    row.max_gamma_available = gamma_available;
    row.gamma_pass = gamma_available && gamma <= thresholds.tau_gamma;
    row.eta_pass = eta_available && score.numerical.eta >= thresholds.tau_eta;
    row.s_pass = s_available && !score.numerical.s_is_infinite &&
                 score.numerical.s_m <= thresholds.tau_s_m;

    bool required_available = false;
    bool pass = false;
    if (method == PaperMethod::FIT_ONLY) {
      required_available = gamma_available;
      pass = row.gamma_pass;
    } else if (method == PaperMethod::S_FIT) {
      required_available = s_available && gamma_available;
      pass = row.s_pass && row.gamma_pass;
    } else if (method == PaperMethod::FULL_GATE) {
      required_available = eta_available && s_available && gamma_available;
      pass = row.eta_pass && row.s_pass && row.gamma_pass;
    } else if (method == PaperMethod::ETA_ONLY) {
      required_available = eta_available;
      pass = row.eta_pass;
    } else {
      required_available = nominal_available;
      pass = nominal_available &&
             nominal >= thresholds.tau_nominal_curvature_m2_inv;
    }
    if (!required_available) {
      row.reason_code = "SUPPRESS_REQUIRED_SCORE_UNAVAILABLE";
    } else if (!pass) {
      row.reason_code = "SUPPRESS_POLICY_PREDICATE_FAILED";
    } else {
      row.decision = GroupDecision::USE;
      row.reason_code = "USE_ALL_REQUIRED_PREDICATES_PASS";
    }
    output.push_back(std::move(row));
  }
  return output;
}

BaselineResult RunPaperBaseline(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& common_initial_values,
    const std::vector<FactorMeta>& base_uwb_metadata,
    const BaselineOptions& options, const RawGaussianReference* prepared) {
  BaselineResult result;
  try {
    ValidateBaselineInputs(base_graph, common_initial_values,
                           base_uwb_metadata);
    if (options.method != PaperMethod::ALL_RANGE &&
        options.method != PaperMethod::ROBUST_HUBER &&
        options.method != PaperMethod::ROBUST_CAUCHY &&
        options.method != PaperMethod::FIXED_REJECTION)
      throw std::invalid_argument("requested method is not a baseline");
    if (options.parameter_provenance.empty())
      throw std::invalid_argument("baseline parameter provenance is required");
    const auto uwb_by_index = MetadataByIndex(base_uwb_metadata);
    const bool robust = options.method == PaperMethod::ROBUST_HUBER ||
                        options.method == PaperMethod::ROBUST_CAUCHY;
    if (robust && !(options.robust_scale > 0.0 &&
                    std::isfinite(options.robust_scale)))
      throw std::invalid_argument("robust scale must be finite and positive");

    if (robust) {
      // A raw Gaussian warm start can itself be dominated by the gross UWB
      // residual this baseline is meant to withstand. Preserve that solve for
      // IE/fixed-rejection callers, but do not make it a prerequisite of the
      // robust navigation baseline.
      result.initialization_path = "COMMON_INITIAL_VALUES_DIRECT_ROBUST_V1";
      result.preliminary.reason = "NOT_RUN_ROBUST_DIRECT_INITIALIZATION";
      result.final_graph = RobustGraph(base_graph, uwb_by_index,
                                       options.method, options.robust_scale);
      std::set<size_t> none;
      gtsam::NonlinearFactorGraph unused;
      result.final_factor_metadata = RetainedMetadata(
          result.final_graph, base_uwb_metadata, none, &unused);
      for (const auto& item : uwb_by_index)
        result.kept_uwb_obs_ids.push_back(item.second->obs_id);
      CheckedLmDiagnosticRequest diagnostic;
      diagnostic.finite_difference_steps = {1e-4, 1e-5, 1e-6};
      const bool capture = std::getenv("UIFGO_BASELINE_DIAGNOSTIC") != nullptr;
      result.final = RunCheckedConditionalLm(
          result.final_graph, common_initial_values, options.lm,
          capture ? &diagnostic : nullptr);
      if (capture) {
        for (const auto& call : result.final.diagnostic.calls)
          std::cout << std::setprecision(17) << "BASELINE_CALL "
                    << call.call_index << ' ' << call.error_before << ' '
                    << call.error_after << ' ' << call.lambda_after << ' '
                    << call.accepted_values_delta_norm << '\n';
        for (const auto& f : result.final.diagnostic.factors)
          std::cout << "BASELINE_FACTOR " << f.factor_index << ' '
                    << f.error_at_capture_start << ' '
                    << f.error_at_capture_final << '\n';
      }
    } else {
      const auto reference = prepared ? *prepared : PrepareRawGaussianReference(
          base_graph, common_initial_values, options.lm);
      result.preliminary = reference.solve;
      if (!result.preliminary.converged) {
        result.reason = "PRELIMINARY_LM_FAILED:" + result.preliminary.reason;
        return result;
      }
      RequireRawGaussianReference(reference, base_graph,
                                  common_initial_values);
      result.initialization_path = "RAW_GAUSSIAN_REFERENCE_V1";
      if (options.method == PaperMethod::FIXED_REJECTION) {
        if (!FiniteNonnegative(options.rejection_threshold_sigma))
          throw std::invalid_argument("fixed rejection threshold is invalid");
        std::set<size_t> rejected_indices;
        const auto preliminary_residuals = ReadScalarUwbFactorResiduals(
            base_graph, result.preliminary.values, base_uwb_metadata);
        for (const auto& residual : preliminary_residuals) {
          const auto item = uwb_by_index.find(residual.factor_index);
          if (item == uwb_by_index.end())
            throw std::runtime_error(
                "fixed rejection residual metadata missing");
          const double q = std::abs(residual.residual_m) / residual.sigma_m;
          if (FixedRejectionKeeps(q, options.rejection_threshold_sigma))
            result.kept_uwb_obs_ids.push_back(item->second->obs_id);
          else {
            rejected_indices.insert(item->first);
            result.rejected_uwb_obs_ids.push_back(item->second->obs_id);
          }
        }
        result.final_factor_metadata = RetainedMetadata(
            base_graph, base_uwb_metadata, rejected_indices,
            &result.final_graph);
        result.final = RunCheckedConditionalLm(
            result.final_graph, result.preliminary.values, options.lm);
      } else {
        result.final_graph = base_graph;
        std::set<size_t> none;
        gtsam::NonlinearFactorGraph unused;
        result.final_factor_metadata = RetainedMetadata(
            result.final_graph, base_uwb_metadata, none, &unused);
        for (const auto& item : uwb_by_index)
          result.kept_uwb_obs_ids.push_back(item.second->obs_id);
        result.final = result.preliminary;
      }
    }
    if (!result.final.converged) {
      result.reason = "FINAL_LM_FAILED:" + result.final.reason;
      return result;
    }
    result.final_values = result.final.values;
    std::string factor_integrity_reason;
    const bool factor_integrity = BaselineFactorIntegrity(
        result.final_graph, result.final_factor_metadata,
        &factor_integrity_reason);
    SolverCertificateRequest certificate_request;
    certificate_request.termination_success = result.final.converged;
    certificate_request.termination_reason = result.final.reason;
    certificate_request.factor_integrity_passed = factor_integrity;
    certificate_request.factor_integrity_reason = factor_integrity_reason;
    certificate_request.solver_specific_checks_passed = true;
    certificate_request.solver_specific_checks_reason =
        "NOT_APPLICABLE_UNCONSTRAINED_BASELINE";
    certificate_request.state_times_s = options.keyframe_times_s;
    certificate_request.require_navigation_stationarity = true;
    certificate_request.navigation_scales = options.lm.navigation_scales;
    certificate_request.navigation_stationarity_tolerance_objective =
        options.lm.navigation_stationarity_tolerance_objective;
    certificate_request.gradient_roundoff_safety_factor =
        options.lm.gradient_roundoff_safety_factor;
    result.solver_certificate = CertifySolverResult(
        result.final_graph, result.final_values, certificate_request);
    if (!result.solver_certificate.certified_success()) {
      result.status = "CERTIFIED_FAILURE";
      result.reason = "FINAL_SOLVER_CERTIFICATE_FAILED:" +
                      result.solver_certificate.reason;
      result.final_values.clear();
      return result;
    }
    result.valid = true;
    result.status = "CERTIFIED_SUCCESS";
    result.reason = "BASELINE_FINAL_SOLVER_CERTIFIED_SUCCESS";
  } catch (const std::exception& error) {
    result.reason = error.what();
  }
  return result;
}

IntermediateOptimizationSeed RetainIntermediateOptimizationSeed(
    const BaselineResult& huber_result,
    const gtsam::Values& huber_input,
    const std::vector<FactorMeta>& base_uwb_metadata,
    const std::vector<double>& state_times_s,
    const IntermediateSeedQualityOptions& options) {
  IntermediateOptimizationSeed seed;
  if (huber_result.final_graph.empty() || huber_result.final.values.empty()) {
    seed.quality.evaluated = true;
    seed.quality.reason = "HUBER_TERMINAL_VALUES_UNAVAILABLE";
    return seed;
  }
  seed.available = true;
  seed.quality = AuditIntermediateOptimizationSeed(
      huber_result.final_graph, huber_input, huber_result.final.values,
      base_uwb_metadata, state_times_s, options);
  seed.eligible_for_cauchy_initialization = seed.quality.accepted;
  if (seed.quality.accepted) seed.values = huber_result.final.values;
  return seed;
}

HuberCauchyWarmStartResult RunHuberToCauchyWarmStart(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& qualified_initial_values,
    const std::vector<FactorMeta>& base_uwb_metadata,
    const BaselineOptions& huber_options,
    const BaselineOptions& cauchy_options,
    const IntermediateSeedQualityOptions& seed_quality_options) {
  if (huber_options.method != PaperMethod::ROBUST_HUBER)
    throw std::invalid_argument("warm-start first stage must be robust_huber");
  if (cauchy_options.method != PaperMethod::ROBUST_CAUCHY)
    throw std::invalid_argument("warm-start final stage must be robust_cauchy");
  if (huber_options.keyframe_times_s != cauchy_options.keyframe_times_s ||
      huber_options.lm.max_iterations != cauchy_options.lm.max_iterations ||
      huber_options.lm.relative_tolerance !=
          cauchy_options.lm.relative_tolerance ||
      huber_options.lm.absolute_tolerance !=
          cauchy_options.lm.absolute_tolerance ||
      huber_options.lm.policy != cauchy_options.lm.policy ||
      huber_options.lm.navigation_stationarity_tolerance_objective !=
          cauchy_options.lm.navigation_stationarity_tolerance_objective ||
      huber_options.lm.gradient_roundoff_safety_factor !=
          cauchy_options.lm.gradient_roundoff_safety_factor)
    throw std::invalid_argument("Huber/Cauchy LM and timeline contract differs");

  HuberCauchyWarmStartResult result;
  result.huber_intermediate = RunPaperBaseline(
      base_graph, qualified_initial_values, base_uwb_metadata, huber_options);
  result.intermediate_seed = RetainIntermediateOptimizationSeed(
      result.huber_intermediate, qualified_initial_values, base_uwb_metadata,
      huber_options.keyframe_times_s, seed_quality_options);
  if (!result.intermediate_seed.eligible_for_cauchy_initialization)
    return result;
  result.cauchy_attempted = true;
  result.cauchy_final = RunPaperBaseline(
      base_graph, result.intermediate_seed.values, base_uwb_metadata,
      cauchy_options);
  return result;
}

}  // namespace uifgo
