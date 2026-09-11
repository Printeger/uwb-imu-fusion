#include "uifgo/paper_methods.h"

#include <gtsam/linear/NoiseModel.h>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <unordered_map>

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
        gtsam::noiseModel::Robust::Create(estimator, base_noise)));
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
    const BaselineOptions& options) {
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

    if (options.method == PaperMethod::FIXED_REJECTION) {
      if (!FiniteNonnegative(options.rejection_threshold_sigma))
        throw std::invalid_argument("fixed rejection threshold is invalid");
      result.preliminary = RunCheckedConditionalLm(
          base_graph, common_initial_values, options.lm);
      if (!result.preliminary.converged) {
        result.reason = "PRELIMINARY_LM_FAILED:" + result.preliminary.reason;
        return result;
      }
      std::set<size_t> rejected_indices;
      for (const auto& item : uwb_by_index) {
        const auto noise_factor = boost::dynamic_pointer_cast<
            gtsam::NoiseModelFactor>(base_graph.at(item.first));
        if (!noise_factor)
          throw std::runtime_error("fixed rejection UWB factor type invalid");
        const gtsam::Vector residual =
            noise_factor->unwhitenedError(result.preliminary.values);
        const auto sigmas = noise_factor->noiseModel()->sigmas();
        if (residual.size() != 1 || sigmas.size() != 1 ||
            !(sigmas[0] > 0.0) || !residual.allFinite())
          throw std::runtime_error("fixed rejection residual/sigma invalid");
        const double q = std::abs(residual[0]) / sigmas[0];
        if (FixedRejectionKeeps(q, options.rejection_threshold_sigma))
          result.kept_uwb_obs_ids.push_back(item.second->obs_id);
        else {
          rejected_indices.insert(item.first);
          result.rejected_uwb_obs_ids.push_back(item.second->obs_id);
        }
      }
      result.final_factor_metadata = RetainedMetadata(
          base_graph, base_uwb_metadata, rejected_indices,
          &result.final_graph);
      result.final = RunCheckedConditionalLm(
          result.final_graph, result.preliminary.values, options.lm);
    } else {
      if ((options.method == PaperMethod::ROBUST_HUBER ||
           options.method == PaperMethod::ROBUST_CAUCHY) &&
          !(options.robust_scale > 0.0 &&
            std::isfinite(options.robust_scale)))
        throw std::invalid_argument("robust scale must be finite and positive");
      result.final_graph =
          options.method == PaperMethod::ALL_RANGE
              ? base_graph
              : RobustGraph(base_graph, uwb_by_index, options.method,
                            options.robust_scale);
      std::set<size_t> none;
      gtsam::NonlinearFactorGraph unused;
      result.final_factor_metadata = RetainedMetadata(
          result.final_graph, base_uwb_metadata, none, &unused);
      for (const auto& item : uwb_by_index)
        result.kept_uwb_obs_ids.push_back(item.second->obs_id);
      result.final = RunCheckedConditionalLm(
          result.final_graph, common_initial_values, options.lm);
    }
    if (!result.final.converged) {
      result.reason = "FINAL_LM_FAILED:" + result.final.reason;
      return result;
    }
    result.final_values = result.final.values;
    if (!GraphAndValuesKeysMatch(result.final_graph, result.final_values)) {
      result.reason = "FINAL_GRAPH_VALUES_KEY_MISMATCH";
      return result;
    }
    result.valid = true;
    result.status = "OK";
    result.reason = "BASELINE_FINAL_LM_AND_FACTOR_AUDIT_OK";
  } catch (const std::exception& error) {
    result.reason = error.what();
  }
  return result;
}

}  // namespace uifgo
