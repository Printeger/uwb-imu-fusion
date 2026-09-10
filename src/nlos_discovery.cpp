#include "uifgo/imu_preint.h"
#include "uifgo/nlos_discovery.h"

#include <Eigen/Cholesky>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>

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
#include <tuple>
#include <unordered_map>

#include "uifgo/hash_utils.h"
#include "uifgo/nlos_inference.h"
#include "uifgo/uwb_factor.h"

namespace uifgo {

const char kA19DevelopmentStage1Schema[] =
    "A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY";
const char kA19DevelopmentStage1Policy[] =
    "PAPER_CERTIFIED_PAIR_REDUCTION_V1";
const char kDevelopmentOnlyRole[] = "development";
const char kDevelopmentNonconsumableProvider[] =
    "development_nonconsumable";
const char kA19ValidationStage1Schema[] =
    "uifgo_t10_validation_stage1_v1";
const char kValidationRole[] = "validation";
const char kValidationNonconsumableProvider[] = "validation_nonconsumable";

DevelopmentStage1Request MakeA19DevelopmentStage1Request(
    const std::string& implementation_identity,
    DevelopmentStage1Navigation conditional_navigation) {
  DevelopmentStage1Request request;
  request.policy = kA19DevelopmentStage1Policy;
  request.implementation_identity = implementation_identity;
  request.role = kDevelopmentOnlyRole;
  request.output_schema = kA19DevelopmentStage1Schema;
  request.output_provider = kDevelopmentNonconsumableProvider;
  request.conditional_navigation = std::move(conditional_navigation);
  return request;
}

DevelopmentStage1ContractCheck ValidateDevelopmentStage1Request(
    const DevelopmentStage1Request& request,
    const DiscoveryContext& context, const DiscoveryOptions& options,
    bool diagnostic_request_present) {
  auto reject = [](const char* reason) {
    return DevelopmentStage1ContractCheck{false, reason};
  };
  if (diagnostic_request_present)
    return reject("DEVELOPMENT_AND_DIAGNOSTIC_REQUEST_CONFLICT");
  if (request.policy != kA19DevelopmentStage1Policy)
    return reject("DEVELOPMENT_STAGE1_POLICY_REJECTED");
  const bool validation = request.role == kValidationRole &&
                          request.output_schema == kA19ValidationStage1Schema;
  if (request.role != kDevelopmentOnlyRole && !validation)
    return reject("DEVELOPMENT_STAGE1_ROLE_REJECTED");
  if (request.implementation_identity.rfind("a18-policy-sha256:", 0) != 0 &&
      request.implementation_identity.rfind("a19-policy-sha256:", 0) != 0)
    return reject("DEVELOPMENT_STAGE1_IMPLEMENTATION_IDENTITY_REJECTED");
  if (request.output_schema != "A18_STAGE1_DIAGNOSTIC_ONLY" &&
      request.output_schema != "A19_STAGE1_STAGE2_SCORE_DIAGNOSTIC_ONLY" &&
      request.output_schema != kA19DevelopmentStage1Schema &&
      request.output_schema != kA19ValidationStage1Schema)
    return reject("DEVELOPMENT_STAGE1_SCHEMA_REJECTED");
  if (request.output_provider != (validation ? kValidationNonconsumableProvider
                                             : kDevelopmentNonconsumableProvider))
    return reject("DEVELOPMENT_STAGE1_PROVIDER_REJECTED");
  if (validation &&
      (request.output_schema != kA19ValidationStage1Schema ||
       request.validation_context_sha256.empty() ||
       request.validation_context_sha256 != context.validation_context_sha256))
    return reject("VALIDATION_STAGE1_CONTEXT_BINDING_REJECTED");
  if (!validation && (!request.validation_context_sha256.empty() ||
                      !context.validation_context_sha256.empty()))
    return reject("DEVELOPMENT_STAGE1_VALIDATION_CONTEXT_REJECTED");
  if (context.solver_config_hash != request.implementation_identity)
    return reject("DEVELOPMENT_STAGE1_CONTEXT_IDENTITY_REJECTED");
  if (!request.conditional_navigation)
    return reject("DEVELOPMENT_STAGE1_CALLBACK_REQUIRED");
  if (options.conditional_lm.policy !=
      ConditionalLmPolicy::
          GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2)
    return reject("DEVELOPMENT_STAGE1_CONDITIONAL_POLICY_REJECTED");
  return {true, "ACCEPTED"};
}
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

double SoftThreshold(double value, double threshold) {
  if (value > threshold) return value - threshold;
  if (value < -threshold) return value + threshold;
  return 0.0;
}

Eigen::VectorXd Difference(const Eigen::VectorXd& x) {
  if (x.size() <= 1) return Eigen::VectorXd(0);
  Eigen::VectorXd output(x.size() - 1);
  for (Eigen::Index i = 0; i + 1 < x.size(); ++i)
    output[i] = x[i + 1] - x[i];
  return output;
}

Eigen::VectorXd DifferenceTranspose(const Eigen::VectorXd& x,
                                    Eigen::Index n) {
  Eigen::VectorXd output = Eigen::VectorXd::Zero(n);
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    output[i] -= x[i];
    output[i + 1] += x[i];
  }
  return output;
}

double OriginalObjective(const Eigen::VectorXd& u, const Eigen::VectorXd& e,
                         const Eigen::VectorXd& w, double lambda_l1,
                         double lambda_tv) {
  const double data = 0.5 * ((u - e).array().square() * w.array()).sum();
  return data + lambda_l1 * u.sum() +
         lambda_tv * Difference(u).array().abs().sum();
}

double StableNorm(const Eigen::VectorXd& x) { return x.stableNorm(); }

std::string Sha256Id(const std::string& text) {
  return "sha256:" + Sha256Hex(text);
}

std::string A02NavigationParameterMismatch(
    const DiscoveryOptions& options) {
  if (options.conditional_lm.policy !=
          ConditionalLmPolicy::GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1 &&
      options.conditional_lm.policy != ConditionalLmPolicy::
          GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2)
    return "";
  if (options.navigation_stationarity_tolerance_objective !=
      options.conditional_lm.navigation_stationarity_tolerance_objective)
    return "navigation_stationarity_tolerance_objective";
  if (options.gradient_roundoff_safety_factor !=
      options.conditional_lm.gradient_roundoff_safety_factor)
    return "gradient_roundoff_safety_factor";
  if (options.navigation_scales.pose_rotation_rad !=
      options.conditional_lm.navigation_scales.pose_rotation_rad)
    return "navigation_scales.pose_rotation_rad";
  if (options.navigation_scales.pose_translation_m !=
      options.conditional_lm.navigation_scales.pose_translation_m)
    return "navigation_scales.pose_translation_m";
  if (options.navigation_scales.velocity_mps !=
      options.conditional_lm.navigation_scales.velocity_mps)
    return "navigation_scales.velocity_mps";
  if (options.navigation_scales.accel_bias_mps2 !=
      options.conditional_lm.navigation_scales.accel_bias_mps2)
    return "navigation_scales.accel_bias_mps2";
  if (options.navigation_scales.gyro_bias_radps !=
      options.conditional_lm.navigation_scales.gyro_bias_radps)
    return "navigation_scales.gyro_bias_radps";
  return "";
}

std::string A02NavigationParameterMismatchReason(
    const DiscoveryOptions& options) {
  const std::string field = A02NavigationParameterMismatch(options);
  return field.empty()
             ? ""
             : "A02 conditional and outer navigation parameters disagree: " +
                   field;
}

std::string DiscoveryOptionsCanonical(const DiscoveryOptions& options) {
  std::ostringstream out;
  out << std::setprecision(17)
      << "uifgo-t06-discovery-config-v2\n"
      << options.fused_lasso.lambda_l1 << '\n'
      << options.fused_lasso.lambda_tv << '\n'
      << options.fused_lasso.rho_scale << '\n'
      << options.fused_lasso.primal_absolute_tolerance_m << '\n'
      << options.fused_lasso.primal_relative_tolerance << '\n'
      << options.fused_lasso.dual_absolute_tolerance_objective_per_m << '\n'
      << options.fused_lasso.dual_relative_tolerance << '\n'
      << options.fused_lasso.kkt_tolerance_objective_per_m << '\n'
      << options.fused_lasso.tv_subgradient_tolerance_objective_per_m << '\n'
      << options.fused_lasso.max_iterations << '\n'
      << options.gap_threshold_s << '\n' << options.active_bias_min_m << '\n'
      << options.change_point_min_m << '\n'
      << options.merge_max_difference_m << '\n'
      << options.short_min_count << '\n' << options.short_min_duration_s << '\n'
      << options.relative_objective_tolerance << '\n'
      << options.scaled_step_tolerance << '\n'
      << options.observation_bias_scale_m << '\n'
      << options.navigation_stationarity_tolerance_objective << '\n'
      << options.gradient_roundoff_safety_factor << '\n'
      << options.navigation_scales.pose_rotation_rad << '\n'
      << options.navigation_scales.pose_translation_m << '\n'
      << options.navigation_scales.velocity_mps << '\n'
      << options.navigation_scales.accel_bias_mps2 << '\n'
      << options.navigation_scales.gyro_bias_radps << '\n'
      << options.max_outer_iterations << '\n'
      << options.conditional_lm.max_iterations << '\n'
      << options.conditional_lm.relative_tolerance << '\n'
      << options.conditional_lm.absolute_tolerance << '\n';
  if (options.conditional_lm.policy ==
      ConditionalLmPolicy::GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1) {
    out << "a02-conditional-navigation-policy-v1\n"
        << ConditionalLmPolicyName(options.conditional_lm.policy) << '\n';
  } else if (options.conditional_lm.policy == ConditionalLmPolicy::
                 GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2) {
    out << "a05-conditional-navigation-policy-v2\n"
        << ConditionalLmPolicyName(options.conditional_lm.policy) << '\n'
        << "optimizer_internal_relative_tolerance=0\n"
        << "external_relative_tolerance="
        << options.conditional_lm.relative_tolerance << '\n'
        << "external_absolute_tolerance="
        << options.conditional_lm.absolute_tolerance << '\n';
  }
  return out.str();
}

std::string ContextHash(const DiscoveryContext& context,
                        const DiscoveryOptions& options) {
  std::ostringstream canonical;
  canonical << "uifgo-t06-discovery-context-v2\n"
            << context.input_plan_hash << '\n' << context.source_hash << '\n'
            << context.config_hash << '\n' << context.calibration_hash << '\n'
            << context.solver_config_hash << '\n'
            << DiscoveryOptionsCanonical(options);
  return Sha256Id(canonical.str());
}

std::string SnapshotHash(const std::vector<DiscoveryObservation>& snapshot,
                         const std::string& context_hash) {
  std::ostringstream canonical;
  canonical << std::setprecision(17) << "uifgo-t06-discovery-snapshot-v2\n"
            << context_hash << '\n';
  for (const auto& item : snapshot)
    canonical << item.obs_id << ',' << item.tag_id << ',' << item.anchor_id
              << ',' << item.chain_id << ',' << item.active_run_id << ','
              << item.sensor_time << ',' << item.weight << ',' << item.bias_m
              << '\n';
  return Sha256Id(canonical.str());
}

std::string PartitionHash(const SupportPartition& partition) {
  std::ostringstream canonical;
  canonical << std::setprecision(17) << "uifgo-t06-partition-v2\n"
            << partition.discovery_context_hash << '\n'
            << partition.discovery_snapshot_hash << '\n';
  for (const auto& segment : partition.segments) {
    canonical << segment.segment_ordinal << ',' << segment.segment_id << ','
              << segment.tag_id << ',' << segment.anchor_id << ','
              << segment.start_time << ',' << segment.end_time << ','
              << segment.observation_count << ',' << segment.duration << ','
              << segment.merge_snapshot_mean_m << ','
              << segment.short_support_debug << '\n';
    for (const auto& parent : segment.parent_segment_ids)
      canonical << "p," << parent << '\n';
    for (auto obs_id : segment.obs_ids) canonical << "o," << obs_id << '\n';
  }
  return Sha256Id(canonical.str());
}

struct StableWeightedAccumulator {
  double scale = 0.0;
  double scaled_weight = 0.0;
  double scaled_weighted_value = 0.0;

  void AddScaled(double input_scale, double input_weight,
                 double input_weighted_value) {
    if (!(input_scale > 0.0) || !std::isfinite(input_scale) ||
        !(input_weight > 0.0) || !std::isfinite(input_weight) ||
        !std::isfinite(input_weighted_value))
      throw std::invalid_argument("A01 weighted accumulator input is invalid");
    const double new_scale = std::max(scale, input_scale);
    const double old_ratio = scale == 0.0 ? 0.0 : scale / new_scale;
    const double input_ratio = input_scale / new_scale;
    scaled_weight = scaled_weight * old_ratio + input_weight * input_ratio;
    scaled_weighted_value = scaled_weighted_value * old_ratio +
                            input_weighted_value * input_ratio;
    scale = new_scale;
    if (!(scaled_weight > 0.0) || !std::isfinite(scaled_weight) ||
        !std::isfinite(scaled_weighted_value))
      throw std::invalid_argument("A01 weighted accumulator is nonfinite");
  }

  void Add(double weight, double value) {
    if (!(weight > 0.0) || !std::isfinite(weight) ||
        !std::isfinite(value))
      throw std::invalid_argument("A01 weight/value is invalid");
    AddScaled(weight, 1.0, value);
  }

  void Merge(const StableWeightedAccumulator& other) {
    AddScaled(other.scale, other.scaled_weight,
              other.scaled_weighted_value);
  }

  double Mean() const {
    if (!(scale > 0.0) || !(scaled_weight > 0.0) ||
        !std::isfinite(scale) || !std::isfinite(scaled_weight) ||
        !std::isfinite(scaled_weighted_value))
      throw std::invalid_argument("A01 weighted denominator is invalid");
    const double result = scaled_weighted_value / scaled_weight;
    if (!std::isfinite(result))
      throw std::invalid_argument("A01 representative amplitude is nonfinite");
    return result;
  }
};

bool NavigationValuesFinite(const gtsam::Values& values,
                            size_t keyframe_count) {
  for (size_t k = 0; k < keyframe_count; ++k) {
    if (!values.exists(X(k)) || !values.exists(V(k)) || !values.exists(B(k)))
      return false;
    const auto pose = values.at<gtsam::Pose3>(X(k));
    const auto velocity = values.at<gtsam::Vector3>(V(k));
    const auto bias = values.at<gtsam::imuBias::ConstantBias>(B(k));
    if (!pose.matrix().allFinite() || !velocity.allFinite() ||
        !bias.accelerometer().allFinite() || !bias.gyroscope().allFinite())
      return false;
  }
  return true;
}

std::map<int, gtsam::Point3> AnchorMap(const Config& cfg) {
  std::map<int, gtsam::Point3> anchors;
  for (const auto& anchor : cfg.anchors) anchors.emplace(anchor.id, anchor.pos);
  return anchors;
}

}  // namespace

const char* FusedLassoStatusName(FusedLassoStatus status) {
  switch (status) {
    case FusedLassoStatus::CONVERGED: return "CONVERGED";
    case FusedLassoStatus::INVALID_INPUT: return "INVALID_INPUT";
    case FusedLassoStatus::FACTORIZATION_FAILED: return "FACTORIZATION_FAILED";
    case FusedLassoStatus::NONFINITE_VALUE: return "NONFINITE_VALUE";
    case FusedLassoStatus::MAX_ITERATIONS: return "MAX_ITERATIONS";
  }
  return "UNKNOWN";
}

FusedLassoResult SolveNonnegativeFusedLassoChain(
    const std::vector<double>& e_input,
    const std::vector<double>& weights_input,
    const FusedLassoOptions& options,
    const std::vector<double>* feasible_warm_start) {
  FusedLassoResult result;
  const size_t n_size = e_input.size();
  const double option_values[] = {
      options.lambda_l1,
      options.lambda_tv,
      options.rho_scale,
      options.primal_absolute_tolerance_m,
      options.primal_relative_tolerance,
      options.dual_absolute_tolerance_objective_per_m,
      options.dual_relative_tolerance,
      options.kkt_tolerance_objective_per_m,
      options.tv_subgradient_tolerance_objective_per_m,
      options.active_boundary_epsilon_m};
  if (n_size == 0 || weights_input.size() != n_size ||
      options.max_iterations == 0) {
    result.reason = "empty/mismatched chain or zero iteration limit";
    return result;
  }
  for (double value : option_values) {
    if (!std::isfinite(value) || value < 0.0) {
      result.reason = "fused-lasso options must be finite and nonnegative";
      return result;
    }
  }
  if (!(options.rho_scale > 0.0)) {
    result.reason = "rho_scale must be positive";
    return result;
  }
  if (feasible_warm_start && feasible_warm_start->size() != n_size) {
    result.reason = "warm start has wrong size";
    return result;
  }

  const Eigen::Index n = static_cast<Eigen::Index>(n_size);
  Eigen::VectorXd e(n), w(n);
  std::vector<double> sorted_weights;
  sorted_weights.reserve(n_size);
  for (Eigen::Index i = 0; i < n; ++i) {
    e[i] = e_input[static_cast<size_t>(i)];
    w[i] = weights_input[static_cast<size_t>(i)];
    if (!std::isfinite(e[i]) || !(w[i] > 0.0) || !std::isfinite(w[i])) {
      result.reason = "chain e/weights are nonfinite or nonpositive";
      return result;
    }
    sorted_weights.push_back(w[i]);
  }
  std::sort(sorted_weights.begin(), sorted_weights.end());
  const double median_weight =
      sorted_weights[(sorted_weights.size() - 1) / 2];
  const double rho = options.rho_scale * median_weight;
  if (!(rho > 0.0) || !std::isfinite(rho)) {
    result.reason = "rho is nonfinite/nonpositive";
    return result;
  }
  result.rho_objective_per_m2 = rho;

  Eigen::MatrixXd normal = w.asDiagonal();
  normal.diagonal().array() += rho;
  if (n > 1) {
    normal(0, 0) += rho;
    normal(n - 1, n - 1) += rho;
    for (Eigen::Index i = 1; i + 1 < n; ++i) normal(i, i) += 2.0 * rho;
    for (Eigen::Index i = 0; i + 1 < n; ++i) {
      normal(i, i + 1) -= rho;
      normal(i + 1, i) -= rho;
    }
  }
  Eigen::LDLT<Eigen::MatrixXd> factor(normal);
  if (factor.info() != Eigen::Success) {
    result.status = FusedLassoStatus::FACTORIZATION_FAILED;
    result.reason = "ADMM normal matrix factorization failed";
    return result;
  }

  Eigen::VectorXd u = Eigen::VectorXd::Zero(n);
  if (feasible_warm_start) {
    for (Eigen::Index i = 0; i < n; ++i) {
      const double value = (*feasible_warm_start)[static_cast<size_t>(i)];
      if (!std::isfinite(value) || value < 0.0) {
        result.reason = "warm start must be finite and nonnegative";
        return result;
      }
      u[i] = value;
    }
  }
  Eigen::VectorXd b = u;
  Eigen::VectorXd v = Difference(u);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(n);      // scaled dual b-u
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n - 1);  // scaled dual Db-v

  auto finalize_audit = [&](const Eigen::VectorXd& previous_u,
                            const Eigen::VectorXd& previous_v,
                            size_t iteration) -> bool {
    const Eigen::VectorXd db = Difference(b);
    Eigen::VectorXd primal_parts(n + n - 1);
    primal_parts.head(n) = b - u;
    if (n > 1) primal_parts.tail(n - 1) = db - v;
    const double primal = StableNorm(primal_parts);
    Eigen::VectorXd ax(n + n - 1), bz(n + n - 1);
    ax.head(n) = b;
    bz.head(n) = u;
    if (n > 1) {
      ax.tail(n - 1) = db;
      bz.tail(n - 1) = v;
    }
    const double primal_threshold =
        std::sqrt(static_cast<double>(2 * n - 1)) *
            options.primal_absolute_tolerance_m +
        options.primal_relative_tolerance *
            std::max(StableNorm(ax), StableNorm(bz));

    const Eigen::VectorXd dual_vector =
        rho * ((u - previous_u) +
               DifferenceTranspose(v - previous_v, n));
    const double dual = StableNorm(dual_vector);
    const Eigen::VectorXd unscaled_combined_dual =
        rho * (y + DifferenceTranspose(q, n));
    const double dual_threshold =
        std::sqrt(static_cast<double>(n)) *
            options.dual_absolute_tolerance_objective_per_m +
        options.dual_relative_tolerance *
            StableNorm(unscaled_combined_dual);

    // q is scaled. p=rho*q is the physical TV dual and is audited against
    // the subgradient of the final declared Du, not the auxiliary v.
    const Eigen::VectorXd p = rho * q;
    const Eigen::VectorXd du = Difference(u);
    double tv_violation = 0.0;
    for (Eigen::Index j = 0; j < p.size(); ++j) {
      double violation = std::max(0.0, std::abs(p[j]) - options.lambda_tv);
      if (du[j] > options.active_boundary_epsilon_m)
        violation = std::max(violation, std::abs(p[j] - options.lambda_tv));
      else if (du[j] < -options.active_boundary_epsilon_m)
        violation = std::max(violation, std::abs(p[j] + options.lambda_tv));
      tv_violation = std::max(tv_violation, violation);
    }
    const Eigen::VectorXd g =
        (w.array() * (u - e).array()).matrix() +
        Eigen::VectorXd::Constant(n, options.lambda_l1) +
        DifferenceTranspose(p, n);
    double kkt = 0.0;
    for (Eigen::Index i = 0; i < n; ++i) {
      const double violation =
          u[i] > options.active_boundary_epsilon_m
              ? std::abs(g[i])
              : std::max(0.0, -g[i]);
      kkt = std::max(kkt, violation);
    }
    const double objective =
        OriginalObjective(u, e, w, options.lambda_l1, options.lambda_tv);
    FusedLassoTrace trace;
    trace.iteration = iteration;
    trace.objective = objective;
    trace.primal_residual_m = primal;
    trace.primal_threshold_m = primal_threshold;
    trace.dual_residual_objective_per_m = dual;
    trace.dual_threshold_objective_per_m = dual_threshold;
    trace.max_kkt_violation_objective_per_m = kkt;
    trace.max_tv_subgradient_violation_objective_per_m = tv_violation;
    result.trace.push_back(trace);
    result.iterations = iteration;
    result.objective = objective;
    result.primal_residual_m = primal;
    result.primal_threshold_m = primal_threshold;
    result.dual_residual_objective_per_m = dual;
    result.dual_threshold_objective_per_m = dual_threshold;
    result.max_kkt_violation_objective_per_m = kkt;
    result.max_tv_subgradient_violation_objective_per_m = tv_violation;
    if (!std::isfinite(objective) || !std::isfinite(primal) ||
        !std::isfinite(dual) || !std::isfinite(kkt) ||
        !std::isfinite(tv_violation)) {
      result.status = FusedLassoStatus::NONFINITE_VALUE;
      result.reason = "ADMM audit produced a nonfinite value";
      return true;
    }
    if (primal <= primal_threshold && dual <= dual_threshold &&
        kkt <= options.kkt_tolerance_objective_per_m &&
        tv_violation <= options.tv_subgradient_tolerance_objective_per_m) {
      result.status = FusedLassoStatus::CONVERGED;
      result.reason = "PRIMAL_DUAL_KKT_AND_TV_SUBGRADIENT_CONVERGED";
      result.u.assign(u.data(), u.data() + u.size());
      result.b.assign(b.data(), b.data() + b.size());
      result.v.assign(v.data(), v.data() + v.size());
      result.p.assign(p.data(), p.data() + p.size());
      return true;
    }
    return false;
  };

  for (size_t iteration = 1; iteration <= options.max_iterations;
       ++iteration) {
    const Eigen::VectorXd previous_u = u;
    const Eigen::VectorXd previous_v = v;
    const Eigen::VectorXd rhs =
        (w.array() * e.array()).matrix() + rho * (u - y) +
        rho * DifferenceTranspose(v - q, n);
    b = factor.solve(rhs);
    if (factor.info() != Eigen::Success || !b.allFinite()) {
      result.status = FusedLassoStatus::FACTORIZATION_FAILED;
      result.reason = "ADMM normal solve failed";
      return result;
    }
    for (Eigen::Index i = 0; i < n; ++i)
      u[i] = std::max(0.0, b[i] + y[i] - options.lambda_l1 / rho);
    const Eigen::VectorXd db = Difference(b);
    for (Eigen::Index j = 0; j < v.size(); ++j)
      v[j] = SoftThreshold(db[j] + q[j], options.lambda_tv / rho);
    y += b - u;
    q += db - v;
    if (finalize_audit(previous_u, previous_v, iteration)) return result;
  }
  result.status = FusedLassoStatus::MAX_ITERATIONS;
  result.reason = "ADMM_MAX_ITERATIONS_BEFORE_DECLARED_TOLERANCES";
  result.u.assign(u.data(), u.data() + u.size());
  result.b.assign(b.data(), b.data() + b.size());
  result.v.assign(v.data(), v.data() + v.size());
  const Eigen::VectorXd p = rho * q;
  result.p.assign(p.data(), p.data() + p.size());
  return result;
}

SupportPartition BuildAutomaticSupportPartition(
    std::vector<DiscoveryObservation> snapshot,
    const DiscoveryOptions& options, const DiscoveryContext& context) {
  const std::string a02_mismatch =
      A02NavigationParameterMismatchReason(options);
  if (!a02_mismatch.empty()) throw std::invalid_argument(a02_mismatch);
  const double thresholds[] = {
      options.active_bias_min_m, options.change_point_min_m,
      options.merge_max_difference_m, options.short_min_duration_s};
  for (double threshold : thresholds)
    if (!std::isfinite(threshold) || threshold < 0.0)
      throw std::invalid_argument("partition thresholds must be nonnegative");
  std::sort(snapshot.begin(), snapshot.end(), [](const auto& a, const auto& b) {
    return std::tie(a.tag_id, a.anchor_id, a.sensor_time, a.obs_id) <
           std::tie(b.tag_id, b.anchor_id, b.sensor_time, b.obs_id);
  });
  std::set<std::uint64_t> all_obs;
  for (const auto& item : snapshot) {
    if (item.obs_id == 0 || !all_obs.insert(item.obs_id).second ||
        !std::isfinite(item.sensor_time) || !(item.weight > 0.0) ||
        !std::isfinite(item.weight) || !std::isfinite(item.bias_m) ||
        item.bias_m < 0.0)
      throw std::invalid_argument("discovery snapshot is invalid");
  }

  struct WorkingSegment {
    SupportSegment support;
    size_t chain_id = 0;
    size_t active_run_id = 0;
    StableWeightedAccumulator weighted;
  };
  std::vector<WorkingSegment> originals;
  WorkingSegment current;
  bool open = false;
  auto close = [&]() {
    if (!open) return;
    current.support.observation_count = current.support.obs_ids.size();
    current.support.duration =
        current.support.end_time - current.support.start_time;
    current.support.merge_snapshot_mean_m = current.weighted.Mean();
    std::ostringstream parent_key;
    parent_key << "uifgo-t06-a01-parent-v1\n" << current.support.tag_id << ','
               << current.support.anchor_id << ','
               << current.support.obs_ids.front() << ','
               << current.support.obs_ids.back();
    current.support.parent_segment_ids = {
        "auto-parent-" + Sha256Id(parent_key.str())};
    originals.push_back(std::move(current));
    current = WorkingSegment();
    open = false;
  };
  const DiscoveryObservation* previous = nullptr;
  for (const auto& item : snapshot) {
    const bool active = item.bias_m >= options.active_bias_min_m;
    if (!active) {
      close();
      previous = &item;
      continue;
    }
    const bool same_active_run =
        previous && previous->tag_id == item.tag_id &&
        previous->anchor_id == item.anchor_id &&
        previous->chain_id == item.chain_id &&
        previous->active_run_id == item.active_run_id &&
        previous->bias_m >= options.active_bias_min_m;
    const bool change = same_active_run &&
                        std::abs(item.bias_m - previous->bias_m) >=
                            options.change_point_min_m;
    if (!open || !same_active_run || change) {
      close();
      open = true;
      current.chain_id = item.chain_id;
      current.active_run_id = item.active_run_id;
      current.support.tag_id = item.tag_id;
      current.support.anchor_id = item.anchor_id;
      current.support.start_time = item.sensor_time;
    }
    current.support.end_time = item.sensor_time;
    current.support.obs_ids.push_back(item.obs_id);
    current.weighted.Add(item.weight, item.bias_m);
    previous = &item;
  }
  close();

  // A01: deterministic left-to-right single pass. The accumulator
  // representative is recomputed after a merge; next remains the immutable
  // original segment. Equality merges. Gap/inactive runs can never merge.
  std::vector<WorkingSegment> merged;
  for (const auto& next : originals) {
    if (merged.empty()) {
      merged.push_back(next);
      continue;
    }
    WorkingSegment& accumulator = merged.back();
    const double accumulator_mean = accumulator.weighted.Mean();
    const double next_mean = next.weighted.Mean();
    if (accumulator.chain_id == next.chain_id &&
        accumulator.active_run_id == next.active_run_id &&
        accumulator.support.tag_id == next.support.tag_id &&
        accumulator.support.anchor_id == next.support.anchor_id &&
        std::abs(accumulator_mean - next_mean) <=
            options.merge_max_difference_m) {
      accumulator.support.end_time = next.support.end_time;
      accumulator.support.obs_ids.insert(accumulator.support.obs_ids.end(),
                                         next.support.obs_ids.begin(),
                                         next.support.obs_ids.end());
      accumulator.support.parent_segment_ids.insert(
          accumulator.support.parent_segment_ids.end(),
          next.support.parent_segment_ids.begin(),
          next.support.parent_segment_ids.end());
      accumulator.weighted.Merge(next.weighted);
      accumulator.support.observation_count =
          accumulator.support.obs_ids.size();
      accumulator.support.duration = accumulator.support.end_time -
                                     accumulator.support.start_time;
      accumulator.support.merge_snapshot_mean_m =
          accumulator.weighted.Mean();
    } else {
      merged.push_back(next);
    }
  }

  SupportPartition partition;
  partition.schema = "t06_automatic_support_v2";
  partition.provider = "automatic_discovery";
  partition.hash_algorithm = "SHA-256";
  partition.partition_rule_version = "T06_A01_SINGLE_PASS_V2";
  partition.input_plan_hash = context.input_plan_hash;
  partition.source_hash = context.source_hash;
  partition.config_hash = context.config_hash;
  partition.calibration_hash = context.calibration_hash;
  partition.solver_config_hash = context.solver_config_hash.empty()
                                     ? Sha256Id(DiscoveryOptionsCanonical(options))
                                     : context.solver_config_hash;
  DiscoveryContext effective_context = context;
  effective_context.solver_config_hash = partition.solver_config_hash;
  partition.discovery_context_hash =
      ContextHash(effective_context, options);
  partition.discovery_snapshot_hash =
      SnapshotHash(snapshot, partition.discovery_context_hash);
  std::set<std::uint64_t> assigned;
  for (size_t ordinal = 0; ordinal < merged.size(); ++ordinal) {
    SupportSegment segment = merged[ordinal].support;
    segment.segment_ordinal = ordinal;
    std::ostringstream id_key;
    id_key << "uifgo-t06-a01-segment-v1\n"
           << partition.discovery_snapshot_hash << '\n' << segment.tag_id
           << ',' << segment.anchor_id;
    for (const auto& parent : segment.parent_segment_ids)
      id_key << '\n' << parent;
    segment.segment_id = "auto-segment-" + Sha256Id(id_key.str());
    segment.observation_count = segment.obs_ids.size();
    segment.duration = segment.end_time - segment.start_time;
    segment.short_support_debug =
        segment.observation_count < options.short_min_count ||
        segment.duration < options.short_min_duration_s;
    for (auto obs_id : segment.obs_ids)
      if (!assigned.insert(obs_id).second)
        throw std::invalid_argument("partition assigns obs_id more than once");
    partition.segments.push_back(std::move(segment));
  }
  partition.partition_hash = PartitionHash(partition);
  return partition;
}

const char* DiscoveryStatusName(DiscoveryStatus status) {
  switch (status) {
    case DiscoveryStatus::CONVERGED: return "CONVERGED";
    case DiscoveryStatus::INVALID_INPUT: return "INVALID_INPUT";
    case DiscoveryStatus::CONDITIONAL_LM_FAILED: return "CONDITIONAL_LM_FAILED";
    case DiscoveryStatus::CHAIN_SOLVE_FAILED: return "CHAIN_SOLVE_FAILED";
    case DiscoveryStatus::NONFINITE_VALUE: return "NONFINITE_VALUE";
    case DiscoveryStatus::OBJECTIVE_INCREASE: return "OBJECTIVE_INCREASE";
    case DiscoveryStatus::NAVIGATION_NOT_STATIONARY:
      return "NAVIGATION_NOT_STATIONARY";
    case DiscoveryStatus::PARTITION_INVALID: return "PARTITION_INVALID";
    case DiscoveryStatus::MAX_OUTER_ITERATIONS: return "MAX_OUTER_ITERATIONS";
  }
  return "UNKNOWN";
}

void AssignDiscoveryChains(std::vector<DiscoveryObservation>* observations,
                           double gap_threshold_s) {
  if (!observations || !std::isfinite(gap_threshold_s) ||
      gap_threshold_s < 0.0)
    throw std::invalid_argument("gap threshold/input is invalid");
  std::sort(observations->begin(), observations->end(),
            [](const auto& a, const auto& b) {
              return std::tie(a.tag_id, a.anchor_id, a.sensor_time, a.obs_id) <
                     std::tie(b.tag_id, b.anchor_id, b.sensor_time, b.obs_id);
            });
  size_t chain_id = 0;
  const DiscoveryObservation* previous = nullptr;
  for (auto& item : *observations) {
    if (!previous || previous->tag_id != item.tag_id ||
        previous->anchor_id != item.anchor_id ||
        item.sensor_time - previous->sensor_time > gap_threshold_s)
      ++chain_id;
    item.chain_id = chain_id;
    previous = &item;
  }
}

DiscoveryResult AutomaticSupportProvider::Run(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& base_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const DiscoveryContext& context) const {
  return Run(base_graph, base_values, base_uwb_factor_metadata, plan, cfg,
             context, nullptr);
}

DiscoveryResult AutomaticSupportProvider::Run(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& base_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const DiscoveryContext& context,
    const DiscoveryDiagnosticRequest* diagnostic_request) const {
  return RunDevelopmentStage1(base_graph, base_values, base_uwb_factor_metadata,
      plan, cfg, context, diagnostic_request, nullptr);
}

DiscoveryResult AutomaticSupportProvider::RunDevelopmentStage1(
    const gtsam::NonlinearFactorGraph& base_graph, const gtsam::Values& base_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg, const DiscoveryContext& context,
    const DiscoveryDiagnosticRequest* diagnostic_request,
    const DevelopmentStage1Request* development_request) const {
  DiscoveryResult result;
  if (development_request) {
    const auto contract = ValidateDevelopmentStage1Request(
        *development_request, context, options_, diagnostic_request != nullptr);
    if (!contract.accepted) {
      result.status = DiscoveryStatus::INVALID_INPUT;
      result.reason = contract.reason;
      return result;
    }
  }
  std::string imu_model_reason;
  if (!PaperImuCovarianceModelMatchesGraph(base_graph, cfg, &imu_model_reason)) {
    result.status = DiscoveryStatus::INVALID_INPUT;
    result.reason = imu_model_reason;
    return result;
  }
  auto fail = [&](DiscoveryStatus status, const std::string& reason) {
    result.status = status;
    result.reason = reason;
    if (result.partition.hash_algorithm.empty()) {
      result.partition.schema = "t06_automatic_support_v2";
      result.partition.provider = "automatic_discovery";
      result.partition.hash_algorithm = "SHA-256";
      result.partition.partition_rule_version = "T06_A01_SINGLE_PASS_V2";
      result.partition.input_plan_hash = context.input_plan_hash;
      result.partition.source_hash = context.source_hash;
      result.partition.config_hash = context.config_hash;
      result.partition.calibration_hash = context.calibration_hash;
      result.partition.solver_config_hash =
          context.solver_config_hash.empty()
              ? Sha256Id(DiscoveryOptionsCanonical(options_))
              : context.solver_config_hash;
      DiscoveryContext effective = context;
      effective.solver_config_hash = result.partition.solver_config_hash;
      result.partition.discovery_context_hash =
          ContextHash(effective, options_);
      if (!result.snapshot.empty())
        result.partition.discovery_snapshot_hash = SnapshotHash(
            result.snapshot, result.partition.discovery_context_hash);
    }
    if (development_request) {
      result.partition.schema = development_request->output_schema;
      result.partition.provider = development_request->output_provider;
    }
    return result;
  };
  const std::string a02_mismatch =
      A02NavigationParameterMismatchReason(options_);
  if (!a02_mismatch.empty())
    return fail(DiscoveryStatus::INVALID_INPUT, a02_mismatch);
  const double option_values[] = {
      options_.gap_threshold_s,
      options_.active_bias_min_m,
      options_.change_point_min_m,
      options_.merge_max_difference_m,
      options_.short_min_duration_s,
      options_.relative_objective_tolerance,
      options_.scaled_step_tolerance,
      options_.observation_bias_scale_m,
      options_.navigation_stationarity_tolerance_objective,
      options_.gradient_roundoff_safety_factor};
  for (double value : option_values)
    if (!std::isfinite(value) || value < 0.0)
      return fail(DiscoveryStatus::INVALID_INPUT,
                  "automatic discovery options are invalid");
  if (options_.max_outer_iterations == 0 ||
      !(options_.observation_bias_scale_m > 0.0) || base_graph.empty() ||
      cfg.calib_lever || cfg.calib_anchor || cfg.calib_range_bias ||
      cfg.calib_td || !GraphAndValuesKeysMatch(base_graph, base_values))
    return fail(DiscoveryStatus::INVALID_INPUT,
                "automatic discovery requires fixed calibration and matching graph/Values");

  std::unordered_map<std::uint64_t, const ObservationRecord*> records;
  for (const auto& record : plan.observations)
    if (!records.emplace(record.obs_id, &record).second)
      return fail(DiscoveryStatus::INVALID_INPUT,
                  "plan contains duplicate obs_id");
  std::unordered_map<size_t, std::uint64_t> obs_by_factor;
  std::unordered_map<std::uint64_t, size_t> factor_by_obs;
  for (const auto& meta : base_uwb_factor_metadata) {
    if (meta.factor_index >= base_graph.size() ||
        meta.factor_type != "uwb_range" || meta.obs_id == 0 ||
        !obs_by_factor.emplace(meta.factor_index, meta.obs_id).second ||
        !factor_by_obs.emplace(meta.obs_id, meta.factor_index).second)
      return fail(DiscoveryStatus::INVALID_INPUT,
                  "base UWB factor metadata is invalid");
  }

  std::vector<const ObservationRecord*> ordered;
  for (const auto& record : plan.observations) {
    if (record.valid && record.planned) {
      if (!factor_by_obs.count(record.obs_id) ||
          !(record.nominal_sigma > 0.0) ||
          !std::isfinite(record.nominal_sigma))
        return fail(DiscoveryStatus::INVALID_INPUT,
                    "planned observation/factor coverage is invalid");
      ordered.push_back(&record);
    }
  }
  std::sort(ordered.begin(), ordered.end(), [](const auto* a, const auto* b) {
    return std::tie(a->tag_id, a->anchor_id, a->sensor_time, a->obs_id) <
           std::tie(b->tag_id, b->anchor_id, b->sensor_time, b->obs_id);
  });
  if (ordered.empty())
    return fail(DiscoveryStatus::INVALID_INPUT,
                "automatic discovery has no planned observations");

  result.snapshot.reserve(ordered.size());
  for (const auto* record : ordered) {
    DiscoveryObservation item;
    item.obs_id = record->obs_id;
    item.tag_id = record->tag_id;
    item.anchor_id = record->anchor_id;
    item.sensor_time = record->sensor_time;
    item.weight = 1.0 / (record->nominal_sigma * record->nominal_sigma);
    result.snapshot.push_back(item);
  }
  AssignDiscoveryChains(&result.snapshot, options_.gap_threshold_s);
  std::unordered_map<std::uint64_t, size_t> snapshot_index;
  for (size_t i = 0; i < result.snapshot.size(); ++i)
    snapshot_index.emplace(result.snapshot[i].obs_id, i);
  const auto anchors = AnchorMap(cfg);
  for (const auto* record : ordered)
    if (!anchors.count(record->anchor_id))
      return fail(DiscoveryStatus::INVALID_INPUT,
                  "planned observation has no fixed anchor");

  std::vector<DevelopmentRangeConstant> development_ranges;
  auto fixed_bias_graph = [&](const std::vector<DiscoveryObservation>& snapshot) {
    development_ranges.clear();
    gtsam::NonlinearFactorGraph graph;
    graph.reserve(base_graph.size());
    for (size_t index = 0; index < base_graph.size(); ++index) {
      const auto found = obs_by_factor.find(index);
      if (found == obs_by_factor.end()) {
        graph.add(base_graph.at(index));
        continue;
      }
      const auto& record = *records.at(found->second);
      const double bias = snapshot.at(snapshot_index.at(found->second)).bias_m;
      const double conditional_beta =
          FixedBetaForLink(cfg, record.tag_id, record.anchor_id) + bias;
      if (development_request)
        development_ranges.push_back({index, X(record.keyframe_id),
            anchors.at(record.anchor_id), cfg.lever_arm_init, record.raw_range,
            record.nominal_sigma, conditional_beta, record.obs_id});
      graph.add(MakeUwbFactor(
          X(record.keyframe_id), 0, 0, 0, anchors.at(record.anchor_id),
          cfg.lever_arm_init, record.raw_range, record.nominal_sigma, false,
          false, false,
          conditional_beta));
    }
    return graph;
  };

  result.navigation_values = base_values;
  double previous_objective = base_graph.error(base_values);
  if (!std::isfinite(previous_objective))
    return fail(DiscoveryStatus::NONFINITE_VALUE,
                "initial Stage-1 objective is nonfinite");

  for (size_t outer = 1; outer <= options_.max_outer_iterations; ++outer) {
    const gtsam::Values navigation_before = result.navigation_values;
    std::vector<double> bias_before;
    bias_before.reserve(result.snapshot.size());
    for (const auto& item : result.snapshot) bias_before.push_back(item.bias_m);
    const auto conditional_graph = fixed_bias_graph(result.snapshot);
    const CheckedLmDiagnosticRequest* conditional_lm_diagnostic = nullptr;
    if (diagnostic_request &&
        diagnostic_request->conditional_lm_outer_iteration == outer)
      conditional_lm_diagnostic = &diagnostic_request->conditional_lm;
    const auto lm = development_request
        ? development_request->conditional_navigation(outer, conditional_graph,
            result.navigation_values, options_.conditional_lm, development_ranges)
        : RunCheckedConditionalLm(conditional_graph, result.navigation_values,
            options_.conditional_lm, conditional_lm_diagnostic);
    result.conditional_lm_attempted = true;
    result.conditional_lm_attempted_outer_iteration = outer;
    result.last_conditional_lm = lm;
    result.added_diagnostics_seconds_total +=
        lm.convergence.added_diagnostics_seconds;
    if (conditional_lm_diagnostic && conditional_lm_diagnostic->passive_terminal_capture)
      return fail(DiscoveryStatus::CONDITIONAL_LM_FAILED,
                  "A15_PASSIVE_CAPTURE_STOP_BEFORE_CHAIN: " + lm.reason);
    if (conditional_lm_diagnostic &&
        conditional_lm_diagnostic->first_block_budget_diagnostic)
      return fail(DiscoveryStatus::CONDITIONAL_LM_FAILED,
                  "A11_FIRST_BLOCK_DIAGNOSTIC_STOP_BEFORE_CHAIN: " + lm.reason);
    if (!lm.converged)
      return fail(DiscoveryStatus::CONDITIONAL_LM_FAILED, lm.reason);
    if (!NavigationValuesFinite(lm.values, plan.keyframes.size()))
      return fail(DiscoveryStatus::NONFINITE_VALUE,
                  "conditional navigation Values are nonfinite");
    const auto pre_chain_stationarity_started =
        std::chrono::steady_clock::now();
    const auto pre_chain_stationarity = AuditNavigationStationarity(
        conditional_graph, lm.values, options_.navigation_scales,
        options_.navigation_stationarity_tolerance_objective,
        options_.gradient_roundoff_safety_factor);
    const double pre_chain_stationarity_seconds =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            pre_chain_stationarity_started).count();
    result.added_diagnostics_seconds_total +=
        pre_chain_stationarity_seconds;

    std::vector<FusedLassoResult> chain_solutions;
    size_t begin = 0;
    while (begin < result.snapshot.size()) {
      size_t end = begin + 1;
      while (end < result.snapshot.size() &&
             result.snapshot[end].chain_id == result.snapshot[begin].chain_id)
        ++end;
      std::vector<double> e, weights, warm;
      for (size_t i = begin; i < end; ++i) {
        const auto& record = *records.at(result.snapshot[i].obs_id);
        const auto pose = lm.values.at<gtsam::Pose3>(X(record.keyframe_id));
        const gtsam::Point3 antenna = pose.transformFrom(cfg.lever_arm_init);
        const double geometric =
            (gtsam::Vector3(antenna) -
             gtsam::Vector3(anchors.at(record.anchor_id))).norm();
        e.push_back(record.raw_range - geometric -
                    FixedBetaForLink(cfg, record.tag_id, record.anchor_id));
        weights.push_back(result.snapshot[i].weight);
        warm.push_back(result.snapshot[i].bias_m);
      }
      const auto solved = SolveNonnegativeFusedLassoChain(
          e, weights, options_.fused_lasso, &warm);
      if (!solved.converged()) {
        result.chain_solutions = std::move(chain_solutions);
        result.chain_solutions.push_back(solved);
        return fail(DiscoveryStatus::CHAIN_SOLVE_FAILED,
                    std::string(FusedLassoStatusName(solved.status)) + ": " +
                        solved.reason);
      }
      for (size_t i = begin; i < end; ++i)
        result.snapshot[i].bias_m = solved.u[i - begin];
      chain_solutions.push_back(solved);
      begin = end;
    }

    // active_run_id is immutable partition provenance derived only after the
    // converged support amplitudes. It prevents A01 from crossing inactive.
    size_t run_id = 0;
    const DiscoveryObservation* prev = nullptr;
    bool prev_active = false;
    for (auto& item : result.snapshot) {
      const bool active = item.bias_m >= options_.active_bias_min_m;
      if (active &&
          (!prev || !prev_active || prev->chain_id != item.chain_id))
        ++run_id;
      item.active_run_id = active ? run_id : 0;
      prev_active = active;
      prev = &item;
    }

    const auto objective_graph = fixed_bias_graph(result.snapshot);
    double objective = objective_graph.error(lm.values);
    for (const auto& solved : chain_solutions) {
      for (double value : solved.u)
        objective += options_.fused_lasso.lambda_l1 * value;
      for (size_t i = 1; i < solved.u.size(); ++i)
        objective += options_.fused_lasso.lambda_tv *
                     std::abs(solved.u[i] - solved.u[i - 1]);
    }
    if (!std::isfinite(objective))
      return fail(DiscoveryStatus::NONFINITE_VALUE,
                  "final feasible-u Stage-1 objective is nonfinite");
    const double allowance =
        Binary64ObjectiveIncreaseAllowance(previous_objective, objective);
    if (objective > previous_objective + allowance)
      return fail(DiscoveryStatus::OBJECTIVE_INCREASE,
                  "Stage-1 objective increased beyond binary64 roundoff allowance");
    const double relative_change =
        std::abs(previous_objective - objective) /
        std::max(1.0, std::abs(previous_objective));
    const auto post_chain_stationarity_started =
        std::chrono::steady_clock::now();
    const auto stationarity = AuditNavigationStationarity(
        objective_graph, lm.values, options_.navigation_scales,
        options_.navigation_stationarity_tolerance_objective,
        options_.gradient_roundoff_safety_factor);
    const double post_chain_stationarity_seconds =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            post_chain_stationarity_started).count();
    if (!stationarity.valid)
      return fail(DiscoveryStatus::NONFINITE_VALUE, stationarity.reason);
    std::vector<double> bias_after;
    bias_after.reserve(result.snapshot.size());
    for (const auto& item : result.snapshot) bias_after.push_back(item.bias_m);
    const auto scaled_step = AuditNavigationAndBiasScaledStep(
        navigation_before, lm.values, bias_before, bias_after,
        options_.navigation_scales, options_.observation_bias_scale_m);
    if (!scaled_step.valid)
      return fail(DiscoveryStatus::NONFINITE_VALUE, scaled_step.reason);

    DiscoveryIteration trace;
    trace.outer_iteration = outer;
    trace.conditional_lm_iterations = lm.iterations;
    trace.conditional_lm_inner_iterations = lm.inner_iterations;
    trace.conditional_lm_lambda = lm.lambda;
    trace.conditional_lm_convergence = lm.convergence;
    trace.conditional_lm_qualification_stationarity =
        lm.last_qualification_stationarity;
    trace.objective_before = previous_objective;
    trace.objective_after = objective;
    trace.allowed_objective_increase = allowance;
    trace.relative_objective_change = relative_change;
    trace.max_navigation_scaled_step = scaled_step.max_navigation_step;
    trace.max_bias_scaled_step = scaled_step.max_bias_step;
    trace.combined_scaled_step = scaled_step.max_combined_step;
    trace.navigation_gradient_objective =
        stationarity.max_scaled_gradient_objective;
    trace.navigation_roundoff_allowance_objective =
        stationarity.roundoff_allowance_objective;
    trace.objective_ok =
        relative_change <= options_.relative_objective_tolerance;
    trace.step_ok =
        scaled_step.max_combined_step <= options_.scaled_step_tolerance;
    trace.chain_optimality_ok = true;
    for (const auto& solved : chain_solutions) {
      trace.max_chain_kkt_objective_per_m =
          std::max(trace.max_chain_kkt_objective_per_m,
                   solved.max_kkt_violation_objective_per_m);
      trace.max_chain_primal_residual_m =
          std::max(trace.max_chain_primal_residual_m,
                   solved.primal_residual_m);
      trace.max_chain_dual_residual_objective_per_m =
          std::max(trace.max_chain_dual_residual_objective_per_m,
                   solved.dual_residual_objective_per_m);
      trace.chain_optimality_ok =
          trace.chain_optimality_ok && solved.converged();
    }
    trace.navigation_stationarity_ok = stationarity.stationary;
    trace.pre_chain_navigation_stationarity = pre_chain_stationarity;
    trace.post_chain_navigation_stationarity = stationarity;
    trace.stationarity_audits_share_navigation_values = true;
    trace.pre_chain_stationarity_seconds =
        pre_chain_stationarity_seconds;
    trace.post_chain_stationarity_seconds =
        post_chain_stationarity_seconds;
    trace.added_diagnostics_seconds =
        lm.convergence.added_diagnostics_seconds +
        pre_chain_stationarity_seconds;
    result.iterations.push_back(trace);
    if (development_request && development_request->outer_observer)
      development_request->outer_observer(trace);
    result.navigation_values = lm.values;
    result.chain_solutions = std::move(chain_solutions);
    previous_objective = objective;
    if (trace.objective_ok && trace.step_ok && trace.chain_optimality_ok &&
        trace.navigation_stationarity_ok) {
      try {
        result.partition =
            BuildAutomaticSupportPartition(result.snapshot, options_, context);
      } catch (const std::exception& error) {
        return fail(DiscoveryStatus::PARTITION_INVALID, error.what());
      }
      if (development_request) {
        result.partition.schema = development_request->output_schema;
        result.partition.provider = development_request->output_provider;
      }
      result.status = DiscoveryStatus::CONVERGED;
      result.reason = result.partition.segments.empty()
                          ? "NO_CANDIDATES"
                          : "AUTOMATIC_SUPPORT_DISCOVERED";
      return result;
    }
  }
  return fail(DiscoveryStatus::MAX_OUTER_ITERATIONS,
              "Stage-1 reached outer iteration limit before all stop conditions");
}

Stage1RegularizedResult BuildStage1RegularizedResult(
    const DiscoveryResult& discovery,
    const gtsam::NonlinearFactorGraph& base_graph,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const DiscoveryContext& context, const DiscoveryOptions& options) {
  Stage1RegularizedResult result;
  auto fail = [&](const std::string& reason) {
    result.reason = reason;
    result.status = "INVALID";
    return result;
  };
  try {
    if (!discovery.converged())
      return fail("STAGE1_NOT_CONVERGED");
    if (cfg.calib_anchor || cfg.calib_lever || cfg.calib_range_bias ||
        cfg.calib_td)
      return fail("STAGE1_ONLY_REQUIRES_FIXED_CALIBRATION");
    if (discovery.snapshot.size() != plan.observations.size())
      return fail("STAGE1_SNAPSHOT_LEDGER_SIZE_MISMATCH");
    if (!GraphAndValuesKeysMatch(base_graph, discovery.navigation_values))
      return fail("STAGE1_BASE_GRAPH_VALUES_KEY_MISMATCH");

    std::unordered_map<std::uint64_t, const DiscoveryObservation*> bias_by_obs;
    for (const auto& item : discovery.snapshot) {
      if (!std::isfinite(item.bias_m) || item.bias_m < 0.0 ||
          !bias_by_obs.emplace(item.obs_id, &item).second)
        return fail("STAGE1_SNAPSHOT_INVALID_OR_DUPLICATE_OBS_ID");
    }
    std::unordered_map<std::uint64_t, const ObservationRecord*> plan_by_obs;
    for (const auto& item : plan.observations) {
      if (!plan_by_obs.emplace(item.obs_id, &item).second)
        return fail("STAGE1_PLAN_DUPLICATE_OBS_ID");
    }
    std::unordered_map<size_t, const FactorMeta*> uwb_by_factor;
    for (const auto& meta : base_uwb_factor_metadata) {
      if (meta.factor_type != "uwb_range") continue;
      if (meta.factor_index >= base_graph.size() ||
          !uwb_by_factor.emplace(meta.factor_index, &meta).second)
        return fail("STAGE1_BASE_FACTOR_METADATA_INVALID");
    }

    for (size_t index = 0; index < base_graph.size(); ++index) {
      FactorMeta output_meta;
      output_meta.factor_index = result.physical_graph.size();
      const auto meta = uwb_by_factor.find(index);
      if (meta == uwb_by_factor.end()) {
        const auto& factor = base_graph.at(index);
        if (!factor) return fail("STAGE1_BASE_GRAPH_NULL_FACTOR");
        result.physical_graph.add(factor);
        output_meta.factor_type = "physical_non_uwb";
        output_meta.keys.assign(factor->keys().begin(), factor->keys().end());
      } else {
        const auto observation = plan_by_obs.find(meta->second->obs_id);
        const auto bias = bias_by_obs.find(meta->second->obs_id);
        if (observation == plan_by_obs.end() || bias == bias_by_obs.end())
          return fail("STAGE1_FACTOR_OBSERVATION_MAPPING_INCOMPLETE");
        const ObservationRecord& obs = *observation->second;
        if (!obs.valid || !obs.planned)
          return fail("STAGE1_FACTOR_REFERENCES_UNPLANNED_OBSERVATION");
        const auto anchor = std::find_if(
            cfg.anchors.begin(), cfg.anchors.end(),
            [&](const AnchorConfig& item) { return item.id == obs.anchor_id; });
        if (anchor == cfg.anchors.end())
          return fail("STAGE1_OBSERVATION_ANCHOR_MISSING");
        const double fixed_dynamic =
            FixedBetaForLink(cfg, obs.tag_id, obs.anchor_id) +
            bias->second->bias_m;
        auto factor = MakeUwbFactor(
            X(obs.keyframe_id), gtsam::Symbol('l', 0),
            gtsam::Symbol('a', obs.anchor_id),
            gtsam::Symbol('z', obs.anchor_id), anchor->pos,
            cfg.lever_arm_init, obs.raw_range, obs.nominal_sigma, false,
            false, false, fixed_dynamic);
        result.physical_graph.add(factor);
        output_meta = *meta->second;
        output_meta.factor_index = result.physical_graph.size() - 1;
        output_meta.factor_type = "uwb_range_stage1_fixed_dynamic_bias";
        output_meta.keys.assign(factor->keys().begin(), factor->keys().end());
      }
      result.physical_factor_metadata.push_back(std::move(output_meta));
    }
    if (!GraphAndValuesKeysMatch(result.physical_graph,
                                 discovery.navigation_values))
      return fail("STAGE1_PHYSICAL_GRAPH_VALUES_KEY_MISMATCH");
    result.objective_physical =
        result.physical_graph.error(discovery.navigation_values);
    std::map<size_t, std::vector<const DiscoveryObservation*>> chains;
    for (const auto& item : discovery.snapshot) {
      const auto plan_item = plan_by_obs.find(item.obs_id);
      if (plan_item == plan_by_obs.end() || !plan_item->second->valid ||
          !plan_item->second->planned)
        continue;
      result.objective_l1 += options.fused_lasso.lambda_l1 * item.bias_m;
      chains[item.chain_id].push_back(&item);
      result.observation_bias_snapshot.push_back(item);
    }
    for (auto& chain : chains) {
      std::sort(chain.second.begin(), chain.second.end(),
                [](const auto* lhs, const auto* rhs) {
                  return std::tie(lhs->sensor_time, lhs->obs_id) <
                         std::tie(rhs->sensor_time, rhs->obs_id);
                });
      for (size_t i = 1; i < chain.second.size(); ++i)
        result.objective_tv += options.fused_lasso.lambda_tv *
            std::abs(chain.second[i]->bias_m - chain.second[i - 1]->bias_m);
    }
    result.objective_total = result.objective_physical +
                             result.objective_l1 + result.objective_tv;
    if (!std::isfinite(result.objective_total))
      return fail("STAGE1_OBJECTIVE_NONFINITE");

    std::ostringstream snapshot_bytes;
    snapshot_bytes << std::setprecision(17)
                   << "uifgo-t09-stage1-observation-snapshot-v1\n";
    for (const auto& item : result.observation_bias_snapshot)
      snapshot_bytes << item.obs_id << ',' << item.tag_id << ','
                     << item.anchor_id << ',' << item.chain_id << ','
                     << item.active_run_id << ',' << item.sensor_time << ','
                     << item.weight << ',' << item.bias_m << '\n';
    result.snapshot_sha256 =
        "t09stage1snapshot-sha256:" + Sha256Hex(snapshot_bytes.str());
    InferenceIdentityContext identity_context;
    identity_context.input_sha256 = context.source_hash;
    identity_context.config_sha256 = context.config_hash;
    identity_context.input_plan_sha256 = context.input_plan_hash;
    identity_context.support_partition_sha256 =
        discovery.partition.partition_hash.empty()
            ? result.snapshot_sha256
            : discovery.partition.partition_hash;
    identity_context.calibration_sha256 = context.calibration_hash;
    identity_context.solver_config_sha256 = context.solver_config_hash;
    const auto identity = ComputeInferenceContentIdentity(
        result.physical_graph, discovery.navigation_values, identity_context);
    result.graph_linearization_sha256 = identity.graph_linearization_sha256;
    result.values_sha256 = identity.values_sha256;
    std::ostringstream identity_bytes;
    identity_bytes << "uifgo-t09-stage1-regularized-result-v1\n"
                   << result.graph_linearization_sha256 << '\n'
                   << result.values_sha256 << '\n'
                   << result.snapshot_sha256 << '\n'
                   << std::setprecision(17) << result.objective_physical << '\n'
                   << result.objective_l1 << '\n' << result.objective_tv << '\n'
                   << result.objective_total << '\n';
    result.result_id =
        "t09stage1-sha256:" + Sha256Hex(identity_bytes.str());
    result.navigation_values = discovery.navigation_values;
    result.partition = discovery.partition;
    result.iterations = discovery.iterations;
    result.valid = true;
    result.status = "CONVERGED";
    result.reason = "STAGE1_REGULARIZED_RESULT_AUDIT_OK";
  } catch (const std::exception& error) {
    return fail(error.what());
  }
  return result;
}

}  // namespace uifgo
