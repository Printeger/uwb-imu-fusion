#include "uifgo/imu_preint.h"
#include "uifgo/nlos_refit.h"

#include <yaml-cpp/yaml.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include "uifgo/uwb_factor.h"
#include "uifgo/nlos_solver_utils.h"

namespace uifgo {
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

template <typename Container>
void RequireAllowedKeys(const YAML::Node& node, const Container& allowed,
                        const std::string& context) {
  if (!node.IsMap()) throw std::invalid_argument(context + " must be a map");
  for (const auto& item : node) {
    const std::string key = item.first.as<std::string>();
    if (!allowed.count(key)) {
      throw std::invalid_argument(context + " contains forbidden/unknown field: " +
                                  key);
    }
  }
}

const ObservationRecord& RequireObservation(
    std::uint64_t obs_id,
    const std::unordered_map<std::uint64_t, const ObservationRecord*>& records) {
  const auto it = records.find(obs_id);
  if (it == records.end()) {
    throw std::invalid_argument("oracle support references unknown obs_id " +
                                std::to_string(obs_id));
  }
  const ObservationRecord& record = *it->second;
  if (!record.valid || !record.planned) {
    throw std::invalid_argument("oracle support references invalid/unplanned obs_id " +
                                std::to_string(obs_id));
  }
  return record;
}

bool RefitValuesFinite(const gtsam::Values& values, size_t keyframe_count,
                       size_t segment_count) {
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
  for (size_t s = 0; s < segment_count; ++s) {
    if (!values.exists(C(s)) || !std::isfinite(values.at<double>(C(s))))
      return false;
  }
  return true;
}

NavigationScales RefitNavigationScales(const RefitOptions& options) {
  NavigationScales scales;
  scales.pose_rotation_rad = options.pose_rotation_scale_rad;
  scales.pose_translation_m = options.pose_translation_scale_m;
  scales.velocity_mps = options.velocity_scale_mps;
  scales.accel_bias_mps2 = options.accel_bias_scale_mps2;
  scales.gyro_bias_radps = options.gyro_bias_scale_radps;
  return scales;
}

struct NavigationStationarityAudit {
  SegmentRefitStatus status = SegmentRefitStatus::CONVERGED;
  std::string reason;
  double max_pose_rotation_gradient_objective_per_rad = 0.0;
  double max_pose_translation_gradient_objective_per_m = 0.0;
  double max_velocity_gradient_objective_per_mps = 0.0;
  double max_accel_bias_gradient_objective_per_mps2 = 0.0;
  double max_gyro_bias_gradient_objective_per_radps = 0.0;
  double max_scaled_navigation_gradient_objective = 0.0;
  double roundoff_allowance_objective = 0.0;
  bool stationarity_ok = true;
};

NavigationStationarityAudit AuditNavigationStationarity(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const RefitOptions& options) {
  NavigationStationarityAudit audit;
  try {
    const auto linearized = graph.linearize(values);
    if (!linearized || linearized->empty()) {
      audit.status = SegmentRefitStatus::INVALID_INPUT;
      audit.reason = "joint graph linearization is empty";
      return audit;
    }
    const gtsam::VectorValues gradient = linearized->gradientAtZero();
    gtsam::VectorValues absolute_factor_gradient_sum = values.zeroVectors();
    for (const auto& factor : *linearized) {
      if (!factor) {
        audit.status = SegmentRefitStatus::INVALID_INPUT;
        audit.reason = "joint graph linearization contains a null factor";
        return audit;
      }
      const gtsam::VectorValues factor_gradient = factor->gradientAtZero();
      for (const auto& key_vector : factor_gradient) {
        absolute_factor_gradient_sum.at(key_vector.first) +=
            key_vector.second.cwiseAbs();
      }
    }

    const double unit_roundoff = std::numeric_limits<double>::epsilon() / 2.0;
    const double product =
        static_cast<double>(linearized->size()) * unit_roundoff;
    if (!(product < 1.0)) {
      audit.status = SegmentRefitStatus::NONFINITE_VALUE;
      audit.reason = "binary64 gradient summation error model is inapplicable";
      return audit;
    }
    const double gamma_n = product / (1.0 - product);
    const double roundoff_multiplier =
        options.gradient_roundoff_safety_factor * (gamma_n + unit_roundoff);

    for (gtsam::Key key : values.keys()) {
      const char symbol = gtsam::Symbol(key).chr();
      if (symbol == 'c') continue;  // Segment coordinates use nonnegative KKT.
      if (!gradient.exists(key) || !absolute_factor_gradient_sum.exists(key)) {
        audit.status = SegmentRefitStatus::GRAPH_VALUES_KEY_MISMATCH;
        audit.reason = "joint gradient and Values keys differ";
        return audit;
      }
      const gtsam::Vector& key_gradient = gradient.at(key);
      const gtsam::Vector& key_absolute_sum =
          absolute_factor_gradient_sum.at(key);
      if (key_gradient.size() != key_absolute_sum.size() ||
          !key_gradient.allFinite() || !key_absolute_sum.allFinite()) {
        audit.status = SegmentRefitStatus::NONFINITE_VALUE;
        audit.reason = "joint navigation gradient is nonfinite";
        return audit;
      }
      for (Eigen::Index j = 0; j < key_gradient.size(); ++j) {
        double scale = 0.0;
        double* category_maximum = nullptr;
        if (symbol == 'x' && key_gradient.size() == 6) {
          if (j < 3) {
            scale = options.pose_rotation_scale_rad;
            category_maximum =
                &audit.max_pose_rotation_gradient_objective_per_rad;
          } else {
            scale = options.pose_translation_scale_m;
            category_maximum =
                &audit.max_pose_translation_gradient_objective_per_m;
          }
        } else if (symbol == 'v' && key_gradient.size() == 3) {
          scale = options.velocity_scale_mps;
          category_maximum = &audit.max_velocity_gradient_objective_per_mps;
        } else if (symbol == 'b' && key_gradient.size() == 6) {
          if (j < 3) {
            scale = options.accel_bias_scale_mps2;
            category_maximum =
                &audit.max_accel_bias_gradient_objective_per_mps2;
          } else {
            scale = options.gyro_bias_scale_radps;
            category_maximum =
                &audit.max_gyro_bias_gradient_objective_per_radps;
          }
        } else {
          audit.status = SegmentRefitStatus::INVALID_INPUT;
          audit.reason =
              "T04 has no declared stationarity scale for free key " +
              gtsam::DefaultKeyFormatter(key);
          return audit;
        }
        const double native_gradient = std::abs(key_gradient[j]);
        const double scaled_gradient = native_gradient * scale;
        const double allowance =
            roundoff_multiplier * key_absolute_sum[j] * scale;
        if (!std::isfinite(scaled_gradient) || !std::isfinite(allowance)) {
          audit.status = SegmentRefitStatus::NONFINITE_VALUE;
          audit.reason = "scaled joint navigation gradient is nonfinite";
          return audit;
        }
        *category_maximum = std::max(*category_maximum, native_gradient);
        audit.max_scaled_navigation_gradient_objective =
            std::max(audit.max_scaled_navigation_gradient_objective,
                     scaled_gradient);
        audit.roundoff_allowance_objective =
            std::max(audit.roundoff_allowance_objective, allowance);
        if (scaled_gradient >
            options.navigation_stationarity_tolerance_objective + allowance) {
          audit.stationarity_ok = false;
        }
      }
    }
  } catch (const std::exception& error) {
    audit.status = SegmentRefitStatus::NONFINITE_VALUE;
    audit.reason = std::string("joint stationarity audit failed: ") +
                   error.what();
  }
  return audit;
}

std::map<int, gtsam::Point3> AnchorMap(const Config& cfg) {
  std::map<int, gtsam::Point3> anchors;
  for (const auto& anchor : cfg.anchors) anchors.emplace(anchor.id, anchor.pos);
  return anchors;
}

}  // namespace

OracleSupport OracleSupportProvider::Load(const std::string& yaml_path,
                                          const PaperInputPlan& plan,
                                          size_t short_min_count,
                                          double short_min_duration) {
  if (!std::isfinite(short_min_duration) || short_min_duration < 0.0)
    throw std::invalid_argument("short_min_duration must be finite/nonnegative");
  const YAML::Node root = YAML::LoadFile(yaml_path);
  const bool t04_schema = root["schema"] &&
      root["schema"].as<std::string>() == "t04_oracle_support_v1";
  const bool t09_fixed_schema = root["schema"] &&
      root["schema"].as<std::string>() == "t09_fixed_partition_v1";
  const char* required_label =
      t09_fixed_schema ? kRq3FixedPartitionDebugLabel : kT04OracleDebugLabel;
  RequireAllowedKeys(root,
                     std::set<std::string>{"schema", required_label,
                                           "recipe_hash", "segments"},
                     "oracle manifest root");
  if (!t04_schema && !t09_fixed_schema) {
    throw std::invalid_argument(
        "support manifest schema must be t04_oracle_support_v1 or t09_fixed_partition_v1");
  }
  if (!root[required_label] || !root[required_label].as<bool>()) {
    throw std::invalid_argument(std::string("oracle manifest requires ") +
                                required_label + ": true");
  }
  if (t09_fixed_schema &&
      (!root["recipe_hash"] || root["recipe_hash"].as<std::string>().empty()))
    throw std::invalid_argument("fixed partition manifest requires recipe_hash");
  if (!root["segments"] || !root["segments"].IsSequence() ||
      root["segments"].size() == 0) {
    throw std::invalid_argument("oracle manifest segments must be nonempty");
  }

  std::unordered_map<std::uint64_t, const ObservationRecord*> records;
  for (const auto& record : plan.observations) {
    if (!records.emplace(record.obs_id, &record).second)
      throw std::invalid_argument("input plan contains duplicate obs_id");
  }

  OracleSupport support;
  support.schema = root["schema"].as<std::string>();
  support.source_path = yaml_path;
  std::set<std::string> segment_ids;
  std::set<std::uint64_t> assigned;

  for (size_t ordinal = 0; ordinal < root["segments"].size(); ++ordinal) {
    const YAML::Node node = root["segments"][ordinal];
    RequireAllowedKeys(
        node,
        std::set<std::string>{"segment_id", "link", "obs_ids",
                              "start_time", "end_time"},
        "oracle segment");
    if (!node["segment_id"] || !node["link"])
      throw std::invalid_argument("oracle segment requires segment_id and link");
    OracleSegment segment;
    segment.segment_id = node["segment_id"].as<std::string>();
    if (segment.segment_id.empty() ||
        !segment_ids.insert(segment.segment_id).second)
      throw std::invalid_argument("oracle segment_id must be nonempty/unique");
    const std::string link = node["link"].as<std::string>();
    if (!ParseRangeLinkKey(link, &segment.tag_id, &segment.anchor_id))
      throw std::invalid_argument("oracle segment link is not canonical: " + link);

    const bool has_obs = static_cast<bool>(node["obs_ids"]);
    const bool has_start = static_cast<bool>(node["start_time"]);
    const bool has_end = static_cast<bool>(node["end_time"]);
    if (has_obs == (has_start || has_end) || has_start != has_end) {
      throw std::invalid_argument(
          "oracle segment requires exactly obs_ids or [start_time,end_time]");
    }

    if (has_obs) {
      if (!node["obs_ids"].IsSequence() || node["obs_ids"].size() == 0)
        throw std::invalid_argument("oracle obs_ids must be a nonempty sequence");
      for (const auto& id_node : node["obs_ids"])
        segment.obs_ids.push_back(id_node.as<std::uint64_t>());
    } else {
      const double start = node["start_time"].as<double>();
      const double end = node["end_time"].as<double>();
      if (!std::isfinite(start) || !std::isfinite(end) || start > end)
        throw std::invalid_argument("oracle interval must be finite and closed");
      for (const auto& record : plan.observations) {
        if (record.valid && record.planned && record.tag_id == segment.tag_id &&
            record.anchor_id == segment.anchor_id &&
            record.sensor_time >= start && record.sensor_time <= end) {
          segment.obs_ids.push_back(record.obs_id);
        }
      }
      if (segment.obs_ids.empty())
        throw std::invalid_argument("oracle interval selects no planned observations");
    }

    double actual_start = std::numeric_limits<double>::infinity();
    double actual_end = -std::numeric_limits<double>::infinity();
    for (std::uint64_t obs_id : segment.obs_ids) {
      const ObservationRecord& record = RequireObservation(obs_id, records);
      if (record.tag_id != segment.tag_id ||
          record.anchor_id != segment.anchor_id) {
        throw std::invalid_argument("oracle segment crosses its declared link");
      }
      if (!assigned.insert(obs_id).second)
        throw std::invalid_argument("oracle obs_id has duplicate segment ownership");
      actual_start = std::min(actual_start, record.sensor_time);
      actual_end = std::max(actual_end, record.sensor_time);
    }
    segment.segment_ordinal = ordinal;
    segment.start_time = actual_start;
    segment.end_time = actual_end;
    segment.observation_count = segment.obs_ids.size();
    segment.duration = actual_end - actual_start;
    segment.short_support_debug =
        segment.observation_count < short_min_count ||
        segment.duration < short_min_duration;
    support.segments.push_back(std::move(segment));
  }
  return support;
}

const char* SegmentRefitStatusName(SegmentRefitStatus status) {
  switch (status) {
    case SegmentRefitStatus::CONVERGED:
      return "CONVERGED";
    case SegmentRefitStatus::INVALID_INPUT:
      return "INVALID_INPUT";
    case SegmentRefitStatus::NONPOSITIVE_OR_NONFINITE_DENOMINATOR:
      return "NONPOSITIVE_OR_NONFINITE_DENOMINATOR";
    case SegmentRefitStatus::CONDITIONAL_LM_FAILED:
      return "CONDITIONAL_LM_FAILED";
    case SegmentRefitStatus::NONFINITE_VALUE:
      return "NONFINITE_VALUE";
    case SegmentRefitStatus::OBJECTIVE_INCREASE:
      return "OBJECTIVE_INCREASE";
    case SegmentRefitStatus::GRAPH_VALUES_KEY_MISMATCH:
      return "GRAPH_VALUES_KEY_MISMATCH";
    case SegmentRefitStatus::MAX_REFIT_ITERATIONS:
      return "MAX_REFIT_ITERATIONS";
  }
  return "UNKNOWN";
}

NonnegativeAmplitudeUpdate ComputeNonnegativeSegmentAmplitude(
    const std::vector<double>& raw_ranges,
    const std::vector<double>& geometry_plus_fixed_beta,
    const std::vector<double>& nominal_sigmas) {
  NonnegativeAmplitudeUpdate update;
  if (raw_ranges.empty() || raw_ranges.size() != geometry_plus_fixed_beta.size() ||
      raw_ranges.size() != nominal_sigmas.size())
    return update;
  for (size_t i = 0; i < raw_ranges.size(); ++i) {
    const double sigma = nominal_sigmas[i];
    if (!(sigma > 0.0) || !std::isfinite(sigma) ||
        !std::isfinite(raw_ranges[i]) ||
        !std::isfinite(geometry_plus_fixed_beta[i]))
      return update;
    const double weight = 1.0 / (sigma * sigma);
    update.denominator += weight;
    update.numerator +=
        weight * (raw_ranges[i] - geometry_plus_fixed_beta[i]);
  }
  if (!(update.denominator > 0.0) || !std::isfinite(update.denominator) ||
      !std::isfinite(update.numerator))
    return update;
  update.amplitude_m = std::max(0.0, update.numerator / update.denominator);
  update.valid = std::isfinite(update.amplitude_m);
  return update;
}

bool Binary64Equal(double lhs, double rhs) {
  return std::memcmp(&lhs, &rhs, sizeof(double)) == 0;
}

bool MatrixBinary64Equal(const gtsam::Matrix& lhs, const gtsam::Matrix& rhs) {
  if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) return false;
  for (Eigen::Index row = 0; row < lhs.rows(); ++row)
    for (Eigen::Index col = 0; col < lhs.cols(); ++col)
      if (!Binary64Equal(lhs(row, col), rhs(row, col))) return false;
  return true;
}

// The legacy ExpressionFactor does not expose its captured anchor/lever
// leaves.  Compare it against the factor rebuilt from the immutable plan and
// fixed calibration at the actual live Values.  Exact measurement, keys,
// whitening, residual and Jacobian equality jointly bind all range inputs.
bool RangeFactorMatchesExpected(
    const gtsam::NonlinearFactor::shared_ptr& actual,
    const gtsam::NonlinearFactor::shared_ptr& expected,
    const gtsam::Values& values, double measurement, double sigma,
    double expected_unwhitened_residual) {
  const auto actual_expression =
      boost::dynamic_pointer_cast<gtsam::ExpressionFactor<double>>(actual);
  const auto expected_expression =
      boost::dynamic_pointer_cast<gtsam::ExpressionFactor<double>>(expected);
  const auto actual_noise =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
          actual_expression ? actual_expression->noiseModel() : nullptr);
  const auto expected_noise =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
          expected_expression ? expected_expression->noiseModel() : nullptr);
  if (!actual_expression || !expected_expression || !actual_noise ||
      !expected_noise || !Binary64Equal(actual_expression->measured(), measurement) ||
      !Binary64Equal(expected_expression->measured(), measurement) ||
      actual->keys() != expected->keys() ||
      !MatrixBinary64Equal(actual_noise->R(), expected_noise->R()) ||
      actual_noise->R().rows() != 1 ||
      !Binary64Equal(actual_noise->R()(0, 0), 1.0 / sigma))
    return false;
  const auto actual_error = actual_expression->unwhitenedError(values);
  const auto expected_error = expected_expression->unwhitenedError(values);
  if (actual_error.size() != 1 || expected_error.size() != 1 ||
      // Residual equality is numerical, not an artifact identity comparison.
      // GTSAM ExpressionFactor can return -0 for an exact fit while the
      // independent arithmetic residual is +0. No nonzero tolerance is added.
      !std::isfinite(expected_unwhitened_residual) ||
      actual_error[0] != expected_unwhitened_residual ||
      expected_error[0] != expected_unwhitened_residual)
    return false;
  const auto actual_linear =
      boost::dynamic_pointer_cast<gtsam::JacobianFactor>(
          actual_expression->linearize(values));
  const auto expected_linear =
      boost::dynamic_pointer_cast<gtsam::JacobianFactor>(
          expected_expression->linearize(values));
  return actual_linear && expected_linear &&
         MatrixBinary64Equal(actual_linear->augmentedJacobian(),
                             expected_linear->augmentedJacobian());
}

SegmentRefitResult SegmentRefitter::RunFrozenCandidatePolicyImpl(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& base_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const SupportPartition& support,
    const std::set<size_t>& accepted_segment_ordinals,
    bool execute_optimization,
    const DevelopmentStage2Request* development_request) const {
  SegmentRefitResult result;
  auto fail = [&](SegmentRefitStatus status, const std::string& reason) {
    result.status = status;
    result.reason = reason;
    return result;
  };

  if (development_request &&
      ((development_request->policy != "PAPER_CERTIFIED_PAIR_REDUCTION_V1" &&
        development_request->policy !=
            "PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1") ||
       (development_request->role != "development" &&
        development_request->role != "validation") ||
       (development_request->role == "validation" &&
        development_request->validation_context_sha256.empty()) ||
       (development_request->role == "development" &&
        !development_request->validation_context_sha256.empty()) ||
       development_request->implementation_identity.rfind(
           "a19-policy-sha256:", 0) != 0 ||
       support.solver_config_hash !=
           development_request->implementation_identity ||
       !development_request->conditional_navigation ||
       (development_request->allow_inexact_handoff !=
        (development_request->policy ==
         "PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1")))) {
    return fail(SegmentRefitStatus::INVALID_INPUT,
                "A19_INVALID_DEVELOPMENT_POLICY_OR_IDENTITY");
  }

  std::string imu_model_reason;
  if (!PaperImuCovarianceModelMatchesGraph(base_graph, cfg, &imu_model_reason))
    return fail(SegmentRefitStatus::INVALID_INPUT, imu_model_reason);

  if (cfg.calib_lever || cfg.calib_anchor || cfg.calib_range_bias ||
      cfg.calib_td || base_graph.empty() ||
      options_.max_refit_iterations == 0 || options_.lm_max_iterations <= 0) {
    return fail(SegmentRefitStatus::INVALID_INPUT,
                "segment refit requires fixed calibration and positive iteration limits");
  }
  const double option_values[] = {
      options_.boundary_epsilon_m,
      options_.relative_objective_tolerance,
      options_.scaled_step_tolerance,
      options_.projected_gradient_tolerance,
      options_.navigation_stationarity_tolerance_objective,
      options_.gradient_roundoff_safety_factor,
      options_.lm_relative_tolerance,
      options_.lm_absolute_tolerance,
      options_.pose_rotation_scale_rad,
      options_.pose_translation_scale_m,
      options_.velocity_scale_mps,
      options_.accel_bias_scale_mps2,
      options_.gyro_bias_scale_radps,
      options_.segment_amplitude_scale_m};
  for (double value : option_values) {
    if (!std::isfinite(value) || value < 0.0)
      return fail(SegmentRefitStatus::INVALID_INPUT,
                  "refit options must be finite and nonnegative");
  }
  if (options_.pose_rotation_scale_rad == 0.0 ||
      options_.pose_translation_scale_m == 0.0 ||
      options_.velocity_scale_mps == 0.0 ||
      options_.accel_bias_scale_mps2 == 0.0 ||
      options_.gyro_bias_scale_radps == 0.0 ||
      options_.segment_amplitude_scale_m == 0.0 ||
      options_.gradient_roundoff_safety_factor == 0.0)
    return fail(SegmentRefitStatus::INVALID_INPUT,
                "state unit scales must be positive");
  gtsam::Values raw_base_values = base_values;
  for (gtsam::Key key : base_values.keys()) {
    if (gtsam::Symbol(key).chr() == 'c') raw_base_values.erase(key);
  }
  if (!GraphAndValuesKeysMatch(base_graph, raw_base_values))
    return fail(SegmentRefitStatus::GRAPH_VALUES_KEY_MISMATCH,
                "base graph and Values keys differ");

  std::unordered_map<std::uint64_t, const ObservationRecord*> records;
  for (const auto& record : plan.observations)
    records.emplace(record.obs_id, &record);
  std::unordered_map<std::uint64_t, size_t> raw_factor_by_obs;
  std::unordered_map<size_t, std::uint64_t> raw_obs_by_factor;
  for (const auto& meta : base_uwb_factor_metadata) {
    if (meta.factor_index >= base_graph.size() || meta.factor_type != "uwb_range" ||
        meta.obs_id == 0 || !raw_factor_by_obs.emplace(meta.obs_id, meta.factor_index).second ||
        !raw_obs_by_factor.emplace(meta.factor_index, meta.obs_id).second) {
      return fail(SegmentRefitStatus::INVALID_INPUT,
                  "invalid or duplicate base UWB FactorMeta");
    }
  }
  for (const auto& record : plan.observations) {
    if (record.valid && record.planned &&
        raw_factor_by_obs.count(record.obs_id) != 1) {
      return fail(SegmentRefitStatus::INVALID_INPUT,
                  "every planned raw obs_id must map to exactly one factor");
    }
  }

  std::unordered_map<std::uint64_t, const SupportSegment*> segment_by_obs;
  std::set<size_t> support_ordinals;
  for (const auto& segment : support.segments) {
    if (segment.segment_ordinal >= support.segments.size())
      return fail(SegmentRefitStatus::INVALID_INPUT,
                  "invalid segment ordinal");
    if (!support_ordinals.insert(segment.segment_ordinal).second)
      return fail(SegmentRefitStatus::INVALID_INPUT,
                  "duplicate segment ordinal");
    for (std::uint64_t obs_id : segment.obs_ids) {
      if (!records.count(obs_id) || !raw_factor_by_obs.count(obs_id) ||
          !segment_by_obs.emplace(obs_id, &segment).second)
        return fail(SegmentRefitStatus::INVALID_INPUT,
                    "support obs_id is invalid, duplicated, or has no raw factor");
    }
  }
  for (size_t ordinal : accepted_segment_ordinals) {
    if (!support_ordinals.count(ordinal))
      return fail(SegmentRefitStatus::INVALID_INPUT,
                  "accepted segment ordinal is outside frozen support");
  }
  const auto anchors = AnchorMap(cfg);

  result.graph.reserve(base_graph.size());
  result.factor_metadata.reserve(base_graph.size());
  for (size_t index = 0; index < base_graph.size(); ++index) {
    RefitFactorMeta output_meta;
    const auto raw_it = raw_obs_by_factor.find(index);
    if (raw_it == raw_obs_by_factor.end()) {
      output_meta.factor_index = result.graph.size();
      result.graph.add(base_graph.at(index));
      output_meta.factor_type = "preserved_non_uwb";
      output_meta.keys.assign(base_graph.at(index)->keys().begin(),
                              base_graph.at(index)->keys().end());
    } else {
      output_meta.obs_id = raw_it->second;
      const auto& record = *records.at(raw_it->second);
      const auto segment_it = segment_by_obs.find(raw_it->second);
      if (segment_it == segment_by_obs.end()) {
        output_meta.factor_index = result.graph.size();
        result.graph.add(base_graph.at(index));
        output_meta.factor_type = "uwb_range";
        output_meta.keys.assign(base_graph.at(index)->keys().begin(),
                                base_graph.at(index)->keys().end());
      } else {
        const SupportSegment& segment = *segment_it->second;
        if (!accepted_segment_ordinals.count(segment.segment_ordinal)) {
          // Frozen suppressed candidate: it is intentionally absent, not
          // reclassified as a noncandidate raw range.
          continue;
        }
        const auto anchor_it = anchors.find(record.anchor_id);
        if (anchor_it == anchors.end())
          return fail(SegmentRefitStatus::INVALID_INPUT,
                      "support observation has no anchor");
        const auto factor = MakeSegmentUwbFactor(
            X(record.keyframe_id), C(segment.segment_ordinal), anchor_it->second,
            cfg.lever_arm_init, record.raw_range, record.nominal_sigma,
            FixedBetaForLink(cfg, record.tag_id, record.anchor_id));
        output_meta.factor_index = result.graph.size();
        result.graph.add(factor);
        output_meta.factor_type = "uwb_segment_range";
        output_meta.segment_id = segment.segment_id;
        output_meta.keys.assign(factor->keys().begin(), factor->keys().end());
      }
    }
    result.factor_metadata.push_back(std::move(output_meta));
  }

  result.values = base_values;
  for (gtsam::Key key : base_values.keys()) {
    if (gtsam::Symbol(key).chr() == 'c') result.values.erase(key);
  }
  for (const auto& segment : support.segments) {
    if (!accepted_segment_ordinals.count(segment.segment_ordinal)) continue;
    const gtsam::Key key = C(segment.segment_ordinal);
    const double initial_amplitude = base_values.exists(key)
                                         ? base_values.at<double>(key)
                                         : 0.0;
    result.values.insert<double>(key, initial_amplitude);
  }
  if (!GraphAndValuesKeysMatch(result.graph, result.values))
    return fail(SegmentRefitStatus::GRAPH_VALUES_KEY_MISMATCH,
                "joint graph and Values keys differ after reconstruction");
  bool selected_values_finite = true;
  for (size_t ordinal : accepted_segment_ordinals) {
    selected_values_finite = selected_values_finite &&
        result.values.exists(C(ordinal)) &&
        std::isfinite(result.values.at<double>(C(ordinal)));
  }
  if (!RefitValuesFinite(result.values, plan.keyframes.size(), 0) ||
      !selected_values_finite)
    return fail(SegmentRefitStatus::NONFINITE_VALUE,
                "initial joint Values are missing or nonfinite");

  // Cache replay rebuilds and audits the exact factor topology around the
  // restored Stage-2 Values, but must never execute the Stage-2 optimizer.
  if (!execute_optimization) {
    for (const auto& segment : support.segments) {
      if (!accepted_segment_ordinals.count(segment.segment_ordinal)) continue;
      SegmentEstimate estimate;
      estimate.segment_id = segment.segment_id;
      estimate.segment_ordinal = segment.segment_ordinal;
      estimate.amplitude_key = C(segment.segment_ordinal);
      estimate.tag_id = segment.tag_id;
      estimate.anchor_id = segment.anchor_id;
      estimate.observation_count = segment.observation_count;
      estimate.start_time = segment.start_time;
      estimate.end_time = segment.end_time;
      estimate.duration = segment.duration;
      estimate.amplitude_m = result.values.at<double>(estimate.amplitude_key);
      estimate.boundary =
          estimate.amplitude_m <= options_.boundary_epsilon_m;
      estimate.short_support_debug = segment.short_support_debug;
      result.segments.push_back(std::move(estimate));
    }
    result.status = SegmentRefitStatus::CONVERGED;
    result.reason = "RESTORED_STAGE2_CACHE_NO_OPTIMIZATION";
    return result;
  }

  // An empty automatic partition is not a shortcut. Re-optimize the shared
  // raw graph with no C(s), L1, or TV terms and export this Stage-2 state only
  // if the navigation solve itself succeeds.
  if (accepted_segment_ordinals.empty()) {
    if (development_request && development_request->allow_inexact_handoff)
      return fail(SegmentRefitStatus::INVALID_INPUT,
                  "DEVELOPMENT_NO_C_FORBIDS_INEXACT_HANDOFF");
    CheckedLmOptions lm_options;
    lm_options.max_iterations = options_.lm_max_iterations;
    lm_options.relative_tolerance = options_.lm_relative_tolerance;
    lm_options.absolute_tolerance = options_.lm_absolute_tolerance;
    if (development_request) {
      lm_options.policy = ConditionalLmPolicy::
          GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
      lm_options.navigation_scales = RefitNavigationScales(options_);
      lm_options.navigation_stationarity_tolerance_objective =
          options_.navigation_stationarity_tolerance_objective;
      lm_options.gradient_roundoff_safety_factor =
          options_.gradient_roundoff_safety_factor;
    }
    double previous_objective = result.graph.error(result.values);
    if (!std::isfinite(previous_objective))
      return fail(SegmentRefitStatus::NONFINITE_VALUE,
                  "raw Stage-2 initial objective is nonfinite");
    for (size_t outer = 1; outer <= options_.max_refit_iterations; ++outer) {
      const gtsam::Values before_values = result.values;
      std::vector<DevelopmentRefitRangeConstant> development_ranges;
      if (development_request) {
        std::set<size_t> range_factor_indices;
        std::set<std::uint64_t> range_obs_ids;
        for (size_t index = 0; index < result.factor_metadata.size(); ++index) {
          const auto& meta = result.factor_metadata[index];
          if (meta.factor_type != "uwb_range") continue;
          const auto record_it = records.find(meta.obs_id);
          if (record_it == records.end() || meta.factor_index != index ||
              meta.keys !=
                  std::vector<gtsam::Key>{X(record_it->second->keyframe_id)})
            return fail(SegmentRefitStatus::INVALID_INPUT,
                        "development no-C range metadata is inconsistent");
          const auto& record = *record_it->second;
          const auto anchor_it = anchors.find(record.anchor_id);
          if (anchor_it == anchors.end())
            return fail(SegmentRefitStatus::INVALID_INPUT,
                        "development no-C range has no configured anchor");
          const double fixed_beta =
              FixedBetaForLink(cfg, record.tag_id, record.anchor_id);
          const auto pose =
              before_values.at<gtsam::Pose3>(X(record.keyframe_id));
          const double geometric =
              (pose.transformFrom(cfg.lever_arm_init) - anchor_it->second)
                  .norm();
          const double expected_residual =
              UwbResidual(geometric, record.raw_range, fixed_beta, 0.0);
          const auto expected_factor = MakeUwbFactor(
              X(record.keyframe_id), 0, 0, 0, anchor_it->second,
              cfg.lever_arm_init, record.raw_range, record.nominal_sigma,
              false, false, false, fixed_beta);
          if (!RangeFactorMatchesExpected(
                  result.graph.at(index), expected_factor, before_values,
                  record.raw_range, record.nominal_sigma, expected_residual) ||
              !range_factor_indices.insert(index).second ||
              !range_obs_ids.insert(record.obs_id).second)
            return fail(SegmentRefitStatus::INVALID_INPUT,
                        "development no-C range does not match raw graph");
          DevelopmentRefitRangeConstant range;
          range.factor_index = index;
          range.pose_key = X(record.keyframe_id);
          range.anchor = anchor_it->second;
          range.lever = cfg.lever_arm_init;
          range.measurement = record.raw_range;
          range.sigma = record.nominal_sigma;
          range.conditional_beta = fixed_beta;
          range.obs_id = record.obs_id;
          range.candidate = false;
          range.fixed_beta = fixed_beta;
          range.segment_amplitude = 0.0;
          range.expected_unwhitened_residual = expected_residual;
          development_ranges.push_back(std::move(range));
        }
        size_t expected_range_count = 0;
        for (const auto& meta : result.factor_metadata)
          expected_range_count += meta.factor_type == "uwb_range";
        if (development_ranges.size() != expected_range_count)
          return fail(SegmentRefitStatus::INVALID_INPUT,
                      "development no-C range metadata is incomplete");
      }
      const auto lm = development_request
          ? development_request->conditional_navigation(
                outer, result.graph, before_values, lm_options,
                development_ranges)
          : RunCheckedConditionalLm(result.graph, before_values, lm_options);
      if (development_request &&
          (lm.inexact_handoff.enabled || lm.inexact_handoff.qualified ||
           lm.inexact_handoff.handoff_count != 0))
        return fail(SegmentRefitStatus::INVALID_INPUT,
                    "DEVELOPMENT_NO_C_RETURNED_INEXACT_HANDOFF");
      if (!lm.converged)
        return fail(SegmentRefitStatus::CONDITIONAL_LM_FAILED, lm.reason);
      if (!GraphAndValuesKeysMatch(result.graph, lm.values) ||
          !RefitValuesFinite(lm.values, plan.keyframes.size(), 0))
        return fail(SegmentRefitStatus::GRAPH_VALUES_KEY_MISMATCH,
                    "raw Stage-2 graph and optimized Values differ");
      const double objective = result.graph.error(lm.values);
      const double scaled_step = MaxScaledValuesStep(
          before_values, lm.values, RefitNavigationScales(options_),
          options_.segment_amplitude_scale_m);
      if (!std::isfinite(objective) || !std::isfinite(scaled_step))
        return fail(SegmentRefitStatus::NONFINITE_VALUE,
                    "raw Stage-2 objective or scaled step is nonfinite");
      const double allowance = Binary64ObjectiveIncreaseAllowance(
          previous_objective, objective);
      if (objective > previous_objective + allowance)
        return fail(SegmentRefitStatus::OBJECTIVE_INCREASE,
                    "raw Stage-2 objective increased beyond roundoff allowance");
      const double relative_change =
          std::abs(previous_objective - objective) /
          std::max(1.0, std::abs(previous_objective));
      const auto stationarity =
          AuditNavigationStationarity(result.graph, lm.values, options_);
      if (stationarity.status != SegmentRefitStatus::CONVERGED)
        return fail(stationarity.status, stationarity.reason);

      RefitIteration trace;
      trace.outer_iteration = outer;
      trace.conditional_lm_iterations = lm.iterations;
      trace.conditional_lm_inner_iterations = lm.inner_iterations;
      trace.conditional_lm_lambda = lm.lambda;
      trace.objective_before = previous_objective;
      trace.objective_after = objective;
      trace.relative_objective_change = relative_change;
      trace.scaled_state_step = scaled_step;
      trace.max_kkt_violation = 0.0;  // No constrained C block is applicable.
      trace.max_pose_rotation_gradient_objective_per_rad =
          stationarity.max_pose_rotation_gradient_objective_per_rad;
      trace.max_pose_translation_gradient_objective_per_m =
          stationarity.max_pose_translation_gradient_objective_per_m;
      trace.max_velocity_gradient_objective_per_mps =
          stationarity.max_velocity_gradient_objective_per_mps;
      trace.max_accel_bias_gradient_objective_per_mps2 =
          stationarity.max_accel_bias_gradient_objective_per_mps2;
      trace.max_gyro_bias_gradient_objective_per_radps =
          stationarity.max_gyro_bias_gradient_objective_per_radps;
      trace.max_scaled_navigation_gradient_objective =
          stationarity.max_scaled_navigation_gradient_objective;
      trace.navigation_gradient_roundoff_allowance_objective =
          stationarity.roundoff_allowance_objective;
      trace.navigation_stationarity_tolerance_objective =
          options_.navigation_stationarity_tolerance_objective;
      trace.allowed_objective_increase = allowance;
      trace.objective_ok =
          relative_change <= options_.relative_objective_tolerance;
      trace.step_ok = scaled_step <= options_.scaled_step_tolerance;
      trace.kkt_ok = true;  // Explicitly not applicable for empty partition.
      trace.navigation_stationarity_ok = stationarity.stationarity_ok;
      result.iterations.push_back(trace);
      if (development_request && development_request->outer_observer)
        development_request->outer_observer(trace);
      result.values = lm.values;
      previous_objective = objective;
      if (trace.objective_ok && trace.step_ok && trace.kkt_ok &&
          trace.navigation_stationarity_ok) {
        result.status = SegmentRefitStatus::CONVERGED;
        result.reason = support.segments.empty()
            ? "NO_CANDIDATES_RAW_STAGE2_ALL_APPLICABLE_STOP_CONDITIONS_SATISFIED"
            : "ALL_CANDIDATES_SUPPRESSED_RAW_STAGE2_ALL_APPLICABLE_STOP_CONDITIONS_SATISFIED";
        return result;
      }
    }
    return fail(SegmentRefitStatus::MAX_REFIT_ITERATIONS,
                "raw Stage-2 applicable stop conditions were not all satisfied");
  }

  auto update_segment_records = [&](const gtsam::Values& values,
                                    std::vector<SegmentEstimate>* estimates,
                                    double* max_kkt) -> SegmentRefitStatus {
    estimates->clear();
    *max_kkt = 0.0;
    for (const auto& segment : support.segments) {
      if (!accepted_segment_ordinals.count(segment.segment_ordinal)) continue;
      double gradient = 0.0;
      for (std::uint64_t obs_id : segment.obs_ids) {
        const auto& record = *records.at(obs_id);
        const auto pose = values.at<gtsam::Pose3>(X(record.keyframe_id));
        const gtsam::Point3 antenna = pose.transformFrom(cfg.lever_arm_init);
        const double geometric =
            (gtsam::Vector3(antenna) -
             gtsam::Vector3(anchors.at(record.anchor_id)))
                .norm();
        const double residual = SegmentUwbResidual(
            geometric, record.raw_range,
            FixedBetaForLink(cfg, record.tag_id, record.anchor_id),
            values.at<double>(C(segment.segment_ordinal)));
        const double weight = 1.0 / (record.nominal_sigma * record.nominal_sigma);
        gradient += weight * residual;
      }
      const double amplitude = values.at<double>(C(segment.segment_ordinal));
      if (!std::isfinite(gradient) || !std::isfinite(amplitude))
        return SegmentRefitStatus::NONFINITE_VALUE;
      const bool boundary = amplitude <= options_.boundary_epsilon_m;
      const double violation = boundary ? std::max(0.0, -gradient)
                                        : std::abs(gradient);
      *max_kkt = std::max(*max_kkt, violation);
      SegmentEstimate estimate;
      estimate.segment_id = segment.segment_id;
      estimate.segment_ordinal = segment.segment_ordinal;
      estimate.amplitude_key = C(segment.segment_ordinal);
      estimate.tag_id = segment.tag_id;
      estimate.anchor_id = segment.anchor_id;
      estimate.observation_count = segment.observation_count;
      estimate.start_time = segment.start_time;
      estimate.end_time = segment.end_time;
      estimate.duration = segment.duration;
      estimate.amplitude_m = amplitude;
      estimate.gradient_objective_per_m = gradient;
      estimate.kkt_violation = violation;
      estimate.boundary = boundary;
      estimate.short_support_debug = segment.short_support_debug;
      estimates->push_back(std::move(estimate));
    }
    return SegmentRefitStatus::CONVERGED;
  };

  double previous_objective = result.graph.error(result.values);
  if (!std::isfinite(previous_objective))
    return fail(SegmentRefitStatus::NONFINITE_VALUE,
                "initial joint objective is nonfinite");

  for (size_t outer = 1; outer <= options_.max_refit_iterations; ++outer) {
    const gtsam::Values before = result.values;
    gtsam::NonlinearFactorGraph conditional_graph;
    std::vector<DevelopmentRefitRangeConstant> development_ranges;
    std::set<size_t> development_range_factor_indices;
    std::set<std::uint64_t> development_range_obs_ids;
    conditional_graph.reserve(base_graph.size());
    for (size_t index = 0; index < result.graph.size(); ++index) {
      const auto& meta = result.factor_metadata[index];
      if (meta.factor_type != "uwb_segment_range" &&
          meta.factor_type != "uwb_range") {
        conditional_graph.add(result.graph.at(index));
        continue;
      }
      const auto& record = *records.at(meta.obs_id);
      const bool candidate = meta.factor_type == "uwb_segment_range";
      const double fixed_beta =
          FixedBetaForLink(cfg, record.tag_id, record.anchor_id);
      double segment_amplitude = 0.0;
      if (candidate) {
        const auto segment_it = segment_by_obs.find(meta.obs_id);
        if (segment_it == segment_by_obs.end() ||
            !before.exists(C(segment_it->second->segment_ordinal)))
          return fail(SegmentRefitStatus::INVALID_INPUT,
                      "candidate range has no live segment amplitude");
        segment_amplitude =
            before.at<double>(C(segment_it->second->segment_ordinal));
      }
      const double conditional_beta = fixed_beta + segment_amplitude;
      const auto anchor_it = anchors.find(record.anchor_id);
      if (anchor_it == anchors.end())
        return fail(SegmentRefitStatus::INVALID_INPUT,
                    "conditional range has no configured anchor");
      const auto pose = before.at<gtsam::Pose3>(X(record.keyframe_id));
      const double geometric =
          (pose.transformFrom(cfg.lever_arm_init) - anchor_it->second).norm();
      const double expected_residual = UwbResidual(
          geometric, record.raw_range, conditional_beta, 0.0);
      const auto conditional_factor = MakeUwbFactor(
          X(record.keyframe_id), 0, 0, 0, anchors.at(record.anchor_id),
          cfg.lever_arm_init, record.raw_range, record.nominal_sigma, false,
          false, false, conditional_beta);
      if (meta.keys != std::vector<gtsam::Key>{X(record.keyframe_id)} &&
          meta.factor_type == "uwb_range")
        return fail(SegmentRefitStatus::INVALID_INPUT,
                    "reference range metadata pose key is inconsistent");
      // Reference factors were copied from the raw base graph into the joint
      // graph.  Bind them back to the immutable observation/calibration before
      // replacing them with the conditional factor.  Candidate factors carry
      // a live C key, so their conditional form is checked after construction.
      if (development_request && !candidate &&
          !RangeFactorMatchesExpected(result.graph.at(index),
                                      conditional_factor, before,
                                      record.raw_range, record.nominal_sigma,
                                      expected_residual))
        return fail(SegmentRefitStatus::INVALID_INPUT,
                    "reference range factor does not match plan/calibration");
      conditional_graph.add(conditional_factor);
      if (development_request) {
        DevelopmentRefitRangeConstant range;
        range.factor_index = conditional_graph.size() - 1;
        range.pose_key = X(record.keyframe_id);
        range.anchor = anchor_it->second;
        range.lever = cfg.lever_arm_init;
        range.measurement = record.raw_range;
        range.sigma = record.nominal_sigma;
        range.conditional_beta = conditional_beta;
        range.obs_id = record.obs_id;
        range.candidate = candidate;
        range.fixed_beta = fixed_beta;
        range.segment_amplitude = segment_amplitude;
        range.expected_unwhitened_residual = expected_residual;
        if (!development_range_factor_indices.insert(range.factor_index).second ||
            !development_range_obs_ids.insert(range.obs_id).second)
          return fail(SegmentRefitStatus::INVALID_INPUT,
                      "conditional range metadata is duplicated");
        development_ranges.push_back(std::move(range));
      }
    }
    gtsam::Values conditional_initial = before;
    for (size_t ordinal : accepted_segment_ordinals)
      conditional_initial.erase(C(ordinal));
    if (!GraphAndValuesKeysMatch(conditional_graph, conditional_initial))
      return fail(SegmentRefitStatus::GRAPH_VALUES_KEY_MISMATCH,
                  "conditional graph and Values keys differ");
    if (development_request) {
      size_t expected_range_count = 0;
      for (const auto& meta : result.factor_metadata)
        expected_range_count += meta.factor_type == "uwb_range" ||
                                meta.factor_type == "uwb_segment_range";
      if (development_ranges.size() != expected_range_count)
        return fail(SegmentRefitStatus::INVALID_INPUT,
                    "conditional range metadata does not cover every range");
      for (const auto& range : development_ranges) {
        if (range.factor_index >= conditional_graph.size() ||
            conditional_graph.at(range.factor_index)->keys() !=
                gtsam::KeyVector{range.pose_key} ||
            !Binary64Equal(range.conditional_beta,
                           range.fixed_beta + range.segment_amplitude) ||
            !RangeFactorMatchesExpected(
                conditional_graph.at(range.factor_index),
                conditional_graph.at(range.factor_index), conditional_initial,
                range.measurement, range.sigma,
                range.expected_unwhitened_residual))
        {
          std::ostringstream detail;
          detail << std::setprecision(17)
                 << "conditional range metadata does not match graph; obs_id="
                 << range.obs_id << "; factor=" << range.factor_index
                 << "; outer=" << outer << "; beta=" << range.fixed_beta
                 << "; c=" << range.segment_amplitude
                 << "; conditional_beta=" << range.conditional_beta
                 << "; expected_residual=" << range.expected_unwhitened_residual;
          if (range.factor_index < conditional_graph.size()) {
            const auto factor = boost::dynamic_pointer_cast<
                gtsam::ExpressionFactor<double>>(
                    conditional_graph.at(range.factor_index));
            if (factor) {
              detail << "; measured=" << factor->measured()
                     << "; expected_measurement=" << range.measurement
                     << "; actual_residual="
                     << factor->unwhitenedError(conditional_initial)[0]
                     << "; pose=" << gtsam::DefaultKeyFormatter(range.pose_key)
                     << "; sigma=" << range.sigma;
            }
          }
          return fail(SegmentRefitStatus::INVALID_INPUT, detail.str());
        }
      }
    }
    CheckedLmOptions lm_options;
    lm_options.max_iterations = options_.lm_max_iterations;
    lm_options.relative_tolerance = options_.lm_relative_tolerance;
    lm_options.absolute_tolerance = options_.lm_absolute_tolerance;
    if (development_request) {
      lm_options.policy = ConditionalLmPolicy::
          GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
      lm_options.navigation_scales = RefitNavigationScales(options_);
      lm_options.navigation_stationarity_tolerance_objective =
          options_.navigation_stationarity_tolerance_objective;
      lm_options.gradient_roundoff_safety_factor =
          options_.gradient_roundoff_safety_factor;
      if (development_request->allow_inexact_handoff &&
          accepted_segment_ordinals.empty())
        return fail(SegmentRefitStatus::INVALID_INPUT,
                    "R03_INEXACT_HANDOFF_REQUIRES_NONEMPTY_LIVE_C");
      if (development_request->allow_inexact_handoff)
        lm_options.inexact_handoff_scaled_step_tolerance =
            options_.scaled_step_tolerance;
    }
    const CheckedLmResult lm = development_request
        ? development_request->conditional_navigation(
              outer, conditional_graph, conditional_initial, lm_options,
              development_ranges)
        : RunCheckedConditionalLm(conditional_graph, conditional_initial,
                                  lm_options);
    const bool inexact_handoff =
        !lm.converged && lm.reason == "INNER_NUMERICAL_STALL_INEXACT" &&
        lm.inexact_handoff.qualified &&
        lm.inexact_handoff.handoff_count == 1 && development_request &&
        development_request->allow_inexact_handoff &&
        !accepted_segment_ordinals.empty();
    if (!lm.converged && !inexact_handoff)
      return fail(SegmentRefitStatus::CONDITIONAL_LM_FAILED, lm.reason);
    if (inexact_handoff)
      result.inexact_handoffs.push_back(lm.inexact_handoff);

    gtsam::Values after = lm.values;
    for (const auto& segment : support.segments) {
      if (!accepted_segment_ordinals.count(segment.segment_ordinal)) continue;
      std::vector<double> raw_ranges;
      std::vector<double> geometry_plus_beta;
      std::vector<double> nominal_sigmas;
      for (std::uint64_t obs_id : segment.obs_ids) {
        const auto& record = *records.at(obs_id);
        const auto pose = after.at<gtsam::Pose3>(X(record.keyframe_id));
        const gtsam::Point3 antenna = pose.transformFrom(cfg.lever_arm_init);
        const double geometric =
            (gtsam::Vector3(antenna) -
             gtsam::Vector3(anchors.at(record.anchor_id)))
                .norm();
        raw_ranges.push_back(record.raw_range);
        geometry_plus_beta.push_back(
            geometric +
            FixedBetaForLink(cfg, record.tag_id, record.anchor_id));
        nominal_sigmas.push_back(record.nominal_sigma);
      }
      const auto update = ComputeNonnegativeSegmentAmplitude(
          raw_ranges, geometry_plus_beta, nominal_sigmas);
      if (!update.valid) {
        return fail(
            SegmentRefitStatus::NONPOSITIVE_OR_NONFINITE_DENOMINATOR,
            "segment closed-form update has invalid numerator/denominator");
      }
      after.insert<double>(C(segment.segment_ordinal),
                           update.amplitude_m);
    }
    if (!GraphAndValuesKeysMatch(result.graph, after))
      return fail(SegmentRefitStatus::GRAPH_VALUES_KEY_MISMATCH,
                  "updated joint graph and Values keys differ");
    bool selected_after_finite = true;
    for (size_t ordinal : accepted_segment_ordinals) {
      selected_after_finite = selected_after_finite && after.exists(C(ordinal)) &&
                              std::isfinite(after.at<double>(C(ordinal)));
    }
    if (!RefitValuesFinite(after, plan.keyframes.size(), 0) ||
        !selected_after_finite)
      return fail(SegmentRefitStatus::NONFINITE_VALUE,
                  "updated joint Values are missing or nonfinite");

    const double objective = result.graph.error(after);
    const double scaled_step = MaxScaledValuesStep(
        before, after, RefitNavigationScales(options_),
        options_.segment_amplitude_scale_m);
    if (!std::isfinite(objective) || !std::isfinite(scaled_step))
      return fail(SegmentRefitStatus::NONFINITE_VALUE,
                  "joint objective or scaled step is nonfinite");
    const double increase_tolerance =
        Binary64ObjectiveIncreaseAllowance(previous_objective, objective);
    if (objective > previous_objective + increase_tolerance)
      return fail(SegmentRefitStatus::OBJECTIVE_INCREASE,
                  "joint objective increased beyond binary64 roundoff scale");
    const double relative_change =
        std::abs(previous_objective - objective) /
        std::max(1.0, std::abs(previous_objective));
    double max_kkt = 0.0;
    std::vector<SegmentEstimate> estimates;
    const auto estimate_status =
        update_segment_records(after, &estimates, &max_kkt);
    if (estimate_status != SegmentRefitStatus::CONVERGED)
      return fail(estimate_status, "segment KKT audit is nonfinite");
    const auto stationarity =
        AuditNavigationStationarity(result.graph, after, options_);
    if (stationarity.status != SegmentRefitStatus::CONVERGED)
      return fail(stationarity.status, stationarity.reason);

    RefitIteration trace;
    trace.outer_iteration = outer;
    trace.conditional_lm_iterations = lm.iterations;
    trace.conditional_lm_inner_iterations = lm.inner_iterations;
    trace.conditional_lm_lambda = lm.lambda;
    trace.objective_before = previous_objective;
    trace.objective_after = objective;
    trace.relative_objective_change = relative_change;
    trace.scaled_state_step = scaled_step;
    trace.max_kkt_violation = max_kkt;
    trace.max_pose_rotation_gradient_objective_per_rad =
        stationarity.max_pose_rotation_gradient_objective_per_rad;
    trace.max_pose_translation_gradient_objective_per_m =
        stationarity.max_pose_translation_gradient_objective_per_m;
    trace.max_velocity_gradient_objective_per_mps =
        stationarity.max_velocity_gradient_objective_per_mps;
    trace.max_accel_bias_gradient_objective_per_mps2 =
        stationarity.max_accel_bias_gradient_objective_per_mps2;
    trace.max_gyro_bias_gradient_objective_per_radps =
        stationarity.max_gyro_bias_gradient_objective_per_radps;
    trace.max_scaled_navigation_gradient_objective =
        stationarity.max_scaled_navigation_gradient_objective;
    trace.navigation_gradient_roundoff_allowance_objective =
        stationarity.roundoff_allowance_objective;
    trace.navigation_stationarity_tolerance_objective =
        options_.navigation_stationarity_tolerance_objective;
    trace.allowed_objective_increase = increase_tolerance;
    trace.objective_ok =
        relative_change <= options_.relative_objective_tolerance;
    trace.step_ok = scaled_step <= options_.scaled_step_tolerance;
    trace.kkt_ok = max_kkt <= options_.projected_gradient_tolerance;
    trace.navigation_stationarity_ok = stationarity.stationarity_ok;
    trace.conditional_inexact_handoff = inexact_handoff;
    trace.conditional_inner_status = lm.reason;
    result.iterations.push_back(trace);
    if (development_request && development_request->outer_observer)
      development_request->outer_observer(trace);
    result.values = std::move(after);
    result.segments = std::move(estimates);
    previous_objective = objective;
    if (trace.objective_ok && trace.step_ok && trace.kkt_ok &&
        trace.navigation_stationarity_ok) {
      result.status = SegmentRefitStatus::CONVERGED;
      result.reason = "ALL_JOINT_STOP_CONDITIONS_SATISFIED";
      return result;
    }
  }
  return fail(SegmentRefitStatus::MAX_REFIT_ITERATIONS,
              "joint stop conditions were not all satisfied");
}

SegmentRefitResult SegmentRefitter::RunFrozenCandidatePolicy(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& stage2_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const SupportPartition& frozen_full_support,
    const std::set<size_t>& accepted_segment_ordinals,
    bool execute_optimization) const {
  return RunFrozenCandidatePolicyImpl(
      base_graph, stage2_values, base_uwb_factor_metadata, plan, cfg,
      frozen_full_support, accepted_segment_ordinals, execute_optimization,
      nullptr);
}

SegmentRefitResult SegmentRefitter::RunDevelopmentFrozenCandidatePolicy(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& stage2_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const SupportPartition& frozen_full_support,
    const std::set<size_t>& accepted_segment_ordinals,
    const DevelopmentStage2Request* development_request) const {
  return RunFrozenCandidatePolicyImpl(
      base_graph, stage2_values, base_uwb_factor_metadata, plan, cfg,
      frozen_full_support, accepted_segment_ordinals, true,
      development_request);
}

SegmentRefitResult SegmentRefitter::Run(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& base_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const SupportPartition& support) const {
  std::set<size_t> all_segments;
  for (const auto& segment : support.segments)
    all_segments.insert(segment.segment_ordinal);
  return RunFrozenCandidatePolicy(base_graph, base_values,
                                  base_uwb_factor_metadata, plan, cfg,
                                  support, all_segments);
}

SegmentRefitResult SegmentRefitter::RunDevelopmentStage2(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& base_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const SupportPartition& support,
    const DevelopmentStage2Request* development_request) const {
  std::set<size_t> all_segments;
  for (const auto& segment : support.segments)
    all_segments.insert(segment.segment_ordinal);
  return RunFrozenCandidatePolicyImpl(
      base_graph, base_values, base_uwb_factor_metadata, plan, cfg, support,
      all_segments, true, development_request);
}

SegmentRefitResult SegmentRefitter::Run(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& base_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const OracleSupport& support) const {
  return Run(base_graph, base_values, base_uwb_factor_metadata, plan, cfg,
             ToSupportPartition(support));
}

}  // namespace uifgo
