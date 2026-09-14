#include "uifgo/common_initializer.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

#include "uifgo/paper_robust_noise.h"

namespace uifgo {
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

constexpr double kInitializationHuberScale = 1.345;
// Initialization acceptance uses the existing physical-unit stationarity
// audit.  This fixed numerical tolerance is intentionally not a scheduling or
// data-selection knob and is not exposed for dataset tuning.
constexpr double kInitializationStationarityTolerance = 1e-5;
constexpr double kProductionCauchyScale = 2.3849;
constexpr double kAlmostAllSilencedFraction = 0.9;

// Initialization-only view of the actual physical CombinedImuFactor with the
// already committed previous state bound as a constant.  The residual, noise
// model, PIM and current-state Jacobian blocks are delegated verbatim to the
// physical factor; only the fixed boundary keys are removed from the local
// optimization variables.
class FixedBoundaryCombinedImuFactor final
    : public gtsam::NoiseModelFactor3<
          gtsam::Pose3, gtsam::Vector3,
          gtsam::imuBias::ConstantBias> {
 public:
  using Base = gtsam::NoiseModelFactor3<
      gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>;

  FixedBoundaryCombinedImuFactor(
      const gtsam::CombinedImuFactor::shared_ptr& physical,
      gtsam::Key pose_j_key, gtsam::Key velocity_j_key,
      gtsam::Key bias_j_key, const gtsam::Pose3& pose_i,
      const gtsam::Vector3& velocity_i,
      const gtsam::imuBias::ConstantBias& bias_i)
      : Base(physical->noiseModel(), pose_j_key, velocity_j_key, bias_j_key),
        physical_(physical),
        pose_i_(pose_i),
        velocity_i_(velocity_i),
        bias_i_(bias_i) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(
        new FixedBoundaryCombinedImuFactor(*this));
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose_j, const gtsam::Vector3& velocity_j,
      const gtsam::imuBias::ConstantBias& bias_j,
      boost::optional<gtsam::Matrix&> pose_j_jacobian = boost::none,
      boost::optional<gtsam::Matrix&> velocity_j_jacobian = boost::none,
      boost::optional<gtsam::Matrix&> bias_j_jacobian = boost::none) const
      override {
    return physical_->evaluateError(
        pose_i_, velocity_i_, pose_j, velocity_j, bias_i_, bias_j,
        boost::none, boost::none, pose_j_jacobian, velocity_j_jacobian,
        boost::none, bias_j_jacobian);
  }

 private:
  gtsam::CombinedImuFactor::shared_ptr physical_;
  gtsam::Pose3 pose_i_;
  gtsam::Vector3 velocity_i_;
  gtsam::imuBias::ConstantBias bias_i_;
};

bool NavigationValueFinite(const gtsam::Values& values, gtsam::Key key) {
  try {
    const char kind = gtsam::Symbol(key).chr();
    if (kind == 'x') return values.at<gtsam::Pose3>(key).matrix().allFinite();
    if (kind == 'v') return values.at<gtsam::Vector3>(key).allFinite();
    if (kind == 'b') {
      const auto bias = values.at<gtsam::imuBias::ConstantBias>(key);
      return bias.accelerometer().allFinite() && bias.gyroscope().allFinite();
    }
  } catch (...) {
    return false;
  }
  return false;
}

bool NavigationRangeFinite(const gtsam::Values& values, size_t first,
                           size_t last) {
  for (size_t k = first; k <= last; ++k) {
    if (!NavigationValueFinite(values, X(k)) ||
        !NavigationValueFinite(values, V(k)) ||
        !NavigationValueFinite(values, B(k)))
      return false;
  }
  return true;
}

gtsam::NonlinearFactor::shared_ptr HuberInitializationFactor(
    const gtsam::NonlinearFactor::shared_ptr& factor) {
  const auto noise_factor =
      boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
  if (!noise_factor)
    throw std::invalid_argument("initialization UWB factor has no noise model");
  if (boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(
          noise_factor->noiseModel()))
    throw std::invalid_argument(
        "initialization expected an unwrapped physical UWB factor");
  const auto huber =
      gtsam::noiseModel::mEstimator::Huber::Create(kInitializationHuberScale);
  return noise_factor->cloneWithNewNoiseModel(
      boost::make_shared<PaperRobustNoise>(huber,
                                           noise_factor->noiseModel()));
}

std::string FailureAt(const std::string& reason, size_t frontier,
                      double time_s) {
  std::ostringstream out;
  out << reason << ":frontier_state=" << frontier
      << ":frontier_time_s=" << std::setprecision(17) << time_s;
  return out.str();
}

double Quantile(std::vector<double> values, double probability) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double index = probability * static_cast<double>(values.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(index));
  const size_t hi = static_cast<size_t>(std::ceil(index));
  const double alpha = index - static_cast<double>(lo);
  return values[lo] * (1.0 - alpha) + values[hi] * alpha;
}

InitializationSeedQualityAudit AuditLocalSeedQuality(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& entering,
    const gtsam::Values& boundary_values, const gtsam::Values& terminal,
    size_t boundary, size_t frontier,
    size_t uwb_factor_count, double window_duration_s,
    const Config& config) {
  InitializationSeedQualityAudit audit;
  audit.evaluated = true;
  audit.uwb_support_present = uwb_factor_count > 0;
  audit.graph_keys_valid = GraphAndValuesKeysMatch(graph, terminal);
  audit.values_finite = NavigationRangeFinite(terminal, boundary + 1, frontier);
  audit.entering_objective = graph.error(entering);
  audit.terminal_objective = graph.error(terminal);
  audit.objective_finite = std::isfinite(audit.entering_objective) &&
                           std::isfinite(audit.terminal_objective);
  audit.objective_increase_allowance =
      64.0 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, std::abs(audit.entering_objective));
  audit.objective_not_worse = audit.objective_finite &&
      audit.terminal_objective <=
          audit.entering_objective + audit.objective_increase_allowance;
  std::vector<double> entering_uwb_residuals, terminal_uwb_residuals;
  for (const auto& factor : graph) {
    const auto noise_factor =
        boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
    if (noise_factor && factor->keys().size() == 1 &&
        gtsam::Symbol(factor->keys().front()).chr() == 'x') {
      const auto entering_error = noise_factor->unwhitenedError(entering);
      const auto terminal_error = noise_factor->unwhitenedError(terminal);
      if (entering_error.size() == 1 && terminal_error.size() == 1) {
        entering_uwb_residuals.push_back(std::abs(entering_error[0]));
        terminal_uwb_residuals.push_back(std::abs(terminal_error[0]));
      }
    }
  }
  audit.entering_median_abs_uwb_residual_m =
      Quantile(entering_uwb_residuals, 0.5);
  audit.terminal_median_abs_uwb_residual_m =
      Quantile(terminal_uwb_residuals, 0.5);
  audit.uwb_quality_not_catastrophic = audit.uwb_support_present &&
      std::isfinite(audit.entering_median_abs_uwb_residual_m) &&
      std::isfinite(audit.terminal_median_abs_uwb_residual_m) &&
      audit.terminal_median_abs_uwb_residual_m <=
          audit.entering_median_abs_uwb_residual_m + config.max_range;

  audit.position_envelope_m =
      config.max_range + config.v_max * window_duration_s;
  audit.velocity_change_envelope_mps =
      config.v_max + config.max_range /
          config.initialization_progression_horizon_s;
  audit.physically_local = audit.values_finite;
  if (audit.values_finite) {
    const auto boundary_position =
        boundary_values.at<gtsam::Pose3>(X(boundary)).translation();
    const auto boundary_velocity =
        boundary_values.at<gtsam::Vector3>(V(boundary));
    for (size_t k = boundary + 1; k <= frontier; ++k) {
      audit.max_position_from_boundary_m = std::max(
          audit.max_position_from_boundary_m,
          (terminal.at<gtsam::Pose3>(X(k)).translation() -
           boundary_position).norm());
      audit.max_velocity_change_from_boundary_mps = std::max(
          audit.max_velocity_change_from_boundary_mps,
          (terminal.at<gtsam::Vector3>(V(k)) - boundary_velocity).norm());
    }
    audit.physically_local =
        audit.max_position_from_boundary_m <= audit.position_envelope_m &&
        audit.max_velocity_change_from_boundary_mps <=
            audit.velocity_change_envelope_mps;
  }
  audit.accepted = audit.uwb_support_present && audit.graph_keys_valid &&
      audit.values_finite && audit.objective_finite &&
      audit.objective_not_worse && audit.uwb_quality_not_catastrophic &&
      audit.physically_local;
  if (!audit.uwb_support_present) audit.reason = "NO_USABLE_UWB_SUPPORT";
  else if (!audit.graph_keys_valid) audit.reason = "GRAPH_VALUES_KEY_MISMATCH";
  else if (!audit.values_finite) audit.reason = "NONFINITE_WINDOW_VALUES";
  else if (!audit.objective_finite) audit.reason = "NONFINITE_WINDOW_OBJECTIVE";
  else if (!audit.objective_not_worse) audit.reason = "WINDOW_OBJECTIVE_WORSENED";
  else if (!audit.uwb_quality_not_catastrophic)
    audit.reason = "WINDOW_UWB_QUALITY_WORSENED";
  else if (!audit.physically_local) audit.reason = "CATASTROPHIC_LOCAL_STATE_UPDATE";
  else audit.reason = "INITIALIZATION_SEED_QUALITY_ACCEPTED";
  return audit;
}

FullInitializationSeedQualityAudit AuditFullSeedQuality(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const std::vector<double>& state_times_s,
    const std::vector<size_t>& uwb_factor_indices, const Config& config) {
  FullInitializationSeedQualityAudit audit;
  audit.evaluated = true;
  audit.state_count = state_times_s.size();
  audit.uwb_factor_count = uwb_factor_indices.size();
  audit.all_states_finite = !state_times_s.empty() &&
      NavigationRangeFinite(values, 0, state_times_s.size() - 1);

  double anchor_norm_max = 0.0;
  for (const auto& anchor : config.anchors)
    anchor_norm_max = std::max(anchor_norm_max, anchor.pos.norm());
  audit.position_envelope_m = anchor_norm_max + config.max_range;
  audit.velocity_envelope_mps = config.v_max + config.max_range /
      config.initialization_progression_horizon_s;
  audit.motion_finite_and_local = audit.all_states_finite;
  if (audit.all_states_finite) {
    for (size_t k = 0; k < state_times_s.size(); ++k) {
      const auto position = values.at<gtsam::Pose3>(X(k)).translation();
      const auto velocity = values.at<gtsam::Vector3>(V(k));
      audit.max_position_norm_m = k == 0
          ? position.norm() : std::max(audit.max_position_norm_m, position.norm());
      audit.max_velocity_norm_mps = k == 0
          ? velocity.norm() : std::max(audit.max_velocity_norm_mps, velocity.norm());
      if (k) {
        const double displacement =
            (position - values.at<gtsam::Pose3>(X(k - 1)).translation()).norm();
        audit.max_consecutive_displacement_m = k == 1 ? displacement :
            std::max(audit.max_consecutive_displacement_m, displacement);
        const double envelope = config.max_range + config.v_max *
            (state_times_s[k] - state_times_s[k - 1]);
        audit.motion_finite_and_local = audit.motion_finite_and_local &&
            std::isfinite(displacement) && displacement <= envelope;
      }
    }
    audit.motion_finite_and_local = audit.motion_finite_and_local &&
        audit.max_position_norm_m <= audit.position_envelope_m &&
        audit.max_velocity_norm_mps <= audit.velocity_envelope_mps;
  }

  std::vector<double> absolute_residuals, absolute_q, weights;
  audit.all_uwb_predictions_finite = !uwb_factor_indices.empty();
  long double residual_sum = 0.0;
  size_t gt05 = 0, gt01 = 0, lt001 = 0, lt1e3 = 0;
  for (size_t index : uwb_factor_indices) {
    if (index >= graph.size()) {
      audit.all_uwb_predictions_finite = false;
      break;
    }
    const auto factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(
        graph.at(index));
    if (!factor) {
      audit.all_uwb_predictions_finite = false;
      break;
    }
    const auto residual = factor->unwhitenedError(values);
    const auto sigmas = factor->noiseModel()->sigmas();
    if (residual.size() != 1 || sigmas.size() != 1 ||
        !std::isfinite(residual[0]) || !(sigmas[0] > 0.0) ||
        !std::isfinite(sigmas[0])) {
      audit.all_uwb_predictions_finite = false;
      break;
    }
    const double absolute = std::abs(residual[0]);
    const double q = absolute / sigmas[0];
    const double weight = 1.0 / (1.0 + std::pow(q / kProductionCauchyScale, 2));
    if (!std::isfinite(q) || !std::isfinite(weight)) {
      audit.all_uwb_predictions_finite = false;
      break;
    }
    absolute_residuals.push_back(absolute);
    absolute_q.push_back(q);
    weights.push_back(weight);
    residual_sum += absolute;
    gt05 += weight > 0.5;
    gt01 += weight > 0.1;
    lt001 += weight < 0.01;
    lt1e3 += weight < 1e-3;
  }
  if (audit.all_uwb_predictions_finite && !absolute_residuals.empty()) {
    const double n = static_cast<double>(absolute_residuals.size());
    audit.mean_abs_uwb_residual_m = static_cast<double>(residual_sum / n);
    audit.median_abs_uwb_residual_m = Quantile(absolute_residuals, 0.5);
    audit.p95_abs_uwb_residual_m = Quantile(absolute_residuals, 0.95);
    audit.max_abs_uwb_residual_m =
        *std::max_element(absolute_residuals.begin(), absolute_residuals.end());
    audit.median_abs_standardized_residual = Quantile(absolute_q, 0.5);
    audit.p95_abs_standardized_residual = Quantile(absolute_q, 0.95);
    audit.max_abs_standardized_residual =
        *std::max_element(absolute_q.begin(), absolute_q.end());
    audit.cauchy_weight_p05 = Quantile(weights, 0.05);
    audit.cauchy_weight_median = Quantile(weights, 0.5);
    audit.cauchy_weight_p95 = Quantile(weights, 0.95);
    audit.fraction_weight_gt_0_5 = gt05 / n;
    audit.fraction_weight_gt_0_1 = gt01 / n;
    audit.fraction_weight_lt_0_01 = lt001 / n;
    audit.fraction_weight_lt_1e_3 = lt1e3 / n;
  }

  audit.accepted = audit.all_states_finite && audit.motion_finite_and_local &&
      audit.all_uwb_predictions_finite &&
      audit.median_abs_uwb_residual_m <= config.max_range &&
      (1.0 - audit.fraction_weight_gt_0_5) < kAlmostAllSilencedFraction;
  if (!audit.all_states_finite) audit.reason = "NONFINITE_FULL_SEED";
  else if (!audit.motion_finite_and_local) audit.reason = "CATASTROPHIC_FULL_SEED_MOTION";
  else if (!audit.all_uwb_predictions_finite) audit.reason = "NONFINITE_FULL_SEED_UWB";
  else if (!(audit.median_abs_uwb_residual_m <= config.max_range))
    audit.reason = "CATASTROPHIC_FULL_SEED_RESIDUAL";
  else if (!((1.0 - audit.fraction_weight_gt_0_5) <
             kAlmostAllSilencedFraction))
    audit.reason = "PRODUCTION_CAUCHY_ALMOST_ALL_SILENCED";
  else audit.reason = "FULL_INITIALIZATION_SEED_QUALITY_ACCEPTED";
  return audit;
}

}  // namespace

std::string CommonInitializer::Identity(const Config& config) {
  std::ostringstream out;
  out << "PAPER_BOUNDED_FIXED_LAG_CAUSAL_INITIALIZER_V4"
      << ":fixed_lag_horizon_s=" << std::setprecision(17)
      << config.initialization_progression_horizon_s
      << ":uwb_loss=PAPER_STANDARD_ROBUST_LOSS_V2_HUBER_1.345"
      << ":solver=CHECKED_LM_STATIONARITY_CONTINUE_V2"
      << ":stationarity_tolerance_objective=1e-5"
      << ":acceptance=INITIALIZATION_SEED_QUALITY_V1"
      << ":boundary=PREDECESSOR_OF_MAXIMAL_FREE_SUFFIX_FIXED_V2"
      << ":commit=ATOMIC_FREE_WINDOW_V1";
  return out.str();
}

CommonInitializationResult CommonInitializer::Run(
    const gtsam::NonlinearFactorGraph& physical_graph,
    const gtsam::Values& open_loop_values,
    const std::vector<double>& state_times_s,
    const std::vector<size_t>& uwb_factor_indices) const {
  CommonInitializationResult result;
  result.identity = Identity(config_);
  const auto started = std::chrono::steady_clock::now();
  auto finish_failure = [&](const std::string& reason, size_t frontier) {
    result.ok = false;
    result.status = "INITIALIZATION_FAILED";
    result.failure_count = 1;
    result.failing_frontier_state = frontier;
    result.failing_frontier_time_s =
        frontier < state_times_s.size()
            ? state_times_s[frontier]
            : std::numeric_limits<double>::quiet_NaN();
    result.reason = FailureAt(reason, result.failing_frontier_state,
                              result.failing_frontier_time_s);
    result.runtime_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    return result;
  };

  const size_t count = state_times_s.size();
  if (count < 2 || physical_graph.empty() || uwb_factor_indices.empty() ||
      !(config_.initialization_progression_horizon_s > 0.0) ||
      !std::isfinite(config_.initialization_progression_horizon_s))
    return finish_failure("INVALID_COMMON_INITIALIZATION_INPUT", 0);

  for (size_t k = 0; k < count; ++k) {
    if (!std::isfinite(state_times_s[k]) ||
        (k > 0 && !(state_times_s[k] > state_times_s[k - 1])) ||
        !open_loop_values.exists(X(k)) || !open_loop_values.exists(V(k)) ||
        !open_loop_values.exists(B(k)))
      return finish_failure("INVALID_STATE_TIMELINE_OR_KEYS", k);
  }
  if (!NavigationRangeFinite(open_loop_values, 0, 0))
    return finish_failure("NONFINITE_FIRST_STATE", 0);

  std::map<size_t, gtsam::CombinedImuFactor::shared_ptr> imu_by_frontier;
  for (const auto& factor : physical_graph) {
    const auto imu = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
    if (!imu) continue;
    const auto& keys = imu->keys();
    if (keys.size() != 6 || gtsam::Symbol(keys[0]).chr() != 'x' ||
        gtsam::Symbol(keys[2]).chr() != 'x')
      return finish_failure("INVALID_COMBINED_IMU_FACTOR_KEYS", 0);
    const size_t from = gtsam::Symbol(keys[0]).index();
    const size_t to = gtsam::Symbol(keys[2]).index();
    if (to != from + 1 || to >= count || !imu_by_frontier.emplace(to, imu).second)
      return finish_failure("NONCONTIGUOUS_OR_DUPLICATE_IMU_FACTOR", to);
    const double pim_duration = imu->preintegratedMeasurements().deltaTij();
    if (!(pim_duration > 0.0) || !std::isfinite(pim_duration))
      return finish_failure("INVALID_PIM_DURATION", to);
  }
  if (imu_by_frontier.size() + 1 != count)
    return finish_failure("COMBINED_IMU_FACTOR_COUNT_MISMATCH", 0);

  std::map<size_t, std::vector<gtsam::NonlinearFactor::shared_ptr>> uwb_by_state;
  std::set<size_t> unique_uwb_indices;
  for (size_t index : uwb_factor_indices) {
    if (index >= physical_graph.size() || !unique_uwb_indices.insert(index).second)
      return finish_failure("INVALID_OR_DUPLICATE_UWB_FACTOR_INDEX", 0);
    const auto& factor = physical_graph.at(index);
    if (!factor || factor->keys().size() != 1 ||
        gtsam::Symbol(factor->keys().front()).chr() != 'x')
      return finish_failure("UNSUPPORTED_NON_FIXED_UWB_FACTOR", 0);
    const size_t state = gtsam::Symbol(factor->keys().front()).index();
    if (state >= count)
      return finish_failure("UWB_FACTOR_STATE_OUT_OF_RANGE", state);
    uwb_by_state[state].push_back(factor);
  }

  gtsam::Values committed;
  for (gtsam::Key key : open_loop_values.keys()) {
    const gtsam::Symbol symbol(key);
    if ((symbol.chr() == 'x' || symbol.chr() == 'v' || symbol.chr() == 'b') &&
        symbol.index() != 0)
      continue;
    committed.insert(key, open_loop_values.at(key));
  }

  for (size_t frontier = 1; frontier < count; ++frontier) {
    // The lag constrains only the maximal suffix of states that can be
    // reoptimized. Its immediate predecessor is the fixed boundary and may be
    // arbitrarily older than the lag; the physical IMU bridge remains in the
    // local graph. State zero is always the immutable initialization anchor.
    size_t free_start = frontier;
    while (free_start > 0 &&
           state_times_s[frontier] - state_times_s[free_start - 1] <=
               config_.initialization_progression_horizon_s)
      --free_start;
    free_start = std::max<size_t>(1, free_start);
    const size_t boundary = free_start - 1;

    CommonInitializationPrefix prefix;
    prefix.ordinal = result.prefixes.size();
    prefix.start_state = boundary;
    prefix.free_start_state = free_start;
    prefix.frontier_state = frontier;
    prefix.frontier_time_s = state_times_s[frontier];
    prefix.window_duration_s =
        state_times_s[frontier] - state_times_s[boundary];
    prefix.free_window_duration_s =
        state_times_s[frontier] - state_times_s[free_start];
    prefix.boundary_bridge_duration_s =
        state_times_s[free_start] - state_times_s[boundary];
    prefix.window_state_count = frontier - boundary + 1;
    prefix.optimized_state_count = frontier - free_start + 1;
    const auto prefix_started = std::chrono::steady_clock::now();

    gtsam::NonlinearFactorGraph local_graph;
    gtsam::Values local_values;

    try {
      for (size_t k = free_start; k <= frontier; ++k) {
        const auto found = imu_by_frontier.find(k);
        if (found == imu_by_frontier.end())
          return finish_failure("MISSING_IMU_FACTOR", k);
        if (k == free_start) {
          const auto previous_pose = committed.at<gtsam::Pose3>(X(boundary));
          const auto previous_velocity =
              committed.at<gtsam::Vector3>(V(boundary));
          const auto previous_bias =
              committed.at<gtsam::imuBias::ConstantBias>(B(boundary));
          local_graph.add(boost::make_shared<FixedBoundaryCombinedImuFactor>(
              found->second, X(k), V(k), B(k), previous_pose,
              previous_velocity, previous_bias));
        } else {
          local_graph.add(found->second);
        }
        ++prefix.imu_factor_count;
        if (k < frontier) {
          local_values.insert(X(k), committed.at(X(k)));
          local_values.insert(V(k), committed.at(V(k)));
          local_values.insert(B(k), committed.at(B(k)));
        } else {
          const auto previous_pose = committed.at<gtsam::Pose3>(X(k - 1));
          const auto previous_velocity =
              committed.at<gtsam::Vector3>(V(k - 1));
          const auto previous_bias =
              committed.at<gtsam::imuBias::ConstantBias>(B(k - 1));
          const auto prediction =
              found->second->preintegratedMeasurements().predict(
                  gtsam::NavState(previous_pose, previous_velocity),
                  previous_bias);
          local_values.insert(X(k), prediction.pose());
          local_values.insert(V(k), prediction.velocity());
          local_values.insert(B(k), previous_bias);
        }
        const auto uwb = uwb_by_state.find(k);
        if (uwb != uwb_by_state.end()) {
          for (const auto& factor : uwb->second) {
            local_graph.add(HuberInitializationFactor(factor));
            ++prefix.uwb_factor_count;
          }
        }
      }
    } catch (const std::exception& error) {
      return finish_failure(std::string("LOCAL_GRAPH_BUILD_FAILED:") +
                                error.what(),
                            frontier);
    }
    if (!GraphAndValuesKeysMatch(local_graph, local_values) ||
        !NavigationRangeFinite(local_values, free_start, frontier))
      return finish_failure("LOCAL_GRAPH_OR_PREDICTION_INVALID", frontier);
    prefix.initial_objective = local_graph.error(local_values);

    gtsam::Values accepted_values;
    CheckedLmResult solved;
    bool solver_ran = false;
    const auto initial_stationarity = AuditNavigationStationarity(
        local_graph, local_values, NavigationScales(),
        kInitializationStationarityTolerance, 8.0);
    if (initial_stationarity.valid && initial_stationarity.stationary) {
      accepted_values = local_values;
      prefix.lm_reason = "ALREADY_NAVIGATION_STATIONARY";
      prefix.stationarity = initial_stationarity;
    } else {
      CheckedLmOptions options;
      options.max_iterations = config_.lm_max_iter;
      options.relative_tolerance = config_.lm_rel_tol;
      options.absolute_tolerance = config_.lm_abs_tol;
      options.policy = ConditionalLmPolicy::
          GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
      options.navigation_stationarity_tolerance_objective =
          kInitializationStationarityTolerance;
      solved = RunCheckedConditionalLmRetainingTerminalForInitialization(
          local_graph, local_values, options);
      solver_ran = true;
      prefix.lm_calls = solved.convergence.iterate_call_count;
      prefix.accepted_updates = solved.convergence.accepted_update_count;
      prefix.rejected_lambda_trials =
          solved.convergence.rejected_lambda_trial_count;
      prefix.lm_reason = solved.reason;
      prefix.stationarity = solved.last_qualification_stationarity;
      if (!solved.values.empty() && !solved.converged) {
        prefix.stationarity = AuditNavigationStationarity(
            local_graph, solved.values, NavigationScales(),
            kInitializationStationarityTolerance, 8.0);
      }
      if (!solved.values.empty()) accepted_values = solved.values;
    }

    if (!accepted_values.empty()) {
      prefix.seed_quality = AuditLocalSeedQuality(
          local_graph, local_values, committed, accepted_values,
          boundary, frontier,
          prefix.uwb_factor_count, prefix.window_duration_s, config_);
    }
    bool accepted = prefix.seed_quality.accepted;
    if (!accepted && solver_ran && !solved.values.empty()) {
        // One deterministic continuation from the exact last accepted Values
        // may recover a numerically invalid seed candidate. Graph, data,
        // policy and tolerances are identical; a second continuation is never
        // attempted, and stationarity remains diagnostic only.
        ++prefix.retry_count;
        ++result.retry_count;
        CheckedLmOptions options;
        options.max_iterations = config_.lm_max_iter;
        options.relative_tolerance = config_.lm_rel_tol;
        options.absolute_tolerance = config_.lm_abs_tol;
        options.policy = ConditionalLmPolicy::
            GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
        options.navigation_stationarity_tolerance_objective =
            kInitializationStationarityTolerance;
        solved = RunCheckedConditionalLmRetainingTerminalForInitialization(
            local_graph, solved.values, options);
        prefix.lm_calls += solved.convergence.iterate_call_count;
        prefix.accepted_updates += solved.convergence.accepted_update_count;
        prefix.rejected_lambda_trials +=
            solved.convergence.rejected_lambda_trial_count;
        prefix.lm_reason = solved.reason;
        prefix.stationarity = solved.values.empty()
                                  ? solved.last_qualification_stationarity
                                  : AuditNavigationStationarity(
                                        local_graph, solved.values,
                                        NavigationScales(),
                                        kInitializationStationarityTolerance,
                                        8.0);
        if (!solved.values.empty()) {
          accepted_values = solved.values;
          prefix.seed_quality = AuditLocalSeedQuality(
              local_graph, local_values, committed, accepted_values,
              boundary, frontier,
              prefix.uwb_factor_count, prefix.window_duration_s, config_);
        }
        accepted = prefix.seed_quality.accepted;
    }
    if (accepted)
      prefix.terminal_objective = local_graph.error(accepted_values);
    prefix.runtime_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - prefix_started).count();
    result.prefixes.push_back(prefix);

    if (!accepted) {
      std::ostringstream rejected;
      rejected << "PREFIX_SEED_QUALITY_REJECTED:"
               << prefix.seed_quality.reason
               << ":raw_lm_termination=" << prefix.lm_reason
               << ":stationarity_reason=" << prefix.stationarity.reason
               << ":dominant_key=" << prefix.stationarity.dominant_key_name
               << ":max_scaled_gradient=" << std::setprecision(17)
               << prefix.stationarity.max_scaled_gradient_objective;
      return finish_failure(rejected.str(), frontier);
    }

    // Commit the qualified free window atomically. Existing in-lag states are
    // revised; states at or before the fixed boundary remain untouched.
    gtsam::Values next_committed = committed;
    for (size_t k = free_start; k <= frontier; ++k) {
      for (gtsam::Key key : {X(k), V(k), B(k)}) {
        if (next_committed.exists(key))
          next_committed.update(key, accepted_values.at(key));
        else
          next_committed.insert(key, accepted_values.at(key));
      }
    }
    committed = std::move(next_committed);
    ++result.accepted_prefix_count;
  }

  if (!GraphAndValuesKeysMatch(physical_graph, committed) ||
      !NavigationRangeFinite(committed, 0, count - 1))
    return finish_failure("FINAL_INITIAL_VALUES_INVALID", count - 1);
  result.full_seed_quality = AuditFullSeedQuality(
      physical_graph, committed, state_times_s, uwb_factor_indices, config_);
  if (!result.full_seed_quality.accepted)
    return finish_failure(
        "FULL_INITIALIZATION_SEED_QUALITY_REJECTED:" +
            result.full_seed_quality.reason,
        count - 1);
  result.ok = true;
  result.status = "INITIALIZATION_OK";
  result.reason = "BOUNDED_FIXED_LAG_CAUSAL_INITIALIZATION_COMPLETE";
  result.values = committed;
  result.runtime_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
  return result;
}

}  // namespace uifgo
