#include "uifgo/nlos_solver_utils.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <iostream>
#include <memory>
#include <sstream>
#include <streambuf>
#include <set>
#include <typeinfo>
#include <utility>

namespace uifgo {

const char* ConditionalLmPolicyName(ConditionalLmPolicy policy) {
  switch (policy) {
    case ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1:
      return "GTSAM_CHECK_ONLY_V1";
    case ConditionalLmPolicy::GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1:
      return "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1";
    case ConditionalLmPolicy::
        GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2:
      return "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2";
  }
  return "UNKNOWN_CONDITIONAL_LM_POLICY";
}

bool GraphAndValuesKeysMatch(const gtsam::NonlinearFactorGraph& graph,
                             const gtsam::Values& values) {
  std::set<gtsam::Key> graph_keys;
  for (const auto& factor : graph) {
    if (!factor) return false;
    for (gtsam::Key key : factor->keys()) graph_keys.insert(key);
  }
  const auto value_keys = values.keys();
  return graph_keys ==
         std::set<gtsam::Key>(value_keys.begin(), value_keys.end());
}

namespace {

class TeeCaptureStreambuf final : public std::streambuf {
 public:
  explicit TeeCaptureStreambuf(std::streambuf* destination)
      : destination_(destination) {}

  const std::string& captured() const { return captured_; }

 protected:
  int overflow(int character) override {
    if (character == traits_type::eof()) return traits_type::not_eof(character);
    const char value = static_cast<char>(character);
    captured_.push_back(value);
    return destination_->sputc(value);
  }

  std::streamsize xsputn(const char* data, std::streamsize count) override {
    captured_.append(data, static_cast<size_t>(count));
    return destination_->sputn(data, count);
  }

  int sync() override { return destination_->pubsync(); }

 private:
  std::streambuf* destination_;
  std::string captured_;
};

std::string Trim(const std::string& input) {
  const size_t first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const size_t last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

bool ParseFormattedKey(const std::string& formatted, gtsam::Key* key) {
  if (formatted.empty()) return false;
  try {
    size_t consumed = 0;
    if (std::isalpha(static_cast<unsigned char>(formatted.front()))) {
      if (formatted.size() == 1) return false;
      const unsigned long long index =
          std::stoull(formatted.substr(1), &consumed);
      if (consumed != formatted.size() - 1) return false;
      *key = gtsam::Symbol(formatted.front(), index).key();
    } else {
      const unsigned long long numeric = std::stoull(formatted, &consumed);
      if (consumed != formatted.size()) return false;
      *key = static_cast<gtsam::Key>(numeric);
    }
    return true;
  } catch (...) {
    return false;
  }
}

std::vector<CheckedLmTrialDirectionDiagnostics> ParseLinkedTryDelta(
    const std::string& captured, size_t call_index,
    const gtsam::Values& base_values) {
  std::vector<CheckedLmTrialDirectionDiagnostics> output;
  std::istringstream input(captured);
  std::string line;
  while (std::getline(input, line)) {
    const std::string trimmed = Trim(line);
    const std::string lambda_prefix = "trying lambda = ";
    if (trimmed.find(lambda_prefix) == 0) {
      CheckedLmTrialDirectionDiagnostics trial;
      trial.call_index = call_index;
      trial.trial_index_within_call = output.size() + 1;
      try {
        size_t consumed = 0;
        trial.lambda = std::stod(trimmed.substr(lambda_prefix.size()),
                                 &consumed);
        if (consumed != trimmed.size() - lambda_prefix.size())
          trial.parse_reason = "LAMBDA_TRAILING_TEXT";
      } catch (...) {
        trial.parse_reason = "LAMBDA_PARSE_FAILED";
      }
      output.push_back(std::move(trial));
      continue;
    }
    if (output.empty()) continue;
    auto& trial = output.back();
    const std::string norm_prefix = "linear delta norm = ";
    if (trimmed.find(norm_prefix) == 0) {
      try {
        size_t consumed = 0;
        trial.linked_reported_delta_norm =
            std::stod(trimmed.substr(norm_prefix.size()), &consumed);
        if (consumed != trimmed.size() - norm_prefix.size())
          trial.parse_reason = "DELTA_NORM_TRAILING_TEXT";
      } catch (...) {
        trial.parse_reason = "DELTA_NORM_PARSE_FAILED";
      }
      continue;
    }
    const std::string delta_prefix = "delta: ";
    if (trimmed.find(delta_prefix) != 0) continue;
    const std::string elements_suffix = " elements";
    const size_t suffix = trimmed.rfind(elements_suffix);
    if (suffix == std::string::npos) {
      trial.parse_reason = "DELTA_HEADER_PARSE_FAILED";
      continue;
    }
    try {
      size_t consumed = 0;
      trial.declared_key_count = static_cast<size_t>(std::stoull(
          trimmed.substr(delta_prefix.size(), suffix - delta_prefix.size()),
          &consumed));
      if (consumed != suffix - delta_prefix.size()) {
        trial.parse_reason = "DELTA_HEADER_TRAILING_TEXT";
        continue;
      }
    } catch (...) {
      trial.parse_reason = "DELTA_HEADER_PARSE_FAILED";
      continue;
    }
    bool rows_valid = true;
    for (size_t row = 0; row < trial.declared_key_count; ++row) {
      if (!std::getline(input, line)) {
        trial.parse_reason = "DELTA_ROW_MISSING";
        rows_valid = false;
        break;
      }
      const std::string delta_row = Trim(line);
      const size_t colon = delta_row.find(':');
      if (colon == std::string::npos) {
        trial.parse_reason = "DELTA_ROW_MISSING_COLON";
        rows_valid = false;
        break;
      }
      const std::string formatted_key = Trim(delta_row.substr(0, colon));
      gtsam::Key key = 0;
      if (!ParseFormattedKey(formatted_key, &key) ||
          !base_values.exists(key) || trial.delta.exists(key)) {
        trial.parse_reason = "DELTA_ROW_KEY_INVALID_OR_DUPLICATE";
        rows_valid = false;
        break;
      }
      std::istringstream values(delta_row.substr(colon + 1));
      std::vector<double> parsed;
      double value = 0.0;
      while (values >> value) parsed.push_back(value);
      if (!values.eof() || parsed.empty() ||
          parsed.size() != base_values.at(key).dim()) {
        trial.parse_reason = "DELTA_ROW_DIMENSION_OR_VALUE_INVALID";
        rows_valid = false;
        break;
      }
      gtsam::Vector vector(parsed.size());
      for (size_t index = 0; index < parsed.size(); ++index)
        vector[static_cast<Eigen::Index>(index)] = parsed[index];
      trial.parsed_dimension_count += parsed.size();
      trial.delta.insert(key, vector);
    }
    trial.parsed_key_count = trial.delta.size();
    trial.parsed_delta_norm = trial.delta.norm();
    bool complete_keys = rows_valid &&
        trial.parsed_key_count == trial.declared_key_count &&
        trial.parsed_key_count == base_values.size();
    if (complete_keys) {
      for (gtsam::Key key : base_values.keys()) {
        if (!trial.delta.exists(key) ||
            trial.delta.at(key).size() !=
                static_cast<Eigen::Index>(base_values.at(key).dim())) {
          complete_keys = false;
          break;
        }
      }
    }
    trial.parsed = complete_keys && std::isfinite(trial.lambda) &&
                   std::isfinite(trial.linked_reported_delta_norm) &&
                   std::isfinite(trial.parsed_delta_norm);
    if (trial.parsed) {
      trial.parse_reason = "OK_COMPLETE_KEYS_DIMENSIONS_MAX_DIGITS10";
    } else if (trial.parse_reason.empty()) {
      trial.parse_reason = "DELTA_INCOMPLETE_OR_NONFINITE";
    }
  }
  for (auto& trial : output) {
    if (!trial.parsed && trial.parse_reason.empty())
      trial.parse_reason = "NO_COMPLETE_TRYDELTA_FOR_TRIAL";
  }
  return output;
}

bool LambdaSelected(double lambda, const std::vector<double>& selected) {
  for (double target : selected) {
    const double scale = std::max({1.0, std::abs(lambda), std::abs(target)});
    if (std::abs(lambda - target) <=
        16.0 * std::numeric_limits<double>::epsilon() * scale)
      return true;
  }
  return false;
}

double VectorValuesDot(const gtsam::VectorValues& left,
                       const gtsam::VectorValues& right, bool* valid) {
  double output = 0.0;
  *valid = left.size() == right.size();
  if (!*valid) return std::numeric_limits<double>::quiet_NaN();
  for (const auto& key_vector : right) {
    if (!left.exists(key_vector.first) ||
        left.at(key_vector.first).size() != key_vector.second.size()) {
      *valid = false;
      return std::numeric_limits<double>::quiet_NaN();
    }
    output += left.at(key_vector.first).dot(key_vector.second);
  }
  return output;
}

double VectorValuesSubsetDot(const gtsam::VectorValues& subset,
                             const gtsam::VectorValues& full, bool* valid) {
  double output = 0.0;
  *valid = subset.size() > 0;
  for (const auto& key_vector : subset) {
    if (!full.exists(key_vector.first) ||
        full.at(key_vector.first).size() != key_vector.second.size()) {
      *valid = false;
      return std::numeric_limits<double>::quiet_NaN();
    }
    output += key_vector.second.dot(full.at(key_vector.first));
  }
  return output;
}

gtsam::VectorValues ScaleVectorValues(const gtsam::VectorValues& input,
                                      double scale) {
  gtsam::VectorValues output(input);
  for (auto& key_vector : output) key_vector.second *= scale;
  return output;
}

void EvaluateSelectedTrialDirections(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& base_values,
    const CheckedLmDiagnosticRequest& request,
    std::vector<CheckedLmTrialDirectionDiagnostics>* trials) {
  const auto linear = graph.linearize(base_values);
  const gtsam::VectorValues gradient = linear->gradientAtZero();
  for (auto& trial : *trials) {
    trial.selected_for_evaluation =
        request.first_block_budget_diagnostic
            ? (&trial == &trials->back() && trial.accepted_retract_matches)
            : LambdaSelected(trial.lambda, request.direction_evaluation_lambdas);
    if (!trial.selected_for_evaluation) continue;
    trial.evaluation_reason = "INVALID_PARSED_TRYDELTA";
    if (!trial.parsed || !(trial.parsed_delta_norm > 0.0)) continue;
    bool dot_valid = false;
    trial.gradient_dot_delta =
        VectorValuesDot(gradient, trial.delta, &dot_valid);
    if (!dot_valid || !std::isfinite(trial.gradient_dot_delta)) {
      trial.evaluation_reason = "GRADIENT_DELTA_KEY_OR_DIMENSION_MISMATCH";
      continue;
    }
    const gtsam::VectorValues unit =
        ScaleVectorValues(trial.delta, 1.0 / trial.parsed_delta_norm);
    trial.gradient_dot_unit_direction =
        VectorValuesDot(gradient, unit, &dot_valid);
    if (!dot_valid || !std::isfinite(trial.gradient_dot_unit_direction)) {
      trial.evaluation_reason = "GRADIENT_UNIT_DIRECTION_INVALID";
      continue;
    }
    const gtsam::Values tentative = base_values.retract(trial.delta);
    trial.base_graph_error = graph.error(base_values);
    trial.tentative_graph_error = graph.error(tentative);
    trial.direct_graph_decrease =
        trial.base_graph_error - trial.tentative_graph_error;
    const gtsam::VectorValues zero = gtsam::VectorValues::Zero(trial.delta);
    trial.old_linearized_error = linear->error(zero);
    trial.new_linearized_error = linear->error(trial.delta);
    trial.predicted_decrease =
        trial.old_linearized_error - trial.new_linearized_error;

    bool finite_differences_valid = true;
    trial.directional_finite_difference.clear();
    std::vector<gtsam::Values> finite_difference_plus_values;
    std::vector<gtsam::Values> finite_difference_minus_values;
    finite_difference_plus_values.reserve(
        request.direction_finite_difference_steps.size());
    finite_difference_minus_values.reserve(
        request.direction_finite_difference_steps.size());
    for (double step : request.direction_finite_difference_steps) {
      CheckedLmDirectionFiniteDifferencePoint point;
      point.step = step;
      if (!(step > 0.0) || !std::isfinite(step)) {
        finite_differences_valid = false;
        finite_difference_plus_values.emplace_back();
        finite_difference_minus_values.emplace_back();
      } else {
        finite_difference_plus_values.push_back(
            base_values.retract(ScaleVectorValues(unit, step)));
        finite_difference_minus_values.push_back(
            base_values.retract(ScaleVectorValues(unit, -step)));
        point.objective_plus = graph.error(finite_difference_plus_values.back());
        point.objective_minus =
            graph.error(finite_difference_minus_values.back());
        point.central_derivative =
            (point.objective_plus - point.objective_minus) / (2.0 * step);
        point.absolute_difference_from_analytic = std::abs(
            point.central_derivative - trial.gradient_dot_unit_direction);
        point.agreement_tolerance = 5e-9 +
            5e-3 * std::abs(trial.gradient_dot_unit_direction);
        point.agrees = std::isfinite(point.central_derivative) &&
                       point.absolute_difference_from_analytic <=
                           point.agreement_tolerance;
        finite_differences_valid = finite_differences_valid &&
            std::isfinite(point.objective_plus) &&
            std::isfinite(point.objective_minus) &&
            std::isfinite(point.central_derivative);
      }
      trial.directional_finite_difference.push_back(point);
    }

    double old_sum = 0.0;
    double tentative_sum = 0.0;
    double decrease_sum = 0.0;
    long double old_long = 0.0L;
    long double tentative_long = 0.0L;
    long double decrease_long = 0.0L;
    bool factors_valid = true;
    trial.factors.clear();
    for (size_t index = 0; index < graph.size(); ++index) {
      CheckedLmDirectionFactorDiagnostics factor;
      factor.factor_index = index;
      if (!graph.at(index)) {
        factor.dynamic_type = "NULL_FACTOR";
        factors_valid = false;
      } else {
        factor.dynamic_type = typeid(*graph.at(index)).name();
        for (gtsam::Key key : graph.at(index)->keys())
          factor.keys.push_back(key);
        factor.error_at_base = graph.at(index)->error(base_values);
        factor.error_at_tentative = graph.at(index)->error(tentative);
        factor.decrease = factor.error_at_base - factor.error_at_tentative;
        factor.directional_derivative_reason =
            "FACTOR_LINEARIZATION_OR_DIRECTION_INVALID";
        const auto factor_linear = graph.at(index)->linearize(base_values);
        bool factor_dot_valid = false;
        if (factor_linear) {
          const gtsam::VectorValues factor_gradient =
              factor_linear->gradientAtZero();
          factor.gradient_dot_unit_direction = VectorValuesSubsetDot(
              factor_gradient, unit, &factor_dot_valid);
        }
        bool factor_fd_valid = true;
        for (size_t step_index = 0;
             step_index < request.direction_finite_difference_steps.size();
             ++step_index) {
          CheckedLmDirectionFiniteDifferencePoint point;
          point.step = request.direction_finite_difference_steps[step_index];
          if (!(point.step > 0.0) || !std::isfinite(point.step) ||
              step_index >= finite_difference_plus_values.size() ||
              finite_difference_plus_values[step_index].empty() ||
              finite_difference_minus_values[step_index].empty()) {
            factor_fd_valid = false;
          } else {
            point.objective_plus = graph.at(index)->error(
                finite_difference_plus_values[step_index]);
            point.objective_minus = graph.at(index)->error(
                finite_difference_minus_values[step_index]);
            point.central_derivative =
                (point.objective_plus - point.objective_minus) /
                (2.0 * point.step);
            point.absolute_difference_from_analytic = std::abs(
                point.central_derivative -
                factor.gradient_dot_unit_direction);
            point.agreement_tolerance = 5e-9 + 5e-3 *
                std::abs(factor.gradient_dot_unit_direction);
            point.agrees = factor_dot_valid &&
                std::isfinite(point.central_derivative) &&
                point.absolute_difference_from_analytic <=
                    point.agreement_tolerance;
            factor_fd_valid = factor_fd_valid &&
                std::isfinite(point.objective_plus) &&
                std::isfinite(point.objective_minus) &&
                std::isfinite(point.central_derivative);
          }
          factor.directional_finite_difference.push_back(point);
        }
        factor.directional_derivative_valid = factor_dot_valid &&
            std::isfinite(factor.gradient_dot_unit_direction) &&
            factor_fd_valid &&
            factor.directional_finite_difference.size() ==
                request.direction_finite_difference_steps.size();
        if (factor.directional_derivative_valid)
          factor.directional_derivative_reason = "OK";
        factors_valid = factors_valid && std::isfinite(factor.error_at_base) &&
                        std::isfinite(factor.error_at_tentative) &&
                        std::isfinite(factor.decrease) &&
                        factor.directional_derivative_valid;
        old_sum += factor.error_at_base;
        tentative_sum += factor.error_at_tentative;
        decrease_sum += factor.decrease;
        old_long += static_cast<long double>(factor.error_at_base);
        tentative_long +=
            static_cast<long double>(factor.error_at_tentative);
        decrease_long += static_cast<long double>(factor.error_at_base) -
                         static_cast<long double>(factor.error_at_tentative);
      }
      trial.factors.push_back(std::move(factor));
    }
    trial.factor_old_sum_double = old_sum;
    trial.factor_tentative_sum_double = tentative_sum;
    trial.factor_decrease_sum_double = decrease_sum;
    trial.factor_old_sum_long_double = old_long;
    trial.factor_tentative_sum_long_double = tentative_long;
    trial.factor_decrease_sum_long_double = decrease_long;

    trial.evaluation_valid = factors_valid && finite_differences_valid &&
        std::isfinite(trial.base_graph_error) &&
        std::isfinite(trial.tentative_graph_error) &&
        std::isfinite(trial.old_linearized_error) &&
        std::isfinite(trial.new_linearized_error) &&
        trial.directional_finite_difference.size() ==
            request.direction_finite_difference_steps.size();
    trial.evaluation_reason = trial.evaluation_valid ? "OK" :
        "NONFINITE_OR_INCOMPLETE_POST_CAPTURE_EVALUATION";
  }
}

bool UsesNavigationStationarityQualification(ConditionalLmPolicy policy) {
  return policy ==
             ConditionalLmPolicy::GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1 ||
         policy == ConditionalLmPolicy::
             GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
}

bool UsesContinuousLambdaSearchV2(ConditionalLmPolicy policy) {
  return policy == ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
}

struct NormalLmTrialAccounting {
  bool complete = false;
  size_t trials = 0;
  size_t rejections = 0;
};

NormalLmTrialAccounting AccountNormalLmReturn(
    int inner_delta, size_t accepted_delta, double lambda,
    double lambda_upper_bound) {
  NormalLmTrialAccounting output;
  const int confirmed_trials = std::max(0, inner_delta);
  output.trials = static_cast<size_t>(confirmed_trials);
  output.rejections = static_cast<size_t>(std::max(
      0, confirmed_trials - static_cast<int>(accepted_delta)));

  // In the linked GTSAM implementation, accepted updates and lambda-search
  // rejections both increment the inner counter. A terminal small-cost-change
  // return does neither, so it contributes one additional trial but no
  // rejection. By contrast, lambda exhaustion follows increaseLambda(), and
  // its terminal rejection is already included in inner_delta.
  const bool linked_state_delta_is_consistent =
      inner_delta >= 0 && accepted_delta <= 1 &&
      accepted_delta <= static_cast<size_t>(confirmed_trials) &&
      !std::isnan(lambda) && !std::isnan(lambda_upper_bound);
  if (!linked_state_delta_is_consistent) return output;
  if (accepted_delta == 0 && lambda < lambda_upper_bound) ++output.trials;
  output.complete = true;
  return output;
}

void MergeLambdaTrialAccountingStatus(bool call_is_complete,
                                      std::string* aggregate_status) {
  if (aggregate_status->find("INCOMPLETE_") == 0) return;
  *aggregate_status = call_is_complete
                          ? "COMPLETE"
                          : "INCOMPLETE_UNEXPECTED_LINKED_STATE_DELTA";
}

std::pair<double, double> LocalDeltaNormAndMaximum(
    const gtsam::Values& before, const gtsam::Values& after) {
  const gtsam::VectorValues delta = before.localCoordinates(after);
  double maximum = 0.0;
  for (const auto& key_vector : delta)
    if (key_vector.second.size() > 0)
      maximum = std::max(maximum,
                         key_vector.second.cwiseAbs().maxCoeff());
  return {delta.norm(), maximum};
}

CheckedLmFirstTryDiagnostics InspectFirstLinkedLmTry(
    gtsam::LevenbergMarquardtOptimizer* optimizer,
    const gtsam::LevenbergMarquardtParams& params) {
  CheckedLmFirstTryDiagnostics output;
  output.minimum_model_fidelity = params.minModelFidelity;
  output.small_cost_change_threshold =
      params.relativeErrorTol * optimizer->error();
  try {
    const auto linear = optimizer->linearize();
    if (!linear) {
      output.reason = "linearization returned null";
      return output;
    }
    gtsam::VectorValues sqrt_hessian_diagonal;
    if (params.diagonalDamping) {
      sqrt_hessian_diagonal = linear->hessianDiagonal();
      for (auto& key_vector : sqrt_hessian_diagonal)
        key_vector.second = key_vector.second.cwiseMax(params.minDiagonal)
                                .cwiseMin(params.maxDiagonal)
                                .cwiseSqrt();
    }
    const auto damped = optimizer->buildDampedSystem(
        *linear, sqrt_hessian_diagonal);
    gtsam::VectorValues delta;
    try {
      delta = optimizer->solve(damped, params);
      output.linear_system_solved = true;
    } catch (const std::exception& error) {
      output.valid = true;
      output.reason = std::string("linear solve failed: ") + error.what();
      output.predicted_first_try_branch = "INCREASE_LAMBDA_AFTER_SOLVE_FAILURE";
      return output;
    }
    output.delta_norm = delta.norm();
    output.max_abs_delta = 0.0;
    for (const auto& key_vector : delta)
      if (key_vector.second.size() > 0)
        output.max_abs_delta = std::max(
            output.max_abs_delta,
            key_vector.second.cwiseAbs().maxCoeff());
    const gtsam::VectorValues zero = gtsam::VectorValues::Zero(delta);
    output.old_linearized_error = linear->error(zero);
    output.new_linearized_error = linear->error(delta);
    output.linearized_cost_change =
        output.old_linearized_error - output.new_linearized_error;
    output.linearized_resolution_threshold =
        std::numeric_limits<double>::epsilon() *
        output.old_linearized_error;
    output.linearized_step_valid = output.linearized_cost_change >= 0.0;
    if (!output.linearized_step_valid) {
      output.valid = true;
      output.reason = "negative linearized cost change";
      output.predicted_first_try_branch =
          "INCREASE_LAMBDA_AFTER_INVALID_LINEARIZED_STEP";
      return output;
    }
    output.linearized_change_resolvable =
        output.linearized_cost_change >
        output.linearized_resolution_threshold;
    const gtsam::Values candidate = optimizer->values().retract(delta);
    output.tentative_error = optimizer->graph().error(candidate);
    output.tentative_cost_change =
        optimizer->error() - output.tentative_error;
    if (output.linearized_change_resolvable) {
      output.model_fidelity = output.tentative_cost_change /
                              output.linearized_cost_change;
      output.model_fidelity_valid = std::isfinite(output.model_fidelity);
      output.model_fidelity_passed =
          output.model_fidelity_valid &&
          output.model_fidelity > params.minModelFidelity;
    }
    output.small_cost_change =
        std::abs(output.tentative_cost_change) <
        output.small_cost_change_threshold;
    if (output.model_fidelity_passed) {
      output.predicted_first_try_branch = "ACCEPT_STATE_UPDATE";
    } else if (output.small_cost_change) {
      output.predicted_first_try_branch =
          "RETURN_WITHOUT_UPDATE_SMALL_COST_CHANGE";
    } else {
      output.predicted_first_try_branch = "INCREASE_LAMBDA";
    }
    output.valid =
        std::isfinite(output.delta_norm) &&
        std::isfinite(output.max_abs_delta) &&
        std::isfinite(output.old_linearized_error) &&
        std::isfinite(output.new_linearized_error) &&
        std::isfinite(output.linearized_cost_change) &&
        std::isfinite(output.linearized_resolution_threshold) &&
        std::isfinite(output.tentative_error) &&
        std::isfinite(output.tentative_cost_change) &&
        std::isfinite(output.small_cost_change_threshold);
    output.reason = output.valid ? "OK" : "NONFINITE_DIAGNOSTIC";
  } catch (const std::exception& error) {
    output.reason = std::string("first-try diagnostic failed: ") +
                    error.what();
  }
  return output;
}

double NavigationCoordinateScale(char symbol, Eigen::Index coordinate,
                                 Eigen::Index dimension,
                                 const NavigationScales& scales,
                                 std::string* unit) {
  if (symbol == 'x' && dimension == 6) {
    if (coordinate < 3) {
      *unit = "rad";
      return scales.pose_rotation_rad;
    }
    *unit = "m";
    return scales.pose_translation_m;
  }
  if (symbol == 'v' && dimension == 3) {
    *unit = "m/s";
    return scales.velocity_mps;
  }
  if (symbol == 'b' && dimension == 6) {
    if (coordinate < 3) {
      *unit = "m/s^2";
      return scales.accel_bias_mps2;
    }
    *unit = "rad/s";
    return scales.gyro_bias_radps;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

CheckedLmFiniteDifferenceDiagnostics CheckGradientCoordinateByFiniteDifference(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const NavigationScales& scales, gtsam::Key key,
    Eigen::Index coordinate, const std::vector<double>& steps) {
  CheckedLmFiniteDifferenceDiagnostics output;
  if (steps.empty()) {
    output.reason = "finite-difference step grid is empty";
    return output;
  }
  for (double step : steps) {
    if (!(step > 0.0) || !std::isfinite(step)) {
      output.reason = "finite-difference step grid is invalid";
      return output;
    }
  }
  try {
    const auto linear = graph.linearize(values);
    if (!linear) {
      output.reason = "finite-difference linearization returned null";
      return output;
    }
    const gtsam::VectorValues gradient = linear->gradientAtZero();
    if (!values.exists(key) || !gradient.exists(key) || coordinate < 0 ||
        coordinate >= gradient.at(key).size()) {
      output.reason = "finite-difference coordinate is absent";
      return output;
    }
    std::string unit;
    const char symbol = gtsam::Symbol(key).chr();
    const double scale = NavigationCoordinateScale(
        symbol, coordinate, gradient.at(key).size(), scales, &unit);
    if (!std::isfinite(scale)) {
      output.reason = "finite-difference coordinate is not navigation";
      return output;
    }
    output.key = gtsam::DefaultKeyFormatter(key);
    output.key_value = key;
    output.coordinate = static_cast<size_t>(coordinate);
    output.dimension = gradient.at(key).size();
    output.coordinate_unit = unit;
    output.coordinate_scale = scale;
    output.objective = graph.error(values);
    output.analytic_gradient = gradient.at(key)[coordinate];
    output.scaled_analytic_gradient = output.analytic_gradient * scale;
    for (double step : steps) {
      gtsam::VectorValues plus_delta = values.zeroVectors();
      gtsam::VectorValues minus_delta = values.zeroVectors();
      plus_delta.at(key)[coordinate] = step;
      minus_delta.at(key)[coordinate] = -step;
      CheckedLmFiniteDifferencePoint point;
      point.step = step;
      point.objective_plus = graph.error(values.retract(plus_delta));
      point.objective_minus = graph.error(values.retract(minus_delta));
      point.objective_change_plus = point.objective_plus - output.objective;
      point.objective_change_minus = point.objective_minus - output.objective;
      point.central_derivative =
          (point.objective_plus - point.objective_minus) / (2.0 * step);
      point.absolute_difference_from_analytic =
          std::abs(point.central_derivative - output.analytic_gradient);
      point.agreement_tolerance =
          5e-9 + 5e-3 * std::abs(output.analytic_gradient);
      point.agrees = point.absolute_difference_from_analytic <=
                     point.agreement_tolerance;
      output.points.push_back(point);
    }
    output.valid = std::isfinite(output.objective) &&
                   std::isfinite(output.analytic_gradient) &&
                   std::isfinite(output.scaled_analytic_gradient);
    for (const auto& point : output.points)
      output.valid = output.valid && std::isfinite(point.objective_plus) &&
                     std::isfinite(point.objective_minus) &&
                     std::isfinite(point.central_derivative);
    output.reason = output.valid ? "OK" : "NONFINITE_FINITE_DIFFERENCE";
  } catch (const std::exception& error) {
    output.reason = std::string("finite-difference diagnostic failed: ") +
                    error.what();
  }
  return output;
}

CheckedLmFiniteDifferenceDiagnostics CheckDominantGradientByFiniteDifference(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const NavigationScales& scales, const std::vector<double>& steps) {
  CheckedLmFiniteDifferenceDiagnostics output;
  if (steps.empty()) {
    output.reason = "finite-difference step grid is empty";
    return output;
  }
  for (double step : steps) {
    if (!(step > 0.0) || !std::isfinite(step)) {
      output.reason = "finite-difference step grid is invalid";
      return output;
    }
  }
  try {
    const auto linear = graph.linearize(values);
    if (!linear) {
      output.reason = "finite-difference linearization returned null";
      return output;
    }
    const gtsam::VectorValues gradient = linear->gradientAtZero();
    bool found = false;
    gtsam::Key dominant_key = 0;
    Eigen::Index dominant_coordinate = 0;
    double dominant_gradient = 0.0;
    double dominant_scale = 0.0;
    std::string dominant_unit;
    for (gtsam::Key key : values.keys()) {
      if (!gradient.exists(key)) continue;
      const char symbol = gtsam::Symbol(key).chr();
      const auto& key_gradient = gradient.at(key);
      for (Eigen::Index coordinate = 0;
           coordinate < key_gradient.size(); ++coordinate) {
        std::string unit;
        const double scale = NavigationCoordinateScale(
            symbol, coordinate, key_gradient.size(), scales, &unit);
        if (!std::isfinite(scale)) continue;
        const double scaled = std::abs(key_gradient[coordinate]) * scale;
        if (!found || scaled > std::abs(dominant_gradient) * dominant_scale) {
          found = true;
          dominant_key = key;
          dominant_coordinate = coordinate;
          dominant_gradient = key_gradient[coordinate];
          dominant_scale = scale;
          dominant_unit = unit;
        }
      }
    }
    if (!found) {
      output.reason = "no declared navigation coordinate in gradient";
      return output;
    }
    output = CheckGradientCoordinateByFiniteDifference(
        graph, values, scales, dominant_key, dominant_coordinate, steps);
  } catch (const std::exception& error) {
    output.reason = std::string("finite-difference diagnostic failed: ") +
                    error.what();
  }
  return output;
}

FixedCheckpointLmArmDiagnostics RunFixedCheckpointLmArm(
    const std::string& arm,
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& checkpoint_values,
    double checkpoint_lambda,
    const CheckedLmOptions& options,
    bool disable_internal_small_change_stop,
    size_t max_calls,
    double max_seconds) {
  FixedCheckpointLmArmDiagnostics output;
  output.arm = arm;
  output.internal_relative_tolerance_changed =
      disable_internal_small_change_stop;
  output.internal_relative_tolerance =
      disable_internal_small_change_stop ? 0.0 : options.relative_tolerance;
  output.external_relative_tolerance = options.relative_tolerance;
  output.external_absolute_tolerance = options.absolute_tolerance;
  output.initial_lambda = checkpoint_lambda;
  output.initial_error = graph.error(checkpoint_values);
  const auto started = std::chrono::steady_clock::now();
  try {
    gtsam::LevenbergMarquardtParams internal_params;
    internal_params.setMaxIterations(static_cast<int>(max_calls));
    internal_params.setRelativeErrorTol(output.internal_relative_tolerance);
    internal_params.setAbsoluteErrorTol(options.absolute_tolerance);
    internal_params.setLinearSolverType("SEQUENTIAL_CHOLESKY");
    internal_params.setlambdaInitial(checkpoint_lambda);
    internal_params.setVerbosityLM("TRYLAMBDA");
    output.lambda_upper_bound = internal_params.getlambdaUpperBound();

    gtsam::LevenbergMarquardtParams external_params = internal_params;
    external_params.setRelativeErrorTol(options.relative_tolerance);
    external_params.setAbsoluteErrorTol(options.absolute_tolerance);

    gtsam::LevenbergMarquardtOptimizer optimizer(
        graph, checkpoint_values, internal_params);
    std::cout << "T10_FIXED_CHECKPOINT_ARM_BEGIN arm=" << arm
              << " internal_relative_tolerance="
              << output.internal_relative_tolerance
              << " initial_lambda=" << checkpoint_lambda << '\n';
    for (size_t call_index = 1; call_index <= max_calls; ++call_index) {
      const double before_call_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count();
      if (before_call_seconds >= max_seconds) {
        output.timed_out = true;
        output.reason = "ARM_WALL_TIME_LIMIT_REACHED_BEFORE_CALL";
        break;
      }
      FixedCheckpointLmCallDiagnostics call;
      auto& linked = call.linked;
      linked.call_index = call_index;
      linked.optimizer_iterations_before = optimizer.iterations();
      linked.inner_iterations_before = optimizer.getInnerIterations();
      linked.lambda_before = optimizer.lambda();
      linked.error_before = optimizer.error();
      const gtsam::Values values_before = optimizer.values();
      linked.first_try = InspectFirstLinkedLmTry(&optimizer, internal_params);
      try {
        optimizer.iterate();
      } catch (const std::exception& error) {
        output.reason = std::string("LINKED_ITERATE_EXCEPTION: ") +
                        error.what();
        output.calls.push_back(std::move(call));
        break;
      }
      linked.optimizer_iterations_after = optimizer.iterations();
      linked.inner_iterations_after = optimizer.getInnerIterations();
      linked.lambda_after = optimizer.lambda();
      linked.error_after = optimizer.error();
      const auto delta = LocalDeltaNormAndMaximum(values_before,
                                                   optimizer.values());
      linked.accepted_values_delta_norm = delta.first;
      linked.accepted_values_max_abs_delta = delta.second;
      linked.accepted_state_update =
          linked.optimizer_iterations_after >
          linked.optimizer_iterations_before;
      const int inner_delta = linked.inner_iterations_after -
                              linked.inner_iterations_before;
      const int accepted_delta = static_cast<int>(
          linked.optimizer_iterations_after -
          linked.optimizer_iterations_before);
      linked.rejected_lambda_trials_before_acceptance =
          static_cast<size_t>(std::max(0, inner_delta - accepted_delta));
      if (linked.accepted_state_update) {
        linked.observed_return_class = "ACCEPTED_STATE_UPDATE";
      } else if (inner_delta == 0 && linked.first_try.small_cost_change) {
        linked.observed_return_class =
            "RETURN_WITHOUT_UPDATE_SMALL_COST_CHANGE";
      } else if (inner_delta > 0) {
        linked.observed_return_class =
            "RETURN_WITHOUT_ACCEPTED_UPDATE_AFTER_LAMBDA_SEARCH";
      } else {
        linked.observed_return_class = "RETURN_WITHOUT_OBSERVED_UPDATE";
      }
      call.accepted_strict_descent =
          linked.accepted_state_update &&
          linked.error_after < linked.error_before;
      call.external_generic_convergence = gtsam::checkConvergence(
          external_params, linked.error_before, linked.error_after);
      call.stationarity = AuditNavigationStationarity(
          graph, optimizer.values(), options.navigation_scales,
          options.navigation_stationarity_tolerance_objective,
          options.gradient_roundoff_safety_factor);
      call.stationary_qualified = call.external_generic_convergence &&
                                  call.stationarity.valid &&
                                  call.stationarity.stationary;
      const auto trial_accounting = AccountNormalLmReturn(
          inner_delta, static_cast<size_t>(accepted_delta),
          linked.lambda_after, internal_params.getlambdaUpperBound());
      output.lambda_trial_count += trial_accounting.trials;
      output.rejected_lambda_trial_count += trial_accounting.rejections;
      output.accepted_update_count += linked.accepted_state_update ? 1 : 0;
      output.accepted_strict_descent_count +=
          call.accepted_strict_descent ? 1 : 0;
      output.calls.push_back(std::move(call));
      if (output.calls.back().stationary_qualified) {
        output.stationary_qualified = true;
        output.reason = "STATIONARY_QUALIFIED";
        break;
      }
      const double after_call_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count();
      if (after_call_seconds >= max_seconds) {
        output.timed_out = true;
        output.reason = "ARM_WALL_TIME_LIMIT_REACHED_AFTER_CALL";
        break;
      }
    }
    output.values_at_final = optimizer.values();
    output.final_error = optimizer.error();
    output.final_lambda = optimizer.lambda();
    output.call_count = output.calls.size();
    output.final_stationarity = AuditNavigationStationarity(
        graph, optimizer.values(), options.navigation_scales,
        options.navigation_stationarity_tolerance_objective,
        options.gradient_roundoff_safety_factor);
    if (output.reason == "NOT_RUN") {
      output.reason = output.call_count == max_calls
                          ? "CALL_LIMIT_WITHOUT_STATIONARITY"
                          : "ENDED_WITHOUT_STATIONARITY";
    }
    output.valid = std::isfinite(output.initial_error) &&
                   std::isfinite(output.final_error) &&
                   std::isfinite(output.initial_lambda) &&
                   std::isfinite(output.final_lambda) &&
                   output.final_stationarity.valid;
    std::cout << "T10_FIXED_CHECKPOINT_ARM_END arm=" << arm
              << " reason=" << output.reason
              << " calls=" << output.call_count
              << " lambda_trials=" << output.lambda_trial_count
              << " accepted=" << output.accepted_update_count
              << " accepted_descent="
              << output.accepted_strict_descent_count
              << " final_lambda=" << output.final_lambda
              << " final_error=" << output.final_error << '\n';
  } catch (const std::exception& error) {
    output.reason = std::string("ARM_SETUP_EXCEPTION: ") + error.what();
  }
  output.elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
  return output;
}

FixedCheckpointLmRecoveryDiagnostics RunFixedCheckpointLmRecovery(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& checkpoint_values,
    double checkpoint_lambda,
    const CheckedLmOptions& options,
    const CheckedLmDiagnosticRequest& request) {
  FixedCheckpointLmRecoveryDiagnostics output;
  output.requested = request.run_fixed_checkpoint_lm_recovery;
  output.captured_lambda = checkpoint_lambda;
  output.captured_error = graph.error(checkpoint_values);
  output.max_calls_per_arm = request.fixed_checkpoint_max_calls;
  output.max_seconds_per_arm = request.fixed_checkpoint_max_seconds;
  if (!output.requested) return output;
  if (request.fixed_checkpoint_max_calls == 0 ||
      !(request.fixed_checkpoint_max_seconds > 0.0) ||
      !std::isfinite(request.fixed_checkpoint_max_seconds) ||
      !(checkpoint_lambda > 0.0) || !std::isfinite(checkpoint_lambda)) {
    output.reason = "INVALID_FIXED_CHECKPOINT_REQUEST";
    return output;
  }
  output.current_behavior = RunFixedCheckpointLmArm(
      "A_CURRENT", graph, checkpoint_values, checkpoint_lambda, options,
      false, request.fixed_checkpoint_max_calls,
      request.fixed_checkpoint_max_seconds);
  output.continue_lambda_search = RunFixedCheckpointLmArm(
      "B_CONTINUE_LAMBDA_SEARCH", graph, checkpoint_values,
      checkpoint_lambda, options, true,
      request.fixed_checkpoint_max_calls,
      request.fixed_checkpoint_max_seconds);
  output.valid = output.current_behavior.valid &&
                 output.continue_lambda_search.valid &&
                 output.current_behavior.elapsed_seconds <=
                     request.fixed_checkpoint_max_seconds &&
                 output.continue_lambda_search.elapsed_seconds <=
                     request.fixed_checkpoint_max_seconds;
  output.reason = output.valid ? "OK" : "ARM_INVALID_OR_TIME_LIMIT_EXCEEDED";
  return output;
}

void FinalizeCheckedLmDiagnostic(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::LevenbergMarquardtOptimizer& optimizer,
    const CheckedLmOptions& options,
    const CheckedLmDiagnosticRequest& request,
    CheckedLmDiagnosticCapture* output) {
  output->values_at_final = optimizer.values();
  output->factors.clear();
  for (size_t index = 0; index < graph.size(); ++index) {
    CheckedLmFactorDiagnostics factor;
    factor.factor_index = index;
    if (!graph.at(index)) {
      factor.dynamic_type = "NULL_FACTOR";
    } else {
      factor.dynamic_type = typeid(*graph.at(index)).name();
      for (gtsam::Key key : graph.at(index)->keys())
        factor.keys.push_back(key);
      factor.error_at_capture_start =
          graph.at(index)->error(output->values_at_start);
      factor.error_at_capture_final =
          graph.at(index)->error(output->values_at_final);
    }
    output->factors.push_back(factor);
  }
  output->stationarity_at_final = AuditNavigationStationarity(
      graph, output->values_at_final, options.navigation_scales,
      options.navigation_stationarity_tolerance_objective,
      options.gradient_roundoff_safety_factor);
  if (request.passive_terminal_capture) {
    output->valid = !output->calls.empty() && output->stationarity_at_final.valid;
    output->reason = output->valid ? "A15_PASSIVE_CAPTURE_ONLY" : "A15_CAPTURE_INVALID";
    return;
  }
  output->finite_difference = CheckDominantGradientByFiniteDifference(
      graph, output->values_at_final, options.navigation_scales,
      request.finite_difference_steps);
  output->fixed_checkpoint_recovery = RunFixedCheckpointLmRecovery(
      graph, output->values_at_final, optimizer.lambda(), options, request);
  output->valid = !output->calls.empty() &&
                  output->stationarity_at_final.valid &&
                  output->finite_difference.valid &&
                  (!request.run_fixed_checkpoint_lm_recovery ||
                   output->fixed_checkpoint_recovery.valid);
  output->reason = output->valid ? "OK" :
      "CALL_OR_FINITE_DIFFERENCE_CAPTURE_INVALID";
}

}  // namespace

CheckedLmFiniteDifferenceDiagnostics
CheckNavigationCoordinateByFiniteDifference(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const NavigationScales& scales, gtsam::Key key, size_t coordinate,
    const std::vector<double>& steps) {
  if (coordinate > static_cast<size_t>(std::numeric_limits<Eigen::Index>::max())) {
    CheckedLmFiniteDifferenceDiagnostics output;
    output.reason = "finite-difference coordinate is out of range";
    return output;
  }
  return CheckGradientCoordinateByFiniteDifference(
      graph, values, scales, key, static_cast<Eigen::Index>(coordinate), steps);
}

bool FirstBlockDerivativesConsistent(const CheckedLmDiagnosticCapture& capture) {
  if (!capture.first_block_budget_diagnostic || capture.calls.size() != 50 ||
      capture.graph_factor_count == 0 || capture.values_key_count == 0 ||
      capture.values_at_call50.empty()) return false;
  const std::vector<double> steps{1e-4, 1e-5, 1e-6};
  auto points_pass = [&](const auto& points) {
    if (points.size() != steps.size()) return false;
    for (size_t i = 0; i < steps.size(); ++i)
      if (points[i].step != steps[i] || !points[i].agrees ||
          !std::isfinite(points[i].central_derivative) ||
          !std::isfinite(points[i].absolute_difference_from_analytic) ||
          points[i].absolute_difference_from_analytic > points[i].agreement_tolerance)
        return false;
    return true;
  };
  for (size_t index : {size_t(48), size_t(49)}) {
    const auto& call = capture.calls[index];
    if (call.call_index != index + 1 || !call.accepted_state_update ||
        call.optimizer_iterations_after != call.optimizer_iterations_before + 1 ||
        call.actual_trial_directions.empty()) return false;
    const auto& trial = call.actual_trial_directions.back();
    if (!trial.parsed || !trial.accepted_retract_matches ||
        !std::isfinite(trial.accepted_retract_max_difference) ||
        trial.accepted_retract_max_difference > 1e-12 ||
        trial.parsed_key_count != capture.values_key_count ||
        !trial.selected_for_evaluation || !trial.evaluation_valid ||
        !points_pass(trial.directional_finite_difference) ||
        trial.factors.size() != capture.graph_factor_count) return false;
    for (size_t i = 0; i < trial.factors.size(); ++i) {
      const auto& factor = trial.factors[i];
      if (factor.factor_index != i || !factor.directional_derivative_valid ||
          !points_pass(factor.directional_finite_difference)) return false;
    }
  }
  return capture.finite_difference_at_call50.valid &&
         points_pass(capture.finite_difference_at_call50.points);
}

CheckedLmResult RunCheckedConditionalLm(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const CheckedLmOptions& options) {
  return RunCheckedConditionalLm(graph, initial, options, nullptr);
}

CheckedLmResult RunCheckedConditionalLmImpl(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const CheckedLmOptions& options,
    const CheckedLmDiagnosticRequest* diagnostic_request,
    bool retain_recoverable_terminal_values) {
  CheckedLmResult result;
  result.convergence.policy_version = ConditionalLmPolicyName(options.policy);
  result.convergence.stationarity_qualification_enabled =
      UsesNavigationStationarityQualification(options.policy);
  if (options.max_iterations <= 0 ||
      !(options.relative_tolerance >= 0.0) ||
      !(options.absolute_tolerance >= 0.0) ||
      !std::isfinite(options.relative_tolerance) ||
      !std::isfinite(options.absolute_tolerance) ||
      std::string(ConditionalLmPolicyName(options.policy)) ==
          "UNKNOWN_CONDITIONAL_LM_POLICY") {
    result.reason = "CONDITIONAL_LM_INVALID_OPTIONS";
    return result;
  }
  const bool passive = diagnostic_request && diagnostic_request->passive_terminal_capture;
  if (passive && (options.max_iterations != 50 ||
      !UsesContinuousLambdaSearchV2(options.policy) ||
      !diagnostic_request->capture_linked_gtsam_trydelta ||
      !diagnostic_request->emit_linked_gtsam_trylambda ||
      diagnostic_request->first_block_budget_diagnostic ||
      diagnostic_request->run_fixed_checkpoint_lm_recovery ||
      !diagnostic_request->finite_difference_steps.empty() ||
      !diagnostic_request->direction_finite_difference_steps.empty() ||
      !diagnostic_request->direction_evaluation_lambdas.empty())) {
    result.reason = "A15_INVALID_PASSIVE_CAPTURE_REQUEST";
    return result;
  }
  const bool first_block = diagnostic_request &&
      diagnostic_request->first_block_budget_diagnostic;
  if (first_block && (options.max_iterations != 50 ||
      !UsesContinuousLambdaSearchV2(options.policy) ||
      !diagnostic_request->capture_linked_gtsam_trydelta ||
      !diagnostic_request->emit_linked_gtsam_trylambda ||
      diagnostic_request->run_fixed_checkpoint_lm_recovery ||
      diagnostic_request->direction_finite_difference_steps !=
          std::vector<double>({1e-4, 1e-5, 1e-6}) ||
      diagnostic_request->finite_difference_steps !=
          std::vector<double>({1e-4, 1e-5, 1e-6}))) {
    result.reason = "A11_INVALID_FROZEN_DIAGNOSTIC_REQUEST";
    return result;
  }
  try {
    gtsam::LevenbergMarquardtParams external_check_params;
    external_check_params.setMaxIterations(options.max_iterations);
    external_check_params.setRelativeErrorTol(options.relative_tolerance);
    external_check_params.setAbsoluteErrorTol(options.absolute_tolerance);
    external_check_params.setLinearSolverType("SEQUENTIAL_CHOLESKY");
    gtsam::LevenbergMarquardtParams optimizer_params = external_check_params;
    if (UsesContinuousLambdaSearchV2(options.policy))
      optimizer_params.setRelativeErrorTol(0.0);
    if (diagnostic_request &&
        diagnostic_request->capture_linked_gtsam_trydelta)
      optimizer_params.setVerbosityLM("TRYDELTA");
    else if (diagnostic_request &&
             diagnostic_request->emit_linked_gtsam_trylambda)
      optimizer_params.setVerbosityLM("TRYLAMBDA");
    result.convergence.relative_tolerance =
        external_check_params.getRelativeErrorTol();
    result.convergence.absolute_tolerance =
        external_check_params.getAbsoluteErrorTol();
    result.convergence.error_tolerance = external_check_params.getErrorTol();
    result.convergence.optimizer_internal_relative_tolerance =
        optimizer_params.getRelativeErrorTol();
    result.convergence.optimizer_internal_small_change_stop_enabled =
        optimizer_params.getRelativeErrorTol() != 0.0;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial,
                                                 optimizer_params);
    if (diagnostic_request) {
      result.diagnostic.requested = true;
      result.diagnostic.first_block_budget_diagnostic = first_block;
      if (first_block) result.diagnostic.continuation_status = "NOT_RUN_BEFORE_CALL50";
      result.diagnostic.graph_factor_count = graph.size();
      result.diagnostic.values_key_count = initial.size();
      result.diagnostic.values_at_start = initial;
    }
    auto finalize_diagnostic = [&]() {
      if (diagnostic_request)
        FinalizeCheckedLmDiagnostic(graph, optimizer, options,
                                    *diagnostic_request,
                                    &result.diagnostic);
    };
    double previous = optimizer.error();
    result.convergence.initial_error = previous;
    result.convergence.initial_error_valid = std::isfinite(previous);
    if (!std::isfinite(previous)) {
      result.convergence.check_status = "NOT_EVALUATED_INITIAL_ERROR_NONFINITE";
      result.reason = "CONDITIONAL_LM_INITIAL_OBJECTIVE_NONFINITE";
      finalize_diagnostic();
      return result;
    }
    for (int attempt = 0; attempt < (first_block ? 200 : options.max_iterations); ++attempt) {
      CheckedLmCallDiagnostics call;
      gtsam::Values values_before;
      const size_t optimizer_iterations_before = optimizer.iterations();
      const int inner_iterations_before = optimizer.getInnerIterations();
      if (diagnostic_request) {
        call.call_index = static_cast<size_t>(attempt + 1);
        call.optimizer_iterations_before = optimizer_iterations_before;
        call.inner_iterations_before = inner_iterations_before;
        call.lambda_before = optimizer.lambda();
        call.error_before = optimizer.error();
        values_before = optimizer.values();
        call.values_before = values_before;
        if (!first_block && !passive)
          call.first_try = InspectFirstLinkedLmTry(&optimizer, optimizer_params);
        else
          call.first_try.reason = passive ? "NOT_RUN_A15_NO_EXTRA_SOLVE" : "NOT_RUN_A11_NO_SHADOW_SOLVE";
      }
      ++result.convergence.iterate_call_count;
      const bool emit_linked_trylambda =
          diagnostic_request &&
          diagnostic_request->emit_linked_gtsam_trylambda;
      const std::streamsize previous_stdout_precision = std::cout.precision();
      std::unique_ptr<TeeCaptureStreambuf> trydelta_capture;
      std::streambuf* previous_stdout_buffer = nullptr;
      if (emit_linked_trylambda) {
        std::cout.precision(std::numeric_limits<double>::max_digits10);
        std::cout << "T10_CONDITIONAL_LM_LINKED_CALL_BEGIN call_index="
                  << attempt + 1
                  << " optimizer_iterations=" << optimizer_iterations_before
                  << " inner_iterations=" << inner_iterations_before
                  << " lambda=" << optimizer.lambda()
                  << " error=" << optimizer.error()
                  << " numeric_precision=binary64_max_digits10\n";
      }
      if (diagnostic_request &&
          diagnostic_request->capture_linked_gtsam_trydelta) {
        previous_stdout_buffer = std::cout.rdbuf();
        trydelta_capture.reset(
            new TeeCaptureStreambuf(previous_stdout_buffer));
        std::cout.rdbuf(trydelta_capture.get());
      }
      try {
        optimizer.iterate();
      } catch (...) {
        if (previous_stdout_buffer)
          std::cout.rdbuf(previous_stdout_buffer);
        if (trydelta_capture && diagnostic_request) {
          call.linked_trydelta_stdout = trydelta_capture->captured();
          try {
            call.actual_trial_directions = ParseLinkedTryDelta(
                call.linked_trydelta_stdout, call.call_index, values_before);
          } catch (const std::exception& exception) {
            CheckedLmTrialDirectionDiagnostics failed_capture;
            failed_capture.call_index = call.call_index;
            failed_capture.parse_reason =
                std::string("CAPTURE_PARSE_EXCEPTION:") + exception.what();
            call.actual_trial_directions.push_back(
                std::move(failed_capture));
          } catch (...) {
            CheckedLmTrialDirectionDiagnostics failed_capture;
            failed_capture.call_index = call.call_index;
            failed_capture.parse_reason = "CAPTURE_PARSE_UNKNOWN_EXCEPTION";
            call.actual_trial_directions.push_back(
                std::move(failed_capture));
          }
        }
        if (emit_linked_trylambda) {
          std::cout << "T10_CONDITIONAL_LM_LINKED_CALL_END call_index="
                    << attempt + 1
                    << " status=EXCEPTION_AFTER_ITERATE_INVOCATION"
                    << " optimizer_iterations=" << optimizer.iterations()
                    << " inner_iterations=" << optimizer.getInnerIterations()
                    << " lambda=" << optimizer.lambda()
                    << " error=" << optimizer.error() << '\n';
          std::cout.precision(previous_stdout_precision);
        }
        result.iterations = optimizer.iterations();
        result.inner_iterations = optimizer.getInnerIterations();
        result.lambda = optimizer.lambda();
        const int inner_delta =
            result.inner_iterations - inner_iterations_before;
        const size_t accepted_delta =
            result.iterations - optimizer_iterations_before;
        // Linked GTSAM updates its inner counter only when increaseLambda() or
        // decreaseLambda() commits a rejected/accepted candidate. An exception
        // may happen during linearization (no trial) or after a candidate has
        // started but before either state transition. Therefore only retain
        // transitions that are proven by the linked state and explicitly mark
        // the aggregate incomplete; never invent a trial/rejection here.
        const int confirmed_trial_delta = std::max(0, inner_delta);
        result.convergence.lambda_trial_count +=
            static_cast<size_t>(confirmed_trial_delta);
        result.convergence.accepted_update_count += accepted_delta;
        result.convergence.rejected_lambda_trial_count +=
            static_cast<size_t>(std::max(
                0, confirmed_trial_delta - static_cast<int>(accepted_delta)));
        result.convergence.lambda_trial_accounting_status =
            "INCOMPLETE_EXCEPTION_DURING_ITERATE";
        throw;
      }
      if (previous_stdout_buffer)
        std::cout.rdbuf(previous_stdout_buffer);
      if (trydelta_capture && diagnostic_request) {
        call.linked_trydelta_stdout = trydelta_capture->captured();
        try {
          call.actual_trial_directions = ParseLinkedTryDelta(
              call.linked_trydelta_stdout, call.call_index, values_before);
          if (!passive && (!first_block || attempt == 48 || attempt == 49)) {
            if (first_block && !call.actual_trial_directions.empty() &&
                optimizer.iterations() == optimizer_iterations_before + 1) {
              auto& actual = call.actual_trial_directions.back();
              if (actual.parsed) {
                actual.accepted_retract_max_difference = LocalDeltaNormAndMaximum(
                    values_before.retract(actual.delta), optimizer.values()).second;
                actual.accepted_retract_matches =
                    std::isfinite(actual.accepted_retract_max_difference) &&
                    actual.accepted_retract_max_difference <= 1e-12;
              }
            }
            EvaluateSelectedTrialDirections(
                graph, values_before, *diagnostic_request,
                &call.actual_trial_directions);
          }
        } catch (const std::exception& exception) {
          call.actual_trial_directions.clear();
          CheckedLmTrialDirectionDiagnostics failed_capture;
          failed_capture.call_index = call.call_index;
          failed_capture.parse_reason =
              std::string("POST_CAPTURE_EVALUATION_EXCEPTION:") +
              exception.what();
          call.actual_trial_directions.push_back(std::move(failed_capture));
        } catch (...) {
          call.actual_trial_directions.clear();
          CheckedLmTrialDirectionDiagnostics failed_capture;
          failed_capture.call_index = call.call_index;
          failed_capture.parse_reason =
              "POST_CAPTURE_EVALUATION_UNKNOWN_EXCEPTION";
          call.actual_trial_directions.push_back(std::move(failed_capture));
        }
      }
      if (emit_linked_trylambda) {
        std::cout << "T10_CONDITIONAL_LM_LINKED_CALL_END call_index="
                  << attempt + 1 << " status=NORMAL_RETURN"
                  << " optimizer_iterations=" << optimizer.iterations()
                  << " inner_iterations=" << optimizer.getInnerIterations()
                  << " lambda=" << optimizer.lambda()
                  << " error=" << optimizer.error() << '\n';
        std::cout.precision(previous_stdout_precision);
      }
      const double current = optimizer.error();
      result.iterations = optimizer.iterations();
      result.inner_iterations = optimizer.getInnerIterations();
      result.lambda = optimizer.lambda();
      const int inner_delta = result.inner_iterations - inner_iterations_before;
      const size_t accepted_delta =
          result.iterations - optimizer_iterations_before;
      const auto trial_accounting = AccountNormalLmReturn(
          inner_delta, accepted_delta, result.lambda,
          optimizer_params.getlambdaUpperBound());
      result.convergence.lambda_trial_count += trial_accounting.trials;
      result.convergence.accepted_update_count += accepted_delta;
      result.convergence.rejected_lambda_trial_count +=
          trial_accounting.rejections;
      MergeLambdaTrialAccountingStatus(
          trial_accounting.complete,
          &result.convergence.lambda_trial_accounting_status);
      if (accepted_delta == 0) ++result.convergence.no_update_return_count;
      if (diagnostic_request) {
        call.optimizer_iterations_after = result.iterations;
        call.inner_iterations_after = result.inner_iterations;
        call.lambda_after = result.lambda;
        call.error_after = current;
        const auto delta = LocalDeltaNormAndMaximum(values_before,
                                                    optimizer.values());
        call.accepted_values_delta_norm = delta.first;
        call.accepted_values_max_abs_delta = delta.second;
        call.accepted_state_update =
            call.optimizer_iterations_after > call.optimizer_iterations_before;
        const int inner_delta = call.inner_iterations_after -
                                call.inner_iterations_before;
        const int accepted_delta = static_cast<int>(
            call.optimizer_iterations_after - call.optimizer_iterations_before);
        call.rejected_lambda_trials_before_acceptance =
            static_cast<size_t>(std::max(0, inner_delta - accepted_delta));
        if (call.accepted_state_update) {
          call.observed_return_class = "ACCEPTED_STATE_UPDATE";
        } else if (inner_delta == 0 && call.first_try.small_cost_change) {
          call.observed_return_class =
              "RETURN_WITHOUT_UPDATE_SMALL_COST_CHANGE";
        } else if (inner_delta > 0) {
          call.observed_return_class =
              "RETURN_WITHOUT_ACCEPTED_UPDATE_AFTER_LAMBDA_SEARCH";
        } else {
          call.observed_return_class = "RETURN_WITHOUT_OBSERVED_UPDATE";
        }
        if (first_block || passive)
          call.stationarity_after = AuditNavigationStationarity(
              graph, optimizer.values(), options.navigation_scales,
              options.navigation_stationarity_tolerance_objective,
              options.gradient_roundoff_safety_factor);
        result.diagnostic.calls.push_back(std::move(call));
      }
      if (UsesContinuousLambdaSearchV2(options.policy) &&
          accepted_delta == 0 &&
          result.lambda >= optimizer_params.getlambdaUpperBound()) {
        result.convergence.check_status =
            "NOT_EVALUATED_LAMBDA_SEARCH_EXHAUSTED";
        result.reason = "CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED";
        if (retain_recoverable_terminal_values)
          result.values = optimizer.values();
        finalize_diagnostic();
        return result;
      }
      if (!std::isfinite(current) || !std::isfinite(result.lambda)) {
        result.convergence.check_status = "NOT_EVALUATED_RESULT_NONFINITE";
        result.reason = "CONDITIONAL_LM_RESULT_NONFINITE";
        finalize_diagnostic();
        return result;
      }
      const bool linked_check_result =
          gtsam::checkConvergence(external_check_params, previous, current);
      const auto diagnostics_started = std::chrono::steady_clock::now();
      auto& diagnostics = result.convergence;
      diagnostics.check_evaluated = true;
      ++diagnostics.convergence_check_count;
      diagnostics.previous_error = previous;
      diagnostics.current_error = current;
      diagnostics.absolute_decrease = previous - current;
      diagnostics.relative_decrease =
          std::numeric_limits<double>::quiet_NaN();
      diagnostics.relative_decrease_valid = false;
      if (previous != 0.0) {
        diagnostics.relative_decrease =
            diagnostics.absolute_decrease / previous;
        diagnostics.relative_decrease_valid =
            std::isfinite(diagnostics.relative_decrease);
      }
      diagnostics.relative_tolerance_enabled =
          diagnostics.relative_tolerance != 0.0;
      // GTSAM 4.2 checks errorTol first. If it is not met, it computes both
      // decreases and returns the OR below. These flags are diagnostic only;
      // linked_check_result above remains the sole stopping decision.
      diagnostics.error_tolerance_triggered =
          current <= diagnostics.error_tolerance;
      diagnostics.decrease_predicates_reached_by_linked_check =
          !diagnostics.error_tolerance_triggered;
      diagnostics.absolute_tolerance_triggered =
          diagnostics.absolute_decrease <= diagnostics.absolute_tolerance;
      diagnostics.relative_tolerance_triggered =
          diagnostics.relative_tolerance_enabled &&
          diagnostics.relative_decrease_valid &&
          diagnostics.relative_decrease <= diagnostics.relative_tolerance;
      diagnostics.check_result = linked_check_result;
      const bool predicate_union =
          diagnostics.error_tolerance_triggered ||
          diagnostics.absolute_tolerance_triggered ||
          diagnostics.relative_tolerance_triggered;
      diagnostics.predicate_union_matches_check_result =
          predicate_union == linked_check_result;
      diagnostics.check_status =
          diagnostics.predicate_union_matches_check_result
              ? "EVALUATED_MATCHED_LINKED_GTSAM"
              : "EVALUATED_PREDICATE_MISMATCH_LINKED_GTSAM_AUTHORITATIVE";
      diagnostics.added_diagnostics_seconds += std::chrono::duration<double>(
          std::chrono::steady_clock::now() - diagnostics_started).count();
      if (first_block && attempt == 49) {
        result.diagnostic.values_at_call50 = optimizer.values();
        result.diagnostic.finite_difference_at_call50 =
            CheckDominantGradientByFiniteDifference(
                graph, optimizer.values(), options.navigation_scales,
                diagnostic_request->finite_difference_steps);
        result.diagnostic.derivative_gate_passed =
            FirstBlockDerivativesConsistent(result.diagnostic);
        result.diagnostic.continuation_status = result.diagnostic.derivative_gate_passed
            ? "AUTHORIZED_SAME_OPTIMIZER_TO_TOTAL200"
            : "NOT_RUN_DERIVATIVE_CHECK_FAILED_OR_INCOMPLETE";
      }
      if (linked_check_result) {
        ++diagnostics.generic_convergence_count;
        diagnostics.generic_convergence_last_iteration = result.iterations;
        if (!diagnostics.stationarity_qualification_enabled) {
          result.converged = true;
          result.reason = "CONDITIONAL_LM_CONVERGED";
          result.values = optimizer.values();
          finalize_diagnostic();
          return result;
        }
        const auto qualification_started = std::chrono::steady_clock::now();
        result.last_qualification_stationarity = AuditNavigationStationarity(
            graph, optimizer.values(), options.navigation_scales,
            options.navigation_stationarity_tolerance_objective,
            options.gradient_roundoff_safety_factor);
        const double qualification_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - qualification_started).count();
        diagnostics.qualification_seconds += qualification_seconds;
        diagnostics.added_diagnostics_seconds += qualification_seconds;
        ++diagnostics.qualification_evaluation_count;
        diagnostics.qualification_last_evaluated = true;
        diagnostics.qualification_last_iteration = result.iterations;
        if (!result.last_qualification_stationarity.valid) {
          diagnostics.qualification_status =
              "INVALID:" + result.last_qualification_stationarity.reason;
          result.reason =
              "CONDITIONAL_LM_STATIONARITY_AUDIT_INVALID: " +
              result.last_qualification_stationarity.reason;
          finalize_diagnostic();
          return result;
        }
        diagnostics.qualification_last_passed =
            result.last_qualification_stationarity.stationary;
        diagnostics.qualification_status =
            diagnostics.qualification_last_passed
                ? "STATIONARY_QUALIFIED"
                : "NOT_STATIONARY_CONTINUING";
        if (diagnostics.qualification_last_passed) {
          result.converged = true;
          result.reason =
              "CONDITIONAL_LM_CONVERGED_AND_NAVIGATION_STATIONARY";
          result.values = optimizer.values();
          finalize_diagnostic();
          return result;
        }
      }
      if (first_block && attempt == 49 &&
          !result.diagnostic.derivative_gate_passed) {
        result.reason = "A11_DIAGNOSTIC_STOP_DERIVATIVE_CHECK_FAILED_OR_INCOMPLETE";
        finalize_diagnostic();
        return result;
      }
      previous = current;
    }
    if (result.convergence.stationarity_qualification_enabled &&
        result.convergence.qualification_evaluation_count > 0) {
      result.convergence.qualification_status =
          "NOT_STATIONARY_BUDGET_EXHAUSTED";
      result.reason = "CONDITIONAL_LM_STATIONARITY_NOT_REACHED";
    } else {
      result.reason = "CONDITIONAL_LM_MAX_ITERATIONS";
    }
    if (retain_recoverable_terminal_values)
      result.values = optimizer.values();
    finalize_diagnostic();
  } catch (const std::exception& error) {
    result.convergence.check_status =
        result.convergence.check_evaluated
            ? result.convergence.check_status + "_THEN_EXCEPTION"
            : "NOT_EVALUATED_EXCEPTION";
    result.reason = std::string("CONDITIONAL_LM_EXCEPTION: ") + error.what();
  }
  return result;
}

CheckedLmResult RunCheckedConditionalLm(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const CheckedLmOptions& options,
    const CheckedLmDiagnosticRequest* diagnostic_request) {
  return RunCheckedConditionalLmImpl(graph, initial, options,
                                     diagnostic_request, false);
}

CheckedLmResult RunCheckedConditionalLmWithFixedCheckpointRecovery(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const CheckedLmOptions& options, size_t max_restarts,
    size_t max_total_calls) {
  CheckedLmResult invalid;
  if (!UsesContinuousLambdaSearchV2(options.policy) || max_restarts == 0 ||
      max_total_calls == 0 || options.max_iterations <= 0) {
    invalid.reason = "CONDITIONAL_LM_FIXED_CHECKPOINT_RECOVERY_INVALID_OPTIONS";
    return invalid;
  }
  gtsam::Values checkpoint = initial;
  double checkpoint_objective = graph.error(checkpoint);
  if (!std::isfinite(checkpoint_objective) ||
      !GraphAndValuesKeysMatch(graph, checkpoint)) {
    invalid.reason = "CONDITIONAL_LM_FIXED_CHECKPOINT_RECOVERY_INVALID_START";
    return invalid;
  }

  size_t total_iterations = 0;
  int total_inner_iterations = 0;
  size_t total_iterate_calls = 0;
  size_t total_lambda_trials = 0;
  size_t total_rejected_trials = 0;
  size_t total_accepted_updates = 0;
  size_t total_no_update_returns = 0;
  for (size_t attempt = 0; attempt <= max_restarts; ++attempt) {
    if (total_iterate_calls >= max_total_calls) {
      invalid.reason =
          "CONDITIONAL_LM_FIXED_CHECKPOINT_RECOVERY_TOTAL_BUDGET_EXHAUSTED";
      invalid.fixed_checkpoint_recovery_used = attempt > 0;
      invalid.fixed_checkpoint_restart_count = attempt;
      invalid.iterations = total_iterations;
      invalid.inner_iterations = total_inner_iterations;
      invalid.convergence.iterate_call_count = total_iterate_calls;
      invalid.convergence.lambda_trial_count = total_lambda_trials;
      invalid.convergence.rejected_lambda_trial_count = total_rejected_trials;
      invalid.convergence.accepted_update_count = total_accepted_updates;
      invalid.convergence.no_update_return_count = total_no_update_returns;
      return invalid;
    }
    auto attempt_options = options;
    attempt_options.max_iterations = static_cast<int>(std::min<size_t>(
        static_cast<size_t>(options.max_iterations),
        max_total_calls - total_iterate_calls));
    CheckedLmResult result = RunCheckedConditionalLmImpl(
        graph, checkpoint, attempt_options, nullptr, true);

    total_iterations += result.iterations;
    total_inner_iterations += result.inner_iterations;
    total_iterate_calls += result.convergence.iterate_call_count;
    total_lambda_trials += result.convergence.lambda_trial_count;
    total_rejected_trials +=
        result.convergence.rejected_lambda_trial_count;
    total_accepted_updates += result.convergence.accepted_update_count;
    total_no_update_returns += result.convergence.no_update_return_count;
    result.iterations = total_iterations;
    result.inner_iterations = total_inner_iterations;
    result.convergence.iterate_call_count = total_iterate_calls;
    result.convergence.lambda_trial_count = total_lambda_trials;
    result.convergence.rejected_lambda_trial_count = total_rejected_trials;
    result.convergence.accepted_update_count = total_accepted_updates;
    result.convergence.no_update_return_count = total_no_update_returns;
    result.fixed_checkpoint_recovery_used = attempt > 0;
    result.fixed_checkpoint_restart_count = attempt;
    if (result.converged) {
      if (attempt > 0)
        result.reason =
            "CONDITIONAL_LM_CONVERGED_AFTER_FIXED_CHECKPOINT_RECOVERY";
      return result;
    }

    const bool recoverable =
        result.reason == "CONDITIONAL_LM_STATIONARITY_NOT_REACHED" ||
        result.reason == "CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED" ||
        result.reason == "CONDITIONAL_LM_MAX_ITERATIONS";
    if (!recoverable || result.values.empty()) {
      result.values.clear();
      return result;
    }
    const double next_objective = graph.error(result.values);
    const auto audit = AuditNavigationStationarity(
        graph, result.values, options.navigation_scales,
        options.navigation_stationarity_tolerance_objective,
        options.gradient_roundoff_safety_factor);
    const double allowance = Binary64ObjectiveIncreaseAllowance(
        checkpoint_objective, next_objective);
    if (!GraphAndValuesKeysMatch(graph, result.values) || !audit.valid ||
        !std::isfinite(next_objective) ||
        next_objective > checkpoint_objective + allowance) {
      result.converged = false;
      result.values.clear();
      result.reason =
          "CONDITIONAL_LM_FIXED_CHECKPOINT_RECOVERY_INVALID_CHECKPOINT";
      return result;
    }
    checkpoint = result.values;
    checkpoint_objective = next_objective;
  }
  invalid.reason =
      "CONDITIONAL_LM_FIXED_CHECKPOINT_RECOVERY_RESTART_LIMIT_EXHAUSTED";
  invalid.fixed_checkpoint_recovery_used = true;
  invalid.fixed_checkpoint_restart_count = max_restarts;
  invalid.iterations = total_iterations;
  invalid.inner_iterations = total_inner_iterations;
  invalid.convergence.iterate_call_count = total_iterate_calls;
  invalid.convergence.lambda_trial_count = total_lambda_trials;
  invalid.convergence.rejected_lambda_trial_count = total_rejected_trials;
  invalid.convergence.accepted_update_count = total_accepted_updates;
  invalid.convergence.no_update_return_count = total_no_update_returns;
  return invalid;
}

NavigationStationarityAudit AuditNavigationStationarity(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const NavigationScales& scales, double tolerance_objective,
    double roundoff_safety_factor) {
  NavigationStationarityAudit audit;
  const double scale_values[] = {
      scales.pose_rotation_rad, scales.pose_translation_m,
      scales.velocity_mps, scales.accel_bias_mps2,
      scales.gyro_bias_radps, tolerance_objective,
      roundoff_safety_factor};
  for (double value : scale_values) {
    if (!std::isfinite(value) || value < 0.0) {
      audit.reason = "navigation stationarity options are invalid";
      return audit;
    }
  }
  if (scales.pose_rotation_rad == 0.0 ||
      scales.pose_translation_m == 0.0 || scales.velocity_mps == 0.0 ||
      scales.accel_bias_mps2 == 0.0 || scales.gyro_bias_radps == 0.0 ||
      roundoff_safety_factor == 0.0) {
    audit.reason = "navigation stationarity scales must be positive";
    return audit;
  }
  try {
    const auto linearized = graph.linearize(values);
    if (!linearized || linearized->empty()) {
      audit.reason = "joint graph linearization is empty";
      return audit;
    }
    const gtsam::VectorValues gradient = linearized->gradientAtZero();
    gtsam::VectorValues absolute_factor_gradient_sum = values.zeroVectors();
    for (const auto& factor : *linearized) {
      if (!factor) {
        audit.reason = "joint graph linearization contains a null factor";
        return audit;
      }
      const gtsam::VectorValues factor_gradient = factor->gradientAtZero();
      for (const auto& key_vector : factor_gradient)
        absolute_factor_gradient_sum.at(key_vector.first) +=
            key_vector.second.cwiseAbs();
    }
    const double unit_roundoff = std::numeric_limits<double>::epsilon() / 2.0;
    const double product = static_cast<double>(linearized->size()) * unit_roundoff;
    if (!(product < 1.0)) {
      audit.reason = "binary64 gradient summation model is inapplicable";
      return audit;
    }
    const double gamma_n = product / (1.0 - product);
    const double multiplier =
        roundoff_safety_factor * (gamma_n + unit_roundoff);
    audit.stationary = true;
    for (gtsam::Key key : values.keys()) {
      const char symbol = gtsam::Symbol(key).chr();
      if (symbol == 'c') continue;
      if (!gradient.exists(key) || !absolute_factor_gradient_sum.exists(key)) {
        audit.reason = "joint gradient and Values keys differ";
        return audit;
      }
      const gtsam::Vector& grad = gradient.at(key);
      const gtsam::Vector& abs_sum = absolute_factor_gradient_sum.at(key);
      if (grad.size() != abs_sum.size() || !grad.allFinite() ||
          !abs_sum.allFinite()) {
        audit.reason = "joint navigation gradient is nonfinite";
        return audit;
      }
      for (Eigen::Index j = 0; j < grad.size(); ++j) {
        double scale = 0.0;
        double* category = nullptr;
        const char* category_name = nullptr;
        if (symbol == 'x' && grad.size() == 6) {
          if (j < 3) {
            scale = scales.pose_rotation_rad;
            category = &audit.max_pose_rotation_gradient_objective_per_rad;
            category_name = "pose_rotation";
          } else {
            scale = scales.pose_translation_m;
            category = &audit.max_pose_translation_gradient_objective_per_m;
            category_name = "pose_translation";
          }
        } else if (symbol == 'v' && grad.size() == 3) {
          scale = scales.velocity_mps;
          category = &audit.max_velocity_gradient_objective_per_mps;
          category_name = "velocity";
        } else if (symbol == 'b' && grad.size() == 6) {
          if (j < 3) {
            scale = scales.accel_bias_mps2;
            category = &audit.max_accel_bias_gradient_objective_per_mps2;
            category_name = "accelerometer_bias";
          } else {
            scale = scales.gyro_bias_radps;
            category = &audit.max_gyro_bias_gradient_objective_per_radps;
            category_name = "gyro_bias";
          }
        } else {
          audit.reason = "no declared scale for free key " +
                         gtsam::DefaultKeyFormatter(key);
          return audit;
        }
        const double native = std::abs(grad[j]);
        const double scaled = native * scale;
        const double allowance = multiplier * abs_sum[j] * scale;
        if (!std::isfinite(scaled) || !std::isfinite(allowance)) {
          audit.reason = "scaled navigation gradient is nonfinite";
          return audit;
        }
        *category = std::max(*category, native);
        if (scaled > audit.dominant_scaled_gradient_objective) {
          audit.dominant_key = key;
          audit.dominant_key_name = gtsam::DefaultKeyFormatter(key);
          audit.dominant_coordinate = static_cast<size_t>(j);
          audit.dominant_category = category_name;
          audit.dominant_native_gradient_objective = grad[j];
          audit.dominant_physical_scale = scale;
          audit.dominant_scaled_gradient_objective = scaled;
          audit.dominant_absolute_factor_gradient_sum_objective = abs_sum[j];
          audit.dominant_roundoff_allowance_objective = allowance;
        }
        audit.max_scaled_gradient_objective =
            std::max(audit.max_scaled_gradient_objective, scaled);
        audit.roundoff_allowance_objective =
            std::max(audit.roundoff_allowance_objective, allowance);
        if (scaled > tolerance_objective + allowance)
          audit.stationary = false;
      }
    }
    audit.valid = true;
    audit.reason = audit.stationary ? "STATIONARY" : "NOT_STATIONARY";
  } catch (const std::exception& error) {
    audit.reason = std::string("joint stationarity audit failed: ") +
                   error.what();
  }
  return audit;
}

namespace {

double CoordinateScale(char symbol, Eigen::Index coordinate,
                       Eigen::Index dimension, const NavigationScales& scales,
                       double amplitude_scale_m, bool allow_amplitude) {
  if (symbol == 'x' && dimension == 6)
    return coordinate < 3 ? scales.pose_rotation_rad
                          : scales.pose_translation_m;
  if (symbol == 'v' && dimension == 3) return scales.velocity_mps;
  if (symbol == 'b' && dimension == 6)
    return coordinate < 3 ? scales.accel_bias_mps2 : scales.gyro_bias_radps;
  if (allow_amplitude && symbol == 'c' && dimension == 1)
    return amplitude_scale_m;
  return std::numeric_limits<double>::quiet_NaN();
}

double MaxScaledLocalStep(const gtsam::Values& before,
                          const gtsam::Values& after,
                          const NavigationScales& scales,
                          double amplitude_scale_m, bool allow_amplitude) {
  if (before.keys() != after.keys())
    return std::numeric_limits<double>::quiet_NaN();
  const gtsam::VectorValues delta = before.localCoordinates(after);
  double maximum = 0.0;
  for (gtsam::Key key : before.keys()) {
    if (!delta.exists(key)) return std::numeric_limits<double>::quiet_NaN();
    const gtsam::Vector& vector = delta.at(key);
    const char symbol = gtsam::Symbol(key).chr();
    for (Eigen::Index j = 0; j < vector.size(); ++j) {
      const double scale = CoordinateScale(symbol, j, vector.size(), scales,
                                           amplitude_scale_m,
                                           allow_amplitude);
      if (!(scale > 0.0) || !std::isfinite(scale) ||
          !std::isfinite(vector[j]))
        return std::numeric_limits<double>::quiet_NaN();
      maximum = std::max(maximum, std::abs(vector[j]) / scale);
    }
  }
  return maximum;
}

}  // namespace

ScaledStepAudit AuditNavigationAndBiasScaledStep(
    const gtsam::Values& before_navigation,
    const gtsam::Values& after_navigation,
    const std::vector<double>& before_bias_m,
    const std::vector<double>& after_bias_m,
    const NavigationScales& scales, double bias_scale_m) {
  ScaledStepAudit audit;
  if (before_bias_m.size() != after_bias_m.size() ||
      !(bias_scale_m > 0.0) || !std::isfinite(bias_scale_m)) {
    audit.reason = "scaled-step bias inputs/scale are invalid";
    return audit;
  }
  try {
    audit.max_navigation_step = MaxScaledLocalStep(
        before_navigation, after_navigation, scales, 1.0, false);
    for (size_t i = 0; i < before_bias_m.size(); ++i) {
      if (!std::isfinite(before_bias_m[i]) ||
          !std::isfinite(after_bias_m[i])) {
        audit.reason = "scaled-step bias value is nonfinite";
        return audit;
      }
      audit.max_bias_step =
          std::max(audit.max_bias_step,
                   std::abs(after_bias_m[i] - before_bias_m[i]) /
                       bias_scale_m);
    }
    audit.max_combined_step =
        std::max(audit.max_navigation_step, audit.max_bias_step);
    if (!std::isfinite(audit.max_combined_step)) {
      audit.reason = "scaled navigation/bias step is invalid";
      return audit;
    }
    audit.valid = true;
    audit.reason = "VALID";
  } catch (const std::exception& error) {
    audit.reason = std::string("scaled-step localCoordinates failed: ") +
                   error.what();
  }
  return audit;
}

double MaxScaledValuesStep(const gtsam::Values& before,
                           const gtsam::Values& after,
                           const NavigationScales& scales,
                           double amplitude_scale_m) {
  try {
    return MaxScaledLocalStep(before, after, scales, amplitude_scale_m, true);
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

double Binary64ObjectiveIncreaseAllowance(double before, double after) {
  return 64.0 * std::numeric_limits<double>::epsilon() *
         std::max({1.0, std::abs(before), std::abs(after)});
}

}  // namespace uifgo
