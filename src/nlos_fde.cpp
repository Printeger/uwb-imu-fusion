#include "uifgo/nlos_fde.h"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

#include "uifgo/hash_utils.h"
#include "uifgo/optimizer.h"

namespace uifgo {

const char kImuAidedFdeProvider[] = "imu_aided_residual_fde_v1";
const char kImuAidedFdeIdentityVersion[] =
    "UIFGO_IMU_AIDED_RESIDUAL_FDE_IDENTITY_V1";

namespace {

std::string Binary64(double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "binary64 required");
  std::memcpy(&bits, &value, sizeof(bits));
  std::ostringstream out;
  out << std::hex << bits;
  return out.str();
}

void Field(std::ostringstream* out, const std::string& value) {
  *out << value.size() << ':' << value << '\n';
}

bool OptionsValid(const FdeOptions& options) {
  return options.chi2_probability == 0.99 &&
         options.chi2_degrees_of_freedom == 1 &&
         std::isfinite(options.gap_threshold_s) &&
         options.gap_threshold_s >= 0.0 && options.minimum_count > 0 &&
         std::isfinite(options.minimum_duration_s) &&
         options.minimum_duration_s >= 0.0 &&
         options.preliminary_lm.max_iterations > 0 &&
         std::isfinite(options.preliminary_lm.relative_tolerance) &&
         options.preliminary_lm.relative_tolerance >= 0.0 &&
         std::isfinite(options.preliminary_lm.absolute_tolerance) &&
         options.preliminary_lm.absolute_tolerance >= 0.0;
}

bool ContextValid(const FdeContext& context) {
  return !context.input_plan_hash.empty() && !context.source_hash.empty() &&
         !context.config_hash.empty() && !context.calibration_hash.empty() &&
         !context.solver_config_hash.empty() &&
         !context.common_preparation_id.empty() &&
         !context.physical_graph_hash.empty() &&
         !context.initial_values_hash.empty();
}

std::string SnapshotHash(const std::vector<FdeObservationRecord>& records,
                         const std::string& identity) {
  std::ostringstream canonical;
  canonical << kImuAidedFdeIdentityVersion << "\nSNAPSHOT\n" << identity
            << '\n';
  for (const auto& row : records) {
    canonical << row.obs_id << ',' << row.tag_id << ',' << row.anchor_id << ','
              << Binary64(row.sensor_time) << ',' << row.valid << ','
              << row.planned << ',' << row.factor_index << ',' << row.tested
              << ',' << Binary64(row.residual_m) << ','
              << Binary64(row.factor_sigma_m) << ','
              << Binary64(row.standardized_residual) << ','
              << Binary64(row.statistic) << ',' << row.fault_detected << ','
              << row.positive_excess << ',' << row.nlos_candidate << '\n';
  }
  return "sha256:" + Sha256Hex(canonical.str());
}

std::string PartitionHash(const SupportPartition& partition) {
  std::ostringstream canonical;
  canonical << kImuAidedFdeIdentityVersion << "\nPARTITION\n"
            << partition.discovery_context_hash << '\n'
            << partition.discovery_snapshot_hash << '\n';
  for (const auto& segment : partition.segments) {
    canonical << segment.segment_ordinal << ',' << segment.segment_id << ','
              << segment.tag_id << ',' << segment.anchor_id << ','
              << Binary64(segment.start_time) << ','
              << Binary64(segment.end_time) << ','
              << segment.observation_count << ','
              << Binary64(segment.duration) << '\n';
    for (const auto obs_id : segment.obs_ids) canonical << obs_id << ',';
    canonical << '\n';
  }
  return "sha256:" + Sha256Hex(canonical.str());
}

std::string RawSegmentId(const FdeObservationRecord& first,
                         const FdeObservationRecord& last) {
  std::ostringstream canonical;
  canonical << kImuAidedFdeIdentityVersion << "\nRAW_RUN\n" << first.tag_id
            << ',' << first.anchor_id << ',' << first.obs_id << ','
            << last.obs_id << ',' << Binary64(first.sensor_time) << ','
            << Binary64(last.sensor_time);
  return "fde-raw-" + Sha256Hex(canonical.str());
}

}  // namespace

CheckedLmResult RunPreliminaryTightlyCoupledLm(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& initial_values, const CheckedLmOptions& options) {
  return RunCheckedConditionalLm(graph, initial_values, options);
}

std::vector<PreliminaryUwbResidual> ReadScalarUwbFactorResiduals(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const std::vector<FactorMeta>& metadata) {
  if (graph.empty() || values.empty() ||
      !GraphAndValuesKeysMatch(graph, values))
    throw std::invalid_argument("physical graph/Values are invalid");
  std::set<size_t> factor_indices;
  std::set<std::uint64_t> obs_ids;
  std::vector<PreliminaryUwbResidual> output;
  for (const auto& meta : metadata) {
    if (meta.factor_type != "uwb_range")
      throw std::invalid_argument("non-UWB metadata in UWB residual reader");
    if (meta.obs_id == 0 || meta.factor_index >= graph.size() ||
        !graph.at(meta.factor_index) ||
        !factor_indices.insert(meta.factor_index).second ||
        !obs_ids.insert(meta.obs_id).second)
      throw std::invalid_argument("UWB metadata identity is invalid");
    const auto keys = graph.at(meta.factor_index)->keys();
    if (std::vector<gtsam::Key>(keys.begin(), keys.end()) != meta.keys)
      throw std::invalid_argument("UWB metadata keys differ from factor");
    const auto factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(
        graph.at(meta.factor_index));
    if (!factor)
      throw std::invalid_argument("UWB factor is not a NoiseModelFactor");
    const gtsam::Vector residual = factor->unwhitenedError(values);
    const gtsam::Vector sigmas = factor->noiseModel()->sigmas();
    if (residual.size() != 1 || sigmas.size() != 1 ||
        !residual.allFinite() || !sigmas.allFinite() ||
        !(sigmas[0] > 0.0))
      throw std::runtime_error("scalar UWB residual/noise is invalid");
    output.push_back(
        {meta.factor_index, meta.obs_id, residual[0], sigmas[0]});
  }
  if (output.empty())
    throw std::invalid_argument("UWB residual reader has no metadata");
  std::sort(output.begin(), output.end(), [](const auto& a, const auto& b) {
    return a.factor_index < b.factor_index;
  });
  return output;
}

const char* FdeStatusName(FdeStatus status) {
  switch (status) {
    case FdeStatus::SUCCESS: return "SUCCESS";
    case FdeStatus::INVALID_INPUT: return "INVALID_INPUT";
    case FdeStatus::REFERENCE_LM_FAILED: return "REFERENCE_LM_FAILED";
    case FdeStatus::OBSERVATION_TEST_FAILED:
      return "OBSERVATION_TEST_FAILED";
    case FdeStatus::PARTITION_INVALID: return "PARTITION_INVALID";
  }
  return "UNKNOWN";
}

std::string ComputeFdeIdentity(const FdeOptions& options,
                               const FdeContext& context) {
  std::ostringstream canonical;
  canonical << kImuAidedFdeIdentityVersion << '\n';
  Field(&canonical, kImuAidedFdeProvider);
  Field(&canonical, Binary64(options.chi2_probability));
  Field(&canonical, std::to_string(options.chi2_degrees_of_freedom));
  Field(&canonical, Binary64(Chi2inv(options.chi2_probability,
                                    options.chi2_degrees_of_freedom)));
  Field(&canonical, Binary64(options.gap_threshold_s));
  Field(&canonical, std::to_string(options.minimum_count));
  Field(&canonical, Binary64(options.minimum_duration_s));
  Field(&canonical, std::to_string(options.preliminary_lm.max_iterations));
  Field(&canonical, Binary64(options.preliminary_lm.relative_tolerance));
  Field(&canonical, Binary64(options.preliminary_lm.absolute_tolerance));
  Field(&canonical,
        ConditionalLmPolicyName(options.preliminary_lm.policy));
  Field(&canonical, context.input_plan_hash);
  Field(&canonical, context.source_hash);
  Field(&canonical, context.config_hash);
  Field(&canonical, context.calibration_hash);
  Field(&canonical, context.solver_config_hash);
  Field(&canonical, context.common_preparation_id);
  Field(&canonical, context.physical_graph_hash);
  Field(&canonical, context.initial_values_hash);
  return "sha256:" + Sha256Hex(canonical.str());
}

void ClassifyFdeResidual(double residual_m, double factor_sigma_m,
                         const FdeOptions& options,
                         FdeObservationRecord* record) {
  if (!record || !OptionsValid(options) || !std::isfinite(residual_m) ||
      !std::isfinite(factor_sigma_m) || !(factor_sigma_m > 0.0))
    throw std::invalid_argument("FDE residual classification input invalid");
  record->tested = true;
  record->residual_m = residual_m;
  record->factor_sigma_m = factor_sigma_m;
  record->standardized_residual = residual_m / factor_sigma_m;
  record->statistic = record->standardized_residual *
                      record->standardized_residual;
  const double threshold = Chi2inv(options.chi2_probability,
                                   options.chi2_degrees_of_freedom);
  if (!std::isfinite(record->standardized_residual) ||
      !std::isfinite(record->statistic) || !std::isfinite(threshold))
    throw std::runtime_error("FDE residual classification is nonfinite");
  record->fault_detected = record->statistic > threshold;
  record->positive_excess = residual_m < 0.0;
  record->nlos_candidate =
      record->fault_detected && record->positive_excess;
  record->candidate_filter_reason = record->nlos_candidate
                                        ? "PENDING_TEMPORAL_FILTER"
                                        : (record->fault_detected
                                               ? "FAULT_NON_POSITIVE_EXCESS"
                                               : "NOT_FAULT");
}

SupportPartition BuildFdeSupportPartition(
    std::vector<FdeObservationRecord>* observations,
    const FdeOptions& options, const FdeContext& context,
    size_t* raw_run_count, size_t* filtered_run_count) {
  if (!observations || !OptionsValid(options) || !ContextValid(context))
    throw std::invalid_argument("FDE partition input/options/context invalid");
  std::set<std::uint64_t> unique;
  for (const auto& row : *observations) {
    if (row.obs_id == 0 || !unique.insert(row.obs_id).second ||
        !std::isfinite(row.sensor_time))
      throw std::invalid_argument("FDE observation identity/time invalid");
    if (row.valid && row.planned && !row.tested)
      throw std::invalid_argument("planned FDE observation was not tested");
  }

  std::vector<size_t> tested;
  for (size_t i = 0; i < observations->size(); ++i)
    if ((*observations)[i].valid && (*observations)[i].planned)
      tested.push_back(i);
  std::sort(tested.begin(), tested.end(), [&](size_t a, size_t b) {
    const auto& x = (*observations)[a];
    const auto& y = (*observations)[b];
    return std::tie(x.tag_id, x.anchor_id, x.sensor_time, x.obs_id) <
           std::tie(y.tag_id, y.anchor_id, y.sensor_time, y.obs_id);
  });

  struct Run {
    std::vector<size_t> rows;
    std::string raw_id;
  };
  std::vector<Run> runs;
  Run open;
  auto close = [&]() {
    if (open.rows.empty()) return;
    open.raw_id = RawSegmentId((*observations)[open.rows.front()],
                              (*observations)[open.rows.back()]);
    for (const size_t row : open.rows)
      (*observations)[row].raw_segment_id = open.raw_id;
    runs.push_back(std::move(open));
    open = Run();
  };
  const FdeObservationRecord* previous_candidate = nullptr;
  for (const size_t index : tested) {
    auto& row = (*observations)[index];
    if (!row.nlos_candidate) {
      close();
      previous_candidate = nullptr;
      continue;
    }
    const bool same_link = previous_candidate &&
                           previous_candidate->tag_id == row.tag_id &&
                           previous_candidate->anchor_id == row.anchor_id;
    const bool gap_break = same_link &&
        row.sensor_time - previous_candidate->sensor_time >
            options.gap_threshold_s;
    if (!same_link || gap_break) close();
    open.rows.push_back(index);
    previous_candidate = &row;
  }
  close();

  SupportPartition partition;
  partition.schema = "uifgo_fde_support_v1";
  partition.provider = kImuAidedFdeProvider;
  partition.hash_algorithm = "SHA-256";
  partition.partition_rule_version = "FDE_TEMPORAL_RUN_V1";
  partition.input_plan_hash = context.input_plan_hash;
  partition.source_hash = context.source_hash;
  partition.config_hash = context.config_hash;
  partition.calibration_hash = context.calibration_hash;
  partition.solver_config_hash = context.solver_config_hash;
  partition.discovery_context_hash = ComputeFdeIdentity(options, context);
  partition.discovery_snapshot_hash =
      SnapshotHash(*observations, partition.discovery_context_hash);

  size_t filtered = 0;
  for (const auto& run : runs) {
    const auto& first = (*observations)[run.rows.front()];
    const auto& last = (*observations)[run.rows.back()];
    const size_t count = run.rows.size();
    const double duration = last.sensor_time - first.sensor_time;
    const bool count_ok = count >= options.minimum_count;
    const bool duration_ok = duration >= options.minimum_duration_s;
    if (!count_ok || !duration_ok) {
      ++filtered;
      const std::string reason =
          !count_ok && !duration_ok
              ? "FILTERED_MIN_COUNT_AND_DURATION"
              : (!count_ok ? "FILTERED_MIN_COUNT"
                           : "FILTERED_MIN_DURATION");
      for (const size_t row : run.rows)
        (*observations)[row].candidate_filter_reason = reason;
      continue;
    }
    SupportSegment segment;
    segment.segment_ordinal = partition.segments.size();
    segment.tag_id = first.tag_id;
    segment.anchor_id = first.anchor_id;
    segment.start_time = first.sensor_time;
    segment.end_time = last.sensor_time;
    segment.observation_count = count;
    segment.duration = duration;
    segment.merge_snapshot_mean_m = 0.0;
    segment.short_support_debug = false;
    segment.parent_segment_ids = {run.raw_id};
    std::ostringstream canonical;
    canonical << kImuAidedFdeIdentityVersion << "\nSEGMENT\n"
              << partition.discovery_context_hash << '\n' << run.raw_id;
    segment.segment_id = "fde-segment-" + Sha256Hex(canonical.str());
    for (const size_t row : run.rows) {
      auto& observation = (*observations)[row];
      observation.segment_id = segment.segment_id;
      observation.segment_ordinal = segment.segment_ordinal;
      observation.candidate_filter_reason = "RETAINED";
      segment.obs_ids.push_back(observation.obs_id);
    }
    partition.segments.push_back(std::move(segment));
  }
  partition.partition_hash = PartitionHash(partition);
  if (raw_run_count) *raw_run_count = runs.size();
  if (filtered_run_count) *filtered_run_count = filtered;
  return partition;
}

FdeResult ImuAidedFdeSupportProvider::Run(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& base_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const FdeContext& context) const {
  FdeResult result;
  result.chi2_threshold = Chi2inv(options_.chi2_probability,
                                  options_.chi2_degrees_of_freedom);
  auto fail = [&](FdeStatus status, const std::string& reason) {
    result.status = status;
    result.reason = reason;
    return result;
  };
  try {
    result.observations.reserve(plan.observations.size());
    std::unordered_map<std::uint64_t, size_t> row_by_obs;
    for (const auto& input : plan.observations) {
      FdeObservationRecord row;
      row.obs_id = input.obs_id;
      row.source_frame_index = input.source_frame_index;
      row.source_message_index = input.source_message_index;
      row.source_range_index = input.source_range_index;
      row.source_observation_index = input.source_observation_index;
      row.tag_id = input.tag_id;
      row.anchor_id = input.anchor_id;
      row.sensor_time = input.sensor_time;
      row.valid = input.valid;
      row.planned = input.planned;
      row.keyframe_id = input.keyframe_id;
      row.ledger_nominal_sigma_m = input.nominal_sigma;
      row.candidate_filter_reason = input.valid && input.planned
                                          ? "PENDING_TEST"
                                          : (input.valid ? "NOT_PLANNED"
                                                         : "INVALID_INPUT_ROW");
      if (input.obs_id == 0 ||
          !row_by_obs.emplace(input.obs_id, result.observations.size()).second)
        return fail(FdeStatus::INVALID_INPUT,
                    "plan contains invalid or duplicate obs_id");
      result.observations.push_back(std::move(row));
      result.planned_count += input.valid && input.planned;
    }
    if (!OptionsValid(options_) || !ContextValid(context) ||
        cfg.nlos_mode != "imu_aided_fde" || cfg.calib_lever ||
        cfg.calib_anchor || cfg.calib_range_bias || cfg.calib_td ||
        base_graph.empty() || base_values.empty() ||
        !GraphAndValuesKeysMatch(base_graph, base_values) ||
        result.planned_count == 0)
      return fail(FdeStatus::INVALID_INPUT,
                  "FDE requires fixed calibration, valid identity, graph/Values and planned observations");

    std::set<size_t> factors;
    std::set<std::uint64_t> metadata_obs;
    for (const auto& meta : base_uwb_factor_metadata) {
      const auto found = row_by_obs.find(meta.obs_id);
      if (meta.factor_type != "uwb_range" || meta.obs_id == 0 ||
          found == row_by_obs.end() || meta.factor_index >= base_graph.size() ||
          !base_graph.at(meta.factor_index) ||
          !factors.insert(meta.factor_index).second ||
          !metadata_obs.insert(meta.obs_id).second ||
          !result.observations[found->second].valid ||
          !result.observations[found->second].planned ||
          std::vector<gtsam::Key>(
              base_graph.at(meta.factor_index)->keys().begin(),
              base_graph.at(meta.factor_index)->keys().end()) != meta.keys)
        return fail(FdeStatus::INVALID_INPUT,
                    "planned observation/factor metadata identity is invalid");
      result.observations[found->second].factor_index = meta.factor_index;
    }
    if (metadata_obs.size() != result.planned_count)
      return fail(FdeStatus::INVALID_INPUT,
                  "planned observation/factor mapping is not one-to-one");

    result.identity_hash = ComputeFdeIdentity(options_, context);
    result.partition.schema = "uifgo_fde_support_v1";
    result.partition.provider = kImuAidedFdeProvider;
    result.partition.hash_algorithm = "SHA-256";
    result.partition.partition_rule_version = "FDE_TEMPORAL_RUN_V1";
    result.partition.input_plan_hash = context.input_plan_hash;
    result.partition.source_hash = context.source_hash;
    result.partition.config_hash = context.config_hash;
    result.partition.calibration_hash = context.calibration_hash;
    result.partition.solver_config_hash = context.solver_config_hash;
    result.partition.discovery_context_hash = result.identity_hash;
    result.reference = RunPreliminaryTightlyCoupledLm(
        base_graph, base_values, options_.preliminary_lm);
    if (!result.reference.converged)
      return fail(FdeStatus::REFERENCE_LM_FAILED,
                  "PRELIMINARY_LM_FAILED:" + result.reference.reason);

    const auto residuals = ReadScalarUwbFactorResiduals(
        base_graph, result.reference.values, base_uwb_factor_metadata);
    for (const auto& residual : residuals) {
      auto& row = result.observations.at(row_by_obs.at(residual.obs_id));
      ClassifyFdeResidual(residual.residual_m, residual.sigma_m, options_,
                          &row);
      ++result.tested_count;
      result.fault_count += row.fault_detected;
      result.positive_candidate_count += row.nlos_candidate;
    }
    if (result.tested_count != result.planned_count)
      return fail(FdeStatus::OBSERVATION_TEST_FAILED,
                  "not every planned observation was tested exactly once");
    result.partition = BuildFdeSupportPartition(
        &result.observations, options_, context, &result.raw_run_count,
        &result.filtered_run_count);
    result.retained_segment_count = result.partition.segments.size();
    if (result.partition.discovery_context_hash != result.identity_hash)
      return fail(FdeStatus::PARTITION_INVALID,
                  "partition/provider identity mismatch");
    result.status = FdeStatus::SUCCESS;
    result.reason = result.partition.segments.empty()
                        ? "REFERENCE_AND_ALL_TESTS_COMPLETE_EMPTY_SUPPORT"
                        : "REFERENCE_AND_ALL_TESTS_COMPLETE";
    return result;
  } catch (const std::invalid_argument& error) {
    return fail(FdeStatus::INVALID_INPUT, error.what());
  } catch (const std::exception& error) {
    return fail(FdeStatus::OBSERVATION_TEST_FAILED, error.what());
  }
}

}  // namespace uifgo
