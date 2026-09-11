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
#include "uifgo/nlos_inference.h"
#include <gtsam/linear/GaussianFactorGraph.h>
#include <Eigen/Eigenvalues>

namespace uifgo {

const char kImuAidedFdeProvider[] = "imu_aided_postfit_fde_v2";
const char kImuAidedFdeIdentityVersion[] =
    "UIFGO_IMU_AIDED_POSTFIT_FDE_IDENTITY_V2";

const char* FdeProvider(bool grouped, bool windowed) {
  if (grouped && windowed)
    throw std::invalid_argument("FDE_V3_V4_OPTIONS_MUTUALLY_EXCLUSIVE");
  if (windowed) return "imu_aided_windowed_fde_v4";
  return grouped ? "imu_aided_grouped_fde_v3" : kImuAidedFdeProvider;
}
const char* FdeVersion(bool grouped, bool windowed) {
  if (grouped && windowed)
    throw std::invalid_argument("FDE_V3_V4_OPTIONS_MUTUALLY_EXCLUSIVE");
  if (windowed) return "UIFGO_IMU_AIDED_WINDOWED_FDE_IDENTITY_V4";
  return grouped ? "UIFGO_IMU_AIDED_GROUPED_FDE_IDENTITY_V3" : kImuAidedFdeIdentityVersion;
}

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
  return !(options.grouped_test && options.windowed_test) &&
         options.chi2_probability == 0.99 &&
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
              << Binary64(row.residual_variance_m2) << ',' << row.test_status << ','
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

namespace {
std::string ReferenceIdentity(const gtsam::NonlinearFactorGraph& graph,
                              const gtsam::Values& values) {
  InferenceIdentityContext context;
  context.input_sha256 = context.config_sha256 = context.input_plan_sha256 =
      context.support_partition_sha256 = context.calibration_sha256 =
      context.solver_config_sha256 = "PAPER_RAW_REFERENCE_LM_V2";
  const auto id = ComputeInferenceContentIdentity(graph, values, context);
  return Sha256Hex(id.graph_linearization_sha256 + id.values_sha256);
}
void RequireGaussianGraph(const gtsam::NonlinearFactorGraph& graph) {
  for (const auto& factor : graph) {
    const auto noise = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
    if (!noise || !boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise->noiseModel()) ||
        noise->noiseModel()->isConstrained())
      throw std::invalid_argument("REFERENCE_REQUIRES_COMPLETE_GAUSSIAN_GRAPH");
  }
}
}

RawGaussianReference PrepareRawGaussianReference(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const CheckedLmOptions& options) {
  RawGaussianReference result;
  RequireGaussianGraph(graph);
  result.graph = graph;
  result.initial_identity = ReferenceIdentity(graph, initial);
  result.objective_before = graph.error(initial);
  result.solve = RunPreliminaryTightlyCoupledLm(graph, initial, options);
  if (result.solve.converged) {
    result.final_identity = ReferenceIdentity(graph, result.solve.values);
    result.objective_after = graph.error(result.solve.values);
    RequireRawGaussianReference(result, graph, initial);
  }
  return result;
}

void RequireRawGaussianReference(const RawGaussianReference& reference,
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial) {
  RequireGaussianGraph(graph);
  if (!reference.solve.converged || !std::isfinite(reference.objective_before) ||
      !std::isfinite(reference.objective_after) ||
      !GraphAndValuesKeysMatch(graph, reference.solve.values) ||
      reference.initial_identity != ReferenceIdentity(graph, initial) ||
      reference.final_identity != ReferenceIdentity(graph, reference.solve.values) ||
      reference.final_identity != ReferenceIdentity(reference.graph, reference.solve.values) ||
      graph.error(reference.solve.values) != reference.objective_after)
    throw std::invalid_argument("RAW_REFERENCE_SOLVE_OR_CONTENT_IDENTITY_MISMATCH");
}

FdeNormalization ComputeFdeNormalization(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const std::vector<FactorMeta>& metadata) {
  FdeNormalization out;
  try {
    RequireGaussianGraph(graph);
    const auto residuals = ReadScalarUwbFactorResiduals(graph, values, metadata);
    const auto linear = graph.linearize(values);
    gtsam::Ordering ordering;
    for (const auto& item : values) ordering.push_back(item.key);
    size_t rows = 0, columns = 0;
    const auto entries = linear->sparseJacobian(ordering, rows, columns);
    if (columns != values.dim() + 1)
      throw std::runtime_error("FDE_FULL_UNKNOWN_DIMENSION_MISMATCH");
    std::vector<Eigen::Triplet<double>> triplets;
    std::ostringstream canonical;
    canonical << "IMU_AIDED_POSTFIT_LINEARIZATION_V2\n" << ReferenceIdentity(graph, values);
    for (const auto& e : entries) {
      const int row = std::get<0>(e), column = std::get<1>(e);
      const double value = std::get<2>(e);
      if (!std::isfinite(value) || row < 0 || column < 0 ||
          row >= static_cast<int>(rows) || column >= static_cast<int>(columns))
        throw std::runtime_error("FDE_INVALID_WHITENED_JACOBIAN_ENTRY");
      canonical << '\n' << row << ',' << column << ',' << Binary64(value);
      if (column < static_cast<int>(columns - 1)) triplets.emplace_back(row, column, value);
    }
    Eigen::SparseMatrix<double> A(rows, columns - 1);
    A.setFromTriplets(triplets.begin(), triplets.end()); A.makeCompressed();
    std::vector<size_t> offsets;
    size_t offset = 0;
    for (size_t i = 0; i < graph.size(); ++i) {
      offsets.push_back(offset);
      if (!linear->at(i)) throw std::runtime_error("FDE_NULL_LINEAR_FACTOR");
      const size_t n = linear->at(i)->augmentedJacobian().rows();
      if (n != graph.at(i)->dim()) throw std::runtime_error("FDE_FACTOR_ROW_DIMENSION_MISMATCH");
      offset += n;
    }
    if (offset != rows) throw std::runtime_error("FDE_FACTOR_ROW_COVERAGE_MISMATCH");
    std::vector<size_t> selected;
    for (const auto& residual : residuals) {
      selected.push_back(offsets.at(residual.factor_index));
      canonical << '\n' << residual.obs_id << ',' << residual.factor_index << ',' << selected.back();
    }
    out.linearization_identity = "sha256:" + Sha256Hex(canonical.str());
    out.projection = ComputeSparseResidualProjectionDiagonal(A, selected);
    if (!out.projection.valid) return out;
    for (size_t i = 0; i < residuals.size(); ++i) {
      const double sigma = residuals[i].sigma_m;
      const double variance = sigma * sigma * out.projection.diagonal[i];
      if (!std::isfinite(variance) || variance <= 0.0)
        throw std::runtime_error("FDE_RESIDUAL_VARIANCE_NONFINITE_OR_UNRESOLVED");
      out.residual_variance_m2.push_back(variance);
    }
  } catch (const std::exception& error) {
    out.projection.valid = false;
    out.projection.reason = error.what();
    out.residual_variance_m2.clear();
  }
  return out;
}

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
  canonical << FdeVersion(options.grouped_test, options.windowed_test) << '\n';
  Field(&canonical, FdeProvider(options.grouped_test, options.windowed_test));
  if (options.grouped_test) Field(&canonical, "MAXIMAL_PLANNED_LINK_GAP_V1_FULL_P_GLS_SIGN_P099");
  if (options.windowed_test) {
    Field(&canonical, "FDE_WINDOWED_DYADIC_BONFERRONI_MERGE_V4");
    Field(&canonical, "BASE_MAX_4_NMIN_DYADIC_CAP64_STRIDE_HALF_RIGHT_TAIL");
    Field(&canonical, "CHAIN_LOCAL_BONFERRONI_ALL_EXECUTED_WINDOWS_ALPHA_0P01");
    Field(&canonical, "FULL_PWW_GLS_NEGATIVE_EXACT_OBS_UNION_MERGE");
  }
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
                         double residual_variance_m2,
                         const FdeOptions& options,
                         FdeObservationRecord* record) {
  if (!record || !OptionsValid(options) || !std::isfinite(residual_m) ||
      !std::isfinite(factor_sigma_m) || !(factor_sigma_m > 0.0) ||
      !std::isfinite(residual_variance_m2) || !(residual_variance_m2 > 0.0))
    throw std::invalid_argument("FDE residual classification input invalid");
  record->tested = true;
  record->residual_m = residual_m;
  record->factor_sigma_m = factor_sigma_m;
  record->residual_variance_m2 = residual_variance_m2;
  record->measurement_standardized_residual_diagnostic = residual_m / factor_sigma_m;
  record->standardized_residual = residual_m / std::sqrt(residual_variance_m2);
  record->test_status = "TESTED_POSTFIT_GAUSSIAN_V2";
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
  partition.provider = FdeProvider(options.grouped_test, options.windowed_test);
  partition.hash_algorithm = "SHA-256";
  partition.partition_rule_version = options.grouped_test ? "FDE_GROUPED_GAP_THEN_TEMPORAL_V3" : "FDE_TEMPORAL_RUN_V1";
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

SegmentRefitResult ReuseEmptyFdeReference(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const std::vector<FactorMeta>& metadata, const PaperInputPlan& plan,
    const Config& cfg, const FdeResult& fde, const RefitOptions& options) {
  SegmentRefitResult result;
  try {
    if (!fde.success() || !fde.reference.converged ||
        !fde.normalization.projection.valid || !fde.partition.segments.empty() ||
        fde.partition.provider != FdeProvider(cfg.fde_grouped_test,
                                              cfg.fde_windowed_test) ||
        fde.planned_count == 0 || fde.tested_count != fde.planned_count)
      throw std::invalid_argument("SUCCESS_EMPTY_PRECONDITIONS_FAILED");
    RequireRawGaussianReference(fde.raw_reference, graph, initial);
    if (ReferenceIdentity(graph, fde.reference.values) != fde.raw_reference.final_identity)
      throw std::invalid_argument("SUCCESS_EMPTY_REFERENCE_VALUES_MISMATCH");
    size_t tested = 0;
    for (const auto& row : fde.observations) {
      if (!row.valid || !row.planned) continue;
      if (!row.tested || !std::isfinite(row.residual_variance_m2) ||
          row.residual_variance_m2 <= 0.0)
        throw std::invalid_argument("SUCCESS_EMPTY_OBSERVATION_TEST_INCOMPLETE");
      ++tested;
    }
    if (tested != fde.planned_count) throw std::invalid_argument("SUCCESS_EMPTY_COUNT_MISMATCH");
    result = SegmentRefitter(options).RunFrozenCandidatePolicy(
        graph, fde.reference.values, metadata, plan, cfg, fde.partition, {}, false);
    if (!result.successful() || !result.iterations.empty() ||
        ReferenceIdentity(result.graph, result.values) != fde.raw_reference.final_identity)
      throw std::invalid_argument("SUCCESS_EMPTY_GRAPH_RECONSTRUCTION_MISMATCH:" + result.reason);
    result.graph = fde.raw_reference.graph;
    result.values = fde.raw_reference.solve.values;
    result.status = SegmentRefitStatus::SUCCESS_EMPTY;
    result.reason = "SUCCESS_EMPTY_REFERENCE_REUSED_ALTERNATING_CONDITIONS_NOT_APPLICABLE";
  } catch (const std::exception& error) {
    result.status = SegmentRefitStatus::INVALID_INPUT;
    result.reason = error.what();
    result.graph = {}; result.values.clear();
  }
  return result;
}

FdeResult ImuAidedFdeSupportProvider::Run(
    const gtsam::NonlinearFactorGraph& base_graph,
    const gtsam::Values& base_values,
    const std::vector<FactorMeta>& base_uwb_factor_metadata,
    const PaperInputPlan& plan, const Config& cfg,
    const FdeContext& context, const RawGaussianReference* prepared) const {
  FdeResult result;
  result.provider = FdeProvider(options_.grouped_test, options_.windowed_test);
  result.provider_version = FdeVersion(options_.grouped_test,
                                       options_.windowed_test);
  if (options_.windowed_test) result.windowed_status = "WINDOWED_NOT_RUN";
  result.chi2_threshold = Chi2inv(options_.chi2_probability,
                                  options_.chi2_degrees_of_freedom);
  auto fail = [&](FdeStatus status, const std::string& reason) {
    result.status = status;
    result.reason = reason;
    for (auto& row : result.observations)
      if (row.valid && row.planned && !row.tested)
        row.test_status = "NOT_TESTED_FDE_FAILED:" + reason;
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
    result.partition.provider = FdeProvider(options_.grouped_test,
                                            options_.windowed_test);
    result.partition.hash_algorithm = "SHA-256";
    result.partition.partition_rule_version = options_.windowed_test
        ? "FDE_WINDOWED_DYADIC_BONFERRONI_MERGE_V4"
        : "FDE_TEMPORAL_RUN_V1";
    result.partition.input_plan_hash = context.input_plan_hash;
    result.partition.source_hash = context.source_hash;
    result.partition.config_hash = context.config_hash;
    result.partition.calibration_hash = context.calibration_hash;
    result.partition.solver_config_hash = context.solver_config_hash;
    result.partition.discovery_context_hash = result.identity_hash;
    result.raw_reference = prepared ? *prepared : PrepareRawGaussianReference(
        base_graph, base_values, options_.preliminary_lm);
    result.reference = result.raw_reference.solve;
    if (!result.reference.converged)
      return fail(FdeStatus::REFERENCE_LM_FAILED,
                  "PRELIMINARY_LM_FAILED:" + result.reference.reason);

    RequireRawGaussianReference(result.raw_reference, base_graph, base_values);
    result.normalization = ComputeFdeNormalization(
        base_graph, result.reference.values, base_uwb_factor_metadata);
    if (!result.normalization.projection.valid)
      return fail(FdeStatus::OBSERVATION_TEST_FAILED,
                  result.normalization.projection.reason);
    const auto residuals = ReadScalarUwbFactorResiduals(
        base_graph, result.reference.values, base_uwb_factor_metadata);
    size_t residual_index = 0;
    for (const auto& residual : residuals) {
      auto& row = result.observations.at(row_by_obs.at(residual.obs_id));
      ClassifyFdeResidual(residual.residual_m, residual.sigma_m,
                          result.normalization.residual_variance_m2.at(residual_index++), options_,
                          &row);
      ++result.tested_count;
      result.fault_count += row.fault_detected;
      result.positive_candidate_count += row.nlos_candidate;
    }
    if (result.tested_count != result.planned_count)
      return fail(FdeStatus::OBSERVATION_TEST_FAILED,
                  "not every planned observation was tested exactly once");
    if (options_.windowed_test) {
      ApplyFullGraphWindowedFde(base_graph, result.reference.values,
                                base_uwb_factor_metadata, options_, context,
                                &result);
    } else if (options_.grouped_test) {
      ApplyFullGraphGroupedFde(base_graph, result.reference.values, base_uwb_factor_metadata, options_, &result);
    }
    if (!options_.windowed_test) {
      result.partition = BuildFdeSupportPartition(
          &result.observations, options_, context, &result.raw_run_count,
          &result.filtered_run_count);
    }
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
