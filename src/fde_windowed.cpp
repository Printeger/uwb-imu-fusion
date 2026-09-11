#include "uifgo/nlos_fde.h"

#include <gtsam/linear/GaussianFactorGraph.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

#include "uifgo/hash_utils.h"

namespace uifgo {
namespace {

constexpr const char* kWindowedPartitionRule =
    "FDE_WINDOWED_DYADIC_BONFERRONI_MERGE_V4";

std::string Binary64Windowed(double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "binary64 required");
  std::memcpy(&bits, &value, sizeof(bits));
  std::ostringstream out;
  out << std::hex << bits;
  return out.str();
}

std::string HashObsIds(const std::vector<std::uint64_t>& ids) {
  std::ostringstream canonical;
  canonical << FdeVersion(false, true) << "\nOBS_IDS\n";
  for (const auto id : ids) canonical << id << '\n';
  return "sha256:" + Sha256Hex(canonical.str());
}

std::string ObjectId(const char* kind, const std::string& identity,
                     int tag_id, int anchor_id,
                     const std::vector<std::uint64_t>& ids) {
  std::ostringstream canonical;
  canonical << FdeVersion(false, true) << '\n' << kWindowedPartitionRule
            << '\n' << kind << '\n' << identity << '\n' << tag_id << ','
            << anchor_id << '\n';
  for (const auto id : ids) canonical << id << '\n';
  return std::string("fde-v4-") + kind + "-" + Sha256Hex(canonical.str());
}

std::string SnapshotHashV4(const std::vector<FdeObservationRecord>& rows,
                           const std::string& identity) {
  std::ostringstream canonical;
  canonical << FdeVersion(false, true) << "\nSNAPSHOT\n" << identity << '\n';
  for (const auto& row : rows) {
    canonical << row.obs_id << ',' << row.tag_id << ',' << row.anchor_id << ','
              << Binary64Windowed(row.sensor_time) << ',' << row.valid << ','
              << row.planned << ',' << row.factor_index << ',' << row.tested
              << ',' << Binary64Windowed(row.residual_m) << ','
              << Binary64Windowed(row.factor_sigma_m) << ','
              << Binary64Windowed(row.residual_variance_m2) << ','
              << row.test_status << ','
              << Binary64Windowed(row.standardized_residual) << ','
              << Binary64Windowed(row.statistic) << ',' << row.fault_detected
              << ',' << row.positive_excess << '\n';
  }
  return "sha256:" + Sha256Hex(canonical.str());
}

std::string PartitionHashV4(const SupportPartition& partition) {
  std::ostringstream canonical;
  canonical << FdeVersion(false, true) << "\nPARTITION\n"
            << kWindowedPartitionRule << '\n'
            << partition.discovery_context_hash << '\n'
            << partition.discovery_snapshot_hash << '\n';
  for (const auto& segment : partition.segments) {
    canonical << segment.segment_ordinal << ',' << segment.segment_id << ','
              << segment.tag_id << ',' << segment.anchor_id << ','
              << Binary64Windowed(segment.start_time) << ','
              << Binary64Windowed(segment.end_time) << ','
              << segment.observation_count << ','
              << Binary64Windowed(segment.duration) << '\n';
    for (const auto id : segment.obs_ids) canonical << id << ',';
    canonical << '\n';
    for (const auto& id : segment.parent_segment_ids) canonical << id << ',';
    canonical << '\n';
  }
  return "sha256:" + Sha256Hex(canonical.str());
}

struct ChainRows {
  std::vector<size_t> observation_rows;
  FdeContinuousChain record;
};

std::vector<std::pair<size_t, size_t>> WindowBank(size_t count,
                                                  size_t base_count) {
  std::vector<std::pair<size_t, size_t>> windows;
  const size_t cap = std::min<size_t>(64, count);
  for (size_t size = base_count; size <= cap;) {
    const size_t stride = std::max<size_t>(1, size / 2);
    size_t last_start = 0;
    bool emitted = false;
    for (size_t start = 0; start + size <= count; start += stride) {
      windows.emplace_back(start, size);
      last_start = start;
      emitted = true;
      if (start > count - size - std::min(stride, count - size)) break;
    }
    const size_t tail = count - size;
    if (emitted && last_start != tail) windows.emplace_back(tail, size);
    if (size > cap / 2) break;
    size *= 2;
  }
  return windows;
}

double GlsSignedResidual(const Eigen::VectorXd& e,
                         const Eigen::VectorXd& direction,
                         const Eigen::MatrixXd& covariance,
                         const FdeQuadraticTest& test) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(
      (covariance + covariance.transpose()) * 0.5);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() ||
      !eig.eigenvectors().allFinite())
    throw std::runtime_error("WINDOWED_FDE_GLS_EIGEN_FAILED");
  const Eigen::VectorXd de = eig.eigenvectors().transpose() * direction;
  const Eigen::VectorXd ee = eig.eigenvectors().transpose() * e;
  double numerator = 0.0;
  double denominator = 0.0;
  for (int j = 0; j < ee.size(); ++j) {
    if (eig.eigenvalues()[j] <= test.rank_threshold) continue;
    numerator += de[j] * ee[j] / eig.eigenvalues()[j];
    denominator += de[j] * de[j] / eig.eigenvalues()[j];
  }
  if (!std::isfinite(numerator) || !std::isfinite(denominator) ||
      denominator <= 0.0)
    throw std::runtime_error("WINDOWED_FDE_GLS_DIRECTION_UNAVAILABLE");
  return numerator / denominator;
}

}  // namespace

void ApplyWindowedFdeTests(
    std::vector<FdeObservationRecord>* observations,
    const Eigen::MatrixXd& covariance,
    const std::vector<std::uint64_t>& covariance_obs_ids,
    const FdeOptions& options, const FdeContext& context, FdeResult* result) {
  if (!observations || !result || !options.windowed_test ||
      options.grouped_test || options.chi2_probability != 0.99 ||
      options.chi2_degrees_of_freedom != 1 ||
      !std::isfinite(options.gap_threshold_s) ||
      options.gap_threshold_s < 0.0 || options.minimum_count == 0 ||
      !std::isfinite(options.minimum_duration_s) ||
      options.minimum_duration_s < 0.0 ||
      covariance.rows() != static_cast<int>(covariance_obs_ids.size()) ||
      covariance.cols() != covariance.rows() || !covariance.allFinite())
    throw std::invalid_argument("WINDOWED_FDE_COVARIANCE_OPTIONS_INVALID");
  const RecoverabilityOptions numeric;
  if ((covariance - covariance.transpose()).norm() >
      numeric.symmetry_absolute_tolerance +
          numeric.symmetry_relative_tolerance * covariance.norm())
    throw std::invalid_argument("WINDOWED_FDE_COVARIANCE_ASYMMETRIC");

  std::map<std::uint64_t, size_t> covariance_row;
  for (size_t j = 0; j < covariance_obs_ids.size(); ++j) {
    if (!covariance_row.emplace(covariance_obs_ids[j], j).second)
      throw std::invalid_argument("WINDOWED_FDE_DUPLICATE_COVARIANCE_ID");
  }
  std::vector<size_t> order;
  std::set<std::uint64_t> unique;
  for (size_t j = 0; j < observations->size(); ++j) {
    auto& row = observations->at(j);
    if (!row.valid || !row.planned) continue;
    if (!row.tested || !std::isfinite(row.residual_m) ||
        !std::isfinite(row.factor_sigma_m) || row.factor_sigma_m <= 0.0 ||
        !std::isfinite(row.sensor_time) || !covariance_row.count(row.obs_id) ||
        !unique.insert(row.obs_id).second)
      throw std::invalid_argument("WINDOWED_FDE_OBSERVATION_MAPPING_INVALID");
    order.push_back(j);
    row.nlos_candidate = false;
    row.raw_segment_id.clear();
    row.segment_id.clear();
    row.segment_ordinal = std::numeric_limits<size_t>::max();
    row.candidate_filter_reason = "WINDOW_NOT_RETAINED";
  }
  if (order.size() != covariance_obs_ids.size())
    throw std::invalid_argument("WINDOWED_FDE_ROW_COVERAGE_INVALID");
  std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    const auto& x = observations->at(a);
    const auto& y = observations->at(b);
    return std::tie(x.tag_id, x.anchor_id, x.sensor_time, x.obs_id) <
           std::tie(y.tag_id, y.anchor_id, y.sensor_time, y.obs_id);
  });

  std::vector<ChainRows> chains;
  for (const size_t row_index : order) {
    const auto& row = observations->at(row_index);
    bool split = chains.empty();
    if (!split) {
      const auto& previous = observations->at(
          chains.back().observation_rows.back());
      split = previous.tag_id != row.tag_id ||
              previous.anchor_id != row.anchor_id ||
              row.sensor_time - previous.sensor_time > options.gap_threshold_s;
    }
    if (split) chains.emplace_back();
    chains.back().observation_rows.push_back(row_index);
  }

  result->continuous_chains.clear();
  result->local_windows.clear();
  result->merged_segments.clear();
  result->covariance_window_count = 0;
  result->significant_window_count = 0;
  result->merged_segment_count = 0;
  const size_t base_count = std::max<size_t>(4, options.minimum_count);
  for (auto& chain : chains) {
    const auto& first = observations->at(chain.observation_rows.front());
    const auto& last = observations->at(chain.observation_rows.back());
    chain.record.tag_id = first.tag_id;
    chain.record.anchor_id = first.anchor_id;
    chain.record.start_time = first.sensor_time;
    chain.record.end_time = last.sensor_time;
    for (const auto row : chain.observation_rows)
      chain.record.obs_ids.push_back(observations->at(row).obs_id);
    chain.record.chain_id = ObjectId("chain", result->identity_hash,
                                    first.tag_id, first.anchor_id,
                                    chain.record.obs_ids);
    const auto bank = WindowBank(chain.observation_rows.size(), base_count);
    chain.record.multiplicity = bank.size();
    for (const auto row : chain.observation_rows)
      observations->at(row).raw_segment_id = chain.record.chain_id;

    const double adjusted_probability = bank.empty()
        ? std::numeric_limits<double>::quiet_NaN()
        : 1.0 - (1.0 - options.chi2_probability) /
                    static_cast<double>(bank.size());
    for (const auto& [start, size] : bank) {
      FdeLocalWindow window;
      window.chain_id = chain.record.chain_id;
      window.tag_id = first.tag_id;
      window.anchor_id = first.anchor_id;
      window.first_chain_index = start;
      window.last_chain_index = start + size - 1;
      window.start_time = observations->at(
          chain.observation_rows[window.first_chain_index]).sensor_time;
      window.end_time = observations->at(
          chain.observation_rows[window.last_chain_index]).sensor_time;
      window.multiplicity = bank.size();
      window.raw_probability = options.chi2_probability;
      window.family_alpha = 1.0 - options.chi2_probability;
      window.adjusted_probability = adjusted_probability;
      Eigen::VectorXd e(size);
      Eigen::VectorXd direction(size);
      window.covariance.resize(size, size);
      for (size_t j = 0; j < size; ++j) {
        const auto& row = observations->at(chain.observation_rows[start + j]);
        window.obs_ids.push_back(row.obs_id);
        e[j] = row.residual_m / row.factor_sigma_m;
        direction[j] = 1.0 / row.factor_sigma_m;
        for (size_t k = 0; k < size; ++k) {
          const auto& column = observations->at(
              chain.observation_rows[start + k]);
          window.covariance(j, k) = covariance(
              covariance_row.at(row.obs_id), covariance_row.at(column.obs_id));
        }
      }
      window.obs_ids_sha256 = HashObsIds(window.obs_ids);
      window.window_id = ObjectId("window", result->identity_hash,
                                  window.tag_id, window.anchor_id,
                                  window.obs_ids);
      window.test = FdeCovarianceTest(e, window.covariance);
      if (!window.test.valid)
        throw std::runtime_error("WINDOWED_FDE_UNAVAILABLE:" +
                                 window.test.reason);
      window.adjusted_threshold = FdeChiSquareQuantile(
          window.test.rank, window.adjusted_probability);
      window.adjusted_rejected =
          window.test.statistic > window.adjusted_threshold;
      window.gls_signed_residual_m = GlsSignedResidual(
          e, direction, window.covariance, window.test);
      window.positive_excess = window.gls_signed_residual_m < 0.0;
      window.count_eligible = size >= options.minimum_count;
      window.duration_eligible =
          window.end_time - window.start_time >= options.minimum_duration_s;
      window.significant = window.adjusted_rejected &&
                           window.positive_excess && window.count_eligible &&
                           window.duration_eligible;
      if (window.significant)
        window.status = "SIGNIFICANT_ADJUSTED_NEGATIVE_GLS";
      else if (!window.adjusted_rejected)
        window.status = "NOT_REJECTED_ADJUSTED";
      else if (!window.positive_excess)
        window.status = "REJECTED_NON_POSITIVE_EXCESS";
      else
        window.status = "FILTERED_ORIGINAL_COUNT_DURATION";
      result->significant_window_count += window.significant;
      result->local_windows.push_back(std::move(window));
    }
    result->covariance_window_count += bank.size();
    result->continuous_chains.push_back(chain.record);
  }

  // Merge significant windows only within their chain. A segment contains the
  // exact union of window observations; adjacency is defined in chain indices.
  for (const auto& chain : chains) {
    std::vector<size_t> significant;
    for (size_t j = 0; j < result->local_windows.size(); ++j) {
      const auto& window = result->local_windows[j];
      if (window.chain_id == chain.record.chain_id && window.significant)
        significant.push_back(j);
    }
    std::sort(significant.begin(), significant.end(), [&](size_t a, size_t b) {
      const auto& x = result->local_windows[a];
      const auto& y = result->local_windows[b];
      return std::tie(x.first_chain_index, x.last_chain_index, x.window_id) <
             std::tie(y.first_chain_index, y.last_chain_index, y.window_id);
    });
    struct MergeAccumulator {
      std::set<size_t> positions;
      std::vector<size_t> windows;
      size_t max_position = 0;
    } open;
    auto close = [&]() {
      if (open.windows.empty()) return;
      FdeMergedWindowSegment segment;
      segment.chain_id = chain.record.chain_id;
      segment.tag_id = chain.record.tag_id;
      segment.anchor_id = chain.record.anchor_id;
      for (const auto position : open.positions) {
        segment.obs_ids.push_back(observations->at(
            chain.observation_rows[position]).obs_id);
      }
      for (const auto window_index : open.windows)
        segment.window_ids.push_back(
            result->local_windows[window_index].window_id);
      segment.start_time = observations->at(chain.observation_rows[
          *open.positions.begin()]).sensor_time;
      segment.end_time = observations->at(chain.observation_rows[
          *open.positions.rbegin()]).sensor_time;
      segment.count_eligible =
          segment.obs_ids.size() >= options.minimum_count;
      segment.duration_eligible =
          segment.end_time - segment.start_time >= options.minimum_duration_s;
      segment.retained_before_isolation =
          segment.count_eligible && segment.duration_eligible;
      segment.status = segment.retained_before_isolation
          ? "RETAINED_BEFORE_LINK_ISOLATION"
          : "FILTERED_MERGED_COUNT_DURATION";
      segment.segment_id = ObjectId("segment", result->identity_hash,
                                    segment.tag_id, segment.anchor_id,
                                    segment.obs_ids);
      for (const auto window_index : open.windows)
        result->local_windows[window_index].merged_segment_id =
            segment.segment_id;
      result->merged_segments.push_back(std::move(segment));
      open = MergeAccumulator();
    };
    for (const auto window_index : significant) {
      const auto& window = result->local_windows[window_index];
      if (!open.windows.empty() &&
          window.first_chain_index > open.max_position + 1)
        close();
      open.windows.push_back(window_index);
      for (size_t position = window.first_chain_index;
           position <= window.last_chain_index; ++position)
        open.positions.insert(position);
      open.max_position = std::max(open.max_position, window.last_chain_index);
    }
    close();
  }
  result->merged_segment_count = result->merged_segments.size();

  std::set<std::pair<int, int>> qualified_links;
  for (const auto& segment : result->merged_segments) {
    if (segment.retained_before_isolation)
      qualified_links.insert({segment.tag_id, segment.anchor_id});
  }
  if (qualified_links.empty())
    result->windowed_status = "WINDOWED_CONSISTENT_NO_SUPPORT";
  else if (qualified_links.size() == 1)
    result->windowed_status = "WINDOWED_UNIQUE_POSITIVE_LINK";
  else
    result->windowed_status = "FDE_ISOLATION_AMBIGUOUS";

  SupportPartition partition;
  partition.schema = "uifgo_fde_support_v1";
  partition.provider = FdeProvider(false, true);
  partition.hash_algorithm = "SHA-256";
  partition.partition_rule_version = kWindowedPartitionRule;
  partition.input_plan_hash = context.input_plan_hash;
  partition.source_hash = context.source_hash;
  partition.config_hash = context.config_hash;
  partition.calibration_hash = context.calibration_hash;
  partition.solver_config_hash = context.solver_config_hash;
  partition.discovery_context_hash = result->identity_hash;
  partition.discovery_snapshot_hash = SnapshotHashV4(
      *observations, partition.discovery_context_hash);

  size_t filtered = 0;
  for (auto& merged : result->merged_segments) {
    if (!merged.retained_before_isolation) {
      ++filtered;
      continue;
    }
    if (qualified_links.size() != 1) {
      merged.status = "FDE_ISOLATION_AMBIGUOUS";
      for (auto& row : *observations) {
        if (std::find(merged.obs_ids.begin(), merged.obs_ids.end(),
                      row.obs_id) != merged.obs_ids.end())
          row.candidate_filter_reason = "FDE_ISOLATION_AMBIGUOUS";
      }
      continue;
    }
    SupportSegment segment;
    segment.segment_ordinal = partition.segments.size();
    segment.segment_id = merged.segment_id;
    segment.tag_id = merged.tag_id;
    segment.anchor_id = merged.anchor_id;
    segment.start_time = merged.start_time;
    segment.end_time = merged.end_time;
    segment.observation_count = merged.obs_ids.size();
    segment.duration = merged.end_time - merged.start_time;
    segment.merge_snapshot_mean_m = 0.0;
    segment.short_support_debug = false;
    segment.parent_segment_ids = merged.window_ids;
    segment.obs_ids = merged.obs_ids;
    for (auto& row : *observations) {
      if (std::find(merged.obs_ids.begin(), merged.obs_ids.end(), row.obs_id) ==
          merged.obs_ids.end())
        continue;
      row.nlos_candidate = true;
      row.segment_id = segment.segment_id;
      row.segment_ordinal = segment.segment_ordinal;
      row.candidate_filter_reason = "RETAINED_WINDOW_UNION";
    }
    merged.published = true;
    merged.status = "PUBLISHED_UNIQUE_LINK";
    partition.segments.push_back(std::move(segment));
  }
  partition.partition_hash = PartitionHashV4(partition);
  result->partition = std::move(partition);
  result->raw_run_count = chains.size();
  result->filtered_run_count = filtered;
  result->positive_candidate_count = 0;
  for (const auto& row : *observations)
    result->positive_candidate_count += row.nlos_candidate;
  result->retained_segment_count = result->partition.segments.size();
}

void ApplyFullGraphWindowedFde(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const std::vector<FactorMeta>& metadata, const FdeOptions& options,
    const FdeContext& context, FdeResult* result) {
  const auto residuals = ReadScalarUwbFactorResiduals(graph, values, metadata);
  const auto linear = graph.linearize(values);
  gtsam::Ordering ordering;
  for (const auto& item : values) ordering.push_back(item.key);
  size_t rows = 0;
  size_t columns = 0;
  const auto entries = linear->sparseJacobian(ordering, rows, columns);
  if (columns != values.dim() + 1)
    throw std::runtime_error("WINDOWED_FDE_DIMENSION_MISMATCH");
  Eigen::VectorXd e = Eigen::VectorXd::Zero(rows);
  std::vector<Eigen::Triplet<double>> triplets;
  for (const auto& entry : entries) {
    const int row = std::get<0>(entry);
    const int column = std::get<1>(entry);
    const double value = std::get<2>(entry);
    if (!std::isfinite(value) || row < 0 || column < 0 ||
        row >= static_cast<int>(rows) || column >= static_cast<int>(columns))
      throw std::runtime_error("WINDOWED_FDE_INVALID_LINEAR_ENTRY");
    if (column == static_cast<int>(columns - 1))
      e[row] = -value;
    else
      triplets.emplace_back(row, column, value);
  }
  Eigen::SparseMatrix<double> A(rows, columns - 1);
  A.setFromTriplets(triplets.begin(), triplets.end());
  A.makeCompressed();
  std::vector<size_t> offsets;
  size_t offset = 0;
  for (size_t j = 0; j < graph.size(); ++j) {
    offsets.push_back(offset);
    if (!linear->at(j) ||
        linear->at(j)->augmentedJacobian().rows() != graph.at(j)->dim())
      throw std::runtime_error("WINDOWED_FDE_FACTOR_ROW_MISMATCH");
    offset += graph.at(j)->dim();
  }
  if (offset != rows)
    throw std::runtime_error("WINDOWED_FDE_ROW_COVERAGE_MISMATCH");
  std::vector<size_t> selected;
  std::vector<std::uint64_t> ids;
  for (const auto& residual : residuals) {
    selected.push_back(offsets.at(residual.factor_index));
    ids.push_back(residual.obs_id);
  }
  const auto block = FdeFullProjectionBlock(A, e, selected);
  if (!block.certificate.valid)
    throw std::runtime_error("WINDOWED_FDE_PROJECTOR_UNAVAILABLE:" +
                             block.certificate.reason);
  ApplyWindowedFdeTests(&result->observations, block.Pww, ids, options,
                        context, result);
}

}  // namespace uifgo
