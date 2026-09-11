#include "uifgo/pl_conditional_raim.h"

#include <boost/math/distributions/chi_squared.hpp>

#include <Eigen/Cholesky>

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

std::string Bits(double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "binary64 required");
  std::memcpy(&bits, &value, sizeof(bits));
  std::ostringstream out;
  out << std::hex << bits;
  return out.str();
}

bool Symmetric(const Eigen::MatrixXd& matrix) {
  const double scale = std::max(1.0, matrix.norm());
  return (matrix - matrix.transpose()).norm() <= 1e-12 + 1e-10 * scale;
}

PlConditionalInput Subset(const PlConditionalInput& input,
                          const std::vector<size_t>& rows) {
  PlConditionalInput output;
  output.prior_covariance = input.prior_covariance;
  output.p_fa = input.p_fa;
  const Eigen::Index n = static_cast<Eigen::Index>(rows.size());
  output.physical_jacobian.resize(n, 15);
  output.physical_covariance.resize(n, n);
  output.physical_innovation.resize(n);
  for (Eigen::Index i = 0; i < n; ++i) {
    output.physical_jacobian.row(i) =
        input.physical_jacobian.row(static_cast<Eigen::Index>(rows[i]));
    output.physical_innovation[i] =
        input.physical_innovation[static_cast<Eigen::Index>(rows[i])];
    for (Eigen::Index j = 0; j < n; ++j) {
      output.physical_covariance(i, j) = input.physical_covariance(
          static_cast<Eigen::Index>(rows[i]),
          static_cast<Eigen::Index>(rows[j]));
    }
  }
  return output;
}

std::string HashIds(const std::vector<std::uint64_t>& ids,
                    const std::string& prefix) {
  std::ostringstream canonical;
  canonical << prefix << '\n';
  for (const auto id : ids) canonical << id << '\n';
  return "sha256:" + Sha256Hex(canonical.str());
}

std::string PartitionHash(const SupportPartition& partition) {
  std::ostringstream canonical;
  canonical << kPlConditionalIdentityVersion << '\n'
            << kPlConditionalPartitionRule << '\n'
            << partition.discovery_context_hash << '\n'
            << partition.discovery_snapshot_hash << '\n';
  for (const auto& segment : partition.segments) {
    canonical << segment.segment_ordinal << ',' << segment.segment_id << ','
              << segment.tag_id << ',' << segment.anchor_id << ','
              << Bits(segment.start_time) << ',' << Bits(segment.end_time)
              << ',' << segment.observation_count << ','
              << Bits(segment.duration) << '\n';
    for (const auto id : segment.obs_ids) canonical << id << ',';
    canonical << '\n';
  }
  return "sha256:" + Sha256Hex(canonical.str());
}

}  // namespace

PlConditionalDecision EvaluatePlConditional(const PlConditionalInput& input) {
  PlConditionalDecision output;
  output.p_fa = input.p_fa;
  output.dof = static_cast<int>(input.physical_innovation.size());
  const int n = output.dof;
  if (n <= 0 || input.physical_jacobian.rows() != n ||
      input.physical_jacobian.cols() != 15 ||
      input.physical_covariance.rows() != n ||
      input.physical_covariance.cols() != n ||
      !(input.p_fa > 0.0 && input.p_fa < 1.0) ||
      !input.prior_covariance.allFinite() ||
      !input.physical_jacobian.allFinite() ||
      !input.physical_covariance.allFinite() ||
      !input.physical_innovation.allFinite() ||
      !Symmetric(input.prior_covariance) ||
      !Symmetric(input.physical_covariance)) {
    output.status = "INVALID_DIMENSION_OR_NONFINITE_INPUT";
    return output;
  }
  Eigen::LLT<Eigen::Matrix<double, 15, 15>> prior_llt(
      input.prior_covariance);
  Eigen::LLT<Eigen::MatrixXd> covariance_llt(input.physical_covariance);
  if (prior_llt.info() != Eigen::Success ||
      covariance_llt.info() != Eigen::Success) {
    output.status = "PRIOR_OR_MEASUREMENT_COVARIANCE_NOT_SPD";
    return output;
  }
  output.whitener = covariance_llt.matrixL().solve(
      Eigen::MatrixXd::Identity(n, n));
  output.whitened_innovation = output.whitener * input.physical_innovation;
  output.whitened_jacobian = output.whitener * input.physical_jacobian;
  output.innovation_covariance_whitened =
      output.whitened_jacobian * input.prior_covariance *
          output.whitened_jacobian.transpose() +
      Eigen::MatrixXd::Identity(n, n);
  output.innovation_covariance_physical =
      input.physical_jacobian * input.prior_covariance *
          input.physical_jacobian.transpose() +
      input.physical_covariance;
  Eigen::LDLT<Eigen::MatrixXd> innovation_ldlt(
      output.innovation_covariance_whitened);
  if (!output.whitener.allFinite() ||
      !output.innovation_covariance_whitened.allFinite() ||
      !output.innovation_covariance_physical.allFinite() ||
      innovation_ldlt.info() != Eigen::Success ||
      !innovation_ldlt.isPositive()) {
    output.status = "INNOVATION_COVARIANCE_FACTORIZATION_FAILED";
    return output;
  }
  output.threshold = boost::math::quantile(
      boost::math::chi_squared(n), 1.0 - input.p_fa);
  output.statistic = output.whitened_innovation.dot(
      innovation_ldlt.solve(output.whitened_innovation));
  output.numerically_valid = std::isfinite(output.threshold) &&
                             std::isfinite(output.statistic);
  output.passed = output.numerically_valid &&
                  output.statistic <= output.threshold;
  output.status = output.numerically_valid
                      ? (output.passed ? "PASS" : "ALARM")
                      : "NONFINITE_DETECTOR_RESULT";
  return output;
}

const char* PlConditionalGroupOutcomeName(PlConditionalGroupOutcome outcome) {
  switch (outcome) {
    case PlConditionalGroupOutcome::BOOTSTRAP_HISTORY:
      return "BOOTSTRAP_HISTORY";
    case PlConditionalGroupOutcome::GROUP_PASS:
      return "GROUP_PASS";
    case PlConditionalGroupOutcome::UNIQUE_ISOLATION_POSITIVE:
      return "UNIQUE_ISOLATION_POSITIVE";
    case PlConditionalGroupOutcome::UNIQUE_ISOLATION_NONPOSITIVE:
      return "UNIQUE_ISOLATION_NONPOSITIVE";
    case PlConditionalGroupOutcome::FDE_UNISOLATED_FAULT:
      return "FDE_UNISOLATED_FAULT";
    case PlConditionalGroupOutcome::FDE_ISOLATION_AMBIGUOUS:
      return "FDE_ISOLATION_AMBIGUOUS";
    case PlConditionalGroupOutcome::NUMERICAL_FAILURE:
      return "NUMERICAL_FAILURE";
  }
  return "UNKNOWN";
}

PlConditionalGroupResult EvaluatePlConditionalLoao(
    const PlConditionalInput& input, const std::vector<int>& anchor_ids,
    const std::vector<std::uint64_t>& obs_ids) {
  PlConditionalGroupResult output;
  const size_t n = static_cast<size_t>(input.physical_innovation.size());
  if (n == 0 || anchor_ids.size() != n || obs_ids.size() != n ||
      std::set<int>(anchor_ids.begin(), anchor_ids.end()).size() != n ||
      std::set<std::uint64_t>(obs_ids.begin(), obs_ids.end()).size() != n) {
    output.omnibus.status = "V1_GROUP_ANCHOR_OR_OBS_ID_INVALID";
    output.prior_degradation_event = true;
    return output;
  }
  output.omnibus = EvaluatePlConditional(input);
  if (!output.omnibus.numerically_valid) {
    output.prior_degradation_event = true;
    return output;
  }
  if (output.omnibus.passed) {
    output.outcome = PlConditionalGroupOutcome::GROUP_PASS;
    output.committed_rows.resize(n);
    for (size_t i = 0; i < n; ++i) output.committed_rows[i] = i;
    return output;
  }
  size_t passing = 0;
  size_t passing_exclusion = 0;
  for (size_t excluded = 0; excluded < n; ++excluded) {
    std::vector<size_t> retained;
    for (size_t row = 0; row < n; ++row)
      if (row != excluded) retained.push_back(row);
    PlConditionalHypothesis hypothesis;
    hypothesis.excluded_anchor_id = anchor_ids[excluded];
    for (const auto row : retained)
      hypothesis.retained_obs_ids.push_back(obs_ids[row]);
    hypothesis.decision = EvaluatePlConditional(Subset(input, retained));
    if (!hypothesis.decision.numerically_valid) {
      output.hypotheses.push_back(std::move(hypothesis));
      output.prior_degradation_event = true;
      return output;
    }
    if (hypothesis.decision.passed) {
      ++passing;
      passing_exclusion = excluded;
    }
    output.hypotheses.push_back(std::move(hypothesis));
  }
  if (passing == 0) {
    output.outcome = PlConditionalGroupOutcome::FDE_UNISOLATED_FAULT;
    output.prior_degradation_event = true;
    return output;
  }
  if (passing > 1) {
    output.outcome = PlConditionalGroupOutcome::FDE_ISOLATION_AMBIGUOUS;
    output.prior_degradation_event = true;
    return output;
  }
  output.isolated_anchor_id = anchor_ids[passing_exclusion];
  output.isolated_obs_id = obs_ids[passing_exclusion];
  output.isolated_innovation_m =
      input.physical_innovation[static_cast<Eigen::Index>(passing_exclusion)];
  output.positive_excess_candidate = output.isolated_innovation_m > 0.0;
  output.outcome = output.positive_excess_candidate
                       ? PlConditionalGroupOutcome::UNIQUE_ISOLATION_POSITIVE
                       : PlConditionalGroupOutcome::UNIQUE_ISOLATION_NONPOSITIVE;
  for (size_t row = 0; row < n; ++row)
    if (row != passing_exclusion) output.committed_rows.push_back(row);
  return output;
}

SupportPartition BuildPlConditionalSupport(
    std::vector<PlConditionalCandidateRecord>* records,
    double gap_threshold_s, size_t minimum_count,
    double minimum_duration_s, const PlConditionalSupportContext& context) {
  if (!records || !std::isfinite(gap_threshold_s) || gap_threshold_s < 0.0 ||
      minimum_count == 0 || !std::isfinite(minimum_duration_s) ||
      minimum_duration_s < 0.0 || context.input_plan_hash.empty() ||
      context.detector_identity_hash.empty()) {
    throw std::invalid_argument("PL_CONDITIONAL_SUPPORT_INPUT_INVALID");
  }
  std::vector<size_t> order;
  std::set<std::uint64_t> ids;
  for (size_t i = 0; i < records->size(); ++i) {
    auto& row = records->at(i);
    if (!row.planned) continue;
    if (!ids.insert(row.obs_id).second || !std::isfinite(row.sensor_time))
      throw std::invalid_argument("PL_CONDITIONAL_SUPPORT_ROW_INVALID");
    row.segment_id.clear();
    row.segment_ordinal = std::numeric_limits<size_t>::max();
    order.push_back(i);
  }
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    const auto& x = records->at(a);
    const auto& y = records->at(b);
    return std::tie(x.sensor_time, x.keyframe_id, x.obs_id) <
           std::tie(y.sensor_time, y.keyframe_id, y.obs_id);
  });
  struct Run { int tag = 0; int anchor = 0; std::vector<size_t> rows; };
  std::map<std::pair<int, int>, Run> active;
  std::vector<Run> completed;
  auto flush = [&](const std::pair<int, int>& link) {
    auto found = active.find(link);
    if (found != active.end() && !found->second.rows.empty()) {
      completed.push_back(std::move(found->second));
      active.erase(found);
    }
  };
  for (const size_t index : order) {
    const auto& row = records->at(index);
    const auto link = std::make_pair(row.tag_id, row.anchor_id);
    if (!row.candidate) {
      flush(link);
      continue;
    }
    auto& run = active[link];
    run.tag = row.tag_id;
    run.anchor = row.anchor_id;
    if (!run.rows.empty()) {
      const auto& previous = records->at(run.rows.back());
      if (row.sensor_time - previous.sensor_time > gap_threshold_s) {
        completed.push_back(std::move(run));
        run = Run{};
        run.tag = row.tag_id;
        run.anchor = row.anchor_id;
      }
    }
    run.rows.push_back(index);
  }
  for (auto& item : active)
    if (!item.second.rows.empty()) completed.push_back(std::move(item.second));
  std::stable_sort(completed.begin(), completed.end(), [&](const Run& a,
                                                            const Run& b) {
    const auto& x = records->at(a.rows.front());
    const auto& y = records->at(b.rows.front());
    return std::tie(x.sensor_time, x.tag_id, x.anchor_id, x.obs_id) <
           std::tie(y.sensor_time, y.tag_id, y.anchor_id, y.obs_id);
  });

  SupportPartition partition;
  partition.schema = "uifgo_support_partition_v1";
  partition.provider = context.production ? kPlConditionalProductionProvider
                                          : kPlConditionalPreflightProvider;
  partition.hash_algorithm = "sha256";
  partition.partition_rule_version = kPlConditionalPartitionRule;
  partition.input_plan_hash = context.input_plan_hash;
  partition.source_hash = context.source_hash;
  partition.config_hash = context.config_hash;
  partition.calibration_hash = context.calibration_hash;
  partition.solver_config_hash = context.solver_config_hash;
  partition.discovery_context_hash = context.detector_identity_hash;
  std::ostringstream snapshot;
  snapshot << kPlConditionalIdentityVersion << "\nCANDIDATES\n";
  for (const auto index : order) {
    const auto& row = records->at(index);
    snapshot << row.obs_id << ',' << row.tag_id << ',' << row.anchor_id << ','
             << row.keyframe_id << ',' << Bits(row.sensor_time) << ','
             << row.candidate << ',' << row.reason << '\n';
  }
  partition.discovery_snapshot_hash =
      "sha256:" + Sha256Hex(snapshot.str());
  for (auto& run : completed) {
    const double start = records->at(run.rows.front()).sensor_time;
    const double end = records->at(run.rows.back()).sensor_time;
    const bool retain = run.rows.size() >= minimum_count &&
                        end - start >= minimum_duration_s;
    if (!retain) {
      for (const auto index : run.rows)
        records->at(index).reason = "TEMPORAL_SUPPORT_FILTERED";
      continue;
    }
    SupportSegment segment;
    segment.segment_ordinal = partition.segments.size();
    segment.tag_id = run.tag;
    segment.anchor_id = run.anchor;
    segment.start_time = start;
    segment.end_time = end;
    segment.duration = end - start;
    for (const auto index : run.rows)
      segment.obs_ids.push_back(records->at(index).obs_id);
    segment.observation_count = segment.obs_ids.size();
    segment.segment_id = "pl-conditional-segment-" + Sha256Hex(
        context.detector_identity_hash + "\n" + std::to_string(run.tag) +
        ":" + std::to_string(run.anchor) + "\n" +
        HashIds(segment.obs_ids, "PL_CONDITIONAL_IDS"));
    segment.parent_segment_ids = {segment.segment_id};
    for (const auto index : run.rows) {
      records->at(index).segment_id = segment.segment_id;
      records->at(index).segment_ordinal = segment.segment_ordinal;
      records->at(index).reason = "RETAINED_PERSISTENT_SUPPORT";
    }
    partition.segments.push_back(std::move(segment));
  }
  partition.partition_hash = PartitionHash(partition);
  return partition;
}

std::string PlConditionalDetectorIdentity(
    const PlConditionalSupportContext& context, size_t bootstrap_last_keyframe,
    size_t fixed_lag_epochs, double imu_max_gap_s,
    double relinearize_threshold, size_t relinearize_skip) {
  std::ostringstream canonical;
  canonical << kPlConditionalIdentityVersion << '\n'
            << kPlConditionalSourceCommit << '\n'
            << "p_fa=" << Bits(1e-5) << '\n'
            << "group=paper_keyframe_all_planned_source_order_v1\n"
            << "whitening=physical_cholesky_lower_inverse_v1\n"
            << "bootstrap_last=" << bootstrap_last_keyframe << '\n'
            << "fixed_lag_epochs=" << fixed_lag_epochs << '\n'
            << "imu_max_gap=" << Bits(imu_max_gap_s) << '\n'
            << "imu_gap_policy=PL_CONTROLLED_REINITIALIZE_V1\n"
            << "reinitialize_prior_sigmas="
               "0.1,0.1,0.1,0.5,0.5,0.5,0.5,0.5,0.5,"
               "0.1,0.1,0.1,0.01,0.01,0.01\n"
            << "relinearize_threshold=" << Bits(relinearize_threshold) << '\n'
            << "relinearize_skip=" << relinearize_skip << '\n'
            << "unique_loao=true\npositive_innovation_strict=true\n"
            << "healthy_subset_commit=true\n"
            << "temporal=gap_gt_1_count_ge_2_duration_ge_0.01\n"
            << context.input_plan_hash << '\n' << context.source_hash << '\n'
            << context.config_hash << '\n' << context.calibration_hash << '\n'
            << context.solver_config_hash << '\n';
  return "sha256:" + Sha256Hex(canonical.str());
}

}  // namespace uifgo
