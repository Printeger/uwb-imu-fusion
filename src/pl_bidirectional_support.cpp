#include "uifgo/pl_bidirectional_support.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <set>
#include <sstream>
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

std::string LinkText(const PlCusumLinkKey& key) {
  return std::to_string(key.tag_id) + ":" + std::to_string(key.anchor_id);
}

struct BackwardState {
  double b = 0.0;
  bool has_valid_time = false;
  double previous_valid_time = 0.0;
  size_t reverse_index = 0;
  size_t excursion_counter = 0;
  bool in_excursion = false;
  bool crossing_latched = false;
  std::string excursion_id;
  double excursion_start_time = std::numeric_limits<double>::quiet_NaN();
  std::vector<size_t> pending_trace_indices;
};

void EndExcursion(BackwardState* state) {
  state->in_excursion = false;
  state->crossing_latched = false;
  state->excursion_id.clear();
  state->excursion_start_time = std::numeric_limits<double>::quiet_NaN();
  state->pending_trace_indices.clear();
}

std::string PartitionHash(const SupportPartition& support,
                          const std::string& version) {
  std::ostringstream canonical;
  canonical << version << '\n' << support.partition_rule_version << '\n'
            << support.discovery_context_hash << '\n';
  for (const auto& segment : support.segments) {
    canonical << segment.segment_ordinal << ',' << segment.segment_id << ','
              << segment.tag_id << ',' << segment.anchor_id << ','
              << Bits(segment.start_time) << ',' << Bits(segment.end_time)
              << ',' << segment.observation_count << '\n';
    for (const auto id : segment.obs_ids) canonical << id << ',';
    canonical << '\n';
  }
  return "sha256:" + Sha256Hex(canonical.str());
}

using MembershipKey = std::tuple<int, int, std::uint64_t>;

MembershipKey Key(const PlBidirectionalMembershipRow& row) {
  return {row.input.tag_id, row.input.anchor_id, row.input.obs_id};
}

}  // namespace

std::string PlBackwardCusumIdentity(
    const PlBackwardCusumOptions& options) {
  std::ostringstream canonical;
  canonical << kPlBackwardCusumAlgorithmVersion << '\n'
            << kPlPersistentCusumSignalVersion << '\n'
            << "direction=descending_physical_time\n"
            << "state_key=tag_id,anchor_id\n"
            << "kappa_bits=" << Bits(options.kappa) << '\n'
            << "threshold_bits=" << Bits(options.threshold) << '\n'
            << "gap_bits=" << Bits(options.gap_threshold_s) << '\n'
            << "gap=abs_delta_t_strictly_greater\n"
            << "invalid=backward_reset_no_alarm\n"
            << "comparison=B_after>=threshold\n"
            << "backfill=reverse_last_zero_excursion_start\n"
            << "termination=B_zero_or_gap_or_invalid\n"
            << "split=" << options.original_clean_split_manifest_hash << '\n'
            << "clean_input=" << options.original_clean_input_hash << '\n'
            << "calibration=" << options.calibration_hash << '\n';
  return "plbackward-sha256:" + Sha256Hex(canonical.str());
}

PlBackwardCusumResult EvaluatePlBackwardCusum(
    const std::vector<PlCusumInputRow>& input_rows,
    const PlBackwardCusumOptions& options) {
  PlBackwardCusumResult result;
  result.closure_identity = PlBackwardCusumIdentity(options);
  if (!(std::isfinite(options.kappa) && options.kappa == 0.5) ||
      !(std::isfinite(options.threshold) && options.threshold > 0.0) ||
      !(std::isfinite(options.gap_threshold_s) &&
        options.gap_threshold_s == 1.0)) {
    result.status = "INVALID_LOCKED_BACKWARD_OPTIONS";
    return result;
  }
  std::set<std::tuple<int, int, std::uint64_t>> identities;
  for (const auto& row : input_rows) {
    if (!std::isfinite(row.timestamp)) {
      result.status = "NONFINITE_TIMESTAMP";
      return result;
    }
    if (!identities.emplace(row.tag_id, row.anchor_id, row.obs_id).second) {
      result.status = "DUPLICATE_LINK_OBS_ID";
      return result;
    }
  }

  std::vector<PlCusumInputRow> rows = input_rows;
  std::stable_sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
    return std::tie(a.timestamp, a.keyframe_id, a.source_order, a.obs_id) >
           std::tie(b.timestamp, b.keyframe_id, b.source_order, b.obs_id);
  });
  std::map<PlCusumLinkKey, BackwardState> states;
  std::set<PlCusumLinkKey> alarm_links;
  for (const auto& row : rows) {
    const PlCusumLinkKey key{row.tag_id, row.anchor_id};
    auto& state = states[key];
    PlBackwardCusumTraceRow trace;
    trace.input = row;
    trace.reverse_index = ++state.reverse_index;
    trace.b_before = state.b;
    const bool valid = row.diagnostic_valid &&
                       std::isfinite(row.conditional_z);
    if (!valid) {
      trace.reset = true;
      trace.reset_reason = "BACKWARD_NUMERICAL_INVALID_RESET";
      state.b = 0.0;
      state.has_valid_time = false;
      EndExcursion(&state);
      ++result.invalid_row_count;
      ++result.reset_counts[trace.reset_reason];
      result.trace.push_back(std::move(trace));
      continue;
    }
    ++result.valid_row_count;
    if (state.has_valid_time &&
        std::abs(row.timestamp - state.previous_valid_time) >
            options.gap_threshold_s) {
      trace.reset = true;
      trace.reset_reason = "BACKWARD_TIME_GAP_RESET";
      state.b = 0.0;
      EndExcursion(&state);
      ++result.reset_counts[trace.reset_reason];
    }
    trace.b_before = state.b;
    trace.increment = row.conditional_z - options.kappa;
    state.b = std::max(0.0, state.b + trace.increment);
    trace.b_after = state.b;
    state.has_valid_time = true;
    state.previous_valid_time = row.timestamp;
    const size_t trace_index = result.trace.size();
    if (state.b > 0.0) {
      if (!state.in_excursion) {
        state.in_excursion = true;
        ++state.excursion_counter;
        state.excursion_id = "plbackward-" + LinkText(key) + "-" +
                             std::to_string(state.excursion_counter);
        state.excursion_start_time = row.timestamp;
      }
      trace.reverse_excursion_id = state.excursion_id;
      trace.reverse_excursion_start_time = state.excursion_start_time;
      state.pending_trace_indices.push_back(trace_index);
      const bool crossing = !state.crossing_latched &&
                            state.b >= options.threshold;
      if (crossing) {
        state.crossing_latched = true;
        trace.threshold_crossing = true;
        trace.first_crossing_for_excursion = true;
        ++result.alarm_count;
        alarm_links.insert(key);
        for (const auto index : state.pending_trace_indices)
          if (index < result.trace.size())
            result.trace[index].backward_candidate = true;
        trace.backward_candidate = true;
      } else if (state.crossing_latched) {
        trace.backward_candidate = true;
      }
    } else {
      EndExcursion(&state);
    }
    result.max_b = std::max(result.max_b, state.b);
    result.per_link_max_b[key] =
        std::max(result.per_link_max_b[key], state.b);
    result.trace.push_back(std::move(trace));
  }
  result.alarm_link_count = alarm_links.size();

  std::map<std::string, SupportSegment> segments;
  for (const auto& trace : result.trace) {
    if (!trace.backward_candidate) continue;
    auto& segment = segments[trace.reverse_excursion_id];
    if (segment.obs_ids.empty()) {
      segment.segment_id = trace.reverse_excursion_id;
      segment.tag_id = trace.input.tag_id;
      segment.anchor_id = trace.input.anchor_id;
      segment.start_time = trace.input.timestamp;
      segment.end_time = trace.input.timestamp;
      segment.parent_segment_ids = {trace.reverse_excursion_id};
    }
    segment.start_time = std::min(segment.start_time, trace.input.timestamp);
    segment.end_time = std::max(segment.end_time, trace.input.timestamp);
    segment.obs_ids.push_back(trace.input.obs_id);
  }
  for (auto& item : segments) {
    auto& segment = item.second;
    std::reverse(segment.obs_ids.begin(), segment.obs_ids.end());
    segment.segment_ordinal = result.support.segments.size();
    segment.observation_count = segment.obs_ids.size();
    segment.duration = segment.end_time - segment.start_time;
    result.support.segments.push_back(std::move(segment));
  }
  result.segment_count = result.support.segments.size();
  result.support.schema = "pl_backward_cusum_support_v1";
  result.support.provider = "pl_backward_cusum_closure_preflight_v1";
  result.support.hash_algorithm = "sha256";
  result.support.partition_rule_version = "PL_BACKWARD_LAST_ZERO_CLOSURE_V1";
  result.support.discovery_context_hash = result.closure_identity;
  result.support.calibration_hash = options.calibration_hash;
  result.support.partition_hash = PartitionHash(
      result.support, kPlBackwardCusumAlgorithmVersion);
  result.valid = true;
  result.status = "OK";
  return result;
}

PlBidirectionalSupportResult IntersectPlCusumSupport(
    const std::vector<PlBidirectionalMembershipRow>& forward_rows,
    const std::vector<PlBidirectionalMembershipRow>& backward_rows,
    const std::string& forward_detector_identity,
    const std::string& backward_closure_identity,
    double gap_threshold_s) {
  PlBidirectionalSupportResult result;
  result.forward_detector_identity = forward_detector_identity;
  result.backward_closure_identity = backward_closure_identity;
  if (forward_detector_identity.empty() || backward_closure_identity.empty() ||
      !(std::isfinite(gap_threshold_s) && gap_threshold_s == 1.0)) {
    result.status = "INVALID_INTERSECTION_CONTEXT";
    return result;
  }
  std::map<MembershipKey, PlBidirectionalMembershipRow> forward;
  std::map<MembershipKey, PlBidirectionalMembershipRow> backward;
  for (const auto& row : forward_rows)
    if (!forward.emplace(Key(row), row).second) {
      result.status = "DUPLICATE_FORWARD_LINK_OBS_ID";
      return result;
    }
  for (const auto& row : backward_rows)
    if (!backward.emplace(Key(row), row).second) {
      result.status = "DUPLICATE_BACKWARD_LINK_OBS_ID";
      return result;
    }
  if (forward.size() != backward.size()) {
    result.status = "FORWARD_BACKWARD_IDENTITY_UNIVERSE_MISMATCH";
    return result;
  }
  for (const auto& item : forward) {
    const auto found = backward.find(item.first);
    if (found == backward.end()) {
      result.status = "FORWARD_BACKWARD_IDENTITY_UNIVERSE_MISMATCH";
      return result;
    }
    auto row = item.second;
    if (row.input.timestamp != found->second.input.timestamp ||
        row.input.keyframe_id != found->second.input.keyframe_id ||
        row.input.source_order != found->second.input.source_order) {
      result.status = "FORWARD_BACKWARD_ROW_IDENTITY_MISMATCH";
      return result;
    }
    row.backward_candidate = found->second.backward_candidate;
    row.final_candidate = row.forward_candidate && row.backward_candidate;
    result.rows.push_back(std::move(row));
  }
  std::stable_sort(result.rows.begin(), result.rows.end(),
                   [](const auto& a, const auto& b) {
    return std::tie(a.input.timestamp, a.input.keyframe_id,
                    a.input.source_order, a.input.obs_id) <
           std::tie(b.input.timestamp, b.input.keyframe_id,
                    b.input.source_order, b.input.obs_id);
  });

  struct Run {
    bool active = false;
    double previous_time = 0.0;
    SupportSegment segment;
  };
  std::map<PlCusumLinkKey, Run> runs;
  auto flush = [&](Run* run) {
    if (!run->active) return;
    run->segment.segment_ordinal = result.support.segments.size();
    run->segment.observation_count = run->segment.obs_ids.size();
    run->segment.duration = run->segment.end_time - run->segment.start_time;
    result.support.segments.push_back(run->segment);
    run->active = false;
    run->segment = SupportSegment{};
  };
  for (auto& row : result.rows) {
    const PlCusumLinkKey key{row.input.tag_id, row.input.anchor_id};
    auto& run = runs[key];
    if (!row.final_candidate ||
        (run.active && row.input.timestamp - run.previous_time >
                           gap_threshold_s)) {
      flush(&run);
    }
    if (!row.final_candidate) continue;
    if (!run.active) {
      run.active = true;
      run.segment.tag_id = key.tag_id;
      run.segment.anchor_id = key.anchor_id;
      run.segment.start_time = row.input.timestamp;
      run.segment.segment_id = "plbidirectional-" + LinkText(key) + "-" +
                               std::to_string(result.support.segments.size() + 1);
      run.segment.parent_segment_ids = {"forward", "backward"};
    }
    run.segment.end_time = row.input.timestamp;
    run.segment.obs_ids.push_back(row.input.obs_id);
    run.previous_time = row.input.timestamp;
  }
  for (auto& item : runs) flush(&item.second);

  for (auto& row : result.rows) {
    if (!row.final_candidate) continue;
    for (const auto& segment : result.support.segments) {
      if (segment.tag_id == row.input.tag_id &&
          segment.anchor_id == row.input.anchor_id &&
          std::find(segment.obs_ids.begin(), segment.obs_ids.end(),
                    row.input.obs_id) != segment.obs_ids.end()) {
        row.segment_id = segment.segment_id;
        row.segment_ordinal = segment.segment_ordinal;
        break;
      }
    }
  }
  std::ostringstream identity;
  identity << kPlBidirectionalSupportVersion << '\n'
           << kPlBidirectionalIntersectionRule << '\n'
           << "forward=" << forward_detector_identity << '\n'
           << "backward=" << backward_closure_identity << '\n'
           << "gap_bits=" << Bits(gap_threshold_s) << '\n';
  result.support_identity = "plbidirectional-sha256:" +
                            Sha256Hex(identity.str());
  result.support.schema = "pl_bidirectional_cusum_support_v1";
  result.support.provider = "pl_bidirectional_cusum_support_preflight_v1";
  result.support.hash_algorithm = "sha256";
  result.support.partition_rule_version = kPlBidirectionalIntersectionRule;
  result.support.discovery_context_hash = result.support_identity;
  result.support.partition_hash = PartitionHash(
      result.support, kPlBidirectionalSupportVersion);
  result.valid = true;
  result.status = "OK";
  return result;
}

}  // namespace uifgo
