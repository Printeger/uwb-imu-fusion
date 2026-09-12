#include "uifgo/pl_persistent_cusum.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
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

struct LinkState {
  double g = 0.0;
  bool has_valid_time = false;
  double last_valid_time = 0.0;
  size_t excursion_counter = 0;
  bool in_excursion = false;
  bool alarm_latched = false;
  std::string excursion_id;
  double excursion_start_time = std::numeric_limits<double>::quiet_NaN();
  std::vector<size_t> pending_trace_indices;
  std::vector<size_t> detected_trace_indices;
};

void EndExcursion(LinkState* state) {
  state->in_excursion = false;
  state->alarm_latched = false;
  state->excursion_id.clear();
  state->excursion_start_time = std::numeric_limits<double>::quiet_NaN();
  state->pending_trace_indices.clear();
  state->detected_trace_indices.clear();
}

std::string PartitionHash(const SupportPartition& support) {
  std::ostringstream canonical;
  canonical << kPlPersistentCusumAlgorithmVersion << '\n'
            << kPlPersistentCusumPartitionRule << '\n'
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

}  // namespace

std::string PlPersistentCusumIdentity(const PlCusumOptions& options) {
  std::ostringstream canonical;
  canonical << kPlPersistentCusumAlgorithmVersion << '\n'
            << kPlPersistentCusumSignalVersion << '\n'
            << "state_key=tag_id,anchor_id\n"
            << "kappa_bits=" << Bits(options.kappa) << '\n'
            << "threshold_bits=" << Bits(options.threshold) << '\n'
            << "gap_bits=" << Bits(options.gap_threshold_s) << '\n'
            << "invalid=reset_no_alarm\n"
            << "comparison=G_after>=threshold\n"
            << "backfill=last_zero_excursion_start\n"
            << "termination=G_zero_or_gap_or_invalid\n"
            << "calibration=" << options.calibration_hash << '\n'
            << "split=" << options.split_manifest_hash << '\n';
  return "plcusum-sha256:" + Sha256Hex(canonical.str());
}

PlCusumResult EvaluatePlPersistentCusum(
    const std::vector<PlCusumInputRow>& input_rows,
    const PlCusumOptions& options) {
  PlCusumResult result;
  result.detector_identity = PlPersistentCusumIdentity(options);
  if (!(std::isfinite(options.kappa) && options.kappa == 0.5) ||
      !(std::isfinite(options.threshold) && options.threshold > 0.0) ||
      !(std::isfinite(options.gap_threshold_s) &&
        options.gap_threshold_s == 1.0)) {
    result.status = "INVALID_LOCKED_OPTIONS";
    return result;
  }

  // Fail closed before sorting. Sorting must never conceal a reverse-time
  // source sequence for one link.
  std::map<PlCusumLinkKey, double> supplied_last_time;
  for (const auto& row : input_rows) {
    const PlCusumLinkKey key{row.tag_id, row.anchor_id};
    if (!std::isfinite(row.timestamp)) {
      result.status = "NONFINITE_TIMESTAMP";
      return result;
    }
    const auto found = supplied_last_time.find(key);
    if (found != supplied_last_time.end() && row.timestamp < found->second) {
      result.status = "REVERSE_TIME_FAIL_CLOSED:" + LinkText(key);
      return result;
    }
    supplied_last_time[key] = row.timestamp;
  }

  std::vector<PlCusumInputRow> rows = input_rows;
  std::stable_sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
    return std::tie(a.timestamp, a.keyframe_id, a.source_order, a.obs_id) <
           std::tie(b.timestamp, b.keyframe_id, b.source_order, b.obs_id);
  });
  std::map<PlCusumLinkKey, double> sorted_last_time;
  for (const auto& row : rows) {
    const PlCusumLinkKey key{row.tag_id, row.anchor_id};
    const auto found = sorted_last_time.find(key);
    if (found != sorted_last_time.end() && row.timestamp < found->second) {
      result.status = "SORTED_REVERSE_TIME_FAIL_CLOSED:" + LinkText(key);
      return result;
    }
    sorted_last_time[key] = row.timestamp;
  }

  std::map<PlCusumLinkKey, LinkState> states;
  std::set<PlCusumLinkKey> alarm_links;
  for (const auto& row : rows) {
    const PlCusumLinkKey key{row.tag_id, row.anchor_id};
    auto& state = states[key];
    PlCusumTraceRow trace;
    trace.input = row;
    trace.g_before = state.g;
    const bool valid = row.diagnostic_valid &&
                       std::isfinite(row.conditional_z);
    if (!valid) {
      trace.reset = true;
      trace.reset_reason = "NUMERICAL_INVALID_RESET";
      trace.g_after = 0.0;
      state.g = 0.0;
      state.has_valid_time = false;
      EndExcursion(&state);
      ++result.invalid_row_count;
      ++result.reset_counts[trace.reset_reason];
      result.trace.push_back(std::move(trace));
      continue;
    }
    ++result.valid_row_count;
    if (state.has_valid_time &&
        row.timestamp - state.last_valid_time > options.gap_threshold_s) {
      trace.reset = true;
      trace.reset_reason = "TIME_GAP_RESET";
      state.g = 0.0;
      EndExcursion(&state);
      ++result.reset_counts[trace.reset_reason];
    }
    trace.g_before = state.g;
    trace.increment = row.conditional_z - options.kappa;
    state.g = std::max(0.0, state.g + trace.increment);
    trace.g_after = state.g;
    state.has_valid_time = true;
    state.last_valid_time = row.timestamp;

    const size_t trace_index = result.trace.size();
    if (state.g > 0.0) {
      if (!state.in_excursion) {
        state.in_excursion = true;
        ++state.excursion_counter;
        state.excursion_id = "plcusum-" + LinkText(key) + "-" +
                             std::to_string(state.excursion_counter);
        state.excursion_start_time = row.timestamp;
      }
      trace.excursion_id = state.excursion_id;
      trace.excursion_start_time = state.excursion_start_time;
      state.pending_trace_indices.push_back(trace_index);
      const bool crossing = !state.alarm_latched &&
                            state.g >= options.threshold;
      if (crossing) {
        state.alarm_latched = true;
        trace.threshold_crossing = true;
        trace.first_alarm_for_excursion = true;
        ++result.alarm_count;
        alarm_links.insert(key);
        for (const auto index : state.pending_trace_indices) {
          if (index < result.trace.size()) result.trace[index].candidate = true;
          state.detected_trace_indices.push_back(index);
        }
        trace.candidate = true;
      } else if (state.alarm_latched) {
        trace.candidate = true;
      }
    } else {
      EndExcursion(&state);
    }
    result.max_g = std::max(result.max_g, state.g);
    result.per_link_max_g[key] =
        std::max(result.per_link_max_g[key], state.g);
    result.trace.push_back(std::move(trace));
  }
  result.alarm_link_count = alarm_links.size();

  // Candidate rows belonging to one detected excursion form one segment.
  std::map<std::string, SupportSegment> by_excursion;
  for (const auto& trace : result.trace) {
    if (!trace.candidate) continue;
    auto& segment = by_excursion[trace.excursion_id];
    if (segment.obs_ids.empty()) {
      segment.segment_id = trace.excursion_id;
      segment.tag_id = trace.input.tag_id;
      segment.anchor_id = trace.input.anchor_id;
      segment.start_time = trace.input.timestamp;
      segment.parent_segment_ids = {trace.excursion_id};
    }
    segment.end_time = trace.input.timestamp;
    segment.obs_ids.push_back(trace.input.obs_id);
  }
  for (auto& item : by_excursion) {
    auto& segment = item.second;
    segment.segment_ordinal = result.support.segments.size();
    segment.observation_count = segment.obs_ids.size();
    segment.duration = segment.end_time - segment.start_time;
    result.support.segments.push_back(std::move(segment));
  }
  result.segment_count = result.support.segments.size();
  result.support.schema = "pl_persistent_cusum_support_v1";
  result.support.provider = kPlPersistentCusumProvider;
  result.support.hash_algorithm = "sha256";
  result.support.partition_rule_version = kPlPersistentCusumPartitionRule;
  result.support.discovery_context_hash = result.detector_identity;
  result.support.calibration_hash = options.calibration_hash;
  result.support.partition_hash = PartitionHash(result.support);
  result.valid = true;
  result.status = "OK";
  return result;
}

}  // namespace uifgo
