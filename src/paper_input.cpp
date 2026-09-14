#include "uifgo/paper_input.h"
#include "uifgo/hash_utils.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <cstring>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace uifgo {
namespace {

std::uint64_t Fnv1a64(const std::string& text) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (unsigned char c : text) {
    hash ^= static_cast<std::uint64_t>(c);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string Hex(std::uint64_t value) {
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << value;
  return out.str();
}

std::string ObservationIdentity(const std::string& source_namespace,
                                size_t frame_index, size_t range_index,
                                const UwbFrame& frame, const UwbRange& range) {
  std::ostringstream out;
  out << std::setprecision(17) << source_namespace << '|';
  if (range.source_message_index !=
          std::numeric_limits<std::uint64_t>::max() &&
      range.source_range_index !=
          std::numeric_limits<std::uint64_t>::max()) {
    out << "source_message=" << range.source_message_index
        << "|source_range=" << range.source_range_index;
  } else if (range.source_obs_index !=
      std::numeric_limits<std::uint64_t>::max()) {
    out << "source_obs=" << range.source_obs_index;
  } else {
    out << "input_pos=" << frame_index << ':' << range_index << '|'
        << frame.t << '|' << frame.tag_id << '|' << range.anchor_id;
  }
  return out.str();
}

std::uint64_t DoubleBits(double value) {
  static_assert(sizeof(value) == sizeof(std::uint64_t),
                "binary64 payload comparison requires 64-bit double");
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

using PayloadSignature =
    std::tuple<int, int, std::uint64_t, std::uint64_t, std::uint64_t, bool,
               std::string>;

PayloadSignature ExactPayloadSignature(const ObservationRecord& row) {
  return std::make_tuple(row.tag_id, row.anchor_id, DoubleBits(row.raw_range),
                         DoubleBits(row.fp_rssi), DoubleBits(row.rx_rssi),
                         row.source_valid, row.source_validity_reason);
}

}  // namespace

const char* UwbIntegrityStatusName(UwbIntegrityStatus status) {
  switch (status) {
    case UwbIntegrityStatus::SOURCE_INVALID: return "SOURCE_INVALID";
    case UwbIntegrityStatus::ESTIMATOR_UNUSABLE:
      return "ESTIMATOR_UNUSABLE";
    case UwbIntegrityStatus::USABLE: return "USABLE";
    case UwbIntegrityStatus::STALE_REPEAT: return "STALE_REPEAT";
  }
  return "UNKNOWN";
}

std::uint64_t StableObservationId(const std::string& source_namespace,
                                  size_t frame_index, size_t range_index,
                                  const UwbFrame& frame,
                                  const UwbRange& range) {
  return Fnv1a64(ObservationIdentity(source_namespace, frame_index,
                                    range_index, frame, range));
}

PaperInputPlan BuildPaperInputPlan(const std::vector<UwbFrame>& raw_frames,
                                   const Config& cfg,
                                   const std::string& source_namespace) {
  if (source_namespace.empty()) {
    throw std::invalid_argument("paper input source namespace must not be empty");
  }
  if (cfg.kf_step < 1) {
    throw std::invalid_argument("paper input requires keyframe.step >= 1");
  }
  if (!(cfg.sigma_range > 0.0) || !std::isfinite(cfg.sigma_range)) {
    throw std::invalid_argument("paper input requires finite sigma_range > 0");
  }

  std::set<int> known_anchors;
  for (const auto& anchor : cfg.anchors) known_anchors.insert(anchor.id);

  PaperInputPlan plan;
  plan.source_namespace = source_namespace;
  plan.state_step = static_cast<size_t>(cfg.kf_step);
  plan.state_min_interval = cfg.kf_min_interval;
  plan.association_policy =
      "EXACT_SELECTED_SOURCE_FRAME_ONE_PER_CORRELATION_GROUP_V3";
  plan.sensor_noise_model = "FIXED_CONFIGURED_SIGMA_RANGE_V2";
  plan.integrity_policy =
      "EXACT_BINARY64_FULL_PAYLOAD_ADJACENT_MESSAGE_SAME_SLOT_V1";
  std::vector<std::vector<size_t>> record_indices(raw_frames.size());
  std::vector<bool> frame_has_usable(raw_frames.size(), false);
  std::set<int> tags;
  std::set<std::uint64_t> ids;

  // Ledger creation is deliberately first and preserves input positions.
  for (size_t fi = 0; fi < raw_frames.size(); ++fi) {
    const UwbFrame& frame = raw_frames[fi];
    for (size_t ri = 0; ri < frame.ranges.size(); ++ri) {
      const UwbRange& range = frame.ranges[ri];
      const double source_time =
          std::isfinite(range.source_time) ? range.source_time : frame.t;
      const int source_tag =
          range.source_tag_id == std::numeric_limits<int>::min()
              ? frame.tag_id
              : range.source_tag_id;
      tags.insert(source_tag);
      ObservationRecord record;
      record.obs_id = StableObservationId(source_namespace, fi, ri, frame,
                                          range);
      record.ledger_index = plan.observations.size();
      if (range.obs_id != 0 && range.obs_id != record.obs_id) {
        throw std::runtime_error(
            "cached obs_id does not match base recording/source ordinals");
      }
      if (!ids.insert(record.obs_id).second) {
        throw std::runtime_error("paper input obs_id collision");
      }
      record.source_frame_index = fi;
      record.source_message_index =
          range.source_message_index ==
                  std::numeric_limits<std::uint64_t>::max()
              ? fi
              : range.source_message_index;
      record.source_range_index =
          range.source_range_index ==
                  std::numeric_limits<std::uint64_t>::max()
              ? ri
              : range.source_range_index;
      record.source_observation_index =
          range.source_obs_index == std::numeric_limits<std::uint64_t>::max()
              ? record.obs_id
              : range.source_obs_index;
      record.sensor_time = source_time;
      record.tag_id = source_tag;
      record.anchor_id = range.anchor_id;
      record.raw_range = range.dist;
      record.fp_rssi = range.fp_rssi;
      record.rx_rssi = range.rx_rssi;

      if (!range.source_valid) {
        record.source_validity_reason = range.source_validity_reason.empty()
                                            ? "SOURCE_PROTOCOL_INVALID"
                                            : range.source_validity_reason;
      } else if (!std::isfinite(source_time) || !std::isfinite(range.dist) ||
          !std::isfinite(range.fp_rssi) || !std::isfinite(range.rx_rssi)) {
        record.source_validity_reason = "SOURCE_NONFINITE";
      } else {
        record.source_valid = true;
        record.source_validity_reason = "SOURCE_VALID";
        record.suspected_nlos =
            range.rx_rssi - range.fp_rssi > cfg.nlos_rssi_diff;
      }

      record_indices[fi].push_back(plan.observations.size());
      plan.observations.push_back(record);
      MeasurementPlanEntry measurement;
      measurement.obs_id = record.obs_id;
      measurement.observation_index = record.ledger_index;
      measurement.sensor_sigma = cfg.sigma_range;
      measurement.correlation_group_id = record.obs_id;
      measurement.correlation_representative_obs_id = record.obs_id;
      if (!record.source_valid) {
        measurement.integrity_status = UwbIntegrityStatus::SOURCE_INVALID;
        measurement.usability_reason =
            "SOURCE_INVALID:" + record.source_validity_reason;
      } else if (!known_anchors.count(record.anchor_id)) {
        measurement.integrity_status =
            UwbIntegrityStatus::ESTIMATOR_UNUSABLE;
        measurement.usability_reason = "UNKNOWN_ANCHOR";
      } else if (record.raw_range < cfg.min_range ||
                 record.raw_range > cfg.max_range) {
        measurement.integrity_status =
            UwbIntegrityStatus::ESTIMATOR_UNUSABLE;
        measurement.usability_reason = "INVALID_RANGE";
      } else {
        measurement.estimator_usable = true;
        measurement.integrity_status = UwbIntegrityStatus::USABLE;
        measurement.usability_reason = "ESTIMATOR_USABLE";
        frame_has_usable[fi] = true;
      }
      measurement.selection_reason = measurement.estimator_usable
                                         ? "NOT_SELECTED_STATE_SUBSAMPLING"
                                         : "NOT_SELECTED_NOT_ESTIMATOR_USABLE";
      plan.measurements.push_back(std::move(measurement));
    }
  }

  if (tags.size() > 1) {
    throw std::invalid_argument(
        "paper input T02 supports exactly one tag per run");
  }

  // Correlate only estimator-usable rows. Exact means bit-identical range and
  // RSSI payload plus identical validity payload on the same tag-anchor link.
  // Repetition is scoped to one source message or to the same range slot in
  // immediately adjacent source-message ordinals, so ordinary slowly changing
  // or later recurring measurements are never inferred to be duplicates.
  std::vector<size_t> source_order(plan.observations.size());
  for (size_t i = 0; i < source_order.size(); ++i) source_order[i] = i;
  std::stable_sort(source_order.begin(), source_order.end(),
                   [&](size_t lhs, size_t rhs) {
    const auto& a = plan.observations[lhs];
    const auto& b = plan.observations[rhs];
    return std::tie(a.source_message_index, a.source_range_index,
                    a.source_observation_index, a.ledger_index) <
           std::tie(b.source_message_index, b.source_range_index,
                    b.source_observation_index, b.ledger_index);
  });
  using SameMessageKey =
      std::tuple<std::uint64_t, PayloadSignature>;
  using SlotKey = std::tuple<int, int, std::uint64_t>;
  std::map<SameMessageKey, size_t> first_same_message_payload;
  std::map<SlotKey, size_t> previous_same_slot;
  for (size_t index : source_order) {
    auto& measurement = plan.measurements[index];
    const auto& row = plan.observations[index];
    if (!measurement.estimator_usable) continue;
    const PayloadSignature signature = ExactPayloadSignature(row);
    const SameMessageKey message_key(row.source_message_index, signature);
    const SlotKey slot_key(row.tag_id, row.anchor_id,
                           row.source_range_index);
    size_t repeated_from = std::numeric_limits<size_t>::max();
    const auto same_message = first_same_message_payload.find(message_key);
    if (same_message != first_same_message_payload.end()) {
      repeated_from = same_message->second;
    } else {
      const auto previous = previous_same_slot.find(slot_key);
      if (previous != previous_same_slot.end()) {
        const auto& prior = plan.observations[previous->second];
        if (prior.source_message_index !=
                std::numeric_limits<std::uint64_t>::max() &&
            prior.source_message_index + 1 == row.source_message_index &&
            ExactPayloadSignature(prior) == signature) {
          repeated_from = previous->second;
        }
      }
      first_same_message_payload.emplace(message_key, index);
    }
    if (repeated_from != std::numeric_limits<size_t>::max()) {
      const auto& prior_measurement = plan.measurements[repeated_from];
      measurement.integrity_status = UwbIntegrityStatus::STALE_REPEAT;
      measurement.correlation_group_id =
          prior_measurement.correlation_group_id;
      measurement.correlation_representative_obs_id =
          prior_measurement.correlation_representative_obs_id;
    }
    previous_same_slot[slot_key] = index;
  }
  std::map<std::uint64_t, size_t> correlation_group_sizes;
  for (const auto& measurement : plan.measurements)
    ++correlation_group_sizes[measurement.correlation_group_id];
  for (auto& measurement : plan.measurements)
    measurement.correlation_group_size =
        correlation_group_sizes.at(measurement.correlation_group_id);

  std::vector<size_t> eligible_frames;
  for (size_t fi = 0; fi < raw_frames.size(); ++fi) {
    if (frame_has_usable[fi]) eligible_frames.push_back(fi);
  }
  std::stable_sort(eligible_frames.begin(), eligible_frames.end(),
                   [&](size_t lhs, size_t rhs) {
                     if (raw_frames[lhs].t != raw_frames[rhs].t)
                       return raw_frames[lhs].t < raw_frames[rhs].t;
                     return lhs < rhs;
                   });

  double last_keyframe_time = -std::numeric_limits<double>::infinity();
  std::set<std::uint64_t> instantiated_correlation_groups;
  for (size_t order = 0; order < eligible_frames.size(); ++order) {
    if (order % static_cast<size_t>(cfg.kf_step) != 0) continue;
    const size_t fi = eligible_frames[order];
    const UwbFrame& frame = raw_frames[fi];
    if (cfg.kf_min_interval > 0.0 && !plan.keyframes.empty() &&
        frame.t - last_keyframe_time < cfg.kf_min_interval) {
      continue;
    }

    const size_t kf = plan.keyframes.size();
    plan.keyframes.push_back({kf, fi, frame.t, frame.tag_id});
    last_keyframe_time = frame.t;
    for (size_t record_index : record_indices[fi]) {
      MeasurementPlanEntry& measurement = plan.measurements[record_index];
      if (!measurement.estimator_usable) continue;
      if (!instantiated_correlation_groups
               .insert(measurement.correlation_group_id).second) {
        measurement.selection_reason = "NOT_SELECTED_CORRELATED_REPEAT";
        continue;
      }
      measurement.selected = true;
      measurement.independent_likelihood_representative = true;
      measurement.selection_reason = "SELECTED_EXACT_STATE_FRAME";
      measurement.keyframe_id = kf;
    }
  }
  for (auto& measurement : plan.measurements) {
    if (!measurement.estimator_usable || measurement.selected) continue;
    if (instantiated_correlation_groups.count(
            measurement.correlation_group_id))
      measurement.selection_reason = "NOT_SELECTED_CORRELATED_REPEAT";
  }

  std::ostringstream ledger_canonical;
  ledger_canonical << std::setprecision(17) << "observation_ledger_v2|"
                   << source_namespace;
  for (const auto& record : plan.observations) {
    ledger_canonical << '|' << record.ledger_index << ':' << record.obs_id
                     << ':' << record.source_frame_index << ':'
                     << record.source_message_index << ':'
                     << record.source_range_index << ':'
                     << record.source_observation_index << ':'
                     << record.sensor_time << ':' << record.tag_id << ':'
                     << record.anchor_id << ':' << record.raw_range << ':'
                     << record.fp_rssi << ':' << record.rx_rssi << ':'
                     << record.source_valid << ':'
                     << record.source_validity_reason << ':'
                     << record.suspected_nlos;
  }
  plan.observation_ledger_hash = Hex(Fnv1a64(ledger_canonical.str()));
  plan.observation_ledger_sha256 =
      "sha256:" + Sha256Hex(ledger_canonical.str());

  std::ostringstream state_canonical;
  state_canonical << std::setprecision(17) << "state_timeline_v2|state_step="
                  << plan.state_step << "|state_min_interval="
                  << plan.state_min_interval;
  for (const auto& keyframe : plan.keyframes) {
    state_canonical << '|' << keyframe.keyframe_id << ':'
                    << keyframe.source_frame_index << ':'
                    << keyframe.sensor_time << ':' << keyframe.tag_id;
  }
  plan.state_timeline_hash = Hex(Fnv1a64(state_canonical.str()));
  plan.state_timeline_sha256 =
      "sha256:" + Sha256Hex(state_canonical.str());

  std::ostringstream integrity_canonical;
  integrity_canonical << "integrity_plan_v1|" << plan.integrity_policy;
  for (const auto& measurement : plan.measurements) {
    integrity_canonical << '|' << measurement.observation_index << ':'
                        << measurement.obs_id << ':'
                        << UwbIntegrityStatusName(
                               measurement.integrity_status)
                        << ':' << measurement.correlation_group_id << ':'
                        << measurement.correlation_representative_obs_id << ':'
                        << measurement.correlation_group_size;
  }
  plan.integrity_plan_hash = Hex(Fnv1a64(integrity_canonical.str()));
  plan.integrity_plan_sha256 =
      "sha256:" + Sha256Hex(integrity_canonical.str());

  std::ostringstream measurement_canonical;
  measurement_canonical << std::setprecision(17) << "measurement_plan_v3|"
                        << plan.association_policy << '|'
                        << plan.sensor_noise_model << '|'
                        << plan.integrity_plan_sha256;
  for (const auto& measurement : plan.measurements) {
    measurement_canonical << '|' << measurement.observation_index << ':'
                          << measurement.obs_id << ':'
                          << measurement.estimator_usable << ':'
                          << measurement.usability_reason << ':'
                          << UwbIntegrityStatusName(
                                 measurement.integrity_status) << ':'
                          << measurement.correlation_group_id << ':'
                          << measurement.correlation_representative_obs_id
                          << ':' << measurement.correlation_group_size << ':'
                          << measurement.independent_likelihood_representative
                          << ':'
                          << measurement.selected << ':'
                          << measurement.selection_reason << ':'
                          << measurement.sensor_sigma;
    if (measurement.selected)
      measurement_canonical << ':' << measurement.keyframe_id;
  }
  plan.measurement_plan_hash = Hex(Fnv1a64(measurement_canonical.str()));
  plan.measurement_plan_sha256 =
      "sha256:" + Sha256Hex(measurement_canonical.str());

  std::ostringstream canonical;
  canonical << "paper_input_v4|" << plan.observation_ledger_sha256 << '|'
            << plan.state_timeline_sha256 << '|'
            << plan.integrity_plan_sha256 << '|'
            << plan.measurement_plan_sha256;
  for (const auto& beta : cfg.fixed_beta_by_link) {
    canonical << "|beta=" << beta.first << ':' << beta.second;
  }
  plan.plan_hash = Hex(Fnv1a64(canonical.str()));
  plan.plan_sha256 = "sha256:" + Sha256Hex(canonical.str());
  return plan;
}

const MeasurementPlanEntry& MeasurementForObservation(
    const PaperInputPlan& plan, const ObservationRecord& observation) {
  const size_t index = observation.ledger_index;
  if (index >= plan.observations.size() || index >= plan.measurements.size() ||
      &plan.observations[index] != &observation ||
      plan.measurements[index].observation_index != index ||
      plan.measurements[index].obs_id != observation.obs_id) {
    throw std::invalid_argument(
        "observation/measurement plan identity mismatch");
  }
  return plan.measurements[index];
}

MeasurementPlanEntry& MutableMeasurementForObservation(
    PaperInputPlan* plan, const ObservationRecord& observation) {
  if (!plan) throw std::invalid_argument("paper input plan is null");
  const auto& measurement = MeasurementForObservation(*plan, observation);
  return plan->measurements[measurement.observation_index];
}

std::vector<bool> AllSelectedObservationMask(const PaperInputPlan& plan) {
  if (plan.measurements.size() != plan.observations.size()) {
    throw std::invalid_argument(
        "measurement plan size does not match observation ledger");
  }
  std::vector<bool> mask(plan.observations.size(), false);
  for (size_t i = 0; i < plan.observations.size(); ++i) {
    const auto& measurement = plan.measurements[i];
    mask[i] = measurement.estimator_usable && measurement.selected;
  }
  return mask;
}

std::vector<bool> AllPlannedObservationMask(const PaperInputPlan& plan) {
  return AllSelectedObservationMask(plan);
}

FixedBetaValidation ValidatePaperFixedBeta(const PaperInputPlan& plan,
                                           const Config& cfg) {
  FixedBetaValidation result;
  result.configured_links = cfg.fixed_beta_by_link.size();
  std::set<std::string> used_links;
  for (const auto& record : plan.observations) {
    const auto& measurement = MeasurementForObservation(plan, record);
    if (measurement.estimator_usable && measurement.selected)
      used_links.insert(RangeLinkKey(record.tag_id, record.anchor_id));
  }
  result.used_links = used_links.size();
  if (cfg.fixed_beta_by_link.empty()) {
    result.status = "MISSING_CALIBRATION_DEVELOPMENT_ONLY";
    return result;
  }

  std::set<int> known_anchors;
  for (const auto& anchor : cfg.anchors) known_anchors.insert(anchor.id);
  for (const auto& entry : cfg.fixed_beta_by_link) {
    int tag_id = 0, anchor_id = 0;
    if (!ParseRangeLinkKey(entry.first, &tag_id, &anchor_id)) {
      throw std::invalid_argument("invalid fixed beta link key: " +
                                  entry.first);
    }
    if (!known_anchors.count(anchor_id)) {
      throw std::invalid_argument("fixed beta link uses unknown anchor: " +
                                  entry.first);
    }
  }

  std::vector<std::string> missing;
  for (const auto& link : used_links) {
    if (!cfg.fixed_beta_by_link.count(link)) missing.push_back(link);
  }
  if (!missing.empty()) {
    std::ostringstream message;
    message << "MISSING_CALIBRATION: fixed_beta_by_link does not cover "
            << missing.size() << " used link(s): ";
    for (size_t i = 0; i < missing.size(); ++i) {
      if (i) message << ',';
      message << missing[i];
    }
    throw std::invalid_argument(message.str());
  }
  result.status = "EXPLICIT_COMPLETE_NOT_PROVENANCE_VERIFIED";
  return result;
}

std::vector<UwbFrame> MaterializePaperKeyframes(
    const PaperInputPlan& plan, const std::vector<bool>& strategy_mask) {
  if (strategy_mask.size() != plan.observations.size()) {
    throw std::invalid_argument("strategy mask size does not match ledger");
  }
  if (plan.measurements.size() != plan.observations.size()) {
    throw std::invalid_argument(
        "measurement plan size does not match observation ledger");
  }

  std::vector<UwbFrame> frames;
  frames.reserve(plan.keyframes.size());
  for (const auto& entry : plan.keyframes) {
    UwbFrame frame;
    frame.t = entry.sensor_time;
    frame.tag_id = entry.tag_id;
    frames.push_back(frame);
  }
  for (size_t i = 0; i < plan.observations.size(); ++i) {
    const ObservationRecord& record = plan.observations[i];
    const MeasurementPlanEntry& measurement = plan.measurements[i];
    if (!measurement.estimator_usable || !measurement.selected ||
        !strategy_mask[i])
      continue;
    if (measurement.keyframe_id >= frames.size()) {
      throw std::runtime_error("paper input keyframe assignment is invalid");
    }
    UwbRange range;
    range.anchor_id = record.anchor_id;
    range.dist = record.raw_range;
    range.fp_rssi = record.fp_rssi;
    range.rx_rssi = record.rx_rssi;
    range.obs_id = record.obs_id;
    range.nominal_sigma = measurement.sensor_sigma;
    range.suspected_nlos = record.suspected_nlos;
    range.noise_semantics = UwbNoiseSemantics::FIXED_SENSOR_SIGMA_V2;
    range.source_message_index = record.source_message_index;
    range.source_range_index = record.source_range_index;
    range.source_obs_index = record.source_observation_index;
    range.source_time = record.sensor_time;
    range.source_tag_id = record.tag_id;
    frames[measurement.keyframe_id].ranges.push_back(range);
  }
  return frames;
}

}  // namespace uifgo
