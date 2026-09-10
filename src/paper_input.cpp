#include "uifgo/paper_input.h"
#include "uifgo/hash_utils.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

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

double AdaptiveSigma(const Config& cfg, double dt_since_last) {
  const double extra =
      dt_since_last > 0.0 ? cfg.v_max * dt_since_last / 3.0 : 0.0;
  return std::sqrt(cfg.sigma_range * cfg.sigma_range + extra * extra);
}

}  // namespace

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
  std::vector<std::vector<size_t>> record_indices(raw_frames.size());
  std::vector<bool> frame_has_valid(raw_frames.size(), false);
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
        record.validity_reason = range.source_validity_reason.empty()
                                     ? "SOURCE_PROTOCOL_INVALID"
                                     : range.source_validity_reason;
      } else if (!std::isfinite(source_time) || !std::isfinite(range.dist) ||
          !std::isfinite(range.fp_rssi) || !std::isfinite(range.rx_rssi)) {
        record.validity_reason = "NONFINITE";
      } else if (!known_anchors.count(range.anchor_id)) {
        record.validity_reason = "UNKNOWN_ANCHOR";
      } else if (range.dist < cfg.min_range || range.dist > cfg.max_range) {
        record.validity_reason = "INVALID_RANGE";
      } else {
        record.valid = true;
        record.validity_reason = "VALID";
        record.suspected_nlos =
            range.rx_rssi - range.fp_rssi > cfg.nlos_rssi_diff;
        frame_has_valid[fi] = true;
      }

      record_indices[fi].push_back(plan.observations.size());
      plan.observations.push_back(record);
    }
  }

  if (tags.size() > 1) {
    throw std::invalid_argument(
        "paper input T02 supports exactly one tag per run");
  }

  std::vector<size_t> eligible_frames;
  for (size_t fi = 0; fi < raw_frames.size(); ++fi) {
    if (frame_has_valid[fi]) eligible_frames.push_back(fi);
  }
  std::stable_sort(eligible_frames.begin(), eligible_frames.end(),
                   [&](size_t lhs, size_t rhs) {
                     if (raw_frames[lhs].t != raw_frames[rhs].t)
                       return raw_frames[lhs].t < raw_frames[rhs].t;
                     return lhs < rhs;
                   });

  double last_keyframe_time = -std::numeric_limits<double>::infinity();
  std::unordered_map<std::string, double> last_link_time;
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
      ObservationRecord& record = plan.observations[record_index];
      if (!record.valid) continue;
      record.planned = true;
      record.keyframe_id = kf;
      const std::string link = RangeLinkKey(record.tag_id, record.anchor_id);
      double dt = 0.0;
      const auto last = last_link_time.find(link);
      if (last != last_link_time.end()) dt = record.sensor_time - last->second;
      record.nominal_sigma = AdaptiveSigma(cfg, dt);
      last_link_time[link] = record.sensor_time;
    }
  }

  std::ostringstream canonical;
  canonical << std::setprecision(17) << "paper_input_v2|" << source_namespace
            << "|kf_step=" << cfg.kf_step
            << "|kf_min_interval=" << cfg.kf_min_interval;
  for (const auto& record : plan.observations) {
    canonical << '|' << record.obs_id << ':' << record.valid << ':'
              << record.validity_reason << ':' << record.suspected_nlos << ':'
              << record.planned << ':' << record.sensor_time << ':'
              << record.raw_range << ':' << record.fp_rssi << ':'
              << record.rx_rssi;
    if (record.planned) {
      canonical << ':' << record.keyframe_id << ':' << record.nominal_sigma;
    }
  }
  for (const auto& beta : cfg.fixed_beta_by_link) {
    canonical << "|beta=" << beta.first << ':' << beta.second;
  }
  plan.plan_hash = Hex(Fnv1a64(canonical.str()));
  plan.plan_sha256 = "sha256:" + Sha256Hex(canonical.str());
  return plan;
}

std::vector<bool> AllPlannedObservationMask(const PaperInputPlan& plan) {
  std::vector<bool> mask(plan.observations.size(), false);
  for (size_t i = 0; i < plan.observations.size(); ++i) {
    mask[i] = plan.observations[i].valid && plan.observations[i].planned;
  }
  return mask;
}

FixedBetaValidation ValidatePaperFixedBeta(const PaperInputPlan& plan,
                                           const Config& cfg) {
  FixedBetaValidation result;
  result.configured_links = cfg.fixed_beta_by_link.size();
  std::set<std::string> used_links;
  for (const auto& record : plan.observations) {
    if (record.valid && record.planned)
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
    if (!record.valid || !record.planned || !strategy_mask[i]) continue;
    if (record.keyframe_id >= frames.size()) {
      throw std::runtime_error("paper input keyframe assignment is invalid");
    }
    UwbRange range;
    range.anchor_id = record.anchor_id;
    range.dist = record.raw_range;
    range.fp_rssi = record.fp_rssi;
    range.rx_rssi = record.rx_rssi;
    range.obs_id = record.obs_id;
    range.nominal_sigma = record.nominal_sigma;
    range.suspected_nlos = record.suspected_nlos;
    range.source_message_index = record.source_message_index;
    range.source_range_index = record.source_range_index;
    range.source_obs_index = record.source_observation_index;
    range.source_time = record.sensor_time;
    range.source_tag_id = record.tag_id;
    frames[record.keyframe_id].ranges.push_back(range);
  }
  return frames;
}

}  // namespace uifgo
