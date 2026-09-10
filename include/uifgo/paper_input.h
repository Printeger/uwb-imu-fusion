#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "uifgo/config.h"
#include "uifgo/types.h"

namespace uifgo {

struct ObservationRecord {
  std::uint64_t obs_id = 0;
  size_t source_frame_index = 0;
  std::uint64_t source_message_index = 0;
  std::uint64_t source_range_index = 0;
  std::uint64_t source_observation_index = 0;
  double sensor_time = 0.0;
  int tag_id = 0;
  int anchor_id = 0;
  double raw_range = 0.0;
  double fp_rssi = 0.0;
  double rx_rssi = 0.0;
  bool valid = false;
  std::string validity_reason;
  bool suspected_nlos = false;
  bool planned = false;
  size_t keyframe_id = 0;
  double nominal_sigma = 0.0;
};

struct KeyframePlanEntry {
  size_t keyframe_id = 0;
  size_t source_frame_index = 0;
  double sensor_time = 0.0;
  int tag_id = 0;
};

struct PaperInputPlan {
  std::string source_namespace;
  std::string plan_hash;
  std::string plan_sha256;
  std::vector<ObservationRecord> observations;
  std::vector<KeyframePlanEntry> keyframes;
};

struct FixedBetaValidation {
  std::string status;
  size_t configured_links = 0;
  size_t used_links = 0;
};

// Build the immutable paper-path ledger before applying any strategy mask.
// The input vector/range positions are retained as source identifiers before
// the function performs chronological keyframe planning.
PaperInputPlan BuildPaperInputPlan(const std::vector<UwbFrame>& raw_frames,
                                   const Config& cfg,
                                   const std::string& source_namespace);

// Stable raw identity used by both the base loader and T07 cache generator.
// Values already carried by a cache are checked against this identity rather
// than trusted or renumbered from cache content.
std::uint64_t StableObservationId(const std::string& source_namespace,
                                  size_t frame_index, size_t range_index,
                                  const UwbFrame& frame,
                                  const UwbRange& range);

std::vector<bool> AllPlannedObservationMask(const PaperInputPlan& plan);

// Empty beta is allowed only as an explicit development state. A non-empty
// paper configuration must cover every link used by the frozen input plan.
FixedBetaValidation ValidatePaperFixedBeta(const PaperInputPlan& plan,
                                           const Config& cfg);

// Materialize one strategy view without recomputing keyframes or sigma.
// Empty keyframes remain present so every strategy uses the same IMU plan.
std::vector<UwbFrame> MaterializePaperKeyframes(
    const PaperInputPlan& plan, const std::vector<bool>& strategy_mask);

}  // namespace uifgo
