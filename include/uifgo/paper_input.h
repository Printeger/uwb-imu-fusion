#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "uifgo/config.h"
#include "uifgo/types.h"

namespace uifgo {

enum class UwbIntegrityStatus {
  SOURCE_INVALID,
  ESTIMATOR_UNUSABLE,
  USABLE,
  STALE_REPEAT,
};

const char* UwbIntegrityStatusName(UwbIntegrityStatus status);

struct ObservationRecord {
  std::uint64_t obs_id = 0;
  size_t ledger_index = 0;
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
  // Source/protocol validity only. Anchor-model coverage, range-policy
  // usability, estimator selection and state association live in the separate
  // MeasurementPlanEntry at the same ledger index.
  bool source_valid = false;
  std::string source_validity_reason;
  bool suspected_nlos = false;
};

struct MeasurementPlanEntry {
  std::uint64_t obs_id = 0;
  size_t observation_index = 0;
  bool estimator_usable = false;
  std::string usability_reason;
  // Integrity/correlation is separate from source validity and NLOS
  // suspicion. Every raw row remains in ObservationRecord; only one selected
  // row per exact-repeat group may instantiate an independent likelihood.
  UwbIntegrityStatus integrity_status = UwbIntegrityStatus::SOURCE_INVALID;
  std::uint64_t correlation_group_id = 0;
  std::uint64_t correlation_representative_obs_id = 0;
  size_t correlation_group_size = 1;
  bool independent_likelihood_representative = false;
  bool selected = false;
  std::string selection_reason;
  size_t keyframe_id = std::numeric_limits<size_t>::max();
  double sensor_sigma = std::numeric_limits<double>::quiet_NaN();
};

struct KeyframePlanEntry {
  size_t keyframe_id = 0;
  size_t source_frame_index = 0;
  double sensor_time = 0.0;
  int tag_id = 0;
};

struct PaperInputPlan {
  std::string source_namespace;
  size_t state_step = 1;
  double state_min_interval = 0.0;
  std::string association_policy;
  std::string sensor_noise_model;
  std::string integrity_policy;
  std::string observation_ledger_hash;
  std::string observation_ledger_sha256;
  std::string state_timeline_hash;
  std::string state_timeline_sha256;
  std::string measurement_plan_hash;
  std::string measurement_plan_sha256;
  std::string integrity_plan_hash;
  std::string integrity_plan_sha256;
  // Backward-compatible composite identity used by existing artifacts/caches.
  std::string plan_hash;
  std::string plan_sha256;
  std::vector<ObservationRecord> observations;
  std::vector<KeyframePlanEntry> keyframes;
  // One-to-one with observations. Raw ledger membership never depends on this
  // estimator-specific layer.
  std::vector<MeasurementPlanEntry> measurements;
};

struct FixedBetaValidation {
  std::string status;
  size_t configured_links = 0;
  size_t used_links = 0;
};

// Build the immutable source ledger, state timeline, and measurement plan as
// distinct layers before applying any strategy mask. The input vector/range
// positions are retained as source identifiers before estimator planning.
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

const MeasurementPlanEntry& MeasurementForObservation(
    const PaperInputPlan& plan, const ObservationRecord& observation);
MeasurementPlanEntry& MutableMeasurementForObservation(
    PaperInputPlan* plan, const ObservationRecord& observation);

std::vector<bool> AllSelectedObservationMask(const PaperInputPlan& plan);
// Compatibility spelling retained for existing artifact/tool callers. Its
// meaning is exactly estimator-selected, not ledger membership.
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
