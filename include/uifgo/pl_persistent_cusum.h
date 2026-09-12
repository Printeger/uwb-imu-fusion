#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "uifgo/nlos_support.h"

namespace uifgo {

inline constexpr const char* kPlPersistentCusumAlgorithmVersion =
    "UIFGO_PL_PERSISTENT_CUSUM_V1";
inline constexpr const char* kPlPersistentCusumSignalVersion =
    "PL_GAUSSIAN_CONDITIONAL_Z_V1";
inline constexpr const char* kPlPersistentCusumProvider =
    "pl_persistent_cusum_preflight_v1";
inline constexpr const char* kPlPersistentCusumPartitionRule =
    "PL_CUSUM_LAST_ZERO_BACKFILL_V1";

struct PlCusumLinkKey {
  int tag_id = 0;
  int anchor_id = 0;
  bool operator<(const PlCusumLinkKey& other) const {
    return tag_id < other.tag_id ||
           (tag_id == other.tag_id && anchor_id < other.anchor_id);
  }
  bool operator==(const PlCusumLinkKey& other) const {
    return tag_id == other.tag_id && anchor_id == other.anchor_id;
  }
};

// Deliberately contains no group statistic, DoF, omnibus alarm, truth, RSSI,
// or injection metadata. conditional_z is the sole scientific detector input.
struct PlCusumInputRow {
  double timestamp = std::numeric_limits<double>::quiet_NaN();
  std::string group_id;
  size_t keyframe_id = 0;
  size_t source_order = 0;
  int tag_id = 0;
  int anchor_id = 0;
  std::uint64_t obs_id = 0;
  double conditional_z = std::numeric_limits<double>::quiet_NaN();
  bool diagnostic_valid = false;
};

struct PlCusumOptions {
  double kappa = 0.5;
  double threshold = 5.0;
  double gap_threshold_s = 1.0;
  std::string calibration_hash;
  std::string split_manifest_hash;
};

struct PlCusumTraceRow {
  PlCusumInputRow input;
  double increment = std::numeric_limits<double>::quiet_NaN();
  double g_before = 0.0;
  double g_after = 0.0;
  bool reset = false;
  std::string reset_reason;
  std::string excursion_id;
  double excursion_start_time = std::numeric_limits<double>::quiet_NaN();
  bool threshold_crossing = false;
  bool first_alarm_for_excursion = false;
  bool candidate = false;
};

struct PlCusumResult {
  bool valid = false;
  std::string status = "NOT_EVALUATED";
  std::string detector_identity;
  std::vector<PlCusumTraceRow> trace;
  SupportPartition support;
  size_t valid_row_count = 0;
  size_t invalid_row_count = 0;
  size_t alarm_count = 0;
  size_t alarm_link_count = 0;
  size_t segment_count = 0;
  double max_g = 0.0;
  std::map<PlCusumLinkKey, double> per_link_max_g;
  std::map<std::string, size_t> reset_counts;
};

std::string PlPersistentCusumIdentity(const PlCusumOptions& options);

// Input is checked in its supplied order for reverse time within each link,
// then deterministically sorted by (timestamp,keyframe,source_order,obs_id).
PlCusumResult EvaluatePlPersistentCusum(
    const std::vector<PlCusumInputRow>& rows,
    const PlCusumOptions& options);

}  // namespace uifgo
