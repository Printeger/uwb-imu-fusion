#pragma once

#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "uifgo/nlos_support.h"
#include "uifgo/pl_persistent_cusum.h"

namespace uifgo {

inline constexpr const char* kPlBackwardCusumAlgorithmVersion =
    "UIFGO_PL_BACKWARD_CUSUM_CLOSURE_V1";
inline constexpr const char* kPlBidirectionalSupportVersion =
    "UIFGO_PL_BIDIRECTIONAL_CUSUM_SUPPORT_V1";
inline constexpr const char* kPlBidirectionalIntersectionRule =
    "EXACT_SAME_LINK_OBS_ID_FORWARD_AND_BACKWARD_V1";

struct PlBackwardCusumOptions {
  double kappa = 0.5;
  double threshold = 5.0;
  double gap_threshold_s = 1.0;
  std::string original_clean_split_manifest_hash;
  std::string original_clean_input_hash;
  std::string calibration_hash;
};

struct PlBackwardCusumTraceRow {
  PlCusumInputRow input;
  size_t reverse_index = 0;
  double increment = std::numeric_limits<double>::quiet_NaN();
  double b_before = 0.0;
  double b_after = 0.0;
  bool reset = false;
  std::string reset_reason;
  std::string reverse_excursion_id;
  double reverse_excursion_start_time =
      std::numeric_limits<double>::quiet_NaN();
  bool threshold_crossing = false;
  bool first_crossing_for_excursion = false;
  bool backward_candidate = false;
};

struct PlBackwardCusumResult {
  bool valid = false;
  std::string status = "NOT_EVALUATED";
  std::string closure_identity;
  std::vector<PlBackwardCusumTraceRow> trace;
  SupportPartition support;
  size_t valid_row_count = 0;
  size_t invalid_row_count = 0;
  size_t alarm_count = 0;
  size_t alarm_link_count = 0;
  size_t segment_count = 0;
  double max_b = 0.0;
  std::map<PlCusumLinkKey, double> per_link_max_b;
  std::map<std::string, size_t> reset_counts;
};

std::string PlBackwardCusumIdentity(
    const PlBackwardCusumOptions& options);

// Dedicated reverse-time API. It sorts each link in descending physical time
// and does not relax the forward API's reverse-time fail-closed contract.
PlBackwardCusumResult EvaluatePlBackwardCusum(
    const std::vector<PlCusumInputRow>& rows,
    const PlBackwardCusumOptions& options);

struct PlBidirectionalMembershipRow {
  PlCusumInputRow input;
  bool forward_candidate = false;
  bool backward_candidate = false;
  bool final_candidate = false;
  std::string segment_id;
  size_t segment_ordinal = std::numeric_limits<size_t>::max();
};

struct PlBidirectionalSupportResult {
  bool valid = false;
  std::string status = "NOT_EVALUATED";
  std::string support_identity;
  std::string forward_detector_identity;
  std::string backward_closure_identity;
  std::vector<PlBidirectionalMembershipRow> rows;
  SupportPartition support;
};

// Requires exact equality of the forward/backward (link,obs_id) universes.
// The final candidate is the literal Boolean AND; there is no fallback.
PlBidirectionalSupportResult IntersectPlCusumSupport(
    const std::vector<PlBidirectionalMembershipRow>& forward_rows,
    const std::vector<PlBidirectionalMembershipRow>& backward_rows,
    const std::string& forward_detector_identity,
    const std::string& backward_closure_identity,
    double gap_threshold_s = 1.0);

}  // namespace uifgo
