#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "uifgo/types.h"

namespace uifgo {

enum class T07InjectionShape { kStep, kRamp };

struct T07InjectionEvent {
  std::string event_id;
  std::string link;
  double start_s = 0.0;
  double end_s = 0.0;
  T07InjectionShape shape = T07InjectionShape::kStep;
  double amplitude_m = 0.0;
  double start_amplitude_m = 0.0;
  double slope_m_per_s = 0.0;
  size_t expected_match_count = 0;
  bool has_expected_match_count = false;
};

struct T07ScenarioRecipe {
  std::string scenario_id;
  std::uint64_t scenario_seed = 0;
  std::vector<T07InjectionEvent> events;
  std::string canonical_sha256;
};

struct T07InjectedTruthRow {
  std::uint64_t obs_id = 0;
  std::uint64_t source_message_index = 0;
  std::uint64_t source_range_index = 0;
  double sensor_time_s = 0.0;
  int tag_id = 0;
  int anchor_id = 0;
  double base_range_m = 0.0;
  double observed_range_m = 0.0;
  double injected_bias_delta_m = 0.0;
  std::string event_id;
  std::string shape;
};

struct T07InjectedScenario {
  std::vector<ImuSample> imu;
  std::vector<UwbFrame> uwb;
  std::vector<T07InjectedTruthRow> truth;
  std::vector<size_t> event_match_counts;
  double recording_time_origin_s = 0.0;
};

T07ScenarioRecipe LoadT07ScenarioRecipe(const std::string& path);

// Full-base generation requires nonzero event matches and enforces any locked
// expected counts. Prefix-level calls set require_full_event_matches=false;
// the inherited origin/identity still make common historical observations
// identical even when a prefix contains no event observations.
T07InjectedScenario InjectT07Scenario(
    const std::vector<ImuSample>& imu, const std::vector<UwbFrame>& base_uwb,
    const std::string& base_recording_id, const T07ScenarioRecipe& recipe,
    double inherited_recording_time_origin_s,
    bool require_full_event_matches);

void WriteT07ScenarioCache(const T07InjectedScenario& scenario,
                           const T07ScenarioRecipe& recipe,
                           const std::string& base_recording_id,
                           const std::string& base_source_sha256,
                           const std::string& base_audit_path,
                           const std::string& cache_root,
                           const std::string& truth_root,
                           std::string* cache_id_out,
                           std::string* cache_manifest_out);

}  // namespace uifgo
