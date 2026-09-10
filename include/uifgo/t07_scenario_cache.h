#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "uifgo/types.h"

namespace uifgo {

inline constexpr const char* kT07EstimatorCacheSchema =
    "t07_estimator_cache_v1";

struct T07ScenarioCache {
  std::string cache_id;
  std::string base_recording_id;
  std::string base_source_sha256;
  double recording_time_origin_s = 0.0;
  size_t full_imu_count = 0;
  size_t full_uwb_observation_count = 0;
  size_t full_uwb_message_count = 0;
  std::vector<ImuSample> imu;
  std::vector<UwbFrame> uwb;
};

// Load and hash-validate a self-contained cache.  start/duration use sensor
// time relative to recording_time_origin_s and are applied to both IMU/UWB
// before initialization.  No unit conversion is performed while reading.
T07ScenarioCache LoadT07ScenarioCache(const std::string& manifest_path,
                                      double start_s, double duration_s);

std::string T07CacheCanonicalId(
    const std::string& base_recording_id,
    const std::string& base_source_sha256, double recording_time_origin_s,
    const std::string& imu_sha256, const std::string& uwb_sha256,
    size_t imu_count, size_t uwb_observation_count,
    size_t uwb_message_count);

}  // namespace uifgo
