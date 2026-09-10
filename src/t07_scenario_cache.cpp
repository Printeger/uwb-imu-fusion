#include "uifgo/t07_scenario_cache.h"

#include "uifgo/hash_utils.h"
#include "uifgo/paper_input.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace uifgo {
namespace {

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream input(line);
  std::string field;
  while (std::getline(input, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

template <typename T>
T ParseInteger(const std::string& text, const char* field);

template <>
std::uint64_t ParseInteger<std::uint64_t>(const std::string& text,
                                         const char* field) {
  size_t used = 0;
  const auto value = std::stoull(text, &used);
  if (used != text.size()) throw std::runtime_error(std::string("invalid ") + field);
  return value;
}

template <>
int ParseInteger<int>(const std::string& text, const char* field) {
  size_t used = 0;
  const auto value = std::stoll(text, &used);
  if (used != text.size() || value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max())
    throw std::runtime_error(std::string("invalid ") + field);
  return static_cast<int>(value);
}

double ParseDouble(const std::string& text, const char* field) {
  size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value))
    throw std::runtime_error(std::string("invalid/nonfinite ") + field);
  return value;
}

bool ParseBool01(const std::string& text, const char* field) {
  if (text == "0") return false;
  if (text == "1") return true;
  throw std::runtime_error(std::string("invalid ") + field);
}

std::string DecodeHex(const std::string& text) {
  if (text.size() % 2 != 0) throw std::runtime_error("invalid hex text");
  std::string decoded;
  decoded.reserve(text.size() / 2);
  for (size_t i = 0; i < text.size(); i += 2) {
    const std::string byte = text.substr(i, 2);
    size_t used = 0;
    const unsigned long value = std::stoul(byte, &used, 16);
    if (used != 2 || value > 255) throw std::runtime_error("invalid hex text");
    decoded.push_back(static_cast<char>(value));
  }
  return decoded;
}

void RequireHeader(std::ifstream* input, const std::string& expected,
                   const std::string& name) {
  std::string header;
  if (!std::getline(*input, header) || header != expected)
    throw std::runtime_error("invalid " + name + " header");
}

bool InWindow(double time, double begin, double end) {
  return time >= begin && time <= end;
}

}  // namespace

std::string T07CacheCanonicalId(
    const std::string& base_recording_id,
    const std::string& base_source_sha256, double recording_time_origin_s,
    const std::string& imu_sha256, const std::string& uwb_sha256,
    size_t imu_count, size_t uwb_observation_count,
    size_t uwb_message_count) {
  std::ostringstream canonical;
  canonical << std::setprecision(17) << kT07EstimatorCacheSchema << '\n'
            << base_recording_id << '\n' << base_source_sha256 << '\n'
            << recording_time_origin_s << '\n'
            << "imu_units=m/s^2,rad/s,quaternion_wxyz\n"
            << "uwb_units=s,m,dBm\n"
            << "time_basis=sensor_time_from_recording_origin\n"
            << "grouping=source_message_index\n"
            << imu_sha256 << '\n' << uwb_sha256 << '\n' << imu_count << '\n'
            << uwb_observation_count << '\n' << uwb_message_count << '\n';
  return "sha256:" + Sha256Hex(canonical.str());
}

T07ScenarioCache LoadT07ScenarioCache(const std::string& manifest_path,
                                      double start_s, double duration_s) {
  namespace fs = boost::filesystem;
  if (!std::isfinite(start_s) || start_s < 0.0 ||
      !std::isfinite(duration_s) ||
      (duration_s < 0.0 && duration_s != -1.0))
    throw std::invalid_argument("invalid T07 cache window");
  const fs::path manifest = fs::canonical(fs::absolute(manifest_path));
  const YAML::Node root = YAML::LoadFile(manifest.string());
  const std::set<std::string> allowed = {
      "schema", "cache_id", "base_recording_id", "base_source_sha256",
      "time_basis", "recording_time_origin_s", "imu_units", "uwb_units",
      "uwb_message_grouping", "imu_file", "imu_sha256", "imu_count",
      "uwb_file", "uwb_sha256", "uwb_observation_count",
      "uwb_message_count"};
  if (!root.IsMap()) throw std::runtime_error("T07 cache manifest is not a map");
  for (const auto& kv : root) {
    const std::string key = kv.first.as<std::string>();
    if (!allowed.count(key))
      throw std::runtime_error("forbidden/unknown T07 cache field: " + key);
  }
  const std::vector<std::string> required(allowed.begin(), allowed.end());
  for (const auto& key : required)
    if (!root[key].IsDefined())
      throw std::runtime_error("missing T07 cache field: " + key);
  if (root["schema"].as<std::string>() != kT07EstimatorCacheSchema ||
      root["time_basis"].as<std::string>() !=
          "sensor_time_from_recording_origin" ||
      root["imu_units"].as<std::string>() !=
          "acc_mps2,gyro_radps,orientation_quaternion_wxyz" ||
      root["uwb_units"].as<std::string>() != "time_s,range_m,rssi_dbm" ||
      root["uwb_message_grouping"].as<std::string>() !=
          "source_message_index")
    throw std::runtime_error("unsupported T07 cache schema/units/time/grouping");
  if (root["imu_file"].as<std::string>() != "imu.csv" ||
      root["uwb_file"].as<std::string>() != "uwb_observations.csv")
    throw std::runtime_error("noncanonical T07 cache payload filenames");

  T07ScenarioCache result;
  result.cache_id = root["cache_id"].as<std::string>();
  result.base_recording_id = root["base_recording_id"].as<std::string>();
  result.base_source_sha256 = root["base_source_sha256"].as<std::string>();
  result.recording_time_origin_s =
      root["recording_time_origin_s"].as<double>();
  result.full_imu_count = root["imu_count"].as<size_t>();
  result.full_uwb_observation_count =
      root["uwb_observation_count"].as<size_t>();
  result.full_uwb_message_count = root["uwb_message_count"].as<size_t>();
  if (!std::isfinite(result.recording_time_origin_s) ||
      result.base_recording_id.empty())
    throw std::runtime_error("invalid T07 cache identity/time origin");

  const fs::path dir = manifest.parent_path();
  const fs::path imu_path = dir / root["imu_file"].as<std::string>();
  const fs::path uwb_path = dir / root["uwb_file"].as<std::string>();
  const std::string imu_sha = "sha256:" + Sha256FileHex(imu_path.string());
  const std::string uwb_sha = "sha256:" + Sha256FileHex(uwb_path.string());
  if (imu_sha != root["imu_sha256"].as<std::string>() ||
      uwb_sha != root["uwb_sha256"].as<std::string>())
    throw std::runtime_error("T07 cache payload hash mismatch");
  const std::string expected_id = T07CacheCanonicalId(
      result.base_recording_id, result.base_source_sha256,
      result.recording_time_origin_s, imu_sha, uwb_sha, result.full_imu_count,
      result.full_uwb_observation_count, result.full_uwb_message_count);
  if (result.cache_id != expected_id)
    throw std::runtime_error("T07 cache_id does not match manifest/payload");

  std::vector<ImuSample> full_imu;
  {
    std::ifstream input(imu_path.string());
    if (!input) throw std::runtime_error("cannot open T07 IMU cache");
    RequireHeader(&input,
                  "source_index,sensor_time_s,acc_x_mps2,acc_y_mps2,"
                  "acc_z_mps2,gyro_x_radps,gyro_y_radps,gyro_z_radps,"
                  "has_orientation,qw,qx,qy,qz",
                  "T07 IMU cache");
    std::string line;
    size_t row = 0;
    double previous_time = -std::numeric_limits<double>::infinity();
    while (std::getline(input, line)) {
      const auto f = SplitCsv(line);
      if (f.size() != 13) throw std::runtime_error("invalid T07 IMU row");
      if (ParseInteger<std::uint64_t>(f[0], "IMU source_index") != row)
        throw std::runtime_error("noncanonical T07 IMU source order");
      ImuSample sample;
      sample.t = ParseDouble(f[1], "IMU sensor_time");
      if (sample.t < previous_time)
        throw std::runtime_error("noncanonical T07 IMU sensor-time order");
      previous_time = sample.t;
      sample.acc = {ParseDouble(f[2], "acc_x"), ParseDouble(f[3], "acc_y"),
                    ParseDouble(f[4], "acc_z")};
      sample.gyro = {ParseDouble(f[5], "gyro_x"), ParseDouble(f[6], "gyro_y"),
                     ParseDouble(f[7], "gyro_z")};
      sample.has_orientation = ParseBool01(f[8], "has_orientation");
      sample.orientation =
          Eigen::Quaterniond(ParseDouble(f[9], "qw"),
                             ParseDouble(f[10], "qx"),
                             ParseDouble(f[11], "qy"),
                             ParseDouble(f[12], "qz"));
      if (!std::isfinite(sample.orientation.norm()))
        throw std::runtime_error("nonfinite cached IMU orientation");
      full_imu.push_back(sample);
      ++row;
    }
  }
  if (full_imu.size() != result.full_imu_count)
    throw std::runtime_error("T07 IMU count mismatch");

  std::vector<UwbFrame> full_uwb;
  {
    std::ifstream input(uwb_path.string());
    if (!input) throw std::runtime_error("cannot open T07 UWB cache");
    RequireHeader(&input,
                  "obs_id,source_message_index,source_range_index,"
                  "source_observation_index,sensor_time_s,tag_id,anchor_id,"
                  "observed_range_m,fp_rssi_dbm,rx_rssi_dbm,source_valid,"
                  "source_validity_reason_hex",
                  "T07 UWB cache");
    std::string line;
    std::unordered_set<std::uint64_t> ids;
    std::unordered_set<std::uint64_t> closed_messages;
    std::uint64_t current_message = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t previous_message = 0;
    std::uint64_t previous_source_observation = 0;
    bool have_previous_message = false;
    bool have_previous_source_observation = false;
    double previous_group_time = -std::numeric_limits<double>::infinity();
    while (std::getline(input, line)) {
      const auto f = SplitCsv(line);
      if (f.size() != 12) throw std::runtime_error("invalid T07 UWB row");
      const std::uint64_t obs_id = ParseInteger<std::uint64_t>(f[0], "obs_id");
      const std::uint64_t message =
          ParseInteger<std::uint64_t>(f[1], "source_message_index");
      const std::uint64_t range_index =
          ParseInteger<std::uint64_t>(f[2], "source_range_index");
      const std::uint64_t source_observation =
          ParseInteger<std::uint64_t>(f[3], "source_observation_index");
      const double time = ParseDouble(f[4], "UWB sensor_time");
      const int tag = ParseInteger<int>(f[5], "tag_id");
      if (message == std::numeric_limits<std::uint64_t>::max() ||
          source_observation == std::numeric_limits<std::uint64_t>::max() ||
          tag == std::numeric_limits<int>::min())
        throw std::runtime_error("missing T07 source ordinal/tag identity");
      if (!ids.insert(obs_id).second)
        throw std::runtime_error("duplicate T07 cache obs_id");
      if (message != current_message) {
        if (current_message != std::numeric_limits<std::uint64_t>::max())
          closed_messages.insert(current_message);
        if (closed_messages.count(message))
          throw std::runtime_error("noncontiguous T07 source message group");
        if (have_previous_message &&
            (time < previous_group_time || message <= previous_message))
          throw std::runtime_error(
              "noncanonical T07 source message group order");
        full_uwb.push_back({time, tag, {}});
        current_message = message;
        previous_message = message;
        previous_group_time = time;
        have_previous_message = true;
      }
      UwbFrame& frame = full_uwb.back();
      if (frame.t != time || frame.tag_id != tag ||
          range_index != frame.ranges.size())
        throw std::runtime_error("invalid T07 source message grouping");
      if (have_previous_source_observation &&
          source_observation <= previous_source_observation)
        throw std::runtime_error(
            "noncanonical T07 source observation order");
      UwbRange range;
      range.obs_id = obs_id;
      range.source_message_index = message;
      range.source_range_index = range_index;
      range.source_obs_index = source_observation;
      range.source_time = time;
      range.source_tag_id = tag;
      range.anchor_id = ParseInteger<int>(f[6], "anchor_id");
      range.dist = ParseDouble(f[7], "observed_range");
      range.fp_rssi = ParseDouble(f[8], "fp_rssi");
      range.rx_rssi = ParseDouble(f[9], "rx_rssi");
      range.source_valid = ParseBool01(f[10], "source_valid");
      range.source_validity_reason = DecodeHex(f[11]);
      if (range.obs_id != StableObservationId(result.base_recording_id,
                                              full_uwb.size() - 1,
                                              frame.ranges.size(), frame,
                                              range))
        throw std::runtime_error(
            "T07 cache obs_id does not match inherited source ordinals");
      frame.ranges.push_back(range);
      previous_source_observation = source_observation;
      have_previous_source_observation = true;
    }
  }
  size_t full_obs = 0;
  for (const auto& frame : full_uwb) full_obs += frame.ranges.size();
  if (full_obs != result.full_uwb_observation_count ||
      full_uwb.size() != result.full_uwb_message_count)
    throw std::runtime_error("T07 UWB count mismatch");

  const double begin = result.recording_time_origin_s + start_s;
  const double end = duration_s < 0.0
                         ? std::numeric_limits<double>::infinity()
                         : begin + duration_s;
  for (const auto& sample : full_imu)
    if (InWindow(sample.t, begin, end)) result.imu.push_back(sample);
  for (const auto& frame : full_uwb)
    if (InWindow(frame.t, begin, end)) result.uwb.push_back(frame);
  if (result.imu.empty() || result.uwb.empty())
    throw std::runtime_error("T07 cache window has incomplete IMU/UWB data");
  return result;
}

}  // namespace uifgo
