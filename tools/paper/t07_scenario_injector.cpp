#include "uifgo/t07_scenario_injector.h"

#include "uifgo/config.h"
#include "uifgo/hash_utils.h"
#include "uifgo/paper_input.h"
#include "uifgo/t07_scenario_cache.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace uifgo {
namespace {

std::string ShapeName(T07InjectionShape shape) {
  return shape == T07InjectionShape::kStep ? "step" : "ramp";
}

bool IsSafeId(const std::string& value) {
  if (value.empty()) return false;
  for (unsigned char c : value) {
    if (!(std::isalnum(c) || c == '_' || c == '-' || c == '.')) return false;
  }
  return true;
}

std::string HexEncode(const std::string& text) {
  static const char* digits = "0123456789abcdef";
  std::string encoded;
  encoded.reserve(text.size() * 2);
  for (unsigned char c : text) {
    encoded.push_back(digits[c >> 4]);
    encoded.push_back(digits[c & 15]);
  }
  return encoded;
}

void WriteFile(const boost::filesystem::path& path, const std::string& text) {
  std::ofstream out(path.string(), std::ios::binary);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out.write(text.data(), static_cast<std::streamsize>(text.size()));
  out.close();
  if (!out) throw std::runtime_error("failed writing " + path.string());
}

bool PathEntryExists(const boost::filesystem::path& path) {
  return boost::filesystem::exists(path) || boost::filesystem::is_symlink(path);
}

boost::filesystem::path ResolvePhysicalCandidate(
    const boost::filesystem::path& input) {
  namespace fs = boost::filesystem;
  fs::path current = fs::absolute(input).lexically_normal();
  std::vector<fs::path> missing;
  while (!PathEntryExists(current)) {
    if (current == current.root_path())
      throw std::runtime_error("cannot resolve T07 output root");
    missing.push_back(current.filename());
    current = current.parent_path();
  }
  fs::path resolved = fs::canonical(current);
  for (auto it = missing.rbegin(); it != missing.rend(); ++it)
    resolved /= *it;
  return resolved.lexically_normal();
}

bool IsSameOrAncestor(const boost::filesystem::path& parent,
                      const boost::filesystem::path& child) {
  auto parent_it = parent.begin();
  auto child_it = child.begin();
  for (; parent_it != parent.end() && child_it != child.end();
       ++parent_it, ++child_it) {
    if (*parent_it != *child_it) return false;
  }
  return parent_it == parent.end();
}

void RequireDisjointPhysicalPaths(const boost::filesystem::path& cache,
                                  const boost::filesystem::path& truth) {
  if (IsSameOrAncestor(cache, truth) || IsSameOrAncestor(truth, cache))
    throw std::runtime_error(
        "T07 cache and truth paths must be physically disjoint siblings");
}

void ValidateCanonicalScenario(const T07InjectedScenario& scenario,
                               const std::string& base_recording_id) {
  if (scenario.imu.empty() || scenario.uwb.empty())
    throw std::runtime_error("T07 cache requires nonempty IMU/UWB input");
  double previous_imu_time = -std::numeric_limits<double>::infinity();
  for (const auto& sample : scenario.imu) {
    if (!std::isfinite(sample.t) || sample.t < previous_imu_time ||
        !sample.acc.allFinite() || !sample.gyro.allFinite() ||
        !std::isfinite(sample.orientation.norm()))
      throw std::runtime_error("noncanonical T07 IMU order/value");
    previous_imu_time = sample.t;
  }

  bool have_group = false;
  bool have_source_observation = false;
  std::uint64_t previous_message = 0;
  std::uint64_t previous_source_observation = 0;
  double previous_time = -std::numeric_limits<double>::infinity();
  size_t truth_index = 0;
  for (size_t frame_index = 0; frame_index < scenario.uwb.size();
       ++frame_index) {
    const auto& frame = scenario.uwb[frame_index];
    if (frame.ranges.empty())
      throw std::runtime_error("T07 cache cannot encode empty UWB group");
    const auto& first = frame.ranges.front();
    const std::uint64_t message = first.source_message_index;
    const double time = first.source_time;
    const int tag = first.source_tag_id;
    if (message == std::numeric_limits<std::uint64_t>::max() ||
        tag == std::numeric_limits<int>::min() || !std::isfinite(time) ||
        frame.t != time || frame.tag_id != tag ||
        (have_group && (time < previous_time || message <= previous_message)))
      throw std::runtime_error("noncanonical T07 source message group order");
    for (size_t range_index = 0; range_index < frame.ranges.size();
         ++range_index) {
      const auto& range = frame.ranges[range_index];
      if (range.source_message_index != message ||
          range.source_range_index != range_index ||
          range.source_obs_index ==
              std::numeric_limits<std::uint64_t>::max() ||
          range.source_time != time || range.source_tag_id != tag ||
          !std::isfinite(range.dist) || !std::isfinite(range.fp_rssi) ||
          !std::isfinite(range.rx_rssi) ||
          (have_source_observation &&
           range.source_obs_index <= previous_source_observation) ||
          range.obs_id != StableObservationId(base_recording_id, frame_index,
                                              range_index, frame, range))
        throw std::runtime_error("noncanonical T07 source range identity/order");
      if (truth_index >= scenario.truth.size() ||
          scenario.truth[truth_index].obs_id != range.obs_id ||
          scenario.truth[truth_index].source_message_index != message ||
          scenario.truth[truth_index].source_range_index != range_index ||
          scenario.truth[truth_index].sensor_time_s != time ||
          scenario.truth[truth_index].observed_range_m != range.dist)
        throw std::runtime_error("T07 truth/cache observation order mismatch");
      previous_source_observation = range.source_obs_index;
      have_source_observation = true;
      ++truth_index;
    }
    previous_message = message;
    previous_time = time;
    have_group = true;
  }
  if (truth_index != scenario.truth.size())
    throw std::runtime_error("T07 truth/cache observation count mismatch");
}

boost::filesystem::path UniqueStagingPath(
    const boost::filesystem::path& root, const std::string& kind,
    const std::string& id_hex) {
  namespace fs = boost::filesystem;
  for (unsigned int attempt = 0; attempt < 1000; ++attempt) {
    const fs::path candidate =
        root / (".t07-staging-" + kind + "-" + id_hex + "-" +
                std::to_string(static_cast<unsigned long>(getpid())) + "-" +
                std::to_string(attempt));
    if (!PathEntryExists(candidate)) return candidate;
  }
  throw std::runtime_error("cannot allocate unique T07 staging directory");
}

void RequireExactFiles(const boost::filesystem::path& dir,
                       const std::set<std::string>& expected) {
  std::set<std::string> actual;
  for (boost::filesystem::directory_iterator it(dir), end; it != end; ++it) {
    if (!boost::filesystem::is_regular_file(it->symlink_status()))
      throw std::runtime_error("non-file entry in finalized T07 output");
    actual.insert(it->path().filename().string());
  }
  if (actual != expected)
    throw std::runtime_error("unexpected finalized T07 output file set");
}

std::string EventCanonical(const T07ScenarioRecipe& recipe) {
  std::ostringstream out;
  out << std::setprecision(17) << "t07_scenario_v1\n" << recipe.scenario_id
      << '\n' << recipe.scenario_seed << '\n'
      << "sensor_time_from_recording_origin\n";
  for (const auto& event : recipe.events) {
    out << event.event_id << '|' << event.link << '|' << event.start_s << '|'
        << event.end_s << '|' << ShapeName(event.shape) << '|'
        << event.amplitude_m << '|' << event.start_amplitude_m << '|'
        << event.slope_m_per_s << '|';
    if (event.has_expected_match_count) out << event.expected_match_count;
    out << '\n';
  }
  return out.str();
}

double EventDelta(const T07InjectionEvent& event, double relative_time) {
  if (event.shape == T07InjectionShape::kStep) return event.amplitude_m;
  return event.start_amplitude_m +
         event.slope_m_per_s * (relative_time - event.start_s);
}

void ValidateEvent(const T07InjectionEvent& event) {
  int tag = 0, anchor = 0;
  if (!IsSafeId(event.event_id) ||
      !ParseRangeLinkKey(event.link, &tag, &anchor) ||
      !std::isfinite(event.start_s) || !std::isfinite(event.end_s) ||
      event.start_s < 0.0 || !(event.end_s > event.start_s))
    throw std::runtime_error("invalid T07 event identity/link/interval");
  if (event.shape == T07InjectionShape::kStep) {
    if (!std::isfinite(event.amplitude_m) || event.amplitude_m < 0.0)
      throw std::runtime_error("invalid nonnegative T07 step amplitude");
  } else {
    const double end_value = event.start_amplitude_m +
                             event.slope_m_per_s *
                                 (event.end_s - event.start_s);
    if (!std::isfinite(event.start_amplitude_m) ||
        !std::isfinite(event.slope_m_per_s) || !std::isfinite(end_value) ||
        std::min(event.start_amplitude_m, end_value) < 0.0)
      throw std::runtime_error("invalid nonnegative T07 ramp");
  }
}

}  // namespace

T07ScenarioRecipe LoadT07ScenarioRecipe(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  const std::set<std::string> root_allowed = {
      "schema", "scenario_id", "scenario_seed", "time_basis", "events"};
  if (!root.IsMap()) throw std::runtime_error("T07 recipe must be a map");
  for (const auto& kv : root) {
    const std::string key = kv.first.as<std::string>();
    if (!root_allowed.count(key))
      throw std::runtime_error("unknown T07 recipe field: " + key);
  }
  if (!root["schema"] || root["schema"].as<std::string>() !=
                             "t07_scenario_v1" ||
      !root["scenario_id"] || !root["scenario_seed"] ||
      !root["time_basis"] ||
      root["time_basis"].as<std::string>() !=
          "sensor_time_from_recording_origin" ||
      !root["events"] || !root["events"].IsSequence())
    throw std::runtime_error("missing/unsupported T07 recipe contract");
  T07ScenarioRecipe recipe;
  recipe.scenario_id = root["scenario_id"].as<std::string>();
  recipe.scenario_seed = root["scenario_seed"].as<std::uint64_t>();
  if (!IsSafeId(recipe.scenario_id) || recipe.scenario_seed == 0 ||
      root["events"].size() == 0)
    throw std::runtime_error("T07 recipe requires id, nonzero seed and events");
  std::set<std::string> event_ids;
  std::map<std::string, std::vector<std::pair<double, double>>> intervals;
  for (const auto& item : root["events"]) {
    const std::set<std::string> allowed = {
        "id", "link", "start_s", "end_s", "shape", "amplitude_m",
        "start_amplitude_m", "slope_m_per_s", "expected_match_count"};
    for (const auto& kv : item) {
      const std::string key = kv.first.as<std::string>();
      if (!allowed.count(key))
        throw std::runtime_error("unknown T07 event field: " + key);
    }
    T07InjectionEvent event;
    if (!item["id"] || !item["link"] || !item["start_s"] ||
        !item["end_s"] || !item["shape"])
      throw std::runtime_error("missing T07 event field");
    event.event_id = item["id"].as<std::string>();
    event.link = item["link"].as<std::string>();
    event.start_s = item["start_s"].as<double>();
    event.end_s = item["end_s"].as<double>();
    const std::string shape = item["shape"].as<std::string>();
    if (shape == "step") {
      event.shape = T07InjectionShape::kStep;
      if (!item["amplitude_m"] || item["start_amplitude_m"] ||
          item["slope_m_per_s"])
        throw std::runtime_error("step requires only amplitude_m");
      event.amplitude_m = item["amplitude_m"].as<double>();
    } else if (shape == "ramp") {
      event.shape = T07InjectionShape::kRamp;
      if (item["amplitude_m"] || !item["start_amplitude_m"] ||
          !item["slope_m_per_s"])
        throw std::runtime_error(
            "ramp requires start_amplitude_m and slope_m_per_s");
      event.start_amplitude_m = item["start_amplitude_m"].as<double>();
      event.slope_m_per_s = item["slope_m_per_s"].as<double>();
    } else {
      throw std::runtime_error("unsupported T07 injection shape");
    }
    if (item["expected_match_count"]) {
      event.expected_match_count = item["expected_match_count"].as<size_t>();
      event.has_expected_match_count = true;
    }
    ValidateEvent(event);
    if (!event_ids.insert(event.event_id).second)
      throw std::runtime_error("duplicate T07 event id");
    for (const auto& prior : intervals[event.link]) {
      if (event.start_s < prior.second && prior.first < event.end_s)
        throw std::runtime_error("overlapping T07 events on one link");
    }
    intervals[event.link].push_back({event.start_s, event.end_s});
    recipe.events.push_back(event);
  }
  recipe.canonical_sha256 = "sha256:" + Sha256Hex(EventCanonical(recipe));
  return recipe;
}

T07InjectedScenario InjectT07Scenario(
    const std::vector<ImuSample>& imu, const std::vector<UwbFrame>& base_uwb,
    const std::string& base_recording_id, const T07ScenarioRecipe& recipe,
    double inherited_recording_time_origin_s,
    bool require_full_event_matches) {
  if (imu.empty() || base_uwb.empty() || base_recording_id.empty() ||
      !std::isfinite(inherited_recording_time_origin_s))
    throw std::invalid_argument("incomplete T07 base input/origin/identity");
  T07InjectedScenario result;
  result.imu = imu;
  result.uwb = base_uwb;
  result.recording_time_origin_s = inherited_recording_time_origin_s;
  result.event_match_counts.assign(recipe.events.size(), 0);
  std::set<std::uint64_t> ids;
  for (size_t fi = 0; fi < base_uwb.size(); ++fi) {
    for (size_t ri = 0; ri < base_uwb[fi].ranges.size(); ++ri) {
      const auto& base = base_uwb[fi].ranges[ri];
      auto& observed = result.uwb[fi].ranges[ri];
      const std::uint64_t obs_id = StableObservationId(
          base_recording_id, fi, ri, base_uwb[fi], base);
      if (!ids.insert(obs_id).second)
        throw std::runtime_error("T07 base obs_id collision");
      observed.obs_id = obs_id;
      const double sensor_time = std::isfinite(base.source_time)
                                     ? base.source_time
                                     : base_uwb[fi].t;
      const int tag = base.source_tag_id == std::numeric_limits<int>::min()
                          ? base_uwb[fi].tag_id
                          : base.source_tag_id;
      const double relative_time =
          sensor_time - inherited_recording_time_origin_s;
      const std::string link = RangeLinkKey(tag, base.anchor_id);
      double delta = 0.0;
      std::string event_id;
      std::string shape = "none";
      for (size_t ei = 0; ei < recipe.events.size(); ++ei) {
        const auto& event = recipe.events[ei];
        if (event.link == link && relative_time >= event.start_s &&
            relative_time < event.end_s) {
          if (!event_id.empty())
            throw std::runtime_error("multiple T07 events matched one obs_id");
          delta = EventDelta(event, relative_time);
          event_id = event.event_id;
          shape = ShapeName(event.shape);
          ++result.event_match_counts[ei];
        }
      }
      const double final_range = base.dist + delta;
      if (!std::isfinite(delta) || delta < 0.0 ||
          !std::isfinite(final_range) || final_range < 0.0)
        throw std::runtime_error("nonfinite/negative T07 generated range");
      observed.dist = final_range;
      result.truth.push_back(
          {obs_id, base.source_message_index, base.source_range_index,
           sensor_time, tag, base.anchor_id, base.dist, final_range, delta,
           event_id, shape});
    }
  }
  if (require_full_event_matches) {
    for (size_t i = 0; i < recipe.events.size(); ++i) {
      if (result.event_match_counts[i] == 0)
        throw std::runtime_error("T07 full base event matched zero observations");
      if (recipe.events[i].has_expected_match_count &&
          result.event_match_counts[i] != recipe.events[i].expected_match_count)
        throw std::runtime_error("T07 frozen fixture match count changed");
    }
  }
  return result;
}

void WriteT07ScenarioCache(const T07InjectedScenario& scenario,
                           const T07ScenarioRecipe& recipe,
                           const std::string& base_recording_id,
                           const std::string& base_source_sha256,
                           const std::string& base_audit_path,
                           const std::string& cache_root,
                           const std::string& truth_root,
                           std::string* cache_id_out,
                           std::string* cache_manifest_out) {
  namespace fs = boost::filesystem;
  const fs::path cache_candidate = ResolvePhysicalCandidate(cache_root);
  const fs::path truth_candidate = ResolvePhysicalCandidate(truth_root);
  RequireDisjointPhysicalPaths(cache_candidate, truth_candidate);
  const YAML::Node audit = YAML::LoadFile(base_audit_path);
  if (!audit["schema"] ||
      audit["schema"].as<std::string>() != "t07_base_component_audit_v1" ||
      !audit["base_source_sha256"] ||
      audit["base_source_sha256"].as<std::string>() != base_source_sha256 ||
      !audit["base_recording_id"] ||
      audit["base_recording_id"].as<std::string>() != base_recording_id ||
      !audit["recording_time_origin_s"] ||
      audit["recording_time_origin_s"].as<double>() !=
          scenario.recording_time_origin_s ||
      !audit["fixture_scope"] ||
      audit["fixture_scope"].as<std::string>() !=
          "FROZEN_DEVELOPMENT_FIXTURE_ONLY" ||
      !audit["truth_semantics"] ||
      audit["truth_semantics"].as<std::string>() !=
          "INJECTED_COMPONENT_ONLY" ||
      !audit["total_latent_bias_status"] ||
      audit["total_latent_bias_status"].as<std::string>() != "UNKNOWN")
    throw std::runtime_error("T07 base audit does not authorize component-only truth");

  ValidateCanonicalScenario(scenario, base_recording_id);

  std::ostringstream imu;
  imu << "source_index,sensor_time_s,acc_x_mps2,acc_y_mps2,acc_z_mps2,"
         "gyro_x_radps,gyro_y_radps,gyro_z_radps,has_orientation,qw,qx,qy,qz\n"
      << std::setprecision(17);
  for (size_t i = 0; i < scenario.imu.size(); ++i) {
    const auto& s = scenario.imu[i];
    imu << i << ',' << s.t << ',' << s.acc.x() << ',' << s.acc.y() << ','
        << s.acc.z() << ',' << s.gyro.x() << ',' << s.gyro.y() << ','
        << s.gyro.z() << ',' << s.has_orientation << ',' << s.orientation.w()
        << ',' << s.orientation.x() << ',' << s.orientation.y() << ','
        << s.orientation.z() << '\n';
  }
  std::ostringstream uwb;
  uwb << "obs_id,source_message_index,source_range_index,"
         "source_observation_index,sensor_time_s,tag_id,anchor_id,"
         "observed_range_m,fp_rssi_dbm,rx_rssi_dbm,source_valid,"
         "source_validity_reason_hex\n"
      << std::setprecision(17);
  size_t obs_count = 0;
  for (const auto& frame : scenario.uwb) {
    for (const auto& r : frame.ranges) {
      uwb << r.obs_id << ',' << r.source_message_index << ','
          << r.source_range_index << ',' << r.source_obs_index << ','
          << r.source_time << ',' << r.source_tag_id << ',' << r.anchor_id
          << ',' << r.dist << ',' << r.fp_rssi << ',' << r.rx_rssi << ','
          << r.source_valid << ',' << HexEncode(r.source_validity_reason)
          << '\n';
      ++obs_count;
    }
  }
  std::ostringstream truth;
  truth << "obs_id,source_message_index,source_range_index,sensor_time_s,"
           "tag_id,anchor_id,base_range_m,observed_range_m,"
           "injected_bias_delta_m,event_id_hex,shape\n"
        << std::setprecision(17);
  for (const auto& row : scenario.truth) {
    truth << row.obs_id << ',' << row.source_message_index << ','
          << row.source_range_index << ',' << row.sensor_time_s << ','
          << row.tag_id << ',' << row.anchor_id << ',' << row.base_range_m
          << ',' << row.observed_range_m << ','
          << row.injected_bias_delta_m << ',' << HexEncode(row.event_id) << ','
          << row.shape << '\n';
  }

  const std::string imu_sha = "sha256:" + Sha256Hex(imu.str());
  const std::string uwb_sha = "sha256:" + Sha256Hex(uwb.str());
  const std::string cache_id = T07CacheCanonicalId(
      base_recording_id, base_source_sha256,
      scenario.recording_time_origin_s, imu_sha, uwb_sha, scenario.imu.size(),
      obs_count, scenario.uwb.size());
  const std::string id_hex = cache_id.substr(std::string("sha256:").size());
  fs::create_directories(cache_candidate);
  fs::create_directories(truth_candidate);
  if (!fs::is_directory(cache_candidate) || !fs::is_directory(truth_candidate))
    throw std::runtime_error("T07 output root is not a directory");
  const fs::path cache_root_abs = fs::canonical(cache_candidate);
  const fs::path truth_root_abs = fs::canonical(truth_candidate);
  RequireDisjointPhysicalPaths(cache_root_abs, truth_root_abs);
  const fs::path cache_dir = cache_root_abs / id_hex;
  const fs::path truth_dir = truth_root_abs / id_hex;
  RequireDisjointPhysicalPaths(cache_dir, truth_dir);
  if (PathEntryExists(cache_dir) || PathEntryExists(truth_dir))
    throw std::runtime_error("T07 output cache/truth directory already exists");

  std::ostringstream manifest;
  manifest << std::setprecision(17)
           << "{\n  \"schema\": \"" << kT07EstimatorCacheSchema
           << "\",\n  \"cache_id\": \"" << cache_id
           << "\",\n  \"base_recording_id\": \"" << base_recording_id
           << "\",\n  \"base_source_sha256\": \"" << base_source_sha256
           << "\",\n  \"time_basis\": \"sensor_time_from_recording_origin\",\n"
           << "  \"recording_time_origin_s\": "
           << scenario.recording_time_origin_s << ",\n"
           << "  \"imu_units\": \"acc_mps2,gyro_radps,orientation_quaternion_wxyz\",\n"
           << "  \"uwb_units\": \"time_s,range_m,rssi_dbm\",\n"
           << "  \"uwb_message_grouping\": \"source_message_index\",\n"
           << "  \"imu_file\": \"imu.csv\",\n  \"imu_sha256\": \""
           << imu_sha << "\",\n  \"imu_count\": " << scenario.imu.size()
           << ",\n  \"uwb_file\": \"uwb_observations.csv\",\n"
           << "  \"uwb_sha256\": \"" << uwb_sha
           << "\",\n  \"uwb_observation_count\": " << obs_count
           << ",\n  \"uwb_message_count\": " << scenario.uwb.size()
           << "\n}\n";
  const std::string truth_sha = "sha256:" + Sha256Hex(truth.str());
  const std::string audit_sha = "sha256:" + Sha256FileHex(base_audit_path);
  std::ostringstream truth_manifest;
  truth_manifest << std::setprecision(17)
                 << "{\n  \"schema\": \"t07_injected_component_truth_v1\",\n"
                 << "  \"truth_semantics\": \"INJECTED_COMPONENT_ONLY\",\n"
                 << "  \"total_latent_bias_status\": \"UNKNOWN\",\n"
                 << "  \"cache_id\": \"" << cache_id << "\",\n"
                 << "  \"scenario_id\": \"" << recipe.scenario_id << "\",\n"
                 << "  \"scenario_seed\": " << recipe.scenario_seed << ",\n"
                 << "  \"seed_effect\": \"CONTEXT_ONLY_NO_STOCHASTIC_INJECTOR_COMPONENTS\",\n"
                 << "  \"recipe_sha256\": \"" << recipe.canonical_sha256 << "\",\n"
                 << "  \"base_component_audit_sha256\": \"" << audit_sha << "\",\n"
                 << "  \"truth_file\": \"injected_component_truth.csv\",\n"
                 << "  \"truth_sha256\": \"" << truth_sha << "\",\n"
                 << "  \"observation_count\": " << scenario.truth.size() << ",\n"
                 << "  \"event_match_counts\": [";
  for (size_t i = 0; i < scenario.event_match_counts.size(); ++i) {
    if (i) truth_manifest << ',';
    truth_manifest << scenario.event_match_counts[i];
  }
  truth_manifest << "]\n}\n";

  const fs::path cache_stage =
      UniqueStagingPath(cache_root_abs, "cache", id_hex);
  const fs::path truth_stage =
      UniqueStagingPath(truth_root_abs, "truth", id_hex);
  bool cache_stage_owned = false;
  bool truth_stage_owned = false;
  bool cache_committed = false;
  bool truth_committed = false;
  try {
    if (!fs::create_directory(cache_stage))
      throw std::runtime_error("cannot create T07 cache staging directory");
    cache_stage_owned = true;
    if (!fs::create_directory(truth_stage))
      throw std::runtime_error("cannot create T07 truth staging directory");
    truth_stage_owned = true;
    WriteFile(cache_stage / "imu.csv", imu.str());
    WriteFile(cache_stage / "uwb_observations.csv", uwb.str());
    WriteFile(cache_stage / "input_manifest.json", manifest.str());
    WriteFile(truth_stage / "injected_component_truth.csv", truth.str());
    WriteFile(truth_stage / "truth_manifest.json", truth_manifest.str());
    RequireExactFiles(cache_stage,
                      {"input_manifest.json", "imu.csv",
                       "uwb_observations.csv"});
    RequireExactFiles(truth_stage,
                      {"injected_component_truth.csv", "truth_manifest.json"});
    if (PathEntryExists(cache_dir) || PathEntryExists(truth_dir))
      throw std::runtime_error("T07 final output appeared during staging");
    fs::rename(cache_stage, cache_dir);
    cache_stage_owned = false;
    cache_committed = true;
    fs::rename(truth_stage, truth_dir);
    truth_stage_owned = false;
    truth_committed = true;
    RequireExactFiles(cache_dir,
                      {"input_manifest.json", "imu.csv",
                       "uwb_observations.csv"});
    RequireExactFiles(truth_dir,
                      {"injected_component_truth.csv", "truth_manifest.json"});
  } catch (...) {
    if (cache_stage_owned && PathEntryExists(cache_stage))
      fs::remove_all(cache_stage);
    if (truth_stage_owned && PathEntryExists(truth_stage))
      fs::remove_all(truth_stage);
    if (cache_committed && PathEntryExists(cache_dir)) fs::remove_all(cache_dir);
    if (truth_committed && PathEntryExists(truth_dir)) fs::remove_all(truth_dir);
    throw;
  }
  if (cache_id_out) *cache_id_out = cache_id;
  if (cache_manifest_out)
    *cache_manifest_out = (cache_dir / "input_manifest.json").string();
}

}  // namespace uifgo
