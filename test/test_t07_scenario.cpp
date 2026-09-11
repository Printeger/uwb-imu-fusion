#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "uifgo/hash_utils.h"
#include "uifgo/t07_scenario_cache.h"
#include "uifgo/t07_scenario_injector.h"

namespace {

constexpr double kOrigin = 100.0;
const std::string kRecordingId = "content-fnv1a64:testrecording";

uifgo::ImuSample Imu(double t) {
  uifgo::ImuSample sample;
  sample.t = t;
  sample.acc = {1.25, -2.5, 9.75};
  sample.gyro = {0.01, 0.02, -0.03};
  return sample;
}

std::vector<uifgo::UwbFrame> Frames(size_t count) {
  std::vector<uifgo::UwbFrame> frames;
  std::uint64_t observation = 40;
  for (size_t i = 0; i < count; ++i) {
    uifgo::UwbFrame frame;
    frame.t = kOrigin + static_cast<double>(i);
    frame.tag_id = 1;
    for (size_t range_index = 0; range_index < 2; ++range_index) {
      uifgo::UwbRange range;
      range.anchor_id = static_cast<int>(range_index + 1);
      range.dist = 5.0 + i + range_index;
      range.fp_rssi = -80.0;
      range.rx_rssi = -77.0;
      range.source_obs_index = observation++;
      range.source_message_index = 20 + i;
      range.source_range_index = range_index;
      range.source_time = frame.t;
      range.source_tag_id = 1;
      frame.ranges.push_back(range);
    }
    frames.push_back(frame);
  }
  return frames;
}

uifgo::T07ScenarioRecipe Recipe(size_t expected = 2) {
  uifgo::T07ScenarioRecipe recipe;
  recipe.scenario_id = "unit_scenario";
  recipe.scenario_seed = 7001;
  recipe.canonical_sha256 = "sha256:unit_recipe";
  uifgo::T07InjectionEvent step;
  step.event_id = "step_1_1";
  step.link = "1:1";
  step.start_s = 1.0;
  step.end_s = 3.0;
  step.shape = uifgo::T07InjectionShape::kStep;
  step.amplitude_m = 0.4;
  step.has_expected_match_count = true;
  step.expected_match_count = expected;
  uifgo::T07InjectionEvent ramp;
  ramp.event_id = "ramp_1_2";
  ramp.link = "1:2";
  ramp.start_s = 1.0;
  ramp.end_s = 3.0;
  ramp.shape = uifgo::T07InjectionShape::kRamp;
  ramp.start_amplitude_m = 0.2;
  ramp.slope_m_per_s = 0.1;
  ramp.has_expected_match_count = true;
  ramp.expected_match_count = expected;
  recipe.events = {step, ramp};
  return recipe;
}

std::map<std::uint64_t, double> ObservedById(
    const uifgo::T07InjectedScenario& scenario) {
  std::map<std::uint64_t, double> values;
  for (const auto& row : scenario.truth) values[row.obs_id] = row.observed_range_m;
  return values;
}

void WriteAudit(const boost::filesystem::path& path) {
  std::ofstream out(path.string());
  out << "schema: t07_base_component_audit_v1\n"
         "fixture_scope: FROZEN_DEVELOPMENT_FIXTURE_ONLY\n"
         "base_source_sha256: 'sha256:base'\n"
         "base_recording_id: 'content-fnv1a64:testrecording'\n"
         "recording_time_origin_s: 100.0\n"
         "truth_semantics: INJECTED_COMPONENT_ONLY\n"
         "total_latent_bias_status: UNKNOWN\n";
}

std::string ReadFile(const boost::filesystem::path& path) {
  std::ifstream input(path.string(), std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(input)), {});
}

void WriteText(const boost::filesystem::path& path, const std::string& text) {
  std::ofstream output(path.string(), std::ios::binary | std::ios::trunc);
  output << text;
}

void ReplaceOnce(std::string* text, const std::string& before,
                 const std::string& after) {
  const auto position = text->find(before);
  ASSERT_NE(position, std::string::npos);
  text->replace(position, before.size(), after);
}

void RehashCacheManifest(const boost::filesystem::path& manifest_path) {
  namespace fs = boost::filesystem;
  const YAML::Node old = YAML::LoadFile(manifest_path.string());
  const fs::path dir = manifest_path.parent_path();
  const std::string imu_sha =
      "sha256:" + uifgo::Sha256FileHex((dir / "imu.csv").string());
  const std::string uwb_sha = "sha256:" +
      uifgo::Sha256FileHex((dir / "uwb_observations.csv").string());
  const std::string cache_id = uifgo::T07CacheCanonicalId(
      old["base_recording_id"].as<std::string>(),
      old["base_source_sha256"].as<std::string>(),
      old["recording_time_origin_s"].as<double>(), imu_sha, uwb_sha,
      old["imu_count"].as<size_t>(),
      old["uwb_observation_count"].as<size_t>(),
      old["uwb_message_count"].as<size_t>());
  std::string text = ReadFile(manifest_path);
  ReplaceOnce(&text, old["imu_sha256"].as<std::string>(), imu_sha);
  ReplaceOnce(&text, old["uwb_sha256"].as<std::string>(), uwb_sha);
  ReplaceOnce(&text, old["cache_id"].as<std::string>(), cache_id);
  WriteText(manifest_path, text);
}

std::set<std::string> DirectoryEntries(const boost::filesystem::path& dir) {
  std::set<std::string> result;
  for (boost::filesystem::directory_iterator it(dir), end; it != end; ++it)
    result.insert(it->path().filename().string());
  return result;
}

TEST(T07Scenario, ConservesBasePlusInjectedComponentAndUsesHalfOpenWindows) {
  const std::vector<uifgo::ImuSample> imu = {Imu(100), Imu(101), Imu(102), Imu(103)};
  const auto result = uifgo::InjectT07Scenario(
      imu, Frames(4), kRecordingId, Recipe(), kOrigin, true);
  ASSERT_EQ(result.event_match_counts, (std::vector<size_t>{2, 2}));
  ASSERT_EQ(result.truth.size(), 8u);
  for (const auto& row : result.truth) {
    EXPECT_DOUBLE_EQ(row.observed_range_m,
                     row.base_range_m + row.injected_bias_delta_m);
    const double relative = row.sensor_time_s - kOrigin;
    if (relative == 1.0 || relative == 2.0) {
      const double expected = row.anchor_id == 1 ? 0.4 : 0.2 + 0.1 * (relative - 1.0);
      EXPECT_NEAR(row.injected_bias_delta_m, expected, 1e-15);
    } else {
      EXPECT_DOUBLE_EQ(row.injected_bias_delta_m, 0.0);
    }
  }
}

TEST(T07Scenario, PrefixAllowsZeroMatchesAndCommonHistoryIsIdentical) {
  const std::vector<uifgo::ImuSample> imu = {Imu(100), Imu(101), Imu(102), Imu(103)};
  const auto full = uifgo::InjectT07Scenario(
      imu, Frames(4), kRecordingId, Recipe(), kOrigin, true);
  const auto zero_prefix = uifgo::InjectT07Scenario(
      {imu.front()}, Frames(1), kRecordingId, Recipe(), kOrigin, false);
  EXPECT_EQ(zero_prefix.event_match_counts, (std::vector<size_t>{0, 0}));
  const auto prefix = uifgo::InjectT07Scenario(
      {imu[0], imu[1], imu[2]}, Frames(3), kRecordingId, Recipe(), kOrigin,
      false);
  const auto full_values = ObservedById(full);
  for (const auto& item : ObservedById(prefix))
    EXPECT_DOUBLE_EQ(item.second, full_values.at(item.first));
}

TEST(T07Scenario, FullBaseRejectsZeroOrChangedFrozenCounts) {
  const std::vector<uifgo::ImuSample> imu = {Imu(100), Imu(101), Imu(102), Imu(103)};
  EXPECT_THROW(uifgo::InjectT07Scenario(
                   imu, Frames(1), kRecordingId, Recipe(), kOrigin, true),
               std::runtime_error);
  EXPECT_THROW(uifgo::InjectT07Scenario(
                   imu, Frames(4), kRecordingId, Recipe(3), kOrigin, true),
               std::runtime_error);
}

TEST(T07Scenario, CacheRoundTripLocksIdentityUnitsAndNeedsNoRecipe) {
  namespace fs = boost::filesystem;
  const fs::path root = fs::temp_directory_path() /
      ("uifgo_t07_" + std::to_string(getpid()));
  fs::remove_all(root);
  fs::create_directories(root);
  const fs::path audit = root / "audit.yaml";
  WriteAudit(audit);
  const std::vector<uifgo::ImuSample> imu = {Imu(100), Imu(101), Imu(102), Imu(103)};
  const auto scenario = uifgo::InjectT07Scenario(
      imu, Frames(4), kRecordingId, Recipe(), kOrigin, true);
  std::string id_a, manifest_a;
  uifgo::WriteT07ScenarioCache(
      scenario, Recipe(), kRecordingId, "sha256:base", audit.string(),
      (root / "cache_a").string(), (root / "truth_a").string(), &id_a,
      &manifest_a);
  auto unrelated_recipe = Recipe();
  unrelated_recipe.scenario_id = "different_recipe_metadata";
  unrelated_recipe.scenario_seed = 9999;
  unrelated_recipe.canonical_sha256 = "sha256:different";
  std::string id_b, manifest_b;
  uifgo::WriteT07ScenarioCache(
      scenario, unrelated_recipe, kRecordingId, "sha256:base", audit.string(),
      (root / "cache_b").string(), (root / "truth_b").string(), &id_b,
      &manifest_b);
  EXPECT_EQ(id_a, id_b);
  EXPECT_EQ(DirectoryEntries(fs::path(manifest_a).parent_path()),
            (std::set<std::string>{"imu.csv", "input_manifest.json",
                                   "uwb_observations.csv"}));
  EXPECT_EQ(DirectoryEntries(fs::path(manifest_a).parent_path().parent_path()
                                 .parent_path() / "truth_a" /
                             id_a.substr(7)),
            (std::set<std::string>{"injected_component_truth.csv",
                                   "truth_manifest.json"}));

  const auto cache = uifgo::LoadT07ScenarioCache(manifest_a, 0.0, -1.0);
  ASSERT_EQ(cache.imu.size(), imu.size());
  EXPECT_DOUBLE_EQ(cache.imu.front().acc.x(), 1.25);
  EXPECT_DOUBLE_EQ(cache.imu.front().gyro.z(), -0.03);
  EXPECT_EQ(cache.base_recording_id, kRecordingId);
  ASSERT_EQ(cache.uwb.size(), 4u);
  EXPECT_EQ(cache.uwb[2].ranges[1].source_message_index, 22u);
  EXPECT_EQ(cache.uwb[2].ranges[1].source_range_index, 1u);
  EXPECT_EQ(cache.uwb[2].ranges[1].obs_id, scenario.uwb[2].ranges[1].obs_id);

  const auto cache_prefix =
      uifgo::LoadT07ScenarioCache(manifest_a, 0.0, 1.0);
  ASSERT_EQ(cache_prefix.uwb.size(), 2u);
  for (size_t frame_index = 0; frame_index < cache_prefix.uwb.size();
       ++frame_index) {
    ASSERT_EQ(cache_prefix.uwb[frame_index].ranges.size(),
              cache.uwb[frame_index].ranges.size());
    for (size_t range_index = 0;
         range_index < cache_prefix.uwb[frame_index].ranges.size();
         ++range_index) {
      const auto& prefix_range =
          cache_prefix.uwb[frame_index].ranges[range_index];
      const auto& full_range = cache.uwb[frame_index].ranges[range_index];
      EXPECT_EQ(prefix_range.obs_id, full_range.obs_id);
      EXPECT_EQ(prefix_range.source_message_index,
                full_range.source_message_index);
      EXPECT_EQ(prefix_range.source_range_index, full_range.source_range_index);
      EXPECT_DOUBLE_EQ(prefix_range.dist, full_range.dist);
    }
  }

  std::ifstream input(manifest_a);
  const std::string manifest((std::istreambuf_iterator<char>(input)), {});
  EXPECT_EQ(manifest.find("recipe"), std::string::npos);
  EXPECT_EQ(manifest.find("truth"), std::string::npos);
  EXPECT_EQ(manifest.find("support"), std::string::npos);

  {
    std::ofstream payload(fs::path(manifest_b).parent_path().string() +
                              "/uwb_observations.csv",
                          std::ios::app);
    payload << "tamper\n";
  }
  EXPECT_THROW(uifgo::LoadT07ScenarioCache(manifest_b, 0.0, -1.0),
               std::runtime_error);
  {
    std::string forbidden = manifest;
    const auto close = forbidden.rfind("\n}");
    ASSERT_NE(close, std::string::npos);
    forbidden.replace(close, 2, ",\n  \"truth_file\": \"forbidden.csv\"\n}");
    std::ofstream output(manifest_a, std::ios::trunc);
    output << forbidden;
  }
  EXPECT_THROW(uifgo::LoadT07ScenarioCache(manifest_a, 0.0, -1.0),
               std::runtime_error);
  fs::remove_all(root);
}

TEST(T07Scenario, OutputRootsRejectPhysicalAliasesAndOverlap) {
  namespace fs = boost::filesystem;
  const fs::path root = fs::temp_directory_path() /
      ("uifgo_t07_paths_" + std::to_string(getpid()));
  fs::remove_all(root);
  fs::create_directories(root);
  const fs::path audit = root / "audit.yaml";
  WriteAudit(audit);
  const auto scenario = uifgo::InjectT07Scenario(
      {Imu(100), Imu(101), Imu(102), Imu(103)}, Frames(4), kRecordingId,
      Recipe(), kOrigin, true);
  std::string id, manifest;

  EXPECT_THROW(uifgo::WriteT07ScenarioCache(
                   scenario, Recipe(), kRecordingId, "sha256:base",
                   audit.string(), (root / "same").string(),
                   (root / "same").string(), &id, &manifest),
               std::runtime_error);

  const fs::path physical = root / "physical";
  fs::create_directories(physical);
  fs::create_directory_symlink(physical, root / "physical_alias");
  EXPECT_THROW(uifgo::WriteT07ScenarioCache(
                   scenario, Recipe(), kRecordingId, "sha256:base",
                   audit.string(), physical.string(),
                   (root / "physical_alias").string(), &id, &manifest),
               std::runtime_error);

  EXPECT_THROW(uifgo::WriteT07ScenarioCache(
                   scenario, Recipe(), kRecordingId, "sha256:base",
                   audit.string(), (root / "overlap").string(),
                   (root / "overlap" / "truth").string(), &id, &manifest),
               std::runtime_error);
  EXPECT_FALSE(fs::exists(root / "same"));
  EXPECT_FALSE(fs::exists(root / "overlap"));
  fs::remove_all(root);
}

TEST(T07Scenario, OutputFailureRemovesOwnedStagingAndLeavesNoCache) {
  namespace fs = boost::filesystem;
  const fs::path root = fs::temp_directory_path() /
      ("uifgo_t07_cleanup_" + std::to_string(getpid()));
  fs::remove_all(root);
  fs::create_directories(root);
  const fs::path audit = root / "audit.yaml";
  const fs::path cache_root = root / "cache";
  const fs::path truth_root = root / "truth";
  WriteAudit(audit);
  fs::create_directories(cache_root);
  fs::create_directories(truth_root);
  ASSERT_EQ(chmod(truth_root.string().c_str(), 0500), 0);
  const auto scenario = uifgo::InjectT07Scenario(
      {Imu(100), Imu(101), Imu(102), Imu(103)}, Frames(4), kRecordingId,
      Recipe(), kOrigin, true);
  std::string id, manifest;
  EXPECT_THROW(uifgo::WriteT07ScenarioCache(
                   scenario, Recipe(), kRecordingId, "sha256:base",
                   audit.string(), cache_root.string(), truth_root.string(),
                   &id, &manifest),
               boost::filesystem::filesystem_error);
  ASSERT_EQ(chmod(truth_root.string().c_str(), 0700), 0);
  EXPECT_TRUE(fs::is_empty(cache_root));
  EXPECT_TRUE(fs::is_empty(truth_root));
  fs::remove_all(root);
}

TEST(T07Scenario, SemanticRehashCannotHideNoncanonicalPayloadOrder) {
  namespace fs = boost::filesystem;
  const fs::path root = fs::temp_directory_path() /
      ("uifgo_t07_order_" + std::to_string(getpid()));
  fs::remove_all(root);
  fs::create_directories(root);
  const fs::path audit = root / "audit.yaml";
  WriteAudit(audit);
  const auto scenario = uifgo::InjectT07Scenario(
      {Imu(100), Imu(101), Imu(102), Imu(103)}, Frames(4), kRecordingId,
      Recipe(), kOrigin, true);

  auto writer_bad_uwb = scenario;
  std::swap(writer_bad_uwb.uwb[0], writer_bad_uwb.uwb[1]);
  std::string rejected_id, rejected_manifest;
  EXPECT_THROW(uifgo::WriteT07ScenarioCache(
                   writer_bad_uwb, Recipe(), kRecordingId, "sha256:base",
                   audit.string(), (root / "writer_bad_uwb_cache").string(),
                   (root / "writer_bad_uwb_truth").string(), &rejected_id,
                   &rejected_manifest),
               std::runtime_error);
  auto writer_bad_imu = scenario;
  std::swap(writer_bad_imu.imu[0], writer_bad_imu.imu[1]);
  EXPECT_THROW(uifgo::WriteT07ScenarioCache(
                   writer_bad_imu, Recipe(), kRecordingId, "sha256:base",
                   audit.string(), (root / "writer_bad_imu_cache").string(),
                   (root / "writer_bad_imu_truth").string(), &rejected_id,
                   &rejected_manifest),
               std::runtime_error);

  std::string uwb_id, uwb_manifest;
  uifgo::WriteT07ScenarioCache(
      scenario, Recipe(), kRecordingId, "sha256:base", audit.string(),
      (root / "uwb_cache").string(), (root / "uwb_truth").string(),
      &uwb_id, &uwb_manifest);
  const fs::path uwb_path = fs::path(uwb_manifest).parent_path() /
                            "uwb_observations.csv";
  std::istringstream uwb_input(ReadFile(uwb_path));
  std::vector<std::string> uwb_lines;
  std::string line;
  while (std::getline(uwb_input, line)) uwb_lines.push_back(line);
  ASSERT_EQ(uwb_lines.size(), 9u);
  std::ostringstream swapped_uwb;
  swapped_uwb << uwb_lines[0] << '\n';
  for (size_t i : {3u, 4u, 1u, 2u, 5u, 6u, 7u, 8u})
    swapped_uwb << uwb_lines[i] << '\n';
  WriteText(uwb_path, swapped_uwb.str());
  RehashCacheManifest(uwb_manifest);
  EXPECT_THROW(uifgo::LoadT07ScenarioCache(uwb_manifest, 0.0, -1.0),
               std::runtime_error);

  std::string imu_id, imu_manifest;
  uifgo::WriteT07ScenarioCache(
      scenario, Recipe(), kRecordingId, "sha256:base", audit.string(),
      (root / "imu_cache").string(), (root / "imu_truth").string(),
      &imu_id, &imu_manifest);
  const fs::path imu_path = fs::path(imu_manifest).parent_path() / "imu.csv";
  std::istringstream imu_input(ReadFile(imu_path));
  std::vector<std::string> imu_lines;
  while (std::getline(imu_input, line)) imu_lines.push_back(line);
  ASSERT_EQ(imu_lines.size(), 5u);
  std::swap(imu_lines[1], imu_lines[2]);
  std::ostringstream swapped_imu;
  for (const auto& imu_line : imu_lines) swapped_imu << imu_line << '\n';
  WriteText(imu_path, swapped_imu.str());
  RehashCacheManifest(imu_manifest);
  EXPECT_THROW(uifgo::LoadT07ScenarioCache(imu_manifest, 0.0, -1.0),
               std::runtime_error);
  fs::remove_all(root);
}

TEST(T07Scenario, NlosV2OpaqueIdentityAndTruthRejection) {
 namespace fs = boost::filesystem;
 const auto root=fs::temp_directory_path()/fs::unique_path("nlos-v2-%%%%%%");fs::create_directories(root);
 WriteAudit(root/"audit.yaml");
 auto scenario=uifgo::InjectT07Scenario({Imu(100),Imu(101),Imu(102),Imu(103)},Frames(4),kRecordingId,Recipe(),kOrigin,true);
 std::string id,path;
 uifgo::WriteT07ScenarioCache(scenario,Recipe(),kRecordingId,"sha256:base",(root/"audit.yaml").string(),(root/"cache").string(),(root/"truth").string(),&id,&path);
 auto m=YAML::LoadFile(path);m["schema"]="nlos_measurement_cache_v2";
 const std::string transform="sha256:"+uifgo::Sha256Hex("spec/subset");m["transform_sha256"]=transform;
 m["cache_id"]="sha256:"+uifgo::Sha256Hex("nlos_measurement_cache_v2\n"+id+"\n"+transform+"\n");
 auto save=[&](){YAML::Emitter e;e.SetDoublePrecision(17);e<<m;WriteText(path,e.c_str());};save();
 fs::remove_all(root/"truth");fs::remove(root/"audit.yaml");
 const auto loaded=uifgo::LoadT07ScenarioCache(path,0,-1);
 ASSERT_EQ(loaded.uwb.size(),scenario.uwb.size());
 for(size_t i=0;i<loaded.uwb.size();++i)for(size_t j=0;j<loaded.uwb[i].ranges.size();++j){
 EXPECT_EQ(loaded.uwb[i].ranges[j].dist,scenario.uwb[i].ranges[j].dist);
 EXPECT_EQ(loaded.uwb[i].ranges[j].obs_id,scenario.uwb[i].ranges[j].obs_id);}
 m["transform_sha256"]="sha256:"+uifgo::Sha256Hex("other scenario");save();
 EXPECT_THROW(uifgo::LoadT07ScenarioCache(path,0,-1),std::runtime_error);
 m["transform_sha256"]=transform;m["truth_path"]="unavailable";save();
 EXPECT_THROW(uifgo::LoadT07ScenarioCache(path,0,-1),std::runtime_error);
 fs::remove_all(root);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
