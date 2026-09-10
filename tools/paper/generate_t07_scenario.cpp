#include "uifgo/config.h"
#include "uifgo/data_loader.h"
#include "uifgo/hash_utils.h"
#include "uifgo/t07_scenario_injector.h"

#include <boost/filesystem.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::uint64_t Fnv1aFile(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot hash base source");
  std::uint64_t hash = 1469598103934665603ULL;
  char buffer[1 << 16];
  while (input) {
    input.read(buffer, sizeof(buffer));
    const auto count = input.gcount();
    for (std::streamsize i = 0; i < count; ++i) {
      hash ^= static_cast<unsigned char>(buffer[i]);
      hash *= 1099511628211ULL;
    }
  }
  return hash;
}

std::string Hex(std::uint64_t value) {
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << value;
  return out.str();
}

struct Args {
  std::string base_config, recipe, base_audit, cache_root, truth_root;
};

Args ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; i += 2) {
    if (i + 1 >= argc) throw std::invalid_argument("missing CLI value");
    const std::string key = argv[i], value = argv[i + 1];
    if (key == "--base-config") args.base_config = value;
    else if (key == "--recipe") args.recipe = value;
    else if (key == "--base-audit") args.base_audit = value;
    else if (key == "--cache-root") args.cache_root = value;
    else if (key == "--truth-root") args.truth_root = value;
    else throw std::invalid_argument("unknown CLI option: " + key);
  }
  if (args.base_config.empty() || args.recipe.empty() ||
      args.base_audit.empty() || args.cache_root.empty() ||
      args.truth_root.empty())
    throw std::invalid_argument("incomplete T07 generator arguments");
  return args;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    namespace fs = boost::filesystem;
    const Args args = ParseArgs(argc, argv);
    uifgo::Config cfg = uifgo::ConfigLoader::Load(args.base_config);
    if (cfg.data_interface != "original" || cfg.bag_start != 0.0 ||
        cfg.bag_durr != -1.0)
      throw std::runtime_error(
          "T07 generator requires full original base (start=0,durr=-1)");
    const fs::path config_dir = fs::absolute(args.base_config).parent_path();
    const std::string bag = fs::canonical(fs::absolute(
        uifgo::ConfigLoader::ResolveBagPath(config_dir.string(), cfg.bag_path)))
                                .string();
    const std::string source_sha = "sha256:" + uifgo::Sha256FileHex(bag);
    const std::string recording_id =
        "content-fnv1a64:" + Hex(Fnv1aFile(bag));
    std::vector<uifgo::ImuSample> imu;
    std::vector<uifgo::UwbFrame> uwb;
    if (!uifgo::DataLoader(cfg).LoadFromBag(bag, &imu, &uwb))
      throw std::runtime_error("T07 base loader failed");
    double origin = std::numeric_limits<double>::infinity();
    for (const auto& sample : imu) origin = std::min(origin, sample.t);
    for (const auto& frame : uwb)
      for (const auto& range : frame.ranges)
        origin = std::min(origin, std::isfinite(range.source_time)
                                     ? range.source_time
                                     : frame.t);
    if (!std::isfinite(origin))
      throw std::runtime_error("T07 base has no finite sensor-time origin");
    const auto recipe = uifgo::LoadT07ScenarioRecipe(args.recipe);
    const auto generated = uifgo::InjectT07Scenario(
        imu, uwb, recording_id, recipe, origin, true);
    std::string cache_id, manifest;
    uifgo::WriteT07ScenarioCache(
        generated, recipe, recording_id, source_sha, args.base_audit,
        args.cache_root, args.truth_root, &cache_id, &manifest);
    std::cout << "cache_id=" << cache_id << '\n'
              << "cache_manifest=" << manifest << '\n'
              << "recording_time_origin_s=" << std::setprecision(17) << origin
              << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "T07 scenario generation failed: " << e.what() << '\n';
    return 1;
  }
}
