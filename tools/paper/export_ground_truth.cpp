// Evaluation-only GT normalizer. This binary is intentionally separate from
// uwb_imu_fgo_paper_runner, and its output directory must not be an estimator
// run/cache directory.
#include <boost/filesystem.hpp>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "uifgo/config.h"
#include "uifgo/data_loader.h"
#include "uifgo/hash_utils.h"
#include "uifgo/trajectory_io.h"

namespace fs = boost::filesystem;

namespace {

std::string Escape(const std::string& value) {
  std::string output;
  const char hex[] = "0123456789abcdef";
  for (unsigned char c : value) {
    switch (c) {
      case '"': output += "\\\""; break;
      case '\\': output += "\\\\"; break;
      case '\n': output += "\\n"; break;
      case '\r': output += "\\r"; break;
      case '\t': output += "\\t"; break;
      default:
        if (c < 0x20) {
          output += "\\u00";
          output.push_back(hex[c >> 4]);
          output.push_back(hex[c & 15]);
        } else {
          output.push_back(static_cast<char>(c));
        }
    }
  }
  return output;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string config_path, output_directory;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (i + 1 >= argc) throw std::invalid_argument("argument needs value");
      if (arg == "--config") config_path = argv[++i];
      else if (arg == "--output-directory") output_directory = argv[++i];
      else throw std::invalid_argument("unknown argument: " + arg);
    }
    if (config_path.empty() || output_directory.empty())
      throw std::invalid_argument(
          "usage: export_ground_truth --config FILE --output-directory DIR");
    const fs::path config = fs::canonical(fs::absolute(config_path));
    const fs::path output = fs::absolute(output_directory);
    if (fs::exists(output))
      throw std::invalid_argument("truth output directory already exists");
    fs::create_directories(output);
    const auto cfg = uifgo::ConfigLoader::Load(config.string());
    const std::string base = config.parent_path().string();
    uifgo::DataLoader loader(cfg);
    std::vector<uifgo::NavState> truth;
    std::string source;
    if (cfg.data_interface == "miluv") {
      const std::string root = uifgo::ConfigLoader::ResolveBagPath(
          base, cfg.miluv_data_dir);
      source = root + "/" + cfg.miluv_robot_dir + "/" + cfg.miluv_gt_csv;
      truth = loader.LoadGroundTruthMiluv(source);
    } else if (cfg.data_interface == "viunet") {
      const std::string root = uifgo::ConfigLoader::ResolveBagPath(
          base, cfg.viunet_data_dir);
      source = root + "/" + cfg.viunet_gt_csv;
      truth = loader.LoadGroundTruthViunet(source);
    } else if (cfg.data_interface == "sfuise") {
      const std::string root = uifgo::ConfigLoader::ResolveBagPath(
          base, cfg.sfuise_data_dir);
      source = root + "/ISAS-Walk" + std::to_string(cfg.sfuise_sequence) +
               ".bag";
      truth = loader.LoadGroundTruthSfuise(source);
    } else if (cfg.data_interface == "mcd") {
      source = uifgo::ConfigLoader::ResolveBagPath(base, cfg.gt_csv_path);
      truth = loader.LoadGroundTruthCsv(source);
    } else if (cfg.data_interface == "original" ||
               cfg.data_interface == "viral") {
      source = uifgo::ConfigLoader::ResolveBagPath(base, cfg.bag_path);
      truth = loader.LoadGroundTruth(source);
      if (truth.empty()) truth = loader.LoadGroundTruthOdom(source);
    } else {
      throw std::invalid_argument(
          "GT export is not defined for data interface " + cfg.data_interface);
    }
    if (truth.empty()) throw std::runtime_error("ground truth loader returned no poses");
    const fs::path trajectory = output / "ground_truth.tum";
    uifgo::TrajectoryIO::WriteTum(trajectory.string(), truth);
    std::ofstream manifest((output / "truth_export_manifest.json").string());
    manifest << "{\n  \"schema\": \"uifgo_t09_truth_export_v1\",\n"
             << "  \"source\": \"" << Escape(source) << "\",\n"
             << "  \"source_sha256\": \"sha256:"
             << uifgo::Sha256FileHex(source) << "\",\n"
             << "  \"config_sha256\": \"sha256:"
             << uifgo::Sha256FileHex(config.string()) << "\",\n"
             << "  \"pose_count\": " << truth.size() << ",\n"
             << "  \"trajectory_sha256\": \"sha256:"
             << uifgo::Sha256FileHex(trajectory.string()) << "\",\n"
             << "  \"estimator_input\": false\n}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "export_ground_truth ERROR: " << error.what() << '\n';
    return 2;
  }
}
