#include "uifgo/config.h"
#include "uifgo/data_loader.h"
#include "uifgo/hash_utils.h"
#include "uifgo/t07_scenario_cache.h"
#include "uifgo/paper_input.h"
#include <yaml-cpp/yaml.h>

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

}  // namespace

int main(int argc, char** argv) {
 try {
  namespace fs = boost::filesystem;
  if (argc != 3) throw std::runtime_error("usage: export_sfuise_input CONFIG OUTPUT_DIRECTORY");
  auto cfg = uifgo::ConfigLoader::Load(argv[1]);
  if(cfg.data_interface != "sfuise" || cfg.bag_start != 0 || cfg.bag_durr != -1 || cfg.sfuise_uwb_group_window != 0)
    throw std::runtime_error("full ungrouped SFUISE required");
  fs::path dir(argv[2]);
  if(fs::exists(dir)) throw std::runtime_error("output exists");
  fs::create_directories(dir);
  const auto data = uifgo::ConfigLoader::ResolveBagPath(fs::absolute(argv[1]).parent_path().string(),cfg.sfuise_data_dir);
  const auto bag = data + "/ISAS-Walk" + std::to_string(cfg.sfuise_sequence) + ".bag";
  const auto recording = "content-fnv1a64:" + Hex(Fnv1aFile(bag));
  const auto source = "sha256:" + uifgo::Sha256FileHex(bag);
  std::vector<uifgo::ImuSample> samples; std::vector<uifgo::UwbFrame> frames;
  std::vector<uifgo::AnchorConfig> anchors;
  if(!uifgo::DataLoader(cfg).LoadSfuiseBag(data,cfg.sfuise_sequence,&samples,&frames,&anchors,true))
    throw std::runtime_error("SFUISE loader failed");
  double origin=std::min(samples.front().t,frames.front().t);
  for(size_t f=0;f<frames.size();++f) for(size_t r=0;r<frames[f].ranges.size();++r)
    frames[f].ranges[r].obs_id=uifgo::StableObservationId(recording,f,r,frames[f],frames[f].ranges[r]);
  auto HexEncode=[](const std::string& value) { std::ostringstream o; for(unsigned char c:value) o<<std::hex<<std::setw(2)<<std::setfill('0')<<int(c);return o.str();};
  std::ostringstream imu;
  imu << "source_index,sensor_time_s,acc_x_mps2,acc_y_mps2,acc_z_mps2,"
         "gyro_x_radps,gyro_y_radps,gyro_z_radps,has_orientation,qw,qx,qy,qz\n"
      << std::setprecision(17);
  for (size_t i = 0; i < samples.size(); ++i) {
    const auto& s = samples[i];
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
  for (const auto& frame : frames) {
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

  std::ofstream((dir/"imu.csv").string())<<imu.str();
  std::ofstream((dir/"uwb_observations.csv").string())<<uwb.str();
  const auto ish="sha256:"+uifgo::Sha256Hex(imu.str()), ush="sha256:"+uifgo::Sha256Hex(uwb.str());
  auto exact=[](double x){std::ostringstream o;o<<std::setprecision(17)<<x;return o.str();};
  YAML::Node m;
  m["schema"]=uifgo::kT07EstimatorCacheSchema;
  m["cache_id"]=uifgo::T07CacheCanonicalId(recording,source,origin,ish,ush,samples.size(),obs_count,frames.size());
  m["base_recording_id"]=recording; m["base_source_sha256"]=source;
  m["time_basis"]="sensor_time_from_recording_origin"; m["recording_time_origin_s"]=exact(origin);
  m["imu_units"]="acc_mps2,gyro_radps,orientation_quaternion_wxyz";
  m["uwb_units"]="time_s,range_m,rssi_dbm";m["uwb_message_grouping"]="source_message_index";
  m["imu_file"]="imu.csv";m["imu_sha256"]=ish;m["imu_count"]=samples.size();
  m["uwb_file"]="uwb_observations.csv";m["uwb_sha256"]=ush;m["uwb_observation_count"]=obs_count;m["uwb_message_count"]=frames.size();
  YAML::Emitter out;out.SetDoublePrecision(17);out<<m;
  std::ofstream((dir/"input_manifest.yaml").string())<<out.c_str();
  YAML::Node ac;
  for(const auto& anchor:anchors) { YAML::Node v;v["id"]=anchor.id;for(int i=0;i<3;++i)v["pos"].push_back(exact(anchor.pos[i]));v["prior_sigma"]=exact(anchor.prior_sigma);ac.push_back(v); }
  YAML::Emitter ao;ao.SetDoublePrecision(17);ao<<ac;
  std::ofstream((dir/"anchors.yaml").string())<<ao.c_str();
  uifgo::LoadT07ScenarioCache((dir/"input_manifest.yaml").string(),0,-1);
  return 0;
 } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
