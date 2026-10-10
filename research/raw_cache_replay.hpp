#pragma once
#include "scenario_material.hpp"
#include <fstream>
#include <algorithm>
#include <sstream>
namespace uwb_imu_pl::research {
inline std::vector<std::string> csvFields(const std::string& line){
 std::vector<std::string> out;std::stringstream stream(line);std::string field;
 while(std::getline(stream,field,','))out.push_back(field);
 return out;
}
// Read the existing t07 raw cache, not its refit, GT or probability outputs.
// Anchor/frame/lever and initialization are supplied by research config and
// remain UNQUALIFIED. Preserve actual source grouping and sensor timestamps.
inline void loadRawCache(NoisyMotionScenario& scene,const std::string& directory){
 std::ifstream im(directory+"/imu.csv"),uw(directory+"/uwb_observations.csv");
 if(!im || !uw)throw std::runtime_error("raw IMU/UWB cache unavailable");
 std::string line;std::getline(im,line);auto ih=csvFields(line);std::getline(uw,line);auto uh=csvFields(line);
 auto column=[](const auto& headers,const std::string& key){auto it=std::find(headers.begin(),headers.end(),key);if(it==headers.end())throw std::runtime_error("missing raw cache column "+key);return int(it-headers.begin());};
 int itime=column(ih,"sensor_time_s"),idx=column(ih,"source_index");
 const int acc[3]={column(ih,"acc_x_mps2"),column(ih,"acc_y_mps2"),column(ih,"acc_z_mps2")};
 const int gyro[3]={column(ih,"gyro_x_radps"),column(ih,"gyro_y_radps"),column(ih,"gyro_z_radps")};
 std::getline(im,line);auto first=csvFields(line);long double origin=std::stold(first.at(itime));
 auto timestamp=[&](const std::string& time){return TimestampNs(std::llround((std::stold(time)-origin)*1e9L));};
 const int utime=column(uh,"sensor_time_s"),message=column(uh,"source_message_index"),anchor=column(uh,"anchor_id"),range=column(uh,"observed_range_m"),id=column(uh,"obs_id"),valid=column(uh,"source_valid"),tag=column(uh,"tag_id");
 scene.uwb.clear();scene.imu.clear();long long previous_message=-1;
 while(std::getline(uw,line)){auto f=csvFields(line);if(std::stoi(f.at(valid))!=1)continue;
  if(std::stoi(f.at(tag))!=1)throw std::runtime_error("unsupported cache tag (only tag 1)");
  long long msg=std::stoll(f.at(message));
  if(msg!=previous_message){if(scene.uwb.size()==size_t(scene.epochs))break;
   UwbBatch b;b.id=BatchId(msg+1);b.timestamp=timestamp(f.at(utime));scene.uwb.push_back(b);previous_message=msg;}
  auto& b=scene.uwb.back();UwbMeasurement m;m.id=MeasurementId(std::stoull(f.at(id)));m.factor_id=FactorId(m.id.value());
  m.anchor_id=AnchorId(std::stoull(f.at(anchor)));m.timestamp=timestamp(f.at(utime));
  if(m.timestamp!=b.timestamp)throw std::runtime_error("asynchronous ranges require a different factor adapter");
  auto a=std::find_if(scene.cfg.anchors.begin(),scene.cfg.anchors.end(),[&](const auto& a){return a.id==m.anchor_id;});
  if(a==scene.cfg.anchors.end())throw std::runtime_error("unconfigured raw anchor");
  m.anchor_position_m=a->position_world_m;
  m.range_m=std::stod(f.at(range));m.sigma_m=scene.cfg.realtime.range_sigma_m;b.measurements.push_back(m);
 }
 if(scene.uwb.size()!=size_t(scene.epochs))throw std::runtime_error("short raw cache fragment");
 auto add=[&](const auto& f){ImuMeasurement m;m.id=MeasurementId(std::stoull(f.at(idx))+1);m.timestamp=timestamp(f.at(itime));
  for(int j=0;j<3;++j){m.specific_force_mps2(j)=std::stod(f.at(acc[j]));m.angular_velocity_radps(j)=std::stod(f.at(gyro[j]));}
  scene.imu.push_back(m);};add(first);
 while(std::getline(im,line)){auto f=csvFields(line);if(scene.uwb.back().timestamp<timestamp(f.at(itime)))break;add(f);}
 if(scene.imu.size()<2 || scene.imu.back().timestamp<scene.uwb.front().timestamp)throw std::runtime_error("insufficient raw IMU support");
}
}
