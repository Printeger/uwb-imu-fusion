#include "unified_rolling_core.hpp"
#include "measurement_bootstrap.hpp"
#include "numerical_candidate.hpp"
#include <tbb/global_control.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
namespace {
using Row=std::map<std::string,std::string>;
std::vector<std::string> split(const std::string& line){std::vector<std::string> out;std::stringstream stream(line);std::string cell;while(std::getline(stream,cell,',')){if(!cell.empty() && cell.back()=='\r')cell.pop_back();out.push_back(cell);}return out;}
std::vector<Row> csv(const std::string& path){std::ifstream input(path);std::string line;if(!std::getline(input,line))throw std::runtime_error("CSV_OPEN: "+path);auto header=split(line);std::vector<Row> out;
 while(std::getline(input,line)){if(line.empty())continue;auto values=split(line);if(values.size()!=header.size())throw std::runtime_error("CSV_ROW_WIDTH: "+path);Row row;for(size_t i=0;i<header.size();++i)row.emplace(header[i],values[i]);out.push_back(std::move(row));}return out;}
double number(const Row& row,const std::string& field){const double value=std::stod(row.at(field));if(!std::isfinite(value))throw std::runtime_error("CSV_NONFINITE: "+field);return value;}
uint64_t integer(const Row& row,const std::string& field){const auto& value=row.at(field);size_t used=0;if(value.empty() || value.front()=='-')throw std::runtime_error("CSV_UNSIGNED: "+field);const auto result=std::stoull(value,&used);if(used!=value.size())throw std::runtime_error("CSV_INTEGER: "+field);return result;}
TimestampNs stamp(double seconds){const long double ns=static_cast<long double>(seconds)*1000000000.L;if(!std::isfinite(seconds) || ns<INT64_MIN || ns>INT64_MAX)throw std::runtime_error("CSV_TIME_CAPACITY");return TimestampNs(std::llround(ns));}
Eigen::Vector3d vector(const YAML::Node& n){if(n.size()!=3)throw std::runtime_error("MODEL_VECTOR_SIZE");return {n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};}
struct Batch {UwbBatch values;std::vector<CanonicalRangeEvent> events;};
}
int main(int argc,char** argv){try{
 if(argc<5 || argc>7)throw std::invalid_argument("usage: sc9_dataset_runner MODEL_JSON SENSOR_DIRECTORY OUTPUT_CSV on|off [MAX_EPOCHS=0] [ENABLE_AFTER=0]");
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 const auto model=YAML::LoadFile(argv[1]);const std::string folder=argv[2],mode=argv[4];if(mode!="on" && mode!="off")throw std::invalid_argument("FDE_MODE");
 const size_t maximum=argc>5?std::stoull(argv[5]):0,enable_after=argc>6?std::stoull(argv[6]):0;
 if(model["status"].as<std::string>()!="DECLARED_EMPIRICAL")throw std::invalid_argument("DATASET_REQUIRES_DECLARED_EMPIRICAL_MODEL");
 auto law=researchLaw("config/sc7.yaml");auto cfg=IntegrityConfigLoader::load(law["imu_base_config"].as<std::string>());cfg.anchors.clear();
 const auto policy=resolveRollingPolicy(law,cfg);const Eigen::Vector3d lever(.1,-.025,0);
 JointRawLaw rawlaw;rawlaw.gravity=model["gravity_mps2"].as<double>();rawlaw.accel_adc=model["accel_adc_sigma"].as<double>();rawlaw.gyro_adc=model["gyro_adc_sigma"].as<double>();rawlaw.accel_rw=model["accel_rw_sigma"].as<double>();rawlaw.gyro_rw=model["gyro_rw_sigma"].as<double>();
 Sc6MotionDomain domain;domain.id=model["model_id"].as<std::string>();domain.acceleration=model["motion"]["acceleration"].as<double>();domain.rate=model["motion"]["rate"].as<double>();domain.jerk=model["motion"]["jerk"].as<double>();domain.alpha=model["motion"]["alpha"].as<double>();
 rawlaw.acceleration=domain.acceleration;rawlaw.rate=domain.rate;rawlaw.jerk=domain.jerk;
 cfg.imu.gravity_mps2=rawlaw.gravity;cfg.imu.accelerometer_sigma=rawlaw.accel_adc;cfg.imu.gyroscope_sigma=rawlaw.gyro_adc;
 cfg.imu.accelerometer_bias_rw_sigma=rawlaw.accel_rw;cfg.imu.gyroscope_bias_rw_sigma=rawlaw.gyro_rw;cfg.imu.max_gap_s=.05;
 cfg.bridge.generic.acceleration_bound_mps2=domain.acceleration;cfg.bridge.generic.angular_rate_bound_radps=domain.rate;cfg.bridge.generic.angular_acceleration_bound_radps2=domain.alpha;cfg.bridge.generic.calibration_id=domain.id;
 Eigen::Matrix3d world;for(int j=0;j<3;++j)for(int k=0;k<3;++k)world(j,k)=model["R_model_from_anchor_world"][j][k].as<double>();
 if((world.transpose()*world-Eigen::Matrix3d::Identity()).norm()>1e-10 || world.determinant()<.999999)throw std::invalid_argument("MODEL_WORLD_ROTATION");
 const double clock_offset=model["range_to_imu_time_s"].as<double>();
 std::vector<ImuMeasurement> imu;for(const auto& row:csv(folder+"/imu.csv")){ImuMeasurement m;m.id=MeasurementId(imu.size()+1);m.timestamp=stamp(number(row,"t"));
  m.specific_force_mps2={number(row,"ax"),number(row,"ay"),number(row,"az")};m.angular_velocity_radps={number(row,"gx"),number(row,"gy"),number(row,"gz")};
  if(!imu.empty() && !(imu.back().timestamp<m.timestamp))throw std::invalid_argument("CSV_IMU_ORDER");imu.push_back(m);}
 if(imu.empty())throw std::invalid_argument("CSV_IMU_EMPTY");
 GeneralizedCoreInitialization initialization;initialization.raw_law=rawlaw;
 initialization.sources=std::make_unique<GeneralizedSourceLedger>(policy.budget,rawlaw,ModelEvidenceStatus::DeclaredEmpirical);
 std::map<uint64_t,Batch> grouped;
 for(const auto& row:csv(folder+"/anchors.csv")){AnchorRecord anchor;anchor.id=AnchorId(integer(row,"anchor_id"));anchor.position_world_m=world*Eigen::Vector3d(number(row,"x"),number(row,"y"),number(row,"z"));anchor.covariance_m2.setZero();anchor.frame="world";anchor.map_version=domain.id;initialization.anchors.declare(anchor,1);}
 for(const auto& row:csv(folder+"/uwb.csv")){const auto message=integer(row,"source_message"),member=integer(row,"source_range");
  if(message>=(uint64_t(1)<<46) || member>65535)throw std::invalid_argument("CSV_SOURCE_ID_CAPACITY");
  UwbMeasurement m;m.id=MeasurementId((uint64_t(1)<<62)+(message<<16)+member);m.factor_id=FactorId(m.id.value());m.anchor_id=AnchorId(integer(row,"anchor_id"));m.tag_id=TagId(integer(row,"tag_id"));m.sequence=message;
  m.timestamp=stamp(number(row,"t")+clock_offset);m.range_m=number(row,"range");m.sigma_m=number(row,"sigma");m.lever_arm_body_m=Eigen::Vector3d(number(row,"lever_x"),number(row,"lever_y"),number(row,"lever_z"));
  m.anchor_position_m=world*Eigen::Vector3d(number(row,"anchor_x"),number(row,"anchor_y"),number(row,"anchor_z"));
  const bool valid=integer(row,"valid")!=0;auto& batch=grouped[message];batch.values.id=BatchId(message+1);
  if(batch.events.empty() || batch.values.timestamp<m.timestamp)batch.values.timestamp=m.timestamp;
  batch.events.push_back({m,m.timestamp,1,valid});if(valid)batch.values.measurements.push_back(m);
 }
 std::vector<Batch> batches;for(auto& item:grouped){auto& batch=item.second;for(auto& event:batch.events)event.received=batch.values.timestamp;
  if(!batches.empty() && !(batches.back().values.timestamp<batch.values.timestamp))throw std::invalid_argument("CSV_BATCH_ORDER");batches.push_back(std::move(batch));}
 size_t startup=0;while(startup<batches.size() && (batches[startup].values.timestamp.seconds()<imu.front().timestamp.seconds()+.5 || batches[startup].values.measurements.size()<4))++startup;
 if(startup==batches.size())throw std::invalid_argument("CAUSAL_STARTUP_UNAVAILABLE");
 auto first=std::lower_bound(imu.begin(),imu.end(),batches[startup].values.timestamp,[](const auto& m,TimestampNs time){return m.timestamp<time;});
 if(first==imu.end())throw std::invalid_argument("CAUSAL_STARTUP_IMU_MISSING");const size_t first_index=first-imu.begin();
 Eigen::Vector3d mean_force=Eigen::Vector3d::Zero();for(size_t i=0;i<=first_index;++i){initialization.sources->append(imu[i]);mean_force+=imu[i].specific_force_mps2;}mean_force/=first_index+1;
 for(size_t k=0;k<=startup;++k)for(const auto& event:batches[k].events)initialization.sources->registerPotentialRange(event);
 UwbBatch initial_ranges=batches[startup].values;initial_ranges.timestamp=first->timestamp;
 RollingRawState physical;const double speed=model["initial_velocity_radius"].as<double>();
 NavigationState numerical;numerical.timestamp=first->timestamp;numerical.accel_bias_mps2=vector(model["accel_bias_mps2"]);numerical.gyro_bias_radps=vector(model["gyro_bias_radps"]);
 for(int j=0;j<3;++j){physical.p[j]=RollingInterval(0);physical.v[j]=RollingInterval(-speed,speed);for(int k=0;k<3;++k)physical.rotation[j][k]=RollingInterval(-1,1);}
 for(int j=0;j<6;++j){const double center=j<3?numerical.accel_bias_mps2[j]:numerical.gyro_bias_radps[j-3],radius=model["initial_bias_radius"][j].as<double>();physical.bias[j]=RollingInterval(center)+RollingInterval(-radius,radius);}
 initialization.initial_set=measurementInitial(*initialization.sources,initial_ranges,physical,lever,domain.acceleration,domain.rate,ModelEvidenceStatus::DeclaredEmpirical,domain.id);
 const Eigen::Vector3d specific=mean_force-numerical.accel_bias_mps2;if(specific.norm()>1e-6)numerical.q_world_body=Eigen::Quaterniond::FromTwoVectors(specific,Eigen::Vector3d::UnitZ());
 auto seed=numericalUwbCandidate(initial_ranges,-1).functional.tag;
 if(!seed.allFinite()){seed.setZero();for(const auto& m:initial_ranges.measurements)seed+=m.anchor_position_m;seed/=initial_ranges.measurements.size();}
 numerical.position_world_m=seed-numerical.q_world_body*lever;
 UnifiedRollingIntegrityCore core(law,cfg,numerical,lever,domain,*first,std::move(initialization));
 size_t cursor=first_index+1,epoch=0;const size_t planned=batches.size()-startup-1;
 std::cerr<<"core="<<core.implementationId()<<" model="<<domain.id<<" startup_raw="<<first->id.value()<<" startup_ns="<<first->timestamp.value()<<" planned_after_startup="<<planned<<" mode="<<mode<<"\n";
 std::ofstream out(argv[3]);if(!out)throw std::runtime_error("OUTPUT_OPEN");out<<std::setprecision(17);
 out<<"epoch,batch_id,stamp_ns,core_id,model_status,bound_status,set_valid,status,recovery,imu_alarm,uwb_alarm,excluded,excluded_source,bridge,repair,polluted_prior,rejoined,first_dependency,updates,version,generation,parent,boundary,source_prefix,q,event_risk,registered,visible,measurements,invalid,time_groups,time_live,max_transport,hpl,vpl,raw_qualified,publication_protected,tag_x,tag_y,tag_z,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bax,bay,baz,bgx,bgy,bgz,core_ms,prepare_ms,candidate_ms,proof_ms,commit_ms,cleanup_ms,active_states,proof_explicit,slots,ledger,replay,capsules,rss_kb\n";
 for(size_t k=startup+1;k<batches.size() && (!maximum || epoch<maximum);++k){if(!(numerical.timestamp<batches[k].values.timestamp))continue;
  Sc6Envelope input;input.epoch=++epoch;input.uwb=batches[k].values;input.range_events=batches[k].events;input.ready=input.uwb.timestamp;
  while(cursor<imu.size() && !(input.uwb.timestamp<imu[cursor].timestamp))input.imu.push_back(imu[cursor++]);
  const auto started=std::chrono::steady_clock::now();Sc6Result r;
  try{r=core.step(input,mode=="on" && epoch>enable_after);}catch(const std::exception& error){std::cerr<<"FAILED epoch="<<epoch<<" source_batch="<<input.uwb.id.value()<<" stamp="<<input.uwb.timestamp.value()<<" reason="<<error.what()<<'\n';throw;}
  const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
  out<<epoch<<','<<input.uwb.id.value()<<','<<r.state.timestamp.value()<<','<<core.implementationId()<<",DECLARED_EMPIRICAL,"<<r.bound.status<<','<<r.bound.valid<<','<<r.action.status<<','<<r.action.recovery<<','<<r.imu_alarm<<','<<r.action.uwb_alarm<<','<<r.action.excluded<<','<<(r.action.excluded_source?std::to_string(r.action.excluded_source->value()):"")<<','<<r.action.current_bridge<<','<<r.action.boundary_repair<<','<<r.action.history_marginalized<<','<<r.action.rejoined<<','<<r.action.first_dependency<<",1,"<<r.version<<','<<r.generation<<','<<r.parent<<','<<r.boundary<<','<<r.source_prefix<<','<<r.q<<','<<r.bound.event_risk<<','<<r.input_counts.registered<<','<<r.input_counts.visible<<','<<r.input_counts.measurements<<','<<r.input_counts.invalid<<','<<r.time_factor_groups<<','<<r.time_committed_factors<<','<<r.max_range_transport<<','<<r.bound.hpl<<','<<r.bound.pl.z()<<",0,0";
  for(const auto& v:std::vector<Eigen::Vector3d>{r.tag,r.state.position_world_m,r.state.velocity_world_mps})for(int j=0;j<3;++j)out<<','<<v[j];
  const auto& q=r.state.q_world_body;out<<','<<q.w()<<','<<q.x()<<','<<q.y()<<','<<q.z();for(const auto& v:std::vector<Eigen::Vector3d>{r.state.accel_bias_mps2,r.state.gyro_bias_radps})for(int j=0;j<3;++j)out<<','<<v[j];
  out<<','<<elapsed<<','<<r.prepare_ms<<','<<r.candidate_ms<<','<<r.proof_ms<<','<<r.commit_ms<<','<<r.cleanup_ms<<','<<r.active_states<<','<<r.proof_explicit<<','<<r.slots<<','<<r.ledger<<','<<r.replay<<','<<r.capsules<<','<<r.rss<<'\n';out.flush();
 }
 std::cerr<<"COMMITTED="<<epoch<<" planned="<<planned<<" max_requested="<<maximum<<" backend_updates="<<core.updateCount()<<'\n';return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
