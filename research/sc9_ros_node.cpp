#include "canonical_wire.hpp"
#include "wire_result.hpp"
#include "uwb_imu_pl/CanonicalSensorEvent.h"
#include "uwb_imu_pl/UnifiedRollingOutput.h"
#include <ros/ros.h>
#include <tbb/global_control.h>
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <fstream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
namespace {
int64_t steady(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
template<class V>Eigen::Vector3d vector(const V& v){return {v[0],v[1],v[2]};}
WireEvent decode(const CanonicalSensorEvent& m){
 WireEvent e;e.sequence=m.event_sequence;e.stamp=TimestampNs(m.stamp_ns);e.received=TimestampNs(m.received_ns);e.is_imu=m.kind==CanonicalSensorEvent::IMU;
 if(m.kind!=CanonicalSensorEvent::IMU && m.kind!=CanonicalSensorEvent::RANGE)throw std::invalid_argument("ROS_WIRE_KIND");
 if(e.is_imu){if(!m.ranges.empty())throw std::invalid_argument("ROS_IMU_HAS_RANGES");e.imu.id=MeasurementId(m.measurement_id);e.imu.timestamp=e.stamp;e.imu.quality_flags=m.quality_flags;e.imu.specific_force_mps2=vector(m.accel);e.imu.angular_velocity_radps=vector(m.gyro);}
 else{e.batch=BatchId(m.batch_id);for(const auto& r:m.ranges){WireRange w;auto& v=w.measurement;v.id=MeasurementId(r.measurement_id);v.factor_id=FactorId(r.factor_id);v.anchor_id=AnchorId(r.anchor_id);v.tag_id=TagId(r.tag_id);v.sequence=r.source_sequence;v.timestamp=TimestampNs(r.stamp_ns);v.quality_flags=r.quality_flags;v.range_m=r.range_m;v.sigma_m=r.sigma_m;v.anchor_position_m=vector(r.anchor);v.lever_arm_body_m=vector(r.lever);w.received=TimestampNs(r.received_ns);w.calibration_version=r.calibration_version;w.valid=r.valid;w.raw_range_m=r.raw_range_m;e.ranges.push_back(w);}}
 return e;
}
struct Queued{CanonicalSensorEvent::ConstPtr message;int64_t callback;};
}
int main(int argc,char** argv){ros::init(argc,argv,"sc9_unified_rolling");ros::NodeHandle nh,priv("~");
 auto publisher=nh.advertise<UnifiedRollingOutput>("sc9/result",256,true);
 try{
  std::string backend,model,anchors,mode;priv.param<std::string>("integrity_backend",backend,"");priv.param<std::string>("model",model,"");priv.param<std::string>("anchors",anchors,"");priv.param<std::string>("rolling",mode,"on");
  if(backend!="unified_rolling")throw std::invalid_argument("ROS_EXPLICIT_UNIFIED_BACKEND_REQUIRED");if(mode!="on" && mode!="off")throw std::invalid_argument("ROS_ROLLING_MODE");
  tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
  auto setup=empiricalSetup(YAML::LoadFile(model));CanonicalWireAdapter adapter(setup,wireAnchors(YAML::LoadFile(anchors),setup));
  std::string audit_path;priv.param<std::string>("input_audit",audit_path,"");std::ofstream audit;
  if(!audit_path.empty()){audit.open(audit_path);if(!audit)throw std::runtime_error("ROS_INPUT_AUDIT_OPEN");audit<<"event_sequence,kind,stamp_ns,received_ns,release_ns,callback_ns,queue_before,accepted\n";}
  std::mutex mutex;std::condition_variable condition;std::deque<Queued> queue;size_t high_water=0;bool overflow=false;
  const auto connect_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(30);
  while(publisher.getNumSubscribers()!=1){if(std::chrono::steady_clock::now()>connect_deadline)throw std::runtime_error("ROS_EXPECTED_ONE_ACTUAL_SINK");ros::WallDuration(.01).sleep();}
  auto subscriber=nh.subscribe<CanonicalSensorEvent>("sc9/input",512,[&](const CanonicalSensorEvent::ConstPtr& m){const auto arrived=steady();std::lock_guard<std::mutex> lock(mutex);const bool accept=queue.size()<512;if(audit.is_open()){audit<<m->event_sequence<<','<<unsigned(m->kind)<<','<<m->stamp_ns<<','<<m->received_ns<<','<<m->release_steady_ns<<','<<arrived<<','<<queue.size()<<','<<accept<<'\n';if(!audit)overflow=true;}if(!accept){overflow=true;}else{queue.push_back({m,arrived});high_water=std::max(high_water,queue.size());}condition.notify_one();},ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  ros::AsyncSpinner spinner(1);spinner.start();std::optional<std::pair<int64_t,int64_t>> origin;int64_t auxiliary_ns=0;
  while(ros::ok()){
   Queued q;size_t watermark;{std::unique_lock<std::mutex> lock(mutex);condition.wait_for(lock,std::chrono::milliseconds(100),[&]{return overflow || !queue.empty();});if(overflow)throw std::length_error("ROS_INPUT_QUEUE_CAPACITY");if(queue.empty())continue;q=std::move(queue.front());queue.pop_front();watermark=high_water;}
   const auto& m=*q.message;const int64_t start=steady();const auto clock=std::make_pair(m.replay_origin_steady_ns,m.recording_origin_ns);
   if(origin && *origin!=clock)throw std::invalid_argument("ROS_REPLAY_CLOCK_CHANGED");origin=clock;
   UnifiedRollingOutput result;result.callback_steady_ns=q.callback;result.worker_start_steady_ns=start;result.replay_origin_steady_ns=clock.first;result.recording_origin_ns=clock.second;result.input_queue_high_water=watermark;
   if(m.kind==CanonicalSensorEvent::END){
    const auto& s=adapter.stream();if(m.event_sequence!=adapter.events()+1)throw std::invalid_argument("ROS_END_SEQUENCE_GAP");
    if(!s.initialized() || s.pendingBatches())throw std::runtime_error("ROS_END_WITH_UNCLOSED_BATCHES");
    std::ostringstream info;info<<"startup_digest="<<s.startupDigest()<<" startup_batches="<<s.startupBatches()<<" commits="<<s.epochs()<<" pending_batches="<<s.pendingBatches()<<" unused_imu="<<s.pendingImu()<<" updates="<<s.updates();
    result.record=info.str();result.finished=true;result.input_events=adapter.events();result.publish_steady_ns=steady();publisher.publish(result);ROS_INFO_STREAM(result.record);ros::WallDuration(.5).sleep();return 0;
   }
   const __int128 scheduled=__int128(m.replay_origin_steady_ns)+__int128(m.received_ns)-__int128(m.recording_origin_ns);
   if(scheduled<INT64_MIN || scheduled>INT64_MAX || __int128(m.release_steady_ns)<scheduled || q.callback<m.release_steady_ns)throw std::invalid_argument("ROS_FUTURE_RELEASE_OR_CLOCK_OVERFLOW");
   adapter.append(decode(m));result.input_events=adapter.events();
   int64_t segment_start=start;
   for(;;){const auto core_start=steady();auto emission=adapter.next(mode=="on");if(!emission){auxiliary_ns+=steady()-segment_start;break;}
    result.core_ms=(steady()-core_start)/1e6;result.epoch=adapter.stream().epochs();result.batch_id=emission->batch.value();result.sample_ns=emission->result.state.timestamp.value();result.latest_original_used_ns=emission->latest_original_used.value();result.record=wireResultRecord(emission->result,emission->batch,result.epoch,result.core_ms);result.core_ms=(auxiliary_ns+steady()-segment_start)/1e6;auxiliary_ns=0;result.publish_steady_ns=steady();publisher.publish(result);segment_start=steady();
   }
  }throw std::runtime_error("ROS_STOPPED_BEFORE_END");
 }catch(const std::exception& e){UnifiedRollingOutput failure;failure.finished=true;failure.error=e.what();failure.publish_steady_ns=steady();publisher.publish(failure);ROS_ERROR_STREAM(e.what());ros::WallDuration(.5).sleep();return 1;}
}
