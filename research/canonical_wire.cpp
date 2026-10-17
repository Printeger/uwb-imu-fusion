#include "canonical_wire.hpp"

namespace uwb_imu_pl::research {
namespace {
Eigen::Vector3d vector(const YAML::Node& n){
 if(!n.IsSequence() || n.size()!=3)throw std::invalid_argument("WIRE_VECTOR_SIZE");
 Eigen::Vector3d out(n[0].as<double>(),n[1].as<double>(),n[2].as<double>());
 if(!out.allFinite())throw std::invalid_argument("WIRE_VECTOR_NONFINITE");return out;
}
TimestampNs shifted(TimestampNs t,int64_t offset){
 if((offset>0 && t.value()>INT64_MAX-offset) || (offset<0 && t.value()<INT64_MIN-offset))throw std::overflow_error("WIRE_CLOCK_OVERFLOW");
 return TimestampNs(t.value()+offset);
}
}
WireEvent parseWireEvent(const YAML::Node& n){
 WireEvent e;e.sequence=n["event_sequence"].as<uint64_t>();e.received=TimestampNs(n["received_ns"].as<int64_t>());e.stamp=TimestampNs(n["stamp_ns"].as<int64_t>());
 const auto kind=n["kind"].as<std::string>();if(kind!="imu" && kind!="range")throw std::invalid_argument("WIRE_KIND");e.is_imu=kind=="imu";
 if(e.is_imu){e.imu.id=MeasurementId(n["measurement_id"].as<uint64_t>());e.imu.timestamp=e.stamp;e.imu.quality_flags=n["quality_flags"].as<uint32_t>();e.imu.specific_force_mps2=vector(n["accel"]);e.imu.angular_velocity_radps=vector(n["gyro"]);}
 else{e.batch=BatchId(n["batch_id"].as<uint64_t>());if(!n["ranges"].IsSequence() || n["ranges"].size()>4096)throw std::invalid_argument("WIRE_RANGE_CAPACITY");
  for(const auto& r:n["ranges"]){WireRange w;auto& m=w.measurement;m.id=MeasurementId(r["measurement_id"].as<uint64_t>());m.factor_id=FactorId(r["factor_id"].as<uint64_t>());m.anchor_id=AnchorId(r["anchor_id"].as<uint64_t>());m.tag_id=TagId(r["tag_id"].as<uint64_t>());m.sequence=r["source_sequence"].as<uint64_t>();m.timestamp=TimestampNs(r["stamp_ns"].as<int64_t>());m.quality_flags=r["quality_flags"].as<uint32_t>();m.range_m=r["range_m"].as<double>();m.sigma_m=r["sigma_m"].as<double>();m.anchor_position_m=vector(r["anchor"]);m.lever_arm_body_m=vector(r["lever"]);w.received=TimestampNs(r["received_ns"].as<int64_t>());w.calibration_version=r["calibration_version"].as<uint64_t>();w.valid=r["valid"].as<bool>();w.raw_range_m=r["raw_range_m"].as<double>();e.ranges.push_back(w);}
 }return e;
}
AnchorRegistry wireAnchors(const YAML::Node& rows,const EmpiricalSetup& setup){
 if(!rows.IsSequence())throw std::invalid_argument("WIRE_ANCHOR_SCHEMA");AnchorRegistry registry;
 for(const auto& row:rows){AnchorRecord a;a.id=AnchorId(row["id"].as<uint64_t>());a.position_world_m=setup.world*vector(row["position"]);a.frame="world";a.map_version=setup.domain.id;registry.declare(a,row["calibration_version"].as<uint64_t>());}return registry;
}
CanonicalWireAdapter::CanonicalWireAdapter(EmpiricalSetup setup,AnchorRegistry registry):setup_(std::move(setup)),stream_(setup_,std::move(registry)){
 const long double offset=setup_.range_clock_offset*1000000000.L;
 if(!std::isfinite(offset) || offset<INT64_MIN || offset>INT64_MAX)throw std::invalid_argument("WIRE_CLOCK_OFFSET");range_offset_ns_=std::llround(offset);
}
void CanonicalWireAdapter::append(const WireEvent& e)try{
 if(failed_)throw std::logic_error("WIRE_FAILED_CLOSED");
 if(sequence_==UINT64_MAX || e.sequence!=sequence_+1)throw std::invalid_argument("WIRE_EVENT_SEQUENCE_GAP_OR_DUPLICATE");
 if(received_ && e.received<*received_)throw std::invalid_argument("WIRE_RECEIVE_ORDER");
 if(e.received<e.stamp)throw std::invalid_argument("WIRE_FUTURE_SAMPLE");
 if(e.is_imu){if(!e.ranges.empty() || e.imu.timestamp!=e.stamp)throw std::invalid_argument("WIRE_IMU_SCHEMA");stream_.appendImu(e.imu,e.received);}
 else{
  if(e.ranges.empty() || e.ranges.size()>4096)throw std::invalid_argument("WIRE_RANGE_CAPACITY");
  EmpiricalBatch b;b.values.id=e.batch;b.values.timestamp=shifted(e.stamp,range_offset_ns_);
  for(const auto& w:e.ranges){auto m=w.measurement;
   if(e.stamp<m.timestamp || w.received<m.timestamp || e.received<w.received || !std::isfinite(w.raw_range_m))throw std::invalid_argument("WIRE_RANGE_TIME_OR_RAW_VALUE");
   if(w.valid && (!b.latest_original_valid_range || *b.latest_original_valid_range<m.timestamp))b.latest_original_valid_range=m.timestamp;
   m.timestamp=shifted(m.timestamp,range_offset_ns_);m.anchor_position_m=setup_.world*m.anchor_position_m;
   // Both timestamps describe this channel in the calibrated model clock.
   // Original sample/receive clocks remain on WireEvent for latency accounting.
   b.events.push_back({m,shifted(w.received,range_offset_ns_),w.calibration_version,w.valid});if(w.valid)b.values.measurements.push_back(m);
  }stream_.appendBatch(std::move(b));
 }
 sequence_=e.sequence;received_=e.received;
}catch(...){failed_=true;throw;}
}
