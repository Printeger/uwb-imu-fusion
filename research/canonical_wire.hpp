#pragma once
#include "empirical_sensor_adapter.hpp"

namespace uwb_imu_pl::research {
// Transport-neutral, lossless values. ROS and JSON populate this same object.
struct WireRange {
 UwbMeasurement measurement;
 TimestampNs received;
 uint64_t calibration_version=0;
 bool valid=false;
 double raw_range_m=0;
};
struct WireEvent {
 uint64_t sequence=0;
 TimestampNs received,stamp;
 bool is_imu=false;
 ImuMeasurement imu;
 BatchId batch;
 std::vector<WireRange> ranges;
};
WireEvent parseWireEvent(const YAML::Node&);
AnchorRegistry wireAnchors(const YAML::Node&,const EmpiricalSetup&);
class CanonicalWireAdapter {
 EmpiricalSetup setup_;
 CausalEmpiricalStream stream_;
 uint64_t sequence_=0;
 std::optional<TimestampNs> received_;
 int64_t range_offset_ns_=0;
 bool failed_=false;
 public:
 CanonicalWireAdapter(EmpiricalSetup,AnchorRegistry);
 void append(const WireEvent&);
 std::optional<EmpiricalEmission> next(bool actions){if(failed_)throw std::logic_error("WIRE_FAILED_CLOSED");return stream_.next(actions);}
 const CausalEmpiricalStream& stream()const{return stream_;}
 uint64_t events()const{return sequence_;}
};
}
