#pragma once
#include "rolling_monitor.hpp"
#include "time_uniform_event.hpp"
#include "rolling_raw.hpp"
namespace uwb_imu_pl::research {
struct OpenFaultRecord {
 uint64_t generation=0;TimestampNs begin,end;std::string id;
};
struct OpenUwbCandidate {uint64_t begin=0,end=0;unsigned anchor=0;double low=-INFINITY,high=INFINITY;std::string id;};
// An outer automaton over ALL nonoverlapping IMU episode sequences. Ended
// paths merge existentially into a no-active state; their state effects stay
// in the rolling certificate, and global birth identities remain a recipe.
class OpenFaultPool {
 TimeUniformEvent event_;RawInclusionParameters law_;
 Eigen::Matrix<double,6,1> bias_center_;
 std::array<RollingInterval,6> rw_support_;
 std::vector<Sc2PhysicalCandidate> active_;
 std::deque<ImuMeasurement> recent_;
 std::vector<OpenFaultRecord> records_;
 Sc2ImuAssessment result_;
 uint64_t samples_=0,births_=0,ended_=0,excluded_=0,generation_=0;
 size_t capacity_;double acceleration_,rate_;
 bool no_active_=true,alert_=false;
 TimestampNs alert_begin_,alert_end_;
 std::vector<OpenUwbCandidate> uwb_active_;
 uint64_t uwb_epoch_=0,uwb_ended_=0;
 bool uwb_no_active_=true;
 public:
 OpenFaultPool(const NavigationState&,const RawInclusionParameters&,double budget,size_t capacity=4096,double acceleration=.04,double rate=.2);
 void append(const ImuMeasurement&);
 RollingMonitorView view()const;
 void appendRanges(const UwbBatch&,const RollingRawState&,const Eigen::Vector3d&);
 size_t activeUwb()const{return uwb_active_.size();}
 uint64_t endedUwb()const{return uwb_ended_;}
 uint64_t samples()const{return samples_;}
 size_t activeSize()const{return active_.size();}
 uint64_t births()const{return births_;}
 uint64_t ended()const{return ended_;}
 const std::vector<OpenFaultRecord>& records()const{return records_;}
};
}
