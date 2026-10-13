#pragma once
#include "sc3_joint_coordinator.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include <chrono>
#include <deque>
#include <functional>

namespace uwb_imu_pl::research {
using Sc4Clock = std::chrono::steady_clock;
struct Sc4Input {
 IntegrityConfig config;
 NavigationState initial;
 Eigen::Vector3d lever=Eigen::Vector3d::Zero();
 RawInclusionParameters law;
 double source_time_origin_s=0;
 std::vector<ImuMeasurement> imu;
 std::vector<UwbBatch> batches;
 std::vector<RawRangeLawEvidence> ranges;
 bool matched_source=false;
 std::string source="RECORD_UNCALIBRATED",scope="NONE";
};
Sc4Input sc4MatchedInput(const std::string&,const std::string&,int);
// Canonical benchmark cache: actual measurements only, no GT or certificate.
Sc4Input sc4RecordInput(const std::string&,const std::string&,int);
std::string sc4InputError(const Sc4Input&);

struct Sc4Timing {
 double ingest_ms=0,prepare_ms=0,candidate_ms=0,commit_ms=0,bound_ms=0;
 double cleanup_ms=0,core_ms=0,full_ms=0,queue_ms=0,arrival_to_output_ms=0,measurement_age_ms=0;
 int64_t release_ns=0,enqueue_ns=0,start_ns=0,alarm_ns=0,selected_ns=0,commit_ns=0,proof_ns=0,output_ns=0;
};
struct Sc4QueuedBatch {size_t index;Sc4Clock::time_point release,enqueued;};
class Sc4Queue {
 size_t capacity_,peak_=0;
 std::deque<Sc4QueuedBatch> jobs_;
 public:
 explicit Sc4Queue(size_t capacity):capacity_(capacity){if(!capacity)throw std::runtime_error("SC4 zero queue capacity");}
 bool push(Sc4QueuedBatch job){if(jobs_.size()==capacity_)return false;jobs_.push_back(job);peak_=std::max(peak_,jobs_.size());return true;}
 Sc4QueuedBatch pop(){if(jobs_.empty())throw std::runtime_error("SC4 empty queue");auto job=jobs_.front();jobs_.pop_front();return job;}
 bool cancel(size_t index){auto i=std::find_if(jobs_.begin(),jobs_.end(),[&](const auto& j){return j.index==index;});if(i==jobs_.end())return false;jobs_.erase(i);return true;}
 size_t size()const{return jobs_.size();}size_t peak()const{return peak_;}
};
struct Sc4Output {
 size_t sequence=0,epoch=0,updates=0,history_epochs=0,active_slots=0,physical_expected=481401,physical_evaluated=0;
 uint64_t transaction=0,version=0;
 NavigationState state;
 Eigen::Vector3d tag=Eigen::Vector3d::Constant(NAN);
 Sc3JointBound proof;
 Sc3NativeAssessment action;
 Sc4Timing timing;
 std::string functional="NOT_ATTEMPTED",bound_status="SOURCE_UNQUALIFIED";
 std::string source,scope;
 bool finite=false,on_time=false,core_on_time=false,carry=false,protected_output=false;
 double cumulative_risk=0;
 std::string retained_ids,input_ids,removed_groups;
};
class Sc4System {
 Sc4Input input_;
 std::unique_ptr<IncrementalUwbImuEstimator> backend_;
 std::unique_ptr<Sc3JointCoordinator> joint_;
 size_t imu_cursor_=0,sequence_=0;uint64_t last_version_=1;
 bool gap_=false,carry_=false;
 TimestampNs carry_end_;
 double risk_=0;
 public:
 explicit Sc4System(Sc4Input,bool optimized=false);
 const Sc4Input& input()const{return input_;}
 Sc4Output process(size_t index,bool fde,Sc4Clock::time_point release,
                   Sc4Clock::time_point mission_origin);
 Sc4Output reject(size_t index,const std::string&,Sc4Clock::time_point release,
                  Sc4Clock::time_point mission_origin);
 void sourceGap(){gap_=true;}
};
int runSc4System(int,char**);
}
