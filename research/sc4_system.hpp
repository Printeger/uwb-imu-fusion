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
 int64_t release_ns=0,start_ns=0,alarm_ns=0,selected_ns=0,commit_ns=0,proof_ns=0,output_ns=0;
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
 size_t imu_cursor_=0,sequence_=0;
 bool gap_=false,carry_=false;
 TimestampNs carry_end_;
 double risk_=0;
 public:
 explicit Sc4System(Sc4Input);
 const Sc4Input& input()const{return input_;}
 Sc4Output process(size_t index,bool fde,Sc4Clock::time_point release,
                   Sc4Clock::time_point mission_origin);
 void sourceGap(){gap_=true;}
};
int runSc4System(int,char**);
}
