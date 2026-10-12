#pragma once
#include "raw_imu_episode.hpp"
#include "raw_nonlinear_inclusion.hpp"
#include "uwb_imu_pl/integrity/fault_model.hpp"
#include <array>
namespace uwb_imu_pl::research {
struct Sc2ImuScope {
 std::string id="sc2-single-imu-episode-rw-v1";
 int horizon=50;std::vector<int> durations_ms{50,500};
 double acceleration_bound=.04,angular_rate_bound=.2;
 size_t candidate_capacity=601;
};
struct Sc2PhysicalCandidate {
 RawImuEpisode episode;double theta_low=-INFINITY,theta_high=INFINITY;bool possible=true;
};
struct Sc2ImuAssessment {
 bool valid=false,nominal_possible=true,alarm=false,current_violation=false;
 size_t possible=0,examined=0;int samples=0;
 TimestampNs first_alarm,union_begin,union_end;
 std::string status="NOT_READY",detector_level="RAW_DETECTOR_BOUND";
};
// Immutable observed prefix only. Force/gyro domain and all nominal source
// coordinates are registered independently of the actual fault realization.
class Sc2ImuMonitor {
 NavigationState initial_;RawInclusionParameters law_;Sc2ImuScope scope_;
 std::vector<Sc2PhysicalCandidate> candidates_;Sc2ImuAssessment result_;
 public:
 Sc2ImuMonitor(const NavigationState&,const RawInclusionParameters&,Sc2ImuScope={});
 void append(const ImuMeasurement&);
 Sc2ImuAssessment assessment()const{return result_;}
 const std::vector<Sc2PhysicalCandidate>& candidates()const{return candidates_;}
};
struct Sc2FaultSupportClosure {
 std::vector<MeasurementId> raw_samples;
 std::vector<size_t> direct_pim_epochs,reference_dependent_epochs;
 size_t first_epoch=0;bool valid=false;
};
// Includes the last shared point sample in BOTH trapezoids/PIMs, and the
// downstream state/biasHat/reference dependency, not just the fault row.
Sc2FaultSupportClosure sc2SupportClosure(const RawImuEpisode&,int through_epoch,int horizon=50);
}
