#pragma once
#include "rolling_raw.hpp"
#include "time_uniform_event.hpp"
namespace uwb_imu_pl::research {
struct JointRawLaw {
 double accel_adc=.02,gyro_adc=.001,accel_rw=.001,gyro_rw=.0001;
 double gravity=9.81,acceleration=.04,rate=.2,jerk=.02;
 // Optional global physical speed envelope. Infinity means no such evidence.
 double global_speed=INFINITY;
};
// The original automaton has no episode at sample0; this is an original-law
// necessary constraint, not FD declaring a later sample healthy.
bool jointInitial(RollingRawState&,const ImuMeasurement&,const JointRawLaw&,const TimeUniformEvent&);
JointConstraintEvidence jointContract(RollingRawState&,RollingRawState&,
 const std::vector<ImuMeasurement>&,const JointRawLaw&,const TimeUniformEvent&,size_t branch_budget=7,bool dominant_hull_shortcut=true);
// Actual RAW sample plus its integration interval end. A held endpoint keeps
// the original measurement timestamp/ID and reuses its noise support.
struct JointTimedSample {
 ImuMeasurement raw;
 TimestampNs integration_end;
 std::array<RollingInterval,6> adc_radius,rw_from_begin,rw_to_end;
};
JointConstraintEvidence jointContract(RollingRawState&,RollingRawState&,
 TimestampNs begin,TimestampNs end,const std::vector<JointTimedSample>&,
 const JointRawLaw&,size_t branch_budget=7);
}
