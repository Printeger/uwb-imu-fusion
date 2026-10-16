#pragma once
#include "joint_raw.hpp"
#include "physical_time.hpp"
#include "source_layout_event.hpp"

namespace uwb_imu_pl::research {
// Numerical initialization is deliberately separate from this source-bound
// feasible domain. No covariance of a measurement bootstrap is treated as an
// independent observation. Producers must retain their original birth IDs.
struct InitialSetEvidence {
 RollingRawState domain;
 TimestampNs timestamp;
 ModelEvidenceStatus status=ModelEvidenceStatus::MissingOrInconsistent;
 std::string event_id,model_id;
 uint64_t source_prefix=0;
 double risk_upper=0;
 bool measurement_derived=false;
};
// Immutable epoch material produced by the core's source ledger. Range noise
// radii are at original measurement times; deterministic transport is added
// by the raw operator using its own velocity domain, never a selected center.
struct GeneralizedRawEvidence {
 TimestampNs begin,end;
 ModelEvidenceStatus status=ModelEvidenceStatus::MissingOrInconsistent;
 std::string event_id;
 uint64_t source_prefix=0;
 double risk_upper=0;
 double max_coordinate_radius=0;
 RawTimeStep evolution;
 std::vector<JointTimedSample> joint_samples;
 std::vector<double> range_noise_radius;
};
}
