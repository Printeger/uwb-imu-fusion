#pragma once
#include "raw_range_contraction.hpp"
namespace uwb_imu_pl::research {
// No simulation truth, random draws, or fault injection labels cross this API.
// Legacy source_valid remains a legacy-law qualification, never an input-valid flag.
struct Sc6Envelope {
 size_t epoch=0;
 std::vector<ImuMeasurement> imu;
 UwbBatch uwb;
 RawRangeLawEvidence range;
 bool source_valid=false;
};
}
