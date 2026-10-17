#pragma once
#include "uwb_imu_pl/common/types.hpp"

namespace uwb_imu_pl {
// Difference integers before floating conversion. Unsigned subtraction also
// handles the full signed timestamp range without signed-overflow UB.
inline double timestampDifferenceSecondsV1(TimestampNs end,TimestampNs begin){
 if(end<begin)return -static_cast<double>((uint64_t(begin.value())-uint64_t(end.value()))/1000000000.L);
 return static_cast<double>((uint64_t(end.value())-uint64_t(begin.value()))/1000000000.L);
}
}
