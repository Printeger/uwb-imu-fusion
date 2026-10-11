#pragma once
#include "fused_epoch_pl.hpp"
namespace uwb_imu_pl::research {
// A is advisory. Either detector triggers the complete B candidate family;
// A's ranking never removes a candidate and A never mutates the transaction.
inline EpochRaim hybridCurrent(const UwbBatch& batch,
 const ResearchEpochPredictionV1& prediction,const Eigen::Vector3d& lever,
 const EpochRaim& a,
 bool fde,double alpha,double tail) {
 return epochFde(batch.measurements.size(),fde,[&](int excluded){
  return fusedCandidateNumerical(batch,excluded,prediction,lever,alpha,tail);
 },a.alarm);
}
} // namespace
