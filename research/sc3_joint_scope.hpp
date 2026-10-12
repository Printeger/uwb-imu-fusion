#pragma once
#include "sc2_imu_scope.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
namespace uwb_imu_pl::research {
struct Sc3PhysicalEpisode {
 FaultUnit unit;FaultModeBasis mode;std::string identity;
 int duration=0; // UWB epochs or IMU milliseconds according to sensor
};
struct Sc3PhysicalHypothesis {
 FaultHypothesisV2 hypothesis;std::string identity;
 int uwb=-1,imu=-1;
};
// Exact factored Cartesian product. Physical identity is never merged by a
// current graph-operation equivalence or local FD pass. Parameter blocks are
// disjoint even if their projections cancel in a frozen model.
struct Sc3FaultPool {
 std::string scope="sc3-one-uwb-one-imu-episode-rw-v1";
 std::vector<Sc3PhysicalEpisode> uwb,imu;
 size_t expected=0,represented=0,evaluated=0;
 Sc3PhysicalHypothesis at(size_t index)const;
 size_t size()const{return represented;}
 bool complete()const;
};
Sc3PhysicalEpisode bindSc3PhysicalEpisode(const Sc3PhysicalEpisode&,const EpochTransaction&);
Sc3FaultPool makeSc3FaultPool(const UwbBatch& identity_batch,size_t capacity=1400);
}
