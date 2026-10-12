#pragma once
#include "sc3_joint_scope.hpp"
#include "sc2_imu_bridge.hpp"
namespace uwb_imu_pl::research {
struct Sc3JointBound {
 RawRangeResult geometry;
 size_t expected=0,represented=0,evaluated=0;
 TransactionId transaction;uint64_t version=0;TimestampNs timestamp;
 Eigen::Vector3d actual_tag=Eigen::Vector3d::Constant(NAN),lever=actual_tag;
 std::string scope="sc3-one-uwb-one-imu-episode-rw-v1";
 bool valid=false;
};
// Envelopes use original source boundary and independent physics only;
// all9 physical UWB projections are required, including during local PASS.
class Sc3IndependentJointBound {
 Sc2IndependentBridge physical_;Sc3FaultPool pool_;
 public:
 Sc3IndependentJointBound(const NavigationState& initial,const RawInclusionParameters& law,const UwbBatch& identity):physical_(initial,law),pool_(makeSc3FaultPool(identity)){}
 Sc2StateEnvelope envelope(int epoch,const Eigen::Vector3d& lever)const{return physical_.envelope(epoch,lever);}
 Sc3JointBound project(const Sc2ImuMonitor&,const RawRangeLawEvidence&,const EpochTransaction&,const NavigationState&,const Eigen::Vector3d&,RawRangeBudget={})const;
 Sc3JointBound issue(const Sc2ImuMonitor&,const RawRangeLawEvidence&,const EpochTransaction&,const CommitReceipt&,const NavigationState&,const Eigen::Vector3d&,RawRangeBudget={})const;
 const Sc3FaultPool& pool()const{return pool_;}
};
}
