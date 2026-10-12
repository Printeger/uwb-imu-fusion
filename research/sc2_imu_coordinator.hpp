#pragma once
#include "sc2_imu_bridge.hpp"
#include <gtsam/inference/Symbol.h>
#include "bounded_imu_recovery.hpp"
namespace uwb_imu_pl::research {
struct Sc2NativeAssessment {
 ExclusionAction action;bool legal=true,current_bridge=false,history_purge=false;
 size_t first_dependency=0;std::string status="KEEP",reason;
 BoundedRecoveryAttempt candidate;
};
class Sc2RecoveryCoordinator {
 NavigationState initial_;RawInclusionParameters law_;Sc2ImuScope scope_;
 Sc2ImuMonitor monitor_;Sc2IndependentBridge bridge_;
 std::vector<ImuMeasurement> prefix_;
 TimestampNs quarantine_begin_,quarantine_end_;bool isolated_=false;
 public:
 Sc2RecoveryCoordinator(const NavigationState&,const RawInclusionParameters&,Sc2ImuScope={});
 void append(const ImuMeasurement&);
 Sc2ImuAssessment monitor()const{return monitor_.assessment();}
 const std::vector<Sc2PhysicalCandidate>& candidates()const{return monitor_.candidates();}
 Sc2NativeAssessment prepare(EpochTransaction&,bool fde,const GenericBridgeSpec&);
 void committed(const Sc2NativeAssessment&,const EpochTransaction&,const CommitReceipt&);
 RawRangeResult issue(const RawRangeLawEvidence& r,const EpochTransaction& tx,const CommitReceipt& receipt,const NavigationState& state,const Eigen::Vector3d& lever)const{return bridge_.issue(monitor_,r,tx,receipt,state,lever);}
 Sc2StateEnvelope envelope(int epoch,const Eigen::Vector3d& lever)const{return bridge_.envelope(epoch,lever);}
};
EpochCommitPlan sc2CommitPlan(const EpochTransaction&,const Sc2NativeAssessment&);
int runSc2Imu(int argc,char** argv);
}
