#pragma once
#include "sc3_joint_bound.hpp"
#include "sc2_imu_coordinator.hpp"
#include "epoch_raim.hpp"
#include "uwb_imu_pl/integrity/fde_manager.hpp"
namespace uwb_imu_pl::research {
struct Sc3DecisionContext {
 const Sc3IndependentJointBound* bound=nullptr;const Sc2ImuMonitor* monitor=nullptr;
 const RawRangeLawEvidence* range=nullptr;const EpochTransaction* transaction=nullptr;
 const std::vector<NavigationState>* centers=nullptr;
 const ActionSearchResultV1* search=nullptr;const GeneratedActionSnapshotV1* trusted=nullptr;
 Eigen::Vector3d lever=Eigen::Vector3d::Constant(NAN);
 size_t expected_roots=0;
};
struct Sc3NativeAssessment {
 Sc2NativeAssessment imu;
 FdeDecision decision;
 int excluded=-1;size_t candidates=0,finite_candidates=0,post_candidates=0;
 size_t expected_actions=0,evaluated_actions=0,uwb_history_removed=0;
 double candidate_ms=0,detector_statistic=INFINITY,detector_threshold=0;
 bool uwb_alarm=false,legal=false,imu_rejoined=false;
 std::string status="NOT_RUN";
};
class Sc3JointCoordinator {
 NavigationState initial_;RawInclusionParameters law_;
 Sc2RecoveryCoordinator imu_;Sc3IndependentJointBound bound_;
 int anchor_=-1,uwb_end_=0;size_t action_capacity_;
 public:
 Sc3JointCoordinator(const NavigationState&,const RawInclusionParameters&,const UwbBatch&,size_t action_capacity=18);
 void append(const ImuMeasurement& x){imu_.append(x);}
 Sc3NativeAssessment prepare(EpochTransaction&,const RawRangeLawEvidence&,const Eigen::Vector3d&,const GenericBridgeSpec&,bool fde);
 void committed(const Sc3NativeAssessment&,const EpochTransaction&,const CommitReceipt&);
 Sc3JointBound issue(const RawRangeLawEvidence& r,const EpochTransaction& t,const CommitReceipt& c,const NavigationState& s,const Eigen::Vector3d& lever)const{return bound_.issue(imu_.rawMonitor(),r,t,c,s,lever);}
 Sc2ImuAssessment monitor()const{return imu_.monitor();}
};
int runSc3Joint(int argc,char** argv);
}
