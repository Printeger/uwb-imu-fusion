#include "sc3_joint_bound.hpp"
namespace uwb_imu_pl::research {
Sc3JointBound Sc3IndependentJointBound::project(const Sc2ImuMonitor& monitor,const RawRangeLawEvidence& source,const EpochTransaction& tx,const NavigationState& center,const Eigen::Vector3d& lever,RawRangeBudget budget)const{
 Sc3JointBound out;auto imu=monitor.assessment();std::array<RawRangeShell,8> shells;
 out.expected=pool_.expected;out.represented=pool_.represented;
 if(!pool_.complete() || budget.all_ranges_healthy || !imu.valid || (!imu.nominal_possible && !imu.possible) || !monitor.matches(tx) || tx.previous_epoch+1!=tx.proposed_epoch || tx.begin.value()!=(static_cast<long long>(tx.proposed_epoch)-1)*50000000LL || tx.end.value()!=static_cast<long long>(tx.proposed_epoch)*50000000LL || center.timestamp!=tx.end || !rawRangeShells(source,tx.uwb_batch,tx.proposed_epoch,shells))return out;
 // Pool source identity is a semantic condition, not a hash or selected mask.
 for(size_t row=0;row<8;++row)if(pool_.uwb[row*100].mode.anchor_id!=tx.uwb_batch.measurements[row].anchor_id)return out;
 auto full=physical_.envelope(tx.proposed_epoch,lever);if(!full.valid)return out;
 out.actual_tag=center.position_world_m+center.q_world_body*lever;out.lever=lever;
 out.geometry=contractRawRangeBox(full.tag_low,full.tag_high,tx.uwb_batch,shells,out.actual_tag,budget);
 out.geometry.source="raw-joint-independent-physics-all-uwb-masks-v1";
 out.geometry.relaxation="CURRENT_UWB_MASK_OUTER_RELAXATION_OF_FULL_UWB_IMU_EPISODE_PRODUCT";
 out.geometry.event_risk=full.event_risk;
 // Every episode's current projection is NONE or one of8 anchor omissions;
 // every IMU branch is contained by this same independent full-state box.
 // Ignoring inter-epoch theta equality only enlarges the feasible set.
 out.valid=out.geometry.valid && out.geometry.considered==9;
 out.evaluated=out.valid?pool_.represented:0;out.transaction=tx.id;out.version=tx.base_graph_version;out.timestamp=tx.end;return out;
}
Sc3JointBound Sc3IndependentJointBound::issue(const Sc2ImuMonitor& monitor,const RawRangeLawEvidence& source,const EpochTransaction& tx,const CommitReceipt& receipt,const NavigationState& state,const Eigen::Vector3d& lever,RawRangeBudget budget)const{
 if(receipt.transaction_id!=tx.id || receipt.graph_version!=tx.base_graph_version+1 || receipt.committed_epoch!=tx.proposed_epoch || receipt.state_timestamp!=tx.end || receipt.backend_updates!=1)return {};
 auto out=project(monitor,source,tx,state,lever,budget);out.version=receipt.graph_version;return out;
}
}
