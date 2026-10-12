#pragma once
#include "fused_epoch_pl.hpp"
#include "hybrid_prediction.hpp"
#include "online_gaussian_history.hpp"
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
inline bool hybridRawEpisodeColumns(const UwbBatch& batch,const HybridHistoryEvidence& history,std::vector<int>& columns){
 columns.clear();for(const auto& segment:history.segments){int anchor,onset,duration;
  if(!hybridEpisodeIdentity(segment.episode,anchor,onset,duration,true) || onset>int(history.epoch))return false;
  int column=-1;for(size_t i=0;i<batch.measurements.size();++i)if(batch.measurements[i].anchor_id.value()==uint64_t(anchor))column=i;
  if(column<0)return false;
  columns.push_back(int(history.epoch+1)<onset+duration?column:-1);
 }return true;
}
// Raw path consumes the extracted historical set and full joint prediction.
// Its mathematical snapshot is diagnostic: random chart/nonlinear remainder
// and hardware laws remain unqualified even when all numbers are finite.
inline EpochRaim hybridRawCurrent(const UwbBatch& batch,const OnlinePreUwbEvidence& evidence,
 const Eigen::Vector3d& lever,const EpochRaim& a,bool fde,double alpha,
 Eigen::Vector3d* history_part=nullptr,double* eta=nullptr,bool shared_episode=false){
 if(evidence.history.status!="BOUNDED" || evidence.transaction.value()==0 || evidence.excluded_batch!=batch.id || evidence.chart.timestamp!=batch.timestamp){EpochRaim r;r.status="HISTORY_EVIDENCE_REFUSED";return r;}
 auto c=tagMap(evidence.chart,lever);
 gtsam::Pose3 pose(gtsam::Rot3(evidence.chart.q_world_body.toRotationMatrix()),evidence.chart.position_world_m);
 gtsam::Matrix hp;Eigen::VectorXd innovation=-UwbPoseBatchFactor(1,batch,lever).evaluateError(pose,hp);
 Eigen::MatrixXd h=Eigen::MatrixXd::Zero(batch.measurements.size(),15);h.leftCols(6)=hp;
 innovation-=h*evidence.correction;
 std::vector<int> continuing;
 if(shared_episode && !hybridRawEpisodeColumns(batch,evidence.history,continuing)){EpochRaim r;r.status="HISTORY_EPISODE_PROVENANCE_REFUSED";return r;}
 auto candidate=[&](int excluded){EpochRaim r;r.excluded=excluded;
  if(!spatialAnchors(retainRanges(batch,excluded))){r.status="GEOMETRY_OR_MIRROR_UNQUALIFIED";return r;}
  auto bound=hybridBound(evidence.history.covariance,h,validatedUwbCovariance(batch),Eigen::MatrixXd::Zero(15,batch.measurements.size()),c,innovation,evidence.history,excluded,alpha,evidence.history.radius);
  if(shared_episode)bound=hybridJointEpisodeBound(std::move(bound),h,c,innovation,evidence.history,continuing,evidence.history.radius);
  r.gaussian=bound.gaussian;r.retained=bound.retained;
  if(r.gaussian.gain.size())responses(r,c,15,bound.retained.size());
  r.status=bound.status=="FIXED_MODEL_VERIFIED"?"FROZEN_MODEL_ONLY":bound.status;
  r.gaussian.status=r.status;
  if(r.gaussian.mean.size()==15)r.tag=evidence.chart.position_world_m+evidence.chart.q_world_body*lever+c*(evidence.correction+r.gaussian.mean);
  return r;
 };
 auto result=epochFde(batch.measurements.size(),fde,candidate,a.alarm);
 if(result.gaussian.mean.size()==15){auto bound=hybridBound(evidence.history.covariance,h,validatedUwbCovariance(batch),Eigen::MatrixXd::Zero(15,batch.measurements.size()),c,innovation,evidence.history,result.excluded,alpha,evidence.history.radius);
  if(history_part)*history_part=bound.history;
  if(eta)*eta=bound.eta;
 }
 return result;
}
enum class HybridDomain { FixedModel, RawUnqualified };
struct HybridFinalResult {
 Eigen::Vector3d pl=Eigen::Vector3d::Constant(INFINITY),center=Eigen::Vector3d::Constant(INFINITY);
 bool numerical=false,model=false,risk=false,within_al=false,receipt_bound=false,on_time=false;
 bool finiteBoundQualified()const{return numerical && model && risk && receipt_bound;}
 // Unknown application requirements never acquire qualification from a
 // diagnostic AL pair. Legacy available() retains its historical semantics.
 bool application_policy_qualified=false,formal_eligible=false,publication_protected=false;
 bool applicationAvailable()const{return finiteBoundQualified() && application_policy_qualified && within_al && on_time;}
 bool available()const{return numerical && model && risk && within_al && receipt_bound;}
 bool timelyAvailable()const{return available() && on_time;}
};
inline HybridFinalResult finalizeHybrid(const Eigen::Vector3d& referencePL,
 const Eigen::Vector3d& reference,const NavigationState& actual,const Eigen::Vector3d& lever,
 const CommitReceipt& receipt,TransactionId expected,size_t epoch,TimestampNs time,
 uint64_t previousGraph,HybridDomain domain,double risk,double budget,double horizontalAL,
 double verticalAL,double serviceMs){
 HybridFinalResult result;Eigen::Vector3d tag=actual.position_world_m+actual.q_world_body*lever;
 result.center=(tag-reference).cwiseAbs();result.pl=referencePL+result.center;
 result.numerical=referencePL.allFinite() && (referencePL.array()>=0).all() && reference.allFinite() && tag.allFinite() && result.pl.allFinite() && std::isfinite(result.pl.head<2>().norm());
 result.model=domain==HybridDomain::FixedModel;
 result.risk=result.model && std::isfinite(risk) && risk>0 && std::isfinite(budget) && budget>0 && budget<1 && risk<=budget;
 result.receipt_bound=receipt.transaction_id==expected && receipt.committed_epoch==epoch && receipt.state_timestamp==time && actual.timestamp==time && receipt.graph_version==previousGraph+1 && receipt.backend_updates==1;
 result.within_al=result.numerical && std::isfinite(horizontalAL) && horizontalAL>0 && std::isfinite(verticalAL) && verticalAL>0 && result.pl.head<2>().norm()<=horizontalAL && result.pl.z()<=verticalAL;
 result.on_time=std::isfinite(serviceMs) && serviceMs>=0 && serviceMs<=50;return result;
}
inline void advanceNativeHistory(HybridHistoryProducer& producer,const EpochTransaction& tx,
 const EpochCommitPlan& plan,const CommitReceipt& receipt,const Eigen::VectorXd& observations,
 const std::vector<int>& retained,const std::vector<uint64_t>& actualIds){
 if(receipt.transaction_id!=tx.id || receipt.committed_epoch!=tx.proposed_epoch || receipt.state_timestamp!=tx.end || receipt.graph_version!=tx.base_graph_version+1 || receipt.backend_updates!=1 || tx.previous_epoch!=producer.query().epoch)throw std::runtime_error("H_NATIVE_RECEIPT_REFUSED");
 std::set<uint64_t> committedIds;
 for(auto group:plan.groups_to_add){if(!receipt.group_to_slots.count(group.value()))throw std::runtime_error("H_NATIVE_GROUP_RECEIPT");
  for(const auto& g:tx.uwb_groups)if(g.id==group){
   for(auto id:g.source_measurements)committedIds.insert(id.value());
   for(const auto& factor:g.factors){auto uwb=dynamic_cast<const UwbPoseBatchFactor*>(factor.get());if(!uwb)throw std::runtime_error("H_NATIVE_FACTOR_TYPE");
    for(const auto& measurement:uwb->batch().measurements){int row=int(measurement.id.value()-receipt.committed_epoch*100)-1;
     if(row<0 || row>=observations.size() || !std::isfinite(observations(row)))throw std::runtime_error("H_NATIVE_DATA_SHAPE");
     double actual=measurement.range_m-(measurement.anchor_position_m-producer.law().reference_tag).norm();
     if(std::abs(actual-observations(row))>1e-12*(1+std::abs(actual)))throw std::runtime_error("H_NATIVE_RAW_VALUES");
    }
   }
  }
 }
 if(committedIds!=std::set<uint64_t>(actualIds.begin(),actualIds.end()))throw std::runtime_error("H_NATIVE_RETAINED_SOURCES");
 producer.committed(receipt.committed_epoch,receipt.committed_epoch,observations,retained,actualIds);
}
} // namespace
