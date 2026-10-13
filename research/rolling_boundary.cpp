#include "rolling_boundary.hpp"
namespace uwb_imu_pl::research {
namespace {
using I=RollingInterval;
RollingVector contracted(RollingRawState& s,const RollingRawMaterial& m,
 const RollingSourceEvent& e,const Eigen::Vector3d& l,size_t& branches){
 std::array<RawRangeShell,8> shells;
 const int epoch=int(m.batch.id.value());
 if(!rawRangeShells(m.range,m.batch,epoch,shells,e.mission,e.q))throw std::invalid_argument("RAW_BATCH_SOURCE_SCOPE_REFUSED");
 auto t=rollingTag(s,l);Eigen::Vector3d low,high;
 for(int j=0;j<3;++j){low(j)=t[j].lower();high(j)=t[j].upper();}
 auto result=contractRawRangeBox(low,high,m.batch,shells,Eigen::Vector3d::Zero());
 if(!result.valid || result.considered!=9)throw std::invalid_argument("RAW_CONTRACTION_SCOPE_INCONSISTENT");
 branches+=9;
 for(int j=0;j<3;++j)t[j]=I(result.low(j),result.high(j));
 if(!rollingContractTag(s,t,l))throw std::invalid_argument("RAW_TAG_BODY_INCONSISTENT");
 return t;
}
}
RollingIntegrity::RollingIntegrity(const ResolvedRollingPolicyV1& p,const NavigationState& s,
 const Eigen::Matrix<double,15,1>& sigmas,double ba,double bg,const Eigen::Vector3d& lever,double acceleration,double rate):
 policy_(p),event_(p.mission,p.budget),accel_rw_(ba),gyro_rw_(bg),acceleration_(acceleration),rate_(rate),lever_(lever){
 certificate_.separator=rollingInitial(s,sigmas,event_.q,1e-10);
 certificate_.event_id=event_.id;certificate_.fault_not_started=28ULL*p.mission;
}
RollingBoundaryProposalV1 RollingIntegrity::propose(const EpochTransaction& tx,
 const EpochCommitPlan& plan,const RawRangeLawEvidence& source,const Sc2ImuMonitor& monitor)const{
 if(failed_ || certificate_.event_id!=event_.id || tx.base_graph_version!=certificate_.version ||
    tx.proposed_epoch!=certificate_.covered_through+1)throw std::invalid_argument("BOUNDARY_PARENT_VERSION_REFUSED");
 event_.prefix(tx.proposed_epoch);
 if(!monitor.matches(tx) || (!monitor.assessment().nominal_possible && !monitor.assessment().possible))
  throw std::invalid_argument("IMU_TASK_SCOPE_INCONSISTENT");
 RollingBoundaryProposalV1 out;out.material=captureBeforeUpdate(tx,plan,policy_);
 out.parent_generation=certificate_.generation;out.explicit_material=explicit_;
 out.explicit_material.push_back({tx.uwb_batch,source,out.material.raw_first,out.material.raw_last});
 std::vector<RollingRawState> states{certificate_.separator};
 size_t expected=certificate_.separator.epoch+1;
 for(const auto& m:out.explicit_material){
  if(m.batch.id.value()!=expected++)throw std::invalid_argument("EXPLICIT_WINDOW_SOURCE_GAP");
  auto s=rollingPropagate(states.back(),acceleration_,rate_,accel_rw_,gyro_rw_,event_.q);
  contracted(s,m,event_,lever_,out.physical_branches);states.push_back(std::move(s));
 }
 // Registered bounded number of forward/backward necessary-constraint passes.
 for(int pass=0;pass<2;++pass){
  for(size_t j=states.size()-1;j>0;--j)if(!rollingContractDynamics(states[j-1],states[j],acceleration_))throw std::invalid_argument("RAW_BACKWARD_RELATION_INCONSISTENT");
  for(size_t j=1;j<states.size();++j){
   if(!rollingContractDynamics(states[j-1],states[j],acceleration_))throw std::invalid_argument("RAW_FORWARD_RELATION_INCONSISTENT");
   contracted(states[j],out.explicit_material[j-1],event_,lever_,out.physical_branches);
  }
 }
 const size_t offset=out.material.cutoff-certificate_.separator.epoch;
 if(offset>=states.size())throw std::invalid_argument("SEPARATOR_PROJECTION_MISSING");
 out.current=rollingProject(states.back());out.certificate=certificate_;
 out.certificate.separator=rollingProject(states[offset]);out.certificate.parent=certificate_.generation;
 ++out.certificate.generation;out.certificate.covered_through=tx.proposed_epoch;
 out.certificate.version=tx.base_graph_version+1;out.certificate.source_prefix=event_.prefix(tx.proposed_epoch);
 out.certificate.shared_adc_index=10*out.material.cutoff;
 out.certificate.fault_active=out.certificate.fault_ended_residual=out.certificate.fault_excluded=0;
 out.certificate.fault_not_started=16ULL*(policy_.mission-int(tx.proposed_epoch));
 // UWB task recipe is retained symbolically, not inferred from the selected action.
 out.certificate.fault_active=8ULL*(1+std::min<size_t>(3,tx.proposed_epoch));
 out.certificate.fault_ended_residual=16ULL*tx.proposed_epoch-out.certificate.fault_active;
 const auto now=tx.end;
 for(const auto& c:monitor.candidates()){
  if(!c.possible){++out.certificate.fault_excluded;continue;}
  if(now<c.episode.begin)++out.certificate.fault_not_started;
  else if(now<c.episode.end)++out.certificate.fault_active;
  else ++out.certificate.fault_ended_residual;
 }
 out.raw_read_first=out.explicit_material.front().raw_first;
 out.raw_read_last=out.explicit_material.back().raw_last;
 while(!out.explicit_material.empty() && out.explicit_material.front().batch.id.value()<=out.material.cutoff)out.explicit_material.pop_front();
 return out;
}
RawRangeResult RollingIntegrity::finalize(RollingBoundaryProposalV1&& p,const CommitReceipt& r,
 const std::vector<gtsam::Key>& keys,const NavigationState& state){
 try{
  if(failed_ || p.parent_generation!=certificate_.generation || p.certificate.parent!=certificate_.generation ||
     p.certificate.version!=certificate_.version+1 || p.certificate.event_id!=event_.id ||
     state.timestamp!=p.material.end)throw std::invalid_argument("BOUNDARY_STALE_PROPOSAL");
  finalizeTransfer(p.material,r,keys);
  auto t=rollingTag(p.current,lever_);Eigen::Vector3d actual=state.position_world_m+state.q_world_body*lever_;
  RawRangeResult bound;bound.source="sc5-raw-rolling-full-state-outer-v1";
  bound.relaxation="FULL_TASK_PRODUCT_OUTER_MASK_RELAXATION_NO_PROBABILISTIC_PRUNING";
  for(int j=0;j<3;++j){bound.low(j)=t[j].lower();bound.high(j)=t[j].upper();bound.pl(j)=abs(t[j]-I(actual(j))).upper();}
  bound.hpl=sqrt(square(I(bound.pl.x()))+square(I(bound.pl.y()))).upper();
  bound.valid=bound.pl.allFinite() && actual.allFinite() && std::isfinite(bound.hpl);
  if(!bound.valid)throw std::invalid_argument("COMMITTED_TAG_NONFINITE");
  bound.status="ROLLING_RAW_BOUND_VERIFIED_FOR_FINITE_SCOPE";bound.event_risk=event_.risk_upper;
  certificate_=std::move(p.certificate);explicit_=std::move(p.explicit_material);return bound;
 }catch(...){failed_=true;throw;}
}
}
