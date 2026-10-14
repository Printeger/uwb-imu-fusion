#include "rolling_boundary.hpp"
namespace uwb_imu_pl::research {
namespace {
using I=RollingInterval;
bool sameState(const RollingRawState& a,const RollingRawState& b){
 auto equal=[](I x,I y){return x.lower()==y.lower() && x.upper()==y.upper();};
 if(a.epoch!=b.epoch)return false;
 for(int j=0;j<3;++j){if(!equal(a.p[j],b.p[j]) || !equal(a.v[j],b.v[j]))return false;for(int k=0;k<3;++k)if(!equal(a.rotation[j][k],b.rotation[j][k]))return false;}
 for(int j=0;j<6;++j)if(!equal(a.bias[j],b.bias[j]))return false;return true;
}
bool sameCertificate(const RollingBoundaryCertificateV1& a,const RollingBoundaryCertificateV1& b){
 if(!sameState(a.separator,b.separator) || a.generation!=b.generation || a.parent!=b.parent || a.version!=b.version || a.source_prefix!=b.source_prefix || a.covered_through!=b.covered_through || a.shared_adc_index!=b.shared_adc_index || a.event_id!=b.event_id || a.fault_not_started!=b.fault_not_started || a.fault_active!=b.fault_active || a.fault_ended_residual!=b.fault_ended_residual || a.fault_excluded!=b.fault_excluded || a.anchor_ids!=b.anchor_ids)return false;
 for(int j=0;j<8;++j)if(a.anchor_positions[j]!=b.anchor_positions[j])return false;return true;
}
RollingVector contracted(RollingRawState& s,const RollingRawMaterial& m,
 const RollingSourceEvent& e,const Eigen::Vector3d& l,size_t& branches,bool tight){
 std::array<RawRangeShell,8> shells;
 const int epoch=int(m.batch.id.value());
 if(!rawRangeShells(m.range,m.batch,epoch,shells,e.open?std::numeric_limits<int>::max():e.mission,e.radius(epoch)))throw std::invalid_argument("RAW_BATCH_SOURCE_SCOPE_REFUSED");
 auto t=tight?rollingTagSphere(s,l):rollingTag(s,l);Eigen::Vector3d low,high;
 for(int j=0;j<3;++j){low(j)=t[j].lower();high(j)=t[j].upper();}
 RawRangeBudget budget;budget.tight_shell=tight;auto result=contractRawRangeBox(low,high,m.batch,shells,Eigen::Vector3d::Zero(),budget);
 if(!result.valid || result.considered!=9)throw std::invalid_argument("RAW_CONTRACTION_SCOPE_INCONSISTENT");
 branches+=9;
 for(int j=0;j<3;++j)t[j]=I(result.low(j),result.high(j));
 if(!(tight?rollingContractTagSphere(s,t,l):rollingContractTag(s,t,l)))throw std::invalid_argument("RAW_TAG_BODY_INCONSISTENT");
 return t;
}
}
RollingIntegrity::RollingIntegrity(const ResolvedRollingPolicyV1& p,const NavigationState& s,
 const Eigen::Matrix<double,15,1>& sigmas,double ba,double bg,const Eigen::Vector3d& lever,double acceleration,double rate,bool tight):
 policy_(p),event_(p.mission,p.budget,p.open),accel_rw_(ba),gyro_rw_(bg),acceleration_(acceleration),rate_(rate),lever_(lever),tight_(tight){
 certificate_.separator=rollingInitial(s,sigmas,event_.q,1e-10);
 certificate_.event_id=event_.id;certificate_.fault_not_started=28ULL*p.mission;
}
RollingBoundaryProposalV1 RollingIntegrity::propose(const EpochTransaction& tx,
 const EpochCommitPlan& plan,const RawRangeLawEvidence& source,const Sc2ImuMonitor& monitor)const{
 return propose(tx,plan,source,finiteMonitorView(monitor,tx.end));
}
RollingBoundaryProposalV1 RollingIntegrity::propose(const EpochTransaction& tx,
 const EpochCommitPlan& plan,const RawRangeLawEvidence& source,const RollingMonitorView& monitor)const{
 if(failed_ || certificate_.event_id!=event_.id || tx.base_graph_version!=certificate_.version ||
    tx.proposed_epoch!=certificate_.covered_through+1)throw std::invalid_argument("BOUNDARY_PARENT_VERSION_REFUSED");
 event_.prefix(tx.proposed_epoch);
 if(monitor.open!=event_.open || (event_.open && monitor.event_id!=event_.id))throw std::invalid_argument("FINITE_OPEN_CERTIFICATE_MODE_MIX");
 if(!monitor.matches(tx) || (!monitor.state.nominal_possible && !monitor.state.possible))
  throw std::invalid_argument("IMU_TASK_SCOPE_INCONSISTENT");
 RollingBoundaryProposalV1 out;out.material=captureBeforeUpdate(tx,plan,policy_);
 out.parent_generation=certificate_.generation;out.explicit_material=explicit_;
 out.explicit_material.push_back({tx.uwb_batch,source,out.material.raw_first,out.material.raw_last});
 std::vector<RollingRawState> states{certificate_.separator};
 size_t expected=certificate_.separator.epoch+1;
 for(const auto& m:out.explicit_material){
  if(m.batch.id.value()!=expected++)throw std::invalid_argument("EXPLICIT_WINDOW_SOURCE_GAP");
  auto s=rollingPropagate(states.back(),acceleration_,rate_,accel_rw_,gyro_rw_,event_.radius(states.back().epoch+1));
  contracted(s,m,event_,lever_,out.physical_branches,tight_);states.push_back(std::move(s));
 }
 // Registered bounded number of forward/backward necessary-constraint passes.
 for(int pass=0;pass<2;++pass){
  for(size_t j=states.size()-1;j>0;--j)if(!rollingContractDynamics(states[j-1],states[j],acceleration_) || (tight_ && !rollingContractCoupledDynamics(states[j-1],states[j],acceleration_)))throw std::invalid_argument("RAW_BACKWARD_RELATION_INCONSISTENT");
  for(size_t j=1;j<states.size();++j){
   if(!rollingContractDynamics(states[j-1],states[j],acceleration_) || (tight_ && !rollingContractCoupledDynamics(states[j-1],states[j],acceleration_)))throw std::invalid_argument("RAW_FORWARD_RELATION_INCONSISTENT");
   contracted(states[j],out.explicit_material[j-1],event_,lever_,out.physical_branches,tight_);
  }
 }
 const size_t offset=out.material.cutoff-certificate_.separator.epoch;
 if(offset>=states.size())throw std::invalid_argument("SEPARATOR_PROJECTION_MISSING");
 out.current=rollingProject(states.back());out.certificate=certificate_;
 for(size_t j=0;j<8;++j){
  const auto& m=tx.uwb_batch.measurements[j];
  if(certificate_.covered_through && (certificate_.anchor_ids[j]!=m.anchor_id || certificate_.anchor_positions[j]!=m.anchor_position_m))throw std::invalid_argument("TASK_ANCHOR_SOURCE_IDENTITY_CHANGED");
  out.certificate.anchor_ids[j]=m.anchor_id;out.certificate.anchor_positions[j]=m.anchor_position_m;
 }
 out.certificate.separator=rollingProject(states[offset]);out.certificate.parent=certificate_.generation;
 ++out.certificate.generation;out.certificate.covered_through=tx.proposed_epoch;
 out.certificate.version=tx.base_graph_version+1;out.certificate.source_prefix=event_.prefix(tx.proposed_epoch);
 out.certificate.shared_adc_index=10*out.material.cutoff;
 out.certificate.fault_active=out.certificate.fault_ended_residual=out.certificate.fault_excluded=0;
 out.certificate.fault_not_started=event_.open?0:16ULL*(policy_.mission-int(tx.proposed_epoch));
 // UWB task recipe is retained symbolically, not inferred from the selected action.
 out.certificate.fault_active=8ULL*(1+std::min<size_t>(3,tx.proposed_epoch));
 out.certificate.fault_ended_residual=16ULL*tx.proposed_epoch-out.certificate.fault_active;
 out.certificate.fault_excluded+=monitor.excluded;out.certificate.fault_active+=monitor.active;
 out.certificate.fault_not_started+=monitor.not_started;out.certificate.fault_ended_residual+=monitor.ended;
 out.raw_read_first=out.explicit_material.front().raw_first;
 out.raw_read_last=out.explicit_material.back().raw_last;
 while(!out.explicit_material.empty() && out.explicit_material.front().batch.id.value()<=out.material.cutoff)out.explicit_material.pop_front();
 out.seal_=std::make_shared<const RollingBoundaryProposalV1::Seal>(RollingBoundaryProposalV1::Seal{this,out.material,out.certificate,out.current,out.explicit_material,out.parent_generation});
 return out;
}
RawRangeResult RollingIntegrity::finalize(RollingBoundaryProposalV1&& p,const CommitReceipt& r,
 const std::vector<gtsam::Key>& keys,const NavigationState& state){
 try{
  if(!p.seal_ || p.seal_->owner!=this || !sameState(p.current,p.seal_->current) || !sameCertificate(p.certificate,p.seal_->certificate) || p.parent_generation!=p.seal_->parent || p.explicit_material.size()!=p.seal_->explicit_material.size())throw std::invalid_argument("BOUNDARY_NUMERIC_SET_TAMPERED");
  const auto& material=p.seal_->material;
  if(p.material.transaction!=material.transaction || p.material.version!=material.version || p.material.epoch!=material.epoch || p.material.cutoff!=material.cutoff || p.material.begin!=material.begin || p.material.end!=material.end || p.material.pre_keys!=material.pre_keys || p.material.expected_retired!=material.expected_retired || p.material.selected!=material.selected || p.material.raw_first!=material.raw_first || p.material.raw_last!=material.raw_last)throw std::invalid_argument("BOUNDARY_MATERIAL_TAMPERED");
  for(size_t j=0;j<p.explicit_material.size();++j){const auto& a=p.explicit_material[j];const auto& b=p.seal_->explicit_material[j];if(!sameRawBatch(a.batch,b.batch) || !sameRawBatch(a.range.batch,b.range.batch) || a.range.valid!=b.range.valid || a.range.rho_num!=b.range.rho_num || a.range.source_law_id!=b.range.source_law_id || a.range.frame!=b.range.frame || a.raw_first!=b.raw_first || a.raw_last!=b.raw_last)throw std::invalid_argument("BOUNDARY_RAW_WINDOW_TAMPERED");}
  if(failed_ || p.parent_generation!=certificate_.generation || p.certificate.parent!=certificate_.generation ||
     p.certificate.version!=certificate_.version+1 || p.certificate.event_id!=event_.id ||
     state.timestamp!=p.material.end)throw std::invalid_argument("BOUNDARY_STALE_PROPOSAL");
  finalizeTransfer(p.material,r,keys);
  auto t=tight_?rollingTagSphere(p.current,lever_):rollingTag(p.current,lever_);Eigen::Vector3d actual=state.position_world_m+state.q_world_body*lever_;
  RawRangeResult bound;bound.source=tight_?"sc6-coupled-sphere-outer-v1":"sc5-raw-rolling-full-state-outer-v1";
  bound.relaxation="FULL_TASK_PRODUCT_OUTER_MASK_RELAXATION_NO_PROBABILISTIC_PRUNING";
  for(int j=0;j<3;++j){bound.low(j)=t[j].lower();bound.high(j)=t[j].upper();bound.pl(j)=abs(t[j]-I(actual(j))).upper();}
  bound.hpl=sqrt(square(I(bound.pl.x()))+square(I(bound.pl.y()))).upper();
  bound.valid=bound.pl.allFinite() && actual.allFinite() && std::isfinite(bound.hpl);
  if(!bound.valid)throw std::invalid_argument("COMMITTED_TAG_NONFINITE");
  bound.status=event_.open?"TIME_UNIFORM_RAW_BOUND_VERIFIED":"ROLLING_RAW_BOUND_VERIFIED_FOR_FINITE_SCOPE";bound.event_risk=event_.risk(p.material.epoch);
  certificate_=std::move(p.certificate);explicit_=std::move(p.explicit_material);return bound;
 }catch(...){failed_=true;throw;}
}
}
