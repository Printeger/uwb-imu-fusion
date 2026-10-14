#include "rolling_boundary.hpp"
#include "tag_support.hpp"
namespace uwb_imu_pl::research {
namespace {
using I=RollingInterval;
bool sameState(const RollingRawState& a,const RollingRawState& b){
 auto equal=[](I x,I y){return x.lower()==y.lower() && x.upper()==y.upper();};
 if(a.epoch!=b.epoch || a.has_tag_relation!=b.has_tag_relation || a.tag_lever!=b.tag_lever)return false;
 if(a.has_tag_relation)for(int j=0;j<3;++j)if(!equal(a.tag_relation[j],b.tag_relation[j]))return false;
 for(int j=0;j<3;++j){if(!equal(a.p[j],b.p[j]) || !equal(a.v[j],b.v[j]))return false;for(int k=0;k<3;++k)if(!equal(a.rotation[j][k],b.rotation[j][k]))return false;}
 for(int j=0;j<6;++j)if(!equal(a.bias[j],b.bias[j]))return false;return true;
}
bool sameCertificate(const RollingBoundaryCertificateV1& a,const RollingBoundaryCertificateV1& b){
 if(!sameState(a.separator,b.separator) || a.generation!=b.generation || a.parent!=b.parent || a.version!=b.version || a.source_prefix!=b.source_prefix || a.covered_through!=b.covered_through || a.shared_adc_index!=b.shared_adc_index || a.event_id!=b.event_id || a.state_method!=b.state_method || a.fault_not_started!=b.fault_not_started || a.fault_active!=b.fault_active || a.fault_ended_residual!=b.fault_ended_residual || a.fault_excluded!=b.fault_excluded || a.anchor_ids!=b.anchor_ids)return false;
 for(int j=0;j<8;++j)if(a.anchor_positions[j]!=b.anchor_positions[j])return false;return true;
}
RollingVector contracted(RollingRawState& s,const RollingRawMaterial& m,
 const RollingSourceEvent& e,const Eigen::Vector3d& l,size_t& branches,bool tight,bool optimized,std::vector<AttributionSnapshot>* audit=nullptr){
 std::array<RawRangeShell,8> shells;
 const int epoch=int(m.batch.id.value());
 if(!rawRangeShells(m.range,m.batch,epoch,shells,e.open?std::numeric_limits<int>::max():e.mission,e.radius(epoch)))throw std::invalid_argument("RAW_BATCH_SOURCE_SCOPE_REFUSED");
 auto t=tight?rollingTagSphere(s,l):rollingTag(s,l);Eigen::Vector3d low,high;
 for(int j=0;j<3;++j){low(j)=t[j].lower();high(j)=t[j].upper();}
 RawRangeBudget budget;budget.retain_support=audit;budget.tight_shell=tight;budget.optimized=optimized;auto result=contractRawRangeBox(low,high,m.batch,shells,Eigen::Vector3d::Zero(),budget);
 if(!result.valid || result.considered!=9)throw std::invalid_argument("RAW_CONTRACTION_SCOPE_INCONSISTENT");
 branches+=9;
 if(audit){for(const auto& leaf:result.support)audit->push_back({leaf.frontier?"range_frontier":"range_leaf",s.epoch,leaf.omitted,leaf.low,leaf.high});audit->push_back({"range_hull",s.epoch,-2,result.low,result.high});}
 for(int j=0;j<3;++j)t[j]=I(result.low(j),result.high(j));
 if(!(tight?rollingContractTagSphere(s,t,l):rollingContractTag(s,t,l)))throw std::invalid_argument("RAW_TAG_BODY_INCONSISTENT");
 return t;
}
}
RollingIntegrity::RollingIntegrity(const ResolvedRollingPolicyV1& p,const NavigationState& s,
 const Eigen::Matrix<double,15,1>& sigmas,double ba,double bg,const Eigen::Vector3d& lever,double acceleration,double rate,bool tight,bool optimized):
 policy_(p),event_(p.mission,p.budget,p.open),accel_rw_(ba),gyro_rw_(bg),acceleration_(acceleration),rate_(rate),lever_(lever),tight_(tight),optimized_(optimized){
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
 out.parent_generation=certificate_.generation;
 const bool capture=attribute_ && (tx.proposed_epoch==1 || tx.proposed_epoch==31 || tx.proposed_epoch==61 || tx.proposed_epoch==250 || tx.proposed_epoch==300);
 auto snapshot=[&](const char* stage,const RollingRawState& s){if(!capture)return;auto t=tight_?rollingTagSphere(s,lever_):rollingTag(s,lever_);AttributionSnapshot a;a.stage=stage;a.epoch=s.epoch;for(int j=0;j<3;++j){a.low(j)=t[j].lower();a.high(j)=t[j].upper();}out.attribution.push_back(a);};
 snapshot("parent_separator",certificate_.separator);out.explicit_material=explicit_;
 out.explicit_material.push_back({tx.uwb_batch,source,out.material.raw_first,out.material.raw_last,tx.raw_imu_slice});
 std::vector<RollingRawState> states{certificate_.separator};
 size_t expected=certificate_.separator.epoch+1;
 for(const auto& m:out.explicit_material){
  if(m.batch.id.value()!=expected++)throw std::invalid_argument("EXPLICIT_WINDOW_SOURCE_GAP");
  auto s=rollingPropagate(states.back(),acceleration_,rate_,accel_rw_,gyro_rw_,event_.radius(states.back().epoch+1));
  snapshot("propagated",s);contracted(s,m,event_,lever_,out.physical_branches,tight_,optimized_,capture?&out.attribution:nullptr);snapshot("tag_lever_contracted",s);states.push_back(std::move(s));
 }
 // Registered bounded number of forward/backward necessary-constraint passes.
 for(int pass=0;pass<2;++pass){
  for(size_t j=states.size()-1;j>0;--j)if(!rollingContractDynamics(states[j-1],states[j],acceleration_) || (tight_ && !rollingContractCoupledDynamics(states[j-1],states[j],acceleration_)))throw std::invalid_argument("RAW_BACKWARD_RELATION_INCONSISTENT");
  for(size_t j=1;j<states.size();++j){
   if(!rollingContractDynamics(states[j-1],states[j],acceleration_) || (tight_ && !rollingContractCoupledDynamics(states[j-1],states[j],acceleration_)))throw std::invalid_argument("RAW_FORWARD_RELATION_INCONSISTENT");
   snapshot("time_contracted",states[j]);if(joint_ && pass>=2-int(joint_sweeps_))out.joint=jointContract(states[j-1],states[j],out.explicit_material[j-1].imu,joint_law_,TimeUniformEvent(policy_.budget));
   contracted(states[j],out.explicit_material[j-1],event_,lever_,out.physical_branches,tight_,optimized_,capture?&out.attribution:nullptr);snapshot("tag_lever_contracted",states[j]);
  }
 }
 // Current tag support is retained through actual-center binding. Its
 // complete branch/frontier union also contracts the separator tag hull.
 if(support_){const auto& m=out.explicit_material.back();std::array<RawRangeShell,8> shells;if(!rawRangeShells(m.range,m.batch,int(m.batch.id.value()),shells,event_.open?std::numeric_limits<int>::max():event_.mission,event_.radius(m.batch.id.value())))throw std::invalid_argument("SUPPORT_SOURCE_SCOPE_REFUSED");
  auto tag=tight_?rollingTagSphere(states.back(),lever_):rollingTag(states.back(),lever_);Eigen::Vector3d low,high;for(int j=0;j<3;++j){low(j)=tag[j].lower();high(j)=tag[j].upper();}RawRangeBudget budget;budget.tight_shell=true;budget.optimized=optimized_;budget.retain_support=true;budget.lower_shell=true;budget.split_depth=2;budget.max_boxes=support_budget_;
  out.geometry=contractRawRangeBox(low,high,m.batch,shells,Eigen::Vector3d::Zero(),budget);
  if(!out.geometry.valid)throw std::invalid_argument("SUPPORT_ALL_BRANCHES_INCONSISTENT");
  for(int j=0;j<3;++j)tag[j]=I(out.geometry.low(j),out.geometry.high(j));if(!(tight_?rollingContractTagSphere(states.back(),tag,lever_):rollingContractTag(states.back(),tag,lever_)))throw std::invalid_argument("SUPPORT_STATE_INTERSECTION_EMPTY");
 }
 const size_t offset=out.material.cutoff-certificate_.separator.epoch;
 if(offset>=states.size())throw std::invalid_argument("SEPARATOR_PROJECTION_MISSING");
 snapshot("before_projection",states.back());out.current=rollingProject(states.back());snapshot("after_projection",out.current);snapshot("next_propagation",rollingPropagate(out.current,acceleration_,rate_,accel_rw_,gyro_rw_,event_.radius(tx.proposed_epoch+1)));out.certificate=certificate_;
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
 out.seal_=std::make_shared<const RollingBoundaryProposalV1::Seal>(RollingBoundaryProposalV1::Seal{this,out.material,out.certificate,out.current,out.explicit_material,out.joint,out.geometry,out.parent_generation});
 return out;
}
void RollingIntegrity::enableJoint(const ImuMeasurement& first,const JointRawLaw& law,unsigned sweeps){
 if(sweeps<1 || sweeps>2)throw std::invalid_argument("JOINT_SCHEDULE_UNREGISTERED");
 if(certificate_.generation || certificate_.separator.epoch || joint_)throw std::invalid_argument("JOINT_MUST_START_FROM_ORIGINAL_INITIAL_SCOPE");
 auto state=certificate_.separator;
 if(!jointInitial(state,first,law,TimeUniformEvent(policy_.budget)))throw std::invalid_argument("JOINT_INITIAL_INCONSISTENT");
 state.tag_relation=rollingTagSphere(state,lever_);state.tag_lever=lever_;state.has_tag_relation=true;
 certificate_.separator=rollingProject(state);certificate_.state_method="sc7-joint-imu-tag-relation-v1";joint_law_=law;joint_sweeps_=sweeps;joint_=true;
}
Eigen::Vector3d RollingIntegrity::committedTag(const NavigationState& state)const{return state.position_world_m+state.q_world_body*lever_;}
RawRangeResult RollingIntegrity::finalize(RollingBoundaryProposalV1&& p,const CommitReceipt& r,
 const std::vector<gtsam::Key>& keys,const NavigationState& state){
 try{
  if(!p.seal_ || p.seal_->owner!=this || !sameState(p.current,p.seal_->current) || !sameCertificate(p.certificate,p.seal_->certificate) || p.parent_generation!=p.seal_->parent || p.explicit_material.size()!=p.seal_->explicit_material.size())throw std::invalid_argument("BOUNDARY_NUMERIC_SET_TAMPERED");
  if(support_){const auto& a=p.geometry;const auto& b=p.seal_->geometry;if(a.valid!=b.valid || a.bound_method!=b.bound_method || a.low!=b.low || a.high!=b.high || a.input_low!=b.input_low || a.input_high!=b.input_high || a.considered!=b.considered || a.frontier!=b.frontier || a.support.size()!=b.support.size())throw std::invalid_argument("TAG_SUPPORT_SEAL_TAMPERED");for(size_t i=0;i<a.support.size();++i)if(a.support[i].low!=b.support[i].low || a.support[i].high!=b.support[i].high || a.support[i].omitted!=b.support[i].omitted || a.support[i].frontier!=b.support[i].frontier)throw std::invalid_argument("TAG_SUPPORT_SEAL_TAMPERED");}
  const auto& material=p.seal_->material;
  if(p.material.transaction!=material.transaction || p.material.version!=material.version || p.material.epoch!=material.epoch || p.material.cutoff!=material.cutoff || p.material.begin!=material.begin || p.material.end!=material.end || p.material.pre_keys!=material.pre_keys || p.material.expected_retired!=material.expected_retired || p.material.selected!=material.selected || p.material.raw_first!=material.raw_first || p.material.raw_last!=material.raw_last)throw std::invalid_argument("BOUNDARY_MATERIAL_TAMPERED");
  for(size_t j=0;j<p.explicit_material.size();++j){const auto& a=p.explicit_material[j];const auto& b=p.seal_->explicit_material[j];if(a.imu.size()!=b.imu.size())throw std::invalid_argument("JOINT_RAW_SOURCE_TAMPERED");for(size_t k=0;k<a.imu.size();++k)if(a.imu[k].id!=b.imu[k].id || a.imu[k].timestamp!=b.imu[k].timestamp || a.imu[k].specific_force_mps2!=b.imu[k].specific_force_mps2 || a.imu[k].angular_velocity_radps!=b.imu[k].angular_velocity_radps)throw std::invalid_argument("JOINT_RAW_SOURCE_TAMPERED");if(!sameRawBatch(a.batch,b.batch) || !sameRawBatch(a.range.batch,b.range.batch) || a.range.valid!=b.range.valid || a.range.rho_num!=b.range.rho_num || a.range.source_law_id!=b.range.source_law_id || a.range.frame!=b.range.frame || a.raw_first!=b.raw_first || a.raw_last!=b.raw_last)throw std::invalid_argument("BOUNDARY_RAW_WINDOW_TAMPERED");}
  if(failed_ || p.parent_generation!=certificate_.generation || p.certificate.parent!=certificate_.generation ||
     p.certificate.version!=certificate_.version+1 || p.certificate.event_id!=event_.id ||
     state.timestamp!=p.material.end)throw std::invalid_argument("BOUNDARY_STALE_PROPOSAL");
  finalizeTransfer(p.material,r,keys);
  auto t=tight_?rollingTagSphere(p.current,lever_):rollingTag(p.current,lever_);Eigen::Vector3d actual=committedTag(state);
  RawRangeResult bound;bound.source=joint_?"sc7-joint-imu-tag-relation-box-v1":tight_?"sc6-coupled-sphere-outer-v1":"sc5-raw-rolling-full-state-outer-v1";
  bound.bound_method=joint_?"sc7-joint-imu-tag-box-v1":"coordinate-box-v1";bound.attribution=std::move(p.attribution);bound.joint=p.seal_->joint;
  bound.relaxation="FULL_TASK_PRODUCT_OUTER_MASK_RELAXATION_NO_PROBABILISTIC_PRUNING";
  for(int j=0;j<3;++j){bound.low(j)=t[j].lower();bound.high(j)=t[j].upper();bound.pl(j)=abs(t[j]-I(actual(j))).upper();}
  bound.hpl=sqrt(square(I(bound.pl.x()))+square(I(bound.pl.y()))).upper();
  if(support_ && !p.seal_->geometry.support.empty()){auto branch=issueTagSupport(p.seal_->geometry,t,actual,p.seal_->explicit_material.back().batch,r,p.parent_generation,event_.id,event_.prefix(p.material.epoch),event_.radius(p.material.epoch));branch.relaxation=bound.relaxation;branch.attribution=std::move(bound.attribution);branch.joint=bound.joint;bound=std::move(branch);}
  bound.valid=bound.pl.allFinite() && actual.allFinite() && std::isfinite(bound.hpl);
  if(!bound.valid)throw std::invalid_argument("COMMITTED_TAG_NONFINITE");
  bound.status=event_.open?"TIME_UNIFORM_RAW_BOUND_VERIFIED":"ROLLING_RAW_BOUND_VERIFIED_FOR_FINITE_SCOPE";bound.event_risk=event_.risk(p.material.epoch);
  certificate_=std::move(p.certificate);explicit_=std::move(p.explicit_material);return bound;
 }catch(...){failed_=true;throw;}
}
}
