#include "sc3_joint_scope.hpp"
#include "scenario_material.hpp"
#include "sc3_joint_bound.hpp"
#include "sc3_joint_coordinator.hpp"
#include "gaussian_reference.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include <gtsam/inference/Symbol.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void check(bool p,const char* m){if(!p)throw std::runtime_error(m);}
RawInclusionParameters lawOf(const NoisyMotionScenario& s){RawInclusionParameters p;p.numeric=s.imu_fault_numeric_certificate;p.initial_sigmas=s.cfg.realtime.prior_sigmas;return p;}
void jointBoundChecks(){
 for(const auto* kind:{"SC3_A","SC3_G"}){
  NoisyMotionScenario s("config/sc3_joint.yaml",kind,20261010,16);auto law=lawOf(s);check(law.numeric.valid && !s.numeric_certificate.valid,"joint numeric admission is distinct from healthy-IMU detector");
  Sc2ImuMonitor monitor(s.initial_observation,law);Sc3IndependentJointBound bound(s.initial_observation,law,s.uwb[0]);Sc2IndependentBridge old(s.initial_observation,law);
  IncrementalUwbImuEstimator backend(s.cfg,s.lever);backend.initialize(s.initial_observation,s.cfg.realtime.prior_sigmas);backend.ingestImu(s.imu[0]);monitor.append(s.imu[0]);
  for(int e=1;e<=16;++e){for(int i=(e-1)*10+1;i<=e*10;++i){backend.ingestImu(s.imu[i]);monitor.append(s.imu[i]);}auto tx=backend.prepareEpoch(s.uwb[e-1],EpochPreparationOptions{true,true,true,true});auto plan=EpochCommitPlan::nominalPlan(tx);
   if(std::string(kind)=="SC3_A" && e==14){plan.groups_to_add={tx.generic_bridge_group.id,tx.generic_bias_continuity_group.id,tx.uwb_groups.front().id};plan.bridge_mode=BridgeMode::GenericKinematic;}
   if(std::string(kind)=="SC3_A" && e==15){bool found=false;for(const auto& g:tx.uwb_groups)if(!g.nominal && g.source_measurements.size()==7 && std::find(g.source_measurements.begin(),g.source_measurements.end(),tx.uwb_batch.measurements[1].id)==g.source_measurements.end()){plan.groups_to_add={tx.imu_group.id,g.id};found=true;break;}check(found,"actual wrong UWB action retains true faulty anchor0");}
   auto retained=tx;auto receipt=backend.commitEpoch(std::move(tx),plan);auto state=backend.currentState();
   if(std::string(kind)=="SC3_A" && e==14)check(!receipt.group_to_slots.count(retained.imu_group.id.value()),"wrong healthy IMU deletion actually committed");
   auto out=bound.issue(monitor,s.range_certificates[e-1],retained,receipt,state,s.lever);check(out.valid && out.expected==481401 && out.evaluated==out.expected && out.geometry.considered==9,"entire physical product shares valid current projection recipe");
   if(e==15)std::cout<<"SC3 snapshot "<<kind<<" native wrong-FDE/off epoch15 H/V="<<out.geometry.hpl<<"/"<<out.geometry.pl.z()<<" masks="<<out.geometry.considered<<" product="<<out.evaluated<<"\n";
   auto truth=s.truth(e*.05);Eigen::Vector3d tag=truth.position_world_m+truth.q_world_body*s.lever;check((tag.array()>=out.geometry.low.array()).all() && (tag.array()<=out.geometry.high.array()).all(),"joint wrong/FDE-off output preserves actual tag truth");
   if(e>=15){check(monitor.assessment().alarm,"actual no-label IMU alarm");std::array<RawRangeShell,8> shell;check(rawRangeShells(s.range_certificates[e-1],retained.uwb_batch,e,shell),"actual certified noisy shells");double truthRange=(tag-retained.uwb_batch.measurements[0].anchor_position_m).norm();check(truthRange<shell[0].low || truthRange>shell[0].high,"joint truth violates SC2 all-healthy-UWB shell premise even if its numerical relaxation is finite");
    RawRangeBudget wrong;wrong.all_ranges_healthy=true;check(!bound.issue(monitor,s.range_certificates[e-1],retained,receipt,state,s.lever,wrong).valid,"joint proof cannot silently drop UWB masks");
    RawRangeBudget exhausted;exhausted.max_boxes=0;auto fallback=bound.issue(monitor,s.range_certificates[e-1],retained,receipt,state,s.lever,exhausted);check(fallback.valid && fallback.geometry.budget_exhausted && fallback.evaluated==481401,"all unprocessed joint branches remain via physical frontier");
    auto mutated=retained;mutated.raw_imu_slice.back().specific_force_mps2.x()+=1;check(!bound.issue(monitor,s.range_certificates[e-1],mutated,receipt,state,s.lever).valid,"mutated raw prefix refuses");
    auto foreign=s.range_certificates[e-1];foreign.frame="body";check(!bound.issue(monitor,foreign,retained,receipt,state,s.lever).valid,"foreign range frame refuses");
    auto twice=receipt;twice.backend_updates=2;check(!bound.issue(monitor,s.range_certificates[e-1],retained,twice,state,s.lever).valid,"two-update receipt refuses");
    auto late=state;late.timestamp=TimestampNs(2550000000LL);check(!bound.issue(monitor,s.range_certificates[e-1],retained,receipt,late,s.lever).valid,"expired state refuses");
    check(!old.envelope(51,s.lever).valid,"whole task horizon cannot renew after50");
    auto stale=receipt;stale.graph_version++;check(!bound.issue(monitor,s.range_certificates[e-1],retained,stale,state,s.lever).valid,"foreign receipt version refused");
   }
  }
 }
 std::cout<<"SC3 joint all9 projection/full physical product/polluted FDE-off center/healthy-UWB misuse/frontier/version checks PASS\n";
}
void jointResponseOracle(){
 NoisyMotionScenario s("config/sc3_joint.yaml","SC3_U0",20261010,2);IncrementalUwbImuEstimator backend(s.cfg,s.lever);backend.initialize(s.initial_observation,s.cfg.realtime.prior_sigmas);backend.ingestImu(s.imu[0]);
 for(int i=1;i<=10;++i)backend.ingestImu(s.imu[i]);auto first=backend.prepareEpoch(s.uwb[0]);auto firstSaved=first;backend.commitEpoch(std::move(first),EpochCommitPlan::nominalPlan(firstSaved));
 for(int i=11;i<=20;++i)backend.ingestImu(s.imu[i]);auto tx=backend.prepareEpoch(s.uwb[1]);IntegrityWindowRequest request;request.epochs=2;auto w=backend.buildIntegrityWindow(tx,request);
 check(w.model_valid && w.H.allFinite(),"actual frozen joint model");
 for(int axis=0;axis<6;++axis){RawImuEpisode episode{"shared-oracle",TimestampNs(5000000),TimestampNs(55000000),axis};Eigen::MatrixXd B=Eigen::MatrixXd::Zero(w.H.rows(),2);Eigen::VectorXd observed=Eigen::VectorXd::Zero(w.H.rows());int offset=0;
  for(const auto& block:w.blocks){int n=block.residual_raw.size();
   if(block.group_id==tx.imu_group.id || block.group_id==firstSaved.imu_group.id){auto material=block.group_id==tx.imu_group.id?tx:firstSaved;material.frozen_values=tx.frozen_values;
    auto response=episodeResponse(material,block,episode);B.block(offset,1,n,1)=response.frozen_whitened;
    auto literal=[&](double amplitude){auto samples=material.raw_imu_slice;for(auto& sample:samples)if(!(sample.timestamp<episode.begin) && sample.timestamp<episode.end){if(axis<3)sample.specific_force_mps2(axis)+=amplitude;else sample.angular_velocity_radps(axis-3)+=amplitude;}auto pim=*material.preintegration;pim.resetIntegration();for(size_t j=1;j<samples.size();++j)pim.integrateMeasurement(.5*(samples[j-1].specific_force_mps2+samples[j].specific_force_mps2),.5*(samples[j-1].angular_velocity_radps+samples[j].angular_velocity_radps),.005);return episodeFactorError(material,pim);};
    observed.segment(offset,n)=-(block.whitener*(literal(2e-5)-literal(-2e-5)))/(4e-5);
   }
   if(block.group_id==tx.uwb_groups.front().id)B.block(offset,0,n,1)=block.whitener.col(0);
   offset+=n;
  }
  check(offset==w.H.rows() && (B.col(1)-observed).norm()<2e-6*std::max(1e-12,observed.norm()),"independent literal raw six-axis joint response and shared-PIM coefficient");
  Eigen::MatrixXd QR=w.H.colPivHouseholderQr().solve(B);Eigen::JacobiSVD<Eigen::MatrixXd> svd(w.H,Eigen::ComputeThinU|Eigen::ComputeThinV);Eigen::MatrixXd SVD=svd.solve(B);
  check((w.protected_state_map*(QR-SVD)).norm()<1e-6,"joint response protected QR/SVD oracle");
  auto pool=makeSc3FaultPool(s.uwb[0]);auto boundMode=bindSc3PhysicalEpisode(pool.imu.at(axis*100),tx);check(boundMode.mode.raw_group_maps.size()==2 && boundMode.unit.affected_measurements.size()==10,"lazy episode material carries native two-PIM response/support under shared amplitude");
  auto z=w.z+1e-5*B.col(0)-2e-5*B.col(1);auto qr=w.H.colPivHouseholderQr();Eigen::VectorXd independent=qr.solve(z)-qr.solve(w.z);
  check((w.protected_state_map*(independent-(1e-5*QR.col(0)-2e-5*QR.col(1)))).norm()<1e-8,"two-parameter joint response cannot use max(single slope)");
 }
 backend.discardEpoch(std::move(tx),DiscardReason{FdeStatus::NotTriggered,"read-only joint response oracle",false});
 std::cout<<"SC3 actual native shared-two-PIM six-axis joint amplitude response and independent QR/SVD/difference oracle PASS\n";
}
void nativeJointChecks(){
 for(const auto* kind:{"SC3_A","SC3_G"}){
  NoisyMotionScenario s("config/sc3_joint.yaml",kind,20261010,26);auto law=lawOf(s);Sc3JointCoordinator joint(s.initial_observation,law,s.uwb[0]);IncrementalUwbImuEstimator backend(s.cfg,s.lever);backend.initialize(s.initial_observation,s.cfg.realtime.prior_sigmas);backend.ingestImu(s.imu[0]);joint.append(s.imu[0]);
  for(int e=1;e<=26;++e){for(int i=(e-1)*10+1;i<=e*10;++i){backend.ingestImu(s.imu[i]);joint.append(s.imu[i]);}auto tx=backend.prepareEpoch(s.uwb[e-1],EpochPreparationOptions{true,true,true,true});auto assessment=joint.prepare(tx,s.range_certificates[e-1],s.lever,s.cfg.bridge.generic,true);check(assessment.legal,"actual joint native candidate selection");
   if(e>=15 && e<=17)check(assessment.excluded==0 && assessment.imu.current_bridge && assessment.status=="JOINT_UWB_IMU_BRIDGE","simultaneous real UWB and full IMU support action");
   if(e>=15 && e<=25)check(assessment.expected_actions==18 && assessment.evaluated_actions==18 && assessment.imu.candidate.rebuilds==2 && assessment.imu.current_bridge,"complete joint roots and bounded two rebuilds");
   if(e==26)check(assessment.imu_rejoined && !assessment.imu.current_bridge,"true support-contract native IMU rejoin");
   auto retained=tx;auto plan=sc2CommitPlan(tx,assessment.imu);auto receipt=backend.commitEpoch(std::move(tx),plan);joint.committed(assessment,retained,receipt);auto state=backend.currentState();auto proof=joint.issue(s.range_certificates[e-1],retained,receipt,state,s.lever);check(proof.valid && receipt.backend_updates==1,"joint actual raw receipt issuance");
   for(auto id:assessment.imu.action.groups_to_remove)if(id!=retained.imu_group.id && id!=retained.uwb_groups.front().id)check(backend.factorLedger().activeSlots(id).empty(),"actual historical joint slots still active");
   if(e>=15 && e<=17){check(!receipt.group_to_slots.count(retained.imu_group.id.value()) && !receipt.group_to_slots.count(retained.uwb_groups.front().id.value()),"pending IMU and faulty full UWB group really removed");}
   if(e==15)std::cout<<"SC3 native "<<kind<<" simultaneous commit15 actions18 H/V="<<proof.geometry.hpl<<"/"<<proof.geometry.pl.z()<<" historicalUWB="<<assessment.uwb_history_removed<<"\n";
  }
 }
 std::cout<<"SC3 actual UA/UG native full-support/history removal, shared-tail25 and restored IMU26 PASS\n";
}
void cancellationOracle(){
 // Frozen 1D UWB displacement row + integrated raw acceleration row. Two
 // disjoint amplitudes imitate the protected state exactly, while each local
 // fit passes. This is a rank/detectability oracle, not a raw Gaussian claim.
 Eigen::MatrixXd A(2,1);A<<1,1;Eigen::MatrixXd B=Eigen::MatrixXd::Zero(2,2);B(0,0)=1;B(1,1)=.05*.05/2;
 Eigen::Vector2d theta(2,2/B(1,1));Eigen::Vector2d r=B*theta;
 Eigen::VectorXd fit=A.colPivHouseholderQr().solve(r);Eigen::MatrixXd parity=Eigen::Matrix2d::Identity()-A*(A.transpose()*A).inverse()*A.transpose();
 check((r-A*fit).norm()<1e-12 && std::abs(fit(0))>1.9,"joint dangerous cancellation makes large protected estimate with zero statistic");
 check((parity*B.col(0)).norm()>.1 && (parity*B.col(1)).norm()>1e-5,"each fault alone is detectable in joint frozen model");
 double sigma,condition;int rank;Eigen::MatrixXd G=(A.transpose()*A).inverse()*A.transpose()*B;Eigen::MatrixXd protectedResponse=Eigen::MatrixXd::Zero(3,2);protectedResponse.row(0)=G;
 check(classifyDetectionResponse(parity*B,protectedResponse,1e-10,&sigma,&condition,&rank)==3,"joint dangerous nullspace cannot use independent slopes");
 // Individual one-row sensor fits have zero residual: local PASS is not
 // a removal proof. The frozen joint Gaussian result must remain unbounded.
 Eigen::MatrixXd C=Eigen::MatrixXd::Zero(3,1);C(0,0)=1;auto gaussian=gaussianReference(A,Eigen::Vector2d::Zero(),Eigen::Matrix2d::Identity(),C,{B},1e-6,1e-9);
 check(!gaussian.pl.allFinite(),"dangerous/indeterminate frozen joint PL cannot borrow finite single slopes");
 std::cout<<"SC3 UWB/physical-integrated-accel cancellation local residuals0, joint protected shift2m, rank1/q2 and Gaussian unbounded PASS\n";
}
int main(int argc,char** argv){try{
 if(argc==2 && std::string(argv[1])=="native-only"){nativeJointChecks();return 0;}
 if(argc==2 && std::string(argv[1])=="linear-only"){jointResponseOracle();cancellationOracle();return 0;}
 jointBoundChecks();jointResponseOracle();cancellationOracle();nativeJointChecks();
 NoisyMotionScenario scene("config/sc3_joint.yaml","U0",20261010,1);auto pool=makeSc3FaultPool(scene.uwb[0]);check(pool.complete() && pool.at(0).identity=="NONE","full raw physical census");
 size_t u=0,a=0,g=0,ua=0,ug=0;
 for(size_t n=1;n<pool.size();++n){auto h=pool.at(n);check(h.bound_status=="NOT_CONSUMED" && h.detector_status=="LOCAL_PASS_NEVER_EXCLUDES_PHYSICAL_BRANCH" && h.source_event=="raw-source-rw-v1:E6421-7sigma-50epoch" && h.hypothesis.id.value()==n+1 && h.hypothesis.modes.size()==size_t((h.uwb>=0)+(h.imu>=0)),"bijection/parameter dimension");
  if(h.imu<0)++u;else if(pool.imu[h.imu].unit.axis<3){if(h.uwb>=0)++ua;else ++a;}else{if(h.uwb>=0)++ug;else ++g;}
 }
 check(u==800 && a==300 && g==300 && ua==240000 && ug==240000,"H0/UWB/accel/gyro/joint categories cannot disappear");
 auto corrupted=pool;corrupted.imu.pop_back();check(!corrupted.complete(),"missing consumed family refused");corrupted=pool;corrupted.imu[0].identity=corrupted.imu[1].identity;check(!corrupted.complete(),"duplicate physical identity refused");
 bool actionRefused=false;try{Sc3JointCoordinator limited(scene.initial_observation,lawOf(scene),scene.uwb[0],17);}catch(const std::exception&){actionRefused=true;}check(actionRefused,"action17 rejects entire joint scope");
 bool refused=false;try{makeSc3FaultPool(scene.uwb[0],1399);}catch(const std::exception&){refused=true;}check(refused,"capacity cannot silently truncate requested scope");
 auto h=pool.at(1+800+600);check(h.uwb==0 && h.imu==0 && h.hypothesis.modes.size()==2,"two independent shared physical amplitudes");
 auto episode=pool.imu.at(28);auto closure=sc2SupportClosure({episode.identity,episode.unit.time_begin,episode.unit.time_end,episode.unit.axis},30);check(closure.valid,"upstream raw support mapper consumed");
 std::cout<<"SC3 factored physical census expected/represented=481401, proof-consumed=0; U800 A300 G300 UA240000 UG240000; identity/capacity/support PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
