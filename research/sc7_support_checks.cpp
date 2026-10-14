#include "tag_support.hpp"
#include "sc6_service.hpp"
#include <tbb/global_control.h>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;using I=RollingInterval;
void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
template<class F>void refuses(F f){bool refused=false;try{f();}catch(...){refused=true;}require(refused,"expected seal/receipt refusal");}
int main(){try{
 tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 Sc6Provider src("config/sc7.yaml","S0",20261010,40);auto envelope=src.certify(1);auto policy=resolveRollingPolicy(src.law,src.cfg);TimeUniformEvent event(policy.budget);
 // The joint contractor consumes global time-uniform birth q. A finite
 // event certificate cannot silently borrow its narrower initial q.
 auto finite_policy=policy;finite_policy.open=false;finite_policy.mission=200;RollingIntegrity finite_proof(finite_policy,src.initial,src.cfg.realtime.prior_sigmas,.001,.0001,src.lever,.04,.2,true,true);JointRawLaw finite_law;finite_law.gravity=src.cfg.imu.gravity_mps2;bool scope_refused=false;try{finite_proof.enableJoint(src.imu[0],finite_law);}catch(const std::invalid_argument& error){scope_refused=std::string(error.what())=="JOINT_REQUIRES_TIME_UNIFORM_EVENT";}require(scope_refused && finite_proof.certificate().generation==0,"finite/time-uniform joint scope mixed");
 CommitReceipt receipt;receipt.transaction_id=TransactionId(1);receipt.graph_version=2;receipt.committed_epoch=1;receipt.state_timestamp=TimestampNs(50000000);receipt.backend_updates=1;
 RawRangeResult geometry;geometry.valid=true;geometry.input_low={0,0,-1};geometry.input_high={3,4,2};geometry.support={{{3,0,-1},{3,0,0},-1,false},{{0,4,1},{0,4,2},0,true}};RollingVector tag{I(0,3),I(0,4),I(-1,2)};
 auto b=issueTagSupport(geometry,tag,Eigen::Vector3d::Zero(),envelope.uwb,receipt,0,event.id(),149,event.block(1).q);require(b.hpl>=4 && b.hpl<4+1e-12 && b.pl.z()==2 && b.old_hpl>=5,"branch H3/4/5 or frontier coverage");
 auto verify=[&](const RawRangeResult& a,const Eigen::Vector3d& center,const CommitReceipt& r,uint64_t parent,const std::string& eid,double q){return verifyTagSupport(a,center,envelope.uwb,r,parent,eid,149,q);};
 require(verify(b,Eigen::Vector3d::Zero(),receipt,0,event.id(),event.block(1).q),"valid support refused");
 require(!verify(b,Eigen::Vector3d(1,0,0),receipt,0,event.id(),event.block(1).q),"center swap accepted");auto forged=receipt;forged.graph_version++;require(!verify(b,Eigen::Vector3d::Zero(),forged,0,event.id(),event.block(1).q),"receipt swap accepted");require(!verify(b,Eigen::Vector3d::Zero(),receipt,1,event.id(),event.block(1).q),"parent swap accepted");require(!verify(b,Eigen::Vector3d::Zero(),receipt,0,"wrong",event.block(1).q),"event swap accepted");require(!verify(b,Eigen::Vector3d::Zero(),receipt,0,event.id(),event.block(1).q-1),"q swap accepted");
 for(int mode=0;mode<4;++mode){auto changed=b;if(mode==0)changed.hpl*=.9;if(mode==1)changed.support.pop_back();if(mode==2)changed.bound_method="coordinate-box-v1";if(mode==3)changed.input_high.x()+=1;require(!verify(changed,Eigen::Vector3d::Zero(),receipt,0,event.id(),event.block(1).q),"support method/number/box tamper accepted");}
 auto moved=issueTagSupport(geometry,tag,Eigen::Vector3d(1,-2,.3),envelope.uwb,receipt,0,event.id(),149,event.block(1).q);require(moved.hpl>6 && moved.hpl<=std::hypot(2.,6.)+1e-12 && moved.pl.z()>=1.7,"bound reused origin radius at new center");
 // Full native proposals: sealed branch/IMU/relation mutation must fail after
 // the sole actual backend mutation, never trigger a second commit.
 RawInclusionParameters rawlaw;rawlaw.initial_sigmas=src.cfg.realtime.prior_sigmas;rawlaw.numeric.valid=true;rawlaw.accel_adc=.02;rawlaw.gyro_adc=.001;rawlaw.accel_rw=.001;rawlaw.gyro_rw=.0001;rawlaw.gravity=src.cfg.imu.gravity_mps2;
 for(int tamper=0;tamper<4;++tamper){IncrementalUwbImuEstimator backend(src.cfg,src.lever);backend.initialize(src.initial,src.cfg.realtime.prior_sigmas);OpenFaultPool pool(src.initial,rawlaw,policy.budget,4096,src.domain.acceleration,src.domain.rate);for(int i=0;i<=10;++i){backend.ingestImu(src.imu[i]);pool.append(src.imu[i]);}
  RollingIntegrity proof(policy,src.initial,src.cfg.realtime.prior_sigmas,.001,.0001,src.lever,src.domain.acceleration,src.domain.rate,true,true);JointRawLaw l;l.gravity=src.cfg.imu.gravity_mps2;proof.enableJoint(src.imu[0],l,1);proof.enableTagSupport(tamper==3?0:256);
  auto tx=backend.prepareEpoch(envelope.uwb);auto plan=EpochCommitPlan::nominalPlan(tx);auto p=proof.propose(tx,plan,envelope.range,pool.view());require(p.geometry.considered==9 && !p.geometry.support.empty(),"incomplete raw branch census");
  if(tamper==0)p.geometry.support.pop_back();if(tamper==1)p.explicit_material.back().imu[4].specific_force_mps2.x()+=1;if(tamper==2)p.current.tag_relation[0]=I(0);
  auto r=backend.commitEpoch(std::move(tx),plan);if(tamper<3)refuses([&]{proof.finalize(std::move(p),r,backend.retainedKeysV1(),backend.currentState());});else{auto out=proof.finalize(std::move(p),r,backend.retainedKeysV1(),backend.currentState());require(out.valid && out.frontier==9 && out.budget_exhausted,"budget0 lost original9 frontiers");}
  require(backend.backendUpdateCount()==2,"proof failure caused second backend commit");
 }
 Sc6Service service(src.law,src.cfg,src.initial,src.lever,src.domain,src.imu.front(),true,false,true);int h=0,v=0;
 for(int k=1;k<=40;++k){auto r=service.step(src.certify(k),true);require(r.q==event.block(k).q && r.shared_q==event.adcRadius((k-1)*10) && r.active_states==size_t(std::min(k+1,30)) && r.replay<=size_t(10*policy.replay+1) && r.capsules<=size_t(policy.replay+1),"native resource/source ABI mismatch");require(r.bound.valid && r.bound.bound_method=="sc7-branch-box-union-tag-v1" && r.bound.tag_support && r.bound.tag_support->center==r.tag,"main actual-center support unbound");require(r.bound.hpl<=r.bound.old_hpl && r.bound.pl.z()<=r.bound.old_pl.z(),"same-parent/center support widened H or V");h+=r.bound.hpl<r.bound.old_hpl-1e-9;v+=r.bound.pl.z()<r.bound.old_pl.z()-1e-9;}
 require(h>0 && v>0 && service.updateCount()==41,"no native strict H/V gain or repeated commit");
 std::cout<<"SC7 K1/K2/K7/K8/K9 support PASS actual40 strict H="<<h<<" V="<<v<<"; center/method/source/q/receipt/parent tamper; all-frontier budget0; no second native commit\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
