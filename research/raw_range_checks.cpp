#include "scenario_material.hpp"
#include <iostream>
#include <boost/multiprecision/cpp_dec_float.hpp>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool v,const char* s){if(!v)throw std::runtime_error(s);}

using MP=boost::multiprecision::cpp_dec_float_100;
MP distance(const Eigen::Vector3d& p,const Eigen::Vector3d& a){MP sum=0;for(int j=0;j<3;++j){MP d=MP(p(j))-MP(a(j));sum+=d*d;}return sqrt(sum);}
bool feasible(const Eigen::Vector3d& p,const UwbBatch& b,const std::array<RawRangeShell,8>& sh){
 for(int omitted:rawRangeFaultMasks()){bool good=true;for(int i=0;i<8;++i)if(i!=omitted){MP d=distance(p,b.measurements[i].anchor_position_m);good=good && d>=MP(sh[i].low) && d<=MP(sh[i].high);}if(good)return true;}return false;
}
void contains(const RawRangeResult& r,const Eigen::Vector3d& p){
 require(MP(r.hpl)>=sqrt(MP(r.pl.x())*MP(r.pl.x())+MP(r.pl.y())*MP(r.pl.y())),"100-digit outward horizontal norm");require(r.valid && (r.low.array()<=p.array()).all() && (r.high.array()>=p.array()).all(),"feasible point lost");}
void geometryChecks(){
 UwbBatch b;std::array<RawRangeShell,8> shells;
 for(int i=0;i<8;++i){UwbMeasurement m;m.anchor_position_m={double(i&1)*8-4,double((i>>1)&1)*8-4,double((i>>2)&1)*8-4};b.measurements.push_back(m);}
 Eigen::Vector3d point(.25,-.5,.75),lo=Eigen::Vector3d::Constant(-20),hi=-lo;
 // Fault at EACH physical source, boundary noise with both signs, weak and
 // zero faults. No selected healthy subset is passed to the kernel.
 for(int fault=-1;fault<8;++fault)for(double sign:{-1.,1.})for(double magnitude:{0.,.02,20.}){
  for(int i=0;i<8;++i){double z=distance(point,b.measurements[i].anchor_position_m).convert_to<double>()+sign*.7+(i==fault?magnitude:0);shells[i]={std::max(0.,std::nextafter(z-.70000000000001,-INFINITY)),std::nextafter(z+.70000000000001,INFINITY)};}
  require(feasible(point,b,shells),"independent 100-digit boundary oracle");
  auto r=contractRawRangeBox(lo,hi,b,shells,Eigen::Vector3d(100,-100,50));contains(r,point);
  require((r.low.array()>=lo.array()).all() && (r.high.array()<=hi.array()).all() && r.hpl<=r.old_hpl && (r.pl.array()<=r.old_pl.array()).all(),"subset and same-center monotonic bound");
  require(r.considered==9 && r.status=="CONTRACTED_VERIFIED","all branches and nontrivial contraction");
  // Budget expires with nonempty frontier, including unvisited fault roots.
  auto budget=contractRawRangeBox(lo,hi,b,shells,point,{1,8,3});contains(budget,point);require(budget.budget_exhausted && budget.frontier>0 && budget.considered==9,"frontier/branch union preserved");
  auto zero=contractRawRangeBox(lo,hi,b,shells,point,{0,8,3});require(zero.low==lo && zero.high==hi && zero.frontier==9,"zero budget retains original");
 }
 // Independent dense lattice oracle checks arbitrary feasible points, rather
 // than only the point used to generate the shell family.
 for(int i=0;i<8;++i){double d=distance(point,b.measurements[i].anchor_position_m).convert_to<double>();shells[i]={d-.7,d+.7};}
 auto r=contractRawRangeBox(lo,hi,b,shells,point,{256,8,2});int checked=0;
 for(int x=-8;x<=8;++x)for(int y=-8;y<=8;++y)for(int z=-8;z<=8;++z){Eigen::Vector3d p(x*.25,y*.25,z*.25);if(feasible(p,b,shells)){contains(r,p);++checked;}}require(checked>10,"nontrivial lattice oracle");
 // Coplanar mirrors: no GT side is selected. Nearly singular slab coefficient
 // and zero distance use literal interval arithmetic, not rank thresholds.
 for(auto& m:b.measurements)m.anchor_position_m.z()=0;
 for(double height:{0.,1e-10,1.}){
  Eigen::Vector3d p(0,0,height),mirror(0,0,-height);
  for(int i=0;i<8;++i){double d=distance(p,b.measurements[i].anchor_position_m).convert_to<double>();shells[i]={std::max(0.,d-1e-12),d+1e-12};}
  auto mirrors=contractRawRangeBox(lo,hi,b,shells,p,{256,8,2});contains(mirrors,p);contains(mirrors,mirror);
 }
 // 1D and 2D analytical upper-shell domains are exact necessary conditions.
 for(int dim:{1,2,3}){for(auto& m:b.measurements)m.anchor_position_m.setZero();for(auto& sh:shells)sh={0,1};Eigen::Vector3d l=Eigen::Vector3d::Zero(),h=l;for(int j=0;j<dim;++j){l(j)=-10;h(j)=10;}
  auto analytic=contractRawRangeBox(l,h,b,shells,point);require(analytic.low.head(dim).minCoeff()==-1 && analytic.high.head(dim).maxCoeff()==1,"analytic unit ball box");contains(analytic,Eigen::Vector3d::Zero());
 }
 for(auto& sh:shells)sh={0,-1};
 auto empty=contractRawRangeBox(lo,hi,b,shells,point);require(!empty.valid && empty.status=="MODEL_INCONSISTENT" && std::isinf(empty.hpl),"empty is not zero PL");
 for(auto& sh:shells)sh={0,1e308};
 auto overflow=contractRawRangeBox(lo,hi,b,shells,point);require(overflow.valid && overflow.status=="FALLBACK_INERTIAL_VERIFIED" && overflow.low==lo,"overflow safe fallback");
 std::cout<<"geometry oracle feasible_lattice="<<checked<<" fault/boundary/mirror/analytic/frontier/overflow passed\n";
}



void numericBoundaryChecks(){
 NoisyMotionScenario s("config/hybrid_raw.yaml","U0",20261010,1);
 for(double sign:{-1.,1.}){
  auto b=s.uwb[0];std::array<double,8> draw{},fault{};
  for(int i=0;i<8;++i){auto& m=b.measurements[i];m.anchor_position_m={0,0,1};draw[i]=sign*7;m.range_m=m.sigma_m*draw[i];}
  auto law=certifyRawRanges(b,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),0,Eigen::Vector3d::Zero(),draw,fault);
  std::array<RawRangeShell,8> sh;require(rawRangeShells(law,b,1,sh),"actual 7sigma zero-distance numeric source");
  auto out=contractRawRangeBox(Eigen::Vector3d::Constant(-2),Eigen::Vector3d::Constant(2),b,sh,Eigen::Vector3d::Zero());contains(out,Eigen::Vector3d(0,0,1));
 }
 auto b=s.uwb[0];std::array<RawRangeShell,8> sh;Eigen::Vector3d p(.1,.2,.3);
 for(int i=0;i<8;++i){b.measurements[i].anchor_position_m={double(i),double(i%2),double(i)*1e-15};double d=distance(p,b.measurements[i].anchor_position_m).convert_to<double>();sh[i]={std::max(0.,d-1e-12),d+1e-12};}
 auto out=contractRawRangeBox(Eigen::Vector3d::Constant(-10),Eigen::Vector3d::Constant(10),b,sh,p);contains(out,p);
 std::cout<<"actual source 7sigma/zero-distance and near-singular checks passed\n";
}
void wrongFdeChecks(){
 NoisyMotionScenario s("config/hybrid_raw.yaml","U1",20261010,16);
 IncrementalUwbImuEstimator backend(s.cfg,s.lever);backend.initialize(s.initial_observation,s.cfg.realtime.prior_sigmas);backend.ingestImu(s.imu[0]);
 RawInclusionParameters param;param.numeric=s.numeric_certificate;param.initial_sigmas=s.cfg.realtime.prior_sigmas;RawNonlinearInclusion inclusion(s.initial_observation,param);inclusion.append(s.imu[0]);
 for(int e=1;e<=16;++e){for(int i=(e-1)*10+1;i<=e*10;++i){backend.ingestImu(s.imu[i]);inclusion.append(s.imu[i]);}
  auto tx=backend.prepareEpoch(s.uwb[e-1]);auto plan=EpochCommitPlan::nominalPlan(tx);
  if(e==15){bool found=false;for(const auto& group:tx.uwb_groups)if(!group.nominal && group.source_measurements.size()==7 && std::find(group.source_measurements.begin(),group.source_measurements.end(),MeasurementId(1502))==group.source_measurements.end()){plan.groups_to_add={tx.imu_group.id,group.id};found=true;break;}require(found,"wrong FDE fixture");}
  auto retained=tx;auto receipt=backend.commitEpoch(std::move(tx),plan);auto state=backend.currentState();auto proof=inclusion.issue(retained,receipt,state,s.lever);
  auto out=issueRawRangeContraction(proof,s.range_certificates[e-1],retained,receipt,state,s.lever);require(out.valid,"wrong FDE independent bound");
  auto truth=s.truth(e*.05);contains(out,truth.position_world_m+truth.q_world_body*s.lever);
 }
 std::cout<<"actual wrong-FDE native commit excludes healthy 1502, retains faulty 1501; coverage/recovery passed\n";
}
void bindingChecks(){
 NoisyMotionScenario s("config/hybrid_raw.yaml","U0",20261010,2);
 IncrementalUwbImuEstimator backend(s.cfg,s.lever);backend.initialize(s.initial_observation,s.cfg.realtime.prior_sigmas);backend.ingestImu(s.imu[0]);
 RawInclusionParameters param;param.numeric=s.numeric_certificate;param.initial_sigmas=s.cfg.realtime.prior_sigmas;
 RawNonlinearInclusion inclusion(s.initial_observation,param);inclusion.append(s.imu[0]);
 for(int e=1;e<=2;++e){for(int i=(e-1)*10+1;i<=e*10;++i){backend.ingestImu(s.imu[i]);inclusion.append(s.imu[i]);}
  auto tx=backend.prepareEpoch(s.uwb[e-1]);auto retained=tx;auto receipt=backend.commitEpoch(std::move(tx),EpochCommitPlan::nominalPlan(retained));auto state=backend.currentState();
  auto proof=inclusion.issue(retained,receipt,state,s.lever);require(proof.valid,"inertial issue");require(MP(proof.hpl)>=sqrt(MP(proof.pl.x())*MP(proof.pl.x())+MP(proof.pl.y())*MP(proof.pl.y())),"100-digit old outward horizontal norm");
  const auto& law=s.range_certificates[e-1];auto result=issueRawRangeContraction(proof,law,retained,receipt,state,s.lever);require(result.valid && result.event_risk==proof.event_risk,"native same event receipt");
  RawRangeBudget healthyOnly;healthyOnly.all_ranges_healthy=true;require(!issueRawRangeContraction(proof,law,retained,receipt,state,s.lever,healthyOnly).valid,"SC1 cannot drop its physical UWB fault masks");
  auto refuse=[&](const RawInclusionEvidence& p,const RawRangeLawEvidence& l,const EpochTransaction& t,const CommitReceipt& r,const NavigationState& a){auto out=issueRawRangeContraction(p,l,t,r,a,s.lever);require(!out.valid && out.status=="EVIDENCE_REFUSED","tamper/stale/missing refused");};
  for(int field=0;field<10;++field){auto t=retained;auto& m=t.uwb_batch.measurements[0];switch(field){case 0:m.range_m+=1e-9;break;case 1:m.id=MeasurementId(999);break;case 2:m.anchor_position_m.x()+=1e-9;break;case 3:m.sigma_m*=.5;break;case 4:m.timestamp=TimestampNs(0);break;case 5:m.anchor_id=AnchorId(999);break;case 6:t.uwb_batch.measurements.pop_back();break;case 7:t.uwb_batch.covariance_model_id="correlated";break;case 8:m.tag_id=TagId(1);break;case 9:m.lever_arm_body_m=s.lever;break;}refuse(proof,law,t,receipt,state);}
  for(int field=0;field<6;++field){auto l=law;switch(field){case 0:l.valid=false;break;case 1:l.frame="body";break;case 2:l.source_law_id="other";break;case 3:l.rho_num[0]=0;break;case 4:l.batch.measurements[0].range_m+=1e-9;break;case 5:l.rho_num[0]=NAN;break;}refuse(proof,l,retained,receipt,state);}
  for(int field=0;field<9;++field){auto p=proof;switch(field){case 0:p.valid=false;break;case 1:p.receipt_bound=false;break;case 2:p.event_risk=0;break;case 3:p.horizon=51;break;case 4:p.low.x()+=.001;break;case 5:p.source_law_id="legacy";break;case 6:p.low.x()=NAN;break;case 7:p.low.x()=p.high.x()+1;break;case 8:p.pl.z()=INFINITY;break;}refuse(p,law,retained,receipt,state);}
  auto r=receipt;r.backend_updates=2;refuse(proof,law,retained,r,state);r=receipt;r.transaction_id=TransactionId(999);refuse(proof,law,retained,r,state);
  auto a=state;a.timestamp=TimestampNs(0);refuse(proof,law,retained,receipt,a);a=state;a.position_world_m.x()+=.01;refuse(proof,law,retained,receipt,a);
  auto expired=proof;expired.epoch=51;refuse(expired,law,retained,receipt,state);
  // The future batch is inaccessible to the consumer; changing it cannot
  // change the current result. Input evidence/batch are value snapshots.
  auto future=s.uwb.back();s.uwb.back().measurements[0].range_m+=100;
  auto again=issueRawRangeContraction(proof,law,retained,receipt,state,s.lever);require(again.low==result.low && again.high==result.high && again.hpl==result.hpl,"future independence");s.uwb.back()=future;
 }
 NoisyMotionScenario bad("config/hybrid_raw.yaml","I2",20261010,20);require(!bad.numeric_certificate.valid,"unsupported IMU cannot inherit range fallback");
 std::cout<<"native evidence/receipt/center/tamper/expiry/future checks passed\n";
}
int main(){try{
 geometryChecks();numericBoundaryChecks();bindingChecks();wrongFdeChecks();
 auto masks=rawRangeFaultMasks();require(masks.size()==9 && masks.front()==-1 && masks.back()==7,"complete masks");
 for(auto kind:{"U0","U1","U2","U3"}){NoisyMotionScenario s("config/hybrid_raw.yaml",kind,20261010,16);
  for(size_t e=0;e<s.uwb.size();++e){std::array<RawRangeShell,8> shell;require(rawRangeShells(s.range_certificates[e],s.uwb[e],e+1,shell),"source numeric certificate/shell");
   auto changed=s.uwb[e];changed.measurements[0].range_m+=1e-8;require(!rawRangeShells(s.range_certificates[e],changed,e+1,shell),"changed raw range refused");
  }
 }
 std::cout<<"raw range source/shell checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
