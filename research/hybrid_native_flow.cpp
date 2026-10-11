#include "hybrid_prediction.hpp"
#include "hybrid_coordinator.hpp"
#include "scenario_material.hpp"
#include <chrono>
#include <iostream>
#include <numeric>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
namespace {
using Clock=std::chrono::steady_clock;
double elapsed(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
void check(bool b,const char* message){if(!b)throw std::runtime_error(message);}
}
void runHybridNative(const std::string& path,bool currentFault){
 NoisyMotionScenario scene(path,"U0",20261010,24);HybridFixedLaw law;
 // Matrices registered once, before drawing noise or observing any data.
 Eigen::Vector3d body(0,0,1),base=body+scene.lever;law.reference_tag=base;
 Eigen::Matrix3d skew;skew<<0,-scene.lever.z(),scene.lever.y(),scene.lever.z(),0,-scene.lever.x(),-scene.lever.y(),scene.lever.x(),0;law.C.leftCols(3)=-skew;
 for(int i=0;i<8;++i)law.H.row(i)=(base-scene.cfg.anchors[i].position_world_m).normalized().transpose()*law.C;
 auto initStart=Clock::now();HybridHistoryProducer history(law);double hInitMs=elapsed(initStart,Clock::now());IncrementalUwbImuEstimator estimator(scene.cfg,scene.lever);
 NavigationState observation;observation.position_world_m=body;
 estimator.initialize(observation,scene.cfg.realtime.prior_sigmas);estimator.ingestImu(sample(scene.cfg,0));
 std::mt19937 rng(20261010);std::normal_distribution<double> normal(0,1);
 auto noise=[&](int n,double sigma){Eigen::VectorXd x(n);for(int i=0;i<n;++i)x(i)=sigma*normal(rng);return x;};
 Eigen::VectorXd trueState=noise(15,law.initial_sigma),endpoint=noise(6,law.endpoint_sigma);
 std::vector<double> full;Eigen::Matrix<double,8,1> costs=Eigen::Matrix<double,8,1>::Zero();
 int finite=0,nonzero=0,exclusions=0,followup=0;double maxHistory=0,maxEta=0,maxError=0,maxShift=0,maxHpl=0;
 for(int e=1;e<=24;++e){auto start=Clock::now();
  Eigen::VectorXd nextEndpoint=noise(6,law.endpoint_sigma);
  trueState=(law.F*trueState+law.L*(endpoint+nextEndpoint)+noise(15,law.process_sigma)).eval();endpoint=nextEndpoint;
  Eigen::VectorXd observed=law.H*trueState+noise(8,law.range_sigma);
  // Exactly one episode, either weak duration3 or current duration1. The
  // producer and selector receive only observations and real retained IDs.
  if(currentFault && e==10)observed(0)+=.5;
  if(!currentFault && e>=3 && e<=5)observed(0)+=.002;
  for(int i=(e-1)*10+1;i<=e*10;++i)estimator.ingestImu(sample(scene.cfg,i));
  auto b=ranges(scene.cfg,e);
  for(int i=0;i<8;++i){b.measurements[i].range_m=(base-b.measurements[i].anchor_position_m).norm()+observed(i);b.measurements[i].sigma_m=law.range_sigma;}
  b.covariance_m2=Eigen::MatrixXd::Identity(8,8)*law.range_sigma*law.range_sigma;
  auto tx=estimator.prepareEpoch(b,EpochPreparationOptions{false,true,false,false});
  if(e==1){OnlineGaussianHistory unavailable(4,3,.01,.001,scene.lever);bool refused=false;try{unavailable.prepare(tx);}catch(const std::runtime_error&){refused=true;}check(refused,"missing frozen material refused before dereference");}
  auto hstart=Clock::now();auto h=history.query();auto hend=Clock::now();
  auto nativePrediction=estimator.researchPreUwbPredictionV1(tx);
  auto p=hybridPredict(law,history,h,e,e-1);auto pend=Clock::now();
  check(nativePrediction.transaction_id==tx.id && nativePrediction.excluded_batch_id==b.id && estimator.backendUpdateCount()==nativePrediction.backend_updates,"native pre-UWB lineage");
  // Same Gaussian WLS kernel used by A, with the registered linear sensor
  // matrix for this fixture. The raw nonlinear A remains in the raw runner.
  std::vector<Eigen::MatrixXd> faults;for(int i=0;i<8;++i)faults.push_back(Eigen::MatrixXd::Identity(8,8).col(i));
  EpochRaim a;a.gaussian=gaussianReference(law.H.middleCols(3,3),observed,b.covariance_m2,Eigen::Matrix3d::Identity(),faults,law.alpha,1e-9);
  a.alarm=a.gaussian.statistic>a.gaussian.threshold;
  auto aend=Clock::now();
  auto result=hybridFixedCurrent(law,history,p,observed,a.alarm);auto bend=Clock::now();
  check(result.status=="FIXED_MODEL_VERIFIED","fixed native decision evidence");
  auto plan=EpochCommitPlan::nominalPlan(tx);
  if(result.excluded>=0){const PendingFactorGroup* selected=nullptr;
   for(const auto& g:tx.uwb_groups)if(!g.nominal && g.source_measurements.size()==result.retained.size()){
    bool match=true;for(int i:result.retained)match=match && std::find(g.source_measurements.begin(),g.source_measurements.end(),b.measurements[i].id)!=g.source_measurements.end();if(match)selected=&g;
   }
   check(selected!=nullptr,"native retained candidate");plan.groups_to_add={tx.imu_group.id,selected->id};plan.fde_status=FdeStatus::SuccessUwbExclusion;
  }
  auto retainedTx=std::make_unique<EpochTransaction>(tx);auto id=tx.id;auto version=tx.base_graph_version;auto updates=estimator.backendUpdateCount();
  check(law.epsilon<=law.budget && result.gaussian.pl.allFinite(),"precommit risk/finite");auto vend=Clock::now();
  auto receipt=estimator.commitEpoch(std::move(tx),plan);auto cend=Clock::now();
  auto actual=estimator.currentState();Eigen::Vector3d tag=actual.position_world_m+actual.q_world_body*scene.lever;
  Eigen::Vector3d reference=base+law.C*(p.mean+result.gaussian.mean);
  Eigen::Vector3d shift=(tag-reference).cwiseAbs(),served=result.gaussian.pl+shift;
  auto finalResult=finalizeHybrid(result.gaussian.pl,reference,actual,scene.lever,receipt,id,e,b.timestamp,version,HybridDomain::FixedModel,law.epsilon,law.budget,scene.cfg.risk_v2.horizontal_alert_limit_m,scene.cfg.risk_v2.vertical_alert_limit_m,elapsed(start,Clock::now()));
  bool final=finalResult.available() && estimator.backendUpdateCount()==updates+1;
  check((finalResult.pl-served).norm()==0,"final scalar recomputation");
  if(e==7){auto forged=receipt;forged.transaction_id=TransactionId(receipt.transaction_id.value()+1);
   auto invalid=finalizeHybrid(result.gaussian.pl,reference,actual,scene.lever,forged,id,e,b.timestamp,version,HybridDomain::FixedModel,law.epsilon,law.budget,2,3,1);check(!invalid.available(),"wrong receipt downgrade");
   auto late=finalizeHybrid(result.gaussian.pl,reference,actual,scene.lever,receipt,id,e,b.timestamp,version,HybridDomain::FixedModel,law.epsilon,law.budget,2,3,51);check(late.available() && !late.timelyAvailable(),"late result downgrade");
   Eigen::Vector3d away=reference+Eigen::Vector3d(10,0,0);auto shifted=finalizeHybrid(result.gaussian.pl,away,actual,scene.lever,receipt,id,e,b.timestamp,version,HybridDomain::FixedModel,law.epsilon,law.budget,2,3,1);check(!shifted.available(),"final center AL downgrade");
   check(estimator.backendUpdateCount()==updates+1,"final downgrade never commits twice");
  }
  check(final,"receipt/final center/AL");auto fend=Clock::now();
  std::vector<uint64_t> actualIds;
  for(int i:result.retained){bool found=false;for(auto group:plan.groups_to_add)for(const auto& entry:estimator.factorLedger().groupEntries(group))if(entry.kind==FactorKind::UwbBatch && entry.lifecycle==FactorLifecycle::Active && std::find(entry.source_measurements.begin(),entry.source_measurements.end(),b.measurements[i].id)!=entry.source_measurements.end())found=true;check(found,"actual ledger provenance");actualIds.push_back(b.measurements[i].id.value());}
  if(e==1){auto absent=receipt;absent.backend_updates=0;bool refused=false;try{advanceNativeHistory(history,*retainedTx,plan,absent,observed,result.retained,actualIds);}catch(const std::exception&){refused=true;}check(refused && history.query().epoch==0,"missing commit cannot advance H");
   auto forgedObservations=observed;forgedObservations(0)+=.1;refused=false;
   try{advanceNativeHistory(history,*retainedTx,plan,receipt,forgedObservations,result.retained,actualIds);}catch(const std::exception&){refused=true;}check(refused && history.query().epoch==0,"H observations bound to actual native factors");
  }
  advanceNativeHistory(history,*retainedTx,plan,receipt,observed,result.retained,actualIds);
  if(e==8){history.invalidate("STALE");check(hybridPredict(law,history,history.query(),9,8).status!="FIXED_MODEL_VERIFIED","native receipt prefix invalidated");history.rebuild();check(history.query().status=="BOUNDED","native actual recipe rebuilt");}
  auto uend=Clock::now();
  ++finite;if(result.history.norm()>0)++nonzero;maxHistory=std::max(maxHistory,result.history.norm());maxEta=std::max(maxEta,result.eta);maxShift=std::max(maxShift,shift.norm());maxHpl=std::max(maxHpl,served.head<2>().norm());
  double error=(tag-(base+law.C*trueState)).norm();maxError=std::max(maxError,error);
  check(((tag-(base+law.C*trueState)).cwiseAbs().array()<=served.array()).all(),"fixed realization error coverage");
  if(result.excluded>=0)++exclusions;
  if(currentFault && e>=11 && e<=14 && final)++followup;
  if(currentFault && e==10)check(result.excluded==0,"fixed current source exclusion");
  // Timed cleanup ends before recording scalar metrics; no C is invoked.
  actualIds.clear();retainedTx.reset();p=HybridPrediction{};h=HybridHistoryEvidence{};result=HybridBound{};auto end=Clock::now();full.push_back(elapsed(start,end));
  costs<<costs(0)+elapsed(hstart,hend),costs(1)+elapsed(hend,pend),costs(2)+elapsed(pend,aend),costs(3)+elapsed(aend,bend),costs(4)+elapsed(bend,vend),costs(5)+elapsed(vend,cend),costs(6)+elapsed(cend,fend),costs(7)+elapsed(fend,end);
 }
 std::sort(full.begin(),full.end());
 std::cout<<"V0 native "<<(currentFault?"current":"weak-history")<<" outputs="<<finite<<" nonzero_H="<<nonzero<<" exclusions="<<exclusions<<" followup="<<followup<<" max_history="<<maxHistory<<" max_eta="<<maxEta<<" max_shift="<<maxShift<<" max_HPL="<<maxHpl<<" max_error="<<maxError<<" full_mean/median/P95/max_ms="<<std::accumulate(full.begin(),full.end(),0.)/full.size()<<"/"<<full[full.size()/2]<<"/"<<full[size_t(.95*(full.size()-1))]<<"/"<<full.back()<<" H/P/A/B/verify/commit/final/update-cleanup_mean_ms="<<(costs/24).transpose()<<" H_init_ms="<<hInitMs<<" H2_rebuilt=1 full_C_calls=0 risk="<<law.epsilon<<" formal=0 raw=UNKNOWN hardware=UNKNOWN\n";
 check(history.query().status=="STALE","task horizon termination");
}
