#include "fused_epoch_pl.hpp"
#include "scenario_material.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
int main(int argc,char** argv){try{
 if(argc<5)throw std::runtime_error("usage: epoch_raim_runner CONFIG CSV SCENE uwb_epoch_raim|fused_epoch_pl [on|off] [epochs<=50] [history-invalid]");
 std::string algorithm=argv[4],kind=argv[3];bool isB=algorithm=="fused_epoch_pl",fde=argc<6 || std::string(argv[5])=="on";
 if(!isB && algorithm!="uwb_epoch_raim")throw std::runtime_error("unsupported algorithm");
 NoisyMotionScenario scene(argv[1],kind,20261010,argc>6?std::stoi(argv[6]):50);
 double alpha=scene.law["alpha_per_test"].as<double>(),tail=scene.law["noise_tail_per_candidate"].as<double>();
 double risk=2*(scene.cfg.anchors.size()+1)*scene.epochs*tail;
 if(risk>scene.law["risk_budget"].as<double>())throw std::runtime_error("fixed candidate union exceeds budget");
 IncrementalUwbImuEstimator estimator(scene.cfg,scene.lever);
 estimator.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);estimator.ingestImu(scene.imu[0]);
 std::ofstream out(argv[2]);out<<std::setprecision(17);
 out<<"epoch,algorithm,label,status,alarm,excluded,correct,healthy_excluded,retained,updates,available,statistic,threshold,dof,hpl,vpl,error,increment_error,translation_rpe,rotation_rpe,core_ms,prediction_ms,commit_ms,full_ms,candidates,rows,cols,center_shift,union_charge,tag_x,tag_y,tag_z,timestamp_ns,formal_eligible,publication_protected,raw_total_risk,pure_uwb_pl,post_statistic,post_dof,excluded_anchor,excluded_measurement,max_slope_x,max_slope_y,max_slope_z,reference_x,reference_y,reference_z,horizontal_al,vertical_al,within_al,transaction_id,prediction_version,protection_frame,prior_independence,history_condition,model_version,linearization_tag_x,linearization_tag_y,linearization_tag_z\n";
 Eigen::Vector3d lastTag=Eigen::Vector3d::Zero(),lastTruth=Eigen::Vector3d::Zero();NavigationState lastState,lastGt;
 bool quarantined=argc>7 && std::string(argv[7])=="history-invalid";
 for(int e=1;e<=scene.epochs;++e){
  auto start=Clock::now();for(int i=(e-1)*10+1;i<=e*10;++i)estimator.ingestImu(scene.imu[i]);
  auto b=scene.uwb[e-1];EpochPreparationOptions options{false,true,false,false};auto tx=estimator.prepareEpoch(b,options);
  auto predStart=Clock::now();ResearchEpochPredictionV1 pred;
  if(isB)pred=estimator.researchPreUwbPredictionV1(tx);
  auto predEnd=Clock::now();
  // Explicit conditional frozen research law. I2 is a known unsupported test
  // supplied by the scenario law, not a GT label used to choose an exclusion.
  PredictionConditionsV1 conditions{true,true,!quarantined,kind!="I2" && kind!="I1",true};
  auto coreStart=Clock::now();auto r=isB?fusedEpochPl(b,pred,scene.lever,conditions,fde,alpha,tail):uwbEpochRaim(b,fde,alpha,tail);auto coreEnd=Clock::now();
  auto plan=EpochCommitPlan::nominalPlan(tx);
  if(r.excluded>=0){
   const PendingFactorGroup* selected=nullptr;
   for(const auto& group:tx.uwb_groups)if(!group.nominal && group.source_measurements.size()==r.retained.size()){
    bool match=true;for(int i:r.retained)match=match && std::find(group.source_measurements.begin(),group.source_measurements.end(),b.measurements[i].id)!=group.source_measurements.end();
    if(match){selected=&group;break;}
   }
   if(!selected)throw std::runtime_error("selected raw range subset absent from native transaction");
   plan.groups_to_add={tx.imu_group.id,selected->id};plan.fde_status=FdeStatus::SuccessUwbExclusion;
  }
  if(isB && r.alarm && r.excluded<0)quarantined=true;
  auto transactionId=tx.id.value();auto predictionVersion=tx.base_graph_version;
  auto before=estimator.backendUpdateCount();auto receipt=estimator.commitEpoch(std::move(tx),plan);auto state=estimator.currentState();
  if(receipt.backend_updates!=1 || estimator.backendUpdateCount()!=before+1)throw std::runtime_error("not exactly one backend update");
  Eigen::Vector3d fusedTag=state.position_world_m+state.q_world_body*scene.lever;
  Eigen::Vector3d tag=isB?fusedTag:r.tag,pl=r.gaussian.pl;
  double shift=isB && r.tag.allFinite()?(fusedTag-r.tag).norm():0;
  bool available=r.status=="FROZEN_MODEL_ONLY";
  if(isB && available){pl+=(fusedTag-r.tag).cwiseAbs();if(shift>.25){available=false;r.status="COMMITTED_CENTER_DOMAIN_FAILED";pl.setConstant(INFINITY);}}
  auto gt=scene.truth(e*scene.dt);Eigen::Vector3d truth=gt.position_world_m+gt.q_world_body*scene.lever;
  double error=tag.allFinite()?(tag-truth).norm():NAN,inc=e>1 && tag.allFinite() && lastTag.allFinite()?((tag-lastTag)-(truth-lastTruth)).norm():NAN;
  double tr=NAN,rot=NAN;
  if(e>1){tr=((lastState.q_world_body.conjugate()*(state.position_world_m-lastState.position_world_m))-(lastGt.q_world_body.conjugate()*(gt.position_world_m-lastGt.position_world_m))).norm();rot=Eigen::AngleAxisd((lastGt.q_world_body.conjugate()*gt.q_world_body).conjugate()*(lastState.q_world_body.conjugate()*state.q_world_body)).angle();}
  int fault=scene.law["fault_epoch"].as<int>(),stop=fault+(kind=="U2"?scene.law["persistent_epochs"].as<int>():1);
  bool injected=(kind=="U1" || kind=="U2") && e>=fault && e<stop;
  bool correct=injected && r.excluded==scene.law["fault_anchor_index"].as<int>();
  auto end=Clock::now();
  out<<e<<','<<algorithm<<','<<(isB?"FUSED_EPOCH_FROZEN_GAUSSIAN":"UWB_EPOCH_RAIM_CONDITIONAL")<<','<<r.status<<','<<r.alarm<<','<<r.excluded<<','<<correct<<','<<(r.excluded>=0 && !correct)<<','<<r.retained.size()<<','<<receipt.backend_updates<<','<<available<<','<<r.detection_statistic<<','<<r.detection_threshold<<','<<r.detection_dof<<','<<pl.head<2>().norm()<<','<<pl.z()<<','<<error<<','<<inc<<','<<tr<<','<<rot<<','<<ms(coreStart,coreEnd)<<','<<ms(predStart,predEnd)<<','<<estimator.lastUwbUpdateMs()<<','<<ms(start,end)<<','<<r.candidates<<','<<(isB?15:0)+r.retained.size()<<','<<(isB?15:3)<<','<<shift<<','<<risk<<','<<tag.x()<<','<<tag.y()<<','<<tag.z()<<','<<state.timestamp.value()<<",0,0,UNKNOWN,"<<!isB<<','<<r.gaussian.statistic<<','<<r.gaussian.dof<<','<<(r.excluded>=0?b.measurements[r.excluded].anchor_id.value():0)<<','<<(r.excluded>=0?b.measurements[r.excluded].id.value():0);
  Eigen::Vector3d slopes=Eigen::Vector3d::Constant(INFINITY);if(r.slope.cols())slopes=r.slope.rowwise().maxCoeff();
  out<<','<<slopes.x()<<','<<slopes.y()<<','<<slopes.z()<<','<<r.tag.x()<<','<<r.tag.y()<<','<<r.tag.z()<<','<<scene.cfg.risk_v2.horizontal_alert_limit_m<<','<<scene.cfg.risk_v2.vertical_alert_limit_m<<','<<(available && pl.head<2>().norm()<=scene.cfg.risk_v2.horizontal_alert_limit_m && pl.z()<=scene.cfg.risk_v2.vertical_alert_limit_m)<<','<<transactionId<<','<<predictionVersion<<",world,"<<(isB?"FIXED_MODEL_ASSUMED":"NOT_USED")<<','<<(isB?(!conditions.clean_history?"INVALID":"CONDITION_ASSUMED"):"NOT_USED")<<','<<(isB?"epoch-fused-gaussian-v1":"epoch-uwb-slope-v1");
  Eigen::Vector3d linearization=Eigen::Vector3d::Constant(NAN);
  if(isB)linearization=pred.prior.mean.position_world_m+pred.prior.mean.q_world_body*scene.lever;
  else if(r.gaussian.mean.size()==3)linearization=r.tag-r.gaussian.mean;
  out<<','<<linearization.x()<<','<<linearization.y()<<','<<linearization.z()<<'\n';
  if(e==14 || e==15)std::cout<<algorithm<<" "<<kind<<" epoch="<<e<<" status="<<r.status<<" exclude="<<r.excluded<<" stat="<<r.gaussian.statistic<<" hpl="<<pl.head<2>().norm()<<" vpl="<<pl.z()<<" core_ms="<<ms(coreStart,coreEnd)<<" full_ms="<<ms(start,end)<<"\n";
  lastTag=tag;lastTruth=truth;lastState=state;lastGt=gt;
 }
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
