#include "hybrid_coordinator.hpp"
#include "scenario_material.hpp"
#include "raw_source_law.hpp"
#include "raw_nonlinear_inclusion.hpp"
#include "sc2_imu_coordinator.hpp"
#include "sc3_joint_coordinator.hpp"
#include "sc4_system.hpp"
#include "rolling_boundary.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point a,Clock::time_point b){return std::chrono::duration<double,std::milli>(b-a).count();}
int main(int argc,char** argv){try{
 if(argc>=5 && std::string(argv[4])=="system")return runSc4System(argc,argv);
 if(argc>=5 && std::string(argv[4])=="rolling")return runSc5Rolling(argc,argv);
 if(std::getenv("SC3_JOINT_RECOVERY") && std::string(std::getenv("SC3_JOINT_RECOVERY"))=="1")return runSc3Joint(argc,argv);
 if(std::getenv("SC2_IMU_RECOVERY") && std::string(std::getenv("SC2_IMU_RECOVERY"))=="1")return runSc2Imu(argc,argv);
 if(argc<5)throw std::runtime_error("usage: epoch_raim_runner CONFIG CSV SCENE uwb_epoch_raim|fused_epoch_pl|hybrid [on|off] [epochs<=50] [history-invalid|imu-invalid]");
 std::string algorithm=argv[4],kind=argv[3];bool hybrid=algorithm=="hybrid",isB=hybrid || algorithm=="fused_epoch_pl",fde=argc<6 || std::string(argv[5])=="on";
 if(!isB && algorithm!="uwb_epoch_raim")throw std::runtime_error("unsupported algorithm");
 bool sharedEpisode=hybrid && std::getenv("HYBRID_SHARED_EPISODE") && std::string(std::getenv("HYBRID_SHARED_EPISODE"))=="1";
 bool rangeContraction=hybrid && std::getenv("SC1_RANGE_CONTRACTION") && std::string(std::getenv("SC1_RANGE_CONTRACTION"))=="1";
 auto sourceInitBegin=Clock::now();NoisyMotionScenario scene(argv[1],kind=="H1" && !hybrid?"U0":kind,20261010,argc>6?std::stoi(argv[6]):50);
 std::cout<<"source_law_init_ms="<<ms(sourceInitBegin,Clock::now())<<" numeric_mapping_valid="<<scene.numeric_certificate.valid<<"\n";
 double alpha=scene.law["alpha_per_test"].as<double>(),tail=scene.law["noise_tail_per_candidate"].as<double>();
 double risk=2*(scene.cfg.anchors.size()+1)*scene.epochs*tail;
 if(risk>scene.law["risk_budget"].as<double>())throw std::runtime_error("fixed candidate union exceeds budget");
 IncrementalUwbImuEstimator estimator(scene.cfg,scene.lever);
 estimator.initialize(scene.initial_observation,scene.cfg.realtime.prior_sigmas);estimator.ingestImu(scene.imu[0]);
 std::ofstream out(argv[2]);out<<std::setprecision(17);
 out<<"epoch,algorithm,label,status,alarm,excluded,correct,healthy_excluded,retained,updates,available,statistic,threshold,dof,hpl,vpl,error,increment_error,translation_rpe,rotation_rpe,core_ms,prediction_ms,commit_ms,full_ms,candidates,rows,cols,center_shift,union_charge,tag_x,tag_y,tag_z,timestamp_ns,formal_eligible,publication_protected,raw_total_risk,pure_uwb_pl,post_statistic,post_dof,excluded_anchor,excluded_measurement,max_slope_x,max_slope_y,max_slope_z,reference_x,reference_y,reference_z,horizontal_al,vertical_al,within_al,transaction_id,prediction_version,protection_frame,prior_independence,history_condition,model_version,linearization_tag_x,linearization_tag_y,linearization_tag_z,evidence_level,history_status,history_segments,history_bound,history_eta,a_alarm,a_available,a_tag_x,a_tag_y,a_tag_z,a_hpl,a_vpl,h_ms,h_update_ms,verification_ms,finalization_ms,cleanup_ms,a_ms,b_ms,full_C_calls,history_bytes,functional_status,numerical_status,model_status,risk_status,on_time,fused_conditional_available,committed_retained,retained_ids,selection_policy,finite_bound_qualified,application_policy_status,application_available,source_law_id,bound_risk_upper,risk_horizon,bound_scope,proof_binding_status,frozen_hpl,frozen_vpl,inclusion_x,inclusion_y,inclusion_z,inclusion_center_x,inclusion_center_y,inclusion_center_z,guarantee_domain,remainder_status,scope_risk_status,timeliness_status,horizontal_pe,vertical_pe,initial_support_x,initial_support_y,initial_support_z,adc_support,initial_bias_support,bias_rw_support,rotation_discretization_support,lever_support,sc1_status,sc1_valid,sc1_source,sc1_relaxation,sc1_old_hpl,sc1_old_vpl,sc1_hpl,sc1_vpl,sc1_old_x,sc1_old_y,sc1_old_z,sc1_x,sc1_y,sc1_z,sc1_low_x,sc1_low_y,sc1_low_z,sc1_high_x,sc1_high_y,sc1_high_z,sc1_old_width_x,sc1_old_width_y,sc1_old_width_z,sc1_width_x,sc1_width_y,sc1_width_z,range_contraction_ms,sc1_considered,sc1_empty,sc1_unresolved,sc1_visited,sc1_retained,sc1_frontier,sc1_peak_boxes,sc1_budget_exhausted,sc1_input_ids,sc1_event_risk\n";
 Eigen::Vector3d lastTag=Eigen::Vector3d::Zero(),lastTruth=Eigen::Vector3d::Zero();NavigationState lastState,lastGt;
 std::unique_ptr<OnlineGaussianHistory> online;
 if(hybrid)online=std::make_unique<OnlineGaussianHistory>(scene.cfg.integrity_window.epochs,3,scene.law["raw_accel_sigma_mps2"].as<double>(),scene.law["raw_gyro_sigma_radps"].as<double>(),scene.lever,true,64u*1024u*1024u,scene.physical_source_reference);
 std::unique_ptr<RawNonlinearInclusion> inclusion;
 if(hybrid && scene.physical_source_reference && scene.initial_law_valid && scene.numeric_certificate.valid){
  RawInclusionParameters p;p.numeric=scene.numeric_certificate;p.risk_budget=scene.law["raw_event_risk"].as<double>();p.horizon=scene.law["risk_horizon_epochs"].as<int>();p.initial_sigmas=scene.cfg.realtime.prior_sigmas;p.accel_adc=scene.law["raw_accel_sigma_mps2"].as<double>();p.gyro_adc=scene.law["raw_gyro_sigma_radps"].as<double>();p.accel_rw=scene.cfg.imu.accelerometer_bias_rw_sigma;p.gyro_rw=scene.cfg.imu.gyroscope_bias_rw_sigma;p.gravity=scene.cfg.imu.gravity_mps2;p.force_lipschitz=scene.law["specific_force_lipschitz_mps3"].as<double>();p.motion_amplitude=scene.amp;p.motion_frequency=scene.freq;p.motion_angular_rate=scene.rate;
  inclusion=std::make_unique<RawNonlinearInclusion>(scene.initial_observation,p);inclusion->append(scene.imu[0]);
 }
 bool imuEvidenceMissing=argc>7 && std::string(argv[7])=="imu-invalid";
 bool quarantined=argc>7 && std::string(argv[7])=="history-invalid";
 for(int e=1;e<=scene.epochs;++e){
  auto start=Clock::now();for(int i=(e-1)*10+1;i<=e*10;++i){estimator.ingestImu(scene.imu[i]);if(inclusion)inclusion->append(scene.imu[i]);}
  auto b=scene.uwb[e-1];EpochPreparationOptions options{hybrid,true,false,false};auto tx=estimator.prepareEpoch(b,options);
  auto predStart=Clock::now();ResearchEpochPredictionV1 pred;
  double hMs=0,hUpdateMs=0,verificationMs=0,finalizationMs=0,cleanupMs=0;size_t hBytes=0;
  OnlinePreUwbEvidence rawEvidence;Eigen::Vector3d historyPart=Eigen::Vector3d::Zero();double historyEta=0;
  if(hybrid){auto hs=Clock::now();
   rawEvidence.transaction=tx.id;rawEvidence.excluded_batch=b.id;rawEvidence.version=tx.base_version;
   rawEvidence.history.status="UNKNOWN";
   try{if(!online)throw std::runtime_error("H_LATCHED_UNKNOWN");online->prepare(tx,false);
   // Fixed primitive dimension bound is only a frozen raw diagnostic radius.
   // It does not pay for random linearization/model escape; raw risk UNKNOWN.
   int dimension=15+50*(66+15+8);
   if(scene.physical_source_reference)dimension=rawSourceInventory(50,scene.cfg.anchors.size(),scene.cfg.realtime.prior_sigmas,scene.law["raw_accel_sigma_mps2"].as<double>(),scene.law["raw_gyro_sigma_radps"].as<double>(),scene.cfg.imu.accelerometer_bias_rw_sigma,scene.cfg.imu.gyroscope_bias_rw_sigma,scene.cfg.realtime.range_sigma_m).size();
   double q=std::sqrt(boost::math::quantile(boost::math::complement(boost::math::chi_squared(dimension),1e-6)));
   rawEvidence=online->preUwbEvidence(tx,q);
   hBytes=online->counts().bytes;
   }catch(const std::exception&){online.reset();rawEvidence.history.status="UNKNOWN";}
   hMs=ms(hs,Clock::now());
   if(quarantined)rawEvidence.history.status="UNKNOWN";
  }
  if(isB){try{pred=estimator.researchPreUwbPredictionV1(tx);}catch(const std::exception&){if(!hybrid)throw;rawEvidence.history.status="UNKNOWN";online.reset();}}
  auto predEnd=Clock::now();
  // Explicit conditional frozen research law. I2 is a known unsupported test
  // supplied by the scenario law, not a GT label used to choose an exclusion.
  PredictionConditionsV1 conditions;
  if(!hybrid)conditions={true,true,!quarantined,kind!="I2" && kind!="I1",true};
  auto coreStart=Clock::now();double aMs=0,bMs=0;EpochRaim advisory;EpochRaim r;if(hybrid){advisory=uwbEpochRaim(b,fde,alpha,tail);aMs=ms(coreStart,Clock::now());r=hybridRawCurrent(b,rawEvidence,scene.lever,advisory,fde,alpha,&historyPart,&historyEta,sharedEpisode);if(imuEvidenceMissing){r=EpochRaim{};r.status="IMU_EVIDENCE_MISSING";}}else r=isB?fusedEpochPl(b,pred,scene.lever,conditions,fde,alpha,tail):uwbEpochRaim(b,fde,alpha,tail);auto coreEnd=Clock::now();bMs=hybrid?ms(coreStart,coreEnd)-aMs:0;
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
  if(isB && !hybrid && r.alarm && r.excluded<0)quarantined=true;
  auto transactionId=tx.id.value();auto predictionVersion=tx.base_graph_version;
  auto vs=coreEnd;
  if(hybrid && (rawEvidence.transaction!=tx.id || rawEvidence.excluded_batch!=b.id || !(rawEvidence.version==tx.base_version)))throw std::runtime_error("raw evidence binding failed");
  auto retainedTx=hybrid?std::make_unique<EpochTransaction>(tx):nullptr;
  verificationMs=ms(vs,Clock::now());
  auto before=estimator.backendUpdateCount();auto commitStart=Clock::now();auto receipt=estimator.commitEpoch(std::move(tx),plan);auto commitEnd=Clock::now();auto state=estimator.currentState();
  if(receipt.backend_updates!=1 || estimator.backendUpdateCount()!=before+1)throw std::runtime_error("not exactly one backend update");
  Eigen::Vector3d fusedTag=state.position_world_m+state.q_world_body*scene.lever;
  Eigen::Vector3d tag=isB?fusedTag:r.tag,pl=r.gaussian.pl;
  double shift=isB && r.tag.allFinite()?(fusedTag-r.tag).norm():0;
  bool available=r.status=="FROZEN_MODEL_ONLY";
  if(hybrid && (receipt.transaction_id.value()!=transactionId || receipt.state_timestamp!=b.timestamp || receipt.committed_epoch!=size_t(e) || receipt.graph_version!=predictionVersion+1))throw std::runtime_error("hybrid receipt binding failed");
  if(isB && !hybrid && available){pl+=(fusedTag-r.tag).cwiseAbs();if(shift>.25){available=false;r.status="COMMITTED_CENTER_DOMAIN_FAILED";pl.setConstant(INFINITY);}}
  std::set<uint64_t> actualSources;
  for(auto group:plan.groups_to_add)for(const auto& entry:estimator.factorLedger().groupEntries(group))if(entry.kind==FactorKind::UwbBatch && entry.lifecycle==FactorLifecycle::Active)for(auto source:entry.source_measurements)actualSources.insert(source.value());
  if(hybrid){std::set<uint64_t> expectedSources;for(auto group:plan.groups_to_add)for(const auto& g:retainedTx->uwb_groups)if(g.id==group)for(auto source:g.source_measurements)expectedSources.insert(source.value());if(actualSources!=expectedSources)throw std::runtime_error("actual retained lineage mismatch");}
  std::ostringstream sourceIds;bool firstSource=true;for(auto source:actualSources){if(!firstSource)sourceIds<<'|';sourceIds<<source;firstSource=false;}
  auto fs=commitEnd;RawRangeResult contracted;RawInclusionEvidence rawBound;bool rawQualified=false;
  Eigen::Vector3d frozenPL=r.gaussian.pl;

  if(hybrid){auto final=finalizeHybrid(r.gaussian.pl,r.tag,state,scene.lever,receipt,TransactionId(transactionId),e,b.timestamp,predictionVersion,HybridDomain::RawUnqualified,NAN,scene.law["risk_budget"].as<double>(),scene.cfg.risk_v2.horizontal_alert_limit_m,scene.cfg.risk_v2.vertical_alert_limit_m,ms(start,fs));
   if(!final.receipt_bound)throw std::runtime_error("hybrid final receipt failed");
   if(final.numerical)pl=final.pl;
   available=final.available();frozenPL=pl;
   if(inclusion && !imuEvidenceMissing && !quarantined && rawEvidence.history.status=="BOUNDED" && r.status=="FROZEN_MODEL_ONLY"){
    rawBound=inclusion->issue(*retainedTx,receipt,state,scene.lever);
    rawQualified=rawBound.valid && final.receipt_bound && rawBound.source_law_id==scene.law["source_law_id"].as<std::string>() && rawBound.epoch==e && rawBound.horizon==50;
    if(rawQualified)pl=rawBound.pl;
    if(rangeContraction){
     auto rangeStart=Clock::now();contracted=issueRawRangeContraction(rawBound,scene.range_certificates.at(e-1),*retainedTx,receipt,state,scene.lever);contracted.elapsed_ms=ms(rangeStart,Clock::now());
     if(contracted.valid)pl=contracted.pl;else{rawQualified=false;pl.setConstant(INFINITY);}
    }
   }

  }
  if(hybrid){auto hu=Clock::now();for(auto group:plan.groups_to_add)if(!receipt.group_to_slots.count(group.value()))throw std::runtime_error("missing committed group receipt");
   if(online)online->committed(*retainedTx,plan);hUpdateMs=ms(hu,Clock::now());}
  auto gt=scene.truth(e*scene.dt);Eigen::Vector3d truth=gt.position_world_m+gt.q_world_body*scene.lever;
  double error=tag.allFinite()?(tag-truth).norm():NAN,inc=e>1 && tag.allFinite() && lastTag.allFinite()?((tag-lastTag)-(truth-lastTruth)).norm():NAN;
  double tr=NAN,rot=NAN;
  if(e>1){tr=((lastState.q_world_body.conjugate()*(state.position_world_m-lastState.position_world_m))-(lastGt.q_world_body.conjugate()*(gt.position_world_m-lastGt.position_world_m))).norm();rot=Eigen::AngleAxisd((lastGt.q_world_body.conjugate()*gt.q_world_body).conjugate()*(lastState.q_world_body.conjugate()*state.q_world_body)).angle();}
  int fault=scene.law["fault_epoch"].as<int>(),stop=fault+(kind=="U2"?scene.law["persistent_epochs"].as<int>():1);
  bool injected=(kind=="U1" || kind=="U2") && e>=fault && e<stop;
  bool correct=injected && r.excluded==scene.law["fault_anchor_index"].as<int>();
  finalizationMs=ms(fs,Clock::now())-hUpdateMs;
  size_t historySegments=rawEvidence.history.segments.size();
  auto cleanupStart=Clock::now();retainedTx.reset();
  if(hybrid){rawEvidence.history.segments.clear();rawEvidence.history.covariance.resize(0,0);rawEvidence.history.mean.resize(0);rawEvidence.correction.resize(0);
   r.gaussian.covariance.resize(0,0);r.gaussian.gain.resize(0,0);r.gaussian.parity.resize(0,0);r.gaussian.whitener.resize(0,0);
   advisory.gaussian.covariance.resize(0,0);advisory.gaussian.gain.resize(0,0);advisory.gaussian.parity.resize(0,0);advisory.gaussian.whitener.resize(0,0);
   pred.previous_covariance.resize(0,0);pred.transition_design.resize(0,0);
  }
  cleanupMs=ms(cleanupStart,Clock::now());
  auto end=Clock::now();
  if(hybrid)available=false; // P1: raw chart/noise/remainder qualification is incomplete.
  out<<e<<','<<algorithm<<','<<(hybrid?"HYBRID_RAW_DIAGNOSTIC":isB?"FUSED_EPOCH_FROZEN_GAUSSIAN":"UWB_EPOCH_RAIM_CONDITIONAL")<<','<<r.status<<','<<r.alarm<<','<<r.excluded<<','<<correct<<','<<(r.excluded>=0 && !correct)<<','<<r.retained.size()<<','<<receipt.backend_updates<<','<<available<<','<<r.detection_statistic<<','<<r.detection_threshold<<','<<r.detection_dof<<','<<(contracted.valid?contracted.hpl:rawQualified?rawBound.hpl:pl.head<2>().norm())<<','<<pl.z()<<','<<error<<','<<inc<<','<<tr<<','<<rot<<','<<ms(coreStart,coreEnd)<<','<<(ms(predStart,predEnd)-hMs)<<','<<(hybrid?ms(commitStart,commitEnd):estimator.lastUwbUpdateMs())<<','<<ms(start,end)<<','<<r.candidates<<','<<(isB?15:0)+r.retained.size()<<','<<(isB?15:3)<<','<<shift<<','<<(hybrid?NAN:risk)<<','<<tag.x()<<','<<tag.y()<<','<<tag.z()<<','<<state.timestamp.value()<<",0,0,UNKNOWN,"<<!isB<<','<<r.gaussian.statistic<<','<<r.gaussian.dof<<','<<(r.excluded>=0?b.measurements[r.excluded].anchor_id.value():0)<<','<<(r.excluded>=0?b.measurements[r.excluded].id.value():0);
  Eigen::Vector3d slopes=Eigen::Vector3d::Constant(INFINITY);if(r.slope.cols())slopes=r.slope.rowwise().maxCoeff();
  out<<','<<slopes.x()<<','<<slopes.y()<<','<<slopes.z()<<','<<r.tag.x()<<','<<r.tag.y()<<','<<r.tag.z()<<','<<scene.cfg.risk_v2.horizontal_alert_limit_m<<','<<scene.cfg.risk_v2.vertical_alert_limit_m<<','<<((hybrid?pl.allFinite():available) && (contracted.valid?contracted.hpl:rawQualified?rawBound.hpl:pl.head<2>().norm())<=scene.cfg.risk_v2.horizontal_alert_limit_m && pl.z()<=scene.cfg.risk_v2.vertical_alert_limit_m)<<','<<transactionId<<','<<predictionVersion<<",world,"<<(hybrid?"RAW_SOURCE_MODEL_UNQUALIFIED":isB?"FIXED_MODEL_ASSUMED":"NOT_USED")<<','<<(hybrid?rawEvidence.history.status:isB?(!conditions.clean_history?"INVALID":"CONDITION_ASSUMED"):"NOT_USED")<<','<<(hybrid?(sharedEpisode?"hybrid-raw-history-shared-episode-v2":"hybrid-raw-history-joint-v1"):isB?"epoch-fused-gaussian-v1":"epoch-uwb-slope-v1");
  Eigen::Vector3d linearization=Eigen::Vector3d::Constant(NAN);
  if(hybrid)linearization=rawEvidence.chart.position_world_m+rawEvidence.chart.q_world_body*scene.lever;
  else if(isB)linearization=pred.prior.mean.position_world_m+pred.prior.mean.q_world_body*scene.lever;
  else if(r.gaussian.mean.size()==3)linearization=r.tag-r.gaussian.mean;
  out<<','<<linearization.x()<<','<<linearization.y()<<','<<linearization.z()<<','<<(hybrid?"RAW_UNQUALIFIED":"ASSUMPTION_ONLY")<<','<<(hybrid?rawEvidence.history.status:"NOT_USED")<<','<<historySegments<<','<<historyPart.norm()<<','<<historyEta<<','<<advisory.alarm<<','<<(advisory.status=="FROZEN_MODEL_ONLY")<<','<<advisory.tag.x()<<','<<advisory.tag.y()<<','<<advisory.tag.z()<<','<<advisory.gaussian.pl.head<2>().norm()<<','<<advisory.gaussian.pl.z()<<','<<hMs<<','<<hUpdateMs<<','<<verificationMs<<','<<finalizationMs<<','<<cleanupMs<<','<<aMs<<','<<bMs<<",0,"<<hBytes<<','<<(r.excluded>=0?"UWB_EXCLUDED":hybrid?"BEST_EFFORT":"NOMINAL")<<','<<(r.gaussian.pl.allFinite()?"FINITE":"INDETERMINATE")<<','<<(hybrid?"RAW_UNQUALIFIED":"ASSUMPTION_ONLY")<<",UNKNOWN,"<<(ms(start,end)<=50)<<",0,"<<actualSources.size()<<','<<sourceIds.str()<<','<<(hybrid?"bound_first_research-v1":"legacy_al_gated")<<','<<rawQualified<<",APPLICATION_AL_UNQUALIFIED,0,"<<(scene.physical_source_reference?"raw-source-rw-v1":"legacy-mismatch")<<','<<(rawQualified?rawBound.event_risk:NAN)<<",50,"<<(rawQualified?rawBound.scope:"UNQUALIFIED")<<','<<(rawQualified?"BOUND_TO_ACTUAL_RECEIPT":"REFUSED")<<','<<frozenPL.head<2>().norm()<<','<<frozenPL.z();
  Eigen::Vector3d inclusionPart=Eigen::Vector3d::Constant(NAN),inclusionCenter=Eigen::Vector3d::Constant(NAN);
  if(rawQualified){inclusionPart=(rawBound.high-rawBound.low)/2;inclusionCenter=(fusedTag-rawBound.reference).cwiseAbs();}
  out<<','<<inclusionPart.x()<<','<<inclusionPart.y()<<','<<inclusionPart.z()<<','<<inclusionCenter.x()<<','<<inclusionCenter.y()<<','<<inclusionCenter.z()<<','<<(rawQualified?"RAW_GENERATOR_CONDITIONAL":"FROZEN_LINEAR_DIAGNOSTIC")<<','<<(rawQualified?"CONTINUOUS_INTERVAL_INCLUSION":"UNQUALIFIED")<<','<<(rawQualified?"VERIFIED":"UNKNOWN")<<",UNKNOWN,"<<(tag-truth).head<2>().norm()<<','<<std::abs((tag-truth).z())<<','<<rawBound.initial.x()<<','<<rawBound.initial.y()<<','<<rawBound.initial.z()<<','<<rawBound.adc<<','<<rawBound.initial_bias<<','<<rawBound.bias_rw<<','<<rawBound.rotation_discretization<<','<<rawBound.lever;
  std::ostringstream allIds;for(size_t i=0;i<b.measurements.size();++i){if(i)allIds<<'|';allIds<<b.measurements[i].id.value();}
  out<<','<<(rangeContraction?contracted.status:"DISABLED")<<','<<contracted.valid<<','<<contracted.source<<','<<contracted.relaxation<<','<<rawBound.hpl<<','<<rawBound.pl.z()<<','<<contracted.hpl<<','<<contracted.pl.z();
  for(auto vector:{rawBound.pl,contracted.pl,contracted.low,contracted.high,Eigen::Vector3d(rawBound.high-rawBound.low),Eigen::Vector3d(contracted.high-contracted.low)})for(int j=0;j<3;++j)out<<','<<vector(j);
  out<<','<<contracted.elapsed_ms<<','<<contracted.considered<<','<<contracted.empty<<','<<contracted.unresolved<<','<<contracted.visited<<','<<contracted.retained<<','<<contracted.frontier<<','<<contracted.peak_boxes<<','<<contracted.budget_exhausted<<','<<allIds.str()<<','<<contracted.event_risk<<'\n';
  if(e==14 || e==15)std::cout<<algorithm<<" "<<kind<<" epoch="<<e<<" status="<<r.status<<" exclude="<<r.excluded<<" stat="<<r.gaussian.statistic<<" hpl="<<(contracted.valid?contracted.hpl:rawQualified?rawBound.hpl:pl.head<2>().norm())<<" vpl="<<pl.z()<<" core_ms="<<ms(coreStart,coreEnd)<<" full_ms="<<ms(start,end)<<"\n";
  lastTag=tag;lastTruth=truth;lastState=state;lastGt=gt;
 }
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
