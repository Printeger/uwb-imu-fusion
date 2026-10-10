#include "scenario_material.hpp"
#include "gaussian_window.hpp"
#include "statistical_history.hpp"
#include "imu_recovery_scope.hpp"
#include "raw_cache_replay.hpp"
#include <tbb/global_control.h>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include <gtsam/inference/Symbol.h>
#include <yaml-cpp/yaml.h>
#include <random>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
using Clock=std::chrono::steady_clock;
namespace {
void finiteUwbModes(GeneratedFaultModelSet& models,int duration){
 const auto original=models.modes;
 for(const auto& first:original)if(first.sensor==SensorType::Uwb){
  auto mode=first;mode.id=FaultModeId(models.modes.size()+1);mode.kind=FaultKind::AnchorBiasPersistentConstant;
  mode.physical_source_id+="|finite-"+std::to_string(duration);mode.raw_group_maps.clear();mode.affected_groups.clear();mode.affected_measurements.clear();
  for(const auto& m:original)if(m.sensor==SensorType::Uwb && m.anchor_id==first.anchor_id && m.onset_epoch>=first.onset_epoch && m.onset_epoch<first.onset_epoch+duration){
   for(const auto& kv:m.raw_group_maps){auto it=mode.raw_group_maps.find(kv.first);if(it==mode.raw_group_maps.end())mode.raw_group_maps.emplace(kv.first,kv.second);else it->second+=kv.second;}
   mode.affected_groups.insert(mode.affected_groups.end(),m.affected_groups.begin(),m.affected_groups.end());mode.affected_measurements.insert(mode.affected_measurements.end(),m.affected_measurements.begin(),m.affected_measurements.end());}
  auto unit=models.units.at(first.id.value()-1);unit.id=FaultUnitId(mode.id.value());unit.kind=mode.kind;unit.epoch_end=first.onset_epoch+duration;unit.time_end=TimestampNs(unit.time_begin.value()+duration*50000000LL);
  unit.affected_groups=mode.affected_groups;unit.affected_measurements=mode.affected_measurements;unit.physical_source_id=mode.physical_source_id;
  auto h=models.hypotheses.at(first.id.value()-1);h.id=HypothesisId(models.hypotheses.size()+1);h.modes={mode.id};h.units={unit.id};
  models.modes.push_back(std::move(mode));models.units.push_back(std::move(unit));models.hypotheses.push_back(std::move(h));
 }
}
EpochCommitPlan planFor(const EpochTransaction& tx,const ExclusionAction& a){
 auto p=EpochCommitPlan::nominalPlan(tx);p.action_id=a.id;p.bridge_mode=a.bridge_mode;p.fde_status=FdeStatus::SuccessKeepAll;
 for(auto id:a.groups_to_remove){bool pending=id==tx.imu_group.id;for(const auto& g:tx.uwb_groups)pending=pending || id==g.id;
  if(!pending)p.groups_to_remove.push_back(id);
  p.groups_to_add.erase(std::remove(p.groups_to_add.begin(),p.groups_to_add.end(),id),p.groups_to_add.end());}
 for(auto id:a.groups_to_add)if(std::find(p.groups_to_add.begin(),p.groups_to_add.end(),id)==p.groups_to_add.end())p.groups_to_add.push_back(id);
 return p;
}
}
int main(int argc,char** argv){try{
 tbb::global_control scheduler(tbb::global_control::max_allowed_parallelism,1);Eigen::setNbThreads(1);
 if(argc<5)throw std::runtime_error("usage: fde_next_runner CONFIG OUTPUT SCENE SCOPE [seed-or-replay-cache] [epochs] [reference|optimized] [attempts<=35]");
 std::string kind=argv[3],scope=argv[4];if(kind!="U0"&&kind!="U1"&&kind!="U2"&&kind!="U3"&&kind!="I1"&&kind!="I2"&&kind!="screen"&&kind!="replay")throw std::runtime_error("unsupported scene");
 if(scope!="off"&&scope!="uwb"&&scope!="imu-diagnostic")throw std::runtime_error("unsupported fault scope: IMU/joint require explicit diagnostic entry");
 auto law=YAML::LoadFile(argv[1]);int seed=argc>5 && kind!="replay"?std::stoi(argv[5]):law["seed"].as<int>();NoisyMotionScenario scene(argv[1],kind,seed,argc>6?std::stoi(argv[6]):0);
 if(kind=="replay"){if(argc<7 || scope!="off")throw std::runtime_error("replay usage: CONFIG OUTPUT replay off CACHE COUNT; unqualified diagnostic only");loadRawCache(scene,argv[5]);}
 bool benchmark=argc>8;int attempts=benchmark?std::stoi(argv[8]):1;
 std::string numeric=argc>7?argv[7]:"reference";bool profile=numeric=="profile";bool optimized=numeric=="optimized";
 if(numeric!="reference" && numeric!="profile" && numeric!="optimized")throw std::runtime_error("unsupported numerical mode");
 if(attempts<1 || attempts>35 || (benchmark && scope!="uwb"))throw std::runtime_error("invalid benchmark workload");
 double alpha=law["alpha_per_test"].as<double>(),tail=law["noise_tail_per_candidate"].as<double>();
 double task_risk=std::nextafter(2*tail*law["epochs"].as<int>()*law["max_candidates"].as<int>(),INFINITY);
 if(task_risk>law["risk_budget"].as<double>())throw std::runtime_error("risk precheck rejects before replay");
 std::ofstream out(argv[2]);if(!out)throw std::runtime_error("output unavailable");out<<std::setprecision(17);
 out<<"epoch,alarm,statistic,threshold,rows,cols,hypotheses,actions,numerical,post,pl_evaluated,selected,exclusion,correct,updates,error,hpl,vpl,reference_x,reference_y,reference_z,commit_x,commit_y,commit_z,core_ms,commit_ms,cycle_ms,cross_covariance,integrations,status,frozen_union_charge,raw_qualified,formal_eligible,publication_protected,increment_error,rotation_rpe,velocity_error,se3_translation_rpe,final_local_available,legacy_alarm,current_tag_local_hpl,current_tag_local_vpl,future_state_status,raw_total_risk,model,scope,protection_point,timestamp_ns,attempt,end_to_end_ms,cleanup_ms,gaussian_decompositions,proof_entries,selected_action,removed_groups\n";
 std::unique_ptr<std::ofstream> audit;
 if(const char* path=std::getenv("FDE_NEXT_NUMERIC_AUDIT")){
  if(!benchmark || attempts!=1)throw std::runtime_error("numeric audit requires one benchmark snapshot");
  audit=std::make_unique<std::ofstream>(path);if(!*audit)throw std::runtime_error("audit output unavailable");*audit<<std::setprecision(17);
 }
 auto dump=[&](const std::string& label,const Eigen::MatrixXd& m){if(audit){*audit<<label<<' '<<m.rows()<<' '<<m.cols()<<'\n';for(int i=0;i<m.rows();++i)for(int j=0;j<m.cols();++j)*audit<<m(i,j)<<' ';*audit<<'\n';}};
 int exclusions=0,alarms=0,failures=0;
 for(int attempt=0;attempt<attempts;++attempt){
 Clock::time_point measured_start;double measured_compute=0.;std::string benchmark_record,benchmark_suffix;
 {
 IncrementalUwbImuEstimator estimator(scene.cfg,scene.lever);auto init=kind=="replay"?initialState():scene.truth(0);if(kind!="replay")init.position_world_m+=researchVector(law["initial_position_error_m"]);
 estimator.initialize(init,scene.cfg.realtime.prior_sigmas);estimator.ingestImu(scene.imu[0]);
 Eigen::Vector3d previous_tag=init.position_world_m+init.q_world_body*scene.lever;auto previous_truth=scene.truth(0);
 Eigen::Quaterniond previous_rotation=init.q_world_body;Eigen::Vector3d previous_body=init.position_world_m;
 size_t imu_cursor=1;
 for(int e=1;e<=scene.epochs;++e){auto start=Clock::now();
  while(imu_cursor<scene.imu.size() && !(scene.uwb[e-1].timestamp<scene.imu[imu_cursor].timestamp))estimator.ingestImu(scene.imu[imu_cursor++]);
  auto tx=estimator.prepareEpoch(scene.uwb[e-1]);
  if((kind=="screen" || benchmark) && e<scene.epochs){auto prewarm_plan=EpochCommitPlan::nominalPlan(tx);estimator.commitEpoch(std::move(tx),prewarm_plan);continue;}
  if(benchmark)measured_start=start;
  IntegrityWindowRequest req;req.epochs=scene.cfg.integrity_window.epochs;ResearchHistoryAudit history_audit;
  bool history_check=std::getenv("FDE_NEXT_HISTORY_AUDIT") && e>=30 && e<=32;
  auto built=history_check?estimator.buildIntegrityWindowWithResearchHistory(tx,req,&history_audit):estimator.buildIntegrityWindow(tx,req);
  auto seal=freezeIntegrityWindow(std::move(built));auto admission=admitFrozenIntegrityWindow(seal);
  if(!admission)throw std::runtime_error(admission.reason);
  const auto& w=admission.window();auto t_window=Clock::now();
  DetectorRiskContext dr;dr.p_fa_per_test=alpha;dr.continuity_horizon_tests=law["epochs"].as<int>();
  auto detector=JointWindowDetector{}.evaluate(w,dr); // retained estimator-weight diagnostic only
  HypothesisGeneratorConfig gc;gc.include_imu_faults=scope=="imu-diagnostic";gc.include_uwb_faults=scope!="imu-diagnostic";gc.include_persistent_uwb=false;gc.include_ramp_uwb=false;gc.max_exclusion_cardinality=2;
  auto models=gc.include_imu_faults?HypothesisGenerator(gc).generate(w,tx,ImuFaultSubspaceBuilder{}.buildAnalytic(tx,estimator.buildPendingFactorBlock(tx,tx.imu_group.id)),{}):HypothesisGenerator(gc).generate(w,tx,{},{});
  if(scope=="imu-diagnostic")models=imuEpisodeModels(models,w,tx,law["imu_episode_durations_ms"].as<std::vector<int>>());
  else finiteUwbModes(models,law["persistent_epochs"].as<int>());
  auto t_models=Clock::now();
  auto joint=rawGaussianWindow(w,tx,models.modes,law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
  if(history_check && history_audit.available){
   const auto& h=history_audit.input;int m=h.rows(),old=h.oldStateColumns();
   auto compressed=statisticalHistory(h.h_old_state,h.h_boundary*history_audit.boundary_embedding,h.rhs,
       Eigen::MatrixXd::Identity(m,m),h.fault_map,Eigen::MatrixXd::Identity(m,m),Eigen::MatrixXd(m,0));
   int current=0;for(const auto& b:w.blocks)if(b.kind!=FactorKind::BoundaryPrior)current+=b.residual_whitened.size();
   Eigen::MatrixXd a=Eigen::MatrixXd::Zero(m+current,old+w.H.cols());Eigen::VectorXd z(m+current);
   a.topLeftCorner(m,old)=h.h_old_state;a.topRightCorner(m,w.H.cols())=h.h_boundary*history_audit.boundary_embedding;z.head(m)=h.rhs;
   Eigen::MatrixXd ac=Eigen::MatrixXd::Zero(compressed.z.size()+current,w.H.cols());Eigen::VectorXd zc(ac.rows());
   ac.topRows(compressed.z.size())=compressed.a;zc.head(compressed.z.size())=compressed.z;
   int rawoff=m,off=compressed.z.size();for(const auto& b:w.blocks)if(b.kind!=FactorKind::BoundaryPrior){
    int n=b.residual_whitened.size();a.block(rawoff,old,n,w.H.cols())=b.jacobian_whitened;z.segment(rawoff,n)=b.residual_whitened;
    ac.middleRows(off,n)=b.jacobian_whitened;zc.segment(off,n)=b.residual_whitened;rawoff+=n;off+=n;}
   Eigen::MatrixXd c=Eigen::MatrixXd::Zero(3,a.cols());c.rightCols(w.H.cols())=joint.c;
   auto direct=gaussianReference(a,z,Eigen::MatrixXd::Identity(a.rows(),a.rows()),c,{},alpha,tail);
   auto adapted=gaussianReference(ac,zc,Eigen::MatrixXd::Identity(ac.rows(),ac.rows()),joint.c,{},alpha,tail);
   auto legacy=gaussianReference(w.H,w.z,Eigen::MatrixXd::Identity(w.H.rows(),w.H.rows()),joint.c,{},alpha,tail);
   std::cout<<"HISTORY epoch="<<e<<" raw_rows="<<m<<" eliminated="<<old<<" raw_dof="<<direct.dof<<" adapted_dof="<<adapted.dof<<" legacy_dof="<<legacy.dof
     <<" S_direct="<<direct.statistic<<" S_adapted="<<adapted.statistic<<" S_legacy="<<legacy.statistic<<" tau_raw="<<direct.threshold<<" tau_legacy="<<legacy.threshold
     <<" mean_diff="<<(direct.mean.tail(w.H.cols())-adapted.mean).norm()<<" cov_diff="<<(c*direct.covariance*c.transpose()-joint.c*adapted.covariance*joint.c.transpose()).norm()
     <<" info_factors="<<w.history_summary.information_form_factors<<" offset="<<w.history_summary.constant_offset<<" raw_noise_history=UNQUALIFIED\n";
  }
  auto t_raw=Clock::now();
  auto raw_detector=gaussianReference(joint.a,joint.z,joint.sigma,joint.c,{},alpha,tail);
  auto t_detector=Clock::now();
  bool alarm=raw_detector.status!="FROZEN_MODEL_ONLY" || raw_detector.statistic>raw_detector.threshold;alarms+=alarm;
  if(kind=="screen"){
   double max_a=scene.amp.cwiseProduct(scene.freq.cwiseProduct(scene.freq)).norm();
   bool motion=bridgeMotionDomain(max_a,scene.rate,law["bridge_acceleration_bound_mps2"].as<double>(),law["bridge_angular_rate_bound_radps"].as<double>());
   for(size_t i=0;i<models.modes.size();++i)if(models.modes[i].onset_epoch+1==law["fault_epoch"].as<unsigned>()){
    auto signal=raw_detector.parity*raw_detector.whitener*joint.faults[i];
    double lambda=std::pow(law["imu_accel_amplitude_mps2"].as<double>(),2)*signal.squaredNorm();
    double beta=boost::math::cdf(boost::math::non_central_chi_squared(raw_detector.dof,lambda),raw_detector.threshold);
    std::cout<<"SCREEN event="<<models.modes[i].physical_source_id<<" lambda="<<lambda<<" beta="<<beta<<" target="<<law["imu_target_beta"].as<double>()<<" factors="<<models.modes[i].affected_groups.size()<<" max_a="<<max_a<<" bridge_motion="<<motion<<" bridge_full_state=UNQUALIFIED\n";
   }
   break;
  }
  AttemptProofArena arena;std::shared_ptr<const FrozenHypothesisNumerics> frozen;std::shared_ptr<const FrozenHypothesisDualNumerics> dual;
  auto evidence=HypothesisEvidenceEvaluator{}.evaluateAll(admission,models.modes,&models.hypotheses,raw_detector.threshold,&frozen,&dual,nullptr,&arena);
  std::vector<ExclusionAction> actions;
  if(!alarm || scope=="off"){ExclusionAction keep;keep.id=ExclusionActionId(1);keep.action_model_id="KEEP_ALL";actions.push_back(keep);}
  else {auto search=HypothesisGenerator(gc).actionsForPlausibleSetV1(w,tx,&models,evidence);
   if(search.lifecycle==ActionSearchLifecycleV1::ReadyForEvaluation)actions=std::move(search.actions);
   else std::cerr<<"epoch="<<e<<" action_search="<<search.census.terminal_reason<<'\n';}
  if(actions.size()>law["max_candidates"].as<unsigned>())throw std::runtime_error("candidate cap: union proof invalid");
  if(audit){dump("raw_design",joint.a);dump("raw_covariance",joint.sigma);dump("tag_map",joint.c);
   for(size_t i=0;i<joint.faults.size();++i){dump("fault_"+std::to_string(i),joint.faults[i]);*audit<<"#mode "<<models.modes[i].id.value()<<' '<<models.modes[i].physical_source_id<<'\n';}}
  if(e==2){
   Eigen::LLT<Eigen::MatrixXd> check_cov(joint.sigma);
   if(check_cov.info()!=Eigen::Success || joint.cross_norm<=1e-12)throw std::runtime_error("joint ADC covariance check failed");
   Eigen::MatrixXd check_w=check_cov.matrixL().solve(Eigen::MatrixXd::Identity(joint.sigma.rows(),joint.sigma.rows()));
   if((check_w*joint.sigma*check_w.transpose()-Eigen::MatrixXd::Identity(joint.sigma.rows(),joint.sigma.rows())).norm()>1e-7)throw std::runtime_error("joint whitening check failed");
  }
  auto t_actions=Clock::now();
  RankUpdateConfig rc;const double scales[5]={law["recovery_rotation_rad"].as<double>(),law["recovery_position_m"].as<double>(),law["recovery_velocity_mps"].as<double>(),law["recovery_accel_bias_mps2"].as<double>(),law["recovery_gyro_bias_radps"].as<double>()};
  double square_sum=0.;for(double scale:scales)square_sum+=scale*scale;
  rc.max_linearization_step_norm=std::sqrt((w.H.cols()/15)*square_sum); // enclosing gate; individual physical gates follow
  RankUpdateEvaluator evaluator(rc);auto base=evaluator.factorizeOnce(admission,actions);
  GaussianCandidateCache gaussian_cache(joint,w,alpha,tail);size_t decompositions=1;
  int numerical=0,post=0,pl_count=0;int best=-1;double best_cost=INFINITY;GaussianReference selected_ref;Eigen::VectorXd best_increment;
  std::string status="NO_ADMISSIBLE_CANDIDATE";
  for(size_t k=0;k<actions.size();++k){auto c=evaluator.evaluate(admission,base,actions[k]);
   bool domain=true;Eigen::VectorXd block_max=Eigen::VectorXd::Zero(5);
   if(c.state_increment.size()%15)domain=false;
   for(int q=0;q+15<=c.state_increment.size();q+=15){
    const int offsets[5]={0,3,6,9,12};
    for(int j=0;j<5;++j){double norm=c.state_increment.segment(q+offsets[j],3).norm();block_max(j)=std::max(block_max(j),norm);domain=domain && norm<=scales[j];}}
   if(!domain){c.valid=false;c.reason="research per-state physical trust domain exceeded";}
   if(!benchmark && (alarm || (!domain && e==3))){
    std::cerr<<"epoch="<<e<<" action="<<actions[k].id.value()<<" block_rot_pos_vel_ba_bg="<<block_max.transpose();
    if(scope=="imu-diagnostic"){std::cerr<<" sources=";for(const auto& source:actions[k].physical_source_ids)std::cerr<<source<<';';std::cerr<<" remove=";for(auto id:actions[k].groups_to_remove)std::cerr<<id.value()<<';';}
    std::cerr<<'\n';
   }
   if(!c.valid){if(!benchmark && alarm)std::cerr<<"epoch="<<e<<" action="<<actions[k].id.value()<<" invalid="<<c.reason<<" step="<<c.state_increment.norm()<<'\n';continue;}
   ++numerical;
   if(scope=="imu-diagnostic" && actions[k].bridge_mode!=BridgeMode::None){
    for(auto id:actions[k].covered_modes){auto it=std::find_if(models.modes.begin(),models.modes.end(),[&](const auto& m){return m.id==id;});
     if(it==models.modes.end() || !episodeCoverageComplete(*it,actions[k]))throw std::runtime_error("IMU candidate omits raw/history support");}
    std::cerr<<"epoch="<<e<<" action="<<actions[k].id.value()<<" removed_groups=";for(auto id:actions[k].groups_to_remove)std::cerr<<id.value()<<';';
    std::cerr<<" native_post="<<JointWindowDetector{}.evaluateCandidate(w,c,dr).passed<<" bridge_covariance=UNQUALIFIED\n";
   }
   auto r=gaussianCandidate(joint,w,actions[k],alpha,tail,optimized,optimized?&raw_detector:nullptr,optimized?&gaussian_cache:nullptr,&decompositions);++pl_count;
   if(r.status!="FROZEN_MODEL_ONLY"){status=scope=="imu-diagnostic" && actions[k].bridge_mode!=BridgeMode::None?"BRIDGE_GAUSSIAN_UNQUALIFIED":r.status;continue;}
   if(r.statistic>r.threshold)continue;
   ++post;
   bool reference_domain=true;
   for(int q=0;q+15<=r.mean.size();q+=15)for(int j=0;j<5;++j)
    reference_domain=reference_domain && r.mean.segment(q+3*j,3).norm()<=scales[j];
   if(!reference_domain){status="REFERENCE_DOMAIN_EXCEEDED";continue;}
   auto pl=(r.pl+(joint.c*(c.state_increment-r.mean)).cwiseAbs()).eval();
   if(optimized && (std::abs(pl.head<2>().norm()-scene.cfg.risk_v2.horizontal_alert_limit_m)<1e-6 || std::abs(pl.z()-scene.cfg.risk_v2.vertical_alert_limit_m)<1e-6)){
    r=gaussianCandidate(joint,w,actions[k],alpha,tail,false,nullptr,nullptr,&decompositions);pl=r.pl+(joint.c*(c.state_increment-r.mean)).cwiseAbs();
   }
   if(pl.head<2>().norm()>scene.cfg.risk_v2.horizontal_alert_limit_m || pl.z()>scene.cfg.risk_v2.vertical_alert_limit_m) {status="AL_EXCEEDED";continue;}
   double cost=actions[k].exclusion_cardinality*1000.+pl.norm();
   if(optimized && best>=0 && std::abs(cost-best_cost)<1e-7 &&
      (actions[k].groups_to_remove!=actions[best].groups_to_remove || actions[k].groups_to_add!=actions[best].groups_to_add)){
    auto old=gaussianCandidate(joint,w,actions[best],alpha,tail,false,nullptr,nullptr,&decompositions);
    r=gaussianCandidate(joint,w,actions[k],alpha,tail,false,nullptr,nullptr,&decompositions);
    if(old.status!="FROZEN_MODEL_ONLY" || r.status!="FROZEN_MODEL_ONLY"){best=-1;best_cost=INFINITY;status="INDETERMINATE_TIE";continue;}
    best_cost=actions[best].exclusion_cardinality*1000.+(old.pl+(joint.c*(best_increment-old.mean)).cwiseAbs()).norm();selected_ref=std::move(old);
    pl=r.pl+(joint.c*(c.state_increment-r.mean)).cwiseAbs();cost=actions[k].exclusion_cardinality*1000.+pl.norm();
   }
   dump("candidate_"+std::to_string(actions[k].id.value())+"_actual_delta",c.state_increment);
   if(audit)dump("candidate_"+std::to_string(actions[k].id.value())+"_actual_cov",c.covarianceTimes(Eigen::MatrixXd::Identity(w.H.cols(),w.H.cols())));
   dump("candidate_"+std::to_string(actions[k].id.value())+"_reference_delta",r.mean);
   dump("candidate_"+std::to_string(actions[k].id.value())+"_reference_cov",r.covariance);
   dump("candidate_"+std::to_string(actions[k].id.value())+"_reference_pl",r.pl);
   if(cost<best_cost){best_cost=cost;best=k;best_increment=c.state_increment;selected_ref=std::move(r);}
  }
  if(scope=="imu-diagnostic" && std::any_of(models.modes.begin(),models.modes.end(),[](const auto& m){return m.recoverability!=HistoryRecoverability::Recoverable;})){
   best=-1;status="HISTORY_RAW_EPISODE_UNQUALIFIED";
  }
  // Flat geometry may have a nonlinear mirror even if the local Jacobian is
  // full rank. Local PL does not choose the physical branch.
  if(kind=="U3"){best=-1;status="GEOMETRY_GLOBAL_BRANCH_UNQUALIFIED";}
  if(scope=="off"){best=-1;status=kind=="replay"?"RAWDATA_UNQUALIFIED":"FDE_OFF";}
  bool selected=best>=0;auto plan=selected?planFor(tx,actions[best]):EpochCommitPlan::nominalPlan(tx);plan.best_effort_integrity_unavailable=!selected;
  bool exclusion=selected&&!actions[best].groups_to_remove.empty();exclusions+=exclusion;
  auto pose=tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));
  Eigen::Vector3d reference=selected?Eigen::Vector3d(pose.translation()+pose.rotation().matrix()*scene.lever+joint.c*selected_ref.mean):Eigen::Vector3d::Constant(NAN);
  std::vector<MeasurementId> retained;for(const auto& g:tx.uwb_groups)if(std::find(plan.groups_to_add.begin(),plan.groups_to_add.end(),g.id)!=plan.groups_to_add.end())retained.insert(retained.end(),g.source_measurements.begin(),g.source_measurements.end());
  int correct=-1;int f=law["fault_epoch"].as<int>(),end=f+(kind=="U2"?law["persistent_epochs"].as<int>():1);
  if((kind=="U1"||kind=="U2")&&e>=f&&e<end){auto bad=tx.uwb_batch.measurements.at(law["fault_anchor_index"].as<int>()).id;
   correct=exclusion&&retained.size()==tx.uwb_batch.measurements.size()-1&&std::find(retained.begin(),retained.end(),bad)==retained.end();}
  if(audit){
   Eigen::VectorXd add(plan.groups_to_add.size()),remove(plan.groups_to_remove.size());
   for(size_t i=0;i<plan.groups_to_add.size();++i)add(i)=plan.groups_to_add[i].value();
   for(size_t i=0;i<plan.groups_to_remove.size();++i)remove(i)=plan.groups_to_remove[i].value();
   dump("actual_recipe_add",add);dump("actual_recipe_remove",remove);
  }
  if(benchmark && (!selected || (kind=="U1" && correct!=1)))throw std::runtime_error("benchmark requires successful declared normal/recovery workload");
  auto core_end=Clock::now();auto receipt=estimator.commitEpoch(std::move(tx),plan);auto state=estimator.currentState();auto tag=state.position_world_m+state.q_world_body*scene.lever;
  Eigen::Vector3d pl=selected?Eigen::Vector3d(selected_ref.pl+(tag-reference).cwiseAbs()):Eigen::Vector3d::Constant(INFINITY);
  double hpl=pl.head<2>().norm(),vpl=pl.z();if(selected)status="LINEARIZATION_UNQUALIFIED";else {++failures;if(scope=="imu-diagnostic" && alarm && numerical==0)status="IMU_CANDIDATE_DOMAIN_OR_PROVENANCE_FAILED";}
  auto truth=scene.truth(e*scene.dt);auto truth_tag=truth.position_world_m+truth.q_world_body*scene.lever;
  double inc=((tag-previous_tag)-(truth_tag-(previous_truth.position_world_m+previous_truth.q_world_body*scene.lever))).norm();
  double rot=Eigen::AngleAxisd((previous_rotation.conjugate()*state.q_world_body).conjugate()*(previous_truth.q_world_body.conjugate()*truth.q_world_body)).angle();
  double se3_translation=(previous_rotation.conjugate()*(state.position_world_m-previous_body)-previous_truth.q_world_body.conjugate()*(truth.position_world_m-previous_truth.position_world_m)).norm();
  bool final_available=selected && hpl<=scene.cfg.risk_v2.horizontal_alert_limit_m && vpl<=scene.cfg.risk_v2.vertical_alert_limit_m;
  double tag_hpl=INFINITY,tag_vpl=INFINITY;
  if(scope=="imu-diagnostic"){
   // Current UWB-only local position diagnostic; never added as an independent
   // velocity factor or used by the selector. No future-state guarantee.
   Eigen::MatrixXd a(scene.uwb[e-1].measurements.size(),3);Eigen::VectorXd z(a.rows());int row=0;
   Eigen::Vector3d frozen_tag=pose.translation()+pose.rotation().matrix()*scene.lever;
   for(const auto& m:scene.uwb[e-1].measurements){auto diff=(frozen_tag-m.anchor_position_m).eval();a.row(row)=diff.normalized().transpose();z(row++)=m.range_m-diff.norm();}
   auto position=gaussianReference(a,z,scene.cfg.realtime.range_sigma_m*scene.cfg.realtime.range_sigma_m*Eigen::MatrixXd::Identity(a.rows(),a.rows()),Eigen::Matrix3d::Identity(),{},alpha,tail);
   if(position.status=="FROZEN_MODEL_ONLY" && position.mean.norm()<=law["recovery_position_m"].as<double>()){
    auto transferred=(position.pl+(tag-frozen_tag-position.mean).cwiseAbs()).eval();tag_hpl=transferred.head<2>().norm();tag_vpl=transferred.z();}
  }
  auto end_clock=Clock::now();double core=std::chrono::duration<double,std::milli>(core_end-start).count(),commit=std::chrono::duration<double,std::milli>(end_clock-core_end).count();
  double error=(tag-truth_tag).norm(),velocity_error=(state.velocity_world_mps-truth.velocity_world_mps).norm();
  if(kind=="replay")error=velocity_error=inc=rot=se3_translation=NAN;
  std::ostringstream row;row<<std::setprecision(17)<<e<<','<<alarm<<','<<raw_detector.statistic<<','<<raw_detector.threshold<<','<<joint.a.rows()<<','<<joint.a.cols()<<','<<models.hypotheses.size()<<','<<actions.size()<<','<<numerical<<','<<post<<','<<pl_count<<','<<selected<<','<<exclusion<<','<<correct<<','<<receipt.backend_updates<<','<<error<<','<<hpl<<','<<vpl<<','<<reference.x()<<','<<reference.y()<<','<<reference.z()<<','<<tag.x()<<','<<tag.y()<<','<<tag.z()<<','<<core<<','<<commit<<','<<(core+commit)<<','<<joint.cross_norm<<','<<joint.integrations<<','<<status<<','<<task_risk<<",0,0,0,"<<inc<<','<<rot<<','<<velocity_error<<','<<se3_translation<<','<<final_available<<','<<!detector.passed<<','<<tag_hpl<<','<<tag_vpl<<",UNPROTECTED,UNKNOWN,"<<law["model_version"].as<std::string>()<<','<<scope<<",uwb_tag_world,"<<state.timestamp.value();
  if(benchmark){benchmark_record=row.str();measured_compute=core+commit;
   std::ostringstream suffix;suffix<<decompositions<<','<<arena.entryCount()<<','<<(selected?actions[best].id.value():0)<<',';
   if(selected)for(auto id:actions[best].groups_to_remove)suffix<<id.value()<<';';
   benchmark_suffix=suffix.str();}
  else {out<<row.str()<<','<<attempt<<','<<(core+commit)<<",0,"<<decompositions<<','<<arena.entryCount()<<','<<(selected?actions[best].id.value():0)<<',';
   if(selected)for(auto id:actions[best].groups_to_remove)out<<id.value()<<';';
   out<<'\n';}
  if(profile){auto ms=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
   std::cout<<"PROFILE prep="<<ms(start,t_window)<<" models="<<ms(t_window,t_models)<<" raw_cov="<<ms(t_models,t_raw)<<" detector="<<ms(t_raw,t_detector)<<" evidence_actions="<<ms(t_detector,t_actions)<<" candidates="<<ms(t_actions,core_end)<<" commit_final="<<commit<<'\n';}
  previous_body=state.position_world_m;previous_tag=tag;previous_truth=truth;previous_rotation=state.q_world_body;
 }
 } // include estimator and numerical-object destruction in attempt cycle
 if(benchmark){double total=std::chrono::duration<double,std::milli>(Clock::now()-measured_start).count();
  out<<benchmark_record<<','<<attempt<<','<<total<<','<<(total-measured_compute)<<','<<benchmark_suffix<<'\n';}
 } // attempts
 std::cout<<"scene="<<kind<<" scope="<<scope<<" seed="<<seed<<" epochs="<<scene.epochs<<" alarms="<<alarms<<" exclusions="<<exclusions<<" no_selected="<<failures<<" raw_qualified=false\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
