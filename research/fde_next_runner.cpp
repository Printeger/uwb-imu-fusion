#include "scenario_material.hpp"
#include "gaussian_window.hpp"
#include "imu_recovery_scope.hpp"
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include <gtsam/inference/Symbol.h>
#include <yaml-cpp/yaml.h>
#include <random>
#include <fstream>
#include <iostream>
#include <iomanip>
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
 if(argc<5)throw std::runtime_error("usage: fde_next_runner CONFIG OUTPUT U0|U1|U2|U3 off|uwb [seed] [epochs]");
 std::string kind=argv[3],scope=argv[4];if(kind!="U0"&&kind!="U1"&&kind!="U2"&&kind!="U3"&&kind!="I1"&&kind!="I2"&&kind!="screen")throw std::runtime_error("unsupported scene");
 if(scope!="off"&&scope!="uwb"&&scope!="imu-diagnostic")throw std::runtime_error("unsupported fault scope: IMU/joint require explicit diagnostic entry");
 auto law=YAML::LoadFile(argv[1]);int seed=argc>5?std::stoi(argv[5]):law["seed"].as<int>();NoisyMotionScenario scene(argv[1],kind,seed,argc>6?std::stoi(argv[6]):0);
 double alpha=law["alpha_per_test"].as<double>(),tail=law["noise_tail_per_candidate"].as<double>();
 double task_risk=std::nextafter(2*tail*law["epochs"].as<int>()*law["max_candidates"].as<int>(),INFINITY);
 if(task_risk>law["risk_budget"].as<double>())throw std::runtime_error("risk precheck rejects before replay");
 IncrementalUwbImuEstimator estimator(scene.cfg,scene.lever);auto init=scene.truth(0);init.position_world_m+=researchVector(law["initial_position_error_m"]);
 estimator.initialize(init,scene.cfg.realtime.prior_sigmas);estimator.ingestImu(scene.imu[0]);
 std::ofstream out(argv[2]);if(!out)throw std::runtime_error("output unavailable");out<<std::setprecision(17);
 out<<"epoch,alarm,statistic,threshold,rows,cols,hypotheses,actions,numerical,post,pl_evaluated,selected,exclusion,correct,updates,error,hpl,vpl,reference_x,reference_y,reference_z,commit_x,commit_y,commit_z,core_ms,commit_ms,cycle_ms,cross_covariance,integrations,status,risk,raw_qualified,formal_eligible,publication_protected,increment_error,rotation_rpe,velocity_error,se3_translation_rpe,final_local_available,legacy_alarm,current_tag_local_hpl,current_tag_local_vpl,future_state_status\n";
 int exclusions=0,alarms=0,failures=0;Eigen::Vector3d previous_tag=init.position_world_m+init.q_world_body*scene.lever;auto previous_truth=scene.truth(0);
 Eigen::Quaterniond previous_rotation=init.q_world_body;Eigen::Vector3d previous_body=init.position_world_m;
 for(int e=1;e<=scene.epochs;++e){auto start=Clock::now();for(int i=(e-1)*10+1;i<=e*10;++i)estimator.ingestImu(scene.imu[i]);
  auto tx=estimator.prepareEpoch(scene.uwb[e-1]);
  if(kind=="screen" && e<scene.epochs){auto prewarm_plan=EpochCommitPlan::nominalPlan(tx);estimator.commitEpoch(std::move(tx),prewarm_plan);continue;}
  IntegrityWindowRequest req;req.epochs=scene.cfg.integrity_window.epochs;auto w=estimator.buildIntegrityWindow(tx,req);
  DetectorRiskContext dr;dr.p_fa_per_test=alpha;dr.continuity_horizon_tests=law["epochs"].as<int>();
  auto detector=JointWindowDetector{}.evaluate(w,dr); // retained estimator-weight diagnostic only
  HypothesisGeneratorConfig gc;gc.include_imu_faults=scope=="imu-diagnostic";gc.include_uwb_faults=scope!="imu-diagnostic";gc.include_persistent_uwb=false;gc.include_ramp_uwb=false;gc.max_exclusion_cardinality=2;
  auto models=gc.include_imu_faults?HypothesisGenerator(gc).generate(w,tx,ImuFaultSubspaceBuilder{}.buildAnalytic(tx,estimator.buildPendingFactorBlock(tx,tx.imu_group.id)),{}):HypothesisGenerator(gc).generate(w,tx,{},{});
  if(scope=="imu-diagnostic")models=imuEpisodeModels(models,w,tx,law["imu_episode_durations_ms"].as<std::vector<int>>());
  else finiteUwbModes(models,law["persistent_epochs"].as<int>());
  auto joint=rawGaussianWindow(w,tx,models.modes,law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
  auto raw_detector=gaussianReference(joint.a,joint.z,joint.sigma,joint.c,{},alpha,tail);
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
  auto evidence=HypothesisEvidenceEvaluator{}.evaluateAll(w,models.modes,&models.hypotheses,raw_detector.threshold);
  std::vector<ExclusionAction> actions;
  if(!alarm || scope=="off"){ExclusionAction keep;keep.id=ExclusionActionId(1);keep.action_model_id="KEEP_ALL";actions.push_back(keep);}
  else {auto search=HypothesisGenerator(gc).actionsForPlausibleSetV1(w,tx,&models,evidence);
   if(search.lifecycle==ActionSearchLifecycleV1::ReadyForEvaluation)actions=std::move(search.actions);
   else std::cerr<<"epoch="<<e<<" action_search="<<search.census.terminal_reason<<'\n';}
  if(actions.size()>law["max_candidates"].as<unsigned>())throw std::runtime_error("candidate cap: union proof invalid");
  if(e==2){
   Eigen::LLT<Eigen::MatrixXd> check_cov(joint.sigma);
   if(check_cov.info()!=Eigen::Success || joint.cross_norm<=1e-12)throw std::runtime_error("joint ADC covariance check failed");
   Eigen::MatrixXd check_w=check_cov.matrixL().solve(Eigen::MatrixXd::Identity(joint.sigma.rows(),joint.sigma.rows()));
   if((check_w*joint.sigma*check_w.transpose()-Eigen::MatrixXd::Identity(joint.sigma.rows(),joint.sigma.rows())).norm()>1e-7)throw std::runtime_error("joint whitening check failed");
  }
  RankUpdateConfig rc;const double scales[5]={law["recovery_rotation_rad"].as<double>(),law["recovery_position_m"].as<double>(),law["recovery_velocity_mps"].as<double>(),law["recovery_accel_bias_mps2"].as<double>(),law["recovery_gyro_bias_radps"].as<double>()};
  double square_sum=0.;for(double scale:scales)square_sum+=scale*scale;
  rc.max_linearization_step_norm=std::sqrt((w.H.cols()/15)*square_sum); // enclosing gate; individual physical gates follow
  RankUpdateEvaluator evaluator(rc);auto base=evaluator.factorizeOnce(w,actions);
  int numerical=0,post=0,pl_count=0;int best=-1;double best_cost=INFINITY;GaussianReference selected_ref;
  std::string status="NO_ADMISSIBLE_CANDIDATE";
  for(size_t k=0;k<actions.size();++k){auto c=evaluator.evaluate(base,actions[k]);
   bool domain=true;Eigen::VectorXd block_max=Eigen::VectorXd::Zero(5);
   if(c.state_increment.size()%15)domain=false;
   for(int q=0;q+15<=c.state_increment.size();q+=15){
    const int offsets[5]={0,3,6,9,12};
    for(int j=0;j<5;++j){double norm=c.state_increment.segment(q+offsets[j],3).norm();block_max(j)=std::max(block_max(j),norm);domain=domain && norm<=scales[j];}}
   if(!domain){c.valid=false;c.reason="research per-state physical trust domain exceeded";}
   if(alarm || (!domain && e==3)){
    std::cerr<<"epoch="<<e<<" action="<<actions[k].id.value()<<" block_rot_pos_vel_ba_bg="<<block_max.transpose();
    if(scope=="imu-diagnostic"){std::cerr<<" sources=";for(const auto& source:actions[k].physical_source_ids)std::cerr<<source<<';';std::cerr<<" remove=";for(auto id:actions[k].groups_to_remove)std::cerr<<id.value()<<';';}
    std::cerr<<'\n';
   }
   if(!c.valid){if(alarm)std::cerr<<"epoch="<<e<<" action="<<actions[k].id.value()<<" invalid="<<c.reason<<" step="<<c.state_increment.norm()<<'\n';continue;}
   ++numerical;
   if(scope=="imu-diagnostic" && actions[k].bridge_mode!=BridgeMode::None){
    for(auto id:actions[k].covered_modes){auto it=std::find_if(models.modes.begin(),models.modes.end(),[&](const auto& m){return m.id==id;});
     if(it==models.modes.end() || !episodeCoverageComplete(*it,actions[k]))throw std::runtime_error("IMU candidate omits raw/history support");}
    std::cerr<<"epoch="<<e<<" action="<<actions[k].id.value()<<" removed_groups=";for(auto id:actions[k].groups_to_remove)std::cerr<<id.value()<<';';
    std::cerr<<" native_post="<<JointWindowDetector{}.evaluateCandidate(w,c,dr).passed<<" bridge_covariance=UNQUALIFIED\n";
   }
   auto r=gaussianCandidate(joint,w,actions[k],alpha,tail);++pl_count;
   if(r.status!="FROZEN_MODEL_ONLY"){status=scope=="imu-diagnostic" && actions[k].bridge_mode!=BridgeMode::None?"BRIDGE_GAUSSIAN_UNQUALIFIED":r.status;continue;}
   if(r.statistic>r.threshold)continue;
   ++post;
   bool reference_domain=true;
   for(int q=0;q+15<=r.mean.size();q+=15)for(int j=0;j<5;++j)
    reference_domain=reference_domain && r.mean.segment(q+3*j,3).norm()<=scales[j];
   if(!reference_domain){status="REFERENCE_DOMAIN_EXCEEDED";continue;}
   auto pl=(r.pl+(joint.c*(c.state_increment-r.mean)).cwiseAbs()).eval();
   if(pl.head<2>().norm()>scene.cfg.risk_v2.horizontal_alert_limit_m || pl.z()>scene.cfg.risk_v2.vertical_alert_limit_m) {status="AL_EXCEEDED";continue;}
   double cost=actions[k].exclusion_cardinality*1000.+pl.norm();
   if(cost<best_cost){best_cost=cost;best=k;selected_ref=std::move(r);}
  }
  if(scope=="imu-diagnostic" && std::any_of(models.modes.begin(),models.modes.end(),[](const auto& m){return m.recoverability!=HistoryRecoverability::Recoverable;})){
   best=-1;status="HISTORY_RAW_EPISODE_UNQUALIFIED";
  }
  // Flat geometry may have a nonlinear mirror even if the local Jacobian is
  // full rank. Local PL does not choose the physical branch.
  if(kind=="U3"){best=-1;status="GEOMETRY_GLOBAL_BRANCH_UNQUALIFIED";}
  if(scope=="off"){best=-1;status="FDE_OFF";}
  bool selected=best>=0;auto plan=selected?planFor(tx,actions[best]):EpochCommitPlan::nominalPlan(tx);plan.best_effort_integrity_unavailable=!selected;
  bool exclusion=selected&&!actions[best].groups_to_remove.empty();exclusions+=exclusion;
  auto pose=tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));
  Eigen::Vector3d reference=selected?Eigen::Vector3d(pose.translation()+pose.rotation().matrix()*scene.lever+joint.c*selected_ref.mean):Eigen::Vector3d::Constant(NAN);
  std::vector<MeasurementId> retained;for(const auto& g:tx.uwb_groups)if(std::find(plan.groups_to_add.begin(),plan.groups_to_add.end(),g.id)!=plan.groups_to_add.end())retained.insert(retained.end(),g.source_measurements.begin(),g.source_measurements.end());
  int correct=-1;int f=law["fault_epoch"].as<int>(),end=f+(kind=="U2"?law["persistent_epochs"].as<int>():1);
  if((kind=="U1"||kind=="U2")&&e>=f&&e<end){auto bad=tx.uwb_batch.measurements.at(law["fault_anchor_index"].as<int>()).id;
   correct=exclusion&&retained.size()==tx.uwb_batch.measurements.size()-1&&std::find(retained.begin(),retained.end(),bad)==retained.end();}
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
  out<<e<<','<<alarm<<','<<raw_detector.statistic<<','<<raw_detector.threshold<<','<<joint.a.rows()<<','<<joint.a.cols()<<','<<models.hypotheses.size()<<','<<actions.size()<<','<<numerical<<','<<post<<','<<pl_count<<','<<selected<<','<<exclusion<<','<<correct<<','<<receipt.backend_updates<<','<<(tag-truth_tag).norm()<<','<<hpl<<','<<vpl<<','<<reference.x()<<','<<reference.y()<<','<<reference.z()<<','<<tag.x()<<','<<tag.y()<<','<<tag.z()<<','<<core<<','<<commit<<','<<(core+commit)<<','<<joint.cross_norm<<','<<joint.integrations<<','<<status<<','<<task_risk<<",0,0,0,"<<inc<<','<<rot<<','<<(state.velocity_world_mps-truth.velocity_world_mps).norm()<<','<<se3_translation<<','<<final_available<<','<<!detector.passed<<','<<tag_hpl<<','<<tag_vpl<<",UNPROTECTED\n";
  previous_body=state.position_world_m;previous_tag=tag;previous_truth=truth;previous_rotation=state.q_world_body;
 }
 std::cout<<"scene="<<kind<<" scope="<<scope<<" seed="<<seed<<" epochs="<<scene.epochs<<" alarms="<<alarms<<" exclusions="<<exclusions<<" no_selected="<<failures<<" raw_qualified=false\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
