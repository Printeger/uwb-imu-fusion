#include "scenario_material.hpp"
#include "gaussian_window.hpp"
#include "statistical_history.hpp"
#include "raw_domain.hpp"
#include "bounded_imu_recovery.hpp"
#include "raw_prefix_history.hpp"
#include "online_gaussian_history.hpp"
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
 if(argc<5)throw std::runtime_error("usage: fde_next_runner CONFIG OUTPUT SCENE SCOPE [seed-or-replay-cache] [epochs] [reference|optimized] [attempts<=35] [continuous]");
 std::string kind=argv[3],scope=argv[4];if(kind!="U0"&&kind!="U1"&&kind!="U2"&&kind!="U3"&&kind!="I1"&&kind!="I2"&&kind!="screen"&&kind!="replay")throw std::runtime_error("unsupported scene");
 if(scope!="off"&&scope!="uwb"&&scope!="imu-diagnostic")throw std::runtime_error("unsupported fault scope: IMU/joint require explicit diagnostic entry");
 auto generation_start=Clock::now();
 auto law=YAML::LoadFile(argv[1]);int seed=argc>5 && kind!="replay"?std::stoi(argv[5]):law["seed"].as<int>();NoisyMotionScenario scene(argv[1],kind,seed,argc>6?std::stoi(argv[6]):0);
 if(kind=="replay"){if(argc<7 || scope!="off")throw std::runtime_error("replay usage: CONFIG OUTPUT replay off CACHE COUNT; unqualified diagnostic only");loadRawCache(scene,argv[5]);}
 auto generation_end=Clock::now();
 bool continuous=argc>9 && std::string(argv[9])=="continuous";
 bool benchmark=argc>8 && !continuous;int attempts=benchmark?std::stoi(argv[8]):1;
 std::string numeric=argc>7?argv[7]:"reference";bool profile=numeric=="profile" || std::getenv("FDE_NEXT_PROFILE")!=nullptr;bool optimized=numeric=="optimized";
 if(numeric!="reference" && numeric!="profile" && numeric!="optimized")throw std::runtime_error("unsupported numerical mode");
 if(attempts<1 || attempts>35 || (benchmark && scope!="uwb"))throw std::runtime_error("invalid benchmark workload");
 double alpha=law["alpha_per_test"].as<double>(),tail=law["noise_tail_per_candidate"].as<double>();
 double task_risk=std::nextafter(2*tail*law["epochs"].as<int>()*law["max_candidates"].as<int>(),INFINITY);
 if(task_risk>law["risk_budget"].as<double>())throw std::runtime_error("risk precheck rejects before replay");
 std::ofstream out(argv[2]);if(!out)throw std::runtime_error("output unavailable");out<<std::setprecision(17);
 out<<"epoch,alarm,statistic,threshold,rows,cols,hypotheses,actions,numerical,post,pl_evaluated,selected,exclusion,correct,updates,error,hpl,vpl,reference_x,reference_y,reference_z,commit_x,commit_y,commit_z,core_ms,commit_ms,cycle_ms,cross_covariance,integrations,status,frozen_union_charge,raw_qualified,formal_eligible,publication_protected,increment_error,rotation_rpe,velocity_error,se3_translation_rpe,final_local_available,legacy_alarm,current_tag_local_hpl,current_tag_local_vpl,future_state_status,raw_total_risk,model,scope,protection_point,timestamp_ns,attempt,end_to_end_ms,cleanup_ms,gaussian_decompositions,proof_entries,selected_action,removed_groups,initial_law_valid,adaptive_event_valid,range_domain_valid,range_remainder_m,raw_domain_status,history_fixed_model_valid,noise_source_columns,reference_faults,source_products,covariance_fallback,attitude_error_rad,detector_dof,summary_bytes,summary_rows,summary_updates,summary_frontier\n";
 std::unique_ptr<std::ofstream> audit;
 if(const char* path=std::getenv("FDE_NEXT_NUMERIC_AUDIT")){
  if(!benchmark || attempts!=1)throw std::runtime_error("numeric audit requires one benchmark snapshot");
  audit=std::make_unique<std::ofstream>(path);if(!*audit)throw std::runtime_error("audit output unavailable");*audit<<std::setprecision(17);
 }
 auto dump=[&](const std::string& label,const Eigen::MatrixXd& m){if(audit){*audit<<label<<' '<<m.rows()<<' '<<m.cols()<<'\n';for(int i=0;i<m.rows();++i)for(int j=0;j<m.cols();++j)*audit<<m(i,j)<<' ';*audit<<'\n';}};
 int exclusions=0,alarms=0,failures=0;
 bool online_reference=std::getenv("FDE_NEXT_ONLINE_SUMMARY")!=nullptr;
 if(online_reference && (scope!="uwb" || (kind!="U0" && kind!="U1" && kind!="U2" && kind!="U3")))throw std::runtime_error("UNAVAILABLE_ONLINE_SUMMARY_SCOPE");
 if(online_reference && std::getenv("FDE_NEXT_USE_PREFIX_REFERENCE"))throw std::runtime_error("choose online summary or dense prefix oracle");
 if(online_reference && std::getenv("FDE_NEXT_NUMERIC_AUDIT"))throw std::runtime_error("online uses independent oracle checks rather than identical raw matrices");
 bool imu_functional=std::getenv("FDE_NEXT_IMU_POSITION_FUNCTIONAL")!=nullptr;
 if(imu_functional && (scope!="imu-diagnostic" || kind!="I2"))throw std::runtime_error("position-functional recovery requires the declared I2 IMU-only synthetic scope");
 for(int attempt=0;attempt<attempts;++attempt){
 Clock::time_point measured_start,estimator_cleanup_start;double initialization_ms=0.,prewarm_ms=0.;auto lifecycle_start=Clock::now();
 Clock::time_point initialized_at;
 double measured_compute=0.;std::string benchmark_record,benchmark_suffix;
 {
 IncrementalUwbImuEstimator estimator(scene.cfg,scene.lever);auto init=kind=="replay"?initialState():scene.initial_observation;
 estimator.initialize(init,scene.cfg.realtime.prior_sigmas);estimator.ingestImu(scene.imu[0]);
 initialized_at=Clock::now();initialization_ms=std::chrono::duration<double,std::milli>(initialized_at-lifecycle_start).count();
 Eigen::Vector3d previous_tag=init.position_world_m+init.q_world_body*scene.lever;auto previous_truth=scene.truth(0);
 Eigen::Quaterniond previous_rotation=init.q_world_body;Eigen::Vector3d previous_body=init.position_world_m;
 TimestampNs quarantine_begin,quarantine_end;bool quarantine_active=false,functional_recovery_seen=false;
 RawPrefixHistory raw_prefix_history;
 std::unique_ptr<OnlineGaussianHistory> online;
 if(online_reference)online=std::make_unique<OnlineGaussianHistory>(scene.cfg.integrity_window.epochs,law["persistent_epochs"].as<int>(),law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
 size_t imu_cursor=1;std::vector<ImuMeasurement> observed_raw_prefix{scene.imu[0]};
 for(int e=1;e<=scene.epochs;++e){auto cycle_start=Clock::now();
 if(continuous && e==14)prewarm_ms=std::chrono::duration<double,std::milli>(cycle_start-initialized_at).count();
 std::string stream_record,stream_suffix;double stream_compute=0.;
 {auto start=cycle_start;
  while(imu_cursor<scene.imu.size() && !(scene.uwb[e-1].timestamp<scene.imu[imu_cursor].timestamp)){observed_raw_prefix.push_back(scene.imu[imu_cursor]);estimator.ingestImu(scene.imu[imu_cursor++]);}
  auto tx=estimator.prepareEpoch(scene.uwb[e-1]);
  if(online)online->prepare(tx);
  bool prefix_reference=std::getenv("FDE_NEXT_USE_PREFIX_REFERENCE")!=nullptr;
  bool prefix_enabled=prefix_reference || std::getenv("FDE_NEXT_RAW_PREFIX_HISTORY")!=nullptr;
  if(prefix_reference && scope=="imu-diagnostic")throw std::runtime_error("prefix Gaussian reference does not include IMU bridge law");
  if(prefix_enabled)raw_prefix_history.observe(tx,scene.cfg.integrity_window.epochs);
  if(((kind=="screen" || benchmark) && e<scene.epochs) || (continuous && e<=13)){auto prewarm_plan=EpochCommitPlan::nominalPlan(tx);if(prefix_enabled)raw_prefix_history.committed(tx,prewarm_plan);if(online)online->committed(tx,prewarm_plan);estimator.commitEpoch(std::move(tx),prewarm_plan);continue;}
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
  GaussianWindow joint;
  auto native_tag_map=worldTagMap(w,tx,scene.lever);
  if((!prefix_reference && !online) || history_check)joint=rawGaussianWindow(w,tx,models.modes,law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
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
  if(std::getenv("FDE_NEXT_RAW_PREFIX_HISTORY") && e>=30 && e<=32){
   auto pw=raw_prefix_history.window(tx);auto pm=raw_prefix_history.material(tx);
   auto pj=rawGaussianWindow(pw,pm,{},law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever);
   int old=15*(e-scene.cfg.integrity_window.epochs);
   // Predetermined earliest-anchor three-epoch episode, including its full
   // frozen history response after onset has left the explicit window.
   auto history_modes=raw_prefix_history.uwbModes(tx,law["persistent_epochs"].as<int>());
   const auto& history_fault=history_modes.at(1);
   Eigen::MatrixXd fault=Eigen::MatrixXd::Zero(pj.a.rows(),1);
   for(size_t k=0;k<pw.blocks.size();++k){auto it=history_fault.raw_group_maps.find(pw.blocks[k].group_id);
    if(it!=history_fault.raw_group_maps.end())fault.middleRows(pj.offsets[k],it->second.rows())=it->second;}

   auto compressed=statisticalHistory(pj.a.leftCols(old),pj.a.rightCols(pj.a.cols()-old),pj.z,pj.sigma,fault,
       pj.noise_j,Eigen::MatrixXd(pj.a.rows(),0));
   auto direct=gaussianReference(pj.a,pj.z,pj.sigma,pj.c,{fault},alpha,tail);
   auto reduced=gaussianReference(compressed.a,compressed.z,compressed.sigma,pj.c.rightCols(pj.c.cols()-old),{compressed.b},alpha,tail);
   std::cout<<"RAW_PREFIX_HISTORY epoch="<<e<<" rows="<<pj.a.rows()<<" cols="<<pj.a.cols()<<" dof="<<direct.dof<<" compressed_dof="<<reduced.dof
     <<" S="<<direct.statistic<<" compressed_S="<<reduced.statistic<<" tau="<<direct.threshold<<" mean_diff="<<(direct.mean.tail(reduced.mean.size())-reduced.mean).norm()
     <<" protected_cov_diff="<<(pj.c*direct.covariance*pj.c.transpose()-pj.c.rightCols(reduced.mean.size())*reduced.covariance*pj.c.rightCols(reduced.mean.size()).transpose()).norm()
     <<" pl_diff="<<(direct.pl-reduced.pl).norm()<<" source_columns="<<compressed.j.cols()<<" cross="<<pj.cross_norm<<" source_groups="<<raw_prefix_history.groups.size()<<" model=FINITE_PREFIX_FROZEN_HISTORY_REFERENCE raw_risk=UNKNOWN\n";
  }
  LinearizedIntegrityWindow prefix_window;std::vector<FaultModeBasis> reference_modes=models.modes;
  if(prefix_reference){
   prefix_window=raw_prefix_history.window(tx);auto prefix_material=raw_prefix_history.material(tx);
   reference_modes=raw_prefix_history.uwbModes(tx,law["persistent_epochs"].as<int>());
   joint=rawGaussianWindow(prefix_window,prefix_material,reference_modes,law["raw_accel_sigma_mps2"].as<double>(),law["raw_gyro_sigma_radps"].as<double>(),scene.lever,optimized);
   joint.history_qualified=true; // fixed finite raw model only, NOT raw law qualification
  }
  OnlineHistoryCounts online_counts;
  if(online){online_counts=online->counts();joint.c=online->tagMap();joint.history_qualified=true;joint.integrations=online_counts.integrations;}
  const auto& reference_window=prefix_reference?prefix_window:w;
  const bool reuse_numerics=optimized || prefix_reference; // S4 optimizations in BOTH HGIR baselines
  auto referenceAction=[&](const ExclusionAction& a){auto adapted=a;
   if(prefix_reference)for(auto& block:adapted.added_blocks){
    Eigen::MatrixXd embedded=Eigen::MatrixXd::Zero(block.jacobian_raw.rows(),joint.a.cols());embedded.rightCols(w.H.cols())=block.jacobian_raw;block.jacobian_raw=std::move(embedded);}
   return adapted;
  };
  auto t_raw=Clock::now();
  auto raw_detector=online?online->evaluate(tx,nullptr,alpha,tail,false).reference:gaussianReference(joint.a,joint.z,joint.sigma,joint.c,{},alpha,tail);
  auto t_detector=Clock::now();
  RawDomainAssessment domain_assessment;
  if(law["raw_event_risk"] && kind!="replay"){
   RawNoiseEvent event(6*(10*law["epochs"].as<int>()+1)+scene.cfg.anchors.size()*law["epochs"].as<int>()+15,law["raw_event_risk"].as<double>());
   domain_assessment.initial_law_valid=scene.initial_law_valid;domain_assessment.adaptive_event_valid=true;domain_assessment.event_risk=event.risk;
   auto d=physicalDomain(e*scene.dt,scene.cfg.realtime.prior_sigmas,event,
       law["bridge_acceleration_bound_mps2"].as<double>(),law["bridge_angular_rate_bound_radps"].as<double>());
   auto current=tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));
   Eigen::Vector3d center=init.position_world_m+e*scene.dt*init.velocity_world_mps;
   double worldp=d.position+(current.translation()-center).norm();
   double theta=d.rotation+Eigen::AngleAxisd(init.q_world_body.conjugate()*Eigen::Quaterniond(current.rotation().matrix())).angle();
   double rho=1.570796326794897*worldp; // inverse SO(3) left-Jacobian bound for theta < pi
   bool valid=scene.initial_law_valid;double remainder=0.,radius=0.;
   Eigen::Vector3d frozen_tag=current.translation()+current.rotation().matrix()*scene.lever;
   std::vector<Eigen::Vector3d> contraction_anchors;std::vector<double> contraction_ranges;
   for(const auto& measurement:tx.uwb_batch.measurements){contraction_anchors.push_back(measurement.anchor_position_m);contraction_ranges.push_back(measurement.range_m);}
   auto contraction=rangeDomainBranches(contraction_anchors,contraction_ranges,event.q*scene.cfg.realtime.range_sigma_m);
   double contracted_radius=0.,contracted_remainder=0.;bool contracted_valid=!contraction.empty();
   for(const auto& branch:contraction){contracted_valid=contracted_valid && branch.valid;
    contracted_radius=std::max(contracted_radius,(frozen_tag-branch.center).norm()+branch.box.norm());}
   for(const auto& anchor:contraction_anchors){double distance=(frozen_tag-anchor).norm();
    if(contracted_radius>=distance){contracted_valid=false;contracted_remainder=INFINITY;}
    else contracted_remainder=std::max(contracted_remainder,upper(contracted_radius*contracted_radius/(2*(distance-contracted_radius))));}
   if(!benchmark && !continuous && (e==14 || e==15 || (e>=30 && e<=32)))std::cout<<"G_TAG_CONTRACTION epoch="<<e<<" branches="<<contraction.size()<<" radius="<<contracted_radius<<" range_rem="<<contracted_remainder<<" valid="<<contracted_valid<<" navigation_domain=UNQUALIFIED raw_risk=UNKNOWN\n";
   for(const auto& measurement:tx.uwb_batch.measurements){auto bound=tagRemainder((frozen_tag-measurement.anchor_position_m).norm(),theta,rho,scene.lever.norm());
    valid=valid && bound.valid;remainder=std::max(remainder,bound.range);radius=std::max(radius,bound.radius);}
   domain_assessment.range_remainder=remainder;domain_assessment.tag_radius=radius;domain_assessment.range_domain_valid=valid;
   domain_assessment.status=!scene.initial_law_valid?"INITIAL_LAW_MISSING":(valid?"PIM_HISTORY_UNIFORM_BOUND_MISSING":"DOMAIN_CROSSES_ANCHOR_OR_ROTATION_CHART");
   double force_norm=0.;for(const auto& sample:tx.raw_imu_slice)force_norm=std::max(force_norm,sample.specific_force_mps2.norm());
   auto pim_rem=healthyIntegrationRemainder(scene.dt,event.q*std::sqrt(3.)*law["raw_gyro_sigma_radps"].as<double>()+d.gyro_bias,
       event.q*std::sqrt(3.)*law["raw_accel_sigma_mps2"].as<double>()+d.accel_bias,force_norm+d.accel_bias);
   if(!benchmark && !continuous && (e==14 || e==15 || (e>=30 && e<=32)))std::cout<<"RAW_DOMAIN epoch="<<e<<" coordinates="<<event.coordinates<<" q="<<event.q<<" risk="<<event.risk<<" p="<<d.position<<" v="<<d.velocity<<" theta="<<theta<<" tag_radius="<<radius<<" range_rem="<<remainder<<" status="<<domain_assessment.status<<" pim_rot_pos_vel_rem="<<pim_rem.rotation<<';'<<pim_rem.position<<';'<<pim_rem.velocity<<" raw_risk=UNKNOWN fixed_position_grid_charge_lower="<<fixedReferenceFamilyCharge(d.position,law["recovery_position_m"].as<double>(),law["epochs"].as<int>(),law["max_candidates"].as<int>(),tail)<<"\n";
  }
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
  bool quarantine_pending=false;
  if(imu_functional && quarantine_active){
   for(const auto& sample:tx.raw_imu_slice)quarantine_pending=quarantine_pending || (!(sample.timestamp<quarantine_begin) && sample.timestamp<quarantine_end);
   if(quarantine_pending){ExclusionAction q;q.id=ExclusionActionId(1001);q.bridge_mode=BridgeMode::GenericKinematic;q.action_model_id="declared_episode_carry_quarantine";
    q.groups_to_remove={tx.imu_group.id};q.groups_to_add={tx.generic_bridge_group.id,tx.generic_bias_continuity_group.id};q.exclusion_cardinality=1;actions={q};}
   else quarantine_active=false;
  }
  if(actions.size()>law["max_candidates"].as<unsigned>())throw std::runtime_error("candidate cap: union proof invalid");
  if(audit){dump("raw_design",joint.a);dump("raw_covariance",joint.sigma);dump("raw_source_map",joint.noise_j);dump("raw_auxiliary_covariance",joint.auxiliary_sigma);dump("tag_map",joint.c);
   for(size_t i=0;i<joint.faults.size();++i){dump("fault_"+std::to_string(i),joint.faults[i]);*audit<<"#mode "<<reference_modes[i].id.value()<<' '<<reference_modes[i].physical_source_id<<'\n';}}
  if(e==2 && !online){
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
  GaussianCandidateCache gaussian_cache(joint,reference_window,alpha,tail);size_t decompositions=1;
  auto candidateReference=[&](const ExclusionAction& action,bool fast,const GaussianReference* root,GaussianCandidateCache* cache){
   if(online){++decompositions;try{return online->evaluate(tx,&action,alpha,tail,true).reference;}catch(const std::runtime_error&){return GaussianReference{};}}
   return gaussianCandidate(joint,reference_window,referenceAction(action),alpha,tail,fast,root,cache,&decompositions);
  };
  int numerical=0,post=0,pl_count=0;int best=-1;double best_cost=INFINITY;GaussianReference selected_ref;Eigen::VectorXd best_increment;
  std::string status="NO_ADMISSIBLE_CANDIDATE";
  int functional_best=-1,functional_numerical=0,functional_post=0;double functional_cost=INFINITY;
  Eigen::Vector3d functional_reference=Eigen::Vector3d::Constant(NAN),functional_pl=Eigen::Vector3d::Constant(INFINITY);
  TimestampNs functional_begin,functional_end;
  auto positionReference=[&](const Eigen::Vector3d& center){
   Eigen::MatrixXd design(tx.uwb_batch.measurements.size(),3);Eigen::VectorXd rhs(design.rows());
   int row=0;for(const auto& m:tx.uwb_batch.measurements){auto diff=(center-m.anchor_position_m).eval();design.row(row)=diff.normalized().transpose();rhs(row++)=m.range_m-diff.norm();}
   return gaussianReference(design,rhs,scene.cfg.realtime.range_sigma_m*scene.cfg.realtime.range_sigma_m*Eigen::MatrixXd::Identity(design.rows(),design.rows()),Eigen::Matrix3d::Identity(),{},alpha,tail);
  };
  for(size_t k=0;k<actions.size();++k){
   if(scope=="imu-diagnostic" && (alarm || quarantine_pending) && (std::getenv("FDE_NEXT_BOUNDED_RECOVERY") || imu_functional) && actions[k].bridge_mode!=BridgeMode::None){
    bool complete=quarantine_pending || !actions[k].covered_modes.empty();for(auto id:actions[k].covered_modes){auto it=std::find_if(models.modes.begin(),models.modes.end(),[&](const auto& m){return m.id==id;});complete=complete && it!=models.modes.end() && episodeCoverageComplete(*it,actions[k]);}
    if(complete){auto recovery=boundedRecovery(tx,actions[k],init,law["bridge_acceleration_bound_mps2"].as<double>(),law["bridge_angular_rate_bound_radps"].as<double>(),&observed_raw_prefix,&scene.cfg.bridge.generic);
     std::cout<<"BOUNDED_RECOVERY epoch="<<e<<" action="<<actions[k].id.value()<<" status="<<recovery.status<<" rebuilds="<<recovery.rebuilds<<" removed="<<recovery.removed<<" retained="<<recovery.retained
       <<" objective="<<recovery.objective_before<<"->"<<recovery.objective_after<<" local="<<recovery.local.transpose()<<" cumulative="<<recovery.cumulative.transpose();
     if(recovery.rebuilds==2 && std::isfinite(recovery.objective_after))std::cout<<" bridge_sd="<<recovery.bridge.covariance.diagonal().cwiseMax(0.).cwiseSqrt().transpose()<<" bridge_box="<<recovery.bridge.box.transpose();
     if(imu_functional && recovery.rebuilds==2 && std::isfinite(recovery.objective_after)){
      bool trust=true;for(int j=0;j<5;++j)trust=trust && recovery.cumulative(j)<=scales[j] && recovery.local(j)<=scales[j];
      if(trust){++functional_numerical;auto pose_ref=recovery.reference.at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));
       Eigen::Vector3d tag_ref=pose_ref.translation()+pose_ref.rotation().matrix()*scene.lever;auto rp=positionReference(tag_ref);
       if(rp.status=="FROZEN_MODEL_ONLY" && rp.mean.norm()<=scales[1]){++functional_post;
        double cost=1000.*actions[k].exclusion_cardinality+rp.pl.norm();
        if(rp.pl.head<2>().norm()<=scene.cfg.risk_v2.horizontal_alert_limit_m && rp.pl.z()<=scene.cfg.risk_v2.vertical_alert_limit_m && cost<functional_cost){
         functional_cost=cost;functional_best=k;functional_reference=tag_ref+rp.mean;functional_pl=rp.pl;
         if(quarantine_pending){functional_begin=quarantine_begin;functional_end=quarantine_end;}
         else{long long begin=std::numeric_limits<long long>::max(),end=0;
          for(auto mode_id:actions[k].covered_modes)for(const auto& unit:models.units)if(unit.id.value()==mode_id.value()){begin=std::min(begin,(long long)unit.time_begin.value());end=std::max(end,(long long)unit.time_end.value());}
          functional_begin=TimestampNs(begin);functional_end=TimestampNs(end);
         }
        }
       }
      }
     }
     std::cout<<" healthy_rank="<<recovery.healthy_rank<<" healthy_columns="<<recovery.healthy_columns<<" bridge_domain_ratio="<<recovery.full_state_domain_ratio<<" candidate_backend_updates=0\n";
    }
   }
   auto reference_action=referenceAction(actions[k]);
   auto c=evaluator.evaluate(admission,base,actions[k]);
   bool domain=true;Eigen::VectorXd block_max=Eigen::VectorXd::Zero(5);
   if(c.state_increment.size()%15)domain=false;
   for(int q=0;q+15<=c.state_increment.size();q+=15){
    const int offsets[5]={0,3,6,9,12};
    for(int j=0;j<5;++j){double norm=c.state_increment.segment(q+offsets[j],3).norm();block_max(j)=std::max(block_max(j),norm);domain=domain && norm<=scales[j];}}
   if(!domain){c.valid=false;c.reason="research per-state physical trust domain exceeded";}
   if(!benchmark && !continuous && (alarm || (!domain && e==3))){
    std::cerr<<"epoch="<<e<<" action="<<actions[k].id.value()<<" block_rot_pos_vel_ba_bg="<<block_max.transpose();
    if(scope=="imu-diagnostic"){std::cerr<<" sources=";for(const auto& source:actions[k].physical_source_ids)std::cerr<<source<<';';std::cerr<<" remove=";for(auto id:actions[k].groups_to_remove)std::cerr<<id.value()<<';';}
    std::cerr<<'\n';
   }
   if(!c.valid){if(!benchmark && !continuous && alarm)std::cerr<<"epoch="<<e<<" action="<<actions[k].id.value()<<" invalid="<<c.reason<<" step="<<c.state_increment.norm()<<'\n';continue;}
   ++numerical;
   if(scope=="imu-diagnostic" && actions[k].bridge_mode!=BridgeMode::None){
    for(auto id:actions[k].covered_modes){auto it=std::find_if(models.modes.begin(),models.modes.end(),[&](const auto& m){return m.id==id;});
     if(it==models.modes.end() || !episodeCoverageComplete(*it,actions[k]))throw std::runtime_error("IMU candidate omits raw/history support");}
    std::cerr<<"epoch="<<e<<" action="<<actions[k].id.value()<<" removed_groups=";for(auto id:actions[k].groups_to_remove)std::cerr<<id.value()<<';';
    std::cerr<<" native_post="<<JointWindowDetector{}.evaluateCandidate(w,c,dr).passed<<" bridge_covariance=UNQUALIFIED\n";
   }
   auto r=candidateReference(actions[k],reuse_numerics,reuse_numerics?&raw_detector:nullptr,reuse_numerics?&gaussian_cache:nullptr);++pl_count;
   if(r.status!="FROZEN_MODEL_ONLY"){status=scope=="imu-diagnostic" && actions[k].bridge_mode!=BridgeMode::None?"BRIDGE_GAUSSIAN_UNQUALIFIED":r.status;continue;}
   if(r.statistic>r.threshold)continue;
   ++post;
   bool reference_domain=true;
   if(online){auto steps=online->referenceSteps(r);for(int j=0;j<5;++j)reference_domain=reference_domain && steps(j)<=scales[j];}
   else for(int q=0;q+15<=r.mean.size();q+=15)for(int j=0;j<5;++j)reference_domain=reference_domain && r.mean.segment(q+3*j,3).norm()<=scales[j];
   if(!reference_domain){status="REFERENCE_DOMAIN_EXCEEDED";continue;}
   auto pl=(r.pl+(native_tag_map*c.state_increment-joint.c*r.mean).cwiseAbs()).eval();
   if(reuse_numerics && (std::abs(pl.head<2>().norm()-scene.cfg.risk_v2.horizontal_alert_limit_m)<1e-6 || std::abs(pl.z()-scene.cfg.risk_v2.vertical_alert_limit_m)<1e-6)){
    if(online){status="INDETERMINATE_AL_OR_TIE";continue;}
    r=candidateReference(actions[k],false,nullptr,nullptr);pl=r.pl+(native_tag_map*c.state_increment-joint.c*r.mean).cwiseAbs();
   }
   if(pl.head<2>().norm()>scene.cfg.risk_v2.horizontal_alert_limit_m || pl.z()>scene.cfg.risk_v2.vertical_alert_limit_m) {status="AL_EXCEEDED";continue;}
   double cost=actions[k].exclusion_cardinality*1000.+pl.norm();
   if(reuse_numerics && best>=0 && std::abs(cost-best_cost)<1e-7 &&
      (actions[k].groups_to_remove!=actions[best].groups_to_remove || actions[k].groups_to_add!=actions[best].groups_to_add)){
    if(online){best=-1;best_cost=INFINITY;status="INDETERMINATE_AL_OR_TIE";continue;}
    auto old=candidateReference(actions[best],false,nullptr,nullptr);
    if(online){status="INDETERMINATE_AL_OR_TIE";continue;}
    r=candidateReference(actions[k],false,nullptr,nullptr);
    if(old.status!="FROZEN_MODEL_ONLY" || r.status!="FROZEN_MODEL_ONLY"){best=-1;best_cost=INFINITY;status="INDETERMINATE_TIE";continue;}
    best_cost=actions[best].exclusion_cardinality*1000.+(old.pl+(native_tag_map*best_increment-joint.c*old.mean).cwiseAbs()).norm();selected_ref=std::move(old);
    pl=r.pl+(native_tag_map*c.state_increment-joint.c*r.mean).cwiseAbs();cost=actions[k].exclusion_cardinality*1000.+pl.norm();
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
  // After a committed recovery, refresh the current-tag reference from this
  // epoch's healthy UWB only. Never reuse an older PL or grant navigation PL.
  if(imu_functional && functional_recovery_seen && !quarantine_pending){
   auto nominal_pose=tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));Eigen::Vector3d center=nominal_pose.translation()+nominal_pose.rotation().matrix()*scene.lever;
   auto rp=positionReference(center);
   if(rp.status=="FROZEN_MODEL_ONLY" && rp.mean.norm()<=scales[1] && rp.pl.head<2>().norm()<=scene.cfg.risk_v2.horizontal_alert_limit_m && rp.pl.z()<=scene.cfg.risk_v2.vertical_alert_limit_m){
    ExclusionAction keep;keep.id=ExclusionActionId(1);keep.action_model_id="current_tag_only_continuation";actions.push_back(keep);functional_best=actions.size()-1;
    functional_reference=center+rp.mean;functional_pl=rp.pl;++functional_numerical;++functional_post;
   }
  }
  bool functional_selected=imu_functional && functional_best>=0;
  if(functional_selected){best=functional_best;numerical+=functional_numerical;post+=functional_post;pl_count+=functional_post;
   if(actions[best].bridge_mode!=BridgeMode::None){quarantine_begin=functional_begin;quarantine_end=functional_end;quarantine_active=quarantine_begin<quarantine_end;}
   functional_recovery_seen=true;
   std::cout<<"IMU_POSITION_FUNCTIONAL epoch="<<e<<" action="<<actions[best].id.value()<<" quarantine_ns="<<quarantine_begin.value()<<';'<<quarantine_end.value()<<" current_tag_reference="<<functional_reference.transpose()<<" reference_pl="<<functional_pl.transpose()<<" navigation=UNPROTECTED raw_risk=UNKNOWN\n";
   status="CURRENT_POSITION_CONDITIONAL_NAVIGATION_UNPROTECTED";
  }
  // Flat geometry may have a nonlinear mirror even if the local Jacobian is
  // full rank. Local PL does not choose the physical branch.
  if(kind=="U3"){best=-1;status="GEOMETRY_GLOBAL_BRANCH_UNQUALIFIED";}
  if(scope=="off"){best=-1;status=kind=="replay"?"RAWDATA_UNQUALIFIED":"FDE_OFF";}
  bool selected=best>=0;auto plan=selected?planFor(tx,actions[best]):EpochCommitPlan::nominalPlan(tx);plan.best_effort_integrity_unavailable=!selected;
  bool exclusion=selected&&!actions[best].groups_to_remove.empty();exclusions+=exclusion;
  auto pose=tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));
  Eigen::Vector3d reference=functional_selected?functional_reference:(selected?Eigen::Vector3d(pose.translation()+pose.rotation().matrix()*scene.lever+joint.c*selected_ref.mean):Eigen::Vector3d::Constant(NAN));
  std::vector<MeasurementId> retained;for(const auto& g:tx.uwb_groups)if(std::find(plan.groups_to_add.begin(),plan.groups_to_add.end(),g.id)!=plan.groups_to_add.end())retained.insert(retained.end(),g.source_measurements.begin(),g.source_measurements.end());
  int correct=-1;int f=law["fault_epoch"].as<int>(),end=f+(kind=="U2"?law["persistent_epochs"].as<int>():1);
  if((kind=="U1"||kind=="U2")&&e>=f&&e<end){auto bad=tx.uwb_batch.measurements.at(law["fault_anchor_index"].as<int>()).id;
   correct=exclusion&&retained.size()==tx.uwb_batch.measurements.size()-1&&std::find(retained.begin(),retained.end(),bad)==retained.end();}
  if(kind=="I2" && selected){
   // Evaluation oracle only, after action selection. No truth enters recovery.
   TimestampNs truth_begin((law["fault_epoch"].as<int>()-1)*50000000LL+5000000LL),truth_end(truth_begin.value()+500000000LL);
   auto touched=[&](const std::vector<ImuMeasurement>& raw){for(const auto& sample:raw)if(!(sample.timestamp<truth_begin) && sample.timestamp<truth_end)return true;return false;};
   bool isolation=true,has_bad=touched(tx.raw_imu_slice);
   if(has_bad)isolation=std::find(plan.groups_to_add.begin(),plan.groups_to_add.end(),tx.imu_group.id)==plan.groups_to_add.end();
   for(const auto& past:tx.recoverable_history)if(touched(past.raw_imu_slice))for(const auto& g:past.groups)if(g.kind==FactorKind::CombinedImu && std::find(past.selected_groups.begin(),past.selected_groups.end(),g.id)!=past.selected_groups.end()){
    has_bad=true;isolation=isolation && std::find(plan.groups_to_remove.begin(),plan.groups_to_remove.end(),g.id)!=plan.groups_to_remove.end();}
   if(has_bad)correct=exclusion && isolation;
  }
  if(audit){
   Eigen::VectorXd add(plan.groups_to_add.size()),remove(plan.groups_to_remove.size());
   for(size_t i=0;i<plan.groups_to_add.size();++i)add(i)=plan.groups_to_add[i].value();
   for(size_t i=0;i<plan.groups_to_remove.size();++i)remove(i)=plan.groups_to_remove[i].value();
   dump("actual_recipe_add",add);dump("actual_recipe_remove",remove);
  }
  if(benchmark && (!selected || (kind=="U1" && correct!=1)))throw std::runtime_error("benchmark requires successful declared normal/recovery workload");
  if(online){online_counts=online->counts();decompositions=online_counts.state_decompositions;joint.source_products=online_counts.fault_products;}
  if(prefix_enabled)raw_prefix_history.committed(tx,plan);
  if(online)online->committed(tx,plan);
  auto core_end=Clock::now();auto receipt=estimator.commitEpoch(std::move(tx),plan);auto state=estimator.currentState();Eigen::Vector3d tag=state.position_world_m+state.q_world_body*scene.lever;
  Eigen::Vector3d pl=selected?Eigen::Vector3d((functional_selected?functional_pl:selected_ref.pl)+(tag-reference).cwiseAbs()):Eigen::Vector3d::Constant(INFINITY);
  double hpl=pl.head<2>().norm(),vpl=pl.z();if(selected && !functional_selected)status="LINEARIZATION_UNQUALIFIED";else if(functional_selected)status="CURRENT_POSITION_CONDITIONAL_NAVIGATION_UNPROTECTED";else {++failures;if(scope=="imu-diagnostic" && alarm && numerical==0)status="IMU_CANDIDATE_DOMAIN_OR_PROVENANCE_FAILED";}
  auto truth=scene.truth(e*scene.dt);Eigen::Vector3d truth_tag=truth.position_world_m+truth.q_world_body*scene.lever;
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
  double attitude_error=Eigen::AngleAxisd(state.q_world_body.conjugate()*truth.q_world_body).angle();
  double error=(tag-truth_tag).norm(),velocity_error=(state.velocity_world_mps-truth.velocity_world_mps).norm();
  if(kind=="replay")error=velocity_error=inc=rot=se3_translation=NAN;
  std::ostringstream row;row<<std::setprecision(17)<<e<<','<<alarm<<','<<raw_detector.statistic<<','<<raw_detector.threshold<<','<<(online?online_counts.original_rows:joint.a.rows())<<','<<(online?online_counts.active_states:joint.a.cols())<<','<<models.hypotheses.size()<<','<<actions.size()<<','<<numerical<<','<<post<<','<<pl_count<<','<<selected<<','<<exclusion<<','<<correct<<','<<receipt.backend_updates<<','<<error<<','<<hpl<<','<<vpl<<','<<reference.x()<<','<<reference.y()<<','<<reference.z()<<','<<tag.x()<<','<<tag.y()<<','<<tag.z()<<','<<core<<','<<commit<<','<<(core+commit)<<','<<joint.cross_norm<<','<<joint.integrations<<','<<status<<','<<task_risk<<",0,0,0,"<<inc<<','<<rot<<','<<velocity_error<<','<<se3_translation<<','<<final_available<<','<<!detector.passed<<','<<tag_hpl<<','<<tag_vpl<<",UNPROTECTED,UNKNOWN,"<<(functional_selected?"fde-hgir-imu-current-position-v1":(online?"fde-online-sufficient-summary-v1":(prefix_reference?"fde-hgir-finite-raw-prefix-v1":law["model_version"].as<std::string>())))<<','<<(imu_functional?"imu-current-position-functional":scope)<<",uwb_tag_world,"<<state.timestamp.value();
  if(benchmark || continuous){benchmark_record=row.str();measured_compute=core+commit;
   std::ostringstream suffix;suffix<<decompositions<<','<<arena.entryCount()<<','<<(selected?actions[best].id.value():0)<<',';
   if(selected)for(auto id:actions[best].groups_to_remove)suffix<<id.value()<<';';
   suffix<<','<<domain_assessment.initial_law_valid<<','<<domain_assessment.adaptive_event_valid<<','<<domain_assessment.range_domain_valid<<','<<domain_assessment.range_remainder<<','<<domain_assessment.status<<','<<joint.history_qualified<<','<<(online?online_counts.source_columns:joint.noise_j.cols())<<','<<(online?online_counts.fault_modes:joint.faults.size())<<','<<joint.source_products<<','<<joint.covariance_fallback<<','<<attitude_error<<','<<raw_detector.dof<<','<<(online?online_counts.bytes:0)<<','<<(online?online_counts.history_rows:0)<<','<<(online?online_counts.orthogonal_updates:0)<<','<<(online?online_counts.frontier_epoch:0);
   benchmark_suffix=suffix.str();
   if(continuous){stream_record=benchmark_record;stream_suffix=benchmark_suffix;stream_compute=measured_compute;}}
  else {out<<row.str()<<','<<attempt<<','<<(core+commit)<<",0,"<<decompositions<<','<<arena.entryCount()<<','<<(selected?actions[best].id.value():0)<<',';
   if(selected)for(auto id:actions[best].groups_to_remove)out<<id.value()<<';';
   out<<','<<domain_assessment.initial_law_valid<<','<<domain_assessment.adaptive_event_valid<<','<<domain_assessment.range_domain_valid<<','<<domain_assessment.range_remainder<<','<<domain_assessment.status<<','<<joint.history_qualified<<','<<(online?online_counts.source_columns:joint.noise_j.cols())<<','<<(online?online_counts.fault_modes:joint.faults.size())<<','<<joint.source_products<<','<<joint.covariance_fallback<<','<<attitude_error<<','<<raw_detector.dof<<','<<(online?online_counts.bytes:0)<<','<<(online?online_counts.history_rows:0)<<','<<(online?online_counts.orthogonal_updates:0)<<','<<(online?online_counts.frontier_epoch:0)<<'\n';}
  if(online && !benchmark && !continuous && (e==15 || (e>=30 && e<=32)))std::cout<<"ONLINE_SUMMARY epoch="<<e<<" dof="<<raw_detector.dof<<" active_states="<<online_counts.active_states<<" history_rows="<<online_counts.history_rows<<" faults="<<online_counts.fault_modes<<" numeric_bytes_lower="<<online_counts.bytes<<" frontier="<<online_counts.frontier_epoch<<" ortho_updates="<<online_counts.orthogonal_updates<<"\n";
  if(profile){auto ms=[](auto a,auto b){return std::chrono::duration<double,std::milli>(b-a).count();};
   std::cout<<"PROFILE prep="<<ms(start,t_window)<<" models="<<ms(t_window,t_models)<<" raw_cov="<<ms(t_models,t_raw)<<" detector="<<ms(t_raw,t_detector)<<" evidence_actions="<<ms(t_detector,t_actions)<<" candidates="<<ms(t_actions,core_end)<<" commit_final="<<commit<<'\n';}
  previous_body=state.position_world_m;previous_tag=tag;previous_truth=truth;previous_rotation=state.q_world_body;
 } // all epoch numerical products and arena destroyed before final timing
 if(continuous && !stream_record.empty()){
  double total=std::chrono::duration<double,std::milli>(Clock::now()-cycle_start).count();
  out<<stream_record<<",0,"<<total<<','<<(total-stream_compute)<<','<<stream_suffix<<'\n';
 }
 }
 estimator_cleanup_start=Clock::now();
 } // include estimator and numerical-object destruction in attempt cycle
 if(continuous)std::cout<<"STREAM_PHASES generation_ms="<<std::chrono::duration<double,std::milli>(generation_end-generation_start).count()<<" initialization_ms="<<initialization_ms<<" prewarm13_ms="<<prewarm_ms<<" estimator_teardown_ms="<<std::chrono::duration<double,std::milli>(Clock::now()-estimator_cleanup_start).count()<<'\n';
 if(benchmark){double total=std::chrono::duration<double,std::milli>(Clock::now()-measured_start).count();
  out<<benchmark_record<<','<<attempt<<','<<total<<','<<(total-measured_compute)<<','<<benchmark_suffix<<'\n';}
 } // attempts
 std::cout<<"scene="<<kind<<" scope="<<scope<<" seed="<<seed<<" epochs="<<scene.epochs<<" alarms="<<alarms<<" exclusions="<<exclusions<<" no_selected="<<failures<<" raw_qualified=false\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
