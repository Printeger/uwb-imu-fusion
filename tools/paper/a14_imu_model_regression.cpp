// A14 engineering regression. A12/A13 restoration gate reused verbatim.
// No optimizer construction, navigation solve, or iterate entry point.
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <array>
#include <Eigen/Eigenvalues>
#include "uifgo/paper_run_io.h"
#include "uifgo/paper_stage2_cache.h"
#include "uifgo/nlos_discovery.h"
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include "uifgo/config.h"
#include "uifgo/hash_utils.h"
#include "uifgo/graph_builder.h"
#include "uifgo/initializer.h"
#include "uifgo/imu_preint.h"
#include "uifgo/paper_input.h"
#include "uifgo/paper_pose_prior_factor.h"
#include "uifgo/t07_scenario_cache.h"
#include "uifgo/nlos_inference.h"
#include "uifgo/nlos_solver_utils.h"
#include "uifgo/uwb_factor.h"
using namespace gtsam;
using Row = std::map<std::string,std::string>;
std::vector<std::string> split(std::string line) {
  if (!line.empty() && line.back()=='\r') line.pop_back();
  std::vector<std::string> out; std::stringstream s(line); std::string v;
  while(std::getline(s,v,',')) out.push_back(v);
  if(!line.empty() && line.back()==',') out.emplace_back(); return out;
}
std::vector<Row> read(const std::string& path) {
  std::ifstream f(path); if(!f) throw std::runtime_error("missing "+path);
  std::string line; std::getline(f,line); auto h=split(line); std::vector<Row> rows;
  while(std::getline(f,line)) {auto v=split(line); if(v.size()!=h.size())throw std::runtime_error("CSV columns "+path); Row r; for(size_t i=0;i<h.size();++i)r[h[i]]=v[i]; rows.push_back(r);} return rows;
}
std::ofstream output(const std::string& path) {std::ofstream f(path); if(!f)throw std::runtime_error("cannot write "+path);f<<std::setprecision(17);return f;}
Values loadValues(const std::string& path,const Values& initial) {
 struct Entry {std::string type; std::map<std::string,double> c;};std::map<Key,Entry> parsed;
 for(auto&r:read(path)){Key k=std::stoull(r.at("key_value"));auto&e=parsed[k];if(!e.type.empty()&&e.type!=r.at("value_type"))throw std::runtime_error("type collision");e.type=r.at("value_type");if(!e.c.emplace(r.at("coordinate"),std::stod(r.at("value"))).second)throw std::runtime_error("duplicate value coordinate");}
 Values out;
 for(auto&kv:parsed){auto k=kv.first;auto&e=kv.second;if(!initial.exists(k))throw std::runtime_error("unexpected key");
 if(e.type=="Pose3" && e.c.size()==16){Matrix4 m;for(int i=0;i<4;++i)for(int j=0;j<4;++j)m(i,j)=e.c.at("m"+std::to_string(i)+std::to_string(j));out.insert(k,Pose3(m));}
 else if(e.type=="Vector3" && e.c.size()==3){Vector3 v;for(int i=0;i<3;++i)v[i]=e.c.at("v"+std::to_string(i));out.insert(k,v);}
 else if(e.type=="ConstantBias" && e.c.size()==6){Vector3 a,g;for(int i=0;i<3;++i){a[i]=e.c.at("accel"+std::to_string(i));g[i]=e.c.at("gyro"+std::to_string(i));}out.insert(k,imuBias::ConstantBias(a,g));}
 else throw std::runtime_error("incomplete/unsupported Values");}
 if(out.keys()!=initial.keys())throw std::runtime_error("key mismatch");
 for(auto k:out.keys())if(typeid(out.at(k))!=typeid(initial.at(k))||out.at(k).dim()!=initial.at(k).dim())throw std::runtime_error("type/dim mismatch");
 return out;
}
VectorValues loadDelta(const std::string& path,const Values& initial) {
 auto d=initial.zeroVectors();std::map<std::pair<Key,int>,bool> seen;
 for(auto&r:read(path)){if(r.at("call_index")!="50"||r.at("trial_index_within_call")!="2")throw std::runtime_error("wrong delta selection");Key k=std::stoull(r.at("key_value"));int j=std::stoi(r.at("coordinate"));if(!seen.emplace(std::make_pair(k,j),true).second||j<0||j>=d.at(k).size())throw std::runtime_error("delta duplicate/dim");d.at(k)[j]=std::stod(r.at("value"));}
 if(seen.size()!=615)throw std::runtime_error("incomplete actual delta");return d;
}
VectorValues scale(VectorValues v,double s){for(auto&kv:v)kv.second*=s;return v;}
double dot(const VectorValues& a,const VectorValues& b){double out=0;for(auto&kv:a)out+=kv.second.dot(b.at(kv.first));return out;}
double maxabs(const VectorValues& a){double out=0;for(auto&kv:a)out=std::max(out,kv.second.cwiseAbs().maxCoeff());return out;}
std::string keys(const NonlinearFactor& f){std::string s;for(auto k:f.keys()){if(!s.empty())s+=';';s+=DefaultKeyFormatter(k)+":"+std::to_string(k);}return s;}
void matrix(const std::string& path,const Matrix& m){auto f=output(path);for(int i=0;i<m.rows();++i){for(int j=0;j<m.cols();++j){if(j)f<<',';f<<m(i,j);}f<<'\n';}}
void station(std::ostream& f,const uifgo::NavigationStationarityAudit& a){f<<a.valid<<','<<a.stationary<<','<<a.max_pose_rotation_gradient_objective_per_rad<<','<<a.max_pose_translation_gradient_objective_per_m<<','<<a.max_velocity_gradient_objective_per_mps<<','<<a.max_accel_bias_gradient_objective_per_mps2<<','<<a.max_gyro_bias_gradient_objective_per_radps<<','<<a.max_scaled_gradient_objective<<','<<a.roundoff_allowance_objective;}

using M15=Eigen::Matrix<double,15,15>;
void dump(std::ostream& f,size_t index,const std::string& name,const Matrix& m) {
  for(int i=0;i<m.rows();++i)for(int j=0;j<m.cols();++j)
    f<<index<<','<<name<<','<<m.rows()<<','<<m.cols()<<','<<i<<','<<j<<','<<m(i,j)<<'\n';
}
int group(Key k,int j){char c=Symbol(k).chr();return c=='x'?(j<3?0:1):(c=='v'?2:(j<3?3:4));}
bool near(double a,double b,double at=2e-12,double rt=2e-12){return std::abs(a-b)<=at+rt*std::abs(b);}

int main(int argc,char**argv) {try {
 if(argc!=4)throw std::runtime_error("usage: CONFIG A13_REFERENCE OUTPUT");
 std::string config=argv[1],ref=argv[2],out=argv[3];
 auto started=std::chrono::steady_clock::now(); auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();};
 {std::ifstream maps("/proc/self/maps");auto f=output(out+"/process_maps.txt");f<<maps.rdbuf();}
 auto cfg=uifgo::ConfigLoader::Load(config);
 auto cache=uifgo::LoadT07ScenarioCache(cfg.t07_cache_manifest,cfg.t07_cache_start_s,cfg.t07_cache_duration_s);
 auto plan=uifgo::BuildPaperInputPlan(cache.uwb,cfg,cache.base_recording_id);
 auto frames=uifgo::MaterializePaperKeyframes(plan,uifgo::AllPlannedObservationMask(plan));
 auto init=uifgo::Initializer(cfg).Run(cache.imu,frames);if(!init.ok)throw std::runtime_error("raw initializer failed");
 NonlinearFactorGraph graph;Values initial;std::vector<size_t> uwb;
 uifgo::GraphBuilder(cfg).Build(frames,cache.imu,init,&graph,&initial,&uwb);
 if(uifgo::ReplacePosePriorsForPaperPath(&graph)!=1)throw std::runtime_error("prior count");
 // Exact first conditional replacement, c=0 before any chain update.
 size_t n=0;for(auto&r:plan.observations)if(r.planned){auto anchor=std::find_if(cfg.anchors.begin(),cfg.anchors.end(),[&](const auto&a){return a.id==r.anchor_id;});if(anchor==cfg.anchors.end())throw std::runtime_error("anchor");graph.at(uwb.at(n++))=uifgo::MakeUwbFactor(Symbol('x',r.keyframe_id),0,0,0,anchor->pos,cfg.lever_arm_init,r.raw_range,r.nominal_sigma,false,false,false,uifgo::FixedBetaForLink(cfg,r.tag_id,r.anchor_id)+0.0);}
 uifgo::InferenceIdentityContext context; context.input_sha256=cache.cache_id; context.config_sha256="sha256:"+uifgo::Sha256FileHex(config); context.input_plan_sha256=plan.plan_sha256; context.support_partition_sha256="t09-common-no-support:"+uifgo::Sha256Hex(plan.plan_sha256);
 auto identity=uifgo::ComputeInferenceContentIdentity(graph,initial,context);
 auto ident=output(out+"/restoration_identity.csv");ident<<"name,value\ninitial_graph_sha,"<<identity.graph_linearization_sha256<<"\ninitial_values_sha,"<<identity.values_sha256<<"\nkeys,"<<initial.size()<<"\nfactors,"<<graph.size()<<"\nuwb_factors,"<<uwb.size()<<'\n';ident.flush();
 if(identity.graph_linearization_sha256!="t08graphlin-sha256:22fab638c30f51770c7d8d34b1b4d4d7bfe4d71ed1812dee75b6b83df2ffe789"||identity.values_sha256!="t08values-sha256:7115e76d406123d3a30079dc115c6d3501ceca157d270eaced2d8be492b7cd3f"||initial.size()!=123||graph.size()!=371||uwb.size()!=328)throw std::runtime_error("RESTORATION_INITIAL_IDENTITY_FAILED");
 auto values=loadValues(ref+"/first_block_call50_values.csv",initial);
 auto checkpoint=uifgo::ComputeInferenceContentIdentity(graph,values,context);
 ident<<"checkpoint_graph_sha,"<<checkpoint.graph_linearization_sha256<<"\ncheckpoint_values_sha,"<<checkpoint.values_sha256<<"\ncheckpoint_error,"<<graph.error(values)<<'\n';
 // A12 original gate, before inspecting any PIM or decomposing curvature.
 if(checkpoint.graph_linearization_sha256!="t08graphlin-sha256:60f7180fb01eca83634129937cede207e8c5c276bfda7ccd96c5eccb679a1e00" || checkpoint.values_sha256!="t08values-sha256:48012c8dd18fb70afaa7abf7bf363edd3363e467a0a145bc05d94f4b157e92db" || !near(graph.error(values),1911.3132066226271))throw std::runtime_error("A12_CHECKPOINT_IDENTITY_FAILED");
 auto audit=uifgo::AuditNavigationStationarity(graph,values,{},1e-6,8);auto expected=read(ref+"/checkpoint_stationarity.csv").at(0);
 auto ast=output(out+"/checkpoint_stationarity.csv");ast<<"valid,stationary,rotation,translation,velocity,accel_bias,gyro_bias,max_scaled_gradient,roundoff\n";station(ast,audit);ast<<'\n';
 const double actualGrad[]={audit.max_pose_rotation_gradient_objective_per_rad,audit.max_pose_translation_gradient_objective_per_m,audit.max_velocity_gradient_objective_per_mps,audit.max_accel_bias_gradient_objective_per_mps2,audit.max_gyro_bias_gradient_objective_per_radps};
 const char* groups[]={"rotation","translation","velocity","accel_bias","gyro_bias"};
 if(!audit.valid)throw std::runtime_error("INVALID_STATIONARITY_AUDIT");
 for(int j=0;j<5;++j)if(!near(actualGrad[j],std::stod(expected.at(groups[j])),2e-10,2e-12))throw std::runtime_error("A12_GRADIENT_MISMATCH");
 auto oldFactors=read(ref+"/factor_identity.csv");
 for(size_t i=0;i<graph.size();++i)if(oldFactors.at(i).at("dynamic_type")!=typeid(*graph[i]).name()||oldFactors.at(i).at("keys")!=keys(*graph[i])||!near(std::stod(oldFactors.at(i).at("error_at_checkpoint")),graph[i]->error(values)))throw std::runtime_error("A12_FACTOR_IDENTITY_MISMATCH");
 ident<<"restoration_gate,PASS\n";ident.flush();
 auto columns=read(ref+"/columns.csv"),direction=read(ref+"/frozen_weak_direction.csv");
 auto u=values.zeroVectors();if(columns.size()!=615||direction.size()!=615)throw std::runtime_error("FROZEN_DIRECTION_SIZE");
 for(size_t j=0;j<615;++j){auto&c=columns[j];auto&d=direction[j];if(d.at("column")!=std::to_string(j)||d.at("key")!=c.at("key")||d.at("coordinate")!=c.at("coordinate"))throw std::runtime_error("FROZEN_DIRECTION_MAPPING");u.at(std::stoull(c.at("key_value")))[std::stoi(c.at("coordinate"))]=std::stod(d.at("dimensionless_component"));}
 if(std::abs(u.norm()-1)>1e-12)throw std::runtime_error("FROZEN_DIRECTION_NORM");

 auto checks=output(out+"/checks.csv");checks<<"name,value,limit,pass\n";int failures=0;
 auto check=[&](std::string name,double value,double limit){bool ok=std::isfinite(value)&&value<=limit;checks<<name<<','<<value<<','<<limit<<','<<ok<<'\n';if(!ok)++failures;};
 auto newcfg=cfg;newcfg.paper_imu_covariance_model=uifgo::ImuCovarianceModel::PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1;
 auto newinit=uifgo::Initializer(newcfg).Run(cache.imu,frames);
 NonlinearFactorGraph ng;Values ni;std::vector<size_t> nw;
 uifgo::GraphBuilder(newcfg,newcfg.paper_imu_covariance_model).Build(frames,cache.imu,newinit,&ng,&ni,&nw);
 uifgo::ReplacePosePriorsForPaperPath(&ng);
 // Same conditional raw factor instances. No old Values enter an optimizer.
 for(size_t j=0;j<uwb.size();++j)ng[nw[j]]=graph[uwb[j]];
 std::string model_reason;
 check("legacy_graph_model_match",!uifgo::PaperImuCovarianceModelMatchesGraph(graph,cfg,&model_reason),0);
 check("new_graph_model_match",!uifgo::PaperImuCovarianceModelMatchesGraph(ng,newcfg,&model_reason),0);
 check("old_graph_new_model_rejected",uifgo::PaperImuCovarianceModelMatchesGraph(graph,newcfg,&model_reason),0);
 check("new_graph_old_model_rejected",uifgo::PaperImuCovarianceModelMatchesGraph(ng,cfg,&model_reason),0);
 // Run the actual discovery/refit entry guards with a deliberately mismatched
 // graph/config. They must return before constructing an optimizer.
 auto blockedDiscovery=uifgo::AutomaticSupportProvider(uifgo::DiscoveryOptions()).Run(
     graph,values,{},plan,newcfg,uifgo::DiscoveryContext());
 check("discovery_cross_model_rejected",blockedDiscovery.reason!="PAPER_IMU_COVARIANCE_MODEL_GRAPH_MISMATCH",0);
 auto blockedRefit=uifgo::SegmentRefitter(uifgo::RefitOptions()).RunFrozenCandidatePolicy(
     graph,values,{},plan,newcfg,uifgo::SupportPartition(),{},false);
 check("refit_cross_model_rejected",blockedRefit.reason!="PAPER_IMU_COVARIANCE_MODEL_GRAPH_MISMATCH",0);
 uifgo::SegmentRefitResult restoreOnly;restoreOnly.graph=graph;restoreOnly.values=values;
 auto blockedFinal=uifgo::FinalInferenceEngine({},{}).Run(graph,{},restoreOnly,{}, {},plan,newcfg,context);
 check("final_cross_model_rejected",blockedFinal.reason!="PAPER_IMU_COVARIANCE_MODEL_GRAPH_MISMATCH",0);
 check("final_cross_model_no_decision",!blockedFinal.decisions.empty(),0);
 restoreOnly.graph=ng;uifgo::SupportPartition wrongContext;wrongContext.calibration_hash="other-model-calibration";
 auto blockedContext=uifgo::FinalInferenceEngine({},{}).Run(ng,{},restoreOnly,wrongContext,{},plan,newcfg,context);
 check("final_context_rejected",blockedContext.reason!="FINAL_SUPPORT_CALIBRATION_IMU_CONTEXT_MISMATCH",0);
 check("final_context_no_decision",!blockedContext.decisions.empty(),0);
 auto nid=uifgo::ComputeInferenceContentIdentity(ng,ni,context);
 check("initial_values_exact",nid.values_sha256!=identity.values_sha256,0);
 check("initial_graph_changed",nid.graph_linearization_sha256==identity.graph_linearization_sha256,0);
 auto nc=uifgo::ComputeInferenceContentIdentity(ng,values,context);
 check("checkpoint_graph_changed",nc.graph_linearization_sha256==checkpoint.graph_linearization_sha256,0);
 auto mats=output(out+"/matrices.csv");
 mats<<"factor_index,name,nrows,ncols,row,col,value\n";
 auto deriv=output(out+"/derivatives.csv");deriv<<"factor_index,h,residual_fd_max_error,residual_Jd_max,objective_analytic,objective_fd\n";
 size_t count=0;std::vector<double> hs={1e-4,1e-5,1e-6};
 for(size_t fi=0;fi<ng.size();++fi){
   auto old=boost::dynamic_pointer_cast<CombinedImuFactor>(graph[fi]);
   auto fresh=boost::dynamic_pointer_cast<CombinedImuFactor>(ng[fi]);
   if(!old){check("unchanged_nonimu_"+std::to_string(fi),graph[fi]->error(values)!=ng[fi]->error(values),0);continue;}
   ++count;const auto& p=old->preintegratedMeasurements();const auto&q=fresh->preintegratedMeasurements();
   std::string tag=std::to_string(fi);
   check("legacy_K_"+tag,(p.p().biasAccOmegaInt-I_6x6).cwiseAbs().maxCoeff(),0);
   check("new_K_"+tag,q.p().biasAccOmegaInt.cwiseAbs().maxCoeff(),0);
   check("mean_exact_"+tag,(p.preintegrated()-q.preintegrated()).cwiseAbs().maxCoeff(),0);
   check("mean_accJac_exact_"+tag,(p.preintegrated_H_biasAcc()-q.preintegrated_H_biasAcc()).cwiseAbs().maxCoeff(),0);
   check("mean_gyroJac_exact_"+tag,(p.preintegrated_H_biasOmega()-q.preintegrated_H_biasOmega()).cwiseAbs().maxCoeff(),0);
   check("biasHat_exact_"+tag,(p.biasHat().vector()-q.biasHat().vector()).cwiseAbs().maxCoeff(),0);
   std::vector<Matrix> a(6),b(6);auto re=old->unwhitenedError(values,a);auto rn=fresh->unwhitenedError(values,b);
   check("unwhite_residual_exact_"+tag,(re-rn).cwiseAbs().maxCoeff(),0);
   Matrix H(15,30);int c=0;for(size_t j=0;j<6;++j){check("unwhite_J_exact_"+tag+"_"+std::to_string(j),(a[j]-b[j]).cwiseAbs().maxCoeff(),0);H.middleCols(c,b[j].cols())=b[j];c+=b[j].cols();}
   dump(mats,fi,"new_covariance",q.preintMeasCov());dump(mats,fi,"new_mean",q.preintegrated());dump(mats,fi,"new_unwhitened_H",H);dump(mats,fi,"new_unwhitened_residual",rn);
   auto noise=boost::dynamic_pointer_cast<noiseModel::Gaussian>(fresh->noiseModel());
   if(!noise)throw std::runtime_error("NON_GAUSSIAN_NEW_IMU");dump(mats,fi,"new_R",noise->R());
   const auto cov=q.preintMeasCov();double cm=cov.cwiseAbs().maxCoeff();
   check("symmetry_"+tag,(cov-cov.transpose()).cwiseAbs().maxCoeff(),1e-15+1e-12*cm);
   Eigen::SelfAdjointEigenSolver<Matrix> eig(cov);
   check("PSD_"+tag,-eig.eigenvalues().minCoeff(),1e-12*eig.eigenvalues().cwiseAbs().maxCoeff());
   Eigen::LLT<Matrix> llt(cov);check("LLT_"+tag,llt.info()!=Eigen::Success,0);
   check("whitening_"+tag,(noise->R()*cov*noise->R().transpose()-Matrix::Identity(15,15)).cwiseAbs().maxCoeff(),1e-8);
   for(auto model:{uifgo::ImuCovarianceModel::LEGACY_GTSAM_COMBINED_DEFAULT_V1,newcfg.paper_imu_covariance_model}){
     const auto& target=model==newcfg.paper_imu_covariance_model?q:p;
     uifgo::ImuPreintegrator reset(cfg,init.gravity_world,model);
     // Exercise Reset after nonempty integration and a different linearization bias.
     reset.Integrate(Vector3(1,2,3),Vector3(.1,.2,.3),.005);
     reset.Reset(imuBias::ConstantBias(Vector3(.2,.1,-.1),Vector3(.01,.02,.03)));
     check("reset_cov_zero_"+tag+uifgo::ImuCovarianceModelName(model),reset.Pim().preintMeasCov().cwiseAbs().maxCoeff(),0);
     reset.Reset(target.biasHat());
     auto ki=Symbol(fresh->key1()).index(),kj=Symbol(fresh->key3()).index();
     uifgo::IntegrateBetween(cache.imu,0,frames[ki].t,frames[kj].t,&reset);
     check("reset_replay_cov_"+tag+uifgo::ImuCovarianceModelName(model),(reset.Pim().preintMeasCov()-target.preintMeasCov()).cwiseAbs().maxCoeff(),0);
     check("reset_replay_mean_"+tag+uifgo::ImuCovarianceModelName(model),(reset.Pim().preintegrated()-target.preintegrated()).cwiseAbs().maxCoeff(),0);
     check("reset_K_"+tag+uifgo::ImuCovarianceModelName(model),(reset.Params()->biasAccOmegaInt-target.p().biasAccOmegaInt).cwiseAbs().maxCoeff(),0);
   }
   Vector local(30);c=0;for(auto k:fresh->keys())for(int j=0;j<u.at(k).size();++j)local[c++]=u.at(k)[j];
   Vector jd=noise->R()*H*local;double ana=jd.dot(noise->R()*rn);
   for(double h:hs){auto plus=values.retract(scale(u,h)),minus=values.retract(scale(u,-h));
     Vector fd=noise->R()*(fresh->unwhitenedError(plus)-fresh->unwhitenedError(minus))/(2*h);
     double err=(fd-jd).cwiseAbs().maxCoeff(),limit=1e-6+1e-5*jd.cwiseAbs().maxCoeff();
     check("native_retract_"+tag+"_"+std::to_string(h),err,limit);
     deriv<<fi<<','<<h<<','<<err<<','<<jd.cwiseAbs().maxCoeff()<<','<<ana<<','<<(fresh->error(plus)-fresh->error(minus))/(2*h)<<'\n';
   }
 }
 check("pim_count",count!=40,0);
 auto gradient=ng.linearize(values)->gradientAtZero();double ga=dot(gradient,u);
 for(double h:hs){double fd=(ng.error(values.retract(scale(u,h)))-ng.error(values.retract(scale(u,-h))))/(2*h);check("graph_native_retract_"+std::to_string(h),std::abs(fd-ga),1e-3+1e-5*std::abs(ga));deriv<<-1<<','<<h<<",,,"<<ga<<','<<fd<<'\n';}
 auto ids=output(out+"/model_identities.csv");ids<<"model,imu,common,stage2_cache,final_request,context\n";
 std::vector<std::string> commons,caches,finals,contexts,partitions;
 for(auto model:{uifgo::ImuCovarianceModel::LEGACY_GTSAM_COMBINED_DEFAULT_V1,newcfg.paper_imu_covariance_model}){
   bool isnew=model==newcfg.paper_imu_covariance_model;
   auto im=uifgo::ImuCovarianceModelIdentity(cfg,init.gravity_world,model);
   uifgo::CommonPreparationIdentityInput ci;ci.raw_source_sha256=cache.cache_id;ci.window_role_split_cutoff_sha256="P1-development";ci.observation_ledger_sha256=plan.plan_sha256;ci.input_plan_sha256=plan.plan_sha256;ci.nominal_sigma_sha256=plan.plan_sha256;ci.initialization_rule="T02_SHARED_INITIALIZER_V1";ci.initial_values_sha256=identity.values_sha256;ci.calibration_sha256=im;ci.physical_graph_sha256=isnew?nid.graph_linearization_sha256:identity.graph_linearization_sha256;ci.factor_metadata_sha256="same-meta";ci.solver_precision_sha256="same-solver";
   auto common=uifgo::ComputeCommonPreparationId(ci);commons.push_back(common);
   uifgo::DiscoveryContext dc;dc.input_plan_hash=plan.plan_sha256;dc.source_hash=cache.cache_id;dc.config_hash="same-config-except-model";dc.calibration_hash=im;dc.solver_config_hash="same-solver";
   auto partition=uifgo::BuildAutomaticSupportPartition({},uifgo::DiscoveryOptions(),dc);partitions.push_back(partition.partition_hash);
   uifgo::Stage2CacheManifest cm;cm.common_preparation_id=common;auto cacheid=uifgo::ComputeStage2CacheId(cm);caches.push_back(cacheid);
   uifgo::RequireStage2CommonPreparation(cm,common);
   uifgo::FinalRequestIdentityInput req;req.stage2_cache_id=cacheid;req.canonical_mode="full_gate";req.policy_version="engineering-identity-only";req.thresholds_sha256="same-thresholds";req.threshold_provenance="NOT_RUN";req.final_refit_score_config_sha256=im;req.solver_sha256="same-solver";req.common_preparation_id=common;
   auto final=uifgo::ComputeFinalRequestId(req);finals.push_back(final);
   auto ctx=context;ctx.calibration_sha256=im;auto content=uifgo::ComputeInferenceContentIdentity(isnew?ng:graph,values,ctx);contexts.push_back(content.context_sha256);
   ids<<uifgo::ImuCovarianceModelName(model)<<','<<im<<','<<common<<','<<cacheid<<','<<final<<','<<content.context_sha256<<'\n';
 }
 check("support_partition_model_separation",partitions[0]==partitions[1],0);
 check("common_model_separation",commons[0]==commons[1],0);check("cache_model_separation",caches[0]==caches[1],0);check("request_model_separation",finals[0]==finals[1],0);check("final_context_model_separation",contexts[0]==contexts[1],0);
 for(int i=0;i<2;++i){uifgo::Stage2CacheManifest cm;cm.common_preparation_id=commons[i];bool rejected=false;try{uifgo::RequireStage2CommonPreparation(cm,commons[1-i]);}catch(const std::runtime_error&){rejected=true;}check("cross_model_rejected_"+std::to_string(i),!rejected,0);}
 auto after=uifgo::ComputeInferenceContentIdentity(graph,values,context);check("legacy_graph_after_exact",after.graph_linearization_sha256!=checkpoint.graph_linearization_sha256,0);check("legacy_values_after_exact",after.values_sha256!=checkpoint.values_sha256,0);
 ident<<"new_initial_graph_sha,"<<nid.graph_linearization_sha256<<"\nnew_initial_values_sha,"<<nid.values_sha256<<"\nnew_checkpoint_graph_sha,"<<nc.graph_linearization_sha256<<"\noptimizer_constructions,0\niterate_calls,0\nfailures,"<<failures<<"\nwall_seconds,"<<elapsed()<<'\n';
 std::cout<<"A14 regression checks completed; failures="<<failures<<" iterate=0\n";return failures?1:0;
 }catch(const std::exception&e){std::cerr<<"A14_FAILED: "<<e.what()<<'\n';return 1;}}
