// Default-off, single frozen checkpoint diagnostic. No discovery/chain entry point.
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
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
int main(int argc,char**argv) {try {
 if(argc!=6)throw std::runtime_error("usage: CONFIG REFERENCE OUTPUT static|A|B AUTHORIZATION_FILE_OR_NONE");
 std::string config=argv[1],ref=argv[2],out=argv[3],mode=argv[4];
 if(mode!="static"&&mode!="A"&&mode!="B")throw std::runtime_error("mode");
 if(mode!="static"){std::ifstream auth(argv[5]);std::string s((std::istreambuf_iterator<char>(auth)),{});if(s.find("AUTHORIZE_ARM_"+mode)==std::string::npos)throw std::runtime_error("arm not authorized");}
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
 auto calls=read(ref+"/conditional_lm_calls.csv");double lambda=std::stod(calls.at(49).at("lambda_after"));
 LevenbergMarquardtParams external; external.setMaxIterations(50);external.setRelativeErrorTol(1e-6);external.setAbsoluteErrorTol(1e-8);external.setLinearSolverType("SEQUENTIAL_CHOLESKY");
 auto params=external;params.setRelativeErrorTol(0.0);params.setVerbosityLM("TRYDELTA");params.lambdaInitial=lambda;params.diagonalDamping=(mode=="B");
 auto p=output(out+"/lm_parameters.csv");p<<"name,value\nmode,"<<mode<<"\nlambdaInitial,"<<params.lambdaInitial<<"\nlambdaFactor,"<<params.lambdaFactor<<"\nlambdaUpperBound,"<<params.lambdaUpperBound<<"\nlambdaLowerBound,"<<params.lambdaLowerBound<<"\nminModelFidelity,"<<params.minModelFidelity<<"\nuseFixedLambdaFactor,"<<params.useFixedLambdaFactor<<"\nminDiagonal,"<<params.minDiagonal<<"\nmaxDiagonal,"<<params.maxDiagonal<<"\ndiagonalDamping,"<<params.diagonalDamping<<"\ninternalRelativeErrorTol,"<<params.relativeErrorTol<<"\nabsoluteErrorTol,"<<params.absoluteErrorTol<<"\nexternalRelativeErrorTol,"<<external.relativeErrorTol<<"\nerrorTol,"<<params.errorTol<<"\nmaxIterationsParameter,"<<params.maxIterations<<"\nlinearSolverType,SEQUENTIAL_CHOLESKY\norderingType,COLAMD\nverbosityLM,TRYDELTA\n";
 auto audit=uifgo::AuditNavigationStationarity(graph,values,{},1e-6,8);
 auto st=output(out+"/checkpoint_stationarity.csv");st<<"valid,stationary,rotation,translation,velocity,accel_bias,gyro_bias,max_scaled_gradient,roundoff\n";station(st,audit);st<<'\n';
 if(mode=="static") {
   auto base=loadValues(ref+"/conditional_lm_direction_base_values.csv",initial);auto delta=loadDelta(ref+"/conditional_lm_trial_deltas.csv",initial);auto unit=scale(delta,1/delta.norm());
   ident<<"accepted_delta_norm,"<<delta.norm()<<"\naccepted_retract_max_local,"<<maxabs(values.localCoordinates(base.retract(delta)))<<"\nbase_error,"<<graph.error(base)<<'\n';
   auto f=output(out+"/factor_identity.csv");f<<"factor_index,dynamic_type,keys,error_at_base,error_at_checkpoint\n";
   for(size_t i=0;i<graph.size();++i)f<<i<<','<<typeid(*graph[i]).name()<<','<<keys(*graph[i])<<','<<graph[i]->error(base)<<','<<graph[i]->error(values)<<'\n';
   auto fd=output(out+"/direction_fd.csv");fd<<"factor_index,step,analytic,central,abs_diff,tolerance,agrees\n";
   for(double h:{1e-4,1e-5,1e-6}){auto plus=base.retract(scale(unit,h)),minus=base.retract(scale(unit,-h));
     for(int i=-1;i<static_cast<int>(graph.size());++i){double a=i<0?dot(graph.linearize(base)->gradientAtZero(),unit):dot(graph[i]->linearize(base)->gradientAtZero(),unit);double c=i<0?(graph.error(plus)-graph.error(minus))/(2*h):(graph[i]->error(plus)-graph[i]->error(minus))/(2*h);double tol=5e-9+.005*std::abs(a);fd<<i<<','<<h<<','<<a<<','<<c<<','<<std::abs(c-a)<<','<<tol<<','<<(std::abs(c-a)<=tol)<<'\n';}}
   auto linear=graph.linearize(values);auto grad=linear->gradientAtZero();auto diag=linear->hessianDiagonal();Ordering ordering(values.keys());auto jb=linear->jacobian(ordering);matrix(out+"/whitened_J.csv",jb.first);matrix(out+"/whitened_rhs.csv",jb.second);
   auto cols=output(out+"/columns.csv");cols<<"column,key,key_value,value_type,coordinate,group,unit,stationarity_scale,prior_scale,gradient,actual_call50_delta,hessian_diagonal\n";int col=0;
   for(auto k:values.keys()){char s=Symbol(k).chr();for(int j=0;j<grad.at(k).size();++j){std::string group,unitname,type;double prior;
    if(s=='x'){type="Pose3";group=j<3?"rotation":"translation";unitname=j<3?"rad":"m";prior=j<3?.01:.05;}
    else if(s=='v'){type="Vector3";group="velocity";unitname="m/s";prior=.1;}
    else {type="ConstantBias";group=j<3?"accel_bias":"gyro_bias";unitname=j<3?"m/s^2":"rad/s";prior=.01;}
    cols<<col++<<','<<DefaultKeyFormatter(k)<<','<<k<<','<<type<<','<<j<<','<<group<<','<<unitname<<",1,"<<prior<<','<<grad.at(k)[j]<<','<<delta.at(k)[j]<<','<<diag.at(k)[j]<<'\n';}}
   // Test actual linked buildDampedSystem, with zero calls to iterate/solve.
   auto damp=output(out+"/linked_damping.csv");damp<<"arm,column,physical_diagonal,clipped_diagonal,added_diagonal_from_linked_damping_factors,expected,clipped_low,clipped_high\n";
   for(bool diagonal:{false,true}){auto q=params;q.diagonalDamping=diagonal;LevenbergMarquardtOptimizer opt(graph,values,q);auto sqrt=diag;for(auto&kv:sqrt)kv.second=kv.second.cwiseMax(q.minDiagonal).cwiseMin(q.maxDiagonal).cwiseSqrt();auto damped=opt.buildDampedSystem(*linear,sqrt);GaussianFactorGraph additions;for(size_t i=linear->size();i<damped.size();++i)additions.push_back(damped.at(i));auto add=additions.hessianDiagonal();int c=0;for(auto k:values.keys())for(int j=0;j<diag.at(k).size();++j){double d=diag.at(k)[j],clip=std::min(q.maxDiagonal,std::max(q.minDiagonal,d));damp<<(diagonal?'B':'A')<<','<<c++<<','<<d<<','<<clip<<','<<add.at(k)[j]<<','<<lambda*(diagonal?clip:1)<<','<<(d<q.minDiagonal)<<','<<(d>q.maxDiagonal)<<'\n';}}
   ident<<"iterate_calls,0\nwall_seconds,"<<elapsed()<<'\n';return 0;
 }
 LevenbergMarquardtOptimizer optimizer(graph,values,params);
 auto tr=output(out+"/trace.csv");tr<<"call,reference_call,error_before,error_after,lambda_before,lambda_after,accepted,rejected,inner_total,accepted_total,step_norm,step_max,valid,stationary,rotation,translation,velocity,accel_bias,gyro_bias,max_scaled_gradient,roundoff,generic,iterate_seconds,elapsed_seconds,clip_low,clip_high\n";tr.flush();
 std::string reason="ITERATE_CAP_150_NOT_CONVERGED";size_t completed=0;
 for(size_t call=1;call<=150;++call){if(elapsed()>=29.5){reason="WALL_BUDGET_STOP_NOT_CONVERGED";break;}
  auto before=optimizer.values();double e=optimizer.error(),l=optimizer.lambda();int inner=optimizer.getInnerIterations(),acc=optimizer.iterations();
  size_t low=0,high=0;if(params.diagonalDamping){auto h=graph.linearize(before)->hessianDiagonal();for(auto&kv:h)for(int j=0;j<kv.second.size();++j){low+=kv.second[j]<params.minDiagonal;high+=kv.second[j]>params.maxDiagonal;}}
  auto t=std::chrono::steady_clock::now();optimizer.iterate();double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count();++completed;
  int accepted=optimizer.iterations()-acc,rejected=optimizer.getInnerIterations()-inner-accepted;auto step=before.localCoordinates(optimizer.values());auto a=uifgo::AuditNavigationStationarity(graph,optimizer.values(),{},1e-6,8);bool generic=checkConvergence(external,e,optimizer.error());
  tr<<call<<','<<call+50<<','<<e<<','<<optimizer.error()<<','<<l<<','<<optimizer.lambda()<<','<<accepted<<','<<rejected<<','<<optimizer.getInnerIterations()<<','<<optimizer.iterations()<<','<<step.norm()<<','<<maxabs(step)<<',';station(tr,a);tr<<','<<generic<<','<<seconds<<','<<elapsed()<<','<<low<<','<<high<<'\n';tr.flush();
  if(!a.valid||!std::isfinite(optimizer.error())||!std::isfinite(optimizer.lambda())){reason="INVALID_RESULT";break;}
  if(!accepted && optimizer.lambda()>=params.lambdaUpperBound){reason="LAMBDA_SEARCH_EXHAUSTED";break;}
  if(generic&&a.stationary){reason="GENERIC_AND_STATIONARITY_CONVERGED";break;}
 }
 auto result=output(out+"/result.csv");result<<"name,value\nreason,"<<reason<<"\niterate_calls,"<<completed<<"\nerror,"<<optimizer.error()<<"\nlambda,"<<optimizer.lambda()<<"\naccepted,"<<optimizer.iterations()<<"\ninner_trials,"<<optimizer.getInnerIterations()<<"\nwall_seconds,"<<elapsed()<<'\n';
 auto finalid=uifgo::ComputeInferenceContentIdentity(graph,optimizer.values(),context);result<<"final_values_sha,"<<finalid.values_sha256<<"\nfinal_graph_sha,"<<finalid.graph_linearization_sha256<<'\n';
 return 0;
 }catch(const std::exception&e){std::cerr<<"A12_FAILED: "<<e.what()<<'\n';return 1;}}
