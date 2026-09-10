// A15 static recomputation only. No optimizer, solve, or iterate entry point.
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
Values loadValues(const std::string& path,const Values& initial,const std::string& phase) {
 struct Entry {std::string type; std::map<std::string,double> c;};std::map<Key,Entry> parsed;
 for(auto&r:read(path)){if(r.at("phase")!=phase)continue;Key k=std::stoull(r.at("key_value"));auto&e=parsed[k];if(!e.type.empty()&&e.type!=r.at("value_type"))throw std::runtime_error("type collision");e.type=r.at("value_type");if(!e.c.emplace(r.at("coordinate"),std::stod(r.at("value"))).second)throw std::runtime_error("duplicate value coordinate");}
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
VectorValues scale(VectorValues v,double s){for(auto&kv:v)kv.second*=s;return v;}
long double dotld(const Vector&a,const Vector&b){long double v=0;for(int i=0;i<a.size();++i)v+=(long double)a[i]*b[i];return v;}
long double drop(const Vector&a,const Vector&b){long double v=0;for(int i=0;i<a.size();++i)v+=.5L*((long double)a[i]-b[i])*((long double)a[i]+b[i]);return v;}
std::string keys(const NonlinearFactor& f){std::string s;for(auto k:f.keys()){if(!s.empty())s+=';';s+=DefaultKeyFormatter(k)+":"+std::to_string(k);}return s;}
Vector residual(const NonlinearFactor::shared_ptr& f,const Values& v){auto n=boost::dynamic_pointer_cast<NoiseModelFactor>(f);if(!n)throw std::runtime_error("not noise factor");return n->whitenedError(v);}
std::shared_ptr<JacobianFactor> jac(const NonlinearFactor::shared_ptr& f,const Values& v){return std::make_shared<JacobianFactor>(*f->linearize(v));}
Vector product(const JacobianFactor& j,const VectorValues& d){Vector r=Vector::Zero(j.rows());for(auto k:j.keys())r+=j.getA(j.find(k))*d.at(k);return r;}
void station(std::ostream& f,const uifgo::NavigationStationarityAudit&a){f<<a.valid<<','<<a.stationary<<','<<a.max_pose_rotation_gradient_objective_per_rad<<','<<a.max_pose_translation_gradient_objective_per_m<<','<<a.max_velocity_gradient_objective_per_mps<<','<<a.max_accel_bias_gradient_objective_per_mps2<<','<<a.max_gyro_bias_gradient_objective_per_radps<<','<<a.max_scaled_gradient_objective<<','<<a.roundoff_allowance_objective;}
bool near(double a,double b,double at=2e-12,double rt=0){return std::abs(a-b)<=at+rt*std::abs(b);}
int main(int argc,char**argv){try{
 if(argc!=4)throw std::runtime_error("usage CONFIG CAPTURE_DIR OUTPUT_DIR");
 std::string config=argv[1],cap=argv[2],out=argv[3];
 {std::ifstream m("/proc/self/maps");auto f=output(out+"/process_maps.txt");f<<m.rdbuf();}
 auto cfg=uifgo::ConfigLoader::Load(config);
 if(cfg.paper_imu_covariance_model!=uifgo::ImuCovarianceModel::PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1)throw std::runtime_error("wrong model");
 auto cache=uifgo::LoadT07ScenarioCache(cfg.t07_cache_manifest,cfg.t07_cache_start_s,cfg.t07_cache_duration_s);
 auto plan=uifgo::BuildPaperInputPlan(cache.uwb,cfg,cache.base_recording_id);
 auto frames=uifgo::MaterializePaperKeyframes(plan,uifgo::AllPlannedObservationMask(plan));
 auto init=uifgo::Initializer(cfg).Run(cache.imu,frames);if(!init.ok)throw std::runtime_error("raw init failed");
 NonlinearFactorGraph graph;Values initial;std::vector<size_t> uwb;
 uifgo::GraphBuilder(cfg,cfg.paper_imu_covariance_model).Build(frames,cache.imu,init,&graph,&initial,&uwb);
 if(uifgo::ReplacePosePriorsForPaperPath(&graph)!=1)throw std::runtime_error("prior count");
 size_t n=0;for(auto&r:plan.observations)if(r.planned){auto a=std::find_if(cfg.anchors.begin(),cfg.anchors.end(),[&](const auto&a){return a.id==r.anchor_id;});if(a==cfg.anchors.end())throw std::runtime_error("anchor");graph.at(uwb.at(n++))=uifgo::MakeUwbFactor(Symbol('x',r.keyframe_id),0,0,0,a->pos,cfg.lever_arm_init,r.raw_range,r.nominal_sigma,false,false,false,uifgo::FixedBetaForLink(cfg,r.tag_id,r.anchor_id)+0.0);}
 uifgo::InferenceIdentityContext ctx;ctx.input_sha256=cache.cache_id;ctx.config_sha256="sha256:"+uifgo::Sha256FileHex(config);ctx.input_plan_sha256=plan.plan_sha256;ctx.support_partition_sha256="t09-common-no-support:"+uifgo::Sha256Hex(plan.plan_sha256);
 auto id=uifgo::ComputeInferenceContentIdentity(graph,initial,ctx);auto ids=output(out+"/identity.csv");ids<<"phase,graph,values,error\nINITIAL,"<<id.graph_linearization_sha256<<','<<id.values_sha256<<','<<graph.error(initial)<<'\n';ids.flush();
 if(id.graph_linearization_sha256!="t08graphlin-sha256:99cdfbc7032cd3e2f6a2488b12d87b1219d75e60ba0c4f7ca80d56aff3c12710"||id.values_sha256!="t08values-sha256:7115e76d406123d3a30079dc115c6d3501ceca157d270eaced2d8be492b7cd3f"||initial.size()!=123||graph.size()!=371||uwb.size()!=328)throw std::runtime_error("A14_INITIAL_IDENTITY_MISMATCH");
 auto start=loadValues(cap+"/conditional_lm_values.csv",initial,"START");if(uifgo::ComputeInferenceContentIdentity(graph,start,ctx).values_sha256!=id.values_sha256)throw std::runtime_error("capture start mismatch");
 auto terminal=loadValues(cap+"/conditional_lm_values.csv",initial,"FINAL");auto tid=uifgo::ComputeInferenceContentIdentity(graph,terminal,ctx);ids<<"TERMINAL,"<<tid.graph_linearization_sha256<<','<<tid.values_sha256<<','<<graph.error(terminal)<<'\n';
 auto a=uifgo::AuditNavigationStationarity(graph,terminal,{},1e-6,8);auto ast=output(out+"/terminal_stationarity.csv");ast<<"valid,stationary,rotation,translation,velocity,accel_bias,gyro_bias,max_scaled_gradient,roundoff\n";station(ast,a);ast<<'\n';ast.flush();
 double actual[]={a.max_pose_rotation_gradient_objective_per_rad,a.max_pose_translation_gradient_objective_per_m,a.max_velocity_gradient_objective_per_mps,a.max_accel_bias_gradient_objective_per_mps2,a.max_gyro_bias_gradient_objective_per_radps,a.max_scaled_gradient_objective,a.roundoff_allowance_objective};
 double expected[]={4.9631366891844664e-6,1.2054936689764872e-5,4.159606419307238e-7,8.6872436355633909e-8,5.072699877928244e-7,1.2054936689764872e-5,5.0389876118355474e-9};
 if(!a.valid||a.stationary||!near(graph.error(terminal),2325.3170707993281))throw std::runtime_error("A14_TERMINAL_MISMATCH");
 for(int j=0;j<7;++j)if(!near(actual[j],expected[j],2e-12,2e-10))throw std::runtime_error("A14_GRADIENT_MISMATCH");
 auto calls=read(cap+"/conditional_lm_calls.csv");if(calls.size()!=17||calls.back().at("optimizer_iterations_after")!="16"||calls.back().at("inner_iterations_after")!="42"||!near(std::stod(calls.back().at("lambda_after")),100000.00000000007,0))throw std::runtime_error("A14_CALL_COUNT_LAMBDA_MISMATCH");
 // Verify every actual trial has the complete same key/dimension mapping before selecting three.
 std::map<std::pair<int,int>,VectorValues> trials;std::map<std::pair<int,int>,std::set<std::pair<Key,int>>> seen;
 for(auto&r:read(cap+"/conditional_lm_trial_deltas.csv")){if(r.at("parsed")!="1"||r.at("parsed_key_count")!="123"||r.at("parsed_dimension_count")!="615")throw std::runtime_error("trial incomplete");auto t=std::make_pair(std::stoi(r.at("call_index")),std::stoi(r.at("trial_index_within_call")));if(!trials.count(t))trials.emplace(t,initial.zeroVectors());Key k=std::stoull(r.at("key_value"));int j=std::stoi(r.at("coordinate"));if(!seen[t].emplace(k,j).second||j<0||j>=trials.at(t).at(k).size())throw std::runtime_error("trial duplicate/dim");trials.at(t).at(k)[j]=std::stod(r.at("value"));}
 if(trials.size()!=42)throw std::runtime_error("trial count");for(auto&s:seen)if(s.second.size()!=615)throw std::runtime_error("trial missing coord");
 auto match=output(out+"/accepted_retract_checks.csv");match<<"call,trial,max_local_difference,pass\n";
 for(auto&r:calls){int c=std::stoi(r.at("call_index"));int last=0;for(auto&t:trials)if(t.first.first==c)last=std::max(last,t.first.second);if(r.at("accepted_state_update")!="1")continue;auto b=loadValues(cap+"/conditional_lm_direction_base_values.csv",initial,"CALL_"+std::to_string(c)+"_BASE");auto after=c==17?terminal:loadValues(cap+"/conditional_lm_direction_base_values.csv",initial,"CALL_"+std::to_string(c+1)+"_BASE");auto dif=after.localCoordinates(b.retract(trials.at({c,last})));double max=0;for(auto&kv:dif)max=std::max(max,kv.second.cwiseAbs().maxCoeff());match<<c<<','<<last<<','<<max<<','<<(max<=1e-12)<<'\n';if(max>1e-12)throw std::runtime_error("accepted native retract mismatch");}
 auto factid=read(cap+"/conditional_lm_factor_audit.csv"); // all factor errors and types at both ends
 if(factid.size()!=graph.size())throw std::runtime_error("factor count mismatch");
 for(size_t i=0;i<graph.size();++i){auto&r=factid.at(i);if(r.at("keys")!=keys(*graph[i])||r.at("dynamic_type")!=typeid(*graph[i]).name()||!near(std::stod(r.at("error_at_capture_start")),graph[i]->error(initial))||!near(std::stod(r.at("error_at_capture_final")),graph[i]->error(terminal)))throw std::runtime_error("factor evidence mismatch");}
 {auto f=output(out+"/RESTORATION_GATE.txt");f<<"PASS: exact initial hashes, A14 terminal and all factor evidence, 17/16/42, all trial dimensions, all accepted native retracts. Zero optimizer iterate.\n";}
 struct Selection{std::string name;Values base;VectorValues delta;int call;int trial;};std::vector<Selection> selections;
 for(int c:{16,17}){int last=0;for(auto&t:trials)if(t.first.first==c)last=std::max(last,t.first.second);auto b=loadValues(cap+"/conditional_lm_direction_base_values.csv",initial,"CALL_"+std::to_string(c)+"_BASE");if(c==16)selections.push_back({"call16_last_accepted",b,trials.at({c,last}),c,last});else{selections.push_back({"call17_first_rejected",b,trials.at({c,1}),c,1});selections.push_back({"call17_last_rejected",b,trials.at({c,last}),c,last});}}
 auto grad=graph.linearize(terminal)->gradientAtZero();Key maxkey=0;int maxcoord=0;double maxgrad=-1;for(auto&kv:grad)for(int j=0;j<kv.second.size();++j)if(std::abs(kv.second[j])>maxgrad){maxgrad=std::abs(kv.second[j]);maxkey=kv.first;maxcoord=j;}
 auto coord=terminal.zeroVectors();coord.at(maxkey)[maxcoord]=1;selections.push_back({"terminal_max_gradient_coordinate",terminal,coord,0,0});
 {auto f=output(out+"/coordinate_selection.csv");f<<"key,key_value,coordinate,gradient,physical_scale\n"<<DefaultKeyFormatter(maxkey)<<','<<maxkey<<','<<maxcoord<<','<<grad.at(maxkey)[maxcoord]<<",1\n";}
 auto sum=output(out+"/direction_summary.csv");sum<<std::setprecision(21)<<"selection,call,trial,delta_norm,base_error,trial_error,direct_drop,residual_identity_drop_ld,old_linear_error,new_linear_error,linked_linear_drop,linear_identity_drop_ld,gradient_dot_delta_ld,half_Jdelta_squared_ld,resolution_threshold,linked_fidelity_if_resolved,linear_valid,linear_resolved,linked_reconstructed_accept,native_gradient_dot_delta,native_linear_identity_drop\n";
 auto fd=output(out+"/fd_summary.csv");fd<<std::setprecision(21)<<"selection,h,analytic_ld,objective_fd_direct,objective_fd_residual_identity_ld,direct_abs_error,identity_abs_error,objective_tolerance,direct_pass,identity_pass,residual_max_error,max_Ju,residual_tolerance,residual_pass,failed_factors\n";
 auto factors=output(out+"/factor_contributions.csv");factors<<std::setprecision(21)<<"selection,index,type,keys,old_error,new_error,direct_drop,residual_identity_drop_ld,g_delta_ld,half_Jdelta_squared_ld,linear_identity_drop_ld,linear_residual_difference\n";
 auto fdFactors=output(out+"/factor_fd.csv");fdFactors<<std::setprecision(21)<<"selection,h,index,analytic_ld,objective_fd_direct,objective_fd_identity_ld,residual_max_error,max_Ju,tolerance,residual_pass\n";
 const std::vector<double> steps={1e-3,3e-4,1e-4,3e-5,1e-5,3e-6,1e-6,3e-7,1e-7};
 for(auto&s:selections){auto candidate=s.call?s.base.retract(s.delta):s.base;auto linear=graph.linearize(s.base);double e0=graph.error(s.base),e1=graph.error(candidate),l0=linear->error(VectorValues::Zero(s.delta)),l1=linear->error(s.delta),ld=l0-l1;double norm=s.delta.norm();auto u=scale(s.delta,1/norm);
  auto jdump=output(out+"/"+s.name+"_jacobian.csv");jdump<<"factor,row,key,key_value,coordinate,value\n";
  auto rdump=output(out+"/"+s.name+"_residual.csv");rdump<<"phase,h,factor,row,value\n";
  std::vector<Vector> r0,ju,jd;std::vector<std::shared_ptr<JacobianFactor>> js;long double actualDrop=0,gd=0,q=0;
  auto dumpR=[&](std::string phase,double h,size_t i,const Vector&r){for(int j=0;j<r.size();++j)rdump<<phase<<','<<h<<','<<i<<','<<j<<','<<r[j]<<'\n';};
  for(size_t i=0;i<graph.size();++i){auto j=jac(graph[i],s.base);js.push_back(j);auto r=residual(graph[i],s.base),rn=residual(graph[i],candidate);r0.push_back(r);ju.push_back(product(*j,u));jd.push_back(product(*j,s.delta));long double di=drop(r,rn),gi=dotld(r,jd.back()),qi=.5L*dotld(jd.back(),jd.back());actualDrop+=di;gd+=gi;q+=qi;
   factors<<s.name<<','<<i<<','<<typeid(*graph[i]).name()<<','<<keys(*graph[i])<<','<<graph[i]->error(s.base)<<','<<graph[i]->error(candidate)<<','<<graph[i]->error(s.base)-graph[i]->error(candidate)<<','<<di<<','<<gi<<','<<qi<<','<<-gi-qi<<','<<(r+j->getb()).cwiseAbs().maxCoeff()<<'\n';
   dumpR("BASE",0,i,r);dumpR("TRIAL",1,i,rn);
   for(auto k:j->keys()){auto A=j->getA(j->find(k));for(int row=0;row<A.rows();++row)for(int col=0;col<A.cols();++col)jdump<<i<<','<<row<<','<<DefaultKeyFormatter(k)<<','<<k<<','<<col<<','<<A(row,col)<<'\n';}
  }
  double nativegd=0;auto nativegradient=linear->gradientAtZero();for(auto&kv:nativegradient)nativegd+=kv.second.dot(s.delta.at(kv.first));
  double threshold=std::numeric_limits<double>::epsilon()*l0;bool resolved=ld>threshold;double fidelity=resolved?(e0-e1)/ld:0;
  if(s.call) sum<<s.name<<','<<s.call<<','<<s.trial<<','<<norm<<','<<e0<<','<<e1<<','<<e0-e1<<','<<actualDrop<<','<<l0<<','<<l1<<','<<ld<<','<<-gd-q<<','<<gd<<','<<q<<','<<threshold<<','<<fidelity<<','<<(ld>=0)<<','<<resolved<<','<<(ld>=0&&resolved&&fidelity>1e-3)<<','<<nativegd<<','<<-nativegd-(double)q<<'\n';
  long double analytic=0;double maxju=0;for(size_t i=0;i<graph.size();++i){analytic+=dotld(r0[i],ju[i]);maxju=std::max(maxju,ju[i].cwiseAbs().maxCoeff());}
  for(double h:steps){auto plus=s.base.retract(scale(u,h)),minus=s.base.retract(scale(u,-h));double dp=(graph.error(plus)-graph.error(minus))/(2*h),rerr=0;long double df=0;int failed=0;
   for(size_t i=0;i<graph.size();++i){auto rp=residual(graph[i],plus),rm=residual(graph[i],minus);dumpR("PLUS",h,i,rp);dumpR("MINUS",h,i,rm);long double dfi=drop(rp,rm)/(2*(long double)h);df+=dfi;double er=((rp-rm)/(2*h)-ju[i]).cwiseAbs().maxCoeff(),mx=ju[i].cwiseAbs().maxCoeff(),tol=1e-6+1e-5*mx;rerr=std::max(rerr,er);if(er>tol)++failed;
    fdFactors<<s.name<<','<<h<<','<<i<<','<<dotld(r0[i],ju[i])<<','<<(graph[i]->error(plus)-graph[i]->error(minus))/(2*h)<<','<<dfi<<','<<er<<','<<mx<<','<<tol<<','<<(er<=tol)<<'\n';
   }
   long double tolerance=1e-7L+1e-5L*std::abs(analytic);fd<<s.name<<','<<h<<','<<analytic<<','<<dp<<','<<df<<','<<std::abs(dp-analytic)<<','<<std::abs(df-analytic)<<','<<tolerance<<','<<(std::abs(dp-analytic)<=tolerance)<<','<<(std::abs(df-analytic)<=tolerance)<<','<<rerr<<','<<maxju<<','<<1e-6+1e-5*maxju<<','<<(failed==0)<<','<<failed<<'\n';
  }
 }
 std::cout<<"A15_STATIC_COMPLETE no optimizer/no solve; FD pass/fail retained in CSV\n";return 0;
 }catch(const std::exception&e){std::cerr<<"A15_STATIC_FAILED: "<<e.what()<<'\n';return 2;}}
