// A16 fixed endpoint exporter. Reuses A15 restoration only; no optimizer/solve/iterate.
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <array>
#include <cstring>
#include <set>
#include <gtsam/nonlinear/ExpressionFactor.h>
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
std::string keys(const NonlinearFactor& f){std::string s;for(auto k:f.keys()){if(!s.empty())s+=';';s+=DefaultKeyFormatter(k)+":"+std::to_string(k);}return s;}
void emit(std::ostream& f,const std::string& phase,size_t i,const std::string& name,const Matrix& m){for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c){uint64_t bits;double x=m(r,c);std::memcpy(&bits,&x,8);f<<phase<<','<<i<<','<<name<<','<<r<<','<<c<<','<<std::hexfloat<<x<<std::defaultfloat<<','<<std::hex<<bits<<std::dec<<'\n';}}
void scalar(std::ostream& f,const std::string& phase,size_t i,const std::string& name,double x){Matrix m(1,1);m(0,0)=x;emit(f,phase,i,name,m);}
void values(std::ostream& f,const std::string& phase,const Values& v){for(auto k:v.keys()){char c=Symbol(k).chr();if(c=='x')emit(f,phase,Symbol(k).index(),"X",v.at<Pose3>(k).matrix());else if(c=='v')emit(f,phase,Symbol(k).index(),"V",v.at<Vector3>(k));else if(c=='b')emit(f,phase,Symbol(k).index(),"B",v.at<imuBias::ConstantBias>(k).vector());else throw std::runtime_error("unsupported value");}}
void logbranch(std::ostream& f,const std::string& phase,size_t i,const std::string& name,const Matrix3& R){double tr=R.trace();emit(f,phase,i,name+"_matrix",R);scalar(f,phase,i,name+"_trace",tr);scalar(f,phase,i,name+"_branch",tr+1<1e-3?3:(tr-3 < -1e-6?2:1));}
int main(int argc,char**argv){try{
 if(argc!=5)throw std::runtime_error("usage CONFIG A15_CAPTURE A15_STATIC OUTPUT");std::string config=argv[1],cap=argv[2],ref=argv[3],out=argv[4];
 {auto f=output(out+"/process_maps.txt");std::ifstream m("/proc/self/maps");f<<m.rdbuf();}
 auto cfg=uifgo::ConfigLoader::Load(config);if(cfg.paper_imu_covariance_model!=uifgo::ImuCovarianceModel::PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1)throw std::runtime_error("model mismatch");
 auto cache=uifgo::LoadT07ScenarioCache(cfg.t07_cache_manifest,cfg.t07_cache_start_s,cfg.t07_cache_duration_s);auto plan=uifgo::BuildPaperInputPlan(cache.uwb,cfg,cache.base_recording_id);auto frames=uifgo::MaterializePaperKeyframes(plan,uifgo::AllPlannedObservationMask(plan));auto init=uifgo::Initializer(cfg).Run(cache.imu,frames);if(!init.ok)throw std::runtime_error("restore initializer failed");
 NonlinearFactorGraph graph;Values initial;std::vector<size_t> uwb;uifgo::GraphBuilder(cfg,cfg.paper_imu_covariance_model).Build(frames,cache.imu,init,&graph,&initial,&uwb);if(uifgo::ReplacePosePriorsForPaperPath(&graph)!=1)throw std::runtime_error("prior count");
 std::map<size_t,std::map<std::string,Matrix>> ranges;size_t n=0;for(auto&r:plan.observations)if(r.planned){auto a=std::find_if(cfg.anchors.begin(),cfg.anchors.end(),[&](const auto&a){return a.id==r.anchor_id;});if(a==cfg.anchors.end())throw std::runtime_error("anchor");size_t i=uwb.at(n++);double beta=uifgo::FixedBetaForLink(cfg,r.tag_id,r.anchor_id)+0.0;graph.at(i)=uifgo::MakeUwbFactor(Symbol('x',r.keyframe_id),0,0,0,a->pos,cfg.lever_arm_init,r.raw_range,r.nominal_sigma,false,false,false,beta);ranges[i]["anchor"]=a->pos;ranges[i]["lever"]=cfg.lever_arm_init;ranges[i]["beta"]=Matrix::Constant(1,1,beta);ranges[i]["measurement"]=Matrix::Constant(1,1,r.raw_range);}
 uifgo::InferenceIdentityContext ctx;ctx.input_sha256=cache.cache_id;ctx.config_sha256="sha256:"+uifgo::Sha256FileHex(config);ctx.input_plan_sha256=plan.plan_sha256;ctx.support_partition_sha256="t09-common-no-support:"+uifgo::Sha256Hex(plan.plan_sha256);
 auto id=uifgo::ComputeInferenceContentIdentity(graph,initial,ctx);if(id.graph_linearization_sha256!="t08graphlin-sha256:99cdfbc7032cd3e2f6a2488b12d87b1219d75e60ba0c4f7ca80d56aff3c12710"||id.values_sha256!="t08values-sha256:7115e76d406123d3a30079dc115c6d3501ceca157d270eaced2d8be492b7cd3f"||graph.size()!=371||initial.size()!=123)throw std::runtime_error("INITIAL_IDENTITY_GATE_FAILED");
 auto base=loadValues(cap+"/conditional_lm_direction_base_values.csv",initial,"CALL_17_BASE");auto bid=uifgo::ComputeInferenceContentIdentity(graph,base,ctx);if(bid.graph_linearization_sha256!="t08graphlin-sha256:a97cdd8aca336f329c2930a4d0fa1f88f84c5fba695df7f6da3c2cca9f79598b"||bid.values_sha256!="t08values-sha256:ab7868143714aafa9dad71b30f3329625d759d3f8f105035dd89a44a89fa426c")throw std::runtime_error("BASE_IDENTITY_GATE_FAILED");
 auto factors=read(cap+"/conditional_lm_factor_audit.csv");for(size_t i=0;i<graph.size();++i){auto&r=factors.at(i);if(r.at("keys")!=keys(*graph[i])||r.at("dynamic_type")!=typeid(*graph[i]).name()||std::stod(r.at("error_at_capture_final"))!=graph[i]->error(base))throw std::runtime_error("FACTOR_IDENTITY_GATE_FAILED");}
 auto delta=base.zeroVectors();std::set<std::pair<Key,int>> seen;
 for(auto&r:read(cap+"/conditional_lm_trial_deltas.csv")){if(r.at("call_index")!="17"||r.at("trial_index_within_call")!="1")continue;Key k=std::stoull(r.at("key_value"));int j=std::stoi(r.at("coordinate"));if(r.at("parsed")!="1"||!seen.emplace(k,j).second||j<0||j>=delta.at(k).size())throw std::runtime_error("DELTA_MAPPING_FAILED");delta.at(k)[j]=std::stod(r.at("value"));}
 if(seen.size()!=615)throw std::runtime_error("DELTA_INCOMPLETE");
 // The only endpoint retraction in this tool. This result is fixed thereafter.
 const Values trial=base.retract(delta);auto tid=uifgo::ComputeInferenceContentIdentity(graph,trial,ctx);
 auto identity=output(out+"/identity.csv");identity<<"phase,graph,values,error\nBASE,"<<bid.graph_linearization_sha256<<','<<bid.values_sha256<<','<<graph.error(base)<<"\nTRIAL,"<<tid.graph_linearization_sha256<<','<<tid.values_sha256<<','<<graph.error(trial)<<'\n';
 std::map<std::tuple<std::string,int,int>,double> saved;for(auto&r:read(ref+"/call17_first_rejected_residual.csv"))if(r.at("phase")=="BASE"||r.at("phase")=="TRIAL")saved[{r.at("phase"),std::stoi(r.at("factor")),std::stoi(r.at("row"))}]=std::stod(r.at("value"));
 auto check=output(out+"/restoration_checks.csv");check<<"phase,factor,white_residual_max_difference,pass\n";
 for(const auto&pv:std::vector<std::pair<std::string,Values>>{{"BASE",base},{"TRIAL",trial}})for(size_t i=0;i<graph.size();++i){auto f=boost::dynamic_pointer_cast<NoiseModelFactor>(graph[i]);auto r=f->whitenedError(pv.second);double e=0;for(int j=0;j<r.size();++j)e=std::max(e,std::abs(r[j]-saved.at({pv.first,(int)i,j})));check<<pv.first<<','<<i<<','<<e<<','<<(e==0)<<'\n';if(e!=0)throw std::runtime_error("A15_ENDPOINT_RESIDUAL_MISMATCH");}
 {auto f=output(out+"/RESTORATION_GATE.txt");f<<"PASS exact initial/base graph/Values, 371 type/key/error, 615 delta, 1886 endpoint residual entries. Zero optimizer.\n";}
 auto data=output(out+"/exact_binary64.csv");data<<"phase,index,name,row,column,hex,bits\n";values(data,"BASE",base);values(data,"TRIAL",trial);
 for(auto&kv:delta)emit(data,"DELTA",Symbol(kv.first).index(),std::string(1,Symbol(kv.first).chr()),kv.second);
 scalar(data,"CONST",0,"epsilon",std::numeric_limits<double>::epsilon());scalar(data,"CONST",0,"log_near_zero_threshold",-1e-6);scalar(data,"CONST",0,"log_near_pi_threshold",1e-3);scalar(data,"CONST",0,"pose_log_threshold",1e-10);
 auto meta=output(out+"/factors.csv");meta<<"factor,type,keys,dimension\n";int nim=0;
 for(size_t i=0;i<graph.size();++i){auto nf=boost::dynamic_pointer_cast<NoiseModelFactor>(graph[i]);auto nm=boost::dynamic_pointer_cast<noiseModel::Gaussian>(nf->noiseModel());if(!nm)throw std::runtime_error("not Gaussian");emit(data,"CONST",i,"whitening_R",nm->R());
  std::string type;
  if(auto im=boost::dynamic_pointer_cast<CombinedImuFactor>(graph[i])){type="IMU";++nim;const auto&p=im->preintegratedMeasurements();if(p.p().omegaCoriolis||p.p().body_P_sensor||p.p().use2ndOrderCoriolis)throw std::runtime_error("unsupported actual PIM options");emit(data,"CONST",i,"pim",p.preintegrated());emit(data,"CONST",i,"H_bias_acc",p.preintegrated_H_biasAcc());emit(data,"CONST",i,"H_bias_gyro",p.preintegrated_H_biasOmega());emit(data,"CONST",i,"bias_hat",p.biasHat().vector());emit(data,"CONST",i,"gravity",p.p().n_gravity);scalar(data,"CONST",i,"dt",p.deltaTij());emit(data,"CONST",i,"pim_covariance",p.preintMeasCov());
   for(const auto&pv:std::vector<std::pair<std::string,Values>>{{"BASE",base},{"TRIAL",trial}}){auto&v=pv.second;auto k=im->keys();NavState si(v.at<Pose3>(k[0]),v.at<Vector3>(k[1]));NavState sj(v.at<Pose3>(k[2]),v.at<Vector3>(k[3]));auto bc=p.biasCorrectedDelta(v.at<imuBias::ConstantBias>(k[4]));auto xi=si.correctPIM(bc,p.deltaTij(),p.p().n_gravity,p.p().omegaCoriolis,p.p().use2ndOrderCoriolis);auto pred=p.predict(si,v.at<imuBias::ConstantBias>(k[4]));emit(data,pv.first,i,"native_bias_corrected",bc);emit(data,pv.first,i,"native_xi",xi);emit(data,pv.first,i,"native_pred_R",pred.R().matrix());emit(data,pv.first,i,"native_pred_p",pred.position());emit(data,pv.first,i,"native_pred_v",pred.velocity());logbranch(data,pv.first,i,"imu_log",sj.R().transpose()*pred.R());scalar(data,pv.first,i,"exp_near_zero",bc.head<3>().squaredNorm()<=std::numeric_limits<double>::epsilon());}
  }else if(auto pr=boost::dynamic_pointer_cast<uifgo::PaperPosePriorFactor>(graph[i])){type="POSE_PRIOR";emit(data,"CONST",i,"prior",pr->prior().matrix());for(const auto&pv:std::vector<std::pair<std::string,Values>>{{"BASE",base},{"TRIAL",trial}}){Pose3 relative=pv.second.at<Pose3>(pr->key()).between(pr->prior());logbranch(data,pv.first,i,"pose_log",relative.rotation().matrix());emit(data,pv.first,i,"native_relative_T",relative.translation());scalar(data,pv.first,i,"pose_log_small",Rot3::Logmap(relative.rotation()).norm()<1e-10);}}
  else if(auto pr=boost::dynamic_pointer_cast<PriorFactor<Vector3>>(graph[i])){type="V_PRIOR";emit(data,"CONST",i,"prior",pr->prior());}
  else if(auto pr=boost::dynamic_pointer_cast<PriorFactor<imuBias::ConstantBias>>(graph[i])){type="B_PRIOR";emit(data,"CONST",i,"prior",pr->prior().vector());}
  else if(ranges.count(i)){type="RANGE";auto ex=boost::dynamic_pointer_cast<ExpressionFactor<double>>(graph[i]);if(!ex||ex->measured()!=ranges.at(i).at("measurement")(0,0))throw std::runtime_error("expression identity");for(auto&m:ranges.at(i))emit(data,"CONST",i,m.first,m.second);}
  else throw std::runtime_error("unsupported factor");
  meta<<i<<','<<type<<','<<keys(*graph[i])<<','<<nf->dim()<<'\n';
  for(const auto&pv:std::vector<std::pair<std::string,Values>>{{"BASE",base},{"TRIAL",trial}}){emit(data,pv.first,i,"native_unwhite",nf->unwhitenedError(pv.second));emit(data,pv.first,i,"native_white",nf->whitenedError(pv.second));scalar(data,pv.first,i,"native_factor_error",nf->error(pv.second));}
 }
 scalar(data,"BASE",0,"native_graph_error",graph.error(base));scalar(data,"TRIAL",0,"native_graph_error",graph.error(trial));
 if(nim!=40)throw std::runtime_error("IMU count");std::cout<<"A16_ENDPOINT_EXPORT_PASS zero optimizer; fixed native endpoints; exact hex/bits constants\n";return 0;
 }catch(const std::exception&e){std::cerr<<"A16_EXPORT_FAILED: "<<e.what()<<'\n';return 2;}}
