// T10-A17 opt-in first-block prototype. Native GTSAM direction/retract/state.
#include "a17_graph_io.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtState.h>
#include <gtsam/linear/linearExceptions.h>
#include <filesystem>
#include <functional>
#include <sys/wait.h>
#include <unistd.h>
#include <csignal>
namespace fs=std::filesystem;
static const std::string policy="PAPER_CERTIFIED_PAIR_REDUCTION_V1";
struct Reply {std::string status;double fidelity=0;};
class Worker {
 FILE* in_=nullptr;FILE* out_=nullptr;pid_t pid_=-1;
 public:
 explicit Worker(const std::string& script){int a[2],b[2];if(pipe(a)||pipe(b))throw std::runtime_error("CERTIFICATE_PIPE");pid_=fork();if(pid_<0)throw std::runtime_error("CERTIFICATE_FORK");if(pid_==0){dup2(a[0],0);dup2(b[1],1);close(a[0]);close(a[1]);close(b[0]);close(b[1]);execlp("python3","python3",script.c_str(),"--serve",(char*)nullptr);_exit(127);}close(a[0]);close(b[1]);in_=fdopen(a[1],"w");out_=fdopen(b[0],"r");}
 ~Worker(){if(in_){fputs("QUIT\n",in_);fflush(in_);fclose(in_);}if(out_)fclose(out_);if(pid_>0){int c;waitpid(pid_,&c,0);}}
 Reply ask(const std::string& dir){if(fprintf(in_,"%s\n",dir.c_str())<0||fflush(in_))throw std::runtime_error("CERTIFICATE_IPC_WRITE");char line[512];if(!fgets(line,sizeof(line),out_))throw std::runtime_error("CERTIFICATE_IPC_EOF");std::istringstream s(line);Reply r;std::string hex,extra;s>>r.status>>hex;if(s>>extra||hex.empty())throw std::runtime_error("CERTIFICATE_IPC_FORMAT");char* end=nullptr;r.fidelity=strtod(hex.c_str(),&end);if(!end||*end||!std::isfinite(r.fidelity))throw std::runtime_error("NUMERIC_REFERENCE_NONFINITE:FIDELITY");return r;}
};
using PairWriter=std::function<void(const std::string&,const Values&,const Values&,const VectorValues&)>;
class CertifiedLm:public LevenbergMarquardtOptimizer {
 using State=gtsam::internal::LevenbergMarquardtState;
 Worker& worker_;PairWriter writer_;std::string out_;std::ofstream trials_;
 public:
 size_t calls=0,trials=0,solves=0,accepted=0,rejected=0,unresolved=0;
 std::string failure;
 CertifiedLm(const NonlinearFactorGraph&g,const Values&v,const LevenbergMarquardtParams&p,Worker&w,PairWriter writer,const std::string&out):LevenbergMarquardtOptimizer(g,v,p),worker_(w),writer_(std::move(writer)),out_(out),trials_(output(out+"/trials.csv")){
  if(p.diagonalDamping||!p.useFixedLambdaFactor||p.lambdaFactor!=10||p.lambdaUpperBound!=1e5||p.lambdaLowerBound!=0||p.minModelFidelity!=1e-3||p.relativeErrorTol!=0||p.getLinearSolverType()!="SEQUENTIAL_CHOLESKY")throw std::runtime_error("A17_UNSUPPORTED_LM_PARAMETERS");
  trials_<<"call,trial_in_call,total_trial,lambda_before,lambda_after,delta_norm,error_before,error_native_trial,status,fidelity_lower_binary64,accepted_total,rejected_total,unresolved_total,elapsed_s,artifact\n";
 }
 VectorValues fixedDirection(const GaussianFactorGraph&linear){++solves;return solve(buildDampedSystem(linear,VectorValues()),params_);}
 GaussianFactorGraph::shared_ptr iterate()override{
  ++calls;auto linear=linearize();size_t local=0;
  while(true){++local;++trials;auto*s=static_cast<State*>(state_.get());double lb=s->lambda,eb=error();auto begin=std::chrono::steady_clock::now();std::string dir=out_+"/call"+std::to_string(calls)+"_trial"+std::to_string(local);fs::create_directory(dir);Reply reply;double dn=std::numeric_limits<double>::quiet_NaN(),et=dn;
   try{
    auto delta=fixedDirection(*linear);dn=delta.norm();const Values base=values();Values trial=base.retract(delta);et=graph_.error(trial);
    if(!std::isfinite(dn)||!std::isfinite(et))throw std::runtime_error("NUMERIC_REFERENCE_NONFINITE:NATIVE_TRIAL");
    writer_(dir,base,trial,delta);writeLinear(dir,*linear,delta);{std::ofstream f(dir+"/exact_binary64.csv",std::ios::app);scalar(f,"CONST",0,"min_model_fidelity",params_.minModelFidelity);}
    reply=worker_.ask(dir);
    if(reply.status=="ACCEPT"){
     // Worker certifies RNDD conversion too. Do not recalculate the quotient.
     if(!(reply.fidelity>params_.minModelFidelity))throw std::runtime_error("CERTIFICATE_FIDELITY_CONVERSION_INVALID");
     state_=s->decreaseLambda(params_,reply.fidelity,std::move(trial),et);++accepted;
    }else if(reply.status=="REJECT"){s->increaseLambda(params_);++rejected;}
    else{++unresolved;failure=reply.status;}
   }catch(const IndeterminantLinearSystemException&){reply.status="LINEAR_SOLVE_FAILED";s->increaseLambda(params_);++rejected;}
   catch(const std::exception&e){reply.status=std::string(e.what()).find("NUMERIC_REFERENCE_NONFINITE")!=std::string::npos?"NUMERIC_REFERENCE_NONFINITE":"NUMERIC_REFERENCE_UNSUPPORTED";failure=std::string(e.what());++unresolved;}
   double secs=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
   trials_<<calls<<','<<local<<','<<trials<<','<<lb<<','<<lambda()<<','<<dn<<','<<eb<<','<<et<<','<<reply.status<<','<<reply.fidelity<<','<<accepted<<','<<rejected<<','<<unresolved<<','<<secs<<','<<fs::path(dir).filename().string()<<'\n';trials_.flush();
   if(!failure.empty())return linear;
   if(reply.status=="ACCEPT")return linear;
   if(lambda()>=params_.lambdaUpperBound){failure="CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED";return linear;}
  }
 }
};
LevenbergMarquardtParams parameters(const uifgo::Config&cfg){LevenbergMarquardtParams p;p.setMaxIterations(cfg.lm_max_iter);p.setRelativeErrorTol(0);p.setAbsoluteErrorTol(cfg.lm_abs_tol);p.setLinearSolverType("SEQUENTIAL_CHOLESKY");return p;}
void maps(const std::string&out){auto f=output(out+"/process_maps.txt");std::ifstream s("/proc/self/maps");f<<s.rdbuf();}
void identity(const std::string&out,const A17Graph&g,const Values&v,const std::string&label){auto i=uifgo::ComputeInferenceContentIdentity(g.graph,v,g.ctx);auto f=output(out+"/"+label+"_identity.csv");f<<"graph,values,error\n"<<i.graph_linearization_sha256<<','<<i.values_sha256<<','<<g.graph.error(v)<<'\n';}
void manifest(const std::string&out,const std::string&config,const std::string&worker){std::string provenance=policy+uifgo::Sha256FileHex(config)+uifgo::Sha256FileHex("/proc/self/exe");for(const auto&path:std::vector<std::string>{worker,(fs::path(worker).parent_path()/"a16_mpfr.py").string(),(fs::path(worker).parent_path()/"a16_reference.py").string(),"/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so","/usr/local/lib/libgtsam.so.4.2.0","/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2","/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0"})provenance+=uifgo::Sha256FileHex(path);auto f=output(out+"/diagnostic_manifest.json");f<<"{\"schema\":\"A17_FIRST_BLOCK_DIAGNOSTIC_ONLY\",\"policy\":\""<<policy<<"\",\"role\":\"development\",\"consumable\":false,\"config_sha256\":\""<<uifgo::Sha256FileHex(config)<<"\",\"strategy_identity\":\"a17-policy-sha256:"<<uifgo::Sha256Hex(provenance)<<"\",\"precision_bits\":333,\"scope\":\"OUTER1_FIRST_CONDITIONAL_ONLY\"}\n";}
void rejectCache(const std::string&path){bool rejected=false;try{uifgo::ReadStage2CacheManifest(path);}catch(const std::exception&){rejected=true;}if(!rejected)throw std::runtime_error("DIAGNOSTIC_CACHE_CONSUMPTION_NOT_REJECTED");}
int main(int argc,char**argv){try{
 std::signal(SIGPIPE,SIG_IGN);
 if(argc!=8||std::string(argv[1])!="--policy"||argv[2]!=policy)throw std::runtime_error("A17_DEFAULT_OFF_OR_INVALID_POLICY: explicit policy MODE CONFIG EVIDENCE OUTPUT WORKER required");
 std::string mode=argv[3],config=argv[4],evidence=argv[5],out=fs::absolute(argv[6]),worker=fs::absolute(argv[7]);
 if(mode!="regression"&&mode!="development-first-block"&&mode!="fixture")throw std::runtime_error("A17_SCOPE_REJECTED");
 if(!fs::create_directory(out))throw std::runtime_error("A17_OUTPUT_MUST_BE_NEW");maps(out);manifest(out,config,worker);rejectCache(out+"/diagnostic_manifest.json");
 if(uifgo::Sha256FileHex(config)!="479df3daae2096bb665582f60611c66841ed87edc496ddcbaf7cc9e180ed6c4f")throw std::runtime_error("A17_FROZEN_CONFIG_IDENTITY");
 A17Graph fx(config);auto p=parameters(fx.cfg);{auto par=output(out+"/lm_parameters.txt");auto* old=std::cout.rdbuf(par.rdbuf());p.print();std::cout.rdbuf(old);par<<"external_relative="<<fx.cfg.lm_rel_tol<<" external_absolute="<<fx.cfg.lm_abs_tol<<" stationarity="<<fx.cfg.refit_navigation_stationarity_tolerance_objective<<" roundoff="<<fx.cfg.refit_gradient_roundoff_safety_factor<<"\n";}identity(out,fx,fx.initial,"initial");
 auto pair=[&](const std::string&o,const Values&a,const Values&b,const VectorValues&d){fx.pair(o,a,b,d);};
 if(mode=="regression"){
  auto base=loadValues(evidence+"/inputs/A15_selected/conditional_lm_direction_base_values.csv",fx.initial,"CALL_17_BASE");identity(out,fx,base,"base");auto bid=uifgo::ComputeInferenceContentIdentity(fx.graph,base,fx.ctx);if(bid.values_sha256!="t08values-sha256:ab7868143714aafa9dad71b30f3329625d759d3f8f105035dd89a44a89fa426c"||bid.graph_linearization_sha256!="t08graphlin-sha256:a97cdd8aca336f329c2930a4d0fa1f88f84c5fba695df7f6da3c2cca9f79598b")throw std::runtime_error("BASE_IDENTITY_GATE");p.lambdaInitial=1.0000000000000002e-14;
  Worker w(worker);CertifiedLm lm(fx.graph,base,p,w,pair,out);auto linear=lm.linearize();auto delta=lm.fixedDirection(*linear);auto expected=base.zeroVectors();std::set<std::pair<Key,int>> seen;
  for(auto&r:read(evidence+"/inputs/A15_selected/conditional_lm_trial_deltas.csv")){Key k=std::stoull(r.at("key_value"));int j=std::stoi(r.at("coordinate"));if(!seen.emplace(k,j).second)throw std::runtime_error("DUPLICATE_DELTA");expected.at(k)[j]=std::stod(r.at("value"));}
  if(seen.size()!=615)throw std::runtime_error("DELTA_DIMENSION");for(auto&kv:delta)for(int j=0;j<kv.second.size();++j)if(kv.second[j]!=expected.at(kv.first)[j])throw std::runtime_error("NATIVE_DIRECTION_NOT_BIT_IDENTICAL");
  auto trial=base.retract(delta);auto id=uifgo::ComputeInferenceContentIdentity(fx.graph,trial,fx.ctx);if(id.values_sha256!="t08values-sha256:7f1dc3973b6711dce516f0df6522d4c24a361bbfbbf8a41e3d60cd8f55977135"||id.graph_linearization_sha256!="t08graphlin-sha256:0b9312f8c5f8c0f9c6ef36c658d62878d8743140b16468069e3d8f17057f6d38")throw std::runtime_error("NATIVE_ENDPOINT_NOT_IDENTICAL");
  identity(out,fx,trial,"trial");pair(out,base,trial,delta);writeLinear(out,*linear,delta);{std::ofstream f(out+"/exact_binary64.csv",std::ios::app);scalar(f,"CONST",0,"min_model_fidelity",p.minModelFidelity);}
  auto r=w.ask(out);if(r.status!="ACCEPT")throw std::runtime_error("FIXED_PAIR_CERTIFICATE_"+r.status);auto f=output(out+"/REGRESSION_GATE.txt");f<<"PASS 615 exact direction coordinates; A16 endpoint graph/Values exact; original one GTSAM solve; zero iterate; cache consumer rejects diagnostic schema\n";std::cout<<"STATIC_NATIVE_DIRECTION_PASS\n";return 0;
 }
 if(evidence!="NO_CHECKPOINT")throw std::runtime_error("A17_PILOT_WARM_START_FORBIDDEN");
 Worker w(worker);
 // Fixture uses one existing vector-prior factor definition, no scenario search.
 if(mode=="fixture"){
  NonlinearFactorGraph g;Values v;v.insert(Symbol('v',0),Vector3(1,0,0));g.add(PriorFactor<Vector3>(Symbol('v',0),Vector3::Zero(),noiseModel::Unit::Create(3)));
  auto writer=[](const std::string&o,const Values&a,const Values&b,const VectorValues&d){auto f=output(o+"/exact_binary64.csv");f<<"phase,index,name,row,column,hex,bits\n";values(f,"BASE",a);values(f,"TRIAL",b);emit(f,"CONST",0,"prior",Vector3::Zero());emit(f,"CONST",0,"whitening_R",Matrix3::Identity());auto m=output(o+"/factors.csv");m<<"factor,type,keys,dimension\n0,V_PRIOR,v0:8502796096475496448,3\n";};
  CertifiedLm lm(g,v,p,w,writer,out);double prev=lm.error();bool ok=false;size_t checked=0;for(int i=0;i<50;++i){lm.iterate();if(!lm.failure.empty())throw std::runtime_error(lm.failure);++checked;bool gen=checkConvergence(fx.cfg.lm_rel_tol,fx.cfg.lm_abs_tol,p.errorTol,prev,lm.error());auto audit=uifgo::AuditNavigationStationarity(g,lm.values(),{},1e-6,8);if(gen&&audit.stationary){ok=true;break;}prev=lm.error();}
  if(!ok||lm.calls!=2||lm.accepted!=2||lm.trials!=2||lm.solves!=2||lm.rejected||lm.unresolved||lm.iterations()!=2||lm.getInnerIterations()!=2||checked!=2)throw std::runtime_error("FIXTURE_COUNTER_OR_STATIONARITY");auto f=output(out+"/FIXTURE_GATE.txt");f<<"PASS original generic AND stationarity; 2 calls/2 solves/2 trials/2 accepted/0 rejected/0 unresolved; native state counters match\n";return 0;
 }
 CertifiedLm lm(fx.graph,fx.initial,p,w,pair,out);auto calls=output(out+"/calls.csv");calls<<"call,error,lambda,trials,accepted,rejected,unresolved,generic,stationarity_valid,stationary,max_scaled_gradient,roundoff,rot_gradient,pos_gradient,vel_gradient,acc_bias_gradient,gyro_bias_gradient,call_s,reason\n";
 double prev=lm.error();bool converged=false;std::string reason="CONDITIONAL_LM_STATIONARITY_NOT_REACHED";uifgo::NavigationStationarityAudit audit;size_t generic_count=0;
 for(int c=0;c<fx.cfg.lm_max_iter;++c){auto t=std::chrono::steady_clock::now();lm.iterate();bool generic=false;
  audit=uifgo::AuditNavigationStationarity(fx.graph,lm.values(),{},fx.cfg.refit_navigation_stationarity_tolerance_objective,fx.cfg.refit_gradient_roundoff_safety_factor);
  if(lm.failure.empty()){generic=checkConvergence(fx.cfg.lm_rel_tol,fx.cfg.lm_abs_tol,p.errorTol,prev,lm.error());generic_count+=generic;if(!audit.valid)reason="CONDITIONAL_LM_STATIONARITY_AUDIT_INVALID";else if(generic&&audit.stationary){converged=true;reason="CONDITIONAL_LM_CONVERGED_AND_NAVIGATION_STATIONARY";}}
  else reason=lm.failure;
  calls<<lm.calls<<','<<lm.error()<<','<<lm.lambda()<<','<<lm.trials<<','<<lm.accepted<<','<<lm.rejected<<','<<lm.unresolved<<','<<generic<<','<<audit.valid<<','<<audit.stationary<<','<<audit.max_scaled_gradient_objective<<','<<audit.roundoff_allowance_objective<<','<<audit.max_pose_rotation_gradient_objective_per_rad<<','<<audit.max_pose_translation_gradient_objective_per_m<<','<<audit.max_velocity_gradient_objective_per_mps<<','<<audit.max_accel_bias_gradient_objective_per_mps2<<','<<audit.max_gyro_bias_gradient_objective_per_radps<<','<<std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count()<<','<<reason<<'\n';calls.flush();
  {auto v=output(out+"/last_complete_values.csv");v<<"phase,index,name,row,column,hex,bits\n";values(v,"LAST_COMPLETE",lm.values());}
  if(converged||!lm.failure.empty()||!audit.valid)break;prev=lm.error();
 }
 identity(out,fx,lm.values(),"final");auto f=output(out+"/first_block_status.json");f<<"{\"schema\":\"A17_FIRST_BLOCK_DIAGNOSTIC_ONLY\",\"policy\":\""<<policy<<"\",\"consumable\":false,\"first_block_converged\":"<<(converged?"true":"false")<<",\"reason\":\""<<reason<<"\",\"calls\":"<<lm.calls<<",\"trials\":"<<lm.trials<<",\"solves\":"<<lm.solves<<",\"accepted\":"<<lm.accepted<<",\"rejected\":"<<lm.rejected<<",\"unresolved\":"<<lm.unresolved<<",\"native_iterations\":"<<lm.iterations()<<",\"native_inner_iterations\":"<<lm.getInnerIterations()<<",\"generic_count\":"<<generic_count<<",\"lambda\":"<<lm.lambda()<<",\"error\":"<<lm.error()<<",\"max_scaled_gradient\":"<<audit.max_scaled_gradient_objective<<",\"roundoff\":"<<audit.roundoff_allowance_objective<<",\"Stage1\":\"NOT_RUN_BEYOND_FIRST_BLOCK\",\"chain\":\"NOT_RUN\",\"Stage2\":\"NOT_RUN\",\"candidate_count\":null,\"eligible_count\":null}\n";
 std::cout<<reason<<" calls="<<lm.calls<<" trials="<<lm.trials<<" accepted="<<lm.accepted<<" rejected="<<lm.rejected<<" unresolved="<<lm.unresolved<<" gradient="<<audit.max_scaled_gradient_objective<<'\n';return converged?0:1;
 }catch(const std::exception&e){std::cerr<<"A17_FAILED: "<<e.what()<<'\n';return 2;}}
