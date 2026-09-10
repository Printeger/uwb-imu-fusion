#include "a18_optimizer.h"
#include <sys/utsname.h>
using namespace uifgo;
static const std::string policy="PAPER_CERTIFIED_PAIR_REDUCTION_V1";
DiscoveryOptions options(const Config&cfg){
        uifgo::DiscoveryOptions discovery_options;
        discovery_options.fused_lasso.lambda_l1 = cfg.discovery_lambda_l1;
        discovery_options.fused_lasso.lambda_tv = cfg.discovery_lambda_tv;
        discovery_options.fused_lasso.rho_scale = cfg.discovery_rho_scale;
        discovery_options.fused_lasso.primal_absolute_tolerance_m =
            cfg.discovery_primal_abs_tolerance_m;
        discovery_options.fused_lasso.primal_relative_tolerance =
            cfg.discovery_primal_rel_tolerance;
        discovery_options.fused_lasso
            .dual_absolute_tolerance_objective_per_m =
            cfg.discovery_dual_abs_tolerance_objective_per_m;
        discovery_options.fused_lasso.dual_relative_tolerance =
            cfg.discovery_dual_rel_tolerance;
        discovery_options.fused_lasso.kkt_tolerance_objective_per_m =
            cfg.discovery_kkt_tolerance_objective_per_m;
        discovery_options.fused_lasso
            .tv_subgradient_tolerance_objective_per_m =
            cfg.discovery_tv_subgradient_tolerance_objective_per_m;
        discovery_options.fused_lasso.max_iterations =
            static_cast<size_t>(cfg.discovery_admm_max_iterations);
        discovery_options.gap_threshold_s = cfg.discovery_gap_threshold_s;
        discovery_options.active_bias_min_m =
            cfg.discovery_active_bias_min_m;
        discovery_options.change_point_min_m =
            cfg.discovery_change_point_min_m;
        discovery_options.merge_max_difference_m =
            cfg.discovery_merge_max_difference_m;
        discovery_options.short_min_count =
            static_cast<size_t>(cfg.discovery_short_min_count);
        discovery_options.short_min_duration_s =
            cfg.discovery_short_min_duration_s;
        discovery_options.relative_objective_tolerance =
            cfg.refit_relative_objective_tolerance;
        discovery_options.scaled_step_tolerance =
            cfg.discovery_scaled_step_tolerance;
        discovery_options.observation_bias_scale_m =
            cfg.discovery_observation_bias_scale_m;
        discovery_options.navigation_stationarity_tolerance_objective =
            cfg.refit_navigation_stationarity_tolerance_objective;
        discovery_options.gradient_roundoff_safety_factor =
            cfg.refit_gradient_roundoff_safety_factor;
        discovery_options.navigation_scales.pose_rotation_rad =
            cfg.refit_pose_rotation_scale_rad;
        discovery_options.navigation_scales.pose_translation_m =
            cfg.refit_pose_translation_scale_m;
        discovery_options.navigation_scales.velocity_mps =
            cfg.refit_velocity_scale_mps;
        discovery_options.navigation_scales.accel_bias_mps2 =
            cfg.refit_accel_bias_scale_mps2;
        discovery_options.navigation_scales.gyro_bias_radps =
            cfg.refit_gyro_bias_scale_radps;
        discovery_options.max_outer_iterations =
            static_cast<size_t>(cfg.discovery_max_outer_iterations);
        discovery_options.conditional_lm.max_iterations = cfg.lm_max_iter;
        discovery_options.conditional_lm.relative_tolerance = cfg.lm_rel_tol;
        discovery_options.conditional_lm.absolute_tolerance = cfg.lm_abs_tol;
        discovery_options.conditional_lm.policy =
            uifgo::ConditionalLmPolicy::GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
        discovery_options.conditional_lm.navigation_scales =
            discovery_options.navigation_scales;
        discovery_options.conditional_lm
            .navigation_stationarity_tolerance_objective =
            discovery_options.navigation_stationarity_tolerance_objective;
        discovery_options.conditional_lm.gradient_roundoff_safety_factor =
            discovery_options.gradient_roundoff_safety_factor;
return discovery_options;}
std::string strategy(const std::string&config){std::string s=policy+std::string("A18_CPP_MPFR_333_V1")+Sha256FileHex(config)+Sha256FileHex("/proc/self/exe");for(auto*p:{"/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so","/usr/local/lib/libgtsam.so.4.2.0","/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2","/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0"})s+=Sha256FileHex(p);return "a18-policy-sha256:"+Sha256Hex(s);}
std::vector<DevelopmentRangeConstant> ranges(const A17Graph&fx){std::vector<DevelopmentRangeConstant>v;for(auto&kv:fx.ranges){auto&m=kv.second;v.push_back({kv.first,fx.graph[kv.first]->keys()[0],m.at("anchor"),m.at("lever"),m.at("measurement")(0,0),.05,m.at("beta")(0,0),0});}return v;}
Values restore(const a18::Raw&r,const std::string&phase){Values v;for(auto&kv:r){if(std::get<0>(kv.first)!=phase)continue;auto name=std::get<2>(kv.first);size_t index=std::get<1>(kv.first);auto&m=kv.second;if(name=="X"){Matrix4 M;for(int i=0;i<4;++i)for(int j=0;j<4;++j)M(i,j)=m.at(i).at(j);v.insert(Symbol('x',index),Pose3(M));}else if(name=="V"){v.insert(Symbol('v',index),Vector3(m[0][0],m[1][0],m[2][0]));}else if(name=="B"){v.insert(Symbol('b',index),imuBias::ConstantBias(Vector3(m[0][0],m[1][0],m[2][0]),Vector3(m[3][0],m[4][0],m[5][0])));}}return v;}
bool exactMatrix(const a18::DMat&a,const a18::DMat&b){if(a.size()!=b.size())return false;for(size_t i=0;i<a.size();++i){if(a[i].size()!=b[i].size())return false;for(size_t j=0;j<a[i].size();++j)if(std::memcmp(&a[i][j],&b[i][j],8))return false;}return true;}
void staticNative(const std::string&manifest,const A17Graph&fx,const std::string&out){std::ifstream f(manifest);std::string path;size_t count=0;a18::LiveGraph live(fx.graph,ranges(fx));auto report=output(out+"/native_pairs.csv");report<<"pair,coordinates,endpoint_exact,linear_exact,constants_exact\n";
 while(std::getline(f,path)){auto d=a18::load(path);Values base=restore(d.raw,"BASE"),expected=restore(d.raw,"TRIAL");auto delta=base.zeroVectors();size_t n=0;for(auto&kv:delta){auto&m=d.raw.at({"DELTA",Symbol(kv.first).index(),std::string(1,Symbol(kv.first).chr())});if(m.size()!=size_t(kv.second.size()))throw std::runtime_error("STATIC_DELTA_DIM");for(int j=0;j<kv.second.size();++j){kv.second[j]=m[j][0];++n;}}
  Values trial=base.retract(delta);a18::Raw actual;a18::states(actual,"TRIAL",trial);for(auto&kv:actual){auto&m=d.raw.at(kv.first);if(!exactMatrix(kv.second,m))throw std::runtime_error("NATIVE_ENDPOINT_MISMATCH");}
  auto linear=fx.graph.linearize(base);auto liveData=live.pair(base,trial,delta,*linear);for(auto&kv:liveData.raw){auto it=d.raw.find(kv.first);if(it==d.raw.end()||!exactMatrix(it->second,kv.second))throw std::runtime_error("LIVE_CONSTANT_OR_STATE_MISMATCH");}if(liveData.linear.size()!=d.linear.size())throw std::runtime_error("LINEAR_ROW_COUNT");for(size_t i=0;i<d.linear.size();++i){auto&a=liveData.linear[i];auto&b=d.linear[i];if(a.factor!=b.factor||a.row!=b.row||a.r!=b.r||a.ad!=b.ad)throw std::runtime_error("NATIVE_LINEAR_MISMATCH");}
  if(fs::path(path).filename()=="call17_trial1"){LevenbergMarquardtParams p;p.relativeErrorTol=0;p.lambdaInitial=1.0000000000000002e-14;p.setLinearSolverType("SEQUENTIAL_CHOLESKY");auto dir=out+"/native_direction";fs::create_directory(dir);CertifiedLm lm(fx.graph,base,p,ranges(fx),dir);auto solved=lm.fixedDirection(*linear);for(auto&kv:delta)if((kv.second-solved.at(kv.first)).cwiseAbs().maxCoeff()!=0)throw std::runtime_error("NATIVE_DIRECTION_MISMATCH");}
  report<<fs::path(path).filename().string()<<','<<n<<",1,1,1\n";++count;
 }
 if(count!=43)throw std::runtime_error("PAIR_COUNT");std::cout<<"PASS 43 pairs native endpoint/linear/constant identities; call17 trial1 one original solve; zero iterate\n";
}
int main(int argc,char**argv){try{if(argc!=7||std::string(argv[1])!="--policy"||argv[2]!=policy)throw std::runtime_error("A18_DEFAULT_OFF_OR_INVALID_POLICY");std::string mode=argv[3],config=argv[4],input=argv[5],out=fs::absolute(argv[6]);if(mode!="static-native"&&mode!="development-stage1")throw std::runtime_error("A18_SCOPE_REJECTED");if(mode=="development-stage1"&&input!="NO_CHECKPOINT")throw std::runtime_error("WARM_START_FORBIDDEN");if(Sha256FileHex(config)!="479df3daae2096bb665582f60611c66841ed87edc496ddcbaf7cc9e180ed6c4f")throw std::runtime_error("FROZEN_CONFIG_IDENTITY");if(!fs::create_directory(out))throw std::runtime_error("OUTPUT_EXISTS");
 auto sid=strategy(config);{auto f=output(out+"/diagnostic_manifest.json");f<<"{\"schema\":\"A18_STAGE1_DIAGNOSTIC_ONLY\",\"role\":\"development\",\"consumable\":false,\"policy\":\""<<policy<<"\",\"strategy_identity\":\""<<sid<<"\",\"implementation\":\"A18_CPP_MPFR_333_V1\"}\n";}
 bool rejected=false;try{ReadStage2CacheManifest(out+"/diagnostic_manifest.json");}catch(const std::exception&){rejected=true;}if(!rejected)throw std::runtime_error("CACHE_READER_MUST_REJECT");
 A17Graph fx(config);{auto id=ComputeInferenceContentIdentity(fx.graph,fx.initial,fx.ctx);auto f=output(out+"/initial_identity.csv");f<<"graph,values\n"<<id.graph_linearization_sha256<<','<<id.values_sha256<<'\n';}
 if(mode=="static-native"){staticNative(input,fx,out);return 0;}
 auto cache=LoadT07ScenarioCache(fx.cfg.t07_cache_manifest,fx.cfg.t07_cache_start_s,fx.cfg.t07_cache_duration_s);auto plan=BuildPaperInputPlan(cache.uwb,fx.cfg,cache.base_recording_id);std::vector<FactorMeta> metadata;auto range=ranges(fx);size_t j=0;for(auto&r:plan.observations)if(r.planned){FactorMeta m;m.factor_index=range.at(j++).factor_index;m.factor_type="uwb_range";m.obs_id=r.obs_id;metadata.push_back(m);}
 DiscoveryContext ctx;ctx.input_plan_hash=plan.plan_sha256;ctx.source_hash=sid;ctx.config_hash=fx.ctx.config_sha256;ctx.calibration_hash="sha256:"+Sha256Hex(fx.ctx.config_sha256+"fixed-synthetic-assumptions");ctx.solver_config_hash=sid;
 DevelopmentStage1Request req;req.policy=policy;req.role="development";req.implementation_identity=sid;
 req.conditional_navigation=[&](size_t outer,const NonlinearFactorGraph&g,const Values&v,const CheckedLmOptions&o,const std::vector<DevelopmentRangeConstant>&r){return runCertified(outer,g,v,o,r,out+"/outer"+std::to_string(outer));};
 auto trace=output(out+"/outers.csv");trace<<"outer,objective_before,objective_after,relative_change,navigation_step,bias_step,combined_step,KKT,gradient,roundoff,objective_ok,step_ok,KKT_ok,navigation_ok,calls,trials,lambda\n";
 req.outer_observer=[&](const DiscoveryIteration&t){trace<<t.outer_iteration<<','<<t.objective_before<<','<<t.objective_after<<','<<t.relative_objective_change<<','<<t.max_navigation_scaled_step<<','<<t.max_bias_scaled_step<<','<<t.combined_scaled_step<<','<<t.max_chain_kkt_objective_per_m<<','<<t.navigation_gradient_objective<<','<<t.navigation_roundoff_allowance_objective<<','<<t.objective_ok<<','<<t.step_ok<<','<<t.chain_optimality_ok<<','<<t.navigation_stationarity_ok<<','<<t.conditional_lm_convergence.iterate_call_count<<','<<t.conditional_lm_convergence.lambda_trial_count<<','<<t.conditional_lm_lambda<<'\n';trace.flush();};
 auto result=AutomaticSupportProvider(options(fx.cfg)).RunDevelopmentStage1(fx.graph,fx.initial,metadata,plan,fx.cfg,ctx,nullptr,&req);
 auto snap=output(out+"/snapshot.csv");snap<<"obs_id,bias_m,chain_id,active_run_id\n";for(auto&r:result.snapshot)snap<<r.obs_id<<','<<r.bias_m<<','<<r.chain_id<<','<<r.active_run_id<<'\n';
 auto val=output(out+"/last_navigation.csv");val<<"phase,index,name,row,column,hex,bits\n";values(val,"LAST",result.last_conditional_lm.values.empty()?result.navigation_values:result.last_conditional_lm.values);
 auto stat=output(out+"/stage1_status.json");stat<<"{\"schema\":\"A18_STAGE1_DIAGNOSTIC_ONLY\",\"consumable\":false,\"status\":\""<<DiscoveryStatusName(result.status)<<"\",\"reason\":\""<<result.reason<<"\",\"converged\":"<<(result.converged()?"true":"false")<<",\"completed_outers\":"<<result.iterations.size()<<",\"attempted_outer\":"<<result.conditional_lm_attempted_outer_iteration<<",\"segment_count\":"<<(result.converged()?std::to_string(result.partition.segments.size()):"null")<<",\"boundary_count\":null,\"eligible_count\":null,\"Stage2\":\"NOT_RUN\"}\n";
 if(result.converged()){auto f=output(out+"/partition.csv");f<<"segment_id,obs_count,start,end,duration,short_support,merge_mean_m,parent_ids,obs_ids\n";for(auto&s:result.partition.segments){f<<s.segment_id<<','<<s.obs_ids.size()<<','<<s.start_time<<','<<s.end_time<<','<<s.duration<<','<<s.short_support_debug<<','<<s.merge_snapshot_mean_m<<',';for(auto&p:s.parent_segment_ids)f<<p<<';';f<<',';for(auto id:s.obs_ids)f<<id<<';';f<<'\n';}}
 std::cout<<DiscoveryStatusName(result.status)<<":"<<result.reason<<" completed_outers="<<result.iterations.size()<<'\n';return result.converged()?0:1;
 }catch(const std::exception&e){std::cerr<<a18::exceptionStatus(e)<<":"<<e.what()<<'\n';return 2;}}
