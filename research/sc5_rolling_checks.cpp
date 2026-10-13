#include "rolling_policy.hpp"
#include <iostream>
using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
template<class F> void refuses(F f){bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}require(rejected,"expected refusal");}
int rollingBoundaryChecks();
int main(int argc,char** argv){try{
 if(argc>1 && std::string(argv[1])=="boundary")return rollingBoundaryChecks();
 auto root=researchLaw("config/sc5_rolling.yaml");
 auto base=IntegrityConfigLoader::load(root["imu_base_config"].as<std::string>());
 for(auto pair:std::vector<std::pair<int,int>>{{30,30},{20,20},{40,40},{30,10}}){
  auto n=YAML::Clone(root);n["fixed_lag_epochs"]=pair.first;
  if(pair.first!=pair.second){n["rolling_integrity"]["window_mode"]="explicit";n["rolling_integrity"]["explicit_epochs"]=pair.second;}
  auto cfg=base;auto p=resolveRollingPolicy(n,cfg);
  require(p.fgo==pair.first && p.integrity==pair.second && p.mission==200,"resolution");
  IncrementalUwbImuEstimator observed(cfg,Eigen::Vector3d::Zero()),control(cfg,Eigen::Vector3d::Zero());
  for(auto* e:{&observed,&control}){e->initialize(initialState(),cfg.realtime.prior_sigmas);e->ingestImu(sample(cfg,0));}
  size_t marginal=0,exit=0;
  for(int k=1;k<=70;++k){
   auto tx=prepare(observed,cfg,k);auto plan=EpochCommitPlan::nominalPlan(tx);
   auto proposal=captureBeforeUpdate(tx,plan,p);
   auto r=observed.commitEpoch(std::move(tx),plan);
   auto receipt=finalizeTransfer(proposal,r,observed.retainedKeysV1());
   auto c=prepare(control,cfg,k);auto cp=EpochCommitPlan::nominalPlan(c);control.commitEpoch(std::move(c),cp);
   auto a=observed.currentState(),b=control.currentState();
   require(a.position_world_m==b.position_world_m && a.velocity_world_mps==b.velocity_world_mps && a.q_world_body.coeffs()==b.q_world_body.coeffs() && a.accel_bias_mps2==b.accel_bias_mps2 && a.gyro_bias_radps==b.gyro_bias_radps,"observer changed native state");
   require(receipt.retained==std::min(k+1,p.fgo),"actual window");
   marginal+=receipt.marginalized;if(!receipt.x0_present && !exit)exit=k;
   auto bad=r;--bad.graph_version;refuses([&]{finalizeTransfer(proposal,bad,observed.retainedKeysV1());});
  }
  require(exit==size_t(p.fgo) && marginal==size_t(3*(71-p.fgo)),"actual x0 exit");
  std::cout<<"W1 "<<p.fgo<<'/'<<p.integrity<<" commits70+70 x0_exit="<<exit<<" actual_marginal_keys="<<marginal<<'\n';
 }
 for(auto lag:{-1LL,0LL,1LL,2147483648LL}){auto n=YAML::Clone(root);n["fixed_lag_epochs"]=lag;auto cfg=base;refuses([&]{resolveRollingPolicy(n,cfg);});}
 auto cfg=base;auto n=YAML::Clone(root);n["rolling_integrity"]["explicit_epochs"]=10;refuses([&]{resolveRollingPolicy(n,cfg);});
 n=YAML::Clone(root);n["rolling_integrity"]["window_mode"]="explicit";n["rolling_integrity"]["explicit_epochs"]=31;refuses([&]{resolveRollingPolicy(n,cfg);});
 refuses([&]{rollingResize({},20,20);});
 std::cout<<"SC5 W1 lifecycle/config PASS; native560 commits; old defaults untouched\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
