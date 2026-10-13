#include "sc4_system.hpp"
#include <iostream>
#include <map>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void check(bool condition,const char* reason){if(!condition)throw std::runtime_error(reason);}
int main(int argc,char** argv){try{
 if(argc==2 && std::string(argv[1])=="local"){
  for(const auto& recipe:std::vector<std::pair<std::string,int>>{{"SC3_U0",2},{"SC3_IG",17}}){auto input=sc4MatchedInput("config/sc4_performance.yaml",recipe.first,recipe.second);Sc4System reference(input),optimized(input,true);auto origin=Sc4Clock::now();
   for(int i=0;i<recipe.second;++i){auto a=reference.process(i,true,Sc4Clock::now(),origin);auto b=optimized.process(i,true,Sc4Clock::now(),origin);
    check(a.tag==b.tag && a.state.velocity_world_mps==b.state.velocity_world_mps && a.state.q_world_body.coeffs()==b.state.q_world_body.coeffs() && a.state.accel_bias_mps2==b.state.accel_bias_mps2 && a.state.gyro_bias_radps==b.state.gyro_bias_radps,"profile exact full native output state");
    check(a.retained_ids==b.retained_ids && a.removed_groups==b.removed_groups && a.input_ids==b.input_ids && a.action.excluded==b.action.excluded && a.action.imu.current_bridge==b.action.imu.current_bridge && a.action.expected_actions==b.action.expected_actions && a.action.evaluated_actions==b.action.evaluated_actions && a.version==b.version && a.transaction==b.transaction,"complete source/action/transaction equivalence");
    check(a.proof.valid && b.proof.valid && a.proof.geometry.low==b.proof.geometry.low && a.proof.geometry.high==b.proof.geometry.high && a.proof.geometry.pl==b.proof.geometry.pl && a.proof.geometry.hpl==b.proof.geometry.hpl && a.cumulative_risk==b.cumulative_risk,"same conditional physical coverage/source/risk");
    if(i==0 || i>=15)std::cout<<"SC4 local "<<recipe.first<<" epoch="<<i+1<<" roots="<<a.action.evaluated_actions<<" GN="<<a.action.total_rebuilds<<" optGN="<<b.action.total_rebuilds<<" optNumeric="<<b.action.numeric_evaluated<<" postRejected="<<b.action.numeric_skipped_post_rejected<<" dominated="<<b.action.numeric_skipped_dominated<<" rawPIM="<<a.action.total_reintegrations<<" diagnostic="<<a.action.diagnostic_calls<<" ref/opt core="<<a.timing.core_ms<<'/'<<b.timing.core_ms<<" GNms="<<a.action.optimizer_ms<<" diagnosticms="<<a.action.diagnostics_ms<<" rawms="<<a.action.reintegrations_ms<<"\n";
   }
  }std::cout<<"SC4 local two profiles full-state/source/action/receipt/PL exact equivalence PASS\n";return 0;
 }
 if(argc==2 && std::string(argv[1])=="carry"){
  auto input=sc4MatchedInput("config/sc4_carry.yaml","SC3_G",60);auto old=sc4MatchedInput("config/sc4_carry.yaml","SC3_G",50);
  for(size_t i=0;i<old.imu.size();++i)check(input.imu[i].specific_force_mps2==old.imu[i].specific_force_mps2 && input.imu[i].angular_velocity_radps==old.imu[i].angular_velocity_radps,"same original source prefix in long input");
  Sc4System longSystem(std::move(input));auto origin=Sc4Clock::now();uint64_t last=1;std::map<std::string,int> statuses;
  for(size_t i=0;i<60;++i){auto out=longSystem.process(i,true,Sc4Clock::now(),origin);check(out.updates==1 && out.version==last+1,"continuous one backend/version");last=out.version;++statuses[out.bound_status];
   check(out.proof.valid==(i<50) && out.cumulative_risk==std::ldexp(2.*6421,-35),"expired proof never renews or clears risk");
   if(i>=47)check(out.carry,"old physical fault remains in bank across horizon");
   if(i>=50 && i<58)check(out.action.imu.current_bridge && out.physical_evaluated==0 && !out.protected_output,"unprotected carry bridge keeps contaminated raw support out");
   if(i==49 || i==50 || i==57 || i==59)std::cout<<"SC4 carry "<<i+1<<" version="<<out.version<<" qualified="<<out.proof.valid<<" bridge="<<out.action.imu.current_bridge<<" carry="<<out.carry<<" risk="<<out.cumulative_risk<<"\n";
  }
  for(const auto& status:statuses)std::cout<<"SC4 carry bound_count "<<status.first<<'='<<status.second<<"\n";
  std::cout<<"SC4 continuous60 late joint episode48, original prefix, horizon/carry/risk PASS\n";return 0;
 }
 auto input=sc4MatchedInput("config/sc4_system.yaml","SC3_U0",2);check(sc4InputError(input).empty(),"matched admission");
 auto bad=input;bad.imu[3].timestamp=bad.imu[2].timestamp;check(!sc4InputError(bad).empty(),"nonmonotone IMU");
 bad=input;bad.imu.erase(bad.imu.begin()+5);check(sc4InputError(bad)=="MATCHED_SAMPLE_GAP","missing shared sample");
 bad=input;bad.batches[1].measurements.clear();check(!sc4InputError(bad).empty(),"missing UWB");
 bad=input;bad.batches[1].measurements[0].id=bad.batches[0].measurements[0].id;check(!sc4InputError(bad).empty(),"source identity reuse");
 bad=input;bad.law.numeric.valid=false;check(!sc4InputError(bad).empty(),"expired numeric source");
 Sc4System system(input);auto origin=Sc4Clock::now();
 uint64_t previousVersion=0;
 for(size_t i=0;i<2;++i){auto out=system.process(i,true,Sc4Clock::now(),origin);check(out.updates==1 && out.proof.valid && out.proof.geometry.considered==9 && out.physical_evaluated==481401,"single estimator raw output");check(out.state.timestamp.value()==int64_t(i+1)*50000000LL && out.version==out.proof.version && (!previousVersion || out.version==previousVersion+1),"actual output binding");previousVersion=out.version;check(out.timing.start_ns<=out.timing.selected_ns && out.timing.selected_ns<=out.timing.commit_ns && out.timing.commit_ns<=out.timing.proof_ns && out.timing.proof_ns<=out.timing.output_ns,"complete clock ordering");check(out.timing.core_ms<=out.timing.full_ms && !out.protected_output,"distinct compute/application levels");}
 Sc4Queue queue(2);auto now=Sc4Clock::now();check(queue.push({0,now,now}) && queue.push({1,now,now}) && !queue.push({2,now,now}) && queue.peak()==2,"finite queue does not silently drop oldest source");check(queue.cancel(1) && !queue.cancel(1) && queue.pop().index==0,"cancellation only removes pending job");
 Sc4System cancelled(input);auto refusal=cancelled.reject(0,"CANCELLED_BEFORE_PREPARE",now,now);check(refusal.updates==0 && !refusal.proof.valid && refusal.state.timestamp.value()==0,"cancel is zero native updates and never reuses a PL");auto subsequent=cancelled.process(1,true,now,now);check(subsequent.updates==0 && subsequent.functional=="INPUT_GAP_LATCHED" && !subsequent.proof.valid,"gap latched instead of empty healthy history restart");
 std::cout<<"SC4 unified source/time/identity/missing input/two unique commits; queue/cancel/gap PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
