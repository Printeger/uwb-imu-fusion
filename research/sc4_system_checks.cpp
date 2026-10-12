#include "sc4_system.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void check(bool condition,const char* reason){if(!condition)throw std::runtime_error(reason);}
int main(){try{
 auto input=sc4MatchedInput("config/sc4_system.yaml","SC3_U0",2);check(sc4InputError(input).empty(),"matched admission");
 auto bad=input;bad.imu[3].timestamp=bad.imu[2].timestamp;check(!sc4InputError(bad).empty(),"nonmonotone IMU");
 bad=input;bad.imu.erase(bad.imu.begin()+5);check(sc4InputError(bad)=="MATCHED_SAMPLE_GAP","missing shared sample");
 bad=input;bad.batches[1].measurements.clear();check(!sc4InputError(bad).empty(),"missing UWB");
 bad=input;bad.batches[1].measurements[0].id=bad.batches[0].measurements[0].id;check(!sc4InputError(bad).empty(),"source identity reuse");
 bad=input;bad.law.numeric.valid=false;check(!sc4InputError(bad).empty(),"expired numeric source");
 Sc4System system(input);auto origin=Sc4Clock::now();
 uint64_t previousVersion=0;
 for(size_t i=0;i<2;++i){auto out=system.process(i,true,Sc4Clock::now(),origin);check(out.updates==1 && out.proof.valid && out.proof.geometry.considered==9 && out.physical_evaluated==481401,"single estimator raw output");check(out.state.timestamp.value()==int64_t(i+1)*50000000LL && out.version==out.proof.version && (!previousVersion || out.version==previousVersion+1),"actual output binding");previousVersion=out.version;check(out.timing.start_ns<=out.timing.selected_ns && out.timing.selected_ns<=out.timing.commit_ns && out.timing.commit_ns<=out.timing.proof_ns && out.timing.proof_ns<=out.timing.output_ns,"complete clock ordering");check(out.timing.core_ms<=out.timing.full_ms && !out.protected_output,"distinct compute/application levels");}
 std::cout<<"SC4 unified source/time/identity/missing input/two unique commits PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
