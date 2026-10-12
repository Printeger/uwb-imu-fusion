#include "sc3_joint_scope.hpp"
#include "scenario_material.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void check(bool p,const char* m){if(!p)throw std::runtime_error(m);}
int main(){try{
 NoisyMotionScenario scene("config/sc3_joint.yaml","U0",20261010,1);auto pool=makeSc3FaultPool(scene.uwb[0]);check(pool.complete() && pool.at(0).identity=="NONE","full raw physical census");
 size_t u=0,a=0,g=0,ua=0,ug=0;
 for(size_t n=1;n<pool.size();++n){auto h=pool.at(n);check(h.hypothesis.id.value()==n+1 && h.hypothesis.modes.size()==size_t((h.uwb>=0)+(h.imu>=0)),"bijection/parameter dimension");
  if(h.imu<0)++u;else if(pool.imu[h.imu].unit.axis<3){if(h.uwb>=0)++ua;else ++a;}else{if(h.uwb>=0)++ug;else ++g;}
 }
 check(u==800 && a==300 && g==300 && ua==240000 && ug==240000,"H0/UWB/accel/gyro/joint categories cannot disappear");
 auto corrupted=pool;corrupted.imu.pop_back();check(!corrupted.complete(),"missing consumed family refused");corrupted=pool;corrupted.imu[0].identity=corrupted.imu[1].identity;check(!corrupted.complete(),"duplicate physical identity refused");
 bool refused=false;try{makeSc3FaultPool(scene.uwb[0],1399);}catch(const std::exception&){refused=true;}check(refused,"capacity cannot silently truncate requested scope");
 auto h=pool.at(1+800+600);check(h.uwb==0 && h.imu==0 && h.hypothesis.modes.size()==2,"two independent shared physical amplitudes");
 auto episode=pool.imu.at(28);auto closure=sc2SupportClosure({episode.identity,episode.unit.time_begin,episode.unit.time_end,episode.unit.axis},30);check(closure.valid,"upstream raw support mapper consumed");
 std::cout<<"SC3 factored physical census expected/represented=481401, proof-consumed=0; U800 A300 G300 UA240000 UG240000; identity/capacity/support PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
