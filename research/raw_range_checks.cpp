#include "scenario_material.hpp"
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool v,const char* s){if(!v)throw std::runtime_error(s);}
int main(){try{
 auto masks=rawRangeFaultMasks();require(masks.size()==9 && masks.front()==-1 && masks.back()==7,"complete masks");
 for(auto kind:{"U0","U1","U2","U3"}){NoisyMotionScenario s("config/hybrid_raw.yaml",kind,20261010,16);
  for(size_t e=0;e<s.uwb.size();++e){std::array<RawRangeShell,8> shell;require(rawRangeShells(s.range_certificates[e],s.uwb[e],e+1,shell),"source numeric certificate/shell");
   auto changed=s.uwb[e];changed.measurements[0].range_m+=1e-8;require(!rawRangeShells(s.range_certificates[e],changed,e+1,shell),"changed raw range refused");
  }
 }
 std::cout<<"raw range source/shell checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
