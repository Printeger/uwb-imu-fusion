#pragma once
#include "raw_domain.hpp"
#include <vector>
#include <set>
namespace uwb_imu_pl::research {
struct PrimitiveSource {
 std::string id,unit,distribution="independent-standard-normal";
 long long timestamp_ns=0; double sigma=0;
};
// Pre-register the full mission, including shared endpoint samples ONCE.
// True initial state is conditional fixed; its independent observation has
// 15 Gaussian coordinates. No biasHat, integration jitter, or per-factor
// auxiliary prior is a primitive source.
inline std::vector<PrimitiveSource> rawSourceInventory(int horizon,int anchors,
 const Eigen::Matrix<double,15,1>& initial,double sa,double sg,double ba,double bg,double range){
 if(horizon!=50 || anchors!=8 || !initial.allFinite() || (initial.array()<=0).any() || !(sa>0 && sg>0 && ba>0 && bg>0 && range>0))throw std::runtime_error("unregistered raw law");
 std::vector<PrimitiveSource> out;
 for(int a=0;a<15;++a)out.push_back({"initial:"+std::to_string(a),a<3?"rad":a<6?"m":a<9?"m/s":a<12?"m/s2":"rad/s","independent-standard-normal",0,initial(a)});
 for(int i=0;i<=10*horizon;++i)for(int a=0;a<6;++a)out.push_back({"adc:"+std::to_string(i*5000000LL)+":"+std::to_string(a),a<3?"m/s2":"rad/s","independent-standard-normal",i*5000000LL,a<3?sa:sg});
 for(int i=1;i<=10*horizon;++i)for(int a=0;a<6;++a)out.push_back({"bias-increment:"+std::to_string(i*5000000LL)+":"+std::to_string(a),a<3?"m/s2":"rad/s","independent-standard-normal",i*5000000LL,(a<3?ba:bg)*std::sqrt(.005)});
 for(int e=1;e<=horizon;++e)for(int a=0;a<anchors;++a)out.push_back({"uwb:"+std::to_string(e)+":"+std::to_string(a),"m","independent-standard-normal",e*50000000LL,range});
 return out;
}
}
