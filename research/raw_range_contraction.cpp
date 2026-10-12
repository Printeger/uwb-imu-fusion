#include "raw_range_contraction.hpp"
#include <boost/numeric/interval.hpp>
#include <set>
namespace uwb_imu_pl::research {
using I=boost::numeric::interval<double>;
bool sameRawBatch(const UwbBatch& a,const UwbBatch& b){
 if(a.id!=b.id || a.timestamp!=b.timestamp || a.measurements.size()!=b.measurements.size() || a.covariance_model_id!=b.covariance_model_id || a.covariance_m2.rows()!=b.covariance_m2.rows() || a.covariance_m2.cols()!=b.covariance_m2.cols() || a.covariance_m2!=b.covariance_m2)return false;
 for(size_t i=0;i<a.measurements.size();++i){const auto& x=a.measurements[i];const auto& y=b.measurements[i];
  if(x.id!=y.id || x.factor_id!=y.factor_id || x.anchor_id!=y.anchor_id || x.tag_id!=y.tag_id || x.timestamp!=y.timestamp || x.range_m!=y.range_m || x.sigma_m!=y.sigma_m || x.anchor_position_m!=y.anchor_position_m || x.sequence!=y.sequence || x.quality_flags!=y.quality_flags || bool(x.lever_arm_body_m)!=bool(y.lever_arm_body_m))return false;
  if(x.lever_arm_body_m && *x.lever_arm_body_m!=*y.lever_arm_body_m)return false;
 }return true;
}
std::array<int,9> rawRangeFaultMasks(){return {-1,0,1,2,3,4,5,6,7};}
bool rawRangeShells(const RawRangeLawEvidence& law,const UwbBatch& b,int epoch,std::array<RawRangeShell,8>& shells){
 if(!law.valid || law.source_law_id!="raw-source-rw-v1" || law.frame!="world" || epoch<1 || epoch>50 || b.id.value()!=uint64_t(epoch) || b.timestamp.value()!=epoch*50000000LL || b.measurements.size()!=8 || b.covariance_m2.size()!=0 || !b.covariance_model_id.empty() || !sameRawBatch(law.batch,b))return false;
 std::set<uint64_t> anchors;
 for(size_t i=0;i<8;++i){const auto& m=b.measurements[i];double rho=law.rho_num[i];
  if(m.id.value()!=epoch*100+i+1 || m.factor_id!=FactorId(m.id.value()) || !anchors.insert(m.anchor_id.value()).second || m.timestamp!=b.timestamp || m.tag_id.value()!=0 || m.lever_arm_body_m || !m.anchor_position_m.allFinite() || !std::isfinite(m.range_m) || !std::isfinite(m.sigma_m) || m.sigma_m<=0 || !std::isfinite(rho) || rho<0 || rho>1e-8)return false;
  I delta=7.*I(m.sigma_m)+I(rho);I lower=I(m.range_m)-delta,upper=I(m.range_m)+delta;
  shells[i]={std::max(0.,lower.lower()),upper.upper()};
 }return true;
}
}
