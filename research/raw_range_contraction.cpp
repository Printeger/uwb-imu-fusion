#include "raw_range_contraction.hpp"
#include <boost/numeric/interval.hpp>
#include <set>
#include <deque>
#include <chrono>
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
 if(!law.valid || law.source_law_id!="raw-source-rw-v1" || law.frame!="world" || epoch<1 || epoch>50 || b.id.value()!=uint64_t(epoch) || b.timestamp.value()!=epoch*50000000LL || b.measurements.size()!=8 || b.covariance_m2.size()!=0 || !b.covariance_model_id.empty() || !sameRawBatch(law.batch,b) || !sameRawBatch(law.sealed_batch_,b) || law.rho_num!=law.sealed_rho_)return false;
 std::set<uint64_t> anchors;
 for(size_t i=0;i<8;++i){const auto& m=b.measurements[i];double rho=law.rho_num[i];
  if(m.id.value()!=epoch*100+i+1 || m.factor_id!=FactorId(m.id.value()) || !anchors.insert(m.anchor_id.value()).second || m.timestamp!=b.timestamp || m.tag_id.value()!=0 || m.lever_arm_body_m || !m.anchor_position_m.allFinite() || !std::isfinite(m.range_m) || !std::isfinite(m.sigma_m) || m.sigma_m<=0 || !std::isfinite(rho) || rho<0 || rho>1e-8)return false;
  I delta=7.*I(m.sigma_m)+I(rho);I lower=I(m.range_m)-delta,upper=I(m.range_m)+delta;
  shells[i]={std::max(0.,lower.lower()),upper.upper()};
 }return true;
}

namespace {
using Box=std::array<I,3>;
I squaredNorm(const Eigen::Vector3d& a){I n(0);for(int j=0;j<3;++j)n+=square(I(a(j)));return n;}
void checkFinite(I x){if(!std::isfinite(x.lower()) || !std::isfinite(x.upper()))throw std::runtime_error("nonfinite interval arithmetic");}
bool intersect(I& x,I y){checkFinite(y);double l=std::max(x.lower(),y.lower()),h=std::min(x.upper(),y.upper());if(l>h)return false;x=I(l,h);return true;}
bool finiteBox(const Box& b){for(auto x:b)if(!std::isfinite(x.lower()) || !std::isfinite(x.upper()))return false;return true;}
void bounds(RawRangeResult& r,const Eigen::Vector3d& actual){
 for(int j=0;j<3;++j)r.pl(j)=abs(I(r.low(j),r.high(j))-I(actual(j))).upper();
 r.hpl=sqrt(square(I(r.pl.x()))+square(I(r.pl.y()))).upper();
}
// Every update is a necessary condition; interval dependency only weakens it.
bool propagate(Box& box,const UwbBatch& b,const std::array<RawRangeShell,8>& shells,int omitted,unsigned passes){
 int ref=omitted==0?1:0;
 for(size_t i=0;i<8;++i)if(int(i)!=omitted && shells[i].high<shells[i].low)return false;
 for(unsigned pass=0;pass<passes;++pass){
  for(size_t i=0;i<8;++i)if(int(i)!=omitted){
   const auto& a=b.measurements[i].anchor_position_m;
   I distance2(0);for(int j=0;j<3;++j)distance2+=square(box[j]-I(a(j)));
   I shell2=square(I(shells[i].low,shells[i].high));
   checkFinite(distance2);checkFinite(shell2);
   if(distance2.lower()>shell2.upper() || distance2.upper()<shell2.lower())return false;
   // Upper shell gives each coordinate |p_j-a_j| <= u. No sign
   // selection from lower shell, so both mirror branches are retained.
   for(int j=0;j<3;++j)if(!intersect(box[j],I(a(j))+I(-shells[i].high,shells[i].high)))return false;
   if(int(i)==ref)continue;
   const auto& a0=b.measurements[ref].anchor_position_m;
   std::array<I,3> c;for(int j=0;j<3;++j)c[j]=2.*(I(a(j))-I(a0(j)));
   I rhs=squaredNorm(a)-squaredNorm(a0)+square(I(shells[ref].low,shells[ref].high))-square(I(shells[i].low,shells[i].high));
   checkFinite(rhs);for(auto coefficient:c)checkFinite(coefficient);
   for(int j=0;j<3;++j){if(c[j].lower()<=0 && c[j].upper()>=0)continue;
    I remainder=rhs;for(int k=0;k<3;++k)if(k!=j)remainder-=c[k]*box[k];
    if(!intersect(box[j],remainder/c[j]))return false;
   }
  }
 }
 return true;
}
}
RawRangeResult contractRawRangeBox(const Eigen::Vector3d& low,const Eigen::Vector3d& high,
 const UwbBatch& batch,const std::array<RawRangeShell,8>& shells,const Eigen::Vector3d& actual,RawRangeBudget budget){
 RawRangeResult r;auto begin=std::chrono::steady_clock::now();
 if(!low.allFinite() || !high.allFinite() || !actual.allFinite() || (low.array()>high.array()).any() || batch.measurements.size()!=8 || budget.passes>64 || budget.split_depth>16){r.reason="invalid kernel input";return r;}
 for(size_t i=0;i<8;++i)if(!batch.measurements[i].anchor_position_m.allFinite() || !std::isfinite(shells[i].low) || !std::isfinite(shells[i].high) || shells[i].low<0){r.reason="invalid shell";return r;}
 r.low=low;r.high=high;bounds(r,actual);r.old_pl=r.pl;r.old_hpl=r.hpl;
 Box original;for(int j=0;j<3;++j)original[j]=I(low(j),high(j));
 Eigen::Vector3d unionLow=Eigen::Vector3d::Constant(INFINITY),unionHigh=Eigen::Vector3d::Constant(-INFINITY);
 auto retain=[&](const Box& box){++r.retained;for(int j=0;j<3;++j){unionLow(j)=std::min(unionLow(j),box[j].lower());unionHigh(j)=std::max(unionHigh(j),box[j].upper());}};
 try{
  // Run-time directed addition witness, same as the inertial kernel.
  I witness=I(1)+I(std::ldexp(1.,-53));if(witness.lower()>1 || witness.upper()<std::nextafter(1.,INFINITY))throw std::runtime_error("directed arithmetic unavailable");
  for(int omitted:rawRangeFaultMasks()){
   ++r.considered;size_t branchRetained=r.retained;
   std::deque<std::pair<Box,unsigned>> queue{{original,0}};
   while(!queue.empty()){
    r.peak_boxes=std::max(r.peak_boxes,queue.size());
    if(r.visited>=budget.max_boxes){r.budget_exhausted=true;r.frontier+=queue.size();for(const auto& item:queue)retain(item.first);queue.clear();break;}
    auto item=queue.front();queue.pop_front();++r.visited;
    if(!propagate(item.first,batch,shells,omitted,budget.passes))continue;
    if(!finiteBox(item.first))throw std::runtime_error("nonfinite interval arithmetic");
    int axis=0;for(int j=1;j<3;++j)if(item.first[j].upper()-item.first[j].lower()>item.first[axis].upper()-item.first[axis].lower())axis=j;
    double mid=item.first[axis].lower()/2+item.first[axis].upper()/2;
    if(item.second<budget.split_depth && mid>item.first[axis].lower() && mid<item.first[axis].upper()){
     auto right=item.first;right[axis]=I(mid,right[axis].upper());item.first[axis]=I(item.first[axis].lower(),mid);
     queue.push_back({item.first,item.second+1});queue.push_back({right,item.second+1});
    }else retain(item.first);
   }
   if(r.retained==branchRetained)++r.empty;else ++r.unresolved;
  }
  if(!r.retained){r.status="MODEL_INCONSISTENT";r.reason="all current fault branches proved empty; no bound issued";r.pl.setConstant(INFINITY);r.hpl=INFINITY;}
  else{r.low=unionLow;r.high=unionHigh;bounds(r,actual);r.valid=r.pl.allFinite() && std::isfinite(r.hpl);
   if(!r.valid)throw std::runtime_error("nonfinite transferred bound");
   r.status=(r.low!=low || r.high!=high)?"CONTRACTED_VERIFIED":"UNCHANGED_VERIFIED";
  }
 }catch(const std::exception& e){r.low=low;r.high=high;bounds(r,actual);r.valid=r.pl.allFinite() && std::isfinite(r.hpl);r.status="FALLBACK_INERTIAL_VERIFIED";r.reason=e.what();}
 r.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return r;
}
RawRangeResult issueRawRangeContraction(const RawInclusionEvidence& inertial,const RawRangeLawEvidence& law,
 const EpochTransaction& tx,const CommitReceipt& receipt,const NavigationState& state,const Eigen::Vector3d& lever,RawRangeBudget budget){
 RawRangeResult refused;std::array<RawRangeShell,8> shells;
 Eigen::Vector3d actual=state.position_world_m+state.q_world_body*lever;
 if(inertial.transaction_id!=receipt.transaction_id || inertial.graph_version!=receipt.graph_version || inertial.timestamp!=receipt.state_timestamp || inertial.committed_tag!=actual || inertial.committed_lever!=lever || !inertial.valid || !inertial.receipt_bound || inertial.source_law_id!="raw-source-rw-v1" || inertial.scope!="healthy-IMU-single-UWB-episode-50-v1" || inertial.horizon!=50 || inertial.epoch!=int(tx.proposed_epoch) || inertial.event_risk!=std::ldexp(2.*6421,-35) || tx.previous_epoch+1!=tx.proposed_epoch || tx.begin.value()!=(inertial.epoch-1)*50000000LL || tx.end!=tx.uwb_batch.timestamp || receipt.transaction_id!=tx.id || receipt.committed_epoch!=tx.proposed_epoch || receipt.graph_version!=tx.base_graph_version+1 || receipt.backend_updates!=1 || receipt.state_timestamp!=tx.end || state.timestamp!=tx.end || !lever.allFinite() || !actual.allFinite() || !rawRangeShells(law,tx.uwb_batch,inertial.epoch,shells)){refused.reason="raw law/batch/inertial/receipt/time/frame refused";return refused;}
 // Recheck inertial bound at THIS center: sidecars cannot swap an evidence
 // generated for a different committed state or a modified box.
 for(int j=0;j<3;++j)if(inertial.pl(j)!=abs(I(inertial.low(j),inertial.high(j))-I(actual(j))).upper()){refused.reason="inertial center/box mismatch";return refused;}
 auto r=contractRawRangeBox(inertial.low,inertial.high,tx.uwb_batch,shells,actual,budget);r.event_risk=inertial.event_risk;return r;
}
}
