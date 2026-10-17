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
std::vector<int> rawRangeFaultMasks(const UwbBatch& batch){
 if(batch.measurements.size()>size_t(std::numeric_limits<int>::max()))throw std::length_error("RANGE_INDEX_CAPACITY");
 std::vector<int> masks{-1};std::set<AnchorId> seen;
 for(size_t i=0;i<batch.measurements.size();++i)if(seen.insert(batch.measurements[i].anchor_id).second)masks.push_back(static_cast<int>(i));
 return masks;
}
bool rawRangeShells(const RawRangeLawEvidence& law,const UwbBatch& b,int epoch,std::array<RawRangeShell,8>& shells){
 return rawRangeShells(law,b,epoch,shells,50,7);
}
bool rawRangeShells(const RawRangeLawEvidence& law,const UwbBatch& b,int epoch,std::array<RawRangeShell,8>& shells,int mission,double q){
 if(mission<1 || !std::isfinite(q) || q<=0 || !law.valid || law.source_law_id!="raw-source-rw-v1" || law.frame!="world" || epoch<1 || epoch>mission || b.id.value()!=uint64_t(epoch) || b.timestamp.value()!=epoch*50000000LL || b.measurements.size()!=8 || b.covariance_m2.size()!=0 || !b.covariance_model_id.empty() || !sameRawBatch(law.batch,b) || !sameRawBatch(law.sealed_batch_,b) || law.rho_num!=law.sealed_rho_)return false;
 std::set<uint64_t> anchors;
 for(size_t i=0;i<8;++i){const auto& m=b.measurements[i];double rho=law.rho_num[i];
  if(m.id.value()!=epoch*100+i+1 || m.factor_id!=FactorId(m.id.value()) || !anchors.insert(m.anchor_id.value()).second || m.timestamp!=b.timestamp || m.tag_id.value()!=0 || m.lever_arm_body_m || !m.anchor_position_m.allFinite() || !std::isfinite(m.range_m) || !std::isfinite(m.sigma_m) || m.sigma_m<=0 || !std::isfinite(rho) || rho<0 || rho>1e-8)return false;
  I delta=I(q)*I(m.sigma_m)+I(rho);I lower=I(m.range_m)-delta,upper=I(m.range_m)+delta;
  if(!std::isfinite(lower.lower()) || !std::isfinite(upper.upper()))return false;
  shells[i]={std::max(0.,lower.lower()),upper.upper()};
 }return true;
}

namespace {
using Box=std::array<I,3>;
template<class T>T squaredNorm(const Eigen::Vector3d& a){T n(0);for(int j=0;j<3;++j)n+=square(T(a(j)));return n;}
template<class T>void checkFinite(T x){if(!std::isfinite(x.lower()) || !std::isfinite(x.upper()))throw std::runtime_error("nonfinite interval arithmetic");}
template<class T>bool intersect(T& x,T y){checkFinite(y);double l=std::max(x.lower(),y.lower()),h=std::min(x.upper(),y.upper());if(l>h)return false;x=T(l,h);return true;}
bool finiteBox(const Box& b){for(auto x:b)if(!std::isfinite(x.lower()) || !std::isfinite(x.upper()))return false;return true;}
void bounds(RawRangeResult& r,const Eigen::Vector3d& actual){
 for(int j=0;j<3;++j)r.pl(j)=abs(I(r.low(j),r.high(j))-I(actual(j))).upper();
 r.hpl=sqrt(square(I(r.pl.x()))+square(I(r.pl.y()))).upper();
}
template<class T>std::array<T,3> cross(const std::array<T,3>& a,const std::array<T,3>& b){
 return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
template<class T>T dot(const std::array<T,3>& a,const std::array<T,3>& b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
template<class T>T determinant(const std::array<std::array<T,3>,3>& a){return dot(a[0],cross(a[1],a[2]));}
template<class T>bool linearContract(std::array<T,3>& box,const std::vector<std::array<T,3>>& rows,const std::vector<T>& rhs,const std::vector<T>& without_reference,T reference_squared,const std::vector<size_t>& eligible){
 if(eligible.size()<3)return true;
 // Pivot selection affects strength only. All chosen equations remain exact
 // necessary squared-range differences; no floating inverse is trusted.
 size_t first=eligible.front(),second=first,third=first;double best=0;
 for(auto i:eligible){const auto q=dot(rows[i],rows[i]);checkFinite(q);if(q.lower()>best){best=q.lower();first=i;}}
 if(best<=0)return true;best=0;
 for(auto i:eligible)if(i!=first){auto c=cross(rows[first],rows[i]);const auto q=dot(c,c);checkFinite(q);if(q.lower()>best){best=q.lower();second=i;}}
 if(best<=0)return true;best=0;
 for(auto i:eligible)if(i!=first&&i!=second){const auto q=abs(dot(rows[first],cross(rows[second],rows[i])));checkFinite(q);if(q.lower()>best){best=q.lower();third=i;}}
 if(best<=0)return true;
 const std::array<std::array<T,3>,3> matrix{rows[first],rows[second],rows[third]};
 const T denominator=determinant(matrix);checkFinite(denominator);
 if(denominator.lower()<=0&&denominator.upper()>=0)return true;
 const std::array<T,3> values{rhs[first],rhs[second],rhs[third]};
 const std::array<size_t,3> selected{first,second,third};
 // Cramer's rule evaluated entirely with outward intervals encloses the
 // solution for every admitted range/geometry value. Rank-uncertain cases
 // keep the original box and still run the existing spherical contractor.
 for(int j=0;j<3;++j){auto numerator=matrix;for(int k=0;k<3;++k)numerator[k][j]=values[k];
  if(!intersect(box[j],determinant(numerator)/denominator))return false;
  // All three differences reuse the SAME r_ref^2. Combine its coefficient
  // before interval evaluation, instead of treating its occurrences as
  // unrelated values. Both this form and the direct Cramer form are necessary
  // outer bounds; intersecting them also keeps tiny inverse-roundoff effects
  // from weakening the previous form. No noise independence is introduced.
  T common_weight(0),separate(0);
  for(int k=0;k<3;++k){auto unit=matrix;for(int row=0;row<3;++row)unit[row][j]=T(row==k?1:0);
   const T weight=determinant(unit)/denominator;common_weight+=weight;separate+=weight*without_reference[selected[k]];}
  if(!intersect(box[j],separate+common_weight*reference_squared))return false;
 }
 return true;
}
// Every update is a necessary condition; interval dependency only weakens it.
template<class T>bool propagateImpl(std::array<T,3>& box,const UwbBatch& b,const std::vector<RawRangeShell>& shells,int omitted,unsigned passes,bool tight_shell,bool optimized,bool lower_shell,bool precondition){
 using I=T;using Box=std::array<I,3>;
 auto retained=[&](size_t i){return omitted<0 || b.measurements[i].anchor_id!=b.measurements.at(omitted).anchor_id;};
 size_t ref=0;while(ref<b.measurements.size() && !retained(ref))++ref;
 if(ref==b.measurements.size())return true; // Zero constraints: keep the entire prior domain.
 for(size_t i=0;i<b.measurements.size();++i)if(retained(i) && shells[i].high<shells[i].low)return false;
 std::vector<I> shell_squared(shells.size()),upper_squared(shells.size()),linear_rhs(shells.size()),without_reference(precondition?shells.size():0);std::vector<std::array<I,3>> coefficients(shells.size());
 if(optimized||precondition)for(int i=0;i<b.measurements.size();++i){if(!retained(i))continue;shell_squared[i]=square(I(shells[i].low,shells[i].high));upper_squared[i]=square(I(shells[i].high));const auto& a=b.measurements[i].anchor_position_m;const auto& a0=b.measurements[ref].anchor_position_m;for(int j=0;j<3;++j)coefficients[i][j]=2.*(I(a(j))-I(a0(j)));linear_rhs[i]=squaredNorm<I>(a)-squaredNorm<I>(a0)+square(I(shells[ref].low,shells[ref].high))-square(I(shells[i].low,shells[i].high));if(precondition)without_reference[i]=squaredNorm<I>(a)-squaredNorm<I>(a0)-shell_squared[i];}
 if(precondition){std::vector<size_t> eligible;for(size_t i=0;i<b.measurements.size();++i)if(i!=ref&&retained(i))eligible.push_back(i);
  if(!linearContract(box,coefficients,linear_rhs,without_reference,shell_squared[ref],eligible))return false;}
 for(unsigned pass=0;pass<passes;++pass){Box previous=box;
  for(size_t i=0;i<b.measurements.size();++i)if(retained(i)){
   const auto& a=b.measurements[i].anchor_position_m;
   I distance2(0);for(int j=0;j<3;++j)distance2+=square(box[j]-I(a(j)));
   I shell2=optimized?shell_squared[i]:square(I(shells[i].low,shells[i].high));
   checkFinite(distance2);checkFinite(shell2);
   if(distance2.lower()>shell2.upper() || distance2.upper()<shell2.lower())return false;
   // Upper shell gives each coordinate |p_j-a_j| <= u. No sign
   // selection from lower shell, so both mirror branches are retained.
   for(int j=0;j<3;++j)if(!intersect(box[j],I(a(j))+I(-shells[i].high,shells[i].high)))return false;
   if(tight_shell)for(int j=0;j<3;++j){
    // Sum of other squared distances is a necessary nonnegative quantity.
    I rest(0);for(int k=0;k<3;++k)if(k!=j)rest+=square(box[k]-I(a(k)));
    I cap=(optimized?upper_squared[i]:square(I(shells[i].high)))-I(rest.lower());if(cap.upper()<0)return false;
    double radius=sqrt(I(0,std::max(0.,cap.upper()))).upper();
    if(!intersect(box[j],I(a(j))+I(-radius,radius)))return false;
    if(lower_shell){I floor=square(I(shells[i].low))-I(rest.upper());double minimum=sqrt(I(std::max(0.,floor.lower()),std::max(0.,floor.upper()))).lower();I delta=box[j]-I(a(j));
     if(delta.lower()>=0){if(!intersect(box[j],I(a(j))+I(minimum,radius)))return false;}
     else if(delta.upper()<=0){if(!intersect(box[j],I(a(j))+I(-radius,-minimum)))return false;}
    }
   }
   if(int(i)==ref)continue;
   const auto& a0=b.measurements[ref].anchor_position_m;
   std::array<I,3> c;if(optimized)c=coefficients[i];else for(int j=0;j<3;++j)c[j]=2.*(I(a(j))-I(a0(j)));
   I rhs=optimized?linear_rhs[i]:squaredNorm<I>(a)-squaredNorm<I>(a0)+square(I(shells[ref].low,shells[ref].high))-square(I(shells[i].low,shells[i].high));
   checkFinite(rhs);for(auto coefficient:c)checkFinite(coefficient);
   for(int j=0;j<3;++j){if(c[j].lower()<=0 && c[j].upper()>=0)continue;
    I remainder=rhs;for(int k=0;k<3;++k)if(k!=j)remainder-=c[k]*box[k];
    if(!intersect(box[j],remainder/c[j]))return false;
   }
  }
  if(optimized){bool unchanged=true;for(int j=0;j<3;++j)unchanged&=previous[j].lower()==box[j].lower() && previous[j].upper()==box[j].upper();if(unchanged)break;}
 }
 return true;
}
bool propagate(Box& box,const UwbBatch& b,const std::vector<RawRangeShell>& shells,int omitted,const RawRangeBudget& budget){
 if(!budget.scoped_rounding)return propagateImpl(box,b,shells,omitted,budget.passes,budget.tight_shell,budget.optimized,budget.lower_shell,budget.linear_precondition);
 // Exactly the same directed operations, with one exception-safe rounding
 // guard around this pure interval kernel. No split/center/Eigen arithmetic
 // runs inside it; the caller's original mode is restored on every exit.
 I::traits_type::rounding guard;
 using U=boost::numeric::interval_lib::unprotect<I>::type;
 std::array<U,3> local;for(int j=0;j<3;++j)local[j]=U(box[j].lower(),box[j].upper());
 const bool result=propagateImpl(local,b,shells,omitted,budget.passes,budget.tight_shell,budget.optimized,budget.lower_shell,budget.linear_precondition);
 if(result)for(int j=0;j<3;++j)box[j]=I(local[j].lower(),local[j].upper());
 return result;
}
}
RawRangeResult contractRawRangeBox(const Eigen::Vector3d& low,const Eigen::Vector3d& high,
 const UwbBatch& batch,const std::vector<RawRangeShell>& shells,const Eigen::Vector3d& actual,RawRangeBudget budget){
 RawRangeResult r;auto begin=std::chrono::steady_clock::now();
 if(!low.allFinite() || !high.allFinite() || !actual.allFinite() || (low.array()>high.array()).any() || batch.measurements.size()!=shells.size() || batch.measurements.size()>size_t(std::numeric_limits<int>::max()) || budget.passes>64 || budget.split_depth>16){r.reason="invalid kernel input";return r;}
 for(size_t i=0;i<batch.measurements.size();++i)if(!batch.measurements[i].anchor_position_m.allFinite() || !std::isfinite(shells[i].low) || !std::isfinite(shells[i].high) || shells[i].low<0){r.reason="invalid shell";return r;}
 r.input_low=low;r.input_high=high;r.low=low;r.high=high;bounds(r,actual);r.old_pl=r.pl;r.old_hpl=r.hpl;
 Box original;for(int j=0;j<3;++j)original[j]=I(low(j),high(j));
 Eigen::Vector3d unionLow=Eigen::Vector3d::Constant(INFINITY),unionHigh=Eigen::Vector3d::Constant(-INFINITY);
 int current_mask=-1;bool current_frontier=false;
 auto retain=[&](const Box& box){if(budget.retain_support){RawSupportBox leaf;leaf.omitted=current_mask;leaf.frontier=current_frontier;for(int j=0;j<3;++j){leaf.low(j)=box[j].lower();leaf.high(j)=box[j].upper();}r.support.push_back(leaf);}++r.retained;for(int j=0;j<3;++j){unionLow(j)=std::min(unionLow(j),box[j].lower());unionHigh(j)=std::max(unionHigh(j),box[j].upper());}};
 try{
  // Run-time directed addition witness, same as the inertial kernel.
  I witness=I(1)+I(std::ldexp(1.,-53));if(witness.lower()>1 || witness.upper()<std::nextafter(1.,INFINITY))throw std::runtime_error("directed arithmetic unavailable");
  for(int omitted:rawRangeFaultMasks(batch)){
   if(budget.all_ranges_healthy && omitted!=-1)continue;
   current_mask=omitted;current_frontier=false;++r.considered;size_t branchRetained=r.retained;
   std::deque<std::pair<Box,unsigned>> queue{{original,0}};
   while(!queue.empty()){
    r.peak_boxes=std::max(r.peak_boxes,queue.size());
    if(r.visited>=budget.max_boxes){current_frontier=true;r.budget_exhausted=true;r.frontier+=queue.size();for(const auto& item:queue)retain(item.first);queue.clear();break;}
    auto item=queue.front();queue.pop_front();++r.visited;
    if(!propagate(item.first,batch,shells,omitted,budget))continue;
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
 }catch(const std::exception& e){r.support.clear();r.low=low;r.high=high;bounds(r,actual);r.valid=r.pl.allFinite() && std::isfinite(r.hpl);r.status="FALLBACK_INERTIAL_VERIFIED";r.reason=e.what();}
 r.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return r;
}
RawRangeResult contractRawRangeBox(const Eigen::Vector3d& low,const Eigen::Vector3d& high,
 const UwbBatch& batch,const std::array<RawRangeShell,8>& shells,const Eigen::Vector3d& actual,RawRangeBudget budget){
 return contractRawRangeBox(low,high,batch,std::vector<RawRangeShell>(shells.begin(),shells.end()),actual,budget);
}
RawRangeResult issueRawRangeContraction(const RawInclusionEvidence& inertial,const RawRangeLawEvidence& law,
 const EpochTransaction& tx,const CommitReceipt& receipt,const NavigationState& state,const Eigen::Vector3d& lever,RawRangeBudget budget){
 RawRangeResult refused;std::array<RawRangeShell,8> shells;
 Eigen::Vector3d actual=state.position_world_m+state.q_world_body*lever;
 if(budget.all_ranges_healthy || inertial.epoch<1 || inertial.epoch>50 || !inertial.low.allFinite() || !inertial.high.allFinite() || !inertial.pl.allFinite() || !std::isfinite(inertial.hpl) || (inertial.low.array()>inertial.high.array()).any() || (inertial.pl.array()<0).any() || inertial.transaction_id!=receipt.transaction_id || inertial.graph_version!=receipt.graph_version || inertial.timestamp!=receipt.state_timestamp || inertial.committed_tag!=actual || inertial.committed_lever!=lever || !inertial.valid || !inertial.receipt_bound || inertial.source_law_id!="raw-source-rw-v1" || inertial.scope!="healthy-IMU-single-UWB-episode-50-v1" || inertial.horizon!=50 || inertial.epoch!=int(tx.proposed_epoch) || inertial.event_risk!=std::ldexp(2.*6421,-35) || tx.previous_epoch+1!=tx.proposed_epoch || tx.begin.value()!=(inertial.epoch-1)*50000000LL || tx.end!=tx.uwb_batch.timestamp || receipt.transaction_id!=tx.id || receipt.committed_epoch!=tx.proposed_epoch || receipt.graph_version!=tx.base_graph_version+1 || receipt.backend_updates!=1 || receipt.state_timestamp!=tx.end || state.timestamp!=tx.end || !lever.allFinite() || !actual.allFinite() || !rawRangeShells(law,tx.uwb_batch,inertial.epoch,shells)){refused.reason="raw law/batch/inertial/receipt/time/frame refused";return refused;}
 // Recheck inertial bound at THIS center: sidecars cannot swap an evidence
 // generated for a different committed state or a modified box.
 for(int j=0;j<3;++j)if(inertial.pl(j)!=abs(I(inertial.low(j),inertial.high(j))-I(actual(j))).upper()){refused.reason="inertial center/box mismatch";return refused;}
 auto r=contractRawRangeBox(inertial.low,inertial.high,tx.uwb_batch,shells,actual,budget);r.event_risk=inertial.event_risk;return r;
}
}
