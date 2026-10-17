#include "raw_range_contraction.hpp"
#include <cfenv>
#include <iostream>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
void equal(const RawRangeResult& a,const RawRangeResult& b){
 require(a.valid==b.valid&&a.status==b.status&&a.reason==b.reason&&a.low==b.low&&a.high==b.high&&a.pl==b.pl&&a.hpl==b.hpl,"scoped rounding changed result");
 require(a.considered==b.considered&&a.empty==b.empty&&a.unresolved==b.unresolved&&a.visited==b.visited&&a.retained==b.retained&&a.frontier==b.frontier&&a.budget_exhausted==b.budget_exhausted&&a.support.size()==b.support.size(),"scoped rounding changed branch/frontier accounting");
 for(size_t i=0;i<a.support.size();++i){const auto& x=a.support[i];const auto& y=b.support[i];require(x.low==y.low&&x.high==y.high&&x.omitted==y.omitted&&x.frontier==y.frontier,"scoped rounding changed retained support");}
}
int main(){try{
 const int original=std::fegetround();size_t cases=0;
 for(size_t n:{0,1,5,12,65}){
  std::fesetround(FE_TONEAREST);UwbBatch batch;std::vector<RawRangeShell> shells;const Eigen::Vector3d truth(.2,-.1,1.);
  for(size_t k=0;k<n;++k){UwbMeasurement m;m.anchor_id=AnchorId((uint64_t(1)<<49)+k/2);m.anchor_position_m={4*std::cos(double(k/2)),4*std::sin(double(k/2)),double(k/2%3)*2};m.range_m=(truth-m.anchor_position_m).norm();batch.measurements.push_back(m);shells.push_back({m.range_m-.02,m.range_m+.02});}
  for(int mode:{FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO})for(size_t capacity:{0,1,256}){
   std::fesetround(mode);RawRangeBudget budget;budget.optimized=true;budget.tight_shell=true;budget.lower_shell=true;budget.retain_support=true;budget.split_depth=2;budget.max_boxes=capacity;
   auto a=contractRawRangeBox(Eigen::Vector3d(-2,-2,-2),Eigen::Vector3d(2,2,2),batch,shells,truth,budget);require(std::fegetround()==mode,"reference leaked rounding mode");
   budget.scoped_rounding=true;auto b=contractRawRangeBox(Eigen::Vector3d(-2,-2,-2),Eigen::Vector3d(2,2,2),batch,shells,truth,budget);require(std::fegetround()==mode,"scoped kernel leaked rounding mode");equal(a,b);++cases;
  }
 }
 // Finite input that overflows interval squares must take the same fallback,
 // restoring even a non-default caller mode during exception unwinding.
 std::fesetround(FE_TOWARDZERO);UwbBatch huge;UwbMeasurement m;m.anchor_id=AnchorId(1);m.anchor_position_m.setConstant(1e308);huge.measurements.push_back(m);
 RawRangeBudget budget;budget.optimized=true;auto a=contractRawRangeBox(Eigen::Vector3d::Zero(),Eigen::Vector3d::Ones(),huge,std::vector<RawRangeShell>{{1,2}},Eigen::Vector3d::Zero(),budget);
 budget.scoped_rounding=true;auto b=contractRawRangeBox(Eigen::Vector3d::Zero(),Eigen::Vector3d::Ones(),huge,std::vector<RawRangeShell>{{1,2}},Eigen::Vector3d::Zero(),budget);
 equal(a,b);require(std::fegetround()==FE_TOWARDZERO,"exception leaked rounding mode");std::fesetround(original);
 std::cout<<"SC9 scoped rounding PASS "<<cases<<" dynamic geometry/budget/caller-mode cases plus overflow fallback; identical boxes, supports and complete branches\n";return 0;
}catch(const std::exception& e){std::fesetround(FE_TONEAREST);std::cerr<<e.what()<<'\n';return 1;}}
