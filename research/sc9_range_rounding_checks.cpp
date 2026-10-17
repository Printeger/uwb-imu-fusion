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
  for(int mode:{FE_TONEAREST,FE_DOWNWARD,FE_UPWARD,FE_TOWARDZERO})for(size_t capacity:{0,1,256})for(bool precondition:{false,true}){
   std::fesetround(mode);RawRangeBudget budget;budget.optimized=true;budget.tight_shell=true;budget.lower_shell=true;budget.retain_support=true;budget.split_depth=2;budget.max_boxes=capacity;budget.linear_precondition=precondition;
   auto a=contractRawRangeBox(Eigen::Vector3d(-2,-2,-2),Eigen::Vector3d(2,2,2),batch,shells,truth,budget);require(std::fegetround()==mode,"reference leaked rounding mode");
   budget.scoped_rounding=true;auto b=contractRawRangeBox(Eigen::Vector3d(-2,-2,-2),Eigen::Vector3d(2,2,2),batch,shells,truth,budget);require(std::fegetround()==mode,"scoped kernel leaked rounding mode");equal(a,b);++cases;
   require(b.valid&&(b.low.array()<=truth.array()).all()&&(b.high.array()>=truth.array()).all(),"analytic sphere point lost");
  }
 }
 std::fesetround(FE_TONEAREST);size_t strict=0;
 for(int geometry=0;geometry<4;++geometry)for(int fault=-1;fault<5;++fault){
  UwbBatch batch;std::vector<RawRangeShell> shells;const Eigen::Vector3d truth(.2,-.1,1.);
  std::vector<Eigen::Vector3d> anchors{{4,0,0},{0,4,0},{0,0,4},{-4,-4,-4},{4,4,4}};
  if(geometry==1||geometry==2)for(auto& a:anchors)a.z()*=geometry==1?0:1e-12;
  if(geometry==3)anchors.back().z()=-4; // Original B2 has four points on x+y+z=4.
  for(size_t k=0;k<anchors.size();++k){UwbMeasurement m;m.anchor_id=AnchorId(k+1);m.anchor_position_m=anchors[k];m.range_m=(truth-anchors[k]).norm()+(int(k)==fault?2.25:0);batch.measurements.push_back(m);shells.push_back({m.range_m-.02,m.range_m+.02});}
  RawRangeBudget budget;budget.optimized=true;budget.tight_shell=true;budget.scoped_rounding=true;budget.retain_support=true;
  const Eigen::Vector3d low=Eigen::Vector3d::Constant(-20),high=-low;
  auto before=contractRawRangeBox(low,high,batch,shells,truth,budget);budget.linear_precondition=true;
  auto after=contractRawRangeBox(low,high,batch,shells,truth,budget);
  require(after.valid&&(after.low.array()<=truth.array()).all()&&(after.high.array()>=truth.array()).all(),"linear contractor lost single-fault true point");
  require((after.low.array()>=before.low.array()).all()&&(after.high.array()<=before.high.array()).all(),"linear contractor widened its input result");
  if(geometry==0){std::cout<<"linear fixture fault="<<fault<<" HPL before="<<before.hpl<<" after="<<after.hpl<<" status="<<after.status<<" reason="<<after.reason<<std::endl;require(after.hpl<.5,"well-conditioned linear system failed to contract");strict+=after.hpl<before.hpl;}
  if(geometry==1){equal(before,after);require(after.low.z()<=-truth.z()&&after.high.z()>=truth.z(),"rank-uncertain mirror lost");}
  if(geometry==3&&fault==-1){
   const Eigen::Vector3d mirror=truth+Eigen::Vector3d::Ones()*(2*(4-truth.sum())/3);
   for(int k:{0,1,2,4})require(std::abs((truth-anchors[k]).norm()-(mirror-anchors[k]).norm())<1e-12,"independent plane-reflection oracle failed");
   require((after.low.array()<=mirror.array()).all()&&(after.high.array()>=mirror.array()).all(),"original B2 legal single-source mirror removed");
   require((mirror-truth).head<2>().norm()>2,"B2 mirror does not exercise spatial AL failure");
   std::cout<<"original B2 mirror separation H="<<(mirror-truth).head<2>().norm()<<" retained HPL="<<after.hpl<<std::endl;
  }
 }
 require(strict==6,"linear contraction fixture did not exercise strict improvement");
 // Finite input that overflows interval squares must take the same fallback,
 // restoring even a non-default caller mode during exception unwinding.
 std::fesetround(FE_TOWARDZERO);UwbBatch huge;UwbMeasurement m;m.anchor_id=AnchorId(1);m.anchor_position_m.setConstant(1e308);huge.measurements.push_back(m);
 RawRangeBudget budget;budget.optimized=true;auto a=contractRawRangeBox(Eigen::Vector3d::Zero(),Eigen::Vector3d::Ones(),huge,std::vector<RawRangeShell>{{1,2}},Eigen::Vector3d::Zero(),budget);
 budget.scoped_rounding=true;auto b=contractRawRangeBox(Eigen::Vector3d::Zero(),Eigen::Vector3d::Ones(),huge,std::vector<RawRangeShell>{{1,2}},Eigen::Vector3d::Zero(),budget);
 equal(a,b);require(std::fegetround()==FE_TOWARDZERO,"exception leaked rounding mode");std::fesetround(original);
 std::cout<<"SC9 scoped rounding PASS "<<cases<<" dynamic geometry/budget/caller-mode cases plus overflow fallback; identical boxes, supports and complete branches; 24 linear rank/fault fixtures, 6 strict improvements, original B2 mirror retained\n";return 0;
}catch(const std::exception& e){std::fesetround(FE_TONEAREST);std::cerr<<e.what()<<'\n';return 1;}}
