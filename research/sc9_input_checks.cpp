#include "anchor_registry.hpp"
#include "raw_range_contraction.hpp"
#include "tag_support.hpp"
#include <iostream>
#include <limits>
using namespace uwb_imu_pl;
using namespace uwb_imu_pl::research;
void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
template<class F> void refuses(F f){bool caught=false;try{f();}catch(const std::exception&){caught=true;}require(caught,"expected refusal");}

int main(){try{
 const Eigen::Vector3d truth(.2,-.4,.7),lo(-3,-3,-3),hi(3,3,3);
 for(size_t n:{0,1,2,3,4,5,6,8,12,32,65}){
  AnchorRegistry registry;UwbBatch batch;std::vector<RawRangeShell> shells;
  std::vector<CanonicalRangeEvent> events;
  for(size_t i=0;i<n;++i){
   // Sparse64-bit physical IDs exceed uint8 and mask64, including UINT64_MAX.
   AnchorRecord anchor;anchor.id=AnchorId(i==0?UINT64_MAX:(uint64_t(1)<<48)+101*i);
   anchor.position_world_m={4*std::cos(double(i)),4*std::sin(double(i)),double(i%3)*2-2};
   anchor.frame="world";anchor.map_version="cal-v1";registry.declare(anchor,1);
   UwbMeasurement m;m.id=MeasurementId(1000+i);m.factor_id=FactorId(2000+i);m.anchor_id=anchor.id;
   m.anchor_position_m=anchor.position_world_m;m.timestamp=TimestampNs(123000000000LL+i);
   m.range_m=(truth-m.anchor_position_m).norm();m.sigma_m=.02;batch.measurements.push_back(m);
   shells.push_back({m.range_m-.01,m.range_m+.01});events.push_back({m,TimestampNs(124000000000LL),1,true});
  }
  const auto counts=validateRanges(events,registry,TimestampNs(124000000000LL));
  require(counts.registered==n && counts.visible==n && counts.measurements==n,"dynamic counts");
  require(rawRangeFaultMasks(batch).size()==n+1,"dynamic branch census");
  RawRangeBudget budget;budget.retain_support=true;budget.tight_shell=true;budget.lower_shell=true;budget.optimized=true;
  for(size_t cap:{size_t(0),size_t(7),size_t(256)}){
   budget.max_boxes=cap;auto out=contractRawRangeBox(lo,hi,batch,shells,truth,budget);
   require(out.valid && out.considered==n+1,"dynamic raw branch count");
   require((out.low.array()<=truth.array()).all() && (out.high.array()>=truth.array()).all(),"healthy containment");
   if(cap==0)require(out.low==lo && out.high==hi && out.frontier==n+1,"frontier changed domain");
  }
  if(n){
   auto duplicate=events.front();duplicate.measurement.id=MeasurementId(9999);duplicate.measurement.factor_id=FactorId(9999);
   events.push_back(duplicate);auto c=validateRanges(events,registry,TimestampNs(124000000000LL));
   require(c.visible==n && c.measurements==n+1,"M conflated with visible anchors");
   batch.measurements.push_back(duplicate.measurement);shells.push_back(shells.front());
   // Two impossible observations of ONE failed physical anchor. The legal
   // single-source branch must omit both; a single-row implementation fails.
   shells.front()={1000,1001};shells.back()={2000,2001};budget.max_boxes=4096;
   auto fault=contractRawRangeBox(lo,hi,batch,shells,truth,budget);
   require(fault.valid && fault.considered==n+1 && (fault.low.array()<=truth.array()).all() &&
           (fault.high.array()>=truth.array()).all(),"repeated-source fault lost truth");
   events.back().valid=false;c=validateRanges(events,registry,TimestampNs(124000000000LL));
   require(c.visible==n && c.measurements==n && c.invalid==1,"valid counts");
   events.push_back(events.front());refuses([&]{validateRanges(events,registry,TimestampNs(124000000000LL));});events.pop_back();
   auto id=events.front().measurement.anchor_id;registry.online(id,false);
   refuses([&]{registry.validate(events.front().measurement,1);});registry.online(id,true);
   registry.validate(events.front().measurement,1);
   auto a=registry.at(id,1);auto tamper=a;tamper.position_world_m.x()+=1;
   refuses([&]{registry.declare(tamper,1);});tamper.map_version="cal-v2";registry.declare(tamper,2);
   require(registry.at(id,1).position_world_m==a.position_world_m,"old calibration erased");
   refuses([&]{registry.validate(events.front().measurement,1);});
   events.front().measurement.anchor_position_m=tamper.position_world_m;registry.validate(events.front().measurement,2);
  }
 }
 // Analytic mirror oracle: coplanar anchor distances must retain +/-z.
 UwbBatch mirror;std::vector<RawRangeShell> shells;
 for(int i=0;i<5;++i){UwbMeasurement m;m.anchor_id=AnchorId(i);m.anchor_position_m={double(i%2)*4-2,double(i/2)*2-2,0};m.range_m=(Eigen::Vector3d(0,0,1)-m.anchor_position_m).norm();mirror.measurements.push_back(m);shells.push_back({m.range_m-1e-8,m.range_m+1e-8});}
 RawRangeBudget b;b.tight_shell=b.lower_shell=b.optimized=true;b.split_depth=2;b.max_boxes=4096;
 auto both=contractRawRangeBox(lo,hi,mirror,shells,truth,b);require(both.valid && both.low.z()<=-1 && both.high.z()>=1,"mirror branch discarded");
 shells.pop_back();require(!contractRawRangeBox(lo,hi,mirror,shells,truth,b).valid,"shell shape mismatch accepted");
 std::cout<<"SC9 dynamic registry/source-count/raw kernel PASS: N=0/1/2/3/4/5/6/8/12/32/65, sparse64, versions, repeat-source fault, mirror, frontier\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
