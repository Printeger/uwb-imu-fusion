#include "scenario_material.hpp"
#include <iostream>
#include <boost/multiprecision/cpp_dec_float.hpp>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool v,const char* s){if(!v)throw std::runtime_error(s);}

using MP=boost::multiprecision::cpp_dec_float_100;
MP distance(const Eigen::Vector3d& p,const Eigen::Vector3d& a){MP sum=0;for(int j=0;j<3;++j){MP d=MP(p(j))-MP(a(j));sum+=d*d;}return sqrt(sum);}
bool feasible(const Eigen::Vector3d& p,const UwbBatch& b,const std::array<RawRangeShell,8>& sh){
 for(int omitted:rawRangeFaultMasks()){bool good=true;for(int i=0;i<8;++i)if(i!=omitted){MP d=distance(p,b.measurements[i].anchor_position_m);good=good && d>=MP(sh[i].low) && d<=MP(sh[i].high);}if(good)return true;}return false;
}
void contains(const RawRangeResult& r,const Eigen::Vector3d& p){require(r.valid && (r.low.array()<=p.array()).all() && (r.high.array()>=p.array()).all(),"feasible point lost");}
void geometryChecks(){
 UwbBatch b;std::array<RawRangeShell,8> shells;
 for(int i=0;i<8;++i){UwbMeasurement m;m.anchor_position_m={double(i&1)*8-4,double((i>>1)&1)*8-4,double((i>>2)&1)*8-4};b.measurements.push_back(m);}
 Eigen::Vector3d point(.25,-.5,.75),lo=Eigen::Vector3d::Constant(-20),hi=-lo;
 // Fault at EACH physical source, boundary noise with both signs, weak and
 // zero faults. No selected healthy subset is passed to the kernel.
 for(int fault=-1;fault<8;++fault)for(double sign:{-1.,1.})for(double magnitude:{0.,.02,20.}){
  for(int i=0;i<8;++i){double z=distance(point,b.measurements[i].anchor_position_m).convert_to<double>()+sign*.7+(i==fault?magnitude:0);shells[i]={std::max(0.,std::nextafter(z-.70000000000001,-INFINITY)),std::nextafter(z+.70000000000001,INFINITY)};}
  require(feasible(point,b,shells),"independent 100-digit boundary oracle");
  auto r=contractRawRangeBox(lo,hi,b,shells,Eigen::Vector3d(100,-100,50));contains(r,point);
  require((r.low.array()>=lo.array()).all() && (r.high.array()<=hi.array()).all() && r.hpl<=r.old_hpl && (r.pl.array()<=r.old_pl.array()).all(),"subset and same-center monotonic bound");
  require(r.considered==9 && r.status=="CONTRACTED_VERIFIED","all branches and nontrivial contraction");
  // Budget expires with nonempty frontier, including unvisited fault roots.
  auto budget=contractRawRangeBox(lo,hi,b,shells,point,{1,8,3});contains(budget,point);require(budget.budget_exhausted && budget.frontier>0 && budget.considered==9,"frontier/branch union preserved");
  auto zero=contractRawRangeBox(lo,hi,b,shells,point,{0,8,3});require(zero.low==lo && zero.high==hi && zero.frontier==9,"zero budget retains original");
 }
 // Independent dense lattice oracle checks arbitrary feasible points, rather
 // than only the point used to generate the shell family.
 for(int i=0;i<8;++i){double d=distance(point,b.measurements[i].anchor_position_m).convert_to<double>();shells[i]={d-.7,d+.7};}
 auto r=contractRawRangeBox(lo,hi,b,shells,point,{256,8,2});int checked=0;
 for(int x=-8;x<=8;++x)for(int y=-8;y<=8;++y)for(int z=-8;z<=8;++z){Eigen::Vector3d p(x*.25,y*.25,z*.25);if(feasible(p,b,shells)){contains(r,p);++checked;}}require(checked>10,"nontrivial lattice oracle");
 // Coplanar mirrors: no GT side is selected. Nearly singular slab coefficient
 // and zero distance use literal interval arithmetic, not rank thresholds.
 for(auto& m:b.measurements)m.anchor_position_m.z()=0;
 for(double height:{0.,1e-10,1.}){
  Eigen::Vector3d p(0,0,height),mirror(0,0,-height);
  for(int i=0;i<8;++i){double d=distance(p,b.measurements[i].anchor_position_m).convert_to<double>();shells[i]={std::max(0.,d-1e-12),d+1e-12};}
  auto mirrors=contractRawRangeBox(lo,hi,b,shells,p,{256,8,2});contains(mirrors,p);contains(mirrors,mirror);
 }
 // 1D and 2D analytical upper-shell domains are exact necessary conditions.
 for(int dim:{1,2,3}){for(auto& m:b.measurements)m.anchor_position_m.setZero();for(auto& sh:shells)sh={0,1};Eigen::Vector3d l=Eigen::Vector3d::Zero(),h=l;for(int j=0;j<dim;++j){l(j)=-10;h(j)=10;}
  auto analytic=contractRawRangeBox(l,h,b,shells,point);require(analytic.low.head(dim).minCoeff()==-1 && analytic.high.head(dim).maxCoeff()==1,"analytic unit ball box");contains(analytic,Eigen::Vector3d::Zero());
 }
 for(auto& sh:shells)sh={0,-1};auto empty=contractRawRangeBox(lo,hi,b,shells,point);require(!empty.valid && empty.status=="MODEL_INCONSISTENT" && std::isinf(empty.hpl),"empty is not zero PL");
 for(auto& sh:shells)sh={0,1e308};auto overflow=contractRawRangeBox(lo,hi,b,shells,point);require(overflow.valid && overflow.status=="FALLBACK_INERTIAL_VERIFIED" && overflow.low==lo,"overflow safe fallback");
 std::cout<<"geometry oracle feasible_lattice="<<checked<<" fault/boundary/mirror/analytic/frontier/overflow passed\n";
}
int main(){try{
 geometryChecks();
 auto masks=rawRangeFaultMasks();require(masks.size()==9 && masks.front()==-1 && masks.back()==7,"complete masks");
 for(auto kind:{"U0","U1","U2","U3"}){NoisyMotionScenario s("config/hybrid_raw.yaml",kind,20261010,16);
  for(size_t e=0;e<s.uwb.size();++e){std::array<RawRangeShell,8> shell;require(rawRangeShells(s.range_certificates[e],s.uwb[e],e+1,shell),"source numeric certificate/shell");
   auto changed=s.uwb[e];changed.measurements[0].range_m+=1e-8;require(!rawRangeShells(s.range_certificates[e],changed,e+1,shell),"changed raw range refused");
  }
 }
 std::cout<<"raw range source/shell checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
