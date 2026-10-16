#include "source_layout_event.hpp"
#include <iostream>
using namespace uwb_imu_pl::research;
void require(bool v,const char* m){if(!v)throw std::runtime_error(m);}
template<class F>void refuses(F f){bool caught=false;try{f();}catch(const std::exception&){caught=true;}require(caught,"expected refusal");}
int main(){try{
 const double eps=1e-6;SourceLayoutEvent event(eps);TimeUniformEvent old(eps);
 require(event.id()!=old.id(),"V1/V2 confused");
 for(uint64_t k:{0ULL,1ULL,2ULL,10000ULL,1000000000ULL}){
  auto v1=old.block(k),v2=SourceLayoutEvent::allocation(eps,k,k?128:21);
  require(v1.q==v2.q && v1.tail_upper==v2.tail_upper,"old layout q changed");
 }
 long double risk=0;size_t count=0;
 for(uint64_t k=0;k<10000;++k){
  size_t n=k%131+1;std::vector<SourceCoordinate> layout;
  for(size_t j=0;j<n;++j)layout.push_back({SourceKind::Range,UINT64_MAX-j,k,0});
  auto b=event.append(layout);count+=n;
  // Independent long-double normal-tail oracle, not the binary proof formula.
  long double tail=n*std::erfc((long double)b.q/std::sqrt((long double)2));
  const long double delta=k?(long double)eps/(2*(long double)k*(k+1)):(long double)eps/2;
  require(tail<=(long double)b.tail_upper && (long double)b.tail_upper<=delta,"tail allocation underbound");
  risk+=b.tail_upper;require(risk<=event.prefixUpper() && event.prefixUpper()<=eps,"budget reset/overflow");
  auto birth=event.birth(layout.front());const auto blocks=event.blocks();
  auto reuse=event.append(layout);require(reuse.sources==0 && event.blocks()==blocks && event.birth(layout.front()).q==birth.q,"reused sources respent");
  event.append({});require(event.blocks()==blocks && event.coordinates()==count,"zero-new block mutation");
 }
 SourceCoordinate startup{SourceKind::Adc,42,999,2};SourceLayoutEvent init(eps,2);
 auto birth=init.append({startup});init.append({startup});require(init.birth(startup).q==birth.q && init.coordinates()==1,"bootstrap reuse");
 refuses([&]{init.append({startup,startup});});require(init.coordinates()==1,"failed append mutated registry");
 refuses([&]{init.append({{SourceKind::Range,0,1,1}});});
 refuses([&]{init.append({{SourceKind::Range,0,1,0},{SourceKind::Range,0,2,0}});});require(init.blocks()==1,"capacity reset risk");
 refuses([&]{SourceLayoutEvent::allocation(eps,UINT64_MAX,1);});
 refuses([&]{SourceLayoutEvent::allocation(eps,1,UINT64_MAX);});
 require(!SourceLayoutEvent::tailQualified(ModelEvidenceStatus::DeclaredEmpirical,true,true),"empirical became qualified");
 require(!SourceLayoutEvent::tailQualified(ModelEvidenceStatus::VerifiedForDeclaredModel,false,true),"measurement-selected layout qualified");
 require(!SourceLayoutEvent::tailQualified(ModelEvidenceStatus::VerifiedForDeclaredModel,true,false),"missing tail qualified");
 std::cout<<"SC9 source-layout event PASS 10000 varying blocks, independent tail oracle, shared/bootstrap reuse, zero-new, capacity/overflow, qualification split\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
