#include "rolling_event.hpp"
#include "raw_source_law.hpp"
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <iostream>
#include <set>
using namespace uwb_imu_pl::research;
void require(bool b,const char* why){if(!b)throw std::runtime_error(why);}
int main(){try{
 using MP=boost::multiprecision::cpp_dec_float_100;
 for(int t:{1,2,7,49,50,51,100,200,201,777,10000}){
  RollingSourceEvent e(t,1e-6);std::set<std::string> ids;
  const uint64_t explicit_count=t<=201?e.coordinates:256;
  for(uint64_t i=0;i<explicit_count;++i)require(ids.insert(e.sourceId(i)).second,"duplicate shared source");
  require(e.coordinates==uint64_t(21+128*t) && ids.size()==explicit_count,"source formula");
  require(e.sourceId(e.coordinates-1)=="uwb:"+std::to_string(t)+":7","compact registry last source");
  require(e.logical_faults==1+28ULL*t+192ULL*t*t,"fault recipe census");
  MP tail=2*MP(e.coordinates)*exp(-MP(e.q)*MP(e.q)/2);
  require(tail<=MP(e.risk_upper) && MP(e.risk_upper)<=MP(e.budget),"100-digit tail/budget");
  require(e.prefix(t)==e.coordinates && e.prefix(0)==21,"prefix census");
  bool refused=false;try{e.prefix(t+1);}catch(const std::exception&){refused=true;}require(refused,"expired event renewed");
  std::cout<<"EVENT T="<<t<<" N="<<e.coordinates<<" q="<<e.q<<" binary_upper="<<e.risk_upper<<" exact_chernoff="<<tail<<'\n';
 }
 RollingSourceEvent legacy(50,1e-6);Eigen::Matrix<double,15,1> sigmas=Eigen::Matrix<double,15,1>::Constant(.1);
 auto inventory=rawSourceInventory(50,8,sigmas,.02,.001,.001,.0001,.1);
 require(inventory.size()==legacy.coordinates,"legacy inventory changed");
 for(size_t i=0;i<inventory.size();++i)require(inventory[i].id==legacy.sourceId(i),"legacy unique source recipe changed");
 // A finite rational Taylor partial sum establishes exp(.7)>2, so ln2<.7
 // without treating ordinary floating logarithms as certified constants.
 MP sum=1,term=1;for(int i=1;i<=5;++i){term*=MP(7)/(10*i);sum+=term;}require(sum>2,"ln2 rational certificate");
 for(int t:{0,-1,10001}){bool refused=false;try{RollingSourceEvent e(t,1e-6);}catch(const std::exception&){refused=true;}require(refused,"invalid task admitted");}
 std::cout<<"W5 source/event PASS11 task sizes, compact unique shared ADC/RW/initial/range recipe, legacy6421 inventory, 100-digit tail, T+1 refusal\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
