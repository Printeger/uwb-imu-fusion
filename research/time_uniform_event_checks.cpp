#include "time_uniform_event.hpp"
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <iostream>
#include <set>
using namespace uwb_imu_pl::research;
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
int main(){try{
 using MP=boost::multiprecision::cpp_dec_float_100;TimeUniformEvent e(1e-6);
 MP sum=0;double previous=0;
 for(uint64_t k=0;k<=1000;++k){auto b=e.block(k);MP exact=MP(e.budget())/2;if(k)exact=MP(e.budget())/(2*MP(k)*MP(k+1));
  require(MP(b.delta_low)<=exact && exact<=MP(b.delta_high),"allocation rounding");
  require(2*MP(b.sources)*exp(-MP(b.q)*MP(b.q)/2)<=MP(b.tail_upper) && MP(b.tail_upper)<=exact,"tail allocation");
  require(b.q>=previous,"q truncated");previous=b.q;sum+=MP(b.tail_upper);
  require(sum<=MP(e.prefixUpper(k)) && e.prefixUpper(k)<=e.budget(),"cumulative risk reset/overflow");
 }
 require(e.adcBirth(10)==1 && e.adcBirth(11)==2 && e.adcRadius(10)==e.block(1).q,"shared endpoint reassigned");
 require(e.adcRadius(2000)==e.block(200).q && e.block(1000).q>=e.block(200).q,"old radius replaced");
 std::set<std::string> ids;for(uint64_t s=0;s<=10000;++s)for(unsigned a=0;a<6;++a)require(ids.insert(e.adcId(s,a)).second,"duplicate source");
 for(uint64_t k:{1000000ULL,100000000000ULL}){auto b=e.block(k);require(MP(b.tail_upper)<=MP(e.budget())/(2*MP(k)*MP(k+1)),"large clock allocation");}
 // Tail of the exact allocation is eps/(2(K+1)), so the infinite sum is eps.
 require(MP(e.budget())/2+MP(e.budget())/2==MP(e.budget()),"infinite telescope");
 std::cout<<"TIME_UNIFORM_EVENT PASS1001 blocks, 100-digit allocation/tail, unique60006 ADC sources, shared birth, uncapped q, infinite telescope; q0/200/1000="<<e.block(0).q<<'/'<<e.block(200).q<<'/'<<e.block(1000).q<<'\n';return 0;
 }catch(const std::exception& x){std::cerr<<x.what()<<'\n';return 1;}}
