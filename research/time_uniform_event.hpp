#pragma once
#include <boost/numeric/interval.hpp>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <sstream>
namespace uwb_imu_pl::research {
// One infinite allocation, not successive finite missions. delta0=eps/2,
// deltak=eps/(2 k(k+1)); each physical sample keeps its birth allocation.
struct TimeUniformBlock {
 uint64_t index=0,sources=0;int bits=0;double q=0,delta_low=0,delta_high=0,tail_upper=0;
};
class TimeUniformEvent {
 double epsilon_;
 public:
 explicit TimeUniformEvent(double epsilon):epsilon_(epsilon){
  if(!std::isfinite(epsilon) || epsilon<=0 || epsilon>=1)throw std::invalid_argument("TIME_UNIFORM_BUDGET_INVALID");
 }
 double budget()const{return epsilon_;}
 std::string id()const{std::ostringstream s;s<<std::hexfloat<<epsilon_;return "time-uniform-block-rw-repeat-v1:"+s.str();}
 TimeUniformBlock block(uint64_t k)const{
  // Hardware representation limit is a fail-closed operational limit, not a
  // probabilistic horizon. Arithmetic below uses exactly represented integers.
  if(k>uint64_t(std::numeric_limits<int64_t>::max()/50000000LL)-1)throw std::overflow_error("GLOBAL_CLOCK_UNREPRESENTABLE");
  using I=boost::numeric::interval<double>;
  I delta=k?I(epsilon_)/(2.*I(double(k))*I(double(k+1))):I(epsilon_)/2.;
  TimeUniformBlock b{k,k?128ULL:21ULL,0,0,delta.lower(),delta.upper(),0};
  for(b.bits=1;b.bits<1023;++b.bits)if(std::ldexp(double(2*b.sources),-b.bits)<=b.delta_low)break;
  if(b.bits==1023)throw std::overflow_error("SOURCE_ALLOCATION_UNREPRESENTABLE");
  b.q=1;while(5*b.q*b.q<7*b.bits)b.q+=1;
  b.tail_upper=std::ldexp(double(2*b.sources),-b.bits);return b;
 }
 double prefixUpper(uint64_t k)const{
  block(k);using I=boost::numeric::interval<double>;
  // Analytical telescope is also a certificate: use the smaller of two
  // valid upper bounds. This caps accounting, never the increasing q.
  return std::min(epsilon_,(I(epsilon_)*(1.-I(1)/(2.*I(double(k+1))))).upper());
 }
 uint64_t adcBirth(uint64_t sample)const{return sample?1+(sample-1)/10:0;}
 double adcRadius(uint64_t sample)const{return block(adcBirth(sample)).q;}
 std::string adcId(uint64_t sample,unsigned axis)const{
  if(axis>=6)throw std::invalid_argument("ADC_AXIS_INVALID");block(adcBirth(sample));
  return "adc:"+std::to_string(sample*5000000ULL)+":"+std::to_string(axis);
 }
 uint64_t prefixSources(uint64_t k)const{block(k);return 21+128*k;}
};
}
