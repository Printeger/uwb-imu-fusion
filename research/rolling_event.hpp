#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <cstdint>
namespace uwb_imu_pl::research {
// Compact deterministic registry: initial15 + ADC6(10T+1) + RW60T + range8T.
// No dense source directory, and no event refresh at a window boundary.
struct RollingSourceEvent {
 int mission=0,tail_bits=0;uint64_t coordinates=0,logical_faults=0;
 double q=0,risk_upper=0,budget=0;
 std::string id;
 explicit RollingSourceEvent(int t,double eps):mission(t),budget(eps){
  if(t<1 || t>10000 || !std::isfinite(eps) || eps<=0 || eps>=1)throw std::invalid_argument("FINITE_SOURCE_EVENT_REFUSED");
  coordinates=21+128ULL*t;logical_faults=1+28ULL*t+192ULL*t*t;
  // Exact binary upper bound; ln2<7/10 gives 2N exp(-q²/2)<2N 2^-m.
  for(tail_bits=1;tail_bits<1023;++tail_bits)if(std::ldexp(double(2*coordinates),-tail_bits)<=eps)break;
  if(tail_bits==1023)throw std::invalid_argument("TAIL_BUDGET_UNREPRESENTABLE");
  // Integer radius avoids relying on libm log/exp/sqrt to certify a boundary.
  q=1;while(5*q*q<7*tail_bits)q+=1;
  risk_upper=std::ldexp(double(2*coordinates),-tail_bits);
  id="sc5-source-rw-T"+std::to_string(t)+"-m"+std::to_string(tail_bits)+"-q"+std::to_string(int(q));
 }
 uint64_t prefix(size_t epoch)const{
  if(epoch>size_t(mission))throw std::invalid_argument("TASK_HORIZON_EXPIRED");
  return 21+128ULL*epoch;
 }
};
}
