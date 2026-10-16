#pragma once
#include "time_uniform_event.hpp"
#include <map>
#include <set>
#include <tuple>
#include <vector>

namespace uwb_imu_pl::research {
enum class ModelEvidenceStatus { VerifiedForDeclaredModel, DeclaredEmpirical, MissingOrInconsistent };
enum class SourceKind { Initial, Adc, BiasIncrement, Range };
// The raw physical identity survives interpolation, initialization, retries,
// dropout, and history replay. A derived sample is never a new coordinate.
struct SourceCoordinate {
 SourceKind kind=SourceKind::Adc;
 uint64_t sensor=0,measurement=0;
 unsigned axis=0;
 bool operator<(const SourceCoordinate& b)const noexcept{
  return std::tie(kind,sensor,measurement,axis)<std::tie(b.kind,b.sensor,b.measurement,b.axis);
 }
};
struct SourceBirth {uint64_t block=0;double q=0;};

// V2 has no timestamp/sample-count arithmetic. Its deterministic allocator
// alone is NOT evidence of a Gaussian sensor law. The caller must separately
// bind tail qualification and exogenous/potential-source layout evidence.
// All coordinate marginal tails suffice; no cross-coordinate independence
// is required by the union bound. Unknown selection never gains qualification.
class SourceLayoutEvent {
 double epsilon_;
 uint64_t next_block_=0;
 size_t capacity_;
 std::map<SourceCoordinate,SourceBirth> births_;
 public:
 explicit SourceLayoutEvent(double epsilon,size_t capacity=1000000)
  :epsilon_(epsilon),capacity_(capacity){TimeUniformEvent check(epsilon);if(!capacity)throw std::invalid_argument("SOURCE_CAPACITY_ZERO");}
 std::string id()const{std::ostringstream s;s<<std::hexfloat<<epsilon_;return "time-uniform-source-layout-v2:"+s.str();}
 static TimeUniformBlock allocation(double epsilon,uint64_t k,uint64_t dimensions){
  TimeUniformEvent check(epsilon);
  // Exact-double integer arithmetic limit is independent of physical time.
  // Refuse before conversion; never wrap k+1 or 2*d.
  if(k>=(uint64_t(1)<<52) || dimensions>(uint64_t(1)<<51))throw std::overflow_error("LAYOUT_INDEX_CAPACITY");
  using I=boost::numeric::interval<double>;
  I delta=k?I(epsilon)/(2.*I(double(k))*I(double(k+1))):I(epsilon)/2.;
  TimeUniformBlock b{k,dimensions,0,0,delta.lower(),delta.upper(),0};
  if(!dimensions)return b;
  for(b.bits=1;b.bits<1023;++b.bits)if(std::ldexp(double(2*dimensions),-b.bits)<=b.delta_low)break;
  if(b.bits==1023)throw std::overflow_error("LAYOUT_TAIL_UNREPRESENTABLE");
  // P(|Z|>q)<=2 exp(-q²/2)<2*2^-bits since ln(2)<7/10.
  // Integer q avoids relying on libm quantiles for certified rounding.
  b.q=1;while(5*b.q*b.q<7*b.bits)++b.q;
  b.tail_upper=std::ldexp(double(2*dimensions),-b.bits);return b;
 }
 TimeUniformBlock append(const std::vector<SourceCoordinate>& layout){
  std::set<SourceCoordinate> new_sources,seen;
  for(const auto& c:layout){
   if(c.axis>=(c.kind==SourceKind::Initial?15u:c.kind==SourceKind::Range?1u:6u))
    throw std::invalid_argument("SOURCE_COORDINATE_AXIS");
   if(!seen.insert(c).second)throw std::invalid_argument("DUPLICATE_LAYOUT_COORDINATE");
   if(!births_.count(c))new_sources.insert(c);
  }
  if(new_sources.size()>capacity_-births_.size())throw std::length_error("SOURCE_LEDGER_CAPACITY");
  const auto block=allocation(epsilon_,next_block_,new_sources.size());
  if(new_sources.empty())return block; // No birth and no new spending.
  // Stage allocations first: bad_alloc must not partially spend a birth block.
  std::map<SourceCoordinate,SourceBirth> staged;
  for(const auto& c:new_sources)staged.emplace(c,SourceBirth{next_block_,block.q});
  births_.merge(staged);
  ++next_block_;return block;
 }
 const SourceBirth& birth(const SourceCoordinate& c)const{return births_.at(c);}
 uint64_t blocks()const{return next_block_;}
 size_t coordinates()const{return births_.size();}
 double prefixUpper()const{
  if(!next_block_)return 0;
  using I=boost::numeric::interval<double>;
  return std::min(epsilon_,(I(epsilon_)*(1.-I(1)/(2.*I(double(next_block_))))).upper());
 }
 static bool tailQualified(ModelEvidenceStatus status,bool source_layout_predetermined,bool marginal_tail_verified){
  return status==ModelEvidenceStatus::VerifiedForDeclaredModel && source_layout_predetermined && marginal_tail_verified;
 }
};
}
