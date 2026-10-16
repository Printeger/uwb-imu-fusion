#pragma once
#include "uwb_imu_pl/common/types.hpp"
#include <map>
#include <set>
#include <stdexcept>

namespace uwb_imu_pl::research {

// Calibration revisions never erase physical identity or earlier coordinates.
// Resource limits are explicit runtime admission limits, not topology masks.
class AnchorRegistry {
 struct Entry {
  uint64_t current=0;
  bool online=true;
  std::map<uint64_t, AnchorRecord> versions;
 };
 std::map<AnchorId, Entry> anchors_;
 size_t max_anchors_,max_versions_,versions_=0;
 public:
 explicit AnchorRegistry(size_t max_anchors=4096,size_t max_versions=16384)
  :max_anchors_(max_anchors),max_versions_(max_versions){
  if(!max_anchors || !max_versions)throw std::invalid_argument("ANCHOR_CAPACITY_ZERO");
 }
 void declare(const AnchorRecord& anchor,uint64_t version){
  if(!anchor.position_world_m.allFinite() || !anchor.covariance_m2.allFinite() ||
     anchor.frame.empty() || anchor.map_version.empty())
   throw std::invalid_argument("ANCHOR_CALIBRATION_INVALID");
  auto found=anchors_.find(anchor.id);
  if(found!=anchors_.end()){
   auto old=found->second.versions.find(version);
   if(old!=found->second.versions.end()){
    const auto& a=old->second;
    if(a.position_world_m!=anchor.position_world_m || a.covariance_m2!=anchor.covariance_m2 ||
       a.frame!=anchor.frame || a.map_version!=anchor.map_version)
     throw std::invalid_argument("ANCHOR_VERSION_CONFLICT");
    return; // Idempotent; does not reconnect, reset history, or roll back current version.
   }
   if(version<=found->second.current)throw std::invalid_argument("ANCHOR_VERSION_REGRESSION");
  }else if(anchors_.size()>=max_anchors_)throw std::length_error("ANCHOR_REGISTRY_CAPACITY");
  if(versions_>=max_versions_)throw std::length_error("ANCHOR_VERSION_CAPACITY");
  auto& entry=anchors_[anchor.id];entry.versions.emplace(version,anchor);entry.current=version;++versions_;
 }
 void online(AnchorId id,bool present){anchors_.at(id).online=present;}
 size_t registered()const{return anchors_.size();}
 const AnchorRecord& at(AnchorId id,uint64_t version)const{return anchors_.at(id).versions.at(version);}
 void validate(const UwbMeasurement& m,uint64_t version)const{
  const auto& entry=anchors_.at(m.anchor_id);
  if(!entry.online)throw std::invalid_argument("ANCHOR_OFFLINE");
  const auto& a=entry.versions.at(version);
  if(version!=entry.current || m.anchor_position_m!=a.position_world_m)
   throw std::invalid_argument("ANCHOR_CALIBRATION_MISMATCH");
 }
};

struct CanonicalRangeEvent {
 UwbMeasurement measurement;
 TimestampNs received;
 uint64_t calibration_version=0;
 bool valid=false;
};
struct RangeCounts {size_t registered=0,visible=0,measurements=0,invalid=0;};
// Selection preserves original IDs and times. Invalid samples are still in the
// source layout supplied to the event accountant; this helper never assigns q.
inline RangeCounts validateRanges(const std::vector<CanonicalRangeEvent>& events,
                                 const AnchorRegistry& registry,TimestampNs ready){
 RangeCounts count;count.registered=registry.registered();
 std::set<MeasurementId> ids;std::set<FactorId> factors;std::set<AnchorId> visible;
 for(const auto& e:events){
  const auto& m=e.measurement;
  if(!ids.insert(m.id).second || !factors.insert(m.factor_id).second)
   throw std::invalid_argument("DUPLICATE_RANGE_ID");
  if(ready<m.timestamp || e.received<m.timestamp || ready<e.received)
   throw std::invalid_argument("FUTURE_RANGE_SOURCE");
  registry.validate(m,e.calibration_version);
  if(!e.valid){++count.invalid;continue;}
  if(!std::isfinite(m.range_m) || m.range_m<=0 || !std::isfinite(m.sigma_m) || m.sigma_m<=0)
   throw std::invalid_argument("RANGE_VALUE_INVALID");
  ++count.measurements;visible.insert(m.anchor_id);
 }
 count.visible=visible.size();return count;
}
}
