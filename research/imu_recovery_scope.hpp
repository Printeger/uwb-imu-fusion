#pragma once
#include "raw_imu_episode.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
namespace uwb_imu_pl::research {
// One physical raw episode across all affected factor groups, not independent
// per-factor interval faults. Only accel-x is declared in this diagnostic law.
inline GeneratedFaultModelSet imuEpisodeModels(const GeneratedFaultModelSet& templates,
    const LinearizedIntegrityWindow& w,const EpochTransaction& tx,const std::vector<int>& lengths) {
 GeneratedFaultModelSet out;
 std::map<FactorGroupId,EpochTransaction> materials;materials.emplace(tx.imu_group.id,tx);
 for(const auto& h:tx.recoverable_history)for(const auto& g:h.groups)if(g.kind==FactorKind::CombinedImu){
  auto t=historicalEpisodeTransaction(h);t.frozen_values=tx.frozen_values;materials.emplace(g.id,std::move(t));}
 for(const auto& seed:templates.modes)if(seed.sensor==SensorType::ImuAccelerometer && seed.axis==0)for(int length:lengths){
  auto m=seed;m.id=FaultModeId(out.modes.size()+1);m.raw_group_maps.clear();m.affected_groups.clear();m.affected_measurements.clear();
  m.physical_source_id+="|raw-accel-x-"+std::to_string(length)+"ms";
  RawImuEpisode e{m.physical_source_id,TimestampNs(m.onset_time.value()+5000000),TimestampNs(m.onset_time.value()+5000000+length*1000000LL),0};
  for(const auto& b:w.blocks)if(b.kind==FactorKind::CombinedImu){auto it=materials.find(b.group_id);if(it==materials.end())throw std::runtime_error("missing episode lineage");
   const auto& t=it->second;auto r=episodeResponse(t,b,e,1e-5,true);
   if(r.affected_trapezoids){m.raw_group_maps.emplace(b.group_id,r.raw);m.affected_groups.push_back(b.group_id);
    for(const auto& sample:t.raw_imu_slice)if(!(sample.timestamp<e.begin)&&sample.timestamp<e.end && std::find(m.affected_measurements.begin(),m.affected_measurements.end(),sample.id)==m.affected_measurements.end())m.affected_measurements.push_back(sample.id);}
  }
  // A condensed interval map cannot replace a raw sample episode map. Preserve
  // failure rather than quietly dropping old contamination at window exit.
  if(m.onset_epoch<w.detector_first_epoch)m.recoverability=HistoryRecoverability::MissingProvenance;
  FaultUnit u;u.id=FaultUnitId(m.id.value());u.sensor=m.sensor;u.kind=m.kind;u.axis=0;u.physical_source_id=m.physical_source_id;
  u.epoch_begin=m.onset_epoch;u.epoch_end=m.onset_epoch+length/50+1;u.time_begin=e.begin;u.time_end=e.end;u.parameter_dimension=1;
  u.affected_groups=m.affected_groups;u.affected_measurements=m.affected_measurements;
  FaultHypothesisV2 h;h.id=HypothesisId(out.hypotheses.size()+1);h.units={u.id};h.modes={m.id};h.prior_probability_bound=m.prior_probability_bound;h.p_md_allocation=.001;h.hmi_allocation=1e-9;
  out.units.push_back(std::move(u));out.modes.push_back(std::move(m));out.hypotheses.push_back(std::move(h));
 }
 return out;
}
inline bool episodeCoverageComplete(const FaultModeBasis& m,const ExclusionAction& a){
 for(auto id:m.affected_groups)if(std::find(a.groups_to_remove.begin(),a.groups_to_remove.end(),id)==a.groups_to_remove.end())return false;
 return m.recoverability==HistoryRecoverability::Recoverable;
}
// Fault-independent kinematic domain; velocity/attitude uncertainty and gravity
// coupling still require a propagated joint bridge certificate, not this test.
inline bool bridgeMotionDomain(double acceleration_bound,double angular_rate,
                               double acceleration_limit,double rate_limit){
 return acceleration_bound<=acceleration_limit && std::abs(angular_rate)<=rate_limit;
}
}
