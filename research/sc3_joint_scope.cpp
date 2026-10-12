#include "sc3_joint_scope.hpp"
#include <set>
#include <stdexcept>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
namespace uwb_imu_pl::research {
Sc3PhysicalHypothesis Sc3FaultPool::at(size_t index)const{
 if(index>=represented)throw std::out_of_range("SC3 physical hypothesis index");
 Sc3PhysicalHypothesis h;h.hypothesis.id=HypothesisId(index+1);
 if(index==0){h.identity="NONE";return h;}
 --index;
 if(index<uwb.size())h.uwb=int(index);
 else if((index-=uwb.size())<imu.size())h.imu=int(index);
 else{index-=imu.size();h.uwb=int(index/imu.size());h.imu=int(index%imu.size());}
 auto append=[&](const Sc3PhysicalEpisode& e){h.hypothesis.units.push_back(e.unit.id);h.hypothesis.modes.push_back(e.mode.id);if(!h.identity.empty())h.identity+='|';h.identity+=e.identity;};
 if(h.uwb>=0)append(uwb.at(h.uwb));if(h.imu>=0)append(imu.at(h.imu));
 // No prior/MD/HMI numbers: this pool uses sup-fault conditional source risk.
 return h;
}
bool Sc3FaultPool::complete()const{
 if(scope!="sc3-one-uwb-one-imu-episode-rw-v1" || uwb.size()!=800 || imu.size()!=600 || expected!=481401 || represented!=expected || evaluated>expected)return false;
 std::set<std::string> identities;std::set<uint64_t> ids;
 for(const auto* group:{&uwb,&imu})for(const auto& e:*group)if(!identities.insert(e.identity).second || !ids.insert(e.mode.id.value()).second || e.unit.id.value()!=e.mode.id.value() || e.mode.parameter_dimension!=1 || e.unit.parameter_dimension!=1 || e.identity.empty() || e.unit.physical_source_id!=e.mode.physical_source_id || e.unit.axis!=e.mode.axis || e.unit.time_begin!=e.mode.onset_time || !(e.unit.time_begin<e.unit.time_end))return false;
 return represented==1+uwb.size()+imu.size()+uwb.size()*imu.size();
}
Sc3PhysicalEpisode bindSc3PhysicalEpisode(const Sc3PhysicalEpisode& physical,const EpochTransaction& tx){
 auto bound=physical;auto& mode=bound.mode;auto& unit=bound.unit;
 if(unit.sensor==SensorType::Uwb){auto add=[&](const UwbBatch& batch,const std::vector<PendingFactorGroup>& groups,const std::vector<FactorGroupId>* selected){if(batch.timestamp<unit.time_begin || !(batch.timestamp<unit.time_end))return;
  for(const auto& measurement:batch.measurements)if(measurement.anchor_id==mode.anchor_id){unit.affected_measurements.push_back(measurement.id);for(const auto& g:groups)if(g.kind==FactorKind::UwbBatch && (selected?std::find(selected->begin(),selected->end(),g.id)!=selected->end():g.nominal)){Eigen::VectorXd map=Eigen::VectorXd::Zero(g.source_measurements.size());for(size_t i=0;i<g.source_measurements.size();++i)if(g.source_measurements[i]==measurement.id)map(i)=1.;if(map.squaredNorm()>0){unit.affected_groups.push_back(g.id);mode.raw_group_maps[g.id]=map;}}}
 };add(tx.uwb_batch,tx.uwb_groups,nullptr);for(const auto& h:tx.recoverable_history)add(h.uwb_batch,h.groups,&h.selected_groups);
 }else{
  RawImuEpisode episode{bound.identity,unit.time_begin,unit.time_end,unit.axis};auto closure=sc2SupportClosure(episode,int(tx.proposed_epoch));unit.affected_measurements=closure.raw_samples;
  auto add=[&](const EpochTransaction& material){if(!material.preintegration)return;bool direct=false;for(const auto& sample:material.raw_imu_slice)direct=direct || (!(sample.timestamp<episode.begin) && sample.timestamp<episode.end);
   if(material.imu_group.id.value()==0){if(direct)mode.recoverability=HistoryRecoverability::MissingProvenance;return;}
   if(direct){LinearizedFactorBlock block;block.whitener=gtsam::noiseModel::Gaussian::Covariance(material.preintegration->preintMeasCov())->R();mode.raw_group_maps[material.imu_group.id]=episodeResponse(material,block,episode).raw;}
   if(closure.first_epoch && material.proposed_epoch>=closure.first_epoch)unit.affected_groups.push_back(material.imu_group.id);
  };add(tx);for(const auto& h:tx.recoverable_history){auto material=historicalEpisodeTransaction(h);for(const auto& group:h.groups)if(group.kind==FactorKind::CombinedImu){material.imu_group=group;break;}material.frozen_values=tx.frozen_values;add(material);}
  if(!closure.valid || (closure.first_epoch && !tx.frozen_values->exists(gtsam::Symbol('x',0))))mode.recoverability=HistoryRecoverability::MissingProvenance;
 }
 mode.affected_measurements=unit.affected_measurements;mode.affected_groups=unit.affected_groups;return bound;
}
Sc3FaultPool makeSc3FaultPool(const UwbBatch& batch,size_t capacity){
 HypothesisGeneratorConfig config;config.double_faults_enabled=true;
 return HypothesisGenerator(config).generateRawJointResearch(batch,capacity);
}
}
namespace uwb_imu_pl {
research::Sc3FaultPool HypothesisGenerator::generateRawJointResearch(const UwbBatch& batch,size_t capacity)const{
 if(!config_.include_uwb_faults || !config_.include_imu_faults || !config_.single_faults_enabled || !config_.double_faults_enabled || !config_.include_uwb_accel_combinations || !config_.include_uwb_gyro_combinations || config_.max_model_cardinality<2 || batch.measurements.size()!=8 || capacity<1400)throw std::runtime_error("SC3 whole requested pool refused");
 research::Sc3FaultPool pool;std::set<uint64_t> anchors;uint64_t id=0;
 auto build=[&](SensorType sensor,FaultKind kind,std::string source,int axis,int onset,int duration){research::Sc3PhysicalEpisode e;e.duration=duration;
  auto& u=e.unit;auto& m=e.mode;u.id=FaultUnitId(++id);m.id=FaultModeId(id);u.sensor=m.sensor=sensor;u.kind=m.kind=kind;u.physical_source_id=m.physical_source_id=source;u.axis=m.axis=axis;u.parameter_dimension=m.parameter_dimension=1;m.effective_parameter_dimension=1;u.epoch_begin=m.onset_epoch=onset;
  bool uwb=sensor==SensorType::Uwb;u.epoch_end=uwb?onset+duration-1:onset+(duration/50);
  u.time_begin=m.onset_time=TimestampNs(uwb?onset*50000000LL:(onset-1)*50000000LL+5000000LL);u.time_end=TimestampNs(u.time_begin.value()+(uwb?duration*50000000LL:duration*1000000LL));u.persistent=duration>(uwb?1:50);
  e.identity=source+":axis="+std::to_string(axis)+":begin="+std::to_string(u.time_begin.value())+":end="+std::to_string(u.time_end.value())+":constant:unbounded";return e;};
 for(const auto& row:batch.measurements){if(!anchors.insert(row.anchor_id.value()).second)throw std::runtime_error("SC3 duplicate anchor identity");
  for(int onset=1;onset<=50;++onset)for(int duration:{1,3}){auto e=build(SensorType::Uwb,FaultKind::AnchorBiasPersistentConstant,"uwb:"+std::to_string(row.anchor_id.value()),-1,onset,duration);e.mode.anchor_id=row.anchor_id;pool.uwb.push_back(e);}}
 for(int axis=0;axis<6;++axis)for(int onset=1;onset<=50;++onset)for(int duration:{50,500})pool.imu.push_back(build(axis<3?SensorType::ImuAccelerometer:SensorType::ImuGyroscope,axis<3?FaultKind::AccelAxisIntervalConstant:FaultKind::GyroAxisIntervalConstant,"imu:axis:"+std::to_string(axis),axis,onset,duration));
 // Generation does not certify consumption. W2 must bind a valid all9-mask
 // contraction and physical envelope before counting represented branches.
 pool.expected=pool.represented=1+800+600+800*600;pool.evaluated=0;
 if(!pool.complete())throw std::runtime_error("SC3 physical census incomplete");
 for(const auto& u:pool.uwb)for(const auto& i:pool.imu)if(pairFamilySupport(u.mode,i.mode)!=PairFamilySupport::Independent)throw std::runtime_error("SC3 shared/unsupported parameter concatenation");
 return pool;
}
}
