#pragma once
#include "gaussian_window.hpp"
#include "statistical_history.hpp"
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Ordering.h>
namespace uwb_imu_pl::research {
// Finite research reference, distinct from the localization marginal prior.
// Retain only raw source factors/materials and ONE frozen state chart per epoch.
// A chart freezes when it leaves the declared explicit window; old history is
// never silently relinearized at the current backend trajectory. Both dense and
// compressed oracles below consume exactly these same points and raw sources.
struct RawPrefixHistory {
 std::map<std::uint64_t,PendingFactorGroup> groups;
 std::map<std::size_t,HistoricalEpochContext> episodes;
 gtsam::Values points;
 bool initialized=false;
 void observe(const EpochTransaction& tx,size_t window){
  if(tx.proposed_epoch>50)throw std::runtime_error("raw prefix beyond research horizon");
  size_t first=tx.proposed_epoch>window?tx.proposed_epoch-window:0;
  for(const auto& item:*tx.frozen_values){auto key=item.key;
   if(!points.exists(key))points.insert(key,item.value);
   else if(gtsam::Symbol(key).index()>=first)points.update(key,item.value);
  }
  if(!initialized){for(const auto& f:tx.frozen_slots){if(!f.factor)continue;
   if(dynamic_cast<const gtsam::CombinedImuFactor*>(f.factor.get()))throw std::runtime_error("raw prefix must start at epoch1");
   auto id=f.group_id.value_or(FactorGroupId(f.slot));auto& g=groups[id.value()];g.id=id;g.kind=FactorKind::BoundaryPrior;g.sensor=SensorType::Prior;g.factors.push_back(f.factor);
   for(auto key:f.factor->keys())if(std::find(g.keys.begin(),g.keys.end(),key)==g.keys.end())g.keys.push_back(key);
  }initialized=true;}
  HistoricalEpochContext h;h.previous_epoch=tx.previous_epoch;h.proposed_epoch=tx.proposed_epoch;h.begin=tx.begin;h.end=tx.end;
  h.raw_imu_slice=tx.raw_imu_slice;h.preintegration=tx.preintegration;h.groups={tx.imu_group};h.uwb_batch=tx.uwb_batch;
  episodes[tx.proposed_epoch]=std::move(h);
 }
 void committed(const EpochTransaction& tx,const EpochCommitPlan& plan){
  for(auto id:plan.groups_to_remove)groups.erase(id.value());
  std::vector<const PendingFactorGroup*> catalog{&tx.imu_group,&tx.generic_bridge_group,&tx.generic_bias_continuity_group};
  for(const auto& g:tx.uwb_groups)catalog.push_back(&g);
  for(const auto& h:tx.recoverable_history)for(const auto& g:h.groups)catalog.push_back(&g);
  for(auto id:plan.groups_to_add){auto it=std::find_if(catalog.begin(),catalog.end(),[&](const auto* g){return g->id==id;});
   if(it==catalog.end())throw std::runtime_error("prefix commit source missing");
   groups[id.value()]=**it;
  }
 }
 LinearizedIntegrityWindow window(const EpochTransaction& tx)const{
  LinearizedIntegrityWindow w;w.id=WindowId(tx.id.value());w.version=tx.base_version;int columns=15*(tx.proposed_epoch+1);
  auto material=groups;material[tx.imu_group.id.value()]=tx.imu_group;for(const auto& g:tx.uwb_groups)if(g.nominal)material[g.id.value()]=g;
  for(const auto& kv:material){const auto& g=kv.second;
   if(g.kind==FactorKind::KinematicBridge || g.kind==FactorKind::BiasContinuity)
    throw std::runtime_error("raw prefix Gaussian law does not include bridge metrics");
   gtsam::Ordering order;for(auto key:g.keys)order.push_back(key);
   auto linear=g.factors.linearize(points);auto dense=linear->jacobian(order);
   LinearizedFactorBlock b;b.group_id=g.id;b.kind=g.kind;b.sensor=g.sensor;b.version=w.version;
   b.jacobian_whitened=Eigen::MatrixXd::Zero(dense.first.rows(),columns);b.residual_whitened=dense.second;
   int cursor=0;for(auto key:order){char ch=gtsam::Symbol(key).chr();int dim=ch=='v'?3:6;
    int off=15*gtsam::Symbol(key).index()+(ch=='x'?0:(ch=='v'?6:9));
    b.jacobian_whitened.middleCols(off,dim)=dense.first.middleCols(cursor,dim);cursor+=dim;}
   if(g.kind==FactorKind::CombinedImu){auto c=dynamic_cast<const gtsam::CombinedImuFactor*>(g.factors.front().get());b.covariance=c->preintegratedMeasurements().preintMeasCov();}
   else if(g.kind==FactorKind::BoundaryPrior)b.covariance=Eigen::MatrixXd::Identity(dense.second.size(),dense.second.size());
   else b.covariance=g.raw_covariance;
   if(b.covariance.rows()!=dense.second.size())throw std::runtime_error("prefix raw covariance lineage missing");
   auto r=gtsam::noiseModel::Gaussian::Covariance(b.covariance)->R();
   b.jacobian_raw=r.triangularView<Eigen::Upper>().solve(b.jacobian_whitened);b.residual_raw=r.triangularView<Eigen::Upper>().solve(b.residual_whitened);
   w.blocks.push_back(std::move(b));
  }
  w.H=Eigen::MatrixXd::Zero(0,columns);w.protected_state_map=Eigen::MatrixXd::Zero(3,columns);
  const auto pose=points.at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch));w.protected_state_map.block<3,3>(0,columns-12)=pose.rotation().matrix();
  return w;
 }
 std::vector<FaultModeBasis> uwbModes(const EpochTransaction& tx,int duration)const{
  auto material=groups;for(const auto& g:tx.uwb_groups)if(g.nominal)material[g.id.value()]=g;
  std::map<std::uint64_t,std::pair<size_t,AnchorId>> lineage;
  std::set<AnchorId> anchors;
  for(const auto& kv:episodes)for(const auto& m:kv.second.uwb_batch.measurements){lineage[m.id.value()]={kv.first,m.anchor_id};anchors.insert(m.anchor_id);}
  std::vector<FaultModeBasis> modes;
  for(size_t onset=1;onset<=tx.proposed_epoch;++onset)for(auto anchor:anchors)for(int length:{1,duration}){
   FaultModeBasis mode;mode.id=FaultModeId(modes.size()+1);mode.sensor=SensorType::Uwb;mode.anchor_id=anchor;
   mode.onset_epoch=onset-1;mode.parameter_dimension=1;mode.kind=length==1?FaultKind::Unknown:FaultKind::AnchorBiasPersistentConstant;
   mode.physical_source_id="prefix-anchor:"+std::to_string(anchor.value())+":onset:"+std::to_string(onset)+":duration:"+std::to_string(length);
   for(const auto& kv:material){const auto& g=kv.second;if(g.sensor!=SensorType::Uwb)continue;
    Eigen::MatrixXd map=Eigen::MatrixXd::Zero(g.source_measurements.size(),1);
    for(size_t row=0;row<g.source_measurements.size();++row){auto it=lineage.find(g.source_measurements[row].value());if(it==lineage.end())throw std::runtime_error("prefix measurement lineage missing");
     if(it->second.second==anchor && it->second.first>=onset && it->second.first<onset+length)map(row,0)=1.;}
    if(map.squaredNorm()>0)mode.raw_group_maps.emplace(g.id,std::move(map));
   }modes.push_back(std::move(mode));
  }return modes;
 }
 EpochTransaction material(const EpochTransaction& tx)const{
  auto copy=tx;copy.frozen_values=std::make_shared<const gtsam::Values>(points);copy.recoverable_history.clear();
  for(const auto& kv:episodes)if(kv.first<tx.proposed_epoch)copy.recoverable_history.push_back(kv.second);
  return copy;
 }
};
}
