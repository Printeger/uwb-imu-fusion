#pragma once
#include "raw_imu_episode.hpp"
#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include <Eigen/Geometry>

namespace uwb_imu_pl::research {
inline Eigen::Quaterniond attitude(double time,bool rotating) {
  if(!rotating)return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(1.2*time,Eigen::Vector3d(1.,2.,3.).normalized()));
}
inline ImuMeasurement sample(const IntegrityConfig& cfg,int index,bool rotating=false) {
  const double t=index*.005;
  ImuMeasurement s;s.id=MeasurementId(index+1);s.timestamp=TimestampNs(index*5000000LL);
  s.specific_force_mps2=attitude(t,rotating).conjugate()*Eigen::Vector3d(0,0,cfg.imu.gravity_mps2);
  s.angular_velocity_radps.setZero();
  if(rotating)s.angular_velocity_radps=Eigen::Vector3d(1.,2.,3.).normalized()*1.2;
  return s;
}
inline UwbBatch ranges(const IntegrityConfig& cfg,int epoch,bool rotating=false,double anchor_fault=0.,bool flat=false) {
  UwbBatch b;b.id=BatchId(epoch);b.timestamp=TimestampNs(epoch*50000000LL);
  for(std::size_t i=0;i<cfg.anchors.size();++i) {
    UwbMeasurement m;m.id=MeasurementId(epoch*100+i+1);m.factor_id=FactorId(epoch*100+i+1);
    m.anchor_id=cfg.anchors[i].id;m.timestamp=b.timestamp;m.anchor_position_m=cfg.anchors[i].position_world_m;
    if(flat)m.anchor_position_m.z()=1.1;
    m.range_m=(m.anchor_position_m-Eigen::Vector3d(0,0,1)).norm()+(i==0?anchor_fault:0.);
    m.sigma_m=cfg.realtime.range_sigma_m;b.measurements.push_back(m);
  }
  return b;
}
inline NavigationState initialState() { NavigationState s;s.position_world_m={0,0,1};return s; }
inline EpochTransaction prepare(IncrementalUwbImuEstimator& estimator,const IntegrityConfig& cfg,int epoch,bool rotating=false) {
  for(int i=(epoch-1)*10+1;i<=epoch*10;++i)estimator.ingestImu(sample(cfg,i,rotating));
  return estimator.prepareEpoch(ranges(cfg,epoch,rotating));
}
} // namespace
