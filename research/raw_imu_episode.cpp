#include "raw_imu_episode.hpp"
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl::research {
namespace {
double support(const RawImuEpisode& e, TimestampNs t) {
  return !(t<e.begin) && t<e.end ? 1. : 0.;
}
void validate(const EpochTransaction& tx, const RawImuEpisode& e) {
  if(e.event_id.empty() || e.axis<0 || e.axis>5 || !(e.begin<e.end) ||
      !tx.preintegration || tx.raw_imu_slice.size()<2)
    throw std::invalid_argument("invalid research IMU episode/material");
  for(std::size_t i=1;i<tx.raw_imu_slice.size();++i)
    if(!(tx.raw_imu_slice[i-1].timestamp<tx.raw_imu_slice[i].timestamp))
      throw std::invalid_argument("non-increasing raw sample time");
}
Eigen::VectorXd whiten(const EpochTransaction& tx,
    const gtsam::PreintegratedCombinedMeasurements& pim) {
  return gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov())->R()*
      episodeFactorError(tx,pim);
}
}
gtsam::PreintegratedCombinedMeasurements integrateEpisode(
    const EpochTransaction& tx,const RawImuEpisode& e,double amplitude) {
  validate(tx,e);
  if(!std::isfinite(amplitude))throw std::invalid_argument("nonfinite amplitude");
  auto pim=*tx.preintegration;
  pim.resetIntegration(); // preserves params and actual biasHat
  for(std::size_t i=1;i<tx.raw_imu_slice.size();++i) {
    const auto& l=tx.raw_imu_slice[i-1];const auto& r=tx.raw_imu_slice[i];
    Eigen::Vector3d acc=.5*(l.specific_force_mps2+r.specific_force_mps2);
    Eigen::Vector3d gyro=.5*(l.angular_velocity_radps+r.angular_velocity_radps);
    const double weight=.5*(support(e,l.timestamp)+support(e,r.timestamp));
    if(e.axis<3)acc(e.axis)+=weight*amplitude;else gyro(e.axis-3)+=weight*amplitude;
    pim.integrateMeasurement(acc,gyro,r.timestamp.seconds()-l.timestamp.seconds());
  }
  return pim;
}
Eigen::VectorXd episodeFactorError(const EpochTransaction& tx,
    const gtsam::PreintegratedCombinedMeasurements& pim) {
  auto values=tx.frozen_values;
  gtsam::Values recorded;
  if(!values) {
    const auto add=[&](std::size_t epoch,const NavigationState& s) {
      recorded.insert(gtsam::Symbol('x',epoch),gtsam::Pose3(gtsam::Rot3(s.q_world_body.normalized().toRotationMatrix()),s.position_world_m));
      recorded.insert(gtsam::Symbol('v',epoch),gtsam::Vector3(s.velocity_world_mps));
      recorded.insert(gtsam::Symbol('b',epoch),gtsam::imuBias::ConstantBias(s.accel_bias_mps2,s.gyro_bias_radps));
    };
    add(tx.previous_epoch,tx.previous_state);add(tx.proposed_epoch,tx.nominal_predicted_state);
    values=std::make_shared<const gtsam::Values>(recorded);
  }
  const auto& v=*values;
  gtsam::CombinedImuFactor factor(1,2,3,4,5,6,pim);
  return factor.evaluateError(v.at<gtsam::Pose3>(gtsam::Symbol('x',tx.previous_epoch)),
      v.at<gtsam::Vector3>(gtsam::Symbol('v',tx.previous_epoch)),
      v.at<gtsam::Pose3>(gtsam::Symbol('x',tx.proposed_epoch)),
      v.at<gtsam::Vector3>(gtsam::Symbol('v',tx.proposed_epoch)),
      v.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',tx.previous_epoch)),
      v.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',tx.proposed_epoch)));
}
RawImuResponse episodeResponse(const EpochTransaction& tx,
    const LinearizedFactorBlock& block,const RawImuEpisode& e,double epsilon,bool skip_empty_support) {
  validate(tx,e);
  if(!(epsilon>0) || !std::isfinite(epsilon) || block.whitener.rows()!=15 || block.whitener.cols()!=15)
    throw std::invalid_argument("invalid research response inputs");
  if(skip_empty_support) {
    bool affected=false;for(const auto& sample:tx.raw_imu_slice)affected=affected || support(e,sample.timestamp)>0.;
    if(!affected) {
      // One actual integration + noise/error validation preserves exceptions.
      // The raw inputs are identical at every amplitude, hence ALL derivatives
      // are exactly zero. No probability/rank/condition threshold is changed.
      const auto nominal=integrateEpisode(tx,e,0.);
      const auto error=whiten(tx,nominal);
      if(!error.allFinite() || !block.whitener.allFinite())throw std::runtime_error("nonfinite empty-support root");
      RawImuResponse zero;zero.raw=zero.frozen_whitened=zero.input_dependent_whitened=Eigen::VectorXd::Zero(15);
      zero.covariance_derivative=Eigen::MatrixXd::Zero(15,15);zero.integrations=1;return zero;
    }
  }
  const auto plus=integrateEpisode(tx,e,epsilon),minus=integrateEpisode(tx,e,-epsilon);
  const auto half_plus=integrateEpisode(tx,e,epsilon/2),half_minus=integrateEpisode(tx,e,-epsilon/2);
  RawImuResponse out;
  out.raw=-(episodeFactorError(tx,plus)-episodeFactorError(tx,minus))/(2*epsilon);
  const Eigen::VectorXd half=-(episodeFactorError(tx,half_plus)-episodeFactorError(tx,half_minus))/epsilon;
  out.convergence_relative=(out.raw-half).norm()/std::max(1e-12,half.norm());
  out.frozen_whitened=block.whitener*out.raw;
  out.input_dependent_whitened=-(whiten(tx,plus)-whiten(tx,minus))/(2*epsilon);
  out.covariance_derivative=(plus.preintMeasCov()-minus.preintMeasCov())/(2*epsilon);
  for(const auto& s:tx.raw_imu_slice)out.affected_samples+=support(e,s.timestamp)>0.;
  for(std::size_t i=1;i<tx.raw_imu_slice.size();++i)
    out.affected_trapezoids+=support(e,tx.raw_imu_slice[i-1].timestamp)+support(e,tx.raw_imu_slice[i].timestamp)>0.;
  if(!out.raw.allFinite() || !out.frozen_whitened.allFinite() || !out.input_dependent_whitened.allFinite() || !out.covariance_derivative.allFinite())
    throw std::runtime_error("nonfinite research response");
  return out;
}
EpochTransaction historicalEpisodeTransaction(const HistoricalEpochContext& h) {
  EpochTransaction tx;tx.previous_epoch=h.previous_epoch;tx.proposed_epoch=h.proposed_epoch;
  tx.begin=h.begin;tx.end=h.end;tx.previous_state=h.previous_state;
  tx.nominal_predicted_state=h.current_state;tx.raw_imu_slice=h.raw_imu_slice;
  tx.preintegration=h.preintegration;return tx;
}
}  // namespace uwb_imu_pl::research
