#include "uwb_imu_pl/estimation/causal_initializer.hpp"
#include "uwb_imu_pl/estimation/estimation_tuning.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {
gtsam::Key x(std::size_t k) { return gtsam::Symbol('x', k); }
gtsam::Key v(std::size_t k) { return gtsam::Symbol('v', k); }
gtsam::Key b(std::size_t k) { return gtsam::Symbol('b', k); }
gtsam::Pose3 pose(const NavigationState& s) {
  return {gtsam::Rot3(s.q_world_body.toRotationMatrix()), s.position_world_m};
}
gtsam::imuBias::ConstantBias bias(const NavigationState& s) {
  return {s.accel_bias_mps2, s.gyro_bias_radps};
}

Eigen::Vector3d initialPosition(const std::vector<UwbMeasurement>& ranges,
                              const Eigen::Quaterniond& q,
                              const Eigen::Vector3d& fallback, bool below) {
  std::map<std::pair<std::uint64_t, std::uint64_t>, std::vector<UwbMeasurement>> groups;
  for (const auto& r : ranges) groups[{r.tag_id.value(), r.anchor_id.value()}].push_back(r);
  std::vector<UwbMeasurement> observations;
  std::set<std::uint64_t> anchors;
  for (auto& entry : groups) {
    auto& values = entry.second;
    std::sort(values.begin(), values.end(), [](const auto& a, const auto& b) {
      return a.range_m < b.range_m;
    });
    observations.push_back(values[values.size()/2]);
    anchors.insert(values.front().anchor_id.value());
  }
  if (anchors.size() < 4) throw std::runtime_error("BOOTSTRAP_FEWER_THAN_FOUR_ANCHORS");
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto& r : observations) center += r.anchor_position_m;
  center /= observations.size();
  double scale = 1.0;
  for (const auto& r : observations) scale = std::max(scale, (r.anchor_position_m-center).norm());
  auto cost = [&](const Eigen::Vector3d& p) {
    double e = 0.0;
    for (const auto& r : observations) {
      const Eigen::Vector3d lever = r.lever_arm_body_m ? *r.lever_arm_body_m : fallback;
      const double residual = ((p + q*lever-r.anchor_position_m).norm()-r.range_m)/r.sigma_m;
      e += residual*residual;
    }
    return e;
  };
  std::vector<Eigen::Vector3d> seeds{center};
  for (int i=0; i<3; ++i) {
    auto positive=center.eval(), negative=center.eval();
    positive(i)+=2*scale; negative(i)-=2*scale;
    seeds.push_back(positive); seeds.push_back(negative);
  }
  std::vector<std::pair<double, Eigen::Vector3d>> solutions;
  for (auto p : seeds) {
    double lambda=1e-3;
    for (int iter=0; iter<100; ++iter) {
      Eigen::Matrix3d h=Eigen::Matrix3d::Zero();
      Eigen::Vector3d gradient=Eigen::Vector3d::Zero();
      for (const auto& r : observations) {
        const Eigen::Vector3d lever = r.lever_arm_body_m ? *r.lever_arm_body_m : fallback;
        const Eigen::Vector3d delta=p+q*lever-r.anchor_position_m;
        const double norm=std::max(1e-9,delta.norm());
        const Eigen::Vector3d j=delta/(norm*r.sigma_m);
        h+=j*j.transpose(); gradient+=j*(norm-r.range_m)/r.sigma_m;
      }
      const Eigen::Vector3d step=-(h+lambda*Eigen::Matrix3d::Identity()).ldlt().solve(gradient);
      const Eigen::Vector3d candidate=p+step;
      if (candidate.allFinite() && cost(candidate)<cost(p)) {
        p=candidate; lambda*=0.5;
        if(step.norm()<1e-8) break;
      } else lambda*=10.0;
    }
    solutions.emplace_back(cost(p),p);
  }
  std::sort(solutions.begin(),solutions.end(),[](const auto& a,const auto& b){return a.first<b.first;});
  Eigen::Vector3d selected=solutions.front().second;
  if(below) for(const auto& candidate:solutions)
    if(candidate.first<=solutions.front().first+1e-3 && candidate.second.z()<selected.z())
      selected=candidate.second;
  if(!selected.allFinite()) throw std::runtime_error("BOOTSTRAP_NONFINITE_POSITION");
  return selected;
}

gtsam::PreintegratedCombinedMeasurements integrate(
    const std::vector<ImuMeasurement>& samples, TimestampNs begin, TimestampNs end,
    const boost::shared_ptr<gtsam::PreintegrationCombinedParams>& params,
    const gtsam::imuBias::ConstantBias& bias_value, double max_gap) {
  gtsam::PreintegratedCombinedMeasurements p(params,bias_value);
  auto held=samples.front();
  for(const auto& s:samples) if(!(begin<s.timestamp)) held=s;
  auto t=begin;
  for(const auto& s:samples) {
    if(!(t<s.timestamp)) continue;
    if(end<s.timestamp) break;
    const double dt=s.timestamp.seconds()-t.seconds();
    if(dt>max_gap+1e-12) throw std::runtime_error("BOOTSTRAP_IMU_GAP");
    p.integrateMeasurement(0.5*(held.specific_force_mps2+s.specific_force_mps2),
                           0.5*(held.angular_velocity_radps+s.angular_velocity_radps),dt);
    t=s.timestamp; held=s;
  }
  if(t<end) {
    const double dt=end.seconds()-t.seconds();
    if(dt>max_gap+1e-12) throw std::runtime_error("BOOTSTRAP_IMU_GAP");
    p.integrateMeasurement(held.specific_force_mps2,held.angular_velocity_radps,dt);
  }
  return p;
}
}  // namespace

CausalInitializationV1 initializeCausallyV1(
    const std::vector<ImuMeasurement>& imu, const std::vector<UwbMeasurement>& ranges,
    const IntegrityConfig& config, const Eigen::Vector3d& lever, bool below) {
  if(imu.size()<10) throw std::runtime_error("BOOTSTRAP_INSUFFICIENT_IMU");
  const auto begin=imu.front().timestamp;
  const auto end=TimestampNs::fromSeconds(begin.seconds()+2.0);
  if(imu.back().timestamp<end) throw std::runtime_error("BOOTSTRAP_WAITING_FOR_TWO_SECONDS");
  std::vector<ImuMeasurement> samples;
  for(const auto& s:imu) {
    if(end<s.timestamp) break;
    if(!s.specific_force_mps2.allFinite() || !s.angular_velocity_radps.allFinite())
      throw std::runtime_error("BOOTSTRAP_NONFINITE_IMU");
    if(!samples.empty() && !(samples.back().timestamp<s.timestamp))
      throw std::runtime_error("BOOTSTRAP_REVERSED_IMU");
    if(!samples.empty() && s.timestamp.seconds()-samples.back().timestamp.seconds()>config.imu.max_gap_s+1e-12)
      throw std::runtime_error("BOOTSTRAP_IMU_GAP");
    samples.push_back(s);
  }
  if(samples.size()<10) throw std::runtime_error("BOOTSTRAP_INSUFFICIENT_IMU");
  CausalInitializationV1 out;
  out.boundary=samples.back(); out.boundary.timestamp=end;
  if(end.seconds()-samples.back().timestamp.seconds()>config.imu.max_gap_s+1e-12)
    throw std::runtime_error("BOOTSTRAP_IMU_GAP");
  out.prior_sigmas=config.realtime.prior_sigmas;
  out.prior_sigmas(2)=M_PI;  // unobserved world yaw is not known to 0.1 rad
  Eigen::Vector3d acc=Eigen::Vector3d::Zero(), gyro=Eigen::Vector3d::Zero();
  for(const auto& s:samples) {acc+=s.specific_force_mps2;gyro+=s.angular_velocity_radps;}
  acc/=samples.size(); gyro/=samples.size();
  // Prefer a stationary suffix: it directly seeds the terminal boundary.
  for(std::size_t first=0; first<samples.size(); ++first) {
    if(samples.back().timestamp.seconds()-samples[first].timestamp.seconds()<1.0) break;
    Eigen::Vector3d a=Eigen::Vector3d::Zero(),w=Eigen::Vector3d::Zero();
    double w2=0.0,a2=0.0;
    for(std::size_t i=first;i<samples.size();++i){a+=samples[i].specific_force_mps2;w+=samples[i].angular_velocity_radps;w2+=samples[i].angular_velocity_radps.squaredNorm();}
    const double n=samples.size()-first; a/=n; w/=n;
    for(std::size_t i=first;i<samples.size();++i)a2+=(samples[i].specific_force_mps2-a).squaredNorm();
    const double wrms=std::sqrt(w2/n), astd=std::sqrt(a2/n), norm_error=std::abs(a.norm()-config.imu.gravity_mps2);
    out.gyro_rms_radps=wrms;out.accel_std_mps2=astd;out.accel_norm_error_mps2=norm_error;
    if(wrms<=0.05 && astd<=0.20 && norm_error<=0.30){out.stationary=true;acc=a;gyro=w;break;}
  }
  if(acc.norm()<1e-6)throw std::runtime_error("BOOTSTRAP_DEGENERATE_ACCEL");
  NavigationState seed;
  seed.timestamp=begin;
  seed.q_world_body=Eigen::Quaterniond::FromTwoVectors(acc.normalized(),Eigen::Vector3d::UnitZ()).normalized();
  if(out.stationary) {
    seed.gyro_bias_radps=gyro;
    seed.accel_bias_mps2=acc.normalized()*(acc.norm()-config.imu.gravity_mps2);
  }
  std::vector<UwbMeasurement> usable;
  for(const auto& r:ranges) if(!(r.timestamp<begin) && !(end<r.timestamp)) {
    if(!r.anchor_position_m.allFinite() || !std::isfinite(r.range_m) || r.range_m<=0 || !std::isfinite(r.sigma_m) || r.sigma_m<=0)
      throw std::runtime_error("BOOTSTRAP_INVALID_RANGE");
    usable.push_back(r);
  }
  seed.position_world_m=initialPosition(usable,seed.q_world_body,lever,below);
  if(out.stationary) {out.state=seed;out.state.timestamp=end;return out;}

  // Short discrete graph only for initialization. Measurements are consumed
  // once here, and only the terminal state/prior are passed to the backend.
  auto params=gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(config.imu.gravity_mps2);
  params->accelerometerCovariance=gtsam::I_3x3*std::pow(config.imu.accelerometer_sigma,2);
  params->gyroscopeCovariance=gtsam::I_3x3*std::pow(config.imu.gyroscope_sigma,2);
  params->biasAccCovariance=gtsam::I_3x3*std::pow(config.imu.accelerometer_bias_rw_sigma,2);
  params->biasOmegaCovariance=gtsam::I_3x3*std::pow(config.imu.gyroscope_bias_rw_sigma,2);
  params->integrationCovariance=gtsam::I_3x3*1e-9;
  const auto tuning=readEstimationTuningV1(config);
  const Eigen::Matrix<double,6,1> bs=tuning.bias_integration_sigmas ? *tuning.bias_integration_sigmas : out.prior_sigmas.tail<6>().eval();
  params->biasAccOmegaInt=bs.array().square().matrix().asDiagonal();
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.addPrior(x(0),pose(seed),gtsam::noiseModel::Diagonal::Sigmas(out.prior_sigmas.head<6>()));
  graph.addPrior(v(0),seed.velocity_world_mps,gtsam::noiseModel::Isotropic::Sigma(3,2.0));
  graph.addPrior(b(0),bias(seed),gtsam::noiseModel::Diagonal::Sigmas(out.prior_sigmas.tail<6>()));
  values.insert(x(0),pose(seed));values.insert(v(0),seed.velocity_world_mps);values.insert(b(0),bias(seed));
  std::vector<TimestampNs> times{begin};
  for(int i=1;i<=10;++i)times.push_back(TimestampNs::fromSeconds(begin.seconds()+0.2*i));
  auto previous=seed;
  for(std::size_t k=1;k<times.size();++k) {
    auto p=integrate(samples,times[k-1],times[k],params,bias(previous),config.imu.max_gap_s);
    const auto predicted=p.predict(gtsam::NavState(pose(previous),previous.velocity_world_mps),bias(previous));
    previous.position_world_m=predicted.position();previous.velocity_world_mps=predicted.velocity();
    previous.q_world_body=Eigen::Quaterniond(predicted.pose().rotation().matrix());
    graph.add(gtsam::CombinedImuFactor(x(k-1),v(k-1),x(k),v(k),b(k-1),b(k),p));
    values.insert(x(k),pose(previous));values.insert(v(k),previous.velocity_world_mps);values.insert(b(k),bias(previous));
  }
  for(std::size_t k=0;k<times.size();++k) {
    UwbBatch batch;batch.timestamp=times[k];
    for(const auto& r:usable) {
      const auto nearest=static_cast<std::size_t>(std::min(10.0,std::max(0.0,std::round((r.timestamp.seconds()-begin.seconds())/0.2))));
      if(nearest==k)batch.measurements.push_back(r);
    }
    if(!batch.measurements.empty())graph.add(boost::make_shared<UwbPoseBatchFactor>(x(k),batch,lever));
  }
  gtsam::LevenbergMarquardtParams lm;lm.maxIterations=5;
  out.window_cost_before=graph.error(values);
  const auto optimized=gtsam::LevenbergMarquardtOptimizer(graph,values,lm).optimize();
  out.window_cost_after=graph.error(optimized);
  if(!std::isfinite(out.window_cost_after) || out.window_cost_after>out.window_cost_before)
    throw std::runtime_error("BOOTSTRAP_DYNAMIC_OPTIMIZATION_FAILED");
  const std::size_t last=times.size()-1;
  out.state=seed;out.state.timestamp=end;
  const auto final_pose=optimized.at<gtsam::Pose3>(x(last));
  const auto final_bias=optimized.at<gtsam::imuBias::ConstantBias>(b(last));
  out.state.position_world_m=final_pose.translation();
  out.state.q_world_body=Eigen::Quaterniond(final_pose.rotation().matrix());
  out.state.velocity_world_mps=optimized.at<gtsam::Vector3>(v(last));
  out.state.accel_bias_mps2=final_bias.accelerometer();out.state.gyro_bias_radps=final_bias.gyroscope();
  const gtsam::Marginals marginals(graph,optimized,gtsam::Marginals::QR);
  const std::array<gtsam::Key,3> keys{x(last),v(last),b(last)};
  // The existing initializer API carries a diagonal prior. Preserve a
  // conservative linear covariance when eliminating cross-correlations:
  // D_ii = sum_j |C_ij| makes D-C positive semidefinite by diagonal dominance.
  const auto joint=marginals.jointMarginalCovariance(gtsam::KeyVector(keys.begin(),keys.end()));
  Eigen::Matrix<double,15,15> covariance;
  const std::array<int,3> widths{6,3,6};
  int row=0;
  for(int i=0;i<3;++i) {
    int col=0;
    for(int j=0;j<3;++j) {
      covariance.block(row,col,widths[i],widths[j])=joint.at(keys[i],keys[j]);
      col+=widths[j];
    }
    row+=widths[i];
  }
  for(int i=0;i<15;++i)
    out.prior_sigmas(i)=std::sqrt(std::max(1e-12,covariance.row(i).cwiseAbs().sum()));
  if(!out.state.position_world_m.allFinite() || !out.state.velocity_world_mps.allFinite() || !out.prior_sigmas.allFinite() || !out.state.q_world_body.coeffs().allFinite() ||
     !out.state.accel_bias_mps2.allFinite() || !out.state.gyro_bias_radps.allFinite())
    throw std::runtime_error("BOOTSTRAP_NONFINITE_STATE");
  return out;
}
}  // namespace uwb_imu_pl
