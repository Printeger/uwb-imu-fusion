#include "scenario_material.hpp"
#include "sc2_imu_scope.hpp"
#include <iostream>
#include <gtsam/linear/NoiseModel.h>
using namespace uwb_imu_pl;using namespace uwb_imu_pl::research;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
RawInclusionParameters parameters(const NoisyMotionScenario& s){RawInclusionParameters p;p.numeric=s.numeric_certificate;p.initial_sigmas=s.cfg.realtime.prior_sigmas;p.gravity=s.cfg.imu.gravity_mps2;return p;}

void responseOracle(const NoisyMotionScenario& scenario){
 IncrementalUwbImuEstimator backend(scenario.cfg,scenario.lever);backend.initialize(scenario.initial_observation,scenario.cfg.realtime.prior_sigmas);backend.ingestImu(scenario.imu[0]);
 for(int e=1;e<=2;++e){for(int i=(e-1)*10+1;i<=e*10;++i)backend.ingestImu(scenario.imu[i]);auto tx=backend.prepareEpoch(scenario.uwb[e-1]);
  for(int axis=0;axis<6;++axis){RawImuEpisode episode{"response-oracle",TimestampNs(5000000),TimestampNs(55000000),axis};LinearizedFactorBlock block;block.whitener=gtsam::noiseModel::Gaussian::Covariance(tx.preintegration->preintMeasCov())->R();auto response=episodeResponse(tx,block,episode);
   auto independent=[&](double amplitude){auto samples=tx.raw_imu_slice;for(auto& sample:samples)if(!(sample.timestamp<episode.begin) && sample.timestamp<episode.end){if(axis<3)sample.specific_force_mps2(axis)+=amplitude;else sample.angular_velocity_radps(axis-3)+=amplitude;}auto pim=*tx.preintegration;pim.resetIntegration();for(size_t i=1;i<samples.size();++i)pim.integrateMeasurement(.5*(samples[i-1].specific_force_mps2+samples[i].specific_force_mps2),.5*(samples[i-1].angular_velocity_radps+samples[i].angular_velocity_radps),.005);return episodeFactorError(tx,pim);};
   Eigen::VectorXd derivative=-(independent(2e-5)-independent(-2e-5))/(4e-5);require((derivative-response.raw).norm()<2e-6*std::max(1e-12,derivative.norm()),"matched-RW six-axis independent perturbation oracle");require(response.affected_samples==size_t(e==1?10:1) && response.affected_trapezoids==size_t(e==1?10:1),"raw response shared endpoint");
  }
  auto plan=EpochCommitPlan::nominalPlan(tx);backend.commitEpoch(std::move(tx),plan);
 }
}
int main(){try{
 NoisyMotionScenario nominal("config/sc2_imu.yaml","U0",20261010,30);
 responseOracle(nominal);
 auto law=parameters(nominal);Sc2ImuMonitor healthy(nominal.initial_observation,law);
 for(auto sample:nominal.imu)healthy.append(sample);require(healthy.assessment().valid && !healthy.assessment().alarm,"nominal shared source prefix");
 for(int axis=0;axis<6;++axis){Sc2ImuMonitor monitor(nominal.initial_observation,law);double amplitude=axis<3?20.:1.;
  RawImuEpisode trueEpisode{"offline-only",TimestampNs(705000000),TimestampNs(1205000000),axis};
  auto closure=sc2SupportClosure(trueEpisode,30);require(closure.valid && closure.raw_samples.size()==100 && closure.direct_pim_epochs.front()==15 && closure.direct_pim_epochs.back()==25 && closure.reference_dependent_epochs.back()==30,"six-axis shared-tail/reference closure");
  for(auto sample:nominal.imu){if(!(sample.timestamp<trueEpisode.begin) && sample.timestamp<trueEpisode.end){if(axis<3)sample.specific_force_mps2(axis)+=amplitude;else sample.angular_velocity_radps(axis-3)+=amplitude;}monitor.append(sample);}
  auto result=monitor.assessment();require(result.valid && result.alarm && result.first_alarm==trueEpisode.begin,"actual raw six-axis alarm");
  bool retained=false;for(const auto& candidate:monitor.candidates())if(candidate.possible && candidate.episode.axis==axis && candidate.episode.begin==trueEpisode.begin && candidate.episode.end==trueEpisode.end){require(candidate.theta_low<=amplitude && candidate.theta_high>=amplitude,"single shared amplitude retained");retained=true;}
  require(retained,"true physical explanation never discarded");
 }
 RawImuEpisode shortEpisode{"short",TimestampNs(705000000),TimestampNs(755000000),0};auto shortClosure=sc2SupportClosure(shortEpisode,30);require(shortClosure.raw_samples.size()==10 && shortClosure.direct_pim_epochs==std::vector<size_t>({15,16}),"50ms endpoint closure");
 auto capacity=Sc2ImuScope{};capacity.candidate_capacity=600;bool rejected=false;try{Sc2ImuMonitor incomplete(nominal.initial_observation,law,capacity);}catch(const std::exception&){rejected=true;}require(rejected,"capacity cannot truncate fault scope");
 auto bad=nominal.imu[0];bad.id=MeasurementId(999);healthy.append(bad);require(!healthy.assessment().valid,"duplicate/stale prefix refused");
 std::cout<<"SC2 six-axis scope, shared-tail/dependent closure, raw alarm and shared-theta retention PASS\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
