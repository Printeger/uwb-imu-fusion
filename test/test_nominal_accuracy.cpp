#include "uwb_imu_pl/estimation/causal_initializer.hpp"
#include "uwb_imu_pl/estimation/estimation_tuning.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/Eigenvalues>
#include <cmath>

namespace {
using namespace uwb_imu_pl;
IntegrityConfig config(const std::string& tuning = "") {
  IntegrityConfig c;
  c.fde.profile = FdeProfile::Off;
  c.imu.max_gap_s = .03;
  c.imu.accelerometer_sigma = .03;
  c.imu.gyroscope_sigma = .003;
  c.imu.accelerometer_bias_rw_sigma = .001;
  c.imu.gyroscope_bias_rw_sigma = .0001;
  c.realtime.prior_sigmas << .1,.1,3.14,1,1,1,1,1,1,.1,.1,.1,.01,.01,.01;
  c.resolved_yaml = "estimation_tuning:\n  version: 1\n" + tuning;
  return c;
}
UwbBatch ranges(double t, const Eigen::Vector3d& p, std::size_t n=5) {
  const std::vector<Eigen::Vector3d> anchors{{-5,-5,0},{5,-5,1},{5,5,3},{-5,5,4},{0,-6,2}};
  UwbBatch out; out.timestamp=TimestampNs::fromSeconds(t);out.id=BatchId(out.timestamp.value());
  for(std::size_t i=0;i<n;++i){UwbMeasurement m;m.timestamp=out.timestamp;
    m.id=MeasurementId(out.timestamp.value()+i);m.factor_id=FactorId(m.id.value());
    m.anchor_id=AnchorId(i+1);m.anchor_position_m=anchors[i];m.range_m=(p-anchors[i]).norm();m.sigma_m=.1;
    out.measurements.push_back(m);}
  return out;
}
std::vector<ImuMeasurement> imu(double frequency, bool moving=false) {
  std::vector<ImuMeasurement> out;
  for(int i=0;i<=static_cast<int>(2.1*frequency);++i){ImuMeasurement m;m.timestamp=TimestampNs::fromSeconds(i/frequency);
    m.specific_force_mps2={0,0,9.80665}; m.angular_velocity_radps={0,0,moving?.2:0.02};out.push_back(m);}
  return out;
}
void feed(IncrementalUwbImuEstimator& e, double start, double end, double hz=100) {
  for(double t=start;t<=end+1e-9;t+=1/hz){ImuMeasurement m;m.timestamp=TimestampNs::fromSeconds(t);
    m.specific_force_mps2={0,0,9.80665};e.ingestImu(m);}
}

TEST(NominalAccuracy, TuningRejectsIncompatibleFdeAndInvalidValues) {
  auto c=config("  nominal_initial_guess: imu\n  nominal_lm_iterations: 5\n  nominal_lag_s: 2\n");
  EXPECT_EQ(readEstimationTuningV1(c).nominal_lm_iterations,5);
  c.fde.profile=FdeProfile::JointOrder1;EXPECT_THROW(readEstimationTuningV1(c),std::runtime_error);
  EXPECT_THROW(readEstimationTuningV1(config("  bias_integration_sigmas: [0,1,1,1,1,1]\n")),std::runtime_error);
  EXPECT_THROW(readEstimationTuningV1(config("  typo: 1\n")),std::runtime_error);
  EXPECT_THROW(readEstimationTuningV1(config("  nominal_lm_iterations: 6\n")),std::runtime_error);
}
TEST(NominalAccuracy, StationaryBootstrapConsumesTwoSecondsAndIgnoresFuture) {
  auto samples=imu(100);auto batch=ranges(1,{1,2,1});auto c=config();
  const auto result=initializeCausallyV1(samples,batch.measurements,c,Eigen::Vector3d::Zero());
  ASSERT_TRUE(result.stationary);EXPECT_EQ(result.state.timestamp,TimestampNs::fromSeconds(2));
  EXPECT_LT((result.state.position_world_m-Eigen::Vector3d(1,2,1)).norm(),1e-6);
  EXPECT_NEAR(result.state.gyro_bias_radps.z(),.02,1e-12);EXPECT_GT(result.prior_sigmas(2),3);
  for(auto& s:samples)if(TimestampNs::fromSeconds(2)<s.timestamp){s.specific_force_mps2.setConstant(1e5);s.angular_velocity_radps.setConstant(1e4);}
  auto future=ranges(2.05,{100,100,100});batch.measurements.insert(batch.measurements.end(),future.measurements.begin(),future.measurements.end());
  const auto altered=initializeCausallyV1(samples,batch.measurements,c,Eigen::Vector3d::Zero());
  EXPECT_LT((result.state.position_world_m-altered.state.position_world_m).norm(),1e-12);
  EXPECT_LT((result.state.gyro_bias_radps-altered.state.gyro_bias_radps).norm(),1e-12);
}
TEST(NominalAccuracy, DynamicBootstrapDoesNotInterpretRotationAsGyroBias) {
  auto samples=imu(100,true);std::vector<UwbMeasurement> obs;
  for(double t=0;t<=2.001;t+=.2){auto batch=ranges(t,{1,2,1});obs.insert(obs.end(),batch.measurements.begin(),batch.measurements.end());}
  const auto result=initializeCausallyV1(samples,obs,config(),Eigen::Vector3d::Zero());
  EXPECT_FALSE(result.stationary);EXPECT_LT(result.state.gyro_bias_radps.norm(),.01);
  EXPECT_LE(result.window_cost_after,result.window_cost_before);EXPECT_TRUE(result.prior_sigmas.allFinite());
}
TEST(NominalAccuracy, DynamicRangesUseActualTimesWithoutDuplicateInformation) {
  auto samples=imu(100,true);std::vector<UwbMeasurement> obs;
  for(int i=0;i<10;++i) {
    const double t=.07+.19*i;
    auto batch=ranges(t,{1+.8*t,2,1});
    for(auto& r:batch.measurements)r.sigma_m=.01;
    obs.insert(obs.end(),batch.measurements.begin(),batch.measurements.end());
  }
  const auto legacy=initializeCausallyV1(samples,obs,config(),Eigen::Vector3d::Zero());
  const auto exact=initializeCausallyV1(samples,obs,
      config("  bootstrap_exact_uwb_times: true\n"),Eigen::Vector3d::Zero());
  EXPECT_GT(legacy.maximum_range_time_error_s,.05);
  EXPECT_EQ(exact.maximum_range_time_error_s,0);
  EXPECT_EQ(exact.range_factors,obs.size());
  EXPECT_GT(exact.discrete_nodes,legacy.discrete_nodes);
  EXPECT_LT(std::abs(exact.state.position_world_m.x()-2.6),
            std::abs(legacy.state.position_world_m.x()-2.6));
}
TEST(NominalAccuracy, ExplicitIntegrationModelIsIndependentOfInitializationPrior) {
  Eigen::Matrix<double,15,15> cov[2];
  for(int index=0;index<2;++index) {
    auto c=config("  bias_integration_sigmas: [0.1,0.1,0.1,0.01,0.01,0.01]\n");
    auto prior=c.realtime.prior_sigmas;prior.tail<6>()*=index?100.:1.;
    IncrementalUwbImuEstimator e(c,Eigen::Vector3d::Zero());NavigationState s;e.initialize(s,prior);
    feed(e,0,.1);auto tx=e.prepareEpoch(ranges(.1,{0,0,0}),EpochPreparationOptions::nominalOnly());
    auto f=boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(tx.imu_group.factors.front());ASSERT_TRUE(f);
    cov[index]=f->preintegratedMeasurements().preintMeasCov();
    DiscardReason reason;e.discardEpoch(std::move(tx),reason);
  }
  EXPECT_LT((cov[0]-cov[1]).norm(),1e-15);
}
TEST(NominalAccuracy, IdenticalLowExcitationImuCannotConfirmConstantVelocityRest) {
  const auto c=config("  bootstrap_uwb_motion_check: true\n  bootstrap_exact_uwb_times: true\n");
  for(double speed:{0.,.8}) {
    std::vector<UwbMeasurement> obs;
    for(int i=0;i<=20;++i) {
      const double t=.1*i;auto batch=ranges(t,{1+speed*t,2,1});
      for(auto& r:batch.measurements)r.sigma_m=.001;
      obs.insert(obs.end(),batch.measurements.begin(),batch.measurements.end());
    }
    const auto result=initializeCausallyV1(imu(100),obs,c,Eigen::Vector3d::Zero());
    EXPECT_EQ(result.stationary,speed==0);
    EXPECT_EQ(result.motion_status,speed==0?"STATIC_CONFIRMED":"DYNAMIC");
    if(speed>0){EXPECT_GT(result.state.velocity_world_mps.x(),.5);}
  }
  auto single=ranges(1,{1,2,1});
  const auto unresolved=initializeCausallyV1(imu(100),single.measurements,c,Eigen::Vector3d::Zero());
  EXPECT_FALSE(unresolved.stationary);EXPECT_EQ(unresolved.motion_status,"MOTION_UNRESOLVED");
}
TEST(NominalAccuracy, DeclaredHeightPriorSelectsPhysicalBranchWithoutReusingSeedPrior) {
  std::vector<UwbMeasurement> obs;
  const std::vector<Eigen::Vector3d> anchors{{-5,-5,1.58},{5,-5,1.61},{5,5,1.59},{-5,5,1.62}};
  for(int i=0;i<=10;++i) {
    auto batch=ranges(.2*i,{1,2,3},4);
    for(std::size_t j=0;j<4;++j){auto& r=batch.measurements[j];r.anchor_position_m=anchors[j];r.range_m=(Eigen::Vector3d(1,2,3)-anchors[j]).norm();}
    obs.insert(obs.end(),batch.measurements.begin(),batch.measurements.end());
  }
  auto c=config("  bootstrap_seed_only: true\n  bootstrap_enforce_below_anchors: true\n");
  EXPECT_THROW(initializeCausallyV1(imu(100,true),obs,c,Eigen::Vector3d::Zero(),false),std::runtime_error);
  const auto result=initializeCausallyV1(imu(100,true),obs,c,Eigen::Vector3d::Zero(),true);
  EXPECT_LT(result.position_seed_m.z(),1.6);EXPECT_LT(result.state.position_world_m.z(),1.6);
  EXPECT_EQ(result.position_prior_source,"WEAK_CONFIGURED_ORIGIN_REGULARIZER_SIGMA_1E6");
  auto prior_changed=c;prior_changed.realtime.prior_sigmas.segment<3>(3).setConstant(.001);
  const auto changed=initializeCausallyV1(imu(100,true),obs,prior_changed,Eigen::Vector3d::Zero(),true);
  EXPECT_LT((result.state.position_world_m-changed.state.position_world_m).norm(),1e-9);
  EXPECT_EQ(result.range_factors,obs.size());
}
TEST(NominalAccuracy, BootstrapFailsClearlyWithMissingAnchorsOrImuGaps) {
  auto samples=imu(100);auto batch=ranges(1,{1,2,1},3);
  EXPECT_THROW(initializeCausallyV1(samples,batch.measurements,config(),Eigen::Vector3d::Zero()),std::runtime_error);
  batch=ranges(1,{1,2,1});samples.resize(100);
  EXPECT_THROW(initializeCausallyV1(samples,batch.measurements,config(),Eigen::Vector3d::Zero()),std::runtime_error);
}
TEST(NominalAccuracy, BiasIntegrationConfigurationChangesPositiveDefiniteCovariance) {
  for(double hz:{50.,100.,200.})for(bool moving:{false,true}) {
    Eigen::Matrix<double,15,15> cov[2];int index=0;
    for(const auto& tuning:{std::string(""),std::string("  bias_integration_sigmas: [0.3,0.3,0.3,0.03,0.03,0.03]\n")}) {
      auto c=config(tuning);IncrementalUwbImuEstimator e(c,Eigen::Vector3d::Zero());NavigationState s;e.initialize(s,c.realtime.prior_sigmas);
      auto samples=imu(hz,moving);for(const auto& m:samples)if(!(TimestampNs::fromSeconds(.1)<m.timestamp))e.ingestImu(m);
      auto tx=e.prepareEpoch(ranges(.1,{0,0,0}),EpochPreparationOptions::nominalOnly());
      auto factor=boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(tx.imu_group.factors.front());ASSERT_TRUE(factor);
      cov[index++]=factor->preintegratedMeasurements().preintMeasCov();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,15,15>> spectrum(cov[index-1]);EXPECT_GT(spectrum.eigenvalues().minCoeff(),0);
      EXPECT_TRUE(cov[index-1].allFinite());
      DiscardReason reason;e.discardEpoch(std::move(tx),reason);
    }
    EXPECT_GT((cov[1]-cov[0]).norm(),1e-6);
  }
}
TEST(NominalAccuracy, PublishedMeanMatchesAcceptedDoglegRatherThanGaussianBacksolve) {
  auto c=config();c.realtime.prior_sigmas.setConstant(10);NavigationState initial;
  initial.position_world_m={.01,.01,.01};
  IncrementalUwbImuEstimator e(c,Eigen::Vector3d::Zero());e.initialize(initial,c.realtime.prior_sigmas);
  gtsam::ISAM2Params params;params.relinearizeThreshold=c.incremental.relinearize_threshold;params.relinearizeSkip=c.incremental.relinearize_skip;
  params.enablePartialRelinearizationCheck=true;params.enableDetailedResults=true;params.optimizationParams=gtsam::ISAM2DoglegParams();
  gtsam::ISAM2 oracle(params);gtsam::NonlinearFactorGraph graph;gtsam::Values values;
  for(const auto& entry:e.factorLedger().entries())graph.push_back(entry.factor);
  values.insert(gtsam::Symbol('x',0),gtsam::Pose3(gtsam::Rot3(),initial.position_world_m));
  values.insert(gtsam::Symbol('v',0),gtsam::Vector3::Zero().eval());values.insert(gtsam::Symbol('b',0),gtsam::imuBias::ConstantBias());oracle.update(graph,values);
  feed(e,0,.1);auto batch=ranges(.1,{0,0,5});
  for(auto& m:batch.measurements) {
    m.anchor_position_m.z()=0;
    m.range_m=(Eigen::Vector3d(0,0,5)-m.anchor_position_m).norm();
    m.sigma_m=.01;
  }
  auto tx=e.prepareEpoch(batch,EpochPreparationOptions::nominalOnly());
  graph.resize(0);values.clear();for(const auto& f:tx.imu_group.factors)graph.push_back(f);
  for(const auto& group:tx.uwb_groups)if(group.nominal)for(const auto& f:group.factors)graph.push_back(f);
  values.insert(gtsam::Symbol('x',1),gtsam::Pose3(gtsam::Rot3(tx.cv_predicted_state.q_world_body.toRotationMatrix()),tx.cv_predicted_state.position_world_m));
  values.insert(gtsam::Symbol('v',1),tx.cv_predicted_state.velocity_world_mps);values.insert(gtsam::Symbol('b',1),gtsam::imuBias::ConstantBias());
  oracle.update(graph,values);const auto accepted=oracle.calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x',1));
  const auto gaussian=oracle.getFactorsUnsafe().linearize(oracle.getLinearizationPoint())->optimize();
  const auto full=oracle.getLinearizationPoint().at<gtsam::Pose3>(gtsam::Symbol('x',1)).retract(gaussian.at(gtsam::Symbol('x',1)));
  EXPECT_GT((full.translation()-accepted.translation()).norm(),.1);
  auto plan=EpochCommitPlan::nominalPlan(tx);e.commitEpoch(std::move(tx),plan);
  EXPECT_LT((e.currentState().position_world_m-accepted.translation()).norm(),1e-8);
  EXPECT_LT(e.currentState().q_world_body.angularDistance(Eigen::Quaterniond(accepted.rotation().matrix())),1e-8);
}
TEST(NominalAccuracy, WarmStartAddsNoInformationAndMaintainsOneUpdateTransaction) {
  for(const auto& tuning:{std::string(""),std::string("  nominal_initial_guess: imu\n  nominal_lm_iterations: 5\n")}) {
    auto c=config(tuning);IncrementalUwbImuEstimator e(c,Eigen::Vector3d::Zero());NavigationState s;e.initialize(s,c.realtime.prior_sigmas);feed(e,0,.1);
    auto tx=e.prepareEpoch(ranges(.1,{.1,.2,.1}),EpochPreparationOptions::nominalOnly());const auto before=e.backendUpdateCount();
    EXPECT_EQ(before,1u);EXPECT_EQ(e.factorCount(),3u);
    auto plan=EpochCommitPlan::nominalPlan(tx);std::size_t n=tx.imu_group.factors.size();for(const auto& g:tx.uwb_groups)if(g.nominal)n+=g.factors.size();
    e.commitEpoch(std::move(tx),plan);EXPECT_EQ(e.backendUpdateCount(),before+1);EXPECT_EQ(e.factorCount(),3+n);
    EXPECT_EQ(estimatorNumericsAuditV1(e).warm_start_attempts,tuning.empty()?0u:1u);
    feed(e,.11,.2);tx=e.prepareEpoch(ranges(.2,{.1,.2,.1}),EpochPreparationOptions::nominalOnly());
    DiscardReason reason;e.discardEpoch(std::move(tx),reason);EXPECT_EQ(e.backendUpdateCount(),before+1);
  }
}
TEST(NominalAccuracy, SecondsWindowRetainsPhysicalTimeRatherThanEpochCount) {
  auto c=config("  nominal_lag_s: 0.5\n");c.incremental.fixed_lag_epochs=100;
  IncrementalUwbImuEstimator e(c,Eigen::Vector3d::Zero());NavigationState s;e.initialize(s,c.realtime.prior_sigmas);feed(e,0,.1);
  double last=.1;
  for(double t:{.1,.2,.4,.8,1.1}) {
    if(last<t)feed(e,last+.01,t);
    auto tx=e.prepareEpoch(ranges(t,{0,0,1}),EpochPreparationOptions::nominalOnly());auto plan=EpochCommitPlan::nominalPlan(tx);e.commitEpoch(std::move(tx),plan);last=t;
  }
  EXPECT_EQ(e.retainedEpochs(),2u);EXPECT_EQ(e.backendUpdateCount(),6u);EXPECT_GT(e.marginalizationCount(),0u);
}
}  // namespace

TEST(NominalAccuracySparseEpoch, NewlyAddedImuCanBeMarginalizedInSameNominalUpdate) {
  using namespace uwb_imu_pl;
  auto c=config("  nominal_lag_s: 0.05\n");
  IncrementalUwbImuEstimator e(c,Eigen::Vector3d::Zero());NavigationState s;
  e.initialize(s,c.realtime.prior_sigmas);feed(e,0,.2);
  auto tx=e.prepareEpoch(ranges(.2,{0,0,1}),EpochPreparationOptions::nominalOnly());
  const auto imu_id=tx.imu_group.id;auto plan=EpochCommitPlan::nominalPlan(tx);
  EXPECT_NO_THROW(e.commitEpoch(std::move(tx),plan));EXPECT_EQ(e.backendUpdateCount(),2u);
  EXPECT_EQ(e.retainedEpochs(),1u);EXPECT_TRUE(e.currentState().position_world_m.allFinite());
  const auto records=e.factorLedger().groupEntries(imu_id);
  ASSERT_EQ(records.size(),1u);EXPECT_EQ(records.front().lifecycle,FactorLifecycle::Marginalized);
}

TEST(NominalAccuracyRobust, StandardizedScalarHuberIsExperimentalAndNeverFrozenForFde) {
  using namespace uwb_imu_pl;
  auto c=config("  nominal_robust_experimental: true\n");
  IncrementalUwbImuEstimator e(c,Eigen::Vector3d::Zero());NavigationState s;
  e.initialize(s,c.realtime.prior_sigmas);feed(e,0,.1);
  auto batch=ranges(.1,{0,0,1});
  auto tx=e.prepareEpoch(batch,EpochPreparationOptions::nominalOnly());
  for(const auto& group:tx.uwb_groups)if(group.nominal)
    for(const auto& factor:group.factors) {
      const auto nonlinear=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
      ASSERT_TRUE(nonlinear);
      const auto robust=boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(nonlinear->noiseModel());
      ASSERT_TRUE(robust);
      const auto huber=boost::dynamic_pointer_cast<gtsam::noiseModel::mEstimator::Huber>(robust->robust());
      ASSERT_TRUE(huber);
      gtsam::Vector residual(2);residual<<1,3;
      auto weights=robust->robust()->weight(residual);
      EXPECT_NEAR(weights(0),1,1e-12);EXPECT_NEAR(weights(1),.5,1e-12);
      gtsam::Values values;values.insert(gtsam::Symbol('x',1),gtsam::Pose3());
      double expected=0;
      for(const auto& m:batch.measurements)
        expected+=huber->loss((m.anchor_position_m.norm()-m.range_m)/m.sigma_m);
      EXPECT_NEAR(factor->error(values),expected,1e-9);
    }
  DiscardReason reason;e.discardEpoch(std::move(tx),reason);
  EXPECT_THROW(e.prepareEpoch(batch),std::invalid_argument);
}

TEST(NominalAccuracyFde, ExcludingCurrentImuUsesIndependentCvValuesAndBridgeSolution) {
  using namespace uwb_imu_pl;
  auto c=config();c.fde.profile=FdeProfile::JointOrder1;
  std::vector<gtsam::Values> solved;
  for(double acceleration:{-5.,5.}) {
    IncrementalUwbImuEstimator e(c,Eigen::Vector3d::Zero());NavigationState s;
    e.initialize(s,c.realtime.prior_sigmas);
    for(int i=0;i<=10;++i) {
      ImuMeasurement m;m.timestamp=TimestampNs::fromSeconds(.01*i);
      m.specific_force_mps2={acceleration,0,9.80665};m.angular_velocity_radps={0,0,acceleration};e.ingestImu(m);
    }
    auto tx=e.prepareEpoch(ranges(.1,{.1,.1,1}));
    ASSERT_TRUE(tx.frozen_values);
    EXPECT_LT(tx.frozen_values->at<gtsam::Pose3>(gtsam::Symbol('x',1)).translation().norm(),1e-12);
    EXPECT_LT(tx.frozen_values->at<gtsam::Vector3>(gtsam::Symbol('v',1)).norm(),1e-12);
    EXPECT_GT(tx.nominal_predicted_state.velocity_world_mps.norm(),.1);
    gtsam::NonlinearFactorGraph without_imu;
    for(const auto& entry:e.factorLedger().entries())without_imu.push_back(entry.factor);
    for(const auto& f:tx.generic_bridge_group.factors)without_imu.push_back(f);
    for(const auto& f:tx.generic_bias_continuity_group.factors)without_imu.push_back(f);
    for(const auto& group:tx.uwb_groups)if(group.nominal)for(const auto& f:group.factors)without_imu.push_back(f);
    solved.push_back(gtsam::LevenbergMarquardtOptimizer(without_imu,*tx.frozen_values).optimize());
    EXPECT_EQ(e.backendUpdateCount(),1u);DiscardReason reason;e.discardEpoch(std::move(tx),reason);EXPECT_EQ(e.backendUpdateCount(),1u);
  }
  const auto key=gtsam::Symbol('x',1);
  EXPECT_LT(gtsam::Pose3::Logmap(solved[0].at<gtsam::Pose3>(key).between(solved[1].at<gtsam::Pose3>(key))).norm(),1e-10);
  EXPECT_LT((solved[0].at<gtsam::Vector3>(gtsam::Symbol('v',1))-solved[1].at<gtsam::Vector3>(gtsam::Symbol('v',1))).norm(),1e-10);
}
