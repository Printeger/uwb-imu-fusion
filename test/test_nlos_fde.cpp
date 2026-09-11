#include <gtest/gtest.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include "uifgo/imu_preint.h"
#include "uifgo/uwb_factor.h"

#include "uifgo/nlos_fde.h"
#include "uifgo/optimizer.h"

namespace uifgo {
namespace {

FdeOptions Options() {
  FdeOptions options;
  options.gap_threshold_s = 1.0;
  options.minimum_count = 2;
  options.minimum_duration_s = 0.5;
  options.preliminary_lm.max_iterations = 20;
  options.preliminary_lm.relative_tolerance = 1e-6;
  options.preliminary_lm.absolute_tolerance = 1e-8;
  return options;
}

FdeContext Context() {
  return {"plan", "source", "config", "calibration", "solver", "common",
          "graph", "values"};
}

FdeObservationRecord Row(std::uint64_t id, double time, double residual,
                         bool candidate = true, int anchor = 1) {
  FdeObservationRecord row;
  row.obs_id = id;
  row.tag_id = 0;
  row.anchor_id = anchor;
  row.sensor_time = time;
  row.valid = true;
  row.planned = true;
  row.tested = true;
  row.nlos_candidate = candidate;
  row.fault_detected = candidate;
  row.positive_excess = candidate;
  row.residual_m = residual;
  row.factor_sigma_m = 0.1;
  row.candidate_filter_reason = candidate ? "PENDING_TEMPORAL_FILTER"
                                          : "NOT_FAULT";
  return row;
}

TEST(NlosFde, ResidualDirectionAndStrictThresholdAreFrozen) {
  const auto options = Options();
  FdeObservationRecord negative;
  FdeObservationRecord positive;
  ClassifyFdeResidual(-0.4, 0.1, 0.01, options, &negative);
  ClassifyFdeResidual(0.4, 0.1, 0.01, options, &positive);
  EXPECT_TRUE(negative.fault_detected);
  EXPECT_TRUE(negative.positive_excess);
  EXPECT_TRUE(negative.nlos_candidate);
  EXPECT_TRUE(positive.fault_detected);
  EXPECT_FALSE(positive.positive_excess);
  EXPECT_FALSE(positive.nlos_candidate);

  const double boundary = std::sqrt(Chi2inv(0.99, 1));
  FdeObservationRecord equal;
  ClassifyFdeResidual(boundary, 1.0, 1.0, options, &equal);
  EXPECT_FALSE(equal.fault_detected);
  FdeObservationRecord above;
  ClassifyFdeResidual(std::nextafter(boundary, INFINITY), 1.0, 1.0, options,
                      &above);
  EXPECT_TRUE(above.fault_detected);
}

TEST(NlosFde, TemporalAggregationHonorsHealthStrictGapAndShortFiltering) {
  auto options = Options();
  std::vector<FdeObservationRecord> rows = {
      Row(1, 0.0, -0.4), Row(2, 0.5, -0.4),
      Row(3, 0.75, 0.0, false),
      Row(4, 1.0, -0.4), Row(5, 2.0, -0.4),
      Row(6, 3.01, -0.4)};
  size_t raw = 0;
  size_t filtered = 0;
  const auto partition = BuildFdeSupportPartition(
      &rows, options, Context(), &raw, &filtered);
  ASSERT_EQ(raw, 3u);
  EXPECT_EQ(filtered, 1u);
  ASSERT_EQ(partition.segments.size(), 2u);
  EXPECT_EQ(partition.segments[0].obs_ids,
            (std::vector<std::uint64_t>{1, 2}));
  EXPECT_EQ(partition.segments[1].obs_ids,
            (std::vector<std::uint64_t>{4, 5}));
  EXPECT_EQ(rows[5].candidate_filter_reason,
            "FILTERED_MIN_COUNT_AND_DURATION");
  EXPECT_TRUE(rows[5].nlos_candidate);
}

TEST(NlosFde, PersistentPositiveBiasFixtureProducesOneSegment) {
  auto options = Options();
  std::vector<FdeObservationRecord> rows = {
      Row(10, 4.0, -0.4), Row(11, 4.5, -0.6)};
  size_t raw = 0;
  size_t filtered = 0;
  const auto partition = BuildFdeSupportPartition(
      &rows, options, Context(), &raw, &filtered);
  ASSERT_EQ(partition.segments.size(), 1u);
  EXPECT_EQ(partition.segments[0].tag_id, 0);
  EXPECT_EQ(partition.segments[0].anchor_id, 1);
  EXPECT_DOUBLE_EQ(rows[0].residual_m, -0.4);
  EXPECT_DOUBLE_EQ(rows[1].residual_m, -0.6);
}

PaperInputPlan OneObservationPlan(double ledger_sigma) {
  PaperInputPlan plan;
  plan.plan_sha256 = "plan";
  ObservationRecord row;
  row.obs_id = 42;
  row.sensor_time = 1.0;
  row.valid = true;
  row.planned = true;
  row.nominal_sigma = ledger_sigma;
  row.anchor_id = 1;
  plan.observations.push_back(row);
  return plan;
}

TEST(NlosFde, ProviderUsesFactorNoiseAndEmptyCandidateIsSuccess) {
  const gtsam::Key key = gtsam::Symbol('z', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<double>(
      key, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 1e-3)));
  graph.add(gtsam::PriorFactor<double>(
      key, 3.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0)));
  gtsam::Values initial;
  initial.insert<double>(key, 0.0);
  FactorMeta meta;
  meta.factor_index = 1;
  meta.obs_id = 42;
  meta.factor_type = "uwb_range";
  meta.keys = {key};
  Config cfg;
  cfg.nlos_mode = "imu_aided_fde";
  cfg.calib_lever = false;
  cfg.calib_anchor = false;
  cfg.calib_range_bias = false;
  cfg.calib_td = false;
  auto options = Options();
  options.minimum_count = 1;
  options.minimum_duration_s = 0.0;
  const auto result = ImuAidedFdeSupportProvider(options).Run(
      graph, initial, {meta}, OneObservationPlan(1e-6), cfg, Context());
  ASSERT_TRUE(result.success()) << result.reason;
  ASSERT_EQ(result.observations.size(), 1u);
  EXPECT_DOUBLE_EQ(result.observations[0].factor_sigma_m, 1.0);
  EXPECT_GT(result.observations[0].statistic, Chi2inv(0.99, 1));
  EXPECT_TRUE(result.observations[0].nlos_candidate);

  graph[1] = boost::make_shared<gtsam::PriorFactor<double>>(
      key, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
  const auto empty = ImuAidedFdeSupportProvider(options).Run(
      graph, initial, {meta}, OneObservationPlan(1e-6), cfg, Context());
  ASSERT_TRUE(empty.success()) << empty.reason;
  EXPECT_TRUE(empty.partition.segments.empty());
  EXPECT_EQ(empty.reason, "REFERENCE_AND_ALL_TESTS_COMPLETE_EMPTY_SUPPORT");
}

TEST(NlosFde, ProviderRejectsIncompleteMappingAndIdentityIsSensitive) {
  const gtsam::Key key = gtsam::Symbol('z', 0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<double>(
      key, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0)));
  gtsam::Values initial;
  initial.insert<double>(key, 0.0);
  Config cfg;
  cfg.nlos_mode = "imu_aided_fde";
  cfg.calib_lever = false;
  cfg.calib_anchor = false;
  cfg.calib_range_bias = false;
  cfg.calib_td = false;
  const auto failed = ImuAidedFdeSupportProvider(Options()).Run(
      graph, initial, {}, OneObservationPlan(0.1), cfg, Context());
  EXPECT_EQ(failed.status, FdeStatus::INVALID_INPUT);

  const auto base = ComputeFdeIdentity(Options(), Context());
  auto options = Options();
  options.gap_threshold_s = 2.0;
  EXPECT_NE(base, ComputeFdeIdentity(options, Context()));
  options = Options();
  options.preliminary_lm.max_iterations++;
  EXPECT_NE(base, ComputeFdeIdentity(options, Context()));
  auto context = Context();
  context.calibration_hash = "changed";
  EXPECT_NE(base, ComputeFdeIdentity(Options(), context));
  context = Context();
  context.input_plan_hash = "changed";
  EXPECT_NE(base, ComputeFdeIdentity(Options(), context));
  context = Context();
  context.common_preparation_id = "changed";
  EXPECT_NE(base, ComputeFdeIdentity(Options(), context));
}

TEST(NlosFde, ProjectorMatchesSchurWithHeterogeneousNoiseAndFullNuisance) {
  Eigen::MatrixXd H(6, 3);
  H << 1,0,1, 0,1,1, 1,1,0, 2,-1,1, 0,2,1, 1,0,-1;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6, 6);
  covariance(0,0)=.2; covariance(1,1)=2.; covariance(2,2)=.7;
  covariance(0,1)=covariance(1,0)=.15;
  // The first block represents correlated inertial information; final scalar
  // rows represent independent ranges with nonuniform sigma.
  covariance(4,4)=.04; covariance(5,5)=.25;
  const Eigen::MatrixXd A = covariance.llt().matrixL().solve(H);
  const auto actual = ComputeSparseResidualProjectionDiagonal(A.sparseView(), {4,5});
  ASSERT_TRUE(actual.valid) << actual.reason;
  EXPECT_EQ(actual.qr_factorizations, 1u);
  const Eigen::MatrixXd P = (A.transpose()*A).ldlt().solve(Eigen::MatrixXd::Identity(3,3));
  const Eigen::MatrixXd Qr = covariance-H*P*H.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU);
  const Eigen::MatrixXd projector = Eigen::MatrixXd::Identity(6,6)-svd.matrixU()*svd.matrixU().transpose();
  for(size_t i=0;i<2;++i) {
    const int row=4+i;
    EXPECT_NEAR(actual.diagonal[i], projector(row,row), 1e-10);
    EXPECT_NEAR(covariance(row,row)*actual.diagonal[i], Qr(row,row), 1e-10);
  }
  const auto incomplete = ComputeSparseResidualProjectionDiagonal(A.leftCols(2).sparseView(), {4,5});
  ASSERT_TRUE(incomplete.valid);
  EXPECT_GT(std::abs(incomplete.diagonal[0]-actual.diagonal[0]), 1e-4);
  // Coordinate scaling and LM damping are absent from the physical projector.
  Eigen::MatrixXd scaled=A; scaled.col(0)*=1e5; scaled.col(1)*=1e-5;
  const auto invariant=ComputeSparseResidualProjectionDiagonal(scaled.sparseView(),{4,5});
  ASSERT_TRUE(invariant.valid) << invariant.reason;
  EXPECT_NEAR(invariant.diagonal[0],actual.diagonal[0],1e-10);
}

TEST(NlosFde, ProjectorHighLeverageZeroRedundancyAndRankUncertainty) {
  Eigen::MatrixXd A(2,1); A << 1.,1e-5;
  const auto high=ComputeSparseResidualProjectionDiagonal(A.sparseView(),{0});
  ASSERT_TRUE(high.valid) << high.reason;
  EXPECT_NEAR(high.diagonal[0],1e-10/(1.+1e-10),1e-20);
  const Eigen::MatrixXd identity=Eigen::MatrixXd::Identity(2,2);
  const auto zero=ComputeSparseResidualProjectionDiagonal(identity.sparseView(),{0});
  EXPECT_FALSE(zero.valid); EXPECT_TRUE(zero.diagonal.empty());
  Eigen::MatrixXd near(3,2); near << 1,1,0,1.5e-10,0,0;
  const auto uncertain=ComputeSparseResidualProjectionDiagonal(near.sparseView(),{2});
  EXPECT_FALSE(uncertain.valid); EXPECT_TRUE(uncertain.diagonal.empty());
  near(1,1)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ComputeSparseResidualProjectionDiagonal(near.sparseView(),{2}).valid);
  FdeObservationRecord row;
  EXPECT_THROW(ClassifyFdeResidual(-.1,.1,0.,Options(),&row),std::invalid_argument);
}

TEST(NlosFde, PreparedReferenceRejectsFailureGraphAndValuesMismatch) {
  const auto key=gtsam::Symbol('z',0);
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<double>(key,1.,gtsam::noiseModel::Isotropic::Sigma(1,.1)));
  gtsam::Values initial; initial.insert<double>(key,10.);
  const auto reference=PrepareRawGaussianReference(graph,initial,Options().preliminary_lm);
  ASSERT_TRUE(reference.solve.converged);
  EXPECT_NO_THROW(RequireRawGaussianReference(reference,graph,initial));
  auto failed=reference; failed.solve.converged=false;
  EXPECT_THROW(RequireRawGaussianReference(failed,graph,initial),std::invalid_argument);
  auto changed=graph;
  changed[0]=boost::make_shared<gtsam::PriorFactor<double>>(key,2.,gtsam::noiseModel::Isotropic::Sigma(1,.1));
  EXPECT_THROW(RequireRawGaussianReference(reference,changed,initial),std::invalid_argument);
  auto values=initial;values.update<double>(key,9.);
  EXPECT_THROW(RequireRawGaussianReference(reference,graph,values),std::invalid_argument);
}

class CoupledScalarRange : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  explicit CoupledScalarRange(gtsam::Key key)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(gtsam::noiseModel::Isotropic::Sigma(1,.2),key) {}
  gtsam::Vector evaluateError(const gtsam::Vector3& value,
      boost::optional<gtsam::Matrix&> H=boost::none) const override {
    if(H) *H=(gtsam::Matrix(1,3)<<1.,0.,1.).finished();
    return (gtsam::Vector(1)<<value[0]+value[2]-.5).finished();
  }
};

TEST(NlosFde, ActualGaussianWhiteningRetainsCrossCovarianceAndDampingIndependence) {
  const auto key=gtsam::Symbol('v',0);
  Eigen::Matrix3d Q;
  Q << .2,.03,.08, .03,.3,-.04, .08,-.04,.5;
  const gtsam::Vector3 zero=gtsam::Vector3::Zero();
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(key,zero,gtsam::noiseModel::Gaussian::Covariance(Q)));
  graph.add(boost::make_shared<CoupledScalarRange>(key));
  gtsam::Values values;values.insert(key,gtsam::Vector3(.1,.2,.3));
  FactorMeta meta;meta.factor_index=1;meta.obs_id=42;meta.factor_type="uwb_range";meta.keys={key};
  const auto actual=ComputeFdeNormalization(graph,values,{meta});
  ASSERT_TRUE(actual.projection.valid) << actual.projection.reason;
  Eigen::Vector3d h(1,0,1);
  const double expected=.04*.04/(.04+(h.transpose()*Q*h)[0]);
  EXPECT_NEAR(actual.residual_variance_m2[0],expected,1e-12);
  gtsam::LevenbergMarquardtParams low,high;
  low.setLinearSolverType("SEQUENTIAL_CHOLESKY"); high=low;
  low.setlambdaInitial(1e-5);high.setlambdaInitial(1e3);
  gtsam::LevenbergMarquardtOptimizer a(graph,values,low),b(graph,values,high);
  a.iterate();b.iterate();
  EXPECT_GT((a.values().at<gtsam::Vector3>(key)-b.values().at<gtsam::Vector3>(key)).norm(),1e-5);
  const auto same=ComputeFdeNormalization(graph,values,{meta});
  EXPECT_EQ(same.linearization_identity,actual.linearization_identity);
  EXPECT_EQ(same.residual_variance_m2,actual.residual_variance_m2);
  auto diagonal=graph;
  diagonal[0]=boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(key,zero,
      gtsam::noiseModel::Gaussian::Covariance(Q.diagonal().asDiagonal()));
  const auto wrong=ComputeFdeNormalization(diagonal,values,{meta});
  ASSERT_TRUE(wrong.projection.valid);
  EXPECT_GT(std::abs(wrong.residual_variance_m2[0]-expected),1e-4);
}

TEST(NlosFde, FullCombinedImuCovarianceMatchesDenseProjector) {
  using namespace gtsam::symbol_shorthand;
  Config cfg;
  ImuPreintegrator pim(cfg,gtsam::Vector3(0,0,-9.81));
  gtsam::imuBias::ConstantBias bias;
  pim.Reset(bias);
  for(int i=0;i<20;++i) pim.Integrate(gtsam::Vector3(.2,.1,9.81),gtsam::Vector3(.01,.02,.03),.01);
  const gtsam::Pose3 pose;
  const gtsam::Vector3 velocity=gtsam::Vector3::Zero();
  const auto predicted=pim.Pim().predict(gtsam::NavState(pose,velocity),bias);
  gtsam::NonlinearFactorGraph graph;
  graph.addPrior(X(0),pose,gtsam::noiseModel::Isotropic::Sigma(6,.1));
  graph.addPrior(V(0),velocity,gtsam::noiseModel::Isotropic::Sigma(3,.1));
  graph.addPrior(B(0),bias,gtsam::noiseModel::Isotropic::Sigma(6,.1));
  graph.add(gtsam::CombinedImuFactor(X(0),V(0),X(1),V(1),B(0),B(1),pim.Pim()));
  const auto range=MakeUwbFactor(X(1),0,0,0,gtsam::Point3(4,2,1),gtsam::Point3(0,0,0),4.,.2,false,false,false);
  graph.add(range);
  gtsam::Values values;
  values.insert(X(0),pose);values.insert(V(0),velocity);values.insert(B(0),bias);
  values.insert(X(1),predicted.pose());values.insert(V(1),predicted.velocity());values.insert(B(1),bias);
  FactorMeta meta;meta.factor_index=4;meta.obs_id=42;meta.factor_type="uwb_range";meta.keys={X(1)};
  const auto actual=ComputeFdeNormalization(graph,values,{meta});
  ASSERT_TRUE(actual.projection.valid) << actual.projection.reason;
  EXPECT_EQ(actual.projection.rank,30);
  const auto linear=graph.linearize(values);
  const Eigen::MatrixXd A=linear->jacobian().first;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A,Eigen::ComputeThinU);
  const double expected=.04*(1.-svd.matrixU().row(A.rows()-1).squaredNorm());
  EXPECT_NEAR(actual.residual_variance_m2[0],expected,1e-10);
  const auto covariance=pim.Pim().preintMeasCov();
  const gtsam::Matrix diagonal=covariance.diagonal().asDiagonal();
  EXPECT_GT((covariance-diagonal).norm(),1e-10);
  auto changed=graph;
  auto imu=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[3]);
  changed[3]=imu->cloneWithNewNoiseModel(gtsam::noiseModel::Gaussian::Covariance(diagonal));
  const auto wrong=ComputeFdeNormalization(changed,values,{meta});
  ASSERT_TRUE(wrong.projection.valid) << wrong.projection.reason;
  EXPECT_GT(std::abs(wrong.residual_variance_m2[0]-actual.residual_variance_m2[0]),1e-10);
}

}  // namespace
}  // namespace uifgo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
