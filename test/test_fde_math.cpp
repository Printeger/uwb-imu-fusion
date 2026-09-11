#include <gtest/gtest.h>
#include <Eigen/SVD>
#include <Eigen/LU>
#include "uifgo/fde_math.h"
TEST(FdeMath, QuantileAndStrictBoundary){
 EXPECT_NEAR(uifgo::FdeChiSquareQuantile(1),6.6348966010212145,1e-12);
 EXPECT_NEAR(uifgo::FdeChiSquareQuantile(2),9.21034037197618,1e-12);
 EXPECT_THROW(uifgo::FdeChiSquareQuantile(0),std::invalid_argument);
 Eigen::VectorXd unit=Eigen::VectorXd::Ones(1);
 Eigen::MatrixXd boundary(1,1);boundary(0,0)=1/uifgo::FdeChiSquareQuantile(1);
 auto equal=uifgo::FdeCovarianceTest(unit,boundary);
 ASSERT_DOUBLE_EQ(equal.statistic,equal.threshold);EXPECT_FALSE(equal.rejected);
 boundary(0,0)=std::nextafter(boundary(0,0),0.);
 EXPECT_TRUE(uifgo::FdeCovarianceTest(unit,boundary).rejected);

 Eigen::VectorXd e(2);e<<1,2;Eigen::MatrixXd c=Eigen::MatrixXd::Identity(2,2)*5/uifgo::FdeChiSquareQuantile(2);
 auto t=uifgo::FdeCovarianceTest(e,c);EXPECT_TRUE(t.valid);EXPECT_NEAR(t.statistic,t.threshold,1e-12);
 e*=1.00001;EXPECT_TRUE(uifgo::FdeCovarianceTest(e,c).rejected);
 e*=.999;EXPECT_FALSE(uifgo::FdeCovarianceTest(e,c).rejected);
}
TEST(FdeMath, FullCorrelatedWindowNotDiagonalSum){
 Eigen::VectorXd e(2);e<<1,1;Eigen::MatrixXd c(2,2);c<<1,.5,.5,1;
 auto t=uifgo::FdeCovarianceTest(e,c);ASSERT_TRUE(t.valid);EXPECT_NEAR(t.statistic,4./3,1e-12);EXPECT_NE(t.statistic,2.);
 c<<1,1,1,1;t=uifgo::FdeCovarianceTest(e,c);ASSERT_TRUE(t.valid);EXPECT_EQ(t.rank,1);EXPECT_NEAR(t.statistic,1,1e-12);
 c.setZero();EXPECT_FALSE(uifgo::FdeCovarianceTest(e,c).valid);
 c<<1,2,2,1;EXPECT_FALSE(uifgo::FdeCovarianceTest(e,c).valid);
}
TEST(FdeMath, HeteroscedasticProjectorAndCorrelatedSeparation){
 Eigen::MatrixXd A(4,1);A<<1,2,1,3;Eigen::VectorXd e(4);e<<1,-1,.5,2;
 auto b=uifgo::FdeFullProjectionBlock(A.sparseView(),e,{0,1});ASSERT_TRUE(b.certificate.valid);
 Eigen::MatrixXd P=Eigen::MatrixXd::Identity(4,4)-A*A.transpose()/15.;
 EXPECT_TRUE(b.Pww.isApprox(P.topLeftCorner(2,2),1e-12));
 EXPECT_TRUE(b.projected_residual.isApprox(P*e,1e-12));
 Eigen::MatrixXd K0=A.transpose()/15.;Eigen::MatrixXd Kh(1,4);Kh<<0,0,.1,.3;
 Eigen::MatrixXd D=Kh-K0;
 Eigen::MatrixXd C=b.response*b.Pww.inverse()*b.response.transpose();
 EXPECT_TRUE(C.isApprox(D*D.transpose(),1e-12));
 EXPECT_FALSE(C.isApprox(Kh*Kh.transpose()+K0*K0.transpose(),1e-12));
 // Meter covariance is diag(sigma) P diag(sigma), not sigma P.
 Eigen::Vector2d sigma(2,3);Eigen::Matrix2d Q=sigma.asDiagonal()*b.Pww*sigma.asDiagonal();
 EXPECT_NEAR(Q(0,1),6*P(0,1),1e-12);
}
TEST(FdeMath, ZeroRedundancyAndNearDegenerateFail){
 Eigen::MatrixXd A=Eigen::MatrixXd::Identity(2,2);Eigen::VectorXd e=Eigen::VectorXd::Ones(2);
 EXPECT_FALSE(uifgo::FdeFullProjectionBlock(A.sparseView(),e,{0}).certificate.valid);
 A.resize(3,2);A<<1,1,0,1.5e-10,0,0;e=Eigen::VectorXd::Ones(3);
 EXPECT_FALSE(uifgo::FdeFullProjectionBlock(A.sparseView(),e,{2}).certificate.valid);
}
int main(int argc,char**argv){::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}

#include "uifgo/nlos_fde.h"
namespace {
std::vector<uifgo::FdeObservationRecord> GroupRows(int links) {
 std::vector<uifgo::FdeObservationRecord> rows;
 for(int k=0;k<links;++k)for(int j=0;j<8;++j) {
  uifgo::FdeObservationRecord r;r.obs_id=1+8*k+j;r.tag_id=1;r.anchor_id=k;
  r.valid=r.planned=true;r.sensor_time=j;r.factor_sigma_m=1;
  uifgo::ClassifyFdeResidual(-1.8,1,1,uifgo::FdeOptions(),&r);
  rows.push_back(r);
 }
 return rows;
}
std::vector<uint64_t> Ids(const std::vector<uifgo::FdeObservationRecord>& r){
 std::vector<uint64_t> ids;for(const auto&x:r)ids.push_back(x.obs_id);return ids;
}
}
TEST(FdeGrouped, AggregatePromotesUniqueLinkWithoutPointFault){
 auto rows=GroupRows(1);auto ids=Ids(rows);std::reverse(rows.begin(),rows.end());
 uifgo::FdeOptions o;o.grouped_test=true;std::string status;
 auto tests=uifgo::ApplyGroupedFdeTests(&rows,Eigen::MatrixXd::Identity(8,8),ids,o,&status);
 ASSERT_EQ(tests.size(),1);EXPECT_TRUE(tests[0].test.rejected);EXPECT_EQ(status,"GROUP_UNIQUE_POSITIVE_LINK");
 for(const auto&r:rows){EXPECT_FALSE(r.fault_detected);EXPECT_TRUE(r.nlos_candidate);}
 uifgo::FdeContext c;c.input_plan_hash=c.source_hash=c.config_hash=c.calibration_hash=c.solver_config_hash=c.common_preparation_id=c.physical_graph_hash=c.initial_values_hash="test";
 auto partition=uifgo::BuildFdeSupportPartition(&rows,o,c);ASSERT_EQ(partition.segments.size(),1);EXPECT_EQ(partition.provider,"imu_aided_grouped_fde_v3");
 auto grouped=uifgo::ComputeFdeIdentity(o,c);o.grouped_test=false;EXPECT_NE(grouped,uifgo::ComputeFdeIdentity(o,c));
}
TEST(FdeGrouped, MultipleLinksRemainAmbiguous){
 auto rows=GroupRows(2);std::string status;uifgo::FdeOptions o;o.grouped_test=true;
 auto tests=uifgo::ApplyGroupedFdeTests(&rows,Eigen::MatrixXd::Identity(16,16),Ids(rows),o,&status);
 EXPECT_EQ(status,"FDE_ISOLATION_AMBIGUOUS");ASSERT_EQ(tests.size(),2);
 for(const auto&r:rows)EXPECT_FALSE(r.nlos_candidate);
}
TEST(FdeGrouped, OriginalGapCountDurationAndSign){
 auto rows=GroupRows(1);auto ids=Ids(rows);std::string status;uifgo::FdeOptions o;o.grouped_test=true;
 o.gap_threshold_s=.99;
 auto tests=uifgo::ApplyGroupedFdeTests(&rows,Eigen::MatrixXd::Identity(8,8),ids,o,&status);
 EXPECT_EQ(tests.size(),8);for(const auto&t:tests)EXPECT_EQ(t.status,"FILTERED_ORIGINAL_COUNT_DURATION");
 o.gap_threshold_s=1;o.minimum_duration_s=7.01;
 tests=uifgo::ApplyGroupedFdeTests(&rows,Eigen::MatrixXd::Identity(8,8),ids,o,&status);EXPECT_EQ(tests[0].status,"FILTERED_ORIGINAL_COUNT_DURATION");
 o.minimum_duration_s=7;for(auto&r:rows)r.residual_m=1.8;
 tests=uifgo::ApplyGroupedFdeTests(&rows,Eigen::MatrixXd::Identity(8,8),ids,o,&status);EXPECT_TRUE(tests[0].test.rejected);EXPECT_FALSE(tests[0].positive_excess);EXPECT_EQ(status,"GROUP_CONSISTENT_NO_SUPPORT");
}
TEST(FdeGrouped, RejectMappingAndUnresolvedCovariance){
 auto rows=GroupRows(1);auto ids=Ids(rows);std::string status;uifgo::FdeOptions o;o.grouped_test=true;
 EXPECT_THROW(uifgo::ApplyGroupedFdeTests(&rows,Eigen::MatrixXd::Zero(8,8),ids,o,&status),std::runtime_error);
 ids[0]=ids[1];EXPECT_THROW(uifgo::ApplyGroupedFdeTests(&rows,Eigen::MatrixXd::Identity(8,8),ids,o,&status),std::invalid_argument);
}

namespace {
uifgo::FdeContext WindowContext(const std::string& suffix = "base") {
 uifgo::FdeContext c;
 c.input_plan_hash=c.source_hash=c.config_hash=c.calibration_hash=
     c.solver_config_hash=c.common_preparation_id=c.physical_graph_hash=
     c.initial_values_hash=suffix;
 return c;
}
uifgo::FdeOptions WindowOptions() {
 uifgo::FdeOptions o;o.windowed_test=true;o.gap_threshold_s=1.;
 o.minimum_count=2;o.minimum_duration_s=.01;return o;
}
std::vector<uifgo::FdeObservationRecord> WindowRows(
    size_t count, const std::vector<double>& residuals, int anchor=1,
    uint64_t first_id=1) {
 std::vector<uifgo::FdeObservationRecord> rows;
 for(size_t j=0;j<count;++j) {
  uifgo::FdeObservationRecord r;r.obs_id=first_id+j;r.tag_id=7;r.anchor_id=anchor;
  r.valid=r.planned=true;r.sensor_time=static_cast<double>(j);r.factor_sigma_m=1.;
  uifgo::ClassifyFdeResidual(residuals.at(j),1.,1.,uifgo::FdeOptions(),&r);
  rows.push_back(r);
 }
 return rows;
}
std::vector<uint64_t> WindowIds(const std::vector<uifgo::FdeObservationRecord>& rows) {
 std::vector<uint64_t> ids;for(const auto& row:rows)ids.push_back(row.obs_id);return ids;
}
uifgo::FdeResult RunWindowed(std::vector<uifgo::FdeObservationRecord>* rows,
                            const Eigen::MatrixXd& covariance,
                            const uifgo::FdeContext& context=WindowContext()) {
 auto o=WindowOptions();uifgo::FdeResult result;
 result.identity_hash=uifgo::ComputeFdeIdentity(o,context);
 uifgo::ApplyWindowedFdeTests(rows,covariance,WindowIds(*rows),o,context,&result);
 return result;
}
}

TEST(FdeWindowed, DilutionFindsInternalEightPointFaultMissedByWholeChain) {
 std::vector<double> residuals(128,0.);for(size_t j=60;j<68;++j)residuals[j]=-2.3;
 auto grouped_rows=WindowRows(128,residuals);auto ids=WindowIds(grouped_rows);
 auto grouped=WindowOptions();grouped.windowed_test=false;grouped.grouped_test=true;
 std::string grouped_status;
 const auto whole=uifgo::ApplyGroupedFdeTests(
     &grouped_rows,Eigen::MatrixXd::Identity(128,128),ids,grouped,&grouped_status);
 ASSERT_EQ(whole.size(),1u);EXPECT_FALSE(whole[0].test.rejected);
 for(const auto& row:grouped_rows)EXPECT_FALSE(row.fault_detected);
 auto rows=WindowRows(128,residuals);
 const auto result=RunWindowed(&rows,Eigen::MatrixXd::Identity(128,128));
 EXPECT_EQ(result.covariance_window_count,119u);
 EXPECT_GT(result.significant_window_count,0u);
 ASSERT_EQ(result.partition.segments.size(),1u);
}

TEST(FdeWindowed, CleanChainHasNoSignificantWindowOrSegment) {
 auto rows=WindowRows(32,std::vector<double>(32,0.));
 const auto result=RunWindowed(&rows,Eigen::MatrixXd::Identity(32,32));
 EXPECT_GT(result.covariance_window_count,0u);
 EXPECT_EQ(result.significant_window_count,0u);
 EXPECT_TRUE(result.merged_segments.empty());
 EXPECT_TRUE(result.partition.segments.empty());
 EXPECT_EQ(result.windowed_status,"WINDOWED_CONSISTENT_NO_SUPPORT");
}

TEST(FdeWindowed, BoundaryWindowsMergeIntoOneExactUnionSegment) {
 std::vector<double> residuals(12,0.);for(size_t j=4;j<8;++j)residuals[j]=-3.;
 auto rows=WindowRows(12,residuals);
 const auto result=RunWindowed(&rows,Eigen::MatrixXd::Identity(12,12));
 EXPECT_GT(result.significant_window_count,1u);
 ASSERT_EQ(result.partition.segments.size(),1u);
 std::set<uint64_t> union_ids;
 for(const auto& window:result.local_windows)if(window.significant)
  union_ids.insert(window.obs_ids.begin(),window.obs_ids.end());
 EXPECT_EQ(result.partition.segments[0].obs_ids,
           (std::vector<uint64_t>(union_ids.begin(),union_ids.end())));
 for(const auto& window:result.local_windows)if(window.significant)
  EXPECT_EQ(window.merged_segment_id,result.partition.segments[0].segment_id);
}

TEST(FdeWindowed, MultiplicityTailAndAdjustedStrictBoundary) {
 auto rows=WindowRows(10,std::vector<double>(10,0.));
 auto result=RunWindowed(&rows,Eigen::MatrixXd::Identity(10,10));
 ASSERT_EQ(result.continuous_chains.size(),1u);
 EXPECT_EQ(result.continuous_chains[0].multiplicity,6u);
 for(const auto& window:result.local_windows) {
  EXPECT_EQ(window.multiplicity,6u);
  EXPECT_DOUBLE_EQ(window.adjusted_probability,1.-.01/6.);
  EXPECT_NEAR(window.adjusted_threshold,
              uifgo::FdeChiSquareQuantile(window.test.rank,1.-.01/6.),1e-14);
 }
 rows=WindowRows(11,std::vector<double>(11,0.));
 result=RunWindowed(&rows,Eigen::MatrixXd::Identity(11,11));
 EXPECT_EQ(result.covariance_window_count,7u);
 EXPECT_TRUE(std::any_of(result.local_windows.begin(),result.local_windows.end(),
     [](const auto& w){return w.obs_ids.size()==4 && w.first_chain_index==7;}));
 EXPECT_TRUE(std::any_of(result.local_windows.begin(),result.local_windows.end(),
     [](const auto& w){return w.obs_ids.size()==8 && w.first_chain_index==3;}));

 const double q=uifgo::FdeChiSquareQuantile(4,.99);
 std::vector<double> boundary_residual={1.,0.,0.,0.};
 rows=WindowRows(4,boundary_residual);
 Eigen::Matrix4d covariance=Eigen::Matrix4d::Identity();covariance(0,0)=1./q;
 result=RunWindowed(&rows,covariance);
 ASSERT_EQ(result.local_windows.size(),1u);
 EXPECT_DOUBLE_EQ(result.local_windows[0].test.statistic,
                  result.local_windows[0].adjusted_threshold);
 EXPECT_FALSE(result.local_windows[0].adjusted_rejected);
 rows=WindowRows(4,boundary_residual);
 covariance(0,0)=std::nextafter(covariance(0,0),0.);
 result=RunWindowed(&rows,covariance);
 EXPECT_TRUE(result.local_windows[0].adjusted_rejected);
}

TEST(FdeWindowed, CorrelatedCovarianceMatchesManualQuadraticAndGls) {
 auto rows=WindowRows(4,{-1.,-1.,0.,0.});
 Eigen::Matrix4d covariance=Eigen::Matrix4d::Identity();
 covariance(0,1)=covariance(1,0)=.5;
 const auto result=RunWindowed(&rows,covariance);
 ASSERT_EQ(result.local_windows.size(),1u);const auto& window=result.local_windows[0];
 Eigen::Vector4d e(-1.,-1.,0.,0.),d=Eigen::Vector4d::Ones();
 const Eigen::Matrix4d inverse=covariance.inverse();
 EXPECT_NEAR(window.test.statistic,(e.transpose()*inverse*e)(0,0),1e-12);
 EXPECT_NE(window.test.statistic,e.squaredNorm());
 EXPECT_NEAR(window.gls_signed_residual_m,
             (d.transpose()*inverse*e)(0,0)/(d.transpose()*inverse*d)(0,0),1e-12);
}

TEST(FdeWindowed, PositiveSignAmbiguityInvalidMappingAndIds) {
 auto positive=WindowRows(4,std::vector<double>(4,3.));
 auto result=RunWindowed(&positive,Eigen::Matrix4d::Identity());
 ASSERT_TRUE(result.local_windows[0].adjusted_rejected);
 EXPECT_FALSE(result.local_windows[0].positive_excess);
 EXPECT_TRUE(result.partition.segments.empty());

 auto first=WindowRows(4,std::vector<double>(4,-3.),1,1);
 auto second=WindowRows(4,std::vector<double>(4,-3.),2,101);
 first.insert(first.end(),second.begin(),second.end());
 result=RunWindowed(&first,Eigen::MatrixXd::Identity(8,8));
 EXPECT_EQ(result.windowed_status,"FDE_ISOLATION_AMBIGUOUS");
 EXPECT_TRUE(result.partition.segments.empty());
 for(const auto& row:first)EXPECT_FALSE(row.nlos_candidate);

 auto invalid=WindowRows(4,std::vector<double>(4,0.));auto o=WindowOptions();
 uifgo::FdeResult invalid_result;invalid_result.identity_hash=
     uifgo::ComputeFdeIdentity(o,WindowContext());auto ids=WindowIds(invalid);
 EXPECT_THROW(uifgo::ApplyWindowedFdeTests(&invalid,Eigen::Matrix4d::Zero(),ids,o,
              WindowContext(),&invalid_result),std::runtime_error);
 ids[0]=ids[1];
 EXPECT_THROW(uifgo::ApplyWindowedFdeTests(&invalid,Eigen::Matrix4d::Identity(),ids,o,
              WindowContext(),&invalid_result),std::invalid_argument);

 auto deterministic_a=WindowRows(8,std::vector<double>(8,-3.));
 auto deterministic_b=deterministic_a;
 const auto a=RunWindowed(&deterministic_a,Eigen::MatrixXd::Identity(8,8));
 const auto b=RunWindowed(&deterministic_b,Eigen::MatrixXd::Identity(8,8));
 ASSERT_EQ(a.local_windows.size(),b.local_windows.size());
 for(size_t j=0;j<a.local_windows.size();++j)
  EXPECT_EQ(a.local_windows[j].window_id,b.local_windows[j].window_id);
 ASSERT_EQ(a.partition.segments.size(),1u);ASSERT_EQ(b.partition.segments.size(),1u);
 EXPECT_EQ(a.partition.segments[0].segment_id,b.partition.segments[0].segment_id);
 auto changed=WindowRows(8,std::vector<double>(8,-3.));
 const auto c=RunWindowed(&changed,Eigen::MatrixXd::Identity(8,8),WindowContext("changed"));
 EXPECT_NE(a.local_windows[0].window_id,c.local_windows[0].window_id);
}
