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
