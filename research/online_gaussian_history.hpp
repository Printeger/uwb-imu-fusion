#pragma once
#include "gaussian_sufficient_summary.hpp"
#include "gaussian_window.hpp"
#include "hybrid_evidence.hpp"
#include <memory>
namespace uwb_imu_pl::research {
struct OnlinePreUwbEvidence {
 HybridHistoryEvidence history;
 NavigationState chart;
 Eigen::VectorXd correction;
 TransactionId transaction;
 BatchId excluded_batch;
 LinearizationVersion version;
 std::string domain="RAW_UNQUALIFIED";
};
struct OnlineHistoryCounts {
 size_t frontier_epoch=0,active_groups=0,active_states=0,history_rows=0,fault_modes=0;
 size_t source_columns=0,integrations=0,bytes=0,orthogonal_updates=0,state_decompositions=0,fault_products=0;
 size_t active_modes=0,leaf_svds=0,queries=0,query_columns=0,raw_rank=0,equal_raw_columns=0,zero_raw_columns=0,accepted_leaf_rank=0,full_fault_transforms=0,frontier_fault_transforms=0,frontier_kernels=0,candidate_hits=0;bool resource_refused=false;
 double source_ms=0.,retire_ms=0.,root_build_ms=0.,candidate_build_ms=0.,state_ms=0.,fault_ms=0.,query_state_qr_ms=0.,query_fault_qr_ms=0.;
 long long original_rows=0,eliminated_navigation_states=0;
};
// Research-only, fixed chart suffix and immutable frozen-prefix square root.
// Actual localization and commit remain the existing epoch transaction.
class OnlineGaussianHistory {
 public:
 OnlineGaussianHistory(size_t lag,int duration,double sa,double sg,Eigen::Vector3d lever,bool optimized=true,size_t max_numeric_bytes=64u*1024u*1024u,bool physical_sources=false);
 ~OnlineGaussianHistory();
 void prepare(const EpochTransaction& tx,bool build_current_root=true);
 SummaryEvaluation evaluate(const EpochTransaction& tx,const ExclusionAction* action,double alpha,double tail,bool faults=true)const;
 Eigen::MatrixXd tagMap()const;
 Eigen::Matrix<double,5,1> referenceSteps(const GaussianReference&)const;
 Eigen::VectorXd navigationMean(const GaussianReference&)const;
 // Optional diagnostic smoother: materializes O(history^2) covariance only on
 // request. Online detector/PL never calls it; it does not rebind protection.
 Eigen::MatrixXd navigationCovariance(const GaussianReference&)const;
 OnlineHistoryCounts counts()const;
 void committed(const EpochTransaction& tx,const EpochCommitPlan& plan);
 // Extraction only: remove ALL current UWB rows before square-root evaluation.
 // Includes pending IMU and shared ADC separators, but no current measurement.
 // No C candidate search or historical PL computation is invoked.
 OnlinePreUwbEvidence preUwbEvidence(const EpochTransaction& tx,double radius)const;
 private:
 struct Impl;std::unique_ptr<Impl> impl;
};
}
