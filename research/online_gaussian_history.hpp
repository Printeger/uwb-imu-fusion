#pragma once
#include "gaussian_sufficient_summary.hpp"
#include "gaussian_window.hpp"
#include <memory>
namespace uwb_imu_pl::research {
struct OnlineHistoryCounts {
 size_t frontier_epoch=0,active_groups=0,active_states=0,history_rows=0,fault_modes=0;
 size_t source_columns=0,integrations=0,bytes=0,orthogonal_updates=0,state_decompositions=0,fault_products=0;
 long long original_rows=0,eliminated_navigation_states=0;
};
// Research-only, fixed chart suffix and immutable frozen-prefix square root.
// Actual localization and commit remain the existing epoch transaction.
class OnlineGaussianHistory {
 public:
 OnlineGaussianHistory(size_t lag,int duration,double sa,double sg,Eigen::Vector3d lever);
 ~OnlineGaussianHistory();
 void prepare(const EpochTransaction& tx);
 SummaryEvaluation evaluate(const EpochTransaction& tx,const ExclusionAction* action,double alpha,double tail,bool faults=true)const;
 Eigen::MatrixXd tagMap()const;
 Eigen::Matrix<double,5,1> referenceSteps(const GaussianReference&)const;
 Eigen::VectorXd navigationMean(const GaussianReference&)const;
 // Optional diagnostic smoother: materializes O(history^2) covariance only on
 // request. Online detector/PL never calls it; it does not rebind protection.
 Eigen::MatrixXd navigationCovariance(const GaussianReference&)const;
 OnlineHistoryCounts counts()const;
 void committed(const EpochTransaction& tx,const EpochCommitPlan& plan);
 private:
 struct Impl;std::unique_ptr<Impl> impl;
};
}
