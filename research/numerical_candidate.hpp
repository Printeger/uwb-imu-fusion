#pragma once
#include "epoch_raim.hpp"
#include "bounded_range_time.hpp"
namespace uwb_imu_pl::research {
// Observation-only roots. Neither convergence nor this heuristic functional
// post-test inherits the old single-root Gaussian alpha/tail qualification.
struct NumericalCandidateResult {
 EpochRaim functional;
 Eigen::Vector3d squared_seed=Eigen::Vector3d::Constant(NAN);
 Eigen::Vector3d design_singular=Eigen::Vector3d::Zero(),jacobian_singular=design_singular;
 std::vector<Eigen::Vector3d> roots;
 std::vector<double> objectives;
 size_t attempted=0,converged=0,accepted_steps=0,rejected_steps=0;
 bool numeric_converged=false,functional_post_test=false,gaussian_reference_qualified=false;
 std::string seed_source="OBSERVATION_ONLY_ANCHOR_SVD_FIXED_BOTH_SIDES";
};
NumericalCandidateResult numericalUwbCandidate(const UwbBatch&,int excluded,double alpha=1e-6,const RangeTimeRadii* transport=nullptr);
}
