#pragma once

#include "uifgo/config.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <vector>
#include <set>

namespace uifgo {

struct OptimizerResult {
  gtsam::Values        values;                  // final (clean refined) state
  std::vector<size_t>  inlier_uwb_indices;      // kept UWB factor indices
  std::vector<size_t>  outlier_uwb_indices;     // rejected UWB factor indices
  double               reduced_chi2 = 0.0;      // chi2/dof on clean graph
  int                  num_rejection_rounds = 0;
  double               initial_error = 0.0;
  double               final_error   = 0.0;
};

class Optimizer {
 public:
  explicit Optimizer(const Config& cfg);

  // Main entry: three-stage pipeline.
  //   Stage A: Cauchy-robust LM (soft weighting)
  //   Stage B: Chi-square rejection loop + clean LM refine
  //   Stage C: Marginals (covariance) on clean graph
  OptimizerResult Optimize(const gtsam::NonlinearFactorGraph& graph,
                           const gtsam::Values& initial,
                           const std::vector<size_t>& uwb_indices);

  // Covariance access (must call Optimize first).
  gtsam::Matrix Covariance(gtsam::Key key) const;
  gtsam::Matrix PoseCovariance(size_t k) const;
  gtsam::Matrix LeverCovariance() const;
  gtsam::Matrix AnchorCovariance(int m) const;

 private:
  // Build a clean graph (IMU + prior + inlier UWB factors).
  gtsam::NonlinearFactorGraph BuildCleanGraph(
      const gtsam::NonlinearFactorGraph& full,
      const std::set<size_t>& uwb_set,
      const std::set<size_t>& rejected) const;

  // Chi-square test on UWB factors; returns indices (in full graph) to reject.
  std::vector<size_t> ChiSquareReject(
      const gtsam::NonlinearFactorGraph& full,
      const gtsam::Values& values,
      const std::set<size_t>& uwb_set,
      const std::set<size_t>& already_rejected) const;

  double ReducedChi2(const gtsam::NonlinearFactorGraph& g,
                     const gtsam::Values& v) const;

  Config cfg_;
  gtsam::NonlinearFactorGraph clean_graph_;   // cached for Marginals
  gtsam::Values               result_values_;
  bool                        solved_ = false;
};

// Chi-square inverse CDF (p = confidence, dof = degrees of freedom).
double Chi2inv(double p, size_t dof);

}  // namespace uifgo
