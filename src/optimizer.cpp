#include "uifgo/optimizer.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

#include <cmath>
#include <iostream>
#include <set>

namespace uifgo {
using namespace gtsam;
using symbol_shorthand::A;
using symbol_shorthand::L;
using symbol_shorthand::X;

Optimizer::Optimizer(const Config& cfg) : cfg_(cfg) {}

gtsam::NonlinearFactorGraph Optimizer::BuildCleanGraph(
    const NonlinearFactorGraph& full, const std::set<size_t>& uwb_set,
    const std::set<size_t>& rejected) const {
  NonlinearFactorGraph clean;
  for (size_t i = 0; i < full.size(); ++i) {
    if (!full.at(i)) continue;
    if (uwb_set.count(i) && rejected.count(i)) continue;
    clean.add(full.at(i));
  }
  return clean;
}

std::vector<size_t> Optimizer::ChiSquareReject(
    const NonlinearFactorGraph& full, const Values& values,
    const std::set<size_t>& uwb_set,
    const std::set<size_t>& already_rejected) const {
  std::vector<size_t> newly;
  for (size_t i = 0; i < full.size(); ++i) {
    if (!uwb_set.count(i) || already_rejected.count(i)) continue;
    auto f = full.at(i);
    if (!f) continue;
    double chi2 = 2.0 * f->error(values);
    double thresh = Chi2inv(cfg_.chi2_reject_prob, (size_t)f->dim());
    if (chi2 > thresh) {
      newly.push_back(i);
    }
  }
  return newly;
}

double Optimizer::ReducedChi2(const NonlinearFactorGraph& g,
                              const Values& v) const {
  double chi2 = 2.0 * g.error(v);
  size_t m = 0;
  for (size_t i = 0; i < g.size(); ++i) {
    auto f = g.at(i);
    if (f) m += f->dim();
  }
  size_t n = v.dim();
  size_t nu = (m > n) ? (m - n) : 1;
  return chi2 / static_cast<double>(nu);
}

OptimizerResult Optimizer::Optimize(const NonlinearFactorGraph& graph,
                                    const Values& initial,
                                    const std::vector<size_t>& uwb_indices) {
  OptimizerResult out;
  std::set<size_t> uwb_set(uwb_indices.begin(), uwb_indices.end());

  // --- LM params (shared by GNC inner loop and clean refinement) ---
  LevenbergMarquardtParams lm_params;
  lm_params.setMaxIterations(cfg_.lm_max_iter);
  lm_params.setRelativeErrorTol(cfg_.lm_rel_tol);
  lm_params.setAbsoluteErrorTol(cfg_.lm_abs_tol);

  out.initial_error = graph.error(initial);
  std::cout << "  [Stage 0/3] Pre-warming LM... " << std::flush;

  // ============ Stage 0: Pre-warming LM (no robust weighting) ============
  // The initial IMU-predicted trajectory can have huge errors (10^9–10^11)
  // with zero-bias initialization. If GNC+TLS with setKnownInliers runs
  // directly on this, the IMU-dominated cost forces all UWB weights to
  // near-zero, causing catastrophic outlier rejection.
  //
  // A few standard LM iterations on the full graph reduce the initial error
  // by orders of magnitude, bringing the solution into a regime where GNC
  // can correctly distinguish NLOS outliers from inliers.
  LevenbergMarquardtParams prewarm_params = lm_params;
  prewarm_params.setMaxIterations(3);
  Values prewarmed = initial;
  {
    LevenbergMarquardtOptimizer preopt(graph, prewarmed, prewarm_params);
    prewarmed = preopt.optimize();
  }
  std::cout << "done (error " << out.initial_error << " -> "
            << graph.error(prewarmed) << ")\n";
  std::cout << "  [Stage 1/3] GNC+TLS annealing... " << std::flush;

  // ====================== Stage A: GNC + TLS ======================
  // Build known-inliers index vector: all non-UWB factors (IMU preintegration
  // and priors) are pinned as inliers so that GNC's robustness budget is
  // solely spent on identifying UWB NLOS outliers (design doc §15.1.1).
  std::vector<size_t> known_inliers;
  for (size_t i = 0; i < graph.size(); ++i)
    if (!uwb_set.count(i)) known_inliers.push_back(i);

  GncParams<LevenbergMarquardtParams> gncParams(lm_params);
  gncParams.setLossType(GncLossType::TLS);   // Truncated Least Squares
  gncParams.setKnownInliers(known_inliers);  // pin IMU/priors
  gncParams.setMuStep(cfg_.gnc_mu_step);
  gncParams.setMaxIterations(cfg_.gnc_max_iter);
  gncParams.setRelativeCostTol(cfg_.gnc_rel_cost_tol);

  GncOptimizer<GncParams<LevenbergMarquardtParams>> gnc(graph, prewarmed,
                                                        gncParams);
  // Auto-set per-factor inlier cost thresholds from chi2 quantile:
  //   barc^2 = 0.5 * chi2inv(gnc_inlier_prob, dim)
  gnc.setInlierCostThresholdsAtProbability(cfg_.gnc_inlier_prob);
  Values gnc_result = gnc.optimize();
  out.gnc_weights = gnc.getWeights();  // size = graph.size(), inliers ~1

  std::cout << "done (weights [" << out.gnc_weights.minCoeff() << ", "
            << out.gnc_weights.maxCoeff() << "])\n";
  std::cout << "  [Stage 2/3] chi2 rejection + clean LM refine... "
            << std::flush;

  // --- Hard-reject UWB factors with GNC weight below threshold ---
  std::set<size_t> rejected;
  for (size_t idx : uwb_indices)
    if (out.gnc_weights(idx) < cfg_.gnc_weight_thresh) rejected.insert(idx);

  // ============ Stage B: Chi-square rejection loop on clean graph ============
  // The clean graph removes GNC-rejected factors; the follow-up chi2 loop
  // catches "grey-zone" outliers (w ~0.3-0.7) that GNC didn't fully reject.
  Values refined = gnc_result;
  int round = 0;
  for (; round < cfg_.max_rejection_rounds; ++round) {
    NonlinearFactorGraph clean = BuildCleanGraph(graph, uwb_set, rejected);
    LevenbergMarquardtOptimizer lmopt(clean, refined, lm_params);
    refined = lmopt.optimize();

    auto newly = ChiSquareReject(graph, refined, uwb_set, rejected);
    if (newly.empty()) {
      clean_graph_ = clean;
      break;
    }
    rejected.insert(newly.begin(), newly.end());
  }

  // If max rounds reached, build final clean graph and do one last LM refine
  if (round == cfg_.max_rejection_rounds) {
    clean_graph_ = BuildCleanGraph(graph, uwb_set, rejected);
    LevenbergMarquardtOptimizer lmopt(clean_graph_, refined, lm_params);
    refined = lmopt.optimize();
  }

  // --- Result assembly ---
  out.values = refined;
  out.reduced_chi2 = ReducedChi2(clean_graph_, refined);
  out.num_rejection_rounds = round;
  out.outlier_uwb_indices.assign(rejected.begin(), rejected.end());
  for (size_t idx : uwb_indices) {
    if (!rejected.count(idx)) out.inlier_uwb_indices.push_back(idx);
  }
  out.final_error = clean_graph_.error(refined);

  result_values_ = refined;
  solved_ = true;

  std::cout << "done (chi2=" << out.reduced_chi2
            << ", inliers=" << out.inlier_uwb_indices.size() << "/"
            << uwb_indices.size()
            << ", outliers=" << out.outlier_uwb_indices.size()
            << ", #reject-rounds=" << out.num_rejection_rounds << ")\n";
  return out;
}

// ============ Stage C: Marginals ============
Matrix Optimizer::Covariance(Key key) const {
  if (!solved_)
    throw std::runtime_error("Optimizer::Covariance requires Optimize first");
  try {
    Marginals marg(clean_graph_, result_values_, Marginals::CHOLESKY);
    return marg.marginalCovariance(key);
  } catch (const std::exception& e) {
    std::cerr << "Marginals failed for key " << key << ": " << e.what() << "\n";
    return Matrix();
  }
}
Matrix Optimizer::PoseCovariance(size_t k) const { return Covariance(X(k)); }
Matrix Optimizer::LeverCovariance() const { return Covariance(L(0)); }
Matrix Optimizer::AnchorCovariance(int m) const { return Covariance(A(m)); }

}  // namespace uifgo
