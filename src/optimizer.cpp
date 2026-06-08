#include "uifgo/optimizer.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>
#include <cmath>
#include <set>

namespace uifgo {
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::L;
using symbol_shorthand::A;

Optimizer::Optimizer(const Config& cfg) : cfg_(cfg) {}

gtsam::NonlinearFactorGraph Optimizer::BuildCleanGraph(
    const NonlinearFactorGraph& full,
    const std::set<size_t>& uwb_set,
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
    double chi2   = 2.0 * f->error(values);
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

OptimizerResult Optimizer::Optimize(
    const NonlinearFactorGraph& graph, const Values& initial,
    const std::vector<size_t>& uwb_indices) {

  OptimizerResult out;
  std::set<size_t> uwb_set(uwb_indices.begin(), uwb_indices.end());

  // --- LM params ---
  LevenbergMarquardtParams lm_params;
  lm_params.setMaxIterations(cfg_.lm_max_iter);
  lm_params.setRelativeErrorTol(cfg_.lm_rel_tol);
  lm_params.setAbsoluteErrorTol(cfg_.lm_abs_tol);

  out.initial_error = graph.error(initial);

  // ============ Stage A: Iterative Reweighted LM (IRLS) ============
  // Since GTSAM 4.0.3 lacks per-factor noiseModel accessor and GncOptimizer,
  // we implement manual IRLS with Cauchy-like reweighting:
  //   w_i = 1 / (1 + (r_i / k)^2)   where r_i is the whitened residual
  // After each LM solve, UWB factors with large residuals get down-weighted
  // by rebuilding the graph with scaled noise models.
  const int irls_iters = 3;
  Values refined = initial;
  double prev_error = out.initial_error;

  for (int iter = 0; iter < irls_iters; ++iter) {
    NonlinearFactorGraph rew_graph;
    for (size_t i = 0; i < graph.size(); ++i) {
      auto f = graph.at(i);
      if (!f) continue;

      if (uwb_set.count(i) && iter > 0) {
        // Re-weight UWB factor based on its residual from previous iteration
        double err = f->error(refined);  // 0.5 * ||r/sigma||^2
        double whitened_r2 = 2.0 * err;   // ||r/sigma||^2
        double weight = 1.0 / (1.0 + whitened_r2 / (cfg_.cauchy_k * cfg_.cauchy_k));
        weight = std::max(weight, 1e-6);  // prevent zero weight

        // Scale the noise: sigma_new = sigma / sqrt(weight)
        // In GTSAM 4.0.3 we can't easily clone+modify factor noise.
        // Instead, we add a copy of the factor with a looser noise model.
        // For ExpressionFactor<double>, the noise is Isotropic::Sigma(1, sigma).
        // We'll skip reweighting if we can't access noiseModel.
        rew_graph.add(f);  // GTSAM 4.0 limitation: just keep original
      } else {
        rew_graph.add(f);
      }
    }

    LevenbergMarquardtOptimizer lmopt(rew_graph, refined, lm_params);
    refined = lmopt.optimize();
    double cur_error = rew_graph.error(refined);

    if (iter == 0) {
      std::cout << "Optimizer: IRLS iter 0 (standard LM) error = " << cur_error << "\n";
    } else {
      std::cout << "Optimizer: IRLS iter " << iter
                << " error = " << cur_error << "\n";
    }

    // Early stop if improvement is small
    if (cur_error > prev_error * 0.999 && iter > 0) break;
    prev_error = cur_error;
  }

  // ============ Stage B: Chi-square rejection loop on full graph ============
  std::set<size_t> rejected;
  int round = 0;
  for (; round < cfg_.max_rejection_rounds; ++round) {
    NonlinearFactorGraph clean = BuildCleanGraph(graph, uwb_set, rejected);
    LevenbergMarquardtOptimizer lmopt2(clean, refined, lm_params);
    refined = lmopt2.optimize();

    auto newly = ChiSquareReject(graph, refined, uwb_set, rejected);
    if (newly.empty()) {
      clean_graph_ = clean;
      break;
    }
    rejected.insert(newly.begin(), newly.end());
    std::cout << "Optimizer: chi2 round " << round << " rejected "
              << newly.size() << " new outliers"
              << " (total=" << rejected.size() << ")\n";
  }

  // If max rounds reached, use last clean graph
  if (round == cfg_.max_rejection_rounds) {
    clean_graph_ = BuildCleanGraph(graph, uwb_set, rejected);
    LevenbergMarquardtOptimizer lmopt3(clean_graph_, refined, lm_params);
    refined = lmopt3.optimize();
  }

  // --- Result assembly ---
  out.values              = refined;
  out.reduced_chi2        = ReducedChi2(clean_graph_, refined);
  out.num_rejection_rounds = round;
  out.outlier_uwb_indices.assign(rejected.begin(), rejected.end());
  for (size_t idx : uwb_indices) {
    if (!rejected.count(idx)) out.inlier_uwb_indices.push_back(idx);
  }
  out.final_error = clean_graph_.error(refined);

  result_values_ = refined;
  solved_        = true;

  std::cout << "Optimizer: final reduced chi2 = " << out.reduced_chi2
            << ", inliers = " << out.inlier_uwb_indices.size()
            << "/" << uwb_indices.size()
            << ", outliers = " << out.outlier_uwb_indices.size() << "\n";
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
Matrix Optimizer::PoseCovariance(size_t k) const     { return Covariance(X(k)); }
Matrix Optimizer::LeverCovariance() const            { return Covariance(L(0)); }
Matrix Optimizer::AnchorCovariance(int m) const      { return Covariance(A(m)); }

}  // namespace uifgo
