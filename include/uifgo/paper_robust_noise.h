#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace uifgo {

// PAPER_STANDARD_ROBUST_LOSS_V2. The linked GTSAM Robust inherits Base's
// Mahalanobis distance, which calls robust-weighted whiten(). Feeding that
// distance to loss() disagrees with its IRLS Jacobian. Use the underlying
// Gaussian distance for the objective; retain native robust WhitenSystem.
// This subclass is used only by the paper baseline graph, never legacy.
class PaperRobustNoise final : public gtsam::noiseModel::Robust {
 public:
  PaperRobustNoise(
      const gtsam::noiseModel::mEstimator::Base::shared_ptr& kernel,
      const gtsam::SharedNoiseModel& gaussian)
      : gtsam::noiseModel::Robust(kernel, gaussian) {}

  double squaredMahalanobisDistance(const gtsam::Vector& residual) const override {
    return noise()->squaredMahalanobisDistance(residual);
  }
};

}  // namespace uifgo
