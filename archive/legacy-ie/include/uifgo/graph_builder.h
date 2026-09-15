#pragma once

#include "uifgo/types.h"
#include "uifgo/config.h"
#include "uifgo/initializer.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <vector>

namespace uifgo {

class GraphBuilder {
 public:
  explicit GraphBuilder(const Config& cfg);

  // Build full factor graph + initial Values from keyframe'd data.
  //
  // Input:
  //   uwb_kf   : UWB frames (one per keyframe, pre-filtered)
  //   imu      : ALL IMU samples (used for preintegration)
  //   init     : initial state from Initializer
  //
  // Output:
  //   graph         : NonlinearFactorGraph ready for optimization
  //   values        : initial Values (linearization point)
  //   uwb_indices   : list of factor indices in graph that are UWB factors
  //                   (used by Optimizer for robust weighting)
  void Build(const std::vector<UwbFrame>& uwb_kf,
             const std::vector<ImuSample>& imu,
             const InitResult& init,
             gtsam::NonlinearFactorGraph* graph,
             gtsam::Values* values,
             std::vector<size_t>* uwb_indices);

  // Expose for testing
  const std::vector<AnchorConfig>& anchors() const { return anchors_; }

 private:
  // Add UWB factors for a single keyframe.
  void AddUwbFactorsForFrame(size_t kf_idx, const UwbFrame& frame,
                              gtsam::NonlinearFactorGraph* graph,
                              std::vector<size_t>* uwb_indices,
                              gtsam::Values* values,
                              bool first_frame);

  // Time-adaptive sigma computation.
  double AdaptiveSigma(double dt_since_last) const;

  Config cfg_;
  std::vector<AnchorConfig> anchors_;
  std::unordered_map<int, double> last_anchor_time_;  // anchor_id -> last seen t

  // constant keys
  gtsam::Key lever_key() const;
  gtsam::Key anchor_key(int m) const;
  gtsam::Key bias_key(int m) const;
};

}  // namespace uifgo
