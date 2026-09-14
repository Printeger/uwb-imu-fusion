#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "uifgo/config.h"
#include "uifgo/common_initializer.h"
#include "uifgo/initializer.h"
#include "uifgo/paper_input.h"
#include "uifgo/types.h"

namespace uifgo {

// Source-neutral preparation shared by paper applications. Dataset loading,
// CLI flags, experiment labels, cache protocols and artifact serialization are
// deliberately outside this boundary.
struct EstimatorCorePreparation {
  PaperInputPlan input_plan;
  std::vector<bool> selected_observation_mask;
  std::vector<UwbFrame> keyframes;
  InitResult initialization;
  CommonInitializationResult common_initialization;
  gtsam::NonlinearFactorGraph graph;
  // Deterministic GraphBuilder output retained as the initializer-independent
  // reference point for physical-graph contract identities.  The common
  // initializer receives the same graph and may only replace initial_values.
  gtsam::Values pre_common_initial_values;
  gtsam::Values initial_values;
  std::vector<size_t> uwb_factor_indices;
  std::vector<FactorMeta> factor_metadata;
};

EstimatorCorePreparation PrepareEstimatorCore(
    const Config& config, const std::vector<ImuSample>& imu,
    const std::vector<UwbFrame>& raw_uwb,
    const std::string& recording_id);

}  // namespace uifgo
