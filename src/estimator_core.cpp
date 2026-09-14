#include "uifgo/estimator_core.h"

#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "uifgo/graph_builder.h"
#include "uifgo/paper_pose_prior_factor.h"

namespace uifgo {

EstimatorCorePreparation PrepareEstimatorCore(
    const Config& config, const std::vector<ImuSample>& imu,
    const std::vector<UwbFrame>& raw_uwb,
    const std::string& recording_id) {
  EstimatorCorePreparation output;
  output.input_plan = BuildPaperInputPlan(raw_uwb, config, recording_id);
  if (output.input_plan.keyframes.size() < 2)
    throw std::runtime_error("paper input plan has fewer than two keyframes");

  output.selected_observation_mask =
      AllPlannedObservationMask(output.input_plan);
  output.keyframes = MaterializePaperKeyframes(
      output.input_plan, output.selected_observation_mask);
  output.initialization = Initializer(config).Run(imu, output.keyframes);
  if (!output.initialization.ok)
    throw std::runtime_error("initializer failed");

  GraphBuilder builder(config, config.paper_imu_covariance_model);
  builder.Build(output.keyframes, imu, output.initialization, &output.graph,
                &output.initial_values, &output.uwb_factor_indices);
  if (output.graph.empty() || output.uwb_factor_indices.empty())
    throw std::runtime_error("graph contains no UWB factors");

  const std::size_t paper_pose_priors =
      ReplacePosePriorsForPaperPath(&output.graph);
  if (paper_pose_priors != 1) {
    throw std::runtime_error(
        "paper path expected exactly one Pose3 prior, replaced " +
        std::to_string(paper_pose_priors));
  }
  output.pre_common_initial_values = output.initial_values;
  std::vector<double> state_times_s;
  state_times_s.reserve(output.keyframes.size());
  for (const auto& frame : output.keyframes) state_times_s.push_back(frame.t);
  output.common_initialization = CommonInitializer(config).Run(
      output.graph, output.initial_values, state_times_s,
      output.uwb_factor_indices);
  if (!output.common_initialization.ok) {
    std::ostringstream failure;
    failure << output.common_initialization.status << ':'
            << output.common_initialization.reason
            << ":evaluated_prefix_count="
            << output.common_initialization.prefixes.size()
            << ":accepted_prefix_count="
            << output.common_initialization.accepted_prefix_count
            << ":failure_count="
            << output.common_initialization.failure_count
            << ":retry_count=" << output.common_initialization.retry_count
            << ":runtime_s=" << std::setprecision(17)
            << output.common_initialization.runtime_s;
    throw std::runtime_error(failure.str());
  }
  output.initial_values = output.common_initialization.values;
  output.factor_metadata = builder.factor_meta();
  return output;
}

}  // namespace uifgo
