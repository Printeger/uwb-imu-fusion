#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "uifgo/config.h"
#include "uifgo/paper_input.h"
#include "uifgo/pl_bidirectional_support.h"
#include "uifgo/types.h"

namespace uifgo {

inline constexpr const char* kPlBidirectionalProductionProvider =
    "PL_BIDIRECTIONAL_CUSUM_V1";
inline constexpr const char* kPlBidirectionalProductionVersion =
    "UIFGO_PL_BIDIRECTIONAL_CUSUM_PRODUCTION_V1";
inline constexpr const char* kPlBidirectionalForwardCalibrationHash =
    "sha256:02587e4441185c886bb558103484ef82f1a9b2ab1086101f6451613089529f63";
inline constexpr const char* kPlBidirectionalBackwardCalibrationHash =
    "sha256:a81f7cb1afd5f6518399c016b3779b2a0cce2a98c36650f002327800b8f6e201";
inline constexpr const char* kPlBidirectionalCleanSplitHash =
    "sha256:4611aedf457c119aace020d0ee417be28ad6c80c75a59186f7de43f708ab283f";
inline constexpr const char* kPlBidirectionalCleanInputHash =
    "sha256:f0b84cfeb578190b2440819f5d1256e96780889ae30c831c5279ab0255cb00b6";

struct PlBidirectionalProviderContext {
  std::string source_hash;
  std::string config_hash;
  std::string calibration_hash;
  std::string solver_config_hash;
  std::string common_preparation_id;
  std::string physical_graph_hash;
  std::string initial_values_hash;
};

struct PlBidirectionalProviderResult {
  bool valid = false;
  std::string status = "NOT_EVALUATED";
  std::string provider_identity;
  std::string detector_parameter_identity;
  std::string signal_identity;
  std::string support_identity;
  std::vector<PlCusumInputRow> signal_rows;
  PlCusumResult forward;
  PlBackwardCusumResult backward;
  PlBidirectionalSupportResult final_support;
  size_t committed_uwb_count = 0;
  size_t imu_gap_event_count = 0;
  double max_imu_gap_s = 0.0;
};

std::string PlBidirectionalParameterIdentity(const Config& cfg);
std::string PlBidirectionalProviderIdentity(
    const Config& cfg, const PaperInputPlan& plan,
    const PlBidirectionalProviderContext& context);

// Offline production Stage1.  It samples conditional_z immediately before
// the current raw UWB group is committed, commits every planned raw factor,
// then applies the admitted forward/backward cores and freezes their AND.
class PlBidirectionalCusumSupportProvider {
 public:
  PlBidirectionalProviderResult Run(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& initial_values,
      const std::vector<FactorMeta>& uwb_factor_metadata,
      const PaperInputPlan& plan, const std::vector<ImuSample>& imu,
      const Config& cfg,
      const PlBidirectionalProviderContext& context) const;
};

}  // namespace uifgo
