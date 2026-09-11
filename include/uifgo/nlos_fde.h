#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "uifgo/config.h"
#include "uifgo/nlos_solver_utils.h"
#include "uifgo/nlos_support.h"
#include "uifgo/nlos_recoverability.h"
#include "uifgo/nlos_refit.h"
#include "uifgo/paper_input.h"
#include "uifgo/types.h"

namespace uifgo {

extern const char kImuAidedFdeProvider[];
extern const char kImuAidedFdeIdentityVersion[];

struct PreliminaryUwbResidual {
  size_t factor_index = 0;
  std::uint64_t obs_id = 0;
  double residual_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_m = std::numeric_limits<double>::quiet_NaN();
};

// Shared by fixed-rejection and the FDE Stage1. These functions deliberately
// read the scalar noise model attached to the physical factor rather than a
// ledger copy.
CheckedLmResult RunPreliminaryTightlyCoupledLm(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& initial_values, const CheckedLmOptions& options);
std::vector<PreliminaryUwbResidual> ReadScalarUwbFactorResiduals(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const std::vector<FactorMeta>& uwb_factor_metadata);

struct RawGaussianReference {
  gtsam::NonlinearFactorGraph graph;
  CheckedLmResult solve;
  std::string initial_identity;
  std::string final_identity;
  double objective_before = std::numeric_limits<double>::quiet_NaN();
  double objective_after = std::numeric_limits<double>::quiet_NaN();
};
RawGaussianReference PrepareRawGaussianReference(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const CheckedLmOptions& options);
void RequireRawGaussianReference(const RawGaussianReference& reference,
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial);

struct FdeNormalization {
  std::string linearization_identity;
  ResidualProjectionResult projection;
  std::vector<double> residual_variance_m2;
};
FdeNormalization ComputeFdeNormalization(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    const std::vector<FactorMeta>& metadata);

struct FdeOptions {
  double chi2_probability = 0.99;
  size_t chi2_degrees_of_freedom = 1;
  double gap_threshold_s = 1.0;
  size_t minimum_count = 2;
  double minimum_duration_s = 0.01;
  CheckedLmOptions preliminary_lm;
};

struct FdeObservationRecord {
  std::uint64_t obs_id = 0;
  size_t source_frame_index = 0;
  std::uint64_t source_message_index = 0;
  std::uint64_t source_range_index = 0;
  std::uint64_t source_observation_index = 0;
  int tag_id = 0;
  int anchor_id = 0;
  double sensor_time = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool planned = false;
  size_t keyframe_id = 0;
  size_t factor_index = std::numeric_limits<size_t>::max();
  double ledger_nominal_sigma_m = std::numeric_limits<double>::quiet_NaN();
  bool tested = false;
  double residual_m = std::numeric_limits<double>::quiet_NaN();
  double factor_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double residual_variance_m2 = std::numeric_limits<double>::quiet_NaN();
  double measurement_standardized_residual_diagnostic = std::numeric_limits<double>::quiet_NaN();
  std::string test_status = "NOT_TESTED";
  double standardized_residual = std::numeric_limits<double>::quiet_NaN();
  double statistic = std::numeric_limits<double>::quiet_NaN();
  bool fault_detected = false;
  bool positive_excess = false;
  bool nlos_candidate = false;
  std::string raw_segment_id;
  std::string segment_id;
  size_t segment_ordinal = std::numeric_limits<size_t>::max();
  std::string candidate_filter_reason = "NOT_TESTED";
};

enum class FdeStatus {
  SUCCESS,
  INVALID_INPUT,
  REFERENCE_LM_FAILED,
  OBSERVATION_TEST_FAILED,
  PARTITION_INVALID,
};

const char* FdeStatusName(FdeStatus status);

struct FdeContext {
  std::string input_plan_hash;
  std::string source_hash;
  std::string config_hash;
  std::string calibration_hash;
  std::string solver_config_hash;
  std::string common_preparation_id;
  std::string physical_graph_hash;
  std::string initial_values_hash;
};

struct FdeResult {
  FdeStatus status = FdeStatus::INVALID_INPUT;
  std::string reason;
  std::string provider = kImuAidedFdeProvider;
  std::string provider_version = kImuAidedFdeIdentityVersion;
  std::string identity_hash;
  double chi2_threshold = std::numeric_limits<double>::quiet_NaN();
  CheckedLmResult reference;
  RawGaussianReference raw_reference;
  FdeNormalization normalization;
  std::vector<FdeObservationRecord> observations;
  SupportPartition partition;
  size_t planned_count = 0;
  size_t tested_count = 0;
  size_t fault_count = 0;
  size_t positive_candidate_count = 0;
  size_t raw_run_count = 0;
  size_t filtered_run_count = 0;
  size_t retained_segment_count = 0;

  bool success() const { return status == FdeStatus::SUCCESS; }
};

// Pure residual classification and temporal aggregation entry points used by
// deterministic boundary tests. Classification is strict at the threshold.
void ClassifyFdeResidual(double residual_m, double factor_sigma_m,
                         double residual_variance_m2,
                         const FdeOptions& options,
                         FdeObservationRecord* record);
SupportPartition BuildFdeSupportPartition(
    std::vector<FdeObservationRecord>* observations,
    const FdeOptions& options, const FdeContext& context,
    size_t* raw_run_count = nullptr, size_t* filtered_run_count = nullptr);

std::string ComputeFdeIdentity(const FdeOptions& options,
                               const FdeContext& context);

SegmentRefitResult ReuseEmptyFdeReference(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial,
    const std::vector<FactorMeta>& metadata, const PaperInputPlan& plan,
    const Config& cfg, const FdeResult& fde, const RefitOptions& options);

class ImuAidedFdeSupportProvider {
 public:
  explicit ImuAidedFdeSupportProvider(FdeOptions options)
      : options_(std::move(options)) {}

  // Production input intentionally has no GT/oracle/support-label argument.
  FdeResult Run(const gtsam::NonlinearFactorGraph& base_graph,
                const gtsam::Values& base_values,
                const std::vector<FactorMeta>& base_uwb_factor_metadata,
                const PaperInputPlan& plan, const Config& cfg,
                const FdeContext& context,
                const RawGaussianReference* prepared = nullptr) const;

 private:
  FdeOptions options_;
};

}  // namespace uifgo
