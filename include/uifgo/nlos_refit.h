#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <set>
#include <map>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "uifgo/config.h"
#include "uifgo/nlos_support.h"
#include "uifgo/nlos_solver_utils.h"
#include "uifgo/paper_input.h"
#include "uifgo/types.h"

namespace uifgo {

constexpr const char* kT04OracleDebugLabel =
    "T04_ORACLE_SUPPORT_DEBUG_ONLY";
constexpr const char* kRq3FixedPartitionDebugLabel =
    "RQ3_FIXED_PARTITION_DIAGNOSTIC_DEBUG_ONLY";

// Strict debug-only manifest reader. It accepts support identity/intervals,
// never amplitudes, GT, poses, or initialization data.
class OracleSupportProvider {
 public:
  static OracleSupport Load(const std::string& yaml_path,
                            const PaperInputPlan& plan,
                            size_t short_min_count,
                            double short_min_duration);
};

struct RefitOptions {
  double boundary_epsilon_m = 1e-9;
  double relative_objective_tolerance = 1e-8;
  double scaled_step_tolerance = 1e-6;
  double projected_gradient_tolerance = 1e-8;
  // Maximum objective change per unit normalized free-navigation coordinate,
  // evaluated on the same final joint graph/Values after the exact
  // nonnegative c update.
  double navigation_stationarity_tolerance_objective = 1e-6;
  double gradient_roundoff_safety_factor = 8.0;
  size_t max_refit_iterations = 20;
  int lm_max_iterations = 100;
  double lm_relative_tolerance = 1e-6;
  double lm_absolute_tolerance = 1e-8;

  // Physical unit scales used by the dimensionless state-step check and by
  // the objective-per-normalized-coordinate navigation gradient check.
  double pose_rotation_scale_rad = 1.0;
  double pose_translation_scale_m = 1.0;
  double velocity_scale_mps = 1.0;
  double accel_bias_scale_mps2 = 1.0;
  double gyro_bias_scale_radps = 1.0;
  double segment_amplitude_scale_m = 1.0;
};

struct NonnegativeAmplitudeUpdate {
  bool valid = false;
  double numerator = 0.0;
  double denominator = 0.0;
  double amplitude_m = 0.0;
};

// Exact fixed-navigation block update used by SegmentRefitter and analytic
// tests. Entries are raw z, h+beta, and nominal sigma respectively.
NonnegativeAmplitudeUpdate ComputeNonnegativeSegmentAmplitude(
    const std::vector<double>& raw_ranges,
    const std::vector<double>& geometry_plus_fixed_beta,
    const std::vector<double>& nominal_sigmas);

struct RefitIteration {
  size_t outer_iteration = 0;
  size_t conditional_lm_iterations = 0;
  int conditional_lm_inner_iterations = 0;
  double conditional_lm_lambda = 0.0;
  double objective_before = 0.0;
  double objective_after = 0.0;
  double relative_objective_change = 0.0;
  double scaled_state_step = 0.0;
  double max_kkt_violation = 0.0;
  double max_pose_rotation_gradient_objective_per_rad = 0.0;
  double max_pose_translation_gradient_objective_per_m = 0.0;
  double max_velocity_gradient_objective_per_mps = 0.0;
  double max_accel_bias_gradient_objective_per_mps2 = 0.0;
  double max_gyro_bias_gradient_objective_per_radps = 0.0;
  double max_scaled_navigation_gradient_objective = 0.0;
  double navigation_gradient_roundoff_allowance_objective = 0.0;
  double navigation_stationarity_tolerance_objective = 0.0;
  double allowed_objective_increase = 0.0;
  bool objective_ok = false;
  bool step_ok = false;
  bool kkt_ok = false;
  bool navigation_stationarity_ok = false;
  bool conditional_inexact_handoff = false;
  std::string conditional_inner_status;
};

struct SegmentEstimate {
  std::string segment_id;
  size_t segment_ordinal = 0;
  gtsam::Key amplitude_key = 0;
  int tag_id = 0;
  int anchor_id = 0;
  size_t observation_count = 0;
  double start_time = 0.0;
  double end_time = 0.0;
  double duration = 0.0;
  double amplitude_m = 0.0;
  double gradient_objective_per_m = 0.0;
  double kkt_violation = 0.0;
  bool boundary = false;
  bool short_support_debug = false;
};

struct RefitFactorMeta {
  size_t factor_index = 0;
  std::uint64_t obs_id = 0;
  std::string factor_type;
  std::vector<gtsam::Key> keys;
  std::string segment_id;
};

// Explicit A19 development-only Stage-2 numerical-policy injection. The
// constants are captured where each conditional raw-range factor is built.
struct DevelopmentRefitRangeConstant {
  size_t factor_index = 0;
  gtsam::Key pose_key = 0;
  gtsam::Point3 anchor, lever;
  double measurement = 0.0;
  double sigma = 0.0;
  double conditional_beta = 0.0;
  std::uint64_t obs_id = 0;
  // A19-R01 audit fields.  They describe the exact inputs used to build the
  // corresponding conditional graph factor; they are diagnostics only and
  // never enter the objective or the certificate information matrix.
  bool candidate = false;
  double fixed_beta = 0.0;
  double segment_amplitude = 0.0;
  double expected_unwhitened_residual = 0.0;
};

struct DevelopmentStage2Request {
  std::string policy, implementation_identity, role;
  std::string validation_context_sha256;
  bool allow_inexact_handoff = false;
  std::function<CheckedLmResult(
      size_t, const gtsam::NonlinearFactorGraph&, const gtsam::Values&,
      const CheckedLmOptions&,
      const std::vector<DevelopmentRefitRangeConstant>&)>
      conditional_navigation;
  std::function<void(const RefitIteration&)> outer_observer;
};

enum class SegmentRefitStatus {
  CONVERGED,
  INVALID_INPUT,
  NONPOSITIVE_OR_NONFINITE_DENOMINATOR,
  CONDITIONAL_LM_FAILED,
  NONFINITE_VALUE,
  OBJECTIVE_INCREASE,
  GRAPH_VALUES_KEY_MISMATCH,
  MAX_REFIT_ITERATIONS,
};

const char* SegmentRefitStatusName(SegmentRefitStatus status);

// Owns the complete unregularized joint graph and the corresponding Values.
// Callers must export successful trajectory/amplitude/residual data only from
// this object.
struct SegmentRefitResult {
  SegmentRefitStatus status = SegmentRefitStatus::INVALID_INPUT;
  std::string reason;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::vector<RefitFactorMeta> factor_metadata;
  std::vector<SegmentEstimate> segments;
  std::vector<RefitIteration> iterations;
  std::vector<InexactHandoffAudit> inexact_handoffs;

  bool converged() const {
    return status == SegmentRefitStatus::CONVERGED;
  }
};

class SegmentRefitter {
 public:
  explicit SegmentRefitter(RefitOptions options) : options_(options) {}

  SegmentRefitResult Run(
      const gtsam::NonlinearFactorGraph& base_graph,
      const gtsam::Values& base_values,
      const std::vector<FactorMeta>& base_uwb_factor_metadata,
      const PaperInputPlan& plan, const Config& cfg,
      const SupportPartition& support) const;

  // No ordinary runner supplies this request. The result is development-only
  // and callers must keep it outside formal cache/final paths.
  SegmentRefitResult RunDevelopmentStage2(
      const gtsam::NonlinearFactorGraph& base_graph,
      const gtsam::Values& base_values,
      const std::vector<FactorMeta>& base_uwb_factor_metadata,
      const PaperInputPlan& plan, const Config& cfg,
      const SupportPartition& support,
      const DevelopmentStage2Request* development_request) const;

  SegmentRefitResult Run(
      const gtsam::NonlinearFactorGraph& base_graph,
      const gtsam::Values& base_values,
      const std::vector<FactorMeta>& base_uwb_factor_metadata,
      const PaperInputPlan& plan, const Config& cfg,
      const OracleSupport& support) const;

  // Stage-4 reconstruction from the original raw-factor graph.  The support
  // argument is always the complete frozen candidate set; only ordinals in
  // accepted_segment_ordinals receive a raw range factor with a live C key.
  // Every other candidate factor is absent, while every noncandidate raw
  // factor is retained.  This separate interface prevents an accepted-only
  // partition from silently reclassifying suppressed candidates as reference
  // observations.
  SegmentRefitResult RunFrozenCandidatePolicy(
      const gtsam::NonlinearFactorGraph& base_graph,
      const gtsam::Values& stage2_values,
      const std::vector<FactorMeta>& base_uwb_factor_metadata,
      const PaperInputPlan& plan, const Config& cfg,
      const SupportPartition& frozen_full_support,
      const std::set<size_t>& accepted_segment_ordinals,
      bool execute_optimization = true) const;

  // R03 development-only Stage4 adapter.  Formal/default callers cannot
  // reach the inexact-handoff policy and continue to use the method above.
  SegmentRefitResult RunDevelopmentFrozenCandidatePolicy(
      const gtsam::NonlinearFactorGraph& base_graph,
      const gtsam::Values& stage2_values,
      const std::vector<FactorMeta>& base_uwb_factor_metadata,
      const PaperInputPlan& plan, const Config& cfg,
      const SupportPartition& frozen_full_support,
      const std::set<size_t>& accepted_segment_ordinals,
      const DevelopmentStage2Request* development_request) const;

  SegmentRefitResult RunFixedOffsets(
      const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
      const std::vector<FactorMeta>& metadata, const PaperInputPlan& plan,
      const Config& cfg, const SupportPartition& support,
      const std::map<size_t,double>& offsets,
      const DevelopmentStage2Request* request = nullptr) const;

 private:
  SegmentRefitResult RunFrozenCandidatePolicyImpl(
      const gtsam::NonlinearFactorGraph& base_graph,
      const gtsam::Values& stage2_values,
      const std::vector<FactorMeta>& base_uwb_factor_metadata,
      const PaperInputPlan& plan, const Config& cfg,
      const SupportPartition& frozen_full_support,
      const std::set<size_t>& accepted_segment_ordinals,
      bool execute_optimization,
      const DevelopmentStage2Request* development_request,
      const std::map<size_t,double>* fixed_offsets = nullptr) const;
  RefitOptions options_;
};

}  // namespace uifgo
