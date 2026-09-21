#pragma once

#include "uwb_imu_pl/common/integrity_identity.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace uwb_imu_pl {

class TimestampNs {
 public:
  explicit constexpr TimestampNs(std::int64_t value = 0) : value_(value) {}
  static TimestampNs fromSeconds(double seconds);
  constexpr std::int64_t value() const { return value_; }
  double seconds() const;
  friend constexpr bool operator<(TimestampNs a, TimestampNs b) {
    return a.value_ < b.value_;
  }
  friend constexpr bool operator==(TimestampNs a, TimestampNs b) {
    return a.value_ == b.value_;
  }
  friend constexpr bool operator!=(TimestampNs a, TimestampNs b) {
    return !(a == b);
  }

 private:
  std::int64_t value_;
};

template <typename Tag>
class StrongId {
 public:
  constexpr StrongId() = default;
  explicit constexpr StrongId(std::uint64_t value) : value_(value) {}
  constexpr std::uint64_t value() const { return value_; }
  friend constexpr bool operator==(StrongId a, StrongId b) {
    return a.value_ == b.value_;
  }
  friend constexpr bool operator!=(StrongId a, StrongId b) {
    return !(a == b);
  }
  friend constexpr bool operator<(StrongId a, StrongId b) {
    return a.value_ < b.value_;
  }

 private:
  std::uint64_t value_ = 0;
};

struct AnchorIdTag {};
struct MeasurementIdTag {};
struct FactorIdTag {};
struct StateIdTag {};
struct HypothesisIdTag {};
struct BatchIdTag {};
using AnchorId = StrongId<AnchorIdTag>;
using MeasurementId = StrongId<MeasurementIdTag>;
using FactorId = StrongId<FactorIdTag>;
using StateId = StrongId<StateIdTag>;
using HypothesisId = StrongId<HypothesisIdTag>;
using BatchId = StrongId<BatchIdTag>;

enum class RowRole { Measurement, TrustedPrior, Regularizer };
enum class LinearizationConsistency { Strict, CachedChecked, CachedBlind };
enum class IntegrityLabel {
  FormalLocalCurrentFaultOnly,
  ImplementedUnverified,
  HeuristicDebug
};
enum class Availability { Available, Alert, Unavailable };

struct UwbMeasurement {
  MeasurementId id;
  FactorId factor_id;
  AnchorId anchor_id;
  TimestampNs timestamp;
  double range_m = 0.0;
  Eigen::Vector3d anchor_position_m = Eigen::Vector3d::Zero();
  double sigma_m = 0.0;
  std::uint64_t sequence = 0;
  std::uint32_t quality_flags = 0;
};

struct AnchorRecord {
  AnchorId id;
  Eigen::Vector3d position_world_m = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance_m2 = Eigen::Matrix3d::Zero();
  std::string frame;
  std::string map_version;
};

struct UwbBatch {
  BatchId id;
  TimestampNs timestamp;
  std::vector<UwbMeasurement> measurements;
  // Empty means diag(sigma_m^2). Otherwise it must be symmetric positive
  // definite and match measurements.size().
  Eigen::MatrixXd covariance_m2;
  std::string covariance_model_id;
};

struct ImuMeasurement {
  MeasurementId id;
  TimestampNs timestamp;
  Eigen::Vector3d specific_force_mps2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_radps = Eigen::Vector3d::Zero();
  std::uint32_t quality_flags = 0;
};

struct NavigationState {
  StateId id;
  TimestampNs timestamp;
  Eigen::Quaterniond q_world_body = Eigen::Quaterniond::Identity();
  Eigen::Vector3d position_world_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_mps2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_radps = Eigen::Vector3d::Zero();
};

struct FaultHypothesis {
  HypothesisId id;
  AnchorId anchor_id;
  std::vector<MeasurementId> affected_measurements;
  double prior_probability_bound = 0.0;
  double missed_detection_allocation = 0.0;
  bool monitored = true;
  std::string physical_fault_type = "single_anchor_range_bias";
  std::string pruning_reason;
};

struct RiskBudget {
  double p_fa = 0.0;
  double p_hmi_total = 0.0;
  double nominal_axis_tail = 0.0;
  double p_nm = 0.0;
  double horizontal_alert_limit_m = 0.0;
  double vertical_alert_limit_m = 0.0;
  std::vector<FaultHypothesis> hypotheses;
};

struct LinearizationDiagnostics {
  int rows = 0;
  int columns = 0;
  int rank = 0;
  double condition_number = std::numeric_limits<double>::infinity();
  double linearization_step_norm = std::numeric_limits<double>::infinity();
  bool covariance_valid = false;
  bool model_valid = false;
  std::string reason;
};

struct DetectorResult {
  std::string detector_type;
  double statistic = std::numeric_limits<double>::infinity();
  double threshold = 0.0;
  int dof = 0;
  double p_fa = 0.0;
  bool passed = false;
  bool numerically_valid = false;
  std::vector<double> local_anchor_scores;
  std::string reason;
};

struct SensitivityResult {
  HypothesisId hypothesis_id;
  AnchorId anchor_id;
  Eigen::Vector3d slope_xyz = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double detector_gram = 0.0;
  double noncentrality_boundary = 0.0;
  bool finite = false;
  std::string reason;
};

struct ProtectionLevelResult {
  TimestampNs timestamp;
  Eigen::Vector3d pl_xyz_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d nominal_component_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d fault_component_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double hpl_m = std::numeric_limits<double>::infinity();
  double vpl_m = std::numeric_limits<double>::infinity();
  double unmonitored_risk = 0.0;
  double allocated_hmi_risk = std::numeric_limits<double>::infinity();
  double hmi_risk_requirement = 0.0;
  bool formal_eligible = false;
  bool risk_budget_valid = false;
  std::array<AnchorId, 3> maximizing_anchor{};
  Availability availability = Availability::Unavailable;
  IntegrityLabel label = IntegrityLabel::ImplementedUnverified;
  LinearizationConsistency consistency = LinearizationConsistency::Strict;
  std::string reason;
};

struct ResidualRecord {
  TimestampNs timestamp;
  FactorId factor_id;
  AnchorId anchor_id;
  RowRole role = RowRole::Measurement;
  double raw = std::numeric_limits<double>::quiet_NaN();
  double whitened = std::numeric_limits<double>::quiet_NaN();
};

struct StageTiming {
  std::string stage;
  double wall_ms = 0.0;
  bool success = true;
  std::string status = "EXECUTED";
  std::string reason;
};

struct CandidateDiagnostics {
  bool kernel_evaluated = false;
  bool numerical_valid = false;
  bool pl_evaluated = false;
  bool slow_path = false;
  bool near_gate = false;
  bool recovered_replacement = false;
  std::string numerical_path;
  std::string fallback_reason;
  std::string skip_reason;
  std::uint64_t cache_hits = 0;
  std::uint64_t covariance_solve_count = 0;
  std::uint64_t scratch_reuse_count = 0;
  bool certificate_passed = false;
  bool matrix_free_step_rejected = false;
  std::string condition_value_kind = "UNCOMPUTED";
  double condition_lower_bound = std::numeric_limits<double>::quiet_NaN();
  double condition_upper_bound = std::numeric_limits<double>::quiet_NaN();
  double certificate_margin = std::numeric_limits<double>::quiet_NaN();
  double kernel_ms = 0.0;
  double post_ms = 0.0;
  double bridge_ms = 0.0;
  double fault_map_ms = 0.0;
  double pl_ms = 0.0;
};

struct AttemptDiagnostics {
  std::vector<std::uint64_t> frozen_group_ids;
  std::uint64_t input_attempt_id = 0;
  TimestampNs input_timestamp;
  std::uint64_t ordering_version = 0;
  std::uint64_t noise_model_version = 0;
  std::uint64_t backend_epoch_before = 0;
  std::uint64_t backend_epoch_after = 0;
  std::uint64_t raw_imu_samples = 0;
  std::uint64_t consecutive_rejections = 0;
  std::uint64_t marginalization_count = 0;
  std::uint64_t factor_block_cache_hits = 0;
  std::uint64_t factor_block_cache_misses = 0;
  std::uint64_t factor_block_cache_invalidations = 0;
  std::uint64_t statistical_cache_hits = 0;
  std::uint64_t statistical_cache_misses = 0;
  std::size_t cache_entries = 0;
  std::size_t cache_bytes = 0;
  std::string cache_invalidation_reason;
  double pending_duration_s = 0.0;
  double state_age_s = 0.0;
  double base_step_norm = std::numeric_limits<double>::infinity();
  double selected_step_norm = std::numeric_limits<double>::infinity();
  double risk_nominal = 0.0;
  double risk_p_nm = 0.0;
  double risk_bridge = 0.0;
  double risk_history = 0.0;
  double risk_model = 0.0;
  double risk_hypotheses = 0.0;
  double risk_total = 0.0;
  double risk_upper_bound = 0.0;
  double risk_margin = 0.0;
  std::uint64_t generated_actions = 0;
  std::uint64_t hypothesis_count = 0;
  std::uint64_t single_uwb_hypotheses = 0;
  std::uint64_t single_accel_hypotheses = 0;
  std::uint64_t single_gyro_hypotheses = 0;
  std::uint64_t double_uwb_accel_hypotheses = 0;
  std::uint64_t double_uwb_gyro_hypotheses = 0;
  std::uint32_t effective_fault_cardinality = 0;
  std::uint64_t kernel_evaluated_actions = 0;
  std::uint64_t post_passed_actions = 0;
  std::uint64_t pl_evaluated_actions = 0;
  std::uint64_t selected_actions = 0;
  std::uint64_t base_svd = 0;
  std::uint64_t base_llt = 0;
  std::uint64_t base_state_solves = 0;
  std::uint64_t llt_state_solve_calls = 0;
  std::uint64_t svd_state_solve_calls = 0;
  std::uint64_t detector_reference_qr = 0;
  std::uint64_t candidate_reference_svd = 0;
  std::uint64_t candidate_inner_llt = 0;
  std::uint64_t fault_gram_eigen = 0;
  std::uint64_t fault_gram_svd = 0;
  std::uint64_t fault_gram_ldlt = 0;
  std::uint64_t low_dim_fault_gram = 0;
  std::uint64_t generic_fault_gram_fallback = 0;
  std::uint64_t hypothesis_parallel_blocks = 0;
  std::uint64_t hypothesis_shared_hits = 0;
  std::uint64_t hypothesis_shared_misses = 0;
  std::uint64_t covariance_rhs_solves = 0;
  std::uint64_t covariance_rhs_columns = 0;
  std::uint64_t spectral_rhs_solves = 0;
  std::uint64_t spectral_rhs_columns = 0;
  std::uint64_t numerical_contract_mismatches = 0;
  std::uint64_t imu_oracle_reintegrations = 0;
  bool analytic_input_valid = false;
  bool analytic_computation_valid = false;
  bool oracle_executed = false;
  double oracle_relative_error = std::numeric_limits<double>::infinity();
  bool oracle_verified = false;
  // Multi-step finite-difference sweep (A4): epsilons, per-epsilon relative
  // errors and whether the observed sequence stays within tolerance over the
  // whole bracket instead of a single hand-picked step.
  bool oracle_sweep_executed = false;
  bool oracle_sweep_verified = false;
  double oracle_sweep_worst_relative_error =
      std::numeric_limits<double>::infinity();
  std::uint64_t oracle_sweep_reintegrations = 0;
  std::string oracle_sweep_epsilons;
  std::string oracle_sweep_relative_errors;
  // Failure accounting (A3). Codes come from the frozen FailureReason enum and
  // are additive; the legacy `reason` text is unchanged.
  std::string primary_failure = "NONE";
  std::string all_failures;
  std::string not_evaluated_checks;
  std::string status = "EXECUTED";
  std::string reason;
};

struct HypothesisAuditRecord {
  std::uint64_t hypothesis_id = 0;
  std::string fault_unit_ids;
  std::string physical_source_ids;
  std::string sensor;
  std::string fault_kind;
  std::string mode_ids;
  std::uint64_t onset_epoch = 0;
  std::int64_t onset_time_ns = 0;
  int parameter_dimension = 0;
  int fault_rank = 0;
  double sigma_min = 0.0;
  double sigma_max = 0.0;
  double condition_number = std::numeric_limits<double>::infinity();
  Eigen::Vector3d slope_xyz = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double boundary_direction_gram = 0.0;
  double noncentrality_boundary = std::numeric_limits<double>::infinity();
  double prior_bound = 0.0;
  double p_md_allocation = 0.0;
  double hmi_allocation = 0.0;
  bool monitorable = false;
  bool plausible = false;
  double conditioned_statistic = std::numeric_limits<double>::infinity();
  double log_evidence = -std::numeric_limits<double>::infinity();
  // B1: detection-space response Z_h = Q2^T D_h classification (0 = not
  // evaluated, 1 = full rank, 2 = harmless nullspace, 3 = dangerous nullspace,
  // 4 = numerically indistinguishable).  Filled for the multi-dimensional
  // hypotheses handled by the batch path.
  int z_rank = 0;
  double z_smallest_singular_value = std::numeric_limits<double>::infinity();
  double z_condition = std::numeric_limits<double>::infinity();
  int z_classification = 0;
  // B2 (§5.8): coverage certification of this hypothesis.  The online
  // traversal is exact, so the label is EXACT with envelope id 0; the label
  // turns into UPPER_ENVELOPE only when a grouped envelope discharged both the
  // inclusion proof and the dominance obligation, and into UNCOVERED when no
  // enumerated hypothesis or verified envelope serves the leaf.
  std::string coverage_label = "EXACT";
  std::uint64_t coverage_envelope_id = 0;
  std::string reason;
};

// B1: per-window audit of the single square-root context (one row per
// committed attempt).  Values are the certificate and identity quantities that
// the consumers rely on; they are exported so the independent oracle can check
// them without replaying the factorization.
struct SquareRootAuditRecord {
  std::uint64_t attempt_id = 0;
  int rows = 0;
  int columns = 0;
  int rank = 0;
  int dof = 0;
  double r_diagonal_min = 0.0;
  double r_diagonal_max = 0.0;
  double condition_estimate = std::numeric_limits<double>::infinity();
  double identity_residual_relative = std::numeric_limits<double>::infinity();
  double parity_relative_difference = std::numeric_limits<double>::infinity();
  double solution_relative_difference = std::numeric_limits<double>::infinity();
  double forward_error_bound = std::numeric_limits<double>::infinity();
  bool certificate_ok = false;
  bool usable = false;
  int detector_only_rows = 0;
  double statistic = std::numeric_limits<double>::infinity();
  std::string scale_policy;
  std::string permutation_policy;
  std::string reason;
};

struct CandidateAuditRecord {
  CandidateDiagnostics diagnostics;
  std::uint64_t action_id = 0;
  std::string action_type;
  std::string physical_source_ids;
  std::string removed_group_ids;
  std::string added_group_ids;
  std::string bridge_mode;
  int cardinality = 0;
  bool valid = false;
  bool post_detector_passed = false;
  bool covers_plausible_set = false;
  double statistic = std::numeric_limits<double>::infinity();
  double threshold = std::numeric_limits<double>::infinity();
  int rank = 0;
  int dof = 0;
  double condition_number = std::numeric_limits<double>::infinity();
  double information_logdet = -std::numeric_limits<double>::infinity();
  double risk_allocation = 0.0;
  double hpl_m = std::numeric_limits<double>::infinity();
  double vpl_m = std::numeric_limits<double>::infinity();
  bool selected = false;
  std::string reason;
  double wall_ms = 0.0;
};

struct CoverageAuditRecord {
  std::uint64_t action_id = 0;
  std::string action_type;
  std::string plausible_hypothesis_ids;
  std::string mandatory_health_sources;
  std::string mandatory_group_ids;
  std::string covered_mode_ids;
  std::string removed_group_ids;
  std::string added_group_ids;
  std::string uncovered_hypothesis_ids;
  std::string uncovered_mode_ids;
  std::string uncovered_group_ids;
  std::string uncovered_mandatory_group_ids;
  std::string outcome;
  std::string reason;
};

struct StateStepAuditRecord {
  std::string source;
  std::uint64_t epoch = 0;
  double rotation_norm = std::numeric_limits<double>::infinity();
  double position_norm = std::numeric_limits<double>::infinity();
  double velocity_norm = std::numeric_limits<double>::infinity();
  double accel_bias_norm = std::numeric_limits<double>::infinity();
  double gyro_bias_norm = std::numeric_limits<double>::infinity();
  double epoch_norm = std::numeric_limits<double>::infinity();
};

struct FactorLedgerAuditRecord {
  std::uint64_t factor_id = 0;
  std::uint64_t group_id = 0;
  std::string sensor;
  std::string factor_kind;
  std::string lifecycle;
  std::size_t epoch_begin = 0;
  std::size_t epoch_end = 0;
  TimestampNs time_begin;
  TimestampNs time_end;
  std::string backend_slot;
  std::string noise_model_id;
  std::string model_id;
  std::string health;
  std::string source_ids;
  std::string measurement_ids;
  std::string fault_units;
  std::uint64_t commit_graph_version = 0;
  std::uint64_t removed_graph_version = 0;
  std::uint64_t replacement_group_id = 0;
  std::uint64_t replaces_group_id = 0;
  std::size_t recovery_epoch = 0;
};

struct HealthAuditRecord {
  std::string source_id;
  std::string sensor;
  std::string previous_state;
  std::string current_state;
  std::string trigger;
  double evidence_statistic = std::numeric_limits<double>::infinity();
  double evidence_threshold = std::numeric_limits<double>::infinity();
  std::string plausible_hypothesis_ids;
  std::uint64_t selected_action_id = 0;
  std::uint32_t suspicion_count = 0;
  std::uint32_t shadow_pass_count = 0;
  std::uint32_t recovery_pass_count = 0;
  std::uint32_t bridge_count = 0;
  std::uint32_t recovery_reset_count = 0;
};

struct BridgeAuditRecord {
  std::string mode = "NONE";
  std::uint32_t consecutive_epochs = 0;
  double duration_s = 0.0;
  std::string model_id;
  double dt_s = 0.0;
  Eigen::VectorXd optimization_covariance_diagonal;
  std::string integrity_model;
  std::string calibration_id;
  Eigen::Vector3d bound = Eigen::Vector3d::Zero();
  bool control_available = false;
  bool bias_continuity_maneuver_margin_applied = false;
  bool active = false;
  bool timeout = false;
  std::string status;
};

struct IntegrityOutput {
  AttemptDiagnostics diagnostics;
  IntegritySnapshotIdentity snapshot_identity;
  TimestampNs timestamp;
  NavigationState state;
  DetectorResult detector;
  std::vector<SensitivityResult> sensitivities;
  ProtectionLevelResult protection_level;
  double global_graph_residual_statistic =
      std::numeric_limits<double>::quiet_NaN();
  double uwb_postfit_residual_statistic =
      std::numeric_limits<double>::quiet_NaN();
  double conditional_innovation_statistic =
      std::numeric_limits<double>::quiet_NaN();
  // Diagnostic detector records use independently calibrated thresholds when
  // available.  `detector` remains the formal conditional detector (or the
  // snapshot post-fit detector for the ROS-free snapshot path).
  DetectorResult global_detector;
  DetectorResult postfit_detector;
  std::size_t measurement_group_size = 0;
  bool batch_committed = false;
  // True when the current UWB has valid model/capability/provenance inputs.
  // Risk-budget or alert-limit unavailability does not clear this flag.
  bool measurement_model_valid = false;
  // V4+ transaction/FDE audit. Zero IDs mean the legacy V1 facade was used.
  std::uint64_t transaction_id = 0;
  std::uint64_t window_id = 0;
  std::uint64_t base_graph_version = 0;
  std::uint64_t linearization_version = 0;
  std::uint64_t selected_action_id = 0;
  std::string selected_action_type = "KEEP_ALL";
  std::string fde_status = "NOT_TRIGGERED";
  Eigen::Vector3d bridge_component_m = Eigen::Vector3d::Zero();
  bool history_provenance_valid = false;
  std::uint32_t backend_updates = 0;
  bool stale_state = false;
  bool controlled_reinitialization_required = false;
  std::vector<std::uint64_t> historical_groups_removed;
  std::vector<std::uint64_t> historical_groups_added;
  std::uint64_t recovery_epoch_begin = 0;
  std::uint64_t recovery_epoch_end = 0;
  std::uint64_t reinitialization_request_id = 0;
  std::string reinitialization_phase = "RUNNING";
  std::string reinitialization_reason;
  std::vector<ResidualRecord> residual_records;
  std::vector<StageTiming> stage_timings;
  std::vector<HypothesisAuditRecord> hypothesis_audit;
  std::vector<CandidateAuditRecord> candidate_audit;
  // B1: one entry per committed attempt while the frozen window carries a
  // square-root context (see SquareRootAuditRecord).
  std::vector<SquareRootAuditRecord> square_root_audit;
  std::vector<CoverageAuditRecord> coverage_audit;
  std::vector<StateStepAuditRecord> state_step_audit;
  std::vector<FactorLedgerAuditRecord> factor_ledger_audit;
  std::vector<HealthAuditRecord> health_audit;
  std::optional<BridgeAuditRecord> bridge_audit;
};

struct RunManifest {
  std::string schema_version = "uwb-imu-pl/v5";
  std::string created_utc;
  std::string git_sha;
  bool git_dirty = false;
  std::string config_path;
  std::string config_hash;
  std::string protocol_sha256;
  std::string protocol_path;
  std::string raw_inventory_path;
  std::string raw_inventory_sha256;
  std::string artifact_checksum_path;
  std::string seed_domain = "standalone";
  std::uint32_t attempt = 1;
  std::string failure_catalog_path;
  std::string resolved_config;
  std::uint64_t seed = 0;
  std::uint32_t fixed_lag_epochs = 0;
  std::string execution_command;
  std::string build_type;
  std::string compiler;
  std::string os;
  std::string cpu;
  std::uint64_t ram_bytes = 0;
  std::string gtsam_version;
  std::string eigen_version;
  std::string maturity = "IMPLEMENTED_UNVERIFIED";
  bool formal_eligible = false;
  std::string protected_state = "body_origin_position_world";
  std::string protected_quantity = "position_xyz";
  std::string position_reference = "body_origin";
  std::string detector = "joint_window_residual_chi_square";
  std::string pl_method = "residual_failure_mode_slope";
  std::uint32_t window_epochs = 20;
  bool single_faults_enabled = true;
  bool double_faults_enabled = false;
  std::uint32_t supported_fault_cardinality = 2;
  std::uint32_t monitored_fault_cardinality = 1;
  std::uint32_t max_exclusion_cardinality = 2;
  std::string bridge_model = "kinematic_cv_bounded";
  std::string history_recovery = "active_window_only_maturity_delay";
  std::string diagnostics_schema_version = "uwb-imu-pl/gate-d-diagnostics/v12";
  std::string failure_catalog;
  std::string fault_manifest_digest;
  std::string fault_manifest_id;
  std::string risk_calibration_id;
  std::string noise_overbound_calibration_id;
  std::string bridge_calibration_id;
  bool gates_a_to_i_complete = false;
  bool independent_review_complete = false;
};

struct TimingRecord {
  std::uint64_t input_attempt_id = 0;
  std::uint64_t transaction_id = 0;
  std::uint64_t window_id = 0;
  TimestampNs timestamp;
  std::uint64_t epoch = 0;
  std::string stage;
  double wall_ms = 0.0;
  std::size_t problem_size = 0;
  std::size_t hypothesis_count = 0;
  std::size_t factor_count = 0;
  bool cold = false;
  bool success = true;
};

struct ComparisonDiagnostics {
  TimestampNs timestamp;
  std::uint64_t epoch = 0;
  bool candidate_spd = false;
  double candidate_condition = std::numeric_limits<double>::infinity();
  double prior_mean_difference = std::numeric_limits<double>::infinity();
  double covariance_relative_error = std::numeric_limits<double>::infinity();
  double statistic_relative_error = std::numeric_limits<double>::infinity();
  double protection_level_relative_error = std::numeric_limits<double>::infinity();
  bool availability_equal = false;
  bool decision_equal = false;
};

struct GroundTruthRecord {
  TimestampNs timestamp;
  Eigen::Vector3d position_world_m = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_world_body = Eigen::Quaterniond::Identity();
};

struct FaultTruthRecord {
  TimestampNs timestamp;
  std::uint64_t sequence = 0;
  AnchorId anchor_id;
  std::string fault_mode;
  bool active = false;
  bool outage = false;
  double injected_bias_m = 0.0;
  double true_range_m = std::numeric_limits<double>::quiet_NaN();
  std::string sensor_type = "UWB";
  std::string fault_kind = "anchor_range_bias";
  int axis = -1;
  std::uint64_t epoch_begin = 0;
  std::uint64_t epoch_end = 0;
  double injected_value = 0.0;
  std::string injected_units = "m";
};

struct RunSummary {
  std::string status = "IMPLEMENTED_UNVERIFIED";
  std::uint64_t processed = 0;
  std::uint64_t committed = 0;
  std::uint64_t rejected = 0;
  std::uint64_t errors = 0;
  double core_total_ms = 0.0;
  double end_to_end_total_ms = 0.0;
  std::string detail;
};

const char* toString(Availability value);
const char* toString(IntegrityLabel value);

}  // namespace uwb_imu_pl
