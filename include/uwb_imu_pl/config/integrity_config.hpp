#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "uwb_imu_pl/common/types.hpp"
#include "uwb_imu_pl/config/fault_manifest.hpp"
#include "uwb_imu_pl/factors/kinematic_bridge_factor.hpp"
#include "uwb_imu_pl/integrity/fde_manager.hpp"
#include "uwb_imu_pl/integrity/health_manager.hpp"

namespace uwb_imu_pl {

struct SnapshotConfig {
  int dimensions = 3;
  int max_iterations = 30;
  double step_tolerance_m = 1e-7;
  double rank_tolerance = 1e-10;
  double max_condition_number = 1e10;
};

struct IncrementalConfig {
  double relinearize_threshold = 0.1;
  int relinearize_skip = 1;
  double smoothness_sigma_m = 0.5;
  double epoch_bin_s = 0.02;
  double max_time_skew_s = 0.01;
  bool enable_method_b = false;
  double method_b_max_condition = 1e10;
  std::uint32_t fixed_lag_epochs = 0;
  bool single_transaction_per_epoch = true;
};

struct IntegrityWindowConfig {
  std::uint32_t epochs = 20;
  std::uint32_t recovery_margin_epochs = 10;
  std::string boundary_method = "bayes_tree_schur";
  double rank_tolerance = 1e-10;
  double max_condition_number = 1e10;
  double max_linearization_step_norm = 0.25;
  bool require_history_provenance = true;
};

struct HistoryCapacityConfig {
  // C1-c/C2: history-summary capacity keys (strict loader; design freeze
  // history-summary-design.md §7.6/§7.7).  The defaults are the D-2 measured
  // values: with the shipped research configuration the recoverable span is
  // at most (epochs + recovery_margin_epochs) = 20 epochs, and the fault
  // column count is at most 22 per epoch (8 anchors x {constant, linear} +
  // 6 IMU axes); the measured profile reached 220 columns at half span
  // (epoch 26, span 10), so the full-span bound is 440 and the default is
  // 512 with headroom.  `capacity_action` semantics are frozen:
  // REFUSE / RESET / STOP_PROTECTED - the summary path must never silently
  // drop the oldest fault.  Enforcement (counting, unusable marking) lands
  // with the Part C wiring; these defaults are to be re-frozen there.
  std::uint64_t max_summary_rows = 512;
  std::uint64_t max_fault_columns = 512;
  std::uint64_t max_perp_rows = 512;
  std::string capacity_action = "REFUSE";
};

struct DetectorConfigV2 {
  std::string type = "joint_window_residual_chi_square";
  double p_fa_per_test = 1e-6;
  std::uint64_t continuity_horizon_tests = 1000;
  std::string continuity_accounting = "union_bound";
  bool use_squared_norm_statistic = true;
};

struct UwbFaultModelConfig {
  bool enabled = true;
  bool epoch_single_anchor_bias = true;
  bool persistent_anchor_bias = true;
  bool ramp_bias = true;
  double prior_probability_bound = 1e-4;
  double p_md = 1e-3;
};

struct ImuFaultModelConfig {
  bool accel_axis_interval_bias = true;
  bool gyro_axis_interval_bias = true;
  bool accel_xyz_interval_bias = false;
  bool gyro_xyz_interval_bias = false;
  bool bias_jump = false;
  bool ramp_fault = false;
  double accel_prior_probability_bound = 1e-5;
  double gyro_prior_probability_bound = 1e-5;
  double p_md = 1e-3;
  double min_fault_gram_sigma = 1e-8;
  double max_fault_gram_condition = 1e10;
};

struct CombinationFaultModelConfig {
  bool uwb_plus_accel = true;
  bool uwb_plus_gyro = true;
  bool two_uwb = false;
  bool assume_independent_priors = false;
};

struct FaultModelsConfig {
  bool single_faults_enabled = true;
  bool double_faults_enabled = false;
  std::uint32_t max_cardinality = 2;
  // A3 migration: max_fault_order is the canonical switch (1 = single events
  // only, 2 = singles plus declared pair families).  Legacy
  // single_/double_faults_enabled remain readable but conflicts are rejected.
  std::optional<int> max_fault_order;
  // Optional fault manifest path (resolved relative to the config file).
  std::string manifest_path;
  UwbFaultModelConfig uwb;
  ImuFaultModelConfig imu;
  CombinationFaultModelConfig combinations;
};

struct FdeConfigV2 {
  std::string trigger = "joint_detector_alarm_or_hardware_barrier";
  std::string isolation = "profile_parity_glrt";
  std::string ambiguity_policy = "union_exclusion_else_unavailable";
  std::string selection_primary = "minimum_cardinality";
  std::string selection_secondary = "minimum_protection_level";
  bool dense_oracle_online_fallback = false;
  std::uint32_t max_candidate_count = 128;
  // Independent of the monitored hypothesis cardinality.  A single-fault
  // census can still require a conservative union action covering multiple
  // indistinguishable single-fault explanations.
  std::uint32_t max_exclusion_cardinality = 2;
};

std::uint32_t enabledFaultHypothesisCardinality(
    const FaultModelsConfig& config);
std::uint64_t faultModelPolicyFingerprint(const FaultModelsConfig& config);

struct BridgeConfigV2 {
  GenericBridgeSpec generic;
  bool dynamics_enabled = false;
  bool dynamics_require_control_input = true;
  std::string dynamics_model_calibration_id;
  std::string dynamics_integrity_model = "deterministic_ellipsoid";
  std::uint32_t max_consecutive_epochs = 20;
  double max_duration_s = 1.0;
};

struct RobustShadowConfig {
  bool gnc_enabled = false;
  bool never_mutate_formal_weights = true;
};

struct ImuNoiseConfig {
  double accelerometer_sigma = 0.0;
  double gyroscope_sigma = 0.0;
  double accelerometer_bias_rw_sigma = 0.0;
  double gyroscope_bias_rw_sigma = 0.0;
  double gravity_mps2 = 9.80665;
  double max_gap_s = 0.02;
  std::string noise_overbound_calibration_id;
};

struct OutputConfig {
  std::string root;
  bool write_residuals = true;
  bool write_timing = true;
  bool write_global_diagnostics = false;
  std::string schema_version = "uwb-imu-pl/v5";
  bool write_factor_ledger = true;
  bool write_window_rows = false;
  bool write_hypothesis_evidence = true;
  bool write_candidates = true;
  bool write_health = true;
  bool write_bridge = true;
};

struct RealtimeConfig {
  std::string world_frame;
  std::string body_frame;
  std::string imu_topic;
  std::string uwb_topic;
  std::string odometry_topic;
  std::string integrity_topic;
  std::string diagnostics_topic;
  Eigen::Vector3d initial_position_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d initial_velocity_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d lever_arm_body_m = Eigen::Vector3d::Zero();
  double range_sigma_m = 0.10;
  Eigen::Matrix<double, 15, 1> prior_sigmas =
      Eigen::Matrix<double, 15, 1>::Ones();
};

struct IntegrityConfig {
  std::string schema_version = "uwb-imu-pl/v5";
  std::uint64_t seed = 0;
  SnapshotConfig snapshot;
  IncrementalConfig incremental;
  ImuNoiseConfig imu;
  RiskBudget risk;
  RiskBudgetV2 risk_v2;
  IntegrityWindowConfig integrity_window;
  HistoryCapacityConfig history;
  DetectorConfigV2 detector;
  FaultModelsConfig fault_models;
  FdeConfigV2 fde;
  BridgeConfigV2 bridge;
  HealthConfigV2 health;
  RobustShadowConfig robust_shadow;
  OutputConfig output;
  RealtimeConfig realtime;
  std::vector<AnchorRecord> anchors;
  std::string resolved_yaml;
  std::string source_path;
  std::string config_hash;
  // A3: resolved fault manifest (present when fault_models.manifest_path is
  // set) and the migration warnings that participated in the config hash.
  std::optional<FaultManifest> fault_manifest;
  std::vector<std::string> migration_warnings;
};

// Experiment-only overrides are applied to the parsed YAML before strict
// validation, resolved serialization, and hashing.  Keeping the overrides in
// one value object prevents a run from reporting the hash of its base config.
struct IntegrityConfigOverrides {
  std::optional<std::uint64_t> seed;
  std::optional<std::uint32_t> fixed_lag_epochs;
  std::optional<bool> write_global_diagnostics;
  std::optional<bool> write_residuals;
  std::optional<bool> write_timing;
  std::optional<std::string> output_root;
};

class IntegrityConfigLoader {
 public:
  // This loader is deliberately strict: every safety-relevant field is
  // required, unknown keys are rejected, and no random seed is implicit.
  // A non-empty fixed-lag override is applied before validation,
  // serialization, and hashing. An empty optional uses the YAML value.
  static IntegrityConfig load(const std::string& yaml_path,
                              const std::optional<std::string>&
                                  fixed_lag_epochs_override = std::nullopt);
  static IntegrityConfig load(const std::string& yaml_path,
                              const IntegrityConfigOverrides& overrides);
  // Used by explicit development-only runtime manifests after appending a
  // complete description of non-research configuration overrides.
  static std::string hashResolvedYaml(const std::string& resolved_yaml);
};

}  // namespace uwb_imu_pl
