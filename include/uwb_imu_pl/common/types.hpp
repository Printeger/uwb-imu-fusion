#pragma once

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
};

struct IntegrityOutput {
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
  std::vector<ResidualRecord> residual_records;
  std::vector<StageTiming> stage_timings;
};

struct RunManifest {
  std::string schema_version = "uwb-imu-pl/v2";
  std::string created_utc;
  std::string git_sha;
  bool git_dirty = false;
  std::string config_path;
  std::string config_hash;
  std::string resolved_config;
  std::uint64_t seed = 0;
  std::string build_type;
  std::string compiler;
  std::string os;
  std::string cpu;
  std::uint64_t ram_bytes = 0;
  std::string gtsam_version;
  std::string eigen_version;
};

struct TimingRecord {
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
