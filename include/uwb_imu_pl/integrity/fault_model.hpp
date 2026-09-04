#pragma once

#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"

#include <Eigen/Core>

#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace uwb_imu_pl {

struct FaultSubspaceModel {
  Eigen::MatrixXd map;
  std::string model_id;
};

struct AnchorBiasRamp {
  double bias_at_onset_m = 0.0;
  double slope_mps = 0.0;
};

// Compact group-wise physical fault basis.  Raw maps are kept in the native
// rows of each factor group and are whitened only with that group's frozen
// covariance.  This prevents O(hypotheses * window_rows) dense storage.
struct FaultModeBasis {
  FaultModeId id;
  FaultKind kind = FaultKind::Unknown;
  SensorType sensor = SensorType::Unknown;
  std::string physical_source_id;
  AnchorId anchor_id;
  int axis = -1;
  std::size_t onset_epoch = 0;
  TimestampNs onset_time;
  int parameter_dimension = 0;
  std::map<FactorGroupId, Eigen::MatrixXd> raw_group_maps;
  std::vector<FactorGroupId> affected_groups;
  std::vector<MeasurementId> affected_measurements;
  double prior_probability_bound = 0.0;
  HealthState source_health = HealthState::Healthy;
  HistoryRecoverability recoverability = HistoryRecoverability::Recoverable;
};

struct MonitorabilityResult {
  int rank = 0;
  int parameter_dimension = 0;
  double sigma_min = 0.0;
  double sigma_max = 0.0;
  double condition_number = std::numeric_limits<double>::infinity();
  Eigen::Vector3d protected_slopes = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  bool monitorable = false;
  std::string reason;
};

struct FaultUnit {
  FaultUnitId id;
  SensorType sensor = SensorType::Unknown;
  FaultKind kind = FaultKind::Unknown;
  std::string physical_source_id;
  int axis = -1;
  std::size_t epoch_begin = 0;
  std::size_t epoch_end = 0;
  TimestampNs time_begin;
  TimestampNs time_end;
  std::vector<FactorGroupId> affected_groups;
  std::vector<MeasurementId> affected_measurements;
  int parameter_dimension = 0;
  FaultSubspaceModel subspace_model;
  double prior_probability_bound = 0.0;
  HealthState source_health = HealthState::Healthy;
  bool persistent = false;
};

struct FaultHypothesisV2 {
  HypothesisId id;
  std::vector<FaultUnitId> units;
  std::vector<FaultModeId> modes;
  std::vector<FactorGroupId> affected_groups;
  // Deprecated compatibility cache. New hypotheses leave this empty and the
  // evaluator assembles projected mode columns on demand.
  Eigen::MatrixXd A;
  double prior_probability_bound = 0.0;
  double p_md_allocation = 0.0;
  double hmi_allocation = 0.0;
  MonitorabilityResult monitorability;
  bool monitored = false;
  std::string pruning_reason;
};

struct FaultModeEvidence {
  HypothesisId hypothesis;
  Eigen::VectorXd estimated_fault;
  double all_in_statistic = std::numeric_limits<double>::infinity();
  double conditioned_statistic = std::numeric_limits<double>::infinity();
  double explained_energy = 0.0;
  double log_evidence = -std::numeric_limits<double>::infinity();
  MonitorabilityResult monitorability;
  bool plausible = false;
};

struct ExclusionAction {
  ExclusionActionId id;
  std::vector<FaultUnitId> covered_units;
  std::vector<FaultModeId> covered_modes;
  std::vector<std::string> physical_source_ids;
  std::vector<FactorGroupId> groups_to_remove;
  std::vector<FactorGroupId> groups_to_add;
  std::vector<LinearizedFactorBlock> added_blocks;
  BridgeMode bridge_mode = BridgeMode::None;
  int exclusion_cardinality = 0;
  std::string action_model_id;
  HistoryRecoverability recoverability = HistoryRecoverability::Recoverable;
  std::optional<std::size_t> recovery_epoch_begin;
  std::optional<std::size_t> recovery_epoch_end;
};

struct BridgeUncertainty {
  enum class IntegrityModel {
    GaussianOverbound,
    DeterministicBox,
    DeterministicEllipsoid
  };
  Eigen::MatrixXd optimization_covariance;
  IntegrityModel integrity_model = IntegrityModel::DeterministicBox;
  Eigen::VectorXd deterministic_bound;
  std::string calibration_id;
};

}  // namespace uwb_imu_pl
