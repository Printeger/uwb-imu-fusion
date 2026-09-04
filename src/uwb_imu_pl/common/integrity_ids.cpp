#include "uwb_imu_pl/common/integrity_ids.hpp"

namespace uwb_imu_pl {
namespace {
template <typename T>
const char* unknown(T) { return "UNKNOWN"; }
}  // namespace

const char* toString(SensorType v) {
  switch (v) {
    case SensorType::Uwb: return "UWB";
    case SensorType::Imu: return "IMU";
    case SensorType::ImuAccelerometer: return "IMU_ACCELEROMETER";
    case SensorType::ImuGyroscope: return "IMU_GYROSCOPE";
    case SensorType::Bridge: return "BRIDGE";
    case SensorType::Prior: return "PRIOR";
    default: return unknown(v);
  }
}
const char* toString(FaultKind v) {
  switch (v) {
    case FaultKind::AnchorBiasEpochIndependent: return "ANCHOR_BIAS_EPOCH_INDEPENDENT";
    case FaultKind::AnchorBiasPersistentConstant: return "ANCHOR_BIAS_PERSISTENT_CONSTANT";
    case FaultKind::AnchorBiasRamp: return "ANCHOR_BIAS_RAMP";
    case FaultKind::AccelAxisIntervalConstant: return "ACCEL_AXIS_INTERVAL_CONSTANT";
    case FaultKind::GyroAxisIntervalConstant: return "GYRO_AXIS_INTERVAL_CONSTANT";
    case FaultKind::HardwareBarrier: return "HARDWARE_BARRIER";
    case FaultKind::BridgeEscape: return "BRIDGE_ESCAPE";
    default: return unknown(v);
  }
}
const char* toString(FactorKind v) {
  switch (v) {
    case FactorKind::BoundaryPrior: return "BOUNDARY_PRIOR";
    case FactorKind::CombinedImu: return "COMBINED_IMU";
    case FactorKind::UwbBatch: return "UWB_BATCH";
    case FactorKind::KinematicBridge: return "KINEMATIC_BRIDGE";
    case FactorKind::BiasContinuity: return "BIAS_CONTINUITY";
    case FactorKind::Regularizer: return "REGULARIZER";
    default: return unknown(v);
  }
}
const char* toString(FactorLifecycle v) {
  switch (v) {
    case FactorLifecycle::Pending: return "PENDING";
    case FactorLifecycle::Active: return "ACTIVE";
    case FactorLifecycle::RemovedByFde: return "REMOVED_BY_FDE";
    case FactorLifecycle::Marginalized: return "MARGINALIZED";
    case FactorLifecycle::SupersededByBridge: return "SUPERSEDED_BY_BRIDGE";
    case FactorLifecycle::QuarantinedSource: return "QUARANTINED_SOURCE";
  }
  return unknown(v);
}
const char* toString(BridgeMode v) {
  switch (v) {
    case BridgeMode::None: return "NONE";
    case BridgeMode::GenericKinematic: return "GENERIC_KINEMATIC";
    case BridgeMode::Dynamics: return "DYNAMICS";
  }
  return unknown(v);
}
const char* toString(HealthState v) {
  switch (v) {
    case HealthState::Healthy: return "HEALTHY";
    case HealthState::Suspect: return "SUSPECT";
    case HealthState::Quarantined: return "QUARANTINED";
    case HealthState::RecoveryTest: return "RECOVERY_TEST";
    case HealthState::Failed: return "FAILED";
  }
  return unknown(v);
}
const char* toString(HistoryRecoverability v) {
  switch (v) {
    case HistoryRecoverability::Recoverable: return "RECOVERABLE";
    case HistoryRecoverability::Marginalized: return "MARGINALIZED";
    case HistoryRecoverability::MissingProvenance: return "MISSING_PROVENANCE";
    case HistoryRecoverability::OnsetBeforeRecoverableBoundary:
      return "ONSET_BEFORE_RECOVERABLE_BOUNDARY";
  }
  return unknown(v);
}
const char* toString(ReinitializationState v) {
  switch (v) {
    case ReinitializationState::Running: return "RUNNING";
    case ReinitializationState::Requested: return "REQUESTED";
    case ReinitializationState::WaitingForTrustedImu:
      return "WAITING_FOR_TRUSTED_IMU";
    case ReinitializationState::Reinitialized: return "REINITIALIZED";
  }
  return unknown(v);
}
const char* toString(FdeStatus v) {
  switch (v) {
    case FdeStatus::NotTriggered: return "NOT_TRIGGERED";
    case FdeStatus::SuccessKeepAll: return "SUCCESS_KEEP_ALL";
    case FdeStatus::SuccessUwbExclusion: return "SUCCESS_UWB_EXCLUSION";
    case FdeStatus::SuccessImuExclusionGenericBridge: return "SUCCESS_IMU_EXCLUSION_GENERIC_BRIDGE";
    case FdeStatus::SuccessImuExclusionDynamicsBridge: return "SUCCESS_IMU_EXCLUSION_DYNAMICS_BRIDGE";
    case FdeStatus::SuccessMultiSensorExclusion: return "SUCCESS_MULTI_SENSOR_EXCLUSION";
    case FdeStatus::AmbiguousUnionExclusion: return "AMBIGUOUS_UNION_EXCLUSION";
    case FdeStatus::AmbiguousUnavailable: return "AMBIGUOUS_UNAVAILABLE";
    case FdeStatus::NoValidCandidate: return "NO_VALID_CANDIDATE";
    case FdeStatus::FaultModeUnmonitorable: return "FAULT_MODE_UNMONITORABLE";
    case FdeStatus::HistoryPriorContaminated: return "HISTORY_PRIOR_CONTAMINATED";
    case FdeStatus::BridgeTimeout: return "BRIDGE_TIMEOUT";
    case FdeStatus::ModelInvalid: return "MODEL_INVALID";
    case FdeStatus::RiskBudgetInvalid: return "RISK_BUDGET_INVALID";
    case FdeStatus::BackendVersionMismatch: return "BACKEND_VERSION_MISMATCH";
    case FdeStatus::ControlledReinitializationRequired: return "CONTROLLED_REINITIALIZATION_REQUIRED";
  }
  return unknown(v);
}
}  // namespace uwb_imu_pl
