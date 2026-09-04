#pragma once

#include "uwb_imu_pl/common/types.hpp"

namespace uwb_imu_pl {

struct TransactionIdTag {};
struct WindowIdTag {};
struct FactorGroupIdTag {};
struct FaultUnitIdTag {};
struct FaultModeIdTag {};
struct ExclusionActionIdTag {};
struct RecoveryRequestIdTag {};

using TransactionId = StrongId<TransactionIdTag>;
using WindowId = StrongId<WindowIdTag>;
using FactorGroupId = StrongId<FactorGroupIdTag>;
using FaultUnitId = StrongId<FaultUnitIdTag>;
using FaultModeId = StrongId<FaultModeIdTag>;
using ExclusionActionId = StrongId<ExclusionActionIdTag>;
using RecoveryRequestId = StrongId<RecoveryRequestIdTag>;

enum class SensorType {
  Unknown,
  Uwb,
  Imu,
  ImuAccelerometer,
  ImuGyroscope,
  Bridge,
  Prior
};

enum class FaultKind {
  Unknown,
  AnchorBiasEpochIndependent,
  AnchorBiasPersistentConstant,
  AnchorBiasRamp,
  AccelAxisIntervalConstant,
  GyroAxisIntervalConstant,
  HardwareBarrier,
  BridgeEscape
};

enum class FactorKind {
  Unknown,
  BoundaryPrior,
  CombinedImu,
  UwbBatch,
  KinematicBridge,
  BiasContinuity,
  Regularizer
};

enum class FactorLifecycle {
  Pending,
  Active,
  RemovedByFde,
  Marginalized,
  SupersededByBridge,
  QuarantinedSource
};

enum class BridgeMode { None, GenericKinematic, Dynamics };
enum class HealthState { Healthy, Suspect, Quarantined, RecoveryTest, Failed };
enum class HistoryRecoverability {
  Recoverable,
  Marginalized,
  MissingProvenance,
  OnsetBeforeRecoverableBoundary
};
enum class ReinitializationState {
  Running,
  Requested,
  WaitingForTrustedImu,
  Reinitialized
};

enum class FdeStatus {
  NotTriggered,
  SuccessKeepAll,
  SuccessUwbExclusion,
  SuccessImuExclusionGenericBridge,
  SuccessImuExclusionDynamicsBridge,
  SuccessMultiSensorExclusion,
  AmbiguousUnionExclusion,
  AmbiguousUnavailable,
  NoValidCandidate,
  FaultModeUnmonitorable,
  HistoryPriorContaminated,
  BridgeTimeout,
  ModelInvalid,
  RiskBudgetInvalid,
  BackendVersionMismatch,
  ControlledReinitializationRequired
};

const char* toString(SensorType value);
const char* toString(FaultKind value);
const char* toString(FactorKind value);
const char* toString(FactorLifecycle value);
const char* toString(BridgeMode value);
const char* toString(HealthState value);
const char* toString(HistoryRecoverability value);
const char* toString(ReinitializationState value);
const char* toString(FdeStatus value);

}  // namespace uwb_imu_pl
