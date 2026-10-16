#pragma once
#include <cstdint>

namespace uwb_imu_pl {
// Process-lifetime observations, never reset by a replay or model change.
// Constructors count attempted constructions, including ones that later throw.
struct RuntimeRouteAuditV1 {
  std::uint64_t old_pipeline_constructions=0;
  std::uint64_t old_engine_calls=0;
  std::uint64_t old_fde_calls=0;
  std::uint64_t incremental_backend_constructions=0;
};
enum class RuntimeRouteEventV1 {
  OldPipelineConstruction, OldEngineCall, OldFdeCall, IncrementalBackendConstruction
};
void recordRuntimeRouteEventV1(RuntimeRouteEventV1 event);
RuntimeRouteAuditV1 runtimeRouteAuditV1();
}
