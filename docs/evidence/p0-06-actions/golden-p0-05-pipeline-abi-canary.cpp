#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <new>

using namespace uwb_imu_pl;

int main() {
#if EIGEN_MAX_ALIGN_BYTES <= 16
  static_assert(sizeof(RealtimeIntegrityPipeline) == 5056,
                "golden portable pipeline size changed");
  static_assert(alignof(RealtimeIntegrityPipeline) == 16,
                "golden portable pipeline alignment changed");
#else
  // The project DSO is built with -march=native, which selects Eigen's 32-byte
  // ABI on this host. Golden and current headers both produce this layout when
  // compiled with the DSO's exact flags; the portable header check above is
  // the reviewer's 5056/16 canary.
  static_assert(sizeof(RealtimeIntegrityPipeline) == 5184,
                "golden DSO-flags pipeline size changed");
  static_assert(alignof(RealtimeIntegrityPipeline) == 32,
                "golden DSO-flags pipeline alignment changed");
#endif
  struct GuardedStorage {
    std::array<std::uint64_t, 4> before;
    alignas(RealtimeIntegrityPipeline)
        unsigned char object[sizeof(RealtimeIntegrityPipeline)];
    std::array<std::uint64_t, 4> after;
  } storage;
  storage.before.fill(0x13579bdf2468ace0ULL);
  storage.after.fill(0xfedcba9876543210ULL);

  IntegrityConfig config;
  auto* estimator = new IncrementalUwbImuEstimator(
      config, Eigen::Vector3d::Zero());
  IntegrityMonitor monitor(config.risk, 1e-10, 1e10);
  auto* pipeline = new (storage.object) RealtimeIntegrityPipeline(
      estimator, std::move(monitor), PublicationLimits{});

  const bool publication_callable =
      &pipeline->publication() != nullptr;
  const bool output_callable =
      &pipeline->lastAttemptOutput() != nullptr;
  const PipelineCommitBoundaryAuditV1 audit = pipeline->commitBoundaryAudit();
  const bool by_value_callable = audit.consecutive_bridge_epochs == 0 &&
      !audit.bridge_start_present;

  pipeline->~RealtimeIntegrityPipeline();
  delete estimator;
  const bool guards_intact =
      std::all_of(storage.before.begin(), storage.before.end(),
                  [](std::uint64_t value) {
                    return value == 0x13579bdf2468ace0ULL;
                  }) &&
      std::all_of(storage.after.begin(), storage.after.end(),
                  [](std::uint64_t value) {
                    return value == 0xfedcba9876543210ULL;
                  });
  std::cout << "RealtimeIntegrityPipeline "
            << sizeof(RealtimeIntegrityPipeline) << ' '
            << alignof(RealtimeIntegrityPipeline) << '\n';
  std::cout << "construct-destruct-call " << publication_callable << ' '
            << output_callable << ' ' << by_value_callable << ' '
            << guards_intact << '\n';
  return publication_callable && output_callable && by_value_callable &&
      guards_intact ? 0 : 1;
}
