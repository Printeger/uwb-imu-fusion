#include "uwb_imu_pl/common/runtime_route_audit.hpp"
#include <array>
#include <atomic>
#include <limits>
#include <stdexcept>

namespace uwb_imu_pl {
namespace { std::array<std::atomic<std::uint64_t>,4> counts{}; }
void recordRuntimeRouteEventV1(RuntimeRouteEventV1 event) {
  auto& count=counts.at(static_cast<std::size_t>(event));
  auto value=count.load(std::memory_order_relaxed);
  do {
    if(value==std::numeric_limits<std::uint64_t>::max())
      throw std::overflow_error("RUNTIME_ROUTE_AUDIT_CAPACITY");
  } while(!count.compare_exchange_weak(value,value+1,std::memory_order_relaxed));
}
RuntimeRouteAuditV1 runtimeRouteAuditV1() {
  return {counts[0].load(std::memory_order_relaxed),counts[1].load(std::memory_order_relaxed),
          counts[2].load(std::memory_order_relaxed),counts[3].load(std::memory_order_relaxed)};
}
}
