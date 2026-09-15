#pragma once

#include <cstdint>

namespace uwb_imu_pl {

struct StatisticalBoundsCacheStats {
  std::uint64_t hits = 0;
  std::uint64_t misses = 0;
  std::uint64_t entries = 0;
};

class StatisticalBoundsCache {
 public:
  static double chiSquaredThreshold(int dof, double p_fa);
  static double noncentralityBoundary(int dof, double squared_threshold,
                                      double p_md);
  static double normalTwoSidedMultiplier(double tail_probability);
  static StatisticalBoundsCacheStats stats();
};

}  // namespace uwb_imu_pl
