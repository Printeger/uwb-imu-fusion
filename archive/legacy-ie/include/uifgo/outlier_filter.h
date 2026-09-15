#pragma once

#include "uifgo/types.h"
#include "uifgo/config.h"
#include <vector>

namespace uifgo {

class OutlierFilter {
 public:
  explicit OutlierFilter(const Config& cfg);

  // Pre-filter a single UWB frame: apply NLOS (RSSI), range bounds.
  // Returns retained ranges.
  std::vector<UwbRange> PreFilter(const UwbFrame& frame) const;

  // Distance-consistency check using predicted tag position.
  // Returns true if the measurement is consistent.
  bool ConsistencyCheck(const UwbRange& range,
                        const gtsam::Point3& tag_pred_world,
                        const gtsam::Point3& anchor_pos,
                        bool warmed_up) const;

 private:
  Config cfg_;
};

}  // namespace uifgo
