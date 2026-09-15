#include "uifgo/outlier_filter.h"
#include <cmath>

namespace uifgo {

OutlierFilter::OutlierFilter(const Config& cfg) : cfg_(cfg) {}

std::vector<UwbRange> OutlierFilter::PreFilter(const UwbFrame& frame) const {
  std::vector<UwbRange> retained;
  for (const auto& r : frame.ranges) {
    // NLOS: rx_rssi - fp_rssi > threshold => multipath
    if (r.rx_rssi - r.fp_rssi > cfg_.nlos_rssi_diff) continue;
    // Range bounds
    if (r.dist < cfg_.min_range || r.dist > cfg_.max_range) continue;
    retained.push_back(r);
  }
  return retained;
}

bool OutlierFilter::ConsistencyCheck(const UwbRange& range,
                                      const gtsam::Point3& tag_pred_world,
                                      const gtsam::Point3& anchor_pos,
                                      bool warmed_up) const {
  if (!warmed_up) return true;
  double predicted = (gtsam::Vector3(tag_pred_world) - gtsam::Vector3(anchor_pos)).norm();
  return std::abs(predicted - range.dist) <= cfg_.dist_consistency_thresh;
}

}  // namespace uifgo
