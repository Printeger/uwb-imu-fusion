#pragma once

#include "uifgo/types.h"
#include "uifgo/config.h"
#include <vector>

namespace uifgo {

struct InitResult {
  bool ok = false;
  gtsam::Pose3  T0;                     // initial body->world pose
  gtsam::Vector3 v0;                    // initial world velocity (zero)
  gtsam::Vector3 ba0;                   // initial accel bias
  gtsam::Vector3 bg0;                   // initial gyro bias
  gtsam::Vector3 gravity_world;         // (0, 0, -g) after alignment
};

class Initializer {
 public:
  explicit Initializer(const Config& cfg);

  // Detect static interval in [i0, i1) using accel norm variance.
  bool DetectStatic(const std::vector<ImuSample>& imu,
                    size_t i0, size_t i1,
                    double* accel_norm_mean = nullptr);

  // UWB trilateration: Gauss-Newton on sum-of-squared range errors.
  // Requires >= 4 non-coplanar anchors for 3D observability.
  bool Trilaterate(const std::vector<UwbRange>& ranges,
                   const std::vector<AnchorConfig>& anchors,
                   gtsam::Point3* p_out);

  // Full initialization pipeline.
  InitResult Run(const std::vector<ImuSample>& imu,
                 const std::vector<UwbFrame>& uwb_frames);

 private:
  Config cfg_;
};

}  // namespace uifgo
