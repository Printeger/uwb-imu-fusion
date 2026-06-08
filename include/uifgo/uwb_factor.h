#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Key.h>

namespace uifgo {

// Factory for tightly-coupled UWB range factor (GTSAM Expression).
//
// Residual:
//   r = || (A_nominal + dA) - (R * t_UI + p) || + beta - measured_range
//
// Parameters:
//   pose_key    : X(j)  Pose3  (always a variable)
//   lever_key   : L(0)  Point3 (variable if calib_lever, else constant leaf)
//   anchor_key  : A(m)  Point3 (variable if calib_anchor, else constant leaf)
//   bias_key    : Z(m)  double (variable if calib_bias, else constant leaf)
//   anchor_nominal : nominal world anchor position A_m
//   lever_init     : initial lever arm (used as constant when calib off)
//   measured_range : z_{j,m}
//   sigma          : time-adaptive noise sigma (from design doc §4.4)
//   calib_lever, calib_anchor, calib_bias : calibration switches
//
// Calibration switches are handled by using constant-expression leaves
// when a parameter is not being optimized; GTSAM Expression automatically
// collects only the variable keys needed.

gtsam::NonlinearFactor::shared_ptr MakeUwbFactor(
    gtsam::Key pose_key, gtsam::Key lever_key,
    gtsam::Key anchor_key, gtsam::Key bias_key,
    const gtsam::Point3& anchor_nominal,
    const gtsam::Point3& lever_init,
    double measured_range, double sigma,
    bool calib_lever, bool calib_anchor, bool calib_bias);

}  // namespace uifgo
