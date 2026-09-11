#include "uifgo/uwb_factor.h"
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/expressions.h>
#include <cmath>
#include <stdexcept>

namespace uifgo {
using namespace gtsam;

// Helper: build range expression rho = ||antenna - (anchor_nominal + dA[opt])||
static Double_ BuildRange(const Point3_& antenna, const Point3& A_nom,
                          bool calib_anchor, Key anchor_key) {
  if (calib_anchor) {
    auto f = [A_nom](const Point3& ant, const Point3& dA,
                     OptionalJacobian<1,3> Ha, OptionalJacobian<1,3> Hd) -> double {
      Vector3 d = ant - (A_nom + dA);
      double n = d.norm();
      if (n > 1e-9) { if(Ha) *Ha = d.transpose()/n; if(Hd) *Hd = -d.transpose()/n; }
      else { if(Ha) *Ha = Matrix13::Zero(); if(Hd) *Hd = Matrix13::Zero(); }
      return n;
    };
    return Double_(f, antenna, Point3_(anchor_key));
  } else {
    auto f = [A_nom](const Point3& ant, OptionalJacobian<1,3> H) -> double {
      Vector3 d = ant - A_nom;
      double n = d.norm();
      if (n > 1e-9) { if(H) *H = d.transpose()/n; } else { if(H) *H = Matrix13::Zero(); }
      return n;
    };
    return Double_(f, antenna);
  }
}

NonlinearFactor::shared_ptr MakeUwbFactor(
    Key pose_key, Key lever_key, Key anchor_key, Key bias_key,
    const Point3& anchor_nominal, const Point3& lever_init,
    double measured_range, double sigma,
    bool calib_lever, bool calib_anchor, bool calib_bias,
    double fixed_beta) {

  if (!std::isfinite(fixed_beta)) {
    throw std::invalid_argument("MakeUwbFactor: fixed_beta must be finite");
  }
  if (calib_bias && fixed_beta != 0.0) {
    throw std::invalid_argument(
        "MakeUwbFactor: fixed and online range bias are mutually exclusive");
  }

  Pose3_  pose_(pose_key);
  Point3_ lever_   = calib_lever ? Point3_(lever_key) : Point3_(lever_init);
  Point3_ antenna_ = transformFrom(pose_, lever_);

  Double_ rho = BuildRange(antenna_, anchor_nominal, calib_anchor, anchor_key);

  Double_ predicted = rho;
  if (calib_bias) {
    predicted = rho + Double_(bias_key);
  } else if (fixed_beta != 0.0) {
    predicted = rho + Double_(fixed_beta);
  }

  auto noise = noiseModel::Isotropic::Sigma(1, sigma);
  return boost::make_shared<ExpressionFactor<double>>(noise, measured_range, predicted);
}

NonlinearFactor::shared_ptr MakeSegmentUwbFactor(
    Key pose_key, Key segment_key, const Point3& anchor_nominal,
    const Point3& lever_init, double measured_range, double sigma,
    double fixed_beta) {
  if (!(sigma > 0.0) || !std::isfinite(sigma)) {
    throw std::invalid_argument(
        "MakeSegmentUwbFactor: sigma must be finite and positive");
  }
  if (!std::isfinite(fixed_beta) || !std::isfinite(measured_range)) {
    throw std::invalid_argument(
        "MakeSegmentUwbFactor: range and fixed_beta must be finite");
  }

  Pose3_ pose(pose_key);
  Point3_ antenna = transformFrom(pose, Point3_(lever_init));
  Double_ rho = BuildRange(antenna, anchor_nominal, false, 0);
  Double_ predicted = rho + Double_(fixed_beta) + Double_(segment_key);
  auto noise = noiseModel::Isotropic::Sigma(1, sigma);
  return boost::make_shared<ExpressionFactor<double>>(noise, measured_range,
                                                       predicted);
}

NonlinearFactor::shared_ptr MakeOnlineBetaSegmentUwbFactor(
    Key pose_key, Key online_beta_key, Key segment_key,
    const Point3& anchor_nominal, const Point3& lever_init,
    double measured_range, double sigma) {
  if (!(sigma > 0.0) || !std::isfinite(sigma) ||
      !std::isfinite(measured_range))
    throw std::invalid_argument("online-beta segment factor input is invalid");
  Pose3_ pose(pose_key);
  Point3_ antenna = transformFrom(pose, Point3_(lever_init));
  Double_ rho = BuildRange(antenna, anchor_nominal, false, 0);
  Double_ predicted = rho + Double_(online_beta_key) + Double_(segment_key);
  return boost::make_shared<ExpressionFactor<double>>(
      noiseModel::Isotropic::Sigma(1, sigma), measured_range, predicted);
}

double UwbResidual(double geometric_range, double measured_range,
                   double fixed_beta, double online_beta) {
  return geometric_range + fixed_beta + online_beta - measured_range;
}

double SegmentUwbResidual(double geometric_range, double measured_range,
                          double fixed_beta, double segment_amplitude) {
  return geometric_range + fixed_beta + segment_amplitude - measured_range;
}

double RangeForGeometryInitialization(double measured_range,
                                      double fixed_beta) {
  return measured_range - fixed_beta;
}

}  // namespace uifgo

namespace uifgo {
gtsam::NonlinearFactor::shared_ptr MakeFixedOffsetUwbFactor(
    gtsam::Key pose_key, const gtsam::Point3& anchor,
    const gtsam::Point3& lever, double raw_range, double sigma,
    double fixed_beta, double delta) {
  if (!std::isfinite(delta) || delta < 0 || !std::isfinite(raw_range) ||
      !std::isfinite(sigma) || sigma <= 0 || !std::isfinite(fixed_beta+delta))
    throw std::invalid_argument("invalid fixed dynamic offset factor");
  return MakeUwbFactor(pose_key,0,0,0,anchor,lever,raw_range,sigma,
                       false,false,false,fixed_beta+delta);
}
}
