#include "uifgo/uwb_factor.h"
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/expressions.h>

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
    bool calib_lever, bool calib_anchor, bool calib_bias) {

  Pose3_  pose_(pose_key);
  Point3_ lever_   = calib_lever ? Point3_(lever_key) : Point3_(lever_init);
  Point3_ antenna_ = transformFrom(pose_, lever_);

  Double_ rho = BuildRange(antenna_, anchor_nominal, calib_anchor, anchor_key);

  Double_ predicted = calib_bias ? (rho + Double_(bias_key)) : rho;

  auto noise = noiseModel::Isotropic::Sigma(1, sigma);
  return boost::make_shared<ExpressionFactor<double>>(noise, measured_range, predicted);
}

}  // namespace uifgo
