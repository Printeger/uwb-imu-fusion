// A4 analytic and independent-reference fixtures (roadmap section 10.1 GEO-*).
//
// Every quantity in this file is recomputed with plain dense linear algebra
// written in the test itself (JacobiSVD / small solves).  The production
// solvers (frozen window numerics, rank-update kernel, hypothesis evidence,
// protection level) are deliberately NOT used as the oracle here.  The only
// production functions touched are the objects under test: the frozen-window
// assembly (`finalizeIntegrityWindow`), the row whitener convention, and the
// protected-state Jacobian under the D1 body-origin contract.

#include "uwb_imu_pl/common/failure_reason.hpp"
#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

#include <Eigen/SVD>

#include <cmath>
#include <vector>

namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr double kAtol = 1e-12;
constexpr double kRtol = 1e-10;

MatrixXd pinv(const MatrixXd& matrix, double rank_tolerance = 1e-10) {
  Eigen::JacobiSVD<MatrixXd> svd(matrix, Eigen::ComputeThinU |
                                             Eigen::ComputeThinV);
  const VectorXd singular = svd.singularValues();
  const double largest = singular.size() ? singular(0) : 0.0;
  const double gate = rank_tolerance * std::max(1.0, largest);
  MatrixXd inverse = MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());
  for (Eigen::Index i = 0; i < singular.size(); ++i) {
    if (singular(i) > gate) inverse(i, i) = 1.0 / singular(i);
  }
  return svd.matrixV() * inverse * svd.matrixU().transpose();
}

struct LeastSquaresReference {
  int rows = 0;
  int columns = 0;
  int rank = 0;
  int nu = 0;
  MatrixXd covariance;       // (H^T H)^+ = P
  MatrixXd projector;        // I - H P H^T = M
  MatrixXd covariance_solution(const MatrixXd& rhs) const {
    return covariance * rhs;
  }
};

LeastSquaresReference leastSquares(const MatrixXd& H) {
  LeastSquaresReference out;
  out.rows = static_cast<int>(H.rows());
  out.columns = static_cast<int>(H.cols());
  Eigen::JacobiSVD<MatrixXd> svd(H);
  const VectorXd singular = svd.singularValues();
  const double largest = singular.size() ? singular(0) : 0.0;
  const double gate = kRtol * std::max(1.0, largest);
  out.rank = static_cast<int>((singular.array() > gate).count());
  out.nu = out.rows - out.rank;
  out.covariance = pinv(H.transpose() * H);
  out.projector = MatrixXd::Identity(out.rows, out.rows) -
      H * out.covariance * H.transpose();
  return out;
}

// Pure density-operator reference: G = C (H^T H)^+ H^T A, Z = M A, s^2 from
// the rank-revealing factor of Gamma.
struct SlopeReference {
  MatrixXd G;
  MatrixXd Z;
  MatrixXd Gamma;
  int rank = 0;
  VectorXd slopes;
};

SlopeReference slopeReference(const MatrixXd& H, const MatrixXd& A,
                              const MatrixXd& C) {
  SlopeReference out;
  const LeastSquaresReference ls = leastSquares(H);
  out.G = C * ls.covariance * H.transpose() * A;
  out.Z = ls.projector * A;
  out.Gamma = out.Z.transpose() * out.Z;
  Eigen::JacobiSVD<MatrixXd> svd(out.Gamma, Eigen::ComputeThinU |
                                                Eigen::ComputeThinV);
  const VectorXd singular = svd.singularValues();
  const double largest = singular.size() ? singular(0) : 0.0;
  const double gate = 1e-10 * std::max(1.0, largest);
  out.rank = static_cast<int>((singular.array() > gate).count());
  out.slopes = VectorXd::Constant(C.rows(),
                                  std::numeric_limits<double>::infinity());
  if (out.rank == 0) return out;
  // s^2 = g Gamma^+ g^T = sum_i (v_i^T g^T)^2 / sigma_i over retained modes.
  MatrixXd scaled = MatrixXd::Zero(out.Gamma.rows(), out.rank);
  for (int i = 0; i < out.rank; ++i) {
    scaled.col(i) = svd.matrixV().col(i) / std::sqrt(singular(i));
  }
  for (int axis = 0; axis < C.rows(); ++axis) {
    const VectorXd response = out.G.row(axis).transpose();
    out.slopes(axis) = std::sqrt((scaled.transpose() * response).squaredNorm());
  }
  return out;
}

// ---------------------------------------------------------------------------
// GEO-01: two-row scalar least squares (roadmap section 6 / A4)
//   H = (1,1)^T, A = (1,0)^T, C = 1
//   nu = 1, P = 1/2, G = 1/2, Gamma = 1/2, s^2 = 1/2, finite model PL.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, TwoRowScalarLeastSquares) {
  MatrixXd H(2, 1);
  H << 1.0, 1.0;
  MatrixXd A(2, 1);
  A << 1.0, 0.0;
  MatrixXd C(1, 1);
  C << 1.0;
  const LeastSquaresReference ls = leastSquares(H);
  EXPECT_EQ(ls.rank, 1);
  EXPECT_EQ(ls.nu, 1);
  EXPECT_NEAR(ls.covariance(0, 0), 0.5, kAtol);
  const SlopeReference slope = slopeReference(H, A, C);
  EXPECT_NEAR(slope.G(0, 0), 0.5, kAtol);
  EXPECT_NEAR(slope.Gamma(0, 0), 0.5, kAtol);
  EXPECT_NEAR(slope.slopes(0) * slope.slopes(0), 0.5, kRtol);
  EXPECT_TRUE(std::isfinite(slope.slopes(0)));
  // Square-convention guard (roadmap 5.1): Gamma is already the squared
  // noncentrality; a finite model PL uses slope * sqrt(Lambda), never a second
  // squaring of slope.
  const double lambda = 9.0;  // arbitrary finite boundary for the identity check
  const double pl_like = slope.slopes(0) * std::sqrt(lambda);
  EXPECT_NEAR(pl_like, std::sqrt(0.5 * lambda), kRtol);
}

// ---------------------------------------------------------------------------
// GEO-02: one-dimensional completely undetectable fault.  The first diagnostic
// level must already report an unmonitorable direction; this is a fault
// geometry result, not evidence of a template bug.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, OneDimensionalUndetectableFault) {
  MatrixXd H(1, 1);
  H << 1.0;
  MatrixXd A(1, 1);
  A << 1.0;
  MatrixXd C(1, 1);
  C << 1.0;
  const SlopeReference slope = slopeReference(H, A, C);
  EXPECT_EQ(slope.rank, 0);
  EXPECT_NEAR(slope.Gamma(0, 0), 0.0, kAtol);
  EXPECT_FALSE(std::isfinite(slope.slopes(0)));
  EXPECT_EQ(uwb_imu_pl::classifyFailureReason("fault subspace is unmonitorable"),
            uwb_imu_pl::FailureReason::DangerousFaultNullspace);
}

// ---------------------------------------------------------------------------
// GEO-04: q > nu with a harmless nuisance nullspace.  H = [[1,0],[0,1],[1,0]],
// A = [e2, e1], C = [1,0] gives q=2 > nu=1, G = (0, 1/2), ker Z subset ker G
// and a finite protected bound s^2 = 1/2.  q<=nu or a dof margin must never be
// turned into a hard necessity.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, HarmlessNuisanceNullspaceKeepsFiniteBound) {
  MatrixXd H(3, 2);
  H << 1.0, 0.0,
       0.0, 1.0,
       1.0, 0.0;
  MatrixXd A(3, 2);
  A << 0.0, 1.0,
       1.0, 0.0,
       0.0, 0.0;
  MatrixXd C(1, 2);
  C << 1.0, 0.0;
  const LeastSquaresReference ls = leastSquares(H);
  EXPECT_EQ(ls.rank, 2);
  EXPECT_EQ(ls.nu, 1);
  const SlopeReference slope = slopeReference(H, A, C);
  EXPECT_EQ(slope.G.cols(), 2);
  EXPECT_NEAR(slope.G(0, 0), 0.0, kAtol);
  EXPECT_NEAR(slope.G(0, 1), 0.5, kAtol);
  EXPECT_EQ(slope.rank, 1);
  // Every null direction of Z must also be a null direction of G (structural
  // harmless-nullspace certificate): verify with the exact annihilated column.
  EXPECT_NEAR(slope.Z.col(0).norm(), 0.0, kAtol);
  EXPECT_NEAR(slope.G(0, 0), 0.0, kAtol);
  EXPECT_NEAR(slope.slopes(0) * slope.slopes(0), 0.5, kRtol);
  EXPECT_TRUE(std::isfinite(slope.slopes(0)));
}

// ---------------------------------------------------------------------------
// Counterexample A: the action label must not select the kernel.  Deleting the
// process constraint row leaves the position observable; the auxiliary-subset
// covariance difference is 1/3 rather than infinity.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, RemovingProcessConstraintKeepsFiniteSlope) {
  MatrixXd H(3, 2);
  H << 1.0, 0.0,
       0.0, 1.0,
       -1.0, 1.0;
  MatrixXd C(1, 2);
  C << 0.0, 1.0;
  const LeastSquaresReference full = leastSquares(H);
  const double full_slope_squared =
      (C * full.covariance * C.transpose())(0, 0);
  EXPECT_EQ(full.rank, 2);
  EXPECT_NEAR(full_slope_squared, 2.0 / 3.0, kRtol);
  MatrixXd H_subset(2, 2);
  H_subset << 1.0, 0.0,
              0.0, 1.0;
  const LeastSquaresReference subset = leastSquares(H_subset);
  const double subset_slope_squared =
      (C * subset.covariance * C.transpose())(0, 0);
  EXPECT_NEAR(subset_slope_squared, 1.0, kRtol);
  const double growth = (C * (subset.covariance - full.covariance) *
                         C.transpose())(0, 0);
  EXPECT_NEAR(growth, 1.0 / 3.0, kRtol);
  EXPECT_TRUE(std::isfinite(subset_slope_squared));
}

// ---------------------------------------------------------------------------
// Counterexample C: four fixed anchors do not imply a fixed geometry Jacobian.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, FixedAnchorGeometryRankDependsOnTarget) {
  const std::vector<Eigen::Vector3d> anchors = {
      {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0}};
  auto rangeJacobian = [&](const Eigen::Vector3d& target) {
    MatrixXd jacobian(anchors.size(), 3);
    for (std::size_t i = 0; i < anchors.size(); ++i) {
      const Eigen::Vector3d delta = target - anchors[i];
      jacobian.row(static_cast<Eigen::Index>(i)) = delta.transpose() / delta.norm();
    }
    return jacobian;
  };
  Eigen::JacobiSVD<MatrixXd> at_origin(rangeJacobian({0, 0, 0}));
  Eigen::JacobiSVD<MatrixXd> elevated(rangeJacobian({0, 0, 1}));
  const double gate = 1e-10;
  const int rank_origin = static_cast<int>(
      (at_origin.singularValues().array() > gate).count());
  const int rank_elevated = static_cast<int>(
      (elevated.singularValues().array() > gate).count());
  EXPECT_EQ(rank_origin, 2);
  EXPECT_EQ(rank_elevated, 3);
}

// ---------------------------------------------------------------------------
// Counterexample B as a specification test: an early instantaneous template is
// not a subspace of a later "recent-k free profile".  This repository has no
// recent-k concept; the test exists so a future implementation cannot silently
// assume a nested coverage ladder.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, TemplateAndRecentProfileAreNotNested) {
  const Eigen::Vector4d early = (Eigen::Vector4d() << 1, 0, 0, 0).finished();
  MatrixXd recent(4, 2);
  recent << 0, 0,
            0, 0,
            1, 0,
            0, 1;
  const LeastSquaresReference profile = leastSquares(recent);
  const VectorXd residual =
      early - recent * (profile.covariance * recent.transpose() * early);
  EXPECT_NEAR(residual.norm(), 1.0, kRtol);  // early is NOT contained
  // Both are contained in the full four-dimensional space; the coverage graph
  // must therefore carry explicit inclusion evidence instead of relying on a
  // template/recent-k/free naming ladder.
  EXPECT_TRUE(std::isfinite(residual.norm()));
}

// ---------------------------------------------------------------------------
// Whitening convention formalized (A4 item 10): for every frozen block the
// stored whitened Jacobian equals W * raw and the aggregate H is exactly the
// stack of whitened blocks.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, FrozenBlockWhiteningAndAggregateAssembly) {
  using namespace uwb_imu_pl;
  LinearizedIntegrityWindow window;
  window.id = WindowId(11);
  window.version = {1, 2, 3, 4};
  auto addBlock = [&](std::uint64_t id, const MatrixXd& raw,
                      const VectorXd& residual, const MatrixXd& covariance) {
    LinearizedFactorBlock block;
    block.group_id = FactorGroupId(id);
    block.kind = FactorKind::UwbBatch;
    block.sensor = SensorType::Uwb;
    Eigen::LLT<MatrixXd> llt(covariance);
    ASSERT_EQ(llt.info(), Eigen::Success);
    block.whitener = llt.matrixL().solve(
        MatrixXd::Identity(covariance.rows(), covariance.cols()));
    block.jacobian_raw = raw;
    block.residual_raw = residual;
    block.covariance = covariance;
    block.jacobian_whitened = block.whitener * raw;
    block.residual_whitened = block.whitener * residual;
    block.version = window.version;
    window.blocks.push_back(std::move(block));
  };
  MatrixXd raw0(2, 2);
  raw0 << 1.0, 0.2, 0.1, 1.0;
  MatrixXd cov0 = MatrixXd::Identity(2, 2) * 0.04;
  cov0(0, 1) = cov0(1, 0) = 0.005;
  addBlock(1, raw0, VectorXd::Zero(2), cov0);
  MatrixXd raw1(2, 2);
  raw1 << 0.5, -0.3, 0.2, 0.9;
  addBlock(2, raw1, VectorXd::Zero(2), MatrixXd::Identity(2, 2) * 0.01);
  window.protected_state_map = MatrixXd::Zero(3, 2);
  window.protected_state_map(0, 0) = 1.0;
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  ASSERT_TRUE(window.model_valid) << window.reason;
  for (const auto& block : window.blocks) {
    const double error = (block.whitener * block.jacobian_raw -
                          block.jacobian_whitened).cwiseAbs().maxCoeff();
    EXPECT_LE(error, 1e-14);
  }
  MatrixXd stacked(4, 2);
  stacked.topRows(2) = window.blocks[0].jacobian_whitened;
  stacked.bottomRows(2) = window.blocks[1].jacobian_whitened;
  EXPECT_LE((stacked - window.H).cwiseAbs().maxCoeff(), 0.0);
}

// ---------------------------------------------------------------------------
// D1: the protected map is the body-origin world position under the GTSAM
// Pose3 local tangent.  Verified by central finite differences; a non-zero
// lever arm belongs to the tag and is NOT part of C.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, ProtectedMapIsBodyOriginFiniteDifference) {
  using namespace uwb_imu_pl;
  const gtsam::Pose3 pose(
      gtsam::Rot3::RzRyRx(0.21, -0.14, 0.33), gtsam::Point3(1.2, -0.4, 0.8));
  const Eigen::Matrix<double, 3, 6> C =
      worldPositionPoseTangentJacobian(pose);
  const Eigen::Matrix3d rotation = pose.rotation().matrix();
  EXPECT_LE((C.rightCols<3>() - rotation).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_LE(C.leftCols<3>().cwiseAbs().maxCoeff(), 0.0);
  const double h = 1e-6;
  for (int axis = 0; axis < 6; ++axis) {
    VectorXd step = VectorXd::Zero(6);
    step(axis) = h;
    const Eigen::Vector3d plus = pose.retract(step).translation();
    const Eigen::Vector3d minus = pose.retract(-step).translation();
    const Eigen::Vector3d numerical = (plus - minus) / (2.0 * h);
    // Derivative direction: C applied to the unit perturbation (not to the
    // increment itself).
    const Eigen::Vector3d analytic = C * (step / h);
    EXPECT_LE((numerical - analytic).cwiseAbs().maxCoeff(), 1e-7)
        << "axis " << axis;
  }
  // D1: the tag offset would add the rotational coupling -R [v]_x; confirm the
  // declared contract explicitly excludes it from C.
  const Eigen::Vector3d lever_arm(0.30, 0.20, 0.15);
  Eigen::Matrix3d skew;
  skew << 0.0, -lever_arm.z(), lever_arm.y(),
          lever_arm.z(), 0.0, -lever_arm.x(),
          -lever_arm.y(), lever_arm.x(), 0.0;
  const Eigen::Matrix3d tag_rotation_part = -rotation * skew;
  EXPECT_GT(tag_rotation_part.cwiseAbs().maxCoeff(), 0.05);
  EXPECT_LE(C.leftCols<3>().cwiseAbs().maxCoeff(), 0.0)
      << "C must not contain the tag lever-arm coupling";
  // Consistency of the FD with the skew part above: perturbing the rotation
  // direction moves a *tag* point, not the body origin, by R [v]_x^T.
}

}  // namespace
// ---------------------------------------------------------------------------
// GEO-05 (B2): double-fault residual cancellation.  Two single faults are each
// monitorable, but their union cancels in the detection space while moving the
// protected state: the pair is dangerous and the cross term Gamma_12 cannot be
// dropped (the diagonal-only Gram would understate the risk).
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, GEO05DoubleFaultCancellationNeedsTheCrossTerm) {
  const int m = 12, n = 3;
  MatrixXd H = MatrixXd::Zero(m, n);
  for (int row = 0; row < m; ++row) {
    H(row, 0) = 1.0;
    H(row, 1) = 0.25 * row;
    H(row, 2) = (row % 3 == 0) ? 1.0 : 0.0;
  }
  MatrixXd C = MatrixXd::Zero(2, n);
  C(0, 0) = 1.0;
  C(1, 1) = 1.0;

  // d2 = H w - d1 makes the pair direction (1, 1) collapse in parity space
  // (A v = H w) while the protected response stays C w != 0.
  MatrixXd d1 = MatrixXd::Zero(m, 1);
  for (int row = 0; row < m; ++row) d1(row, 0) = (row % 2 == 0) ? 1.0 : -1.0;
  const VectorXd w = (VectorXd(3) << 0.3, -0.2, 0.1).finished();
  const MatrixXd d2 = H * w - d1;
  const LeastSquaresReference ls = leastSquares(H);
  const SlopeReference single1 = slopeReference(H, d1, C);
  const SlopeReference single2 = slopeReference(H, d2, C);
  EXPECT_TRUE(single1.slopes.allFinite());
  EXPECT_TRUE(single2.slopes.allFinite());
  EXPECT_GT(single1.Gamma(0, 0), 1e-3);
  EXPECT_GT(single2.Gamma(0, 0), 1e-3);

  // Double faults add columns to the *same* window: A = [d1 | d2].
  MatrixXd A(m, 2);
  A << d1, d2;
  const SlopeReference pair = slopeReference(H, A, C);
  // The exact Gram of the pair is singular: there is a blind combination.
  EXPECT_LT(pair.rank, 2) << "cross term must couple the two faults";
  Eigen::JacobiSVD<MatrixXd> pair_svd(pair.Gamma, Eigen::ComputeThinU |
                                                       Eigen::ComputeThinV);
  const double largest = pair_svd.singularValues()(0);
  const VectorXd blind = pair_svd.matrixV().col(1);  // null direction of Gamma
  const double blind_gain = pair_svd.singularValues()(1);
  EXPECT_LT(blind_gain, 1e-8 * std::max(1.0, largest));
  // The blind direction is undetectable ...
  const VectorXd detection = pair.Z * blind;
  EXPECT_LT(detection.norm(), 1e-8);
  // ... but it moves the protected state: the combination is dangerous.
  const VectorXd protected_motion = pair.G * blind;
  EXPECT_GT(protected_motion.norm(), 1e-3);

  // Dropping the cross term would make the same direction look detectable, so
  // the cross term cannot be dropped (it hides the dangerous combination).
  MatrixXd Gamma_diagonal = pair.Gamma;
  Gamma_diagonal(0, 1) = 0.0;
  Gamma_diagonal(1, 0) = 0.0;
  Eigen::JacobiSVD<MatrixXd> diagonal_svd(Gamma_diagonal);
  EXPECT_GT(diagonal_svd.singularValues().minCoeff(), 1e-3);
  // Known limitation the Stage 3 grouped envelope must cover: the rank
  // truncated exact bound stays finite even though the pair is dangerous.
  EXPECT_TRUE(pair.slopes.allFinite());
}

// ---------------------------------------------------------------------------
// CFG-03 (D round): fixed anchors, moving platform.  The startup checks are a
// configuration-level pass; the runtime geometry must still be re-validated
// per frozen window.  The same four anchors give a rank-3 range Jacobian at
// (0,0,1) and a rank-2 (state-deficient) one at (0,0,0), so a startup pass
// must never be inherited by a runtime window.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, StartupPassDoesNotMaskRuntimeGeometryDegradation) {
  using namespace uwb_imu_pl;
  const std::vector<Eigen::Vector3d> anchors = {
      {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0}};
  auto rangeJacobian = [&](const Eigen::Vector3d& target) {
    MatrixXd jacobian(anchors.size(), 3);
    for (std::size_t i = 0; i < anchors.size(); ++i) {
      const Eigen::Vector3d delta = target - anchors[i];
      jacobian.row(static_cast<Eigen::Index>(i)) =
          delta.transpose() / delta.norm();
    }
    return jacobian;
  };
  auto buildWindow = [&](const Eigen::Vector3d& target) {
    LinearizedIntegrityWindow window;
    window.id = WindowId(7);
    window.version = {1, 1, 1, 1};
    LinearizedFactorBlock block;
    block.group_id = FactorGroupId(1);
    block.kind = FactorKind::UwbBatch;
    block.sensor = SensorType::Uwb;
    const MatrixXd raw = rangeJacobian(target);
    block.whitener = MatrixXd::Identity(anchors.size(), anchors.size());
    block.jacobian_raw = raw;
    block.residual_raw = VectorXd::Zero(static_cast<Eigen::Index>(anchors.size()));
    block.covariance = MatrixXd::Identity(anchors.size(), anchors.size());
    block.jacobian_whitened = block.whitener * raw;
    block.residual_whitened = VectorXd::Zero(static_cast<Eigen::Index>(anchors.size()));
    block.version = window.version;
    window.blocks.push_back(std::move(block));
    window.protected_state_map = MatrixXd::Identity(3, 3);
    finalizeIntegrityWindow(&window, 1e-10, 1e10);
    return window;
  };
  const auto elevated = buildWindow({0, 0, 1});
  EXPECT_TRUE(elevated.model_valid) << elevated.reason;
  EXPECT_EQ(elevated.rank, 3);
  EXPECT_EQ(elevated.dof, 1);
  const auto at_origin = buildWindow({0, 0, 0});
  EXPECT_FALSE(at_origin.model_valid)
      << "runtime geometry degradation must not be masked by a startup pass";
  EXPECT_EQ(at_origin.rank, 2);
  EXPECT_NE(at_origin.reason.find("rank deficient"), std::string::npos)
      << at_origin.reason;
}

// ---------------------------------------------------------------------------
// GEO-03 (D round): multiple alternate one-dimensional modes with K > nu.  A
// count-based rule ("dictionary bigger than nu -> reject") is wrong in both
// directions; the classification must come from the parity/kernel analysis.
// Here m = 4, rank(H) = 2 (nu = 2) and the dictionary carries four
// one-dimensional modes, K > nu:
//   * d1, d2 stay detectable and span the full 2-D parity content;
//   * d3 = H w3 is blind but harmless (G d3 = 0, GEO-04 contrast);
//   * d4 = H w4 is blind and dangerous (G d4 != 0).
// The verdict is driven by the kernel direction, not by K: with d4 removed the
// K = 3 > nu dictionary keeps a finite bound; with d4 present the dangerous
// kernel direction is identified even though most of the dictionary is fine.
// ---------------------------------------------------------------------------
TEST(ReferenceFixture, Geo03KGreaterThanNuIsClassifiedByTheKernelNotByCounts) {
  MatrixXd H(4, 2);
  H << 1.0, 0.0,
       0.0, 1.0,
       1.0, 1.0,
       1.0, -1.0;
  const LeastSquaresReference ls = leastSquares(H);
  EXPECT_EQ(ls.rank, 2);
  EXPECT_EQ(ls.nu, 2);
  MatrixXd C(1, 2);
  C << 1.0, 0.0;

  const VectorXd w3 = (VectorXd(2) << 0.0, 0.4).finished();   // G d3 = 0
  const VectorXd w4 = (VectorXd(2) << 0.5, 0.0).finished();   // G d4 != 0
  MatrixXd A(4, 4);
  A.col(0) << 1.0, 0.0, 0.0, 0.0;
  A.col(1) << 0.0, 1.0, 0.0, 0.0;
  A.col(2) = H * w3;
  A.col(3) = H * w4;
  const SlopeReference mixed = slopeReference(H, A, C);
  EXPECT_EQ(mixed.rank, 2);  // the detection content lives in parity space
  EXPECT_GT(mixed.Z.col(0).norm(), 0.1);
  EXPECT_GT(mixed.Z.col(1).norm(), 0.1);
  EXPECT_NEAR(mixed.Z.col(2).norm(), 0.0, kAtol);  // blind mode
  EXPECT_NEAR(mixed.Z.col(3).norm(), 0.0, kAtol);  // blind mode
  Eigen::JacobiSVD<MatrixXd> z_svd(mixed.Z, Eigen::ComputeThinU |
                                                Eigen::ComputeThinV);
  const VectorXd z_singular = z_svd.singularValues();
  const double z_gate = 1e-10 * std::max(1.0, z_singular(0));
  const int z_rank =
      static_cast<int>((z_singular.array() > z_gate).count());
  EXPECT_EQ(mixed.Z.cols() - z_rank, 2);
  // Kernel analysis: a dangerous kernel direction exists (G restricted to
  // ker Z is non-zero), and it comes from d4, not from the dictionary size.
  double worst_kernel_gain = 0.0;
  for (int i = z_rank; i < mixed.Z.cols(); ++i) {
    worst_kernel_gain = std::max(
        worst_kernel_gain, (mixed.G * z_svd.matrixV().col(i)).cwiseAbs().maxCoeff());
  }
  EXPECT_GT(worst_kernel_gain, 0.1)
      << "the present dangerous direction must be identified by the kernel "
         "test";

  // Removing the dangerous mode leaves a K = 3 > nu = 2 dictionary whose only
  // blind direction is harmless: the bound stays finite.  K > nu alone never
  // rejects (counts are not the trigger).
  MatrixXd harmless(4, 3);
  harmless << A.col(0), A.col(1), A.col(2);
  const SlopeReference fine = slopeReference(H, harmless, C);
  EXPECT_EQ(fine.rank, 2);
  EXPECT_TRUE(std::isfinite(fine.slopes(0)));
  EXPECT_GT(fine.slopes(0), 0.0);
  Eigen::JacobiSVD<MatrixXd> fine_svd(fine.Z, Eigen::ComputeThinU |
                                                  Eigen::ComputeThinV);
  const VectorXd fine_singular = fine_svd.singularValues();
  const double fine_gate = 1e-10 * std::max(1.0, fine_singular(0));
  const int fine_rank =
      static_cast<int>((fine_singular.array() > fine_gate).count());
  double fine_kernel_gain = 0.0;
  for (int i = fine_rank; i < fine.Z.cols(); ++i) {
    fine_kernel_gain = std::max(
        fine_kernel_gain,
        (fine.G * fine_svd.matrixV().col(i)).cwiseAbs().maxCoeff());
  }
  EXPECT_LT(fine_kernel_gain, 1e-9)
      << "the only blind direction is harmless; K > nu must not blanket-reject";
}
