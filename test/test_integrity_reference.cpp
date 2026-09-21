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
