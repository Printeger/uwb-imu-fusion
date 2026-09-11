#pragma once
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include "uifgo/nlos_recoverability.h"
namespace uifgo {
// New tests use Boost.Math quantiles; the legacy Chi2inv table is untouched.
double FdeChiSquareQuantile(size_t dof, double probability = .99);
struct FdeQuadraticTest {
  bool valid = false, rejected = false;
  std::string reason;
  int rank = 0;
  double statistic = 0, threshold = 0, rank_threshold = 0;
  double null_component_norm = 0;
  Eigen::VectorXd spectrum;
};
FdeQuadraticTest FdeCovarianceTest(const Eigen::VectorXd& residual,
                                 const Eigen::MatrixXd& covariance);
struct FdeProjectionBlock {
  ResidualProjectionResult certificate;
  Eigen::MatrixXd Pww, response; // response=A+ E_W, original state units
  Eigen::VectorXd projected_residual;
  double symmetry_error=0, idempotence_error=0, orthogonality_error=0;
};
FdeProjectionBlock FdeFullProjectionBlock(const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& e, const std::vector<size_t>& rows);
} // namespace uifgo
