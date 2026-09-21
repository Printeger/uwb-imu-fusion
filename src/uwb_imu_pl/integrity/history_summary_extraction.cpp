// C1-b Part B implementation: boundary representation extraction (see the
// header for the route comparison and the documented route-(ii) limitation).

#include "uwb_imu_pl/integrity/history_summary_extraction.hpp"

#include <gtsam/linear/JacobianFactor.h>

#include <Eigen/QR>
#include <boost/shared_ptr.hpp>
#include <map>

namespace uwb_imu_pl {

ExtractedBoundaryRows extractBoundaryRows(
    const gtsam::GaussianFactorGraph& reduced) {
  ExtractedBoundaryRows out;
  std::map<gtsam::Key, int> column_of_key;
  std::size_t rows = 0;
  for (const auto& factor : reduced) {
    const auto jacobian =
        boost::dynamic_pointer_cast<gtsam::JacobianFactor>(factor);
    if (!jacobian) {
      out.reason = "reduced factor is not a JacobianFactor";
      return out;
    }
    for (std::size_t local = 0; local < jacobian->keys().size(); ++local) {
      const gtsam::Key key = jacobian->keys()[local];
      if (column_of_key.find(key) == column_of_key.end()) {
        const int width =
            static_cast<int>(jacobian->getA(jacobian->begin() + local).cols());
        column_of_key.emplace(key, static_cast<int>(out.keys.size()));
        out.keys.push_back(key);
        out.column_begin.push_back(out.total_columns);
        out.key_dim.push_back(width);
        out.total_columns += width;
      }
    }
    rows += static_cast<std::size_t>(jacobian->getA().rows());
  }
  out.rows = Eigen::MatrixXd::Zero(rows, out.total_columns + 1);
  std::size_t cursor = 0;
  for (const auto& factor : reduced) {
    const auto jacobian =
        boost::dynamic_pointer_cast<gtsam::JacobianFactor>(factor);
    if (!jacobian) {
      out.reason = "reduced factor is not a JacobianFactor";
      return out;
    }
    const int factor_rows = static_cast<int>(jacobian->getA().rows());
    for (std::size_t local = 0; local < jacobian->keys().size(); ++local) {
      const int index = column_of_key.at(jacobian->keys()[local]);
      out.rows.block(cursor, out.column_begin[index], factor_rows,
                     out.key_dim[index]) =
          jacobian->getA(jacobian->begin() + local);
    }
    out.rows.block(cursor, out.total_columns, factor_rows, 1) =
        jacobian->getb();
    cursor += static_cast<std::size_t>(factor_rows);
  }
  // Rank audit (no truncation anywhere: the rows are handed over intact).
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(
      out.rows.leftCols(out.total_columns));
  const Eigen::VectorXd diagonal = qr.matrixR().diagonal().cwiseAbs();
  const double scale = diagonal.size() ? diagonal.maxCoeff() : 0.0;
  for (int index = 0; index < diagonal.size(); ++index) {
    if (scale > 0.0 && diagonal(index) > 1e-12 * scale) ++out.rank;
  }
  out.valid = true;
  return out;
}

ExtractedBoundaryRows extractBoundaryRowsFromInformation(
    const Eigen::MatrixXd& lambda, const Eigen::VectorXd& eta) {
  ExtractedBoundaryRows out;
  out.reason =
      "route (ii): no key identity and the normal-matrix constant (kappa / "
      "detector-only content) is not recoverable from (Lambda, eta)";
  if (lambda.rows() != lambda.cols() || lambda.rows() == 0 ||
      eta.size() != lambda.rows()) {
    out.reason = "route (ii): shape mismatch";
    out.valid = false;
    return out;
  }
  if (!lambda.allFinite() || !eta.allFinite()) {
    out.reason = "route (ii): non-finite input";
    return out;
  }
  Eigen::LLT<Eigen::MatrixXd> llt(lambda);
  if (llt.info() != Eigen::Success) {
    out.reason =
        "route (ii): symmetric Cholesky failed (rank-deficient information; "
        "route (i) required)";
    return out;
  }
  const Eigen::MatrixXd R = llt.matrixU();
  const Eigen::VectorXd y = llt.matrixU().solve(eta);
  out.rows = Eigen::MatrixXd(R.rows(), R.cols() + 1);
  out.rows.leftCols(R.cols()) = R;
  out.rows.col(R.cols()) = y;
  out.total_columns = static_cast<int>(R.rows());
  out.rank = static_cast<int>(R.rows());
  out.constant_energy = 0.0;  // dropped by hessian(); route (i) keeps it
  out.valid = true;
  return out;
}

}  // namespace uwb_imu_pl
