// C1-b Part B implementation: boundary representation extraction (see the
// header for the route comparison and the documented route-(ii) limitation).

#include "uwb_imu_pl/integrity/history_summary_extraction.hpp"

#include <gtsam/linear/JacobianFactor.h>

#include <Eigen/QR>
#include <boost/shared_ptr.hpp>
#include <map>
#include <chrono>

namespace uwb_imu_pl {

ExtractedBoundaryRows extractBoundaryRows(
    const gtsam::GaussianFactorGraph& reduced) {
  return extractBoundaryRows(reduced,true);
}

ExtractedBoundaryRows extractBoundaryRows(
    const gtsam::GaussianFactorGraph& reduced, bool audit_rank,
    double* rank_audit_ms) {
  if(rank_audit_ms)*rank_audit_ms=0;
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
      const int width =
          static_cast<int>(jacobian->getA(jacobian->begin() + local).cols());
      if (column_of_key.find(key) == column_of_key.end()) {
        column_of_key.emplace(key, static_cast<int>(out.keys.size()));
        out.keys.push_back(key);
        out.column_begin.push_back(out.total_columns);
        out.key_dim.push_back(width);
        out.total_columns += width;
      } else if (out.key_dim[column_of_key.at(key)] != width) {
        out.reason = "inconsistent tangent dimension for repeated key " +
            std::to_string(static_cast<std::uint64_t>(key));
        return out;
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
    if (!jacobian->getA().allFinite() || !jacobian->getb().allFinite()) {
      out.reason = "input JacobianFactor contains a non-finite A or b";
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
  if (!out.rows.allFinite()) {
    out.reason = "assembled boundary rows became non-finite";
    return out;
  }
  // Rank audit (no truncation anywhere: the rows are handed over intact).
  if(audit_rank) {
  const auto start=std::chrono::steady_clock::now();
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(
      out.rows.leftCols(out.total_columns));
  const Eigen::VectorXd diagonal = qr.matrixR().diagonal().cwiseAbs();
  const double scale = diagonal.size() ? diagonal.maxCoeff() : 0.0;
  for (int index = 0; index < diagonal.size(); ++index) {
    if (scale > 0.0 && diagonal(index) > 1e-12 * scale) ++out.rank;
  }
  if(rank_audit_ms)*rank_audit_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  } else out.rank=-1;
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
