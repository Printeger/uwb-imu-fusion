#pragma once

// C1-b Part B (D-3): boundary representation extraction prototypes.
//
// The boundary construction must hand the history-summary machinery the
// whitened rows [H | z] of the boundary system, without eigendecomposition
// truncation and with correct degrees-of-freedom accounting.  Three routes
// are implemented/measured for comparison:
//
//   (i) SELECTED - `extractBoundaryRows(linearized)`: read the Jacobian rows
//       of the *linearized boundary graph* (`boundary_graph.linearize(
//       frozen_values)`, no elimination yet) and let the C1-a module perform
//       the orthogonal elimination itself.  Rows and the residual column are
//       intact, so the detector content (kappa / perp rows) survives.
//
//   (i-b) COMPARISON - `extractBoundaryRows(reduced)` on the result of
//       `eliminatePartialMultifrontal(..., EliminateQR)`: measured finding
//       (locked by test) - GTSAM elimination preserves the normal/
//       square-root content up to a per-factor additive constant, so the
//       constants of partially eliminated factors are dropped and kappa can
//       no longer be reproduced; fully eliminated factors survive as
//       constant rows only when their residual happens to stay in the graph.
//       Not usable as the sole source of the detector content.
//
//   (ii) COMPARISON - `extractBoundaryRowsFromInformation` (design freeze
//       §11.3 fallback): symmetric Cholesky of the reduced normal matrix
//       (Lambda, eta).  No key identity and no constant either (the same
//       measurement shows kappa is not recoverable); a rank-deficient Lambda
//       also refuses instead of fabricating a factor.
//
// Status: module-level prototypes implemented and locked by
// `HistorySummaryExtraction.*` tests; the pipeline is NOT wired (Part C).
// Complexity note (decision record): the selected route operates on the full
// dense boundary rows; the mature-window cost is measured and recorded there
// (sparse/multifrontal co-design is a C-round concern).

#include <gtsam/linear/GaussianFactorGraph.h>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace uwb_imu_pl {

struct ExtractedBoundaryRows {
  bool valid = false;
  std::string reason;
  // factor rows x (n + 1): [H | z]; the z column carries the (whitened)
  // residual right-hand side.  Key blocks are expanded by their tangent
  // dimension (x/b: 6, v: 3), so n is the number of VARIABLES, not keys.
  Eigen::MatrixXd rows;
  // Column i of H corresponds to keys[i], occupying [column_begin[i],
  // column_begin[i] + key_dim[i]).  Route (i)/(i-b) fill this; route (ii)
  // has no key identity available from (Lambda, eta) and leaves it empty
  // (documented limitation).
  std::vector<gtsam::Key> keys;
  std::vector<int> column_begin;
  std::vector<int> key_dim;
  int total_columns = 0;
  // Audit: rank of H as adjudicated by the extractor, and the squared norm of
  // the residual constant that the route can still represent (0 for the
  // Cholesky route: documented, not a bug).
  int rank = 0;
  double constant_energy = 0.0;

  int data_columns() const { return total_columns; }
};

// Route (i)/(i-b): every factor of the input graph must be a JacobianFactor
// (both the linearized boundary graph and a QR-reduced graph qualify); keys
// are mapped to columns by first appearance.
ExtractedBoundaryRows extractBoundaryRows(
    const gtsam::GaussianFactorGraph& graph);

// Route (ii): R = chol(Lambda); rows = [R | y] with R^T y = eta.  The dropped
// normal-matrix constant sets `constant_energy = 0` and `reason` records the
// limitation on success.
ExtractedBoundaryRows extractBoundaryRowsFromInformation(
    const Eigen::MatrixXd& lambda, const Eigen::VectorXd& eta);

}  // namespace uwb_imu_pl
