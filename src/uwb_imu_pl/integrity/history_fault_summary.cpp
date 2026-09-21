// C1-a: fault-preserving history summary, module level (design freeze
// history-summary-design.md §1-§2).  See the header for the frozen identity,
// the sign convention and the C1-b input mapping.
//
// Algorithm (orthogonal elimination only):
//   Stage A: Householder reduction of H_o; the reflectors are applied to
//            [H_o | H_b | A | z] on the left.  Rows below rank(H_o) carry
//            no x_o support afterwards and form the residual system
//            [H_b2 | A2 | z2].
//   Stage B: Householder reduction of H_b2 over those rows.  The top n_b
//            rows of the transformed system give (R_b, T_b, d_b); the rows
//            below the boundary triangle have structurally zero x_b support
//            and give (F_b, d_perp) = (A_bottom, z_bottom).
//   No normal equations and no inverses are formed anywhere; the only
//   solves are the triangular substitutions of the sign-convention helpers.
//
// Row accounting: m2 = m - rank(H_o) residual rows; k = min(m2, n_b) rows
// feed the boundary triangle, nu_perp = m2 - k rows are detection-only rows.
// Every residual row lands in exactly one of the two blocks; no row is
// dropped.  (When k = m2 < n_b the triangle is zero-padded to n_b x n_b.)

#include "uwb_imu_pl/integrity/history_fault_summary.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace uwb_imu_pl {
namespace {

bool allFinite(const Eigen::MatrixXd& matrix) { return matrix.allFinite(); }

// In-place Householder reduction of M(row0.., col0..col0+ncols-1): after the
// call the block is upper trapezoidal over that column range.  Reflectors
// are applied to every column from col0 onward (columns before col0 are
// untouched, as their entries below the diagonal are structurally zero and
// later reflectors have no support there).  Returns the absolute pivot
// magnitudes |R_ii| (size ncols; 0 for skipped zero columns / exhausted
// rows).  Plain (unpivoted) Householder: a structural rank deficiency shows
// up as a pivot falling to (numerically) zero; pivot-based rank revealing is
// a C1-b hardening item, not part of the frozen C1-a contract.
std::vector<double> householderReduceBlock(Eigen::MatrixXd& M,
                                           Eigen::Index row0, Eigen::Index col0,
                                           Eigen::Index ncols) {
  std::vector<double> pivot_abs;
  pivot_abs.reserve(static_cast<std::size_t>(std::max<Eigen::Index>(0, ncols)));
  const Eigen::Index rows = M.rows();
  const Eigen::Index cols = M.cols();
  for (Eigen::Index j = 0; j < ncols; ++j) {
    const Eigen::Index r = row0 + j;
    if (r >= rows) {
      pivot_abs.push_back(0.0);
      continue;
    }
    Eigen::VectorXd x = M.block(r, col0 + j, rows - r, 1);
    const double norm = x.norm();
    if (norm == 0.0) {
      pivot_abs.push_back(0.0);
      continue;
    }
    Eigen::VectorXd v = x;
    const double alpha = (x(0) >= 0.0) ? -norm : norm;  // -sign(x0) * norm
    v(0) -= alpha;
    const double v_squared = v.squaredNorm();
    if (v_squared == 0.0) {
      // x is already alpha * e1 (up to sign); no reflector needed.
      pivot_abs.push_back(norm);
      continue;
    }
    const Eigen::Index block_cols = cols - (col0 + j);
    Eigen::RowVectorXd w =
        v.transpose() * M.block(r, col0 + j, rows - r, block_cols);
    M.block(r, col0 + j, rows - r, block_cols) -= (2.0 / v_squared) * v * w;
    pivot_abs.push_back(norm);
  }
  return pivot_abs;
}

int rankFromPivots(const std::vector<double>& pivot_abs, double rank_tol) {
  double scale = 0.0;
  for (const double pivot : pivot_abs) {
    scale = std::max(scale, pivot);
  }
  if (scale == 0.0) {
    return 0;
  }
  int rank = 0;
  for (const double pivot : pivot_abs) {
    if (pivot > rank_tol * scale) {
      ++rank;
    }
  }
  return rank;
}

void invalidate(HistoryFaultSummary& summary, const std::string& reason) {
  // The adjudicated ranks are diagnostics, not summary values: they are kept
  // (rank_h_old_state is set before rejection when it was computed).
  summary.valid = false;
  summary.invalid_reason = reason;
  summary.R_b = Eigen::MatrixXd();
  summary.T_b = Eigen::MatrixXd();
  summary.d_b = Eigen::VectorXd();
  summary.F_b = Eigen::MatrixXd();
  summary.d_perp = Eigen::VectorXd();
}

Eigen::VectorXd nanVector(int size) {
  Eigen::VectorXd out(size);
  out.setConstant(std::numeric_limits<double>::quiet_NaN());
  return out;
}

const char* kNonFinite = "non_finite_input";
const char* kShapeMismatch = "shape_mismatch";
const char* kEmptyInput = "empty_input";
const char* kRankDeficient = "h_o_rank_deficient";

}  // namespace

std::optional<HistoryFaultSummaryInput> splitBlockSystem(
    const Eigen::MatrixXd& block, int n_old_state, int n_boundary,
    int n_fault) {
  if (n_old_state < 0 || n_boundary < 0 || n_fault < 0) {
    return std::nullopt;
  }
  const int total = n_old_state + n_boundary + n_fault + 1;
  if (block.cols() != total) {
    return std::nullopt;
  }
  HistoryFaultSummaryInput input;
  input.h_old_state = block.leftCols(n_old_state);
  input.h_boundary = block.middleCols(n_old_state, n_boundary);
  input.fault_map = block.middleCols(n_old_state + n_boundary, n_fault);
  input.rhs = block.col(total - 1);
  return input;
}

HistoryFaultSummary buildHistoryFaultSummary(
    const HistoryFaultSummaryInput& input,
    const HistoryFaultSummaryOptions& options) {
  HistoryFaultSummary summary;
  summary.n_rows = static_cast<int>(input.rhs.size());
  summary.n_old_state = static_cast<int>(input.h_old_state.cols());
  summary.n_boundary = static_cast<int>(input.h_boundary.cols());
  summary.n_fault = static_cast<int>(input.fault_map.cols());
  const int m = summary.n_rows;
  const int n_o = summary.n_old_state;
  const int n_b = summary.n_boundary;
  const int q = summary.n_fault;

  // Degenerate handling, in order: non-finite, shapes, emptiness.
  if (!allFinite(input.h_old_state) || !allFinite(input.h_boundary) ||
      !allFinite(input.fault_map) || !input.rhs.allFinite()) {
    invalidate(summary,
               std::string(kNonFinite) + ": input contains a non-finite entry");
    return summary;
  }
  if (input.h_old_state.rows() != m || input.h_boundary.rows() != m ||
      input.fault_map.rows() != m) {
    invalidate(summary,
               std::string(kShapeMismatch) + ": H_o " +
                   std::to_string(input.h_old_state.rows()) + " rows, H_b " +
                   std::to_string(input.h_boundary.rows()) + " rows, A " +
                   std::to_string(input.fault_map.rows()) + " rows, z has " +
                   std::to_string(m) + " entries");
    return summary;
  }
  if (m == 0) {
    invalidate(summary, std::string(kEmptyInput) + ": no rows (m=0)");
    return summary;
  }

  // Working matrix [H_o | H_b | A | z]; the residual convention is
  // r = H_o x_o + H_b x_b + A f - z, so the transformed z-column feeds
  // d_b / d_perp directly (block system of the design freeze §2).
  Eigen::MatrixXd W(m, n_o + n_b + q + 1);
  if (n_o > 0) {
    W.leftCols(n_o) = input.h_old_state;
  }
  if (n_b > 0) {
    W.middleCols(n_o, n_b) = input.h_boundary;
  }
  if (q > 0) {
    W.middleCols(n_o + n_b, q) = input.fault_map;
  }
  W.col(n_o + n_b + q) = input.rhs;

  // Stage A: eliminate x_o.
  const std::vector<double> pivots_o = householderReduceBlock(W, 0, 0, n_o);
  const int rank_o = rankFromPivots(pivots_o, options.rank_tolerance);
  summary.rank_h_old_state = rank_o;
  {
    double scale = 0.0;
    double smallest = 0.0;
    bool first = true;
    for (const double pivot : pivots_o) {
      scale = std::max(scale, pivot);
      if (first || pivot < smallest) {
        smallest = pivot;
        first = false;
      }
    }
    summary.old_state_pivot_ratio =
        (n_o == 0) ? 1.0 : ((scale == 0.0) ? 0.0 : smallest / scale);
  }
  if (rank_o < n_o) {
    double scale = 0.0;
    for (const double pivot : pivots_o) {
      scale = std::max(scale, pivot);
    }
    const double smallest =
        pivots_o.empty() ? 0.0
                         : *std::min_element(pivots_o.begin(), pivots_o.end());
    invalidate(summary, std::string(kRankDeficient) + ": rank " +
                            std::to_string(rank_o) + " < " +
                            std::to_string(n_o) + " columns, pivot ratio " +
                            std::to_string(smallest) +
                            " <= " + std::to_string(options.rank_tolerance) +
                            " (scale " + std::to_string(scale) + ")");
    return summary;
  }

  // Stage B: eliminate x_b over the residual rows.
  const int m2 = m - rank_o;
  const std::vector<double> pivots_b =
      householderReduceBlock(W, rank_o, n_o, n_b);
  const int rank_b = rankFromPivots(pivots_b, options.rank_tolerance);
  summary.rank_boundary = rank_b;

  const int k = std::min(m2, n_b);  // rows that feed the boundary triangle
  const int nu = m2 - k;            // detection-only rows

  summary.R_b = Eigen::MatrixXd::Zero(n_b, n_b);
  summary.T_b = Eigen::MatrixXd::Zero(n_b, q);
  summary.d_b = Eigen::VectorXd::Zero(n_b);
  if (k > 0) {
    summary.R_b.topRows(k) = W.block(rank_o, n_o, k, n_b);
    if (q > 0) {
      summary.T_b.topRows(k) = W.block(rank_o, n_o + n_b, k, q);
    }
    summary.d_b.head(k) = W.block(rank_o, n_o + n_b + q, k, 1);
  }
  summary.F_b = W.block(rank_o + k, n_o + n_b, nu, q);
  summary.d_perp = W.block(rank_o + k, n_o + n_b + q, nu, 1);

  summary.valid = true;
  return summary;
}

Eigen::VectorXd HistoryFaultSummary::boundaryMeanShiftForFault(
    const Eigen::VectorXd& f) const {
  if (!valid || f.size() != n_fault) {
    return nanVector(n_boundary);
  }
  if (n_boundary == 0) {
    return Eigen::VectorXd(0);
  }
  if (!boundaryShiftUsable()) {
    return nanVector(n_boundary);
  }
  const Eigen::VectorXd rhs = T_b * f;
  return R_b.triangularView<Eigen::Upper>().solve(rhs);
}

Eigen::VectorXd HistoryFaultSummary::conditionalBoundaryMeanDelta(
    const Eigen::VectorXd& f) const {
  return -boundaryMeanShiftForFault(f);
}

std::uint64_t digestHistorySummaryVersion(
    const HistorySummaryVersion& version) {
  const std::uint64_t components[4] = {version.linearization, version.whitening,
                                       version.mode_set, version.capacity};
  std::uint64_t hash = 1469598103934665603ULL;
  for (const std::uint64_t component : components) {
    for (int shift = 0; shift < 64; shift += 8) {
      hash ^= (component >> shift) & 0xFFULL;
      hash *= 1099511628211ULL;
    }
  }
  return hash;
}

}  // namespace uwb_imu_pl
