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

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <type_traits>
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

void hashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= bytes[i];
    *hash *= 1099511628211ULL;
  }
}

template <typename T>
void hashValue(std::uint64_t* hash, const T& value) {
  hashBytes(hash, &value, sizeof(value));
}

std::uint64_t carrierRankProofIdentity(
    const HistoryCarrierRankCertificate& proof) {
  std::uint64_t identity = 1469598103934665603ULL;
  hashValue(&identity, proof.storage_rows_before_compression);
  hashValue(&identity, proof.raw_residual_dof);
  hashValue(&identity, proof.effective_rank);
  hashValue(&identity, proof.rank_tolerance);
  hashValue(&identity, proof.factor_scale);
  hashValue(&identity, proof.roundoff_error_bound);
  hashValue(&identity, proof.rank_threshold);
  hashValue(&identity, proof.discarded_frobenius_bound);
  for (Eigen::Index i = 0; i < proof.singular_values.size(); ++i)
    hashValue(&identity, proof.singular_values(i));
  return identity;
}

bool compressResidualCarrier(HistoryFaultSummary* summary,
                             double rank_tolerance,
                             HistoryCarrierRankCertificate* certificate) {
  if (!summary || summary->F_b.rows() != summary->d_perp.size() ||
      summary->F_b.cols() != summary->n_fault ||
      !(rank_tolerance > 0.0) || !std::isfinite(rank_tolerance)) {
    return false;
  }
  const Eigen::Index raw_rows = summary->d_perp.size();
  Eigen::MatrixXd carrier(raw_rows, summary->n_fault + 1);
  if (summary->n_fault > 0) carrier.leftCols(summary->n_fault) = summary->F_b;
  carrier.col(summary->n_fault) = summary->d_perp;

  HistoryCarrierRankCertificate proof;
  proof.storage_rows_before_compression = static_cast<int>(raw_rows);
  proof.raw_residual_dof = static_cast<int>(raw_rows);
  proof.rank_tolerance = rank_tolerance;
  if (carrier.size() == 0) {
    proof.effective_rank = 0;
    proof.valid = true;
    proof.proof_identity = carrierRankProofIdentity(proof);
    summary->F_b.resize(0, summary->n_fault);
    summary->d_perp.resize(0);
    if (certificate) *certificate = std::move(proof);
    return true;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      carrier, Eigen::ComputeThinU | Eigen::ComputeThinV);
  proof.singular_values = svd.singularValues();
  if (!proof.singular_values.allFinite()) return false;
  proof.factor_scale = proof.singular_values.size() == 0
                           ? 0.0
                           : proof.singular_values(0);
  const double dimension = static_cast<double>(
      std::max<Eigen::Index>(carrier.rows(), carrier.cols()));
  const double eps = std::numeric_limits<double>::epsilon();
  const double gamma = dimension * eps < 1.0
                           ? (dimension * eps) / (1.0 - dimension * eps)
                           : std::numeric_limits<double>::infinity();
  proof.roundoff_error_bound = gamma * carrier.norm();
  proof.rank_threshold =
      rank_tolerance * proof.factor_scale + proof.roundoff_error_bound;
  if (!std::isfinite(proof.rank_threshold)) return false;
  for (Eigen::Index i = 0; i < proof.singular_values.size(); ++i) {
    if (proof.singular_values(i) > proof.rank_threshold)
      ++proof.effective_rank;
  }
  double discarded_squared = 0.0;
  for (Eigen::Index i = proof.effective_rank;
       i < proof.singular_values.size(); ++i) {
    discarded_squared += proof.singular_values(i) * proof.singular_values(i);
  }
  proof.discarded_frobenius_bound = std::sqrt(discarded_squared);

  // C = U S V^T and left multiplication by U^T preserves the residual norm.
  // The retained compact carrier S_r V_r^T therefore represents every
  // certified nonzero direction without depending on arbitrary raw row
  // coordinates.
  Eigen::MatrixXd compact = Eigen::MatrixXd::Zero(
      proof.effective_rank, carrier.cols());
  if (proof.effective_rank > 0) {
    compact = proof.singular_values.head(proof.effective_rank).asDiagonal() *
              svd.matrixV().leftCols(proof.effective_rank).transpose();
  }
  summary->F_b = compact.leftCols(summary->n_fault);
  summary->d_perp = compact.col(summary->n_fault);

  proof.proof_identity = carrierRankProofIdentity(proof);
  proof.valid = true;
  if (certificate) *certificate = std::move(proof);
  return true;
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

std::mutex& treeMutationMutex() {
  static std::mutex mutex;
  return mutex;
}

std::set<const IncrementalHistoryRootCache*>& armedTreeMutations() {
  static std::set<const IncrementalHistoryRootCache*> instances;
  return instances;
}

bool consumeTreeMutation(const IncrementalHistoryRootCache* cache) {
  std::lock_guard<std::mutex> lock(treeMutationMutex());
  return armedTreeMutations().erase(cache) != 0;
}

}  // namespace

static HistoryFaultSummary buildHistoryFaultSummaryImpl(
    const HistoryFaultSummaryInput& input,
    const HistoryFaultSummaryOptions& options,
    Eigen::MatrixXd* transformed_root,
    HistoryCarrierRankCertificate* carrier_rank);

struct HierarchicalBlockRoot {
  std::vector<std::string> state_uids;
  std::vector<std::string> fault_uids;
  Eigen::MatrixXd matrix;
  std::uint64_t raw_rows = 0;
};

struct HistoryTreeNode {
  std::uint64_t group = 0;
  std::uint64_t priority = 0;
  std::vector<std::string> row_uids;
  HierarchicalBlockRoot leaf;
  HierarchicalBlockRoot aggregate;
  std::shared_ptr<HistoryTreeNode> left;
  std::shared_ptr<HistoryTreeNode> right;
};

struct IncrementalHistoryRootCache::Entry {
  HistoryRootCacheRequest request;
  HistoryFaultSummary summary;
  HistoryCarrierRankCertificate carrier_rank;
  std::shared_ptr<HistoryTreeNode> tree;
};

namespace {

bool sameOwner(const std::shared_ptr<const void>& left,
               const std::shared_ptr<const void>& right) {
  return left && right && !left.owner_before(right) &&
         !right.owner_before(left);
}

template <class DerivedA, class DerivedB>
bool exactEigenEqual(const Eigen::MatrixBase<DerivedA>& left,
                     const Eigen::MatrixBase<DerivedB>& right) {
  if (left.rows() != right.rows() || left.cols() != right.cols()) return false;
  if (left.size() == 0) return true;
  using Scalar = typename DerivedA::Scalar;
  static_assert(std::is_same<Scalar, typename DerivedB::Scalar>::value,
                "cache equality requires identical scalar types");
  for (Eigen::Index row = 0; row < left.rows(); ++row) {
    for (Eigen::Index column = 0; column < left.cols(); ++column) {
      const Scalar a = left(row, column);
      const Scalar b = right(row, column);
      if (std::memcmp(&a, &b, sizeof(Scalar)) != 0) return false;
    }
  }
  return true;
}

bool sameInput(const HistoryFaultSummaryInput& left,
               const HistoryFaultSummaryInput& right) {
  return exactEigenEqual(left.h_old_state, right.h_old_state) &&
         exactEigenEqual(left.h_boundary, right.h_boundary) &&
         exactEigenEqual(left.fault_map, right.fault_map) &&
         exactEigenEqual(left.rhs, right.rhs);
}

std::map<std::uint64_t, std::vector<int>> rowsByProvenance(
    const HistoryRootCacheRequest& request) {
  std::map<std::uint64_t, std::vector<int>> result;
  for (std::size_t row = 0; row < request.row_provenance.size(); ++row) {
    result[request.row_provenance[row]].push_back(static_cast<int>(row));
  }
  return result;
}

bool sameGroupRows(const HistoryRootCacheRequest& left,
                   const std::vector<int>& left_rows,
                   const HistoryRootCacheRequest& right,
                   const std::vector<int>& right_rows) {
  if (left_rows.size() != right_rows.size()) return false;
  std::map<std::string, int> left_state;
  std::map<std::string, int> right_state;
  std::map<std::string, int> left_fault;
  std::map<std::string, int> right_fault;
  for (std::size_t i = 0; i < left.state_column_uids.size(); ++i)
    left_state.emplace(left.state_column_uids[i], static_cast<int>(i));
  for (std::size_t i = 0; i < right.state_column_uids.size(); ++i)
    right_state.emplace(right.state_column_uids[i], static_cast<int>(i));
  for (std::size_t i = 0; i < left.fault_column_uids.size(); ++i)
    left_fault.emplace(left.fault_column_uids[i], static_cast<int>(i));
  for (std::size_t i = 0; i < right.fault_column_uids.size(); ++i)
    right_fault.emplace(right.fault_column_uids[i], static_cast<int>(i));
  std::set<std::string> state_uids;
  std::set<std::string> fault_uids;
  for (const auto& item : left_state) state_uids.insert(item.first);
  for (const auto& item : right_state) state_uids.insert(item.first);
  for (const auto& item : left_fault) fault_uids.insert(item.first);
  for (const auto& item : right_fault) fault_uids.insert(item.first);
  auto state_value = [](const HistoryRootCacheRequest& request, int row,
                        int column) {
    return column < request.input.oldStateColumns()
               ? request.input.h_old_state(row, column)
               : request.input.h_boundary(
                     row, column - request.input.oldStateColumns());
  };
  for (std::size_t index = 0; index < left_rows.size(); ++index) {
    const int a = left_rows[index];
    const int b = right_rows[index];
    if (left.row_uids[static_cast<std::size_t>(a)] !=
            right.row_uids[static_cast<std::size_t>(b)] ||
        !exactEigenEqual(left.input.rhs.segment(a, 1),
                         right.input.rhs.segment(b, 1))) {
      return false;
    }
    for (const auto& uid : state_uids) {
      const auto lc = left_state.find(uid);
      const auto rc = right_state.find(uid);
      const double lv = lc == left_state.end()
                            ? 0.0
                            : state_value(left, a, lc->second);
      const double rv = rc == right_state.end()
                            ? 0.0
                            : state_value(right, b, rc->second);
      if (std::memcmp(&lv, &rv, sizeof(double)) != 0) return false;
    }
    for (const auto& uid : fault_uids) {
      const auto lc = left_fault.find(uid);
      const auto rc = right_fault.find(uid);
      const double lv = lc == left_fault.end()
                            ? 0.0
                            : left.input.fault_map(a, lc->second);
      const double rv = rc == right_fault.end()
                            ? 0.0
                            : right.input.fault_map(b, rc->second);
      if (std::memcmp(&lv, &rv, sizeof(double)) != 0) return false;
    }
  }
  return true;
}

std::size_t inputBytes(const HistoryFaultSummaryInput& input) {
  return static_cast<std::size_t>(input.h_old_state.size() +
                                  input.h_boundary.size() +
                                  input.fault_map.size() + input.rhs.size()) *
         sizeof(double);
}

std::uint64_t treePriority(std::uint64_t value) {
  // Fixed splitmix64 permutation: topology depends only on the immutable
  // provenance id, never insertion order or an address.
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

std::vector<std::string> orderedUnion(
    const std::vector<std::string>& left,
    const std::vector<std::string>& right,
    const std::vector<std::string>& requested_order) {
  std::set<std::string> present(left.begin(), left.end());
  present.insert(right.begin(), right.end());
  std::vector<std::string> result;
  result.reserve(present.size());
  for (const auto& uid : requested_order) {
    const auto found = present.find(uid);
    if (found != present.end()) {
      result.push_back(uid);
      present.erase(found);
    }
  }
  // A missing semantic id is an invalid request, but retaining it here makes
  // the representation lossless until the caller's census check rejects it.
  result.insert(result.end(), present.begin(), present.end());
  return result;
}

Eigen::MatrixXd alignBlockRoot(const HierarchicalBlockRoot& root,
                               const std::vector<std::string>& state_uids,
                               const std::vector<std::string>& fault_uids) {
  Eigen::MatrixXd out = Eigen::MatrixXd::Zero(
      root.matrix.rows(),
      static_cast<Eigen::Index>(state_uids.size() + fault_uids.size() + 1));
  std::map<std::string, int> state_index;
  std::map<std::string, int> fault_index;
  for (std::size_t i = 0; i < state_uids.size(); ++i)
    state_index.emplace(state_uids[i], static_cast<int>(i));
  for (std::size_t i = 0; i < fault_uids.size(); ++i)
    fault_index.emplace(fault_uids[i], static_cast<int>(i));
  for (std::size_t i = 0; i < root.state_uids.size(); ++i) {
    const auto found = state_index.find(root.state_uids[i]);
    if (found != state_index.end())
      out.col(found->second) = root.matrix.col(static_cast<int>(i));
  }
  const int root_fault_begin = static_cast<int>(root.state_uids.size());
  const int output_fault_begin = static_cast<int>(state_uids.size());
  for (std::size_t i = 0; i < root.fault_uids.size(); ++i) {
    const auto found = fault_index.find(root.fault_uids[i]);
    if (found != fault_index.end())
      out.col(output_fault_begin + found->second) =
          root.matrix.col(root_fault_begin + static_cast<int>(i));
  }
  out.col(out.cols() - 1) = root.matrix.col(root.matrix.cols() - 1);
  return out;
}

HierarchicalBlockRoot mergeBlockRoots(
    const HierarchicalBlockRoot& left, const HierarchicalBlockRoot& right,
    const std::vector<std::string>& requested_states,
    const std::vector<std::string>& requested_faults) {
  if (left.matrix.rows() == 0) return right;
  if (right.matrix.rows() == 0) return left;
  HierarchicalBlockRoot out;
  out.state_uids =
      orderedUnion(left.state_uids, right.state_uids, requested_states);
  out.fault_uids =
      orderedUnion(left.fault_uids, right.fault_uids, requested_faults);
  const Eigen::MatrixXd a =
      alignBlockRoot(left, out.state_uids, out.fault_uids);
  const Eigen::MatrixXd b =
      alignBlockRoot(right, out.state_uids, out.fault_uids);
  Eigen::MatrixXd work(a.rows() + b.rows(), a.cols());
  work.topRows(a.rows()) = a;
  work.bottomRows(b.rows()) = b;
  const int state_columns = static_cast<int>(out.state_uids.size());
  const auto state_pivots =
      householderReduceBlock(work, 0, 0, state_columns);

  // Once a subtree has full state-column rank, its residual state block is
  // zero.  Orthogonally triangularize [fault | rhs] there and retain only its
  // nonzero-sized root.  The omitted rows are represented by raw_rows and are
  // restored as exact zero rows at the public summary boundary, preserving
  // dof while avoiding normal equations.
  const int state_rank = rankFromPivots(state_pivots, 1e-14);
  const int state_rows = std::min<int>(work.rows(), state_columns);
  if (state_columns <= work.rows() && state_rank == state_columns) {
    const int residual_rows = static_cast<int>(work.rows()) - state_columns;
    const int protected_columns = static_cast<int>(out.fault_uids.size()) + 1;
    if (residual_rows > 0) {
      householderReduceBlock(work, state_columns, state_columns,
                             protected_columns);
      const int retained_residual =
          std::min(residual_rows, protected_columns);
      work.conservativeResize(state_columns + retained_residual,
                              Eigen::NoChange);
    } else {
      work.conservativeResize(state_rows, Eigen::NoChange);
    }
  }
  out.matrix = std::move(work);
  out.raw_rows = left.raw_rows + right.raw_rows;
  return out;
}

HierarchicalBlockRoot groupLeaf(const HistoryRootCacheRequest& request,
                                const std::vector<int>& rows) {
  HierarchicalBlockRoot out;
  const int n_o = request.input.oldStateColumns();
  const int n_b = request.input.boundaryColumns();
  for (std::size_t column = 0; column < request.state_column_uids.size();
       ++column) {
    bool used = false;
    for (const int row : rows) {
      const double value = column < static_cast<std::size_t>(n_o)
          ? request.input.h_old_state(row, static_cast<int>(column))
          : request.input.h_boundary(
                row, static_cast<int>(column) - n_o);
      if (value != 0.0) { used = true; break; }
    }
    if (used) out.state_uids.push_back(request.state_column_uids[column]);
  }
  for (std::size_t column = 0; column < request.fault_column_uids.size();
       ++column) {
    bool used = false;
    for (const int row : rows) {
      if (request.input.fault_map(row, static_cast<int>(column)) != 0.0) {
        used = true;
        break;
      }
    }
    if (used) out.fault_uids.push_back(request.fault_column_uids[column]);
  }
  out.matrix = Eigen::MatrixXd::Zero(
      static_cast<int>(rows.size()),
      static_cast<int>(out.state_uids.size() + out.fault_uids.size() + 1));
  std::map<std::string, int> request_state;
  std::map<std::string, int> request_fault;
  for (std::size_t i = 0; i < request.state_column_uids.size(); ++i)
    request_state.emplace(request.state_column_uids[i], static_cast<int>(i));
  for (std::size_t i = 0; i < request.fault_column_uids.size(); ++i)
    request_fault.emplace(request.fault_column_uids[i], static_cast<int>(i));
  for (std::size_t local_row = 0; local_row < rows.size(); ++local_row) {
    const int row = rows[local_row];
    for (std::size_t column = 0; column < out.state_uids.size(); ++column) {
      const int source = request_state.at(out.state_uids[column]);
      out.matrix(static_cast<int>(local_row), static_cast<int>(column)) =
          source < n_o ? request.input.h_old_state(row, source)
                       : request.input.h_boundary(row, source - n_o);
    }
    for (std::size_t column = 0; column < out.fault_uids.size(); ++column) {
      const int source = request_fault.at(out.fault_uids[column]);
      out.matrix(static_cast<int>(local_row),
                 static_cast<int>(out.state_uids.size() + column)) =
          request.input.fault_map(row, source);
    }
    out.matrix(static_cast<int>(local_row), out.matrix.cols() - 1) =
        request.input.rhs(row);
  }
  out.raw_rows = rows.size();
  return out;
}

bool sameLeaf(const HistoryTreeNode& node,
              const std::vector<std::string>& row_uids,
              const HierarchicalBlockRoot& leaf) {
  return node.row_uids == row_uids &&
         node.leaf.state_uids == leaf.state_uids &&
         node.leaf.fault_uids == leaf.fault_uids &&
         exactEigenEqual(node.leaf.matrix, leaf.matrix);
}

void refreshTreeNode(HistoryTreeNode* node,
                     const std::vector<std::string>& requested_states,
                     const std::vector<std::string>& requested_faults,
                     std::uint64_t* recomputed) {
  HierarchicalBlockRoot root = node->leaf;
  if (node->left)
    root = mergeBlockRoots(node->left->aggregate, root, requested_states,
                           requested_faults);
  if (node->right)
    root = mergeBlockRoots(root, node->right->aggregate, requested_states,
                           requested_faults);
  node->aggregate = std::move(root);
  if (recomputed) ++*recomputed;
}

std::shared_ptr<HistoryTreeNode> rotateTreeRight(
    std::shared_ptr<HistoryTreeNode> node,
    const std::vector<std::string>& states,
    const std::vector<std::string>& faults, std::uint64_t* recomputed) {
  auto child = node->left;
  node->left = child->right;
  child->right = node;
  refreshTreeNode(node.get(), states, faults, recomputed);
  refreshTreeNode(child.get(), states, faults, recomputed);
  return child;
}

std::shared_ptr<HistoryTreeNode> rotateTreeLeft(
    std::shared_ptr<HistoryTreeNode> node,
    const std::vector<std::string>& states,
    const std::vector<std::string>& faults, std::uint64_t* recomputed) {
  auto child = node->right;
  node->right = child->left;
  child->left = node;
  refreshTreeNode(node.get(), states, faults, recomputed);
  refreshTreeNode(child.get(), states, faults, recomputed);
  return child;
}

std::shared_ptr<HistoryTreeNode> upsertTree(
    std::shared_ptr<HistoryTreeNode> node, std::uint64_t group,
    std::vector<std::string> row_uids, HierarchicalBlockRoot leaf,
    const std::vector<std::string>& states,
    const std::vector<std::string>& faults, std::uint64_t* recomputed) {
  if (!node) {
    node = std::make_shared<HistoryTreeNode>();
    node->group = group;
    node->priority = treePriority(group);
    node->row_uids = std::move(row_uids);
    node->leaf = std::move(leaf);
    refreshTreeNode(node.get(), states, faults, recomputed);
    return node;
  }
  if (group == node->group) {
    node->row_uids = std::move(row_uids);
    node->leaf = std::move(leaf);
  } else if (group < node->group) {
    node->left = upsertTree(node->left, group, std::move(row_uids),
                            std::move(leaf), states, faults, recomputed);
    if (node->left->priority > node->priority)
      return rotateTreeRight(node, states, faults, recomputed);
  } else {
    node->right = upsertTree(node->right, group, std::move(row_uids),
                             std::move(leaf), states, faults, recomputed);
    if (node->right->priority > node->priority)
      return rotateTreeLeft(node, states, faults, recomputed);
  }
  refreshTreeNode(node.get(), states, faults, recomputed);
  return node;
}

HistoryTreeNode* findTreeNode(const std::shared_ptr<HistoryTreeNode>& node,
                              std::uint64_t group) {
  HistoryTreeNode* current = node.get();
  while (current && current->group != group)
    current = group < current->group ? current->left.get() : current->right.get();
  return current;
}

void collectTreeGroups(const std::shared_ptr<HistoryTreeNode>& node,
                       std::vector<std::uint64_t>* groups) {
  if (!node) return;
  collectTreeGroups(node->left, groups);
  groups->push_back(node->group);
  collectTreeGroups(node->right, groups);
}

std::shared_ptr<HistoryTreeNode> eraseTree(
    std::shared_ptr<HistoryTreeNode> node, std::uint64_t group,
    const std::vector<std::string>& states,
    const std::vector<std::string>& faults, std::uint64_t* recomputed) {
  if (!node) return node;
  if (group < node->group) {
    node->left = eraseTree(node->left, group, states, faults, recomputed);
  } else if (group > node->group) {
    node->right = eraseTree(node->right, group, states, faults, recomputed);
  } else {
    if (!node->left) return node->right;
    if (!node->right) return node->left;
    if (node->left->priority > node->right->priority) {
      node = rotateTreeRight(node, states, faults, recomputed);
      node->right = eraseTree(node->right, group, states, faults, recomputed);
    } else {
      node = rotateTreeLeft(node, states, faults, recomputed);
      node->left = eraseTree(node->left, group, states, faults, recomputed);
    }
  }
  refreshTreeNode(node.get(), states, faults, recomputed);
  return node;
}

HistoryFaultSummary summaryFromTreeRoot(
    const HierarchicalBlockRoot& root,
    const HistoryRootCacheRequest& request,
    const HistoryFaultSummaryOptions& options,
    HistoryCarrierRankCertificate* carrier_rank) {
  const Eigen::MatrixXd aligned = alignBlockRoot(
      root, request.state_column_uids, request.fault_column_uids);
  HistoryFaultSummaryInput compact;
  compact.h_old_state = aligned.leftCols(request.input.oldStateColumns());
  compact.h_boundary = aligned.middleCols(request.input.oldStateColumns(),
                                           request.input.boundaryColumns());
  compact.fault_map = aligned.middleCols(
      request.input.oldStateColumns() + request.input.boundaryColumns(),
      request.input.faultColumns());
  compact.rhs = aligned.col(aligned.cols() - 1);
  CertifiedHistoryFaultSummary certified =
      buildCertifiedHistoryFaultSummary(compact, options);
  HistoryFaultSummary result = std::move(certified.summary);
  if (!result.valid) return result;
  const int m2 = static_cast<int>(root.raw_rows) - result.n_old_state;
  const int target_nu = m2 - std::min(m2, result.n_boundary);
  if (target_nu < result.nuPerp() || !certified.carrier_rank.valid) {
    invalidate(result, "hierarchical root residual rank exceeds raw dof");
    return result;
  }
  // The tree intentionally stores an orthogonal compact root.  Preserve the
  // original raw statistical row count in the proof, but never pad the
  // effective carrier with numerical-noise or exact-zero rows.
  certified.carrier_rank.raw_residual_dof = target_nu;
  certified.carrier_rank.proof_identity =
      carrierRankProofIdentity(certified.carrier_rank);
  if (carrier_rank) *carrier_rank = std::move(certified.carrier_rank);
  result.n_rows = static_cast<int>(root.raw_rows);
  return result;
}

HistoryFaultSummary invalidCacheRequest(const HistoryRootCacheRequest& request,
                                        const std::string& reason) {
  HistoryFaultSummary result;
  result.n_rows = request.input.rows();
  result.n_old_state = request.input.oldStateColumns();
  result.n_boundary = request.input.boundaryColumns();
  result.n_fault = request.input.faultColumns();
  result.invalid_reason = "incremental_history_root_invalid: " + reason;
  return result;
}

HistoryFaultSummary summaryFromTransformed(const Eigen::MatrixXd& work,
                                           int n_o, int n_b, int q,
                                           double rank_tolerance) {
  HistoryFaultSummary out;
  out.n_rows = static_cast<int>(work.rows());
  out.n_old_state = n_o;
  out.n_boundary = n_b;
  out.n_fault = q;
  std::vector<double> old_pivots;
  for (int i = 0; i < n_o; ++i) old_pivots.push_back(std::abs(work(i, i)));
  out.rank_h_old_state = rankFromPivots(old_pivots, rank_tolerance);
  if (out.rank_h_old_state != n_o) {
    invalidate(out, "h_o_rank_deficient: incremental update lost old-state rank");
    return out;
  }
  double maximum = 0.0;
  double minimum = std::numeric_limits<double>::infinity();
  for (double pivot : old_pivots) {
    maximum = std::max(maximum, pivot);
    minimum = std::min(minimum, pivot);
  }
  out.old_state_pivot_ratio = n_o == 0 ? 1.0 : minimum / maximum;
  std::vector<double> boundary_pivots;
  for (int i = 0; i < n_b; ++i) {
    boundary_pivots.push_back(std::abs(work(n_o + i, n_o + i)));
  }
  out.rank_boundary = rankFromPivots(boundary_pivots, rank_tolerance);
  out.R_b = work.block(n_o, n_o, n_b, n_b);
  out.T_b = work.block(n_o, n_o + n_b, n_b, q);
  out.d_b = work.block(n_o, n_o + n_b + q, n_b, 1);
  const int nu = static_cast<int>(work.rows()) - n_o - n_b;
  out.F_b = work.block(n_o + n_b, n_o + n_b, nu, q);
  out.d_perp = work.block(n_o + n_b, n_o + n_b + q, nu, 1);
  out.valid = work.allFinite();
  if (!out.valid) invalidate(out, "non_finite_input: incremental root is non-finite");
  HistoryCarrierRankCertificate ignored;
  if (out.valid && !compressResidualCarrier(&out, rank_tolerance, &ignored))
    invalidate(out, "carrier_rank_invalid: SVD compression failed");
  return out;
}

void appendRowsToTransformed(const HistoryFaultSummaryInput& input,
                             int first_new_row,
                             Eigen::MatrixXd* transformed) {
  const int n_o = input.oldStateColumns();
  const int n_b = input.boundaryColumns();
  const int q = input.faultColumns();
  const int columns = n_o + n_b + q + 1;
  for (int source_row = first_new_row; source_row < input.rows(); ++source_row) {
    Eigen::RowVectorXd row(columns);
    if (n_o > 0) row.head(n_o) = input.h_old_state.row(source_row);
    if (n_b > 0) row.segment(n_o, n_b) = input.h_boundary.row(source_row);
    if (q > 0) row.segment(n_o + n_b, q) = input.fault_map.row(source_row);
    row(columns - 1) = input.rhs(source_row);
    for (int pivot = 0; pivot < n_o + n_b; ++pivot) {
      const double a = (*transformed)(pivot, pivot);
      const double b = row(pivot);
      if (b == 0.0) continue;
      const double magnitude = std::hypot(a, b);
      const double root = std::copysign(magnitude, a == 0.0 ? 1.0 : a);
      const double c = a / root;
      const double s = b / root;
      const Eigen::RowVectorXd old_top = transformed->row(pivot);
      transformed->row(pivot) = c * old_top + s * row;
      row = -s * old_top + c * row;
      row(pivot) = 0.0;
    }
    const Eigen::Index old_rows = transformed->rows();
    transformed->conservativeResize(old_rows + 1, Eigen::NoChange);
    transformed->row(old_rows) = row;
  }
}

bool prefixEqual(const HistoryFaultSummaryInput& old_input,
                 const HistoryFaultSummaryInput& next) {
  const int rows = old_input.rows();
  if (next.rows() <= rows ||
      old_input.oldStateColumns() != next.oldStateColumns() ||
      old_input.boundaryColumns() != next.boundaryColumns() ||
      old_input.faultColumns() > next.faultColumns()) return false;
  const int added_faults = next.faultColumns() - old_input.faultColumns();
  const Eigen::MatrixXd introduced_on_old_rows =
      next.fault_map.topRows(rows).rightCols(added_faults);
  return exactEigenEqual(old_input.h_old_state,
                         next.h_old_state.topRows(rows)) &&
         exactEigenEqual(old_input.h_boundary,
                         next.h_boundary.topRows(rows)) &&
         exactEigenEqual(old_input.fault_map,
                         next.fault_map.topRows(rows).leftCols(
                             old_input.faultColumns())) &&
         (added_faults == 0 ||
          introduced_on_old_rows.cwiseAbs().maxCoeff() == 0.0) &&
         exactEigenEqual(old_input.rhs, next.rhs.head(rows));
}

void appendFaultColumnsToTransformed(int n_o, int n_b, int old_q, int new_q,
                                     Eigen::MatrixXd* transformed) {
  if (new_q == old_q) return;
  Eigen::MatrixXd expanded = Eigen::MatrixXd::Zero(
      transformed->rows(), n_o + n_b + new_q + 1);
  expanded.leftCols(n_o + n_b + old_q) =
      transformed->leftCols(n_o + n_b + old_q);
  expanded.col(expanded.cols() - 1) = transformed->col(transformed->cols() - 1);
  *transformed = std::move(expanded);
}

}  // namespace

HistoryFaultSummary IncrementalHistoryRootCache::update(
    const HistoryRootCacheRequest& request,
    const HistoryFaultSummaryOptions& options) {
  return update(request, nullptr, options);
}

void IncrementalHistoryRootCache::corruptTreeRootForTesting() {
  std::lock_guard<std::mutex> lock(treeMutationMutex());
  armedTreeMutations().insert(this);
}

HistoryFaultSummary IncrementalHistoryRootCache::update(
    const HistoryRootCacheRequest& request,
    HistoryCarrierRankCertificate* carrier_rank,
    const HistoryFaultSummaryOptions& options) {
  if (carrier_rank) *carrier_rank = {};
  ++audit_.requests;
  if (!request.owner || request.owner_payload == nullptr) {
    invalidate("missing immutable owner/payload");
    return invalidCacheRequest(request, audit_.last_reason);
  }
  if (request.row_uids.size() !=
          static_cast<std::size_t>(request.input.rows()) ||
      request.row_provenance.size() != request.row_uids.size() ||
      request.state_column_uids.size() !=
          static_cast<std::size_t>(request.input.oldStateColumns() +
                                   request.input.boundaryColumns()) ||
      request.fault_column_uids.size() !=
          static_cast<std::size_t>(request.input.faultColumns())) {
    invalidate("row/fault UID/provenance census does not match the input");
    return invalidCacheRequest(request, audit_.last_reason);
  }
  std::set<std::string> unique_rows(request.row_uids.begin(),
                                    request.row_uids.end());
  if (unique_rows.size() != request.row_uids.size()) {
    invalidate("row UIDs are not unique");
    return invalidCacheRequest(request, audit_.last_reason);
  }

  const auto requested_groups = rowsByProvenance(request);
  bool full_tree_rebuild = !entry_;
  if (entry_) {
    const auto& cached = entry_->request;
    full_tree_rebuild =
        !sameOwner(cached.owner, request.owner) ||
        cached.owner_payload != request.owner_payload ||
        cached.ordering_version != request.ordering_version ||
        cached.whitening_version != request.whitening_version ||
        cached.recovery_version != request.recovery_version;
  }

  if (full_tree_rebuild) {
    if (entry_) ++audit_.invalidations;
    entry_ = std::make_shared<Entry>();
    ++audit_.full_rebuilds;
    ++audit_.full_tree_rebuilds;
    audit_.last_reason = "cold/order/whitening/recovery full tree rebuild";
  }

  std::uint64_t changed_groups = 0;
  std::uint64_t reused_groups = 0;
  std::uint64_t recomputed = 0;
  std::uint64_t added_groups = 0;
  std::uint64_t removed_groups = 0;
  std::uint64_t relinearized_groups = 0;
  std::set<std::uint64_t> active;
  for (const auto& item : requested_groups) {
    active.insert(item.first);
    std::vector<std::string> row_uids;
    row_uids.reserve(item.second.size());
    for (const int row : item.second)
      row_uids.push_back(request.row_uids[static_cast<std::size_t>(row)]);
    HierarchicalBlockRoot leaf = groupLeaf(request, item.second);
    HistoryTreeNode* previous = findTreeNode(entry_->tree, item.first);
    if (previous && sameLeaf(*previous, row_uids, leaf)) {
      ++reused_groups;
      continue;
    }
    if (previous) ++relinearized_groups;
    else ++added_groups;
    entry_->tree = upsertTree(entry_->tree, item.first, std::move(row_uids),
                              std::move(leaf), request.state_column_uids,
                              request.fault_column_uids, &recomputed);
    ++changed_groups;
  }
  std::vector<std::uint64_t> old_groups;
  collectTreeGroups(entry_->tree, &old_groups);
  for (const std::uint64_t group : old_groups) {
    if (active.count(group) != 0) continue;
    entry_->tree = eraseTree(entry_->tree, group, request.state_column_uids,
                             request.fault_column_uids, &recomputed);
    ++changed_groups;
    ++removed_groups;
  }
  if (!entry_->tree) {
    invalidate("hierarchical tree contains no factor groups");
    return invalidCacheRequest(request, audit_.last_reason);
  }

  if (!full_tree_rebuild && changed_groups == 0) {
    ++audit_.exact_hits;
    audit_.last_reason = "exact hierarchical root reused";
  } else if (!full_tree_rebuild) {
    ++audit_.incremental_group_updates;
    ++audit_.incremental_path_updates;
    audit_.incremental_add_paths += added_groups;
    audit_.incremental_remove_paths += removed_groups;
    audit_.incremental_relinearize_paths += relinearized_groups;
    audit_.last_reason = "changed group paths recomputed in orthogonal tree";
  }
  audit_.reused_groups += reused_groups;
  audit_.rebuilt_groups += changed_groups;
  audit_.internal_nodes_recomputed += recomputed;
  entry_->request = request;
  entry_->carrier_rank = {};
  if (consumeTreeMutation(this)) {
    // Test-only, instance-scoped mutation after the optimized tree update but
    // before summary extraction.  Immutable raw request rows are untouched.
    entry_->tree->aggregate.matrix(0, 0) =
        std::numeric_limits<double>::quiet_NaN();
  }
  entry_->summary = summaryFromTreeRoot(entry_->tree->aggregate, request,
                                        options, &entry_->carrier_rank);
  if (!entry_->summary.valid) {
    // The hierarchical representation is an optimization, not the numerical
    // contract.  A long sliding run can expose roundoff in a repeatedly
    // updated compact root even though the frozen raw rows are finite and the
    // prescribed full orthogonal rebuild is valid.  In that case use the
    // independent full-row oracle and discard the suspect tree so no later
    // request can reuse it.  An invalid oracle still fails closed below.
    ++audit_.full_oracle_checks;
    const CertifiedHistoryFaultSummary rebuilt =
        buildCertifiedHistoryFaultSummary(request.input, options);
    if (rebuilt.summary.valid && rebuilt.carrier_rank.valid) {
      HistoryFaultSummary result = rebuilt.summary;
      HistoryCarrierRankCertificate proof = rebuilt.carrier_rank;
      ++audit_.full_oracle_mismatches;
      ++audit_.full_rebuilds;
      ++audit_.invalidations;
      audit_.retained_rows = 0;
      audit_.retained_bytes = 0;
      audit_.last_reason =
          "history_root_fallback_v1:tree_invalid=1,full_oracle_valid=1,"
          "full_oracle_used=1,tree_reset=1,fallback_sequence=" +
          std::to_string(audit_.full_oracle_mismatches);
      entry_.reset();
      if (carrier_rank) *carrier_rank = std::move(proof);
      return result;
    }
    // Neither representation is usable.  Discard the suspect tree before
    // returning the invalid full-row result: a later request must rebuild
    // from its immutable raw rows and may not reuse an invalid aggregate.
    ++audit_.invalidations;
    audit_.retained_rows = 0;
    audit_.retained_bytes = 0;
    audit_.last_reason =
        "history_root_fallback_v1:tree_invalid=1,full_oracle_valid=0,"
        "full_oracle_used=0,tree_reset=1,fail_closed=1";
    entry_.reset();
    if (carrier_rank) *carrier_rank = rebuilt.carrier_rank;
    return rebuilt.summary;
  }
  if (request.verify_full_oracle) {
    ++audit_.full_oracle_checks;
    const CertifiedHistoryFaultSummary oracle_certified =
        buildCertifiedHistoryFaultSummary(request.input, options);
    const HistoryFaultSummary& oracle = oracle_certified.summary;
    auto relative_matrix = [](const Eigen::MatrixXd& a,
                              const Eigen::MatrixXd& b) {
      return (a - b).norm() / std::max({1.0, a.norm(), b.norm()});
    };
    auto relative_vector = [](const Eigen::VectorXd& a,
                              const Eigen::VectorXd& b) {
      return (a - b).norm() / std::max({1.0, a.norm(), b.norm()});
    };
    double worst = 0.0;
    bool mismatch = entry_->summary.valid != oracle.valid;
    if (entry_->summary.valid && oracle.valid) {
      worst = std::max(worst, relative_matrix(
          entry_->summary.R_b.transpose() * entry_->summary.R_b,
          oracle.R_b.transpose() * oracle.R_b));
      worst = std::max(worst, relative_matrix(
          entry_->summary.R_b.transpose() * entry_->summary.T_b,
          oracle.R_b.transpose() * oracle.T_b));
      worst = std::max(worst, relative_vector(
          entry_->summary.R_b.transpose() * entry_->summary.d_b,
          oracle.R_b.transpose() * oracle.d_b));
      worst = std::max(worst, relative_matrix(entry_->summary.omegaBoundary(),
                                               oracle.omegaBoundary()));
      worst = std::max(worst, relative_vector(entry_->summary.xiBoundary(),
                                               oracle.xiBoundary()));
      worst = std::max(
          worst, std::abs(entry_->summary.kappaBoundary() -
                          oracle.kappaBoundary()) /
                     std::max({1.0, entry_->summary.kappaBoundary(),
                               oracle.kappaBoundary()}));
      mismatch = mismatch ||
          entry_->summary.rank_h_old_state != oracle.rank_h_old_state ||
          entry_->summary.rank_boundary != oracle.rank_boundary ||
          entry_->summary.nuPerp() != oracle.nuPerp() || worst > 5e-9;
    }
    audit_.max_oracle_relative_error =
        std::max(audit_.max_oracle_relative_error, worst);
    if (mismatch) ++audit_.full_oracle_mismatches;
  }
  audit_.retained_rows = request.row_uids.size();
  audit_.retained_bytes = inputBytes(request.input) +
      static_cast<std::size_t>(entry_->tree->aggregate.matrix.size()) *
          sizeof(double);
  if (carrier_rank) *carrier_rank = entry_->carrier_rank;
  return entry_->summary;
}

void IncrementalHistoryRootCache::invalidate(const std::string& reason) {
  if (entry_) ++audit_.invalidations;
  entry_.reset();
  audit_.retained_rows = 0;
  audit_.retained_bytes = 0;
  audit_.last_reason = reason;
}

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

static HistoryFaultSummary buildHistoryFaultSummaryImpl(
    const HistoryFaultSummaryInput& input,
    const HistoryFaultSummaryOptions& options,
    Eigen::MatrixXd* transformed_root,
    HistoryCarrierRankCertificate* carrier_rank) {
  if (carrier_rank) *carrier_rank = {};
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
  const bool old_finite = allFinite(input.h_old_state);
  const bool boundary_finite = allFinite(input.h_boundary);
  const bool fault_finite = allFinite(input.fault_map);
  const bool rhs_finite = input.rhs.allFinite();
  if (!old_finite || !boundary_finite || !fault_finite || !rhs_finite) {
    invalidate(summary, std::string(kNonFinite) +
        ": H_o=" + (old_finite ? "finite" : "non_finite") +
        ",H_b=" + (boundary_finite ? "finite" : "non_finite") +
        ",A=" + (fault_finite ? "finite" : "non_finite") +
        ",z=" + (rhs_finite ? "finite" : "non_finite"));
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

  if (transformed_root) *transformed_root = W;

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

  HistoryCarrierRankCertificate local_carrier_rank;
  if (!compressResidualCarrier(&summary, options.rank_tolerance,
                               &local_carrier_rank)) {
    invalidate(summary,
               "carrier_rank_invalid: effective-rank compression failed");
    return summary;
  }
  if (carrier_rank) *carrier_rank = std::move(local_carrier_rank);

  summary.valid = true;
  return summary;
}

HistoryFaultSummary buildHistoryFaultSummary(
    const HistoryFaultSummaryInput& input,
    const HistoryFaultSummaryOptions& options) {
  return buildHistoryFaultSummaryImpl(input, options, nullptr, nullptr);
}

CertifiedHistoryFaultSummary buildCertifiedHistoryFaultSummary(
    const HistoryFaultSummaryInput& input,
    const HistoryFaultSummaryOptions& options) {
  CertifiedHistoryFaultSummary result;
  result.summary = buildHistoryFaultSummaryImpl(
      input, options, nullptr, &result.carrier_rank);
  return result;
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
