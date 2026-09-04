#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"

#include <Eigen/SVD>

#include <set>
#include <stdexcept>

namespace uwb_imu_pl {

void finalizeIntegrityWindow(LinearizedIntegrityWindow* window,
                             double rank_tolerance,
                             double max_condition_number) {
  if (!window) throw std::invalid_argument("window must not be null");
  int rows = 0;
  int columns = 0;
  std::set<std::uint64_t> group_ids;
  bool versions_match = true;
  for (const auto& block : window->blocks) {
    if (block.jacobian_whitened.rows() != block.residual_whitened.size() ||
        !block.jacobian_whitened.allFinite() ||
        !block.residual_whitened.allFinite() ||
        !std::isfinite(block.effective_weight) || block.effective_weight <= 0.0) {
      window->model_valid = false;
      window->reason = "invalid or non-finite linearized factor block";
      return;
    }
    rows += static_cast<int>(block.jacobian_whitened.rows());
    columns = std::max(columns, static_cast<int>(block.jacobian_whitened.cols()));
    versions_match = versions_match && block.version == window->version;
    group_ids.insert(block.group_id.value());
  }
  if (!versions_match) {
    window->model_valid = false;
    window->reason = "linearization version mismatch within frozen window";
    return;
  }
  window->H = Eigen::MatrixXd::Zero(rows, columns);
  window->z = Eigen::VectorXd::Zero(rows);
  int offset = 0;
  for (const auto& block : window->blocks) {
    const int n = static_cast<int>(block.jacobian_whitened.rows());
    window->H.block(offset, 0, n, block.jacobian_whitened.cols()) =
        block.jacobian_whitened;
    window->z.segment(offset, n) = block.residual_whitened;
    offset += n;
  }
  window->capabilities.no_duplicate_rows =
      group_ids.size() == window->blocks.size();
  if (rows == 0 || columns == 0) {
    window->model_valid = false;
    window->reason = "empty integrity window";
    return;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(window->H);
  const auto singular = svd.singularValues();
  const double scale = singular.size() ? singular(0) : 0.0;
  const double threshold = rank_tolerance * std::max(1.0, scale);
  window->rank = static_cast<int>((singular.array() > threshold).count());
  window->dof = rows - window->rank;
  const double smallest = window->rank > 0 ? singular(window->rank - 1) : 0.0;
  window->condition_number = smallest > 0.0 ? scale / smallest
                                             : std::numeric_limits<double>::infinity();
  window->base_information.noalias() = window->H.transpose() * window->H;
  window->base_information_rhs.noalias() = window->H.transpose() * window->z;
  window->model_valid = window->dof > 0 && window->rank == columns &&
      window->condition_number <= max_condition_number &&
      window->capabilities.no_duplicate_rows;
  if (!window->model_valid) {
    window->reason = window->rank != columns ? "window state is rank deficient" :
        (window->dof <= 0 ? "window has no residual degrees of freedom" :
         (window->condition_number > max_condition_number
              ? "window condition number exceeds gate"
              : "window contains duplicate factor groups"));
  } else {
    window->reason.clear();
  }
}

}  // namespace uwb_imu_pl
