#include "uwb_imu_pl/integrity/estimation_snapshot.hpp"

#include <Eigen/Cholesky>

#include <stdexcept>

namespace uwb_imu_pl {

ImmutableEstimationSnapshot::ImmutableEstimationSnapshot(
    NavigationState state, LinearizationVersion version,
    LinearizationDiagnostics diagnostics, std::vector<WhitenedRowBlock> rows,
    SnapshotCapabilities capabilities, LinearizationConsistency consistency,
    std::optional<CurrentStatePrior> prior, Eigen::MatrixXd marginal,
    Eigen::MatrixXd information)
    : state_(std::move(state)), version_(version), diagnostics_(std::move(diagnostics)),
      rows_(std::move(rows)), capabilities_(capabilities), consistency_(consistency),
      prior_(std::move(prior)), marginal_(std::move(marginal)),
      information_(std::move(information)) {}

Eigen::MatrixXd ImmutableEstimationSnapshot::solveInformation(
    const Eigen::MatrixXd& rhs) const {
  if (!capabilities_.sparse_solve && information_.size() == 0) {
    throw std::logic_error("information solve capability is unavailable");
  }
  if (information_.rows() != rhs.rows()) {
    throw std::invalid_argument("information solve RHS dimension mismatch");
  }
  Eigen::LDLT<Eigen::MatrixXd> ldlt(information_);
  if (ldlt.info() != Eigen::Success) {
    throw std::runtime_error("information factorization failed");
  }
  return ldlt.solve(rhs);
}

}  // namespace uwb_imu_pl
