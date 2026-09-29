#include "uwb_imu_pl/integrity/history_fault_summary.hpp"

#include <cstddef>
#include <iostream>

int main() {
  using namespace uwb_imu_pl;
  // Compiled with the golden-p1-02 header and linked to the current DSO.
  static_assert(sizeof(HistoryFaultSummary) == 176,
                "golden HistoryFaultSummary size changed");
  static_assert(offsetof(HistoryFaultSummary, d_perp) == 160,
                "golden HistoryFaultSummary tail offset changed");
  static_assert(sizeof(HistoryFaultSummaryOptions) == 8,
                "golden HistoryFaultSummaryOptions size changed");

  HistoryFaultSummaryInput input;
  input.h_old_state = Eigen::MatrixXd::Zero(4, 0);
  input.h_boundary = Eigen::MatrixXd::Zero(4, 0);
  input.fault_map = Eigen::MatrixXd::Zero(4, 2);
  input.rhs = Eigen::VectorXd::Zero(4);
  input.fault_map(0, 0) = 1.0;
  input.fault_map(1, 1) = 1e-8;
  input.rhs(2) = 1e-14;
  HistoryFaultSummaryOptions options;
  options.rank_tolerance = 1e-10;
  HistoryFaultSummary summary = buildHistoryFaultSummary(input, options);
  if (!summary.valid || summary.n_rows != 4 || summary.n_fault != 2 ||
      summary.nuPerp() != 2 || summary.F_b.rows() != 2 ||
      summary.d_perp.size() != 2 || !summary.F_b.allFinite() ||
      !summary.d_perp.allFinite()) {
    return 2;
  }
  std::cout << "HistoryFaultSummary " << sizeof(summary) << ' '
            << offsetof(HistoryFaultSummary, d_perp) << ' '
            << summary.nuPerp() << '\n';
  return 0;  // exercises old-header destruction of the current-DSO result
}
