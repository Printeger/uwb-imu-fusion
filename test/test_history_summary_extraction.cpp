// C1-b Part B tests: boundary representation extraction prototypes (D-3).
//
// Route (i) reads the Jacobian rows of the multifrontal reduced factor;
// route (ii) is the §11.3 Cholesky fallback.  The equivalence targets are the
// C1-a summary module outputs on the same system (direct reference) and the
// reduced information matrix on a real window.  The route decision record
// (precision / dof correctness / complexity) lives in
// doc/evidence/integrity-kernel-refactor/history-fault-parameterization.md.

#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/history_fault_summary.hpp"
#include "uwb_imu_pl/integrity/history_summary_extraction.hpp"

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>

#include <boost/shared_ptr.hpp>

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

using namespace uwb_imu_pl;

double relErr(double got, double want) {
  const double scale = std::max({1.0, std::abs(got), std::abs(want)});
  return std::abs(got - want) / scale;
}

double relErrMatrix(const Eigen::MatrixXd& got, const Eigen::MatrixXd& want) {
  if (got.size() == 0 && want.size() == 0) return 0.0;
  const double scale = std::max({1.0, got.norm(), want.norm()});
  return (got - want).norm() / scale;
}

gtsam::GaussianFactorGraph linearGraph(
    const Eigen::MatrixXd& H, const Eigen::VectorXd& z,
    const std::vector<gtsam::Key>& keys) {
  std::vector<std::pair<gtsam::Key, Eigen::MatrixXd>> terms;
  for (std::size_t i = 0; i < keys.size(); ++i) {
    terms.emplace_back(keys[i],
                       Eigen::MatrixXd(H.col(static_cast<int>(i))));
  }
  gtsam::GaussianFactorGraph graph;
  graph.emplace_shared<gtsam::JacobianFactor>(terms, Eigen::VectorXd(z));
  return graph;
}

HistoryFaultSummary summarizeRows(const Eigen::MatrixXd& rows) {
  HistoryFaultSummaryInput input;
  input.h_old_state = Eigen::MatrixXd::Zero(rows.rows(), 0);
  input.h_boundary = rows.leftCols(rows.cols() - 1);
  input.fault_map = Eigen::MatrixXd::Zero(rows.rows(), 0);
  input.rhs = rows.col(rows.cols() - 1);
  return buildHistoryFaultSummary(input);
}

IntegrityConfig researchConfig() {
  return IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/config/realtime_uwb_imu_pl_research.yaml");
}

UwbBatch batch(const IntegrityConfig& config, std::int64_t time_ns) {
  UwbBatch value;
  value.id = BatchId(static_cast<std::uint64_t>(time_ns));
  value.timestamp = TimestampNs(time_ns);
  for (std::size_t i = 0; i < config.anchors.size(); ++i) {
    UwbMeasurement measurement;
    measurement.id = MeasurementId(i + 1);
    measurement.factor_id = FactorId(i + 1);
    measurement.anchor_id = config.anchors[i].id;
    measurement.timestamp = value.timestamp;
    measurement.anchor_position_m = config.anchors[i].position_world_m;
    measurement.range_m = (measurement.anchor_position_m -
                           Eigen::Vector3d(0, 0, 1)).norm();
    measurement.sigma_m = 0.05;
    value.measurements.push_back(measurement);
  }
  return value;
}

EpochTransaction matureTransaction(IncrementalUwbImuEstimator* estimator,
                                   const IntegrityConfig& config,
                                   std::size_t count) {
  ImuMeasurement boundary;
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator->ingestImu(boundary);
  EpochTransaction last;
  for (std::size_t epoch = 1; epoch <= count; ++epoch) {
    for (int sample = 1; sample <= 10; ++sample) {
      ImuMeasurement imu;
      imu.id = MeasurementId(epoch * 1000 + sample);
      imu.timestamp = TimestampNs(static_cast<std::int64_t>(
          ((epoch - 1) * 10 + sample) * 5000000));
      imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
      estimator->ingestImu(imu);
    }
    auto input = batch(config, static_cast<std::int64_t>(epoch * 50000000));
    for (std::size_t row = 0; row < input.measurements.size(); ++row) {
      input.measurements[row].id = MeasurementId(epoch * 100 + row + 1);
      input.measurements[row].factor_id = FactorId(epoch * 100 + row + 1);
    }
    auto transaction = estimator->prepareEpoch(input);
    if (epoch == count) {
      last = transaction;
    } else {
      const auto plan = EpochCommitPlan::nominalPlan(transaction);
      estimator->commitEpoch(std::move(transaction), plan);
    }
  }
  return last;
}

}  // namespace

TEST(HistorySummaryExtraction, LinearizedRowsKeepTheConstantWhileReducedRowsDoNot) {
  const gtsam::Key k1 = gtsam::Symbol('x', 1);
  const gtsam::Key k2 = gtsam::Symbol('x', 2);
  const gtsam::Key k3 = gtsam::Symbol('x', 3);
  Eigen::MatrixXd H(5, 3);
  H << 2.0, 1.0, 0.3,
       0.4, 1.5, -0.2,
       0.1, 0.2, 1.7,
       -0.3, 0.5, 0.9,
       0.6, -0.4, 0.2;
  Eigen::VectorXd z(5);
  z << 0.7, -0.3, 1.1, 0.2, -0.6;

  const auto graph = linearGraph(H, z, {k1, k2, k3});

  // Selected route (i): rows of the *linearized* graph; the module performs
  // the orthogonal elimination itself.
  const auto full = extractBoundaryRows(graph);
  ASSERT_TRUE(full.valid) << full.reason;
  EXPECT_EQ(full.data_columns(), 3);
  ASSERT_EQ(full.keys.size(), 3u);
  EXPECT_EQ(full.keys[0], k1);
  EXPECT_EQ(full.rank, 3);

  // Direct reference: C1-a summary on the original block system.
  HistoryFaultSummaryInput direct_input;
  direct_input.h_old_state = H.leftCols(1);
  direct_input.h_boundary = H.rightCols(2);
  direct_input.fault_map = Eigen::MatrixXd::Zero(5, 0);
  direct_input.rhs = z;
  const auto direct = buildHistoryFaultSummary(direct_input);
  ASSERT_TRUE(direct.valid) << direct.invalid_reason;

  HistoryFaultSummaryInput full_input;
  full_input.h_old_state = full.rows.col(0);
  full_input.h_boundary = full.rows.middleCols(1, 2);
  full_input.fault_map = Eigen::MatrixXd::Zero(full.rows.rows(), 0);
  full_input.rhs = full.rows.col(3);
  const auto via_linearized = buildHistoryFaultSummary(full_input);
  ASSERT_TRUE(via_linearized.valid) << via_linearized.invalid_reason;
  EXPECT_LE(relErrMatrix(via_linearized.R_b.transpose() * via_linearized.R_b,
                         direct.R_b.transpose() * direct.R_b),
            1e-9);
  EXPECT_LE(relErr(via_linearized.kappaBoundary(), direct.kappaBoundary()),
            1e-9)
      << "selected route: the detector (constant) content must be preserved";
  EXPECT_LE(relErrMatrix(via_linearized.d_b, direct.d_b), 1e-9);

  // Comparison route (i-b): rows of the QR-reduced graph.  The normal
  // content matches, the constant is dropped by GTSAM elimination.
  const auto reduced =
      graph.eliminatePartialMultifrontal(gtsam::KeyVector{k1},
                                         gtsam::EliminateQR)
          .second;
  const auto extracted = extractBoundaryRows(*reduced);
  ASSERT_TRUE(extracted.valid) << extracted.reason;
  EXPECT_EQ(extracted.data_columns(), 2);
  EXPECT_EQ(extracted.keys[0], k2);
  EXPECT_EQ(extracted.keys[1], k3);
  const auto via_rows = summarizeRows(extracted.rows);
  ASSERT_TRUE(via_rows.valid) << via_rows.invalid_reason;
  EXPECT_LE(relErrMatrix(via_rows.R_b.transpose() * via_rows.R_b,
                         direct.R_b.transpose() * direct.R_b),
            1e-9);
  EXPECT_GT(direct.kappaBoundary(), 1e-6);
  EXPECT_EQ(via_rows.kappaBoundary(), 0.0)
      << "measured: partially eliminated factors lose their constant";

  // Comparison route (ii): Cholesky of the reduced normal matrix.
  gtsam::Ordering ordering;
  ordering.push_back(k2);
  ordering.push_back(k3);
  const auto hessian = reduced->hessian(ordering);
  const auto extracted_ii =
      extractBoundaryRowsFromInformation(hessian.first, hessian.second);
  ASSERT_TRUE(extracted_ii.valid) << extracted_ii.reason;
  EXPECT_EQ(extracted_ii.data_columns(), 2);
  const auto via_information = summarizeRows(extracted_ii.rows);
  ASSERT_TRUE(via_information.valid) << via_information.invalid_reason;
  EXPECT_LE(relErrMatrix(via_information.R_b.transpose() *
                             via_information.R_b,
                         direct.R_b.transpose() * direct.R_b),
            1e-9);
  EXPECT_EQ(via_information.kappaBoundary(), 0.0);

  std::printf(
      "[HSE-TABLE] case=synthetic rows=%d rank_ii=%d "
      "rel_information_selected=%.3e rel_information_reduced=%.3e "
      "kappa_direct=%.6e kappa_reduced=%.3e kappa_route_ii=%.3e\n",
      static_cast<int>(full.rows.rows()), extracted_ii.rank,
      relErrMatrix(via_linearized.R_b.transpose() * via_linearized.R_b,
                   direct.R_b.transpose() * direct.R_b),
      relErrMatrix(via_rows.R_b.transpose() * via_rows.R_b,
                   direct.R_b.transpose() * direct.R_b),
      direct.kappaBoundary(), via_rows.kappaBoundary(),
      via_information.kappaBoundary());
}

TEST(HistorySummaryExtraction, RankDeficientReducedSystemKeepsDofAccounting) {
  const gtsam::Key k1 = gtsam::Symbol('x', 1);
  const gtsam::Key k2 = gtsam::Symbol('x', 2);
  const gtsam::Key k3 = gtsam::Symbol('x', 3);
  Eigen::MatrixXd H(4, 3);
  H << 1.0, 1.0, 2.0,
       0.5, 0.5, 1.0,
       0.2, 0.2, 0.4,
       0.9, 0.9, 1.8;
  Eigen::VectorXd z(4);
  z << 1.0, -0.4, 0.6, 0.1;

  const auto graph = linearGraph(H, z, {k1, k2, k3});
  // Selected route: rows of the linearized system; the module eliminates
  // k1 itself, so the row accounting stays exact.
  const auto extracted = extractBoundaryRows(graph);
  ASSERT_TRUE(extracted.valid) << extracted.reason;
  HistoryFaultSummaryInput input;
  input.h_old_state = extracted.rows.col(0);
  input.h_boundary = extracted.rows.middleCols(1, 2);
  input.fault_map = Eigen::MatrixXd::Zero(extracted.rows.rows(), 0);
  input.rhs = extracted.rows.col(3);
  const auto summary = buildHistoryFaultSummary(input);
  ASSERT_TRUE(summary.valid) << summary.invalid_reason;
  EXPECT_EQ(summary.rank_boundary, 1);
  EXPECT_LT(summary.rank_boundary, summary.n_boundary);
  EXPECT_FALSE(summary.boundaryShiftUsable());
  // Row accounting: m2 = m - rank(H_o) = 3, k = min(m2, n_b) = 2, nu = 1.
  EXPECT_EQ(summary.nuPerp(), 1);

  // Route (ii) refuses the rank-deficient information instead of fabricating
  // a factor.
  const auto reduced =
      graph.eliminatePartialMultifrontal(gtsam::KeyVector{k1},
                                         gtsam::EliminateQR)
          .second;
  gtsam::Ordering ordering;
  ordering.push_back(k2);
  ordering.push_back(k3);
  const auto hessian = reduced->hessian(ordering);
  const auto extracted_ii =
      extractBoundaryRowsFromInformation(hessian.first, hessian.second);
  EXPECT_FALSE(extracted_ii.valid);
  EXPECT_NE(extracted_ii.reason.find("Cholesky"), std::string::npos);

  std::printf(
      "[HSE-TABLE] case=rank_deficient rows=%d rank_boundary=%d nu_perp=%d "
      "route_ii=%s\n",
      static_cast<int>(extracted.rows.rows()), summary.rank_boundary,
      summary.nuPerp(), extracted_ii.valid ? "valid" : "invalid");
}

TEST(HistorySummaryExtraction, RealWindowRowsMatchReducedInformation) {
  const auto config = researchConfig();
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  const EpochTransaction tx = matureTransaction(&estimator, config, 26);

  // Replicate the estimator's boundary reduction from the same sources
  // (`buildIntegrityWindow`, boundary segment).
  const std::size_t request_epochs = 10;
  const std::size_t first_epoch = tx.proposed_epoch - request_epochs;
  std::set<gtsam::Key> window_keys;
  for (std::size_t epoch = first_epoch; epoch <= tx.previous_epoch; ++epoch) {
    window_keys.insert(gtsam::Symbol('x', epoch));
    window_keys.insert(gtsam::Symbol('v', epoch));
    window_keys.insert(gtsam::Symbol('b', epoch));
  }
  gtsam::NonlinearFactorGraph boundary_graph;
  for (const auto& slot : tx.frozen_slots) {
    if (!slot.explicit_window_block) boundary_graph.push_back(slot.factor);
  }
  ASSERT_FALSE(boundary_graph.empty());
  std::set<gtsam::Key> graph_keys;
  for (const auto& factor : boundary_graph) {
    graph_keys.insert(factor->keys().begin(), factor->keys().end());
  }
  gtsam::KeyVector outside;
  gtsam::Ordering inside;
  for (const gtsam::Key key : graph_keys) {
    if (window_keys.count(key) == 0) outside.push_back(key);
  }
  for (std::size_t epoch = first_epoch; epoch <= tx.previous_epoch; ++epoch) {
    for (const gtsam::Key key :
         {gtsam::Symbol('x', epoch), gtsam::Symbol('v', epoch),
          gtsam::Symbol('b', epoch)}) {
      if (graph_keys.count(key)) inside.push_back(key);
    }
  }

  const auto gaussian = boundary_graph.linearize(*tx.frozen_values);

  // Selected route (i): Jacobian rows of the linearized boundary graph (no
  // GTSAM elimination); the module performs the orthogonal elimination.
  const auto extracted = extractBoundaryRows(*gaussian);
  ASSERT_TRUE(extracted.valid) << extracted.reason;
  std::map<gtsam::Key, int> column_of_key;
  for (std::size_t index = 0; index < extracted.keys.size(); ++index) {
    column_of_key.emplace(extracted.keys[index], static_cast<int>(index));
  }
  const int rows = static_cast<int>(extracted.rows.rows());
  const int columns = extracted.data_columns();
  int old_columns = 0;
  for (const gtsam::Key key : outside) {
    old_columns += extracted.key_dim[column_of_key.at(key)];
  }
  Eigen::MatrixXd h_old(rows, old_columns);
  int old_cursor = 0;
  for (const gtsam::Key key : outside) {
    const int index = column_of_key.at(key);
    h_old.middleCols(old_cursor, extracted.key_dim[index]) =
        extracted.rows.middleCols(extracted.column_begin[index],
                                  extracted.key_dim[index]);
    old_cursor += extracted.key_dim[index];
  }
  int boundary_columns = 0;
  for (const gtsam::Key key : inside) {
    boundary_columns += extracted.key_dim[column_of_key.at(key)];
  }
  Eigen::MatrixXd h_boundary(rows, boundary_columns);
  int boundary_cursor = 0;
  for (const gtsam::Key key : inside) {
    const int index = column_of_key.at(key);
    h_boundary.middleCols(boundary_cursor, extracted.key_dim[index]) =
        extracted.rows.middleCols(extracted.column_begin[index],
                                  extracted.key_dim[index]);
    boundary_cursor += extracted.key_dim[index];
  }
  HistoryFaultSummaryInput input;
  input.h_old_state = h_old;
  input.h_boundary = h_boundary;
  input.fault_map = Eigen::MatrixXd::Zero(rows, 0);
  input.rhs = extracted.rows.col(columns);
  const auto summary = buildHistoryFaultSummary(input);
  ASSERT_TRUE(summary.valid) << summary.invalid_reason;

  // Reference 1: reduced information (GTSAM elimination of the outside keys).
  gtsam::GaussianFactorGraph::shared_ptr reduced = gaussian;
  if (!outside.empty()) {
    reduced = gaussian->eliminatePartialMultifrontal(outside,
                                                     gtsam::EliminateQR)
                  .second;
  }
  const auto hessian = reduced->hessian(inside);
  const Eigen::MatrixXd summary_information =
      summary.R_b.transpose() * summary.R_b;
  std::printf(
      "[HSE-DIAG] hess_dim=%ldx%ld summary_dim=%ldx%ld hess_finite=%d "
      "summary_finite=%d hess_norm=%.6e summary_norm=%.6e "
      "hess_max=%.6e summary_max=%.6e\n",
      static_cast<long>(hessian.first.rows()),
      static_cast<long>(hessian.first.cols()),
      static_cast<long>(summary_information.rows()),
      static_cast<long>(summary_information.cols()),
      hessian.first.allFinite() ? 1 : 0,
      summary_information.allFinite() ? 1 : 0, hessian.first.norm(),
      summary_information.norm(),
      hessian.first.size() ? hessian.first.cwiseAbs().maxCoeff() : 0.0,
      summary_information.size()
          ? summary_information.cwiseAbs().maxCoeff()
          : 0.0);
  const double information_relative_error =
      relErrMatrix(summary_information, hessian.first);
  EXPECT_LE(information_relative_error, 1e-7);

  // Reference 2 (independent detector-content oracle): dense least squares on
  // the full row system, so kappa is compared against a real minimization.
  const Eigen::MatrixXd full_h = extracted.rows.leftCols(columns);
  const Eigen::VectorXd full_z = extracted.rows.col(columns);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(full_h, Eigen::ComputeThinU |
                                                    Eigen::ComputeThinV);
  const Eigen::VectorXd singular = svd.singularValues();
  const double gate = singular.size() ? 1e-12 * singular(0) : 0.0;
  Eigen::MatrixXd inverse =
      Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());
  for (int index = 0; index < singular.size(); ++index) {
    if (singular(index) > gate) inverse(index, index) = 1.0 / singular(index);
  }
  const Eigen::VectorXd least_squares =
      svd.matrixV() * inverse * svd.matrixU().transpose() * full_z;
  const double kappa_oracle = (full_z - full_h * least_squares).squaredNorm();
  const double kappa_relative_error =
      std::abs(summary.kappaBoundary() - kappa_oracle) /
      std::max({1.0, summary.kappaBoundary(), kappa_oracle});
  EXPECT_LE(kappa_relative_error, 1e-7);

  EXPECT_EQ(extracted.rank, columns)
      << "no truncation: every column survives the extracted rows";

  std::printf(
      "[HSE-TABLE] case=real_window rows=%d columns=%d rank=%d "
      "rel_to_reduced_information=%.3e kappa=%.6e kappa_oracle=%.6e "
      "kappa_rel=%.3e nu_perp=%d\n",
      rows, columns, extracted.rank, information_relative_error,
      summary.kappaBoundary(), kappa_oracle, kappa_relative_error,
      summary.nuPerp());
}
