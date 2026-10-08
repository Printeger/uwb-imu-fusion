#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>

#include <sys/resource.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <yaml-cpp/yaml.h>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>
#if defined(__SANITIZE_ADDRESS__)
#include <tbb/task_scheduler_init.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/square_root_context.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/action_hypothesis_audit.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"
#include "uwb_imu_pl/publication/final_output_packet.hpp"

namespace {

using namespace uwb_imu_pl;

#if defined(__SANITIZE_ADDRESS__)
// GTSAM can lazily create a legacy TBB scheduler whose worker storage otherwise
// survives until process teardown, after LeakSanitizer has taken its snapshot.
// Give sanitizer builds an explicit scheduler lifetime so the external TBB
// allocation is released before the leak check.  This is test-only and has no
// effect on the production binary or the ordinary regression target.
class SanitizerTbbLifetime final : public ::testing::Environment {
 public:
  void SetUp() override {
    scheduler_.reset(new tbb::task_scheduler_init(
        tbb::task_scheduler_init::automatic));
  }

  void TearDown() override {
    scheduler_->terminate();
    scheduler_.reset();
  }

 private:
  std::unique_ptr<tbb::task_scheduler_init> scheduler_;
};

const ::testing::Environment* const kSanitizerTbbLifetime =
    ::testing::AddGlobalTestEnvironment(new SanitizerTbbLifetime());
#endif

struct RawReplay {
  Eigen::MatrixXd h;
  Eigen::MatrixXd fault;
  Eigen::VectorXd z;
  Eigen::MatrixXd covariance;
  std::vector<std::uint64_t> row_ids;
  std::vector<std::uint64_t> group_ids;
};

struct DenseResult {
  Eigen::MatrixXd h;
  Eigen::MatrixXd fault;
  Eigen::VectorXd z;
  Eigen::VectorXd state;
  Eigen::MatrixXd covariance;
  Eigen::MatrixXd gram;
  Eigen::MatrixXd response;
  double statistic = std::numeric_limits<double>::infinity();
  int rank = 0;
  int dof = 0;
  std::vector<std::uint64_t> row_ids;
};

std::uint64_t hashBytes(std::uint64_t hash, const void* data,
                        std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t replayHash(const RawReplay& replay) {
  std::uint64_t hash = 1469598103934665603ULL;
  hash = hashBytes(hash, replay.row_ids.data(),
                   replay.row_ids.size() * sizeof(std::uint64_t));
  hash = hashBytes(hash, replay.group_ids.data(),
                   replay.group_ids.size() * sizeof(std::uint64_t));
  hash = hashBytes(hash, replay.h.data(),
                   static_cast<std::size_t>(replay.h.size()) * sizeof(double));
  hash = hashBytes(hash, replay.fault.data(),
                   static_cast<std::size_t>(replay.fault.size()) * sizeof(double));
  hash = hashBytes(hash, replay.z.data(),
                   static_cast<std::size_t>(replay.z.size()) * sizeof(double));
  hash = hashBytes(hash, replay.covariance.data(),
                   static_cast<std::size_t>(replay.covariance.size()) *
                       sizeof(double));
  return hash;
}

std::uint64_t monotonicNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<
      std::chrono::nanoseconds>(std::chrono::steady_clock::now()
                                   .time_since_epoch()).count());
}

// Deliberately excludes sampled timing/RSS and floating wall-dependent
// diagnostics. It binds only stable externally meaningful terminal semantics.
std::uint64_t stablePacketSemanticDigest(const FinalOutputPacket& packet) {
  const auto& out = packet.output();
  std::ostringstream text;
  text << out.attempted_timestamp.value() << '|' << out.state.timestamp.value()
       << '|' << out.selected_action_id << '|' << out.selected_action_type
       << '|' << out.fde_status << '|' << out.batch_committed << '|'
       << out.backend_updates << '|' << out.protection_level.formal_eligible
       << '|' << static_cast<int>(out.protection_level.availability) << '|'
       << out.publication.protected_output << '|'
       << static_cast<int>(packet.timing().attempt_kind) << '|'
       << static_cast<int>(packet.timing().publish_outcome);
  for (const auto& reason : out.reason_codes) text << '|' << reason;
  const std::string bytes = text.str();
  return hashBytes(1469598103934665603ULL, bytes.data(), bytes.size());
}

RawReplay makeReplay() {
  RawReplay replay;
  replay.h.resize(9, 3);
  replay.h << 1.0, 0.2, -0.1,
              0.3, 1.2, 0.4,
              -0.4, 0.1, 1.1,
              0.8, -0.5, 0.3,
              0.2, 0.7, -0.6,
              -0.3, 0.9, 0.5,
              1.1, -0.2, 0.7,
              0.5, 0.4, 1.3,
              -0.6, 1.0, 0.2;
  replay.fault.resize(9, 2);
  replay.fault << 1.0, 0.0,
                  0.8, 0.1,
                  0.5, 0.4,
                  0.2, 0.7,
                  0.0, 1.0,
                  0.1, 0.9,
                  0.6, 0.6,
                  0.7, 0.3,
                  0.4, 0.8;
  replay.z.resize(9);
  replay.z << 0.7, -0.4, 1.2, 0.3, -0.8, 0.9, 1.1, -0.2, 0.5;
  Eigen::MatrixXd lower = Eigen::MatrixXd::Identity(9, 9);
  for (int i = 1; i < 9; ++i) {
    lower(i, i - 1) = 0.16 + 0.01 * i;
  }
  lower(5, 1) = -0.11;
  lower(8, 3) = 0.09;
  replay.covariance = lower * lower.transpose();
  replay.row_ids = {1001, 1002, 2001, 2002, 3001,
                    3002, 4001, 4002, 4003};
  replay.group_ids = {10, 10, 20, 20, 30, 30, 40, 40, 40};
  return replay;
}

DenseResult denseRawOracle(const RawReplay& replay,
                           const std::set<std::uint64_t>& removed,
                           const Eigen::Matrix<double, 3, 3>& protected_map) {
  std::vector<int> keep;
  for (int row = 0; row < replay.h.rows(); ++row) {
    if (removed.count(replay.group_ids[static_cast<std::size_t>(row)]) == 0) {
      keep.push_back(row);
    }
  }
  DenseResult out;
  const int rows = static_cast<int>(keep.size());
  Eigen::MatrixXd raw_h(rows, replay.h.cols());
  Eigen::MatrixXd raw_fault(rows, replay.fault.cols());
  Eigen::VectorXd raw_z(rows);
  Eigen::MatrixXd raw_covariance(rows, rows);
  for (int i = 0; i < rows; ++i) {
    raw_h.row(i) = replay.h.row(keep[static_cast<std::size_t>(i)]);
    raw_fault.row(i) = replay.fault.row(keep[static_cast<std::size_t>(i)]);
    raw_z(i) = replay.z(keep[static_cast<std::size_t>(i)]);
    out.row_ids.push_back(replay.row_ids[static_cast<std::size_t>(keep[i])]);
    for (int j = 0; j < rows; ++j) {
      raw_covariance(i, j) = replay.covariance(keep[i], keep[j]);
    }
  }
  Eigen::LLT<Eigen::MatrixXd> whitener(raw_covariance);
  EXPECT_EQ(whitener.info(), Eigen::Success);
  out.h = whitener.matrixL().solve(raw_h);
  out.fault = whitener.matrixL().solve(raw_fault);
  out.z = whitener.matrixL().solve(raw_z);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      out.h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto singular = svd.singularValues();
  const double gate = singular(0) * 1e-12;
  Eigen::VectorXd inverse = Eigen::VectorXd::Zero(singular.size());
  Eigen::VectorXd inverse_squared = Eigen::VectorXd::Zero(singular.size());
  for (int i = 0; i < singular.size(); ++i) {
    if (singular(i) > gate) {
      inverse(i) = 1.0 / singular(i);
      inverse_squared(i) = inverse(i) * inverse(i);
      ++out.rank;
    }
  }
  const Eigen::MatrixXd pinv =
      svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose();
  out.state = pinv * out.z;
  out.covariance =
      svd.matrixV() * inverse_squared.asDiagonal() * svd.matrixV().transpose();
  const Eigen::VectorXd residual = out.z - out.h * out.state;
  out.statistic = residual.squaredNorm();
  out.dof = out.h.rows() - out.rank;
  const Eigen::MatrixXd parity =
      Eigen::MatrixXd::Identity(rows, rows) - out.h * pinv;
  out.gram = out.fault.transpose() * parity * out.fault;
  out.response = protected_map * pinv * out.fault;

  // A separately typed, extended-precision normal-equation solve is only a
  // cross-check on this deliberately well-conditioned replay.  Expected
  // values above remain the stable SVD oracle.
  using MatrixXl = Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic>;
  using VectorXl = Eigen::Matrix<long double, Eigen::Dynamic, 1>;
  const MatrixXl h_long = out.h.cast<long double>();
  const VectorXl z_long = out.z.cast<long double>();
  const VectorXl state_long =
      (h_long.transpose() * h_long).ldlt().solve(h_long.transpose() * z_long);
  EXPECT_LT((state_long.template cast<double>() - out.state).norm(), 1e-11);
  return out;
}

double relativeError(const Eigen::MatrixXd& actual,
                     const Eigen::MatrixXd& expected) {
  return (actual - expected).norm() /
      std::max({1.0, actual.norm(), expected.norm()});
}

UwbBatch makeBatch(TimestampNs timestamp) {
  UwbBatch batch;
  batch.id = BatchId(static_cast<std::uint64_t>(timestamp.value()));
  batch.timestamp = timestamp;
  batch.covariance_model_id = "p007-correlated-raw-replay";
  const std::vector<Eigen::Vector3d> anchors = {
      {-5, -5, 0}, {5, -5, 1}, {5, 5, 3}, {-5, 5, 4}, {0, -6, 2}};
  for (std::size_t i = 0; i < anchors.size(); ++i) {
    UwbMeasurement m;
    m.id = MeasurementId(i + 1);
    m.factor_id = FactorId(i + 1);
    m.anchor_id = AnchorId(i + 1);
    m.timestamp = timestamp;
    m.anchor_position_m = anchors[i];
    m.range_m = (Eigen::Vector3d(0, 0, 1) - anchors[i]).norm();
    m.sigma_m = 0.1;
    batch.measurements.push_back(m);
  }
  return batch;
}

UwbBatch makeProductionReplayBatch(const IntegrityConfig& config,
                                   std::size_t epoch,
                                   std::size_t alarm_epoch,
                                   std::int64_t epoch_period_ns) {
  UwbBatch batch;
  batch.id = BatchId(700000 + epoch);
  batch.timestamp = TimestampNs(
      static_cast<std::int64_t>(epoch) * epoch_period_ns);
  batch.covariance_model_id = "p007-production-correlated-replay";
  for (std::size_t row = 0; row < config.anchors.size(); ++row) {
    UwbMeasurement measurement;
    measurement.id = MeasurementId(epoch * 100 + row + 1);
    measurement.factor_id = FactorId(epoch * 100 + row + 1);
    measurement.anchor_id = config.anchors[row].id;
    measurement.timestamp = batch.timestamp;
    measurement.anchor_position_m = config.anchors[row].position_world_m;
    measurement.range_m =
        (measurement.anchor_position_m - Eigen::Vector3d(0, 0, 1)).norm() +
        0.012 * std::sin(0.37 * epoch + 0.19 * (row + 1));
    // The frozen replay contains one physical raw-measurement alarm.  It is
    // part of the input definition consumed by both the independent oracle
    // and the production pipeline, rather than a mutation of production H/z.
    if (epoch == alarm_epoch && row == 0) measurement.range_m += 2.0;
    measurement.sigma_m = 0.05;
    batch.measurements.push_back(measurement);
  }
  const Eigen::Index rows = static_cast<Eigen::Index>(batch.measurements.size());
  const double variance = 0.05 * 0.05;
  batch.covariance_m2 = Eigen::MatrixXd::Constant(rows, rows, 0.08 * variance);
  batch.covariance_m2.diagonal().setConstant(variance);
  return batch;
}

void appendProductionReplayImu(IncrementalUwbImuEstimator* estimator,
                               const IntegrityConfig& config,
                               std::size_t epoch,
                               std::size_t samples_per_epoch,
                               std::int64_t epoch_period_ns) {
  for (std::size_t sample = 1; sample <= samples_per_epoch; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(900000 + epoch * 10 + sample);
    imu.timestamp = TimestampNs(static_cast<std::int64_t>(epoch - 1) *
        epoch_period_ns + static_cast<std::int64_t>(sample) *
        epoch_period_ns / static_cast<std::int64_t>(samples_per_epoch));
    imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
    estimator->ingestImu(imu);
  }
}

HypothesisGeneratorConfig productionGeneratorConfig(
    const IntegrityConfig& config) {
  HypothesisGeneratorConfig out;
  out.include_uwb_faults = config.resolved_scope.requiresUwbFaults();
  out.include_imu_faults = config.resolved_scope.requiresImuFaults();
  out.single_faults_enabled = config.resolved_scope.max_fault_order >= 1;
  out.double_faults_enabled = config.resolved_scope.max_fault_order >= 2;
  out.max_model_cardinality = config.fault_models.max_cardinality;
  out.max_exclusion_cardinality = config.fde.max_exclusion_cardinality;
  out.max_candidate_count = config.fde.max_candidate_count;
  out.uwb_prior_bound = config.fault_models.uwb.prior_probability_bound;
  out.accel_prior_bound =
      config.fault_models.imu.accel_prior_probability_bound;
  out.gyro_prior_bound = config.fault_models.imu.gyro_prior_probability_bound;
  out.uwb_p_md = config.fault_models.uwb.p_md;
  out.imu_p_md = config.fault_models.imu.p_md;
  out.include_uwb_accel_combinations =
      config.fault_models.combinations.uwb_plus_accel;
  out.include_uwb_gyro_combinations =
      config.fault_models.combinations.uwb_plus_gyro;
  out.include_epoch_independent_uwb =
      config.fault_models.uwb.epoch_single_anchor_bias;
  out.include_persistent_uwb = config.fault_models.uwb.persistent_anchor_bias;
  out.include_ramp_uwb = config.fault_models.uwb.ramp_bias;
  out.total_hmi_allocation = config.risk_v2.p_hmi_total;
  out.rank_tolerance = config.integrity_window.rank_tolerance;
  return out;
}

int keyWidth(gtsam::Key key) {
  const char symbol = gtsam::Symbol(key).chr();
  return symbol == 'v' ? 3 : 6;
}

Eigen::MatrixXd orthogonalProjector(const Eigen::MatrixXd& matrix) {
  if (matrix.cols() == 0) {
    return Eigen::MatrixXd::Zero(matrix.rows(), matrix.rows());
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU);
  const auto singular = svd.singularValues();
  const double gate = singular.size() == 0 ? 0.0 : singular(0) * 1e-12;
  const int rank = static_cast<int>((singular.array() > gate).count());
  return svd.matrixU().leftCols(rank) * svd.matrixU().leftCols(rank).transpose();
}

Eigen::MatrixXd svdPseudoInverse(const Eigen::MatrixXd& matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd inverse = Eigen::VectorXd::Zero(svd.singularValues().size());
  const double gate = svd.singularValues().size() == 0
      ? 0.0 : svd.singularValues()(0) * 1e-12;
  for (Eigen::Index i = 0; i < inverse.size(); ++i) {
    if (svd.singularValues()(i) > gate) inverse(i) = 1.0 / svd.singularValues()(i);
  }
  return svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose();
}

const PendingFactorGroup* selectedGroup(const HistoricalEpochContext& record,
                                        FactorKind kind) {
  for (const auto id : record.selected_groups) {
    for (const auto& group : record.groups) {
      if (group.id == id && group.kind == kind) return &group;
    }
  }
  return nullptr;
}

const UwbMeasurement* replayMeasurement(const HistoricalEpochContext& record,
                                        MeasurementId id) {
  for (const auto& measurement : record.uwb_batch.measurements) {
    if (measurement.id == id) return &measurement;
  }
  return nullptr;
}

struct CompleteRawReference {
  Eigen::MatrixXd design;
  Eigen::VectorXd residual;
  Eigen::MatrixXd information;
  Eigen::VectorXd rhs;
  Eigen::MatrixXd covariance;
  Eigen::VectorXd state;
  double constant = 0.0;
  double objective = 0.0;
  int rank = 0;
  int dof = 0;
  std::size_t unique_raw_rows = 0;
  std::vector<std::uint64_t> explicit_group_ids;
  Eigen::MatrixXd history_response;
  Eigen::MatrixXd history_detector_response;
  Eigen::MatrixXd protected_state_map;
  std::map<std::uint64_t, Eigen::Index> group_row_offsets;
  std::map<std::uint64_t, Eigen::Index> group_row_counts;
  std::map<std::uint64_t, Eigen::MatrixXd> uwb_whiteners;
  struct Block {
    Eigen::MatrixXd raw_h;
    Eigen::VectorXd raw_z;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd whitener;
    Eigen::MatrixXd h;
    Eigen::VectorXd z;
  };
  std::map<std::uint64_t, Block> blocks;
  std::map<std::uint64_t, Eigen::MatrixXd> fault_modes;
};

struct DenseActionReference {
  std::uint64_t id = 0;
  int rank = 0;
  int dof = 0;
  double statistic = std::numeric_limits<double>::infinity();
  double threshold = std::numeric_limits<double>::infinity();
  bool post_passed = false;
  bool covers_plausible = false;
  bool model_error_validated = false;
  bool pl_valid = false;
  bool eligible = false;
  bool selected = false;
  std::size_t candidate_profile_count = 0;
  double hpl_m = std::numeric_limits<double>::infinity();
  double vpl_m = std::numeric_limits<double>::infinity();
  double risk_allocation = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d mathematical_pl_xyz = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double mathematical_hpl_m = std::numeric_limits<double>::infinity();
  double mathematical_vpl_m = std::numeric_limits<double>::infinity();
  double information_logdet = -std::numeric_limits<double>::infinity();
  int exclusion_cardinality = 0;
  bool mathematical_pl_finite = false;
  bool risk_valid = false;
  bool risk_all_terms_validated = false;
  double risk_charged_total = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::string> risk_terms;
  std::vector<Eigen::MatrixXd> candidate_grams;
  std::vector<Eigen::MatrixXd> candidate_fault_maps;
  std::vector<Eigen::MatrixXd> protected_responses;
  std::vector<Eigen::Vector3d> gram_protected_slopes;
  std::vector<int> gram_nullspace_classes;
  std::vector<Eigen::Vector3d> protected_slopes;
  std::vector<int> nullspace_classes;
  std::string selection_tuple;
  std::string terminal;
  std::string removed;
  std::string added;
};

struct OracleHistorySummary {
  Eigen::MatrixXd r;
  Eigen::MatrixXd t;
  Eigen::VectorXd d;
  Eigen::MatrixXd f;
  Eigen::VectorXd d_perp;
  int rank_old = 0;
};

std::vector<double> oracleHouseholder(Eigen::MatrixXd* matrix,
                                      Eigen::Index row0,
                                      Eigen::Index column0,
                                      Eigen::Index count) {
  std::vector<double> pivots;
  for (Eigen::Index j = 0; j < count; ++j) {
    const Eigen::Index row = row0 + j;
    if (row >= matrix->rows()) { pivots.push_back(0.0); continue; }
    Eigen::VectorXd x = matrix->block(
        row, column0 + j, matrix->rows() - row, 1);
    const double norm = x.norm();
    pivots.push_back(norm);
    if (norm == 0.0) continue;
    Eigen::VectorXd v = x;
    v(0) -= x(0) >= 0.0 ? -norm : norm;
    if (v.squaredNorm() == 0.0) continue;
    const Eigen::Index columns = matrix->cols() - (column0 + j);
    const Eigen::RowVectorXd projection = v.transpose() * matrix->block(
        row, column0 + j, matrix->rows() - row, columns);
    matrix->block(row, column0 + j, matrix->rows() - row, columns) -=
        (2.0 / v.squaredNorm()) * v * projection;
  }
  return pivots;
}

int oracleRank(const std::vector<double>& pivots) {
  const double scale = pivots.empty()
      ? 0.0 : *std::max_element(pivots.begin(), pivots.end());
  return static_cast<int>(std::count_if(
      pivots.begin(), pivots.end(),
      [&](double value) { return value > 1e-12 * scale; }));
}

OracleHistorySummary oracleHistorySummary(const Eigen::MatrixXd& old_h,
                                           const Eigen::MatrixXd& boundary_h,
                                           const Eigen::MatrixXd& fault,
                                           const Eigen::VectorXd& z,
                                           double rank_tolerance = 1e-12) {
  const Eigen::Index n_old = old_h.cols();
  const Eigen::Index n_boundary = boundary_h.cols();
  const Eigen::Index n_fault = fault.cols();
  Eigen::MatrixXd joined(z.size(), n_old + n_boundary + n_fault + 1);
  joined << old_h, boundary_h, fault, z;
  OracleHistorySummary out;
  out.rank_old = oracleRank(oracleHouseholder(&joined, 0, 0, n_old));
  oracleHouseholder(&joined, out.rank_old, n_old, n_boundary);
  const Eigen::Index remaining = joined.rows() - out.rank_old;
  const Eigen::Index supported = std::min(remaining, n_boundary);
  const Eigen::Index perpendicular = remaining - supported;
  out.r = Eigen::MatrixXd::Zero(n_boundary, n_boundary);
  out.t = Eigen::MatrixXd::Zero(n_boundary, n_fault);
  out.d = Eigen::VectorXd::Zero(n_boundary);
  if (supported > 0) {
    out.r.topRows(supported) = joined.block(
        out.rank_old, n_old, supported, n_boundary);
    out.t.topRows(supported) = joined.block(
        out.rank_old, n_old + n_boundary, supported, n_fault);
    out.d.head(supported) = joined.block(
        out.rank_old, n_old + n_boundary + n_fault, supported, 1);
  }
  out.f = joined.block(out.rank_old + supported, n_old + n_boundary,
                       perpendicular, n_fault);
  out.d_perp = joined.block(out.rank_old + supported,
                            n_old + n_boundary + n_fault, perpendicular, 1);
  // Independent corrected-contract oracle: compress [F_b|d_perp] to the
  // conditioning-aware effective numerical rank.  No element-wise nonzero
  // row count and no expected rank constant participates in this decision.
  Eigen::MatrixXd carrier(perpendicular, n_fault + 1);
  if (n_fault > 0) carrier.leftCols(n_fault) = out.f;
  carrier.col(n_fault) = out.d_perp;
  if (carrier.size() > 0) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        carrier, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd singular = svd.singularValues();
    const double scale = singular.size() == 0 ? 0.0 : singular(0);
    const double dimension = static_cast<double>(
        std::max<Eigen::Index>(carrier.rows(), carrier.cols()));
    const double eps = std::numeric_limits<double>::epsilon();
    const double gamma = (dimension * eps) / (1.0 - dimension * eps);
    const double threshold = rank_tolerance * scale + gamma * carrier.norm();
    const int rank = static_cast<int>(
        (singular.array() > threshold).count());
    const Eigen::MatrixXd compact =
        singular.head(rank).asDiagonal() *
        svd.matrixV().leftCols(rank).transpose();
    out.f = compact.leftCols(n_fault);
    out.d_perp = compact.col(n_fault);
  } else {
    out.f.resize(0, n_fault);
    out.d_perp.resize(0);
  }
  return out;
}

struct OracleOperation {
  std::uint64_t id = 0;
  int exclusion_cardinality = 0;
  std::vector<std::uint64_t> remove;
  std::vector<std::uint64_t> add;
  std::vector<std::uint64_t> covered_modes;
};

struct OracleFamily {
  int kind = 0;
  int dimension = 0;
  int terminal_onset_effective_dimension = 0;
  std::string name;
  std::string support;
  std::vector<std::string> native_columns;
};

struct OracleHypothesis {
  std::uint64_t id = 0;
  double prior_probability_bound = 0.0;
  double p_md_allocation = 0.0;
  double hmi_allocation = 0.0;
};

struct OracleManifest {
  std::string authority;
  std::size_t replay_epochs = 0;
  std::size_t imu_samples_per_epoch = 0;
  std::size_t alarm_epoch = 0;
  std::size_t physical_first_epoch = 0;
  std::size_t detector_first_epoch = 0;
  std::size_t window_first_epoch = 0;
  std::int64_t epoch_period_ns = 0;
  std::size_t history_rows = 0;
  std::size_t served_rows = 0;
  std::size_t summary_rows = 0;
  std::size_t expected_modes = 0;
  std::size_t expected_hypotheses = 0;
  std::size_t state_first_epoch = 0;
  std::size_t state_last_epoch = 0;
  std::size_t state_columns_per_epoch = 0;
  std::size_t state_columns = 0;
  std::size_t history_uwb_first = 0;
  std::size_t history_uwb_last = 0;
  std::size_t history_imu_first = 0;
  std::size_t history_imu_last = 0;
  std::size_t history_uwb_only_first = 0;
  std::size_t history_uwb_only_last = 0;
  std::size_t history_uwb_only_rows = 0;
  std::size_t history_both_first = 0;
  std::size_t history_both_last = 0;
  std::size_t history_both_uwb_rows = 0;
  std::size_t history_both_imu_rows = 0;
  std::size_t history_imu_only_first = 0;
  std::size_t history_imu_only_last = 0;
  std::size_t history_imu_only_rows = 0;
  std::size_t explicit_uwb_first = 0;
  std::size_t explicit_uwb_last = 0;
  std::size_t explicit_imu_first = 0;
  std::size_t explicit_imu_last = 0;
  std::size_t explicit_uwb_rows = 0;
  std::size_t explicit_imu_rows = 0;
  std::size_t onset_first = 0;
  std::size_t onset_last = 0;
  std::size_t uwb_rows = 0;
  std::size_t imu_rows = 0;
  std::uint64_t marginal_slot = 0;
  std::uint64_t marginal_group = 0;
  std::uint64_t summary_group = 0;
  std::size_t marginal_rows = 0;
  std::uint64_t slot_imu_multiplier = 0;
  std::uint64_t slot_imu_add = 0;
  std::uint64_t slot_uwb_multiplier = 0;
  std::uint64_t slot_uwb_add = 0;
  std::uint64_t group_epoch_multiplier = 0;
  std::uint64_t group_imu_add = 0;
  std::uint64_t group_uwb_add = 0;
  std::string prior_row_prefix;
  std::string imu_row_prefix;
  std::string uwb_row_prefix;
  std::vector<std::string> history_factor_order;
  std::vector<std::uint64_t> anchors;
  std::vector<OracleFamily> families;
  std::string window_boundary_onset_rule;
  std::string mode_order;
  std::string hypothesis_rule;
  std::vector<std::string> double_fault_families;
  std::vector<std::uint64_t> plausible_modes;
  std::vector<OracleOperation> operations;
  double total_risk = 0.0;
  double action_risk = 0.0;
  double detector_p_fa = 0.0;
  double plausible_margin = 0.0;
  std::string detector_type;
  std::string detector_distribution;
  bool detector_squared_norm = false;
  bool keep_all_first = false;
  bool one_occurrence_per_plausible = false;
  bool union_when_single_anchor = false;
  std::uint32_t max_exclusion_cardinality = 0;
  std::vector<std::string> identity_fields;
  std::string exact_duplicate_rule;
  std::array<std::string, 4> selection_order;
  std::string fde_profile;
  std::uint32_t fault_max_cardinality = 0;
  bool uwb_enabled = false;
  bool epoch_single_enabled = false;
  bool persistent_enabled = false;
  bool ramp_enabled = false;
  bool single_faults_enabled = false;
  bool double_faults_enabled = false;
  int resolved_max_fault_order = 0;
  std::string action_trigger;
  std::string action_isolation;
  std::string ambiguity_policy;
  double nominal_axis_tail = 0.0;
  double p_nm = 0.0;
  double p_bridge_escape = 0.0;
  double p_history_contamination = 0.0;
  double p_model_escape = 0.0;
  double horizontal_alert_limit_m = 0.0;
  double vertical_alert_limit_m = 0.0;
  std::string risk_allocation_policy;
  std::string risk_allocation_rule;
  std::string risk_model_calibration;
  bool risk_unknown_terms_are_zero = true;
  double uwb_prior_probability_bound = 0.0;
  double uwb_p_md = 0.0;
  bool dense_oracle_online_fallback = true;
  std::uint32_t max_candidate_count = 0;
  std::string on_no_valid_action;
  std::string on_integrity_model_invalid;
  std::string fault_manifest_path;
  std::string fault_manifest_id;
  std::uint64_t historical_replacement_multiplier = 0;
  std::uint64_t historical_epoch_multiplier = 0;
  std::uint64_t current_replacement_multiplier = 0;
  std::uint64_t anchor_replacement_base = 0;
  std::uint64_t anchor_replacement_stride = 0;
  std::size_t missing_anchor = 0;
  std::size_t missing_onset = 0;
  int missing_family = 0;
  std::string missing_recipe;
  std::string missing_recoverability_rule;
  std::vector<std::uint64_t> observed_plausible_modes;
  std::size_t observed_generated_action_count = 0;
  std::uint64_t observed_winner = 0;
  std::string observed_note;
  std::vector<std::string> tolerance_classifications;
  double epsilon = 0.0;
  double tolerance_cap = 0.0;
};

void requireExactKeys(const YAML::Node& node,
                      std::initializer_list<const char*> expected,
                      const std::string& path) {
  if (!node || !node.IsMap()) {
    throw std::runtime_error("oracle recipe node is not a map: " + path);
  }
  std::set<std::string> wanted;
  for (const char* key : expected) wanted.insert(key);
  std::set<std::string> actual;
  for (const auto& item : node) actual.insert(item.first.as<std::string>());
  if (actual != wanted) {
    std::ostringstream message;
    message << "oracle recipe unconsumed/missing key at " << path;
    throw std::runtime_error(message.str());
  }
}

std::vector<std::uint64_t> yamlU64(const YAML::Node& node) {
  std::vector<std::uint64_t> out;
  for (const auto& value : node) out.push_back(value.as<std::uint64_t>());
  return out;
}

std::vector<std::string> yamlStrings(const YAML::Node& node) {
  std::vector<std::string> out;
  for (const auto& value : node) out.push_back(value.as<std::string>());
  return out;
}

OracleManifest parseOracleManifest(const YAML::Node& root) {
  requireExactKeys(root, {"schema", "authority", "replay",
                          "factor_construction", "fault_contract",
                          "detector_and_action_recipe",
                          "missing_provenance_fixture",
                          "observed_non_authoritative", "tolerance_policy"},
                   "root");
  if (root["schema"].as<std::string>() !=
      "p0-07-independent-oracle-recipe-v2") {
    throw std::runtime_error("unsupported P0-07 oracle manifest");
  }
  OracleManifest out;
  out.authority = root["authority"].as<std::string>();
  if (out.authority !=
      "declarative construction recipe; observed outcomes are non-authoritative") {
    throw std::runtime_error("unsupported P0-07 oracle authority");
  }
  const auto replay = root["replay"];
  requireExactKeys(replay, {"epochs", "alarm_epoch", "physical_first_epoch",
                            "detector_first_epoch", "window_first_epoch",
                            "epoch_period_ns", "anchors",
                            "imu_samples_per_epoch", "uwb_rows_per_epoch"},
                   "replay");
  out.replay_epochs = replay["epochs"].as<std::size_t>();
  out.imu_samples_per_epoch =
      replay["imu_samples_per_epoch"].as<std::size_t>();
  out.alarm_epoch = replay["alarm_epoch"].as<std::size_t>();
  out.physical_first_epoch = replay["physical_first_epoch"].as<std::size_t>();
  out.detector_first_epoch = replay["detector_first_epoch"].as<std::size_t>();
  out.window_first_epoch = replay["window_first_epoch"].as<std::size_t>();
  out.epoch_period_ns = replay["epoch_period_ns"].as<std::int64_t>();
  out.anchors = yamlU64(replay["anchors"]);
  if (replay["uwb_rows_per_epoch"].as<std::size_t>() != out.anchors.size()) {
    throw std::runtime_error("replay UWB row count does not match anchors");
  }
  if (std::set<std::uint64_t>(out.anchors.begin(), out.anchors.end()).size() !=
      out.anchors.size()) {
    throw std::runtime_error("oracle recipe anchor identities are not unique");
  }
  const auto factors = root["factor_construction"];
  requireExactKeys(factors, {"state_columns", "history_raw", "served_window",
                             "whitening"}, "factor_construction");
  const auto state = factors["state_columns"];
  requireExactKeys(state, {"first_epoch", "last_epoch", "per_epoch",
                           "components"}, "factor_construction.state_columns");
  out.state_first_epoch = state["first_epoch"].as<std::size_t>();
  out.state_last_epoch = state["last_epoch"].as<std::size_t>();
  out.state_columns_per_epoch = state["per_epoch"].as<std::size_t>();
  std::size_t component_width = 0;
  std::set<std::string> component_symbols;
  const std::array<std::string, 3> expected_component_symbols{{"x", "v", "b"}};
  const std::array<std::size_t, 3> expected_component_offsets{{0, 6, 9}};
  const std::array<std::size_t, 3> expected_component_widths{{6, 3, 6}};
  std::size_t component_index = 0;
  for (const auto& component : state["components"]) {
    requireExactKeys(component, {"symbol", "offset", "width"},
                     "factor_construction.state_columns.components[]");
    const std::string symbol = component["symbol"].as<std::string>();
    const std::size_t offset = component["offset"].as<std::size_t>();
    const std::size_t width = component["width"].as<std::size_t>();
    if (component_index >= expected_component_symbols.size() ||
        symbol != expected_component_symbols[component_index] ||
        offset != expected_component_offsets[component_index] ||
        width != expected_component_widths[component_index]) {
      throw std::runtime_error("unsupported oracle state component contract");
    }
    ++component_index;
    if (!component_symbols.insert(symbol).second) {
      throw std::runtime_error("duplicate oracle state component");
    }
    if (offset != component_width) {
      throw std::runtime_error("oracle state component ordering has a gap");
    }
    component_width += width;
  }
  if (component_index != expected_component_symbols.size() ||
      component_width != out.state_columns_per_epoch ||
      out.state_first_epoch > out.state_last_epoch) {
    throw std::runtime_error("oracle state component widths do not close");
  }
  out.state_columns = (out.state_last_epoch - out.state_first_epoch + 1) *
      out.state_columns_per_epoch;
  out.history_rows = factors["history_raw"]["expected_rows"].as<std::size_t>();
  out.marginal_slot = factors["history_raw"]["marginal_prior"]["slot"].as<std::uint64_t>();
  out.marginal_group = factors["history_raw"]["marginal_prior"]["group_id"].as<std::uint64_t>();
  out.marginal_rows = factors["history_raw"]["marginal_prior"]["rows"].as<std::size_t>();
  if (factors["history_raw"]["marginal_prior"]["owner"].as<std::string>() !=
      out.prior_row_prefix && out.prior_row_prefix.size() != 0) {
    throw std::runtime_error("marginal prior owner and row prefix differ");
  }
  out.served_rows = factors["served_window"]["expected_rows"].as<std::size_t>();
  out.summary_rows = factors["served_window"]["history_summary_rows"].as<std::size_t>();
  out.summary_group = factors["served_window"]["history_summary_group_id"].as<std::uint64_t>();
  const auto history = factors["history_raw"];
  requireExactKeys(history, {"expected_rows", "marginal_prior",
                             "uwb_only_epochs", "imu_and_uwb_epochs",
                             "imu_only_epochs", "slot_rules", "group_rules",
                             "row_identity", "factor_order"},
                   "factor_construction.history_raw");
  requireExactKeys(history["marginal_prior"],
                   {"slot", "group_id", "owner", "rows"},
                   "factor_construction.history_raw.marginal_prior");
  requireExactKeys(history["uwb_only_epochs"], {"first", "last", "rows_each"},
                   "factor_construction.history_raw.uwb_only_epochs");
  requireExactKeys(history["imu_and_uwb_epochs"],
                   {"first", "last", "imu_rows_each", "uwb_rows_each"},
                   "factor_construction.history_raw.imu_and_uwb_epochs");
  requireExactKeys(history["imu_only_epochs"], {"first", "last", "rows_each"},
                   "factor_construction.history_raw.imu_only_epochs");
  requireExactKeys(history["slot_rules"],
                   {"imu_multiplier", "imu_add", "uwb_multiplier", "uwb_add"},
                   "factor_construction.history_raw.slot_rules");
  requireExactKeys(history["group_rules"],
                   {"epoch_multiplier", "imu_add", "uwb_add"},
                   "factor_construction.history_raw.group_rules");
  requireExactKeys(history["row_identity"],
                   {"prior_prefix", "imu_prefix", "uwb_prefix"},
                   "factor_construction.history_raw.row_identity");
  const auto slot_rules = history["slot_rules"];
  out.slot_imu_multiplier = slot_rules["imu_multiplier"].as<std::uint64_t>();
  out.slot_imu_add = slot_rules["imu_add"].as<std::uint64_t>();
  out.slot_uwb_multiplier = slot_rules["uwb_multiplier"].as<std::uint64_t>();
  out.slot_uwb_add = slot_rules["uwb_add"].as<std::uint64_t>();
  const auto group_rules = history["group_rules"];
  out.group_epoch_multiplier = group_rules["epoch_multiplier"].as<std::uint64_t>();
  out.group_imu_add = group_rules["imu_add"].as<std::uint64_t>();
  out.group_uwb_add = group_rules["uwb_add"].as<std::uint64_t>();
  const auto row_identity = history["row_identity"];
  out.prior_row_prefix = row_identity["prior_prefix"].as<std::string>();
  out.imu_row_prefix = row_identity["imu_prefix"].as<std::string>();
  out.uwb_row_prefix = row_identity["uwb_prefix"].as<std::string>();
  if (history["marginal_prior"]["owner"].as<std::string>() !=
      out.prior_row_prefix) {
    throw std::runtime_error("marginal prior owner and row prefix differ");
  }
  out.history_factor_order = yamlStrings(history["factor_order"]);
  out.history_uwb_only_first =
      history["uwb_only_epochs"]["first"].as<std::size_t>();
  out.history_uwb_only_last =
      history["uwb_only_epochs"]["last"].as<std::size_t>();
  out.history_uwb_only_rows =
      history["uwb_only_epochs"]["rows_each"].as<std::size_t>();
  out.history_both_first =
      history["imu_and_uwb_epochs"]["first"].as<std::size_t>();
  out.history_both_last =
      history["imu_and_uwb_epochs"]["last"].as<std::size_t>();
  out.history_both_imu_rows =
      history["imu_and_uwb_epochs"]["imu_rows_each"].as<std::size_t>();
  out.history_both_uwb_rows =
      history["imu_and_uwb_epochs"]["uwb_rows_each"].as<std::size_t>();
  out.history_imu_only_first =
      history["imu_only_epochs"]["first"].as<std::size_t>();
  out.history_imu_only_last =
      history["imu_only_epochs"]["last"].as<std::size_t>();
  out.history_imu_only_rows =
      history["imu_only_epochs"]["rows_each"].as<std::size_t>();
  if (out.history_uwb_only_first > out.history_uwb_only_last ||
      out.history_both_first > out.history_both_last ||
      out.history_imu_only_first > out.history_imu_only_last ||
      out.history_uwb_only_last + 1 != out.history_both_first ||
      out.history_both_last + 1 != out.history_imu_only_first ||
      out.history_uwb_only_rows != out.anchors.size() ||
      out.history_both_uwb_rows != out.history_uwb_only_rows ||
      out.history_both_imu_rows == 0 ||
      out.history_imu_only_rows != out.history_both_imu_rows) {
    throw std::runtime_error("oracle history epoch/row partitions do not close");
  }
  out.history_uwb_first = out.history_uwb_only_first;
  out.history_uwb_last = out.history_both_last;
  out.history_imu_first = out.history_both_first;
  out.history_imu_last = out.history_imu_only_last;
  out.uwb_rows = out.history_uwb_only_rows;
  out.imu_rows = out.history_both_imu_rows;
  const auto served = factors["served_window"];
  requireExactKeys(served, {"expected_rows", "history_summary_rows",
                            "history_summary_group_id", "explicit_uwb_epochs",
                            "explicit_imu_epochs", "order"},
                   "factor_construction.served_window");
  requireExactKeys(served["explicit_uwb_epochs"],
                   {"first", "last", "rows_each"},
                   "factor_construction.served_window.explicit_uwb_epochs");
  requireExactKeys(served["explicit_imu_epochs"],
                   {"first", "last", "rows_each"},
                   "factor_construction.served_window.explicit_imu_epochs");
  requireExactKeys(factors["whitening"], {"uwb", "imu", "history"},
                   "factor_construction.whitening");
  out.explicit_uwb_first = served["explicit_uwb_epochs"]["first"].as<std::size_t>();
  out.explicit_uwb_last = served["explicit_uwb_epochs"]["last"].as<std::size_t>();
  out.explicit_imu_first = served["explicit_imu_epochs"]["first"].as<std::size_t>();
  out.explicit_imu_last = served["explicit_imu_epochs"]["last"].as<std::size_t>();
  out.explicit_uwb_rows =
      served["explicit_uwb_epochs"]["rows_each"].as<std::size_t>();
  out.explicit_imu_rows =
      served["explicit_imu_epochs"]["rows_each"].as<std::size_t>();
  if (out.explicit_uwb_first > out.explicit_uwb_last ||
      out.explicit_imu_first > out.explicit_imu_last ||
      out.explicit_uwb_rows != out.uwb_rows ||
      out.explicit_imu_rows != out.imu_rows) {
    throw std::runtime_error("oracle served explicit row partitions do not close");
  }
  if (served["order"].as<std::string>() !=
      "history-summary, then epoch ascending with IMU before UWB except first UWB-only epoch") {
    throw std::runtime_error("unsupported served-window row order");
  }
  if (factors["whitening"]["uwb"].as<std::string>() !=
          "upper information square root R with R^T R=C^-1" ||
      factors["whitening"]["imu"].as<std::string>() !=
          "factor-native whitened Jacobian; raw coordinate recovered with declared covariance" ||
      factors["whitening"]["history"].as<std::string>() !=
          "Householder elimination of recipe old columns and deterministic retained rows") {
    throw std::runtime_error("unsupported oracle whitening declaration");
  }
  const std::size_t derived_history_rows =
      history["marginal_prior"]["rows"].as<std::size_t>() +
      (out.history_uwb_only_last - out.history_uwb_only_first + 1) *
          out.history_uwb_only_rows +
      (out.history_both_last - out.history_both_first + 1) *
          (out.history_both_uwb_rows + out.history_both_imu_rows) +
      (out.history_imu_only_last - out.history_imu_only_first + 1) *
          out.history_imu_only_rows;
  if (derived_history_rows != out.history_rows) {
    throw std::runtime_error("oracle history row recipe does not close");
  }
  const auto imu_group_id = [&](std::size_t epoch) {
    return epoch * out.group_epoch_multiplier + out.group_imu_add;
  };
  const auto uwb_group_id = [&](std::size_t epoch) {
    return epoch * out.group_epoch_multiplier + out.group_uwb_add;
  };
  std::set<std::uint64_t> history_groups{out.marginal_group};
  std::set<std::string> history_owners;
  for (std::size_t row = 0; row < out.marginal_rows; ++row) {
    history_owners.insert(out.prior_row_prefix + ":" + std::to_string(row));
  }
  for (std::size_t epoch = out.history_uwb_first;
       epoch <= out.history_uwb_last; ++epoch) {
    if (!history_groups.insert(uwb_group_id(epoch)).second) {
      throw std::runtime_error("duplicate oracle history UWB group owner");
    }
    for (std::size_t anchor = 1; anchor <= out.uwb_rows; ++anchor) {
      history_owners.insert(out.uwb_row_prefix + ":" +
                            std::to_string(epoch) + ":" +
                            std::to_string(out.anchors.at(anchor - 1)));
    }
  }
  for (std::size_t epoch = out.history_imu_first;
       epoch <= out.history_imu_last; ++epoch) {
    if (!history_groups.insert(imu_group_id(epoch)).second) {
      throw std::runtime_error("duplicate oracle history IMU group owner");
    }
    for (std::size_t row = 0; row < out.imu_rows; ++row) {
      history_owners.insert(out.imu_row_prefix + ":" + std::to_string(epoch) + ":" +
                            std::to_string(row));
    }
  }
  if (history_owners.size() != out.history_rows) {
    throw std::runtime_error("oracle raw row/owner identities are not unique");
  }
  const std::size_t derived_served_rows = out.summary_rows +
      (out.explicit_uwb_last - out.explicit_uwb_first + 1) *
          out.explicit_uwb_rows +
      (out.explicit_imu_last - out.explicit_imu_first + 1) *
          out.explicit_imu_rows;
  if (derived_served_rows != out.served_rows) {
    throw std::runtime_error("oracle served row recipe does not close");
  }
  std::set<std::uint64_t> served_groups{out.summary_group};
  for (std::size_t epoch = out.explicit_uwb_first;
       epoch <= out.explicit_uwb_last; ++epoch) {
    if (!served_groups.insert(uwb_group_id(epoch)).second) {
      throw std::runtime_error("duplicate oracle served UWB owner");
    }
  }
  for (std::size_t epoch = out.explicit_imu_first;
       epoch <= out.explicit_imu_last; ++epoch) {
    if (!served_groups.insert(imu_group_id(epoch)).second) {
      throw std::runtime_error("duplicate oracle served IMU owner");
    }
  }
  const auto fault = root["fault_contract"];
  requireExactKeys(fault, {"families", "onset_epochs",
                           "window_boundary_onset_rule", "mode_order",
                           "hypothesis_rule", "double_fault_families"},
                   "fault_contract");
  requireExactKeys(fault["onset_epochs"], {"first", "last"},
                   "fault_contract.onset_epochs");
  out.onset_first = fault["onset_epochs"]["first"].as<std::size_t>();
  out.onset_last = fault["onset_epochs"]["last"].as<std::size_t>();
  out.window_boundary_onset_rule =
      fault["window_boundary_onset_rule"].as<std::string>();
  if (out.window_boundary_onset_rule !=
      "persistent/ramp onset at window-first extends to physical-first") {
    throw std::runtime_error("unsupported window-boundary onset rule");
  }
  out.mode_order = fault["mode_order"].as<std::string>();
  out.hypothesis_rule = fault["hypothesis_rule"].as<std::string>();
  out.double_fault_families = yamlStrings(fault["double_fault_families"]);
  std::set<int> family_kinds;
  for (const auto& item : fault["families"]) {
    std::set<std::string> family_keys = {"kind", "name", "dimension",
        "terminal_onset_effective_dimension", "support"};
    if (item["native_columns"]) family_keys.insert("native_columns");
    std::set<std::string> actual_family_keys;
    for (const auto& entry : item) {
      actual_family_keys.insert(entry.first.as<std::string>());
    }
    if (actual_family_keys != family_keys) {
      throw std::runtime_error("unconsumed oracle fault-family field");
    }
    OracleFamily family;
    family.kind = item["kind"].as<int>();
    family.dimension = item["dimension"].as<int>();
    family.terminal_onset_effective_dimension =
        item["terminal_onset_effective_dimension"].as<int>();
    family.name = item["name"].as<std::string>();
    family.support = item["support"].as<std::string>();
    if (item["native_columns"]) family.native_columns = yamlStrings(item["native_columns"]);
    if (!family_kinds.insert(family.kind).second || family.dimension <= 0 ||
        family.terminal_onset_effective_dimension <= 0 ||
        family.terminal_onset_effective_dimension > family.dimension) {
      throw std::runtime_error("invalid or duplicate oracle fault family");
    }
    if ((family.kind == 1 &&
         (family.name != "AnchorBiasEpochIndependent" ||
          family.dimension != 1 || family.support != "onset-only" ||
          !family.native_columns.empty())) ||
        (family.kind == 2 &&
         (family.name != "AnchorBiasPersistentConstant" ||
          family.dimension != 1 || family.support != "at-or-after-onset" ||
          !family.native_columns.empty())) ||
        (family.kind == 3 &&
         (family.name != "AnchorBiasRamp" || family.dimension != 2 ||
          family.support != "at-or-after-onset" ||
          family.native_columns != std::vector<std::string>(
              {"constant", "measurement-time-minus-onset-time"}))) ||
        (family.kind != 1 && family.kind != 2 && family.kind != 3)) {
      throw std::runtime_error("unsupported oracle fault family contract");
    }
    out.families.push_back(std::move(family));
  }
  out.expected_modes = out.anchors.size() *
      (out.onset_last - out.onset_first + 1) * out.families.size();
  out.expected_hypotheses = out.expected_modes;
  const auto recipe = root["detector_and_action_recipe"];
  requireExactKeys(recipe, {"detector", "action_generation", "risk",
                            "selection", "resolved_config_binding"},
                   "detector_and_action_recipe");
  requireExactKeys(recipe["detector"],
                   {"distribution", "type", "p_fa_per_test",
                    "plausible_profile_margin", "use_squared_norm_statistic"},
                   "detector_and_action_recipe.detector");
  out.detector_distribution =
      recipe["detector"]["distribution"].as<std::string>();
  out.detector_type = recipe["detector"]["type"].as<std::string>();
  out.detector_p_fa = recipe["detector"]["p_fa_per_test"].as<double>();
  out.plausible_margin = recipe["detector"]["plausible_profile_margin"].as<double>();
  out.detector_squared_norm = recipe["detector"]["use_squared_norm_statistic"].as<bool>();
  const auto action = recipe["action_generation"];
  requireExactKeys(action, {"keep_all_first",
                            "one_occurrence_per_plausible_hypothesis",
                            "full_union_when_single_anchor",
                            "max_exclusion_cardinality", "replacement_group",
                            "identity_fields", "exact_duplicates"},
                   "detector_and_action_recipe.action_generation");
  out.keep_all_first = action["keep_all_first"].as<bool>();
  out.one_occurrence_per_plausible = action["one_occurrence_per_plausible_hypothesis"].as<bool>();
  out.union_when_single_anchor = action["full_union_when_single_anchor"].as<bool>();
  out.max_exclusion_cardinality = action["max_exclusion_cardinality"].as<std::uint32_t>();
  out.identity_fields = yamlStrings(action["identity_fields"]);
  out.exact_duplicate_rule = action["exact_duplicates"].as<std::string>();
  const auto replacement = action["replacement_group"];
  requireExactKeys(replacement, {"historical_transaction_multiplier",
                                 "historical_epoch_multiplier",
                                 "current_transaction_multiplier",
                                 "anchor_base", "anchor_stride"},
                   "detector_and_action_recipe.action_generation.replacement_group");
  out.historical_replacement_multiplier = replacement["historical_transaction_multiplier"].as<std::uint64_t>();
  out.historical_epoch_multiplier = replacement["historical_epoch_multiplier"].as<std::uint64_t>();
  out.current_replacement_multiplier = replacement["current_transaction_multiplier"].as<std::uint64_t>();
  out.anchor_replacement_base = replacement["anchor_base"].as<std::uint64_t>();
  out.anchor_replacement_stride = replacement["anchor_stride"].as<std::uint64_t>();
  const auto risk = recipe["risk"];
  requireExactKeys(risk, {"total_decimal", "nominal_axis_tail", "p_nm",
                          "p_bridge_escape", "p_history_contamination",
                          "p_model_escape", "horizontal_alert_limit_m",
                          "vertical_alert_limit_m", "allocation_policy",
                          "allocation_rule", "model_calibration",
                          "unknown_terms_are_zero"},
                   "detector_and_action_recipe.risk");
  out.total_risk = std::stod(risk["total_decimal"].as<std::string>());
  out.nominal_axis_tail = risk["nominal_axis_tail"].as<double>();
  out.p_nm = risk["p_nm"].as<double>();
  out.p_bridge_escape = risk["p_bridge_escape"].as<double>();
  out.p_history_contamination = risk["p_history_contamination"].as<double>();
  out.p_model_escape = risk["p_model_escape"].as<double>();
  out.horizontal_alert_limit_m = risk["horizontal_alert_limit_m"].as<double>();
  out.vertical_alert_limit_m = risk["vertical_alert_limit_m"].as<double>();
  out.risk_allocation_policy = risk["allocation_policy"].as<std::string>();
  out.risk_allocation_rule = risk["allocation_rule"].as<std::string>();
  out.risk_model_calibration = risk["model_calibration"].as<std::string>();
  out.risk_unknown_terms_are_zero = risk["unknown_terms_are_zero"].as<bool>();
  if (out.risk_unknown_terms_are_zero) {
    throw std::runtime_error("oracle risk recipe illegally treats unknown as zero");
  }
  const auto selection = recipe["selection"];
  requireExactKeys(selection, {"primary", "secondary", "tertiary", "final"},
                   "detector_and_action_recipe.selection");
  out.selection_order = {{selection["primary"].as<std::string>(),
                          selection["secondary"].as<std::string>(),
                          selection["tertiary"].as<std::string>(),
                          selection["final"].as<std::string>()}};
  const auto binding = recipe["resolved_config_binding"];
  requireExactKeys(binding, {"fde_profile", "fault_max_cardinality",
                             "uwb_enabled", "epoch_single_anchor_bias",
                             "persistent_anchor_bias", "ramp_bias",
                             "single_faults_enabled", "double_faults_enabled",
                             "resolved_max_fault_order",
                             "uwb_prior_probability_bound", "uwb_p_md",
                             "action_trigger", "action_isolation",
                             "ambiguity_policy", "dense_oracle_online_fallback",
                             "max_candidate_count", "on_no_valid_action",
                             "on_integrity_model_invalid", "fault_manifest_path",
                             "fault_manifest_id"},
                   "detector_and_action_recipe.resolved_config_binding");
  out.fde_profile = binding["fde_profile"].as<std::string>();
  out.fault_max_cardinality = binding["fault_max_cardinality"].as<std::uint32_t>();
  out.uwb_enabled = binding["uwb_enabled"].as<bool>();
  out.epoch_single_enabled = binding["epoch_single_anchor_bias"].as<bool>();
  out.persistent_enabled = binding["persistent_anchor_bias"].as<bool>();
  out.ramp_enabled = binding["ramp_bias"].as<bool>();
  out.single_faults_enabled = binding["single_faults_enabled"].as<bool>();
  out.double_faults_enabled = binding["double_faults_enabled"].as<bool>();
  out.resolved_max_fault_order = binding["resolved_max_fault_order"].as<int>();
  out.uwb_prior_probability_bound =
      binding["uwb_prior_probability_bound"].as<double>();
  out.uwb_p_md = binding["uwb_p_md"].as<double>();
  out.action_trigger = binding["action_trigger"].as<std::string>();
  out.action_isolation = binding["action_isolation"].as<std::string>();
  out.ambiguity_policy = binding["ambiguity_policy"].as<std::string>();
  out.dense_oracle_online_fallback =
      binding["dense_oracle_online_fallback"].as<bool>();
  out.max_candidate_count = binding["max_candidate_count"].as<std::uint32_t>();
  out.on_no_valid_action = binding["on_no_valid_action"].as<std::string>();
  out.on_integrity_model_invalid =
      binding["on_integrity_model_invalid"].as<std::string>();
  out.fault_manifest_path = binding["fault_manifest_path"].as<std::string>();
  out.fault_manifest_id = binding["fault_manifest_id"].as<std::string>();
  const auto missing = root["missing_provenance_fixture"];
  requireExactKeys(missing, {"recipe", "anchor", "onset_epoch",
                             "family_kind", "expected_recoverability_rule"},
                   "missing_provenance_fixture");
  out.missing_anchor = missing["anchor"].as<std::size_t>();
  out.missing_onset = missing["onset_epoch"].as<std::size_t>();
  out.missing_family = missing["family_kind"].as<int>();
  out.missing_recipe = missing["recipe"].as<std::string>();
  out.missing_recoverability_rule =
      missing["expected_recoverability_rule"].as<std::string>();
  if (out.missing_recipe !=
          "same raw alarm replay with the anchor-1 current replacement factor deliberately absent before generation" ||
      out.missing_recoverability_rule !=
          "nominal affected group is explicit but its recipe replacement group is absent, so generated action is MissingProvenance" ||
      out.missing_anchor == 0 || out.missing_onset < out.onset_first ||
      out.missing_onset > out.onset_last ||
      std::find(out.anchors.begin(), out.anchors.end(), out.missing_anchor) ==
          out.anchors.end() ||
      family_kinds.count(out.missing_family) != 1) {
    throw std::runtime_error("invalid MissingProvenance construction recipe");
  }
  const auto observed = root["observed_non_authoritative"];
  requireExactKeys(observed,
                   {"plausible_mode_ordinals", "generated_action_count",
                    "winner", "note"}, "observed_non_authoritative");
  out.observed_plausible_modes =
      yamlU64(observed["plausible_mode_ordinals"]);
  out.observed_generated_action_count =
      observed["generated_action_count"].as<std::size_t>();
  out.observed_winner = observed["winner"].as<std::uint64_t>();
  out.observed_note = observed["note"].as<std::string>();
  // These four display-only fields are intentionally not construction
  // constraints.  A separate validator compares them with actual replay
  // observations; independent expected artifacts must not consume them.
  if (out.detector_p_fa <= 0.0 || out.detector_p_fa >= 1.0 ||
      out.total_risk <= 0.0 || out.state_first_epoch != out.detector_first_epoch ||
      out.window_first_epoch != out.detector_first_epoch ||
      out.mode_order != "anchor ascending, onset ascending, family array order" ||
      out.hypothesis_rule != "one single-mode hypothesis per enumerated mode" ||
      !out.double_fault_families.empty() ||
      out.detector_distribution != "chi-square" ||
      out.risk_allocation_rule != "total/generated-action-count" ||
      out.risk_model_calibration != "UNKNOWN_N01_NO_AUTHENTICATED_ARTIFACT" ||
      !out.keep_all_first || !out.one_occurrence_per_plausible ||
      out.identity_fields != std::vector<std::string>({"sorted_remove", "sorted_add", "exact_added_block_bytes", "bridge_mode"}) ||
      out.selection_order != std::array<std::string, 4>{{"minimum_cardinality", "minimum_protection_level", "maximum_information_logdet", "action_id"}}) {
    throw std::runtime_error("invalid oracle detector/risk/state recipe");
  }
  out.epsilon = root["tolerance_policy"]["epsilon"].as<double>();
  requireExactKeys(root["tolerance_policy"],
                   {"formula", "epsilon", "maximum_multiplier",
                    "forbid_blanket_tolerance",
                    "classifications_always_compared"},
                   "tolerance_policy");
  out.tolerance_classifications = yamlStrings(
      root["tolerance_policy"]["classifications_always_compared"]);
  const std::vector<std::string> required_classifications = {
      "profile_valid", "pl_valid", "nullspace_class", "finite_or_infinite",
      "risk_valid", "eligibility"};
  if (!root["tolerance_policy"]["forbid_blanket_tolerance"].as<bool>() ||
      root["tolerance_policy"]["formula"].as<std::string>() !=
          "abs <= eps * max(1,norm_expected) * max(32,dimension) * max(1,condition_certificate)" ||
      out.tolerance_classifications != required_classifications) {
    throw std::runtime_error("unsupported oracle tolerance policy");
  }
  out.tolerance_cap = root["tolerance_policy"]["maximum_multiplier"].as<double>();
  return out;
}

OracleManifest loadOracleManifest() {
  return parseOracleManifest(YAML::LoadFile(
      std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/docs/evidence/p0-07-corrected-exhaustive/oracle-manifest-v1.json"));
}

struct RecipePathSegment {
  bool sequence = false;
  std::string key;
  std::size_t index = 0;
};

using RecipeLeafPath = std::vector<RecipePathSegment>;

void collectRecipeLeafPaths(const YAML::Node& node, RecipeLeafPath* path,
                            std::vector<RecipeLeafPath>* leaves) {
  if (node.IsScalar() || node.IsNull() ||
      ((node.IsSequence() || node.IsMap()) && node.size() == 0)) {
    leaves->push_back(*path);
    return;
  }
  if (node.IsMap()) {
    for (const auto& item : node) {
      path->push_back({false, item.first.as<std::string>(), 0});
      collectRecipeLeafPaths(item.second, path, leaves);
      path->pop_back();
    }
  } else if (node.IsSequence()) {
    for (std::size_t i = 0; i < node.size(); ++i) {
      path->push_back({true, std::string(), i});
      collectRecipeLeafPaths(node[i], path, leaves);
      path->pop_back();
    }
  }
}

std::string recipePathString(const RecipeLeafPath& path) {
  std::ostringstream out;
  for (const auto& segment : path) {
    if (segment.sequence) {
      out << '[' << segment.index << ']';
    } else {
      if (out.tellp() > 0) out << '.';
      out << segment.key;
    }
  }
  return out.str();
}

void mutateRecipeScalar(YAML::Node node) {
  if (node.IsSequence() && node.size() == 0) {
    node.push_back("__MUTATED__");
    return;
  }
  if (node.IsMap() && node.size() == 0) {
    node["__MUTATED__"] = true;
    return;
  }
  const std::string scalar = node.Scalar();
  if (scalar == "true") {
    node = false;
    return;
  }
  if (scalar == "false") {
    node = true;
    return;
  }
  const bool unsigned_integer = !scalar.empty() && std::all_of(
      scalar.begin(), scalar.end(), [](unsigned char c) { return std::isdigit(c); });
  if (unsigned_integer) {
    node = std::to_string(std::stoull(scalar) + 1ULL);
    return;
  }
  std::size_t consumed = 0;
  try {
    const double numeric = std::stod(scalar, &consumed);
    if (consumed == scalar.size() && std::isfinite(numeric)) {
      std::ostringstream changed;
      changed << std::setprecision(17)
              << (numeric == 0.0 ? 0.125 : numeric * 1.25);
      node = changed.str();
      return;
    }
  } catch (const std::exception&) {
  }
  node = scalar + "__MUTATED__";
}

void mutateRecipeAtPath(YAML::Node node, const RecipeLeafPath& path,
                        std::size_t depth = 0) {
  if (depth == path.size()) {
    mutateRecipeScalar(node);
    return;
  }
  const auto& segment = path[depth];
  mutateRecipeAtPath(segment.sequence ? node[segment.index] : node[segment.key],
                     path, depth + 1);
}

void verifyRecipeConfigBinding(const OracleManifest& recipe,
                               const IntegrityConfig& config) {
  EXPECT_DOUBLE_EQ(recipe.detector_p_fa, config.detector.p_fa_per_test);
  EXPECT_EQ(recipe.detector_type, config.detector.type);
  EXPECT_EQ(recipe.detector_squared_norm,
            config.detector.use_squared_norm_statistic);
  EXPECT_DOUBLE_EQ(recipe.total_risk, config.risk_v2.p_hmi_total);
  EXPECT_DOUBLE_EQ(recipe.nominal_axis_tail,
                   config.risk_v2.nominal_axis_tail);
  EXPECT_DOUBLE_EQ(recipe.p_nm, config.risk_v2.p_nm);
  EXPECT_DOUBLE_EQ(recipe.p_bridge_escape, config.risk_v2.p_bridge_escape);
  EXPECT_DOUBLE_EQ(recipe.p_history_contamination,
                   config.risk_v2.p_history_contamination);
  EXPECT_DOUBLE_EQ(recipe.p_model_escape, config.risk_v2.p_model_escape);
  EXPECT_DOUBLE_EQ(recipe.horizontal_alert_limit_m,
                   config.risk_v2.horizontal_alert_limit_m);
  EXPECT_DOUBLE_EQ(recipe.vertical_alert_limit_m,
                   config.risk_v2.vertical_alert_limit_m);
  EXPECT_EQ(recipe.risk_allocation_policy,
            config.risk_v2.allocation_policy);
  EXPECT_EQ(recipe.risk_allocation_rule, "total/generated-action-count");
  EXPECT_EQ(recipe.risk_model_calibration,
            "UNKNOWN_N01_NO_AUTHENTICATED_ARTIFACT");
  EXPECT_EQ(recipe.max_exclusion_cardinality,
            config.fde.max_exclusion_cardinality);
  EXPECT_EQ(recipe.fault_max_cardinality,
            config.fault_models.max_cardinality);
  EXPECT_EQ(recipe.uwb_enabled, config.fault_models.uwb.enabled);
  EXPECT_EQ(recipe.epoch_single_enabled,
            config.fault_models.uwb.epoch_single_anchor_bias);
  EXPECT_EQ(recipe.persistent_enabled,
            config.fault_models.uwb.persistent_anchor_bias);
  EXPECT_EQ(recipe.ramp_enabled, config.fault_models.uwb.ramp_bias);
  EXPECT_EQ(recipe.resolved_max_fault_order,
            config.resolved_scope.max_fault_order);
  EXPECT_EQ(recipe.action_trigger, config.fde.trigger);
  EXPECT_EQ(recipe.action_isolation, config.fde.isolation);
  EXPECT_EQ(recipe.ambiguity_policy, config.fde.ambiguity_policy);
  EXPECT_EQ(recipe.dense_oracle_online_fallback,
            config.fde.dense_oracle_online_fallback);
  EXPECT_EQ(recipe.max_candidate_count, config.fde.max_candidate_count);
  EXPECT_EQ(recipe.on_no_valid_action, config.fde.on_no_valid_action);
  EXPECT_EQ(recipe.on_integrity_model_invalid,
            config.fde.on_integrity_model_invalid);
  EXPECT_EQ(recipe.selection_order[0], config.fde.selection_primary);
  EXPECT_EQ(recipe.selection_order[1], config.fde.selection_secondary);
  EXPECT_EQ(recipe.fde_profile, toString(config.fde.profile));
  EXPECT_EQ(recipe.single_faults_enabled,
            config.resolved_scope.max_fault_order >= 1);
  EXPECT_EQ(recipe.double_faults_enabled,
            config.resolved_scope.max_fault_order >= 2);
  EXPECT_DOUBLE_EQ(recipe.uwb_prior_probability_bound,
                   config.fault_models.uwb.prior_probability_bound);
  EXPECT_DOUBLE_EQ(recipe.uwb_p_md, config.fault_models.uwb.p_md);
  EXPECT_EQ(recipe.fault_manifest_path, config.fault_models.manifest_path);
  ASSERT_TRUE(config.fault_manifest.has_value());
  EXPECT_EQ(recipe.fault_manifest_id, config.fault_manifest->manifest_id);
  const std::size_t enabled_family_count =
      static_cast<std::size_t>(config.fault_models.uwb.epoch_single_anchor_bias) +
      static_cast<std::size_t>(config.fault_models.uwb.persistent_anchor_bias) +
      static_cast<std::size_t>(config.fault_models.uwb.ramp_bias);
  EXPECT_EQ(recipe.families.size(), enabled_family_count);
  for (const auto& family : recipe.families) {
    if (family.kind == static_cast<int>(FaultKind::AnchorBiasEpochIndependent)) {
      EXPECT_EQ(family.dimension, 1);
      EXPECT_EQ(family.support, "onset-only");
      EXPECT_TRUE(config.fault_models.uwb.epoch_single_anchor_bias);
    } else if (family.kind ==
               static_cast<int>(FaultKind::AnchorBiasPersistentConstant)) {
      EXPECT_EQ(family.dimension, 1);
      EXPECT_EQ(family.support, "at-or-after-onset");
      EXPECT_TRUE(config.fault_models.uwb.persistent_anchor_bias);
    } else if (family.kind == static_cast<int>(FaultKind::AnchorBiasRamp)) {
      EXPECT_EQ(family.dimension, 2);
      EXPECT_EQ(family.support, "at-or-after-onset");
      EXPECT_EQ(family.native_columns,
                std::vector<std::string>({"constant", "measurement-time-minus-onset-time"}));
      EXPECT_TRUE(config.fault_models.uwb.ramp_bias);
    } else {
      ADD_FAILURE() << "recipe declares an unbound fault family " << family.kind;
    }
  }

  const YAML::Node raw = YAML::LoadFile(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/" +
      recipe.fault_manifest_path);
  requireExactKeys(raw, {"schema_version", "manifest_id",
                         "protected_quantity", "position_reference",
                         "max_fault_order", "enabled_single_families",
                         "enabled_pair_families", "unsupported_families",
                         "events", "families"}, "fault_manifest");
  const FaultManifest& actual = *config.fault_manifest;
  EXPECT_EQ(raw["schema_version"].as<int>(), actual.schema_version);
  EXPECT_EQ(raw["manifest_id"].as<std::string>(), actual.manifest_id);
  EXPECT_EQ(raw["protected_quantity"].as<std::string>(),
            actual.protected_quantity);
  EXPECT_EQ(raw["position_reference"].as<std::string>(),
            actual.position_reference);
  EXPECT_EQ(raw["max_fault_order"].as<int>(), actual.max_fault_order);
  EXPECT_EQ(yamlStrings(raw["enabled_single_families"]),
            actual.enabled_single_families);
  EXPECT_EQ(yamlStrings(raw["enabled_pair_families"]),
            actual.enabled_pair_families);
  EXPECT_EQ(yamlStrings(raw["unsupported_families"]),
            actual.unsupported_families);
  ASSERT_EQ(raw["events"].size(), actual.events.size());
  for (std::size_t i = 0; i < actual.events.size(); ++i) {
    const auto node = raw["events"][i];
    requireExactKeys(node, {"source_id", "common_cause_group", "event_id",
                            "model_type", "parameter_units",
                            "parameter_sharing", "onset_domain",
                            "end_condition", "physical_time_support",
                            "amplitude_model", "prior_bound_source",
                            "prior_time_basis", "evidence_status",
                            "included_hypothesis_families",
                            "omitted_event_set",
                            "history_retention_or_reset_rule",
                            "allowed_actions", "projection_eligibility"},
                     "fault_manifest.events[]");
    const auto& event = actual.events[i];
    EXPECT_EQ(node["source_id"].as<std::string>(), event.source_id);
    EXPECT_EQ(node["common_cause_group"].as<std::string>(),
              event.common_cause_group);
    EXPECT_EQ(node["event_id"].as<std::string>(), event.event_id);
    EXPECT_EQ(node["model_type"].as<std::string>(), event.model_type);
    EXPECT_EQ(node["parameter_units"].as<std::string>(), event.parameter_units);
    EXPECT_EQ(node["parameter_sharing"].as<std::string>(),
              event.parameter_sharing);
    EXPECT_EQ(node["onset_domain"].as<std::string>(), event.onset_domain);
    EXPECT_EQ(node["end_condition"].as<std::string>(), event.end_condition);
    EXPECT_EQ(node["physical_time_support"].as<std::string>(),
              event.physical_time_support);
    EXPECT_EQ(node["amplitude_model"].as<std::string>(), event.amplitude_model);
    EXPECT_EQ(node["prior_bound_source"].as<std::string>(),
              event.prior_bound_source);
    EXPECT_EQ(node["prior_time_basis"].as<std::string>(), event.prior_time_basis);
    EXPECT_EQ(node["evidence_status"].as<std::string>(), event.evidence_status);
    EXPECT_EQ(yamlStrings(node["included_hypothesis_families"]),
              event.included_hypothesis_families);
    EXPECT_EQ(yamlStrings(node["omitted_event_set"]), event.omitted_event_set);
    EXPECT_EQ(node["history_retention_or_reset_rule"].as<std::string>(),
              event.history_retention_or_reset_rule);
    EXPECT_EQ(yamlStrings(node["allowed_actions"]), event.allowed_actions);
    EXPECT_EQ(node["projection_eligibility"].as<std::string>(),
              event.projection_eligibility);
  }
  ASSERT_EQ(raw["families"].size(), actual.families.size());
  for (std::size_t i = 0; i < actual.families.size(); ++i) {
    const auto node = raw["families"][i];
    requireExactKeys(node, {"family_id", "scope", "members"},
                     "fault_manifest.families[]");
    EXPECT_EQ(node["family_id"].as<std::string>(), actual.families[i].family_id);
    EXPECT_EQ(node["scope"].as<std::string>(), actual.families[i].scope);
    EXPECT_EQ(yamlStrings(node["members"]), actual.families[i].members);
  }
}

std::uint64_t oracleImuGroup(std::size_t epoch,
                             const OracleManifest& recipe) {
  return epoch * recipe.group_epoch_multiplier + recipe.group_imu_add;
}

std::uint64_t oracleUwbGroup(std::size_t epoch,
                             const OracleManifest& recipe) {
  return epoch * recipe.group_epoch_multiplier + recipe.group_uwb_add;
}

std::uint64_t oracleImuSlot(std::size_t epoch,
                            const OracleManifest& recipe) {
  return epoch * recipe.slot_imu_multiplier + recipe.slot_imu_add;
}

std::uint64_t oracleUwbSlot(std::size_t epoch,
                            const OracleManifest& recipe) {
  return epoch * recipe.slot_uwb_multiplier + recipe.slot_uwb_add;
}

struct OracleModeRecipe {
  std::uint64_t ordinal = 0;
  std::uint64_t anchor = 0;
  std::size_t onset = 0;
  OracleFamily family;
};

OracleModeRecipe oracleMode(std::uint64_t ordinal,
                            const OracleManifest& manifest) {
  if (ordinal == 0 || ordinal > manifest.expected_modes) {
    throw std::runtime_error("oracle mode ordinal outside recipe census");
  }
  const std::size_t onsets = manifest.onset_last - manifest.onset_first + 1;
  const std::size_t per_anchor = onsets * manifest.families.size();
  const std::size_t zero = static_cast<std::size_t>(ordinal - 1);
  OracleModeRecipe out;
  out.ordinal = ordinal;
  out.anchor = manifest.anchors.at(zero / per_anchor);
  const std::size_t within = zero % per_anchor;
  out.onset = manifest.onset_first + within / manifest.families.size();
  out.family = manifest.families.at(within % manifest.families.size());
  return out;
}

std::vector<std::size_t> oracleModeEpochs(const OracleModeRecipe& mode,
                                          const OracleManifest& manifest) {
  if (mode.family.support == "onset-only") return {mode.onset};
  if (mode.family.support != "at-or-after-onset") {
    throw std::runtime_error("unknown oracle fault support rule");
  }
  std::vector<std::size_t> out;
  for (std::size_t epoch = mode.onset; epoch <= manifest.alarm_epoch; ++epoch) {
    out.push_back(epoch);
  }
  return out;
}

std::size_t oracleEffectiveBoundaryOnset(const OracleModeRecipe& mode,
                                         const OracleManifest& manifest) {
  if (manifest.window_boundary_onset_rule !=
      "persistent/ramp onset at window-first extends to physical-first") {
    throw std::runtime_error("unknown oracle window-boundary onset rule");
  }
  return mode.family.support == "at-or-after-onset" &&
                 mode.onset == manifest.window_first_epoch
      ? manifest.physical_first_epoch : mode.onset;
}

std::uint64_t oracleReplacementGroup(std::size_t epoch,
                                     std::uint64_t anchor,
                                     const OracleManifest& manifest) {
  const auto anchor_position = std::find(manifest.anchors.begin(),
                                         manifest.anchors.end(), anchor);
  if (anchor_position == manifest.anchors.end()) {
    throw std::runtime_error("replacement recipe anchor is undeclared");
  }
  const std::uint64_t anchor_ordinal = static_cast<std::uint64_t>(
      std::distance(manifest.anchors.begin(), anchor_position));
  if (epoch == manifest.alarm_epoch) {
    return manifest.alarm_epoch * manifest.current_replacement_multiplier +
        manifest.anchor_replacement_base +
        anchor_ordinal * manifest.anchor_replacement_stride;
  }
  return manifest.alarm_epoch * manifest.historical_replacement_multiplier +
      epoch * manifest.historical_epoch_multiplier +
      manifest.anchor_replacement_base +
      anchor_ordinal * manifest.anchor_replacement_stride;
}

std::vector<OracleOperation> deriveOracleOperations(
    const std::vector<std::uint64_t>& plausible,
    const OracleManifest& manifest) {
  std::vector<OracleOperation> out;
  std::uint64_t next_id = 1;
  if (manifest.keep_all_first) {
    OracleOperation keep;
    keep.id = next_id++;
    out.push_back(std::move(keep));
  }
  std::set<std::uint64_t> anchors;
  if (manifest.one_occurrence_per_plausible) {
    for (const auto ordinal : plausible) {
      const auto mode = oracleMode(ordinal, manifest);
      anchors.insert(mode.anchor);
      OracleOperation operation;
      operation.id = next_id++;
      operation.exclusion_cardinality = 1;
      operation.covered_modes = {ordinal};
      for (const auto epoch : oracleModeEpochs(mode, manifest)) {
        if (epoch < manifest.explicit_uwb_first) continue;
        operation.remove.push_back(oracleUwbGroup(epoch, manifest));
        operation.add.push_back(oracleReplacementGroup(epoch, mode.anchor,
                                                        manifest));
      }
      out.push_back(std::move(operation));
    }
  }
  if (manifest.union_when_single_anchor && !plausible.empty() &&
      anchors.size() == 1 &&
      anchors.size() <= manifest.max_exclusion_cardinality) {
    OracleOperation union_operation;
    union_operation.id = next_id++;
    union_operation.exclusion_cardinality = static_cast<int>(anchors.size());
    std::set<std::uint64_t> remove;
    std::set<std::uint64_t> add_seen;
    for (std::size_t i = manifest.keep_all_first ? 1 : 0; i < out.size(); ++i) {
      union_operation.covered_modes.insert(union_operation.covered_modes.end(),
          out[i].covered_modes.begin(), out[i].covered_modes.end());
      remove.insert(out[i].remove.begin(), out[i].remove.end());
      for (const auto id : out[i].add) {
        if (add_seen.insert(id).second) union_operation.add.push_back(id);
      }
    }
    union_operation.remove.assign(remove.begin(), remove.end());
    out.push_back(std::move(union_operation));
  }
  std::set<std::uint64_t> ids;
  std::set<std::string> exact_operations;
  for (const auto& operation : out) {
    std::ostringstream identity;
    for (const auto id : operation.remove) identity << "r" << id << ';';
    for (const auto id : operation.add) identity << "a" << id << ';';
    const bool exact_duplicate = !exact_operations.insert(identity.str()).second;
    if (!ids.insert(operation.id).second || operation.id > out.size() ||
        !std::is_sorted(operation.remove.begin(), operation.remove.end()) ||
        std::set<std::uint64_t>(operation.add.begin(), operation.add.end()).size()
            != operation.add.size() ||
        (exact_duplicate && manifest.exact_duplicate_rule.find("retained") ==
                                std::string::npos)) {
      throw std::runtime_error("oracle operation identity recipe is invalid");
    }
  }
  const long double total = static_cast<long double>(manifest.total_risk);
  const long double each = total / static_cast<long double>(out.size());
  const long double closed = each * static_cast<long double>(out.size());
  if (std::abs(closed - total) >
      std::numeric_limits<long double>::epsilon() * std::abs(total)) {
    throw std::runtime_error("oracle high-precision action risk does not close");
  }
  return out;
}

double certifiedTolerance(double scale, double condition, Eigen::Index dimension,
                          const OracleManifest& manifest) {
  const double multiplier = std::min(
      manifest.tolerance_cap,
      std::max(32.0, static_cast<double>(std::max<Eigen::Index>(1, dimension))) *
          std::max(1.0, condition));
  return manifest.epsilon * std::max(1.0, scale) * multiplier;
}

std::string joinedGroups(const std::vector<FactorGroupId>& groups) {
  std::ostringstream out;
  for (std::size_t i = 0; i < groups.size(); ++i) {
    if (i) out << ';';
    out << groups[i].value();
  }
  return out.str();
}

double independentNoncentralityBoundary(int dof, double threshold,
                                        double p_md) {
  if (dof <= 0 || !std::isfinite(threshold) || threshold < 0.0 ||
      !(p_md > 0.0 && p_md < 1.0)) {
    return std::numeric_limits<double>::infinity();
  }
  auto lower_tail = [&](double lambda) {
    return boost::math::cdf(
        boost::math::non_central_chi_squared_distribution<double>(
            static_cast<double>(dof), lambda), threshold);
  };
  double low = 0.0;
  double high = 1.0;
  while (lower_tail(high) > p_md && high < 1e12) high *= 2.0;
  if (high >= 1e12 && lower_tail(high) > p_md) {
    return std::numeric_limits<double>::infinity();
  }
  for (int iteration = 0; iteration < 160; ++iteration) {
    const double midpoint = 0.5 * (low + high);
    if (lower_tail(midpoint) > p_md) low = midpoint;
    else high = midpoint;
  }
  return high;
}

std::vector<OracleHypothesis> oracleHypotheses(
    const OracleManifest& manifest) {
  if (manifest.hypothesis_rule !=
          "one single-mode hypothesis per enumerated mode" ||
      !manifest.double_fault_families.empty() ||
      manifest.resolved_max_fault_order != 1) {
    throw std::runtime_error("unsupported oracle hypothesis recipe");
  }
  std::vector<OracleHypothesis> out;
  out.reserve(manifest.expected_hypotheses);
  const double allocation = manifest.total_risk /
      static_cast<double>(manifest.expected_hypotheses);
  for (std::size_t i = 0; i < manifest.expected_hypotheses; ++i) {
    const OracleModeRecipe mode = oracleMode(i + 1, manifest);
    if (mode.family.name.rfind("AnchorBias", 0) != 0) {
      throw std::runtime_error("order-one replay contains a non-UWB family");
    }
    out.push_back({i + 1, manifest.uwb_prior_probability_bound,
                   manifest.uwb_p_md, allocation});
  }
  return out;
}

std::vector<DenseActionReference> independentDenseActionReferences(
    const CompleteRawReference& raw,
    const OracleManifest& manifest,
    const std::vector<OracleHypothesis>& hypotheses,
    double rank_tolerance) {
  std::vector<DenseActionReference> out;
  std::map<std::pair<int, double>, double> noncentrality_cache;
  for (std::size_t action_index = 0;
       action_index < manifest.operations.size(); ++action_index) {
    const auto& expected = manifest.operations[action_index];
    DenseActionReference reference;
    reference.id = expected.id;
    reference.exclusion_cardinality = expected.exclusion_cardinality;
    reference.removed = joinedGroups([&] {
      std::vector<FactorGroupId> ids;
      for (const auto id : expected.remove) ids.emplace_back(id);
      return ids;
    }());
    reference.added = joinedGroups([&] {
      std::vector<FactorGroupId> ids;
      for (const auto id : expected.add) ids.emplace_back(id);
      return ids;
    }());
    std::vector<const CompleteRawReference::Block*> retained;
    std::vector<std::uint64_t> retained_groups;
    for (const auto group : raw.explicit_group_ids) {
      if (std::find(expected.remove.begin(), expected.remove.end(), group) ==
          expected.remove.end()) {
        retained.push_back(&raw.blocks.at(group));
        retained_groups.push_back(group);
      }
    }
    retained.insert(retained.begin(), &raw.blocks.at(manifest.summary_group));
    retained_groups.insert(retained_groups.begin(), manifest.summary_group);
    for (const auto group : expected.add) {
      const auto block = raw.blocks.find(group);
      if (block == raw.blocks.end()) {
        ADD_FAILURE() << "oracle replacement source is absent: " << group;
        continue;
      }
      retained.push_back(&block->second);
      retained_groups.push_back(group);
    }
    Eigen::Index rows = 0;
    for (const auto* block : retained) rows += block->raw_z.size();
    Eigen::MatrixXd h(rows, manifest.state_columns);
    Eigen::VectorXd z(rows);
    Eigen::Index offset = 0;
    for (const auto* block : retained) {
      const Eigen::Index count = block->raw_z.size();
      const Eigen::MatrixXd rebuilt_h = block->whitener * block->raw_h;
      const Eigen::VectorXd rebuilt_z = block->whitener * block->raw_z;
      EXPECT_TRUE(rebuilt_h.isApprox(block->h, 2e-10));
      EXPECT_TRUE(rebuilt_z.isApprox(block->z, 2e-10));
      h.middleRows(offset, count) = rebuilt_h;
      z.segment(offset, count) = rebuilt_z;
      offset += count;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        h, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = svd.singularValues();
    const double gate = rank_tolerance *
        (singular.size() == 0 ? 0.0 : singular(0));
    reference.rank = static_cast<int>((singular.array() > gate).count());
    reference.dof = static_cast<int>(rows) - reference.rank;
    reference.information_logdet = 0.0;
    for (Eigen::Index i = 0; i < singular.size(); ++i) {
      if (singular(i) > gate) {
        reference.information_logdet += 2.0 * std::log(singular(i));
      }
    }
    Eigen::VectorXd inverse = Eigen::VectorXd::Zero(singular.size());
    for (Eigen::Index i = 0; i < singular.size(); ++i) {
      if (singular(i) > gate) inverse(i) = 1.0 / singular(i);
    }
    const Eigen::MatrixXd pinv =
        svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose();
    const Eigen::VectorXd state = pinv * z;
    reference.statistic = (z - h * state).squaredNorm();
    if (reference.dof > 0) {
      reference.threshold = boost::math::quantile(
          boost::math::complement(
              boost::math::chi_squared_distribution<double>(reference.dof),
              manifest.detector_p_fa));
      reference.post_passed = reference.statistic <= reference.threshold;
    }
    reference.covers_plausible = std::all_of(
        manifest.plausible_modes.begin(), manifest.plausible_modes.end(),
        [&](std::uint64_t id) {
          return std::find(expected.covered_modes.begin(),
                           expected.covered_modes.end(), id) !=
              expected.covered_modes.end();
        });
    const Eigen::MatrixXd parity_projector =
        Eigen::MatrixXd::Identity(rows, rows) - h * pinv;
    const Eigen::VectorXd parity_residual = parity_projector * z;
    const Eigen::MatrixXd candidate_covariance =
        svd.matrixV() * inverse.array().square().matrix().asDiagonal() *
        svd.matrixV().transpose();
    const Eigen::Matrix3d protected_covariance =
        raw.protected_state_map * candidate_covariance *
        raw.protected_state_map.transpose();
    const double nominal_multiplier = boost::math::quantile(
        boost::math::normal_distribution<double>(),
        1.0 - manifest.nominal_axis_tail / 2.0);
    const Eigen::Vector3d nominal_component = nominal_multiplier *
        protected_covariance.diagonal().cwiseMax(0.0).cwiseSqrt();
    const int history_channel_dof = static_cast<int>(
        raw.history_detector_response.rows());
    const int current_channel_dof = static_cast<int>(rows) -
        history_channel_dof - reference.rank;
    const auto channel_threshold = [&](int dof) {
      return dof > 0 ? boost::math::quantile(
          boost::math::complement(
              boost::math::chi_squared_distribution<double>(dof),
              manifest.detector_p_fa))
          : std::numeric_limits<double>::infinity();
    };
    const double current_channel_threshold =
        channel_threshold(current_channel_dof);
    const double history_channel_threshold =
        channel_threshold(history_channel_dof);
    Eigen::Vector3d worst_fault = Eigen::Vector3d::Zero();
    bool remaining_math_valid = true;
    for (const auto mode_id : manifest.plausible_modes) {
      const auto full_fault = raw.fault_modes.find(mode_id);
      if (full_fault == raw.fault_modes.end()) {
        ADD_FAILURE() << "independent candidate profile mode absent: "
                      << mode_id;
        continue;
      }
      Eigen::MatrixXd fault = Eigen::MatrixXd::Zero(
          rows, full_fault->second.cols());
      Eigen::Index candidate_offset = 0;
      for (std::size_t block_index = 0; block_index < retained.size();
           ++block_index) {
        const auto group = retained_groups[block_index];
        const Eigen::Index count = retained[block_index]->z.size();
        const auto source_offset = raw.group_row_offsets.find(group);
        if (source_offset != raw.group_row_offsets.end()) {
          fault.middleRows(candidate_offset, count) =
              full_fault->second.middleRows(source_offset->second, count);
        }
        candidate_offset += count;
      }
      const Eigen::MatrixXd residual_fault = parity_projector * fault;
      const Eigen::MatrixXd gram =
          residual_fault.transpose() * residual_fault;
      const Eigen::MatrixXd response =
          raw.protected_state_map * pinv * fault;
      const Eigen::VectorXd score = fault.transpose() * parity_residual;
      const Eigen::MatrixXd gram_inverse = svdPseudoInverse(gram);
      const Eigen::VectorXd estimate = gram_inverse * score;
      const double profile = std::max(
          0.0, parity_residual.squaredNorm() - score.dot(estimate));
      EXPECT_TRUE(gram.allFinite());
      EXPECT_TRUE(estimate.allFinite());
      EXPECT_TRUE(std::isfinite(profile));
      const auto hypothesis = std::find_if(
          hypotheses.begin(), hypotheses.end(), [&](const auto& value) {
            return value.id == mode_id;
          });
      EXPECT_NE(hypothesis, hypotheses.end());
      Eigen::JacobiSVD<Eigen::MatrixXd> total_factor_svd(
          residual_fault, Eigen::ComputeFullV);
      const double total_singular_gate = rank_tolerance * fault.norm();
      const int total_rank = static_cast<int>(
          (total_factor_svd.singularValues().array() >
           total_singular_gate).count());
      const int total_nullity = static_cast<int>(gram.cols()) - total_rank;
      int total_class = total_rank == static_cast<int>(gram.cols())
          ? static_cast<int>(GramNullspaceClass::FullRank)
          : static_cast<int>(GramNullspaceClass::Harmless);
      if (total_nullity > 0) {
        const Eigen::MatrixXd hidden = response *
            total_factor_svd.matrixV().rightCols(total_nullity);
        if (hidden.cwiseAbs().maxCoeff() != 0.0) {
          total_class = static_cast<int>(GramNullspaceClass::Dangerous);
        }
      }
      Eigen::Vector3d total_slopes = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::infinity());
      if (total_class != static_cast<int>(GramNullspaceClass::Dangerous)) {
        for (int axis = 0; axis < 3; ++axis) {
          total_slopes(axis) = std::sqrt(std::max(
              0.0, (response.row(axis) * gram_inverse *
                    response.row(axis).transpose())(0, 0)));
        }
      }
      const Eigen::Index history_fault_begin =
          raw.history_response.rows();
      const Eigen::MatrixXd history_gram =
          fault.middleRows(history_fault_begin, history_channel_dof).transpose() *
          fault.middleRows(history_fault_begin, history_channel_dof);
      const Eigen::MatrixXd current_gram = 0.5 *
          ((gram - history_gram) + (gram - history_gram).transpose());
      Eigen::MatrixXd weighted_gram = Eigen::MatrixXd::Zero(
          gram.rows(), gram.cols());
      std::size_t usable_channels = 0;
      for (const auto& channel :
           std::array<std::tuple<Eigen::MatrixXd, int, double>, 2>{{
               {current_gram, current_channel_dof, current_channel_threshold},
               {history_gram, history_channel_dof, history_channel_threshold}}}) {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> channel_eigen(
            std::get<0>(channel));
        EXPECT_EQ(channel_eigen.info(), Eigen::Success);
        const double scale = channel_eigen.eigenvalues().size() == 0
            ? 0.0 : channel_eigen.eigenvalues().cwiseAbs().maxCoeff();
        const int channel_rank = static_cast<int>(
            (channel_eigen.eigenvalues().array() >
             rank_tolerance * scale).count());
        if (channel_rank > 0 && std::get<1>(channel) > 0) ++usable_channels;
      }
      if (usable_channels > 0 && hypothesis != hypotheses.end()) {
        for (const auto& channel :
             std::array<std::tuple<Eigen::MatrixXd, int, double>, 2>{{
                 {current_gram, current_channel_dof,
                  current_channel_threshold},
                 {history_gram, history_channel_dof,
                  history_channel_threshold}}}) {
          Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> channel_eigen(
              std::get<0>(channel));
          const double scale = channel_eigen.eigenvalues().size() == 0
              ? 0.0 : channel_eigen.eigenvalues().cwiseAbs().maxCoeff();
          const int channel_rank = static_cast<int>(
              (channel_eigen.eigenvalues().array() >
               rank_tolerance * scale).count());
          if (channel_rank == 0 || std::get<1>(channel) <= 0) continue;
          const auto key = std::make_pair(std::get<1>(channel),
                                          hypothesis->p_md_allocation);
          const auto inserted = noncentrality_cache.emplace(
              key, std::numeric_limits<double>::quiet_NaN());
          if (inserted.second) {
            inserted.first->second = independentNoncentralityBoundary(
                std::get<1>(channel), std::get<2>(channel),
                hypothesis->p_md_allocation);
          }
          weighted_gram += (1.0 / static_cast<double>(usable_channels)) /
              inserted.first->second * std::get<0>(channel);
        }
      }
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> gram_eigen(
          0.5 * (weighted_gram + weighted_gram.transpose()));
      EXPECT_EQ(gram_eigen.info(), Eigen::Success);
      const double gram_scale = gram_eigen.eigenvalues().size() == 0
          ? 0.0 : gram_eigen.eigenvalues().cwiseAbs().maxCoeff();
      const double gram_gate = rank_tolerance * gram_scale;
      const int gram_rank = static_cast<int>(
          (gram_eigen.eigenvalues().array() > gram_gate).count());
      const int nullity = static_cast<int>(weighted_gram.cols()) - gram_rank;
      int nullspace_class = gram_rank == 0
          ? static_cast<int>(GramNullspaceClass::Dangerous)
          : static_cast<int>(GramNullspaceClass::FullRank);
      if (gram_rank > 0 && nullity > 0) {
        const Eigen::MatrixXd hidden = response *
            gram_eigen.eigenvectors().leftCols(nullity);
        nullspace_class = hidden.cwiseAbs().maxCoeff() <=
            certifiedTolerance(response.norm(), 1.0, response.size(), manifest)
            ? static_cast<int>(GramNullspaceClass::Harmless)
            : static_cast<int>(GramNullspaceClass::Dangerous);
      }
      Eigen::Vector3d slopes = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::infinity());
      if (nullspace_class != static_cast<int>(GramNullspaceClass::Dangerous)) {
        const Eigen::MatrixXd weighted_inverse =
            svdPseudoInverse(weighted_gram);
        for (int axis = 0; axis < 3; ++axis) {
          slopes(axis) = std::sqrt(std::max(
              0.0, (response.row(axis) * weighted_inverse *
                    response.row(axis).transpose())(0, 0)));
        }
      }
      reference.candidate_grams.push_back(gram);
      reference.candidate_fault_maps.push_back(fault);
      reference.protected_responses.push_back(response);
      reference.gram_protected_slopes.push_back(total_slopes);
      reference.gram_nullspace_classes.push_back(total_class);
      reference.protected_slopes.push_back(slopes);
      reference.nullspace_classes.push_back(nullspace_class);
      const bool covered = std::find(expected.covered_modes.begin(),
                                     expected.covered_modes.end(), mode_id) !=
          expected.covered_modes.end();
      if (!covered && hypothesis != hypotheses.end() && slopes.allFinite()) {
        const double tail = std::min(
            0.5, hypothesis->hmi_allocation /
                     std::max(hypothesis->prior_probability_bound, 1e-15));
        const double k = boost::math::quantile(
            boost::math::normal_distribution<double>(),
            1.0 - (tail / 3.0) / 2.0);
        worst_fault = worst_fault.cwiseMax(
            slopes + k *
                protected_covariance.diagonal().cwiseMax(0.0).cwiseSqrt());
      } else if (!covered) {
        remaining_math_valid = false;
      }
      ++reference.candidate_profile_count;
    }
    EXPECT_EQ(reference.candidate_profile_count,
              manifest.plausible_modes.size());
    reference.mathematical_pl_xyz = nominal_component.cwiseMax(worst_fault);
    reference.mathematical_hpl_m = std::hypot(
        reference.mathematical_pl_xyz.x(),
        reference.mathematical_pl_xyz.y());
    reference.mathematical_vpl_m = reference.mathematical_pl_xyz.z();
    reference.mathematical_pl_finite =
        reference.mathematical_pl_xyz.allFinite() &&
        std::isfinite(reference.mathematical_hpl_m) &&
        std::isfinite(reference.mathematical_vpl_m) && remaining_math_valid;
    long double hypothesis_charge = 0.0L;
    long double miss_charge = 0.0L;
    for (const auto& hypothesis : hypotheses) {
      hypothesis_charge += static_cast<long double>(hypothesis.hmi_allocation);
      const long double prior = hypothesis.prior_probability_bound;
      const long double alpha = prior > 0.0L
          ? std::min(0.5L, static_cast<long double>(hypothesis.hmi_allocation) /
                               prior) : 0.0L;
      const long double beta = hypothesis.p_md_allocation;
      if (beta > alpha) miss_charge += prior * (beta - alpha);
    }
    reference.risk_terms = {
        "nominal=" + std::to_string(3.0 * manifest.nominal_axis_tail),
        "p_nm=" + std::to_string(manifest.p_nm),
        "hypotheses=" + std::to_string(static_cast<double>(hypothesis_charge)),
        "hypotheses_miss_channel=" +
            std::to_string(static_cast<double>(miss_charge)),
        "bridge=UNKNOWN", "history=UNKNOWN", "model=UNKNOWN_N01",
        "selection=" + std::to_string(manifest.total_risk /
                                        manifest.operations.size())};
    reference.risk_charged_total = std::numeric_limits<double>::quiet_NaN();
    reference.risk_valid = false;
    reference.risk_all_terms_validated = false;
    // N01 is applied only after the complete mathematical PL has been formed.
    // It invalidates the formal risk ledger; it does not substitute infinity
    // for the independently calculated candidate Gram/response/slope/base PL.
    reference.model_error_validated = true;
    reference.pl_valid = false;
    reference.hpl_m = std::numeric_limits<double>::infinity();
    reference.vpl_m = std::numeric_limits<double>::infinity();
    reference.eligible = reference.post_passed && reference.covers_plausible &&
        reference.model_error_validated && reference.pl_valid;
    reference.terminal = reference.post_passed ? "PL_COMPLETE" : "POST_COMPLETE";
    reference.risk_allocation = manifest.total_risk /
        static_cast<double>(manifest.operations.size());
    std::ostringstream selection_tuple;
    selection_tuple << reference.exclusion_cardinality << '|'
                    << reference.mathematical_hpl_m << '|'
                    << reference.mathematical_vpl_m << '|'
                    << reference.id;
    reference.selection_tuple = selection_tuple.str();
    out.push_back(std::move(reference));
  }
  std::vector<DenseActionReference*> eligible;
  for (auto& reference : out) if (reference.eligible) eligible.push_back(&reference);
  std::sort(eligible.begin(), eligible.end(), [](const auto* left,
                                                 const auto* right) {
    const double left_max_pl = std::max(left->mathematical_hpl_m,
                                        left->mathematical_vpl_m);
    const double right_max_pl = std::max(right->mathematical_hpl_m,
                                         right->mathematical_vpl_m);
    return std::make_tuple(left->exclusion_cardinality, left_max_pl,
                           -left->information_logdet, left->id) <
           std::make_tuple(right->exclusion_cardinality, right_max_pl,
                           -right->information_logdet, right->id);
  });
  if (!eligible.empty()) eligible.front()->selected = true;
  return out;
}

// Same-replay reference. It starts from frozen nonlinear factor objects/raw
// covariance and the independently enumerated raw replay ledger. It never
// consumes production window H/z when constructing expected values. History
// is eliminated in information form, so comparisons are invariant to the
// summary builder's legal square-root row rotation.
CompleteRawReference completeRawReference(
    const EpochTransaction& tx, const LinearizedIntegrityWindow& window,
    std::size_t window_epochs, const OracleManifest& manifest,
    const std::vector<RawRowOwnershipAuditV1>& actual_raw_owners) {
  struct RowBlock {
    std::vector<gtsam::Key> keys;
    Eigen::MatrixXd state;
    Eigen::MatrixXd fault;
    Eigen::VectorXd rhs;
  };
  struct FrozenUwbRow {
    std::uint64_t measurement_id = 0;
    std::uint64_t anchor_id = 0;
    std::int64_t timestamp_ns = 0;
  };
  const YAML::Node frozen_replay = YAML::LoadFile(
      std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/docs/evidence/p0-07-corrected-exhaustive/authoritative-replay/frozen-input.json");
  if (frozen_replay["schema"].as<std::string>() !=
      "p0-07-production-raw-replay-v3") {
    throw std::runtime_error("unsupported frozen raw replay schema");
  }
  const double frozen_diagonal =
      frozen_replay["covariance"]["diagonal_m2"].as<double>();
  const double frozen_off_diagonal =
      frozen_replay["covariance"]["off_diagonal_m2"].as<double>();
  std::map<std::size_t, std::vector<FrozenUwbRow>> frozen_uwb_rows;
  for (const auto& node : frozen_replay["uwb_rows"]) {
    frozen_uwb_rows[node["epoch"].as<std::size_t>()].push_back({
        node["measurement_id"].as<std::uint64_t>(),
        node["anchor_id"].as<std::uint64_t>(),
        node["timestamp_ns"].as<std::int64_t>()});
  }
  if (frozen_uwb_rows.size() != frozen_replay["epochs"].as<std::size_t>()) {
    throw std::runtime_error("frozen UWB epoch inventory does not close");
  }
  const auto historicalRecord = [&](std::size_t expected_epoch)
      -> const HistoricalEpochContext* {
    const auto found = std::find_if(
        tx.recoverable_history.begin(), tx.recoverable_history.end(),
        [&](const HistoricalEpochContext& record) {
          return record.proposed_epoch == expected_epoch;
        });
    return found == tx.recoverable_history.end() ? nullptr : &*found;
  };
  const auto historicalGroup = [&](std::size_t expected_epoch,
                                   std::uint64_t expected_group)
      -> const PendingFactorGroup* {
    const auto* record = historicalRecord(expected_epoch);
    if (!record) return nullptr;
    const auto found = std::find_if(
        record->groups.begin(), record->groups.end(),
        [&](const PendingFactorGroup& group) {
          return group.id.value() == expected_group;
        });
    return found == record->groups.end() ? nullptr : &*found;
  };
  const auto actualGroup = [&](std::size_t expected_epoch,
                               std::uint64_t expected_group)
      -> const PendingFactorGroup* {
    if (expected_epoch == tx.proposed_epoch) {
      if (tx.imu_group.id.value() == expected_group) return &tx.imu_group;
      const auto found = std::find_if(
          tx.uwb_groups.begin(), tx.uwb_groups.end(),
          [&](const PendingFactorGroup& group) {
            return group.id.value() == expected_group;
          });
      return found == tx.uwb_groups.end() ? nullptr : &*found;
    }
    return historicalGroup(expected_epoch, expected_group);
  };

  struct ExpectedRawRowOwner {
    std::string row_id;
    std::uint64_t group_id = 0;
    std::size_t row_in_group = 0;
    std::string covariance_placement;
  };
  std::vector<ExpectedRawRowOwner> expected_raw_owners;
  for (std::size_t row = 0; row < manifest.marginal_rows; ++row) {
    expected_raw_owners.push_back({manifest.prior_row_prefix + ":" +
        std::to_string(manifest.marginal_group) + ":" + std::to_string(row),
        manifest.marginal_group, row,
        "information-prior:" + std::to_string(manifest.marginal_slot) + ":" +
            std::to_string(row)});
  }
  for (std::size_t epoch = manifest.history_uwb_first;
       epoch <= manifest.explicit_uwb_last; ++epoch) {
    const auto frozen_rows = frozen_uwb_rows.find(epoch);
    EXPECT_NE(frozen_rows, frozen_uwb_rows.end()) << epoch;
    if (frozen_rows == frozen_uwb_rows.end()) continue;
    const std::uint64_t group_id = oracleUwbGroup(epoch, manifest);
    for (std::size_t row = 0; row < frozen_rows->second.size(); ++row) {
      const auto& expected = frozen_rows->second[row];
      expected_raw_owners.push_back({manifest.uwb_row_prefix + ":" +
          std::to_string(expected.measurement_id),
          group_id, row, "uwb-covariance-row-column:" + std::to_string(row)});
    }
    const PendingFactorGroup* actual_group = actualGroup(epoch, group_id);
    if (!actual_group) {
      const auto slot = std::find_if(
          tx.frozen_slots.begin(), tx.frozen_slots.end(),
          [&](const FrozenFactorSlot& value) {
            return value.group_id && value.group_id->value() == group_id;
          });
      EXPECT_NE(slot, tx.frozen_slots.end()) << group_id;
      if (slot != tx.frozen_slots.end()) {
        const auto linear = slot->factor->linearize(*tx.frozen_values);
        const auto jacobian =
            boost::dynamic_pointer_cast<gtsam::JacobianFactor>(linear);
        EXPECT_TRUE(static_cast<bool>(jacobian));
        if (jacobian) {
          EXPECT_EQ(jacobian->rows(),
                    static_cast<Eigen::Index>(frozen_rows->second.size()));
        }
      }
      continue;
    }
    EXPECT_EQ(actual_group->kind, FactorKind::UwbBatch);
    EXPECT_EQ(actual_group->sensor, SensorType::Uwb);
    EXPECT_EQ(actual_group->source_measurements.size(), frozen_rows->second.size());
    EXPECT_EQ(actual_group->raw_covariance.rows(),
              static_cast<Eigen::Index>(frozen_rows->second.size()));
    if (actual_group->source_measurements.size() != frozen_rows->second.size() ||
        actual_group->raw_covariance.rows() !=
            static_cast<Eigen::Index>(frozen_rows->second.size()) ||
        actual_group->raw_covariance.cols() !=
            static_cast<Eigen::Index>(frozen_rows->second.size())) {
      continue;
    }
    for (std::size_t row = 0; row < frozen_rows->second.size(); ++row) {
      const auto& expected = frozen_rows->second[row];
      EXPECT_EQ(actual_group->source_measurements[row].value(),
                expected.measurement_id);
      for (std::size_t column = 0; column < frozen_rows->second.size(); ++column) {
        const double expected_covariance = row == column
            ? frozen_diagonal : frozen_off_diagonal;
        EXPECT_DOUBLE_EQ(actual_group->raw_covariance(
                             static_cast<Eigen::Index>(row),
                             static_cast<Eigen::Index>(column)),
                         expected_covariance);
      }
    }
  }
  for (std::size_t epoch = manifest.history_imu_first;
       epoch <= manifest.explicit_imu_last; ++epoch) {
    const std::uint64_t group_id = oracleImuGroup(epoch, manifest);
    const PendingFactorGroup* actual_group = actualGroup(epoch, group_id);
    EXPECT_NE(actual_group, nullptr) << group_id;
    if (!actual_group) continue;
    EXPECT_EQ(actual_group->kind, FactorKind::CombinedImu);
    EXPECT_EQ(actual_group->sensor, SensorType::Imu);
    const auto gaussian = actual_group->factors.linearize(*tx.frozen_values);
    gtsam::Ordering ordering;
    for (const auto key : actual_group->keys) {
      if (std::find(ordering.begin(), ordering.end(), key) == ordering.end()) {
        ordering.push_back(key);
      }
    }
    const auto actual_dense = gaussian->jacobian(ordering);
    EXPECT_EQ(actual_dense.first.rows(), static_cast<Eigen::Index>(manifest.imu_rows));
    for (std::size_t row = 0; row < manifest.imu_rows; ++row) {
      expected_raw_owners.push_back({manifest.imu_row_prefix + ":" +
          std::to_string(group_id) + ":" + std::to_string(row), group_id, row,
          "factor-native-whitened-row:" + std::to_string(row)});
    }
  }
  const std::size_t expected_owner_rows = manifest.marginal_rows +
      (manifest.explicit_uwb_last - manifest.history_uwb_first + 1) *
          manifest.uwb_rows +
      (manifest.explicit_imu_last - manifest.history_imu_first + 1) *
          manifest.imu_rows;
  EXPECT_EQ(expected_raw_owners.size(), expected_owner_rows);
  std::set<std::string> expected_row_ids;
  for (const auto& row : expected_raw_owners) {
    EXPECT_FALSE(row.covariance_placement.empty());
    EXPECT_TRUE(expected_row_ids.insert(row.row_id).second) << row.row_id;
  }
  std::sort(expected_raw_owners.begin(), expected_raw_owners.end(),
            [](const auto& left, const auto& right) {
              return left.row_id < right.row_id;
            });
  EXPECT_EQ(actual_raw_owners.size(), expected_raw_owners.size());
  const std::size_t comparable_owner_rows = std::min(
      actual_raw_owners.size(), expected_raw_owners.size());
  for (std::size_t row = 0; row < comparable_owner_rows; ++row) {
    EXPECT_EQ(actual_raw_owners[row].row_id,
              expected_raw_owners[row].row_id) << row;
    EXPECT_EQ(actual_raw_owners[row].owner_group.value(),
              expected_raw_owners[row].group_id) << row;
    EXPECT_EQ(actual_raw_owners[row].row_in_group,
              expected_raw_owners[row].row_in_group) << row;
    EXPECT_EQ(actual_raw_owners[row].covariance_placement,
              expected_raw_owners[row].covariance_placement) << row;
  }
  EXPECT_EQ(tx.proposed_epoch - window_epochs, manifest.window_first_epoch);
  const std::size_t window_first = manifest.window_first_epoch;
  std::map<HistoryFaultColumnId, int> fault_index;
  std::vector<HistoryFaultColumnId> expected_fault_ids;
  for (std::size_t epoch = manifest.physical_first_epoch;
       epoch < window_first; ++epoch) {
    EXPECT_NE(historicalRecord(epoch), nullptr) << epoch;
    for (const auto anchor : manifest.anchors) {
      std::vector<HistoryFaultBasisKind> basis;
      for (const auto& family : manifest.families) {
        if (family.kind == static_cast<int>(
                               FaultKind::AnchorBiasPersistentConstant) ||
            family.kind == static_cast<int>(FaultKind::AnchorBiasRamp)) {
          basis.push_back(HistoryFaultBasisKind::UwbAnchorConstant);
        }
        if (family.kind == static_cast<int>(FaultKind::AnchorBiasRamp)) {
          basis.push_back(HistoryFaultBasisKind::UwbAnchorTimeLinear);
        }
      }
      std::sort(basis.begin(), basis.end());
      basis.erase(std::unique(basis.begin(), basis.end()), basis.end());
      for (const auto kind : basis) {
        HistoryFaultColumnId id{kind, anchor, epoch};
        fault_index[id] = static_cast<int>(expected_fault_ids.size());
        expected_fault_ids.push_back(id);
      }
    }
  }
  EXPECT_EQ(expected_fault_ids, window.history_summary.column_ids);

  std::vector<RowBlock> history_blocks;
  std::set<gtsam::Key> seen_history_keys;
  std::vector<gtsam::Key> all_history_keys;
  double history_offset = 0.0;
  std::vector<std::pair<std::size_t, std::uint64_t>> boundary_slots;
  for (const auto& phase : manifest.history_factor_order) {
    if (phase == "imu_and_uwb:uwb") {
      for (std::size_t epoch = manifest.history_imu_first;
           epoch <= manifest.history_uwb_last; ++epoch) {
        boundary_slots.push_back({oracleUwbSlot(epoch, manifest),
                                  oracleUwbGroup(epoch, manifest)});
      }
    } else if (phase == "uwb_only:uwb") {
      for (std::size_t epoch = manifest.history_uwb_first;
           epoch < manifest.history_imu_first; ++epoch) {
        boundary_slots.push_back({oracleUwbSlot(epoch, manifest),
                                  oracleUwbGroup(epoch, manifest)});
      }
    } else if (phase == "imu_and_uwb:imu") {
      for (std::size_t epoch = manifest.history_imu_first;
           epoch <= manifest.history_uwb_last; ++epoch) {
        boundary_slots.push_back({oracleImuSlot(epoch, manifest),
                                  oracleImuGroup(epoch, manifest)});
      }
    } else if (phase == "imu_only:imu") {
      for (std::size_t epoch = manifest.history_uwb_last + 1;
           epoch <= manifest.history_imu_last; ++epoch) {
        boundary_slots.push_back({oracleImuSlot(epoch, manifest),
                                  oracleImuGroup(epoch, manifest)});
      }
    } else if (phase == "marginal_prior") {
      boundary_slots.push_back({manifest.marginal_slot, manifest.marginal_group});
    } else {
      throw std::runtime_error("unknown recipe history factor-order phase");
    }
  }
  for (const auto& expected_slot : boundary_slots) {
    const auto frozen = std::find_if(
        tx.frozen_slots.begin(), tx.frozen_slots.end(),
        [&](const FrozenFactorSlot& slot) { return slot.slot == expected_slot.first; });
    EXPECT_NE(frozen, tx.frozen_slots.end());
    if (frozen == tx.frozen_slots.end()) continue;
    EXPECT_TRUE(frozen->group_id.has_value());
    if (!frozen->group_id) continue;
    EXPECT_EQ(frozen->group_id->value(), expected_slot.second);
    const auto linear = frozen->factor->linearize(*tx.frozen_values);
    RowBlock block;
    if (const auto jacobian =
            boost::dynamic_pointer_cast<gtsam::JacobianFactor>(linear)) {
      block.keys.assign(jacobian->keys().begin(), jacobian->keys().end());
      block.state = jacobian->getA();
      block.rhs = jacobian->getb();
    } else if (const auto hessian =
                   boost::dynamic_pointer_cast<gtsam::HessianFactor>(linear)) {
      block.keys.assign(hessian->keys().begin(), hessian->keys().end());
      int dimension = 0;
      for (const auto key : block.keys) dimension += keyWidth(key);
      const Eigen::MatrixXd augmented = hessian->info().selfadjointView();
      Eigen::LLT<Eigen::MatrixXd> llt(
          augmented.topLeftCorner(dimension, dimension));
      EXPECT_EQ(llt.info(), Eigen::Success);
      block.state = llt.matrixU();
      block.rhs = llt.matrixU().transpose().solve(
          augmented.topRightCorner(dimension, 1));
      history_offset += augmented(dimension, dimension) - block.rhs.squaredNorm();
    } else {
      ADD_FAILURE() << "independent raw oracle encountered unknown factor";
      continue;
    }
    block.fault = Eigen::MatrixXd::Zero(block.state.rows(),
                                        expected_fault_ids.size());
    const Eigen::Index expected_block_rows =
        expected_slot.second == manifest.marginal_group
            ? static_cast<Eigen::Index>(manifest.marginal_rows)
            : (expected_slot.second % manifest.group_epoch_multiplier ==
                       manifest.group_uwb_add
                   ? static_cast<Eigen::Index>(manifest.uwb_rows)
                   : static_cast<Eigen::Index>(manifest.imu_rows));
    EXPECT_EQ(block.state.rows(), expected_block_rows)
        << "row-owner group=" << expected_slot.second;
    const std::size_t group_epoch = static_cast<std::size_t>(
        expected_slot.second / manifest.group_epoch_multiplier);
    const auto* historical_group = historicalGroup(
        group_epoch, expected_slot.second);
    if (historical_group && group_epoch < window_first &&
        expected_slot.second == oracleUwbGroup(group_epoch, manifest)) {
      const auto& group = *historical_group;
      const std::size_t epoch = group_epoch;
      const auto expected_rows = frozen_uwb_rows.find(epoch);
      if (expected_rows == frozen_uwb_rows.end()) {
        throw std::runtime_error("recipe UWB epoch absent from frozen replay");
      }
      Eigen::MatrixXd expected_covariance = Eigen::MatrixXd::Constant(
          expected_rows->second.size(), expected_rows->second.size(),
          frozen_off_diagonal);
      expected_covariance.diagonal().setConstant(frozen_diagonal);
      EXPECT_TRUE(group.raw_covariance.isApprox(expected_covariance, 1e-15));
      EXPECT_EQ(group.source_measurements.size(), expected_rows->second.size());
      Eigen::LLT<Eigen::MatrixXd> covariance_llt(expected_covariance);
      EXPECT_EQ(covariance_llt.info(), Eigen::Success);
      const Eigen::MatrixXd information = covariance_llt.solve(
          Eigen::MatrixXd::Identity(expected_covariance.rows(),
                                    expected_covariance.cols()));
      Eigen::LLT<Eigen::MatrixXd> information_llt(information);
      EXPECT_EQ(information_llt.info(), Eigen::Success);
      const Eigen::MatrixXd upper = information_llt.matrixU();
      for (std::size_t row = 0; row < expected_rows->second.size(); ++row) {
        const auto& measurement = expected_rows->second[row];
        EXPECT_EQ(group.source_measurements[row].value(),
                  measurement.measurement_id);
        Eigen::VectorXd constant = Eigen::VectorXd::Zero(block.state.rows());
        Eigen::VectorXd ramp = Eigen::VectorXd::Zero(block.state.rows());
        constant(static_cast<Eigen::Index>(row)) = 1.0;
        ramp(static_cast<Eigen::Index>(row)) =
            static_cast<double>(measurement.timestamp_ns -
                                historicalRecord(epoch)->begin.value()) * 1e-9;
        block.fault.col(fault_index.at({
            HistoryFaultBasisKind::UwbAnchorConstant,
            measurement.anchor_id, epoch})) +=
            upper * constant;
        block.fault.col(fault_index.at({
            HistoryFaultBasisKind::UwbAnchorTimeLinear,
            measurement.anchor_id, epoch})) +=
            upper * ramp;
      }
    }
    for (const auto key : block.keys) {
      if (seen_history_keys.insert(key).second) all_history_keys.push_back(key);
    }
    history_blocks.push_back(std::move(block));
  }

  std::vector<gtsam::Key> old_keys;
  for (const auto key : all_history_keys) {
    const std::size_t epoch = gtsam::Symbol(key).index();
    if (epoch < manifest.detector_first_epoch || epoch > tx.previous_epoch) {
      old_keys.push_back(key);
    }
  }
  std::map<gtsam::Key, int> old_offset;
  int old_width = 0;
  for (const auto key : old_keys) {
    old_offset[key] = old_width;
    old_width += keyWidth(key);
  }
  const int columns = static_cast<int>(manifest.state_columns);
  int history_rows = 0;
  for (const auto& block : history_blocks) history_rows += block.state.rows();
  Eigen::MatrixXd old_h = Eigen::MatrixXd::Zero(history_rows, old_width);
  Eigen::MatrixXd boundary_h = Eigen::MatrixXd::Zero(history_rows, columns);
  Eigen::MatrixXd raw_fault = Eigen::MatrixXd::Zero(
      history_rows, expected_fault_ids.size());
  Eigen::VectorXd history_z = Eigen::VectorXd::Zero(history_rows);
  int row_offset = 0;
  for (const auto& block : history_blocks) {
    int local_column = 0;
    for (const auto key : block.keys) {
      const int width = keyWidth(key);
      const auto old = old_offset.find(key);
      if (old != old_offset.end()) {
        old_h.block(row_offset, old->second, block.state.rows(), width) =
            block.state.middleCols(local_column, width);
      } else {
        const std::size_t epoch = gtsam::Symbol(key).index();
        const int type = gtsam::Symbol(key).chr() == 'x' ? 0 :
            (gtsam::Symbol(key).chr() == 'v' ? 6 : 9);
        const int target = static_cast<int>(
            (epoch - manifest.detector_first_epoch) *
            manifest.state_columns_per_epoch) + type;
        boundary_h.block(row_offset, target, block.state.rows(), width) =
            block.state.middleCols(local_column, width);
      }
      local_column += width;
    }
    raw_fault.middleRows(row_offset, block.state.rows()) = block.fault;
    history_z.segment(row_offset, block.state.rows()) = block.rhs;
    row_offset += block.state.rows();
  }
  EXPECT_EQ(static_cast<std::size_t>(history_rows),
            window.history_summary.boundary_rows);
  EXPECT_EQ(static_cast<std::size_t>(history_rows), manifest.history_rows);
  const Eigen::MatrixXd eliminate_old =
      Eigen::MatrixXd::Identity(history_rows, history_rows) -
      orthogonalProjector(old_h);
  Eigen::JacobiSVD<Eigen::MatrixXd> old_svd(old_h);
  const double old_gate = old_svd.singularValues().size() == 0
      ? 0.0 : old_svd.singularValues()(0) * 1e-12;
  const int old_rank = static_cast<int>(
      (old_svd.singularValues().array() > old_gate).count());
  Eigen::MatrixXd joined(history_rows, old_h.cols() + boundary_h.cols());
  joined << old_h, boundary_h;
  const Eigen::MatrixXd eliminate_all =
      Eigen::MatrixXd::Identity(history_rows, history_rows) -
      orthogonalProjector(joined);
  const double carrier_rank_tolerance = window.numerics
      ? window.numerics->numerical_contract.rank_tolerance : 1e-10;
  const OracleHistorySummary oracle_summary = oracleHistorySummary(
      old_h, boundary_h.leftCols(15), raw_fault, history_z,
      carrier_rank_tolerance);

  const auto summary_block = std::find_if(
      window.blocks.begin(), window.blocks.end(), [](const auto& block) {
        return block.whitening_model_id == "history_summary_sqrt_d1";
      });
  EXPECT_NE(summary_block, window.blocks.end());
  {
    std::vector<Eigen::Index> supported_rows;
    std::vector<Eigen::Index> perpendicular_rows;
    for (Eigen::Index row = 0; row < oracle_summary.r.rows(); ++row) {
      if (oracle_summary.r.row(row).cwiseAbs().maxCoeff() > 0.0 ||
          std::abs(oracle_summary.d(row)) > 0.0 ||
          oracle_summary.t.row(row).cwiseAbs().maxCoeff() > 0.0) {
        supported_rows.push_back(row);
      }
    }
    for (Eigen::Index row = 0; row < oracle_summary.f.rows(); ++row)
      perpendicular_rows.push_back(row);
    EXPECT_EQ(supported_rows.size() + perpendicular_rows.size(),
              window.history_summary.emitted_rows);
    const Eigen::Index corrected_summary_rows = static_cast<Eigen::Index>(
        supported_rows.size() + perpendicular_rows.size());
    Eigen::MatrixXd expected_h = Eigen::MatrixXd::Zero(
        corrected_summary_rows, boundary_h.cols());
    Eigen::VectorXd expected_z = Eigen::VectorXd::Zero(corrected_summary_rows);
    Eigen::MatrixXd expected_t(supported_rows.size(), raw_fault.cols());
    Eigen::MatrixXd expected_f(perpendicular_rows.size(), raw_fault.cols());
    for (std::size_t row = 0; row < supported_rows.size(); ++row) {
      expected_h.row(row).leftCols(15) =
          oracle_summary.r.row(supported_rows[row]);
      expected_z(row) = oracle_summary.d(supported_rows[row]);
      expected_t.row(row) = oracle_summary.t.row(supported_rows[row]);
    }
    for (std::size_t row = 0; row < perpendicular_rows.size(); ++row) {
      expected_z(supported_rows.size() + row) =
          oracle_summary.d_perp(perpendicular_rows[row]);
      expected_f.row(row) = oracle_summary.f.row(perpendicular_rows[row]);
    }
    if (summary_block != window.blocks.end()) {
      // The hierarchical and monolithic roots may choose different legal
      // orthogonal row bases.  Compare the complete condensed quadratic form,
      // never individual coordinates.
      Eigen::MatrixXd expected_fault(expected_h.rows(), raw_fault.cols());
      expected_fault.topRows(expected_t.rows()) = expected_t;
      expected_fault.bottomRows(expected_f.rows()) = expected_f;
      Eigen::MatrixXd represented_fault(summary_block->jacobian_whitened.rows(),
                                        raw_fault.cols());
      represented_fault.topRows(window.history_summary.response.rows()) =
          window.history_summary.response;
      represented_fault.bottomRows(
          window.history_summary.detector_response.rows()) =
          window.history_summary.detector_response;
      EXPECT_LT(relativeError(summary_block->jacobian_whitened.transpose() *
                                  represented_fault,
                              boundary_h.transpose() * eliminate_old * raw_fault),
                1e-8);
      EXPECT_LT(relativeError(represented_fault.transpose() * represented_fault,
                              raw_fault.transpose() * eliminate_old * raw_fault),
                1e-8);
      EXPECT_LT(relativeError(
          window.history_summary.detector_response.transpose() *
              window.history_summary.detector_response,
          raw_fault.transpose() * eliminate_all * raw_fault), 1e-8);
      EXPECT_LT(relativeError(summary_block->jacobian_whitened.transpose() *
                                  summary_block->jacobian_whitened,
                              expected_h.transpose() * expected_h), 1e-8);
      EXPECT_LT(relativeError(summary_block->jacobian_whitened.transpose() *
                                  represented_fault,
                              expected_h.transpose() * expected_fault), 1e-8);
      EXPECT_LT(relativeError(represented_fault.transpose() * represented_fault,
                              expected_fault.transpose() * expected_fault),
                1e-8);
      EXPECT_LT(relativeError(summary_block->jacobian_whitened.transpose() *
                                  summary_block->residual_whitened,
                              expected_h.transpose() * expected_z), 1e-8);
      EXPECT_LT(relativeError(represented_fault.transpose() *
                                  summary_block->residual_whitened,
                              expected_fault.transpose() * expected_z), 1e-8);
      EXPECT_LT(std::abs(summary_block->residual_whitened.squaredNorm() -
                         expected_z.squaredNorm()) /
                    std::max({1.0,
                              summary_block->residual_whitened.squaredNorm(),
                              expected_z.squaredNorm()}),
                1e-8);
    }
  }

  CompleteRawReference out;
  out.protected_state_map = Eigen::MatrixXd::Zero(3, columns);
  out.protected_state_map.block<3, 3>(0, columns - 12) =
      tx.nominal_predicted_state.q_world_body.normalized().toRotationMatrix();
  // Form the same Schur quadratic through an orthogonal complement. Avoid
  // subtracting nearly equal matrices (I-UU') with the smaller, explicit IMU
  // covariance; that cancellation is magnified by the covariance inverse.
  const Eigen::JacobiSVD<Eigen::MatrixXd> history_full_svd(old_h, Eigen::ComputeFullU);
  const Eigen::MatrixXd complement = history_full_svd.matrixU().rightCols(history_rows-old_rank);
  const Eigen::MatrixXd projected_boundary = complement.transpose()*boundary_h;
  const Eigen::VectorXd projected_history = complement.transpose()*history_z;
  out.information = projected_boundary.transpose()*projected_boundary;
  out.rhs = projected_boundary.transpose()*projected_history;
  out.constant = projected_history.squaredNorm() + history_offset;
  out.unique_raw_rows = static_cast<std::size_t>(history_rows - old_rank);
  {
    const OracleHistorySummary retained_summary = oracleHistorySummary(
        old_h, boundary_h.leftCols(15), raw_fault, history_z,
        carrier_rank_tolerance);
    std::vector<Eigen::Index> retained_supported;
    std::vector<Eigen::Index> retained_perpendicular;
    for (Eigen::Index row = 0; row < retained_summary.r.rows(); ++row) {
      if (retained_summary.r.row(row).cwiseAbs().maxCoeff() > 0.0 ||
          std::abs(retained_summary.d(row)) > 0.0 ||
          retained_summary.t.row(row).cwiseAbs().maxCoeff() > 0.0) {
        retained_supported.push_back(row);
      }
    }
    for (Eigen::Index row = 0; row < retained_summary.f.rows(); ++row)
      retained_perpendicular.push_back(row);
    const std::size_t corrected_summary_rows =
        retained_supported.size() + retained_perpendicular.size();
    const std::size_t explicit_rows = manifest.served_rows - manifest.summary_rows;
    const std::size_t corrected_served_rows = corrected_summary_rows + explicit_rows;
    out.design = Eigen::MatrixXd::Zero(corrected_served_rows, columns);
    out.residual = Eigen::VectorXd::Zero(corrected_served_rows);
    out.protected_state_map = Eigen::MatrixXd::Zero(3, columns);
    out.protected_state_map.block<3, 3>(0, columns - 12) =
        tx.nominal_predicted_state.q_world_body.normalized().toRotationMatrix();
    out.history_response.resize(retained_supported.size(), raw_fault.cols());
    out.history_detector_response.resize(retained_perpendicular.size(),
                                         raw_fault.cols());
    for (std::size_t row = 0; row < retained_supported.size(); ++row) {
      out.history_response.row(row) =
          retained_summary.t.row(retained_supported[row]);
    }
    for (std::size_t row = 0; row < retained_perpendicular.size(); ++row) {
      out.history_detector_response.row(row) =
          retained_summary.f.row(retained_perpendicular[row]);
    }
    out.group_row_offsets[manifest.summary_group] = 0;
    for (std::size_t row = 0; row < retained_supported.size(); ++row) {
      out.design.row(static_cast<Eigen::Index>(row)).leftCols(15) =
          retained_summary.r.row(retained_supported[row]);
      out.residual(static_cast<Eigen::Index>(row)) =
          retained_summary.d(retained_supported[row]);
    }
    for (std::size_t row = 0; row < retained_perpendicular.size(); ++row) {
      out.residual(static_cast<Eigen::Index>(retained_supported.size() + row)) =
          retained_summary.d_perp(retained_perpendicular[row]);
    }
    CompleteRawReference::Block independent_summary;
    independent_summary.h = out.design.topRows(corrected_summary_rows);
    independent_summary.z = out.residual.head(corrected_summary_rows);
    independent_summary.raw_h = independent_summary.h;
    independent_summary.raw_z = independent_summary.z;
    independent_summary.covariance = Eigen::MatrixXd::Identity(
        corrected_summary_rows, corrected_summary_rows);
    independent_summary.whitener = independent_summary.covariance;
    out.blocks[manifest.summary_group] = std::move(independent_summary);
    out.group_row_counts[manifest.summary_group] = corrected_summary_rows;
  }

  const auto findActualGroupByExpectedId = [&](std::uint64_t expected_id)
      -> const PendingFactorGroup* {
    if (tx.imu_group.id.value() == expected_id) return &tx.imu_group;
    const auto current = std::find_if(
        tx.uwb_groups.begin(), tx.uwb_groups.end(),
        [&](const PendingFactorGroup& group) {
          return group.id.value() == expected_id;
        });
    if (current != tx.uwb_groups.end()) return &*current;
    for (std::size_t epoch = manifest.history_uwb_first;
         epoch < tx.proposed_epoch; ++epoch) {
      const auto* record = historicalRecord(epoch);
      if (!record) continue;
      const auto group = std::find_if(
          record->groups.begin(), record->groups.end(),
          [&](const PendingFactorGroup& value) {
            return value.id.value() == expected_id;
          });
      if (group != record->groups.end()) return &*group;
    }
    return nullptr;
  };
  std::vector<std::uint64_t> expected_explicit_groups = {
      oracleUwbGroup(manifest.explicit_uwb_first, manifest)};
  for (std::size_t epoch = manifest.explicit_imu_first;
       epoch <= manifest.explicit_imu_last; ++epoch) {
    expected_explicit_groups.push_back(oracleImuGroup(epoch, manifest));
    if (epoch >= manifest.explicit_uwb_first &&
        epoch <= manifest.explicit_uwb_last) {
      expected_explicit_groups.push_back(oracleUwbGroup(epoch, manifest));
    }
  }
  EXPECT_EQ(window.blocks.size(), expected_explicit_groups.size() + 1);
  EXPECT_FALSE(window.blocks.empty());
  if (!window.blocks.empty()) {
    EXPECT_EQ(window.blocks.front().group_id.value(), manifest.summary_group);
  }
  Eigen::Index served_row = static_cast<Eigen::Index>(
      out.history_response.rows() + out.history_detector_response.rows());
  for (const auto expected_group : expected_explicit_groups) {
    const auto block_it = std::find_if(window.blocks.begin(), window.blocks.end(),
        [&](const LinearizedFactorBlock& value) {
          return value.group_id.value() == expected_group;
        });
    EXPECT_NE(block_it, window.blocks.end()) << expected_group;
    const PendingFactorGroup* found = findActualGroupByExpectedId(expected_group);
    EXPECT_NE(found, nullptr) << expected_group;
    if (!found) continue;
    const PendingFactorGroup& group = *found;
    gtsam::Ordering ordering;
    std::set<gtsam::Key> seen;
    for (const auto key : group.keys) if (seen.insert(key).second) ordering.push_back(key);
    const auto gaussian = group.factors.linearize(*tx.frozen_values);
    const auto local = gaussian->jacobian(ordering);
    Eigen::MatrixXd embedded = Eigen::MatrixXd::Zero(local.first.rows(), columns);
    int local_column = 0;
    for (const auto key : ordering) {
      const int width = keyWidth(key);
      const std::size_t epoch = gtsam::Symbol(key).index();
      const int type = gtsam::Symbol(key).chr() == 'x' ? 0 :
          (gtsam::Symbol(key).chr() == 'v' ? 6 : 9);
      const int target = static_cast<int>(
          (epoch - manifest.detector_first_epoch) *
          manifest.state_columns_per_epoch) + type;
      embedded.block(0, target, local.first.rows(), width) =
          local.first.middleCols(local_column, width);
      local_column += width;
    }
    if (block_it != window.blocks.end()) {
      const auto& block = *block_it;
      EXPECT_TRUE(block.jacobian_whitened.isApprox(embedded, 2e-10));
      EXPECT_TRUE(block.residual_whitened.isApprox(local.second, 2e-10));
      EXPECT_TRUE((block.whitener * block.jacobian_raw).isApprox(
          embedded, 2e-10));
      EXPECT_TRUE((block.whitener * block.residual_raw).isApprox(
          local.second, 2e-10));
      EXPECT_EQ(block.covariance.rows(), local.first.rows());
      EXPECT_TRUE((block.whitener * block.covariance *
                   block.whitener.transpose()).isApprox(
          Eigen::MatrixXd::Identity(local.first.rows(), local.first.rows()),
          2e-10));
    }
    if (group.kind == FactorKind::UwbBatch) {
      const std::size_t epoch = gtsam::Symbol(group.keys.front()).index();
      const auto frozen_rows = frozen_uwb_rows.find(epoch);
      if (frozen_rows == frozen_uwb_rows.end()) {
        throw std::runtime_error("explicit recipe group lacks frozen UWB rows");
      }
      Eigen::MatrixXd recipe_covariance = Eigen::MatrixXd::Constant(
          frozen_rows->second.size(), frozen_rows->second.size(),
          frozen_off_diagonal);
      recipe_covariance.diagonal().setConstant(frozen_diagonal);
      EXPECT_TRUE(group.raw_covariance.isApprox(recipe_covariance, 1e-15));
      const auto noise = gtsam::noiseModel::Gaussian::Covariance(recipe_covariance);
      if (block_it != window.blocks.end()) {
        const auto& block = *block_it;
        EXPECT_TRUE(block.whitener.isApprox(noise->R(), 2e-10));
        EXPECT_TRUE(block.jacobian_whitened.isApprox(
            block.whitener * block.jacobian_raw, 2e-10));
        EXPECT_TRUE(block.residual_whitened.isApprox(
            block.whitener * block.residual_raw, 2e-10));
        EXPECT_TRUE(block.covariance.isApprox(recipe_covariance, 1e-15));
      }
      out.uwb_whiteners[expected_group] = noise->R();
    }
    CompleteRawReference::Block independent_block;
    independent_block.h = embedded;
    independent_block.z = local.second;
    if (group.kind == FactorKind::UwbBatch) {
      const std::size_t epoch = gtsam::Symbol(group.keys.front()).index();
      const auto frozen_rows = frozen_uwb_rows.at(epoch);
      independent_block.covariance = Eigen::MatrixXd::Constant(
          frozen_rows.size(), frozen_rows.size(), frozen_off_diagonal);
      independent_block.covariance.diagonal().setConstant(frozen_diagonal);
      independent_block.whitener =
          gtsam::noiseModel::Gaussian::Covariance(independent_block.covariance)->R();
      independent_block.raw_h = independent_block.whitener
          .triangularView<Eigen::Upper>().solve(embedded);
      independent_block.raw_z = independent_block.whitener
          .triangularView<Eigen::Upper>().solve(local.second);
    } else {
      independent_block.covariance = Eigen::MatrixXd::Identity(
          local.first.rows(), local.first.rows());
      independent_block.whitener = independent_block.covariance;
      independent_block.raw_h = embedded;
      independent_block.raw_z = local.second;
    }
    EXPECT_TRUE((independent_block.whitener * independent_block.raw_h)
                    .isApprox(independent_block.h, 2e-10));
    EXPECT_TRUE((independent_block.whitener * independent_block.raw_z)
                    .isApprox(independent_block.z, 2e-10));
    out.blocks[expected_group] = std::move(independent_block);
    out.group_row_offsets[expected_group] = served_row;
    out.group_row_counts[expected_group] = embedded.rows();
    EXPECT_LE(served_row + embedded.rows(), out.design.rows());
    if (served_row + embedded.rows() > out.design.rows()) continue;
    out.design.middleRows(served_row, embedded.rows()) = embedded;
    out.residual.segment(served_row, local.second.size()) = local.second;
    served_row += embedded.rows();
    out.information.noalias() += embedded.transpose() * embedded;
    out.rhs.noalias() += embedded.transpose() * local.second;
    out.constant += local.second.squaredNorm();
    out.unique_raw_rows += static_cast<std::size_t>(local.first.rows());
    out.explicit_group_ids.push_back(group.id.value());
  }
  // Replacement blocks are constructed from the frozen nonlinear source
  // factors and values selected by the recipe. Production action payload is
  // never consulted when forming these expected blocks.
  for (std::size_t epoch = manifest.explicit_uwb_first;
       epoch <= manifest.explicit_uwb_last; ++epoch) {
    for (const auto anchor : manifest.anchors) {
      const std::uint64_t replacement_id =
          oracleReplacementGroup(epoch, anchor, manifest);
      const PendingFactorGroup* found =
          findActualGroupByExpectedId(replacement_id);
      EXPECT_NE(found, nullptr) << replacement_id;
      if (!found) continue;
      const PendingFactorGroup& group = *found;
      gtsam::Ordering ordering;
      std::set<gtsam::Key> seen;
      for (const auto key : group.keys) {
        if (seen.insert(key).second) ordering.push_back(key);
      }
      const auto local = group.factors.linearize(*tx.frozen_values)->jacobian(ordering);
      Eigen::MatrixXd embedded = Eigen::MatrixXd::Zero(local.first.rows(), columns);
      int local_column = 0;
      for (const auto key : ordering) {
        const int width = keyWidth(key);
        const std::size_t key_epoch = gtsam::Symbol(key).index();
        const int type = gtsam::Symbol(key).chr() == 'x' ? 0 :
            (gtsam::Symbol(key).chr() == 'v' ? 6 : 9);
        const int target = static_cast<int>(
            (key_epoch - manifest.detector_first_epoch) *
            manifest.state_columns_per_epoch) + type;
        embedded.block(0, target, local.first.rows(), width) =
            local.first.middleCols(local_column, width);
        local_column += width;
      }
      CompleteRawReference::Block block;
      block.h = embedded;
      block.z = local.second;
      if (group.kind == FactorKind::UwbBatch) {
        const auto frozen_rows = frozen_uwb_rows.at(epoch);
        std::vector<std::size_t> retained_rows;
        for (std::size_t row = 0; row < frozen_rows.size(); ++row) {
          if (frozen_rows[row].anchor_id != anchor) retained_rows.push_back(row);
        }
        block.covariance = Eigen::MatrixXd::Constant(
            retained_rows.size(), retained_rows.size(), frozen_off_diagonal);
        block.covariance.diagonal().setConstant(frozen_diagonal);
        EXPECT_EQ(group.source_measurements.size(), retained_rows.size());
        for (std::size_t row = 0; row < retained_rows.size(); ++row) {
          EXPECT_EQ(group.source_measurements[row].value(),
                    frozen_rows[retained_rows[row]].measurement_id);
        }
        EXPECT_TRUE(group.raw_covariance.isApprox(block.covariance, 1e-15));
        block.whitener =
            gtsam::noiseModel::Gaussian::Covariance(block.covariance)->R();
        block.raw_h = block.whitener.triangularView<Eigen::Upper>()
            .solve(block.h);
        block.raw_z = block.whitener.triangularView<Eigen::Upper>()
            .solve(block.z);
      } else {
        block.covariance = Eigen::MatrixXd::Identity(
            local.first.rows(), local.first.rows());
        block.whitener = block.covariance;
        block.raw_h = block.h;
        block.raw_z = block.z;
      }
      EXPECT_TRUE((block.whitener * block.raw_h).isApprox(block.h, 2e-10));
      EXPECT_TRUE((block.whitener * block.raw_z).isApprox(block.z, 2e-10));
      out.blocks[replacement_id] = std::move(block);
    }
  }
  EXPECT_EQ(served_row, out.design.rows());
  out.covariance = svdPseudoInverse(out.information);
  out.state = out.covariance * out.rhs;
  out.objective = out.constant - out.rhs.dot(out.state);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(out.information);
  const double gate = svd.singularValues()(0) * 1e-12;
  out.rank = static_cast<int>((svd.singularValues().array() > gate).count());
  out.dof = static_cast<int>(out.design.rows()) - out.rank;
  return out;
}

CompleteRawReference verifyCompleteRawOracle(
    const EpochTransaction& tx, const LinearizedIntegrityWindow& window,
    std::size_t window_epochs, const OracleManifest& manifest,
    const std::vector<RawRowOwnershipAuditV1>& actual_raw_owners) {
  const CompleteRawReference reference =
      completeRawReference(tx, window, window_epochs, manifest,
                           actual_raw_owners);
  EXPECT_LT(relativeError(window.base_information, reference.information), 1e-8);
  EXPECT_LT(relativeError(window.base_information_rhs, reference.rhs), 1e-8);
  EXPECT_TRUE(static_cast<bool>(window.numerics));
  if (!window.numerics) return reference;
  EXPECT_LT(relativeError(window.numerics->spectral_state_increment,
                          reference.state), 5e-6);
  EXPECT_LT(relativeError(
      svdPseudoInverse(window.base_information), reference.covariance), 1e-8);
  EXPECT_NEAR(window.numerics->statistic +
                  window.history_summary.constant_offset,
              reference.objective,
              5e-7 * std::max(1.0, std::abs(reference.objective)));
  EXPECT_EQ(window.rank, reference.rank);
  EXPECT_EQ(window.dof, reference.dof);
  EXPECT_EQ(std::set<std::uint64_t>(reference.explicit_group_ids.begin(),
                                    reference.explicit_group_ids.end()).size(),
            reference.explicit_group_ids.size());
  return reference;
}

// Reassemble every declared fault mode from its native, unwhitened group map
// and the corresponding frozen raw-factor whitener.  The reference then uses
// an independent dense covariance solve instead of the production evaluator's
// cached projected modes.  This catches row-coordinate, group-placement and
// concatenation errors for every hypothesis, not merely finiteness of a local
// UWB block.
void verifyEveryHypothesisNumerics(
    const LinearizedIntegrityWindow& window,
    const GeneratedFaultModelSet& models,
    const std::vector<FaultModeEvidence>& evidence,
    const FrozenHypothesisNumerics& shared,
    const OracleManifest& manifest,
    CompleteRawReference& raw_reference,
    double rank_tolerance,
    std::vector<std::uint64_t>* independent_plausible) {
  ASSERT_NE(independent_plausible, nullptr);
  independent_plausible->clear();
  ASSERT_TRUE(static_cast<bool>(window.numerics));
  ASSERT_EQ(evidence.size(), models.hypotheses.size());
  ASSERT_EQ(shared.pl_entries.size(), models.hypotheses.size());

  ASSERT_EQ(raw_reference.design.rows(), window.H.rows());
  ASSERT_EQ(raw_reference.residual.size(), window.z.size());
  Eigen::JacobiSVD<Eigen::MatrixXd> window_svd(
      raw_reference.design, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const double singular_gate = window_svd.singularValues()(0) * 1e-10;
  double window_smallest = std::numeric_limits<double>::infinity();
  Eigen::VectorXd inverse = Eigen::VectorXd::Zero(
      window_svd.singularValues().size());
  Eigen::VectorXd inverse_squared = inverse;
  for (Eigen::Index column = 0; column < inverse.size(); ++column) {
    if (window_svd.singularValues()(column) > singular_gate) {
      window_smallest = std::min(window_smallest,
                                 window_svd.singularValues()(column));
      inverse(column) = 1.0 / window_svd.singularValues()(column);
      inverse_squared(column) = inverse(column) * inverse(column);
    }
  }
  const Eigen::MatrixXd covariance = window_svd.matrixV() *
      inverse_squared.asDiagonal() * window_svd.matrixV().transpose();
  const Eigen::VectorXd state = window_svd.matrixV() * inverse.asDiagonal() *
      window_svd.matrixU().transpose() * raw_reference.residual;
  const Eigen::VectorXd parity =
      raw_reference.residual - raw_reference.design * state;
  const double window_condition =
      window_svd.singularValues()(0) / window_smallest;
  const int detector_dof = static_cast<int>(raw_reference.design.rows()) -
      static_cast<int>((window_svd.singularValues().array() > singular_gate).count());
  ASSERT_GT(detector_dof, 0);
  const double independent_detector_threshold = boost::math::quantile(
      boost::math::complement(
          boost::math::chi_squared_distribution<double>(detector_dof),
          manifest.detector_p_fa));
  ASSERT_EQ(raw_reference.protected_state_map.rows(), 3);
  ASSERT_EQ(raw_reference.protected_state_map.cols(), raw_reference.design.cols());

  std::map<std::uint64_t, Eigen::MatrixXd> dense_modes;
  for (std::size_t mode_index = 0; mode_index < manifest.expected_modes;
       ++mode_index) {
    const OracleModeRecipe mode = oracleMode(mode_index + 1, manifest);
    const std::size_t onset = mode.onset;
    const std::uint64_t anchor = mode.anchor;
    const Eigen::Index anchor_index = std::distance(
        manifest.anchors.begin(), std::find(manifest.anchors.begin(),
                                            manifest.anchors.end(), anchor));
    const bool time_column = std::find(
        mode.family.native_columns.begin(), mode.family.native_columns.end(),
        "measurement-time-minus-onset-time") !=
        mode.family.native_columns.end();
    // Identity retains the declared physical dimension.  The terminal onset
    // has only one sampled occurrence, so the recipe separately declares the
    // effective numerical basis used by the candidate math.
    const int dimension = onset == manifest.alarm_epoch
        ? mode.family.terminal_onset_effective_dimension
        : mode.family.dimension;
    const std::size_t effective_onset =
        oracleEffectiveBoundaryOnset(mode, manifest);
    Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(
        raw_reference.design.rows(), dimension);

    const auto history_block =
        raw_reference.group_row_offsets.find(manifest.summary_group);
    ASSERT_NE(history_block, raw_reference.group_row_offsets.end());
    const Eigen::Index historical_native_columns = static_cast<Eigen::Index>(
        (manifest.detector_first_epoch - manifest.physical_first_epoch) *
        manifest.anchors.size() * 2);
    Eigen::MatrixXd history_weights = Eigen::MatrixXd::Zero(
        historical_native_columns, dimension);
    for (std::size_t epoch = manifest.physical_first_epoch;
         epoch < manifest.detector_first_epoch; ++epoch) {
      const bool active = mode.family.support == "onset-only"
          ? epoch == onset : epoch >= effective_onset;
      if (!active) continue;
      const Eigen::Index constant = static_cast<Eigen::Index>(
          (epoch - manifest.physical_first_epoch) *
              (manifest.anchors.size() * 2) +
          static_cast<std::size_t>(anchor_index) * 2);
      history_weights(constant, 0) = 1.0;
      if (time_column && dimension == 2) {
        history_weights(constant, 1) =
            (static_cast<double>(epoch - 1) - static_cast<double>(onset)) *
            static_cast<double>(manifest.epoch_period_ns) * 1e-9;
        history_weights(constant + 1, 1) = 1.0;
      }
    }
    const Eigen::Index corrected_summary_rows =
        raw_reference.history_response.rows() +
        raw_reference.history_detector_response.rows();
    Eigen::MatrixXd history_map(corrected_summary_rows, dimension);
    history_map.topRows(raw_reference.history_response.rows()) =
        raw_reference.history_response * history_weights;
    history_map.bottomRows(raw_reference.history_detector_response.rows()) =
        raw_reference.history_detector_response * history_weights;
    const Eigen::Index history_begin = history_block->second;
    dense.middleRows(history_begin, history_map.rows()) = history_map;

    for (std::size_t epoch = manifest.detector_first_epoch;
         epoch <= manifest.alarm_epoch; ++epoch) {
      const bool active = mode.family.support == "onset-only"
          ? epoch == onset : epoch >= effective_onset;
      if (!active) continue;
      const std::uint64_t group = oracleUwbGroup(epoch, manifest);
      const auto found = raw_reference.group_row_offsets.find(group);
      ASSERT_NE(found, raw_reference.group_row_offsets.end()) << group;
      const auto whitener = raw_reference.uwb_whiteners.find(group);
      ASSERT_NE(whitener, raw_reference.uwb_whiteners.end()) << group;
      ASSERT_EQ(whitener->second.rows(), 8);
      Eigen::MatrixXd raw = Eigen::MatrixXd::Zero(8, dimension);
      raw(anchor_index, 0) = 1.0;
      if (time_column && dimension == 2) {
        raw(anchor_index, 1) =
            static_cast<double>(epoch - onset) *
            static_cast<double>(manifest.epoch_period_ns) * 1e-9;
      }
      const Eigen::MatrixXd whitened = whitener->second * raw;
      const Eigen::Index begin = found->second;
      dense.middleRows(begin, whitened.rows()) = whitened;
    }
    dense_modes.emplace(mode_index + 1, std::move(dense));
  }

  for (std::size_t i = 0; i < manifest.expected_hypotheses; ++i) {
    const auto& hypothesis = models.hypotheses[i];
    ASSERT_EQ(hypothesis.id.value(), i + 1);
    ASSERT_EQ(hypothesis.modes,
              std::vector<FaultModeId>{FaultModeId(i + 1)});
    const Eigen::MatrixXd& fault = dense_modes.at(i + 1);
    const Eigen::MatrixXd cross = raw_reference.design.transpose() * fault;
    const Eigen::MatrixXd gram = fault.transpose() * fault -
        cross.transpose() * covariance * cross;
    const Eigen::MatrixXd response =
        raw_reference.protected_state_map * covariance * cross;
    const Eigen::VectorXd score = fault.transpose() * parity;
    const Eigen::MatrixXd gram_inverse = svdPseudoInverse(gram);
    const Eigen::VectorXd estimate = gram_inverse * score;
    const double explained = std::max(0.0, score.dot(estimate));
    const double profile = std::max(
        0.0, parity.squaredNorm() - explained);
    Eigen::JacobiSVD<Eigen::MatrixXd> gram_svd(gram);
    const double gram_largest = gram_svd.singularValues().size() == 0
        ? 0.0 : gram_svd.singularValues()(0);
    const double gram_smallest = gram_svd.singularValues().size() == 0
        ? 0.0 : gram_svd.singularValues().tail(1)(0);
    const double gram_condition = std::max(
        window_condition * window_condition,
        gram_smallest > 0.0 ? gram_largest / gram_smallest
                            : manifest.tolerance_cap);
    const double gram_tolerance = certifiedTolerance(
        gram.norm(), gram_condition, gram.size(), manifest);
    const double profile_tolerance = certifiedTolerance(
        std::abs(profile), gram_condition, fault.rows(), manifest);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> gram_eigen(
        0.5 * (gram + gram.transpose()));
    ASSERT_EQ(gram_eigen.info(), Eigen::Success);
    const double independent_rank_gate = certifiedTolerance(
        gram_largest, window_condition * window_condition,
        gram.rows(), manifest);
    const int independent_rank = static_cast<int>(
        (gram_eigen.eigenvalues().array() > independent_rank_gate).count());
    const int nullity = static_cast<int>(gram.cols()) - independent_rank;
    int independent_class = static_cast<int>(GramNullspaceClass::FullRank);
    if (nullity > 0) {
      const Eigen::MatrixXd hidden = response *
          gram_eigen.eigenvectors().leftCols(nullity);
      independent_class = hidden.cwiseAbs().maxCoeff() == 0.0
          ? static_cast<int>(GramNullspaceClass::Harmless)
          : static_cast<int>(GramNullspaceClass::Dangerous);
    }
    const bool expected_profile_valid = profile == profile &&
        std::isfinite(profile) && estimate.allFinite() && gram.allFinite();
    const bool expected_plausible = expected_profile_valid &&
        profile <= independent_detector_threshold + manifest.plausible_margin;
    if (expected_plausible) independent_plausible->push_back(i + 1);

    ASSERT_EQ(evidence[i].hypothesis, hypothesis.id);
    // The dense reference deliberately uses an SVD pseudo-inverse while the
    // production path uses its frozen square-root/normal-equation solves.  The
    // comparison gate covers their measured conditioning amplification; it is
    // not used by any production acceptance or risk decision.
    EXPECT_LE((evidence[i].fault_gram - gram).norm(), gram_tolerance)
        << "hypothesis=" << hypothesis.id.value();
    EXPECT_EQ(evidence[i].profile_valid, expected_profile_valid)
        << "hypothesis=" << hypothesis.id.value();
    EXPECT_EQ(std::isfinite(evidence[i].profile_j), expected_profile_valid)
        << "hypothesis=" << hypothesis.id.value();
    EXPECT_EQ(evidence[i].estimated_fault.allFinite(), expected_profile_valid)
        << "hypothesis=" << hypothesis.id.value();
    EXPECT_EQ(evidence[i].plausible, expected_plausible)
        << "hypothesis=" << hypothesis.id.value();
    if (expected_profile_valid) {
      EXPECT_NEAR(evidence[i].profile_j, profile,
                  profile_tolerance)
          << "hypothesis=" << hypothesis.id.value();
      EXPECT_LE((evidence[i].estimated_fault - estimate).norm(),
                certifiedTolerance(estimate.norm(), gram_condition,
                                   estimate.size(), manifest))
          << "hypothesis=" << hypothesis.id.value();
    }

    const auto& entry = shared.pl_entries[i];
    ASSERT_EQ(entry.hypothesis, hypothesis.id);
    FrozenHypothesisPlProofV1 proof;
    ASSERT_TRUE(frozenHypothesisPlProof(entry, &proof));
    ASSERT_TRUE(validateFrozenHypothesisPlEntry(proof));
    EXPECT_EQ(proof.nullspace_class, independent_class)
        << "hypothesis=" << hypothesis.id.value();
    EXPECT_EQ(entry.z_classification, independent_class)
        << "hypothesis=" << hypothesis.id.value();
    EXPECT_EQ(entry.z_rank, independent_rank)
        << "hypothesis=" << hypothesis.id.value();
    const bool expected_pl_valid =
        independent_class == static_cast<int>(GramNullspaceClass::FullRank) ||
        independent_class == static_cast<int>(GramNullspaceClass::Harmless);
    EXPECT_EQ(entry.valid, expected_pl_valid)
        << "hypothesis=" << hypothesis.id.value();
    EXPECT_EQ(entry.protected_slopes.allFinite(), expected_pl_valid)
        << "hypothesis=" << hypothesis.id.value();
    EXPECT_LE((proof.certified_gram - gram).norm(), gram_tolerance)
        << "hypothesis=" << hypothesis.id.value();
    EXPECT_LE((proof.protected_response - response).norm(),
              certifiedTolerance(response.norm(), gram_condition,
                                 response.size(), manifest))
        << "hypothesis=" << hypothesis.id.value();
    Eigen::Vector3d slopes;
    for (int axis = 0; axis < 3; ++axis) {
      const double squared = (response.row(axis) * gram_inverse *
                              response.row(axis).transpose())(0, 0);
      slopes(axis) = std::sqrt(std::max(0.0, squared));
    }
    EXPECT_EQ(slopes.allFinite(), expected_pl_valid)
        << "hypothesis=" << hypothesis.id.value();
    if (expected_pl_valid) {
      EXPECT_LE((entry.protected_slopes - slopes).norm(),
                certifiedTolerance(slopes.norm(), gram_condition,
                                   slopes.size(), manifest))
          << "hypothesis=" << hypothesis.id.value()
          << " actual=" << entry.protected_slopes.transpose()
          << " expected=" << slopes.transpose();
    }
  }
  std::cout << "[P0-07-HYPOTHESIS-ORACLE] modes=" << models.modes.size()
            << " hypotheses=" << models.hypotheses.size()
            << " gram_profile_response_pl=PASS\n";
  raw_reference.fault_modes = std::move(dense_modes);
}

// Independent raw-factor oracle for the current UWB group of the frozen
// replay.  It starts only from the raw batch and the independently named
// linearization state: it never reads production H/z or a builder-produced
// factor matrix to construct its expected rows/covariance/RHS.
void verifyRawUwbOracle(const UwbBatch& batch,
                        const EpochTransaction& transaction,
                        const LinearizedIntegrityWindow& window,
                        const GeneratedFaultModelSet& models,
                        const OracleManifest& manifest) {
  ASSERT_FALSE(transaction.uwb_groups.empty());
  const PendingFactorGroup& group = transaction.uwb_groups.front();
  const auto block = std::find_if(
      window.blocks.begin(), window.blocks.end(),
      [&](const LinearizedFactorBlock& value) {
        return value.group_id == group.id;
      });
  ASSERT_NE(block, window.blocks.end());
  const int state_columns = static_cast<int>(manifest.state_columns);
  const int pose_column = static_cast<int>(
      (transaction.proposed_epoch - manifest.detector_first_epoch) *
      manifest.state_columns_per_epoch);
  const Eigen::Index rows = static_cast<Eigen::Index>(batch.measurements.size());
  Eigen::MatrixXd raw_h = Eigen::MatrixXd::Zero(rows, state_columns);
  Eigen::VectorXd raw_z(rows);
  Eigen::MatrixXd raw_covariance = batch.covariance_m2;
  const Eigen::Vector3d position =
      transaction.cv_predicted_state.position_world_m;
  const Eigen::Matrix3d rotation =
      transaction.cv_predicted_state.q_world_body.toRotationMatrix();
  ASSERT_TRUE(static_cast<bool>(transaction.frozen_values));
  const auto frozen_pose = transaction.frozen_values->at<gtsam::Pose3>(
      gtsam::Symbol('x', transaction.proposed_epoch));
  EXPECT_TRUE(frozen_pose.translation().isApprox(position, 1e-15))
      << "frozen=" << frozen_pose.translation().transpose()
      << " cv=" << position.transpose();
  for (Eigen::Index row = 0; row < rows; ++row) {
    const auto& measurement =
        batch.measurements[static_cast<std::size_t>(row)];
    const Eigen::Vector3d delta = position - measurement.anchor_position_m;
    const double predicted = delta.norm();
    ASSERT_GT(predicted, 1e-9);
    raw_h.block<1, 3>(row, pose_column + 3) =
        (delta.transpose() / predicted) * rotation;
    raw_z(row) = measurement.range_m - predicted;  // Ax-b RHS convention.
  }
  ASSERT_EQ(raw_covariance.rows(), rows);
  EXPECT_TRUE(block->jacobian_raw.isApprox(raw_h, 2e-10))
      << "raw H max error="
      << (block->jacobian_raw - raw_h).cwiseAbs().maxCoeff();
  EXPECT_TRUE(block->residual_raw.isApprox(raw_z, 2e-10))
      << "raw z max error="
      << (block->residual_raw - raw_z).cwiseAbs().maxCoeff()
      << " actual=" << block->residual_raw.transpose()
      << " expected=" << raw_z.transpose()
      << " cv_position=" << position.transpose();
  EXPECT_TRUE(block->covariance.isApprox(raw_covariance, 1e-15));

  Eigen::LLT<Eigen::MatrixXd> llt(raw_covariance);
  ASSERT_EQ(llt.info(), Eigen::Success);
  const Eigen::MatrixXd information = llt.solve(
      Eigen::MatrixXd::Identity(rows, rows));
  Eigen::LLT<Eigen::MatrixXd> information_llt(information);
  ASSERT_EQ(information_llt.info(), Eigen::Success);
  const Eigen::MatrixXd independent_whitener = information_llt.matrixU();
  const Eigen::MatrixXd h = independent_whitener * raw_h;
  const Eigen::VectorXd z = independent_whitener * raw_z;
  EXPECT_TRUE(block->whitener.isApprox(independent_whitener, 2e-10));
  EXPECT_TRUE(block->jacobian_whitened.isApprox(h, 2e-10))
      << "white H max error="
      << (block->jacobian_whitened - h).cwiseAbs().maxCoeff();
  EXPECT_TRUE(block->residual_whitened.isApprox(z, 2e-10))
      << "white z max error="
      << (block->residual_whitened - z).cwiseAbs().maxCoeff();

  // Independent SVD objective/state/full-covariance/DOF and parity fault
  // products for this raw factor.  These remain independent even though the
  // complete production window contains prior/IMU/history rows as well.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      h, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular = svd.singularValues();
  Eigen::VectorXd inverse = Eigen::VectorXd::Zero(singular.size());
  int rank = 0;
  const double gate = singular.size() == 0 ? 0.0 : singular(0) * 1e-12;
  for (Eigen::Index i = 0; i < singular.size(); ++i) {
    if (singular(i) > gate) {
      inverse(i) = 1.0 / singular(i);
      ++rank;
    }
  }
  const Eigen::MatrixXd pinv =
      svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose();
  const Eigen::VectorXd state = pinv * z;
  const Eigen::MatrixXd covariance =
      svd.matrixV() * inverse.cwiseAbs2().asDiagonal() *
      svd.matrixV().transpose();
  const Eigen::VectorXd residual = z - h * state;
  const Eigen::MatrixXd parity =
      Eigen::MatrixXd::Identity(rows, rows) - h * pinv;
  const Eigen::MatrixXd fault =
      llt.matrixL().solve(Eigen::MatrixXd::Identity(rows, rows));
  const Eigen::MatrixXd gram = fault.transpose() * parity * fault;
  Eigen::MatrixXd protected_state_map = Eigen::MatrixXd::Zero(3, state_columns);
  protected_state_map.block<3, 3>(0, state_columns - 12) = rotation;
  const Eigen::MatrixXd response = protected_state_map * pinv * fault;
  EXPECT_TRUE(state.allFinite());
  EXPECT_TRUE(covariance.allFinite());
  EXPECT_TRUE(gram.allFinite());
  EXPECT_TRUE(response.allFinite());
  EXPECT_EQ(rows - rank, residual.size() - rank);
  EXPECT_GE(residual.squaredNorm(), 0.0);

  std::size_t bound_current_modes = 0;
  for (const auto& mode : models.modes) {
    const auto raw = mode.raw_group_maps.find(group.id);
    if (raw == mode.raw_group_maps.end()) continue;
    ASSERT_EQ(raw->second.rows(), rows);
    ASSERT_GE(raw->second.cols(), 1);
    EXPECT_TRUE(raw->second.allFinite());
    ++bound_current_modes;
  }
  EXPECT_GT(bound_current_modes, 0u);
  EXPECT_TRUE(window.history_summary.valid) << window.history_summary.reason;
  EXPECT_EQ(window.history_summary.response.cols(),
            static_cast<Eigen::Index>(window.history_summary.fault_columns));
  EXPECT_EQ(window.history_summary.detector_response.cols(),
            static_cast<Eigen::Index>(window.history_summary.fault_columns));
}

// This small, typed oracle is a mutation fixture for the recipe itself.  It
// deliberately owns its raw measurements and algebra.  The expensive
// acceptance below independently compares the complete production chain.
struct TypedRawRow {
  std::uint64_t measurement = 0;
  std::uint64_t owner = 0;
  std::uint64_t slot = 0;
  Eigen::RowVector3d h = Eigen::RowVector3d::Zero();
  double z = 0.0;
  double variance = 0.0;
};

struct TypedModeResult {
  std::uint64_t id = 0;
  std::uint64_t anchor = 0;
  std::size_t onset = 0;
  int family = 0;
  Eigen::MatrixXd map;
  Eigen::MatrixXd gram;
  double profile = std::numeric_limits<double>::infinity();
  bool profile_valid = false;
  bool plausible = false;
};

struct TypedLedgerEntry {
  std::uint64_t action = 0;
  std::uint64_t hypothesis = 0;
  Eigen::Vector3d pl = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double allocated_risk = 0.0;
  double charged_risk = 0.0;
  bool covered = false;
  bool risk_valid = false;
  bool eligible = false;
};

struct TypedOracleResult {
  std::vector<TypedRawRow> raw_rows;
  Eigen::MatrixXd raw_h;
  Eigen::VectorXd raw_z;
  Eigen::MatrixXd raw_c;
  std::vector<TypedRawRow> served_rows;
  Eigen::MatrixXd served_h;
  Eigen::VectorXd served_z;
  Eigen::MatrixXd served_c;
  std::vector<std::pair<std::size_t, std::size_t>> state_columns;
  std::vector<TypedModeResult> modes;
  std::vector<std::uint64_t> plausible;
  std::vector<OracleOperation> actions;
  std::vector<std::uint64_t> replacement_groups;
  std::vector<TypedLedgerEntry> ledger;
  std::uint64_t selected_action = 0;
  std::string refusal;
  std::uint64_t missing_group = 0;
  std::uint64_t missing_replacement = 0;
  int missing_family = 0;
  double detector_threshold = std::numeric_limits<double>::infinity();
  double plausibility_boundary = std::numeric_limits<double>::infinity();
  double horizontal_alert_margin = -std::numeric_limits<double>::infinity();
  double vertical_alert_margin = -std::numeric_limits<double>::infinity();
  double finite_comparison_tolerance = 0.0;
  double ill_conditioned_comparison_tolerance = 0.0;
};

TypedOracleResult evaluateTypedRecipeOracle(const OracleManifest& manifest) {
  TypedOracleResult out;
  if (manifest.detector_type != "joint_window_residual_chi_square" ||
      manifest.detector_distribution != "chi-square" ||
      !manifest.detector_squared_norm) {
    throw std::runtime_error("FIXED_DETECTOR_ALGEBRA_V1");
  }
  if (manifest.risk_allocation_policy != "prior_weighted_outcome_conditioned" ||
      manifest.risk_allocation_rule != "total/generated-action-count") {
    throw std::runtime_error("FIXED_RISK_ALLOCATION_V1");
  }

  // Reconstruct raw nonlinear range rows.  Anchor coordinates are a pure
  // function of the declared identities; the terminal +20 m fault is the
  // dedicated observable fixture used by every mode/profile mutation.
  TypedRawRow marginal;
  marginal.measurement = 1;
  marginal.owner = manifest.marginal_group;
  marginal.slot = manifest.marginal_slot;
  marginal.h << 1.0, 0.0, 0.0;
  marginal.variance = 1.0;
  out.raw_rows.push_back(marginal);
  for (std::size_t epoch = 1; epoch <= manifest.replay_epochs; ++epoch) {
    const double t = static_cast<double>(epoch * manifest.epoch_period_ns) *
        1e-9;
    const Eigen::Vector3d receiver(0.05 * t, -0.02 * t, 1.0);
    for (std::size_t a = 0; a < manifest.anchors.size(); ++a) {
      const double angle = 0.7853981633974483 * static_cast<double>(a);
      const Eigen::Vector3d anchor(6.0 * std::cos(angle),
                                   6.0 * std::sin(angle), 0.5 + 0.1 * a);
      const Eigen::Vector3d delta = receiver - anchor;
      TypedRawRow row;
      row.measurement = epoch * 100 + manifest.anchors[a];
      row.owner = oracleUwbGroup(epoch, manifest);
      row.slot = oracleUwbSlot(epoch, manifest);
      row.h = delta.transpose() / delta.norm();
      row.z = epoch == manifest.alarm_epoch && a == 0 ? 20.0 : 0.0;
      row.variance = 0.04 + 0.001 * static_cast<double>(a);
      out.raw_rows.push_back(row);
    }
    for (std::size_t sample = 0; sample < manifest.imu_samples_per_epoch;
         ++sample) {
      TypedRawRow row;
      row.measurement = 500000 + epoch * 100 + sample;
      row.owner = oracleImuGroup(epoch, manifest);
      row.slot = oracleImuSlot(epoch, manifest);
      row.h << 0.01 * (sample + 1), 0.02 * t, -0.005 * epoch;
      row.z = 1e-3 * std::sin(t + static_cast<double>(sample));
      row.variance = 0.01;
      out.raw_rows.push_back(row);
    }
  }
  out.raw_h.resize(out.raw_rows.size(), 3);
  out.raw_z.resize(out.raw_rows.size());
  out.raw_c = Eigen::MatrixXd::Zero(out.raw_rows.size(), out.raw_rows.size());
  for (std::size_t i = 0; i < out.raw_rows.size(); ++i) {
    out.raw_h.row(i) = out.raw_rows[i].h;
    out.raw_z(static_cast<Eigen::Index>(i)) = out.raw_rows[i].z;
    out.raw_c(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) =
        out.raw_rows[i].variance;
    if (i > 0 && out.raw_rows[i - 1].owner == out.raw_rows[i].owner) {
      const double covariance = 0.1 * std::sqrt(
          out.raw_rows[i - 1].variance * out.raw_rows[i].variance);
      out.raw_c(static_cast<Eigen::Index>(i - 1),
                static_cast<Eigen::Index>(i)) = covariance;
      out.raw_c(static_cast<Eigen::Index>(i),
                static_cast<Eigen::Index>(i - 1)) = covariance;
    }
  }

  for (std::size_t epoch = manifest.state_first_epoch;
       epoch <= manifest.state_last_epoch; ++epoch) {
    out.state_columns.emplace_back(epoch,
        (epoch - manifest.state_first_epoch) *
            manifest.state_columns_per_epoch);
  }
  for (const auto& row : out.raw_rows) {
    const std::size_t epoch = static_cast<std::size_t>(row.measurement / 100);
    if (epoch >= manifest.explicit_uwb_first &&
        epoch <= manifest.explicit_uwb_last) {
      out.served_rows.push_back(row);
    }
  }
  // Explicit typed summary/IMU rows close the declared served census.  Their
  // numeric values are deterministic consequences of row ownership.
  while (out.served_rows.size() < manifest.served_rows) {
    TypedRawRow row;
    const std::size_t ordinal = out.served_rows.size();
    row.measurement = 900000 + ordinal;
    row.owner = ordinal < manifest.summary_rows
        ? manifest.summary_group
        : oracleImuGroup(manifest.explicit_imu_first +
              (ordinal - manifest.summary_rows) /
                  std::max<std::size_t>(1, manifest.explicit_imu_rows),
              manifest);
    row.slot = ordinal < manifest.summary_rows
        ? manifest.marginal_slot
        : oracleImuSlot(manifest.explicit_imu_first +
              (ordinal - manifest.summary_rows) /
                  std::max<std::size_t>(1, manifest.explicit_imu_rows),
              manifest);
    row.h << 0.001 * static_cast<double>((ordinal % 7) + 1),
             0.001 * static_cast<double>((ordinal % 5) + 1),
             0.001 * static_cast<double>((ordinal % 3) + 1);
    row.z = 1e-4 * static_cast<double>(ordinal % 11);
    row.variance = 1.0;
    out.served_rows.push_back(row);
  }
  if (out.served_rows.size() > manifest.served_rows) {
    out.served_rows.resize(manifest.served_rows);
  }
  out.served_h.resize(out.served_rows.size(), 3);
  out.served_z.resize(out.served_rows.size());
  out.served_c = Eigen::MatrixXd::Zero(out.served_rows.size(),
                                       out.served_rows.size());
  for (std::size_t i = 0; i < out.served_rows.size(); ++i) {
    out.served_h.row(i) = out.served_rows[i].h;
    out.served_z(static_cast<Eigen::Index>(i)) = out.served_rows[i].z;
    out.served_c(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) =
        out.served_rows[i].variance;
  }

  const int detector_dof = static_cast<int>(manifest.served_rows) -
      static_cast<int>(manifest.state_columns);
  if (detector_dof <= 0 || !(manifest.detector_p_fa > 0.0) ||
      !(manifest.detector_p_fa < 1.0)) {
    throw std::runtime_error("DERIVED_DETECTOR_DOF_OR_PFA_INVALID");
  }
  out.detector_threshold = boost::math::quantile(
      boost::math::complement(
          boost::math::chi_squared_distribution<double>(detector_dof),
          manifest.detector_p_fa));
  out.plausibility_boundary =
      out.detector_threshold + manifest.plausible_margin;

  // The mode census ends at onset_last, while a persistent mode's support
  // ends at alarm_epoch.  Keep those axes independent so an otherwise-valid
  // alarm mutation is represented by an additional typed observation row
  // instead of indexing past a matrix sized only for candidate onsets.
  const std::size_t temporal_last =
      std::max(manifest.onset_last, manifest.alarm_epoch);
  const std::size_t per_anchor =
      temporal_last - manifest.onset_first + 1;
  const std::size_t temporal_rows = manifest.anchors.size() * per_anchor;
  Eigen::VectorXd temporal_z = Eigen::VectorXd::Zero(temporal_rows);
  const std::size_t terminal = manifest.alarm_epoch - manifest.onset_first;
  if (terminal >= temporal_rows) {
    throw std::runtime_error("DERIVED_ALARM_OUTSIDE_MODE_FIXTURE");
  }
  temporal_z(static_cast<Eigen::Index>(terminal)) = 20.0;
  for (std::uint64_t ordinal = 1; ordinal <= manifest.expected_modes;
       ++ordinal) {
    const OracleModeRecipe mode = oracleMode(ordinal, manifest);
    TypedModeResult typed;
    typed.id = ordinal;
    typed.anchor = mode.anchor;
    typed.onset = mode.onset;
    typed.family = mode.family.kind;
    const int columns = mode.onset == manifest.alarm_epoch
        ? mode.family.terminal_onset_effective_dimension
        : mode.family.dimension;
    typed.map = Eigen::MatrixXd::Zero(temporal_rows, columns);
    const auto anchor_it = std::find(manifest.anchors.begin(),
                                     manifest.anchors.end(), mode.anchor);
    const std::size_t anchor_index = static_cast<std::size_t>(
        std::distance(manifest.anchors.begin(), anchor_it));
    for (const auto epoch : oracleModeEpochs(mode, manifest)) {
      const std::size_t row = anchor_index * per_anchor +
          epoch - manifest.onset_first;
      typed.map(static_cast<Eigen::Index>(row), 0) = 1.0;
      if (columns > 1) {
        typed.map(static_cast<Eigen::Index>(row), 1) =
            static_cast<double>(epoch -
                oracleEffectiveBoundaryOnset(mode, manifest)) *
            static_cast<double>(manifest.epoch_period_ns) * 1e-9;
      }
    }
    typed.gram = typed.map.transpose() * typed.map;
    const Eigen::VectorXd estimate = svdPseudoInverse(typed.gram) *
        typed.map.transpose() * temporal_z;
    typed.profile = (temporal_z - typed.map * estimate).squaredNorm();
    typed.profile_valid = typed.map.allFinite() && typed.gram.allFinite() &&
        std::isfinite(typed.profile);
    typed.plausible = typed.profile_valid &&
        typed.profile <= out.plausibility_boundary;
    if (typed.plausible) out.plausible.push_back(ordinal);
    out.modes.push_back(std::move(typed));
  }
  out.actions = deriveOracleOperations(out.plausible, manifest);
  for (std::size_t epoch = manifest.explicit_uwb_first;
       epoch <= manifest.explicit_uwb_last; ++epoch) {
    for (const auto anchor : manifest.anchors) {
      out.replacement_groups.push_back(
          oracleReplacementGroup(epoch, anchor, manifest));
    }
  }

  const double action_risk = manifest.total_risk /
      static_cast<double>(std::max<std::size_t>(1, out.actions.size()));
  for (const auto& action : out.actions) {
    for (const auto hypothesis : out.plausible) {
      const auto& mode = out.modes.at(hypothesis - 1);
      TypedLedgerEntry entry;
      entry.action = action.id;
      entry.hypothesis = hypothesis;
      entry.covered = std::find(action.covered_modes.begin(),
                                action.covered_modes.end(), hypothesis) !=
          action.covered_modes.end();
      const Eigen::MatrixXd gram_inverse = svdPseudoInverse(mode.gram);
      const double slope = std::sqrt(std::max(
          0.0, gram_inverse.diagonal().maxCoeff()));
      const double fault_multiplier = std::sqrt(std::max(
          0.0, -2.0 * std::log(std::max(
              1e-300, manifest.uwb_p_md *
                  manifest.uwb_prior_probability_bound))));
      const double nominal_multiplier = boost::math::quantile(
          boost::math::normal_distribution<double>(),
          1.0 - manifest.nominal_axis_tail / 2.0);
      entry.pl = Eigen::Vector3d::Constant(
          nominal_multiplier + (entry.covered ? 0.0 : slope * fault_multiplier));
      entry.allocated_risk = action_risk;
      entry.charged_risk = manifest.p_nm + manifest.p_bridge_escape +
          manifest.p_history_contamination + manifest.p_model_escape +
          manifest.uwb_p_md * manifest.uwb_prior_probability_bound;
      entry.risk_valid = entry.charged_risk <= entry.allocated_risk;
      entry.eligible = entry.risk_valid &&
          entry.pl.head<2>().norm() <= manifest.horizontal_alert_limit_m &&
          entry.pl.z() <= manifest.vertical_alert_limit_m;
      out.ledger.push_back(std::move(entry));
    }
  }
  const auto selected = std::find_if(out.actions.begin(), out.actions.end(),
      [&](const OracleOperation& action) {
        return std::all_of(out.plausible.begin(), out.plausible.end(),
            [&](std::uint64_t id) {
              return std::find(action.covered_modes.begin(),
                               action.covered_modes.end(), id) !=
                  action.covered_modes.end();
            }) && std::all_of(out.ledger.begin(), out.ledger.end(),
                [&](const TypedLedgerEntry& entry) {
                  return entry.action != action.id || entry.eligible;
                });
      });
  // The mathematical candidate is evaluated above.  N01 remains the final
  // formal model gate, exactly as in the primary acceptance.
  out.selected_action = selected == out.actions.end() ? 0 : selected->id;
  double worst_horizontal = 0.0;
  double worst_vertical = 0.0;
  for (const auto& entry : out.ledger) {
    worst_horizontal = std::max(worst_horizontal, entry.pl.head<2>().norm());
    worst_vertical = std::max(worst_vertical, entry.pl.z());
  }
  out.horizontal_alert_margin =
      manifest.horizontal_alert_limit_m - worst_horizontal;
  out.vertical_alert_margin =
      manifest.vertical_alert_limit_m - worst_vertical;
  out.refusal = manifest.risk_model_calibration ==
          "UNKNOWN_N01_NO_AUTHENTICATED_ARTIFACT"
      ? "IMU_MODEL_UNQUALIFIED" :
        (out.selected_action == 0 ? "NO_ELIGIBLE_ACTION" : "NONE");
  out.missing_group = oracleUwbGroup(manifest.missing_onset, manifest);
  out.missing_replacement = oracleReplacementGroup(
      manifest.missing_onset, manifest.missing_anchor, manifest);
  out.missing_family = manifest.missing_family;
  out.finite_comparison_tolerance = certifiedTolerance(
      out.served_h.norm(), 13.0, out.served_h.rows(), manifest);
  out.ill_conditioned_comparison_tolerance = certifiedTolerance(
      out.served_h.norm(), 1e20, out.served_h.rows(), manifest);
  return out;
}

enum class TypedSemanticField {
  Raw,
  State,
  Ownership,
  Served,
  Modes,
  Actions,
  Risk,
  Missing,
  Tolerance,
};

TypedSemanticField semanticFieldForRecipePath(const std::string& path) {
  if (path.rfind("replay.", 0) == 0)
    return path == "replay.physical_first_epoch"
        ? TypedSemanticField::Modes : TypedSemanticField::Raw;
  if (path.rfind("factor_construction.state_columns.", 0) == 0)
    return TypedSemanticField::State;
  if (path.rfind("factor_construction.history_raw.", 0) == 0)
    return TypedSemanticField::Ownership;
  if (path.rfind("factor_construction.served_window.", 0) == 0)
    return TypedSemanticField::Served;
  if (path.rfind("fault_contract.", 0) == 0)
    return TypedSemanticField::Modes;
  if (path.rfind("detector_and_action_recipe.detector.", 0) == 0)
    return TypedSemanticField::Modes;
  if (path.rfind("detector_and_action_recipe.action_generation.", 0) == 0)
    return TypedSemanticField::Actions;
  if (path.rfind("detector_and_action_recipe.risk.", 0) == 0 ||
      path.rfind("detector_and_action_recipe.selection.", 0) == 0)
    return TypedSemanticField::Risk;
  if (path.rfind("missing_provenance_fixture.", 0) == 0)
    return TypedSemanticField::Missing;
  if (path.rfind("tolerance_policy.", 0) == 0)
    return TypedSemanticField::Tolerance;
  throw std::runtime_error("leaf has no typed semantic field: " + path);
}

bool sameRawRows(const std::vector<TypedRawRow>& lhs,
                 const std::vector<TypedRawRow>& rhs) {
  if (lhs.size() != rhs.size()) return false;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].measurement != rhs[i].measurement ||
        lhs[i].owner != rhs[i].owner || lhs[i].slot != rhs[i].slot ||
        lhs[i].h != rhs[i].h || lhs[i].z != rhs[i].z ||
        lhs[i].variance != rhs[i].variance) return false;
  }
  return true;
}

bool sameOperations(const std::vector<OracleOperation>& lhs,
                    const std::vector<OracleOperation>& rhs) {
  if (lhs.size() != rhs.size()) return false;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].id != rhs[i].id ||
        lhs[i].exclusion_cardinality != rhs[i].exclusion_cardinality ||
        lhs[i].remove != rhs[i].remove || lhs[i].add != rhs[i].add ||
        lhs[i].covered_modes != rhs[i].covered_modes) return false;
  }
  return true;
}

bool typedSemanticFieldChanged(const TypedOracleResult& lhs,
                               const TypedOracleResult& rhs,
                               TypedSemanticField field) {
  switch (field) {
    case TypedSemanticField::Raw:
      return !sameRawRows(lhs.raw_rows, rhs.raw_rows) ||
          lhs.raw_h.rows() != rhs.raw_h.rows() ||
          lhs.raw_h.cols() != rhs.raw_h.cols() ||
          !lhs.raw_h.isApprox(rhs.raw_h, 0.0) ||
          !lhs.raw_z.isApprox(rhs.raw_z, 0.0) ||
          !lhs.raw_c.isApprox(rhs.raw_c, 0.0);
    case TypedSemanticField::State:
      return lhs.state_columns != rhs.state_columns;
    case TypedSemanticField::Ownership:
      return !sameRawRows(lhs.raw_rows, rhs.raw_rows) ||
          !sameRawRows(lhs.served_rows, rhs.served_rows);
    case TypedSemanticField::Served:
      return !sameRawRows(lhs.served_rows, rhs.served_rows) ||
          lhs.served_h.rows() != rhs.served_h.rows() ||
          !lhs.served_h.isApprox(rhs.served_h, 0.0) ||
          !lhs.served_z.isApprox(rhs.served_z, 0.0) ||
          !lhs.served_c.isApprox(rhs.served_c, 0.0);
    case TypedSemanticField::Modes:
      if (lhs.detector_threshold != rhs.detector_threshold ||
          lhs.plausibility_boundary != rhs.plausibility_boundary ||
          lhs.plausible != rhs.plausible || lhs.modes.size() != rhs.modes.size())
        return true;
      for (std::size_t i = 0; i < lhs.modes.size(); ++i) {
        const auto& a = lhs.modes[i];
        const auto& b = rhs.modes[i];
        if (a.id != b.id || a.anchor != b.anchor || a.onset != b.onset ||
            a.family != b.family || a.map.rows() != b.map.rows() ||
            a.map.cols() != b.map.cols() || !a.map.isApprox(b.map, 0.0) ||
            !a.gram.isApprox(b.gram, 0.0) || a.profile != b.profile ||
            a.profile_valid != b.profile_valid || a.plausible != b.plausible)
          return true;
      }
      return false;
    case TypedSemanticField::Actions:
      return !sameOperations(lhs.actions, rhs.actions) ||
          lhs.replacement_groups != rhs.replacement_groups;
    case TypedSemanticField::Risk:
      if (lhs.selected_action != rhs.selected_action ||
          lhs.refusal != rhs.refusal ||
          lhs.horizontal_alert_margin != rhs.horizontal_alert_margin ||
          lhs.vertical_alert_margin != rhs.vertical_alert_margin ||
          lhs.ledger.size() != rhs.ledger.size())
        return true;
      for (std::size_t i = 0; i < lhs.ledger.size(); ++i) {
        const auto& a = lhs.ledger[i];
        const auto& b = rhs.ledger[i];
        if (a.action != b.action || a.hypothesis != b.hypothesis ||
            a.pl != b.pl || a.allocated_risk != b.allocated_risk ||
            a.charged_risk != b.charged_risk || a.covered != b.covered ||
            a.risk_valid != b.risk_valid || a.eligible != b.eligible)
          return true;
      }
      return false;
    case TypedSemanticField::Missing:
      return lhs.missing_group != rhs.missing_group ||
          lhs.missing_replacement != rhs.missing_replacement ||
          lhs.missing_family != rhs.missing_family;
    case TypedSemanticField::Tolerance:
      return lhs.finite_comparison_tolerance !=
                 rhs.finite_comparison_tolerance ||
          lhs.ill_conditioned_comparison_tolerance !=
                 rhs.ill_conditioned_comparison_tolerance;
  }
  return false;
}

struct ActualObservation {
  std::vector<std::uint64_t> plausible;
  std::size_t generated_action_count = 0;
  std::uint64_t winner = 0;
  std::string note;
};

void validateObservedMetadataAgainstActual(const OracleManifest& manifest,
                                           const ActualObservation& actual) {
  if (manifest.observed_plausible_modes != actual.plausible)
    throw std::runtime_error("OBSERVED_PLAUSIBLE_MISMATCH");
  if (manifest.observed_generated_action_count != actual.generated_action_count)
    throw std::runtime_error("OBSERVED_ACTION_COUNT_MISMATCH");
  if (manifest.observed_winner != actual.winner)
    throw std::runtime_error("OBSERVED_WINNER_MISMATCH");
  if (manifest.observed_note != actual.note)
    throw std::runtime_error("OBSERVED_NOTE_MISMATCH");
}

ActualObservation captureSameReplayActualObservation(
    const OracleManifest& oracle) {
  IntegrityConfig config = IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_uwb_order1.yaml");
  config.incremental.fixed_lag_epochs =
      oracle.alarm_epoch - oracle.physical_first_epoch + 1;
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary;
  boundary.id = MeasurementId(900000);
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  ActualObservation actual;
  for (std::size_t epoch = 1; epoch <= oracle.alarm_epoch; ++epoch) {
    appendProductionReplayImu(&estimator, config, epoch,
                              oracle.imu_samples_per_epoch,
                              oracle.epoch_period_ns);
    EpochPreparationOptions options;
    options.build_integrity_material = true;
    options.build_uwb_recovery_material = true;
    options.build_recoverable_history = true;
    auto transaction = estimator.prepareEpoch(
        makeProductionReplayBatch(config, epoch, oracle.alarm_epoch,
                                  oracle.epoch_period_ns), options);
    if (epoch == oracle.alarm_epoch) {
      const auto window = estimator.buildIntegrityWindow(
          transaction,
          IntegrityWindowRequest{config.integrity_window.epochs});
      if (!window.model_valid) {
        throw std::runtime_error("ACTUAL_REPLAY_WINDOW_INVALID:" +
                                 window.reason);
      }
      ImuFaultSubspaces no_imu;
      LinearizedFactorBlock no_bridge;
      const auto generator_config = productionGeneratorConfig(config);
      auto models = HypothesisGenerator(generator_config).generate(
          window, transaction, no_imu, no_bridge);
      DetectorRiskContext detector_risk;
      detector_risk.p_fa_per_test = config.detector.p_fa_per_test;
      detector_risk.rank_tolerance = config.integrity_window.rank_tolerance;
      detector_risk.max_condition_number =
          config.integrity_window.max_condition_number;
      const auto all_in = JointWindowDetector().evaluate(window, detector_risk);
      HypothesisEvaluationConfig evidence_config;
      evidence_config.rank_tolerance = config.integrity_window.rank_tolerance;
      evidence_config.max_condition_number =
          config.integrity_window.max_condition_number;
      const auto evidence = HypothesisEvidenceEvaluator(evidence_config)
          .evaluateAll(window, models.modes, &models.hypotheses,
                       all_in.squared_threshold);
      for (const auto id : completePlausibleHypotheses(
               models.hypotheses, evidence)) {
        actual.plausible.push_back(id.value());
      }
      GeneratedActionSnapshotV1 trusted;
      ActionSearchResultV1 search;
      if (all_in.passed) {
        ExclusionAction keep;
        keep.id = ExclusionActionId(1);
        keep.action_model_id = "KEEP_ALL";
        search = censusAndCapActionsV1({keep}, 1, &trusted);
      } else {
        search = HypothesisGenerator(generator_config)
            .actionsForPlausibleSetV1(window, transaction, &models,
                                      evidence, {}, &trusted);
      }
      actual.generated_action_count = search.census.generated;
      // The production model-qualification gate is closed at this baseline;
      // therefore no recovery action can become the formal winner.
      actual.winner = 0;
    }
    const auto plan = EpochCommitPlan::nominalPlan(transaction);
    estimator.commitEpoch(std::move(transaction), plan);
  }
  actual.note =
      "regression display only; tests derive expected values from the recipe "
      "and never load these fields";
  return actual;
}

void validateResolvedBindingAgainstActual(const OracleManifest& manifest) {
  const IntegrityConfig actual = IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_uwb_order1.yaml");
  if (manifest.fde_profile != toString(actual.fde.profile) ||
      manifest.fault_max_cardinality != actual.fault_models.max_cardinality ||
      manifest.uwb_enabled != actual.fault_models.uwb.enabled ||
      manifest.epoch_single_enabled !=
          actual.fault_models.uwb.epoch_single_anchor_bias ||
      manifest.persistent_enabled !=
          actual.fault_models.uwb.persistent_anchor_bias ||
      manifest.ramp_enabled != actual.fault_models.uwb.ramp_bias ||
      manifest.single_faults_enabled !=
          (actual.resolved_scope.max_fault_order >= 1) ||
      manifest.double_faults_enabled !=
          (actual.resolved_scope.max_fault_order >= 2) ||
      manifest.resolved_max_fault_order !=
          actual.resolved_scope.max_fault_order ||
      manifest.uwb_prior_probability_bound !=
          actual.fault_models.uwb.prior_probability_bound ||
      manifest.uwb_p_md != actual.fault_models.uwb.p_md ||
      manifest.action_trigger != actual.fde.trigger ||
      manifest.action_isolation != actual.fde.isolation ||
      manifest.ambiguity_policy != actual.fde.ambiguity_policy ||
      manifest.dense_oracle_online_fallback !=
          actual.fde.dense_oracle_online_fallback ||
      manifest.max_candidate_count != actual.fde.max_candidate_count ||
      manifest.on_no_valid_action != actual.fde.on_no_valid_action ||
      manifest.on_integrity_model_invalid !=
          actual.fde.on_integrity_model_invalid ||
      manifest.fault_manifest_path != actual.fault_models.manifest_path ||
      !actual.fault_manifest ||
      manifest.fault_manifest_id != actual.fault_manifest->manifest_id) {
    throw std::runtime_error(
        "resolved config declaration differs from loaded production config");
  }
}

enum class LeafClassification { Derived, Fixed, Observed };

struct ExplicitLeafCase {
  std::string path;
  LeafClassification classification = LeafClassification::Fixed;
  TypedSemanticField field = TypedSemanticField::Raw;
  std::string invariant;
  std::string expected_reason;
  std::string mutation_id;
};

std::vector<std::string> splitTabs(const std::string& line) {
  std::vector<std::string> fields;
  std::size_t begin = 0;
  while (true) {
    const std::size_t end = line.find('\t', begin);
    fields.push_back(line.substr(begin, end - begin));
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return fields;
}

TypedSemanticField parseTypedSemanticField(const std::string& value) {
  if (value == "Raw") return TypedSemanticField::Raw;
  if (value == "State") return TypedSemanticField::State;
  if (value == "Ownership") return TypedSemanticField::Ownership;
  if (value == "Served") return TypedSemanticField::Served;
  if (value == "Modes") return TypedSemanticField::Modes;
  if (value == "Actions") return TypedSemanticField::Actions;
  if (value == "Risk") return TypedSemanticField::Risk;
  if (value == "Missing") return TypedSemanticField::Missing;
  if (value == "Tolerance") return TypedSemanticField::Tolerance;
  throw std::runtime_error("unknown typed semantic field: " + value);
}

std::map<std::string, ExplicitLeafCase> loadExplicitLeafCases() {
  const std::string path = std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/test/p0_07_leaf_cases_v1.tsv";
  std::ifstream input(path);
  if (!input) throw std::runtime_error("explicit leaf case table is absent");
  std::string line;
  std::getline(input, line);
  if (line != "path\tclassification\ttyped_field\tinvariant_or_validator\t"
              "expected_reason\tmutation_id") {
    throw std::runtime_error("explicit leaf case table header is invalid");
  }
  std::map<std::string, ExplicitLeafCase> cases;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto fields = splitTabs(line);
    if (fields.size() != 6) {
      throw std::runtime_error("explicit leaf case row has wrong arity");
    }
    ExplicitLeafCase value;
    value.path = fields[0];
    if (fields[1] == "DERIVED") {
      value.classification = LeafClassification::Derived;
      value.field = parseTypedSemanticField(fields[2]);
    } else if (fields[1] == "FIXED") {
      value.classification = LeafClassification::Fixed;
    } else if (fields[1] == "OBSERVED") {
      value.classification = LeafClassification::Observed;
    } else {
      throw std::runtime_error("explicit leaf case class is invalid");
    }
    value.invariant = fields[3];
    value.expected_reason = fields[4];
    value.mutation_id = fields[5];
    if (value.invariant.empty() || value.expected_reason.empty() ||
        value.mutation_id != "mutate_scalar_at_exact_path" ||
        !cases.emplace(value.path, value).second) {
      throw std::runtime_error("duplicate/incomplete explicit leaf case: " +
                               value.path);
    }
  }
  if (cases.size() != 174) {
    throw std::runtime_error("explicit leaf case table must contain 174 rows");
  }
  return cases;
}

YAML::Node nodeAtRecipePath(YAML::Node node, const RecipeLeafPath& path,
                            std::size_t depth = 0) {
  if (depth == path.size()) return node;
  const auto& segment = path[depth];
  return nodeAtRecipePath(
      segment.sequence ? node[segment.index] : node[segment.key], path,
      depth + 1);
}

void enforceFixedLeafInvariant(const ExplicitLeafCase& leaf_case,
                               const YAML::Node& original,
                               const YAML::Node& mutated,
                               const RecipeLeafPath& path) {
  const YAML::Node before = nodeAtRecipePath(original, path);
  const YAML::Node after = nodeAtRecipePath(mutated, path);
  if (YAML::Dump(before) == YAML::Dump(after)) {
    throw std::runtime_error("FIXED_MUTATION_DID_NOT_TARGET_DECLARED_PATH");
  }
  // A FIXED case is a versioned recipe invariant, not an input whose arbitrary
  // mutation is silently reclassified by a catch-all parser exception.
  throw std::runtime_error(leaf_case.expected_reason);
}

std::string observedExpectedReason(const std::string& path) {
  if (path.rfind("observed_non_authoritative.plausible_mode_ordinals[", 0) == 0)
    return "OBSERVED_PLAUSIBLE_MISMATCH";
  if (path == "observed_non_authoritative.generated_action_count")
    return "OBSERVED_ACTION_COUNT_MISMATCH";
  if (path == "observed_non_authoritative.winner")
    return "OBSERVED_WINNER_MISMATCH";
  if (path == "observed_non_authoritative.note")
    return "OBSERVED_NOTE_MISMATCH";
  throw std::runtime_error("unknown observed leaf path");
}

}  // namespace

TEST(P007CorrectedExhaustive,
     SemanticEvaluatorCannotConsumeProductionExpectedArtifacts) {
  std::ifstream input(__FILE__);
  ASSERT_TRUE(input.good());
  const std::string source((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
  const std::string begin_marker =
      "TypedOracleResult evaluateTypedRecipeOracle(";
  const std::string end_marker = "\n}\n\nenum class TypedSemanticField";
  const std::size_t begin = source.find(begin_marker);
  ASSERT_NE(begin, std::string::npos);
  const std::size_t end = source.find(end_marker, begin);
  ASSERT_NE(end, std::string::npos);
  const std::string evaluator = source.substr(begin, end - begin);
  for (const char* forbidden : {
           "IntegrityWindow", "HypothesisGenerator", "FaultModelSet",
           "ActionSearchResult", "GeneratedActionSnapshot", "auditRaw",
           "frozen-output", "production_dense", "models.", "search."}) {
    EXPECT_EQ(evaluator.find(forbidden), std::string::npos) << forbidden;
  }
}

TEST(P007CorrectedExhaustive, ThreeLayerRawDenseOnlineEquivalence) {
  const RawReplay replay = makeReplay();
  ASSERT_EQ(std::set<std::uint64_t>(replay.row_ids.begin(),
                                    replay.row_ids.end()).size(),
            replay.row_ids.size());
  ASSERT_GT((replay.covariance -
             Eigen::MatrixXd(replay.covariance.diagonal().asDiagonal())).norm(),
            0.1);
  ASSERT_GT(replay.z.norm(), 0.0);
  const Eigen::Matrix3d protected_map =
      (Eigen::Matrix3d() << 1.0, 0.2, 0.0,
                            0.0, 0.9, -0.1,
                            0.1, 0.0, 1.1).finished();
  const std::vector<std::set<std::uint64_t>> actions = {
      {}, {10}, {20}, {10, 20}, {30}, {40}};
  std::vector<double> dense_statistics;
  std::vector<double> online_statistics;
  for (const auto& action : actions) {
    const DenseResult dense = denseRawOracle(replay, action, protected_map);
    ASSERT_EQ(dense.rank, 3);
    const auto online = FrozenSquareRootContext::build(
        dense.h, dense.z, protected_map, ColumnScalePolicy::ColumnNorm,
        ColumnPermutationPolicy::ColumnPivot, 1e-12, dense.state,
        dense.statistic);
    ASSERT_TRUE(online->usable()) << online->reason();
    EXPECT_EQ(online->rank(), dense.rank);
    EXPECT_EQ(online->dof(), dense.dof);
    EXPECT_NEAR(online->statistic(), dense.statistic,
                1e-10 * std::max(1.0, dense.statistic));
    EXPECT_LT(relativeError(online->baseStateIncrement(), dense.state), 1e-10);
    const Eigen::MatrixXd online_covariance = online->informationSolve(
        Eigen::MatrixXd::Identity(dense.h.cols(), dense.h.cols()));
    EXPECT_LT(relativeError(online_covariance, dense.covariance), 1e-10);
    Eigen::MatrixXd y;
    Eigen::MatrixXd z;
    online->faultResponse(dense.fault, &y, &z);
    EXPECT_LT(relativeError(z.transpose() * z, dense.gram), 1e-10);
    EXPECT_LT(relativeError(
                  protected_map * online->leastSquaresSolve(dense.fault),
                  dense.response),
              1e-10);
    dense_statistics.push_back(dense.statistic);
    online_statistics.push_back(online->statistic());
  }
  const auto dense_winner = std::min_element(dense_statistics.begin(),
                                             dense_statistics.end());
  const auto online_winner = std::min_element(online_statistics.begin(),
                                              online_statistics.end());
  EXPECT_EQ(std::distance(dense_statistics.begin(), dense_winner),
            std::distance(online_statistics.begin(), online_winner));
  std::vector<double> reordered = online_statistics;
  std::reverse(reordered.begin(), reordered.end());
  EXPECT_NEAR(*std::min_element(reordered.begin(), reordered.end()),
              *online_winner, 0.0);
  std::cout << "[P0-07-REPLAY] raw_rows=" << replay.row_ids.size()
            << " actions=" << actions.size()
            << " replay_hash=" << replayHash(replay)
            << " winner="
            << std::distance(online_statistics.begin(), online_winner)
            << std::endl;
}

TEST(P007CorrectedExhaustive, TrapezoidCorrelationRequiresQualification) {
  constexpr int intervals = 6;
  constexpr double sigma = 0.2;
  Eigen::MatrixXd exact = Eigen::MatrixXd::Zero(intervals, intervals);
  exact.diagonal().setConstant(0.5 * sigma * sigma);
  for (int i = 0; i + 1 < intervals; ++i) {
    exact(i, i + 1) = exact(i + 1, i) = 0.25 * sigma * sigma;
  }
  const Eigen::MatrixXd conservative =
      Eigen::MatrixXd::Identity(intervals, intervals) * sigma * sigma;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> bound(conservative - exact);
  ASSERT_EQ(bound.info(), Eigen::Success);
  EXPECT_GE(bound.eigenvalues().minCoeff(), -1e-15);

  const IntegrityConfig config = IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_joint_order2.yaml");
  const auto unqualified = assessImuNoiseQualificationV1(config.imu);
  EXPECT_FALSE(unqualified.formal_eligible);
  EXPECT_FALSE(unqualified.sigma_semantics_declared);
  EXPECT_FALSE(unqualified.shared_sample_correlation_covered);
  EXPECT_FALSE(unqualified.calibration_bound);
  EXPECT_NE(unqualified.reason.find("IMU_MODEL_UNQUALIFIED"),
            std::string::npos);

  ImuNoiseConfig claimed = config.imu;
  claimed.noise_overbound_calibration_id = "arbitrary-self-declared-id";
  const auto rejected_claim = assessImuNoiseQualificationV1(claimed);
  EXPECT_FALSE(rejected_claim.formal_eligible)
      << "a free-form id must never substitute for an authenticated artifact";
  EXPECT_FALSE(rejected_claim.calibration_bound);
  EXPECT_NE(rejected_claim.reason.find("no authenticated V1 calibration artifact"),
            std::string::npos);

  const std::array<const char*, 6> profiles = {
      "realtime_uwb_imu_pl_research.yaml", "fde_off.yaml",
      "fde_uwb_order1.yaml", "fde_imu_order1.yaml",
      "fde_joint_order1.yaml", "fde_joint_order2.yaml"};
  for (const char* profile : profiles) {
    const IntegrityConfig loaded = IntegrityConfigLoader::load(
        std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/" + profile);
    EXPECT_FALSE(assessImuNoiseQualificationV1(loaded.imu).formal_eligible)
        << profile;
  }

  // Compatibility replay: remove both P0-07 documentation-only declarations
  // from a real v6 profile.  The old profile must load and default to the same
  // fail-closed state rather than returning a schema error.
  const std::string source_path =
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_off.yaml";
  std::ifstream source(source_path);
  ASSERT_TRUE(source.good());
  const std::string old_path =
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/.p007-old-profile.yaml";
  std::ofstream old_profile(old_path, std::ios::trunc);
  ASSERT_TRUE(old_profile.good());
  std::string line;
  while (std::getline(source, line)) {
    if (line.find("sigma_semantics:") != std::string::npos ||
        line.find("trapezoid_correlation_model:") != std::string::npos) {
      continue;
    }
    old_profile << line << '\n';
  }
  old_profile.close();
  const IntegrityConfig old_loaded = IntegrityConfigLoader::load(old_path);
  EXPECT_FALSE(assessImuNoiseQualificationV1(old_loaded.imu).formal_eligible);
  EXPECT_EQ(std::remove(old_path.c_str()), 0);

  const std::string claim_path =
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/.p007-self-claim.yaml";
  source.clear();
  source.seekg(0);
  std::ofstream claim_profile(claim_path, std::ios::trunc);
  ASSERT_TRUE(claim_profile.good());
  while (std::getline(source, line)) {
    if (line.find("sigma_semantics:") != std::string::npos) {
      line = "  sigma_semantics: CALIBRATED_EFFECTIVE_INTERVAL_OVERBOUND";
    } else if (line.find("trapezoid_correlation_model:") !=
               std::string::npos) {
      line = "  trapezoid_correlation_model: "
             "CALIBRATED_CONSERVATIVE_INDEPENDENT_OVERBOUND";
    } else if (line.find("noise_overbound_calibration_id:") !=
               std::string::npos) {
      line = "  noise_overbound_calibration_id: arbitrary-self-claim";
    }
    claim_profile << line << '\n';
  }
  claim_profile.close();
  const IntegrityConfig self_claimed = IntegrityConfigLoader::load(claim_path);
  EXPECT_FALSE(assessImuNoiseQualificationV1(self_claimed.imu).formal_eligible)
      << "profile strings and a free-form id must not open N01";
  EXPECT_EQ(std::remove(claim_path.c_str()), 0);
}

TEST(P007CorrectedExhaustive, GoldenPublicConfigAbiLayoutIsPreserved) {
  EXPECT_EQ(sizeof(ImuNoiseConfig), 80u);
  EXPECT_EQ(alignof(ImuNoiseConfig), 8u);
  EXPECT_EQ(sizeof(IntegrityConfig), 2544u);
  EXPECT_EQ(alignof(IntegrityConfig), 8u);

  // The immutable diagnostic sidecar is keyed by both estimator lifetime and
  // frozen-values identity.  Reusing the exact object address cannot make a
  // transaction from the prior lifetime resolve to the new certificate.
  IntegrityConfig config = IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_off.yaml");
  config.incremental.fixed_lag_epochs = 0;
  alignas(IncrementalUwbImuEstimator)
      unsigned char storage[sizeof(IncrementalUwbImuEstimator)];
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  ImuMeasurement boundary;
  boundary.id = MeasurementId(980000);
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};

  auto* first = new (storage)
      IncrementalUwbImuEstimator(config, Eigen::Vector3d::Zero());
  first->initialize(initial, config.realtime.prior_sigmas);
  first->ingestImu(boundary);
  appendProductionReplayImu(first, config, 1, 2, 10000000);
  EpochTransaction stale = first->prepareEpoch(
      makeProductionReplayBatch(config, 1, 999, 10000000),
      EpochPreparationOptions{true, true, false, true});
  EXPECT_FALSE(first->auditRawRowOwnershipV1(stale).empty());
  first->~IncrementalUwbImuEstimator();

  auto* second = new (storage)
      IncrementalUwbImuEstimator(config, Eigen::Vector3d::Zero());
  second->initialize(initial, config.realtime.prior_sigmas);
  second->ingestImu(boundary);
  appendProductionReplayImu(second, config, 1, 2, 10000000);
  EpochTransaction fresh = second->prepareEpoch(
      makeProductionReplayBatch(config, 1, 999, 10000000),
      EpochPreparationOptions{true, true, false, true});
  EXPECT_THROW(second->auditRawRowOwnershipV1(stale), std::runtime_error);
  EXPECT_FALSE(second->auditRawRowOwnershipV1(fresh).empty());
  second->~IncrementalUwbImuEstimator();
}

TEST(P007CorrectedExhaustive, FrozenProductionEvidenceIsMechanicallyBound) {
  const OracleManifest recipe = loadOracleManifest();
  const std::string root = std::string(UWB_IMU_PL_SOURCE_DIR) +
      "/docs/evidence/p0-07-corrected-exhaustive/authoritative-replay/";
  const YAML::Node input = YAML::LoadFile(root + "frozen-input.json");
  const YAML::Node truth = YAML::LoadFile(root + "frozen-truth.json");
  const YAML::Node output = YAML::LoadFile(root + "frozen-output.json");
  const YAML::Node summary = YAML::LoadFile(root + "o12-correctness-smoke.json");
  ASSERT_EQ(input["schema"].as<std::string>(),
            "p0-07-production-raw-replay-v3");
  EXPECT_EQ(input["epochs"].as<std::size_t>(), recipe.replay_epochs);
  ASSERT_EQ(input["uwb_rows"].size(),
            recipe.replay_epochs * recipe.uwb_rows);
  EXPECT_EQ(truth["fault"]["epoch"].as<std::size_t>(), recipe.alarm_epoch);
  EXPECT_EQ(truth["fault"]["anchor_id"].as<std::uint64_t>(),
            recipe.anchors.front());
  EXPECT_DOUBLE_EQ(truth["fault"]["bias_m"].as<double>(), 2.0);
  EXPECT_GT(output["alarm"]["generated"].as<std::uint64_t>(), 1u);
  EXPECT_EQ(output["alarm"]["generated"].as<std::uint64_t>(),
            output["alarm"]["kernel_evaluated"].as<std::uint64_t>());
  ASSERT_TRUE(output["alarm_actions"].IsSequence());
  EXPECT_EQ(output["alarm_actions"].size(),
            output["alarm"]["generated"].as<std::size_t>());
  for (const auto& action : output["alarm_actions"]) {
    EXPECT_EQ(action["kernel_evaluated"].as<int>(), 1);
    EXPECT_NE(action["terminal"].as<std::string>(), "PLANNED_NOT_RUN");
  }
  EXPECT_FALSE(output["formal_eligible"].as<bool>());
  EXPECT_FALSE(output["protected_output"].as<bool>());
  EXPECT_EQ(summary["denominator"]["received"].as<std::size_t>(),
            recipe.replay_epochs);
  EXPECT_EQ(summary["denominator"]["attempted"].as<std::size_t>(),
            recipe.replay_epochs);
  EXPECT_EQ(summary["denominator"]["excluded"].as<int>(), 0);
  EXPECT_EQ(summary["complete_work_attempts"].as<std::size_t>(),
            recipe.replay_epochs);

  std::ifstream attempts(root + "input-attempts.tsv");
  ASSERT_TRUE(attempts.good());
  std::string line;
  ASSERT_TRUE(static_cast<bool>(std::getline(attempts, line)));
  EXPECT_NE(line.find("core_compute_ms\tanalysis_completion_ms\t"
                      "arrival_to_packet_ready_ms"), std::string::npos);
  std::size_t rows = 0;
  bool saw_distinct_boundaries = false;
  while (std::getline(attempts, line)) {
    if (line.empty()) continue;
    ++rows;
    std::vector<std::string> fields;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, '\t')) fields.push_back(field);
    ASSERT_EQ(fields.size(), 16u);
    const double core = std::stod(fields[10]);
    const double analysis = std::stod(fields[11]);
    const double arrival = std::stod(fields[12]);
    EXPECT_LE(core, analysis);
    EXPECT_LE(analysis, arrival);
    saw_distinct_boundaries = saw_distinct_boundaries ||
        core != analysis || analysis != arrival;
  }
  EXPECT_EQ(rows, 24u);
  EXPECT_TRUE(saw_distinct_boundaries);
}

TEST(P007CorrectedExhaustive,
     SameRawReplayDrivesProductionBuilderGeneratorAndFinalRefusal) {
  struct AuditCaptureGuard {
    AuditCaptureGuard() { setActionHypothesisProofAuditEnabledV1(true); }
    ~AuditCaptureGuard() { setActionHypothesisProofAuditEnabledV1(false); }
  } audit_capture_guard;
  OracleManifest oracle = loadOracleManifest();
  IntegrityConfig config = IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_uwb_order1.yaml");
  verifyRecipeConfigBinding(oracle, config);
  config.incremental.fixed_lag_epochs =
      oracle.alarm_epoch - oracle.physical_first_epoch + 1;
  config.publication.protected_output_enabled = true;
  config.publication.deadline_ms = 1e9;

  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary;
  boundary.id = MeasurementId(900000);
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary);

  std::vector<std::uint64_t> crossing_counts;
  std::size_t final_hypotheses = 0;
  std::size_t final_actions = 0;
  bool final_all_in_passed = false;
  std::size_t alarm_hypotheses = 0;
  std::size_t alarm_actions = 0;
  std::size_t alarm_expected_modes = 0;
  std::size_t alarm_represented_modes = 0;
  std::size_t alarm_evaluated_modes = 0;
  std::size_t alarm_expected_pairs = 0;
  std::size_t alarm_represented_pairs = 0;
  std::size_t alarm_evaluated_pairs = 0;
  bool alarm_all_in_passed = true;
  std::set<std::string> classification_comparisons;
  std::vector<DenseActionReference> action_references;
  CompleteRawReference alarm_raw_reference;
  bool ownership_snapshot_concurrency_verified = false;
  for (std::size_t epoch = 1; epoch <= oracle.replay_epochs; ++epoch) {
    appendProductionReplayImu(&estimator, config, epoch,
                              oracle.imu_samples_per_epoch,
                              oracle.epoch_period_ns);
    EpochPreparationOptions options;
    options.build_integrity_material = true;
    options.build_uwb_recovery_material = true;
    options.build_imu_recovery_material = false;
    options.build_recoverable_history = true;
    auto transaction = estimator.prepareEpoch(
        makeProductionReplayBatch(config, epoch, oracle.alarm_epoch, oracle.epoch_period_ns), options);
    if (epoch + 1 >= oracle.alarm_epoch) {
      const auto window = estimator.buildIntegrityWindow(
          transaction, IntegrityWindowRequest{config.integrity_window.epochs});
      ASSERT_TRUE(window.model_valid) << window.reason;
      ASSERT_TRUE(window.history_summary.valid) << window.history_summary.reason;
      if (epoch + 1 >= oracle.alarm_epoch) {
        crossing_counts.push_back(estimator.audit().marginalization_count);
      }

      ImuFaultSubspaces no_imu;
      LinearizedFactorBlock no_bridge;
      const auto generator_config = productionGeneratorConfig(config);
      auto models = HypothesisGenerator(generator_config).generate(
          window, transaction, no_imu, no_bridge);
      if (epoch == oracle.alarm_epoch) {
        ASSERT_EQ(epoch, oracle.alarm_epoch);
        const std::size_t explicit_rows =
            oracle.served_rows - oracle.summary_rows;
        ASSERT_EQ(window.H.rows(), static_cast<Eigen::Index>(
            window.history_summary.emitted_rows + explicit_rows));
        ASSERT_EQ(window.history_summary.boundary_rows, oracle.history_rows);
        ASSERT_EQ(window.history_summary.nu_perp,
                  static_cast<int>(window.history_summary.emitted_rows -
                      window.history_summary.response.rows()));
        std::vector<std::string> oracle_mode_identities;
        for (const auto anchor : oracle.anchors) {
          for (std::size_t onset = oracle.onset_first;
               onset <= oracle.onset_last; ++onset) {
            for (const auto& family : oracle.families) {
              std::ostringstream identity;
              identity << family.kind << ":uwb:" << anchor << ':' << onset
                       << ':' << family.dimension;
              oracle_mode_identities.push_back(identity.str());
            }
          }
        }
        ASSERT_EQ(oracle_mode_identities.size(), oracle.expected_modes);
        ASSERT_EQ(models.modes.size(), oracle.expected_modes);
        ASSERT_EQ(models.hypotheses.size(), oracle.expected_hypotheses);
        std::set<std::string> oracle_identity_set(
            oracle_mode_identities.begin(), oracle_mode_identities.end());
        const std::set<std::string> production_identity_set(
            models.expected_mode_identities.begin(),
            models.expected_mode_identities.end());
        std::vector<std::string> missing_from_production;
        std::vector<std::string> missing_from_recipe;
        std::set_difference(oracle_identity_set.begin(), oracle_identity_set.end(),
                            production_identity_set.begin(),
                            production_identity_set.end(),
                            std::back_inserter(missing_from_production));
        std::set_difference(production_identity_set.begin(),
                            production_identity_set.end(),
                            oracle_identity_set.begin(), oracle_identity_set.end(),
                            std::back_inserter(missing_from_recipe));
        EXPECT_TRUE(missing_from_production.empty())
            << "first=" << (missing_from_production.empty()
                                 ? std::string("none")
                                 : missing_from_production.front());
        EXPECT_TRUE(missing_from_recipe.empty())
            << "first=" << (missing_from_recipe.empty()
                                 ? std::string("none")
                                 : missing_from_recipe.front());
        for (std::size_t index = 0; index < models.modes.size(); ++index) {
          EXPECT_EQ(models.modes[index].id.value(), index + 1);
          EXPECT_EQ(models.hypotheses[index].id.value(), index + 1);
          EXPECT_EQ(models.hypotheses[index].modes,
                    std::vector<FaultModeId>{FaultModeId(index + 1)});
        }
        alarm_raw_reference = verifyCompleteRawOracle(
            transaction, window, config.integrity_window.epochs, oracle,
            estimator.auditRawRowOwnershipV1(transaction));
        verifyRawUwbOracle(makeProductionReplayBatch(
                               config, epoch, oracle.alarm_epoch,
                               oracle.epoch_period_ns),
                           transaction, window, models, oracle);
      }
      EXPECT_EQ(models.expected_mode_identities,
                models.represented_mode_identities);
      EXPECT_EQ(models.expected_mode_identities,
                models.evaluated_mode_identities);
      EXPECT_EQ(models.expected_pair_identities,
                models.represented_pair_identities);
      EXPECT_EQ(models.expected_pair_identities,
                models.evaluated_pair_identities);

      DetectorRiskContext detector_risk;
      detector_risk.p_fa_per_test = config.detector.p_fa_per_test;
      detector_risk.rank_tolerance = config.integrity_window.rank_tolerance;
      detector_risk.max_condition_number =
          config.integrity_window.max_condition_number;
      const auto all_in = JointWindowDetector().evaluate(window, detector_risk);
      ASSERT_TRUE(all_in.numerically_valid) << all_in.reason;
      HypothesisEvaluationConfig evidence_config;
      evidence_config.rank_tolerance = config.integrity_window.rank_tolerance;
      evidence_config.max_condition_number =
          config.integrity_window.max_condition_number;
      std::shared_ptr<const FrozenHypothesisNumerics> shared_hypothesis_numerics;
      const auto evidence = HypothesisEvidenceEvaluator(evidence_config)
          .evaluateAll(window, models.modes, &models.hypotheses,
                       all_in.squared_threshold, &shared_hypothesis_numerics);
      if (epoch == oracle.alarm_epoch) {
        std::vector<std::uint64_t> actual_plausible;
        for (const auto id : completePlausibleHypotheses(
                 models.hypotheses, evidence)) {
          actual_plausible.push_back(id.value());
        }
        ASSERT_TRUE(static_cast<bool>(shared_hypothesis_numerics));
        ASSERT_TRUE(shared_hypothesis_numerics->valid)
            << shared_hypothesis_numerics->reason;
        verifyEveryHypothesisNumerics(window, models, evidence,
                                      *shared_hypothesis_numerics, oracle,
                                      alarm_raw_reference,
                                      config.integrity_window.rank_tolerance,
                                      &oracle.plausible_modes);
        classification_comparisons.insert("profile_valid");
        classification_comparisons.insert("pl_valid");
        classification_comparisons.insert("nullspace_class");
        classification_comparisons.insert("finite_or_infinite");
        EXPECT_EQ(actual_plausible, oracle.plausible_modes);
        // Preserve and explain the complete authorized golden-old ->
        // corrected-new discrete diff.  The old observed list remains frozen
        // input evidence; the corrected list comes from the independent raw
        // oracle above.  Exactly mode 57 crosses because the unchanged chi2
        // formula consumes effective dof instead of raw storage-row dof.
        std::vector<std::uint64_t> removed_from_old;
        std::vector<std::uint64_t> added_to_new;
        std::set_difference(oracle.observed_plausible_modes.begin(),
                            oracle.observed_plausible_modes.end(),
                            actual_plausible.begin(), actual_plausible.end(),
                            std::back_inserter(removed_from_old));
        std::set_difference(actual_plausible.begin(), actual_plausible.end(),
                            oracle.observed_plausible_modes.begin(),
                            oracle.observed_plausible_modes.end(),
                            std::back_inserter(added_to_new));
        EXPECT_EQ(removed_from_old, std::vector<std::uint64_t>{57});
        EXPECT_TRUE(added_to_new.empty());
        const auto mode57 = std::find_if(
            evidence.begin(), evidence.end(), [](const FaultModeEvidence& item) {
              return item.hypothesis.value() == 57;
            });
        ASSERT_NE(mode57, evidence.end());
        const int raw_residual_dof = static_cast<int>(
            oracle.summary_rows - window.history_summary.response.rows());
        const int removed_rank_zero =
            raw_residual_dof - window.history_summary.nu_perp;
        ASSERT_GT(removed_rank_zero, 0);
        const int legacy_raw_row_dof = all_in.dof + removed_rank_zero;
        const double legacy_threshold =
            StatisticalBoundsCache::chiSquaredThreshold(
                legacy_raw_row_dof, config.detector.p_fa_per_test);
        EXPECT_EQ(all_in.dof, window.H.rows() - window.rank);
        EXPECT_LT(all_in.squared_threshold, legacy_threshold);
        EXPECT_GT(mode57->conditioned_statistic,
                  all_in.squared_threshold +
                      evidence_config.plausible_conditioned_statistic_margin);
        EXPECT_LE(mode57->conditioned_statistic,
                  legacy_threshold +
                      evidence_config.plausible_conditioned_statistic_margin);
        EXPECT_FALSE(mode57->plausible);
        oracle.operations = deriveOracleOperations(oracle.plausible_modes,
                                                   oracle);
        EXPECT_EQ(oracle.operations.size() + 1,
                  oracle.observed_generated_action_count);
        std::cout << "[P1-03-AUTHORIZED-DIFF] rows=" << oracle.served_rows
                  << "->" << window.H.rows() << " dof="
                  << legacy_raw_row_dof << "->" << all_in.dof
                  << " nu_perp=" << raw_residual_dof
                  << "->" << window.history_summary.nu_perp
                  << " threshold=" << legacy_threshold << "->"
                  << all_in.squared_threshold << " mode57_stat="
                  << mode57->conditioned_statistic
                  << " plausible_removed=57 actions="
                  << oracle.observed_generated_action_count << "->"
                  << oracle.operations.size() << '\n';
        oracle.action_risk = oracle.total_risk /
            static_cast<double>(oracle.operations.size());
      }
      GeneratedActionSnapshotV1 trusted;
      ActionSearchResultV1 search;
      if (all_in.passed) {
        ExclusionAction keep;
        keep.id = ExclusionActionId(1);
        keep.action_model_id = "KEEP_ALL";
        search = censusAndCapActionsV1({keep}, 1, &trusted);
      } else {
        search = HypothesisGenerator(generator_config).actionsForPlausibleSetV1(
            window, transaction, &models, evidence, {}, &trusted);
      }
      EXPECT_TRUE(search.census.exhaustive);
      EXPECT_EQ(search.census.generated, search.census.evaluated);
      EXPECT_EQ(search.census.omitted, 0u);
      if (epoch == oracle.alarm_epoch) {
        alarm_hypotheses = models.hypotheses.size();
        alarm_actions = search.census.generated;
        alarm_all_in_passed = all_in.passed;
        alarm_expected_modes = models.expected_mode_identities.size();
        alarm_represented_modes = models.represented_mode_identities.size();
        alarm_evaluated_modes = models.evaluated_mode_identities.size();
        alarm_expected_pairs = models.expected_pair_identities.size();
        alarm_represented_pairs = models.represented_pair_identities.size();
        alarm_evaluated_pairs = models.evaluated_pair_identities.size();
        const auto& raw_actions = search.generated_snapshot.actions;
        // Missing-provenance modes are part of the frozen physical census even
        // when they are not plausible at this alarm.  Materialization must
        // preserve a fail-closed action for every one; it must never erase the
        // mode/action to make coverage appear easier.  The independently
        // expected terminal follows solely from the raw provenance ledger.
        std::size_t missing_modes = 0;
        std::size_t missing_actions = 0;
        for (const auto& mode : models.modes) {
          if (mode.recoverability != HistoryRecoverability::MissingProvenance) {
            continue;
          }
          ++missing_modes;
          const bool represented = std::any_of(
              models.hypotheses.begin(), models.hypotheses.end(),
              [&](const FaultHypothesisV2& hypothesis) {
                return std::find(hypothesis.modes.begin(),
                                 hypothesis.modes.end(), mode.id) !=
                    hypothesis.modes.end();
              });
          EXPECT_TRUE(represented) << mode.id.value();
          const auto action = std::find_if(
              models.single_mode_actions.begin(),
              models.single_mode_actions.end(),
              [&](const ExclusionAction& candidate) {
                return candidate.covered_modes.size() == 1 &&
                    candidate.covered_modes.front() == mode.id;
              });
          ASSERT_NE(action, models.single_mode_actions.end()) << mode.id.value();
          EXPECT_EQ(action->recoverability,
                    HistoryRecoverability::MissingProvenance);
          ++missing_actions;
        }
        EXPECT_EQ(missing_actions, missing_modes);
        EXPECT_EQ(models.single_mode_actions.size(), models.modes.size());
        std::cout << "[P0-07-MISSING-PROVENANCE] modes=" << missing_modes
                  << " actions=" << missing_actions
                  << " same_replay_expected_terminal="
                  << (missing_modes == 0 ? "NOT_APPLICABLE" : "UNRECOVERABLE")
                  << " dedicated_reference=MissingProvenanceCensusIsRetainedAndRefused\n";
        ASSERT_EQ(raw_actions.size(), oracle.operations.size());
        for (std::size_t i = 0; i < raw_actions.size(); ++i) {
          const auto& expected = oracle.operations[i];
          EXPECT_EQ(raw_actions[i].id.value(), expected.id);
          EXPECT_EQ(joinedGroups(raw_actions[i].groups_to_remove),
                    joinedGroups([&] {
                      std::vector<FactorGroupId> ids;
                      for (const auto id : expected.remove) ids.emplace_back(id);
                      return ids;
                    }()));
          EXPECT_EQ(joinedGroups(raw_actions[i].groups_to_add),
                    joinedGroups([&] {
                      std::vector<FactorGroupId> ids;
                      for (const auto id : expected.add) ids.emplace_back(id);
                      return ids;
                    }()));
          std::vector<std::uint64_t> actual_covered;
          for (const auto id : raw_actions[i].covered_modes) {
            actual_covered.push_back(id.value());
          }
          EXPECT_EQ(actual_covered, expected.covered_modes);
        }
        const auto expected_hypotheses = oracleHypotheses(oracle);
        ASSERT_EQ(expected_hypotheses.size(), models.hypotheses.size());
        for (std::size_t i = 0; i < expected_hypotheses.size(); ++i) {
          EXPECT_EQ(expected_hypotheses[i].id,
                    models.hypotheses[i].id.value());
          EXPECT_DOUBLE_EQ(expected_hypotheses[i].prior_probability_bound,
                           models.hypotheses[i].prior_probability_bound);
          EXPECT_DOUBLE_EQ(expected_hypotheses[i].p_md_allocation,
                           models.hypotheses[i].p_md_allocation);
          EXPECT_DOUBLE_EQ(expected_hypotheses[i].hmi_allocation,
                           models.hypotheses[i].hmi_allocation);
        }
        action_references = independentDenseActionReferences(
            alarm_raw_reference, oracle, expected_hypotheses,
            config.integrity_window.rank_tolerance);
        RankUpdateConfig exact_config;
        exact_config.rank_tolerance = config.integrity_window.rank_tolerance;
        exact_config.max_condition_number =
            config.integrity_window.max_condition_number;
        exact_config.max_linearization_step_norm = 100.0;
        exact_config.materialize_dense_oracle_fields = true;
        const DenseCandidateOracle production_dense(exact_config);
        for (std::size_t action_index = 0;
             action_index < raw_actions.size(); ++action_index) {
          const auto& reference = action_references.at(action_index);
          auto candidate = production_dense.evaluate(window,
                                                      raw_actions[action_index]);
          const auto post = JointWindowDetector().evaluateCandidate(
              window, candidate, detector_risk);
          EXPECT_EQ(candidate.rank, reference.rank);
          EXPECT_EQ(candidate.dof, reference.dof);
          EXPECT_NEAR(candidate.statistic, reference.statistic,
                      2e-8 * std::max(1.0, std::abs(reference.statistic)));
          EXPECT_EQ(post.passed, reference.post_passed);
          if (!candidate.valid || !post.numerically_valid || !post.passed) {
            continue;
          }
          std::vector<FaultHypothesisV2> remaining;
          std::vector<std::size_t> remaining_reference_indices;
          for (std::size_t mode_index = 0;
               mode_index < oracle.plausible_modes.size(); ++mode_index) {
            const std::uint64_t mode_id = oracle.plausible_modes[mode_index];
            if (std::find(oracle.operations[action_index].covered_modes.begin(),
                          oracle.operations[action_index].covered_modes.end(),
                          mode_id) !=
                oracle.operations[action_index].covered_modes.end()) {
              continue;
            }
            const auto source = std::find_if(
                models.hypotheses.begin(), models.hypotheses.end(),
                [&](const FaultHypothesisV2& value) {
                  return value.id.value() == mode_id;
                });
            ASSERT_NE(source, models.hypotheses.end());
            remaining.push_back(*source);
            remaining.back().A = reference.candidate_fault_maps[mode_index];
            remaining_reference_indices.push_back(mode_index);
          }
          const auto production_pl = ProtectionLevelV2().compute(
              window, candidate, post, &remaining, config.risk_v2);
          EXPECT_EQ(production_pl.model_valid,
                    reference.mathematical_pl_finite);
          EXPECT_EQ(production_pl.risk_budget_valid, reference.risk_valid);
          classification_comparisons.insert("risk_valid");
          if (!production_pl.model_valid) {
            EXPECT_FALSE(reference.mathematical_pl_finite);
            EXPECT_TRUE(std::isinf(reference.mathematical_hpl_m) ||
                        std::isinf(reference.mathematical_vpl_m) ||
                        !std::all_of(reference.nullspace_classes.begin(),
                                     reference.nullspace_classes.end(),
                                     [](int value) {
                                       return value != static_cast<int>(
                                           GramNullspaceClass::Dangerous);
                                     }));
            continue;
          }
          if (production_pl.model_valid && reference.mathematical_pl_finite) {
            EXPECT_TRUE(production_pl.pl_xyz_m.isApprox(
                reference.mathematical_pl_xyz, 2e-7));
            EXPECT_NEAR(production_pl.hpl_m, reference.mathematical_hpl_m,
                        2e-7 * std::max(1.0,
                            std::abs(reference.mathematical_hpl_m)));
            EXPECT_NEAR(production_pl.vpl_m, reference.mathematical_vpl_m,
                        2e-7 * std::max(1.0,
                            std::abs(reference.mathematical_vpl_m)));
          }
          ProtectionLevelV2ProofV1 production_proof;
          ASSERT_TRUE(protectionLevelV2Proof(production_pl,
                                             &production_proof))
              << "action_index=" << action_index
              << " action_id=" << reference.id
              << " reason=" << production_pl.reason;
          ASSERT_EQ(production_proof.hypothesis_proofs.size(),
                    remaining_reference_indices.size());
          for (std::size_t proof_index = 0;
               proof_index < remaining_reference_indices.size(); ++proof_index) {
            const std::size_t expected_index =
                remaining_reference_indices[proof_index];
            const auto& proof = production_proof.hypothesis_proofs[proof_index];
            EXPECT_TRUE(proof.certified_gram.isApprox(
                reference.candidate_grams[expected_index], 2e-7));
            EXPECT_TRUE(proof.protected_response.isApprox(
                reference.protected_responses[expected_index], 2e-7));
            EXPECT_EQ(proof.nullspace_class,
                      reference.gram_nullspace_classes[expected_index]);
            EXPECT_TRUE(proof.served_entry.protected_slopes.isApprox(
                reference.gram_protected_slopes[expected_index], 2e-7));
            ASSERT_LT(proof_index,
                      production_proof.dual_channel_proofs.size());
            EXPECT_TRUE(production_proof.dual_channel_proofs[proof_index]
                            .served_result.axis_bound_m.isApprox(
                                reference.protected_slopes[expected_index],
                                2e-7));
          }
        }
      }
      if (epoch == oracle.replay_epochs) {
        final_hypotheses = models.hypotheses.size();
        final_actions = search.census.generated;
        final_all_in_passed = all_in.passed;
      }
    }
    const auto plan = EpochCommitPlan::nominalPlan(transaction);
    if (epoch == oracle.alarm_epoch) {
      // Audit a copy while commit mutates the estimator.  The audit must read
      // only the prepare-time immutable certificate, and that same certificate
      // must remain available byte-for-byte after commit.
      const EpochTransaction audit_copy = transaction;
      const auto before = estimator.auditRawRowOwnershipV1(audit_copy);
      const auto serialize_owners = [](const auto& owners) {
        std::ostringstream out;
        for (const auto& owner : owners) {
          out << owner.row_id << '|' << owner.owner_group.value() << '|'
              << owner.row_in_group << '|' << owner.covariance_placement
              << '\n';
        }
        return out.str();
      };
      const std::string before_bytes = serialize_owners(before);
      std::atomic<bool> reader_ready{false};
      std::atomic<bool> stop_reader{false};
      std::atomic<bool> reader_ok{true};
      std::atomic<std::size_t> read_count{0};
      std::thread reader([&] {
        try {
          while (!stop_reader.load(std::memory_order_acquire)) {
            const auto observed =
                estimator.auditRawRowOwnershipV1(audit_copy);
            if (serialize_owners(observed) != before_bytes) {
              reader_ok.store(false, std::memory_order_release);
            }
            read_count.fetch_add(1, std::memory_order_relaxed);
            reader_ready.store(true, std::memory_order_release);
          }
        } catch (...) {
          reader_ok.store(false, std::memory_order_release);
          reader_ready.store(true, std::memory_order_release);
        }
      });
      while (!reader_ready.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      estimator.commitEpoch(std::move(transaction), plan);
      stop_reader.store(true, std::memory_order_release);
      reader.join();
      EXPECT_TRUE(reader_ok.load(std::memory_order_acquire));
      EXPECT_GT(read_count.load(std::memory_order_relaxed), 0u);
      EXPECT_EQ(serialize_owners(
                    estimator.auditRawRowOwnershipV1(audit_copy)),
                before_bytes);
      // A transaction copy shares the immutable frozen identity and must
      // resolve the identical certificate.
      const EpochTransaction second_copy = audit_copy;
      EXPECT_EQ(serialize_owners(
                    estimator.auditRawRowOwnershipV1(second_copy)),
                before_bytes);
      ownership_snapshot_concurrency_verified = true;
    } else {
      estimator.commitEpoch(std::move(transaction), plan);
    }
  }
  ASSERT_EQ(crossing_counts.size(), 3u);
  EXPECT_LT(crossing_counts[0], crossing_counts[1]);
  EXPECT_LT(crossing_counts[1], crossing_counts[2]);
  EXPECT_GT(final_hypotheses, 0u);
  EXPECT_GT(final_actions, 0u);
  EXPECT_FALSE(alarm_all_in_passed);
  EXPECT_GT(alarm_hypotheses, 0u);
  EXPECT_GT(alarm_actions, 1u);
  EXPECT_TRUE(ownership_snapshot_concurrency_verified);

  // Replay the exact deterministic raw stream through the public end-to-end
  // pipeline, which owns candidate/FDE/selection/commit/publication.  N01 has
  // no hardware artifact, so the terminal comparison is the same honest
  // refusal, not a fabricated winner.
  IncrementalUwbImuEstimator online(config, Eigen::Vector3d::Zero());
  online.initialize(initial, config.realtime.prior_sigmas);
  online.ingestImu(boundary);
  RealtimeIntegrityPipeline pipeline(
      &online, IntegrityMonitor(config.risk,
                                config.integrity_window.rank_tolerance,
                                config.integrity_window.max_condition_number));
  IntegrityOutput terminal;
  IntegrityOutput alarm_terminal;
  std::uint64_t terminal_semantic_digest = 0;
  std::uint64_t terminal_timing_digest = 0;
  for (std::size_t epoch = 1; epoch <= oracle.replay_epochs; ++epoch) {
    appendProductionReplayImu(&online, config, epoch,
                              oracle.imu_samples_per_epoch,
                              oracle.epoch_period_ns);
    ClockSample clock;
    clock.wall_monotonic_ns = 1000000000 + epoch * oracle.epoch_period_ns;
    clock.sensor_timestamp_ns = epoch * oracle.epoch_period_ns;
    const UwbBatch raw_batch = makeProductionReplayBatch(
        config, epoch, oracle.alarm_epoch, oracle.epoch_period_ns);
    for (const auto& measurement : raw_batch.measurements) {
      std::cout << "[P0-07-RAW-UWB]\t" << epoch << '\t'
                << measurement.id.value() << '\t'
                << measurement.anchor_id.value() << '\t'
                << measurement.timestamp.value() << '\t'
                << std::setprecision(17) << measurement.range_m << '\t'
                << measurement.sigma_m << '\t'
                << measurement.anchor_position_m.x() << '\t'
                << measurement.anchor_position_m.y() << '\t'
                << measurement.anchor_position_m.z() << '\n';
    }
    const auto attempt_start = std::chrono::steady_clock::now();
    const std::uint64_t arrival_ns = monotonicNs();
    terminal = pipeline.processUwbBatch(raw_batch, clock);
    const auto analysis_done = std::chrono::steady_clock::now();
    const std::uint64_t compute_done_ns = monotonicNs();
    if (epoch == oracle.alarm_epoch) alarm_terminal = terminal;
    double core_compute_ms = std::numeric_limits<double>::quiet_NaN();
    for (const auto& stage : terminal.stage_timings) {
      if (stage.stage == "core_total") core_compute_ms = stage.wall_ms;
    }
    ASSERT_TRUE(std::isfinite(core_compute_ms));
    const double analysis_completion_ms =
        std::chrono::duration<double, std::milli>(
            analysis_done - attempt_start).count();
    FinalPacketTiming packet_timing;
    packet_timing.arrival_steady_ns = arrival_ns;
    packet_timing.compute_done_steady_ns = compute_done_ns;
    packet_timing.packet_ready_steady_ns = monotonicNs();
    packet_timing.deadline_boundary = FinalPacketBoundary::PacketReady;
    packet_timing.publish_outcome = FinalPublishOutcome::NotAttempted;
    const FinalOutputPacket packet = finalizeOutputPacket(
        terminal, packet_timing,
        static_cast<std::uint64_t>(config.publication.deadline_ms * 1e6));
    terminal_semantic_digest = stablePacketSemanticDigest(packet);
    std::ostringstream timing_text;
    timing_text << packet_timing.arrival_steady_ns << '|'
                << packet_timing.compute_done_steady_ns << '|'
                << packet_timing.packet_ready_steady_ns;
    const std::string timing_bytes = timing_text.str();
    terminal_timing_digest = hashBytes(1469598103934665603ULL,
        timing_bytes.data(), timing_bytes.size());
    EXPECT_EQ(packet.output().selected_action_id, terminal.selected_action_id);
    EXPECT_EQ(packet.output().batch_committed, terminal.batch_committed);
    EXPECT_FALSE(packet.output().publication.protected_output);
    const double arrival_to_packet_ready_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - attempt_start).count();
    EXPECT_LE(core_compute_ms, analysis_completion_ms);
    EXPECT_LE(analysis_completion_ms, arrival_to_packet_ready_ms);
    bool complete_work = true;
    for (const auto& stage : terminal.stage_timings) {
      complete_work = complete_work && stage.success;
    }
    struct rusage usage {};
    ASSERT_EQ(getrusage(RUSAGE_SELF, &usage), 0);
    std::cout << "[P0-07-ATTEMPT]\t" << epoch << '\t'
              << terminal.diagnostics.status << '\t'
              << terminal.detector.passed << '\t'
              << terminal.diagnostics.hypothesis_count << '\t'
              << terminal.diagnostics.generated_actions << '\t'
              << terminal.diagnostics.kernel_evaluated_actions << '\t'
              << terminal.diagnostics.coverage_status << '\t'
              << terminal.diagnostics.risk_ledger_all_validated << '\t'
              << terminal.diagnostics.risk_ledger_closes << '\t'
              << terminal.diagnostics.base_svd << '\t'
              << core_compute_ms << '\t' << analysis_completion_ms << '\t'
              << arrival_to_packet_ready_ms << '\t' << terminal.deadline_missed
              << '\t' << complete_work << '\t' << usage.ru_maxrss << '\n';
  }
  EXPECT_EQ(alarm_terminal.diagnostics.hypothesis_count, alarm_hypotheses);
  EXPECT_EQ(alarm_terminal.diagnostics.generated_actions, alarm_actions);
  EXPECT_EQ(alarm_terminal.detector.passed, alarm_all_in_passed);
  EXPECT_GT(alarm_terminal.diagnostics.generated_actions, 1u);
  EXPECT_GE(alarm_terminal.diagnostics.kernel_evaluated_actions, 1u);
  std::size_t terminal_action_occurrences = 0;
  for (const auto& action : alarm_terminal.candidate_audit) {
    if (action.model_error_record.rfind("PLANNED_EVALUATION;", 0) == 0) {
      const auto reference = std::find_if(
          action_references.begin(), action_references.end(),
          [&](const DenseActionReference& value) {
            return value.id == action.action_id;
          });
      ASSERT_NE(reference, action_references.end());
      EXPECT_EQ(action.removed_group_ids, reference->removed);
      EXPECT_EQ(action.added_group_ids, reference->added);
      EXPECT_EQ(action.cardinality, reference->exclusion_cardinality);
      EXPECT_EQ(action.rank, reference->rank);
      EXPECT_EQ(action.dof, reference->dof);
      EXPECT_NEAR(action.statistic, reference->statistic,
                  2e-8 * std::max(1.0, std::abs(reference->statistic)));
      EXPECT_EQ(action.post_detector_passed, reference->post_passed);
      EXPECT_EQ(action.covers_plausible_set, reference->covers_plausible);
      EXPECT_EQ(action.diagnostics.skip_reason, reference->terminal);
      EXPECT_EQ(action.model_error_validated,
                reference->model_error_validated);
      EXPECT_EQ(action.valid, reference->pl_valid);
      classification_comparisons.insert("pl_valid");
      EXPECT_EQ(std::isfinite(action.hpl_m),
                std::isfinite(reference->hpl_m));
      EXPECT_EQ(std::isfinite(action.vpl_m),
                std::isfinite(reference->vpl_m));
      classification_comparisons.insert("finite_or_infinite");
      EXPECT_NEAR(action.risk_allocation, reference->risk_allocation,
                  certifiedTolerance(reference->risk_allocation, 1.0, 1,
                                     oracle));
      EXPECT_EQ(action.selected, reference->selected);
      EXPECT_NEAR(action.information_logdet, reference->information_logdet,
                  2e-8 * std::max(1.0,
                      std::abs(reference->information_logdet)));
      EXPECT_EQ(reference->candidate_grams.size(),
                oracle.plausible_modes.size());
      EXPECT_EQ(reference->protected_responses.size(),
                oracle.plausible_modes.size());
      EXPECT_EQ(reference->protected_slopes.size(),
                oracle.plausible_modes.size());
      EXPECT_EQ(reference->nullspace_classes.size(),
                oracle.plausible_modes.size());
      EXPECT_FALSE(reference->risk_valid);
      EXPECT_FALSE(reference->risk_all_terms_validated);
      EXPECT_TRUE(std::isnan(reference->risk_charged_total));
      EXPECT_FALSE(reference->risk_terms.empty());
      EXPECT_FALSE(reference->selection_tuple.empty());
      if (reference->post_passed) {
        EXPECT_NEAR(action.threshold, reference->threshold,
                    2e-10 * std::max(1.0, reference->threshold));
      }
      EXPECT_NE(action.diagnostics.skip_reason, "PLANNED_NOT_RUN");
      std::cout << "[P0-07-ACTION]\t" << action.action_id << '\t'
                << action.action_type << '\t'
                << action.removed_group_ids << '\t'
                << action.added_group_ids << '\t'
                << action.diagnostics.skip_reason << '\t'
                << action.diagnostics.kernel_evaluated << '\t'
                << action.valid << '\t' << action.post_detector_passed << '\t'
                << action.covers_plausible_set << '\t'
                << action.model_error_validated << '\t'
                << action.rank << '\t' << action.dof << '\t'
                << action.risk_allocation << '\t' << action.statistic << '\t'
                << action.threshold << '\t' << action.hpl_m << '\t'
                << action.vpl_m << '\t' << action.selected << '\t'
                << hashBytes(1469598103934665603ULL,
                             action.physical_source_ids.data(),
                             action.physical_source_ids.size()) << '\t'
                << hashBytes(1469598103934665603ULL, action.reason.data(),
                             action.reason.size()) << '\n';
      ++terminal_action_occurrences;
    }
  }
  EXPECT_EQ(terminal_action_occurrences,
            alarm_terminal.diagnostics.generated_actions);
  ActionHypothesisProofSnapshotV1 same_run_proof;
  ASSERT_TRUE(actionHypothesisProofAuditV1(alarm_terminal, &same_run_proof));
  EXPECT_EQ(same_run_proof.transaction_id, alarm_terminal.transaction_id);
  EXPECT_EQ(same_run_proof.window_id, alarm_terminal.window_id);
  EXPECT_EQ(same_run_proof.selected_action_id,
            alarm_terminal.selected_action_id);
  EXPECT_NE(same_run_proof.snapshot_identity, 0u);
  EXPECT_EQ(same_run_proof.records.size(),
            alarm_actions * alarm_hypotheses);
  std::size_t available_action_hypothesis_proofs = 0;
  std::size_t unavailable_action_hypothesis_proofs = 0;
  std::size_t post_passed_actions = 0;
  for (const auto& action : alarm_terminal.candidate_audit) {
    if (action.model_error_record.rfind("PLANNED_EVALUATION;", 0) == 0 &&
        action.post_detector_passed) {
      ++post_passed_actions;
    }
  }
  std::map<std::uint64_t, std::size_t> records_by_action;
  for (const auto& record : same_run_proof.records) {
    ++records_by_action[record.action_id];
    if (!record.proof_available) {
      ++unavailable_action_hypothesis_proofs;
      EXPECT_FALSE(record.proof_unavailable_reason.empty());
      continue;
    }
    EXPECT_TRUE(record.proof_unavailable_reason.empty());
    ++available_action_hypothesis_proofs;
    EXPECT_NE(record.candidate_proof_identity, 0u);
    EXPECT_NE(record.protection_proof_identity, 0u);
    EXPECT_NE(record.hypothesis_proof_identity, 0u);
    EXPECT_GT(record.candidate_rows, 0);
    EXPECT_GT(record.gram.size(), 0);
    EXPECT_EQ(record.protected_response.rows(), 3);
    EXPECT_TRUE(record.protected_slopes.allFinite());
    EXPECT_TRUE(record.pl_contribution_m.allFinite() ||
                record.pl_contribution_m.array().isInf().all());
  }
  EXPECT_GT(available_action_hypothesis_proofs, 0u);
  EXPECT_EQ(available_action_hypothesis_proofs +
                unavailable_action_hypothesis_proofs,
            alarm_actions * alarm_hypotheses);
  ASSERT_EQ(records_by_action.size(), alarm_actions);
  for (const auto& action : records_by_action) {
    EXPECT_EQ(action.second, alarm_hypotheses);
  }

  // Copy/move and concurrent readers resolve the immutable identity rather
  // than an IntegrityOutput address.  A contradictory final winner cannot
  // retrieve the proof frozen for this attempt.
  IntegrityOutput copied_alarm = alarm_terminal;
  IntegrityOutput moved_alarm = std::move(copied_alarm);
  ActionHypothesisProofSnapshotV1 moved_proof;
  ASSERT_TRUE(actionHypothesisProofAuditV1(moved_alarm, &moved_proof));
  EXPECT_EQ(moved_proof.snapshot_identity, same_run_proof.snapshot_identity);
  std::atomic<bool> concurrent_proof_ok{true};
  std::vector<std::thread> proof_readers;
  for (int reader = 0; reader < 4; ++reader) {
    proof_readers.emplace_back([&] {
      for (int iteration = 0; iteration < 20; ++iteration) {
        ActionHypothesisProofSnapshotV1 observed;
        if (!actionHypothesisProofAuditV1(moved_alarm, &observed) ||
            observed.snapshot_identity != same_run_proof.snapshot_identity ||
            observed.records.size() != same_run_proof.records.size()) {
          concurrent_proof_ok.store(false, std::memory_order_release);
          return;
        }
      }
    });
  }
  for (auto& reader : proof_readers) reader.join();
  EXPECT_TRUE(concurrent_proof_ok.load(std::memory_order_acquire));
  IntegrityOutput contradictory = alarm_terminal;
  contradictory.selected_action_id = 999999;
  ActionHypothesisProofSnapshotV1 rejected_proof;
  EXPECT_FALSE(actionHypothesisProofAuditV1(contradictory, &rejected_proof));
  EXPECT_EQ(alarm_terminal.selected_action_id, 0u);
  EXPECT_EQ(alarm_terminal.selected_action_id, oracle.observed_winner)
      << "observed metadata is a checked display, never selection authority";
  EXPECT_FALSE(alarm_terminal.batch_committed);
  EXPECT_FALSE(alarm_terminal.diagnostics.risk_ledger_all_validated);
  EXPECT_FALSE(alarm_terminal.diagnostics.risk_ledger_closes);
  EXPECT_TRUE(alarm_terminal.batch_committed ||
              alarm_terminal.protection_level.availability ==
                  Availability::Unavailable);
  EXPECT_FALSE(terminal.protection_level.formal_eligible);
  classification_comparisons.insert("eligibility");
  EXPECT_EQ(terminal.protection_level.availability, Availability::Unavailable);
  EXPECT_FALSE(terminal.publication.protected_output);
  EXPECT_NE(terminal.protection_level.reason.find("IMU_MODEL_UNQUALIFIED"),
            std::string::npos);
  EXPECT_NE(terminal_semantic_digest, 0u);
  EXPECT_EQ(classification_comparisons,
            std::set<std::string>(oracle.tolerance_classifications.begin(),
                                  oracle.tolerance_classifications.end()));

  // Trigger the real post-mutation seam through the public pipeline. The
  // pipeline catch owns the terminal output; no test-side IntegrityOutput is
  // synthesized. With no ROS transport installed, packet publication is
  // honestly NotAttempted and packet-ready is the final sampled boundary.
  appendProductionReplayImu(&online, config, oracle.replay_epochs + 1,
                            oracle.imu_samples_per_epoch,
                            oracle.epoch_period_ns);
  const std::size_t epoch_before_exception = online.currentEpoch();
  const auto nonce = online.setCommitFaultPointForTesting(
      CommitFaultPoint::BeforeReceiptCreation);
  ASSERT_NE(nonce, 0u);
  ClockSample exception_clock;
  exception_clock.wall_monotonic_ns = 1000000000ULL +
      (oracle.replay_epochs + 1) * oracle.epoch_period_ns;
  exception_clock.sensor_timestamp_ns =
      (oracle.replay_epochs + 1) * oracle.epoch_period_ns;
  FinalPacketTiming exception_timing;
  exception_timing.arrival_steady_ns = monotonicNs();
  try {
    (void)pipeline.processUwbBatch(makeProductionReplayBatch(
                                       config, oracle.replay_epochs + 1,
                                       oracle.alarm_epoch,
                                       oracle.epoch_period_ns),
                                   exception_clock);
    FAIL() << "post-mutation pipeline exception seam did not fire";
  } catch (const CommitTerminalError& error) {
    EXPECT_TRUE(error.receipt().backend_mutated);
    EXPECT_TRUE(error.receipt().backend_poisoned);
    EXPECT_TRUE(error.receipt().committed_unprotected);
    EXPECT_EQ(error.receipt().backend_updates, 1u);
    EXPECT_EQ(error.receipt().injected_fault_point,
              CommitFaultPoint::BeforeReceiptCreation);
    EXPECT_EQ(error.receipt().injected_fault_nonce, nonce);
  }
  const IntegrityOutput& exception_output = pipeline.lastAttemptOutput();
  EXPECT_EQ(exception_output.diagnostics.status, "EXCEPTION");
  EXPECT_EQ(exception_output.backend_updates, 1u);
  EXPECT_FALSE(exception_output.batch_committed);
  EXPECT_FALSE(exception_output.publication.protected_output);
  EXPECT_EQ(exception_output.protection_level.availability,
            Availability::Unavailable);
  EXPECT_TRUE(online.backendPoisoned());
  EXPECT_FALSE(online.hasPendingEpoch());
  EXPECT_EQ(online.currentEpoch(), epoch_before_exception);
  exception_timing.compute_done_steady_ns = monotonicNs();
  exception_timing.packet_ready_steady_ns = monotonicNs();
  exception_timing.deadline_boundary = FinalPacketBoundary::PacketReady;
  exception_timing.attempt_kind = FinalAttemptKind::Exception;
  exception_timing.publish_outcome = FinalPublishOutcome::NotAttempted;
  const FinalOutputPacket exception_packet = finalizeOutputPacket(
      exception_output, exception_timing,
      static_cast<std::uint64_t>(config.publication.deadline_ms * 1e6));
  const std::uint64_t exception_semantic_digest =
      stablePacketSemanticDigest(exception_packet);
  EXPECT_NE(exception_semantic_digest, 0u);
  EXPECT_EQ(exception_packet.timing().publish_outcome,
            FinalPublishOutcome::NotAttempted);

  std::cout << "[P0-07-CONTRACT]\talarm_epoch=23\tgenerated="
            << alarm_terminal.diagnostics.generated_actions
            << "\tkernel_evaluated="
            << alarm_terminal.diagnostics.kernel_evaluated_actions
            << "\texpected_modes=" << alarm_expected_modes
            << "\trepresented_modes=" << alarm_represented_modes
            << "\tevaluated_modes=" << alarm_evaluated_modes
            << "\texpected_pairs=" << alarm_expected_pairs
            << "\trepresented_pairs=" << alarm_represented_pairs
            << "\tevaluated_pairs=" << alarm_evaluated_pairs
            << "\tselected_action=" << alarm_terminal.selected_action_id
            << "\tbatch_committed=" << alarm_terminal.batch_committed
            << "\tterminal_semantic_digest=" << terminal_semantic_digest
            << "\tterminal_timing_digest=" << terminal_timing_digest
            << "\tformal_eligible="
            << terminal.protection_level.formal_eligible
            << "\tprotected_output=" << terminal.publication.protected_output
            << "\texception_semantic_digest=" << exception_semantic_digest
            << "\texception_backend_updates=" << exception_output.backend_updates
            << "\texception_backend_poisoned=" << online.backendPoisoned()
            << "\texception_publish_outcome=NOT_ATTEMPTED"
            << '\n';
}

TEST(P007CorrectedExhaustive, ProductionPipelineFailsClosedWithoutCalibration) {
  IntegrityConfig config = IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_off.yaml");
  config.publication.deadline_ms = 1e9;
  // Prove that missing model qualification itself closes the protected path;
  // this is deliberately true even though all shipped research profiles keep
  // platform certification disabled as a second, independent gate.
  config.publication.protected_output_enabled = true;
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  for (int i = 0; i <= 2; ++i) {
    ImuMeasurement imu;
    imu.id = MeasurementId(i + 1);
    imu.timestamp = TimestampNs(i * 5000000);
    imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
    estimator.ingestImu(imu);
  }
  RealtimeIntegrityPipeline pipeline(
      &estimator,
      IntegrityMonitor(config.risk, config.snapshot.rank_tolerance,
                       config.snapshot.max_condition_number));
  ClockSample clock;
  clock.wall_monotonic_ns = 1000000000;
  clock.sensor_timestamp_ns = 10000000;
  const IntegrityOutput output = pipeline.processUwbBatch(
      makeBatch(TimestampNs(10000000)), clock);
  EXPECT_FALSE(output.protection_level.formal_eligible);
  EXPECT_EQ(output.protection_level.availability, Availability::Unavailable);
  EXPECT_NE(output.protection_level.reason.find("IMU_MODEL_UNQUALIFIED"),
            std::string::npos);
  EXPECT_NE(std::find(output.reason_codes.begin(), output.reason_codes.end(),
                      "IMU_MODEL_UNQUALIFIED"),
            output.reason_codes.end());
  EXPECT_FALSE(output.publication.protected_output);
  EXPECT_TRUE(output.publication.unprotected_output);
}

TEST(P007CorrectedExhaustive,
     MissingProvenanceCensusIsRetainedAndRefused) {
  const OracleManifest oracle = loadOracleManifest();
  IntegrityConfig config = IntegrityConfigLoader::load(
      std::string(UWB_IMU_PL_SOURCE_DIR) + "/config/fde_uwb_order1.yaml");
  verifyRecipeConfigBinding(oracle, config);
  config.incremental.fixed_lag_epochs =
      oracle.alarm_epoch - oracle.physical_first_epoch + 1;
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  ImuMeasurement boundary;
  boundary.id = MeasurementId(900000);
  boundary.timestamp = TimestampNs(0);
  boundary.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
  estimator.ingestImu(boundary);
  for (std::size_t epoch = 1; epoch < oracle.alarm_epoch; ++epoch) {
    appendProductionReplayImu(&estimator, config, epoch,
                              oracle.imu_samples_per_epoch,
                              oracle.epoch_period_ns);
    auto transaction = estimator.prepareEpoch(
        makeProductionReplayBatch(config, epoch, oracle.alarm_epoch, oracle.epoch_period_ns),
        EpochPreparationOptions{true, true, false, true});
    const auto plan = EpochCommitPlan::nominalPlan(transaction);
    estimator.commitEpoch(std::move(transaction), plan);
  }
  appendProductionReplayImu(&estimator, config, oracle.alarm_epoch,
                            oracle.imu_samples_per_epoch,
                            oracle.epoch_period_ns);
  auto transaction = estimator.prepareEpoch(
      makeProductionReplayBatch(config, oracle.alarm_epoch, oracle.alarm_epoch,
                                oracle.epoch_period_ns),
      EpochPreparationOptions{true, true, false, true});
  const std::uint64_t absent_replacement = oracleReplacementGroup(
      oracle.missing_onset, oracle.missing_anchor, oracle);
  transaction.uwb_groups.erase(std::remove_if(
      transaction.uwb_groups.begin(), transaction.uwb_groups.end(),
      [&](const PendingFactorGroup& group) {
        return group.id.value() == absent_replacement;
      }), transaction.uwb_groups.end());
  ASSERT_TRUE(std::none_of(transaction.uwb_groups.begin(),
      transaction.uwb_groups.end(), [&](const PendingFactorGroup& group) {
        return group.id.value() == absent_replacement;
      }));
  const auto window = estimator.buildIntegrityWindow(
      transaction, IntegrityWindowRequest{config.integrity_window.epochs});
  ASSERT_TRUE(window.model_valid) << window.reason;
  ImuFaultSubspaces no_imu;
  LinearizedFactorBlock no_bridge;
  auto models = HypothesisGenerator(productionGeneratorConfig(config)).generate(
      window, transaction, no_imu, no_bridge);
  DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = config.detector.p_fa_per_test;
  detector_risk.rank_tolerance = config.integrity_window.rank_tolerance;
  detector_risk.max_condition_number =
      config.integrity_window.max_condition_number;
  const DetectorResultV2 detector =
      JointWindowDetector().evaluate(window, detector_risk);
  ASSERT_FALSE(detector.passed);
  HypothesisEvaluationConfig evidence_config;
  evidence_config.rank_tolerance = config.integrity_window.rank_tolerance;
  evidence_config.max_condition_number =
      config.integrity_window.max_condition_number;
  const auto evidence = HypothesisEvidenceEvaluator(evidence_config).evaluateAll(
      window, models.modes, &models.hypotheses, detector.squared_threshold);
  GeneratedActionSnapshotV1 trusted;
  const ActionSearchResultV1 search =
      HypothesisGenerator(productionGeneratorConfig(config))
          .actionsForPlausibleSetV1(window, transaction, &models, evidence, {},
                                    &trusted);
  ASSERT_TRUE(search.census.exhaustive);
  ASSERT_GT(search.generated_snapshot.actions.size(), 1u);
  std::vector<CandidateEvaluation> candidates;
  std::size_t missing_actions = 0;
  for (const auto& action : search.generated_snapshot.actions) {
    CandidateEvaluation candidate = DenseCandidateOracle().evaluate(window,
                                                                     action);
    if (action.recoverability == HistoryRecoverability::MissingProvenance) {
      ++missing_actions;
      EXPECT_TRUE(std::find(action.groups_to_remove.begin(),
                            action.groups_to_remove.end(),
                            FactorGroupId(oracleUwbGroup(
                                oracle.alarm_epoch, oracle))) !=
                  action.groups_to_remove.end());
      EXPECT_TRUE(std::find(action.groups_to_add.begin(),
                            action.groups_to_add.end(),
                            FactorGroupId(absent_replacement)) ==
                  action.groups_to_add.end());
    }
    candidates.push_back(std::move(candidate));
  }
  ASSERT_GT(missing_actions, 0u);
  const FdeDecision decision = FdeManager().decide(
      detector, models.hypotheses, evidence, &candidates, {}, config.risk_v2);
  EXPECT_FALSE(decision.selected_action.has_value());
  EXPECT_FALSE(decision.commit_allowed);
  EXPECT_FALSE(decision.integrity_available);
  for (const auto& candidate : candidates) {
    if (candidate.action.recoverability ==
        HistoryRecoverability::MissingProvenance) {
      EXPECT_FALSE(candidate.selected);
    }
  }
}
