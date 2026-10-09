#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"
#include "uwb_imu_pl/publication/final_output_packet.hpp"
#include "../src/uwb_imu_pl/integrity/numerical_phase_profile.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <Eigen/SVD>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

#ifndef UWB_IMU_PL_GIT_SHA
#define UWB_IMU_PL_GIT_SHA "unknown"
#endif
#ifndef UWB_IMU_PL_GIT_DIRTY
#define UWB_IMU_PL_GIT_DIRTY 1
#endif

namespace uwb_imu_pl {

struct P106GraphSnapshotV1 {
  static constexpr std::uint64_t kSchema = 1;
  std::size_t attempt = 0;
  double position_error_m = 0.0;
  double attitude_error_rad = 0.0;
  Eigen::Vector3d truth_position_m = Eigen::Vector3d::Zero();
  Eigen::Quaterniond truth_orientation = Eigen::Quaterniond::Identity();
  LinearizationVersion version;
  std::size_t epoch = 0;
  gtsam::NonlinearFactorGraph active_factors;
  gtsam::Values linearization_point;
  gtsam::Values production_estimate;
  Eigen::Matrix<double, 15, 15> production_marginal;
};

struct P106ReadOnlyGraphSnapshotPeer {
  static P106GraphSnapshotV1 capture(
      const IncrementalUwbImuEstimator& estimator, std::size_t attempt,
      double position_error_m, double attitude_error_rad,
      const Eigen::Vector3d& truth_position_m,
      const Eigen::Quaterniond& truth_orientation) {
    P106GraphSnapshotV1 out;
    out.attempt = attempt;
    out.position_error_m = position_error_m;
    out.attitude_error_rad = attitude_error_rad;
    out.truth_position_m = truth_position_m;
    out.truth_orientation = truth_orientation;
    out.version = {estimator.graph_version_, estimator.ordering_version_, 1,
                   estimator.linpoint_version_};
    out.epoch = estimator.epoch_;
    out.active_factors = estimator.activeGraph();
    out.linearization_point = estimator.backendIsam().getLinearizationPoint();
    out.production_estimate = estimator.backendIsam().calculateEstimate();
    out.production_marginal = estimator.currentJointMarginal();
    return out;
  }
};

}  // namespace uwb_imu_pl

namespace {
constexpr double kImuDt = 0.005;
constexpr double kPi = 3.14159265358979323846;
// The configured covariance is an integrity overbound.  The frozen nominal
// campaign samples inside that envelope rather than assuming equality.
constexpr double kPhysicalNoiseFraction = 0.05;
constexpr double kGyroPhysicalNoiseFraction = 0.01;

Eigen::Vector3d position(double time, bool degenerate = false) {
  if (degenerate) return {0.0, 0.0, 1.2};
  return {2.0*std::sin(.25*time), 1.5*std::sin(.37*time),
          1.2 + .5*std::sin(.19*time)};
}
Eigen::Vector3d velocity(double time, bool degenerate = false) {
  if (degenerate) return Eigen::Vector3d::Zero();
  return {.5*std::cos(.25*time), .555*std::cos(.37*time),
          .095*std::cos(.19*time)};
}
Eigen::Vector3d acceleration(double time, bool degenerate = false) {
  if (degenerate) return Eigen::Vector3d::Zero();
  return {-.125*std::sin(.25*time), -.20535*std::sin(.37*time),
          -.01805*std::sin(.19*time)};
}
Eigen::Quaterniond orientation(double time, bool degenerate = false) {
  if (degenerate) return Eigen::Quaterniond::Identity();
  const double roll = .12*std::sin(.31*time);
  const double pitch = .10*std::sin(.27*time);
  const double yaw = .35*std::sin(.21*time) + .08*time;
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
}
Eigen::Vector3d angularVelocity(double time, bool degenerate = false) {
  if (degenerate) return Eigen::Vector3d::Zero();
  constexpr double epsilon = 1e-6;
  Eigen::Quaterniond delta =
      orientation(time).conjugate() * orientation(time + epsilon);
  delta.normalize();
  Eigen::AngleAxisd axis_angle(delta);
  return axis_angle.axis() * axis_angle.angle() / epsilon;
}

std::uint64_t environmentSeed(std::uint64_t fallback) {
  const char* text = std::getenv("UWB_IMU_PL_BENCHMARK_SEED");
  if (!text) return fallback;
  std::size_t used = 0;
  const auto value = std::stoull(text, &used);
  if (text[used] != '\0') throw std::invalid_argument("invalid benchmark seed");
  return value;
}

void hashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= bytes[i];
    *hash *= 1099511628211ULL;
  }
}

template <class T>
void hashScalar(std::uint64_t* hash, const T& value) {
  hashBytes(hash, &value, sizeof(value));
}

template <class Derived>
void hashMatrix(std::uint64_t* hash, const Eigen::MatrixBase<Derived>& value) {
  const Eigen::Index rows = value.rows();
  const Eigen::Index cols = value.cols();
  hashScalar(hash, rows);
  hashScalar(hash, cols);
  for (Eigen::Index column = 0; column < cols; ++column) {
    for (Eigen::Index row = 0; row < rows; ++row) {
      const double item = value(row, column);
      hashScalar(hash, item);
    }
  }
}

struct P106KktAudit {
  double production_objective = std::numeric_limits<double>::infinity();
  double batch_objective = std::numeric_limits<double>::infinity();
  double production_gradient_l2 = std::numeric_limits<double>::infinity();
  double production_gradient_inf = std::numeric_limits<double>::infinity();
  double batch_gradient_l2 = std::numeric_limits<double>::infinity();
  double batch_gradient_inf = std::numeric_limits<double>::infinity();
  int rank = 0;
  double sigma_max = 0.0;
  double sigma_min = 0.0;
  double condition = std::numeric_limits<double>::infinity();
  double state_delta_l2 = std::numeric_limits<double>::infinity();
  double covariance_relative_error = std::numeric_limits<double>::infinity();
  double covariance_vs_production_graph_relative_error =
      std::numeric_limits<double>::infinity();
  double covariance_vs_linpoint_graph_relative_error =
      std::numeric_limits<double>::infinity();
  double batch_position_error_m = std::numeric_limits<double>::infinity();
  double batch_attitude_error_rad = std::numeric_limits<double>::infinity();
  std::uint64_t local_graph_hash = 0;
};

std::pair<double, double> graphGradient(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values) {
  const auto gaussian = graph.linearize(values);
  const auto system = gaussian->jacobian();
  const Eigen::VectorXd gradient = -system.first.transpose() * system.second;
  return {gradient.norm(), gradient.size() == 0
                               ? 0.0
                               : gradient.cwiseAbs().maxCoeff()};
}

Eigen::Matrix<double, 15, 15> currentMarginal(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    std::size_t epoch) {
  const gtsam::KeyVector keys{gtsam::Symbol('x', epoch),
                             gtsam::Symbol('v', epoch),
                             gtsam::Symbol('b', epoch)};
  const gtsam::JointMarginal joint =
      gtsam::Marginals(graph, values).jointMarginalCovariance(keys);
  Eigen::Matrix<double, 15, 15> out;
  out.block<6, 6>(0, 0) = joint.at(keys[0], keys[0]);
  out.block<6, 3>(0, 6) = joint.at(keys[0], keys[1]);
  out.block<6, 6>(0, 9) = joint.at(keys[0], keys[2]);
  out.block<3, 6>(6, 0) = joint.at(keys[1], keys[0]);
  out.block<3, 3>(6, 6) = joint.at(keys[1], keys[1]);
  out.block<3, 6>(6, 9) = joint.at(keys[1], keys[2]);
  out.block<6, 6>(9, 0) = joint.at(keys[2], keys[0]);
  out.block<6, 3>(9, 6) = joint.at(keys[2], keys[1]);
  out.block<6, 6>(9, 9) = joint.at(keys[2], keys[2]);
  return out;
}

P106KktAudit auditSnapshot(const uwb_imu_pl::P106GraphSnapshotV1& snapshot) {
  P106KktAudit out;
  const auto production_linear =
      snapshot.active_factors.linearize(snapshot.production_estimate);
  const auto system = production_linear->jacobian();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(system.first);
  const Eigen::VectorXd singular = svd.singularValues();
  out.sigma_max = singular.size() == 0 ? 0.0 : singular(0);
  const double tolerance = out.sigma_max *
      static_cast<double>(std::max(system.first.rows(), system.first.cols())) *
      std::numeric_limits<double>::epsilon();
  out.rank = static_cast<int>((singular.array() > tolerance).count());
  out.sigma_min = out.rank == 0 ? 0.0 : singular(out.rank - 1);
  out.condition = out.sigma_min > 0.0
                      ? out.sigma_max / out.sigma_min
                      : std::numeric_limits<double>::infinity();
  out.production_objective = snapshot.active_factors.error(
      snapshot.production_estimate);
  const auto production_gradient = graphGradient(
      snapshot.active_factors, snapshot.production_estimate);
  out.production_gradient_l2 = production_gradient.first;
  out.production_gradient_inf = production_gradient.second;

  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SILENT");
  params.maxIterations = 200;
  params.relativeErrorTol = 1e-12;
  params.absoluteErrorTol = 1e-12;
  const gtsam::Values batch = gtsam::LevenbergMarquardtOptimizer(
      snapshot.active_factors, snapshot.production_estimate, params).optimize();
  out.batch_objective = snapshot.active_factors.error(batch);
  const auto batch_gradient = graphGradient(snapshot.active_factors, batch);
  out.batch_gradient_l2 = batch_gradient.first;
  out.batch_gradient_inf = batch_gradient.second;

  const gtsam::Key x = gtsam::Symbol('x', snapshot.epoch);
  const gtsam::Key v = gtsam::Symbol('v', snapshot.epoch);
  const gtsam::Key b = gtsam::Symbol('b', snapshot.epoch);
  Eigen::Matrix<double, 15, 1> delta;
  delta.head<6>() = snapshot.production_estimate.at<gtsam::Pose3>(x)
      .localCoordinates(batch.at<gtsam::Pose3>(x));
  delta.segment<3>(6) = batch.at<gtsam::Vector3>(v) -
      snapshot.production_estimate.at<gtsam::Vector3>(v);
  delta.tail<6>() = batch.at<gtsam::imuBias::ConstantBias>(b).vector() -
      snapshot.production_estimate.at<gtsam::imuBias::ConstantBias>(b).vector();
  out.state_delta_l2 = delta.norm();
  const auto batch_pose = batch.at<gtsam::Pose3>(x);
  out.batch_position_error_m =
      (batch_pose.translation() - snapshot.truth_position_m).norm();
  const Eigen::Quaterniond batch_q(batch_pose.rotation().matrix());
  const double batch_q_dot = std::min(
      1.0, std::abs(batch_q.normalized().dot(
               snapshot.truth_orientation.normalized())));
  out.batch_attitude_error_rad = 2.0 * std::acos(batch_q_dot);
  const auto batch_marginal = currentMarginal(
      snapshot.active_factors, batch, snapshot.epoch);
  const auto production_graph_marginal = currentMarginal(
      snapshot.active_factors, snapshot.production_estimate, snapshot.epoch);
  const auto linpoint_graph_marginal = currentMarginal(
      snapshot.active_factors, snapshot.linearization_point, snapshot.epoch);
  out.covariance_relative_error =
      (snapshot.production_marginal - batch_marginal).norm() /
      std::max({1.0, snapshot.production_marginal.norm(),
                batch_marginal.norm()});
  out.covariance_vs_production_graph_relative_error =
      (snapshot.production_marginal - production_graph_marginal).norm() /
      std::max({1.0, snapshot.production_marginal.norm(),
                production_graph_marginal.norm()});
  out.covariance_vs_linpoint_graph_relative_error =
      (snapshot.production_marginal - linpoint_graph_marginal).norm() /
      std::max({1.0, snapshot.production_marginal.norm(),
                linpoint_graph_marginal.norm()});

  std::uint64_t hash = 1469598103934665603ULL;
  hashScalar(&hash, uwb_imu_pl::P106GraphSnapshotV1::kSchema);
  hashScalar(&hash, snapshot.attempt);
  hashScalar(&hash, snapshot.epoch);
  hashScalar(&hash, snapshot.version.graph_version);
  hashScalar(&hash, snapshot.version.ordering_version);
  hashScalar(&hash, snapshot.version.noise_model_version);
  hashScalar(&hash, snapshot.version.linpoint_version);
  for (const auto key : snapshot.linearization_point.keys()) hashScalar(&hash, key);
  const auto linpoint_system =
      snapshot.active_factors.linearize(snapshot.linearization_point)->jacobian();
  hashMatrix(&hash, linpoint_system.first);
  hashMatrix(&hash, linpoint_system.second);
  hashMatrix(&hash, system.first);
  hashMatrix(&hash, system.second);
  hashMatrix(&hash, snapshot.production_marginal);
  out.local_graph_hash = hash;
  return out;
}

void writeKktSnapshot(std::ostream& stream, const char* label,
                      const uwb_imu_pl::P106GraphSnapshotV1& snapshot,
                      const P106KktAudit& audit, bool trailing_comma) {
  stream << "    \"" << label << "\": {\n"
         << "      \"schema\": \"uwb-imu-pl/read-only-graph-snapshot/v1\",\n"
         << "      \"attempt\": " << snapshot.attempt << ",\n"
         << "      \"epoch\": " << snapshot.epoch << ",\n"
         << "      \"graph_version\": " << snapshot.version.graph_version << ",\n"
         << "      \"ordering_version\": " << snapshot.version.ordering_version << ",\n"
         << "      \"noise_model_version\": " << snapshot.version.noise_model_version << ",\n"
         << "      \"linpoint_version\": " << snapshot.version.linpoint_version << ",\n"
         << "      \"active_factor_slots\": " << snapshot.active_factors.size() << ",\n"
         << "      \"active_value_count\": " << snapshot.production_estimate.size() << ",\n"
         << "      \"position_error_m\": " << snapshot.position_error_m << ",\n"
         << "      \"attitude_error_rad\": " << snapshot.attitude_error_rad << ",\n"
         << "      \"local_graph_hash_fnv1a64\": \"" << std::hex
         << audit.local_graph_hash << std::dec << "\",\n"
         << "      \"production_objective\": " << audit.production_objective << ",\n"
         << "      \"batch_objective\": " << audit.batch_objective << ",\n"
         << "      \"production_gradient_l2\": " << audit.production_gradient_l2 << ",\n"
         << "      \"production_gradient_inf\": " << audit.production_gradient_inf << ",\n"
         << "      \"batch_gradient_l2\": " << audit.batch_gradient_l2 << ",\n"
         << "      \"batch_gradient_inf\": " << audit.batch_gradient_inf << ",\n"
         << "      \"rank\": " << audit.rank << ",\n"
         << "      \"sigma_max\": " << audit.sigma_max << ",\n"
         << "      \"sigma_min\": " << audit.sigma_min << ",\n"
         << "      \"condition\": " << audit.condition << ",\n"
         << "      \"production_vs_batch_state_l2\": " << audit.state_delta_l2 << ",\n"
         << "      \"batch_position_error_m\": " << audit.batch_position_error_m << ",\n"
         << "      \"batch_attitude_error_rad\": " << audit.batch_attitude_error_rad << ",\n"
         << "      \"production_vs_batch_covariance_relative_error\": "
         << audit.covariance_relative_error << ",\n"
         << "      \"production_covariance_vs_active_graph_at_production_relative_error\": "
         << audit.covariance_vs_production_graph_relative_error << ",\n"
         << "      \"production_covariance_vs_active_graph_at_linpoint_relative_error\": "
         << audit.covariance_vs_linpoint_graph_relative_error << "\n"
         << "    }" << (trailing_comma ? "," : "") << "\n";
}

struct Scenario {
  std::string name = "nominal";
  bool noiseless = false;
  bool initial_error = false;
  bool degenerate = false;
  bool uwb = false;
  bool imu_accel = false;
  bool imu_gyro = false;
  bool persistent_reject = false;
  bool historical = false;
  bool single_epoch = false;
  double uwb_bias_m = 1.0;
  double accel_bias_mps2 = .8;
  double gyro_bias_radps = .12;
  int axis = 0;
};

struct FaultSchedule {
  int begin = 0;  // zero based, inclusive
  int end = -1;   // zero based, inclusive
};

// This is the frozen synthetic-campaign schedule.  Keeping the calculation in
// one function makes the 300-attempt reference and a scaled campaign use the
// same semantics instead of accidentally taking the first N pre-fault rows.
FaultSchedule faultSchedule(const Scenario& scenario, int epochs) {
  const int begin = scenario.historical ? 5 : std::max(5, epochs / 3);
  const int end = scenario.single_epoch ? begin :
      (scenario.historical ? std::max(5, epochs - 25) :
                             std::max(begin, 2 * epochs / 3 - 1));
  return {begin, end};
}

Scenario scenarioFromEnvironment() {
  Scenario result;
  if (const char* text = std::getenv("UWB_IMU_PL_SCENARIO")) result.name = text;
  auto overrides = [](Scenario value) {
    auto load = [](const char* name, double fallback) {
      const char* text = std::getenv(name);
      if (!text) return fallback;
      std::size_t used = 0;
      const double parsed = std::stod(text, &used);
      if (text[used] != '\0' || !std::isfinite(parsed))
        throw std::invalid_argument(std::string("invalid ") + name);
      return parsed;
    };
    value.uwb_bias_m = load("UWB_IMU_PL_UWB_BIAS_M", value.uwb_bias_m);
    value.accel_bias_mps2 = load("UWB_IMU_PL_ACCEL_BIAS_MPS2", value.accel_bias_mps2);
    value.gyro_bias_radps = load("UWB_IMU_PL_GYRO_BIAS_RADPS", value.gyro_bias_radps);
    return value;
  };
  if (result.name == "nominal") return overrides(result);
  if (result.name == "noiseless") { result.noiseless = true; return overrides(result); }
  if (result.name == "initial_error") { result.initial_error = true; return overrides(result); }
  if (result.name == "degenerate") { result.degenerate = true; return overrides(result); }
  if (result.name == "uwb_fault" || result.name == "history_uwb") {
    result.uwb = true; result.historical = result.name == "history_uwb";
    return overrides(result);
  }
  if (result.name == "uwb_recovery") {
    result.uwb = true; result.single_epoch = true; result.uwb_bias_m = 2.25;
    return overrides(result);
  }
  if (result.name == "imu_recovery") {
    result.imu_accel = true; result.single_epoch = true;
    result.accel_bias_mps2 = 20.0; return overrides(result);
  }
  if (result.name == "joint_recovery") {
    result.uwb = true; result.imu_accel = true; result.single_epoch = true;
    result.uwb_bias_m = 2.25; result.accel_bias_mps2 = 20.0;
    return overrides(result);
  }
  if (result.name == "persistent_reject") {
    result.uwb = true; result.persistent_reject = true; return overrides(result);
  }
  if (result.name == "joint_fault") {
    result.uwb = true; result.imu_accel = true; result.axis = 0;
    return overrides(result);
  }
  const std::string accel_prefix = "imu_accel_";
  const std::string gyro_prefix = "imu_gyro_";
  if (result.name.compare(0, accel_prefix.size(), accel_prefix) == 0 ||
      result.name.compare(0, gyro_prefix.size(), gyro_prefix) == 0) {
    result.imu_accel = result.name.compare(0, accel_prefix.size(), accel_prefix) == 0;
    result.imu_gyro = !result.imu_accel;
    const char axis = result.name.back();
    if (axis < 'x' || axis > 'z') throw std::invalid_argument("invalid IMU fault axis");
    result.axis = axis - 'x';
    return overrides(result);
  }
  throw std::invalid_argument("unknown UWB_IMU_PL_SCENARIO: " + result.name);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: realtime_performance_benchmark CONFIG OUTPUT_DIR EPOCHS\n";
    return 2;
  }
  try {
    auto config = uwb_imu_pl::IntegrityConfigLoader::load(argv[1]);
    const Scenario scenario = scenarioFromEnvironment();
    const int epochs = std::stoi(argv[3]);
    int schedule_epochs = epochs;
    if (const char* value = std::getenv("UWB_IMU_PL_FAULT_SCHEDULE_EPOCHS")) {
      schedule_epochs = std::stoi(value);
      if (schedule_epochs <= 0) throw std::invalid_argument("fault schedule epochs must be positive");
    }
    const FaultSchedule schedule = faultSchedule(scenario, schedule_epochs);
    const FaultSchedule reference_schedule = faultSchedule(scenario, 300);
    const bool fault_expected = scenario.uwb || scenario.imu_accel ||
                                scenario.imu_gyro;
    config.seed = environmentSeed(config.seed);
    // This executable historically discarded these records after paying their
    // formatting cost. Preserve its external log mode while disabling their
    // creation at the source; numerical risk/action summaries remain.
    config.output.write_hypothesis_evidence = false;
    config.output.write_health = false;
    config.output.write_factor_ledger = false;
    config.resolved_yaml +=
        "\n# realtime_performance_benchmark runtime output mode\n"
        "benchmark_write_hypothesis_evidence: false\n"
        "benchmark_write_health: false\n"
        "benchmark_write_factor_ledger: false\n"
        "benchmark_scenario: " + scenario.name + "\n"
        "benchmark_effective_seed: " + std::to_string(config.seed) + "\n"
        "benchmark_uwb_bias_m: " + std::to_string(scenario.uwb_bias_m) + "\n"
        "benchmark_accel_bias_mps2: " +
            std::to_string(scenario.accel_bias_mps2) + "\n"
        "benchmark_gyro_bias_radps: " +
            std::to_string(scenario.gyro_bias_radps) + "\n"
        "benchmark_fault_schedule_algorithm: "
            "proportional_thirds_min5_history_tail25_v1\n"
        "benchmark_fault_schedule_reference_epochs: 300\n"
        "benchmark_fault_expected: " +
            std::string(fault_expected ? "true" : "false") + "\n"
        "benchmark_fault_reference_begin_epoch_1based: " +
            std::to_string(reference_schedule.begin + 1) + "\n"
        "benchmark_fault_reference_end_epoch_1based: " +
            std::to_string(reference_schedule.end + 1) + "\n"
        "benchmark_fault_schedule_epochs: " + std::to_string(schedule_epochs) + "\n"
        "benchmark_fault_effective_epochs: " + std::to_string(epochs) + "\n"
        "benchmark_fault_effective_begin_epoch_1based: " +
            std::to_string(schedule.begin + 1) + "\n"
        "benchmark_fault_effective_end_epoch_1based: " +
            std::to_string(schedule.end + 1) + "\n";
    config.config_hash =
        uwb_imu_pl::IntegrityConfigLoader::hashResolvedYaml(
            config.resolved_yaml);
    if (epochs <= 0 || config.incremental.fixed_lag_epochs <=
                           config.integrity_window.epochs +
                               config.integrity_window.recovery_margin_epochs ||
        !config.output.write_timing || config.output.write_global_diagnostics) {
      throw std::runtime_error("performance config/epoch contract mismatch");
    }
    uwb_imu_pl::RunLogger logger(argv[2], false, true);
    std::ofstream terminal_packets(std::string(argv[2]) +
                                   "/terminal_packets.csv");
    if (!terminal_packets) {
      throw std::runtime_error("failed to open terminal_packets.csv");
    }
    terminal_packets <<
        "input_attempt_id,transaction_id,window_id,selected_action_id,"
        "backend_epoch_before,backend_epoch_after,batch_committed,"
        "risk_ledger_closes,risk_ledger_all_validated,"
        "selection_risk_proof_id,publication_risk_proof_id,"
        "publication_certificate_id,publication_protected,formal_eligible,"
        "deadline_missed,publish_call_steady_ns,publish_return_steady_ns,"
        "arrival_to_publish_ns,"
        "final_packet_digest\n";
    logger.writeResolvedConfig(config.resolved_yaml);
    const std::string execution_command =
        uwb_imu_pl::bindExecutionCommandArguments(
            std::string(argv[0]) + " " + argv[1] + " " + argv[2] + " " +
                argv[3],
            {{"config_path", argv[1]},
             {"run_directory", argv[2]},
             {"fde_profile",
              uwb_imu_pl::toString(config.resolved_scope.profile)},
             {"fixed_lag_epochs",
              std::to_string(config.incremental.fixed_lag_epochs)},
             {"seed", std::to_string(config.seed)},
             {"trajectory", scenario.degenerate ? "static" : "benchmark_3d"},
             {"fault_mode", scenario.name},
             {"fault_anchor_id", "1"},
             {"fault_magnitude_m", std::to_string(scenario.uwb_bias_m)},
             {"accel_fault_mps2",
              std::to_string(scenario.accel_bias_mps2)},
             {"gyro_fault_radps",
              std::to_string(scenario.gyro_bias_radps)},
             {"fault_axis", std::to_string(scenario.axis)},
             {"fault_schedule_algorithm",
              "proportional_thirds_min5_history_tail25_v1"},
             {"fault_schedule_reference_epochs", "300"},
             {"fault_schedule_reference_begin_epoch_1based",
              std::to_string(reference_schedule.begin + 1)},
             {"fault_schedule_reference_end_epoch_1based",
              std::to_string(reference_schedule.end + 1)},
             {"fault_schedule_effective_epochs", std::to_string(epochs)},
             {"fault_schedule_effective_begin_epoch_1based",
              std::to_string(schedule.begin + 1)},
             {"fault_schedule_effective_end_epoch_1based",
              std::to_string(schedule.end + 1)},
             {"packet_loss_prob", "0"},
             {"nlos_probability", "0"},
             {"enable_run_logging", "true"},
             {"write_residuals", "false"},
             {"write_timing", "true"},
             {"write_global_diagnostics", "false"},
             {"output_root", config.output.root}});
    logger.writeManifest(uwb_imu_pl::makeRunManifest(
        config, UWB_IMU_PL_GIT_SHA, UWB_IMU_PL_GIT_DIRTY != 0,
        execution_command));
    uwb_imu_pl::NavigationState initial;
    initial.timestamp = uwb_imu_pl::TimestampNs(0);
    initial.position_world_m = position(0, scenario.degenerate);
    initial.velocity_world_mps = velocity(0, scenario.degenerate);
    initial.q_world_body = orientation(0, scenario.degenerate);
    if (scenario.initial_error) {
      initial.position_world_m += Eigen::Vector3d(.2, -.2, .2);
      initial.velocity_world_mps += Eigen::Vector3d(.2, -.2, .2);
      initial.q_world_body = initial.q_world_body * Eigen::Quaterniond(
          Eigen::AngleAxisd(5.0*kPi/180.0, Eigen::Vector3d(1, 1, 1).normalized()));
    }
    uwb_imu_pl::IncrementalUwbImuEstimator estimator(
        config, config.realtime.lever_arm_body_m);
    estimator.initialize(initial, config.realtime.prior_sigmas);
  // D round: offline replay declares its publication policy explicitly (the
  // default wall timeout is a bridge/streaming policy; this harness computes
  // slower than real time by construction).
    uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          config.risk, config.snapshot.rank_tolerance,
          config.snapshot.max_condition_number),
      uwb_imu_pl::offlineReplayPublicationLimits());
    const bool run_kkt_diagnostic =
        std::getenv("UWB_IMU_PL_KKT_DIAGNOSTIC") != nullptr;
    std::optional<uwb_imu_pl::P106GraphSnapshotV1> worst_position_snapshot;
    std::optional<uwb_imu_pl::P106GraphSnapshotV1> worst_attitude_snapshot;
    double worst_position_error = -1.0;
    double worst_attitude_error = -1.0;
    uwb_imu_pl::ImuMeasurement boundary;
    boundary.timestamp = initial.timestamp;
    boundary.specific_force_mps2 = orientation(0, scenario.degenerate).conjugate() *
        (acceleration(0, scenario.degenerate) +
         Eigen::Vector3d(0, 0, config.imu.gravity_mps2));
    boundary.angular_velocity_radps = angularVelocity(0, scenario.degenerate);
    pipeline.ingestImu(boundary);
    std::mt19937_64 random(config.seed);
    std::normal_distribution<double> normal;
    uwb_imu_pl::RunSummary summary;
    uwb_imu_pl::NumericalWorkCounters::reset();
    std::vector<double> candidate_wall_ms;
    const bool force_candidate_stress =
        std::getenv("UWB_IMU_PL_BENCHMARK_FORCE_ALARM") != nullptr;
    const int fault_begin = schedule.begin;
    const int fault_end = schedule.end;
    std::uint64_t fault_sequence = 0;
    summary.status = "IMPLEMENTED_UNVERIFIED";
    std::optional<std::chrono::steady_clock::time_point> recovery_start;
    std::optional<double> recovery_latency_ms;
    for (int epoch_index = 0; epoch_index < epochs; ++epoch_index) {
      const double time = (epoch_index+1)*.05;
      for (int sample = 1; sample <= 10; ++sample) {
        const double sample_time = epoch_index*.05 + sample*.005;
        uwb_imu_pl::ImuMeasurement imu;
        imu.id = uwb_imu_pl::MeasurementId(epoch_index*10+sample);
        imu.timestamp = uwb_imu_pl::TimestampNs(
            static_cast<std::int64_t>(std::llround(sample_time*1e9)));
        const auto truth_q = orientation(sample_time, scenario.degenerate);
        imu.specific_force_mps2 = truth_q.conjugate() *
            (acceleration(sample_time, scenario.degenerate) +
             Eigen::Vector3d(0, 0, config.imu.gravity_mps2));
        imu.angular_velocity_radps = angularVelocity(sample_time, scenario.degenerate);
        const bool fault_active = epoch_index >= fault_begin && epoch_index <= fault_end;
        if (!recovery_start && fault_active &&
            (scenario.imu_accel || scenario.imu_gyro))
          recovery_start = std::chrono::steady_clock::now();
        if (fault_active && scenario.imu_accel)
          imu.specific_force_mps2(scenario.axis) += scenario.accel_bias_mps2;
        if (fault_active && scenario.imu_gyro)
          imu.angular_velocity_radps(scenario.axis) += scenario.gyro_bias_radps;
        for (int axis = 0; axis < 3; ++axis) {
          if (!scenario.noiseless) {
            imu.specific_force_mps2(axis) +=
                kPhysicalNoiseFraction*config.imu.accelerometer_sigma/
                std::sqrt(kImuDt)*normal(random);
            imu.angular_velocity_radps(axis) +=
                kGyroPhysicalNoiseFraction*config.imu.gyroscope_sigma/
                std::sqrt(kImuDt)*normal(random);
          }
        }
        pipeline.ingestImu(imu);
      }
      const bool fault_active = epoch_index >= fault_begin && epoch_index <= fault_end;
      if (fault_active && (scenario.imu_accel || scenario.imu_gyro)) {
        uwb_imu_pl::FaultTruthRecord truth;
        truth.timestamp = uwb_imu_pl::TimestampNs(
            static_cast<std::int64_t>(std::llround(time*1e9)));
        truth.sequence = ++fault_sequence; truth.active = true;
        truth.sensor_type = "IMU";
        truth.fault_mode = scenario.imu_accel ? "ACCEL_AXIS_BIAS" : "GYRO_AXIS_BIAS";
        truth.fault_kind = scenario.imu_accel ? "accel_axis_interval_bias" :
                                               "gyro_axis_interval_bias";
        truth.axis = scenario.axis; truth.epoch_begin = fault_begin + 1;
        truth.epoch_end = fault_end + 1;
        truth.injected_value = scenario.imu_accel ? scenario.accel_bias_mps2 :
                                                   scenario.gyro_bias_radps;
        truth.injected_units = scenario.imu_accel ? "m/s^2" : "rad/s";
        logger.writeFaultTruth(truth);
      }
      uwb_imu_pl::UwbBatch batch;
      batch.id = uwb_imu_pl::BatchId(epoch_index+1);
      batch.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>(std::llround(time*1e9)));
      batch.covariance_model_id = "week4_performance_diagonal";
      for (const auto& anchor : config.anchors) {
        uwb_imu_pl::UwbMeasurement measurement;
        measurement.id = uwb_imu_pl::MeasurementId(
            static_cast<std::uint64_t>(epoch_index+1)*100 + anchor.id.value());
        measurement.factor_id = uwb_imu_pl::FactorId(measurement.id.value());
        measurement.anchor_id = anchor.id;
        measurement.timestamp = batch.timestamp;
        measurement.anchor_position_m = anchor.position_world_m;
        measurement.sigma_m = config.realtime.range_sigma_m;
        const Eigen::Vector3d tag_position = position(time, scenario.degenerate) +
            orientation(time, scenario.degenerate) * config.realtime.lever_arm_body_m;
        const double true_range = (tag_position-anchor.position_world_m).norm();
        measurement.range_m = true_range +
            (scenario.noiseless ? 0.0 : kPhysicalNoiseFraction*
                                            measurement.sigma_m*normal(random));
        const bool inject_uwb = fault_active && scenario.uwb &&
            (scenario.persistent_reject || anchor.id == config.anchors.front().id);
        if (inject_uwb) {
          const double bias = scenario.persistent_reject ? 5.0 :
                                                            scenario.uwb_bias_m;
          measurement.range_m += bias;
          uwb_imu_pl::FaultTruthRecord truth;
          truth.timestamp = batch.timestamp; truth.sequence = ++fault_sequence;
          truth.anchor_id = anchor.id; truth.fault_mode = "RAW_RANGE_BIAS";
          truth.active = true; truth.injected_bias_m = bias;
          truth.true_range_m = true_range; truth.epoch_begin = fault_begin + 1;
          truth.epoch_end = fault_end + 1; truth.injected_value = bias;
          logger.writeFaultTruth(truth);
        }
        if (force_candidate_stress && epoch_index >= 60 &&
            anchor.id == config.anchors.front().id) {
          measurement.range_m += 5.0;
        }
        batch.measurements.push_back(measurement);
      }
      const std::uint64_t marginalizations_before = estimator.marginalizationCount();
      const auto start = std::chrono::steady_clock::now();
      if (!recovery_start && (scenario.uwb || scenario.imu_accel || scenario.imu_gyro) &&
          epoch_index == fault_begin) recovery_start = start;
      uwb_imu_pl::IntegrityOutput result;
      uwb_imu_pl::AttemptProofLease proof_lease;
      try {
        result = pipeline.processUwbBatch(batch, &proof_lease);
      } catch (...) {
        logger.writeIntegrity(pipeline.lastAttemptOutput());
        throw;
      }
      const auto analysis_complete = std::chrono::steady_clock::now();
      const double core_ms = std::chrono::duration<double, std::milli>(
          analysis_complete-start).count();
      // Keep an explicit boundary even though this synchronous harness has no
      // asynchronous continuation today.  Acceptance must read this sample,
      // not silently alias core_total in the summarizer.
      const double analysis_completion_ms =
          std::chrono::duration<double, std::milli>(
              analysis_complete-start).count();
      for (auto& stage : result.stage_timings) {
        if (stage.stage == "core_total") stage.wall_ms = core_ms;
      }
      for (const auto& candidate : result.candidate_audit) {
        if (std::isfinite(candidate.wall_ms)) {
          candidate_wall_ms.push_back(candidate.wall_ms);
        }
      }
      const auto outer_start = std::chrono::steady_clock::now();
      logger.writeState(result.state);
      const auto steady_ns = [](const std::chrono::steady_clock::time_point& point) {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                point.time_since_epoch()).count());
      };
      // This is the sole integrity-output handoff.  The deadline flag and the
      // reported arrival-to-publish denominator use this exact call sample;
      // analysis completion is deliberately not a publication timestamp.
      const auto publish_call = std::chrono::steady_clock::now();
      uwb_imu_pl::FinalPacketTiming final_timing;
      final_timing.arrival_steady_ns = steady_ns(start);
      final_timing.compute_done_steady_ns = steady_ns(analysis_complete);
      final_timing.packet_ready_steady_ns = final_timing.compute_done_steady_ns;
      final_timing.publish_call_steady_ns = steady_ns(publish_call);
      final_timing.deadline_boundary =
          uwb_imu_pl::FinalPacketBoundary::PublishCall;
      const auto final_packet = uwb_imu_pl::finalizeOutputPacket(
          std::move(result), final_timing, 50000000ULL);
      result = final_packet.output();
      if (run_kkt_diagnostic) {
        const Eigen::Vector3d truth_position =
            position(time, scenario.degenerate);
        const Eigen::Quaterniond truth_orientation =
            orientation(time, scenario.degenerate);
        const double position_error =
            (result.state.position_world_m - truth_position).norm();
        const double quaternion_dot = std::min(
            1.0, std::abs(result.state.q_world_body.normalized().dot(
                     truth_orientation.normalized())));
        const double attitude_error = 2.0 * std::acos(quaternion_dot);
        if (position_error > worst_position_error) {
          worst_position_error = position_error;
          worst_position_snapshot =
              uwb_imu_pl::P106ReadOnlyGraphSnapshotPeer::capture(
                  estimator, static_cast<std::size_t>(epoch_index + 1),
                  position_error, attitude_error, truth_position,
                  truth_orientation);
        }
        if (attitude_error > worst_attitude_error) {
          worst_attitude_error = attitude_error;
          worst_attitude_snapshot =
              uwb_imu_pl::P106ReadOnlyGraphSnapshotPeer::capture(
                  estimator, static_cast<std::size_t>(epoch_index + 1),
                  position_error, attitude_error, truth_position,
                  truth_orientation);
        }
      }
      logger.writeIntegrity(result, final_packet.metadata());
      const auto publish_return = std::chrono::steady_clock::now();
      terminal_packets << result.diagnostics.input_attempt_id << ','
          << result.transaction_id << ',' << result.window_id << ','
          << result.selected_action_id << ','
          << result.diagnostics.backend_epoch_before << ','
          << result.diagnostics.backend_epoch_after << ','
          << result.batch_committed << ','
          << result.diagnostics.risk_ledger_closes << ','
          << result.diagnostics.risk_ledger_all_validated << ','
          << result.diagnostics.selection_risk_proof_id << ','
          << result.publication.risk_proof_id << ','
          << result.publication.certificate_id << ','
          << result.publication.protected_output << ','
          << result.protection_level.formal_eligible << ','
          << result.deadline_missed << ',' << steady_ns(publish_call) << ','
          << steady_ns(publish_return) << ','
          << steady_ns(publish_call) - steady_ns(start) << ','
          << final_packet.metadata().digest << '\n';
      logger.writeGroundTruth({batch.timestamp,
                               position(time, scenario.degenerate),
                               orientation(time, scenario.degenerate)});
      const std::size_t epoch = estimator.currentEpoch();
      auto timing = [&](const std::string& stage, double value, bool success=true) {
        uwb_imu_pl::TimingRecord record;
        record.input_attempt_id = result.diagnostics.input_attempt_id;
        record.transaction_id = result.transaction_id; record.window_id = result.window_id;
        record.timestamp = result.timestamp; record.epoch = epoch;
        record.stage = stage; record.wall_ms = value;
        record.problem_size = batch.measurements.size();
        record.hypothesis_count = result.sensitivities.size();
        record.factor_count = estimator.factorCount();
        record.cold = epoch_index < 100;
        record.success = success; logger.writeTiming(record);
      };
      timing("imu_preintegration", estimator.lastImuPreintegrationMs());
      timing("no_uwb_isam_update", estimator.lastNoUwbUpdateMs());
      timing("state_query", estimator.lastStateQueryMs());
      timing("current_joint_marginal", estimator.lastMarginalMs());
      timing("snapshot_extraction", estimator.lastSnapshotExtractionMs());
      for (const auto& stage : result.stage_timings) {
        if (stage.stage != "core_total" && stage.status == "EXECUTED")
          timing(stage.stage, stage.wall_ms, stage.success);
      }
      timing(result.batch_committed ? "uwb_commit" : "uwb_reject",
             estimator.lastUwbUpdateMs());
      timing("core_total", core_ms);
      logger.writeEvent(result.timestamp,
                        result.batch_committed ? "UWB_COMMIT" : "UWB_REJECT",
                        result.detector.reason);
      if (estimator.marginalizationCount() > marginalizations_before) {
        logger.writeEvent(result.timestamp, "FIXED_LAG_MARGINALIZE",
            "oldest_retained_epoch=" + std::to_string(estimator.oldestRetainedEpoch()) +
            ";retained_epochs=" + std::to_string(estimator.retainedEpochs()) +
            ";active_values=" + std::to_string(estimator.activeValueCount()) +
            ";active_factors=" + std::to_string(estimator.factorCount()));
      }
      const double arrival_to_publish_ms =
          std::chrono::duration<double, std::milli>(
              publish_call - start).count();
      logger.flush();
      const auto flush_complete = std::chrono::steady_clock::now();
      const double logging_flush_ms = std::chrono::duration<double, std::milli>(
          flush_complete - outer_start).count();
      const double outer_epoch_ms = std::chrono::duration<double, std::milli>(
          flush_complete - start).count();
      timing("outer_logging_flush", logging_flush_ms);
      timing("outer_epoch", outer_epoch_ms);
      timing("analysis_completion", analysis_completion_ms);
      timing("arrival_to_publish", arrival_to_publish_ms);
      // This synchronous runner emits its first definitive safety result at
      // publish_call. Completion and publication remain separate boundaries;
      // none of these aliases may be summed with the legacy parent timers.
      timing("safety_decision_latency", arrival_to_publish_ms);
      timing("analysis_completion_latency", analysis_completion_ms);
      timing("core_compute", core_ms);
      if (recovery_start && !recovery_latency_ms &&
          result.publication.protected_output) {
        recovery_latency_ms = std::chrono::duration<double, std::milli>(
            publish_call - *recovery_start).count();
        timing("recovery_latency", *recovery_latency_ms);
      }
      logger.flush();
      summary.processed++;
      summary.committed += result.batch_committed;
      summary.rejected += !result.batch_committed;
      summary.core_total_ms += core_ms;
      summary.end_to_end_total_ms += outer_epoch_ms;
    }
    summary.detail = "deterministic 3D rotating raw-stream benchmark; scenario=" +
                     scenario.name + "; seed=" + std::to_string(config.seed);
    // An unavailable result is a safety decision, not successful recovery.
    // Null/censored measurements must never be reported as zero latency.
    std::ofstream recovery_file(std::string(argv[2]) + "/recovery_latency.json");
    recovery_file << "{\"definition\":\"first_fault_input_to_first_protected_publication_wall_clock\","
                  << "\"status\":\"" << (recovery_latency_ms ? "OBSERVED" :
                      (recovery_start ? "RIGHT_CENSORED" : "NO_FAULT_INPUT"))
                  << "\",\"recovery_latency_ms\":";
    if (recovery_latency_ms) recovery_file << std::setprecision(17) << *recovery_latency_ms;
    else recovery_file << "null";
    recovery_file << "}\n";
    if (!candidate_wall_ms.empty()) {
      std::sort(candidate_wall_ms.begin(), candidate_wall_ms.end());
      auto percentile = [&](double p) {
        const std::size_t index = static_cast<std::size_t>(std::ceil(
            p * static_cast<double>(candidate_wall_ms.size()))) - 1;
        return candidate_wall_ms[std::min(index, candidate_wall_ms.size() - 1)];
      };
      std::cout << "candidate_wall_ms p50=" << percentile(0.50)
                << " p95=" << percentile(0.95)
                << " p99=" << percentile(0.99)
                << " max=" << candidate_wall_ms.back() << '\n';
    }
    logger.writeSummary(summary);
    const auto work = uwb_imu_pl::NumericalWorkCounters::snapshot();
    uwb_imu_pl::detail::writeNumericalPhaseProfile(std::cout);
    std::cout << "numerical_work base_svd=" << work.base_svd
              << " base_llt=" << work.base_llt
              << " base_state_solves=" << work.base_state_solves
              << " llt_state_solve_calls=" << work.llt_state_solve_calls
              << " svd_state_solve_calls=" << work.svd_state_solve_calls
              << " detector_reference_qr=" << work.detector_reference_qr
              << " candidate_reference_svd=" << work.candidate_reference_svd
              << " candidate_inner_llt=" << work.candidate_inner_llt
              << " covariance_rhs_solves=" << work.covariance_rhs_solves
              << " covariance_rhs_columns=" << work.covariance_rhs_columns
              << " spectral_rhs_solves=" << work.spectral_rhs_solves
              << " spectral_rhs_columns=" << work.spectral_rhs_columns
              << " numerical_contract_mismatches="
              << work.numerical_contract_mismatches
              << " imu_oracle_reintegrations=" << work.imu_oracle_reintegrations
              << " window_content_hash_scans="
              << work.window_content_hash_scans
              << " frozen_identity_builds=" << work.frozen_identity_builds
              << " frozen_identity_reuses=" << work.frozen_identity_reuses
              << " frozen_admission_constant_validations="
              << work.frozen_admission_constant_validations
              << " descriptor_id_lookups=" << work.descriptor_id_lookups
              << " descriptor_linear_scans="
              << work.descriptor_linear_scans
              << " block_rhs_solve_batches="
              << work.block_rhs_solve_batches
              << " block_rhs_unique_blocks="
              << work.block_rhs_unique_blocks
              << " pl_payload_validations=" << work.pl_payload_validations
              << " pl_payload_validation_reuses=" << work.pl_payload_validation_reuses
              << " frozen_hypothesis_validations=" << work.frozen_hypothesis_validations
              << " frozen_hypothesis_validation_reuses=" << work.frozen_hypothesis_validation_reuses
              << '\n';
    const auto history_root = estimator.historyRootCacheAuditForTesting();
    std::cout << "history_root_work requests=" << history_root.requests
              << " exact_hits=" << history_root.exact_hits
              << " incremental_path_updates="
              << history_root.incremental_path_updates
              << " incremental_add_paths="
              << history_root.incremental_add_paths
              << " incremental_remove_paths="
              << history_root.incremental_remove_paths
              << " incremental_relinearize_paths="
              << history_root.incremental_relinearize_paths
              << " internal_nodes_recomputed="
              << history_root.internal_nodes_recomputed
              << " full_tree_rebuilds=" << history_root.full_tree_rebuilds
              << " full_oracle_checks=" << history_root.full_oracle_checks
              << " full_oracle_mismatches="
              << history_root.full_oracle_mismatches
              << " history_root_fallback_count="
              << history_root.full_oracle_mismatches
              << " reused_groups=" << history_root.reused_groups
              << " rebuilt_groups=" << history_root.rebuilt_groups
              << " factor_group_hits=" << history_root.factor_group_hits
              << " factor_group_misses=" << history_root.factor_group_misses
              << " retained_rows=" << history_root.retained_rows
              << " retained_bytes=" << history_root.retained_bytes
              << " last_reason=" << history_root.last_reason
              << '\n';
    if (run_kkt_diagnostic) {
      if (!worst_position_snapshot || !worst_attitude_snapshot) {
        throw std::runtime_error("KKT diagnostic captured no graph snapshot");
      }
      const P106KktAudit position_audit =
          auditSnapshot(*worst_position_snapshot);
      const bool same_snapshot =
          worst_position_snapshot->attempt == worst_attitude_snapshot->attempt;
      const P106KktAudit attitude_audit = same_snapshot
          ? position_audit : auditSnapshot(*worst_attitude_snapshot);
      const std::string kkt_path = std::string(argv[2]) +
          "/kkt_diagnostic.json";
      std::ofstream audit_file(kkt_path);
      if (!audit_file) {
        throw std::runtime_error("cannot create kkt_diagnostic.json");
      }
      audit_file << std::setprecision(17)
                 << "{\n"
                 << "  \"schema\": \"uwb-imu-pl/targeted-kkt-replay/v1\",\n"
                 << "  \"source_revision\": \"" << UWB_IMU_PL_GIT_SHA << "\",\n"
                 << "  \"source_dirty\": "
                 << (UWB_IMU_PL_GIT_DIRTY != 0 ? "true" : "false") << ",\n"
                 << "  \"seed\": " << config.seed << ",\n"
                 << "  \"requested_attempts\": " << epochs << ",\n"
                 << "  \"scenario\": \"" << scenario.name << "\",\n"
                 << "  \"snapshots\": {\n";
      writeKktSnapshot(audit_file, "worst_position",
                       *worst_position_snapshot, position_audit, true);
      writeKktSnapshot(audit_file, "worst_attitude",
                       *worst_attitude_snapshot, attitude_audit, false);
      audit_file << "  }\n}\n";
      audit_file.close();
      std::cout << "p106_kkt_diagnostic path=" << kkt_path
                << " worst_position_attempt="
                << worst_position_snapshot->attempt
                << " worst_attitude_attempt="
                << worst_attitude_snapshot->attempt << '\n';
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}
