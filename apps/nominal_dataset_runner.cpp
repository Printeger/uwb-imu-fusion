#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/estimation_tuning.hpp"
#include "uwb_imu_pl/estimation/causal_initializer.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <tbb/global_control.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Existing read-only diagnostic friend; the snapshot never updates production.
namespace uwb_imu_pl {
struct P106ReadOnlyGraphSnapshotPeer {
  static std::pair<gtsam::NonlinearFactorGraph,gtsam::Values> capture(
      const IncrementalUwbImuEstimator& estimator) {
    return {estimator.activeGraph(),estimator.backendIsam().calculateEstimate()};
  }
};
}
namespace {

using Clock = std::chrono::steady_clock;
using uwb_imu_pl::AnchorId;
using uwb_imu_pl::BatchId;
using uwb_imu_pl::EpochCommitPlan;
using uwb_imu_pl::EpochPreparationOptions;
using uwb_imu_pl::FactorId;
using uwb_imu_pl::ImuMeasurement;
using uwb_imu_pl::MeasurementId;
using uwb_imu_pl::NavigationState;
using uwb_imu_pl::TagId;
using uwb_imu_pl::TimestampNs;
using uwb_imu_pl::UwbBatch;
using uwb_imu_pl::UwbMeasurement;

double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

struct CsvTable {
  std::vector<std::string> header;
  std::unordered_map<std::string, std::size_t> column;
  std::vector<std::vector<std::string>> rows;
};

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    if (!field.empty() && field.back() == '\r') field.pop_back();
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

CsvTable readCsv(const boost::filesystem::path& path) {
  std::ifstream stream(path.string());
  if (!stream) throw std::runtime_error("cannot open CSV: " + path.string());
  CsvTable table;
  std::string line;
  if (!std::getline(stream, line)) throw std::runtime_error("empty CSV: " + path.string());
  table.header = split(line);
  for (std::size_t i = 0; i < table.header.size(); ++i) table.column[table.header[i]] = i;
  while (std::getline(stream, line)) {
    if (line.empty()) continue;
    auto fields = split(line);
    if (fields.size() != table.header.size()) {
      throw std::runtime_error("malformed CSV row in " + path.string());
    }
    table.rows.push_back(std::move(fields));
  }
  return table;
}

const std::string& value(const CsvTable& table, const std::vector<std::string>& row,
                         const std::string& key) {
  const auto found = table.column.find(key);
  if (found == table.column.end()) throw std::runtime_error("missing CSV column: " + key);
  return row.at(found->second);
}

double real(const CsvTable& table, const std::vector<std::string>& row,
            const std::string& key) {
  const double result = std::stod(value(table, row, key));
  if (!std::isfinite(result)) throw std::runtime_error("non-finite CSV value: " + key);
  return result;
}

std::uint64_t integer(const CsvTable& table, const std::vector<std::string>& row,
                      const std::string& key) {
  return std::stoull(value(table, row, key));
}

struct RangeRow {
  double time_s = 0.0;
  std::uint64_t source_message = 0;
  std::uint64_t source_range = 0;
  std::uint64_t tag_id = 0;
  std::uint64_t anchor_id = 0;
  double range_m = 0.0;
  double sigma_m = 0.0;
  Eigen::Vector3d lever = Eigen::Vector3d::Zero();
  Eigen::Vector3d anchor = Eigen::Vector3d::Zero();
  bool valid = false;
};

std::vector<ImuMeasurement> loadImu(const boost::filesystem::path& path) {
  const CsvTable table = readCsv(path);
  std::vector<ImuMeasurement> output;
  output.reserve(table.rows.size());
  std::uint64_t id = 1;
  for (const auto& row : table.rows) {
    ImuMeasurement measurement;
    measurement.id = MeasurementId(id++);
    measurement.timestamp = TimestampNs::fromSeconds(real(table, row, "t"));
    measurement.specific_force_mps2 = {real(table, row, "ax"), real(table, row, "ay"),
                                       real(table, row, "az")};
    measurement.angular_velocity_radps = {real(table, row, "gx"), real(table, row, "gy"),
                                          real(table, row, "gz")};
    output.push_back(measurement);
  }
  if (output.size() < 2) throw std::runtime_error("insufficient IMU samples");
  return output;
}

std::vector<RangeRow> loadRanges(const boost::filesystem::path& path) {
  const CsvTable table = readCsv(path);
  std::vector<RangeRow> output;
  output.reserve(table.rows.size());
  for (const auto& row : table.rows) {
    RangeRow item;
    item.time_s = real(table, row, "t");
    item.source_message = integer(table, row, "source_message");
    item.source_range = integer(table, row, "source_range");
    item.tag_id = integer(table, row, "tag_id");
    item.anchor_id = integer(table, row, "anchor_id");
    item.range_m = real(table, row, "range");
    item.sigma_m = real(table, row, "sigma");
    item.lever = {real(table, row, "lever_x"), real(table, row, "lever_y"),
                  real(table, row, "lever_z")};
    item.anchor = {real(table, row, "anchor_x"), real(table, row, "anchor_y"),
                   real(table, row, "anchor_z")};
    item.valid = integer(table, row, "valid") != 0;
    if (item.valid && item.range_m > 0.0 && item.sigma_m > 0.0) output.push_back(item);
  }
  if (output.empty()) throw std::runtime_error("no valid UWB observations");
  return output;
}

Eigen::Vector3d bootstrapTagPosition(const std::vector<RangeRow>& ranges,
                                     std::uint64_t primary_tag,
                                     bool prefer_below_anchors) {
  std::map<std::uint64_t, std::vector<const RangeRow*>> by_anchor;
  const double start = ranges.front().time_s;
  for (const auto& row : ranges) {
    if (row.tag_id == primary_tag && row.time_s <= start + 1.0) {
      by_anchor[row.anchor_id].push_back(&row);
    }
  }
  if (by_anchor.size() < 4) {
    by_anchor.clear();
    for (const auto& row : ranges) {
      if (row.tag_id == primary_tag && row.time_s <= start + 3.0)
        by_anchor[row.anchor_id].push_back(&row);
    }
  }
  if (by_anchor.size() < 4) throw std::runtime_error("BOOTSTRAP_FEWER_THAN_FOUR_ANCHORS");
  struct Observation { Eigen::Vector3d anchor; double range; };
  std::vector<Observation> observations;
  for (const auto& entry : by_anchor) {
    std::vector<double> values;
    for (const auto* row : entry.second) values.push_back(row->range_m);
    std::nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    observations.push_back({entry.second.front()->anchor, values[values.size()/2]});
  }
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto& obs : observations) center += obs.anchor;
  center /= static_cast<double>(observations.size());
  double scale = 1.0;
  for (const auto& obs : observations)
    scale = std::max(scale, (obs.anchor - center).norm());
  std::vector<Eigen::Vector3d> seeds{center};
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d positive = center, negative = center;
    positive(axis) += 2.0 * scale;
    negative(axis) -= 2.0 * scale;
    seeds.push_back(positive);
    seeds.push_back(negative);
  }
  auto optimize = [&](Eigen::Vector3d position) {
   double damping = 1e-3;
   for (int iteration = 0; iteration < 100; ++iteration) {
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    double before = 0.0;
    for (const auto& obs : observations) {
      const Eigen::Vector3d delta = position - obs.anchor;
      const double predicted = std::max(1e-9, delta.norm());
      const double residual = predicted - obs.range;
      const Eigen::RowVector3d jacobian = delta.transpose() / predicted;
      hessian += jacobian.transpose() * jacobian;
      gradient += jacobian.transpose() * residual;
      before += residual * residual;
    }
    const Eigen::Vector3d step = -(hessian + damping * Eigen::Matrix3d::Identity()).ldlt().solve(gradient);
    const Eigen::Vector3d candidate = position + step;
    double after = 0.0;
    for (const auto& obs : observations) {
      const double residual = (candidate - obs.anchor).norm() - obs.range;
      after += residual * residual;
    }
    if (std::isfinite(after) && after < before) {
      position = candidate;
      damping *= 0.5;
      if (step.norm() < 1e-8) break;
    } else {
      damping *= 10.0;
    }
   }
   double cost = 0.0;
   for (const auto& obs : observations) {
     const double residual = (position - obs.anchor).norm() - obs.range;
     cost += residual * residual;
   }
   return std::make_pair(position, cost);
  };
  std::vector<std::pair<Eigen::Vector3d, double>> solutions;
  for (const auto& seed : seeds) solutions.push_back(optimize(seed));
  const double best_cost = std::min_element(
      solutions.begin(), solutions.end(),
      [](const auto& a, const auto& b) { return a.second < b.second; })->second;
  auto selected = std::min_element(
      solutions.begin(), solutions.end(),
      [&](const auto& a, const auto& b) {
        const bool a_admissible = a.second <= best_cost + 1e-4 * observations.size();
        const bool b_admissible = b.second <= best_cost + 1e-4 * observations.size();
        if (prefer_below_anchors && a_admissible && b_admissible &&
            std::abs(a.first.z() - b.first.z()) > 1e-6)
          return a.first.z() < b.first.z();
        return a.second < b.second;
      });
  if (selected == solutions.end() || !selected->first.allFinite())
    throw std::runtime_error("BOOTSTRAP_NONFINITE_POSITION");
  return selected->first;
}

NavigationState bootstrapState(const std::vector<ImuMeasurement>& imu,
                               const std::vector<RangeRow>& ranges,
                               std::uint64_t primary_tag, double gravity,
                               bool prefer_below_anchors,
                               Eigen::Vector3d* primary_lever) {
  const double start = imu.front().timestamp.seconds();
  Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
  std::size_t count = 0;
  for (const auto& sample : imu) {
    if (sample.timestamp.seconds() > start + 2.0) break;
    mean_accel += sample.specific_force_mps2;
    mean_gyro += sample.angular_velocity_radps;
    ++count;
  }
  if (count < 10 || mean_accel.norm() < 1e-6)
    throw std::runtime_error("BOOTSTRAP_INSUFFICIENT_IMU");
  mean_accel /= static_cast<double>(count);
  mean_gyro /= static_cast<double>(count);
  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
      mean_accel.normalized(), Eigen::Vector3d::UnitZ());
  q.normalize();
  *primary_lever = Eigen::Vector3d::Zero();
  for (const auto& range : ranges) {
    if (range.tag_id == primary_tag) { *primary_lever = range.lever; break; }
  }
  NavigationState state;
  state.id = uwb_imu_pl::StateId(0);
  state.timestamp = imu.front().timestamp;
  state.q_world_body = q;
  state.position_world_m = bootstrapTagPosition(
      ranges, primary_tag, prefer_below_anchors) - q * *primary_lever;
  state.velocity_world_mps.setZero();
  state.gyro_bias_radps = mean_gyro;
  state.accel_bias_mps2 = mean_accel - q.conjugate() * (gravity * Eigen::Vector3d::UnitZ());
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(state.gyro_bias_radps(axis)) < 1e-12)
      state.gyro_bias_radps(axis) = 0.0;
    if (std::abs(state.accel_bias_mps2(axis)) < 1e-12)
      state.accel_bias_mps2(axis) = 0.0;
  }
  return state;
}

std::vector<UwbBatch> batches(const std::vector<RangeRow>& ranges,
                              std::uint64_t primary_tag, bool all_tags,
                              double batch_window_s) {
  std::vector<UwbBatch> output;
  std::size_t begin = 0;
  std::uint64_t measurement_id = 1000000;
  while (begin < ranges.size()) {
    std::size_t end = begin + 1;
    // Some datasets serialize one anchor reading per ROS message/CSV row even
    // though four or more radios belong to the same physical ranging epoch.
    // Preserve native multi-range messages and coalesce adjacent single-range
    // records only inside the estimator's audited 10 ms skew allowance.
    while (end < ranges.size() &&
           (ranges[end].source_message == ranges[begin].source_message ||
            ranges[end].time_s - ranges[begin].time_s <= batch_window_s + 1e-12)) {
      ++end;
    }
    UwbBatch batch;
    batch.id = BatchId(ranges[begin].source_message + 1);
    batch.timestamp = TimestampNs::fromSeconds(ranges[end - 1].time_s);
    batch.covariance_model_id = "benchmark_per_measurement_diagonal";
    for (std::size_t index = begin; index < end; ++index) {
      const auto& row = ranges[index];
      if (!all_tags && row.tag_id != primary_tag) continue;
      UwbMeasurement measurement;
      measurement.id = MeasurementId(measurement_id++);
      measurement.factor_id = FactorId(measurement.id.value());
      measurement.anchor_id = AnchorId(row.anchor_id);
      measurement.tag_id = TagId(row.tag_id);
      measurement.timestamp = TimestampNs::fromSeconds(row.time_s);
      measurement.range_m = row.range_m;
      measurement.sigma_m = row.sigma_m;
      measurement.anchor_position_m = row.anchor;
      measurement.lever_arm_body_m = row.lever;
      measurement.sequence = row.source_message;
      batch.measurements.push_back(measurement);
    }
    if (!batch.measurements.empty()) output.push_back(std::move(batch));
    begin = end;
  }
  return output;
}

void writeStatus(const boost::filesystem::path& output, const std::string& status,
                 const std::string& reason, std::size_t epochs, double wall_ms,
                 double cpu_ms, double adapter_ms, double bootstrap_ms) {
  std::string escaped;
  escaped.reserve(reason.size());
  for (const char character : reason) {
    switch (character) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += character;
    }
  }
  std::ofstream stream((output / "run_status.json").string());
  stream << "{\n  \"schema\": \"nominal-dataset-run/v1\",\n"
         << "  \"status\": \"" << status << "\",\n"
         << "  \"reason\": \"" << escaped << "\",\n"
         << "  \"epochs\": " << epochs << ",\n"
         << "  \"wall_ms\": " << std::setprecision(17) << wall_ms << ",\n"
         << "  \"cpu_ms\": " << cpu_ms << ",\n"
         << "  \"adapter_ms\": " << adapter_ms << ",\n"
         << "  \"bootstrap_ms\": " << bootstrap_ms << ",\n"
         << "  \"fde_constructed\": false,\n  \"pl_constructed\": false,\n"
         << "  \"initialization_provenance\": \"measurement_bootstrap\"\n}\n";
}

int run(const boost::filesystem::path& config_path,
        const boost::filesystem::path& input,
        const boost::filesystem::path& output,
        const std::string& tag_policy, bool export_historical,
        const std::string& oracle_times) {
  const auto wall_start = Clock::now();
  const std::clock_t cpu_start = std::clock();
  boost::filesystem::create_directories(output);
  std::size_t committed = 0;
  double adapter_ms = 0.0;
  double bootstrap_ms = 0.0;
  try {
    const auto adapter_start = Clock::now();
    auto config = uwb_imu_pl::IntegrityConfigLoader::load(config_path.string());
    const YAML::Node manifest = YAML::LoadFile((input / "manifest.json").string());
    const std::uint64_t primary_tag = manifest["primary_tag"].as<std::uint64_t>();
    const bool prefer_below_anchors = manifest["bootstrap_prefer_below_anchors"]
        ? manifest["bootstrap_prefer_below_anchors"].as<bool>() : false;
    const double batch_window_s = manifest["epoch_batch_window_s"]
        ? manifest["epoch_batch_window_s"].as<double>() : 0.0;
    auto imu = loadImu(input / "imu.csv");
    auto ranges = loadRanges(input / "uwb.csv");
    const auto tuning = uwb_imu_pl::readEstimationTuningV1(config);
    auto uwb_batches = batches(ranges, primary_tag, tag_policy == "all", batch_window_s);
    adapter_ms = elapsedMs(adapter_start);
    const auto bootstrap_start = Clock::now();
    Eigen::Vector3d primary_lever;
    NavigationState initial;
    Eigen::Matrix<double,15,1> initial_priors=config.realtime.prior_sigmas;
    if (tuning.causal_bootstrap) {
      for(const auto& r:ranges)if(r.tag_id==primary_tag){primary_lever=r.lever;break;}
      std::vector<UwbMeasurement> startup_ranges;
      for(const auto& batch:uwb_batches)
        startup_ranges.insert(startup_ranges.end(),batch.measurements.begin(),batch.measurements.end());
      const auto startup=uwb_imu_pl::initializeCausallyV1(
          imu,startup_ranges,config,primary_lever,prefer_below_anchors);
      initial=startup.state; initial_priors=startup.prior_sigmas;
      std::ofstream audit((output/"initialization_audit.yaml").string());
      audit << std::setprecision(17) << "stationary: " << (startup.stationary ? "true" : "false")
            << "\ngyro_rms_radps: " << startup.gyro_rms_radps
            << "\naccel_std_mps2: " << startup.accel_std_mps2
            << "\naccel_norm_error_mps2: " << startup.accel_norm_error_mps2
            << "\nwindow_cost_before: " << startup.window_cost_before
            << "\nwindow_cost_after: " << startup.window_cost_after
            << "\nmotion_status: " << startup.motion_status
            << "\nuwb_speed_upper_mps: " << startup.uwb_speed_upper_mps
            << "\ndiscrete_nodes: " << startup.discrete_nodes
            << "\nrange_factors: " << startup.range_factors
            << "\nmaximum_range_time_error_s: " << startup.maximum_range_time_error_s
            << "\nposition_seed_m: [" << startup.position_seed_m.x()<<", "<<startup.position_seed_m.y()<<", "<<startup.position_seed_m.z()<<"]"
            << "\nposition_prior_source: " << startup.position_prior_source << '\n';
    } else {
      initial = bootstrapState(imu, ranges, primary_tag,
                                             config.imu.gravity_mps2,
                                             prefer_below_anchors,
                                             &primary_lever);
    }
    bootstrap_ms = elapsedMs(bootstrap_start);
    {
      const Eigen::Quaterniond q = initial.q_world_body.normalized();
      std::ofstream bootstrap((output / "bootstrap.json").string());
      bootstrap << std::setprecision(17)
                << "{\n  \"schema\": \"measurement-bootstrap/v1\",\n"
                << "  \"timestamp_s\": " << initial.timestamp.seconds() << ",\n"
                << "  \"position_world_m\": [" << initial.position_world_m.x()
                << ", " << initial.position_world_m.y() << ", "
                << initial.position_world_m.z() << "],\n"
                << "  \"velocity_world_mps\": [" << initial.velocity_world_mps.x()
                << ", " << initial.velocity_world_mps.y() << ", "
                << initial.velocity_world_mps.z() << "],\n"
                << "  \"q_world_body_xyzw\": [" << q.x() << ", " << q.y()
                << ", " << q.z() << ", " << q.w() << "],\n"
                << "  \"accel_bias_mps2\": [" << initial.accel_bias_mps2.x()
                << ", " << initial.accel_bias_mps2.y() << ", "
                << initial.accel_bias_mps2.z() << "],\n"
                << "  \"gyro_bias_radps\": [" << initial.gyro_bias_radps.x()
                << ", " << initial.gyro_bias_radps.y() << ", "
                << initial.gyro_bias_radps.z() << "],\n"
                << "  \"lever_arm_body_m\": [" << primary_lever.x() << ", "
                << primary_lever.y() << ", " << primary_lever.z() << "],\n"
                << "  \"prior_sigmas\": [";
      for(int i=0;i<15;++i)bootstrap << (i ? ", " : "") << initial_priors(i);
      bootstrap << "],\n  \"causal_bootstrap\": " << (tuning.causal_bootstrap ? "true" : "false")
                << ",\n  \"provenance\": \"measurement_only\"\n}\n";
    }
    uwb_imu_pl::IncrementalUwbImuEstimator estimator(config, primary_lever);
    estimator.initialize(initial, initial_priors);
    std::set<double> pending_oracles;
    if(!oracle_times.empty())for(const auto& item:split(oracle_times))pending_oracles.insert(std::stod(item));
    std::ofstream oracle;
    if(!pending_oracles.empty()) {
      oracle.open((output/"local_oracle.csv").string());oracle<<std::setprecision(17);
      oracle<<"requested_t,state_t,online_objective,batch_objective,online_scaled_gradient,batch_scaled_gradient,iterations,batch_ms,online_px,online_py,online_pz,batch_px,batch_py,batch_pz,rotation_change_rad,velocity_change_mps,bias_change,scope\n";
    }
    std::ofstream resolved((output/"resolved_config.yaml").string());
    resolved << config.resolved_yaml;
    std::ofstream config_hash((output/"config_hash.txt").string());
    config_hash << config.config_hash << '\n';
    std::ofstream trajectory((output / "trajectory.tum").string());
    std::ofstream states((output / "states.csv").string());
    std::ofstream timing((output / "timing.csv").string());
    std::ofstream sensor_diagnostics((output/"sensor_diagnostics.csv").string());
    trajectory << std::setprecision(17);
    states << "t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n";
    timing << "epoch,t,imu_preintegration_ms,prepare_wall_ms,no_uwb_update_ms,commit_wall_ms,uwb_update_ms,state_query_wall_ms,state_query_internal_ms,snapshot_extraction_ms,warm_start_ms,epoch_total_ms\n";
    sensor_diagnostics << "t,range_count,prediction_whitened_sq,posterior_whitened_sq,accel_bias_norm,gyro_bias_norm\n";
    // The estimator requires an explicit causal IMU sample at the initialized
    // state timestamp.  Keep all later samples in time order and let the core
    // perform its documented zero-order hold when an epoch falls between IMU
    // samples.
    std::size_t imu_index = 0;
    std::map<std::int64_t,std::pair<NavigationState,TimestampNs>> historical;
    ImuMeasurement boundary=imu.front();
    while(imu_index<imu.size() && !(initial.timestamp<imu[imu_index].timestamp))
      boundary=imu[imu_index++];
    boundary.timestamp=initial.timestamp;
    estimator.ingestImu(boundary);
    for (const auto& batch : uwb_batches) {
      if (tuning.causal_bootstrap ? !(initial.timestamp<batch.timestamp) :
          batch.timestamp.seconds() < initial.timestamp.seconds() + 2.0) continue;
      if (imu.back().timestamp < batch.timestamp) continue;
      const auto epoch_start=Clock::now();
      while (imu_index < imu.size() && !(batch.timestamp < imu[imu_index].timestamp)) {
        estimator.ingestImu(imu[imu_index++]);
      }
      const auto prepare_start = Clock::now();
      auto transaction = estimator.prepareEpoch(batch, EpochPreparationOptions::nominalOnly());
      const auto predicted=transaction.nominal_predicted_state;
      const double prepare_wall = elapsedMs(prepare_start);
      auto plan = EpochCommitPlan::nominalPlan(transaction);
      const auto commit_start = Clock::now();
      estimator.commitEpoch(std::move(transaction), plan);
      const double commit_wall = elapsedMs(commit_start);
      const auto state_start = Clock::now();
      const NavigationState state = estimator.currentState();
      const double state_wall = elapsedMs(state_start);
      const double epoch_total=elapsedMs(epoch_start);
      const Eigen::Vector3d tag_position = state.position_world_m + state.q_world_body * primary_lever;
      const Eigen::Quaterniond q = state.q_world_body.normalized();
      while(!pending_oracles.empty() && *pending_oracles.begin()<=state.timestamp.seconds()) {
        const double requested=*pending_oracles.begin();pending_oracles.erase(pending_oracles.begin());
        const auto snapshot=uwb_imu_pl::P106ReadOnlyGraphSnapshotPeer::capture(estimator);
        const auto& graph=snapshot.first;const auto& values=snapshot.second;
        auto gradient=[&](const gtsam::Values& point) {
          const auto linear=graph.linearize(point)->jacobian();
          const auto g=(linear.first.transpose()*linear.second).eval();
          double scaled=0;
          for(int i=0;i<g.size();++i)scaled=std::max(scaled,std::abs(g(i))/std::max(1e-12,linear.first.col(i).norm()*std::max(1.,linear.second.norm())));
          return scaled;
        };
        gtsam::LevenbergMarquardtParams params;params.maxIterations=100;
        params.maxIterations=1000;params.relativeErrorTol=1e-9;params.absoluteErrorTol=1e-9;
        const auto start=Clock::now();gtsam::LevenbergMarquardtOptimizer optimizer(graph,values,params);
        const auto batch_values=optimizer.optimize();const double wall=elapsedMs(start);
        const auto key=gtsam::Symbol('x',estimator.currentEpoch());
        const auto online_pose=values.at<gtsam::Pose3>(key),batch_pose=batch_values.at<gtsam::Pose3>(key);
        const Eigen::Vector3d op=online_pose.translation()+online_pose.rotation().matrix()*primary_lever;
        const Eigen::Vector3d bp=batch_pose.translation()+batch_pose.rotation().matrix()*primary_lever;
        oracle<<requested<<','<<state.timestamp.seconds()<<','<<graph.error(values)<<','<<graph.error(batch_values)<<','<<gradient(values)<<','<<gradient(batch_values)<<','<<optimizer.iterations()<<','<<wall<<','<<op.x()<<','<<op.y()<<','<<op.z()<<','<<bp.x()<<','<<bp.y()<<','<<bp.z()<<','
          <<gtsam::Rot3::Logmap(online_pose.rotation().between(batch_pose.rotation())).norm()<<','
          <<(values.at<gtsam::Vector3>(gtsam::Symbol('v',estimator.currentEpoch()))-batch_values.at<gtsam::Vector3>(gtsam::Symbol('v',estimator.currentEpoch()))).norm()<<','
          <<(values.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',estimator.currentEpoch())).vector()-batch_values.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b',estimator.currentEpoch())).vector()).norm()<<",SAME_ACTIVE_GRAPH_AND_PRODUCTION_BOUNDARY\n";
      }
      trajectory << state.timestamp.seconds() << ' ' << tag_position.x() << ' '
                 << tag_position.y() << ' ' << tag_position.z() << ' '
                 << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
      states << state.timestamp.seconds() << ',' << state.position_world_m.x() << ','
             << state.position_world_m.y() << ',' << state.position_world_m.z() << ','
             << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
             << state.velocity_world_mps.x() << ',' << state.velocity_world_mps.y() << ','
             << state.velocity_world_mps.z() << ',' << state.accel_bias_mps2.x() << ','
             << state.accel_bias_mps2.y() << ',' << state.accel_bias_mps2.z() << ','
             << state.gyro_bias_radps.x() << ',' << state.gyro_bias_radps.y() << ','
             << state.gyro_bias_radps.z() << '\n';
      timing << committed << ',' << state.timestamp.seconds() << ','
             << estimator.lastImuPreintegrationMs() << ',' << prepare_wall << ','
             << estimator.lastNoUwbUpdateMs() << ',' << commit_wall << ','
             << estimator.lastUwbUpdateMs() << ',' << state_wall << ','
             << estimator.lastStateQueryMs() << ',' << estimator.lastSnapshotExtractionMs() << ','
             << uwb_imu_pl::estimatorNumericsAuditV1(estimator).last_warm_start_ms << ','
             << epoch_total << '\n';
      const uwb_imu_pl::UwbPoseBatchFactor diagnostic(0,batch,primary_lever);
      auto whitened_error=[&](const NavigationState& s) {
        const gtsam::Pose3 pose(gtsam::Rot3(s.q_world_body.toRotationMatrix()),s.position_world_m);
        return diagnostic.noiseModel()->whiten(diagnostic.evaluateError(pose)).squaredNorm();
      };
      sensor_diagnostics << std::setprecision(17) << state.timestamp.seconds() << ','
          << batch.measurements.size() << ',' << whitened_error(predicted) << ','
          << whitened_error(state) << ',' << state.accel_bias_mps2.norm() << ','
          << state.gyro_bias_radps.norm() << '\n';
      if (export_historical) for (const auto& smoothed:estimator.retainedSmoothedStatesV1())
        if(initial.timestamp<smoothed.timestamp)
          historical[smoothed.timestamp.value()]={smoothed,state.timestamp};
      ++committed;
    }
    if(export_historical) {
      std::ofstream history((output/"trajectory_historical_window.tum").string());
      std::ofstream cutoff((output/"historical_information_cutoff.csv").string());
      history << std::setprecision(17); cutoff << std::setprecision(17)
          << "state_timestamp_s,information_cutoff_s,semantics\n";
      for(const auto& item:historical) {
        const auto& s=item.second.first;const auto q=s.q_world_body.normalized();
        const Eigen::Vector3d p=s.position_world_m+q*primary_lever;
        history << s.timestamp.seconds() << ' ' << p.x() << ' ' << p.y() << ' '
            << p.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
        cutoff << s.timestamp.seconds() << ',' << item.second.second.seconds()
            << ",last_retained_accepted_window_state\n";
      }
    }
    if (committed < 2) throw std::runtime_error("FEWER_THAN_TWO_COMMITTED_EPOCHS");
    const auto numerics=uwb_imu_pl::estimatorNumericsAuditV1(estimator);
    std::ofstream audit((output/"numerics_audit.yaml").string());
    audit << "bias_integration_sigmas: [";
    for(int i=0;i<6;++i)audit << (i ? ", " : "") << numerics.bias_integration_sigmas(i);
    audit << "]\nwarm_start_attempts: " << numerics.warm_start_attempts
          << "\nwarm_start_accepted: " << numerics.warm_start_accepted
          << "\nbackend_updates: " << estimator.backendUpdateCount()
          << "\noutput_semantics: online_current_state\n";
    writeStatus(output, "SUCCESS", "", committed, elapsedMs(wall_start),
                1000.0 * (std::clock() - cpu_start) / CLOCKS_PER_SEC,
                adapter_ms, bootstrap_ms);
    return 0;
  } catch (const std::exception& error) {
    writeStatus(output, "FAIL", error.what(), committed, elapsedMs(wall_start),
                1000.0 * (std::clock() - cpu_start) / CLOCKS_PER_SEC,
                adapter_ms, bootstrap_ms);
    std::cerr << error.what() << '\n';
    return 1;
  }
}

}  // namespace

int main(int argc, char** argv) {
  const tbb::global_control thread_limit(tbb::global_control::max_allowed_parallelism, 1);
  namespace po = boost::program_options;
  po::options_description options("nominal_dataset_runner");
  std::string config, input, output, tag_policy;
  std::string oracle_times;
  bool export_historical=false;
  options.add_options()
      ("help,h", "show help")
      ("export-historical", po::bool_switch(&export_historical), "separate smoothing diagnostic with information cutoff")
      ("local-oracle-times", po::value<std::string>(&oracle_times)->default_value(""), "diagnostic batch LM at comma-separated times; does not update online graph")
      ("config", po::value<std::string>(&config)->required(), "integrity config")
      ("input", po::value<std::string>(&input)->required(), "canonical cache")
      ("output", po::value<std::string>(&output)->required(), "run directory")
      ("tag-policy", po::value<std::string>(&tag_policy)->default_value("primary"),
       "primary or all");
  po::variables_map variables;
  try {
    po::store(po::parse_command_line(argc, argv, options), variables);
    if (variables.count("help")) { std::cout << options << '\n'; return 0; }
    po::notify(variables);
    if (tag_policy != "primary" && tag_policy != "all")
      throw std::invalid_argument("tag-policy must be primary or all");
    return run(config, input, output, tag_policy, export_historical, oracle_times);
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n" << options << '\n';
    return 2;
  }
}
