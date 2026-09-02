#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/common/deterministic_event.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uwb_imu_pl/IntegrityStatus.h>
#include <uwb_imu_pl/FaultTruth.h>
#include <uwb_imu_pl/LinktrackNodeframe3.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#ifndef UWB_IMU_PL_GIT_SHA
#define UWB_IMU_PL_GIT_SHA "unknown"
#endif
#ifndef UWB_IMU_PL_GIT_DIRTY
#define UWB_IMU_PL_GIT_DIRTY 1
#endif

namespace {

uwb_imu_pl::TimestampNs timestamp(const ros::Time& time) {
  return uwb_imu_pl::TimestampNs(
      static_cast<std::int64_t>(time.sec) * 1000000000LL + time.nsec);
}

ros::Time rosTime(uwb_imu_pl::TimestampNs time) {
  ros::Time result;
  result.fromNSec(static_cast<std::uint64_t>(time.value()));
  return result;
}

struct Event {
  using Kind = uwb_imu_pl::EventKind;
  Kind kind = Kind::Imu;
  uwb_imu_pl::TimestampNs timestamp;
  std::uint64_t sequence = 0;
  sensor_msgs::Imu imu;
  uwb_imu_pl::LinktrackNodeframe3 uwb;

  uwb_imu_pl::EventOrderKey orderKey() const {
    return {timestamp, kind, sequence};
  }
};

using EventLess = uwb_imu_pl::DeterministicEventLess<Event>;

struct ProcessingMetrics {
  std::uint32_t queue_depth = 0;
  double sensor_to_process_ms = 0.0;
};

double sensorLagMs(uwb_imu_pl::TimestampNs sensor_timestamp) {
  const auto now = ros::Time::now();
  if (now.isZero()) return 0.0;
  const double lag_ms =
      (now.toSec() - sensor_timestamp.seconds()) * 1000.0;
  return std::isfinite(lag_ms) ? std::max(0.0, lag_ms) : 0.0;
}

std::string processCommandLine(int argc, char** argv) {
  std::ostringstream command;
  for (int index = 0; index < argc; ++index) {
    if (index != 0) command << ' ';
    command << argv[index];
  }
  return command.str();
}

std::optional<bool> parseOptionalBool(const std::string& value,
                                      const std::string& name) {
  if (value.empty()) return std::nullopt;
  if (value == "true" || value == "1") return true;
  if (value == "false" || value == "0") return false;
  throw std::runtime_error(name + " must be true or false");
}

std::optional<std::uint64_t> parseOptionalUnsigned(
    const std::string& value, const std::string& name) {
  if (value.empty()) return std::nullopt;
  if (!std::all_of(value.begin(), value.end(), [](unsigned char character) {
        return std::isdigit(character) != 0;
      })) throw std::runtime_error(name + " must be an unsigned integer");
  try {
    return std::stoull(value);
  } catch (const std::exception&) {
    throw std::runtime_error(name + " must be an unsigned integer");
  }
}

class RealtimeNode {
 public:
  RealtimeNode(ros::NodeHandle& node, uwb_imu_pl::IntegrityConfig config,
               std::string run_directory, std::string execution_command)
      : node_(node), config_(std::move(config)) {
    for (const auto& anchor : config_.anchors) {
      anchors_.emplace(anchor.id.value(), anchor);
    }
    if (run_directory.empty()) {
      run_directory = config_.output.root + "/online_" +
          std::to_string(ros::WallTime::now().toNSec());
    }
    logger_.reset(new uwb_imu_pl::RunLogger(
        run_directory, config_.output.write_residuals,
        config_.output.write_timing));
    logger_->writeResolvedConfig(config_.resolved_yaml);
    logger_->writeManifest(uwb_imu_pl::makeRunManifest(
        config_, UWB_IMU_PL_GIT_SHA, UWB_IMU_PL_GIT_DIRTY != 0,
        execution_command));

    odometry_publisher_ = node_.advertise<nav_msgs::Odometry>(
        config_.realtime.odometry_topic, 10);
    integrity_publisher_ = node_.advertise<uwb_imu_pl::IntegrityStatus>(
        config_.realtime.integrity_topic, 10);
    diagnostics_publisher_ = node_.advertise<diagnostic_msgs::DiagnosticArray>(
        config_.realtime.diagnostics_topic, 10);
    imu_subscriber_ = node_.subscribe(
        config_.realtime.imu_topic, 1000, &RealtimeNode::imuCallback, this);
    uwb_subscriber_ = node_.subscribe(
        config_.realtime.uwb_topic, 100, &RealtimeNode::uwbCallback, this);
    ground_truth_subscriber_ = node_.subscribe(
        "/sim/odom", 100, &RealtimeNode::groundTruthCallback, this);
    fault_truth_subscriber_ = node_.subscribe(
        "/uwb_sim/fault_truth", 1000, &RealtimeNode::faultTruthCallback, this);
    worker_ = std::thread(&RealtimeNode::workerLoop, this);
  }

  ~RealtimeNode() {
    stop_.store(true);
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (logger_) {
      summary_.status = summary_.errors == 0 ? "IMPLEMENTED_UNVERIFIED" : "FAILED";
      summary_.detail = "ROS1 realtime node stopped; research only, not certified";
      logger_->writeSummary(summary_);
    }
  }

 private:
  void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& message) {
    uwb_imu_pl::GroundTruthRecord record;
    record.timestamp = timestamp(message->header.stamp);
    record.position_world_m = {message->pose.pose.position.x,
                               message->pose.pose.position.y,
                               message->pose.pose.position.z};
    record.q_world_body = {message->pose.pose.orientation.w,
                           message->pose.pose.orientation.x,
                           message->pose.pose.orientation.y,
                           message->pose.pose.orientation.z};
    logger_->writeGroundTruth(record);
  }

  void faultTruthCallback(const uwb_imu_pl::FaultTruth::ConstPtr& message) {
    uwb_imu_pl::FaultTruthRecord record;
    record.timestamp = timestamp(message->header.stamp);
    record.sequence = message->sequence;
    record.anchor_id = uwb_imu_pl::AnchorId(message->anchor_id);
    record.fault_mode = message->fault_mode;
    record.active = message->active;
    record.outage = message->outage;
    record.injected_bias_m = message->injected_bias_m;
    record.true_range_m = message->true_range_m;
    logger_->writeFaultTruth(record);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    Event event;
    event.kind = Event::Kind::Imu;
    event.timestamp = timestamp(message->header.stamp);
    event.sequence = sequence_.fetch_add(1);
    event.imu = *message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.insert(std::move(event));
      if (latest_received_imu_ < timestamp(message->header.stamp)) {
        latest_received_imu_ = timestamp(message->header.stamp);
      }
    }
    condition_.notify_one();
  }

  void uwbCallback(const uwb_imu_pl::LinktrackNodeframe3::ConstPtr& message) {
    Event event;
    event.kind = Event::Kind::Uwb;
    event.timestamp = timestamp(message->header.stamp);
    event.sequence = sequence_.fetch_add(1);
    event.uwb = *message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.insert(std::move(event));
    }
    condition_.notify_one();
  }

  void initialize(uwb_imu_pl::TimestampNs time,
                  const std::optional<uwb_imu_pl::NavigationState>& seed_state =
                      std::nullopt) {
    uwb_imu_pl::NavigationState initial;
    initial.timestamp = time;
    initial.position_world_m = seed_state
        ? seed_state->position_world_m : config_.realtime.initial_position_m;
    initial.velocity_world_mps = seed_state
        ? seed_state->velocity_world_mps : config_.realtime.initial_velocity_mps;
    if (seed_state) {
      initial.q_world_body = seed_state->q_world_body;
      initial.accel_bias_mps2 = seed_state->accel_bias_mps2;
      initial.gyro_bias_radps = seed_state->gyro_bias_radps;
    }
    estimator_.reset(new uwb_imu_pl::IncrementalUwbImuEstimator(
        config_, config_.realtime.lever_arm_body_m));
    estimator_->initialize(initial, config_.realtime.prior_sigmas);
    monitor_.reset(new uwb_imu_pl::IntegrityMonitor(
        config_.risk, config_.snapshot.rank_tolerance,
        config_.snapshot.max_condition_number));
    pipeline_.reset(new uwb_imu_pl::RealtimeIntegrityPipeline(
        estimator_.get(), *monitor_));
    logger_->writeEvent(time, "INITIALIZED", "first ordered IMU timestamp");
  }

  uwb_imu_pl::UwbBatch convert(const Event& event) const {
    uwb_imu_pl::UwbBatch batch;
    batch.id = uwb_imu_pl::BatchId(event.sequence);
    batch.timestamp = event.timestamp;
    batch.covariance_model_id = "linktrack_frame_diagonal_research";
    std::set<std::uint64_t> seen;
    for (const auto& node : event.uwb.nodes) {
      const auto anchor = anchors_.find(node.id);
      if (anchor == anchors_.end() || node.dis <= 0.0 ||
          !seen.insert(node.id).second) continue;
      uwb_imu_pl::UwbMeasurement measurement;
      measurement.id = uwb_imu_pl::MeasurementId(
          event.sequence * 1000 + batch.measurements.size());
      measurement.factor_id = uwb_imu_pl::FactorId(measurement.id.value());
      measurement.anchor_id = uwb_imu_pl::AnchorId(node.id);
      measurement.timestamp = event.timestamp;
      measurement.range_m = node.dis;
      measurement.anchor_position_m = anchor->second.position_world_m;
      const double anchor_sigma = std::sqrt(
          anchor->second.covariance_m2.diagonal().maxCoeff());
      measurement.sigma_m = std::hypot(config_.realtime.range_sigma_m,
                                       anchor_sigma);
      measurement.sequence = event.sequence;
      batch.measurements.push_back(measurement);
    }
    if (batch.measurements.empty()) {
      throw std::runtime_error("LinkTrack frame has no valid configured anchor ranges");
    }
    return batch;
  }

  void processImu(const Event& event, const ProcessingMetrics& metrics) {
    if (!estimator_) {
      initialize(event.timestamp);
    } else if (last_processed_imu_ &&
               event.timestamp.seconds() - last_processed_imu_->seconds() >
                   config_.imu.max_gap_s) {
      const auto previous = estimator_->currentState();
      uwb_imu_pl::IntegrityOutput unavailable;
      unavailable.timestamp = event.timestamp;
      unavailable.state = previous;
      unavailable.state.timestamp = event.timestamp;
      unavailable.protection_level.timestamp = event.timestamp;
      unavailable.protection_level.availability =
          uwb_imu_pl::Availability::Unavailable;
      unavailable.protection_level.reason = "IMU gap exceeds configured maximum";
      publish(unavailable, metrics);
      logger_->writeEvent(event.timestamp, "IMU_GAP_REINITIALIZE",
                          "graph left unchanged before controlled reinitialization");
      initialize(event.timestamp, previous);
    }
    uwb_imu_pl::ImuMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(event.sequence);
    measurement.timestamp = event.timestamp;
    measurement.specific_force_mps2 = {
        event.imu.linear_acceleration.x, event.imu.linear_acceleration.y,
        event.imu.linear_acceleration.z};
    measurement.angular_velocity_radps = {
        event.imu.angular_velocity.x, event.imu.angular_velocity.y,
        event.imu.angular_velocity.z};
    pipeline_->ingestImu(measurement);
    last_processed_imu_ = event.timestamp;
    last_imu_measurement_ = measurement;
  }

  void processUwb(const Event& event, const ProcessingMetrics& metrics) {
    if (!pipeline_) {
      logger_->writeEvent(event.timestamp, "UWB_BEFORE_INITIALIZATION_REJECT",
                          "waiting for first ordered IMU sample");
      ++summary_.rejected;
      return;
    }
    const auto recovery_state = estimator_->currentState();
    const std::uint64_t marginalizations_before =
        estimator_->marginalizationCount();
    try {
    const auto end_to_end_start = std::chrono::steady_clock::now();
    const auto parse_start = std::chrono::steady_clock::now();
    const auto batch = convert(event);
    const double parse_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - parse_start).count();
    const auto core_start = std::chrono::steady_clock::now();
    const auto output = pipeline_->processUwbBatch(batch);
    const double core_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - core_start).count();
    const auto publish_start = std::chrono::steady_clock::now();
    publish(output, metrics);
    const double publish_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - publish_start).count();
    const auto logging_start = std::chrono::steady_clock::now();
    logger_->writeState(output.state);
    logger_->writeIntegrity(output);
    for (const auto& residual : output.residual_records) {
      logger_->writeResidual(residual.timestamp, residual.factor_id,
                             residual.anchor_id, residual.role, residual.raw,
                             residual.whitened);
    }
    const double logging_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - logging_start).count();
    const std::size_t epoch = estimator_->currentEpoch();
    const std::size_t factor_count = estimator_->factorCount();
    auto timing = [&](const std::string& stage, double wall_ms, bool success) {
      uwb_imu_pl::TimingRecord record;
      record.timestamp = output.timestamp;
      record.epoch = epoch;
      record.stage = stage;
      record.wall_ms = wall_ms;
      record.problem_size = batch.measurements.size();
      record.hypothesis_count = output.sensitivities.size();
      record.factor_count = factor_count;
      record.cold = epoch <= 100;
      record.success = success;
      logger_->writeTiming(record);
    };
    timing("parse", parse_ms, true);
    timing("imu_preintegration", estimator_->lastImuPreintegrationMs(), true);
    timing("no_uwb_isam_update", estimator_->lastNoUwbUpdateMs(), true);
    timing("state_query", estimator_->lastStateQueryMs(), true);
    timing("current_joint_marginal", estimator_->lastMarginalMs(), true);
    timing("snapshot_extraction", estimator_->lastSnapshotExtractionMs(), true);
    for (const auto& stage : output.stage_timings) {
      timing(stage.stage, stage.wall_ms, stage.success);
    }
    timing(output.batch_committed ? "uwb_commit" : "uwb_reject",
           estimator_->lastUwbUpdateMs(), true);
    timing("core_total", core_ms, true);
    timing("publish", publish_ms, true);
    timing("logging", logging_ms, true);
    const double end_to_end_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - end_to_end_start).count();
    timing("end_to_end_total", end_to_end_ms, true);
    summary_.processed++;
    summary_.committed += output.batch_committed ? 1 : 0;
    summary_.rejected += output.batch_committed ? 0 : 1;
    summary_.core_total_ms += core_ms;
    summary_.end_to_end_total_ms += end_to_end_ms;
    logger_->writeEvent(output.timestamp,
                        output.batch_committed ? "UWB_COMMIT" : "UWB_REJECT",
                        output.detector.reason);
    if (estimator_->marginalizationCount() > marginalizations_before) {
      logger_->writeEvent(
          output.timestamp, "FIXED_LAG_MARGINALIZE",
          "oldest_retained_epoch=" +
              std::to_string(estimator_->oldestRetainedEpoch()) +
              ";retained_epochs=" +
              std::to_string(estimator_->retainedEpochs()) +
              ";active_values=" +
              std::to_string(estimator_->activeValueCount()) +
              ";active_factors=" +
              std::to_string(estimator_->factorCount()));
    }
    } catch (const std::exception& error) {
      // iSAM2 can have accepted a new key before a subsequent marginal query
      // fails. Retrying that partially advanced instance would silently
      // reassociate the next UWB with an existing key. Fail closed, publish an
      // explicit unavailable epoch, and start a fresh audited graph instead.
      uwb_imu_pl::IntegrityOutput unavailable;
      unavailable.timestamp = event.timestamp;
      unavailable.state = recovery_state;
      unavailable.state.timestamp = event.timestamp;
      unavailable.measurement_group_size = event.uwb.nodes.size();
      unavailable.protection_level.timestamp = event.timestamp;
      unavailable.protection_level.availability =
          uwb_imu_pl::Availability::Unavailable;
      unavailable.protection_level.reason =
          std::string("controlled numerical reinitialization: ") + error.what();
      publish(unavailable, metrics);
      logger_->writeState(unavailable.state);
      logger_->writeIntegrity(unavailable);
      logger_->writeEvent(event.timestamp,
                          "NUMERICAL_REINITIALIZE", error.what());
      if (estimator_->marginalizationCount() > marginalizations_before) {
        logger_->writeEvent(
            event.timestamp, "FIXED_LAG_MARGINALIZE",
            "marginalization completed before downstream numerical failure");
      }
      ++summary_.processed;
      ++summary_.rejected;
      initialize(event.timestamp, recovery_state);
      if (last_imu_measurement_) {
        auto boundary = *last_imu_measurement_;
        boundary.timestamp = event.timestamp;
        pipeline_->ingestImu(boundary);
        last_processed_imu_ = event.timestamp;
        last_imu_measurement_ = boundary;
      }
    }
  }

  void workerLoop() {
    while (!stop_.load() && ros::ok()) {
      Event event;
      ProcessingMetrics metrics;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return stop_.load() || !queue_.empty(); });
        if (stop_.load()) break;
        auto first = queue_.begin();
        if (first->kind == Event::Kind::Uwb && latest_received_imu_ < first->timestamp) {
          condition_.wait_for(lock, std::chrono::milliseconds(5));
          continue;
        }
        event = *first;
        queue_.erase(first);
        metrics.queue_depth = static_cast<std::uint32_t>(std::min<std::size_t>(
            queue_.size(), std::numeric_limits<std::uint32_t>::max()));
      }
      metrics.sensor_to_process_ms = sensorLagMs(event.timestamp);
      const auto key = event.orderKey();
      if (last_processed_event_ &&
          uwb_imu_pl::eventOrderLess(key, *last_processed_event_)) {
        const char* name = event.kind == Event::Kind::Uwb
            ? "STALE_UWB_REJECT" : "STALE_IMU_REJECT";
        logger_->writeEvent(
            last_processed_event_->timestamp, name,
            "original_timestamp_ns=" + std::to_string(event.timestamp.value()) +
                "; event order key precedes processed watermark");
        ++summary_.rejected;
        continue;
      }
      try {
        if (event.kind == Event::Kind::Imu) processImu(event, metrics);
        else processUwb(event, metrics);
        last_processed_event_ = key;
      } catch (const std::exception& error) {
        ROS_ERROR_STREAM("realtime integrity event rejected: " << error.what());
        logger_->writeEvent(event.timestamp,
                            "PROCESSING_ERROR", error.what());
        ++summary_.errors;
      }
    }
  }

  void publish(const uwb_imu_pl::IntegrityOutput& output,
               const ProcessingMetrics& metrics) {
    nav_msgs::Odometry odometry;
    odometry.header.stamp = rosTime(output.timestamp);
    odometry.header.frame_id = config_.realtime.world_frame;
    odometry.child_frame_id = config_.realtime.body_frame;
    odometry.pose.pose.position.x = output.state.position_world_m.x();
    odometry.pose.pose.position.y = output.state.position_world_m.y();
    odometry.pose.pose.position.z = output.state.position_world_m.z();
    odometry.pose.pose.orientation.w = output.state.q_world_body.w();
    odometry.pose.pose.orientation.x = output.state.q_world_body.x();
    odometry.pose.pose.orientation.y = output.state.q_world_body.y();
    odometry.pose.pose.orientation.z = output.state.q_world_body.z();
    odometry.twist.twist.linear.x = output.state.velocity_world_mps.x();
    odometry.twist.twist.linear.y = output.state.velocity_world_mps.y();
    odometry.twist.twist.linear.z = output.state.velocity_world_mps.z();
    odometry_publisher_.publish(odometry);

    uwb_imu_pl::IntegrityStatus status;
    status.header = odometry.header;
    status.scope_label = uwb_imu_pl::toString(output.protection_level.label);
    status.availability = uwb_imu_pl::toString(output.protection_level.availability);
    status.detector_passed = output.detector.passed;
    status.batch_committed = output.batch_committed;
    status.statistic = output.detector.statistic;
    status.threshold = output.detector.threshold;
    status.dof = output.detector.dof;
    status.group_size = output.measurement_group_size;
    status.measurement_model_valid = output.measurement_model_valid;
    status.fixed_lag_active = estimator_ && estimator_->fixedLagActive();
    status.retained_epochs = estimator_ ? estimator_->retainedEpochs() : 0;
    status.marginalization_count =
        estimator_ ? estimator_->marginalizationCount() : 0;
    status.historical_fault_provenance = false;
    status.queue_depth = metrics.queue_depth;
    status.sensor_to_process_ms = metrics.sensor_to_process_ms;
    status.sensor_to_publish_ms = sensorLagMs(output.timestamp);
    for (int axis = 0; axis < 3; ++axis) {
      status.pl_xyz_m[axis] = output.protection_level.pl_xyz_m(axis);
      status.maximizing_anchor_id[axis] =
          output.protection_level.maximizing_anchor[axis].value();
    }
    status.hpl_m = output.protection_level.hpl_m;
    status.vpl_m = output.protection_level.vpl_m;
    status.formal_eligible = output.protection_level.formal_eligible;
    status.risk_budget_valid = output.protection_level.risk_budget_valid;
    status.allocated_hmi_risk = output.protection_level.allocated_hmi_risk;
    status.hmi_risk_requirement = output.protection_level.hmi_risk_requirement;
    status.config_hash = config_.config_hash;
    status.reason = output.protection_level.reason;
    integrity_publisher_.publish(status);

    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.header = status.header;
    diagnostic_msgs::DiagnosticStatus item;
    item.name = "uwb_imu_pl/integrity";
    item.hardware_id = "uwb_imu_pl";
    item.level = output.protection_level.availability ==
                         uwb_imu_pl::Availability::Available
                     ? diagnostic_msgs::DiagnosticStatus::OK
                     : (output.protection_level.availability ==
                                uwb_imu_pl::Availability::Alert
                            ? diagnostic_msgs::DiagnosticStatus::ERROR
                            : diagnostic_msgs::DiagnosticStatus::WARN);
    item.message = status.availability + ": " + status.reason;
    diagnostic_msgs::KeyValue scope;
    scope.key = "scope";
    scope.value = status.scope_label;
    item.values.push_back(scope);
    diagnostic_msgs::KeyValue fixed_lag;
    fixed_lag.key = "fixed_lag_active";
    fixed_lag.value = status.fixed_lag_active ? "true" : "false";
    item.values.push_back(fixed_lag);
    diagnostic_msgs::KeyValue retained_epochs;
    retained_epochs.key = "retained_epochs";
    retained_epochs.value = std::to_string(status.retained_epochs);
    item.values.push_back(retained_epochs);
    diagnostic_msgs::KeyValue marginalization_count;
    marginalization_count.key = "marginalization_count";
    marginalization_count.value = std::to_string(status.marginalization_count);
    item.values.push_back(marginalization_count);
    diagnostic_msgs::KeyValue historical_provenance;
    historical_provenance.key = "historical_fault_provenance";
    historical_provenance.value =
        status.historical_fault_provenance ? "true" : "false";
    item.values.push_back(historical_provenance);
    diagnostic_msgs::KeyValue active_values;
    active_values.key = "active_value_count";
    active_values.value =
        estimator_ ? std::to_string(estimator_->activeValueCount()) : "0";
    item.values.push_back(active_values);
    diagnostic_msgs::KeyValue active_factors;
    active_factors.key = "active_factor_count";
    active_factors.value =
        estimator_ ? std::to_string(estimator_->factorCount()) : "0";
    item.values.push_back(active_factors);
    diagnostic_msgs::KeyValue queue_depth;
    queue_depth.key = "queue_depth";
    queue_depth.value = std::to_string(status.queue_depth);
    item.values.push_back(queue_depth);
    diagnostic_msgs::KeyValue process_lag;
    process_lag.key = "sensor_to_process_ms";
    process_lag.value = std::to_string(status.sensor_to_process_ms);
    item.values.push_back(process_lag);
    diagnostic_msgs::KeyValue publish_lag;
    publish_lag.key = "sensor_to_publish_ms";
    publish_lag.value = std::to_string(status.sensor_to_publish_ms);
    item.values.push_back(publish_lag);
    diagnostics.status.push_back(item);
    diagnostics_publisher_.publish(diagnostics);
  }

  ros::NodeHandle node_;
  uwb_imu_pl::IntegrityConfig config_;
  std::map<std::uint64_t, uwb_imu_pl::AnchorRecord> anchors_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber uwb_subscriber_;
  ros::Subscriber ground_truth_subscriber_;
  ros::Subscriber fault_truth_subscriber_;
  ros::Publisher odometry_publisher_;
  ros::Publisher integrity_publisher_;
  ros::Publisher diagnostics_publisher_;
  std::unique_ptr<uwb_imu_pl::IncrementalUwbImuEstimator> estimator_;
  std::unique_ptr<uwb_imu_pl::IntegrityMonitor> monitor_;
  std::unique_ptr<uwb_imu_pl::RealtimeIntegrityPipeline> pipeline_;
  std::unique_ptr<uwb_imu_pl::RunLogger> logger_;
  std::multiset<Event, EventLess> queue_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::atomic<std::uint64_t> sequence_{1};
  uwb_imu_pl::TimestampNs latest_received_imu_;
  std::optional<uwb_imu_pl::TimestampNs> last_processed_imu_;
  std::optional<uwb_imu_pl::ImuMeasurement> last_imu_measurement_;
  std::optional<uwb_imu_pl::EventOrderKey> last_processed_event_;
  uwb_imu_pl::RunSummary summary_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "uwb_imu_pl_realtime");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  std::string config_path;
  std::string run_directory;
  std::string fixed_lag_epochs_override;
  std::string seed_override;
  std::string global_diagnostics_override;
  std::string residual_logging_override;
  std::string timing_logging_override;
  std::string output_root_override;
  std::string execution_command;
  private_node.param("config_path", config_path, std::string());
  private_node.param("run_directory", run_directory, std::string());
  private_node.param("fixed_lag_epochs", fixed_lag_epochs_override,
                     std::string());
  private_node.param("seed", seed_override, std::string());
  private_node.param("write_global_diagnostics", global_diagnostics_override,
                     std::string());
  private_node.param("write_residuals", residual_logging_override,
                     std::string());
  private_node.param("write_timing", timing_logging_override, std::string());
  private_node.param("output_root", output_root_override, std::string());
  private_node.param("execution_command", execution_command, std::string());
  if (config_path.empty()) {
    ROS_FATAL("~config_path is required");
    return 2;
  }
  try {
    uwb_imu_pl::IntegrityConfigOverrides overrides;
    overrides.seed = parseOptionalUnsigned(seed_override, "seed override");
    if (const auto lag = parseOptionalUnsigned(
            fixed_lag_epochs_override, "fixed_lag_epochs override")) {
      if (*lag > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error("fixed_lag_epochs override must fit in uint32");
      }
      overrides.fixed_lag_epochs = static_cast<std::uint32_t>(*lag);
    }
    overrides.write_global_diagnostics = parseOptionalBool(
        global_diagnostics_override, "write_global_diagnostics override");
    overrides.write_residuals = parseOptionalBool(
        residual_logging_override, "write_residuals override");
    overrides.write_timing = parseOptionalBool(
        timing_logging_override, "write_timing override");
    if (!output_root_override.empty()) overrides.output_root = output_root_override;
    auto config = uwb_imu_pl::IntegrityConfigLoader::load(config_path, overrides);
    if (execution_command.empty()) {
      execution_command = processCommandLine(argc, argv);
    }
    ROS_INFO_STREAM("resolved fixed_lag_epochs="
                    << config.incremental.fixed_lag_epochs
                    << " config_hash=" << config.config_hash);
    RealtimeNode realtime(node, std::move(config), run_directory,
                          execution_command);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}
