#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/common/deterministic_event.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/estimation_tuning.hpp"
#include "uwb_imu_pl/estimation/causal_initializer.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"
#include "uwb_imu_pl/publication/final_output_packet.hpp"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tbb/global_control.h>
#include <ros/serialization.h>
#include <sensor_msgs/Imu.h>
#include <uwb_imu_pl/IntegrityStatus.h>
#include <uwb_imu_pl/NavigationIntegrity.h>
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
  std::uint64_t arrival_steady_ns = 0;
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
  std::uint64_t arrival_steady_ns = 0;
  std::uint64_t compute_done_steady_ns = 0;
};

std::uint64_t steadyNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

class RosPublicationTransaction final
    : public uwb_imu_pl::FinalPacketPublicationTransaction {
 public:
  using PublishCandidate =
      std::function<void(const uwb_imu_pl::FinalOutputPacket&)>;
  explicit RosPublicationTransaction(PublishCandidate publish_candidate)
      : publish_candidate_(std::move(publish_candidate)) {}

  void publishCandidate(
      const uwb_imu_pl::FinalOutputPacket& candidate) override {
    publish_candidate_(candidate);
  }
  void commitReceipt(
      const uwb_imu_pl::FinalOutputPacket&) noexcept override {}

  void abort() noexcept override { aborted_ = true; }
  bool aborted() const { return aborted_; }

 private:
  PublishCandidate publish_candidate_;
  bool aborted_ = false;
};

const char* booleanText(bool value) { return value ? "true" : "false"; }

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
               bool enable_run_logging, std::string run_directory,
               std::string execution_command, bool subscribe_truth_topics,
               double worker_start_delay_s)
      : node_(node), config_(std::move(config)),
        worker_start_delay_s_(worker_start_delay_s) {
    tuning_ = uwb_imu_pl::readEstimationTuningV1(config_);
    for (const auto& anchor : config_.anchors) {
      anchors_.emplace(anchor.id.value(), anchor);
    }
    if (enable_run_logging && run_directory.empty()) {
      run_directory = config_.output.root + "/online_" +
          std::to_string(ros::WallTime::now().toNSec());
    }
    if (enable_run_logging) {
      logger_.enable(run_directory, config_.output.write_residuals,
                     config_.output.write_timing);
    }
    run_id_ = logger_.enabled() ? logger_.directory() : "logging_disabled";
    execution_command = uwb_imu_pl::bindExecutionCommandArguments(
        execution_command,
        {{"config_path", config_.source_path},
         {"fde_profile", uwb_imu_pl::toString(config_.resolved_scope.profile)},
         {"fixed_lag_epochs",
          std::to_string(config_.incremental.fixed_lag_epochs)},
         {"enable_run_logging", booleanText(logger_.enabled())},
         {"run_directory", logger_.enabled() ? logger_.directory() : "DISABLED"},
         {"seed", std::to_string(config_.seed)},
         {"write_global_diagnostics",
          booleanText(config_.output.write_global_diagnostics)},
         {"write_residuals", booleanText(config_.output.write_residuals)},
         {"write_timing", booleanText(config_.output.write_timing)},
         {"output_root", config_.output.root}});
    logger_.writeResolvedConfig(config_.resolved_yaml);
    logger_.writeManifest(uwb_imu_pl::makeRunManifest(
        config_, UWB_IMU_PL_GIT_SHA, UWB_IMU_PL_GIT_DIRTY != 0,
        execution_command));
    ROS_INFO_STREAM("run logging "
                    << (logger_.enabled() ? "enabled: " + logger_.directory()
                                          : "disabled"));

    odometry_publisher_ = node_.advertise<nav_msgs::Odometry>(
        config_.realtime.odometry_topic, 10);
    integrity_publisher_ = node_.advertise<uwb_imu_pl::IntegrityStatus>(
        config_.realtime.integrity_topic, 10);
    diagnostics_publisher_ = node_.advertise<diagnostic_msgs::DiagnosticArray>(
        config_.realtime.diagnostics_topic, 10);
    solution_publisher_ = node_.advertise<uwb_imu_pl::NavigationIntegrity>(
        "/uwb_imu_pl/solution", 10);
    imu_subscriber_ = node_.subscribe(
        config_.realtime.imu_topic, 1000, &RealtimeNode::imuCallback, this);
    uwb_subscriber_ = node_.subscribe(
        config_.realtime.uwb_topic, 100, &RealtimeNode::uwbCallback, this);
    if (subscribe_truth_topics) {
      ground_truth_subscriber_ = node_.subscribe(
          "/sim/odom", 100, &RealtimeNode::groundTruthCallback, this);
      fault_truth_subscriber_ = node_.subscribe(
          "/uwb_sim/fault_truth", 1000, &RealtimeNode::faultTruthCallback, this);
    }
    worker_ = std::thread(&RealtimeNode::workerLoop, this);
  }

  ~RealtimeNode() {
    stop_.store(true);
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (logger_.enabled()) {
      summary_.status = summary_.errors == 0 ? "IMPLEMENTED_UNVERIFIED" : "FAILED";
      summary_.detail = "ROS1 realtime node stopped; research only, not certified";
      logger_.writeSummary(summary_);
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
    logger_.writeGroundTruth(record);
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
    record.sensor_type = message->sensor_type;
    record.fault_kind = message->fault_kind;
    record.axis = message->axis;
    record.epoch_begin = message->epoch_begin;
    record.epoch_end = message->epoch_end;
    record.injected_value = message->injected_value;
    record.injected_units = message->injected_units;
    logger_.writeFaultTruth(record);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    Event event;
    event.kind = Event::Kind::Imu;
    event.timestamp = timestamp(message->header.stamp);
    event.sequence = sequence_.fetch_add(1);
    event.arrival_steady_ns = steadyNowNs();
    event.imu = *message;
    bool overflow = false;
    std::size_t discarded = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= config_.realtime.max_queued_events) {
        overflow = true;
        discarded = queue_.size();
        queue_.clear();
        imu_overflow_pending_ = true;
        ++queue_overflow_count_;
      }
      queue_.insert(std::move(event));
      if (latest_received_imu_ < timestamp(message->header.stamp)) {
        latest_received_imu_ = timestamp(message->header.stamp);
      }
    }
    if (overflow) {
      logger_.writeEvent(timestamp(message->header.stamp),
                          "IMU_QUEUE_OVERFLOW_REINITIALIZE",
                          "discarded_events=" + std::to_string(discarded) +
                              ";policy=" + config_.realtime.imu_overflow_policy);
    }
    condition_.notify_one();
  }

  void uwbCallback(const uwb_imu_pl::LinktrackNodeframe3::ConstPtr& message) {
    Event event;
    event.kind = Event::Kind::Uwb;
    event.timestamp = timestamp(message->header.stamp);
    event.sequence = sequence_.fetch_add(1);
    event.arrival_steady_ns = steadyNowNs();
    event.uwb = *message;
    std::optional<uwb_imu_pl::TimestampNs> dropped_timestamp;
    bool drop_incoming = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= config_.realtime.max_queued_events) {
        const auto oldest_uwb = std::find_if(
            queue_.begin(), queue_.end(), [](const Event& queued) {
              return queued.kind == Event::Kind::Uwb;
            });
        if (oldest_uwb == queue_.end()) {
          drop_incoming = true;
          dropped_timestamp = event.timestamp;
        } else {
          dropped_timestamp = oldest_uwb->timestamp;
          queue_.erase(oldest_uwb);
        }
        ++queue_overflow_count_;
        ++dropped_uwb_count_;
      }
      if (!drop_incoming) {
      queue_.insert(std::move(event));
      }
    }
    if (dropped_timestamp) {
      logger_.writeEvent(*dropped_timestamp, "UWB_QUEUE_OVERFLOW_DROP",
                          "policy=" + config_.realtime.uwb_overflow_policy +
                              ";incoming=" + (drop_incoming ? "true" : "false"));
    }
    if (drop_incoming) return;
    condition_.notify_one();
  }

  void initialize(uwb_imu_pl::TimestampNs time,
                  const std::optional<uwb_imu_pl::NavigationState>& seed_state =
                      std::nullopt,
                  const std::optional<Eigen::Matrix<double, 15, 1>>& prior = std::nullopt) {
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
    estimator_->initialize(initial, prior ? *prior : config_.realtime.prior_sigmas);
    monitor_.reset(new uwb_imu_pl::IntegrityMonitor(
        config_.risk, config_.snapshot.rank_tolerance,
        config_.snapshot.max_condition_number));
    pipeline_.reset(new uwb_imu_pl::RealtimeIntegrityPipeline(
        estimator_.get(), *monitor_));
    logger_.writeEvent(time, "INITIALIZED", "ordered sensor boundary;config_hash=" + config_.config_hash);
    ROS_INFO_STREAM("initial state t=" << time.seconds()
        << " p=" << initial.position_world_m.transpose()
        << " v=" << initial.velocity_world_mps.transpose()
        << " q_xyzw=" << initial.q_world_body.coeffs().transpose()
        << " ba=" << initial.accel_bias_mps2.transpose()
        << " bg=" << initial.gyro_bias_radps.transpose()
        << " priors=" << (prior ? *prior : config_.realtime.prior_sigmas).transpose()
        << " bias_integration_sigmas="
        << uwb_imu_pl::estimatorNumericsAuditV1(*estimator_).bias_integration_sigmas.transpose()
        << " config_hash=" << config_.config_hash);
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
    if (!estimator_ && tuning_.causal_bootstrap) {
      uwb_imu_pl::ImuMeasurement sample;
      sample.id = uwb_imu_pl::MeasurementId(event.sequence);
      sample.timestamp = event.timestamp;
      sample.specific_force_mps2 = {event.imu.linear_acceleration.x,
          event.imu.linear_acceleration.y, event.imu.linear_acceleration.z};
      sample.angular_velocity_radps = {event.imu.angular_velocity.x,
          event.imu.angular_velocity.y, event.imu.angular_velocity.z};
      bootstrap_imu_.push_back(sample);
      return;
    }
    if (!estimator_) {
      initialize(event.timestamp);
    } else if (last_processed_imu_ &&
               event.timestamp.seconds() - last_processed_imu_->seconds() >
                   config_.imu.max_gap_s) {
      const auto previous = estimator_->currentState();
      uwb_imu_pl::IntegrityOutput unavailable;
      unavailable.timestamp = previous.timestamp;
      unavailable.attempted_timestamp = event.timestamp;
      unavailable.state = previous;
      unavailable.fde_profile = uwb_imu_pl::toString(config_.resolved_scope.profile);
      unavailable.scope_digest = config_.resolved_scope.scope_digest;
      unavailable.detector_contract_id = config_.resolved_scope.detector_contract_id;
      unavailable.pl_status = config_.resolved_scope.enabled()
                                  ? uwb_imu_pl::ProtectionLevelStatus::Invalid
                                  : uwb_imu_pl::ProtectionLevelStatus::NotComputed;
      unavailable.reason_codes.push_back("IMU_GAP");
      unavailable.protection_level.timestamp = previous.timestamp;
      unavailable.protection_level.availability =
          uwb_imu_pl::Availability::Unavailable;
      unavailable.protection_level.reason = "IMU gap exceeds configured maximum";
      (void)publish(unavailable, metrics);
      logger_.writeEvent(event.timestamp, "IMU_GAP_REINITIALIZE",
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
    if (const auto seed = pipeline_->pendingReinitializationSeed()) {
      logger_.writeEvent(event.timestamp, "REINITIALIZATION_START",
          "request_id=" + std::to_string(
              pipeline_->reinitializationDirective().request_id.value()));
      const auto inflated = pipeline_->reinitializationPriorSigmas(
          config_.realtime.prior_sigmas);
      auto replacement = std::make_unique<uwb_imu_pl::IncrementalUwbImuEstimator>(
          config_, config_.realtime.lever_arm_body_m);
      replacement->initialize(*seed, inflated);
      pipeline_->completeReinitialization(replacement.get(), measurement);
      estimator_ = std::move(replacement);
      logger_.writeEvent(event.timestamp, "REINITIALIZATION_COMPLETE",
          "trusted strictly-increasing finite IMU boundary accepted");
    }
    last_processed_imu_ = event.timestamp;
    last_imu_measurement_ = measurement;
  }

  void processUwb(const Event& event, const ProcessingMetrics& metrics) {
    if (!pipeline_ && tuning_.causal_bootstrap) {
      const auto batch = convert(event);
      if (bootstrap_imu_.empty()) return;
      const auto end = uwb_imu_pl::TimestampNs::fromSeconds(
          bootstrap_imu_.front().timestamp.seconds() + 2.0);
      if (!(end < event.timestamp)) {
        bootstrap_ranges_.insert(bootstrap_ranges_.end(),
            batch.measurements.begin(), batch.measurements.end());
        return;
      }
      if (bootstrap_imu_.back().timestamp < end)
        throw std::runtime_error("BOOTSTRAP_WAITING_FOR_TWO_SECONDS");
      const auto initialized = uwb_imu_pl::initializeCausallyV1(
          bootstrap_imu_, bootstrap_ranges_, config_,
          config_.realtime.lever_arm_body_m,
          tuning_.bootstrap_prefer_below_anchors);
      initialize(initialized.state.timestamp, initialized.state,
                 initialized.prior_sigmas);
      pipeline_->ingestImu(initialized.boundary);
      for (const auto& sample : bootstrap_imu_) {
        if (!(end < sample.timestamp)) continue;
        pipeline_->ingestImu(sample);
        last_processed_imu_ = sample.timestamp;
        last_imu_measurement_ = sample;
      }
      if (!last_processed_imu_) {
        last_processed_imu_ = initialized.boundary.timestamp;
        last_imu_measurement_ = initialized.boundary;
      }
      logger_.writeEvent(end, "CAUSAL_BOOTSTRAP",
          std::string("stationary=") + booleanText(initialized.stationary) +
          ";config_hash=" + config_.config_hash);
      bootstrap_imu_.clear(); bootstrap_ranges_.clear();
    }
    if (!pipeline_) {
      logger_.writeEvent(event.timestamp, "UWB_BEFORE_INITIALIZATION_REJECT",
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
    uwb_imu_pl::AttemptProofLease proof_lease;
    const auto output = pipeline_->processUwbBatch(batch, &proof_lease);
    const double core_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - core_start).count();
    ProcessingMetrics final_metrics = metrics;
    final_metrics.compute_done_steady_ns = steadyNowNs();
    const auto publish_start = std::chrono::steady_clock::now();
    const auto final_packet = publish(
        output, final_metrics, uwb_imu_pl::FinalAttemptKind::Normal,
        &proof_lease);
    const auto& final_output = final_packet.output();
    const double publish_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - publish_start).count();
    const auto logging_start = std::chrono::steady_clock::now();
    for (const auto& residual : final_output.residual_records) {
      logger_.writeResidual(residual.timestamp, residual.factor_id,
                             residual.anchor_id, residual.role, residual.raw,
                             residual.whitened);
    }
    const double logging_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - logging_start).count();
    const std::size_t epoch = estimator_->currentEpoch();
    const std::size_t factor_count = estimator_->factorCount();
    auto timing = [&](const std::string& stage, double wall_ms, bool success) {
      uwb_imu_pl::TimingRecord record;
      record.timestamp = final_output.timestamp;
      record.epoch = epoch;
      record.stage = stage;
      record.wall_ms = wall_ms;
      record.problem_size = batch.measurements.size();
      record.hypothesis_count = final_output.sensitivities.size();
      record.factor_count = factor_count;
      record.cold = epoch <= 100;
      record.success = success;
      logger_.writeTiming(record);
    };
    timing("parse", parse_ms, true);
    timing("imu_preintegration", estimator_->lastImuPreintegrationMs(), true);
    timing("no_uwb_isam_update", estimator_->lastNoUwbUpdateMs(), true);
    timing("state_query", estimator_->lastStateQueryMs(), true);
    timing("current_joint_marginal", estimator_->lastMarginalMs(), true);
    timing("snapshot_extraction", estimator_->lastSnapshotExtractionMs(), true);
    for (const auto& stage : final_output.stage_timings) {
      timing(stage.stage, stage.wall_ms, stage.success);
    }
    timing(final_output.batch_committed ? "uwb_commit" : "uwb_reject",
           estimator_->lastUwbUpdateMs(), true);
    timing("core_total", core_ms, true);
    timing("publish", publish_ms, true);
    timing("logging", logging_ms, true);
    const double end_to_end_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - end_to_end_start).count();
    timing("end_to_end_total", end_to_end_ms, true);
    summary_.processed++;
    summary_.committed += final_output.batch_committed ? 1 : 0;
    summary_.rejected += final_output.batch_committed ? 0 : 1;
    summary_.core_total_ms += core_ms;
    summary_.end_to_end_total_ms += end_to_end_ms;
    logger_.writeEvent(final_output.timestamp,
                        final_output.batch_committed ? "UWB_COMMIT" : "UWB_REJECT",
                        final_output.detector.reason);
    if (estimator_->marginalizationCount() > marginalizations_before) {
      logger_.writeEvent(
          final_output.timestamp, "FIXED_LAG_MARGINALIZE",
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
      uwb_imu_pl::IntegrityOutput unavailable = pipeline_->lastAttemptOutput();
      unavailable.timestamp = recovery_state.timestamp;
      unavailable.attempted_timestamp = event.timestamp;
      unavailable.state = recovery_state;
      unavailable.fde_profile = uwb_imu_pl::toString(config_.resolved_scope.profile);
      unavailable.scope_digest = config_.resolved_scope.scope_digest;
      unavailable.detector_contract_id = config_.resolved_scope.detector_contract_id;
      unavailable.pl_status = config_.resolved_scope.enabled()
                                  ? uwb_imu_pl::ProtectionLevelStatus::Invalid
                                  : uwb_imu_pl::ProtectionLevelStatus::NotComputed;
      if (std::find(unavailable.reason_codes.begin(),
                    unavailable.reason_codes.end(),
                    "NUMERICAL_REINITIALIZE") ==
          unavailable.reason_codes.end()) {
        unavailable.reason_codes.push_back("NUMERICAL_REINITIALIZE");
      }
      unavailable.measurement_group_size = event.uwb.nodes.size();
      unavailable.protection_level.timestamp = recovery_state.timestamp;
      unavailable.protection_level.availability =
          uwb_imu_pl::Availability::Unavailable;
      unavailable.protection_level.reason =
          std::string("controlled numerical reinitialization: ") + error.what();
      ProcessingMetrics final_metrics = metrics;
      final_metrics.compute_done_steady_ns = steadyNowNs();
      (void)publish(
          unavailable, final_metrics, uwb_imu_pl::FinalAttemptKind::Exception);
      logger_.writeEvent(event.timestamp,
                          "NUMERICAL_REINITIALIZE", error.what());
      if (estimator_->marginalizationCount() > marginalizations_before) {
        logger_.writeEvent(
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
    // Test/replay-only deterministic ingress barrier.  Production keeps the
    // default zero; an offline equivalence replay may first enqueue both ROS
    // topics so their shared timestamp ordering is independent of TCP arrival.
    if (worker_start_delay_s_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(
          worker_start_delay_s_));
    }
    while (!stop_.load() && ros::ok()) {
      Event event;
      ProcessingMetrics metrics;
      bool reinitialize_after_overflow = false;
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
        if (event.kind == Event::Kind::Imu && imu_overflow_pending_) {
          reinitialize_after_overflow = true;
          imu_overflow_pending_ = false;
        }
        metrics.queue_depth = static_cast<std::uint32_t>(std::min<std::size_t>(
            queue_.size(), std::numeric_limits<std::uint32_t>::max()));
      }
      metrics.arrival_steady_ns = event.arrival_steady_ns;
      metrics.sensor_to_process_ms = sensorLagMs(event.timestamp);
      const auto key = event.orderKey();
      if (last_processed_event_ &&
          uwb_imu_pl::eventOrderLess(key, *last_processed_event_)) {
        const char* name = event.kind == Event::Kind::Uwb
            ? "STALE_UWB_REJECT" : "STALE_IMU_REJECT";
        logger_.writeEvent(
            last_processed_event_->timestamp, name,
            "original_timestamp_ns=" + std::to_string(event.timestamp.value()) +
                "; event order key precedes processed watermark");
        ++summary_.rejected;
        continue;
      }
      try {
        if (reinitialize_after_overflow && estimator_) {
          const auto previous = estimator_->currentState();
          uwb_imu_pl::IntegrityOutput unavailable;
          unavailable.timestamp = previous.timestamp;
          unavailable.attempted_timestamp = event.timestamp;
          unavailable.state = previous;
          unavailable.fde_profile =
              uwb_imu_pl::toString(config_.resolved_scope.profile);
          unavailable.scope_digest = config_.resolved_scope.scope_digest;
          unavailable.detector_contract_id =
              config_.resolved_scope.detector_contract_id;
          unavailable.pl_status = config_.resolved_scope.enabled()
                                      ? uwb_imu_pl::ProtectionLevelStatus::Invalid
                                      : uwb_imu_pl::ProtectionLevelStatus::NotComputed;
          unavailable.reason_codes.push_back("IMU_QUEUE_OVERFLOW");
          unavailable.controlled_reinitialization_required = true;
          unavailable.reinitialization_reason =
              "IMU queue overflow invalidated preintegration continuity";
          unavailable.protection_level.timestamp = previous.timestamp;
          unavailable.protection_level.availability =
              uwb_imu_pl::Availability::Unavailable;
          unavailable.protection_level.reason =
              unavailable.reinitialization_reason;
          ProcessingMetrics final_metrics = metrics;
          final_metrics.compute_done_steady_ns = steadyNowNs();
          (void)publish(
              unavailable, final_metrics,
              uwb_imu_pl::FinalAttemptKind::QueueOverflow);
          initialize(event.timestamp, previous);
        }
        if (event.kind == Event::Kind::Imu) processImu(event, metrics);
        else processUwb(event, metrics);
        last_processed_event_ = key;
      } catch (const std::exception& error) {
        ROS_ERROR_STREAM("realtime integrity event rejected: " << error.what());
        logger_.writeEvent(event.timestamp,
                            "PROCESSING_ERROR", error.what());
        ++summary_.errors;
      }
    }
  }

  uwb_imu_pl::FinalOutputPacket publish(
      uwb_imu_pl::IntegrityOutput output,
      const ProcessingMetrics& metrics,
      uwb_imu_pl::FinalAttemptKind attempt_kind =
          uwb_imu_pl::FinalAttemptKind::Normal,
      const uwb_imu_pl::AttemptProofLease* proof_lease = nullptr) {
    const std::uint64_t deadline_ns = static_cast<std::uint64_t>(
        config_.publication.deadline_ms * 1e6);
    nav_msgs::Odometry odometry;
    odometry.header.stamp = rosTime(output.state.timestamp);
    odometry.header.frame_id = config_.realtime.world_frame;
    odometry.child_frame_id = config_.realtime.body_frame;
    odometry.pose.pose.position.x = output.state.position_world_m.x();
    odometry.pose.pose.position.y = output.state.position_world_m.y();
    odometry.pose.pose.position.z = output.state.position_world_m.z();
    odometry.pose.pose.orientation.w = output.state.q_world_body.w();
    odometry.pose.pose.orientation.x = output.state.q_world_body.x();
    odometry.pose.pose.orientation.y = output.state.q_world_body.y();
    odometry.pose.pose.orientation.z = output.state.q_world_body.z();
    Eigen::Matrix3d velocity_covariance_world =
        Eigen::Matrix3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    uwb_imu_pl::CommittedStatePublicationV1 committed_state;
    const bool committed_state_found = proof_lease
        ? uwb_imu_pl::committedStatePublicationV1(
              output.transaction_id, output.state.timestamp,
              *proof_lease, &committed_state)
        : uwb_imu_pl::committedStatePublicationV1(
              output.transaction_id, output.state.timestamp,
              &committed_state);
    if (committed_state_found) {
      velocity_covariance_world = committed_state.covariance.block<3, 3>(6, 6);
    }
    const bool twist_covariance_valid = velocity_covariance_world.allFinite();
    const auto child_twist = uwb_imu_pl::childFrameLinearTwist(
        output.state.q_world_body, output.state.velocity_world_mps,
        twist_covariance_valid ? velocity_covariance_world
                               : Eigen::Matrix3d::Zero());
    odometry.twist.twist.linear.x = child_twist.velocity_body_mps.x();
    odometry.twist.twist.linear.y = child_twist.velocity_body_mps.y();
    odometry.twist.twist.linear.z = child_twist.velocity_body_mps.z();
    if (twist_covariance_valid) {
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          odometry.twist.covariance[row * 6 + column] =
              child_twist.covariance_body_m2ps2(row, column);
        }
      }
    } else {
      odometry.twist.covariance[0] = -1.0;
    }
    uwb_imu_pl::IntegrityStatus status;
    status.header = odometry.header;
    status.attempted_timestamp = rosTime(output.attempted_timestamp);
    status.state_timestamp = rosTime(output.state.timestamp);
    status.publish_timestamp = ros::Time::now();
    status.attempt_id = output.diagnostics.input_attempt_id;
    status.solution_id = output.linearization_version;
    status.fde_profile = output.fde_profile;
    status.scope_digest = output.scope_digest;
    status.detector_contract_id = output.detector_contract_id;
    status.pl_status = uwb_imu_pl::toString(output.pl_status);
    status.within_alert_limits = output.within_alert_limits;
    status.state_valid = output.state_valid;
    status.fresh = output.fresh;
    status.deadline_missed = output.deadline_missed;
    status.publication_protected = output.publication.protected_output;
      status.publication_certificate_id = output.publication.certificate_id;
      status.final_packet_protocol_version = 2;
      status.final_packet_digest = 0;
      status.final_packet_authoritative = false;
    status.reason_codes = output.reason_codes;
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
    status.historical_fault_provenance = output.history_provenance_valid;
    status.queue_depth = metrics.queue_depth;
    status.queue_overflow_count = queue_overflow_count_.load();
    status.dropped_uwb_count = dropped_uwb_count_.load();
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
    status.transaction_id = output.transaction_id;
    status.window_id = output.window_id;
    status.base_graph_version = output.base_graph_version;
    status.linearization_version = output.linearization_version;
    status.selected_action_id = output.selected_action_id;
    status.selected_action_type = output.selected_action_type;
    status.fde_status = output.fde_status;
    for (int axis = 0; axis < 3; ++axis) {
      status.bridge_component_m[axis] = output.bridge_component_m(axis);
    }
    status.history_provenance_valid = output.history_provenance_valid;
    status.backend_updates = output.backend_updates;
    status.stale_state = output.stale_state;
    status.controlled_reinitialization_required =
        output.controlled_reinitialization_required;
    status.historical_groups_removed = output.historical_groups_removed;
    status.historical_groups_added = output.historical_groups_added;
    status.recovery_epoch_begin = output.recovery_epoch_begin;
    status.recovery_epoch_end = output.recovery_epoch_end;
    status.reinitialization_request_id = output.reinitialization_request_id;
    status.reinitialization_phase = output.reinitialization_phase;
    status.reinitialization_reason = output.reinitialization_reason;

    uwb_imu_pl::NavigationIntegrity solution;
    solution.header = odometry.header;
    solution.attempted_timestamp = status.attempted_timestamp;
    solution.state_timestamp = status.state_timestamp;
    solution.publish_timestamp = status.publish_timestamp;
    solution.arrival_steady_ns = metrics.arrival_steady_ns;
    solution.finish_steady_ns = 0;
    solution.state_age_s = std::max(
        0.0, output.attempted_timestamp.seconds() -
                 output.state.timestamp.seconds());
    solution.deadline_missed = output.deadline_missed;
    solution.run_id = run_id_;
    solution.attempt_id = output.diagnostics.input_attempt_id;
    solution.transaction_id = output.transaction_id;
    solution.solution_id = output.linearization_version;
    solution.config_hash = config_.config_hash;
    solution.scope_digest = output.scope_digest;
    solution.manifest_digest = config_.fault_manifest
        ? config_.fault_manifest->digest : std::string();
    solution.odometry = odometry;
    solution.integrity = status;
    solution.velocity_world_mps.x = output.state.velocity_world_mps.x();
    solution.velocity_world_mps.y = output.state.velocity_world_mps.y();
    solution.velocity_world_mps.z = output.state.velocity_world_mps.z();
    solution.accel_bias_mps2.x = output.state.accel_bias_mps2.x();
    solution.accel_bias_mps2.y = output.state.accel_bias_mps2.y();
    solution.accel_bias_mps2.z = output.state.accel_bias_mps2.z();
    solution.gyro_bias_radps.x = output.state.gyro_bias_radps.x();
    solution.gyro_bias_radps.y = output.state.gyro_bias_radps.y();
    solution.gyro_bias_radps.z = output.state.gyro_bias_radps.z();
    solution.state_valid = output.state_valid;
    solution.fresh = output.fresh;
    solution.batch_committed = output.batch_committed;
    solution.commit_reason = output.detector.reason.empty()
        ? output.protection_level.reason : output.detector.reason;
    solution.fde_profile = output.fde_profile;
    solution.fault_order = config_.resolved_scope.max_fault_order;
    for (const auto family : config_.resolved_scope.singles) {
      solution.active_families.push_back(uwb_imu_pl::toString(family));
    }
    for (const auto family : config_.resolved_scope.pairs) {
      solution.active_families.push_back(uwb_imu_pl::toString(family));
    }
    solution.detector_contract_id = output.detector_contract_id;
    solution.hypothesis_count = output.diagnostics.hypothesis_count;
    solution.generated_action_count = output.diagnostics.generated_actions;
    solution.selected_action_id = output.selected_action_id;
    for (int axis = 0; axis < 3; ++axis) {
      solution.pl_xyz_m[axis] = output.protection_level.pl_xyz_m(axis);
    }
    solution.hpl_m = output.protection_level.hpl_m;
    solution.vpl_m = output.protection_level.vpl_m;
    solution.pl_status = status.pl_status;
    solution.within_alert_limits = output.within_alert_limits;
    solution.risk_budget_valid = output.protection_level.risk_budget_valid;
    solution.formal_eligible = output.protection_level.formal_eligible;
    solution.publication_protected = output.publication.protected_output;
    solution.publication_certificate_id = output.publication.certificate_id;
    solution.final_packet_protocol_version = 2;
    solution.final_packet_digest = 0;
    solution.final_packet_authoritative = false;
    solution.reason_codes = output.reason_codes;

    // Message construction above is part of the measured deadline interval.
    // Freeze exactly once at the publish-call boundary, then project only this
    // immutable packet into the authoritative and mirror sinks.
    uwb_imu_pl::FinalPacketTiming packet_timing;
    packet_timing.arrival_steady_ns = metrics.arrival_steady_ns;
    packet_timing.compute_done_steady_ns = metrics.compute_done_steady_ns != 0
        ? metrics.compute_done_steady_ns : steadyNowNs();
    packet_timing.packet_ready_steady_ns = steadyNowNs();
    packet_timing.deadline_boundary =
        uwb_imu_pl::FinalPacketBoundary::PublishCall;
    packet_timing.attempt_kind = attempt_kind;
    std::string publication_error;
    RosPublicationTransaction publication_transaction(
        [&](const uwb_imu_pl::FinalOutputPacket& candidate_packet) {
          // Force all potentially allocating ROS serialization paths before
          // the terminal packet is frozen.  No topic is externally visible
          // during this phase; a throw therefore aborts every sink.
          (void)ros::serialization::serializationLength(solution);
          (void)ros::serialization::serializationLength(solution.odometry);
          (void)ros::serialization::serializationLength(solution.integrity);
          const auto& candidate_output = candidate_packet.output();
          status.deadline_missed = candidate_output.deadline_missed;
          status.publication_protected =
              candidate_output.publication.protected_output;
          status.reason_codes = candidate_output.reason_codes;
          const auto& metadata = candidate_packet.metadata();
          status.final_packet_protocol_version = metadata.protocol_version;
          status.final_packet_digest = metadata.digest;
          status.final_packet_authoritative = metadata.authoritative;
          status.protected_frame_id = metadata.protected_frame_id;
          status.protected_position_reference =
              metadata.protected_position_reference;
          status.protection_packet_id = metadata.protection_packet_id;
          status.fresh = candidate_output.fresh;
          status.stale_state = candidate_output.stale_state;
          status.formal_eligible =
              candidate_output.protection_level.formal_eligible;
          status.reason = candidate_output.protection_level.reason;
          solution.finish_steady_ns = 0;
          solution.state_age_s = candidate_packet.stateAgeSeconds();
          solution.deadline_missed = candidate_output.deadline_missed;
          solution.integrity = status;
          solution.fresh = candidate_output.fresh;
          solution.formal_eligible =
              candidate_output.protection_level.formal_eligible;
          solution.publication_protected =
              candidate_output.publication.protected_output;
          solution.reason_codes = candidate_output.reason_codes;
          solution.final_packet_protocol_version =
              status.final_packet_protocol_version;
          solution.final_packet_digest = status.final_packet_digest;
          solution.final_packet_authoritative = false;
          solution.protected_frame_id = status.protected_frame_id;
          solution.protected_position_reference =
              status.protected_position_reference;
          solution.protection_packet_id = status.protection_packet_id;

          // ROS topics and CSV are explicitly non-authoritative mirrors.  All
          // their real, fallible calls occur before the terminal return sample.
          (void)ros::serialization::serializationLength(solution);
          (void)ros::serialization::serializationLength(solution.odometry);
          (void)ros::serialization::serializationLength(solution.integrity);
          solution_publisher_.publish(solution);
          solution.odometry.header.seq =
              static_cast<std::uint32_t>(status.final_packet_digest);
          solution.integrity.publication_protected = false;
          solution.integrity.final_packet_authoritative = false;
          odometry_publisher_.publish(solution.odometry);
          integrity_publisher_.publish(solution.integrity);
          logger_.writeState(candidate_output.state);
          logger_.writeIntegrity(candidate_output, metadata);
          logger_.flush();
        });
    uwb_imu_pl::FinalProtectionProofBundleV1 proof_bundle;
    std::string bundle_reason;
    const bool scoped_bundle_required = proof_lease &&
        !output.protection_level.detector_certificate_id.empty();
    const bool scoped_bundle = scoped_bundle_required &&
        uwb_imu_pl::freezeFinalProtectionProofBundleV1(
            *proof_lease, output.protection_level.detector_certificate_id,
            &proof_bundle, &bundle_reason);
    if (scoped_bundle_required && !scoped_bundle) {
      output.protection_level.formal_eligible = false;
      output.protection_level.risk_budget_valid = false;
      output.publication.protected_output = false;
      output.publication.unprotected_output = true;
      output.publication.refusal = "FINAL_PROOF_BUNDLE_FREEZE_FAILED";
      output.reason_codes.push_back("FINAL_PROOF_BUNDLE_FREEZE_FAILED");
      if (!bundle_reason.empty()) {
        ROS_ERROR_STREAM("scoped final proof bundle rejected: "
                         << bundle_reason);
      }
    }
    // A non-empty scoped packet never falls back to the process-global legacy
    // registry.  Passing the invalid bundle forces the terminal helper to
    // clear the packet identity and remain explicitly unprotected.
    const auto final_packet = scoped_bundle_required
        ? uwb_imu_pl::invokeFinalOutputPacket(
              std::move(output), packet_timing, deadline_ns, proof_bundle,
              [] { return steadyNowNs(); }, &publication_transaction,
              &publication_error)
        : uwb_imu_pl::invokeFinalOutputPacket(
              std::move(output), packet_timing, deadline_ns,
              [] { return steadyNowNs(); }, &publication_transaction,
              &publication_error);
    const auto& final_output = final_packet.output();
    status.deadline_missed = final_output.deadline_missed;
    status.publication_protected =
        final_output.publication.protected_output;
    status.reason_codes = final_output.reason_codes;
    status.fresh = final_output.fresh;
    status.stale_state = final_output.stale_state;
    status.formal_eligible = final_output.protection_level.formal_eligible;
    status.reason = final_output.protection_level.reason;
    solution.finish_steady_ns = final_packet.timing().publish_return_steady_ns;
    solution.state_age_s = final_packet.stateAgeSeconds();
    solution.deadline_missed = final_output.deadline_missed;
    solution.integrity = status;
    solution.fresh = final_output.fresh;
    solution.formal_eligible = final_output.protection_level.formal_eligible;
    solution.publication_protected =
        final_output.publication.protected_output;
    solution.reason_codes = final_output.reason_codes;

    if (!publication_error.empty()) {
      ROS_ERROR_STREAM("final packet publication failed: "
                       << publication_error);
    }

    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.header = status.header;
    diagnostic_msgs::DiagnosticStatus item;
    item.name = "uwb_imu_pl/integrity";
    item.hardware_id = "uwb_imu_pl";
    item.level = final_output.protection_level.availability ==
                         uwb_imu_pl::Availability::Available
                     ? diagnostic_msgs::DiagnosticStatus::OK
                     : (final_output.protection_level.availability ==
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
    return final_packet;
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
  ros::Publisher solution_publisher_;
  std::unique_ptr<uwb_imu_pl::IncrementalUwbImuEstimator> estimator_;
  std::unique_ptr<uwb_imu_pl::IntegrityMonitor> monitor_;
  std::unique_ptr<uwb_imu_pl::RealtimeIntegrityPipeline> pipeline_;
  uwb_imu_pl::RunLoggingSession logger_;
  std::multiset<Event, EventLess> queue_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::atomic<std::uint64_t> sequence_{1};
  std::atomic<std::uint64_t> queue_overflow_count_{0};
  std::atomic<std::uint64_t> dropped_uwb_count_{0};
  bool imu_overflow_pending_ = false;
  double worker_start_delay_s_ = 0.0;
  std::string run_id_;
  uwb_imu_pl::TimestampNs latest_received_imu_;
  std::optional<uwb_imu_pl::TimestampNs> last_processed_imu_;
  std::optional<uwb_imu_pl::ImuMeasurement> last_imu_measurement_;
  std::optional<uwb_imu_pl::EventOrderKey> last_processed_event_;
  uwb_imu_pl::EstimationTuningV1 tuning_;
  std::vector<uwb_imu_pl::ImuMeasurement> bootstrap_imu_;
  std::vector<uwb_imu_pl::UwbMeasurement> bootstrap_ranges_;
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
  std::string fde_profile_override;
  std::string execution_command;
  bool enable_run_logging = false;
  bool subscribe_truth_topics = true;
  double worker_start_delay_s = 0.0;
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
  private_node.param("fde_profile", fde_profile_override, std::string());
  private_node.param("execution_command", execution_command, std::string());
  private_node.param("enable_run_logging", enable_run_logging, false);
  private_node.param("subscribe_truth_topics", subscribe_truth_topics, true);
  private_node.param("worker_start_delay_s", worker_start_delay_s, 0.0);
  if (!std::isfinite(worker_start_delay_s) || worker_start_delay_s < 0.0) {
    ROS_FATAL("~worker_start_delay_s must be finite and non-negative");
    return 2;
  }
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
    if (!fde_profile_override.empty()) {
      overrides.fde_profile =
          uwb_imu_pl::parseFdeProfile(fde_profile_override);
    }
    if (!output_root_override.empty()) overrides.output_root = output_root_override;
    auto config = uwb_imu_pl::IntegrityConfigLoader::load(config_path, overrides);
    if (execution_command.empty()) {
      execution_command = processCommandLine(argc, argv);
    }
    ROS_INFO_STREAM("resolved fixed_lag_epochs="
                    << config.incremental.fixed_lag_epochs
                    << " config_hash=" << config.config_hash);
    std::unique_ptr<tbb::global_control> nominal_threads;
    if (config.fde.profile == uwb_imu_pl::FdeProfile::Off)
      nominal_threads.reset(new tbb::global_control(
          tbb::global_control::max_allowed_parallelism, 1));
    RealtimeNode realtime(node, std::move(config), enable_run_logging,
                          run_directory,
                          execution_command, subscribe_truth_topics,
                          worker_start_delay_s);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}
