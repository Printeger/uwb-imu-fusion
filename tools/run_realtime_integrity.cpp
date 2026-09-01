#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uwb_imu_fgo/IntegrityStatus.h>
#include <uwb_imu_fgo/LinktrackNodeframe3.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
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
  enum class Kind { Imu = 0, Uwb = 1 };
  Kind kind = Kind::Imu;
  uwb_imu_pl::TimestampNs timestamp;
  std::uint64_t sequence = 0;
  sensor_msgs::Imu imu;
  uwb_imu_fgo::LinktrackNodeframe3 uwb;
};

struct EventLess {
  bool operator()(const Event& left, const Event& right) const {
    if (left.timestamp != right.timestamp) return left.timestamp < right.timestamp;
    if (left.kind != right.kind) return left.kind < right.kind;
    return left.sequence < right.sequence;
  }
};

class RealtimeNode {
 public:
  RealtimeNode(ros::NodeHandle& node, uwb_imu_pl::IntegrityConfig config)
      : node_(node), config_(std::move(config)) {
    for (const auto& anchor : config_.anchors) {
      anchors_.emplace(anchor.id.value(), anchor);
    }
    const std::string run_directory = config_.output.root + "/online_" +
        std::to_string(ros::WallTime::now().toNSec());
    logger_.reset(new uwb_imu_pl::RunLogger(run_directory));
    logger_->writeResolvedConfig(config_.resolved_yaml);
    logger_->writeManifest(uwb_imu_pl::makeRunManifest(
        config_, UWB_IMU_PL_GIT_SHA, UWB_IMU_PL_GIT_DIRTY != 0));

    odometry_publisher_ = node_.advertise<nav_msgs::Odometry>(
        config_.realtime.odometry_topic, 10);
    integrity_publisher_ = node_.advertise<uwb_imu_fgo::IntegrityStatus>(
        config_.realtime.integrity_topic, 10);
    diagnostics_publisher_ = node_.advertise<diagnostic_msgs::DiagnosticArray>(
        config_.realtime.diagnostics_topic, 10);
    imu_subscriber_ = node_.subscribe(
        config_.realtime.imu_topic, 1000, &RealtimeNode::imuCallback, this);
    uwb_subscriber_ = node_.subscribe(
        config_.realtime.uwb_topic, 100, &RealtimeNode::uwbCallback, this);
    worker_ = std::thread(&RealtimeNode::workerLoop, this);
  }

  ~RealtimeNode() {
    stop_.store(true);
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (logger_) logger_->writeSummary(
        "IMPLEMENTED_UNVERIFIED", "ROS1 realtime node stopped");
  }

 private:
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

  void uwbCallback(const uwb_imu_fgo::LinktrackNodeframe3::ConstPtr& message) {
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

  void initialize(uwb_imu_pl::TimestampNs time) {
    uwb_imu_pl::NavigationState initial;
    initial.timestamp = time;
    initial.position_world_m = config_.realtime.initial_position_m;
    initial.velocity_world_mps = config_.realtime.initial_velocity_mps;
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

  void processImu(const Event& event) {
    if (!estimator_) {
      initialize(event.timestamp);
      return;
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
  }

  void processUwb(const Event& event) {
    if (!pipeline_) throw std::runtime_error("UWB received before initialization");
    const auto output = pipeline_->processUwbBatch(convert(event));
    publish(output);
    logger_->writeState(output.state);
    logger_->writeIntegrity(output);
    logger_->writeTiming(output.timestamp, "update_no_current_uwb",
                         estimator_->lastNoUwbUpdateMs(), true);
    logger_->writeTiming(output.timestamp, "current_joint_marginal",
                         estimator_->lastMarginalMs(), true);
    logger_->writeTiming(output.timestamp, "update_current_uwb",
                         estimator_->lastUwbUpdateMs(), output.batch_committed);
    logger_->writeEvent(output.timestamp,
                        output.batch_committed ? "UWB_COMMIT" : "UWB_REJECT",
                        output.detector.reason);
  }

  void workerLoop() {
    while (!stop_.load() && ros::ok()) {
      Event event;
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
      }
      try {
        if (event.kind == Event::Kind::Imu) processImu(event);
        else processUwb(event);
      } catch (const std::exception& error) {
        ROS_ERROR_STREAM("realtime integrity event rejected: " << error.what());
        logger_->writeEvent(event.timestamp, "PROCESSING_ERROR", error.what());
      }
    }
  }

  void publish(const uwb_imu_pl::IntegrityOutput& output) {
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

    uwb_imu_fgo::IntegrityStatus status;
    status.header = odometry.header;
    status.scope_label = uwb_imu_pl::toString(output.protection_level.label);
    status.availability = uwb_imu_pl::toString(output.protection_level.availability);
    status.detector_passed = output.detector.passed;
    status.batch_committed = output.batch_committed;
    status.statistic = output.detector.statistic;
    status.threshold = output.detector.threshold;
    status.dof = output.detector.dof;
    for (int axis = 0; axis < 3; ++axis) {
      status.pl_xyz_m[axis] = output.protection_level.pl_xyz_m(axis);
      status.maximizing_anchor_id[axis] =
          output.protection_level.maximizing_anchor[axis].value();
    }
    status.hpl_box_m = output.protection_level.hpl_box_m;
    status.vpl_m = output.protection_level.vpl_m;
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
    diagnostics.status.push_back(item);
    diagnostics_publisher_.publish(diagnostics);
  }

  ros::NodeHandle node_;
  uwb_imu_pl::IntegrityConfig config_;
  std::map<std::uint64_t, uwb_imu_pl::AnchorRecord> anchors_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber uwb_subscriber_;
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
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "uwb_imu_pl_realtime");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  std::string config_path;
  private_node.param("config_path", config_path, std::string());
  if (config_path.empty()) {
    ROS_FATAL("~config_path is required");
    return 2;
  }
  try {
    RealtimeNode realtime(node,
        uwb_imu_pl::IntegrityConfigLoader::load(config_path));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
  return 0;
}
