#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/io/run_logger.hpp"
#include "uwb_imu_pl/publication/final_output_packet.hpp"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <uwb_imu_pl/IntegrityStatus.h>
#include <uwb_imu_pl/NavigationIntegrity.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

std::uint64_t steadyNowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

ros::Time rosTime(uwb_imu_pl::TimestampNs value) {
  ros::Time result;
  result.fromNSec(static_cast<std::uint64_t>(value.value()));
  return result;
}

uwb_imu_pl::IntegrityConfig config() {
  uwb_imu_pl::IntegrityConfig value;
  value.incremental.relinearize_threshold = 0.1;
  value.incremental.relinearize_skip = 1;
  value.snapshot.rank_tolerance = 1e-10;
  value.snapshot.max_condition_number = 1e12;
  value.imu.accelerometer_sigma = 0.1;
  value.imu.gyroscope_sigma = 0.01;
  value.imu.accelerometer_bias_rw_sigma = 0.001;
  value.imu.gyroscope_bias_rw_sigma = 0.0001;
  value.imu.gravity_mps2 = 9.80665;
  value.realtime.world_frame = "map";
  value.realtime.body_frame = "base_link";
  return value;
}

uwb_imu_pl::UwbBatch batch(uwb_imu_pl::TimestampNs timestamp) {
  uwb_imu_pl::UwbBatch value;
  value.id = uwb_imu_pl::BatchId(1);
  value.timestamp = timestamp;
  value.covariance_model_id = "p005_ros_certified";
  const Eigen::Vector3d position(0, 0, 1);
  const Eigen::Vector3d anchors[] = {
      {-5, -5, 0}, {5, -5, 1}, {5, 5, 3}, {-5, 5, 4}, {0, -6, 2}};
  for (std::size_t index = 0; index < 5; ++index) {
    uwb_imu_pl::UwbMeasurement measurement;
    measurement.id = uwb_imu_pl::MeasurementId(index + 1);
    measurement.factor_id = uwb_imu_pl::FactorId(index + 1);
    measurement.anchor_id = uwb_imu_pl::AnchorId(index + 1);
    measurement.timestamp = timestamp;
    measurement.anchor_position_m = anchors[index];
    measurement.range_m = (position - anchors[index]).norm();
    measurement.sigma_m = 0.1;
    value.measurements.push_back(measurement);
  }
  return value;
}

class RosCommit final : public uwb_imu_pl::FinalPacketPublicationTransaction {
 public:
  RosCommit(ros::Publisher* compound, ros::Publisher* odometry,
            ros::Publisher* integrity, std::string packet_id)
      : compound_(compound), odometry_(odometry), integrity_(integrity),
        packet_id_(std::move(packet_id)) {}

  void publishCandidate(
      const uwb_imu_pl::FinalOutputPacket& packet) override {
    publish(packet, true);
    digest_ = uwb_imu_pl::finalOutputPacketDigest(packet);
  }
  void abort() noexcept override { aborted_ = true; }

  void commitReceipt(
      const uwb_imu_pl::FinalOutputPacket&) noexcept override {}

 private:
  void publish(const uwb_imu_pl::FinalOutputPacket& packet, bool visible) {
    try {
      const auto& output = packet.output();
      nav_msgs::Odometry odometry;
      odometry.header.stamp = rosTime(output.state.timestamp);
      odometry.header.frame_id = "map";
      odometry.child_frame_id = "base_link";
      odometry.pose.pose.position.x = output.state.position_world_m.x();
      odometry.pose.pose.position.y = output.state.position_world_m.y();
      odometry.pose.pose.position.z = output.state.position_world_m.z();
      odometry.pose.pose.orientation.w = output.state.q_world_body.w();
      odometry.pose.pose.orientation.x = output.state.q_world_body.x();
      odometry.pose.pose.orientation.y = output.state.q_world_body.y();
      odometry.pose.pose.orientation.z = output.state.q_world_body.z();
      const auto child = uwb_imu_pl::childFrameLinearTwist(
          output.state.q_world_body, output.state.velocity_world_mps,
          covariance_.block<3, 3>(6, 6));
      odometry.twist.twist.linear.x = child.velocity_body_mps.x();
      odometry.twist.twist.linear.y = child.velocity_body_mps.y();
      odometry.twist.twist.linear.z = child.velocity_body_mps.z();
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          odometry.twist.covariance[row * 6 + column] =
              child.covariance_body_m2ps2(row, column);
        }
      }

      uwb_imu_pl::IntegrityStatus status;
      status.header = odometry.header;
      status.attempted_timestamp = rosTime(output.attempted_timestamp);
      status.state_timestamp = rosTime(output.state.timestamp);
      status.publish_timestamp = ros::Time::now();
      status.attempt_id = output.diagnostics.input_attempt_id;
      status.solution_id = output.linearization_version;
      status.transaction_id = output.transaction_id;
      status.window_id = output.window_id;
      status.base_graph_version = output.base_graph_version;
      status.linearization_version = output.linearization_version;
      status.batch_committed = output.batch_committed;
      status.backend_updates = output.backend_updates;
      status.state_valid = output.state_valid;
      status.fresh = output.fresh;
      status.deadline_missed = output.deadline_missed;
      status.publication_protected = output.publication.protected_output;
      status.publication_certificate_id = output.publication.certificate_id;
      status.final_packet_protocol_version = packet.metadata().protocol_version;
      status.final_packet_digest = packet.metadata().digest;
      status.final_packet_authoritative = packet.metadata().authoritative;
      status.protected_frame_id = packet.metadata().protected_frame_id;
      status.protected_position_reference =
          packet.metadata().protected_position_reference;
      status.protection_packet_id = packet.metadata().protection_packet_id;
      status.reason_codes = output.reason_codes;
      status.pl_status = uwb_imu_pl::toString(output.pl_status);
      status.hpl_m = output.protection_level.hpl_m;
      status.vpl_m = output.protection_level.vpl_m;
      status.pl_xyz_m[0] = output.protection_level.pl_xyz_m.x();
      status.pl_xyz_m[1] = output.protection_level.pl_xyz_m.y();
      status.pl_xyz_m[2] = output.protection_level.pl_xyz_m.z();
      status.selected_action_id = output.selected_action_id;
      status.selected_action_type = output.selected_action_type;
      status.fde_status = output.fde_status;
      status.reason = output.protection_level.reason;

      uwb_imu_pl::NavigationIntegrity compound;
      compound.header = status.header;
      compound.attempted_timestamp = status.attempted_timestamp;
      compound.state_timestamp = status.state_timestamp;
      compound.publish_timestamp = status.publish_timestamp;
      compound.arrival_steady_ns = packet.timing().arrival_steady_ns;
      compound.finish_steady_ns = packet.timing().publish_return_steady_ns;
      compound.deadline_missed = output.deadline_missed;
      compound.run_id = run_id_;
      compound.attempt_id = status.attempt_id;
      compound.transaction_id = status.transaction_id;
      compound.solution_id = status.solution_id;
      compound.scope_digest = "p005-certified-reference-transfer";
      compound.manifest_digest = "p005-controlled-input-v1";
      compound.odometry = odometry;
      compound.integrity = status;
      compound.state_valid = output.state_valid;
      compound.fresh = output.fresh;
      compound.batch_committed = output.batch_committed;
      compound.commit_reason = status.reason;
      compound.selected_action_id = output.selected_action_id;
      compound.pl_xyz_m[0] = status.pl_xyz_m[0];
      compound.pl_xyz_m[1] = status.pl_xyz_m[1];
      compound.pl_xyz_m[2] = status.pl_xyz_m[2];
      compound.hpl_m = status.hpl_m;
      compound.vpl_m = status.vpl_m;
      compound.pl_status = status.pl_status;
      compound.publication_protected = false;
      compound.final_packet_protocol_version =
          status.final_packet_protocol_version;
      compound.final_packet_digest = status.final_packet_digest;
      compound.final_packet_authoritative =
          status.final_packet_authoritative;
      compound.protected_frame_id = status.protected_frame_id;
      compound.protected_position_reference =
          status.protected_position_reference;
      compound.protection_packet_id = status.protection_packet_id;
      compound.reason_codes = output.reason_codes;

      if (visible) {
        compound_->publish(compound);
        odometry_->publish(odometry);
        // Standalone status is a correlated legacy mirror, never an
        // authoritative receipt even though it carries the common digest.
        status.publication_protected = false;
        status.final_packet_authoritative = false;
        integrity_->publish(status);
        if (logger_) {
          logger_->writeState(output.state);
          logger_->writeIntegrity(output, packet.metadata());
          logger_->flush();
        }
      } else {
        (void)ros::serialization::serializationLength(compound);
        (void)ros::serialization::serializationLength(odometry);
        (void)ros::serialization::serializationLength(status);
      }
    } catch (...) {
      aborted_ = true;
      throw;
    }
  }

 public:
  void setCovariance(const Eigen::Matrix<double, 15, 15>& covariance) {
    covariance_ = covariance;
  }
  void setRunId(std::string run_id) { run_id_ = std::move(run_id); }
  void setLogger(uwb_imu_pl::RunLogger* logger) { logger_ = logger; }
  std::uint64_t digest() const { return digest_; }
  bool aborted() const { return aborted_; }

  ros::Publisher* compound_;
  ros::Publisher* odometry_;
  ros::Publisher* integrity_;
  uwb_imu_pl::RunLogger* logger_ = nullptr;
  std::string packet_id_;
  std::string run_id_;
  Eigen::Matrix<double, 15, 15> covariance_ =
      Eigen::Matrix<double, 15, 15>::Zero();
  std::uint64_t digest_ = 0;
  bool aborted_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "p005_ros_certified_publisher");
  ros::NodeHandle private_node("~");
  std::string run_directory = "/tmp/p005_ros_certified";
  private_node.param("run_directory", run_directory, run_directory);

  const auto cfg = config();
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(
      cfg, Eigen::Vector3d::Zero());
  uwb_imu_pl::NavigationState initial;
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, cfg.realtime.prior_sigmas);
  for (int index = 0; index <= 2; ++index) {
    uwb_imu_pl::ImuMeasurement imu;
    imu.id = uwb_imu_pl::MeasurementId(index + 1);
    imu.timestamp = uwb_imu_pl::TimestampNs(index * 5000000);
    imu.specific_force_mps2 = {0, 0, cfg.imu.gravity_mps2};
    estimator.ingestImu(imu);
  }

  auto transaction = estimator.prepareEpoch(batch(uwb_imu_pl::TimestampNs(10000000)));
  const auto window = estimator.buildIntegrityWindow(
      transaction, uwb_imu_pl::IntegrityWindowRequest{});
  if (!window.model_valid || !window.numerics) return 2;
  uwb_imu_pl::ExclusionAction keep;
  keep.id = uwb_imu_pl::ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  uwb_imu_pl::RankUpdateConfig rank;
  rank.rank_tolerance = window.numerics->numerical_contract.rank_tolerance;
  rank.max_condition_number =
      window.numerics->numerical_contract.max_condition_number;
  rank.max_linearization_step_norm = 100.0;
  const uwb_imu_pl::RankUpdateEvaluator evaluator(rank);
  auto candidate = evaluator.evaluate(evaluator.factorizeOnce(window, {keep}),
                                      keep);
  uwb_imu_pl::DetectorRiskContext detector_risk;
  detector_risk.p_fa_per_test = 1e-6;
  detector_risk.rank_tolerance = rank.rank_tolerance;
  detector_risk.max_condition_number = rank.max_condition_number;
  const auto detector = uwb_imu_pl::JointWindowDetector().evaluateCandidate(
      window, candidate, detector_risk);
  if (!candidate.valid || !detector.numerically_valid || !detector.passed) return 3;

  Eigen::MatrixXd mode = Eigen::MatrixXd::Zero(candidate.rows, 1);
  mode(0, 0) = 1.0;
  mode(mode.rows() - 1, 0) = 1.0;
  uwb_imu_pl::FaultHypothesisV2 hypothesis;
  hypothesis.id = uwb_imu_pl::HypothesisId(2);
  hypothesis.modes = {uwb_imu_pl::FaultModeId(3)};
  hypothesis.A = mode;
  hypothesis.p_md_allocation = 1e-3;
  hypothesis.hmi_allocation = 1e-6;
  hypothesis.prior_probability_bound = 1e-3;
  std::vector<uwb_imu_pl::FaultHypothesisV2> hypotheses{hypothesis};
  uwb_imu_pl::ProtectionLevelSharedContext shared;
  shared.mode_maps.emplace(3, mode);
  const auto pl = uwb_imu_pl::ProtectionLevelV2().computeShared(
      window, &candidate, detector, &hypotheses, shared,
      uwb_imu_pl::RiskBudgetV2{});
  const auto proof = uwb_imu_pl::protectionLevelV2ProofIdentity(candidate, pl);
  if (!pl.model_valid || proof == 0) return 4;
  uwb_imu_pl::CommitProtectionEvidenceV1 evidence;
  std::string reason;
  if (!uwb_imu_pl::mintCommitProtectionEvidenceV1(
          transaction, window, candidate, pl, proof, "map", &evidence,
          &reason)) return 5;
  const auto base_graph_version = transaction.base_graph_version;
  const auto plan = uwb_imu_pl::EpochCommitPlan::nominalPlan(transaction);
  uwb_imu_pl::CommitCertificationV1 certification;
  const auto receipt = estimator.commitEpochCertified(
      std::move(transaction), plan, &evidence, &certification);
  if (!receipt.integrity_available || !certification.reference_bound) return 6;
  std::string packet_id;
  if (!uwb_imu_pl::bindTransferredProtectionLevelPublicationPacket(
          candidate, pl, proof, certification.protected_reference.mean_world_m,
          certification.committed_mean_world_m,
          certification.transferred_pl_m, receipt.state_timestamp.value(),
          "map", "body_origin", &packet_id) ||
      !uwb_imu_pl::validateProtectionLevelPublicationProof(
          proof, certification.transferred_pl_m, packet_id, &reason)) return 7;

  uwb_imu_pl::IntegrityOutput output;
  output.timestamp = receipt.state_timestamp;
  output.attempted_timestamp = receipt.state_timestamp;
  output.state = estimator.currentState();
  output.state_valid = true;
  output.fresh = true;
  output.batch_committed = true;
  output.transaction_id = receipt.transaction_id.value();
  output.window_id = window.id.value();
  output.base_graph_version = base_graph_version;
  output.linearization_version = receipt.graph_version;
  output.backend_updates = receipt.backend_updates;
  output.selected_action_id = keep.id.value();
  output.selected_action_type = keep.action_model_id;
  output.fde_status = "SUCCESS_KEEP_ALL";
  output.protection_level.pl_xyz_m = certification.transferred_pl_m;
  output.protection_level.hpl_m = std::hypot(
      certification.transferred_pl_m.x(), certification.transferred_pl_m.y());
  output.protection_level.vpl_m = certification.transferred_pl_m.z();
  output.protection_level.detector_certificate_id = packet_id;
  output.protection_level.availability = uwb_imu_pl::Availability::Unavailable;
  output.protection_level.formal_eligible = false;
  output.protection_level.reason =
      "CERTIFIED_COMMITTED_UNPROTECTED_REFERENCE_TRANSFER:" + packet_id;
  output.pl_status = uwb_imu_pl::ProtectionLevelStatus::Finite;
  output.measurement_model_valid = true;
  output.publication.protected_output = false;
  output.publication.unprotected_output = true;
  output.reason_codes = {"FORMAL_GATE_CLOSED",
                         "CERTIFIED_REFERENCE_TRANSFER_COMMITTED_UNPROTECTED"};
  output.diagnostics.input_attempt_id = 1;

  ros::NodeHandle node;
  auto compound = node.advertise<uwb_imu_pl::NavigationIntegrity>(
      "/uwb_imu_pl/solution", 1, true);
  auto odometry = node.advertise<nav_msgs::Odometry>(
      "/uwb_imu_pl/odometry", 1, true);
  auto integrity = node.advertise<uwb_imu_pl::IntegrityStatus>(
      "/uwb_imu_pl/integrity", 1, true);
  const auto wait_end = std::chrono::steady_clock::now() +
      std::chrono::seconds(20);
  while (ros::ok() && std::chrono::steady_clock::now() < wait_end &&
         (compound.getNumSubscribers() == 0 ||
          odometry.getNumSubscribers() == 0 ||
          integrity.getNumSubscribers() == 0)) {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (compound.getNumSubscribers() == 0 || odometry.getNumSubscribers() == 0 ||
      integrity.getNumSubscribers() == 0) return 8;

  RosCommit publication(&compound, &odometry, &integrity, packet_id);
  publication.setCovariance(certification.committed_covariance);
  publication.setRunId(run_directory);
  uwb_imu_pl::RunLogger logger(run_directory, false, false);
  publication.setLogger(&logger);
  uwb_imu_pl::FinalPacketTiming timing;
  timing.arrival_steady_ns = steadyNowNs();
  timing.compute_done_steady_ns = timing.arrival_steady_ns;
  timing.packet_ready_steady_ns = timing.arrival_steady_ns;
  const auto packet = uwb_imu_pl::invokeFinalOutputPacket(
      output, timing, 1000000000ULL, [] { return steadyNowNs(); },
      &publication, &reason);
  if (publication.aborted() || publication.digest() == 0 ||
      publication.digest() != uwb_imu_pl::finalOutputPacketDigest(packet)) return 9;

  logger.flush();
  ros::spinOnce();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return 0;
}
