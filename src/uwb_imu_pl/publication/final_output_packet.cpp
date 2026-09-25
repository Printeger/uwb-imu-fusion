#include "uwb_imu_pl/publication/final_output_packet.hpp"

#include "uwb_imu_pl/common/integrity_identity.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

std::mutex& committedStateMutex() {
  static std::mutex value;
  return value;
}

std::map<std::pair<std::uint64_t, std::int64_t>,
         CommittedStatePublicationV1>& committedStates() {
  static std::map<std::pair<std::uint64_t, std::int64_t>,
                  CommittedStatePublicationV1> value;
  return value;
}

void appendReason(IntegrityOutput* output, const std::string& reason) {
  if (std::find(output->reason_codes.begin(), output->reason_codes.end(),
                reason) == output->reason_codes.end()) {
    output->reason_codes.push_back(reason);
  }
}

void refuseProtection(IntegrityOutput* output, const std::string& reason,
                      bool deadline_missed) {
  output->deadline_missed = output->deadline_missed || deadline_missed;
  output->publication.protected_output = false;
  output->publication.unprotected_output = true;
  output->protection_level.formal_eligible = false;
  output->publication.refusal = reason;
  appendReason(output, reason);
}

}  // namespace

bool recordCommittedStatePublicationV1(
    const CommittedStatePublicationV1& sidecar) {
  if (sidecar.schema_version != 1 || sidecar.transaction_id == 0 ||
      sidecar.state_timestamp.value() == 0 ||
      !sidecar.covariance.allFinite()) return false;
  std::lock_guard<std::mutex> lock(committedStateMutex());
  committedStates()[{sidecar.transaction_id,
                     sidecar.state_timestamp.value()}] = sidecar;
  return true;
}

bool committedStatePublicationV1(
    std::uint64_t transaction_id, TimestampNs state_timestamp,
    CommittedStatePublicationV1* sidecar) {
  if (!sidecar) return false;
  std::lock_guard<std::mutex> lock(committedStateMutex());
  const auto found = committedStates().find(
      {transaction_id, state_timestamp.value()});
  if (found == committedStates().end()) return false;
  *sidecar = found->second;
  return true;
}

FinalOutputPacket finalizeOutputPacket(IntegrityOutput output,
                                       FinalPacketTiming timing,
                                       std::uint64_t deadline_ns) {
  const bool search_incomplete = output.fde_status == "SEARCH_INCOMPLETE" ||
      std::find(output.reason_codes.begin(), output.reason_codes.end(),
                "SEARCH_INCOMPLETE") != output.reason_codes.end();
  if (search_incomplete) {
    output.protection_level.availability = Availability::Unavailable;
    output.protection_level.risk_budget_valid = false;
    refuseProtection(&output, "SEARCH_INCOMPLETE", false);
  }
  const bool clock_valid = timing.arrival_steady_ns > 0 &&
      timing.compute_done_steady_ns >= timing.arrival_steady_ns &&
      timing.packet_ready_steady_ns >= timing.compute_done_steady_ns &&
      (timing.publish_call_steady_ns == 0 ||
       timing.publish_call_steady_ns >= timing.packet_ready_steady_ns) &&
      (timing.publish_return_steady_ns == 0 ||
       (timing.publish_call_steady_ns != 0 &&
        timing.publish_return_steady_ns >= timing.publish_call_steady_ns));
  const std::uint64_t boundary = timing.publish_return_steady_ns != 0
      ? timing.publish_return_steady_ns
      : (timing.deadline_boundary == FinalPacketBoundary::PublishCall &&
                 timing.publish_call_steady_ns != 0
             ? timing.publish_call_steady_ns
             : timing.packet_ready_steady_ns);
  const bool elapsed_valid = clock_valid && boundary >= timing.arrival_steady_ns;
  const std::uint64_t elapsed = elapsed_valid
      ? boundary - timing.arrival_steady_ns
      : std::numeric_limits<std::uint64_t>::max();

  const bool completion_uncertified = std::find(
      output.reason_codes.begin(), output.reason_codes.end(),
      "PUBLICATION_COMPLETION_UNCERTIFIED") != output.reason_codes.end();
  if ((!clock_valid || timing.attempt_kind == FinalAttemptKind::ClockInvalid) &&
      !completion_uncertified) {
    refuseProtection(&output, "PUBLICATION_CLOCK_INVALID", true);
  }
  if (elapsed > deadline_ns && !completion_uncertified) {
    output.publication.wall_timeout = true;
    refuseProtection(&output, "PUBLICATION_DEADLINE_MISSED", true);
  }
  if (timing.attempt_kind == FinalAttemptKind::QueueOverflow &&
      !completion_uncertified) {
    refuseProtection(&output, "IMU_QUEUE_OVERFLOW", false);
  } else if (timing.attempt_kind == FinalAttemptKind::Exception &&
             !completion_uncertified) {
    refuseProtection(&output, "PUBLICATION_EXCEPTION", false);
  }

  const double raw_age = output.attempted_timestamp.seconds() -
      output.state.timestamp.seconds();
  const double state_age = std::isfinite(raw_age) ? std::max(0.0, raw_age) : 0.0;
  output.stale_state = output.stale_state ||
      output.state.timestamp != output.attempted_timestamp;
  output.fresh = output.fresh && !output.stale_state;

  FinalOutputPacket packet;
  packet.output_ = std::move(output);
  packet.timing_ = timing;
  packet.state_age_s_ = state_age;
  ProtectionLevelPublicationPacketV2 protection;
  if (protectionLevelPublicationPacketV2(
          packet.output_.protection_level.detector_certificate_id,
          &protection)) {
    packet.metadata_.protected_frame_id = protection.frame_id;
    packet.metadata_.protected_position_reference =
        protection.position_reference;
    packet.metadata_.protection_packet_id =
        packet.output_.protection_level.detector_certificate_id;
  }
  packet.metadata_.digest = finalOutputPacketDigest(packet);
  return packet;
}

std::uint64_t finalOutputPacketDigest(const FinalOutputPacket& packet) {
  const auto& output = packet.output();
  const auto& timing = packet.timing();
  std::ostringstream text;
  text << std::setprecision(17)
       << output.diagnostics.input_attempt_id << '|' << output.transaction_id
       << '|' << output.linearization_version << '|'
       << output.attempted_timestamp.value() << '|'
       << output.state.timestamp.value() << '|'
       << output.state.q_world_body.w() << '|'
       << output.state.q_world_body.vec().transpose() << '|'
       << output.state.position_world_m.transpose() << '|'
       << output.state.velocity_world_mps.transpose() << '|'
       << output.state.accel_bias_mps2.transpose() << '|'
       << output.state.gyro_bias_radps.transpose() << '|'
       << output.protection_level.pl_xyz_m.transpose() << '|'
       << output.protection_level.hpl_m << '|'
       << output.protection_level.vpl_m << '|'
       << output.protection_level.formal_eligible << '|'
       << output.protection_level.risk_budget_valid << '|'
       << output.protection_level.allocated_hmi_risk << '|'
       << output.protection_level.hmi_risk_requirement << '|'
       << output.scope_digest << '|' << output.detector_contract_id << '|'
       << packet.metadata().protected_frame_id << '|'
       << packet.metadata().protected_position_reference << '|'
       << packet.metadata().protection_packet_id << '|'
       << output.selected_action_id << '|' << output.selected_action_type << '|'
       << output.fde_status << '|' << output.protection_level.reason << '|'
       << output.deadline_missed << '|' << output.stale_state << '|'
       << output.publication.protected_output << '|'
       << output.publication.certificate_id;
  for (const auto& reason : output.reason_codes) text << '|' << reason;
  return identityHash64(text.str());
}

FinalOutputPacket invokeFinalOutputPacket(
    IntegrityOutput output, FinalPacketTiming timing,
    std::uint64_t deadline_ns, const FinalPacketClock& clock,
    FinalPacketPublicationTransaction* transaction,
    std::string* publication_error) {
  if (!clock || !transaction) {
    throw std::invalid_argument(
        "final packet clock/publication transaction is required");
  }
  const bool atomic_receipt = transaction->supportsAtomicProtectedReceipt();
  if (!atomic_receipt) {
    // A fallible fan-out cannot know its final return time while constructing
    // the bytes sent to its first sink.  Publish one conservative immutable
    // mirror instead: it is never authoritative/protected and its completion
    // is deliberately treated as uncertified.  The same semantic packet and
    // digest are returned after the real calls finish, irrespective of which
    // fan-out member failed.
    refuseProtection(&output, "ATOMIC_PROTECTED_RECEIPT_UNAVAILABLE", false);
    refuseProtection(&output, "PUBLICATION_COMPLETION_UNCERTIFIED", true);
    output.publication.wall_timeout = true;
  }
  timing.deadline_boundary = FinalPacketBoundary::PublishReturn;
  timing.publish_call_steady_ns = clock();
  // Anything visible before the actual transport return is deliberately a
  // non-authoritative candidate.  In particular, a later sink exception can
  // never strand a protected compound or legacy message.
  IntegrityOutput candidate_output = output;
  if (atomic_receipt) {
    refuseProtection(&candidate_output,
                     "NON_AUTHORITATIVE_UNPROTECTED_CANDIDATE", false);
  }
  FinalPacketTiming candidate_timing = timing;
  candidate_timing.publish_outcome = FinalPublishOutcome::NotAttempted;
  const auto candidate = finalizeOutputPacket(
      std::move(candidate_output), candidate_timing, deadline_ns);
  try {
    transaction->publishCandidate(candidate);
    timing.publish_outcome = FinalPublishOutcome::Success;
  } catch (const std::exception& error) {
    if (publication_error) *publication_error = error.what();
    timing.attempt_kind = FinalAttemptKind::Exception;
    timing.publish_outcome = FinalPublishOutcome::PublisherException;
  } catch (...) {
    if (publication_error) *publication_error = "unknown publisher exception";
    timing.attempt_kind = FinalAttemptKind::Exception;
    timing.publish_outcome = FinalPublishOutcome::PublisherException;
  }
  timing.publish_return_steady_ns = clock();
  FinalOutputPacket terminal = finalizeOutputPacket(
      std::move(output), timing, deadline_ns);
  if (timing.publish_outcome == FinalPublishOutcome::Success) {
    if (atomic_receipt) {
      terminal.metadata_.authoritative = true;
      transaction->commitReceipt(terminal);
    }
  } else {
    transaction->abort();
  }
  return terminal;
}

ChildFrameLinearTwist childFrameLinearTwist(
    const Eigen::Quaterniond& q_world_body,
    const Eigen::Vector3d& velocity_world_mps,
    const Eigen::Matrix3d& covariance_world_m2ps2) {
  if (!q_world_body.coeffs().allFinite() || q_world_body.norm() == 0.0 ||
      !velocity_world_mps.allFinite() || !covariance_world_m2ps2.allFinite()) {
    throw std::invalid_argument("child-frame twist inputs must be finite");
  }
  const Eigen::Matrix3d body_from_world =
      q_world_body.normalized().toRotationMatrix().transpose();
  ChildFrameLinearTwist result;
  result.velocity_body_mps = body_from_world * velocity_world_mps;
  result.covariance_body_m2ps2 = body_from_world * covariance_world_m2ps2 *
      body_from_world.transpose();
  result.covariance_body_m2ps2 = 0.5 *
      (result.covariance_body_m2ps2 + result.covariance_body_m2ps2.transpose());
  return result;
}

std::uint64_t p005AbiPublicationDiagnosticsByValue(
    PublicationDiagnostics value) {
  return value.certificate_id;
}

std::uint64_t p005AbiIntegrityOutputByValue(IntegrityOutput value) {
  return value.transaction_id ^ (value.publication.certificate_id << 1U);
}

std::uint64_t p005AbiIntegrityOutputArray(const IntegrityOutput* values,
                                         std::size_t count) {
  std::uint64_t result = count;
  for (std::size_t index = 0; index < count; ++index) {
    result ^= values[index].transaction_id << (index + 1U);
    result ^= values[index].publication.certificate_id << (index + 3U);
  }
  return result;
}

std::uint64_t p005AbiIntegrityOutputVector(
    std::vector<IntegrityOutput> values) {
  return p005AbiIntegrityOutputArray(values.data(), values.size());
}

}  // namespace uwb_imu_pl
