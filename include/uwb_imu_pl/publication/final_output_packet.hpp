#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace uwb_imu_pl {

enum class FinalPacketBoundary : std::uint8_t {
  PacketReady = 0,
  PublishCall = 1,
  PublishReturn = 2,
};

enum class FinalAttemptKind : std::uint8_t {
  Normal = 0,
  ClockInvalid = 1,
  QueueOverflow = 2,
  Exception = 3,
};

enum class FinalPublishOutcome : std::uint8_t {
  NotAttempted = 0,
  Success = 1,
  PublisherException = 2,
};

struct FinalPacketTiming {
  std::uint64_t arrival_steady_ns = 0;
  std::uint64_t compute_done_steady_ns = 0;
  std::uint64_t packet_ready_steady_ns = 0;
  std::uint64_t publish_call_steady_ns = 0;
  std::uint64_t publish_return_steady_ns = 0;
  FinalPacketBoundary deadline_boundary = FinalPacketBoundary::PublishCall;
  FinalAttemptKind attempt_kind = FinalAttemptKind::Normal;
  FinalPublishOutcome publish_outcome = FinalPublishOutcome::NotAttempted;
};

// Versioned terminal-publication metadata.  This is intentionally separate
// from PublicationDiagnostics/IntegrityOutput, whose golden-p0-04 public ABI
// is frozen (including by-value and container use).
struct FinalPacketMetadataV2 {
  std::uint32_t protocol_version = 2;
  std::uint64_t digest = 0;
  bool authoritative = false;
  std::string protected_frame_id;
  std::string protected_position_reference;
  std::string protection_packet_id;
};

using FinalPacketClock = std::function<std::uint64_t()>;
class FinalPacketPublicationTransaction;

class FinalOutputPacket {
 public:
  const IntegrityOutput& output() const noexcept { return output_; }
  const FinalPacketTiming& timing() const noexcept { return timing_; }
  const FinalPacketMetadataV2& metadata() const noexcept { return metadata_; }
  double stateAgeSeconds() const noexcept { return state_age_s_; }

 private:
  friend FinalOutputPacket finalizeOutputPacket(
      IntegrityOutput, FinalPacketTiming, std::uint64_t);
  friend FinalOutputPacket invokeFinalOutputPacket(
      IntegrityOutput, FinalPacketTiming, std::uint64_t,
      const FinalPacketClock&, FinalPacketPublicationTransaction*,
      std::string*);
  IntegrityOutput output_;
  FinalPacketTiming timing_;
  FinalPacketMetadataV2 metadata_;
  double state_age_s_ = 0.0;
};

// The deadline endpoint is the actual publication-transaction return sample
// when supplied, then publish-call, then packet-ready as fail-closed fallbacks.
FinalOutputPacket finalizeOutputPacket(IntegrityOutput output,
                                       FinalPacketTiming timing,
                                       std::uint64_t deadline_ns);

std::uint64_t finalOutputPacketDigest(const FinalOutputPacket& packet);

// Fail-closed two-object protocol.
//
// publishCandidate() is the *only* hook in which a transport, mirror, journal,
// allocator or serializer may throw.  It receives an explicitly unprotected,
// non-authoritative candidate. For a non-atomic fan-out this is the immutable,
// conservatively completion-uncertified packet consumed by every ROS/CSV
// mirror; it can never be interpreted as protected or authoritative. Its
// actual invocation and return are sampled by invokeFinalOutputPacket().
//
// After that return the library freezes the terminal deadline/outcome packet.
// commitReceipt() is the single authoritative linearization point.  It must be
// a non-throwing atomic replacement (for example an atomic shared_ptr/store or
// one durable-record rename), and every authoritative consumer reads only that
// receipt.  A transaction may opt in to protected receipt activation only when
// it can provide that atomic property.  ROS/topic fan-out is not atomic and
// therefore must leave this capability false; its compound and legacy topics
// remain explicitly unprotected mirrors.
class FinalPacketPublicationTransaction {
 public:
  virtual ~FinalPacketPublicationTransaction() = default;
  virtual void publishCandidate(const FinalOutputPacket& candidate) = 0;
  virtual bool supportsAtomicProtectedReceipt() const noexcept {
    return false;
  }
  virtual void commitReceipt(const FinalOutputPacket& packet) noexcept = 0;
  virtual void abort() noexcept = 0;
};

FinalOutputPacket invokeFinalOutputPacket(
    IntegrityOutput output, FinalPacketTiming timing,
    std::uint64_t deadline_ns, const FinalPacketClock& clock,
    FinalPacketPublicationTransaction* transaction,
    std::string* publication_error = nullptr);

struct ChildFrameLinearTwist {
  Eigen::Vector3d velocity_body_mps = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance_body_m2ps2 = Eigen::Matrix3d::Zero();
};

struct CommittedStatePublicationV1 {
  std::uint64_t schema_version = 1;
  std::uint64_t transaction_id = 0;
  TimestampNs state_timestamp;
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Constant(
          std::numeric_limits<double>::quiet_NaN());
};

bool recordCommittedStatePublicationV1(
    const CommittedStatePublicationV1& sidecar);
bool committedStatePublicationV1(
    std::uint64_t transaction_id, TimestampNs state_timestamp,
    CommittedStatePublicationV1* sidecar);

ChildFrameLinearTwist childFrameLinearTwist(
    const Eigen::Quaterniond& q_world_body,
    const Eigen::Vector3d& velocity_world_mps,
    const Eigen::Matrix3d& covariance_world_m2ps2);

// Cross-DSO probes used by the golden-header ABI acceptance client.
std::uint64_t p005AbiPublicationDiagnosticsByValue(PublicationDiagnostics value);
std::uint64_t p005AbiIntegrityOutputByValue(IntegrityOutput value);
std::uint64_t p005AbiIntegrityOutputArray(const IntegrityOutput* values,
                                         std::size_t count);
std::uint64_t p005AbiIntegrityOutputVector(std::vector<IntegrityOutput> values);

}  // namespace uwb_imu_pl
