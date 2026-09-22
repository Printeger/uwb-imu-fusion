#pragma once

#include <cstdint>
#include <string>

namespace uwb_imu_pl {

// Frozen identity block for a protected result (roadmap section 4.2, usable
// subset).  Fields that the current schema cannot provide are set to the
// literal NOT_AVAILABLE_IN_SCHEMA instead of being fabricated.
struct IntegritySnapshotIdentity {
  std::string snapshot_id;
  std::string source_revision;
  std::string config_digest;
  std::string manifest_digest;
  std::string state_solution_id;
  std::int64_t sensor_timestamp_ns = 0;
  std::string frame_id;
  std::string position_reference;
  std::string tangent_convention;
  std::string state_scale;
  std::string output_jacobian_contract;
  std::string whitening_id;
  std::string noise_model_id;
  std::string boundary_summary_id;
  std::string history_lineage_id;
  std::string protected_reference_center;
  std::string active_observation_index;
  std::string coverage_epoch;
  std::string validity_assumptions;
  std::string identity_digest;
};

extern const char* const kNotAvailableInSchema;

// FNV-1a 64 with the repository's config-hash variant (offset basis
// 1469598103934665603, prime 1099511628211), rendered as 16 hex digits.
std::string identityDigest(IntegritySnapshotIdentity* identity);

// The same FNV-1a 64 variant as a plain 64-bit value.  C4 (W2) uses it to bind
// the string-valued production identities (snapshot id, manifest digest,
// detector ids, health snapshot, ...) into the numeric PublicationIdentity
// fields: the mapping is a documented stable function of the real value, never
// a fabricated substitution.
std::uint64_t identityHash64(const std::string& text);

std::string serializeIdentity(const IntegritySnapshotIdentity& identity);

}  // namespace uwb_imu_pl
