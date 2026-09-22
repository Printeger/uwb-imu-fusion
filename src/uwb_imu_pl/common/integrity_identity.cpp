#include "uwb_imu_pl/common/integrity_identity.hpp"

#include <iomanip>
#include <sstream>

namespace uwb_imu_pl {

const char* const kNotAvailableInSchema = "NOT_AVAILABLE_IN_SCHEMA";

std::uint64_t identityHash64(const std::string& text) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : text) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string identityDigest(IntegritySnapshotIdentity* identity) {
  if (!identity) return {};
  std::ostringstream text;
  text << identity->snapshot_id << '\x1f' << identity->source_revision << '\x1f'
       << identity->config_digest << '\x1f' << identity->manifest_digest << '\x1f'
       << identity->state_solution_id << '\x1f' << identity->sensor_timestamp_ns
       << '\x1f' << identity->frame_id << '\x1f' << identity->position_reference
       << '\x1f' << identity->tangent_convention << '\x1f' << identity->state_scale
       << '\x1f' << identity->output_jacobian_contract << '\x1f'
       << identity->whitening_id << '\x1f' << identity->noise_model_id << '\x1f'
       << identity->boundary_summary_id << '\x1f' << identity->history_lineage_id
       << '\x1f' << identity->protected_reference_center << '\x1f'
       << identity->active_observation_index << '\x1f' << identity->coverage_epoch
       << '\x1f' << identity->validity_assumptions;
  const std::string payload = text.str();
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : payload) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << hash;
  identity->identity_digest = out.str();
  return identity->identity_digest;
}

std::string serializeIdentity(const IntegritySnapshotIdentity& identity) {
  std::ostringstream out;
  out << "snapshot_id=" << identity.snapshot_id
      << ";source_revision=" << identity.source_revision
      << ";config_digest=" << identity.config_digest
      << ";manifest_digest=" << identity.manifest_digest
      << ";state_solution_id=" << identity.state_solution_id
      << ";sensor_timestamp_ns=" << identity.sensor_timestamp_ns
      << ";frame_id=" << identity.frame_id
      << ";position_reference=" << identity.position_reference
      << ";tangent_convention=" << identity.tangent_convention
      << ";state_scale=" << identity.state_scale
      << ";output_jacobian_contract=" << identity.output_jacobian_contract
      << ";whitening_id=" << identity.whitening_id
      << ";noise_model_id=" << identity.noise_model_id
      << ";boundary_summary_id=" << identity.boundary_summary_id
      << ";history_lineage_id=" << identity.history_lineage_id
      << ";protected_reference_center=" << identity.protected_reference_center
      << ";active_observation_index=" << identity.active_observation_index
      << ";coverage_epoch=" << identity.coverage_epoch
      << ";validity_assumptions=" << identity.validity_assumptions
      << ";identity_digest=" << identity.identity_digest;
  return out.str();
}

}  // namespace uwb_imu_pl
