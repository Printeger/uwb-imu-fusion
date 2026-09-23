#include "uwb_imu_pl/integrity/fault_scope.hpp"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

std::string digest(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << hash;
  return out.str();
}

}  // namespace

const char* toString(FdeProfile profile) {
  switch (profile) {
    case FdeProfile::Off: return "off";
    case FdeProfile::UwbOrder1: return "uwb_order1";
    case FdeProfile::ImuOrder1: return "imu_order1";
    case FdeProfile::JointOrder1: return "joint_order1";
    case FdeProfile::JointOrder2: return "joint_order2";
  }
  return "unknown";
}

const char* toString(SingleFaultFamily family) {
  switch (family) {
    case SingleFaultFamily::UwbAnchor: return "uwb_anchor";
    case SingleFaultFamily::ImuAccelAxis: return "imu_accel_axis";
    case SingleFaultFamily::ImuGyroAxis: return "imu_gyro_axis";
  }
  return "unknown";
}

const char* toString(PairFaultFamily family) {
  switch (family) {
    case PairFaultFamily::UwbImu: return "uwb_imu";
  }
  return "unknown";
}

FdeProfile parseFdeProfile(const std::string& value) {
  if (value == "off") return FdeProfile::Off;
  if (value == "uwb_order1") return FdeProfile::UwbOrder1;
  if (value == "imu_order1") return FdeProfile::ImuOrder1;
  if (value == "joint_order1") return FdeProfile::JointOrder1;
  if (value == "joint_order2") return FdeProfile::JointOrder2;
  throw std::runtime_error(
      "fde.profile must be off, uwb_order1, imu_order1, joint_order1 or "
      "joint_order2");
}

bool ResolvedFaultScope::requiresUwbFaults() const {
  return std::find(singles.begin(), singles.end(),
                   SingleFaultFamily::UwbAnchor) != singles.end();
}

bool ResolvedFaultScope::requiresImuFaults() const {
  return std::find(singles.begin(), singles.end(),
                   SingleFaultFamily::ImuAccelAxis) != singles.end() ||
         std::find(singles.begin(), singles.end(),
                   SingleFaultFamily::ImuGyroAxis) != singles.end();
}

ResolvedFaultScope resolveFaultScope(
    FdeProfile profile, const FaultScopeCapabilities& capabilities) {
  ResolvedFaultScope scope;
  scope.singles.clear();
  scope.pairs.clear();
  scope.omitted_families.clear();
  scope.scope_digest.clear();
  scope.allow_uwb_actions = false;
  scope.allow_imu_actions = false;
  scope.profile = profile;
  scope.manifest_digest = capabilities.manifest_digest;
  scope.max_fault_order = profile == FdeProfile::Off
                              ? 0u
                              : (profile == FdeProfile::JointOrder2 ? 2u : 1u);

  const bool uwb = profile == FdeProfile::UwbOrder1 ||
                   profile == FdeProfile::JointOrder1 ||
                   profile == FdeProfile::JointOrder2;
  const bool imu = profile == FdeProfile::ImuOrder1 ||
                   profile == FdeProfile::JointOrder1 ||
                   profile == FdeProfile::JointOrder2;
  if (uwb) {
    if (!capabilities.uwb_anchor) {
      throw std::runtime_error(
          "fde.profile requires unsupported manifest family uwb_anchor");
    }
    scope.singles.push_back(SingleFaultFamily::UwbAnchor);
    scope.allow_uwb_actions = true;
  } else {
    scope.omitted_families.push_back("uwb_anchor");
  }
  if (imu) {
    if (!capabilities.imu_accel_axis || !capabilities.imu_gyro_axis) {
      throw std::runtime_error(
          "fde.profile requires unsupported manifest IMU single-axis family");
    }
    scope.singles.push_back(SingleFaultFamily::ImuAccelAxis);
    scope.singles.push_back(SingleFaultFamily::ImuGyroAxis);
    scope.allow_imu_actions = true;
  } else {
    scope.omitted_families.push_back("imu_accel_axis");
    scope.omitted_families.push_back("imu_gyro_axis");
  }
  if (profile == FdeProfile::JointOrder2) {
    if (!capabilities.uwb_imu_pair) {
      throw std::runtime_error(
          "fde.profile joint_order2 requires unsupported manifest pair uwb_imu");
    }
    scope.pairs.push_back(PairFaultFamily::UwbImu);
  } else {
    scope.omitted_families.push_back("uwb_imu");
  }

  std::ostringstream identity;
  identity << "fault_scope/v1|profile=" << toString(profile)
           << "|detector=" << scope.detector_contract_id
           << "|manifest=" << scope.manifest_digest << "|singles=";
  for (const auto family : scope.singles) identity << toString(family) << ',';
  identity << "|pairs=";
  for (const auto family : scope.pairs) identity << toString(family) << ',';
  identity << "|actions=" << scope.allow_uwb_actions << ':'
           << scope.allow_imu_actions << "|omitted=";
  for (const auto& family : scope.omitted_families) identity << family << ',';
  scope.scope_digest = digest(identity.str());
  return scope;
}

}  // namespace uwb_imu_pl
