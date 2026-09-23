#pragma once

#include <string>
#include <vector>

namespace uwb_imu_pl {

enum class FdeProfile {
  Off,
  UwbOrder1,
  ImuOrder1,
  JointOrder1,
  JointOrder2,
};

enum class SingleFaultFamily { UwbAnchor, ImuAccelAxis, ImuGyroAxis };
enum class PairFaultFamily { UwbImu };

struct FaultScopeCapabilities {
  bool uwb_anchor = true;
  bool imu_accel_axis = true;
  bool imu_gyro_axis = true;
  bool uwb_imu_pair = true;
  std::string manifest_digest;
};

struct ResolvedFaultScope {
  FdeProfile profile = FdeProfile::JointOrder1;
  // Preserve the pre-v6 programmatic API: tests and embedders that construct
  // IntegrityConfig directly historically received the full first-order
  // monitor.  YAML-loaded configurations always replace this value with the
  // canonical manifest-bound scope produced by resolveFaultScope().
  std::vector<SingleFaultFamily> singles = {
      SingleFaultFamily::UwbAnchor,
      SingleFaultFamily::ImuAccelAxis,
      SingleFaultFamily::ImuGyroAxis};
  std::vector<PairFaultFamily> pairs;
  unsigned max_fault_order = 1;
  std::string detector_contract_id = "dual_channel_v1";
  std::string manifest_digest;
  std::string scope_digest = "programmatic-joint-order1";
  std::vector<std::string> omitted_families = {"uwb_imu"};
  bool allow_uwb_actions = true;
  bool allow_imu_actions = true;

  bool requiresUwbFaults() const;
  bool requiresImuFaults() const;
  bool enabled() const { return profile != FdeProfile::Off; }
};

const char* toString(FdeProfile profile);
const char* toString(SingleFaultFamily family);
const char* toString(PairFaultFamily family);
FdeProfile parseFdeProfile(const std::string& value);
ResolvedFaultScope resolveFaultScope(
    FdeProfile profile, const FaultScopeCapabilities& capabilities);

}  // namespace uwb_imu_pl
