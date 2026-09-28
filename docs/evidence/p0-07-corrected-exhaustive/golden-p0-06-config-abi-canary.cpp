#include "uwb_imu_pl/config/integrity_config.hpp"

#include <cstddef>
#include <iostream>
#include <string>

using namespace uwb_imu_pl;

// Compile this client against headers extracted from golden-p0-06-actions,
// then link/run it against the current DSO.  The loader returns the complete
// aggregate by value, so this exercises the ABI boundary that the first
// P0-07 review found broken as well as construction and destruction.
int main(int argc, char** argv) {
  if (argc != 2) return 2;
  static_assert(sizeof(ImuNoiseConfig) == 80,
                "golden ImuNoiseConfig layout changed");
  static_assert(alignof(ImuNoiseConfig) == 8,
                "golden ImuNoiseConfig alignment changed");
  static_assert(offsetof(ImuNoiseConfig, accelerometer_sigma) == 0);
  static_assert(offsetof(ImuNoiseConfig, gyroscope_sigma) == 8);
  static_assert(offsetof(ImuNoiseConfig, accelerometer_bias_rw_sigma) == 16);
  static_assert(offsetof(ImuNoiseConfig, gyroscope_bias_rw_sigma) == 24);
  static_assert(offsetof(ImuNoiseConfig, gravity_mps2) == 32);
  static_assert(offsetof(ImuNoiseConfig, max_gap_s) == 40);
  static_assert(offsetof(ImuNoiseConfig, noise_overbound_calibration_id) == 48);
  static_assert(sizeof(IntegrityConfig) == 2544,
                "golden IntegrityConfig layout changed");
  static_assert(alignof(IntegrityConfig) == 8,
                "golden IntegrityConfig alignment changed");
  IntegrityConfig config = IntegrityConfigLoader::load(std::string(argv[1]));
  const bool callable = config.imu.accelerometer_sigma > 0.0 &&
      !config.anchors.empty() && !config.resolved_yaml.empty();
  std::cout << "ImuNoiseConfig " << sizeof(ImuNoiseConfig) << ' '
            << alignof(ImuNoiseConfig) << '\n';
  std::cout << "IntegrityConfig " << sizeof(IntegrityConfig) << ' '
            << alignof(IntegrityConfig) << '\n';
  std::cout << "ImuNoiseConfig offsets 0 8 16 24 32 40 48\n";
  std::cout << "construct-load-by-value-destruct " << callable << '\n';
  return callable ? 0 : 1;
}
