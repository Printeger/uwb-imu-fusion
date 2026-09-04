#pragma once

#include "uwb_imu_pl/estimation/epoch_transaction.hpp"
#include "uwb_imu_pl/integrity/fault_model.hpp"

namespace uwb_imu_pl {

struct GenericBridgeSpec {
  bool enabled = true;
  std::string model = "constant_velocity_constant_attitude";
  double optimization_sigma_position_m = 0.5;
  double optimization_sigma_velocity_mps = 1.0;
  double optimization_sigma_rotation_rad = 0.5;
  double optimization_sigma_bias_accel = 0.01;
  double optimization_sigma_bias_gyro = 0.001;
  double acceleration_bound_mps2 = 4.0;
  double angular_rate_bound_radps = 2.0;
  double angular_acceleration_bound_radps2 = 4.0;
  std::string integrity_model = "deterministic_box";
  std::string calibration_id;
};

class BridgeFactory {
 public:
  PendingFactorGroup makeGeneric(const EpochTransaction& transaction,
                                 const GenericBridgeSpec& config) const;
  BridgeUncertainty uncertainty(const EpochTransaction& transaction,
                                const GenericBridgeSpec& config) const;
  Eigen::Vector3d propagateBoxMargin(
      const Eigen::Matrix<double, 3, Eigen::Dynamic>& protected_map,
      const Eigen::MatrixXd& bridge_error_gain,
      const Eigen::VectorXd& box_bound) const;
};

}  // namespace uwb_imu_pl
