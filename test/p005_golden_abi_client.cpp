#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"

#include <cstddef>
#include <iostream>
#include <vector>

namespace uwb_imu_pl {
std::uint64_t p005AbiPublicationDiagnosticsByValue(PublicationDiagnostics value);
std::uint64_t p005AbiIntegrityOutputByValue(IntegrityOutput value);
std::uint64_t p005AbiIntegrityOutputArray(const IntegrityOutput* values,
                                         std::size_t count);
std::uint64_t p005AbiIntegrityOutputVector(std::vector<IntegrityOutput> values);
}

int main() {
  using namespace uwb_imu_pl;
  std::cout << "IntegrityOutput " << sizeof(IntegrityOutput) << ' '
            << offsetof(IntegrityOutput, bridge_audit) << '\n';
  std::cout << "PublicationDiagnostics " << sizeof(PublicationDiagnostics)
            << ' ' << offsetof(PublicationDiagnostics, certificate_id) << '\n';
  std::cout << "EpochCommitPlan " << sizeof(EpochCommitPlan) << ' '
            << offsetof(EpochCommitPlan, recovery_epoch_end) << '\n';
  std::cout << "CommitReceipt " << sizeof(CommitReceipt) << ' '
            << offsetof(CommitReceipt, integrity_available) << '\n';
  std::cout << "ProtectionLevelPublicationPacketV1 "
            << sizeof(ProtectionLevelPublicationPacketV1) << ' '
            << offsetof(ProtectionLevelPublicationPacketV1, packet_identity)
            << '\n';
  if (sizeof(ProtectionLevelPublicationPacketV1) != 88 ||
      offsetof(ProtectionLevelPublicationPacketV1, packet_identity) != 80) {
    return 3;
  }
  if (sizeof(PublicationDiagnostics) != 424 ||
      offsetof(PublicationDiagnostics, certificate_id) != 416 ||
      sizeof(IntegrityOutput) != 4128 ||
      offsetof(IntegrityOutput, bridge_audit) != 3864) return 5;
  PublicationDiagnostics diagnostics;
  diagnostics.certificate_id = 0x1234;
  if (p005AbiPublicationDiagnosticsByValue(diagnostics) != 0x1234) return 6;
  IntegrityOutput by_value;
  by_value.transaction_id = 0x55;
  by_value.publication.certificate_id = 0x22;
  if (p005AbiIntegrityOutputByValue(by_value) != (0x55 ^ (0x22 << 1U)))
    return 7;
  IntegrityOutput values[2];
  values[0].transaction_id = 3;
  values[0].publication.certificate_id = 5;
  values[1].transaction_id = 7;
  values[1].publication.certificate_id = 11;
  const std::uint64_t expected = 2 ^ (3 << 1U) ^ (5 << 3U) ^
      (7 << 2U) ^ (11 << 4U);
  if (p005AbiIntegrityOutputArray(values, 2) != expected) return 8;
  std::vector<IntegrityOutput> vector(values, values + 2);
  if (p005AbiIntegrityOutputVector(vector) != expected) return 9;
  ProtectionLevelPublicationPacketV1 packet;
  if (protectionLevelPublicationPacket("missing-golden-probe", &packet)) {
    return 4;
  }

  IntegrityConfig config;
  config.incremental.relinearize_threshold = 0.1;
  config.incremental.relinearize_skip = 1;
  config.snapshot.rank_tolerance = 1e-10;
  config.snapshot.max_condition_number = 1e12;
  config.imu.accelerometer_sigma = 0.1;
  config.imu.gyroscope_sigma = 0.01;
  config.imu.accelerometer_bias_rw_sigma = 0.001;
  config.imu.gyroscope_bias_rw_sigma = 0.0001;
  config.imu.gravity_mps2 = 9.80665;
  config.realtime.world_frame = "map";
  config.realtime.body_frame = "base_link";
  IncrementalUwbImuEstimator estimator(config, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0, 0, 1};
  estimator.initialize(initial, config.realtime.prior_sigmas);
  for (int index = 0; index <= 2; ++index) {
    ImuMeasurement imu;
    imu.id = MeasurementId(index + 1);
    imu.timestamp = TimestampNs(index * 5000000);
    imu.specific_force_mps2 = {0, 0, config.imu.gravity_mps2};
    estimator.ingestImu(imu);
  }
  UwbBatch batch;
  batch.id = BatchId(1);
  batch.timestamp = TimestampNs(10000000);
  batch.covariance_model_id = "p005_abi";
  const Eigen::Vector3d anchors[] = {
      {-5, -5, 0}, {5, -5, 1}, {5, 5, 3}, {-5, 5, 4}, {0, -6, 2}};
  for (int index = 0; index < 5; ++index) {
    UwbMeasurement measurement;
    measurement.id = MeasurementId(index + 1);
    measurement.factor_id = FactorId(index + 1);
    measurement.anchor_id = AnchorId(index + 1);
    measurement.timestamp = batch.timestamp;
    measurement.anchor_position_m = anchors[index];
    measurement.range_m = (initial.position_world_m - anchors[index]).norm();
    measurement.sigma_m = 0.1;
    batch.measurements.push_back(measurement);
  }
  auto transaction = estimator.prepareEpoch(batch);
  const auto plan = EpochCommitPlan::nominalPlan(transaction);
  const auto receipt = estimator.commitEpoch(std::move(transaction), plan);
  if (receipt.backend_updates != 1 || receipt.integrity_available ||
      estimator.currentState().timestamp != batch.timestamp) {
    return 2;
  }
  IntegrityOutput output;
  output.transaction_id = receipt.transaction_id.value();
  output.state = estimator.currentState();
  std::cout << "old-client-run PASS " << output.transaction_id << '\n';
  return 0;
}
