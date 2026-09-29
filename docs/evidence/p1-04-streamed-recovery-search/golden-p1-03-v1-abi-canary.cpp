#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <cstdint>
#include <iostream>
#include <memory>

using namespace uwb_imu_pl;

namespace {

IntegrityConfig config() {
  IntegrityConfig value;
  value.incremental.relinearize_threshold = 0.1;
  value.incremental.relinearize_skip = 1;
  value.incremental.smoothness_sigma_m = 0.5;
  value.incremental.method_b_max_condition = 1e12;
  value.snapshot.rank_tolerance = 1e-10;
  value.snapshot.max_condition_number = 1e12;
  value.imu.accelerometer_sigma = 0.1;
  value.imu.gyroscope_sigma = 0.01;
  value.imu.accelerometer_bias_rw_sigma = 0.001;
  value.imu.gyroscope_bias_rw_sigma = 0.0001;
  value.imu.gravity_mps2 = 9.80665;
  value.imu.max_gap_s = 0.02;
  value.risk.p_fa = 1e-5;
  value.risk.p_hmi_total = 4e-5;
  value.risk.nominal_axis_tail = 1e-5;
  value.risk.p_nm = 1e-7;
  value.risk.horizontal_alert_limit_m = 100.0;
  value.risk.vertical_alert_limit_m = 100.0;
  for (std::uint64_t id = 1; id <= 5; ++id) {
    FaultHypothesis allocation;
    allocation.id = HypothesisId(id);
    allocation.anchor_id = AnchorId(id);
    allocation.missed_detection_allocation = 1e-3;
    allocation.prior_probability_bound = 1e-4;
    value.risk.hypotheses.push_back(allocation);
  }
  return value;
}

UwbBatch batch(TimestampNs timestamp) {
  UwbBatch value;
  value.id = BatchId(static_cast<std::uint64_t>(timestamp.value()));
  value.timestamp = timestamp;
  value.covariance_model_id = "p104_golden_v1_abi";
  const Eigen::Vector3d position(0.0, 0.0, 1.0);
  const Eigen::Vector3d anchors[] = {
      {-5, -5, 0}, {5, -5, 1}, {5, 5, 3}, {-5, 5, 4}, {0, -6, 2}};
  for (std::size_t index = 0; index < 5; ++index) {
    UwbMeasurement measurement;
    measurement.id = MeasurementId(index + 1);
    measurement.factor_id = FactorId(index + 1);
    measurement.anchor_id = AnchorId(index + 1);
    measurement.timestamp = timestamp;
    measurement.anchor_position_m = anchors[index];
    measurement.range_m = (position - anchors[index]).norm();
    measurement.sigma_m = 0.1;
    value.measurements.push_back(measurement);
  }
  return value;
}

}  // namespace

int main() {
  static_assert(sizeof(PipelineTestDependencySeamsV1) == 128,
                "golden V1 seam layout changed");
  static_assert(alignof(PipelineTestDependencySeamsV1) == 8,
                "golden V1 seam alignment changed");
  PipelineTestDependencySeamsV1 layout;
  const auto* layout_base = reinterpret_cast<const unsigned char*>(&layout);
  const std::size_t offsets[] = {
      static_cast<std::size_t>(reinterpret_cast<const unsigned char*>(
          &layout.after_action_generation_before_census) - layout_base),
      static_cast<std::size_t>(reinterpret_cast<const unsigned char*>(
          &layout.before_candidate) - layout_base),
      static_cast<std::size_t>(reinterpret_cast<const unsigned char*>(
          &layout.before_post_detector) - layout_base),
      static_cast<std::size_t>(reinterpret_cast<const unsigned char*>(
          &layout.before_protection_level) - layout_base)};
  const bool layout_exact = offsets[0] == 0 && offsets[1] == 32 &&
      offsets[2] == 64 && offsets[3] == 96;

  IntegrityConfig cfg = config();
  IncrementalUwbImuEstimator estimator(cfg, Eigen::Vector3d::Zero());
  NavigationState initial;
  initial.timestamp = TimestampNs(0);
  initial.position_world_m = {0.0, 0.0, 1.0};
  estimator.initialize(initial, Eigen::Matrix<double, 15, 1>::Constant(0.1));
  RealtimeIntegrityPipeline pipeline(
      &estimator,
      IntegrityMonitor(cfg.risk, cfg.snapshot.rank_tolerance,
                       cfg.snapshot.max_condition_number));
  for (int sample = 0; sample <= 2; ++sample) {
    ImuMeasurement imu;
    imu.id = MeasurementId(100 + sample);
    imu.timestamp = TimestampNs(sample * 5000000LL);
    imu.specific_force_mps2 = {0.0, 0.0, cfg.imu.gravity_mps2};
    pipeline.ingestImu(imu);
  }

  std::uint64_t callbacks = 0;
  auto seams = std::make_shared<PipelineTestDependencySeamsV1>();
  seams->after_action_generation_before_census =
      [&](std::vector<ExclusionAction>*) { ++callbacks; };
  seams->before_candidate = [&](ExclusionActionId) { ++callbacks; };
  seams->before_post_detector = [&](ExclusionActionId) { ++callbacks; };
  seams->before_protection_level = [&](ExclusionActionId) { ++callbacks; };
  pipeline.setTestDependencySeamsV1(seams);
  const IntegrityOutput output =
      pipeline.processUwbBatch(batch(TimestampNs(10000000)));
  pipeline.setTestDependencySeamsV1(nullptr);
  seams.reset();

  const bool no_winner = output.selected_action_id == 0;
  const bool exercised = callbacks != 0;
  std::cout << "PipelineTestDependencySeamsV1 "
            << sizeof(PipelineTestDependencySeamsV1) << ' '
            << alignof(PipelineTestDependencySeamsV1) << ' '
            << offsets[0] << ' ' << offsets[1] << ' ' << offsets[2] << ' '
            << offsets[3] << '\n';
  std::cout << "old-v1-lifecycle " << exercised << ' ' << no_winner << ' '
            << callbacks << ' ' << output.fde_status << '\n';
  return layout_exact && exercised && no_winner ? 0 : 1;
}
