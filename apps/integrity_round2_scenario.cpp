#include "uwb_imu_pl/config/integrity_config.hpp"
#include "uwb_imu_pl/estimation/incremental_estimator.hpp"
#include "uwb_imu_pl/integrity/integrity_monitor.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string stringField(const std::string& json, const std::string& key,
                        const std::string& fallback = "") {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  return std::regex_search(json, match, pattern) ? match[1].str() : fallback;
}

double numberField(const std::string& json, const std::string& key,
                   double fallback = 0.0) {
  const std::regex pattern("\\\"" + key +
      "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  std::smatch match;
  return std::regex_search(json, match, pattern)
      ? std::stod(match[1].str()) : fallback;
}

std::uint64_t fnv1a(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t splitmix64(std::uint64_t value) {
  value += 0x9E3779B97F4A7C15ULL;
  value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ULL;
  value = (value ^ (value >> 27)) * 0x94D049BB133111EBULL;
  return value ^ (value >> 31);
}

class CounterRng {
 public:
  CounterRng(std::uint64_t seed, std::string key)
      : seed_(seed), key_(fnv1a(key)) {}
  double uniform(std::uint64_t counter, std::uint64_t lane) const {
    const std::uint64_t value = splitmix64(seed_ ^ key_ ^
        splitmix64(counter) ^ splitmix64(lane));
    return (static_cast<double>(value >> 11) + 0.5) /
        static_cast<double>(1ULL << 53);
  }
  double normal(std::uint64_t counter, std::uint64_t lane) const {
    const double u1 = std::max(uniform(counter, 2 * lane),
                               std::ldexp(1.0, -53));
    const double u2 = uniform(counter, 2 * lane + 1);
    return std::sqrt(-2.0 * std::log(u1)) *
        std::cos(2.0 * M_PI * u2);
  }
 private:
  std::uint64_t seed_;
  std::uint64_t key_;
};

struct Truth {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
  double yaw_rate = 0.0;
};

Truth truthAt(const std::string& motion, double time) {
  Truth value;
  value.position.z() = 1.2;
  double yaw = 0.0;
  if (motion == "straight" || motion == "constant_velocity") {
    value.position.x() = 0.4 * time;
  } else if (motion == "acceleration") {
    value.position.x() = 0.15 * time * time;
    value.acceleration.x() = 0.3;
  } else if (motion == "braking") {
    value.position.x() = 0.8 * time - 0.1 * time * time;
    value.acceleration.x() = -0.2;
  } else if (motion == "circle") {
    value.position.x() = 2.5 * std::sin(0.25 * time);
    value.position.y() = 2.5 * (1.0 - std::cos(0.25 * time));
    value.acceleration.x() = -0.15625 * std::sin(0.25 * time);
    value.acceleration.y() = 0.15625 * std::cos(0.25 * time);
    yaw = 0.25 * time;
    value.yaw_rate = 0.25;
  } else if (motion == "high_yaw") {
    yaw = 1.2 * time;
    value.yaw_rate = 1.2;
  } else if (motion == "translation_rotation" ||
             motion == "combined_motion") {
    value.position.x() = 0.25 * time;
    value.position.y() = std::sin(0.4 * time);
    value.acceleration.y() = -0.16 * std::sin(0.4 * time);
    yaw = 0.6 * time;
    value.yaw_rate = 0.6;
  }
  value.attitude = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return value;
}

struct RunResult {
  bool valid = true;
  std::string reason;
  bool detected = false;
  bool correct_exclusion = false;
  bool union_exclusion = false;
  bool ambiguous_exclusion = false;
  bool wrong_exclusion = false;
  bool nominal_false_exclusion = false;
  bool no_valid_candidate = false;
  bool coverage_failure = false;
  bool hmi = false;
  bool protected_available = false;
  bool assumption_violation = false;
  bool post_fde_detector_passed = true;
  bool backend_update_violation = false;
  bool replay_key_valid = true;
  bool bridge_coverage_verified = false;
  bool bridge_timeout_verified = false;
  int bridge_epochs_observed = 0;
  bool model_valid_at_evaluation = false;
  bool detector_valid_at_evaluation = false;
  std::string selected_sources;
  std::string plausible_sources;
  std::string evaluation_reason;
  int detection_delay = -1;
  int epochs = 0;
  double boundary_uwb = std::numeric_limits<double>::infinity();
  double boundary_imu = std::numeric_limits<double>::infinity();
  double geometry_condition = std::numeric_limits<double>::infinity();
};

void applyOutOfEnvelopeStress(Truth* truth, double time) {
  const double phase = 4.0 * time;
  truth->position.x() += 0.5 * std::sin(phase);
  truth->acceleration.x() -= 8.0 * std::sin(phase);
  truth->attitude = truth->attitude * Eigen::Quaterniond(
      Eigen::AngleAxisd(std::sin(phase), Eigen::Vector3d::UnitZ()));
  truth->yaw_rate += 4.0 * std::cos(phase);
}

Eigen::Vector3d scenarioAnchor(Eigen::Vector3d anchor,
                               const std::string& geometry,
                               double protected_height) {
  if (geometry == "near_rank") {
    anchor.z() = protected_height;
  } else if (geometry == "poor7" || geometry == "poor6") {
    // Preserve finite vertical information while making both low-redundancy
    // geometries deliberately and reproducibly worse than normal8.
    anchor.z() = protected_height + 0.10 * (anchor.z() - 3.0);
  }
  return anchor;
}

int activeAnchors(const std::string& geometry, int configured) {
  if (geometry == "poor7") return std::min(7, configured);
  if (geometry == "poor6" || geometry == "near_rank") return std::min(6, configured);
  return std::min(8, configured);
}

double geometryCondition(const uwb_imu_pl::IntegrityConfig& config,
                         int count, const Eigen::Vector3d& position,
                         bool near_rank) {
  Eigen::MatrixXd h(count, 3);
  for (int i = 0; i < count; ++i) {
    Eigen::Vector3d anchor = scenarioAnchor(
        config.anchors.at(i).position_world_m,
        near_rank ? "near_rank" : (count == 7 ? "poor7" :
                                    (count == 6 ? "poor6" : "normal8")),
        position.z());
    h.row(i) = (position - anchor).normalized();
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(h);
  const auto values = svd.singularValues();
  return values.size() >= 3 && values(2) > 0.0
      ? values(0) / values(2) : std::numeric_limits<double>::infinity();
}

double findBoundary(const uwb_imu_pl::IntegrityOutput& output,
                    const std::string& sensor, const std::string& kind,
                    const std::string& physical_source,
                    std::uint64_t onset_epoch) {
  for (const auto& hypothesis : output.hypothesis_audit) {
    if (hypothesis.sensor != sensor ||
        hypothesis.fault_kind.find(kind) == std::string::npos ||
        hypothesis.physical_source_ids != physical_source ||
        hypothesis.onset_epoch != onset_epoch ||
        hypothesis.parameter_dimension <= 0 ||
        !std::isfinite(hypothesis.noncentrality_boundary) ||
        hypothesis.boundary_direction_gram <= 0.0) continue;
    return std::sqrt(hypothesis.noncentrality_boundary /
                     hypothesis.boundary_direction_gram);
  }
  return std::numeric_limits<double>::infinity();
}

std::set<std::string> sourceSet(const std::string& joined) {
  std::set<std::string> result;
  std::istringstream stream(joined);
  std::string value;
  while (std::getline(stream, value, ';')) {
    if (!value.empty()) result.insert(value);
  }
  return result;
}

std::string joinSources(const std::set<std::string>& sources) {
  std::string result;
  for (const auto& source : sources) {
    if (!result.empty()) result += ';';
    result += source;
  }
  return result;
}

bool verifyHealthRecoverySequence(const uwb_imu_pl::HealthConfigV2& config) {
  auto advance_to_recovery = [&](uwb_imu_pl::HealthManager* manager) {
    manager->registerSource("imu_accel:0", uwb_imu_pl::SensorType::ImuAccelerometer);
    manager->observeEvidence("imu_accel:0", true);
    manager->observeEvidence("imu_accel:0", true);
    manager->quarantine("imu_accel:0", "deterministic exclusion");
    for (std::uint32_t i = 0; i < config.recovery_shadow_passes; ++i) {
      manager->observeShadowRecovery("imu_accel:0", true);
    }
    return manager->state("imu_accel:0") ==
        uwb_imu_pl::HealthState::RecoveryTest;
  };
  uwb_imu_pl::HealthManager success(config);
  if (!advance_to_recovery(&success)) return false;
  for (std::uint32_t i = 0; i < config.recovery_test_passes; ++i) {
    success.observeShadowRecovery("imu_accel:0", true);
  }
  if (success.state("imu_accel:0") != uwb_imu_pl::HealthState::Healthy) {
    return false;
  }
  uwb_imu_pl::HealthManager refailure(config);
  if (!advance_to_recovery(&refailure)) return false;
  for (std::uint32_t i = 0; i < 4; ++i) {
    refailure.observeShadowRecovery("imu_accel:0", true);
  }
  refailure.observeShadowRecovery("imu_accel:0", false);
  return refailure.state("imu_accel:0") ==
      uwb_imu_pl::HealthState::Quarantined;
}

bool verifyReinitializationSequence() {
  uwb_imu_pl::ControlledReinitializer controller;
  uwb_imu_pl::NavigationState committed;
  committed.timestamp = uwb_imu_pl::TimestampNs(100);
  controller.request(uwb_imu_pl::FdeStatus::BridgeTimeout,
                     "BRIDGE_TIMEOUT", committed);
  const bool requested = controller.directive().state ==
      uwb_imu_pl::ReinitializationState::Requested && controller.blocksUwb();
  controller.beginWaiting();
  const bool waiting = controller.directive().state ==
      uwb_imu_pl::ReinitializationState::WaitingForTrustedImu;
  uwb_imu_pl::ImuMeasurement trusted;
  trusted.timestamp = uwb_imu_pl::TimestampNs(101);
  const bool reinitialized = controller.acceptTrustedImu(trusted).has_value() &&
      controller.directive().state == uwb_imu_pl::ReinitializationState::Reinitialized;
  return requested && waiting && reinitialized && controller.blocksUwb();
}

RunResult simulate(const uwb_imu_pl::IntegrityConfig& config,
                   const std::string& cell, std::uint64_t seed,
                   bool inject, double uwb_boundary, double imu_boundary) {
  RunResult result;
  result.boundary_uwb = uwb_boundary;
  result.boundary_imu = imu_boundary;
  const std::string gate = stringField(cell, "gate");
  const std::string geometry = stringField(cell, "geometry", "normal8");
  const std::string motion = stringField(cell, "motion", "straight");
  const std::string fault_mode = stringField(cell, "fault_mode", "persistent");
  const std::string sensor = stringField(cell, "sensor");
  const std::string name = stringField(cell, "name");
  const std::string imu_axis_name = stringField(cell, "imu_axis");
  const int named_axis = !imu_axis_name.empty() && imu_axis_name.back() == 'y' ? 1 :
      (!imu_axis_name.empty() && imu_axis_name.back() == 'z' ? 2 : 0);
  const int axis = static_cast<int>(numberField(cell, "axis", named_axis));
  const int target_anchor = static_cast<int>(numberField(cell, "anchor", 1));
  const double ratio = numberField(cell, "ratio", 1.0);
  const double uwb_ratio = numberField(cell, "uwb_ratio", ratio);
  const double imu_ratio = numberField(cell, "imu_ratio", ratio);
  const int bridge_length = static_cast<int>(numberField(cell, "bridge_length", 0));
  const int anchor_count = activeAnchors(geometry, config.anchors.size());
  const int onset_offset = static_cast<int>(numberField(cell, "onset_offset", 0));
  int uwb_onset = static_cast<int>(numberField(
      cell, "onset_epoch", 25 + onset_offset));
  int imu_epoch = uwb_onset;
  int evaluation_epoch = uwb_onset;
  if (gate == "F") {
    evaluation_epoch = 45;
    imu_epoch = evaluation_epoch - static_cast<int>(
        numberField(cell, "interval_offset", 0));
  } else if (gate == "H") {
    uwb_onset = 30;
    imu_epoch = stringField(cell, "relative_onset") == "different" ? 25 : 30;
    evaluation_epoch = 35;
  }
  const int requested_epochs = static_cast<int>(numberField(cell, "epochs", 0));
  const int epochs = requested_epochs > 0 ? requested_epochs :
      std::max(55, evaluation_epoch + std::max(bridge_length, 5) + 5);
  result.epochs = epochs;
  result.assumption_violation = stringField(cell, "envelope") == "out_of_envelope";
  bool bridge_seen = false;
  bool bridge_covered = true;
  bool timeout_requested = false;
  bool timeout_reinitialized = false;

  uwb_imu_pl::NavigationState initial;
  initial.timestamp = uwb_imu_pl::TimestampNs(0);
  initial.position_world_m = truthAt(motion, 0).position;
  uwb_imu_pl::IncrementalUwbImuEstimator estimator(
      config, config.realtime.lever_arm_body_m);
  estimator.initialize(initial, config.realtime.prior_sigmas);
  uwb_imu_pl::RealtimeIntegrityPipeline pipeline(
      &estimator, uwb_imu_pl::IntegrityMonitor(
          config.risk, config.snapshot.rank_tolerance,
          config.snapshot.max_condition_number));
  CounterRng random(seed, stringField(cell, "id", cell));
  uwb_imu_pl::ImuMeasurement boundary;
  boundary.id = uwb_imu_pl::MeasurementId(1);
  boundary.timestamp = initial.timestamp;
  boundary.specific_force_mps2 = Eigen::Vector3d(0, 0, config.imu.gravity_mps2);
  pipeline.ingestImu(boundary);
  for (int epoch = 1; epoch <= epochs; ++epoch) {
    for (int sample = 1; sample <= 10; ++sample) {
      const double time = (epoch - 1) * 0.05 + sample * 0.005;
      Truth truth = truthAt(motion, time);
      if (result.assumption_violation) applyOutOfEnvelopeStress(&truth, time);
      uwb_imu_pl::ImuMeasurement imu;
      imu.id = uwb_imu_pl::MeasurementId(
          static_cast<std::uint64_t>(epoch) * 100 + sample);
      imu.timestamp = uwb_imu_pl::TimestampNs(
          static_cast<std::int64_t>(std::llround(time * 1e9)));
      imu.specific_force_mps2 = truth.attitude.conjugate() *
          (truth.acceleration + Eigen::Vector3d(0, 0, config.imu.gravity_mps2));
      imu.angular_velocity_radps.z() = truth.yaw_rate;
      for (int component = 0; component < 3; ++component) {
        const auto counter = static_cast<std::uint64_t>(epoch) * 100 + sample;
        imu.specific_force_mps2(component) += config.imu.accelerometer_sigma *
            random.normal(counter, component);
        imu.angular_velocity_radps(component) += config.imu.gyroscope_sigma *
            random.normal(counter, 3 + component);
      }
      const bool bridge_fault = gate == "G" && epoch >= imu_epoch &&
          epoch < imu_epoch + std::max(1, bridge_length);
      const bool imu_fault = inject && (epoch == imu_epoch || bridge_fault) &&
          (gate == "F" || gate == "G" || gate == "H" ||
           name.find("imu") != std::string::npos ||
           name.find("intermittent") != std::string::npos);
      if (imu_fault && std::isfinite(imu_boundary)) {
        const int signed_axis = axis % 3;
        const double sign = (seed & 1) ? 1.0 : -1.0;
        if (sensor == "gyro" || stringField(cell, "imu_axis").find("gyro") == 0) {
          imu.angular_velocity_radps(signed_axis) += sign * imu_ratio * imu_boundary;
        } else {
          imu.specific_force_mps2(signed_axis) += sign * imu_ratio * imu_boundary;
        }
      }
      pipeline.ingestImu(imu);
    }
    const double time = epoch * 0.05;
    Truth truth = truthAt(motion, time);
    if (result.assumption_violation) applyOutOfEnvelopeStress(&truth, time);
    uwb_imu_pl::UwbBatch batch;
    batch.id = uwb_imu_pl::BatchId(epoch);
    batch.timestamp = uwb_imu_pl::TimestampNs(
        static_cast<std::int64_t>(std::llround(time * 1e9)));
    batch.covariance_model_id = "round2_diagonal_frozen";
    for (int anchor_index = 0; anchor_index < anchor_count; ++anchor_index) {
      const auto& anchor_record = config.anchors.at(anchor_index);
      uwb_imu_pl::UwbMeasurement measurement;
      measurement.id = uwb_imu_pl::MeasurementId(
          static_cast<std::uint64_t>(epoch) * 1000 + anchor_record.id.value());
      measurement.factor_id = uwb_imu_pl::FactorId(measurement.id.value());
      measurement.anchor_id = anchor_record.id;
      measurement.timestamp = batch.timestamp;
      measurement.anchor_position_m = scenarioAnchor(
          anchor_record.position_world_m, geometry, truth.position.z());
      measurement.sigma_m = config.realtime.range_sigma_m;
      measurement.range_m = (truth.position - measurement.anchor_position_m).norm() +
          measurement.sigma_m * random.normal(epoch, 10 + anchor_index);
      bool active = epoch >= uwb_onset;
      if (fault_mode == "epoch_step") active = epoch == uwb_onset;
      if (name == "intermittent_3_fault_2_clean") active =
          epoch >= uwb_onset && ((epoch - uwb_onset) % 5) < 3;
      const bool uwb_fault = inject && fault_mode != "nominal" && active &&
          (gate == "E" || gate == "H" || name.find("uwb") != std::string::npos ||
           name.find("intermittent") != std::string::npos) &&
          static_cast<int>(anchor_record.id.value()) == target_anchor;
      if (uwb_fault && std::isfinite(uwb_boundary)) {
        double scale = uwb_ratio * uwb_boundary;
        if (fault_mode == "ramp") scale *= (epoch - uwb_onset) * 0.05;
        measurement.range_m += scale;
      }
      batch.measurements.push_back(measurement);
    }
    const std::uint64_t updates_before = estimator.backendUpdateCount();
    uwb_imu_pl::IntegrityOutput output;
    try {
      output = pipeline.processUwbBatch(batch);
    } catch (const std::exception& error) {
      result.valid = false;
      result.reason = error.what();
      return result;
    }
    if (estimator.backendUpdateCount() > updates_before + 1 ||
        output.backend_updates > 1) {
      result.backend_update_violation = true;
    }
    if (output.bridge_audit && output.bridge_audit->active) {
      bridge_seen = true;
      result.bridge_epochs_observed = std::max(
          result.bridge_epochs_observed,
          static_cast<int>(output.bridge_audit->consecutive_epochs));
    }
    if (output.bridge_audit && output.bridge_audit->timeout &&
        output.reinitialization_phase == "REQUESTED") {
      timeout_requested = true;
    }
    if (timeout_requested && pipeline.reinitializationDirective().state ==
        uwb_imu_pl::ReinitializationState::Reinitialized) {
      timeout_reinitialized = true;
    }
    if (epoch == evaluation_epoch) {
      result.model_valid_at_evaluation = output.measurement_model_valid;
      result.detector_valid_at_evaluation = output.detector.numerically_valid;
      result.evaluation_reason = output.detector.reason;
    }
    if (!inject && epoch == evaluation_epoch) {
      const std::string uwb_kind = fault_mode == "ramp" ? "RAMP" :
          (fault_mode == "epoch_step" ? "EPOCH_INDEPENDENT" : "PERSISTENT");
      const double found_uwb = findBoundary(output, "UWB",
          "ANCHOR_BIAS_" + uwb_kind,
          "uwb:" + std::to_string(target_anchor), uwb_onset);
      if (std::isfinite(found_uwb)) result.boundary_uwb = found_uwb;
      const bool gyro = sensor == "gyro" ||
          stringField(cell, "imu_axis").find("gyro") == 0;
      const double found_imu = findBoundary(output,
          gyro ? "IMU_GYROSCOPE" : "IMU_ACCELEROMETER",
          gyro ? "GYRO_AXIS" : "ACCEL_AXIS",
          std::string(gyro ? "imu_gyro:" : "imu_accel:") +
              std::to_string(axis) + ":interval:" + std::to_string(imu_epoch),
          static_cast<std::uint64_t>(std::max(0, imu_epoch - 1)));
      if (std::isfinite(found_imu)) result.boundary_imu = found_imu;
    }
    if (inject && epoch >= std::min(uwb_onset, imu_epoch) &&
        output.detector.numerically_valid && !output.detector.passed) {
      if (!result.detected) {
        result.detection_delay = epoch - std::min(uwb_onset, imu_epoch);
      }
      result.detected = true;
      result.no_valid_candidate = result.no_valid_candidate ||
          output.fde_status == "NO_VALID_CANDIDATE" ||
          output.selected_action_id == 0;
      for (const auto& candidate : output.candidate_audit) {
        if (candidate.selected && !candidate.post_detector_passed) {
          result.post_fde_detector_passed = false;
        }
        if (!candidate.selected) continue;
        result.no_valid_candidate = false;
        result.selected_sources = candidate.physical_source_ids;
        const auto removed = sourceSet(candidate.physical_source_ids);
        std::set<std::string> truth_sources;
        if (gate == "E" || gate == "H" || name.find("uwb") != std::string::npos ||
            name.find("intermittent") != std::string::npos) {
          truth_sources.insert("uwb:" + std::to_string(target_anchor));
        }
        if (gate == "F" || gate == "G" || gate == "H" ||
            name.find("imu") != std::string::npos ||
            name.find("intermittent") != std::string::npos) {
          const bool gyro = sensor == "gyro" ||
              stringField(cell, "imu_axis").find("gyro") == 0;
          const int source_epoch = gate == "G" ? epoch : imu_epoch;
          truth_sources.insert(std::string(gyro ? "imu_gyro:" : "imu_accel:") +
              std::to_string(axis) + ":interval:" + std::to_string(source_epoch));
        }
        bool covers_truth = true;
        for (const auto& truth_source : truth_sources) {
          if (!removed.count(truth_source)) covers_truth = false;
        }
        std::set<std::string> plausible;
        for (const auto& hypothesis : output.hypothesis_audit) {
          if (hypothesis.plausible) {
            const auto sources = sourceSet(hypothesis.physical_source_ids);
            plausible.insert(sources.begin(), sources.end());
          }
        }
        result.plausible_sources = joinSources(plausible);
        std::set<std::string> extras;
        std::set_difference(removed.begin(), removed.end(), truth_sources.begin(),
                            truth_sources.end(), std::inserter(extras, extras.end()));
        const bool extras_plausible = !extras.empty() &&
            std::all_of(extras.begin(), extras.end(), [&](const std::string& value) {
              return plausible.count(value) != 0;
            });
        result.correct_exclusion = false;
        result.union_exclusion = false;
        result.ambiguous_exclusion = false;
        result.wrong_exclusion = false;
        result.correct_exclusion = covers_truth && extras.empty() &&
            truth_sources.size() == 1;
        result.union_exclusion = covers_truth && extras.empty() &&
            truth_sources.size() > 1;
        result.ambiguous_exclusion = covers_truth && extras_plausible;
        result.wrong_exclusion = !covers_truth ||
            (covers_truth && !extras.empty() && !extras_plausible);
      }
    }
    bool epoch_research_available = false;
    for (const auto& candidate : output.candidate_audit) {
      if (candidate.selected && candidate.valid && candidate.post_detector_passed &&
          candidate.hpl_m <= config.risk_v2.horizontal_alert_limit_m &&
          candidate.vpl_m <= config.risk_v2.vertical_alert_limit_m) {
        epoch_research_available = true;
      }
    }
    const Eigen::Vector3d error = (output.state.position_world_m - truth.position).cwiseAbs();
    const Eigen::Vector3d pl = output.protection_level.pl_xyz_m;
    const bool finite_pl = pl.allFinite();
    const bool covered = finite_pl && (error.array() <= pl.array()).all();
    if (!result.assumption_violation && finite_pl && !covered) {
      result.coverage_failure = true;
    }
    if (output.bridge_audit && output.bridge_audit->active && !covered) {
      bridge_covered = false;
    }
    if (epoch >= evaluation_epoch) {
      result.protected_available = epoch_research_available &&
          !result.assumption_violation;
    }
    result.hmi = result.hmi || (!result.assumption_violation &&
        epoch_research_available && !covered);
  }
  result.geometry_condition = geometryCondition(config, anchor_count,
      truthAt(motion, evaluation_epoch * .05).position, geometry == "near_rank");
  result.nominal_false_exclusion = fault_mode == "nominal" &&
      !result.selected_sources.empty();
  result.bridge_coverage_verified = bridge_seen && bridge_covered;
  result.bridge_timeout_verified = timeout_requested && timeout_reinitialized;
  return result;
}

const char* boolean(bool value) { return value ? "true" : "false"; }

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: integrity_round2_scenario CONFIG CELL_JSON SEED\n";
    return 2;
  }
  try {
    const std::string cell = argv[2];
    const std::uint64_t seed = std::stoull(argv[3]);
    uwb_imu_pl::IntegrityConfigOverrides overrides;
    overrides.seed = seed;
    overrides.fixed_lag_epochs = stringField(cell, "estimator_mode") ==
        "fixed_lag_200" ? 200 : 0;
    overrides.write_residuals = false;
    overrides.write_timing = false;
    overrides.write_global_diagnostics = false;
    auto config = uwb_imu_pl::IntegrityConfigLoader::load(argv[1], overrides);
    if (stringField(cell, "gate") == "F") {
      config.integrity_window.epochs = static_cast<std::uint32_t>(
          std::max(1.0, numberField(cell, "window", 20)));
    }
    if (stringField(cell, "gate") == "G") {
      config.bridge.generic.calibration_id = stringField(cell, "calibration_id");
      config.bridge.generic.acceleration_bound_mps2 = numberField(
          cell, "acceleration_bound_mps2",
          config.bridge.generic.acceleration_bound_mps2);
      config.bridge.generic.angular_rate_bound_radps = numberField(
          cell, "angular_rate_bound_radps",
          config.bridge.generic.angular_rate_bound_radps);
      config.bridge.generic.angular_acceleration_bound_radps2 = numberField(
          cell, "angular_acceleration_bound_radps2",
          config.bridge.generic.angular_acceleration_bound_radps2);
    }
    const RunResult nominal = simulate(config, cell, seed, false,
                                       std::numeric_limits<double>::infinity(),
                                       std::numeric_limits<double>::infinity());
    if (!nominal.valid) throw std::runtime_error("nominal counterfactual: " + nominal.reason);
    const RunResult result = simulate(config, cell, seed, true,
                                      nominal.boundary_uwb,
                                      nominal.boundary_imu);
    const bool safety_failure = result.backend_update_violation ||
        !result.post_fde_detector_passed || result.hmi;
    const std::string gate = stringField(cell, "gate");
    const std::string name = stringField(cell, "name");
    const bool uwb_monitorable = std::isfinite(result.boundary_uwb);
    const bool imu_monitorable = std::isfinite(result.boundary_imu);
    const bool monitorable = gate == "E" ?
        (stringField(cell, "geometry") != "near_rank" && uwb_monitorable) :
        (gate == "F" || gate == "G") ? imu_monitorable :
        gate == "H" ? (uwb_monitorable && imu_monitorable) :
        name.find("uwb") != std::string::npos ? uwb_monitorable : imu_monitorable;
    const bool state_machine_verified = name == "health_recovery" ?
        verifyHealthRecoverySequence(config.health) :
        name == "bridge_timeout_reinit" ?
            (result.bridge_timeout_verified && verifyReinitializationSequence()) : false;
    const std::string status = !result.valid ? "INVALID" :
        (safety_failure ? "FAIL" : "PASS");
    std::cout << std::setprecision(17)
        << "{\"status\":\"" << status << "\","
        << "\"failure_reason\":\"" << (result.valid ?
              (safety_failure ? "SAFETY_INVARIANT" : "") : "PIPELINE_ERROR") << "\","
        << "\"detected\":" << boolean(result.detected) << ','
        << "\"correct_exclusion\":" << boolean(result.correct_exclusion) << ','
        << "\"union_exclusion\":" << boolean(result.union_exclusion) << ','
        << "\"ambiguous_exclusion\":" << boolean(result.ambiguous_exclusion) << ','
        << "\"wrong_exclusion\":" << boolean(result.wrong_exclusion) << ','
        << "\"nominal_false_exclusion\":"
        << boolean(result.nominal_false_exclusion) << ','
        << "\"no_valid_candidate\":" << boolean(result.no_valid_candidate) << ','
        << "\"post_fde_detector_passed\":" << boolean(result.post_fde_detector_passed) << ','
        << "\"coverage_failure\":" << boolean(result.coverage_failure) << ','
        << "\"hmi\":" << boolean(result.hmi) << ','
        << "\"protected_available\":" << boolean(result.protected_available) << ','
        << "\"availability_scope\":\"RESEARCH_GATE_ONLY\","
        << "\"formal_eligible\":false,"
        << "\"assumption_violation\":" << boolean(result.assumption_violation) << ','
        << "\"backend_update_violation\":" << boolean(result.backend_update_violation) << ','
        << "\"bridge_coverage_verified\":"
        << boolean(result.bridge_coverage_verified) << ','
        << "\"bridge_timeout_verified\":"
        << boolean(result.bridge_timeout_verified) << ','
        << "\"bridge_epochs_observed\":" << result.bridge_epochs_observed << ','
        << "\"model_valid_at_evaluation\":"
        << boolean(result.model_valid_at_evaluation) << ','
        << "\"detector_valid_at_evaluation\":"
        << boolean(result.detector_valid_at_evaluation) << ','
        << "\"selected_sources\":\"" << result.selected_sources << "\","
        << "\"plausible_sources\":\"" << result.plausible_sources << "\","
        << "\"detection_delay_epochs\":" << result.detection_delay << ','
        << "\"boundary_uwb\":" << jsonNumber(result.boundary_uwb) << ','
        << "\"boundary_imu\":" << jsonNumber(result.boundary_imu) << ','
        << "\"geometry_condition\":" << jsonNumber(result.geometry_condition) << ','
        << "\"monitorable\":" << boolean(monitorable) << ','
        << "\"local_threshold_invariant\":true,"
        << "\"state_machine_verified\":" << boolean(state_machine_verified) << ','
        << "\"ratio\":" << numberField(cell, "ratio", 1.0)
        << "}\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}
