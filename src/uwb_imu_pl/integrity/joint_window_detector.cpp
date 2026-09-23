#include "uwb_imu_pl/integrity/joint_window_detector.hpp"

#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"
#include <sstream>
#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <boost/math/distributions/chi_squared.hpp>

#include <Eigen/QR>

#include <algorithm>
#include <cmath>

namespace uwb_imu_pl {

std::string stateStepAttribution(const LinearizedIntegrityWindow& window,
                                 const Eigen::VectorXd& increment) {
  if (increment.size() == 0) return "step attribution: empty increment";
  double rotation = 0.0, position = 0.0, velocity = 0.0;
  double accel_bias = 0.0, gyro_bias = 0.0;
  double dominant = -1.0;
  std::string dominant_label = "none";
  std::uint64_t dominant_epoch = 0;
  auto consider = [&](const std::string& label, double value,
                      std::uint64_t epoch) {
    if (!std::isfinite(value)) return;
    if (value > dominant) {
      dominant = value;
      dominant_label = label;
      dominant_epoch = epoch;
    }
  };
  for (const auto& layout : window.state_layout) {
    if (layout.dimension < 15 || layout.column_offset < 0 ||
        layout.column_offset + 15 > increment.size()) {
      continue;
    }
    const Eigen::Index offset = layout.column_offset;
    const double r = increment.segment(offset, 3).norm();
    const double p = increment.segment(offset + 3, 3).norm();
    const double v = increment.segment(offset + 6, 3).norm();
    const double ba = increment.segment(offset + 9, 3).norm();
    const double bg = increment.segment(offset + 12, 3).norm();
    rotation = std::max(rotation, r);
    position = std::max(position, p);
    velocity = std::max(velocity, v);
    accel_bias = std::max(accel_bias, ba);
    gyro_bias = std::max(gyro_bias, bg);
    consider("rotation", r, layout.epoch);
    consider("position", p, layout.epoch);
    consider("velocity", v, layout.epoch);
    consider("accel_bias", ba, layout.epoch);
    consider("gyro_bias", bg, layout.epoch);
  }
  std::ostringstream out;
  out << "step attribution: rotation=" << rotation
      << " position=" << position << " velocity=" << velocity
      << " accel_bias=" << accel_bias << " gyro_bias=" << gyro_bias
      << "; dominant=" << dominant_label << "(epoch " << dominant_epoch << ")";
  return out.str();
}

namespace {

void hashDetectorBytes(std::uint64_t* hash, const void* data,
                       std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= bytes[index];
    *hash *= 1099511628211ULL;
  }
}

template <class T>
void hashDetectorScalar(std::uint64_t* hash, const T& value) {
  hashDetectorBytes(hash, &value, sizeof(value));
}

void hashDetectorString(std::uint64_t* hash, const std::string& value) {
  const std::uint64_t size = value.size();
  hashDetectorScalar(hash, size);
  if (!value.empty()) hashDetectorBytes(hash, value.data(), value.size());
}

double squaredThreshold(int dof, double p_fa) {
  if (dof <= 0 || !std::isfinite(p_fa) || p_fa <= 0.0 || p_fa >= 1.0) {
    return std::numeric_limits<double>::infinity();
  }
  return StatisticalBoundsCache::chiSquaredThreshold(dof, p_fa);
}

DetectorResultV2 baseResult(int rows, int rank, int dof,
                            double statistic, const DetectorRiskContext& risk) {
  DetectorResultV2 out;
  out.rows = rows;
  out.rank = rank;
  out.dof = dof;
  out.squared_parity_statistic = statistic;
  out.squared_threshold = squaredThreshold(dof, risk.p_fa_per_test);
  out.p_fa_per_test = risk.p_fa_per_test;
  out.operation_p_fa_upper_bound = std::min(
      1.0, risk.p_fa_per_test * static_cast<double>(risk.continuity_horizon_tests));
  out.numerically_valid = dof > 0 && std::isfinite(statistic) &&
      std::isfinite(out.squared_threshold);
  out.passed = out.numerically_valid && statistic <= out.squared_threshold;
  if (!out.numerically_valid) out.reason = "invalid joint detector rank/dof/statistic";
  else if (!out.passed) out.reason = "joint window squared parity alarm";
  return out;
}

}  // namespace

std::uint64_t detectorContractDigest(const DetectorResultV2& detector) {
  std::uint64_t hash = 1469598103934665603ULL;
  hashDetectorScalar(&hash, detector.window_id.value());
  hashDetectorScalar(&hash, detector.squared_parity_statistic);
  hashDetectorScalar(&hash, detector.squared_threshold);
  hashDetectorScalar(&hash, detector.rows);
  hashDetectorScalar(&hash, detector.rank);
  hashDetectorScalar(&hash, detector.dof);
  hashDetectorScalar(&hash, detector.p_fa_per_test);
  hashDetectorScalar(&hash, detector.operation_p_fa_upper_bound);
  hashDetectorScalar(&hash, detector.passed);
  hashDetectorScalar(&hash, detector.numerically_valid);
  hashDetectorScalar(&hash, detector.channel_current_statistic);
  hashDetectorScalar(&hash, detector.channel_current_threshold);
  hashDetectorScalar(&hash, detector.channel_current_dof);
  hashDetectorScalar(&hash, detector.channel_current_accepted);
  hashDetectorScalar(&hash, detector.channel_history_statistic);
  hashDetectorScalar(&hash, detector.channel_history_threshold);
  hashDetectorScalar(&hash, detector.channel_history_dof);
  hashDetectorScalar(&hash, detector.channel_history_accepted);
  hashDetectorScalar(&hash, detector.channel_split_valid);
  hashDetectorScalar(&hash, detector.joint_accepted);
  hashDetectorScalar(&hash, static_cast<int>(detector.contract_mode));
  hashDetectorString(&hash, detector.accepted_event_id);
  hashDetectorScalar(&hash, detector.candidate_numerical_identity);
  hashDetectorScalar(&hash, detector.candidate_certificate_digest);
  hashDetectorScalar(&hash, detector.continuity_horizon_tests);
  hashDetectorScalar(&hash, detector.active_channel_count);
  hashDetectorScalar(&hash, detector.history_constant);
  return hash;
}

DetectorResultV2 JointWindowDetector::evaluate(
    const LinearizedIntegrityWindow& window,
    const DetectorRiskContext& risk) const {
  if (!window.model_valid) {
    DetectorResultV2 out;
    out.window_id = window.id;
    out.rows = window.H.rows();
    out.rank = window.rank;
    out.dof = window.dof;
    out.reason = "invalid integrity window: " + window.reason;
    return out;
  }
  if (!window.numerics || !window.numerics->valid ||
      window.numerics->content_fingerprint != integrityWindowFingerprint(window) ||
      window.numerics->numerical_contract_fingerprint !=
          numericalContractFingerprint(risk.rank_tolerance,
                                       risk.max_condition_number)) {
    if (window.numerics &&
        window.numerics->numerical_contract_fingerprint !=
            numericalContractFingerprint(risk.rank_tolerance,
                                         risk.max_condition_number)) {
      NumericalWorkCounters::numericalContractMismatch();
    }
    DetectorResultV2 out;
    out.window_id = window.id;
    out.reason = "missing, stale, or numerical-contract-mismatched frozen-window numerics";
    return out;
  }
  int rank = window.numerics->exact_rank;
  double statistic = window.numerics->statistic;
  // Preserve an independent Jacobian-space reference near the original
  // condition gate. Normal well-conditioned windows perform no QR.
  if (window.numerics->exact_condition >= risk.max_condition_number / 10.0) {
    NumericalWorkCounters::detectorReferenceQr();
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(window.H);
    qr.setThreshold(risk.rank_tolerance);
    const Eigen::VectorXd increment = qr.solve(window.z);
    const double reference_statistic =
        (window.z - window.H * increment).squaredNorm();
    const double scale = std::max({1.0, std::abs(statistic),
                                   std::abs(reference_statistic)});
    if (qr.rank() != rank ||
        std::abs(reference_statistic - statistic) >
            window.numerics->numerical_contract.reference_relative_tolerance *
                scale) {
      DetectorResultV2 out;
      out.window_id = window.id;
      out.rows = window.H.rows();
      out.rank = rank;
      out.dof = window.H.rows() - rank;
      out.reason = "shared/reference detector mismatch";
      return out;
    }
  }
  const double history_constant = window.history_summary.present &&
      window.history_summary.valid ? window.history_summary.constant_offset
                                   : 0.0;
  auto out = baseResult(window.H.rows(), rank,
                        window.H.rows() - rank,
                        statistic + history_constant, risk);
  out.window_id = window.id;
  // C2 (§7.3): the separated layout on the same frozen window.  Thresholds are
  // the unchanged per-test budget; the joint acceptance is reported, not
  // substituted for the pooled decision.
  {
    const DualChannelDecision split = evaluateDualChannel(
        window, risk.p_fa_per_test, risk.continuity_horizon_tests);
    out.channel_split_valid = split.numerically_valid;
    out.joint_accepted = split.joint_accepted;
    out.channel_reason = split.reason;
    out.channel_current_statistic = split.current.statistic;
    out.channel_current_threshold = split.current.threshold;
    out.channel_current_dof = split.current.dof;
    out.channel_current_accepted = split.current.accepted;
    out.channel_history_statistic = split.history.statistic;
    out.channel_history_threshold = split.history.threshold;
    out.channel_history_dof = split.history.dof;
    out.channel_history_accepted = split.history.accepted;
    out.operation_p_fa_upper_bound = split.operation_p_fa_upper_bound;
    out.accepted_event_id = "dual_channel_intersection_v6";
    out.candidate_numerical_identity = candidateDetectorNumericalIdentity(
        window, ExclusionAction{});
    out.history_constant = window.history_summary.present
        ? window.history_summary.constant_offset : 0.0;
    out.continuity_horizon_tests =
        std::max<std::uint64_t>(1, risk.continuity_horizon_tests);
    out.active_channel_count = split.channel_count;
    out.numerically_valid = out.numerically_valid && split.numerically_valid;
    out.passed = out.numerically_valid && split.joint_accepted;
    if (!split.numerically_valid) out.reason = split.reason;
    else if (!split.joint_accepted) {
      out.reason = "dual-channel current/history acceptance failed";
    } else {
      out.reason.clear();
    }
  }
  if (window.condition_number > risk.max_condition_number) {
    out.numerically_valid = false;
    out.passed = false;
    out.reason = "joint window condition gate failed";
  }
  out.detector_contract_digest = detectorContractDigest(out);
  return out;
}

DetectorResultV2 JointWindowDetector::evaluateCandidate(
    const LinearizedIntegrityWindow& window,
    const CandidateEvaluation& candidate,
    const DetectorRiskContext& risk) const {
  if (!candidate.valid) {
    DetectorResultV2 out;
    out.numerically_valid = false;
    out.passed = false;
    out.reason = "invalid candidate: " + candidate.reason;
    return out;
  }
  std::string certificate_reason;
  if (!validateCandidateDetectorCertificate(window, candidate,
                                            &certificate_reason)) {
    DetectorResultV2 out;
    out.window_id = window.id;
    out.numerically_valid = false;
    out.passed = false;
    out.reason = "invalid active-v6 candidate detector certificate: " +
                 certificate_reason;
    return out;
  }
  const auto& certificate = candidate.detector_certificate;
  const int current_role_count = static_cast<int>(std::count(
      certificate.row_roles.begin(), certificate.row_roles.end(),
      CandidateDetectorRowRole::CurrentStateSupported));
  const int history_role_count = static_cast<int>(std::count(
      certificate.row_roles.begin(), certificate.row_roles.end(),
      CandidateDetectorRowRole::HistoryDetectorOnly));
  const double represented_pooled = certificate.current_statistic +
      certificate.history_statistic;
  const double expected_pooled = candidate.statistic +
      certificate.history_constant;
  const double identity_scale = std::max(
      {1.0, std::abs(represented_pooled), std::abs(expected_pooled)});
  if (!certificate.valid || !certificate.history_constant_present ||
      certificate.accepted_event_id != "dual_channel_intersection_v6" ||
      certificate.numerical_identity == 0 ||
      certificate.pooled_rank != candidate.rank ||
      certificate.pooled_dof != candidate.dof ||
      certificate.row_roles.size() != static_cast<std::size_t>(candidate.rows) ||
      current_role_count != certificate.current_rows ||
      history_role_count != certificate.history_rows ||
      certificate.current_rows + certificate.history_rows != candidate.rows ||
      certificate.current_rank != candidate.rank ||
      certificate.history_rank != 0 ||
      certificate.current_dof + certificate.history_dof != candidate.dof ||
      !std::isfinite(certificate.history_constant) ||
      !std::isfinite(represented_pooled) ||
      std::abs(represented_pooled - expected_pooled) >
          1e-7 * identity_scale) {
    DetectorResultV2 out;
    out.numerically_valid = false;
    out.passed = false;
    out.reason = "missing or invalid active-v6 candidate detector certificate";
    if (!certificate.reason.empty()) out.reason += ": " + certificate.reason;
    return out;
  }
  auto out = baseResult(
      candidate.rows, candidate.rank, candidate.dof,
      certificate.current_statistic + certificate.history_statistic, risk);
  out.window_id = window.id;
  auto channel = [&](const char* id, int rows, int rank, int dof,
                     double statistic) {
    ChannelTest test;
    test.detector_id = id;
    test.dof = dof;
    test.statistic = statistic;
    test.p_fa = risk.p_fa_per_test;
    if (rows == 0) {
      test.threshold = 0.0;
      test.numerically_valid = rank == 0 && dof == 0;
      test.accepted = test.numerically_valid;
      return test;
    }
    test.threshold = squaredThreshold(dof, risk.p_fa_per_test);
    test.numerically_valid = rank >= 0 && dof > 0 &&
        rows == rank + dof && std::isfinite(statistic) &&
        std::isfinite(test.threshold);
    test.accepted = test.numerically_valid && statistic <= test.threshold;
    return test;
  };
  const ChannelTest current = channel(
      "joint_window_state_supported", certificate.current_rows,
      certificate.current_rank, certificate.current_dof,
      certificate.current_statistic);
  const ChannelTest history = channel(
      "history_eliminated_detector_only", certificate.history_rows,
      certificate.history_rank, certificate.history_dof,
      certificate.history_statistic);
  out.channel_current_statistic = current.statistic;
  out.channel_current_threshold = current.threshold;
  out.channel_current_dof = current.dof;
  out.channel_current_accepted = current.accepted;
  out.channel_history_statistic = history.statistic;
  out.channel_history_threshold = history.threshold;
  out.channel_history_dof = history.dof;
  out.channel_history_accepted = history.accepted;
  const int channel_count = certificate.history_rows > 0 ? 2 : 1;
  const std::uint64_t continuity_horizon_tests =
      std::max<std::uint64_t>(1, risk.continuity_horizon_tests);
  out.operation_p_fa_upper_bound = std::min(
      1.0, risk.p_fa_per_test *
          static_cast<double>(continuity_horizon_tests) *
          static_cast<double>(channel_count));
  out.channel_split_valid = current.numerically_valid &&
                            history.numerically_valid;
  out.joint_accepted = current.accepted && history.accepted;
  out.numerically_valid = out.numerically_valid && out.channel_split_valid;
  out.passed = out.numerically_valid && out.joint_accepted;
  out.accepted_event_id = certificate.accepted_event_id;
  out.candidate_numerical_identity = certificate.numerical_identity;
  out.candidate_certificate_digest = certificate.certificate_digest;
  out.continuity_horizon_tests = continuity_horizon_tests;
  out.active_channel_count = channel_count;
  out.history_constant = certificate.history_constant;
  if (!out.passed) out.reason = out.numerically_valid
      ? "post-FDE dual-channel detector alarm"
      : "post-FDE dual-channel detector invalid";
  else out.reason.clear();
  out.detector_contract_digest = detectorContractDigest(out);
  return out;
}

}  // namespace uwb_imu_pl
