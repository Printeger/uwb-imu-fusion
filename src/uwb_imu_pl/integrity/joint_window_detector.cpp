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
  auto out = baseResult(window.H.rows(), rank,
                        window.H.rows() - rank, statistic, risk);
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
  return out;
}

DetectorResultV2 JointWindowDetector::evaluateCandidate(
    const CandidateEvaluation& candidate,
    const DetectorRiskContext& risk) const {
  auto out = baseResult(candidate.rows, candidate.rank, candidate.dof,
                        candidate.statistic, risk);
  const Eigen::MatrixXd& h = candidate.retainedJacobian();
  if (candidate.valid && h.rows() == candidate.retained_residual.size() &&
      h.rows() > 0) {
    std::vector<Eigen::Index> current_rows;
    std::vector<Eigen::Index> history_rows;
    for (Eigen::Index row = 0; row < h.rows(); ++row) {
      (h.row(row).norm() <= risk.rank_tolerance ? history_rows : current_rows)
          .push_back(row);
    }
    auto channel = [&](const char* id,
                       const std::vector<Eigen::Index>& rows) {
      ChannelTest test;
      test.detector_id = id;
      test.p_fa = risk.p_fa_per_test;
      if (rows.empty()) {
        test.numerically_valid = true;
        test.accepted = true;
        return test;
      }
      Eigen::MatrixXd a(rows.size(), h.cols());
      Eigen::VectorXd b(rows.size());
      for (std::size_t i = 0; i < rows.size(); ++i) {
        a.row(static_cast<Eigen::Index>(i)) = h.row(rows[i]);
        b(static_cast<Eigen::Index>(i)) = candidate.retained_residual(rows[i]);
      }
      Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);
      qr.setThreshold(risk.rank_tolerance);
      const int rank = qr.rank();
      test.dof = static_cast<int>(rows.size()) - rank;
      test.statistic = (b - a * qr.solve(b)).squaredNorm();
      if (test.dof > 0) {
        test.threshold = squaredThreshold(test.dof, risk.p_fa_per_test);
        test.numerically_valid = std::isfinite(test.statistic) &&
                                 std::isfinite(test.threshold);
        test.accepted = test.numerically_valid &&
                        test.statistic <= test.threshold;
      } else if (std::string(id) == "history_eliminated_detector_only") {
        test.numerically_valid = true;
        test.accepted = true;
      }
      return test;
    };
    const ChannelTest current = channel("joint_window_state_supported",
                                        current_rows);
    const ChannelTest history = channel("history_eliminated_detector_only",
                                        history_rows);
    out.channel_current_statistic = current.statistic;
    out.channel_current_threshold = current.threshold;
    out.channel_current_dof = current.dof;
    out.channel_current_accepted = current.accepted;
    out.channel_history_statistic = history.statistic;
    out.channel_history_threshold = history.threshold;
    out.channel_history_dof = history.dof;
    out.channel_history_accepted = history.accepted;
    out.channel_split_valid = current.numerically_valid &&
                              history.numerically_valid;
    out.joint_accepted = current.accepted && history.accepted;
    out.numerically_valid = out.numerically_valid && out.channel_split_valid;
    out.passed = out.numerically_valid && out.joint_accepted;
    if (!out.passed) out.reason = out.numerically_valid
        ? "post-FDE dual-channel detector alarm"
        : "post-FDE dual-channel detector invalid";
  }
  if (!candidate.valid) {
    out.numerically_valid = false;
    out.passed = false;
    out.reason = "invalid candidate: " + candidate.reason;
  }
  return out;
}

}  // namespace uwb_imu_pl
