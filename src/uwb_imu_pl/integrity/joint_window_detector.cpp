#include "uwb_imu_pl/integrity/joint_window_detector.hpp"
#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <boost/math/distributions/chi_squared.hpp>

#include <Eigen/QR>

#include <algorithm>
#include <cmath>

namespace uwb_imu_pl {
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
  if (!candidate.valid) {
    out.numerically_valid = false;
    out.passed = false;
    out.reason = "invalid candidate: " + candidate.reason;
  }
  return out;
}

}  // namespace uwb_imu_pl
