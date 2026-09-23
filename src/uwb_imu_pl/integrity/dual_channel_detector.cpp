// C2 implementation: §7.3 channel separation, §7.4 dual-channel bound,
// §7.5 fault-span projection and the §8.2 model-error channel.

#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"

#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <sstream>

#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

namespace uwb_imu_pl {

namespace {

// FNV-1a over the certificate fields (identity of the proof, not a hash of
// floating point noise: it binds the structural inputs).
void hashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= bytes[index];
    *hash *= 1099511628211ULL;
  }
}

std::string hexDigest(std::uint64_t value) {
  std::ostringstream out;
  out << std::hex << value;
  return out.str();
}

}  // namespace

DualChannelDecision evaluateDualChannel(
    const LinearizedIntegrityWindow& window, double p_fa_per_test,
    std::uint32_t continuity_horizon_tests) {
  DualChannelDecision out;
  if (!window.numerics || !window.numerics->valid || !window.model_valid) {
    out.reason = "window numerics are not usable for the channel split";
    return out;
  }
  const auto& history = window.history_summary;
  // Channel 2 exists only when the summary actually carries detector-only rows;
  // a summary without them makes the history channel empty, not zero-sized
  // (an empty channel must not fabricate a threshold).
  const int history_dof = history.valid ? history.nu_perp : 0;
  const double history_statistic =
      history.valid ? history.kappa_b + history.constant_offset : 0.0;
  // The square-root row system cannot encode the scalar remainder produced
  // when an information-form boundary factor is rebuilt.  C1 records that
  // remainder as constant_offset.  It belongs to the pooled quadratic form
  // and to the eliminated-history channel, so add it before subtracting the
  // latter.  Omitting it here made the current statistic spuriously negative
  // once fixed-lag marginalization produced a non-zero offset.
  const double pooled_statistic = window.numerics->statistic +
      (history.valid ? history.constant_offset : 0.0);
  const int pooled_dof = window.numerics->dof;
  const int current_dof = pooled_dof - history_dof;
  const double current_statistic = pooled_statistic - history_statistic;
  if (current_dof <= 0 || !std::isfinite(current_statistic) ||
      current_statistic < -1e-9 * std::max(1.0, std::abs(pooled_statistic))) {
    out.reason =
        "channel split is not admissible (non-positive current dof or "
        "inconsistent residual accounting)";
    return out;
  }
  auto makeTest = [&](const char* id, int dof, double statistic) {
    ChannelTest test;
    test.detector_id = id;
    test.dof = dof;
    test.statistic = statistic;
    test.p_fa = p_fa_per_test;
    test.threshold = StatisticalBoundsCache::chiSquaredThreshold(
        dof, p_fa_per_test);
    test.numerically_valid = dof > 0 && std::isfinite(statistic) &&
                             std::isfinite(test.threshold);
    test.accepted = test.numerically_valid && statistic <= test.threshold;
    return test;
  };
  out.current = makeTest("joint_window_state_supported", current_dof,
                         std::max(0.0, current_statistic));
  if (history_dof > 0) {
    out.history = makeTest("history_eliminated_detector_only", history_dof,
                           std::max(0.0, history_statistic));
    out.channel_count = 2;
  } else {
    // Explicit cold-start degeneration: no history test is claimed and the
    // contract reduces to the current channel instead of fabricating dof.
    out.history.detector_id = "history_channel_not_present";
    out.history.dof = 0;
    out.history.statistic = 0.0;
    out.history.threshold = 0.0;
    out.history.p_fa = 0.0;
    out.history.numerically_valid = true;
    out.history.accepted = true;
    out.channel_count = 1;
  }
  out.joint_accepted = out.current.accepted && out.history.accepted;
  out.numerically_valid = out.current.numerically_valid &&
                          out.history.numerically_valid;
  // Union bound with the unchanged per-test budget: exactly what the platform
  // pays for testing both channels, reported instead of hidden.
  out.operation_p_fa_upper_bound =
      std::min(1.0, p_fa_per_test *
                        static_cast<double>(std::max(1u, continuity_horizon_tests)) *
                        static_cast<double>(out.channel_count));
  if (!out.numerically_valid) {
    out.reason = "one of the two channels has no usable threshold/dof";
  }
  return out;
}

DualChannelBoundResult computeDualChannelBound(
    const DualChannelBoundRequest& request) {
  DualChannelBoundResult out;
  if (request.channels.empty()) {
    out.reason = "no channels supplied";
    return out;
  }
  if (!(request.p_md > 0.0 && request.p_md < 1.0)) {
    out.reason = "beta_h (p_md) must be in (0,1)";
    return out;
  }
  if (request.protected_response.rows() != 3) {
    out.reason = "protected response must have one row per protected axis";
    return out;
  }
  const Eigen::Index dimension = request.protected_response.cols();
  double weight_sum = 0.0;
  std::vector<const ChannelBoundInput*> usable;
  for (const auto& channel : request.channels) {
    if (!(channel.weight > 0.0)) {
      out.reason = "channel weights must be strictly positive";
      return out;
    }
    if (channel.gram.rows() != dimension || channel.gram.cols() != dimension) {
      out.reason = "channel Gram does not match the protected parameter space";
      return out;
    }
    if (channel.gram.norm() == 0.0) {
      // No fault contribution: the channel must not fabricate a Lambda.
      continue;
    }
    if (channel.dof <= 0) {
      // No degrees of freedom: likewise no threshold can be assigned.
      continue;
    }
    usable.push_back(&channel);
  }
  if (usable.empty()) {
    out.reason =
        "no channel has both degrees of freedom and fault contribution";
    return out;
  }
  for (const auto* channel : usable) weight_sum += channel->weight;

  // Conservative per-channel non-centrality: Lambda_{h,j} solves
  // F_{chi2_{nu_j}(Lambda)}(tau_j) <= beta_h  (B3 verified solver, unchanged).
  std::vector<Eigen::MatrixXd> scaled;
  for (const auto* channel : usable) {
    const double threshold =
        riskAdjustedThreshold(channel->threshold, channel->residual_rho);
    const StatisticalBoundKey key;
    const NoncentralityBoundaryResult boundary =
        StatisticalBoundsCache::noncentralityBoundaryVerified(
            channel->dof, threshold, request.p_md, key);
    if (!boundary.valid) {
      out.reason = "channel " + channel->detector_id +
                   " has no verified non-centrality boundary: " +
                   boundary.reason;
      return out;
    }
    const double lambda = boundary.value;
    if (!(lambda > 0.0) || !std::isfinite(lambda)) {
      out.reason = "channel " + channel->detector_id +
                   " boundary is not strictly positive";
      return out;
    }
    // w_j is split over the *usable* channels so the weights still sum to one.
    const double weight = channel->weight / weight_sum;
    const double scale = std::sqrt(weight / lambda);
    // Small factorization path: accumulate W_h = sum_j (w_j / Lambda_j) Gamma_j.
    // The bound below only needs a thin SVD of this small (parameter_dim^2)
    // symmetric PSD matrix -- W_h is never inverted explicitly.
    scaled.push_back(scale * scale * channel->gram);
    out.channel_lambda.push_back(std::sqrt(lambda));
    out.channel_weight.push_back(weight);
    // F_{chi2_{nu}(Lambda)}(tau) <= beta must hold on the conservative side.
    out.channel_lambda_gap.push_back(boundary.residual);
    out.channel_dof.push_back(channel->dof);
    out.channel_threshold.push_back(threshold);
    if (!std::isfinite(channel->residual_rho) || channel->residual_rho < 0.0) {
      out.reason = "residual model-error term must be finite and non-negative";
      return out;
    }
    out.model_error_validated =
        out.model_error_validated && channel->rho_validated;
    if (!channel->rho_source.empty()) {
      out.model_error_source = channel->rho_source;
    }
  }
  Eigen::MatrixXd w = scaled.empty() ? Eigen::MatrixXd::Zero(dimension, dimension)
                                     : scaled.front();  for (std::size_t index = 1; index < scaled.size(); ++index) w += scaled[index];
  out.w_matrix = w;
  out.channel_lambda_gap.push_back(0.0);  // keep lengths explicit in audits
  out.channel_lambda_gap.pop_back();

  // Rank of W_h (audit only) and the kernel the bound relies on.
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> w_qr(w);
  w_qr.setThreshold(1e-12);
  out.w_rank = static_cast<double>(w_qr.rank());
  // ker W_h subseteq ker G_h: every direction the bound will amplify must still
  // have a protected-state response, otherwise W_h^dagger is being applied to a
  // direction the fault can move without the protected state noticing.
  {
    const Eigen::MatrixXd q_matrix = w_qr.matrixQ();
    const Eigen::MatrixXd kernel =
        q_matrix.rightCols(std::max<Eigen::Index>(0, w.cols() - w_qr.rank()));
    const double covered = (request.protected_response * kernel).norm();
    const double scale = std::max(1.0, request.protected_response.norm());
    out.w_kernel_covered = covered <= 1e-9 * scale;
    if (!out.w_kernel_covered) {
      out.reason = "ker W_h is not covered by ker G_h";
      return out;
    }
  }
  // b_{h,d} = sqrt(g_{h,d} W_h^dagger g_{h,d}') through the small SVD of the
  // symmetric PSD W_h:
  //   b_d = || Sigma^-1 V' g_d' ||   with W_h = V Sigma^2 V'.
  // No explicit inverse of W_h is formed; the rank truncation is exactly the
  // kernel the feasibility check above certified as immaterial for G_h.
  Eigen::JacobiSVD<Eigen::MatrixXd> w_svd(w, Eigen::ComputeThinV);
  const Eigen::VectorXd singular = w_svd.singularValues();
  const double singular_floor =
      singular.size() > 0 ? 1e-12 * singular(0) : 0.0;
  int kept = 0;
  for (int index = 0; index < singular.size(); ++index) {
    if (singular(index) > singular_floor) ++kept;
  }
  const Eigen::MatrixXd v_kept = Eigen::MatrixXd(w_svd.matrixV().leftCols(kept));
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::VectorXd g = request.protected_response.row(axis).transpose();
    double quadratic = 0.0;
    for (int index = 0; index < v_kept.cols(); ++index) {
      const double projection = v_kept.col(index).dot(g);
      quadratic += projection * projection / singular(index);
    }
    out.axis_bound_m(axis) = std::sqrt(std::max(0.0, quadratic));
  }
  out.position_rho_m = request.position_rho_m;
  out.model_error_validated =
      out.model_error_validated && request.position_rho_validated;
  if (!request.position_rho_source.empty()) {
    out.model_error_source = request.position_rho_source;
  }
  if (!out.position_rho_m.allFinite() || (out.position_rho_m.array() < 0.0).any()) {
    out.reason = "position model-error terms must be finite and non-negative";
    return out;
  }
  // Certificate: detector ids, tau_j, nu_j, beta_h, Lambda_{h,j}, w_j and the
  // W proof id -- the identity of the formula instance, not of the numbers.
  {
    std::uint64_t hash = 1469598103934665603ULL;
    const std::uint64_t dim = static_cast<std::uint64_t>(dimension);
    hashBytes(&hash, &dim, sizeof(dim));
    hashBytes(&hash, &request.p_md, sizeof(request.p_md));
    for (const auto* channel : usable) {
      hashBytes(&hash, channel->detector_id.data(), channel->detector_id.size());
      const std::uint64_t tuple[2] = {static_cast<std::uint64_t>(channel->dof), 0};
      hashBytes(&hash, tuple, sizeof(tuple));
      hashBytes(&hash, &channel->threshold, sizeof(channel->threshold));
    }
    for (const double value : out.channel_lambda) {
      hashBytes(&hash, &value, sizeof(value));
    }
    for (const double value : out.channel_weight) {
      hashBytes(&hash, &value, sizeof(value));
    }
    out.certificate_id = std::string("dual_channel_bound_w_") + hexDigest(hash);
  }
  out.valid = true;
  return out;
}

FaultSpanProjectionResult buildFaultSpanProjection(
    const Eigen::MatrixXd& declared_directions, double tolerance) {
  FaultSpanProjectionResult out;
  if (declared_directions.rows() == 0 || declared_directions.cols() == 0) {
    out.reason = "declared fault span is empty";
    return out;
  }
  // The retained basis IS the declared span: no top-K selection, no observed
  // fault ranking.  An orthonormal basis of the declared directions is taken
  // from the thin SVD, so the span is preserved exactly (up to the rank the
  // declaration actually has).
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(declared_directions, Eigen::ComputeThinU);
  const Eigen::VectorXd singular = svd.singularValues();
  const double floor = tolerance * (singular.size() > 0 ? singular(0) : 0.0);
  int rank = 0;
  for (int index = 0; index < singular.size(); ++index) {
    if (singular(index) > floor) ++rank;
  }
  out.declared_directions = static_cast<std::size_t>(declared_directions.cols());
  out.retained_directions = static_cast<std::size_t>(rank);
  out.declared_span_gap = rank != static_cast<int>(declared_directions.cols());
  if (out.declared_span_gap) {
    out.reason =
        "declared fault span is rank deficient: some declared direction is a "
        "combination of others (recorded, not silently trimmed)";
  }
  out.basis = Eigen::MatrixXd(svd.matrixU().leftCols(rank));
  out.valid = rank > 0;
  if (!out.valid) out.reason = "declared fault span has no rank";
  return out;
}

FaultSpanProjectionResult evaluateFaultSpanIdentity(
    const Eigen::MatrixXd& full_response,
    const Eigen::MatrixXd& projected_basis, double tolerance) {
  FaultSpanProjectionResult out;
  if (full_response.rows() == 0 || projected_basis.rows() != full_response.cols()) {
    out.reason = "shape mismatch between F_all and the projection basis";
    return out;
  }
  // Z_b = F_all U ; the identity under test is Z_b' Z_b == F_all' F_all.  It is
  // checked in the FULL parameter space,
  //   U (Z_b' Z_b) U' == F_all' F_all,
  // which stays dimensionally valid for a truncated (top-K) basis and fails
  // exactly when the retained span misses a declared direction.
  const Eigen::MatrixXd projected = full_response * projected_basis;  // rows x kept
  const Eigen::MatrixXd covered =
      projected_basis * (projected.transpose() * projected) *
      projected_basis.transpose();  // dim x dim
  const Eigen::MatrixXd rhs = full_response.transpose() * full_response;
  const double scale = std::max(1.0, rhs.norm());
  out.exactness_residual = (covered - rhs).norm() / scale;
  out.retained_directions = static_cast<std::size_t>(projected_basis.cols());
  out.valid = out.exactness_residual <= tolerance;
  if (!out.valid) {
    out.reason =
        "Z_b'Z_b != F_all'F_all: the projected span does not cover the "
        "declared fault directions";
  }
  return out;
}

double riskAdjustedThreshold(double actual_threshold, double residual_rho) {
  if (!(actual_threshold > 0.0) || !std::isfinite(actual_threshold) ||
      !std::isfinite(residual_rho) || residual_rho < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double root = std::sqrt(actual_threshold) + residual_rho;
  return root * root;
}

}  // namespace uwb_imu_pl
