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

template <class Derived>
void hashMatrix(std::uint64_t* hash,
                const Eigen::MatrixBase<Derived>& matrix) {
  const std::int64_t rows = matrix.rows();
  const std::int64_t columns = matrix.cols();
  hashBytes(hash, &rows, sizeof(rows));
  hashBytes(hash, &columns, sizeof(columns));
  for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      const double value = matrix(row, column) == 0.0
          ? 0.0 : static_cast<double>(matrix(row, column));
      hashBytes(hash, &value, sizeof(value));
    }
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

DualChannelBoundResult computeDualChannelBoundCertified(
    const DualChannelBoundRequest& request, double rank_tolerance,
    std::uint64_t parent_proof_identity,
    DualChannelNumericalProofV1* proof) {
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
  std::vector<SymmetricPsdCertificate> channel_certificates;
  for (const auto& channel : request.channels) {
    if (!(channel.weight > 0.0)) {
      out.reason = "channel weights must be strictly positive";
      return out;
    }
    if (channel.gram.rows() != dimension || channel.gram.cols() != dimension) {
      out.reason = "channel Gram does not match the protected parameter space";
      return out;
    }
    const SymmetricPsdCertificate gram_certificate = certifySymmetricPsd(
        channel.gram, rank_tolerance, parent_proof_identity);
    if (!gram_certificate.valid) {
      out.reason = "channel " + channel.detector_id +
          " has no symmetric PSD certificate: " + gram_certificate.reason;
      return out;
    }
    if (gram_certificate.rank == 0) {
      // A channel with no certified fault-response rank must not fabricate a
      // Lambda from solver roundoff.  Dense and matrix-free paths therefore
      // make the same discrete decision (or jointly fail closed).
      continue;
    }
    if (channel.dof <= 0) {
      // No degrees of freedom: likewise no threshold can be assigned.
      continue;
    }
    usable.push_back(&channel);
    channel_certificates.push_back(gram_certificate);
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
  for (std::size_t channel_index = 0; channel_index < usable.size();
       ++channel_index) {
    const auto* channel = usable[channel_index];
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
    scaled.push_back(scale * scale *
                     channel_certificates[channel_index].symmetric_matrix);
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

  // Rank, PSD, ker(W) and protected slopes are one decision.  In particular,
  // an SVD of an indefinite W is not accepted as a covariance proof.
  const GramResponseCertificate w_certificate =
      certifyGramAndProtectedResponse(w, request.protected_response,
                                      rank_tolerance,
                                      parent_proof_identity);
  if (!w_certificate.valid) {
    out.reason = "W_h numerical certificate failed: " + w_certificate.reason;
    return out;
  }
  out.w_rank = static_cast<double>(w_certificate.gram.rank);
  out.w_kernel_covered =
      w_certificate.nullspace_class != GramNullspaceClass::Dangerous;
  if (!out.w_kernel_covered) {
    out.reason = "ker W_h is not covered by ker G_h";
    return out;
  }
  out.axis_bound_m = w_certificate.protected_slopes;
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
    // The externally compared formula identity is path-independent: dense and
    // matrix-free evaluation of the same frozen candidate must name the same
    // bound.  The path/result-specific proof remains available separately in
    // `numerical_proof_identity`; hashing it here would make equivalent routes
    // spuriously disagree.
    hashMatrix(&hash, out.w_matrix);
    hashMatrix(&hash, request.protected_response);
    hashMatrix(&hash, out.axis_bound_m);
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
  if (proof) {
    proof->request = request;
    proof->rank_tolerance = rank_tolerance;
    proof->parent_proof_identity = parent_proof_identity;
    proof->w_matrix = w;
    proof->w_eigenvalues = w_certificate.gram.eigenvalues;
    proof->w_eigenvectors = w_certificate.gram.eigenvectors;
    proof->w_eigenvalue_errors = w_certificate.gram.eigenvalue_errors;
    proof->served_result = out;
    std::uint64_t identity = w_certificate.proof_identity;
    hashBytes(&identity, &proof->schema_version, sizeof(proof->schema_version));
    hashBytes(&identity, &rank_tolerance, sizeof(rank_tolerance));
    hashBytes(&identity, &parent_proof_identity,
              sizeof(parent_proof_identity));
    hashMatrix(&identity, request.protected_response);
    hashBytes(&identity, &request.p_md, sizeof(request.p_md));
    hashMatrix(&identity, request.k_axis);
    hashMatrix(&identity, request.position_rho_m);
    for (const auto& channel : request.channels) {
      hashBytes(&identity, channel.detector_id.data(), channel.detector_id.size());
      hashBytes(&identity, &channel.dof, sizeof(channel.dof));
      hashBytes(&identity, &channel.threshold, sizeof(channel.threshold));
      hashBytes(&identity, &channel.actual_threshold,
                sizeof(channel.actual_threshold));
      hashBytes(&identity, &channel.weight, sizeof(channel.weight));
      hashMatrix(&identity, channel.gram);
    }
    hashMatrix(&identity, out.w_matrix);
    hashMatrix(&identity, out.axis_bound_m);
    hashBytes(&identity, out.certificate_id.data(), out.certificate_id.size());
    proof->proof_identity = identity;
  }
  return out;
}

DualChannelBoundResult computeDualChannelBound(
    const DualChannelBoundRequest& request) {
  return computeDualChannelBoundCertified(request, 1e-10, 0, nullptr);
}

bool validateDualChannelNumericalProof(
    const DualChannelNumericalProofV1& proof, std::string* reason) {
  DualChannelNumericalProofV1 rebuilt;
  const DualChannelBoundResult result = computeDualChannelBoundCertified(
      proof.request, proof.rank_tolerance, proof.parent_proof_identity,
      &rebuilt);
  const auto same = [](const auto& left, const auto& right) {
    return left.rows() == right.rows() && left.cols() == right.cols() &&
        (left.array() == right.array()).all();
  };
  const bool valid = proof.schema_version == 1 && result.valid &&
      proof.proof_identity != 0 && proof.proof_identity == rebuilt.proof_identity &&
      same(proof.w_matrix, rebuilt.w_matrix) &&
      same(proof.w_eigenvalues, rebuilt.w_eigenvalues) &&
      same(proof.w_eigenvectors, rebuilt.w_eigenvectors) &&
      same(proof.w_eigenvalue_errors, rebuilt.w_eigenvalue_errors) &&
      proof.served_result.valid == result.valid &&
      proof.served_result.certificate_id == result.certificate_id &&
      same(proof.served_result.axis_bound_m, result.axis_bound_m) &&
      same(proof.served_result.w_matrix, result.w_matrix) &&
      proof.served_result.channel_lambda == result.channel_lambda &&
      proof.served_result.channel_weight == result.channel_weight &&
      proof.served_result.channel_lambda_gap == result.channel_lambda_gap &&
      proof.served_result.channel_dof == result.channel_dof &&
      proof.served_result.channel_threshold == result.channel_threshold &&
      proof.served_result.w_rank == result.w_rank &&
      proof.served_result.w_kernel_covered == result.w_kernel_covered &&
      same(proof.served_result.position_rho_m, result.position_rho_m) &&
      proof.served_result.model_error_validated ==
          result.model_error_validated &&
      proof.served_result.model_error_source == result.model_error_source &&
      proof.served_result.reason == result.reason;
  if (!valid && reason) *reason = "dual-channel numerical proof mismatch";
  return valid;
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
  const double scale = rhs.norm();
  const double difference = (covered - rhs).norm();
  out.exactness_residual = scale > 0.0
      ? difference / scale
      : (difference == 0.0 ? 0.0
                           : std::numeric_limits<double>::infinity());
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
