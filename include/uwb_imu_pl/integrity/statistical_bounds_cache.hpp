#pragma once

#include <cstdint>
#include <string>

namespace uwb_imu_pl {

struct StatisticalBoundsCacheStats {
  std::uint64_t hits = 0;
  std::uint64_t misses = 0;
  std::uint64_t entries = 0;
  std::uint64_t invalid_inputs = 0;
  std::uint64_t non_converged = 0;
  std::uint64_t policy_mismatches = 0;
};

// B3/C1-c: cache identity.  The key carries the detector identity, the
// numerical contract version, the envelope parameters and their versions, and
// the history-summary binding digest, so a policy, envelope or summary change
// can never be served from a stale entry (and never cached by a human
// readable mode name).  Every field above participates in the stored key:
// there is no declared-but-unhashed axis.
struct StatisticalBoundKey {
  std::uint32_t detector_id = 0;
  std::uint32_t dof = 0;
  std::uint64_t contract_version = 1;
  std::uint64_t envelope_kind = 0;
  std::uint64_t envelope_fingerprint = 0;
  // C1-c: digest of the summary binding (linearization / whitening / mode
  // set / capacity; see `digestHistorySummaryVersion` in
  // history_fault_summary.hpp).  Zero means "no summary version bound"
  // (pre-C1-b default), which is exactly the current production state.
  std::uint64_t history_summary_version = 0;
};

struct NoncentralityBoundaryResult {
  double value = 0.0;  // conservative side: F(value) <= p_md
  bool valid = false;
  bool converged = false;
  double residual = 0.0;  // |F(value) - p_md|
  double bracket_width = 0.0;
  std::uint64_t iterations = 0;
  std::string reason;
};

class StatisticalBoundsCache {
 public:
  static double chiSquaredThreshold(int dof, double p_fa);
  static double noncentralityBoundary(int dof, double squared_threshold,
                                      double p_md);
  static double normalTwoSidedMultiplier(double tail_probability);

  // B3 hardened entry points: validated inputs, convergence-checked root
  // finding, complement-quantile evaluation for tiny tails, and a versioned
  // cache identity.
  static NoncentralityBoundaryResult noncentralityBoundaryVerified(
      int dof, double squared_threshold, double p_md,
      const StatisticalBoundKey& key = {});
  static double normalTwoSidedMultiplierVerified(double tail_probability,
                                                 bool* valid = nullptr);
  static StatisticalBoundsCacheStats stats();
  // Drops every cached entry (used by tests and by contract changes).
  static void clear();
};

}  // namespace uwb_imu_pl
