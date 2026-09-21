#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <tuple>

namespace uwb_imu_pl {
namespace {
std::uint64_t bits(double value) {
  std::uint64_t result = 0;
  static_assert(sizeof(result) == sizeof(value), "binary64 required");
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

// B3/C1-c key identity: (detector, dof, contract version, threshold, p_md)
// plus the envelope parameters and their version and the history-summary
// binding digest.  Nothing is keyed by a mode name.  C1-c fix: `envelope_kind`
// was declared on the struct but missing from this tuple (a silent aliasing
// axis); it now participates in the identity together with
// `history_summary_version`.
using NoncentralKey = std::tuple<std::uint32_t, std::uint32_t, std::uint64_t,
                                 std::uint64_t, std::uint64_t, std::uint64_t,
                                 std::uint64_t, std::uint64_t>;

NoncentralKey noncentralKey(int dof, double threshold, double p_md,
                            const StatisticalBoundKey& key) {
  return std::make_tuple(key.detector_id, static_cast<std::uint32_t>(dof),
                         key.contract_version, bits(threshold), bits(p_md),
                         key.envelope_kind, key.envelope_fingerprint,
                         key.history_summary_version);
}

struct Storage {
  std::mutex mutex;
  std::map<std::pair<int, std::uint64_t>, double> chi;
  std::map<NoncentralKey, double> noncentral;
  std::map<std::uint64_t, double> normal;
  StatisticalBoundsCacheStats stats;
};
Storage& storage() { static Storage value; return value; }

// Complement-quantile evaluation: P(X > x) = upper_tail.  Never forms 1 - tiny.
double normalUpperQuantile(double upper_tail) {
  const boost::math::normal distribution;
  return boost::math::quantile(boost::math::complement(distribution,
                                                       upper_tail));
}

double chiSquaredUpperQuantile(int dof, double upper_tail) {
  const boost::math::chi_squared distribution(dof);
  return boost::math::quantile(boost::math::complement(distribution,
                                                       upper_tail));
}

double noncentralMissProbability(int dof, double noncentrality,
                                 double threshold) {
  return boost::math::cdf(
      boost::math::non_central_chi_squared(dof, noncentrality), threshold);
}
}  // namespace

double StatisticalBoundsCache::chiSquaredThreshold(int dof, double p_fa) {
  if (dof <= 0 || !std::isfinite(p_fa) || p_fa <= 0.0 || p_fa >= 1.0) {
    ++storage().stats.invalid_inputs;
    return std::numeric_limits<double>::infinity();
  }
  Storage& data = storage();
  const auto key = std::make_pair(dof, bits(p_fa));
  std::lock_guard<std::mutex> lock(data.mutex);
  const auto found = data.chi.find(key);
  if (found != data.chi.end()) { ++data.stats.hits; return found->second; }
  // (1 - p_fa) quantile through the complement: tiny p_fa cannot collapse into
  // quantile(1.0) = +inf.
  const double value = chiSquaredUpperQuantile(dof, p_fa);
  data.chi.emplace(key, value); ++data.stats.misses;
  return value;
}

double StatisticalBoundsCache::noncentralityBoundary(
    int dof, double threshold, double p_md) {
  const NoncentralityBoundaryResult verified =
      noncentralityBoundaryVerified(dof, threshold, p_md);
  if (!verified.valid) {
    throw std::invalid_argument("invalid noncentrality boundary inputs: " +
                                verified.reason);
  }
  return verified.value;
}

NoncentralityBoundaryResult StatisticalBoundsCache::noncentralityBoundaryVerified(
    int dof, double squared_threshold, double p_md,
    const StatisticalBoundKey& key) {
  NoncentralityBoundaryResult result;
  if (dof <= 0) {
    result.reason = "zero or negative degrees of freedom";
    ++storage().stats.invalid_inputs;
    return result;
  }
  if (!std::isfinite(squared_threshold) || squared_threshold <= 0.0) {
    result.reason = "detector threshold is not a finite positive number";
    ++storage().stats.invalid_inputs;
    return result;
  }
  if (!std::isfinite(p_md) || p_md <= 0.0 || p_md >= 1.0) {
    result.reason = "p_md allocation outside (0, 1)";
    ++storage().stats.invalid_inputs;
    return result;
  }
  if (key.dof != 0 && key.dof != static_cast<std::uint32_t>(dof)) {
    result.reason = "cache key degrees of freedom disagree with the request";
    ++storage().stats.policy_mismatches;
    return result;
  }
  Storage& data = storage();
  const NoncentralKey cache_key =
      noncentralKey(dof, squared_threshold, p_md, key);
  {
    std::lock_guard<std::mutex> lock(data.mutex);
    const auto found = data.noncentral.find(cache_key);
    if (found != data.noncentral.end()) {
      ++data.stats.hits;
      result.value = found->second;
      result.valid = true;
      result.converged = true;
      result.residual =
          noncentralMissProbability(dof, found->second, squared_threshold) - p_md;
      return result;
    }
  }
  auto missed = [&](double noncentrality) {
    return noncentralMissProbability(dof, noncentrality, squared_threshold);
  };
  if (missed(0.0) < p_md) {
    {
      std::lock_guard<std::mutex> lock(data.mutex);
      data.noncentral.emplace(cache_key, 0.0);
      ++data.stats.misses;
    }
    result.value = 0.0;
    result.valid = true;
    result.converged = true;
    result.residual = missed(0.0) - p_md;
    result.reason = "detector boundary is already satisfied at zero "
                    "noncentrality";
    return result;
  }
  double lower = 0.0;
  double upper = 1.0;
  while (missed(upper) > p_md && upper < 1e12) upper *= 2.0;
  if (missed(upper) > p_md) {
    result.reason = "cannot bracket the noncentrality boundary";
    ++storage().stats.invalid_inputs;
    return result;
  }
  double previous_width = upper - lower;
  std::uint64_t iterations = 0;
  for (; iterations < 200; ++iterations) {
    const double middle = 0.5 * (lower + upper);
    if (missed(middle) > p_md) lower = middle; else upper = middle;
    const double width = upper - lower;
    const double scale = std::max(1.0, std::abs(upper));
    const bool converged = (width <= 1e-13 * scale) ||
        (width >= previous_width) ||
        (std::abs(missed(upper) - p_md) <= 1e-12 * std::max(1.0, p_md) &&
         width <= 1e-9 * scale);
    previous_width = width;
    result.bracket_width = width;
    if (converged) break;
  }
  // Conservative side: return the endpoint whose miss probability is <= p_md
  // (the larger noncentrality, hence the larger protection level).
  const double conservative = missed(upper) <= p_md ? upper : lower;
  result.value = conservative;
  result.iterations = iterations;
  result.residual = missed(conservative) - p_md;
  result.valid = std::isfinite(conservative) && result.residual <= 0.0;
  result.converged = result.valid &&
      (result.bracket_width <= 1e-9 * std::max(1.0, std::abs(conservative)) ||
       std::abs(result.residual) <= 1e-12 * std::max(1.0, p_md));
  if (!result.valid) {
    result.reason = "noncentrality solve did not produce a conservative side "
                    "endpoint";
    ++storage().stats.non_converged;
    return result;
  }
  if (!result.converged) ++storage().stats.non_converged;
  {
    std::lock_guard<std::mutex> lock(data.mutex);
    data.noncentral.emplace(cache_key, result.value);
    ++data.stats.misses;
  }
  return result;
}

double StatisticalBoundsCache::normalTwoSidedMultiplier(double tail) {
  bool valid = false;
  const double value = normalTwoSidedMultiplierVerified(tail, &valid);
  if (!valid) throw std::invalid_argument("invalid normal tail probability");
  return value;
}

double StatisticalBoundsCache::normalTwoSidedMultiplierVerified(
    double tail_probability, bool* valid) {
  if (valid) *valid = false;
  if (!std::isfinite(tail_probability) || tail_probability <= 0.0 ||
      tail_probability >= 1.0) {
    ++storage().stats.invalid_inputs;
    return std::numeric_limits<double>::quiet_NaN();
  }
  Storage& data = storage();
  const std::uint64_t key = bits(tail_probability);
  std::lock_guard<std::mutex> lock(data.mutex);
  const auto found = data.normal.find(key);
  double value = 0.0;
  if (found != data.normal.end()) {
    ++data.stats.hits;
    value = found->second;
  } else {
    // Two-sided multiplier: each tail carries tail/2 and is evaluated through
    // the complement, so a tiny tail never rounds 1 - tiny to 1.0.
    value = normalUpperQuantile(0.5 * tail_probability);
    data.normal.emplace(key, value); ++data.stats.misses;
  }
  if (valid) *valid = std::isfinite(value) && value > 0.0;
  return value;
}

StatisticalBoundsCacheStats StatisticalBoundsCache::stats() {
  Storage& data = storage();
  std::lock_guard<std::mutex> lock(data.mutex);
  auto result = data.stats;
  result.entries = data.chi.size() + data.noncentral.size() + data.normal.size();
  return result;
}

void StatisticalBoundsCache::clear() {
  Storage& data = storage();
  std::lock_guard<std::mutex> lock(data.mutex);
  data.chi.clear();
  data.noncentral.clear();
  data.normal.clear();
}

}  // namespace uwb_imu_pl
