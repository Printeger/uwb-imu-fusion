#include "uwb_imu_pl/integrity/statistical_bounds_cache.hpp"

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/non_central_chi_squared.hpp>
#include <boost/math/distributions/normal.hpp>

#include <cmath>
#include <cstring>
#include <map>
#include <limits>
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
struct Storage {
  std::mutex mutex;
  std::map<std::pair<int, std::uint64_t>, double> chi;
  std::map<std::tuple<int, std::uint64_t, std::uint64_t>, double> noncentral;
  std::map<std::uint64_t, double> normal;
  StatisticalBoundsCacheStats stats;
};
Storage& storage() { static Storage value; return value; }
}  // namespace

double StatisticalBoundsCache::chiSquaredThreshold(int dof, double p_fa) {
  if (dof <= 0 || !std::isfinite(p_fa) || p_fa <= 0.0 || p_fa >= 1.0)
    return std::numeric_limits<double>::infinity();
  Storage& data = storage();
  const auto key = std::make_pair(dof, bits(p_fa));
  std::lock_guard<std::mutex> lock(data.mutex);
  const auto found = data.chi.find(key);
  if (found != data.chi.end()) { ++data.stats.hits; return found->second; }
  const double value = boost::math::quantile(
      boost::math::chi_squared(dof), 1.0 - p_fa);
  data.chi.emplace(key, value); ++data.stats.misses;
  return value;
}

double StatisticalBoundsCache::noncentralityBoundary(
    int dof, double threshold, double p_md) {
  if (dof <= 0 || !std::isfinite(threshold) || threshold <= 0.0 ||
      !std::isfinite(p_md) || p_md <= 0.0 || p_md >= 1.0)
    throw std::invalid_argument("invalid noncentrality boundary inputs");
  Storage& data = storage();
  const auto key = std::make_tuple(dof, bits(threshold), bits(p_md));
  std::lock_guard<std::mutex> lock(data.mutex);
  const auto found = data.noncentral.find(key);
  if (found != data.noncentral.end()) { ++data.stats.hits; return found->second; }
  auto missed = [&](double noncentrality) {
    return boost::math::cdf(
        boost::math::non_central_chi_squared(dof, noncentrality), threshold);
  };
  if (missed(0.0) < p_md) {
    data.noncentral.emplace(key, 0.0); ++data.stats.misses;
    return 0.0;
  }
  double lower = 0.0, upper = 1.0;
  while (missed(upper) > p_md && upper < 1e12) upper *= 2.0;
  if (upper >= 1e12 && missed(upper) > p_md)
    throw std::runtime_error("cannot bracket noncentrality boundary");
  for (int i = 0; i < 120; ++i) {
    const double middle = 0.5 * (lower + upper);
    if (missed(middle) > p_md) lower = middle; else upper = middle;
  }
  const double value = 0.5 * (lower + upper);
  data.noncentral.emplace(key, value); ++data.stats.misses;
  return value;
}

double StatisticalBoundsCache::normalTwoSidedMultiplier(double tail) {
  if (!std::isfinite(tail) || tail <= 0.0 || tail >= 1.0)
    throw std::invalid_argument("invalid normal tail probability");
  Storage& data = storage();
  const std::uint64_t key = bits(tail);
  std::lock_guard<std::mutex> lock(data.mutex);
  const auto found = data.normal.find(key);
  if (found != data.normal.end()) { ++data.stats.hits; return found->second; }
  const double value = boost::math::quantile(
      boost::math::normal(), 1.0 - 0.5 * tail);
  data.normal.emplace(key, value); ++data.stats.misses;
  return value;
}

StatisticalBoundsCacheStats StatisticalBoundsCache::stats() {
  Storage& data = storage();
  std::lock_guard<std::mutex> lock(data.mutex);
  auto result = data.stats;
  result.entries = data.chi.size() + data.noncentral.size() + data.normal.size();
  return result;
}

}  // namespace uwb_imu_pl
