#pragma once

#include <mutex>
#include <memory>
#include <utility>

namespace uwb_imu_pl {
namespace detail {

inline bool sameImmutablePayloadOwner(
    const std::shared_ptr<const void>& payload,
    const std::shared_ptr<const void>& owner, const void* expected_address) {
  return payload && owner && payload.get() == expected_address &&
      !payload.owner_before(owner) && !owner.owner_before(payload);
}

// Internal runtime state, never a proof or a caller-supplied trust receipt.
// Its owner must bind it to one immutable payload before calling check().
class SuccessfulValidationMemo {
 public:
  template <class Validator>
  bool check(Validator&& validator) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (succeeded_) return true;
    // False is not cached. If the validator throws, the lock unwinds and the
    // next reader retries; no successful state can escape an exception.
    if (!std::forward<Validator>(validator)()) return false;
    succeeded_ = true;
    return true;
  }

 private:
  mutable std::mutex mutex_;
  mutable bool succeeded_ = false;
};

}  // namespace detail
}  // namespace uwb_imu_pl
