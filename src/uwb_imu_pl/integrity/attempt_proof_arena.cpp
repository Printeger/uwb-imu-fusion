#include "uwb_imu_pl/integrity/attempt_proof_arena.hpp"

#include <algorithm>
#include <atomic>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <tuple>

namespace uwb_imu_pl {
namespace {

using ArenaKey = std::tuple<std::uint32_t, std::uint64_t, std::string>;

std::atomic<std::uint64_t>& nextGeneration() {
  static std::atomic<std::uint64_t> value{1};
  return value;
}

struct WeakArenaIndex {
  std::mutex mutex;
  std::map<std::uint64_t, std::weak_ptr<AttemptProofArena::State>> entries;
  std::map<std::pair<std::uint32_t, std::uint64_t>,
           std::weak_ptr<AttemptProofArena::State>> consumables;
};

WeakArenaIndex& weakArenaIndex() {
  static WeakArenaIndex index;
  return index;
}

}  // namespace

struct AttemptProofArena::State {
  explicit State(std::uint64_t value) : generation(value) {}
  mutable std::mutex mutex;
  std::map<ArenaKey, std::shared_ptr<const void>> entries;
  std::set<std::pair<std::uint32_t, std::uint64_t>> consumables;
  const std::uint64_t generation;
  bool closed = false;
};

AttemptProofArena::AttemptProofArena()
    : state_(std::make_shared<State>(nextGeneration().fetch_add(1))) {
  if (state_->generation == 0) {
    state_ = std::make_shared<State>(nextGeneration().fetch_add(1));
  }
  auto& index = weakArenaIndex();
  std::lock_guard<std::mutex> lock(index.mutex);
  index.entries[state_->generation] = state_;
}

AttemptProofArena::AttemptProofArena(std::shared_ptr<State> state)
    : state_(std::move(state)) {}

AttemptProofArena::~AttemptProofArena() { close(); }
AttemptProofArena::AttemptProofArena(AttemptProofArena&&) noexcept = default;
AttemptProofArena& AttemptProofArena::operator=(AttemptProofArena&& other) noexcept {
  if (this == &other) return *this;
  close();
  state_ = std::move(other.state_);
  return *this;
}

AttemptProofLease AttemptProofArena::lease() const {
  return AttemptProofLease(state_);
}

void AttemptProofArena::close() noexcept {
  if (!state_) return;
  auto& index = weakArenaIndex();
  // The global token index is always locked before the arena.  Consumption
  // uses the same order, making close-vs-consume a single linearized choice.
  std::lock_guard<std::mutex> index_lock(index.mutex);
  std::lock_guard<std::mutex> state_lock(state_->mutex);
  if (state_->closed) return;
  state_->closed = true;
  for (const auto& key : state_->consumables) {
    const auto found = index.consumables.find(key);
    if (found != index.consumables.end()) {
      const auto owner = found->second.lock();
      if (!owner || owner.get() == state_.get()) {
        index.consumables.erase(found);
      }
    }
    state_->entries.erase({key.first, key.second, ""});
  }
  state_->consumables.clear();
  const auto found = index.entries.find(state_->generation);
  if (found != index.entries.end()) {
    const auto indexed = found->second.lock();
    if (!indexed || indexed.get() == state_.get()) index.entries.erase(found);
  }
}

bool AttemptProofArena::closed() const noexcept {
  if (!state_) return true;
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->closed;
}

std::uint64_t AttemptProofArena::generation() const noexcept {
  return state_ ? state_->generation : 0;
}

std::size_t AttemptProofArena::entryCount() const noexcept {
  if (!state_) return 0;
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->entries.size();
}

bool AttemptProofLease::valid() const noexcept { return !!state_; }
bool AttemptProofLease::closed() const noexcept {
  if (!state_) return true;
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->closed;
}
std::uint64_t AttemptProofLease::generation() const noexcept {
  return state_ ? state_->generation : 0;
}
std::size_t AttemptProofLease::entryCount() const noexcept {
  if (!state_) return 0;
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->entries.size();
}

bool detail::AttemptProofArenaAccess::store(
    AttemptProofArena* arena, std::uint32_t kind,
    std::uint64_t numeric_key, const char* string_key,
    std::shared_ptr<const void> payload) {
  if (!arena || !arena->state_ || !payload) return false;
  auto& state = *arena->state_;
  std::lock_guard<std::mutex> lock(state.mutex);
  if (state.closed) return false;
  state.entries[{kind, numeric_key, string_key ? string_key : ""}] =
      std::move(payload);
  return true;
}

bool detail::AttemptProofArenaAccess::storeNumericBatch(
    AttemptProofArena* arena, std::uint32_t kind,
    const std::vector<std::pair<std::uint64_t, std::shared_ptr<const void>>>& payloads) {
  if (!arena || !arena->state_) return false;
  for (const auto& value : payloads) if (!value.second) return false;
  auto& state = *arena->state_;
  std::lock_guard<std::mutex> lock(state.mutex);
  if (state.closed) return false;
  for (const auto& value : payloads)
    state.entries[{kind, value.first, ""}] = value.second;
  return true;
}

namespace {
std::shared_ptr<const void> findImpl(
    const std::shared_ptr<AttemptProofArena::State>& state,
    std::uint32_t kind, std::uint64_t numeric_key, const char* string_key) {
  if (!state) return {};
  std::lock_guard<std::mutex> lock(state->mutex);
  const auto found = state->entries.find(
      {kind, numeric_key, string_key ? string_key : ""});
  return found == state->entries.end() ? std::shared_ptr<const void>{}
                                       : found->second;
}
}  // namespace

std::shared_ptr<const void> detail::AttemptProofArenaAccess::find(
    const AttemptProofArena& arena, std::uint32_t kind,
    std::uint64_t numeric_key, const char* string_key) {
  return findImpl(arena.state_, kind, numeric_key, string_key);
}

std::shared_ptr<const void> detail::AttemptProofArenaAccess::find(
    const AttemptProofLease& lease, std::uint32_t kind,
    std::uint64_t numeric_key, const char* string_key) {
  return findImpl(lease.state_, kind, numeric_key, string_key);
}

std::size_t detail::AttemptProofArenaAccess::countKind(
    const AttemptProofArena& arena, std::uint32_t kind) {
  if (!arena.state_) return 0;
  std::lock_guard<std::mutex> lock(arena.state_->mutex);
  return static_cast<std::size_t>(std::count_if(
      arena.state_->entries.begin(), arena.state_->entries.end(),
      [kind](const auto& entry) { return std::get<0>(entry.first) == kind; }));
}

std::size_t detail::AttemptProofArenaAccess::countKind(
    const AttemptProofLease& lease, std::uint32_t kind) {
  if (!lease.state_) return 0;
  std::lock_guard<std::mutex> lock(lease.state_->mutex);
  return static_cast<std::size_t>(std::count_if(
      lease.state_->entries.begin(), lease.state_->entries.end(),
      [kind](const auto& entry) { return std::get<0>(entry.first) == kind; }));
}

bool detail::AttemptProofArenaAccess::registerConsumable(
    AttemptProofArena* arena, std::uint32_t kind,
    std::uint64_t numeric_key, std::shared_ptr<const void> payload) {
  if (!arena || !arena->state_ || !payload || numeric_key == 0) return false;
  auto& index = weakArenaIndex();
  std::lock_guard<std::mutex> index_lock(index.mutex);
  std::lock_guard<std::mutex> state_lock(arena->state_->mutex);
  if (arena->state_->closed) return false;
  const auto key = std::make_pair(kind, numeric_key);
  const auto existing = index.consumables.find(key);
  if (existing != index.consumables.end() && !existing->second.expired()) {
    return false;
  }
  arena->state_->entries[{kind, numeric_key, ""}] = std::move(payload);
  arena->state_->consumables.insert(key);
  index.consumables[key] = arena->state_;
  return true;
}

std::shared_ptr<const void> detail::AttemptProofArenaAccess::consumeConsumable(
    std::uint32_t kind, std::uint64_t numeric_key) {
  auto& index = weakArenaIndex();
  std::lock_guard<std::mutex> index_lock(index.mutex);
  const auto key = std::make_pair(kind, numeric_key);
  const auto indexed = index.consumables.find(key);
  if (indexed == index.consumables.end()) return {};
  const auto state = indexed->second.lock();
  if (!state) {
    index.consumables.erase(indexed);
    return {};
  }
  std::lock_guard<std::mutex> state_lock(state->mutex);
  if (state->closed) {
    index.consumables.erase(indexed);
    state->consumables.erase(key);
    state->entries.erase({kind, numeric_key, ""});
    return {};
  }
  const auto payload = state->entries.find({kind, numeric_key, ""});
  if (payload == state->entries.end()) {
    index.consumables.erase(indexed);
    state->consumables.erase(key);
    return {};
  }
  auto result = payload->second;
  state->entries.erase(payload);
  state->consumables.erase(key);
  index.consumables.erase(indexed);
  return result;
}

std::size_t detail::AttemptProofArenaAccess::liveConsumableCountForTesting() {
  auto& index = weakArenaIndex();
  std::lock_guard<std::mutex> lock(index.mutex);
  for (auto it = index.consumables.begin(); it != index.consumables.end();) {
    if (it->second.expired()) it = index.consumables.erase(it);
    else ++it;
  }
  return index.consumables.size();
}

}  // namespace uwb_imu_pl
