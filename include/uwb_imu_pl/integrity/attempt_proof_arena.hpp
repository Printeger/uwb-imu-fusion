#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

namespace uwb_imu_pl {

namespace detail {
class AttemptProofArenaAccess;
}

// Explicit owner for proof sidecars produced by one integrity attempt.  The
// arena accepts writes until close(); leases keep its immutable contents alive
// for publication/final-packet construction without extending global state.
class AttemptProofLease;

class AttemptProofArena {
 public:
  struct State;
  AttemptProofArena();
  ~AttemptProofArena();
  AttemptProofArena(AttemptProofArena&&) noexcept;
  AttemptProofArena& operator=(AttemptProofArena&&) noexcept;
  AttemptProofArena(const AttemptProofArena&) = delete;
  AttemptProofArena& operator=(const AttemptProofArena&) = delete;

  AttemptProofLease lease() const;
  void close() noexcept;
  bool closed() const noexcept;
  std::uint64_t generation() const noexcept;
  std::size_t entryCount() const noexcept;

 private:
  explicit AttemptProofArena(std::shared_ptr<State> state);
  std::shared_ptr<State> state_;
  friend class AttemptProofLease;
  friend class detail::AttemptProofArenaAccess;
};

class AttemptProofLease {
 public:
  AttemptProofLease() = default;
  bool valid() const noexcept;
  bool closed() const noexcept;
  std::uint64_t generation() const noexcept;
  std::size_t entryCount() const noexcept;
  void reset() noexcept { state_.reset(); }

 private:
  explicit AttemptProofLease(std::shared_ptr<AttemptProofArena::State> state)
      : state_(std::move(state)) {}
  std::shared_ptr<AttemptProofArena::State> state_;
  friend class AttemptProofArena;
  friend class detail::AttemptProofArenaAccess;
};

namespace detail {

// Type-erased access is private to proof producers/consumers.  Kind values are
// stable internal protocol identifiers; a mismatched kind never casts.
class AttemptProofArenaAccess {
 public:
  static bool store(AttemptProofArena* arena, std::uint32_t kind,
                    std::uint64_t numeric_key, const char* string_key,
                    std::shared_ptr<const void> payload);
  static std::shared_ptr<const void> find(
      const AttemptProofArena& arena, std::uint32_t kind,
      std::uint64_t numeric_key, const char* string_key);
  static std::shared_ptr<const void> find(
      const AttemptProofLease& lease, std::uint32_t kind,
      std::uint64_t numeric_key, const char* string_key);
  static std::size_t countKind(const AttemptProofArena& arena,
                               std::uint32_t kind);
  static std::size_t countKind(const AttemptProofLease& lease,
                               std::uint32_t kind);
  // A consumable is a single-use handoff.  Registration and consumption are
  // serialized with close() on the owning arena.  Therefore either consume
  // wins and returns an immutable reader, or close wins and consumption is
  // rejected.  Ordinary readers already holding shared payloads are not
  // invalidated by close().
  static bool registerConsumable(AttemptProofArena* arena,
                                 std::uint32_t kind,
                                 std::uint64_t numeric_key,
                                 std::shared_ptr<const void> payload);
  static std::shared_ptr<const void> consumeConsumable(
      std::uint32_t kind, std::uint64_t numeric_key);
  static std::size_t liveConsumableCountForTesting();
};

}  // namespace detail
}  // namespace uwb_imu_pl
