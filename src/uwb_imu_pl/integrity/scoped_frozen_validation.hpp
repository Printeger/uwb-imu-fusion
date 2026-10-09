#pragma once
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"

namespace uwb_imu_pl {
namespace detail {
// Producer-owned immutable arena proof only. A mutable output copy never
// carries successful validation state; public payload validators remain full.
bool readAndValidateScopedFrozenHypothesisProof(
    const FrozenHypothesisPlEntry& entry, const AttemptProofArena& arena,
    FrozenHypothesisPlProofV1* proof = nullptr, std::string* reason = nullptr);
}  // namespace detail
}  // namespace uwb_imu_pl
