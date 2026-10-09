#pragma once
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"

namespace uwb_imu_pl {
namespace detail {
// Producer-owned immutable arena proof only. A mutable output copy never
// carries successful validation state; public payload validators remain full.
bool readAndValidateScopedFrozenHypothesisProof(
    const FrozenHypothesisPlEntry& entry, const AttemptProofArena& arena,
    FrozenHypothesisPlProofV1* proof = nullptr, std::string* reason = nullptr,
    GramResponseCertificate* validated_response = nullptr);
// Returns the numerical work already performed by exhaustive validation.
// No successful-state cache, additional arena index or retained wrapper.
bool validateFrozenHypothesisWithResponse(
    const FrozenHypothesisPlProofV1& proof, std::string* reason,
    GramResponseCertificate* validated_response);
// Internal consumer: only after full validation and exact raw/G/Gram/policy
// matching. Rebinds proof identity to a different consumer root without SVD.
GramResponseCertificate rebindValidatedFactorResponse(
    const GramResponseCertificate& validated, const Eigen::MatrixXd& raw,
    const Eigen::MatrixXd& actual_gram, const Eigen::MatrixXd& protected_response,
    double rank_tolerance, double raw_factor_scale, std::uint64_t parent);
}  // namespace detail
}  // namespace uwb_imu_pl
