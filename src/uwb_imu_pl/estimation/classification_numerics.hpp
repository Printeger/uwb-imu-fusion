#pragma once
#include "uwb_imu_pl/estimation/square_root_context.hpp"

namespace uwb_imu_pl {
namespace detail {
// Private classifier numerical result only: proof identities stay zero and
// must never be stored or consumed as proof. Public certifiers always hash.
// This does not accept or trust a caller certificate/validation receipt.
GramResponseCertificate classificationNumericsWithoutProofIdentity(
    const Eigen::MatrixXd& raw_factor, const Eigen::MatrixXd& actual_gram,
    const Eigen::MatrixXd& protected_response, double rank_tolerance,
    double raw_factor_scale);
}  // namespace detail
}  // namespace uwb_imu_pl
