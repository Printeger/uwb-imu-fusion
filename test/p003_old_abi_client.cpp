#include "uwb_imu_pl/common/types.hpp"
#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"
#include "uwb_imu_pl/estimation/rank_update_kernel.hpp"
#include "uwb_imu_pl/integrity/dual_channel_detector.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"
#include "uwb_imu_pl/integrity/publication_identity.hpp"

#include <cstddef>
#include <iostream>

#define ABI_LAYOUT(Type, Tail) \
  std::cout << #Type << ' ' << sizeof(uwb_imu_pl::Type) << ' ' \
            << offsetof(uwb_imu_pl::Type, Tail) << '\n'

int main() {
  ABI_LAYOUT(ProtectionLevelResult, detector_certificate_id);
  ABI_LAYOUT(PublicationDiagnostics, certificate_id);
  ABI_LAYOUT(FrozenWindowNumerics, reason);
  ABI_LAYOUT(CandidateDetectorCertificate, reason);
  ABI_LAYOUT(CandidateEvaluation, reason);
  ABI_LAYOUT(DualChannelBoundResult, model_error_source);
  ABI_LAYOUT(DualChannelBoundRequest, position_rho_source);
  ABI_LAYOUT(FrozenHypothesisPlEntry, bound_from_projected_path);
  ABI_LAYOUT(FrozenHypothesisNumerics, reason);
  ABI_LAYOUT(ProtectionLevelV2Result, reason);
  ABI_LAYOUT(PublicationIdentity, protected_output);

  Eigen::MatrixXd z = Eigen::MatrixXd::Ones(1, 1);
  Eigen::MatrixXd g = Eigen::MatrixXd::Ones(3, 1);
  double sigma = 0.0;
  double condition = 0.0;
  int rank = 0;
  Eigen::Vector3d residual;
  Eigen::Vector3d slopes;
  const int classification = uwb_imu_pl::classifyDetectionResponse(
      z, g, 1e-10, &sigma, &condition, &rank, &residual, &slopes);
  std::cout << "classify8 " << classification << ' ' << rank << ' '
            << sigma << ' ' << condition << '\n';
  return classification == 1 && rank == 1 ? 0 : 1;
}
