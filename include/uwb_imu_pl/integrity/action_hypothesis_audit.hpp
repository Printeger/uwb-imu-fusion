#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace uwb_imu_pl {

struct RawFactorBlockAuditV1 {
  std::uint64_t group_id = 0;
  int factor_kind = 0;
  int sensor = 0;
  int row_role = 0;
  Eigen::MatrixXd jacobian_raw;
  Eigen::VectorXd residual_raw;
  Eigen::MatrixXd covariance;
  Eigen::MatrixXd jacobian_whitened;
  Eigen::VectorXd residual_whitened;
  std::vector<int> window_column_indices;
};

struct RawRowOwnerAuditV1 {
  std::string row_id;
  std::uint64_t owner_group_id = 0;
  std::size_t row_in_group = 0;
  std::string covariance_placement;
};

struct ActionCandidateInputAuditV1 {
  std::uint64_t action_id = 0;
  std::vector<std::uint64_t> removed_group_ids;
  std::vector<RawFactorBlockAuditV1> added_blocks;
};

struct HistoryFaultColumnAuditV1 {
  int kind = 0;
  std::uint64_t source = 0;
  std::size_t epoch = 0;
};

// P0-07 additive audit protocol.  The production objects keep their existing
// layouts: the pipeline freezes this payload out-of-object from the exact
// candidate/PL objects used by FDE, and readers can only obtain an immutable
// copy bound to the returned IntegrityOutput identity.
struct ActionHypothesisProofAuditV1 {
  std::uint64_t schema_version = 1;
  std::uint64_t action_id = 0;
  std::uint64_t hypothesis_id = 0;
  std::uint64_t candidate_proof_identity = 0;
  std::uint64_t protection_proof_identity = 0;
  std::uint64_t hypothesis_proof_identity = 0;
  std::uint64_t candidate_row_identity = 0;
  int candidate_rows = 0;
  bool proof_available = false;
  std::string proof_unavailable_reason;
  Eigen::MatrixXd gram;
  Eigen::MatrixXd protected_response;
  Eigen::Vector3d protected_slopes = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  int nullspace_class = 0;
  Eigen::Vector3d pl_contribution_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double prior_probability_bound = 0.0;
  double p_md_allocation = 0.0;
  double hmi_allocation = 0.0;
  double hypothesis_tail = 0.0;
  bool candidate_valid = false;
  bool post_detector_passed = false;
  int candidate_disposition = 0;
  bool action_eligible = false;
  bool action_selected = false;
  std::string refusal;
};

struct ActionHypothesisProofSnapshotV1 {
  std::uint64_t schema_version = 1;
  std::uint64_t transaction_id = 0;
  std::uint64_t window_id = 0;
  std::uint64_t selected_action_id = 0;
  std::vector<std::uint64_t> plausible_hypothesis_ids;
  std::string decision_status;
  std::string decision_reason;
  Eigen::MatrixXd served_h;
  Eigen::VectorXd served_z;
  Eigen::MatrixXd base_information;
  Eigen::VectorXd base_information_rhs;
  Eigen::VectorXd base_state_increment;
  int served_rank = 0;
  int served_dof = 0;
  double served_statistic = std::numeric_limits<double>::infinity();
  std::vector<RawFactorBlockAuditV1> raw_blocks;
  std::vector<RawRowOwnerAuditV1> raw_row_owners;
  Eigen::MatrixXd protected_state_map;
  Eigen::MatrixXd history_response;
  Eigen::MatrixXd history_detector_response;
  Eigen::VectorXd history_d_perp;
  std::vector<HistoryFaultColumnAuditV1> history_columns;
  std::vector<ActionCandidateInputAuditV1> action_inputs;
  std::vector<ActionHypothesisProofAuditV1> records;
  std::uint64_t snapshot_identity = 0;
};

// The sole new public accessor.  It performs no candidate, detector or PL
// computation.  It only copies the immutable same-run snapshot, and refuses
// an output whose final selected action contradicts the frozen binding.
bool actionHypothesisProofAuditV1(
    const IntegrityOutput& output,
    ActionHypothesisProofSnapshotV1* snapshot);

// Audit capture is opt-in so normal production attempts keep their original
// early-return work and timing. The switch changes diagnostics only.
void setActionHypothesisProofAuditEnabledV1(bool enabled);
bool actionHypothesisProofAuditEnabledV1();

}  // namespace uwb_imu_pl
