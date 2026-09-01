#pragma once

#include "uwb_imu_pl/common/types.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace uwb_imu_pl {

struct LinearizationVersion {
  std::uint64_t graph_version = 0;
  std::uint64_t ordering_version = 0;
  std::uint64_t noise_model_version = 0;
  std::uint64_t linpoint_version = 0;
  bool operator==(const LinearizationVersion& other) const {
    return graph_version == other.graph_version &&
           ordering_version == other.ordering_version &&
           noise_model_version == other.noise_model_version &&
           linpoint_version == other.linpoint_version;
  }
};

struct WhitenedRowBlock {
  FactorId factor_id;
  std::vector<MeasurementId> measurement_ids;
  std::vector<AnchorId> anchor_ids;
  RowRole role = RowRole::Measurement;
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
  Eigen::MatrixXd covariance;
  Eigen::MatrixXd whitener;
  Eigen::MatrixXd jacobian_raw;
  Eigen::VectorXd residual_raw;
  std::vector<int> column_indices;
  int row_offset = 0;
  double robust_weight = 1.0;
  std::string whitening_model_id;
  LinearizationVersion version;
};

struct CurrentStatePrior {
  NavigationState mean;
  // Tangent ordering: Pose3(rotation, local translation), world velocity,
  // accelerometer bias, gyroscope bias.
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  bool excludes_current_uwb = false;
  LinearizationVersion version;
};

enum class PriorExtractionMethod {
  ExplicitDoubleUpdateA,
  LeaveCurrentOutDowndateB
};

struct SnapshotCapabilities {
  bool dense_snapshot = false;
  bool sparse_solve = false;
  bool pre_measurement_prior = false;
  bool factor_provenance = false;
  bool temporary_factor_removal = false;
  bool consistent_relinearization = false;
  bool fixed_lag = false;
  bool historical_fault_provenance = false;
};

class EstimationSnapshot {
 public:
  virtual ~EstimationSnapshot() = default;
  virtual const NavigationState& state() const = 0;
  virtual const LinearizationVersion& version() const = 0;
  virtual const LinearizationDiagnostics& diagnostics() const = 0;
  virtual const std::vector<WhitenedRowBlock>& rowBlocks() const = 0;
  virtual const SnapshotCapabilities& capabilities() const = 0;
  virtual LinearizationConsistency consistency() const = 0;
  virtual std::optional<CurrentStatePrior> currentPrior() const = 0;
  virtual Eigen::MatrixXd currentMarginal() const = 0;
  virtual Eigen::MatrixXd solveInformation(const Eigen::MatrixXd& rhs) const = 0;
};

class ImmutableEstimationSnapshot final : public EstimationSnapshot {
 public:
  ImmutableEstimationSnapshot(NavigationState state,
                              LinearizationVersion version,
                              LinearizationDiagnostics diagnostics,
                              std::vector<WhitenedRowBlock> rows,
                              SnapshotCapabilities capabilities,
                              LinearizationConsistency consistency,
                              std::optional<CurrentStatePrior> prior,
                              Eigen::MatrixXd marginal,
                              Eigen::MatrixXd information);

  const NavigationState& state() const override { return state_; }
  const LinearizationVersion& version() const override { return version_; }
  const LinearizationDiagnostics& diagnostics() const override { return diagnostics_; }
  const std::vector<WhitenedRowBlock>& rowBlocks() const override { return rows_; }
  const SnapshotCapabilities& capabilities() const override { return capabilities_; }
  LinearizationConsistency consistency() const override { return consistency_; }
  std::optional<CurrentStatePrior> currentPrior() const override { return prior_; }
  Eigen::MatrixXd currentMarginal() const override { return marginal_; }
  Eigen::MatrixXd solveInformation(const Eigen::MatrixXd& rhs) const override;

 private:
  NavigationState state_;
  LinearizationVersion version_;
  LinearizationDiagnostics diagnostics_;
  std::vector<WhitenedRowBlock> rows_;
  SnapshotCapabilities capabilities_;
  LinearizationConsistency consistency_;
  std::optional<CurrentStatePrior> prior_;
  Eigen::MatrixXd marginal_;
  Eigen::MatrixXd information_;
};

}  // namespace uwb_imu_pl
