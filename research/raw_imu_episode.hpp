#pragma once
// Physically isolated research library: never linked by the production DSO.
#include "uwb_imu_pl/estimation/epoch_transaction.hpp"
#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"

namespace uwb_imu_pl::research {
struct RawImuEpisode {
  std::string event_id;
  TimestampNs begin; // inclusive raw sample time
  TimestampNs end;   // exclusive raw sample time (not interval end)
  int axis = 0;     // accel xyz then gyro xyz, input body frame
};
struct RawImuResponse {
  Eigen::VectorXd raw;
  Eigen::VectorXd frozen_whitened;
  Eigen::VectorXd input_dependent_whitened;
  Eigen::MatrixXd covariance_derivative;
  double convergence_relative = 0.;
  std::size_t integrations = 4;
  std::size_t affected_samples = 0;
  std::size_t affected_trapezoids = 0;
};
gtsam::PreintegratedCombinedMeasurements integrateEpisode(
    const EpochTransaction&, const RawImuEpisode&, double amplitude);
Eigen::VectorXd episodeFactorError(const EpochTransaction&,
    const gtsam::PreintegratedCombinedMeasurements&);
RawImuResponse episodeResponse(const EpochTransaction&,
    const LinearizedFactorBlock&, const RawImuEpisode&, double epsilon=1e-5, bool skip_empty_support=false);
// Recreates the historical frozen-point material without pretending its
// recorded states are the latest transaction's Values.
EpochTransaction historicalEpisodeTransaction(const HistoricalEpochContext&);
}  // namespace uwb_imu_pl::research
