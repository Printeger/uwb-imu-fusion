#include "uwb_imu_pl/estimation/incremental_estimator.hpp"

#include "uwb_imu_pl/factors/realtime_factors.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <boost/make_shared.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <chrono>
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>

namespace uwb_imu_pl {

class FixedLagBackend final : public gtsam::IncrementalFixedLagSmoother {
 public:
  FixedLagBackend(double lag, const gtsam::ISAM2Params& params)
      : gtsam::IncrementalFixedLagSmoother(lag, params) {}

  const gtsam::ISAM2& isam() const { return isam_; }
};

namespace {

gtsam::Key positionKey(std::size_t epoch) { return gtsam::Symbol('p', epoch); }
gtsam::Key velocityKey(std::size_t epoch) { return gtsam::Symbol('v', epoch); }
gtsam::Key poseKey(std::size_t epoch) { return gtsam::Symbol('x', epoch); }
gtsam::Key biasKey(std::size_t epoch) { return gtsam::Symbol('b', epoch); }

gtsam::Pose3 toGtsamPose(const NavigationState& state) {
  const Eigen::Quaterniond q = state.q_world_body.normalized();
  return gtsam::Pose3(gtsam::Rot3(q.toRotationMatrix()), state.position_world_m);
}

gtsam::imuBias::ConstantBias toGtsamBias(const NavigationState& state) {
  return {state.accel_bias_mps2, state.gyro_bias_radps};
}

Eigen::MatrixXd whitener(const Eigen::MatrixXd& covariance) {
  Eigen::LLT<Eigen::MatrixXd> llt(covariance);
  if (llt.info() != Eigen::Success) throw std::runtime_error("whitening failed");
  return llt.matrixL().solve(Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
}

double elapsedMs(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
}

gtsam::GaussianBayesNet currentCliqueClosure(
    const gtsam::ISAM2& isam2, const gtsam::KeyVector& keys) {
  gtsam::GaussianBayesNet bayes_net;
  std::vector<gtsam::ISAM2Clique::shared_ptr> closure;
  std::set<const gtsam::ISAM2Clique*> seen;
  for (const auto key : keys) {
    auto clique = isam2.clique(key);
    while (clique) {
      if (seen.insert(clique.get()).second) closure.push_back(clique);
      clique = clique->parent();
    }
  }
  auto depth = [](gtsam::ISAM2Clique::shared_ptr clique) {
    int value = 0;
    while ((clique = clique->parent())) ++value;
    return value;
  };
  // Bayes-net elimination order is deepest child to root. Back-substitution
  // consequently visits the returned container in reverse order.
  std::stable_sort(closure.begin(), closure.end(),
                   [&](const auto& left, const auto& right) {
                     return depth(left) > depth(right);
                   });
  for (const auto& clique : closure) {
    bayes_net.push_back(clique->conditional());
  }
  return bayes_net;
}

}  // namespace

IncrementalUwbEstimator::IncrementalUwbEstimator(const IncrementalConfig& config)
    : config_(config), isam2_([&config] {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = config.relinearize_threshold;
        params.relinearizeSkip = config.relinearize_skip;
        return params;
      }()) {}

void IncrementalUwbEstimator::initialize(
    TimestampNs timestamp, const Eigen::Vector3d& position_world_m,
    const Eigen::Vector3d& velocity_world_mps) {
  if (initialized_) throw std::logic_error("UWB incremental estimator already initialized");
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.addPrior(positionKey(0), gtsam::Point3(position_world_m),
                 gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  graph.addPrior(velocityKey(0), gtsam::Vector3(velocity_world_mps),
                 gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
  values.insert(positionKey(0), gtsam::Point3(position_world_m));
  values.insert(velocityKey(0), gtsam::Vector3(velocity_world_mps));
  isam2_.update(graph, values);
  for (const auto& factor : graph) full_graph_.push_back(factor);
  estimate_ = isam2_.calculateEstimate();
  timestamp_ = timestamp;
  initialized_ = true;
}

IntegrityOutput IncrementalUwbEstimator::update(const UwbBatch& batch) {
  if (!initialized_) throw std::logic_error("UWB incremental estimator is not initialized");
  if (!(timestamp_ < batch.timestamp)) throw std::invalid_argument("UWB batches must be strictly ordered");
  const double dt = batch.timestamp.seconds() - timestamp_.seconds();
  const gtsam::Point3 p0 = estimate_.at<gtsam::Point3>(positionKey(epoch_));
  const gtsam::Vector3 v0 = estimate_.at<gtsam::Vector3>(velocityKey(epoch_));
  ++epoch_;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.add(boost::make_shared<ConstantVelocityRegularizer>(
      positionKey(epoch_ - 1), velocityKey(epoch_ - 1), positionKey(epoch_),
      velocityKey(epoch_), dt, config_.smoothness_sigma_m));
  graph.add(boost::make_shared<UwbPositionBatchFactor>(positionKey(epoch_), batch));
  const gtsam::Point3 predicted_position(p0 + dt * v0);
  values.insert(positionKey(epoch_), predicted_position);
  values.insert(velocityKey(epoch_), v0);
  isam2_.update(graph, values);
  for (const auto& factor : graph) full_graph_.push_back(factor);
  estimate_ = isam2_.calculateEstimate();
  timestamp_ = batch.timestamp;

  gtsam::Marginals marginals(full_graph_, estimate_);
  const gtsam::KeyVector keys{positionKey(epoch_), velocityKey(epoch_)};
  const gtsam::JointMarginal joint = marginals.jointMarginalCovariance(keys);
  marginal_ = Eigen::MatrixXd::Zero(6, 6);
  marginal_.block<3, 3>(0, 0) = joint.at(keys[0], keys[0]);
  marginal_.block<3, 3>(0, 3) = joint.at(keys[0], keys[1]);
  marginal_.block<3, 3>(3, 0) = joint.at(keys[1], keys[0]);
  marginal_.block<3, 3>(3, 3) = joint.at(keys[1], keys[1]);
  const gtsam::Point3 position = estimate_.at<gtsam::Point3>(positionKey(epoch_));
  const Eigen::MatrixXd covariance = validatedUwbCovariance(batch);
  const Eigen::MatrixXd w = whitener(covariance);
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(batch.measurements.size(), 6);
  Eigen::VectorXd residual(batch.measurements.size());
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    const Eigen::Vector3d delta = position - batch.measurements[i].anchor_position_m;
    h.block<1, 3>(static_cast<Eigen::Index>(i), 0) = delta.transpose() / delta.norm();
    residual(static_cast<Eigen::Index>(i)) = batch.measurements[i].range_m - delta.norm();
  }
  WhitenedRowBlock measurement_rows;
  measurement_rows.factor_id = batch.measurements.front().factor_id;
  measurement_rows.role = RowRole::Measurement;
  measurement_rows.jacobian = w * h;
  measurement_rows.residual = w * residual;
  measurement_rows.column_indices = {0,1,2,3,4,5};
  measurement_rows.whitening_model_id = batch.covariance_model_id;
  for (const auto& m : batch.measurements) {
    measurement_rows.measurement_ids.push_back(m.id);
    measurement_rows.anchor_ids.push_back(m.anchor_id);
  }
  WhitenedRowBlock regularizer_rows;
  regularizer_rows.factor_id = FactorId(batch.id.value());
  regularizer_rows.role = RowRole::Regularizer;
  regularizer_rows.jacobian = Eigen::MatrixXd::Zero(6, 6);
  regularizer_rows.jacobian.topLeftCorner<3, 3>() =
      Eigen::Matrix3d::Identity() / config_.smoothness_sigma_m;
  regularizer_rows.jacobian.bottomRightCorner<3, 3>() =
      Eigen::Matrix3d::Identity() / config_.smoothness_sigma_m;
  regularizer_rows.residual.resize(6);
  regularizer_rows.residual.head<3>() =
      (p0 + dt * v0 - position) / config_.smoothness_sigma_m;
  regularizer_rows.residual.tail<3>() =
      (v0 - estimate_.at<gtsam::Vector3>(velocityKey(epoch_))) /
      config_.smoothness_sigma_m;
  regularizer_rows.column_indices = {0,1,2,3,4,5};
  regularizer_rows.whitening_model_id = "constant_velocity_regularizer";
  row_blocks_ = {std::move(regularizer_rows), std::move(measurement_rows)};
  diagnostics_.rows = static_cast<int>(batch.measurements.size());
  diagnostics_.columns = 3;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(w * h.leftCols(3));
  diagnostics_.rank = static_cast<int>((svd.singularValues().array() > 1e-10).count());
  diagnostics_.covariance_valid = marginal_.allFinite();
  diagnostics_.model_valid = diagnostics_.rank == 3 && diagnostics_.covariance_valid;
  if (!diagnostics_.model_valid) diagnostics_.reason = "UWB-only current geometry/marginal invalid";

  IntegrityOutput output;
  output.timestamp = timestamp_;
  output.state = currentState();
  output.batch_committed = true;
  output.protection_level.label = IntegrityLabel::ImplementedUnverified;
  output.protection_level.availability = Availability::Unavailable;
  output.protection_level.reason = "UWB-only bridge does not publish formal fusion PL";
  return output;
}

NavigationState IncrementalUwbEstimator::currentState() const {
  if (!initialized_) throw std::logic_error("UWB incremental estimator is not initialized");
  NavigationState state;
  state.id = StateId(epoch_);
  state.timestamp = timestamp_;
  state.position_world_m = estimate_.at<gtsam::Point3>(positionKey(epoch_));
  state.velocity_world_mps = estimate_.at<gtsam::Vector3>(velocityKey(epoch_));
  return state;
}

std::shared_ptr<const EstimationSnapshot> IncrementalUwbEstimator::snapshot() const {
  SnapshotCapabilities capabilities;
  capabilities.dense_snapshot = true;
  capabilities.sparse_solve = true;
  capabilities.factor_provenance = true;
  capabilities.consistent_relinearization = true;
  const LinearizationVersion version{epoch_ + 1, 1, 1, epoch_ + 1};
  auto rows = row_blocks_;
  for (auto& row : rows) row.version = version;
  Eigen::MatrixXd information;
  if (marginal_.size() > 0) information = marginal_.inverse();
  return std::make_shared<ImmutableEstimationSnapshot>(
      currentState(), version, diagnostics_, std::move(rows), capabilities,
      LinearizationConsistency::Strict, std::nullopt, marginal_, information);
}

IncrementalUwbImuEstimator::IncrementalUwbImuEstimator(
    const IntegrityConfig& config, const Eigen::Vector3d& lever_arm_body_m)
    : config_(config), lever_arm_body_m_(lever_arm_body_m),
      isam2_([&config] {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = config.incremental.relinearize_threshold;
        params.relinearizeSkip = config.incremental.relinearize_skip;
        // The newest navigation state is at the top of this temporal Bayes
        // tree. Stop the relinearization check once an unchanged separator is
        // reached instead of scanning every historical state each period.
        params.enablePartialRelinearizationCheck = true;
        // Range factors are strongly nonlinear in the coupled pose/velocity
        // state. Dogleg prevents the unbounded Gauss-Newton steps that can
        // corrupt the following IMU prediction and make an otherwise anchored
        // graph appear indefinite during Cholesky elimination.
        params.optimizationParams = gtsam::ISAM2DoglegParams();
        return params;
      }()) {
  if (config_.incremental.fixed_lag_epochs == 1) {
    throw std::invalid_argument("fixed_lag_epochs must be 0 or at least 2");
  }
  if (config_.incremental.fixed_lag_epochs > 0) {
    gtsam::ISAM2Params params = isam2_.params();
    params.findUnusedFactorSlots = true;
    fixed_lag_backend_.reset(new FixedLagBackend(
        static_cast<double>(config_.incremental.fixed_lag_epochs - 1), params));
  }
  auto params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(
      config_.imu.gravity_mps2);
  params->accelerometerCovariance = Eigen::Matrix3d::Identity() *
      std::pow(config_.imu.accelerometer_sigma, 2);
  params->gyroscopeCovariance = Eigen::Matrix3d::Identity() *
      std::pow(config_.imu.gyroscope_sigma, 2);
  params->integrationCovariance = Eigen::Matrix3d::Identity() * 1e-9;
  params->biasAccCovariance = Eigen::Matrix3d::Identity() *
      std::pow(config_.imu.accelerometer_bias_rw_sigma, 2);
  params->biasOmegaCovariance = Eigen::Matrix3d::Identity() *
      std::pow(config_.imu.gyroscope_bias_rw_sigma, 2);
  imu_params_ = params;
}

IncrementalUwbImuEstimator::~IncrementalUwbImuEstimator() = default;

const gtsam::ISAM2& IncrementalUwbImuEstimator::backendIsam() const {
  return fixed_lag_backend_ ? fixed_lag_backend_->isam() : isam2_;
}

const gtsam::NonlinearFactorGraph&
IncrementalUwbImuEstimator::activeGraph() const {
  return backendIsam().getFactorsUnsafe();
}

std::size_t IncrementalUwbImuEstimator::backendUpdate(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    std::size_t timestamp_epoch, bool add_timestamps) {
  if (!fixed_lag_backend_) {
    isam2_.update(graph, values);
    return 0;
  }

  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
  if (add_timestamps) {
    const double logical_timestamp = static_cast<double>(timestamp_epoch);
    timestamps.emplace(poseKey(timestamp_epoch), logical_timestamp);
    timestamps.emplace(velocityKey(timestamp_epoch), logical_timestamp);
    timestamps.emplace(biasKey(timestamp_epoch), logical_timestamp);
  }
  const std::size_t before = fixed_lag_backend_->timestamps().size();
  fixed_lag_backend_->update(graph, values, timestamps);
  const std::size_t after = fixed_lag_backend_->timestamps().size();
  const std::size_t added = add_timestamps ? 3 : 0;
  if (before + added < after || (before + added - after) % 3 != 0) {
    throw std::runtime_error("fixed-lag timestamp bookkeeping is inconsistent");
  }
  return (before + added - after) / 3;
}

std::uint32_t IncrementalUwbImuEstimator::retainedEpochs() const {
  if (!initialized_) return 0;
  if (!fixed_lag_backend_) {
    return static_cast<std::uint32_t>(std::min<std::size_t>(
        epoch_ + 1, std::numeric_limits<std::uint32_t>::max()));
  }
  return static_cast<std::uint32_t>(fixed_lag_backend_->timestamps().size() / 3);
}

std::size_t IncrementalUwbImuEstimator::oldestRetainedEpoch() const {
  if (!initialized_ || !fixed_lag_backend_) return 0;
  const std::uint32_t retained = retainedEpochs();
  return retained == 0 ? epoch_ : epoch_ + 1 - retained;
}

std::size_t IncrementalUwbImuEstimator::factorCount() const {
  std::size_t count = 0;
  for (const auto& factor : activeGraph()) count += factor ? 1 : 0;
  return count;
}

std::size_t IncrementalUwbImuEstimator::activeValueCount() const {
  return initialized_ ? backendIsam().getLinearizationPoint().size() : 0;
}

void IncrementalUwbImuEstimator::pruneRetainedMetadata() {
  if (!fixed_lag_backend_) return;
  const std::size_t oldest = oldestRetainedEpoch();
  while (!committed_uwb_batches_.empty() &&
         committed_uwb_batches_.front().first < oldest) {
    committed_uwb_batches_.pop_front();
  }
}

void IncrementalUwbImuEstimator::initialize(
    const NavigationState& initial_state,
    const Eigen::Matrix<double, 15, 1>& prior_sigmas) {
  if (initialized_) throw std::logic_error("UWB/IMU estimator already initialized");
  if (!prior_sigmas.allFinite() || (prior_sigmas.array() <= 0.0).any()) {
    throw std::invalid_argument("all initial prior sigmas must be finite and > 0");
  }
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  const gtsam::Pose3 pose = toGtsamPose(initial_state);
  const gtsam::Vector3 velocity = initial_state.velocity_world_mps;
  const gtsam::imuBias::ConstantBias bias = toGtsamBias(initial_state);
  graph.addPrior(poseKey(0), pose,
                 gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas.head<6>()));
  graph.addPrior(velocityKey(0), velocity,
                 gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas.segment<3>(6)));
  graph.addPrior(biasKey(0), bias,
                 gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas.tail<6>()));
  values.insert(poseKey(0), pose);
  values.insert(velocityKey(0), velocity);
  values.insert(biasKey(0), bias);
  const std::size_t marginalized = backendUpdate(graph, values, 0, true);
  if (marginalized != 0) {
    throw std::runtime_error("fixed-lag initialization marginalized state");
  }
  current_state_ = initial_state;
  current_state_.id = StateId(0);
  state_timestamp_ = initial_state.timestamp;
  queryCurrentState();
  preintegrated_.reset(new gtsam::PreintegratedCombinedMeasurements(imu_params_, bias));
  initialized_ = true;
  graph_version_ = 1;
  linpoint_version_ = 1;
}

void IncrementalUwbImuEstimator::ingestImu(const ImuMeasurement& measurement) {
  if (!initialized_) throw std::logic_error("UWB/IMU estimator is not initialized");
  if (!measurement.specific_force_mps2.allFinite() ||
      !measurement.angular_velocity_radps.allFinite()) {
    throw std::invalid_argument("IMU measurement must be finite");
  }
  if (last_received_imu_timestamp_ &&
      !(last_received_imu_timestamp_.value() < measurement.timestamp)) {
    throw std::invalid_argument("IMU timestamps must be strictly increasing");
  }
  if (measurement.timestamp < state_timestamp_) {
    throw std::invalid_argument("IMU timestamp precedes estimator state");
  }
  last_received_imu_timestamp_ = measurement.timestamp;
  if (measurement.timestamp == state_timestamp_) {
    if (imu_boundary_) {
      throw std::invalid_argument("duplicate IMU boundary timestamp");
    }
    imu_boundary_ = measurement;
    return;
  }
  imu_queue_.push_back(measurement);
}

void IncrementalUwbImuEstimator::validateUwbBatch(
    const UwbBatch& batch) const {
  if (!initialized_) throw std::logic_error("UWB/IMU estimator is not initialized");
  if (batch.measurements.empty()) {
    throw std::invalid_argument("UWB batch must not be empty");
  }
  if (!(state_timestamp_ < batch.timestamp)) {
    throw std::invalid_argument("UWB batch timestamp must increase");
  }
  std::set<std::uint64_t> configured;
  for (const auto& anchor : config_.anchors) configured.insert(anchor.id.value());
  std::set<std::uint64_t> measurement_ids;
  std::set<std::uint64_t> factor_ids;
  TimestampNs earliest = batch.measurements.front().timestamp;
  TimestampNs latest = earliest;
  for (const auto& measurement : batch.measurements) {
    if (!std::isfinite(measurement.range_m) || measurement.range_m <= 0.0 ||
        !std::isfinite(measurement.sigma_m) || measurement.sigma_m <= 0.0 ||
        !measurement.anchor_position_m.allFinite()) {
      throw std::invalid_argument("UWB batch contains invalid measurement values");
    }
    if (!configured.empty() && configured.count(measurement.anchor_id.value()) == 0) {
      throw std::invalid_argument("UWB batch references an unconfigured physical anchor");
    }
    if (!measurement_ids.insert(measurement.id.value()).second ||
        !factor_ids.insert(measurement.factor_id.value()).second) {
      throw std::invalid_argument("UWB batch measurement/factor IDs must be unique");
    }
    if (measurement.timestamp < earliest) earliest = measurement.timestamp;
    if (latest < measurement.timestamp) latest = measurement.timestamp;
    const double skew = std::abs(
        measurement.timestamp.seconds() - batch.timestamp.seconds());
    if (skew > config_.incremental.max_time_skew_s + 1e-12) {
      throw std::invalid_argument("UWB batch member exceeds max_time_skew_s");
    }
  }
  if (latest.seconds() - earliest.seconds() >
      config_.incremental.epoch_bin_s + 1e-12) {
    throw std::invalid_argument("UWB batch span exceeds epoch_bin_s");
  }
  (void)validatedUwbCovariance(batch);
}

void IncrementalUwbImuEstimator::predictTo(TimestampNs timestamp) {
  if (!initialized_) throw std::logic_error("UWB/IMU estimator is not initialized");
  if (pending_epoch_) throw std::logic_error("commit or reject pending UWB epoch first");
  if (!(state_timestamp_ < timestamp)) throw std::invalid_argument("prediction timestamp must increase");
  if (!imu_boundary_ || imu_boundary_->timestamp != state_timestamp_) {
    throw std::runtime_error("missing causal IMU sample at state boundary");
  }

  const auto start = std::chrono::steady_clock::now();
  const auto preintegration_start = start;
  const auto previous_bias = toGtsamBias(current_state_);
  gtsam::PreintegratedCombinedMeasurements proposed(imu_params_, previous_bias);
  TimestampNs previous_time = state_timestamp_;
  ImuMeasurement previous_measurement = *imu_boundary_;
  std::size_t consume_count = 0;
  for (const auto& measurement : imu_queue_) {
    if (timestamp < measurement.timestamp) break;
    if (!(previous_time < measurement.timestamp)) {
      throw std::runtime_error("duplicate or reversed IMU sample in prediction interval");
    }
    const double dt = measurement.timestamp.seconds() - previous_time.seconds();
    if (!std::isfinite(dt) || dt > config_.imu.max_gap_s + 1e-12) {
      throw std::runtime_error("IMU gap exceeds imu.max_gap_s");
    }
    proposed.integrateMeasurement(
        0.5 * (previous_measurement.specific_force_mps2 +
               measurement.specific_force_mps2),
        0.5 * (previous_measurement.angular_velocity_radps +
               measurement.angular_velocity_radps), dt);
    previous_time = measurement.timestamp;
    previous_measurement = measurement;
    ++consume_count;
  }
  if (previous_time < timestamp) {
    const double dt = timestamp.seconds() - previous_time.seconds();
    if (!std::isfinite(dt) || dt > config_.imu.max_gap_s + 1e-12) {
      throw std::runtime_error("causal IMU hold exceeds imu.max_gap_s");
    }
    proposed.integrateMeasurement(previous_measurement.specific_force_mps2,
                                  previous_measurement.angular_velocity_radps,
                                  dt);
  }

  const gtsam::Pose3 previous_pose = toGtsamPose(current_state_);
  const gtsam::Vector3 previous_velocity = current_state_.velocity_world_mps;
  const gtsam::NavState predicted = proposed.predict(
      gtsam::NavState(previous_pose, previous_velocity), previous_bias);
  last_imu_preintegration_ms_ = elapsedMs(preintegration_start);
  const std::size_t next_epoch = epoch_ + 1;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.add(gtsam::CombinedImuFactor(
      poseKey(epoch_), velocityKey(epoch_), poseKey(next_epoch),
      velocityKey(next_epoch), biasKey(epoch_), biasKey(next_epoch), proposed));
  values.insert(poseKey(next_epoch), predicted.pose());
  values.insert(velocityKey(next_epoch), predicted.velocity());
  values.insert(biasKey(next_epoch), previous_bias);
  const std::size_t marginalized =
      backendUpdate(graph, values, next_epoch, true);
  epoch_ = next_epoch;
  if (marginalized > 0) {
    marginalization_count_ += marginalized;
    ++ordering_version_;
    ++linpoint_version_;
  }
  pruneRetainedMetadata();
  state_timestamp_ = timestamp;
  const auto state_query_start = std::chrono::steady_clock::now();
  queryCurrentState();
  last_state_query_ms_ = elapsedMs(state_query_start);
  for (std::size_t i = 0; i < consume_count; ++i) imu_queue_.pop_front();
  previous_measurement.timestamp = timestamp;
  imu_boundary_ = previous_measurement;
  pending_epoch_ = true;
  current_uwb_committed_ = false;
  pending_batch_id_.reset();
  ++graph_version_;
  ++linpoint_version_;
  last_no_uwb_update_ms_ = elapsedMs(start);
}

void IncrementalUwbImuEstimator::queryCurrentState() {
  // GTSAM's single-key calculateEstimate<T>() still enters getDelta(), whose
  // wildfire update can visit the full Bayes tree. Solve only the current
  // cliques and their ancestor closure, then retract these three keys at the
  // stored linearization point. The closure is also the exact dependency set
  // used by the current-state marginal below.
  const gtsam::KeyVector keys{
      poseKey(epoch_), velocityKey(epoch_), biasKey(epoch_)};
  const gtsam::ISAM2& isam = backendIsam();
  const gtsam::GaussianBayesNet bayes_net =
      currentCliqueClosure(isam, keys);
  gtsam::VectorValues local_delta;
  for (std::size_t i = bayes_net.size(); i > 0; --i) {
    local_delta.insert(bayes_net[i - 1]->solve(local_delta));
  }
  const gtsam::Values& theta = isam.getLinearizationPoint();
  const gtsam::Pose3 pose = gtsam::traits<gtsam::Pose3>::Retract(
      theta.at<gtsam::Pose3>(keys[0]), local_delta.at(keys[0]));
  const gtsam::Vector3 velocity = gtsam::traits<gtsam::Vector3>::Retract(
      theta.at<gtsam::Vector3>(keys[1]), local_delta.at(keys[1]));
  const auto bias =
      gtsam::traits<gtsam::imuBias::ConstantBias>::Retract(
          theta.at<gtsam::imuBias::ConstantBias>(keys[2]),
          local_delta.at(keys[2]));
  current_state_.id = StateId(epoch_);
  current_state_.timestamp = state_timestamp_;
  current_state_.position_world_m = pose.translation();
  current_state_.q_world_body = Eigen::Quaterniond(pose.rotation().matrix());
  current_state_.velocity_world_mps = velocity;
  current_state_.accel_bias_mps2 = bias.accelerometer();
  current_state_.gyro_bias_radps = bias.gyroscope();
}

Eigen::Matrix<double, 15, 15>
IncrementalUwbImuEstimator::currentJointMarginal() const {
  const gtsam::KeyVector keys{poseKey(epoch_), velocityKey(epoch_), biasKey(epoch_)};
  const std::array<int, 3> offsets{0, 6, 9};
  const std::array<int, 3> dimensions{6, 3, 6};
  // Extract only the current cliques and their ancestor closure from the
  // already-factorized iSAM2 Bayes tree. Other child subtrees integrate out
  // and cannot affect this marginal. Triangular selected-inverse solves on
  // this small Bayes net are exact without scanning the historical graph.
  const gtsam::GaussianBayesNet bayes_net =
      currentCliqueClosure(backendIsam(), keys);

  gtsam::VectorValues zero_rhs;
  for (const auto& conditional : bayes_net) {
    for (auto item = conditional->begin(); item != conditional->end(); ++item) {
      if (!zero_rhs.exists(*item)) {
        zero_rhs.insert(*item, Eigen::VectorXd::Zero(conditional->getDim(item)));
      }
    }
  }
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Zero();
  for (std::size_t column_key = 0; column_key < keys.size(); ++column_key) {
    for (int local_column = 0; local_column < dimensions[column_key];
         ++local_column) {
      gtsam::VectorValues rhs = zero_rhs;
      rhs.at(keys[column_key])(local_column) = 1.0;
      const gtsam::VectorValues intermediate =
          bayes_net.backSubstituteTranspose(rhs);
      const gtsam::VectorValues inverse_column =
          bayes_net.backSubstitute(intermediate);
      const int output_column = offsets[column_key] + local_column;
      for (std::size_t row_key = 0; row_key < keys.size(); ++row_key) {
        covariance.block(offsets[row_key], output_column,
                         dimensions[row_key], 1) =
            inverse_column.at(keys[row_key]);
      }
    }
  }
  covariance = 0.5 * (covariance + covariance.transpose());
  if (covariance.rows() != 15 || covariance.cols() != 15 ||
      !covariance.allFinite()) {
    throw std::runtime_error("invalid 15x15 current-state marginal");
  }
  return covariance;
}

CurrentStatePrior IncrementalUwbImuEstimator::queryCurrentPrior(
    TimestampNs timestamp) {
  const auto start = std::chrono::steady_clock::now();
  const Eigen::Matrix<double, 15, 15> covariance = currentJointMarginal();
  CurrentStatePrior prior;
  prior.mean = current_state_;
  prior.mean.timestamp = timestamp;
  prior.covariance = covariance;
  prior.excludes_current_uwb = !current_uwb_committed_;
  prior.version = {graph_version_, ordering_version_, 1, linpoint_version_};
  last_marginal_ms_ = elapsedMs(start);
  return prior;
}

std::shared_ptr<const EstimationSnapshot>
IncrementalUwbImuEstimator::preMeasurementSnapshot(const UwbBatch& batch) {
  const auto snapshot_start = std::chrono::steady_clock::now();
  if (!pending_epoch_ || batch.timestamp != state_timestamp_) {
    throw std::logic_error("pre-measurement snapshot requires matching predicted epoch");
  }
  CurrentStatePrior prior = queryCurrentPrior(batch.timestamp);
  pending_batch_id_ = batch.id;
  if (!prior.excludes_current_uwb) throw std::logic_error("current UWB already contaminates prior");
  const gtsam::Pose3 pose = toGtsamPose(current_state_);
  UwbPoseBatchFactor proposed(poseKey(epoch_), batch, lever_arm_body_m_);
  gtsam::Matrix raw_h;
  const Eigen::VectorXd factor_error = proposed.evaluateError(pose, raw_h);
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(raw_h.rows(), 15);
  h.leftCols(6) = raw_h;
  const Eigen::MatrixXd covariance = validatedUwbCovariance(batch);
  const Eigen::MatrixXd w = whitener(covariance);
  WhitenedRowBlock rows;
  rows.factor_id = batch.measurements.front().factor_id;
  rows.role = RowRole::Measurement;
  rows.jacobian = w * h;
  rows.residual = -w * factor_error;
  rows.covariance = covariance;
  rows.whitener = w;
  rows.jacobian_raw = h;
  rows.residual_raw = -factor_error;
  rows.whitening_model_id = batch.covariance_model_id;
  rows.version = prior.version;
  for (int i = 0; i < 15; ++i) rows.column_indices.push_back(i);
  for (const auto& measurement : batch.measurements) {
    rows.measurement_ids.push_back(measurement.id);
    rows.anchor_ids.push_back(measurement.anchor_id);
  }
  LinearizationDiagnostics diagnostics;
  diagnostics.rows = static_cast<int>(batch.measurements.size());
  diagnostics.columns = 15;
  Eigen::JacobiSVD<Eigen::Matrix<double, 15, 15>> svd(prior.covariance);
  const double smallest = svd.singularValues().tail<1>()(0);
  diagnostics.rank = static_cast<int>((svd.singularValues().array() >
      config_.snapshot.rank_tolerance).count());
  diagnostics.condition_number = smallest > 0.0 ?
      svd.singularValues()(0) / smallest : std::numeric_limits<double>::infinity();
  diagnostics.covariance_valid = diagnostics.rank == 15;
  diagnostics.model_valid = diagnostics.covariance_valid &&
      diagnostics.condition_number <= config_.snapshot.max_condition_number;
  if (!diagnostics.model_valid) diagnostics.reason = "pre-UWB prior marginal is ill-conditioned";
  diagnostics.linearization_step_norm = 0.0;
  SnapshotCapabilities capabilities;
  capabilities.dense_snapshot = true;
  capabilities.sparse_solve = true;
  capabilities.pre_measurement_prior = true;
  capabilities.factor_provenance = true;
  capabilities.consistent_relinearization = true;
  capabilities.fixed_lag = fixed_lag_backend_ != nullptr;
  capabilities.historical_fault_provenance = false;
  const Eigen::Matrix<double, 15, 15> information = prior.covariance.inverse();
  auto snapshot = std::make_shared<ImmutableEstimationSnapshot>(
      prior.mean, prior.version, diagnostics, std::vector<WhitenedRowBlock>{rows},
      capabilities, LinearizationConsistency::Strict, prior, prior.covariance,
      information);
  last_snapshot_extraction_ms_ = elapsedMs(snapshot_start) - last_marginal_ms_;
  return snapshot;
}

void IncrementalUwbImuEstimator::commitUwbBatch(const UwbBatch& batch) {
  if (!pending_epoch_ || batch.timestamp != state_timestamp_) {
    throw std::logic_error("commit requires matching pending epoch");
  }
  if (pending_batch_id_ && pending_batch_id_.value() != batch.id) {
    throw std::logic_error("commit batch ID differs from audited pending batch");
  }
  const auto start = std::chrono::steady_clock::now();
  gtsam::NonlinearFactorGraph graph;
  graph.add(boost::make_shared<UwbPoseBatchFactor>(
      poseKey(epoch_), batch, lever_arm_body_m_));
  const std::size_t marginalized =
      backendUpdate(graph, gtsam::Values(), epoch_, false);
  if (marginalized != 0) {
    throw std::runtime_error("UWB-only update unexpectedly marginalized state");
  }
  const auto state_query_start = std::chrono::steady_clock::now();
  queryCurrentState();
  last_state_query_ms_ = elapsedMs(state_query_start);
  pending_epoch_ = false;
  current_uwb_committed_ = true;
  committed_uwb_batches_.emplace_back(epoch_, batch.id);
  pending_batch_id_.reset();
  ++graph_version_;
  ++linpoint_version_;
  last_uwb_update_ms_ = elapsedMs(start);
}

void IncrementalUwbImuEstimator::rejectUwbBatch(
    const UwbBatch& batch, const std::string& reason) {
  (void)reason;
  if (!pending_epoch_ || batch.timestamp != state_timestamp_) {
    throw std::logic_error("reject requires matching pending epoch");
  }
  if (pending_batch_id_ && pending_batch_id_.value() != batch.id) {
    throw std::logic_error("reject batch ID differs from audited pending batch");
  }
  pending_epoch_ = false;
  current_uwb_committed_ = false;
  pending_batch_id_.reset();
  last_uwb_update_ms_ = 0.0;
}

NavigationState IncrementalUwbImuEstimator::currentState() const {
  if (!initialized_) throw std::logic_error("UWB/IMU estimator is not initialized");
  return current_state_;
}

double IncrementalUwbImuEstimator::globalGraphResidualStatistic() const {
  if (!initialized_) return std::numeric_limits<double>::quiet_NaN();
  // GTSAM graph error is one half of the total whitened squared residual.
  // This is a graph-health diagnostic only; it is deliberately not assigned a
  // chi-square threshold or used by the formal current-fault PL.
  // Full Values are intentionally materialized only on this explicitly
  // enabled diagnostic path. They are never cached by the realtime estimator.
  const gtsam::Values full_estimate = backendIsam().calculateEstimate();
  return 2.0 * activeGraph().error(full_estimate);
}

EstimatorAudit IncrementalUwbImuEstimator::audit() const {
  if (!initialized_) throw std::logic_error("UWB/IMU estimator is not initialized");
  EstimatorAudit result;
  result.current_marginal = currentJointMarginal();
  result.epoch = epoch_;
  result.state_timestamp = state_timestamp_;
  result.version = {graph_version_, ordering_version_, 1, linpoint_version_};
  result.factor_count = factorCount();
  result.factor_slot_count = activeGraph().size();
  result.active_value_count = activeValueCount();
  for (const auto key : backendIsam().getLinearizationPoint().keys()) {
    const char prefix = gtsam::Symbol(key).chr();
    result.pose_value_count += prefix == 'x' ? 1 : 0;
    result.velocity_value_count += prefix == 'v' ? 1 : 0;
    result.bias_value_count += prefix == 'b' ? 1 : 0;
  }
  for (const auto& factor : activeGraph()) {
    if (factor && dynamic_cast<const gtsam::LinearContainerFactor*>(
                      factor.get()) != nullptr) {
      ++result.boundary_prior_factor_count;
    }
  }
  result.timestamp_count = fixed_lag_backend_
      ? fixed_lag_backend_->timestamps().size() : activeValueCount();
  result.oldest_retained_epoch = oldestRetainedEpoch();
  result.retained_epochs = retainedEpochs();
  result.marginalization_count = marginalization_count_;
  result.fixed_lag_active = fixed_lag_backend_ != nullptr;
  result.historical_fault_provenance = false;
  result.committed_uwb_batch_ids.reserve(committed_uwb_batches_.size());
  for (const auto& committed : committed_uwb_batches_) {
    result.committed_uwb_batch_ids.push_back(committed.second);
  }
  result.pending_epoch = pending_epoch_;
  result.pending_batch_id = pending_batch_id_;
  return result;
}

std::optional<Eigen::Matrix<double, 15, 15>>
IncrementalUwbImuEstimator::methodBCandidatePrior(
    const Eigen::Matrix<double, 15, 15>& all_in_covariance,
    const Eigen::MatrixXd& current_uwb_jacobian,
    const Eigen::MatrixXd& current_uwb_covariance) const {
  if (!all_in_covariance.allFinite() || current_uwb_jacobian.cols() != 15 ||
      current_uwb_covariance.rows() != current_uwb_jacobian.rows() ||
      current_uwb_covariance.cols() != current_uwb_jacobian.rows()) return std::nullopt;
  Eigen::LDLT<Eigen::Matrix<double, 15, 15>> covariance_ldlt(all_in_covariance);
  Eigen::LDLT<Eigen::MatrixXd> measurement_ldlt(current_uwb_covariance);
  if (covariance_ldlt.info() != Eigen::Success ||
      measurement_ldlt.info() != Eigen::Success) return std::nullopt;
  const Eigen::Matrix<double, 15, 15> prior_information =
      covariance_ldlt.solve(Eigen::Matrix<double, 15, 15>::Identity()) -
      current_uwb_jacobian.transpose() *
      measurement_ldlt.solve(current_uwb_jacobian);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> eigen(prior_information);
  if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() <= 0.0) {
    return std::nullopt;
  }
  const double condition = eigen.eigenvalues().maxCoeff() /
      eigen.eigenvalues().minCoeff();
  if (!std::isfinite(condition) ||
      condition > config_.incremental.method_b_max_condition) return std::nullopt;
  Eigen::LDLT<Eigen::Matrix<double, 15, 15>> prior_ldlt(prior_information);
  return prior_ldlt.solve(Eigen::Matrix<double, 15, 15>::Identity());
}

std::optional<MethodBCandidatePrior>
IncrementalUwbImuEstimator::methodBCandidatePrior(
    const Eigen::Matrix<double, 15, 1>& all_in_mean,
    const Eigen::Matrix<double, 15, 15>& all_in_covariance,
    const Eigen::MatrixXd& current_uwb_jacobian,
    const Eigen::MatrixXd& current_uwb_covariance,
    const Eigen::VectorXd& current_uwb_linear_measurement) const {
  if (!all_in_mean.allFinite() ||
      current_uwb_linear_measurement.size() != current_uwb_jacobian.rows()) {
    return std::nullopt;
  }
  const auto covariance = methodBCandidatePrior(
      all_in_covariance, current_uwb_jacobian, current_uwb_covariance);
  if (!covariance) return std::nullopt;
  Eigen::LDLT<Eigen::Matrix<double, 15, 15>> all_in_ldlt(all_in_covariance);
  Eigen::LDLT<Eigen::MatrixXd> measurement_ldlt(current_uwb_covariance);
  if (all_in_ldlt.info() != Eigen::Success ||
      measurement_ldlt.info() != Eigen::Success) return std::nullopt;
  const Eigen::Matrix<double, 15, 15> all_in_information =
      all_in_ldlt.solve(Eigen::Matrix<double, 15, 15>::Identity());
  const Eigen::Matrix<double, 15, 1> prior_information_vector =
      all_in_information * all_in_mean - current_uwb_jacobian.transpose() *
      measurement_ldlt.solve(current_uwb_linear_measurement);
  MethodBCandidatePrior result;
  result.covariance = *covariance;
  result.mean = result.covariance * prior_information_vector;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> eigen(
      result.covariance.inverse());
  if (!result.mean.allFinite() || eigen.info() != Eigen::Success ||
      eigen.eigenvalues().minCoeff() <= 0.0) return std::nullopt;
  result.condition_number = eigen.eigenvalues().maxCoeff() /
      eigen.eigenvalues().minCoeff();
  return result;
}

}  // namespace uwb_imu_pl
