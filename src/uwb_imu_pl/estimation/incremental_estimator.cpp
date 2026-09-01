#include "uwb_imu_pl/estimation/incremental_estimator.hpp"

#include "uwb_imu_pl/factors/realtime_factors.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

#include <boost/make_shared.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace uwb_imu_pl {
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
        return params;
      }()) {
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

void IncrementalUwbImuEstimator::appendGraph(
    const gtsam::NonlinearFactorGraph& graph) {
  for (const auto& factor : graph) full_graph_.push_back(factor);
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
  isam2_.update(graph, values);
  appendGraph(graph);
  estimate_ = isam2_.calculateEstimate();
  state_timestamp_ = initial_state.timestamp;
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
  if (!imu_queue_.empty() && !(imu_queue_.back().timestamp < measurement.timestamp)) {
    throw std::invalid_argument("IMU timestamps must be strictly increasing");
  }
  if (measurement.timestamp < state_timestamp_ ||
      measurement.timestamp == state_timestamp_) return;
  imu_queue_.push_back(measurement);
}

void IncrementalUwbImuEstimator::predictTo(TimestampNs timestamp) {
  if (!initialized_) throw std::logic_error("UWB/IMU estimator is not initialized");
  if (pending_epoch_) throw std::logic_error("commit or reject pending UWB epoch first");
  if (!(state_timestamp_ < timestamp)) throw std::invalid_argument("prediction timestamp must increase");
  if (imu_queue_.empty()) throw std::runtime_error("no IMU samples available for prediction");

  const auto start = std::chrono::steady_clock::now();
  const auto previous_bias = estimate_.at<gtsam::imuBias::ConstantBias>(biasKey(epoch_));
  preintegrated_->resetIntegrationAndSetBias(previous_bias);
  TimestampNs previous_time = state_timestamp_;
  std::size_t integrated = 0;
  std::optional<ImuMeasurement> last_measurement;
  while (!imu_queue_.empty() && !(timestamp < imu_queue_.front().timestamp)) {
    const ImuMeasurement measurement = imu_queue_.front();
    imu_queue_.pop_front();
    if (measurement.timestamp < previous_time || measurement.timestamp == previous_time) continue;
    const double dt = measurement.timestamp.seconds() - previous_time.seconds();
    preintegrated_->integrateMeasurement(measurement.specific_force_mps2,
                                         measurement.angular_velocity_radps, dt);
    previous_time = measurement.timestamp;
    last_measurement = measurement;
    ++integrated;
  }
  if (integrated == 0 && !imu_queue_.empty()) {
    last_measurement = imu_queue_.front();
  }
  if (!last_measurement) throw std::runtime_error("no IMU sample in prediction interval");
  if (previous_time < timestamp) {
    const double dt = timestamp.seconds() - previous_time.seconds();
    preintegrated_->integrateMeasurement(last_measurement->specific_force_mps2,
                                         last_measurement->angular_velocity_radps, dt);
  }

  const gtsam::Pose3 previous_pose = estimate_.at<gtsam::Pose3>(poseKey(epoch_));
  const gtsam::Vector3 previous_velocity = estimate_.at<gtsam::Vector3>(velocityKey(epoch_));
  const gtsam::NavState predicted = preintegrated_->predict(
      gtsam::NavState(previous_pose, previous_velocity), previous_bias);
  ++epoch_;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.add(gtsam::CombinedImuFactor(
      poseKey(epoch_ - 1), velocityKey(epoch_ - 1), poseKey(epoch_),
      velocityKey(epoch_), biasKey(epoch_ - 1), biasKey(epoch_), *preintegrated_));
  values.insert(poseKey(epoch_), predicted.pose());
  values.insert(velocityKey(epoch_), predicted.velocity());
  values.insert(biasKey(epoch_), previous_bias);
  isam2_.update(graph, values);
  appendGraph(graph);
  estimate_ = isam2_.calculateEstimate();
  state_timestamp_ = timestamp;
  pending_epoch_ = true;
  current_uwb_committed_ = false;
  ++graph_version_;
  ++linpoint_version_;
  last_no_uwb_update_ms_ = elapsedMs(start);
}

NavigationState IncrementalUwbImuEstimator::navigationState(
    std::size_t epoch, TimestampNs timestamp) const {
  NavigationState state;
  state.id = StateId(epoch);
  state.timestamp = timestamp;
  const gtsam::Pose3 pose = estimate_.at<gtsam::Pose3>(poseKey(epoch));
  state.position_world_m = pose.translation();
  state.q_world_body = Eigen::Quaterniond(pose.rotation().matrix());
  state.velocity_world_mps = estimate_.at<gtsam::Vector3>(velocityKey(epoch));
  const auto bias = estimate_.at<gtsam::imuBias::ConstantBias>(biasKey(epoch));
  state.accel_bias_mps2 = bias.accelerometer();
  state.gyro_bias_radps = bias.gyroscope();
  return state;
}

CurrentStatePrior IncrementalUwbImuEstimator::queryCurrentPrior(
    TimestampNs timestamp) {
  const auto start = std::chrono::steady_clock::now();
  gtsam::Marginals marginals(full_graph_, estimate_);
  const gtsam::KeyVector keys{poseKey(epoch_), velocityKey(epoch_), biasKey(epoch_)};
  const gtsam::JointMarginal joint = marginals.jointMarginalCovariance(keys);
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Zero();
  const std::array<int, 3> offsets{0, 6, 9};
  const std::array<int, 3> dimensions{6, 3, 6};
  for (std::size_t row = 0; row < keys.size(); ++row) {
    for (std::size_t column = 0; column < keys.size(); ++column) {
      covariance.block(offsets[row], offsets[column], dimensions[row],
                       dimensions[column]) = joint.at(keys[row], keys[column]);
    }
  }
  if (covariance.rows() != 15 || covariance.cols() != 15 ||
      !covariance.allFinite()) {
    throw std::runtime_error("invalid 15x15 current-state marginal");
  }
  CurrentStatePrior prior;
  prior.mean = navigationState(epoch_, timestamp);
  prior.covariance = covariance;
  prior.excludes_current_uwb = !current_uwb_committed_;
  prior.version = {graph_version_, 1, 1, linpoint_version_};
  last_marginal_ms_ = elapsedMs(start);
  return prior;
}

std::shared_ptr<const EstimationSnapshot>
IncrementalUwbImuEstimator::preMeasurementSnapshot(const UwbBatch& batch) {
  if (!pending_epoch_ || batch.timestamp != state_timestamp_) {
    throw std::logic_error("pre-measurement snapshot requires matching predicted epoch");
  }
  CurrentStatePrior prior = queryCurrentPrior(batch.timestamp);
  if (!prior.excludes_current_uwb) throw std::logic_error("current UWB already contaminates prior");
  const gtsam::Pose3 pose = estimate_.at<gtsam::Pose3>(poseKey(epoch_));
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
  SnapshotCapabilities capabilities;
  capabilities.dense_snapshot = true;
  capabilities.sparse_solve = true;
  capabilities.pre_measurement_prior = true;
  capabilities.factor_provenance = true;
  capabilities.consistent_relinearization = true;
  capabilities.fixed_lag = false;
  capabilities.historical_fault_provenance = false;
  const Eigen::Matrix<double, 15, 15> information = prior.covariance.inverse();
  return std::make_shared<ImmutableEstimationSnapshot>(
      prior.mean, prior.version, diagnostics, std::vector<WhitenedRowBlock>{rows},
      capabilities, LinearizationConsistency::Strict, prior, prior.covariance,
      information);
}

void IncrementalUwbImuEstimator::commitUwbBatch(const UwbBatch& batch) {
  if (!pending_epoch_ || batch.timestamp != state_timestamp_) {
    throw std::logic_error("commit requires matching pending epoch");
  }
  const auto start = std::chrono::steady_clock::now();
  gtsam::NonlinearFactorGraph graph;
  graph.add(boost::make_shared<UwbPoseBatchFactor>(
      poseKey(epoch_), batch, lever_arm_body_m_));
  isam2_.update(graph, gtsam::Values());
  appendGraph(graph);
  estimate_ = isam2_.calculateEstimate();
  pending_epoch_ = false;
  current_uwb_committed_ = true;
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
  pending_epoch_ = false;
  current_uwb_committed_ = false;
  last_uwb_update_ms_ = 0.0;
}

NavigationState IncrementalUwbImuEstimator::currentState() const {
  if (!initialized_) throw std::logic_error("UWB/IMU estimator is not initialized");
  return navigationState(epoch_, state_timestamp_);
}

double IncrementalUwbImuEstimator::globalGraphResidualStatistic() const {
  if (!initialized_) return std::numeric_limits<double>::quiet_NaN();
  // GTSAM graph error is one half of the total whitened squared residual.
  // This is a graph-health diagnostic only; it is deliberately not assigned a
  // chi-square threshold or used by the formal current-fault PL.
  return 2.0 * full_graph_.error(estimate_);
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

}  // namespace uwb_imu_pl
