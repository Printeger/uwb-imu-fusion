#include "uwb_imu_pl/estimation/incremental_estimator.hpp"

#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include "uwb_imu_pl/factors/kinematic_bridge_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
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
#include <cstring>
#include <numeric>
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

void cacheHashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= bytes[i];
    *hash *= 1099511628211ULL;
  }
}
template <class T> void cacheHashScalar(std::uint64_t* hash, const T& value) {
  cacheHashBytes(hash, &value, sizeof(value));
}
template <class Derived>
void cacheHashMatrix(std::uint64_t* hash, const Eigen::MatrixBase<Derived>& value) {
  const Eigen::Index rows = value.rows(), cols = value.cols();
  cacheHashScalar(hash, rows); cacheHashScalar(hash, cols);
  for (Eigen::Index column = 0; column < cols; ++column)
    for (Eigen::Index row = 0; row < rows; ++row) {
      const double item = value(row, column);
      cacheHashScalar(hash, item);
    }
}
void cacheHashText(std::uint64_t* hash, const std::string& value) {
  cacheHashBytes(hash, value.data(), value.size());
  const unsigned char separator = 0xff;
  cacheHashBytes(hash, &separator, 1);
}
std::uint64_t factorBlockFingerprint(const PendingFactorGroup& group,
                                     const EpochTransaction& tx) {
  std::uint64_t hash = 1469598103934665603ULL;
  const auto id = group.id.value();
  cacheHashScalar(&hash, id); cacheHashScalar(&hash, group.kind);
  cacheHashScalar(&hash, group.sensor); cacheHashScalar(&hash, group.nominal);
  cacheHashText(&hash, group.noise_model_id); cacheHashText(&hash, group.model_id);
  cacheHashMatrix(&hash, group.raw_covariance);
  cacheHashScalar(&hash, tx.previous_epoch); cacheHashScalar(&hash, tx.proposed_epoch);
  cacheHashScalar(&hash, tx.base_version.graph_version);
  cacheHashScalar(&hash, tx.base_version.ordering_version);
  cacheHashScalar(&hash, tx.base_version.noise_model_version);
  cacheHashScalar(&hash, tx.base_version.linpoint_version);
  cacheHashMatrix(&hash, tx.previous_state.position_world_m);
  cacheHashMatrix(&hash, tx.previous_state.velocity_world_mps);
  cacheHashMatrix(&hash, tx.previous_state.accel_bias_mps2);
  cacheHashMatrix(&hash, tx.previous_state.gyro_bias_radps);
  cacheHashMatrix(&hash, tx.nominal_predicted_state.position_world_m);
  cacheHashMatrix(&hash, tx.nominal_predicted_state.velocity_world_mps);
  for (const auto key : group.keys) cacheHashScalar(&hash, key);
  for (const auto id_value : group.source_measurements) {
    const auto value = id_value.value(); cacheHashScalar(&hash, value);
  }
  for (const auto& factor : group.factors) {
    const auto pointer = reinterpret_cast<std::uintptr_t>(factor.get());
    cacheHashScalar(&hash, pointer);
  }
  return hash;
}

std::size_t blockBytes(const LinearizedFactorBlock& block) {
  return sizeof(block) + sizeof(double) * static_cast<std::size_t>(
      block.jacobian_raw.size() + block.residual_raw.size() +
      block.covariance.size() + block.whitener.size() +
      block.jacobian_whitened.size() + block.residual_whitened.size());
}

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

NavigationState stateFromValues(const gtsam::Values& values,
                                std::size_t epoch,
                                TimestampNs timestamp) {
  const auto pose = values.at<gtsam::Pose3>(poseKey(epoch));
  const auto velocity = values.at<gtsam::Vector3>(velocityKey(epoch));
  const auto bias = values.at<gtsam::imuBias::ConstantBias>(biasKey(epoch));
  NavigationState state;
  state.id = StateId(epoch);
  state.timestamp = timestamp;
  state.position_world_m = pose.translation();
  state.q_world_body = Eigen::Quaterniond(pose.rotation().matrix());
  state.velocity_world_mps = velocity;
  state.accel_bias_mps2 = bias.accelerometer();
  state.gyro_bias_radps = bias.gyroscope();
  return state;
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

gtsam::Values transactionValues(const EpochTransaction& tx) {
  if (tx.frozen_values) return *tx.frozen_values;
  gtsam::Values values;
  values.insert(poseKey(tx.previous_epoch), toGtsamPose(tx.previous_state));
  values.insert(velocityKey(tx.previous_epoch), tx.previous_state.velocity_world_mps);
  values.insert(biasKey(tx.previous_epoch), toGtsamBias(tx.previous_state));
  values.insert(poseKey(tx.proposed_epoch), toGtsamPose(tx.nominal_predicted_state));
  values.insert(velocityKey(tx.proposed_epoch), tx.nominal_predicted_state.velocity_world_mps);
  values.insert(biasKey(tx.proposed_epoch), toGtsamBias(tx.nominal_predicted_state));
  return values;
}

int keyDimension(gtsam::Key key) {
  const char type = gtsam::Symbol(key).chr();
  if (type == 'x' || type == 'b') return 6;
  if (type == 'v') return 3;
  throw std::logic_error("unknown navigation-state key type");
}

int transactionColumn(const EpochTransaction& tx, gtsam::Key key) {
  if (key == poseKey(tx.previous_epoch)) return 0;
  if (key == velocityKey(tx.previous_epoch)) return 6;
  if (key == biasKey(tx.previous_epoch)) return 9;
  if (key == poseKey(tx.proposed_epoch)) return 15;
  if (key == velocityKey(tx.proposed_epoch)) return 21;
  if (key == biasKey(tx.proposed_epoch)) return 24;
  throw std::logic_error("factor key is outside epoch transaction");
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> linearizeGroupDense(
    const PendingFactorGroup& group, const EpochTransaction& tx) {
  gtsam::Ordering ordering;
  std::set<gtsam::Key> seen;
  for (const auto key : group.keys) {
    if (seen.insert(key).second) ordering.push_back(key);
  }
  const auto gaussian = group.factors.linearize(transactionValues(tx));
  const auto local = gaussian->jacobian(ordering);
  Eigen::MatrixXd full = Eigen::MatrixXd::Zero(local.first.rows(), 30);
  int local_column = 0;
  for (const auto key : ordering) {
    const int dimension = keyDimension(key);
    full.block(0, transactionColumn(tx, key), local.first.rows(), dimension) =
        local.first.block(0, local_column, local.first.rows(), dimension);
    local_column += dimension;
  }
  return {full, local.second};
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
  if (marginal_.size() > 0) {
    Eigen::LDLT<Eigen::MatrixXd> solve(marginal_);
    if (solve.info() == Eigen::Success) {
      information = solve.solve(Eigen::MatrixXd::Identity(
          marginal_.rows(), marginal_.cols()));
    }
  }
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

IncrementalUwbImuEstimator::BackendUpdateAudit
IncrementalUwbImuEstimator::backendUpdate(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
    std::size_t timestamp_epoch, bool add_timestamps,
    const std::vector<std::size_t>& remove_factor_slots) {
  BackendUpdateAudit audit;
  audit.removed_factor_slots = remove_factor_slots;
  const gtsam::FactorIndices removals(remove_factor_slots.begin(),
                                      remove_factor_slots.end());
  std::set<gtsam::Key> keys_before;
  if (fixed_lag_backend_) {
    for (const auto& item : fixed_lag_backend_->timestamps()) {
      keys_before.insert(item.first);
    }
  }
  if (!fixed_lag_backend_) {
    const gtsam::ISAM2Result result = isam2_.update(graph, values, removals);
    audit.new_factor_slots.assign(result.newFactorsIndices.begin(),
                                  result.newFactorsIndices.end());
    ++backend_update_count_;
    return audit;
  }

  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
  if (add_timestamps) {
    const double logical_timestamp = static_cast<double>(timestamp_epoch);
    timestamps.emplace(poseKey(timestamp_epoch), logical_timestamp);
    timestamps.emplace(velocityKey(timestamp_epoch), logical_timestamp);
    timestamps.emplace(biasKey(timestamp_epoch), logical_timestamp);
  }
  const std::size_t before = fixed_lag_backend_->timestamps().size();
  fixed_lag_backend_->update(graph, values, timestamps, removals);
  const auto& isam_result = fixed_lag_backend_->getISAM2Result();
  audit.new_factor_slots.assign(isam_result.newFactorsIndices.begin(),
                                isam_result.newFactorsIndices.end());
  ++backend_update_count_;
  const std::size_t after = fixed_lag_backend_->timestamps().size();
  const std::size_t added = add_timestamps ? 3 : 0;
  if (before + added < after || (before + added - after) % 3 != 0) {
    throw std::runtime_error("fixed-lag timestamp bookkeeping is inconsistent");
  }
  audit.marginalized_epochs = (before + added - after) / 3;
  for (const auto key : keys_before) {
    if (fixed_lag_backend_->timestamps().count(key) == 0) {
      audit.marginalized_keys.push_back(key);
    }
  }
  const auto& active = activeGraph();
  for (std::size_t slot = 0; slot < active.size(); ++slot) {
    if (active[slot] &&
        dynamic_cast<const gtsam::LinearContainerFactor*>(active[slot].get())) {
      audit.boundary_factor_slots.push_back(slot);
    }
  }
  return audit;
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
  const std::size_t configured_history =
      static_cast<std::size_t>(config_.integrity_window.epochs) +
      static_cast<std::size_t>(config_.integrity_window.recovery_margin_epochs) + 1;
  std::size_t oldest = epoch_ > configured_history
      ? epoch_ - configured_history : 0;
  if (fixed_lag_backend_) oldest = std::max(oldest, oldestRetainedEpoch());
  while (!committed_uwb_batches_.empty() &&
         committed_uwb_batches_.front().first < oldest) {
    committed_uwb_batches_.pop_front();
  }
  while (!committed_epochs_.empty() &&
         committed_epochs_.front().transaction.proposed_epoch <= oldest) {
    committed_epochs_.pop_front();
  }
  for (auto it = state_history_.begin(); it != state_history_.end() &&
       it->first < oldest;) it = state_history_.erase(it);
  for (auto it = prefix_covariances_.begin();
       it != prefix_covariances_.end() && it->first < oldest;) {
    it = prefix_covariances_.erase(it);
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
  const auto update = backendUpdate(graph, values, 0, true);
  if (update.marginalized_epochs != 0) {
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
  factor_ledger_.recordInitialPriors(
      graph, update.new_factor_slots, initial_state.timestamp,
      {graph_version_, ordering_version_, 1, linpoint_version_});
  state_history_[0] = current_state_;
  prefix_covariances_[0] = currentJointMarginal();
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

EpochTransaction IncrementalUwbImuEstimator::prepareTransaction(
    TimestampNs timestamp, const UwbBatch* batch) {
  if (!initialized_) throw std::logic_error("UWB/IMU estimator is not initialized");
  if (backend_poisoned_) throw std::runtime_error("backend is fail-closed after a partial update exception");
  if (active_transaction_id_) throw std::logic_error("commit or discard pending epoch first");
  if (!(state_timestamp_ < timestamp)) throw std::invalid_argument("prediction timestamp must increase");
  if (!imu_boundary_ || imu_boundary_->timestamp != state_timestamp_) {
    throw std::runtime_error("missing causal IMU sample at state boundary");
  }
  if (!factor_block_cache_.empty()) {
    factor_block_cache_.clear();
    ++cache_audit_.invalidations;
    cache_audit_.last_invalidation_reason = "new frozen transaction/version";
    cache_audit_.factor_block_entries = 0;
    cache_audit_.factor_block_bytes = 0;
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
  EpochTransaction tx;
  tx.id = TransactionId(next_transaction_id_++);
  tx.base_graph_version = graph_version_;
  tx.base_version = {graph_version_, ordering_version_, 1, linpoint_version_};
  tx.previous_epoch = epoch_;
  tx.proposed_epoch = next_epoch;
  tx.begin = state_timestamp_;
  tx.end = timestamp;
  tx.previous_state = current_state_;
  tx.nominal_predicted_state = current_state_;
  tx.nominal_predicted_state.id = StateId(next_epoch);
  tx.nominal_predicted_state.timestamp = timestamp;
  tx.nominal_predicted_state.q_world_body =
      Eigen::Quaterniond(predicted.pose().rotation().matrix());
  tx.nominal_predicted_state.position_world_m = predicted.position();
  tx.nominal_predicted_state.velocity_world_mps = predicted.velocity();
  tx.cv_predicted_state = current_state_;
  tx.cv_predicted_state.id = StateId(next_epoch);
  tx.cv_predicted_state.timestamp = timestamp;
  const double prediction_dt = timestamp.seconds() - state_timestamp_.seconds();
  tx.cv_predicted_state.position_world_m +=
      current_state_.velocity_world_mps * prediction_dt;
  tx.preintegration = std::make_shared<const gtsam::PreintegratedCombinedMeasurements>(proposed);
  tx.raw_imu_slice.push_back(*imu_boundary_);
  for (std::size_t i = 0; i < consume_count; ++i) {
    tx.raw_imu_slice.push_back(imu_queue_[i]);
  }
  if (tx.raw_imu_slice.back().timestamp < timestamp) {
    ImuMeasurement held = tx.raw_imu_slice.back();
    held.timestamp = timestamp;
    tx.raw_imu_slice.push_back(held);
  }
  tx.imu_cursor.consume_count = consume_count;
  previous_measurement.timestamp = timestamp;
  tx.imu_cursor.new_boundary = previous_measurement;
  tx.imu_group.id = FactorGroupId(tx.id.value() * 1000 + 1);
  tx.imu_group.kind = FactorKind::CombinedImu;
  tx.imu_group.sensor = SensorType::Imu;
  tx.imu_group.keys = {poseKey(epoch_), velocityKey(epoch_), poseKey(next_epoch),
                       velocityKey(next_epoch), biasKey(epoch_), biasKey(next_epoch)};
  tx.imu_group.noise_model_id = "gtsam_preintegrated_combined_covariance_v1";
  tx.imu_group.model_id = "combined_imu_interval_constant_fault_map_v1";
  tx.imu_group.factors.add(gtsam::CombinedImuFactor(
      poseKey(epoch_), velocityKey(epoch_), poseKey(next_epoch),
      velocityKey(next_epoch), biasKey(epoch_), biasKey(next_epoch), proposed));
  tx.generic_bridge_group.id = FactorGroupId(tx.id.value() * 1000 + 3);
  tx.generic_bias_continuity_group.id =
      FactorGroupId(tx.id.value() * 1000 + 4);
  tx.generic_bridge_group = BridgeFactory().makeGeneric(tx, config_.bridge.generic);
  tx.generic_bias_continuity_group =
      BridgeFactory().makeBiasContinuity(tx, config_.bridge.generic);
  if (batch) attachUwbGroups(&tx, *batch);
  tx.ledger_version = factor_ledger_.version();
  tx.frozen_graph = std::make_shared<const gtsam::NonlinearFactorGraph>(
      activeGraph());
  gtsam::Values frozen = backendIsam().calculateEstimate();
  frozen.insert(poseKey(next_epoch), toGtsamPose(tx.cv_predicted_state));
  frozen.insert(velocityKey(next_epoch), tx.cv_predicted_state.velocity_world_mps);
  frozen.insert(biasKey(next_epoch), toGtsamBias(tx.cv_predicted_state));
  tx.frozen_values = std::make_shared<const gtsam::Values>(std::move(frozen));
  std::map<std::size_t, FactorGroupId> slot_groups;
  for (const auto& entry : factor_ledger_.entries()) {
    if (entry.lifecycle == FactorLifecycle::Active && entry.backend_slot) {
      slot_groups.emplace(*entry.backend_slot, entry.group_id);
    }
  }
  for (std::size_t slot = 0; slot < tx.frozen_graph->size(); ++slot) {
    if (!(*tx.frozen_graph)[slot]) continue;
    FrozenFactorSlot frozen_slot;
    frozen_slot.slot = slot;
    frozen_slot.factor = (*tx.frozen_graph)[slot];
    const auto owner = slot_groups.find(slot);
    if (owner != slot_groups.end()) frozen_slot.group_id = owner->second;
    tx.frozen_slots.push_back(std::move(frozen_slot));
  }
  const std::size_t context_intervals =
      static_cast<std::size_t>(config_.integrity_window.epochs) +
      static_cast<std::size_t>(config_.integrity_window.recovery_margin_epochs);
  tx.oldest_recoverable_epoch = tx.previous_epoch > context_intervals
      ? tx.previous_epoch - context_intervals : 0;
  if (fixed_lag_backend_) {
    tx.oldest_recoverable_epoch =
        std::max(tx.oldest_recoverable_epoch, oldestRetainedEpoch());
  }
  for (const auto& record : committed_epochs_) {
    const auto& committed = record.transaction;
    if (committed.proposed_epoch <= tx.oldest_recoverable_epoch) continue;
    HistoricalEpochContext context;
    context.previous_epoch = committed.previous_epoch;
    context.proposed_epoch = committed.proposed_epoch;
    context.begin = committed.begin;
    context.end = committed.end;
    context.raw_imu_slice = committed.raw_imu_slice;
    context.preintegration = committed.preintegration;
    context.uwb_batch = committed.uwb_batch;
    context.previous_state = stateFromValues(
        *tx.frozen_values, committed.previous_epoch,
        state_history_.at(committed.previous_epoch).timestamp);
    context.current_state = stateFromValues(
        *tx.frozen_values, committed.proposed_epoch,
        state_history_.at(committed.proposed_epoch).timestamp);
    context.groups.push_back(committed.imu_group);
    const std::uint64_t history_base =
        tx.id.value() * 1000000ULL + committed.proposed_epoch * 100ULL;
    std::uint64_t replacement_index = 0;
    FactorGroupId selected_uwb;
    const PendingFactorGroup* selected_uwb_group = nullptr;
    for (const auto& group : committed.uwb_groups) {
      if (std::find(record.selected_groups.begin(), record.selected_groups.end(),
                    group.id) != record.selected_groups.end()) {
        selected_uwb = group.id;
        selected_uwb_group = &group;
        context.groups.push_back(group);
      }
    }
    std::set<std::uint64_t> selected_measurements;
    std::set<std::uint64_t> selected_anchors;
    if (selected_uwb_group) {
      for (const auto id : selected_uwb_group->source_measurements) {
        selected_measurements.insert(id.value());
        const auto measurement = std::find_if(
            committed.uwb_batch.measurements.begin(),
            committed.uwb_batch.measurements.end(),
            [&](const UwbMeasurement& value) { return value.id == id; });
        if (measurement != committed.uwb_batch.measurements.end()) {
          selected_anchors.insert(measurement->anchor_id.value());
        }
      }
    }
    const Eigen::MatrixXd historical_covariance = selected_uwb_group
        ? validatedUwbCovariance(committed.uwb_batch) : Eigen::MatrixXd{};
    for (const auto excluded_anchor : selected_anchors) {
      UwbBatch retained = committed.uwb_batch;
      retained.measurements.clear();
      std::vector<Eigen::Index> retained_indices;
      for (std::size_t i = 0; i < committed.uwb_batch.measurements.size(); ++i) {
        const auto& measurement = committed.uwb_batch.measurements[i];
        if (selected_measurements.count(measurement.id.value()) &&
            measurement.anchor_id.value() != excluded_anchor) {
          retained.measurements.push_back(measurement);
          retained_indices.push_back(static_cast<Eigen::Index>(i));
        }
      }
      if (retained.measurements.empty()) continue;
      retained.covariance_m2.resize(retained_indices.size(), retained_indices.size());
      for (std::size_t row = 0; row < retained_indices.size(); ++row) {
        for (std::size_t column = 0; column < retained_indices.size(); ++column) {
          retained.covariance_m2(row, column) = historical_covariance(
              retained_indices[row], retained_indices[column]);
        }
      }
      retained.covariance_model_id = committed.uwb_batch.covariance_model_id +
          ":recovery_principal_covariance";
      PendingFactorGroup replacement;
      replacement.id = FactorGroupId(history_base + 10 + replacement_index++);
      replacement.kind = FactorKind::UwbBatch;
      replacement.sensor = SensorType::Uwb;
      replacement.keys = {poseKey(committed.proposed_epoch)};
      replacement.noise_model_id = retained.covariance_model_id;
      replacement.model_id = "historical_uwb_exclude_anchor_" +
          std::to_string(excluded_anchor);
      replacement.nominal = false;
      replacement.raw_covariance = retained.covariance_m2;
      replacement.excluded_fault_units = {FaultUnitId(excluded_anchor)};
      for (const auto& measurement : retained.measurements) {
        replacement.source_measurements.push_back(measurement.id);
        replacement.fault_units.push_back(FaultUnitId(measurement.anchor_id.value()));
        replacement.source_ids.push_back("uwb:" +
                                         std::to_string(measurement.anchor_id.value()));
      }
      replacement.factors.add(boost::make_shared<UwbPoseBatchFactor>(
          poseKey(committed.proposed_epoch), retained, lever_arm_body_m_));
      replacement.replaces_group = selected_uwb;
      replacement.recovery_epoch = next_epoch;
      context.groups.push_back(std::move(replacement));
    }
    PendingFactorGroup historical_bridge = committed.generic_bridge_group;
    historical_bridge.id = FactorGroupId(history_base + 80);
    historical_bridge.replaces_group = committed.imu_group.id;
    historical_bridge.recovery_epoch = next_epoch;
    context.groups.push_back(std::move(historical_bridge));
    PendingFactorGroup historical_bias = committed.generic_bias_continuity_group;
    historical_bias.id = FactorGroupId(history_base + 81);
    historical_bias.replaces_group = committed.imu_group.id;
    historical_bias.recovery_epoch = next_epoch;
    context.groups.push_back(std::move(historical_bias));
    if (committed.dynamics_bridge_group) {
      context.groups.push_back(*committed.dynamics_bridge_group);
    }
    context.selected_groups = record.selected_groups;
    for (auto& group : context.groups) group.recovery_epoch = next_epoch;
    tx.recoverable_history.push_back(std::move(context));
  }
  tx.history_recoverability =
      factor_ledger_.hasCompleteActiveProvenance(*tx.frozen_graph)
          ? HistoryRecoverability::Recoverable
          : HistoryRecoverability::MissingProvenance;
  active_transaction_id_ = tx.id;
  pending_epoch_ = true;
  current_uwb_committed_ = false;
  pending_batch_id_ = batch ? std::optional<BatchId>(batch->id) : std::nullopt;
  last_no_uwb_update_ms_ = elapsedMs(start);
  return tx;
}

void IncrementalUwbImuEstimator::attachUwbGroups(
    EpochTransaction* tx, const UwbBatch& batch) const {
  if (!tx || !tx->uwb_groups.empty()) throw std::logic_error("UWB group already attached");
  tx->uwb_batch = batch;
  PendingFactorGroup group;
  group.id = FactorGroupId(tx->id.value() * 1000 + 2);
  group.kind = FactorKind::UwbBatch;
  group.sensor = SensorType::Uwb;
  group.keys = {poseKey(tx->proposed_epoch)};
  group.noise_model_id = batch.covariance_model_id.empty()
      ? "per_measurement_diagonal" : batch.covariance_model_id;
  group.model_id = "physical_anchor_correlated_batch_v1";
  group.raw_covariance = validatedUwbCovariance(batch);
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    group.source_measurements.push_back(batch.measurements[i].id);
    group.fault_units.push_back(FaultUnitId(batch.measurements[i].anchor_id.value()));
    group.source_ids.push_back("uwb:" +
                               std::to_string(batch.measurements[i].anchor_id.value()));
  }
  group.factors.add(boost::make_shared<UwbPoseBatchFactor>(
      poseKey(tx->proposed_epoch), batch, lever_arm_body_m_));
  tx->uwb_groups.push_back(std::move(group));

  // A correlated UWB group is indivisible once whitened. Prepare one
  // retained-principal-covariance replacement for every physical anchor so
  // an exclusion remains an atomic factor replacement at commit time.
  std::set<std::uint64_t> anchors;
  for (const auto& measurement : batch.measurements) {
    anchors.insert(measurement.anchor_id.value());
  }
  std::uint64_t candidate_index = 0;
  for (const std::uint64_t excluded_anchor : anchors) {
    UwbBatch retained = batch;
    retained.measurements.clear();
    std::vector<Eigen::Index> retained_indices;
    for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
      if (batch.measurements[i].anchor_id.value() != excluded_anchor) {
        retained.measurements.push_back(batch.measurements[i]);
        retained_indices.push_back(static_cast<Eigen::Index>(i));
      }
    }
    if (retained.measurements.empty()) continue;
    const Eigen::MatrixXd full_covariance = validatedUwbCovariance(batch);
    retained.covariance_m2.resize(retained_indices.size(), retained_indices.size());
    for (std::size_t row = 0; row < retained_indices.size(); ++row) {
      for (std::size_t col = 0; col < retained_indices.size(); ++col) {
        retained.covariance_m2(row, col) =
            full_covariance(retained_indices[row], retained_indices[col]);
      }
    }
    retained.covariance_model_id = batch.covariance_model_id.empty()
        ? "retained_principal_covariance" :
          batch.covariance_model_id + ":retained_principal_covariance";
    PendingFactorGroup replacement;
    replacement.id = FactorGroupId(tx->id.value() * 1000 + 10 + candidate_index++);
    replacement.kind = FactorKind::UwbBatch;
    replacement.sensor = SensorType::Uwb;
    replacement.keys = {poseKey(tx->proposed_epoch)};
    replacement.noise_model_id = retained.covariance_model_id;
    replacement.model_id = "uwb_principal_covariance_exclude_anchor_" +
        std::to_string(excluded_anchor);
    replacement.nominal = false;
    replacement.raw_covariance = retained.covariance_m2;
    replacement.excluded_fault_units.push_back(FaultUnitId(excluded_anchor));
    replacement.replaces_group = tx->uwb_groups.front().id;
    for (const auto& measurement : retained.measurements) {
      replacement.source_measurements.push_back(measurement.id);
      replacement.fault_units.push_back(FaultUnitId(measurement.anchor_id.value()));
      replacement.source_ids.push_back(
          "uwb:" + std::to_string(measurement.anchor_id.value()));
    }
    replacement.factors.add(boost::make_shared<UwbPoseBatchFactor>(
        poseKey(tx->proposed_epoch), retained, lever_arm_body_m_));
    tx->uwb_groups.push_back(std::move(replacement));
  }
}

EpochTransaction IncrementalUwbImuEstimator::prepareEpoch(const UwbBatch& batch) {
  validateUwbBatch(batch);
  return prepareTransaction(batch.timestamp, &batch);
}

void IncrementalUwbImuEstimator::predictTo(TimestampNs timestamp) {
  adapter_transaction_ = prepareTransaction(timestamp, nullptr);
}

LinearizedFactorBlock IncrementalUwbImuEstimator::linearizePendingGroup(
    const PendingFactorGroup& group, const EpochTransaction& tx,
    const Eigen::MatrixXd& h, const Eigen::VectorXd& rhs) const {
  LinearizedFactorBlock block;
  block.group_id = group.id;
  block.kind = group.kind;
  block.sensor = group.sensor;
  block.role = group.kind == FactorKind::BoundaryPrior
      ? RowRole::TrustedPrior : RowRole::Measurement;
  block.jacobian_whitened = h;
  block.residual_whitened = rhs;
  block.fault_units = group.fault_units;
  block.whitening_model_id = group.noise_model_id;
  block.version = tx.base_version;
  block.window_column_indices.resize(h.cols());
  std::iota(block.window_column_indices.begin(),
            block.window_column_indices.end(), 0);
  if (group.kind == FactorKind::CombinedImu && tx.preintegration) {
    block.covariance = tx.preintegration->preintMeasCov();
  } else if (group.kind == FactorKind::UwbBatch &&
             group.raw_covariance.rows() == h.rows()) {
    block.covariance = group.raw_covariance;
  } else {
    block.covariance = Eigen::MatrixXd::Identity(h.rows(), h.rows());
  }
  block.whitener = whitener(block.covariance);
  block.jacobian_raw = block.whitener.triangularView<Eigen::Lower>().solve(h);
  block.residual_raw =
      block.whitener.triangularView<Eigen::Lower>().solve(rhs);
  return block;
}

CurrentStatePrior IncrementalUwbImuEstimator::pendingCurrentPrior(
    const EpochTransaction& tx) const {
  const Eigen::Matrix<double, 15, 15> previous_covariance = currentJointMarginal();
  Eigen::LLT<Eigen::Matrix<double, 15, 15>> prior_llt(previous_covariance);
  if (prior_llt.info() != Eigen::Success) {
    throw std::runtime_error("committed current marginal is not SPD");
  }
  Eigen::Matrix<double, 15, 15> prior_information =
      prior_llt.solve(Eigen::Matrix<double, 15, 15>::Identity());
  const auto imu = linearizeGroupDense(tx.imu_group, tx);
  Eigen::Matrix<double, 30, 30> information = imu.first.transpose() * imu.first;
  information.topLeftCorner<15, 15>() += prior_information;
  Eigen::LLT<Eigen::Matrix<double, 30, 30>> llt(information);
  if (llt.info() != Eigen::Success) {
    throw std::runtime_error("pending IMU prediction information is not SPD");
  }
  const Eigen::Matrix<double, 30, 30> covariance =
      llt.solve(Eigen::Matrix<double, 30, 30>::Identity());
  CurrentStatePrior prior;
  prior.mean = tx.nominal_predicted_state;
  prior.covariance = covariance.bottomRightCorner<15, 15>();
  prior.excludes_current_uwb = true;
  prior.version = tx.base_version;
  return prior;
}

LinearizedIntegrityWindow IncrementalUwbImuEstimator::buildIntegrityWindow(
    const EpochTransaction& tx, const IntegrityWindowRequest& request) const {
  if (!active_transaction_id_ || *active_transaction_id_ != tx.id ||
      tx.backend_mutated || tx.base_graph_version != graph_version_ ||
      tx.ledger_version != factor_ledger_.version() ||
      !(tx.base_version == LinearizationVersion{
          graph_version_, ordering_version_, 1, linpoint_version_})) {
    throw std::logic_error("integrity window requires the active frozen transaction");
  }
  LinearizedIntegrityWindow window;
  window.id = WindowId(tx.id.value());
  window.version = tx.base_version;
  const std::size_t interval_count = std::min<std::size_t>(
      request.epochs, tx.proposed_epoch);
  const std::size_t first_epoch = tx.proposed_epoch - interval_count;
  window.detector_first_epoch = first_epoch;
  window.recovery_first_epoch = tx.oldest_recoverable_epoch;
  const int total_columns = static_cast<int>((interval_count + 1) * 15);
  for (std::size_t epoch = first_epoch; epoch <= tx.proposed_epoch; ++epoch) {
    window.state_layout.push_back(StateLayoutEntry{
        epoch, {poseKey(epoch), velocityKey(epoch), biasKey(epoch)},
        static_cast<int>((epoch - first_epoch) * 15), 15,
        epoch == tx.proposed_epoch});
  }

  if (!tx.frozen_graph || !tx.frozen_values ||
      tx.ledger_version != factor_ledger_.version()) {
    window.reason = "transaction graph/values/ledger freeze is incomplete";
    return window;
  }

  std::set<std::uint64_t> explicit_group_ids;
  for (const auto& record : tx.recoverable_history) {
    if (record.proposed_epoch <= first_epoch ||
        record.proposed_epoch > tx.previous_epoch) continue;
    for (const auto group : record.selected_groups) {
      explicit_group_ids.insert(group.value());
    }
  }
  std::set<std::size_t> explicit_slots;
  bool slot_identity_valid = true;
  for (const auto group_value : explicit_group_ids) {
    const auto slots = factor_ledger_.activeSlots(FactorGroupId(group_value));
    if (slots.empty()) slot_identity_valid = false;
    explicit_slots.insert(slots.begin(), slots.end());
  }
  gtsam::NonlinearFactorGraph boundary_graph;
  for (const auto& frozen : tx.frozen_slots) {
    const bool pointer_ok = frozen.slot < tx.frozen_graph->size() &&
        (*tx.frozen_graph)[frozen.slot] && frozen.factor &&
        (*tx.frozen_graph)[frozen.slot].get() == frozen.factor.get() &&
        frozen.slot < activeGraph().size() && activeGraph()[frozen.slot] &&
        activeGraph()[frozen.slot].get() == frozen.factor.get();
    const bool explicit_block = explicit_slots.count(frozen.slot) != 0;
    FactorSlotAccounting accounting;
    accounting.slot = frozen.slot;
    accounting.group_id = frozen.group_id;
    accounting.explicit_window_block = explicit_block;
    accounting.boundary_input = !explicit_block;
    accounting.pointer_identity_valid = pointer_ok;
    window.slot_accounting.push_back(accounting);
    slot_identity_valid = slot_identity_valid && pointer_ok &&
        (accounting.explicit_window_block != accounting.boundary_input) &&
        frozen.group_id.has_value();
    if (!explicit_block) boundary_graph.push_back(frozen.factor);
  }
  window.capabilities.frozen_slot_identity_valid = slot_identity_valid;
  window.capabilities.every_active_factor_accounted_once = slot_identity_valid &&
      window.slot_accounting.size() == tx.frozen_slots.size();
  if (!slot_identity_valid) {
    window.reason = "active factor slot ownership/pointer identity is invalid";
    return window;
  }

  if (!boundary_graph.empty()) {
    std::set<gtsam::Key> window_keys;
    for (std::size_t epoch = first_epoch; epoch <= tx.previous_epoch; ++epoch) {
      window_keys.insert(poseKey(epoch));
      window_keys.insert(velocityKey(epoch));
      window_keys.insert(biasKey(epoch));
    }
    std::set<gtsam::Key> graph_keys;
    for (const auto& factor : boundary_graph) {
      graph_keys.insert(factor->keys().begin(), factor->keys().end());
    }
    gtsam::KeyVector outside_variables;
    gtsam::Ordering inside_ordering;
    for (const auto key : graph_keys) {
      if (window_keys.count(key) == 0) {
        outside_variables.push_back(key);
      }
    }
    for (std::size_t epoch = first_epoch; epoch <= tx.previous_epoch; ++epoch) {
      for (const auto key : {poseKey(epoch), velocityKey(epoch), biasKey(epoch)}) {
        if (graph_keys.count(key)) inside_ordering.push_back(key);
      }
    }
    const auto gaussian = boundary_graph.linearize(*tx.frozen_values);
    // Eliminate old variables on the sparse factor graph.  The previous code
    // first formed a dense normal matrix for every fixed-lag variable and then
    // took a Schur complement, making boundary construction cubic in the full
    // lag.  Partial QR produces the identical reduced Gaussian model while the
    // dense eigensolve below is bounded by the requested integrity window.
    gtsam::GaussianFactorGraph::shared_ptr reduced = gaussian;
    if (!outside_variables.empty()) {
      reduced = gaussian->eliminatePartialMultifrontal(
          outside_variables, gtsam::EliminateQR).second;
    }
    const auto reduced_system = reduced->hessian(inside_ordering);
    const int inside_columns = reduced_system.first.cols();
    Eigen::MatrixXd reduced_information = reduced_system.first;
    Eigen::VectorXd reduced_rhs = reduced_system.second;
    reduced_information =
        0.5 * (reduced_information + reduced_information.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(reduced_information);
    if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() ||
        eigen.eigenvalues().minCoeff() <
            -config_.integrity_window.rank_tolerance) {
      window.reason = "boundary partial Schur is not positive semidefinite";
      return window;
    }
    const double max_eigen = std::max(1.0, eigen.eigenvalues().maxCoeff());
    std::vector<int> retained;
    for (int i = 0; i < eigen.eigenvalues().size(); ++i) {
      if (eigen.eigenvalues()(i) >
          config_.integrity_window.rank_tolerance * max_eigen) retained.push_back(i);
    }
    LinearizedFactorBlock boundary;
    boundary.group_id = FactorGroupId(tx.id.value() * 1000);
    boundary.kind = FactorKind::BoundaryPrior;
    boundary.sensor = SensorType::Prior;
    boundary.role = RowRole::TrustedPrior;
    boundary.jacobian_whitened = Eigen::MatrixXd::Zero(
        retained.size(), total_columns);
    Eigen::MatrixXd compact = Eigen::MatrixXd::Zero(retained.size(), inside_columns);
    for (std::size_t row = 0; row < retained.size(); ++row) {
      compact.row(row) = std::sqrt(eigen.eigenvalues()(retained[row])) *
          eigen.eigenvectors().col(retained[row]).transpose();
    }
    // Boundary graph has no pending epoch columns and is ordered by epoch.
    boundary.jacobian_whitened.leftCols(inside_columns) = compact;
    boundary.residual_whitened = compact.transpose()
        .completeOrthogonalDecomposition().solve(reduced_rhs);
    boundary.jacobian_raw = boundary.jacobian_whitened;
    boundary.residual_raw = boundary.residual_whitened;
    boundary.covariance = Eigen::MatrixXd::Identity(retained.size(), retained.size());
    boundary.whitener = boundary.covariance;
    boundary.effective_weight = 1.0;
    boundary.whitening_model_id = "frozen_graph_partial_qr_schur";
    boundary.version = tx.base_version;
    window.blocks.push_back(std::move(boundary));
    window.capabilities.includes_boundary_prior = true;
  }

  auto append_group = [&](const PendingFactorGroup& group,
                          EpochTransaction linearization_tx) {
    linearization_tx.base_version = window.version;
    linearization_tx.backend_mutated = false;
    const std::uint64_t fingerprint = factorBlockFingerprint(
        group, linearization_tx);
    const auto cached = factor_block_cache_.find(group.id.value());
    LinearizedFactorBlock block;
    if (cached != factor_block_cache_.end() &&
        cached->second.transaction_id == tx.id &&
        cached->second.version == window.version &&
        cached->second.content_fingerprint == fingerprint) {
      block = cached->second.block;
      ++cache_audit_.factor_block_hits;
    } else {
      ++cache_audit_.factor_block_misses;
      const auto local = linearizeGroupDense(group, linearization_tx);
      block = linearizePendingGroup(
          group, linearization_tx, local.first, local.second);
      FactorBlockCacheEntry entry;
      entry.transaction_id = tx.id;
      entry.version = window.version;
      entry.content_fingerprint = fingerprint;
      entry.block = block;
      factor_block_cache_[group.id.value()] = std::move(entry);
    }
    Eigen::MatrixXd embedded_h = Eigen::MatrixXd::Zero(
        block.jacobian_whitened.rows(), total_columns);
    Eigen::MatrixXd embedded_raw = Eigen::MatrixXd::Zero(
        block.jacobian_raw.rows(), total_columns);
    const int offset = static_cast<int>(
        (linearization_tx.previous_epoch - first_epoch) * 15);
    embedded_h.block(0, offset, embedded_h.rows(), 30) =
        block.jacobian_whitened;
    embedded_raw.block(0, offset, embedded_raw.rows(), 30) =
        block.jacobian_raw;
    block.jacobian_whitened = std::move(embedded_h);
    block.jacobian_raw = std::move(embedded_raw);
    block.version = window.version;
    block.window_column_indices.resize(total_columns);
    std::iota(block.window_column_indices.begin(),
              block.window_column_indices.end(), 0);
    window.blocks.push_back(std::move(block));
    cache_audit_.factor_block_entries = factor_block_cache_.size();
    cache_audit_.factor_block_bytes = 0;
    for (const auto& item : factor_block_cache_)
      cache_audit_.factor_block_bytes += blockBytes(item.second.block);
  };

  for (const auto& record : tx.recoverable_history) {
    const auto& history = record;
    if (history.proposed_epoch <= first_epoch ||
        history.proposed_epoch > tx.previous_epoch) continue;
    EpochTransaction linearization_tx;
    linearization_tx.previous_epoch = history.previous_epoch;
    linearization_tx.proposed_epoch = history.proposed_epoch;
    linearization_tx.previous_state = history.previous_state;
    linearization_tx.nominal_predicted_state = history.current_state;
    linearization_tx.cv_predicted_state = history.current_state;
    linearization_tx.preintegration = history.preintegration;
    linearization_tx.raw_imu_slice = history.raw_imu_slice;
    linearization_tx.base_version = tx.base_version;
    linearization_tx.frozen_values = tx.frozen_values;
    std::vector<const PendingFactorGroup*> groups;
    for (const auto& group : history.groups) groups.push_back(&group);
    for (const auto selected_group : history.selected_groups) {
      const auto group = std::find_if(groups.begin(), groups.end(),
          [&](const PendingFactorGroup* candidate) {
            return candidate->id == selected_group;
          });
      if (group == groups.end()) {
        window.reason = "committed group provenance is incomplete";
        return window;
      }
      append_group(**group, linearization_tx);
    }
  }

  if (request.include_pending_imu) {
    append_group(tx.imu_group, tx);
    window.capabilities.includes_pending_imu = true;
  }
  if (request.include_pending_uwb) {
    for (const auto& group : tx.uwb_groups) {
      if (!group.nominal) continue;
      append_group(group, tx);
    }
    window.capabilities.includes_pending_uwb = !tx.uwb_groups.empty();
  }
  if (request.include_generic_bridge) {
    append_group(tx.generic_bridge_group, tx);
    append_group(tx.generic_bias_continuity_group, tx);
  }
  window.protected_state_map = Eigen::MatrixXd::Zero(3, total_columns);
  window.protected_state_map.block<3, 6>(
      0, total_columns - 15) =
      worldPositionPoseTangentJacobian(toGtsamPose(tx.nominal_predicted_state));
  window.capabilities.complete_factor_provenance =
      factor_ledger_.hasCompleteActiveProvenance(activeGraph());
  window.capabilities.history_provenance_valid = tx.previous_epoch == 0 ||
      window.capabilities.complete_factor_provenance;
  window.capabilities.fixed_lag_maturity_valid =
      config_.incremental.fixed_lag_epochs == 0 ||
      config_.incremental.fixed_lag_epochs >
          request.epochs + config_.integrity_window.recovery_margin_epochs;
  finalizeIntegrityWindow(&window, config_.integrity_window.rank_tolerance,
                          config_.integrity_window.max_condition_number);
  if (!window.capabilities.fixed_lag_maturity_valid) {
    window.model_valid = false;
    window.reason = "fixed lag violates integrity-window maturity delay";
  }
  return window;
}

LinearizedFactorBlock IncrementalUwbImuEstimator::buildPendingFactorBlock(
    const EpochTransaction& tx, FactorGroupId id) const {
  if (!active_transaction_id_ || *active_transaction_id_ != tx.id ||
      tx.backend_mutated || tx.base_graph_version != graph_version_) {
    throw std::logic_error("pending block requires active frozen transaction");
  }
  std::vector<const PendingFactorGroup*> groups{&tx.imu_group,
                                                &tx.generic_bridge_group,
                                                &tx.generic_bias_continuity_group};
  for (const auto& group : tx.uwb_groups) groups.push_back(&group);
  if (tx.dynamics_bridge_group) groups.push_back(&*tx.dynamics_bridge_group);
  for (const auto& history : tx.recoverable_history) {
    for (const auto& group : history.groups) groups.push_back(&group);
  }
  const auto found = std::find_if(groups.begin(), groups.end(),
      [&](const PendingFactorGroup* group) { return group->id == id; });
  if (found == groups.end()) {
    throw std::out_of_range("unknown pending factor group");
  }
  EpochTransaction local = tx;
  for (const auto& history : tx.recoverable_history) {
    const auto historical = std::find_if(history.groups.begin(), history.groups.end(),
        [&](const PendingFactorGroup& group) { return group.id == id; });
    if (historical != history.groups.end()) {
      local.previous_epoch = history.previous_epoch;
      local.proposed_epoch = history.proposed_epoch;
      local.previous_state = history.previous_state;
      local.nominal_predicted_state = history.current_state;
      local.cv_predicted_state = history.current_state;
      local.preintegration = history.preintegration;
      local.raw_imu_slice = history.raw_imu_slice;
      break;
    }
  }
  const std::uint64_t fingerprint = factorBlockFingerprint(**found, local);
  const auto cached = factor_block_cache_.find(id.value());
  if (cached != factor_block_cache_.end() &&
      cached->second.transaction_id == tx.id &&
      cached->second.version == tx.base_version &&
      cached->second.content_fingerprint == fingerprint) {
    ++cache_audit_.factor_block_hits;
    return cached->second.block;
  }
  ++cache_audit_.factor_block_misses;
  const auto linear = linearizeGroupDense(**found, local);
  LinearizedFactorBlock block = linearizePendingGroup(
      **found, local, linear.first, linear.second);
  FactorBlockCacheEntry entry;
  entry.transaction_id = tx.id;
  entry.version = tx.base_version;
  entry.content_fingerprint = fingerprint;
  entry.block = block;
  factor_block_cache_[id.value()] = std::move(entry);
  cache_audit_.factor_block_entries = factor_block_cache_.size();
  cache_audit_.factor_block_bytes = 0;
  for (const auto& item : factor_block_cache_)
    cache_audit_.factor_block_bytes += blockBytes(item.second.block);
  return block;
}

EstimatorCacheAudit IncrementalUwbImuEstimator::cacheAudit() const {
  EstimatorCacheAudit result = cache_audit_;
  result.factor_block_entries = factor_block_cache_.size();
  result.factor_block_bytes = 0;
  for (const auto& item : factor_block_cache_)
    result.factor_block_bytes += blockBytes(item.second.block);
  return result;
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
  return jointMarginal(epoch_);
}

Eigen::Matrix<double, 15, 15>
IncrementalUwbImuEstimator::jointMarginal(std::size_t requested_epoch) const {
  const gtsam::KeyVector keys{poseKey(requested_epoch),
                             velocityKey(requested_epoch),
                             biasKey(requested_epoch)};
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
  if (!adapter_transaction_ || batch.timestamp != adapter_transaction_->end) {
    throw std::logic_error("pre-measurement snapshot requires matching predicted epoch");
  }
  validateUwbBatch(batch);
  if (adapter_transaction_->uwb_groups.empty()) {
    attachUwbGroups(&*adapter_transaction_, batch);
  }
  CurrentStatePrior prior = pendingCurrentPrior(*adapter_transaction_);
  pending_batch_id_ = batch.id;
  if (!prior.excludes_current_uwb) throw std::logic_error("current UWB already contaminates prior");
  const gtsam::Pose3 pose = toGtsamPose(adapter_transaction_->nominal_predicted_state);
  UwbPoseBatchFactor proposed(poseKey(adapter_transaction_->proposed_epoch), batch, lever_arm_body_m_);
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
  Eigen::LDLT<Eigen::Matrix<double, 15, 15>> information_solve(
      prior.covariance);
  const Eigen::Matrix<double, 15, 15> information = information_solve.solve(
      Eigen::Matrix<double, 15, 15>::Identity());
  auto snapshot = std::make_shared<ImmutableEstimationSnapshot>(
      prior.mean, prior.version, diagnostics, std::vector<WhitenedRowBlock>{rows},
      capabilities, LinearizationConsistency::Strict, prior, prior.covariance,
      information);
  last_snapshot_extraction_ms_ = elapsedMs(snapshot_start);
  return snapshot;
}

CommitReceipt IncrementalUwbImuEstimator::commitEpoch(
    EpochTransaction&& tx, const EpochCommitPlan& plan) {
  if (backend_poisoned_) throw std::runtime_error("backend is fail-closed");
  if (!active_transaction_id_ || *active_transaction_id_ != tx.id ||
      tx.backend_mutated || tx.base_graph_version != graph_version_ ||
      !(tx.base_version == LinearizationVersion{
          graph_version_, ordering_version_, 1, linpoint_version_})) {
    throw std::logic_error("BACKEND_VERSION_MISMATCH or repeated transaction");
  }
  const auto start = std::chrono::steady_clock::now();
  gtsam::NonlinearFactorGraph graph;
  std::vector<const PendingFactorGroup*> all_groups;
  all_groups.push_back(&tx.imu_group);
  for (const auto& group : tx.uwb_groups) all_groups.push_back(&group);
  all_groups.push_back(&tx.generic_bridge_group);
  all_groups.push_back(&tx.generic_bias_continuity_group);
  if (tx.dynamics_bridge_group) all_groups.push_back(&*tx.dynamics_bridge_group);
  for (const auto& history : tx.recoverable_history) {
    for (const auto& group : history.groups) {
      if (group.replaces_group) all_groups.push_back(&group);
    }
  }
  std::set<std::uint64_t> selected_ids;
  std::vector<const PendingFactorGroup*> selected_groups;
  for (const auto id : plan.groups_to_add) {
    if (!selected_ids.insert(id.value()).second) {
      throw std::logic_error("commit plan contains duplicate add group");
    }
    const auto found = std::find_if(all_groups.begin(), all_groups.end(),
        [&](const PendingFactorGroup* group) { return group->id == id; });
    if (found == all_groups.end()) throw std::logic_error("commit plan references unknown add group");
    selected_groups.push_back(*found);
    for (const auto& factor : (*found)->factors) graph.push_back(factor);
  }
  const bool has_imu = selected_ids.count(tx.imu_group.id.value()) != 0;
  const bool has_pose_bridge =
      selected_ids.count(tx.generic_bridge_group.id.value()) != 0;
  const bool has_bias_bridge =
      selected_ids.count(tx.generic_bias_continuity_group.id.value()) != 0;
  const bool has_bridge = has_pose_bridge ||
      (tx.dynamics_bridge_group &&
       selected_ids.count(tx.dynamics_bridge_group->id.value()) != 0);
  if (has_imu == has_bridge || has_pose_bridge != has_bias_bridge) {
    throw std::logic_error(
        "commit plan must select IMU or a complete pose/bias bridge pair");
  }
  std::vector<std::size_t> remove_slots;
  for (const auto group : plan.groups_to_remove) {
    const auto slots = factor_ledger_.activeSlots(group);
    if (slots.empty()) throw std::logic_error("commit plan cannot remove inactive/unknown group");
    remove_slots.insert(remove_slots.end(), slots.begin(), slots.end());
  }
  for (const auto& history : tx.recoverable_history) {
    const auto removed_imu = std::find(
        plan.groups_to_remove.begin(), plan.groups_to_remove.end(),
        history.groups.front().id) != plan.groups_to_remove.end();
    if (!removed_imu) continue;
    int historical_pose_bridges = 0;
    int historical_bias_bridges = 0;
    for (const auto& group : history.groups) {
      if (!group.replaces_group || *group.replaces_group != history.groups.front().id ||
          selected_ids.count(group.id.value()) == 0) continue;
      historical_pose_bridges += group.kind == FactorKind::KinematicBridge;
      historical_bias_bridges += group.kind == FactorKind::BiasContinuity;
    }
    if (historical_pose_bridges != 1 || historical_bias_bridges != 1) {
      throw std::logic_error(
          "historical IMU removal requires exactly one complete bridge pair");
    }
  }
  std::sort(remove_slots.begin(), remove_slots.end());
  if (std::adjacent_find(remove_slots.begin(), remove_slots.end()) != remove_slots.end()) {
    throw std::logic_error("commit plan attempts duplicate factor-slot removal");
  }
  gtsam::Values values;
  values.insert(poseKey(tx.proposed_epoch), toGtsamPose(tx.cv_predicted_state));
  values.insert(velocityKey(tx.proposed_epoch), tx.cv_predicted_state.velocity_world_mps);
  values.insert(biasKey(tx.proposed_epoch), toGtsamBias(tx.cv_predicted_state));
  BackendUpdateAudit update;
  try {
    update = backendUpdate(graph, values, tx.proposed_epoch, true, remove_slots);
  } catch (...) {
    backend_poisoned_ = true;
    active_transaction_id_.reset();
    adapter_transaction_.reset();
    pending_epoch_ = false;
    throw;
  }
  tx.backend_mutated = true;
  epoch_ = tx.proposed_epoch;
  state_timestamp_ = tx.end;
  if (update.marginalized_epochs > 0) {
    marginalization_count_ += update.marginalized_epochs;
    ++ordering_version_;
  }
  ++graph_version_;
  ++linpoint_version_;
  for (std::size_t i = 0; i < tx.imu_cursor.consume_count; ++i) imu_queue_.pop_front();
  imu_boundary_ = tx.imu_cursor.new_boundary;
  const auto state_query_start = std::chrono::steady_clock::now();
  queryCurrentState();
  last_state_query_ms_ = elapsedMs(state_query_start);
  state_history_[epoch_] = current_state_;
  prefix_covariances_[epoch_] = currentJointMarginal();

  // Historical removal/replacement can change every retained state linpoint.
  // Synchronize the recovery catalog with the single post-update estimate so
  // the next transaction never mixes pre- and post-recovery states.
  if (!plan.groups_to_remove.empty()) {
    const gtsam::Values recovered_values = backendIsam().calculateEstimate();
    for (auto& item : state_history_) {
      if (!recovered_values.exists(poseKey(item.first)) ||
          !recovered_values.exists(velocityKey(item.first)) ||
          !recovered_values.exists(biasKey(item.first))) continue;
      item.second = stateFromValues(recovered_values, item.first,
                                    item.second.timestamp);
    }
    current_state_ = state_history_.at(epoch_);
  }

  factor_ledger_.recordSelected(tx, plan.groups_to_add);
  for (const auto& item : plan.group_health) {
    factor_ledger_.setHealth(FactorGroupId(item.first), item.second);
  }
  const auto& active = activeGraph();
  factor_ledger_.markSlotsAbsent(active, oldestRetainedEpoch());
  std::set<std::size_t> previously_occupied(remove_slots.begin(), remove_slots.end());
  std::set<std::size_t> reused_slots;
  for (const auto* group : selected_groups) {
    std::vector<std::size_t> slots;
    for (const auto& expected : group->factors) {
      bool found = false;
      for (std::size_t slot = 0; slot < active.size(); ++slot) {
        if (active[slot] && active[slot].get() == expected.get()) {
          slots.push_back(slot);
          found = true;
          break;
        }
      }
      if (!found) throw std::runtime_error("committed factor identity missing from backend");
    }
    factor_ledger_.activate(group->id, slots,
        {graph_version_, ordering_version_, 1, linpoint_version_});
    for (const auto slot : slots) {
      if (previously_occupied.count(slot)) reused_slots.insert(slot);
    }
  }
  for (const auto group : plan.groups_to_remove) {
    std::optional<FactorGroupId> replacement;
    const auto relation = plan.replacement_relations.find(group.value());
    if (relation != plan.replacement_relations.end()) {
      replacement = FactorGroupId(relation->second);
    }
    factor_ledger_.transition(
        group, replacement ? FactorLifecycle::SupersededByBridge
                           : FactorLifecycle::RemovedByFde,
        {graph_version_, ordering_version_, 1, linpoint_version_}, replacement);
  }
  factor_ledger_.syncBoundaryFactors(
      activeGraph(), update.boundary_factor_slots, state_timestamp_, epoch_,
      {graph_version_, ordering_version_, 1, linpoint_version_});

  for (auto& committed : committed_epochs_) {
    bool changed = false;
    for (const auto removed : plan.groups_to_remove) {
      auto selected = std::find(committed.selected_groups.begin(),
                                committed.selected_groups.end(), removed);
      if (selected != committed.selected_groups.end()) {
        committed.selected_groups.erase(selected);
        changed = true;
      }
    }
    if (!changed) continue;
    for (const auto* group : selected_groups) {
      if (group->replaces_group &&
          std::find(plan.groups_to_remove.begin(), plan.groups_to_remove.end(),
                    *group->replaces_group) != plan.groups_to_remove.end()) {
        committed.selected_groups.push_back(group->id);
        if (group->kind == FactorKind::UwbBatch) {
          committed.transaction.uwb_groups.push_back(*group);
        } else if (group->kind == FactorKind::KinematicBridge) {
          committed.transaction.generic_bridge_group = *group;
        } else if (group->kind == FactorKind::BiasContinuity) {
          committed.transaction.generic_bias_continuity_group = *group;
        }
      }
    }
  }
  EpochTransaction committed_transaction = tx;
  committed_transaction.frozen_graph.reset();
  committed_transaction.frozen_values.reset();
  committed_transaction.frozen_slots.clear();
  committed_transaction.recoverable_history.clear();
  committed_epochs_.push_back(
      CommittedEpochRecord{std::move(committed_transaction), plan.groups_to_add});

  current_uwb_committed_ = false;
  for (const auto& group : tx.uwb_groups) {
    if (selected_ids.count(group.id.value())) current_uwb_committed_ = true;
  }
  if (current_uwb_committed_) committed_uwb_batches_.emplace_back(epoch_, tx.uwb_batch.id);
  pruneRetainedMetadata();
  pending_epoch_ = false;
  pending_batch_id_.reset();
  active_transaction_id_.reset();
  adapter_transaction_.reset();
  last_uwb_update_ms_ = elapsedMs(start);

  CommitReceipt receipt;
  receipt.transaction_id = tx.id;
  receipt.action_id = plan.action_id;
  receipt.graph_version = graph_version_;
  receipt.committed_epoch = epoch_;
  receipt.state_timestamp = state_timestamp_;
  receipt.added_factor_slots = update.new_factor_slots;
  receipt.removed_factor_slots = update.removed_factor_slots;
  receipt.marginalized_keys = update.marginalized_keys;
  receipt.boundary_factor_slots = update.boundary_factor_slots;
  receipt.group_to_slots.clear();
  for (const auto* group : selected_groups) {
    receipt.group_to_slots[group->id.value()] =
        factor_ledger_.activeSlots(group->id);
  }
  receipt.reused_factor_slots.assign(reused_slots.begin(), reused_slots.end());
  receipt.recovery_epoch_begin = plan.recovery_epoch_begin;
  receipt.recovery_epoch_end = plan.recovery_epoch_end;
  for (const auto group : plan.groups_to_remove) {
    const auto entries = factor_ledger_.groupEntries(group);
    if (!entries.empty() && entries.front().epoch_end < tx.proposed_epoch) {
      receipt.historical_groups_removed.push_back(group);
    }
  }
  for (const auto* group : selected_groups) {
    if (group->recovery_epoch != 0 && group->recovery_epoch == tx.proposed_epoch) {
      receipt.historical_groups_added.push_back(group->id);
    }
  }
  receipt.backend_updates = 1;
  receipt.integrity_available = !plan.best_effort_integrity_unavailable;
  return receipt;
}

DiscardReceipt IncrementalUwbImuEstimator::discardEpoch(
    EpochTransaction&& tx, const DiscardReason& reason) {
  if (!active_transaction_id_ || *active_transaction_id_ != tx.id ||
      tx.backend_mutated) {
    throw std::logic_error("discard requires the active unmutated transaction");
  }
  DiscardReceipt receipt;
  receipt.transaction_id = tx.id;
  receipt.attempted_timestamp = tx.end;
  receipt.last_committed_timestamp = state_timestamp_;
  receipt.controlled_reinitialization_required =
      reason.controlled_reinitialization_required;
  receipt.reason = reason.detail;
  active_transaction_id_.reset();
  adapter_transaction_.reset();
  pending_epoch_ = false;
  pending_batch_id_.reset();
  return receipt;
}

void IncrementalUwbImuEstimator::commitUwbBatch(const UwbBatch& batch) {
  if (!adapter_transaction_ || batch.timestamp != adapter_transaction_->end ||
      !pending_batch_id_ || *pending_batch_id_ != batch.id) {
    throw std::logic_error("commit requires matching pending adapter transaction");
  }
  EpochCommitPlan plan = EpochCommitPlan::nominalPlan(*adapter_transaction_);
  EpochTransaction tx = std::move(*adapter_transaction_);
  (void)commitEpoch(std::move(tx), plan);
}

void IncrementalUwbImuEstimator::rejectUwbBatch(
    const UwbBatch& batch, const std::string& reason) {
  if (!adapter_transaction_ || batch.timestamp != adapter_transaction_->end ||
      !pending_batch_id_ || *pending_batch_id_ != batch.id) {
    throw std::logic_error("reject requires matching pending adapter transaction");
  }
  EpochCommitPlan plan;
  plan.groups_to_add = {adapter_transaction_->imu_group.id};
  plan.fde_status = FdeStatus::SuccessUwbExclusion;
  plan.best_effort_integrity_unavailable = true;
  EpochTransaction tx = std::move(*adapter_transaction_);
  (void)reason;
  (void)commitEpoch(std::move(tx), plan);
}

NavigationState IncrementalUwbImuEstimator::currentState() const {
  if (!initialized_) throw std::logic_error("UWB/IMU estimator is not initialized");
  if (adapter_transaction_) return adapter_transaction_->nominal_predicted_state;
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
  result.historical_fault_provenance =
      factor_ledger_.hasCompleteActiveProvenance(activeGraph());
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
      result.covariance);
  if (!result.mean.allFinite() || eigen.info() != Eigen::Success ||
      eigen.eigenvalues().minCoeff() <= 0.0) return std::nullopt;
  result.condition_number = eigen.eigenvalues().maxCoeff() /
      eigen.eigenvalues().minCoeff();
  return result;
}

}  // namespace uwb_imu_pl
