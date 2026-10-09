#include "uwb_imu_pl/estimation/incremental_estimator.hpp"

#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <boost/make_shared.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <numeric>
#include <mutex>
#include <set>
#include <stdexcept>

#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"
#include "uwb_imu_pl/estimation/estimation_tuning.hpp"
#include "uwb_imu_pl/factors/kinematic_bridge_factor.hpp"
#include "uwb_imu_pl/factors/realtime_factors.hpp"
#include "uwb_imu_pl/integrity/history_fault_parameterization.hpp"
#include "uwb_imu_pl/integrity/history_fault_summary.hpp"
#include "uwb_imu_pl/integrity/history_summary_extraction.hpp"
#include "uwb_imu_pl/integrity/history_summary_lifecycle.hpp"
#include "uwb_imu_pl/integrity/protection_level_v2.hpp"

namespace uwb_imu_pl {

struct P106HistoryRootMutationPeer {
  static void corrupt(IncrementalHistoryRootCache* cache) {
    cache->corruptTreeRootForTesting();
  }
};

struct P103BoundaryGroupCacheValue {
  std::uint64_t fingerprint = 0;
  gtsam::GaussianFactorGraph rows;
  std::size_t row_count = 0;
  std::size_t information_form_factors = 0;
  double information_form_offset = 0.0;
};

class FixedLagBackend final : public gtsam::IncrementalFixedLagSmoother {
 public:
  FixedLagBackend(double lag, const gtsam::ISAM2Params& params)
      : gtsam::IncrementalFixedLagSmoother(lag, params),
        history_owner_(
            std::make_shared<const std::uint64_t>(0x503130332d523039ULL)) {}

  const gtsam::ISAM2& isam() const { return isam_; }
  HistoryFaultSummary updateHistoryRoot(
      HistoryRootCacheRequest request,
      HistoryCarrierRankCertificate* carrier_rank,
      const HistoryFaultSummaryOptions& options) {
    request.owner = history_owner_;
    request.owner_payload = history_owner_.get();
    request.verify_full_oracle = verify_history_root_oracle_;
    return history_root_.update(request, carrier_rank, options);
  }
  void enableHistoryRootOracleForTesting(bool enabled) {
    verify_history_root_oracle_ = enabled;
  }
  void corruptHistoryTreeRootForTesting() {
    P106HistoryRootMutationPeer::corrupt(&history_root_);
  }
  void resetHistoryTreeForTesting() {
    history_root_.invalidate("p106 test-only forced full rebuild");
  }
  HistoryRootCacheAudit historyRootAudit() const {
    HistoryRootCacheAudit result = history_root_.audit();
    result.factor_group_hits = factor_group_hits_;
    result.factor_group_misses = factor_group_misses_;
    return result;
  }
  const P103BoundaryGroupCacheValue* findHistoryGroup(
      std::uint64_t group, std::uint64_t fingerprint) {
    const auto found = history_groups_.find(group);
    if (found != history_groups_.end() &&
        found->second.fingerprint == fingerprint) {
      ++factor_group_hits_;
      return &found->second;
    }
    ++factor_group_misses_;
    return nullptr;
  }
  void storeHistoryGroup(std::uint64_t group,
                         P103BoundaryGroupCacheValue value) {
    history_groups_[group] = std::move(value);
  }
  void retainHistoryGroups(const std::set<std::uint64_t>& active) {
    for (auto it = history_groups_.begin(); it != history_groups_.end();) {
      if (active.count(it->first) == 0) {
        it = history_groups_.erase(it);
      } else {
        ++it;
      }
    }
  }

 private:
  std::shared_ptr<const std::uint64_t> history_owner_;
  IncrementalHistoryRootCache history_root_;
  std::map<std::uint64_t, P103BoundaryGroupCacheValue> history_groups_;
  std::uint64_t factor_group_hits_ = 0;
  std::uint64_t factor_group_misses_ = 0;
  bool verify_history_root_oracle_ = false;
};

namespace {

struct EstimationRuntimeV1 {
  EstimationTuningV1 tuning;
  EstimatorNumericsAuditV1 audit;
  double physical_time_s = 0.0;
  const gtsam::VectorValues* accepted_delta = nullptr;
  std::uint64_t accepted_delta_version = 0;
};
std::mutex estimation_runtime_mutex;
std::map<const IncrementalUwbImuEstimator*, EstimationRuntimeV1> estimation_runtime;
EstimationRuntimeV1 readEstimationRuntime(const IncrementalUwbImuEstimator* estimator) {
  std::lock_guard<std::mutex> lock(estimation_runtime_mutex);
  return estimation_runtime.at(estimator);
}

struct P005EstimatorRuntime {
  CommitFaultPoint fault_point = CommitFaultPoint::None;
  std::uint64_t armed_nonce = 0;
  CommitFaultPoint hit_point = CommitFaultPoint::None;
  std::uint64_t hit_nonce = 0;
  bool armed = false;
  bool hit = false;
  bool consumed = false;
};

// P0-07 diagnostic certificates are frozen while prepareEpoch still owns a
// coherent view of the graph and ledger.  The sidecar is deliberately outside
// the estimator and EpochTransaction layouts so the golden public ABI remains
// unchanged.  Audit readers take a shared immutable snapshot under the mutex
// and never touch the live ledger while commit publishes a replacement.
using P007RawOwnerCertificate =
    std::shared_ptr<const std::vector<RawRowOwnershipAuditV1>>;
struct P007CertificateEntry {
  std::weak_ptr<const gtsam::Values> lifetime;
  P007RawOwnerCertificate certificate;
};

std::mutex& p007CertificateMutex() {
  static std::mutex mutex;
  return mutex;
}

std::map<const IncrementalUwbImuEstimator*,
         std::map<std::pair<std::uint64_t, const gtsam::Values*>,
                  P007CertificateEntry>>&
p007Certificates() {
  static std::map<const IncrementalUwbImuEstimator*,
                  std::map<std::pair<std::uint64_t, const gtsam::Values*>,
                           P007CertificateEntry>> states;
  return states;
}

void freezeP007Certificate(
    const IncrementalUwbImuEstimator* estimator,
    const EpochTransaction& transaction,
    std::vector<RawRowOwnershipAuditV1> certificate) {
  auto immutable = std::make_shared<const std::vector<RawRowOwnershipAuditV1>>(
      std::move(certificate));
  std::lock_guard<std::mutex> lock(p007CertificateMutex());
  auto& by_transaction = p007Certificates()[estimator];
  for (auto it = by_transaction.begin(); it != by_transaction.end();) {
    if (it->second.lifetime.expired()) {
      it = by_transaction.erase(it);
    } else {
      ++it;
    }
  }
  by_transaction[{transaction.id.value(), transaction.frozen_values.get()}] =
      {transaction.frozen_values, std::move(immutable)};
}

P007RawOwnerCertificate readP007Certificate(
    const IncrementalUwbImuEstimator* estimator,
    const EpochTransaction& transaction) {
  std::lock_guard<std::mutex> lock(p007CertificateMutex());
  const auto owner = p007Certificates().find(estimator);
  if (owner == p007Certificates().end()) return {};
  const auto found = owner->second.find(
      {transaction.id.value(), transaction.frozen_values.get()});
  return found == owner->second.end() ? P007RawOwnerCertificate{}
                                      : found->second.certificate;
}

void eraseP007Certificates(const IncrementalUwbImuEstimator* estimator) {
  std::lock_guard<std::mutex> lock(p007CertificateMutex());
  p007Certificates().erase(estimator);
}

std::atomic<std::uint64_t>& p005FaultNonce() {
  static std::atomic<std::uint64_t> nonce{1};
  return nonce;
}

std::mutex& p005RuntimeMutex() {
  static std::mutex mutex;
  return mutex;
}

std::map<const IncrementalUwbImuEstimator*, P005EstimatorRuntime>&
p005RuntimeStates() {
  static std::map<const IncrementalUwbImuEstimator*, P005EstimatorRuntime>
      states;
  return states;
}

std::uint64_t armP005Fault(const IncrementalUwbImuEstimator* estimator,
                           CommitFaultPoint point) {
  std::lock_guard<std::mutex> lock(p005RuntimeMutex());
  auto& state = p005RuntimeStates()[estimator];
  state = P005EstimatorRuntime{};
  if (point == CommitFaultPoint::None) return 0;
  std::uint64_t nonce = p005FaultNonce().fetch_add(1);
  if (nonce == 0) nonce = p005FaultNonce().fetch_add(1);
  state.fault_point = point;
  state.armed_nonce = nonce;
  state.armed = true;
  return nonce;
}

std::uint64_t hitP005Fault(const IncrementalUwbImuEstimator* estimator,
                           CommitFaultPoint point) {
  std::lock_guard<std::mutex> lock(p005RuntimeMutex());
  auto& state = p005RuntimeStates()[estimator];
  if (!state.armed || state.fault_point != point || state.armed_nonce == 0) {
    return 0;
  }
  state.hit_point = point;
  state.hit_nonce = state.armed_nonce;
  state.hit = true;
  state.consumed = true;
  state.fault_point = CommitFaultPoint::None;
  state.armed_nonce = 0;
  state.armed = false;
  return state.hit_nonce;
}

void disarmP005Fault(const IncrementalUwbImuEstimator* estimator) {
  std::lock_guard<std::mutex> lock(p005RuntimeMutex());
  auto& state = p005RuntimeStates()[estimator];
  state.fault_point = CommitFaultPoint::None;
  state.armed_nonce = 0;
  state.armed = false;
}

CommitFaultInjectionAuditV1 p005FaultAudit(
    const IncrementalUwbImuEstimator* estimator) {
  std::lock_guard<std::mutex> lock(p005RuntimeMutex());
  const auto state = p005RuntimeStates()[estimator];
  CommitFaultInjectionAuditV1 audit;
  audit.armed_point = state.fault_point;
  audit.armed_nonce = state.armed_nonce;
  audit.hit_point = state.hit_point;
  audit.hit_nonce = state.hit_nonce;
  audit.armed = state.armed;
  audit.hit = state.hit;
  audit.consumed = state.consumed;
  return audit;
}

void eraseP005Runtime(const IncrementalUwbImuEstimator* estimator) {
  std::lock_guard<std::mutex> lock(p005RuntimeMutex());
  p005RuntimeStates().erase(estimator);
}

void cacheHashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= bytes[i];
    *hash *= 1099511628211ULL;
  }
}
template <class T>
void cacheHashScalar(std::uint64_t* hash, const T& value) {
  cacheHashBytes(hash, &value, sizeof(value));
}
template <class Derived>
void cacheHashMatrix(std::uint64_t* hash,
                     const Eigen::MatrixBase<Derived>& value) {
  const Eigen::Index rows = value.rows(), cols = value.cols();
  cacheHashScalar(hash, rows);
  cacheHashScalar(hash, cols);
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
  cacheHashScalar(&hash, id);
  cacheHashScalar(&hash, group.kind);
  cacheHashScalar(&hash, group.sensor);
  cacheHashScalar(&hash, group.nominal);
  cacheHashText(&hash, group.noise_model_id);
  cacheHashText(&hash, group.model_id);
  cacheHashMatrix(&hash, group.raw_covariance);
  cacheHashScalar(&hash, tx.previous_epoch);
  cacheHashScalar(&hash, tx.proposed_epoch);
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
    const auto value = id_value.value();
    cacheHashScalar(&hash, value);
  }
  for (const auto& factor : group.factors) {
    const auto pointer = reinterpret_cast<std::uintptr_t>(factor.get());
    cacheHashScalar(&hash, pointer);
  }
  return hash;
}

std::uint64_t boundaryGroupFingerprint(
    FactorGroupId group, const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& values) {
  std::uint64_t hash = 1469598103934665603ULL;
  const std::uint64_t group_value = group.value();
  cacheHashScalar(&hash, group_value);
  std::set<gtsam::Key> keys;
  for (const auto& factor : factors) {
    const auto pointer = reinterpret_cast<std::uintptr_t>(factor.get());
    cacheHashScalar(&hash, pointer);
    for (const auto key : factor->keys()) keys.insert(key);
  }
  for (const auto key : keys) {
    cacheHashScalar(&hash, key);
    const char symbol = gtsam::Symbol(key).chr();
    if (symbol == 'x') {
      cacheHashMatrix(&hash, values.at<gtsam::Pose3>(key).matrix());
    } else if (symbol == 'v') {
      cacheHashMatrix(&hash, values.at<gtsam::Vector3>(key));
    } else if (symbol == 'b') {
      const auto bias = values.at<gtsam::imuBias::ConstantBias>(key);
      cacheHashMatrix(&hash, bias.accelerometer());
      cacheHashMatrix(&hash, bias.gyroscope());
    } else {
      // Unknown value types are never guessed.  Force a miss by binding the
      // current Values owner; the complete rebuild path remains available.
      const auto owner = reinterpret_cast<std::uintptr_t>(&values);
      cacheHashScalar(&hash, owner);
    }
  }
  return hash;
}

std::size_t blockBytes(const LinearizedFactorBlock& block) {
  return sizeof(block) +
         sizeof(double) *
             static_cast<std::size_t>(
                 block.jacobian_raw.size() + block.residual_raw.size() +
                 block.covariance.size() + block.whitener.size() +
                 block.jacobian_whitened.size() +
                 block.residual_whitened.size());
}

gtsam::Key positionKey(std::size_t epoch) { return gtsam::Symbol('p', epoch); }
gtsam::Key velocityKey(std::size_t epoch) { return gtsam::Symbol('v', epoch); }
gtsam::Key poseKey(std::size_t epoch) { return gtsam::Symbol('x', epoch); }
gtsam::Key biasKey(std::size_t epoch) { return gtsam::Symbol('b', epoch); }

gtsam::Pose3 toGtsamPose(const NavigationState& state) {
  const Eigen::Quaterniond q = state.q_world_body.normalized();
  return gtsam::Pose3(gtsam::Rot3(q.toRotationMatrix()),
                      state.position_world_m);
}

gtsam::imuBias::ConstantBias toGtsamBias(const NavigationState& state) {
  return {state.accel_bias_mps2, state.gyro_bias_radps};
}

// A warm-start problem, not a backend factor: the previous accepted state is
// fixed and only the new 15 variables are optimized. No information is added
// to the persistent graph, nor to a frozen FDE candidate.
class FixedPreviousImuWarmStart final
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3,
                                      gtsam::imuBias::ConstantBias> {
 public:
  FixedPreviousImuWarmStart(const gtsam::CombinedImuFactor& factor,
                           const NavigationState& previous)
      : NoiseModelFactor3(factor.noiseModel(), factor.keys()[2],
                          factor.keys()[3], factor.keys()[5]),
        factor_(factor), previous_(previous) {}
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
      const gtsam::imuBias::ConstantBias& bias,
      boost::optional<gtsam::Matrix&> h1=boost::none,
      boost::optional<gtsam::Matrix&> h2=boost::none,
      boost::optional<gtsam::Matrix&> h3=boost::none) const override {
    return factor_.evaluateError(toGtsamPose(previous_), previous_.velocity_world_mps,
        pose, velocity, toGtsamBias(previous_), bias,
        boost::none, boost::none, h1, h2, boost::none, h3);
  }
 private:
  gtsam::CombinedImuFactor factor_;
  NavigationState previous_;
};

class ExperimentalHuberRangeFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  ExperimentalHuberRangeFactor(gtsam::Key key, const UwbBatch& batch,
                              const Eigen::Vector3d& lever)
      : NoiseModelFactor1(gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(
                1.5, gtsam::noiseModel::mEstimator::Base::Scalar),
            gtsam::noiseModel::Gaussian::Covariance(validatedUwbCovariance(batch))), key),
        gaussian_(key,batch,lever) {
    const auto covariance=validatedUwbCovariance(batch);
    const Eigen::MatrixXd diagonal=covariance.diagonal().asDiagonal();
    if (!covariance.isApprox(diagonal,1e-12))
      throw std::invalid_argument("nominal robust experimental requires independent per-range covariance");
  }
  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> h=boost::none) const override {
    return gaussian_.evaluateError(pose,h);
  }
  double error(const gtsam::Values& values) const override {
    // GTSAM Robust::loss evaluates the block norm even with Scalar IRLS.
    // Explicitly sum scalar Huber costs so LM acceptance matches its rows.
    const auto residual=gaussian_.whitenedError(values);
    double cost=0.0;
    for (int i=0;i<residual.size();++i) {
      const double magnitude=std::abs(residual(i));
      cost+=magnitude<=1.5 ? 0.5*magnitude*magnitude : 1.5*(magnitude-0.75);
    }
    return cost;
  }
 private:
  UwbPoseBatchFactor gaussian_;
};

NavigationState stateFromValues(const gtsam::Values& values, std::size_t epoch,
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
  if (llt.info() != Eigen::Success)
    throw std::runtime_error("whitening failed");
  return llt.matrixL().solve(
      Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
}

double elapsedMs(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - start)
      .count();
}

gtsam::GaussianBayesNet currentCliqueClosure(const gtsam::ISAM2& isam2,
                                             const gtsam::KeyVector& keys) {
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
  values.insert(velocityKey(tx.previous_epoch),
                tx.previous_state.velocity_world_mps);
  values.insert(biasKey(tx.previous_epoch), toGtsamBias(tx.previous_state));
  values.insert(poseKey(tx.proposed_epoch),
                toGtsamPose(tx.nominal_predicted_state));
  values.insert(velocityKey(tx.proposed_epoch),
                tx.nominal_predicted_state.velocity_world_mps);
  values.insert(biasKey(tx.proposed_epoch),
                toGtsamBias(tx.nominal_predicted_state));
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

static std::vector<RawRowOwnershipAuditV1> buildRawRowOwnershipCertificateV1(
    const EpochTransaction& transaction, const FactorLedger& frozen_ledger);

IncrementalUwbEstimator::IncrementalUwbEstimator(
    const IncrementalConfig& config)
    : config_(config), isam2_([&config] {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = config.relinearize_threshold;
        params.relinearizeSkip = config.relinearize_skip;
        return params;
      }()) {}

void IncrementalUwbEstimator::initialize(
    TimestampNs timestamp, const Eigen::Vector3d& position_world_m,
    const Eigen::Vector3d& velocity_world_mps) {
  if (initialized_)
    throw std::logic_error("UWB incremental estimator already initialized");
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
  if (!initialized_)
    throw std::logic_error("UWB incremental estimator is not initialized");
  if (!(timestamp_ < batch.timestamp))
    throw std::invalid_argument("UWB batches must be strictly ordered");
  const double dt = batch.timestamp.seconds() - timestamp_.seconds();
  const gtsam::Point3 p0 = estimate_.at<gtsam::Point3>(positionKey(epoch_));
  const gtsam::Vector3 v0 = estimate_.at<gtsam::Vector3>(velocityKey(epoch_));
  ++epoch_;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  graph.add(boost::make_shared<ConstantVelocityRegularizer>(
      positionKey(epoch_ - 1), velocityKey(epoch_ - 1), positionKey(epoch_),
      velocityKey(epoch_), dt, config_.smoothness_sigma_m));
  graph.add(
      boost::make_shared<UwbPositionBatchFactor>(positionKey(epoch_), batch));
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
  const gtsam::Point3 position =
      estimate_.at<gtsam::Point3>(positionKey(epoch_));
  const Eigen::MatrixXd covariance = validatedUwbCovariance(batch);
  const Eigen::MatrixXd w = whitener(covariance);
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(batch.measurements.size(), 6);
  Eigen::VectorXd residual(batch.measurements.size());
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    const Eigen::Vector3d delta =
        position - batch.measurements[i].anchor_position_m;
    h.block<1, 3>(static_cast<Eigen::Index>(i), 0) =
        delta.transpose() / delta.norm();
    residual(static_cast<Eigen::Index>(i)) =
        batch.measurements[i].range_m - delta.norm();
  }
  WhitenedRowBlock measurement_rows;
  measurement_rows.factor_id = batch.measurements.front().factor_id;
  measurement_rows.role = RowRole::Measurement;
  measurement_rows.jacobian = w * h;
  measurement_rows.residual = w * residual;
  measurement_rows.column_indices = {0, 1, 2, 3, 4, 5};
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
  regularizer_rows.column_indices = {0, 1, 2, 3, 4, 5};
  regularizer_rows.whitening_model_id = "constant_velocity_regularizer";
  row_blocks_ = {std::move(regularizer_rows), std::move(measurement_rows)};
  diagnostics_.rows = static_cast<int>(batch.measurements.size());
  diagnostics_.columns = 3;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(w * h.leftCols(3));
  diagnostics_.rank =
      static_cast<int>((svd.singularValues().array() > 1e-10).count());
  diagnostics_.covariance_valid = marginal_.allFinite();
  diagnostics_.model_valid =
      diagnostics_.rank == 3 && diagnostics_.covariance_valid;
  if (!diagnostics_.model_valid)
    diagnostics_.reason = "UWB-only current geometry/marginal invalid";

  IntegrityOutput output;
  output.timestamp = timestamp_;
  output.state = currentState();
  output.batch_committed = true;
  output.protection_level.label = IntegrityLabel::ImplementedUnverified;
  output.protection_level.availability = Availability::Unavailable;
  output.protection_level.reason =
      "UWB-only bridge does not publish formal fusion PL";
  return output;
}

NavigationState IncrementalUwbEstimator::currentState() const {
  if (!initialized_)
    throw std::logic_error("UWB incremental estimator is not initialized");
  NavigationState state;
  state.id = StateId(epoch_);
  state.timestamp = timestamp_;
  state.position_world_m = estimate_.at<gtsam::Point3>(positionKey(epoch_));
  state.velocity_world_mps = estimate_.at<gtsam::Vector3>(velocityKey(epoch_));
  return state;
}

std::shared_ptr<const EstimationSnapshot> IncrementalUwbEstimator::snapshot()
    const {
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
      information = solve.solve(
          Eigen::MatrixXd::Identity(marginal_.rows(), marginal_.cols()));
    }
  }
  return std::make_shared<ImmutableEstimationSnapshot>(
      currentState(), version, diagnostics_, std::move(rows), capabilities,
      LinearizationConsistency::Strict, std::nullopt, marginal_, information);
}

IncrementalUwbImuEstimator::IncrementalUwbImuEstimator(
    const IntegrityConfig& config, const Eigen::Vector3d& lever_arm_body_m)
    : config_(config), lever_arm_body_m_(lever_arm_body_m), isam2_([&config] {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = config.incremental.relinearize_threshold;
        params.relinearizeSkip = config.incremental.relinearize_skip;
        // The newest navigation state is at the top of this temporal Bayes
        // tree. Stop the relinearization check once an unchanged separator is
        // reached instead of scanning every historical state each period.
        params.enablePartialRelinearizationCheck = true;
        // R09 dependency receipts need the actual per-key relinearization
        // closure.  Aggregate counters cannot prove that an untouched factor
        // group is reusable.
        params.enableDetailedResults = true;
        // Range factors are strongly nonlinear in the coupled pose/velocity
        // state. Dogleg prevents the unbounded Gauss-Newton steps that can
        // corrupt the following IMU prediction and make an otherwise anchored
        // graph appear indefinite during Cholesky elimination.
        params.optimizationParams = gtsam::ISAM2DoglegParams();
        return params;
      }()) {
  // Defensively clear any registry entry left at a recycled object address.
  eraseP007Certificates(this);
  const auto tuning = readEstimationTuningV1(config_);
  {
    std::lock_guard<std::mutex> lock(estimation_runtime_mutex);
    estimation_runtime[this] = EstimationRuntimeV1{tuning, {}, 0.0};
  }
  if (config_.incremental.fixed_lag_epochs == 1) {
    throw std::invalid_argument("fixed_lag_epochs must be 0 or at least 2");
  }
  if (config_.incremental.fixed_lag_epochs > 0 || tuning.nominal_lag_s > 0.0) {
    gtsam::ISAM2Params params = isam2_.params();
    params.findUnusedFactorSlots = true;
    fixed_lag_backend_.reset(new FixedLagBackend(
        tuning.nominal_lag_s > 0.0 ? tuning.nominal_lag_s :
            static_cast<double>(config_.incremental.fixed_lag_epochs - 1), params));
  }
  auto params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(
      config_.imu.gravity_mps2);
  params->accelerometerCovariance =
      Eigen::Matrix3d::Identity() *
      std::pow(config_.imu.accelerometer_sigma, 2);
  params->gyroscopeCovariance =
      Eigen::Matrix3d::Identity() * std::pow(config_.imu.gyroscope_sigma, 2);
  params->integrationCovariance = Eigen::Matrix3d::Identity() * 1e-9;
  params->biasAccCovariance =
      Eigen::Matrix3d::Identity() *
      std::pow(config_.imu.accelerometer_bias_rw_sigma, 2);
  params->biasOmegaCovariance =
      Eigen::Matrix3d::Identity() *
      std::pow(config_.imu.gyroscope_bias_rw_sigma, 2);
  imu_params_ = params;
}

IncrementalUwbImuEstimator::~IncrementalUwbImuEstimator() {
  eraseP007Certificates(this);
  eraseP005Runtime(this);
  std::lock_guard<std::mutex> lock(estimation_runtime_mutex);
  estimation_runtime.erase(this);
}

EstimatorNumericsAuditV1 estimatorNumericsAuditV1(
    const IncrementalUwbImuEstimator& estimator) {
  return readEstimationRuntime(&estimator).audit;
}

bool IncrementalUwbImuEstimator::backendPoisoned() const {
  return backend_poisoned_;
}

std::uint64_t IncrementalUwbImuEstimator::setCommitFaultPointForTesting(
    CommitFaultPoint point) {
  return armP005Fault(this, point);
}

CommitFaultInjectionAuditV1
IncrementalUwbImuEstimator::commitFaultInjectionAuditForTesting() const {
  return p005FaultAudit(this);
}

const gtsam::ISAM2& IncrementalUwbImuEstimator::backendIsam() const {
  return fixed_lag_backend_ ? fixed_lag_backend_->isam() : isam2_;
}

const gtsam::NonlinearFactorGraph& IncrementalUwbImuEstimator::activeGraph()
    const {
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
    audit.marked_keys.assign(result.markedKeys.begin(), result.markedKeys.end());
    if (result.detail) {
      for (const auto& item : result.detail->variableStatus) {
        if (item.second.isRelinearized) audit.relinearized_keys.push_back(item.first);
      }
    }
    ++backend_update_count_;
    const std::uint64_t injected_nonce =
        hitP005Fault(this, CommitFaultPoint::DuringBackendUpdate);
    if (injected_nonce != 0) {
      throw std::runtime_error(
          "injected commit failure during backend update; nonce=" +
          std::to_string(injected_nonce));
    }
    return audit;
  }

  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
  if (add_timestamps) {
    const auto runtime = readEstimationRuntime(this);
    const double logical_timestamp = runtime.tuning.nominal_lag_s > 0.0
        ? runtime.physical_time_s : static_cast<double>(timestamp_epoch);
    timestamps.emplace(poseKey(timestamp_epoch), logical_timestamp);
    timestamps.emplace(velocityKey(timestamp_epoch), logical_timestamp);
    timestamps.emplace(biasKey(timestamp_epoch), logical_timestamp);
  }
  const std::size_t before = fixed_lag_backend_->timestamps().size();
  fixed_lag_backend_->update(graph, values, timestamps, removals);
  const auto& isam_result = fixed_lag_backend_->getISAM2Result();
  audit.new_factor_slots.assign(isam_result.newFactorsIndices.begin(),
                                isam_result.newFactorsIndices.end());
  audit.marked_keys.assign(isam_result.markedKeys.begin(),
                           isam_result.markedKeys.end());
  if (isam_result.detail) {
    for (const auto& item : isam_result.detail->variableStatus) {
      if (item.second.isRelinearized) {
        audit.relinearized_keys.push_back(item.first);
      }
    }
  }
  ++backend_update_count_;
  const std::uint64_t injected_nonce =
      hitP005Fault(this, CommitFaultPoint::DuringBackendUpdate);
  if (injected_nonce != 0) {
    throw std::runtime_error(
        "injected commit failure during backend update; nonce=" +
        std::to_string(injected_nonce));
  }
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
  return static_cast<std::uint32_t>(fixed_lag_backend_->timestamps().size() /
                                    3);
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
      static_cast<std::size_t>(
          config_.integrity_window.recovery_margin_epochs) +
      1;
  std::size_t oldest =
      epoch_ > configured_history ? epoch_ - configured_history : 0;
  if (fixed_lag_backend_) oldest = std::max(oldest, oldestRetainedEpoch());
  while (!committed_uwb_batches_.empty() &&
         committed_uwb_batches_.front().first < oldest) {
    committed_uwb_batches_.pop_front();
  }
  while (!committed_epochs_.empty() &&
         committed_epochs_.front().transaction.proposed_epoch <= oldest) {
    committed_epochs_.pop_front();
  }
  for (auto it = state_history_.begin();
       it != state_history_.end() && it->first < oldest;)
    it = state_history_.erase(it);
  for (auto it = prefix_covariances_.begin();
       it != prefix_covariances_.end() && it->first < oldest;) {
    it = prefix_covariances_.erase(it);
  }
}

void IncrementalUwbImuEstimator::initialize(
    const NavigationState& initial_state,
    const Eigen::Matrix<double, 15, 1>& prior_sigmas) {
  backend_poisoned_ = false;
  (void)armP005Fault(this, CommitFaultPoint::None);
  if (initialized_)
    throw std::logic_error("UWB/IMU estimator already initialized");
  if (!prior_sigmas.allFinite() || (prior_sigmas.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "all initial prior sigmas must be finite and > 0");
  }
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  const gtsam::Pose3 pose = toGtsamPose(initial_state);
  const gtsam::Vector3 velocity = initial_state.velocity_world_mps;
  const gtsam::imuBias::ConstantBias bias = toGtsamBias(initial_state);
  graph.addPrior(poseKey(0), pose,
                 gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas.head<6>()));
  graph.addPrior(
      velocityKey(0), velocity,
      gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas.segment<3>(6)));
  graph.addPrior(biasKey(0), bias,
                 gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas.tail<6>()));
  values.insert(poseKey(0), pose);
  values.insert(velocityKey(0), velocity);
  values.insert(biasKey(0), bias);
  {
    std::lock_guard<std::mutex> lock(estimation_runtime_mutex);
    auto& runtime = estimation_runtime.at(this);
    runtime.physical_time_s = initial_state.timestamp.seconds();
    runtime.audit.bias_integration_sigmas = runtime.tuning.bias_integration_sigmas
        ? *runtime.tuning.bias_integration_sigmas : prior_sigmas.tail<6>().eval();
    imu_params_->biasAccOmegaInt =
        runtime.audit.bias_integration_sigmas.array().square().matrix().asDiagonal();
  }
  const auto update = backendUpdate(graph, values, 0, true);
  if (update.marginalized_epochs != 0) {
    throw std::runtime_error("fixed-lag initialization marginalized state");
  }
  current_state_ = initial_state;
  current_state_.id = StateId(0);
  state_timestamp_ = initial_state.timestamp;
  queryCurrentState();
  preintegrated_.reset(
      new gtsam::PreintegratedCombinedMeasurements(imu_params_, bias));
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
  if (!initialized_)
    throw std::logic_error("UWB/IMU estimator is not initialized");
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

void IncrementalUwbImuEstimator::validateUwbBatch(const UwbBatch& batch) const {
  if (!initialized_)
    throw std::logic_error("UWB/IMU estimator is not initialized");
  if (batch.measurements.empty()) {
    throw std::invalid_argument("UWB batch must not be empty");
  }
  if (!(state_timestamp_ < batch.timestamp)) {
    throw std::invalid_argument("UWB batch timestamp must increase");
  }
  std::set<std::uint64_t> configured;
  for (const auto& anchor : config_.anchors)
    configured.insert(anchor.id.value());
  std::set<std::uint64_t> measurement_ids;
  std::set<std::uint64_t> factor_ids;
  TimestampNs earliest = batch.measurements.front().timestamp;
  TimestampNs latest = earliest;
  for (const auto& measurement : batch.measurements) {
    if (!std::isfinite(measurement.range_m) || measurement.range_m <= 0.0 ||
        !std::isfinite(measurement.sigma_m) || measurement.sigma_m <= 0.0 ||
        !measurement.anchor_position_m.allFinite()) {
      throw std::invalid_argument(
          "UWB batch contains invalid measurement values");
    }
    if (!configured.empty() &&
        configured.count(measurement.anchor_id.value()) == 0) {
      throw std::invalid_argument(
          "UWB batch references an unconfigured physical anchor");
    }
    if (!measurement_ids.insert(measurement.id.value()).second ||
        !factor_ids.insert(measurement.factor_id.value()).second) {
      throw std::invalid_argument(
          "UWB batch measurement/factor IDs must be unique");
    }
    if (measurement.timestamp < earliest) earliest = measurement.timestamp;
    if (latest < measurement.timestamp) latest = measurement.timestamp;
    const double skew =
        std::abs(measurement.timestamp.seconds() - batch.timestamp.seconds());
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
    TimestampNs timestamp, const UwbBatch* batch,
    const EpochPreparationOptions& options) {
  if (!initialized_)
    throw std::logic_error("UWB/IMU estimator is not initialized");
  if (backendPoisoned())
    throw std::runtime_error(
        "backend is fail-closed after a partial update exception");
  if (active_transaction_id_)
    throw std::logic_error("commit or discard pending epoch first");
  if (!(state_timestamp_ < timestamp))
    throw std::invalid_argument("prediction timestamp must increase");
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
      throw std::runtime_error(
          "duplicate or reversed IMU sample in prediction interval");
    }
    const double dt = measurement.timestamp.seconds() - previous_time.seconds();
    if (!std::isfinite(dt) || dt > config_.imu.max_gap_s + 1e-12) {
      throw std::runtime_error("IMU gap exceeds imu.max_gap_s");
    }
    proposed.integrateMeasurement(
        0.5 * (previous_measurement.specific_force_mps2 +
               measurement.specific_force_mps2),
        0.5 * (previous_measurement.angular_velocity_radps +
               measurement.angular_velocity_radps),
        dt);
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
  tx.preparation = options;
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
  tx.preintegration =
      std::make_shared<const gtsam::PreintegratedCombinedMeasurements>(
          proposed);
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
  tx.imu_group.keys = {poseKey(epoch_),     velocityKey(epoch_),
                       poseKey(next_epoch), velocityKey(next_epoch),
                       biasKey(epoch_),     biasKey(next_epoch)};
  tx.imu_group.noise_model_id = "gtsam_preintegrated_combined_covariance_v1";
  tx.imu_group.model_id = "combined_imu_interval_constant_fault_map_v1";
  tx.imu_group.factors.add(gtsam::CombinedImuFactor(
      poseKey(epoch_), velocityKey(epoch_), poseKey(next_epoch),
      velocityKey(next_epoch), biasKey(epoch_), biasKey(next_epoch), proposed));
  if (options.build_imu_recovery_material) {
    tx.generic_bridge_group.id = FactorGroupId(tx.id.value() * 1000 + 3);
    tx.generic_bias_continuity_group.id = FactorGroupId(tx.id.value() * 1000 + 4);
    tx.generic_bridge_group =
        BridgeFactory().makeGeneric(tx, config_.bridge.generic);
    tx.generic_bias_continuity_group =
        BridgeFactory().makeBiasContinuity(tx, config_.bridge.generic);
  }
  if (batch) {
    attachUwbGroups(&tx, *batch, options.build_uwb_recovery_material);
  }
  tx.ledger_version = factor_ledger_.version();
  if (options.build_integrity_material) {
    tx.frozen_graph =
        std::make_shared<const gtsam::NonlinearFactorGraph>(activeGraph());
    gtsam::Values frozen = backendIsam().calculateEstimate();
    frozen.insert(poseKey(next_epoch), toGtsamPose(tx.cv_predicted_state));
    frozen.insert(velocityKey(next_epoch),
                  tx.cv_predicted_state.velocity_world_mps);
    frozen.insert(biasKey(next_epoch), toGtsamBias(tx.cv_predicted_state));
    tx.frozen_values = std::make_shared<const gtsam::Values>(std::move(frozen));
  }
  std::map<std::size_t, FactorGroupId> slot_groups;
  if (tx.frozen_graph) {
    for (const auto& entry : factor_ledger_.entries()) {
      if (entry.lifecycle == FactorLifecycle::Active && entry.backend_slot) {
        slot_groups.emplace(*entry.backend_slot, entry.group_id);
      }
    }
  }
  for (std::size_t slot = 0;
       tx.frozen_graph && slot < tx.frozen_graph->size(); ++slot) {
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
                                    ? tx.previous_epoch - context_intervals
                                    : 0;
  if (fixed_lag_backend_) {
    tx.oldest_recoverable_epoch =
        std::max(tx.oldest_recoverable_epoch, oldestRetainedEpoch());
  }
  if (options.build_recoverable_history) for (const auto& record : committed_epochs_) {
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
    context.previous_state =
        stateFromValues(*tx.frozen_values, committed.previous_epoch,
                        state_history_.at(committed.previous_epoch).timestamp);
    context.current_state =
        stateFromValues(*tx.frozen_values, committed.proposed_epoch,
                        state_history_.at(committed.proposed_epoch).timestamp);
    const auto selected_in_record = [&](FactorGroupId id) {
      return std::find(record.selected_groups.begin(),
                       record.selected_groups.end(),
                       id) != record.selected_groups.end();
    };
    const bool committed_imu_selected =
        selected_in_record(committed.imu_group.id);
    // Provider scope controls fault maps and recovery alternatives, never the
    // nominal observation history.  A UWB-only profile still needs the
    // selected IMU group in the epoch catalog for complete provenance.
    if (committed_imu_selected) {
      context.groups.push_back(committed.imu_group);
    } else {
      if (selected_in_record(committed.generic_bridge_group.id)) {
        context.groups.push_back(committed.generic_bridge_group);
      }
      if (selected_in_record(committed.generic_bias_continuity_group.id)) {
        context.groups.push_back(committed.generic_bias_continuity_group);
      }
      if (committed.dynamics_bridge_group &&
          selected_in_record(committed.dynamics_bridge_group->id)) {
        context.groups.push_back(*committed.dynamics_bridge_group);
      }
    }
    const std::uint64_t history_base =
        tx.id.value() * 1000000ULL + committed.proposed_epoch * 100ULL;
    std::uint64_t replacement_index = 0;
    FactorGroupId selected_uwb;
    const PendingFactorGroup* selected_uwb_group = nullptr;
    for (const auto& group : committed.uwb_groups) {
      if (std::find(record.selected_groups.begin(),
                    record.selected_groups.end(),
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
    const Eigen::MatrixXd historical_covariance =
        selected_uwb_group ? validatedUwbCovariance(committed.uwb_batch)
                           : Eigen::MatrixXd{};
    if (options.build_uwb_recovery_material)
    for (const auto excluded_anchor : selected_anchors) {
      UwbBatch retained = committed.uwb_batch;
      retained.measurements.clear();
      std::vector<Eigen::Index> retained_indices;
      for (std::size_t i = 0; i < committed.uwb_batch.measurements.size();
           ++i) {
        const auto& measurement = committed.uwb_batch.measurements[i];
        if (selected_measurements.count(measurement.id.value()) &&
            measurement.anchor_id.value() != excluded_anchor) {
          retained.measurements.push_back(measurement);
          retained_indices.push_back(static_cast<Eigen::Index>(i));
        }
      }
      if (retained.measurements.empty()) continue;
      retained.covariance_m2.resize(retained_indices.size(),
                                    retained_indices.size());
      for (std::size_t row = 0; row < retained_indices.size(); ++row) {
        for (std::size_t column = 0; column < retained_indices.size();
             ++column) {
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
      replacement.model_id =
          "historical_uwb_exclude_anchor_" + std::to_string(excluded_anchor);
      replacement.nominal = false;
      replacement.raw_covariance = retained.covariance_m2;
      replacement.excluded_fault_units = {FaultUnitId(excluded_anchor)};
      for (const auto& measurement : retained.measurements) {
        replacement.source_measurements.push_back(measurement.id);
        replacement.fault_units.push_back(
            FaultUnitId(measurement.anchor_id.value()));
        replacement.source_ids.push_back(
            "uwb:" + std::to_string(measurement.anchor_id.value()));
      }
      replacement.factors.add(boost::make_shared<UwbPoseBatchFactor>(
          poseKey(committed.proposed_epoch), retained, lever_arm_body_m_));
      replacement.replaces_group = selected_uwb;
      replacement.recovery_epoch = next_epoch;
      context.groups.push_back(std::move(replacement));
    }
    if (committed_imu_selected && options.build_imu_recovery_material) {
      PendingFactorGroup historical_bridge = committed.generic_bridge_group;
      historical_bridge.id = FactorGroupId(history_base + 80);
      historical_bridge.replaces_group = committed.imu_group.id;
      historical_bridge.recovery_epoch = next_epoch;
      context.groups.push_back(std::move(historical_bridge));
      PendingFactorGroup historical_bias =
          committed.generic_bias_continuity_group;
      historical_bias.id = FactorGroupId(history_base + 81);
      historical_bias.replaces_group = committed.imu_group.id;
      historical_bias.recovery_epoch = next_epoch;
      context.groups.push_back(std::move(historical_bias));
      if (committed.dynamics_bridge_group) {
        context.groups.push_back(*committed.dynamics_bridge_group);
      }
    }
    context.selected_groups = record.selected_groups;
    for (auto& group : context.groups) group.recovery_epoch = next_epoch;
    tx.recoverable_history.push_back(std::move(context));
  }
  tx.history_recoverability = !options.build_recoverable_history
      ? HistoryRecoverability::Recoverable
      : (tx.frozen_graph &&
                 factor_ledger_.hasCompleteActiveProvenance(*tx.frozen_graph)
             ? HistoryRecoverability::Recoverable
             : HistoryRecoverability::MissingProvenance);
  if (tx.frozen_values) {
    freezeP007Certificate(
        this, tx, buildRawRowOwnershipCertificateV1(tx, factor_ledger_));
  }
  active_transaction_id_ = tx.id;
  pending_epoch_ = true;
  current_uwb_committed_ = false;
  pending_batch_id_ = batch ? std::optional<BatchId>(batch->id) : std::nullopt;
  last_no_uwb_update_ms_ = elapsedMs(start);
  return tx;
}

void IncrementalUwbImuEstimator::attachUwbGroups(EpochTransaction* tx,
                                                 const UwbBatch& batch,
                                                 bool build_recovery_material) const {
  if (!tx || !tx->uwb_groups.empty())
    throw std::logic_error("UWB group already attached");
  tx->uwb_batch = batch;
  PendingFactorGroup group;
  group.id = FactorGroupId(tx->id.value() * 1000 + 2);
  group.kind = FactorKind::UwbBatch;
  group.sensor = SensorType::Uwb;
  group.keys = {poseKey(tx->proposed_epoch)};
  group.noise_model_id = batch.covariance_model_id.empty()
                             ? "per_measurement_diagonal"
                             : batch.covariance_model_id;
  group.model_id = "physical_anchor_correlated_batch_v1";
  group.raw_covariance = validatedUwbCovariance(batch);
  for (std::size_t i = 0; i < batch.measurements.size(); ++i) {
    group.source_measurements.push_back(batch.measurements[i].id);
    group.fault_units.push_back(
        FaultUnitId(batch.measurements[i].anchor_id.value()));
    group.source_ids.push_back(
        "uwb:" + std::to_string(batch.measurements[i].anchor_id.value()));
  }
  if (readEstimationRuntime(this).tuning.nominal_robust_experimental) {
    if (build_recovery_material || tx->preparation.build_integrity_material)
      throw std::invalid_argument("experimental Huber is forbidden in an integrity transaction");
    group.model_id = "nominal_robust_experimental_huber_scalar_1_5_v1";
    group.factors.add(boost::make_shared<ExperimentalHuberRangeFactor>(
        poseKey(tx->proposed_epoch),batch,lever_arm_body_m_));
  } else {
    group.factors.add(boost::make_shared<UwbPoseBatchFactor>(
        poseKey(tx->proposed_epoch), batch, lever_arm_body_m_));
  }
  tx->uwb_groups.push_back(std::move(group));

  if (!build_recovery_material) return;

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
    retained.covariance_m2.resize(retained_indices.size(),
                                  retained_indices.size());
    for (std::size_t row = 0; row < retained_indices.size(); ++row) {
      for (std::size_t col = 0; col < retained_indices.size(); ++col) {
        retained.covariance_m2(row, col) =
            full_covariance(retained_indices[row], retained_indices[col]);
      }
    }
    retained.covariance_model_id =
        batch.covariance_model_id.empty()
            ? "retained_principal_covariance"
            : batch.covariance_model_id + ":retained_principal_covariance";
    PendingFactorGroup replacement;
    replacement.id =
        FactorGroupId(tx->id.value() * 1000 + 10 + candidate_index++);
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
      replacement.fault_units.push_back(
          FaultUnitId(measurement.anchor_id.value()));
      replacement.source_ids.push_back(
          "uwb:" + std::to_string(measurement.anchor_id.value()));
    }
    replacement.factors.add(boost::make_shared<UwbPoseBatchFactor>(
        poseKey(tx->proposed_epoch), retained, lever_arm_body_m_));
    tx->uwb_groups.push_back(std::move(replacement));
  }
}

EpochTransaction IncrementalUwbImuEstimator::prepareEpoch(
    const UwbBatch& batch) {
  return prepareEpoch(batch, EpochPreparationOptions{});
}

EpochTransaction IncrementalUwbImuEstimator::prepareEpoch(
    const UwbBatch& batch, const EpochPreparationOptions& options) {
  validateUwbBatch(batch);
  return prepareTransaction(batch.timestamp, &batch, options);
}

void IncrementalUwbImuEstimator::predictTo(TimestampNs timestamp) {
  adapter_transaction_ =
      prepareTransaction(timestamp, nullptr, EpochPreparationOptions{});
}

LinearizedFactorBlock IncrementalUwbImuEstimator::linearizePendingGroup(
    const PendingFactorGroup& group, const EpochTransaction& tx,
    const Eigen::MatrixXd& h, const Eigen::VectorXd& rhs) const {
  LinearizedFactorBlock block;
  block.group_id = group.id;
  block.kind = group.kind;
  block.sensor = group.sensor;
  block.role = group.kind == FactorKind::BoundaryPrior ? RowRole::TrustedPrior
                                                       : RowRole::Measurement;
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
  if (group.kind == FactorKind::UwbBatch) {
    // `GaussianFactor::jacobian()` is expressed using GTSAM's upper-triangular
    // square-root information convention.  It is not, in general, the same
    // matrix as the lower-Cholesky whitener initially constructed above for
    // correlated UWB covariance.  Recover the physical raw range system with
    // the matching GTSAM model and retain its exact deterministic whitening
    // coordinate, avoiding a gratuitous row rotation of rank-update inputs.
    const auto gaussian_noise =
        gtsam::noiseModel::Gaussian::Covariance(block.covariance);
    block.whitener = gaussian_noise->R();
    block.jacobian_raw.resize(h.rows(), h.cols());
    for (Eigen::Index column = 0; column < h.cols(); ++column) {
      block.jacobian_raw.col(column) =
          gaussian_noise->unwhiten(h.col(column));
    }
    block.residual_raw = gaussian_noise->unwhiten(rhs);
  } else {
    block.jacobian_raw =
        block.whitener.triangularView<Eigen::Lower>().solve(h);
    block.residual_raw =
        block.whitener.triangularView<Eigen::Lower>().solve(rhs);
  }
  return block;
}

CurrentStatePrior IncrementalUwbImuEstimator::pendingCurrentPrior(
    const EpochTransaction& tx) const {
  const Eigen::Matrix<double, 15, 15> previous_covariance =
      currentJointMarginal();
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
  const auto preparation_start = std::chrono::steady_clock::now();
  if (!active_transaction_id_ || *active_transaction_id_ != tx.id ||
      tx.backend_mutated || tx.base_graph_version != graph_version_ ||
      tx.ledger_version != factor_ledger_.version() ||
      !(tx.base_version == LinearizationVersion{graph_version_,
                                                ordering_version_, 1,
                                                linpoint_version_})) {
    throw std::logic_error(
        "integrity window requires the active frozen transaction");
  }
  LinearizedIntegrityWindow window;
  window.id = WindowId(tx.id.value());
  window.version = tx.base_version;
  // No condensed material at all is an explicit state, not an absent field:
  // every consumer can distinguish "nothing to summarize" from "summary could
  // not be established" (§7.6 row 7 / cold start).
  window.history_summary.state =
      toString(HistorySummaryState::NotRequired);
  const std::size_t interval_count =
      std::min<std::size_t>(request.epochs, tx.proposed_epoch);
  const std::size_t first_epoch = tx.proposed_epoch - interval_count;
  window.detector_first_epoch = first_epoch;
  window.recovery_first_epoch = tx.oldest_recoverable_epoch;
  const int total_columns = static_cast<int>((interval_count + 1) * 15);
  for (std::size_t epoch = first_epoch; epoch <= tx.proposed_epoch; ++epoch) {
    window.state_layout.push_back(
        StateLayoutEntry{epoch,
                         {poseKey(epoch), velocityKey(epoch), biasKey(epoch)},
                         static_cast<int>((epoch - first_epoch) * 15),
                         15,
                         epoch == tx.proposed_epoch});
  }

  if (!tx.frozen_graph || !tx.frozen_values ||
      tx.ledger_version != factor_ledger_.version()) {
    window.reason = "transaction graph/values/ledger freeze is incomplete";
    return window;
  }

  std::set<gtsam::Key> explicit_window_keys;
  for (std::size_t epoch = first_epoch; epoch <= tx.previous_epoch; ++epoch) {
    explicit_window_keys.insert(poseKey(epoch));
    explicit_window_keys.insert(velocityKey(epoch));
    explicit_window_keys.insert(biasKey(epoch));
  }
  struct HistoricalGroupMetadata {
    const PendingFactorGroup* group = nullptr;
    std::size_t epoch = 0;
  };
  std::map<std::uint64_t, HistoricalGroupMetadata> historical_groups;
  std::set<std::uint64_t> explicit_group_ids;
  for (const auto& record : tx.recoverable_history) {
    if (record.proposed_epoch < first_epoch ||
        record.proposed_epoch > tx.previous_epoch)
      continue;
    for (const auto group : record.selected_groups) {
      const auto found =
          std::find_if(record.groups.begin(), record.groups.end(),
                       [&](const PendingFactorGroup& candidate) {
                         return candidate.id == group;
                       });
      if (found == record.groups.end()) {
        window.reason =
            "committed group provenance is incomplete: selected group " +
            std::to_string(group.value()) + " is absent from epoch " +
            std::to_string(record.proposed_epoch) + " catalog";
        return window;
      }
      historical_groups[group.value()] = {&*found, record.proposed_epoch};
      const bool all_keys_in_window =
          !found->keys.empty() &&
          std::all_of(found->keys.begin(), found->keys.end(),
                      [&](gtsam::Key key) {
                        return explicit_window_keys.count(key) != 0;
                      });
      if (all_keys_in_window) explicit_group_ids.insert(group.value());
    }
  }
  std::set<std::size_t> explicit_slots;
  bool slot_identity_valid = true;
  for (const auto group_value : explicit_group_ids) {
    const auto slots = factor_ledger_.activeSlots(FactorGroupId(group_value));
    if (slots.empty()) slot_identity_valid = false;
    explicit_slots.insert(slots.begin(), slots.end());
  }
  // Boundary inputs, grouped by factor group.  Each group is linearized as a
  // whole (`NonlinearFactorGraph::linearize` may hand back HessianFactors, and
  // a HessianFactor is exactly the normal matrix of that group: the residual
  // *constant* is only lost by GTSAM's reduction/elimination (route (i-b)),
  // not by the per-group factorization used here).  Grouping also keeps the
  // condensed boundary attributable to the factor ledger group by group.
  struct BoundaryGroupRows {
    FactorGroupId id;
    gtsam::NonlinearFactorGraph factors;
  };
  std::vector<BoundaryGroupRows> boundary_groups;
  std::map<std::uint64_t, std::size_t> boundary_group_index;
  for (const auto& frozen : tx.frozen_slots) {
    const bool pointer_ok =
        frozen.slot < tx.frozen_graph->size() &&
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
    slot_identity_valid =
        slot_identity_valid && pointer_ok &&
        (accounting.explicit_window_block != accounting.boundary_input) &&
        frozen.group_id.has_value();
    if (explicit_block) continue;
    const std::uint64_t group_value =
        frozen.group_id ? frozen.group_id->value() : 0;
    const auto found = boundary_group_index.find(group_value);
    if (found == boundary_group_index.end()) {
      boundary_group_index.emplace(group_value, boundary_groups.size());
      BoundaryGroupRows rows;
      if (frozen.group_id) rows.id = *frozen.group_id;
      else rows.id = FactorGroupId(group_value);
      rows.factors.push_back(frozen.factor);
      boundary_groups.push_back(std::move(rows));
    } else {
      boundary_groups[found->second].factors.push_back(frozen.factor);
    }
  }
  std::map<std::uint64_t, FrozenWindowFactorInventoryEntry> frozen_inventory;
  for (const auto& accounting : window.slot_accounting) {
    if (!accounting.group_id) continue;
    auto& entry = frozen_inventory[accounting.group_id->value()];
    entry.group_id = *accounting.group_id;
    entry.slots.push_back(accounting.slot);
    entry.disposition = accounting.explicit_window_block
                            ? FrozenFactorDisposition::ExplicitMeasurement
                            : FrozenFactorDisposition::BoundaryInput;
    const auto metadata = historical_groups.find(accounting.group_id->value());
    if (metadata != historical_groups.end()) {
      entry.epoch = metadata->second.epoch;
      entry.kind = metadata->second.group->kind;
      entry.sensor = metadata->second.group->sensor;
      entry.keys = metadata->second.group->keys;
    }
  }
  for (auto& item : frozen_inventory) {
    window.factor_inventory.push_back(std::move(item.second));
  }
  window.capabilities.frozen_slot_identity_valid = slot_identity_valid;
  window.capabilities.every_active_factor_accounted_once =
      slot_identity_valid &&
      window.slot_accounting.size() == tx.frozen_slots.size();
  if (!slot_identity_valid) {
    window.reason = "active factor slot ownership/pointer identity is invalid";
    return window;
  }

  if (!boundary_groups.empty()) {
    std::set<gtsam::Key> window_keys;
    for (std::size_t epoch = first_epoch; epoch <= tx.previous_epoch; ++epoch) {
      window_keys.insert(poseKey(epoch));
      window_keys.insert(velocityKey(epoch));
      window_keys.insert(biasKey(epoch));
    }

    // ---- C1-b: fault-preserving history summary (route (i)) --------------
    // The condensed boundary is the square-root summary produced by the C1-a
    // module from the *linearized* frozen rows (no eigendecomposition, no
    // pre-elimination: the route-(i-b) measurement shows GTSAM's partial
    // elimination drops per-factor constants, so kappa/d_perp must come from
    // the linearized rows).  Each boundary group contributes one Jacobian
    // factor: its rows are the group's whitened residual rows [A | b], so the
    // constant stays available (a HessianFactor alone would only carry
    // A'A, A'b, b'b).  Historical fault columns (Part A granularity) are
    // injected with fault keys in the separator; only groups that are boundary
    // inputs carry them (window-explicit groups keep their own maps).
    gtsam::GaussianFactorGraph augmented;
    std::vector<std::string> augmented_row_uids;
    std::vector<std::uint64_t> augmented_row_provenance;
    // Stage nominal rows by their ledger owner.  A group receiving historical
    // fault columns is inserted later through its augmented factor *instead
    // of* these rows.  This preserves the single stochastic-row/covariance
    // ownership required by the raw factor model.
    std::map<std::uint64_t, gtsam::GaussianFactorGraph>
        nominal_boundary_graphs;
    std::map<std::uint64_t, std::size_t> nominal_boundary_rows;
    std::size_t information_form_factors = 0;
    double information_form_offset = 0.0;
    std::set<std::uint64_t> active_boundary_group_ids;
    for (const auto& group : boundary_groups) {
      const std::uint64_t group_value = group.id.value();
      active_boundary_group_ids.insert(group_value);
      const std::uint64_t group_fingerprint =
          boundaryGroupFingerprint(group.id, group.factors, *tx.frozen_values);
      if (fixed_lag_backend_) {
        const auto* cached = fixed_lag_backend_->findHistoryGroup(
            group_value, group_fingerprint);
        if (cached != nullptr) {
          nominal_boundary_graphs[group_value] = cached->rows;
          nominal_boundary_rows[group_value] = cached->row_count;
          information_form_factors += cached->information_form_factors;
          information_form_offset += cached->information_form_offset;
          continue;
        }
      }
      P103BoundaryGroupCacheValue cache_value;
      cache_value.fingerprint = group_fingerprint;
      const std::size_t information_count_before = information_form_factors;
      const double information_offset_before = information_form_offset;
      for (const auto& factor : group.factors) {
        const auto jacobian = boost::dynamic_pointer_cast<gtsam::JacobianFactor>(
            factor->linearize(*tx.frozen_values));
        if (jacobian) {
          // Measurement-style factor: the linearization already IS the
          // whitened row block [A | b], so the row identity (and with it the
          // per-anchor fault map) is preserved exactly.
          nominal_boundary_graphs[group.id.value()].push_back(jacobian);
          nominal_boundary_rows[group.id.value()] +=
              static_cast<std::size_t>(jacobian->rows());
          continue;
        }
        const auto hessian = boost::dynamic_pointer_cast<gtsam::HessianFactor>(
            factor->linearize(*tx.frozen_values));
        if (!hessian) {
          window.reason =
              "boundary factor is neither a Jacobian nor a Hessian factor";
          return window;
        }
        // Information-form factor (the fixed-lag marginalization stores the
        // condensed prior as a LinearContainerFactor).  Its rows are rebuilt
        // from the information block by Cholesky: [R | c] with R'R = Lambda
        // and c = R^-T eta, which reproduces the quadratic form exactly.  The
        // constant f - c'c cannot be represented by rows at all, so it is
        // accumulated explicitly instead of being silently dropped.
        // `info().selfadjointView()` is the full augmented matrix
        // [Lambda, eta; eta', f] of dimension (dimension + 1).
        const Eigen::MatrixXd augmented_information =
            hessian->info().selfadjointView();
        int dimension = 0;
        for (const auto key : hessian->keys()) dimension += keyDimension(key);
        if (augmented_information.rows() != dimension + 1) {
          window.reason =
              "information-form boundary factor has an unexpected dimension";
          return window;
        }
        const Eigen::MatrixXd lambda =
            augmented_information.topLeftCorner(dimension, dimension);
        const Eigen::VectorXd eta =
            augmented_information.topRightCorner(dimension, 1);
        const double constant =
            augmented_information(dimension, dimension);
        Eigen::LLT<Eigen::MatrixXd> llt(lambda);
        if (llt.info() != Eigen::Success) {
          window.reason =
              "information-form boundary factor is not positive definite";
          return window;
        }
        const Eigen::MatrixXd rows = llt.matrixU();
        const Eigen::VectorXd rhs = llt.matrixU().transpose().solve(eta);
        information_form_offset += constant - rhs.squaredNorm();
        ++information_form_factors;
        std::vector<std::pair<gtsam::Key, Eigen::MatrixXd>> terms;
        int column = 0;
        for (const auto key : hessian->keys()) {
          const int width = keyDimension(key);
          terms.emplace_back(key,
                             Eigen::MatrixXd(rows.middleCols(column, width)));
          column += width;
        }
        nominal_boundary_graphs[group.id.value()]
            .emplace_shared<gtsam::JacobianFactor>(terms, rhs);
        nominal_boundary_rows[group.id.value()] +=
            static_cast<std::size_t>(rows.rows());
      }
      if (fixed_lag_backend_) {
        cache_value.rows = nominal_boundary_graphs[group_value];
        cache_value.row_count = nominal_boundary_rows[group_value];
        cache_value.information_form_factors =
            information_form_factors - information_count_before;
        cache_value.information_form_offset =
            information_form_offset - information_offset_before;
        fixed_lag_backend_->storeHistoryGroup(group_value,
                                              std::move(cache_value));
      }
    }
    if (fixed_lag_backend_) {
      fixed_lag_backend_->retainHistoryGroups(active_boundary_group_ids);
    }
    HistoryFaultParameterizationOptions history_options;
    history_options.include_uwb_faults =
        config_.resolved_scope.requiresUwbFaults();
    history_options.include_imu_faults =
        config_.resolved_scope.requiresImuFaults();
    history_options.scope_digest = config_.resolved_scope.scope_digest;
    const HistoryFaultParameterizationPlan plan =
        planHistoryFaultParameterization(tx, interval_count, history_options);
    WindowHistorySummary& history = window.history_summary;
    history.present = true;
    history.scope_digest = plan.scope_digest;
    history.fault_columns = plan.q_hist();
    history.horizon_first_epoch = plan.horizon.first_epoch;
    history.window_first_epoch = plan.horizon.window_first_epoch;
    history.omitted_epoch_count = plan.omitted_epoch_count;
    history.material_gap_epoch_count = plan.material_gap_epoch_count;
    history.claims_full_coverage = plan.claimsFullCoverage();
    history.assumptions = plan.validityAssumptions();
    history.omitted_risk_source = plan.omittedRiskSource();
    history.skipped_columns = plan.skipped_columns;
    HistoryFaultCapacityLimits capacity_limits;
    capacity_limits.max_fault_columns = config_.history.max_fault_columns;
    capacity_limits.capacity_action = config_.history.capacity_action;
    const HistoryFaultCapacityDecision capacity =
        evaluateHistoryFaultCapacity(plan.q_hist(), capacity_limits);
    history.capacity_ok = capacity.fits;
    history.capacity_action = capacity.action;
    if (!capacity.fits) {
      history.valid = false;
      history.state = toString(HistorySummaryState::CapacityExceeded);
      history.reason = capacity.reason;
      NumericalWorkCounters::historyCapacityRefusal();
      NumericalWorkCounters::historySummaryInvalid();
      window.reason = "history fault capacity refused: " + capacity.reason;
      return window;
    }

    std::set<std::uint64_t> boundary_group_values;
    for (const auto& accounting : window.slot_accounting) {
      if (accounting.boundary_input && accounting.group_id) {
        boundary_group_values.insert(accounting.group_id->value());
      }
    }
    std::map<gtsam::Key, HistoryFaultColumnId> column_id_of_key;
    std::set<std::uint64_t> fault_augmented_groups;
    std::map<std::size_t, const HistoricalEpochContext*> history_by_epoch;
    for (const auto& record : tx.recoverable_history) {
      history_by_epoch[record.proposed_epoch] = &record;
    }
    std::uint64_t next_fault_index = 1;
    for (std::size_t epoch = plan.horizon.first_epoch;
         epoch < plan.horizon.window_first_epoch; ++epoch) {
      const auto record = history_by_epoch.find(epoch);
      if (record == history_by_epoch.end()) {
        // Ordering guard (design freeze §7.1 / readiness item 2): material of
        // an epoch inside the A3 horizon may not disappear before the summary
        // consumed it.  If the frozen boundary still carries factors of that
        // epoch, the deletion happened first and the window must fail instead
        // of silently claiming the interval.
        bool epoch_has_boundary_factors = false;
        for (const auto& group : boundary_groups) {
          for (const auto& factor : group.factors) {
            for (const auto key : factor->keys()) {
              if (gtsam::Symbol(key).index() == epoch) {
                epoch_has_boundary_factors = true;
                break;
              }
            }
            if (epoch_has_boundary_factors) break;
          }
          if (epoch_has_boundary_factors) break;
        }
        if (epoch_has_boundary_factors) {
          history.valid = false;
          history.state = toString(HistorySummaryState::ColdStartInvalid);
          history.reason = "history material for epoch " +
                           std::to_string(epoch) +
                           " was deleted before the summary update";
          NumericalWorkCounters::historySummaryInvalid();
          window.reason = history.reason;
          return window;
        }
        continue;
      }
      std::vector<HistoryFaultColumn> columns;
      for (const auto& column : plan.columns) {
        if (column.id.epoch != epoch) continue;
        if (boundary_group_values.count(column.group.value()) == 0) continue;
        columns.push_back(column);
      }
      if (columns.empty()) continue;
      const HistoryFaultInjectionResult injection =
          buildHistoricalFaultInjection(*record->second, *tx.frozen_values,
                                        columns, next_fault_index);
      if (!injection.valid) {
        history.valid = false;
        history.state = toString(HistorySummaryState::BuildInvalid);
        history.reason = "history fault injection failed: " + injection.reason;
        NumericalWorkCounters::historySummaryInvalid();
        window.reason = history.reason;
        return window;
      }
      std::set<std::uint64_t> epoch_groups;
      for (const auto& column : columns) epoch_groups.insert(column.group.value());
      std::size_t expected_injection_rows = 0;
      for (const auto group_value : epoch_groups) {
        if (!fault_augmented_groups.insert(group_value).second) {
          history.valid = false;
          history.state = toString(HistorySummaryState::BuildInvalid);
          history.reason =
              "historical group was assigned more than one augmented row owner";
          NumericalWorkCounters::historySummaryInvalid();
          window.reason = history.reason;
          return window;
        }
        const auto nominal_rows = nominal_boundary_rows.find(group_value);
        if (nominal_rows == nominal_boundary_rows.end()) {
          history.valid = false;
          history.state = toString(HistorySummaryState::BuildInvalid);
          history.reason = "historical fault group has no nominal row owner";
          NumericalWorkCounters::historySummaryInvalid();
          window.reason = history.reason;
          return window;
        }
        expected_injection_rows += nominal_rows->second;
      }
      if (injection.rows != expected_injection_rows) {
        history.valid = false;
        history.state = toString(HistorySummaryState::BuildInvalid);
        history.reason =
            "augmented historical rows do not match their nominal row owner";
        NumericalWorkCounters::historySummaryInvalid();
        window.reason = history.reason;
        return window;
      }
      for (const auto& factor : injection.graph) augmented.push_back(factor);
      std::set<std::uint64_t> injection_uid_groups;
      for (const auto& column : columns) {
        const std::uint64_t group_value = column.group.value();
        if (!injection_uid_groups.insert(group_value).second) continue;
        const std::size_t rows = nominal_boundary_rows.at(group_value);
        for (std::size_t row = 0; row < rows; ++row) {
          augmented_row_uids.push_back(
              "factor-group:" + std::to_string(group_value) + ":row:" +
              std::to_string(row));
          augmented_row_provenance.push_back(group_value);
        }
      }
      for (std::size_t index = 0; index < columns.size(); ++index) {
        column_id_of_key[injection.fault_keys[index]] = columns[index].id;
      }
      next_fault_index += static_cast<std::uint64_t>(columns.size());
      ++history.injected_epochs;
    }

    // Insert the untouched nominal groups after all replacements are known.
    // Every ledger group now contributes exactly one row block: nominal or
    // fault-augmented, never both.
    for (const auto& item : nominal_boundary_graphs) {
      if (fault_augmented_groups.count(item.first) != 0) continue;
      std::size_t group_row = 0;
      for (const auto& factor : item.second) {
        augmented.push_back(factor);
        const auto jacobian =
            boost::dynamic_pointer_cast<gtsam::JacobianFactor>(factor);
        if (!jacobian) {
          history.valid = false;
          history.state = toString(HistorySummaryState::BuildInvalid);
          history.reason = "staged nominal boundary row is not Jacobian form";
          NumericalWorkCounters::historySummaryInvalid();
          window.reason = history.reason;
          return window;
        }
        for (std::size_t row = 0;
             row < static_cast<std::size_t>(jacobian->rows()); ++row) {
          augmented_row_uids.push_back(
              "factor-group:" + std::to_string(item.first) + ":row:" +
              std::to_string(group_row++));
          augmented_row_provenance.push_back(item.first);
        }
      }
    }

    // extracted.rank has no consumer in the production history path. The
    // complete rows/identities still reach the certified summary elimination.
    double rank_audit_ms=0;
    const bool audit_boundary_rank=std::getenv("UWB_IMU_PL_EXHAUSTIVE_BOUNDARY_RANK")!=nullptr;
    const ExtractedBoundaryRows extracted = extractBoundaryRows(augmented,audit_boundary_rank,&rank_audit_ms);
    if(std::getenv("UWB_IMU_PL_BOUNDARY_RANK_TIMING")) {
      std::printf("boundary_rank_audit epoch=%zu rows=%ld columns=%d computed=%d wall_ms=%.9f\n",
          tx.proposed_epoch,static_cast<long>(extracted.rows.rows()),
          extracted.total_columns,audit_boundary_rank,rank_audit_ms);
    }
    if (!extracted.valid || extracted.rows.rows() == 0 ||
        extracted.total_columns == 0) {
      history.valid = false;
      history.state = toString(HistorySummaryState::BuildInvalid);
      history.reason = "boundary row extraction failed: " + extracted.reason;
      NumericalWorkCounters::historySummaryInvalid();
      window.reason = history.reason;
      return window;
    }
    if (augmented_row_uids.size() !=
            static_cast<std::size_t>(extracted.rows.rows()) ||
        augmented_row_provenance.size() != augmented_row_uids.size()) {
      history.valid = false;
      history.state = toString(HistorySummaryState::BuildInvalid);
      history.reason =
          "boundary raw row UID/provenance census does not match extraction";
      NumericalWorkCounters::historySummaryInvalid();
      window.reason = history.reason;
      return window;
    }
    history.boundary_rows = static_cast<std::size_t>(extracted.rows.rows());
    history.constant_energy = extracted.constant_energy;

    std::vector<int> old_columns;
    std::vector<int> boundary_columns;
    std::vector<int> fault_columns;
    const int extracted_columns = static_cast<int>(extracted.keys.size());
    for (int column = 0; column < extracted_columns; ++column) {
      const gtsam::Key key = extracted.keys[column];
      if (gtsam::Symbol(key).chr() == 'f') {
        if (column_id_of_key.count(key) == 0) {
          history.valid = false;
          history.reason =
              "extracted fault key is not attributable to a column id";
          NumericalWorkCounters::historySummaryInvalid();
          window.reason = history.reason;
          return window;
        }
        fault_columns.push_back(column);
      } else if (window_keys.count(key) != 0) {
        boundary_columns.push_back(column);
      } else {
        old_columns.push_back(column);
      }
    }
    // Canonical boundary column order (epoch ascending, then x < v < b) so the
    // summary's x_b basis maps directly onto the window column layout.
    std::sort(boundary_columns.begin(), boundary_columns.end(),
              [&](int left, int right) {
                const gtsam::Key a = extracted.keys[left];
                const gtsam::Key b = extracted.keys[right];
                const std::uint64_t epoch_a = gtsam::Symbol(a).index();
                const std::uint64_t epoch_b = gtsam::Symbol(b).index();
                if (epoch_a != epoch_b) return epoch_a < epoch_b;
                auto rank = [](char chr) {
                  return chr == 'x' ? 0 : (chr == 'v' ? 1 : 2);
                };
                return rank(gtsam::Symbol(a).chr()) <
                       rank(gtsam::Symbol(b).chr());
              });
    auto concatColumns = [&](const std::vector<int>& columns) {
      int width = 0;
      for (const int column : columns) width += extracted.key_dim[column];
      Eigen::MatrixXd out(extracted.rows.rows(), width);
      int cursor = 0;
      for (const int column : columns) {
        const int dim = extracted.key_dim[column];
        out.middleCols(cursor, dim) =
            extracted.rows.middleCols(extracted.column_begin[column], dim);
        cursor += dim;
      }
      return out;
    };
    HistoryFaultSummaryInput summary_input;
    summary_input.h_old_state = concatColumns(old_columns);
    summary_input.h_boundary = concatColumns(boundary_columns);
    summary_input.fault_map = concatColumns(fault_columns);
    // GTSAM JacobianFactor uses 0.5 ||A x - b||^2.  The summary uses the same
    // H x - z convention, so getb()/the extracted last column is already z.
    summary_input.rhs = extracted.rows.col(extracted.total_columns);
    HistoryRootCacheRequest root_request;
    root_request.input = summary_input;
    root_request.linearization_version = tx.base_version.linpoint_version;
    root_request.ordering_version = tx.base_version.ordering_version;
    root_request.whitening_version = tx.base_version.noise_model_version;
    root_request.marginalization_version = marginalization_count_;
    // Recovery changes are bound by the exact original group-row provenance
    // and byte payload below.  A normal ledger append must not masquerade as
    // a mutation of every retained row; rollback/replacement changes those
    // inputs and therefore invalidates precisely the affected closure.
    root_request.recovery_version = 0;
    std::uint64_t column_digest = 1469598103934665603ULL;
    auto fold_root = [&](const void* data, std::size_t bytes) {
      const auto* source = static_cast<const unsigned char*>(data);
      for (std::size_t i = 0; i < bytes; ++i) {
        column_digest ^= source[i];
        column_digest *= 1099511628211ULL;
      }
    };
    for (std::size_t index = 0; index < extracted.keys.size(); ++index) {
      const auto key = extracted.keys[index];
      if (gtsam::Symbol(key).chr() == 'f') continue;
      fold_root(&key, sizeof(key));
      fold_root(&extracted.key_dim[index], sizeof(extracted.key_dim[index]));
    }
    fold_root(plan.scope_digest.data(), plan.scope_digest.size());
    root_request.column_order_digest = column_digest;
    root_request.row_uids = std::move(augmented_row_uids);
    root_request.row_provenance = std::move(augmented_row_provenance);
    auto append_state_column_uids = [&](const std::vector<int>& columns) {
      for (const int column : columns) {
        const gtsam::Key key = extracted.keys[column];
        for (int local = 0; local < extracted.key_dim[column]; ++local) {
          root_request.state_column_uids.push_back(
              std::to_string(static_cast<std::uint64_t>(key)) + ":" +
              std::to_string(local));
        }
      }
    };
    append_state_column_uids(old_columns);
    append_state_column_uids(boundary_columns);
    for (const int column : fault_columns) {
      const auto& id = column_id_of_key.at(extracted.keys[column]);
      root_request.fault_column_uids.push_back(
          std::to_string(static_cast<std::uint64_t>(id.kind)) + ":" +
          std::to_string(id.source) + ":" + std::to_string(id.epoch));
    }
    HistoryFaultSummaryOptions summary_options;
    summary_options.rank_tolerance = config_.integrity_window.rank_tolerance;
    HistoryCarrierRankCertificate carrier_rank;
    HistoryFaultSummary summary;
    if (fixed_lag_backend_) {
      summary = fixed_lag_backend_->updateHistoryRoot(
          std::move(root_request), &carrier_rank, summary_options);
    } else {
      CertifiedHistoryFaultSummary certified =
          buildCertifiedHistoryFaultSummary(summary_input, summary_options);
      summary = std::move(certified.summary);
      carrier_rank = std::move(certified.carrier_rank);
    }
    history.boundary_columns =
        static_cast<std::size_t>(summary_input.h_boundary.cols());
    const std::uint64_t input_width = static_cast<std::uint64_t>(
        summary_input.h_old_state.cols() + summary_input.h_boundary.cols() +
        summary_input.fault_map.cols() + 1);
    NumericalWorkCounters::historySummaryBuild(
        history.boundary_rows, input_width,
        static_cast<std::uint64_t>(summary_input.fault_map.cols()),
        static_cast<std::uint64_t>(summary.n_boundary + summary.nuPerp()));
    if (!summary.valid) {
      history.valid = false;
      history.state = toString(HistorySummaryState::BuildInvalid);
      history.reason = summary.invalid_reason;
      NumericalWorkCounters::historySummaryInvalid();
      window.reason = "history summary invalid: " + summary.invalid_reason;
      return window;
    }

    // Condensed rows: the boundary triangle [R_b] carries the state content,
    // the detection-only rows [0 | d_perp] carry the state-free residual
    // content (kappa_b = ||d_perp||^2, nu_perp dof).  The perpendicular
    // carrier is already orthogonally compressed using the numerical
    // contract.  Never infer its dof from element-wise nonzero tests.
    if (!carrier_rank.valid ||
        carrier_rank.effective_rank != summary.nuPerp() ||
        summary.F_b.rows() != summary.nuPerp() ||
        summary.d_perp.size() != summary.nuPerp()) {
      history.valid = false;
      history.state = toString(HistorySummaryState::BuildInvalid);
      history.reason = "history residual carrier rank proof is invalid";
      window.reason = history.reason;
      return window;
    }
    std::vector<int> kept_rows;
    std::vector<char> field_is_perp;
    for (int row = 0; row < summary.n_boundary; ++row) {
      const bool nonzero = summary.R_b.row(row).cwiseAbs().maxCoeff() > 0.0 ||
                           std::abs(summary.d_b(row)) > 0.0 ||
                           (summary.n_fault > 0 &&
                            summary.T_b.row(row).cwiseAbs().maxCoeff() > 0.0);
      if (nonzero) {
        kept_rows.push_back(row);
        field_is_perp.push_back(0);
      }
    }
    for (int row = 0; row < summary.nuPerp(); ++row) {
      kept_rows.push_back(summary.n_boundary + row);
      field_is_perp.push_back(1);
    }
    LinearizedFactorBlock boundary;
    boundary.group_id = FactorGroupId(tx.id.value() * 1000);
    boundary.kind = FactorKind::BoundaryPrior;
    boundary.sensor = SensorType::Prior;
    boundary.role = RowRole::TrustedPrior;
    boundary.jacobian_whitened = Eigen::MatrixXd::Zero(
        static_cast<Eigen::Index>(kept_rows.size()), total_columns);
    boundary.residual_whitened =
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kept_rows.size()));
    for (std::size_t index = 0; index < kept_rows.size(); ++index) {
      const int row = kept_rows[index];
      if (field_is_perp[index]) {
        boundary.residual_whitened(static_cast<Eigen::Index>(index)) =
            summary.d_perp(row - summary.n_boundary);
        continue;
      }
      boundary.residual_whitened(static_cast<Eigen::Index>(index)) =
          summary.d_b(row);
      int cursor = 0;
      for (const int column : boundary_columns) {
        const int dim = extracted.key_dim[column];
        const gtsam::Key key = extracted.keys[column];
        const std::size_t epoch = gtsam::Symbol(key).index();
        const int offset = static_cast<int>((epoch - first_epoch) * 15) +
                           (gtsam::Symbol(key).chr() == 'x'
                                ? 0
                                : (gtsam::Symbol(key).chr() == 'v' ? 6 : 9));
        boundary.jacobian_whitened.block(static_cast<Eigen::Index>(index),
                                         offset, 1, dim) =
            summary.R_b.block(row, cursor, 1, dim);
        cursor += dim;
      }
    }
    boundary.jacobian_raw = boundary.jacobian_whitened;
    boundary.residual_raw = boundary.residual_whitened;
    boundary.covariance =
        Eigen::MatrixXd::Identity(static_cast<Eigen::Index>(kept_rows.size()),
                                  static_cast<Eigen::Index>(kept_rows.size()));
    boundary.whitener = boundary.covariance;
    boundary.effective_weight = 1.0;
    boundary.whitening_model_id = "history_summary_sqrt_d1";
    boundary.window_column_indices.resize(total_columns);
    std::iota(boundary.window_column_indices.begin(),
              boundary.window_column_indices.end(), 0);
    boundary.version = tx.base_version;
    window.blocks.push_back(std::move(boundary));
    window.capabilities.includes_boundary_prior = true;
    window.capabilities.history_summary_present = true;
    window.capabilities.history_summary_valid = true;
    window.capabilities.history_summary_capacity_ok = capacity.fits;

    // Fault response carrier: rows align 1:1 with the emitted boundary block,
    // so a historical mode map over the block is [response.col; detector.col].
    std::size_t kept_perp = 0;
    for (const char flag : field_is_perp) kept_perp += flag ? 1u : 0u;
    history.emitted_rows = kept_rows.size();
    history.nu_perp = static_cast<int>(kept_perp);
    history.rank_boundary = summary.rank_boundary;
    const std::size_t kept_supported = kept_rows.size() - kept_perp;
    history.response = Eigen::MatrixXd::Zero(
        static_cast<Eigen::Index>(kept_supported), summary.n_fault);
    history.detector_response = Eigen::MatrixXd::Zero(
        static_cast<Eigen::Index>(kept_perp), summary.n_fault);
    history.d_perp = Eigen::VectorXd::Zero(
        static_cast<Eigen::Index>(kept_perp));
    {
      std::size_t boundary_row = 0;
      std::size_t perp_row = 0;
      for (std::size_t index = 0; index < kept_rows.size(); ++index) {
        const int row = kept_rows[index];
        if (field_is_perp[index]) {
          history.detector_response.row(static_cast<Eigen::Index>(perp_row)) =
              summary.F_b.row(row - summary.n_boundary);
          history.d_perp(static_cast<Eigen::Index>(perp_row)) =
              summary.d_perp(row - summary.n_boundary);
          ++perp_row;
        } else {
          history.response.row(static_cast<Eigen::Index>(boundary_row)) =
              summary.T_b.row(row);
          ++boundary_row;
        }
      }
    }
    history.kappa_b = history.d_perp.squaredNorm();
    history.constant_offset = information_form_offset;
    history.information_form_factors = information_form_factors;
    for (const int column : fault_columns) {
      const HistoryFaultColumnId id =
          column_id_of_key.at(extracted.keys[column]);
      history.column_ids.push_back(id);
      if (id.kind == HistoryFaultBasisKind::UwbAnchorConstant) {
        ++history.constant_columns;
      } else if (id.kind == HistoryFaultBasisKind::UwbAnchorTimeLinear) {
        ++history.time_linear_columns;
      } else {
        ++history.imu_columns;
      }
    }
    NumericalWorkCounters::historyPerpRows(kept_perp);

    // Binding digest (design freeze §3/§5): fold every component that can
    // change the meaning of the summary.  The digest is what caches and the
    // FrozenWindowNumerics fingerprint carry, so a changed whitening / mode
    // set / linearization / capacity can never be served from a stale entry.
    auto fnv = [](std::uint64_t hash, const void* data, std::size_t bytes) {
      const auto* bytes_ptr = static_cast<const unsigned char*>(data);
      for (std::size_t index = 0; index < bytes; ++index) {
        hash ^= bytes_ptr[index];
        hash *= 1099511628211ULL;
      }
      return hash;
    };
    const std::uint64_t fnv_offset = 1469598103934665603ULL;
    HistorySummaryVersion summary_version;
    {
      std::uint64_t hash = fnv_offset;
      const std::uint64_t fields[4] = {
          window.version.graph_version, window.version.ordering_version,
          window.version.noise_model_version, window.version.linpoint_version};
      hash = fnv(hash, fields, sizeof(fields));
      summary_version.linearization = hash;
    }
    {
      std::uint64_t hash = fnv_offset;
      std::set<std::string> whitening_ids;
      for (const auto& item : historical_groups) {
        whitening_ids.insert(item.second.group->noise_model_id);
      }
      for (const auto& id_text : whitening_ids) {
        hash = fnv(hash, id_text.data(), id_text.size());
      }
      summary_version.whitening = hash;
    }
    {
      std::uint64_t hash = fnv_offset;
      std::vector<HistoryFaultColumnId> ids = history.column_ids;
      std::sort(ids.begin(), ids.end(),
                [](const HistoryFaultColumnId& a,
                   const HistoryFaultColumnId& b) { return a < b; });
      for (const auto& id : ids) {
        const std::uint64_t tuple[3] = {static_cast<std::uint64_t>(id.kind),
                                        id.source, id.epoch};
        hash = fnv(hash, tuple, sizeof(tuple));
      }
      const std::uint64_t counts[4] = {
          plan.horizon.first_epoch, plan.horizon.window_first_epoch,
          static_cast<std::uint64_t>(plan.q_hist()),
          static_cast<std::uint64_t>(plan.skipped_columns)};
      hash = fnv(hash, counts, sizeof(counts));
      hash = fnv(hash, plan.scope_digest.data(), plan.scope_digest.size());
      hash = fnv(hash, &carrier_rank.proof_identity,
                 sizeof(carrier_rank.proof_identity));
      summary_version.mode_set = hash;
    }
    {
      std::uint64_t hash = fnv_offset;
      hash = fnv(hash, &capacity_limits.max_fault_columns,
                 sizeof(capacity_limits.max_fault_columns));
      hash = fnv(hash, capacity_limits.capacity_action.data(),
                 capacity_limits.capacity_action.size());
      summary_version.capacity = hash;
    }
    history.version = summary_version;
    history.version_digest = digestHistorySummaryVersion(summary_version);
    history.valid = true;
    history.state = toString(HistorySummaryState::Valid);
  }

  const auto factor_start = std::chrono::steady_clock::now();
  window.preparation_timing.boundary_and_provenance_ms =
      std::chrono::duration<double, std::milli>(factor_start -
                                                preparation_start)
          .count();

  auto append_group = [&](const PendingFactorGroup& group,
                          EpochTransaction linearization_tx) {
    linearization_tx.base_version = window.version;
    linearization_tx.backend_mutated = false;
    const std::uint64_t fingerprint =
        factorBlockFingerprint(group, linearization_tx);
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
      block = linearizePendingGroup(group, linearization_tx, local.first,
                                    local.second);
      FactorBlockCacheEntry entry;
      entry.transaction_id = tx.id;
      entry.version = window.version;
      entry.content_fingerprint = fingerprint;
      entry.block = block;
      factor_block_cache_[group.id.value()] = std::move(entry);
    }
    Eigen::MatrixXd embedded_h =
        Eigen::MatrixXd::Zero(block.jacobian_whitened.rows(), total_columns);
    Eigen::MatrixXd embedded_raw =
        Eigen::MatrixXd::Zero(block.jacobian_raw.rows(), total_columns);
    for (const auto key : group.keys) {
      const std::size_t epoch = gtsam::Symbol(key).index();
      if (epoch < first_epoch || epoch > tx.proposed_epoch) {
        window.reason = "explicit group key is outside integrity window";
        return;
      }
      const int source = transactionColumn(linearization_tx, key);
      const int destination = static_cast<int>((epoch - first_epoch) * 15) +
                              (gtsam::Symbol(key).chr() == 'x'
                                   ? 0
                                   : (gtsam::Symbol(key).chr() == 'v' ? 6 : 9));
      const int dimension = keyDimension(key);
      embedded_h.block(0, destination, embedded_h.rows(), dimension) =
          block.jacobian_whitened.block(
              0, source, block.jacobian_whitened.rows(), dimension);
      embedded_raw.block(0, destination, embedded_raw.rows(), dimension) =
          block.jacobian_raw.block(0, source, block.jacobian_raw.rows(),
                                   dimension);
    }
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
    if (history.proposed_epoch < first_epoch ||
        history.proposed_epoch > tx.previous_epoch)
      continue;
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
      if (explicit_group_ids.count(selected_group.value()) == 0) continue;
      const auto group = std::find_if(groups.begin(), groups.end(),
                                      [&](const PendingFactorGroup* candidate) {
                                        return candidate->id == selected_group;
                                      });
      if (group == groups.end()) {
        window.reason =
            "committed group provenance is incomplete: explicit group " +
            std::to_string(selected_group.value()) + " is absent from epoch " +
            std::to_string(history.proposed_epoch) + " catalog";
        return window;
      }
      append_group(**group, linearization_tx);
    }
  }

  if (request.include_pending_imu) {
    append_group(tx.imu_group, tx);
    window.factor_inventory.push_back({tx.imu_group.id,
                                       tx.proposed_epoch,
                                       tx.imu_group.kind,
                                       tx.imu_group.sensor,
                                       FrozenFactorDisposition::PendingExplicit,
                                       tx.imu_group.keys,
                                       {}});
    window.capabilities.includes_pending_imu = true;
  }
  if (request.include_pending_uwb) {
    for (const auto& group : tx.uwb_groups) {
      if (!group.nominal) continue;
      append_group(group, tx);
      window.factor_inventory.push_back(
          {group.id,
           tx.proposed_epoch,
           group.kind,
           group.sensor,
           FrozenFactorDisposition::PendingExplicit,
           group.keys,
           {}});
    }
    window.capabilities.includes_pending_uwb = !tx.uwb_groups.empty();
  }
  if (request.include_generic_bridge) {
    append_group(tx.generic_bridge_group, tx);
    append_group(tx.generic_bias_continuity_group, tx);
    for (const auto* group :
         {&tx.generic_bridge_group, &tx.generic_bias_continuity_group}) {
      window.factor_inventory.push_back(
          {group->id,
           tx.proposed_epoch,
           group->kind,
           group->sensor,
           FrozenFactorDisposition::PendingExplicit,
           group->keys,
           {}});
    }
  }
  window.protected_state_map = Eigen::MatrixXd::Zero(3, total_columns);
  window.protected_state_map.block<3, 6>(0, total_columns - 15) =
      worldPositionPoseTangentJacobian(
          transactionValues(tx).at<gtsam::Pose3>(poseKey(tx.proposed_epoch)));
  window.capabilities.complete_factor_provenance =
      factor_ledger_.hasCompleteActiveProvenance(activeGraph());
  window.capabilities.history_provenance_valid =
      tx.previous_epoch == 0 || window.capabilities.complete_factor_provenance;
  window.capabilities.fixed_lag_maturity_valid =
      config_.incremental.fixed_lag_epochs == 0 ||
      config_.incremental.fixed_lag_epochs >
          request.epochs + config_.integrity_window.recovery_margin_epochs;
  window.preparation_timing.factor_linearization_whitening_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - factor_start)
          .count();
  finalizeIntegrityWindow(&window, config_.integrity_window.rank_tolerance,
                          config_.integrity_window.max_condition_number);
  if (std::getenv("UWB_IMU_PL_C1P_DIAG") != nullptr) {
    const auto& history = window.history_summary;
    std::fprintf(stderr,
                 "[C1P-DIAG] tx=%llu first=%zu window_first=%zu rows=%ld rank=%d "
                 "dof=%d blocks=%zu | history present=%d valid=%d state=%s "
                 "boundary_rows=%zu "
                 "emitted=%zu nu_perp=%d rank_boundary=%d kappa=%.6e offset=%.3e "
                 "info_form=%zu q=%zu injected=%zu capacity=%d | boundary_ms=%.3f "
                 "assembly_ms=%.3f\n",
                 static_cast<unsigned long long>(tx.id.value()), first_epoch,
                 interval_count, static_cast<long>(window.H.rows()), window.rank,
                 window.dof, window.blocks.size(), history.present ? 1 : 0,
                 history.valid ? 1 : 0, history.state.c_str(),
                 history.boundary_rows,
                 history.emitted_rows, history.nu_perp, history.rank_boundary,
                 history.kappa_b, history.constant_offset,
                 history.information_form_factors, history.fault_columns,
                 history.injected_epochs, history.capacity_ok ? 1 : 0,
                 window.preparation_timing.boundary_and_provenance_ms,
                 window.preparation_timing.dense_assembly_ms);
  }
  if (!window.capabilities.fixed_lag_maturity_valid) {
    window.model_valid = false;
    window.reason = "fixed lag violates integrity-window maturity delay";
  }
  return window;
}

FrozenIntegrityWindow IncrementalUwbImuEstimator::buildFrozenIntegrityWindow(
    const EpochTransaction& tx, const IntegrityWindowRequest& request) const {
  LinearizedIntegrityWindow builder = buildIntegrityWindow(tx, request);
  return freezeIntegrityWindow(std::move(builder));
}

LinearizedFactorBlock IncrementalUwbImuEstimator::buildPendingFactorBlock(
    const EpochTransaction& tx, FactorGroupId id) const {
  if (!active_transaction_id_ || *active_transaction_id_ != tx.id ||
      tx.backend_mutated || tx.base_graph_version != graph_version_) {
    throw std::logic_error("pending block requires active frozen transaction");
  }
  std::vector<const PendingFactorGroup*> groups{
      &tx.imu_group, &tx.generic_bridge_group,
      &tx.generic_bias_continuity_group};
  for (const auto& group : tx.uwb_groups) groups.push_back(&group);
  if (tx.dynamics_bridge_group) groups.push_back(&*tx.dynamics_bridge_group);
  for (const auto& history : tx.recoverable_history) {
    for (const auto& group : history.groups) groups.push_back(&group);
  }
  const auto found = std::find_if(
      groups.begin(), groups.end(),
      [&](const PendingFactorGroup* group) { return group->id == id; });
  if (found == groups.end()) {
    throw std::out_of_range("unknown pending factor group");
  }
  EpochTransaction local = tx;
  for (const auto& history : tx.recoverable_history) {
    const auto historical = std::find_if(
        history.groups.begin(), history.groups.end(),
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
  LinearizedFactorBlock block =
      linearizePendingGroup(**found, local, linear.first, linear.second);
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

HistoryRootCacheAudit
IncrementalUwbImuEstimator::historyRootCacheAuditForTesting() const {
  return fixed_lag_backend_ ? fixed_lag_backend_->historyRootAudit()
                            : HistoryRootCacheAudit{};
}

void IncrementalUwbImuEstimator::enableHistoryRootOracleForTesting(
    bool enabled) {
  if (fixed_lag_backend_)
    fixed_lag_backend_->enableHistoryRootOracleForTesting(enabled);
}

void IncrementalUwbImuEstimator::corruptHistoryTreeRootForTesting() {
  if (!fixed_lag_backend_) {
    throw std::logic_error("history root mutation requires fixed lag");
  }
  fixed_lag_backend_->corruptHistoryTreeRootForTesting();
}

void IncrementalUwbImuEstimator::resetHistoryTreeForTesting() {
  if (!fixed_lag_backend_) {
    throw std::logic_error("history root reset requires fixed lag");
  }
  fixed_lag_backend_->resetHistoryTreeForTesting();
}

void IncrementalUwbImuEstimator::queryCurrentState() {
  current_state_ = queryState(epoch_, state_timestamp_);
}

std::vector<NavigationState> IncrementalUwbImuEstimator::retainedSmoothedStatesV1() const {
  std::vector<NavigationState> states;
  if (!initialized_) return states;
  if (backend_poisoned_) throw std::logic_error("historical export on poisoned backend");
  const auto& theta=backendIsam().getLinearizationPoint();
  for (const auto& item:state_history_) {
    if (!theta.exists(poseKey(item.first))) continue;
    states.push_back(queryState(item.first,item.second.timestamp));
  }
  return states;
}

NavigationState IncrementalUwbImuEstimator::queryState(
    std::size_t requested_epoch, TimestampNs timestamp) const {
  // A Bayes-tree backsolve returns the full Newton step, NOT the step accepted
  // by Dogleg. Read the accepted delta once (GTSAM caches it until update) and
  // retract only the three requested variables. The covariance still uses
  // the Gaussian tree; it must not be confused with the nonlinear mean.
  const gtsam::KeyVector keys{poseKey(requested_epoch),
                              velocityKey(requested_epoch),
                              biasKey(requested_epoch)};
  const gtsam::ISAM2& isam = backendIsam();
  const gtsam::VectorValues* accepted_delta = nullptr;
  {
    std::lock_guard<std::mutex> lock(estimation_runtime_mutex);
    auto& runtime = estimation_runtime.at(this);
    if (!runtime.accepted_delta || runtime.accepted_delta_version != backend_update_count_) {
      runtime.accepted_delta = &isam.getDelta();
      runtime.accepted_delta_version = backend_update_count_;
    }
    accepted_delta = runtime.accepted_delta;
  }
  const gtsam::VectorValues& local_delta = *accepted_delta;
  const gtsam::Values& theta = isam.getLinearizationPoint();
  const gtsam::Pose3 pose = gtsam::traits<gtsam::Pose3>::Retract(
      theta.at<gtsam::Pose3>(keys[0]), local_delta.at(keys[0]));
  const gtsam::Vector3 velocity = gtsam::traits<gtsam::Vector3>::Retract(
      theta.at<gtsam::Vector3>(keys[1]), local_delta.at(keys[1]));
  const auto bias = gtsam::traits<gtsam::imuBias::ConstantBias>::Retract(
      theta.at<gtsam::imuBias::ConstantBias>(keys[2]), local_delta.at(keys[2]));
  NavigationState state;
  state.id = StateId(requested_epoch);
  state.timestamp = timestamp;
  state.position_world_m = pose.translation();
  state.q_world_body = Eigen::Quaterniond(pose.rotation().matrix());
  state.velocity_world_mps = velocity;
  state.accel_bias_mps2 = bias.accelerometer();
  state.gyro_bias_radps = bias.gyroscope();
  return state;
}

Eigen::Matrix<double, 15, 15> IncrementalUwbImuEstimator::currentJointMarginal()
    const {
  return jointMarginal(epoch_);
}

Eigen::Matrix<double, 15, 15> IncrementalUwbImuEstimator::jointMarginal(
    std::size_t requested_epoch) const {
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
        zero_rhs.insert(*item,
                        Eigen::VectorXd::Zero(conditional->getDim(item)));
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
        covariance.block(offsets[row_key], output_column, dimensions[row_key],
                         1) = inverse_column.at(keys[row_key]);
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
    throw std::logic_error(
        "pre-measurement snapshot requires matching predicted epoch");
  }
  validateUwbBatch(batch);
  if (adapter_transaction_->uwb_groups.empty()) {
    attachUwbGroups(&*adapter_transaction_, batch, true);
  }
  CurrentStatePrior prior = pendingCurrentPrior(*adapter_transaction_);
  pending_batch_id_ = batch.id;
  if (!prior.excludes_current_uwb)
    throw std::logic_error("current UWB already contaminates prior");
  const gtsam::Pose3 pose =
      toGtsamPose(adapter_transaction_->nominal_predicted_state);
  UwbPoseBatchFactor proposed(poseKey(adapter_transaction_->proposed_epoch),
                              batch, lever_arm_body_m_);
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
  diagnostics.rank = static_cast<int>(
      (svd.singularValues().array() > config_.snapshot.rank_tolerance).count());
  diagnostics.condition_number = smallest > 0.0
                                     ? svd.singularValues()(0) / smallest
                                     : std::numeric_limits<double>::infinity();
  diagnostics.covariance_valid = diagnostics.rank == 15;
  diagnostics.model_valid =
      diagnostics.covariance_valid &&
      diagnostics.condition_number <= config_.snapshot.max_condition_number;
  if (!diagnostics.model_valid)
    diagnostics.reason = "pre-UWB prior marginal is ill-conditioned";
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
  const Eigen::Matrix<double, 15, 15> information =
      information_solve.solve(Eigen::Matrix<double, 15, 15>::Identity());
  auto snapshot = std::make_shared<ImmutableEstimationSnapshot>(
      prior.mean, prior.version, diagnostics,
      std::vector<WhitenedRowBlock>{rows}, capabilities,
      LinearizationConsistency::Strict, prior, prior.covariance, information);
  last_snapshot_extraction_ms_ = elapsedMs(snapshot_start);
  return snapshot;
}

CommitReceipt IncrementalUwbImuEstimator::commitEpoch(
    EpochTransaction&& tx, const EpochCommitPlan& plan) {
  return commitEpochCertified(std::move(tx), plan, nullptr, nullptr);
}

CommitReceipt IncrementalUwbImuEstimator::commitEpochCertified(
    EpochTransaction&& tx, const EpochCommitPlan& plan,
    const CommitProtectionEvidenceV1* protection_evidence,
    CommitCertificationV1* certification) {
  if (backendPoisoned()) throw std::runtime_error("backend is fail-closed");
  if (!active_transaction_id_ || *active_transaction_id_ != tx.id ||
      tx.backend_mutated || tx.base_graph_version != graph_version_ ||
      !(tx.base_version == LinearizationVersion{graph_version_,
                                                ordering_version_, 1,
                                                linpoint_version_})) {
    throw std::logic_error("BACKEND_VERSION_MISMATCH or repeated transaction");
  }
  const auto start = std::chrono::steady_clock::now();

  // Everything that can be validated, allocated, or copied without touching
  // iSAM is completed before arming the mutation guard.  After the update the
  // code below writes only staged copies until the complete receipt exists.
  gtsam::NonlinearFactorGraph graph;
  std::vector<const PendingFactorGroup*> all_groups;
  all_groups.push_back(&tx.imu_group);
  for (const auto& group : tx.uwb_groups) all_groups.push_back(&group);
  all_groups.push_back(&tx.generic_bridge_group);
  all_groups.push_back(&tx.generic_bias_continuity_group);
  if (tx.dynamics_bridge_group)
    all_groups.push_back(&*tx.dynamics_bridge_group);
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
    const auto found = std::find_if(
        all_groups.begin(), all_groups.end(),
        [&](const PendingFactorGroup* group) { return group->id == id; });
    if (found == all_groups.end())
      throw std::logic_error("commit plan references unknown add group");
    selected_groups.push_back(*found);
    for (const auto& factor : (*found)->factors) graph.push_back(factor);
  }
  const bool has_imu = selected_ids.count(tx.imu_group.id.value()) != 0;
  const bool has_pose_bridge =
      selected_ids.count(tx.generic_bridge_group.id.value()) != 0;
  const bool has_bias_bridge =
      selected_ids.count(tx.generic_bias_continuity_group.id.value()) != 0;
  const bool has_bridge =
      has_pose_bridge ||
      (tx.dynamics_bridge_group &&
       selected_ids.count(tx.dynamics_bridge_group->id.value()) != 0);
  if (has_imu == has_bridge || has_pose_bridge != has_bias_bridge) {
    throw std::logic_error(
        "commit plan must select IMU or a complete pose/bias bridge pair");
  }
  std::vector<std::size_t> remove_slots;
  for (const auto group : plan.groups_to_remove) {
    const auto slots = factor_ledger_.activeSlots(group);
    if (slots.empty())
      throw std::logic_error(
          "commit plan cannot remove inactive/unknown group");
    remove_slots.insert(remove_slots.end(), slots.begin(), slots.end());
  }
  for (const auto& history : tx.recoverable_history) {
    const auto removed_imu =
        std::find(plan.groups_to_remove.begin(), plan.groups_to_remove.end(),
                  history.groups.front().id) != plan.groups_to_remove.end();
    if (!removed_imu) continue;
    int historical_pose_bridges = 0;
    int historical_bias_bridges = 0;
    for (const auto& group : history.groups) {
      if (!group.replaces_group ||
          *group.replaces_group != history.groups.front().id ||
          selected_ids.count(group.id.value()) == 0)
        continue;
      historical_pose_bridges += group.kind == FactorKind::KinematicBridge;
      historical_bias_bridges += group.kind == FactorKind::BiasContinuity;
    }
    if (historical_pose_bridges != 1 || historical_bias_bridges != 1) {
      throw std::logic_error(
          "historical IMU removal requires exactly one complete bridge pair");
    }
  }
  std::sort(remove_slots.begin(), remove_slots.end());
  if (std::adjacent_find(remove_slots.begin(), remove_slots.end()) !=
      remove_slots.end()) {
    throw std::logic_error(
        "commit plan attempts duplicate factor-slot removal");
  }
  CommitCertificationV1 staged_certification;
  bool protection_evidence_valid = false;
  if (protection_evidence) {
    std::string evidence_reason;
    protection_evidence_valid = consumeCommitProtectionEvidenceV1(
        *protection_evidence, tx, config_.realtime.world_frame,
        &evidence_reason);
    if (!protection_evidence_valid) {
      throw std::invalid_argument(
          "commit protection rejected before mutation: " + evidence_reason);
    }
    staged_certification.protected_reference =
        protection_evidence->reference;
    staged_certification.protection_token_identity =
        protection_evidence->token_identity;
  }
  gtsam::Values values;
  const auto runtime = readEstimationRuntime(this);
  const bool nominal_only = config_.fde.profile == FdeProfile::Off &&
      !tx.preparation.build_integrity_material &&
      !tx.preparation.build_uwb_recovery_material &&
      !tx.preparation.build_imu_recovery_material &&
      !tx.preparation.build_recoverable_history && has_imu && !has_bridge;
  const NavigationState& seed = nominal_only && runtime.tuning.nominal_initial_guess == "imu"
      ? tx.nominal_predicted_state : tx.cv_predicted_state;
  values.insert(poseKey(tx.proposed_epoch), toGtsamPose(seed));
  values.insert(velocityKey(tx.proposed_epoch),
                seed.velocity_world_mps);
  values.insert(biasKey(tx.proposed_epoch), toGtsamBias(seed));
  if (nominal_only && runtime.tuning.nominal_lm_iterations > 0) {
    const auto warm_start = std::chrono::steady_clock::now();
    double before=0.0, after=0.0;
    bool accepted=false;
    try {
      gtsam::NonlinearFactorGraph local;
      const auto* imu_factor=dynamic_cast<const gtsam::CombinedImuFactor*>(tx.imu_group.factors.front().get());
      if (!imu_factor) throw std::logic_error("warm start requires CombinedImuFactor");
      local.add(boost::make_shared<FixedPreviousImuWarmStart>(*imu_factor,tx.previous_state));
      for (const auto& group:tx.uwb_groups)
        if (group.nominal) for (const auto& factor:group.factors) local.push_back(factor);
      before=local.error(values); after=before;
      gtsam::LevenbergMarquardtParams params;
      params.maxIterations=runtime.tuning.nominal_lm_iterations;
      const auto refined=gtsam::LevenbergMarquardtOptimizer(local,values,params).optimize();
      after=local.error(refined);
      const auto refined_state=stateFromValues(refined,tx.proposed_epoch,tx.end);
      if (std::isfinite(after) && after < before &&
          refined_state.position_world_m.allFinite() &&
          refined_state.velocity_world_mps.allFinite() &&
          refined_state.accel_bias_mps2.allFinite() &&
          refined_state.gyro_bias_radps.allFinite()) {
        values=refined; accepted=true;
      }
    } catch (const std::exception&) {
      // Warm-start failure is reversible: values retain the IMU prediction.
      after=before;
    }
    std::lock_guard<std::mutex> lock(estimation_runtime_mutex);
    auto& audit=estimation_runtime.at(this).audit;
    ++audit.warm_start_attempts;
    audit.warm_start_accepted+=accepted ? 1 : 0;
    audit.last_warm_start_before=before; audit.last_warm_start_after=after;
    audit.last_warm_start_ms=elapsedMs(warm_start);
  }
  FactorLedger staged_ledger = factor_ledger_;
  auto staged_committed_epochs = committed_epochs_;
  auto staged_committed_batches = committed_uwb_batches_;
  auto staged_state_history = state_history_;
  auto staged_prefix_covariances = prefix_covariances_;
  auto staged_imu_queue = imu_queue_;
  const auto staged_imu_boundary = tx.imu_cursor.new_boundary;
  EpochTransaction committed_transaction = tx;
  committed_transaction.frozen_graph.reset();
  committed_transaction.frozen_values.reset();
  committed_transaction.frozen_slots.clear();
  committed_transaction.recoverable_history.clear();
  std::vector<FactorGroupId> current_selected_groups;
  current_selected_groups.reserve(plan.groups_to_add.size());
  CommitReceipt receipt;
  receipt.added_factor_slots.reserve(graph.size());
  receipt.removed_factor_slots.reserve(remove_slots.size());
  receipt.group_to_slots.clear();

  auto inject = [&](CommitFaultPoint point, const char* boundary) {
    const std::uint64_t injected_nonce = hitP005Fault(this, point);
    if (injected_nonce != 0) {
      throw std::runtime_error(std::string("injected commit failure at ") +
                               boundary + "; nonce=" +
                               std::to_string(injected_nonce));
    }
  };
  inject(CommitFaultPoint::BeforeBackendUpdate, "before_backend_update");

  const std::uint64_t updates_before = backend_update_count_;
  BackendUpdateAudit update;
  std::string boundary = "backend_update";
  tx.backend_mutated = true;  // guard is armed before iSAM can mutate
  try {
    {
      std::lock_guard<std::mutex> lock(estimation_runtime_mutex);
      estimation_runtime.at(this).physical_time_s = tx.end.seconds();
    }
    update = backendUpdate(graph, values, tx.proposed_epoch, true, remove_slots);
    boundary = "after_backend_update";
    inject(CommitFaultPoint::AfterBackendUpdate, boundary.c_str());

    const std::size_t staged_epoch = tx.proposed_epoch;
    const TimestampNs staged_timestamp = tx.end;
    const std::uint64_t staged_graph_version = graph_version_ + 1;
    const std::uint64_t staged_linpoint_version = linpoint_version_ + 1;
    const std::uint64_t staged_ordering_version =
        ordering_version_ + (update.marginalized_epochs > 0 ? 1 : 0);
    const std::uint64_t staged_marginalization_count =
        marginalization_count_ + update.marginalized_epochs;
    const LinearizationVersion staged_version{
        staged_graph_version, staged_ordering_version, 1,
        staged_linpoint_version};
    const std::uint32_t retained = fixed_lag_backend_
        ? static_cast<std::uint32_t>(fixed_lag_backend_->timestamps().size() / 3)
        : static_cast<std::uint32_t>(staged_epoch + 1);
    const std::size_t backend_oldest = retained == 0
        ? staged_epoch : staged_epoch + 1 - retained;

    for (std::size_t i = 0; i < tx.imu_cursor.consume_count; ++i) {
      if (staged_imu_queue.empty()) {
        throw std::logic_error("staged IMU cursor exceeds queue");
      }
      staged_imu_queue.pop_front();
    }

    boundary = "state_query";
    inject(CommitFaultPoint::BeforeStateQuery, boundary.c_str());
    const auto state_query_start = std::chrono::steady_clock::now();
    NavigationState staged_current_state =
        queryState(staged_epoch, staged_timestamp);
    const double staged_state_query_ms = elapsedMs(state_query_start);

    boundary = "marginal_query";
    inject(CommitFaultPoint::BeforeMarginalQuery, boundary.c_str());
    const Eigen::Matrix<double, 15, 15> staged_current_covariance =
        jointMarginal(staged_epoch);
    staged_state_history[staged_epoch] = staged_current_state;
    staged_prefix_covariances[staged_epoch] = staged_current_covariance;

    // Historical replacement can alter retained nonlinear means.  Those
    // means are written only to the staged catalog.
    if (!plan.groups_to_remove.empty()) {
      const gtsam::Values recovered_values = backendIsam().calculateEstimate();
      for (auto& item : staged_state_history) {
        if (!recovered_values.exists(poseKey(item.first)) ||
            !recovered_values.exists(velocityKey(item.first)) ||
            !recovered_values.exists(biasKey(item.first))) {
          continue;
        }
        item.second = stateFromValues(recovered_values, item.first,
                                      item.second.timestamp);
      }
      staged_current_state = staged_state_history.at(staged_epoch);
    }

    boundary = "ledger_bind";
    inject(CommitFaultPoint::BeforeLedgerBind, boundary.c_str());
    staged_ledger.recordSelected(tx, plan.groups_to_add);
    for (const auto& item : plan.group_health) {
      staged_ledger.setHealth(FactorGroupId(item.first), item.second);
    }
    const auto& active = activeGraph();
    staged_ledger.markSlotsAbsent(active, backend_oldest);

    boundary = "slot_bind";
    inject(CommitFaultPoint::BeforeSlotBind, boundary.c_str());
    std::set<std::size_t> previously_occupied(remove_slots.begin(),
                                              remove_slots.end());
    std::set<std::size_t> reused_slots;
    for (const auto* group : selected_groups) {
      std::vector<std::size_t> slots;
      slots.reserve(group->factors.size());
      for (const auto& expected : group->factors) {
        const auto found = std::find_if(
            active.begin(), active.end(), [&](const auto& factor) {
              return factor && factor.get() == expected.get();
            });
        if (found == active.end()) {
          // A sparse nominal epoch may marginalize its previous state in the
          // same seconds-based update. Its just-added IMU factor is then
          // represented by the Schur boundary, rather than an active slot.
          // FDE binding remains strict and unchanged.
          const bool marginalized_now = nominal_only && runtime.tuning.nominal_lag_s > 0.0 &&
              std::any_of(expected->keys().begin(), expected->keys().end(),
                  [&](gtsam::Key key) {
                    return std::find(update.marginalized_keys.begin(),
                        update.marginalized_keys.end(),key)!=update.marginalized_keys.end();
                  });
          if (marginalized_now) continue;
          throw std::runtime_error(
              "committed factor identity missing from backend");
        }
        slots.push_back(static_cast<std::size_t>(found - active.begin()));
      }
      if (slots.empty() && nominal_only && runtime.tuning.nominal_lag_s > 0.0) {
        staged_ledger.transition(group->id, FactorLifecycle::Marginalized,
                                 staged_version);
      } else {
        staged_ledger.activate(group->id, slots, staged_version);
      }
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
      staged_ledger.transition(
          group,
          replacement ? FactorLifecycle::SupersededByBridge
                      : FactorLifecycle::RemovedByFde,
          staged_version, replacement);
    }
    staged_ledger.syncBoundaryFactors(active, update.boundary_factor_slots,
                                      staged_timestamp, staged_epoch,
                                      staged_version);
    if (fixed_lag_backend_) {
      staged_ledger.pruneInactiveBefore(backend_oldest);
    }

    for (auto& committed : staged_committed_epochs) {
      std::vector<FactorGroupId> removed_from_record;
      for (const auto removed : plan.groups_to_remove) {
        auto selected = std::find(committed.selected_groups.begin(),
                                  committed.selected_groups.end(), removed);
        if (selected != committed.selected_groups.end()) {
          committed.selected_groups.erase(selected);
          removed_from_record.push_back(removed);
        }
      }
      if (removed_from_record.empty()) continue;
      for (const auto* group : selected_groups) {
        if (group->replaces_group &&
            std::find(removed_from_record.begin(), removed_from_record.end(),
                      *group->replaces_group) != removed_from_record.end()) {
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

    // A recovery commit may add replacement groups that belong to older
    // committed epochs.  Those groups are transferred to the corresponding
    // records above and must not also be advertised as selected by the current
    // epoch: the current transaction does not own their factor catalog.
    std::set<std::uint64_t> current_catalog;
    current_catalog.insert(tx.imu_group.id.value());
    for (const auto& group : tx.uwb_groups) {
      current_catalog.insert(group.id.value());
    }
    current_catalog.insert(tx.generic_bridge_group.id.value());
    current_catalog.insert(tx.generic_bias_continuity_group.id.value());
    if (tx.dynamics_bridge_group) {
      current_catalog.insert(tx.dynamics_bridge_group->id.value());
    }
    for (const auto group : plan.groups_to_add) {
      if (current_catalog.count(group.value()) != 0) {
        current_selected_groups.push_back(group);
      }
    }
    staged_committed_epochs.push_back(CommittedEpochRecord{
        std::move(committed_transaction), std::move(current_selected_groups)});

    bool staged_current_uwb_committed = false;
    for (const auto& group : tx.uwb_groups) {
      if (selected_ids.count(group.id.value())) {
        staged_current_uwb_committed = true;
      }
    }
    if (staged_current_uwb_committed) {
      staged_committed_batches.emplace_back(staged_epoch, tx.uwb_batch.id);
    }

    boundary = "metadata_prune";
    inject(CommitFaultPoint::BeforeMetadataPrune, boundary.c_str());
    const std::size_t configured_history =
        static_cast<std::size_t>(config_.integrity_window.epochs) +
        static_cast<std::size_t>(
            config_.integrity_window.recovery_margin_epochs) + 1;
    std::size_t oldest = staged_epoch > configured_history
        ? staged_epoch - configured_history : 0;
    if (fixed_lag_backend_) oldest = std::max(oldest, backend_oldest);
    while (!staged_committed_batches.empty() &&
           staged_committed_batches.front().first < oldest) {
      staged_committed_batches.pop_front();
    }
    while (!staged_committed_epochs.empty() &&
           staged_committed_epochs.front().transaction.proposed_epoch <= oldest) {
      staged_committed_epochs.pop_front();
    }
    for (auto it = staged_state_history.begin();
         it != staged_state_history.end() && it->first < oldest;) {
      it = staged_state_history.erase(it);
    }
    for (auto it = staged_prefix_covariances.begin();
         it != staged_prefix_covariances.end() && it->first < oldest;) {
      it = staged_prefix_covariances.erase(it);
    }

    boundary = "receipt_creation";
    inject(CommitFaultPoint::BeforeReceiptCreation, boundary.c_str());
    receipt.transaction_id = tx.id;
    receipt.action_id = plan.action_id;
    receipt.graph_version = staged_graph_version;
    receipt.committed_epoch = staged_epoch;
    receipt.state_timestamp = staged_timestamp;
    receipt.added_factor_slots = update.new_factor_slots;
    receipt.removed_factor_slots = update.removed_factor_slots;
    receipt.marginalized_keys = update.marginalized_keys;
    receipt.boundary_factor_slots = update.boundary_factor_slots;
    receipt.group_to_slots.clear();
    for (const auto* group : selected_groups) {
      receipt.group_to_slots[group->id.value()] =
          staged_ledger.activeSlots(group->id);
    }
    receipt.reused_factor_slots.assign(reused_slots.begin(), reused_slots.end());
    receipt.recovery_epoch_begin = plan.recovery_epoch_begin;
    receipt.recovery_epoch_end = plan.recovery_epoch_end;
    for (const auto group : plan.groups_to_remove) {
      const auto entries = staged_ledger.groupEntries(group);
      if (!entries.empty() && entries.front().epoch_end < tx.proposed_epoch) {
        receipt.historical_groups_removed.push_back(group);
      }
    }
    for (const auto* group : selected_groups) {
      if (group->recovery_epoch != 0 &&
          group->recovery_epoch == tx.proposed_epoch) {
        receipt.historical_groups_added.push_back(group->id);
      }
    }
    receipt.backend_updates = 1;
    staged_certification.committed_mean_world_m =
        staged_current_state.position_world_m;
    staged_certification.committed_covariance = staged_current_covariance;
    if (protection_evidence_valid) {
      staged_certification.reference_transfer_m =
          (staged_certification.committed_mean_world_m -
           staged_certification.protected_reference.mean_world_m).cwiseAbs();
      staged_certification.transferred_pl_m =
          staged_certification.protected_reference.pl_at_reference_m +
          staged_certification.reference_transfer_m;
      staged_certification.reference_bound =
          staged_certification.transferred_pl_m.allFinite();
    }
    staged_certification.integrity_available =
        !plan.best_effort_integrity_unavailable &&
        staged_certification.reference_bound;
    receipt.integrity_available = staged_certification.integrity_available;

    // Atomic metadata publication.  All operations that can query, allocate,
    // bind, prune, or create the receipt have already succeeded.
    factor_ledger_ = std::move(staged_ledger);
    committed_epochs_.swap(staged_committed_epochs);
    committed_uwb_batches_.swap(staged_committed_batches);
    state_history_.swap(staged_state_history);
    prefix_covariances_.swap(staged_prefix_covariances);
    imu_queue_.swap(staged_imu_queue);
    imu_boundary_ = staged_imu_boundary;
    epoch_ = staged_epoch;
    state_timestamp_ = staged_timestamp;
    current_state_ = staged_current_state;
    current_uwb_committed_ = staged_current_uwb_committed;
    marginalization_count_ = staged_marginalization_count;
    ordering_version_ = staged_ordering_version;
    graph_version_ = staged_graph_version;
    linpoint_version_ = staged_linpoint_version;
    last_state_query_ms_ = staged_state_query_ms;
    pending_epoch_ = false;
    pending_batch_id_.reset();
    active_transaction_id_.reset();
    adapter_transaction_.reset();
    last_uwb_update_ms_ = elapsedMs(start);
    disarmP005Fault(this);
    if (certification) *certification = std::move(staged_certification);
    return receipt;
  } catch (...) {
    std::string reason = "unknown post-mutation commit failure";
    try {
      throw;
    } catch (const std::exception& error) {
      reason = error.what();
    } catch (...) {
    }
    backend_poisoned_ = true;
    active_transaction_id_.reset();
    adapter_transaction_.reset();
    pending_epoch_ = false;
    pending_batch_id_.reset();
    const CommitFaultInjectionAuditV1 fault_audit = p005FaultAudit(this);
    disarmP005Fault(this);
    CommitFailureReceipt terminal;
    terminal.transaction_id = tx.id;
    terminal.backend_updates = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(backend_update_count_ - updates_before,
                                std::numeric_limits<std::uint32_t>::max()));
    terminal.backend_mutated = true;
    terminal.backend_poisoned = true;
    terminal.committed_unprotected = true;
    terminal.injected_fault_point = fault_audit.hit_point;
    terminal.injected_fault_nonce = fault_audit.hit_nonce;
    terminal.injected_fault_hit = fault_audit.hit;
    terminal.boundary = boundary;
    terminal.reason = "commit terminal at " + boundary + ": " + reason;
    throw CommitTerminalError(std::move(terminal));
  }
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
    throw std::logic_error(
        "commit requires matching pending adapter transaction");
  }
  EpochCommitPlan plan = EpochCommitPlan::nominalPlan(*adapter_transaction_);
  EpochTransaction tx = std::move(*adapter_transaction_);
  (void)commitEpoch(std::move(tx), plan);
}

void IncrementalUwbImuEstimator::rejectUwbBatch(const UwbBatch& batch,
                                                const std::string& reason) {
  if (!adapter_transaction_ || batch.timestamp != adapter_transaction_->end ||
      !pending_batch_id_ || *pending_batch_id_ != batch.id) {
    throw std::logic_error(
        "reject requires matching pending adapter transaction");
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
  if (!initialized_)
    throw std::logic_error("UWB/IMU estimator is not initialized");
  if (adapter_transaction_)
    return adapter_transaction_->nominal_predicted_state;
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
  if (!initialized_)
    throw std::logic_error("UWB/IMU estimator is not initialized");
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
                               ? fixed_lag_backend_->timestamps().size()
                               : activeValueCount();
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

CommitBoundaryAuditV1
IncrementalUwbImuEstimator::commitBoundaryAudit() const {
  CommitBoundaryAuditV1 out;
  out.published_state = current_state_;
  const auto covariance = prefix_covariances_.find(epoch_);
  if (covariance != prefix_covariances_.end()) {
    out.published_covariance = covariance->second;
  }
  out.ledger_version = factor_ledger_.version();
  out.ledger_entries = factor_ledger_.entries();
  for (const auto& entry : out.ledger_entries) {
    if (entry.backend_slot && entry.lifecycle == FactorLifecycle::Active) {
      out.active_ledger_slots.push_back(*entry.backend_slot);
    }
  }
  std::sort(out.active_ledger_slots.begin(), out.active_ledger_slots.end());
  out.committed_epoch_catalog.reserve(committed_epochs_.size());
  for (const auto& record : committed_epochs_) {
    out.committed_epoch_catalog.push_back({
        record.transaction.id,
        record.transaction.previous_epoch,
        record.transaction.proposed_epoch,
        record.transaction.begin,
        record.transaction.end,
        record.transaction.uwb_batch.id,
        record.selected_groups});
  }
  out.committed_batch_catalog.assign(committed_uwb_batches_.begin(),
                                     committed_uwb_batches_.end());
  out.state_history = state_history_;
  out.covariance_history = prefix_covariances_;
  out.imu_queue.assign(imu_queue_.begin(), imu_queue_.end());
  out.imu_boundary = imu_boundary_;
  out.last_received_imu_timestamp = last_received_imu_timestamp_;
  out.epoch = epoch_;
  out.state_timestamp = state_timestamp_;
  out.version = {graph_version_, ordering_version_, 1, linpoint_version_};
  out.active_transaction_id =
      active_transaction_id_ ? active_transaction_id_->value() : 0;
  out.pending_batch_id = pending_batch_id_;
  out.pending_epoch = pending_epoch_;
  out.backend_poisoned = backendPoisoned();
  out.backend_update_count = backend_update_count_;
  return out;
}

std::vector<CommittedEpochCatalogAuditV1>
IncrementalUwbImuEstimator::debugCommittedEpochCatalog() const {
  std::vector<CommittedEpochCatalogAuditV1> result;
  result.reserve(committed_epochs_.size());
  for (const auto& record : committed_epochs_) {
    result.push_back({record.transaction.id,
                      record.transaction.previous_epoch,
                      record.transaction.proposed_epoch,
                      record.transaction.begin,
                      record.transaction.end,
                      record.transaction.uwb_batch.id,
                      record.selected_groups});
  }
  return result;
}

std::optional<Eigen::Matrix<double, 15, 15>>
IncrementalUwbImuEstimator::methodBCandidatePrior(
    const Eigen::Matrix<double, 15, 15>& all_in_covariance,
    const Eigen::MatrixXd& current_uwb_jacobian,
    const Eigen::MatrixXd& current_uwb_covariance) const {
  if (!all_in_covariance.allFinite() || current_uwb_jacobian.cols() != 15 ||
      current_uwb_covariance.rows() != current_uwb_jacobian.rows() ||
      current_uwb_covariance.cols() != current_uwb_jacobian.rows())
    return std::nullopt;
  Eigen::LDLT<Eigen::Matrix<double, 15, 15>> covariance_ldlt(all_in_covariance);
  Eigen::LDLT<Eigen::MatrixXd> measurement_ldlt(current_uwb_covariance);
  if (covariance_ldlt.info() != Eigen::Success ||
      measurement_ldlt.info() != Eigen::Success)
    return std::nullopt;
  const Eigen::Matrix<double, 15, 15> prior_information =
      covariance_ldlt.solve(Eigen::Matrix<double, 15, 15>::Identity()) -
      current_uwb_jacobian.transpose() *
          measurement_ldlt.solve(current_uwb_jacobian);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> eigen(
      prior_information);
  if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() <= 0.0) {
    return std::nullopt;
  }
  const double condition =
      eigen.eigenvalues().maxCoeff() / eigen.eigenvalues().minCoeff();
  if (!std::isfinite(condition) ||
      condition > config_.incremental.method_b_max_condition)
    return std::nullopt;
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
      measurement_ldlt.info() != Eigen::Success)
    return std::nullopt;
  const Eigen::Matrix<double, 15, 15> all_in_information =
      all_in_ldlt.solve(Eigen::Matrix<double, 15, 15>::Identity());
  const Eigen::Matrix<double, 15, 1> prior_information_vector =
      all_in_information * all_in_mean -
      current_uwb_jacobian.transpose() *
          measurement_ldlt.solve(current_uwb_linear_measurement);
  MethodBCandidatePrior result;
  result.covariance = *covariance;
  result.mean = result.covariance * prior_information_vector;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> eigen(
      result.covariance);
  if (!result.mean.allFinite() || eigen.info() != Eigen::Success ||
      eigen.eigenvalues().minCoeff() <= 0.0)
    return std::nullopt;
  result.condition_number =
      eigen.eigenvalues().maxCoeff() / eigen.eigenvalues().minCoeff();
  return result;
}

std::vector<RawRowOwnershipAuditV1>
IncrementalUwbImuEstimator::auditRawRowOwnershipV1(
    const EpochTransaction& transaction) const {
  if (!transaction.frozen_values) {
    throw std::runtime_error("raw row ownership audit requires frozen values");
  }
  const auto frozen = readP007Certificate(this, transaction);
  if (!frozen) {
    throw std::runtime_error(
        "raw row ownership audit certificate is absent or belongs to a "
        "different estimator lifetime");
  }
  return *frozen;
}

static std::vector<RawRowOwnershipAuditV1> buildRawRowOwnershipCertificateV1(
    const EpochTransaction& transaction, const FactorLedger& frozen_ledger) {
  std::vector<RawRowOwnershipAuditV1> result;
  std::set<std::uint64_t> represented_groups;
  const auto append_group = [&](const PendingFactorGroup& group) {
    if (!group.nominal ||
        (group.kind != FactorKind::CombinedImu &&
         group.kind != FactorKind::UwbBatch)) {
      return;
    }
    gtsam::Ordering ordering;
    for (const auto key : group.keys) {
      if (std::find(ordering.begin(), ordering.end(), key) == ordering.end()) {
        ordering.push_back(key);
      }
    }
    const auto dense = group.factors.linearize(
        *transaction.frozen_values)->jacobian(ordering);
    const std::size_t rows = static_cast<std::size_t>(dense.first.rows());
    if (group.kind == FactorKind::UwbBatch &&
        group.source_measurements.size() > rows) {
      throw std::runtime_error(
          "UWB row ownership audit size mismatch for group " +
          std::to_string(group.id.value()) + ": sources=" +
          std::to_string(group.source_measurements.size()) + ", rows=" +
          std::to_string(rows));
    }
    represented_groups.insert(group.id.value());
    for (std::size_t row = 0; row < rows; ++row) {
      RawRowOwnershipAuditV1 entry;
      entry.owner_group = group.id;
      entry.row_in_group = row;
      if (group.kind == FactorKind::UwbBatch &&
          row < group.source_measurements.size()) {
        entry.row_id = "uwb-measurement:" +
            std::to_string(group.source_measurements[row].value());
        entry.covariance_placement = "uwb-covariance-row-column:" +
            std::to_string(row);
      } else {
        entry.row_id = (group.kind == FactorKind::CombinedImu ? "imu:" :
                                                              "group-aux:") +
            std::to_string(group.id.value()) + ":" + std::to_string(row);
        entry.covariance_placement = "factor-native-whitened-row:" +
            std::to_string(row);
      }
      result.push_back(std::move(entry));
    }
  };
  for (const auto& record : transaction.recoverable_history) {
    for (const auto& group : record.groups) append_group(group);
  }
  append_group(transaction.imu_group);
  for (const auto& group : transaction.uwb_groups) append_group(group);

  for (const auto& slot : transaction.frozen_slots) {
    if (!slot.group_id || represented_groups.count(slot.group_id->value()) != 0) {
      continue;
    }
    const auto linear = slot.factor->linearize(*transaction.frozen_values);
    std::size_t rows = 0;
    if (const auto jacobian =
            boost::dynamic_pointer_cast<gtsam::JacobianFactor>(linear)) {
      rows = static_cast<std::size_t>(jacobian->rows());
    } else if (const auto hessian =
                   boost::dynamic_pointer_cast<gtsam::HessianFactor>(linear)) {
      rows = static_cast<std::size_t>(hessian->info().rows() - 1);
    } else {
      throw std::runtime_error("unknown frozen factor in row ownership audit");
    }
    const auto ledger_entries = frozen_ledger.groupEntries(*slot.group_id);
    const auto ledger_uwb = std::find_if(
        ledger_entries.begin(), ledger_entries.end(),
        [&](const FactorLedgerEntry& entry) {
          return entry.kind == FactorKind::UwbBatch &&
              entry.source_measurements.size() == rows;
        });
    const bool frozen_uwb = ledger_uwb != ledger_entries.end();
    for (std::size_t row = 0; row < rows; ++row) {
      if (frozen_uwb) {
        result.push_back({
            "uwb-measurement:" +
                std::to_string(ledger_uwb->source_measurements[row].value()),
            *slot.group_id, row,
            "uwb-covariance-row-column:" + std::to_string(row)});
      } else {
        result.push_back({"prior:marginalized-through-epoch-1:" +
                              std::to_string(slot.group_id->value()) + ":" +
                              std::to_string(row),
                          *slot.group_id, row,
                          "information-prior:" + std::to_string(slot.slot) +
                              ":" + std::to_string(row)});
      }
    }
  }
  std::sort(result.begin(), result.end(), [](const auto& left,
                                             const auto& right) {
    return left.row_id < right.row_id;
  });
  return result;
}

}  // namespace uwb_imu_pl
