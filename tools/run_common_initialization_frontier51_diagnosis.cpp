// Gate06R-D diagnostic-only binary. It reconstructs the frozen common
// initialization locally, never publishes a trajectory, and never reads GT.
#define Open Gate06rdApplicationOpen
#define RunIePaperApplication gate06rd_unused_paper_application
#include "paper/run_ie_app.cpp"
#undef RunIePaperApplication
#undef Open

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <numeric>
#include <typeinfo>

#include "uifgo/paper_pose_prior_factor.h"
#include "uifgo/paper_robust_noise.h"

namespace {

constexpr double kHuberScale = 1.345;
constexpr double kStationarityTolerance = 1e-5;
constexpr size_t kFirstReportedFrontier = 45;
constexpr size_t kFailedFrontier = 51;

std::ofstream DOpen(const fs::path& path) {
  auto out = Gate06rdApplicationOpen(path);
  out << std::setprecision(17);
  return out;
}

double Quantile(std::vector<double> values, double probability) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double index = probability * static_cast<double>(values.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(index));
  const size_t hi = static_cast<size_t>(std::ceil(index));
  const double alpha = index - static_cast<double>(lo);
  return values[lo] * (1.0 - alpha) + values[hi] * alpha;
}

double Mean(const std::vector<double>& values) {
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  long double sum = 0.0L;
  for (double value : values) sum += value;
  return static_cast<double>(sum / values.size());
}

double ValuesDeltaNorm(const gtsam::Values& before,
                       const gtsam::Values& after) {
  const auto delta = before.localCoordinates(after);
  long double squared = 0.0L;
  for (const auto& item : delta) squared += item.second.squaredNorm();
  return std::sqrt(static_cast<double>(squared));
}

double StateDeltaNorm(const gtsam::Values& before, const gtsam::Values& after,
                      size_t state) {
  gtsam::Values a, b;
  for (gtsam::Key key : {X(state), V(state), B(state)}) {
    a.insert(key, before.at(key));
    b.insert(key, after.at(key));
  }
  return ValuesDeltaNorm(a, b);
}

bool StateFinite(const gtsam::Values& values, size_t state) {
  try {
    const auto pose = values.at<gtsam::Pose3>(X(state));
    const auto velocity = values.at<gtsam::Vector3>(V(state));
    const auto bias = values.at<gtsam::imuBias::ConstantBias>(B(state));
    return pose.matrix().allFinite() && velocity.allFinite() &&
           bias.accelerometer().allFinite() && bias.gyroscope().allFinite();
  } catch (...) {
    return false;
  }
}

class FixedBoundaryCombinedImuFactor final
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3,
                                      gtsam::imuBias::ConstantBias> {
 public:
  using Base = gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3,
                                        gtsam::imuBias::ConstantBias>;

  FixedBoundaryCombinedImuFactor(
      const gtsam::CombinedImuFactor::shared_ptr& physical,
      gtsam::Key pose_j_key, gtsam::Key velocity_j_key,
      gtsam::Key bias_j_key, const gtsam::Pose3& pose_i,
      const gtsam::Vector3& velocity_i,
      const gtsam::imuBias::ConstantBias& bias_i)
      : Base(physical->noiseModel(), pose_j_key, velocity_j_key, bias_j_key),
        physical_(physical), pose_i_(pose_i), velocity_i_(velocity_i),
        bias_i_(bias_i) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(
        new FixedBoundaryCombinedImuFactor(*this));
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose_j, const gtsam::Vector3& velocity_j,
      const gtsam::imuBias::ConstantBias& bias_j,
      boost::optional<gtsam::Matrix&> H_pose = boost::none,
      boost::optional<gtsam::Matrix&> H_velocity = boost::none,
      boost::optional<gtsam::Matrix&> H_bias = boost::none) const override {
    return physical_->evaluateError(
        pose_i_, velocity_i_, pose_j, velocity_j, bias_i_, bias_j,
        boost::none, boost::none, H_pose, H_velocity, boost::none, H_bias);
  }

 private:
  gtsam::CombinedImuFactor::shared_ptr physical_;
  gtsam::Pose3 pose_i_;
  gtsam::Vector3 velocity_i_;
  gtsam::imuBias::ConstantBias bias_i_;
};

gtsam::NonlinearFactor::shared_ptr HuberClone(
    const gtsam::NonlinearFactor::shared_ptr& factor) {
  const auto noise_factor =
      boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
  if (!noise_factor) throw std::runtime_error("UWB_FACTOR_HAS_NO_NOISE_MODEL");
  const auto huber =
      gtsam::noiseModel::mEstimator::Huber::Create(kHuberScale);
  return noise_factor->cloneWithNewNoiseModel(
      boost::make_shared<uifgo::PaperRobustNoise>(
          huber, noise_factor->noiseModel()));
}

struct Prepared {
  uifgo::Config cfg;
  LoadedInput loaded;
  std::vector<uifgo::ImuSample> imu;
  std::vector<uifgo::UwbFrame> raw;
  uifgo::PaperInputPlan plan;
  std::vector<uifgo::UwbFrame> keyframes;
  uifgo::InitResult initialization;
  gtsam::NonlinearFactorGraph physical_graph;
  gtsam::Values open_loop;
  std::vector<size_t> uwb_indices;
  std::vector<uifgo::FactorMeta> factor_metadata;
};

Prepared PrepareRaw(const fs::path& config_path) {
  Prepared output;
  output.cfg = uifgo::ConfigLoader::Load(config_path.string());
  ValidateSupportedConfig(output.cfg);
  output.loaded = LoadData(config_path.parent_path().string(), &output.cfg,
                           &output.imu, &output.raw, config_path.string());
  if (output.imu.empty() || output.raw.empty() || output.cfg.anchors.empty())
    throw std::runtime_error("GATE06RD_LOADER_RETURNED_INCOMPLETE_INPUT");
  output.plan = uifgo::BuildPaperInputPlan(
      output.raw, output.cfg, output.loaded.recording_id);
  const auto mask = uifgo::AllPlannedObservationMask(output.plan);
  output.keyframes = uifgo::MaterializePaperKeyframes(output.plan, mask);
  output.initialization =
      uifgo::Initializer(output.cfg).Run(output.imu, output.keyframes);
  if (!output.initialization.ok) throw std::runtime_error("INITIALIZER_FAILED");
  uifgo::GraphBuilder builder(output.cfg,
                              output.cfg.paper_imu_covariance_model);
  builder.Build(output.keyframes, output.imu, output.initialization,
                &output.physical_graph, &output.open_loop,
                &output.uwb_indices);
  if (uifgo::ReplacePosePriorsForPaperPath(&output.physical_graph) != 1)
    throw std::runtime_error("PAPER_POSE_PRIOR_REPLACEMENT_FAILED");
  output.factor_metadata = builder.factor_meta();
  uifgo::ValidatePaperFixedBeta(output.plan, output.cfg);
  if (output.plan.keyframes.size() <= kFailedFrontier)
    throw std::runtime_error("WALK1_HAS_TOO_FEW_STATES");
  return output;
}

struct UwbInfo {
  size_t physical_factor_index = 0;
  std::uint64_t obs_id = 0;
  int anchor_id = 0;
  double timestamp_s = 0.0;
  double measured_m = 0.0;
  double sigma_m = 0.0;
  gtsam::NoiseModelFactor::shared_ptr physical;
};

struct Maps {
  std::map<size_t, gtsam::CombinedImuFactor::shared_ptr> imu;
  std::map<size_t, std::vector<UwbInfo>> uwb;
};

Maps BuildMaps(const Prepared& prepared) {
  Maps maps;
  for (const auto& factor : prepared.physical_graph) {
    const auto imu =
        boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
    if (!imu) continue;
    const size_t state = gtsam::Symbol(imu->keys().at(2)).index();
    if (!maps.imu.emplace(state, imu).second)
      throw std::runtime_error("DUPLICATE_IMU_FRONTIER");
  }
  std::unordered_map<std::uint64_t, const uifgo::ObservationRecord*> records;
  for (const auto& row : prepared.plan.observations)
    records.emplace(row.obs_id, &row);
  for (const auto& meta : prepared.factor_metadata) {
    if (meta.factor_type != "uwb_range") continue;
    if (meta.factor_index >= prepared.physical_graph.size() ||
        meta.keys.size() != 1 || !records.count(meta.obs_id))
      throw std::runtime_error("UWB_METADATA_INVALID");
    const auto state = gtsam::Symbol(meta.keys.front()).index();
    const auto factor = prepared.physical_graph.at(meta.factor_index);
    const auto expression =
        boost::dynamic_pointer_cast<gtsam::ExpressionFactor<double>>(factor);
    const auto noise =
        boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
    if (!expression || !noise || noise->noiseModel()->sigmas().size() != 1)
      throw std::runtime_error("UWB_FACTOR_INVALID");
    const auto& record = *records.at(meta.obs_id);
    maps.uwb[state].push_back(
        UwbInfo{meta.factor_index, meta.obs_id, record.anchor_id,
                record.sensor_time, expression->measured(),
                noise->noiseModel()->sigmas()[0], noise});
  }
  return maps;
}

struct LocalProblem {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  std::vector<UwbInfo> uwb;
};

LocalProblem BuildOneState(const Maps& maps, const gtsam::Values& committed,
                           size_t frontier) {
  if (!maps.imu.count(frontier)) throw std::runtime_error("MISSING_IMU_FACTOR");
  LocalProblem local;
  const auto& imu = maps.imu.at(frontier);
  const auto pose_i = committed.at<gtsam::Pose3>(X(frontier - 1));
  const auto velocity_i = committed.at<gtsam::Vector3>(V(frontier - 1));
  const auto bias_i =
      committed.at<gtsam::imuBias::ConstantBias>(B(frontier - 1));
  local.graph.add(boost::make_shared<FixedBoundaryCombinedImuFactor>(
      imu, X(frontier), V(frontier), B(frontier), pose_i, velocity_i, bias_i));
  const auto prediction = imu->preintegratedMeasurements().predict(
      gtsam::NavState(pose_i, velocity_i), bias_i);
  local.initial.insert(X(frontier), prediction.pose());
  local.initial.insert(V(frontier), prediction.velocity());
  local.initial.insert(B(frontier), bias_i);
  const auto found = maps.uwb.find(frontier);
  if (found != maps.uwb.end()) {
    local.uwb = found->second;
    for (const auto& row : found->second)
      local.graph.add(HuberClone(row.physical));
  }
  return local;
}

uifgo::CheckedLmOptions SolverOptions(const uifgo::Config& config) {
  uifgo::CheckedLmOptions options;
  options.max_iterations = config.lm_max_iter;
  options.relative_tolerance = config.lm_rel_tol;
  options.absolute_tolerance = config.lm_abs_tol;
  options.policy = uifgo::ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  options.navigation_stationarity_tolerance_objective =
      kStationarityTolerance;
  return options;
}

struct Block {
  std::string label;
  std::string reason;
  gtsam::Values start;
  gtsam::Values terminal;
  uifgo::NavigationStationarityAudit stationarity;
  double initial_objective = std::numeric_limits<double>::quiet_NaN();
  double terminal_objective = std::numeric_limits<double>::quiet_NaN();
  double update_norm = std::numeric_limits<double>::quiet_NaN();
  size_t calls = 0;
  size_t accepted_updates = 0;
  size_t rejected_trials = 0;
  size_t no_update_returns = 0;
  double initial_lambda = std::numeric_limits<double>::quiet_NaN();
  double final_lambda = std::numeric_limits<double>::quiet_NaN();
  bool qualified = false;
  bool diagnostic_trace = false;
  std::vector<uifgo::CheckedLmCallDiagnostics> call_trace;
};

Block RunProductionBlock(const std::string& label,
                         const gtsam::NonlinearFactorGraph& graph,
                         const gtsam::Values& start,
                         const uifgo::CheckedLmOptions& options) {
  Block block;
  block.label = label;
  block.start = start;
  block.initial_objective = graph.error(start);
  const auto result =
      uifgo::RunCheckedConditionalLmRetainingTerminalForInitialization(
          graph, start, options);
  block.reason = result.reason;
  block.terminal = result.values;
  block.calls = result.convergence.iterate_call_count;
  block.accepted_updates = result.convergence.accepted_update_count;
  block.rejected_trials = result.convergence.rejected_lambda_trial_count;
  block.no_update_returns = result.convergence.no_update_return_count;
  block.final_lambda = result.lambda;
  if (!block.terminal.empty()) {
    block.terminal_objective = graph.error(block.terminal);
    block.update_norm = ValuesDeltaNorm(start, block.terminal);
    block.stationarity = uifgo::AuditNavigationStationarity(
        graph, block.terminal, uifgo::NavigationScales(),
        kStationarityTolerance, 8.0);
    block.qualified = block.stationarity.valid &&
                      block.stationarity.stationary &&
                      uifgo::GraphAndValuesKeysMatch(graph, block.terminal) &&
                      std::isfinite(block.terminal_objective);
  }
  return block;
}

Block RunDiagnosticBlock(const std::string& label,
                         const gtsam::NonlinearFactorGraph& graph,
                         const gtsam::Values& start,
                         const uifgo::CheckedLmOptions& options) {
  uifgo::CheckedLmDiagnosticRequest request;
  request.emit_linked_gtsam_trylambda = true;
  request.finite_difference_steps = {1e-4, 1e-5, 1e-6};
  const auto result =
      uifgo::RunCheckedConditionalLm(graph, start, options, &request);
  Block block;
  block.label = label;
  block.reason = result.reason;
  block.start = start;
  block.initial_objective = graph.error(start);
  block.terminal = !result.values.empty() ? result.values
                                          : result.diagnostic.values_at_final;
  block.calls = result.convergence.iterate_call_count;
  block.accepted_updates = result.convergence.accepted_update_count;
  block.rejected_trials = result.convergence.rejected_lambda_trial_count;
  block.no_update_returns = result.convergence.no_update_return_count;
  block.final_lambda = result.lambda;
  block.call_trace = result.diagnostic.calls;
  block.diagnostic_trace = true;
  if (!block.call_trace.empty())
    block.initial_lambda = block.call_trace.front().lambda_before;
  if (!block.terminal.empty()) {
    block.terminal_objective = graph.error(block.terminal);
    block.update_norm = ValuesDeltaNorm(start, block.terminal);
    block.stationarity = uifgo::AuditNavigationStationarity(
        graph, block.terminal, uifgo::NavigationScales(),
        kStationarityTolerance, 8.0);
    block.qualified = block.stationarity.valid &&
                      block.stationarity.stationary &&
                      uifgo::GraphAndValuesKeysMatch(graph, block.terminal) &&
                      std::isfinite(block.terminal_objective);
  }
  return block;
}

struct UwbEval {
  UwbInfo info;
  double predicted_m = 0.0;
  double residual_m = 0.0;
  double q = 0.0;
  double huber_weight = 0.0;
};

std::vector<UwbEval> EvaluateUwb(const LocalProblem& local,
                                 const gtsam::Values& values) {
  std::vector<UwbEval> output;
  for (size_t i = 0; i < local.uwb.size(); ++i) {
    const auto residual = local.uwb[i].physical->unwhitenedError(values);
    if (residual.size() != 1) throw std::runtime_error("UWB_NOT_SCALAR");
    const auto robust = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(
        local.graph.at(i + 1));
    UwbEval row;
    row.info = local.uwb[i];
    row.residual_m = residual[0];
    row.predicted_m = row.info.measured_m + row.residual_m;
    row.q = row.residual_m / row.info.sigma_m;
    row.huber_weight = robust->weight(values);
    output.push_back(row);
  }
  return output;
}

std::string Anchors(const std::vector<UwbEval>& rows) {
  std::set<int> ids;
  for (const auto& row : rows) ids.insert(row.info.anchor_id);
  std::ostringstream out;
  bool first = true;
  for (int id : ids) {
    if (!first) out << ';';
    first = false;
    out << id;
  }
  return out.str();
}

struct Dist {
  double mean = std::numeric_limits<double>::quiet_NaN();
  double median = std::numeric_limits<double>::quiet_NaN();
  double minimum = std::numeric_limits<double>::quiet_NaN();
  double maximum = std::numeric_limits<double>::quiet_NaN();
};

Dist Distribution(const std::vector<double>& values) {
  Dist result;
  if (values.empty()) return result;
  result.mean = Mean(values);
  result.median = Quantile(values, 0.5);
  result.minimum = *std::min_element(values.begin(), values.end());
  result.maximum = *std::max_element(values.begin(), values.end());
  return result;
}

void WriteBlockRows(std::ostream& out, const Block& block) {
  for (const auto& call : block.call_trace) {
    out << block.label << ',' << call.call_index << ',' << call.lambda_before
        << ',' << call.lambda_after << ',' << call.error_before << ','
        << call.error_after << ',' << call.accepted_state_update << ','
        << call.rejected_lambda_trials_before_acceptance << ','
        << call.accepted_values_delta_norm << ','
        << call.stationarity_after.max_scaled_gradient_objective << ','
        << call.observed_return_class << '\n';
  }
}

void WriteUwbStage(std::ostream& out, const std::string& stage,
                   const std::vector<UwbEval>& rows) {
  for (const auto& row : rows) {
    out << stage << ',' << row.info.physical_factor_index << ','
        << row.info.obs_id << ',' << row.info.anchor_id << ','
        << row.info.timestamp_s << ',' << row.info.measured_m << ','
        << row.predicted_m << ',' << row.residual_m << ',' << row.info.sigma_m
        << ',' << row.q << ',' << row.huber_weight << '\n';
  }
}

struct Conditioning {
  size_t rows = 0;
  size_t cols = 0;
  size_t jacobian_rank = 0;
  size_t normal_rank = 0;
  double rank_threshold = 0.0;
  double singular_min = 0.0;
  double singular_max = 0.0;
  double condition = 0.0;
  double normal_min = 0.0;
  double normal_max = 0.0;
  gtsam::Vector singular_values;
  gtsam::Vector normal_eigenvalues;
  gtsam::Vector weak_direction;
  gtsam::Vector gradient_imu;
  gtsam::Vector gradient_uwb;
  gtsam::Vector gradient_total;
};

Conditioning AuditConditioning(const gtsam::NonlinearFactorGraph& graph,
                               const gtsam::Values& values, size_t state) {
  Conditioning result;
  gtsam::Ordering ordering;
  ordering.push_back(X(state));
  ordering.push_back(V(state));
  ordering.push_back(B(state));
  const auto linear = graph.linearize(values);
  const auto jb = linear->jacobian(ordering);
  const auto& J = jb.first;
  const auto& rhs = jb.second;
  result.rows = J.rows();
  result.cols = J.cols();
  Eigen::JacobiSVD<gtsam::Matrix> svd(J, Eigen::ComputeFullV);
  result.singular_values = svd.singularValues();
  result.singular_max = result.singular_values.size()
                            ? result.singular_values[0] : 0.0;
  result.rank_threshold = std::max(J.rows(), J.cols()) *
      std::numeric_limits<double>::epsilon() * result.singular_max;
  for (Eigen::Index i = 0; i < result.singular_values.size(); ++i)
    result.jacobian_rank += result.singular_values[i] > result.rank_threshold;
  result.singular_min = result.jacobian_rank
      ? result.singular_values[result.jacobian_rank - 1] : 0.0;
  result.condition = result.singular_min > 0.0
                         ? result.singular_max / result.singular_min
                         : std::numeric_limits<double>::infinity();
  result.weak_direction =
      svd.matrixV().col(static_cast<Eigen::Index>(result.cols - 1));
  const gtsam::Matrix normal = J.transpose() * J;
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(normal);
  result.normal_eigenvalues = eigen.eigenvalues();
  const double normal_threshold = result.rank_threshold * result.rank_threshold;
  for (Eigen::Index i = 0; i < result.normal_eigenvalues.size(); ++i)
    result.normal_rank += result.normal_eigenvalues[i] > normal_threshold;
  result.normal_min = result.normal_rank
      ? result.normal_eigenvalues[result.normal_eigenvalues.size() -
                                  result.normal_rank] : 0.0;
  result.normal_max = result.normal_eigenvalues.size()
      ? result.normal_eigenvalues[result.normal_eigenvalues.size() - 1] : 0.0;
  const size_t imu_rows = graph.at(0)->dim();
  result.gradient_imu = -J.topRows(imu_rows).transpose() * rhs.head(imu_rows);
  result.gradient_uwb = -J.bottomRows(J.rows() - imu_rows).transpose() *
                        rhs.tail(J.rows() - imu_rows);
  result.gradient_total = result.gradient_imu + result.gradient_uwb;
  return result;
}

std::vector<std::string> CoordinateNames() {
  return {"x.rot_x", "x.rot_y", "x.rot_z", "x.trans_x", "x.trans_y",
          "x.trans_z", "v.x", "v.y", "v.z", "b.acc_x", "b.acc_y",
          "b.acc_z", "b.gyro_x", "b.gyro_y", "b.gyro_z"};
}

LocalProblem BuildSlidingWindow(const Maps& maps,
                                const gtsam::Values& committed,
                                size_t boundary, size_t frontier) {
  LocalProblem local;
  for (size_t state = boundary + 1; state <= frontier; ++state) {
    const auto& imu = maps.imu.at(state);
    if (state == boundary + 1) {
      local.graph.add(boost::make_shared<FixedBoundaryCombinedImuFactor>(
          imu, X(state), V(state), B(state),
          committed.at<gtsam::Pose3>(X(boundary)),
          committed.at<gtsam::Vector3>(V(boundary)),
          committed.at<gtsam::imuBias::ConstantBias>(B(boundary))));
    } else {
      local.graph.add(imu);
    }
    if (state < frontier) {
      local.initial.insert(X(state), committed.at<gtsam::Pose3>(X(state)));
      local.initial.insert(V(state), committed.at<gtsam::Vector3>(V(state)));
      local.initial.insert(
          B(state), committed.at<gtsam::imuBias::ConstantBias>(B(state)));
    } else {
      const auto prediction = imu->preintegratedMeasurements().predict(
          gtsam::NavState(committed.at<gtsam::Pose3>(X(state - 1)),
                          committed.at<gtsam::Vector3>(V(state - 1))),
          committed.at<gtsam::imuBias::ConstantBias>(B(state - 1)));
      local.initial.insert(X(state), prediction.pose());
      local.initial.insert(V(state), prediction.velocity());
      local.initial.insert(
          B(state), committed.at<gtsam::imuBias::ConstantBias>(B(state - 1)));
    }
    const auto found = maps.uwb.find(state);
    if (found != maps.uwb.end()) {
      for (const auto& row : found->second) {
        local.uwb.push_back(row);
        local.graph.add(HuberClone(row.physical));
      }
    }
  }
  return local;
}

void WriteStateJson(std::ostream& out, const gtsam::Values& values,
                    size_t state) {
  const auto pose = values.at<gtsam::Pose3>(X(state));
  const auto p = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  const auto v = values.at<gtsam::Vector3>(V(state));
  const auto bias = values.at<gtsam::imuBias::ConstantBias>(B(state));
  const auto ba = bias.accelerometer();
  const auto bg = bias.gyroscope();
  out << "{\"rotation_quaternion_xyzw\":[" << q.x() << ',' << q.y()
      << ',' << q.z() << ',' << q.w() << "],\"position_m\":["
      << p.x() << ',' << p.y() << ',' << p.z()
      << "],\"position_norm_m\":" << p.norm() << ",\"velocity_mps\":["
      << v.x() << ',' << v.y() << ',' << v.z()
      << "],\"velocity_norm_mps\":" << v.norm()
      << ",\"accel_bias_mps2\":[" << ba.x() << ',' << ba.y() << ','
      << ba.z() << "],\"gyro_bias_radps\":[" << bg.x() << ',' << bg.y()
      << ',' << bg.z() << "],\"finite\":"
      << (StateFinite(values, state) ? "true" : "false") << '}';
}

void WriteBlockJson(std::ostream& out, const Block& block) {
  out << "{\"reason\":\"" << JsonEscape(block.reason)
      << "\",\"initial_objective\":" << block.initial_objective
      << ",\"terminal_objective\":" << block.terminal_objective
      << ",\"update_norm\":" << block.update_norm
      << ",\"iterate_calls\":" << block.calls
      << ",\"accepted_updates\":" << block.accepted_updates
      << ",\"rejected_lambda_trials\":" << block.rejected_trials
      << ",\"no_update_returns\":" << block.no_update_returns
      << ",\"initial_lambda\":";
  if (std::isfinite(block.initial_lambda)) out << block.initial_lambda;
  else out << "null";
  out
      << ",\"final_lambda\":" << block.final_lambda
      << ",\"stationarity_valid\":"
      << (block.stationarity.valid ? "true" : "false")
      << ",\"stationary\":" << (block.qualified ? "true" : "false")
      << ",\"max_scaled_gradient\":"
      << block.stationarity.max_scaled_gradient_objective
      << ",\"dominant_key\":\""
      << JsonEscape(block.stationarity.dominant_key_name)
      << "\",\"dominant_coordinate\":"
      << block.stationarity.dominant_coordinate
      << ",\"dominant_category\":\""
      << JsonEscape(block.stationarity.dominant_category) << "\"}";
}

int Run(const fs::path& config_path, const fs::path& output) {
  const auto prepared = PrepareRaw(config_path);
  const auto maps = BuildMaps(prepared);
  const auto options = SolverOptions(prepared.cfg);
  gtsam::Values committed;
  for (gtsam::Key key : prepared.open_loop.keys()) {
    const gtsam::Symbol symbol(key);
    if ((symbol.chr() == 'x' || symbol.chr() == 'v' || symbol.chr() == 'b') &&
        symbol.index() != 0) continue;
    committed.insert(key, prepared.open_loop.at(key));
  }

  auto accepted_csv = DOpen(output / "accepted_frontiers_45_50.csv");
  accepted_csv << "frontier,timestamp_s,px,py,pz,position_norm_m,vx,vy,vz,"
                  "velocity_norm_mps,bax,bay,baz,bgx,bgy,bgz,uwb_count,"
                  "anchor_ids,residual_signed_mean_m,residual_signed_median_m,"
                  "residual_abs_mean_m,residual_abs_median_m,residual_abs_max_m,"
                  "q_signed_mean,q_signed_median,q_abs_mean,q_abs_median,q_abs_max,"
                  "huber_weight_mean,huber_weight_min,huber_weight_median,"
                  "huber_weight_max,initial_objective,terminal_objective,"
                  "lm_reason,max_scaled_gradient,dominant_key,dominant_coordinate,"
                  "accepted_state_update_norm,lm_calls,accepted_updates,"
                  "rejected_lambda_trials,initial_lambda,final_lambda,trace_match\n";
  auto calls_csv = DOpen(output / "lm_call_trace.csv");
  calls_csv << "block,call,lambda_before,lambda_after,error_before,error_after,"
               "accepted_update,rejected_trials_before_acceptance,update_norm,"
               "max_scaled_gradient,return_class\n";

  LocalProblem failed_local;
  Block failed_first, failed_current_continuation;
  for (size_t frontier = 1; frontier <= kFailedFrontier; ++frontier) {
    const LocalProblem local = BuildOneState(maps, committed, frontier);
    const auto initially_stationary = uifgo::AuditNavigationStationarity(
        local.graph, local.initial, uifgo::NavigationScales(),
        kStationarityTolerance, 8.0);
    Block first;
    if (initially_stationary.valid && initially_stationary.stationary) {
      first.label = "frontier_" + std::to_string(frontier) + "_already_stationary";
      first.reason = "ALREADY_NAVIGATION_STATIONARY";
      first.start = local.initial;
      first.terminal = local.initial;
      first.initial_objective = local.graph.error(local.initial);
      first.terminal_objective = first.initial_objective;
      first.update_norm = 0.0;
      first.stationarity = initially_stationary;
      first.qualified = true;
    } else {
      first = RunProductionBlock(
          "frontier_" + std::to_string(frontier) + "_first", local.graph,
          local.initial, options);
    }
    Block second;
    bool accepted = first.qualified;
    gtsam::Values accepted_values = first.terminal;
    if (!accepted && !first.terminal.empty()) {
      second = RunProductionBlock(
          "frontier_" + std::to_string(frontier) + "_current_continuation",
          local.graph, first.terminal, options);
      accepted = second.qualified;
      accepted_values = second.terminal;
    }

    if (frontier >= kFirstReportedFrontier) {
      const Block trace_first = RunDiagnosticBlock(
          "frontier_" + std::to_string(frontier) + "_first_trace",
          local.graph, local.initial, options);
      WriteBlockRows(calls_csv, trace_first);
      double trace_difference = std::numeric_limits<double>::quiet_NaN();
      if (!first.terminal.empty() && !trace_first.terminal.empty())
        trace_difference = ValuesDeltaNorm(first.terminal, trace_first.terminal);
      if (!first.qualified && !first.terminal.empty()) {
        const Block trace_second = RunDiagnosticBlock(
            "frontier_" + std::to_string(frontier) + "_current_continuation_trace",
            local.graph, first.terminal, options);
        WriteBlockRows(calls_csv, trace_second);
      }
      if (frontier < kFailedFrontier) {
        if (!accepted) throw std::runtime_error("EARLY_FRONTIER_REPLAY_FAILED");
        const auto rows = EvaluateUwb(local, accepted_values);
        std::vector<double> residual, abs_residual, q, abs_q, weight;
        for (const auto& row : rows) {
          residual.push_back(row.residual_m);
          abs_residual.push_back(std::abs(row.residual_m));
          q.push_back(row.q);
          abs_q.push_back(std::abs(row.q));
          weight.push_back(row.huber_weight);
        }
        const auto rd = Distribution(residual);
        const auto ar = Distribution(abs_residual);
        const auto qd = Distribution(q);
        const auto aq = Distribution(abs_q);
        const auto wd = Distribution(weight);
        const auto pose = accepted_values.at<gtsam::Pose3>(X(frontier));
        const auto p = pose.translation();
        const auto v = accepted_values.at<gtsam::Vector3>(V(frontier));
        const auto bias = accepted_values.at<gtsam::imuBias::ConstantBias>(B(frontier));
        const auto ba = bias.accelerometer();
        const auto bg = bias.gyroscope();
        const Block& terminal_block = first.qualified ? first : second;
        accepted_csv << frontier << ','
          << prepared.plan.keyframes[frontier].sensor_time << ','
          << p.x() << ',' << p.y() << ',' << p.z() << ',' << p.norm() << ','
          << v.x() << ',' << v.y() << ',' << v.z() << ',' << v.norm() << ','
          << ba.x() << ',' << ba.y() << ',' << ba.z() << ','
          << bg.x() << ',' << bg.y() << ',' << bg.z() << ',' << rows.size()
          << ",\"" << Anchors(rows) << "\"," << rd.mean << ',' << rd.median
          << ',' << ar.mean << ',' << ar.median << ',' << ar.maximum << ','
          << qd.mean << ',' << qd.median << ',' << aq.mean << ',' << aq.median
          << ',' << aq.maximum << ',' << wd.mean << ',' << wd.minimum << ','
          << wd.median << ',' << wd.maximum << ',' << local.graph.error(local.initial)
          << ',' << local.graph.error(accepted_values) << ','
          << terminal_block.reason << ','
          << terminal_block.stationarity.max_scaled_gradient_objective << ','
          << terminal_block.stationarity.dominant_key_name << ','
          << terminal_block.stationarity.dominant_coordinate << ','
          << ValuesDeltaNorm(local.initial, accepted_values) << ','
          << terminal_block.calls << ',' << terminal_block.accepted_updates << ','
          << terminal_block.rejected_trials << ',' << trace_first.initial_lambda
          << ',' << trace_first.final_lambda << ','
          << (trace_difference <= 1e-12) << '\n';
      } else {
        failed_local = local;
        failed_first = first;
        failed_current_continuation = second;
      }
    }
    if (!accepted) {
      if (frontier != kFailedFrontier)
        throw std::runtime_error("UNEXPECTED_FAILURE_FRONTIER");
      break;
    }
    for (gtsam::Key key : {X(frontier), V(frontier), B(frontier)})
      committed.insert(key, accepted_values.at(key));
  }

  if (failed_current_continuation.terminal.empty())
    throw std::runtime_error("FAILED_TERMINAL_NOT_CAPTURED");
  const Block diagnostic_continuation = RunDiagnosticBlock(
      "frontier_51_additional_identical_continuation", failed_local.graph,
      failed_current_continuation.terminal, options);
  WriteBlockRows(calls_csv, diagnostic_continuation);

  size_t boundary = kFailedFrontier;
  const double cutoff = prepared.plan.keyframes[kFailedFrontier].sensor_time -
                        prepared.cfg.initialization_progression_horizon_s;
  for (size_t state = 0; state <= kFailedFrontier; ++state) {
    if (prepared.plan.keyframes[state].sensor_time >= cutoff) {
      boundary = state;
      break;
    }
  }
  if (boundary >= kFailedFrontier)
    throw std::runtime_error("SLIDING_WINDOW_HAS_NO_FREE_STATE");
  LocalProblem window =
      BuildSlidingWindow(maps, committed, boundary, kFailedFrontier);
  Block window_first = RunDiagnosticBlock(
      "sliding_window_first", window.graph, window.initial, options);
  WriteBlockRows(calls_csv, window_first);
  Block window_second;
  Block window_terminal = window_first;
  if (!window_first.qualified && !window_first.terminal.empty()) {
    window_second = RunDiagnosticBlock(
        "sliding_window_current_continuation", window.graph,
        window_first.terminal, options);
    WriteBlockRows(calls_csv, window_second);
    window_terminal = window_second;
  }

  const Conditioning conditioning = AuditConditioning(
      failed_local.graph, failed_current_continuation.terminal,
      kFailedFrontier);
  auto conditioning_json = DOpen(output / "frontier51_conditioning.json");
  conditioning_json << "{\n  \"schema\": \"GATE06RD_CONDITIONING_V1\",\n"
      << "  \"gt_truth_oracle_read\": false,\n"
      << "  \"optimized_dimension\": " << conditioning.cols << ",\n"
      << "  \"whitened_jacobian_rows\": " << conditioning.rows << ",\n"
      << "  \"whitened_jacobian_rank\": " << conditioning.jacobian_rank << ",\n"
      << "  \"normal_matrix_rank\": " << conditioning.normal_rank << ",\n"
      << "  \"rank_threshold\": " << conditioning.rank_threshold << ",\n"
      << "  \"smallest_meaningful_singular_value\": " << conditioning.singular_min << ",\n"
      << "  \"largest_singular_value\": " << conditioning.singular_max << ",\n"
      << "  \"condition_estimate\": " << conditioning.condition << ",\n"
      << "  \"smallest_meaningful_normal_eigenvalue\": " << conditioning.normal_min << ",\n"
      << "  \"largest_normal_eigenvalue\": " << conditioning.normal_max << ",\n"
      << "  \"singular_values_descending\": [";
  for (Eigen::Index i = 0; i < conditioning.singular_values.size(); ++i) {
    if (i) conditioning_json << ',';
    conditioning_json << conditioning.singular_values[i];
  }
  conditioning_json << "],\n  \"normal_eigenvalues_ascending\": [";
  for (Eigen::Index i = 0; i < conditioning.normal_eigenvalues.size(); ++i) {
    if (i) conditioning_json << ',';
    conditioning_json << conditioning.normal_eigenvalues[i];
  }
  conditioning_json << "],\n"
      << "  \"coordinates\": [";
  const auto names = CoordinateNames();
  for (size_t i = 0; i < names.size(); ++i) {
    if (i) conditioning_json << ',';
    conditioning_json << "{\"name\":\"" << names[i]
      << "\",\"weak_direction\":" << conditioning.weak_direction[i]
      << ",\"gradient_imu_boundary\":" << conditioning.gradient_imu[i]
      << ",\"gradient_uwb\":" << conditioning.gradient_uwb[i]
      << ",\"gradient_total\":" << conditioning.gradient_total[i] << '}';
  }
  conditioning_json << "]\n}\n";

  auto uwb_csv = DOpen(output / "frontier51_uwb_stages.csv");
  uwb_csv << "stage,factor_index,obs_id,anchor_id,timestamp_s,measured_m,"
             "predicted_m,signed_residual_m,sigma_m,standardized_residual,"
             "huber_weight\n";
  WriteUwbStage(uwb_csv, "before", EvaluateUwb(failed_local, failed_local.initial));
  WriteUwbStage(uwb_csv, "after_first", EvaluateUwb(failed_local, failed_first.terminal));
  WriteUwbStage(uwb_csv, "after_current_continuation",
                EvaluateUwb(failed_local, failed_current_continuation.terminal));
  WriteUwbStage(uwb_csv, "after_diagnostic_continuation",
                EvaluateUwb(failed_local, diagnostic_continuation.terminal));
  if (!window_terminal.terminal.empty()) {
    LocalProblem frontier_view = failed_local;
    WriteUwbStage(uwb_csv, "sliding_window_terminal",
                  EvaluateUwb(frontier_view, window_terminal.terminal));
  }

  auto summary = DOpen(output / "summary.json");
  summary << "{\n  \"schema\": \"GATE06R_D_FRONTIER51_DIAGNOSIS_V1\",\n"
      << "  \"gt_truth_oracle_read\": false,\n"
      << "  \"gate06_final_run\": false,\n"
      << "  \"ie_stages_run\": false,\n"
      << "  \"recording_id\": \"" << JsonEscape(prepared.loaded.recording_id) << "\",\n"
      << "  \"physical_state_count\": " << prepared.plan.keyframes.size() << ",\n"
      << "  \"physical_factor_count\": " << prepared.physical_graph.size() << ",\n"
      << "  \"physical_uwb_factor_count\": " << prepared.uwb_indices.size() << ",\n"
      << "  \"frontier51_timestamp_s\": "
      << prepared.plan.keyframes[kFailedFrontier].sensor_time << ",\n"
      << "  \"state50_to_predicted51_displacement_m\": "
      << (failed_local.initial.at<gtsam::Pose3>(X(kFailedFrontier)).translation() -
          committed.at<gtsam::Pose3>(X(kFailedFrontier - 1)).translation()).norm() << ",\n"
      << "  \"frontier51_local_factor_count\": " << failed_local.graph.size() << ",\n"
      << "  \"frontier51_uwb_factor_count\": " << failed_local.uwb.size() << ",\n"
      << "  \"frontier51_anchor_ids\": \""
      << Anchors(EvaluateUwb(failed_local, failed_local.initial)) << "\",\n"
      << "  \"frontier51_factor_types\": [\"FixedBoundaryCombinedImuFactor\",\"ExpressionFactor<double>/Huber\"],\n"
      << "  \"before_state\": ";
  WriteStateJson(summary, failed_local.initial, kFailedFrontier);
  summary << ",\n  \"after_first_state\": ";
  WriteStateJson(summary, failed_first.terminal, kFailedFrontier);
  summary << ",\n  \"after_current_continuation_state\": ";
  WriteStateJson(summary, failed_current_continuation.terminal, kFailedFrontier);
  summary << ",\n  \"after_diagnostic_continuation_state\": ";
  WriteStateJson(summary, diagnostic_continuation.terminal, kFailedFrontier);
  summary << ",\n  \"first_block\": ";
  WriteBlockJson(summary, failed_first);
  summary << ",\n  \"current_continuation\": ";
  WriteBlockJson(summary, failed_current_continuation);
  summary << ",\n  \"diagnostic_identical_continuation\": ";
  WriteBlockJson(summary, diagnostic_continuation);
  summary << ",\n  \"sliding_window\": {\"boundary_state\":" << boundary
      << ",\"first_free_state\":" << boundary + 1
      << ",\"frontier_state\":" << kFailedFrontier
      << ",\"boundary_time_s\":" << prepared.plan.keyframes[boundary].sensor_time
      << ",\"frontier_time_s\":" << prepared.plan.keyframes[kFailedFrontier].sensor_time
      << ",\"span_s\":"
      << prepared.plan.keyframes[kFailedFrontier].sensor_time -
         prepared.plan.keyframes[boundary].sensor_time
      << ",\"optimized_state_count\":" << kFailedFrontier - boundary
      << ",\"factor_count\":" << window.graph.size()
      << ",\"uwb_factor_count\":" << window.uwb.size()
      << ",\"initial_objective\":" << window.graph.error(window.initial)
      << ",\"first_block\":";
  WriteBlockJson(summary, window_first);
  summary << ",\"terminal_block\":";
  WriteBlockJson(summary, window_terminal);
  summary << ",\"latest_state_update_norm\":"
      << StateDeltaNorm(window.initial, window_terminal.terminal,
                        kFailedFrontier)
      << ",\"all_states_finite\":";
  bool window_finite = true;
  for (size_t state = boundary + 1; state <= kFailedFrontier; ++state)
    window_finite = window_finite && StateFinite(window_terminal.terminal, state);
  summary << (window_finite ? "true" : "false") << "}\n}\n";

  std::cout << "GATE06RD_DIAGNOSIS_COMPLETE output=" << output.string()
            << " additional_continuation_stationary="
            << diagnostic_continuation.qualified
            << " sliding_window_stationary=" << window_terminal.qualified
            << '\n';
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  fs::path output;
  try {
    if (argc != 3)
      throw std::invalid_argument(
          "usage: uwb_imu_fgo_frontier51_diagnosis CONFIG OUTPUT_NEW_DIR");
    const fs::path config = fs::absolute(argv[1]);
    output = fs::absolute(argv[2]);
    if (fs::exists(output)) throw std::runtime_error("OUTPUT_ALREADY_EXISTS");
    if (!fs::create_directories(output))
      throw std::runtime_error("CREATE_OUTPUT_FAILED");
    {
      auto command = DOpen(output / "command.txt");
      for (int i = 0; i < argc; ++i) {
        if (i) command << ' ';
        command << argv[i];
      }
      command << '\n';
    }
    return Run(config, output);
  } catch (const std::exception& error) {
    if (!output.empty() && fs::exists(output)) {
      auto failure = DOpen(output / "failure.json");
      failure << "{\n  \"schema\": \"GATE06R_D_FAILURE_V1\",\n"
              << "  \"gt_truth_oracle_read\": false,\n"
              << "  \"reason\": \"" << JsonEscape(error.what())
              << "\"\n}\n";
    }
    std::cerr << "GATE06RD_FAILED: " << error.what() << '\n';
    return 1;
  }
}
