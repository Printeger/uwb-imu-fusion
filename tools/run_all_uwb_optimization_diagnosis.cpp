// Truth-free Gate06 audit binary. Diagnostic modes never publish. The explicit
// warm_start mode may publish only a certified Cauchy final estimate; it never
// reads GT itself.
#define Open Gate06dApplicationOpen
#define RunIePaperApplication gate06d_unused_paper_application
#include "paper/run_ie_app.cpp"
#undef RunIePaperApplication
#undef Open

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ExpressionFactor.h>

#include <numeric>

#include "uifgo/paper_robust_noise.h"

namespace {

constexpr double kCauchyScale = 2.3849;
constexpr double kHuberScale = 1.345;

std::ofstream Open(const fs::path& path) {
  auto out = Gate06dApplicationOpen(path);
  out << std::setprecision(17);
  return out;
}

struct Prepared {
  uifgo::Config cfg;
  LoadedInput loaded;
  std::vector<uifgo::ImuSample> imu;
  std::vector<uifgo::UwbFrame> raw;
  uifgo::EstimatorCorePreparation estimator;
};

Prepared Prepare(const fs::path& config_path) {
  Prepared output;
  output.cfg = uifgo::ConfigLoader::Load(config_path.string());
  ValidateSupportedConfig(output.cfg);
  output.loaded = LoadData(config_path.parent_path().string(), &output.cfg,
                           &output.imu, &output.raw, config_path.string());
  if (output.imu.empty() || output.raw.empty() || output.cfg.anchors.empty())
    throw std::runtime_error("GATE06D_LOADER_RETURNED_INCOMPLETE_INPUT");
  output.estimator = uifgo::PrepareEstimatorCore(
      output.cfg, output.imu, output.raw, output.loaded.recording_id);
  uifgo::ValidatePaperFixedBeta(output.estimator.input_plan, output.cfg);
  return output;
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
  long double sum = 0.0;
  for (double value : values) sum += value;
  return static_cast<double>(sum / values.size());
}

void StatsJson(std::ostream& out, const std::vector<double>& values) {
  if (values.empty()) {
    out << "null";
    return;
  }
  out << "{\"count\":" << values.size() << ",\"mean\":" << Mean(values)
      << ",\"min\":" << *std::min_element(values.begin(), values.end())
      << ",\"p05\":" << Quantile(values, 0.05)
      << ",\"median\":" << Quantile(values, 0.5)
      << ",\"p95\":" << Quantile(values, 0.95)
      << ",\"max\":" << *std::max_element(values.begin(), values.end())
      << '}';
}

bool VectorFinite(const gtsam::Vector& value) { return value.allFinite(); }

struct StateAudit {
  bool finite = true;
  std::vector<double> position_norms;
  std::vector<double> velocity_norms;
  std::vector<double> accel_bias_norms;
  std::vector<double> gyro_bias_norms;
  std::vector<double> consecutive_position_displacements;
};

StateAudit AuditStates(const uifgo::PaperInputPlan& plan,
                       const gtsam::Values& values,
                       const fs::path& csv_path) {
  StateAudit audit;
  auto out = Open(csv_path);
  out << "state_index,state_key,timestamp_s,px,py,pz,position_norm_m,"
         "vx,vy,vz,velocity_norm_mps,bax,bay,baz,accel_bias_norm_mps2,"
         "bgx,bgy,bgz,gyro_bias_norm_radps,previous_displacement_m,finite\n";
  gtsam::Point3 previous;
  for (size_t k = 0; k < plan.keyframes.size(); ++k) {
    const auto pose = values.at<gtsam::Pose3>(X(k));
    const auto velocity = values.at<gtsam::Vector3>(V(k));
    const auto bias = values.at<gtsam::imuBias::ConstantBias>(B(k));
    const auto position = pose.translation();
    const auto ba = bias.accelerometer();
    const auto bg = bias.gyroscope();
    const bool finite = pose.matrix().allFinite() && velocity.allFinite() &&
                        ba.allFinite() && bg.allFinite() &&
                        std::isfinite(plan.keyframes[k].sensor_time);
    audit.finite = audit.finite && finite;
    audit.position_norms.push_back(position.norm());
    audit.velocity_norms.push_back(velocity.norm());
    audit.accel_bias_norms.push_back(ba.norm());
    audit.gyro_bias_norms.push_back(bg.norm());
    const double displacement = k ? (position - previous).norm() : 0.0;
    if (k) audit.consecutive_position_displacements.push_back(displacement);
    out << k << ',' << X(k) << ',' << plan.keyframes[k].sensor_time << ','
        << position.x() << ',' << position.y() << ',' << position.z() << ','
        << position.norm() << ',' << velocity.x() << ',' << velocity.y() << ','
        << velocity.z() << ',' << velocity.norm() << ',' << ba.x() << ','
        << ba.y() << ',' << ba.z() << ',' << ba.norm() << ',' << bg.x() << ','
        << bg.y() << ',' << bg.z() << ',' << bg.norm() << ',' << displacement
        << ',' << finite << '\n';
    previous = position;
  }
  return audit;
}

void StateAuditJson(std::ostream& out, const StateAudit& audit) {
  out << "{\"finite\":" << (audit.finite ? "true" : "false")
      << ",\"position_norm_m\":";
  StatsJson(out, audit.position_norms);
  out << ",\"velocity_norm_mps\":";
  StatsJson(out, audit.velocity_norms);
  out << ",\"accel_bias_norm_mps2\":";
  StatsJson(out, audit.accel_bias_norms);
  out << ",\"gyro_bias_norm_radps\":";
  StatsJson(out, audit.gyro_bias_norms);
  out << ",\"consecutive_position_displacement_m\":";
  StatsJson(out, audit.consecutive_position_displacements);
  out << '}';
}

struct UwbRow {
  size_t order = 0;
  size_t factor_index = 0;
  std::uint64_t obs_id = 0;
  int tag_id = 0;
  int anchor_id = 0;
  double observation_time = 0.0;
  size_t state_index = 0;
  gtsam::Key state_key = 0;
  double state_time = 0.0;
  double mismatch = 0.0;
  double measured = 0.0;
  double predicted = 0.0;
  double geometric = 0.0;
  gtsam::Point3 anchor_world;
  double fixed_beta = 0.0;
  double residual = 0.0;
  double sigma = 0.0;
  double q = 0.0;
  double cauchy_weight = 0.0;
  bool identity_ok = false;
  bool equation_ok = false;
};

const uifgo::AnchorConfig& Anchor(const uifgo::Config& cfg, int anchor_id) {
  const auto it = std::find_if(cfg.anchors.begin(), cfg.anchors.end(),
      [&](const auto& anchor) { return anchor.id == anchor_id; });
  if (it == cfg.anchors.end()) throw std::runtime_error("ANCHOR_NOT_FOUND");
  return *it;
}

std::vector<UwbRow> AuditUwb(const Prepared& prepared) {
  const auto& estimator = prepared.estimator;
  std::unordered_map<std::uint64_t, const uifgo::ObservationRecord*> records;
  std::unordered_map<std::uint64_t, const uifgo::MeasurementPlanEntry*> plans;
  for (const auto& record : estimator.input_plan.observations) {
    records.emplace(record.obs_id, &record);
    plans.emplace(record.obs_id,
                  &uifgo::MeasurementForObservation(estimator.input_plan,
                                                     record));
  }
  std::vector<UwbRow> rows;
  for (const auto& meta : estimator.factor_metadata) {
    if (meta.factor_type != "uwb_range") continue;
    if (meta.factor_index >= estimator.graph.size() || meta.keys.size() != 1)
      throw std::runtime_error("UWB_FACTOR_IDENTITY_INVALID");
    const auto record_it = records.find(meta.obs_id);
    const auto plan_it = plans.find(meta.obs_id);
    if (record_it == records.end() || plan_it == plans.end())
      throw std::runtime_error("UWB_OBSERVATION_NOT_IN_PLAN");
    const auto& record = *record_it->second;
    const auto& measurement = *plan_it->second;
    const gtsam::Symbol symbol(meta.keys.front());
    const size_t state = symbol.index();
    const auto factor = boost::dynamic_pointer_cast<gtsam::ExpressionFactor<double>>(
        estimator.graph.at(meta.factor_index));
    if (!factor || symbol.chr() != 'x' || state >= estimator.input_plan.keyframes.size())
      throw std::runtime_error("UWB_FACTOR_TYPE_OR_STATE_KEY_INVALID");
    const auto residual_vector = factor->unwhitenedError(estimator.initial_values);
    const auto sigmas = factor->noiseModel()->sigmas();
    if (residual_vector.size() != 1 || sigmas.size() != 1)
      throw std::runtime_error("UWB_FACTOR_NOT_SCALAR");
    const auto pose = estimator.initial_values.at<gtsam::Pose3>(meta.keys.front());
    const auto& anchor = Anchor(prepared.cfg, record.anchor_id);
    const auto antenna = pose.transformFrom(prepared.cfg.lever_arm_init);
    const double geometric = (antenna - anchor.pos).norm();
    const double beta = uifgo::FixedBetaForLink(prepared.cfg, record.tag_id,
                                                record.anchor_id);
    UwbRow row;
    row.order = rows.size();
    row.factor_index = meta.factor_index;
    row.obs_id = meta.obs_id;
    row.tag_id = record.tag_id;
    row.anchor_id = record.anchor_id;
    row.observation_time = record.sensor_time;
    row.state_index = state;
    row.state_key = meta.keys.front();
    row.state_time = estimator.input_plan.keyframes[state].sensor_time;
    row.mismatch = std::abs(row.observation_time - row.state_time);
    row.measured = factor->measured();
    row.residual = residual_vector[0];
    row.predicted = row.measured + row.residual;
    row.geometric = geometric;
    row.anchor_world = anchor.pos;
    row.fixed_beta = beta;
    row.sigma = sigmas[0];
    row.q = row.residual / row.sigma;
    row.cauchy_weight = 1.0 / (1.0 + std::pow(row.q / kCauchyScale, 2));
    row.identity_ok = measurement.estimator_usable && measurement.selected &&
        measurement.independent_likelihood_representative &&
        measurement.keyframe_id == state && meta.obs_id == record.obs_id;
    const double scale = std::max({1.0, std::abs(row.predicted),
                                   std::abs(geometric + beta)});
    row.equation_ok = std::abs(row.predicted - geometric - beta) <=
        32.0 * std::numeric_limits<double>::epsilon() * scale;
    rows.push_back(row);
  }
  return rows;
}

void WriteUwbRows(const fs::path& path, const std::vector<UwbRow>& rows) {
  std::set<size_t> representatives;
  for (size_t j = 0; j < 20; ++j)
    representatives.insert(static_cast<size_t>(std::llround(
        static_cast<double>(j) * static_cast<double>(rows.size() - 1) / 19.0)));
  std::vector<size_t> by_residual(rows.size());
  std::iota(by_residual.begin(), by_residual.end(), 0);
  std::stable_sort(by_residual.begin(), by_residual.end(), [&](size_t a, size_t b) {
    return std::abs(rows[a].residual) > std::abs(rows[b].residual);
  });
  std::map<size_t, size_t> top_rank;
  for (size_t rank = 0; rank < std::min<size_t>(20, rows.size()); ++rank)
    top_rank[by_residual[rank]] = rank + 1;
  auto out = Open(path);
  out << "factor_order,factor_index,obs_id,tag_id,anchor_id,observation_timestamp_s,"
         "state_index,state_key,state_timestamp_s,absolute_timestamp_mismatch_s,"
         "measured_range_m,predicted_range_m,geometric_range_m,fixed_beta_m,"
         "anchor_world_x_m,anchor_world_y_m,anchor_world_z_m,"
         "signed_raw_residual_m,sensor_sigma_m,standardized_residual,cauchy_weight,"
         "identity_ok,equation_ok,top20_initial_abs_residual_rank,"
         "deterministic_representative\n";
  for (const auto& row : rows) {
    out << row.order << ',' << row.factor_index << ',' << row.obs_id << ','
        << row.tag_id << ',' << row.anchor_id << ',' << row.observation_time << ','
        << row.state_index << ',' << row.state_key << ',' << row.state_time << ','
        << row.mismatch << ',' << row.measured << ',' << row.predicted << ','
        << row.geometric << ',' << row.fixed_beta << ',' << row.anchor_world.x()
        << ',' << row.anchor_world.y() << ',' << row.anchor_world.z() << ','
        << row.residual << ','
        << row.sigma << ',' << row.q << ',' << row.cauchy_weight << ','
        << row.identity_ok << ',' << row.equation_ok << ',';
    if (top_rank.count(row.order)) out << top_rank.at(row.order);
    out << ',' << representatives.count(row.order) << '\n';
  }
}

void WriteUwbSummary(const fs::path& path, const std::vector<UwbRow>& rows) {
  std::map<int, std::vector<const UwbRow*>> groups;
  groups[-1] = {};
  for (const auto& row : rows) {
    groups[-1].push_back(&row);
    groups[row.anchor_id].push_back(&row);
  }
  auto out = Open(path);
  out << "scope,anchor_id,count,mean_abs_residual_m,median_abs_residual_m,"
         "p95_abs_residual_m,max_abs_residual_m,median_abs_q,p95_abs_q,max_abs_q,"
         "weight_median,weight_p05,weight_p95,fraction_weight_gt_0_5,"
         "fraction_weight_gt_0_1,fraction_weight_lt_0_01,"
         "fraction_weight_lt_1e_3,fraction_weight_lt_1e_6\n";
  for (const auto& group : groups) {
    std::vector<double> residual, q, weight;
    size_t gt05 = 0, gt01 = 0, lt001 = 0, lt1e3 = 0, lt1e6 = 0;
    for (const auto* row : group.second) {
      residual.push_back(std::abs(row->residual));
      q.push_back(std::abs(row->q));
      weight.push_back(row->cauchy_weight);
      gt05 += row->cauchy_weight > 0.5;
      gt01 += row->cauchy_weight > 0.1;
      lt001 += row->cauchy_weight < 0.01;
      lt1e3 += row->cauchy_weight < 1e-3;
      lt1e6 += row->cauchy_weight < 1e-6;
    }
    const double n = group.second.size();
    out << (group.first == -1 ? "global" : "anchor") << ',';
    if (group.first != -1) out << group.first;
    out << ',' << group.second.size() << ',' << Mean(residual) << ','
        << Quantile(residual, 0.5) << ',' << Quantile(residual, 0.95) << ','
        << *std::max_element(residual.begin(), residual.end()) << ','
        << Quantile(q, 0.5) << ',' << Quantile(q, 0.95) << ','
        << *std::max_element(q.begin(), q.end()) << ','
        << Quantile(weight, 0.5) << ',' << Quantile(weight, 0.05) << ','
        << Quantile(weight, 0.95) << ',' << gt05 / n << ',' << gt01 / n << ','
        << lt001 / n << ',' << lt1e3 / n << ',' << lt1e6 / n << '\n';
  }
}

void WriteImuAudit(const Prepared& prepared, const fs::path& path) {
  const auto& graph = prepared.estimator.graph;
  const auto& plan = prepared.estimator.input_plan;
  std::map<size_t, std::pair<size_t, boost::shared_ptr<gtsam::CombinedImuFactor>>> by_k;
  for (size_t index = 0; index < graph.size(); ++index) {
    auto factor = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(graph.at(index));
    if (!factor) continue;
    const auto keys = factor->keys();
    if (keys.size() != 6) throw std::runtime_error("COMBINED_IMU_KEY_COUNT");
    const size_t k = gtsam::Symbol(keys[2]).index();
    if (!by_k.emplace(k, std::make_pair(index, factor)).second)
      throw std::runtime_error("DUPLICATE_COMBINED_IMU_INTERVAL");
  }
  auto out = Open(path);
  out << "interval_index,factor_index,t0_s,t1_s,timeline_dt_s,pim_dt_s,"
         "interior_imu_samples,keys,keys_expected,times_finite_increasing\n";
  for (size_t k = 1; k < plan.keyframes.size(); ++k) {
    if (!by_k.count(k)) throw std::runtime_error("MISSING_COMBINED_IMU_INTERVAL");
    const auto& item = by_k.at(k);
    const auto keys = item.second->keys();
    const std::vector<gtsam::Key> expected =
        {X(k - 1), V(k - 1), X(k), V(k), B(k - 1), B(k)};
    const double t0 = plan.keyframes[k - 1].sensor_time;
    const double t1 = plan.keyframes[k].sensor_time;
    size_t samples = 0;
    for (const auto& sample : prepared.imu)
      samples += sample.t > t0 && sample.t < t1;
    out << k << ',' << item.first << ',' << t0 << ',' << t1 << ','
        << t1 - t0 << ',' << item.second->preintegratedMeasurements().deltaTij()
        << ',' << samples << ",\"";
    for (const auto key : keys) out << key << ';';
    out << "\"," << (std::vector<gtsam::Key>(keys.begin(), keys.end()) == expected)
        << ',' << (std::isfinite(t0) && std::isfinite(t1) && t1 > t0) << '\n';
  }
  if (by_k.size() != plan.keyframes.size() - 1)
    throw std::runtime_error("EXTRA_COMBINED_IMU_INTERVAL");
}

void WriteInitializationWindows(
    const fs::path& path,
    const uifgo::CommonInitializationResult& initialization) {
  auto out = Open(path);
  out << "ordinal,boundary_state,free_start_state,frontier_state,frontier_time_s,"
         "window_duration_s,free_window_duration_s,boundary_bridge_duration_s,"
         "window_state_count,optimized_state_count,"
         "imu_factor_count,uwb_factor_count,lm_calls,accepted_updates,"
         "rejected_lambda_trials,retry_count,lm_reason,initial_objective,"
         "terminal_objective,stationarity_valid,stationary,"
         "max_scaled_navigation_gradient,roundoff_allowance,"
         "dominant_key,dominant_coordinate,dominant_category,"
         "seed_quality_accepted,seed_quality_reason,"
         "max_position_from_boundary_m,position_envelope_m,"
         "max_velocity_change_from_boundary_mps,velocity_envelope_mps,"
         "entering_median_abs_uwb_residual_m,"
         "terminal_median_abs_uwb_residual_m,runtime_s\n";
  for (const auto& prefix : initialization.prefixes) {
    out << prefix.ordinal << ',' << prefix.start_state << ','
        << prefix.free_start_state << ',' << prefix.frontier_state << ','
        << prefix.frontier_time_s << ',' << prefix.window_duration_s << ','
        << prefix.free_window_duration_s << ','
        << prefix.boundary_bridge_duration_s << ','
        << prefix.window_state_count << ','
        << prefix.optimized_state_count << ',' << prefix.imu_factor_count << ','
        << prefix.uwb_factor_count << ',' << prefix.lm_calls << ','
        << prefix.accepted_updates << ',' << prefix.rejected_lambda_trials << ','
        << prefix.retry_count << ",\"" << JsonEscape(prefix.lm_reason) << "\","
        << prefix.initial_objective << ',' << prefix.terminal_objective << ','
        << prefix.stationarity.valid << ',' << prefix.stationarity.stationary
        << ',' << prefix.stationarity.max_scaled_gradient_objective << ','
        << prefix.stationarity.roundoff_allowance_objective << ",\""
        << JsonEscape(prefix.stationarity.dominant_key_name) << "\","
        << prefix.stationarity.dominant_coordinate << ",\""
        << JsonEscape(prefix.stationarity.dominant_category) << "\","
        << prefix.seed_quality.accepted << ",\""
        << JsonEscape(prefix.seed_quality.reason) << "\","
        << prefix.seed_quality.max_position_from_boundary_m << ','
        << prefix.seed_quality.position_envelope_m << ','
        << prefix.seed_quality.max_velocity_change_from_boundary_mps << ','
        << prefix.seed_quality.velocity_change_envelope_mps << ','
        << prefix.seed_quality.entering_median_abs_uwb_residual_m << ','
        << prefix.seed_quality.terminal_median_abs_uwb_residual_m << ','
        << prefix.runtime_s << '\n';
  }
}

uifgo::InferenceContentIdentity Identity(const Prepared& prepared) {
  uifgo::InferenceIdentityContext context;
  context.input_sha256 = prepared.estimator.input_plan.observation_ledger_sha256;
  context.config_sha256 = "GATE06D_LOCKED_CONFIG";
  context.input_plan_sha256 = prepared.estimator.input_plan.plan_sha256;
  context.support_partition_sha256 = "NOT_APPLICABLE_BASELINE";
  context.calibration_sha256 = "GATE06D_LOCKED_CALIBRATION";
  context.solver_config_sha256 = "LM100_REL1E-6_ABS1E-8";
  return uifgo::ComputeInferenceContentIdentity(prepared.estimator.graph,
                                                prepared.estimator.initial_values,
                                                context);
}

void RunAudit(const Prepared& prepared, const fs::path& output,
              bool write_sparse_comparator) {
  const auto states = AuditStates(prepared.estimator.input_plan,
                                  prepared.estimator.initial_values,
                                  output / "initial_states.csv");
  const auto rows = AuditUwb(prepared);
  WriteUwbRows(output / "uwb_association_initial.csv", rows);
  WriteUwbSummary(output / "uwb_initial_summary.csv", rows);
  WriteImuAudit(prepared, output / "imu_intervals.csv");
  WriteInitializationWindows(
      output / "initialization_windows.csv",
      prepared.estimator.common_initialization);

  bool identity_ok = true, equation_ok = true, sigma_ok = true;
  std::vector<double> mismatch;
  std::set<int> anchors;
  for (const auto& row : rows) {
    identity_ok = identity_ok && row.identity_ok;
    equation_ok = equation_ok && row.equation_ok;
    sigma_ok = sigma_ok && row.sigma == 0.15;
    mismatch.push_back(row.mismatch);
    anchors.insert(row.anchor_id);
  }
  std::vector<double> window_duration_s, window_state_count,
      optimized_state_count, maximum_scaled_gradient;
  std::map<std::string, size_t> lm_termination_counts;
  size_t windows_above_scientific_stationarity = 0;
  const uifgo::CommonInitializationPrefix* frontier_51 = nullptr;
  for (const auto& prefix : prepared.estimator.common_initialization.prefixes) {
    window_duration_s.push_back(prefix.window_duration_s);
    window_state_count.push_back(prefix.window_state_count);
    optimized_state_count.push_back(prefix.optimized_state_count);
    maximum_scaled_gradient.push_back(
        prefix.stationarity.max_scaled_gradient_objective);
    ++lm_termination_counts[prefix.lm_reason];
    windows_above_scientific_stationarity +=
        prefix.stationarity.max_scaled_gradient_objective > 1e-5;
    if (prefix.frontier_state == 51) frontier_51 = &prefix;
  }
  const auto identity = Identity(prepared);
  auto summary = Open(output / "audit_summary.json");
  summary << "{\n  \"schema\": \""
          << (write_sparse_comparator ? "REFACTOR_GATE06D_AUDIT_V1"
                                      : "REFACTOR_GATE06RA_INITIALIZATION_AUDIT_V1")
          << "\",\n"
          << "  \"gt_truth_oracle_read\": false,\n"
          << "  \"final_estimator_run\": false,\n"
          << "  \"ie_stages_run\": false,\n"
          << "  \"recording_id\": \"" << JsonEscape(prepared.loaded.recording_id) << "\",\n"
          << "  \"state_count\": " << prepared.estimator.input_plan.keyframes.size() << ",\n"
          << "  \"graph_factor_count\": " << prepared.estimator.graph.size() << ",\n"
          << "  \"uwb_factor_count\": " << rows.size() << ",\n"
          << "  \"initial_graph_objective\": "
          << prepared.estimator.graph.error(prepared.estimator.initial_values) << ",\n"
          << "  \"graph_linearization_sha256\": \""
          << identity.graph_linearization_sha256 << "\",\n"
          << "  \"initial_values_sha256\": \"" << identity.values_sha256 << "\",\n"
          << "  \"common_initializer_identity\": \""
          << JsonEscape(prepared.estimator.common_initialization.identity)
          << "\",\n"
          << "  \"initialization_progression_horizon_s\": "
          << prepared.cfg.initialization_progression_horizon_s << ",\n"
          << "  \"initializer_prefix_count\": "
          << prepared.estimator.common_initialization.prefixes.size() << ",\n"
          << "  \"initializer_accepted_prefix_count\": "
          << prepared.estimator.common_initialization.accepted_prefix_count
          << ",\n"
          << "  \"initializer_failure_count\": "
          << prepared.estimator.common_initialization.failure_count << ",\n"
          << "  \"initializer_retry_count\": "
          << prepared.estimator.common_initialization.retry_count << ",\n"
          << "  \"initializer_runtime_s\": "
          << prepared.estimator.common_initialization.runtime_s << ",\n"
          << "  \"initializer_window_duration_s\": ";
  StatsJson(summary, window_duration_s);
  summary << ",\n  \"initializer_window_state_count\": ";
  StatsJson(summary, window_state_count);
  summary << ",\n  \"initializer_optimized_state_count\": ";
  StatsJson(summary, optimized_state_count);
  summary << ",\n  \"initializer_max_scaled_gradient\": ";
  StatsJson(summary, maximum_scaled_gradient);
  summary << ",\n  \"initializer_windows_above_1e_5\": "
          << windows_above_scientific_stationarity
          << ",\n  \"initializer_raw_lm_termination_counts\": {";
  bool first_termination = true;
  for (const auto& item : lm_termination_counts) {
    if (!first_termination) summary << ',';
    first_termination = false;
    summary << "\"" << JsonEscape(item.first) << "\":" << item.second;
  }
  const auto& full_seed = prepared.estimator.common_initialization.full_seed_quality;
  summary << "},\n  \"initialization_seed_quality\": {"
          << "\"policy\":\"INITIALIZATION_SEED_QUALITY_V1\""
          << ",\"evaluated\":" << (full_seed.evaluated ? "true" : "false")
          << ",\"accepted\":" << (full_seed.accepted ? "true" : "false")
          << ",\"reason\":\"" << JsonEscape(full_seed.reason) << "\""
          << ",\"all_states_finite\":"
          << (full_seed.all_states_finite ? "true" : "false")
          << ",\"motion_finite_and_local\":"
          << (full_seed.motion_finite_and_local ? "true" : "false")
          << ",\"all_uwb_predictions_finite\":"
          << (full_seed.all_uwb_predictions_finite ? "true" : "false")
          << ",\"state_count\":" << full_seed.state_count
          << ",\"uwb_factor_count\":" << full_seed.uwb_factor_count
          << ",\"max_position_norm_m\":" << full_seed.max_position_norm_m
          << ",\"position_envelope_m\":" << full_seed.position_envelope_m
          << ",\"max_velocity_norm_mps\":" << full_seed.max_velocity_norm_mps
          << ",\"velocity_envelope_mps\":" << full_seed.velocity_envelope_mps
          << ",\"max_consecutive_displacement_m\":"
          << full_seed.max_consecutive_displacement_m
          << ",\"mean_abs_uwb_residual_m\":"
          << full_seed.mean_abs_uwb_residual_m
          << ",\"median_abs_uwb_residual_m\":"
          << full_seed.median_abs_uwb_residual_m
          << ",\"p95_abs_uwb_residual_m\":"
          << full_seed.p95_abs_uwb_residual_m
          << ",\"max_abs_uwb_residual_m\":"
          << full_seed.max_abs_uwb_residual_m
          << ",\"median_abs_standardized_residual\":"
          << full_seed.median_abs_standardized_residual
          << ",\"p95_abs_standardized_residual\":"
          << full_seed.p95_abs_standardized_residual
          << ",\"max_abs_standardized_residual\":"
          << full_seed.max_abs_standardized_residual
          << ",\"cauchy_weight_p05\":" << full_seed.cauchy_weight_p05
          << ",\"cauchy_weight_median\":" << full_seed.cauchy_weight_median
          << ",\"cauchy_weight_p95\":" << full_seed.cauchy_weight_p95
          << ",\"fraction_weight_gt_0_5\":"
          << full_seed.fraction_weight_gt_0_5
          << ",\"fraction_weight_gt_0_1\":"
          << full_seed.fraction_weight_gt_0_1
          << ",\"fraction_weight_lt_0_01\":"
          << full_seed.fraction_weight_lt_0_01
          << ",\"fraction_weight_lt_1e_3\":"
          << full_seed.fraction_weight_lt_1e_3 << '}';
  summary << ",\n  \"frontier_51\": ";
  if (frontier_51) {
    summary << "{\"boundary_state\":" << frontier_51->start_state
            << ",\"frontier_state\":" << frontier_51->frontier_state
            << ",\"optimized_states\":[";
    for (size_t k = frontier_51->start_state + 1;
         k <= frontier_51->frontier_state; ++k) {
      if (k != frontier_51->start_state + 1) summary << ',';
      summary << k;
    }
    summary << "],\"lm_calls\":" << frontier_51->lm_calls
            << ",\"termination\":\""
            << JsonEscape(frontier_51->lm_reason) << "\""
            << ",\"stationarity_valid\":"
            << (frontier_51->stationarity.valid ? "true" : "false")
            << ",\"stationary\":"
            << (frontier_51->stationarity.stationary ? "true" : "false")
            << ",\"max_scaled_gradient\":"
            << frontier_51->stationarity.max_scaled_gradient_objective << '}';
  } else {
    summary << "null";
  }
  summary << ",\n"
          << "  \"initial_states\": ";
  StateAuditJson(summary, states);
  summary << ",\n  \"timestamp_mismatch_s\": ";
  StatsJson(summary, mismatch);
  summary << ",\n  \"association_identity_ok\": " << (identity_ok ? "true" : "false")
          << ",\n  \"factor_equation_ok\": " << (equation_ok ? "true" : "false")
          << ",\n  \"all_sigma_exactly_0_15\": " << (sigma_ok ? "true" : "false")
          << ",\n  \"world_body_convention\": \"Pose3 X is body-to-world; antenna=p+R*lever_body\",\n"
          << "  \"range_equation\": \"norm(p+R*lever_body-anchor_world)+fixed_beta-measured\",\n"
          << "  \"anchor_count_used\": " << anchors.size() << ",\n"
          << "  \"lever_arm_body_m\": [" << prepared.cfg.lever_arm_init.x() << ','
          << prepared.cfg.lever_arm_init.y() << ',' << prepared.cfg.lever_arm_init.z() << "]\n}\n";

  if (!write_sparse_comparator) return;

  uifgo::Config sparse_cfg = prepared.cfg;
  sparse_cfg.kf_step = 4;
  const auto sparse = uifgo::PrepareEstimatorCore(
      sparse_cfg, prepared.imu, prepared.raw, prepared.loaded.recording_id);
  auto comparison = Open(output / "dense_sparse_construction_comparison.json");
  comparison << "{\n  \"shared_preparation_function\": \"uifgo::PrepareEstimatorCore\",\n"
             << "  \"shared_graph_builder\": \"uifgo::GraphBuilder::Build\",\n"
             << "  \"step1_states\": " << prepared.estimator.input_plan.keyframes.size() << ",\n"
             << "  \"step4_states\": " << sparse.input_plan.keyframes.size() << ",\n"
             << "  \"step1_uwb_factors\": " << prepared.estimator.uwb_factor_indices.size() << ",\n"
             << "  \"step4_uwb_factors\": " << sparse.uwb_factor_indices.size() << ",\n"
             << "  \"step1_graph_factors\": " << prepared.estimator.graph.size() << ",\n"
             << "  \"step4_graph_factors\": " << sparse.graph.size() << ",\n"
             << "  \"observation_ledger_sha256_equal\": "
             << (prepared.estimator.input_plan.observation_ledger_sha256 ==
                 sparse.input_plan.observation_ledger_sha256 ? "true" : "false") << ",\n"
             << "  \"difference\": \"Only frozen state timeline, selected observations, resulting adjacent IMU intervals and counts differ; factor construction code path is shared.\"\n}\n";
}

gtsam::NonlinearFactorGraph LossGraph(const Prepared& prepared,
                                      const std::string& mode) {
  std::set<size_t> uwb(prepared.estimator.uwb_factor_indices.begin(),
                        prepared.estimator.uwb_factor_indices.end());
  gtsam::NonlinearFactorGraph graph;
  for (size_t index = 0; index < prepared.estimator.graph.size(); ++index) {
    const auto& factor = prepared.estimator.graph.at(index);
    if (!uwb.count(index) || mode == "gaussian") {
      graph.add(factor);
      continue;
    }
    auto noise_factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
    if (!noise_factor) throw std::runtime_error("UWB_NOT_NOISE_MODEL_FACTOR");
    gtsam::noiseModel::mEstimator::Base::shared_ptr kernel;
    if (mode == "cauchy")
      kernel = gtsam::noiseModel::mEstimator::Cauchy::Create(kCauchyScale);
    else if (mode == "huber")
      kernel = gtsam::noiseModel::mEstimator::Huber::Create(kHuberScale);
    else
      throw std::invalid_argument("UNKNOWN_LOSS_MODE");
    graph.add(noise_factor->cloneWithNewNoiseModel(
        boost::make_shared<uifgo::PaperRobustNoise>(kernel,
                                                    noise_factor->noiseModel())));
  }
  return graph;
}

double ReplayWeight(const std::string& mode, double q) {
  const double absolute = std::abs(q);
  if (mode == "cauchy") return 1.0 / (1.0 + std::pow(q / kCauchyScale, 2));
  if (mode == "huber") return absolute <= kHuberScale ? 1.0 : kHuberScale / absolute;
  return 1.0;
}

struct ResidualAudit {
  std::vector<double> absolute_m;
  std::vector<double> absolute_q;
  std::vector<double> weight;
};

ResidualAudit WriteReplayResiduals(const Prepared& prepared,
                                   const gtsam::Values& initial,
                                   const gtsam::Values& terminal,
                                   const std::string& mode,
                                   const fs::path& path) {
  const auto before = uifgo::ReadScalarUwbFactorResiduals(
      prepared.estimator.graph, initial, prepared.estimator.factor_metadata);
  const auto after = uifgo::ReadScalarUwbFactorResiduals(
      prepared.estimator.graph, terminal, prepared.estimator.factor_metadata);
  if (before.size() != after.size()) throw std::runtime_error("RESIDUAL_COUNT_MISMATCH");
  ResidualAudit terminal_audit;
  auto out = Open(path);
  out << "factor_index,obs_id,sigma_m,initial_residual_m,initial_abs_q,"
         "terminal_residual_m,terminal_abs_q,initial_loss_weight,terminal_loss_weight\n";
  for (size_t i = 0; i < before.size(); ++i) {
    if (before[i].factor_index != after[i].factor_index ||
        before[i].obs_id != after[i].obs_id || before[i].sigma_m != after[i].sigma_m)
      throw std::runtime_error("RESIDUAL_IDENTITY_MISMATCH");
    const double q0 = before[i].residual_m / before[i].sigma_m;
    const double q1 = after[i].residual_m / after[i].sigma_m;
    const double w0 = ReplayWeight(mode, q0);
    const double w1 = ReplayWeight(mode, q1);
    terminal_audit.absolute_m.push_back(std::abs(after[i].residual_m));
    terminal_audit.absolute_q.push_back(std::abs(q1));
    terminal_audit.weight.push_back(w1);
    out << before[i].factor_index << ',' << before[i].obs_id << ','
        << before[i].sigma_m << ',' << before[i].residual_m << ',' << std::abs(q0)
        << ',' << after[i].residual_m << ',' << std::abs(q1) << ',' << w0 << ','
        << w1 << '\n';
  }
  return terminal_audit;
}

void RunReplay(const Prepared& prepared, const std::string& mode,
               const fs::path& output) {
  const auto graph = LossGraph(prepared, mode);
  if (!uifgo::GraphAndValuesKeysMatch(graph, prepared.estimator.initial_values))
    throw std::runtime_error("REPLAY_GRAPH_VALUES_KEYS_MISMATCH");
  uifgo::CheckedLmOptions options;
  options.max_iterations = prepared.cfg.lm_max_iter;
  options.relative_tolerance = prepared.cfg.lm_rel_tol;
  options.absolute_tolerance = prepared.cfg.lm_abs_tol;
  uifgo::CheckedLmDiagnosticRequest request;
  request.finite_difference_steps = {1e-4, 1e-5, 1e-6};
  const auto result = uifgo::RunCheckedConditionalLm(
      graph, prepared.estimator.initial_values, options, &request);
  if (result.diagnostic.values_at_final.empty())
    throw std::runtime_error("REPLAY_TERMINAL_VALUES_NOT_CAPTURED");
  const auto& terminal = result.diagnostic.values_at_final;
  const auto initial_states = AuditStates(prepared.estimator.input_plan,
                                          prepared.estimator.initial_values,
                                          output / "initial_states.csv");
  const auto terminal_states = AuditStates(prepared.estimator.input_plan,
                                           terminal,
                                           output / "terminal_states_uncertified.csv");
  const auto residuals = WriteReplayResiduals(
      prepared, prepared.estimator.initial_values, terminal, mode,
      output / "uwb_residuals.csv");
  double last_call_delta = std::numeric_limits<double>::quiet_NaN();
  double last_accepted_delta = std::numeric_limits<double>::quiet_NaN();
  if (!result.diagnostic.calls.empty()) {
    last_call_delta = result.diagnostic.calls.back().accepted_values_delta_norm;
    for (const auto& call : result.diagnostic.calls)
      if (call.accepted_state_update) last_accepted_delta = call.accepted_values_delta_norm;
  }
  size_t active = 0, effective = 0;
  for (double weight : residuals.weight) {
    active += weight < 1.0 - 1e-12;
    effective += weight < 0.5;
  }
  const auto& stationarity = result.diagnostic.stationarity_at_final;
  const auto identity = Identity(prepared);
  auto summary = Open(output / "replay_summary.json");
  summary << "{\n  \"schema\": \"REFACTOR_GATE06D_REPLAY_V1\",\n"
          << "  \"mode\": \"" << mode << "\",\n"
          << "  \"robust_parameter\": ";
  if (mode == "cauchy") summary << kCauchyScale;
  else if (mode == "huber") summary << kHuberScale;
  else summary << "null";
  summary << ",\n  \"gt_truth_oracle_read\": false,\n"
          << "  \"base_graph_linearization_sha256\": \""
          << identity.graph_linearization_sha256 << "\",\n"
          << "  \"common_initial_values_sha256\": \""
          << identity.values_sha256 << "\",\n"
          << "  \"solver_max_calls\": " << options.max_iterations << ",\n"
          << "  \"solver_relative_tolerance\": " << options.relative_tolerance << ",\n"
          << "  \"solver_absolute_tolerance\": " << options.absolute_tolerance << ",\n"
          << "  \"raw_converged\": " << (result.converged ? "true" : "false") << ",\n"
          << "  \"raw_termination\": \"" << JsonEscape(result.reason) << "\",\n"
          << "  \"call_count\": " << result.diagnostic.calls.size() << ",\n"
          << "  \"optimizer_accepted_iterations\": " << result.iterations << ",\n"
          << "  \"initial_objective\": " << graph.error(prepared.estimator.initial_values) << ",\n"
          << "  \"terminal_objective\": " << graph.error(terminal) << ",\n"
          << "  \"last_call_values_delta_norm\": " << last_call_delta << ",\n"
          << "  \"last_accepted_values_delta_norm\": " << last_accepted_delta << ",\n"
          << "  \"initial_states\": ";
  StateAuditJson(summary, initial_states);
  summary << ",\n  \"terminal_states_uncertified\": ";
  StateAuditJson(summary, terminal_states);
  summary << ",\n  \"terminal_raw_uwb_abs_residual_m\": ";
  StatsJson(summary, residuals.absolute_m);
  summary << ",\n  \"terminal_raw_uwb_abs_standardized_residual\": ";
  StatsJson(summary, residuals.absolute_q);
  summary << ",\n  \"terminal_loss_weight\": ";
  StatsJson(summary, residuals.weight);
  summary << ",\n  \"downweighted_fraction_weight_lt_1_minus_1e_12\": "
          << static_cast<double>(active) / residuals.weight.size()
          << ",\n  \"effectively_downweighted_definition\": \"weight < 0.5\",\n"
          << "  \"effectively_downweighted_fraction\": "
          << static_cast<double>(effective) / residuals.weight.size()
          << ",\n  \"navigation_stationarity_valid\": "
          << (stationarity.valid ? "true" : "false")
          << ",\n  \"navigation_stationary\": "
          << (stationarity.stationary ? "true" : "false")
          << ",\n  \"navigation_stationarity_reason\": \""
          << JsonEscape(stationarity.reason) << "\",\n"
          << "  \"max_scaled_navigation_gradient_objective\": "
          << stationarity.max_scaled_gradient_objective << "\n}\n";

  auto calls = Open(output / "lm_calls.csv");
  calls << "call_index,optimizer_iterations_before,optimizer_iterations_after,"
           "error_before,error_after,lambda_before,lambda_after,values_delta_norm,"
           "accepted_state_update,observed_return_class\n";
  for (const auto& call : result.diagnostic.calls)
    calls << call.call_index << ',' << call.optimizer_iterations_before << ','
          << call.optimizer_iterations_after << ',' << call.error_before << ','
          << call.error_after << ',' << call.lambda_before << ',' << call.lambda_after
          << ',' << call.accepted_values_delta_norm << ',' << call.accepted_state_update
          << ',' << call.observed_return_class << '\n';
}

void WarmStageJson(std::ostream& out, const StateAudit& states,
                   const ResidualAudit& residuals, double objective,
                   const uifgo::CheckedLmResult* solve,
                   const uifgo::SolverCertificate* certificate) {
  out << "{\"objective\":" << JsonNumberOrNull(objective)
      << ",\"states\":";
  StateAuditJson(out, states);
  out << ",\"raw_uwb_abs_residual_m\":";
  StatsJson(out, residuals.absolute_m);
  out << ",\"absolute_standardized_uwb_residual\":";
  StatsJson(out, residuals.absolute_q);
  out << ",\"robust_weight\":";
  StatsJson(out, residuals.weight);
  if (solve) {
    out << ",\"raw_termination\":\"" << JsonEscape(solve->reason)
        << "\",\"iterations\":" << solve->iterations;
  } else {
    out << ",\"raw_termination\":\"NOT_RUN_INITIALIZATION_SEED\""
        << ",\"iterations\":0";
  }
  if (certificate) {
    out << ",\"solver_certificate\":\""
        << uifgo::SolverCertificateStatusName(certificate->status)
        << "\",\"stationarity_valid\":"
        << (certificate->navigation_stationarity.valid ? "true" : "false")
        << ",\"stationary\":"
        << (certificate->navigation_stationarity.stationary ? "true" : "false")
        << ",\"max_scaled_navigation_gradient\":"
        << JsonNumberOrNull(
               certificate->navigation_stationarity.max_scaled_gradient_objective);
  } else {
    out << ",\"solver_certificate\":\"NOT_APPLICABLE_INITIALIZATION_SEED\""
        << ",\"stationarity_valid\":false,\"stationary\":false"
        << ",\"max_scaled_navigation_gradient\":null";
  }
  out << '}';
}

void RunWarmStart(const Prepared& prepared, const fs::path& output) {
  uifgo::BaselineOptions huber;
  huber.method = uifgo::PaperMethod::ROBUST_HUBER;
  huber.robust_scale = kHuberScale;
  huber.parameter_provenance = "LOCKED_GATE06_HUBER_CAUCHY_WARM_START";
  uifgo::BaselineOptions cauchy = huber;
  cauchy.method = uifgo::PaperMethod::ROBUST_CAUCHY;
  cauchy.robust_scale = kCauchyScale;
  for (const auto& keyframe : prepared.estimator.input_plan.keyframes) {
    huber.keyframe_times_s.push_back(keyframe.sensor_time);
    cauchy.keyframe_times_s.push_back(keyframe.sensor_time);
  }
  huber.lm.max_iterations = cauchy.lm.max_iterations = prepared.cfg.lm_max_iter;
  huber.lm.relative_tolerance = cauchy.lm.relative_tolerance =
      prepared.cfg.lm_rel_tol;
  huber.lm.absolute_tolerance = cauchy.lm.absolute_tolerance =
      prepared.cfg.lm_abs_tol;

  double anchor_norm_max = 0.0;
  for (const auto& anchor : prepared.cfg.anchors)
    anchor_norm_max = std::max(anchor_norm_max, anchor.pos.norm());
  uifgo::IntermediateSeedQualityOptions quality_options;
  quality_options.position_envelope_m =
      anchor_norm_max + prepared.cfg.max_range;
  quality_options.velocity_envelope_mps = prepared.cfg.v_max +
      prepared.cfg.max_range /
          prepared.cfg.initialization_progression_horizon_s;

  const auto physical_factor_count = prepared.estimator.graph.size();
  std::vector<const void*> physical_factor_addresses;
  for (const auto& factor : prepared.estimator.graph)
    physical_factor_addresses.push_back(factor.get());
  const auto result = uifgo::RunHuberToCauchyWarmStart(
      prepared.estimator.graph, prepared.estimator.initial_values,
      prepared.estimator.factor_metadata, huber, cauchy, quality_options);
  bool physical_graph_unchanged =
      prepared.estimator.graph.size() == physical_factor_count;
  for (size_t i = 0; physical_graph_unchanged &&
                     i < physical_factor_addresses.size(); ++i)
    physical_graph_unchanged =
        prepared.estimator.graph.at(i).get() == physical_factor_addresses[i];

  const auto initial_states = AuditStates(
      prepared.estimator.input_plan, prepared.estimator.initial_values,
      output / "A_initialization_seed_states.csv");
  const auto initial_residuals = WriteReplayResiduals(
      prepared, prepared.estimator.initial_values,
      prepared.estimator.initial_values, "huber",
      output / "A_initialization_seed_uwb.csv");
  const auto& huber_terminal = result.huber_intermediate.final.values;
  if (huber_terminal.empty())
    throw std::runtime_error("HUBER_TERMINAL_VALUES_UNAVAILABLE");
  const auto huber_states = AuditStates(
      prepared.estimator.input_plan, huber_terminal,
      output / "B_huber_intermediate_states.csv");
  const auto huber_residuals = WriteReplayResiduals(
      prepared, prepared.estimator.initial_values, huber_terminal, "huber",
      output / "B_huber_intermediate_uwb.csv");

  const bool has_cauchy_terminal =
      result.cauchy_attempted && !result.cauchy_final.final.values.empty();
  StateAudit cauchy_states;
  ResidualAudit cauchy_residuals;
  if (has_cauchy_terminal) {
    cauchy_states = AuditStates(
        prepared.estimator.input_plan, result.cauchy_final.final.values,
        output / "C_cauchy_final_states.csv");
    cauchy_residuals = WriteReplayResiduals(
        prepared, huber_terminal, result.cauchy_final.final.values, "cauchy",
        output / "C_cauchy_final_uwb.csv");
  }

  const bool trajectory_exported = result.cauchy_final.valid;
  if (trajectory_exported) {
    WriteTrajectory(output, prepared.estimator.keyframes,
                    result.cauchy_final.final_values);
    WriteBiases(output, prepared.estimator.keyframes,
                result.cauchy_final.final_values);
  }
  auto summary = Open(output / "warm_start_summary.json");
  summary << "{\n  \"schema\": \"HUBER_CAUCHY_WARM_START_V1\",\n"
          << "  \"gt_truth_oracle_read\": false,\n"
          << "  \"physical_graph_unchanged\": "
          << (physical_graph_unchanged ? "true" : "false") << ",\n"
          << "  \"physical_factor_count\": " << physical_factor_count << ",\n"
          << "  \"uwb_factor_count\": "
          << prepared.estimator.uwb_factor_indices.size() << ",\n"
          << "  \"huber_scale\": " << kHuberScale << ",\n"
          << "  \"cauchy_scale\": " << kCauchyScale << ",\n"
          << "  \"sensor_sigma_m\": " << prepared.cfg.sigma_range << ",\n"
          << "  \"lm_max_iterations\": " << prepared.cfg.lm_max_iter << ",\n"
          << "  \"A_initialization_seed\": ";
  WarmStageJson(summary, initial_states, initial_residuals,
                result.huber_intermediate.final_graph.error(
                    prepared.estimator.initial_values), nullptr, nullptr);
  summary << ",\n  \"B_huber_intermediate_seed\": ";
  WarmStageJson(summary, huber_states, huber_residuals,
                result.huber_intermediate.final_graph.error(huber_terminal),
                &result.huber_intermediate.final,
                &result.huber_intermediate.solver_certificate);
  const auto& quality = result.intermediate_seed.quality;
  summary << ",\n  \"intermediate_seed_quality\": {"
          << "\"policy\":\"" << JsonEscape(quality.policy_version) << "\""
          << ",\"status\":\"" << (quality.accepted ? "PASS" : "FAIL") << "\""
          << ",\"reason\":\"" << JsonEscape(quality.reason) << "\""
          << ",\"graph_keys_valid\":" << quality.graph_keys_valid
          << ",\"all_xvb_finite\":" << quality.all_xvb_finite
          << ",\"objective_finite\":" << quality.objective_finite
          << ",\"objective_not_worse\":" << quality.objective_not_worse
          << ",\"uwb_residuals_finite\":" << quality.uwb_residuals_finite
          << ",\"physically_plausible\":" << quality.physically_plausible
          << ",\"input_objective\":" << JsonNumberOrNull(quality.input_objective)
          << ",\"terminal_objective\":"
          << JsonNumberOrNull(quality.terminal_objective)
          << ",\"objective_roundoff_allowance\":"
          << JsonNumberOrNull(quality.objective_roundoff_allowance)
          << ",\"max_position_norm_m\":"
          << JsonNumberOrNull(quality.max_position_norm_m)
          << ",\"position_envelope_m\":"
          << JsonNumberOrNull(quality.position_envelope_m)
          << ",\"max_velocity_norm_mps\":"
          << JsonNumberOrNull(quality.max_velocity_norm_mps)
          << ",\"velocity_envelope_mps\":"
          << JsonNumberOrNull(quality.velocity_envelope_mps) << '}';
  summary << ",\n  \"C_cauchy_final_solve\": ";
  if (has_cauchy_terminal) {
    WarmStageJson(summary, cauchy_states, cauchy_residuals,
                  result.cauchy_final.final_graph.error(
                      result.cauchy_final.final.values),
                  &result.cauchy_final.final,
                  &result.cauchy_final.solver_certificate);
  } else {
    summary << "null";
  }
  summary << ",\n  \"cauchy_attempted\": "
          << (result.cauchy_attempted ? "true" : "false")
          << ",\n  \"certified_final_estimate\": "
          << (result.cauchy_final.valid ? "true" : "false")
          << ",\n  \"valid_trajectory_exported\": "
          << (trajectory_exported ? "true" : "false")
          << ",\n  \"gt_evaluation_authorized\": "
          << (trajectory_exported ? "true" : "false") << "\n}\n";

  if (!physical_graph_unchanged)
    throw std::runtime_error("PHYSICAL_GRAPH_MUTATED");
  if (!quality.accepted)
    throw std::runtime_error("INTERMEDIATE_SEED_QUALITY_FAILED:" +
                             quality.reason);
  if (!result.cauchy_final.valid)
    throw std::runtime_error("CAUCHY_FINAL_CERTIFICATE_FAILED:" +
                             result.cauchy_final.reason);
}

}  // namespace

int main(int argc, char** argv) {
  fs::path output;
  std::string mode;
  try {
    if (argc != 4)
      throw std::invalid_argument(
          "usage: uwb_imu_fgo_all_uwb_optimization_diagnosis CONFIG OUTPUT_NEW_DIR initialization|audit|cauchy|huber|gaussian|warm_start");
    const fs::path config = fs::absolute(argv[1]);
    output = fs::absolute(argv[2]);
    mode = argv[3];
    if (fs::exists(output)) throw std::runtime_error("OUTPUT_ALREADY_EXISTS");
    if (!fs::create_directories(output)) throw std::runtime_error("CREATE_OUTPUT_FAILED");
    {
      auto command = Open(output / "command.txt");
      for (int index = 0; index < argc; ++index) {
        if (index) command << ' ';
        command << argv[index];
      }
      command << '\n';
    }
    const auto prepared = Prepare(config);
    if (mode == "initialization") RunAudit(prepared, output, false);
    else if (mode == "audit") RunAudit(prepared, output, true);
    else if (mode == "cauchy" || mode == "huber" || mode == "gaussian")
      RunReplay(prepared, mode, output);
    else if (mode == "warm_start") RunWarmStart(prepared, output);
    else
      throw std::invalid_argument("UNKNOWN_MODE");
    return 0;
  } catch (const std::exception& error) {
    if (!output.empty() && fs::exists(output)) {
      auto failure = Open(output / "failure.json");
      const bool warm_start = mode == "warm_start";
      failure << "{\n  \"schema\": \""
              << (warm_start ? "HUBER_CAUCHY_WARM_START_FAILURE_V1"
                             : "REFACTOR_GATE06RA_INITIALIZATION_FAILURE_V1")
              << "\",\n"
              << "  \"status\": \""
              << (warm_start ? "ALL_UWB_CORRECTNESS_FAIL"
                             : "INITIALIZATION_FAILED") << "\",\n"
              << "  \"gt_truth_oracle_read\": false,\n"
              << "  \"final_estimator_run\": "
              << (warm_start ? "true" : "false") << ",\n"
              << "  \"ie_stages_run\": false,\n"
              << "  \"reason\": \"" << JsonEscape(error.what())
              << "\"\n}\n";
    }
    std::cerr << "GATE06D_FAILED: " << error.what() << '\n';
    return 1;
  }
}
