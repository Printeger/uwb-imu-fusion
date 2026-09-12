#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

#include "uifgo/config.h"
#include "uifgo/graph_builder.h"
#include "uifgo/hash_utils.h"
#include "uifgo/initializer.h"
#include "uifgo/paper_input.h"
#include "uifgo/paper_pose_prior_factor.h"
#include "uifgo/pl_bidirectional_support.h"
#include "uifgo/pl_conditional_raim.h"
#include "uifgo/pl_persistent_cusum.h"
#include "uifgo/t07_scenario_cache.h"

namespace fs = boost::filesystem;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace {

class ShadowFixedLagBackend final
    : public gtsam::IncrementalFixedLagSmoother {
 public:
  ShadowFixedLagBackend(double lag, const gtsam::ISAM2Params& params)
      : gtsam::IncrementalFixedLagSmoother(lag, params) {}
  const gtsam::ISAM2& isam() const { return isam_; }
};

struct Args {
  std::string config;
  std::string output;
  std::string dynamic_role;
  std::string detector_partition = "all";
  std::string forward_calibration;
  std::string backward_calibration;
};

struct StateSnapshot {
  size_t keyframe = 0;
  double time = 0.0;
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias bias;
};

struct Calibration {
  double kappa = 0.5;
  double threshold = 0.0;
  double validation_start = -std::numeric_limits<double>::infinity();
  double validation_end = std::numeric_limits<double>::infinity();
  std::string artifact_hash;
  std::string split_hash;
  std::string clean_input_hash;
};

struct Epoch {
  size_t keyframe = 0;
  double time = 0.0;
  std::vector<std::uint64_t> ids;
  std::vector<int> anchors;
  std::string committed_before_hash;
  size_t committed_before_count = 0;
  uifgo::PlConditionalGroupResult result;
  struct AnchorDiagnostic {
    size_t source_order = 0;
    int tag_id = 0;
    int anchor_id = 0;
    std::uint64_t obs_id = 0;
    double physical_innovation_m = std::numeric_limits<double>::quiet_NaN();
    double measurement_variance_m2 = std::numeric_limits<double>::quiet_NaN();
    double prior_projected_variance_m2 =
        std::numeric_limits<double>::quiet_NaN();
    double innovation_variance_m2 = std::numeric_limits<double>::quiet_NaN();
    uifgo::PlConditionalRowDiagnostic conditional;
  };
  std::vector<AnchorDiagnostic> anchor_diagnostics;
};

struct ImuGapEvent {
  double before_time = 0.0;
  double after_time = 0.0;
  size_t reinitialize_keyframe = std::numeric_limits<size_t>::max();
};

Args Parse(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto take = [&]() {
      if (++i >= argc) throw std::invalid_argument("missing CLI value");
      return std::string(argv[i]);
    };
    if (arg == "--config") args.config = take();
    else if (arg == "--output-dir") args.output = take();
    else if (arg == "--dynamic-role") args.dynamic_role = take();
    else if (arg == "--detector-partition") args.detector_partition = take();
    else if (arg == "--forward-calibration") args.forward_calibration = take();
    else if (arg == "--backward-calibration") args.backward_calibration = take();
    else if (arg == "--help") {
      std::cout << "Usage: pl_conditional_preflight --config FILE --output-dir DIR\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }
  if (args.config.empty() || args.output.empty())
    throw std::invalid_argument("--config and --output-dir are required");
  if (!args.dynamic_role.empty() && args.dynamic_role != "control" &&
      args.dynamic_role != "shadow")
    throw std::invalid_argument("--dynamic-role must be control or shadow");
  if (args.detector_partition != "all" &&
      args.detector_partition != "validation")
    throw std::invalid_argument("--detector-partition must be all or validation");
  if (args.dynamic_role == "shadow" &&
      (args.forward_calibration.empty() || args.backward_calibration.empty()))
    throw std::invalid_argument(
        "shadow role requires forward and backward calibration artifacts");
  return args;
}

std::string ReadAll(const fs::path& path) {
  std::ifstream in(path.string(), std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + path.string());
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

double JsonNumber(const std::string& json, const std::string& name) {
  const std::string token = "\"" + name + "\"";
  auto p = json.find(token);
  if (p == std::string::npos || (p = json.find(':', p + token.size())) ==
                                    std::string::npos)
    throw std::runtime_error("missing JSON number " + name);
  return std::stod(json.substr(p + 1));
}

std::string JsonString(const std::string& json, const std::string& name) {
  const std::string token = "\"" + name + "\"";
  auto p = json.find(token);
  if (p == std::string::npos || (p = json.find(':', p + token.size())) ==
                                    std::string::npos ||
      (p = json.find('"', p + 1)) == std::string::npos)
    throw std::runtime_error("missing JSON string " + name);
  const auto end = json.find('"', p + 1);
  if (end == std::string::npos)
    throw std::runtime_error("malformed JSON string " + name);
  return json.substr(p + 1, end - p - 1);
}

Calibration ReadForwardCalibration(const fs::path& path) {
  const auto json = ReadAll(path);
  Calibration result;
  result.kappa = JsonNumber(json, "kappa");
  result.threshold = JsonNumber(json, "h_locked");
  result.validation_start = JsonNumber(json, "validation_start");
  result.validation_end = JsonNumber(json, "validation_end");
  result.split_hash = JsonString(json, "clean_split_manifest_hash");
  result.clean_input_hash = JsonString(json, "calibration_input_hash");
  result.artifact_hash = "sha256:" + uifgo::Sha256FileHex(path.string());
  return result;
}

Calibration ReadBackwardCalibration(const fs::path& path) {
  const auto json = ReadAll(path);
  Calibration result;
  result.kappa = JsonNumber(json, "kappa_backward");
  result.threshold = JsonNumber(json, "h_backward");
  result.split_hash = JsonString(json, "original_clean_split_manifest_hash");
  result.clean_input_hash = JsonString(json, "original_clean_input_hash");
  result.artifact_hash = "sha256:" + uifgo::Sha256FileHex(path.string());
  return result;
}

std::ofstream Open(const fs::path& path) {
  std::ofstream out(path.string());
  if (!out) throw std::runtime_error("cannot write " + path.string());
  return out;
}

void WriteFiniteOrEmpty(std::ostream* out, double value) {
  if (std::isfinite(value)) *out << value;
}

std::string Escape(const std::string& text) {
  std::ostringstream out;
  for (const char c : text) {
    if (c == '\\' || c == '"') out << '\\';
    if (c == '\n') out << "\\n";
    else out << c;
  }
  return out.str();
}

std::string HashIds(const std::set<std::uint64_t>& ids) {
  std::ostringstream canonical;
  canonical << "PL_CONDITIONAL_COMMITTED_IDS_V1\n";
  for (const auto id : ids) canonical << id << '\n';
  return "sha256:" + uifgo::Sha256Hex(canonical.str());
}

gtsam::GaussianBayesNet CurrentCliqueClosure(
    const gtsam::ISAM2& isam, const gtsam::KeyVector& keys) {
  gtsam::GaussianBayesNet result;
  std::vector<gtsam::ISAM2Clique::shared_ptr> closure;
  std::set<const gtsam::ISAM2Clique*> seen;
  for (const auto key : keys) {
    auto clique = isam.clique(key);
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
  std::stable_sort(closure.begin(), closure.end(),
                   [&](const auto& a, const auto& b) {
                     return depth(a) > depth(b);
                   });
  for (const auto& clique : closure) result.push_back(clique->conditional());
  return result;
}

Eigen::Matrix<double,15,15> CurrentMarginal(const gtsam::ISAM2& isam,
                                             size_t epoch) {
  const gtsam::KeyVector keys{X(epoch), V(epoch), B(epoch)};
  const std::array<int,3> offsets{0,6,9};
  const std::array<int,3> dimensions{6,3,6};
  const auto bayes = CurrentCliqueClosure(isam, keys);
  gtsam::VectorValues zero;
  for (const auto& conditional : bayes) {
    for (auto item = conditional->begin(); item != conditional->end(); ++item) {
      if (!zero.exists(*item))
        zero.insert(*item, Eigen::VectorXd::Zero(conditional->getDim(item)));
    }
  }
  Eigen::Matrix<double,15,15> covariance =
      Eigen::Matrix<double,15,15>::Zero();
  for (size_t column_key = 0; column_key < keys.size(); ++column_key) {
    for (int local = 0; local < dimensions[column_key]; ++local) {
      auto rhs = zero;
      rhs.at(keys[column_key])(local) = 1.0;
      const auto intermediate = bayes.backSubstituteTranspose(rhs);
      const auto column = bayes.backSubstitute(intermediate);
      const int output_column = offsets[column_key] + local;
      for (size_t row_key = 0; row_key < keys.size(); ++row_key) {
        covariance.block(offsets[row_key], output_column,
                         dimensions[row_key], 1) = column.at(keys[row_key]);
      }
    }
  }
  covariance = 0.5 * (covariance + covariance.transpose());
  if (!covariance.allFinite())
    throw std::runtime_error("PL_CONDITIONAL_CURRENT_MARGINAL_INVALID");
  return covariance;
}

size_t FactorEpoch(const gtsam::NonlinearFactor::shared_ptr& factor) {
  size_t epoch = 0;
  bool state_key = false;
  for (const auto key : factor->keys()) {
    const gtsam::Symbol symbol(key);
    if (symbol.chr() == 'x' || symbol.chr() == 'v' || symbol.chr() == 'b') {
      epoch = std::max(epoch, static_cast<size_t>(symbol.index()));
      state_key = true;
    }
  }
  return state_key ? epoch : 0;
}

gtsam::Values NewValuesAt(const gtsam::Values& initial, size_t epoch,
                          bool include_constants) {
  gtsam::Values output;
  for (const auto key : initial.keys()) {
    const gtsam::Symbol symbol(key);
    const bool state = symbol.chr() == 'x' || symbol.chr() == 'v' ||
                       symbol.chr() == 'b';
    if ((state && static_cast<size_t>(symbol.index()) == epoch) ||
        (!state && include_constants)) {
      output.insert(key, initial.at(key));
    }
  }
  return output;
}

gtsam::FixedLagSmoother::KeyTimestampMap Timestamps(
    const gtsam::Values& values, size_t epoch) {
  gtsam::FixedLagSmoother::KeyTimestampMap output;
  for (const auto key : values.keys()) output[key] = static_cast<double>(epoch);
  return output;
}

gtsam::NonlinearFactorGraph ReinitializationPriors(
    size_t epoch, const gtsam::Pose3& pose, const gtsam::Vector3& velocity,
    const gtsam::imuBias::ConstantBias& bias) {
  gtsam::NonlinearFactorGraph graph;
  graph.addPrior(
      X(epoch), pose,
      gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.10, 0.10, 0.10, 0.50, 0.50, 0.50)
              .finished()));
  graph.addPrior(V(epoch), velocity,
                 gtsam::noiseModel::Isotropic::Sigma(3, 0.50));
  graph.addPrior(
      B(epoch), bias,
      gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.10, 0.10, 0.10, 0.01, 0.01, 0.01)
              .finished()));
  return graph;
}

void AddFactors(gtsam::NonlinearFactorGraph* target,
                const std::vector<size_t>& indices,
                const gtsam::NonlinearFactorGraph& source) {
  for (const auto index : indices) target->push_back(source.at(index));
}

void SeedFromShadowPrediction(
    const ShadowFixedLagBackend& backend, size_t epoch,
    const std::vector<size_t>& factor_indices,
    const gtsam::NonlinearFactorGraph& graph, gtsam::Values* values) {
  if (!values || epoch == 0) return;
  gtsam::CombinedImuFactor::shared_ptr imu_factor;
  for (const auto index : factor_indices) {
    const auto candidate = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(
        graph.at(index));
    if (!candidate) continue;
    if (imu_factor)
      throw std::runtime_error("PL_CONDITIONAL_MULTIPLE_IMU_FACTORS_PER_EPOCH");
    imu_factor = candidate;
  }
  if (!imu_factor)
    throw std::runtime_error("PL_CONDITIONAL_IMU_FACTOR_MISSING");
  const auto previous = backend.calculateEstimate();
  const auto pose = previous.at<gtsam::Pose3>(X(epoch - 1));
  const auto velocity = previous.at<gtsam::Vector3>(V(epoch - 1));
  const auto bias =
      previous.at<gtsam::imuBias::ConstantBias>(B(epoch - 1));
  const auto predicted = imu_factor->preintegratedMeasurements().predict(
      gtsam::NavState(pose, velocity), bias);
  values->update(X(epoch), predicted.pose());
  values->update(V(epoch), predicted.velocity());
  values->update(B(epoch), bias);
}

std::string CalibrationIdentity(const uifgo::Config& cfg,
                                const uifgo::InitResult& init) {
  std::ostringstream out;
  out << std::setprecision(17) << "PL_CONDITIONAL_IE_MODEL_V1\n"
      << cfg.sigma_a << ',' << cfg.sigma_g << ',' << cfg.sigma_wa << ','
      << cfg.sigma_wg << ',' << cfg.gravity << '\n'
      << init.gravity_world.transpose() << '\n'
      << cfg.lever_arm_init.transpose() << '\n';
  for (const auto& anchor : cfg.anchors)
    out << anchor.id << ',' << anchor.pos.transpose() << '\n';
  for (const auto& beta : cfg.fixed_beta_by_link)
    out << beta.first << ',' << beta.second << '\n';
  return "sha256:" + uifgo::Sha256Hex(out.str());
}

void WritePartition(const fs::path& path,
                    const uifgo::SupportPartition& partition) {
  auto out = Open(path);
  out << std::setprecision(17)
      << "{\n  \"schema\": \"" << partition.schema << "\",\n"
      << "  \"provider\": \"" << partition.provider << "\",\n"
      << "  \"partition_rule_version\": \""
      << partition.partition_rule_version << "\",\n"
      << "  \"input_plan_hash\": \"" << partition.input_plan_hash << "\",\n"
      << "  \"discovery_context_hash\": \""
      << partition.discovery_context_hash << "\",\n"
      << "  \"discovery_snapshot_hash\": \""
      << partition.discovery_snapshot_hash << "\",\n"
      << "  \"partition_hash\": \"" << partition.partition_hash << "\",\n"
      << "  \"segments\": [";
  for (size_t i = 0; i < partition.segments.size(); ++i) {
    const auto& s = partition.segments[i];
    if (i) out << ',';
    out << "\n    {\"segment_id\":\"" << s.segment_id
        << "\",\"segment_ordinal\":" << s.segment_ordinal
        << ",\"tag_id\":" << s.tag_id << ",\"anchor_id\":"
        << s.anchor_id << ",\"start_time\":" << s.start_time
        << ",\"end_time\":" << s.end_time << ",\"duration\":"
        << s.duration << ",\"observation_count\":" << s.observation_count
        << ",\"obs_ids\":[";
    for (size_t j = 0; j < s.obs_ids.size(); ++j) {
      if (j) out << ',';
      out << s.obs_ids[j];
    }
    out << "]}";
  }
  out << "\n  ]\n}\n";
}

void WriteDynamicScientificLogs(
    const fs::path& output, const std::vector<StateSnapshot>& states,
    const std::vector<Epoch>& epochs,
    const std::vector<uifgo::ObservationRecord>& observations) {
  {
    auto out = Open(output / "runtime_commit_log.csv");
    out << "sequence,keyframe_id,timestamp,uwb_commit_count,update_count\n"
        << std::setprecision(17);
    for (size_t i = 0; i < states.size(); ++i) {
      size_t count = 0;
      for (const auto& epoch : epochs)
        if (epoch.keyframe == states[i].keyframe)
          count = epoch.ids.size();
      out << i << ',' << states[i].keyframe << ',' << states[i].time << ','
          << count << ',' << (count ? 2 : 1) << '\n';
    }
  }
  {
    auto out = Open(output / "measurement_decision_log.csv");
    out << "keyframe_id,timestamp,tag_id,anchor_id,obs_id,accepted,reason\n"
        << std::setprecision(17);
    for (const auto& row : observations) {
      if (!row.valid || !row.planned) continue;
      out << row.keyframe_id << ',' << row.sensor_time << ',' << row.tag_id
          << ',' << row.anchor_id << ',' << row.obs_id
          << ",1,PRODUCTION_RAW_UWB_COMMIT\n";
    }
  }
  {
    auto out = Open(output / "state_commit_log.csv");
    out << "sequence,keyframe_id,timestamp,qx,qy,qz,qw,px,py,pz,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n"
        << std::setprecision(17);
    for (size_t i = 0; i < states.size(); ++i) {
      const auto& state = states[i];
      const auto q = state.pose.rotation().toQuaternion();
      const auto p = state.pose.translation();
      const auto ba = state.bias.accelerometer();
      const auto bg = state.bias.gyroscope();
      out << i << ',' << state.keyframe << ',' << state.time << ',' << q.x()
          << ',' << q.y() << ',' << q.z() << ',' << q.w() << ',' << p.x()
          << ',' << p.y() << ',' << p.z() << ',' << state.velocity.x() << ','
          << state.velocity.y() << ',' << state.velocity.z() << ',' << ba.x()
          << ',' << ba.y() << ',' << ba.z() << ',' << bg.x() << ',' << bg.y()
          << ',' << bg.z() << '\n';
    }
  }
  {
    auto out = Open(output / "trajectory.tum");
    out << std::setprecision(17);
    for (const auto& state : states) {
      const auto q = state.pose.rotation().toQuaternion();
      const auto p = state.pose.translation();
      out << state.time << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' '
          << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
    }
  }
}

void WriteDynamicCusum(
    const fs::path& output, const uifgo::PlCusumResult& forward,
    const uifgo::PlBackwardCusumResult& backward,
    const uifgo::PlBidirectionalSupportResult& final_support) {
  {
    auto out = Open(output / "dynamic_cusum_trace.csv");
    out << "timestamp,group_id,keyframe_id,source_order,tag_id,anchor_id,obs_id,conditional_z,diagnostic_valid,kappa,h_locked,increment,G_before,G_after,reset,reset_reason,excursion_id,excursion_start_time,threshold_crossing,first_alarm_for_excursion,candidate\n"
        << std::setprecision(17);
    for (const auto& row : forward.trace) {
      out << row.input.timestamp << ',' << row.input.group_id << ','
          << row.input.keyframe_id << ',' << row.input.source_order << ','
          << row.input.tag_id << ',' << row.input.anchor_id << ','
          << row.input.obs_id << ',' << row.input.conditional_z << ','
          << row.input.diagnostic_valid << ",0.5,," << row.increment << ','
          << row.g_before << ',' << row.g_after << ',' << row.reset << ','
          << row.reset_reason << ',' << row.excursion_id << ',';
      WriteFiniteOrEmpty(&out, row.excursion_start_time);
      out << ',' << row.threshold_crossing << ','
          << row.first_alarm_for_excursion << ',' << row.candidate << '\n';
    }
  }
  {
    auto out = Open(output / "dynamic_backward_trace.csv");
    out << "timestamp,group_id,keyframe_id,source_order,tag_id,anchor_id,obs_id,conditional_z,diagnostic_valid,reverse_index,increment,B_before,B_after,reset,reset_reason,reverse_excursion_id,reverse_excursion_start_time,threshold_crossing,first_crossing_for_excursion,backward_candidate\n"
        << std::setprecision(17);
    for (const auto& row : backward.trace) {
      out << row.input.timestamp << ',' << row.input.group_id << ','
          << row.input.keyframe_id << ',' << row.input.source_order << ','
          << row.input.tag_id << ',' << row.input.anchor_id << ','
          << row.input.obs_id << ',' << row.input.conditional_z << ','
          << row.input.diagnostic_valid << ',' << row.reverse_index << ','
          << row.increment << ',' << row.b_before << ',' << row.b_after << ','
          << row.reset << ',' << row.reset_reason << ','
          << row.reverse_excursion_id << ',';
      WriteFiniteOrEmpty(&out, row.reverse_excursion_start_time);
      out << ',' << row.threshold_crossing << ','
          << row.first_crossing_for_excursion << ','
          << row.backward_candidate << '\n';
    }
  }
  {
    auto out = Open(output / "dynamic_cusum_candidates.csv");
    out << "timestamp,keyframe_id,tag_id,anchor_id,obs_id,forward_candidate,backward_candidate,final_candidate,segment_id,segment_ordinal\n"
        << std::setprecision(17);
    for (const auto& row : final_support.rows) {
      out << row.input.timestamp << ',' << row.input.keyframe_id << ','
          << row.input.tag_id << ',' << row.input.anchor_id << ','
          << row.input.obs_id << ',' << row.forward_candidate << ','
          << row.backward_candidate << ',' << row.final_candidate << ','
          << row.segment_id << ',';
      if (row.segment_ordinal != std::numeric_limits<size_t>::max())
        out << row.segment_ordinal;
      out << '\n';
    }
  }
  WritePartition(output / "dynamic_cusum_support.json", final_support.support);
}

}  // namespace

int main(int argc, char** argv) {
  fs::path output;
  try {
    const auto args = Parse(argc, argv);
    const fs::path config_path = fs::canonical(fs::absolute(args.config));
    output = fs::absolute(args.output);
    if (fs::exists(output))
      throw std::invalid_argument("refusing to overwrite output directory");
    fs::create_directories(output);
    auto cfg = uifgo::ConfigLoader::Load(config_path.string());
    if (cfg.data_interface != "t07_cache" || cfg.calib_anchor ||
        cfg.calib_lever || cfg.calib_range_bias || cfg.calib_td ||
        cfg.td_init != 0.0)
      throw std::invalid_argument("PL preflight requires fixed t07 paper input");
    fs::path manifest(cfg.t07_cache_manifest);
    if (manifest.is_relative()) manifest = config_path.parent_path() / manifest;
    manifest = fs::canonical(fs::absolute(manifest));
    const auto cache = uifgo::LoadT07ScenarioCache(
        manifest.string(), cfg.t07_cache_start_s, cfg.t07_cache_duration_s);
    const auto& imu = cache.imu;
    const auto& raw_uwb = cache.uwb;
    const auto plan = uifgo::BuildPaperInputPlan(
        raw_uwb, cfg, cache.base_recording_id);
    if (plan.keyframes.size() < 6)
      throw std::runtime_error("PL_CONDITIONAL_INSUFFICIENT_KEYFRAMES");
    const auto mask = uifgo::AllPlannedObservationMask(plan);
    const auto keyframes = uifgo::MaterializePaperKeyframes(plan, mask);
    constexpr size_t kBootstrapLastKeyframe = 4;
    const double bootstrap_cutoff = keyframes[kBootstrapLastKeyframe].t;
    std::vector<uifgo::UwbFrame> bootstrap_keyframes(
        keyframes.begin(), keyframes.begin() + kBootstrapLastKeyframe + 1);
    std::vector<uifgo::ImuSample> bootstrap_imu;
    for (const auto& sample : imu) {
      if (sample.t > bootstrap_cutoff + 1e-12) break;
      bootstrap_imu.push_back(sample);
    }
    if (bootstrap_imu.empty())
      throw std::runtime_error("PL_CONDITIONAL_BOOTSTRAP_IMU_EMPTY");
    const auto init =
        uifgo::Initializer(cfg).Run(bootstrap_imu, bootstrap_keyframes);
    if (!init.ok) throw std::runtime_error("PL_CONDITIONAL_INITIALIZER_FAILED");
    uifgo::GraphBuilder builder(cfg, cfg.paper_imu_covariance_model);
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    std::vector<size_t> uwb_indices;
    builder.Build(keyframes, imu, init, &graph, &initial, &uwb_indices);
    if (uifgo::ReplacePosePriorsForPaperPath(&graph) != 1)
      throw std::runtime_error("PL_CONDITIONAL_POSE_PRIOR_REPLACEMENT_FAILED");

    double max_imu_gap = 0.0;
    std::vector<ImuGapEvent> imu_gap_events;
    for (size_t i = 1; i < imu.size(); ++i) {
      const double gap = imu[i].t - imu[i-1].t;
      if (!std::isfinite(gap) || gap <= 0.0)
        throw std::runtime_error("PL_CONDITIONAL_IMU_TIME_INVALID");
      max_imu_gap = std::max(max_imu_gap, gap);
      if (gap > 0.02 + 1e-12)
        imu_gap_events.push_back({imu[i-1].t, imu[i].t});
    }

    std::unordered_map<std::uint64_t,size_t> factor_by_obs;
    std::set<size_t> uwb_set;
    for (const auto& meta : builder.factor_meta()) {
      if (meta.factor_type != "uwb_range" ||
          !factor_by_obs.emplace(meta.obs_id, meta.factor_index).second)
        throw std::runtime_error("PL_CONDITIONAL_FACTOR_METADATA_INVALID");
      uwb_set.insert(meta.factor_index);
    }
    std::vector<std::vector<size_t>> non_uwb(plan.keyframes.size());
    for (size_t i = 0; i < graph.size(); ++i) {
      if (uwb_set.count(i)) continue;
      const size_t epoch = FactorEpoch(graph.at(i));
      if (epoch >= non_uwb.size())
        throw std::runtime_error("PL_CONDITIONAL_FACTOR_EPOCH_INVALID");
      non_uwb[epoch].push_back(i);
    }
    std::vector<std::vector<const uifgo::ObservationRecord*>> groups(
        plan.keyframes.size());
    for (const auto& row : plan.observations)
      if (row.valid && row.planned) groups[row.keyframe_id].push_back(&row);
    for (auto& group : groups) {
      std::stable_sort(group.begin(), group.end(), [](const auto* a,
                                                       const auto* b) {
        return std::tie(a->source_message_index, a->source_range_index,
                        a->source_observation_index, a->obs_id) <
               std::tie(b->source_message_index, b->source_range_index,
                        b->source_observation_index, b->obs_id);
      });
    }

    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.1;
    params.relinearizeSkip = 200;
    std::unique_ptr<ShadowFixedLagBackend> backend(
        new ShadowFixedLagBackend(200.0, params));
    std::set<std::uint64_t> committed;
    std::vector<Epoch> epochs;
    std::vector<StateSnapshot> dynamic_states;
    std::vector<uifgo::PlConditionalCandidateRecord> candidate_rows;
    std::map<std::uint64_t,size_t> candidate_index;
    for (const auto& row : plan.observations) {
      if (!row.valid || !row.planned) continue;
      uifgo::PlConditionalCandidateRecord candidate;
      candidate.obs_id = row.obs_id;
      candidate.tag_id = row.tag_id;
      candidate.anchor_id = row.anchor_id;
      candidate.keyframe_id = row.keyframe_id;
      candidate.sensor_time = row.sensor_time;
      candidate.planned = true;
      candidate.reason = "NOT_EVALUATED";
      candidate_index[row.obs_id] = candidate_rows.size();
      candidate_rows.push_back(std::move(candidate));
    }

    size_t next_gap_event = 0;
    for (size_t k = 0; k < groups.size(); ++k) {
      gtsam::NonlinearFactorGraph prediction;
      auto new_values = NewValuesAt(initial, k, k == 0);
      if (k > 0 && next_gap_event < imu_gap_events.size() &&
          imu_gap_events[next_gap_event].after_time <=
              plan.keyframes[k].sensor_time + 1e-12 &&
          imu_gap_events[next_gap_event].after_time >
              plan.keyframes[k-1].sensor_time + 1e-12) {
        const auto previous = backend->calculateEstimate();
        const auto pose = previous.at<gtsam::Pose3>(X(k-1));
        const auto velocity = previous.at<gtsam::Vector3>(V(k-1));
        const auto bias = previous.at<gtsam::imuBias::ConstantBias>(B(k-1));
        prediction = ReinitializationPriors(k, pose, velocity, bias);
        new_values.clear();
        new_values.insert(X(k), pose);
        new_values.insert(V(k), velocity);
        new_values.insert(B(k), bias);
        backend.reset(new ShadowFixedLagBackend(200.0, params));
        do {
          imu_gap_events[next_gap_event].reinitialize_keyframe = k;
          ++next_gap_event;
        } while (next_gap_event < imu_gap_events.size() &&
                 imu_gap_events[next_gap_event].after_time <=
                     plan.keyframes[k].sensor_time + 1e-12);
      } else {
        AddFactors(&prediction, non_uwb[k], graph);
        SeedFromShadowPrediction(*backend, k, non_uwb[k], graph, &new_values);
      }
      backend->update(prediction, new_values, Timestamps(new_values, k));
      if (groups[k].empty()) {
        if (!args.dynamic_role.empty()) {
          const auto estimate = backend->calculateEstimate();
          dynamic_states.push_back(
              {k, plan.keyframes[k].sensor_time,
               estimate.at<gtsam::Pose3>(X(k)),
               estimate.at<gtsam::Vector3>(V(k)),
               estimate.at<gtsam::imuBias::ConstantBias>(B(k))});
        }
        continue;
      }
      Epoch epoch;
      epoch.keyframe = k;
      epoch.time = plan.keyframes[k].sensor_time;
      epoch.committed_before_hash = HashIds(committed);
      epoch.committed_before_count = committed.size();
      std::set<int> anchors;
      std::vector<size_t> factor_indices;
      for (const auto* row : groups[k]) {
        if (!anchors.insert(row->anchor_id).second)
          throw std::runtime_error("PL_CONDITIONAL_DUPLICATE_ANCHOR_IN_GROUP");
        if (!factor_by_obs.count(row->obs_id))
          throw std::runtime_error("PL_CONDITIONAL_OBS_FACTOR_MISSING");
        if (committed.count(row->obs_id))
          throw std::runtime_error(
              "CONDITIONAL_PRIOR_CONTAMINATED_BY_CURRENT_UWB");
        epoch.ids.push_back(row->obs_id);
        epoch.anchors.push_back(row->anchor_id);
        factor_indices.push_back(factor_by_obs.at(row->obs_id));
      }
      if (k <= 4) {
        epoch.result.outcome =
            uifgo::PlConditionalGroupOutcome::BOOTSTRAP_HISTORY;
        epoch.result.committed_rows.resize(groups[k].size());
        for (size_t i = 0; i < groups[k].size(); ++i) {
          epoch.result.committed_rows[i] = i;
          candidate_rows[candidate_index.at(groups[k][i]->obs_id)].reason =
              "BOOTSTRAP_HISTORY";
        }
      } else {
        const auto estimate = backend->calculateEstimate();
        uifgo::PlConditionalInput input;
        input.prior_covariance = CurrentMarginal(backend->isam(), k);
        const size_t n = groups[k].size();
        input.physical_jacobian = Eigen::MatrixXd::Zero(n, 15);
        input.physical_covariance = Eigen::MatrixXd::Zero(n, n);
        input.physical_innovation = Eigen::VectorXd::Zero(n);
        for (size_t i = 0; i < n; ++i) {
          const auto& factor = graph.at(factor_indices[i]);
          const auto noise_factor = boost::dynamic_pointer_cast<
              gtsam::NoiseModelFactor>(factor);
          if (!noise_factor)
            throw std::runtime_error("PL_CONDITIONAL_FACTOR_MODEL_INVALID");
          const auto residual = noise_factor->unwhitenedError(estimate);
          if (residual.size() != 1)
            throw std::runtime_error("PL_CONDITIONAL_FACTOR_JACOBIAN_INVALID");
          if (std::find(factor->keys().begin(), factor->keys().end(), X(k)) ==
              factor->keys().end())
            throw std::runtime_error("PL_CONDITIONAL_POSE_JACOBIAN_INVALID");
          const auto pose = estimate.at<gtsam::Pose3>(X(k));
          const auto residual_at_pose = [&](const gtsam::Pose3& trial_pose) {
            auto trial = estimate;
            trial.update(X(k), trial_pose);
            return noise_factor->unwhitenedError(trial);
          };
          const auto pose_jacobian =
              gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                  residual_at_pose, pose, 1e-6);
          if (pose_jacobian.rows() != 1 || pose_jacobian.cols() != 6 ||
              !pose_jacobian.allFinite())
            throw std::runtime_error("PL_CONDITIONAL_POSE_JACOBIAN_INVALID");
          input.physical_jacobian.block<1,6>(i,0) = pose_jacobian;
          if (!noise_factor ||
              noise_factor->noiseModel()->sigmas().size() != 1)
            throw std::runtime_error("PL_CONDITIONAL_FACTOR_MODEL_INVALID");
          const double sigma = noise_factor->noiseModel()->sigmas()[0];
          if (std::abs(sigma - groups[k][i]->nominal_sigma) >
              1e-12 + 1e-10 * std::max(std::abs(sigma),
                                        std::abs(groups[k][i]->nominal_sigma)))
            throw std::runtime_error("PL_CONDITIONAL_SIGMA_IDENTITY_MISMATCH");
          input.physical_covariance(i,i) = sigma * sigma;
          input.physical_innovation[i] = -residual[0];
        }
        const Eigen::MatrixXd physical_innovation_covariance =
            input.physical_jacobian * input.prior_covariance *
                input.physical_jacobian.transpose() +
            input.physical_covariance;
        const auto row_diagnostics = uifgo::EvaluatePlConditionalRows(
            input.physical_innovation, physical_innovation_covariance);
        if (row_diagnostics.rows.size() != n)
          throw std::runtime_error(
              "PL_CONDITIONAL_ROW_DIAGNOSTIC_DIMENSION_INVALID:" +
              row_diagnostics.status);
        epoch.anchor_diagnostics.reserve(n);
        for (size_t i = 0; i < n; ++i) {
          Epoch::AnchorDiagnostic diagnostic;
          diagnostic.source_order = i;
          diagnostic.tag_id = groups[k][i]->tag_id;
          diagnostic.anchor_id = groups[k][i]->anchor_id;
          diagnostic.obs_id = groups[k][i]->obs_id;
          diagnostic.physical_innovation_m = input.physical_innovation[i];
          diagnostic.measurement_variance_m2 =
              input.physical_covariance(i, i);
          diagnostic.prior_projected_variance_m2 =
              (input.physical_jacobian.row(i) * input.prior_covariance *
               input.physical_jacobian.row(i).transpose())(0, 0);
          diagnostic.innovation_variance_m2 =
              physical_innovation_covariance(i, i);
          diagnostic.conditional = row_diagnostics.rows[i];
          epoch.anchor_diagnostics.push_back(std::move(diagnostic));
        }
        epoch.result = uifgo::EvaluatePlConditionalLoao(
            input, epoch.anchors, epoch.ids);
        for (size_t i = 0; i < groups[k].size(); ++i) {
          auto& row = candidate_rows[candidate_index.at(groups[k][i]->obs_id)];
          row.reason = uifgo::PlConditionalGroupOutcomeName(epoch.result.outcome);
          if (epoch.result.positive_excess_candidate &&
              groups[k][i]->obs_id == epoch.result.isolated_obs_id)
            row.candidate = true;
        }
      }
      // Dynamic CONTROL and SHADOW both use the production raw-UWB commit
      // policy. The historical group statistic remains diagnostic and cannot
      // reject, reweight, or otherwise affect a dynamic replay measurement.
      if (!args.dynamic_role.empty()) {
        epoch.result.committed_rows.resize(groups[k].size());
        for (size_t i = 0; i < groups[k].size(); ++i)
          epoch.result.committed_rows[i] = i;
      }
      gtsam::NonlinearFactorGraph commit_graph;
      for (const auto row : epoch.result.committed_rows) {
        commit_graph.push_back(graph.at(factor_indices.at(row)));
        committed.insert(epoch.ids.at(row));
      }
      if (!commit_graph.empty()) backend->update(commit_graph);
      if (!args.dynamic_role.empty()) {
        const auto estimate = backend->calculateEstimate();
        dynamic_states.push_back(
            {k, plan.keyframes[k].sensor_time,
             estimate.at<gtsam::Pose3>(X(k)),
             estimate.at<gtsam::Vector3>(V(k)),
             estimate.at<gtsam::imuBias::ConstantBias>(B(k))});
      }
      epochs.push_back(std::move(epoch));
    }

    uifgo::PlConditionalSupportContext context;
    context.input_plan_hash = plan.plan_sha256;
    context.source_hash = cache.cache_id;
    context.config_hash = "sha256:" + uifgo::Sha256FileHex(config_path.string());
    context.calibration_hash = CalibrationIdentity(cfg, init);
    context.solver_config_hash =
        "sha256:" + uifgo::Sha256Hex("PL_ISAM2_RELIN_0.1_SKIP_200_FIXED_LAG_200");
    context.detector_identity_hash = uifgo::PlConditionalDetectorIdentity(
        context, 4, 200, 0.02, 0.1, 200);
    const auto support = uifgo::BuildPlConditionalSupport(
        &candidate_rows, 1.0, 2, 0.01, context);

    std::unique_ptr<uifgo::PlCusumResult> dynamic_forward;
    std::unique_ptr<uifgo::PlBackwardCusumResult> dynamic_backward;
    std::unique_ptr<uifgo::PlBidirectionalSupportResult> dynamic_final;
    Calibration forward_calibration;
    Calibration backward_calibration;
    std::vector<uifgo::PlCusumInputRow> dynamic_rows;
    if (args.dynamic_role == "shadow") {
      forward_calibration = ReadForwardCalibration(args.forward_calibration);
      backward_calibration = ReadBackwardCalibration(args.backward_calibration);
      if (forward_calibration.kappa != 0.5 ||
          std::abs(forward_calibration.threshold -
                   7.0234689587858723) > 1e-15)
        throw std::runtime_error("DYNAMIC_FORWARD_CALIBRATION_NOT_LOCKED");
      if (backward_calibration.kappa != 0.5 ||
          backward_calibration.split_hash != forward_calibration.split_hash ||
          backward_calibration.clean_input_hash !=
              forward_calibration.clean_input_hash)
        throw std::runtime_error("DYNAMIC_BACKWARD_CALIBRATION_NOT_LOCKED");
      for (const auto& epoch : epochs) {
        if (epoch.keyframe <= 4) continue;
        if (args.detector_partition == "validation" &&
            (epoch.time < forward_calibration.validation_start ||
             epoch.time > forward_calibration.validation_end))
          continue;
        for (const auto& row : epoch.anchor_diagnostics) {
          uifgo::PlCusumInputRow input;
          input.timestamp = epoch.time;
          input.group_id = "pl-group-" + std::to_string(epoch.keyframe);
          input.keyframe_id = epoch.keyframe;
          input.source_order = row.source_order;
          input.tag_id = row.tag_id;
          input.anchor_id = row.anchor_id;
          input.obs_id = row.obs_id;
          input.conditional_z = row.conditional.conditional_z;
          input.diagnostic_valid = row.conditional.numerically_valid;
          dynamic_rows.push_back(std::move(input));
        }
      }
      uifgo::PlCusumOptions forward_options;
      forward_options.kappa = forward_calibration.kappa;
      forward_options.threshold = forward_calibration.threshold;
      forward_options.gap_threshold_s = 1.0;
      forward_options.calibration_hash = forward_calibration.artifact_hash;
      forward_options.split_manifest_hash = forward_calibration.split_hash;
      dynamic_forward.reset(new uifgo::PlCusumResult(
          uifgo::EvaluatePlPersistentCusum(dynamic_rows, forward_options)));
      uifgo::PlBackwardCusumOptions backward_options;
      backward_options.kappa = backward_calibration.kappa;
      backward_options.threshold = backward_calibration.threshold;
      backward_options.gap_threshold_s = 1.0;
      backward_options.original_clean_split_manifest_hash =
          backward_calibration.split_hash;
      backward_options.original_clean_input_hash =
          backward_calibration.clean_input_hash;
      backward_options.calibration_hash = backward_calibration.artifact_hash;
      dynamic_backward.reset(new uifgo::PlBackwardCusumResult(
          uifgo::EvaluatePlBackwardCusum(dynamic_rows, backward_options)));
      if (!dynamic_forward->valid || !dynamic_backward->valid)
        throw std::runtime_error("DYNAMIC_CUSUM_EVALUATION_FAILED");
      std::vector<uifgo::PlBidirectionalMembershipRow> forward_membership;
      std::vector<uifgo::PlBidirectionalMembershipRow> backward_membership;
      for (const auto& row : dynamic_forward->trace) {
        uifgo::PlBidirectionalMembershipRow item;
        item.input = row.input;
        item.forward_candidate = row.candidate;
        forward_membership.push_back(std::move(item));
      }
      for (const auto& row : dynamic_backward->trace) {
        uifgo::PlBidirectionalMembershipRow item;
        item.input = row.input;
        item.backward_candidate = row.backward_candidate;
        backward_membership.push_back(std::move(item));
      }
      dynamic_final.reset(new uifgo::PlBidirectionalSupportResult(
          uifgo::IntersectPlCusumSupport(
              forward_membership, backward_membership,
              dynamic_forward->detector_identity,
              dynamic_backward->closure_identity, 1.0)));
      if (!dynamic_final->valid)
        throw std::runtime_error("DYNAMIC_SUPPORT_INTERSECTION_FAILED:" +
                                 dynamic_final->status);
    }

    {
      auto out = Open(output / "pl_conditional_epochs.csv");
      out << "keyframe_id,sensor_time,group_size,obs_ids,anchors,committed_before_count,committed_before_hash,omnibus_status,statistic,threshold,dof,alarm,outcome,isolated_anchor_id,isolated_obs_id,isolated_innovation_m,positive_candidate,committed_count,prior_degradation_event\n";
      out << std::setprecision(17);
      for (const auto& e : epochs) {
        std::ostringstream ids, anchors_out;
        for (size_t i = 0; i < e.ids.size(); ++i) {
          if (i) { ids << ';'; anchors_out << ';'; }
          ids << e.ids[i]; anchors_out << e.anchors[i];
        }
        const auto& d = e.result.omnibus;
        out << e.keyframe << ',' << e.time << ',' << e.ids.size() << ','
            << ids.str() << ',' << anchors_out.str() << ','
            << e.committed_before_count << ',' << e.committed_before_hash << ','
            << (e.keyframe <= 4 ? "BOOTSTRAP_HISTORY" : d.status) << ',';
        if (e.keyframe <= 4) out << ",,0,0,";
        else out << d.statistic << ',' << d.threshold << ',' << d.dof << ','
                 << (!d.passed) << ',';
        out << uifgo::PlConditionalGroupOutcomeName(e.result.outcome) << ','
            << e.result.isolated_anchor_id << ',' << e.result.isolated_obs_id
            << ',';
        if (std::isfinite(e.result.isolated_innovation_m))
          out << e.result.isolated_innovation_m;
        out << ',' << e.result.positive_excess_candidate << ','
            << e.result.committed_rows.size() << ','
            << e.result.prior_degradation_event << '\n';
      }
    }
    {
      auto out = Open(output / "pl_conditional_isolation.csv");
      out << "keyframe_id,excluded_anchor_id,retained_obs_ids,status,statistic,threshold,dof,passed\n";
      out << std::setprecision(17);
      for (const auto& e : epochs) for (const auto& h : e.result.hypotheses) {
        std::ostringstream ids;
        for (size_t i = 0; i < h.retained_obs_ids.size(); ++i) {
          if (i) ids << ';';
          ids << h.retained_obs_ids[i];
        }
        out << e.keyframe << ',' << h.excluded_anchor_id << ',' << ids.str()
            << ',' << h.decision.status << ',' << h.decision.statistic << ','
            << h.decision.threshold << ',' << h.decision.dof << ','
            << h.decision.passed << '\n';
      }
    }
    {
      auto out = Open(output / "pl_per_anchor_innovations.csv");
      out << "timestamp,group_id,keyframe_id,source_order,tag_id,anchor_id,obs_id,nu_m,R_mm,prior_projected_variance_m,S_mm,marginal_z,diagnostic_valid,invalid_reason\n";
      out << std::setprecision(17);
      for (const auto& e : epochs) {
        if (e.keyframe <= 4) continue;
        for (const auto& row : e.anchor_diagnostics) {
          out << e.time << ",pl-group-" << e.keyframe << ',' << e.keyframe
              << ',' << row.source_order << ',' << row.tag_id << ','
              << row.anchor_id << ',' << row.obs_id << ',';
          WriteFiniteOrEmpty(&out, row.physical_innovation_m);
          out << ',';
          WriteFiniteOrEmpty(&out, row.measurement_variance_m2);
          out << ',';
          WriteFiniteOrEmpty(&out, row.prior_projected_variance_m2);
          out << ',';
          WriteFiniteOrEmpty(&out, row.innovation_variance_m2);
          out << ',';
          WriteFiniteOrEmpty(&out, row.conditional.marginal_z);
          out << ',' << row.conditional.numerically_valid << ','
              << row.conditional.invalid_reason << '\n';
        }
      }
    }
    {
      auto out = Open(output / "pl_conditional_innovations.csv");
      out << "timestamp,group_id,keyframe_id,source_order,tag_id,anchor_id,obs_id,conditional_nu_m,conditional_variance_m2,conditional_z,additive_group_contribution,conditional_quadratic_increment,group_statistic,dof,diagnostic_valid,invalid_reason\n";
      out << std::setprecision(17);
      for (const auto& e : epochs) {
        if (e.keyframe <= 4) continue;
        for (const auto& row : e.anchor_diagnostics) {
          out << e.time << ",pl-group-" << e.keyframe << ',' << e.keyframe
              << ',' << row.source_order << ',' << row.tag_id << ','
              << row.anchor_id << ',' << row.obs_id << ',';
          WriteFiniteOrEmpty(&out, row.conditional.conditional_innovation);
          out << ',';
          WriteFiniteOrEmpty(&out, row.conditional.conditional_variance);
          out << ',';
          WriteFiniteOrEmpty(&out, row.conditional.conditional_z);
          out << ',';
          WriteFiniteOrEmpty(
              &out, row.conditional.additive_quadratic_contribution);
          out << ',';
          WriteFiniteOrEmpty(
              &out, row.conditional.conditional_quadratic_increment);
          out << ',';
          WriteFiniteOrEmpty(&out, e.result.omnibus.statistic);
          out << ',' << e.result.omnibus.dof << ','
              << row.conditional.numerically_valid << ','
              << row.conditional.invalid_reason << '\n';
        }
      }
    }
    {
      auto out = Open(output / "pl_conditional_candidates.csv");
      out << "obs_id,keyframe_id,sensor_time,tag_id,anchor_id,planned,candidate,reason,segment_id,segment_ordinal\n";
      out << std::setprecision(17);
      for (const auto& row : candidate_rows) {
        out << row.obs_id << ',' << row.keyframe_id << ',' << row.sensor_time
            << ',' << row.tag_id << ',' << row.anchor_id << ',' << row.planned
            << ',' << row.candidate << ',' << row.reason << ',' << row.segment_id
            << ',';
        if (row.segment_ordinal != std::numeric_limits<size_t>::max())
          out << row.segment_ordinal;
        out << '\n';
      }
    }
    WritePartition(output / "pl_conditional_support.json", support);
    WritePartition(output / "support_partition.json", support);
    {
      auto out = Open(output / "pl_conditional_prior_audit.json");
      out << "{\n  \"schema\":\"uifgo_pl_conditional_prior_audit_v1\",\n"
          << "  \"bootstrap_last_keyframe\":4,\n"
          << "  \"bootstrap_cutoff_time\":" << std::setprecision(17)
          << bootstrap_cutoff << ",\n"
          << "  \"bootstrap_imu_count\":" << bootstrap_imu.size() << ",\n"
          << "  \"bootstrap_uwb_keyframe_count\":"
          << bootstrap_keyframes.size() << ",\n"
          << "  \"bootstrap_future_input_count\":0,\n"
          << "  \"detect_before_commit_first_keyframe\":5,\n"
          << "  \"current_group_intersection_count\":0,\n"
          << "  \"committed_uwb_count\":" << committed.size() << ",\n"
          << "  \"committed_uwb_ids_hash\":\"" << HashIds(committed)
          << "\",\n  \"max_observed_imu_gap_s\":" << std::setprecision(17)
          << max_imu_gap << ",\n  \"imu_gap_limit_s\":0.02,\n"
          << "  \"imu_gap_policy\":\"PL_CONTROLLED_REINITIALIZE_V1\",\n"
          << "  \"imu_gap_event_count\":" << imu_gap_events.size() << ",\n"
          << "  \"imu_gap_events\":[";
      for (size_t i = 0; i < imu_gap_events.size(); ++i) {
        if (i) out << ',';
        const auto& event = imu_gap_events[i];
        out << "{\"before_time\":" << event.before_time
            << ",\"after_time\":" << event.after_time
            << ",\"reinitialize_keyframe\":";
        if (event.reinitialize_keyframe == std::numeric_limits<size_t>::max())
          out << "null";
        else
          out << event.reinitialize_keyframe;
        out << '}';
      }
      out << "],\n"
          << "  \"fixed_lag_epochs\":200,\n  \"pass\":true\n}\n";
    }
    size_t alarms = 0, unique = 0, degradations = 0;
    for (const auto& e : epochs) {
      alarms += e.keyframe > 4 && !e.result.omnibus.passed;
      unique += e.result.outcome ==
                    uifgo::PlConditionalGroupOutcome::UNIQUE_ISOLATION_POSITIVE ||
                e.result.outcome ==
                    uifgo::PlConditionalGroupOutcome::UNIQUE_ISOLATION_NONPOSITIVE;
      degradations += e.result.prior_degradation_event;
    }
    {
      auto out = Open(output / "pl_conditional_status.json");
      out << "{\n  \"schema\":\"uifgo_pl_conditional_preflight_status_v1\",\n"
          << "  \"status\":\"SUCCESS\",\n  \"provider\":\""
          << uifgo::kPlConditionalPreflightProvider << "\",\n"
          << "  \"provider_version\":\""
          << uifgo::kPlConditionalIdentityVersion << "\",\n"
          << "  \"pl_source_commit\":\""
          << uifgo::kPlConditionalSourceCommit << "\",\n"
          << "  \"detector_identity_hash\":\""
          << context.detector_identity_hash << "\",\n"
          << "  \"input_plan_hash\":\"" << plan.plan_sha256 << "\",\n"
          << "  \"keyframe_count\":" << plan.keyframes.size() << ",\n"
          << "  \"planned_count\":" << candidate_rows.size() << ",\n"
          << "  \"detected_group_count\":"
          << std::count_if(epochs.begin(), epochs.end(), [](const Epoch& e) {
               return e.keyframe > 4;
             }) << ",\n"
          << "  \"alarm_group_count\":" << alarms << ",\n"
          << "  \"unique_isolation_count\":" << unique << ",\n"
          << "  \"prior_degradation_event_count\":" << degradations << ",\n"
          << "  \"retained_segment_count\":" << support.segments.size() << ",\n"
          << "  \"partition_hash\":\"" << support.partition_hash << "\",\n"
          << "  \"gt_read\":false,\n  \"oracle_read\":false\n}\n";
    }
    if (!args.dynamic_role.empty()) {
      WriteDynamicScientificLogs(output, dynamic_states, epochs,
                                 plan.observations);
      fs::copy_file(output / "pl_conditional_innovations.csv",
                    output / "dynamic_conditional_trace.csv");
      if (args.dynamic_role == "shadow")
        WriteDynamicCusum(output, *dynamic_forward, *dynamic_backward,
                          *dynamic_final);
      auto out = Open(output / "dynamic_status.json");
      out << std::setprecision(17)
          << "{\n  \"schema\":\"uifgo_pl_bidirectional_dynamic_shadow_v1\",\n"
          << "  \"role\":\"" << args.dynamic_role << "\",\n"
          << "  \"commit_policy\":\"PRODUCTION_RAW_UWB_COMMIT_ALL_V1\",\n"
          << "  \"observer_feedback_to_estimator\":false,\n"
          << "  \"group_statistic_used_for_candidate_decision\":false,\n"
          << "  \"group_statistic_used_for_commit_decision\":false,\n"
          << "  \"truth_access_count\":0,\n"
          << "  \"keyframe_count\":" << dynamic_states.size() << ",\n"
          << "  \"committed_measurement_count\":" << committed.size();
      if (args.dynamic_role == "shadow") {
        out << ",\n  \"detector_partition\":\""
            << args.detector_partition << "\",\n"
            << "  \"forward_detector_identity\":\""
            << dynamic_forward->detector_identity << "\",\n"
            << "  \"backward_closure_identity\":\""
            << dynamic_backward->closure_identity << "\",\n"
            << "  \"final_support_identity\":\""
            << dynamic_final->support_identity << "\",\n"
            << "  \"detector_row_count\":" << dynamic_rows.size() << ",\n"
            << "  \"forward_alarm_count\":"
            << dynamic_forward->alarm_count << ",\n"
            << "  \"forward_alarm_link_count\":"
            << dynamic_forward->alarm_link_count << ",\n"
            << "  \"forward_segment_count\":"
            << dynamic_forward->segment_count << ",\n"
            << "  \"forward_max_G\":" << dynamic_forward->max_g << ",\n"
            << "  \"backward_alarm_count\":"
            << dynamic_backward->alarm_count << ",\n"
            << "  \"backward_alarm_link_count\":"
            << dynamic_backward->alarm_link_count << ",\n"
            << "  \"backward_segment_count\":"
            << dynamic_backward->segment_count << ",\n"
            << "  \"backward_max_B\":" << dynamic_backward->max_b << ",\n"
            << "  \"final_segment_count\":"
            << dynamic_final->support.segments.size() << "\n";
      } else {
        out << "\n";
      }
      out << "}\n";
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    if (!output.empty() && fs::exists(output)) {
      auto out = Open(output / "pl_conditional_status.json");
      out << "{\"status\":\"FAILED\",\"reason\":\""
          << Escape(error.what()) << "\",\"gt_read\":false}\n";
    }
    return 1;
  }
}
