#define main a18_embedded_main
#include "a18_stage1.cpp"
#undef main

#include "a19_r01_status.h"

#include "uifgo/nlos_refit.h"
#include "uifgo/nlos_scoring.h"
#include "uifgo/nlos_inference.h"
#include "uifgo/nlos_inference_io.h"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include <csignal>
#include <thread>

namespace {

const char* kA19Schema = uifgo::kA19DevelopmentStage1Schema;
const char* kR03Policy =
    "PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1";
const char* kCertifiedPolicy = "PAPER_CERTIFIED_PAIR_REDUCTION_V1";

std::string Strategy19(const std::string& config, const std::string& validation_context_sha256) {
  std::string material = kR03Policy +
      std::string("A19_SHARED_DEVELOPMENT_IDENTITY_CMAKE_ABI_MATCHED_MPFR_333_INEXACT_HANDOFF_FINAL_EMPTY_CERTIFIED_V1") +
      Sha256FileHex(config) + validation_context_sha256 + Sha256FileHex("/proc/self/exe");
  for (const auto* path : {
           "/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so",
           "/usr/local/lib/libgtsam.so.4.2.0",
           "/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2",
           "/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0"})
    material += Sha256FileHex(path);
  return "a19-policy-sha256:" + Sha256Hex(material);
}

uifgo::RefitOptions RefitOptionsFrom(const uifgo::Config& cfg) {
  uifgo::RefitOptions o;
  o.boundary_epsilon_m = cfg.refit_boundary_epsilon_m;
  o.relative_objective_tolerance = cfg.refit_relative_objective_tolerance;
  o.scaled_step_tolerance = cfg.refit_scaled_step_tolerance;
  o.projected_gradient_tolerance = cfg.refit_projected_gradient_tolerance;
  o.navigation_stationarity_tolerance_objective =
      cfg.refit_navigation_stationarity_tolerance_objective;
  o.gradient_roundoff_safety_factor =
      cfg.refit_gradient_roundoff_safety_factor;
  o.max_refit_iterations = static_cast<size_t>(cfg.max_refit_iterations);
  o.lm_max_iterations = cfg.lm_max_iter;
  o.lm_relative_tolerance = cfg.lm_rel_tol;
  o.lm_absolute_tolerance = cfg.lm_abs_tol;
  o.pose_rotation_scale_rad = cfg.refit_pose_rotation_scale_rad;
  o.pose_translation_scale_m = cfg.refit_pose_translation_scale_m;
  o.velocity_scale_mps = cfg.refit_velocity_scale_mps;
  o.accel_bias_scale_mps2 = cfg.refit_accel_bias_scale_mps2;
  o.gyro_bias_scale_radps = cfg.refit_gyro_bias_scale_radps;
  o.segment_amplitude_scale_m = cfg.refit_segment_amplitude_scale_m;
  return o;
}

std::vector<uifgo::DevelopmentRangeConstant> ConvertRanges(
    const std::vector<uifgo::DevelopmentRefitRangeConstant>& input) {
  std::vector<uifgo::DevelopmentRangeConstant> output;
  output.reserve(input.size());
  for (const auto& r : input)
    output.push_back({r.factor_index, r.pose_key, r.anchor, r.lever,
                      r.measurement, r.sigma, r.conditional_beta, r.obs_id});
  return output;
}

struct Totals {
  size_t calls = 0, trials = 0, accepted = 0, rejected = 0, unresolved = 0;
  double certificate_seconds = 0.0;
  void Add(const uifgo::CheckedLmResult& r) {
    calls += r.convergence.iterate_call_count;
    trials += r.convergence.lambda_trial_count;
    accepted += r.convergence.accepted_update_count;
    rejected += r.convergence.rejected_lambda_trial_count;
    const size_t classified = r.convergence.accepted_update_count +
                              r.convergence.rejected_lambda_trial_count;
    unresolved += r.convergence.lambda_trial_count >= classified
                      ? r.convergence.lambda_trial_count - classified
                      : 0;
    certificate_seconds += r.convergence.added_diagnostics_seconds;
  }
};

struct FinalPolicySpec {
  const char* name;
  uifgo::FinalGatePolicy policy;
};

const std::array<FinalPolicySpec, 5> kFinalPolicies{{
    {"suppress_all", uifgo::FinalGatePolicy::SUPPRESS_ALL},
    {"structured_debias", uifgo::FinalGatePolicy::STRUCTURED_DEBIAS},
    {"fit_only", uifgo::FinalGatePolicy::FIT_ONLY},
    {"s_fit", uifgo::FinalGatePolicy::S_FIT},
    {"full_gate", uifgo::FinalGatePolicy::FULL_GATE},
}};

uifgo::GateThresholds R03Thresholds() {
  uifgo::GateThresholds thresholds;
  thresholds.tau_eta = 0.10;
  thresholds.tau_s_m = 0.10;
  thresholds.tau_gamma = 1.0;
  thresholds.parameter_provenance = uifgo::kT08DevelopmentGateLabel;
  return thresholds;
}

void WriteAtomicText(const std::string& path, const std::string& text) {
  const std::string temporary = path + ".tmp";
  {
    std::ofstream stream(temporary, std::ios::out | std::ios::trunc);
    if (!stream) throw std::runtime_error("ATOMIC_STATUS_OPEN_FAILED");
    stream << text;
    stream.flush();
    if (!stream) throw std::runtime_error("ATOMIC_STATUS_WRITE_FAILED");
  }
  if (::rename(temporary.c_str(), path.c_str()) != 0)
    throw std::runtime_error("ATOMIC_STATUS_RENAME_FAILED");
}

int WaitFinalChild(pid_t child, int timeout_seconds, bool* timed_out) {
  const auto started = std::chrono::steady_clock::now();
  *timed_out = false;
  for (;;) {
    int status = 0;
    const pid_t waited = ::waitpid(child, &status, WNOHANG);
    if (waited == child) {
      if (WIFEXITED(status)) return WEXITSTATUS(status);
      return 128 + (WIFSIGNALED(status) ? WTERMSIG(status) : 0);
    }
    if (waited < 0) throw std::runtime_error("FINAL_WAITPID_FAILED");
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                     started).count() >= timeout_seconds) {
      *timed_out = true;
      ::kill(-child, SIGKILL);
      ::kill(child, SIGKILL);
      if (::waitpid(child, &status, 0) != child)
        throw std::runtime_error("FINAL_TIMEOUT_REAP_FAILED");
      return 137;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void WriteDenseCsv(const std::string& path, const Eigen::MatrixXd& matrix) {
  auto file = output(path);
  file << "row,column,value\n";
  for (Eigen::Index row = 0; row < matrix.rows(); ++row)
    for (Eigen::Index column = 0; column < matrix.cols(); ++column)
      file << row << ',' << column << ',' << matrix(row, column) << '\n';
}

void WriteSparseCsv(const std::string& path,
                    const Eigen::SparseMatrix<double>& matrix) {
  auto file = output(path);
  file << "rows," << matrix.rows() << "\ncolumns," << matrix.cols()
       << "\nrow,column,value\n";
  for (int outer = 0; outer < matrix.outerSize(); ++outer)
    for (Eigen::SparseMatrix<double>::InnerIterator item(matrix, outer); item;
         ++item)
      file << item.row() << ',' << item.col() << ',' << item.value() << '\n';
}

#include "t11_diagnostics.h"

int RunSuppressCertifiedEngineering(const std::string& output_root, bool lcb = false, bool full_offset = false) {
  if (!fs::create_directory(output_root))
    throw std::runtime_error("OUTPUT_EXISTS");
  fs::create_directories(output_root + "/normal/recovery");
  fs::create_directories(output_root + "/normal/fallback");
  fs::create_directories(output_root + "/normal/final");
  fs::create_directories(output_root + "/forced_fallback/recovery");
  fs::create_directories(output_root + "/forced_fallback/fallback");
  fs::create_directories(output_root + "/forced_fallback/final");

  Config cfg;
  cfg.calib_lever = false;
  cfg.calib_anchor = false;
  cfg.calib_range_bias = false;
  cfg.calib_td = false;
  cfg.lever_arm_init = gtsam::Point3(0.1, -0.03, 0.02);
  cfg.anchors = {
      {1, gtsam::Point3(-4, -3, 1), 0.1},
      {2, gtsam::Point3(5, -3, 2), 0.1},
      {3, gtsam::Point3(-3, 5, 3), 0.1},
      {4, gtsam::Point3(4, 4, -1), 0.1},
  };
  PaperInputPlan plan;
  plan.plan_sha256 = "sha256:" + Sha256Hex("r07-engineering-plan");
  plan.keyframes = {{0, 0, 0.0, 7}, {1, 1, 1.0, 7}};
  SupportPartition support;
  support.schema = "t06_automatic_support_v1";
  support.provider = "automatic_engineering_fixture_no_oracle_no_gt";
  support.partition_hash = "sha256:" + Sha256Hex("r07-engineering-support");
  support.solver_config_hash =
      "a19-policy-sha256:" + Sha256Hex("r07-engineering-certified");
  for (size_t ordinal = 0; ordinal < 2; ++ordinal) {
    SupportSegment segment;
    segment.segment_id = "r07-segment-" + std::to_string(ordinal);
    segment.segment_ordinal = ordinal;
    segment.tag_id = 7;
    segment.anchor_id = static_cast<int>(ordinal + 1);
    segment.start_time = 0.0;
    segment.end_time = 1.0;
    segment.duration = 1.0;
    support.segments.push_back(segment);
  }

  const std::vector<gtsam::Pose3> truth = {
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.08, -0.04, 0.15),
                   gtsam::Point3(1.0, 1.5, 0.6)),
      gtsam::Pose3(gtsam::Rot3::RzRyRx(0.10, -0.03, 0.20),
                   gtsam::Point3(1.4, 1.8, 0.7)),
  };
  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)
          .finished());
  auto vector_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
  auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  const gtsam::Vector3 zero_velocity(0.0, 0.0, 0.0);
  NonlinearFactorGraph graph;
  Values initial;
  std::vector<FactorMeta> metadata;
  std::uint64_t obs_id = 70000;
  for (size_t k = 0; k < truth.size(); ++k) {
    graph.add(gtsam::NonlinearFactor::shared_ptr(
        new uifgo::PaperPosePriorFactor(
            gtsam::symbol_shorthand::X(k), truth[k], pose_noise)));
    graph.addPrior(gtsam::symbol_shorthand::V(k), zero_velocity,
                   vector_noise);
    graph.addPrior(gtsam::symbol_shorthand::B(k),
                   gtsam::imuBias::ConstantBias(), bias_noise);
    initial.insert(
        gtsam::symbol_shorthand::X(k),
        truth[k].retract((gtsam::Vector(6)
                              << 0.01, -0.01, 0.01, 0.03, -0.02, 0.02)
                             .finished()));
    initial.insert(gtsam::symbol_shorthand::V(k),
                   gtsam::Vector3(0.01, -0.01, 0.01));
    initial.insert(gtsam::symbol_shorthand::B(k),
                   gtsam::imuBias::ConstantBias());
    for (const auto& anchor : cfg.anchors) {
      const double geometric =
          (truth[k].transformFrom(cfg.lever_arm_init) - anchor.pos).norm();
      const bool candidate = anchor.id <= 2;
      const double amplitude =
          candidate ? (anchor.id == 1 ? 0.4 : 0.3) : 0.0;
      const double raw = geometric + amplitude;
      const auto factor = MakeUwbFactor(
          gtsam::symbol_shorthand::X(k), 0, 0, 0, anchor.pos,
          cfg.lever_arm_init, raw, 0.05,
          false, false, false, 0.0);
      const size_t factor_index = graph.size();
      graph.add(factor);
      FactorMeta meta;
      meta.factor_index = factor_index;
      meta.factor_type = "uwb_range";
      meta.obs_id = obs_id;
      metadata.push_back(meta);
      ObservationRecord observation;
      observation.obs_id = obs_id;
      observation.sensor_time = static_cast<double>(k);
      observation.tag_id = 7;
      observation.anchor_id = anchor.id;
      observation.raw_range = raw;
      observation.valid = true;
      observation.planned = true;
      observation.keyframe_id = k;
      observation.nominal_sigma = 0.05;
      plan.observations.push_back(observation);
      if (candidate)
        support.segments.at(anchor.id - 1).obs_ids.push_back(obs_id);
      ++obs_id;
    }
  }
  for (auto& segment : support.segments)
    segment.observation_count = segment.obs_ids.size();

  RefitOptions refit_options;
  refit_options.max_refit_iterations = 50;
  refit_options.lm_relative_tolerance = 1e-10;
  refit_options.lm_absolute_tolerance = 1e-12;
  if (lcb) {
    auto ranges_file=output(output_root+"/input_ranges.csv");
    ranges_file << "obs_id,keyframe_id,raw_z_m,sigma_m,beta_m,ax,ay,az,lx,ly,lz\n";
    for(const auto& row:plan.observations) {
      const auto a=std::find_if(cfg.anchors.begin(),cfg.anchors.end(),[&](const auto& x){return x.id==row.anchor_id;});
      ranges_file << row.obs_id << ',' << row.keyframe_id << ',' << row.raw_range << ',' << row.nominal_sigma << ",0,"
                  << a->pos.x() << ',' << a->pos.y() << ',' << a->pos.z() << ','
                  << cfg.lever_arm_init.x() << ',' << cfg.lever_arm_init.y() << ',' << cfg.lever_arm_init.z() << '\n';
    }
  }
  const auto stage2 = SegmentRefitter(refit_options).Run(
      graph, initial, metadata, plan, cfg, support);
  if (!stage2.converged())
    throw std::runtime_error("R07_ENGINEERING_STAGE2_FAILED:" +
                             stage2.reason);
  const auto scores =
      ScoreRefitRecoverability(stage2, support, plan, cfg);
  if (scores.empty())
    throw std::runtime_error("R07_ENGINEERING_SCORE_EMPTY");

  InferenceIdentityContext identity;
  identity.input_sha256 = "sha256:" + Sha256Hex("r07-engineering-input");
  identity.config_sha256 = "sha256:" + Sha256Hex("r07-engineering-config");
  identity.input_plan_sha256 = plan.plan_sha256;
  identity.support_partition_sha256 = support.partition_hash;
  identity.calibration_sha256 =
      "sha256:" + Sha256Hex("r07-engineering-calibration");
  identity.solver_config_sha256 = support.solver_config_hash;
  GateThresholds gate;
  gate.tau_eta = 0.10;
  gate.tau_s_m = 0.10;
  gate.tau_gamma = 1.0;
  gate.parameter_provenance = kT08DevelopmentGateLabel;

  auto make_request = [&](const std::string& root, size_t* callback_count) {
    DevelopmentStage2Request request;
    request.policy = kCertifiedPolicy;
    request.role = "development";
    request.implementation_identity = support.solver_config_hash;
    request.allow_inexact_handoff = false;
    request.conditional_navigation =
        [&, root, callback_count](
            size_t outer, const NonlinearFactorGraph& conditional_graph,
            const Values& values, const CheckedLmOptions& options,
            const std::vector<DevelopmentRefitRangeConstant>& constants) {
          ++*callback_count;
          if ((!lcb && conditional_graph.size() != 10) || values.size() != 6 ||
              (!lcb && constants.size() != 4) ||
              options.policy != ConditionalLmPolicy::
                  GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2)
            throw std::runtime_error("R07_ENGINEERING_NO_C_SHAPE_OR_POLICY");
          for (const auto& item : constants) {
            if ((!lcb && (item.candidate || item.segment_amplitude != 0.0)) ||
                item.conditional_beta != item.fixed_beta + item.segment_amplitude ||
                item.factor_index >= conditional_graph.size() ||
                conditional_graph.at(item.factor_index)->keys() !=
                    gtsam::KeyVector{item.pose_key})
              throw std::runtime_error(
                  "R07_ENGINEERING_RANGE_METADATA_MISMATCH");
          }
          return runCertified(
              outer, conditional_graph, values, options,
              ConvertRanges(constants),
              root + "/outer" + std::to_string(outer), false);
        };
    return request;
  };

  size_t normal_recovery_callbacks = 0;
  size_t normal_fallback_callbacks = 0;
  auto normal_recovery = make_request(
      output_root + "/normal/recovery", &normal_recovery_callbacks);
  auto normal_fallback = make_request(
      output_root + "/normal/fallback", &normal_fallback_callbacks);
  const auto normal = FinalInferenceEngine(
      gate, refit_options, {}, {}, lcb ? (full_offset ? FinalGatePolicy::LCB_FIXED_FULL : FinalGatePolicy::LCB_PARTIAL) : FinalGatePolicy::SUPPRESS_ALL, nullptr,
      &normal_recovery, &normal_fallback)
      .Run(graph, metadata, stage2, support, scores, plan, cfg, identity);
  if (!normal.valid_estimate() || normal_recovery_callbacks == 0 ||
      normal_fallback_callbacks != 0 || normal.fallback.attempted ||
      normal.factor_audit.accepted_candidate_count != (lcb ? 4u : 0u) ||
      normal.factor_audit.suppressed_candidate_count != (lcb ? 0u : 4u) ||
      normal.factor_audit.noncandidate_reference_count != 4 ||
      normal.final_graph.size() != (lcb ? 14u : 10u) || normal.final_values.size() != 6)
    throw std::runtime_error("R07_ENGINEERING_NORMAL_RECOVERY_FAILED:"+normal.reason+":"+normal.fallback.recovery_failure_reason);

  size_t forced_recovery_callbacks = 0;
  size_t forced_fallback_callbacks = 0;
  auto forced_recovery = make_request(
      output_root + "/forced_fallback/recovery",
      &forced_recovery_callbacks);
  auto forced_fallback = make_request(
      output_root + "/forced_fallback/fallback",
      &forced_fallback_callbacks);
  InferenceTestHooks hooks;
  hooks.force_recovery_failure = true;
  const auto fallback = FinalInferenceEngine(
      gate, refit_options, {}, hooks, lcb ? (full_offset ? FinalGatePolicy::LCB_FIXED_FULL : FinalGatePolicy::LCB_PARTIAL) : FinalGatePolicy::SUPPRESS_ALL, nullptr,
      &forced_recovery, &forced_fallback)
      .Run(graph, metadata, stage2, support, scores, plan, cfg, identity);
  if (!fallback.valid_estimate() ||
      fallback.status != InferenceStatus::FALLBACK_OK ||
      forced_recovery_callbacks == 0 || forced_fallback_callbacks == 0 ||
      fallback.fallback.attempt_count != 1 ||
      fallback.fallback.status != "SUCCESS")
    throw std::runtime_error("R07_ENGINEERING_CERTIFIED_FALLBACK_FAILED");

  size_t invalid_callbacks = 0;
  auto invalid = make_request(output_root + "/invalid", &invalid_callbacks);
  invalid.implementation_identity =
      "a19-policy-sha256:wrong-r07-engineering-identity";
  const auto rejected = SegmentRefitter(refit_options)
      .RunDevelopmentFrozenCandidatePolicy(
          graph, stage2.values, metadata, plan, cfg, support, {}, &invalid);
  if (rejected.status != SegmentRefitStatus::INVALID_INPUT ||
      invalid_callbacks != 0)
    throw std::runtime_error("R07_ENGINEERING_IDENTITY_FAIL_CLOSED_FAILED");

  WriteInferenceArtifacts(output_root + "/normal/final", normal);
  WriteInferenceArtifacts(output_root + "/forced_fallback/final", fallback);
  auto summary = output(output_root + "/summary.json");
  summary
      << "{\"schema\":\"IE0911_OR_R07_CERTIFIED_ENGINEERING_V1\","
         "\"status\":\"PASS\",\"actual_certified_callback\":true,"
         "\"policy\":\""
      << kCertifiedPolicy
      << "\",\"inexact_handoff\":false,\"raw_graph_factors\":"
      << graph.size() << ",\"stage2_values\":" << stage2.values.size()
      << ",\"candidate_excluded_graph_factors\":"
      << normal.final_graph.size()
      << ",\"candidate_excluded_values\":" << normal.final_values.size()
      << ",\"candidate_count\":4,\"remaining_raw_range_count\":4,"
         "\"normal_recovery_callbacks\":"
      << normal_recovery_callbacks
      << ",\"normal_fallback_callbacks\":" << normal_fallback_callbacks
      << ",\"forced_recovery_callbacks\":" << forced_recovery_callbacks
      << ",\"forced_fallback_callbacks\":" << forced_fallback_callbacks
      << ",\"fallback_attempt_count\":" << fallback.fallback.attempt_count
      << ",\"identity_failure_callbacks\":" << invalid_callbacks
      << ",\"normal_inference_id\":\"" << normal.inference_id
      << "\",\"fallback_inference_id\":\"" << fallback.inference_id
      << "\"}\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string out;
  bool output_created = false;
  bool stage1_completed = false;
  bool stage1_callback_active = false;
  bool stage2_started = false;
  bool stage2_completed = false;
  bool stage2_callback_active = false;
  Totals stage1_totals, stage2_totals;
  try {
    if (argc == 3 && std::string(argv[1]) == "--ie0911-full-engineering") {
      compact_diagnostic_output = true;
      return RunSuppressCertifiedEngineering(fs::absolute(argv[2]).string(), true, true);
    }
    if (argc == 3 && std::string(argv[1]) == "--ie0911-fixed-engineering") {
      compact_diagnostic_output = true;
      return RunSuppressCertifiedEngineering(fs::absolute(argv[2]).string(), true);
    }
    if (argc == 5 && std::string(argv[1]) == "--policy" &&
        std::string(argv[2]) == kR03Policy &&
        (std::string(argv[3]) == "development-suppress-policy-engineering" ||
         std::string(argv[3]) == "development-suppress-policy-engineering-compact")) {
      compact_diagnostic_output = std::string(argv[3]) == "development-suppress-policy-engineering-compact";
      return RunSuppressCertifiedEngineering(
          fs::absolute(argv[4]).string());
    }
    if (argc == 5 && std::string(argv[1]) == "--policy" &&
        std::string(argv[2]) == kR03Policy &&
        std::string(argv[3]) == "development-audit-only") {
      out = fs::absolute(argv[4]);
      if (!fs::create_directory(out)) throw std::runtime_error("OUTPUT_EXISTS");
      output_created = true;
      const auto write_case = [&](const std::string& name, bool converged,
                                  const InexactHandoffAudit& audit,
                                  bool evaluated, const std::string& reason) {
        fs::create_directory(out + "/" + name);
        writeInexactHandoffAudit(out + "/" + name + "/inexact_handoff.json",
                                 converged, audit, evaluated, reason);
        auto block = output(out + "/" + name + "/block_status.json");
        block << "{\"converged\":" << (converged ? "true" : "false")
              << ",\"handoff_qualified\":"
              << (audit.qualified ? "true" : "false")
              << ",\"handoff_count\":" << audit.handoff_count << "}\n";
      };
      InexactHandoffAudit normal;
      normal.enabled = true;
      normal.status = "ENABLED_NOT_TRIGGERED";
      normal.trigger_reason = "ENABLED_NOT_TRIGGERED";
      normal.scaled_step_tolerance = 1e-6;
      write_case("normal_converged", true, normal, false,
                 "NOT_EVALUATED_INNER_CONVERGED");
      InexactHandoffAudit qualified = normal;
      qualified.qualified = true;
      qualified.status = "INNER_NUMERICAL_STALL_INEXACT";
      qualified.trigger_reason = "ENGINEERING_QUALIFIED";
      qualified.accepted_update_count = 1;
      qualified.last_accepted_generic_convergence = true;
      qualified.last_accepted_stationarity_valid = true;
      qualified.last_accepted_scaled_navigation_step = 1e-9;
      qualified.last_accepted_scaled_navigation_gradient = 1e-8;
      qualified.last_accepted_gradient_roundoff_allowance = 1e-12;
      qualified.objective_increase_allowance = 1e-10;
      qualified.last_accepted_values_identity = "r03values-sha256:fixture";
      qualified.handoff_count = 1;
      InexactHandoffTrialAudit trial;
      trial.trial_index = 1;
      trial.certificate_valid = true;
      trial.certificate_status = "REJECT";
      trial.predicted_lo = 1e-9;
      trial.predicted_hi = 2e-9;
      trial.actual_decrease_lo = -2e-9;
      trial.actual_decrease_hi = -1e-9;
      trial.scaled_navigation_step = 1e-10;
      qualified.last_lambda_search_trials.push_back(trial);
      write_case("qualified_handoff", false, qualified, true,
                 "EVALUATED_QUALIFIED");
      InexactHandoffAudit rejected = normal;
      rejected.status = "GUARD_REJECTED";
      rejected.trigger_reason = "ONE_OR_MORE_INEXACT_HANDOFF_GUARDS_FAILED";
      InexactHandoffTrialAudit invalid_trial;
      invalid_trial.trial_index = 1;
      invalid_trial.certificate_status = "LINEAR_SOLVE_FAILED";
      rejected.last_lambda_search_trials.push_back(invalid_trial);
      write_case("guard_rejected", false, rejected, true,
                 "EVALUATED_NOT_QUALIFIED");
      write_case("not_evaluated", false, normal, false,
                 "NOT_EVALUATED_NO_NUMERICAL_STALL");
      return 0;
    }
    if (argc != 8 || std::string(argv[1]) != "--policy" ||
        std::string(argv[2]) != kR03Policy)
      throw std::runtime_error("A19_DEFAULT_OFF_OR_INVALID_POLICY");
    const std::string mode = argv[3], config = argv[4], initialization = argv[5];
    const std::string validation_context_path = fs::absolute(argv[6]).string();
    out = fs::absolute(argv[7]);
    const bool prepare_only = mode == "validation-prepare-only";
    if (!prepare_only && mode != "validation-stage1-stage2-score")
      throw std::runtime_error("A19_SCOPE_REJECTED");
    if (initialization != "ORIGINAL_RAW_NO_CHECKPOINT")
      throw std::runtime_error("A19_WARM_START_FORBIDDEN");
    if (!fs::is_regular_file(validation_context_path))
      throw std::runtime_error("VALIDATION_CONTEXT_MISSING");
    if (!fs::create_directory(out)) throw std::runtime_error("OUTPUT_EXISTS");
    output_created = true;
    if (!prepare_only) ::alarm(900);

    const auto validation_node = YAML::LoadFile(validation_context_path);
    const bool t11_mode = bool(validation_node["t11"]);
    if (t11_mode) {
      compact_diagnostic_output = validation_node["t11"]["compact_output"].as<bool>();
      t11::CheckPrefix(config, validation_node, out);
    }
    const std::set<std::string> allowed_scenarios = {
        "los", "step1", "step2", "step3", "ramp_gentle", "ramp_steep"};
    if (validation_node["schema"].as<std::string>() !=
            "uifgo_t10_validation_admission_v1" ||
        validation_node["role"].as<std::string>() != "validation" ||
        !validation_node["truth_paths_forbidden"].as<bool>() ||
        validation_node["split_sha256"].as<std::string>().size() != 71 ||
        validation_node["admission_sha256"].as<std::string>().size() != 71 ||
        validation_node["budget_id"].as<std::string>() !=
            "T10_A19_R08_BTOTAL14400_SCHEDULED10800_RESERVE3600" ||
        validation_node["reservation_key"].as<std::string>().find(
            "a10_val_turn_") != 0 ||
        validation_node["ancestry_group"].as<std::string>() !=
            validation_node["base_trajectory_id"].as<std::string>() ||
        !allowed_scenarios.count(
            validation_node["scenario_id"].as<std::string>()))
      throw std::runtime_error("VALIDATION_CONTEXT_REJECTED");
    const std::string reservation_key =
        validation_node["reservation_key"].as<std::string>();
    const int validation_seed = validation_node["seed"].as<int>();
    if ((reservation_key == "a10_val_turn_01_seed20101" &&
         validation_seed != 20101) ||
        (reservation_key == "a10_val_turn_02_seed20102" &&
         validation_seed != 20102) ||
        (reservation_key != "a10_val_turn_01_seed20101" &&
         reservation_key != "a10_val_turn_02_seed20102"))
      throw std::runtime_error("VALIDATION_RESERVATION_SEED_REJECTED");
    const std::string validation_context_sha256 =
        "sha256:" + Sha256FileHex(validation_context_path);
    const std::string scenario_id = validation_node["scenario_id"].as<std::string>();
    const std::string sid = Strategy19(config, validation_context_sha256);
    {
      auto f = output(out + "/diagnostic_manifest.json");
      f << "{\"schema\":\"uifgo_t10_validation_admission_v1\""
        << ",\"role\":\"validation\",\"validation_context_sha256\":\""
        << validation_context_sha256 << "\",\"consumable\":false,"
           "\"policy\":\"" << kR03Policy << "\",\"strategy_identity\":\""
        << sid << "\",\"implementation\":\"A19_R08_VALIDATION_CONTEXT_BOUND_CERTIFIED_V1\"}\n";
    }
    bool cache_rejected = false;
    try { ReadStage2CacheManifest(out + "/diagnostic_manifest.json"); }
    catch (const std::exception&) { cache_rejected = true; }
    if (!cache_rejected) throw std::runtime_error("CACHE_READER_MUST_REJECT");

    A17Graph fixture(config, false);
    const auto cache = LoadT07ScenarioCache(
        fixture.cfg.t07_cache_manifest, fixture.cfg.t07_cache_start_s,
        fixture.cfg.t07_cache_duration_s);
    if (validation_node["cache_id"].as<std::string>() != cache.cache_id ||
        validation_node["recording_id"].as<std::string>() !=
            cache.base_recording_id ||
        validation_node["raw_input_manifest_sha256"].as<std::string>() !=
            "sha256:" + Sha256FileHex(fixture.cfg.t07_cache_manifest) ||
        validation_node["config_sha256"].as<std::string>() !=
            "sha256:" + Sha256FileHex(config) ||
        validation_node["runner_sha256"].as<std::string>() !=
            "sha256:" + Sha256FileHex("/proc/self/exe") ||
        validation_node["core_sha256"].as<std::string>() !=
            "sha256:" + Sha256FileHex(
                "/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/libuwb_imu_fgo.so") ||
        validation_node["gtsam_sha256"].as<std::string>() !=
            "sha256:" + Sha256FileHex("/usr/local/lib/libgtsam.so.4.2.0") ||
        validation_node["mpfr_sha256"].as<std::string>() !=
            "sha256:" + Sha256FileHex(
                "/usr/lib/x86_64-linux-gnu/libmpfr.so.6.0.2") ||
        validation_node["gmp_sha256"].as<std::string>() !=
            "sha256:" + Sha256FileHex(
                "/usr/lib/x86_64-linux-gnu/libgmp.so.10.4.0"))
      throw std::runtime_error("VALIDATION_RAW_CONFIG_RUNNER_BINDING_REJECTED");
    const auto plan = BuildPaperInputPlan(
        cache.uwb, fixture.cfg, cache.base_recording_id);
    std::vector<FactorMeta> metadata;
    const auto initial_ranges = ranges(fixture);
    size_t range_index = 0;
    for (const auto& observation : plan.observations) {
      if (!observation.planned) continue;
      FactorMeta meta;
      meta.factor_index = initial_ranges.at(range_index++).factor_index;
      meta.factor_type = "uwb_range";
      meta.obs_id = observation.obs_id;
      metadata.push_back(meta);
    }
    if (range_index != initial_ranges.size())
      throw std::runtime_error("RAW_FACTOR_METADATA_IDENTITY");

    if (t11_mode) t11::AuditGraph(fixture, plan, cache, out);
    DiscoveryContext context;
    context.input_plan_hash = plan.plan_sha256;
    context.source_hash = sid;
    context.config_hash = fixture.ctx.config_sha256;
    context.calibration_hash = "sha256:" +
        Sha256Hex(fixture.ctx.config_sha256 + "fixed-synthetic-assumptions");
    if (t11_mode) context.calibration_hash = t11::Calibration(config, out);
    context.solver_config_hash = sid;
    context.validation_context_sha256 = validation_context_sha256;
    fixture.ctx.validation_context_sha256 = validation_context_sha256;

    size_t stage1_callback_invocations = 0;
    DevelopmentStage1Request stage1_request;
    stage1_request.policy = kA19DevelopmentStage1Policy;
    stage1_request.implementation_identity = sid;
    stage1_request.role = kValidationRole;
    stage1_request.output_schema = kA19ValidationStage1Schema;
    stage1_request.output_provider = kValidationNonconsumableProvider;
    stage1_request.validation_context_sha256 = validation_context_sha256;
    stage1_request.conditional_navigation = [&](
        size_t outer, const NonlinearFactorGraph& graph, const Values& values,
        const CheckedLmOptions& lm_options,
        const std::vector<DevelopmentRangeConstant>& range_constants) {
      ++stage1_callback_invocations;
      stage1_callback_active = true;
      try {
        auto result = runCertified(
            outer, graph, values, lm_options, range_constants,
            out + "/stage1/outer" + std::to_string(outer));
        stage1_totals.Add(result);
        stage1_callback_active = false;
        return result;
      } catch (...) {
        throw;
      }
    };
    const auto discovery_options = options(fixture.cfg);
    const auto contract_check = ValidateDevelopmentStage1Request(
        stage1_request, context, discovery_options, false);
    if (prepare_only) {
      const auto identity = ComputeInferenceContentIdentity(
          fixture.graph, fixture.initial, fixture.ctx);
      std::ostringstream prepared;
      prepared << "{\"schema\":\"A19_R05_PREPARE_CONTRACT_CHECK_V1\","
                  "\"status\":\"PREPARED_AND_CONTRACT_CHECKED_BEFORE_STAGE1\","
                  "\"raw_load\":true,\"plan\":true,\"materialize\":true,"
                  "\"initialize\":true,\"graph_values\":true,"
                  "\"content_identity\":true,\"contract_accepted\":"
               << (contract_check.accepted ? "true" : "false")
               << ",\"contract_reason\":\"" << contract_check.reason
               << "\",\"request_schema\":\"" << stage1_request.output_schema
               << "\",\"request_provider\":\""
               << stage1_request.output_provider << "\",\"request_policy\":\""
               << stage1_request.policy << "\",\"request_role\":\""
               << stage1_request.role << "\",\"request_identity\":\""
               << stage1_request.implementation_identity
               << "\",\"context_solver_identity\":\""
               << context.solver_config_hash << "\",\"factor_count\":"
               << fixture.graph.size() << ",\"values_count\":"
               << fixture.initial.size() << ",\"graph_identity\":\""
               << identity.graph_linearization_sha256
               << "\",\"values_identity\":\"" << identity.values_sha256
               << "\",\"stage1_calls\":0,\"conditional_optimizer_calls\":"
               << stage1_callback_invocations
               << ",\"consumable\":false}\n";
      WriteAtomicText(out + "/prepare_status.json", prepared.str());
      if (!contract_check.accepted)
        throw std::runtime_error(contract_check.reason);
      return 0;
    }
    if (!contract_check.accepted)
      throw std::runtime_error(contract_check.reason);
    fs::create_directory(out + "/stage1");
    auto stage1_trace = output(out + "/stage1/outers.csv");
    stage1_trace << "outer,objective_before,objective_after,relative_change,"
                    "navigation_step,bias_step,combined_step,KKT,gradient,roundoff,"
                    "objective_ok,step_ok,KKT_ok,navigation_ok,calls,trials,lambda\n";
    stage1_request.outer_observer = [&](const DiscoveryIteration& t) {
      stage1_trace << t.outer_iteration << ',' << t.objective_before << ','
        << t.objective_after << ',' << t.relative_objective_change << ','
        << t.max_navigation_scaled_step << ',' << t.max_bias_scaled_step << ','
        << t.combined_scaled_step << ',' << t.max_chain_kkt_objective_per_m << ','
        << t.navigation_gradient_objective << ','
        << t.navigation_roundoff_allowance_objective << ',' << t.objective_ok
        << ',' << t.step_ok << ',' << t.chain_optimality_ok << ','
        << t.navigation_stationarity_ok << ','
        << t.conditional_lm_convergence.iterate_call_count << ','
        << t.conditional_lm_convergence.lambda_trial_count << ','
        << t.conditional_lm_lambda << '\n';
      stage1_trace.flush();
    };

    const auto stage1_started = std::chrono::steady_clock::now();
    auto discovery = AutomaticSupportProvider(discovery_options)
        .RunDevelopmentStage1(fixture.graph, fixture.initial, metadata, plan,
                              fixture.cfg, context, nullptr, &stage1_request);
    const double stage1_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - stage1_started).count();
    {
      auto f = output(out + "/stage1/status.json");
      f << "{\"status\":\"" << DiscoveryStatusName(discovery.status)
        << "\",\"reason\":\"" << discovery.reason
        << "\",\"converged\":" << (discovery.converged() ? "true" : "false")
        << ",\"outers\":" << discovery.iterations.size()
        << ",\"calls\":" << stage1_totals.calls
        << ",\"trials\":" << stage1_totals.trials
        << ",\"accepted\":" << stage1_totals.accepted
        << ",\"rejected\":" << stage1_totals.rejected
        << ",\"unresolved\":" << stage1_totals.unresolved
        << ",\"certificate_seconds\":" << stage1_totals.certificate_seconds
        << ",\"elapsed_seconds\":" << stage1_seconds << "}\n";
    }
    if (!discovery.converged()) {
      a19r01::Counts s1{stage1_totals.calls, stage1_totals.trials,
                        stage1_totals.accepted, stage1_totals.rejected,
                        stage1_totals.unresolved, true};
      a19r01::WriteFailure(out, discovery.reason, "FAILED", "NOT_RUN",
                           s1, a19r01::Counts{});
      return 1;
    }

    // R3 checkpoint: atomically persist this run's actual immutable discovery
    // snapshot and partition before any Stage-2 object can be constructed.
    if (t11_mode) {
      auto exact = output(out + "/stage1/snapshot_exact.csv");
      exact << "obs_id,bias_m,weight,sensor_time,chain_id,active_run_id\n";
      for (const auto& row : discovery.snapshot)
        exact << row.obs_id << ',' << row.bias_m << ',' << row.weight << ',' << row.sensor_time << ',' << row.chain_id << ',' << row.active_run_id << '\n';
      auto nav = output(out + "/stage1/navigation_exact.csv");
      nav << "phase,index,name,row,column,hex,bits\n"; values(nav, "STAGE1", discovery.navigation_values);
    }
    const size_t candidate_observations = a19r01::WriteStage1Checkpoint(
        out, discovery.snapshot, discovery.partition);
    stage1_completed = true;

    fs::create_directory(out + "/stage2");
    stage2_started = true;
    DevelopmentStage2Request stage2_request;
    stage2_request.policy = kR03Policy;
    stage2_request.role = "validation";
    stage2_request.validation_context_sha256 = validation_context_sha256;
    stage2_request.implementation_identity = sid;
    stage2_request.allow_inexact_handoff = true;
    stage2_request.conditional_navigation = [&](
        size_t outer, const NonlinearFactorGraph& graph, const Values& values,
        const CheckedLmOptions& lm_options,
        const std::vector<DevelopmentRefitRangeConstant>& range_constants) {
      stage2_callback_active = true;
      try {
        auto result = runCertified(outer, graph, values, lm_options,
            ConvertRanges(range_constants),
            out + "/stage2/outer" + std::to_string(outer), true);
        stage2_totals.Add(result);
        stage2_callback_active = false;
        return result;
      } catch (...) {
        throw;
      }
    };
    auto stage2_trace = output(out + "/stage2/outers.csv");
    stage2_trace << "outer,objective_before,objective_after,relative_change,"
                    "scaled_step,KKT,gradient,roundoff,objective_ok,step_ok,"
                    "KKT_ok,navigation_ok,inner_status,inexact_handoff,"
                    "calls,trials,lambda\n";
    stage2_request.outer_observer = [&](const RefitIteration& t) {
      stage2_trace << t.outer_iteration << ',' << t.objective_before << ','
        << t.objective_after << ',' << t.relative_objective_change << ','
        << t.scaled_state_step << ',' << t.max_kkt_violation << ','
        << t.max_scaled_navigation_gradient_objective << ','
        << t.navigation_gradient_roundoff_allowance_objective << ','
        << t.objective_ok << ',' << t.step_ok << ',' << t.kkt_ok << ','
        << t.navigation_stationarity_ok << ','
        << t.conditional_inner_status << ','
        << t.conditional_inexact_handoff << ','
        << t.conditional_lm_iterations
        << ',' << t.conditional_lm_inner_iterations << ','
        << t.conditional_lm_lambda << '\n';
      stage2_trace.flush();
    };
    DevelopmentStage2Request stage2_no_c_request = stage2_request;
    stage2_no_c_request.policy = kCertifiedPolicy;
    stage2_no_c_request.allow_inexact_handoff = false;
    stage2_no_c_request.conditional_navigation = [&](
        size_t outer, const NonlinearFactorGraph& graph, const Values& values,
        const CheckedLmOptions& lm_options,
        const std::vector<DevelopmentRefitRangeConstant>& range_constants) {
      stage2_callback_active = true;
      auto result = runCertified(
          outer, graph, values, lm_options, ConvertRanges(range_constants),
          out + "/stage2/outer" + std::to_string(outer), false);
      stage2_totals.Add(result);
      stage2_callback_active = false;
      return result;
    };
    stage2_no_c_request.outer_observer = stage2_request.outer_observer;
    const DevelopmentStage2Request* stage2_request_for_partition =
        discovery.partition.segments.empty() ? &stage2_no_c_request
                                             : &stage2_request;
    const auto stage2_started = std::chrono::steady_clock::now();
    auto refit = SegmentRefitter(RefitOptionsFrom(fixture.cfg))
        .RunDevelopmentStage2(fixture.graph, discovery.navigation_values,
                              metadata, plan, fixture.cfg,
                              discovery.partition,
                              stage2_request_for_partition);
    const double stage2_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - stage2_started).count();
    {
      auto f = output(out + "/stage2/status.json");
      f << "{\"status\":\"" << SegmentRefitStatusName(refit.status)
        << "\",\"reason\":\"" << refit.reason
        << "\",\"converged\":" << (refit.converged() ? "true" : "false")
        << ",\"outers\":" << refit.iterations.size()
        << ",\"calls\":" << stage2_totals.calls
        << ",\"trials\":" << stage2_totals.trials
        << ",\"accepted\":" << stage2_totals.accepted
        << ",\"rejected\":" << stage2_totals.rejected
        << ",\"unresolved\":" << stage2_totals.unresolved
        << ",\"inexact_handoffs\":" << refit.inexact_handoffs.size()
        << ",\"certificate_seconds\":" << stage2_totals.certificate_seconds
        << ",\"elapsed_seconds\":" << stage2_seconds << "}\n";
    }
    if (!refit.converged()) {
      a19r01::Counts s1{stage1_totals.calls, stage1_totals.trials,
                        stage1_totals.accepted, stage1_totals.rejected,
                        stage1_totals.unresolved, true};
      a19r01::Counts s2{stage2_totals.calls, stage2_totals.trials,
                        stage2_totals.accepted, stage2_totals.rejected,
                        stage2_totals.unresolved, true};
      a19r01::WriteFailure(out, refit.reason, "CONVERGED", "FAILED",
                           s1, s2);
      return 1;
    }
    stage2_completed = true;

    WriteAtomicText(out + "/stage2/stage_status.json",
                    "{\"solver_converged\":true,\"export_complete\":false,"
                    "\"score_complete\":false,\"consumable\":false}\n");

    // Preserve the exact final Stage-2 Values and the graph/factor identity
    // before scoring.  Scoring below receives this same in-memory result and
    // performs no optimizer call.
    {
      InferenceIdentityContext stage2_context = fixture.ctx;
      stage2_context.support_partition_sha256 =
          discovery.partition.partition_hash;
      stage2_context.calibration_sha256 = context.calibration_hash;
      stage2_context.solver_config_sha256 = sid;
      const auto export_result = WriteDevelopmentStage2Bundle(
          out + "/stage2", refit.graph, refit.values, refit.factor_metadata,
          stage2_context);
      if (!export_result.ok)
        throw std::runtime_error("STAGE2_EXPORT_FAILED:" + export_result.reason);
      WriteAtomicText(out + "/stage2/stage_status.json",
                      "{\"solver_converged\":true,\"export_complete\":true,"
                      "\"score_complete\":false,\"consumable\":false}\n");
    }

    const auto score_started = std::chrono::steady_clock::now();
    const bool no_candidates = discovery.partition.segments.empty();
    std::vector<GroupRecoverabilityScore> scores;
    if (no_candidates) {
      if (refit.status != SegmentRefitStatus::CONVERGED ||
          !refit.segments.empty())
        throw std::runtime_error("INVALID_EMPTY_SUPPORT_REFIT");
      for (const auto key : refit.values.keys())
        if (gtsam::Symbol(key).chr() == 'c')
          throw std::runtime_error("EMPTY_SUPPORT_HAS_LIVE_C");
    } else {
      scores = ScoreRefitRecoverability(
          refit, discovery.partition, plan, fixture.cfg);
    }
    fs::create_directories(out + "/scoring");
    WriteAtomicText(out + "/scoring/status.json",
        no_candidates
          ? "{\"status\":\"not_applicable\",\"reason\":\"NOT_APPLICABLE_NO_CANDIDATES\",\"computed\":false,\"eta\":null,\"s_m\":null,\"gamma\":null}\n"
          : "{\"status\":\"executed\",\"computed\":true}\n");
    const double score_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - score_started).count();
    size_t eligible = 0, unavailable = 0;
    for (const auto& score : scores) {
      eligible += score.eligible;
      unavailable += !score.valid_score_exported;
    }
    auto segments = output(out + "/segments.csv");
    segments << "segment_id,ordinal,stage1_merge_amplitude_m,"
                "stage2_refit_amplitude_m,amplitude_m,boundary,short_support,truth_m,"
                "postfit_gamma\n";
    for (const auto& estimate : refit.segments) {
      const auto support_it = std::find_if(
          discovery.partition.segments.begin(), discovery.partition.segments.end(),
          [&](const SupportSegment& s) {
            return s.segment_ordinal == estimate.segment_ordinal;
          });
      double gamma = std::numeric_limits<double>::quiet_NaN();
      for (const auto& score : scores)
        for (const auto& fit : score.segment_fit)
          if (fit.segment_ordinal == estimate.segment_ordinal) gamma = fit.gamma;
      segments << estimate.segment_id << ',' << estimate.segment_ordinal << ','
        << support_it->merge_snapshot_mean_m << ',' << estimate.amplitude_m << ','
        << estimate.amplitude_m << ','
        << estimate.boundary << ',' << estimate.short_support_debug << ",NA,"
        << gamma << '\n';
    }
    auto groups = output(out + "/scores.csv");
    groups << "group_id,ordinals,eligible,valid_score_exported,status,eta,s_m,"
              "s_is_infinite,numerical_reason,linearization_id\n";
    for (const auto& score : scores) {
      groups << score.group.group_id << ',';
      for (size_t i = 0; i < score.group.segment_ordinals.size(); ++i)
        groups << (i ? ";" : "") << score.group.segment_ordinals[i];
      groups << ',' << score.eligible << ',' << score.valid_score_exported << ','
        << score.status << ',';
      if (score.valid_score_exported)
        groups << score.numerical.eta << ',' << score.numerical.s_m << ','
               << score.numerical.s_is_infinite;
      else groups << "NA,NA,NA";
      groups << ',' << score.numerical.reason << ',' << score.linearization_id
             << '\n';

      const std::string group_dir =
          out + "/scoring/" + score.group.group_id;
      fs::create_directories(group_dir);
      WriteSparseCsv(group_dir + "/F_whitened.csv", score.F_whitened);
      WriteDenseCsv(group_dir + "/G_whitened.csv", score.G_whitened);
      WriteDenseCsv(group_dir + "/N.csv", score.numerical.N);
      WriteDenseCsv(group_dir + "/R.csv", score.numerical.R);
      WriteDenseCsv(group_dir + "/E.csv", score.numerical.E);
      auto rhs = output(group_dir + "/rhs_whitened.csv");
      rhs << "row,value\n";
      for (Eigen::Index row = 0; row < score.rhs_whitened.size(); ++row)
        rhs << row << ',' << score.rhs_whitened[row] << '\n';
      auto factors = output(group_dir + "/factor_rows.csv");
      factors << "original_factor_index,row_offset,row_count,obs_id,factor_type,segment_id\n";
      for (const auto& row : score.factor_rows)
        factors << row.original_factor_index << ',' << row.row_offset << ','
                << row.row_count << ',' << row.obs_id << ','
                << row.factor_type << ',' << row.segment_id << '\n';
      auto columns = output(group_dir + "/key_columns.csv");
      columns << "key,offset,dimension,role\n";
      for (const auto& column : score.key_columns)
        columns << gtsam::DefaultKeyFormatter(column.key) << ','
                << column.offset << ',' << column.dimension << ','
                << column.role << '\n';
      auto numerical = output(group_dir + "/numerical_audit.json");
      numerical << "{\"schema\":\"A19_R01_SCORE_AUDIT_V1\","
                   "\"linearization_id\":\"" << score.linearization_id
                << "\",\"status\":\"" << score.status
                << "\",\"numerical_status\":\""
                << RecoverabilityStatusName(score.numerical.status)
                << "\",\"reason\":\""
                << a19r01::JsonEscape(score.numerical.reason)
                << "\",\"rows\":" << score.numerical.rows
                << ",\"nuisance_columns_total\":"
                << score.numerical.nuisance_columns_total
                << ",\"nuisance_columns_active\":"
                << score.numerical.nuisance_columns_active
                << ",\"amplitude_columns\":"
                << score.numerical.amplitude_columns
                << ",\"frozen_F_rank\":" << score.numerical.frozen_F_rank
                << ",\"N_rank\":" << score.numerical.N_rank
                << ",\"R_rank\":" << score.numerical.R_rank
                << ",\"eta\":";
      if (score.valid_score_exported) numerical << score.numerical.eta;
      else numerical << "null";
      numerical << ",\"s_m\":";
      if (score.valid_score_exported && !score.numerical.s_is_infinite)
        numerical << score.numerical.s_m;
      else
        numerical << "null";
      numerical << ",\"s_is_infinite\":"
                << (score.numerical.s_is_infinite ? "true" : "false")
                << ",\"eligible\":" << (score.eligible ? "true" : "false")
                << ",\"valid_score_exported\":"
                << (score.valid_score_exported ? "true" : "false")
                << ",\"consumable\":false}\n";
    }
    segments.close();
    groups.close();
    MarkDevelopmentStage2ScoreComplete(out + "/stage2");
    WriteAtomicText(out + "/stage2/stage_status.json",
                    "{\"solver_converged\":true,\"export_complete\":true,"
                    "\"score_complete\":true,\"score_elapsed_seconds\":" +
                    std::to_string(score_seconds) +
                    ",\"consumable\":false}\n");

    if (t11_mode && validation_node["t11"]["cutoff_s"].as<double>() == 8) {
      try { t11::FixedModel(scores, refit, plan, cache, out); }
      catch (const std::exception& e) {
        fs::create_directories(out + "/diagnostic_fixed_model");
        WriteAtomicText(out + "/diagnostic_fixed_model/status.json",
          "{\"status\":\"FAILED\",\"reason\":\"" + a19r01::JsonEscape(e.what()) + "\"}\n");
      }
    }
    std::vector<FinalPolicySpec> selected_policies(kFinalPolicies.begin(), kFinalPolicies.end());
    if (t11_mode) selected_policies = {{"full_gate", FinalGatePolicy::FULL_GATE}, {"structured_debias", FinalGatePolicy::STRUCTURED_DEBIAS}};
    // Freeze every preregistered decision before any final optimizer runs.
    // The independent evaluator is a separate post-run process and no truth
    // path is accepted anywhere in this executable.
    fs::create_directories(out + "/decisions");
    for (const auto& spec : selected_policies) {
      const auto decisions = FreezeGroupDecisions(
          scores, R03Thresholds(), spec.policy);
      auto decision_file = output(out + "/decisions/" + spec.name + ".csv");
      decision_file << "group_id,eligible,decision,reason,eta_pass,s_pass,"
                       "gamma_pass,max_gamma,linearization_id\n";
      for (const auto& decision : decisions)
        decision_file << decision.group.group_id << ',' << decision.eligible
          << ',' << GroupDecisionName(decision.decision) << ','
          << decision.reason_code << ',' << decision.eta_pass << ','
          << decision.s_pass << ',' << decision.gamma_pass << ','
          << (decision.max_gamma_available
                  ? std::to_string(decision.max_gamma) : "NA") << ','
          << decision.decision_linearization_id << '\n';
    }
    {
      auto frozen = output(out + "/decisions/manifest.json");
      frozen << "{\"schema\":\"T10_A19_R08_VALIDATION_FROZEN_DECISIONS_V1\","
                "\"role\":\"validation\",\"validation_context_sha256\":\""
             << validation_context_sha256 << "\","
                "\"tau_eta\":0.10,\"tau_s_m\":0.10,\"tau_gamma\":1.0,"
                "\"epsilon_bad_m\":0.20,\"truth_read\":false,"
                "\"policy_identity\":\"" << sid
             << "\",\"consumable\":false}\n";
    }
    ::alarm(0);

    size_t final_successes = 0;
    struct FinalExit {
      std::string name;
      int code = 0;
      bool timed_out = false;
      double elapsed_seconds = 0.0;
    };
    std::vector<FinalExit> final_exits;
    {
      fs::create_directories(out + "/final");
      fs::create_directories(out + "/final_solver");
      std::cout.flush();
      std::cerr.flush();
      std::vector<FinalPolicySpec> scheduled_policies(
          selected_policies.begin(), selected_policies.end());
      if (scenario_id == "los")
        scheduled_policies.push_back(
            {"all_range", uifgo::FinalGatePolicy::SUPPRESS_ALL});
      for (const auto& spec : scheduled_policies) {
        const pid_t child = ::fork();
        if (child < 0) throw std::runtime_error("FINAL_FORK_FAILED");
        if (child == 0) {
          ::setpgid(0, 0);
          try {
            const std::string solver_root =
                out + "/final_solver/" + spec.name;
            fs::create_directories(solver_root);
            fs::create_directories(solver_root + "/accepted_recovery");
            fs::create_directories(solver_root + "/empty_recovery");
            fs::create_directories(solver_root + "/empty_fallback");
            DevelopmentStage2Request final_request;
            final_request.policy = kR03Policy;
            final_request.role = "validation";
            final_request.validation_context_sha256 = validation_context_sha256;
            final_request.implementation_identity = sid;
            final_request.allow_inexact_handoff = true;
            final_request.conditional_navigation = [&, solver_root](
                size_t outer, const NonlinearFactorGraph& graph,
                const Values& values, const CheckedLmOptions& lm_options,
                const std::vector<DevelopmentRefitRangeConstant>& constants) {
              return runCertified(
                  outer, graph, values, lm_options, ConvertRanges(constants),
                  solver_root + "/accepted_recovery/outer" +
                      std::to_string(outer),
                  true);
            };
            DevelopmentStage2Request empty_recovery_request;
            empty_recovery_request.policy = kCertifiedPolicy;
            empty_recovery_request.role = "validation";
            empty_recovery_request.validation_context_sha256 = validation_context_sha256;
            empty_recovery_request.implementation_identity = sid;
            empty_recovery_request.allow_inexact_handoff = false;
            empty_recovery_request.conditional_navigation =
                [&, solver_root](
                    size_t outer, const NonlinearFactorGraph& graph,
                    const Values& values,
                    const CheckedLmOptions& lm_options,
                    const std::vector<DevelopmentRefitRangeConstant>&
                        constants) {
                  return runCertified(
                      outer, graph, values, lm_options,
                      ConvertRanges(constants),
                      solver_root + "/empty_recovery/outer" +
                          std::to_string(outer),
                      false);
                };
            DevelopmentStage2Request empty_fallback_request =
                empty_recovery_request;
            empty_fallback_request.conditional_navigation =
                [&, solver_root](
                    size_t outer, const NonlinearFactorGraph& graph,
                    const Values& values,
                    const CheckedLmOptions& lm_options,
                    const std::vector<DevelopmentRefitRangeConstant>&
                        constants) {
                  return runCertified(
                      outer, graph, values, lm_options,
                      ConvertRanges(constants),
                      solver_root + "/empty_fallback/outer" +
                          std::to_string(outer),
                      false);
                };

            SegmentRefitResult inference_stage2 = refit;
            SupportPartition inference_support = discovery.partition;
            std::vector<GroupRecoverabilityScore> inference_scores = scores;
            if (std::string(spec.name) == "all_range") {
              inference_support.segments.clear();
              inference_support.partition_hash = "sha256:" +
                  Sha256Hex(validation_context_sha256 + ":all_range_empty_support");
              inference_support.solver_config_hash = sid;
              inference_stage2 = SegmentRefitter(RefitOptionsFrom(fixture.cfg))
                  .RunDevelopmentStage2(
                      fixture.graph, fixture.initial, metadata, plan,
                      fixture.cfg, inference_support,
                      &empty_recovery_request);
              if (!inference_stage2.converged())
                throw std::runtime_error("ALL_RANGE_PREPARATION_FAILED:" +
                                         inference_stage2.reason);
              inference_scores.clear();
            }
            InferenceIdentityContext final_context = fixture.ctx;
            final_context.support_partition_sha256 =
                inference_support.partition_hash;
            final_context.calibration_sha256 = context.calibration_hash;
            final_context.solver_config_sha256 = sid;
            const auto result = FinalInferenceEngine(
                R03Thresholds(), RefitOptionsFrom(fixture.cfg), {}, {},
                spec.policy, &final_request, &empty_recovery_request,
                &empty_fallback_request)
                .Run(fixture.graph, metadata, inference_stage2,
                     inference_support, inference_scores, plan, fixture.cfg,
                     final_context);
            const std::string final_root = out + "/final/" + spec.name;
            fs::create_directory(final_root);
            WriteInferenceArtifacts(final_root, result);
            auto status = output(final_root + "/validation_status.json");
            status << "{\"policy\":\"" << spec.name << "\",\"status\":\""
                   << InferenceStatusName(result.status)
                   << "\",\"valid_estimate\":"
                   << (result.valid_estimate() ? "true" : "false")
                   << ",\"consumable\":false}\n";
            status.close();
            ::_exit(result.valid_estimate() ? 0 : 3);
          } catch (const std::exception& error) {
            try {
              fs::create_directories(out + "/final/" + spec.name);
              auto failure = output(out + "/final/" + spec.name +
                                    "/child_failure.txt");
              failure << error.what() << '\n';
              failure.close();
            } catch (...) {
            }
            ::_exit(4);
          }
        }
        ::setpgid(child, child);
        bool timed_out = false;
        const auto final_started = std::chrono::steady_clock::now();
        const int code = WaitFinalChild(child, 900, &timed_out);
        const double final_seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - final_started).count();
        final_exits.push_back({spec.name, code, timed_out, final_seconds});
        final_successes += code == 0;
      }
      auto final_status = output(out + "/final/status.csv");
      final_status << "policy,exit_code,timed_out,hard_limit_seconds,elapsed_seconds\n";
      for (const auto& item : final_exits)
        final_status << item.name << ',' << item.code << ','
                     << item.timed_out << ",900," << item.elapsed_seconds << '\n';
    }
    {
      auto f = output(out + "/pipeline_status.json");
      f << "{\"status\":\"" << (no_candidates ? "NO_CANDIDATES" : "SCORED")
        << "\",\"Stage1\":\"CONVERGED\",\"Stage2\":\"CONVERGED\",\"scoring\":\""
        << (no_candidates ? "NOT_APPLICABLE_NO_CANDIDATES" : "EXECUTED")
        << "\",\"segments\":" << refit.segments.size() << ",\"groups\":"
        << scores.size() << ",\"eligible\":" << eligible
        << ",\"unavailable\":" << unavailable
        << ",\"final_attempted\":" << final_exits.size()
        << ",\"final_successes\":" << final_successes
        << ",\"stage1_elapsed_seconds\":" << stage1_seconds
        << ",\"stage2_elapsed_seconds\":" << stage2_seconds
        << ",\"score_elapsed_seconds\":" << score_seconds << "}\n";
    }
    std::cout << "SCORED segments=" << refit.segments.size()
              << " groups=" << scores.size() << " eligible=" << eligible
              << " unavailable=" << unavailable << '\n';
    return 0;
  } catch (const std::exception& e) {
    if (output_created) {
      a19r01::Counts s1{stage1_totals.calls, stage1_totals.trials,
                        stage1_totals.accepted, stage1_totals.rejected,
                        stage1_totals.unresolved, !stage1_callback_active};
      const bool constructor_before_iterate =
          stage2_callback_active && stage2_totals.calls == 0 &&
          std::string(e.what()).find("UNKNOWN_FACTOR") != std::string::npos;
      a19r01::Counts s2{stage2_totals.calls, stage2_totals.trials,
                        stage2_totals.accepted, stage2_totals.rejected,
                        stage2_totals.unresolved,
                        !stage2_callback_active || constructor_before_iterate};
      try {
        a19r01::WriteFailure(
            out, std::string(a18::exceptionStatus(e)) + ":" + e.what(),
            stage1_completed ? "CONVERGED" : "FAILED",
            stage2_completed
                ? "CONVERGED"
                : stage2_started
                ? (constructor_before_iterate
                       ? "FAILED_CONSTRUCTOR_BEFORE_ITERATE"
                       : "FAILED_EXCEPTION")
                : "NOT_RUN",
            s1, s2);
      } catch (const std::exception& status_error) {
        std::cerr << "STATUS_PERSISTENCE_FAILED:" << status_error.what()
                  << '\n';
      }
    }
    std::cerr << a18::exceptionStatus(e) << ':' << e.what() << '\n';
    return 2;
  }
}
