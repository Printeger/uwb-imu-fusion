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

namespace {

const char* kA19Schema = "A19_R03_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY";
const char* kR03Policy =
    "PAPER_CERTIFIED_PAIR_REDUCTION_V1+PAPER_STAGE2_INEXACT_HANDOFF_V1";

std::string Strategy19(const std::string& config) {
  std::string material = kR03Policy +
      std::string("A19_R03_CPP_MPFR_333_STAGE2_INEXACT_HANDOFF_V1") +
      Sha256FileHex(config) + Sha256FileHex("/proc/self/exe");
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
    if (argc != 7 || std::string(argv[1]) != "--policy" ||
        std::string(argv[2]) != kR03Policy)
      throw std::runtime_error("A19_DEFAULT_OFF_OR_INVALID_POLICY");
    const std::string mode = argv[3], config = argv[4], initialization = argv[5];
    out = fs::absolute(argv[6]);
    if (mode != "development-stage1-stage2-score")
      throw std::runtime_error("A19_SCOPE_REJECTED");
    if (initialization != "ORIGINAL_RAW_NO_CHECKPOINT")
      throw std::runtime_error("A19_WARM_START_FORBIDDEN");
    if (Sha256FileHex(config) !=
        "479df3daae2096bb665582f60611c66841ed87edc496ddcbaf7cc9e180ed6c4f")
      throw std::runtime_error("FROZEN_CONFIG_IDENTITY");
    if (!fs::create_directory(out)) throw std::runtime_error("OUTPUT_EXISTS");
    output_created = true;

    const std::string sid = Strategy19(config);
    {
      auto f = output(out + "/diagnostic_manifest.json");
      f << "{\"schema\":\"" << kA19Schema
        << "\",\"role\":\"development\",\"consumable\":false,"
           "\"policy\":\"" << kR03Policy << "\",\"strategy_identity\":\""
        << sid << "\",\"implementation\":\"A19_R03_CPP_MPFR_333_STAGE2_INEXACT_HANDOFF_V1\"}\n";
    }
    bool cache_rejected = false;
    try { ReadStage2CacheManifest(out + "/diagnostic_manifest.json"); }
    catch (const std::exception&) { cache_rejected = true; }
    if (!cache_rejected) throw std::runtime_error("CACHE_READER_MUST_REJECT");

    A17Graph fixture(config);
    const auto cache = LoadT07ScenarioCache(
        fixture.cfg.t07_cache_manifest, fixture.cfg.t07_cache_start_s,
        fixture.cfg.t07_cache_duration_s);
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

    DiscoveryContext context;
    context.input_plan_hash = plan.plan_sha256;
    context.source_hash = sid;
    context.config_hash = fixture.ctx.config_sha256;
    context.calibration_hash = "sha256:" +
        Sha256Hex(fixture.ctx.config_sha256 + "fixed-synthetic-assumptions");
    context.solver_config_hash = sid;

    DevelopmentStage1Request stage1_request;
    stage1_request.policy = policy;
    stage1_request.role = "development";
    stage1_request.implementation_identity = sid;
    stage1_request.output_schema = kA19Schema;
    stage1_request.output_provider = "development_nonconsumable";
    stage1_request.conditional_navigation = [&](
        size_t outer, const NonlinearFactorGraph& graph, const Values& values,
        const CheckedLmOptions& lm_options,
        const std::vector<DevelopmentRangeConstant>& range_constants) {
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
    auto discovery = AutomaticSupportProvider(options(fixture.cfg))
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
    const size_t candidate_observations = a19r01::WriteStage1Checkpoint(
        out, discovery.snapshot, discovery.partition);
    stage1_completed = true;

    fs::create_directory(out + "/stage2");
    stage2_started = true;
    DevelopmentStage2Request stage2_request;
    stage2_request.policy = kR03Policy;
    stage2_request.role = "development";
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
    const auto stage2_started = std::chrono::steady_clock::now();
    auto refit = SegmentRefitter(RefitOptionsFrom(fixture.cfg))
        .RunDevelopmentStage2(fixture.graph, discovery.navigation_values,
                              metadata, plan, fixture.cfg,
                              discovery.partition, &stage2_request);
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

    // Preserve the exact final Stage-2 Values and the graph/factor identity
    // before scoring.  Scoring below receives this same in-memory result and
    // performs no optimizer call.
    {
      auto values_file = output(out + "/stage2/final_values.csv");
      values_file << "phase,index,name,row,column,hex,bits\n";
      values(values_file, "STAGE2_FINAL", refit.values);
      auto factor_file = output(out + "/stage2/factor_metadata.csv");
      factor_file << "factor_index,obs_id,factor_type,segment_id,keys\n";
      for (const auto& meta : refit.factor_metadata) {
        factor_file << meta.factor_index << ',' << meta.obs_id << ','
                    << meta.factor_type << ',' << meta.segment_id << ',';
        for (auto key : meta.keys)
          factor_file << gtsam::DefaultKeyFormatter(key) << ';';
        factor_file << '\n';
      }
      auto identity = ComputeInferenceContentIdentity(
          refit.graph, refit.values, fixture.ctx);
      auto identity_file = output(out + "/stage2/content_identity.json");
      identity_file << "{\"schema\":\"A19_R01_STAGE2_CONTENT_IDENTITY_V1\","
                       "\"graph_linearization_sha256\":\""
                    << identity.graph_linearization_sha256
                    << "\",\"values_sha256\":\"" << identity.values_sha256
                    << "\",\"factor_count\":" << refit.graph.size()
                    << ",\"values_count\":" << refit.values.size()
                    << ",\"consumable\":false}\n";
    }

    const auto scores = ScoreRefitRecoverability(
        refit, discovery.partition, plan, fixture.cfg);
    size_t eligible = 0, unavailable = 0;
    for (const auto& score : scores) {
      eligible += score.eligible;
      unavailable += !score.valid_score_exported;
    }
    auto segments = output(out + "/segments.csv");
    segments << "segment_id,ordinal,stage1_merge_amplitude_m,"
                "stage2_refit_amplitude_m,boundary,short_support,truth_m,"
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

    // Freeze every preregistered decision before any final optimizer runs.
    // The independent evaluator is a separate post-run process and no truth
    // path is accepted anywhere in this executable.
    fs::create_directories(out + "/decisions");
    for (const auto& spec : kFinalPolicies) {
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
      frozen << "{\"schema\":\"A19_R03_FROZEN_DECISIONS_V1\","
                "\"tau_eta\":0.10,\"tau_s_m\":0.10,\"tau_gamma\":1.0,"
                "\"epsilon_bad_m\":0.20,\"truth_read\":false,"
                "\"policy_identity\":\"" << sid
             << "\",\"consumable\":false}\n";
    }

    size_t final_successes = 0;
    std::vector<std::pair<std::string, int>> final_exits;
    if (eligible != 0) {
      fs::create_directories(out + "/final");
      fs::create_directories(out + "/final_solver");
      std::cout.flush();
      std::cerr.flush();
      for (const auto& spec : kFinalPolicies) {
        const pid_t child = ::fork();
        if (child < 0) throw std::runtime_error("FINAL_FORK_FAILED");
        if (child == 0) {
          try {
            const std::string solver_root =
                out + "/final_solver/" + spec.name;
            fs::create_directories(solver_root);
            DevelopmentStage2Request final_request;
            final_request.policy = kR03Policy;
            final_request.role = "development";
            final_request.implementation_identity = sid;
            final_request.allow_inexact_handoff = true;
            final_request.conditional_navigation = [&, solver_root](
                size_t outer, const NonlinearFactorGraph& graph,
                const Values& values, const CheckedLmOptions& lm_options,
                const std::vector<DevelopmentRefitRangeConstant>& constants) {
              return runCertified(
                  outer, graph, values, lm_options, ConvertRanges(constants),
                  solver_root + "/outer" + std::to_string(outer), true);
            };

            InferenceIdentityContext final_context = fixture.ctx;
            final_context.support_partition_sha256 =
                discovery.partition.partition_hash;
            final_context.calibration_sha256 = context.calibration_hash;
            final_context.solver_config_sha256 = sid;
            const auto result = FinalInferenceEngine(
                R03Thresholds(), RefitOptionsFrom(fixture.cfg), {}, {},
                spec.policy, &final_request)
                .Run(fixture.graph, metadata, refit, discovery.partition,
                     scores, plan, fixture.cfg, final_context);
            const std::string final_root = out + "/final/" + spec.name;
            fs::create_directory(final_root);
            WriteInferenceArtifacts(final_root, result);
            auto status = output(final_root + "/development_status.json");
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
        int child_status = 0;
        if (::waitpid(child, &child_status, 0) != child)
          throw std::runtime_error("FINAL_WAITPID_FAILED");
        const int code = WIFEXITED(child_status)
                             ? WEXITSTATUS(child_status)
                             : 128 + (WIFSIGNALED(child_status)
                                          ? WTERMSIG(child_status) : 0);
        final_exits.emplace_back(spec.name, code);
        final_successes += code == 0;
      }
      auto final_status = output(out + "/final/status.csv");
      final_status << "policy,exit_code\n";
      for (const auto& item : final_exits)
        final_status << item.first << ',' << item.second << '\n';
    }
    {
      auto f = output(out + "/pipeline_status.json");
      f << "{\"status\":\"SCORED\",\"Stage1\":\"CONVERGED\","
           "\"Stage2\":\"CONVERGED\",\"scoring\":\"EXECUTED\","
           "\"segments\":" << refit.segments.size() << ",\"groups\":"
        << scores.size() << ",\"eligible\":" << eligible
        << ",\"unavailable\":" << unavailable
        << ",\"final_attempted\":" << final_exits.size()
        << ",\"final_successes\":" << final_successes << "}\n";
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
