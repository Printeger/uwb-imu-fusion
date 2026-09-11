// Independent Stage-2 numerical diagnostic. Reuse the production loader,
// plan, graph construction, initialization, raw reference, refitter, and
// stationarity utilities. Oracle input contains support identity only.
#define Open OriginalPaperOpen
#define main uifgo_original_paper_main
#include "run_ie_paper.cpp"
#undef main
#undef Open

#include <random>

namespace {

std::ofstream Open(const fs::path& path) {
  auto out = OriginalPaperOpen(path);
  out << std::setprecision(17);
  return out;
}

struct Prepared {
  uifgo::Config cfg;
  uifgo::PaperInputPlan plan;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  std::vector<uifgo::FactorMeta> metadata;
};

Prepared Prepare(const fs::path& config_path) {
  Prepared output;
  output.cfg = uifgo::ConfigLoader::Load(config_path.string());
  ValidateSupportedConfig(output.cfg);
  std::vector<uifgo::ImuSample> imu;
  std::vector<uifgo::UwbFrame> raw;
  const auto loaded = LoadData(config_path.parent_path().string(), &output.cfg,
                               &imu, &raw, config_path.string());
  output.plan =
      uifgo::BuildPaperInputPlan(raw, output.cfg, loaded.recording_id);
  const auto keyframes = uifgo::MaterializePaperKeyframes(
      output.plan, uifgo::AllPlannedObservationMask(output.plan));
  uifgo::ValidatePaperFixedBeta(output.plan, output.cfg);
  const auto init = uifgo::Initializer(output.cfg).Run(imu, keyframes);
  if (!init.ok) throw std::runtime_error("INITIALIZER_FAILED");
  uifgo::GraphBuilder builder(output.cfg,
                               output.cfg.paper_imu_covariance_model);
  std::vector<size_t> uwb_indices;
  builder.Build(keyframes, imu, init, &output.graph, &output.initial,
                &uwb_indices);
  output.metadata = builder.factor_meta();
  if (uifgo::ReplacePosePriorsForPaperPath(&output.graph) != 1)
    throw std::runtime_error("PAPER_PRIOR_COUNT_MISMATCH");
  return output;
}

uifgo::CheckedLmOptions RawOptions(const uifgo::Config& cfg) {
  uifgo::CheckedLmOptions options;
  options.max_iterations = cfg.lm_max_iter;
  options.relative_tolerance = cfg.lm_rel_tol;
  options.absolute_tolerance = cfg.lm_abs_tol;
  return options;
}

uifgo::RefitOptions RefitOptions(const uifgo::Config& cfg,
                                  size_t max_outer) {
  uifgo::RefitOptions options;
  options.boundary_epsilon_m = cfg.refit_boundary_epsilon_m;
  options.relative_objective_tolerance =
      cfg.refit_relative_objective_tolerance;
  options.scaled_step_tolerance = cfg.refit_scaled_step_tolerance;
  options.projected_gradient_tolerance =
      cfg.refit_projected_gradient_tolerance;
  options.navigation_stationarity_tolerance_objective =
      cfg.refit_navigation_stationarity_tolerance_objective;
  options.gradient_roundoff_safety_factor =
      cfg.refit_gradient_roundoff_safety_factor;
  options.pose_rotation_scale_rad = cfg.refit_pose_rotation_scale_rad;
  options.pose_translation_scale_m = cfg.refit_pose_translation_scale_m;
  options.velocity_scale_mps = cfg.refit_velocity_scale_mps;
  options.accel_bias_scale_mps2 = cfg.refit_accel_bias_scale_mps2;
  options.gyro_bias_scale_radps = cfg.refit_gyro_bias_scale_radps;
  options.segment_amplitude_scale_m = cfg.refit_segment_amplitude_scale_m;
  options.max_refit_iterations = max_outer;
  options.lm_max_iterations = cfg.lm_max_iter;
  options.lm_relative_tolerance = cfg.lm_rel_tol;
  options.lm_absolute_tolerance = cfg.lm_abs_tol;
  return options;
}

uifgo::NavigationScales NavigationScales(const uifgo::Config& cfg) {
  uifgo::NavigationScales scales;
  scales.pose_rotation_rad = cfg.refit_pose_rotation_scale_rad;
  scales.pose_translation_m = cfg.refit_pose_translation_scale_m;
  scales.velocity_mps = cfg.refit_velocity_scale_mps;
  scales.accel_bias_mps2 = cfg.refit_accel_bias_scale_mps2;
  scales.gyro_bias_radps = cfg.refit_gyro_bias_scale_radps;
  return scales;
}

void WriteStationarity(std::ostream& out, const std::string& label,
                       const uifgo::NavigationStationarityAudit& audit) {
  out << label << ',' << audit.valid << ',' << audit.stationary << ','
      << audit.max_pose_rotation_gradient_objective_per_rad << ','
      << audit.max_pose_translation_gradient_objective_per_m << ','
      << audit.max_velocity_gradient_objective_per_mps << ','
      << audit.max_accel_bias_gradient_objective_per_mps2 << ','
      << audit.max_gyro_bias_gradient_objective_per_radps << ','
      << audit.max_scaled_gradient_objective << ','
      << audit.roundoff_allowance_objective << ','
      << gtsam::DefaultKeyFormatter(audit.dominant_key) << ','
      << audit.dominant_coordinate << ',' << audit.dominant_category << ','
      << audit.dominant_native_gradient_objective << ','
      << audit.dominant_physical_scale << ','
      << audit.dominant_scaled_gradient_objective << ','
      << audit.dominant_absolute_factor_gradient_sum_objective << ','
      << audit.dominant_roundoff_allowance_objective << ','
      << CsvEscape(audit.reason) << '\n';
}

void WriteTrace(const fs::path& path,
                const uifgo::SegmentRefitResult& result) {
  auto out = Open(path);
  out << "outer,c_before,c_after,delta_c,conditional_objective,"
         "navigation_gradient_before_c,dominant_key_before,"
         "dominant_coordinate_before,dominant_category_before,"
         "dominant_native_before,dominant_scale_before,"
         "absolute_factor_gradient_sum_before,roundoff_before,lm_status,"
         "lm_iterations,lm_inner_iterations,lambda,conditional_scaled_step,"
         "joint_objective,relative_objective_change,navigation_gradient_after_c,"
         "dominant_key_after,dominant_coordinate_after,dominant_category_after,"
         "dominant_native_after,dominant_scale_after,"
         "absolute_factor_gradient_sum_after,roundoff_after,c_gradient,c_kkt,"
         "full_scaled_step,objective_ok,step_ok,kkt_ok,stationarity_ok\n";
  for (const auto& row : result.iterations) {
    if (row.segment_updates.empty()) {
      out << row.outer_iteration << ",,,,,";
    } else {
      const auto& segment = row.segment_updates.front();
      out << row.outer_iteration << ',' << segment.c_before_m << ','
          << segment.c_after_m << ',' << segment.delta_c_m << ',';
    }
    out << row.conditional_objective << ','
        << row.navigation_gradient_before_c_update << ','
        << gtsam::DefaultKeyFormatter(row.dominant_key_before_c_update) << ','
        << row.dominant_coordinate_before_c_update << ','
        << row.dominant_category_before_c_update << ','
        << row.dominant_native_gradient_before_c_update << ','
        << row.dominant_scale_before_c_update << ','
        << row.dominant_absolute_factor_gradient_sum_before_c_update << ','
        << row.navigation_roundoff_before_c_update << ','
        << CsvEscape(row.conditional_inner_status) << ','
        << row.conditional_lm_iterations << ','
        << row.conditional_lm_inner_iterations << ','
        << row.conditional_lm_lambda << ','
        << row.conditional_scaled_navigation_step << ','
        << row.objective_after << ',' << row.relative_objective_change << ','
        << row.max_scaled_navigation_gradient_objective << ','
        << gtsam::DefaultKeyFormatter(
               row.dominant_navigation_key_after_c_update)
        << ',' << row.dominant_navigation_coordinate_after_c_update << ','
        << row.dominant_navigation_category_after_c_update << ','
        << row.dominant_navigation_native_gradient_after_c_update << ','
        << row.dominant_navigation_scale_after_c_update << ','
        << row.dominant_navigation_absolute_factor_gradient_sum_after_c_update
        << ',' << row.navigation_gradient_roundoff_allowance_objective << ',';
    if (row.segment_updates.empty()) {
      out << ",,";
    } else {
      out << row.segment_updates.front().gradient_after_objective_per_m << ','
          << row.segment_updates.front().kkt_violation_after << ',';
    }
    out << row.scaled_state_step << ',' << row.objective_ok << ','
        << row.step_ok << ',' << row.kkt_ok << ','
        << row.navigation_stationarity_ok << '\n';
  }
}

void WriteFiniteDifference(
    const fs::path& path,
    const std::vector<std::pair<std::string,
        uifgo::CheckedLmFiniteDifferenceDiagnostics>>& diagnostics) {
  auto out = Open(path);
  out << "selection,key,key_value,coordinate,dimension,unit,scale,objective,"
         "analytic_gradient,scaled_analytic_gradient,step,objective_plus,"
         "objective_minus,central_derivative,absolute_mismatch,"
         "agreement_tolerance,agrees,valid,reason\n";
  for (const auto& item : diagnostics) {
    const auto& d = item.second;
    if (d.points.empty()) {
      out << item.first << ',' << d.key << ',' << d.key_value << ','
          << d.coordinate << ',' << d.dimension << ',' << d.coordinate_unit
          << ',' << d.coordinate_scale << ',' << d.objective << ','
          << d.analytic_gradient << ',' << d.scaled_analytic_gradient
          << ",,,,,,," << d.valid << ',' << CsvEscape(d.reason) << '\n';
      continue;
    }
    for (const auto& point : d.points) {
      out << item.first << ',' << d.key << ',' << d.key_value << ','
          << d.coordinate << ',' << d.dimension << ',' << d.coordinate_unit
          << ',' << d.coordinate_scale << ',' << d.objective << ','
          << d.analytic_gradient << ',' << d.scaled_analytic_gradient << ','
          << point.step << ',' << point.objective_plus << ','
          << point.objective_minus << ',' << point.central_derivative << ','
          << point.absolute_difference_from_analytic << ','
          << point.agreement_tolerance << ',' << point.agrees << ','
          << d.valid << ',' << CsvEscape(d.reason) << '\n';
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 5)
      throw std::invalid_argument(
          "usage: stage2_stationarity_forensics CLEAN_CONFIG "
          "INJECTED_CONFIG ORACLE_SUPPORT OUTPUT_NEW_DIR");
    const fs::path clean_config = fs::canonical(fs::absolute(argv[1]));
    const fs::path injected_config = fs::canonical(fs::absolute(argv[2]));
    const fs::path oracle_path = fs::canonical(fs::absolute(argv[3]));
    const fs::path root = fs::absolute(argv[4]);
    if (fs::exists(root)) throw std::runtime_error("OUTPUT_EXISTS");
    fs::create_directories(root);

    const auto clean = Prepare(clean_config);
    const auto injected = Prepare(injected_config);
    const auto clean_reference = uifgo::PrepareRawGaussianReference(
        clean.graph, clean.initial, RawOptions(clean.cfg));
    const auto injected_reference = uifgo::PrepareRawGaussianReference(
        injected.graph, injected.initial, RawOptions(injected.cfg));
    if (!clean_reference.solve.converged ||
        !injected_reference.solve.converged)
      throw std::runtime_error("RAW_REFERENCE_FAILED");

    auto oracle = uifgo::OracleSupportProvider::Load(
        oracle_path.string(), injected.plan,
        static_cast<size_t>(injected.cfg.oracle_short_min_count_debug),
        injected.cfg.oracle_short_min_duration_debug);
    auto support = uifgo::ToSupportPartition(oracle);
    if (support.segments.size() != 1 ||
        support.segments.front().obs_ids.size() != 30)
      throw std::runtime_error("LOCKED_SUPPORT_IDENTITY_MISMATCH");

    const auto stage2_50 = uifgo::SegmentRefitter(
        RefitOptions(injected.cfg, 50)).Run(
            injected.graph, injected.initial, injected.metadata,
            injected.plan, injected.cfg, support);
    WriteTrace(root / "stage2_original_50.csv", stage2_50);

    const auto stage2_200 = uifgo::SegmentRefitter(
        RefitOptions(injected.cfg, 200)).Run(
            injected.graph, injected.initial, injected.metadata,
            injected.plan, injected.cfg, support);
    WriteTrace(root / "stage2_original_200.csv", stage2_200);

    const auto scales = NavigationScales(injected.cfg);
    const auto clean_audit = uifgo::AuditNavigationStationarity(
        clean.graph, clean_reference.solve.values, scales,
        injected.cfg.refit_navigation_stationarity_tolerance_objective,
        injected.cfg.refit_gradient_roundoff_safety_factor);
    const auto injected_audit = uifgo::AuditNavigationStationarity(
        injected.graph, injected_reference.solve.values, scales,
        injected.cfg.refit_navigation_stationarity_tolerance_objective,
        injected.cfg.refit_gradient_roundoff_safety_factor);
    const auto stage2_audit = uifgo::AuditNavigationStationarity(
        stage2_50.graph, stage2_50.values, scales,
        injected.cfg.refit_navigation_stationarity_tolerance_objective,
        injected.cfg.refit_gradient_roundoff_safety_factor);
    {
      auto out = Open(root / "three_solution_stationarity.csv");
      out << "solution,valid,stationary,max_pose_rotation_gradient,"
             "max_pose_translation_gradient,max_velocity_gradient,"
             "max_accel_bias_gradient,max_gyro_bias_gradient,"
             "max_scaled_navigation_gradient,roundoff_allowance,"
             "dominant_key,dominant_coordinate,dominant_category,"
             "dominant_native_gradient,physical_scale,"
             "dominant_scaled_gradient,absolute_factor_gradient_sum,"
             "dominant_roundoff_allowance,reason\n";
      WriteStationarity(out, "clean_raw_reference", clean_audit);
      WriteStationarity(out, "injected_raw_reference", injected_audit);
      WriteStationarity(out, "oracle_stage2_terminal_50", stage2_audit);
    }

    const std::vector<double> steps{1e-2, 3e-3, 1e-3, 3e-4, 1e-4,
                                    3e-5, 1e-5, 3e-6, 1e-6, 3e-7};
    std::vector<std::pair<gtsam::Key, size_t>> coordinates;
    for (gtsam::Key key : stage2_50.values.keys()) {
      const char symbol = gtsam::Symbol(key).chr();
      if (symbol == 'c') continue;
      const size_t dimension = stage2_50.values.at(key).dim();
      if ((symbol == 'x' && dimension == 6) ||
          (symbol == 'v' && dimension == 3) ||
          (symbol == 'b' && dimension == 6))
        for (size_t coordinate = 0; coordinate < dimension; ++coordinate)
          coordinates.emplace_back(key, coordinate);
    }
    std::mt19937_64 generator(911);
    std::shuffle(coordinates.begin(), coordinates.end(), generator);
    std::vector<std::pair<std::string,
        uifgo::CheckedLmFiniteDifferenceDiagnostics>> fd;
    fd.emplace_back(
        "dominant",
        uifgo::CheckNavigationCoordinateByFiniteDifference(
            stage2_50.graph, stage2_50.values, scales,
            stage2_audit.dominant_key, stage2_audit.dominant_coordinate,
            steps));
    for (size_t index = 0; index < std::min<size_t>(8, coordinates.size());
         ++index)
      fd.emplace_back(
          "random_seed911_" + std::to_string(index),
          uifgo::CheckNavigationCoordinateByFiniteDifference(
              stage2_50.graph, stage2_50.values, scales,
              coordinates[index].first, coordinates[index].second, steps));
    WriteFiniteDifference(root / "finite_difference.csv", fd);

    // Replay the original V1 alternating path and, at its final fixed-C
    // checkpoint, probe the already-implemented V2 continuation policy.  The
    // probe is development-only and is never returned to the refitter, so it
    // cannot publish Stage-2 outputs or alter the authoritative V1 trace.
    auto checkpoint_support = support;
    checkpoint_support.solver_config_hash =
        "a19-policy-sha256:" +
        uifgo::Sha256Hex("stage2-terminal-fixed-c-v2-probe");
    std::vector<std::pair<int, uifgo::CheckedLmResult>>
        terminal_fixed_c_v2_probes;
    bool terminal_fixed_c_v2_ran = false;
    uifgo::DevelopmentStage2Request checkpoint_request;
    checkpoint_request.policy = "PAPER_CERTIFIED_PAIR_REDUCTION_V1";
    checkpoint_request.role = "development";
    checkpoint_request.implementation_identity =
        checkpoint_support.solver_config_hash;
    checkpoint_request.conditional_navigation = [&](
        size_t outer, const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values& values, const uifgo::CheckedLmOptions& options,
        const std::vector<uifgo::DevelopmentRefitRangeConstant>&) {
      auto original_options = options;
      original_options.policy =
          uifgo::ConditionalLmPolicy::GTSAM_CHECK_ONLY_V1;
      if (outer == 50) {
        terminal_fixed_c_v2_ran = true;
        for (const int budget : {100, 200, 500, 1000}) {
          auto probe_options = options;
          probe_options.max_iterations = budget;
          terminal_fixed_c_v2_probes.emplace_back(
              budget, uifgo::RunCheckedConditionalLm(
                          graph, values, probe_options));
        }
      }
      return uifgo::RunCheckedConditionalLm(graph, values, original_options);
    };
    const auto checkpoint_replay = uifgo::SegmentRefitter(
        RefitOptions(injected.cfg, 50)).RunDevelopmentStage2(
            injected.graph, injected.initial, injected.metadata,
            injected.plan, injected.cfg, checkpoint_support,
            &checkpoint_request);
    WriteTrace(root / "stage2_checkpoint_replay.csv", checkpoint_replay);
    {
      auto probe = Open(root / "terminal_fixed_c_v2_probe.json");
      probe << "{\n  \"ran\": " << terminal_fixed_c_v2_ran
            << ",\n  \"probes\": [\n";
      for (size_t index = 0; index < terminal_fixed_c_v2_probes.size();
           ++index) {
        const int budget = terminal_fixed_c_v2_probes[index].first;
        const auto& result = terminal_fixed_c_v2_probes[index].second;
        const auto& audit = result.last_qualification_stationarity;
        probe << "    {\"budget\": " << budget
              << ", \"converged\": " << result.converged
              << ", \"reason\": \"" << JsonEscape(result.reason)
              << "\", \"iterations\": " << result.iterations
              << ", \"inner_iterations\": " << result.inner_iterations
              << ", \"lambda\": " << result.lambda
              << ", \"iterate_calls\": "
              << result.convergence.iterate_call_count
              << ", \"accepted_updates\": "
              << result.convergence.accepted_update_count
              << ", \"rejected_lambda_trials\": "
              << result.convergence.rejected_lambda_trial_count
              << ", \"qualification_evaluations\": "
              << result.convergence.qualification_evaluation_count
              << ", \"last_stationarity_valid\": " << audit.valid
              << ", \"last_stationarity_passed\": " << audit.stationary
              << ", \"last_max_scaled_navigation_gradient\": "
              << audit.max_scaled_gradient_objective
              << ", \"last_roundoff_allowance\": "
              << audit.roundoff_allowance_objective << '}';
        if (index + 1 != terminal_fixed_c_v2_probes.size()) probe << ',';
        probe << '\n';
      }
      probe << "  ]\n}\n";
    }

    auto v2_support = support;
    v2_support.solver_config_hash =
        "a19-policy-sha256:" + uifgo::Sha256Hex("stage2-stationarity-v2-diagnostic");
    uifgo::DevelopmentStage2Request request;
    request.policy = "PAPER_CERTIFIED_PAIR_REDUCTION_V1";
    request.role = "development";
    request.implementation_identity = v2_support.solver_config_hash;
    std::vector<std::pair<size_t, uifgo::CheckedLmResult>> v2_calls;
    std::vector<std::pair<int, uifgo::CheckedLmResult>> v2_restarts;
    request.conditional_navigation = [&v2_calls, &v2_restarts](
        size_t outer, const gtsam::NonlinearFactorGraph& graph,
        const gtsam::Values& values, const uifgo::CheckedLmOptions& options,
        const std::vector<uifgo::DevelopmentRefitRangeConstant>&) {
      uifgo::CheckedLmDiagnosticRequest capture;
      auto result =
          uifgo::RunCheckedConditionalLm(graph, values, options, &capture);
      v2_calls.emplace_back(outer, result);
      if (!result.converged && !result.diagnostic.values_at_final.empty()) {
        for (const int budget : {100, 500}) {
          auto restart_options = options;
          restart_options.max_iterations = budget;
          v2_restarts.emplace_back(
              budget, uifgo::RunCheckedConditionalLm(
                          graph, result.diagnostic.values_at_final,
                          restart_options));
        }
      }
      return result;
    };
    auto v2_refit_options = RefitOptions(injected.cfg, 50);
    v2_refit_options.lm_max_iterations = 500;
    const auto stage2_v2 = uifgo::SegmentRefitter(
        v2_refit_options).RunDevelopmentStage2(
            injected.graph, injected.initial, injected.metadata,
            injected.plan, injected.cfg, v2_support, &request);
    WriteTrace(root / "stage2_v2_diagnostic.csv", stage2_v2);
    {
      auto calls = Open(root / "stage2_v2_conditional_calls.csv");
      calls << "outer,converged,reason,iterations,inner_iterations,lambda,"
               "iterate_calls,accepted_updates,rejected_lambda_trials,"
               "qualification_evaluations,last_stationarity_valid,"
               "last_stationarity_passed,last_max_scaled_gradient,"
               "last_roundoff_allowance,current_error\n";
      for (const auto& item : v2_calls) {
        const auto& result = item.second;
        const auto& audit = result.last_qualification_stationarity;
        calls << item.first << ',' << result.converged << ','
              << CsvEscape(result.reason) << ',' << result.iterations << ','
              << result.inner_iterations << ',' << result.lambda << ','
              << result.convergence.iterate_call_count << ','
              << result.convergence.accepted_update_count << ','
              << result.convergence.rejected_lambda_trial_count << ','
              << result.convergence.qualification_evaluation_count << ','
              << audit.valid << ',' << audit.stationary << ','
              << audit.max_scaled_gradient_objective << ','
              << audit.roundoff_allowance_objective << ','
              << result.convergence.current_error << '\n';
      }
    }
    {
      auto calls = Open(root / "stage2_v2_fixed_checkpoint_restarts.csv");
      calls << "budget,converged,reason,iterations,inner_iterations,lambda,"
               "iterate_calls,accepted_updates,rejected_lambda_trials,"
               "qualification_evaluations,last_stationarity_valid,"
               "last_stationarity_passed,last_max_scaled_gradient,"
               "last_roundoff_allowance,current_error\n";
      for (const auto& item : v2_restarts) {
        const auto& result = item.second;
        const auto& audit = result.last_qualification_stationarity;
        calls << item.first << ',' << result.converged << ','
              << CsvEscape(result.reason) << ',' << result.iterations << ','
              << result.inner_iterations << ',' << result.lambda << ','
              << result.convergence.iterate_call_count << ','
              << result.convergence.accepted_update_count << ','
              << result.convergence.rejected_lambda_trial_count << ','
              << result.convergence.qualification_evaluation_count << ','
              << audit.valid << ',' << audit.stationary << ','
              << audit.max_scaled_gradient_objective << ','
              << audit.roundoff_allowance_objective << ','
              << result.convergence.current_error << '\n';
      }
    }

    auto out = Open(root / "summary.json");
    out << "{\n  \"schema\": \"stage2_stationarity_forensics_v1\",\n"
        << "  \"clean_config_sha256\": \""
        << uifgo::Sha256FileHex(clean_config.string()) << "\",\n"
        << "  \"injected_config_sha256\": \""
        << uifgo::Sha256FileHex(injected_config.string()) << "\",\n"
        << "  \"oracle_support_sha256\": \""
        << uifgo::Sha256FileHex(oracle_path.string()) << "\",\n"
        << "  \"oracle_obs_count\": "
        << support.segments.front().obs_ids.size() << ",\n"
        << "  \"truth_amplitude_read\": false,\n"
        << "  \"stage2_50_status\": \""
        << uifgo::SegmentRefitStatusName(stage2_50.status) << "\",\n"
        << "  \"stage2_50_reason\": \"" << JsonEscape(stage2_50.reason)
        << "\",\n  \"stage2_50_iterations\": "
        << stage2_50.iterations.size() << ",\n"
        << "  \"stage2_200_status\": \""
        << uifgo::SegmentRefitStatusName(stage2_200.status) << "\",\n"
        << "  \"stage2_200_reason\": \"" << JsonEscape(stage2_200.reason)
        << "\",\n  \"stage2_200_iterations\": "
        << stage2_200.iterations.size() << ",\n"
        << "  \"stage2_v2_status\": \""
        << uifgo::SegmentRefitStatusName(stage2_v2.status) << "\",\n"
        << "  \"stage2_v2_reason\": \"" << JsonEscape(stage2_v2.reason)
        << "\",\n  \"stage2_v2_iterations\": "
        << stage2_v2.iterations.size() << "\n}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "STAGE2_STATIONARITY_FORENSICS_FAILED: " << error.what()
              << '\n';
    return 1;
  }
}
