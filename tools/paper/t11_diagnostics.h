#pragma once
// Runner-only T11 audits. No estimator or score definitions are replaced.
namespace t11 {
inline void CheckPrefix(const std::string& config, const YAML::Node& context,
                        const std::string& out) {
  const auto cfg = ConfigLoader::Load(config);
  const double cutoff = context["t11"]["cutoff_s"].as<double>();
  if ((cutoff != 6 && cutoff != 7 && cutoff != 8) ||
      cfg.t07_cache_start_s != 0 || cfg.t07_cache_duration_s != cutoff ||
      context["scenario_id"].as<std::string>() != "step2")
    throw std::runtime_error("T11_INVALID_PREFIX_CONTEXT");
  const auto manifest = YAML::LoadFile(cfg.t07_cache_manifest);
  const auto directory = fs::path(cfg.t07_cache_manifest).parent_path();
  auto audit = output(out + "/prefix_before_initialization.csv");
  audit << "input,count,min_time_s,max_time_s,cutoff_s\n";
  for (const auto* kind : {"imu", "uwb"}) {
    const std::string name(kind);
    const auto path = directory / manifest[name + "_file"].as<std::string>();
    if (fs::canonical(path).parent_path() != fs::canonical(directory))
      throw std::runtime_error("T11_PREFIX_PATH_ESCAPES_DIRECTORY");
    const auto records = read(path.string());
    const size_t expected = name == "imu" ? size_t(cutoff * 200 + 1)
                                          : size_t((cutoff * 5 + 1) * 8);
    double lo = cutoff, hi = 0;
    for (const auto& row : records) {
      const double time = std::stod(row.at("sensor_time_s"));
      if (!std::isfinite(time) || time < 0 || time > cutoff)
        throw std::runtime_error("T11_STRICT_FUTURE_SAMPLE_REJECTED");
      lo = std::min(lo, time); hi = std::max(hi, time);
    }
    if (records.size() != expected || lo != 0 || hi != cutoff)
      throw std::runtime_error("T11_PREFIX_COUNT_OR_EXACT_ENDPOINT");
    audit << name << ',' << records.size() << ',' << lo << ',' << hi << ',' << cutoff << '\n';
  }
}
inline std::string Calibration(const std::string& config, const std::string& out) {
  const auto source = YAML::LoadFile(config);
  YAML::Node fixed;
  for (auto key : {"anchors", "extrinsics", "calibration", "imu", "uwb", "paper"})
    fixed[key] = source[key];
  const std::string content = YAML::Dump(fixed);
  WriteAtomicText(out + "/calibration_content.yaml", content + "\n");
  return "sha256:" + Sha256Hex(content);
}
struct Dependency { double start=0, end=0, sample_min=0, sample_max=0; std::string type; };
inline Dependency FactorDependency(const NonlinearFactor& factor,
    const PaperInputPlan& plan, const std::vector<ImuSample>& imu) {
  Dependency d;
  const auto* pim = dynamic_cast<const gtsam::CombinedImuFactor*>(&factor);
  if (pim) {
    d.type = "imu";
    d.start = plan.keyframes.at(Symbol(factor.keys().at(0)).index()).sensor_time;
    d.end = plan.keyframes.at(Symbol(factor.keys().at(2)).index()).sensor_time;
    if (!(d.start < d.end) || imu.empty() || d.start < imu.front().t || d.end > imu.back().t)
      throw std::runtime_error("T11_IMU_ENDPOINT_OUTSIDE_INPUT");
    auto first = std::upper_bound(imu.begin(), imu.end(), d.start,
        [](double t, const ImuSample& s) { return t < s.t; });
    auto endpoint = std::lower_bound(imu.begin(), imu.end(), d.end,
        [](const ImuSample& s, double t) { return s.t < t; });
    if (first == imu.end() || endpoint == imu.end())
      throw std::runtime_error("T11_IMU_SAMPLE_DEPENDENCY_MISSING");
    d.sample_min = first->t; d.sample_max = endpoint->t;
    // InterpolateImu reads the preceding endpoint even at exact t1.
    if (endpoint != imu.begin()) d.sample_min = std::min(d.sample_min, (endpoint-1)->t);
    if (std::abs(pim->preintegratedMeasurements().deltaTij()-(d.end-d.start)) > 1e-10)
      throw std::runtime_error("T11_IMU_DURATION_MISMATCH");
  } else {
    d.type = "prior";
    for (auto key : factor.keys()) {
      const auto symbol = Symbol(key);
      if ((symbol.chr()!='x' && symbol.chr()!='v' && symbol.chr()!='b') || symbol.index()!=0)
        throw std::runtime_error("T11_UNEXPECTED_NON_UWB_NON_IMU_FACTOR");
    }
  }
  return d;
}
inline void AuditGraph(const A17Graph& fixture, const PaperInputPlan& plan,
                      const T07ScenarioCache& cache, const std::string& out) {
  auto vals = output(out + "/initial_values.csv");
  vals << "phase,index,name,row,column,hex,bits\n"; values(vals, "INITIAL", fixture.initial);
  const auto identity = ComputeInferenceContentIdentity(fixture.graph, fixture.initial, fixture.ctx);
  WriteAtomicText(out + "/initial_identity.json", "{\"values\":\"" + identity.values_sha256 +
      "\",\"graph\":\"" + identity.graph_linearization_sha256 + "\"}\n");
  auto keyframes = output(out + "/keyframes.csv"); keyframes << "keyframe,time_s\n";
  for (const auto& k : plan.keyframes) {
    if (k.sensor_time > cache.imu.back().t) throw std::runtime_error("T11_KEYFRAME_AFTER_PREFIX");
    keyframes << k.keyframe_id << ',' << k.sensor_time << '\n';
  }
  auto deps = output(out + "/integration_dependencies.csv");
  deps << "factor,type,start_s,end_s,sample_min_s,sample_max_s\n";
  for (size_t i=0; i<fixture.graph.size(); ++i) {
    if (fixture.ranges.count(i)) continue;
    const auto d = FactorDependency(*fixture.graph[i], plan, cache.imu);
    deps << i << ',' << d.type << ',' << d.start << ',' << d.end << ',' << d.sample_min << ',' << d.sample_max << '\n';
  }
}
inline void FixedModel(const std::vector<GroupRecoverabilityScore>& scores,
    const SegmentRefitResult& refit, const PaperInputPlan& plan,
    const T07ScenarioCache& cache, const std::string& out) {
  const std::string root = out + "/diagnostic_fixed_model";
  fs::create_directory(root);
  auto summary = output(root + "/results.csv");
  summary << "group,H,status,reason,rows,columns,F_rank,N_rank,R_rank,eta,s_m,s_is_infinite,common_linearization\n";
  size_t applicable=0;
  for (const auto& score : scores) {
    if (score.group.start_time < 3 || score.group.end_time > 6) continue;
    ++applicable;
    const std::string group = root + "/" + score.group.group_id;
    fs::create_directory(group);
    auto deps = output(group + "/factor_dependencies.csv");
    deps << "factor,row_offset,row_count,type,start_s,end_s,sample_min_s,sample_max_s\n";
    std::vector<Dependency> dependencies;
    for (const auto& row : score.factor_rows) {
      Dependency d;
      if (row.factor_type == "uwb_range" || row.factor_type == "uwb_segment_range") {
        d.type = "uwb";
        auto obs = std::find_if(plan.observations.begin(),plan.observations.end(),
            [&](const auto& o) { return o.obs_id == row.obs_id; });
        if (obs == plan.observations.end()) throw std::runtime_error("T11_UNKNOWN_OBSERVATION");
        d.start=d.end=d.sample_min=d.sample_max=obs->sensor_time;
      } else d=FactorDependency(*refit.graph.at(row.original_factor_index),plan,cache.imu);
      dependencies.push_back(d);
      deps << row.original_factor_index << ',' << row.row_offset << ',' << row.row_count << ',' << d.type << ',' << d.start << ',' << d.end << ',' << d.sample_min << ',' << d.sample_max << '\n';
    }
    for (int h=0; h<=2; ++h) {
      const auto path = group + "/H" + std::to_string(h); fs::create_directory(path);
      std::vector<int> mapping(score.F_whitened.rows(), -1); int count=0;
      auto rows = output(path + "/row_mapping.csv"); rows << "new_row,common_row\n";
      for (size_t i=0;i<dependencies.size();++i) {
        const auto& d=dependencies[i];
        if (d.end>6+h || d.sample_max>6+h) continue;
        const auto& r=score.factor_rows[i];
        for (size_t j=0;j<r.row_count;++j) {
          mapping.at(r.row_offset+j)=count;
          rows << count++ << ',' << r.row_offset+j << '\n';
        }
      }
      std::vector<Eigen::Triplet<double>> triplets;
      for (int i=0;i<score.F_whitened.outerSize();++i)
        for (Eigen::SparseMatrix<double>::InnerIterator it(score.F_whitened,i);it;++it)
          if (mapping.at(it.row())>=0) triplets.emplace_back(mapping[it.row()],it.col(),it.value());
      Eigen::SparseMatrix<double> F(count,score.F_whitened.cols()); F.setFromTriplets(triplets.begin(),triplets.end());
      Eigen::MatrixXd G(count,score.G_whitened.cols());
      for (size_t i=0;i<mapping.size();++i) if (mapping[i]>=0) G.row(mapping[i])=score.G_whitened.row(i);
      const auto result=ComputeSparseRecoverability(F,G);
      WriteSparseCsv(path+"/F.csv",F); WriteDenseCsv(path+"/G.csv",G);
      WriteDenseCsv(path+"/N.csv",result.N); WriteDenseCsv(path+"/R.csv",result.R);
      auto spectrum=output(path+"/spectrum.csv"); spectrum << "matrix,index,eigenvalue\n";
      for (const auto& named : std::vector<std::pair<std::string,Eigen::MatrixXd>>{{"N",result.N},{"R",result.R}}) {
        if (!named.second.size()) continue;
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(named.second);
        if (eigen.info()!=Eigen::Success) throw std::runtime_error("T11_SPECTRUM_FAILED");
        for (int i=0;i<eigen.eigenvalues().size();++i) spectrum << named.first << ',' << i << ',' << eigen.eigenvalues()[i] << '\n';
      }
      auto audit=output(path+"/audit.csv");
      audit << "rank_certified,zero_columns,sigma_min_lower,rank_threshold_upper,orthogonality_residual,orthogonality_tolerance,R_PD_threshold,projection_floor\n"
        << result.frozen_rank_certified << ',' << result.exact_zero_columns << ',' << result.rank_certificate_sigma_min_lower << ',' << result.rank_threshold_upper << ',' << result.orthogonality_residual << ',' << result.orthogonality_tolerance << ',' << result.R_rank_pd_threshold << ',' << result.projection_residual_floor << '\n';
      summary << score.group.group_id << ',' << h << ',' << RecoverabilityStatusName(result.status) << ',' << result.reason << ',' << count << ',' << F.cols() << ',' << result.frozen_F_rank << ',' << result.N_rank << ',' << result.R_rank << ',';
      if (result.valid_score()) summary << result.eta << ',' << result.s_m;
      else summary << "NA,NA";
      summary << ',' << result.s_is_infinite << ',' << score.linearization_id << '\n';
    }
  }
  WriteAtomicText(root+"/status.json", "{\"label\":\"RQ4_FUTURE_COMMON_LINEARIZATION_DIAGNOSTIC\",\"applicable_groups\":"+std::to_string(applicable)+",\"status\":\""+(applicable?"COMPUTED":"NO_APPLICABLE_HISTORY_GROUP")+"\"}\n");
}
} // namespace t11
