// test/test_config.cpp — UT-1: ConfigLoader tests
#include "uifgo/config.h"
#include <gtest/gtest.h>
#include <fstream>
#include <boost/filesystem.hpp>

TEST(ConfigLoader, LoadSampleYaml) {
  // Write a temporary config
  std::string tmp_path = "/tmp/test_slam.yaml";
  std::ofstream f(tmp_path);
  f << "anchors:\n"
    << "  - {id: 101, pos: [0.0, 0.0, 0.0], prior_sigma: 0.08}\n"
    << "  - {id: 102, pos: [5.0, 0.0, 0.0], prior_sigma: 0.08}\n"
    << "extrinsics:\n"
    << "  lever_arm_init: [0.1, 0.0, -0.05]\n"
    << "  calib_lever: true\n"
    << "  lever_prior_sigma: 0.03\n"
    << "imu:\n"
    << "  sigma_a: 0.2\n"
    << "  sigma_g: 0.02\n"
    << "  gravity: 9.80\n"
    << "uwb:\n"
    << "  sigma_range: 0.15\n"
    << "  v_max: 2.0\n"
    << "  nlos_rssi_diff: 8.0\n"
    << "solver:\n"
    << "  lm_max_iter: 50\n";
  f.close();

  auto cfg = uifgo::ConfigLoader::Load(tmp_path);
  EXPECT_EQ(cfg.anchors.size(), 2u);
  EXPECT_EQ(cfg.anchors[0].id, 101);
  EXPECT_DOUBLE_EQ(cfg.anchors[0].pos.x(), 0.0);
  EXPECT_DOUBLE_EQ(cfg.anchors[1].pos.y(), 0.0);
  EXPECT_DOUBLE_EQ(cfg.lever_arm_init.x(), 0.1);
  EXPECT_TRUE(cfg.calib_lever);
  EXPECT_DOUBLE_EQ(cfg.lever_prior_sigma, 0.03);
  EXPECT_DOUBLE_EQ(cfg.sigma_a, 0.2);
  EXPECT_DOUBLE_EQ(cfg.sigma_g, 0.02);
  EXPECT_DOUBLE_EQ(cfg.gravity, 9.80);
  EXPECT_DOUBLE_EQ(cfg.sigma_range, 0.15);
  EXPECT_DOUBLE_EQ(cfg.v_max, 2.0);
  EXPECT_DOUBLE_EQ(cfg.nlos_rssi_diff, 8.0);
  EXPECT_EQ(cfg.lm_max_iter, 50);
  EXPECT_EQ(cfg.discovery_conditional_navigation_policy,
            "GTSAM_CHECK_ONLY_V1");
}

TEST(ConfigLoader, ConditionalNavigationPolicyRejectsUnknownAndNonAutomaticUse) {
  const std::string path = "/tmp/test_conditional_navigation_policy.yaml";
  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: disabled\n"
      << "  discovery_conditional_navigation_policy: UNKNOWN_V9\n";
  }
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);
  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: oracle_debug\n"
      << "  oracle_support: debug.yaml\n"
      << "  discovery_conditional_navigation_policy: "
         "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1\n";
  }
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);
  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: automatic_discovery\n"
      << "  score_recoverability: true\n"
      << "  lambda_l1: 0.07\n"
      << "  lambda_tv: 0.11\n"
      << "  gap_threshold_s: 1.0\n"
      << "  active_bias_min_m: 0.02\n"
      << "  change_point_min_m: 0.05\n"
      << "  merge_max_difference_m: 0.05\n"
      << "  discovery_short_min_count: 2\n"
      << "  discovery_short_min_duration_s: 0.01\n"
      << "  discovery_scaled_step_tolerance: 1.0e-6\n"
      << "  discovery_observation_bias_scale_m: 1.0\n"
      << "  discovery_conditional_navigation_policy: "
         "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2\n";
  }
  const auto v2 = uifgo::ConfigLoader::Load(path);
  EXPECT_EQ(v2.discovery_conditional_navigation_policy,
            "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2");
}

TEST(ConfigLoader, McdDatasetInterface) {
  std::string tmp_path = "/tmp/test_mcd_dataset.yaml";
  std::ofstream f(tmp_path);
  f << "dataset:\n"
    << "  interface: mcd\n"
    << "  imu_bag_path: imu.bag\n"
    << "  uwb_bag_path: uwb.bag\n"
    << "  gt_csv_path: gt.csv\n"
    << "initialization:\n"
    << "  use_imu_orientation: true\n"
    << "  imu_orientation_world: ned\n";
  f.close();

  auto cfg = uifgo::ConfigLoader::Load(tmp_path);
  EXPECT_EQ(cfg.data_interface, "mcd");
  EXPECT_EQ(cfg.imu_bag_path, "imu.bag");
  EXPECT_EQ(cfg.uwb_bag_path, "uwb.bag");
  EXPECT_EQ(cfg.gt_csv_path, "gt.csv");
  EXPECT_TRUE(cfg.use_imu_orientation_init);
  EXPECT_EQ(cfg.imu_orientation_world, "ned");
}

TEST(ConfigLoader, T07CacheInterfaceIsStrictAndUsesSensorTimeWindow) {
  const std::string path = "/tmp/test_t07_cache_dataset.yaml";
  {
    std::ofstream f(path);
    f << "dataset:\n"
      << "  interface: t07_cache\n"
      << "  cache_manifest: cache/input_manifest.json\n"
      << "  cache_start_s: 8.0\n"
      << "  cache_duration_s: 4.0\n";
  }
  const auto cfg = uifgo::ConfigLoader::Load(path);
  EXPECT_EQ(cfg.data_interface, "t07_cache");
  EXPECT_EQ(cfg.t07_cache_manifest, "cache/input_manifest.json");
  EXPECT_DOUBLE_EQ(cfg.t07_cache_start_s, 8.0);
  EXPECT_DOUBLE_EQ(cfg.t07_cache_duration_s, 4.0);

  for (const std::string& forbidden : {"recipe", "truth", "support"}) {
    std::ofstream f(path);
    f << "dataset:\n"
      << "  interface: t07_cache\n"
      << "  cache_manifest: cache/input_manifest.json\n"
      << "  " << forbidden << ": forbidden.yaml\n";
    f.close();
    EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error)
        << forbidden;
  }
}

TEST(ConfigLoader, DefaultValues) {
  std::string tmp_path = "/tmp/test_default.yaml";
  std::ofstream f(tmp_path);
  f << "anchors:\n"
    << "  - {id: 1, pos: [1.0, 2.0, 3.0]}\n";
  f.close();

  auto cfg = uifgo::ConfigLoader::Load(tmp_path);
  EXPECT_EQ(cfg.anchors.size(), 1u);
  EXPECT_TRUE(cfg.calib_lever);  // default is true
  EXPECT_DOUBLE_EQ(cfg.sigma_a, 0.1);
  EXPECT_DOUBLE_EQ(cfg.gravity, 9.81);
  EXPECT_DOUBLE_EQ(cfg.nlos_rssi_diff, 6.0);
  EXPECT_FALSE(cfg.score_recoverability);
}

TEST(ConfigLoader, RecoverabilityScoreIsExplicitOptIn) {
  const std::string path = "/tmp/test_score_recoverability.yaml";
  std::ofstream f(path);
  f << "nlos:\n"
    << "  score_recoverability: true\n";
  f.close();
  const auto cfg = uifgo::ConfigLoader::Load(path);
  EXPECT_TRUE(cfg.score_recoverability);
}

TEST(ConfigLoader, T08GateRequiresExplicitFiniteDevelopmentValues) {
  const std::string path = "/tmp/test_t08_gate.yaml";
  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: oracle_debug\n"
      << "  oracle_support: support.yaml\n"
      << "  score_recoverability: true\n"
      << "  final_inference_enabled: true\n"
      << "  tau_eta: 0.4\n"
      << "  tau_s_m: 0.8\n"
      << "  tau_gamma: 1.2\n"
      << "  gate_parameter_provenance: "
         "T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION\n";
  }
  const auto cfg = uifgo::ConfigLoader::Load(path);
  EXPECT_TRUE(cfg.final_inference_enabled);
  EXPECT_DOUBLE_EQ(cfg.gate_tau_eta, 0.4);
  EXPECT_DOUBLE_EQ(cfg.gate_tau_s_m, 0.8);
  EXPECT_DOUBLE_EQ(cfg.gate_tau_gamma, 1.2);

  for (const std::string& omitted : {"tau_eta", "tau_s_m", "tau_gamma",
                                    "gate_parameter_provenance"}) {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: oracle_debug\n"
      << "  oracle_support: support.yaml\n"
      << "  score_recoverability: true\n"
      << "  final_inference_enabled: true\n";
    if (omitted != "tau_eta") f << "  tau_eta: 0.4\n";
    if (omitted != "tau_s_m") f << "  tau_s_m: 0.8\n";
    if (omitted != "tau_gamma") f << "  tau_gamma: 1.2\n";
    if (omitted != "gate_parameter_provenance")
      f << "  gate_parameter_provenance: "
           "T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION\n";
    f.close();
    EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error)
        << omitted;
  }

  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: oracle_debug\n"
      << "  oracle_support: support.yaml\n"
      << "  score_recoverability: true\n"
      << "  final_inference_enabled: true\n"
      << "  tau_eta: .nan\n"
      << "  tau_s_m: 0.8\n"
      << "  tau_gamma: 1.2\n"
      << "  gate_parameter_provenance: "
         "T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION\n";
  }
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);

  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: oracle_debug\n"
      << "  oracle_support: support.yaml\n"
      << "  score_recoverability: true\n"
      << "  final_inference_enabled: true\n"
      << "  tau_eta: 0.4\n"
      << "  tau_s_m: 0.8\n"
      << "  tau_gamma: 1.2\n"
      << "  gate_parameter_provenance: T10_LOCKED\n";
  }
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);
}

TEST(ConfigLoader, AutomaticDiscoveryParametersAreExplicitAndOracleIsForbidden) {
  const std::string path = "/tmp/test_automatic_discovery.yaml";
  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: automatic_discovery\n"
      << "  score_recoverability: true\n"
      << "  lambda_l1: 0.07\n"
      << "  lambda_tv: 0.11\n"
      << "  gap_threshold_s: 1.0\n"
      << "  active_bias_min_m: 0.02\n"
      << "  change_point_min_m: 0.05\n"
      << "  merge_max_difference_m: 0.05\n"
      << "  discovery_short_min_count: 2\n"
      << "  discovery_short_min_duration_s: 0.01\n"
      << "  discovery_scaled_step_tolerance: 1.0e-6\n"
      << "  discovery_observation_bias_scale_m: 1.0\n"
      << "  rho_scale: 3.0\n"
      << "  admm_primal_abs_tolerance_m: 1.0e-7\n"
      << "  admm_dual_abs_tolerance_objective_per_m: 2.0e-7\n";
  }
  const auto cfg = uifgo::ConfigLoader::Load(path);
  EXPECT_EQ(cfg.nlos_mode, "automatic_discovery");
  EXPECT_DOUBLE_EQ(cfg.discovery_lambda_l1, 0.07);
  EXPECT_DOUBLE_EQ(cfg.discovery_lambda_tv, 0.11);
  EXPECT_DOUBLE_EQ(cfg.discovery_rho_scale, 3.0);
  EXPECT_DOUBLE_EQ(cfg.discovery_primal_abs_tolerance_m, 1.0e-7);
  EXPECT_DOUBLE_EQ(cfg.discovery_dual_abs_tolerance_objective_per_m, 2.0e-7);

  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: automatic_discovery\n"
      << "  oracle_support: ''\n"
      << "  score_recoverability: true\n"
      << "  lambda_l1: 0.07\n"
      << "  lambda_tv: 0.11\n"
      << "  gap_threshold_s: 1.0\n"
      << "  active_bias_min_m: 0.02\n"
      << "  change_point_min_m: 0.05\n"
      << "  merge_max_difference_m: 0.05\n"
      << "  discovery_short_min_count: 2\n"
      << "  discovery_short_min_duration_s: 0.01\n"
      << "  discovery_scaled_step_tolerance: 1.0e-6\n"
      << "  discovery_observation_bias_scale_m: 1.0\n";
  }
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);

  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: automatic_discovery\n"
      << "  score_recoverability: true\n";
  }
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);

  {
    std::ofstream f(path);
    f << "nlos:\n"
      << "  mode: automatic_discovery\n"
      << "  score_recoverability: false\n"
      << "  lambda_l1: 0.07\n"
      << "  lambda_tv: 0.11\n"
      << "  gap_threshold_s: 1.0\n"
      << "  active_bias_min_m: 0.02\n"
      << "  change_point_min_m: 0.05\n"
      << "  merge_max_difference_m: 0.05\n"
      << "  discovery_short_min_count: 2\n"
      << "  discovery_short_min_duration_s: 0.01\n"
      << "  discovery_scaled_step_tolerance: 1.0e-6\n"
      << "  discovery_observation_bias_scale_m: 1.0\n";
  }
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);
}

TEST(ConfigLoader, ImuAidedFdeHasIndependentStrictConfiguration) {
  const std::string path = "/tmp/test_imu_aided_fde.yaml";
  const auto write = [&](double probability, bool oracle, bool temporal) {
    std::ofstream f(path);
    f << "solver:\n  chi2_reject_prob: " << probability << "\n"
      << "nlos:\n  mode: imu_aided_fde\n"
      << "  score_recoverability: true\n";
    if (oracle) f << "  oracle_support: forbidden.yaml\n";
    if (temporal)
      f << "  gap_threshold_s: 1.0\n"
        << "  discovery_short_min_count: 2\n"
        << "  discovery_short_min_duration_s: 0.01\n";
  };
  write(0.99, false, true);
  const auto cfg = uifgo::ConfigLoader::Load(path);
  EXPECT_EQ(cfg.nlos_mode, "imu_aided_fde");
  EXPECT_DOUBLE_EQ(cfg.chi2_reject_prob, 0.99);

  write(0.95, false, true);
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);
  write(0.99, true, true);
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);
  write(0.99, false, false);
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);
}

TEST(ConfigLoader, FixedBetaByLink) {
  const std::string path = "/tmp/test_fixed_beta.yaml";
  std::ofstream f(path);
  f << "calibration:\n"
    << "  calib_range_bias: false\n"
    << "  fixed_beta_by_link:\n"
    << "    '7:101': 0.3\n";
  f.close();
  const auto cfg = uifgo::ConfigLoader::Load(path);
  EXPECT_DOUBLE_EQ(uifgo::FixedBetaForLink(cfg, 7, 101), 0.3);
  EXPECT_DOUBLE_EQ(uifgo::FixedBetaForLink(cfg, 7, 102), 0.0);
}

TEST(ConfigLoader, FixedAndOnlineBetaConflict) {
  const std::string path = "/tmp/test_fixed_online_beta_conflict.yaml";
  std::ofstream f(path);
  f << "calibration:\n"
    << "  calib_range_bias: true\n"
    << "  fixed_beta_by_link:\n"
    << "    '7:101': 0.3\n";
  f.close();
  EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error);
}

TEST(ConfigLoader, FixedBetaLinkKeyMustBeCanonical) {
  for (const std::string& bad_key : {"wrong_link", "7-101", "-7:101",
                                     "7:-101", "07:101", "7:0101",
                                     "7:101:2"}) {
    const std::string path = "/tmp/test_bad_fixed_beta_key.yaml";
    std::ofstream f(path);
    f << "calibration:\n"
      << "  fixed_beta_by_link:\n"
      << "    '" << bad_key << "': 0.3\n";
    f.close();
    EXPECT_THROW(uifgo::ConfigLoader::Load(path), std::runtime_error)
        << bad_key;
  }

  int tag_id = -1;
  int anchor_id = -1;
  EXPECT_TRUE(uifgo::ParseRangeLinkKey("7:101", &tag_id, &anchor_id));
  EXPECT_EQ(tag_id, 7);
  EXPECT_EQ(anchor_id, 101);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(ConfigLoader, A14ExplicitImuModelOnly) {
  const auto path = boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("a14-config-%%%%-%%%%.yaml");
  for (const auto& name : {"LEGACY_GTSAM_COMBINED_DEFAULT_V1",
                           "PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1"}) {
    { std::ofstream f(path.string()); f << "paper:\n  imu_covariance_model: " << name << '\n'; }
    EXPECT_EQ(uifgo::ImuCovarianceModelName(uifgo::ConfigLoader::Load(path.string()).paper_imu_covariance_model), std::string(name));
  }
  for (const auto& content : {"paper: {imu_covariance_model: BAD}",
                             "paper: {biasAccOmegaInt: 0}", "paper: 0"}) {
    { std::ofstream f(path.string()); f << content; }
    EXPECT_THROW(uifgo::ConfigLoader::Load(path.string()), std::runtime_error);
  }
  { std::ofstream f(path.string()); f << "{}"; }
  EXPECT_EQ(uifgo::ConfigLoader::Load(path.string()).paper_imu_covariance_model,
            uifgo::ImuCovarianceModel::LEGACY_GTSAM_COMBINED_DEFAULT_V1);
  boost::filesystem::remove(path);
}
