#include "a18_optimizer.h"
#include "a19_r01_status.h"

#include "uifgo/nlos_refit.h"
#include "uifgo/nlos_scoring.h"
#include "uifgo/paper_pose_prior_factor.h"
#include "uifgo/uwb_factor.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace uifgo;

namespace {

constexpr const char* kIdentity =
    "a19-policy-sha256:r01-certified-mixed-engineering-fixture";

struct Fixture {
  Config cfg;
  PaperInputPlan plan;
  NonlinearFactorGraph graph;
  Values values;
  std::vector<FactorMeta> metadata;
  SupportPartition support;
};

bool BitsEqual(double a, double b) {
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

void Require(bool condition, const std::string& reason) {
  if (!condition) throw std::runtime_error(reason);
}

Fixture MakeMixedFixture() {
  Fixture fixture;
  fixture.cfg.calib_lever = false;
  fixture.cfg.calib_anchor = false;
  fixture.cfg.calib_range_bias = false;
  fixture.cfg.calib_td = false;
  fixture.cfg.lever_arm_init = Point3(0.12, -0.04, 0.03);
  fixture.cfg.anchors = {
      {1, Point3(-4.0, -3.0, 1.0), 0.05},
      {2, Point3(5.0, -3.0, 2.0), 0.05},
  };
  fixture.cfg.fixed_beta_by_link = {{"7:1", 0.125}, {"7:2", -0.08}};
  fixture.plan.keyframes = {{0, 0, 0.0, 7}, {1, 1, 1.0, 7}};

  fixture.support.schema = "t06_automatic_support_v1";
  fixture.support.provider = "automatic_discovery";
  fixture.support.input_plan_hash = "engineering-input-plan-sha256";
  fixture.support.discovery_context_hash = "engineering-context-sha256";
  fixture.support.solver_config_hash = kIdentity;
  fixture.support.discovery_snapshot_hash = "engineering-snapshot";
  fixture.support.partition_hash = "engineering-partition";
  SupportSegment segment;
  segment.segment_id = "candidate-anchor-1";
  segment.tag_id = 7;
  segment.anchor_id = 1;
  segment.segment_ordinal = 0;
  segment.start_time = 0.0;
  segment.end_time = 1.0;
  segment.duration = 1.0;
  segment.short_support_debug = false;
  segment.parent_segment_ids = {"candidate-anchor-1-parent"};

  const std::vector<Pose3> truth = {
      Pose3(Rot3::RzRyRx(0.08, -0.04, 0.15), Point3(1.0, 1.5, 0.6)),
      Pose3(Rot3::RzRyRx(0.10, -0.03, 0.20), Point3(1.4, 1.8, 0.7)),
  };
  auto pose_noise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
  auto velocity_noise = noiseModel::Isotropic::Sigma(3, 0.05);
  auto bias_noise = noiseModel::Isotropic::Sigma(6, 0.05);
  std::uint64_t obs_id = 91000;
  for (size_t k = 0; k < truth.size(); ++k) {
    fixture.graph.add(boost::make_shared<PaperPosePriorFactor>(
        X(k), truth[k], pose_noise));
    const Vector3 zero_velocity = Vector3::Zero();
    fixture.graph.addPrior(V(k), zero_velocity, velocity_noise);
    fixture.graph.addPrior(B(k), imuBias::ConstantBias(), bias_noise);
    fixture.values.insert(
        X(k), truth[k].retract((Vector(6) << 0.004, -0.003, 0.002,
                               0.006, -0.005, 0.004).finished()));
    fixture.values.insert(V(k), Vector3(0.001, -0.001, 0.002));
    fixture.values.insert(B(k), imuBias::ConstantBias());
    for (const auto& anchor : fixture.cfg.anchors) {
      const bool candidate = anchor.id == 1;
      const double beta = fixture.cfg.fixed_beta_by_link.at(
          std::string("7:") + std::to_string(anchor.id));
      const double geometric =
          (truth[k].transformFrom(fixture.cfg.lever_arm_init) - anchor.pos)
              .norm();
      const double raw = geometric + beta + (candidate ? 0.4 : 0.0);
      const size_t factor_index = fixture.graph.size();
      auto factor = MakeUwbFactor(X(k), 0, 0, 0, anchor.pos,
                                  fixture.cfg.lever_arm_init, raw, 0.05,
                                  false, false, false, beta);
      fixture.graph.add(factor);
      fixture.metadata.push_back(
          {factor_index, obs_id, "uwb_range",
           std::vector<Key>(factor->keys().begin(), factor->keys().end())});
      ObservationRecord record;
      record.obs_id = obs_id;
      record.sensor_time = static_cast<double>(k);
      record.tag_id = 7;
      record.anchor_id = anchor.id;
      record.raw_range = raw;
      record.valid = true;
      record.planned = true;
      record.keyframe_id = k;
      record.nominal_sigma = 0.05;
      fixture.plan.observations.push_back(record);
      if (candidate) segment.obs_ids.push_back(obs_id);
      ++obs_id;
    }
  }
  segment.observation_count = segment.obs_ids.size();
  fixture.support.segments.push_back(std::move(segment));
  return fixture;
}

RefitOptions Options() {
  RefitOptions options;
  options.max_refit_iterations = 20;
  options.lm_max_iterations = 50;
  options.lm_relative_tolerance = 1e-6;
  options.lm_absolute_tolerance = 1e-8;
  return options;
}

int RunMixedCertified(const std::string& out) {
  Require(fs::create_directory(out), "OUTPUT_EXISTS");
  {
    auto manifest = output(out + "/diagnostic_manifest.json");
    manifest << "{\"schema\":\"A19_STAGE1_STAGE2_SCORE_DIAGNOSTIC_ONLY\","
                "\"role\":\"development\",\"consumable\":false}\n";
  }
  bool formal_reader_rejected = false;
  try {
    (void)ReadStage2CacheManifest(out + "/diagnostic_manifest.json");
  } catch (const std::exception&) {
    formal_reader_rejected = true;
  }
  Require(formal_reader_rejected, "FORMAL_READER_ACCEPTED_DIAGNOSTIC");
  Fixture fixture = MakeMixedFixture();
  size_t total_calls = 0, total_trials = 0, total_accepted = 0;
  bool saw_initial_zero_c = false, saw_live_nonzero_c = false;
  auto audit = output(out + "/range_metadata.csv");
  audit << "outer,factor_index,obs_id,candidate,fixed_beta,segment_c,"
           "conditional_beta,residual\n";

  DevelopmentStage2Request request;
  request.policy = "PAPER_CERTIFIED_PAIR_REDUCTION_V1";
  request.role = "development";
  request.implementation_identity = kIdentity;
  request.conditional_navigation = [&]
      (size_t outer, const NonlinearFactorGraph& graph, const Values& values,
       const CheckedLmOptions& options,
       const std::vector<DevelopmentRefitRangeConstant>& ranges) {
    Require(ranges.size() == 4, "MIXED_RANGE_COVERAGE");
    std::set<size_t> factor_indices;
    std::set<std::uint64_t> obs_ids;
    size_t candidate_count = 0, reference_count = 0;
    for (const auto& range : ranges) {
      Require(factor_indices.insert(range.factor_index).second,
              "DUPLICATE_FACTOR_INDEX");
      Require(obs_ids.insert(range.obs_id).second, "DUPLICATE_OBS_ID");
      Require(range.factor_index < graph.size(), "RANGE_FACTOR_BOUNDS");
      Require(BitsEqual(range.conditional_beta,
                        range.fixed_beta + range.segment_amplitude),
              "BETA_PLUS_C_NOT_BITWISE");
      auto factor = boost::dynamic_pointer_cast<NoiseModelFactor>(
          graph.at(range.factor_index));
      Require(static_cast<bool>(factor), "RANGE_FACTOR_TYPE");
      Require(factor->keys() == KeyVector{range.pose_key}, "RANGE_POSE_KEY");
      const Vector actual_error = factor->unwhitenedError(values);
      Require(actual_error.size() == 1 &&
                  BitsEqual(actual_error[0],
                            range.expected_unwhitened_residual),
              "RANGE_RESIDUAL_IDENTITY");
      if (range.candidate) {
        ++candidate_count;
        Require(BitsEqual(range.fixed_beta, 0.125), "CANDIDATE_BETA");
        if (outer == 1 && BitsEqual(range.segment_amplitude, 0.0))
          saw_initial_zero_c = true;
        if (outer > 1 && range.segment_amplitude > 0.0)
          saw_live_nonzero_c = true;
      } else {
        ++reference_count;
        Require(BitsEqual(range.fixed_beta, -0.08), "REFERENCE_BETA");
        Require(BitsEqual(range.segment_amplitude, 0.0), "REFERENCE_C_ZERO");
      }
      audit << outer << ',' << range.factor_index << ',' << range.obs_id << ','
            << range.candidate << ',' << range.fixed_beta << ','
            << range.segment_amplitude << ',' << range.conditional_beta << ','
            << range.expected_unwhitened_residual << '\n';
    }
    audit.flush();
    Require(candidate_count == 2 && reference_count == 2,
            "MIXED_RANGE_CLASS_COUNTS");
    auto converted = std::vector<DevelopmentRangeConstant>();
    converted.reserve(ranges.size());
    for (const auto& range : ranges)
      converted.push_back({range.factor_index, range.pose_key, range.anchor,
                           range.lever, range.measurement, range.sigma,
                           range.conditional_beta, range.obs_id});
    auto result = runCertified(outer, graph, values, options, converted,
                               out + "/outer" + std::to_string(outer));
    total_calls += result.convergence.iterate_call_count;
    total_trials += result.convergence.lambda_trial_count;
    total_accepted += result.convergence.accepted_update_count;
    return result;
  };

  const auto refit = SegmentRefitter(Options()).RunDevelopmentStage2(
      fixture.graph, fixture.values, fixture.metadata, fixture.plan,
      fixture.cfg, fixture.support, &request);
  Require(refit.converged(), "REFIT_FAILED:" + refit.reason);
  Require(saw_initial_zero_c && saw_live_nonzero_c,
          "LIVE_NONZERO_C_UPDATE_NOT_OBSERVED");
  Require(total_calls > 0 && total_trials > 0 && total_accepted > 0,
          "CERTIFIED_SOLVE_OR_NATIVE_RETRACT_NOT_EXERCISED");
  Require(refit.values.exists(C(0)) && refit.values.at<double>(C(0)) > 0.0,
          "FINAL_LIVE_C_MISSING");
  size_t candidate_factors = 0, reference_factors = 0;
  for (const auto& meta : refit.factor_metadata) {
    candidate_factors += meta.factor_type == "uwb_segment_range";
    reference_factors += meta.factor_type == "uwb_range";
  }
  Require(candidate_factors == 2 && reference_factors == 2,
          "FINAL_JOINT_FACTOR_COUNTS");

  const auto scores = ScoreRefitRecoverability(
      refit, fixture.support, fixture.plan, fixture.cfg);
  Require(scores.size() == 1, "SCORING_GROUP_COUNT");
  const auto& score = scores.front();
  Require(score.eligible && score.valid_score_exported,
          "SCORING_UNAVAILABLE:" + score.status + ":" +
              score.numerical.reason);
  size_t scored_candidates = 0, scored_references = 0;
  for (const auto& row : score.factor_rows) {
    scored_candidates += row.factor_type == "uwb_segment_range";
    scored_references += row.factor_type == "uwb_range";
  }
  Require(scored_candidates == 2 && scored_references == 2,
          "COMMON_REFERENCE_OR_GROUP_FACTOR_MASK");
  Require(score.segment_fit.size() == 1 &&
              std::isfinite(score.segment_fit.front().gamma) &&
              std::isfinite(score.numerical.eta) &&
              (std::isfinite(score.numerical.s_m) ||
               score.numerical.s_is_infinite),
          "ETA_S_GAMMA_SEMANTICS");

  auto summary = output(out + "/summary.json");
  summary << "{\"schema\":\"A19_R01_MIXED_CERTIFIED_FIXTURE_V1\","
             "\"converged\":true,\"calls\":" << total_calls
          << ",\"trials\":" << total_trials
          << ",\"accepted\":" << total_accepted
          << ",\"stage2_outers\":" << refit.iterations.size()
          << ",\"candidate_factors\":2,\"reference_factors\":2,"
             "\"live_c_m\":" << refit.values.at<double>(C(0))
          << ",\"eta\":" << score.numerical.eta
          << ",\"s_m\":" << score.numerical.s_m
          << ",\"gamma\":" << score.segment_fit.front().gamma
          << ",\"linearization_id\":\"" << score.linearization_id
          << "\"}\n";
  return 0;
}

int RunConstructorFailure(const std::string& out) {
  Require(fs::create_directory(out), "OUTPUT_EXISTS");
  Fixture fixture = MakeMixedFixture();
  std::vector<DiscoveryObservation> snapshot;
  for (const auto& obs : fixture.plan.observations)
    snapshot.push_back({obs.obs_id, obs.tag_id, obs.anchor_id, 0, 0,
                        obs.sensor_time, 400.0, 0.0});
  const size_t candidate_observations = a19r01::WriteStage1Checkpoint(
      out, snapshot, fixture.support);
  Require(candidate_observations == 2 &&
              fs::exists(out + "/stage1/support_snapshot.csv") &&
              fs::exists(out + "/stage1/partition.csv") &&
              fs::exists(out + "/stage1/partition_identity.json") &&
              fs::exists(out + "/pipeline_status.json"),
          "STAGE1_CHECKPOINT_NOT_PERSISTED_BEFORE_CONSTRUCTOR");
  std::vector<DevelopmentRangeConstant> incomplete;
  // One candidate-looking entry cannot cover the other reference/candidate
  // ranges.  LiveGraph must fail in its constructor before iterate().
  const auto& meta = fixture.metadata.front();
  const auto& obs = fixture.plan.observations.front();
  incomplete.push_back({meta.factor_index, X(obs.keyframe_id),
                        fixture.cfg.anchors.front().pos,
                        fixture.cfg.lever_arm_init, obs.raw_range,
                        obs.nominal_sigma, 0.125, obs.obs_id});
  LevenbergMarquardtParams params;
  params.relativeErrorTol = 0;
  params.setLinearSolverType("SEQUENTIAL_CHOLESKY");
  bool failed = false;
  try {
    CertifiedLm optimizer(fixture.graph, fixture.values, params, incomplete,
                          out + "/optimizer");
    (void)optimizer;
  } catch (const std::exception& e) {
    failed = std::string(e.what()).find("UNKNOWN_FACTOR") != std::string::npos;
  }
  Require(failed, "CONSTRUCTOR_FAILURE_NOT_OBSERVED");
  a19r01::Counts zero;
  a19r01::WriteFailure(out,
      "NUMERIC_REFERENCE_UNSUPPORTED:UNKNOWN_FACTOR", "CONVERGED",
      "FAILED_CONSTRUCTOR_BEFORE_ITERATE", zero, zero);
  return 0;
}

int RunPartialFailure(const std::string& out) {
  Require(fs::create_directory(out), "OUTPUT_EXISTS");
  Fixture fixture = MakeMixedFixture();
  std::vector<DevelopmentRangeConstant> ranges;
  size_t range_ordinal = 0;
  for (const auto& meta : fixture.metadata) {
    const auto& obs = fixture.plan.observations.at(range_ordinal++);
    const auto anchor = std::find_if(
        fixture.cfg.anchors.begin(), fixture.cfg.anchors.end(),
        [&](const AnchorConfig& item) { return item.id == obs.anchor_id; });
    Require(anchor != fixture.cfg.anchors.end(), "PARTIAL_ANCHOR");
    const double beta = fixture.cfg.fixed_beta_by_link.at(
        std::string("7:") + std::to_string(obs.anchor_id));
    ranges.push_back({meta.factor_index, X(obs.keyframe_id), anchor->pos,
                      fixture.cfg.lever_arm_init, obs.raw_range,
                      obs.nominal_sigma, beta, obs.obs_id});
  }
  CheckedLmOptions options;
  options.max_iterations = 1;
  options.relative_tolerance = 1e-6;
  options.absolute_tolerance = 1e-8;
  options.policy = ConditionalLmPolicy::
      GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2;
  auto result = runCertified(1, fixture.graph, fixture.values, options, ranges,
                             out + "/completed_block");
  Require(result.convergence.iterate_call_count > 0 &&
              result.convergence.lambda_trial_count > 0,
          "PARTIAL_CERTIFIED_WORK_NOT_EXECUTED");
  a19r01::Counts stage1;
  a19r01::Counts stage2;
  stage2.calls = result.convergence.iterate_call_count;
  stage2.trials = result.convergence.lambda_trial_count;
  stage2.accepted = result.convergence.accepted_update_count;
  stage2.rejected = result.convergence.rejected_lambda_trial_count;
  stage2.exact = false;
  a19r01::WriteFailure(out, "ENGINEERING_EXCEPTION_AFTER_COMPLETED_BLOCK",
                       "CONVERGED", "FAILED_PARTIAL_EXECUTION",
                       stage1, stage2);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("USAGE");
    const std::string mode = argv[1];
    if (mode == "mixed-certified") return RunMixedCertified(argv[2]);
    if (mode == "constructor-failure") return RunConstructorFailure(argv[2]);
    if (mode == "partial-failure") return RunPartialFailure(argv[2]);
    throw std::runtime_error("UNKNOWN_MODE");
  } catch (const std::exception& e) {
    std::cerr << a18::exceptionStatus(e) << ':' << e.what() << '\n';
    return 2;
  }
}
