#include "uifgo/pl_bidirectional_provider.h"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

#include "uifgo/hash_utils.h"
#include "uifgo/pl_conditional_raim.h"

namespace uifgo {
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

constexpr size_t kBootstrapLastKeyframe = 4;
constexpr double kFixedLagEpochs = 200.0;
constexpr double kImuMaxGapS = 0.02;
constexpr double kRelinearizeThreshold = 0.1;
constexpr size_t kRelinearizeSkip = 200;

class ProductionFixedLagBackend final
    : public gtsam::IncrementalFixedLagSmoother {
 public:
  ProductionFixedLagBackend(double lag, const gtsam::ISAM2Params& params)
      : gtsam::IncrementalFixedLagSmoother(lag, params) {}
  const gtsam::ISAM2& isam() const { return isam_; }
};

std::string Bits(double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "binary64 required");
  std::memcpy(&bits, &value, sizeof(bits));
  std::ostringstream out;
  out << std::hex << bits;
  return out.str();
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

Eigen::Matrix<double, 15, 15> CurrentMarginal(const gtsam::ISAM2& isam,
                                               size_t epoch) {
  const gtsam::KeyVector keys{X(epoch), V(epoch), B(epoch)};
  const std::array<int, 3> offsets{0, 6, 9};
  const std::array<int, 3> dimensions{6, 3, 6};
  const auto bayes = CurrentCliqueClosure(isam, keys);
  gtsam::VectorValues zero;
  for (const auto& conditional : bayes) {
    for (auto item = conditional->begin(); item != conditional->end(); ++item) {
      if (!zero.exists(*item))
        zero.insert(*item,
                    Eigen::VectorXd::Zero(conditional->getDim(item)));
    }
  }
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Zero();
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
        (!state && include_constants))
      output.insert(key, initial.at(key));
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

void SeedFromPrediction(const ProductionFixedLagBackend& backend,
                        size_t epoch,
                        const std::vector<size_t>& factor_indices,
                        const gtsam::NonlinearFactorGraph& graph,
                        gtsam::Values* values) {
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

std::string ProductionPartitionHash(const SupportPartition& support) {
  std::ostringstream out;
  out << kPlBidirectionalProductionVersion << '\n'
      << support.provider << '\n' << support.partition_rule_version << '\n'
      << support.discovery_context_hash << '\n' << support.input_plan_hash
      << '\n' << support.source_hash << '\n';
  for (const auto& segment : support.segments) {
    out << segment.segment_ordinal << ',' << segment.segment_id << ','
        << segment.tag_id << ',' << segment.anchor_id << ','
        << Bits(segment.start_time) << ',' << Bits(segment.end_time) << ','
        << segment.observation_count << '\n';
    for (const auto id : segment.obs_ids) out << id << ',';
    out << '\n';
  }
  return "sha256:" + Sha256Hex(out.str());
}

}  // namespace

std::string PlBidirectionalParameterIdentity(const Config& cfg) {
  std::ostringstream out;
  out << kPlBidirectionalProductionVersion << '\n'
      << kPlPersistentCusumSignalVersion << '\n'
      << kPlPersistentCusumAlgorithmVersion << '\n'
      << kPlBackwardCusumAlgorithmVersion << '\n'
      << kPlBidirectionalIntersectionRule << '\n'
      << "forward_kappa=" << Bits(cfg.cusum_forward_kappa) << '\n'
      << "forward_h=" << Bits(cfg.cusum_forward_h) << '\n'
      << "backward_kappa=" << Bits(cfg.cusum_backward_kappa) << '\n'
      << "backward_h=" << Bits(cfg.cusum_backward_h) << '\n'
      << "gap=" << Bits(cfg.discovery_gap_threshold_s) << '\n'
      << "provenance=" << cfg.cusum_parameter_provenance << '\n'
      << "forward_calibration=" << kPlBidirectionalForwardCalibrationHash
      << '\n' << "backward_calibration="
      << kPlBidirectionalBackwardCalibrationHash << '\n'
      << "clean_split=" << kPlBidirectionalCleanSplitHash << '\n'
      << "clean_input=" << kPlBidirectionalCleanInputHash << '\n';
  return "plparams-sha256:" + Sha256Hex(out.str());
}

std::string PlBidirectionalProviderIdentity(
    const Config& cfg, const PaperInputPlan& plan,
    const PlBidirectionalProviderContext& context) {
  std::ostringstream out;
  out << kPlBidirectionalProductionProvider << '\n'
      << PlBidirectionalParameterIdentity(cfg) << '\n'
      << "signal_location=PRE_COMMIT_PRODUCTION_RAW_UWB_V1\n"
      << "bootstrap_last_keyframe=" << kBootstrapLastKeyframe << '\n'
      << "fixed_lag_epochs=" << kFixedLagEpochs << '\n'
      << "imu_gap=" << kImuMaxGapS << '\n'
      << "relinearize_threshold=" << kRelinearizeThreshold << '\n'
      << "relinearize_skip=" << kRelinearizeSkip << '\n'
      << plan.plan_sha256 << '\n' << context.source_hash << '\n'
      << context.config_hash << '\n' << context.calibration_hash << '\n'
      << context.solver_config_hash << '\n' << context.common_preparation_id
      << '\n' << context.physical_graph_hash << '\n'
      << context.initial_values_hash << '\n';
  return "plprovider-sha256:" + Sha256Hex(out.str());
}

PlBidirectionalProviderResult PlBidirectionalCusumSupportProvider::Run(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& initial_values,
    const std::vector<FactorMeta>& metadata, const PaperInputPlan& plan,
    const std::vector<ImuSample>& imu, const Config& cfg,
    const PlBidirectionalProviderContext& context) const {
  PlBidirectionalProviderResult result;
  result.detector_parameter_identity = PlBidirectionalParameterIdentity(cfg);
  result.provider_identity = PlBidirectionalProviderIdentity(cfg, plan, context);
  result.signal_identity =
      "plsignal-sha256:" + Sha256Hex(
          std::string(kPlPersistentCusumSignalVersion) +
          "\nPRE_COMMIT_PRODUCTION_RAW_UWB_V1\n" +
          result.provider_identity + "\n");
  try {
    if (plan.keyframes.size() < kBootstrapLastKeyframe + 2)
      throw std::invalid_argument("PL_CONDITIONAL_INSUFFICIENT_KEYFRAMES");
    if (graph.empty() || initial_values.empty())
      throw std::invalid_argument("PL_CONDITIONAL_EMPTY_GRAPH_OR_VALUES");

    struct GapEvent { double after_time; };
    std::vector<GapEvent> gaps;
    for (size_t i = 1; i < imu.size(); ++i) {
      const double gap = imu[i].t - imu[i - 1].t;
      if (!std::isfinite(gap) || gap <= 0.0)
        throw std::invalid_argument("PL_CONDITIONAL_IMU_TIME_INVALID");
      result.max_imu_gap_s = std::max(result.max_imu_gap_s, gap);
      if (gap > kImuMaxGapS + 1e-12) gaps.push_back({imu[i].t});
    }
    result.imu_gap_event_count = gaps.size();

    std::unordered_map<std::uint64_t, size_t> factor_by_obs;
    std::set<size_t> uwb_set;
    for (const auto& meta : metadata) {
      if (meta.factor_type != "uwb_range" ||
          !factor_by_obs.emplace(meta.obs_id, meta.factor_index).second)
        throw std::invalid_argument("PL_CONDITIONAL_FACTOR_METADATA_INVALID");
      uwb_set.insert(meta.factor_index);
    }
    std::vector<std::vector<size_t>> non_uwb(plan.keyframes.size());
    for (size_t i = 0; i < graph.size(); ++i) {
      if (uwb_set.count(i)) continue;
      const size_t epoch = FactorEpoch(graph.at(i));
      if (epoch >= non_uwb.size())
        throw std::invalid_argument("PL_CONDITIONAL_FACTOR_EPOCH_INVALID");
      non_uwb[epoch].push_back(i);
    }
    std::vector<std::vector<const ObservationRecord*>> groups(
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
    params.relinearizeThreshold = kRelinearizeThreshold;
    params.relinearizeSkip = kRelinearizeSkip;
    std::unique_ptr<ProductionFixedLagBackend> backend(
        new ProductionFixedLagBackend(kFixedLagEpochs, params));
    size_t next_gap = 0;
    for (size_t k = 0; k < groups.size(); ++k) {
      gtsam::NonlinearFactorGraph prediction;
      auto new_values = NewValuesAt(initial_values, k, k == 0);
      if (k > 0 && next_gap < gaps.size() &&
          gaps[next_gap].after_time <= plan.keyframes[k].sensor_time + 1e-12 &&
          gaps[next_gap].after_time >
              plan.keyframes[k - 1].sensor_time + 1e-12) {
        const auto previous = backend->calculateEstimate();
        const auto pose = previous.at<gtsam::Pose3>(X(k - 1));
        const auto velocity = previous.at<gtsam::Vector3>(V(k - 1));
        const auto bias =
            previous.at<gtsam::imuBias::ConstantBias>(B(k - 1));
        prediction = ReinitializationPriors(k, pose, velocity, bias);
        new_values.clear();
        new_values.insert(X(k), pose);
        new_values.insert(V(k), velocity);
        new_values.insert(B(k), bias);
        backend.reset(new ProductionFixedLagBackend(kFixedLagEpochs, params));
        do {
          ++next_gap;
        } while (next_gap < gaps.size() &&
                 gaps[next_gap].after_time <=
                     plan.keyframes[k].sensor_time + 1e-12);
      } else {
        AddFactors(&prediction, non_uwb[k], graph);
        SeedFromPrediction(*backend, k, non_uwb[k], graph, &new_values);
      }
      backend->update(prediction, new_values, Timestamps(new_values, k));
      if (groups[k].empty()) continue;

      std::set<int> anchors;
      std::vector<size_t> factor_indices;
      for (const auto* row : groups[k]) {
        if (!anchors.insert(row->anchor_id).second)
          throw std::invalid_argument(
              "PL_CONDITIONAL_DUPLICATE_ANCHOR_IN_GROUP");
        const auto found = factor_by_obs.find(row->obs_id);
        if (found == factor_by_obs.end())
          throw std::invalid_argument("PL_CONDITIONAL_OBS_FACTOR_MISSING");
        factor_indices.push_back(found->second);
      }

      if (k > kBootstrapLastKeyframe) {
        const auto estimate = backend->calculateEstimate();
        const size_t n = groups[k].size();
        PlConditionalInput input;
        input.prior_covariance = CurrentMarginal(backend->isam(), k);
        input.physical_jacobian = Eigen::MatrixXd::Zero(n, 15);
        input.physical_covariance = Eigen::MatrixXd::Zero(n, n);
        input.physical_innovation = Eigen::VectorXd::Zero(n);
        for (size_t i = 0; i < n; ++i) {
          const auto& factor = graph.at(factor_indices[i]);
          const auto noise_factor =
              boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
          if (!noise_factor || noise_factor->unwhitenedError(estimate).size() != 1 ||
              std::find(factor->keys().begin(), factor->keys().end(), X(k)) ==
                  factor->keys().end())
            throw std::runtime_error("PL_CONDITIONAL_FACTOR_MODEL_INVALID");
          const auto residual = noise_factor->unwhitenedError(estimate);
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
              !pose_jacobian.allFinite() ||
              noise_factor->noiseModel()->sigmas().size() != 1)
            throw std::runtime_error("PL_CONDITIONAL_POSE_JACOBIAN_INVALID");
          input.physical_jacobian.block<1, 6>(i, 0) = pose_jacobian;
          const double sigma = noise_factor->noiseModel()->sigmas()[0];
          if (std::abs(sigma - groups[k][i]->nominal_sigma) >
              1e-12 + 1e-10 * std::max(std::abs(sigma),
                                        std::abs(groups[k][i]->nominal_sigma)))
            throw std::runtime_error("PL_CONDITIONAL_SIGMA_IDENTITY_MISMATCH");
          input.physical_covariance(i, i) = sigma * sigma;
          input.physical_innovation[i] = -residual[0];
        }
        const Eigen::MatrixXd innovation_covariance =
            input.physical_jacobian * input.prior_covariance *
                input.physical_jacobian.transpose() +
            input.physical_covariance;
        const auto diagnostics = EvaluatePlConditionalRows(
            input.physical_innovation, innovation_covariance);
        if (diagnostics.rows.size() != n)
          throw std::runtime_error(
              "PL_CONDITIONAL_ROW_DIAGNOSTIC_DIMENSION_INVALID:" +
              diagnostics.status);
        for (size_t i = 0; i < n; ++i) {
          PlCusumInputRow row;
          row.timestamp = plan.keyframes[k].sensor_time;
          row.group_id = "pl-group-" + std::to_string(k);
          row.keyframe_id = k;
          row.source_order = i;
          row.tag_id = groups[k][i]->tag_id;
          row.anchor_id = groups[k][i]->anchor_id;
          row.obs_id = groups[k][i]->obs_id;
          row.conditional_z = diagnostics.rows[i].conditional_z;
          row.diagnostic_valid = diagnostics.rows[i].numerically_valid;
          result.signal_rows.push_back(std::move(row));
        }
      }

      gtsam::NonlinearFactorGraph commit;
      AddFactors(&commit, factor_indices, graph);
      if (!commit.empty()) backend->update(commit);
      result.committed_uwb_count += factor_indices.size();
    }

    PlCusumOptions forward_options;
    forward_options.kappa = cfg.cusum_forward_kappa;
    forward_options.threshold = cfg.cusum_forward_h;
    forward_options.gap_threshold_s = cfg.discovery_gap_threshold_s;
    forward_options.calibration_hash = kPlBidirectionalForwardCalibrationHash;
    forward_options.split_manifest_hash = kPlBidirectionalCleanSplitHash;
    result.forward = EvaluatePlPersistentCusum(result.signal_rows,
                                                forward_options);

    PlBackwardCusumOptions backward_options;
    backward_options.kappa = cfg.cusum_backward_kappa;
    backward_options.threshold = cfg.cusum_backward_h;
    backward_options.gap_threshold_s = cfg.discovery_gap_threshold_s;
    backward_options.original_clean_split_manifest_hash =
        kPlBidirectionalCleanSplitHash;
    backward_options.original_clean_input_hash =
        kPlBidirectionalCleanInputHash;
    backward_options.calibration_hash =
        kPlBidirectionalBackwardCalibrationHash;
    result.backward = EvaluatePlBackwardCusum(result.signal_rows,
                                               backward_options);
    if (!result.forward.valid || !result.backward.valid)
      throw std::runtime_error("PL_BIDIRECTIONAL_CORE_EVALUATION_FAILED");

    std::vector<PlBidirectionalMembershipRow> forward_rows;
    std::vector<PlBidirectionalMembershipRow> backward_rows;
    for (const auto& row : result.forward.trace) {
      PlBidirectionalMembershipRow item;
      item.input = row.input;
      item.forward_candidate = row.candidate;
      forward_rows.push_back(std::move(item));
    }
    for (const auto& row : result.backward.trace) {
      PlBidirectionalMembershipRow item;
      item.input = row.input;
      item.backward_candidate = row.backward_candidate;
      backward_rows.push_back(std::move(item));
    }
    result.final_support = IntersectPlCusumSupport(
        forward_rows, backward_rows, result.forward.detector_identity,
        result.backward.closure_identity, cfg.discovery_gap_threshold_s);
    if (!result.final_support.valid)
      throw std::runtime_error("PL_BIDIRECTIONAL_INTERSECTION_FAILED:" +
                               result.final_support.status);

    result.support_identity = result.final_support.support_identity;
    auto& support = result.final_support.support;
    support.provider = kPlBidirectionalProductionProvider;
    support.source_path.clear();
    support.input_plan_hash = plan.plan_sha256;
    support.source_hash = context.source_hash;
    support.config_hash = context.config_hash;
    support.calibration_hash = context.calibration_hash;
    support.solver_config_hash = context.solver_config_hash;
    support.discovery_context_hash = result.provider_identity;
    support.discovery_snapshot_hash = result.signal_identity;
    support.partition_hash = ProductionPartitionHash(support);
    result.valid = true;
    result.status = "SUCCESS_SUPPORT_FROZEN";
  } catch (const std::exception& error) {
    result.valid = false;
    result.status = error.what();
  }
  return result;
}

}  // namespace uifgo
