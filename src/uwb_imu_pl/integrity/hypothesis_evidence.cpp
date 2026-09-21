#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"

#include "uwb_imu_pl/estimation/candidate_worker_pool.hpp"
#include "uwb_imu_pl/estimation/numerical_work_counters.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

namespace uwb_imu_pl {
namespace {

void hashBytes(std::uint64_t* hash, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= bytes[i];
    *hash *= 1099511628211ULL;
  }
}

template <class Derived>
void hashMatrix(std::uint64_t* hash, const Eigen::MatrixBase<Derived>& matrix) {
  const Eigen::Index rows = matrix.rows(), columns = matrix.cols();
  hashBytes(hash, &rows, sizeof(rows));
  hashBytes(hash, &columns, sizeof(columns));
  for (Eigen::Index column = 0; column < columns; ++column)
    for (Eigen::Index row = 0; row < rows; ++row) {
      const double value = matrix(row, column);
      hashBytes(hash, &value, sizeof(value));
    }
}

struct WorkDelta {
  std::uint64_t eigen = 0;
  std::uint64_t svd = 0;
  std::uint64_t ldlt = 0;
  std::uint64_t low_dim = 0;
  std::uint64_t generic = 0;
};

void publish(const WorkDelta& work) {
  NumericalWorkCounters::faultGramEigen(work.eigen);
  NumericalWorkCounters::faultGramSvd(work.svd);
  NumericalWorkCounters::faultGramLdlt(work.ldlt);
  NumericalWorkCounters::lowDimFaultGram(work.low_dim);
  NumericalWorkCounters::genericFaultGramFallback(work.generic);
}

void failDimension(FaultHypothesisV2* hypothesis, FaultModeEvidence* evidence,
                   int physical_dimension) {
  auto& monitor = hypothesis->monitorability;
  monitor.parameter_dimension = 0;
  monitor.physical_parameter_dimension = physical_dimension;
  monitor.reason = "fault map dimension/non-finite gate failed";
  hypothesis->monitored = false;
  evidence->monitorability = monitor;
}

void fillEvidenceMonitor(const Eigen::VectorXd& eigenvalues,
                         int dimension, int physical_dimension,
                         const HypothesisEvaluationConfig& config,
                         FaultHypothesisV2* hypothesis,
                         FaultModeEvidence* evidence) {
  auto& monitor = hypothesis->monitorability;
  monitor.parameter_dimension = dimension;
  monitor.physical_parameter_dimension = physical_dimension;
  const Eigen::VectorXd singular =
      eigenvalues.cwiseMax(0.0).cwiseSqrt().reverse();
  monitor.sigma_max = singular.size() ? singular(0) : 0.0;
  const double rank_gate = config.rank_tolerance *
      std::max(1.0, monitor.sigma_max);
  monitor.rank = static_cast<int>((singular.array() > rank_gate).count());
  monitor.sigma_min = monitor.rank > 0 ? singular(monitor.rank - 1) : 0.0;
  monitor.condition_number = monitor.sigma_min > 0.0
      ? monitor.sigma_max / monitor.sigma_min
      : std::numeric_limits<double>::infinity();
  monitor.monitorable = monitor.rank == dimension &&
      monitor.sigma_min >= config.min_fault_gram_sigma &&
      monitor.condition_number <= config.max_fault_gram_condition;
  monitor.reason = monitor.monitorable ? "" : "fault subspace is unmonitorable";
  hypothesis->monitored = monitor.monitorable;
  evidence->monitorability = monitor;
}

void fillPlMonitor(const Eigen::VectorXd& singular, int dimension,
                   FrozenHypothesisPlEntry* entry) {
  auto& monitor = entry->monitorability;
  monitor.parameter_dimension = dimension;
  const double largest = singular.size() ? singular(0) : 0.0;
  const double gate = 1e-10 * std::max(1.0, largest);
  monitor.rank = static_cast<int>((singular.array() > gate).count());
  const double smallest = monitor.rank > 0 ? singular(monitor.rank - 1) : 0.0;
  monitor.sigma_min = std::sqrt(std::max(0.0, smallest));
  monitor.sigma_max = std::sqrt(std::max(0.0, largest));
  monitor.condition_number = smallest > 0.0
      ? largest / smallest : std::numeric_limits<double>::infinity();
  monitor.monitorable = monitor.rank == dimension;
  if (!monitor.monitorable) {
    monitor.reason = "remaining post-FDE fault is unmonitorable";
  }
}

void analyzeDynamic(const Eigen::MatrixXd& input_gram,
                    const Eigen::VectorXd& score,
                    const Eigen::MatrixXd* protected_fault,
                    int physical_dimension, double all_in,
                    double squared_detector_threshold,
                    const HypothesisEvaluationConfig& config,
                    FaultHypothesisV2* hypothesis,
                    FaultModeEvidence* evidence,
                    FrozenHypothesisPlEntry* pl_entry,
                    WorkDelta* work) {
  ++work->generic;
  const int dimension = input_gram.rows();
  if (dimension <= 0 || input_gram.cols() != dimension ||
      score.size() != dimension || !input_gram.allFinite() ||
      !score.allFinite()) {
    failDimension(hypothesis, evidence, physical_dimension);
    return;
  }
  const Eigen::MatrixXd gram = 0.5 * (input_gram + input_gram.transpose());
  if (config.retain_detailed_results) evidence->fault_gram = gram;
  ++work->eigen;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(gram);
  if (eigen.info() != Eigen::Success) {
    hypothesis->monitorability.reason = "fault Gram eigensolve failed";
    hypothesis->monitored = false;
    evidence->monitorability = hypothesis->monitorability;
    return;
  }
  fillEvidenceMonitor(eigen.eigenvalues(), dimension, physical_dimension,
                      config, hypothesis, evidence);
  if (pl_entry) {
    ++work->svd;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(gram);
    fillPlMonitor(svd.singularValues(), dimension, pl_entry);
  }
  const bool need_solve = hypothesis->monitored ||
      (pl_entry && pl_entry->monitorability.monitorable);
  if (!need_solve) return;
  ++work->ldlt;
  Eigen::LDLT<Eigen::MatrixXd> solve(gram);
  const bool positive = solve.info() == Eigen::Success && solve.isPositive();
  if (pl_entry) pl_entry->gram_spd = positive;
  if (!positive) {
    if (hypothesis->monitored) {
      hypothesis->monitorability.monitorable = false;
      hypothesis->monitorability.reason = "fault Gram solve failed";
      hypothesis->monitored = false;
      evidence->monitorability = hypothesis->monitorability;
    }
    return;
  }
  if (hypothesis->monitored) {
    const Eigen::VectorXd estimated = solve.solve(score);
    if (config.retain_detailed_results) evidence->estimated_fault = estimated;
    evidence->explained_energy = std::max(0.0, score.dot(estimated));
    evidence->conditioned_statistic = std::max(
        0.0, all_in - evidence->explained_energy);
    evidence->log_evidence = 0.5 * evidence->explained_energy;
    evidence->plausible = evidence->conditioned_statistic <=
        squared_detector_threshold +
            config.plausible_conditioned_statistic_margin;
  }
  if (pl_entry && pl_entry->monitorability.monitorable && protected_fault &&
      protected_fault->rows() == 3 && protected_fault->cols() == dimension &&
      protected_fault->allFinite()) {
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::VectorXd response = protected_fault->row(axis).transpose();
      pl_entry->protected_slopes(axis) = std::sqrt(std::max(
          0.0, response.dot(solve.solve(response))));
    }
    pl_entry->monitorability.protected_slopes = pl_entry->protected_slopes;
    pl_entry->valid = pl_entry->protected_slopes.allFinite();
    // A3 (G9): the audit export reads evidence.monitorability; copy the values
    // that were just computed for the frozen PL entry instead of leaving the
    // struct default (+inf).  Diagnostics only; the PL path above is unchanged.
    evidence->monitorability.protected_slopes = pl_entry->protected_slopes;
  }
}

template <int Dimension>
void analyzeFixed(const Eigen::Matrix<double, Dimension, Dimension>& input_gram,
                  const Eigen::Matrix<double, Dimension, 1>& score,
                  const Eigen::Matrix<double, 3, Dimension>& protected_fault,
                  int physical_dimension, double all_in,
                  double squared_detector_threshold,
                  const HypothesisEvaluationConfig& config,
                  FaultHypothesisV2* hypothesis,
                  FaultModeEvidence* evidence,
                  FrozenHypothesisPlEntry* pl_entry,
                  WorkDelta* work) {
  ++work->low_dim;
  const auto gram = (0.5 * (input_gram + input_gram.transpose())).eval();
  if (!gram.allFinite() || !score.allFinite() || !protected_fault.allFinite()) {
    failDimension(hypothesis, evidence, physical_dimension);
    return;
  }
  if (config.retain_detailed_results) evidence->fault_gram = gram;
  ++work->eigen;
  Eigen::SelfAdjointEigenSolver<
      Eigen::Matrix<double, Dimension, Dimension>> eigen(gram);
  if (eigen.info() != Eigen::Success) {
    hypothesis->monitorability.reason = "fault Gram eigensolve failed";
    hypothesis->monitored = false;
    evidence->monitorability = hypothesis->monitorability;
    return;
  }
  fillEvidenceMonitor(eigen.eigenvalues(), Dimension, physical_dimension,
                      config, hypothesis, evidence);
  if (pl_entry) {
    ++work->svd;
    Eigen::JacobiSVD<Eigen::Matrix<double, Dimension, Dimension>> svd(gram);
    fillPlMonitor(svd.singularValues(), Dimension, pl_entry);
  }
  const bool need_solve = hypothesis->monitored ||
      (pl_entry && pl_entry->monitorability.monitorable);
  if (!need_solve) return;
  ++work->ldlt;
  Eigen::LDLT<Eigen::Matrix<double, Dimension, Dimension>> solve(gram);
  const bool positive = solve.info() == Eigen::Success && solve.isPositive();
  if (pl_entry) pl_entry->gram_spd = positive;
  if (!positive) {
    if (hypothesis->monitored) {
      hypothesis->monitorability.monitorable = false;
      hypothesis->monitorability.reason = "fault Gram solve failed";
      hypothesis->monitored = false;
      evidence->monitorability = hypothesis->monitorability;
    }
    return;
  }
  if (hypothesis->monitored) {
    const Eigen::Matrix<double, Dimension, 1> estimated = solve.solve(score);
    if (config.retain_detailed_results) evidence->estimated_fault = estimated;
    evidence->explained_energy = std::max(0.0, score.dot(estimated));
    evidence->conditioned_statistic = std::max(
        0.0, all_in - evidence->explained_energy);
    evidence->log_evidence = 0.5 * evidence->explained_energy;
    evidence->plausible = evidence->conditioned_statistic <=
        squared_detector_threshold +
            config.plausible_conditioned_statistic_margin;
  }
  if (pl_entry && pl_entry->monitorability.monitorable) {
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Matrix<double, Dimension, 1> response =
          protected_fault.row(axis).transpose();
      pl_entry->protected_slopes(axis) = std::sqrt(std::max(
          0.0, response.dot(solve.solve(response))));
    }
    pl_entry->monitorability.protected_slopes = pl_entry->protected_slopes;
    pl_entry->valid = pl_entry->protected_slopes.allFinite();
    // A3 (G9): mirror into the evidence record for the audit export.
    evidence->monitorability.protected_slopes = pl_entry->protected_slopes;
  }
}

struct ModeDescriptor {
  FaultModeId id;
  SensorType sensor = SensorType::Unknown;
  int physical_dimension = 0;
  int dimension = 0;
  Eigen::Index offset = 0;
};

bool useEffectiveBasis(const FaultModeBasis& mode) {
  return mode.effective_basis_certified &&
      mode.effective_parameter_dimension > 0 &&
      mode.effective_parameter_basis.rows() == mode.parameter_dimension &&
      mode.effective_parameter_basis.cols() ==
          mode.effective_parameter_dimension;
}

std::vector<FaultModeEvidence> evaluateContiguous(
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    const HypothesisEvaluationConfig& config,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared,
    CandidateWorkerPool* worker_pool) {
  std::vector<FaultModeEvidence> results(hypotheses->size());
  std::map<std::uint64_t, std::size_t> block_index;
  std::vector<Eigen::Index> block_offsets(window.blocks.size());
  Eigen::Index row_offset = 0;
  for (std::size_t index = 0; index < window.blocks.size(); ++index) {
    block_index.emplace(window.blocks[index].group_id.value(), index);
    block_offsets[index] = row_offset;
    row_offset += window.blocks[index].residual_whitened.size();
  }
  std::vector<ModeDescriptor> descriptors;
  descriptors.reserve(modes.size());
  std::map<std::uint64_t, std::size_t> mode_index;
  Eigen::Index total_columns = 0;
  for (const auto& mode : modes) {
    ModeDescriptor descriptor;
    descriptor.id = mode.id;
    descriptor.sensor = mode.sensor;
    descriptor.physical_dimension = mode.parameter_dimension;
    descriptor.dimension = useEffectiveBasis(mode)
        ? mode.effective_parameter_dimension : mode.parameter_dimension;
    descriptor.offset = total_columns;
    total_columns += descriptor.dimension;
    mode_index.emplace(mode.id.value(), descriptors.size());
    descriptors.push_back(descriptor);
  }
  Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(window.H.rows(), total_columns);
  std::vector<bool> mode_valid(modes.size(), true);
  for (std::size_t mode_number = 0; mode_number < modes.size(); ++mode_number) {
    const auto& mode = modes[mode_number];
    const auto& descriptor = descriptors[mode_number];
    mode_valid[mode_number] = descriptor.dimension > 0;
    for (const auto& item : mode.raw_group_maps) {
      const auto found = block_index.find(item.first.value());
      if (found == block_index.end()) {
        mode_valid[mode_number] = false;
        break;
      }
      const auto& block = window.blocks[found->second];
      if (item.second.rows() != block.residual_raw.size() ||
          item.second.cols() != mode.parameter_dimension) {
        mode_valid[mode_number] = false;
        break;
      }
      const Eigen::MatrixXd whitened = block.whitener * item.second;
      dense.block(block_offsets[found->second], descriptor.offset,
                  item.second.rows(), descriptor.dimension) =
          useEffectiveBasis(mode)
              ? whitened * mode.effective_parameter_basis : whitened;
    }
  }
  if (!dense.allFinite()) return results;
  const Eigen::MatrixXd normal_cross = window.H.transpose() * dense;
  Eigen::MatrixXd rhs(window.H.cols(), 3 + total_columns);
  rhs.leftCols<3>() = window.protected_state_map.transpose();
  rhs.rightCols(total_columns) = normal_cross;
  const Eigen::MatrixXd solved = solveFrozenInformation(
      window.square_root.get(), *window.numerics, window.base_information, rhs);
  if (solved.rows() != window.H.cols() || solved.cols() != rhs.cols() ||
      !solved.allFinite()) return results;
  const Eigen::MatrixXd covariance_modes = solved.rightCols(total_columns);
  const Eigen::MatrixXd protected_modes =
      window.protected_state_map * covariance_modes;
  const Eigen::VectorXd scores = dense.transpose() * window.numerics->parity;
  // B1: one implicit Q^T application gives Z = Q2^T D for every fault mode.
  // Z^T Z equals the fault Gram below (verified by COV-02 and NUM-01), and the
  // per-hypothesis block of Z supplies the rank/condition audit that B2 needs;
  // the numeric Gram itself stays the shared normal-equation form so this
  // batch reproduces the P2 numbers exactly.
  Eigen::MatrixXd all_mode_response;
  if (window.square_root && window.square_root->usable()) {
    Eigen::MatrixXd z_response;
    window.square_root->faultResponse(dense, nullptr, &z_response);
    if (z_response.rows() > 0 && z_response.cols() == total_columns &&
        z_response.allFinite()) {
      all_mode_response = std::move(z_response);
    }
  }
  const Eigen::MatrixXd all_mode_gram =
      dense.transpose() * dense - normal_cross.transpose() * covariance_modes;
  auto context = std::make_shared<FrozenHypothesisNumerics>();
  context->window_id = window.id;
  context->version = window.version;
  context->window_content_fingerprint = window.numerics->content_fingerprint;
  context->numerical_contract_fingerprint =
      window.numerics->numerical_contract_fingerprint;
  context->hypothesis_fingerprint = hypothesisSetFingerprint(*hypotheses);
  context->fault_mode_fingerprint = faultModeSetFingerprint(modes);
  context->fault_model_policy_fingerprint =
      config.fault_model_policy_fingerprint;
  context->protected_covariance = window.protected_state_map * solved.leftCols<3>();
  context->pl_entries.resize(hypotheses->size());
  context->mode_count = modes.size();
  context->hypothesis_count = hypotheses->size();

  // One contiguous GEMM pair materializes every self and pair cross term.
  // This is transient window work (not retained by the shared context) and
  // replaces tens of thousands of tiny per-hypothesis products.

  auto crossBlock = [&](const ModeDescriptor& left,
                        const ModeDescriptor& right) -> Eigen::MatrixXd {
    return all_mode_gram.block(left.offset, right.offset,
                               left.dimension, right.dimension);
  };

  auto evaluate_range = [&](std::size_t begin, std::size_t end) {
    WorkDelta work;
    for (std::size_t hypothesis_index = begin;
         hypothesis_index < end; ++hypothesis_index) {
      auto& hypothesis = (*hypotheses)[hypothesis_index];
      auto& evidence = results[hypothesis_index];
      auto& pl_entry = context->pl_entries[hypothesis_index];
      evidence.hypothesis = hypothesis.id;
      evidence.all_in_statistic = window.numerics->statistic;
      pl_entry.hypothesis = hypothesis.id;
      int dimension = 0;
      int physical_dimension = 0;
      bool found_all = !hypothesis.modes.empty();
      std::vector<std::size_t> parts;
      parts.reserve(hypothesis.modes.size());
      for (const auto id : hypothesis.modes) {
        const auto found = mode_index.find(id.value());
        if (found == mode_index.end() || !mode_valid[found->second]) {
          found_all = false;
          break;
        }
        const auto& descriptor = descriptors[found->second];
        dimension += descriptor.dimension;
        physical_dimension += descriptor.physical_dimension;
        parts.push_back(found->second);
      }
      if (!found_all || dimension <= 0) {
        failDimension(&hypothesis, &evidence, physical_dimension);
        continue;
      }
      // B1 audit: detection-space response of this hypothesis, classified from
      // the small SVD of Z_h = Q2^T D_h (never from the squared normal form).
      // Applied to every hypothesis; the added work is one small SVD whose size
      // is (m - rank) x dimension, counted as square_root_qt_columns.
      if (all_mode_response.cols() > 0 && config.enable_shared_context) {
        std::vector<Eigen::MatrixXd> z_parts;
        int width = 0;
        for (const auto part : parts) {
          const auto& descriptor = descriptors[part];
          if (descriptor.offset + descriptor.dimension >
              all_mode_response.cols()) continue;
          z_parts.push_back(all_mode_response.middleCols(
              descriptor.offset, descriptor.dimension));
          width += descriptor.dimension;
        }
        if (width == dimension) {
          Eigen::MatrixXd z_h(all_mode_response.rows(), dimension);
          int offset = 0;
          for (const auto& part : z_parts) {
            z_h.middleCols(offset, part.cols()) = part;
            offset += static_cast<int>(part.cols());
          }
          Eigen::MatrixXd g_h(3, dimension);
          offset = 0;
          for (const auto part : parts) {
            const auto& descriptor = descriptors[part];
            g_h.middleCols(offset, descriptor.dimension) =
                protected_modes.middleCols(descriptor.offset,
                                           descriptor.dimension);
            offset += descriptor.dimension;
          }
          pl_entry.z_classification = classifyDetectionResponse(
              z_h, g_h,
              window.numerics
                  ? window.numerics->numerical_contract.rank_tolerance : 1e-10,
              &pl_entry.z_smallest_singular_value, &pl_entry.z_condition,
              &pl_entry.z_rank);
        }
      }
      if (dimension <= 3 && parts.size() <= 2) {
        Eigen::Matrix3d gram = Eigen::Matrix3d::Zero();
        Eigen::Vector3d score = Eigen::Vector3d::Zero();
        Eigen::Matrix3d protected_fault = Eigen::Matrix3d::Zero();
        int left_offset = 0;
        for (std::size_t left = 0; left < parts.size(); ++left) {
          const auto& left_descriptor = descriptors[parts[left]];
          score.segment(left_offset, left_descriptor.dimension) =
              scores.segment(left_descriptor.offset, left_descriptor.dimension);
          protected_fault.middleCols(left_offset, left_descriptor.dimension) =
              protected_modes.middleCols(left_descriptor.offset,
                                          left_descriptor.dimension);
          int right_offset = 0;
          for (std::size_t right = 0; right < parts.size(); ++right) {
            const auto& right_descriptor = descriptors[parts[right]];
            if (left == right) {
              gram.block(left_offset, right_offset,
                         left_descriptor.dimension, right_descriptor.dimension) =
                  crossBlock(left_descriptor, right_descriptor);
            } else {
              gram.block(left_offset, right_offset,
                         left_descriptor.dimension, right_descriptor.dimension) =
                  crossBlock(left_descriptor, right_descriptor);
            }
            right_offset += right_descriptor.dimension;
          }
          left_offset += left_descriptor.dimension;
        }
        if (dimension == 1) {
          analyzeFixed<1>(gram.topLeftCorner<1, 1>(), score.head<1>(),
                          protected_fault.leftCols<1>(), physical_dimension,
                          window.numerics->statistic, squared_detector_threshold,
                          config, &hypothesis, &evidence, &pl_entry, &work);
        } else if (dimension == 2) {
          analyzeFixed<2>(gram.topLeftCorner<2, 2>(), score.head<2>(),
                          protected_fault.leftCols<2>(), physical_dimension,
                          window.numerics->statistic, squared_detector_threshold,
                          config, &hypothesis, &evidence, &pl_entry, &work);
        } else {
          analyzeFixed<3>(gram, score, protected_fault, physical_dimension,
                          window.numerics->statistic, squared_detector_threshold,
                          config, &hypothesis, &evidence, &pl_entry, &work);
        }
      } else {
        Eigen::MatrixXd gram(dimension, dimension);
        Eigen::VectorXd score(dimension);
        Eigen::MatrixXd protected_fault(3, dimension);
        int left_offset = 0;
        for (std::size_t left = 0; left < parts.size(); ++left) {
          const auto& left_descriptor = descriptors[parts[left]];
          score.segment(left_offset, left_descriptor.dimension) =
              scores.segment(left_descriptor.offset, left_descriptor.dimension);
          protected_fault.middleCols(left_offset, left_descriptor.dimension) =
              protected_modes.middleCols(left_descriptor.offset,
                                          left_descriptor.dimension);
          int right_offset = 0;
          for (std::size_t right = 0; right < parts.size(); ++right) {
            const auto& right_descriptor = descriptors[parts[right]];
            if (left == right && left_descriptor.dimension <= 3) {
              gram.block(left_offset, right_offset, left_descriptor.dimension,
                         right_descriptor.dimension) =
                  crossBlock(left_descriptor, right_descriptor);
            } else {
              gram.block(left_offset, right_offset, left_descriptor.dimension,
                         right_descriptor.dimension) =
                  crossBlock(left_descriptor, right_descriptor);
            }
            right_offset += right_descriptor.dimension;
          }
          left_offset += left_descriptor.dimension;
        }
        analyzeDynamic(gram, score, &protected_fault, physical_dimension,
                       window.numerics->statistic, squared_detector_threshold,
                       config, &hypothesis, &evidence, &pl_entry, &work);
      }
    }
    publish(work);
  };

  const std::size_t active_workers = std::max<std::size_t>(1,
      std::min(config.hypothesis_workers, hypotheses->size()));
  if (worker_pool && active_workers > 1) {
    context->worker_blocks = active_workers;
    NumericalWorkCounters::hypothesisParallelBlocks(active_workers);
    worker_pool->run(active_workers, active_workers,
        [&](std::size_t block, std::size_t, RankUpdateScratch&) {
          const std::size_t begin = hypotheses->size() * block / active_workers;
          const std::size_t end = hypotheses->size() * (block + 1) / active_workers;
          evaluate_range(begin, end);
        });
  } else {
    context->worker_blocks = 1;
    evaluate_range(0, hypotheses->size());
  }
  for (const auto& hypothesis : *hypotheses) {
    int dimension = 0;
    for (const auto id : hypothesis.modes) {
      const auto found = mode_index.find(id.value());
      if (found != mode_index.end()) dimension += descriptors[found->second].dimension;
    }
    if (dimension == 1) ++context->dimension_one_count;
    else if (dimension == 2) ++context->dimension_two_count;
    else if (dimension == 3) ++context->dimension_three_count;
    else ++context->dimension_other_count;
    if (dimension >= 1 && dimension <= 3 && hypothesis.modes.size() <= 2) {
      ++context->low_dimensional_count;
    } else {
      ++context->generic_fallback_count;
    }
  }
  context->bytes = sizeof(*context) +
      context->pl_entries.capacity() * sizeof(FrozenHypothesisPlEntry);
  context->valid = context->protected_covariance.allFinite() &&
      context->pl_entries.size() == hypotheses->size();
  if (!context->valid) context->reason = "shared hypothesis context is incomplete";
  if (shared) *shared = std::move(context);
  return results;
}

std::vector<FaultModeEvidence> evaluateMapped(
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    const HypothesisEvaluationConfig& config,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared) {
  struct Projection {
    Eigen::MatrixXd dense;
    Eigen::MatrixXd normal_cross;
    Eigen::MatrixXd covariance_cross;
    Eigen::MatrixXd protected_fault;
    Eigen::VectorXd score;
    int physical_dimension = 0;
  };
  std::map<std::uint64_t, Projection> projected_modes;
  std::map<std::uint64_t, int> block_offsets;
  int row_offset = 0;
  for (const auto& block : window.blocks) {
    block_offsets[block.group_id.value()] = row_offset;
    row_offset += block.residual_whitened.size();
  }
  for (const auto& mode : modes) {
    const bool effective = useEffectiveBasis(mode);
    const int dimension = effective ? mode.effective_parameter_dimension
                                    : mode.parameter_dimension;
    Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(window.H.rows(), dimension);
    bool valid = dimension > 0;
    for (const auto& item : mode.raw_group_maps) {
      const auto offset = block_offsets.find(item.first.value());
      const auto block = std::find_if(window.blocks.begin(), window.blocks.end(),
          [&](const LinearizedFactorBlock& value) {
            return value.group_id == item.first;
          });
      if (offset == block_offsets.end() || block == window.blocks.end() ||
          item.second.rows() != block->residual_raw.size() ||
          item.second.cols() != mode.parameter_dimension) {
        valid = false;
        break;
      }
      const Eigen::MatrixXd whitened = block->whitener * item.second;
      dense.block(offset->second, 0, item.second.rows(), dimension) =
          effective ? whitened * mode.effective_parameter_basis : whitened;
    }
    if (!valid || !dense.allFinite()) continue;
    Projection projection;
    projection.dense = std::move(dense);
    projection.normal_cross = window.H.transpose() * projection.dense;
    projection.physical_dimension = mode.parameter_dimension;
    projected_modes.emplace(mode.id.value(), std::move(projection));
  }
  Eigen::Index solve_columns = config.enable_shared_context ? 3 : 0;
  for (const auto& item : projected_modes)
    solve_columns += item.second.normal_cross.cols();
  Eigen::MatrixXd combined_cross(window.H.cols(), solve_columns);
  Eigen::Index solve_offset = 0;
  if (config.enable_shared_context) {
    combined_cross.leftCols<3>() = window.protected_state_map.transpose();
    solve_offset = 3;
  }
  for (const auto& item : projected_modes) {
    const Eigen::Index columns = item.second.normal_cross.cols();
    combined_cross.middleCols(solve_offset, columns) = item.second.normal_cross;
    solve_offset += columns;
  }
  const Eigen::MatrixXd combined_solutions = solveFrozenInformation(
      *window.numerics, window.base_information, combined_cross);
  if (combined_solutions.rows() != window.H.cols() ||
      combined_solutions.cols() != solve_columns ||
      !combined_solutions.allFinite()) return {};

  auto context = std::make_shared<FrozenHypothesisNumerics>();
  if (config.enable_shared_context) {
    context->window_id = window.id;
    context->version = window.version;
    context->window_content_fingerprint = window.numerics->content_fingerprint;
    context->numerical_contract_fingerprint =
        window.numerics->numerical_contract_fingerprint;
    context->hypothesis_fingerprint = hypothesisSetFingerprint(*hypotheses);
    context->fault_mode_fingerprint = faultModeSetFingerprint(modes);
    context->fault_model_policy_fingerprint =
        config.fault_model_policy_fingerprint;
    context->protected_covariance =
        window.protected_state_map * combined_solutions.leftCols<3>();
    context->pl_entries.resize(hypotheses->size());
    context->mode_count = modes.size();
    context->hypothesis_count = hypotheses->size();
  }
  solve_offset = config.enable_shared_context ? 3 : 0;
  for (auto& item : projected_modes) {
    const Eigen::Index columns = item.second.normal_cross.cols();
    item.second.covariance_cross =
        combined_solutions.middleCols(solve_offset, columns);
    item.second.score = item.second.dense.transpose() * window.numerics->parity;
    if (config.enable_shared_context) {
      item.second.protected_fault =
          window.protected_state_map * item.second.covariance_cross;
    }
    solve_offset += columns;
  }

  std::vector<FaultModeEvidence> results;
  results.reserve(hypotheses->size());
  std::map<std::pair<std::uint64_t, std::uint64_t>, Eigen::MatrixXd> pair_grams;
  WorkDelta work;
  for (std::size_t hypothesis_index = 0;
       hypothesis_index < hypotheses->size(); ++hypothesis_index) {
    auto& hypothesis = (*hypotheses)[hypothesis_index];
    FaultModeEvidence evidence;
    evidence.hypothesis = hypothesis.id;
    evidence.all_in_statistic = window.numerics->statistic;
    int columns = 0;
    int physical_dimension = 0;
    bool found_all = !hypothesis.modes.empty();
    std::vector<std::pair<std::uint64_t, const Projection*>> parts;
    for (const auto id : hypothesis.modes) {
      const auto found = projected_modes.find(id.value());
      if (found == projected_modes.end()) {
        found_all = false;
        break;
      }
      columns += found->second.dense.cols();
      physical_dimension += found->second.physical_dimension;
      parts.emplace_back(id.value(), &found->second);
    }
    if (!found_all) {
      failDimension(&hypothesis, &evidence, physical_dimension);
      results.push_back(std::move(evidence));
      continue;
    }
    Eigen::MatrixXd gram = Eigen::MatrixXd::Zero(columns, columns);
    Eigen::VectorXd score = Eigen::VectorXd::Zero(columns);
    Eigen::MatrixXd protected_fault = Eigen::MatrixXd::Zero(3, columns);
    int left_column = 0;
    for (std::size_t left = 0; left < parts.size(); ++left) {
      const int left_size = parts[left].second->dense.cols();
      score.segment(left_column, left_size) = parts[left].second->score;
      if (config.enable_shared_context) {
        protected_fault.middleCols(left_column, left_size) =
            parts[left].second->protected_fault;
      }
      int right_column = 0;
      for (std::size_t right = 0; right < parts.size(); ++right) {
        const int right_size = parts[right].second->dense.cols();
        const std::pair<std::uint64_t, std::uint64_t> key{
            std::min(parts[left].first, parts[right].first),
            std::max(parts[left].first, parts[right].first)};
        auto found = pair_grams.find(key);
        if (found == pair_grams.end()) {
          const auto& first = projected_modes.at(key.first);
          const auto& second = projected_modes.at(key.second);
          Eigen::MatrixXd value = first.dense.transpose() * second.dense -
              first.normal_cross.transpose() * second.covariance_cross;
          found = pair_grams.emplace(key, std::move(value)).first;
        }
        gram.block(left_column, right_column, left_size, right_size) =
            parts[left].first <= parts[right].first
                ? found->second : found->second.transpose();
        right_column += right_size;
      }
      left_column += left_size;
    }
    FrozenHypothesisPlEntry* entry = nullptr;
    if (config.enable_shared_context) {
      entry = &context->pl_entries[hypothesis_index];
      entry->hypothesis = hypothesis.id;
    }
    analyzeDynamic(gram, score,
                   config.enable_shared_context ? &protected_fault : nullptr,
                   physical_dimension, window.numerics->statistic,
                   squared_detector_threshold, config, &hypothesis,
                   &evidence, entry, &work);
    results.push_back(std::move(evidence));
  }
  publish(work);
  if (config.enable_shared_context) {
    context->generic_fallback_count = hypotheses->size();
    context->dimension_other_count = hypotheses->size();
    context->worker_blocks = 1;
    context->bytes = sizeof(*context) +
        context->pl_entries.capacity() * sizeof(FrozenHypothesisPlEntry);
    context->valid = context->protected_covariance.allFinite();
    if (!context->valid) context->reason = "shared hypothesis context is incomplete";
    if (shared) *shared = std::move(context);
  }
  return results;
}



}  // namespace

int classifyDetectionResponse(const Eigen::MatrixXd& z_h,
                              const Eigen::MatrixXd& g_h,
                              double rank_tolerance,
                              double* smallest_singular_value,
                              double* condition, int* rank_out) {
  if (z_h.cols() == 0 || z_h.rows() == 0) return 0;
  // ThinV is required: the structural nullspace test needs V (the default
  // JacobiSVD option computes singular values only).
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(z_h, Eigen::ComputeThinV);
  const Eigen::VectorXd singular = svd.singularValues();
  const double largest = singular.size() ? singular(0) : 0.0;
  const double gate = rank_tolerance * std::max(1.0, largest);
  int rank = 0;
  for (int index = 0; index < singular.size(); ++index) {
    if (singular(index) > gate) ++rank;
  }
  const int columns = static_cast<int>(z_h.cols());
  if (rank_out) *rank_out = rank;
  if (smallest_singular_value) {
    *smallest_singular_value = rank > 0 ? singular(rank - 1) : 0.0;
  }
  if (condition) {
    *condition = rank > 0 && singular(rank - 1) > 0.0
        ? largest / singular(rank - 1)
        : std::numeric_limits<double>::infinity();
  }
  if (rank == columns) return 1;  // full rank
  // The band check uses the first *discarded* singular value: a value that is
  // below the gate but well above zero means the rank decision is a tolerance
  // artifact rather than a structural nullspace.
  const double excluded = rank < singular.size() ? singular(rank) : 0.0;
  if (excluded > gate * 0.01) return 4;  // numerically indistinguishable
  double residual = 0.0;
  if (g_h.cols() == columns && rank > 0) {
    const Eigen::MatrixXd retained =
        svd.matrixV().leftCols(rank) *
        svd.matrixV().leftCols(rank).transpose() * g_h.transpose();
    residual = (g_h.transpose() - retained).cwiseAbs().maxCoeff();
  } else if (g_h.size() > 0) {
    residual = g_h.cwiseAbs().maxCoeff();
  }
  const double response_scale = std::max(1.0, g_h.size() ? g_h.cwiseAbs().maxCoeff() : 0.0);
  return residual <= 1e-9 * response_scale ? 2 : 3;
}

std::uint64_t hypothesisSetFingerprint(
    const std::vector<FaultHypothesisV2>& hypotheses) {
  std::uint64_t hash = 1469598103934665603ULL;
  const std::size_t count = hypotheses.size();
  hashBytes(&hash, &count, sizeof(count));
  for (const auto& hypothesis : hypotheses) {
    const auto id = hypothesis.id.value();
    hashBytes(&hash, &id, sizeof(id));
    hashBytes(&hash, &hypothesis.prior_probability_bound,
              sizeof(hypothesis.prior_probability_bound));
    hashBytes(&hash, &hypothesis.p_md_allocation,
              sizeof(hypothesis.p_md_allocation));
    hashBytes(&hash, &hypothesis.hmi_allocation,
              sizeof(hypothesis.hmi_allocation));
    for (const auto mode : hypothesis.modes) {
      const auto value = mode.value();
      hashBytes(&hash, &value, sizeof(value));
    }
  }
  return hash;
}

std::uint64_t faultModeSetFingerprint(
    const std::vector<FaultModeBasis>& modes) {
  std::uint64_t hash = 1469598103934665603ULL;
  const std::size_t count = modes.size();
  hashBytes(&hash, &count, sizeof(count));
  for (const auto& mode : modes) {
    const auto id = mode.id.value();
    const auto anchor = mode.anchor_id.value();
    const auto onset = mode.onset_time.value();
    hashBytes(&hash, &id, sizeof(id));
    hashBytes(&hash, &mode.kind, sizeof(mode.kind));
    hashBytes(&hash, &mode.sensor, sizeof(mode.sensor));
    hashBytes(&hash, mode.physical_source_id.data(),
              mode.physical_source_id.size());
    hashBytes(&hash, &anchor, sizeof(anchor));
    hashBytes(&hash, &mode.axis, sizeof(mode.axis));
    hashBytes(&hash, &mode.onset_epoch, sizeof(mode.onset_epoch));
    hashBytes(&hash, &onset, sizeof(onset));
    hashBytes(&hash, &mode.parameter_dimension,
              sizeof(mode.parameter_dimension));
    hashBytes(&hash, &mode.effective_parameter_dimension,
              sizeof(mode.effective_parameter_dimension));
    hashBytes(&hash, &mode.effective_basis_certified,
              sizeof(mode.effective_basis_certified));
    hashMatrix(&hash, mode.effective_parameter_basis);
    for (const auto& item : mode.raw_group_maps) {
      const auto group = item.first.value();
      hashBytes(&hash, &group, sizeof(group));
      hashMatrix(&hash, item.second);
    }
    for (const auto group : mode.affected_groups) {
      const auto value = group.value();
      hashBytes(&hash, &value, sizeof(value));
    }
    for (const auto measurement : mode.affected_measurements) {
      const auto value = measurement.value();
      hashBytes(&hash, &value, sizeof(value));
    }
  }
  return hash;
}

std::vector<HypothesisId> completePlausibleHypotheses(
    const std::vector<FaultHypothesisV2>& hypotheses,
    const std::vector<FaultModeEvidence>& evidence) {
  std::set<std::uint64_t> plausible;
  for (const auto& item : evidence) {
    if (item.plausible) plausible.insert(item.hypothesis.value());
  }
  std::vector<HypothesisId> complete;
  for (const auto& hypothesis : hypotheses) {
    if (plausible.count(hypothesis.id.value())) complete.push_back(hypothesis.id);
  }
  return complete;
}

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold,
    std::shared_ptr<const FrozenHypothesisNumerics>* shared,
    CandidateWorkerPool* worker_pool) const {
  if (!hypotheses) throw std::invalid_argument("hypotheses must not be null");
  if (shared) shared->reset();
  if (!window.model_valid || !window.numerics || !window.numerics->valid ||
      window.numerics->content_fingerprint != integrityWindowFingerprint(window) ||
      !window.numerics->information_factorization) return {};
  if (window.numerics->numerical_contract_fingerprint !=
      numericalContractFingerprint(config_.rank_tolerance,
                                   config_.max_condition_number)) {
    NumericalWorkCounters::numericalContractMismatch();
    return {};
  }
  if (modes.empty()) {
    std::vector<FaultModeEvidence> results;
    results.reserve(hypotheses->size());
    WorkDelta work;
    for (auto& hypothesis : *hypotheses) {
      FaultModeEvidence evidence;
      evidence.hypothesis = hypothesis.id;
      evidence.all_in_statistic = window.numerics->statistic;
      Eigen::MatrixXd gram;
      Eigen::VectorXd score;
      if (hypothesis.A.rows() == window.H.rows() && hypothesis.A.cols() > 0 &&
          hypothesis.A.allFinite()) {
        const Eigen::MatrixXd cross = window.H.transpose() * hypothesis.A;
        const Eigen::MatrixXd covariance_cross = solveFrozenInformation(
            window.square_root.get(), *window.numerics,
            window.base_information, cross);
        gram = hypothesis.A.transpose() * hypothesis.A -
            cross.transpose() * covariance_cross;
        score = hypothesis.A.transpose() * window.numerics->parity;
      }
      analyzeDynamic(gram, score, nullptr, hypothesis.A.cols(),
                     window.numerics->statistic, squared_detector_threshold,
                     config_, &hypothesis, &evidence, nullptr, &work);
      results.push_back(std::move(evidence));
    }
    publish(work);
    return results;
  }
  if (config_.enable_shared_context && config_.enable_low_dim_batch) {
    return evaluateContiguous(window, modes, hypotheses,
                              squared_detector_threshold, config_, shared,
                              worker_pool);
  }
  return evaluateMapped(window, modes, hypotheses, squared_detector_threshold,
                        config_, shared);
}

std::vector<FaultModeEvidence> HypothesisEvidenceEvaluator::evaluateAll(
    const LinearizedIntegrityWindow& window,
    std::vector<FaultHypothesisV2>* hypotheses,
    double squared_detector_threshold) const {
  return evaluateAll(window, {}, hypotheses, squared_detector_threshold,
                     nullptr, nullptr);
}

}  // namespace uwb_imu_pl
