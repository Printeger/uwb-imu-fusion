#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

struct Occurrence {
  std::size_t previous_epoch = 0;
  std::size_t epoch = 0;
  TimestampNs begin;
  TimestampNs end;
  const UwbBatch* batch = nullptr;
  const PendingFactorGroup* uwb = nullptr;
  const PendingFactorGroup* imu = nullptr;
  const PendingFactorGroup* pose_bridge = nullptr;
  const PendingFactorGroup* bias_bridge = nullptr;
  const HistoricalEpochContext* history = nullptr;
};

const LinearizedFactorBlock* blockFor(const LinearizedIntegrityWindow& window,
                                      FactorGroupId id, int* row_offset = nullptr) {
  int offset = 0;
  for (const auto& block : window.blocks) {
    if (block.group_id == id) {
      if (row_offset) *row_offset = offset;
      return &block;
    }
    offset += block.residual_whitened.size();
  }
  return nullptr;
}

bool explicitlyMonitored(const LinearizedIntegrityWindow& window,
                         FactorGroupId id) {
  const auto found = std::find_if(window.factor_inventory.begin(),
      window.factor_inventory.end(), [&](const auto& entry) {
        return entry.group_id == id;
      });
  return found != window.factor_inventory.end() &&
      (found->disposition == FrozenFactorDisposition::ExplicitMeasurement ||
       found->disposition == FrozenFactorDisposition::PendingExplicit);
}

void finalizeEffectiveBasis(const LinearizedIntegrityWindow& window,
                            double rank_tolerance,
                            FaultModeBasis* mode) {
  if (!mode || mode->parameter_dimension <= 0) return;
  const int physical = mode->parameter_dimension;
  mode->effective_parameter_basis = Eigen::MatrixXd::Identity(physical, physical);
  mode->effective_parameter_dimension = physical;
  mode->effective_basis_certified = true;
  int rows = 0;
  for (const auto& item : mode->raw_group_maps) {
    const auto* block = blockFor(window, item.first);
    if (!block || item.second.rows() != block->residual_raw.size() ||
        item.second.cols() != physical) {
      mode->effective_basis_certified = false;
      return;
    }
    rows += item.second.rows();
  }
  if (rows == 0) {
    mode->effective_basis_certified = false;
    return;
  }
  Eigen::MatrixXd map(rows, physical);
  int offset = 0;
  for (const auto& item : mode->raw_group_maps) {
    const auto* block = blockFor(window, item.first);
    map.middleRows(offset, item.second.rows()) = block->whitener * item.second;
    offset += item.second.rows();
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(map, Eigen::ComputeFullV);
  if (!svd.singularValues().allFinite()) {
    mode->effective_basis_certified = false;
    return;
  }
  const double largest = svd.singularValues().size()
      ? svd.singularValues()(0) : 0.0;
  const double gate = rank_tolerance * std::max(1.0, largest);
  const int rank = static_cast<int>(
      (svd.singularValues().array() > gate).count());
  if (rank <= 0 || rank == physical) return;
  const Eigen::MatrixXd retained = svd.matrixV().leftCols(rank);
  const Eigen::MatrixXd discarded = svd.matrixV().rightCols(physical - rank);
  const Eigen::MatrixXd discarded_map = map * discarded;
  mode->discarded_measurement_norm = discarded_map.norm();
  if (window.numerics && window.numerics->information_factorization &&
      window.protected_state_map.cols() == window.H.cols()) {
    Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(window.H.rows(), physical);
    int aggregate_offset = 0;
    for (const auto& block : window.blocks) {
      const auto found = mode->raw_group_maps.find(block.group_id);
      if (found != mode->raw_group_maps.end()) {
        dense.middleRows(aggregate_offset, found->second.rows()) =
            block.whitener * found->second;
      }
      aggregate_offset += block.residual_whitened.size();
    }
    const Eigen::MatrixXd rhs = window.H.transpose() * dense * discarded;
    const Eigen::MatrixXd response = window.protected_state_map *
        window.numerics->information_factorization->solve(rhs);
    mode->discarded_protected_response_norm = response.norm();
  } else {
    mode->effective_basis_certified = false;
    return;
  }
  const double measurement_limit = rank_tolerance *
      std::max(1.0, map.norm()) * 16.0;
  const double response_limit = rank_tolerance *
      std::max(1.0, window.protected_state_map.norm()) * 16.0;
  if (mode->discarded_measurement_norm <= measurement_limit &&
      mode->discarded_protected_response_norm <= response_limit) {
    mode->effective_parameter_basis = retained;
    mode->effective_parameter_dimension = rank;
  } else {
    // Leave the physical basis intact so the downstream Gram rank gate fails
    // closed instead of hiding a protected-state-relevant direction.
    mode->effective_basis_certified = false;
  }
}

const PendingFactorGroup* selectedKind(const HistoricalEpochContext& history,
                                       FactorKind kind) {
  for (const auto id : history.selected_groups) {
    const auto found = std::find_if(history.groups.begin(), history.groups.end(),
        [&](const PendingFactorGroup& group) {
          return group.id == id && group.kind == kind;
        });
    if (found != history.groups.end()) return &*found;
  }
  return nullptr;
}

std::vector<Occurrence> occurrences(const EpochTransaction& tx) {
  std::vector<Occurrence> out;
  for (const auto& history : tx.recoverable_history) {
    Occurrence value;
    value.previous_epoch = history.previous_epoch;
    value.epoch = history.proposed_epoch;
    value.begin = history.begin;
    value.end = history.end;
    value.batch = &history.uwb_batch;
    value.uwb = selectedKind(history, FactorKind::UwbBatch);
    value.imu = selectedKind(history, FactorKind::CombinedImu);
    for (const auto& group : history.groups) {
      if (!group.replaces_group) continue;
      if (group.kind == FactorKind::KinematicBridge) value.pose_bridge = &group;
      if (group.kind == FactorKind::BiasContinuity) value.bias_bridge = &group;
    }
    value.history = &history;
    out.push_back(value);
  }
  Occurrence current;
  current.previous_epoch = tx.previous_epoch;
  current.epoch = tx.proposed_epoch;
  current.begin = tx.begin;
  current.end = tx.end;
  current.batch = &tx.uwb_batch;
  current.imu = &tx.imu_group;
  current.pose_bridge = &tx.generic_bridge_group;
  current.bias_bridge = &tx.generic_bias_continuity_group;
  const auto nominal = std::find_if(tx.uwb_groups.begin(), tx.uwb_groups.end(),
      [](const PendingFactorGroup& group) { return group.nominal; });
  if (nominal != tx.uwb_groups.end()) current.uwb = &*nominal;
  out.push_back(current);
  std::stable_sort(out.begin(), out.end(), [](const Occurrence& a,
                                              const Occurrence& b) {
    return a.epoch < b.epoch;
  });
  return out;
}

const UwbMeasurement* measurementById(const UwbBatch& batch,
                                      MeasurementId id) {
  const auto found = std::find_if(batch.measurements.begin(),
      batch.measurements.end(), [&](const UwbMeasurement& measurement) {
        return measurement.id == id;
      });
  return found == batch.measurements.end() ? nullptr : &*found;
}

std::string actionKey(const ExclusionAction& action) {
  std::vector<std::uint64_t> remove, add;
  for (const auto id : action.groups_to_remove) remove.push_back(id.value());
  for (const auto id : action.groups_to_add) add.push_back(id.value());
  std::sort(remove.begin(), remove.end());
  std::sort(add.begin(), add.end());
  std::ostringstream key;
  for (const auto id : remove) key << 'r' << id << ';';
  for (const auto id : add) key << 'a' << id << ';';
  key << 'b' << static_cast<int>(action.bridge_mode) << ';';
  std::vector<const LinearizedFactorBlock*> blocks;
  for (const auto& block : action.added_blocks) blocks.push_back(&block);
  std::sort(blocks.begin(), blocks.end(),
            [](const auto* left, const auto* right) {
              return left->group_id < right->group_id;
            });
  for (const auto* block : blocks) {
    key << 'g' << block->group_id.value() << ':'
        << block->jacobian_whitened.rows() << 'x'
        << block->jacobian_whitened.cols() << ':'
        << block->residual_whitened.size() << ':'
        << std::setprecision(17) << block->jacobian_whitened.squaredNorm()
        << ':' << block->residual_whitened.squaredNorm() << ':'
        << block->whitening_model_id << ';';
  }
  return key.str();
}

template <class DerivedA, class DerivedB>
bool exactEigenContent(const Eigen::MatrixBase<DerivedA>& left,
                       const Eigen::MatrixBase<DerivedB>& right) {
  if (left.rows() != right.rows() || left.cols() != right.cols()) return false;
  for (Eigen::Index column = 0; column < left.cols(); ++column) {
    for (Eigen::Index row = 0; row < left.rows(); ++row) {
      if (left(row, column) != right(row, column)) return false;
    }
  }
  return true;
}

bool equivalentBlock(const LinearizedFactorBlock& left,
                     const LinearizedFactorBlock& right) {
  return left.group_id == right.group_id && left.kind == right.kind &&
      left.sensor == right.sensor && left.role == right.role &&
      left.window_column_indices == right.window_column_indices &&
      left.fault_units == right.fault_units &&
      left.effective_weight == right.effective_weight &&
      left.whitening_model_id == right.whitening_model_id &&
      left.version == right.version &&
      exactEigenContent(left.jacobian_raw, right.jacobian_raw) &&
      exactEigenContent(left.residual_raw, right.residual_raw) &&
      exactEigenContent(left.covariance, right.covariance) &&
      exactEigenContent(left.whitener, right.whitener) &&
      exactEigenContent(left.jacobian_whitened, right.jacobian_whitened) &&
      exactEigenContent(left.residual_whitened, right.residual_whitened);
}

template <class T>
std::vector<T> sortedCopy(std::vector<T> values) {
  std::sort(values.begin(), values.end());
  return values;
}

}  // namespace

bool equivalentActionOperation(const ExclusionAction& left,
                               const ExclusionAction& right) {
  if (sortedCopy(left.groups_to_remove) != sortedCopy(right.groups_to_remove) ||
      sortedCopy(left.groups_to_add) != sortedCopy(right.groups_to_add) ||
      left.bridge_mode != right.bridge_mode ||
      left.recoverability != right.recoverability ||
      left.recovery_epoch_begin != right.recovery_epoch_begin ||
      left.recovery_epoch_end != right.recovery_epoch_end ||
      left.added_blocks.size() != right.added_blocks.size()) {
    return false;
  }
  std::vector<const LinearizedFactorBlock*> left_blocks, right_blocks;
  for (const auto& block : left.added_blocks) left_blocks.push_back(&block);
  for (const auto& block : right.added_blocks) right_blocks.push_back(&block);
  const auto order = [](const auto* a, const auto* b) {
    return a->group_id < b->group_id;
  };
  std::sort(left_blocks.begin(), left_blocks.end(), order);
  std::sort(right_blocks.begin(), right_blocks.end(), order);
  for (std::size_t i = 0; i < left_blocks.size(); ++i) {
    if (!equivalentBlock(*left_blocks[i], *right_blocks[i])) return false;
  }
  return true;
}

std::string healthSourceId(const FaultUnit& unit) {
  if (unit.sensor == SensorType::Uwb) {
    const auto prefix = unit.physical_source_id.rfind("uwb:", 0) == 0
        ? unit.physical_source_id.substr(4)
        : std::to_string(unit.id.value());
    return "anchor:" + prefix;
  }
  static const char* axes[] = {"x", "y", "z"};
  if (unit.axis < 0 || unit.axis >= 3) return {};
  if (unit.sensor == SensorType::ImuAccelerometer) {
    return std::string("accel:") + axes[unit.axis];
  }
  if (unit.sensor == SensorType::ImuGyroscope) {
    return std::string("gyro:") + axes[unit.axis];
  }
  return {};
}

namespace {

void mergeCoverage(ExclusionAction* target, const ExclusionAction& source) {
  auto merge_ids = [](auto* destination, const auto& additions) {
    for (const auto id : additions) {
      if (std::find(destination->begin(), destination->end(), id) ==
          destination->end()) destination->push_back(id);
    }
    std::sort(destination->begin(), destination->end());
  };
  merge_ids(&target->covered_modes, source.covered_modes);
  merge_ids(&target->covered_units, source.covered_units);
  for (const auto& physical : source.physical_source_ids) {
    if (std::find(target->physical_source_ids.begin(),
                  target->physical_source_ids.end(), physical) ==
        target->physical_source_ids.end()) {
      target->physical_source_ids.push_back(physical);
    }
  }
  std::sort(target->physical_source_ids.begin(),
            target->physical_source_ids.end());
  target->exclusion_cardinality = static_cast<int>(
      target->physical_source_ids.size());
}

LinearizedFactorBlock replacementUwbBlock(
    const LinearizedIntegrityWindow& window,
    const PendingFactorGroup& original,
    const PendingFactorGroup& replacement) {
  const auto* source = blockFor(window, original.id);
  if (!source) throw std::runtime_error("recovery UWB source block is absent");
  std::map<std::uint64_t, int> source_rows;
  for (std::size_t i = 0; i < original.source_measurements.size(); ++i) {
    source_rows.emplace(original.source_measurements[i].value(),
                        static_cast<int>(i));
  }
  LinearizedFactorBlock out;
  out.group_id = replacement.id;
  out.kind = FactorKind::UwbBatch;
  out.sensor = SensorType::Uwb;
  out.role = RowRole::Measurement;
  out.covariance = replacement.raw_covariance;
  Eigen::LLT<Eigen::MatrixXd> llt(out.covariance);
  if (llt.info() != Eigen::Success) {
    throw std::runtime_error("retained UWB covariance is not SPD");
  }
  out.whitener = llt.matrixL().solve(Eigen::MatrixXd::Identity(
      out.covariance.rows(), out.covariance.cols()));
  out.jacobian_raw = Eigen::MatrixXd::Zero(
      replacement.source_measurements.size(), window.H.cols());
  out.residual_raw = Eigen::VectorXd::Zero(replacement.source_measurements.size());
  for (std::size_t row = 0; row < replacement.source_measurements.size(); ++row) {
    const auto found = source_rows.find(replacement.source_measurements[row].value());
    if (found == source_rows.end()) {
      throw std::runtime_error("retained UWB measurement provenance mismatch");
    }
    out.jacobian_raw.row(row) = source->jacobian_raw.row(found->second);
    out.residual_raw(row) = source->residual_raw(found->second);
  }
  out.jacobian_whitened = out.whitener * out.jacobian_raw;
  out.residual_whitened = out.whitener * out.residual_raw;
  out.whitening_model_id = replacement.noise_model_id;
  out.version = window.version;
  return out;
}

const PendingFactorGroup* replacementFor(const EpochTransaction& tx,
                                         const Occurrence& occurrence,
                                         std::uint64_t anchor) {
  const std::vector<PendingFactorGroup>* groups = occurrence.history
      ? &occurrence.history->groups : &tx.uwb_groups;
  for (const auto& group : *groups) {
    if (group.kind == FactorKind::UwbBatch && !group.nominal &&
        group.excluded_fault_units.size() == 1 &&
        group.excluded_fault_units.front().value() == anchor &&
        (!group.replaces_group || *group.replaces_group == occurrence.uwb->id)) {
      return &group;
    }
  }
  return nullptr;
}

LinearizedFactorBlock bridgeBlock(const EpochTransaction& tx,
                                  const LinearizedIntegrityWindow& window,
                                  const Occurrence& occurrence,
                                  const PendingFactorGroup& group) {
  if (!tx.frozen_values) throw std::runtime_error("frozen values unavailable");
  gtsam::Ordering ordering;
  for (const auto key : group.keys) {
    if (std::find(ordering.begin(), ordering.end(), key) == ordering.end()) {
      ordering.push_back(key);
    }
  }
  const auto gaussian = group.factors.linearize(*tx.frozen_values);
  const auto dense = gaussian->jacobian(ordering);
  LinearizedFactorBlock out;
  out.group_id = group.id;
  out.kind = group.kind;
  out.sensor = SensorType::Bridge;
  out.role = RowRole::Measurement;
  out.jacobian_whitened = Eigen::MatrixXd::Zero(dense.first.rows(), window.H.cols());
  int local_column = 0;
  for (const auto key : ordering) {
    const char type = gtsam::Symbol(key).chr();
    const std::size_t epoch = gtsam::Symbol(key).index();
    const int dimension = type == 'v' ? 3 : 6;
    const int within = type == 'x' ? 0 : (type == 'v' ? 6 : 9);
    const int target = static_cast<int>((epoch - window.detector_first_epoch) * 15) + within;
    out.jacobian_whitened.block(0, target, dense.first.rows(), dimension) =
        dense.first.block(0, local_column, dense.first.rows(), dimension);
    local_column += dimension;
  }
  out.residual_whitened = dense.second;
  out.jacobian_raw = out.jacobian_whitened;
  out.residual_raw = out.residual_whitened;
  out.covariance = Eigen::MatrixXd::Identity(dense.first.rows(), dense.first.rows());
  out.whitener = out.covariance;
  out.whitening_model_id = group.noise_model_id;
  out.version = window.version;
  return out;
}

ExclusionAction actionForMode(const EpochTransaction& tx,
                              const LinearizedIntegrityWindow& window,
                              const FaultModeBasis& mode,
                              const std::vector<Occurrence>& all) {
  ExclusionAction action;
  action.covered_modes = {mode.id};
  action.physical_source_ids = {mode.physical_source_id};
  action.covered_units = {FaultUnitId(mode.id.value())};
  action.exclusion_cardinality = 1;
  action.recoverability = mode.recoverability;
  for (const auto group_id : mode.affected_groups) {
    const auto occurrence = std::find_if(all.begin(), all.end(),
        [&](const Occurrence& item) {
          return (item.uwb && item.uwb->id == group_id) ||
                 (item.imu && item.imu->id == group_id);
        });
    if (occurrence == all.end()) continue;
    action.groups_to_remove.push_back(group_id);
    if (mode.sensor == SensorType::Uwb) {
      const auto* replacement = replacementFor(tx, *occurrence,
                                                 mode.anchor_id.value());
      if (!replacement) {
        action.recoverability = HistoryRecoverability::MissingProvenance;
        continue;
      }
      action.groups_to_add.push_back(replacement->id);
      action.added_blocks.push_back(replacementUwbBlock(
          window, *occurrence->uwb, *replacement));
    } else {
      if (!occurrence->pose_bridge || !occurrence->bias_bridge) {
        action.recoverability = HistoryRecoverability::MissingProvenance;
        continue;
      }
      action.groups_to_add.push_back(occurrence->pose_bridge->id);
      action.groups_to_add.push_back(occurrence->bias_bridge->id);
      action.added_blocks.push_back(bridgeBlock(
          tx, window, *occurrence, *occurrence->pose_bridge));
      action.added_blocks.push_back(bridgeBlock(
          tx, window, *occurrence, *occurrence->bias_bridge));
      action.bridge_mode = BridgeMode::GenericKinematic;
    }
    if (occurrence->epoch < tx.proposed_epoch) {
      action.recovery_epoch_begin = action.recovery_epoch_begin
          ? std::min(*action.recovery_epoch_begin, occurrence->epoch)
          : occurrence->epoch;
      action.recovery_epoch_end = action.recovery_epoch_end
          ? std::max(*action.recovery_epoch_end, occurrence->epoch)
          : occurrence->epoch;
    }
  }
  action.action_model_id = mode.sensor == SensorType::Uwb
      ? "UWB_HISTORY_RECOVERY" : "IMU_HISTORY_BRIDGE_RECOVERY";
  return action;
}

ExclusionAction unite(const std::vector<const ExclusionAction*>& parts) {
  ExclusionAction out;
  std::set<std::uint64_t> remove, add, modes, units;
  std::set<std::string> physical_sources;
  for (const auto* part : parts) {
    if (!part) continue;
    out.bridge_mode = part->bridge_mode != BridgeMode::None
        ? part->bridge_mode : out.bridge_mode;
    if (part->recoverability != HistoryRecoverability::Recoverable) {
      out.recoverability = part->recoverability;
    }
    if (part->recovery_epoch_begin) out.recovery_epoch_begin =
        out.recovery_epoch_begin ? std::min(*out.recovery_epoch_begin,
                                             *part->recovery_epoch_begin)
                                 : part->recovery_epoch_begin;
    if (part->recovery_epoch_end) out.recovery_epoch_end =
        out.recovery_epoch_end ? std::max(*out.recovery_epoch_end,
                                           *part->recovery_epoch_end)
                               : part->recovery_epoch_end;
    for (const auto id : part->groups_to_remove) {
      if (remove.insert(id.value()).second) out.groups_to_remove.push_back(id);
    }
    for (std::size_t i = 0; i < part->groups_to_add.size(); ++i) {
      const auto id = part->groups_to_add[i];
      if (add.insert(id.value()).second) {
        out.groups_to_add.push_back(id);
        if (i < part->added_blocks.size()) out.added_blocks.push_back(part->added_blocks[i]);
      }
    }
    for (const auto id : part->covered_modes) {
      if (modes.insert(id.value()).second) out.covered_modes.push_back(id);
    }
    for (const auto id : part->covered_units) {
      if (units.insert(id.value()).second) out.covered_units.push_back(id);
    }
    for (const auto& source : part->physical_source_ids) {
      if (physical_sources.insert(source).second) {
        out.physical_source_ids.push_back(source);
      }
    }
  }
  std::sort(out.groups_to_remove.begin(), out.groups_to_remove.end());
  out.exclusion_cardinality = static_cast<int>(physical_sources.size());
  out.action_model_id = parts.size() > 1 ? "CANONICAL_UNION_RECOVERY"
                                         : (parts.empty() ? "KEEP_ALL"
                                                          : parts.front()->action_model_id);
  return out;
}

}  // namespace

GeneratedFaultModelSet HypothesisGenerator::generate(
    const LinearizedIntegrityWindow& window, const EpochTransaction& tx,
    const ImuFaultSubspaces& current_imu,
    const LinearizedFactorBlock&) const {
  if (!window.model_valid || !current_imu.analytic_input_valid ||
      !current_imu.analytic_computation_valid ||
      config_.max_model_cardinality != 2 ||
      config_.max_exclusion_cardinality == 0 ||
      config_.max_exclusion_cardinality > config_.max_model_cardinality ||
      config_.max_candidate_count == 0 ||
      (!config_.single_faults_enabled && !config_.double_faults_enabled) ||
      (config_.double_faults_enabled &&
       !config_.include_uwb_accel_combinations &&
       !config_.include_uwb_gyro_combinations)) {
    throw std::invalid_argument("fault model generation precondition failed");
  }
  GeneratedFaultModelSet out;
  const auto all = occurrences(tx);
  std::map<std::uint64_t, std::vector<const Occurrence*>> anchor_occurrences;
  for (const auto& occurrence : all) {
    if (!occurrence.uwb || !occurrence.batch ||
        !explicitlyMonitored(window, occurrence.uwb->id)) continue;
    for (const auto id : occurrence.uwb->source_measurements) {
      const auto* measurement = measurementById(*occurrence.batch, id);
      if (measurement) anchor_occurrences[measurement->anchor_id.value()].push_back(&occurrence);
    }
  }

  std::uint64_t next_mode = 1;
  auto append_mode = [&](FaultModeBasis mode) {
    finalizeEffectiveBasis(window, config_.rank_tolerance, &mode);
    mode.id = FaultModeId(next_mode++);
    FaultUnit unit;
    unit.id = FaultUnitId(mode.id.value());
    unit.sensor = mode.sensor;
    unit.kind = mode.kind;
    unit.physical_source_id = mode.physical_source_id;
    unit.axis = mode.axis;
    unit.epoch_begin = mode.onset_epoch;
    unit.epoch_end = tx.proposed_epoch;
    unit.time_begin = mode.onset_time;
    unit.time_end = tx.end;
    unit.affected_groups = mode.affected_groups;
    unit.affected_measurements = mode.affected_measurements;
    unit.parameter_dimension = mode.parameter_dimension;
    unit.prior_probability_bound = mode.prior_probability_bound;
    unit.persistent = mode.kind != FaultKind::AnchorBiasEpochIndependent;
    out.units.push_back(std::move(unit));
    out.modes.push_back(std::move(mode));
  };

  for (const auto& anchor_item : anchor_occurrences) {
    std::vector<const Occurrence*> unique = anchor_item.second;
    std::sort(unique.begin(), unique.end(), [](const Occurrence* a,
                                               const Occurrence* b) {
      return a->epoch < b->epoch;
    });
    unique.erase(std::unique(unique.begin(), unique.end()), unique.end());
    auto fill = [&](FaultModeBasis* mode, std::size_t onset, bool ramp,
                    bool expand_left) {
      const std::size_t effective_onset = expand_left
          ? tx.oldest_recoverable_epoch : onset;
      for (const auto& occurrence : all) {
        if (!occurrence.uwb || !occurrence.batch ||
            occurrence.epoch < effective_onset) continue;
        if (!explicitlyMonitored(window, occurrence.uwb->id)) continue;
        const auto* block = blockFor(window, occurrence.uwb->id);
        if (!block) continue;
        Eigen::MatrixXd raw = Eigen::MatrixXd::Zero(
            block->residual_raw.size(), ramp ? 2 : 1);
        for (std::size_t row = 0; row < occurrence.uwb->source_measurements.size(); ++row) {
          const auto* measurement = measurementById(
              *occurrence.batch, occurrence.uwb->source_measurements[row]);
          if (!measurement || measurement->anchor_id.value() != anchor_item.first) continue;
          raw(row, 0) = 1.0;
          if (ramp) raw(row, 1) = measurement->timestamp.seconds() -
              mode->onset_time.seconds();
          mode->affected_measurements.push_back(measurement->id);
        }
        if (raw.cwiseAbs().maxCoeff() > 0.0) {
          mode->raw_group_maps[occurrence.uwb->id] = raw;
          mode->affected_groups.push_back(occurrence.uwb->id);
        }
      }
    };
    for (const auto* onset : unique) {
      if (config_.include_epoch_independent_uwb) {
        FaultModeBasis mode;
        mode.kind = FaultKind::AnchorBiasEpochIndependent;
        mode.sensor = SensorType::Uwb;
        mode.physical_source_id = "uwb:" + std::to_string(anchor_item.first);
        mode.anchor_id = AnchorId(anchor_item.first);
        mode.onset_epoch = onset->epoch;
        mode.onset_time = onset->end;
        mode.parameter_dimension = 1;
        mode.prior_probability_bound = config_.uwb_prior_bound;
        mode.recoverability = tx.history_recoverability;
        for (const auto& occurrence : all) {
          if (occurrence.epoch == onset->epoch) fill(&mode, occurrence.epoch, false, false);
        }
        // Epoch-independent means exactly the onset group.
        for (auto it = mode.raw_group_maps.begin(); it != mode.raw_group_maps.end();) {
          const auto found = std::find_if(all.begin(), all.end(), [&](const Occurrence& o) {
            return o.uwb && o.uwb->id == it->first;
          });
          if (found != all.end() && found->epoch != onset->epoch) it = mode.raw_group_maps.erase(it);
          else ++it;
        }
        mode.affected_groups.clear();
        for (const auto& item : mode.raw_group_maps) mode.affected_groups.push_back(item.first);
        mode.affected_measurements.clear();
        for (const auto& occurrence : all) {
          if (!occurrence.uwb || !occurrence.batch ||
              occurrence.epoch != onset->epoch) continue;
          for (const auto measurement_id : occurrence.uwb->source_measurements) {
            const auto* measurement = measurementById(*occurrence.batch, measurement_id);
            if (measurement && measurement->anchor_id.value() == anchor_item.first) {
              mode.affected_measurements.push_back(measurement_id);
            }
          }
        }
        append_mode(std::move(mode));
      }
      for (const bool ramp : {false, true}) {
        if ((!ramp && !config_.include_persistent_uwb) ||
            (ramp && !config_.include_ramp_uwb)) continue;
        FaultModeBasis mode;
        mode.kind = ramp ? FaultKind::AnchorBiasRamp
                         : FaultKind::AnchorBiasPersistentConstant;
        mode.sensor = SensorType::Uwb;
        mode.physical_source_id = "uwb:" + std::to_string(anchor_item.first);
        mode.anchor_id = AnchorId(anchor_item.first);
        mode.onset_epoch = onset->epoch;
        mode.onset_time = onset->end;
        mode.parameter_dimension = ramp ? 2 : 1;
        mode.prior_probability_bound = config_.uwb_prior_bound;
        mode.recoverability = tx.history_recoverability;
        fill(&mode, onset->epoch, ramp,
             onset->epoch == window.detector_first_epoch);
        append_mode(std::move(mode));
      }
    }
  }

  for (const auto& occurrence : all) {
    if (!occurrence.imu ||
        !explicitlyMonitored(window, occurrence.imu->id)) continue;
    const auto* block = blockFor(window, occurrence.imu->id);
    if (!block) continue;
    ImuFaultSubspaces subspaces = current_imu;
    if (occurrence.history) {
      EpochTransaction local;
      local.previous_state = occurrence.history->previous_state;
      local.nominal_predicted_state = occurrence.history->current_state;
      local.preintegration = occurrence.history->preintegration;
      local.raw_imu_slice = occurrence.history->raw_imu_slice;
      const auto sensitivity_start = std::chrono::steady_clock::now();
      subspaces = ImuFaultSubspaceBuilder().buildAnalytic(local, *block);
      out.historical_sensitivity_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - sensitivity_start).count();
    }
    if (!subspaces.analytic_input_valid ||
        !subspaces.analytic_computation_valid) continue;
    for (int axis = 0; axis < 6; ++axis) {
      FaultModeBasis mode;
      mode.kind = axis < 3 ? FaultKind::AccelAxisIntervalConstant
                           : FaultKind::GyroAxisIntervalConstant;
      mode.sensor = axis < 3 ? SensorType::ImuAccelerometer
                             : SensorType::ImuGyroscope;
      mode.physical_source_id = std::string(axis < 3 ? "imu_accel:" : "imu_gyro:") +
          std::to_string(axis % 3) + ":interval:" + std::to_string(occurrence.epoch);
      mode.axis = axis % 3;
      mode.onset_epoch = occurrence.previous_epoch;
      mode.onset_time = occurrence.begin;
      mode.parameter_dimension = 1;
      mode.prior_probability_bound = axis < 3 ? config_.accel_prior_bound
                                              : config_.gyro_prior_bound;
      mode.recoverability = tx.history_recoverability;
      mode.affected_groups = {occurrence.imu->id};
      const Eigen::VectorXd whitened = axis < 3
          ? subspaces.accel_axis[axis] : subspaces.gyro_axis[axis - 3];
      mode.raw_group_maps[occurrence.imu->id] =
          block->whitener.triangularView<Eigen::Lower>().solve(whitened);
      append_mode(std::move(mode));
    }
  }

  std::uint64_t next_hypothesis = 1;
  auto hypothesis = [&](std::initializer_list<FaultModeId> ids) {
    FaultHypothesisV2 value;
    value.id = HypothesisId(next_hypothesis++);
    value.modes.assign(ids.begin(), ids.end());
    value.prior_probability_bound = 1.0;
    bool all_uwb = true;
    for (const auto id : value.modes) {
      const auto& mode = out.modes.at(id.value() - 1);
      value.units.push_back(FaultUnitId(id.value()));
      value.affected_groups.insert(value.affected_groups.end(),
                                   mode.affected_groups.begin(),
                                   mode.affected_groups.end());
      value.prior_probability_bound = std::min(
          value.prior_probability_bound, mode.prior_probability_bound);
      all_uwb = all_uwb && mode.sensor == SensorType::Uwb;
    }
    std::sort(value.affected_groups.begin(), value.affected_groups.end());
    value.affected_groups.erase(
        std::unique(value.affected_groups.begin(), value.affected_groups.end()),
        value.affected_groups.end());
    value.p_md_allocation = all_uwb ? config_.uwb_p_md : config_.imu_p_md;
    out.hypotheses.push_back(std::move(value));
  };
  if (config_.single_faults_enabled) {
    for (const auto& mode : out.modes) hypothesis({mode.id});
  }
  if (config_.double_faults_enabled) {
    for (const auto& uwb : out.modes) {
      if (uwb.sensor != SensorType::Uwb) continue;
      for (const auto& imu : out.modes) {
        const bool enabled =
            (imu.sensor == SensorType::ImuAccelerometer &&
             config_.include_uwb_accel_combinations) ||
            (imu.sensor == SensorType::ImuGyroscope &&
             config_.include_uwb_gyro_combinations);
        if (enabled) hypothesis({uwb.id, imu.id});
      }
    }
  }
  for (const auto& value : out.hypotheses) {
    out.effective_max_cardinality = std::max<std::uint32_t>(
        out.effective_max_cardinality,
        static_cast<std::uint32_t>(value.modes.size()));
    if (value.modes.size() == 1) {
      const auto& mode = out.modes.at(value.modes.front().value() - 1);
      if (mode.sensor == SensorType::Uwb) ++out.single_uwb_hypotheses;
      else if (mode.sensor == SensorType::ImuAccelerometer)
        ++out.single_accel_hypotheses;
      else if (mode.sensor == SensorType::ImuGyroscope)
        ++out.single_gyro_hypotheses;
    } else if (value.modes.size() == 2) {
      bool accel = false, gyro = false, uwb = false;
      for (const auto id : value.modes) {
        const auto sensor = out.modes.at(id.value() - 1).sensor;
        uwb = uwb || sensor == SensorType::Uwb;
        accel = accel || sensor == SensorType::ImuAccelerometer;
        gyro = gyro || sensor == SensorType::ImuGyroscope;
      }
      if (uwb && accel) ++out.double_uwb_accel_hypotheses;
      else if (uwb && gyro) ++out.double_uwb_gyro_hypotheses;
    }
  }
  const double allocation = conservativeEqualRiskAllocation(
      config_.total_hmi_allocation, out.hypotheses.size());
  for (auto& value : out.hypotheses) value.hmi_allocation = allocation;

  std::uint64_t next_action = 2;
  for (const auto& mode : out.modes) {
    auto action = actionForMode(tx, window, mode, all);
    action.id = ExclusionActionId(next_action++);
    out.single_mode_actions.push_back(std::move(action));
  }
  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  out.actions.push_back(keep);
  std::map<std::string, std::vector<std::size_t>> seen;
  for (const auto& action : out.single_mode_actions) {
    if (action.recoverability != HistoryRecoverability::Recoverable) continue;
    const auto key = actionKey(action);
    const auto duplicate = seen.find(key);
    auto equivalent = out.actions.end();
    if (duplicate != seen.end()) {
      for (const auto index : duplicate->second) {
        if (equivalentActionOperation(out.actions[index], action)) {
          equivalent = out.actions.begin() + index;
          break;
        }
      }
    }
    if (equivalent != out.actions.end()) {
      mergeCoverage(&*equivalent, action);
      continue;
    }
    if (out.actions.size() == config_.max_candidate_count) break;
    seen[key].push_back(out.actions.size());
    out.actions.push_back(action);
  }
  return out;
}

std::vector<ExclusionAction> HypothesisGenerator::actionsForPlausibleSet(
    const LinearizedIntegrityWindow&, const EpochTransaction& transaction,
    const GeneratedFaultModelSet& models,
    const std::vector<FaultModeEvidence>& evidence,
    const std::vector<std::string>& mandatory_health_sources) const {
  std::map<std::uint64_t, const ExclusionAction*> singles;
  for (const auto& action : models.single_mode_actions) {
    for (const auto mode : action.covered_modes) singles[mode.value()] = &action;
  }
  std::vector<std::vector<const ExclusionAction*>> requested;
  std::vector<const ExclusionAction*> full;
  std::vector<const ExclusionAction*> mandatory;
  std::set<std::uint64_t> plausible_anchors;
  const auto is_current_group = [&](FactorGroupId group) {
    if (group == transaction.imu_group.id) return true;
    return std::any_of(transaction.uwb_groups.begin(),
        transaction.uwb_groups.end(), [&](const PendingFactorGroup& value) {
          return value.nominal && value.id == group;
        });
  };
  for (const auto& action : models.single_mode_actions) {
    if (!std::any_of(action.groups_to_remove.begin(),
                     action.groups_to_remove.end(), is_current_group)) {
      continue;
    }
    bool required = false;
    for (const auto unit_id : action.covered_units) {
      const auto unit = std::find_if(models.units.begin(), models.units.end(),
          [&](const FaultUnit& value) { return value.id == unit_id; });
      if (unit != models.units.end() &&
          std::find(mandatory_health_sources.begin(),
                    mandatory_health_sources.end(), healthSourceId(*unit)) !=
              mandatory_health_sources.end()) {
        required = true;
        break;
      }
    }
    if (required) mandatory.push_back(&action);
  }
  const auto plausible = completePlausibleHypotheses(
      models.hypotheses, evidence);
  for (const auto hypothesis_id : plausible) {
    const auto found = std::find_if(models.hypotheses.begin(), models.hypotheses.end(),
        [&](const FaultHypothesisV2& hypothesis) {
          return hypothesis.id == hypothesis_id;
        });
    if (found == models.hypotheses.end()) continue;
    std::vector<const ExclusionAction*> parts;
    for (const auto mode : found->modes) {
      const auto mode_value = std::find_if(models.modes.begin(), models.modes.end(),
          [&](const FaultModeBasis& value) { return value.id == mode; });
      if (mode_value != models.modes.end() &&
          mode_value->sensor == SensorType::Uwb) {
        plausible_anchors.insert(mode_value->anchor_id.value());
      }
      const auto action = singles.find(mode.value());
      if (action != singles.end()) {
        parts.push_back(action->second);
        full.push_back(action->second);
      }
    }
    parts.insert(parts.end(), mandatory.begin(), mandatory.end());
    if (!parts.empty()) requested.push_back(std::move(parts));
  }
  if (plausible.empty() && !mandatory.empty()) requested.push_back(mandatory);
  std::vector<ExclusionAction> out;
  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  out.push_back(keep);
  std::map<std::string, std::vector<std::size_t>> seen;
  std::uint64_t id = 2;
  for (const auto& parts : requested) {
    auto action = unite(parts);
    if (action.exclusion_cardinality >
        static_cast<int>(config_.max_exclusion_cardinality)) {
      action.recoverability = HistoryRecoverability::MissingProvenance;
      action.action_model_id = "CARDINALITY_CONTRACT_REJECTED";
    }
    const auto key = actionKey(action);
    const auto duplicate = seen.find(key);
    std::optional<std::size_t> equivalent;
    if (duplicate != seen.end()) {
      for (const auto index : duplicate->second) {
        if (equivalentActionOperation(out[index], action)) {
          equivalent = index;
          break;
        }
      }
    }
    if (!equivalent) {
      action.id = ExclusionActionId(id++);
      seen[key].push_back(out.size());
      out.push_back(std::move(action));
    } else {
      mergeCoverage(&out[*equivalent], action);
    }
  }
  std::sort(full.begin(), full.end(), [](const ExclusionAction* left,
                                        const ExclusionAction* right) {
    const auto left_id = left->covered_modes.empty()
        ? std::uint64_t{0} : left->covered_modes.front().value();
    const auto right_id = right->covered_modes.empty()
        ? std::uint64_t{0} : right->covered_modes.front().value();
    return left_id < right_id;
  });
  full.erase(std::unique(full.begin(), full.end(),
                         [](const ExclusionAction* left,
                            const ExclusionAction* right) {
    return left->covered_modes == right->covered_modes;
  }), full.end());
  full.insert(full.end(), mandatory.begin(), mandatory.end());
  if (!full.empty() && plausible_anchors.size() <= 1) {
    auto union_action = unite(full);
    if (union_action.exclusion_cardinality >
        static_cast<int>(config_.max_exclusion_cardinality)) {
      union_action.recoverability = HistoryRecoverability::MissingProvenance;
    }
    const auto key = actionKey(union_action);
    const auto duplicate = seen.find(key);
    std::optional<std::size_t> equivalent;
    if (duplicate != seen.end()) {
      for (const auto index : duplicate->second) {
        if (equivalentActionOperation(out[index], union_action)) {
          equivalent = index;
          break;
        }
      }
    }
    if (!equivalent) {
      union_action.id = ExclusionActionId(id++);
      union_action.action_model_id = "FULL_PLAUSIBLE_UNION_RECOVERY";
      seen[key].push_back(out.size());
      out.push_back(std::move(union_action));
    } else {
      mergeCoverage(&out[*equivalent], union_action);
      out[*equivalent].action_model_id =
          "FULL_PLAUSIBLE_UNION_RECOVERY";
    }
  }
  if (out.size() > config_.max_candidate_count) {
    const ExclusionAction full_union = out.back();
    out.resize(config_.max_candidate_count);
    out.back() = full_union;
  }
  return out;
}

}  // namespace uwb_imu_pl
