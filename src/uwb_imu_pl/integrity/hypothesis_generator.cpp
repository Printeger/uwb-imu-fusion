#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"
#include "uwb_imu_pl/integrity/risk_budget_audit.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>

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

const LinearizedFactorBlock* historySummaryBlock(
    const LinearizedIntegrityWindow& window) {
  const auto found = std::find_if(
      window.blocks.begin(), window.blocks.end(), [](const auto& block) {
        return block.whitening_model_id == "history_summary_sqrt_d1";
      });
  return found == window.blocks.end() ? nullptr : &*found;
}

bool hasHistoryColumn(const LinearizedIntegrityWindow& window,
                      HistoryFaultBasisKind kind, std::uint64_t source,
                      std::size_t epoch) {
  const auto& ids = window.history_summary.column_ids;
  return std::find(ids.begin(), ids.end(),
                   HistoryFaultColumnId{kind, source, epoch}) != ids.end();
}

Eigen::MatrixXd historicalMap(
    const LinearizedIntegrityWindow& window, int parameter_dimension,
    const std::function<Eigen::VectorXd(const HistoryFaultColumnId&)>&
        coefficient) {
  const auto& history = window.history_summary;
  const Eigen::Index supported = history.response.rows();
  const Eigen::Index perpendicular = history.detector_response.rows();
  Eigen::MatrixXd map = Eigen::MatrixXd::Zero(
      supported + perpendicular, parameter_dimension);
  if (!history.valid || history.column_ids.size() != history.fault_columns ||
      history.response.cols() != static_cast<Eigen::Index>(history.fault_columns) ||
      history.detector_response.cols() !=
          static_cast<Eigen::Index>(history.fault_columns)) {
    return map;
  }
  for (std::size_t column = 0; column < history.column_ids.size(); ++column) {
    const Eigen::VectorXd weights = coefficient(history.column_ids[column]);
    if (weights.size() != parameter_dimension || weights.isZero(0.0)) continue;
    map.topRows(supported).noalias() +=
        history.response.col(static_cast<Eigen::Index>(column)) *
        weights.transpose();
    map.bottomRows(perpendicular).noalias() +=
        history.detector_response.col(static_cast<Eigen::Index>(column)) *
        weights.transpose();
  }
  return map;
}

std::string physicalModeIdentity(const FaultModeBasis& mode) {
  std::ostringstream out;
  out << static_cast<int>(mode.kind) << ':' << mode.physical_source_id << ':'
      << mode.onset_epoch << ':' << mode.parameter_dimension;
  return out.str();
}

std::string physicalPairIdentity(std::string left, std::string right) {
  if (right < left) std::swap(left, right);
  return left + "||" + right;
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
  if (!map.allFinite()) {
    mode->effective_basis_certified = false;
    return;
  }
  // P0-03: an unbounded fault has no numerical amplitude below which a
  // nonzero map can be discarded.  Keep the complete physical basis.  Exact
  // zero columns may still be removed: their standard-basis identity is a
  // symbolic proof, not a magnitude decision.  SVD truncation (even when G is
  // also small) is not such a proof.
  std::vector<int> structurally_active;
  for (int column = 0; column < physical; ++column) {
    bool active = false;
    for (Eigen::Index row = 0; row < map.rows(); ++row) {
      if (map(row, column) != 0.0) {
        active = true;
        break;
      }
    }
    if (active) structurally_active.push_back(column);
  }
  if (!structurally_active.empty() &&
      structurally_active.size() < static_cast<std::size_t>(physical)) {
    mode->effective_parameter_basis = Eigen::MatrixXd::Zero(
        physical, static_cast<int>(structurally_active.size()));
    for (std::size_t column = 0; column < structurally_active.size(); ++column) {
      mode->effective_parameter_basis(
          structurally_active[column], static_cast<int>(column)) = 1.0;
    }
    mode->effective_parameter_dimension =
        static_cast<int>(structurally_active.size());
    mode->discarded_measurement_norm = 0.0;
    mode->discarded_protected_response_norm = 0.0;
  }
  (void)rank_tolerance;
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

template <class T>
void appendIntegralIdentity(std::ostringstream* out, T value) {
  *out << std::hex << static_cast<std::uint64_t>(value) << ';' << std::dec;
}

void appendStringIdentity(std::ostringstream* out, const std::string& value) {
  *out << value.size() << ':';
  for (const unsigned char c : value) {
    *out << std::hex << std::setw(2) << std::setfill('0')
         << static_cast<unsigned int>(c);
  }
  *out << ';' << std::dec << std::setfill(' ');
}

void appendDoubleIdentity(std::ostringstream* out, double value) {
  static_assert(sizeof(double) == sizeof(std::uint64_t),
                "P0-06 identity requires 64-bit doubles");
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  appendIntegralIdentity(out, bits);
}

template <class Derived>
void appendEigenIdentity(std::ostringstream* out,
                         const Eigen::MatrixBase<Derived>& value) {
  appendIntegralIdentity(out, value.rows());
  appendIntegralIdentity(out, value.cols());
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    for (Eigen::Index column = 0; column < value.cols(); ++column) {
      appendDoubleIdentity(out, value(row, column));
    }
  }
}

std::string exactBlockIdentity(const LinearizedFactorBlock& block) {
  std::ostringstream out;
  appendIntegralIdentity(&out, block.group_id.value());
  appendIntegralIdentity(&out, static_cast<int>(block.kind));
  appendIntegralIdentity(&out, static_cast<int>(block.sensor));
  appendIntegralIdentity(&out, static_cast<int>(block.role));
  appendIntegralIdentity(&out, block.window_column_indices.size());
  for (const int column : block.window_column_indices) {
    appendIntegralIdentity(&out, column);
  }
  appendIntegralIdentity(&out, block.fault_units.size());
  for (const auto unit : block.fault_units) {
    appendIntegralIdentity(&out, unit.value());
  }
  appendDoubleIdentity(&out, block.effective_weight);
  appendStringIdentity(&out, block.whitening_model_id);
  appendIntegralIdentity(&out, block.version.graph_version);
  appendIntegralIdentity(&out, block.version.ordering_version);
  appendIntegralIdentity(&out, block.version.noise_model_version);
  appendIntegralIdentity(&out, block.version.linpoint_version);
  appendEigenIdentity(&out, block.jacobian_raw);
  appendEigenIdentity(&out, block.residual_raw);
  appendEigenIdentity(&out, block.covariance);
  appendEigenIdentity(&out, block.whitener);
  appendEigenIdentity(&out, block.jacobian_whitened);
  appendEigenIdentity(&out, block.residual_whitened);
  return out.str();
}

bool finiteBlockOperation(const LinearizedFactorBlock& block) {
  return std::isfinite(block.effective_weight) &&
      block.jacobian_raw.allFinite() && block.residual_raw.allFinite() &&
      block.covariance.allFinite() && block.whitener.allFinite() &&
      block.jacobian_whitened.allFinite() &&
      block.residual_whitened.allFinite();
}

bool finiteActionOperation(const ExclusionAction& action) {
  return std::all_of(action.added_blocks.begin(), action.added_blocks.end(),
                     finiteBlockOperation);
}

std::string occurrenceIdentity(std::uint64_t duplicate_index,
                               const ExclusionAction& action,
                               const std::string& operation_identity) {
  std::ostringstream out;
  out << "ACTION_OCCURRENCE_V1|action=" << action.id.value()
      << "|duplicate_index=" << duplicate_index
      << "|operation=" << operation_identity;
  return out.str();
}

template <class T>
std::vector<T> sortedCopy(std::vector<T> values) {
  std::sort(values.begin(), values.end());
  return values;
}

}  // namespace

std::string toString(PairFamilySupport support) {
  switch (support) {
    case PairFamilySupport::Independent: return "INDEPENDENT";
    case PairFamilySupport::SharedParameters: return "SHARED_PARAMETERS";
    case PairFamilySupport::Unsupported: return "UNSUPPORTED";
  }
  return "UNKNOWN";
}

PairFamilySupport pairFamilySupport(const FaultModeBasis& left,
                                    const FaultModeBasis& right) {
  const bool left_uwb = left.sensor == SensorType::Uwb;
  const bool right_uwb = right.sensor == SensorType::Uwb;
  // two_uwb / imu_imu / same_device_multiaxis are declared NOT_IMPLEMENTED in
  // the fault manifest: the runtime must never silently concatenate them.
  if (left_uwb && right_uwb) return PairFamilySupport::Unsupported;
  if (!left_uwb && !right_uwb) return PairFamilySupport::Unsupported;
  // A UWB mode and an IMU mode share parameters when they claim the same
  // physical source and the same support (only reachable through a malformed
  // registry, but the check must be structural rather than assumed).
  if (left.physical_source_id == right.physical_source_id &&
      !left.physical_source_id.empty()) {
    return PairFamilySupport::SharedParameters;
  }
  return PairFamilySupport::Independent;
}

bool hypothesisParametersIndependent(
    const std::vector<FaultModeBasis>& modes,
    const std::vector<FaultModeId>& hypothesis_modes, std::string* reason) {
  std::vector<const FaultModeBasis*> parts;
  parts.reserve(hypothesis_modes.size());
  for (const auto id : hypothesis_modes) {
    const auto found = std::find_if(
        modes.begin(), modes.end(),
        [&](const FaultModeBasis& mode) { return mode.id.value() == id.value(); });
    if (found == modes.end()) {
      if (reason) *reason = "hypothesis references an unregistered mode";
      return false;
    }
    parts.push_back(&*found);
  }
  for (std::size_t i = 0; i < parts.size(); ++i) {
    for (std::size_t j = i + 1; j < parts.size(); ++j) {
      const auto support = pairFamilySupport(*parts[i], *parts[j]);
      if (support == PairFamilySupport::Unsupported) {
        if (reason) *reason = "unsupported pair family: concatenation refused";
        return false;
      }
      if (support == PairFamilySupport::SharedParameters) {
        if (reason) *reason = "shared parameters require shared semantics";
        return false;
      }
      // Deliberately do not apply a rank gate to an individual common group.
      // Two physically independent parameters may be collinear in one factor
      // and distinguished by another.  Conversely, a genuinely dangerous
      // global nullspace is a numerical monitorability result, not permission
      // to erase the declared hypothesis from the registry.  The evidence
      // evaluator therefore keeps every structurally supported pair and
      // classifies its global stacked response fail-closed.
    }
  }
  return true;
}

bool equivalentActionOperation(const ExclusionAction& left,
                               const ExclusionAction& right) {
  // NaN/Inf graph operations are invalid, never duplicates.  For finite
  // operations equality is equality of the complete canonical bytes, so
  // +0/-0 and every floating-point bit pattern remain distinct.
  return finiteActionOperation(left) && finiteActionOperation(right) &&
      exactActionOperationIdentityV1(left) ==
          exactActionOperationIdentityV1(right);
}

std::vector<FaultHypothesisV2> projectRemainingHypothesesForActionV1(
    const std::vector<FaultHypothesisV2>& hypotheses,
    const ExclusionAction& action) {
  std::vector<FaultHypothesisV2> remaining;
  remaining.reserve(hypotheses.size());
  for (const auto& hypothesis : hypotheses) {
    FaultHypothesisV2 residual = hypothesis;
    residual.modes.erase(std::remove_if(
        residual.modes.begin(), residual.modes.end(), [&](FaultModeId id) {
          return std::find(action.covered_modes.begin(),
                           action.covered_modes.end(), id) !=
              action.covered_modes.end();
        }), residual.modes.end());
    residual.units.erase(std::remove_if(
        residual.units.begin(), residual.units.end(), [&](FaultUnitId id) {
          return std::find(action.covered_units.begin(),
                           action.covered_units.end(), id) !=
              action.covered_units.end();
        }), residual.units.end());
    if (!residual.modes.empty() || !residual.units.empty()) {
      remaining.push_back(std::move(residual));
    }
  }
  return remaining;
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

bool bridgeGroupRepresentable(const LinearizedIntegrityWindow& window,
                              const PendingFactorGroup& group) {
  std::set<gtsam::Key> unique;
  for (const auto key : group.keys) {
    if (!unique.insert(key).second) continue;
    const gtsam::Symbol symbol(key);
    const char type = symbol.chr();
    const int dimension = type == 'v' ? 3 : 6;
    const int within = type == 'x' ? 0 : (type == 'v' ? 6 : 9);
    if ((type != 'x' && type != 'v' && type != 'b') ||
        symbol.index() < window.detector_first_epoch) {
      return false;
    }
    const std::size_t relative =
        symbol.index() - window.detector_first_epoch;
    if (relative > static_cast<std::size_t>(
                       std::numeric_limits<int>::max() / 15)) {
      return false;
    }
    const int target = static_cast<int>(relative * 15) + within;
    if (target < 0 || dimension > window.H.cols() - target) return false;
  }
  return true;
}

LinearizedFactorBlock bridgeBlock(const EpochTransaction& tx,
                                  const LinearizedIntegrityWindow& window,
                                  const Occurrence& occurrence,
                                  const PendingFactorGroup& group) {
  if (!tx.frozen_values) throw std::runtime_error("frozen values unavailable");
  if (!bridgeGroupRepresentable(window, group)) {
    throw std::runtime_error("bridge keys are outside the frozen window");
  }
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
    if (local_column < 0 || dimension > dense.first.cols() - local_column ||
        target < 0 || dimension > window.H.cols() - target) {
      throw std::runtime_error("bridge Jacobian placement is outside its matrix");
    }
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
      // A condensed historical occurrence has no explicit source rows in the
      // current window.  It is therefore not recoverable by a row-level
      // retained-principal-covariance replacement.  Preserve the mode and
      // fail this action closed instead of throwing while constructing the
      // complete production action census.
      if (!replacement || blockFor(window, occurrence->uwb->id) == nullptr) {
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
      // A recovery bridge that references a state already outside the frozen
      // detector window cannot be represented by this candidate matrix.  It
      // is missing provenance, not a writable negative/underflowed block.
      if (!bridgeGroupRepresentable(window, *occurrence->pose_bridge) ||
          !bridgeGroupRepresentable(window, *occurrence->bias_bridge)) {
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
  // C3 wiring (§6 item 3): the removed interval's data source and its model
  // error record are explicit fields; they are never carried implicitly by the
  // action model id.  `model_error_validated` stays false while the pipeline
  // has no trial-validated model error bound for these removals, so a consumer
  // can never mistake a declared model for a validated one.
  action.removal_data_source = std::string(
      mode.sensor == SensorType::Uwb ? "uwb_range:" : "imu_interval:") +
      mode.physical_source_id;
  action.model_error_record = action.added_blocks.empty()
      ? "model_error:none"
      : "whitening:" + action.added_blocks.front().whitening_model_id;
  action.model_error_validated = false;
  return action;
}

ExclusionAction unite(const std::vector<const ExclusionAction*>& parts,
                      bool copy_added_blocks = true) {
  ExclusionAction out;
  std::set<std::uint64_t> remove, add, modes, units;
  std::set<std::string> physical_sources;
  std::set<std::string> data_sources;
  std::set<std::string> model_records;
  bool any_part = false;
  bool all_validated = true;
  for (const auto* part : parts) {
    if (!part) continue;
    any_part = true;
    if (!part->removal_data_source.empty()) {
      data_sources.insert(part->removal_data_source);
    }
    if (!part->model_error_record.empty()) {
      model_records.insert(part->model_error_record);
    }
    all_validated = all_validated && part->model_error_validated;
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
        if (copy_added_blocks && i < part->added_blocks.size()) {
          out.added_blocks.push_back(part->added_blocks[i]);
        }
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
  // C3 wiring (§6 item 3): a union carries the union of the removal data
  // sources and model error records of its parts, so the provenance of every
  // removed interval survives the merge instead of being flattened onto one
  // model id string.
  for (const auto& item : data_sources) {
    out.removal_data_source += out.removal_data_source.empty() ? item : "|" + item;
  }
  for (const auto& item : model_records) {
    out.model_error_record += out.model_error_record.empty() ? item : "|" + item;
  }
  out.model_error_validated = any_part && all_validated;
  return out;
}

}  // namespace

namespace {

class CanonicalByteSinkV3 {
 public:
  virtual ~CanonicalByteSinkV3() = default;
  virtual bool append(const char* data, std::size_t size) = 0;
};

class CanonicalStringSinkV3 final : public CanonicalByteSinkV3 {
 public:
  bool append(const char* data, std::size_t size) override {
    bytes_.append(data, size);
    return true;
  }
  std::string take() { return std::move(bytes_); }
 private:
  std::string bytes_;
};

class CanonicalCountSinkV3 final : public CanonicalByteSinkV3 {
 public:
  bool append(const char*, std::size_t size) override {
    if (size > std::numeric_limits<std::uint64_t>::max() - size_) return false;
    size_ += size;
    return true;
  }
  std::uint64_t size() const { return size_; }
 private:
  std::uint64_t size_ = 0;
};

// Small attempt-local SHA-256 implementation.  It avoids adding a new shared
// library dependency to the frozen P0 ABI while giving sidecar indexes a
// cryptographic digest.  Digests are never treated as equality proofs.
class CanonicalSha256SinkV3 final : public CanonicalByteSinkV3 {
 public:
  bool append(const char* data, std::size_t size) override {
    if (finalized_) return false;
    if (size > std::numeric_limits<std::uint64_t>::max() - total_bytes_) {
      return false;
    }
    total_bytes_ += size;
    const auto* input = reinterpret_cast<const unsigned char*>(data);
    while (size != 0) {
      const std::size_t count = std::min(size, block_.size() - used_);
      std::memcpy(block_.data() + used_, input, count);
      used_ += count;
      input += count;
      size -= count;
      if (used_ == block_.size()) {
        transform(block_.data());
        used_ = 0;
      }
    }
    return true;
  }

  std::uint64_t length() const { return total_bytes_; }

  std::string finishHex() {
    if (!finalized_) {
      const std::uint64_t bit_count = total_bytes_ * 8;
      block_[used_++] = 0x80;
      if (used_ > 56) {
        std::fill(block_.begin() + used_, block_.end(), 0);
        transform(block_.data());
        used_ = 0;
      }
      std::fill(block_.begin() + used_, block_.begin() + 56, 0);
      for (int index = 0; index < 8; ++index) {
        block_[63 - index] = static_cast<unsigned char>(
            bit_count >> (8 * index));
      }
      transform(block_.data());
      finalized_ = true;
    }
    static constexpr char kHex[] = "0123456789abcdef";
    std::string out(64, '0');
    for (std::size_t word = 0; word < state_.size(); ++word) {
      for (std::size_t byte = 0; byte < 4; ++byte) {
        const unsigned char value = static_cast<unsigned char>(
            state_[word] >> (24 - 8 * byte));
        out[8 * word + 2 * byte] = kHex[value >> 4];
        out[8 * word + 2 * byte + 1] = kHex[value & 0x0f];
      }
    }
    return out;
  }

 private:
  static std::uint32_t rotate(std::uint32_t value, unsigned count) {
    return (value >> count) | (value << (32 - count));
  }
  void transform(const unsigned char* input) {
    static constexpr std::array<std::uint32_t, 64> k = {{
      0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,
      0x3956c25bu,0x59f111f1u,0x923f82a4u,0xab1c5ed5u,
      0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,
      0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,
      0xe49b69c1u,0xefbe4786u,0x0fc19dc6u,0x240ca1ccu,
      0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
      0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,
      0xc6e00bf3u,0xd5a79147u,0x06ca6351u,0x14292967u,
      0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,
      0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,
      0xa2bfe8a1u,0xa81a664bu,0xc24b8b70u,0xc76c51a3u,
      0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
      0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,
      0x391c0cb3u,0x4ed8aa4au,0x5b9cca4fu,0x682e6ff3u,
      0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,
      0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u}};
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16; ++index) {
      words[index] = (static_cast<std::uint32_t>(input[4 * index]) << 24) |
          (static_cast<std::uint32_t>(input[4 * index + 1]) << 16) |
          (static_cast<std::uint32_t>(input[4 * index + 2]) << 8) |
          static_cast<std::uint32_t>(input[4 * index + 3]);
    }
    for (std::size_t index = 16; index < words.size(); ++index) {
      const std::uint32_t s0 = rotate(words[index - 15], 7) ^
          rotate(words[index - 15], 18) ^ (words[index - 15] >> 3);
      const std::uint32_t s1 = rotate(words[index - 2], 17) ^
          rotate(words[index - 2], 19) ^ (words[index - 2] >> 10);
      words[index] = words[index - 16] + s0 + words[index - 7] + s1;
    }
    std::uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
    std::uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
    for (std::size_t index = 0; index < words.size(); ++index) {
      const std::uint32_t s1 = rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25);
      const std::uint32_t choose = (e & f) ^ (~e & g);
      const std::uint32_t temp1 = h + s1 + choose + k[index] + words[index];
      const std::uint32_t s0 = rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = s0 + majority;
      h = g; g = f; f = e; e = d + temp1;
      d = c; c = b; b = a; a = temp1 + temp2;
    }
    state_[0] += a; state_[1] += b; state_[2] += c; state_[3] += d;
    state_[4] += e; state_[5] += f; state_[6] += g; state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_{{
      0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,
      0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u}};
  std::array<unsigned char, 64> block_{};
  std::size_t used_ = 0;
  std::uint64_t total_bytes_ = 0;
  bool finalized_ = false;
};

bool appendCanonicalRawV3(CanonicalByteSinkV3* sink,
                          const std::string& value) {
  return sink && sink->append(value.data(), value.size());
}

template <class T>
bool appendCanonicalIntegralV3(CanonicalByteSinkV3* sink, T value) {
  std::array<char, 32> bytes{};
  const int count = std::snprintf(
      bytes.data(), bytes.size(), "%llx;",
      static_cast<unsigned long long>(static_cast<std::uint64_t>(value)));
  return count > 0 && static_cast<std::size_t>(count) < bytes.size() &&
      sink->append(bytes.data(), static_cast<std::size_t>(count));
}

bool appendCanonicalDoubleV3(CanonicalByteSinkV3* sink, double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "64-bit canonical double");
  std::memcpy(&bits, &value, sizeof(bits));
  return appendCanonicalIntegralV3(sink, bits);
}

class CanonicalHexSinkV3 final : public CanonicalByteSinkV3 {
 public:
  explicit CanonicalHexSinkV3(CanonicalByteSinkV3* destination)
      : destination_(destination) {}
  bool append(const char* data, std::size_t size) override {
    static constexpr char kHex[] = "0123456789abcdef";
    std::array<char, 8192> encoded{};
    std::size_t input = 0;
    while (input < size) {
      const std::size_t count = std::min(
          size - input, encoded.size() / 2);
      for (std::size_t index = 0; index < count; ++index) {
        const unsigned char value =
            static_cast<unsigned char>(data[input + index]);
        encoded[2 * index] = kHex[value >> 4];
        encoded[2 * index + 1] = kHex[value & 0x0f];
      }
      if (!destination_->append(encoded.data(), 2 * count)) return false;
      input += count;
    }
    return true;
  }
 private:
  CanonicalByteSinkV3* destination_ = nullptr;
};

bool appendCanonicalStringV3(CanonicalByteSinkV3* sink,
                             const std::string& value) {
  std::array<char, 32> prefix{};
  const int count = std::snprintf(
      prefix.data(), prefix.size(), "%llu:",
      static_cast<unsigned long long>(value.size()));
  if (count <= 0 || static_cast<std::size_t>(count) >= prefix.size() ||
      !sink->append(prefix.data(), static_cast<std::size_t>(count))) {
    return false;
  }
  CanonicalHexSinkV3 hex(sink);
  return hex.append(value.data(), value.size()) &&
      sink->append(";", 1);
}

template <class Derived>
bool appendCanonicalEigenV3(CanonicalByteSinkV3* sink,
                            const Eigen::MatrixBase<Derived>& value) {
  if (!appendCanonicalIntegralV3(sink, value.rows()) ||
      !appendCanonicalIntegralV3(sink, value.cols())) return false;
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    for (Eigen::Index column = 0; column < value.cols(); ++column) {
      if (!appendCanonicalDoubleV3(sink, value(row, column))) return false;
    }
  }
  return true;
}

bool emitCanonicalBlockV3(const LinearizedFactorBlock& block,
                          CanonicalByteSinkV3* sink) {
  if (!appendCanonicalIntegralV3(sink, block.group_id.value()) ||
      !appendCanonicalIntegralV3(sink, static_cast<int>(block.kind)) ||
      !appendCanonicalIntegralV3(sink, static_cast<int>(block.sensor)) ||
      !appendCanonicalIntegralV3(sink, static_cast<int>(block.role)) ||
      !appendCanonicalIntegralV3(sink, block.window_column_indices.size())) {
    return false;
  }
  for (const int column : block.window_column_indices) {
    if (!appendCanonicalIntegralV3(sink, column)) return false;
  }
  if (!appendCanonicalIntegralV3(sink, block.fault_units.size())) return false;
  for (const auto unit : block.fault_units) {
    if (!appendCanonicalIntegralV3(sink, unit.value())) return false;
  }
  return appendCanonicalDoubleV3(sink, block.effective_weight) &&
      appendCanonicalStringV3(sink, block.whitening_model_id) &&
      appendCanonicalIntegralV3(sink, block.version.graph_version) &&
      appendCanonicalIntegralV3(sink, block.version.ordering_version) &&
      appendCanonicalIntegralV3(sink, block.version.noise_model_version) &&
      appendCanonicalIntegralV3(sink, block.version.linpoint_version) &&
      appendCanonicalEigenV3(sink, block.jacobian_raw) &&
      appendCanonicalEigenV3(sink, block.residual_raw) &&
      appendCanonicalEigenV3(sink, block.covariance) &&
      appendCanonicalEigenV3(sink, block.whitener) &&
      appendCanonicalEigenV3(sink, block.jacobian_whitened) &&
      appendCanonicalEigenV3(sink, block.residual_whitened);
}

bool appendCanonicalBlockAsStringV3(CanonicalByteSinkV3* sink,
                                    const LinearizedFactorBlock& block) {
  CanonicalCountSinkV3 count;
  if (!emitCanonicalBlockV3(block, &count)) return false;
  std::array<char, 32> prefix{};
  const int prefix_size = std::snprintf(
      prefix.data(), prefix.size(), "%llu:",
      static_cast<unsigned long long>(count.size()));
  if (prefix_size <= 0 ||
      static_cast<std::size_t>(prefix_size) >= prefix.size() ||
      !sink->append(prefix.data(), static_cast<std::size_t>(prefix_size))) {
    return false;
  }
  CanonicalHexSinkV3 hex(sink);
  return emitCanonicalBlockV3(block, &hex) && sink->append(";", 1);
}

bool emitCanonicalActionOperationV3(const ExclusionAction& action,
                                    CanonicalByteSinkV3* sink) {
  if (!sink->append("ACTION_OPERATION_V1|", 20)) return false;
  auto remove = sortedCopy(action.groups_to_remove);
  auto add = sortedCopy(action.groups_to_add);
  if (!appendCanonicalIntegralV3(sink, remove.size())) return false;
  for (const auto id : remove) {
    if (!appendCanonicalIntegralV3(sink, id.value())) return false;
  }
  if (!appendCanonicalIntegralV3(sink, add.size())) return false;
  for (const auto id : add) {
    if (!appendCanonicalIntegralV3(sink, id.value())) return false;
  }
  if (!appendCanonicalIntegralV3(sink, static_cast<int>(action.bridge_mode)) ||
      !appendCanonicalIntegralV3(sink,
                                 static_cast<int>(action.recoverability)) ||
      !appendCanonicalIntegralV3(
          sink, action.recovery_epoch_begin.has_value())) return false;
  if (action.recovery_epoch_begin) {
    if (!appendCanonicalIntegralV3(sink, *action.recovery_epoch_begin)) {
      return false;
    }
  }
  if (!appendCanonicalIntegralV3(
          sink, action.recovery_epoch_end.has_value())) return false;
  if (action.recovery_epoch_end) {
    if (!appendCanonicalIntegralV3(sink, *action.recovery_epoch_end)) {
      return false;
    }
  }
  std::vector<const LinearizedFactorBlock*> blocks;
  blocks.reserve(action.added_blocks.size());
  for (const auto& block : action.added_blocks) {
    blocks.push_back(&block);
  }
  // Preserve V1's lexicographic ordering without retaining complete block
  // strings.  A valid action has unique added group IDs (enforced by
  // finiteActionOperation); the group-id encoding is the first canonical
  // field, so comparing that small prefix determines the complete ordering.
  auto block_less = [](const LinearizedFactorBlock* left,
                       const LinearizedFactorBlock* right) {
    std::array<char, 32> left_key{};
    std::array<char, 32> right_key{};
    std::snprintf(left_key.data(), left_key.size(), "%llx;",
                  static_cast<unsigned long long>(left->group_id.value()));
    std::snprintf(right_key.data(), right_key.size(), "%llx;",
                  static_cast<unsigned long long>(right->group_id.value()));
    return std::strcmp(left_key.data(), right_key.data()) < 0;
  };
  std::sort(blocks.begin(), blocks.end(), block_less);
  if (!appendCanonicalIntegralV3(sink, blocks.size())) return false;
  for (const auto* block : blocks) {
    if (!appendCanonicalBlockAsStringV3(sink, *block)) return false;
  }
  return appendCanonicalStringV3(sink, action.removal_data_source) &&
      appendCanonicalStringV3(sink, action.model_error_record) &&
      appendCanonicalIntegralV3(sink, action.model_error_validated);
}

bool emitCanonicalActionSemanticV3(const ExclusionAction& action,
                                   CanonicalByteSinkV3* sink) {
  CanonicalCountSinkV3 operation_count;
  if (!emitCanonicalActionOperationV3(action, &operation_count) ||
      !sink->append("ACTION_SEMANTICS_V1|", 20)) return false;
  std::array<char, 32> prefix{};
  const int prefix_size = std::snprintf(
      prefix.data(), prefix.size(), "%llu:",
      static_cast<unsigned long long>(operation_count.size()));
  if (prefix_size <= 0 ||
      static_cast<std::size_t>(prefix_size) >= prefix.size() ||
      !sink->append(prefix.data(), static_cast<std::size_t>(prefix_size))) {
    return false;
  }
  CanonicalHexSinkV3 operation_hex(sink);
  if (!emitCanonicalActionOperationV3(action, &operation_hex) ||
      !sink->append(";", 1) ||
      !appendCanonicalIntegralV3(sink, action.id.value()) ||
      !appendCanonicalStringV3(sink, action.action_model_id)) return false;
  const auto units = sortedCopy(action.covered_units);
  const auto modes = sortedCopy(action.covered_modes);
  const auto sources = sortedCopy(action.physical_source_ids);
  if (!appendCanonicalIntegralV3(sink, units.size())) return false;
  for (const auto id : units) {
    if (!appendCanonicalIntegralV3(sink, id.value())) return false;
  }
  if (!appendCanonicalIntegralV3(sink, modes.size())) return false;
  for (const auto id : modes) {
    if (!appendCanonicalIntegralV3(sink, id.value())) return false;
  }
  if (!appendCanonicalIntegralV3(sink, sources.size())) return false;
  for (const auto& source : sources) {
    if (!appendCanonicalStringV3(sink, source)) return false;
  }
  return appendCanonicalIntegralV3(sink, action.exclusion_cardinality);
}

}  // namespace

std::string exactActionOperationIdentityV1(const ExclusionAction& action) {
  // Keep the exported V1 serializer exactly as frozen.  The streaming V3
  // emitter is independently useful for compact sidecars, but substituting
  // it here would make any tiny implementation discrepancy an ABI-visible
  // identity change.
  std::ostringstream out;
  out << "ACTION_OPERATION_V1|";
  auto remove = sortedCopy(action.groups_to_remove);
  auto add = sortedCopy(action.groups_to_add);
  appendIntegralIdentity(&out, remove.size());
  for (const auto id : remove) appendIntegralIdentity(&out, id.value());
  appendIntegralIdentity(&out, add.size());
  for (const auto id : add) appendIntegralIdentity(&out, id.value());
  appendIntegralIdentity(&out, static_cast<int>(action.bridge_mode));
  appendIntegralIdentity(&out, static_cast<int>(action.recoverability));
  appendIntegralIdentity(&out, action.recovery_epoch_begin.has_value());
  if (action.recovery_epoch_begin) {
    appendIntegralIdentity(&out, *action.recovery_epoch_begin);
  }
  appendIntegralIdentity(&out, action.recovery_epoch_end.has_value());
  if (action.recovery_epoch_end) {
    appendIntegralIdentity(&out, *action.recovery_epoch_end);
  }
  std::vector<std::string> blocks;
  blocks.reserve(action.added_blocks.size());
  for (const auto& block : action.added_blocks) {
    blocks.push_back(exactBlockIdentity(block));
  }
  std::sort(blocks.begin(), blocks.end());
  appendIntegralIdentity(&out, blocks.size());
  for (const auto& block : blocks) appendStringIdentity(&out, block);
  appendStringIdentity(&out, action.removal_data_source);
  appendStringIdentity(&out, action.model_error_record);
  appendIntegralIdentity(&out, action.model_error_validated);
  return out.str();
}

namespace {

std::string exactActionSemanticIdentityFromOperationV1(
    const ExclusionAction& action, const std::string& operation_identity) {
  std::ostringstream out;
  out << "ACTION_SEMANTICS_V1|";
  appendStringIdentity(&out, operation_identity);
  appendIntegralIdentity(&out, action.id.value());
  appendStringIdentity(&out, action.action_model_id);
  const auto units = sortedCopy(action.covered_units);
  const auto modes = sortedCopy(action.covered_modes);
  const auto sources = sortedCopy(action.physical_source_ids);
  appendIntegralIdentity(&out, units.size());
  for (const auto id : units) appendIntegralIdentity(&out, id.value());
  appendIntegralIdentity(&out, modes.size());
  for (const auto id : modes) appendIntegralIdentity(&out, id.value());
  appendIntegralIdentity(&out, sources.size());
  for (const auto& source : sources) appendStringIdentity(&out, source);
  appendIntegralIdentity(&out, action.exclusion_cardinality);
  return out.str();
}

}  // namespace

std::string exactActionSemanticIdentityV1(const ExclusionAction& action) {
  const std::string operation = exactActionOperationIdentityV1(action);
  return exactActionSemanticIdentityFromOperationV1(action, operation);
}

namespace {

std::string exactGeneratedSnapshotIdentityV1(
    const std::vector<ExclusionAction>& generated) {
  std::vector<std::pair<std::string, std::string>> records;
  records.reserve(generated.size());
  for (const auto& action : generated) {
    std::string operation = exactActionOperationIdentityV1(action);
    std::string semantics =
        exactActionSemanticIdentityFromOperationV1(action, operation);
    records.emplace_back(std::move(semantics), std::move(operation));
  }
  std::sort(records.begin(), records.end());
  std::ostringstream out;
  out << "GENERATED_ACTION_SNAPSHOT_V1|";
  appendIntegralIdentity(&out, records.size());
  for (const auto& record : records) {
    appendStringIdentity(&out, record.first);
    appendStringIdentity(&out, record.second);
  }
  return out.str();
}

}  // namespace

ActionSearchResultV1 censusAndCapActionsV1(
    const std::vector<ExclusionAction>& generated,
    std::size_t max_evaluated_actions,
    GeneratedActionSnapshotV1* trusted_generated_snapshot) {
  ActionSearchResultV1 result;
  result.generated_snapshot.protocol_version = 1;
  result.generated_snapshot.generator_identity =
      "censusAndCapActionsV1/raw-generator-output";
  result.generated_snapshot.actions = generated;
  result.generated_snapshot.snapshot_identity =
      exactGeneratedSnapshotIdentityV1(generated);
  result.max_evaluated_actions = max_evaluated_actions;
  result.census.generated_snapshot_identity =
      result.generated_snapshot.snapshot_identity;
  result.census.generated = static_cast<std::uint64_t>(generated.size());
  result.census.generated_identities.reserve(generated.size());
  result.census.generated_records.reserve(generated.size());

  struct RawAction {
    const ExclusionAction* action = nullptr;
    std::string operation;
    std::string semantics;
    std::string occurrence;
  };
  std::vector<RawAction> raw;
  raw.reserve(generated.size());
  std::map<std::pair<std::uint64_t, std::string>, std::uint64_t>
      occurrence_counts;
  std::map<std::uint64_t, std::set<std::string>> semantics_by_action_id;
  for (std::size_t raw_index = 0; raw_index < generated.size(); ++raw_index) {
    const auto& action = generated[raw_index];
    const std::string operation = exactActionOperationIdentityV1(action);
    const std::string semantics =
        exactActionSemanticIdentityFromOperationV1(action, operation);
    const auto occurrence_key = std::make_pair(action.id.value(), semantics);
    const std::uint64_t duplicate_index = occurrence_counts[occurrence_key]++;
    const std::string occurrence = occurrenceIdentity(
        duplicate_index, action, semantics);
    raw.push_back({&action, operation, semantics, occurrence});
    semantics_by_action_id[action.id.value()].insert(semantics);
  }

  // Canonicalize the raw census and the representative set. Neither an exact
  // duplicate representative nor cap membership depends on input order.
  std::sort(raw.begin(), raw.end(), [](const RawAction& left,
                                      const RawAction& right) {
    return std::make_tuple(left.action->id.value(), left.semantics,
                           left.occurrence) <
        std::make_tuple(right.action->id.value(), right.semantics,
                        right.occurrence);
  });
  for (const auto& entry : raw) {
    result.census.generated_identities.push_back(entry.occurrence);
    ActionOccurrenceV1 record;
    record.action_id = entry.action->id.value();
    record.occurrence_identity = entry.occurrence;
    record.operation_identity = entry.operation;
    record.semantic_identity = entry.semantics;
    result.census.generated_records.push_back(std::move(record));
  }

  struct DistinctAction {
    ExclusionAction action;
    std::string occurrence;
    std::string operation;
    std::string semantics;
  };
  std::vector<DistinctAction> distinct;
  distinct.reserve(raw.size());
  for (std::size_t begin = 0; begin < raw.size();) {
    std::size_t end = begin + 1;
    while (end < raw.size() && raw[end].semantics == raw[begin].semantics) ++end;
    const RawAction& canonical = raw[begin];
    const bool id_conflict =
        semantics_by_action_id[canonical.action->id.value()].size() != 1;
    if (!finiteActionOperation(*canonical.action) || id_conflict) {
      const char* reason = id_conflict ? "DUPLICATE_ACTION_ID_CONFLICT"
                                       : "INVALID_OPERATION_ENCODING";
      for (std::size_t index = begin; index < end; ++index) {
        ActionOmissionV1 omission;
        omission.action_id = raw[index].action->id.value();
        omission.occurrence_identity = raw[index].occurrence;
        omission.operation_identity = raw[index].operation;
        omission.reason = reason;
        omission.semantic_identity = raw[index].semantics;
        result.census.omitted_actions.push_back(std::move(omission));
      }
    } else {
      distinct.push_back({*canonical.action, canonical.occurrence,
                          canonical.operation, canonical.semantics});
      for (std::size_t index = begin + 1; index < end; ++index) {
        ActionOmissionV1 omission;
        omission.action_id = raw[index].action->id.value();
        omission.occurrence_identity = raw[index].occurrence;
        omission.operation_identity = raw[index].operation;
        omission.duplicate_of_identity = canonical.occurrence;
        omission.reason = "EXACT_SEMANTIC_DUPLICATE";
        omission.proven_safe = true;
        omission.semantic_identity = raw[index].semantics;
        result.census.omitted_actions.push_back(std::move(omission));
      }
    }
    begin = end;
  }

  const std::size_t evaluated =
      std::min(max_evaluated_actions, distinct.size());
  result.actions.reserve(evaluated);
  result.census.evaluated = static_cast<std::uint64_t>(evaluated);
  result.census.evaluated_identities.reserve(evaluated);
  for (std::size_t index = 0; index < evaluated; ++index) {
    result.actions.push_back(distinct[index].action);
    result.census.evaluated_identities.push_back(distinct[index].occurrence);
  }
  for (std::size_t index = evaluated; index < distinct.size(); ++index) {
    ActionOmissionV1 omission;
    omission.action_id = distinct[index].action.id.value();
    omission.occurrence_identity = distinct[index].occurrence;
    omission.operation_identity = distinct[index].operation;
    omission.reason = "RESOURCE_CAP_UNPROVEN";
    omission.semantic_identity = distinct[index].semantics;
    result.census.omitted_actions.push_back(std::move(omission));
  }
  result.census.omitted = static_cast<std::uint64_t>(
      result.census.omitted_actions.size());
  result.census.exhaustive = std::all_of(
      result.census.omitted_actions.begin(),
      result.census.omitted_actions.end(),
      [](const ActionOmissionV1& omitted) { return omitted.proven_safe; });
  result.census.terminal_reason =
      result.census.exhaustive ? "COMPLETE" : "SEARCH_INCOMPLETE";
  if (trusted_generated_snapshot) {
    *trusted_generated_snapshot = result.generated_snapshot;
  }
  return result;
}

CompleteActionStreamV2 makeCompleteActionStreamV2(
    const std::vector<ExclusionAction>& generated,
    std::size_t batch_capacity,
    GeneratedActionSnapshotV1* trusted_generated_snapshot) {
  CompleteActionStreamV2 stream;
  stream.batch_capacity = batch_capacity;
  if (batch_capacity == 0) return stream;

  // A generated occurrence may be an exact semantic duplicate, so the raw
  // size is a safe cap that retains every distinct representative.  This does
  // not increase a correctness cap: V2 validates that the resulting census is
  // exhaustive before exposing any batch.
  stream.complete = censusAndCapActionsV1(
      generated, generated.size(), trusted_generated_snapshot);
  const std::size_t evaluated = stream.complete.actions.size();
  stream.batch_count = evaluated == 0
      ? 0 : 1 + (evaluated - 1) / batch_capacity;
  stream.peak_batch_actions = std::min(batch_capacity, evaluated);
  return stream;
}

ActionSearchValidationV1 validateCompleteActionStreamV2(
    const CompleteActionStreamV2& stream,
    const GeneratedActionSnapshotV1& trusted_generated_snapshot) {
  if (stream.protocol_version != 2) {
    return {false, false, "unsupported complete-action stream protocol"};
  }
  if (stream.batch_capacity == 0) {
    return {false, false, "complete-action stream batch capacity is zero"};
  }
  const ActionSearchValidationV1 base = validateActionSearchCensusV1(
      stream.complete.census, trusted_generated_snapshot,
      stream.complete.max_evaluated_actions, stream.complete.lifecycle,
      stream.complete.actions);
  if (!base.valid || !base.exhaustive) return base;
  const std::size_t evaluated = stream.complete.actions.size();
  const std::size_t expected_batches = evaluated == 0
      ? 0 : 1 + (evaluated - 1) / stream.batch_capacity;
  if (stream.batch_count != expected_batches ||
      stream.peak_batch_actions !=
          std::min(stream.batch_capacity, evaluated)) {
    return {false, false, "complete-action stream range metadata mismatch"};
  }
  return {true, true, "COMPLETE_STREAM"};
}

namespace {

std::uint64_t actionIdentityDigestV3(const std::string& bytes) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char value : bytes) {
    hash ^= value;
    hash *= 1099511628211ULL;
  }
  // Zero is reserved for an unavailable identity.
  return hash == 0 ? 1 : hash;
}

struct SidecarRangeV3 {
  std::uint64_t offset = 0;
  std::uint64_t length = 0;
  std::uint64_t digest = 0;
};

class AttemptSidecarFileV3 {
 public:
  AttemptSidecarFileV3() { openAnonymous(); }
  ~AttemptSidecarFileV3() {
    if (fd_ >= 0) {
      while (::close(fd_) != 0 && errno == EINTR) {}
    }
  }
  AttemptSidecarFileV3(const AttemptSidecarFileV3&) = delete;
  AttemptSidecarFileV3& operator=(const AttemptSidecarFileV3&) = delete;

  bool valid() const { return fd_ >= 0 && !failed_; }
  const std::string& failure() const { return failure_; }
  bool sealedAvailable() const {
    return valid() && sealed_ && !injected_read_failure_;
  }

  bool appendCanonical(
      const std::function<bool(CanonicalByteSinkV3*)>& emit,
      SidecarRangeV3* range) {
    if (!range || sealed_ || !valid()) return fail("sidecar append invalid");
    class FileSink final : public CanonicalByteSinkV3 {
     public:
      explicit FileSink(AttemptSidecarFileV3* file) : file_(file) {}
      bool append(const char* data, std::size_t size) override {
        if (!file_->writeAll(data, size)) return false;
        for (std::size_t index = 0; index < size; ++index) {
          digest_ ^= static_cast<unsigned char>(data[index]);
          digest_ *= 1099511628211ULL;
        }
        if (size > std::numeric_limits<std::uint64_t>::max() - length_) {
          return file_->fail("sidecar canonical length overflow");
        }
        length_ += size;
        return true;
      }
      std::uint64_t length() const { return length_; }
      std::uint64_t digest() const { return digest_ == 0 ? 1 : digest_; }
     private:
      AttemptSidecarFileV3* file_ = nullptr;
      std::uint64_t length_ = 0;
      std::uint64_t digest_ = 1469598103934665603ULL;
    } sink(this);
    range->offset = length_;
    if (!emit(&sink)) return fail("canonical serialization failed");
    range->length = sink.length();
    range->digest = sink.digest();
    return true;
  }

  bool seal(std::uint64_t* length, std::uint64_t* digest) {
    if (!valid() || sealed_) return fail("sidecar seal invalid");
    if (::fdatasync(fd_) != 0) return failErrno("sidecar fdatasync");
    struct stat status {};
    if (::fstat(fd_, &status) != 0 || status.st_size < 0 ||
        static_cast<std::uint64_t>(status.st_size) != length_) {
      return failErrno("sidecar size changed before seal");
    }
    std::uint64_t value = 0;
    if (!digestRange({0, length_, 0}, &value)) return false;
    sealed_ = true;
    sealed_length_ = length_;
    sealed_digest_ = value;
    if (length) *length = sealed_length_;
    if (digest) *digest = sealed_digest_;
    return true;
  }

  bool verifySealed() const {
    if (!valid() || !sealed_ || injected_read_failure_) {
      return const_cast<AttemptSidecarFileV3*>(this)->fail(
          "sidecar unavailable/read failure");
    }
    struct stat status {};
    if (::fstat(fd_, &status) != 0 || status.st_size < 0 ||
        static_cast<std::uint64_t>(status.st_size) != sealed_length_) {
      return const_cast<AttemptSidecarFileV3*>(this)->failErrno(
          "sealed sidecar length mismatch");
    }
    std::uint64_t digest = 0;
    return digestRange({0, sealed_length_, 0}, &digest) &&
        digest == sealed_digest_;
  }

  bool verifyRange(const SidecarRangeV3& range) const {
    std::uint64_t digest = 0;
    return sealed_ && range.offset <= sealed_length_ &&
        range.length <= sealed_length_ - range.offset &&
        digestRange(range, &digest) && digest == range.digest;
  }

  bool sameBytes(const SidecarRangeV3& left,
                 const SidecarRangeV3& right) const {
    if (!sealed_ || left.length != right.length ||
        left.offset > sealed_length_ || right.offset > sealed_length_ ||
        left.length > sealed_length_ - left.offset ||
        right.length > sealed_length_ - right.offset) return false;
    std::array<char, 4096> a{};
    std::array<char, 4096> b{};
    std::uint64_t done = 0;
    while (done < left.length) {
      const std::size_t count = static_cast<std::size_t>(std::min<std::uint64_t>(
          a.size(), left.length - done));
      if (!readExact(left.offset + done, a.data(), count) ||
          !readExact(right.offset + done, b.data(), count)) return false;
      if (std::memcmp(a.data(), b.data(), count) != 0) return false;
      done += count;
    }
    return true;
  }

  // Compare a freshly streamed canonical payload with an existing range.
  // This is intentionally usable before seal so content-addressed blocks can
  // be deduplicated without first appending a duplicate.  Reads are bounded
  // and a digest match is never accepted without this byte comparison.
  bool sameCanonical(
      const SidecarRangeV3& existing,
      const std::function<bool(CanonicalByteSinkV3*)>& emit) const {
    if (!valid() || existing.offset > length_ ||
        existing.length > length_ - existing.offset) return false;
    class CompareSink final : public CanonicalByteSinkV3 {
     public:
      CompareSink(const AttemptSidecarFileV3* file,
                  const SidecarRangeV3& range)
          : file_(file), range_(range) {}
      bool append(const char* data, std::size_t size) override {
        if (size > range_.length - compared_) return false;
        std::array<char, 4096> bytes{};
        std::size_t consumed = 0;
        while (consumed < size) {
          const std::size_t count = std::min(
              bytes.size(), size - consumed);
          if (!file_->readExact(range_.offset + compared_, bytes.data(),
                                count) ||
              std::memcmp(bytes.data(), data + consumed, count) != 0) {
            return false;
          }
          consumed += count;
          compared_ += count;
        }
        return true;
      }
      bool complete() const { return compared_ == range_.length; }
     private:
      const AttemptSidecarFileV3* file_ = nullptr;
      SidecarRangeV3 range_;
      std::uint64_t compared_ = 0;
    } sink(this, existing);
    return emit(&sink) && sink.complete();
  }

  bool copyRange(const SidecarRangeV3& range, std::string* bytes) const {
    if (!bytes || !sealed_ || range.offset > sealed_length_ ||
        range.length > sealed_length_ - range.offset ||
        range.length > std::numeric_limits<std::size_t>::max()) return false;
    bytes->resize(static_cast<std::size_t>(range.length));
    return readExact(range.offset, bytes->data(), bytes->size());
  }

  void injectReadFailureForTest() { injected_read_failure_ = true; }

 private:
  void openAnonymous() {
#if defined(O_TMPFILE)
    fd_ = ::open("/tmp", O_TMPFILE | O_RDWR | O_CLOEXEC, 0600);
#endif
    if (fd_ >= 0) return;
    char path[] = "/tmp/uwb_imu_pl_action_sidecar.XXXXXX";
#if defined(O_CLOEXEC)
    fd_ = ::mkostemp(path, O_CLOEXEC);
#else
    fd_ = ::mkstemp(path);
#endif
    if (fd_ < 0) {
      failErrno("sidecar temporary file creation");
      return;
    }
    if (::fchmod(fd_, 0600) != 0) {
      failErrno("sidecar chmod");
      ::close(fd_);
      fd_ = -1;
      ::unlink(path);
      return;
    }
    if (::unlink(path) != 0) {
      failErrno("sidecar unlink");
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool writeAll(const char* data, std::size_t size) {
    std::size_t done = 0;
    while (done < size) {
      const ssize_t count = ::write(fd_, data + done, size - done);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) return failErrno("sidecar write");
      done += static_cast<std::size_t>(count);
      length_ += static_cast<std::size_t>(count);
    }
    return true;
  }

  bool readExact(std::uint64_t offset, char* data, std::size_t size) const {
    if (injected_read_failure_ || fd_ < 0 ||
        offset > static_cast<std::uint64_t>(std::numeric_limits<off_t>::max())) {
      return const_cast<AttemptSidecarFileV3*>(this)->fail(
          "sidecar read unavailable");
    }
    std::size_t done = 0;
    while (done < size) {
      const ssize_t count = ::pread(
          fd_, data + done, size - done,
          static_cast<off_t>(offset + done));
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) {
        return const_cast<AttemptSidecarFileV3*>(this)->failErrno(
            "sidecar short read");
      }
      done += static_cast<std::size_t>(count);
    }
    return true;
  }

  bool digestRange(const SidecarRangeV3& range,
                   std::uint64_t* digest) const {
    if (!digest || range.offset > length_ ||
        range.length > length_ - range.offset) return false;
    std::uint64_t value = 1469598103934665603ULL;
    std::array<char, 4096> bytes{};
    std::uint64_t done = 0;
    while (done < range.length) {
      const std::size_t count = static_cast<std::size_t>(std::min<std::uint64_t>(
          bytes.size(), range.length - done));
      if (!readExact(range.offset + done, bytes.data(), count)) return false;
      for (std::size_t index = 0; index < count; ++index) {
        value ^= static_cast<unsigned char>(bytes[index]);
        value *= 1099511628211ULL;
      }
      done += count;
    }
    *digest = value == 0 ? 1 : value;
    return true;
  }

  bool fail(const std::string& reason) {
    failed_ = true;
    if (failure_.empty()) failure_ = reason;
    return false;
  }
  bool failErrno(const std::string& reason) {
    const int error = errno;
    return fail(reason + ": errno=" + std::to_string(error));
  }

  int fd_ = -1;
  std::uint64_t length_ = 0;
  std::uint64_t sealed_length_ = 0;
  std::uint64_t sealed_digest_ = 0;
  bool sealed_ = false;
  mutable bool failed_ = false;
  mutable std::string failure_;
  bool injected_read_failure_ = false;
};

SidecarRangeV3 operationRangeV3(const CompactActionDescriptorV3& descriptor) {
  return {descriptor.operation_offset, descriptor.operation_length,
          descriptor.operation_digest};
}

SidecarRangeV3 semanticRangeV3(const CompactActionDescriptorV3& descriptor) {
  return {descriptor.semantic_offset, descriptor.semantic_length,
          descriptor.semantic_digest};
}

struct CanonicalMetadataV3 {
  std::uint64_t length = 0;
  std::string strong_digest;
};

CanonicalMetadataV3 canonicalMetadataV3(
    const std::function<bool(CanonicalByteSinkV3*)>& emit) {
  CanonicalSha256SinkV3 sink;
  if (!emit(&sink)) return {};
  return {sink.length(), sink.finishHex()};
}

std::vector<const LinearizedFactorBlock*> orderedActionBlocksV3(
    const ExclusionAction& action) {
  std::vector<const LinearizedFactorBlock*> blocks;
  blocks.reserve(action.added_blocks.size());
  for (const auto& block : action.added_blocks) blocks.push_back(&block);
  std::sort(blocks.begin(), blocks.end(),
      [](const LinearizedFactorBlock* left,
         const LinearizedFactorBlock* right) {
        std::array<char, 32> left_key{};
        std::array<char, 32> right_key{};
        std::snprintf(left_key.data(), left_key.size(), "%llx;",
                      static_cast<unsigned long long>(
                          left->group_id.value()));
        std::snprintf(right_key.data(), right_key.size(), "%llx;",
                      static_cast<unsigned long long>(
                          right->group_id.value()));
        return std::strcmp(left_key.data(), right_key.data()) < 0;
      });
  return blocks;
}

bool emitCompactOperationRecordV3(
    const ExclusionAction& action,
    const CompactActionDescriptorV3& descriptor,
    CanonicalByteSinkV3* sink) {
  if (!sink->append("COMPACT_ACTION_OPERATION_V3|", 28)) return false;
  auto remove = sortedCopy(action.groups_to_remove);
  auto add = sortedCopy(action.groups_to_add);
  if (!appendCanonicalIntegralV3(sink, remove.size())) return false;
  for (const auto id : remove) {
    if (!appendCanonicalIntegralV3(sink, id.value())) return false;
  }
  if (!appendCanonicalIntegralV3(sink, add.size())) return false;
  for (const auto id : add) {
    if (!appendCanonicalIntegralV3(sink, id.value())) return false;
  }
  if (!appendCanonicalIntegralV3(sink, static_cast<int>(action.bridge_mode)) ||
      !appendCanonicalIntegralV3(sink,
                                 static_cast<int>(action.recoverability)) ||
      !appendCanonicalIntegralV3(
          sink, action.recovery_epoch_begin.has_value())) return false;
  if (action.recovery_epoch_begin &&
      !appendCanonicalIntegralV3(sink, *action.recovery_epoch_begin)) {
    return false;
  }
  if (!appendCanonicalIntegralV3(
          sink, action.recovery_epoch_end.has_value())) return false;
  if (action.recovery_epoch_end &&
      !appendCanonicalIntegralV3(sink, *action.recovery_epoch_end)) {
    return false;
  }
  if (!appendCanonicalIntegralV3(sink, descriptor.ordered_blocks.size())) {
    return false;
  }
  for (const auto& block : descriptor.ordered_blocks) {
    if (!appendCanonicalIntegralV3(sink, block.block_id) ||
        !appendCanonicalIntegralV3(sink, block.offset) ||
        !appendCanonicalIntegralV3(sink, block.length) ||
        !appendCanonicalStringV3(sink, block.strong_digest)) return false;
  }
  return appendCanonicalStringV3(sink, action.removal_data_source) &&
      appendCanonicalStringV3(sink, action.model_error_record) &&
      appendCanonicalIntegralV3(sink, action.model_error_validated);
}

bool emitCompactSemanticRecordV3(
    const ExclusionAction& action,
    const CompactActionDescriptorV3& descriptor,
    CanonicalByteSinkV3* sink) {
  if (!sink->append("COMPACT_ACTION_SEMANTICS_V3|", 28) ||
      !emitCompactOperationRecordV3(action, descriptor, sink) ||
      !appendCanonicalIntegralV3(sink, action.id.value()) ||
      !appendCanonicalStringV3(sink, action.action_model_id)) return false;
  const auto units = sortedCopy(action.covered_units);
  const auto modes = sortedCopy(action.covered_modes);
  const auto sources = sortedCopy(action.physical_source_ids);
  if (!appendCanonicalIntegralV3(sink, units.size())) return false;
  for (const auto id : units) {
    if (!appendCanonicalIntegralV3(sink, id.value())) return false;
  }
  if (!appendCanonicalIntegralV3(sink, modes.size())) return false;
  for (const auto id : modes) {
    if (!appendCanonicalIntegralV3(sink, id.value())) return false;
  }
  if (!appendCanonicalIntegralV3(sink, sources.size())) return false;
  for (const auto& source : sources) {
    if (!appendCanonicalStringV3(sink, source)) return false;
  }
  return appendCanonicalIntegralV3(sink, action.exclusion_cardinality);
}

}  // namespace

struct AttemptActionLeaseV3::State {
  CompactActionSearchV3 search;
  std::function<ExclusionAction(const CompactActionDescriptorV3&)>
      materialize_one;
  std::function<ExclusionAction(const CompactActionDescriptorV3&)>
      materialize_metadata;
  std::shared_ptr<AttemptSidecarFileV3> sidecar;
  bool producer_certificate_valid = false;
};

namespace {

std::uint64_t compactSnapshotDigestV3(
    const std::vector<CompactActionDescriptorV3>& descriptors,
    const std::vector<CompactActionOmissionV3>& omissions,
    std::uint64_t generated, std::uint64_t sidecar_digest,
    std::uint64_t sidecar_length) {
  using Row = std::tuple<std::uint64_t, std::uint64_t, std::uint64_t,
                         std::uint64_t, std::uint64_t, std::uint64_t,
                         std::uint64_t, std::uint64_t, std::uint64_t, bool>;
  std::vector<Row> rows;
  rows.reserve(descriptors.size() + omissions.size());
  for (const auto& descriptor : descriptors) {
    rows.emplace_back(descriptor.action_id, descriptor.operation_digest,
                      descriptor.semantic_digest, descriptor.operation_offset,
                      descriptor.operation_length, descriptor.semantic_offset,
                      descriptor.semantic_length, 0, 0, true);
  }
  for (const auto& omission : omissions) {
    const auto& descriptor = omission.occurrence;
    rows.emplace_back(descriptor.action_id, descriptor.operation_digest,
                      descriptor.semantic_digest, descriptor.operation_offset,
                      descriptor.operation_length, descriptor.semantic_offset,
                      descriptor.semantic_length, 0,
                      omission.representative_action_id,
                      omission.proven_safe);
  }
  std::sort(rows.begin(), rows.end());
  std::ostringstream bytes;
  bytes << "COMPACT_ACTION_SNAPSHOT_V3|" << generated << '|'
        << sidecar_digest << '|' << sidecar_length << '|';
  for (const auto& row : rows) {
    bytes << std::get<0>(row) << ':' << std::get<1>(row) << ':'
          << std::get<2>(row) << ':' << std::get<3>(row) << ':'
          << std::get<4>(row) << ':' << std::get<5>(row) << ':'
          << std::get<6>(row) << ':' << std::get<7>(row) << ':'
          << std::get<8>(row) << ':' << std::get<9>(row) << ';';
  }
  return actionIdentityDigestV3(bytes.str());
}

std::size_t materializedBlockHeavyBytesV3(
    const LinearizedFactorBlock& block) {
  std::size_t bytes = 0;
  auto matrix_bytes = [&](const auto& value) {
    const auto size = static_cast<std::size_t>(value.size());
    if (size > (std::numeric_limits<std::size_t>::max() - bytes) /
                   sizeof(double)) {
      throw std::overflow_error("compact action heavy-byte count overflow");
    }
    bytes += size * sizeof(double);
  };
  matrix_bytes(block.jacobian_raw);
  matrix_bytes(block.residual_raw);
  matrix_bytes(block.covariance);
  matrix_bytes(block.whitener);
  matrix_bytes(block.jacobian_whitened);
  matrix_bytes(block.residual_whitened);
  bytes += block.window_column_indices.size() * sizeof(int);
  bytes += block.fault_units.size() * sizeof(FaultUnitId);
  return bytes;
}

std::size_t materializedActionHeavyBytesV3(const ExclusionAction& action) {
  std::size_t bytes = 0;
  for (const auto& block : action.added_blocks) {
    const std::size_t block_bytes = materializedBlockHeavyBytesV3(block);
    if (block_bytes > std::numeric_limits<std::size_t>::max() - bytes) {
      throw std::overflow_error("compact action heavy-byte count overflow");
    }
    bytes += block_bytes;
  }
  return bytes;
}

bool validateProducedDescriptorV3(
    const std::shared_ptr<AttemptSidecarFileV3>& sidecar,
    const CompactActionDescriptorV3& descriptor) {
  if (!sidecar || !sidecar->verifyRange(operationRangeV3(descriptor)) ||
      !sidecar->verifyRange(semanticRangeV3(descriptor))) return false;
  return std::all_of(descriptor.ordered_blocks.begin(),
                     descriptor.ordered_blocks.end(),
      [](const CompactActionDescriptorV3::BlockReference& reference) {
        return reference.block_id != 0 && reference.length != 0 &&
            reference.strong_digest.size() == 64;
      });
}

}  // namespace

std::vector<ExclusionAction> AttemptActionLeaseV3::materializeBatch(
    std::size_t begin, std::size_t count) const {
  if (!state_ || !state_->materialize_one || begin > state_->search.actions.size() ||
      count > state_->search.actions.size() - begin ||
      count > state_->search.batch_capacity) {
    throw std::runtime_error("compact action lease batch is invalid");
  }
  std::vector<ExclusionAction> actions;
  actions.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    actions.push_back(state_->materialize_one(
        state_->search.actions[begin + index]));
  }
  return actions;
}

ExclusionAction AttemptActionLeaseV3::materialize(std::size_t index) const {
  auto batch = materializeBatch(index, 1);
  return std::move(batch.front());
}

ExclusionAction AttemptActionLeaseV3::materializeMetadata(
    std::size_t index) const {
  if (!state_ || !state_->materialize_metadata ||
      index >= state_->search.actions.size()) {
    throw std::runtime_error("compact action metadata lease is invalid");
  }
  return state_->materialize_metadata(state_->search.actions[index]);
}

const CompactActionSearchV3& AttemptActionLeaseV3::search() const {
  if (!state_) throw std::runtime_error("compact action lease is closed");
  return state_->search;
}

bool AttemptActionLeaseV3::valid() const {
  return state_ && state_->materialize_one;
}

void AttemptActionLeaseV3::close() { state_.reset(); }

AttemptActionLeaseV3 AttemptActionArenaV3::lease() const {
  return AttemptActionLeaseV3(state_);
}

const CompactActionSearchV3& AttemptActionArenaV3::search() const {
  if (!state_) throw std::runtime_error("compact action arena is closed");
  return state_->search;
}

bool AttemptActionArenaV3::valid() const { return state_ != nullptr; }

void AttemptActionArenaV3::close() { state_.reset(); }

void AttemptActionArenaV3::injectSidecarReadFailureForTest() {
  if (state_ && state_->sidecar) state_->sidecar->injectReadFailureForTest();
}

ActionSearchValidationV1 validateCompactActionSearchV3(
    const AttemptActionLeaseV3& lease) {
  if (!lease.valid()) return {false, false, "compact action lease missing"};
  const auto state = lease.state_;
  const auto& search = lease.search();
  if (search.protocol_version != 3 || search.batch_capacity == 0) {
    return {false, false, "compact action protocol/capacity invalid"};
  }
  if (!state || !state->sidecar || !state->producer_certificate_valid ||
      !search.producer_certificate_valid ||
      !state->sidecar->sealedAvailable()) {
    return {false, false, "compact action sidecar/certificate invalid"};
  }
  if (search.generated != search.evaluated + search.omitted ||
      search.evaluated != search.actions.size() ||
      search.omitted != search.omitted_actions.size() ||
      search.batch_count != (search.actions.empty()
          ? 0 : 1 + (search.actions.size() - 1) / search.batch_capacity) ||
      search.peak_batch_actions !=
          std::min(search.batch_capacity, search.actions.size())) {
    return {false, false, "compact action census/ranges invalid"};
  }
  std::set<std::uint64_t> action_ids;
  std::set<std::uint64_t> referenced_block_ids;
  std::uint64_t referenced_block_count = 0;
  for (std::size_t index = 0; index < search.actions.size(); ++index) {
    const auto& descriptor = search.actions[index];
    if (descriptor.action_id == 0 ||
        !action_ids.insert(descriptor.action_id).second) {
      return {false, false, "compact action id invalid/duplicate"};
    }
    if (!state->sidecar->verifyRange(operationRangeV3(descriptor)) ||
        !state->sidecar->verifyRange(semanticRangeV3(descriptor))) {
      return {false, false,
              "compact descriptor canonical range invalid"};
    }
    for (const auto& block : descriptor.ordered_blocks) {
      referenced_block_ids.insert(block.block_id);
      ++referenced_block_count;
    }
    (void)state->materialize_metadata(descriptor);
  }
  for (const auto& omission : search.omitted_actions) {
    for (const auto& block : omission.occurrence.ordered_blocks) {
      referenced_block_ids.insert(block.block_id);
      ++referenced_block_count;
    }
    const auto representative = std::find_if(
        search.actions.begin(), search.actions.end(),
        [&](const CompactActionDescriptorV3& descriptor) {
          return descriptor.action_id == omission.representative_action_id;
        });
    const bool exact_duplicate = representative != search.actions.end() &&
        state->sidecar->sameBytes(operationRangeV3(omission.occurrence),
                                  operationRangeV3(*representative)) &&
        state->sidecar->sameBytes(semanticRangeV3(omission.occurrence),
                                  semanticRangeV3(*representative));
    const bool safe = omission.reason == "EXACT_SEMANTIC_DUPLICATE" &&
        exact_duplicate;
    if (safe != omission.proven_safe) {
      return {false, false, "compact omission proof invalid"};
    }
  }
  if (search.unique_block_count != referenced_block_ids.size() ||
      search.referenced_block_count != referenced_block_count) {
    return {false, false, "compact block census mismatch"};
  }
  if (compactSnapshotDigestV3(
          search.actions, search.omitted_actions, search.generated,
          search.sidecar_digest, search.sidecar_length) !=
      search.snapshot_digest) {
    return {false, false, "compact snapshot digest mismatch"};
  }
  std::size_t derived_peak_bytes = 0;
  for (std::size_t begin = 0; begin < search.actions.size();
       begin += search.batch_capacity) {
    std::size_t batch_bytes = 0;
    const std::size_t end = std::min(
        search.actions.size(), begin + search.batch_capacity);
    for (std::size_t index = begin; index < end; ++index) {
      const std::size_t value =
          search.actions[index].materialized_heavy_bytes;
      if (value > std::numeric_limits<std::size_t>::max() - batch_bytes) {
        return {false, false, "compact batch heavy-byte count overflow"};
      }
      batch_bytes += value;
    }
    derived_peak_bytes = std::max(derived_peak_bytes, batch_bytes);
  }
  if (derived_peak_bytes != search.peak_batch_heavy_bytes) {
    return {false, false, "compact batch heavy-byte metadata mismatch"};
  }
  const bool all_omissions_safe = std::all_of(
      search.omitted_actions.begin(), search.omitted_actions.end(),
      [](const CompactActionOmissionV3& omission) {
        return omission.proven_safe;
      });
  if (search.exhaustive != all_omissions_safe ||
      search.terminal_reason != (search.exhaustive ? "COMPLETE"
                                                   : "SEARCH_INCOMPLETE")) {
    return {false, false, "compact terminal completeness mismatch"};
  }
  return {true, search.exhaustive, "COMPLETE_COMPACT_STREAM"};
}

ActionSearchValidationV1 validateCompactActionConsumptionV3(
    const AttemptActionLeaseV3& lease,
    const std::vector<ExclusionAction>& consumed) {
  const ActionSearchValidationV1 search =
      validateCompactActionSearchV3(lease);
  if (!search.valid || !search.exhaustive) return search;
  if (consumed.size() != lease.search().actions.size()) {
    return {false, false, "compact consumer action count mismatch"};
  }
  for (std::size_t index = 0; index < consumed.size(); ++index) {
    const ExclusionAction exact = lease.materializeMetadata(index);
    const ExclusionAction& compact = consumed[index];
    const bool same = compact.id == exact.id &&
        compact.covered_units == exact.covered_units &&
        compact.covered_modes == exact.covered_modes &&
        compact.physical_source_ids == exact.physical_source_ids &&
        compact.groups_to_remove == exact.groups_to_remove &&
        compact.groups_to_add == exact.groups_to_add &&
        compact.bridge_mode == exact.bridge_mode &&
        compact.exclusion_cardinality == exact.exclusion_cardinality &&
        compact.action_model_id == exact.action_model_id &&
        compact.recoverability == exact.recoverability &&
        compact.recovery_epoch_begin == exact.recovery_epoch_begin &&
        compact.recovery_epoch_end == exact.recovery_epoch_end &&
        compact.removal_data_source == exact.removal_data_source &&
        compact.model_error_record == exact.model_error_record &&
        compact.model_error_validated == exact.model_error_validated;
    if (!same) {
      return {false, false,
              "compact consumer action differs from exact recipe"};
    }
  }
  return {true, true, "COMPLETE_COMPACT_CONSUMPTION"};
}

namespace {

void abortIncompleteActionSearchUnchecked(ActionSearchResultV1* result) {
  result->actions.clear();
  result->census.evaluated = 0;
  result->census.evaluated_identities.clear();
  const auto original_omissions = result->census.omitted_actions;
  result->census.omitted_actions.clear();
  for (const auto& record : result->census.generated_records) {
    const auto original = std::find_if(
        original_omissions.begin(), original_omissions.end(),
        [&](const ActionOmissionV1& omission) {
          return omission.occurrence_identity == record.occurrence_identity;
        });
    if (original != original_omissions.end() &&
        original->reason == "EXACT_SEMANTIC_DUPLICATE") {
      result->census.omitted_actions.push_back(*original);
    } else {
      const std::string reason = original != original_omissions.end()
          ? original->reason : "SEARCH_ABORTED_INCOMPLETE";
      result->census.omitted_actions.push_back({
          record.action_id, record.occurrence_identity,
          record.operation_identity, {}, reason, false,
          record.semantic_identity});
    }
  }
  result->census.omitted = result->census.generated;
  result->census.exhaustive = false;
  result->census.terminal_reason = "SEARCH_INCOMPLETE";
  result->lifecycle =
      ActionSearchLifecycleV1::AbortedIncompleteBeforeEvaluation;
}

bool sameOccurrence(const ActionOccurrenceV1& left,
                    const ActionOccurrenceV1& right) {
  return left.action_id == right.action_id &&
      left.occurrence_identity == right.occurrence_identity &&
      left.operation_identity == right.operation_identity &&
      left.semantic_identity == right.semantic_identity;
}

bool sameOmission(const ActionOmissionV1& left,
                  const ActionOmissionV1& right) {
  return left.action_id == right.action_id &&
      left.occurrence_identity == right.occurrence_identity &&
      left.operation_identity == right.operation_identity &&
      left.duplicate_of_identity == right.duplicate_of_identity &&
      left.reason == right.reason && left.proven_safe == right.proven_safe &&
      left.semantic_identity == right.semantic_identity;
}

bool sameCensus(const ActionSearchCensusV1& left,
                const ActionSearchCensusV1& right) {
  if (left.protocol_version != right.protocol_version ||
      left.generated != right.generated || left.evaluated != right.evaluated ||
      left.omitted != right.omitted || left.exhaustive != right.exhaustive ||
      left.generated_identities != right.generated_identities ||
      left.evaluated_identities != right.evaluated_identities ||
      left.generated_snapshot_identity != right.generated_snapshot_identity ||
      left.terminal_reason != right.terminal_reason ||
      left.generated_records.size() != right.generated_records.size() ||
      left.omitted_actions.size() != right.omitted_actions.size()) {
    return false;
  }
  for (std::size_t i = 0; i < left.generated_records.size(); ++i) {
    if (!sameOccurrence(left.generated_records[i], right.generated_records[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < left.omitted_actions.size(); ++i) {
    if (!sameOmission(left.omitted_actions[i], right.omitted_actions[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

ActionSearchValidationV1 validateActionSearchCensusV1(
    const ActionSearchCensusV1& census,
    const GeneratedActionSnapshotV1& trusted_generated_snapshot,
    std::size_t max_evaluated_actions,
    ActionSearchLifecycleV1 lifecycle,
    const std::vector<ExclusionAction>& actual_evaluated_actions) {
  auto fail = [](const std::string& reason) {
    return ActionSearchValidationV1{false, false, reason};
  };
  if (census.protocol_version != 1) return fail("unsupported protocol_version");
  if (trusted_generated_snapshot.protocol_version != 1 ||
      trusted_generated_snapshot.generator_identity.empty()) {
    return fail("trusted raw snapshot version/generator identity invalid");
  }
  if (trusted_generated_snapshot.actions.size() >
      static_cast<std::size_t>(std::numeric_limits<std::uint64_t>::max())) {
    return fail("raw generated action count overflows protocol");
  }
  if (census.generated != census.generated_records.size() ||
      census.generated != census.generated_identities.size()) {
    return fail("generated record count mismatch");
  }
  if (census.evaluated != census.evaluated_identities.size() ||
      census.omitted != census.omitted_actions.size()) {
    return fail("terminal record count mismatch");
  }
  if (census.evaluated > std::numeric_limits<std::uint64_t>::max() -
          census.omitted || census.evaluated + census.omitted != census.generated) {
    return fail("checked census arithmetic mismatch/overflow");
  }
  std::map<std::string, const ActionOccurrenceV1*> generated;
  for (std::size_t i = 0; i < census.generated_records.size(); ++i) {
    const auto& record = census.generated_records[i];
    if (record.occurrence_identity.empty() || record.operation_identity.empty() ||
        record.semantic_identity.empty() ||
        record.occurrence_identity != census.generated_identities[i] ||
        !generated.emplace(record.occurrence_identity, &record).second) {
      return fail("generated occurrence identity invalid/duplicate");
    }
  }
  std::set<std::string> terminal;
  for (const auto& identity : census.evaluated_identities) {
    if (!generated.count(identity) || !terminal.insert(identity).second) {
      return fail("evaluated identity absent/duplicate");
    }
  }
  std::map<std::string, const ActionOmissionV1*> omission_index;
  for (const auto& omission : census.omitted_actions) {
    if (!omission_index.emplace(omission.occurrence_identity, &omission).second) {
      return fail("duplicate omission occurrence identity");
    }
  }
  bool derived_exhaustive = true;
  for (const auto& omission : census.omitted_actions) {
    const auto source = generated.find(omission.occurrence_identity);
    if (source == generated.end() ||
        source->second->action_id != omission.action_id ||
        source->second->operation_identity != omission.operation_identity ||
        source->second->semantic_identity != omission.semantic_identity ||
        !terminal.insert(omission.occurrence_identity).second) {
      return fail("omission record does not bind generated occurrence");
    }
    bool safe = false;
    if (omission.reason == "EXACT_SEMANTIC_DUPLICATE") {
      const auto retained = generated.find(omission.duplicate_of_identity);
      const auto retained_omission = omission_index.find(
          omission.duplicate_of_identity);
      const bool retained_is_canonical_occurrence =
          retained_omission == omission_index.end() ||
          retained_omission->second->reason != "EXACT_SEMANTIC_DUPLICATE";
      safe = retained != generated.end() &&
          retained_is_canonical_occurrence &&
          retained->second->operation_identity == omission.operation_identity &&
          retained->second->semantic_identity == omission.semantic_identity;
    } else if (!omission.duplicate_of_identity.empty()) {
      return fail("unsafe omission carries duplicate proof");
    }
    if (omission.proven_safe != safe) {
      return fail("omission proof-safety flag is not derived");
    }
    derived_exhaustive = derived_exhaustive && safe;
  }
  if (terminal.size() != generated.size()) return fail("unaccounted occurrence");
  {
    if (actual_evaluated_actions.size() != census.evaluated_identities.size()) {
      return fail("evaluated action vector count mismatch");
    }
    for (std::size_t i = 0; i < actual_evaluated_actions.size(); ++i) {
      const auto record = generated.find(census.evaluated_identities[i]);
      const std::string operation =
          exactActionOperationIdentityV1(actual_evaluated_actions[i]);
      const std::string semantics = exactActionSemanticIdentityFromOperationV1(
          actual_evaluated_actions[i], operation);
      if (record == generated.end() ||
          record->second->action_id != actual_evaluated_actions[i].id.value() ||
          record->second->operation_identity != operation ||
          record->second->semantic_identity != semantics ||
          !finiteActionOperation(actual_evaluated_actions[i])) {
        return fail("evaluated action does not match census bytes");
      }
    }
  }
  if (census.exhaustive != derived_exhaustive ||
      census.terminal_reason !=
          (derived_exhaustive ? "COMPLETE" : "SEARCH_INCOMPLETE")) {
    return fail("producer completeness claim disagrees with derived result");
  }
  // The preceding checks make malformed counts safe to inspect.  This final
  // independent regeneration is the trust boundary: operation bytes,
  // semantic bytes, occurrence numbering, duplicate representative, cap and
  // lifecycle all have to match the complete raw action objects.
  ActionSearchResultV1 expected = censusAndCapActionsV1(
      trusted_generated_snapshot.actions, max_evaluated_actions);
  const std::string independently_derived_snapshot_identity =
      exactGeneratedSnapshotIdentityV1(trusted_generated_snapshot.actions);
  if (trusted_generated_snapshot.snapshot_identity !=
          independently_derived_snapshot_identity ||
      census.generated_snapshot_identity !=
          independently_derived_snapshot_identity) {
    return fail("raw generated snapshot identity mismatch");
  }
  if (lifecycle ==
      ActionSearchLifecycleV1::AbortedIncompleteBeforeEvaluation) {
    if (expected.census.exhaustive) {
      return fail("complete raw search cannot have aborted-incomplete lifecycle");
    }
    abortIncompleteActionSearchUnchecked(&expected);
  } else if (lifecycle != ActionSearchLifecycleV1::ReadyForEvaluation) {
    return fail("unknown action-search lifecycle");
  }
  if (!sameCensus(census, expected.census)) {
    return fail("census differs from trusted raw generated actions");
  }
  if (actual_evaluated_actions.size() != expected.actions.size()) {
    return fail("evaluated actions differ from trusted raw derivation");
  }
  for (std::size_t i = 0; i < actual_evaluated_actions.size(); ++i) {
    const std::string actual_operation =
        exactActionOperationIdentityV1(actual_evaluated_actions[i]);
    const std::string expected_operation =
        exactActionOperationIdentityV1(expected.actions[i]);
    if (actual_operation != expected_operation ||
        exactActionSemanticIdentityFromOperationV1(
            actual_evaluated_actions[i], actual_operation) !=
            exactActionSemanticIdentityFromOperationV1(
                expected.actions[i], expected_operation)) {
      return fail("evaluated action bytes differ from trusted raw derivation");
    }
  }
  return {true, derived_exhaustive, "VALID"};
}

void abortIncompleteActionSearchBeforeEvaluationV1(
    ActionSearchResultV1* result) {
  if (!result) return;
  const auto validation = validateActionSearchCensusV1(
      result->census, result->generated_snapshot,
      result->max_evaluated_actions, result->lifecycle, result->actions);
  if (!validation.valid || validation.exhaustive) return;
  abortIncompleteActionSearchUnchecked(result);
}

GeneratedFaultModelSet HypothesisGenerator::generate(
    const LinearizedIntegrityWindow& window, const EpochTransaction& tx,
    const ImuFaultSubspaces& current_imu,
    const LinearizedFactorBlock&) const {
  std::string invalid_reason;
  if (!window.model_valid) invalid_reason = "integrity window is invalid";
  else if (config_.include_imu_faults &&
           (!current_imu.analytic_input_valid ||
            !current_imu.analytic_computation_valid)) {
    invalid_reason = "active IMU provider has no valid analytic subspace";
  } else if (config_.max_model_cardinality != 2) {
    invalid_reason = "max_model_cardinality must equal 2";
  } else if (config_.max_exclusion_cardinality == 0 ||
             config_.max_exclusion_cardinality >
                 config_.max_model_cardinality) {
    invalid_reason = "max_exclusion_cardinality is outside [1,2]";
  } else if (config_.max_candidate_count == 0) {
    invalid_reason = "max_candidate_count must be positive";
  } else if (!config_.single_faults_enabled &&
             !config_.double_faults_enabled) {
    invalid_reason = "no fault order is enabled";
  } else if (!config_.include_uwb_faults && !config_.include_imu_faults) {
    invalid_reason = "no fault provider is active";
  } else if (config_.double_faults_enabled &&
             !config_.include_uwb_accel_combinations &&
             !config_.include_uwb_gyro_combinations) {
    invalid_reason = "second order has no supported UWB-IMU pair family";
  }
  if (!invalid_reason.empty()) {
    throw std::invalid_argument(
        "fault model generation precondition failed: " + invalid_reason);
  }
  GeneratedFaultModelSet out;
  const auto all = occurrences(tx);
  const LinearizedFactorBlock* history_block = historySummaryBlock(window);
  std::map<std::size_t, double> epoch_begin_s;
  for (const auto& occurrence : all) {
    epoch_begin_s[occurrence.epoch] = occurrence.begin.seconds();
  }
  std::map<std::uint64_t, std::vector<const Occurrence*>> anchor_occurrences;
  for (const auto& occurrence : all) {
    if (!occurrence.uwb || !occurrence.batch) continue;
    const bool explicit_rows = explicitlyMonitored(window, occurrence.uwb->id);
    for (const auto id : occurrence.uwb->source_measurements) {
      const auto* measurement = measurementById(*occurrence.batch, id);
      if (!measurement) continue;
      const std::uint64_t anchor = measurement->anchor_id.value();
      const bool history_rows =
          hasHistoryColumn(window, HistoryFaultBasisKind::UwbAnchorConstant,
                           anchor, occurrence.epoch);
      if (explicit_rows || history_rows) {
        anchor_occurrences[anchor].push_back(&occurrence);
      }
    }
  }

  std::uint64_t next_mode = 1;
  std::set<std::string> expected_identities;
  std::set<std::string> represented_identities;
  std::set<std::string> evaluated_identities;
  std::map<std::string, SensorType> expected_sensors;
  std::set<std::string> expected_pairs;
  std::set<std::string> represented_pairs;
  std::set<std::string> evaluated_pairs;
  auto expect_mode = [&](FaultKind kind, const std::string& source,
                         std::size_t onset, int dimension,
                         SensorType sensor) {
    FaultModeBasis expected;
    expected.kind = kind;
    expected.physical_source_id = source;
    expected.onset_epoch = onset;
    expected.parameter_dimension = dimension;
    const std::string identity = physicalModeIdentity(expected);
    expected_identities.insert(identity);
    expected_sensors.emplace(identity, sensor);
  };
  // Expected identities are deliberately enumerated independently of
  // history.column_ids and of the mode registry.  The only inputs are the
  // frozen raw transaction, the configured providers/families, and the
  // declared physical horizon carried by the window.  A missing production
  // history column therefore creates a census failure instead of shrinking
  // the denominator.
  std::map<std::uint64_t, std::set<std::size_t>> expected_anchor_epochs;
  const std::size_t physical_first = window.history_summary.present
      ? window.history_summary.horizon_first_epoch
      : window.detector_first_epoch;
  for (const auto& occurrence : all) {
    if (!occurrence.uwb || !occurrence.batch ||
        occurrence.epoch < physical_first ||
        occurrence.epoch > tx.proposed_epoch) {
      continue;
    }
    for (const auto id : occurrence.uwb->source_measurements) {
      const auto* measurement = measurementById(*occurrence.batch, id);
      if (measurement) {
        expected_anchor_epochs[measurement->anchor_id.value()].insert(
            occurrence.epoch);
      }
    }
  }
  if (config_.include_uwb_faults) for (const auto& anchor_item : expected_anchor_epochs) {
    std::set<std::size_t> onsets;
    onsets = anchor_item.second;
    const std::string source = "uwb:" + std::to_string(anchor_item.first);
    for (const std::size_t onset : onsets) {
      if (config_.include_epoch_independent_uwb) {
        expect_mode(FaultKind::AnchorBiasEpochIndependent, source, onset, 1,
                    SensorType::Uwb);
      }
      if (config_.include_persistent_uwb) {
        expect_mode(FaultKind::AnchorBiasPersistentConstant, source, onset, 1,
                    SensorType::Uwb);
      }
      if (config_.include_ramp_uwb) {
        expect_mode(FaultKind::AnchorBiasRamp, source, onset, 2,
                    SensorType::Uwb);
      }
    }
  }
  if (config_.include_imu_faults) {
    for (const auto& occurrence : all) {
      // The physical scope is the union of complete raw historical epochs
      // and explicit in-window intervals.  The sole interval that straddles
      // history.window_first_epoch is neither: it cannot be represented by a
      // history column and is not an all-keys-in-window explicit factor.
      const bool historical_interval =
          occurrence.epoch >= physical_first &&
          occurrence.epoch < window.history_summary.window_first_epoch;
      const bool explicit_interval =
          occurrence.previous_epoch >= window.detector_first_epoch;
      if (!occurrence.imu || (!historical_interval && !explicit_interval) ||
          occurrence.epoch > tx.proposed_epoch) continue;
      for (int axis = 0; axis < 6; ++axis) {
        const FaultKind kind = axis < 3
            ? FaultKind::AccelAxisIntervalConstant
            : FaultKind::GyroAxisIntervalConstant;
        const std::string source =
            std::string(axis < 3 ? "imu_accel:" : "imu_gyro:") +
            std::to_string(axis % 3) + ":interval:" +
            std::to_string(occurrence.epoch);
        expect_mode(kind, source, occurrence.previous_epoch, 1,
                    axis < 3 ? SensorType::ImuAccelerometer
                             : SensorType::ImuGyroscope);
      }
    }
  }
  if (config_.double_faults_enabled) {
    for (const auto& left : expected_sensors) {
      if (left.second != SensorType::Uwb) continue;
      for (const auto& right : expected_sensors) {
        const bool enabled =
            (right.second == SensorType::ImuAccelerometer &&
             config_.include_uwb_accel_combinations) ||
            (right.second == SensorType::ImuGyroscope &&
             config_.include_uwb_gyro_combinations);
        if (enabled) {
          expected_pairs.insert(physicalPairIdentity(left.first, right.first));
        }
      }
    }
  }
  auto append_mode = [&](FaultModeBasis mode) {
    std::sort(mode.affected_groups.begin(), mode.affected_groups.end());
    mode.affected_groups.erase(
        std::unique(mode.affected_groups.begin(), mode.affected_groups.end()),
        mode.affected_groups.end());
    represented_identities.insert(physicalModeIdentity(mode));
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

  if (config_.include_uwb_faults) for (const auto& anchor_item : anchor_occurrences) {
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
      if (history_block != nullptr) {
        const bool has_history_support = std::any_of(
            window.history_summary.column_ids.begin(),
            window.history_summary.column_ids.end(),
            [&](const HistoryFaultColumnId& id) {
              return id.source == anchor_item.first &&
                     id.epoch >= effective_onset &&
                     id.kind == HistoryFaultBasisKind::UwbAnchorConstant;
            });
        const Eigen::MatrixXd history_map = historicalMap(
            window, ramp ? 2 : 1,
            [&](const HistoryFaultColumnId& id) {
              Eigen::VectorXd weights =
                  Eigen::VectorXd::Zero(ramp ? 2 : 1);
              if (id.source != anchor_item.first ||
                  id.epoch < effective_onset) {
                return weights;
              }
              if (id.kind == HistoryFaultBasisKind::UwbAnchorConstant) {
                weights(0) = 1.0;
                if (ramp) {
                  const auto begin = epoch_begin_s.find(id.epoch);
                  if (begin == epoch_begin_s.end()) return Eigen::VectorXd();
                  weights(1) = begin->second - mode->onset_time.seconds();
                }
              } else if (ramp &&
                         id.kind ==
                             HistoryFaultBasisKind::UwbAnchorTimeLinear) {
                weights(1) = 1.0;
              }
              return weights;
            });
        if (has_history_support) {
          mode->raw_group_maps[history_block->group_id] = history_map;
          for (const auto& occurrence : all) {
            if (!occurrence.uwb || occurrence.epoch < effective_onset ||
                explicitlyMonitored(window, occurrence.uwb->id)) {
              continue;
            }
            if (hasHistoryColumn(
                    window, HistoryFaultBasisKind::UwbAnchorConstant,
                    anchor_item.first, occurrence.epoch)) {
              mode->affected_groups.push_back(occurrence.uwb->id);
            }
          }
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
        // Epoch-independent means exactly the onset group.  Replace the
        // persistent-style historical map produced by fill() with the one
        // constant history column owned by this epoch.
        if (history_block != nullptr) {
          mode.raw_group_maps.erase(history_block->group_id);
          const Eigen::MatrixXd history_map = historicalMap(
              window, 1, [&](const HistoryFaultColumnId& id) {
                Eigen::VectorXd weight = Eigen::VectorXd::Zero(1);
                if (id.kind == HistoryFaultBasisKind::UwbAnchorConstant &&
                    id.source == anchor_item.first &&
                    id.epoch == onset->epoch) {
                  weight(0) = 1.0;
                }
                return weight;
              });
          if (hasHistoryColumn(
                  window, HistoryFaultBasisKind::UwbAnchorConstant,
                  anchor_item.first, onset->epoch)) {
            mode.raw_group_maps[history_block->group_id] = history_map;
          }
        }
        for (auto it = mode.raw_group_maps.begin(); it != mode.raw_group_maps.end();) {
          if (history_block != nullptr && it->first == history_block->group_id) {
            ++it;
            continue;
          }
          const auto found = std::find_if(all.begin(), all.end(), [&](const Occurrence& o) {
            return o.uwb && o.uwb->id == it->first;
          });
          if (found != all.end() && found->epoch != onset->epoch) it = mode.raw_group_maps.erase(it);
          else ++it;
        }
        mode.affected_groups.clear();
        for (const auto& occurrence : all) {
          if (occurrence.uwb && occurrence.epoch == onset->epoch) {
            mode.affected_groups.push_back(occurrence.uwb->id);
          }
        }
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

  if (config_.include_imu_faults) for (const auto& occurrence : all) {
    if (!occurrence.imu) continue;
    const bool explicit_rows = explicitlyMonitored(window, occurrence.imu->id);
    bool history_rows = false;
    for (int axis = 0; axis < 6; ++axis) {
      history_rows = history_rows ||
          hasHistoryColumn(window, HistoryFaultBasisKind::ImuAxisConstant,
                           static_cast<std::uint64_t>(axis), occurrence.epoch);
    }
    if (!explicit_rows && !history_rows) continue;
    const auto* block = blockFor(window, occurrence.imu->id);
    if (explicit_rows && !block) continue;
    ImuFaultSubspaces subspaces = current_imu;
    if (explicit_rows && occurrence.history) {
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
    if (explicit_rows && (!subspaces.analytic_input_valid ||
                          !subspaces.analytic_computation_valid)) continue;
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
      if (explicit_rows) {
        const Eigen::VectorXd whitened = axis < 3
            ? subspaces.accel_axis[axis] : subspaces.gyro_axis[axis - 3];
        mode.raw_group_maps[occurrence.imu->id] =
            block->whitener.triangularView<Eigen::Lower>().solve(whitened);
      }
      if (history_block != nullptr &&
          hasHistoryColumn(window, HistoryFaultBasisKind::ImuAxisConstant,
                           static_cast<std::uint64_t>(axis),
                           occurrence.epoch)) {
        const Eigen::MatrixXd history_map = historicalMap(
            window, 1, [&](const HistoryFaultColumnId& id) {
              Eigen::VectorXd weight = Eigen::VectorXd::Zero(1);
              if (id.kind == HistoryFaultBasisKind::ImuAxisConstant &&
                  id.source == static_cast<std::uint64_t>(axis) &&
                  id.epoch == occurrence.epoch) {
                weight(0) = 1.0;
              }
              return weight;
            });
        mode.raw_group_maps[history_block->group_id] = history_map;
      }
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
        if (!enabled) continue;
        // B2 runtime defence: a pair is only admitted when the two parameter
        // blocks are structurally independent.  Unsupported families and
        // shared parameters are rejected here (counted), in addition to the
        // startup-time manifest cross-check.
        ++out.pair_candidates_considered;
        const auto support = pairFamilySupport(uwb, imu);
        if (support == PairFamilySupport::Unsupported) {
          ++out.pair_candidates_rejected_unsupported;
          continue;
        }
        if (support == PairFamilySupport::SharedParameters) {
          ++out.pair_candidates_rejected_shared;
          continue;
        }
        std::string reason;
        const std::vector<FaultModeBasis> pair{uwb, imu};
        const std::vector<FaultModeId> pair_ids{uwb.id, imu.id};
        if (!hypothesisParametersIndependent(pair, pair_ids, &reason)) {
          ++out.pair_candidates_rejected_shared;
          continue;
        }
        represented_pairs.insert(physicalPairIdentity(
            physicalModeIdentity(uwb), physicalModeIdentity(imu)));
        hypothesis({uwb.id, imu.id});
      }
    }
  }
  for (const auto& value : out.hypotheses) {
    for (const auto id : value.modes) {
      evaluated_identities.insert(
          physicalModeIdentity(out.modes.at(id.value() - 1)));
    }
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
      evaluated_pairs.insert(physicalPairIdentity(
          physicalModeIdentity(out.modes.at(value.modes[0].value() - 1)),
          physicalModeIdentity(out.modes.at(value.modes[1].value() - 1))));
    }
  }
  const double allocation = conservativeEqualRiskAllocation(
      config_.total_hmi_allocation, out.hypotheses.size());
  for (auto& value : out.hypotheses) value.hmi_allocation = allocation;

  out.expected_mode_identities.assign(expected_identities.begin(),
                                      expected_identities.end());
  out.represented_mode_identities.assign(represented_identities.begin(),
                                         represented_identities.end());
  out.evaluated_mode_identities.assign(evaluated_identities.begin(),
                                       evaluated_identities.end());
  out.expected_pair_identities.assign(expected_pairs.begin(),
                                      expected_pairs.end());
  out.represented_pair_identities.assign(represented_pairs.begin(),
                                         represented_pairs.end());
  out.evaluated_pair_identities.assign(evaluated_pairs.begin(),
                                       evaluated_pairs.end());
  if (out.expected_mode_identities != out.represented_mode_identities) {
    std::vector<std::string> missing;
    std::set_difference(expected_identities.begin(), expected_identities.end(),
                        represented_identities.begin(), represented_identities.end(),
                        std::back_inserter(missing));
    std::vector<std::string> unexpected;
    std::set_difference(represented_identities.begin(), represented_identities.end(),
                        expected_identities.begin(), expected_identities.end(),
                        std::back_inserter(unexpected));
    throw std::runtime_error(
        "physical fault census mismatch: expected modes are not fully "
        "represented; expected=" + std::to_string(expected_identities.size()) +
        " represented=" + std::to_string(represented_identities.size()) +
        " missing=" + std::to_string(missing.size()) +
        (missing.empty() ? "" : " first_missing=" + missing.front()) +
        " unexpected=" + std::to_string(unexpected.size()) +
        (unexpected.empty() ? "" : " first_unexpected=" + unexpected.front()));
  }
  if (config_.single_faults_enabled &&
      out.represented_mode_identities != out.evaluated_mode_identities) {
    throw std::runtime_error(
        "physical fault census mismatch: represented modes are not fully "
        "evaluated");
  }
  if (config_.double_faults_enabled &&
      out.expected_pair_identities != out.represented_pair_identities) {
    throw std::runtime_error(
        "physical fault census mismatch: expected pairs are not fully "
        "represented");
  }
  if (config_.double_faults_enabled &&
      out.represented_pair_identities != out.evaluated_pair_identities) {
    throw std::runtime_error(
        "physical fault census mismatch: represented pairs are not fully "
        "evaluated");
  }

  // B4: a healthy, alarm-free frame needs the mode descriptions, their
  // sensitivity/slopes and the KEEP_ALL reference solution - not the exclusion
  // action entities (removal sets, replacement groups and bridge blocks).
  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  out.actions.push_back(keep);
  if (config_.lazy_action_entities) {
    out.action_entities_deferred = out.modes.size();
  } else {
    HypothesisGenerator::ensureActionEntities(tx, window, &out);
  }
  return out;
}

void HypothesisGenerator::ensureActionEntities(
    const EpochTransaction& tx, const LinearizedIntegrityWindow& window,
    GeneratedFaultModelSet* models) {
  if (!models || models->action_entities_built) return;
  const auto all = occurrences(tx);
  std::uint64_t next_action = 2;
  models->single_mode_actions.clear();
  models->single_mode_actions.reserve(models->modes.size());
  for (const auto& mode : models->modes) {
    auto action = actionForMode(tx, window, mode, all);
    action.id = ExclusionActionId(next_action++);
    ++models->action_entities_constructed;
    models->bridge_blocks_built += action.added_blocks.size();
    models->single_mode_actions.push_back(std::move(action));
  }
  // KEEP_ALL stays first; the deduplicated single-mode actions follow in the
  // same order and with the same identities as the eager path.
  models->actions.clear();
  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  models->actions.push_back(keep);
  std::map<std::string, std::vector<std::size_t>> seen;
  for (const auto& action : models->single_mode_actions) {
    if (action.recoverability != HistoryRecoverability::Recoverable) continue;
    const auto key = actionKey(action);
    const auto duplicate = seen.find(key);
    auto equivalent = models->actions.end();
    if (duplicate != seen.end()) {
      for (const auto index : duplicate->second) {
        if (equivalentActionOperation(models->actions[index], action)) {
          equivalent = models->actions.begin() + index;
          break;
        }
      }
    }
    if (equivalent != models->actions.end()) {
      mergeCoverage(&*equivalent, action);
      continue;
    }
    seen[key].push_back(models->actions.size());
    models->actions.push_back(action);
  }
  models->action_entities_built = true;
  models->action_entities_deferred = 0;
}

std::vector<ExclusionAction> HypothesisGenerator::actionsForPlausibleSet(
    const LinearizedIntegrityWindow& window, const EpochTransaction& transaction,
    GeneratedFaultModelSet* models_ptr,
    const std::vector<FaultModeEvidence>& evidence,
    const std::vector<std::string>& mandatory_health_sources) const {
  ActionSearchResultV1 result = actionsForPlausibleSetV1(
      window, transaction, models_ptr, evidence, mandatory_health_sources);
  if (!result.census.exhaustive) {
    throw std::runtime_error(
        "SEARCH_INCOMPLETE: legacy action-vector API cannot carry census; "
        "use actionsForPlausibleSetV1");
  }
  return result.actions;
}

ActionSearchResultV1 HypothesisGenerator::actionsForPlausibleSetV1(
    const LinearizedIntegrityWindow& window, const EpochTransaction& transaction,
    GeneratedFaultModelSet* models_ptr,
    const std::vector<FaultModeEvidence>& evidence,
    const std::vector<std::string>& mandatory_health_sources,
    GeneratedActionSnapshotV1* trusted_generated_snapshot) const {
  if (!models_ptr) return {};
  ensureActionEntities(transaction, window, models_ptr);
  const GeneratedFaultModelSet& models = *models_ptr;
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
  std::uint64_t id = 2;
  for (const auto& parts : requested) {
    auto action = unite(parts);
    if (action.exclusion_cardinality >
        static_cast<int>(config_.max_exclusion_cardinality)) {
      action.recoverability = HistoryRecoverability::MissingProvenance;
      action.action_model_id = "CARDINALITY_CONTRACT_REJECTED";
    }
    // Preserve every raw request.  The single census/dedup boundary below is
    // the only place allowed to merge exact operations, and records a proof
    // for every merged occurrence.
    action.id = ExclusionActionId(id++);
    out.push_back(std::move(action));
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
    union_action.id = ExclusionActionId(id++);
    union_action.action_model_id = "FULL_PLAUSIBLE_UNION_RECOVERY";
    out.push_back(std::move(union_action));
  }
  ActionSearchResultV1 result =
      censusAndCapActionsV1(out, config_.max_candidate_count);
  result.generated_snapshot.generator_identity =
      "HypothesisGenerator::actionsForPlausibleSetV1/production-v1";
  if (trusted_generated_snapshot) {
    *trusted_generated_snapshot = result.generated_snapshot;
  }
  return result;
}

CompleteActionStreamV2 HypothesisGenerator::actionsForPlausibleSetV2(
    const LinearizedIntegrityWindow& window,
    const EpochTransaction& transaction,
    GeneratedFaultModelSet* models,
    const std::vector<FaultModeEvidence>& evidence,
    const std::vector<std::string>& mandatory_health_sources,
    GeneratedActionSnapshotV1* trusted_generated_snapshot) const {
  if (!models) return {};
  // Reuse the frozen production generator, but remove its legacy total cap.
  // Generation and exact-semantic canonicalization are unchanged; only the
  // consumer interpretation of max_candidate_count changes to batch capacity.
  HypothesisGeneratorConfig uncapped = config_;
  uncapped.max_candidate_count = std::numeric_limits<std::uint32_t>::max();
  ActionSearchResultV1 generated = HypothesisGenerator(uncapped)
      .actionsForPlausibleSetV1(
          window, transaction, models, evidence, mandatory_health_sources);
  CompleteActionStreamV2 stream = makeCompleteActionStreamV2(
      generated.generated_snapshot.actions, config_.max_candidate_count,
      trusted_generated_snapshot);
  stream.complete.generated_snapshot.generator_identity =
      "HypothesisGenerator::actionsForPlausibleSetV2/production-v2";
  if (trusted_generated_snapshot) {
    trusted_generated_snapshot->generator_identity =
        stream.complete.generated_snapshot.generator_identity;
  }
  return stream;
}

AttemptActionArenaV3 HypothesisGenerator::actionsForPlausibleSetV3(
    const LinearizedIntegrityWindow& window,
    const EpochTransaction& transaction,
    const GeneratedFaultModelSet& models,
    const std::vector<FaultModeEvidence>& evidence,
    const std::vector<std::string>& mandatory_health_sources,
    bool force_digest_collision_for_test) const {
  auto state = std::make_shared<AttemptActionLeaseV3::State>();
  state->sidecar = std::make_shared<AttemptSidecarFileV3>();
  auto& search = state->search;
  const std::size_t requested_batch_capacity = config_.max_candidate_count;
  search.batch_capacity = requested_batch_capacity;
  if (requested_batch_capacity == 0) return AttemptActionArenaV3(state);

  // The state is attempt-scoped: these references remain valid until the
  // monitor closes the lease before leaving processUwbBatchImpl().
  const auto all = std::make_shared<const std::vector<Occurrence>>(
      occurrences(transaction));
  // A mode operation is immutable for the frozen attempt.  Construct and
  // linearize it exactly once.  Order-2 actions compose these frozen parts
  // instead of rebuilding both bridges at every producer/consumer boundary.
  auto frozen_mode_actions = std::make_shared<std::vector<ExclusionAction>>();
  frozen_mode_actions->reserve(models.modes.size());
  for (const auto& mode : models.modes) {
    frozen_mode_actions->push_back(
        actionForMode(transaction, window, mode, *all));
  }
  const auto compose_action =
      [&models, frozen_mode_actions](
          const CompactActionDescriptorV3& descriptor,
          bool copy_added_blocks) {
        ExclusionAction action;
        if (descriptor.mode_indices.empty()) {
          action.action_model_id = "KEEP_ALL";
        } else {
          std::vector<const ExclusionAction*> parts;
          parts.reserve(descriptor.mode_indices.size());
          for (const std::size_t mode_index : descriptor.mode_indices) {
            if (mode_index >= models.modes.size() ||
                mode_index >= frozen_mode_actions->size()) {
              throw std::runtime_error("compact action mode index invalid");
            }
            parts.push_back(&frozen_mode_actions->at(mode_index));
          }
          action = unite(parts, copy_added_blocks);
        }
        action.id = ExclusionActionId(descriptor.action_id);
        if (descriptor.force_missing_provenance) {
          action.recoverability = HistoryRecoverability::MissingProvenance;
        }
        if (!descriptor.action_model_override.empty()) {
          action.action_model_id = descriptor.action_model_override;
        }
        return action;
      };
  state->materialize_one =
      [compose_action](const CompactActionDescriptorV3& descriptor) {
        return compose_action(descriptor, true);
      };
  state->materialize_metadata =
      [compose_action](const CompactActionDescriptorV3& descriptor) {
        return compose_action(descriptor, false);
      };

  std::map<std::uint64_t, std::size_t> mode_index;
  for (std::size_t index = 0; index < models.modes.size(); ++index) {
    mode_index.emplace(models.modes[index].id.value(), index);
  }
  const auto is_current_group = [&](FactorGroupId group) {
    if (group == transaction.imu_group.id) return true;
    return std::any_of(transaction.uwb_groups.begin(),
        transaction.uwb_groups.end(), [&](const PendingFactorGroup& value) {
          return value.nominal && value.id == group;
        });
  };
  std::vector<std::size_t> mandatory;
  for (std::size_t index = 0; index < models.modes.size(); ++index) {
    const ExclusionAction& action = frozen_mode_actions->at(index);
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
    if (required) mandatory.push_back(index);
  }

  std::vector<std::vector<std::size_t>> requested;
  std::vector<std::size_t> full;
  std::set<std::uint64_t> plausible_anchors;
  const auto plausible = completePlausibleHypotheses(
      models.hypotheses, evidence);
  for (const auto hypothesis_id : plausible) {
    const auto hypothesis = std::find_if(
        models.hypotheses.begin(), models.hypotheses.end(),
        [&](const FaultHypothesisV2& value) {
          return value.id == hypothesis_id;
        });
    if (hypothesis == models.hypotheses.end()) continue;
    std::vector<std::size_t> parts;
    for (const auto mode_id : hypothesis->modes) {
      const auto found = mode_index.find(mode_id.value());
      if (found == mode_index.end()) continue;
      const auto& mode = models.modes[found->second];
      if (mode.sensor == SensorType::Uwb) {
        plausible_anchors.insert(mode.anchor_id.value());
      }
      parts.push_back(found->second);
      full.push_back(found->second);
    }
    parts.insert(parts.end(), mandatory.begin(), mandatory.end());
    if (!parts.empty()) requested.push_back(std::move(parts));
  }
  if (plausible.empty() && !mandatory.empty()) requested.push_back(mandatory);

  std::vector<CompactActionDescriptorV3> raw_actions;
  CompactActionDescriptorV3 keep;
  keep.action_id = 1;
  keep.action_model_override = "KEEP_ALL";
  raw_actions.push_back(std::move(keep));
  std::uint64_t next_id = 2;
  auto physicalCardinality = [&](const std::vector<std::size_t>& parts) {
    std::set<std::string> sources;
    for (const auto index : parts) {
      if (index < models.modes.size()) {
        sources.insert(models.modes[index].physical_source_id);
      }
    }
    return sources.size();
  };
  for (auto& parts : requested) {
    CompactActionDescriptorV3 descriptor;
    descriptor.action_id = next_id++;
    descriptor.mode_indices = std::move(parts);
    if (physicalCardinality(descriptor.mode_indices) >
        config_.max_exclusion_cardinality) {
      descriptor.cardinality_contract_rejected = true;
      descriptor.force_missing_provenance = true;
      descriptor.action_model_override = "CARDINALITY_CONTRACT_REJECTED";
    }
    raw_actions.push_back(std::move(descriptor));
  }
  std::sort(full.begin(), full.end(), [&](std::size_t left,
                                          std::size_t right) {
    return models.modes[left].id < models.modes[right].id;
  });
  full.erase(std::unique(full.begin(), full.end(),
                         [&](std::size_t left, std::size_t right) {
    return models.modes[left].id == models.modes[right].id;
  }), full.end());
  full.insert(full.end(), mandatory.begin(), mandatory.end());
  if (!full.empty() && plausible_anchors.size() <= 1) {
    CompactActionDescriptorV3 descriptor;
    descriptor.action_id = next_id++;
    descriptor.mode_indices = std::move(full);
    descriptor.action_model_override = "FULL_PLAUSIBLE_UNION_RECOVERY";
    descriptor.force_missing_provenance =
        physicalCardinality(descriptor.mode_indices) >
        config_.max_exclusion_cardinality;
    raw_actions.push_back(std::move(descriptor));
  }

  // Materialize one action at a time and stream its canonical bytes directly
  // into the anonymous attempt sidecar.  No complete operation/semantic
  // string is retained.  Digests only select possible equality buckets;
  // equality itself is a fixed-buffer byte comparison over sidecar ranges.
  std::map<std::pair<std::uint64_t, std::uint64_t>, std::vector<std::size_t>>
      digest_buckets;
  using BlockBucketKeyV3 = std::pair<std::string, std::uint64_t>;
  std::map<BlockBucketKeyV3,
           std::vector<CompactActionDescriptorV3::BlockReference>>
      block_buckets;
  std::uint64_t next_block_id = 1;
  bool finite = true;
  constexpr std::size_t kActionArenaHeavyBudgetBytes = 8u << 20;
  bool indivisible_payload_too_large = false;
  for (std::size_t index = 0; index < raw_actions.size(); ++index) {
    ExclusionAction action = state->materialize_metadata(raw_actions[index]);
    auto& descriptor = raw_actions[index];
    // Resolve the ordered block content by reference to the frozen single-mode
    // operations.  This is byte-for-byte the same group-id union as unite(),
    // but it never copies a dense matrix merely to hash or index it.
    std::vector<const LinearizedFactorBlock*> ordered_blocks;
    std::set<std::uint64_t> added_group_ids;
    for (const std::size_t mode_index : descriptor.mode_indices) {
      if (mode_index >= frozen_mode_actions->size()) {
        finite = false;
        break;
      }
      const auto& part = frozen_mode_actions->at(mode_index);
      for (std::size_t block_index = 0;
           block_index < part.groups_to_add.size(); ++block_index) {
        if (block_index >= part.added_blocks.size()) {
          finite = false;
          break;
        }
        const auto group_id = part.groups_to_add[block_index].value();
        if (added_group_ids.insert(group_id).second) {
          ordered_blocks.push_back(&part.added_blocks[block_index]);
        }
      }
      if (!finite) break;
    }
    std::sort(ordered_blocks.begin(), ordered_blocks.end(),
        [](const LinearizedFactorBlock* left,
           const LinearizedFactorBlock* right) {
          std::array<char, 32> left_key{};
          std::array<char, 32> right_key{};
          std::snprintf(left_key.data(), left_key.size(), "%llx;",
                        static_cast<unsigned long long>(
                            left->group_id.value()));
          std::snprintf(right_key.data(), right_key.size(), "%llx;",
                        static_cast<unsigned long long>(
                            right->group_id.value()));
          return std::strcmp(left_key.data(), right_key.data()) < 0;
        });
    finite = finite && std::all_of(
        ordered_blocks.begin(), ordered_blocks.end(),
        [](const LinearizedFactorBlock* block) {
          return block != nullptr && finiteBlockOperation(*block);
        });
    descriptor.materialized_heavy_bytes = 0;
    for (const auto* block : ordered_blocks) {
      const std::size_t block_bytes = materializedBlockHeavyBytesV3(*block);
      if (block_bytes > std::numeric_limits<std::size_t>::max() -
                            descriptor.materialized_heavy_bytes) {
        throw std::overflow_error("compact action heavy-byte count overflow");
      }
      descriptor.materialized_heavy_bytes += block_bytes;
    }
    indivisible_payload_too_large = indivisible_payload_too_large ||
        descriptor.materialized_heavy_bytes > kActionArenaHeavyBudgetBytes;
    // Persist every exact immutable matrix payload once per attempt.  Actions
    // retain only ordered references.  SHA-256/length selects a bucket and a
    // bounded pread comparison proves equality, including collision cases.
    for (const auto* block : ordered_blocks) {
      const auto block_emit = [&](CanonicalByteSinkV3* sink) {
        return emitCanonicalBlockV3(*block, sink);
      };
      const CanonicalMetadataV3 metadata = canonicalMetadataV3(block_emit);
      if (metadata.strong_digest.empty()) {
        finite = false;
        break;
      }
      const BlockBucketKeyV3 key{
          metadata.strong_digest, metadata.length};
      std::optional<CompactActionDescriptorV3::BlockReference> existing;
      for (const auto& candidate : block_buckets[key]) {
        if (state->sidecar->sameCanonical(
                {candidate.offset, candidate.length, 0}, block_emit)) {
          existing = candidate;
          break;
        }
      }
      if (existing) {
        descriptor.ordered_blocks.push_back(*existing);
        continue;
      }
      SidecarRangeV3 range;
      if (!state->sidecar->appendCanonical(block_emit, &range)) {
        finite = false;
        break;
      }
      CompactActionDescriptorV3::BlockReference reference;
      reference.block_id = next_block_id++;
      reference.offset = range.offset;
      reference.length = range.length;
      reference.strong_digest = metadata.strong_digest;
      block_buckets[key].push_back(reference);
      descriptor.ordered_blocks.push_back(std::move(reference));
    }
    if (!finite) break;

    SidecarRangeV3 operation;
    SidecarRangeV3 semantic;
    if (!state->sidecar->appendCanonical(
            [&](CanonicalByteSinkV3* sink) {
              return emitCompactOperationRecordV3(action, descriptor, sink);
            }, &operation) ||
        !state->sidecar->appendCanonical(
            [&](CanonicalByteSinkV3* sink) {
              return emitCompactSemanticRecordV3(action, descriptor, sink);
            }, &semantic)) {
      finite = false;
      break;
    }
    descriptor.operation_offset = operation.offset;
    descriptor.operation_length = operation.length;
    descriptor.operation_digest = operation.digest;
    descriptor.semantic_offset = semantic.offset;
    descriptor.semantic_length = semantic.length;
    descriptor.semantic_digest = semantic.digest;
  }
  search.unique_block_count = next_block_id - 1;
  for (const auto& descriptor : raw_actions) {
    search.referenced_block_count += descriptor.ordered_blocks.size();
  }
  if (finite && !state->sidecar->seal(
          &search.sidecar_length, &search.sidecar_digest)) {
    finite = false;
  }
  std::map<std::uint64_t, std::vector<std::size_t>> action_id_representatives;
  if (finite) {
    for (const auto& descriptor : raw_actions) {
      const auto bucket_key = force_digest_collision_for_test
          ? std::make_pair<std::uint64_t, std::uint64_t>(1, 1)
          : std::make_pair(descriptor.operation_digest,
                           descriptor.semantic_digest);
      const auto& candidates = digest_buckets[bucket_key];
      std::optional<std::size_t> exact_representative;
      for (const std::size_t representative_index : candidates) {
        ++search.digest_collision_comparisons;
        const auto& representative = search.actions[representative_index];
        if (state->sidecar->sameBytes(operationRangeV3(descriptor),
                                      operationRangeV3(representative)) &&
            state->sidecar->sameBytes(semanticRangeV3(descriptor),
                                      semanticRangeV3(representative))) {
          exact_representative = representative_index;
          break;
        }
      }
      bool id_conflict = false;
      const auto existing_ids = action_id_representatives.find(
          descriptor.action_id);
      if (existing_ids != action_id_representatives.end()) {
        for (const auto representative_index : existing_ids->second) {
          const auto& representative = search.actions[representative_index];
          if (!state->sidecar->sameBytes(semanticRangeV3(descriptor),
                                         semanticRangeV3(representative))) {
            id_conflict = true;
          }
        }
      }
      if (id_conflict) {
        search.omitted_actions.push_back({
            descriptor, 0, "DUPLICATE_ACTION_ID_CONFLICT", false});
      } else if (exact_representative) {
        search.omitted_actions.push_back({
            descriptor,
            search.actions[*exact_representative].action_id,
            "EXACT_SEMANTIC_DUPLICATE", true});
      } else {
        const std::size_t retained_index = search.actions.size();
        search.actions.push_back(descriptor);
        digest_buckets[bucket_key].push_back(retained_index);
        action_id_representatives[descriptor.action_id].push_back(
            retained_index);
      }
    }
  }
  search.generated = raw_actions.size();
  search.evaluated = finite ? search.actions.size() : 0;
  search.omitted = search.generated - search.evaluated;
  if (!finite) {
    search.actions.clear();
    search.omitted_actions.clear();
    for (const auto& descriptor : raw_actions) {
      search.omitted_actions.push_back(
          {descriptor, 0, "SIDECAR_IO_OR_ENCODING_FAILURE", false});
    }
  }
  const bool all_omissions_safe = std::all_of(
      search.omitted_actions.begin(), search.omitted_actions.end(),
      [](const CompactActionOmissionV3& omission) {
        return omission.proven_safe;
      });
  search.exhaustive = finite && !indivisible_payload_too_large &&
      all_omissions_safe;
  search.terminal_reason = search.exhaustive ? "COMPLETE"
                                             : "SEARCH_INCOMPLETE";
  // A count-only bound is insufficient when an order-2 action contains large
  // dense bridge matrices.  Derive one fixed action count for this attempt
  // from the worst exact recipe so every leased batch stays below the explicit
  // heavy-payload budget (except an indivisible single action).
  std::size_t max_action_bytes = 0;
  for (const auto& descriptor : search.actions) {
    max_action_bytes = std::max(
        max_action_bytes, descriptor.materialized_heavy_bytes);
  }
  const std::size_t byte_bounded_capacity = max_action_bytes == 0
      ? requested_batch_capacity
      : kActionArenaHeavyBudgetBytes / max_action_bytes;
  search.batch_capacity = std::min(
      requested_batch_capacity, byte_bounded_capacity);
  if (search.batch_capacity == 0) {
    search.batch_capacity = 1;
    search.exhaustive = false;
    search.terminal_reason = "SEARCH_INCOMPLETE";
  }
  search.batch_count = search.actions.empty()
      ? 0 : 1 + (search.actions.size() - 1) / search.batch_capacity;
  search.peak_batch_actions =
      std::min(search.batch_capacity, search.actions.size());
  search.snapshot_digest = compactSnapshotDigestV3(
      search.actions, search.omitted_actions, search.generated,
      search.sidecar_digest, search.sidecar_length);
  for (std::size_t begin = 0; begin < search.actions.size();
       begin += search.batch_capacity) {
    std::size_t batch_bytes = 0;
    const std::size_t end = std::min(
        search.actions.size(), begin + search.batch_capacity);
    for (std::size_t index = begin; index < end; ++index) {
      batch_bytes += search.actions[index].materialized_heavy_bytes;
    }
    search.peak_batch_heavy_bytes = std::max(
        search.peak_batch_heavy_bytes, batch_bytes);
  }
  bool producer_exact = search.exhaustive;
  std::map<std::uint64_t,
           std::tuple<std::uint64_t, std::uint64_t, std::string>>
      block_identities;
  for (const auto& descriptor : raw_actions) {
    producer_exact = producer_exact && validateProducedDescriptorV3(
        state->sidecar, descriptor);
    for (const auto& block : descriptor.ordered_blocks) {
      const auto identity = std::make_tuple(
          block.offset, block.length, block.strong_digest);
      const auto inserted = block_identities.emplace(block.block_id, identity);
      producer_exact = producer_exact && block.block_id != 0 &&
          (inserted.second || inserted.first->second == identity);
    }
  }
  // The complete file/root is read exactly once by the producer.  Subsequent
  // audit and FDE consumers reuse this sealed certificate and validate only
  // compact ranges/census; the descriptor recipes can stream an exact block
  // again on demand without retaining all canonical bytes.
  state->producer_certificate_valid = producer_exact &&
      state->sidecar->verifySealed();
  search.producer_certificate_valid = state->producer_certificate_valid;
  return AttemptActionArenaV3(std::move(state));
}

}  // namespace uwb_imu_pl
