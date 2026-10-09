// C1-b Part A implementation: historical fault parameterization (see the
// header for the frozen contract and D-1/D-2 decisions).
//
// Materials are read exclusively from `EpochTransaction::recoverable_history`
// records; nothing mutates the transaction, the frozen graph or the window.

#include "uwb_imu_pl/integrity/history_fault_parameterization.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <set>
#include <sstream>

#include "uwb_imu_pl/integrity/imu_fault_subspace.hpp"

namespace uwb_imu_pl {
namespace {

// Same upper information coordinate as the actual CombinedImuFactor rows.
bool whitenerFromCovariance(const Eigen::MatrixXd& covariance,
                            Eigen::MatrixXd* whitener) {
  if (covariance.rows() != covariance.cols() || covariance.rows() == 0) {
    return false;
  }
  Eigen::LLT<Eigen::MatrixXd> llt(covariance);
  if (llt.info() != Eigen::Success) {
    return false;
  }
  *whitener = gtsam::noiseModel::Gaussian::Covariance(covariance)->R();
  return whitener->allFinite();
}

// UWB Gaussian factors use GTSAM's upper square-root information coordinate.
// Fault columns must use the identical coordinate; an independently valid
// lower root would be a row rotation and cannot be mixed with upper-root H/z.
bool upperInformationRootFromCovariance(const Eigen::MatrixXd& covariance,
                                        Eigen::MatrixXd* whitener) {
  if (covariance.rows() != covariance.cols() || covariance.rows() == 0) {
    return false;
  }
  Eigen::LLT<Eigen::MatrixXd> covariance_llt(covariance);
  if (covariance_llt.info() != Eigen::Success) return false;
  const Eigen::MatrixXd information = covariance_llt.solve(
      Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
  Eigen::LLT<Eigen::MatrixXd> information_llt(information);
  if (information_llt.info() != Eigen::Success) return false;
  *whitener = information_llt.matrixU();
  return whitener->allFinite();
}

const PendingFactorGroup* selectedGroupOfKind(
    const HistoricalEpochContext& epoch, FactorKind kind) {
  for (const auto& group : epoch.groups) {
    if (group.kind != kind) continue;
    if (std::find(epoch.selected_groups.begin(), epoch.selected_groups.end(),
                  group.id) != epoch.selected_groups.end()) {
      return &group;
    }
  }
  return nullptr;
}

const UwbMeasurement* measurementInBatch(const UwbBatch& batch,
                                         MeasurementId id) {
  for (const auto& measurement : batch.measurements) {
    if (measurement.id == id) return &measurement;
  }
  return nullptr;
}

// Raw constant/linear columns for one (anchor, epoch) over the group row
// order.  Returns false when the anchor has no rows in this epoch.
bool uwbAnchorPatterns(const HistoricalEpochContext& epoch,
                       const PendingFactorGroup& group, std::uint64_t anchor,
                       Eigen::VectorXd* constant_raw,
                       Eigen::VectorXd* linear_raw, std::size_t* row_count) {
  const std::size_t rows = group.source_measurements.size();
  Eigen::VectorXd constant = Eigen::VectorXd::Zero(rows);
  Eigen::VectorXd linear = Eigen::VectorXd::Zero(rows);
  const double begin_s = epoch.begin.seconds();
  bool any = false;
  for (std::size_t row = 0; row < rows; ++row) {
    const UwbMeasurement* measurement =
        measurementInBatch(epoch.uwb_batch, group.source_measurements[row]);
    if (!measurement || measurement->anchor_id.value() != anchor) continue;
    constant(row) = 1.0;
    linear(row) = measurement->timestamp.seconds() - begin_s;
    any = true;
  }
  if (!any) return false;
  *constant_raw = std::move(constant);
  *linear_raw = std::move(linear);
  *row_count = rows;
  return true;
}

}  // namespace

const char* toString(HistoryFaultBasisKind kind) {
  switch (kind) {
    case HistoryFaultBasisKind::UwbAnchorConstant:
      return "uwb_anchor_constant";
    case HistoryFaultBasisKind::UwbAnchorTimeLinear:
      return "uwb_anchor_time_linear";
    case HistoryFaultBasisKind::ImuAxisConstant:
      return "imu_axis_constant";
  }
  return "unknown";
}

bool HistoryFaultColumnId::operator==(const HistoryFaultColumnId& other) const {
  return kind == other.kind && source == other.source && epoch == other.epoch;
}

bool HistoryFaultColumnId::operator<(const HistoryFaultColumnId& other) const {
  if (static_cast<int>(kind) != static_cast<int>(other.kind)) {
    return static_cast<int>(kind) < static_cast<int>(other.kind);
  }
  if (source != other.source) return source < other.source;
  return epoch < other.epoch;
}

HistoryFaultParameterizationPlan planHistoryFaultParameterization(
    const EpochTransaction& tx, std::size_t window_epochs,
    const HistoryFaultParameterizationOptions& options) {
  HistoryFaultParameterizationPlan plan;
  plan.scope_digest = options.scope_digest;
  const std::size_t derived_first = tx.oldest_recoverable_epoch + 1;
  const std::size_t window_first =
      tx.proposed_epoch > window_epochs ? tx.proposed_epoch - window_epochs : 0;
  plan.horizon.first_epoch = options.configured_first_epoch
                                 ? *options.configured_first_epoch
                                 : derived_first;
  plan.horizon.source = options.configured_first_epoch
                            ? "configured:history.horizon_first_epoch"
                            : "derived:tx.oldest_recoverable_epoch+1";
  plan.horizon.window_first_epoch = window_first;
  if (plan.horizon.first_epoch > 0) {
    plan.omitted_epoch_begin = 0;
    plan.omitted_epoch_end = plan.horizon.first_epoch;
    plan.omitted_epoch_count = plan.horizon.first_epoch;
  }

  std::set<std::size_t> observed_epochs;
  for (const auto& epoch : tx.recoverable_history) {
    const std::size_t current = epoch.proposed_epoch;
    if (current < plan.horizon.first_epoch || current >= window_first) {
      continue;  // in-window epochs belong to the generator; older ones to
                 // the omitted range
    }
    observed_epochs.insert(current);
    if (options.include_uwb_faults) {
      std::vector<HistoryFaultColumn> uwb_columns =
          buildHistoricalUwbColumnsForEpoch(epoch, &plan.skipped_columns);
      for (auto& column : uwb_columns) {
        if (!options.include_time_linear_basis &&
            column.id.kind == HistoryFaultBasisKind::UwbAnchorTimeLinear) {
          continue;
        }
        plan.columns.push_back(std::move(column));
      }
    }
    if (options.include_imu_faults) {
      std::vector<HistoryFaultColumn> imu_columns =
          buildHistoricalImuColumnsForEpoch(epoch, &plan.skipped_columns);
      for (auto& column : imu_columns) plan.columns.push_back(std::move(column));
    }
  }

  // Material gaps: requested epochs without a recoverable record.
  if (window_first > plan.horizon.first_epoch) {
    const std::size_t expected = window_first - plan.horizon.first_epoch;
    plan.material_gap_epoch_count = expected > observed_epochs.size()
                                        ? expected - observed_epochs.size()
                                        : 0;
  }
  return plan;
}

std::vector<HistoryFaultColumn> buildHistoricalUwbColumnsForEpoch(
    const HistoricalEpochContext& epoch, std::size_t* skipped) {
  std::vector<HistoryFaultColumn> columns;
  const std::size_t current = epoch.proposed_epoch;
  const PendingFactorGroup* uwb =
      selectedGroupOfKind(epoch, FactorKind::UwbBatch);
  if (!uwb) return columns;
  Eigen::MatrixXd group_whitener;
  const std::size_t rows = uwb->source_measurements.size();
  bool have_whitener = false;
  if (uwb->raw_covariance.rows() == static_cast<int>(rows) &&
      uwb->raw_covariance.cols() == static_cast<int>(rows)) {
    have_whitener = upperInformationRootFromCovariance(
        uwb->raw_covariance, &group_whitener);
  } else {
    group_whitener = Eigen::MatrixXd::Identity(rows, rows);
    have_whitener = true;
  }
  std::set<std::uint64_t> anchors;
  for (const auto id : uwb->source_measurements) {
    const UwbMeasurement* measurement = measurementInBatch(epoch.uwb_batch, id);
    if (measurement) anchors.insert(measurement->anchor_id.value());
  }
  for (const std::uint64_t anchor : anchors) {
    Eigen::VectorXd constant_raw, linear_raw;
    std::size_t row_count = 0;
    if (!uwbAnchorPatterns(epoch, *uwb, anchor, &constant_raw, &linear_raw,
                           &row_count)) {
      continue;
    }
    if (!have_whitener) {
      if (skipped) *skipped += 1;
      continue;
    }
    HistoryFaultColumn constant;
    constant.id = {HistoryFaultBasisKind::UwbAnchorConstant, anchor, current};
    constant.group = uwb->id;
    constant.raw_map = constant_raw;
    constant.whitened_map = group_whitener * constant_raw;
    constant.time_begin_s = epoch.begin.seconds();
    constant.time_end_s = epoch.end.seconds();
    columns.push_back(std::move(constant));
    HistoryFaultColumn linear;
    linear.id = {HistoryFaultBasisKind::UwbAnchorTimeLinear, anchor, current};
    linear.group = uwb->id;
    linear.raw_map = linear_raw;
    linear.whitened_map = group_whitener * linear_raw;
    linear.time_begin_s = epoch.begin.seconds();
    linear.time_end_s = epoch.end.seconds();
    columns.push_back(std::move(linear));
  }
  return columns;
}

std::vector<HistoryFaultColumn> buildHistoricalImuColumnsForEpoch(
    const HistoricalEpochContext& epoch, std::size_t* skipped) {
  std::vector<HistoryFaultColumn> columns;
  const std::size_t current = epoch.proposed_epoch;
  const PendingFactorGroup* imu =
      selectedGroupOfKind(epoch, FactorKind::CombinedImu);
  if (!imu || !epoch.preintegration) return columns;
  // Same material contract as `HypothesisGenerator` for historical epochs: a
  // local transaction with the recorded states/preintegration plus a
  // synthesized material block whose whitener follows the estimator
  // convention (`preintegration->preintMeasCov()`).
  EpochTransaction local;
  local.previous_state = epoch.previous_state;
  local.nominal_predicted_state = epoch.current_state;
  local.preintegration = epoch.preintegration;
  local.raw_imu_slice = epoch.raw_imu_slice;
  LinearizedFactorBlock material;
  material.kind = FactorKind::CombinedImu;
  material.covariance = epoch.preintegration->preintMeasCov();
  if (!whitenerFromCovariance(material.covariance, &material.whitener)) {
    if (skipped) *skipped += 6;
    return columns;
  }
  material.residual_whitened = Eigen::VectorXd::Zero(15);
  const ImuFaultSubspaces subspaces =
      ImuFaultSubspaceBuilder().buildAnalytic(local, material);
  if (!subspaces.analytic_input_valid ||
      !subspaces.analytic_computation_valid) {
    if (skipped) *skipped += 6;
    return columns;
  }
  for (int axis = 0; axis < 6; ++axis) {
    HistoryFaultColumn column;
    column.id = {HistoryFaultBasisKind::ImuAxisConstant,
                 static_cast<std::uint64_t>(axis), current};
    column.group = imu->id;
    const Eigen::VectorXd& whitened =
        axis < 3 ? subspaces.accel_axis[axis] : subspaces.gyro_axis[axis - 3];
    column.whitened_map = whitened;
    column.raw_map =
        material.whitener.triangularView<Eigen::Upper>().solve(whitened);
    column.time_begin_s = epoch.begin.seconds();
    column.time_end_s = epoch.end.seconds();
    columns.push_back(std::move(column));
  }
  return columns;
}

HistoryFaultCombination persistentUwbCombination(
    std::uint64_t anchor, std::size_t onset_epoch,
    const std::vector<HistoryFaultColumnId>& basis) {
  HistoryFaultCombination combination;
  for (const auto& id : basis) {
    if (id.kind != HistoryFaultBasisKind::UwbAnchorConstant) continue;
    if (id.source != anchor || id.epoch < onset_epoch) continue;
    combination.columns.push_back(id);
    combination.coefficients.conservativeResize(
        combination.coefficients.size() + 1);
    combination.coefficients(combination.coefficients.size() - 1) = 1.0;
  }
  return combination;
}

HistoryFaultCombination rampUwbCombination(
    std::uint64_t anchor, std::size_t onset_epoch, double onset_time_s,
    const std::map<std::size_t, double>& epoch_begin_s,
    const std::vector<HistoryFaultColumnId>& basis) {
  HistoryFaultCombination combination;
  for (const auto& id : basis) {
    if (id.source != anchor || id.epoch < onset_epoch) continue;
    const auto begin = epoch_begin_s.find(id.epoch);
    if (begin == epoch_begin_s.end()) continue;
    double coefficient = 0.0;
    if (id.kind == HistoryFaultBasisKind::UwbAnchorTimeLinear) {
      coefficient = 1.0;
    } else if (id.kind == HistoryFaultBasisKind::UwbAnchorConstant) {
      coefficient = begin->second - onset_time_s;
    } else {
      continue;
    }
    combination.columns.push_back(id);
    combination.coefficients.conservativeResize(
        combination.coefficients.size() + 1);
    combination.coefficients(combination.coefficients.size() - 1) = coefficient;
  }
  return combination;
}

std::string HistoryFaultParameterizationPlan::validityAssumptions() const {
  std::ostringstream out;
  out << "history_fault_coverage=[" << horizon.first_epoch << ","
      << horizon.window_first_epoch << ") source=" << horizon.source
      << "; omitted_epochs=" << omitted_epoch_count
      << " material_gap_epochs=" << material_gap_epoch_count
      << "; no detection or protection claim is made outside the covered "
         "range";
  return out.str();
}

std::string HistoryFaultParameterizationPlan::omittedRiskSource() const {
  std::ostringstream out;
  out << "history_fault_omitted:[" << omitted_epoch_begin << ","
      << omitted_epoch_end << ") count=" << omitted_epoch_count;
  return out.str();
}

HistoryFaultInjectionResult buildHistoricalFaultInjection(
    const HistoricalEpochContext& epoch, const gtsam::Values& linearization,
    const std::vector<HistoryFaultColumn>& columns,
    std::uint64_t first_fault_index) {
  HistoryFaultInjectionResult out;
  if (columns.empty()) {
    out.reason = "no fault columns to inject";
    return out;
  }
  // Group the columns by source group; only groups that carry columns are
  // rebuilt (groups without columns stay untouched).
  std::vector<FactorGroupId> group_order;
  std::map<std::uint64_t, std::vector<const HistoryFaultColumn*>> by_group;
  for (const auto& column : columns) {
    auto& bucket = by_group[column.group.value()];
    if (bucket.empty()) group_order.push_back(column.group);
    bucket.push_back(&column);
  }
  std::uint64_t fault_index = first_fault_index;
  for (const auto group_id : group_order) {
    const PendingFactorGroup* group = nullptr;
    for (const auto& candidate : epoch.groups) {
      if (candidate.id == group_id) group = &candidate;
    }
    if (group == nullptr) {
      out.reason = "fault columns reference a missing group";
      return out;
    }
    // Linearize the group's factors at the linearization point and stack
    // them; the fault columns are appended in the same (whitened) units.
    // Rows are copied key-block by key-block: a navigation-state key carries a
    // 6- or 3-dimensional tangent block (x/b: 6, v: 3), so a per-key single
    // column would truncate the state map and break the separator algebra.
    std::vector<boost::shared_ptr<gtsam::JacobianFactor>> jacobians;
    gtsam::KeyVector key_order;
    std::map<gtsam::Key, int> column_of_key;
    int key_columns = 0;
    std::size_t rows = 0;
    for (const auto& factor : group->factors) {
      const auto jacobian = boost::dynamic_pointer_cast<gtsam::JacobianFactor>(
          factor->linearize(linearization));
      if (!jacobian) {
        out.reason = "group factor did not linearize to a JacobianFactor";
        return out;
      }
      if (!jacobian->getA().allFinite() || !jacobian->getb().allFinite()) {
        out.reason = "non-finite historical factor linearization at epoch " +
            std::to_string(epoch.proposed_epoch) + ", group " +
            std::to_string(group_id.value()) +
            ": A=" + (jacobian->getA().allFinite() ? "finite" : "non_finite") +
            ", b=" + (jacobian->getb().allFinite() ? "finite" : "non_finite");
        return out;
      }
      for (std::size_t local = 0; local < jacobian->keys().size(); ++local) {
        const gtsam::Key key = jacobian->keys()[local];
        if (column_of_key.find(key) == column_of_key.end()) {
          column_of_key.emplace(key, key_columns);
          key_order.push_back(key);
          key_columns +=
              static_cast<int>(jacobian->getA(jacobian->begin() + local).cols());
        }
      }
      rows += static_cast<std::size_t>(jacobian->getA().rows());
      jacobians.push_back(jacobian);
    }
    Eigen::MatrixXd stacked = Eigen::MatrixXd::Zero(rows, key_columns);
    Eigen::VectorXd stacked_b = Eigen::VectorXd::Zero(rows);
    std::size_t row_cursor = 0;
    for (const auto& jacobian : jacobians) {
      const Eigen::MatrixXd A = jacobian->getA();
      const Eigen::VectorXd b = jacobian->getb();
      for (std::size_t local = 0; local < jacobian->keys().size(); ++local) {
        const int global = column_of_key.at(jacobian->keys()[local]);
        const Eigen::MatrixXd block = jacobian->getA(jacobian->begin() + local);
        stacked.block(row_cursor, global, A.rows(), block.cols()) = block;
      }
      stacked_b.segment(row_cursor, b.size()) = b;
      row_cursor += static_cast<std::size_t>(A.rows());
    }
    for (const HistoryFaultColumn* column : by_group.at(group_id.value())) {
      if (column->whitened_map.rows() != static_cast<int>(rows)) {
        out.reason = "fault column row count does not match the group stack";
        return out;
      }
      if (!column->whitened_map.allFinite()) {
        out.reason = "non-finite historical fault column at epoch " +
            std::to_string(epoch.proposed_epoch) + ", group " +
            std::to_string(group_id.value()) + ", basis " +
            std::string(toString(column->id.kind)) + ", source " +
            std::to_string(column->id.source);
        return out;
      }
    }
    // One augmented linear factor over [state keys | fault key columns].
    std::vector<std::pair<gtsam::Key, Eigen::MatrixXd>> terms;
    for (std::size_t index = 0; index < key_order.size(); ++index) {
      const int begin = column_of_key.at(key_order[index]);
      const int width = [&] {
        for (const auto& jacobian : jacobians) {
          for (std::size_t local = 0; local < jacobian->keys().size();
               ++local) {
            if (jacobian->keys()[local] == key_order[index]) {
              return static_cast<int>(
                  jacobian->getA(jacobian->begin() + local).cols());
            }
          }
        }
        return 0;
      }();
      terms.emplace_back(key_order[index],
                         Eigen::MatrixXd(stacked.middleCols(begin, width)));
    }
    for (const HistoryFaultColumn* column : by_group.at(group_id.value())) {
      const gtsam::Key fault_key = gtsam::Symbol('f', fault_index++);
      terms.emplace_back(fault_key, Eigen::MatrixXd(column->whitened_map));
      out.fault_keys.push_back(fault_key);
    }
    const auto augmented_factor =
        boost::make_shared<gtsam::JacobianFactor>(terms, stacked_b);
    if (!augmented_factor->getA().allFinite() ||
        !augmented_factor->getb().allFinite()) {
      out.reason = "non-finite augmented historical factor at epoch " +
          std::to_string(epoch.proposed_epoch) + ", group " +
          std::to_string(group_id.value()) +
          ": A=" +
          (augmented_factor->getA().allFinite() ? "finite" : "non_finite") +
          ", b=" +
          (augmented_factor->getb().allFinite() ? "finite" : "non_finite");
      return out;
    }
    out.graph.push_back(augmented_factor);
    for (const gtsam::Key key : key_order) out.state_keys.push_back(key);
    out.rows += rows;
  }
  out.valid = true;
  return out;
}

HistoryFaultCapacityDecision evaluateHistoryFaultCapacity(
    std::size_t q_hist, const HistoryFaultCapacityLimits& limits) {
  HistoryFaultCapacityDecision decision;
  decision.action = limits.capacity_action;
  const bool known_action = limits.capacity_action == "REFUSE" ||
                            limits.capacity_action == "RESET" ||
                            limits.capacity_action == "STOP_PROTECTED";
  if (!known_action) {
    decision.fits = false;
    decision.unusable = true;
    decision.reason =
        "unknown history capacity action: " + limits.capacity_action;
    return decision;
  }
  if (q_hist <= limits.max_fault_columns) {
    decision.fits = true;
    return decision;
  }
  // Over-limit: the summary is unusable and must be counted; the oldest fault
  // is never silently dropped.  RESET / STOP_PROTECTED remain explicit
  // actions that require their own evidence before they may be executed.
  decision.fits = false;
  decision.unusable = true;
  decision.reason = "HISTORY_CAPACITY_EXCEEDED";
  return decision;
}

}  // namespace uwb_imu_pl
