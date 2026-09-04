#include "uwb_imu_pl/integrity/hypothesis_generator.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <map>
#include <stdexcept>

namespace uwb_imu_pl {
namespace {

Eigen::MatrixXd whitening(const Eigen::MatrixXd& covariance) {
  Eigen::LLT<Eigen::MatrixXd> llt(covariance);
  if (llt.info() != Eigen::Success) {
    throw std::runtime_error("retained UWB principal covariance is not SPD");
  }
  return llt.matrixL().solve(
      Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
}

const PendingFactorGroup* nominalUwb(const EpochTransaction& tx) {
  for (const auto& group : tx.uwb_groups) {
    if (group.nominal) return &group;
  }
  return nullptr;
}

const PendingFactorGroup* replacementFor(const EpochTransaction& tx,
                                         FaultUnitId unit) {
  for (const auto& group : tx.uwb_groups) {
    if (!group.nominal && group.excluded_fault_units.size() == 1 &&
        group.excluded_fault_units.front() == unit) return &group;
  }
  return nullptr;
}

}  // namespace

GeneratedFaultModelSet HypothesisGenerator::generate(
    const LinearizedIntegrityWindow& window,
    const EpochTransaction& tx,
    const ImuFaultSubspaces& imu,
    const LinearizedFactorBlock& bridge_block) const {
  if (!window.model_valid || !imu.analytic_verified ||
      config_.max_cardinality != 2 || config_.max_candidate_count == 0) {
    throw std::invalid_argument("fault model generation precondition failed");
  }
  GeneratedFaultModelSet out;
  LinearizedFactorBlock embedded_bridge = bridge_block;
  if (bridge_block.jacobian_whitened.cols() != window.H.cols()) {
    if (bridge_block.jacobian_whitened.cols() != 30 || window.H.cols() < 30) {
      throw std::invalid_argument("generic bridge block/window column mismatch");
    }
    embedded_bridge.jacobian_whitened = Eigen::MatrixXd::Zero(
        bridge_block.jacobian_whitened.rows(), window.H.cols());
    embedded_bridge.jacobian_raw = Eigen::MatrixXd::Zero(
        bridge_block.jacobian_raw.rows(), window.H.cols());
    embedded_bridge.jacobian_whitened.rightCols(30) =
        bridge_block.jacobian_whitened;
    embedded_bridge.jacobian_raw.rightCols(30) = bridge_block.jacobian_raw;
  }
  std::map<std::uint64_t, Eigen::VectorXd> maps;
  std::map<std::uint64_t, SensorType> sensors;
  std::map<std::uint64_t, FactorGroupId> groups;
  int row_offset = 0;
  const PendingFactorGroup* nominal_group = nominalUwb(tx);
  const LinearizedFactorBlock* nominal_uwb_block = nullptr;
  const LinearizedFactorBlock* imu_block = nullptr;
  int uwb_offset = -1;
  int imu_offset = -1;
  for (const auto& block : window.blocks) {
    if (nominal_group && block.group_id == nominal_group->id) {
      nominal_uwb_block = &block;
      uwb_offset = row_offset;
    }
    if (block.group_id == tx.imu_group.id) {
      imu_block = &block;
      imu_offset = row_offset;
    }
    row_offset += block.residual_whitened.size();
  }
  if (!nominal_group || !nominal_uwb_block || !imu_block || uwb_offset < 0 ||
      imu_offset < 0 || nominal_group->source_measurements.size() !=
          static_cast<std::size_t>(nominal_uwb_block->residual_whitened.size())) {
    throw std::runtime_error("nominal UWB/IMU blocks missing from integrity window");
  }

  std::map<std::uint64_t, std::vector<int>> anchor_rows;
  for (std::size_t i = 0; i < tx.uwb_batch.measurements.size(); ++i) {
    anchor_rows[tx.uwb_batch.measurements[i].anchor_id.value()].push_back(
        static_cast<int>(i));
  }
  for (const auto& pair : anchor_rows) {
    FaultUnit unit;
    unit.id = FaultUnitId(pair.first);
    unit.sensor = SensorType::Uwb;
    unit.kind = FaultKind::AnchorBiasPersistentConstant;
    unit.epoch_begin = tx.previous_epoch;
    unit.epoch_end = tx.proposed_epoch;
    unit.time_begin = tx.begin;
    unit.time_end = tx.end;
    unit.affected_groups = {nominal_group->id};
    unit.parameter_dimension = 1;
    unit.prior_probability_bound = config_.uwb_prior_bound;
    unit.persistent = true;
    Eigen::VectorXd raw = Eigen::VectorXd::Zero(nominal_uwb_block->residual_raw.size());
    for (const int index : pair.second) {
      raw(index) = 1.0;
      unit.affected_measurements.push_back(
          tx.uwb_batch.measurements[static_cast<std::size_t>(index)].id);
    }
    Eigen::VectorXd embedded = Eigen::VectorXd::Zero(window.H.rows());
    embedded.segment(uwb_offset, raw.size()) = nominal_uwb_block->whitener * raw;
    unit.subspace_model.map = embedded;
    unit.subspace_model.model_id = "uwb_physical_anchor_bias_v2";
    maps[unit.id.value()] = embedded;
    sensors[unit.id.value()] = unit.sensor;
    groups[unit.id.value()] = nominal_group->id;
    out.units.push_back(std::move(unit));
  }

  for (int axis = 0; axis < 6; ++axis) {
    FaultUnit unit;
    unit.id = FaultUnitId((axis < 3 ? 0x1000000000000000ULL :
                                      0x2000000000000000ULL) +
                          static_cast<std::uint64_t>(axis % 3));
    unit.sensor = axis < 3 ? SensorType::ImuAccelerometer
                           : SensorType::ImuGyroscope;
    unit.kind = axis < 3 ? FaultKind::AccelAxisIntervalConstant
                         : FaultKind::GyroAxisIntervalConstant;
    unit.axis = axis % 3;
    unit.epoch_begin = tx.previous_epoch;
    unit.epoch_end = tx.proposed_epoch;
    unit.time_begin = tx.begin;
    unit.time_end = tx.end;
    unit.affected_groups = {tx.imu_group.id};
    unit.parameter_dimension = 1;
    unit.prior_probability_bound = axis < 3 ? config_.accel_prior_bound
                                            : config_.gyro_prior_bound;
    Eigen::VectorXd embedded = Eigen::VectorXd::Zero(window.H.rows());
    embedded.segment(imu_offset, imu_block->residual_whitened.size()) =
        axis < 3 ? imu.accel_axis[axis] : imu.gyro_axis[axis - 3];
    unit.subspace_model.map = embedded;
    unit.subspace_model.model_id = axis < 3
        ? "imu_accel_axis_interval_constant_v2"
        : "imu_gyro_axis_interval_constant_v2";
    maps[unit.id.value()] = embedded;
    sensors[unit.id.value()] = unit.sensor;
    groups[unit.id.value()] = tx.imu_group.id;
    out.units.push_back(std::move(unit));
  }

  std::uint64_t hypothesis_id = 1;
  auto append_hypothesis = [&](const std::vector<FaultUnitId>& units) {
    FaultHypothesisV2 hypothesis;
    hypothesis.id = HypothesisId(hypothesis_id++);
    hypothesis.units = units;
    hypothesis.A.resize(window.H.rows(), units.size());
    hypothesis.prior_probability_bound = 1.0;
    for (std::size_t i = 0; i < units.size(); ++i) {
      hypothesis.A.col(i) = maps.at(units[i].value());
      const auto found = std::find_if(out.units.begin(), out.units.end(),
          [&](const FaultUnit& unit) { return unit.id == units[i]; });
      hypothesis.prior_probability_bound = std::min(
          hypothesis.prior_probability_bound,
          found->prior_probability_bound);  // conservative Frechet upper bound
    }
    hypothesis.p_md_allocation = units.size() == 1 &&
        sensors.at(units.front().value()) == SensorType::Uwb
        ? config_.uwb_p_md : config_.imu_p_md;
    out.hypotheses.push_back(std::move(hypothesis));
  };
  for (const auto& unit : out.units) append_hypothesis({unit.id});
  for (const auto& uwb : out.units) {
    if (uwb.sensor != SensorType::Uwb) continue;
    for (const auto& imu_unit : out.units) {
      const bool include =
          (imu_unit.sensor == SensorType::ImuAccelerometer &&
           config_.include_uwb_accel_combinations) ||
          (imu_unit.sensor == SensorType::ImuGyroscope &&
           config_.include_uwb_gyro_combinations);
      if (include) append_hypothesis({uwb.id, imu_unit.id});
    }
  }
  const double allocation = out.hypotheses.empty() ? 0.0 :
      config_.total_hmi_allocation / static_cast<double>(out.hypotheses.size());
  for (auto& hypothesis : out.hypotheses) hypothesis.hmi_allocation = allocation;

  ExclusionAction keep;
  keep.id = ExclusionActionId(1);
  keep.action_model_id = "KEEP_ALL";
  out.actions.push_back(keep);
  std::uint64_t action_id = 2;
  std::map<std::uint64_t, ExclusionAction> uwb_actions;
  for (const auto& pair : anchor_rows) {
    const FaultUnitId unit(pair.first);
    const auto* replacement = replacementFor(tx, unit);
    if (!replacement) continue;
    std::vector<int> retained;
    for (int i = 0; i < nominal_uwb_block->residual_raw.size(); ++i) {
      if (std::find(pair.second.begin(), pair.second.end(), i) == pair.second.end())
        retained.push_back(i);
    }
    LinearizedFactorBlock block;
    block.group_id = replacement->id;
    block.kind = FactorKind::UwbBatch;
    block.sensor = SensorType::Uwb;
    block.role = RowRole::Measurement;
    block.covariance = replacement->raw_covariance;
    block.whitener = whitening(block.covariance);
    block.jacobian_raw.resize(retained.size(), window.H.cols());
    block.residual_raw.resize(retained.size());
    for (std::size_t row = 0; row < retained.size(); ++row) {
      block.jacobian_raw.row(row) = nominal_uwb_block->jacobian_raw.row(retained[row]);
      block.residual_raw(row) = nominal_uwb_block->residual_raw(retained[row]);
    }
    block.jacobian_whitened = block.whitener * block.jacobian_raw;
    block.residual_whitened = block.whitener * block.residual_raw;
    block.version = window.version;
    block.whitening_model_id = replacement->noise_model_id;
    ExclusionAction action;
    action.id = ExclusionActionId(action_id++);
    action.covered_units = {unit};
    action.groups_to_remove = {nominal_group->id};
    action.groups_to_add = {replacement->id};
    action.added_blocks = {block};
    action.exclusion_cardinality = 1;
    action.action_model_id = "UWB_ANCHOR_EXCLUSION";
    uwb_actions[pair.first] = action;
    out.actions.push_back(std::move(action));
  }
  std::map<std::uint64_t, ExclusionAction> imu_actions;
  for (const auto& unit : out.units) {
    if (unit.sensor != SensorType::ImuAccelerometer &&
        unit.sensor != SensorType::ImuGyroscope) continue;
    ExclusionAction action;
    action.id = ExclusionActionId(action_id++);
    action.covered_units = {unit.id};
    action.groups_to_remove = {tx.imu_group.id};
    action.groups_to_add = {tx.generic_bridge_group.id};
    action.added_blocks = {embedded_bridge};
    action.bridge_mode = BridgeMode::GenericKinematic;
    action.exclusion_cardinality = 1;
    action.action_model_id = "IMU_AXIS_EXCLUSION_GENERIC_BRIDGE";
    imu_actions[unit.id.value()] = action;
    out.actions.push_back(std::move(action));
  }
  for (const auto& uwb : out.units) {
    if (uwb.sensor != SensorType::Uwb || !uwb_actions.count(uwb.id.value())) continue;
    for (const auto& imu_unit : out.units) {
      if (!imu_actions.count(imu_unit.id.value())) continue;
      ExclusionAction action = uwb_actions.at(uwb.id.value());
      action.id = ExclusionActionId(action_id++);
      action.covered_units.push_back(imu_unit.id);
      action.groups_to_remove.push_back(tx.imu_group.id);
      action.groups_to_add.push_back(tx.generic_bridge_group.id);
      action.added_blocks.push_back(embedded_bridge);
      action.bridge_mode = BridgeMode::GenericKinematic;
      action.exclusion_cardinality = 2;
      action.action_model_id = "UWB_PLUS_IMU_UNION_EXCLUSION";
      out.actions.push_back(std::move(action));
    }
  }
  if (out.actions.size() > config_.max_candidate_count) {
    throw std::runtime_error("fault action count exceeds configured bound");
  }
  return out;
}

}  // namespace uwb_imu_pl
