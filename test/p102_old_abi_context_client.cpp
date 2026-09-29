#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"
#include "uwb_imu_pl/integrity/hypothesis_evidence.hpp"

#include <cstddef>
#include <iostream>
#include <memory>
#include <vector>

namespace {

uwb_imu_pl::LinearizedFactorBlock block(
    std::uint64_t id, const Eigen::MatrixXd& h, const Eigen::VectorXd& z,
    const uwb_imu_pl::LinearizationVersion& version) {
  uwb_imu_pl::LinearizedFactorBlock value;
  value.group_id = uwb_imu_pl::FactorGroupId(id);
  value.jacobian_whitened = h;
  value.residual_whitened = z;
  value.jacobian_raw = h;
  value.residual_raw = z;
  value.covariance = Eigen::MatrixXd::Identity(h.rows(), h.rows());
  value.whitener = value.covariance;
  value.version = version;
  return value;
}

}  // namespace

int main() {
  using namespace uwb_imu_pl;
  static_assert(sizeof(FrozenHypothesisNumerics) == 320,
                "golden P1-01 context size changed");
  static_assert(offsetof(FrozenHypothesisNumerics, reason) == 264,
                "golden P1-01 reason offset changed");

  LinearizedIntegrityWindow window;
  window.id = WindowId(10201);
  window.version = {10, 2, 0, 1};
  window.blocks.push_back(block(
      1, Eigen::MatrixXd::Identity(2, 2) * 3.0,
      Eigen::Vector2d(0.1, -0.2), window.version));
  Eigen::MatrixXd h1(3, 2);
  h1 << 1.0, 0.2, 0.1, 1.0, 0.7, -0.4;
  window.blocks.push_back(block(
      2, h1, Eigen::Vector3d(0.3, -0.1, 0.2), window.version));
  window.protected_state_map = Eigen::MatrixXd::Zero(3, 2);
  window.protected_state_map(0, 0) = 1.0;
  window.protected_state_map(1, 1) = 1.0;
  finalizeIntegrityWindow(&window, 1e-10, 1e10);
  const auto handle = freezeIntegrityWindowCopy(window);
  const auto admission = admitFrozenIntegrityWindow(handle);
  if (!admission) return 2;

  FaultModeBasis mode;
  mode.id = FaultModeId(10201);
  mode.sensor = SensorType::Uwb;
  mode.parameter_dimension = 1;
  mode.raw_group_maps[FactorGroupId(1)] = Eigen::Vector2d::Ones();
  std::vector<FaultModeBasis> modes{mode};
  FaultHypothesisV2 hypothesis;
  hypothesis.id = HypothesisId(10201);
  hypothesis.modes = {mode.id};
  std::vector<FaultHypothesisV2> hypotheses{hypothesis};
  std::shared_ptr<const FrozenHypothesisNumerics> context;
  const auto evidence = HypothesisEvidenceEvaluator().evaluateAll(
      admission, modes, &hypotheses, 100.0, &context);
  if (evidence.size() != 1 || !context || !context->valid) return 3;
  if (context->window_id != admission.window().id ||
      context->pl_entries.size() != 1 || context->mode_count != 1 ||
      context->hypothesis_count != 1 || context->reason != "" ||
      !context->compact_mode_used || context->compact_capacity_exceeded) {
    return 4;
  }
  std::cout << "FrozenHypothesisNumerics " << sizeof(*context) << ' '
            << offsetof(FrozenHypothesisNumerics, reason) << ' '
            << context->pl_entries.size() << ' ' << context->mode_count
            << ' ' << context->hypothesis_count << '\n';
  context.reset();
  return 0;
}
