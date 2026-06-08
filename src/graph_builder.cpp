#include "uifgo/graph_builder.h"
#include "uifgo/imu_preint.h"
#include "uifgo/uwb_factor.h"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <cmath>
#include <iostream>

namespace uifgo {
using namespace gtsam;
using symbol_shorthand::X;  // Pose3
using symbol_shorthand::V;  // Vector3 velocity
using symbol_shorthand::B;  // imuBias::ConstantBias
using symbol_shorthand::L;  // Point3 lever arm
using symbol_shorthand::A;  // Point3 anchor correction
using symbol_shorthand::Z;  // double range bias

GraphBuilder::GraphBuilder(const Config& cfg) : cfg_(cfg), anchors_(cfg.anchors) {}

gtsam::Key GraphBuilder::lever_key() const  { return L(0); }
gtsam::Key GraphBuilder::anchor_key(int m) const { return A(m); }
gtsam::Key GraphBuilder::bias_key(int m) const   { return Z(m); }

double GraphBuilder::AdaptiveSigma(double dt_since_last) const {
  double extra = (dt_since_last > 0) ? (cfg_.v_max * dt_since_last / 3.0) : 0.0;
  return std::sqrt(cfg_.sigma_range * cfg_.sigma_range + extra * extra);
}

void GraphBuilder::AddUwbFactorsForFrame(
    size_t kf_idx, const UwbFrame& frame,
    NonlinearFactorGraph* graph,
    std::vector<size_t>* uwb_indices,
    Values* values,
    bool first_frame) {

  for (const UwbRange& r : frame.ranges) {
    int m = r.anchor_id;

    // Lookup nominal anchor position
    auto it = std::find_if(anchors_.begin(), anchors_.end(),
        [m](const AnchorConfig& a) { return a.id == m; });
    if (it == anchors_.end()) {
      std::cerr << "Warning: unknown anchor " << m << " in frame kf=" << kf_idx << "\n";
      continue;
    }
    const Point3& A_m = it->pos;

    // Time-adaptive sigma
    double dt_last = 0.0;
    auto tit = last_anchor_time_.find(m);
    if (tit != last_anchor_time_.end()) {
      dt_last = frame.t - tit->second;
    }
    double sigma = AdaptiveSigma(dt_last);
    last_anchor_time_[m] = frame.t;

    auto factor = MakeUwbFactor(
        X(kf_idx), lever_key(), anchor_key(m), bias_key(m),
        A_m, cfg_.lever_arm_init,
        r.dist, sigma,
        cfg_.calib_lever, cfg_.calib_anchor, cfg_.calib_range_bias);

    size_t idx = graph->size();
    graph->add(factor);
    uwb_indices->push_back(idx);
  }
}

void GraphBuilder::Build(const std::vector<UwbFrame>& uwb_kf,
                          const std::vector<ImuSample>& imu,
                          const InitResult& init,
                          NonlinearFactorGraph* graph,
                          Values* values,
                          std::vector<size_t>* uwb_indices) {
  const size_t N = uwb_kf.size();
  if (N < 2) {
    std::cerr << "GraphBuilder: need >= 2 keyframes, got " << N << "\n";
    return;
  }

  graph->resize(0);
  values->clear();
  uwb_indices->clear();
  last_anchor_time_.clear();

  // Bias vector
  const imuBias::ConstantBias bias0(init.ba0, init.bg0);

  // --- Step 1: First keyframe priors + initial Values ---
  auto posePriorNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished());
  auto velPriorNoise  = noiseModel::Isotropic::Sigma(3, 0.1);
  auto biasPriorNoise = noiseModel::Isotropic::Sigma(6, 0.01);

  graph->addPrior(X(0), init.T0,  posePriorNoise);
  graph->addPrior(V(0), init.v0,  velPriorNoise);
  graph->addPrior(B(0), bias0,    biasPriorNoise);

  values->insert(X(0), init.T0);
  values->insert(V(0), init.v0);
  values->insert(B(0), bias0);

  // --- Step 2: Calibration variables + loose priors ---
  if (cfg_.calib_lever) {
    values->insert(lever_key(), cfg_.lever_arm_init);
    graph->addPrior(lever_key(), cfg_.lever_arm_init,
                    noiseModel::Isotropic::Sigma(3, cfg_.lever_prior_sigma));
  }
  for (const auto& a : anchors_) {
    if (cfg_.calib_anchor) {
      values->insert(anchor_key(a.id), Point3(0, 0, 0));
      graph->addPrior(anchor_key(a.id), Point3(0, 0, 0),
                      noiseModel::Isotropic::Sigma(3, a.prior_sigma));
    }
    if (cfg_.calib_range_bias) {
      values->insert<double>(bias_key(a.id), 0.0);
      graph->addPrior<double>(bias_key(a.id), 0.0,
                      noiseModel::Isotropic::Sigma(1, cfg_.range_bias_sigma));
    }
  }

  // --- Step 3: UWB factors for first frame ---
  AddUwbFactorsForFrame(0, uwb_kf[0], graph, uwb_indices, values, true);

  // --- Step 4: Loop keyframes ---
  ImuPreintegrator pim(cfg_, init.gravity_world);
  size_t i_imu = 0;
  while (i_imu < imu.size() && imu[i_imu].t <= uwb_kf[0].t) ++i_imu;

  Pose3   prev_pose = init.T0;
  Vector3 prev_vel  = init.v0;
  imuBias::ConstantBias prev_bias = bias0;

  for (size_t k = 1; k < N; ++k) {
    // --- 4a: IMU preintegration ---
    pim.Reset(prev_bias);
    i_imu = IntegrateBetween(imu, i_imu, uwb_kf[k-1].t, uwb_kf[k].t, &pim);

    // --- 4b: CombinedImuFactor ---
    graph->add(CombinedImuFactor(
        X(k-1), V(k-1), X(k), V(k), B(k-1), B(k), pim.Pim()));

    // --- 4c: Predict initial values ---
    gtsam::NavState pred = pim.Pim().predict(gtsam::NavState(prev_pose, prev_vel), prev_bias);
    values->insert(X(k), pred.pose());
    values->insert(V(k), pred.velocity());
    values->insert(B(k), prev_bias);

    // --- 4d: UWB factors ---
    AddUwbFactorsForFrame(k, uwb_kf[k], graph, uwb_indices, values, false);

    prev_pose = pred.pose();
    prev_vel  = pred.velocity();
  }

  std::cout << "GraphBuilder: " << N << " keyframes, "
            << graph->size() << " factors ("
            << uwb_indices->size() << " UWB).\n";
}

}  // namespace uifgo
