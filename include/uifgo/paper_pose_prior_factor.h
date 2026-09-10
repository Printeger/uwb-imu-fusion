#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>

namespace uifgo {

// Paper-path-only replacement for GTSAM 4.2 PriorFactor<Pose3>.  The linked
// implementation returns -Local(x, prior) but reports an identity Jacobian.
// This factor preserves that residual exactly and supplies its derivative with
// respect to GTSAM's Pose3 retract coordinates.
class PaperPosePriorFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  PaperPosePriorFactor(gtsam::Key key, const gtsam::Pose3& prior,
                       const gtsam::SharedNoiseModel& model)
      : Base(model, key), prior_(prior) {}

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(
        new PaperPosePriorFactor(*this));
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3& x,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    if (H) {
      gtsam::Matrix66 local_H_x;
      const gtsam::Vector6 local =
          gtsam::traits<gtsam::Pose3>::Local(x, prior_, local_H_x);
      *H = -local_H_x;
      return -local;
    }
    return -gtsam::traits<gtsam::Pose3>::Local(x, prior_);
  }

  void print(
      const std::string& prefix,
      const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter)
      const override {
    std::cout << prefix << "PaperPosePriorFactor on "
              << formatter(this->key()) << "\n";
    prior_.print("  prior mean: ");
    if (this->noiseModel_) this->noiseModel_->print("  noise model: ");
  }

  bool equals(const gtsam::NonlinearFactor& expected,
              double tolerance = 1e-9) const override {
    const auto* other = dynamic_cast<const PaperPosePriorFactor*>(&expected);
    return other != nullptr && Base::equals(*other, tolerance) &&
           prior_.equals(other->prior_, tolerance);
  }

  const gtsam::Pose3& prior() const { return prior_; }

 private:
  gtsam::Pose3 prior_;
};

inline std::size_t ReplacePosePriorsForPaperPath(
    gtsam::NonlinearFactorGraph* graph) {
  if (graph == nullptr) {
    throw std::invalid_argument("paper graph must not be null");
  }
  std::size_t replaced = 0;
  for (std::size_t index = 0; index < graph->size(); ++index) {
    const auto linked = boost::dynamic_pointer_cast<
        gtsam::PriorFactor<gtsam::Pose3>>(graph->at(index));
    if (!linked) continue;
    graph->replace(index, gtsam::NonlinearFactor::shared_ptr(
        new PaperPosePriorFactor(linked->key(), linked->prior(),
                                 linked->noiseModel())));
    ++replaced;
  }
  return replaced;
}

inline const char* PaperPosePriorJacobianIdentity() {
  return "PAPER_POSE3_PRIOR_LOCAL_JACOBIAN_V1";
}

}  // namespace uifgo
