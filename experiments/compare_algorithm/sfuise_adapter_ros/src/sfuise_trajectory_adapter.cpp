#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <set>
#include <stdexcept>
#include <string>

#include <isas_msgs/RTLSStick.h>
#include <sfuise_msgs/Calib.h>
#include <sfuise_msgs/Estimate.h>
#include <std_msgs/Int64.h>

#include "SplineState.h"

namespace {

class TrajectoryAdapter {
 public:
  explicit TrajectoryAdapter(ros::NodeHandle& private_nh) {
    if (!private_nh.getParam("output_path", output_path_) || output_path_.empty()) {
      throw std::runtime_error("~output_path is required");
    }
    private_nh.param<std::string>("metadata_path", metadata_path_, output_path_ + ".meta");
    sub_start_ = nh_.subscribe("/SplineFusion/start_time", 10,
                              &TrajectoryAdapter::StartCallback, this);
    sub_calib_ = nh_.subscribe("/SplineFusion/sys_calib", 100,
                              &TrajectoryAdapter::CalibCallback, this);
    sub_estimate_ = nh_.subscribe("/SplineFusion/est_window", 1000,
                                 &TrajectoryAdapter::EstimateCallback, this);
    sub_toa_ = nh_.subscribe("/EstimationInterface/toa_ds", 2000,
                            &TrajectoryAdapter::ToaCallback, this);
  }

  ~TrajectoryAdapter() {
    try {
      WriteOutputs();
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("SFUISE trajectory adapter export failed: " << e.what());
    }
  }

 private:
  void StartCallback(const std_msgs::Int64::ConstPtr& msg) {
    global_start_ns_ = msg->data;
    have_start_ = true;
  }

  void CalibCallback(const sfuise_msgs::Calib::ConstPtr& msg) {
    q_nav_uwb_ = Eigen::Quaterniond(msg->q_nav_uwb.w, msg->q_nav_uwb.x,
                                   msg->q_nav_uwb.y, msg->q_nav_uwb.z);
    t_nav_uwb_ = Eigen::Vector3d(msg->t_nav_uwb.x, msg->t_nav_uwb.y,
                                msg->t_nav_uwb.z);
    if (std::abs(q_nav_uwb_.norm() - 1.0) > 1e-5) q_nav_uwb_.normalize();
    have_calib_ = true;
    ++calibration_messages_;
  }

  void ToaCallback(const isas_msgs::RTLSStick::ConstPtr& msg) {
    toa_times_ns_.insert(static_cast<int64_t>(msg->header.stamp.toNSec()));
  }

  void EstimateCallback(const sfuise_msgs::Estimate::ConstPtr& msg) {
    const auto& spline_msg = msg->spline;
    SplineState local;
    local.init(spline_msg.dt, 0, spline_msg.start_t,
               static_cast<int>(spline_msg.start_idx));
    for (const auto& knot : spline_msg.knots) {
      Eigen::Matrix<double, 6, 1> bias;
      bias << knot.bias_acc.x, knot.bias_acc.y, knot.bias_acc.z,
          knot.bias_gyro.x, knot.bias_gyro.y, knot.bias_gyro.z;
      local.addOneStateKnot(
          Eigen::Quaterniond(knot.orientation.w, knot.orientation.x,
                             knot.orientation.y, knot.orientation.z),
          Eigen::Vector3d(knot.position.x, knot.position.y, knot.position.z), bias);
    }
    if (spline_msg.idles.size() != 3) {
      throw std::runtime_error("SFUISE spline does not contain three idle knots");
    }
    for (int i = 0; i < 3; ++i) {
      const auto& knot = spline_msg.idles[i];
      Eigen::Matrix<double, 6, 1> bias;
      bias << knot.bias_acc.x, knot.bias_acc.y, knot.bias_acc.z,
          knot.bias_gyro.x, knot.bias_gyro.y, knot.bias_gyro.z;
      local.setIdles(i, Eigen::Vector3d(knot.position.x, knot.position.y,
                                       knot.position.z),
                     Eigen::Quaterniond(knot.orientation.w, knot.orientation.x,
                                        knot.orientation.y, knot.orientation.z),
                     bias);
    }
    if (!have_global_) {
      const int64_t origin = have_start_
          ? global_start_ns_
          : spline_msg.start_t - static_cast<int64_t>(spline_msg.start_idx) * spline_msg.dt;
      global_.init(spline_msg.dt, 0, origin);
      have_global_ = true;
    }
    global_.updateKnots(&local);
    last_native_average_runtime_ = msg->runtime.data;
    ++estimate_messages_;
  }

  void WriteOutputs() {
    if (written_) return;
    written_ = true;
    std::ofstream trajectory(output_path_);
    if (!trajectory) throw std::runtime_error("cannot open output trajectory");
    trajectory << std::setprecision(17);
    std::size_t count = 0;
    if (have_global_ && have_calib_) {
      const int64_t lo = global_.minTimeNs();
      const int64_t hi = global_.maxTimeNs();
      for (const int64_t t_ns : toa_times_ns_) {
        if (t_ns < lo || t_ns > hi) continue;
        Eigen::Quaterniond q_nav_body;
        global_.itpQuaternion(t_ns, &q_nav_body);
        const Eigen::Vector3d p_nav_body = global_.itpPosition(t_ns);
        Eigen::Quaterniond q_uwb_body = q_nav_uwb_ * q_nav_body;
        q_uwb_body.normalize();
        const Eigen::Vector3d p_uwb_body = q_nav_uwb_ * p_nav_body + t_nav_uwb_;
        trajectory << static_cast<double>(t_ns) * 1e-9 << ' '
                   << p_uwb_body.x() << ' ' << p_uwb_body.y() << ' '
                   << p_uwb_body.z() << ' ' << q_uwb_body.x() << ' '
                   << q_uwb_body.y() << ' ' << q_uwb_body.z() << ' '
                   << q_uwb_body.w() << '\n';
        ++count;
      }
    }
    std::ofstream metadata(metadata_path_);
    if (!metadata) throw std::runtime_error("cannot open output metadata");
    metadata << "schema=sfuise_adapter_runtime_v1\n"
             << "measurement_mode=ABSOLUTE_TOA\n"
             << "reference_point=BODY_IMU_ORIGIN\n"
             << "sampling_source=TOA_SENSOR_TIMESTAMPS_NO_GT\n"
             << "estimate_messages=" << estimate_messages_ << '\n'
             << "calibration_messages=" << calibration_messages_ << '\n'
             << "toa_timestamps=" << toa_times_ns_.size() << '\n'
             << "trajectory_samples=" << count << '\n'
             << std::setprecision(17)
             << "native_average_window_runtime=" << last_native_average_runtime_ << '\n';
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_start_, sub_calib_, sub_estimate_, sub_toa_;
  SplineState global_;
  Eigen::Quaterniond q_nav_uwb_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d t_nav_uwb_{Eigen::Vector3d::Zero()};
  std::set<int64_t> toa_times_ns_;
  std::string output_path_, metadata_path_;
  int64_t global_start_ns_{0};
  std::size_t estimate_messages_{0}, calibration_messages_{0};
  double last_native_average_runtime_{0.0};
  bool have_start_{false}, have_global_{false}, have_calib_{false}, written_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "sfuise_trajectory_adapter");
  ros::NodeHandle private_nh("~");
  try {
    TrajectoryAdapter adapter(private_nh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    return 1;
  }
  return 0;
}
