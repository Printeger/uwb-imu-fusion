#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <isas_msgs/RTLSStick.h>
#include <sfuise_msgs/Calib.h>
#include <sfuise_msgs/Estimate.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Imu.h>

#include "SplineState.h"

namespace {

class TrajectoryAdapter {
 public:
  explicit TrajectoryAdapter(ros::NodeHandle& private_nh) {
    if (!private_nh.getParam("output_path", output_path_) || output_path_.empty())
      throw std::runtime_error("~output_path is required");
    private_nh.param<std::string>("metadata_path", metadata_path_, output_path_ + ".meta");
    online_.open(output_path_ + ".online.tum");
    cutoff_.open(output_path_ + ".online_cutoff.csv");
    if (!online_ || !cutoff_) throw std::runtime_error("cannot open online export");
    online_ << std::setprecision(17);
    cutoff_ << "state_ns,window_information_cutoff_ns,observed_sensor_cutoff_ns,estimate_index,calibration_index\n";
    std::vector<double> lever{0.0, 0.0, 0.0};
    private_nh.getParam("lever_arm_body_m", lever);
    if (lever.size() != 3) throw std::runtime_error("~lever_arm_body_m must have 3 values");
    lever_body_ = Eigen::Vector3d(lever[0], lever[1], lever[2]);
    sub_imu_ = nh_.subscribe("/EstimationInterface/imu_ds", 2000,
                             &TrajectoryAdapter::ImuCallback, this);
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
    try { WriteOutputs(); }
    catch (const std::exception& e) { ROS_ERROR_STREAM("SFUISE export failed: " << e.what()); }
  }

 private:
  void StartCallback(const std_msgs::Int64::ConstPtr& msg) {
    global_start_ns_ = msg->data;
    have_start_ = true;
  }

  void CalibCallback(const sfuise_msgs::Calib::ConstPtr& msg) {
    q_nav_uwb_ = Eigen::Quaterniond(msg->q_nav_uwb.w, msg->q_nav_uwb.x,
                                    msg->q_nav_uwb.y, msg->q_nav_uwb.z).normalized();
    t_nav_uwb_ = Eigen::Vector3d(msg->t_nav_uwb.x, msg->t_nav_uwb.y,
                                 msg->t_nav_uwb.z);
    have_calib_ = true;
    ++calibration_messages_;
  }

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    latest_sensor_ns_ = std::max(latest_sensor_ns_, static_cast<int64_t>(msg->header.stamp.toNSec()));
  }

  void ToaCallback(const isas_msgs::RTLSStick::ConstPtr& msg) {
    toa_times_ns_.insert(static_cast<int64_t>(msg->header.stamp.toNSec()));
    latest_sensor_ns_ = std::max(latest_sensor_ns_, static_cast<int64_t>(msg->header.stamp.toNSec()));
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
    if (spline_msg.idles.size() != 3) return;
    for (int i = 0; i < 3; ++i) {
      const auto& knot = spline_msg.idles[i];
      Eigen::Matrix<double, 6, 1> bias;
      bias << knot.bias_acc.x, knot.bias_acc.y, knot.bias_acc.z,
              knot.bias_gyro.x, knot.bias_gyro.y, knot.bias_gyro.z;
      local.setIdles(i, Eigen::Vector3d(knot.position.x, knot.position.y,
                                       knot.position.z),
                     Eigen::Quaterniond(knot.orientation.w, knot.orientation.x,
                                        knot.orientation.y, knot.orientation.z), bias);
    }
    if (!have_global_) {
      const int64_t origin = have_start_ ? global_start_ns_
          : spline_msg.start_t - static_cast<int64_t>(spline_msg.start_idx) * spline_msg.dt;
      global_.init(spline_msg.dt, 0, origin);
      have_global_ = true;
    }
    // Freeze one current-window output when each estimate arrives. Never
    // rewrite past samples using later knots or the final calibration.
    if (have_calib_ && local.numKnots() >= 2) {
      const int64_t t_ns = local.maxTimeNs();
      if (t_ns > last_online_ns_) {
        Eigen::Quaterniond q_nav_body;
        local.itpQuaternion(t_ns, &q_nav_body); q_nav_body.normalize();
        const auto q = (q_nav_uwb_ * q_nav_body).normalized();
        const Eigen::Vector3d p = q_nav_uwb_ *
            (local.itpPosition(t_ns) + q_nav_body * lever_body_) + t_nav_uwb_;
        online_ << static_cast<double>(t_ns)*1e-9 << ' ' << p.x() << ' '
                << p.y() << ' ' << p.z() << ' ' << q.x() << ' ' << q.y()
                << ' ' << q.z() << ' ' << q.w() << '\n';
        cutoff_ << t_ns << ',' << t_ns+1 << ',' << latest_sensor_ns_ << ','
                << estimate_messages_+1 << ',' << calibration_messages_ << '\n';
        online_records_.push_back({t_ns,local.itpPosition(t_ns),q_nav_body,
                                   q_nav_uwb_,t_nav_uwb_});
        last_online_ns_ = t_ns; ++online_samples_;
      }
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
      const int64_t lo = global_.minTimeNs(), hi = global_.maxTimeNs();
      for (const int64_t t_ns : toa_times_ns_) {
        if (t_ns < lo || t_ns > hi) continue;
        Eigen::Quaterniond q_nav_body;
        global_.itpQuaternion(t_ns, &q_nav_body);
        q_nav_body.normalize();
        const Eigen::Vector3d p_nav_body = global_.itpPosition(t_ns);
        Eigen::Quaterniond q_uwb_body = (q_nav_uwb_ * q_nav_body).normalized();
        // SFUISE optimizes the body/IMU origin.  The benchmark contract evaluates
        // the physical UWB tag, matching the range residual's configured lever.
        const Eigen::Vector3d p_uwb_tag =
            q_nav_uwb_ * (p_nav_body + q_nav_body * lever_body_) + t_nav_uwb_;
        trajectory << static_cast<double>(t_ns) * 1e-9 << ' '
                   << p_uwb_tag.x() << ' ' << p_uwb_tag.y() << ' '
                   << p_uwb_tag.z() << ' ' << q_uwb_body.x() << ' '
                   << q_uwb_body.y() << ' ' << q_uwb_body.z() << ' '
                   << q_uwb_body.w() << '\n';
        ++count;
      }
      // One run, one identical timestamp set: separate knot revision from
      // calibration revision without changing the upstream estimator.
      std::ofstream online_final(output_path_+".online_final_calibration.tum");
      std::ofstream history_online(output_path_+".history_online_calibration.tum");
      std::ofstream history_final(output_path_+".history_final_calibration.tum");
      if(!online_final || !history_online || !history_final)throw std::runtime_error("cannot open four-view exports");
      auto write=[&](std::ofstream& stream,int64_t t,const Eigen::Vector3d& p,
                     const Eigen::Quaterniond& q,const Eigen::Quaterniond& world_q,
                     const Eigen::Vector3d& world_p) {
        const auto rotation=(world_q*q).normalized();
        const Eigen::Vector3d tag=world_q*(p+q*lever_body_)+world_p;
        stream<<std::setprecision(17)<<static_cast<double>(t)*1e-9<<' '
          <<tag.x()<<' '<<tag.y()<<' '<<tag.z()<<' '<<rotation.x()<<' '
          <<rotation.y()<<' '<<rotation.z()<<' '<<rotation.w()<<'\n';
      };
      for(const auto& record:online_records_) {
        if(record.time<lo || record.time>hi)continue;
        Eigen::Quaterniond history_q;global_.itpQuaternion(record.time,&history_q);history_q.normalize();
        const auto history_p=global_.itpPosition(record.time);
        write(online_final,record.time,record.position,record.orientation,q_nav_uwb_,t_nav_uwb_);
        write(history_online,record.time,history_p,history_q,record.world_rotation,record.world_translation);
        write(history_final,record.time,history_p,history_q,q_nav_uwb_,t_nav_uwb_);
      }
    }
    std::ofstream metadata(metadata_path_);
    if (!metadata) throw std::runtime_error("cannot open output metadata");
    metadata << "schema=sfuise_adapter_runtime_v1\n"
             << "measurement_mode=ABSOLUTE_TOA\n"
             << "reference_point=UWB_TAG\n"
             << "sampling_source=TOA_SENSOR_TIMESTAMPS_NO_GT\n"
             << "estimate_messages=" << estimate_messages_ << '\n'
             << "calibration_messages=" << calibration_messages_ << '\n'
             << "toa_timestamps=" << toa_times_ns_.size() << '\n'
             << "trajectory_samples=" << count << '\n'
             << "historical_semantics=final_knots_final_calibration\n"
             << "online_semantics=current_window_last_observed_calibration\n"
             << "online_samples=" << online_samples_ << '\n'
             << "online_cutoff_semantics=window_boundary_and_observed_sensor_timestamps;message_has_no_exact_input_receipt\n"
             << std::setprecision(17)
             << "native_average_window_runtime=" << last_native_average_runtime_ << '\n';
  }

  ros::NodeHandle nh_;
  struct OnlineRecord {
    int64_t time;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation,world_rotation;
    Eigen::Vector3d world_translation;
  };
  std::vector<OnlineRecord> online_records_;
  ros::Subscriber sub_start_, sub_calib_, sub_estimate_, sub_toa_, sub_imu_;
  SplineState global_;
  Eigen::Quaterniond q_nav_uwb_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d t_nav_uwb_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d lever_body_{Eigen::Vector3d::Zero()};
  std::set<int64_t> toa_times_ns_;
  std::string output_path_, metadata_path_;
  std::ofstream online_, cutoff_;
  int64_t last_online_ns_{-1}, latest_sensor_ns_{0};
  std::size_t online_samples_{0};
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
