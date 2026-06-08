#include "uifgo/data_loader.h"

#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <uwb_imu_fgo/LinktrackNode2.h>
#include <uwb_imu_fgo/LinktrackNodeframe3.h>

#include <algorithm>
#include <iostream>

namespace uifgo {

DataLoader::DataLoader(const Config& cfg)
    : imu_topic_(cfg.imu_topic),
      uwb_topic_(cfg.uwb_topic),
      gt_topic_(cfg.vicon_topic),
      cfg_(cfg) {}

bool DataLoader::LoadFromBag(const std::string& bag_path,
                             std::vector<ImuSample>* out_imu,
                             std::vector<UwbFrame>* out_uwb) {
  if (bag_path.empty()) {
    std::cerr << "DataLoader: empty bag path.\n";
    return false;
  }

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    std::cerr << "DataLoader: failed to open bag: " << e.what() << "\n";
    return false;
  }

  std::vector<std::string> topics = {imu_topic_, uwb_topic_};
  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_start =
      view_full.getBeginTime() + ros::Duration(cfg_.bag_start);
  ros::Time time_finish = (cfg_.bag_durr < 0)
                              ? view_full.getEndTime()
                              : time_start + ros::Duration(cfg_.bag_durr);

  rosbag::View view;
  view.addQuery(bag, rosbag::TopicQuery(topics), time_start, time_finish);

  if (view.size() == 0) {
    std::cerr << "DataLoader: no messages on topics " << imu_topic_ << " / "
              << uwb_topic_ << "\n";
    bag.close();
    return false;
  }

  std::cout << "DataLoader: loading bag " << bag_path << " (" << view.size()
            << " messages)\n";

  out_imu->clear();
  out_uwb->clear();

  for (const rosbag::MessageInstance& m : view) {
    std::string topic = m.getTopic();

    if (topic == imu_topic_) {
      sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      if (!imu_msg) continue;

      ImuSample s;
      s.t = imu_msg->header.stamp.toSec();
      // Many IMU drivers (e.g. Livox) publish linear_acceleration in g-units
      // (1.0 = 1g) rather than m/s^2.  Convert to m/s^2 by scaling with
      // configured gravity so that GTSAM preintegration receives raw SI units.
      const double g_scale = cfg_.gravity;  // typically 9.81
      s.acc = Eigen::Vector3d(imu_msg->linear_acceleration.x * g_scale,
                              imu_msg->linear_acceleration.y * g_scale,
                              imu_msg->linear_acceleration.z * g_scale);
      s.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                               imu_msg->angular_velocity.y,
                               imu_msg->angular_velocity.z);
      out_imu->push_back(s);

    } else if (topic == uwb_topic_) {
      // Try LinktrackNodeframe3 first
      auto uwb_msg = m.instantiate<uwb_imu_fgo::LinktrackNodeframe3>();
      if (!uwb_msg) continue;

      UwbFrame frame;
      frame.t = uwb_msg->header.stamp.toSec();
      frame.tag_id = uwb_msg->id;

      for (const auto& node : uwb_msg->nodes) {
        UwbRange r;
        r.anchor_id = node.id;
        r.dist = node.dis;
        r.fp_rssi = node.fp_rssi;
        r.rx_rssi = node.rx_rssi;
        frame.ranges.push_back(r);
      }

      if (!frame.ranges.empty()) {
        out_uwb->push_back(frame);
      }
    }
  }

  bag.close();

  // Sort by time
  std::sort(out_imu->begin(), out_imu->end(),
            [](const ImuSample& a, const ImuSample& b) { return a.t < b.t; });
  std::sort(out_uwb->begin(), out_uwb->end(),
            [](const UwbFrame& a, const UwbFrame& b) { return a.t < b.t; });

  std::cout << "DataLoader: loaded " << out_imu->size() << " IMU samples, "
            << out_uwb->size() << " UWB frames.\n";

  // Diagnostic: check IMU accelerometer units after g→m/s^2 scaling
  if (!out_imu->empty()) {
    size_t n_diag = std::min(out_imu->size(), (size_t)200);
    double acc_norm_sum = 0.0;
    for (size_t i = 0; i < n_diag; ++i)
      acc_norm_sum += out_imu->at(i).acc.norm();
    double acc_norm_mean = acc_norm_sum / n_diag;
    std::cout << "[DIAG] IMU acc norm after scaling (first " << n_diag
              << " samples): mean = " << acc_norm_mean << " m/s^2"
              << "  |  expected ~9.81 (static)\n";
    if (acc_norm_mean < 2.0) {
      std::cout << "[WARN] IMU acc norm still small after g-scaling — "
                << "data may be gravity-compensated rather than g-unit.\n";
    }
  }

  return (!out_imu->empty() && !out_uwb->empty());
}

std::vector<NavState> DataLoader::LoadGroundTruth(const std::string& bag_path) {
  std::vector<NavState> gt;
  if (gt_topic_.empty()) {
    std::cout << "LoadGroundTruth: no vicon topic configured, skipping.\n";
    return gt;
  }

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    std::cerr << "LoadGroundTruth: cannot open bag: " << e.what() << "\n";
    return gt;
  }

  rosbag::View view;
  view.addQuery(bag, rosbag::TopicQuery({gt_topic_}));
  if (view.size() == 0) {
    std::cerr << "LoadGroundTruth: no messages on " << gt_topic_ << "\n";
    bag.close();
    return gt;
  }

  for (const auto& m : view) {
    auto pose_msg = m.instantiate<geometry_msgs::PoseStamped>();
    if (!pose_msg) continue;
    NavState s;
    s.t = pose_msg->header.stamp.toSec();
    auto& q = pose_msg->pose.orientation;
    auto& p = pose_msg->pose.position;
    s.T = gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                       gtsam::Point3(p.x, p.y, p.z));
    s.v = gtsam::Vector3::Zero();
    s.ba = gtsam::Vector3::Zero();
    s.bg = gtsam::Vector3::Zero();
    gt.push_back(s);
  }
  bag.close();
  std::cout << "LoadGroundTruth: " << gt.size() << " poses from " << gt_topic_
            << "\n";
  return gt;
}

}  // namespace uifgo
