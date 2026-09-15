#pragma once

#include <vector>

#include "uifgo/config.h"
#include "uifgo/types.h"

namespace uifgo {

class DataLoader {
 public:
  explicit DataLoader(const Config& cfg);

  // Load IMU and UWB from rosbag.  Returns true on success.
  // out_imu and out_uwb are filled in time-ascending order.
  bool LoadFromBag(const std::string& bag_path, std::vector<ImuSample>* out_imu,
                   std::vector<UwbFrame>* out_uwb);

  // Load MCD-style split bags. The common time interval of both bags is used,
  // then bag.start / bag.durr are applied to that interval.
  bool LoadFromBags(const std::string& imu_bag_path,
                    const std::string& uwb_bag_path,
                    std::vector<ImuSample>* out_imu,
                    std::vector<UwbFrame>* out_uwb);

  // Load NTU VIRAL dataset — single rosbag with uwb_driver::UwbRange +
  // sensor_msgs::Imu.  Anchor positions are auto-extracted from UWB messages
  // when viral_auto_anchors is true (default); otherwise they must come from
  // the config anchors list.
  bool LoadViralBag(const std::string& bag_path,
                    std::vector<ImuSample>* out_imu,
                    std::vector<UwbFrame>* out_uwb,
                    std::vector<AnchorConfig>* out_anchors);

  // Load ground truth poses (VICON PoseStamped) from rosbag.
  // Returns empty vector if topic not configured or not found.
  std::vector<NavState> LoadGroundTruth(const std::string& bag_path);

  // Load ground truth poses (nav_msgs::Odometry) from rosbag.
  // Falls back to this if PoseStamped-based GT is empty.
  std::vector<NavState> LoadGroundTruthOdom(const std::string& bag_path);

  // Load MCD pose_inW CSV: num,t,x,y,z,qx,qy,qz,qw.
  std::vector<NavState> LoadGroundTruthCsv(const std::string& csv_path);

  // Load VIUNet CSV dataset (not rosbag). Returns true on success.
  // Anchor positions are auto-extracted from the UWB CSV file.
  bool LoadViunetCsv(const std::string& data_dir,
                     std::vector<ImuSample>* out_imu,
                     std::vector<UwbFrame>* out_uwb,
                     std::vector<AnchorConfig>* out_anchors);

  // Load VIUNet ground truth from 3×4 affine matrix CSV.
  std::vector<NavState> LoadGroundTruthViunet(const std::string& csv_path);

  // Load MILUV CSV dataset (PX4 IMU + Decawave UWB + Mocap GT).
  bool LoadMiluvCsv(const std::string& data_dir,
                    std::vector<ImuSample>* out_imu,
                    std::vector<UwbFrame>* out_uwb,
                    std::vector<AnchorConfig>* out_anchors);

  // Load MILUV ground truth from mocap.csv.
  std::vector<NavState> LoadGroundTruthMiluv(const std::string& csv_path);

  // Load SFUISE ISAS-Walk dataset — rosbag with isas_msgs::RTLSStick +
  // sensor_msgs::Imu. Anchor positions are auto-extracted from the
  // /anchor_list topic.
  bool LoadSfuiseBag(const std::string& data_dir, int sequence,
                     std::vector<ImuSample>* out_imu,
                     std::vector<UwbFrame>* out_uwb,
                     std::vector<AnchorConfig>* out_anchors);

  // Load SFUISE ground truth from VIVE TransformStamped in rosbag.
  std::vector<NavState> LoadGroundTruthSfuise(const std::string& bag_path);

  const std::string& imu_topic() const { return imu_topic_; }
  const std::string& uwb_topic() const { return uwb_topic_; }
  const std::string& gt_topic() const { return gt_topic_; }

 private:
  std::string imu_topic_;
  std::string uwb_topic_;
  std::string gt_topic_;
  std::string gt_odom_topic_;
  Config cfg_;
};

}  // namespace uifgo
