#pragma once

#include "uifgo/types.h"
#include "uifgo/config.h"
#include <vector>

namespace uifgo {

class DataLoader {
 public:
  explicit DataLoader(const Config& cfg);

  // Load IMU and UWB from rosbag.  Returns true on success.
  // out_imu and out_uwb are filled in time-ascending order.
  bool LoadFromBag(const std::string& bag_path,
                   std::vector<ImuSample>* out_imu,
                   std::vector<UwbFrame>* out_uwb);

  // Load ground truth poses (VICON PoseStamped) from rosbag.
  // Returns empty vector if topic not configured or not found.
  std::vector<NavState> LoadGroundTruth(const std::string& bag_path);

  const std::string& imu_topic()  const { return imu_topic_; }
  const std::string& uwb_topic()  const { return uwb_topic_; }
  const std::string& gt_topic()   const { return gt_topic_; }

 private:
  std::string imu_topic_;
  std::string uwb_topic_;
  std::string gt_topic_;
  Config cfg_;
};

}  // namespace uifgo
