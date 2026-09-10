#include "uifgo/data_loader.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <isas_msgs/Anchorlist.h>
#include <isas_msgs/RTLSStick.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <uwb_driver/UwbRange.h>
#include <uwb_imu_fgo/LinktrackNode2.h>
#include <uwb_imu_fgo/LinktrackNodeframe3.h>
#include <uwb_imu_fgo/McdLinktrackNodeframe3.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>

namespace uifgo {

namespace {

template <typename Msg>
UwbFrame ConvertUwbFrame(const Msg& msg) {
  UwbFrame frame;
  frame.t = msg.header.stamp.toSec();
  frame.tag_id = msg.id;
  for (const auto& node : msg.nodes) {
    frame.ranges.push_back({node.id, node.dis, node.fp_rssi, node.rx_rssi});
  }
  return frame;
}

void PrintImuDiagnostic(const std::vector<ImuSample>& imu) {
  if (imu.empty()) return;
  const size_t n_diag = std::min(imu.size(), size_t(200));
  double acc_norm_sum = 0.0;
  for (size_t i = 0; i < n_diag; ++i) acc_norm_sum += imu[i].acc.norm();
  const double acc_norm_mean = acc_norm_sum / n_diag;
  std::cout << "[DIAG] IMU acc norm after scaling (first " << n_diag
            << " samples): mean = " << acc_norm_mean
            << " m/s^2  |  expected ~9.81 (static)\n";
  if (acc_norm_mean < 2.0) {
    std::cout << "[WARN] IMU acc norm still small after scaling -- "
                 "check imu.imu_acc_in_g.\n";
  }
}

void SetImuOrientation(const sensor_msgs::Imu& msg, ImuSample* sample) {
  if (msg.orientation_covariance[0] < 0.0) return;
  Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y,
                       msg.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-6) return;
  sample->orientation = q.normalized();
  sample->has_orientation = true;
}

bool InBagWindow(const rosbag::MessageInstance& message,
                 const ros::Time& begin, const ros::Time& end) {
  return message.getTime() >= begin && message.getTime() <= end;
}

void AttachSourceProvenance(UwbFrame* frame,
                            std::uint64_t source_message_index,
                            std::uint64_t* source_obs_index) {
  for (size_t range_index = 0; range_index < frame->ranges.size();
       ++range_index) {
    UwbRange& range = frame->ranges[range_index];
    range.source_obs_index = (*source_obs_index)++;
    range.source_message_index = source_message_index;
    range.source_range_index = range_index;
    range.source_time = frame->t;
    range.source_tag_id = frame->tag_id;
  }
}

}  // namespace

DataLoader::DataLoader(const Config& cfg)
    : imu_topic_(cfg.imu_topic),
      uwb_topic_(cfg.uwb_topic),
      gt_topic_(cfg.vicon_topic),
      gt_odom_topic_(cfg.gt_odom_topic),
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
  std::uint64_t source_obs_index = 0;
  std::uint64_t source_message_index = 0;
  rosbag::View source_view(bag, rosbag::TopicQuery(topics));

  for (const rosbag::MessageInstance& m : source_view) {
    std::string topic = m.getTopic();

    if (topic == imu_topic_) {
      if (!InBagWindow(m, time_start, time_finish)) continue;
      sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      if (!imu_msg) continue;

      ImuSample s;
      s.t = imu_msg->header.stamp.toSec();
      // Hardware IMU (e.g. Livox) publishes in g-units → scale to m/s².
      // Simulation IMU publishes directly in m/s² → skip scaling.
      const double g_scale = cfg_.imu_acc_in_g ? cfg_.gravity : 1.0;
      s.acc = Eigen::Vector3d(imu_msg->linear_acceleration.x * g_scale,
                              imu_msg->linear_acceleration.y * g_scale,
                              imu_msg->linear_acceleration.z * g_scale);
      s.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                               imu_msg->angular_velocity.y,
                               imu_msg->angular_velocity.z);
      SetImuOrientation(*imu_msg, &s);
      out_imu->push_back(s);

    } else if (topic == uwb_topic_) {
      const std::uint64_t message_index = source_message_index++;
      // Original project message. MessageInstance checks the ROS MD5, so MCD's
      // uint32 local_time variant is handled by the second definition below.
      auto uwb_msg = m.instantiate<uwb_imu_fgo::LinktrackNodeframe3>();
      UwbFrame frame;
      if (uwb_msg) {
        frame = ConvertUwbFrame(*uwb_msg);
      } else {
        auto mcd_msg = m.instantiate<uwb_imu_fgo::McdLinktrackNodeframe3>();
        if (!mcd_msg) continue;
        frame = ConvertUwbFrame(*mcd_msg);
      }

      AttachSourceProvenance(&frame, message_index, &source_obs_index);
      if (!InBagWindow(m, time_start, time_finish)) continue;

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

  PrintImuDiagnostic(*out_imu);

  return (!out_imu->empty() && !out_uwb->empty());
}

bool DataLoader::LoadFromBags(const std::string& imu_bag_path,
                              const std::string& uwb_bag_path,
                              std::vector<ImuSample>* out_imu,
                              std::vector<UwbFrame>* out_uwb) {
  if (imu_bag_path.empty() || uwb_bag_path.empty()) {
    std::cerr << "DataLoader: MCD IMU/UWB bag path is empty.\n";
    return false;
  }

  rosbag::Bag imu_bag, uwb_bag;
  try {
    imu_bag.open(imu_bag_path, rosbag::bagmode::Read);
    uwb_bag.open(uwb_bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    std::cerr << "DataLoader: failed to open split bags: " << e.what() << "\n";
    return false;
  }

  rosbag::View imu_full(imu_bag), uwb_full(uwb_bag);
  const ros::Time common_begin =
      std::max(imu_full.getBeginTime(), uwb_full.getBeginTime()) +
      ros::Duration(cfg_.bag_start);
  ros::Time common_end = std::min(imu_full.getEndTime(), uwb_full.getEndTime());
  if (cfg_.bag_durr >= 0.0) {
    common_end =
        std::min(common_end, common_begin + ros::Duration(cfg_.bag_durr));
  }
  if (common_end <= common_begin) {
    std::cerr << "DataLoader: split bags have no common time interval.\n";
    imu_bag.close();
    uwb_bag.close();
    return false;
  }

  out_imu->clear();
  out_uwb->clear();
  std::uint64_t source_obs_index = 0;
  rosbag::View imu_view(imu_bag, rosbag::TopicQuery({imu_topic_}), common_begin,
                        common_end);
  rosbag::View uwb_view(uwb_bag, rosbag::TopicQuery({uwb_topic_}), common_begin,
                        common_end);

  const double g_scale = cfg_.imu_acc_in_g ? cfg_.gravity : 1.0;
  for (const auto& m : imu_view) {
    auto msg = m.instantiate<sensor_msgs::Imu>();
    if (!msg) continue;
    ImuSample sample;
    sample.t = msg->header.stamp.toSec();
    sample.acc = Eigen::Vector3d(msg->linear_acceleration.x * g_scale,
                                 msg->linear_acceleration.y * g_scale,
                                 msg->linear_acceleration.z * g_scale);
    sample.gyro =
        Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                        msg->angular_velocity.z);
    SetImuOrientation(*msg, &sample);
    out_imu->push_back(sample);
  }

  for (const auto& m : uwb_view) {
    UwbFrame frame;
    auto original = m.instantiate<uwb_imu_fgo::LinktrackNodeframe3>();
    if (original) {
      frame = ConvertUwbFrame(*original);
    } else {
      auto mcd = m.instantiate<uwb_imu_fgo::McdLinktrackNodeframe3>();
      if (!mcd) {
        std::cerr << "DataLoader: unsupported UWB type/MD5 on " << uwb_topic_
                  << ": " << m.getDataType() << " / " << m.getMD5Sum() << "\n";
        continue;
      }
      frame = ConvertUwbFrame(*mcd);
    }
    for (auto& range : frame.ranges)
      range.source_obs_index = source_obs_index++;
    if (!frame.ranges.empty()) out_uwb->push_back(std::move(frame));
  }
  imu_bag.close();
  uwb_bag.close();

  std::sort(out_imu->begin(), out_imu->end(),
            [](const ImuSample& a, const ImuSample& b) { return a.t < b.t; });
  std::sort(out_uwb->begin(), out_uwb->end(),
            [](const UwbFrame& a, const UwbFrame& b) { return a.t < b.t; });
  std::cout << "DataLoader: loaded split bags: " << out_imu->size()
            << " IMU samples from " << imu_bag_path << ", " << out_uwb->size()
            << " UWB frames from " << uwb_bag_path << "\n";
  PrintImuDiagnostic(*out_imu);
  return !out_imu->empty() && !out_uwb->empty();
}

// =============================================================================
//  NTU VIRAL Dataset Loader
// =============================================================================

bool DataLoader::LoadViralBag(const std::string& bag_path,
                              std::vector<ImuSample>* out_imu,
                              std::vector<UwbFrame>* out_uwb,
                              std::vector<AnchorConfig>* out_anchors) {
  if (bag_path.empty()) {
    std::cerr << "DataLoader(VIRAL): empty bag path.\n";
    return false;
  }

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    std::cerr << "DataLoader(VIRAL): failed to open bag: " << e.what() << "\n";
    return false;
  }

  const std::string& imu_topic = cfg_.viral_imu_topic;
  const std::string& uwb_topic = cfg_.viral_uwb_topic;
  const std::string& gt_topic = cfg_.viral_gt_topic;

  // Build a set of requester/responder IDs for optional filtering.
  std::set<int> req_filter(cfg_.viral_requester_ids.begin(),
                           cfg_.viral_requester_ids.end());
  std::set<int> resp_filter(cfg_.viral_responder_ids.begin(),
                            cfg_.viral_responder_ids.end());
  const bool filter_req = !req_filter.empty();
  const bool filter_resp = !resp_filter.empty();

  std::vector<std::string> topics = {imu_topic, uwb_topic};
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
    std::cerr << "DataLoader(VIRAL): no messages on topics " << imu_topic
              << " / " << uwb_topic << "\n";
    bag.close();
    return false;
  }

  std::cout << "DataLoader(VIRAL): loading bag " << bag_path << " ("
            << view.size() << " messages)\n";

  out_imu->clear();
  out_uwb->clear();
  out_anchors->clear();

  // Discovered anchor positions: responder_id -> {pos, count for averaging}
  std::map<int, gtsam::Point3> anchor_pos;
  std::map<int, int> anchor_count;

  // Accumulated ranges within the current time-window group.
  UwbFrame pending_frame;
  bool have_pending = false;

  // IMU acc scaling: VN100 outputs m/s² natively.
  const double g_scale = cfg_.imu_acc_in_g ? cfg_.gravity : 1.0;

  for (const rosbag::MessageInstance& m : view) {
    std::string topic = m.getTopic();

    if (topic == imu_topic) {
      auto imu_msg = m.instantiate<sensor_msgs::Imu>();
      if (!imu_msg) continue;

      ImuSample s;
      s.t = imu_msg->header.stamp.toSec();
      s.acc = Eigen::Vector3d(imu_msg->linear_acceleration.x * g_scale,
                              imu_msg->linear_acceleration.y * g_scale,
                              imu_msg->linear_acceleration.z * g_scale);
      s.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                               imu_msg->angular_velocity.y,
                               imu_msg->angular_velocity.z);
      SetImuOrientation(*imu_msg, &s);
      out_imu->push_back(s);

    } else if (topic == uwb_topic) {
      auto uwb_msg = m.instantiate<uwb_driver::UwbRange>();
      if (!uwb_msg) continue;

      const int req_id = uwb_msg->requester_id;
      const int resp_id = uwb_msg->responder_id;

      // Optional ID filtering
      if (filter_req && req_filter.find(req_id) == req_filter.end()) continue;
      if (filter_resp && resp_filter.find(resp_id) == resp_filter.end())
        continue;

      // Auto-extract anchor position from responder_location.
      // The dataset convention: 99999 means unknown; otherwise it's valid.
      const auto& rloc = uwb_msg->responder_location;
      const bool known_pos =
          (std::abs(rloc.x) < 99998.0 || std::abs(rloc.y) < 99998.0 ||
           std::abs(rloc.z) < 99998.0);
      if (cfg_.viral_auto_anchors && known_pos) {
        gtsam::Point3 p(rloc.x, rloc.y, rloc.z);
        if (anchor_count.find(resp_id) == anchor_count.end()) {
          anchor_pos[resp_id] = p;
          anchor_count[resp_id] = 1;
        } else {
          // Running average
          int n = anchor_count[resp_id] + 1;
          anchor_pos[resp_id] =
              anchor_pos[resp_id] * (double(n - 1) / n) + p * (1.0 / n);
          anchor_count[resp_id] = n;
        }
      }

      // Build the single-range frame.
      UwbRange range;
      range.anchor_id = resp_id;
      range.dist = uwb_msg->distance;
      range.fp_rssi = 0.0;  // UwbRange does not carry RSSI in LinkTrack style
      range.rx_rssi = 0.0;

      double t_msg = uwb_msg->header.stamp.toSec();

      if (cfg_.viral_uwb_group_window <= 0.0) {
        // No grouping: every UwbRange becomes its own UwbFrame.
        UwbFrame frame;
        frame.t = t_msg;
        frame.tag_id = req_id;
        frame.ranges.push_back(range);
        out_uwb->push_back(frame);
      } else {
        // Group ranges within the time window.
        if (!have_pending) {
          pending_frame.t = t_msg;
          pending_frame.tag_id = req_id;
          pending_frame.ranges.clear();
          pending_frame.ranges.push_back(range);
          have_pending = true;
        } else if (t_msg - pending_frame.t <= cfg_.viral_uwb_group_window) {
          pending_frame.ranges.push_back(range);
        } else {
          out_uwb->push_back(pending_frame);
          pending_frame.t = t_msg;
          pending_frame.tag_id = req_id;
          pending_frame.ranges.clear();
          pending_frame.ranges.push_back(range);
        }
      }
    }
  }

  // Flush last pending frame.
  if (have_pending && !pending_frame.ranges.empty()) {
    out_uwb->push_back(pending_frame);
  }

  bag.close();

  // Sort by time.
  std::sort(out_imu->begin(), out_imu->end(),
            [](const ImuSample& a, const ImuSample& b) { return a.t < b.t; });
  std::sort(out_uwb->begin(), out_uwb->end(),
            [](const UwbFrame& a, const UwbFrame& b) { return a.t < b.t; });

  std::cout << "DataLoader(VIRAL): loaded " << out_imu->size()
            << " IMU samples, " << out_uwb->size() << " UWB frames.\n";
  PrintImuDiagnostic(*out_imu);

  // --- Populate auto-discovered anchors ---
  if (cfg_.viral_auto_anchors && !anchor_pos.empty()) {
    std::cout << "DataLoader(VIRAL): auto-discovered " << anchor_pos.size()
              << " anchors:\n";
    for (const auto& kv : anchor_pos) {
      AnchorConfig ac;
      ac.id = kv.first;
      ac.pos = kv.second;
      ac.prior_sigma = 0.10;  // default
      out_anchors->push_back(ac);
      std::cout << "  Anchor #" << kv.first << " @ [" << kv.second.x() << ", "
                << kv.second.y() << ", " << kv.second.z() << "]\n";
    }
  }

  return !out_imu->empty() && !out_uwb->empty();
}

// ===================================================================
// VIUNet CSV loader (no rosbag dependency)
// ===================================================================

bool DataLoader::LoadViunetCsv(const std::string& data_dir,
                               std::vector<ImuSample>* out_imu,
                               std::vector<UwbFrame>* out_uwb,
                               std::vector<AnchorConfig>* out_anchors) {
  if (data_dir.empty()) {
    std::cerr << "DataLoader(VIUNet): empty data dir.\n";
    return false;
  }

  std::string imu_path = data_dir + "/" + cfg_.viunet_imu_csv;
  std::string uwb_path = data_dir + "/" + cfg_.viunet_uwb_csv;

  // --- Parse UWB CSV first (to get anchor positions) ---
  out_uwb->clear();
  out_anchors->clear();
  {
    std::ifstream f(uwb_path);
    if (!f) {
      std::cerr << "DataLoader(VIUNet): cannot open " << uwb_path << "\n";
      return false;
    }
    std::string line;
    // Skip header
    std::getline(f, line);
    // UWB format: t, d1,d2,d3,d4, x1,z1,y1, x2,z2,y2, x3,z3,y3, x4,z4,y4
    // Note: anchor columns are in order (x, z, y), not (x, y, z).
    bool anchors_saved = false;
    while (std::getline(f, line)) {
      if (line.empty()) continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream ss(line);
      double t;
      std::vector<double> vals;
      vals.reserve(17);
      double v;
      while (ss >> v) vals.push_back(v);
      if (vals.size() < 17) continue;
      t = vals[0];

      UwbFrame frame;
      frame.t = t / 1.0e6;  // VIUNet timestamps are in microseconds
      frame.tag_id = 0;

      // 4 anchors: cols 1-4 = ranges, cols 5-16 = anchor positions (x,z,y per
      // anchor)
      for (int a = 0; a < 4; ++a) {
        double dist = vals[1 + a];
        double ax = vals[5 + a * 3 + 0];
        double az = vals[5 + a * 3 + 1];
        double ay = vals[5 + a * 3 + 2];
        // Swap z↔y if configured (VIUNet stores x,z,y column order)
        double anchor_x = ax;
        double anchor_y = cfg_.viunet_swap_uwb_yz ? ay : az;
        double anchor_z = cfg_.viunet_swap_uwb_yz ? az : ay;

        UwbRange r;
        r.anchor_id = a + 1;  // anchor IDs 1..4
        r.dist = dist;
        r.fp_rssi = 0.0;
        r.rx_rssi = 0.0;
        frame.ranges.push_back(r);

        // Save anchor positions once from the first row
        if (!anchors_saved) {
          AnchorConfig ac;
          ac.id = a + 1;
          ac.pos = gtsam::Point3(anchor_x, anchor_y, anchor_z);
          ac.prior_sigma = 0.10;
          // Avoid duplicates
          bool dup = false;
          for (const auto& ex : *out_anchors)
            if (ex.id == ac.id) {
              dup = true;
              break;
            }
          if (!dup) out_anchors->push_back(ac);
        }
      }
      anchors_saved = true;
      if (!frame.ranges.empty()) out_uwb->push_back(frame);
    }
    f.close();
  }

  // --- Parse IMU CSV ---
  out_imu->clear();
  {
    std::ifstream f(imu_path);
    if (!f) {
      std::cerr << "DataLoader(VIUNet): cannot open " << imu_path << "\n";
      return false;
    }
    std::string line;
    // IMU format: t(µs), ax, ay, az, gx, gy, gz  (m/s², rad/s)
    // D435i in VIUNet: y-axis is vertical (ay ≈ -9.81 when static).
    // Rotate to ENU (z-up) if configured.
    Eigen::Matrix3d R_yup_to_enu;
    if (cfg_.viunet_rotate_imu_yup) {
      // y-up → z-up: rotate about x-axis by -90°
      R_yup_to_enu << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
    } else {
      R_yup_to_enu = Eigen::Matrix3d::Identity();
    }

    while (std::getline(f, line)) {
      if (line.empty()) continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream ss(line);
      double t, ax, ay, az, gx, gy, gz;
      if (!(ss >> t >> ax >> ay >> az >> gx >> gy >> gz)) continue;

      ImuSample s;
      s.t = t / 1.0e6;  // µs → seconds
      Eigen::Vector3d acc_raw(ax, ay, az);
      Eigen::Vector3d gyro_raw(gx, gy, gz);
      s.acc = R_yup_to_enu * acc_raw;
      s.gyro = R_yup_to_enu * gyro_raw;
      out_imu->push_back(s);
    }
    f.close();
  }

  // Sort by time
  std::sort(out_imu->begin(), out_imu->end(),
            [](const ImuSample& a, const ImuSample& b) { return a.t < b.t; });
  std::sort(out_uwb->begin(), out_uwb->end(),
            [](const UwbFrame& a, const UwbFrame& b) { return a.t < b.t; });

  // Print anchor info
  std::cout << "DataLoader(VIUNet): loaded " << out_imu->size()
            << " IMU samples, " << out_uwb->size() << " UWB frames, "
            << out_anchors->size() << " anchors.\n";
  for (const auto& a : *out_anchors)
    std::cout << "  Anchor #" << a.id << " @ [" << a.pos.x() << ", "
              << a.pos.y() << ", " << a.pos.z() << "]\n";
  PrintImuDiagnostic(*out_imu);

  return !out_imu->empty() && !out_uwb->empty();
}

std::vector<NavState> DataLoader::LoadGroundTruthViunet(
    const std::string& csv_path) {
  std::vector<NavState> gt;
  std::ifstream f(csv_path);
  if (!f) {
    std::cerr << "DataLoader(VIUNet): cannot open GT " << csv_path << "\n";
    return gt;
  }

  // GT format: timestamp(µs), r11,r12,r13,tx, r21,r22,r23,ty, r31,r32,r33,tz
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    double t;
    double m[12];
    if (!(ss >> t)) continue;
    for (int i = 0; i < 12; ++i)
      if (!(ss >> m[i])) {
        t = -1;
        break;
      }
    if (t < 0) continue;

    // Build 3×4 affine matrix and extract rotation + translation
    Eigen::Matrix<double, 3, 4> T_mat;
    T_mat << m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10],
        m[11];
    Eigen::Matrix3d R_mat = T_mat.block<3, 3>(0, 0);
    Eigen::Vector3d p = T_mat.col(3);

    NavState s;
    s.t = t / 1.0e6;  // µs → seconds
    s.T = gtsam::Pose3(gtsam::Rot3(R_mat), gtsam::Point3(p));
    s.v = gtsam::Vector3::Zero();
    s.ba = gtsam::Vector3::Zero();
    s.bg = gtsam::Vector3::Zero();
    gt.push_back(s);
  }
  f.close();

  std::sort(gt.begin(), gt.end(),
            [](const NavState& a, const NavState& b) { return a.t < b.t; });
  std::cout << "DataLoader(VIUNet): loaded " << gt.size() << " GT poses from "
            << csv_path << "\n";
  return gt;
}

// ===================================================================
// MILUV CSV loader (PX4 IMU + Decawave UWB + Mocap GT)
// ===================================================================

bool DataLoader::LoadMiluvCsv(const std::string& data_dir,
                              std::vector<ImuSample>* out_imu,
                              std::vector<UwbFrame>* out_uwb,
                              std::vector<AnchorConfig>* out_anchors) {
  if (data_dir.empty()) {
    std::cerr << "DataLoader(MILUV): empty data dir.\n";
    return false;
  }
  std::string imu_path =
      data_dir + "/" + cfg_.miluv_robot_dir + "/" + cfg_.miluv_imu_csv;
  std::string uwb_path =
      data_dir + "/" + cfg_.miluv_robot_dir + "/" + cfg_.miluv_uwb_csv;

  out_imu->clear();
  out_uwb->clear();
  out_anchors->clear();

  // --- Parse IMU CSV (PX4: z-up, m/s², rad/s) ---
  {
    std::ifstream f(imu_path);
    if (!f) {
      std::cerr << "DataLoader(MILUV): cannot open " << imu_path << "\n";
      return false;
    }
    std::string line;
    std::getline(f, line);  // skip header
    while (std::getline(f, line)) {
      if (line.empty()) continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream ss(line);
      double t, wx, wy, wz, ax, ay, az;
      if (!(ss >> t >> wx >> wy >> wz >> ax >> ay >> az)) continue;
      ImuSample s;
      s.t = t;
      s.acc = Eigen::Vector3d(ax, ay, az);
      s.gyro = Eigen::Vector3d(wx, wy, wz);
      out_imu->push_back(s);
    }
    f.close();
  }

  // --- Parse UWB CSV (one range per row, group by timestamp) ---
  {
    std::ifstream f(uwb_path);
    if (!f) {
      std::cerr << "DataLoader(MILUV): cannot open " << uwb_path << "\n";
      return false;
    }
    std::string line;
    std::getline(f, line);  // skip header
    UwbFrame pending_frame;
    bool have_pending = false;

    while (std::getline(f, line)) {
      if (line.empty()) continue;
      // Parse: range, from_id, to_id, ...(29 columns)... , timestamp
      // Some columns contain bracket-format values that break istringstream >>,
      // so we split on commas and take the first 3 + last column.
      std::vector<std::string> cols;
      {
        std::istringstream css(line);
        std::string cell;
        while (std::getline(css, cell, ',')) cols.push_back(cell);
      }
      if (cols.size() < 5) continue;  // need at least range,from,to,...,ts
      double range = std::stod(cols[0]);
      int from_id = std::stoi(cols[1]);
      int to_id = std::stoi(cols[2]);
      double t = std::stod(cols.back());  // last column = timestamp
      if (t < 0.001) continue;

      UwbRange r;
      r.anchor_id = to_id;
      r.dist = range;
      r.fp_rssi = 0.0;
      r.rx_rssi = 0.0;

      if (cfg_.miluv_uwb_group_window <= 0.0) {
        UwbFrame frame;
        frame.t = t;
        frame.tag_id = from_id;
        frame.ranges.push_back(r);
        out_uwb->push_back(frame);
      } else {
        if (!have_pending) {
          pending_frame.t = t;
          pending_frame.tag_id = from_id;
          pending_frame.ranges.clear();
          pending_frame.ranges.push_back(r);
          have_pending = true;
        } else if (t - pending_frame.t <= cfg_.miluv_uwb_group_window) {
          pending_frame.ranges.push_back(r);
        } else {
          out_uwb->push_back(pending_frame);
          pending_frame.t = t;
          pending_frame.tag_id = from_id;
          pending_frame.ranges.clear();
          pending_frame.ranges.push_back(r);
        }
      }
    }
    if (have_pending && !pending_frame.ranges.empty())
      out_uwb->push_back(pending_frame);
    f.close();
  }

  // Sort
  std::sort(out_imu->begin(), out_imu->end(),
            [](const ImuSample& a, const ImuSample& b) { return a.t < b.t; });
  std::sort(out_uwb->begin(), out_uwb->end(),
            [](const UwbFrame& a, const UwbFrame& b) { return a.t < b.t; });

  std::cout << "DataLoader(MILUV): loaded " << out_imu->size()
            << " IMU samples, " << out_uwb->size() << " UWB frames.\n";
  PrintImuDiagnostic(*out_imu);
  return !out_imu->empty() && !out_uwb->empty();
}

std::vector<NavState> DataLoader::LoadGroundTruthMiluv(
    const std::string& csv_path) {
  std::vector<NavState> gt;
  std::ifstream f(csv_path);
  if (!f) {
    std::cerr << "DataLoader(MILUV): cannot open GT " << csv_path << "\n";
    return gt;
  }
  std::string line;
  std::getline(f, line);  // skip header
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    double t, px, py, pz, qx, qy, qz, qw;
    if (!(ss >> t >> px >> py >> pz >> qx >> qy >> qz >> qw)) continue;
    NavState s;
    s.t = t;
    s.T = gtsam::Pose3(gtsam::Rot3::Quaternion(qw, qx, qy, qz),
                       gtsam::Point3(px, py, pz));
    s.v = gtsam::Vector3::Zero();
    s.ba = gtsam::Vector3::Zero();
    s.bg = gtsam::Vector3::Zero();
    gt.push_back(s);
  }
  f.close();
  std::sort(gt.begin(), gt.end(),
            [](const NavState& a, const NavState& b) { return a.t < b.t; });
  std::cout << "DataLoader(MILUV): loaded " << gt.size() << " GT poses\n";
  return gt;
}

// ===================================================================
// SFUISE ISAS-Walk Dataset Loader (rosbag: isas_msgs + sensor_msgs)
// ===================================================================

bool DataLoader::LoadSfuiseBag(const std::string& data_dir, int sequence,
                               std::vector<ImuSample>* out_imu,
                               std::vector<UwbFrame>* out_uwb,
                               std::vector<AnchorConfig>* out_anchors,
                               bool preserve_invalid_ranges) {
  if (data_dir.empty()) {
    std::cerr << "DataLoader(SFUISE): empty data dir.\n";
    return false;
  }

  std::string bag_path =
      data_dir + "/ISAS-Walk" + std::to_string(sequence) + ".bag";
  if (!boost::filesystem::exists(bag_path)) {
    std::cerr << "DataLoader(SFUISE): bag not found: " << bag_path << "\n";
    return false;
  }

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    std::cerr << "DataLoader(SFUISE): failed to open bag: " << e.what() << "\n";
    return false;
  }

  const std::string& imu_topic = cfg_.sfuise_imu_topic;
  const std::string& uwb_topic = cfg_.sfuise_uwb_topic;
  const std::string& anchor_topic = cfg_.sfuise_anchor_topic;

  // --- First pass: discover anchor positions from /anchor_list ---
  std::map<int, gtsam::Point3> anchor_pos;
  std::map<int, int> anchor_count;
  int anchor_msgs_seen = 0;
  const int anchor_msg_limit = 20;  // same as SFUISE code
  {
    rosbag::View anchor_view(bag, rosbag::TopicQuery({anchor_topic}));
    for (const auto& m : anchor_view) {
      if (anchor_msgs_seen >= anchor_msg_limit) break;
      auto alist = m.instantiate<isas_msgs::Anchorlist>();
      if (!alist) continue;
      ++anchor_msgs_seen;
      for (const auto& a : alist->anchor) {
        gtsam::Point3 p(a.position.x, a.position.y, a.position.z);
        int aid = static_cast<int>(a.id);
        if (anchor_count.find(aid) == anchor_count.end()) {
          anchor_pos[aid] = p;
          anchor_count[aid] = 1;
        } else {
          int n = anchor_count[aid] + 1;
          anchor_pos[aid] =
              anchor_pos[aid] * (double(n - 1) / n) + p * (1.0 / n);
          anchor_count[aid] = n;
        }
      }
    }
  }

  if (anchor_pos.empty()) {
    std::cerr << "DataLoader(SFUISE): no anchors discovered on " << anchor_topic
              << "\n";
  } else {
    for (const auto& kv : anchor_pos) {
      AnchorConfig ac;
      ac.id = kv.first;
      ac.pos = kv.second;
      ac.prior_sigma = 0.10;
      out_anchors->push_back(ac);
    }
    std::cout << "DataLoader(SFUISE): auto-discovered " << anchor_pos.size()
              << " anchors (" << anchor_msgs_seen << " messages):\n";
    for (const auto& kv : anchor_pos)
      std::cout << "  Anchor #" << kv.first << " @ [" << kv.second.x() << ", "
                << kv.second.y() << ", " << kv.second.z() << "]\n";
  }

  // --- Second pass: load IMU + UWB ---
  std::vector<std::string> topics = {imu_topic, uwb_topic};
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
    std::cerr << "DataLoader(SFUISE): no messages on topics " << imu_topic
              << " / " << uwb_topic << "\n";
    bag.close();
    return false;
  }

  std::cout << "DataLoader(SFUISE): loading bag " << bag_path << " ("
            << view.size() << " messages)\n";

  out_imu->clear();
  out_uwb->clear();

  // SFUISE Waveshare Sense HAT: acc in m/s² (acc_ratio=false), gyro in rad/s
  const double g_scale = cfg_.imu_acc_in_g ? cfg_.gravity : 1.0;

  // Accumulated ranges within the current time-window group.
  UwbFrame pending_frame;
  bool have_pending = false;
  size_t uwb_msg_count = 0, uwb_range_count = 0, uwb_valid_count = 0;
  std::uint64_t source_obs_index = 0;
  std::uint64_t source_message_index = 0;
  rosbag::View source_view(bag, rosbag::TopicQuery(topics));

  for (const rosbag::MessageInstance& m : source_view) {
    std::string topic = m.getTopic();

    if (topic == imu_topic) {
      if (!InBagWindow(m, time_start, time_finish)) continue;
      auto imu_msg = m.instantiate<sensor_msgs::Imu>();
      if (!imu_msg) continue;

      ImuSample s;
      s.t = imu_msg->header.stamp.toSec();
      s.acc = Eigen::Vector3d(imu_msg->linear_acceleration.x * g_scale,
                              imu_msg->linear_acceleration.y * g_scale,
                              imu_msg->linear_acceleration.z * g_scale);
      s.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                               imu_msg->angular_velocity.y,
                               imu_msg->angular_velocity.z);
      SetImuOrientation(*imu_msg, &s);
      out_imu->push_back(s);

    } else if (topic == uwb_topic) {
      const std::uint64_t message_index = source_message_index++;
      auto stick = m.instantiate<isas_msgs::RTLSStick>();
      if (!stick) continue;
      const bool selected = InBagWindow(m, time_start, time_finish);
      const double t_msg = stick->header.stamp.toSec();
      const int tag_id = static_cast<int>(stick->id);
      if (selected) ++uwb_msg_count;

      // Capture source ordinals before grouping. The paper path preserves
      // protocol-invalid entries in its ledger; legacy callers retain the
      // historical behavior through preserve_invalid_ranges=false.
      std::vector<UwbRange> decoded_ranges;
      for (size_t range_index = 0; range_index < stick->ranges.size();
           ++range_index) {
        const auto& rg = stick->ranges[range_index];
        UwbRange r;
        r.anchor_id = static_cast<int>(rg.id);
        r.dist = rg.range;
        r.fp_rssi = rg.fpp;  // first path power
        r.rx_rssi = rg.rxp;  // received power
        r.source_obs_index = source_obs_index++;
        r.source_message_index = message_index;
        r.source_range_index = range_index;
        r.source_time = t_msg;
        r.source_tag_id = tag_id;
        r.source_valid = rg.ra != 0;
        if (!r.source_valid)
          r.source_validity_reason = "PROTOCOL_INVALID_RA_ZERO";
        if (selected && (r.source_valid || preserve_invalid_ranges))
          decoded_ranges.push_back(std::move(r));
        if (selected && rg.ra != 0) ++uwb_valid_count;
      }
      if (selected) uwb_range_count += stick->ranges.size();

      if (!selected) continue;
      if (decoded_ranges.empty()) continue;

      if (cfg_.sfuise_uwb_group_window <= 0.0) {
        // No grouping: every RTLSStick becomes its own UwbFrame.
        UwbFrame frame;
        frame.t = t_msg;
        frame.tag_id = tag_id;
        frame.ranges = std::move(decoded_ranges);
        out_uwb->push_back(frame);
      } else {
        // Group ranges within the time window.
        if (!have_pending) {
          pending_frame.t = t_msg;
          pending_frame.tag_id = tag_id;
          pending_frame.ranges = decoded_ranges;
          have_pending = true;
        } else if (t_msg - pending_frame.t <= cfg_.sfuise_uwb_group_window) {
          pending_frame.ranges.insert(pending_frame.ranges.end(),
                                      decoded_ranges.begin(),
                                      decoded_ranges.end());
        } else {
          out_uwb->push_back(pending_frame);
          pending_frame.t = t_msg;
          pending_frame.tag_id = tag_id;
          pending_frame.ranges = std::move(decoded_ranges);
        }
      }
    }
  }

  // Flush last pending frame.
  if (have_pending && !pending_frame.ranges.empty()) {
    out_uwb->push_back(pending_frame);
  }

  bag.close();

  // Sort by time.
  std::sort(out_imu->begin(), out_imu->end(),
            [](const ImuSample& a, const ImuSample& b) { return a.t < b.t; });
  std::sort(out_uwb->begin(), out_uwb->end(),
            [](const UwbFrame& a, const UwbFrame& b) { return a.t < b.t; });

  std::cout << "DataLoader(SFUISE): loaded " << out_imu->size()
            << " IMU samples, " << out_uwb->size() << " UWB frames ("
            << uwb_msg_count << " RTLSStick msgs, " << uwb_valid_count << "/"
            << uwb_range_count << " valid ranges).\n";
  PrintImuDiagnostic(*out_imu);

  return !out_imu->empty() && !out_uwb->empty();
}

std::vector<NavState> DataLoader::LoadGroundTruthSfuise(
    const std::string& bag_path) {
  std::vector<NavState> gt;
  const std::string& gt_topic = cfg_.sfuise_gt_topic;
  if (gt_topic.empty()) {
    std::cout << "LoadGroundTruth(SFUISE): no GT topic configured.\n";
    return gt;
  }

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    std::cerr << "LoadGroundTruth(SFUISE): cannot open bag: " << e.what()
              << "\n";
    return gt;
  }

  rosbag::View view;
  view.addQuery(bag, rosbag::TopicQuery({gt_topic}));
  if (view.size() == 0) {
    std::cerr << "LoadGroundTruth(SFUISE): no messages on " << gt_topic << "\n";
    bag.close();
    return gt;
  }

  for (const auto& m : view) {
    auto tf_msg = m.instantiate<geometry_msgs::TransformStamped>();
    if (!tf_msg) continue;
    NavState s;
    s.t = tf_msg->header.stamp.toSec();
    auto& q = tf_msg->transform.rotation;
    auto& p = tf_msg->transform.translation;
    s.T = gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                       gtsam::Point3(p.x, p.y, p.z));
    s.v = gtsam::Vector3::Zero();
    s.ba = gtsam::Vector3::Zero();
    s.bg = gtsam::Vector3::Zero();
    gt.push_back(s);
  }
  bag.close();
  std::cout << "LoadGroundTruth(SFUISE): " << gt.size() << " GT poses from "
            << gt_topic << "\n";
  return gt;
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

std::vector<NavState> DataLoader::LoadGroundTruthOdom(
    const std::string& bag_path) {
  std::vector<NavState> gt;
  if (gt_odom_topic_.empty()) {
    return gt;
  }

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    std::cerr << "LoadGroundTruthOdom: cannot open bag: " << e.what() << "\n";
    return gt;
  }

  rosbag::View view;
  view.addQuery(bag, rosbag::TopicQuery({gt_odom_topic_}));
  if (view.size() == 0) {
    std::cerr << "LoadGroundTruthOdom: no messages on " << gt_odom_topic_
              << "\n";
    bag.close();
    return gt;
  }

  for (const auto& m : view) {
    auto odom_msg = m.instantiate<nav_msgs::Odometry>();
    if (!odom_msg) continue;
    NavState s;
    s.t = odom_msg->header.stamp.toSec();
    auto& q = odom_msg->pose.pose.orientation;
    auto& p = odom_msg->pose.pose.position;
    s.T = gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                       gtsam::Point3(p.x, p.y, p.z));
    // Odometry may carry velocity
    s.v = gtsam::Vector3(odom_msg->twist.twist.linear.x,
                         odom_msg->twist.twist.linear.y,
                         odom_msg->twist.twist.linear.z);
    s.ba = gtsam::Vector3::Zero();
    s.bg = gtsam::Vector3::Zero();
    gt.push_back(s);
  }
  bag.close();
  std::cout << "LoadGroundTruthOdom: " << gt.size() << " poses from "
            << gt_odom_topic_ << "\n";
  return gt;
}

std::vector<NavState> DataLoader::LoadGroundTruthCsv(
    const std::string& csv_path) {
  std::vector<NavState> gt;
  std::ifstream input(csv_path);
  if (!input) {
    std::cerr << "LoadGroundTruthCsv: cannot open " << csv_path << "\n";
    return gt;
  }

  std::string line;
  std::getline(input, line);  // num,t,x,y,z,qx,qy,qz,qw
  size_t line_no = 1;
  while (std::getline(input, line)) {
    ++line_no;
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    long long index;
    double t, x, y, z, qx, qy, qz, qw;
    if (!(ss >> index >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      std::cerr << "LoadGroundTruthCsv: malformed line " << line_no << "\n";
      continue;
    }
    const double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!std::isfinite(t) || qnorm < 1e-9) continue;
    NavState s;
    s.t = t;
    s.T = gtsam::Pose3(
        gtsam::Rot3::Quaternion(qw / qnorm, qx / qnorm, qy / qnorm, qz / qnorm),
        gtsam::Point3(x, y, z));
    s.v = gtsam::Vector3::Zero();
    s.ba = gtsam::Vector3::Zero();
    s.bg = gtsam::Vector3::Zero();
    gt.push_back(s);
  }
  std::sort(gt.begin(), gt.end(),
            [](const NavState& a, const NavState& b) { return a.t < b.t; });
  std::cout << "LoadGroundTruthCsv: " << gt.size() << " poses from " << csv_path
            << "\n";
  return gt;
}

}  // namespace uifgo
