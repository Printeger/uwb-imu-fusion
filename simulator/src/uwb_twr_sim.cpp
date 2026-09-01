/**
 * uwb_twr_sim — UWB TWR Simulation Node (design doc: doc/uwb_sim.tex)
 *
 * Simulates Nooploop LinkTrack TWR ranging between a drone (tag) and N
 * fixed anchors. Implements TDMA scheduling, realistic error models
 * (Gaussian noise, per-link bias, NLOS positive bias, clock drift),
 * and log-distance RSSI generation.
 *
 * Subscribes: /sim/odom      (drone ground-truth from quadrotor_simulator_so3)
 * Publishes:  /nlink_linktrack_nodeframe3  (LinktrackNodeframe3, matches
 * hardware)
 */

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <uwb_imu_fgo/LinktrackNode2.h>
#include <uwb_imu_fgo/LinktrackNodeframe3.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

class UwbTwrSim {
 public:
  UwbTwrSim(ros::NodeHandle& nh) : nh_(nh), rng_(0) {
    nh_.param("num_anchors", N_, 4);
    nh_.param("superframe_period", T_sf_, 0.10);
    nh_.param("pub_rate", pub_rate_, 20.0);  // output rate [Hz]
    nh_.param("ranging_mode", mode_str_, std::string("DS"));
    nh_.param("range_noise_std", sigma_, 0.06);
    nh_.param("range_bias_const", b0_, 0.02);
    nh_.param("per_link_bias_max", bmax_, 0.05);
    nh_.param("clock_ppm", ppm_, 20.0);
    nh_.param("max_range", max_range_, 80.0);
    nh_.param("packet_loss_prob", ploss_, 0.02);
    nh_.param("odom_timeout", odom_to_, 0.20);
    nh_.param("nlos_probability", nlos_p_, 0.05);
    nh_.param("nlos_bias_mean", nlos_mean_, 0.40);
    nh_.param("rssi_tx_power", P_tx_, -45.0);
    nh_.param("rssi_path_loss_n", n_p_, 2.2);
    nh_.param("rssi_ref_dist", d0_, 1.0);
    nh_.param("rssi_shadowing_std", s_sh_, 2.0);
    nh_.param("fp_offset_los", fp_lo_, 3.0);
    nh_.param("fp_offset_nlos", fp_nl_, 8.0);
    nh_.param("battery_voltage", batt_v_, 4.10f);
    nh_.param("fault_mode", fault_mode_, std::string("none"));
    nh_.param("fault_anchor_id", fault_anchor_id_, -1);
    nh_.param("fault_onset_s", fault_onset_s_, 10.0);
    nh_.param("fault_duration_s", fault_duration_s_, -1.0);
    nh_.param("fault_magnitude_m", fault_magnitude_m_, 1.0);
    nh_.param("fault_ramp_rate_mps", fault_ramp_rate_mps_, 0.05);
    nh_.param("fault_sweep_min_m", fault_sweep_min_m_, 0.0);
    nh_.param("fault_sweep_max_m", fault_sweep_max_m_, 2.0);
    nh_.param("fault_sweep_period_s", fault_sweep_period_s_, 20.0);
    if (fault_mode_ != "none" && fault_mode_ != "step" &&
        fault_mode_ != "ramp" && fault_mode_ != "magnitude_sweep" &&
        fault_mode_ != "outage") {
      throw std::runtime_error("unsupported fault_mode: " + fault_mode_);
    }
    if (fault_sweep_period_s_ <= 0.0 || fault_duration_s_ == 0.0) {
      throw std::runtime_error(
          "fault_sweep_period_s must be positive and fault_duration_s nonzero");
    }
    int seed;
    nh_.param("random_seed", seed, -1);
    if (seed < 0) {
      throw std::runtime_error("random_seed must be explicitly configured");
    }
    rng_.seed(static_cast<std::mt19937::result_type>(seed));
    sim_start_ = ros::Time::now();

    // Load anchor positions from ROS param
    XmlRpc::XmlRpcValue anc;
    if (nh_.getParam("anchors", anc) &&
        anc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < anc.size(); ++i) {
        Anchor a;
        a.id = anc[i]["id"];
        a.pos =
            Eigen::Vector3d((double)anc[i]["pos"][0], (double)anc[i]["pos"][1],
                            (double)anc[i]["pos"][2]);
        anchors_.push_back(a);
        ROS_INFO("  Loaded anchor %d @ [%.2f, %.2f, %.2f]", a.id, a.pos.x(),
                 a.pos.y(), a.pos.z());
      }
      N_ = anchors_.size();
    }
    if (anchors_.empty()) {
      ROS_WARN("No anchors loaded from param, generating %d default anchors.",
               N_);
      double R = 5.0;
      for (int i = 0; i < N_; ++i) {
        Anchor a;
        a.id = i + 1;
        a.pos = Eigen::Vector3d(R * cos(2 * M_PI * i / N_),
                                R * sin(2 * M_PI * i / N_), 1.8);
        anchors_.push_back(a);
      }
    }
    if (fault_mode_ != "none" &&
        std::none_of(anchors_.begin(), anchors_.end(), [this](const Anchor& a) {
          return a.id == fault_anchor_id_;
        })) {
      throw std::runtime_error("fault_anchor_id is absent from the anchor map");
    }
    ROS_INFO("UwbTwrSim: total anchors = %d", (int)anchors_.size());

    // Auto-align superframe to pub rate: T_sf = 1/pub_rate ensures every
    // publish captures a complete TDMA cycle with all N anchors ranged.
    double T_sf_user = T_sf_;
    if (pub_rate_ > 0) {
      T_sf_ = 1.0 / pub_rate_;  // one full superframe per publish interval
    }
    tau_ = T_sf_ / N_;
    int m = (mode_str_ == "DS") ? 3 : 2;
    double t_msg;
    nh_.param("msg_air_time", t_msg, 0.0015);
    kmax_ = std::max(1, (int)(tau_ / (m * t_msg)));
    c_ss_ = 2.0;
    c_ds_ = 0.05;

    // Per-link biases
    blink_.resize(N_, std::vector<double>(N_, 0));
    std::uniform_real_distribution<double> ub(-bmax_, bmax_);
    for (int i = 0; i < N_; ++i)
      for (int j = 0; j < N_; ++j)
        if (i != j) blink_[i][j] = ub(rng_);

    clk_.resize(N_, 0);
    nlos_exp_ = std::exponential_distribution<double>(1.0 / nlos_mean_);

    std::string odom_topic;
    nh_.param("odom_topic", odom_topic, std::string("/sim/odom"));
    odom_sub_ = nh_.subscribe(odom_topic, 10, &UwbTwrSim::odomCb, this);
    pub_ = nh_.advertise<uwb_imu_fgo::LinktrackNodeframe3>(
        "/nlink_linktrack_nodeframe3", 10);
    viz_anchor_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/uwb_sim/anchor_markers", 100, true);  // high queue, latched
    viz_line_pub_ =
        nh_.advertise<visualization_msgs::Marker>("/uwb_sim/range_lines", 100);
    timer_ = nh_.createTimer(ros::Duration(tau_), &UwbTwrSim::onSlot, this);
    pub_timer_ = nh_.createTimer(ros::Duration(1.0 / pub_rate_),
                                 &UwbTwrSim::onPublish, this);
    viz_timer_ = nh_.createTimer(ros::Duration(0.1), &UwbTwrSim::onViz, this);

    publishAnchorMarkers();
    double slots_per_pub = (1.0 / pub_rate_) / tau_;
    int est_anchors_per_msg = (int)(slots_per_pub * kmax_);
    ROS_INFO(
        "UwbTwrSim: %d anchors, T_sf=%.3fs (auto-align to pub_rate=%.1f Hz), "
        "tau=%.4fs, kmax=%d → ~%d ranges/msg",
        N_, T_sf_, pub_rate_, tau_, kmax_, est_anchors_per_msg);
  }

 private:
  struct Anchor {
    int id;
    Eigen::Vector3d pos;
  };
  struct Drone {
    Eigen::Vector3d p;
    ros::Time t;
    bool ok = false;
  };

  double injectedFault(int anchor_id, const ros::Time& now,
                       bool* outage) const {
    *outage = false;
    if (fault_mode_ == "none" || anchor_id != fault_anchor_id_) return 0.0;
    const double elapsed = (now - sim_start_).toSec();
    const double active_time = elapsed - fault_onset_s_;
    if (active_time < 0.0 ||
        (fault_duration_s_ >= 0.0 && active_time > fault_duration_s_)) {
      return 0.0;
    }
    if (fault_mode_ == "outage") {
      *outage = true;
      return 0.0;
    }
    if (fault_mode_ == "step") return fault_magnitude_m_;
    if (fault_mode_ == "ramp") {
      return fault_magnitude_m_ + fault_ramp_rate_mps_ * active_time;
    }
    if (fault_mode_ == "magnitude_sweep") {
      const double phase = std::fmod(active_time, fault_sweep_period_s_) /
          fault_sweep_period_s_;
      return fault_sweep_min_m_ +
          phase * (fault_sweep_max_m_ - fault_sweep_min_m_);
    }
    return 0.0;
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& m) {
    drone_.p = {m->pose.pose.position.x, m->pose.pose.position.y,
                m->pose.pose.position.z};
    drone_.t = m->header.stamp;
    drone_.ok = true;
  }

  void onSlot(const ros::TimerEvent&) {
    int i = slot_;
    ros::Time now = ros::Time::now();
    clk_[i] += ppm_ * 1e-6 * tau_;
    if (!drone_.ok || (now - drone_.t).toSec() > odom_to_) {
      slot_ = (slot_ + 1) % N_;
      return;
    }

    int cnt = 0;
    for (int off = 0; off < N_ && cnt < kmax_; ++off) {
      int j = (i + off) % N_;
      bool forced_outage = false;
      const double injected_bias =
          injectedFault(anchors_[j].id, now, &forced_outage);
      if (forced_outage) continue;
      double d = (drone_.p - anchors_[j].pos).norm();
      if (d > max_range_) continue;
      if (uni_(rng_) < ploss_) continue;

      bool nlos = uni_(rng_) < nlos_p_;
      last_ranges_[anchors_[j].id] = {anchors_[j].pos, d, nlos};
      double eps = (mode_str_ == "DS") ? (clk_[i] - clk_[j]) * c_ds_
                                       : (clk_[i] - clk_[j]) * c_ss_;
      double dd = d * (1.0 + eps) + b0_ + blink_[i][j];
      if (nlos) dd += nlos_exp_(rng_);
      dd += injected_bias;
      dd += gauss_(rng_) * sigma_;

      uwb_imu_fgo::LinktrackNode2 nd;
      nd.role = 1;
      nd.id = anchors_[j].id;
      nd.dis = (float)std::max(0.0, dd);
      double dist = std::max(d, 0.1);
      nd.rx_rssi = (float)(P_tx_ - 10 * n_p_ * log10(dist / d0_) +
                           shadow_(rng_) * s_sh_);
      nd.fp_rssi = nd.rx_rssi - (float)(nlos ? (fp_lo_ + fp_nl_) : fp_lo_);
      pending_map_[anchors_[j].id] = nd;  // dedup: keep latest per anchor
      ++cnt;
    }
    slot_ = (slot_ + 1) % N_;
  }

  void onPublish(const ros::TimerEvent&) {
    if (pending_map_.empty()) return;

    uwb_imu_fgo::LinktrackNodeframe3 f;
    ros::Time now = ros::Time::now();
    f.header.stamp = now;
    f.id = tag_id_;
    f.role = 0;
    f.local_time = (uint64_t)(now.toSec() * 1e6);
    f.system_time = (uint32_t)(now.toSec() * 1e3);
    f.voltage = batt_v_;

    for (auto& kv : pending_map_) {
      f.nodes.push_back(kv.second);
    }
    pub_.publish(f);
    pending_map_.clear();
  }

  void publishAnchorMarkers() {
    for (const auto& a : anchors_) {
      visualization_msgs::Marker m;
      m.header.frame_id = "world";
      m.header.stamp = ros::Time::now();
      m.ns = "uwb_anchors";
      m.id = a.id;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = m.scale.y = m.scale.z = 0.2;
      m.color.r = 1.0;
      m.color.g = 0.5;
      m.color.b = 0.0;
      m.color.a = 1.0;
      m.pose.position.x = a.pos.x();
      m.pose.position.y = a.pos.y();
      m.pose.position.z = a.pos.z();
      m.pose.orientation.w = 1.0;
      viz_anchor_pub_.publish(m);

      visualization_msgs::Marker label;
      label.header.frame_id = "world";
      label.header.stamp = ros::Time::now();
      label.ns = "uwb_anchor_labels";
      label.id = a.id;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.scale.z = 0.4;
      label.color.r = 1.0;
      label.color.g = 1.0;
      label.color.b = 1.0;
      label.color.a = 1.0;
      label.pose.position.x = a.pos.x();
      label.pose.position.y = a.pos.y();
      label.pose.position.z = a.pos.z() + 0.5;
      label.pose.orientation.w = 1.0;
      label.text = "A" + std::to_string(a.id);
      viz_anchor_pub_.publish(label);
    }
    ROS_INFO_THROTTLE(5, "Published %d anchor markers", (int)anchors_.size());
  }

  void onViz(const ros::TimerEvent&) {
    publishAnchorMarkers();  // re-publish periodically for late-joining RViz
    if (drone_.ok) publishRangeLines();
  }

  void publishRangeLines() {
    visualization_msgs::Marker lines;
    lines.header.frame_id = "world";
    lines.header.stamp = ros::Time::now();
    lines.ns = "uwb_range_lines";
    lines.id = 0;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.scale.x = 0.015;
    lines.pose.orientation.w = 1.0;

    for (const auto& kv : last_ranges_) {
      const auto& rd = kv.second;
      geometry_msgs::Point drone_pt, anchor_pt;
      drone_pt.x = drone_.p.x();
      drone_pt.y = drone_.p.y();
      drone_pt.z = drone_.p.z();
      anchor_pt.x = rd.anchor_pos.x();
      anchor_pt.y = rd.anchor_pos.y();
      anchor_pt.z = rd.anchor_pos.z();
      lines.points.push_back(drone_pt);
      lines.points.push_back(anchor_pt);

      // Cyan lines for LOS, faded gray for NLOS
      std_msgs::ColorRGBA c;
      c.a = 0.5;
      if (rd.nlos) {
        c.r = 0.5;
        c.g = 0.5;
        c.b = 0.5;
      } else {
        c.r = 0.0;
        c.g = 0.8;
        c.b = 1.0;
      }
      lines.colors.push_back(c);
      lines.colors.push_back(c);
    }
    viz_line_pub_.publish(lines);
  }

  ros::NodeHandle nh_;
  int N_, slot_ = 0, kmax_, tag_id_ = 1;
  double T_sf_, tau_, pub_rate_, sigma_, b0_, bmax_, ppm_, max_range_, ploss_,
      odom_to_, nlos_p_, nlos_mean_, P_tx_, n_p_, d0_, s_sh_, fp_lo_, fp_nl_,
      c_ss_, c_ds_;
  std::string mode_str_;
  std::string fault_mode_;
  int fault_anchor_id_;
  double fault_onset_s_, fault_duration_s_, fault_magnitude_m_,
      fault_ramp_rate_mps_, fault_sweep_min_m_, fault_sweep_max_m_,
      fault_sweep_period_s_;
  ros::Time sim_start_;
  float batt_v_;
  Drone drone_;
  std::vector<Anchor> anchors_;
  std::vector<double> clk_;
  std::vector<std::vector<double>> blink_;
  ros::Subscriber odom_sub_;
  ros::Publisher pub_;
  ros::Publisher viz_anchor_pub_, viz_line_pub_;
  ros::Timer timer_, pub_timer_, viz_timer_;

  // Cached range data for visualization
  struct RangeData {
    Eigen::Vector3d anchor_pos;
    double dist;
    bool nlos;
  };
  std::map<int, RangeData> last_ranges_;
  // Accumulate latest range per anchor between publish ticks (dedup by anchor
  // ID)
  std::map<int, uwb_imu_fgo::LinktrackNode2> pending_map_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> uni_{0, 1};
  std::normal_distribution<double> gauss_{0, 1}, shadow_{0, 1};
  std::exponential_distribution<double> nlos_exp_{1.0 / 0.40};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "uwb_twr_sim");
  ros::NodeHandle nh("~");
  UwbTwrSim sim(nh);
  ros::spin();
  return 0;
}
