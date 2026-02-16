#include <chrono>
#include <deque>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

struct StreamStats
{
  // last receive time in node time (sim time if enabled)
  rclcpp::Time last_rx_time{0, 0, RCL_ROS_TIME};

  // last header stamp (if the msg has header)
  std::optional<rclcpp::Time> last_stamp;

  // sliding window of receive times for hz estimation
  std::deque<rclcpp::Time> rx_times;

  // computed hz
  double hz{0.0};

  // msg frame id (if available)
  std::string frame_id;

  // misc info
  std::string extra;
};

class NhSystemMonitor : public rclcpp::Node
{
public:
  NhSystemMonitor()
  : Node("nh_system_monitor")
  {
    // ---- Parameters (topics)
    topic_clock_  = declare_parameter<std::string>("topics.clock",  "/clock");
    topic_odom_   = declare_parameter<std::string>("topics.odom",   "/j100_0000/platform/odom/filtered");
    topic_imu_    = declare_parameter<std::string>("topics.imu",    "/j100_0000/sensors/imu_0/data");
    topic_points_ = declare_parameter<std::string>("topics.points", "/j100_0000/sensors/lidar3d_0/points");
    topic_cmdvel_ = declare_parameter<std::string>("topics.cmd_vel","/j100_0000/cmd_vel");

    // ---- Parameters (frames)
    odom_frame_  = declare_parameter<std::string>("frames.odom_frame",  "odom");
    base_frame_  = declare_parameter<std::string>("frames.base_frame",  "base_link");
    lidar_frame_ = declare_parameter<std::string>("frames.lidar_frame", "lidar3d_0_link");
    imu_frame_   = declare_parameter<std::string>("frames.imu_frame",   "imu_0_link");

    // ---- Parameters (thresholds)
    stale_clock_s_  = declare_parameter<double>("thresholds.stale_seconds.clock",  1.0);
    stale_odom_s_   = declare_parameter<double>("thresholds.stale_seconds.odom",   1.0);
    stale_imu_s_    = declare_parameter<double>("thresholds.stale_seconds.imu",    1.0);
    stale_points_s_ = declare_parameter<double>("thresholds.stale_seconds.points", 2.0);
    stale_cmdvel_s_ = declare_parameter<double>("thresholds.stale_seconds.cmd_vel", 2.0);

    min_hz_odom_   = declare_parameter<double>("thresholds.min_hz.odom",   5.0);
    min_hz_imu_    = declare_parameter<double>("thresholds.min_hz.imu",   20.0);
    min_hz_points_ = declare_parameter<double>("thresholds.min_hz.points", 2.0);

    hz_window_s_ = declare_parameter<double>("rate_window_seconds", 5.0);

    // Use sim time by default (can be overridden by launch)
    if (!this->has_parameter("use_sim_time")) {
      this->declare_parameter<bool>("use_sim_time", true);
      }

    // ---- TF listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Publisher
    status_pub_ = create_publisher<std_msgs::msg::String>("/nh/status", 10);

    // ---- Subscriptions
    sub_clock_ = create_subscription<rosgraph_msgs::msg::Clock>(
      topic_clock_, rclcpp::QoS(10),
      [this](const rosgraph_msgs::msg::Clock::SharedPtr /*msg*/) {
        touch_stream(clock_stats_, /*has_header=*/false, std::nullopt, "", "");
      });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      topic_odom_, rclcpp::QoS(50),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::string extra;
        extra.reserve(128);
        extra = "child=" + msg->child_frame_id;
        touch_stream(odom_stats_, true, msg->header.stamp, msg->header.frame_id, extra);
      });

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      topic_imu_, rclcpp::QoS(100),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        touch_stream(imu_stats_, true, msg->header.stamp, msg->header.frame_id, "");
      });

    sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_points_, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::ostringstream ss;
        ss << "w=" << msg->width << " h=" << msg->height
           << " step=" << msg->point_step
           << " bytes=" << msg->data.size();
        touch_stream(points_stats_, true, msg->header.stamp, msg->header.frame_id, ss.str());
      });

    sub_cmdvel_ = create_subscription<geometry_msgs::msg::Twist>(
      topic_cmdvel_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr /*msg*/) {
        touch_stream(cmdvel_stats_, false, std::nullopt, "", "");
      });

    // ---- Timer: print + publish every 1s
    timer_ = create_wall_timer(1s, [this]() { this->tick(); });

    RCLCPP_INFO(get_logger(), "nh_system_monitor started.");
    RCLCPP_INFO(get_logger(), "Monitoring: odom=%s imu=%s points=%s cmd_vel=%s clock=%s",
                topic_odom_.c_str(), topic_imu_.c_str(), topic_points_.c_str(),
                topic_cmdvel_.c_str(), topic_clock_.c_str());
  }

private:
  void touch_stream(StreamStats & st,
                    bool has_header,
                    std::optional<builtin_interfaces::msg::Time> header_stamp,
                    const std::string & frame_id,
                    const std::string & extra)
  {
    const auto now = this->now();

    st.last_rx_time = now;
    st.rx_times.push_back(now);

    // prune rx times outside window
    const rclcpp::Duration window = rclcpp::Duration::from_seconds(hz_window_s_);
    while (!st.rx_times.empty() && (now - st.rx_times.front()) > window) {
      st.rx_times.pop_front();
    }

    if (st.rx_times.size() >= 2) {
      const double dt = (st.rx_times.back() - st.rx_times.front()).seconds();
      st.hz = (dt > 0.0) ? (static_cast<double>(st.rx_times.size() - 1) / dt) : 0.0;
    } else {
      st.hz = 0.0;
    }

    if (has_header && header_stamp.has_value()) {
      st.last_stamp = rclcpp::Time(header_stamp.value(), RCL_ROS_TIME);
      st.frame_id = frame_id;
    }

    st.extra = extra;
  }

  enum class Health { DEAD, WARN, OK };

  static const char* health_str(Health h)
  {
    switch (h) {
      case Health::OK: return "OK";
      case Health::WARN: return "WARN";
      case Health::DEAD: return "DEAD";
    }
    return "???";
  }

  Health stream_health(const StreamStats & st, double stale_s, double min_hz = 0.0) const
  {
    const auto now = this->now();

    if (st.last_rx_time.nanoseconds() == 0) {
      return Health::DEAD; // never received anything
    }

    const double age = (now - st.last_rx_time).seconds();
    if (age > stale_s) {
      return Health::DEAD;
    }

    if (min_hz > 0.0 && st.hz > 0.0 && st.hz < min_hz) {
      return Health::WARN;
    }

    if (min_hz > 0.0 && st.hz == 0.0) {
      return Health::WARN; // got only one message in window
    }

    return Health::OK;
  }

  std::pair<Health, std::string> check_tf(const std::string & target, const std::string & source)
  {
    try {
      (void)tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
      return {Health::OK, "lookup " + target + "<-" + source + " OK"};
    } catch (const tf2::TransformException & ex) {
      return {Health::DEAD, std::string("lookup ") + target + "<-" + source + " FAIL: " + ex.what()};
    }
  }

  void tick()
  {
    const auto h_clock  = stream_health(clock_stats_,  stale_clock_s_);
    const auto h_odom   = stream_health(odom_stats_,   stale_odom_s_,   min_hz_odom_);
    const auto h_imu    = stream_health(imu_stats_,    stale_imu_s_,    min_hz_imu_);
    const auto h_points = stream_health(points_stats_, stale_points_s_, min_hz_points_);
    const auto h_cmdvel = stream_health(cmdvel_stats_, stale_cmdvel_s_);

    auto [tf_odom_base_h, tf_odom_base_msg]   = check_tf(odom_frame_, base_frame_);
    auto [tf_base_lidar_h, tf_base_lidar_msg] = check_tf(base_frame_, lidar_frame_);
    auto [tf_base_imu_h, tf_base_imu_msg]     = check_tf(base_frame_, imu_frame_);

    Health h_tf =
      (tf_odom_base_h == Health::DEAD || tf_base_lidar_h == Health::DEAD || tf_base_imu_h == Health::DEAD) ? Health::DEAD :
      (tf_odom_base_h == Health::WARN || tf_base_lidar_h == Health::WARN || tf_base_imu_h == Health::WARN) ? Health::WARN :
      Health::OK;

    std::ostringstream out;
    out << "\n========== nh_system_monitor ==========\n";
    out << "TIME: " << std::fixed << std::setprecision(3) << this->now().seconds() << " (node time)\n";
    out << "CLOCK:  " << health_str(h_clock) << "\n";
    out << "TF:     " << health_str(h_tf) << "\n";
    out << "  - " << tf_odom_base_msg << "\n";
    out << "  - " << tf_base_lidar_msg << "\n";
    out << "  - " << tf_base_imu_msg << "\n";

    out << "ODOM:   " << health_str(h_odom)
        << " hz=" << std::setprecision(2) << odom_stats_.hz
        << " frame=" << odom_stats_.frame_id
        << " " << odom_stats_.extra << "\n";

    out << "IMU:    " << health_str(h_imu)
        << " hz=" << std::setprecision(2) << imu_stats_.hz
        << " frame=" << imu_stats_.frame_id << "\n";

    out << "LIDAR:  " << health_str(h_points)
        << " hz=" << std::setprecision(2) << points_stats_.hz
        << " frame=" << points_stats_.frame_id
        << " " << points_stats_.extra << "\n";

    out << "CMD_VEL:" << health_str(h_cmdvel)
        << " hz=" << std::setprecision(2) << cmdvel_stats_.hz << "\n";

    out << "======================================\n";

    RCLCPP_INFO(get_logger(), "%s", out.str().c_str());

    std_msgs::msg::String msg;
    msg.data = out.str();
    status_pub_->publish(msg);
  }

private:
  // topics
  std::string topic_clock_, topic_odom_, topic_imu_, topic_points_, topic_cmdvel_;
  // frames
  std::string odom_frame_, base_frame_, lidar_frame_, imu_frame_;

  // thresholds
  double stale_clock_s_{1.0};
  double stale_odom_s_{1.0};
  double stale_imu_s_{1.0};
  double stale_points_s_{2.0};
  double stale_cmdvel_s_{2.0};
  double min_hz_odom_{5.0};
  double min_hz_imu_{20.0};
  double min_hz_points_{2.0};
  double hz_window_s_{5.0};

  // stats
  StreamStats clock_stats_;
  StreamStats odom_stats_;
  StreamStats imu_stats_;
  StreamStats points_stats_;
  StreamStats cmdvel_stats_;

  // ROS
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_clock_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdvel_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NhSystemMonitor>());
  rclcpp::shutdown();
  return 0;
}
