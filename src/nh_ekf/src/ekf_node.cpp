#include <memory>
#include <cmath>
#include <cmath>
#include <Eigen/Dense>
#include <tf2/utils.h>
#include <rclcpp/logging.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"


class JackalEkfNode : public rclcpp::Node {
public:
    JackalEkfNode::JackalEkfNode() 
    : Node("Jackal_ekf_node"), 
    state_(Eigen::VectorXd::Zero(5)),
    P_(Eigen::MatrixXd::Identity(5,5)),
    Q_(Eigen::MatrixXd::Identity(5,5)),
    R_odom_(Eigen::MatrixXd::Identity(2,2)),
    R_imu_(Eigen::MatrixXd::Identity(1,1)),
    is_initialized_(false)
     {
        P_ *= 0.1;
        Q_ *= 0.01;
        R_odom_(0,0) *= 0.05;
        R_odom_(1,1) *= 0.05;
        R_imu_(0,0) *= 0.02;
        R_imu_(1,1) *= 0.02;
        // ROS subscriptions
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/j100_0000/platform/odom/filtered", 10,
            std::bind(&JackalEkfNode::odomCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/j100_0000/sensors/imu_0/data", 10,
            std::bind(&JackalEkfNode::imuCallback, this, std::placeholders::_1));
        state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/j100_0000/ekf/state", 10);
        RCLCPP_INFO(this->get_logger(), "Jackal EKF Node Initialized. Subscribed to odometry and IMU.");
    }
private:
    void predict(double dt);
    void updateOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    //================
    // Init
    //================
    void initializeFromOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void initializeFromImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    //================
    // ROS callbacks
    //================
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    //================
    //utils
    //================
    void publishState(const rclcpp::Time & stamp);
    double normalizeAngle(double angle);
    bool isValidTimestamp(const rclcpp::Time & stamp) const;
    //================
    //State Vec and Cov
    //================
    Eigen::VectorXd state_; // [x, y, theta, v, omega]
    Eigen::MatrixXd P_; // State covariance 5 x 5
    Eigen::MatrixXd Q_; // Process noise 5 x 5
    Eigen::MatrixXd R_odom_; // Measurement noise for odometry 2 x 2
    Eigen::MatrixXd R_imu_;  // Measurement noise for IMU 2 x 2
    //================
    // Filter status
    //================
    bool is_initialized_;
    rclcpp::Time last_filter_time_;
    //================
    // ROS interfaces
    //================
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;


};
void JackalEkfNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const rclcpp::Time stamp = msg->header.stamp; 
    if (!is_initialized_) {
        
        initializeFromOdom(msg);
        last_filter_time_ = stamp;
        publishState(stamp);
        return;
    }
    double dt = ( stamp - last_filter_time_).seconds();
    if (dt <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Non-positive dt in odom callback, skipping update.");
        return;
    }
    predict(dt);
    updateOdom(msg);
    last_filter_time_ = stamp;
    publishState(stamp);
}
void JackalEkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const rclcpp::Time stamp = msg->header.stamp; 
    if (!is_initialized_) {
        return;
    }
    double dt = ( stamp - last_filter_time_).seconds();
    if (dt <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Non-positive dt in IMU callback, skipping update.");
        return;
    }
    predict(dt);
    updateImu(msg);
    last_filter_time_ = stamp;
    publishState(stamp);






