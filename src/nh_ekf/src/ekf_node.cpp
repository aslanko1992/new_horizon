#include <memory>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>
#include <tf2/utils.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"


using namespace std::chrono_literals;

class JackalEkfNode : public rclcpp::Node {
public:
    JackalEkfNode() : Node("jackal_ekf_node") {
        // --- 1. EKF Matrix Initialization ---
        state_ = Eigen::VectorXd::Zero(5); // x, y, theta, v, omega
        P_ = Eigen::MatrixXd::Identity(5, 5) * 0.1;
        Q_ = Eigen::MatrixXd::Identity(5, 5) * 0.01;
        R_odom_ = Eigen::MatrixXd::Identity(2, 2) * 0.05;
        R_imu_ = Eigen::MatrixXd::Identity(1, 1) * 0.02;

        // --- 2. ROS Communication ---
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/j100_0000/platform/odom", 10, 
            std::bind(&JackalEkfNode::odom_callback, this, std::placeholders::_1));

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/j100_0000/sensors/imu_0/data", 10, 
            std::bind(&JackalEkfNode::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "EKF Node Started (Single File Mode)");
    }

private:

    // varaiable members

    rclcpp::Time last_update_time_;
    bool is_initialized_ = false;

    //--------------
    void predict(double dt) {
        // 1. Extract current state for readability
        double theta = state_(2);
        double v     = state_(3);
        double omega = state_(4);

        // 2. Update State Vector (Non-linear projection)
        state_(0) += v * std::cos(theta) * dt; // New X
        state_(1) += v * std::sin(theta) * dt; // New Y
        state_(2) += omega * dt;              // New Theta
        // state_(3) and state_(4) (v, omega) stay same in prediction

        // 3. Construct Jacobian Matrix F (5x5)
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
        F(0, 2) = -v * std::sin(theta) * dt;
        F(0, 3) = std::cos(theta) * dt;
        F(1, 2) =  v * std::cos(theta) * dt;
        F(1, 3) = std::sin(theta) * dt;
        F(2, 4) = dt;

        // 4. Update Covariance Matrix P
        // P = F * P * F_transpose + Q
        Eigen::MatrixXd Q_scaled = Q_ * dt;
        P_ = F * P_ * F.transpose() + Q_scaled;
    }
    void updateOdom(double v_meas, double omega_meas){
        //1.measurment vector:
        Eigen::Vector2d z(v_meas, omega_meas);
        //2.observation vector
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
        H(0, 3) = 1.0; //v
        H(1, 4) = 1.0; //w

        // 3. Innovation: y = z-H*x
        Eigen::Vector2d y = z - H*state_;

        // 4. Covariance
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_odom_;

        // 5. Klaman gain
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 6. update

        state_ = state_ + K * y;

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5 ,5);
        P_ = (I - K * H) * P_;



    }
    void updateImu(double omega_meas) {
        // 1. Observation matrix H (1x5) - we only extract state_(4)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 5);
        H(0, 4) = 1.0;

        // 2. Innovation: y = z - H*x (Scalar in this case)
        double z = omega_meas;
        double y = z - state_(4);

        // 3. Innovation Covariance: S = H*P*H' + R_imu
        // S is a 1x1 matrix (scalar)
        double S = P_(4, 4) + R_imu_(0, 0);

        // 4. Kalman Gain: K = P * H' * inv(S)
        // K is 5x1
        Eigen::VectorXd K = P_.col(4) / S;

        // 5. Update State and Covariance
        state_ += K * y;
        
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
        P_ = (I - K * H) * P_;
    }

    // Inside your imu_callback:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!is_initialized_) return;

        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_update_time_).seconds();

        if (dt > 0.0) {
            // 1. Predict to the exact moment of this IMU reading
            predict(dt);
            last_update_time_ = current_time;

            // 2. Extract IMU Yaw (Orientation)
            tf2::Quaternion q(
                msg->orientation.x, msg->orientation.y,
                msg->orientation.z, msg->orientation.w);
            double r, p, yaw_imu;
            tf2::Matrix3x3(q).getRPY(r, p, yaw_imu);

            // 3. Update with both Heading and Rate
            // H will be 2x5, z will be [yaw, omega]
            updateImu(yaw_imu, msg->angular_velocity.z);
        }
    }

    
    // --- ROS Callbacks ---
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        (void)msg;
        // Logic will go here
        rclcpp::Time curent_time = msg->header.stamp;
        if(!is_initialized_){
            
            state_(0) = msg->pose.pose.position.x;
            state_(1) = msg->pose.pose.position.y;
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); 

            // On initialization:
            state_(2) = yaw;
            state_(3) = msg->twist.twist.linear.x;
            state_(4) = msg->twist.twist.angular.z;

            is_initialized_ = true;
            last_update_time_ = curent_time;
            return;
        }
        double dt = (curent_time - last_update_time_).seconds();
        if (dt > 0) {

            predict(dt);
            last_update_time_ = curent_time;
            updateOdom(msg->twist.twist.linear.x, msg->twist.twist.angular.z);

        }


    }


    // --- EKF Members ---
    Eigen::VectorXd state_;
    Eigen::MatrixXd P_, Q_, R_odom_, R_imu_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    
    
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JackalEkfNode>());
    rclcpp::shutdown();
    return 0;
}