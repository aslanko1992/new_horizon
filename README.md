--

# new_horizon

## Overview

`new_horizon` is a ROS 2 robotics integration project focused on building a clean, reproducible perception and state-estimation pipeline on top of a simulated mobile robot (Clearpath + Gazebo).

The project is designed as a modular foundation for:

* robust system monitoring
* sensor fusion (EKF)
* navigation and autonomy experiments
* reproducible robotics software architecture

The long-term goal is to demonstrate end-to-end robotics system integration with clear interfaces, monitoring, and structured launch orchestration.

---

## Current Features

### 1. Reproducible Bringup Layer (`nh_bringup`)

* Launches Gazebo simulation and RViz in one command
* Handles namespace remapping (`/j100_0000/...`)
* Normalizes TF topics for clean downstream usage
* Loads predefined RViz configuration

This ensures consistent startup and eliminates manual configuration steps.

---

### 2. C++ System Monitor (`nh_system_monitor`)

Custom ROS 2 C++ node that monitors system health in real time:

Monitored components:

* `/clock`
* TF (`/j100_0000/tf`, `/j100_0000/tf_static`)
* Odometry (`/j100_0000/platform/odom/filtered`)
* IMU (`/j100_0000/sensors/imu_0/data`)
* LiDAR (`/j100_0000/sensors/lidar3d_0/points`)
* Optional command velocity

Features:

* Message rate estimation
* Staleness detection
* Frame validation via tf2 lookups
* Periodic structured status output

This node serves as the foundation for reliable debugging and future autonomy development.

---

### 3. Initial State Estimation (EKF)

* Integration of robot_localization EKF
* Fusion of odometry and IMU data
* Clean filtered state output
* TF consistency validation via system monitor

---

## Architecture

High-level pipeline:

Gazebo Simulation
→ Sensor Topics (IMU, LiDAR, Odom)
→ EKF (State Estimation)
→ TF + Filtered Odometry
→ Monitoring & Visualization

The system is structured to isolate:

* Simulation
* Interface normalization
* Monitoring
* Estimation
* Future navigation modules

---

## Project Goals

Short-Term Goals:

* Complete topic normalization layer
* Finalize EKF ownership and tuning
* Improve monitoring granularity
* Ensure reproducibility across environments

Mid-Term Goals:

* Integrate Nav2 for goal-based navigation
* Evaluate SLAM (e.g., RTAB-Map or slam_toolbox)
* Add performance diagnostics and logging

Long-Term Vision:

* Expand toward a reusable robotics experimentation framework
* Demonstrate strong C++ ROS 2 system architecture skills
* Serve as a portfolio-grade example of structured robotics software design

---

## Why This Project Exists

Many robotics demos focus on launching stacks.

This project focuses on:

* System structure
* Monitoring
* Clean interfaces
* Ownership of state estimation
* Engineering discipline in ROS 2

It is intentionally built incrementally to demonstrate understanding of:

* rclcpp patterns
* tf2 integration
* Launch system design
* Namespaced topic handling
* Sensor fusion pipeline structuring

---

## How to Run

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch nh_bringup sim_with_rviz.launch.py
```

---

