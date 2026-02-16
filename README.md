# new_horizon

ROS 2 robotics integration project.

## Features
- Gazebo bringup integration
- Custom RViz launch
- C++ System Monitor node
- Initial EKF configuration

## Run

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch nh_bringup sim_with_rviz.launch.py

