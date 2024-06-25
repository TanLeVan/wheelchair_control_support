# Overview
This is my master thesis project.

Obstacle Avoidance: Used a modified Dynamic Window Approach to change the velocity of the wheelchair when detecting future collision to prevent collision

# Environment
- Ubuntu 22.04
- Ros Humble

# Dependencies
- urg_node2: 
    - To start and run Hokugo UST-20LX
    - URL: https://github.com/Hokuyo-aut/urg_node2
- ros2_laser_scan_merger:
    - To merge to Hokugo UST-20LX laser scan
    - URL: https://github.com/mich1342/ros2_laser_scan_merger.git
- pointcloud_to_laserscan:
    - URL: https://github.com/ros-perception/pointcloud_to_laserscan.git
- ros2_whill:
    - ROS package for WHILL. Non-official
    - URL: https://github.com/nanoshimarobot/ros2_whill
- ros2_whill_interfaces:
    - Requirement for ros2_whill
    - URL: https://github.com/whill-labs/ros2_whill_interfaces/tree/crystal-devel
# How to use
Installing
```
mkdir ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/TanLeVan/wheelchair_control_support
cd ../
colcon build --packages-select wheelchair_control_support
```

To run autonomous obstacle avoidance
```bash
ros2 launch wheelchair_control_support support_system.launch.py
```
