#! /bin/bash
#! This script sets up the shared folder and a ROS2 workspace for the insertion base station application.
mkdir -p ~/insertion_shared/ros2_ws/src
cd ~/insertion_shared/ros2_ws/src

git clone https://github.com/robotics-upo/rviz_satellite.git -b foxy
git clone https://github.com/robotics-upo/upo_markers.git -b ros2
git clone https://github.com/robotics-upo/ars548_ros.git
git clone https://github.com/robotics-upo/insertion_base_station.git -b humble
git clone https://github.com/robotics-upo/eliko_ros.git

touch ~/insertion_shared/ros2_ws/src/ars548_ros/ars548_driver/COLCON_IGNORE
