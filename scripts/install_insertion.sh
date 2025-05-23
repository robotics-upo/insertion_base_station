#! /bin/bash
export ROS_DISTRO=noetic
sudo apt-get install ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-geographic-msgs -y
sudo apt-get install ros-$ROS_DISTRO-costmap-2d ros-$ROS_DISTRO-octomap-server ros-$ROS_DISTRO-octomap-mapping -y
sudo apt-get install libpcl-dev libpcl-kdtree1.10

# Create workspace
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~
mkdir -p ~/insertion_ws/src
cd ~/insertion_ws/
catkin_make
cd ~/insertion_ws/src

source ~/insertion_ws/devel/setup.bash

## Dependencies for Behavior Tree
echo "\n Installing BT Dependencies \n\n"
sudo apt-get install ros-$ROS_DISTRO-ros-type-introspection ros-$ROS_DISTRO-nmea-msgs ros-$ROS_DISTRO-mavlink ros-$ROS_DISTRO-geographic-msgs -y
sudo apt-get install ros-$ROS_DISTRO-mavros
sudo apt-get install qtbase5-dev libqt5svg5-dev -y
sudo apt install libdw-dev libann-dev -y
sudo apt-get install ros-noetic-geographic-msgs ros-noetic-nmea-msgs

sudo apt-get install libqt5charts5-dev libqt5widgets5
cd ~/insertion_ws/src

if [ $1="no_git" ]; then
   echo Not downloading git
else
    # For localization stuff
    git clone -b noetic https://github.com/robotics-upo/dll.git
    git clone https://github.com/robotics-upo/odom_to_tf.git

    echo "\n Installing BT Packages \n\n"
    # git clone https://github.com/robotics-upo/behavior-tree-upo-actions.git
    git clone https://github.com/robotics-upo/BehaviorTree.CPP.git
    git clone https://github.com/robotics-upo/Groot.git

    # To install planner lazy-theta*
    echo "\n Installing Lazy Theta* Planner \n\n"
    git clone -b marsupial https://github.com/robotics-upo/lazy_theta_star_planners.git

    # To install NIX_common package
    echo "\n Installing NIX common package \n\n"
    git clone https://github.com/robotics-upo/nix_common.git

    # To get action for actionlib
    echo "\n Installing UPO Actions \n\n"
    git clone -b GPS https://github.com/robotics-upo/upo_actions.git

    #To get a marker in the desired frame_link
    echo "\n Installing UPO Markers \n\n"
    git clone https://github.com/robotics-upo/upo_markers.git

    # Get timed roslaunch
    git clone -b melodic-devel https://github.com/robotics-upo/timed_roslaunch.git

    # Get upo path tracker
    git clone -b simplified https://github.com/robotics-upo/upo_path_tracker.git

    # Get matrice trajectory tracker
    git clone -b GPS https://github.com/robotics-upo/matrice_traj_tracker.git

    # ARS548 ROS
    git clone -b noetic https://github.com/robotics-upo/ars548_ros.git

    # RViz Satellite
    git clone -b ros1 https://github.com/nobleo/rviz_satellite

fi

