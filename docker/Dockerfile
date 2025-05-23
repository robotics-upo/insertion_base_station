FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"] 

# Required ROS packages  
RUN sudo apt update && apt install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers \
    ros-noetic-vision-msgs ros-noetic-costmap-2d ros-noetic-amcl ros-noetic-map-server ros-noetic-urg-node \
    ros-noetic-aruco ros-noetic-dynamixel-sdk libgeographic-dev \
    ros-noetic-turtlebot3-*

# We also install Git and other tools
RUN sudo apt install -y git vim nano iputils-ping net-tools xterm gdb tmuxinator cmake 

# MESA drivers for hardware acceleration graphics (Gazebo and RViz)
RUN sudo apt -y install libgl1-mesa-glx libgl1-mesa-dri

# Create workspace and download files
RUN cd ~/ && git clone https://github.com/robotics-upo/insertion_base_station
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.sh
RUN cd ~/insertion_base_station/scripts && git pull && ./install_insertion.sh no_git 
#RUN cd ~/insertion_base_station/ && git pull && ~/insertion_base_station/scripts/install_g2o.sh 
#RUN cd ~/insertion_base_station/scripts && pwd && ./install_ceres.sh 
RUN cd ~/insertion_base_station/scripts && pwd && ./install_insertion.sh 

# By default the DockerFile uses the root as user. 
# We will add a custom user so that we work in user space by default
# And then switch to that user
# Source: https://www.docker.com/blog/understanding-the-docker-user-instruction/
RUN useradd -ms /bin/bash insertion
# Sometimes it is good to add a password so that we can use sudo to install things
# To this end, uncomment the following line, remove extra ### and change the password 
RUN echo "insertion:insertion" | chpasswd  
#### Remove #### to change the password
RUN adduser insertion sudo & adduser insertion dialout & adduser insertion plugdev & echo Hi
RUN usermod -aG sudo insertion
USER insertion

# We will use the home directory as work directory
# So that the docker starts at HOME!
WORKDIR "/home/insertion"

