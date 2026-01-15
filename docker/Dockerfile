# This is a custom-made image for simulating the new version of VRX
# With ROS2, Ubuntu 24.04 and new Gazebo
FROM osrf/ros:foxy-desktop


#RUN apt install ros-foxy-joy-teleop -y
#RUN apt install libspdlog-dev libceres-dev -y    

RUN apt-get update && apt-get install -y ros-foxy-teleop-twist-joy libspdlog-dev libceres-dev  ros-foxy-tf2-geometry-msgs geographiclib-tools ros-foxy-geographic-info libgeographic-dev 
RUN apt-get install -y libtclap-dev

# By default the DockerFile uses the root as user. 
# And then switch to that user
# We will add a custom user so that we work in user space by default
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

