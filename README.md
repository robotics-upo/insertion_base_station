# insertion_base_station

ROS2 package to start the ROS2 part of the Insertion project. In particular:

* The ROS2 Bridge in the M600 PC
* The Eliko Driver in the M600 PC
* The displays at the base station (RViz2)


## Usage

To use the base station at this moment, you first have to build the Docker:

```
    > cd docker
    > docker build . -t insertion
```

Then, you should install the ros2_ws into the insertion shared folder:

```
    cd ../scripts
    ./install_ws.bash
``` 

Finally, you should execute the docker and inside it, you should build the workspace:

```
    cd ../docker
    ./run_insertion_base_station.bash
```

## Inside the docker

Once the docker is running, you first you should compile the workspace. We advice to
use symlink-install to reduce disk usage and to make the changes directly present in the repositories:

```
    > source /opt/ros/foxy/setup.bash
    > cd ros2_ws
    > colcon build --symlink-install
``` 

If everything goes OK, you can enjoy the base_station (maybe you have to fix some topic names in the launch and RViz):

```
    > source ~/ros2_ws/install/setup.bash
    > ros2 launch insertion_base_station base_station.launch.xml
```

Enjoy!