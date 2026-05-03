# Insertion Base Station

A custom ROS 2 Ground Control Station (GCS) developed for the **INSERTION project** (PID2021-127648OB-C31).

---

## Docker

### Build and Run the Container

```bash
cd ~/insertion_base_station/docker
./run_insertion_base_station.bash
```

### Access the Running Container

```bash
sudo docker exec -it insertion bash
```

---

## Usage

### Environment Initialization

Load ROS 2 environment variables and build the workspace:

```bash
source /opt/ros/foxy/setup.bash

cd ros2_ws
colcon build --symlink-install

source ~/ros2_ws/install/setup.bash
```

---

## Configuration and Parameters

The base station requires specific ROS 2 parameters to define the local Cartesian origin `(0, 0, 0)` for the map. These parameters must be set in the launch file:

| Parameter     | Value      | Description                               |
|--------------|-----------|-------------------------------------------|
| `origin_lat` | 39.794258 | Latitude of the local origin              |
| `origin_lon` | -4.081346 | Longitude of the local origin             |
| `origin_alt` | 536.3     | Altitude of the local origin (meters)     |

### Additional Parameters

| Parameter             | Default | Description                                           |
|----------------------|--------|-------------------------------------------------------|
| `default_flight_alt` | 20.0   | Default altitude for newly added waypoints            |
| `min_safe_alt`       | 5.0    | Minimum allowed altitude when selecting points on map |
| `status_timeout_ms`  | 3000   | Time (ms) without telemetry before DISCONNECTED state |

---

## Important Note on Networking

The **Execution** and **Abort** functions rely on hardcoded IP addresses and SSH credentials to ensure immediate execution without depending on ROS topic discovery.

If your network configuration changes, update the following values in:

```bash
mission_panel.cpp
```

| Setting     | Value        |
|------------|-------------|
| Drone 1 IP | 192.168.25.5 |
| Drone 2 IP | 192.168.25.7 |
| SSH User   | m600         |
| SSH Pass   | upo          |

---

## Mission File Format (CSV)

Missions are stored and loaded using a CSV format with the following structure:

```csv
ACTION, LATITUDE, LONGITUDE, RELATIVE_ALTITUDE
```

---

## Operation

The project includes two launch files:

- **`base_station.launch.xml`**  
  Used to start the base station in **simulation mode**, typically working with recorded ROS bags.

- **`live_station.launch.xml`**  
  Used to start the base station for **real-time missions** with actual UAVs.

---
