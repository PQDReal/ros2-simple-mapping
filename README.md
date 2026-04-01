# ROS2 Simple Mapping

This project implements a **basic 2D occupancy grid mapping system** in ROS 2 using Gazebo simulation — without using any built-in SLAM libraries (e.g., no SLAM Toolbox, no Nav2 mapping).

The robot explores the environment using LiDAR and builds a map in real-time.

---

## 🚀 Features

* Custom occupancy grid mapping (log-odds)
* LaserScan-based obstacle detection
* Autonomous exploration (reactive obstacle avoidance)
* Map visualization in RViz
* Map saving as `.pgm + .yaml`

---

## 🧰 Requirements

### OS

* Ubuntu 22.04

### Install ROS 2 Humble

Follow official guide:
👉 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Or quick install:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

---

### Install dependencies

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-gazebo-ros \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-rviz2
```

---

## 📦 Build Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repo
git clone https://github.com/YOUR_USERNAME/ros2-simple-mapping.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🌍 Run Simulation

### Terminal 1 — Start Gazebo

```bash
source /opt/ros/humble/setup.bash

gzserver --verbose ~/ros2_ws/src/ros2-simple-mapping/worlds/my_world.world \
  -s /opt/ros/humble/lib/libgazebo_ros_init.so \
  -s /opt/ros/humble/lib/libgazebo_ros_factory.so \
  -s /opt/ros/humble/lib/libgazebo_ros_force_system.so
```

---

### Terminal 2 — Launch GUI

```bash
gzclient
```

---

### Terminal 3 — Start Mapper

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run simple_mapper mapper_node --ros-args -p use_sim_time:=True
```

---

### Terminal 4 — Start Explorer

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run simple_mapper explorer_node --ros-args -p use_sim_time:=True
```

---

### Terminal 5 — Visualize in RViz

```bash
rviz2
```

Set:

* Fixed Frame → `odom`
* Add:

  * Map → `/map`
  * LaserScan → `/scan`
  * TF

---

## 💾 Save Map

```bash
ros2 service call /save_map std_srvs/srv/Empty {}
```

Output:

```
~/maps/my_map.pgm
~/maps/my_map.yaml
```

---

## 🧠 How It Works

### Mapping (mapper_node)

1. Receives LiDAR data (`/scan`)
2. Gets robot pose from `/odom`
3. Projects laser rays into the world
4. Uses **Bresenham algorithm** to trace rays
5. Updates grid:

   * free space → decrease log-odds
   * obstacles → increase log-odds
6. Publishes `/map` as `OccupancyGrid`

---

### Exploration (explorer_node)

* Moves forward by default
* Stops when obstacle is too close
* Rotates to avoid collision
* Adds small randomness to avoid loops

---

## ⚙️ Key Parameters

### Mapper

| Parameter    | Description            |
| ------------ | ---------------------- |
| resolution   | Size of each grid cell |
| width/height | Map size               |
| lo_occ       | Occupied confidence    |
| lo_free      | Free confidence        |

### Explorer

| Parameter     | Description           |
| ------------- | --------------------- |
| stop_dist     | Minimum safe distance |
| forward_speed | Movement speed        |
| turn_speed    | Turning speed         |

---

## ⚠️ Notes

* No SLAM libraries are used — everything is implemented manually.
* Map quality depends on exploration coverage.
* Odometry drift may affect accuracy.

---

## 📸 Example Output

* Black = obstacles
* White = free space
* Gray = unknown


