# Autonomous Navigation Bot using LiDAR (ROS2)

This project implements a **fully autonomous navigation robot** using **ROS2**, **LiDAR**, **Nav2**, and custom motor control nodes. The bot performs mapping, localization, path planning, and obstacle avoidance.

---

## Overview

The robot consists of:

* **RPLiDAR A1** for 2D scan data
* **Custom BLDC + ESC motor system** controlled via ROS2
* **Encoders** for odometry
* **IMU (optional)**
* **Nav2 stack** for autonomous navigation
* **SLAM Toolbox** for mapping
* **TF2 transforms** for coordinate frames

This README explains the purpose of each ROS2 node and how to run them.

---

## Workspace Structure

```
ros2_ws/
 └── src/
      ├── robot_control        # Custom motor + encoder control package
      ├── rplidar_ros          # LiDAR driver
      ├── slam_toolbox         # SLAM
      └── ... others
```

---

# Running the Autonomous Navigation Bot

Open **multiple terminals** (or use tmux). Each command below shows **what to run** and **why it is needed**.

---

## Start Motor Bridge

```bash
ros2 run robot_control motor_bridge
```

**Purpose:**

* Sends motor PWM / speed commands to ESC
* Receives velocity commands (`cmd_vel`) from Nav2
* Bridges high-level navigation with low-level motor driver

---

## Start Encoder Odometry

```bash
ros2 run robot_control encoder_motor
```

**Purpose:**

* Reads wheel encoder pulses
* Publishes odometry on `/odom`
* Crucial for localization + navigation

---

## Start Laser → Base TF

```bash
ros2 launch robot_control laser_tf.launch.py
```

**Purpose:**

* Publishes static transform between LiDAR frame and robot base frame
* Required for correct scan alignment and SLAM

---

## Start the LiDAR

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1 serial_baudrate:=115200
```

**Purpose:**

* Starts RPLiDAR driver
* Publishes `/scan` topic with 360° laser data

---

## Start SLAM Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py
```

**Purpose:**

* Builds real-time map of the environment
* Publishes `/map` and TF frames
* Used for navigation and localization

---

## Start Nav2 Stack

```bash
ros2 launch nav2_bringup navigation_launch.py
```

**Purpose:**

* Activates Nav2 navigation pipeline:

  * Local + global planners
  * Controller server
  * Behavior tree navigator
  * Costmaps
* Listens to `/map`, `/odom`, `/scan`, and outputs `/cmd_vel`

---

## Start RViz Navigation View

```bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Purpose:**

* Visualizes:

  * Map
  * Robot position
  * Path planning
  * LiDAR scans
  * Costmaps
* Lets you set navigation goals

---

## View All TF Frames

```bash
ros2 run tf2_tools view_frames
```

**Purpose:**

* Generates `frames.pdf`
* Helps debug frame connectivity (`map → odom → base_link → laser`)

---

## Manual Control (Testing)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Purpose:**

* Manually drive the robot
* Useful to test motors + odometry before turning on Nav2

---

# Autonomous Navigation Flow

1. LiDAR publishes `/scan`
2. SLAM Toolbox creates `/map`
3. Odometry publishes `/odom`
4. TF connects map → odom → base_link → laser
5. Nav2 receives sensor data and computes path
6. `motor_bridge` converts `/cmd_vel` → motor PWM

---

# Saving And Loading Maps

Save a map:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

Load a map:

```bash
ros2 launch nav2_bringup bringup_launch.py map:=/path/to/my_map.yaml
```

---

# Notes

* Ensure LiDAR is detected at `/dev/ttyUSB1`
* Ensure correct static transforms
* Odometry must be stable for Nav2
* Build workspace using:

```bash
colcon build
source install/setup.bash
```

---

# Conclusion

Your autonomous robot is now capable of:

* Real-time LiDAR-based mapping
* Localization
* Path planning
* Fully autonomous navigation

