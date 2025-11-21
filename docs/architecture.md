# 🧩 System Architecture — Waiter Robot Simulation (Gazebo + ROS 2)

This document describes the complete architecture of the Waiter Robot Simulation built on **ROS 2 Humble**, **Gazebo**, and a custom restaurant environment.

---

## 📌 Overview

The project simulates a fully autonomous waiter robot able to:
- Perceive its environment using simulated sensors  
- Build a map using **Cartographer SLAM**  
- Navigate autonomously using **Nav2**  
- Follow predefined routes inside a restaurant world built in Gazebo  
- React to obstacles and dynamic changes  

The system is split into 4 main modules:

---

## 🏠 1. Restaurant Gazebo World
- Custom **`.world`** file
- Includes tables, walls, chairs, floor textures  
- Defines lights, camera, and physics  
- Optional navigation markers or QR codes for extra orientation  
- All assets stored inside `ws_mobile/src/my_worlds/`

---

## 🤖 2. Robot Model (URDF/Xacro)
Your robot includes:
- Differential drive base (left + right wheel joints)  
- Lidar (360° 2D)  
- IMU  
- Front ultrasonic sensors  
- Base link + chassis + caster wheel  
- ROS 2 controllers:
  - `diff_drive_controller`
  - `joint_state_broadcaster`

URDF is included in:
`ws_mobile/src/my_robot_description/`

---

## 📡 3. Sensor Simulation
Simulated by Gazebo plugins:

| Sensor | Plugin | Topic |
|--------|-------------------------------|------------------------|
| Lidar | `gazebo_ros_ray_sensor` | `/scan` |
| IMU | `gazebo_ros_imu_sensor` | `/imu` |
| Ultrasonic (optional) | custom plugin | `/ultrasonic` |
| Wheel Odometry | diff drive plugin | `/odom` |

---

## 🧭 4. Navigation Stack (Nav2)
You use:
- **AMCL** for localization (when using saved map)  
- **Cartographer** for SLAM (when building the map)  
- `map_server` + `lifecycle_manager`  
- Path planner + controller  
- Custom costmap parameters to match the restaurant world  

All configs are stored in:
`ws_mobile/src/my_robot_nav/config/`

---

## 🗺️ Data Flow

```
Gazebo → Sensors → ROS 2 Topics → SLAM / Nav2 → Velocity Commands → Gazebo Robot Motion
```

---

## 🧱 Packages Overview

```
ws_mobile/
│── src/
│   ├── my_worlds/                 → Gazebo world + launch
│   ├── my_robot_description/      → URDF, meshes, sensors
│   ├── my_robot_bringup/          → Launch files
│   ├── my_robot_nav/              → Nav2 configs, SLAM config
│   └── my_robot_controllers/      → Diff drive controller configs
│
└── install/
```

---

## ✔️ Summary

This architecture allows complete simulation of a waiter robot in a realistic restaurant with mapping, navigation, sensors, and URDF modeling.  
