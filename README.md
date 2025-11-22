# Waiter Robot Simulation in Gazebo + Navigation

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-orange?style=for-the-badge)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-blue?style=for-the-badge)](http://gazebosim.org/)
[![Cartographer](https://img.shields.io/badge/Cartographer-SLAM-red?style=for-the-badge)](https://google-cartographer.readthedocs.io/en/latest/)


---

## 🚀 Overview

Simulates a **waiter robot** in a **custom restaurant** using **Gazebo**, **ROS2 Navigation**, and **Cartographer SLAM**.  

Key features:

 🔹 Custom restaurant Gazebo world  
 🔹 Mobile robot with navigation & obstacle avoidance  
 🔹 ROS2 nodes for robot control & simulation  
 🔹 **Cartographer SLAM** for mapping before navigation  
 🔹 Path planning & RViz2 visualization  

---

## 🗂️ Project Structure

```text
ws_mobile/
├── src/
│   ├── mobile_robot/         # Robot package
│   │   ├── launch/           # Launch files for navigation & SLAM
│   │   ├── mobile_robot/     # Robot description & scripts
│   │   ├── model/            # URDF or mesh models
│   │   ├── parameters/       # Navigation parameters
│   │   ├── resource/         # Misc resources
│   │   └── test/             # Unit tests
│   ├── my_worlds/            # Custom Gazebo world package
│   │   └── launch/
│   └── nav2_config/          # Navigation2 configuration
├── README.md
├── .gitignore
└── ...
```

---

## ⚙️ Prerequisites

- Ubuntu 24.04 (PC or Raspberry Pi 5)  
- ROS2 Jazzy installed  
- Gazebo simulator installed  
- `colcon` build tool  
- RViz2  
- Cartographer ROS2 SLAM package  

---

## 🛠️ Setup Instructions

1. **Build the ROS2 workspace:**
```bash
colcon build --symlink-install
source install/setup.bash
```

2. **Create a map with Cartographer:**
```bash
ros2 launch mobile_robot cartographer_slam_launch.py
```

3. **Run navigation using the generated map:**
```bash
ros2 launch mobile_robot navigation_launch.py
```

---

## ▶️ How it Works

1. Gazebo loads the restaurant world  
2. Robot spawns in the environment  
3. **Cartographer SLAM** builds a map  
4. Navigation stack plans paths & avoids obstacles using the map  
5. Use RViz2 to visualize movement, sensors, and paths  

---


## 🖼️ Visual Demo

### Map_creation
[Watch Map_Creation](media/map.mp4)
### Navigation
[Watch Navigation](media/nav.mp4)

---
## 💡 Notes

- Edit `my_restaurant.world` to customize the environment  
- Navigation parameters are in `nav2_config` and robot package launch files  
- Always generate the SLAM map before running navigation  

---


## 🔗 References

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)  
- [Gazebo Simulator](http://gazebosim.org/)  
- [ROS2 Navigation2 Stack](https://navigation.ros.org/)  
- [Cartographer ROS2 SLAM](https://google-cartographer.readthedocs.io/en/latest/)



---
## 📧 Contact
**Manar Daghsni**  
📧 manardaghsni@gmail.com  
🔗 [LinkedIn](https://linkedin.com/in/daghsni-manar)

---
