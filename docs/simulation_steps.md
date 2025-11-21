# 🧪 Simulation Steps — Building & Running the Waiter Robot in Gazebo

This document explains all steps to launch the Gazebo world, spawn the robot, visualize sensors, and prepare for SLAM and navigation.

---

## 1️⃣ Build the Workspace

```bash
cd ws_mobile
colcon build
source install/setup.bash
```

---

## 2️⃣ Launch the Restaurant World

```bash
ros2 launch my_worlds restaurant_world.launch.py
```

This opens:
- Your custom restaurant environment  
- Correct physics  
- Lights and objects  

---

## 3️⃣ Spawn the Robot Into Gazebo

```bash
ros2 launch my_robot_bringup spawn_robot.launch.py
```

This loads:
- URDF model  
- Lidar, IMU, ultrasonic sensors  
- Diff drive controllers  

---

## 4️⃣ Visualize All Topics in RViz2

```bash
ros2 launch my_robot_bringup rviz.launch.py
```

Check:
- `/scan` (lidar)  
- `/odom`  
- `/imu`  
- `/joint_states`  
- TF tree: `base_link → base_footprint → sensors`  

---

## 5️⃣ Teleoperate the Robot (Optional)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Useful to test:
- Wheel controllers  
- Odometry  
- Lidar scanning  

---

## 6️⃣ Generate the Map Using Cartographer SLAM

```bash
ros2 launch my_robot_nav cartographer.launch.py
```

Then move the robot to scan all the restaurant.

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f my_restaurant_map
```

Map files generated:
```
my_restaurant_map.yaml
my_restaurant_map.pgm
```

---

## ✔️ Simulation Ready

At this point:
- World loads correctly  
- Robot moves  
- Sensors publish data  
- Map is saved  

we can now run navigation.
