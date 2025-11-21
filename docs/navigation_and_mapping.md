# 🧭 Navigation & Mapping — Cartographer + Nav2 Setup

This document describes how to build a map, load a map, and navigate autonomously using Nav2.

---

## 🗺️ 1. Mapping With Cartographer

Launch SLAM:

```bash
ros2 launch my_robot_nav cartographer.launch.py
```

Move the robot slowly to:
- Scan all tables  
- Capture corners of the restaurant  
- Avoid isolated zones  

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f my_restaurant_map
```

Generated files:

```
my_restaurant_map.yaml
my_restaurant_map.pgm
```

Store them in:
`ws_mobile/src/my_robot_nav/maps/`

---

## 🧭 2. Localization + Navigation (Nav2)

Start navigation:

```bash
ros2 launch my_robot_nav navigation.launch.py map:=my_restaurant_map.yaml
```

This launches:
- `map_server`
- `amcl`
- `controller_server`
- `planner_server`
- `behavior_server`
- `bt_navigator`
- `lifecycle_manager`

---

## 🗂️ 3. Costmaps Configuration

### Global costmap:
- Uses stored static map  
- Computes long-range paths  

### Local costmap:
- Uses Lidar + inflation  
- Detects dynamic obstacles  

Both configs available in:
`ws_mobile/src/my_robot_nav/config/costmaps.yaml`

---

## 🧠 4. Navigation Behavior

The robot supports:
- Goal navigation  
- Obstacle avoidance  
- Dynamic replanning  
- Recovery behaviors  

---

## 🎯 5. Sending a Navigation Goal

In RViz → Click “2D Nav Goal”

Robot will choose:
- Best global trajectory  
- Local velocity commands  
- Path adjustments  

---

## ✔️ Final Notes

This navigation setup fully supports:
- Mapping with Cartographer  
- Localization with AMCL  
- Full autonomous navigation inside a custom restaurant Gazebo world  

