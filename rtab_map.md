## 🕷️ Hexapod Robot - SLAM & Autonomous Navigation (ROS 2)

This project enables full **3D SLAM**, **2D map generation**, and **autonomous navigation** for a custom Arduino-based **hexapod robot**, using:

- **Intel RealSense D435i**
- **RTAB-Map** for 3D SLAM and localization
- **Nav2** for path planning
- **Custom controller node** to interpret `/cmd_vel` and send serial commands to the MCU

---

## 🧩 Dependencies

Ensure the following ROS 2 packages and tools are installed:
- ROS 2 Humble (Desktop or Base + Nav2 + Rviz)
- `rtabmap_ros` (`ros-humble-rtabmap-ros`)
- `realsense2_camera` (Intel RealSense SDK + ROS2 wrapper)
- `depthimage_to_laserscan`
- `nav2_bringup`
- Your custom packages:
  - `hexapod_navigation`
  - `hexapod_mapping`
  - `hexapod_controller`

---

## 🗺️ Mapping Workflow

### 1. Start TF & URDF
```bash
ros2 launch hexapod_mapping mapping.launch.py
````

### 2\. Launch RealSense Camera

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.profile:=640x480x15 \
  depth_module.profile:=640x480x15 \
  enable_gyro:=true \
  enable_accel:=true \
  unite_imu_method:=2
```

### 3\. Convert Depth to LaserScan

```bash
ros2 run depthimage_to_laserscan depth_to_laserscan_node
```

### 4\. Start RTAB-Map for Mapping

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  use_sim_time:=false \
  frame_id:=base_link \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  scan_topic:=/scan \
  subscribe_depth:=true \
  subscribe_scan:=true \
  subscribe_rgbd:=false \
  approx_sync:=true \
  rtabmap_args:="--Reg/Force3DoF true"
```

### 5\. Save the Maps

Save the 3D RTAB-Map database:

```bash
ros2 service call /rtabmap/save_database rtabmap_msgs/srv/SaveDatabase "{database_path: '/home/nithish/slam_hexapod_5th_jan_1/map_3d.db', overwrite: true}"
```

Save the 2D Occupancy Map:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/nithish/slam_hexapod_5th_jan_1/map_2d \
  --ros-args -p use_sim_time:=false
```

-----

## 🧭 Localization and Navigation Workflow

### 1\. Start RTAB-Map in Localization Mode

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  database_path:=/home/nithish/slam_hexapod_5th_jan_1/map_3d.db \
  localization:=true \
  use_sim_time:=false \
  frame_id:=base_link \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  scan_topic:=/scan \
  subscribe_depth:=true \
  subscribe_scan:=true \
  subscribe_rgbd:=false \
  approx_sync:=true \
  rtabmap_args:="--Reg/Force3DoF true"
```

### 2\. Start Nav2 Stack

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true \
  params_file:=/home/nithish/slam_hexapod_5th_jan_1/src/hexapod_navigation/config/nav2_params.yaml
```

### 3\. Launch RViz2 (Autoconfigured)

```bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

-----

## 🕹️ Hexapod Controller

Launch Hexapod Serial Command Node:
This node listens to `/cmd_vel` and sends motion commands to your Arduino-based hexapod via `/serial_commands`.

```bash
ros2 launch hexapod_controller controller.launch.py
```

Start Navigation Translator Node:

```bash
ros2 run hexapod_navigation hexapod_navigator
```

-----

## 📂 Project Structure

```
slam_hexapod_5th_jan_1/
├── map_3d.db                      # RTAB-Map 3D DB
├── map_2d.yaml                    # 2D map YAML
├── map_2d.pgm                     # 2D map image
└── src/
    ├── hexapod_navigation/
    │   └── config/nav2_params.yaml
    ├── hexapod_mapping/
    └── hexapod_controller/
```

-----

## 💡 Notes

  - `map_2d.yaml` must have a valid origin, resolution, and `mode: trinary`.
  - `map_server` should load `map_2d.yaml` in localization mode.
  - Global costmap should subscribe to `/rtabmap/grid_prob_map` if you're using RTAB-Map grid maps.
  - Ensure that transforms like `base_link -> camera_link`, `base_link -> odom`, and `map -> odom` are available.
  - Joystick, GUI goal selection, or simple action client can be used to send goals.

-----

## 🎯 Sending Goals (Test Example)

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

-----

## 🧪 Debugging Tips

  - Use `ros2 topic echo /tf` and `ros2 topic echo /scan` to verify data.
  - Use `ros2 run tf2_tools view_frames` to inspect the TF tree.
  - Use `ros2 topic hz /cmd_vel` to check Nav2 is issuing commands.
  - Make sure your robot doesn’t rotate more than ±15° per command.

-----

## 🔧 Future Improvements

  - Add joystick or GUI controller for manual override
  - Include obstacle avoidance testing
  - Tune Nav2 behavior tree and planner/controller plugins for better pathing

-----

## 🤖 Credits

Created by **Nithish Kumar R**, using:

  - ROS 2 Humble
  - RTAB-Map
  - Nav2
  - Intel RealSense D435i
  - Custom Arduino-based hexapod

## Outputs

<img width="1506" height="932" alt="Screenshot from 2025-07-16 12-00-24" src="https://github.com/user-attachments/assets/711c178c-8c84-48f8-8eb5-158e7d2ce96a" />


<!-- end list -->

```
```
