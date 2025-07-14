# Hexapod RealSense Visual + Depth SLAM with SLAM Toolbox

This repository demonstrates **2D SLAM** on a hexapod robot using an Intel RealSense D455 camera. It combines **visual odometry** (`rtabmap_odom`) with **depth-to-laserscan** conversion, allowing `slam_toolbox` to produce a 2D occupancy map in ROS 2 Humble.

The names of the files must be changed in the future 

---

## 🚀 Launch Sequence

**Run the following commands in order:**

1. **Start the RSP by launching this:**
    ```bash
    ros2 launch hexapod_mapping mapping.launch.py 
    ```
    
2. **Start the RealSense camera driver:**
    ```bash
    ros2 launch realsense2_camera rs_launch.py \
        rgb_camera.profile:=640x480x15 \
        depth_module.profile:=640x480x15 \
        enable_gyro:=true \
        enable_accel:=true \
        unite_imu_method:=2
    ```

3. **Start RGB-D visual odometry (input odometry for SLAM Toolbox):**
    ```bash
    ros2 run rtabmap_odom rgbd_odometry --ros-args --remap rgb/image:=/camera/camera/color/image_raw --remap depth/image:=/camera/camera/depth/image_rect_raw --remap rgb/camera_info:=/camera/camera/color/camera_info --remap odom:=/odom_cam -p frame_id:=base_link -p publish_tf:=false -p approx_sync:=true -p approx_sync_max_interval:=0.02 -p Reg/MinInliers:=10 -p queue_size:=30

    ```

    **or**

   ```
   ros2 run rtabmap_odom rgbd_odometry --ros-args --remap rgb/image:=/camera/camera/color/image_raw --remap depth/image:=/camera/camera/depth/image_rect_raw --remap rgb/camera_info:=/camera/camera/color/camera_info --remap odom:=/odom_cam -p frame_id:=base_link -p publish_tf:=false -p approx_sync:=true -p approx_sync_max_interval:=0.02 -p Reg/MinInliers:=10 -p queue_size:=30


   

   

5. **Start the depth-to-laserscan converter:**
    ```bash
    ros2 run depth_to_laserscan depth_to_laserscan_node
    ```

6. Launch the 2d_odom from the camera
    ```bash
    ros2 run realsense_imu_fusion odom_2d_filter_node 
    ```
7. Launch Slam_Tool_Box usign this command
    ```bash
    ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/nithish/slam_hexapod_5th_jan_1/src/hexapod_mapping/config/mapper_params_online_async.yaml use_sim_time:=false
    ```

8. Launch debugging commands 
    ```bash
    ros2 run rqt_tf_tree rqt_tf_tree --force-discover

    ros2 run tf2_tools view_frames
    ```
Output must be like for the tf_tree :
![image](https://github.com/user-attachments/assets/f7118f0b-da14-418d-9c35-87c5e7572698)

Rqt_Graph Output:
![image](https://github.com/user-attachments/assets/25462cc4-a92f-4e2d-a61b-33e306c46a7c)

---

## 🌐 TF Tree Overview

```
map
 └── odom
      └── base_link
           └── camera_link
                ├── camera_color_frame
                │    └── camera_color_optical_frame
                └── camera_depth_frame
                     └── camera_depth_optical_frame
```

- `base_link` is the root of the robot.
- `camera_link` is fixed to the robot chassis.
- `odom` is published by `rtabmap_odom`.
- `map → odom` is published by `slam_toolbox` (for loop closure and drift correction).

---

## 🔁 Important Topics

| Topic   | Type                        | Publisher                    | Consumer           | Frame      |
|---------|-----------------------------|------------------------------|--------------------|------------|
| /scan   | `sensor_msgs/LaserScan`     | depth_to_laserscan_node      | slam_toolbox       | base_link  |
| /odom   | `nav_msgs/Odometry`         | rtabmap_odom                 | slam_toolbox       | odom       |
| /tf, /tf_static | TF transforms        | all nodes                    | everyone           | -          |

---

## ⚙️ Key Parameters

### **slam_toolbox** (in `toolbox.launch.py`)
- `scan_topic`: `/scan`
- `base_frame`: `base_link`
- `odom_frame`: `odom`
- `map_frame`: `map`
- `use_sim_time`: `false`

### **rtabmap_odom**
- `publish_tf`: `true`
- `approx_sync`: `true`
- `frame_id`: `base_link`
- `odom_frame_id`: `odom`
- `queue_size`: `30`
- `Reg/MinInliers`: `10` (for odometry stability)

---

## 📸 Sensor Calibration

Intel RealSense D455 intrinsics (adjust as needed):
- `fx = 617.0`
- `fy = 617.0`
- `cx = 320.0`
- `cy = 240.0`

---

## 🛠️ Node Roles

- **rtabmap_odom**: Provides robust odometry from RGB-D data.
- **slam_toolbox**: Performs 2D mapping with loop closure and map serialization.
- **depth_to_laserscan_node**: Generates 2D laser scans from depth images (replaces a real LiDAR).
- **realsense2_camera**: Publishes depth, RGB, and IMU data.
- **transform.launch.py**: Establishes all required static transforms for SLAM and navigation.

---

## 🧪 Testing Tips

- Use **rviz2** to visualize `/map`, `/odom`, `/scan`, and the TF tree.
- If you see `Message Filter dropping message` errors, try reducing the `/scan` rate or ensure TF timestamps are synchronized.
- Run `ros2 topic hz /scan` — a frequency of ~5 Hz works best with slam_toolbox.

---

## 🧼 Future Improvements

- Add rosbag2 recording support.
- Integrate joystick/keyboard teleop.
- Add YAML config files for launch parameters.

---
## Map output 
![image](https://github.com/user-attachments/assets/230ffa4b-d8e9-497b-bf0e-1d7718d697dd)

