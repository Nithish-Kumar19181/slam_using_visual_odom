#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
import os

def generate_launch_description():
    
    description_pkg = FindPackageShare('hexapod_description').find('hexapod_description')
    urdf_xacro = os.path.join(description_pkg, 'urdf', 'hexapod.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_xacro])
    robot_description = {'robot_description': robot_description_content}

    ekf_config = os.path.join(
        FindPackageShare('hexapod_mapping').find('hexapod_mapping'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': False}],
        ),

        # RealSense Camera
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(realsense_launch),
        #     launch_arguments={
        #         'pointcloud.enable': 'false',
        #         'depth_module.profile': '640x480x15',
        #         'enable_depth': 'true',
        #         'enable_color': 'false',
        #         'enable_gyro': 'true',
        #         'enable_accel': 'true',
        #         'unite_imu_method': 'linear_interpolation',
        #         'publish_odom_tf': 'true',
        #         'clip_distance': '5.0',
        #         'align_depth.enable': 'true',
        #     }.items()
        # ),

        # IMU Fusion Node (Publishes to /camera/imu)
        # Node(
        #     package='realsense_imu_fusion',
        #     executable='realsense_imu_fusion_node',
        #     name='realsense_imu_fusion_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': False}],
        #     # No remapping needed if the fusion node publishes to /camera/imu
        # ),

        # Depth to LaserScan Conversion
        # Node(
        #     package='depth_to_laserscan',
        #     executable='depth_to_laserscan_node',
        #     name='depth_to_laserscan',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'output_frame': 'camera_link',
        #         'range_min': 0.3,
        #         'range_max': 5.0,
        #         'scan_time': 0.033,
        #         'scan_height': 10,
        #         'depth_topic': '/camera/camera/depth/image_rect_raw',
        #         'camera_info_topic': '/camera/camera/depth/camera_info',
        #         'inf_epsilon': 1.0
        #     }],
        #     remappings=[
        #         ('depth', '/camera/camera/depth/image_rect_raw'),
        #         ('depth_camera_info', '/camera/camera/depth/camera_info'),
        #         ('scan', '/scan')
        #     ]
        # ),

        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[ekf_config],
        # ),

        # SLAM Toolbox for Mapping
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'odom_frame': 'odom',
        #         'map_frame': 'map',
        #         'base_frame': 'base_link',
        #         'scan_topic': '/camera/scan',
        #         'queue_size': 700,
        #         'transform_publish_period': 0.1,
        #         'mode': 'mapping',
        #         'min_laser_range': 0.3,
        #         'max_laser_range': 5.0,
        #         'minimum_time_interval': 0.1,
        #         'throttle_scans': 25,
        #         'resolution': 0.04,
        #         'enable_interactive_mode': True
        #     }]
        # ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        # )
    ])