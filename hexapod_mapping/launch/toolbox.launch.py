#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
import os

def generate_launch_description():
    description_pkg = FindPackageShare('hexapod_description').find('hexapod_description')
    urdf_xacro = os.path.join(description_pkg, 'urdf', 'hexapod.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_xacro])
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
<<<<<<< HEAD
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': False}],
        ),

=======
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[robot_description, {'use_sim_time': False}],
        # ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_footprint',  # Correct for your robot
                'scan_topic': '/scan',           # Your scan topic
                'queue_size': 500,
                'transform_publish_period': 0.1,
                'mode': 'mapping',
                'max_laser_range': 5.0,
                'min_laser_range': 0.3,
                'minimum_time_interval': 1.0,    # Lowered for more frequent updates
                'throttle_scans': 20,            # Lowered for most robots
                'resolution': 0.10,              # More typical for SLAM (0.01 = high CPU)
                'enable_interactive_mode': True
            }]
        )
>>>>>>> d779087 (Add all SLAM-related source packages including:)
    ])