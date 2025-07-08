from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
<<<<<<< HEAD
            parameters=['/home/nithish/nithish_hexapod/src/hexapod_mapping/config/ekf.yaml']
=======
            parameters=['/home/nithish/slam_hexapod_5th_jan_1/src/hexapod_mapping/config/ekf.yaml']
>>>>>>> d779087 (Add all SLAM-related source packages including:)
        )
    ])
