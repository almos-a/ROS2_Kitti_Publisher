from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('kitti_publisher'),
        'rviz',
        'kitti_raw_viz.rviz'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_kitti_viewer',
            output='screen',
            arguments=['-d', config_file_path],
        ),
        Node(
            package='kitti_publisher',
            executable='kitti_raw_publisher',
            name='kitti_raw_publisher',
            output='screen',
            parameters=[
                {'playback_rate': 1.0}
            ]
        )
    ])
