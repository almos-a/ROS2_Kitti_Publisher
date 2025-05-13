from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('kitti_publisher'),
        'rviz',
        'kitti_viz.rviz'
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
            executable='kitti_publisher',
            name='kitti_publisher',
            output='screen',
            parameters=[
                {'kitti_seq_path': '/home/almos/kitti_dataset/KITTI_ds/dataset/sequences/00/'},
                {'playback_rate': 10.0}
            ]
        )
    ])
