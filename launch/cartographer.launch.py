from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'kitti_publisher' # Replace with your package name
    cartographer_config_dir = os.path.join(
        get_package_share_directory(package_name),
        'config'
    )
    configuration_basename = 'carto.lua'
    print(f"Cartographer Configuration Directory (Launch File): {cartographer_config_dir}")

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_occupancy_grid',
            default_value='True',
            description='Whether to run the occupancy grid node'
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='True',
            description='Whether to run RViz'
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[('/scan', '/kitti/velodyne'),
                        ('/image', '/kitti/image')]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node', 
            name='occupancy_grid_node',
            parameters=[{'resolution': 0.05}],
            condition=IfCondition(LaunchConfiguration('run_occupancy_grid'))
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(cartographer_config_dir, 'default.rviz')],
            condition=IfCondition(LaunchConfiguration('run_rviz'))
        )
    ])