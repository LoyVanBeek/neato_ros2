import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    map_config = Path(
        get_package_share_directory('neato_bringup'), 'config', 'map_server.yaml')

    nav2_bringup = get_package_share_directory('nav2_bringup')
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py'),
        )
    )

    return LaunchDescription([
        Node(
              package='nav2_amcl',
              executable='amcl',
              name='amcl',
              output='screen'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[map_config]),
        navigation2,
      ])