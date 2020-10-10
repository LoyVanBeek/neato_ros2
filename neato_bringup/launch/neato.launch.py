import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    robot_description_pkg = get_package_share_directory('neato_description')
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg, 'launch', 'description.launch.py'),
        )
    )

    return LaunchDescription([
        robot_description,
        Node(
              package='neato_ros2_python',
              executable='neato_node',
              name='neato_node',
              output='screen'),
      ])