import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    speedlink_config_path = Path(
        get_package_share_directory('neato_operator'), 'config', 'speedlink.yaml')

    return LaunchDescription([
        Node(
              package='joy',
              executable='joy_node',
              name='joystick_input',
              output='screen'),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='joystick_teleop',
            output='screen',
            parameters=[speedlink_config_path]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('virtual_joystick'),
                    'launch',
                    'vjoy.launch.py'),
            )
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='virtual_joystick_teleop',
            output='screen',
            parameters=[speedlink_config_path]),
      ])