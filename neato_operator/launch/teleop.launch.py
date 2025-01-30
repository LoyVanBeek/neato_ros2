import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    speedlink_config_path = Path(
        get_package_share_directory('neato_operator'), 'config', 'speedlink.yaml')
    virtual_joy_config_path = Path(
        get_package_share_directory('neato_operator'), 'config', 'virtual.yaml')

    return LaunchDescription([
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
            name='joystick_teleop',
            output='screen',
            parameters=[virtual_joy_config_path]),

        # Push this to a separate namespace: https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html
        # because now both publish to the same /joy topic but the virtual joystick needs a separate config
        # include another launch file in the virtual_joy_ns namespace
        # GroupAction(
        #     actions=[
        #         # push-ros-namespace to set namespace of included nodes
        #         PushRosNamespace('virtual_joy_ns'),
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(
        #                 os.path.join(
        #                     get_package_share_directory('virtual_joystick'),
        #                     'launch',
        #                     'vjoy.launch.py'),
        #             )
        #         ),
        #         Node(
        #             package='teleop_twist_joy',
        #             executable='teleop_node',
        #             name='virtual_joystick_teleop',
        #             output='screen',
        #             parameters=[virtual_joy_config_path]
        #         ),
        #     ]
        # )
    ])