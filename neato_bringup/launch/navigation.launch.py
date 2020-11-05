import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector
from launch.events import matches_action
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition

from typing import Optional


def transition_trigger(trigger_source: LifecycleNode,
                       in_state: str,
                       transition: int,
                       trigger_sink: Optional[LifecycleNode]=None):
    if not trigger_sink:
        trigger_sink = trigger_source

    transition_desc = ChangeState.valid_transitions[transition]
    return RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=trigger_source, goal_state=in_state,
            entities=[
                LogInfo(
                    msg="{} reached the '{}' state, calling '{}' on {}".format(trigger_source,
                                                                    in_state,
                                                                    transition_desc,
                                                                    trigger_sink)),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(trigger_sink),
                    transition_id=transition,
                )),
            ],
        )
    )


def generate_launch_description():
    map_config = Path(
        get_package_share_directory('neato_bringup'), 'config', 'map_server.yaml')

    nav2_bringup = get_package_share_directory('nav2_bringup')
    navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py'),
        )
    )

    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen')

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[map_config])

    activate_amcl = transition_trigger(amcl, 'inactive', Transition.TRANSITION_ACTIVATE)

    configure_amcl = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(amcl),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    configure_map_server = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_server = transition_trigger(map_server, 'inactive', Transition.TRANSITION_ACTIVATE)

    ld = LaunchDescription([
        activate_amcl,
        amcl,
        activate_map_server,
        map_server,
        configure_map_server,
        configure_amcl,
        # navigation2,
        # ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/loy/map.yaml is enough
      ])

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    return ld