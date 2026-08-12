#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('formation')
    mission_config = os.path.join(
        package_share,
        'config',
        'mission.yaml',
    )
    leader_command_config = os.path.join(
        package_share,
        'config',
        'leader_command.yaml',
    )
    leader_control_config = os.path.join(
        package_share,
        'config',
        'leader_control.yaml',
    )
    follower_formation_config = os.path.join(
        package_share,
        'config',
        'follower_formation.yaml',
    )

    control_mode = LaunchConfiguration('control_mode')

    mission_node = Node(
        package='formation',
        executable='mission_node',
        name='mission_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            mission_config,
            {'control_mode': control_mode},
        ],
    )

    leader_command_node = Node(
        package='formation',
        executable='leader_command_node',
        name='leader_command_node',
        output='screen',
        emulate_tty=True,
        parameters=[leader_command_config],
    )

    leader_control_node = Node(
        package='formation',
        executable='leader_control_node',
        name='leader_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[leader_control_config],
    )

    follower_formation_node = Node(
        package='formation',
        executable='follower_formation_node',
        name='follower_formation_node',
        output='screen',
        emulate_tty=True,
        parameters=[follower_formation_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode',
            default_value='leader_follower',
            description=(
                'leader_follower is the normal split-node mode. '
                'centralized/distributed are legacy modes.'
            ),
        ),
        mission_node,
        leader_command_node,
        leader_control_node,
        follower_formation_node,
    ])
