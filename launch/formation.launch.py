#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('formation')

    default_mission_config = os.path.join(
        package_share,
        'config',
        'mission.yaml',
    )
    default_leader_command_config = os.path.join(
        package_share,
        'config',
        'leader_command.yaml',
    )
    default_leader_control_config = os.path.join(
        package_share,
        'config',
        'leader_control.yaml',
    )
    default_follower_control_config = os.path.join(
        package_share,
        'config',
        'follower_control.yaml',
    )

    control_mode = LaunchConfiguration('control_mode')
    mission_config = LaunchConfiguration('mission_config')
    leader_command_config = LaunchConfiguration(
        'leader_command_config'
    )
    leader_control_config = LaunchConfiguration(
        'leader_control_config'
    )
    follower_control_config = LaunchConfiguration(
        'follower_control_config'
    )

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

    follower_control_node = Node(
        package='formation',
        executable='follower_control_node',
        name='follower_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[follower_control_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode',
            default_value='centralized',
            description=(
                'leader_follower is the normal split-node mode. '
                'centralized/distributed are legacy modes.'
            ),
        ),
        DeclareLaunchArgument(
            'mission_config',
            default_value=default_mission_config,
            description='Mission node parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'leader_command_config',
            default_value=default_leader_command_config,
            description='Leader command node parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'leader_control_config',
            default_value=default_leader_control_config,
            description='Leader control node parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'follower_control_config',
            default_value=default_follower_control_config,
            description=(
                'Follower control node parameter YAML file.'
            ),
        ),
        mission_node,
        leader_command_node,
        leader_control_node,
        follower_control_node,
    ])
