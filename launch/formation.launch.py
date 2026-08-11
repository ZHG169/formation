#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def distributed_condition(control_mode):
    return IfCondition(
        PythonExpression([
            "'",
            control_mode,
            "' == 'distributed'",
        ])
    )


def generate_launch_description():
    package_share = get_package_share_directory('formation')
    config_file = os.path.join(
        package_share,
        'config',
        'formation.yaml',
    )

    control_mode = LaunchConfiguration('control_mode')

    formation_node = Node(
        package='formation',
        executable='formation_node',
        name='formation_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,
            {'control_mode': control_mode},
        ],
    )

    leader_command_node = Node(
        package='formation',
        executable='leader_command_node',
        name='leader_command_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
    )

    distributed_nodes = [
        Node(
            package='formation',
            executable='distributed_vehicle_node',
            name=f'distributed_vehicle_{vehicle_id}',
            output='screen',
            emulate_tty=True,
            parameters=[config_file],
            condition=distributed_condition(control_mode),
        )
        for vehicle_id in (1, 2, 3)
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode',
            default_value='centralized',
            description='centralized or distributed formation control',
        ),
        formation_node,
        leader_command_node,
        *distributed_nodes,
    ])
