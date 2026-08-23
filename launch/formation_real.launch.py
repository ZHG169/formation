#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    package_share = get_package_share_directory('formation')

    formation_launch = os.path.join(
        package_share,
        'launch',
        'formation.launch.py',
    )
    mission_real_config = os.path.join(
        package_share,
        'config',
        'mission_real.yaml',
    )
    follower_real_config = os.path.join(
        package_share,
        'config',
        'follower_formation_real.yaml',
    )

    control_mode = LaunchConfiguration('control_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode',
            default_value='leader_follower',
            description='Real-flight formation control mode.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(formation_launch),
            launch_arguments={
                'control_mode': control_mode,
                'mission_config': mission_real_config,
                'follower_formation_config': follower_real_config,
            }.items(),
        ),
    ])
