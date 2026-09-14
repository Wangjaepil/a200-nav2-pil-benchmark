# Copyright 2026 A200 Navigation Project
# SPDX-License-Identifier: Apache-2.0

"""Launch the shadow-mode A200 predictive collision monitor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description with the installed YAML as its default."""
    package_share = get_package_share_directory(
        'a200_predictive_collision_monitor'
    )
    default_params = os.path.join(
        package_share, 'config', 'predictive_collision_monitor.yaml'
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Predictive collision monitor parameter file',
        ),
        Node(
            package='a200_predictive_collision_monitor',
            executable='predictive_collision_monitor',
            name='a200_predictive_collision_monitor',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/tf', '/a200_0000/tf'),
                ('/tf_static', '/a200_0000/tf_static'),
            ],
        ),
    ])
