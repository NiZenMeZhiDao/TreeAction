#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_config_arg = DeclareLaunchArgument(
        'param_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('r2_bt'),
            'config',
            'param.yaml',
        ]),
        description='YAML file containing the place_kfs section.',
    )

    return LaunchDescription([
        param_config_arg,
        Node(
            package='place_kfs',
            executable='place_kfs_action_server',
            name='place_kfs_action_server',
            output='screen',
            parameters=[{
                'param_config': LaunchConfiguration('param_config'),
            }],
        ),
    ])
