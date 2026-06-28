#!/usr/bin/env python3
"""Launch the real hardware path needed to test /step_motion_control."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # USB bridge: forwards ROS float-array commands to/from the lower MCU.
        Node(
            package='ares_usb',
            executable='usb_bridge_node',
            name='usb_bridge_node',
            output='screen',
        ),

        # Real CH9344 TOF sensor reader, publishes /sensor_distances.
        Node(
            package='multi_serial_sensor',
            executable='multi_serial_node',
            name='multi_serial_node',
            output='screen',
        ),

        # Step motion server: publishes t0x0112_action and t0x0111_pid.
        # It expects /r0x0121 from ares_usb and /odin1/relocation from localization.
        Node(
            package='r2_hardware',
            executable='step_motion_action_server',
            name='step_motion_action_server',
            output='screen',
        ),
    ])
