#!/usr/bin/env python3
"""
r2_full.launch.py — 真实机器人全系统启动

启动顺序:
  1. ares_usb (USB 桥接，下位机通信)
  2. ares_tool_control (/ares_tool_node/tool_action service)
  3. multi_serial_sensor (TOF 距离传感器)
  4. r2_hardware (各 Action Server)
  5. r2_bt (BT 决策引擎 + Groot2 监控)

启动示例:
  ros2 launch r2_bringup r2_full.launch.py                              # 默认 full_match
  ros2 launch r2_bringup r2_full.launch.py tree_file:=meilin_stage.xml   # 美林赛段
  ros2 launch r2_bringup r2_full.launch.py tree_file:=full_match.xml groot2_port:=1668
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- BT 引擎启动参数 ----
    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value='full_match.xml',
        description='行为树 XML 文件名 (位于 r2_bt/trees/ 目录下): '
                    'full_match.xml / meilin_stage.xml / suspension_example.xml')

    groot2_port_arg = DeclareLaunchArgument(
        'groot2_port',
        default_value='1667',
        description='Groot2 监控端口 (0 表示禁用)')

    tick_frequency_arg = DeclareLaunchArgument(
        'tick_frequency',
        default_value='100.0',
        description='行为树 Tick 频率 (Hz)')

    segment_topic_arg = DeclareLaunchArgument(
        'segment_topic',
        default_value='/planning/segments',
        description='兼容旧 Segment Plan 的 ROS2 Topic')

    mf_action_topic_arg = DeclareLaunchArgument(
        'mf_action_topic',
        default_value='/mf_action_seq',
        description='接收梅林 move/fetch Float32MultiArray 的 ROS2 Topic')

    meilin_pose_topic_arg = DeclareLaunchArgument(
        'meilin_pose_topic',
        default_value='/transformed/pose',
        description='梅林 move 使用的 map 系 base_link 定位 PoseStamped Topic')

    tool_vid_arg = DeclareLaunchArgument(
        'tool_vid',
        default_value='4617',
        description='ARES tool USB VID (decimal, default 0x1209)')

    tool_pid_arg = DeclareLaunchArgument(
        'tool_pid',
        default_value='2',
        description='ARES tool USB PID (decimal, default 0x0002)')

    tool_timeout_arg = DeclareLaunchArgument(
        'tool_completion_timeout_ms',
        default_value='15000',
        description='ARES tool service completion timeout in milliseconds')

    return LaunchDescription([
        tree_file_arg,
        groot2_port_arg,
        tick_frequency_arg,
        segment_topic_arg,
        mf_action_topic_arg,
        meilin_pose_topic_arg,
        tool_vid_arg,
        tool_pid_arg,
        tool_timeout_arg,

        # ---- 底层: USB 桥接 (下位机通信) ----
        Node(
            package='ares_usb',
            executable='usb_bridge_node',
            name='usb_bridge_node',
            output='screen',
        ),

        # ---- 底层: ARES tool service (PrepareArea 等待 /ares_tool_node/tool_action) ----
        Node(
            package='ares_tool_control',
            executable='tool_node',
            name='ares_tool_node',
            output='screen',
            parameters=[{
                'vid': ParameterValue(LaunchConfiguration('tool_vid'), value_type=int),
                'pid': ParameterValue(LaunchConfiguration('tool_pid'), value_type=int),
                'completion_timeout_ms': ParameterValue(
                    LaunchConfiguration('tool_completion_timeout_ms'),
                    value_type=int),
            }],
        ),

        # ---- 底层: 距离传感器 ----
        Node(
            package='multi_serial_sensor',
            executable='multi_serial_node',
            name='multi_serial_node',
            output='screen',
        ),

        # ---- Action Server: 底盘微调 ----
        Node(
            package='action_of_motion',
            executable='motion_action_node',
            name='motion_action_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('action_of_motion'),
                    'config',
                    'param.yaml',
                ]),
            ],
        ),

        # ---- Action Server: 主动悬挂 ----
        Node(
            package='r2_hardware',
            executable='suspension_action_server',
            name='suspension_action_server',
            output='screen',
        ),

        # ---- Action Server: 机械臂 ----
        Node(
            package='r2_hardware',
            executable='arm_action_server',
            name='arm_action_server',
            output='screen',
        ),

        # ---- Action Server: 矛头机构 ----
        Node(
            package='r2_hardware',
            executable='spear_action_server',
            name='spear_action_server',
            output='screen',
        ),

        # ---- 上层: BT 决策引擎 ----
        Node(
            package='r2_bt',
            executable='r2_bt_engine',
            name='bt_engine_node',
            output='screen',
            parameters=[{
                'tree_file': LaunchConfiguration('tree_file'),
                'groot2_port': LaunchConfiguration('groot2_port'),
                'tick_frequency': LaunchConfiguration('tick_frequency'),
                'segment_topic': LaunchConfiguration('segment_topic'),
                'mf_action_topic': LaunchConfiguration('mf_action_topic'),
                'meilin_pose_topic': LaunchConfiguration('meilin_pose_topic'),
            }],
        ),

        # ---- 规划: meilin_translator 已废弃，bt_engine_node 直接订阅 /mf_action_seq ----

    ])
