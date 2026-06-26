#!/usr/bin/env python3
"""
r2_sim.launch.py — 仿真环境启动

启动顺序:
  1. r2_hardware (里程计模拟器 + 各 Action Server)
  2. r2_bt (BT 决策引擎)

启动示例:
  ros2 launch r2_bringup r2_sim.launch.py match_config:=config/match_red.json    # 红方全场
  ros2 launch r2_bringup r2_sim.launch.py match_config:=config/match_blue.json   # 蓝方全场
  ros2 launch r2_bringup r2_sim.launch.py tree_file:=meilin_stage.xml            # 梅林调试
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- BT 引擎启动参数 ----
    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value='full_match.xml',
        description='行为树 XML 文件名 (位于 r2_bt/trees/ 目录下): '
                    'full_match.xml / meilin_stage.xml')

    match_config_arg = DeclareLaunchArgument(
        'match_config',
        default_value='',
        description='比赛配置文件 (位于 r2_bt/config/ 目录下): '
                    'config/match_red.json / config/match_blue.json。'
                    '留空则跳过启动加载。')

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

    return LaunchDescription([
        tree_file_arg,
        match_config_arg,
        groot2_port_arg,
        tick_frequency_arg,
        segment_topic_arg,
        mf_action_topic_arg,
        meilin_pose_topic_arg,

        # ---- 仿真: 里程计模拟器 ----
        Node(
            package='r2_hardware',
            executable='odom_simulator',
            name='odom_simulator',
            output='screen',
        ),

        # ---- Action Server: 底盘微调 ----
        Node(
            package='r2_hardware',
            executable='motion_action_node',
            name='motion_action_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('r2_hardware'),
                    'config',
                    'motion_param.yaml',
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
                'match_config': LaunchConfiguration('match_config'),
                'groot2_port': LaunchConfiguration('groot2_port'),
                'tick_frequency': LaunchConfiguration('tick_frequency'),
                'segment_topic': LaunchConfiguration('segment_topic'),
                'mf_action_topic': LaunchConfiguration('mf_action_topic'),
                'meilin_pose_topic': LaunchConfiguration('meilin_pose_topic'),
            }],
        ),

        # ---- 规划: meilin_translator 已废弃，bt_engine_node 直接订阅 /mf_action_seq ----

    ])
