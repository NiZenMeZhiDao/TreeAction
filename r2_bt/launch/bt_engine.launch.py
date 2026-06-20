#!/usr/bin/env python3
"""
bt_engine.launch.py — BT 决策引擎启动

启动示例:
  # 红方全场
  ros2 launch r2_bt bt_engine.launch.py match_config:=config/match_red.json

  # 蓝方全场
  ros2 launch r2_bt bt_engine.launch.py match_config:=config/match_blue.json

  # 美林赛段独立调试（不需要 match_config）
  ros2 launch r2_bt bt_engine.launch.py tree_file:=meilin_stage.xml

  # 自定义 Groot2 端口
  ros2 launch r2_bt bt_engine.launch.py match_config:=config/match_red.json groot2_port:=1668
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 声明启动参数 ----
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
                    '留空则跳过启动加载，PrepareArea/FinalArea 需自行设置 blackboard 变量。')

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

    return LaunchDescription([
        tree_file_arg,
        match_config_arg,
        groot2_port_arg,
        tick_frequency_arg,
        segment_topic_arg,
        mf_action_topic_arg,
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
            }],
        ),
    ])
