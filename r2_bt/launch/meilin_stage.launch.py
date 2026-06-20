#!/usr/bin/env python3
"""
meilin_stage.launch.py — 梅林赛段独立启动（BT 引擎）

启动梅林区专用调试环境:
  - BT 决策引擎（加载 meilin_stage.xml）
  - BT 引擎直接订阅 /mf_action_seq

启动示例:
  # 蓝方梅林区（默认）
  ros2 launch r2_bt meilin_stage.launch.py

  # 红方梅林区
  ros2 launch r2_bt meilin_stage.launch.py is_red_zone:=true

  # 自定义 Groot2 端口
  ros2 launch r2_bt meilin_stage.launch.py groot2_port:=1668
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 启动参数 ----
    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value='meilin_stage.xml',
        description='行为树 XML 文件名 (位于 r2_bt/trees/ 目录下)')

    match_config_arg = DeclareLaunchArgument(
        'match_config',
        default_value='',
        description='比赛配置文件（可选）。当前梅林段不再从配置加载静态 segment。')

    groot2_port_arg = DeclareLaunchArgument(
        'groot2_port',
        default_value='1667',
        description='Groot2 监控端口（0 表示禁用）')

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

    is_red_zone_arg = DeclareLaunchArgument(
        'is_red_zone',
        default_value='false',
        description='是否为红方区域（红方高度地图不同）')

    grid_size_arg = DeclareLaunchArgument(
        'grid_size',
        default_value='1.2',
        description='梅林区网格大小 (m)')

    grid_origin_arg = DeclareLaunchArgument(
        'grid_origin',
        default_value='[1.2, 1.2]',
        description='梅林区网格原点 [x, y] (m)')

    grasp_distance_arg = DeclareLaunchArgument(
        'grasp_distance',
        default_value='0.4',
        description='抓取时车身距格子边线的距离 (m)')

    return LaunchDescription([
        tree_file_arg,
        match_config_arg,
        groot2_port_arg,
        tick_frequency_arg,
        segment_topic_arg,
        mf_action_topic_arg,
        is_red_zone_arg,
        grid_size_arg,
        grid_origin_arg,
        grasp_distance_arg,

        # ---- BT 决策引擎 ----
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
                'meilin_grid_size': LaunchConfiguration('grid_size'),
                'meilin_grid_origin': LaunchConfiguration('grid_origin'),
                'meilin_grasp_distance': LaunchConfiguration('grasp_distance'),
                'meilin_side': PythonExpression(["'red' if '", LaunchConfiguration('is_red_zone'), "' == 'true' else 'blue'"]),
            }],
        ),

        # ---- 规划: meilin_translator 已废弃，bt_engine_node 直接订阅 /mf_action_seq ----

    ])
