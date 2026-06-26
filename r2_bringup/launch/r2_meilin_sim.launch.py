#!/usr/bin/env python3
"""
r2_meilin_sim.launch.py — 梅林赛段仿真启动

启动梅林区完整的仿真环境:
  1. r2_hardware: 里程计模拟器 + 所有 Action Server (底盘/悬挂/机械臂/矛头)
  2. r2_bt: BT 决策引擎（加载 meilin_stage.xml）
  3. mf_action_planner 或 Web 直接发布 /mf_action_seq

与全场 r2_sim.launch.py 的区别:
  - 树固定为 meilin_stage.xml（不加载准备区/竞技区）
  - BT 直接订阅 /mf_action_seq
  - 无需 match_config
  - 适合单独调试梅林区路径规划

启动示例:
  # 蓝方梅林区（默认）
  ros2 launch r2_bringup r2_meilin_sim.launch.py

  # 红方梅林区
  ros2 launch r2_bringup r2_meilin_sim.launch.py is_red_zone:=true

  # 自定义 Groot2 端口
  ros2 launch r2_bringup r2_meilin_sim.launch.py groot2_port:=1668
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- BT 引擎参数 ----
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

    meilin_pose_topic_arg = DeclareLaunchArgument(
        'meilin_pose_topic',
        default_value='/transformed/pose',
        description='梅林 move 使用的 map 系 base_link 定位 PoseStamped Topic')

    # ---- 梅林区几何参数（传给 BT 引擎）----
    is_red_zone_arg = DeclareLaunchArgument(
        'is_red_zone',
        default_value='false',
        description='是否为红方区域')

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
        meilin_pose_topic_arg,
        is_red_zone_arg,
        grid_size_arg,
        grid_origin_arg,
        grasp_distance_arg,

        # ============================================================
        #  仿真硬件层: r2_hardware
        # ============================================================

        # ---- 里程计模拟器 ----
        Node(
            package='r2_hardware',
            executable='odom_simulator',
            name='odom_simulator',
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

        # ============================================================
        #  决策层: r2_bt
        # ============================================================

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
                'meilin_pose_topic': LaunchConfiguration('meilin_pose_topic'),
                'meilin_grid_size': LaunchConfiguration('grid_size'),
                'meilin_grid_origin': LaunchConfiguration('grid_origin'),
                'meilin_grasp_distance': LaunchConfiguration('grasp_distance'),
                'meilin_side': PythonExpression(["'red' if '", LaunchConfiguration('is_red_zone'), "' == 'true' else 'blue'"]),
            }],
        ),

        # ---- 规划: meilin_translator 已废弃，bt_engine_node 直接订阅 /mf_action_seq ----

    ])
