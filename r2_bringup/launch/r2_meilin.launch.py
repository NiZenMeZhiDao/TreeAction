#!/usr/bin/env python3
"""
r2_meilin.launch.py — 真机梅林区单独启动

仅启动梅林区调试所需的节点:
  1. ares_usb (USB 桥接，下位机通信)
  2. multi_serial_sensor (TOF 距离传感器)
  3. Action Server: 底盘微调 / 主动悬挂 / 机械臂 / 矛头机构
  4. r2_bt (BT 决策引擎，加载 meilin_stage.xml)

与 r2_full.launch.py 的区别:
  - 树固定为 meilin_stage.xml（不加载准备区/竞技区）
  - 包含梅林区几何参数（grid_size / grid_origin / grasp_distance / is_red_zone）
  - 无需 match_config
  - 适合单独调试梅林区

启动示例:
  ros2 launch r2_bringup r2_meilin.launch.py                              # 默认蓝方
  ros2 launch r2_bringup r2_meilin.launch.py is_red_zone:=true             # 红方
  ros2 launch r2_bringup r2_meilin.launch.py groot2_port:=1668             # 自定义 Groot2 端口
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- BT 引擎启动参数 ----
    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value='meilin_stage.xml',
        description='行为树 XML 文件名 (位于 r2_bt/trees/ 目录下)')

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
        default_value='[2.14, 0.49]',
        description='梅林区网格原点 [x, y] (m)')

    grasp_distance_arg = DeclareLaunchArgument(
        'grasp_distance',
        default_value='0.4',
        description='抓取时车身距格子边线的距离 (m)')

    return LaunchDescription([
        tree_file_arg,
        groot2_port_arg,
        tick_frequency_arg,
        segment_topic_arg,
        mf_action_topic_arg,
        meilin_pose_topic_arg,
        is_red_zone_arg,
        grid_size_arg,
        grid_origin_arg,
        grasp_distance_arg,

        # ---- 底层: USB 桥接 (下位机通信) ----
        Node(
            package='ares_usb',
            executable='usb_bridge_node',
            name='usb_bridge_node',
            output='screen',
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
                'meilin_grid_size': LaunchConfiguration('grid_size'),
                'meilin_grid_origin': LaunchConfiguration('grid_origin'),
                'meilin_grasp_distance': LaunchConfiguration('grasp_distance'),
                'meilin_side': PythonExpression(["'red' if '", LaunchConfiguration('is_red_zone'), "' == 'true' else 'blue'"]),
            }],
        ),

        # ---- 规划: meilin_translator 已废弃，bt_engine_node 直接订阅 /mf_action_seq ----

    ])
