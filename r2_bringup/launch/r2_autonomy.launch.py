#!/usr/bin/env python3
"""
r2_autonomy.launch.py — 真机全自动两阶段启动

启动后默认只初始化系统，不执行行为树动作。正式执行通过:
  ros2 service call /bt_engine/start_autonomy r2_interfaces/srv/StartAutonomy "{region: full}"

分区域执行 region:
  full / prepare / meilin / final
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    startup_profile_arg = DeclareLaunchArgument(
        'startup_profile',
        default_value='full',
        description='Startup profile: full or minimal_meilin')

    match_config_arg = DeclareLaunchArgument(
        'match_config',
        default_value='',
        description='Match config under r2_bt/config, for example config/match_blue.json')

    default_region_arg = DeclareLaunchArgument(
        'default_region',
        default_value='full',
        description='Default region shown in /bt_engine/status before service start')

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Whether BT execution starts immediately after launch')

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Whether Odin launch should start RViz2')

    relocation_topic_arg = DeclareLaunchArgument(
        'relocation_topic',
        default_value='/odin1/relocation',
        description='Relocalized PoseStamped topic used by BT readiness and motion control')

    groot2_port_arg = DeclareLaunchArgument(
        'groot2_port',
        default_value='1667',
        description='Groot2 monitoring port, 0 disables it')

    tick_frequency_arg = DeclareLaunchArgument(
        'tick_frequency',
        default_value='100.0',
        description='Behavior tree tick frequency')

    require_map_relocalization_arg = DeclareLaunchArgument(
        'require_map_relocalization',
        default_value='true',
        description='Require relocation topic frame_id to be map before start')

    localization_timeout_arg = DeclareLaunchArgument(
        'localization_timeout_sec',
        default_value='1.0',
        description='Maximum allowed relocation pose age before start')

    tool_vid_arg = DeclareLaunchArgument(
        'tool_vid',
        default_value='4617',
        description='ARES tool USB VID, decimal')

    tool_pid_arg = DeclareLaunchArgument(
        'tool_pid',
        default_value='2',
        description='ARES tool USB PID, decimal')

    tool_timeout_arg = DeclareLaunchArgument(
        'tool_completion_timeout_ms',
        default_value='15000',
        description='ARES tool service completion timeout in milliseconds')

    full_profile = PythonExpression([
        "'", LaunchConfiguration('startup_profile'), "' == 'full'"
    ])

    odin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('coordinate_transformer'),
            'launch',
            'odin_transformer.launch.py',
        ])),
        launch_arguments={
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items(),
    )

    usb_bridge_node = Node(
        package='ares_usb',
        executable='usb_bridge_node',
        name='usb_bridge_node',
        output='screen',
    )

    tool_node = Node(
        condition=IfCondition(full_profile),
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
    )

    multi_serial_node = Node(
        package='multi_serial_sensor',
        executable='multi_serial_node',
        name='multi_serial_node',
        output='screen',
    )

    motion_action_node = Node(
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
            {'relocation_topic': LaunchConfiguration('relocation_topic')},
        ],
    )

    suspension_action_server = Node(
        package='r2_hardware',
        executable='suspension_action_server',
        name='suspension_action_server',
        output='screen',
    )

    arm_action_server = Node(
        package='r2_hardware',
        executable='arm_action_server',
        name='arm_action_server',
        output='screen',
    )

    spear_action_server = Node(
        package='r2_hardware',
        executable='spear_action_server',
        name='spear_action_server',
        output='screen',
    )

    mf_buffer_node = Node(
        package='mf_action_planner',
        executable='mf_buffer_node',
        name='mf_buffer_node',
        output='screen',
    )

    bt_engine_node = Node(
        package='r2_bt',
        executable='r2_bt_engine',
        name='bt_engine_node',
        output='screen',
        parameters=[{
            'tree_file': 'full_match.xml',
            'match_config': LaunchConfiguration('match_config'),
            'groot2_port': ParameterValue(
                LaunchConfiguration('groot2_port'), value_type=int),
            'tick_frequency': ParameterValue(
                LaunchConfiguration('tick_frequency'), value_type=float),
            'segment_topic': '/planning/segments',
            'mf_action_topic': '/mf_action_seq',
            'buffer_service': '/get_action_seq',
            'meilin_pose_topic': LaunchConfiguration('relocation_topic'),
            'autostart': ParameterValue(
                LaunchConfiguration('autostart'), value_type=bool),
            'default_region': LaunchConfiguration('default_region'),
            'require_map_relocalization': ParameterValue(
                LaunchConfiguration('require_map_relocalization'), value_type=bool),
            'localization_timeout_sec': ParameterValue(
                LaunchConfiguration('localization_timeout_sec'), value_type=float),
        }],
    )

    return LaunchDescription([
        startup_profile_arg,
        match_config_arg,
        default_region_arg,
        autostart_arg,
        launch_rviz_arg,
        relocation_topic_arg,
        groot2_port_arg,
        tick_frequency_arg,
        require_map_relocalization_arg,
        localization_timeout_arg,
        tool_vid_arg,
        tool_pid_arg,
        tool_timeout_arg,

        LogInfo(msg='=== R2 autonomy bringup: localization ==='),
        odin_launch,
        LogInfo(msg='=== R2 autonomy bringup: hardware IO ==='),
        usb_bridge_node,
        tool_node,
        multi_serial_node,
        LogInfo(msg='=== R2 autonomy bringup: action servers ==='),
        motion_action_node,
        suspension_action_server,
        arm_action_server,
        spear_action_server,
        LogInfo(msg='=== R2 autonomy bringup: planner buffer ==='),
        mf_buffer_node,
        LogInfo(msg='=== R2 autonomy bringup: BT engine gated by /bt_engine/start_autonomy ==='),
        bt_engine_node,
    ])
