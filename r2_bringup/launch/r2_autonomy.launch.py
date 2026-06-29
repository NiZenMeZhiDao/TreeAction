#!/usr/bin/env python3
"""
r2_autonomy.launch.py — 真机全自动分阶段启动

启动示例:
  ros2 launch r2_bringup r2_autonomy.launch.py
  ros2 launch r2_bringup r2_autonomy.launch.py startup_profile:=prepare

startup_profile:
  full / prepare / minimal_meilin / final
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
        description='Startup profile: full, prepare, minimal_meilin, or final')

    match_config_arg = DeclareLaunchArgument(
        'match_config',
        default_value='',
        description='Legacy match JSON under r2_bt/config. Prefer param_config for new flow.')

    param_config_arg = DeclareLaunchArgument(
        'param_config',
        default_value='config/param.yaml',
        description='Area parameter YAML under r2_bt/config, for example '
                    'config/param_blue.yaml or config/param_red.yaml')

    default_region_arg = DeclareLaunchArgument(
        'default_region',
        default_value='',
        description='Default region for /bt_engine/start_autonomy. Empty follows startup_profile')

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
        default_value='60000',
        description='ARES tool service completion timeout in milliseconds')

    pick_use_synthetic_arg = DeclareLaunchArgument(
        'pick_use_synthetic',
        default_value='true',
        description='Use synthetic LiDAR scan for pick_action testing')

    pick_lidar_port_arg = DeclareLaunchArgument(
        'pick_lidar_port',
        default_value='/dev/ttyUSB0',
        description='STL-27L serial device used by pick_action when synthetic mode is false')

    pick_expected_count_arg = DeclareLaunchArgument(
        'pick_expected_count',
        default_value='3',
        description='Number of spear targets expected by pick_action recognition')

    prepare_or_full_profile = PythonExpression([
        "'", LaunchConfiguration('startup_profile'), "' in ['prepare', 'full']"
    ])
    meilin_or_full_profile = PythonExpression([
        "'", LaunchConfiguration('startup_profile'), "' in ['minimal_meilin', 'full']"
    ])
    sensor_profile = PythonExpression([
        "'", LaunchConfiguration('startup_profile'),
        "' in ['prepare', 'minimal_meilin', 'full']"
    ])
    localization_profile = PythonExpression([
        "'", LaunchConfiguration('startup_profile'),
        "' in ['prepare', 'minimal_meilin', 'full']"
    ])
    tree_file = PythonExpression([
        "'prepare_area.xml' if '", LaunchConfiguration('startup_profile'),
        "' == 'prepare' else ('meilin_stage.xml' if '",
        LaunchConfiguration('startup_profile'),
        "' == 'minimal_meilin' else ('final_area.xml' if '",
        LaunchConfiguration('startup_profile'),
        "' == 'final' else 'full_match.xml'))"
    ])
    default_region = PythonExpression([
        "'", LaunchConfiguration('default_region'), "' if '",
        LaunchConfiguration('default_region'), "' else ('prepare' if '",
        LaunchConfiguration('startup_profile'),
        "' == 'prepare' else ('meilin' if '",
        LaunchConfiguration('startup_profile'),
        "' == 'minimal_meilin' else ('final' if '",
        LaunchConfiguration('startup_profile'),
        "' == 'final' else 'full')))"
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
        condition=IfCondition(localization_profile),
    )

    usb_bridge_node = Node(
        package='ares_usb',
        executable='usb_bridge_node',
        name='usb_bridge_node',
        output='screen',
    )

    tool_node = Node(
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

    pick_action_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('pick_action'),
            'launch',
            'pick_action.launch.py',
        ])),
        launch_arguments={
            'port_name': LaunchConfiguration('pick_lidar_port'),
            'use_synthetic': LaunchConfiguration('pick_use_synthetic'),
            'expected_count': LaunchConfiguration('pick_expected_count'),
        }.items(),
        condition=IfCondition(prepare_or_full_profile),
    )

    multi_serial_node = Node(
        condition=IfCondition(sensor_profile),
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
        condition=IfCondition(meilin_or_full_profile),
        package='r2_hardware',
        executable='suspension_action_server',
        name='suspension_action_server',
        output='screen',
    )

    mf_buffer_node = Node(
        condition=IfCondition(meilin_or_full_profile),
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
            'tree_file': tree_file,
            'match_config': LaunchConfiguration('match_config'),
            'param_config': LaunchConfiguration('param_config'),
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
            'default_region': default_region,
            'require_map_relocalization': ParameterValue(
                LaunchConfiguration('require_map_relocalization'), value_type=bool),
            'localization_timeout_sec': ParameterValue(
                LaunchConfiguration('localization_timeout_sec'), value_type=float),
        }],
    )

    return LaunchDescription([
        startup_profile_arg,
        match_config_arg,
        param_config_arg,
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
        pick_use_synthetic_arg,
        pick_lidar_port_arg,
        pick_expected_count_arg,

        LogInfo(msg='=== R2 autonomy bringup: localization ===',
                condition=IfCondition(localization_profile)),
        odin_launch,
        LogInfo(msg='=== R2 autonomy bringup: hardware IO ==='),
        usb_bridge_node,
        tool_node,
        multi_serial_node,
        LogInfo(msg='=== R2 autonomy bringup: pick_action pipeline ==='),
        pick_action_launch,
        LogInfo(msg='=== R2 autonomy bringup: action servers ==='),
        motion_action_node,
        suspension_action_server,
        LogInfo(msg='=== R2 autonomy bringup: planner buffer ==='),
        mf_buffer_node,
        LogInfo(msg='=== R2 autonomy bringup: BT engine gated by /bt_engine/start_autonomy ==='),
        bt_engine_node,
    ])
