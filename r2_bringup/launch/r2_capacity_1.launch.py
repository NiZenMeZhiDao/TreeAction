#!/usr/bin/env python3
"""
r2_capacity_1.launch.py - 前两区独立启动

固定以 startup_profile:=capacity_1 调用 r2_autonomy.launch.py，只执行
PrepareArea -> WaitSeconds -> MeilinArea，不进入 FinalArea。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_config_arg = DeclareLaunchArgument(
        'param_config',
        default_value='config/param.yaml',
        description='Area parameter YAML under r2_bt/config')

    default_region_arg = DeclareLaunchArgument(
        'default_region',
        default_value='capacity_1',
        description='Default region for /bt_engine/start_autonomy')

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

    meilin_motion_mode_arg = DeclareLaunchArgument(
        'meilin_motion_mode',
        default_value='single_axis',
        description='Meilin motion mode: single_axis or omni')

    prepare_to_meilin_wait_arg = DeclareLaunchArgument(
        'prepare_to_meilin_wait_sec',
        default_value='1.0',
        description='Wait time between PrepareArea success and MeilinArea start; 0 disables it')

    launch_meilin_web_arg = DeclareLaunchArgument(
        'launch_meilin_web',
        default_value='true',
        description='Start rosbridge and DFS planner for the Meilin web UI')

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='WebSocket port used by mf_manager.html/path_editor.html')

    mf_planner_params_arg = DeclareLaunchArgument(
        'mf_planner_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mf_action_planner'),
            'config',
            'para.yaml',
        ]),
        description='Parameter YAML for mf_action_planner dfs_planner_node')

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

    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('r2_bringup'),
            'launch',
            'r2_autonomy.launch.py',
        ])),
        launch_arguments={
            'startup_profile': 'capacity_1',
            'param_config': LaunchConfiguration('param_config'),
            'default_region': LaunchConfiguration('default_region'),
            'autostart': LaunchConfiguration('autostart'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'relocation_topic': LaunchConfiguration('relocation_topic'),
            'groot2_port': LaunchConfiguration('groot2_port'),
            'tick_frequency': LaunchConfiguration('tick_frequency'),
            'require_map_relocalization': LaunchConfiguration('require_map_relocalization'),
            'localization_timeout_sec': LaunchConfiguration('localization_timeout_sec'),
            'meilin_motion_mode': LaunchConfiguration('meilin_motion_mode'),
            'prepare_to_meilin_wait_sec': LaunchConfiguration('prepare_to_meilin_wait_sec'),
            'launch_meilin_web': LaunchConfiguration('launch_meilin_web'),
            'rosbridge_port': LaunchConfiguration('rosbridge_port'),
            'mf_planner_params_file': LaunchConfiguration('mf_planner_params_file'),
            'tool_vid': LaunchConfiguration('tool_vid'),
            'tool_pid': LaunchConfiguration('tool_pid'),
            'tool_completion_timeout_ms': LaunchConfiguration('tool_completion_timeout_ms'),
            'pick_use_synthetic': LaunchConfiguration('pick_use_synthetic'),
            'pick_lidar_port': LaunchConfiguration('pick_lidar_port'),
            'pick_expected_count': LaunchConfiguration('pick_expected_count'),
        }.items(),
    )

    return LaunchDescription([
        param_config_arg,
        default_region_arg,
        autostart_arg,
        launch_rviz_arg,
        relocation_topic_arg,
        groot2_port_arg,
        tick_frequency_arg,
        require_map_relocalization_arg,
        localization_timeout_arg,
        meilin_motion_mode_arg,
        prepare_to_meilin_wait_arg,
        launch_meilin_web_arg,
        rosbridge_port_arg,
        mf_planner_params_arg,
        tool_vid_arg,
        tool_pid_arg,
        tool_timeout_arg,
        pick_use_synthetic_arg,
        pick_lidar_port_arg,
        pick_expected_count_arg,
        autonomy_launch,
    ])
