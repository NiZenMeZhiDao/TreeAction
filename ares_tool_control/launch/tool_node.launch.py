from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vid_arg = DeclareLaunchArgument("vid", default_value="4617")  # 0x1209
    pid_arg = DeclareLaunchArgument("pid", default_value="2")     # 0x0002
    timeout_arg = DeclareLaunchArgument("completion_timeout_ms", default_value="15000")

    return LaunchDescription([
        vid_arg,
        pid_arg,
        timeout_arg,
        Node(
            package="ares_tool_control",
            executable="tool_node",
            name="ares_tool_node",
            output="screen",
            parameters=[{
                "vid": ParameterValue(LaunchConfiguration("vid"), value_type=int),
                "pid": ParameterValue(LaunchConfiguration("pid"), value_type=int),
                "completion_timeout_ms": ParameterValue(
                    LaunchConfiguration("completion_timeout_ms"), value_type=int),
            }],
        ),
    ])
