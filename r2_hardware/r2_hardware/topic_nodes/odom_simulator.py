#!/usr/bin/env python3
"""
里程计模拟器

接收 /cmd_vel (Float32MultiArray)，积分生成里程计并发布 /odom_world。
单向模式: vx=前进速度, vy=侧向速度
全向模式: vx=全局x速度, vy=全局y速度
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class OdomSimulator(Node):
    """Simple omni-directional base simulator."""

    def __init__(self) -> None:
        super().__init__("odom_simulator")

        self.declare_parameter("odom_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("motion_type", "omnidirectional")
        self.declare_parameter("map_origin", [3.2, 1.2, 0.0])
        self.declare_parameter("grid_resolution", 1.2)
        self.declare_parameter("start_row", -1)
        self.declare_parameter("start_col", 0)
        self.declare_parameter("cmd_vel", "/t0x0101_pid")
        self.declare_parameter("odom_world", "/odin1/relocation")

        odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        publish_rate = max(1.0, self.get_parameter("publish_rate").value)
        motion_type = (
            self.get_parameter("motion_type").get_parameter_value().string_value
        )

        self._odom_frame = odom_frame or "map"
        self._base_frame = base_frame or "base_link"
        self._motion_type = motion_type or "omnidirectional"
        self._dt = 1.0 / publish_rate

        map_origin_raw = self.get_parameter("map_origin").value
        grid_resolution = self.get_parameter("grid_resolution").value
        start_row = self.get_parameter("start_row").value
        start_col = self.get_parameter("start_col").value

        # 梅林区仿真：初始位置设为 grid origin (1.2, 1.2) = grid (0,0)
        grid_res = self.get_parameter("grid_resolution").value
        origin = self.get_parameter("map_origin").value
        start_x = origin[0] if len(origin) >= 1 else 1.2
        start_y = origin[1] if len(origin) >= 2 else 1.2

        self._pose_x = start_x
        self._pose_y = start_y
        self._pose_z = 0.0
        self._yaw = 0.0
        self._vel_cmd = Float32MultiArray()
        self._last_update: Optional[Time] = None

        self._cmd_sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter("cmd_vel").value,
            self._cmd_callback,
            10,
        )
        self._initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initial_pose_callback,
            10,
        )
        self._odom_pub = self.create_publisher(
            PoseStamped, self.get_parameter("odom_world").value, 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(self._dt, self._publish_odometry)

        self.get_logger().info(
            f"Odom simulator started. Publishing {publish_rate:.1f} Hz on "
            f'{self.get_parameter("odom_world").value}, '
            f"motion_type={self._motion_type}"
        )
        self.get_logger().info(
            f"Publishing TF transform: {self._odom_frame} -> {self._base_frame}"
        )
        self.get_logger().info(
            f"Initial position: [{self._pose_x:.2f}, {self._pose_y:.2f}] "
            f"(grid [{start_row}][{start_col}])"
        )

    def _cmd_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 3:
            self._vel_cmd = msg

    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self._pose_x = pose.position.x
        self._pose_y = pose.position.y
        self._pose_z = pose.position.z

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)

        if norm > 0.0:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self._yaw = math.atan2(siny_cosp, cosy_cosp)
        else:
            self._yaw = 0.0

        self.get_logger().info(
            f"Received /initialpose: ({self._pose_x:.2f}, {self._pose_y:.2f}), "
            f"yaw={math.degrees(self._yaw):.1f} deg"
        )

    def _publish_odometry(self) -> None:
        now = self.get_clock().now().to_msg()

        dt = self._dt

        vx_cmd = (
            float(self._vel_cmd.data[0]) if len(self._vel_cmd.data) >= 1 else 0.0
        )
        vy_cmd = (
            float(self._vel_cmd.data[1]) if len(self._vel_cmd.data) >= 2 else 0.0
        )
        wz = (
            float(self._vel_cmd.data[2]) if len(self._vel_cmd.data) >= 3 else 0.0
        )

        if self._motion_type == "omnidirectional":
            vx_global = vx_cmd
            vy_global = vy_cmd
        else:
            yaw = self._yaw
            vx_global = vx_cmd * math.cos(yaw) - vy_cmd * math.sin(yaw)
            vy_global = vx_cmd * math.sin(yaw) + vy_cmd * math.cos(yaw)

        self._pose_x += vx_global * dt
        self._pose_y += vy_global * dt
        self._pose_z = 0.0
        self._yaw += wz * dt
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        half_yaw = self._yaw * 0.5

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self._odom_frame
        pose_msg.pose.position.x = self._pose_x
        pose_msg.pose.position.y = self._pose_y
        pose_msg.pose.position.z = self._pose_z
        pose_msg.pose.orientation.z = math.sin(half_yaw)
        pose_msg.pose.orientation.w = math.cos(half_yaw)

        self._odom_pub.publish(pose_msg)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = self._pose_x
        t.transform.translation.y = self._pose_y
        t.transform.translation.z = self._pose_z
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        self._tf_broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
