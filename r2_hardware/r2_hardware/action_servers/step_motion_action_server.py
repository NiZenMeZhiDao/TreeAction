#!/usr/bin/env python3
"""
上下台阶 + 阶段速度耦合 Action Server
状态机参考 suspension_action_server.py；额外在控制循环里根据当前阶段
发布底盘速度，并用进入 action 时的位置/yaw 做横向和角度修正。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from r2_interfaces.action import StepMotionControl
from std_msgs.msg import Float32MultiArray, Int32
from enum import Enum
import collections
import math
import time
from rclpy.executors import MultiThreadedExecutor


class State(Enum):
    IDLE = 0
    UP_1_PREPARE = 10
    UP_2_LIFT = 11
    UP_3_FRONT_DOCK = 12
    UP_4_RETRACT_FRONT = 13
    UP_5_FRONT_LAND = 14
    UP_6_SIDE_DOCK_RETRACT_REAR = 15
    UP_7_REAR_LAND = 16
    UP_8_RECOVER = 17

    DOWN_1_PREPARE = 20
    DOWN_2_FRONT_HOVER_LAND = 21
    DOWN_3_REAR_HOVER_LAND = 22
    DOWN_4_RECOVERY = 23


class Direction(Enum):
    FORWARD = 0
    LEFT = 1
    RIGHT = 2
    BACKWARD = 3


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def map_velocity_to_body(vx_map, vy_map, yaw_rad):
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    vx_body = cos_yaw * vx_map + sin_yaw * vy_map
    vy_body = -sin_yaw * vx_map + cos_yaw * vy_map
    return vx_body, vy_body


class StepMotionActionServer(Node):
    def __init__(self):
        super().__init__('step_motion_action_server')

        # 参数配置
        self.DEFAULT_LIFT_LOW = 205.0
        self.DEFAULT_LIFT_HIGH = 410.0
        self.H_LIFT_LOW = self.DEFAULT_LIFT_LOW
        self.H_LIFT_HIGH = self.DEFAULT_LIFT_HIGH
        self.H_INIT = 20.0
        self.HEIGHT_TOLERANCE = 20.0
        self.down_high = 5
        # 状态变量
        self.current_state = State.IDLE
        self.target_height = 0.0
        self.current_direction = Direction.FORWARD

        self.distances_raw = [math.nan] * 8
        self.pe_switches_raw = [0] * 4
        self.wheel_heights_current = [0.0] * 4

        self.distance_filtered = [math.nan] * 8
        self.pe_switches_filtered = [0] * 4

        self.wheel_heights_target = [self.H_INIT] * 4
        self.v_distances_idx = [0.0] * 6

        self.v_wheels_idx = [0, 1, 2, 3]
        self.v_pe_idx = [0, 1, 3, 2]

        # 滤波器
        self.distance_buffers = [collections.deque(maxlen=5) for _ in range(8)]
        self.pe_debounce_counters = [0] * 4
        self.pe_last_states = [0] * 4

        self._height_latched = False
        self._stable_counters = collections.defaultdict(int)
        self._idle_debug_counter = 0
        self._active_goal = False
        self._active_mode = 0
        self._requested_height = 0.0
        self._known_step_height = False
        self._sequence_done = False
        self._step_motion_active = False
        self._step_motion_enabled = False
        self._step_pose_correction_enabled = False
        self._step_speed_scale = 1.0
        self._step_start_pose = None
        self._latest_pose = None
        self._last_velocity_cmd = [0.0, 0.0, 0.0]
        self._load_step_motion_parameters()

        # 传感器订阅
        self.sub_sensor_dist = self.create_subscription(
            Float32MultiArray, 'sensor_distances', self.dist_cb, 10)
        self.sub_r0x0201 = self.create_subscription(
            Float32MultiArray, 'r0x0121', self.hw_status_cb, 10)
        self.sub_pose = self.create_subscription(
            PoseStamped, self.step_relocation_topic, self.pose_cb, 10)

        # 控制发布
        self.pub_action = self.create_publisher(Float32MultiArray, 't0x0112_action', 10)
        self.pub_velocity = self.create_publisher(
            Float32MultiArray, self.step_velocity_topic, 10)
        self.pub_state = self.create_publisher(Int32, 'current_state', 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            StepMotionControl,
            'step_motion_control',
            goal_callback=self._step_goal_callback,
            cancel_callback=self._step_cancel_callback,
            execute_callback=self._execute_step_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # 控制循环 (100Hz)
        self.delay_timer = self.create_timer(0.2, self._start_control_loop)
        self.get_logger().info(
            'Step Motion Action Server initialized.'
        )

    def _load_step_motion_parameters(self):
        self.declare_parameter('step_motion.relocation_topic', '/odin1/relocation')
        self.declare_parameter('step_motion.velocity_topic', 't0x0111_pid')
        self.declare_parameter('step_motion.default_speed_scale', 1.0)
        self.declare_parameter('step_motion.cross_kp', 2.0)
        self.declare_parameter('step_motion.yaw_kp', 1.0)
        self.declare_parameter('step_motion.max_cross_speed', 0.35)
        self.declare_parameter('step_motion.max_yaw_speed', 0.45)

        self._stage_speed_params = {
            State.UP_1_PREPARE: 'step_motion.stage_speed.up_1_prepare',
            State.UP_2_LIFT: 'step_motion.stage_speed.up_2_lift',
            State.UP_3_FRONT_DOCK: 'step_motion.stage_speed.up_3_front_dock',
            State.UP_4_RETRACT_FRONT: 'step_motion.stage_speed.up_4_retract_front',
            State.UP_5_FRONT_LAND: 'step_motion.stage_speed.up_5_front_land',
            State.UP_6_SIDE_DOCK_RETRACT_REAR:
                'step_motion.stage_speed.up_6_side_dock_retract_rear',
            State.UP_7_REAR_LAND: 'step_motion.stage_speed.up_7_rear_land',
            State.UP_8_RECOVER: 'step_motion.stage_speed.up_8_recover',
            State.DOWN_1_PREPARE: 'step_motion.stage_speed.down_1_prepare',
            State.DOWN_2_FRONT_HOVER_LAND:
                'step_motion.stage_speed.down_2_front_hover_land',
            State.DOWN_3_REAR_HOVER_LAND:
                'step_motion.stage_speed.down_3_rear_hover_land',
            State.DOWN_4_RECOVERY: 'step_motion.stage_speed.down_4_recovery',
        }
        default_stage_speeds = {
            State.UP_1_PREPARE: 0.0,
            State.UP_2_LIFT: 0.0,
            State.UP_3_FRONT_DOCK: 0.20,
            State.UP_4_RETRACT_FRONT: 0.0,
            State.UP_5_FRONT_LAND: 0.08,
            State.UP_6_SIDE_DOCK_RETRACT_REAR: 0.18,
            State.UP_7_REAR_LAND: 0.08,
            State.UP_8_RECOVER: 0.0,
            State.DOWN_1_PREPARE: 0.0,
            State.DOWN_2_FRONT_HOVER_LAND: 0.10,
            State.DOWN_3_REAR_HOVER_LAND: 0.16,
            State.DOWN_4_RECOVERY: 0.0,
        }
        for state, param_name in self._stage_speed_params.items():
            self.declare_parameter(param_name, default_stage_speeds[state])

        self.step_relocation_topic = self.get_parameter(
            'step_motion.relocation_topic').value
        self.step_velocity_topic = self.get_parameter(
            'step_motion.velocity_topic').value

    def _start_control_loop(self):
        self.delay_timer.cancel()
        self.control_timer = self.create_timer(0.01, self.control_loop)

    # ---- Action Server 回调 ----
    def _step_goal_callback(self, goal_request):
        if self._active_goal:
            self.get_logger().warning('Rejecting step motion goal: another goal is active.')
            return GoalResponse.REJECT
        mode = goal_request.mode
        direction = goal_request.direction
        if mode not in (0, 1, 2):
            return GoalResponse.REJECT
        if direction not in (0, 1, 2, 3):
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.height)):
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.timeout_sec)):
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.speed_scale)):
            return GoalResponse.REJECT
        self.get_logger().info(
            f'Accepting step motion goal: mode={mode}, direction={direction}, '
            f'height={goal_request.height:.1f}, '
            f'motion={goal_request.enable_stage_motion}, '
            f'correction={goal_request.enable_pose_correction}, '
            f'speed_scale={goal_request.speed_scale:.2f}'
        )
        return GoalResponse.ACCEPT

    def _step_cancel_callback(self, goal_handle):
        self.get_logger().info('Step motion goal cancelled, returning to IDLE.')
        self._reset_active_step_state()
        self._publish_velocity(0.0, 0.0, 0.0)
        return CancelResponse.ACCEPT

    def _execute_step_callback(self, goal_handle: ServerGoalHandle):
        mode = int(goal_handle.request.mode)
        direction = int(goal_handle.request.direction)
        height = float(goal_handle.request.height)
        timeout_sec = float(goal_handle.request.timeout_sec)

        if mode == 0:
            if height > 0.0:
                mode = 1
            elif height < 0.0:
                mode = 2

        requested_step_height = abs(height)
        self._known_step_height = requested_step_height > 0.0
        if self._known_step_height:
            self._requested_height = self._map_step_height_to_prepare_height(
                requested_step_height)
            self.H_LIFT_LOW = self._requested_height
            self.H_LIFT_HIGH = self._requested_height
        else:
            self._requested_height = 0.0
            self.H_LIFT_LOW = self.DEFAULT_LIFT_LOW
            self.H_LIFT_HIGH = self.DEFAULT_LIFT_HIGH

        self.current_direction = Direction(direction)
        self.current_state = {
            1: State.UP_1_PREPARE,
            2: State.DOWN_1_PREPARE,
        }.get(mode, State.IDLE)

        default_speed_scale = float(self.get_parameter(
            'step_motion.default_speed_scale').value)
        requested_speed_scale = float(goal_handle.request.speed_scale)
        self._step_speed_scale = (
            requested_speed_scale
            if requested_speed_scale > 0.0
            else default_speed_scale
        )
        self._step_motion_enabled = bool(goal_handle.request.enable_stage_motion)
        self._step_pose_correction_enabled = bool(
            goal_handle.request.enable_pose_correction)
        self._step_start_pose = self._latest_pose
        self._step_motion_active = True
        self._active_goal = True
        self._active_mode = mode
        self._sequence_done = False
        self._stable_counters.clear()
        self._height_latched = False
        start_time = self.get_clock().now()

        feedback_msg = StepMotionControl.Feedback()

        while rclpy.ok():
            elapsed_sec = (self.get_clock().now() - start_time).nanoseconds / 1e9

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._reset_active_step_state()
                self._publish_velocity(0.0, 0.0, 0.0)
                return StepMotionControl.Result(
                    success=False,
                    message='Cancelled',
                    final_state=self.current_state.value,
                    elapsed_sec=float(elapsed_sec),
                )

            feedback_msg.current_state = self.current_state.value
            feedback_msg.elapsed_sec = float(elapsed_sec)
            feedback_msg.cmd_vx = float(self._last_velocity_cmd[0])
            feedback_msg.cmd_vy = float(self._last_velocity_cmd[1])
            feedback_msg.cmd_wz = float(self._last_velocity_cmd[2])
            if self._latest_pose is not None:
                feedback_msg.current_x = float(self._latest_pose[0])
                feedback_msg.current_y = float(self._latest_pose[1])
                feedback_msg.current_yaw_deg = math.degrees(self._latest_pose[2])
            feedback_msg.wheel_heights_current = [
                float(h) for h in self.wheel_heights_current
            ]
            feedback_msg.wheel_heights_target = [
                float(h) for h in self.wheel_heights_target
            ]
            goal_handle.publish_feedback(feedback_msg)

            if timeout_sec > 0.0 and elapsed_sec > timeout_sec:
                goal_handle.abort()
                self._reset_active_step_state()
                self._publish_velocity(0.0, 0.0, 0.0)
                return StepMotionControl.Result(
                    success=False,
                    message='Timeout',
                    final_state=self.current_state.value,
                    elapsed_sec=float(elapsed_sec),
                )

            if self._sequence_done:
                self._sequence_done = False
                goal_handle.succeed()
                self._active_goal = False
                self._active_mode = 0
                self._requested_height = 0.0
                self._known_step_height = False
                self._step_motion_active = False
                return StepMotionControl.Result(
                    success=True,
                    message='Step sequence complete',
                    final_state=self.current_state.value,
                    elapsed_sec=float(elapsed_sec),
                )

            time.sleep(0.01)

        goal_handle.abort()
        self._reset_active_step_state()
        return StepMotionControl.Result(
            success=False,
            message='Node shutdown',
            final_state=self.current_state.value,
            elapsed_sec=float((self.get_clock().now() - start_time).nanoseconds / 1e9),
        )

    # ---- 传感器回调 ----
    def dist_cb(self, msg):
        if len(msg.data) >= 8:
            for i in range(8):
                distance = float(msg.data[i])
                self.distances_raw[i] = distance
                if math.isfinite(distance) and distance > 0.0:
                    self.distance_buffers[i].append(distance)
                    self.distance_filtered[i] = sum(self.distance_buffers[i]) / len(self.distance_buffers[i])
                else:
                    self.distance_buffers[i].clear()
                    self.distance_filtered[i] = math.nan

    def hw_status_cb(self, msg):
        if len(msg.data) >= 12:
            for i in range(4):
                current_pe = int(msg.data[i])
                if current_pe != self.pe_last_states[i]:
                    self.pe_debounce_counters[i] += 1
                    if self.pe_debounce_counters[i] >= 2:
                        self.pe_switches_filtered[i] = current_pe
                        self.pe_last_states[i] = current_pe
                        self.pe_debounce_counters[i] = 0
                else:
                    self.pe_debounce_counters[i] = 0

            for i in range(4):
                self.wheel_heights_current[i] = msg.data[4 + i]

    def pose_cb(self, msg):
        pose = msg.pose
        self._latest_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )

    # ---- 辅助 ----
    def _is_stable(self, condition, key, threshold=5):
        if condition:
            if self._stable_counters[key] < threshold:
                self._stable_counters[key] += 1
            if self._stable_counters[key] >= threshold:
                return True
        else:
            self._stable_counters[key] = 0
        return False

    def _map_step_height_to_prepare_height(self, step_height):
        """Map a known stair height to the suspension preparation height."""
        if step_height <= 300.0:
            return 205.0
        return 410.0

    def _format_distance(self, value):
        if math.isfinite(value):
            return f'{value:.1f}'
        return 'NaN'

    def update_virtual_mapping(self):
        # Legacy wheel IDs were remapped on the hardware side:
        # old 1/2/3/4 -> new 4/2/1/3. Indices below are zero-based.
        if self.current_direction == Direction.FORWARD:
            self.v_wheels_idx = [0, 1, 3, 2]
            self.v_pe_idx = [0, 3, 1, 2]
            self.v_distances_idx = [0, 1, 5, 4]
        elif self.current_direction == Direction.LEFT:
            self.v_wheels_idx = [3, 0, 2, 1]
            self.v_pe_idx = [1, 0, 2, 3]
            self.v_distances_idx = [2, 3, 7, 6]
        elif self.current_direction == Direction.RIGHT:
            self.v_wheels_idx = [1, 2, 0, 3]
            self.v_pe_idx = [3, 2, 0, 1]
            self.v_distances_idx = [6, 7, 3, 2]
        elif self.current_direction == Direction.BACKWARD:
            self.v_wheels_idx = [3, 2, 0, 1]
            self.v_pe_idx = [2, 3, 1, 0]
            self.v_distances_idx = [4, 5, 1, 0]

    def check_height_reached(self, virtual_indices, target_h):
        for v_idx in virtual_indices:
            phys_idx = self.v_wheels_idx[v_idx]
            if abs(self.wheel_heights_current[phys_idx] - target_h) > self.HEIGHT_TOLERANCE:
                return False
        return True

    def _reset_active_step_state(self):
        self._active_goal = False
        self._active_mode = 0
        self._requested_height = 0.0
        self._known_step_height = False
        self._sequence_done = False
        self._step_motion_active = False
        self._step_motion_enabled = False
        self._step_pose_correction_enabled = False
        self._step_start_pose = None
        self.current_state = State.IDLE
        self.wheel_heights_target = [self.H_INIT] * 4
        self._stable_counters.clear()

    def _set_v_wheel_height(self, v_indices, height):
        for v_idx in v_indices:
            phys_idx = self.v_wheels_idx[v_idx]
            self.wheel_heights_target[phys_idx] = float(height)

    def _get_v_pe(self, v_idx):
        return self.pe_switches_filtered[self.v_pe_idx[v_idx]]

    def _get_v_distance(self, v_idx):
        """获取虚拟方向上的 TOF 距离，返回 NaN 表示无效值"""
        val = self.distance_filtered[self.v_distances_idx[v_idx]]
        if not math.isfinite(val) or val <= 0.0:
            return math.nan
        return val

    def _get_v_distance_safe(self, v_idx, default=999.0):
        """获取虚拟方向上的 TOF 距离，NaN 时返回默认值。

        注意: _get_v_distance_safe 仅用于状态机内部高度/距离判断，
        NaN 时将比较委托给默认值。默认值选择需匹配比较语义:
        - v < threshold 判断: 使用大默认值 (999.0)，NaN 时不会误触发
        - v > threshold 判断: 使用小默认值 (0.0)，NaN 时不会误触发
        - IDLE 状态的升降触发条件直接使用 _get_v_distance，利用 NaN 比较
          总是返回 False 的特性保证安全。
        """
        val = self._get_v_distance(v_idx)
        if math.isnan(val):
            return default
        return val

    def _stage_forward_speed(self):
        if not self._step_motion_enabled:
            return 0.0
        param_name = self._stage_speed_params.get(self.current_state)
        if not param_name:
            return 0.0
        speed = float(self.get_parameter(param_name).value)
        return speed * self._step_speed_scale

    def _publish_step_velocity(self):
        if self.current_state == State.IDLE:
            return

        along_speed = self._stage_forward_speed()
        vx = 0.0
        vy = 0.0
        if self.current_direction == Direction.FORWARD:
            vx = along_speed
        elif self.current_direction == Direction.BACKWARD:
            vx = -along_speed
        elif self.current_direction == Direction.LEFT:
            vy = along_speed
        elif self.current_direction == Direction.RIGHT:
            vy = -along_speed

        if (
            self._step_pose_correction_enabled
            and self._step_start_pose is not None
            and self._latest_pose is not None
        ):
            corr_vx, corr_vy, corr_wz = self._pose_hold_correction()
            vx += corr_vx
            vy += corr_vy
            wz = corr_wz
        else:
            wz = 0.0

        self._publish_velocity(vx, vy, wz)

    def _pose_hold_correction(self):
        start_x, start_y, start_yaw = self._step_start_pose
        current_x, current_y, current_yaw = self._latest_pose
        dx = current_x - start_x
        dy = current_y - start_y
        cos_yaw = math.cos(start_yaw)
        sin_yaw = math.sin(start_yaw)
        start_body_x = cos_yaw * dx + sin_yaw * dy
        start_body_y = -sin_yaw * dx + cos_yaw * dy

        cross_kp = float(self.get_parameter('step_motion.cross_kp').value)
        yaw_kp = float(self.get_parameter('step_motion.yaw_kp').value)
        max_cross_speed = abs(float(self.get_parameter(
            'step_motion.max_cross_speed').value))
        max_yaw_speed = abs(float(self.get_parameter(
            'step_motion.max_yaw_speed').value))

        if self.current_direction in (Direction.FORWARD, Direction.BACKWARD):
            cross_error = start_body_y
            cross_unit_x = -sin_yaw
            cross_unit_y = cos_yaw
        else:
            cross_error = start_body_x
            cross_unit_x = cos_yaw
            cross_unit_y = sin_yaw

        corr_speed = clamp(-cross_kp * cross_error,
                           -max_cross_speed, max_cross_speed)
        corr_vx_map = corr_speed * cross_unit_x
        corr_vy_map = corr_speed * cross_unit_y
        corr_vx, corr_vy = map_velocity_to_body(
            corr_vx_map, corr_vy_map, current_yaw)

        yaw_error = normalize_angle(start_yaw - current_yaw)
        corr_wz = clamp(yaw_kp * yaw_error, -max_yaw_speed, max_yaw_speed)
        return corr_vx, corr_vy, corr_wz

    def _publish_velocity(self, vx, vy, wz):
        self._last_velocity_cmd = [float(vx), float(vy), float(wz)]
        msg = Float32MultiArray()
        msg.data = self._last_velocity_cmd
        self.pub_velocity.publish(msg)

    # ---- 主控制循环 ----
    def control_loop(self):
        self.update_virtual_mapping()
        v_0, v_1, v_2, v_3 = 0, 1, 2, 3

        if self._active_goal:
            self.execute_state_machine(v_0, v_1, v_2, v_3)
            if self._step_motion_active:
                self._publish_step_velocity()

        ros_msg = Float32MultiArray()
        ros_msg.data = [float(h) for h in self.wheel_heights_target]
        self.pub_action.publish(ros_msg)

        state_msg = Int32(data=self.current_state.value)
        self.pub_state.publish(state_msg)

    def execute_state_machine(self, v_0, v_1, v_2, v_3):
        state = self.current_state
        prev_state = self.current_state

        if state == State.IDLE:
            # 参考 active_suspension_control: 使用 _get_v_distance 直接获取，
            # NaN 时比较返回 False，不会误触发升降（安全侧）
            idle_v0_dist = self._get_v_distance(0)
            idle_v1_dist = self._get_v_distance(1)
            cond_up = idle_v1_dist < 200
            cond_down = idle_v0_dist > 200

            self._idle_debug_counter += 1
            if self._idle_debug_counter % 50 == 0:
                self.get_logger().info(
                    f'状态0: 方向={self.current_direction.name}, '
                    f'v0_dist={self._format_distance(idle_v0_dist)}, '
                    f'v1_dist={self._format_distance(idle_v1_dist)}, '
                    f'上升条件={cond_up}, 下降条件={cond_down}, '
                    f'上升稳定计数={self._stable_counters.get("idle_to_up", 0)}, '
                    f'下降稳定计数={self._stable_counters.get("idle_to_down", 0)}'
                )

            if self._is_stable(cond_up, 'idle_to_up'):
                self.current_state = State.UP_1_PREPARE
            elif self._is_stable(cond_down, 'idle_to_down'):
                self.current_state = State.DOWN_1_PREPARE

        # ---- 上台阶 ----
        elif state == State.UP_1_PREPARE:
            self.target_height = self.H_LIFT_LOW
            self.wheel_heights_target = [self.target_height] * 4
            cond = self.check_height_reached([v_0, v_1, v_2, v_3], self.target_height)
            if self._is_stable(cond, 'up1_height', threshold=2):
                if self._known_step_height:
                    self.current_state = State.UP_3_FRONT_DOCK
                else:
                    self.current_state = State.UP_2_LIFT

        elif state == State.UP_2_LIFT:
            v1_dist = self._get_v_distance_safe(1, default=999.0)
            cond_high = v1_dist < 200
            if self._is_stable(cond_high, 'up2_high_dist', threshold=18):
                self.target_height = self.H_LIFT_HIGH
                self.wheel_heights_target = [self.target_height] * 4
                cond_h = self.check_height_reached([v_0, v_1, v_2, v_3], self.target_height)
                if self._is_stable(cond_h, 'up2_height', threshold=2):
                    self.current_state = State.UP_3_FRONT_DOCK
            elif self._is_stable(not cond_high, 'up2_low_dist'):
                self.current_state = State.UP_3_FRONT_DOCK

        elif state == State.UP_3_FRONT_DOCK:
            v0_dist = self._get_v_distance_safe(0, default=999.0)
            cond = v0_dist < 80.0
            if self._is_stable(cond, 'up3_dist',threshold=2):
                self.current_state = State.UP_4_RETRACT_FRONT

        elif state == State.UP_4_RETRACT_FRONT:
            self._set_v_wheel_height([v_0, v_1], 5.0)
            cond = self.check_height_reached([v_0, v_1], 5.0)
            if self._is_stable(cond, 'up4_height', threshold=2):
                self.current_state = State.UP_5_FRONT_LAND

        elif state == State.UP_5_FRONT_LAND:
            cond = self._get_v_pe(v_0) == 1
            if self._is_stable(cond, 'up5_pe'):
                self._set_v_wheel_height([v_0, v_1], 10.0)
                self._set_v_wheel_height([v_2, v_3], self.target_height + 5.0)
                cond_h = self.check_height_reached([v_2, v_3], self.target_height + 5.0)
                if self._is_stable(cond_h, 'up5_height', threshold=2):
                    self.current_state = State.UP_6_SIDE_DOCK_RETRACT_REAR

        elif state == State.UP_6_SIDE_DOCK_RETRACT_REAR:
            cond = self._get_v_pe(v_2) == 1
            up6_pe_stable = self._is_stable(cond, 'up6_pe', threshold=2)
            cond_h = False
            if up6_pe_stable:
                self._set_v_wheel_height([v_2, v_3], -10.0)
                cond_h = self.check_height_reached([v_2, v_3], -10.0)
                if self._is_stable(cond_h, 'up6_height', threshold=2):
                    self.current_state = State.UP_7_REAR_LAND

        elif state == State.UP_7_REAR_LAND:
            cond = self._get_v_pe(v_3) == 1 
            if self._is_stable(cond, 'up7_pe'):
                self._set_v_wheel_height([v_2, v_3], 10.0)
                cond_h = self.check_height_reached([v_2, v_3], 10.0)
                if self._is_stable(cond_h, 'up7_height', threshold=2):
                    self.current_state = State.UP_8_RECOVER

        elif state == State.UP_8_RECOVER:
            self.wheel_heights_target = [self.H_INIT] * 4
            cond = self.check_height_reached([v_0, v_1, v_2, v_3], self.H_INIT)
            if self._is_stable(cond, 'up8_height', threshold=2):
                self.get_logger().info('上台阶序列完成。')
                self.current_state = State.IDLE

        # ---- 下台阶 ----
        elif state == State.DOWN_1_PREPARE:
            self.wheel_heights_target=[self.down_high] * 4 
            cond_pe = self._get_v_pe(v_0) == 0
            if self._is_stable(cond_pe, 'down1_pe', threshold=2):
                if not self._height_latched:
                    if self._requested_height > 0.0:
                        self.target_height = self._requested_height
                    else:
                        dist = self._get_v_distance_safe(0, default=0.0)
                        if dist > 380:
                            self.target_height = self.H_LIFT_HIGH
                        elif dist > 180:
                            self.target_height = self.H_LIFT_LOW
                        else:
                            self.target_height = self.H_LIFT_LOW
                    self._height_latched = True

                self._set_v_wheel_height([v_0, v_1], self.target_height + 30.0)
                cond_h = self.check_height_reached([v_0, v_1], self.target_height + 10.0)
                if self._is_stable(cond_h, 'down1_height', threshold=2):
                    self._height_latched = False
                    self.current_state = State.DOWN_2_FRONT_HOVER_LAND

        elif state == State.DOWN_2_FRONT_HOVER_LAND:
            cond_pe = self._get_v_pe(v_3) == 0
            if self._is_stable(cond_pe, 'down2_pe', threshold=2):
                self._set_v_wheel_height([v_2, v_3], self.target_height + 10.0)
                cond_h = self.check_height_reached([v_2, v_3], self.target_height)
                if self._is_stable(cond_h, 'down2_height', threshold=2):
                    self.current_state = State.DOWN_3_REAR_HOVER_LAND

        elif state == State.DOWN_3_REAR_HOVER_LAND:
            v3_dist = self._get_v_distance_safe(3, default=0.0)
            cond = v3_dist > 200.0
            if self._is_stable(cond, 'down3_dist'):
                self.wheel_heights_target = [self.H_INIT] * 4
                self.current_state = State.DOWN_4_RECOVERY

        elif state == State.DOWN_4_RECOVERY:
            cond = self.check_height_reached([v_0, v_1, v_2, v_3], self.H_INIT)
            if self._is_stable(cond, 'down4_height', threshold=2):
                self.get_logger().info('下台阶序列完成。')
                self.current_state = State.IDLE

        if self.current_state != prev_state:
            if self._active_goal and prev_state != State.IDLE and self.current_state == State.IDLE:
                self._sequence_done = True
            self._stable_counters.clear()


def main(args=None):
    rclpy.init(args=args)
    node = StepMotionActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
