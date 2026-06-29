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
import os
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
    DOWN_3_WAIT_REAR_HOVER_LAND = 22
    DOWN_4_REAR_HOVER_LAND = 23
    DOWN_5_RECOVERY = 24


class Direction(Enum):
    FORWARD = 0
    LEFT = 1
    RIGHT = 2


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def ensure_ros_log_dir():
    if os.environ.get('ROS_LOG_DIR'):
        return
    log_dir = '/tmp/ros_logs'
    os.makedirs(log_dir, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = log_dir


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
        self.H_INIT = 20.0
        self.HEIGHT_TOLERANCE = 20.0
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
        self._step_target_pose = None
        self._stage_speed_level = '200'
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
        self.declare_parameter('step_motion.cross_kp', 2.0)
        self.declare_parameter('step_motion.yaw_kp', 1.0)
        self.declare_parameter('step_motion.max_cross_speed', 0.35)
        self.declare_parameter('step_motion.max_yaw_speed', 0.45)
        self.declare_parameter('step_motion.max_start_position_error', 0.30)
        self.declare_parameter('step_motion.max_start_yaw_error_deg', 30.0)

        self._stage_speed_suffixes = {
            State.UP_1_PREPARE: 'up_1_prepare',
            State.UP_2_LIFT: 'up_2_lift',
            State.UP_3_FRONT_DOCK: 'up_3_front_dock',
            State.UP_4_RETRACT_FRONT: 'up_4_retract_front',
            State.UP_5_FRONT_LAND: 'up_5_front_land',
            State.UP_6_SIDE_DOCK_RETRACT_REAR:
                'up_6_side_dock_retract_rear',
            State.UP_7_REAR_LAND: 'up_7_rear_land',
            State.UP_8_RECOVER: 'up_8_recover',
            State.DOWN_1_PREPARE: 'down_1_prepare',
            State.DOWN_2_FRONT_HOVER_LAND: 'down_2_front_hover_land',
            State.DOWN_3_WAIT_REAR_HOVER_LAND:
                'down_3_wait_rear_hover_land',
            State.DOWN_4_REAR_HOVER_LAND: 'down_4_rear_hover_land',
            State.DOWN_5_RECOVERY: 'down_5_recovery',
        }

        self._declare_stage_height_parameters()

        # 200mm 台阶速度参数。速度沿上下台阶方向，单位 m/s。
        # 上台阶: 四轮先抬到准备高度，通常原地等待悬挂到位。
        self.declare_parameter('step_motion.stage_speed.height_200.up_1_prepare', 1.0)
        # 上台阶: 未知台阶高度时继续判断高/低台阶，通常原地等待。
        self.declare_parameter('step_motion.stage_speed.height_200.up_2_lift', 1.0)
        # 上台阶: 前轮对接台阶边缘，需要沿台阶方向向前贴近。
        self.declare_parameter('step_motion.stage_speed.height_200.up_3_front_dock', 0.40)
        # 上台阶: 收前轮悬挂，避免边走边收导致前轮落点不稳。
        self.declare_parameter('step_motion.stage_speed.height_200.up_4_retract_front', 0.5)
        # 上台阶: 前轮落地后轻推，让车身稳定进入台阶面。
        self.declare_parameter('step_motion.stage_speed.height_200.up_5_front_land', 1.0)
        # 上台阶: 后轮靠近台阶并收后轮，是主要向前推进阶段。
        self.declare_parameter(
            'step_motion.stage_speed.height_200.up_6_side_dock_retract_rear',
            0.5,
        )
        # 上台阶: 后轮落地阶段，低速保持前进避免冲过头。
        self.declare_parameter('step_motion.stage_speed.height_200.up_7_rear_land', 1.0)
        # 上台阶: 四轮恢复行驶高度，通常原地等待恢复完成。
        self.declare_parameter('step_motion.stage_speed.height_200.up_8_recover', 1.0)
        # 下台阶: 降到下台阶准备高度，通常原地等待检测边缘。
        self.declare_parameter('step_motion.stage_speed.height_200.down_1_prepare', 0.8)
        # 下台阶: 前轮悬空/落地阶段，低速向前送前轮。
        self.declare_parameter(
            'step_motion.stage_speed.height_200.down_2_front_hover_land',
            0.80,
        )
        # 下台阶: 等待后轮到达台阶边缘，继续沿台阶方向向前。
        self.declare_parameter(
            'step_motion.stage_speed.height_200.down_3_wait_rear_hover_land',
            0.8,
        )
        # 下台阶: 后轮悬空/落地阶段，继续沿台阶方向向前。
        self.declare_parameter(
            'step_motion.stage_speed.height_200.down_4_rear_hover_land',
            0.8,
        )
        # 下台阶: 四轮恢复行驶高度，通常原地等待恢复完成。
        self.declare_parameter('step_motion.stage_speed.height_200.down_5_recovery', 0.8)

        # 400mm 台阶速度参数。和 200mm 分开显式声明，方便单独调参。
        # 上台阶: 四轮先抬到准备高度，通常原地等待悬挂到位。
        self.declare_parameter('step_motion.stage_speed.height_400.up_1_prepare', 0.8)
        # 上台阶: 未知台阶高度时继续判断高/低台阶，通常原地等待。
        self.declare_parameter('step_motion.stage_speed.height_400.up_2_lift', 0.8)
        # 上台阶: 前轮对接台阶边缘，需要沿台阶方向向前贴近。
        self.declare_parameter('step_motion.stage_speed.height_400.up_3_front_dock', 0.50)
        # 上台阶: 收前轮悬挂，避免边走边收导致前轮落点不稳。
        self.declare_parameter('step_motion.stage_speed.height_400.up_4_retract_front', 0.2)
        # 上台阶: 前轮落地后轻推，让车身稳定进入台阶面。
        self.declare_parameter('step_motion.stage_speed.height_400.up_5_front_land', 0.8)
        # 上台阶: 后轮靠近台阶并收后轮，是主要向前推进阶段。
        self.declare_parameter(
            'step_motion.stage_speed.height_400.up_6_side_dock_retract_rear',
            0.2,
        )
        # 上台阶: 后轮落地阶段，低速保持前进避免冲过头。
        self.declare_parameter('step_motion.stage_speed.height_400.up_7_rear_land', 0.8)
        # 上台阶: 四轮恢复行驶高度，通常原地等待恢复完成。
        self.declare_parameter('step_motion.stage_speed.height_400.up_8_recover', 0.8)
        # 下台阶: 降到下台阶准备高度，通常原地等待检测边缘。
        self.declare_parameter('step_motion.stage_speed.height_400.down_1_prepare', 0.8)
        # 下台阶: 前轮悬空/落地阶段，低速向前送前轮。
        self.declare_parameter(
            'step_motion.stage_speed.height_400.down_2_front_hover_land',
            0.20,
        )
        # 下台阶: 等待后轮到达台阶边缘，继续沿台阶方向向前。
        self.declare_parameter(
            'step_motion.stage_speed.height_400.down_3_wait_rear_hover_land',
            0.3,
        )
        # 下台阶: 后轮悬空/落地阶段，继续沿台阶方向向前。
        self.declare_parameter(
            'step_motion.stage_speed.height_400.down_4_rear_hover_land',
            0.3,
        )
        # 下台阶: 四轮恢复行驶高度，通常原地等待恢复完成。
        self.declare_parameter('step_motion.stage_speed.height_400.down_5_recovery', 0.8)

        # 仅前向上台阶使用: 根据前向 TOF 距离动态覆盖前轮搭上台阶前速度。
        # 距离 >= max_distance_mm 时使用 max_speed，距离 <= min_distance_mm 时使用 min_speed。
        self.declare_parameter('step_motion.forward_up_speed_by_distance.enabled', True)
        self.declare_parameter('step_motion.forward_up_speed_by_distance.min_distance_mm', 80.0)
        self.declare_parameter('step_motion.forward_up_speed_by_distance.max_distance_mm', 200.0)
        self.declare_parameter('step_motion.forward_up_speed_by_distance.min_speed', 0.3)
        self.declare_parameter('step_motion.forward_up_speed_by_distance.max_speed', 1.0)

        self.step_relocation_topic = self.get_parameter(
            'step_motion.relocation_topic').value
        self.step_velocity_topic = self.get_parameter(
            'step_motion.velocity_topic').value

    def _declare_stage_height_parameters(self):
        # 高度单位: mm。front/rear 均为虚拟方向上的前/后轮组，all 表示四轮。
        # 和 stage_speed 一样分 200/400 两套，方便实车调参时直接覆盖。
        profile_defaults = {
            '200': {
                'up_1_prepare': {'all': 205.0},
                'up_2_lift': {'all': 205.0},
                'up_4_retract_front': {'front': 2.0},
                'up_5_front_land': {'front': 2.0, 'rear': 207.0},
                'up_6_side_dock_retract_rear': {'rear': -2.0},
                'up_7_rear_land': {'rear': 10.0},
                'up_8_recover': {'all': 20.0},
                'down_1_prepare': {'all': 5.0},
                'down_2_front_hover_land': {'front': 215.0},
                'down_4_rear_hover_land': {'rear': 215.0},
                'down_5_recovery': {'all': 20.0},
            },
            '400': {
                'up_1_prepare': {'all': 410.0},
                'up_2_lift': {'all': 410.0},
                'up_4_retract_front': {'front': 2.0},
                'up_5_front_land': {'front': 2.0, 'rear': 412.0},
                'up_6_side_dock_retract_rear': {'rear': -2.0},
                'up_7_rear_land': {'rear': 10.0},
                'up_8_recover': {'all': 20.0},
                'down_1_prepare': {'all': 5.0},
                'down_2_front_hover_land': {'front': 420.0},
                'down_4_rear_hover_land': {'rear': 420.0},
                'down_5_recovery': {'all': 20.0},
            },
        }

        for profile, stage_defaults in profile_defaults.items():
            for suffix, group_defaults in stage_defaults.items():
                for group, default in group_defaults.items():
                    self.declare_parameter(
                        f'step_motion.stage_height.height_{profile}.{suffix}.{group}',
                        default,
                    )

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
        if mode not in (0, 1, 2, 3):
            return GoalResponse.REJECT
        if direction not in (0, 1, 2):
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.height)):
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.timeout_sec)):
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.correction_x)):
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.correction_y)):
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.correction_yaw_deg)):
            return GoalResponse.REJECT

        if mode != 3:
            if self._latest_pose is None:
                self.get_logger().warning(
                    'Rejecting step motion goal: no relocation pose yet.')
                return GoalResponse.REJECT
            pose_x, pose_y, pose_yaw = self._latest_pose
            target_yaw = math.radians(float(goal_request.correction_yaw_deg))
            cross_error = abs(
                self._cross_track_error(
                    int(direction),
                    pose_x,
                    pose_y,
                    float(goal_request.correction_x),
                    float(goal_request.correction_y),
                    target_yaw,
                )
            )
            yaw_error = abs(normalize_angle(target_yaw - pose_yaw))
            max_position_error = float(self.get_parameter(
                'step_motion.max_start_position_error').value)
            max_yaw_error = math.radians(float(self.get_parameter(
                'step_motion.max_start_yaw_error_deg').value))
            if cross_error > max_position_error or yaw_error > max_yaw_error:
                self.get_logger().warning(
                    f'Rejecting step motion goal: correction target too far, '
                    f'cross_error={cross_error:.3f}m, '
                    f'yaw_error={math.degrees(yaw_error):.1f}deg'
                )
                return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepting step motion goal: mode={mode}, direction={direction}, '
            f'height={goal_request.height:.1f}, '
            f'correction=({goal_request.correction_x:.3f}, '
            f'{goal_request.correction_y:.3f}, '
            f'{goal_request.correction_yaw_deg:.1f}deg)'
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

        direct_height_mode = mode == 3
        requested_step_height = abs(height)
        self._stage_speed_level = self._height_profile_for_step_height(
            requested_step_height)
        self._known_step_height = (not direct_height_mode) and requested_step_height > 0.0
        if direct_height_mode:
            self._requested_height = 0.0
        elif self._known_step_height:
            self._requested_height = self._map_step_height_to_prepare_height(
                requested_step_height)
        else:
            self._requested_height = 0.0

        self.current_direction = Direction(direction)
        self.current_state = {
            1: State.UP_1_PREPARE,
            2: State.DOWN_1_PREPARE,
        }.get(mode, State.IDLE)
        if direct_height_mode:
            self.wheel_heights_target = [height] * 4

        self._step_target_pose = (
            float(goal_handle.request.correction_x),
            float(goal_handle.request.correction_y),
            math.radians(float(goal_handle.request.correction_yaw_deg)),
        )
        self._step_motion_active = not direct_height_mode
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
            feedback_msg.distance_data = [
                float(d) for d in self.distance_filtered
            ]
            feedback_msg.photoelectric_data = [
                float(pe) for pe in self.pe_switches_filtered
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

            if direct_height_mode and self.check_height_reached([0, 1, 2, 3], height):
                goal_handle.succeed()
                self._active_goal = False
                self._active_mode = 0
                self._requested_height = 0.0
                self._known_step_height = False
                self._step_motion_active = False
                return StepMotionControl.Result(
                    success=True,
                    message='Direct height set',
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
        profile = self._height_profile_for_step_height(step_height)
        return self._stage_height_for_profile(profile, State.UP_1_PREPARE, 'all')

    def _height_profile_for_step_height(self, step_height):
        return '400' if step_height > 300.0 else '200'

    def _stage_height_for_profile(self, profile, state, group):
        suffix = self._stage_speed_suffixes.get(state)
        if not suffix:
            return 0.0
        param_name = (
            f'step_motion.stage_height.height_{profile}.{suffix}.{group}'
        )
        return float(self.get_parameter(param_name).value)

    def _stage_height(self, state, group):
        return self._stage_height_for_profile(
            self._stage_speed_level, state, group)

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

    def check_height_reached(self, virtual_indices, target_h):
        for v_idx in virtual_indices:
            phys_idx = self.v_wheels_idx[v_idx]
            if abs(self.wheel_heights_current[phys_idx] - target_h) > self.HEIGHT_TOLERANCE:
                return False
        return True

    def _reset_active_step_state(self):
        down_states = {
            State.DOWN_1_PREPARE,
            State.DOWN_2_FRONT_HOVER_LAND,
            State.DOWN_3_WAIT_REAR_HOVER_LAND,
            State.DOWN_4_REAR_HOVER_LAND,
            State.DOWN_5_RECOVERY,
        }
        recover_state = (
            State.DOWN_5_RECOVERY
            if self.current_state in down_states
            else State.UP_8_RECOVER
        )
        recover_height = self._stage_height_for_profile(
            self._stage_speed_level, recover_state, 'all')

        self._active_goal = False
        self._active_mode = 0
        self._requested_height = 0.0
        self._known_step_height = False
        self._sequence_done = False
        self._step_motion_active = False
        self._step_target_pose = None
        self.current_state = State.IDLE
        self.wheel_heights_target = [recover_height] * 4
        self._stage_speed_level = '200'
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
        suffix = self._stage_speed_suffixes.get(self.current_state)
        if not suffix:
            return 0.0
        profile = f'height_{self._stage_speed_level}'
        param_name = f'step_motion.stage_speed.{profile}.{suffix}'
        base_speed = float(self.get_parameter(param_name).value)
        distance_speed = self._forward_up_distance_speed_override()
        if distance_speed is not None:
            return distance_speed
        return base_speed

    def _forward_up_distance_speed_override(self):
        if self.current_direction != Direction.FORWARD:
            return None
        if self.current_state not in (
            State.UP_3_FRONT_DOCK,
            State.UP_4_RETRACT_FRONT,
            State.UP_5_FRONT_LAND,
        ):
            return None
        enabled = bool(self.get_parameter(
            'step_motion.forward_up_speed_by_distance.enabled').value)
        if not enabled:
            return None

        front_distance = self._get_v_distance(0)
        if not math.isfinite(front_distance):
            return None

        min_distance = float(self.get_parameter(
            'step_motion.forward_up_speed_by_distance.min_distance_mm').value)
        max_distance = float(self.get_parameter(
            'step_motion.forward_up_speed_by_distance.max_distance_mm').value)
        min_speed = float(self.get_parameter(
            'step_motion.forward_up_speed_by_distance.min_speed').value)
        max_speed = float(self.get_parameter(
            'step_motion.forward_up_speed_by_distance.max_speed').value)

        if max_distance <= min_distance:
            return clamp(max_speed, min(min_speed, max_speed),
                         max(min_speed, max_speed))

        ratio = (front_distance - min_distance) / (max_distance - min_distance)
        ratio = clamp(ratio, 0.0, 1.0)
        speed = min_speed + ratio * (max_speed - min_speed)
        return clamp(speed, min(min_speed, max_speed), max(min_speed, max_speed))

    def _cross_track_error(
        self,
        direction,
        current_x,
        current_y,
        target_x,
        target_y,
        target_yaw,
    ):
        dx = current_x - target_x
        dy = current_y - target_y
        cos_yaw = math.cos(target_yaw)
        sin_yaw = math.sin(target_yaw)
        target_body_x = cos_yaw * dx + sin_yaw * dy
        target_body_y = -sin_yaw * dx + cos_yaw * dy

        if direction == Direction.FORWARD.value:
            return target_body_y
        return target_body_x

    def _publish_step_velocity(self):
        if self.current_state == State.IDLE:
            return

        along_speed = self._stage_forward_speed()
        vx = 0.0
        vy = 0.0
        if self.current_direction == Direction.FORWARD:
            vx = along_speed
        elif self.current_direction == Direction.LEFT:
            vy = along_speed
        elif self.current_direction == Direction.RIGHT:
            vy = -along_speed

        if (
            self._step_target_pose is not None
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
        start_x, start_y, start_yaw = self._step_target_pose
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

        if self.current_direction == Direction.FORWARD:
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

        if self._active_goal and self._active_mode != 3:
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
            self.target_height = self._stage_height(State.UP_1_PREPARE, 'all')
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
                self._stage_speed_level = '400'
                self.target_height = self._stage_height(State.UP_2_LIFT, 'all')
                self.wheel_heights_target = [self.target_height] * 4
                cond_h = self.check_height_reached([v_0, v_1, v_2, v_3], self.target_height)
                if self._is_stable(cond_h, 'up2_height', threshold=2):
                    self.current_state = State.UP_3_FRONT_DOCK
            elif self._is_stable(not cond_high, 'up2_low_dist'):
                self._stage_speed_level = '200'
                self.current_state = State.UP_3_FRONT_DOCK

        elif state == State.UP_3_FRONT_DOCK:
            v0_dist = self._get_v_distance_safe(0, default=999.0)
            cond = v0_dist < 80.0
            if self._is_stable(cond, 'up3_dist',threshold=2):
                self.current_state = State.UP_4_RETRACT_FRONT

        elif state == State.UP_4_RETRACT_FRONT:
            front_height = self._stage_height(State.UP_4_RETRACT_FRONT, 'front')
            self._set_v_wheel_height([v_0, v_1], front_height)
            cond = self.check_height_reached([v_0, v_1], front_height)
            if self._is_stable(cond, 'up4_height', threshold=2):
                self.current_state = State.UP_5_FRONT_LAND

        elif state == State.UP_5_FRONT_LAND:
            cond = self._get_v_pe(v_0) == 1
            if self._is_stable(cond, 'up5_pe'):
                front_height = self._stage_height(State.UP_5_FRONT_LAND, 'front')
                rear_height = self._stage_height(State.UP_5_FRONT_LAND, 'rear')
                self._set_v_wheel_height([v_0, v_1], front_height)
                self._set_v_wheel_height([v_2, v_3], rear_height)
                cond_h = self.check_height_reached([v_2, v_3], rear_height)
                if self._is_stable(cond_h, 'up5_height', threshold=2):
                    self.current_state = State.UP_6_SIDE_DOCK_RETRACT_REAR

        elif state == State.UP_6_SIDE_DOCK_RETRACT_REAR:
            cond = self._get_v_pe(v_2) == 1
            up6_pe_stable = self._is_stable(cond, 'up6_pe', threshold=2)
            cond_h = False
            if up6_pe_stable:
                rear_height = self._stage_height(
                    State.UP_6_SIDE_DOCK_RETRACT_REAR, 'rear')
                self._set_v_wheel_height([v_2, v_3], rear_height)
                cond_h = self.check_height_reached([v_2, v_3], rear_height)
                if self._is_stable(cond_h, 'up6_height', threshold=2):
                    self.current_state = State.UP_7_REAR_LAND

        elif state == State.UP_7_REAR_LAND:
            cond = self._get_v_pe(v_3) == 1 
            if self._is_stable(cond, 'up7_pe'):
                rear_height = self._stage_height(State.UP_7_REAR_LAND, 'rear')
                self._set_v_wheel_height([v_2, v_3], rear_height)
                cond_h = self.check_height_reached([v_2, v_3], rear_height)
                if self._is_stable(cond_h, 'up7_height', threshold=2):
                    self.current_state = State.UP_8_RECOVER

        elif state == State.UP_8_RECOVER:
            recover_height = self._stage_height(State.UP_8_RECOVER, 'all')
            self.wheel_heights_target = [recover_height] * 4
            cond = self.check_height_reached([v_0, v_1, v_2, v_3], recover_height)
            if self._is_stable(cond, 'up8_height', threshold=2):
                self.get_logger().info('上台阶序列完成。')
                self.current_state = State.IDLE

        # ---- 下台阶 ----
        elif state == State.DOWN_1_PREPARE:
            prepare_height = self._stage_height(State.DOWN_1_PREPARE, 'all')
            self.wheel_heights_target = [prepare_height] * 4
            cond_pe = self._get_v_pe(v_0) == 0
            if self._is_stable(cond_pe, 'down1_pe', threshold=2):
                if not self._height_latched:
                    if self._requested_height > 0.0:
                        self.target_height = self._stage_height(
                            State.DOWN_2_FRONT_HOVER_LAND, 'front')
                    else:
                        dist = self._get_v_distance_safe(0, default=0.0)
                        if dist > 380:
                            self._stage_speed_level = '400'
                        elif dist > 180:
                            self._stage_speed_level = '200'
                        else:
                            self._stage_speed_level = '200'
                        self.target_height = self._stage_height(
                            State.DOWN_2_FRONT_HOVER_LAND, 'front')
                    self._height_latched = True

                self.current_state = State.DOWN_2_FRONT_HOVER_LAND

        elif state == State.DOWN_2_FRONT_HOVER_LAND:
            front_height = self._stage_height(
                State.DOWN_2_FRONT_HOVER_LAND, 'front')
            self._set_v_wheel_height([v_0, v_1], front_height)
            cond_h = self.check_height_reached([v_0, v_1], front_height)
            if self._is_stable(cond_h, 'down2_height', threshold=2):
                self._height_latched = False
                self.current_state = State.DOWN_3_WAIT_REAR_HOVER_LAND

        elif state == State.DOWN_3_WAIT_REAR_HOVER_LAND:
            cond_pe = self._get_v_pe(v_3) == 0
            if self._is_stable(cond_pe, 'down3_pe', threshold=2):
                self.current_state = State.DOWN_4_REAR_HOVER_LAND

        elif state == State.DOWN_4_REAR_HOVER_LAND:
            rear_height = self._stage_height(
                State.DOWN_4_REAR_HOVER_LAND, 'rear')
            self._set_v_wheel_height([v_2, v_3], rear_height)
            cond_h = self.check_height_reached([v_2, v_3], rear_height)
            v3_dist = self._get_v_distance_safe(3, default=0.0)
            cond = v3_dist > 200.0
            if (
                self._is_stable(cond_h, 'down4_height', threshold=2)
                and self._is_stable(cond, 'down4_dist')
            ):
                recover_height = self._stage_height(State.DOWN_5_RECOVERY, 'all')
                self.wheel_heights_target = [recover_height] * 4
                self.current_state = State.DOWN_5_RECOVERY

        elif state == State.DOWN_5_RECOVERY:
            recover_height = self._stage_height(State.DOWN_5_RECOVERY, 'all')
            cond = self.check_height_reached([v_0, v_1, v_2, v_3], recover_height)
            if self._is_stable(cond, 'down5_height', threshold=2):
                self.get_logger().info('下台阶序列完成。')
                self.current_state = State.IDLE

        if self.current_state != prev_state:
            if self._active_goal and prev_state != State.IDLE and self.current_state == State.IDLE:
                self._sequence_done = True
            self._stable_counters.clear()


def main(args=None):
    ensure_ros_log_dir()
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
