#!/usr/bin/env python3
"""
主动悬挂 Action Server
将上下台阶状态机封装为 ROS 2 Action，接收 BT 决策层的目标并异步执行。
通过 /t0x0102_action 发布四轮目标高度，经由 USB 桥接下发给下位机。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from r2_interfaces.action import SuspensionControl
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


class SuspensionActionServer(Node):
    def __init__(self):
        super().__init__('suspension_action_server')

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

        # 传感器订阅
        self.sub_sensor_dist = self.create_subscription(
            Float32MultiArray, 'sensor_distances', self.dist_cb, 10)
        self.sub_r0x0201 = self.create_subscription(
            Float32MultiArray, 'r0x0121', self.hw_status_cb, 10)

        # 控制发布
        self.pub_action = self.create_publisher(Float32MultiArray, 't0x0112_action', 10)
        self.pub_state = self.create_publisher(Int32, 'current_state', 10)

        # Action Server
        self._action_server = ActionServer(
            self,
            SuspensionControl,
            'suspension_control',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # 控制循环 (100Hz)
        self.delay_timer = self.create_timer(0.2, self._start_control_loop)
        self.get_logger().info('Suspension Action Server initialized.')

    def _start_control_loop(self):
        self.delay_timer.cancel()
        self.control_timer = self.create_timer(0.01, self.control_loop)

    # ---- Action Server 回调 ----
    def _goal_callback(self, goal_request):
        mode = goal_request.mode
        direction = goal_request.direction
        if mode not in (0, 1, 2, 3):
            return GoalResponse.REJECT
        if direction not in (0, 1, 2, 3):
            return GoalResponse.REJECT
        if not math.isfinite(float(goal_request.height)):
            return GoalResponse.REJECT
        self.get_logger().info(
            f'Accepting suspension goal: mode={mode}, direction={direction}, '
            f'height={goal_request.height:.1f}'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Suspension goal cancelled, returning to IDLE.')
        self._active_goal = False
        self._active_mode = 0
        self._requested_height = 0.0
        self._known_step_height = False
        self._sequence_done = False
        self.current_state = State.IDLE
        self.wheel_heights_target = [self.H_INIT] * 4
        self._stable_counters.clear()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        mode = int(goal_handle.request.mode)
        direction = goal_handle.request.direction
        height = float(goal_handle.request.height)
        timeout_sec = float(goal_handle.request.timeout_sec)

        if mode == 0:
            if height > 0.0:
                mode = 1
            elif height < 0.0:
                mode = 2

        requested_step_height = abs(height)
        self._known_step_height = mode != 3 and requested_step_height > 0.0
        if self._known_step_height:
            self._requested_height = self._map_step_height_to_prepare_height(requested_step_height)
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
        if mode == 3:
            # MODE_DIRECT: 直接设置四轮统一高度
            self.wheel_heights_target = [height] * 4

        self._active_goal = True
        self._active_mode = mode
        self._sequence_done = False
        self._stable_counters.clear()
        self._height_latched = False
        start_time = self.get_clock().now()

        feedback_msg = SuspensionControl.Feedback()

        while rclpy.ok():
            elapsed_sec = (self.get_clock().now() - start_time).nanoseconds / 1e9

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._active_goal = False
                self._active_mode = 0
                self._requested_height = 0.0
                self._known_step_height = False
                self._sequence_done = False
                self.current_state = State.IDLE
                self.wheel_heights_target = [self.H_INIT] * 4
                return SuspensionControl.Result(
                    success=False,
                    message='Cancelled',
                    final_state=self.current_state.value,
                    elapsed_sec=float(elapsed_sec),
                )

            feedback_msg.current_state = self.current_state.value
            feedback_msg.elapsed_sec = float(elapsed_sec)
            feedback_msg.distance_data = [
                float(d) for d in self.distance_filtered
            ]
            feedback_msg.photoelectric_data = [
                float(pe) for pe in self.pe_switches_filtered
            ]
            feedback_msg.wheel_heights_current = [
                float(h) for h in self.wheel_heights_current
            ]
            feedback_msg.wheel_heights_target = [
                float(h) for h in self.wheel_heights_target
            ]
            goal_handle.publish_feedback(feedback_msg)

            if timeout_sec > 0.0 and elapsed_sec > timeout_sec:
                goal_handle.abort()
                self._active_goal = False
                self._active_mode = 0
                self._requested_height = 0.0
                self._known_step_height = False
                self._sequence_done = False
                self.current_state = State.IDLE
                self.wheel_heights_target = [self.H_INIT] * 4
                return SuspensionControl.Result(
                    success=False,
                    message='Timeout',
                    final_state=self.current_state.value,
                    elapsed_sec=float(elapsed_sec),
                )

            if mode == 3 and self.check_height_reached([0, 1, 2, 3], height):
                goal_handle.succeed()
                self._active_goal = False
                self._active_mode = 0
                self._requested_height = 0.0
                self._known_step_height = False
                return SuspensionControl.Result(
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
                return SuspensionControl.Result(
                    success=True,
                    message='Sequence complete',
                    final_state=self.current_state.value,
                    elapsed_sec=float(elapsed_sec),
                )

            time.sleep(0.01)

        goal_handle.abort()
        self._active_goal = False
        self._active_mode = 0
        self._requested_height = 0.0
        self._known_step_height = False
        return SuspensionControl.Result(
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

    # ---- 主控制循环 ----
    def control_loop(self):
        self.update_virtual_mapping()
        v_0, v_1, v_2, v_3 = 0, 1, 2, 3

        if self._active_goal and self._active_mode != 3:
            self.execute_state_machine(v_0, v_1, v_2, v_3)

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
    node = SuspensionActionServer()
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
