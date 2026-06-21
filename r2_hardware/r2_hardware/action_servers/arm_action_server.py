#!/usr/bin/env python3
"""
机械臂语义 Action Server

接收 BT 决策层的语义命令，翻译为机械臂控制序列并执行。
支持同步 (wait_result=true) 和后台 (wait_result=false) 两种模式。

控制 topic: /t0x0103_ (Float32MultiArray)
状态 topic: /r0x0203 (Float32MultiArray, 下位机回传)
状态发布: /arm_runtime_state (std_msgs/Bool) - 后台动作完成状态
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from r2_interfaces.action import ArmAction
from std_msgs.msg import Float32MultiArray, Bool
import math
import time

class ArmState:
    """机械臂状态机"""
    IDLE = 0
    MOVING = 1
    GRASPING = 2
    STORING = 3
    PLACING = 4
    RECOVERING = 5
    DONE = 10
    ERROR = -1


class ArmActionServer(Node):
    def __init__(self):
        super().__init__('arm_action_server')

        # ---- 参数 ----
        self.declare_parameter('control_topic', '/t0x0103_')
        self.declare_parameter('feedback_topic', '/r0x0203')
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('default_timeout', 10.0)

        self.control_topic = self.get_parameter('control_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.default_timeout = self.get_parameter('default_timeout').value

        # ---- 状态变量 ----
        self.current_state = ArmState.IDLE
        self.current_command = ''
        self.wait_result = True

        # 后台动作状态（用于 WaitArmIdle 节点轮询）
        self.background_active = False
        self.background_done = True
        self.background_success = True
        self.background_message = ''

        # 下位机反馈数据 (来自 /r0x0203)
        self.feedback_data = [0.0] * 9  # [arm完成标志, ...电机角度...]

        # ---- 发布/订阅 ----
        self.pub_action = self.create_publisher(
            Float32MultiArray, self.control_topic, 10)

        # 发布后台动作状态，供 WaitArmIdle 节点订阅
        self.pub_runtime_state = self.create_publisher(
            Bool, '/arm_runtime_state', 10)

        self.sub_feedback = self.create_subscription(
            Float32MultiArray, self.feedback_topic, self._feedback_cb, 10)

        # ---- Action Server ----
        self._action_server = ActionServer(
            self,
            ArmAction,
            'arm_action',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info('Arm Action Server initialized.')

    # ---- 传感器回调 ----
    def _feedback_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 9:
            for i in range(9):
                self.feedback_data[i] = float(msg.data[i])

    # ---- Action Server 回调 ----
    def _goal_callback(self, goal_request):
        cmd = goal_request.command  
        valid_commands = [
            0,  # idle
            1,  # grasp
            2,  # store_to_body
            3,  # store_on_arm
            4,  # get_body
            5,  # place_mid
            6,  # place_high
        ]
        if cmd not in valid_commands:
            self.get_logger().warn(f'Rejecting unknown arm command: {cmd}')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepting arm goal: command={cmd}, '
            f'wait_result={goal_request.wait_result}'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Arm goal cancelled, returning to safe pose.')
        self._send_safe_command()
        self.current_state = ArmState.IDLE
        self.current_command = ''
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        command = goal_handle.request.command
        wait_result = goal_handle.request.wait_result
        timeout_sec = float(goal_handle.request.timeout_sec)

        if timeout_sec <= 0.0:
            timeout_sec = self.default_timeout

        self.current_command = command
        self.wait_result = wait_result

        # TODO: 临时联调桩。机械臂下位机协议稳定后，恢复下面的同步/后台等待逻辑。
        self.current_state = ArmState.DONE
        self.background_active = False
        self.background_done = True
        self.background_success = True
        self.background_message = f'Stubbed arm command {command} done'
        self._publish_runtime_state()
        self.get_logger().info(
            f'Stubbed arm goal returning immediately: command={command}, '
            f'wait_result={wait_result}, timeout={timeout_sec}s'
        )
        goal_handle.succeed()
        self.current_state = ArmState.IDLE
        self.current_command = ''
        return ArmAction.Result(
            success=True,
            message=f'Stubbed arm command {command} done',
            final_state=ArmState.IDLE,
            elapsed_sec=0.0,
        )

        # 后台执行模式：立即返回成功，后续由 WaitArmIdle 轮询
        if not wait_result:
            self._dispatch_command(command)
            self.background_active = True
            self.background_done = False
            self.background_success = True
            self.background_message = f'Background command {command} dispatched'
            self._publish_runtime_state()
            self.get_logger().info(f'Background arm command dispatched: {command}')
            goal_handle.succeed()
            return ArmAction.Result(
                success=True,
                message=f'Background command {command} dispatched',
                final_state=ArmState.IDLE,
                elapsed_sec=0.0,
            )

        # 同步执行模式
        self._dispatch_command(command)
        start_time = self.get_clock().now()

        feedback_msg = ArmAction.Feedback()

        while rclpy.ok():
            elapsed_sec = (self.get_clock().now() - start_time).nanoseconds / 1e9

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._send_safe_command()
                self.current_state = ArmState.IDLE
                self.current_command = ''
                return ArmAction.Result(
                    success=False,
                    message='Cancelled',
                    final_state=ArmState.IDLE,
                    elapsed_sec=float(elapsed_sec),
                )

            if timeout_sec > 0.0 and elapsed_sec > timeout_sec:
                goal_handle.abort()
                self._send_safe_command()
                self.current_state = ArmState.IDLE
                self.current_command = ''
                return ArmAction.Result(
                    success=False,
                    message=f'Timeout ({timeout_sec}s)',
                    final_state=ArmState.IDLE,
                    elapsed_sec=float(elapsed_sec),
                )

            # 发布反馈
            feedback_msg.state = self._state_name(self.current_state)
            feedback_msg.progress = self._estimate_progress(command, elapsed_sec, timeout_sec)
            feedback_msg.message = f'Executing {command}: {self._state_name(self.current_state)}'
            goal_handle.publish_feedback(feedback_msg)

            # 检查完成
            if self._is_command_done(command):
                self.get_logger().info(f'Arm command {command} completed.')
                goal_handle.succeed()
                self.current_state = ArmState.IDLE
                self.current_command = ''
                # 更新后台状态
                if self.background_active:
                    self.background_active = False
                    self.background_done = True
                    self.background_success = True
                    self.background_message = f'{command} done'
                    self._publish_runtime_state()
                return ArmAction.Result(
                    success=True,
                    message=f'{command} done',
                    final_state=ArmState.IDLE,
                    elapsed_sec=float(elapsed_sec),
                )

            time.sleep(1.0 / self.control_freq)

        goal_handle.abort()
        self.current_state = ArmState.IDLE
        self.current_command = ''
        return ArmAction.Result(
            success=False,
            message='Node shutdown',
            final_state=ArmState.IDLE,
            elapsed_sec=float(
                (self.get_clock().now() - start_time).nanoseconds / 1e9
            ),
        )

    def _sleep_async(self, duration_sec):
        time.sleep(duration_sec)

    def _publish_runtime_state(self):
        """发布后台动作状态到 /arm_runtime_state topic"""
        msg = Bool()
        msg.data = self.background_done and self.background_success
        self.pub_runtime_state.publish(msg)

    # ---- 命令分发 ----
    def _dispatch_command(self, command: int):
        """
        将语义命令翻译为下位机控制指令。
        发布到 /t0x0103_ (Float32MultiArray)。

        指令格式约定:
          [cmd_id, param1, param2, ...]

          cmd_id 映射:
            0 = idle / 停止
            1 = grasp (抓取KFS)
            2 = store_to_body (转存到车体)
            3 = store_on_arm (暂持在手臂)
            4 = get_body (从车体取出到手臂)
            5 = place_mid (放置到中层)
            6 = place_high (放置到上层)
        """
        cmd_map = {
            0: [0.0, 0.0, 0.0, 0.0],
            1: [1.0, 0.0, 0.0, 0.0],
            2: [2.0, 0.0, 0.0, 0.0],
            3: [3.0, 0.0, 0.0, 0.0],
            4: [4.0, 0.0, 0.0, 0.0],
            5: [5.0, 0.0, 0.0, 0.0],
            6: [6.0, 0.0, 0.0, 0.0],
        }

        data = cmd_map.get(command, [0.0, 0.0, 0.0, 0.0])
        state_map = {
            0: ArmState.IDLE,
            1: ArmState.GRASPING,
            2: ArmState.STORING,
            3: ArmState.STORING,
            4: ArmState.MOVING,
            5: ArmState.PLACING,
            6: ArmState.PLACING,
        }
        self.current_state = state_map.get(command, ArmState.IDLE)

        self.pub_action.publish(Float32MultiArray(data=data))
        self.get_logger().debug(f'Dispatched arm command: {command} -> {data}')

    def _send_safe_command(self):
        """发送停止指令"""
        self.pub_action.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0]))

    # ---- 完成检测 ----
    def _is_command_done(self, command: int) -> bool:
        """
        通过下位机反馈判断命令是否完成。
        /r0x0203 data[0] == 1.0 表示动作完成。
        不同命令可扩展不同的完成条件。
        """
        if len(self.feedback_data) > 0:
            # data[0] 为动作完成标志位
            return self.feedback_data[0] == 1.0
        return False

    def _estimate_progress(self, command: int, elapsed: float, timeout: float) -> float:
        """基于时间的粗略进度估算 (0.0~1.0)"""
        # 不同命令有不同预期耗时
        expected_times = {
            1: 3.0,  # grasp
            2: 2.0,  # store_to_body
            3: 1.5,  # store_on_arm
            4: 2.0,  # get_body
            5: 3.0,  # place_mid
            6: 3.0,  # place_high
            0: 0.1,  # idle
        }
        expected = expected_times.get(command, timeout)
        return min(elapsed / max(expected, 0.1), 0.99)

    @staticmethod
    def _state_name(state: int) -> str:
        names = {
            ArmState.IDLE: 'idle',
            ArmState.MOVING: 'moving',
            ArmState.GRASPING: 'grasping',
            ArmState.STORING: 'storing',
            ArmState.PLACING: 'placing',
            ArmState.RECOVERING: 'recovering',
            ArmState.DONE: 'done',
            ArmState.ERROR: 'error',
        }
        return names.get(state, 'unknown')


def main(args=None):
    rclpy.init(args=args)
    node = ArmActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
