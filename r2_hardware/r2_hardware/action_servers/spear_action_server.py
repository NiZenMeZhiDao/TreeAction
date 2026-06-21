#!/usr/bin/env python3
"""
矛头机构 Action Server

控制矛头夹爪、锁紧机构和伸缩机构，用于准备区的矛头抓取和对接流程。
接收 BT 决策层的语义命令并异步执行。

控制 topic: /t0x0104_ (Float32MultiArray)
状态 topic: /r0x0204 (Float32MultiArray, 下位机回传)

命令:
  - prepare:      张开夹爪、解锁机构、移动到待抓取位姿
  - grasp:        夹爪闭合抓取矛头 → 检测到位 → 锁紧 → 抬起机构
  - dock_extend:  将矛头伸出到对接位
  - dock_release: 松开矛头并收回机械臂
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from r2_interfaces.action import SpearAction
from std_msgs.msg import Float32MultiArray


class SpearState:
    """矛头机构状态机"""
    IDLE = 0
    PREPARING = 1      # 张开夹爪、解锁
    GRASPING = 2       # 闭合夹爪、抓取
    LOCKING = 3        # 锁紧矛头
    LIFTING = 4        # 抬起机构
    DOCK_EXTENDING = 5 # 伸出到对接位
    DOCK_RELEASING = 6 # 松开收回
    RECOVERING = 7     # 回到安全位
    DONE = 10
    ERROR = -1


class SpearActionServer(Node):
    def __init__(self):
        super().__init__('spear_action_server')

        # ---- 参数 ----
        self.declare_parameter('control_topic', '/t0x0104_')
        self.declare_parameter('feedback_topic', '/r0x0204')
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('default_timeout', 5.0)

        self.control_topic = self.get_parameter('control_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.default_timeout = self.get_parameter('default_timeout').value

        # ---- 状态变量 ----
        self.current_state = SpearState.IDLE
        self.current_command = ''

        # 下位机反馈数据 (来自 /r0x0204)
        # [完成标志, 电机角度0-3, 力传感0-3, ...]
        self.feedback_data = [0.0] * 9

        # 各命令耗时估算 (s)
        self.command_times = {
            1: 2.0,  # prepare
            2: 3.0,  # grasp
            3: 2.0,  # dock_extend
            4: 2.0,  # dock_release
        }

        # ---- 发布/订阅 ----
        self.pub_action = self.create_publisher(
            Float32MultiArray, self.control_topic, 10)

        self.sub_feedback = self.create_subscription(
            Float32MultiArray, self.feedback_topic, self._feedback_cb, 10)

        # ---- Action Server ----
        self._action_server = ActionServer(
            self,
            SpearAction,
            'spear_action',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info('Spear Action Server initialized.')

    # ---- 传感器回调 ----
    def _feedback_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 9:
            for i in range(9):
                self.feedback_data[i] = float(msg.data[i])

    # ---- Action Server 回调 ----
    def _goal_callback(self, goal_request):
        cmd = goal_request.command
        valid_commands = [0, 1, 2, 3, 4]  # idle, prepare, grasp, dock_extend, dock_release
        if cmd not in valid_commands:
            self.get_logger().warn(f'Rejecting unknown spear command: {cmd}')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepting spear goal: command={cmd}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Spear goal cancelled, returning to safe state.')
        self._send_command(0)  # 停止指令
        self.current_state = SpearState.IDLE
        self.current_command = ''
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        command = goal_handle.request.command
        timeout_sec = float(goal_handle.request.timeout_sec)

        if timeout_sec <= 0.0:
            timeout_sec = self.default_timeout

        self.current_command = command

        # 发送指令
        self._dispatch_command(command)
        start_time = self.get_clock().now()

        feedback_msg = SpearAction.Feedback()

        while rclpy.ok():
            elapsed_sec = (self.get_clock().now() - start_time).nanoseconds / 1e9

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._send_command(0)  # 停止
                self.current_state = SpearState.IDLE
                self.current_command = ''
                return SpearAction.Result(
                    success=False,
                    message='Cancelled',
                    elapsed_sec=float(elapsed_sec),
                )

            if timeout_sec > 0.0 and elapsed_sec > timeout_sec:
                goal_handle.abort()
                self._send_command(0)  # 停止
                self.current_state = SpearState.IDLE
                self.current_command = ''
                return SpearAction.Result(
                    success=False,
                    message=f'Timeout ({timeout_sec}s)',
                    elapsed_sec=float(elapsed_sec),
                )

            # 发布反馈
            feedback_msg.state = self._state_name(self.current_state)
            feedback_msg.progress = self._estimate_progress(command, elapsed_sec, timeout_sec)
            goal_handle.publish_feedback(feedback_msg)

            # 检查完成
            if self._is_command_done(command):
                goal_handle.succeed()
                elapsed_sec = (self.get_clock().now() - start_time).nanoseconds / 1e9
                self.get_logger().info(f'Spear command {command} completed in {elapsed_sec:.2f}s')
                self.current_state = SpearState.IDLE
                self.current_command = ''
                return SpearAction.Result(
                    success=True,
                    message=f'{command} done',
                    elapsed_sec=float(elapsed_sec),
                )

            time.sleep(1.0 / self.control_freq)

        goal_handle.abort()
        self.current_state = SpearState.IDLE
        self.current_command = ''
        return SpearAction.Result(
            success=False,
            message='Node shutdown',
            elapsed_sec=float(
                (self.get_clock().now() - start_time).nanoseconds / 1e9
            ),
        )

    def _sleep_async(self, duration_sec):
        time.sleep(duration_sec)

    # ---- 命令分发 ----
    def _dispatch_command(self, command: int):
        """
        将语义命令翻译为下位机控制指令。
        发布到 /t0x0104_ (Float32MultiArray)。

        指令格式约定:
          [cmd_id, param1, param2, ...]

          cmd_id 映射:
            0 = idle / 停止
            1 = prepare (张开夹爪 + 解锁 + 移动到待抓取位姿)
            2 = grasp  (闭合夹爪 → 检测力 → 锁紧 → 抬起)
            3 = dock_extend (伸出到对接位)
            4 = dock_release (松开 + 收回)
        """
        cmd_map = {
            0: [0.0, 0.0, 0.0, 0.0],
            1: [1.0, 0.0, 0.0, 0.0],
            2: [2.0, 0.0, 0.0, 0.0],
            3: [3.0, 0.0, 0.0, 0.0],
            4: [4.0, 0.0, 0.0, 0.0],
        }

        data = cmd_map.get(command, [0.0, 0.0, 0.0, 0.0])

        state_map = {
            0: SpearState.IDLE,
            1: SpearState.PREPARING,
            2: SpearState.GRASPING,
            3: SpearState.DOCK_EXTENDING,
            4: SpearState.DOCK_RELEASING,
        }
        self.current_state = state_map.get(command, SpearState.IDLE)

        self.pub_action.publish(Float32MultiArray(data=data))
        self.get_logger().info(f'Dispatched spear command: {command} -> {data}')

    def _send_command(self, cmd_id: int):
        """发送原始指令"""
        self.pub_action.publish(Float32MultiArray(data=[float(cmd_id), 0.0, 0.0, 0.0]))

    # ---- 完成检测 ----
    def _is_command_done(self, command: int) -> bool:
        """
        通过下位机反馈判断命令是否完成。
        /r0x0204 data[0] == 1.0 表示动作完成。

        grasp 命令额外检查:
          - data[0] == 1.0 (动作完成标志)
        """
        if len(self.feedback_data) > 0:
            # data[0] 为动作完成标志位
            return self.feedback_data[0] == 1.0
        return False

    def _estimate_progress(self, command: int, elapsed: float, timeout: float) -> float:
        """基于时间的粗略进度估算 (0.0~1.0)"""
        expected = self.command_times.get(command, timeout)
        return min(elapsed / max(expected, 0.1), 0.99)

    @staticmethod
    def _state_name(state: int) -> str:
        names = {
            SpearState.IDLE: 'idle',
            SpearState.PREPARING: 'preparing',
            SpearState.GRASPING: 'grasping',
            SpearState.LOCKING: 'locking',
            SpearState.LIFTING: 'lifting',
            SpearState.DOCK_EXTENDING: 'dock_extending',
            SpearState.DOCK_RELEASING: 'dock_releasing',
            SpearState.RECOVERING: 'recovering',
            SpearState.DONE: 'done',
            SpearState.ERROR: 'error',
        }
        return names.get(state, 'unknown')


def main(args=None):
    rclpy.init(args=args)
    node = SpearActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
