import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any

import rclpy
import yaml
from action_of_motion_interfaces.action import MoveToPose
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from r2_interfaces.srv import ToolAction
from std_msgs.msg import Float32MultiArray, Int32


LAYER3_COMMANDS = {1, 2, 3}
LAYER2_COMMANDS = {4, 5, 6}
PLACE_SIGNAL = 7


class ConfigError(RuntimeError):
    pass


@dataclass
class ActionOutcome:
    success: bool
    message: str
    cancelled: bool = False


@dataclass
class MotionTarget:
    x: float
    y: float
    yaw: float
    pid_profile: int
    max_vel: float
    max_wz: float
    timeout_sec: float


@dataclass
class PrepareConfig:
    name: str
    prepare_first_timeout_sec: float
    fetch_kfs_timeout_sec: float
    prepare_second_timeout_sec: float


@dataclass
class ForwardApproachConfig:
    enabled: bool
    velocity_topic: str
    speed_mps: float
    duration_sec: float
    publish_rate_hz: float


@dataclass
class PlaceActionConfig:
    service_name: str
    mid_action: str
    high_action: str
    timeout_sec: float
    retry_attempts: int
    args: list[float]


@dataclass
class PlaceKFSConfig:
    action_name: str
    deck_topic: str
    command_timeout_sec: float
    place_signal_timeout_sec: float
    prepare_action: PrepareConfig
    forward_approach: ForwardApproachConfig
    place_action: PlaceActionConfig
    targets: dict[int, MotionTarget]
    standby: MotionTarget


def _float(value: Any, key: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f'Invalid float for {key}') from exc
    if not math.isfinite(result):
        raise ConfigError(f'Invalid finite float for {key}')
    return result


def _int(value: Any, key: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f'Invalid int for {key}') from exc


def _bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def _required(mapping: dict[str, Any], key: str, prefix: str) -> Any:
    if key not in mapping:
        raise ConfigError(f'Missing {prefix}.{key}')
    return mapping[key]


def _get(mapping: dict[str, Any], key: str, fallback: Any) -> Any:
    if not isinstance(mapping, dict):
        return fallback
    return mapping.get(key, fallback)


def _message_fields(message: Any) -> set[str]:
    getter = getattr(message, 'get_fields_and_field_types', None)
    if callable(getter):
        return set(getter().keys())
    return set(getattr(message, '__slots__', []))


def _set_if_present(message: Any, field: str, value: Any) -> None:
    if field in _message_fields(message):
        setattr(message, field, value)


class PlaceKFSActionServer(Node):
    def __init__(self) -> None:
        super().__init__('place_kfs_action_server')
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('param_config', '')
        self.config = self._load_config()
        self.PlaceKFS, self.PrepareKFS = self._load_future_action_types()

        self._deck_lock = threading.Lock()
        self._deck_sequence = 0
        self._last_deck_value: int | None = None
        self._deck_sub = self.create_subscription(
            Int32,
            self.config.deck_topic,
            self._deck_callback,
            10,
            callback_group=self.callback_group,
        )

        self._prepare_client = ActionClient(
            self,
            self.PrepareKFS,
            self.config.prepare_action.name,
            callback_group=self.callback_group,
        )
        self._motion_client = ActionClient(
            self,
            MoveToPose,
            '/move_to_pose',
            callback_group=self.callback_group,
        )
        self._tool_client = self.create_client(
            ToolAction,
            self.config.place_action.service_name,
            callback_group=self.callback_group,
        )
        self._forward_pub = self.create_publisher(
            Float32MultiArray,
            self.config.forward_approach.velocity_topic,
            10,
        )
        self._action_server = ActionServer(
            self,
            self.PlaceKFS,
            self.config.action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info('place_kfs action server ready on %s', self.config.action_name)

    def _load_future_action_types(self):
        try:
            from r2_interfaces.action import PlaceKFS, PrepareKFS
        except ImportError as exc:
            raise RuntimeError(
                'r2_interfaces.action.PlaceKFS and PrepareKFS are required '
                'before place_kfs can run.'
            ) from exc
        return PlaceKFS, PrepareKFS

    def _default_param_config_path(self) -> str:
        try:
            return os.path.join(
                get_package_share_directory('r2_bt'),
                'config',
                'param.yaml',
            )
        except Exception:
            return os.path.join(os.getcwd(), 'r2_bt', 'config', 'param.yaml')

    def _resolve_param_config_path(self, param_config: str) -> str:
        if not param_config:
            return self._default_param_config_path()
        if os.path.isabs(param_config):
            return param_config
        try:
            share_dir = get_package_share_directory('r2_bt')
            if param_config.startswith('config/'):
                return os.path.join(share_dir, param_config)
            return os.path.join(share_dir, 'config', param_config)
        except Exception:
            return param_config

    def _load_config(self) -> PlaceKFSConfig:
        param_config = self._resolve_param_config_path(
            str(self.get_parameter('param_config').value or '')
        )
        if not os.path.exists(param_config):
            raise ConfigError(f'param_config does not exist: {param_config}')

        with open(param_config, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        final_cfg = _required(data, 'final_area', 'root')
        cfg = _required(data, 'place_kfs', 'root')
        if not isinstance(final_cfg, dict) or not isinstance(cfg, dict):
            raise ConfigError('final_area and place_kfs must be YAML mappings')

        prepare = _required(cfg, 'prepare_action', 'place_kfs')
        forward = _get(cfg, 'forward_approach', {})
        place = _get(final_cfg, 'place_action', _get(cfg, 'place_action', {}))
        targets = _required(final_cfg, 'targets', 'final_area')
        standby = _required(final_cfg, 'standby', 'final_area')

        raw_args = _get(place, 'args', [0.0, 0.0, 0.0, 0.0])
        if not isinstance(raw_args, list):
            raise ConfigError('place_action.args must be a list')
        args = [_float(v, 'place_action.args') for v in raw_args[:4]]
        args.extend([0.0] * (4 - len(args)))

        return PlaceKFSConfig(
            action_name=str(_get(cfg, 'action_name', '/place_kfs')),
            deck_topic=str(_get(cfg, 'deck_topic', _get(final_cfg, 'deck_topic', '/aruco_comm/tx_id'))),
            command_timeout_sec=_float(
                _get(cfg, 'command_timeout_sec', _get(final_cfg, 'command_timeout_sec', 0.0)),
                'place_kfs.command_timeout_sec',
            ),
            place_signal_timeout_sec=_float(
                _get(cfg, 'place_signal_timeout_sec',
                     _get(final_cfg, 'place_signal_timeout_sec', 0.0)),
                'place_kfs.place_signal_timeout_sec',
            ),
            prepare_action=PrepareConfig(
                name=str(_get(prepare, 'name', '/prepare_kfs')),
                prepare_first_timeout_sec=_float(
                    _get(prepare, 'prepare_first_timeout_sec', 30.0),
                    'place_kfs.prepare_action.prepare_first_timeout_sec',
                ),
                fetch_kfs_timeout_sec=_float(
                    _get(prepare, 'fetch_kfs_timeout_sec', 30.0),
                    'place_kfs.prepare_action.fetch_kfs_timeout_sec',
                ),
                prepare_second_timeout_sec=_float(
                    _get(prepare, 'prepare_second_timeout_sec', 30.0),
                    'place_kfs.prepare_action.prepare_second_timeout_sec',
                ),
            ),
            forward_approach=ForwardApproachConfig(
                enabled=_bool(_get(forward, 'enabled', False)),
                velocity_topic=str(_get(forward, 'velocity_topic', '/t0x0111_pid')),
                speed_mps=_float(_get(forward, 'speed_mps', 0.15),
                                 'place_kfs.forward_approach.speed_mps'),
                duration_sec=_float(_get(forward, 'duration_sec', 0.5),
                                    'place_kfs.forward_approach.duration_sec'),
                publish_rate_hz=_float(_get(forward, 'publish_rate_hz', 50.0),
                                       'place_kfs.forward_approach.publish_rate_hz'),
            ),
            place_action=PlaceActionConfig(
                service_name=str(_get(place, 'service_name', '/ares_tool_node/tool_action')),
                mid_action=str(_get(place, 'mid_action', 'arm_place_mid')),
                high_action=str(_get(place, 'high_action', 'arm_place_high')),
                timeout_sec=_float(_get(place, 'timeout_sec', 30.0),
                                   'place_action.timeout_sec'),
                retry_attempts=max(1, _int(_get(place, 'retry_attempts', 3),
                                           'place_action.retry_attempts')),
                args=args,
            ),
            targets={
                1: self._load_target(targets, '3_left'),
                2: self._load_target(targets, '3_mid'),
                3: self._load_target(targets, '3_right'),
                4: self._load_target(targets, '2_left'),
                5: self._load_target(targets, '2_mid'),
                6: self._load_target(targets, '2_right'),
            },
            standby=self._load_standby(final_cfg, standby),
        )

    def _load_target(self, targets: dict[str, Any], key: str) -> MotionTarget:
        node = _required(targets, key, 'final_area.targets')
        return self._target_from_node(node, f'final_area.targets.{key}')

    def _load_standby(self, final_cfg: dict[str, Any], standby: dict[str, Any]) -> MotionTarget:
        motion = _get(standby, 'motion', _get(final_cfg, 'motion', {}))
        return MotionTarget(
            x=_float(_required(standby, 'target_x', 'final_area.standby'),
                     'final_area.standby.target_x'),
            y=_float(_required(standby, 'target_y', 'final_area.standby'),
                     'final_area.standby.target_y'),
            yaw=_float(_required(standby, 'target_yaw', 'final_area.standby'),
                       'final_area.standby.target_yaw'),
            pid_profile=_int(_get(motion, 'pid_profile', 1), 'final_area.standby.motion.pid_profile'),
            max_vel=_float(_get(motion, 'max_vel', 0.0), 'final_area.standby.motion.max_vel'),
            max_wz=_float(_get(motion, 'max_wz', 0.0), 'final_area.standby.motion.max_wz'),
            timeout_sec=_float(_get(motion, 'timeout_sec', 60.0),
                               'final_area.standby.motion.timeout_sec'),
        )

    def _target_from_node(self, node: dict[str, Any], prefix: str) -> MotionTarget:
        motion = _get(node, 'motion', {})
        return MotionTarget(
            x=_float(_required(node, 'target_x', prefix), f'{prefix}.target_x'),
            y=_float(_required(node, 'target_y', prefix), f'{prefix}.target_y'),
            yaw=_float(_required(node, 'target_yaw', prefix), f'{prefix}.target_yaw'),
            pid_profile=_int(_get(motion, 'pid_profile', 1), f'{prefix}.motion.pid_profile'),
            max_vel=_float(_get(motion, 'max_vel', 0.0), f'{prefix}.motion.max_vel'),
            max_wz=_float(_get(motion, 'max_wz', 0.0), f'{prefix}.motion.max_wz'),
            timeout_sec=_float(_get(motion, 'timeout_sec', 30.0), f'{prefix}.motion.timeout_sec'),
        )

    def _deck_callback(self, msg: Int32) -> None:
        with self._deck_lock:
            self._deck_sequence += 1
            self._last_deck_value = int(msg.data)

    def _goal_callback(self, goal_request) -> GoalResponse:
        deck_command = int(getattr(goal_request, 'deck_command', 0))
        if deck_command != 0 and deck_command not in LAYER3_COMMANDS | LAYER2_COMMANDS:
            self.get_logger().warning('Rejecting place_kfs goal: invalid deck_command=%d',
                                      deck_command)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested for place_kfs')
        return CancelResponse.ACCEPT

    def _make_result(self, success: bool, message: str, deck_command: int,
                     flow: str, start_time: float):
        result = self.PlaceKFS.Result()
        _set_if_present(result, 'success', bool(success))
        _set_if_present(result, 'message', message)
        _set_if_present(result, 'deck_command', int(deck_command))
        _set_if_present(result, 'flow', flow)
        _set_if_present(result, 'elapsed_sec', float(time.monotonic() - start_time))
        return result

    def _publish_feedback(self, goal_handle, phase: str, start_time: float,
                          deck_command: int = 0, message: str = '') -> None:
        feedback = self.PlaceKFS.Feedback()
        _set_if_present(feedback, 'phase', phase)
        _set_if_present(feedback, 'elapsed_sec', float(time.monotonic() - start_time))
        _set_if_present(feedback, 'deck_command', int(deck_command))
        _set_if_present(feedback, 'message', message)
        goal_handle.publish_feedback(feedback)

    def _wait_for_future(self, future: Future, timeout_sec: float,
                         goal_handle, start_time: float) -> str:
        while rclpy.ok() and not future.done():
            if goal_handle.is_cancel_requested:
                return 'cancelled'
            if timeout_sec > 0.0 and time.monotonic() - start_time > timeout_sec:
                return 'timeout'
            time.sleep(0.02)
        return 'done' if future.done() else 'shutdown'

    def _call_action(self, client: ActionClient, goal_msg: Any, timeout_sec: float,
                     goal_handle, label: str) -> ActionOutcome:
        if not client.wait_for_server(timeout_sec=3.0):
            return ActionOutcome(False, f'{label} action server not available')

        start_time = time.monotonic()
        send_future = client.send_goal_async(goal_msg)
        state = self._wait_for_future(send_future, timeout_sec, goal_handle, start_time)
        if state != 'done':
            return ActionOutcome(False, f'{label} {state} before goal accepted',
                                 cancelled=(state == 'cancelled'))

        child_goal_handle: ClientGoalHandle = send_future.result()
        if child_goal_handle is None or not child_goal_handle.accepted:
            return ActionOutcome(False, f'{label} goal rejected')

        result_future = child_goal_handle.get_result_async()
        state = self._wait_for_future(result_future, timeout_sec, goal_handle, start_time)
        if state == 'cancelled':
            child_goal_handle.cancel_goal_async()
            return ActionOutcome(False, f'{label} cancelled', cancelled=True)
        if state == 'timeout':
            child_goal_handle.cancel_goal_async()
            return ActionOutcome(False, f'{label} timed out')
        if state != 'done':
            return ActionOutcome(False, f'{label} interrupted by {state}')

        wrapped = result_future.result()
        result = getattr(wrapped, 'result', None)
        code = getattr(wrapped, 'code', None)
        if code == GoalStatus.STATUS_SUCCEEDED and (
            result is None or bool(getattr(result, 'success', True))
        ):
            return ActionOutcome(True, str(getattr(result, 'message', 'success')))
        return ActionOutcome(False, str(getattr(result, 'message', f'{label} failed')))

    def _prepare_command_value(self, name: str, fallback: int) -> int:
        return int(getattr(self.PrepareKFS.Goal, name, fallback))

    def _prepare_goal(self, command: int, deck_command: int, timeout_sec: float):
        goal = self.PrepareKFS.Goal()
        _set_if_present(goal, 'command', int(command))
        _set_if_present(goal, 'deck_command', int(deck_command))
        _set_if_present(goal, 'timeout_sec', float(timeout_sec))
        return goal

    def _call_prepare(self, command_name: str, fallback: int, deck_command: int,
                      timeout_sec: float, goal_handle) -> ActionOutcome:
        command = self._prepare_command_value(command_name, fallback)
        goal = self._prepare_goal(command, deck_command, timeout_sec)
        return self._call_action(
            self._prepare_client,
            goal,
            timeout_sec,
            goal_handle,
            f'PrepareKFS:{command_name}',
        )

    def _wait_for_deck(self, expected: set[int], timeout_sec: float, goal_handle,
                       start_time: float, feedback_phase: str) -> ActionOutcome:
        with self._deck_lock:
            start_sequence = self._deck_sequence

        wait_start = time.monotonic()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return ActionOutcome(False, 'cancelled', cancelled=True)
            with self._deck_lock:
                sequence = self._deck_sequence
                value = self._last_deck_value
            if sequence > start_sequence and value in expected:
                return ActionOutcome(True, str(value))
            if timeout_sec > 0.0 and time.monotonic() - wait_start > timeout_sec:
                return ActionOutcome(False, f'timed out waiting for deck {sorted(expected)}')
            self._publish_feedback(
                goal_handle,
                feedback_phase,
                start_time,
                message=f'waiting for deck {sorted(expected)}',
            )
            time.sleep(0.05)
        return ActionOutcome(False, 'shutdown while waiting for deck')

    def _call_motion(self, target: MotionTarget, goal_handle, label: str) -> ActionOutcome:
        goal = MoveToPose.Goal()
        goal.x = target.x
        goal.y = target.y
        goal.yaw_deg = math.degrees(target.yaw)
        goal.pid_profile = int(target.pid_profile)
        goal.max_vel = target.max_vel
        goal.max_wz = target.max_wz
        return self._call_action(
            self._motion_client,
            goal,
            target.timeout_sec,
            goal_handle,
            label,
        )

    def _run_forward_approach(self, goal_handle) -> ActionOutcome:
        cfg = self.config.forward_approach
        if not cfg.enabled or cfg.duration_sec <= 0.0 or abs(cfg.speed_mps) <= 1e-6:
            return ActionOutcome(True, 'forward approach disabled')

        period = 1.0 / cfg.publish_rate_hz if cfg.publish_rate_hz > 0.0 else 0.02
        msg = Float32MultiArray()
        msg.data = [float(cfg.speed_mps), 0.0, 0.0]
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < cfg.duration_sec:
            if goal_handle.is_cancel_requested:
                self._publish_zero_velocity()
                return ActionOutcome(False, 'cancelled', cancelled=True)
            self._forward_pub.publish(msg)
            time.sleep(period)
        self._publish_zero_velocity()
        return ActionOutcome(True, 'forward approach complete')

    def _publish_zero_velocity(self) -> None:
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self._forward_pub.publish(msg)

    def _call_place(self, deck_command: int, goal_handle) -> ActionOutcome:
        action = (
            self.config.place_action.high_action
            if deck_command in LAYER3_COMMANDS
            else self.config.place_action.mid_action
        )
        if not self._tool_client.wait_for_service(timeout_sec=3.0):
            return ActionOutcome(False, 'ToolAction service not available')

        last_message = ''
        for attempt in range(1, self.config.place_action.retry_attempts + 1):
            if goal_handle.is_cancel_requested:
                return ActionOutcome(False, 'cancelled', cancelled=True)
            request = ToolAction.Request()
            request.action = action
            request.args = [float(v) for v in self.config.place_action.args[:4]]
            start_time = time.monotonic()
            future = self._tool_client.call_async(request)
            state = self._wait_for_future(
                future,
                self.config.place_action.timeout_sec,
                goal_handle,
                start_time,
            )
            if state == 'cancelled':
                return ActionOutcome(False, 'cancelled', cancelled=True)
            if state != 'done':
                last_message = f'place {action} {state}'
            else:
                response = future.result()
                if response is not None and response.success and response.ret == 0:
                    return ActionOutcome(True, response.message or 'place completed')
                last_message = (
                    response.message if response is not None
                    else f'place {action} failed'
                )
            self.get_logger().warning(
                'Place attempt %d/%d failed: %s',
                attempt,
                self.config.place_action.retry_attempts,
                last_message,
            )

        return ActionOutcome(False, last_message or 'place failed')

    def _execute_callback(self, goal_handle):
        start_time = time.monotonic()
        deck_command = int(getattr(goal_handle.request, 'deck_command', 0))
        goal_timeout = float(getattr(goal_handle.request, 'timeout_sec', 0.0))
        command_timeout = goal_timeout if goal_timeout > 0.0 else self.config.command_timeout_sec
        flow = ''

        def fail(message: str):
            self._publish_zero_velocity()
            self.get_logger().error('place_kfs failed: %s', message)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return self._make_result(False, message, deck_command, flow, start_time)

        def run_step(phase: str, callback):
            self._publish_feedback(goal_handle, phase, start_time, deck_command)
            outcome = callback()
            if not outcome.success:
                return outcome
            self.get_logger().info('%s complete: %s', phase, outcome.message)
            return outcome

        if deck_command == 0:
            outcome = self._wait_for_deck(
                LAYER3_COMMANDS | LAYER2_COMMANDS,
                command_timeout,
                goal_handle,
                start_time,
                'WAIT_DECK_COMMAND',
            )
            if not outcome.success:
                return fail(outcome.message)
            deck_command = int(outcome.message)

        target = self.config.targets[deck_command]
        flow = 'LAYER3' if deck_command in LAYER3_COMMANDS else 'LAYER2'
        self.get_logger().info('place_kfs started: deck=%d flow=%s',
                               deck_command, flow)

        outcome = run_step(
            'PREPARE_FIRST',
            lambda: self._call_prepare(
                'CMD_PREPARE_FIRST',
                1,
                deck_command,
                self.config.prepare_action.prepare_first_timeout_sec,
                goal_handle,
            ),
        )
        if not outcome.success:
            return fail(outcome.message)

        outcome = run_step(
            'MOVE_TARGET',
            lambda: self._call_motion(target, goal_handle, 'MoveToPose:place_target'),
        )
        if not outcome.success:
            return fail(outcome.message)

        outcome = run_step('FORWARD_APPROACH', lambda: self._run_forward_approach(goal_handle))
        if not outcome.success:
            return fail(outcome.message)

        if deck_command in LAYER2_COMMANDS:
            outcome = run_step('PLACE_FIRST', lambda: self._call_place(deck_command, goal_handle))
            if not outcome.success:
                return fail(outcome.message)

            outcome = run_step(
                'FETCH_KFS',
                lambda: self._call_prepare(
                    'CMD_FETCH_KFS',
                    2,
                    deck_command,
                    self.config.prepare_action.fetch_kfs_timeout_sec,
                    goal_handle,
                ),
            )
            if not outcome.success:
                return fail(outcome.message)

            outcome = run_step(
                'PREPARE_SECOND',
                lambda: self._call_prepare(
                    'CMD_PREPARE_SECOND',
                    3,
                    deck_command,
                    self.config.prepare_action.prepare_second_timeout_sec,
                    goal_handle,
                ),
            )
            if not outcome.success:
                return fail(outcome.message)

            outcome = self._wait_for_deck(
                {PLACE_SIGNAL},
                self.config.place_signal_timeout_sec,
                goal_handle,
                start_time,
                'WAIT_SECOND_PLACE_SIGNAL',
            )
            if not outcome.success:
                return fail(outcome.message)

            outcome = run_step('PLACE_SECOND', lambda: self._call_place(deck_command, goal_handle))
            if not outcome.success:
                return fail(outcome.message)
        else:
            outcome = self._wait_for_deck(
                {PLACE_SIGNAL},
                self.config.place_signal_timeout_sec,
                goal_handle,
                start_time,
                'WAIT_PLACE_SIGNAL',
            )
            if not outcome.success:
                return fail(outcome.message)

            outcome = run_step('PLACE', lambda: self._call_place(deck_command, goal_handle))
            if not outcome.success:
                return fail(outcome.message)

        outcome = run_step(
            'MOVE_STANDBY',
            lambda: self._call_motion(self.config.standby, goal_handle, 'MoveToPose:standby'),
        )
        if not outcome.success:
            return fail(outcome.message)

        self._publish_feedback(goal_handle, 'DONE', start_time, deck_command)
        goal_handle.succeed()
        return self._make_result(
            True,
            f'place_kfs complete: deck={deck_command} flow={flow}',
            deck_command,
            flow,
            start_time,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = PlaceKFSActionServer()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
