"""JointState transport for two Sharpa hands, optionally with both CRX arms."""

import math
import threading
import time

import numpy as np

from teleoperation.backends.base import BackendStepResult
from teleoperation.backends.sharpa_contract import joint_channels, named_positions


class SharpaJointBackend:
    def __init__(self, *, robot_names, initial_qpos, lower, upper, control_period,
                 startup_timeout=5., feedback_timeout=.5, target_timeout=.25,
                 crx_namespace='crx5ia', sharpa_namespace='sharpa'):
        self.channels = joint_channels(robot_names)
        self._size = sum(map(len, robot_names))
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        if (self.lower.shape != (self._size,) or self.upper.shape != (self._size,)
                or not np.isfinite(self.lower).all() or not np.isfinite(self.upper).all()
                or np.any(self.lower > self.upper)):
            raise ValueError('Expected finite model bounds for every joint')
        self._actual = self._validate_command(initial_qpos)
        self._target = self._actual.copy()
        for value in (control_period, startup_timeout, feedback_timeout, target_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Periods and timeouts must be finite and positive')
        self.control_period = control_period
        self._feedback_timeout, self._target_timeout = feedback_timeout, target_timeout
        self._lock = threading.RLock()
        self._closed = self._stopped = self._paused = False
        self._last_command_at = None
        self._spin_error = None
        self._feedback = {key: dict(stamp=0, received=0., advanced=0., error='waiting for feedback')
                          for key in self.channels}
        self._publishers = {}
        self._context = self._node = self._executor = self._thread = self._watchdog = None
        try:
            import rclpy
            from rclpy.clock import Clock, ClockType
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from sensor_msgs.msg import JointState
        except ImportError as exc:
            raise RuntimeError('Source ROS Jazzy and use the ROS-compatible project venv') from exc
        self._message_type = JointState
        try:
            self._context = Context()
            rclpy.init(args=[], context=self._context)
            self._node = Node('retargeting_sharpa_joint', context=self._context)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            for key in self.channels:
                namespace = crx_namespace.strip('/') if key == 'arms' else f'{sharpa_namespace.strip("/")}/{key}'
                if not namespace or namespace.startswith('/'):
                    raise ValueError('Namespaces must not be empty')
                target = 'joint_targets' if key == 'arms' else 'joint_command'
                self._publishers[key] = self._node.create_publisher(JointState, f'/{namespace}/{target}', qos)
                self._node.create_subscription(
                    JointState, f'/{namespace}/joint_states',
                    lambda msg, key=key: self._receive_state(key, msg), qos)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._spin, daemon=True, name='sharpa_feedback')
            self._thread.start()
            deadline = time.monotonic() + startup_timeout
            while True:
                try:
                    with self._lock:
                        self._check_feedback()
                        self._target = self._validate_command(self._actual)
                    break
                except RuntimeError as exc:
                    if time.monotonic() >= deadline:
                        raise RuntimeError(f'Sharpa startup timeout: {exc}') from exc
                    time.sleep(.01)
            self._watchdog = self._node.create_timer(
                .02, self._watchdog_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        except BaseException:
            self.close()
            raise

    def _validate_command(self, qpos):
        values = np.asarray(qpos, dtype=float)
        if values.shape != (self._size,) or not np.isfinite(values).all():
            raise ValueError(f'Expected {self._size} finite joint positions')
        if np.any(values < self.lower - 1e-7) or np.any(values > self.upper + 1e-7):
            raise ValueError('Joint command exceeds model limits')
        return values.copy()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception as exc:
            with self._lock:
                self._spin_error = str(exc)
                self._stopped = True

    def _receive_state(self, key, message):
        with self._lock:
            state = self._feedback[key]
            try:
                names, indices = self.channels[key]
                values = named_positions(message.name, message.position, names)
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                age = (self._node.get_clock().now().nanoseconds - stamp) / 1e9
                if stamp <= 0 or not -.02 <= age <= self._feedback_timeout or stamp < state['stamp']:
                    raise ValueError('stale, future or backwards feedback timestamp')
            except (ValueError, TypeError) as exc:
                state['error'] = str(exc)
                return
            now = time.monotonic()
            if stamp > state['stamp']:
                state['advanced'] = now
            state.update(stamp=stamp, received=now, error=None)
            self._actual[indices] = values

    def _check_feedback(self):
        if self._stopped:
            raise RuntimeError(f'Sharpa output stopped: {self._spin_error or "stop requested"}')
        if (self._context is None or self._thread is None or self._node is None
                or not self._context.ok() or not self._thread.is_alive()):
            raise RuntimeError('ROS executor unavailable')
        now, ros_now = time.monotonic(), self._node.get_clock().now().nanoseconds
        for key, state in self._feedback.items():
            if state['error']:
                raise RuntimeError(f'{key}: {state["error"]}')
            if (now - state['received'] > self._feedback_timeout
                    or now - state['advanced'] > self._feedback_timeout
                    or not -.02 <= (ros_now - state['stamp']) / 1e9 <= self._feedback_timeout):
                raise RuntimeError(
                    f'{key}: stale or frozen feedback '
                    f'(received={now - state["received"]:.3f}s, '
                    f'advanced={now - state["advanced"]:.3f}s, '
                    f'stamp_age={(ros_now - state["stamp"]) / 1e9:.3f}s)'
                )
            if self._publishers[key].get_subscription_count() == 0:
                raise RuntimeError(f'{key}: command subscriber missing')

    def _publish(self, values):
        # Validate all groups before any publication. Topics are not atomic transport.
        values = self._validate_command(values)
        stamp = self._node.get_clock().now().to_msg()
        messages = []
        for key, (names, indices) in self.channels.items():
            msg = self._message_type()
            msg.header.stamp = stamp
            msg.name, msg.position = list(names), values[indices].tolist()
            messages.append((key, msg))
        for key, msg in messages:
            self._publishers[key].publish(msg)

    def get_joint_pos(self):
        with self._lock:
            self._check_feedback()
            return self._actual.copy()

    def get_target_joint_pos(self):
        with self._lock:
            return self._target.copy()

    def execute(self, qpos):
        values = self._validate_command(qpos)
        with self._lock:
            self._check_feedback()
            if self._paused:
                raise RuntimeError('Output paused; recalibrate before resuming')
            self._publish(values)
            self._target = values
            self._last_command_at = time.monotonic()
            return BackendStepResult(command_qpos=values, actual_qpos=self._actual.copy(), diagnostics={})

    def _pause_locked(self):
        if self._paused:
            return
        self._paused = True
        self._last_command_at = None
        try:
            self._check_feedback()
            self._publish(self._actual)  # One measured hold target, never a repeated stale goal.
            self._target = self._actual.copy()
        except (RuntimeError, ValueError):
            pass  # Missing feedback: stop sending; drivers own their timeout policy.

    def _watchdog_tick(self):
        with self._lock:
            if self._closed or self._stopped or self._last_command_at is None:
                return
            try:
                self._check_feedback()
                if time.monotonic() - self._last_command_at > self._target_timeout:
                    raise RuntimeError('No fresh retargeting target')
            except RuntimeError as exc:
                self._pause_locked()
                self._node.get_logger().warning(str(exc))

    def pause_tracking(self):
        with self._lock:
            self._pause_locked()

    def resume_tracking(self):
        with self._lock:
            try:
                self._check_feedback()
                self._validate_command(self._actual)
            except (RuntimeError, ValueError):
                return False
            self._target = self._actual.copy()
            self._paused = False
            self._last_command_at = None
            return True

    def assert_tracking(self):
        with self._lock:
            if self._stopped:
                raise RuntimeError(f'Output stopped: {self._spin_error or "stop requested"}')
            try:
                self._check_feedback()
            except RuntimeError:
                self._pause_locked()
                return False
            return not self._paused

    def request_stop(self, reason):
        with self._lock:
            self._pause_locked()
            self._stopped = True

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._pause_locked()
            self._closed = self._stopped = True
            if self._watchdog is not None:
                self._watchdog.cancel()
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join()
        if self._node is not None:
            self._node.destroy_node()
        if self._context is not None:
            self._context.try_shutdown()
