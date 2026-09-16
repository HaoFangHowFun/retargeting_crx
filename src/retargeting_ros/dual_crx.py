"""ROS2 backends for right-only and synchronized bimanual CRX+LEAP output."""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from typing import Any

import numpy as np

from teleoperation.backends.base import BackendStepResult
from teleoperation.backends.dual_crx_contract import (
    BIMANUAL_CRX_NAMES, DUAL_CRX_NAMES, to_bimanual_crx_positions, to_dual_crx_positions,
)


class DualCrxRobotBackend:
    """Publish CRX+LEAP targets through the dual-crx control gateway."""

    NAMES = DUAL_CRX_NAMES
    SCOPE = 2
    TOPIC = "/dual_crx/teleop"
    positions = staticmethod(to_dual_crx_positions)

    def __init__(self, *, initial_qpos: Sequence[float], control_period: float, client_id: str = "retargeting_crx"):
        values = np.asarray(initial_qpos, dtype=float)
        if values.shape != (len(self.NAMES),) or not np.isfinite(values).all():
            raise ValueError(f"dual_crx initial_qpos must be finite with {len(self.NAMES)} positions.")
        if control_period <= 0:
            raise ValueError("control_period must be positive.")
        try:
            import rclpy
            from dual_crx_interfaces.msg import LeapState, SystemState, TeleopCommand
            from dual_crx_interfaces.srv import AcquireControl, Heartbeat, ReleaseControl, SetTeleop, SoftwareStop
            from std_srvs.srv import SetBool
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
        except ImportError as exc:
            raise RuntimeError("dual_crx backend requires a ROS-compatible Python environment.") from exc
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("retargeting_crx_dual_backend")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._state_lock = threading.Lock()
        self._state = None
        self._hand_state = None
        self._feedback_at = {"arm": 0.0, "hand": 0.0}
        self._target = values.copy()
        self._actual = values.copy()
        self._period = float(control_period)
        self._client_id = client_id
        self._scope = self.SCOPE
        self._lease_duration = 5.0
        self._publisher = self._node.create_publisher(TeleopCommand, self.TOPIC + "/command", 1)
        self._node.create_subscription(SystemState, "/dual_crx/state", self._state_cb, 10)
        self._clients = {
            "acquire": self._node.create_client(AcquireControl, "/dual_crx/acquire_control"),
            "heartbeat": self._node.create_client(Heartbeat, "/dual_crx/heartbeat"),
            "release": self._node.create_client(ReleaseControl, "/dual_crx/release_control"),
            "teleop": self._node.create_client(SetTeleop, self.TOPIC + "/enable"),
            "stop": self._node.create_client(SoftwareStop, "/dual_crx/stop"),
        }
        self._create_hand_channels()
        self._heartbeat = self._node.create_timer(1.0, self._renew_lease)
        self._started = False
        self._acquired = False
        self._hand_requested = False
        self._has_executed = False
        self._startup_timer = None
        self._thread.start()
        try:
            self._start()
        except BaseException:
            self.close()
            raise

    @property
    def control_period(self) -> float:
        return self._period

    def _state_cb(self, message: Any) -> None:
        with self._state_lock:
            self._state = message
            values = dict(zip(message.right_joints.name, message.right_joints.position))
            self._feedback_at["arm"] = 0.0
            if message.fresh and all(name in values and np.isfinite(values[name]) for name in DUAL_CRX_NAMES[:6]):
                self._actual[:6] = [values[name] for name in DUAL_CRX_NAMES[:6]]
                self._feedback_at["arm"] = time.monotonic()

    def _hand_state_cb(self, message: Any) -> None:
        with self._state_lock:
            self._hand_state = message
            values = dict(zip(message.joints.name, message.joints.position))
            self._feedback_at["hand"] = 0.0
            stamp = message.joints.header.stamp
            age = self._node.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9
            if message.healthy and -.02 <= age <= .3 and all(name in values and np.isfinite(values[name]) for name in DUAL_CRX_NAMES[6:]):
                self._actual[6:] = [values[name] for name in DUAL_CRX_NAMES[6:]]
                self._feedback_at["hand"] = time.monotonic()

    def _call(self, key: str, request: Any, timeout: float = 5.0) -> Any:
        client = self._clients[key]
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"dual-crx service unavailable: {client.srv_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f"timeout calling {client.srv_name}")
        result = future.result()
        if hasattr(result, "accepted") and not result.accepted:
            raise RuntimeError(result.reason)
        if hasattr(result, "success") and not result.success:
            raise RuntimeError(result.message)
        return result

    def _seed_from_feedback(self) -> None:
        with self._state_lock:
            now = time.monotonic()
            if any(stamp <= 0 or now - stamp > .5 for stamp in self._feedback_at.values()):
                raise RuntimeError("fresh complete arm and hand feedback required for startup")
            self._target = self._actual.copy()

    def _start(self) -> None:
        deadline = time.monotonic() + 10.0
        while self._state is None and time.monotonic() < deadline:
            time.sleep(0.01)
        if self._state is None:
            raise RuntimeError("timed out waiting for dual-crx state")
        self._call("acquire", self._request("acquire"))
        self._acquired = True
        self._enable_hands()
        self._seed_from_feedback()
        deadline = time.monotonic() + 10.0
        while True:
            try:
                self._call("teleop", self._request("teleop"))
                break
            except RuntimeError as exc:
                if "wait for initial Servo configuration" not in str(exc):
                    raise
                if time.monotonic() >= deadline:
                    raise RuntimeError("timed out waiting for initial Servo configuration") from exc
                time.sleep(0.1)
        self._started = True
        self._startup_timer = self._node.create_timer(self._period, self._publish_startup_target)

    def _create_hand_channels(self):
        from dual_crx_interfaces.msg import LeapState
        from std_srvs.srv import SetBool
        self._node.create_subscription(LeapState, "/right_leap/state", self._hand_state_cb, 10)
        self._clients["hand"] = self._node.create_client(SetBool, "/right_leap/enable")

    def _enable_hands(self):
        self._hand_requested = True
        self._call("hand", self._request("hand"))
        deadline = time.monotonic() + 5.0
        while (self._hand_state is None or not self._hand_state.enabled) and time.monotonic() < deadline:
            time.sleep(.01)
        if self._hand_state is None or not self._hand_state.enabled:
            raise RuntimeError("timed out waiting for enabled hand feedback")

    def _hand_cleanup_actions(self):
        from std_srvs.srv import SetBool
        return [("hand", SetBool.Request(data=False))] if self._hand_requested else []

    def _publish_startup_target(self) -> None:
        if not self._started or self._has_executed:
            return
        self._publish_target(self.positions(self._target))

    def _publish_target(self, positions) -> None:
        """Build the same named message for startup holds and new solved targets."""
        from dual_crx_interfaces.msg import TeleopCommand
        message = TeleopCommand(client_id=self._client_id)
        message.target.header.stamp = self._node.get_clock().now().to_msg()
        message.target.name = list(self.NAMES)
        message.target.position = list(positions)
        self._publisher.publish(message)

    def _request(self, key: str) -> Any:
        if key == "acquire":
            from dual_crx_interfaces.srv import AcquireControl
            return AcquireControl.Request(client_id=self._client_id, source_type="RETARGETING_CRX",
                                          arm_scope=self._scope, requested_mode="TELEOP",
                                          lease_duration=self._lease_duration)
        if key == "hand":
            from std_srvs.srv import SetBool
            return SetBool.Request(data=True)
        if key == "teleop":
            from dual_crx_interfaces.srv import SetTeleop
            return SetTeleop.Request(client_id=self._client_id, enabled=True)
        raise ValueError(key)

    def _renew_lease(self) -> None:
        if not self._acquired:
            return
        from dual_crx_interfaces.srv import Heartbeat
        client = self._clients["heartbeat"]
        if not client.service_is_ready():
            self._node.get_logger().error("dual-crx heartbeat service unavailable")
            return
        future = client.call_async(Heartbeat.Request(
            client_id=self._client_id,
            arm_scope=self._scope,
            lease_duration=self._lease_duration,
        ))

        def check_result(done: Any) -> None:
            try:
                result = done.result()
                if not result.accepted:
                    self._node.get_logger().error(
                        f"dual-crx heartbeat rejected: {result.reason}"
                    )
            except Exception as exc:
                self._node.get_logger().error(f"dual-crx heartbeat failed: {exc}")

        future.add_done_callback(check_result)

    def reset(self, qpos: Sequence[float] | None = None) -> None:
        if qpos is not None:
            self._target = np.asarray(qpos, dtype=float).copy()
        self.execute(self._target)

    def get_joint_pos(self) -> np.ndarray:
        with self._state_lock:
            return self._actual.copy()

    def get_target_joint_pos(self) -> np.ndarray:
        return self._target.copy()

    def execute(self, qpos: np.ndarray) -> BackendStepResult:
        values = np.asarray(qpos, dtype=float)
        positions = self.positions(values)
        self._has_executed = True
        self._publish_target(positions)
        self._target = values.copy()
        return BackendStepResult(command_qpos=self._target, actual_qpos=self.get_joint_pos(), diagnostics={})

    def close(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
        # Cleanup also covers partial startup, including hand enable followed by
        # a rejected gateway enable. Each action is attempted independently.
        from dual_crx_interfaces.srv import SoftwareStop, ReleaseControl
        from std_srvs.srv import SetBool
        actions = []
        if self._acquired:
            actions.append(("stop", SoftwareStop.Request(reason="retargeting shutdown")))
        actions.extend(self._hand_cleanup_actions())
        if self._acquired:
            actions.append(("release", ReleaseControl.Request(client_id=self._client_id, arm_scope=self._scope)))
        for key, request in actions:
            try:
                self._call(key, request)
            except Exception as exc:
                self._node.get_logger().error(f"dual-crx cleanup {key} failed: {exc}")
        self._started = self._acquired = self._hand_requested = False
        self._executor.shutdown()
        self._thread.join(timeout=2.0)
        self._node.destroy_node()



class BimanualCrxRobotBackend(DualCrxRobotBackend):
    NAMES = BIMANUAL_CRX_NAMES
    SCOPE = 3
    TOPIC = "/dual_crx/teleop/bimanual"
    positions = staticmethod(to_bimanual_crx_positions)

    def __init__(self, *, initial_qpos, control_period, left_hand_enabled=False,
                 right_hand_enabled=True, client_id="retargeting_bimanual"):
        self._output_lock = threading.Lock()
        self._output_stopped = False
        self.hand_enabled = {"left": left_hand_enabled, "right": right_hand_enabled}
        self._hands = {}
        self._requested_hands = set()
        super().__init__(initial_qpos=initial_qpos, control_period=control_period, client_id=client_id)

    def _create_hand_channels(self):
        from dual_crx_interfaces.msg import LeapState
        from std_srvs.srv import SetBool
        from dual_crx_interfaces.srv import SetTeleop
        self._clients["pause_tracking"] = self._node.create_client(SetTeleop, self.TOPIC + "/pause_tracking")
        self._feedback_at = {"left_arm": 0., "right_arm": 0.}
        for side, enabled in self.hand_enabled.items():
            if enabled:
                self._feedback_at[side + "_hand"] = 0.
                self._node.create_subscription(LeapState, f"/{side}_leap/state",
                                              lambda msg, s=side: self._receive_hand(s, msg), 10)
                self._clients[side + "_hand"] = self._node.create_client(SetBool, f"/{side}_leap/enable")

    def _state_cb(self, message):
        with self._state_lock:
            self._state = message
            for side, offset in (("left", 0), ("right", 22)):
                joints = getattr(message, side + "_joints")
                values = dict(zip(joints.name, joints.position))
                names = self.NAMES[offset:offset + 6]
                self._feedback_at[side + "_arm"] = 0.
                if message.fresh and all(n in values and np.isfinite(values[n]) for n in names):
                    self._actual[offset:offset + 6] = [values[n] for n in names]
                    self._feedback_at[side + "_arm"] = time.monotonic()

    def _receive_hand(self, side, message):
        with self._state_lock:
            self._hands[side] = message
            key = side + "_hand"
            self._feedback_at[key] = 0.
            offset = 6 if side == "left" else 28
            names = self.NAMES[offset:offset + 16]
            values = dict(zip(message.joints.name, message.joints.position))
            stamp = message.joints.header.stamp
            age = self._node.get_clock().now().nanoseconds / 1e9 - stamp.sec - stamp.nanosec / 1e9
            if message.healthy and -.02 <= age <= .3 and all(n in values and np.isfinite(values[n]) for n in names):
                self._actual[offset:offset + 16] = [values[n] for n in names]
                self._feedback_at[key] = time.monotonic()

    def _enable_hands(self):
        from std_srvs.srv import SetBool
        for side, enabled in self.hand_enabled.items():
            if not enabled:
                continue
            self._requested_hands.add(side)
            self._call(side + "_hand", SetBool.Request(data=True))
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                state = self._hands.get(side)
                if state is not None and state.enabled and state.healthy:
                    break
                time.sleep(.01)
            else:
                raise RuntimeError(f"timed out waiting for enabled {side} hand feedback")
        # Hand enable and SystemState arrive on independent subscriptions. Wait
        # for a complete fresh seed instead of racing the first arm-state update.
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            with self._state_lock:
                now = time.monotonic()
                if all(stamp > 0 and now - stamp <= .5 for stamp in self._feedback_at.values()):
                    return
            time.sleep(.01)
        raise RuntimeError("timed out waiting for fresh complete arm and hand feedback")

    def _hand_cleanup_actions(self):
        # Every session exit disables requested hands, including timed stops.
        from std_srvs.srv import SetBool
        return [(side + "_hand", SetBool.Request(data=False)) for side in sorted(self._requested_hands)]

    def execute(self, qpos):
        with self._output_lock:
            if self._output_stopped:
                raise RuntimeError("dual-arm command output has been stopped")
            result = super().execute(qpos)
        now = time.monotonic()
        diagnostics = {}
        for side, enabled in self.hand_enabled.items():
            diagnostics[side + "_hand_output_enabled"] = float(enabled)
            stamp = self._feedback_at.get(side + "_hand", 0.)
            diagnostics[side + "_hand_feedback_fresh"] = float(enabled and stamp > 0 and now - stamp <= .5)
        return BackendStepResult(command_qpos=result.command_qpos, actual_qpos=result.actual_qpos,
                                 diagnostics=diagnostics)

    def _publish_startup_target(self):
        with self._output_lock:
            if not self._output_stopped:
                super()._publish_startup_target()

    def pause_tracking(self):
        from dual_crx_interfaces.srv import SetTeleop
        self._call("pause_tracking", SetTeleop.Request(client_id=self._client_id, enabled=True))

    def resume_tracking(self):
        from dual_crx_interfaces.srv import SetTeleop
        if self._output_stopped:
            raise RuntimeError("dual-arm command output has been stopped")
        self._enable_hands()
        self._seed_from_feedback()
        if self._output_stopped:
            raise RuntimeError("dual-arm command output has been stopped")
        self._call("pause_tracking", SetTeleop.Request(client_id=self._client_id, enabled=False))

    def request_stop(self, reason):
        """Latch output off and request gateway stop without blocking the timer."""
        from dual_crx_interfaces.srv import SoftwareStop
        with self._output_lock:
            self._output_stopped = True
            if self._startup_timer is not None:
                self._startup_timer.cancel()
        try:
            client = self._clients["stop"]
            if not client.service_is_ready():
                raise RuntimeError("stop service unavailable; command output is stopped")
            future = client.call_async(SoftwareStop.Request(reason=reason))
            def completed(done):
                try:
                    if not done.result().stopped:
                        raise RuntimeError("gateway did not confirm stop")
                except Exception as exc:
                    self._node.get_logger().error(f"Timed stop: {exc}")
            future.add_done_callback(completed)
        except Exception as exc:
            self._node.get_logger().error(f"Timed stop: {exc}")

    def assert_tracking(self):
        """Fail visibly after a gateway latch instead of silently publishing forever."""
        with self._state_lock:
            if self._state is None:
                raise RuntimeError("dual-arm gateway state unavailable")
            if self._state.active_command != "bimanual teleop":
                raise RuntimeError(f"dual-arm teleop stopped: {self._state.reason}")
            now = time.monotonic()
            if any(now - self._feedback_at[side + "_arm"] > .5 for side in ("left", "right")):
                raise RuntimeError("dual-arm feedback stale")


__all__ = ["DualCrxRobotBackend", "BimanualCrxRobotBackend"]
