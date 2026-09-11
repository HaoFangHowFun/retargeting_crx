"""44-joint backend using one BOTH lease and a synchronized gateway command."""
import time
import threading
import numpy as np

from .dual_crx import DualCrxRobotBackend
from .dual_crx_contract import BIMANUAL_CRX_NAMES, to_bimanual_crx_positions


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
        from .base import BackendStepResult
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
