"""Quest retargeting backend that publishes LEAP targets only."""
from __future__ import annotations
import threading
import time
from collections.abc import Sequence
import numpy as np
from teleoperation.backends.base import BackendStepResult
from teleoperation.backends.dual_crx_contract import DUAL_CRX_NAMES

class LeapOnlyRobotBackend:
    """Own LEAP enable/command lifecycle without acquiring CRX control."""
    def __init__(self, *, initial_qpos: Sequence[float], control_period: float):
        values=np.asarray(initial_qpos,dtype=float)
        if values.shape!=(22,) or not np.isfinite(values).all(): raise ValueError("leap_only initial_qpos must be finite shape (22,)")
        if control_period<=0: raise ValueError("control_period must be positive")
        try:
            import rclpy
            from dual_crx_interfaces.msg import LeapState
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from std_srvs.srv import SetBool
        except ImportError as exc: raise RuntimeError("leap_only backend requires ROS Jazzy") from exc
        self._rclpy=rclpy
        self._JointState=JointState
        if not rclpy.ok(): rclpy.init()
        self._node=Node("quest_leap_only_backend")
        self._executor=SingleThreadedExecutor(); self._executor.add_node(self._node)
        self._thread=threading.Thread(target=self._executor.spin,daemon=True); self._thread.start()
        self._lock=threading.Lock(); self._hand=None
        self._actual=values.copy(); self._target=values.copy(); self._period=float(control_period)
        self._publisher=self._node.create_publisher(JointState,"/dual_crx/internal/right_leap/command",1)
        self._node.create_subscription(LeapState,"/right_leap/state",self._hand_cb,10)
        self._enable_client=self._node.create_client(SetBool,"/right_leap/enable")
        try:
            self._wait_feedback(); self._call_enable(True); self._wait_hand_enabled(); self._seed_feedback()
        except BaseException:
            self.close(); raise
    @property
    def control_period(self): return self._period
    def _hand_cb(self,message):
        values=dict(zip(message.joints.name,message.joints.position))
        with self._lock:
            self._hand=message
            if message.healthy and all(n in values and np.isfinite(values[n]) for n in DUAL_CRX_NAMES[6:]): self._actual[6:]=[values[n] for n in DUAL_CRX_NAMES[6:]]
    def _wait_feedback(self):
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            with self._lock:
                if self._hand is not None and self._hand.healthy: return
            time.sleep(.01)
        raise RuntimeError("timed out waiting for LEAP feedback")
    def _wait_hand_enabled(self):
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            with self._lock:
                if self._hand is not None and self._hand.enabled and self._hand.torque_enabled: return
            time.sleep(.01)
        raise RuntimeError("timed out waiting for LEAP enable")
    def _call_enable(self,enabled):
        if not self._enable_client.wait_for_service(timeout_sec=5): raise RuntimeError("/right_leap/enable unavailable")
        from std_srvs.srv import SetBool
        future=self._enable_client.call_async(SetBool.Request(data=enabled)); deadline=time.monotonic()+5
        while not future.done() and time.monotonic()<deadline: time.sleep(.01)
        if not future.done() or not future.result().success: raise RuntimeError("LEAP enable request failed")
    def _seed_feedback(self):
        with self._lock: self._target=self._actual.copy()
    def reset(self,qpos=None): self._seed_feedback()
    def get_joint_pos(self):
        with self._lock: return self._actual.copy()
    def get_target_joint_pos(self):
        with self._lock: return self._target.copy()
    def execute(self,qpos):
        values=np.asarray(qpos,dtype=float)
        if values.shape!=(22,) or not np.isfinite(values).all(): raise ValueError("leap_only command must be finite shape (22,)")
        msg=self._JointState(); msg.header.stamp=self._node.get_clock().now().to_msg(); msg.name=list(DUAL_CRX_NAMES[6:]); msg.position=[float(v) for v in values[6:]]; self._publisher.publish(msg)
        with self._lock: self._target=values.copy(); actual=self._actual.copy()
        return BackendStepResult(values,actual,{"leap_only":1.0})
    def close(self):
        try:
            if hasattr(self,"_enable_client"): self._call_enable(False)
        except Exception: pass
        if hasattr(self,"_executor"): self._executor.shutdown()
        if hasattr(self,"_thread"): self._thread.join(timeout=2)
        if hasattr(self,"_node"): self._node.destroy_node()
