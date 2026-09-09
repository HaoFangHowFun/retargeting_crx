"""Hardware-free checks for physical startup target selection."""
import threading
import time
from types import SimpleNamespace
import numpy as np
import pytest
from teleoperation.backends.dual_crx import DualCrxRobotBackend
from retargeting_ros.dual_crx_contract import DUAL_CRX_NAMES


def backend():
    b = DualCrxRobotBackend.__new__(DualCrxRobotBackend)
    b._state_lock = threading.Lock()
    b._actual = np.zeros(22)
    b._target = np.ones(22) * 2
    b._feedback_at = {"arm": 0., "hand": 0.}
    b._node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
    return b


def hand_message(healthy=True, stamp=10, names=None):
    return SimpleNamespace(healthy=healthy, joints=SimpleNamespace(
        name=list(DUAL_CRX_NAMES[6:]) if names is None else names,
        position=[.2] * 16, header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0))))


def test_startup_uses_actual_pose_instead_of_config_home():
    b = backend()
    b._state_cb(SimpleNamespace(fresh=True, right_joints=SimpleNamespace(name=list(DUAL_CRX_NAMES[:6]), position=[.1] * 6)))
    b._hand_state_cb(hand_message())
    b._seed_from_feedback()
    assert b.get_target_joint_pos() == pytest.approx([.1] * 6 + [.2] * 16)
    b._actual[:] = 3
    assert b.get_target_joint_pos()[0] == pytest.approx(.1)


@pytest.mark.parametrize("message", [hand_message(healthy=False), hand_message(stamp=9), hand_message(names=['wrong'] * 16)])
def test_unhealthy_stale_or_incomplete_hand_cannot_seed(message):
    b = backend()
    b._feedback_at['arm'] = time.monotonic()
    b._hand_state_cb(message)
    with pytest.raises(RuntimeError, match='fresh complete'):
        b._seed_from_feedback()


def test_old_arm_feedback_cannot_seed():
    b = backend()
    b._feedback_at = {'arm': time.monotonic() - 1, 'hand': time.monotonic()}
    with pytest.raises(RuntimeError, match='fresh complete'):
        b._seed_from_feedback()


def test_start_seeds_measured_pose_before_enabling_stream():
    b = backend()
    b._state = object()
    b._hand_state = SimpleNamespace(enabled=True)
    b._feedback_at = {'arm': time.monotonic(), 'hand': time.monotonic()}
    b._actual[:] = .4
    b._period = .05
    b._request = lambda key: key
    calls = []
    def call(key, request):
        calls.append(key)
        if key == 'teleop':
            assert b._target == pytest.approx([.4] * 22)
    b._call = call
    b._node.create_timer = lambda period, callback: object()
    b._start()
    assert calls == ['acquire', 'hand', 'teleop']
    assert b._started


def test_partial_startup_cleanup_attempts_all_actions(monkeypatch):
    import sys
    request_type = SimpleNamespace(Request=lambda **kwargs: SimpleNamespace(**kwargs))
    monkeypatch.setitem(sys.modules, 'dual_crx_interfaces.srv', SimpleNamespace(SoftwareStop=request_type, ReleaseControl=request_type))
    monkeypatch.setitem(sys.modules, 'std_srvs.srv', SimpleNamespace(SetBool=request_type))
    b = backend()
    b._startup_timer = None
    b._started = False
    b._acquired = b._hand_requested = True
    b._client_id = 'test'
    b._scope = 2
    calls = []
    def call(key, request):
        calls.append(key)
        if key == 'stop': raise RuntimeError('stop service unavailable')
    b._call = call
    b._node.get_logger = lambda: SimpleNamespace(error=lambda message: None)
    b._node.destroy_node = lambda: calls.append('destroy')
    b._executor = SimpleNamespace(shutdown=lambda: calls.append('shutdown'))
    b._thread = SimpleNamespace(join=lambda timeout: calls.append('join'))
    b.close()
    assert calls == ['stop', 'hand', 'release', 'shutdown', 'join', 'destroy']
    assert not b._acquired and not b._hand_requested
