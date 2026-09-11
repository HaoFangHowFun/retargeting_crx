"""Hardware-free flow and feedback routing regression tests."""
import threading
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest
from teleoperation.backends.bimanual_crx import BimanualCrxRobotBackend
from teleoperation.backends.dual_crx_contract import BIMANUAL_CRX_NAMES
from teleoperation.bimanual_execution import BimanualExecutionFlow


def test_feedback_routes_both_arms_without_absent_hand_dependency():
    backend = BimanualCrxRobotBackend.__new__(BimanualCrxRobotBackend)
    backend._state_lock = threading.Lock()
    backend._actual = np.zeros(44)
    backend._target = np.ones(44)
    backend._feedback_at = {"left_arm": 0., "right_arm": 0., "right_hand": time.monotonic()}
    backend._state_cb(NS(fresh=True,
        left_joints=NS(name=list(BIMANUAL_CRX_NAMES[:6]), position=[.1]*6),
        right_joints=NS(name=list(BIMANUAL_CRX_NAMES[22:28]), position=[-.2]*6)))
    backend._seed_from_feedback()
    np.testing.assert_allclose(backend._target[:6], .1)
    np.testing.assert_allclose(backend._target[22:28], -.2)
    assert "left_hand" not in backend._feedback_at
    assert np.all(backend._target[6:22] == 0)  # virtual placeholder only
    backend._feedback_at['left_arm'] = time.monotonic() - 1.
    with pytest.raises(RuntimeError, match="fresh complete"):
        backend._seed_from_feedback()


def test_disabled_hand_has_no_service_or_subscription():
    b = BimanualCrxRobotBackend.__new__(BimanualCrxRobotBackend)
    b.hand_enabled = {"left": False, "right": True}
    b._clients = {}
    subscriptions, clients = [], []
    b._node = NS(create_subscription=lambda typ, topic, cb, qos: subscriptions.append(topic),
                 create_client=lambda typ, topic: clients.append(topic))
    # Message imports are supplied by tiny stand-ins; no ROS runtime needed.
    import sys
    from unittest.mock import patch
    with patch.dict(sys.modules, {"dual_crx_interfaces.msg": NS(LeapState=object),
                                  "dual_crx_interfaces.srv": NS(SetTeleop=object),
                                  "std_srvs.srv": NS(SetBool=object)}):
        b._create_hand_channels()
    assert subscriptions == ["/right_leap/state"]
    assert clients == ["/dual_crx/teleop/bimanual/pause_tracking", "/right_leap/enable"]
    assert set(b._feedback_at) == {"left_arm", "right_arm", "right_hand"}


class Pipeline:
    initialized = False
    def __init__(self):
        self.seeds = []
        self.left_retargeter = self.right_retargeter = NS(reset=lambda q: self.seeds.append(q.copy()))
    def initialize(self, sample, left, right):
        self.initialized = True
        return True
    def reset(self):
        self.initialized = False
    def step(self, sample):
        return NS(qpos=np.arange(44, dtype=float))


def sample(index=1, complete=True, right_index=None):
    return NS(complete=complete, source_index=index,
              left=NS(source_index=index), right=NS(source_index=index if right_index is None else right_index))


def test_flow_uses_measured_seed_and_never_republishes_missing_or_repeated_frames():
    commands = []
    pipeline = Pipeline()
    backend = NS(get_joint_pos=lambda: np.full(44, .12), execute=lambda q: commands.append(q),
                 pause_tracking=lambda: None)
    flow = BimanualExecutionFlow(source=NS(max_age_s=.15), pipeline=pipeline,
                                initial_qpos=np.zeros(44), backend_factory=lambda: backend)
    assert flow.step(sample()) is None  # backend setup first; discard pre-startup sample
    assert flow.step(sample(2)) is not None
    assert len(commands) == 1 and commands[0].shape == (44,)
    np.testing.assert_allclose(pipeline.seeds, .12)
    assert flow.step(sample(2)) is None
    assert flow.step(sample(3, False)) is None
    assert len(commands) == 1
    with pytest.raises(ValueError, match="same Quest frame"):
        flow.step(sample(4, right_index=5))


def test_cleanup_runs_even_when_source_open_fails():
    closed = []
    def fail():
        raise RuntimeError("no quest")
    flow = BimanualExecutionFlow(source=NS(open=fail, close=lambda: closed.append(True)),
                                pipeline=Pipeline(), initial_qpos=np.zeros(44))
    with pytest.raises(RuntimeError, match="no quest"):
        flow.run()
    assert closed == [True]


@pytest.mark.parametrize('duration', [-1, float('nan'), float('inf')])
def test_duration_rejects_invalid_values(duration):
    with pytest.raises(ValueError, match='duration'):
        BimanualExecutionFlow(source=NS(), pipeline=Pipeline(), initial_qpos=np.zeros(44), duration=duration)


def test_duration_starts_after_calibration_and_stops_during_slow_solver():
    stopped = threading.Event()
    commands = []
    pipeline = Pipeline()
    backend = NS(get_joint_pos=lambda: np.zeros(44), execute=lambda q: commands.append(q),
                 request_stop=lambda reason: stopped.set())
    flow = BimanualExecutionFlow(source=NS(max_age_s=1.), pipeline=pipeline,
                                initial_qpos=np.zeros(44), backend_factory=lambda: backend, duration=.03)
    flow.step(sample(1, complete=False))
    assert flow._duration_timer is None
    flow.step(sample(2))  # backend created; calibration waits for new sample
    assert flow._duration_timer is None
    def slow_step(sample):
        assert stopped.wait(1.), 'stop request must not wait for solver completion'
        return NS(qpos=np.zeros(44))
    pipeline.step = slow_step
    try:
        assert flow.step(sample(3)) is None
        assert flow._duration_expired.is_set()
        assert not commands
        assert flow.step(sample(4)) is None
    finally:
        if flow._duration_timer is not None:
            flow._duration_timer.cancel()
            flow._duration_timer.join()


def test_timed_stop_latches_backend_even_if_stop_service_unavailable(monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, 'dual_crx_interfaces.srv', NS(SoftwareStop=NS(Request=lambda **kw: NS(**kw))))
    b = BimanualCrxRobotBackend.__new__(BimanualCrxRobotBackend)
    b._output_lock = threading.Lock()
    b._output_stopped = False
    b._startup_timer = None
    b._clients = {'stop': NS(service_is_ready=lambda: False)}
    errors = []
    b._node = NS(get_logger=lambda: NS(error=errors.append))
    b.request_stop('duration reached')
    assert b._output_stopped
    assert errors
    with pytest.raises(RuntimeError, match='output has been stopped'):
        b.execute(np.zeros(44))


def test_preview_duration_finishes_and_closes_source():
    closed = []
    source = NS(open=lambda: None, close=lambda: closed.append(True),
                read=lambda: sample(), max_age_s=1., stats=NS(accepted=1))
    flow = BimanualExecutionFlow(source=source, pipeline=Pipeline(), initial_qpos=np.zeros(44), duration=.03)
    flow.run()
    assert flow._duration_expired.is_set()
    assert closed == [True]
    assert not flow._duration_timer.is_alive()


@pytest.mark.parametrize("timed_stop", [False, True])
def test_session_cleanup_disables_requested_hand_torque(monkeypatch, timed_stop):
    import sys
    monkeypatch.setitem(sys.modules, 'std_srvs.srv', NS(SetBool=NS(Request=lambda **kw: NS(**kw))))
    b = BimanualCrxRobotBackend.__new__(BimanualCrxRobotBackend)
    b._output_stopped = timed_stop
    b._requested_hands = {'right'}
    actions = b._hand_cleanup_actions()
    assert [(key, req.data) for key, req in actions] == [('right_hand', False)]


def test_keyboard_interrupt_closes_backend_and_quest():
    closed = []
    def interrupt():
        raise KeyboardInterrupt
    source = NS(open=lambda: None, read=interrupt, close=lambda: closed.append('quest'))
    flow = BimanualExecutionFlow(source=source, pipeline=NS(), initial_qpos=np.zeros(44))
    flow.backend = NS(close=lambda: closed.append('backend'))
    with pytest.raises(KeyboardInterrupt):
        flow.run()
    assert closed == ['backend', 'quest']


def test_startup_waits_for_delayed_complete_arm_feedback(monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, 'std_srvs.srv', NS(SetBool=object))
    b = BimanualCrxRobotBackend.__new__(BimanualCrxRobotBackend)
    b.hand_enabled = {'left': False, 'right': False}
    b._state_lock = threading.Lock()
    b._feedback_at = {'left_arm': 0., 'right_arm': 0.}
    def feedback():
        with b._state_lock:
            b._feedback_at = {'left_arm': time.monotonic(), 'right_arm': time.monotonic()}
    arrival = threading.Timer(.02, feedback)
    arrival.start()
    try:
        b._enable_hands()
        assert all(b._feedback_at.values())
    finally:
        arrival.cancel()
        arrival.join()


def test_tracking_loss_requires_stable_new_frames_then_recalibrates(monkeypatch):
    now = [10.]
    monkeypatch.setattr('teleoperation.bimanual_execution.time.monotonic', lambda: now[0])
    actions = []
    measured = np.full(44, .12)
    backend = NS(get_joint_pos=lambda: measured.copy(), execute=lambda q: actions.append('execute'),
                 pause_tracking=lambda: actions.append('pause'), resume_tracking=lambda: actions.append('resume'))
    pipeline = Pipeline()
    flow = BimanualExecutionFlow(source=NS(max_age_s=.15), pipeline=pipeline,
        initial_qpos=np.zeros(44), backend_factory=lambda: backend)
    flow.step(sample(1)); flow.step(sample(2))
    initial_start = flow.started_at
    flow.step(sample(3, False))
    now[0] += 5
    flow.step(sample(4, False))
    assert actions == ['execute', 'pause']
    flow.step(sample(5))
    now[0] += .2
    flow.step(sample(6, False))  # unstable visibility restarts recovery interval
    flow.step(sample(7))
    now[0] += .2
    flow.step(sample(8))
    assert 'resume' not in actions
    now[0] += .11
    measured[:] = .3
    flow.step(sample(9))
    assert actions[-1] == 'resume'
    assert not pipeline.initialized
    flow.step(sample(10))
    assert actions[-1] == 'execute'
    np.testing.assert_allclose(pipeline.seeds[-2:], .3)
    assert flow.started_at == initial_start


def test_tracking_pause_does_not_hide_gateway_fault():
    def fault():
        raise RuntimeError('Servo warning')
    flow = BimanualExecutionFlow(source=NS(), pipeline=Pipeline(), initial_qpos=np.zeros(44))
    flow.started_at = 1.
    flow.backend = NS(pause_tracking=fault)
    with pytest.raises(RuntimeError, match='Servo warning'):
        flow.step(sample(1, False))
    assert not flow.tracking_paused


def test_duration_expiry_while_tracking_paused_never_resumes():
    actions = []
    flow = BimanualExecutionFlow(source=NS(), pipeline=Pipeline(), initial_qpos=np.zeros(44))
    flow.started_at = 1.
    flow.backend = NS(pause_tracking=lambda: actions.append('pause'),
                      request_stop=lambda reason: actions.append('stop'))
    flow.step(sample(1, False))
    flow._expire_duration()
    assert flow.step(sample(2)) is None
    assert actions == ['pause', 'stop']
