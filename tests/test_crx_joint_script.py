"""Headless script checks; explicitly opt in to isolated ROS mock integration."""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/run_crx_joint_teleop.py'
spec = importlib.util.spec_from_file_location('crx_joint_script', SCRIPT)
script = importlib.util.module_from_spec(spec)
spec.loader.exec_module(script)


@pytest.fixture
def connection():
    """Exercise message handling without importing any ROS packages."""
    link = script.JointConnection.__new__(script.JointConnection)
    link._actual = np.full(44, 0.123)
    link._target = link._actual.copy()
    link._lock = threading.RLock()
    link._stopped = link._paused = link._closed = False
    link._feedback_error = 'waiting for complete arm feedback'
    link._stamp_ns = 0
    link._received_at = link._advanced_at = 0.0
    link._spin_error = None
    link._feedback_timeout = 0.5
    link._context = NS(ok=lambda: True)
    link._thread = NS(is_alive=lambda: True)
    link._node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)))
    link.commands = []
    link._publisher = NS(publish=lambda msg: link.commands.append(msg.data))
    link._command_type = lambda **kwargs: NS(**kwargs)
    return link


def state(names=None, positions=None, sec=100, nanosec=0):
    return NS(name=list(script.ARM_NAMES) if names is None else names,
              position=list(range(12)) if positions is None else positions,
              header=NS(stamp=NS(sec=sec, nanosec=nanosec)))


def test_named_feedback_and_44_to_12_mapping(connection):
    link = connection
    with pytest.raises(RuntimeError, match='waiting'):
        link.execute(np.arange(44))
    link._receive_state(state(list(reversed(script.ARM_NAMES)), list(reversed(range(12)))))
    measured = link.get_joint_pos()
    np.testing.assert_equal(measured[script.ARM_INDICES], np.arange(12))
    np.testing.assert_equal(measured[6:22], 0.123)
    np.testing.assert_equal(measured[28:], 0.123)
    measured[:] = 999  # Returned arrays must not mutate the feedback cache.
    result = link.execute(np.arange(44))
    assert link.commands == [[*range(6), *range(22, 28)]]
    np.testing.assert_equal(result.actual_qpos[script.ARM_INDICES], np.arange(12))
    assert not result.diagnostics['right_hand_output_enabled']
    assert not result.diagnostics['left_hand_feedback_fresh']


@pytest.mark.parametrize('values', [np.zeros(12), np.zeros(43), np.zeros(45),
                                  np.full(44, np.nan), np.full(44, np.inf),
                                  np.full(44, -np.inf)])
def test_bad_commands_rejected(connection, values):
    connection._receive_state(state())
    with pytest.raises(ValueError, match='44 finite'):
        connection.execute(values)
    assert not connection.commands


@pytest.mark.parametrize('message', [
    state(names=list(script.ARM_NAMES)[:-1], positions=[0.] * 11),
    state(names=['left_J1'] * 12), state(positions=[0.] * 11),
    state(positions=[np.nan] * 12), state(sec=0), state(sec=99), state(sec=101),
])
def test_bad_feedback_blocks_output(connection, message):
    connection._receive_state(state())
    connection._receive_state(message)
    with pytest.raises(RuntimeError):
        connection.execute(np.zeros(44))
    assert not connection.commands


def test_cached_feedback_does_not_refresh_source_freshness(connection):
    link = connection
    link._receive_state(state())
    # ROS time is frozen but callbacks still arrive, just like cached bridge feedback.
    link._advanced_at = time.monotonic() - 1.0
    link._receive_state(state())
    with pytest.raises(RuntimeError, match='frozen'):
        link.execute(np.zeros(44))
    assert not link.commands


def test_feedback_loss_and_clock_regression(connection):
    link = connection
    link._receive_state(state())
    link._received_at -= 1.0
    with pytest.raises(RuntimeError, match='stale'):
        link.get_joint_pos()
    link._receive_state(state(sec=99, nanosec=990_000_000))
    with pytest.raises(RuntimeError, match='backwards'):
        link.get_joint_pos()


def test_reset_pause_resume_and_stop_never_command_home(connection):
    link = connection
    link._receive_state(state())
    link.reset(np.full(44, 2.0))
    np.testing.assert_equal(link.get_target_joint_pos()[script.ARM_INDICES], np.arange(12))
    link.pause_tracking()
    link.assert_tracking()  # Healthy feedback while paused is normal.
    with pytest.raises(RuntimeError, match='paused'):
        link.execute(np.zeros(44))
    link._receive_state(state(positions=[0.25] * 12))
    link.resume_tracking()
    np.testing.assert_equal(link.get_target_joint_pos()[script.ARM_INDICES], 0.25)
    assert not link.commands
    link.execute(np.zeros(44))
    link.request_stop('session expired')
    with pytest.raises(RuntimeError, match='stopped'):
        link.resume_tracking()
    with pytest.raises(RuntimeError, match='stopped'):
        link.execute(np.ones(44))
    assert len(link.commands) == 1


def test_cli_help_and_invalid_timing_do_not_import_ros_or_open_devices():
    command = (
        'import importlib.util, sys; '
        f'spec=importlib.util.spec_from_file_location("joint_script", {str(SCRIPT)!r}); '
        'module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); '
        'assert "rclpy" not in sys.modules; '
        'assert "teleoperation.inputs.quest3.online" not in sys.modules'
    )
    subprocess.run([sys.executable, '-c', command], check=True, cwd=ROOT)
    result = subprocess.run([sys.executable, str(SCRIPT), '--help'], capture_output=True, text=True)
    assert result.returncode == 0 and '--command-hz' in result.stdout
    for args in (['--command-hz', 'nan'], ['--command-hz', '0'], ['--duration', '-1']):
        result = subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True)
        assert result.returncode == 2
        assert 'must be finite' in result.stderr


def test_flow_composition_keeps_measured_seed_and_arm_filtering(monkeypatch):
    from teleoperation.bimanual import BimanualRetargetedFrame

    args = NS(command_hz=20.0, duration=1.0, serial='test-headset', viewer=True, viewer_port=9219)
    flow, config = script.build_flow(args)  # Actual configs/models, but no source.open() or ROS.
    assert config['viewer']['enabled'] and config['viewer']['port'] == 9219
    assert not config['viewer']['wait_for_client']
    assert flow.backend is None
    assert flow.period == 0.05 and flow.duration == 1.0
    assert flow.hand_output_filters is None
    assert len(flow.arm_output_filters) == 2
    assert all(f.mode_config.output.smooth_output_qpos for f in flow.arm_output_filters)
    assert all(f.mode_config.output.smoothing_alpha == 0.3 for f in flow.arm_output_filters)
    measured = np.full(44, 0.12)
    commands, seeds = [], []
    link = NS(get_joint_pos=lambda: measured.copy(), execute=lambda q: commands.append(q.copy()))
    monkeypatch.setattr(script, 'JointConnection', lambda initial, period: link)
    pipeline = NS(initialized=False)
    pipeline.left_retargeter = NS(reset=lambda q: seeds.append(q.copy()), previous_qpos=None)
    pipeline.right_retargeter = NS(reset=lambda q: seeds.append(q.copy()), previous_qpos=None)

    def initialize(sample, left, right):
        pipeline.initialized = True
        return True

    pipeline.initialize = initialize
    pipeline.step = lambda sample: BimanualRetargetedFrame(
        np.ones(22), np.full(22, -1.), NS(), NS())
    flow.pipeline = pipeline
    flow.source = NS(max_age_s=10.0)
    flow.duration = 0.0  # This test does not start a duration timer.
    sample = lambda index: NS(complete=True, source_index=index,
                             left=NS(source_index=index), right=NS(source_index=index))
    assert flow.step(sample(1)) is None  # Backend startup discards the pre-startup sample.
    assert not commands
    flow.step(sample(2))
    np.testing.assert_allclose(seeds, np.full((2, 22), 0.12))
    np.testing.assert_allclose(commands[0][:6], 0.3 * 1. + 0.7 * 0.12)
    np.testing.assert_allclose(commands[0][22:28], 0.3 * -1. + 0.7 * 0.12)
    flow.step(sample(2))
    assert len(commands) == 1


@pytest.mark.parametrize('failure', [None, KeyboardInterrupt(), RuntimeError('source failed')])
def test_script_starts_viewer_and_closes_it_on_exit(monkeypatch, failure):
    from retargeting_apps.visualization.execution import manager

    events = []

    def run():
        events.append('run')
        if failure is not None:
            raise failure

    flow = NS(run=run)

    def build(args):
        assert args.viewer is True
        return flow, {'viewer': {'enabled': args.viewer, 'port': args.viewer_port}}

    def create(config, actual_flow):
        assert actual_flow is flow and config['viewer']['port'] == 9220
        events.append('viewer')
        return NS(close=lambda: events.append('close'))

    monkeypatch.setattr(script, 'build_flow', build)
    monkeypatch.setattr(manager, 'create_optional_execution_visualizer', create)
    assert script.main(['--viewer-port', '9220']) == (1 if isinstance(failure, RuntimeError) else 0)
    assert events == ['viewer', 'run', 'close']


def test_script_no_viewer_never_constructs_visualizer(monkeypatch):
    from retargeting_apps.visualization.execution import manager

    ran = []

    def build(args):
        assert args.viewer is False
        return NS(run=lambda: ran.append(True)), {}

    monkeypatch.setattr(script, 'build_flow', build)
    monkeypatch.setattr(manager, 'create_optional_execution_visualizer',
                        lambda *args: pytest.fail('headless script must not create a viewer'))
    assert script.main(['--no-viewer']) == 0
    assert ran == [True]


@pytest.mark.skipif(os.environ.get('CRX_JOINT_ROS_TEST') != '1',
                    reason='Set CRX_JOINT_ROS_TEST=1 after sourcing ROS to run isolated mock')
def test_ros_mock_round_trip_and_cleanup(monkeypatch, tmp_path):
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray

    monkeypatch.setenv('ROS_DOMAIN_ID', '185')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    # Failure to obtain feedback must clean up its own ROS executor and context.
    with pytest.raises(RuntimeError, match='startup timeout'):
        script.JointConnection(np.zeros(44), 0.05, startup_timeout=0.2)
    assert not any(t.name == 'crx_joint_feedback' for t in threading.enumerate())
    context = Context()
    rclpy.init(args=[], context=context)
    peer = Node('joint_script_test_monitor', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(peer)
    commands = []
    peer.create_subscription(Float64MultiArray, '/teleop/joint_command',
                             lambda msg: commands.append(list(msg.data)), 10)

    def wait_for(predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), (tmp_path / 'launch.log').read_text()

    process = link = None
    try:
        with (tmp_path / 'launch.log').open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'dual_crx_control', 'teleop_joint.launch.py',
                 'mock:=true', 'rviz:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            for side in ('left', 'right'):
                client = peer.create_client(ListControllers, f'/{side}/controller_manager/list_controllers')
                wait_for(client.service_is_ready)
                deadline = time.monotonic() + 10
                while True:
                    future = client.call_async(ListControllers.Request())
                    wait_for(future.done)
                    states = {c.name: c.state for c in future.result().controller}
                    if states == {'joint_state_broadcaster': 'active', 'forward_position_controller': 'active'}:
                        break
                    assert time.monotonic() < deadline, states
                    time.sleep(0.05)  # Do not flood the service while spawners activate.
            initial = np.full(44, 0.123)
            link = script.JointConnection(initial, 0.05)
            wait_for(lambda: link._publisher.get_subscription_count() >= 2)
            assert not commands
            mock_pose = [0., 0., 0., 0., -np.pi / 2, 0.,
                         -np.pi / 2, 0., np.pi, 0., np.pi / 2, 0.]
            np.testing.assert_allclose(link.get_joint_pos()[script.ARM_INDICES], mock_pose)
            np.testing.assert_allclose(link.get_target_joint_pos()[script.ARM_INDICES], mock_pose)
            q = link.get_joint_pos()
            q[0] += 0.00872665
            q[22] -= 0.00872665
            link.execute(q)
            wait_for(lambda: len(commands) == 1 and np.allclose(
                link.get_joint_pos()[script.ARM_INDICES], q[script.ARM_INDICES]))
            np.testing.assert_allclose(commands[0], q[script.ARM_INDICES])
            link.pause_tracking()
            time.sleep(0.3)
            link.assert_tracking()
            link.resume_tracking()
            np.testing.assert_allclose(link.get_joint_pos()[6:22], initial[6:22])
            link.request_stop('test complete')
            with pytest.raises(RuntimeError, match='stopped'):
                link.execute(q)
            link.close()
            link.close()
            assert context.ok()  # Closing the script must not shut down the monitor.
            assert not link._thread.is_alive()
            executor.spin_once(timeout_sec=0.1)
            assert len(commands) == 1
    finally:
        if link is not None:
            link.close()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        executor.shutdown()
        peer.destroy_node()
        context.try_shutdown()
    assert process.returncode == 0, (tmp_path / 'launch.log').read_text()
