"""Exercise the shared lifecycle with actual Sharpa solvers, without sensors."""

from dataclasses import replace
from types import SimpleNamespace as NS
import time

import numpy as np
import pytest

from retargeting_apps.sharpa_teleop import build_flow
from teleoperation.inputs.synthetic_hand import SyntheticBimanualInput
from teleoperation.types import BimanualSensorHandSample


def arguments(**overrides):
    values = dict(config=None, backend='preview', duration=0., command_hz=20.,
                  adb=None, serial=None, viewer=False, viewer_port=9219,
                  startup_timeout=3., crx_namespace='crx5ia', sharpa_namespace='sharpa')
    values.update(overrides)
    return NS(**values)


@pytest.mark.parametrize('with_arms', [False, True])
def test_real_solver_shapes_limits_fifth_finger_and_finite_input(with_arms):
    source = SyntheticBimanualInput(frames=4, max_age_s=10.)
    flow, _ = build_flow(arguments(), with_arms=with_arms, source=source)
    source.open()
    previous = flow.initial_qpos.copy()
    assert flow.robot_dofs == ((28, 28) if with_arms else (22, 22))
    for _ in range(4):
        result = flow.step(source.read())
        assert result is not None and np.isfinite(result.qpos).all()
        for i, r in enumerate((flow.pipeline.left_retargeter, flow.pipeline.right_retargeter)):
            assert r.human_fingertip_indices.tolist() == [4, 8, 12, 16, 20]
            joints = flow.robot_slices[i]
            qpos = result.qpos[joints]
            assert np.all(qpos >= r.optimizer.joint_limits[:, 0] - 1e-7)
            assert np.all(qpos <= r.optimizer.joint_limits[:, 1] + 1e-7)
            assert np.all(np.abs(qpos - previous[joints]) <= flow.command_limiters[i].max_delta + 1e-7)
            np.testing.assert_allclose(r.previous_qpos, qpos)
        previous = result.qpos.copy()
    assert flow.command_count == 4
    with pytest.raises(StopIteration):
        source.read()


def test_hand_only_mapper_ignores_global_wrist_motion():
    source = SyntheticBimanualInput()
    flow, _ = build_flow(arguments(), with_arms=False, source=source)
    sample = source.read()
    pipeline = flow.pipeline
    assert pipeline.initialize(sample, flow.initial_qpos[:22], flow.initial_qpos[22:])
    for mapper, hand in ((pipeline.left_mapper, sample.left), (pipeline.right_mapper, sample.right)):
        before = mapper.map(hand)
        pose = np.eye(4)
        pose[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        pose[:3, 3] = [10, -3, 8]
        after = mapper.map(replace(hand, wrist_pose_sensor=pose))
        np.testing.assert_array_equal(after.wrist_pose_world, before.wrist_pose_world)
        np.testing.assert_array_equal(after.keypoints_wrist, before.keypoints_wrist)


def test_combined_initial_wrist_alignment_preserves_measured_orientation():
    source = SyntheticBimanualInput()
    flow, _ = build_flow(arguments(), with_arms=True, source=source)
    sample = source.read()
    assert flow.pipeline.initialize(sample, flow.initial_qpos[:28], flow.initial_qpos[28:])
    for mapper, hand in ((flow.pipeline.left_mapper, sample.left), (flow.pipeline.right_mapper, sample.right)):
        first = mapper.map(hand)
        np.testing.assert_allclose(first.wrist_pose_world, mapper._robot_initial_wrist_pose, atol=1e-12)
        pose = hand.wrist_pose_sensor.copy()
        pose[0, 3] += .01
        moved = mapper.map(replace(hand, wrist_pose_sensor=pose))
        assert np.linalg.norm(moved.wrist_pose_world[:3, 3] - first.wrist_pose_world[:3, 3]) == pytest.approx(.01)


def test_measured_56_dim_seed_pause_and_recovery(monkeypatch):
    source = SyntheticBimanualInput(max_age_s=10.)
    flow, _ = build_flow(arguments(), with_arms=True, source=source)
    measured = flow.initial_qpos.copy()
    measured[[0, 28]] += .01
    calls = []
    backend = NS(get_joint_pos=lambda: measured.copy(), execute=lambda q: calls.append(q.copy()),
                 pause_tracking=lambda: None, resume_tracking=lambda: False)
    flow.backend_factory = lambda: backend
    assert flow.step(source.read()) is None
    flow.step(source.read())
    assert len(calls) == 1
    backend.assert_tracking = lambda: False
    assert flow.step(source.read()) is None
    assert flow.tracking_paused and len(calls) == 1
    bad = source.read()
    flow.step(BimanualSensorHandSample(replace(bad.left, keypoints_wrist=None), bad.right))
    assert flow.tracking_paused
    flow._recovery_started = time.monotonic() - 1
    assert flow.step(source.read()) is None
    assert flow.tracking_paused and len(calls) == 1
    backend.resume_tracking = lambda: True
    backend.assert_tracking = lambda: True
    flow._recovery_started = time.monotonic() - 1
    flow.step(source.read())
    assert not flow.tracking_paused and not flow.pipeline.initialized
    flow.step(source.read())
    assert len(calls) == 2
    assert np.max(np.abs(calls[-1] - measured)) <= .05 + 1e-7


def test_cli_help_never_opens_devices():
    import subprocess
    import sys
    for script in ('run_sharpa_joint_teleop.py', 'run_crx_sharpa_joint_teleop.py'):
        result = subprocess.run([sys.executable, f'scripts/{script}', '--help'], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        assert '--backend' in result.stdout and '--synthetic-frames' in result.stdout
