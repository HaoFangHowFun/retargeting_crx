from __future__ import annotations

import sys
import types

import numpy as np
import pytest

from teleoperation.config import DetectionSourceConfig
from teleoperation.inputs.quest3 import (
    JOINT_NAMES,
    MANO_JOINT_NAMES,
    HandFrame,
    HandSample,
    JointPose,
    Quest3OnlineInput,
    decode_quest3_sample,
)
from teleoperation.inputs.quest3.common import OPERATOR2MANO_LEFT_QUEST
from teleoperation.inputs.quest3.common import OPERATOR2MANO_RIGHT_QUEST
from teleoperation.observation_mapping import RelativeWristMapper


def _frame(*, sequence: int = 7, right_tracked: bool = True) -> HandFrame:
    wrist_origin = np.array([0.4, 1.2, -0.7])
    joints = {
        name: JointPose(
            position_m=tuple(
                wrist_origin
                + np.array([0.01 * index, 0.002 * index, -0.003 * index])
            ),
            rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            radius_m=0.008,
        )
        for index, name in enumerate(JOINT_NAMES)
    }
    return HandFrame(
        sequence=sequence,
        source_sequence=3,
        sender_timestamp_ns=123,
        received_monotonic_ns=5_000_000_000,
        reference_space="local",
        hands={
            "left": HandSample(side="left", tracked=False, joints={}),
            "right": HandSample(
                side="right",
                tracked=right_tracked,
                joints=joints if right_tracked else {},
            ),
        },
    )


def test_decode_quest3_sample_maps_webxr_25_joints_to_mano_21() -> None:
    frame = _frame()

    sample = decode_quest3_sample(frame, hand_side="right")

    assert sample.has_hand
    assert sample.keypoints_wrist.shape == (21, 3)
    assert sample.wrist_pose_sensor.shape == (4, 4)
    assert sample.source_index == 7
    assert sample.timestamp == 5.0
    expected_rotation = np.array(
        [
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ]
    )
    np.testing.assert_allclose(sample.wrist_pose_sensor[:3, :3], expected_rotation)
    wrist_position = np.asarray(frame.hand("right").joints["wrist"].position_m)
    selected_positions = np.asarray(
        [frame.hand("right").joints[name].position_m for name in MANO_JOINT_NAMES]
    )
    np.testing.assert_allclose(
        sample.keypoints_wrist,
        (selected_positions - wrist_position) @ expected_rotation,
    )
    np.testing.assert_allclose(sample.keypoints_wrist[0], np.zeros(3))
    np.testing.assert_allclose(
        sample.keypoints_wrist @ sample.wrist_pose_sensor[:3, :3].T
        + sample.wrist_pose_sensor[:3, 3],
        selected_positions,
        atol=1e-12,
    )
    assert not sample.keypoints_wrist.flags.writeable
    assert not sample.wrist_pose_sensor.flags.writeable


def test_quest_left_basis_is_named_and_matches_selected_calibration() -> None:
    np.testing.assert_array_equal(OPERATOR2MANO_LEFT_QUEST, OPERATOR2MANO_RIGHT_QUEST)


def test_decode_quest3_sample_preserves_missing_hand_as_a_sample() -> None:
    frame = _frame(right_tracked=False)

    sample = decode_quest3_sample(frame)

    assert not sample.has_hand
    assert sample.raw is frame
    assert sample.source_index == frame.sequence
    assert sample.timestamp == 5.0
    assert decode_quest3_sample(None).raw is None


def test_quest3_online_input_owns_session_and_never_uses_none_as_end_of_stream(
    monkeypatch,
) -> None:
    frame = _frame(sequence=11)
    events: list[object] = []

    class FakeSession:
        def __init__(self, **kwargs: object) -> None:
            events.append(("init", kwargs))
            self.serial = "quest-serial"
            self.frames = [frame, None]

        def start(self):
            events.append("start")
            return self

        def poll(self):
            return self.frames.pop(0)

        def age_s(self) -> float:
            return 0.01

        def close(self) -> None:
            events.append("close")

    fake_session_module = types.ModuleType("teleoperation.inputs.quest3.session")
    fake_session_module.Quest3UsbSession = FakeSession
    monkeypatch.setitem(sys.modules, "teleoperation.inputs.quest3.session", fake_session_module)

    source = Quest3OnlineInput(hand_side="right", port=9123, max_age_s=0.1)
    source.open()
    valid = source.read()
    no_new_frame = source.read()
    source.close()

    assert source.device_serial is None
    assert valid.has_hand and valid.source_index == 11
    assert no_new_frame is not None and not no_new_frame.has_hand
    assert events[0] == (
        "init",
        {"port": 9123, "adb": None, "serial": None},
    )
    assert events[1:] == ["start", "close"]


def test_quest3_online_input_holds_stale_frame(monkeypatch) -> None:
    frame = _frame()

    class FakeSession:
        serial = "quest"

        def __init__(self, **_kwargs: object) -> None:
            pass

        def start(self):
            return self

        def poll(self):
            return frame

        def age_s(self) -> float:
            return 0.2

        def close(self) -> None:
            pass

    fake_session_module = types.ModuleType("teleoperation.inputs.quest3.session")
    fake_session_module.Quest3UsbSession = FakeSession
    monkeypatch.setitem(sys.modules, "teleoperation.inputs.quest3.session", fake_session_module)
    source = Quest3OnlineInput(max_age_s=0.1)
    source.open()

    sample = source.read()

    assert not sample.has_hand
    assert sample.raw is frame
    assert sample.source_index == frame.sequence


def test_relative_wrist_mapper_accepts_quest3_config() -> None:
    config = DetectionSourceConfig(
        name="quest3",
        input_device="quest3",
        rotation_euler_xyz_deg=(0.0, 0.0, 180.0),
        translation=(0.7, 0.2, -1.0),
        use_relative_wrist_alignment=True,
    )

    mapper = RelativeWristMapper(
        config,
        human_hand_scale=1.5,
        robot_adaptor=object(),
        robot_model=object(),
        wrist_frame_name="wrist",
    )

    assert mapper.config.input_device == "quest3"


@pytest.mark.parametrize(
    "teleoperation_mode",
    ["online_quest3_kinematic", "online_quest3_mujoco"],
)
def test_quest3_sample_retargets_to_panda_leap_23_dof(
    teleoperation_mode: str,
) -> None:
    from retargeting_apps.composition import build_execution_flow
    from retargeting_apps.main import compose_hydra_base_config
    from teleoperation.inputs.avp import AvpOfflineInput
    from teleoperation.types import ExecutionStatus

    archived = AvpOfflineInput("tests/fixtures/avp_short_replay.npz", start=0, end=0)
    archived.open()
    reference = archived.read()
    archived.close()
    assert reference is not None and reference.has_hand

    operator_to_mano = np.array(
        [
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]
    )
    wrist_origin = np.array([0.4, 1.2, -0.7])
    selected = {
        name: wrist_origin + point @ operator_to_mano.T
        for name, point in zip(MANO_JOINT_NAMES, reference.keypoints_wrist)
    }
    fallback = {
        "index_metacarpal": "index_proximal",
        "middle_metacarpal": "middle_proximal",
        "ring_metacarpal": "ring_proximal",
        "little_metacarpal": "little_proximal",
    }
    joints = {
        name: JointPose(
            position_m=tuple(selected[name if name in selected else fallback[name]]),
            rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            radius_m=0.008,
        )
        for name in JOINT_NAMES
    }
    frame = HandFrame(
        sequence=1,
        source_sequence=1,
        sender_timestamp_ns=1,
        received_monotonic_ns=1,
        reference_space="local",
        hands={
            "left": HandSample(side="left", tracked=False, joints={}),
            "right": HandSample(side="right", tracked=True, joints=joints),
        },
    )
    sample = decode_quest3_sample(frame)
    config = compose_hydra_base_config(
        ["app=teleop_exe", f"teleoperation_modes={teleoperation_mode}"]
    )
    flow = build_execution_flow(config)

    result = flow.step(sample)

    assert result.status is ExecutionStatus.EXECUTED
    assert result.retargeted_frame is not None
    assert result.retargeted_frame.retargeted_qpos.shape == (23,)
    assert np.isfinite(result.retargeted_frame.retargeted_qpos).all()
