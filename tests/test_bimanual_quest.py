from __future__ import annotations

import numpy as np
import yaml

from retargeting.config import load_robot_config
from teleoperation.bimanual import BimanualRetargetingPipeline
from teleoperation.inputs.quest3 import HandFrame, HandSample, JOINT_NAMES, JointPose, Quest3BimanualOnlineInput


def _frame() -> HandFrame:
    def hand(side: str, offset: float) -> HandSample:
        joints = {
            name: JointPose(
                position_m=(offset + 0.001 * index, 1.0, 0.5),
                rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
                radius_m=0.008,
            )
            for index, name in enumerate(JOINT_NAMES)
        }
        return HandSample(side=side, tracked=True, joints=joints)

    return HandFrame(
        sequence=4,
        source_sequence=4,
        sender_timestamp_ns=10,
        received_monotonic_ns=2_000_000_000,
        reference_space="local",
        hands={"left": hand("left", -0.1), "right": hand("right", 0.1)},
    )


def test_bimanual_quest_input_decodes_one_synchronized_frame(monkeypatch):
    frame = _frame()

    class FakeSession:
        serial = "quest"

        def __init__(self, **kwargs):
            del kwargs

        def start(self):
            return self

        def poll(self):
            return frame

        def age_s(self):
            return 0.0

        def close(self):
            pass

    monkeypatch.setitem(__import__("sys").modules, "teleoperation.inputs.quest3.session", type("M", (), {"Quest3UsbSession": FakeSession}))
    source = Quest3BimanualOnlineInput()
    source.open()
    sample = source.read()
    source.close()
    assert sample.complete
    assert sample.left.source_index == sample.right.source_index == 4
    assert sample.left.has_hand and sample.right.has_hand


def test_bimanual_pipeline_keeps_left_and_right_commands_separate():
    class Mapper:
        def initialize(self, sample, qpos):
            return sample.has_hand and qpos.size == 1

        def reset(self):
            pass

        def map(self, sample):
            from retargeting.core.types import RetargetingHandObservation

            if not sample.has_hand:
                return None
            return RetargetingHandObservation(sample.keypoints_wrist, sample.wrist_pose_sensor)

    class Solver:
        def __init__(self, value):
            self.qpos_init = np.array([value], dtype=float)
            self.previous_qpos = self.qpos_init.copy()

        def reset(self, qpos):
            self.previous_qpos = np.asarray(qpos, dtype=float).copy()

        def solve(self, observation, previous_qpos):
            del observation, previous_qpos
            return type("Result", (), {"qpos": self.qpos_init.copy()})()

    from teleoperation.inputs.quest3.common import decode_quest3_sample
    from teleoperation.types import BimanualSensorHandSample

    frame = _frame()
    sample = BimanualSensorHandSample(
        left=decode_quest3_sample(frame, hand_side="left"),
        right=decode_quest3_sample(frame, hand_side="right"),
    )
    pipeline = BimanualRetargetingPipeline(
        left_mapper=Mapper(), right_mapper=Mapper(),
        left_retargeter=Solver(1.0), right_retargeter=Solver(2.0),
    )
    assert pipeline.initialize(sample, np.array([0.0]), np.array([0.0]))
    result = pipeline.step(sample)
    assert result is not None
    np.testing.assert_allclose(result.left_qpos, [1.0])
    np.testing.assert_allclose(result.right_qpos, [2.0])
    np.testing.assert_allclose(result.qpos, [1.0, 2.0])


def test_bimanual_leap_config_uses_left_dual_crx_home_and_22_dof_each():
    with open("configs/bimanual/crx5ia_coact_leap.yaml", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    left = load_robot_config(config["left"]["robot"])
    right = load_robot_config(config["right"]["robot"])

    assert len(left.actuated_joints) == len(right.actuated_joints) == 22
    np.testing.assert_allclose(left.initial_qpos[:6], [0.0, 0.0, 0.0, 0.0, -np.pi / 2, np.pi])
    np.testing.assert_allclose(right.initial_qpos[:6], [-np.pi / 2, 0.0, np.pi, 0.0, np.pi / 2, np.pi])
