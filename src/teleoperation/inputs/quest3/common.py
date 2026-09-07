"""Convert validated Quest WebXR hand frames into teleoperation samples."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from mr_utils.utils_mano import OPERATOR2MANO_LEFT, OPERATOR2MANO_RIGHT
from teleoperation.inputs.quest3.model import HandFrame, JOINT_NAMES
from teleoperation.types import SensorHandSample


# MediaPipe/MANO uses four joints for each non-thumb finger. WebXR additionally
# reports a metacarpal joint, so those four entries are intentionally omitted.
MANO_JOINT_NAMES = (
    "wrist",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "little_proximal",
    "little_intermediate",
    "little_distal",
    "little_tip",
)

if not set(MANO_JOINT_NAMES).issubset(JOINT_NAMES):
    raise RuntimeError("Quest-to-MANO joint mapping does not match the WebXR protocol.")


def _missing_sample(
    frame: HandFrame | None,
    *,
    source_index: int | None = None,
    timestamp: float | None = None,
) -> SensorHandSample:
    if frame is not None:
        source_index = frame.sequence if source_index is None else source_index
        timestamp = (
            frame.received_monotonic_ns / 1e9 if timestamp is None else timestamp
        )
    return SensorHandSample(
        keypoints_wrist=None,
        wrist_pose_sensor=None,
        raw=frame,
        source_index=source_index,
        timestamp=timestamp,
    )


def decode_quest3_sample(
    frame: HandFrame | None,
    *,
    hand_side: str = "right",
    source_index: int | None = None,
    timestamp: float | None = None,
) -> SensorHandSample:
    """Normalize one Quest frame into the existing MANO hand contract.

    Args:
        frame: Validated Quest WebXR frame, or None when no new frame arrived.
        hand_side: Hand selected for the single-hand Panda+LEAP pipeline.
        source_index: Optional acquisition index override.
        timestamp: Optional monotonic timestamp override in seconds.

    Returns:
        Wrist-local 21-keypoint sample, or a missing-hand sample.
    """
    if hand_side not in {"left", "right"}:
        raise ValueError("hand_side must be 'left' or 'right'.")
    if frame is None:
        return _missing_sample(
            None,
            source_index=source_index,
            timestamp=timestamp,
        )
    hand = frame.hands.get(hand_side)
    if hand is None or not hand.complete:
        return _missing_sample(
            frame,
            source_index=source_index,
            timestamp=timestamp,
        )

    wrist = hand.joints["wrist"]
    wrist_rotation = Rotation.from_quat(wrist.rotation_xyzw).as_matrix()
    operator_to_mano = (
        OPERATOR2MANO_RIGHT if hand_side == "right" else OPERATOR2MANO_LEFT
    )
    wrist_rotation_mano = wrist_rotation @ operator_to_mano
    wrist_position = np.asarray(wrist.position_m, dtype=float)
    positions_sensor = np.asarray(
        [hand.joints[name].position_m for name in MANO_JOINT_NAMES],
        dtype=float,
    )
    keypoints_wrist = (positions_sensor - wrist_position) @ wrist_rotation_mano
    wrist_pose_sensor = np.eye(4, dtype=float)
    wrist_pose_sensor[:3, :3] = wrist_rotation_mano
    wrist_pose_sensor[:3, 3] = wrist_position

    return SensorHandSample(
        keypoints_wrist=keypoints_wrist,
        wrist_pose_sensor=wrist_pose_sensor,
        raw=frame,
        source_index=frame.sequence if source_index is None else source_index,
        timestamp=(
            frame.received_monotonic_ns / 1e9
            if timestamp is None
            else timestamp
        ),
    )


__all__ = ["MANO_JOINT_NAMES", "decode_quest3_sample"]
