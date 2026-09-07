"""Integrated Meta Quest 3 hand tracking input."""

from teleoperation.inputs.quest3.common import MANO_JOINT_NAMES, decode_quest3_sample
from teleoperation.inputs.quest3.model import (
    HAND_SIDES,
    JOINT_NAMES,
    HandFrame,
    HandSample,
    HeadPose,
    JointPose,
)
from teleoperation.inputs.quest3.online import Quest3OnlineInput

__all__ = [
    "HAND_SIDES",
    "JOINT_NAMES",
    "MANO_JOINT_NAMES",
    "HandFrame",
    "HandSample",
    "HeadPose",
    "JointPose",
    "Quest3OnlineInput",
    "Quest3Receiver",
    "Quest3UsbSession",
    "QuestUsbBridge",
    "QuestUsbError",
    "ReceiverStats",
    "decode_quest3_sample",
    "find_adb",
]


def __getattr__(name: str):
    """Load aiohttp/ADB-backed transport classes only when requested."""
    if name in {"Quest3Receiver", "ReceiverStats"}:
        from teleoperation.inputs.quest3.receiver import Quest3Receiver, ReceiverStats

        return {"Quest3Receiver": Quest3Receiver, "ReceiverStats": ReceiverStats}[name]
    if name == "Quest3UsbSession":
        from teleoperation.inputs.quest3.session import Quest3UsbSession

        return Quest3UsbSession
    if name in {"QuestUsbBridge", "QuestUsbError", "find_adb"}:
        from teleoperation.inputs.quest3.usb import QuestUsbBridge, QuestUsbError, find_adb

        return {
            "QuestUsbBridge": QuestUsbBridge,
            "QuestUsbError": QuestUsbError,
            "find_adb": find_adb,
        }[name]
    raise AttributeError(name)
