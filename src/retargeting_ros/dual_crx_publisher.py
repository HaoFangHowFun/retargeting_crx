"""Thin ROS2 publisher for the dual_crx_ros2 teleoperation contract."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from retargeting_ros.dual_crx_contract import (
    CRX_PROFILE_NAMES,
    to_bimanual_crx_command,
    to_dual_crx_command,
)


class DualCrxPublisher:
    """Publish solved CRX+LEAP qpos vectors as ``TeleopCommand`` messages."""

    def __init__(self, node: Any, client_id: str = "retargeting_crx") -> None:
        if not isinstance(client_id, str) or not client_id.strip():
            raise ValueError("client_id must be a non-empty string.")
        try:
            from dual_crx_interfaces.msg import TeleopCommand
        except ImportError as exc:
            raise RuntimeError(
                "dual_crx_interfaces is unavailable; source the dual_crx_ros2 ROS environment."
            ) from exc
        self.node = node
        self.client_id = client_id
        self._message_type = TeleopCommand
        self.publisher = node.create_publisher(TeleopCommand, "/dual_crx/teleop/command", 1)

    def publish(self, qpos: Sequence[float]) -> Any:
        """Publish one validated current-time target in retargeting profile order."""
        names, positions = to_dual_crx_command(qpos, names=CRX_PROFILE_NAMES)
        message = self._message_type(client_id=self.client_id)
        message.target.header.stamp = self.node.get_clock().now().to_msg()
        message.target.name = list(names)
        message.target.position = list(positions)
        self.publisher.publish(message)
        return message


class BimanualDualCrxPublisher(DualCrxPublisher):
    """Publish one atomic 44-DOF left-then-right command."""

    def publish(self, qpos: Sequence[float]) -> Any:
        names, positions = to_bimanual_crx_command(qpos)
        message = self._message_type(client_id=self.client_id)
        message.target.header.stamp = self.node.get_clock().now().to_msg()
        message.target.name = list(names)
        message.target.position = list(positions)
        self.publisher.publish(message)
        return message


__all__ = ["BimanualDualCrxPublisher", "DualCrxPublisher"]
