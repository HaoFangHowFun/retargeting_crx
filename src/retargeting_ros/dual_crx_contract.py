"""Compatibility exports for the canonical ROS-free dual-CRX contract."""

from teleoperation.backends import dual_crx_contract as _contract
from teleoperation.backends.dual_crx_contract import *  # noqa: F403

__all__ = _contract.__all__
