"""Mapping between the CRX retargeting profile and dual_crx_ros2 commands.

This module is deliberately ROS-free so the contract can be tested from the
retargeting environment.  The ROS adapter can use :func:`to_dual_crx_command`
when constructing ``TeleopCommand`` messages.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

CRX_PROFILE_ARM_NAMES = tuple(f"J{i}" for i in range(1, 7))
CRX_PROFILE_HAND_NAMES = tuple(f"joint_{i}" for i in range(16))
CRX_PROFILE_NAMES = CRX_PROFILE_ARM_NAMES + CRX_PROFILE_HAND_NAMES

DUAL_CRX_ARM_NAMES = tuple(f"right_J{i}" for i in range(1, 7))
DUAL_CRX_HAND_NAMES = tuple(f"right_leap_joint_{i}" for i in range(16))
DUAL_CRX_NAMES = DUAL_CRX_ARM_NAMES + DUAL_CRX_HAND_NAMES


def to_dual_crx_names(names: Sequence[str] | None = None) -> tuple[str, ...]:
    """Return canonical dual-crx names for a CRX+LEAP qpos vector.

    The profile order is fixed by ``configs/robots/crx5ia_leap_paxini.yaml``.
    When names are supplied, they must be exactly that order; accepting an
    arbitrary vector order would make a retargeting/robot mismatch silent.
    """
    if names is not None and tuple(names) != CRX_PROFILE_NAMES:
        raise ValueError(
            "CRX+LEAP retargeting names must be exactly "
            f"{list(CRX_PROFILE_NAMES)!r}, got {list(names)!r}."
        )
    return DUAL_CRX_NAMES


def to_dual_crx_positions(qpos: Sequence[float] | np.ndarray) -> tuple[float, ...]:
    """Validate and detach one 22-element retargeting position vector."""
    values = np.asarray(qpos, dtype=float)
    if values.shape != (22,):
        raise ValueError(f"CRX+LEAP qpos must have shape (22,), got {values.shape}.")
    if not np.isfinite(values).all():
        raise ValueError("CRX+LEAP qpos must contain only finite values.")
    return tuple(float(value) for value in values)


def to_dual_crx_command(
    qpos: Sequence[float] | np.ndarray,
    *,
    names: Sequence[str] | None = None,
) -> tuple[tuple[str, ...], tuple[float, ...]]:
    """Return the validated names and positions for ``TeleopCommand``."""
    return to_dual_crx_names(names), to_dual_crx_positions(qpos)


__all__ = [
    "CRX_PROFILE_ARM_NAMES",
    "CRX_PROFILE_HAND_NAMES",
    "CRX_PROFILE_NAMES",
    "DUAL_CRX_ARM_NAMES",
    "DUAL_CRX_HAND_NAMES",
    "DUAL_CRX_NAMES",
    "to_dual_crx_command",
    "to_dual_crx_names",
    "to_dual_crx_positions",
]
